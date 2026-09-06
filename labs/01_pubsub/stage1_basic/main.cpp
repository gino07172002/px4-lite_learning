/**
 * @file main.cpp
 * @brief Lab 01 - Stage 1 (基礎版): 泛型化單緩衝 Pub/Sub
 * 
 * [本階段目標與設計重點]:
 * 1. 泛型化 (Template): 支援任意飛控資料結構 (如 SensorImu)。
 * 2. 原子讀取防護 (Fix Torn Read):
 *    在 Topic 內部提供 read(T& out, uint64_t& gen)，在「同一個 Lock」保護下
 *    同時完成資料複製與世代獲取，徹底杜絕多執行緒下的資料世代脫節。
 * 3. 清楚的分工:
 *    - Topic<T>: 負責資料持有、計數器與線程安全保護。
 *    - Publisher<T>: 負責寫入發布。
 *    - Subscription<T>: 負責記錄個人的 last_generation，並提供 updated() 與 copy()。
 */

#include <iostream>
#include <mutex>
#include <cstdint>
#include <cassert>

// 飛控標準感測器訊息範例
struct SensorImu {
    uint64_t timestamp_us;
    float gyro[3];   // rad/s (Roll, Pitch, Yaw)
    float accel[3];  // m/s^2 (X, Y, Z)
};

template <typename T>
class Topic {
public:
    Topic() = default;

    // 發布新資料，累加世代
    void publish(const T& newData) {
        std::lock_guard<std::mutex> lock(_mutex);
        _data = newData;
        ++_generation;
    }

    // 原子化讀取：在同一個鎖內同時返回資料與世代號碼
    bool read(T& outData, uint64_t& outGen) const {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_generation == 0) {
            return false; // 尚未發布過任何資料
        }
        outData = _data;
        outGen = _generation;
        return true;
    }

    // 取得當前世代號（用於快速檢查）
    uint64_t getGeneration() const {
        std::lock_guard<std::mutex> lock(_mutex);
        return _generation;
    }

private:
    T _data{};
    uint64_t _generation{0};
    mutable std::mutex _mutex;
};

template <typename T>
class Subscription {
public:
    Subscription() = default;

    void subscribe(Topic<T>& topic) {
        _topic = &topic;
        _lastGeneration = 0;
    }

    // 檢查是否有新世代資料
    bool updated() const {
        if (_topic == nullptr) return false;
        return _topic->getGeneration() > _lastGeneration;
    }

    // 複製最新資料
    bool copy(T& out) {
        if (_topic == nullptr) return false;
        
        uint64_t currentGen = 0;
        if (_topic->read(out, currentGen)) {
            _lastGeneration = currentGen;
            return true;
        }
        return false;
    }

    uint64_t getLastGeneration() const { return _lastGeneration; }

private:
    Topic<T>* _topic{nullptr};
    uint64_t _lastGeneration{0};
};

template <typename T>
class Publisher {
public:
    Publisher() = default;

    void advertise(Topic<T>& topic) {
        _topic = &topic;
    }

    bool publish(const T& data) {
        if (_topic == nullptr) return false;
        _topic->publish(data);
        return true;
    }

private:
    Topic<T>* _topic{nullptr};
};

int main() {
    std::cout << "========================================================\n";
    std::cout << " [Lab 01 - Stage 1 基礎版]: Template 泛型化 Pub/Sub 測試 \n";
    std::cout << "========================================================\n";

    Topic<SensorImu> imuTopic;
    Publisher<SensorImu> imuPub;
    Subscription<SensorImu> imuSub1;
    Subscription<SensorImu> imuSub2;

    imuPub.advertise(imuTopic);
    imuSub1.subscribe(imuTopic);
    imuSub2.subscribe(imuTopic);

    // 驗證 1: 初期無資料
    assert(!imuSub1.updated());
    std::cout << "✓ 測試 1 通過: 初始狀態 updated() == false\n";

    // 驗證 2: 發布第一筆資料
    SensorImu raw1{1000000, {0.01f, -0.02f, 0.05f}, {0.0f, 0.0f, -9.81f}};
    imuPub.publish(raw1);

    assert(imuSub1.updated());
    assert(imuSub2.updated());
    std::cout << "✓ 測試 2 通過: 發布後兩個 Subscriber 皆檢測到 updated() == true\n";

    // 驗證 3: Sub1 讀取資料
    SensorImu read1{};
    bool ok1 = imuSub1.copy(read1);
    assert(ok1);
    assert(read1.timestamp_us == 1000000);
    assert(read1.gyro[0] == 0.01f);
    assert(!imuSub1.updated()); // 讀過之後應該不再是 updated
    assert(imuSub2.updated());  // 但 Sub2 還沒讀，依舊是 updated
    std::cout << "✓ 測試 3 通過: Sub1 複製成功且更新指標，Sub2 狀態保持獨立\n";

    // 驗證 4: 發布第二筆資料
    SensorImu raw2{1004000, {0.05f, -0.01f, 0.02f}, {0.1f, -0.1f, -9.79f}};
    imuPub.publish(raw2);

    SensorImu read2{};
    imuSub2.copy(read2); // Sub2 跳過第一筆直接讀取最新第二筆
    assert(read2.timestamp_us == 1004000);
    assert(read2.gyro[0] == 0.05f);
    std::cout << "✓ 測試 4 通過: Sub2 成功拿到最新第二筆資料 (世代 = " << imuSub2.getLastGeneration() << ")\n";

    std::cout << "\n>>> Stage 1 全部驗收標準達成！ <<<\n";
    return 0;
}
