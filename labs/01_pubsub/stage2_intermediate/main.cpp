/**
 * @file main.cpp
 * @brief Lab 01 - Stage 2 (核心進階版): 全域主題匯流排 (TopicBus) 與真實多執行緒驗證
 * 
 * [本階段目標與設計重點]:
 * 1. TopicBus (中介者 / 登錄註冊中心):
 *    - 發布者與訂閱者不再需要互相傳遞 Topic 指針。
 *    - 透過主題名稱 (字串，例如 "sensor_imu") 進行全域註冊與動態查找 (Singleton Registry)。
 * 2. 真實多執行緒模型 (Multithreading Simulation):
 *    - 模擬 PX4 架構中的實體執行緒環境：
 *      * FakeImuPublisher 執行緒: 模擬硬體中斷以 250Hz 發送 IMU 數據。
 *      * FastRateController 執行緒: 以 100Hz 訂閱 IMU 並執行內環控制運算。
 *      * SlowLogger 執行緒: 以 20Hz 異步採樣並記錄數據。
 * 3. 執行緒安全與生命週期健全性：保證各執行緒並發運行無 Data Race。
 */

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstdint>

// 標準 IMU 感測器訊息
struct SensorImu {
    uint64_t timestamp_us;
    float gyro[3];
    float accel[3];
};

// 抽象 Topic 基底介面
class TopicBase {
public:
    virtual ~TopicBase() = default;
};

// 具體型別 Topic
template <typename T>
class Topic : public TopicBase {
public:
    void publish(const T& data) {
        std::lock_guard<std::mutex> lock(_mutex);
        _data = data;
        ++_generation;
    }

    bool read(T& outData, uint64_t& outGen) const {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_generation == 0) return false;
        outData = _data;
        outGen = _generation;
        return true;
    }

    uint64_t getGeneration() const {
        std::lock_guard<std::mutex> lock(_mutex);
        return _generation;
    }

private:
    T _data{};
    uint64_t _generation{0};
    mutable std::mutex _mutex;
};

// 全域主題管理匯流排 (對標 PX4 uORB::Manager)
class TopicBus {
public:
    static TopicBus& instance() {
        static TopicBus s_instance;
        return s_instance;
    }

    template <typename T>
    std::shared_ptr<Topic<T>> getTopic(const std::string& name) {
        std::lock_guard<std::mutex> lock(_busMutex);
        auto it = _topics.find(name);
        if (it != _topics.end()) {
            return std::dynamic_pointer_cast<Topic<T>>(it->second);
        }
        // 若尚未建立則自動註冊
        auto newTopic = std::make_shared<Topic<T>>();
        _topics[name] = newTopic;
        return newTopic;
    }

private:
    TopicBus() = default;
    std::unordered_map<std::string, std::shared_ptr<TopicBase>> _topics;
    std::mutex _busMutex;
};

// 封裝後的 Publisher
template <typename T>
class Publisher {
public:
    explicit Publisher(const std::string& topicName) {
        _topic = TopicBus::instance().getTopic<T>(topicName);
    }

    void publish(const T& data) {
        if (_topic) _topic->publish(data);
    }

private:
    std::shared_ptr<Topic<T>> _topic;
};

// 封裝後的 Subscription
template <typename T>
class Subscription {
public:
    explicit Subscription(const std::string& topicName) {
        _topic = TopicBus::instance().getTopic<T>(topicName);
    }

    bool updated() const {
        if (!_topic) return false;
        return _topic->getGeneration() > _lastGeneration;
    }

    bool copy(T& out) {
        if (!_topic) return false;
        uint64_t currentGen = 0;
        if (_topic->read(out, currentGen)) {
            _lastGeneration = currentGen;
            return true;
        }
        return false;
    }

    uint64_t getLastGeneration() const { return _lastGeneration; }

private:
    std::shared_ptr<Topic<T>> _topic;
    uint64_t _lastGeneration{0};
};

int main() {
    std::cout << "===============================================================\n";
    std::cout << " [Lab 01 - Stage 2 核心進階版]: TopicBus & 多執行緒並發驗證 \n";
    std::cout << "===============================================================\n";

    std::atomic<bool> keepRunning{true};
    std::atomic<int> publishedCount{0};
    std::atomic<int> controllerReceivedCount{0};
    std::atomic<int> loggerReceivedCount{0};

    // 1. Fake IMU Publisher 執行緒 (250Hz = 每 4ms 一次)
    std::thread imuThread([&]() {
        Publisher<SensorImu> imuPub("sensor_imu");
        uint64_t timeUs = 1000000;
        float pitchRate = 0.0f;

        while (keepRunning) {
            timeUs += 4000;
            pitchRate += 0.01f;
            SensorImu msg{timeUs, {0.0f, pitchRate, 0.0f}, {0.0f, 0.0f, -9.81f}};
            imuPub.publish(msg);
            publishedCount++;
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
    });

    // 2. Fast Controller 訂閱執行緒 (100Hz = 每 10ms 輪詢一次)
    std::thread controllerThread([&]() {
        Subscription<SensorImu> imuSub("sensor_imu");
        SensorImu msg{};

        while (keepRunning) {
            if (imuSub.updated()) {
                if (imuSub.copy(msg)) {
                    controllerReceivedCount++;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    // 3. Slow Logger 訂閱執行緒 (20Hz = 每 50ms 輪詢一次)
    std::thread loggerThread([&]() {
        Subscription<SensorImu> imuSub("sensor_imu");
        SensorImu msg{};

        while (keepRunning) {
            if (imuSub.updated()) {
                if (imuSub.copy(msg)) {
                    loggerReceivedCount++;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });

    // 主執行緒運行 1 秒後結束
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    keepRunning = false;

    imuThread.join();
    controllerThread.join();
    loggerThread.join();

    std::cout << "\n[多執行緒並發測試統計]:\n";
    std::cout << " - IMU Publisher 總發送次數 (預期 ~250): " << publishedCount.load() << "\n";
    std::cout << " - Fast Controller 接收次數 (預期 ~100): " << controllerReceivedCount.load() << "\n";
    std::cout << " - Slow Logger 接收次數 (預期 ~20):     " << loggerReceivedCount.load() << "\n";

    if (publishedCount > 200 && controllerReceivedCount > 80 && loggerReceivedCount > 15) {
        std::cout << "✓ 多執行緒並發無 Crash、無 Deadlock，各頻率運作符合預期！\n";
        std::cout << ">>> Stage 2 全部驗收標準達成！ <<<\n";
    } else {
        std::cout << "! 注意: 計數稍微偏離，請確認系統排程負荷。\n";
    }

    return 0;
}
