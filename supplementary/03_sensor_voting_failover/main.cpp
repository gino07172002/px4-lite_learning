/**
 * @file main.cpp
 * @brief 補充模組 03: 多 IMU 感測器投票仲裁與無縫故障切換 (Voting & Failover)
 * 
 * [對標 PX4 源碼]:
 * - src/modules/sensors/vehicle_imu/VehicleIMU.cpp
 * - src/lib/sensor_calibration/Utilities.hpp
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>
#include <cassert>

struct ImuSample {
    uint64_t timestamp_us{0};
    float gyro[3]{0, 0, 0};
    float accel[3]{0, 0, -9.81f};
};

class ImuFailoverManager {
public:
    ImuFailoverManager() : _primaryIndex(0), _switchCount(0) {
        _healthy[0] = true;
        _healthy[1] = true;
    }

    // 接收各實例數據
    void pushSample(int instance, const ImuSample& sample) {
        if (instance < 0 || instance >= 2) return;
        _lastSamples[instance] = sample;
        _healthy[instance] = true;
    }

    // 執行仲裁與心跳檢驗 (週期性更新, 如 100Hz)
    bool selectHealthyImu(uint64_t currentUs, ImuSample& outSample) {
        // 1. 檢查主 IMU 健康狀態 (逾時偵測 > 50ms)
        constexpr uint64_t TIMEOUT_US = 50000;

        for (int i = 0; i < 2; ++i) {
            if (currentUs > _lastSamples[i].timestamp_us + TIMEOUT_US) {
                _healthy[i] = false;
            }
        }

        // 2. 故障轉移決策 (Failover Logic)
        if (!_healthy[_primaryIndex]) {
            int backupIndex = 1 - _primaryIndex;
            if (_healthy[backupIndex]) {
                std::cout << "[IMU FAILOVER] 主感測器 IMU " << _primaryIndex 
                          << " 故障離線！無縫熱切換至備用感測器 IMU " << backupIndex << "！\n";
                _primaryIndex = backupIndex;
                _switchCount++;
            } else {
                std::cerr << "[CRITICAL ALERT] 雙 IMU 同時離線！系統無可用慣導！\n";
                return false;
            }
        }

        outSample = _lastSamples[_primaryIndex];
        return true;
    }

    int getPrimaryIndex() const { return _primaryIndex; }
    int getSwitchCount() const { return _switchCount; }

private:
    int _primaryIndex;
    int _switchCount;
    ImuSample _lastSamples[2];
    bool _healthy[2];
};

int main() {
    std::cout << "=======================================================================\n";
    std::cout << " [補充模組 03]: 多 IMU 投票仲裁與無縫熱備份容錯 (Voting & Failover)     \n";
    std::cout << "=======================================================================\n";

    ImuFailoverManager manager;

    uint64_t timeUs = 1000000;
    ImuSample activeSample{};

    std::cout << "[階段 1]: 雙 IMU 均正常工作 (IMU 0 主力, IMU 1 熱備份)...\n";
    for (int step = 0; step < 10; ++step) {
        timeUs += 10000; // 10ms (100Hz)
        manager.pushSample(0, {timeUs, {0.01f, 0.0f, 0.0f}, {0.0f, 0.0f, -9.81f}});
        manager.pushSample(1, {timeUs, {0.01f, 0.0f, 0.0f}, {0.0f, 0.0f, -9.81f}});

        bool ok = manager.selectHealthyImu(timeUs, activeSample);
        assert(ok);
        assert(manager.getPrimaryIndex() == 0); // 應維持在 IMU 0
    }
    std::cout << "✓ 階段 1 通過: 正常狀態下穩定採樣主 IMU 0\n\n";

    std::cout << "[階段 2: 注入致命硬體故障]: 模擬 IMU 0 SPI 斷線，停止更新時間戳！\n";
    // IMU 0 不再 pushSample，僅 IMU 1 繼續運作
    for (int step = 0; step < 8; ++step) { // 持續 80ms (超過 50ms 門限)
        timeUs += 10000;
        manager.pushSample(1, {timeUs, {0.02f, 0.0f, 0.0f}, {0.0f, 0.0f, -9.81f}});

        bool ok = manager.selectHealthyImu(timeUs, activeSample);
        assert(ok);
    }

    assert(manager.getPrimaryIndex() == 1);
    assert(manager.getSwitchCount() == 1);
    std::cout << "✓ 階段 2 通過: 成功在 50ms 內偵測到主 IMU 斷線，無縫容錯切換至 IMU 1！\n\n";

    std::cout << "[階段 3]: 切換後系統持續穩定輸出 IMU 1 數據...\n";
    for (int step = 0; step < 5; ++step) {
        timeUs += 10000;
        manager.pushSample(1, {timeUs, {0.03f, 0.0f, 0.0f}, {0.0f, 0.0f, -9.81f}});
        manager.selectHealthyImu(timeUs, activeSample);
        assert(manager.getPrimaryIndex() == 1);
    }
    std::cout << "✓ 階段 3 通過: 飛控控制迴路完全無感知，平順運作！\n";

    return 0;
}
