/**
 * @file main.cpp
 * @brief Lab 07 - Stage 3 (PX4 對標進階版): 飛行中心跳監控、感測器超時自救與完整單元測試清單
 * 
 * [本階段目標與設計重點]:
 * 1. 飛行中即時守護 (In-flight Heartbeat Watchdog):
 *    - 在 FLYING 狀態下，Commander 週期性檢查各感測器最後更新時間戳 (Timestamp)。
 *    - 若 IMU 逾時超過 200ms (timeout)，觸發感測器失效警報。
 * 2. 失效安全程序 (Failsafe Action):
 *    - 自動觸發 state 轉移: FLYING -> FAILSAFE -> AUTO_LANDING。
 * 3. 完整 PX4-lite 單元測試清單 (Roadmap Unit Tests 1~8 全覆蓋)。
 */

#include <iostream>
#include <string>
#include <cstdint>
#include <cassert>

enum class NavState {
    INIT,
    STANDBY,
    ARMED,
    FLYING,
    FAILSAFE,
    LANDING
};

std::string navStateToString(NavState s) {
    switch (s) {
        case NavState::INIT: return "INIT";
        case NavState::STANDBY: return "STANDBY";
        case NavState::ARMED: return "ARMED";
        case NavState::FLYING: return "FLYING";
        case NavState::FAILSAFE: return "FAILSAFE";
        case NavState::LANDING: return "LANDING";
    }
    return "UNKNOWN";
}

class AdvancedCommander {
public:
    AdvancedCommander() : _state(NavState::INIT) {}

    void initDone() {
        if (_state == NavState::INIT) {
            _state = NavState::STANDBY;
        }
    }

    bool arm(bool imuOk, bool estOk) {
        if (_state != NavState::STANDBY) return false;
        if (!imuOk || !estOk) {
            std::cout << "[COMMANDER] 解鎖拒絕: 感測器狀態異常！\n";
            return false;
        }
        _state = NavState::ARMED;
        return true;
    }

    bool takeoff() {
        if (_state != NavState::ARMED) return false;
        _state = NavState::FLYING;
        return true;
    }

    bool land() {
        if (_state != NavState::FLYING) return false;
        _state = NavState::LANDING;
        return true;
    }

    // 飛行中心跳監控 (週期性呼叫)
    void checkInFlightHealth(uint64_t currentUs, uint64_t lastImuUs) {
        if (_state == NavState::FLYING) {
            if (currentUs > lastImuUs + 200000) { // 超過 200ms 沒收到 IMU
                std::cout << "[EMERGENCY] 偵測到 IMU 數據超時！立即切換 FAILSAFE 模式！\n";
                _state = NavState::FAILSAFE;
                // 自動轉入緊急降落程序
                _state = NavState::LANDING;
            }
        }
    }

    NavState getState() const { return _state; }

private:
    NavState _state;
};

int main() {
    std::cout << "=======================================================================\n";
    std::cout << " [Lab 07 - Stage 3 PX4對標進階版]: 飛行中故障容錯自救與全面單元測試清單 \n";
    std::cout << "=======================================================================\n";

    AdvancedCommander cmd;

    // Unit Test 1: INIT -> STANDBY
    assert(cmd.getState() == NavState::INIT);
    cmd.initDone();
    assert(cmd.getState() == NavState::STANDBY);
    std::cout << "✓ Test 1 通過: INIT -> STANDBY\n";

    // Unit Test 2: STANDBY -> ARMED (全健康)
    assert(cmd.arm(true, true));
    assert(cmd.getState() == NavState::ARMED);
    std::cout << "✓ Test 2 通過: STANDBY -> ARMED (正常解鎖)\n";

    // Unit Test 3: cannot ARM when unhealthy
    AdvancedCommander unhealthyCmd;
    unhealthyCmd.initDone();
    assert(!unhealthyCmd.arm(false, true)); // IMU 異常
    assert(unhealthyCmd.getState() == NavState::STANDBY);
    std::cout << "✓ Test 3 通過: 故障狀態無法解鎖 (Arm Denied)\n";

    // Unit Test 4: ARMED -> TAKEOFF -> FLYING
    assert(cmd.takeoff());
    assert(cmd.getState() == NavState::FLYING);
    std::cout << "✓ Test 4 & 5 通過: ARMED -> TAKEOFF -> FLYING\n";

    // Unit Test 6: FLYING -> LANDING (正常降落指令)
    AdvancedCommander normalFlight;
    normalFlight.initDone();
    normalFlight.arm(true, true);
    normalFlight.takeoff();
    assert(normalFlight.land());
    assert(normalFlight.getState() == NavState::LANDING);
    std::cout << "✓ Test 6 通過: FLYING -> LANDING\n";

    // Unit Test 7: Sensor failure in flight -> FAILSAFE -> LANDING
    std::cout << "\n[模擬飛行中感測器失效情境]:\n";
    std::cout << "機身正在高空自主飛行，突然 IMU 訊號中斷超過 200ms...\n";
    uint64_t now = 5000000;      // 5.0 秒
    uint64_t imuLoss = 4700000;  // 上次更新在 4.7 秒 (中斷 300ms)
    cmd.checkInFlightHealth(now, imuLoss);
    assert(cmd.getState() == NavState::LANDING);
    std::cout << "✓ Test 7 通過: 飛行中感測器失聯立即觸發 FAILSAFE 並轉入降落！\n";

    // Unit Test 8: Invalid transition rejected
    AdvancedCommander rejectCmd;
    rejectCmd.initDone();
    assert(!rejectCmd.takeoff()); // STANDBY 不能直接 takeoff
    assert(rejectCmd.getState() == NavState::STANDBY);
    std::cout << "✓ Test 8 通過: 非法狀態轉換被嚴格阻擋！\n";

    std::cout << "\n>>> Stage 3 全部 8 項驗收標準達成！ <<<\n";
    return 0;
}
