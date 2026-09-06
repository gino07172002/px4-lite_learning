/**
 * @file main.cpp
 * @brief Lab 07 - Stage 2 (核心進階版): 預解鎖健康檢查 (Pre-arm Health Checks & Arm Denied)
 * 
 * [本階段目標與設計重點]:
 * 1. 對標 PX4 預解鎖安全檢查 (Pre-arm Checks):
 *    - 在解鎖 (ARM) 前，嚴格審查各子系統健康報告:
 *      * IMU 感測器是否在線且校準良好 (imu_ok)
 *      * 姿態估計器是否已收斂 (estimator_ok)
 *      * 電池電量是否高於安全門檻 (battery_ok)
 * 2. 拒絕解鎖保護 (ARM DENIED):
 *    - 若有任何一項不符合，拒絕切換至 ARMED，並向地面站/使用者明確回報原因。
 */

#include <iostream>
#include <string>
#include <cassert>

struct SubsystemHealth {
    bool imu_ok{false};
    bool estimator_ok{false};
    bool battery_ok{false};
};

class SafeCommander {
public:
    enum class State {
        STANDBY,
        ARMED,
        FLYING
    };

    SafeCommander() : _state(State::STANDBY) {}

    // 更新硬體健康狀態
    void setHealth(const SubsystemHealth& health) {
        _health = health;
    }

    // 嘗試解鎖
    bool arm(std::string& outDenyReason) {
        if (_state != State::STANDBY) {
            outDenyReason = "系統不在 STANDBY 狀態";
            return false;
        }

        // 預解鎖審查 (Pre-arm Checks)
        if (!_health.imu_ok) {
            outDenyReason = "PRE-ARM FAIL: IMU 感測器離線或未校準！";
            return false;
        }
        if (!_health.estimator_ok) {
            outDenyReason = "PRE-ARM FAIL: 姿態估計器未收斂！";
            return false;
        }
        if (!_health.battery_ok) {
            outDenyReason = "PRE-ARM FAIL: 電池電壓過低，禁止起飛！";
            return false;
        }

        _state = State::ARMED;
        std::cout << "[COMMANDER] 解鎖成功 -> 進入 ARMED 狀態！\n";
        return true;
    }

    void disarm() {
        _state = State::STANDBY;
        std::cout << "[COMMANDER] 上鎖 -> 進入 STANDBY 狀態\n";
    }

    State getState() const { return _state; }

private:
    State _state;
    SubsystemHealth _health;
};

int main() {
    std::cout << "===============================================================\n";
    std::cout << " [Lab 07 - Stage 2 核心進階版]: 預解鎖健康檢查與安全防護測試   \n";
    std::cout << "===============================================================\n";

    SafeCommander cmd;
    std::string denyReason;

    // 情況 1: IMU 故障，嘗試解鎖
    std::cout << "\n[測試 1]: IMU 不健康時嘗試解鎖...\n";
    cmd.setHealth({false, true, true}); // imu_ok = false
    bool ok1 = cmd.arm(denyReason);
    assert(!ok1);
    assert(cmd.getState() == SafeCommander::State::STANDBY);
    std::cout << " -> " << denyReason << "\n";
    std::cout << "✓ 測試 1 通過: 成功攔截並防止帶病解鎖！\n";

    // 情況 2: 估計器未收斂，嘗試解鎖
    std::cout << "\n[測試 2]: 姿態估計器未收斂時嘗試解鎖...\n";
    cmd.setHealth({true, false, true}); // estimator_ok = false
    bool ok2 = cmd.arm(denyReason);
    assert(!ok2);
    std::cout << " -> " << denyReason << "\n";
    std::cout << "✓ 測試 2 通過: 成功攔截姿態估計異常！\n";

    // 情況 3: 全部系統健康，正常解鎖
    std::cout << "\n[測試 3]: 系統全健康狀態解鎖...\n";
    cmd.setHealth({true, true, true});
    bool ok3 = cmd.arm(denyReason);
    assert(ok3);
    assert(cmd.getState() == SafeCommander::State::ARMED);
    std::cout << "✓ 測試 3 通過: 系統順利解鎖進入 ARMED！\n";

    std::cout << "\n>>> Stage 2 全部驗收標準達成！ <<<\n";
    return 0;
}
