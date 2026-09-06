/**
 * @file main.cpp
 * @brief Lab 07 - Stage 1 (基礎版): Commander 核心有限狀態機 (FSM)
 * 
 * [本階段目標與設計重點]:
 * 1. 狀態定義: INIT, STANDBY, ARMED, FLYING, LANDING。
 * 2. 指令定義: CMD_ARM, CMD_DISARM, CMD_TAKEOFF, CMD_LAND, CMD_TOUCHDOWN。
 * 3. 轉移防護: 非法狀態切換安全攔截 (如 STANDBY 狀態下不能直接 TAKEOFF)。
 */

#include <iostream>
#include <string>
#include <cassert>

enum class FlightState {
    INIT,
    STANDBY,
    ARMED,
    FLYING,
    LANDING
};

enum class Command {
    INIT_DONE,
    ARM,
    DISARM,
    TAKEOFF,
    LAND,
    TOUCHDOWN
};

std::string stateToString(FlightState s) {
    switch (s) {
        case FlightState::INIT: return "INIT";
        case FlightState::STANDBY: return "STANDBY";
        case FlightState::ARMED: return "ARMED";
        case FlightState::FLYING: return "FLYING";
        case FlightState::LANDING: return "LANDING";
    }
    return "UNKNOWN";
}

class SimpleCommander {
public:
    SimpleCommander() : _state(FlightState::INIT) {}

    bool handleCommand(Command cmd) {
        switch (_state) {
            case FlightState::INIT:
                if (cmd == Command::INIT_DONE) {
                    transitionTo(FlightState::STANDBY);
                    return true;
                }
                break;

            case FlightState::STANDBY:
                if (cmd == Command::ARM) {
                    transitionTo(FlightState::ARMED);
                    return true;
                }
                break;

            case FlightState::ARMED:
                if (cmd == Command::TAKEOFF) {
                    transitionTo(FlightState::FLYING);
                    return true;
                } else if (cmd == Command::DISARM) {
                    transitionTo(FlightState::STANDBY);
                    return true;
                }
                break;

            case FlightState::FLYING:
                if (cmd == Command::LAND) {
                    transitionTo(FlightState::LANDING);
                    return true;
                }
                break;

            case FlightState::LANDING:
                if (cmd == Command::TOUCHDOWN) {
                    transitionTo(FlightState::STANDBY);
                    return true;
                }
                break;
        }

        std::cout << "[REJECTED] 在狀態 " << stateToString(_state) << " 下無法執行此指令！\n";
        return false;
    }

    FlightState getState() const { return _state; }

private:
    void transitionTo(FlightState next) {
        std::cout << "[STATE] " << stateToString(_state) << " -> " << stateToString(next) << "\n";
        _state = next;
    }

    FlightState _state;
};

int main() {
    std::cout << "========================================================\n";
    std::cout << " [Lab 07 - Stage 1 基礎版]: Commander 狀態機轉移驗證     \n";
    std::cout << "========================================================\n";

    SimpleCommander commander;
    assert(commander.getState() == FlightState::INIT);

    // 1. 初始化完成
    commander.handleCommand(Command::INIT_DONE);
    assert(commander.getState() == FlightState::STANDBY);

    // 2. 測試非法指令: 尚未 ARM 就嘗試 TAKEOFF
    std::cout << "測試未解鎖時起飛:\n";
    bool rejected = !commander.handleCommand(Command::TAKEOFF);
    assert(rejected);
    assert(commander.getState() == FlightState::STANDBY);
    std::cout << "✓ 非法轉移成功被攔截！\n";

    // 3. 正常飛行週期
    std::cout << "\n執行標準起降生命週期:\n";
    commander.handleCommand(Command::ARM);
    assert(commander.getState() == FlightState::ARMED);

    commander.handleCommand(Command::TAKEOFF);
    assert(commander.getState() == FlightState::FLYING);

    commander.handleCommand(Command::LAND);
    assert(commander.getState() == FlightState::LANDING);

    commander.handleCommand(Command::TOUCHDOWN);
    assert(commander.getState() == FlightState::STANDBY);

    std::cout << "\n>>> Stage 1 全部驗收標準達成！ <<<\n";
    return 0;
}
