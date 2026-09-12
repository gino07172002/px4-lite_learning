/**
 * @file main.cpp
 * @brief Lab 03: Clock Abstraction & Parameters (時間抽象與參數管理) - 開局範本
 * 
 * [Lab 03 核心任務目標]:
 * 1. 時間抽象 (Clock Abstraction):
 *    - 定義 IClock 介面 (nowUs())
 *    - 實作 RealtimeClock (實體機時鐘，依賴 std::chrono::steady_clock)
 *    - 實作 SimulationClock (SITL 模擬時鐘，可手動 advanceUs 步進，支援可重現模擬)
 * 
 * 2. 參數伺服器 (Parameter Manager):
 *    - 支援鍵值型態管理 (如 int, float, bool)
 *    - 邊界檢查 (Min / Max Clamping)
 *    - 參數變更通知機制 (對標 PX4 parameter_update)
 */

#include <iostream>
#include <chrono>
#include <cstdint>
#include <string>
#include <memory>

// ==========================================
// 1. 時間抽象介面 (IClock)
// ==========================================
class IClock {
public:
    virtual ~IClock() = default;
    virtual uint64_t nowUs() const = 0;
};

// 真實世界時鐘 (用於硬體執行緒與即時運行)
class RealtimeClock : public IClock {
public:
    RealtimeClock() : _epoch(std::chrono::steady_clock::now()) {}

    uint64_t nowUs() const override {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(now - _epoch).count();
    }

private:
    std::chrono::steady_clock::time_point _epoch;
};

// 模擬時鐘 (用於 SITL 軟體在環模擬與單元測試，時間確定性前進)
class SimulationClock : public IClock {
public:
    explicit SimulationClock(uint64_t initialTimeUs = 0) : _currentTimeUs(initialTimeUs) {}

    uint64_t nowUs() const override {
        return _currentTimeUs;
    }

    void advanceUs(uint64_t dtUs) {
        _currentTimeUs += dtUs;
    }

private:
    uint64_t _currentTimeUs{0};
};

// ==========================================
// 2. 參數管理器初階骨架 (TODO: 依需求擴充型別安全與邊界檢查)
// ==========================================
class ParameterManager {
public:
    ParameterManager() = default;
    // 使用者可以在此實作 registerParam, get, set 等功能
};

int main() {
    std::cout << "=========================================" << std::endl;
    std::cout << " Hello Lab 03: Clock & Parameters!       " << std::endl;
    std::cout << "=========================================" << std::endl;

    // 1. 測試真實時鐘
    RealtimeClock realClock;
    std::cout << "[RealtimeClock] 當前時間: " << realClock.nowUs() << " us" << std::endl;

    // 2. 測試模擬時鐘
    SimulationClock simClock(0);
    std::cout << "[SimulationClock] 初始時間: " << simClock.nowUs() << " us" << std::endl;
    simClock.advanceUs(4000); // 模擬步進 4ms (250Hz)
    std::cout << "[SimulationClock] 前進 4ms 後時間: " << simClock.nowUs() << " us" << std::endl;

    std::cout << "\nLab 03 Hello World 開局成功，準備開始實作！" << std::endl;
    return 0;
}
