/**
 * @file main.cpp
 * @brief Lab 03 - Stage 1 (基礎版): 時間抽象 (Clock Abstraction)
 * 
 * [本階段目標與設計重點]:
 * 1. 介面解耦: 定義 IClock 抽象基底類別 (nowUs)。
 * 2. 實體時鐘 RealtimeClock: 封裝系統高精度時鐘。
 * 3. 模擬時鐘 SimulationClock: 支援 step-by-step 人為前進 (advanceUs)，保證時間不可倒退。
 * 4. 控制器依賴注入: 控制器只感知 IClock，具備在真實硬體與模擬器中無縫切換的能力。
 */

#include <iostream>
#include <chrono>
#include <cstdint>
#include <memory>
#include <cassert>
#include <thread>

class IClock {
public:
    virtual ~IClock() = default;
    virtual uint64_t nowUs() const = 0;
};

// 真實世界時鐘 (用於實體無人機硬體或即時 SITL)
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

// 步進模擬時鐘 (用於軟體在環 SITL 與單元測試)
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

// 飛控控制器範例：不直接呼叫 steady_clock，而是透過 IClock 獲取時間
class ControllerModule {
public:
    explicit ControllerModule(const IClock& clock) : _clock(clock), _lastTimeUs(0) {}

    float update() {
        uint64_t now = _clock.nowUs();
        if (_lastTimeUs == 0) {
            _lastTimeUs = now;
            return 0.0f;
        }
        float dtSec = (now - _lastTimeUs) * 1e-6f;
        _lastTimeUs = now;
        return dtSec;
    }

private:
    const IClock& _clock;
    uint64_t _lastTimeUs{0};
};

int main() {
    std::cout << "========================================================\n";
    std::cout << " [Lab 03 - Stage 1 基礎版]: Clock 時間抽象與依賴注入     \n";
    std::cout << "========================================================\n";

    // 1. 測試 SimulationClock 的確定性 (Deterministic)
    SimulationClock simClock(1000000); // 初始 1 秒 (1000000 us)
    ControllerModule simController(simClock);

    simController.update(); // 初始化 lastTime

    simClock.advanceUs(4000); // 前進 4ms (250Hz)
    float dt1 = simController.update();
    std::cout << "SimClock 推進 4000 us -> Controller 計算 dt = " << dt1 << " s (預期 0.0040)\n";
    assert(std::abs(dt1 - 0.004f) < 1e-6f);

    simClock.advanceUs(10000); // 前進 10ms (100Hz)
    float dt2 = simController.update();
    std::cout << "SimClock 推進 10000 us -> Controller 計算 dt = " << dt2 << " s (預期 0.0100)\n";
    assert(std::abs(dt2 - 0.010f) < 1e-6f);

    // 2. 測試 RealtimeClock 的即時性
    RealtimeClock realClock;
    ControllerModule realController(realClock);
    realController.update();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    float realDt = realController.update();
    std::cout << "RealtimeClock 等待 50ms -> Controller 計算 dt = " << realDt << " s\n";
    assert(realDt >= 0.045f && realDt <= 0.065f);

    std::cout << "\n✓ 驗收通過: 控制器完全依賴 IClock 介面，時間可人工步進且確定可重現！\n";
    std::cout << ">>> Stage 1 全部驗收標準達成！ <<<\n";
    return 0;
}
