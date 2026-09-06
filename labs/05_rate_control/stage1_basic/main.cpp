/**
 * @file main.cpp
 * @brief Lab 05 - Stage 1 (基礎版): 基礎角速度 PID 控制器
 * 
 * [本階段目標與設計重點]:
 * 1. 實現經典 PID 控制公式:
 *    u(t) = Kp * e + Ki * integral(e) + Kd * de/dt
 * 2. 單元驗證:
 *    - Test A: 當目標角速度與實際角速度均為 0 時，輸出接近 0。
 *    - Test B: 當目標為 +1 rad/s，實際為 0 時，輸出正向滾轉力矩。
 */

#include <iostream>
#include <cmath>
#include <cassert>

class SimpleRatePID {
public:
    SimpleRatePID(float kp, float ki, float kd)
        : _kp(kp), _ki(ki), _kd(kd) {}

    float update(float rateSetpoint, float currentRate, float dt) {
        if (dt <= 0.0f) return 0.0f;

        float error = rateSetpoint - currentRate;
        _integral += error * dt;
        float derivative = (error - _prevError) / dt;
        _prevError = error;

        return (_kp * error) + (_ki * _integral) + (_kd * derivative);
    }

    void reset() {
        _integral = 0.0f;
        _prevError = 0.0f;
    }

private:
    float _kp;
    float _ki;
    float _kd;
    float _integral{0.0f};
    float _prevError{0.0f};
};

int main() {
    std::cout << "========================================================\n";
    std::cout << " [Lab 05 - Stage 1 基礎版]: 基礎 PID 角速度控制單元測試  \n";
    std::cout << "========================================================\n";

    SimpleRatePID pid(0.2f, 0.05f, 0.005f);

    // Test A: 零誤差狀態
    std::cout << "\n[Test A]: Setpoint = 0.0, Rate = 0.0 ...\n";
    float torqueZero = pid.update(0.0f, 0.0f, 0.01f);
    std::cout << " -> 輸出扭矩: " << torqueZero << " Nm (預期接近 0.0)\n";
    assert(std::abs(torqueZero) < 1e-6f);
    std::cout << "✓ Test A 通過: 無誤差時輸出為 0\n";

    // Test B: 正向階躍誤差
    std::cout << "\n[Test B]: Setpoint = +1.0 rad/s, Current Rate = 0.0 ...\n";
    pid.reset();
    float torquePos = pid.update(1.0f, 0.0f, 0.01f);
    std::cout << " -> 輸出扭矩: " << torquePos << " Nm (預期為正值)\n";
    assert(torquePos > 0.0f);
    std::cout << "✓ Test B 通過: 產生正向校正扭矩！\n";

    std::cout << "\n>>> Stage 1 全部驗收標準達成！ <<<\n";
    return 0;
}
