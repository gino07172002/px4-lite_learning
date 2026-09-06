/**
 * @file main.cpp
 * @brief Lab 06 - Stage 1 (基礎版): 真值姿態外環 P 控制器 (Ground Truth Feedback)
 * 
 * [本階段目標與設計重點]:
 * 1. 串級控制外環 (Outer Loop Attitude P Controller):
 *    - 輸入: 目標角度 (roll_sp) 與當前真值角度 (roll_actual)。
 *    - 輸出: 期望角速度 setpoint (rate_sp)。
 * 2. 串聯內環 Rate PID:
 *    - rate_sp 傳遞給角速度內環，計算控制扭矩。
 * 3. 閉迴路自穩驗收:
 *    - 初始機身傾斜 +15 度，目標為 0 度。
 *    - 驗證機身角度平滑收斂歸零。
 */

#include <iostream>
#include <cmath>
#include <algorithm>
#include <cassert>

class AttitudePController {
public:
    explicit AttitudePController(float kp, float maxRateRadS = 3.0f)
        : _kp(kp), _maxRate(maxRateRadS) {}

    float update(float targetAngleRad, float currentAngleRad) {
        float angleError = targetAngleRad - currentAngleRad;
        float desiredRate = _kp * angleError;
        return std::max(-_maxRate, std::min(_maxRate, desiredRate));
    }

private:
    float _kp;
    float _maxRate;
};

class InnerRatePID {
public:
    InnerRatePID(float kp, float ki, float kd)
        : _kp(kp), _ki(ki), _kd(kd) {}

    float update(float rateSp, float rateAct, float dt) {
        float error = rateSp - rateAct;
        _integral += error * dt;
        _integral = std::max(-0.5f, std::min(0.5f, _integral));
        float derivative = (error - _prevError) / dt;
        _prevError = error;
        return (_kp * error) + (_ki * _integral) + (_kd * derivative);
    }

private:
    float _kp, _ki, _kd;
    float _integral{0.0f};
    float _prevError{0.0f};
};

// 簡化動力學
class SimpleRollPlant {
public:
    explicit SimpleRollPlant(float initialRollRad) : _roll(initialRollRad) {}

    void step(float torque, float dt) {
        float alpha = torque / _Ixx;
        _rate += alpha * dt;
        _roll += _rate * dt;
    }

    float getRoll() const { return _roll; }
    float getRate() const { return _rate; }

private:
    float _Ixx{0.008f};
    float _roll{0.0f};
    float _rate{0.0f};
};

int main() {
    std::cout << "========================================================\n";
    std::cout << " [Lab 06 - Stage 1 基礎版]: 真值姿態外環 P 串級控制測試  \n";
    std::cout << "========================================================\n";

    constexpr float DEG2RAD = 3.14159265f / 180.0f;
    constexpr float RAD2DEG = 180.0f / 3.14159265f;

    SimpleRollPlant quad(15.0f * DEG2RAD); // 初始 +15 度
    AttitudePController attCtrl(6.5f, 4.0f);
    InnerRatePID rateCtrl(0.35f, 0.12f, 0.006f);

    float dt = 0.004f; // 250Hz (4ms)

    std::cout << "初始姿態: " << quad.getRoll() * RAD2DEG << " 度 -> 目標姿態: 0.0 度\n\n";

    for (int step = 0; step <= 250; ++step) { // 模擬 1.0 秒
        float currentRoll = quad.getRoll();
        float currentRate = quad.getRate();

        // 1. 外環: 姿態誤差 -> 期望角速度
        float desiredRate = attCtrl.update(0.0f, currentRoll);

        // 2. 內環: 角速度誤差 -> 扭矩
        float torque = rateCtrl.update(desiredRate, currentRate, dt);

        // 3. 物理環境響應
        quad.step(torque, dt);

        if (step % 50 == 0) {
            std::cout << "時間 t = " << step * dt << " s | Roll = " 
                      << quad.getRoll() * RAD2DEG << " 度 | 期望角速度 = " 
                      << desiredRate << " rad/s | 扭矩 = " << torque << "\n";
        }
    }

    float finalDeg = quad.getRoll() * RAD2DEG;
    std::cout << "\n1.0 秒後最終姿態角度: " << finalDeg << " 度\n";
    assert(std::abs(finalDeg) < 0.5f);
    std::cout << "✓ 驗證通過: 串級 P-PID 控制成功將 +15 度拉平至接近 0 度！\n";

    std::cout << "\n>>> Stage 1 全部驗收標準達成！ <<<\n";
    return 0;
}
