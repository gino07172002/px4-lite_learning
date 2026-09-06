/**
 * @file main.cpp
 * @brief Lab 06 - Stage 3 (PX4 對標進階版): 完整串級姿態控制閉迴路自穩 (從 +15° 歸平至 0°)
 * 
 * [本階段目標與設計重點]:
 * 1. 完整飛控回授閉迴路 (Full Cascade Flight Loop):
 *    [Quad Dynamics] ──> IMU (帶噪聲) ──> [互補濾波 Estimator]
 *           ▲                                      │
 *           │                                      ▼ vehicle_attitude
 *     [Mixer 混控] ◀── [Rate PID] ◀── [Attitude P] ◀── (target = 0°)
 * 2. 實體驗收考核 (Acceptance Milestone):
 *    - 機體初始處於非平衡狀態 (Roll = +15°)。
 *    - 系統各模組依序運算，在 2.5 秒內完全自穩，達到 |Roll| < 0.8°。
 */

#include <iostream>
#include <cmath>
#include <random>
#include <algorithm>
#include <cassert>

constexpr float DEG2RAD = 3.14159265f / 180.0f;
constexpr float RAD2DEG = 180.0f / 3.14159265f;

// 1. 互補濾波估計器
class ComplementaryFilter {
public:
    explicit ComplementaryFilter(float alpha = 0.98f) : _alpha(alpha), _roll(0.0f) {}

    void update(float gyroP, float ay, float az, float dt) {
        float rollAcc = std::atan2(ay, -az);
        _roll = (_alpha * (_roll + gyroP * dt)) + ((1.0f - _alpha) * rollAcc);
    }
    float getRoll() const { return _roll; }
    void init(float r) { _roll = r; }

private:
    float _alpha;
    float _roll;
};

// 2. 姿態外環 P 控制器
class AttitudeLoopP {
public:
    AttitudeLoopP(float kp, float maxRate) : _kp(kp), _maxRate(maxRate) {}

    float update(float targetRoll, float estRoll) {
        float err = targetRoll - estRoll;
        float rateSp = _kp * err;
        return std::max(-_maxRate, std::min(_maxRate, rateSp));
    }

private:
    float _kp, _maxRate;
};

// 3. 角速度內環 PID
class RateLoopPID {
public:
    RateLoopPID(float kp, float ki, float kd, float iLimit, float outLimit)
        : _kp(kp), _ki(ki), _kd(kd), _iLimit(iLimit), _outLimit(outLimit) {}

    float update(float rateSp, float rateMeas, float dt) {
        float err = rateSp - rateMeas;
        _integral += err * dt;
        _integral = std::max(-_iLimit, std::min(_iLimit, _integral));
        float deriv = (err - _prevErr) / dt;
        _prevErr = err;
        float out = (_kp * err) + (_ki * _integral) + (_kd * deriv);
        return std::max(-_outLimit, std::min(_outLimit, out));
    }

private:
    float _kp, _ki, _kd, _iLimit, _outLimit;
    float _integral{0.0f};
    float _prevErr{0.0f};
};

// 4. 四旋翼動力學與感測器生成
class QuadDynamics {
public:
    explicit QuadDynamics(float initialRollRad)
        : _roll(initialRollRad), _rng(1337), _noise(0.0f, 0.02f) {}

    void step(float rollCmd, float dt) {
        // rollCmd 是混控器計算後左右兩側推力產生的合力矩 (Nm)
        float alpha = rollCmd / _Ixx;
        _gyro_p += alpha * dt;
        _roll += _gyro_p * dt;
    }

    // 感測器讀數 (含噪聲)
    void getSensors(float& outGyroP, float& outAy, float& outAz) {
        outGyroP = _gyro_p + _noise(_rng);
        outAy = 9.81f * std::sin(_roll) + _noise(_rng) * 0.5f;
        outAz = -9.81f * std::cos(_roll) + _noise(_rng) * 0.5f;
    }

    float getTrueRoll() const { return _roll; }

private:
    float _roll{0.0f};
    float _gyro_p{0.0f};
    float _Ixx{0.008f};
    std::mt19937 _rng;
    std::normal_distribution<float> _noise;
};

int main() {
    std::cout << "=======================================================================\n";
    std::cout << " [Lab 06 - Stage 3 PX4對標進階版]: 完整串級姿態控制自穩 (+15° 歸平測試) \n";
    std::cout << "=======================================================================\n";

    QuadDynamics quad(15.0f * DEG2RAD); // 初始傾角 +15°
    ComplementaryFilter estimator(0.98f);
    estimator.init(15.0f * DEG2RAD);

    AttitudeLoopP attCtrl(6.5f, 3.5f);
    RateLoopPID rateCtrl(0.32f, 0.10f, 0.005f, 0.3f, 0.5f);

    float dt = 0.004f; // 250Hz 控制頻率 (4ms)

    std::cout << "初始機身姿態: " << quad.getTrueRoll() * RAD2DEG << " 度\n";
    std::cout << "控制目標: 0.0 度 (水平懸停)\n\n";

    for (int step = 0; step <= 625; ++step) { // 模擬 2.5 秒 (625 步)
        // 1. 感測器採樣
        float gyroP, ay, az;
        quad.getSensors(gyroP, ay, az);

        // 2. 姿態估計器更新
        estimator.update(gyroP, ay, az, dt);

        // 3. 姿態外環計算期望角速度
        float targetRate = attCtrl.update(0.0f, estimator.getRoll());

        // 4. 角速度內環計算扭矩
        float torqueCmd = rateCtrl.update(targetRate, gyroP, dt);

        // 5. 物理推進
        quad.step(torqueCmd, dt);

        if (step % 125 == 0) { // 每 0.5 秒輸出一次狀態
            std::cout << "t = " << step * dt << " s | 真實姿態 = " 
                      << quad.getTrueRoll() * RAD2DEG << " 度 | 估計姿態 = " 
                      << estimator.getRoll() * RAD2DEG << " 度 | 內環角速度 SP = " 
                      << targetRate << " rad/s\n";
        }
    }

    float finalRoll = quad.getTrueRoll() * RAD2DEG;
    std::cout << "\n2.5 秒後最終姿態: " << finalRoll << " 度\n";
    assert(std::abs(finalRoll) < 0.8f);
    std::cout << "✓ 終極驗收成功: 成功在 2.5 秒內克服 +15 度大傾角，平穩自主收斂至 < 0.8 度！\n";

    std::cout << "\n>>> Stage 3 全部驗收標準達成！ <<<\n";
    return 0;
}
