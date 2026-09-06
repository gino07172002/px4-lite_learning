/**
 * @file main.cpp
 * @brief Lab 05 - Stage 3 (PX4 對標進階版): 混控器 (Mixer) 與四旋翼角速度閉迴路階躍響應
 * 
 * [本階段目標與設計重點]:
 * 1. 控制分配混控器 (Control Allocator / Mixer):
 *    - 根據 PX4 Quad-X 幾何矩陣，將 (Roll Torque, Pitch Torque, Yaw Torque, Hover Throttle)
 *      解耦分配給 4 顆馬達 (M1, M2, M3, M4)。
 * 2. 閉迴路物理模擬 (Closed-Loop Rate Control):
 *    - 結合 Lab 04 的 6-DOF 剛體動力學受控體。
 *    - 給定 step command: roll_rate_sp = +1.0 rad/s。
 *    - 驗證閉迴路角速度追隨響應，最終穩定收斂至 1.0 rad/s！
 */

#include <iostream>
#include <cmath>
#include <algorithm>
#include <cassert>

// 1. 工業級 PID
class RatePID {
public:
    RatePID(float kp, float ki, float kd, float iLimit, float outLimit)
        : _kp(kp), _ki(ki), _kd(kd), _iLimit(iLimit), _outLimit(outLimit) {}

    float update(float setpoint, float measured, float dt) {
        float error = setpoint - measured;
        _integral += error * dt;
        _integral = std::max(-_iLimit, std::min(_iLimit, _integral));

        // 微分項作用於測量值 (Derivative on Measurement) 防止 setpoint 突變微分衝擊
        float derivative = -(measured - _prevMeasured) / dt;
        _prevMeasured = measured;

        float out = (_kp * error) + (_ki * _integral) + (_kd * derivative);
        return std::max(-_outLimit, std::min(_outLimit, out));
    }

private:
    float _kp, _ki, _kd, _iLimit, _outLimit;
    float _integral{0.0f};
    float _prevMeasured{0.0f};
};

// 2. PX4 Quad-X 控制分配混控器
class ControlAllocator {
public:
    struct MotorOutputs {
        float m[4];
    };

    // rollTorque, pitchTorque, yawTorque [-1, 1], throttle [0, 1]
    static MotorOutputs mix(float rollCmd, float pitchCmd, float yawCmd, float throttle) {
        MotorOutputs out;
        out.m[0] = throttle - rollCmd - pitchCmd - yawCmd; // M1
        out.m[1] = throttle + rollCmd + pitchCmd - yawCmd; // M2
        out.m[2] = throttle + rollCmd - pitchCmd + yawCmd; // M3
        out.m[3] = throttle - rollCmd + pitchCmd + yawCmd; // M4

        for (int i = 0; i < 4; ++i) {
            out.m[i] = std::max(0.0f, std::min(1.0f, out.m[i]));
        }
        return out;
    }
};

// 3. 四旋翼動力學 (含空氣阻尼)
class QuadPlant {
public:
    void step(const float motorCmds[4], float dt) {
        float T[4];
        for (int i = 0; i < 4; ++i) T[i] = motorCmds[i] * _maxThrust;

        float d = _armLength * 0.7071f;
        float tau_x = d * ((T[1] + T[2]) - (T[0] + T[3])); // 滾轉扭矩
        float tau_drag = -_damping * _gyro_p;               // 空氣旋轉阻尼

        // 角加速度與角速度
        float alpha_x = (tau_x + tau_drag) / _Ixx;
        _gyro_p += alpha_x * dt;
    }

    float getRollRate() const { return _gyro_p; }

private:
    float _maxThrust{6.0f};
    float _armLength{0.15f};
    float _Ixx{0.008f};
    float _damping{0.02f}; // 空氣旋轉阻尼係數
    float _gyro_p{0.0f};
};

int main() {
    std::cout << "=======================================================================\n";
    std::cout << " [Lab 05 - Stage 3 PX4對標進階版]: Mixer 混控與角速度閉迴路控制整合    \n";
    std::cout << "=======================================================================\n";

    QuadPlant quad;
    // 調諧好的角速度 PID: P=0.04, I=0.08, D=0.001
    RatePID rollPID(0.04f, 0.08f, 0.001f, 0.2f, 0.3f);

    float targetRollRate = 1.0f; // 目標: +1.0 rad/s
    float hoverThrottle = 0.49f;  // 懸停油門 49%
    float dt = 0.004f;            // 250Hz 控制頻率 (4ms)

    std::cout << "設定階躍命令: roll_rate_sp = +1.0 rad/s, 模擬 1.0 秒 (250 個控制循環)...\n\n";

    for (int step = 0; step < 250; ++step) {
        float currentRate = quad.getRollRate();

        // 1. PID 計算所需滾轉控制扭矩
        float rollTorqueCmd = rollPID.update(targetRollRate, currentRate, dt);

        // 2. 混控器計算 4 顆馬達油門
        auto motors = ControlAllocator::mix(rollTorqueCmd, 0.0f, 0.0f, hoverThrottle);

        // 3. 物理環境步進
        quad.step(motors.m, dt);

        if (step % 50 == 0) {
            std::cout << "時間 t = " << step * dt << " s | 實際角速度 = " 
                      << quad.getRollRate() << " rad/s | 扭矩命令 = " << rollTorqueCmd 
                      << " | 馬達[M2左後=" << motors.m[1] << ", M1右前=" << motors.m[0] << "]\n";
        }
    }

    float finalRate = quad.getRollRate();
    std::cout << "\n1.0 秒後最終角速度: " << finalRate << " rad/s (目標 1.0 rad/s)\n";

    assert(std::abs(finalRate - targetRollRate) < 0.08f);
    std::cout << "✓ 閉迴路響應驗證成功: 系統迅速克服慣性並精確鎖定目標角速度！\n";

    std::cout << "\n>>> Stage 3 全部驗收標準達成！ <<<\n";
    return 0;
}
