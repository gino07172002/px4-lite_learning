/**
 * @file main.cpp
 * @brief 補充模組 02: 3D 位置與速度控制器 (Multicopter Position Control)
 * 
 * [對標 PX4 源碼]:
 * - src/modules/mc_pos_control
 * - src/modules/mc_pos_control/PositionControl/PositionControl.cpp
 */

#include <iostream>
#include <cmath>
#include <algorithm>
#include <cassert>

struct Vector3f {
    float x{0.0f}, y{0.0f}, z{0.0f};

    Vector3f operator+(const Vector3f& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vector3f operator-(const Vector3f& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vector3f operator*(float s) const { return {x * s, y * s, z * s}; }
    float norm() const { return std::sqrt(x * x + y * y + z * z); }
};

struct AttitudeThrustSetpoint {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float thrust_norm; // 0.0 ~ 1.0
};

class MulticopterPositionControl {
public:
    MulticopterPositionControl() = default;

    AttitudeThrustSetpoint update(const Vector3f& posSp, const Vector3f& posActual,
                                  const Vector3f& velActual, float dt) {
        // 1. 位置外環 P 控制器 (Position Error -> Velocity Setpoint)
        Vector3f posErr = posSp - posActual;
        Vector3f velSp;
        velSp.x = std::max(-_maxHorizVel, std::min(_maxHorizVel, _posP * posErr.x));
        velSp.y = std::max(-_maxHorizVel, std::min(_maxHorizVel, _posP * posErr.y));
        velSp.z = std::max(-_maxVerticalVel, std::min(_maxVerticalVel, _posP_z * posErr.z));

        // 2. 速度內環 PID 控制器 (Velocity Error -> Desired Acceleration)
        Vector3f velErr = velSp - velActual;
        _velInt = _velInt + velErr * dt;
        // 積分限幅
        _velInt.x = std::max(-2.0f, std::min(2.0f, _velInt.x));
        _velInt.y = std::max(-2.0f, std::min(2.0f, _velInt.y));
        _velInt.z = std::max(-2.0f, std::min(2.0f, _velInt.z));

        Vector3f a_des;
        a_des.x = _velP * velErr.x + _velI * _velInt.x;
        a_des.y = _velP * velErr.y + _velI * _velInt.y;
        // Z 軸需加入抵消重力的 +g 項 (以向上為正的座標系)
        a_des.z = _velP_z * velErr.z + _velI_z * _velInt.z + _gravity;

        // 3. 幾何投影解算機身姿態與推力 (Geometric Attitude & Thrust Setpoint)
        // 水平加速度限制 (避免傾角過大造成失速翻機)
        float maxTiltRad = 35.0f * (3.14159265f / 180.0f);
        float a_horiz = std::sqrt(a_des.x * a_des.x + a_des.y * a_des.y);
        float max_a_horiz = _gravity * std::tan(maxTiltRad);
        if (a_horiz > max_a_horiz) {
            float scale = max_a_horiz / a_horiz;
            a_des.x *= scale;
            a_des.y *= scale;
        }

        // 解算 Roll 與 Pitch (單位: 度)
        // 往前加速 (+X) 需要機頭下壓 (負 Pitch)
        // 往右加速 (+Y) 需要機身右傾 (正 Roll)
        constexpr float RAD2DEG = 180.0f / 3.14159265f;
        float pitch_sp = -std::atan2(a_des.x, a_des.z) * RAD2DEG;
        float roll_sp  =  std::atan2(a_des.y, a_des.z) * RAD2DEG;

        // 計算正規化推力: 總期望力 / 最大推力
        float totalAccel = a_des.norm();
        float hoverThrustNorm = 0.50f; // 懸停推力約 50%
        float thrust_norm = hoverThrustNorm * (totalAccel / _gravity);
        thrust_norm = std::max(0.1f, std::min(0.95f, thrust_norm));

        return {roll_sp, pitch_sp, 0.0f, thrust_norm};
    }

private:
    float _gravity{9.81f};
    float _posP{1.0f};
    float _posP_z{1.2f};
    float _velP{1.8f};
    float _velI{0.15f};
    float _velP_z{2.5f};
    float _velI_z{0.3f};

    float _maxHorizVel{5.0f};   // 最大水平速度 5 m/s
    float _maxVerticalVel{2.5f}; // 最大垂直速度 2.5 m/s

    Vector3f _velInt{0, 0, 0};
};

int main() {
    std::cout << "=======================================================================\n";
    std::cout << " [補充模組 02]: 3D 位置與速度控制器 (Position -> Attitude Setpoint)    \n";
    std::cout << "=======================================================================\n";

    MulticopterPositionControl posCtrl;

    Vector3f currentPos{0.0f, 0.0f, 1.0f}; // 當前位置 (0, 0, 1m)
    Vector3f currentVel{0.0f, 0.0f, 0.0f}; // 初始靜止
    Vector3f targetPos{3.0f, 0.0f, 2.0f};  // 目標位置: 前方 3m, 上方 2m

    std::cout << "初始位置: (" << currentPos.x << ", " << currentPos.y << ", " << currentPos.z << ") m\n";
    std::cout << "目標位置: (" << targetPos.x  << ", " << targetPos.y  << ", " << targetPos.z  << ") m\n\n";

    // 測試 1: 剛出發時的前傾與推力加大特性
    auto sp1 = posCtrl.update(targetPos, currentPos, currentVel, 0.02f);
    std::cout << "[起跑瞬間指令]:\n";
    std::cout << " - 期望 Pitch: " << sp1.pitch_deg << " 度 (預期為負值，即機頭前傾加速)\n";
    std::cout << " - 期望 Roll:  " << sp1.roll_deg  << " 度 (預期接近 0 度，因無側向誤差)\n";
    std::cout << " - 總推力命令: " << sp1.thrust_norm * 100.0f << " % (預期高於 50% 懸停推力，用於爬升)\n";

    assert(sp1.pitch_deg < -5.0f);  // 必須前傾
    assert(std::abs(sp1.roll_deg) < 1e-4f);
    assert(sp1.thrust_norm > 0.50f); // 必須加大推力
    std::cout << "✓ 測試 1 通過: 位置控制器成功生成正確的前傾推進與爬升姿態推力！\n\n";

    // 測試 2: 模擬 3D 空間位置收斂閉迴路
    std::cout << "[測試 2]: 模擬 3 秒 3D 軌跡飛行...\n";
    for (int step = 0; step < 150; ++step) { // 3 秒 (50Hz, dt=0.02s)
        auto sp = posCtrl.update(targetPos, currentPos, currentVel, 0.02f);

        // 簡化質點運動響應 (模擬機體姿態響應後的加速度)
        float ax = -sp.pitch_deg * 0.15f;
        float az = (sp.thrust_norm / 0.50f - 1.0f) * 9.81f;

        currentVel.x += ax * 0.02f;
        currentVel.z += az * 0.02f;
        currentPos.x += currentVel.x * 0.02f;
        currentPos.z += currentVel.z * 0.02f;
    }

    std::cout << "3.0 秒後到達位置: (" << currentPos.x << ", " << currentPos.y << ", " << currentPos.z << ") m\n";
    assert(std::abs(currentPos.x - 3.0f) < 0.3f);
    assert(std::abs(currentPos.z - 2.0f) < 0.2f);
    std::cout << "✓ 測試 2 通過: 3D 位置控制器成功將機體引導至目標航點並精確懸停！\n";

    return 0;
}
