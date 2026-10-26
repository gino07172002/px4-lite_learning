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

#include "position_control.hpp"

using px4lite::supplementary::AttitudeThrustSetpoint;
using px4lite::supplementary::MulticopterPositionControl;
using px4lite::supplementary::Vector3f;


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
