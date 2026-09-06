/**
 * @file main.cpp
 * @brief Lab 04 - Stage 2 (核心進階版): 6-DOF 四旋翼剛體動力學模擬器 (Quad-X)
 * 
 * [本階段目標與設計重點]:
 * 1. 四旋翼 X 架構混控幾何 (FRD 機體座標系):
 *    - M1 (右前 Front-Right, CW)
 *    - M2 (左後 Rear-Left,   CW)
 *    - M3 (左前 Front-Left,  CCW)
 *    - M4 (右後 Rear-Right,  CCW)
 * 2. 轉動力矩 (Torques):
 *    - Roll 力矩  tau_x = d * ( (T2 + T3) - (T1 + T4) )  [左側推力 - 右側推力]
 *    - Pitch 力矩 tau_y = d * ( (T2 + T4) - (T1 + T3) )  [後方推力 - 前方推力]
 *    - Yaw 力矩   tau_z = km * ( (T3 + T4) - (T1 + T2) ) [CCW 反扭矩 - CW 反扭矩]
 * 3. 歐拉運動方程積分:
 *    - domega/dt = tau / I
 *    - 姿態角積分 (Euler integration)
 */

#include <iostream>
#include <cmath>
#include <cassert>

struct Quad6DState {
    float roll{0.0f};       // rad (滾轉角)
    float pitch{0.0f};      // rad (俯仰角)
    float yaw{0.0f};        // rad (偏航角)
    float gyro[3]{0, 0, 0}; // rad/s (機體角速度 p, q, r)
    float z{0.0f};          // m (高度)
    float vz{0.0f};         // m/s (垂直速度)
};

class Quad6DSimulator {
public:
    Quad6DSimulator() = default;

    void step(const float motorCmds[4], float dt) {
        // 計算四馬達推力 (N)
        float T[4];
        float totalThrust = 0.0f;
        for (int i = 0; i < 4; ++i) {
            float cmd = std::max(0.0f, std::min(1.0f, motorCmds[i]));
            T[i] = cmd * _maxThrustPerMotor;
            totalThrust += T[i];
        }

        // 機體力矩 (Nm) - 機臂斜投影距離 d = L / sqrt(2)
        float d = _armLength * 0.7071f;
        float tau_x = d * ((T[1] + T[2]) - (T[0] + T[3])); // 左減右 -> 正滾轉 (右傾)
        float tau_y = d * ((T[1] + T[3]) - (T[0] + T[2])); // 後減前 -> 正俯仰 (抬頭)
        float tau_z = _torqueCoeff * ((T[2] + T[3]) - (T[0] + T[1])); // 反扭矩差

        // 角加速度 (rad/s^2)
        float alpha_x = tau_x / _Ixx;
        float alpha_y = tau_y / _Iyy;
        float alpha_z = tau_z / _Izz;

        // 角速度積分
        _state.gyro[0] += alpha_x * dt;
        _state.gyro[1] += alpha_y * dt;
        _state.gyro[2] += alpha_z * dt;

        // 姿態歐拉角積分
        _state.roll  += _state.gyro[0] * dt;
        _state.pitch += _state.gyro[1] * dt;
        _state.yaw   += _state.gyro[2] * dt;

        // 垂直線加速度與積分 (簡化在小角度下)
        float az = (totalThrust / _mass) - _gravity;
        _state.vz += az * dt;
        _state.z  += _state.vz * dt;
        if (_state.z < 0.0f) {
            _state.z = 0.0f;
            if (_state.vz < 0.0f) _state.vz = 0.0f;
        }
    }

    const Quad6DState& getState() const { return _state; }

    void reset() { _state = Quad6DState{}; }

private:
    float _mass{1.2f};               // 1.2 kg
    float _gravity{9.81f};
    float _maxThrustPerMotor{6.0f};  // 6 N / motor
    float _armLength{0.15f};         // 15 cm
    float _torqueCoeff{0.02f};       // 反扭矩比值
    float _Ixx{0.008f};              // kg*m^2
    float _Iyy{0.008f};
    float _Izz{0.015f};
    Quad6DState _state;
};

int main() {
    std::cout << "===============================================================\n";
    std::cout << " [Lab 04 - Stage 2 核心進階版]: 6-DOF 四旋翼動力學與力矩驗證   \n";
    std::cout << "===============================================================\n";

    Quad6DSimulator sim;

    // 驗證 Case 1: 四馬達完全相等 -> 力矩應為 0，姿態角不變
    std::cout << "\n[Case 1]: 四馬達輸出對稱 (皆為 50%)...\n";
    float symCmd[4] = {0.5f, 0.5f, 0.5f, 0.5f};
    for (int i = 0; i < 50; ++i) sim.step(symCmd, 0.01f);
    std::cout << " -> Roll 角速度:  " << sim.getState().gyro[0] << " rad/s\n";
    std::cout << " -> Pitch 角速度: " << sim.getState().gyro[1] << " rad/s\n";
    std::cout << " -> Yaw 角速度:   " << sim.getState().gyro[2] << " rad/s\n";
    assert(std::abs(sim.getState().gyro[0]) < 1e-5f);
    assert(std::abs(sim.getState().gyro[1]) < 1e-5f);
    assert(std::abs(sim.getState().gyro[2]) < 1e-5f);
    std::cout << "✓ Case 1 通過: 四馬達相等時三軸無淨力矩！\n";

    // 驗證 Case 2: 左側推力大於右側推力 (M2, M3 = 0.7; M1, M4 = 0.3)
    std::cout << "\n[Case 2]: 左大右小 (M2,M3=0.7, M1,M4=0.3) -> 預期向右滾轉 (Roll > 0)...\n";
    sim.reset();
    float rollCmd[4] = {0.3f, 0.7f, 0.7f, 0.3f};
    for (int i = 0; i < 50; ++i) sim.step(rollCmd, 0.01f);
    std::cout << " -> 0.5s 後 Roll 角速度: " << sim.getState().gyro[0] << " rad/s\n";
    std::cout << " -> 0.5s 後 Roll 角度:   " << sim.getState().roll * 57.3f << " 度\n";
    assert(sim.getState().gyro[0] > 0.0f);
    assert(sim.getState().roll > 0.0f);
    std::cout << "✓ Case 2 通過: 滾轉力矩與角加速度方向嚴格符合座標系定義！\n";

    // 驗證 Case 3: 後方推力大於前方推力 (M2, M4 = 0.7; M1, M3 = 0.3)
    std::cout << "\n[Case 3]: 後大前小 (M2,M4=0.7, M1,M3=0.3) -> 預期抬頭 (Pitch > 0)...\n";
    sim.reset();
    float pitchCmd[4] = {0.3f, 0.7f, 0.3f, 0.7f};
    for (int i = 0; i < 50; ++i) sim.step(pitchCmd, 0.01f);
    std::cout << " -> 0.5s 後 Pitch 角速度: " << sim.getState().gyro[1] << " rad/s\n";
    std::cout << " -> 0.5s 後 Pitch 角度:   " << sim.getState().pitch * 57.3f << " 度\n";
    assert(sim.getState().gyro[1] > 0.0f);
    assert(sim.getState().pitch > 0.0f);
    std::cout << "✓ Case 3 通過: 俯仰力矩與運動方向正確！\n";

    std::cout << "\n>>> Stage 2 全部驗收標準達成！ <<<\n";
    return 0;
}
