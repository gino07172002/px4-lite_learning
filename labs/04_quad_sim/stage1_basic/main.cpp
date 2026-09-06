/**
 * @file main.cpp
 * @brief Lab 04 - Stage 1 (基礎版): 1-DOF 垂直升降動力學模擬器
 * 
 * [本階段目標與設計重點]:
 * 1. 物理狀態: 高度 z 與垂直速度 vz。
 * 2. 推力計算: 4 顆馬達油門 (0.0~1.0) 轉換為總推力。
 * 3. 牛頓第二定律: a_z = (Total_Thrust - m * g) / m。
 * 4. 嚴禁任何控制器 (PID)，純驗證受控體物理真實性。
 */

#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>

struct Vehicle1DState {
    float z{0.0f};   // 高度 (m)
    float vz{0.0f};  // 垂直速度 (m/s)
    float az{0.0f};  // 垂直加速度 (m/s^2)
};

class SimpleAltitudeSim {
public:
    SimpleAltitudeSim(float massKg = 1.0f, float maxThrustPerMotorN = 5.0f)
        : _mass(massKg), _maxThrustPerMotor(maxThrustPerMotorN) {}

    // 輸入 4 顆馬達油門 (0.0 ~ 1.0)，步進 dt 秒
    void step(const float motorCommands[4], float dt) {
        float totalThrust = 0.0f;
        for (int i = 0; i < 4; ++i) {
            float cmd = std::max(0.0f, std::min(1.0f, motorCommands[i]));
            totalThrust += cmd * _maxThrustPerMotor;
        }

        float netForce = totalThrust - (_mass * _gravity);
        _state.az = netForce / _mass;
        _state.vz += _state.az * dt;
        _state.z  += _state.vz * dt;

        // 地面碰撞保護
        if (_state.z < 0.0f) {
            _state.z = 0.0f;
            if (_state.vz < 0.0f) _state.vz = 0.0f;
        }
    }

    const Vehicle1DState& getState() const { return _state; }
    float getHoverThrottle() const { return (_mass * _gravity) / (4.0f * _maxThrustPerMotor); }

private:
    float _mass{1.0f};
    float _gravity{9.81f};
    float _maxThrustPerMotor{5.0f};
    Vehicle1DState _state;
};

int main() {
    std::cout << "========================================================\n";
    std::cout << " [Lab 04 - Stage 1 基礎版]: 1-DOF 垂直動力學驗證         \n";
    std::cout << "========================================================\n";

    SimpleAltitudeSim sim(1.0f, 5.0f); // 1kg 機身，4x5N = 20N 最大推力
    float hoverCmd = sim.getHoverThrottle();
    std::cout << "機身質量: 1.0 kg, 理論懸停油門: " << hoverCmd * 100.0f << " %\n";

    // 測試 1: 大油門上升測試 (油門 70% > 懸停 49%)
    float climbThrottle[4] = {0.7f, 0.7f, 0.7f, 0.7f};
    for (int i = 0; i < 100; ++i) { // 步進 1.0 秒 (dt = 0.01s)
        sim.step(climbThrottle, 0.01f);
    }
    std::cout << "油門 70% 運行 1 秒後: 高度 z = " << sim.getState().z 
              << " m, 速度 vz = " << sim.getState().vz << " m/s\n";
    assert(sim.getState().vz > 0.0f);
    assert(sim.getState().z > 0.0f);
    std::cout << "✓ 測試 1 通過: 推力 > 重力時穩定加速上升！\n";

    // 測試 2: 怠速墜落測試 (油門 10% < 懸停 49%)
    float fallThrottle[4] = {0.1f, 0.1f, 0.1f, 0.1f};
    for (int i = 0; i < 100; ++i) {
        sim.step(fallThrottle, 0.01f);
    }
    std::cout << "油門 10% 運行 1 秒後: 速度 vz = " << sim.getState().vz << " m/s (垂直速度下降)\n";
    assert(sim.getState().vz < 3.0f); // 速度受重力下拉
    std::cout << "✓ 測試 2 通過: 推力 < 重力時垂直減速/下墜！\n";

    std::cout << "\n>>> Stage 1 全部驗收標準達成！ <<<\n";
    return 0;
}
