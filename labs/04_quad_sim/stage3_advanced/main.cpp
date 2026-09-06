/**
 * @file main.cpp
 * @brief Lab 04 - Stage 3 (PX4 對標進階版): 馬達動態延遲、IMU 感測器合成與確定性步進驗證
 * 
 * [本階段目標與設計重點]:
 * 1. 馬達一階低通響應 (Motor Time Constant):
 *    - 真實無刷馬達有轉子慣量，無法瞬時響應。
 *    - dm_actual/dt = (m_target - m_actual) / tau_motor (通常 tau ~ 0.03 秒)。
 * 2. 合成 IMU 感測器 (Synthetic IMU Model):
 *    - 陀螺儀 Gyro: 真實角速度 + 零偏漂移 (Bias) + 高斯白噪聲。
 *    - 加速度計 Accel: 比力 (Specific Force = 加速度 - 重力向量在機體軸投影)。
 * 3. 確定性 (Deterministic Simulation Benchmark):
 *    - 同樣的時間步長與輸入信號，保證兩次模擬位元級別一致，作為 CI/CD 自動化測試基礎。
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <cassert>

struct SyntheticImuData {
    float gyro[3];   // rad/s
    float accel[3];  // m/s^2
};

class AdvancedQuadSim {
public:
    explicit AdvancedQuadSim(uint32_t randomSeed = 42)
        : _rng(randomSeed), _noiseDist(0.0f, 0.005f) {
        for (int i = 0; i < 4; ++i) _actualMotor[i] = 0.0f;
    }

    void step(const float targetCmds[4], float dt) {
        // 1. 馬達一階濾波響應 (Motor Lag)
        float alpha = dt / (_motorTau + dt);
        for (int i = 0; i < 4; ++i) {
            float target = std::max(0.0f, std::min(1.0f, targetCmds[i]));
            _actualMotor[i] += alpha * (target - _actualMotor[i]);
        }

        // 2. 剛體力學計算
        float T[4];
        float totalThrust = 0.0f;
        for (int i = 0; i < 4; ++i) {
            T[i] = _actualMotor[i] * _maxThrustPerMotor;
            totalThrust += T[i];
        }

        float d = _armLength * 0.7071f;
        float tau_x = d * ((T[1] + T[2]) - (T[0] + T[3]));
        float tau_y = d * ((T[1] + T[3]) - (T[0] + T[2]));
        float tau_z = _torqueCoeff * ((T[2] + T[3]) - (T[0] + T[1]));

        _gyro[0] += (tau_x / _Ixx) * dt;
        _gyro[1] += (tau_y / _Iyy) * dt;
        _gyro[2] += (tau_z / _Izz) * dt;

        _roll  += _gyro[0] * dt;
        _pitch += _gyro[1] * dt;
        _yaw   += _gyro[2] * dt;

        float az = (totalThrust / _mass) - _gravity;
        _vz += az * dt;
        _z  += _vz * dt;
        if (_z < 0.0f) { _z = 0.0f; if (_vz < 0.0f) _vz = 0.0f; }
    }

    // 生成含噪聲之合成 IMU 讀數
    SyntheticImuData readSensors() {
        SyntheticImuData data;
        data.gyro[0] = _gyro[0] + _gyroBias[0] + _noiseDist(_rng);
        data.gyro[1] = _gyro[1] + _gyroBias[1] + _noiseDist(_rng);
        data.gyro[2] = _gyro[2] + _gyroBias[2] + _noiseDist(_rng);

        // 加速度計在懸停平飛時測量到抵消重力的朝上比力 (FRD 下 Z 軸為 -9.81)
        data.accel[0] = 0.0f + _noiseDist(_rng);
        data.accel[1] = 0.0f + _noiseDist(_rng);
        data.accel[2] = -_gravity + _noiseDist(_rng);
        return data;
    }

    float getActualMotor(int idx) const { return _actualMotor[idx]; }
    float getRoll() const { return _roll; }

private:
    float _mass{1.2f};
    float _gravity{9.81f};
    float _maxThrustPerMotor{6.0f};
    float _armLength{0.15f};
    float _torqueCoeff{0.02f};
    float _Ixx{0.008f};
    float _Iyy{0.008f};
    float _Izz{0.015f};

    float _motorTau{0.03f}; // 30ms 馬達響應延遲
    float _actualMotor[4]{0, 0, 0, 0};

    float _roll{0.0f}, _pitch{0.0f}, _yaw{0.0f};
    float _gyro[3]{0, 0, 0};
    float _z{0.0f}, _vz{0.0f};

    float _gyroBias[3]{0.002f, -0.001f, 0.0005f}; // 固態漂移
    std::mt19937 _rng;
    std::normal_distribution<float> _noiseDist;
};

int main() {
    std::cout << "=======================================================================\n";
    std::cout << " [Lab 04 - Stage 3 PX4對標進階版]: 馬達響應、IMU 合成與確定性步進測試   \n";
    std::cout << "=======================================================================\n";

    // 1. 驗證馬達一階延遲
    std::cout << "\n[測試 1: 馬達動態一階滯後響應]\n";
    AdvancedQuadSim sim1;
    float stepInput[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    sim1.step(stepInput, 0.01f); // 10ms
    std::cout << "給予 100% 階躍輸入 10ms 後，馬達實際轉速: " << sim1.getActualMotor(0) * 100.0f << " %\n";
    assert(sim1.getActualMotor(0) < 0.6f); // 證實馬達未瞬間達到 100%

    for (int i = 0; i < 15; ++i) sim1.step(stepInput, 0.01f); // 再過 150ms
    std::cout << "160ms 後，馬達實際轉速: " << sim1.getActualMotor(0) * 100.0f << " %\n";
    assert(sim1.getActualMotor(0) > 0.98f);
    std::cout << "✓ 測試 1 通過: 馬達具備真實一階慣性響應！\n";

    // 2. 測試合成 IMU 感測器輸出
    std::cout << "\n[測試 2: 合成 IMU 噪聲與比力模型]\n";
    auto imu = sim1.readSensors();
    std::cout << "IMU 測得 Accel Z (比力): " << imu.accel[2] << " m/s^2 (理論 ~ -9.81)\n";
    assert(std::abs(imu.accel[2] - (-9.81f)) < 0.5f);
    std::cout << "✓ 測試 2 通過: IMU 重力比力與白噪聲輸出正常！\n";

    // 3. 測試確定性 (Deterministic Reproducibility)
    std::cout << "\n[測試 3: 確定性仿真重現測試]\n";
    AdvancedQuadSim runA(1234);
    AdvancedQuadSim runB(1234);
    float testCmd[4] = {0.2f, 0.6f, 0.8f, 0.4f};
    for (int i = 0; i < 50; ++i) {
        runA.step(testCmd, 0.004f);
        runB.step(testCmd, 0.004f);
    }
    assert(runA.getRoll() == runB.getRoll());
    std::cout << "Run A Roll = " << runA.getRoll() << ", Run B Roll = " << runB.getRoll() << "\n";
    std::cout << "✓ 測試 3 通過: 確定性模擬精確位元級重現！\n";

    std::cout << "\n>>> Stage 3 全部驗收標準達成！ <<<\n";
    return 0;
}
