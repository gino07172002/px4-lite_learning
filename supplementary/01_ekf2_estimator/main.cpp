/**
 * @file main.cpp
 * @brief 補充模組 01: EKF2 卡爾曼狀態估計器與異常觀測拒絕 (Innovation Gating)
 * 
 * [對標 PX4 源碼]:
 * - src/modules/ekf2
 * - src/lib/ecl/EKF/ekf.cpp (fuseHeight, innovation check)
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <cassert>

class AltitudeEKF {
public:
    AltitudeEKF() {
        // 初始狀態: [z, vz, accel_bias]^T
        _x[0] = 0.0f; // 高度 z (m)
        _x[1] = 0.0f; // 垂直速度 vz (m/s)
        _x[2] = 0.0f; // 加速度計零偏 (m/s^2)

        // 初始協方差矩陣 P (對角線)
        _P[0][0] = 1.0f; _P[0][1] = 0.0f; _P[0][2] = 0.0f;
        _P[1][0] = 0.0f; _P[1][1] = 1.0f; _P[1][2] = 0.0f;
        _P[2][0] = 0.0f; _P[2][1] = 0.0f; _P[2][2] = 0.01f;

        // 過程噪聲 Q
        _Q_z = 0.01f;
        _Q_v = 0.05f;
        _Q_bias = 0.0001f;

        // 氣壓計觀測噪聲 R
        _R_baro = 0.25f; // (0.5m)^2
    }

    void init(float initialZ) {
        _x[0] = initialZ;
        _x[1] = 0.0f;
        _x[2] = 0.0f;
    }

    // 1. 預測步 (由 IMU 高頻加速度計驅動, 如 100Hz)
    void predict(float accelZ, float dt) {
        float a_corr = accelZ - _x[2]; // 扣除當前估計之零偏

        // 狀態外推
        _x[0] += _x[1] * dt + 0.5f * a_corr * dt * dt;
        _x[1] += a_corr * dt;
        // _x[2] (bias) 保持恆定隨機遊走

        // 協方差外推: P = F * P * F^T + Q
        // F = [ [1, dt, -0.5*dt^2], [0, 1, -dt], [0, 0, 1] ]
        float P00 = _P[0][0] + 2.0f * _P[0][1] * dt + _P[1][1] * dt * dt + _Q_z;
        float P01 = _P[0][1] + _P[1][1] * dt;
        float P11 = _P[1][1] + _Q_v;
        float P22 = _P[2][2] + _Q_bias;

        _P[0][0] = P00;
        _P[0][1] = _P[1][0] = P01;
        _P[1][1] = P11;
        _P[2][2] = P22;
    }

    // 2. 觀測更新步 (由氣壓計/高度計驅動, 如 20Hz), 內建 Innovation Gating
    bool updateBaro(float measuredZ, float gateLimitSigma = 3.0f) {
        // H = [1, 0, 0]
        float predictedZ = _x[0];
        float innovation = measuredZ - predictedZ; // 新息 y

        // 新息協方差 S = H * P * H^T + R = P00 + R
        float S = _P[0][0] + _R_baro;
        if (S <= 0.0f) return false;

        // PX4 核心：創新一致性檢驗 (Innovation Gating)
        float normalizedInnovSq = (innovation * innovation) / S;
        if (normalizedInnovSq > (gateLimitSigma * gateLimitSigma)) {
            std::cout << "[EKF REJECT] 氣壓計異常突波! 測量值=" << measuredZ 
                      << " m, 預測值=" << predictedZ 
                      << " m, 偏差=" << std::sqrt(normalizedInnovSq) << " sigma (> 門限) -> 丟棄！\n";
            _rejectedCount++;
            return false; // 拒絕融合，保護狀態不被污染
        }

        // 計算卡爾曼增益 K = P * H^T / S
        float K[3];
        K[0] = _P[0][0] / S;
        K[1] = _P[1][0] / S;
        K[2] = _P[2][0] / S;

        // 更新狀態
        _x[0] += K[0] * innovation;
        _x[1] += K[1] * innovation;
        _x[2] += K[2] * innovation;

        // 更新協方差 P = (I - K * H) * P
        _P[0][0] -= K[0] * _P[0][0];
        _P[1][1] -= K[1] * _P[0][1];
        _P[2][2] -= K[2] * _P[0][2];

        return true;
    }

    float getAltitude() const { return _x[0]; }
    float getVelocity() const { return _x[1]; }
    float getAccelBias() const { return _x[2]; }
    int getRejectedCount() const { return _rejectedCount; }

private:
    float _x[3];       // [z, vz, bias]
    float _P[3][3];    // 協方差矩陣
    float _Q_z, _Q_v, _Q_bias;
    float _R_baro;
    int _rejectedCount{0};
};

int main() {
    std::cout << "=======================================================================\n";
    std::cout << " [補充模組 01]: EKF2 狀態估計器與野值觀測拒絕 (Innovation Gating)      \n";
    std::cout << "=======================================================================\n";

    AltitudeEKF ekf;
    std::mt19937 rng(42);
    std::normal_distribution<float> baroNoise(0.0f, 0.2f); // 20cm 氣壓計噪聲

    float trueZ = 5.0f; // 飛機穩定懸停在 5.0 公尺
    float dt = 0.01f;   // 100Hz IMU

    // PX4 EKF 初始化: 首次對齊氣壓計基準高度 (resetHeight)
    ekf.init(5.0f);

    std::cout << "系統初始化對齊高度 5.0 公尺，運行 2 秒 (200 個 IMU 週期)...\n\n";

    for (int step = 0; step < 200; ++step) {
        // 1. IMU 預測步 (100Hz)
        ekf.predict(0.0f, dt);

        // 2. 氣壓計更新步 (每 5 步觸發一次 = 20Hz)
        if (step % 5 == 0) {
            float measuredZ = trueZ + baroNoise(rng);

            // 在第 100 步故意注入一個嚴重的氣壓計異常突波 (+12.0m，例如受地面氣流或電磁干擾)
            if (step == 100) {
                std::cout << "\n>>> [故障注入]: 氣壓計遭遇瞬間強氣流衝擊，產生 +12.0m 嚴重突波！ <<<\n";
                measuredZ += 12.0f; // 測量值瞬間飆到 17 米
            }

            ekf.updateBaro(measuredZ);
        }

        if (step % 40 == 0) {
            std::cout << "t = " << step * dt << " s | EKF 估計高度 = " << ekf.getAltitude() 
                      << " m | 垂直速度 = " << ekf.getVelocity() << " m/s\n";
        }
    }

    std::cout << "\n最終估計高度: " << ekf.getAltitude() << " m (真值: 5.0 m)\n";
    std::cout << "累計異常拒絕次數: " << ekf.getRejectedCount() << "\n";

    assert(ekf.getRejectedCount() >= 1);
    assert(std::abs(ekf.getAltitude() - 5.0f) < 0.4f);
    std::cout << "✓ 驗收成功: EKF2 完美攔截異常野值，高度估計平穩不發散！\n";

    return 0;
}
