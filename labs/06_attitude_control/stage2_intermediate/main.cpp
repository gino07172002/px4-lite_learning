/**
 * @file main.cpp
 * @brief Lab 06 - Stage 2 (核心進階版): 互補濾波姿態估計器 (Complementary Filter)
 * 
 * [本階段目標與設計重點]:
 * 1. 感測器特性融合:
 *    - 陀螺儀 Gyro: 高頻動態佳，長期積分累積漂移 (Drift)。
 *    - 加速度計 Accel: 低頻可測量重力向量得到絕對水平角，但受機身震動噪聲污染。
 * 2. 互補濾波演算法:
 *    roll_acc = atan2(ay, -az)
 *    roll_est = alpha * (roll_est + gyro_p * dt) + (1 - alpha) * roll_acc
 * 3. 性能比對驗證:
 *    - 在帶有高斯噪聲與感測器漂移的環境下，比對 Estimated Roll 與 Ground Truth Roll。
 */

#include <iostream>
#include <cmath>
#include <random>
#include <cassert>

class ComplementaryEstimator {
public:
    explicit ComplementaryEstimator(float alpha = 0.98f)
        : _alpha(alpha), _estimatedRoll(0.0f) {}

    // 輸入陀螺儀角速度 (rad/s) 與加速度計 (m/s^2)，以及步長 dt (s)
    void update(float gyroP, float ay, float az, float dt) {
        // 1. 加速度計計算低頻參考角度 (Roll)
        float rollAcc = std::atan2(ay, -az);

        // 2. 陀螺儀預測角度 (高頻積分)
        float rollGyroPred = _estimatedRoll + gyroP * dt;

        // 3. 互補濾波融合
        _estimatedRoll = (_alpha * rollGyroPred) + ((1.0f - _alpha) * rollAcc);
    }

    float getEstimatedRoll() const { return _estimatedRoll; }
    void setInitialRoll(float rollRad) { _estimatedRoll = rollRad; }

private:
    float _alpha;
    float _estimatedRoll{0.0f};
};

int main() {
    std::cout << "===============================================================\n";
    std::cout << " [Lab 06 - Stage 2 核心進階版]: 互補濾波姿態估計器對比測試     \n";
    std::cout << "===============================================================\n";

    constexpr float DEG2RAD = 3.14159265f / 180.0f;
    constexpr float RAD2DEG = 180.0f / 3.14159265f;

    ComplementaryEstimator estimator(0.98f);
    std::mt19937 rng(42);
    std::normal_distribution<float> noise(0.0f, 0.05f); // 噪聲

    float groundTruthRoll = 10.0f * DEG2RAD; // 真值姿態保持 10 度
    estimator.setInitialRoll(0.0f); // 初始估計器從 0 度開始收斂

    float dt = 0.004f; // 250Hz

    std::cout << "真實機身滾轉角: 10.0 度 (加入 Accel/Gyro 高頻振動噪聲與微小漂移)...\n";

    for (int step = 0; step < 500; ++step) { // 模擬 2 秒
        // 模擬感測器讀數
        float gyroP = 0.0f + 0.002f + noise(rng); // 靜止時帶有漂移與噪聲
        float ay = 9.81f * std::sin(groundTruthRoll) + noise(rng) * 2.0f;
        float az = -9.81f * std::cos(groundTruthRoll) + noise(rng) * 2.0f;

        estimator.update(gyroP, ay, az, dt);

        if (step % 100 == 0) {
            std::cout << "t = " << step * dt << " s | 估計姿態 = " 
                      << estimator.getEstimatedRoll() * RAD2DEG << " 度 (真值 10.0 度)\n";
        }
    }

    float finalEstDeg = estimator.getEstimatedRoll() * RAD2DEG;
    std::cout << "\n2.0 秒後最終估計角度: " << finalEstDeg << " 度\n";
    assert(std::abs(finalEstDeg - 10.0f) < 0.6f);
    std::cout << "✓ 互補濾波估計器驗證成功: 成功消除陀螺儀漂移並平滑濾除震動噪聲！\n";

    std::cout << "\n>>> Stage 2 全部驗收標準達成！ <<<\n";
    return 0;
}
