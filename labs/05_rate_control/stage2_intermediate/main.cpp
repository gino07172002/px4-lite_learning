/**
 * @file main.cpp
 * @brief Lab 05 - Stage 2 (核心進階版): 工業級 PID 控制器 (抗積分飽和、低通濾波與魯棒防護)
 * 
 * [本階段目標與設計重點]:
 * 1. 抗積分飽和 (Anti-Windup):
 *    - 積分項設有硬限制 (I-clamp)，防止大誤差持續時積分器風吹飽和，造成巨大超調。
 * 2. 微分項低通濾波 (Derivative LPF):
 *    - 陀螺儀高頻噪聲直接微分會產生高頻抖動燒毀電調，使用一階低通濾波衰減高頻雜訊。
 * 3. 輸出飽和限制 (Output Saturation Limit):
 *    - 輸出扭矩限制在機身實體能提供的 [-maxTorque, +maxTorque] 範圍內。
 * 4. 異常數據攔截:
 *    - dt 異常 (<=0 或 >0.1s) 攔截。
 *    - NaN 或 Inf 數值攔截，避免整機崩潰。
 */

#include <iostream>
#include <cmath>
#include <cassert>
#include <limits>

class RobustRatePID {
public:
    RobustRatePID(float kp, float ki, float kd, float iLimit, float outLimit, float dFilterCutoffHz = 30.0f)
        : _kp(kp), _ki(ki), _kd(kd), _iLimit(iLimit), _outLimit(outLimit), _cutoffHz(dFilterCutoffHz) {}

    float update(float rateSp, float rateActual, float dt) {
        // 1. 異常防護: NaN / Inf 檢查
        if (std::isnan(rateSp) || std::isnan(rateActual) || std::isinf(rateSp) || std::isinf(rateActual)) {
            std::cerr << "[PID WARN] 偵測到 NaN 或 Inf，安全抑制輸出為 0！\n";
            return 0.0f;
        }

        // 2. dt 有效性檢查 (介於 0.0005s ~ 0.1s 之間)
        if (dt < 0.0005f || dt > 0.1f) {
            std::cerr << "[PID WARN] 異常時間步長 dt = " << dt << " s, 跳過本輪積分！\n";
            return 0.0f;
        }

        float error = rateSp - rateActual;

        // 3. 比例項 (P)
        float pTerm = _kp * error;

        // 4. 積分項 (I) 與 Anti-Windup Clamping
        _integral += error * dt;
        if (_integral > _iLimit) _integral = _iLimit;
        else if (_integral < -_iLimit) _integral = -_iLimit;
        float iTerm = _ki * _integral;

        // 5. 微分項 (D) 與一階低通濾波器
        float rawDerivative = (error - _prevError) / dt;
        _prevError = error;

        float rc = 1.0f / (2.0f * 3.14159265f * _cutoffHz);
        float alpha = dt / (rc + dt);
        _filteredDerivative += alpha * (rawDerivative - _filteredDerivative);
        float dTerm = _kd * _filteredDerivative;

        // 6. 總和並進行輸出飽和截斷 (Saturation Limit)
        float totalTorque = pTerm + iTerm + dTerm;
        if (totalTorque > _outLimit) totalTorque = _outLimit;
        else if (totalTorque < -_outLimit) totalTorque = -_outLimit;

        return totalTorque;
    }

    float getIntegral() const { return _integral; }

private:
    float _kp;
    float _ki;
    float _kd;
    float _iLimit;
    float _outLimit;
    float _cutoffHz;

    float _integral{0.0f};
    float _prevError{0.0f};
    float _filteredDerivative{0.0f};
};

int main() {
    std::cout << "===============================================================\n";
    std::cout << " [Lab 05 - Stage 2 核心進階版]: 工業級 PID 防護與濾波特性驗證   \n";
    std::cout << "===============================================================\n";

    // P=0.5, I=0.2, D=0.01, I_Limit=0.5, Out_Limit=0.8
    RobustRatePID pid(0.5f, 0.2f, 0.01f, 0.5f, 0.8f);

    // 1. 驗證 Anti-Windup 積分限制
    std::cout << "\n[測試 1: 驗證 Anti-Windup 積分上限]\n";
    for (int i = 0; i < 200; ++i) {
        pid.update(10.0f, 0.0f, 0.01f); // 大誤差持續 2 秒
    }
    std::cout << " -> 長時間大誤差後積分項: " << pid.getIntegral() << " (設定上限 0.5)\n";
    assert(pid.getIntegral() <= 0.5001f);
    std::cout << "✓ 測試 1 通過: 積分成功被箝位 (Clamped)，防止風吹失控！\n";

    // 2. 驗證輸出飽和限幅 (Saturation)
    std::cout << "\n[測試 2: 輸出飽和限制]\n";
    float saturatedOut = pid.update(100.0f, 0.0f, 0.01f);
    std::cout << " -> 巨大階躍輸出扭矩: " << saturatedOut << " Nm (設定上限 0.8)\n";
    assert(saturatedOut <= 0.8001f);
    std::cout << "✓ 測試 2 通過: 輸出扭矩安全限制在機身物理極限內！\n";

    // 3. 驗證 NaN 異常數據輸入防護
    std::cout << "\n[測試 3: NaN 輸入安全性防護]\n";
    float nanVal = std::numeric_limits<float>::quiet_NaN();
    float safeOut = pid.update(nanVal, 0.0f, 0.01f);
    assert(safeOut == 0.0f);
    std::cout << "✓ 測試 3 通過: 成功捕獲 NaN 並返回 0，系統未發生浮點崩潰！\n";

    std::cout << "\n>>> Stage 2 全部驗收標準達成！ <<<\n";
    return 0;
}
