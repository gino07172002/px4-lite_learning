# 補充模組 01: EKF2 卡爾曼狀態估計器與異常觀測拒絕 (Innovation Gating)

## 1. PX4 原始碼對照與設計背景

在 PX4 官方程式碼中，`src/modules/ekf2`（依賴 `src/lib/ecl/EKF`）是整套飛行系統中最龐大、最重要的數學演算法核心。
雖然 Lab 06 介紹了互補濾波（Complementary Filter），但在實際惡劣的飛行環境中，感測器會面臨三大挑戰：
1. **感測器噪聲統計特性不同**：IMU 高頻噪聲、氣壓計低頻噪聲、GPS 隨機跳變。
2. **未知零偏（Sensor Bias）**：加速度計與陀螺儀開機零偏隨溫度漂移。
3. **野值干擾（Outliers / Glitches）**：氣壓計受地面效應氣流衝擊產生 5 米瞬間突波、GPS 穿過高樓受多路徑效應（Multipath）發生瞬間階躍。

EKF（擴展卡爾曼濾波器）透過**狀態協方差矩陣 $P$** 進行不確定度傳播，並利用**新息檢驗門限（Innovation Gating）** 達成強大的抗干擾能力。

---

## 2. 核心數學原理

### 2A. 狀態預測 (Prediction Step - 由高頻 IMU 驅動)
狀態向量定義：$x = \begin{bmatrix} z \\ v_z \\ b_a \end{bmatrix}$（高度、垂直速度、加速度計零偏）
- 狀態轉移矩陣：
  $$x_{k|k-1} = F x_{k-1} + B (a_{meas} - b_a)$$
- 協方差外推：
  $$P_{k|k-1} = F P_{k-1} F^T + Q$$

### 2B. 觀測更新與新息拒絕 (Update Step & Innovation Gating)
當低頻感測器（如氣壓計 / GPS）收到觀測值 $z_{meas}$ 時：
1. 計算**新息 (Innovation / 測量殘差)**：
   $$y = z_{meas} - H \hat{x}$$
2. 計算新息協方差：
   $$S = H P H^T + R$$
3. **PX4 核心守護門限 (Innovation Consistency Check)**：
   $$D^2 = \frac{y^2}{S}$$
   若 $D^2 > \text{Gate}^2$（例如大於 $3^2 = 9$），判定為**感測器突發野值（Glitch / Spurious Measurement）**，直接丟棄不進行卡爾曼增益更新，保護飛機不被帶入危險俯衝！
4. 正常情況下更新狀態與協方差：
   $$K = P H^T S^{-1}, \quad \hat{x} = \hat{x} + K y, \quad P = (I - K H) P$$

---

## 3. 實作驗證重點

- [x] 在常態白噪聲環境下，卡爾曼濾波能穩定追蹤飛行高度與速度。
- [x] 在系統穩定懸停時，故意注入 +10.0m 的瞬間氣壓計突波。
- [x] 成功觸發 `INNOVATION REJECTION`，狀態估計維持平穩，完全不受突波干擾。
