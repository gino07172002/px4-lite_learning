# Lab 06: Estimator + Attitude Controller (姿態估計器與串級姿態控制器)

## 1. Lab 目標與設計理念

在飛控系統中，控制飛機「姿態角度（角度）」是實現自主飛行的第一步。
PX4 採用經典的**串級控制架構（Cascade Control）**：
- **外環（Outer Loop，姿態角度環，約 50Hz~100Hz）**：比較目標角度與當前估計角度，輸出**期望角速度（Rate Setpoint）**。
- **內環（Inner Loop，角速度環，約 250Hz~1000Hz）**：比較期望角速度與陀螺儀讀數，輸出**控制扭矩（Torque Setpoint）**。

### 系統整體資料流
```
[物理模擬器] ──> IMU (陀螺儀+加計) ──> [姿態估計器 (Estimator)] ──> vehicle_attitude
                                                                          │
 目標姿態 (angle_sp = 0°) ───────────────────────────────────────────────( - )
                                                                          │
                                                                          ▼
                                                         [ 姿態外環 P 控制器 ]
                                                                          │
                                                                          ▼ rate_sp (期望角速度)
                                                         [ 角速度內環 PID 控制器 ] (Lab 05)
                                                                          │
                                                                          ▼ torque (扭矩)
                                                         [ 混控分配器 Mixer ]
                                                                          │
                                                                          ▼ 馬達推力
                                                                     [物理模擬器]
```

---

## 2. 核心原理

### 2A. 姿態外環 P 控制器 (Attitude P Control)
角度誤差 $e_\theta = \theta_{sp} - \theta_{est}$。
期望角速度 $\omega_{sp} = K_p \cdot e_\theta$（限制在最大允許角速度 $\omega_{max}$ 內）。

### 2B. 互補濾波器 (Complementary Filter)
- **陀螺儀（Gyroscope）**：動態響應極快、高頻精確，但存在低頻零偏漂移（Drift）。
- **加速度計（Accelerometer）**：在靜態或平穩飛行下，可測量重力向量以解算傾角 $\phi_{acc} = \arctan2(a_y, -a_z)$，長期無漂移，但極易受高頻震動污染。
- **互補融合公式**：
  $$\theta_{est} = \alpha \cdot (\theta_{est} + \omega_{gyro} \cdot dt) + (1 - \alpha) \cdot \theta_{acc}$$
  其中權重 $\alpha \approx 0.98$。

---

## 3. 三個實現階段說明

### 階段一：基礎版（Stage 1 - 真值估計外環 P 串級控制）
- **原則**：先不加 Estimator，直接使用物理模擬器的真值姿態（Ground Truth），專注驗證外環 P 控制器與內環 PID 的串級協同工作。
- **驗證**：初始給定 Roll = +15°，目標 0°，觀察角度逐階收斂歸零。

### 階段二：核心進階版（Stage 2 - 互補濾波姿態估計器）
- **重點**：融合帶有噪聲與漂移的陀螺儀與加速度計數據。
- **驗證**：在動態俯仰/滾轉過程中，估計角度誤差小於 1.5°，並有效濾除震動噪聲。

### 階段三：PX4 對標進階版（Stage 3 - 完整串級閉迴路自穩驗收）
- **重點**：串接「物理模擬器 + 合成感測器 + 互補濾波器 + 姿態外環 + 角速度內環 + Mixer 混控」。
- **終極驗收標準**：
  - 初始傾斜 +15°。
  - 在 3 秒內自動將機身拉平至 $|roll| < 0.8^\circ$。
