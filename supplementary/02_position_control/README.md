# 補充模組 02: 3D 位置與速度控制器 (Multicopter Position Control)

## 1. PX4 原始碼對照與設計背景

在 PX4 官方程式碼中，`src/modules/mc_pos_control` 負責實現無人機的「定點懸停（Position Hold）」與「自主航點飛行（Mission / Offboard）」。
在 Lab 05 與 Lab 06 中，我們只控制了角速度與姿態角度。但如果無人機要在空中停在座標 $(X, Y, Z) = (0, 0, 2\text{m})$，它是如何達成這項任務的？

答案就是**「位置 ➔ 速度 ➔ 期望加速度 ➔ 姿態角與推力」**的幾何解算鏈：
```
目標位置 (pos_sp)
      │
      ▼  [ 位置外環 P 控制器 ]
期望速度 (vel_sp)
      │
      ▼  [ 速度內環 PID 控制器 ]
期望加速度向量 a_des = [ax, ay, az + g]
      │
      ▼  [ 幾何姿態投影 Geometric Projection ]
輸出: 目標滾轉角 (roll_sp) + 目標俯仰角 (pitch_sp) + 總推力 (thrust_sp)
      │
      ▼
送往 Lab 06 (Attitude Controller) 執行姿態自穩！
```

---

## 2. 幾何投影解算推導 (Geometric Attitude & Thrust Generation)

在三維空間中（以 ENU 座標為例，X:東, Y:北, Z:天）：
1. 為了抵消重力並產生水平加速度，總期望加速度向量為：
   $$\vec{a}_{des} = \text{PID}_{vel}(\vec{v}_{sp} - \vec{v}) + \begin{bmatrix} 0 \\ 0 \\ g \end{bmatrix}$$
2. 機身必須將螺旋槳垂直推力軸（機體 Z 軸 $\vec{z}_B$）旋轉對齊 $\vec{a}_{des}$：
   $$\vec{z}_B = \frac{\vec{a}_{des}}{\|\vec{a}_{des}\|}$$
3. 從旋轉向量解算出期望的姿態角（小角度線性化或四元數分解）：
   $$\text{pitch}_{sp} \approx -\frac{a_{x, des}}{g}, \quad \text{roll}_{sp} \approx \frac{a_{y, des}}{g}$$
4. 總推力設定值（包含傾斜後升力補償）：
   $$T_{norm} = \frac{m \cdot \|\vec{a}_{des}\|}{T_{max}}$$

---

## 3. 實作驗證重點

- [x] 水平位置偏移時，能正確計算出向目標方向傾斜的 Roll/Pitch 姿態命令。
- [x] 高度誤差時，能正確增減垂直油門推力。
- [x] 成功將 3D 位置導航層與 Lab 06 姿態控制層無縫對接。
