# PX4 原始碼深度架構補充庫 (PX4 Architecture Supplementary Modules)

本目錄作為 **PX4-Lite** 核心 8 個 Lab 的進階補充。在深入研讀官方最新 **PX4 Autopilot (PX4-Autopilot Source Code)** 後，我們提煉出真實飛控系統中**極度關鍵、在工業界與開源生態中不可或缺，但在基礎教學中常被簡化的 5 大核心深水區架構**：

---

## 5 大補充架構對照表

| 補充模組 | PX4 原始碼對應目錄 | 核心架構價值與工程意義 |
| :--- | :--- | :--- |
| **01. EKF2 狀態估計器** | `src/modules/ekf2` / `src/lib/ecl/EKF` | 飛控界黃金標準。超越互補濾波，利用協方差矩陣 $P$ 動態預測，並透過**新息檢驗 (Innovation Gating)** 剔除異常 GPS/氣壓計突波。 |
| **02. 3D 位置與速度控制器** | `src/modules/mc_pos_control` | 實現自主懸停（Position Hold）與航點飛行。將 3D 位置 P 與速度 PID 誤差，經幾何投影解算為**機身傾斜向量 (Tilt Vector)** 與油門推力。 |
| **03. 多感測器仲裁與故障容錯** | `src/modules/sensors` / `vehicle_imu` | 真實 Pixhawk 飛控配備 2~3 顆 IMU。實作**多 IMU 投票仲裁 (Voting)**、感測器分歧檢測 (Divergence) 與**無縫故障切換 (Failover)**。 |
| **04. 動態控制分配與飽和退飽和** | `src/modules/control_allocator` | 取代舊版 `.mix` 檔案。當強風或激烈機動導致馬達油門超過 100% 飽和時，依**優先級保護 (姿態優先於高度與偏航)** 動態消飽和 (Desaturation)。 |
| **05. MAVLink 2.0 微通訊引擎** | `src/modules/mavlink` | 與地面站 (QGroundControl) 及伴隨電腦 (ROS 2 / MAVROS) 溝通之基石。解析封包校驗碼 (CRC Extra)、心跳包與 `COMMAND_LONG` 互動。 |

---

## 目錄結構與詳細導讀

```text
supplementary/
├── README.md                   # 本文件 (PX4 源碼架構全景與導讀手冊)
├── CMakeLists.txt              # 補充庫 CMake 配置檔
│
├── 01_ekf2_estimator/          # [模組 1]: 擴展卡爾曼濾波 (EKF2) 與新息門限檢驗
│   ├── README.md               # 數學公式、協方差傳播與濾波器推導
│   └── main.cpp                # 完整 C++17 實作與異常噪聲剔除單元測試
│
├── 02_position_control/        # [模組 2]: 3D 位置外環與速度內環串級控制器
│   ├── README.md               # 期望加速度投影至機體姿態向量的幾何數學
│   └── main.cpp                # 完整 3D 空間懸停與步階位置追隨測試
│
├── 03_sensor_voting_failover/  # [模組 3]: 雙/三 IMU 投票仲裁與熱備份容錯
│   ├── README.md               # 實體飛控多硬體感測器容錯架構
│   └── main.cpp                # 模擬飛行中主 IMU 斷線/卡死並自動切換備用 IMU
│
├── 04_control_allocator/       # [模組 4]: 矩陣控制分配器與優先級消飽和
│   ├── README.md               # 偽逆矩陣分配與姿態優先級消飽和策略
│   └── main.cpp                # 驗證馬達飽和時優先確保 Roll/Pitch 自穩不翻機
│
└── 05_mavlink_bridge/          # [模組 5]: MAVLink 2.0 序列化/反序列化與狀態機
    ├── README.md               # MAVLink 封包結構、CRC-16-MCRF4XX 與 QGC 對接
    └── main.cpp                # 模擬與 QGroundControl 雙向握手、解鎖與參數請求
```

---

## 建置與執行方式

本補充庫已整合至全專案根 `CMakeLists.txt`，亦可獨立建置：

```powershell
# 於專案根目錄編譯
cmake -B build -G "Ninja"
cmake --build build

# 執行 EKF2 狀態估計測試
.\build\supplementary\01_ekf2_estimator\supp01_ekf2.exe

# 執行 3D 位置控制空間懸停測試
.\build\supplementary\02_position_control\supp02_pos_ctrl.exe

# 執行 多 IMU 故障無縫熱切換測試
.\build\supplementary\03_sensor_voting_failover\supp03_voting.exe

# 執行 控制分配器與姿態優先消飽和測試
.\build\supplementary\04_control_allocator\supp04_allocator.exe

# 執行 MAVLink 地面站通訊協定測試
.\build\supplementary\05_mavlink_bridge\supp05_mavlink.exe
```
