# 補充模組 03: 多 IMU 感測器投票仲裁與無縫故障切換 (Voting & Failover)

## 1. PX4 原始碼對照與設計背景

在工業級飛控硬體（如 Pixhawk 4、Cube Orange、Pixhawk 6X）中，主板上通常同時焊接了 **2 到 3 顆不同型號的獨立實體 IMU 晶片**（例如 InvenSense ICM-42688P + Bosch BMI088）。
對照 PX4 源碼目錄：
- `src/modules/sensors`
- `src/modules/sensors/vehicle_imu`

### 為什麼飛控不能只依賴單一 IMU？
1. **SPI 匯流排干擾或接觸不良**：飛行途中若單一晶片掉線，單 IMU 架構將在 10ms 內炸機。
2. **高頻共振飽和 (Vibration Clipping)**：機身某特定轉速的機械諧振可能使某顆 IMU 陀螺儀飽和，而另一顆不同量程的 IMU 依然完好。
3. **數值卡死 (Sensor Freezing)**：感測器 FIFO 緩衝區鎖死，讀數維持固定不變。

PX4 的 `sensors` 模組負責訂閱多路 `sensor_imu` 實例，執行**一致性交叉比對（Consistency Check）**、**卡死偵測**與**主備熱切換（Seamless Failover）**，最終為後續模組提供單一穩定且具備冗餘保護的 `vehicle_imu`。

---

## 2. 核心容錯仲裁邏輯

```
  [ IMU 0 (Primary) ] ──┐
                        ├──> [ 投票與健康仲裁器 (Voting & Failover) ] ──> vehicle_imu (純淨可靠輸出)
  [ IMU 1 (Secondary) ] ──┘             │
                                        ├── 1. 逾時監控 (Timeout Watchdog > 50ms)
                                        ├── 2. 數值卡死監控 (Frozen Sensor Detection)
                                        └── 3. 雙機分歧監控 (Divergence Check > 0.5 rad/s)
```

1. **主選定原則**：開機自檢合格後，優先選取預設品質最優之 IMU 0 作為 `_selected_imu`。
2. **故障偵測條件**：
   - **Timeout**：超過 50ms 未收到新數據。
   - **Frozen**：連續多次更新陀螺儀數值完全相同（方差為 0）。
   - **Divergence**：主感測器與備用感測器數據出現巨大偏差且主感測器發生異常跳躍。
3. **無縫切換（Failover Action）**：
   - 立即發出告警：`[WARN] Primary IMU 0 failure detected! Seamless failover to IMU 1`。
   - 切換指標，零延遲改取 IMU 1 數據，飛控控制迴路完全無感知！

---

## 3. 實作驗證重點

- [x] 雙 IMU 正常運作時，系統穩定輸出 Primary IMU 數據。
- [x] 飛行途中人為切斷 IMU 0 訊號（模擬硬體掉線故障）。
- [x] 仲裁器在 50ms 內迅速判定故障，自動無縫熱切換至 IMU 1，無斷流或崩潰。
