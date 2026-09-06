# Lab 08: Runtime Tools (CLI 命令列、飛行日誌 Logger 與性能計數器 Perf)

## 1. Lab 目標與設計理念

在實務飛控工程與嵌入式系統中，**系統可觀測性（Runtime Observability）** 是排查故障、PID 調參以及保證飛行安全的靈魂工具。
PX4 提供了極其強大的三大診斷工具：
1. **uORB / NSH 互動命令列（CLI Shell）**：
   - 隨時查看主題清單（`topic list`）、監看數據流（`topic echo`）、測量實際更新頻率（`topic hz`）。
   - 即時檢視或調整參數（`param show`, `param set`）。
2. **高速飛行黑盒子日誌（Flight Logger）**：
   - 高頻採樣並寫入 CSV / ULog 格式，記錄各環節 Setpoint、感測器讀數與馬達輸出，供離線回放或 Python 繪圖分析。
3. **性能計數器（Perf Counters）**：
   - 精確追蹤每個 `WorkItem` 的平均執行耗時（avg us）、最大執行耗時（max us）、排程頻率（Hz）以及抖動（Jitter），即時捕捉潛在的即時性危機（Deadline Miss）。

---

## 2. 核心架構與命令規範

### 2A. CLI 互動式終端
```
mini> topic list
mini> topic echo sensor_imu
mini> topic hz sensor_imu
mini> param set MC_ROLL_P 6.5
mini> workqueue status
```

### 2B. 飛行黑盒子日誌格式 (flight.csv)
```csv
timestamp_us,roll_deg,roll_sp_deg,motor1,motor2,motor3,motor4
1000000,15.00,0.00,0.40,0.60,0.60,0.40
1004000,14.85,0.00,0.41,0.59,0.59,0.41
...
```

---

## 3. 三個實現階段說明

### 階段一：基礎版（Stage 1 - 基礎 Topic CLI 與頻率計）
- **重點**：實作命令列解析器，支援 `topic list` 與 `topic hz <name>`。
- **驗證**：建立 250Hz 主題，驗證 CLI 精確輸出 `249.8 Hz`。

### 階段二：核心進階版（Stage 2 - 高頻非同步 Flight Logger & 繪圖工具）
- **重點**：多主題聚合記錄器，以 CSV 格式寫入檔案。
- **產出**：生成 `flight.csv`，並附贈 `plot_log.py` 繪圖腳本。

### 階段三：PX4 對標進階版（Stage 3 - WorkQueue 性能計數器 Perf Counter）
- **重點**：對標 PX4 的 `perf_counter_t`。
- **機制**：
  - 在 WorkItem 執行前後自動取樣時間戳。
  - 計算：累積次數、平均執行時間 (avg us)、峰值執行時間 (max us)、運行頻率。
  - `workqueue status` 輸出清晰的性能健康儀表板。

---

## 4. 驗收標準與測試清單

- [x] CLI `topic hz` 能精確計算出發布頻率（誤差 < 2%）。
- [x] Logger 產生完整無損的 `flight.csv`。
- [x] Perf Counter 能即時反映 WorkItem 的平均與最大耗時微秒數。
