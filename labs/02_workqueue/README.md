# Lab 02: WorkQueue / 協作式任務排程器 (PX4 Scheduler)

## 1. Lab 目標與設計理念

在傳統的多工嵌入式系統中，若為每個模組（IMU驅動、GPS解析、磁力計、姿態估計、角速度控制、航行日誌）各開一條 OS Thread，將面臨兩大災難：
1. **執行緒上下文切換（Context Switch）開銷過大**：在 MCU（如 STM32）上會耗盡 CPU 時間與記憶體 Stack。
2. **優先級反轉與同步鎖競爭**：幾十條 Thread 互相競爭鎖容易造成抖動（Jitter）與死鎖。

PX4 採用 **WorkQueue（工作佇列）** 機制：
- **只建立少數幾條（通常 2~4 條）常駐的工作執行緒（Worker Threads）**，例如 `wq:rate_ctrl`（高頻即時）、`wq:lp_default`（低頻任務）。
- 模組繼承 `WorkItem`，透過協作式非同步排程（Cooperative Scheduling），由少數 Worker Thread 依序執行任務回呼（`Run()`）。

---

## 2. 核心架構與 API 規範

```
                        Worker Thread (e.g. wq:high)
                                     │
                 ┌───────────────────┼───────────────────┐
                 ▼                   ▼                   ▼
           FastRateControl       ImuFilter           AttitudeLoop
              (250 Hz)            (250 Hz)             (100 Hz)
```

### 核心類別
- `WorkItem`: 模組基底介面，包含純虛擬函式 `virtual void Run() = 0;`。
- `WorkQueue`: 執行緒與任務佇列實體，內部維護時間排序佇列（Priority Queue / Min-Heap）。
- 排程控制 API：
  - `ScheduleNow()`: 立即將工作放入佇列執行。
  - `ScheduleDelayed(uint64_t delay_us)`: 延遲特定微秒後執行。
  - `ScheduleOnInterval(uint64_t interval_us)`: 以固定週期重複執行。
  - `ScheduleClear()`: 取消未來的排程。

---

## 3. 三個實現階段說明

### 階段一：基礎版（Stage 1 - Basic WorkQueue）
- **重點**：掌握單一 Worker 執行緒、條件變數（`std::condition_variable`）與定時小頂堆（Min-Heap Priority Queue）。
- **目標**：支援 `ScheduleNow` 與 `ScheduleDelayed`，能在單一執行緒中無偏誤地依到期時間執行多項任務。

### 階段二：核心進階版（Stage 2 - Multi-Priority Queues & Periodic Tasks）
- **重點**：對標 PX4 的多優先級工作佇列架構。
- **機制**：
  - 建立 `HIGH` (高頻即時) 與 `LOW` (低頻輔助) 兩條獨立的 Worker 執行緒。
  - 支援 `ScheduleOnInterval(interval_us)` 自動週期循環排程。
  - 驗證：2 條 Thread 同時服務 4 個不同頻率任務（250Hz, 100Hz, 20Hz, 10Hz），統計理論執行次數 vs 實際執行次數。

### 階段三：PX4 對標進階版（Stage 3 - Overrun Detection & Blocking Analysis）
- **重點**：物件導向 `WorkItem` 自排程 + 執行超時與阻塞防護。
- **機制**：
  - 模組繼承 `WorkItem`，可直接呼叫 `this->ScheduleOnInterval()`。
  - 統計各任務單次執行耗時（Execution Time）、延遲抖動（Jitter）。
  - **阻塞實驗（Blocking Experiment）**：故意模擬某任務發生阻塞（如 `sleep` 100ms），即時印出超時警報（Overrun Warning），並觀察對同佇列其他任務的連帶影響。

---

## 4. 驗收標準與測試清單

- [x] 雙執行緒模型下，可正常調度 4 個不同週期工作。
- [x] 在 2 秒測試週期內，各任務實際執行頻率誤差在合理範圍（< 5%）。
- [x] 線程數為 2，而非每個任務一條線程。
- [x] 逾時或長耗時任務能被精準檢測並觸發 Overrun 警告。
