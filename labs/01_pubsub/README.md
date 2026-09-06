# Lab 01: uORB-lite / Pub-Sub 訊息發布訂閱中介軟體

## 1. Lab 目標與設計理念

在飛控系統（如 PX4 Autopilot）中，飛行姿態估計器（Estimator）、姿態控制器（Attitude Controller）、馬達混控器（Mixer）以及各類感測器驅動（IMU, Baro, GPS）彼此之間需要大量的資料交換。
如果採用傳統的「物件直接呼叫（Direct Call）」：
```
SensorDriver ──(呼叫)──> Controller ──(呼叫)──> MotorActuator
```
這會導致模組間強耦合、執行緒優先級錯亂，且感測器更新頻率（如 1000Hz）與控制循環（如 250Hz）難以解耦。

PX4 採用 **uORB (Micro Object Request Broker)** 實現**進程內/非同步 Pub-Sub 機制**：
- **發布者（Publisher）** 只管將資料寫入對應的 Topic，不知道也不在乎有多少人讀取。
- **訂閱者（Subscriber）** 擁有獨立的讀取指標，可以隨時主動輪詢（Polling）或非同步等待新資料。
- 核心特點：**以最後一筆有效資料（Latest Value）為核心**、**低延遲、無阻塞（Non-blocking）、無鎖/少鎖設計**。

---

## 2. 核心架構與 API 規範

### 核心類別關係
```
                ┌───────────────────────────────────┐
                │             Topic<T>              │
                │ ───────────────────────────────── │
                │ - data: T                         │
                │ - generation: uint64_t            │
                │ - mutex / atomic                  │
                └───────────────────────────────────┘
                     ▲                         ▲
                     │ publish(&)              │ copy() / updated()
        ┌────────────┴──────┐       ┌──────────┴────────┐
        │   Publisher<T>    │       │  Subscription<T>  │
        └───────────────────┘       └───────────────────┘
```

### 必備 API
- `publish(const T& data)`: 發布新資料，更新資料載體並將 `generation` 計數器加 1。
- `updated()`: 檢查當前 Subscription 的 `last_generation` 是否落後於 Topic 的 `generation`。
- `copy(T& out)`: 在互斥鎖保護下**原子性地**複製資料與對應的 generation，避免數據競爭與世代不匹配。

### 核心資料結構範例（感測器與控制資料）
```cpp
struct SensorImu {
    uint64_t timestamp_us;
    float gyro[3];   // rad/s (Roll, Pitch, Yaw)
    float accel[3];  // m/s^2 (X, Y, Z)
    float temperature;
};
```

---

## 3. 三個實現階段說明

### 階段一：基礎版（Stage 1 - Basic Generic PubSub）
- **重點**：掌握 Template 泛型化、單緩衝區、Mutex 臨界區保護。
- **修正重點**：解決多執行緒下 `getData()` 與 `getGeneration()` 分開取得造成的撕裂問題（Torn Read），保證資料與 generation 在同一個 Lock 下獲取。
- **目標**：實現單 Publisher 與多 Subscriber 的解耦通訊。

### 階段二：核心進階版（Stage 2 - TopicBus & Multithreading）
- **重點**：引入全域主題匯流排（TopicBus / Manager 單例模式）。
- **機制**：模組不需要持有彼此的指標或物件參照，僅需透過字串名稱（如 `"sensor_imu"`）向匯流排註冊或訂閱。
- **多執行緒驗證**：建立 250Hz Fake IMU 執行緒與兩個異步讀取執行緒，驗證 Thread-safety。

### 階段三：PX4 對標進階版（Stage 3 - Lock-Free Generation & Drop Detection）
- **重點**：對標 PX4 uORB 核心機制。
- **機制**：
  - 利用 `std::atomic<uint64_t>` 世代計數器與雙緩衝/讀寫世代鎖，實現超低延遲讀取。
  - 遺失封包偵測（Drop / Loss Detection）：訂閱者透過 `current_gen - last_gen - 1` 精確統計因消費者速度落後而錯過的幀數。
  - 壓測與基準測試（Throughput & Latency Benchmark）。

---

## 4. 驗收標準與測試清單

- [x] 單發布者對單訂閱者測試通過。
- [x] 單發布者對多訂閱者測試通過（每個訂閱者獨立維護其讀取 generation）。
- [x] generation 計數器單調遞增且無數據競爭。
- [x] `updated()` 能精確反映是否有未讀新資料（連續呼叫兩次第二次應回傳 false）。
- [x] 多執行緒並發寫入與讀取時，不發生崩潰或撕裂數據（Torn Read）。
