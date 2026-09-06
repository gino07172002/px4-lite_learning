# Lab 03: Clock + Parameters (時間抽象與參數管理系統)

## 1. Lab 目標與設計理念

在飛控軟體架構中，有兩項關鍵基礎設施直接決定了系統的「可模擬性（Simulatability）」與「可調校性（Tunability）」：
1. **時間抽象（Clock Abstraction）**：
   - 在實體無人機硬體上，系統使用硬體定時器（HRT, High Resolution Timer）或系統時鐘（`RealtimeClock`）。
   - 在軟體在環模擬（SITL, Software-in-the-Loop）中，系統必須能夠**人為推進時間（Step-by-Step Stepping）**，甚至以 10 倍速或暫停運行。
   - 若飛控演算法內部直接呼叫 `std::chrono::system_clock::now()` 或系統 API，將永遠無法進行精確、可重現（Deterministic）的 SITL 模擬。
2. **參數伺服器（Parameter Manager）**：
   - 飛控擁有數百個控制增益與校準值（如 `MC_ROLL_P`, `SIM_MASS`）。
   - 必須支援執行時熱更新（Runtime Update）、型別安全檢查、邊界保護以及「參數變更通知機制」。

---

## 2. 核心架構與 API 規範

### 3A. 時間抽象 (Clock Interface)
```cpp
class IClock {
public:
    virtual ~IClock() = default;
    virtual uint64_t nowUs() const = 0;
};
```
- `RealtimeClock`: 讀取宿主機高解析時脈。
- `SimulationClock`: 人工控制步進，提供 `advanceUs(uint64_t dtUs)`，時間絕不倒退。

### 3B. 參數管理器 (Parameter Manager)
- 型別支援：`int32_t`, `float`, `bool`, `std::string`。
- API：
  - `registerParam<T>(name, defaultValue, minVal, maxVal)`
  - `get<T>(name)`
  - `set<T>(name, value)`
- **進階聯動機制**：參數修改時發布 `parameter_update` 訊息至 TopicBus，使高頻控制模組能緩存參數，避免在 250Hz 控制迴路中進行字串查找。

---

## 3. 三個實現階段說明

### 階段一：基礎版（Stage 1 - Clock Abstraction）
- **重點**：實作 `IClock` 介面，並完成 `RealtimeClock` 與 `SimulationClock`。
- **依賴注入驗證**：建立 `DummyFlightModule`，注入不同的 Clock，驗證其時間前進與完全無感於實際時鐘源。

### 階段二：核心進階版（Stage 2 - Type-Safe Parameter Manager）
- **重點**：型別安全參數表、執行緒安全保護與邊界檢查（Min/Max Clamping）。
- **異常防護**：讀取不存在參數時拋出或回傳錯誤、型別錯誤拒絕轉換。

### 階段三：PX4 對標進階版（Stage 3 - Param Notification & File Persistence）
- **重點**：對標 PX4 的參數更新事件廣播與快取機制。
- **機制**：
  - 當 `param.set()` 發生時，自動在 TopicBus 上發送 `parameter_update` 訊息。
  - 控制器在收到通知時更新自身本機暫存（Local Cached Floats），實現零查表開銷。
  - 支援模擬 PX4 參數檔（文字 key-value）匯入與匯出。

---

## 4. 驗收標準與測試清單

- [x] `SimulationClock` 步進具有確定性（Deterministic），且時間不可倒退。
- [x] 控制演算法依賴 `IClock`，無論何種時鐘都能正確計算 `dt`。
- [x] 參數型別錯誤或超出上限下限時受到攔截保護。
- [x] 參數修改時成功觸發 `parameter_update` 事件，訂閱端及時更新快取。
