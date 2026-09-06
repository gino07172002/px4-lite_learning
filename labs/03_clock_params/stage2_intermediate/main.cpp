/**
 * @file main.cpp
 * @brief Lab 03 - Stage 2 (核心進階版): 型別安全參數管理器 (ParameterManager)
 * 
 * [本階段目標與設計重點]:
 * 1. 統一管理與型別安全:
 *    - 使用 std::variant 支援 float, int32_t, bool 型別。
 *    - 提供 get<T>(name) 與 set<T>(name, value)。
 * 2. 邊界保護 (Min/Max Clamping):
 *    - 設定數值超出範圍時自動截斷或拒絕，避免飛控因為不合理的 PID 參數而炸機。
 * 3. 執行緒安全與例外防護:
 *    - 讀取不存在的參數或型別不符合時給予安全回報。
 */

#include <iostream>
#include <string>
#include <unordered_map>
#include <variant>
#include <mutex>
#include <optional>
#include <cstdint>
#include <cassert>

using ParamValue = std::variant<int32_t, float, bool>;

struct ParamMetadata {
    ParamValue value;
    ParamValue defaultValue;
    std::optional<float> minVal;
    std::optional<float> maxVal;
    std::string description;
};

class ParameterManager {
public:
    static ParameterManager& instance() {
        static ParameterManager s_instance;
        return s_instance;
    }

    // 註冊參數
    void declareFloat(const std::string& name, float defaultVal, float minVal, float maxVal, const std::string& desc = "") {
        std::lock_guard<std::mutex> lock(_mutex);
        ParamMetadata meta;
        meta.value = defaultVal;
        meta.defaultValue = defaultVal;
        meta.minVal = minVal;
        meta.maxVal = maxVal;
        meta.description = desc;
        _params[name] = meta;
    }

    void declareInt(const std::string& name, int32_t defaultVal, int32_t minVal, int32_t maxVal, const std::string& desc = "") {
        std::lock_guard<std::mutex> lock(_mutex);
        ParamMetadata meta;
        meta.value = defaultVal;
        meta.defaultValue = defaultVal;
        meta.minVal = static_cast<float>(minVal);
        meta.maxVal = static_cast<float>(maxVal);
        meta.description = desc;
        _params[name] = meta;
    }

    // 取得參數
    template <typename T>
    std::optional<T> get(const std::string& name) const {
        std::lock_guard<std::mutex> lock(_mutex);
        auto it = _params.find(name);
        if (it == _params.end()) {
            std::cerr << "[PARAM ERROR] 找不到參數: " << name << "\n";
            return std::nullopt;
        }
        if (!std::holds_alternative<T>(it->second.value)) {
            std::cerr << "[PARAM ERROR] 參數 " << name << " 型別不符合！\n";
            return std::nullopt;
        }
        return std::get<T>(it->second.value);
    }

    // 設定參數 (具備 Clamp 限制)
    bool set(const std::string& name, float val) {
        std::lock_guard<std::mutex> lock(_mutex);
        auto it = _params.find(name);
        if (it == _params.end() || !std::holds_alternative<float>(it->second.value)) return false;

        if (it->second.minVal && val < *it->second.minVal) val = *it->second.minVal;
        if (it->second.maxVal && val > *it->second.maxVal) val = *it->second.maxVal;

        it->second.value = val;
        return true;
    }

    bool set(const std::string& name, int32_t val) {
        std::lock_guard<std::mutex> lock(_mutex);
        auto it = _params.find(name);
        if (it == _params.end() || !std::holds_alternative<int32_t>(it->second.value)) return false;

        float fval = static_cast<float>(val);
        if (it->second.minVal && fval < *it->second.minVal) fval = *it->second.minVal;
        if (it->second.maxVal && fval > *it->second.maxVal) fval = *it->second.maxVal;

        it->second.value = static_cast<int32_t>(fval);
        return true;
    }

private:
    ParameterManager() = default;
    std::unordered_map<std::string, ParamMetadata> _params;
    mutable std::mutex _mutex;
};

int main() {
    std::cout << "===============================================================\n";
    std::cout << " [Lab 03 - Stage 2 核心進階版]: 型別安全參數伺服器與邊界保護  \n";
    std::cout << "===============================================================\n";

    auto& params = ParameterManager::instance();

    // 註冊飛控常用參數
    params.declareFloat("MC_ROLL_P", 4.0f, 0.5f, 12.0f, "Multicopter Roll Proportional Gain");
    params.declareFloat("SIM_MASS", 1.2f, 0.1f, 5.0f, "Simulation Quadcopter Mass (kg)");
    params.declareInt("SYS_AUTOSTART", 4001, 1000, 9999, "PX4 Autostart Airframe ID");

    // 1. 正常獲取預設值
    auto rollP = params.get<float>("MC_ROLL_P");
    assert(rollP.has_value() && *rollP == 4.0f);
    std::cout << "✓ 預設值讀取成功: MC_ROLL_P = " << *rollP << "\n";

    // 2. 正常修改數值
    params.set("MC_ROLL_P", 6.5f);
    std::cout << "✓ 修改後讀取: MC_ROLL_P = " << *params.get<float>("MC_ROLL_P") << " (預期 6.5)\n";

    // 3. 測試邊界限制 (Clamp): 嘗試設定超過上限 12.0f 的數值 99.0f
    params.set("MC_ROLL_P", 99.0f);
    std::cout << "✓ 超出最大值 Clamping 驗證: MC_ROLL_P = " << *params.get<float>("MC_ROLL_P") << " (已被限縮為 12.0)\n";
    assert(*params.get<float>("MC_ROLL_P") == 12.0f);

    // 4. 型別錯誤攔截測試 (用 int 取 float)
    std::cout << "測試型別不匹配攔截:\n";
    auto invalidType = params.get<int32_t>("MC_ROLL_P");
    assert(!invalidType.has_value());

    // 5. 不存在之鍵值攔截測試
    auto nonExistent = params.get<float>("NON_EXISTENT_KEY");
    assert(!nonExistent.has_value());

    std::cout << "\n>>> Stage 2 全部驗收標準達成！ <<<\n";
    return 0;
}
