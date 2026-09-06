/**
 * @file main.cpp
 * @brief Lab 03 - Stage 3 (PX4 對標進階版): 參數更新廣播機制 (parameter_update) 與高頻快取架構
 * 
 * [本階段目標與設計重點]:
 * 1. 對標 PX4 parameter_update 機制:
 *    - 嚴禁在 250Hz 的即時控制迴路中每次都透過字串查找 (如 params.get<float>("MC_ROLL_P"))。
 *    - 解決方案：模組訂閱 parameter_update 主題，內部維護本地 float 快取成員。
 *    - 只有當地面站或 CLI 呼叫 param.set() 時，廣播事件通知模組刷新快取。
 * 2. 參數檔案持久化與解析 (Persistence & Parsing):
 *    - 支援模擬 PX4 格式的參數字串匯入與匯出。
 */

#include <iostream>
#include <string>
#include <unordered_map>
#include <variant>
#include <mutex>
#include <sstream>
#include <memory>
#include <cstdint>
#include <cassert>

// 參數更新通知結構體 (對標 PX4 parameter_update.msg)
struct ParameterUpdateMsg {
    uint64_t timestamp_us;
    uint32_t instance;
};

// 簡化版 Topic
class ParamUpdateTopic {
public:
    void publish(const ParameterUpdateMsg& msg) {
        std::lock_guard<std::mutex> lock(_mutex);
        _data = msg;
        ++_generation;
    }
    bool checkUpdated(uint64_t& lastGen) const {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_generation > lastGen) {
            lastGen = _generation;
            return true;
        }
        return false;
    }
private:
    ParameterUpdateMsg _data{};
    uint64_t _generation{0};
    mutable std::mutex _mutex;
};

// 具備事件通知的參數管理器
class AdvancedParameterManager {
public:
    static AdvancedParameterManager& instance() {
        static AdvancedParameterManager s_instance;
        return s_instance;
    }

    void setFloat(const std::string& name, float val) {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _floatParams[name] = val;
        }
        // 發布參數更新廣播
        _updateTopic.publish({1000, ++_updateCounter});
    }

    float getFloat(const std::string& name, float defaultVal = 0.0f) const {
        std::lock_guard<std::mutex> lock(_mutex);
        auto it = _floatParams.find(name);
        return (it != _floatParams.end()) ? it->second : defaultVal;
    }

    ParamUpdateTopic& getUpdateTopic() { return _updateTopic; }

    // 匯入文字格式參數
    void loadFromString(const std::string& config) {
        std::istringstream stream(config);
        std::string line;
        while (std::getline(stream, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::string key;
            float val;
            if (std::istringstream(line) >> key >> val) {
                setFloat(key, val);
            }
        }
    }

private:
    AdvancedParameterManager() = default;
    std::unordered_map<std::string, float> _floatParams;
    ParamUpdateTopic _updateTopic;
    uint32_t _updateCounter{0};
    mutable std::mutex _mutex;
};

// 飛控控制器模組：展示 PX4 如何零開銷使用參數
class FastRateController {
public:
    FastRateController() {
        // 初始化時加載第一次參數
        updateParams();
    }

    void updateParams() {
        auto& pm = AdvancedParameterManager::instance();
        _roll_p = pm.getFloat("MC_ROLL_P", 4.0f);
        _roll_i = pm.getFloat("MC_ROLL_I", 0.1f);
        std::cout << "[Controller] 快取參數更新: MC_ROLL_P=" << _roll_p << ", MC_ROLL_I=" << _roll_i << "\n";
    }

    // 250Hz 控制運算迴路 (每秒呼叫 250 次)
    void runCycle() {
        // 1. 僅以極低代價檢查是否有 parameter_update
        if (AdvancedParameterManager::instance().getUpdateTopic().checkUpdated(_lastGen)) {
            updateParams();
        }

        // 2. 核心運算完全使用直接記憶體成員變數，無任何 Hash Table 字串查找開銷！
        float controlOutput = _roll_p * 1.5f;
        (void)controlOutput;
    }

    float getCachedRollP() const { return _roll_p; }

private:
    uint64_t _lastGen{0};
    float _roll_p{4.0f};
    float _roll_i{0.1f};
};

int main() {
    std::cout << "=======================================================================\n";
    std::cout << " [Lab 03 - Stage 3 PX4對標進階版]: parameter_update 廣播與本地快取架構 \n";
    std::cout << "=======================================================================\n";

    auto& pm = AdvancedParameterManager::instance();
    pm.setFloat("MC_ROLL_P", 4.0f);
    pm.setFloat("MC_ROLL_I", 0.15f);

    FastRateController controller;
    assert(controller.getCachedRollP() == 4.0f);

    std::cout << "\n模擬即時控制迴路 3 次運算...\n";
    controller.runCycle();
    controller.runCycle();

    std::cout << "\n模擬地面站 (QGC) 或 CLI 修改參數: MC_ROLL_P = 8.5...\n";
    pm.setFloat("MC_ROLL_P", 8.5f);

    std::cout << "下次控制循環呼叫 runCycle()...\n";
    controller.runCycle(); // 此時應檢測到 parameter_update 並自動同步快取

    assert(controller.getCachedRollP() == 8.5f);
    std::cout << "✓ 成功驗證: 控制器快取已自動無縫更新為: " << controller.getCachedRollP() << "！\n";

    std::cout << "\n模擬載入組態檔文字內容...\n";
    std::string configData = 
        "# 機身參數表\n"
        "MC_ROLL_P 10.2\n"
        "MC_ROLL_I 0.35\n";
    pm.loadFromString(configData);
    controller.runCycle();
    assert(controller.getCachedRollP() == 10.2f);
    std::cout << "✓ 成功驗證: 組態文字匯入並透過廣播同步完成！\n";

    std::cout << "\n>>> Stage 3 全部驗收標準達成！ <<<\n";
    return 0;
}
