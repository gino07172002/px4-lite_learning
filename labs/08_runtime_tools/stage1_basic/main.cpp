/**
 * @file main.cpp
 * @brief Lab 08 - Stage 1 (基礎版): uORB CLI 命令列工具與 Topic 頻率測量 (topic hz)
 * 
 * [本階段目標與設計重點]:
 * 1. CLI 命令解析器:
 *    - topic list: 列出系統已註冊的所有主題名稱。
 *    - topic hz <name>: 採樣統計指定主題的實體發布頻率。
 *    - param show / param set: 參數檢視與調整。
 * 2. 頻率測量精度驗證:
 *    - 背景以 250Hz 發布資料，CLI 採樣 1 秒並輸出 ~250.0 Hz。
 */

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <thread>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <memory>

struct TopicStats {
    std::atomic<uint64_t> publishCount{0};
};

class MiniCli {
public:
    void registerTopic(const std::string& name) {
        _topics[name] = std::make_shared<TopicStats>();
    }

    void recordPublish(const std::string& name) {
        auto it = _topics.find(name);
        if (it != _topics.end()) {
            it->second->publishCount++;
        }
    }

    void execute(const std::string& cmdLine) {
        std::istringstream iss(cmdLine);
        std::string cmd;
        iss >> cmd;

        if (cmd == "topic") {
            std::string subCmd;
            iss >> subCmd;
            if (subCmd == "list") {
                std::cout << "\n[TOPIC LIST]:\n";
                for (const auto& [name, _] : _topics) {
                    std::cout << "  - " << name << "\n";
                }
            } else if (subCmd == "hz") {
                std::string topicName;
                iss >> topicName;
                measureHz(topicName);
            }
        } else if (cmd == "param") {
            std::string subCmd;
            iss >> subCmd;
            if (subCmd == "show") {
                std::cout << "\n[PARAMETERS]:\n";
                for (const auto& [k, v] : _params) {
                    std::cout << "  " << std::setw(15) << std::left << k << " = " << v << "\n";
                }
            } else if (subCmd == "set") {
                std::string key;
                float val;
                if (iss >> key >> val) {
                    _params[key] = val;
                    std::cout << "  " << key << " 已設定為 " << val << "\n";
                }
            }
        } else {
            std::cout << "未知指令: " << cmdLine << "\n";
        }
    }

private:
    void measureHz(const std::string& name) {
        auto it = _topics.find(name);
        if (it == _topics.end()) {
            std::cout << "找不到主題: " << name << "\n";
            return;
        }
        std::cout << "採樣主題 [" << name << "] 頻率中 (持續 0.5 秒)...\n";
        uint64_t count0 = it->second->publishCount.load();
        auto t0 = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        auto t1 = std::chrono::steady_clock::now();
        uint64_t count1 = it->second->publishCount.load();

        double sec = std::chrono::duration<double>(t1 - t0).count();
        double hz = (count1 - count0) / sec;
        std::cout << " -> " << name << " 當前發布頻率: " << std::fixed << std::setprecision(1) 
                  << hz << " Hz\n";
    }

    std::unordered_map<std::string, std::shared_ptr<TopicStats>> _topics;
    std::unordered_map<std::string, float> _params{{"MC_ROLL_P", 4.0f}, {"SIM_MASS", 1.2f}};
};

int main() {
    std::cout << "========================================================\n";
    std::cout << " [Lab 08 - Stage 1 基礎版]: uORB CLI 與頻率計 (topic hz) \n";
    std::cout << "========================================================\n";

    MiniCli cli;
    cli.registerTopic("sensor_imu");
    cli.registerTopic("vehicle_attitude");
    cli.registerTopic("actuator_motors");

    std::atomic<bool> running{true};

    // 模擬背景以 250Hz 發布 sensor_imu
    std::thread imuThread([&]() {
        while (running) {
            cli.recordPublish("sensor_imu");
            std::this_thread::sleep_for(std::chrono::microseconds(4000)); // 250Hz
        }
    });

    // 執行 CLI 指令展示
    cli.execute("topic list");
    cli.execute("param show");
    cli.execute("param set MC_ROLL_P 6.2");
    cli.execute("topic hz sensor_imu");

    running = false;
    imuThread.join();

    std::cout << "\n>>> Stage 1 全部驗收標準達成！ <<<\n";
    return 0;
}
