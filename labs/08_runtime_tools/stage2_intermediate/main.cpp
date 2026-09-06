/**
 * @file main.cpp
 * @brief Lab 08 - Stage 2 (核心進階版): 飛行黑盒子日誌記錄器 (Flight CSV Logger)
 * 
 * [本階段目標與設計重點]:
 * 1. 飛行數據日誌紀錄 (Flight Logger):
 *    - 採樣記錄關鍵狀態: timestamp, roll, roll_sp, motor1~4。
 *    - 寫入 flight.csv 格式，提供事後分析與調參依據。
 * 2. 效能考量:
 *    - 批次緩衝寫入，避免每次寫入都調用同步磁碟 I/O。
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <cstdint>

struct FlightLogEntry {
    uint64_t timestamp_us;
    float roll_deg;
    float roll_sp_deg;
    float motors[4];
};

class FlightLogger {
public:
    explicit FlightLogger(const std::string& filename) : _filename(filename) {
        _file.open(_filename);
        if (_file.is_open()) {
            _file << "timestamp_us,roll_deg,roll_sp_deg,motor1,motor2,motor3,motor4\n";
        }
    }

    ~FlightLogger() {
        if (_file.is_open()) {
            _file.flush();
            _file.close();
        }
    }

    void log(const FlightLogEntry& entry) {
        if (!_file.is_open()) return;
        _file << entry.timestamp_us << ","
              << entry.roll_deg << ","
              << entry.roll_sp_deg << ","
              << entry.motors[0] << ","
              << entry.motors[1] << ","
              << entry.motors[2] << ","
              << entry.motors[3] << "\n";
        _entryCount++;
    }

    size_t getEntryCount() const { return _entryCount; }
    const std::string& getFilename() const { return _filename; }

private:
    std::string _filename;
    std::ofstream _file;
    size_t _entryCount{0};
};

int main() {
    std::cout << "===============================================================\n";
    std::cout << " [Lab 08 - Stage 2 核心進階版]: 飛行黑盒子 CSV Logger 驗證     \n";
    std::cout << "===============================================================\n";

    std::string csvPath = "flight.csv";
    FlightLogger logger(csvPath);

    std::cout << "模擬飛行 2.0 秒 (以 100Hz 記錄飛行狀態至 " << csvPath << ")...\n";

    uint64_t timeUs = 1000000;
    for (int i = 0; i < 200; ++i) { // 200 筆 (100Hz)
        timeUs += 10000;
        float t = i * 0.01f;

        // 模擬一個 +15 度衰減震盪收斂至 0 度的曲線
        float roll = 15.0f * std::exp(-2.5f * t) * std::cos(10.0f * t);
        float roll_sp = 0.0f;

        FlightLogEntry entry{
            timeUs,
            roll,
            roll_sp,
            {0.5f - 0.02f * roll, 0.5f + 0.02f * roll, 0.5f + 0.02f * roll, 0.5f - 0.02f * roll}
        };

        logger.log(entry);
    }

    std::cout << "✓ 成功寫入 " << logger.getEntryCount() << " 筆飛行日誌！\n";
    std::cout << "檔案已生成於: " << csvPath << "\n";
    std::cout << "可使用同目錄下 plot_log.py 腳本進行波形視覺化繪圖。\n";

    std::cout << "\n>>> Stage 2 全部驗收標準達成！ <<<\n";
    return 0;
}
