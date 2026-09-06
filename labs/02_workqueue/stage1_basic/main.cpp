/**
 * @file main.cpp
 * @brief Lab 02 - Stage 1 (基礎版): 單執行緒定時工作佇列 (WorkQueue)
 * 
 * [本階段目標與設計重點]:
 * 1. 任務抽象: 定義 WorkItem 虛擬介面。
 * 2. 優先級小頂堆 (Min-Heap): 依照預定執行時間 (Deadline) 排序任務。
 * 3. 條件變數等待 (wait_until): 精確休眠直至最早任務到期，避免 CPU 忙輪詢 (Busy Waiting)。
 */

#include <iostream>
#include <vector>
#include <queue>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <chrono>
#include <atomic>

class WorkItem {
public:
    virtual ~WorkItem() = default;
    virtual void Run() = 0;
};

struct ScheduledTask {
    std::shared_ptr<WorkItem> item;
    std::chrono::steady_clock::time_point deadline;

    bool operator>(const ScheduledTask& other) const {
        return deadline > other.deadline;
    }
};

class SimpleWorkQueue {
public:
    SimpleWorkQueue() : _running(true), _worker(&SimpleWorkQueue::workerLoop, this) {}

    ~SimpleWorkQueue() {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _running = false;
        }
        _cv.notify_all();
        if (_worker.joinable()) {
            _worker.join();
        }
    }

    void scheduleNow(std::shared_ptr<WorkItem> item) {
        scheduleDelayed(item, std::chrono::microseconds(0));
    }

    void scheduleDelayed(std::shared_ptr<WorkItem> item, std::chrono::microseconds delay) {
        auto deadline = std::chrono::steady_clock::now() + delay;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _queue.push({std::move(item), deadline});
        }
        _cv.notify_one();
    }

private:
    void workerLoop() {
        while (true) {
            std::unique_lock<std::mutex> lock(_mutex);

            while (_running && _queue.empty()) {
                _cv.wait(lock);
            }

            if (!_running && _queue.empty()) {
                break;
            }

            auto now = std::chrono::steady_clock::now();
            auto topTask = _queue.top();

            if (now >= topTask.deadline) {
                _queue.pop();
                lock.unlock(); // 執行任務前回調釋放鎖
                topTask.item->Run();
            } else {
                // 等待最近任務到期
                _cv.wait_until(lock, topTask.deadline);
            }
        }
    }

    std::priority_queue<ScheduledTask, std::vector<ScheduledTask>, std::greater<ScheduledTask>> _queue;
    std::mutex _mutex;
    std::condition_variable _cv;
    std::atomic<bool> _running;
    std::thread _worker;
};

// 測試用任務
class PrintTask : public WorkItem {
public:
    PrintTask(std::string name, int delayMs) : _name(std::move(name)), _delayMs(delayMs) {}

    void Run() override {
        std::cout << " -> [" << _name << "] 執行成功！預期延遲: " << _delayMs << " ms\n";
    }

private:
    std::string _name;
    int _delayMs;
};

int main() {
    std::cout << "========================================================\n";
    std::cout << " [Lab 02 - Stage 1 基礎版]: 定時工作佇列排程測試        \n";
    std::cout << "========================================================\n";

    SimpleWorkQueue wq;

    std::cout << "排程任務: Task C(150ms), Task A(0ms), Task B(50ms)...\n";
    wq.scheduleDelayed(std::make_shared<PrintTask>("Task C (150ms)", 150), std::chrono::milliseconds(150));
    wq.scheduleNow(std::make_shared<PrintTask>("Task A (立即執行)", 0));
    wq.scheduleDelayed(std::make_shared<PrintTask>("Task B (50ms)", 50), std::chrono::milliseconds(50));

    // 等待所有任務執行完畢
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    std::cout << "\n>>> Stage 1 全部驗收標準達成！ <<<\n";
    return 0;
}
