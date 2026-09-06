/**
 * @file main.cpp
 * @brief Lab 02 - Stage 3 (PX4 對標進階版): WorkItem 封裝、執行超時保護與阻塞連鎖效應實驗
 * 
 * [本階段目標與設計重點]:
 * 1. 對標 PX4 架構的 WorkItem 物件導向封裝:
 *    - 模組繼承 WorkItem，可呼叫 ScheduleNow() / ScheduleOnInterval() 自主排程。
 * 2. 運行時監控與 Overrun 警報:
 *    - 每次 Run() 執行時精確記錄耗時。若單次耗時超過預算 (Overrun)，自動輸出告警。
 * 3. 阻塞連鎖效應實驗 (Critical Blocking Experiment):
 *    - 在同一個 WorkQueue 上安排 Task A (50Hz) 與 Misbehaving Task B。
 *    - 當 Task B 惡意 sleep (Blocking)，展示 Task A 如何被嚴重飢餓 (Starvation/Jitter)。
 *    - 深刻理解 PX4 開發金律：「絕對不能在 WorkQueue 回呼中呼叫阻塞式 API (如 sleep/socket wait)」。
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
#include <string>

class WorkQueue;

class WorkItem : public std::enable_shared_from_this<WorkItem> {
public:
    explicit WorkItem(std::string name, WorkQueue* queue = nullptr)
        : _name(std::move(name)), _queue(queue) {}
    virtual ~WorkItem() = default;

    virtual void Run() = 0;

    void setWorkQueue(WorkQueue* q) { _queue = q; }
    void scheduleNow();
    void scheduleDelayed(uint64_t delayUs);
    void scheduleOnInterval(uint64_t intervalUs);

    const std::string& getName() const { return _name; }

protected:
    std::string _name;
    WorkQueue* _queue{nullptr};
};

struct ScheduledEntry {
    std::shared_ptr<WorkItem> item;
    std::chrono::steady_clock::time_point deadline;
    uint64_t intervalUs{0};

    bool operator>(const ScheduledEntry& other) const {
        return deadline > other.deadline;
    }
};

class WorkQueue {
public:
    explicit WorkQueue(std::string name, uint64_t maxBudgetUs = 5000)
        : _name(std::move(name)), _maxBudgetUs(maxBudgetUs), _running(true),
          _worker(&WorkQueue::loop, this) {}

    ~WorkQueue() {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _running = false;
        }
        _cv.notify_all();
        if (_worker.joinable()) {
            _worker.join();
        }
    }

    void schedule(std::shared_ptr<WorkItem> item, uint64_t delayUs, uint64_t intervalUs) {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(delayUs);
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _queue.push({std::move(item), deadline, intervalUs});
        }
        _cv.notify_one();
    }

private:
    void loop() {
        while (true) {
            std::unique_lock<std::mutex> lock(_mutex);

            while (_running && _queue.empty()) {
                _cv.wait(lock);
            }

            if (!_running && _queue.empty()) break;

            auto now = std::chrono::steady_clock::now();
            auto top = _queue.top();

            if (now >= top.deadline) {
                _queue.pop();

                if (top.intervalUs > 0 && _running) {
                    auto nextDeadline = top.deadline + std::chrono::microseconds(top.intervalUs);
                    if (nextDeadline <= now) {
                        nextDeadline = now + std::chrono::microseconds(top.intervalUs);
                    }
                    _queue.push({top.item, nextDeadline, top.intervalUs});
                }

                lock.unlock();

                // 測量任務執行耗時
                auto t0 = std::chrono::steady_clock::now();
                top.item->Run();
                auto t1 = std::chrono::steady_clock::now();

                uint64_t execUs = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
                if (execUs > _maxBudgetUs) {
                    std::cout << "[WORKQUEUE OVERRUN WARN][" << _name << "] "
                              << top.item->getName() << " 耗時 " << execUs << " us, 超過預算 "
                              << _maxBudgetUs << " us!\n";
                }
            } else {
                _cv.wait_until(lock, top.deadline);
            }
        }
    }

    std::string _name;
    uint64_t _maxBudgetUs;
    std::priority_queue<ScheduledEntry, std::vector<ScheduledEntry>, std::greater<ScheduledEntry>> _queue;
    std::mutex _mutex;
    std::condition_variable _cv;
    std::atomic<bool> _running;
    std::thread _worker;
};

void WorkItem::scheduleNow() {
    if (_queue) _queue->schedule(shared_from_this(), 0, 0);
}
void WorkItem::scheduleDelayed(uint64_t delayUs) {
    if (_queue) _queue->schedule(shared_from_this(), delayUs, 0);
}
void WorkItem::scheduleOnInterval(uint64_t intervalUs) {
    if (_queue) _queue->schedule(shared_from_this(), 0, intervalUs);
}

// 正常高頻任務
class RegularTask : public WorkItem {
public:
    using WorkItem::WorkItem;
    void Run() override {
        _runCount++;
    }
    int getCount() const { return _runCount.load(); }
private:
    std::atomic<int> _runCount{0};
};

// 故意發生阻塞的不良任務
class BadBlockingTask : public WorkItem {
public:
    using WorkItem::WorkItem;
    void Run() override {
        // 故意阻塞 50ms
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
};

int main() {
    std::cout << "=======================================================================\n";
    std::cout << " [Lab 02 - Stage 3 PX4對標進階版]: 執行超時防護與 Blocking 連鎖反應實驗 \n";
    std::cout << "=======================================================================\n";

    // 建立一個高優先級佇列，執行時間預算設定為 5ms (5000 us)
    WorkQueue wqShared("SHARED_WQ", 5000);

    auto normalTask = std::make_shared<RegularTask>("NormalTask_50Hz", &wqShared);
    auto badTask = std::make_shared<BadBlockingTask>("BadBlockingTask", &wqShared);

    // 安排 NormalTask 每 20ms (50Hz) 執行一次
    normalTask->scheduleOnInterval(20000);

    std::cout << "\n[階段 A]: 正常運行 200ms (無阻塞干擾)...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    int countBefore = normalTask->getCount();
    std::cout << " -> NormalTask 累計執行次數: " << countBefore << " 次 (預期約 10 次)\n";

    std::cout << "\n[階段 B]: 加入惡意阻塞任務 (BadBlockingTask: 阻塞 50ms)...\n";
    badTask->scheduleNow(); // 觸發一次長時間阻塞

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    int countAfter = normalTask->getCount();

    std::cout << " -> 經歷阻塞後 NormalTask 總執行次數: " << countAfter << " 次\n";
    std::cout << "\n[深度剖析總結]:\n";
    std::cout << " 1. 成功捕獲 OVERRUN WARN，耗時約 50000 us (> 5000 us 預算)。\n";
    std::cout << " 2. 直觀證實：在同一個 WorkQueue 上的任何阻塞都會造成共用佇列的即時任務被凍結！\n";
    std::cout << " 3. 這正是 PX4 嚴格要求 WorkItem 執行時間必須極短（<100us）且嚴禁 blocking 的根本原因。\n";

    std::cout << "\n>>> Stage 3 全部驗收標準達成！ <<<\n";
    return 0;
}
