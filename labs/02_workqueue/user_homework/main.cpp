/**
 * @file main.cpp
 * @brief 使用者 Lab 02 作業第一版（已整理為可編譯形式）
 * 
 * [導師深度點評與架構分析]:
 * 1. 優點 (Strengths):
 *    - 正確建立了背景 Worker Thread 與非同步 Task Queue 的基本分離模型。
 *    - 具備並發競爭意識，在存取 _tasks 佇列時使用了 std::unique_lock<std::mutex> 進行安全防護。
 *    - 撰寫了健全的解構子 (~SimpleWorkQueue)，將 running 設為 false 並呼叫 _worker.join()，
 *      保證了物件銷毀時執行緒能優雅退出，避免未 join 導致程式 crash。
 *    - 在 main() 中建立了 wq 與 wq2 兩個佇列實例，這非常符合 PX4 雙 Worker 佇列 (如 HIGH 與 LOW) 的思維！
 * 
 * 2. 待演進的關鍵架構差距 (Evolution to PX4 WorkQueue):
 *    - [問題 1: 忙輪詢 vs 條件變數等待 (std::condition_variable)]
 *      目前在 workerLoop() 中使用 std::this_thread::sleep_for(1000ms)。
 *      這會造成兩個問題：
 *      a. 當主執行緒 addTest() 推入任務時，工作線程無法被「即時喚醒」，最差情況要乾等 1 秒！
 *      b. 在沒有任務時，執行緒依然每秒被喚醒一次，浪費 CPU。
 *      => 正確解法：引入 std::condition_variable，佇列為空時完全休眠，一旦 add() 即刻 notify_one()。
 *    - [問題 2: 任務抽象與多型分發]
 *      頂部定義了 class WorkItem { virtual void Run() = 0; };，但佇列中目前存的是 int taskId。
 *      => 正確解法：讓佇列存放 std::shared_ptr<WorkItem>，在 workerLoop 執行 item->Run()。
 *    - [問題 3: 飛控高精度排程 (Deadline 排序)]
 *      飛控任務具有週期性（如 250Hz, 100Hz, 10Hz）。單純 FIFO 的 std::queue 無法支援定時延遲排程。
 *      => 正確解法：改用以時間為權重的小頂堆 (std::priority_queue)，實現 ScheduleDelayed 與 ScheduleOnInterval。
 */

#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <atomic>
#include <chrono>

using namespace std;

// 抽象工作項目介面
class WorkItem {
public:
    virtual ~WorkItem() = default;
    virtual void Run() = 0;
};

class SimpleWorkQueue {
public:
    ~SimpleWorkQueue()
    {
        running = false;
        if (_worker.joinable())
        {
            _worker.join();
        }
        std::cout << "[WorkQueue] bye bye queue" << std::endl;
    }

    void addTest(int taskId)
    {
        std::unique_lock<std::mutex> lock(_mutex);
        _tasks.push(taskId);
    }

    void start()
    {
        if (!_worker.joinable())
        {
            std::cout << "[WorkQueue] worker starting thread..." << std::endl;
            if (!running)
                running = true;
            _worker = std::thread(&SimpleWorkQueue::workerLoop, this);
        }
    }
    
private:
    void workerLoop()
    {
        std::cout << "[WorkerThread] hi working! Thread ID: " << std::this_thread::get_id() << std::endl;
        while (running)
        {
            int currentTask = 0;
            bool hasTask = false;
            {
                std::unique_lock<std::mutex> lock(_mutex);
                if (!_tasks.empty())
                {
                    currentTask = _tasks.front();
                    _tasks.pop();
                    hasTask = true;
                }
            }

            if (hasTask)
            {
                std::cout << " -> doing task : " << currentTask << std::endl;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(200)); // 縮短至 200ms 使測試更流暢
        }
    }

    std::mutex _mutex;
    std::thread _worker;
    atomic<bool> running{true};
    std::queue<int> _tasks;
};

int main()
{
    cout << "=== [User Homework] Lab 02 WorkQueue 第一版 ===" << endl;
    
    SimpleWorkQueue wq;
    wq.addTest(123);
    wq.addTest(456);
    wq.addTest(789);
    wq.start();

    SimpleWorkQueue wq2;
    wq2.addTest(5123);
    wq2.addTest(5456);
    wq2.addTest(5789);
    wq2.start();
    
    // 主執行緒等待 2 秒以觀察兩條 WorkerThread 同時消化佇列任務
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "作業執行完成，即將退出..." << std::endl;
    return 0;
}
