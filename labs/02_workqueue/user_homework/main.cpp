#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <atomic>
#include <condition_variable>

using namespace std;

// 1. 抽象基底類別：定義飛控任務介面
class WorkItem {
public:
    explicit WorkItem(int id) : _id(id) {}
    virtual ~WorkItem() = default; // [必備] 虛擬解構子，避免多型 delete 時未定義行為
    virtual void Run() = 0;        // 純虛函式，由具體任務繼承實作

    int getId() const { return _id; }

private:
    int _id;
};

// 2. 具體任務實作類別 (解決抽象類別不可直接 new 的編譯錯誤)
class SimpleTask : public WorkItem {
public:
    explicit SimpleTask(int id) : WorkItem(id) {}

    void Run() override {
        std::cout << " -> doing _task : " << getId() << std::endl;
    }
};

class SimpleWorkQueue {
public:
    ~SimpleWorkQueue()
    {
        {
            std::unique_lock<std::mutex> lock(_mutex);
            running = false;
        }
        _cv.notify_all();
        if (_worker.joinable())
        {
            _worker.join();
        }
        std::cout << " bye bye queue " << std::endl;
    }

    void addTest(WorkItem* task)
    {
        {
            std::unique_lock<std::mutex> lock(_mutex);
            _tasks.push(task);
        }
        _cv.notify_one();
    }

    void start()
    {
        if (!_worker.joinable())
        {
            std::cout << "worker not yet init ! " << std::endl;
            if (!running)
                running = true;
            _worker = std::thread(&SimpleWorkQueue::workerLoop, this);
        }
    }
    
private:
    void workerLoop()
    {
        std::cout << " hi working" << std::endl;
        while (running)
        {
            WorkItem* currentTask = nullptr;
            {
                std::unique_lock<std::mutex> lock(_mutex);
                _cv.wait(lock, [this]() { return !_tasks.empty() || !running; });
                
                if (!_tasks.empty())
                {
                    currentTask = _tasks.front();
                    _tasks.pop();
                }
            } // [優點] 鎖在此處釋放！

            // [關鍵修復]: 喚醒時若因 running=false 且佇列為空，currentTask 會是 nullptr！
            // 必須先做非空檢查，否則會發生 Null Pointer Dereference 崩潰 (Segmentation fault)
            if (currentTask != nullptr)
            {
                currentTask->Run();
                delete currentTask; // [修復]: 執行後釋放堆疊記憶體，杜絕 Memory Leak
            }
        }
    }

    std::mutex _mutex;
    std::thread _worker;
    std::condition_variable _cv;
    std::atomic<bool> running{true}; // 使用 atomic 保證跨線程安全
    std::queue<WorkItem*> _tasks;
};

int main()
{
    cout << "Hello Work Queue!" << endl;
    
    SimpleWorkQueue wq;
    wq.addTest(new SimpleTask(123));
    wq.addTest(new SimpleTask(456));
    wq.addTest(new SimpleTask(789));
    wq.start();

    SimpleWorkQueue wq2;
    wq2.addTest(new SimpleTask(5123));
    wq2.addTest(new SimpleTask(5456));
    wq2.addTest(new SimpleTask(5789));
    wq2.start();
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << " bye bye " << std::endl;
    return 0;
}