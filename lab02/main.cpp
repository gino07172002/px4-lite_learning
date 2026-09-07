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
    virtual ~WorkItem() = default; // 虛擬解構子
    virtual void Run() = 0;        // 純虛函式

    int getId() const { return _id; }

private:
    int _id;
};

// 2. 具體任務實作類別 (繼承 WorkItem)
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
            } // 鎖在此處及時釋放

            // 必須防範 nullptr
            if (currentTask != nullptr)
            {
                currentTask->Run();
                delete currentTask; // 釋放堆疊記憶體
            }
        }
    }

    std::mutex _mutex;
    std::thread _worker;
    std::condition_variable _cv;
    std::atomic<bool> running{true};
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