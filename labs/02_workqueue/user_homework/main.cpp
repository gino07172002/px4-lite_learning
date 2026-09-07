#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <atomic>
#include <condition_variable>
using namespace std;

class WorkItem {
public:
    virtual void Run() = 0;
    virtual ~WorkItem() = default; // [必備] 加上 virtual 滿足 C++ 多型安全解構規範
};

class LogTask : public WorkItem {
public:
    explicit LogTask(int id) : _id(id) {}
    void Run() override {
        std::cout << " logging task id : " << _id << std::endl;
    }
private:
    int _id;
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

    void addTest(WorkItem* taskId)
    {
        {
            std::unique_lock<std::mutex> lock(_mutex);
            _tasks.push(taskId);
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
        while (true)
        {
            WorkItem* currentTask = nullptr;
            {
                std::unique_lock<std::mutex> lock(_mutex);
                _cv.wait(lock, [this]() { return !_tasks.empty() || !running; });
                
                // 立即終止模式 (Abort on shutdown): 不亂 pop，直接退出
                if (!running)
                {
                    break;
                }
                
                if (!_tasks.empty())
                {
                    currentTask = _tasks.front();
                    _tasks.pop();
                }
            }
            
            // 安全防護: 確保非空才呼叫
            if (currentTask)
            {
                currentTask->Run();
            }
        }
    }

    std::mutex _mutex;
    std::thread _worker;
    std::condition_variable _cv;
    bool running{true};
    std::queue<WorkItem*> _tasks;
};

int main()
{
    cout << "Hello Work Queue!" << endl;
    
    // 巧妙利用 C++ 區域變數 LIFO 逆序解構：item 先於 wq 宣告，因此 wq 必定先析構 join 完畢，物件才銷毀！
    LogTask item1(123);
    LogTask item2(456);
    LogTask item3(789);
    SimpleWorkQueue wq;
    wq.addTest(&item1);
    wq.addTest(&item2);
    wq.addTest(&item3);
    wq.start();
   
    LogTask item4(5123);
    LogTask item5(5456);
    LogTask item6(5789);
    SimpleWorkQueue wq2;
    wq2.addTest(&item4);
    wq2.addTest(&item5);
    wq2.addTest(&item6);
    wq2.start();
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << " bye bye " << std::endl;
    return 0;
}