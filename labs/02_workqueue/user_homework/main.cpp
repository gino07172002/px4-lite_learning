#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <atomic>
#include <condition_variable>
using namespace std;

std::queue<int> tasks;
class WorkItem {
public:
    virtual void Run() = 0;
    
};

class SimpleWorkQueue {
public:
    ~SimpleWorkQueue()
    {
        {
            std::unique_lock<std::mutex> lock(_mutex);
            running=false;
        }
        _cv.notify_all();
        if(_worker.joinable())
        {
            _worker.join();
        }
        std::cout<<" bye bye queue "<<std::endl;
    }
    void addTest(int taskId)
    {
        std::unique_lock<std::mutex> lock(_mutex);
        _tasks.push(taskId);
        _cv.notify_one();
    }
    void start()
    {
        if(!_worker.joinable())
        {
            std::cout<<"worker not yet init ! "<<std::endl;
            if(!running)
                running=true;
            _worker = std::thread(&SimpleWorkQueue::workerLoop,this);
        }
    }
    
private:
    void workerLoop()
    {
        
        std::cout<<" hi wordking"<<std::endl;
        while(running)
        {
            int currentTask=0;
            {
                std::unique_lock<std::mutex> lock(_mutex);
                
                _cv.wait(lock,[this](){return !_tasks.empty() || !running;});
                if(!_tasks.empty())
                {
                    currentTask=_tasks.front();
                    
                    _tasks.pop();
                    std::cout<<" doing _task : "<< currentTask<<std::endl;
                }
            }
            
           
        }
    }
    std::mutex _mutex;
    std::thread _worker;
    std::condition_variable _cv;
    atomic<bool> running{true};
    std::queue<int> _tasks;
    
};

int main()
{
    cout << "Hello Work Queue!" << endl;
    
    
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
    
    
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout<<" bye bye "<<std::endl;
    return 0;
}
/*
[使用者備註]:
我第一版長這樣
*/