#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <atomic>
#include <condition_variable>
using namespace std;

class WorkItem {
public:
    virtual void Run(){ 
        std::cout<<" working on task id : "<<_id<<std::endl;
     }
    WorkItem(int id):_id(id){};
    private:
    int _id;
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
    void addTest( WorkItem* taskId)
    {
        {
        std::unique_lock<std::mutex> lock(_mutex);
        _tasks.push(taskId);
        }
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
        while(true)
        {
             WorkItem* currentTask=nullptr;
            {
                std::unique_lock<std::mutex> lock(_mutex);
                
                _cv.wait(lock,[this](){return !_tasks.empty() || !running;});
                
                if(!_tasks.empty())
                {
                    currentTask=_tasks.front();
                    
                    _tasks.pop();
                   
                }
                
                if(!running)
                {
                    break;
                }
            }
             currentTask->Run();
           
        }
    }
    std::mutex _mutex;
    std::thread _worker;
    std::condition_variable _cv;
    bool running{true};
    std::queue< WorkItem*> _tasks;
    
};

int main()
{
    cout << "Hello Work Queue!" << endl;
    
    
    SimpleWorkQueue wq;
    WorkItem item1(123);
    WorkItem item2(456);
    WorkItem item3(789);

    wq.addTest(&item1);
    wq.addTest(&item2);
    wq.addTest(&item3);
    wq.start();
    SimpleWorkQueue wq2;
    WorkItem item4(5123);
    WorkItem item5(5456);
    WorkItem item6(5789);
    wq2.addTest(&item4);
    wq2.addTest(&item5);
    wq2.addTest(&item6);
    wq2.start();
    
    
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout<<" bye bye "<<std::endl;
    return 0;
}
/*
[使用者備註]:
我第一版長這樣
*/