#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <unordered_map>
using namespace std;
using namespace std::chrono;

class WorkItem {
public:
    virtual void Run() = 0;
    virtual ~WorkItem() = default; // [必備] 加上 virtual 滿足 C++ 多型安全解構規範

};

struct ScheduledTask
{
    WorkItem* item=nullptr;
    time_point<steady_clock> runAt;
    uint64_t generation=0;

    bool repeat=false;

    milliseconds interval=0ms;

    bool operator>(const ScheduledTask& other) const
    {
        return runAt > other.runAt;
    }
};

class LogTask : public WorkItem
{
public:
    explicit LogTask(int id) : _id(id) {}

    void Run() override
    {
        auto now = steady_clock::now();

        if (!_first)
        {
            auto gap =
                duration_cast<microseconds>(now - _lastRun);

            if (gap > _maxGap)
                _maxGap = gap;
        }

        _lastRun = now;
        _first = false;
        ++_count;
    }

    int count() const
    {
        return _count;
    }

    double maxGapMs() const
    {
        return _maxGap.count() / 1000.0;
    }

private:
    int _count{0};
    int _id;

    bool _first{true};
    steady_clock::time_point _lastRun;
    microseconds _maxGap{0};
};
class BlockTask : public WorkItem {
public:
    explicit BlockTask(int id) : _id(id) {}
    void Run() override {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        _count++;
    }
    int count() const
    {
        return _count;
    }


private:
    int _count=0;
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

    void ScheduleNow(WorkItem* item)
    {
        ScheduleDelayed(item,0ms);

    }

    void ScheduleDelayed(WorkItem* item, std::chrono::milliseconds delay)
    {
        {
            std::unique_lock<std::mutex> lock(_mutex);

            _scheduledTask.push({item, steady_clock::now() + delay,_generation[item]});


        }
        _cv.notify_one();

    }
    void ScheduleOnInterval(WorkItem* item, std::chrono::milliseconds interval)
    {
        if(interval==0s)
            return;
        {
            std::unique_lock<std::mutex> lock(_mutex);

            ScheduledTask task;
            task.item=item;
            task.runAt=steady_clock::now() + interval;
            task.interval=interval;
            task.repeat=true;
            task.generation=_generation[item];
            _scheduledTask.push(std::move(task));

        }
        _cv.notify_one();

    }
    void ScheduleClear(WorkItem* item)
    {
        {
            std::unique_lock<std::mutex> lock(_mutex);
            ++_generation[item];

        }
        std::cout<<" cancel work item : "<<std::endl;
    }

    void start()
    {
        if (!_worker.joinable())
        {
            std::cout << "worker not yet init ! " << std::endl;
            {
                std::unique_lock<std::mutex> lock(_mutex);
                if (!running)
                    running = true;
            }
            _worker = std::thread(&SimpleWorkQueue::workerLoop, this);
        }
    }

private:
    void workerLoop()
    {
        std::cout << " hi working" << std::endl;
        while (true)
        {
            ScheduledTask currentTask ;
            {
                std::unique_lock<std::mutex> lock(_mutex);
                _cv.wait(lock, [this]() {
                    return !running || !_scheduledTask.empty();
                });

                // 立即終止模式 (Abort on shutdown): 不亂 pop，直接退出
                if (!running)
                {
                    break;
                }
                if(!_scheduledTask.empty())
                {
                    auto targetTime = _scheduledTask.top().runAt;

                    _cv.wait_until(lock, targetTime, [this, targetTime]() {

                        return !running ||
                                (!_scheduledTask.empty() && _scheduledTask.top().runAt < targetTime) ||
                                std::chrono::steady_clock::now() >= targetTime;
                    });
                    if (!running)
                    {
                        break;
                    }

                    auto now = steady_clock::now();
                    if(now>=_scheduledTask.top().runAt)
                    {


                        currentTask =  _scheduledTask.top();
                        _scheduledTask.pop();
                        if(currentTask.generation!=_generation[currentTask.item])
                        {
                            continue;
                        }
                        if(currentTask.repeat)
                        {
                            currentTask.runAt+=currentTask.interval;
                            _scheduledTask.push(currentTask);
                        }

                    }

                }

            }

            // 安全防護: 確保非空才呼叫
            if (currentTask.item!=nullptr)
            {
                currentTask.item->Run();
            }
        }
    }

    std::mutex _mutex;
    std::thread _worker;
    std::condition_variable _cv;
    bool running{false};
    std::unordered_map<WorkItem*, uint64_t> _generation;
    std::priority_queue<ScheduledTask, std::vector<ScheduledTask>, std::greater<ScheduledTask>> _scheduledTask;
};

int main()
{
    LogTask item1(250); //4ms
    LogTask item2(100); //10ms
    LogTask item3(20);  //50ms
    LogTask item4(10);  //100ms

    BlockTask blockTask(1);

    cout << "Hello Work Queue!" << endl;
    {


        SimpleWorkQueue high;
        SimpleWorkQueue low;

        high.ScheduleOnInterval(&item1,4ms);
        high.ScheduleOnInterval(&item2,10ms);


        low.ScheduleOnInterval(&item3,50ms);
        low.ScheduleOnInterval(&item4,100ms);

        high.start();
        low.start();

         std::this_thread::sleep_for(std::chrono::seconds(1));

        high.ScheduleNow(&blockTask);

        std::this_thread::sleep_for(std::chrono::seconds(10));
    }


    cout << "250Hz expected=2500 actual="
         << item1.count() << '\n';

    cout << "100Hz expected=1000 actual="
         << item2.count() << '\n';

    cout << "20Hz expected=200 actual="
         << item3.count() << '\n';

    cout << "10Hz expected=100 actual="
         << item4.count() << '\n';

    cout << "250Hz count=" << item1.count()
         << " maxGap=" << item1.maxGapMs()
         << "ms\n";
    std::cout << " bye bye " << std::endl;
    return 0;
}


