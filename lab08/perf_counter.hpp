#pragma once

#include <chrono>
#include <cstdint>
#include <algorithm>
#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <unordered_map>
#include <memory>
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

class Clock
{
public:
    virtual uint64_t nowUs() const = 0;
    virtual ~Clock() = default;
};

class SimulationClock : public Clock
{
public:
    SimulationClock() : currentTimeUs(0) {}

    uint64_t nowUs() const override
    {
        return currentTimeUs;
    }

    void advance(uint64_t us)
    {
        currentTimeUs += us;
    }

private:
    uint64_t currentTimeUs;
};

class RealtimeClock : public Clock
{
public:
    RealtimeClock() : startTime(std::chrono::steady_clock::now()) {}

    uint64_t nowUs() const override
    {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(now - startTime).count();
    }

private:
    std::chrono::steady_clock::time_point startTime;
};

class PerfCounter
{
public:
    PerfCounter(Clock& clock) : _clock(clock) {}

    void begin()
    {
        if (_running)
            return;
        _running = true;
        auto newStartTime = _clock.nowUs();

        if (_hasStarted)
        {
            auto period = (newStartTime - _startTime);
            _totalPeriodUs += period;
            ++_periodCount;
        }
        else
        {
            _hasStarted = true;
        }
        _startTime = newStartTime;
    }

    void end()
    {
        if (!_running)
            return;
        auto endTime = _clock.nowUs();
        auto elapsed = (endTime - _startTime);
        _lastUs = elapsed;

        ++_count;
        _totalUs += elapsed;

        _averageUs = _totalUs / _count;
        _maxUs = std::max(_maxUs, _lastUs);
        _running = false;
    }

    uint64_t count() const
    {
        return _count;
    }

    uint64_t lastUs() const
    {
        return _lastUs;
    }

    uint64_t averageUs() const
    {
        return _averageUs;
    }

    uint64_t maxUs() const
    {
        return _maxUs;
    }

    double frequency() const
    {
        if (_totalPeriodUs == 0 || _periodCount == 0)
            return 0.0;

        return static_cast<double>(_periodCount) * 1000000.0
               / static_cast<double>(_totalPeriodUs);
    }

private:
    uint64_t _startTime{};
    uint64_t _count = 0;
    Clock& _clock;
    uint64_t _lastUs = 0;
    uint64_t _totalUs = 0;
    uint64_t _averageUs = 0;
    uint64_t _maxUs = 0;
    bool _hasStarted = false;
    uint64_t _totalPeriodUs = 0;
    uint64_t _periodCount = 0;
    bool _running = false;
};

class SimpleWorkQueue {
public:

   SimpleWorkQueue(Clock& clock):_clock(clock)
{
}
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
if (_perf.find(currentTask.item) == _perf.end())
{
    _perf[currentTask.item] =
        std::make_unique<PerfCounter>(_clock);
}
auto& perf = *_perf[currentTask.item];

perf.begin();
                currentTask.item->Run();
perf.end();
            }
        }
    }

    std::mutex _mutex;
    std::thread _worker;
    std::condition_variable _cv;
    bool running{false};
    std::unordered_map<WorkItem*, uint64_t> _generation;
    std::priority_queue<ScheduledTask, std::vector<ScheduledTask>, std::greater<ScheduledTask>> _scheduledTask;
    Clock& _clock;
std::unordered_map<WorkItem*, std::unique_ptr<PerfCounter>> _perf;
};

class Timer
{
public:
    Timer(Clock& clock) : clock(clock), startTime(clock.nowUs()) {}

    void reset()
    {
        startTime = clock.nowUs();
    }

    uint64_t elapsedUs() const
    {
        auto elapsed = clock.nowUs() - startTime;
        return elapsed;
    }

private:
    Clock& clock;
    uint64_t startTime;
};
