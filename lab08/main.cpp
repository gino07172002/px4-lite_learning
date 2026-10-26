

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>
using namespace std;
using namespace std::chrono;
struct VehicleAttitude
{
    float roll{0.0f};
    float pitch{0.0f};
    float yaw{0.0f};
};
std::ostream& operator<<(std::ostream& os,
                         const VehicleAttitude& attitude)
{
    os << "roll = " << attitude.roll
       << " pitch = " << attitude.pitch
       << " yaw = " << attitude.yaw;

    return os;
}

struct RatesSetpoint
{
    float rollRate{0.0f};
    float pitchRate{0.0f};
    float yawRate{0.0f};
};
std::ostream& operator<<(std::ostream& os,
                         const RatesSetpoint& rate)
{
    os << "rollRate = " << rate.rollRate
       << " pitchRate = " << rate.pitchRate
       << " yawRate = " << rate.yawRate;

    return os;
}
struct TorqueSetpoint
{
    float rollTorque{0.0f};
    float pitchTorque{0.0f};
    float yawTorque{0.0f};
};

std::ostream& operator<<(std::ostream& os,
                         const TorqueSetpoint& torque)
{
    os << " rollTorque = " << torque.rollTorque
       << " pitchTorque = " << torque. pitchTorque
       << " yawTorque = " <<torque.yawTorque;

    return os;
}

struct TestMessage {
    uint64_t value{0};
    uint64_t inverse{0};

    TestMessage() = default;
    explicit TestMessage(uint64_t v) : value(v), inverse(~v) {};
};

std::ostream& operator<<(std::ostream& os, const TestMessage& msg) {
    os << "value = " << msg.value << " inverse = " << msg.inverse;
    return os;
}

struct ActuatorMotors
{
    float m1{0.0f};
    float m2{0.0f};
    float m3{0.0f};
    float m4{0.0f};
};

std::ostream& operator<<(std::ostream& os,
                         const ActuatorMotors& motors)
{
    os << "m1 = " << motors.m1
       << " m2 = " << motors.m2
       << " m3 = " << motors.m3
       << " m4 = " << motors.m4;
    return os;
}


class ITopic {
public:
    virtual ~ITopic() = default;
    virtual void echo() const = 0;
    virtual uint64_t generation() const = 0;
};

class TopicManager {
public:
    void registerTopic(const std::string& name, ITopic* itopic) {
        _names.push_back(name);
        _topicNames[name] = itopic;
    }

    std::vector<std::string> topicNames() const { return _names; }
    ITopic* findTopic(const std::string& name) {
        auto it = _topicNames.find(name);
        if (it == _topicNames.end())
            return nullptr;
        else
            return it->second;
    }

private:
    std::vector<std::string> _names;
    unordered_map<string, ITopic*> _topicNames;
};

// ------------------------------------------------------------
// main
// ------------------------------------------------------------

template <typename T> class Topic : public ITopic {
public:
    void publish(const T& newData) {
        std::lock_guard<std::mutex> lock(_mutex);

        msg = newData;
        ++_generation;
    }
    bool copy(T& outData, uint64_t& outGeneration) const {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_generation == 0)
            return false;

        outData = msg;
        outGeneration = _generation;

        return true;
    }
    uint64_t getGeneration() const {
        std::lock_guard<std::mutex> lock(_mutex);
        return _generation;
    }
    uint64_t generation() const override {
        std::lock_guard<std::mutex> lock(_mutex);
        return _generation;
    };

    void echo() const override {

        T data{};
        uint64_t gen = 0;
        if (copy(data, gen)) {

            std::cout << "generation : " << gen << " " << data << "\n";

        } else {
            std::cout << "no data\n ";
        }
    };

private:
    T msg{};
    uint64_t _generation = 0;
    mutable std::mutex _mutex;
};

template <typename T> class Subscription {
public:
    Subscription() = default;
    void sub(Topic<T>& topic) { _topic = &topic; }
    bool copy(T& out) {
        if (_topic == nullptr) {
            return false;

        } else {
            return (_topic->copy(out, _lastGeneration));
        }
    }
    bool updated() const {
        if (_topic == nullptr)
            return false;

        return (_lastGeneration != _topic->getGeneration());
    }

private:
    uint64_t _lastGeneration = 0;
    Topic<T>* _topic = nullptr;
};

template <typename T> class Publisher {
public:
    Publisher() = default;
    void bind(Topic<T>& topic) { _topic = &topic; }
    bool publish(const T& data) {
        if (_topic != nullptr) {
            _topic->publish(data);
            return true;
        }
        return false;
    }

private:
    Topic<T>* _topic = nullptr;
};
class CsvLogger {
public:
    CsvLogger(const std::string& filename) : file_(filename) {}

    void log(const std::string& message) {
        if (file_.is_open()) {
            file_ << message << "\n";
        }
    }
    void flush() {
        if (file_.is_open()) {
            file_.flush();
        }
    }

private:
    std::ofstream file_;
};
class WorkItem {
public:
    virtual void Run() = 0;
    virtual ~WorkItem() =
        default; // [必備] 加上 virtual 滿足 C++ 多型安全解構規範
    virtual const std::string& name() const = 0;
};

struct PerfStats {
    uint64_t count = 0;
    uint64_t lastUs = 0;
    uint64_t averageUs = 0;
    uint64_t maxUs = 0;
    double frequency = 0.0;
uint64_t maxLatenessUs = 0;
uint64_t skippedCycles = 0;
};

struct ScheduledTask {
    WorkItem* item = nullptr;
    time_point<steady_clock> runAt;
    uint64_t generation = 0;

    bool repeat = false;

    milliseconds interval = 0ms;

    bool operator>(const ScheduledTask& other) const {
        return runAt > other.runAt;
    }
};

class LogTask : public WorkItem {
public:
    explicit LogTask(int id, string name) : _id(id), _name(std::move(name)) {}

    void Run() override {
        auto now = steady_clock::now();

        if (!_first) {
            auto gap = duration_cast<microseconds>(now - _lastRun);

            if (gap > _maxGap)
                _maxGap = gap;
        }

        _lastRun = now;
        _first = false;
        ++_count;
    }

    int count() const { return _count; }

    double maxGapMs() const { return _maxGap.count() / 1000.0; }
    const string& name() const override { return _name; }

private:
    int _count{0};
    int _id;
    std::string _name{""};
    bool _first{true};
    steady_clock::time_point _lastRun;
    microseconds _maxGap{0};
};
class BlockTask : public WorkItem {
public:
    BlockTask(int id, std::string name) : _id(id), _name(std::move(name)) {}

    void Run() override {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        ++_count;
    }

    const std::string& name() const override { return _name; }

private:
    int _count = 0;
    int _id;
    std::string _name;
};
class ActuatorMotorsTask : public WorkItem {
public:
    ActuatorMotorsTask(Topic<ActuatorMotors>& topic,
                       std::string name)
        : _topic(topic), _name(std::move(name)) {}

    void Run() override {
        ActuatorMotors motors{};
        motors.m1 = 0.50f;
        motors.m2 = 0.48f;
        motors.m3 = 0.52f;
        motors.m4 = 0.50f;

        _topic.publish(motors);
    }

    const std::string& name() const override {
        return _name;
    }

private:
    Topic<ActuatorMotors>& _topic;
    std::string _name;
};
class Clock {
public:
    virtual uint64_t nowUs() const = 0;
    virtual ~Clock() = default;
};

class SimulationClock : public Clock {
public:
    SimulationClock() : currentTimeUs(0) {}

    uint64_t nowUs() const override { return currentTimeUs; }

    void advance(uint64_t us) { currentTimeUs += us; }

private:
    uint64_t currentTimeUs;
};

class RealtimeClock : public Clock {
public:
    RealtimeClock() : startTime(std::chrono::steady_clock::now()) {}

    uint64_t nowUs() const override {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(now -
                                                                     startTime)
            .count();
    }

private:
    std::chrono::steady_clock::time_point startTime;
};

    std::string toCsvField(const TestMessage& msg)
    {
        std::ostringstream ss;

    ss << msg.value
       << ","
       << msg.inverse;

    return ss.str();
    }
    std::string toCsvField(const VehicleAttitude& msg)
    {
           std::ostringstream ss;

    ss << msg.roll
       << ","
       << msg.pitch
       << ","
       << msg.yaw;

    return ss.str();
    }

template <typename T>
class CsvLogTask : public WorkItem {
public:
    CsvLogTask(Topic<T>& topic, CsvLogger& logger, Clock& clock,
               std::string name)
        : _logger(logger), _clock(clock), _name(std::move(name)) {
        _imuSub.sub(topic);
    }



    void Run() override {
        if (_imuSub.updated()) {
            T msg{};

            if (_imuSub.copy(msg)) {
                std::ostringstream row;

                row << _clock.nowUs() <<","<< toCsvField(msg);

                _logger.log(row.str());

                ++_rowsSinceFlush;

                if(_rowsSinceFlush>=100)
                {
                 _logger.flush();
                _rowsSinceFlush=0;
                }
            }
        }
    }

    const std::string& name() const override { return _name; }

private:
    CsvLogger& _logger;
    Clock& _clock;
    Subscription<T> _imuSub;
    std::string _name;
      uint32_t _rowsSinceFlush{0};
};

class FlightCsvLogTask : public WorkItem
{
public:
    FlightCsvLogTask(Topic<TestMessage>& imuTopic,
                     Topic<VehicleAttitude>& attitudeTopic,
 Topic<RatesSetpoint>& ratesSubTopic,
    Topic<TorqueSetpoint>& torqueTopic,
Topic<ActuatorMotors>& motorsTopic,
                     CsvLogger& logger,
                     Clock& clock,
                     std::string name)
        : _logger(logger),
          _clock(clock),
          _name(std::move(name))
    {
        _imuSub.sub(imuTopic);
        _attitudeSub.sub(attitudeTopic);
_ratesSub.sub(ratesSubTopic);
_torqueSub.sub(torqueTopic);
_motorsSub.sub(motorsTopic);
    }
    void Run() override
    {
        TestMessage imu{};
        VehicleAttitude attitude{};
RatesSetpoint rates{};
        TorqueSetpoint torque{};
ActuatorMotors motors{};

        bool imuOk = _imuSub.copy(imu);
        bool attitudeOk = _attitudeSub.copy(attitude);

bool ratesOk = _ratesSub.copy(rates);
bool torqueOk = _torqueSub.copy(torque);
bool motorsOk = _motorsSub.copy(motors);

        if(imuOk&& attitudeOk  && ratesOk&& torqueOk &&motorsOk )
        {

    std::ostringstream row;

row << _clock.nowUs()
    << "," << imu.value
    << "," << imu.inverse
    << "," << attitude.roll
    << "," << attitude.pitch
    << "," << attitude.yaw
    << "," << rates.rollRate
    << "," << rates.pitchRate
    << "," << rates.yawRate
      << "," << torque.rollTorque
    << "," << torque.pitchTorque
    << "," << torque.yawTorque
<< "," << motors.m1
       << "," << motors.m2
       << "," << motors.m3
       << "," << motors.m4;

    _logger.log(row.str());
    ++_rowsSinceFlush;
    if(_rowsSinceFlush>=100)
    {
        _logger.flush();
        _rowsSinceFlush=0;
    }
        }
    }

    const std::string& name() const override
    {
        return _name;
    }

private:
    CsvLogger& _logger;
    Clock& _clock;

    Subscription<TestMessage> _imuSub;
    Subscription<VehicleAttitude> _attitudeSub;
    Subscription<RatesSetpoint> _ratesSub;
Subscription<TorqueSetpoint> _torqueSub;
Subscription<ActuatorMotors> _motorsSub;
    uint32_t _rowsSinceFlush{0};
    std::string _name;
};

class VehicleAttitudeTask : public WorkItem {
public:
    VehicleAttitudeTask(Topic<VehicleAttitude >& topic, std::string name)
        : _topic(topic), _name(std::move(name)) {}

    void Run() override {
        ++_vehicleAttitude.roll;
        ++_vehicleAttitude.pitch;
        ++_vehicleAttitude.yaw;
        _topic.publish(_vehicleAttitude);
    }

    const std::string& name() const override { return _name; }

private:
    Topic<VehicleAttitude>& _topic;
    std::string _name;

    VehicleAttitude _vehicleAttitude;

};

class RatesSetpointTask : public WorkItem {
public:
	RatesSetpointTask(Topic<RatesSetpoint>& topic, std::string name)
        : _topic(topic), _name(std::move(name)) {}

    void Run() override {
        ++_ratesSetpoint.rollRate;
        ++_ratesSetpoint.pitchRate;
        ++_ratesSetpoint.yawRate;
        _topic.publish(_ratesSetpoint);
    }

    const std::string& name() const override { return _name; }

private:
    Topic<RatesSetpoint>& _topic;
    std::string _name;

    RatesSetpoint _ratesSetpoint;


};

class TorqueSetpointTask : public WorkItem {
public:
	TorqueSetpointTask(Topic<TorqueSetpoint>& topic, std::string name)
        : _topic(topic), _name(std::move(name)) {}

    void Run() override {
        ++_torqueSetpoint.rollTorque;
        ++_torqueSetpoint.pitchTorque;
        ++_torqueSetpoint.yawTorque;
        _topic.publish(_torqueSetpoint);
    }

    const std::string& name() const override { return _name; }

private:
    Topic<TorqueSetpoint>& _topic;
TorqueSetpoint _torqueSetpoint;
std::string _name;


};

class SensorImuTask : public WorkItem {
public:
    SensorImuTask(Topic<TestMessage>& topic, std::string name)
        : _topic(topic), _name(std::move(name)) {}

    void Run() override {
        ++_value;
        _topic.publish(TestMessage{_value});
    }

    const std::string& name() const override { return _name; }

private:
    Topic<TestMessage>& _topic;
    std::string _name;
    uint64_t _value{0};
};

#include <atomic>

class ShutdownTestTask : public WorkItem {
public:
    explicit ShutdownTestTask(
        std::string name,
        std::chrono::milliseconds workTime = 0ms)
        : _name(std::move(name)),
          _workTime(workTime) {}

    void Run() override {
        started.store(true);

        std::this_thread::sleep_for(_workTime);

        ++count;
        finished.store(true);
    }

    const std::string& name() const override {
        return _name;
    }

    std::atomic<int> count{0};
    std::atomic<bool> started{false};
    std::atomic<bool> finished{false};

private:
    std::string _name;
    std::chrono::milliseconds _workTime;
};
class LifetimeTestTask : public ShutdownTestTask {
public:
    explicit LifetimeTestTask(
        std::vector<std::string>& events)
        : ShutdownTestTask("LifetimeTestTask", 300ms),
          _events(events) {}

    void Run() override {
        _events.push_back("Run started");

        ShutdownTestTask::Run();

        _events.push_back("Run finished");
    }

    ~LifetimeTestTask() override {
        _events.push_back("Task destroyed");
    }

private:
    std::vector<std::string>& _events;
};
class PerfCounter {
public:
    PerfCounter(Clock& clock) : _clock(clock) {}

    PerfStats snapShot() const {
        std::lock_guard<std::mutex> lock(_mutex);
        PerfStats stats;
        stats.count = _count;
        stats.lastUs = _lastUs;
        stats.averageUs = _averageUs;
        stats.maxUs = _maxUs;
stats.maxLatenessUs = _maxLatenessUs;
         stats.skippedCycles= _skippedCycles;

        if (_totalPeriodUs != 0 && _periodCount != 0) {
            stats.frequency = static_cast<double>(_periodCount) * 1000000.0 /
                              static_cast<double>(_totalPeriodUs);
        }

        return stats;
    }
void recordLateness(uint64_t latenessUs) {
    std::lock_guard<std::mutex> lock(_mutex);

    _maxLatenessUs =
        std::max(_maxLatenessUs, latenessUs);
}
void  recordSkippedCycles(uint64_t skippedCycles) {
    std::lock_guard<std::mutex> lock(_mutex);

    _skippedCycles +=
        skippedCycles;
}

    void begin() {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_running)
            return;
        _running = true;

        auto newStartTime = _clock.nowUs();

        if (_hasStarted) {
            auto period = (newStartTime - _startTime);
            _totalPeriodUs += period;
            ++_periodCount;
        } else {
            _hasStarted = true;
        }
        _startTime = newStartTime;
    }

    void end() {
        std::lock_guard<std::mutex> lock(_mutex);
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

    uint64_t count() const { return _count; }

    uint64_t lastUs() const { return _lastUs; }

    uint64_t averageUs() const { return _averageUs; }

    uint64_t maxUs() const { return _maxUs; }

    double frequency() const {
        if (_totalPeriodUs == 0 || _periodCount == 0)
            return 0.0;

        return static_cast<double>(_periodCount) * 1000000.0 /
               static_cast<double>(_totalPeriodUs);
    }

private:
    uint64_t _startTime{};
    uint64_t _count = 0;
    Clock& _clock;
    uint64_t _lastUs = 0;
    uint64_t _totalUs = 0;
    uint64_t _averageUs = 0;
    uint64_t _maxUs = 0;
uint64_t _maxLatenessUs{0};
    bool _hasStarted = false;
    uint64_t _totalPeriodUs = 0;
    uint64_t _periodCount = 0;
uint64_t _skippedCycles= 0;

    bool _running = false;
    mutable mutex _mutex;
};
struct WorkItemStats {
    std::string name;
    PerfStats perf;
};
class SimpleWorkQueue {
public:
    SimpleWorkQueue(Clock& clock) : _clock(clock) {}
    ~SimpleWorkQueue() {
        {
            std::unique_lock<std::mutex> lock(_mutex);
            running = false;
        }
        _cv.notify_all();
        if (_worker.joinable()) {
            _worker.join();
        }
        std::cout << " bye bye queue " << std::endl;
    }

    void ScheduleNow(WorkItem* item) { ScheduleDelayed(item, 0ms); }

    void ScheduleDelayed(WorkItem* item, std::chrono::milliseconds delay) {
        {
            std::unique_lock<std::mutex> lock(_mutex);

            _scheduledTask.push(
                {item, steady_clock::now() + delay, _generation[item]});
        }
        _cv.notify_one();
    }
    void ScheduleOnInterval(WorkItem* item,
                            std::chrono::milliseconds interval) {
        if (interval <= 0s)
            return;
        {
            std::unique_lock<std::mutex> lock(_mutex);

            ScheduledTask task;
            task.item = item;
            task.runAt = steady_clock::now() + interval;
            task.interval = interval;
            task.repeat = true;
            task.generation = _generation[item];
            _scheduledTask.push(std::move(task));
        }
        _cv.notify_one();
    }
    void ScheduleClear(WorkItem* item) {
        {
            std::unique_lock<std::mutex> lock(_mutex);
            ++_generation[item];
        }
        std::cout << " cancel work item : " << std::endl;
    }

    void start() {
        if (!_worker.joinable()) {
            std::cout << "worker not yet init ! " << std::endl;
            {
                std::unique_lock<std::mutex> lock(_mutex);
                if (!running)
                    running = true;
            }
            _worker = std::thread(&SimpleWorkQueue::workerLoop, this);
        }
    }
    bool getPerfStats(WorkItem* item, PerfStats& out) {
        std::lock_guard<std::mutex> lock(_mutex);
        auto it = _perf.find(item);

        if (it == _perf.end())
            return false;

        out = it->second->snapShot();
        return true;
    }
    std::vector<WorkItemStats> getAllPerfStats() {
        std::vector<WorkItemStats> result;

        std::lock_guard<std::mutex> lock(_mutex);

        for (const auto& entry : _perf) {
            // entry.first
            //     是 WorkItem*
            //
            // entry.second
            //     是 unique_ptr<PerfCounter>

            WorkItemStats stats;

            stats.name = entry.first->name();
            stats.perf = entry.second->snapShot();

            result.push_back(stats);
        }

        return result;
    }

private:
    void workerLoop() {
        while (true) {
            ScheduledTask currentTask;
uint64_t latenessUs = 0;
uint64_t skippedCycles = 0;
            {
                std::unique_lock<std::mutex> lock(_mutex);
                _cv.wait(lock, [this]() {
                    return !running || !_scheduledTask.empty();
                });

                // 立即終止模式 (Abort on shutdown): 不亂 pop，直接退出
                if (!running) {
                    break;
                }
                if (!_scheduledTask.empty()) {
                    auto targetTime = _scheduledTask.top().runAt;

                    _cv.wait_until(lock, targetTime, [this, targetTime]() {
                        return !running ||
                               (!_scheduledTask.empty() &&
                                _scheduledTask.top().runAt < targetTime) ||
                               std::chrono::steady_clock::now() >= targetTime;
                    });
                    if (!running) {
                        break;
                    }

                    auto now = steady_clock::now();
                    if (now >= _scheduledTask.top().runAt) {

                        currentTask = _scheduledTask.top();
                        _scheduledTask.pop();

 latenessUs =
    duration_cast<microseconds>(
        now - currentTask.runAt
    ).count();

                        if (currentTask.generation !=
                            _generation[currentTask.item]) {
                            continue;
                        }
                        if (currentTask.repeat) {

auto missed =
    (now - currentTask.runAt) / currentTask.interval;

skippedCycles+=static_cast<uint64_t>(missed);

currentTask.runAt += currentTask.interval*(missed+1);


                            _scheduledTask.push(currentTask);
                        }
                    }
                }
            }

            PerfCounter* perf2 = nullptr;

            // 安全防護: 確保非空才呼叫
            if (currentTask.item != nullptr) {
                {
                    std::lock_guard<std::mutex> lock(_mutex);

                    if (_perf.find(currentTask.item) == _perf.end()) {
                        _perf[currentTask.item] =
                            std::make_unique<PerfCounter>(_clock);
                    }
                    auto& perf = *_perf[currentTask.item];

                    perf2 = &perf;
                }
               perf2->recordLateness(latenessUs);
perf2->recordSkippedCycles(skippedCycles);
                perf2->begin();
                currentTask.item->Run();
                perf2->end();
            }
        }
    }

    std::mutex _mutex;
    std::thread _worker;
    std::condition_variable _cv;
    bool running{false};
    std::unordered_map<WorkItem*, uint64_t> _generation;
    std::priority_queue<ScheduledTask, std::vector<ScheduledTask>,
                        std::greater<ScheduledTask>>
        _scheduledTask;
    Clock& _clock;
    std::unordered_map<WorkItem*, std::unique_ptr<PerfCounter>> _perf;
};

class Timer {
public:
    Timer(Clock& clock) : clock(clock), startTime(clock.nowUs()) {}

    void reset() { startTime = clock.nowUs(); }

    uint64_t elapsedUs() const {
        auto elapsed = clock.nowUs() - startTime;
        return elapsed;
    }

private:
    Clock& clock;
    uint64_t startTime;
};
using ParamValue = std::variant<int32_t, float, bool>;

class ParameterManager {
public:
    template <typename T> void set(const std::string& name, T value) {
        std::lock_guard<std::mutex> lock(mtx); // 確保線程安全
        params[name] = value;
        ++generation; // 更新參數版本
    }
    std::vector<std::pair<std::string, ParamValue>> snapshot() const {
        std::lock_guard<std::mutex> lock(mtx);

        std::vector<std::pair<std::string, ParamValue>> result;

        for (const auto& entry : params) {
            result.push_back(entry);
        }

        return result;
    }
    template <typename T> T get(const std::string& name) const {
        std::lock_guard<std::mutex> lock(mtx); // 確保線程安全
        // params.at(name) 在找不到 key 時會拋出 std::out_of_range
        // std::get<T>(...) 在型別不符時會拋出 std::bad_variant_access
        return std::get<T>(params.at(name));
    }
    bool exists(const std::string& name) const {
        std::lock_guard<std::mutex> lock(mtx);
        return params.find(name) != params.end();
    }
    size_t getTypeIndex(const std::string& name) const {
        std::lock_guard<std::mutex> lock(mtx);
        return params.at(name).index();
    }
    uint32_t getGeneration() const {
        std::lock_guard<std::mutex> lock(mtx); // 確保線程安全
        return generation;
    }

private:
    std::unordered_map<std::string, ParamValue> params;
    mutable std::mutex mtx;  // 用於保護 params 的互斥鎖
    uint32_t generation = 0; // 用於追蹤參數的版本或更新次數
};
int main() {

{
RealtimeClock clock;
ShutdownTestTask task("ShutdownTest");

{
    SimpleWorkQueue queue(clock);

    queue.ScheduleDelayed(&task, 1000ms);
    queue.start();

    std::this_thread::sleep_for(100ms);

} // queue 在這裡解構

std::cout << "count = " << task.count << '\n';
}


{
    RealtimeClock clock;
ShutdownTestTask slowTask("SlowShutdownTask", 300ms);

{
    SimpleWorkQueue queue(clock);

    queue.start();
    queue.ScheduleNow(&slowTask);

    auto deadline = steady_clock::now() + 1s;

    while (!slowTask.started.load() &&
           steady_clock::now() < deadline) {
        std::this_thread::sleep_for(1ms);
    }

    if (!slowTask.started.load()) {
        std::cerr << "Task did not start\n";
        // 不進入預期成功的驗收流程
    }

} // Queue 在這裡等待 Run() 結束

std::cout << "finished = "
          << slowTask.finished.load() << '\n';
}

// Test 3: Lifetime Safety

std::vector<std::string> events;
bool startedInTime = false;

{
    RealtimeClock clock;
    LifetimeTestTask task(events);

    {
        SimpleWorkQueue queue(clock);

        queue.start();
        queue.ScheduleNow(&task);

        // 確保 Task 已經開始執行
        auto deadline = steady_clock::now() + 1s;

        while (!task.started.load() &&
               steady_clock::now() < deadline) {
            std::this_thread::sleep_for(1ms);
        }

        startedInTime = task.started.load();

    } // ~SimpleWorkQueue()，等待 Worker 完成

    events.push_back("Queue destroyed");

} // ~LifetimeTestTask()，記錄 Task destroyed

// 印出事件順序
for (const auto& event : events) {
    std::cout << event << '\n';
}

// 自動驗證
std::vector<std::string> expected = {
    "Run started",
    "Run finished",
    "Queue destroyed",
    "Task destroyed"
};

if (startedInTime && events == expected) {
    std::cout << "Test 3: PASS\n";
} else {
    std::cout << "Test 3: FAIL\n";
}
    std::string line;
    CsvLogger logger("flight.csv");
logger.log(
    "timestamp_us,value,inverse,"
    "roll,pitch,yaw,"
    "roll_rate_sp,pitch_rate_sp,yaw_rate_sp,"
    "roll_torque_sp,pitch_torque_sp,yaw_torque_sp,"
    "m1,m2,m3,m4"
);
    logger.flush();
    {
        SimulationClock clock;
        PerfCounter perf(clock);

        for (int i = 0; i < 10; ++i) {
            perf.begin();

            // 模擬 task 實際執行 100 us
            clock.advance(100);

            perf.end();

            // 剩下 3900 us 為 idle
            clock.advance(3900);
        }

        std::cout << "count = " << perf.count() << '\n';

        std::cout << "last = " << perf.lastUs() << " us\n";

        std::cout << "avg = " << perf.averageUs() << " us\n";

        std::cout << "max = " << perf.maxUs() << " us\n";

        std::cout << "freq = " << perf.frequency() << " Hz\n";
    }
    {
        ParameterManager paramManager;

        paramManager.set<float>("MC_ROLL_P", 5.0f);
        paramManager.set<float>("MC_PITCH_P", 4.0f);
        paramManager.set<bool>("SYS_ARM_OK", true);
        TopicManager topicManager;

        Topic<TestMessage> sensorImu;
        Topic<VehicleAttitude> vehicleAttitude;
        Topic<RatesSetpoint> ratesSetpoint;
        Topic<TorqueSetpoint> torqueSetpoint;
Topic<ActuatorMotors> actuatorMotors;



        topicManager.registerTopic("sensor_imu", &sensorImu);
        topicManager.registerTopic("vehicle_attitude", &vehicleAttitude);
        topicManager.registerTopic("rates_setpoint", &ratesSetpoint);
        topicManager.registerTopic(    "torque_setpoint",    &torqueSetpoint);
topicManager.registerTopic(
    "actuator_motors",
    &actuatorMotors
);

        ITopic* topic = topicManager.findTopic("sensor_imu");

        RealtimeClock clock;

        LogTask fast(1, "LogTask");
        SensorImuTask imuTask(sensorImu, "SensorImuTask");

        FlightCsvLogTask csvTask(
    sensorImu,
    vehicleAttitude,
    ratesSetpoint,
    torqueSetpoint,
    actuatorMotors,
    logger,
    clock,
    "CsvLogTask"
);

VehicleAttitudeTask attitudeTask(
    vehicleAttitude,
    "VehicleAttitudeTask"
);

RatesSetpointTask ratesSetpointTask
{
ratesSetpoint,
"RatesSetpointTask"
};

TorqueSetpointTask torqueTask
{
torqueSetpoint,
"TorqueSetpointTask"
};
ActuatorMotorsTask motorsTask(
    actuatorMotors,
    "ActuatorMotorsTask"
);

BlockTask blocker(99, "BlockTask");
LogTask oneShot(77, "OneShotCancelTask");
        SimpleWorkQueue queue(clock);

        queue.ScheduleOnInterval(&imuTask, 4ms);
        queue.ScheduleOnInterval(&fast, 4ms);
        queue.ScheduleOnInterval(&csvTask, 4ms);
        queue.ScheduleOnInterval(&attitudeTask, 20ms);
        queue.ScheduleOnInterval(&ratesSetpointTask, 10ms);
        queue.ScheduleOnInterval(&torqueTask, 10ms);
       queue.ScheduleOnInterval(&motorsTask, 10ms);

        queue.start();

std::this_thread::sleep_for(1s);
queue.ScheduleClear(&torqueTask);
std::this_thread::sleep_for(1s);
queue.ScheduleOnInterval(&torqueTask, 10ms);
queue.ScheduleDelayed(&oneShot, 1000ms);

std::this_thread::sleep_for(300ms);
queue.ScheduleClear(&oneShot);

std::this_thread::sleep_for(900ms);

PerfStats stats{};
bool exists = queue.getPerfStats(&oneShot, stats);

std::cout << "after cancel = "
          << (exists ? stats.count : 0) << '\n';

queue.ScheduleDelayed(&oneShot, 200ms);
std::this_thread::sleep_for(300ms);

queue.getPerfStats(&oneShot, stats);
std::cout << "after reschedule = "
          << stats.count << '\n';

std::this_thread::sleep_for(300ms);

queue.getPerfStats(&oneShot, stats);
std::cout << "final = "
          << stats.count << '\n';
        while (true) {
            std::cout << "mini> " << std::flush;

            if (!std::getline(std::cin, line))
                break;

            if (line.empty())
                continue;

            std::stringstream ss(line);

            std::string cmd;
            std::string subcmd;
            std::string arg;
            std::string valueArg;

            ss >> cmd >> subcmd >> arg >> valueArg;

            if (cmd == "exit" || cmd == "quit")
                break;

            if (cmd == "workqueue" && subcmd == "status") {
                auto allStats = queue.getAllPerfStats();

                for (const auto& stats : allStats) {
                    std::cout << stats.name << ":\n"
                              << "  count = " << stats.perf.count << '\n'
                              << "  avg = " << stats.perf.averageUs << " us\n"
                              << "  max = " << stats.perf.maxUs << " us\n"
                              << "  freq = " << stats.perf.frequency << " Hz\n"
<< "  maxLatenessUs = "
<< stats.perf.maxLatenessUs << '\n'
<< "  skippedCycles = "
<< stats.perf.skippedCycles << '\n';
                }
            } else if (cmd == "topic" && subcmd == "list") {
                for (const auto& topic : topicManager.topicNames()) {
                    std::cout << topic << '\n';
                }

            } else if (cmd == "topic" && subcmd == "echo") {
                if (arg.empty()) {
                    std::cout << "usage: topic echo <name>\n";
                } else {
                    ITopic* topic = topicManager.findTopic(arg);

                    if (!topic) {
                        std::cout << "no such topic\n";
                    } else {
                        topic->echo();
                    }
                }
            } else if (cmd == "param" && subcmd == "show") {
                auto allParams = paramManager.snapshot();

                for (const auto& entry : allParams) {
                    std::cout << entry.first << " = ";
                    std::visit([](const auto& value) { std::cout << value; },
                               entry.second);
                    std::cout << "\n";
                }

            } else if (cmd == "param" && subcmd == "set") {
                if (arg.empty() || valueArg.empty()) {
                    std::cout << "usage: param set <name> <value>\n";
                } else if (!paramManager.exists(arg)) {
                    std::cout << "no such parameter\n";
                } else {

                    size_t typeIndex = paramManager.getTypeIndex(arg);
                    if (typeIndex == 0) {
                        try {
                            paramManager.set<int32_t>(arg, std::stoi(valueArg));
                        } catch (const std::invalid_argument&) {
                            std::cout << "invalid value\n";
                        } catch (const std::out_of_range&) {
                            std::cout << "value out of range\n";
                        }

                    } else if (typeIndex == 1) {
                        try {
                            paramManager.set<float>(arg, std::stof(valueArg));
                        } catch (const std::invalid_argument&) {
                            std::cout << "invalid value\n";
                        } catch (const std::out_of_range&) {
                            std::cout << "value out of range\n";
                        }

                    } else if (typeIndex == 2) {
                        if (valueArg == "0" || valueArg == "false")
                            paramManager.set<bool>(arg, false);
                        else if (valueArg == "1" || valueArg == "true")
                            paramManager.set<bool>(arg, true);
                        else
                            std::cout << " invalid argument" << "\n";
                    }
                }
            }

            else if (cmd == "topic" && subcmd == "hz") {
                if (arg.empty()) {
                    std::cout << "usage: topic hz <name>\n";
                } else {
                    ITopic* topic = topicManager.findTopic(arg);

                    if (!topic) {
                        std::cout << "no such topic\n";
                    } else {
                        uint64_t g0 = topic->generation();
                        auto t0 = steady_clock::now();

                        std::this_thread::sleep_for(1s);

                        uint64_t g1 = topic->generation();
                        auto t1 = steady_clock::now();

                        double seconds = duration<double>(t1 - t0).count();

                        double hz = static_cast<double>(g1 - g0) / seconds;

                        std::cout << hz << " Hz\n";
                    }
                }
            } else {
                std::cout << "unknown command\n";
            }
        }
    }
}
