#include <iostream>
#include <cstdint>
#include <thread>
#include <chrono>
#include <unordered_map>
#include <variant>
#include <mutex>
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
    RealtimeClock() :startTime(std::chrono::steady_clock::now()) {

    }

    uint64_t nowUs() const override
    {
        auto now =  std::chrono::steady_clock::now();

        return std::chrono::duration_cast<std::chrono::microseconds>(now - startTime).count();
    }


    private:
    std::chrono::steady_clock::time_point startTime;
};
class Timer
{
public:
    Timer(Clock& clock):clock(clock), startTime(clock.nowUs()) {}
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


//3b

using ParamValue =
    std::variant<int32_t, float, bool>;

class ParameterManager
{
    public :
    template<typename T>
void set(const std::string& name, T value)
{
    std::lock_guard<std::mutex> lock(mtx); // 確保線程安全
    params[name] = value;
    ++generation; // 更新參數版本
}

template<typename T>
T get(const std::string& name) const
{
    std::lock_guard<std::mutex> lock(mtx); // 確保線程安全
    // params.at(name) 在找不到 key 時會拋出 std::out_of_range
    // std::get<T>(...) 在型別不符時會拋出 std::bad_variant_access
    return std::get<T>(params.at(name));
}
uint32_t getGeneration() const
{
    std::lock_guard<std::mutex> lock(mtx); // 確保線程安全
    return generation;
}
private:
    std::unordered_map<std::string, ParamValue> params;
    mutable std::mutex mtx; // 用於保護 params 的互斥鎖
    uint32_t generation = 0; // 用於追蹤參數的版本或更新次數
};

int main()
{
     SimulationClock clock;


     Timer timer1(clock);

   std::cout << timer1.elapsedUs() << '\n';
  Timer timer3(clock);
    clock.advance(4000);
    std::cout << timer1.elapsedUs() << '\n';

    clock.advance(4000);
    std::cout << timer1.elapsedUs() << '\n';

std::cout << timer3.elapsedUs() << '\n';

    RealtimeClock clock2;
    Timer timer2(clock2);
    std::cout << timer2.elapsedUs() << '\n';

    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << timer2.elapsedUs() << '\n';

    std::cout<<" ready to write new homework for clock interface "<<std::endl;


    //3b
    ParameterManager params;

params.set<float>("MC_ROLL_P", 4.0f);
params.set<int32_t>("SYS_ID", 1);
params.set<bool>("SIM_ENABLE", true);

float p = params.get<float>("MC_ROLL_P");
std::cout << "MC_ROLL_P: " << p << std::endl;
int32_t id = params.get<int32_t>("SYS_ID");
std::cout << "SYS_ID: " << id << std::endl;
bool sim = params.get<bool>("SIM_ENABLE");
std::cout << "SIM_ENABLE: " << std::boolalpha << sim << std::endl;
try
{
    bool nothing = params.get<bool>("NOTHING_ENABLE");
}
catch (const std::out_of_range& e)
{
    std::cout << "Parameter not found" << std::endl;
}
catch (const std::bad_variant_access& e)
{
    std::cout << "Parameter type mismatch" << std::endl;
}


params.set<int32_t>("COUNTER", 0);

std::thread writer([&]()
{
    for (int i = 0; i < 100000; ++i)
    {
        params.set<int32_t>("COUNTER", i);
    }
});

std::thread reader([&]()
{
    for (int i = 0; i < 100000; ++i)
    {
        auto value = params.get<int32_t>("COUNTER");
    }
});

writer.join();
reader.join();

std::cout << params.get<int32_t>("COUNTER") << '\n';
    return 0;
}
