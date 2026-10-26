#include <gtest/gtest.h>
#include "perf_counter.hpp"

// ============================================================================
// PerfCounterTest 單元測試
// ├── SingleMeasurement
// ├── MultipleMeasurements
// ├── Frequency
// ├── EndWithoutBegin
// └── DoubleEnd
// ============================================================================

TEST(PerfCounterTest, SingleMeasurement)
{
    SimulationClock clock;
    PerfCounter perf(clock);

    perf.begin();

    clock.advance(100);

    perf.end();

    EXPECT_EQ(perf.count(), 1);
    EXPECT_EQ(perf.lastUs(), 100);
    EXPECT_EQ(perf.averageUs(), 100);
    EXPECT_EQ(perf.maxUs(), 100);
}

TEST(PerfCounterTest, MultipleMeasurements)
{
    SimulationClock clock;
    PerfCounter perf(clock);

    // 第一次測量：耗時 100 us
    perf.begin();
    clock.advance(100);
    perf.end();

    EXPECT_EQ(perf.count(), 1);
    EXPECT_EQ(perf.lastUs(), 100);
    EXPECT_EQ(perf.averageUs(), 100);
    EXPECT_EQ(perf.maxUs(), 100);

    // 模擬任務之間的間隔 (idle)
    clock.advance(500);

    // 第二次測量：耗時 300 us
    perf.begin();
    clock.advance(300);
    perf.end();

    EXPECT_EQ(perf.count(), 2);
    EXPECT_EQ(perf.lastUs(), 300);
    EXPECT_EQ(perf.averageUs(), 200); // (100 + 300) / 2 = 200
    EXPECT_EQ(perf.maxUs(), 300);

    clock.advance(500);

    // 第三次測量：耗時 200 us
    perf.begin();
    clock.advance(200);
    perf.end();

    EXPECT_EQ(perf.count(), 3);
    EXPECT_EQ(perf.lastUs(), 200);
    EXPECT_EQ(perf.averageUs(), 200); // (100 + 300 + 200) / 3 = 200
    EXPECT_EQ(perf.maxUs(), 300);     // 歷史最大耗時仍維持 300
}

TEST(PerfCounterTest, Frequency)
{
    SimulationClock clock;
    PerfCounter perf(clock);

    // 尚未產生任何週期前，頻率應為 0.0
    EXPECT_DOUBLE_EQ(perf.frequency(), 0.0);

    // 僅執行 1 次測量，只有 1 個起始時間點，尚無週期差
    perf.begin();
    clock.advance(100);
    perf.end();
    EXPECT_DOUBLE_EQ(perf.frequency(), 0.0);

    // 模擬週期性任務，每個週期為 4000 us (250 Hz)
    clock.advance(3900);
    for (int i = 1; i < 10; ++i)
    {
        perf.begin();
        clock.advance(100);
        perf.end();
        clock.advance(3900);
    }

    // 10 次執行共 9 個週期，總週期時間 = 9 * 4000 = 36000 us
    // 頻率 = 9 * 1,000,000 / 36000 = 250.0 Hz
    EXPECT_EQ(perf.count(), 10);
    EXPECT_DOUBLE_EQ(perf.frequency(), 250.0);
}

TEST(PerfCounterTest, EndWithoutBegin)
{
    SimulationClock clock;
    PerfCounter perf(clock);

    clock.advance(100);

    // 未先呼叫 begin() 就直接呼叫 end()，應安全返回而不記錄數據
    perf.end();

    EXPECT_EQ(perf.count(), 0);
    EXPECT_EQ(perf.lastUs(), 0);
    EXPECT_EQ(perf.averageUs(), 0);
    EXPECT_EQ(perf.maxUs(), 0);
    EXPECT_DOUBLE_EQ(perf.frequency(), 0.0);
}

TEST(PerfCounterTest, DoubleEnd)
{
    SimulationClock clock;
    PerfCounter perf(clock);

    perf.begin();
    clock.advance(100);
    perf.end();

    EXPECT_EQ(perf.count(), 1);
    EXPECT_EQ(perf.lastUs(), 100);
    EXPECT_EQ(perf.averageUs(), 100);
    EXPECT_EQ(perf.maxUs(), 100);

    // 連續第二次呼叫 end()，此時 _running 為 false，不應產生額外計數或變更數據
    clock.advance(50);
    perf.end();

    EXPECT_EQ(perf.count(), 1);
    EXPECT_EQ(perf.lastUs(), 100);
    EXPECT_EQ(perf.averageUs(), 100);
    EXPECT_EQ(perf.maxUs(), 100);
}
TEST(PerfCounterTest, DoubleBegin)
{
    SimulationClock clock;
    PerfCounter perf(clock);

    perf.begin();

    clock.advance(50);

    // 應該被忽略，不能重設 start time
    perf.begin();

    clock.advance(50);

    perf.end();

    EXPECT_EQ(perf.count(), 1);
    EXPECT_EQ(perf.lastUs(), 100);
}
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
