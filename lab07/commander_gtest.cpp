#include <gtest/gtest.h>
#include "commander.hpp"

// ============================================================================
// GoogleTest 版本
// ============================================================================

TEST(CommanderTest, Init) {
    Commander commander{};

    EXPECT_TRUE(commander.init());

    EXPECT_EQ(commander.state(), State::STANDBY);
}

TEST(CommanderTest, InitTwiceRejected) {
    Commander commander{};

    EXPECT_TRUE(commander.init());

    EXPECT_EQ(commander.state(), State::STANDBY);
    EXPECT_FALSE(commander.init());

    EXPECT_EQ(commander.state(), State::STANDBY);
}

TEST(CommanderTest, ArmHealthy) {
    Commander commander{};

    HealthStatus health{
        true,
        true,
        true
    };

    EXPECT_TRUE(commander.init());
    EXPECT_TRUE(commander.arm(health));

    EXPECT_EQ(commander.state(), State::ARMED);
}

TEST(CommanderTest, ArmUnhealthy) {
    Commander commander{};

    HealthStatus health{
        true,
        false,
        true
    };

    EXPECT_TRUE(commander.init());
    EXPECT_FALSE(commander.arm(health));

    EXPECT_EQ(commander.state(), State::STANDBY);
}

TEST(CommanderTest, Disarm) {
    Commander commander{};

    HealthStatus health{
        true,
        true,
        true
    };

    EXPECT_TRUE(commander.init());
    EXPECT_TRUE(commander.arm(health));

    EXPECT_EQ(commander.state(), State::ARMED);
    EXPECT_TRUE(commander.disarm());

    EXPECT_EQ(commander.state(), State::STANDBY);
}

TEST(CommanderTest, Takeoff) {
    Commander commander{};

    HealthStatus health{
        true,
        true,
        true
    };

    EXPECT_TRUE(commander.init());
    EXPECT_TRUE(commander.arm(health));

    EXPECT_EQ(commander.state(), State::ARMED);
    EXPECT_TRUE(commander.takeoff(health));
    EXPECT_EQ(commander.state(), State::TAKEOFF);
}

TEST(CommanderTest, TakeoffBecomesFlying) {
    Commander commander{};

    HealthStatus health{
        true,
        true,
        true
    };

    EXPECT_TRUE(commander.init());
    EXPECT_TRUE(commander.arm(health));

    EXPECT_EQ(commander.state(), State::ARMED);
    EXPECT_TRUE(commander.takeoff(health));
    EXPECT_EQ(commander.state(), State::TAKEOFF);
    VehicleStatus vehicleStatus{true};
    commander.update(vehicleStatus, health);
    EXPECT_EQ(commander.state(), State::FLYING);
}

TEST(CommanderTest, Land) {
    Commander commander{};

    HealthStatus health{
        true,
        true,
        true
    };

    EXPECT_TRUE(commander.init());
    EXPECT_TRUE(commander.arm(health));

    EXPECT_EQ(commander.state(), State::ARMED);
    EXPECT_TRUE(commander.takeoff(health));
    EXPECT_EQ(commander.state(), State::TAKEOFF);
    VehicleStatus vehicleStatus{true};
    commander.update(vehicleStatus, health);
    EXPECT_EQ(commander.state(), State::FLYING);
    EXPECT_TRUE(commander.land());
    EXPECT_EQ(commander.state(), State::LANDING);
}

TEST(CommanderTest, LandingBecomesStandby) {
    Commander commander{};

    HealthStatus health{
        true,
        true,
        true
    };

    EXPECT_TRUE(commander.init());
    EXPECT_TRUE(commander.arm(health));

    EXPECT_EQ(commander.state(), State::ARMED);
    EXPECT_TRUE(commander.takeoff(health));
    EXPECT_EQ(commander.state(), State::TAKEOFF);
    VehicleStatus vehicleStatus{true};
    commander.update(vehicleStatus, health);
    EXPECT_EQ(commander.state(), State::FLYING);
    EXPECT_TRUE(commander.land());
    EXPECT_EQ(commander.state(), State::LANDING);
    vehicleStatus.airborne = false;
    commander.update(vehicleStatus, health);
    EXPECT_EQ(commander.state(), State::STANDBY);
}

TEST(CommanderTest, FailsafeLatch) {
    Commander commander{};

    HealthStatus goodHealth{
        true,
        true,
        true
    };

    HealthStatus badHealth{
        false,
        true,
        true
    };

    VehicleStatus vehicleStatus{false};

    EXPECT_TRUE(commander.init());
    EXPECT_TRUE(commander.arm(goodHealth));
    EXPECT_TRUE(commander.takeoff(goodHealth));

    // TAKEOFF -> FLYING
    vehicleStatus.airborne = true;
    commander.update(vehicleStatus, goodHealth);

    EXPECT_EQ(commander.state(), State::FLYING);

    // FLYING -> FAILSAFE
    commander.update(vehicleStatus, badHealth);

    EXPECT_EQ(commander.state(), State::FAILSAFE);

    // FAILSAFE -> LANDING
    commander.update(vehicleStatus, badHealth);

    EXPECT_EQ(commander.state(), State::LANDING);

    // 故障持續存在，但不能重新跳回 FAILSAFE
    commander.update(vehicleStatus, badHealth);

    EXPECT_EQ(commander.state(), State::LANDING);

    commander.update(vehicleStatus, badHealth);

    EXPECT_EQ(commander.state(), State::LANDING);

    // 真正落地
    vehicleStatus.airborne = false;
    commander.update(vehicleStatus, badHealth);

    EXPECT_EQ(commander.state(), State::STANDBY);

    // health 還沒恢復，不允許重新 ARM
    EXPECT_FALSE(commander.arm(badHealth));
    EXPECT_EQ(commander.state(), State::STANDBY);
}

TEST(CommanderTest, CannotDisarmFromStandby) {
    Commander commander{};
    EXPECT_TRUE(commander.init());

    EXPECT_FALSE(commander.disarm());
    EXPECT_EQ(commander.state(), State::STANDBY);
}

TEST(CommanderTest, CannotTakeoffFromStandby) {
    Commander commander{};

    HealthStatus health{true, true, true};

    EXPECT_TRUE(commander.init());

    EXPECT_FALSE(commander.takeoff(health));
    EXPECT_EQ(commander.state(), State::STANDBY);
}

TEST(CommanderTest, CannotLandWhenNotFlying) {
    Commander commander{};

    EXPECT_FALSE(commander.land());
    EXPECT_EQ(commander.state(), State::INIT);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
