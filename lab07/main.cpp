#include <cassert>
#include <iostream>

enum class State { INIT, STANDBY, ARMED, TAKEOFF, FLYING, LANDING, FAILSAFE };
const char* stateToString(State state) {
    switch (state) {
    case State::INIT:
        return "INIT";

    case State::STANDBY:
        return "STANDBY";

    case State::ARMED:
        return "ARMED";

    case State::TAKEOFF:
        return "TAKEOFF";

    case State::FLYING:
        return "FLYING";

    case State::LANDING:
        return "LANDING";

    case State::FAILSAFE:
        return "FAILSAFE";
    }

    return "UNKNOWN";
}
struct HealthStatus {
    bool imuOk = false;
    bool estimatorOk = false;
    bool simulatorOk = false;
};
struct VehicleStatus {
    bool airborne = false;
};
class Commander {
public:
    Commander() : _state(State::INIT) {}
    State state() const { return _state; }
    bool init() {
        if (_state != State::INIT) {
            return false;
        }
        _state = State::STANDBY;
        return true;
    }
    bool arm(const HealthStatus& status) {
        if (_state != State::STANDBY)
            return false;
        if (!status.imuOk || !status.estimatorOk || !status.simulatorOk)
            return false;

        _state = State::ARMED;
        return true;
    }
    bool disarm() {

        if (_state != State::ARMED)
            return false;

        _state = State::STANDBY;
        return true;
    }
    bool takeoff(const HealthStatus& status) {

        if (_state != State::ARMED)
            return false;
        if (!status.imuOk || !status.estimatorOk || !status.simulatorOk)
            return false;

        _state = State::TAKEOFF;
        return true;
    }
    bool land() {
        if (_state != State::FLYING)
            return false;
        _state = State::LANDING;
        return true;
        // 只有 FLYING 才允許
    }
    void update(const VehicleStatus& vehicleStatus,
                const HealthStatus& status) {
        if (_failsafeActive) {
            if (_state == State::FAILSAFE) {
                _state = State::LANDING;
                return;
            }

            if (_state == State::LANDING) {
                if (!vehicleStatus.airborne) {
                    _state = State::STANDBY;
                    _failsafeActive = false;
                }

                return;
            }
        }

        if (_state == State::LANDING && !vehicleStatus.airborne) {
            _state = State::STANDBY;
        }
        bool unhealthy =
            !status.imuOk || !status.estimatorOk || !status.simulatorOk;

        if (!_failsafeActive && unhealthy &&
            (_state == State::TAKEOFF || _state == State::FLYING ||
             _state == State::LANDING)) {
            _state = State::FAILSAFE;
            _failsafeActive = true;
            return;
        }

        if (_state == State::TAKEOFF && vehicleStatus.airborne) {
            _state = State::FLYING;
        }
    }

private:
    State _state;
    bool _failsafeActive = false;
};
void testInit() {
      Commander commander{};



    assert(commander.init());

    assert(commander.state() == State::STANDBY);
};
void testInitTwiceRejected() {
       Commander commander{};



    assert(commander.init());

    assert(commander.state() == State::STANDBY);
         assert(!commander.init());

    assert(commander.state() ==  State::STANDBY);
};

void testArmHealthy() {
      Commander commander{};

    HealthStatus health{
        true,
        true,
        true
    };

    assert(commander.init());
    assert(commander.arm(health));

    assert(commander.state() == State::ARMED);
};
void testArmUnhealthy() {
    Commander commander{};

    HealthStatus health{
        true,
        false,
        true
    };

    assert(commander.init());
    assert(!commander.arm(health));

    assert(commander.state() == State::STANDBY);
};
void testDisarm() {
      Commander commander{};

    HealthStatus health{
        true,
        true,
        true
    };

    assert(commander.init());
    assert(commander.arm(health));

    assert(commander.state() == State::ARMED);
     assert(commander.disarm());

    assert(commander.state() == State::STANDBY);
};

void testTakeoff() {
     Commander commander{};

    HealthStatus health{
        true,
        true,
        true
    };

    assert(commander.init());
    assert(commander.arm(health));

    assert(commander.state() == State::ARMED);
     assert(commander.takeoff(health));
     assert(commander.state() == State::TAKEOFF);



};
void testTakeoffBecomesFlying() {
     Commander commander{};

    HealthStatus health{
        true,
        true,
        true
    };

    assert(commander.init());
    assert(commander.arm(health));

    assert(commander.state() == State::ARMED);
     assert(commander.takeoff(health));
     assert(commander.state() == State::TAKEOFF);
     VehicleStatus vehicleStatus{true};
     commander.update(vehicleStatus, health);
    assert(commander.state() == State::FLYING);
};

void testLand() {
        Commander commander{};

    HealthStatus health{
        true,
        true,
        true
    };

    assert(commander.init());
    assert(commander.arm(health));

    assert(commander.state() == State::ARMED);
     assert(commander.takeoff(health));
     assert(commander.state() == State::TAKEOFF);
     VehicleStatus vehicleStatus{true};
     commander.update(vehicleStatus, health);
    assert(commander.state() == State::FLYING);
     assert(commander.land());
    assert(commander.state() == State::LANDING);
};
void testLandingBecomesStandby() {
        Commander commander{};

    HealthStatus health{
        true,
        true,
        true
    };

    assert(commander.init());
    assert(commander.arm(health));

    assert(commander.state() == State::ARMED);
     assert(commander.takeoff(health));
     assert(commander.state() == State::TAKEOFF);
     VehicleStatus vehicleStatus{true};
     commander.update(vehicleStatus, health);
    assert(commander.state() == State::FLYING);
     assert(commander.land());
    assert(commander.state() == State::LANDING);
      vehicleStatus.airborne=false;
     commander.update(vehicleStatus, health);
     assert(commander.state() == State::STANDBY);
};

void testFailsafeLatch() {
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

    assert(commander.init());
    assert(commander.arm(goodHealth));
    assert(commander.takeoff(goodHealth));

    // TAKEOFF -> FLYING
    vehicleStatus.airborne = true;
    commander.update(vehicleStatus, goodHealth);

    assert(commander.state() == State::FLYING);

    // FLYING -> FAILSAFE
    commander.update(vehicleStatus, badHealth);

    assert(commander.state() == State::FAILSAFE);

    // FAILSAFE -> LANDING
    commander.update(vehicleStatus, badHealth);

    assert(commander.state() == State::LANDING);

    // 故障持續存在，但不能重新跳回 FAILSAFE
    commander.update(vehicleStatus, badHealth);

    assert(commander.state() == State::LANDING);

    commander.update(vehicleStatus, badHealth);

    assert(commander.state() == State::LANDING);

    // 真正落地
    vehicleStatus.airborne = false;
    commander.update(vehicleStatus, badHealth);

    assert(commander.state() == State::STANDBY);

    // health 還沒恢復，不允許重新 ARM
    assert(!commander.arm(badHealth));
    assert(commander.state() == State::STANDBY);

};
void testCannotDisarmFromStandby()
{
    Commander commander{};
    assert(commander.init());

    assert(!commander.disarm());
    assert(commander.state() == State::STANDBY);
}

void testCannotTakeoffFromStandby()
{
    Commander commander{};

    HealthStatus health{true, true, true};

    assert(commander.init());

    assert(!commander.takeoff(health));
    assert(commander.state() == State::STANDBY);
}

void testCannotLandWhenNotFlying()
{
    Commander commander{};

    assert(!commander.land());
    assert(commander.state() == State::INIT);
}
int main() {


    testInit();
    testInitTwiceRejected();

    testArmHealthy();
    testArmUnhealthy();
    testDisarm();

    testTakeoff();
    testTakeoffBecomesFlying();

    testLand();
    testLandingBecomesStandby();

    testFailsafeLatch();
    testCannotDisarmFromStandby();
     testCannotTakeoffFromStandby();
testCannotLandWhenNotFlying();
    std::cout << "All tests passed\n";
}
