#pragma once

#include <ostream>

enum class State {
    INIT,
    STANDBY,
    ARMED,
    TAKEOFF,
    FLYING,
    LANDING,
    FAILSAFE
};

const char* stateToString(State state);

std::ostream& operator<<(std::ostream& os, State state);

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
    Commander();
    State state() const;
    bool init();
    bool arm(const HealthStatus& status);
    bool disarm();
    bool takeoff(const HealthStatus& status);
    bool land();
    void update(const VehicleStatus& vehicleStatus,
                const HealthStatus& status);

private:
    State _state;
    bool _failsafeActive = false;
};
