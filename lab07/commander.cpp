#include "commander.hpp"

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

std::ostream& operator<<(std::ostream& os, State state) {
    return os << stateToString(state);
}

Commander::Commander() : _state(State::INIT) {}

State Commander::state() const {
    return _state;
}

bool Commander::init() {
    if (_state != State::INIT) {
        return false;
    }
    _state = State::STANDBY;
    return true;
}

bool Commander::arm(const HealthStatus& status) {
    if (_state != State::STANDBY)
        return false;
    if (!status.imuOk || !status.estimatorOk || !status.simulatorOk)
        return false;

    _state = State::ARMED;
    return true;
}

bool Commander::disarm() {
    if (_state != State::ARMED)
        return false;

    _state = State::STANDBY;
    return true;
}

bool Commander::takeoff(const HealthStatus& status) {
    if (_state != State::ARMED)
        return false;
    if (!status.imuOk || !status.estimatorOk || !status.simulatorOk)
        return false;

    _state = State::TAKEOFF;
    return true;
}

bool Commander::land() {
    if (_state != State::FLYING)
        return false;
    _state = State::LANDING;
    return true;
    // 只有 FLYING 才允許
}

void Commander::update(const VehicleStatus& vehicleStatus,
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
