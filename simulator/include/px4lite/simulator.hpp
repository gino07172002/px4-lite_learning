#pragma once

#include <array>
#include <cstddef>

namespace px4lite::simulator {

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct Quaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct EulerAngles {
    double rollRad{0.0};
    double pitchRad{0.0};
    double yawRad{0.0};
};

struct MotorCommands {
    // PX4-Lite / Lab04 order: M1 front-right, M2 rear-left,
    // M3 front-left, M4 rear-right. Values are normalized [0, 1].
    std::array<double, 4> normalized{{0.0, 0.0, 0.0, 0.0}};
};

struct VehicleState {
    // PX4-style local NED coordinates: north, east, down (metres).
    Vec3 positionNedM{};
    Vec3 velocityNedMps{};
    Vec3 angularVelocityBodyRadS{};
    Quaternion attitudeNedFromBody{};
    EulerAngles eulerRad{};
    std::array<double, 4> motorOutput{{0.0, 0.0, 0.0, 0.0}};
    double simulationTimeS{0.0};
    bool onGround{true};
};

// A backend boundary: SimpleQuadSimulator and a future Gazebo adapter expose
// the same fixed-tick actuator/state contract to PX4-Lite.
class IQuadSimulator {
public:
    virtual ~IQuadSimulator() = default;

    virtual bool setMotorCommands(const MotorCommands& commands) noexcept = 0;
    virtual void step() noexcept = 0;
    virtual void reset() noexcept = 0;
    virtual const VehicleState& state() const noexcept = 0;
    virtual double fixedDtSeconds() const noexcept = 0;
};

} // namespace px4lite::simulator
