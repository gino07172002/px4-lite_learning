#pragma once

#include "px4lite/simulator.hpp"

namespace px4lite::simulator {

class SimpleQuadSimulator final : public IQuadSimulator {
public:
    struct Config {
        double fixedDtSeconds{0.004};       // 250 Hz
        double massKg{1.2};
        double gravityMps2{9.81};
        double maxThrustPerMotorN{6.0};
        double armLengthM{0.15};
        double yawTorqueRatio{0.02};
        double inertiaXxKgM2{0.008};
        double inertiaYyKgM2{0.008};
        double inertiaZzKgM2{0.015};
        double motorTimeConstantS{0.03};
        double linearDragNPerMps{0.18};
        double angularDampingNmPerRadS{0.02};
    };

    SimpleQuadSimulator();
    explicit SimpleQuadSimulator(const Config& config);

    bool setMotorCommands(const MotorCommands& commands) noexcept override;
    void step() noexcept override;
    void reset() noexcept override;

    const VehicleState& state() const noexcept override { return _state; }
    double fixedDtSeconds() const noexcept override { return _config.fixedDtSeconds; }
    const Config& config() const noexcept { return _config; }

private:
    static bool validConfig(const Config& config) noexcept;
    static Vec3 rotateBodyVectorToNed(const Quaternion& q, const Vec3& v) noexcept;
    static EulerAngles toEuler(const Quaternion& q) noexcept;
    static void normalize(Quaternion& q) noexcept;

    Config _config{};
    VehicleState _state{};
    MotorCommands _targetMotors{};
};

} // namespace px4lite::simulator
