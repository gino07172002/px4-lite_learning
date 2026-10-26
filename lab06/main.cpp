#include "control_stack.hpp"
#include <iostream>

using px4lite::lab06::AttitudeController;
using px4lite::lab06::RateController;

constexpr float DEG_TO_RAD = 3.1415926535f / 180.0f;
struct MotorOutput {
    float m1;
    float m2;
    float m3;
    float m4;
};

class RollAllocator {
public:
    RollAllocator(float maxThrust, float armLength)
        : _maxThrust(maxThrust), _armLength(armLength) {}

    MotorOutput allocate(float baseThrottle, float rollTorque) {
        float delta = rollTorque / (4.0f * _maxThrust * _armLength);

        MotorOutput out;

        out.m1 = baseThrottle + delta;
        out.m4 = baseThrottle + delta;

        out.m2 = baseThrottle - delta;
        out.m3 = baseThrottle - delta;

        return out;
    }

private:
    float _maxThrust;
    float _armLength;
};

class QuadSimulator {
public:
    QuadSimulator() = default;
    ~QuadSimulator() = default;

    struct State {
        float roll = 0.0f;
        float pitch = 0.0f;
        float pitchRate = 0.0f;
        float rollRate = 0.0f;
        float z = 0.0f;
        float vz = 0.0f;
    };

    static float clampMotor(float v) { return std::clamp(v, 0.0f, 1.0f); }

    void setMotors(float v1, float v2, float v3, float v4) {
        _motor[0] = clampMotor(v1);
        _motor[1] = clampMotor(v2);
        _motor[2] = clampMotor(v3);
        _motor[3] = clampMotor(v4);
    }

    void update(float dt) {
        if (dt <= 0.0f)
            return;

        float thrust1 = _motor[0] * _maxThrust;
        float thrust2 = _motor[1] * _maxThrust;
        float thrust3 = _motor[2] * _maxThrust;
        float thrust4 = _motor[3] * _maxThrust;

        // 垂直推力與高度動力學
        float totalThrust = thrust1 + thrust2 + thrust3 + thrust4;
        float gravityForce = _mass * _gravity;
        float verticalThrust =
            totalThrust * std::cos(_state.roll) * std::cos(_state.pitch);
        float netForce = verticalThrust - gravityForce;
        float acceleration = netForce / _mass;
        _state.vz += acceleration * dt;
        _state.z += _state.vz * dt;

        // Roll 軸動力學 (左側: 1,4; 右側: 2,3)
        float leftThrust = thrust1 + thrust4;
        float rightThrust = thrust2 + thrust3;
        float rollTorque = (leftThrust - rightThrust) * _armLength -
                           _rollDamping * _state.rollRate;
        float rollAcceleration = rollTorque / _Ixx;
        _state.rollRate += rollAcceleration * dt;
        _state.roll += _state.rollRate * dt;

        // Pitch 軸動力學 (前側: 1,2; 後側: 4,3)
        float frontThrust = thrust1 + thrust2;
        float backThrust = thrust4 + thrust3;
        float pitchTorque = (frontThrust - backThrust) * _armLength;
        float pitchAcceleration = pitchTorque / _Iyy;
        _state.pitchRate += pitchAcceleration * dt;
        _state.pitch += _state.pitchRate * dt;
    }
    void setRoll(float roll) { _state.roll = roll; }
    const State& state() const { return _state; }

private:
    State _state;
    float _motor[4]{0.0f, 0.0f, 0.0f, 0.0f};
    float _maxThrust = 5.0f; // N
    float _mass = 1.0f;      // kg
    float _gravity = 9.81f;  // m/s^2
    float _armLength = 0.2f; // m
    float _Ixx = 0.02f;      // kg * m^2
    float _Iyy = 0.02f;      // kg * m^2
    float _rollDamping = 0.02f;
};

struct VehicleAttitude
{
    float roll = 0.0f;
    float pitch = 0.0f;
};
struct VehicleAngularVelocity
{
    float rollRate = 0.0f;
    float pitchRate = 0.0f;
};


class GroundTruthEstimator
{
public:
    VehicleAttitude attitude(const QuadSimulator::State& state)
    {
        return {state.roll,state.pitch};

    }
    VehicleAngularVelocity angularVelocity(const QuadSimulator::State& state)
    {
        return {state.rollRate,state.pitchRate};
    }
};

struct ImuSample
{
    float gyroX = 0.0f;
    float gyroY = 0.0f;
    float gyroZ = 0.0f;

    float accelX = 0.0f;
    float accelY = 0.0f;
    float accelZ = 0.0f;
};
class ImuSensor
{
public:
    ImuSample sample(const QuadSimulator::State& state)
    {
        constexpr float g = 9.81f;

        ImuSample imu;

        imu.gyroX = state.rollRate + 0.01f;
        imu.gyroY = state.pitchRate;
        imu.gyroZ = 0.0f;

        imu.accelX =
            -g * std::sin(state.pitch);

        imu.accelY =
            g * std::sin(state.roll) *
            std::cos(state.pitch);

        imu.accelZ =
            g * std::cos(state.roll) *
            std::cos(state.pitch);

        return imu;
    }
};

class GyroRollEstimator
{
public:
    float update(const ImuSample& imu, float dt)
    {
        _roll += imu.gyroX * dt;
        return _roll;
    }

private:
    float _roll = 0.0f;
};

class ComplementaryFilter
{
public:
    explicit ComplementaryFilter(float alpha)
    : _alpha(std::clamp(alpha, 0.0f, 1.0f))
{
}

    float update(const ImuSample& imu, float dt)
{
    if (!std::isfinite(dt) || dt <= 0.0f)
        return _roll;

    float accelRoll =
        std::atan2(imu.accelY, imu.accelZ);

    if (!_initialized)
    {
        _roll = accelRoll;
        _initialized = true;
        return _roll;
    }

    float gyroPrediction =
        _roll + imu.gyroX * dt;

    _roll =
        _alpha * gyroPrediction
        + (1.0f - _alpha) * accelRoll;

    return _roll;
}

private:
    float _alpha;
    float _roll = 0.0f;
    bool _initialized = false;
};
int main() {
    AttitudeController attitudeController(1.0f);
    GroundTruthEstimator estimator;
    ImuSensor imuSensor;

    RateController rateController(0.1f, 0.1f, 0.001f);

    RollAllocator allocator(5.0f, 0.2f);

    constexpr float dt = 0.01f;
    constexpr float baseThrottle = 0.4905f;

    float angleSp = 0.0f;

    QuadSimulator sim{};
    sim.setRoll(15.0f * DEG_TO_RAD);
GyroRollEstimator gyroEstimator;
    ComplementaryFilter filter(0.98f);
  for (int i = 0; i < 500; ++i)
{
    // 這個 timestep 的 sensor reading
    ImuSample imu =
        imuSensor.sample(sim.state());

    float gyroRoll =
        gyroEstimator.update(imu, dt);

    float accelRoll =
        std::atan2(
            imu.accelY,
            imu.accelZ
        );

    VehicleAttitude attitude =
        estimator.attitude(sim.state());


    float filteredRoll=  filter.update(imu, dt);
    float rateSp =
        attitudeController.update(
            angleSp,
            filteredRoll
        );

    float torque =
        rateController.update(
            rateSp,
            imu.gyroX,
            dt
        );

    MotorOutput motor =
        allocator.allocate(
            baseThrottle,
            torque
        );

    sim.setMotors(
        motor.m1,
        motor.m2,
        motor.m3,
        motor.m4
    );

    sim.update(dt);

    if (i % 10 == 0)
    {
        std::cout
    << "t=" << i * dt

    << " truthRoll="
    << attitude.roll / DEG_TO_RAD

    << " accelRoll="
    << accelRoll / DEG_TO_RAD

    << " gyroRoll="
    << gyroRoll / DEG_TO_RAD

    << " filteredRoll="
    << filteredRoll / DEG_TO_RAD

    << " gyroX="
    << imu.gyroX

    << '\n';
    }
}
}
