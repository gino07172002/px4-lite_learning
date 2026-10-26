#include <algorithm>
#include <cmath>
#include <iostream>

struct MotorOutput {
    float m1;
    float m2;
    float m3;
    float m4;
};

class RateController {
public:
    RateController(float kp, float ki, float kd) : _kp(kp), _ki(ki), _kd(kd) {}

    float update(float rateSp, float rate, float dt) {
        if (!std::isfinite(rateSp) || !std::isfinite(rate) ||
            !std::isfinite(dt) || dt <= 0.0f) {
            return 0.0f;
        }

        float error = rateSp - rate;

        _integral += error * dt;
        if (_integral > _integralLimit) {
            _integral = _integralLimit;
        } else if (_integral < -_integralLimit) {
            _integral = -_integralLimit;
        }

        float derivative = 0.0f;
        if (_firstUpdate) {
            _firstUpdate = false;
        } else {
            derivative = -(rate - _lastRate) / dt;
        }

        float torque = _kp * error + _ki * _integral + _kd * derivative;

        _lastRate = rate;

        return std::clamp(torque, -_outputLimit, _outputLimit);
    }

    void reset()
{
    _integral = 0.0f;
    _lastRate = 0.0f;
    _firstUpdate = true;
}

private:
    float _kp = 0.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;

    float _integralLimit = 1.0f;
    float _outputLimit = 1.0f;
    float _integral = 0.0f;
    float _lastRate = 0.0f;
    bool _firstUpdate = true;
};

class ControlAllocator {
public:
    MotorOutput mixRoll(float rollCmd) {
        float base = 0.5f;
        float rollMix = rollCmd * 0.2f;

        MotorOutput out;
        out.m1 = std::clamp(base + rollMix, 0.0f, 1.0f);
        out.m4 = std::clamp(base + rollMix, 0.0f, 1.0f);
        out.m2 = std::clamp(base - rollMix, 0.0f, 1.0f);
        out.m3 = std::clamp(base - rollMix, 0.0f, 1.0f);
        return out;
    }
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

    static float clampMotor(float v) {
        return std::clamp(v, 0.0f, 1.0f);
    }

    void setMotors(float v1, float v2, float v3, float v4) {
        _motor[0] = clampMotor(v1);
        _motor[1] = clampMotor(v2);
        _motor[2] = clampMotor(v3);
        _motor[3] = clampMotor(v4);
    }

    void update(float dt) {
        if (dt <= 0.0f) return;

        float thrust1 = _motor[0] * _maxThrust;
        float thrust2 = _motor[1] * _maxThrust;
        float thrust3 = _motor[2] * _maxThrust;
        float thrust4 = _motor[3] * _maxThrust;

        // 垂直推力與高度動力學
        float totalThrust = thrust1 + thrust2 + thrust3 + thrust4;
        float gravityForce = _mass * _gravity;
        float verticalThrust = totalThrust * std::cos(_state.roll) * std::cos(_state.pitch);
        float netForce = verticalThrust - gravityForce;
        float acceleration = netForce / _mass;
        _state.vz += acceleration * dt;
        _state.z += _state.vz * dt;

        // Roll 軸動力學 (左側: 1,4; 右側: 2,3)
        float leftThrust = thrust1 + thrust4;
        float rightThrust = thrust2 + thrust3;
        float rollTorque =
    (leftThrust - rightThrust) * _armLength
    - _rollDamping * _state.rollRate;
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

    const State& state() const { return _state; }

private:
    State _state;
    float _motor[4]{0.0f, 0.0f, 0.0f, 0.0f};
    float _maxThrust = 5.0f;   // N
    float _mass = 1.0f;        // kg
    float _gravity = 9.81f;    // m/s^2
    float _armLength = 0.2f;   // m
    float _Ixx = 0.02f;        // kg * m^2
    float _Iyy = 0.02f;        // kg * m^2
    float _rollDamping = 0.02f;
};
int main()
{
    RateController controller(
        1.0f,
        0.2f,
        0.005f
    );

    ControlAllocator allocator;
    QuadSimulator sim;

    float dt = 0.01f;

    for (int i = 0; i < 600; ++i)
    {
        float time = i * dt;

        float rateSp;

        if (time < 3.0f) {
            rateSp = 1.0f;
        } else {
            rateSp = -1.0f;
        }

        float rate = sim.state().rollRate;

        float rollCmd =
            controller.update(rateSp, rate, dt);

        MotorOutput motor =
            allocator.mixRoll(rollCmd);

        sim.setMotors(
            motor.m1,
            motor.m2,
            motor.m3,
            motor.m4
        );

        sim.update(dt);

        if (i % 10 == 0) {
            std::cout
                << "t=" << time
                << " sp=" << rateSp
                << " rate=" << sim.state().rollRate
                << " cmd=" << rollCmd
                << '\n';
        }
    }
}
