#pragma once

#include <algorithm>
#include <cmath>

namespace px4lite::lab06 {

// Reusable Lab06 control blocks. The demo imports these implementations
// directly instead of maintaining a second controller.
class RateController {
public:
    RateController(float kp, float ki, float kd) : _kp(kp), _ki(ki), _kd(kd) {}

    float update(float rateSp, float rate, float dt) {
        if (!std::isfinite(rateSp) || !std::isfinite(rate) ||
            !std::isfinite(dt) || dt <= 0.0f) {
            return 0.0f;
        }
        const float error = rateSp - rate;
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
        const float torque = _kp * error + _ki * _integral + _kd * derivative;
        _lastRate = rate;
        return std::clamp(torque, -_outputLimit, _outputLimit);
    }

    void reset() {
        _integral = 0.0f;
        _lastRate = 0.0f;
        _firstUpdate = true;
    }

private:
    float _kp{0.0f};
    float _ki{0.0f};
    float _kd{0.0f};
    float _integralLimit{1.0f};
    float _outputLimit{1.0f};
    float _integral{0.0f};
    float _lastRate{0.0f};
    bool _firstUpdate{true};
};

class AttitudeController {
public:
    explicit AttitudeController(float kp) : _kp(kp) {}
    float update(float angleSp, float angle) const {
        return _kp * (angleSp - angle);
    }

private:
    float _kp;
};

} // namespace px4lite::lab06
