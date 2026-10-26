#pragma once

#include <algorithm>
#include <cmath>

namespace px4lite::supplementary {

struct Vector3f {
    float x{0.0f}, y{0.0f}, z{0.0f};
    Vector3f operator+(const Vector3f& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }
    Vector3f operator-(const Vector3f& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }
    Vector3f operator*(float scale) const {
        return {x * scale, y * scale, z * scale};
    }
    float norm() const {
        return std::sqrt(x * x + y * y + z * z);
    }
};

struct AttitudeThrustSetpoint {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float thrust_norm;
};

class MulticopterPositionControl {
public:
    AttitudeThrustSetpoint update(const Vector3f& posSp,
                                  const Vector3f& posActual,
                                  const Vector3f& velActual,
                                  float dt) {
        const Vector3f posErr = posSp - posActual;
        Vector3f velSp;
        velSp.x = std::max(-_maxHorizVel, std::min(_maxHorizVel, _posP * posErr.x));
        velSp.y = std::max(-_maxHorizVel, std::min(_maxHorizVel, _posP * posErr.y));
        velSp.z = std::max(-_maxVerticalVel, std::min(_maxVerticalVel, _posP_z * posErr.z));

        const Vector3f velErr = velSp - velActual;
        _velInt = _velInt + velErr * dt;
        _velInt.x = std::max(-2.0f, std::min(2.0f, _velInt.x));
        _velInt.y = std::max(-2.0f, std::min(2.0f, _velInt.y));
        _velInt.z = std::max(-2.0f, std::min(2.0f, _velInt.z));

        Vector3f desiredAcceleration;
        desiredAcceleration.x = _velP * velErr.x + _velI * _velInt.x;
        desiredAcceleration.y = _velP * velErr.y + _velI * _velInt.y;
        desiredAcceleration.z = _velP_z * velErr.z + _velI_z * _velInt.z + _gravity;

        constexpr float maxTiltRad = 35.0f * (3.14159265f / 180.0f);
        const float horizontalAcceleration = std::sqrt(
            desiredAcceleration.x * desiredAcceleration.x
            + desiredAcceleration.y * desiredAcceleration.y);
        const float maxHorizontalAcceleration = _gravity * std::tan(maxTiltRad);
        if (horizontalAcceleration > maxHorizontalAcceleration) {
            const float scale = maxHorizontalAcceleration / horizontalAcceleration;
            desiredAcceleration.x *= scale;
            desiredAcceleration.y *= scale;
        }

        constexpr float radToDeg = 180.0f / 3.14159265f;
        const float pitchSp = -std::atan2(desiredAcceleration.x, desiredAcceleration.z) * radToDeg;
        const float rollSp = std::atan2(desiredAcceleration.y, desiredAcceleration.z) * radToDeg;
        const float totalAcceleration = desiredAcceleration.norm();
        const float thrustNorm = std::max(
            0.1f, std::min(0.95f, 0.50f * (totalAcceleration / _gravity)));

        return {rollSp, pitchSp, 0.0f, thrustNorm};
    }

private:
    float _gravity{9.81f};
    float _posP{1.0f};
    float _posP_z{1.2f};
    float _velP{1.8f};
    float _velI{0.15f};
    float _velP_z{2.5f};
    float _velI_z{0.3f};
    float _maxHorizVel{5.0f};
    float _maxVerticalVel{2.5f};
    Vector3f _velInt{0.0f, 0.0f, 0.0f};
};

} // namespace px4lite::supplementary
