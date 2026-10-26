#include "px4lite/simple_quad_simulator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4lite::simulator {
namespace {

constexpr double kArmProjection = 0.7071067811865475244;

Vec3 cross(const Vec3& a, const Vec3& b) noexcept
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

double clampUnit(double value) noexcept
{
    return std::clamp(value, 0.0, 1.0);
}

} // namespace

SimpleQuadSimulator::SimpleQuadSimulator() = default;

SimpleQuadSimulator::SimpleQuadSimulator(const Config& config)
    : _config(config)
{
    if (!validConfig(config)) {
        throw std::invalid_argument("SimpleQuadSimulator has an invalid physical configuration");
    }
}

bool SimpleQuadSimulator::validConfig(const Config& c) noexcept
{
    return std::isfinite(c.fixedDtSeconds) && c.fixedDtSeconds > 0.0
        && std::isfinite(c.massKg) && c.massKg > 0.0
        && std::isfinite(c.gravityMps2) && c.gravityMps2 > 0.0
        && std::isfinite(c.maxThrustPerMotorN) && c.maxThrustPerMotorN > 0.0
        && std::isfinite(c.armLengthM) && c.armLengthM > 0.0
        && std::isfinite(c.yawTorqueRatio) && c.yawTorqueRatio >= 0.0
        && std::isfinite(c.inertiaXxKgM2) && c.inertiaXxKgM2 > 0.0
        && std::isfinite(c.inertiaYyKgM2) && c.inertiaYyKgM2 > 0.0
        && std::isfinite(c.inertiaZzKgM2) && c.inertiaZzKgM2 > 0.0
        && std::isfinite(c.motorTimeConstantS) && c.motorTimeConstantS >= 0.0
        && std::isfinite(c.linearDragNPerMps) && c.linearDragNPerMps >= 0.0
        && std::isfinite(c.angularDampingNmPerRadS) && c.angularDampingNmPerRadS >= 0.0;
}

bool SimpleQuadSimulator::setMotorCommands(const MotorCommands& commands) noexcept
{
    for (const double value : commands.normalized) {
        if (!std::isfinite(value)) {
            return false;
        }
    }

    for (std::size_t i = 0; i < commands.normalized.size(); ++i) {
        _targetMotors.normalized[i] = clampUnit(commands.normalized[i]);
    }
    return true;
}

void SimpleQuadSimulator::step() noexcept
{
    const double dt = _config.fixedDtSeconds;
    const double motorAlpha = _config.motorTimeConstantS == 0.0
        ? 1.0
        : dt / (_config.motorTimeConstantS + dt);

    std::array<double, 4> thrustN{};
    double totalThrustN = 0.0;
    for (std::size_t i = 0; i < thrustN.size(); ++i) {
        double& actual = _state.motorOutput[i];
        actual += motorAlpha * (_targetMotors.normalized[i] - actual);
        actual = clampUnit(actual);
        thrustN[i] = actual * _config.maxThrustPerMotorN;
        totalThrustN += thrustN[i];
    }

    // Matches the motor order and roll/yaw torque signs documented in Lab04.
    const double d = _config.armLengthM * kArmProjection;
    const double tauX = d * ((thrustN[1] + thrustN[2]) - (thrustN[0] + thrustN[3]));
    // Positive pitch follows the right-hand FRD body convention (nose-up):
    // front thrust exceeds rear thrust.
    const double tauY = d * ((thrustN[0] + thrustN[2]) - (thrustN[1] + thrustN[3]));
    const double tauZ = _config.yawTorqueRatio
        * ((thrustN[2] + thrustN[3]) - (thrustN[0] + thrustN[1]));

    const Vec3& omega = _state.angularVelocityBodyRadS;
    const Vec3 angularMomentum{
        _config.inertiaXxKgM2 * omega.x,
        _config.inertiaYyKgM2 * omega.y,
        _config.inertiaZzKgM2 * omega.z
    };
    const Vec3 gyroscopic = cross(omega, angularMomentum);

    _state.angularVelocityBodyRadS.x +=
        (tauX - gyroscopic.x - _config.angularDampingNmPerRadS * omega.x)
        / _config.inertiaXxKgM2 * dt;
    _state.angularVelocityBodyRadS.y +=
        (tauY - gyroscopic.y - _config.angularDampingNmPerRadS * omega.y)
        / _config.inertiaYyKgM2 * dt;
    _state.angularVelocityBodyRadS.z +=
        (tauZ - gyroscopic.z - _config.angularDampingNmPerRadS * omega.z)
        / _config.inertiaZzKgM2 * dt;

    // Integrate q_dot = 1/2 * q * [0, omega_body], then renormalize.
    const Quaternion q = _state.attitudeNedFromBody;
    const Vec3 updatedOmega = _state.angularVelocityBodyRadS;
    _state.attitudeNedFromBody = {
        q.w - 0.5 * (q.x * updatedOmega.x + q.y * updatedOmega.y + q.z * updatedOmega.z) * dt,
        q.x + 0.5 * (q.w * updatedOmega.x + q.y * updatedOmega.z - q.z * updatedOmega.y) * dt,
        q.y + 0.5 * (q.w * updatedOmega.y + q.z * updatedOmega.x - q.x * updatedOmega.z) * dt,
        q.z + 0.5 * (q.w * updatedOmega.z + q.x * updatedOmega.y - q.y * updatedOmega.x) * dt
    };
    normalize(_state.attitudeNedFromBody);

    const Vec3 thrustBody{0.0, 0.0, -totalThrustN};
    const Vec3 thrustNed = rotateBodyVectorToNed(_state.attitudeNedFromBody, thrustBody);
    const Vec3& velocity = _state.velocityNedMps;
    const Vec3 accelerationNed{
        thrustNed.x / _config.massKg - _config.linearDragNPerMps * velocity.x / _config.massKg,
        thrustNed.y / _config.massKg - _config.linearDragNPerMps * velocity.y / _config.massKg,
        _config.gravityMps2 + thrustNed.z / _config.massKg
            - _config.linearDragNPerMps * velocity.z / _config.massKg
    };

    _state.velocityNedMps.x += accelerationNed.x * dt;
    _state.velocityNedMps.y += accelerationNed.y * dt;
    _state.velocityNedMps.z += accelerationNed.z * dt;
    _state.positionNedM.x += _state.velocityNedMps.x * dt;
    _state.positionNedM.y += _state.velocityNedMps.y * dt;
    _state.positionNedM.z += _state.velocityNedMps.z * dt;

    // A simple deterministic ground-contact plane at NED z = 0.
    if (_state.positionNedM.z > 0.0) {
        _state.positionNedM.z = 0.0;
        if (_state.velocityNedMps.z > 0.0) {
            _state.velocityNedMps.z = 0.0;
        }
    }

    _state.eulerRad = toEuler(_state.attitudeNedFromBody);
    _state.simulationTimeS += dt;
    _state.onGround = _state.positionNedM.z >= -0.02
        && std::abs(_state.velocityNedMps.z) < 0.08;
}

void SimpleQuadSimulator::reset() noexcept
{
    _state = VehicleState{};
    _targetMotors = MotorCommands{};
}

Vec3 SimpleQuadSimulator::rotateBodyVectorToNed(const Quaternion& q, const Vec3& v) noexcept
{
    const Vec3 qv{q.x, q.y, q.z};
    const Vec3 t = cross(qv, v);
    const Vec3 u = cross(qv, t);
    return {
        v.x + 2.0 * (q.w * t.x + u.x),
        v.y + 2.0 * (q.w * t.y + u.y),
        v.z + 2.0 * (q.w * t.z + u.z)
    };
}

EulerAngles SimpleQuadSimulator::toEuler(const Quaternion& q) noexcept
{
    const double sinRollCosPitch = 2.0 * (q.w * q.x + q.y * q.z);
    const double cosRollCosPitch = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    const double sinPitch = std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0);
    const double sinYawCosPitch = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosYawCosPitch = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

    return {
        std::atan2(sinRollCosPitch, cosRollCosPitch),
        std::asin(sinPitch),
        std::atan2(sinYawCosPitch, cosYawCosPitch)
    };
}

void SimpleQuadSimulator::normalize(Quaternion& q) noexcept
{
    const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm <= 0.0 || !std::isfinite(norm)) {
        q = Quaternion{};
        return;
    }
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
}

} // namespace px4lite::simulator
