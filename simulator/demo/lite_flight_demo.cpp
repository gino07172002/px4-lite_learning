#include "px4lite/simple_quad_simulator.hpp"

#include "control_stack.hpp"
#include "position_control.hpp"
#include "control_allocator.hpp"
#include "commander.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <future>
#include <sstream>
#include <string>
#include <thread>

using namespace std::chrono_literals;
using px4lite::lab06::AttitudeController;
using px4lite::lab06::RateController;
using px4lite::simulator::MotorCommands;
using px4lite::simulator::SimpleQuadSimulator;
using px4lite::simulator::VehicleState;
using px4lite::supplementary::MulticopterPositionControl;
using px4lite::supplementary::PX4ControlAllocator;
using px4lite::supplementary::Vector3f;

namespace {

constexpr double kRadiansToDegrees = 57.29577951308232;
constexpr double kDegreesToRadians = 1.0 / kRadiansToDegrees;
constexpr int kTelemetryEverySteps = 10;

struct MissionTarget {
    Vector3f positionUp;
    const char* label;
};

MissionTarget targetForTime(double timeS)
{
    if (timeS < 5.0)  return {{0.0f, 0.0f, 2.5f}, "TAKEOFF"};
    if (timeS < 10.0) return {{4.0f, 0.0f, 2.5f}, "NORTH"};
    if (timeS < 15.0) return {{4.0f, 4.0f, 2.5f}, "NORTH_EAST"};
    if (timeS < 20.0) return {{0.0f, 4.0f, 2.5f}, "EAST"};
    if (timeS < 25.0) return {{0.0f, 0.0f, 2.5f}, "HOME"};
    if (timeS < 32.0) return {{0.0f, 0.0f, 0.0f}, "LAND"};
    return {{0.0f, 0.0f, 0.0f}, "COMPLETE"};
}

bool isArmed(State state)
{
    return state == State::ARMED || state == State::TAKEOFF
        || state == State::FLYING || state == State::LANDING
        || state == State::FAILSAFE;
}

void printSnapshot(const VehicleState& state,
                   State commanderState,
                   const MissionTarget& target)
{
    std::cout << std::setprecision(7)
        << "{\"vehicle\":\"px4lite\",\"source\":\"PX4-Lite / SimpleQuadSimulator\""
        << ",\"sim_time_s\":" << state.simulationTimeS
        << ",\"position_ned_m\":{\"x\":" << state.positionNedM.x
        << ",\"y\":" << state.positionNedM.y
        << ",\"z\":" << state.positionNedM.z << "}"
        << ",\"velocity_ned_mps\":{\"x\":" << state.velocityNedMps.x
        << ",\"y\":" << state.velocityNedMps.y
        << ",\"z\":" << state.velocityNedMps.z << "}"
        << ",\"attitude_rad\":{\"roll\":" << state.eulerRad.rollRad
        << ",\"pitch\":" << state.eulerRad.pitchRad
        << ",\"yaw\":" << state.eulerRad.yawRad << "}"
        << ",\"motors\":["
        << state.motorOutput[0] << ',' << state.motorOutput[1] << ','
        << state.motorOutput[2] << ',' << state.motorOutput[3] << ']'
        << ",\"mode\":\"" << stateToString(commanderState) << "\""
        << ",\"target\":\"" << target.label << "\""
        << ",\"armed\":" << (isArmed(commanderState) ? "true" : "false")
        << ",\"on_ground\":" << (state.onGround ? "true" : "false")
        << ",\"ready_for_takeoff\":" << (std::string(target.label) == "READY" ? "true" : "false")
        << "}\n";
    std::cout.flush();
}

} // namespace

int main(int argc, char** argv)
{
    bool realtime = true;
    bool waitForStart = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--fast") {
            realtime = false;
        }
        if (std::string(argv[i]) == "--wait-for-start") waitForStart = true;
    }

    SimpleQuadSimulator::Config config;
    config.fixedDtSeconds = 0.004;
    SimpleQuadSimulator simulator(config);
    MulticopterPositionControl positionControl;
    AttitudeController rollController(4.0f);
    AttitudeController pitchController(4.0f);
    RateController rollRateController(0.12f, 0.015f, 0.002f);
    RateController pitchRateController(0.12f, 0.015f, 0.002f);
    Commander commander;
    const HealthStatus health{true, true, true};

    if (!commander.init()) return 2;
    if (waitForStart) {
        // Waiting never steps the plant or arms the existing Lab Commander.
        std::promise<double> command;
        auto received = command.get_future();
        std::thread reader([&command] {
            std::string line, verb;
            double epoch = 0.0;
            if (std::getline(std::cin, line)) {
                std::istringstream input(line);
                if (!(input >> verb >> epoch) || verb != "START" || !std::isfinite(epoch)) epoch = 0.0;
            }
            command.set_value(epoch);
        });
        do {
            printSnapshot(simulator.state(), commander.state(), {{0, 0, 0}, "READY"});
        } while (received.wait_for(200ms) != std::future_status::ready);
        const double epoch = received.get();
        reader.join();
        if (epoch <= 0.0) return 0; // EOF/invalid command must never cause takeoff.
        const auto deadline = std::chrono::system_clock::time_point(
            std::chrono::duration_cast<std::chrono::system_clock::duration>(std::chrono::duration<double>(epoch)));
        std::this_thread::sleep_until(deadline);
    }
    if (!commander.arm(health) || !commander.takeoff(health)) {
        std::cerr << "PX4-Lite Commander refused demo startup\n";
        return 2;
    }

    constexpr double dt = 0.004;
    constexpr double flightDurationS = 32.0;
    constexpr float angleLimitRad = 0.55f;
    MotorCommands motors{};
    auto nextWallTick = std::chrono::steady_clock::now();
    const auto wallTick = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(dt));
    bool landRequested = false;

    const int totalSteps = static_cast<int>(flightDurationS / dt);
    for (int step = 0; step <= totalSteps; ++step) {
        const double timeS = simulator.state().simulationTimeS;
        const MissionTarget target = targetForTime(timeS);
        const auto& state = simulator.state();

        const Vector3f positionSp = target.positionUp;
        const Vector3f positionActual{
            static_cast<float>(state.positionNedM.x),
            static_cast<float>(state.positionNedM.y),
            static_cast<float>(-state.positionNedM.z)
        };
        const Vector3f velocityActual{
            static_cast<float>(state.velocityNedMps.x),
            static_cast<float>(state.velocityNedMps.y),
            static_cast<float>(-state.velocityNedMps.z)
        };
        const auto positionSetpoint =
            positionControl.update(positionSp, positionActual, velocityActual,
                                   static_cast<float>(dt));

        const float rollSetpoint = std::clamp(
            positionSetpoint.roll_deg * static_cast<float>(kDegreesToRadians),
            -angleLimitRad, angleLimitRad);
        const float pitchSetpoint = std::clamp(
            positionSetpoint.pitch_deg * static_cast<float>(kDegreesToRadians),
            -angleLimitRad, angleLimitRad);
        const float rollRateSetpoint =
            rollController.update(rollSetpoint, static_cast<float>(state.eulerRad.rollRad));
        const float pitchRateSetpoint =
            pitchController.update(pitchSetpoint, static_cast<float>(state.eulerRad.pitchRad));
        const float rollTorque = rollRateController.update(
            rollRateSetpoint, static_cast<float>(state.angularVelocityBodyRadS.x),
            static_cast<float>(dt));
        const float pitchTorque = pitchRateController.update(
            pitchRateSetpoint, static_cast<float>(state.angularVelocityBodyRadS.y),
            static_cast<float>(dt));

        // The Lab04 body FRD plant uses positive pitch for nose-up, while the
        // Lab supplementary mixer encodes positive pitch with rear-motor thrust.
        // Negate only at this adapter boundary; the original controller/mixer stay intact.
        const auto output = PX4ControlAllocator::allocate(
            rollTorque, -pitchTorque, 0.0f, positionSetpoint.thrust_norm);
        motors.normalized = {{
            output.m[0], output.m[1], output.m[2], output.m[3]
        }};
        simulator.setMotorCommands(motors);
        simulator.step();

        const bool airborne = !simulator.state().onGround;
        commander.update(VehicleStatus{airborne}, health);
        if (timeS >= 25.0 && !landRequested && commander.state() == State::FLYING) {
            landRequested = commander.land();
        }

        if (step % kTelemetryEverySteps == 0) {
            printSnapshot(simulator.state(), commander.state(), target);
        }

        if (realtime) {
            nextWallTick += wallTick;
            std::this_thread::sleep_until(nextWallTick);
        }
    }

    const auto& finalState = simulator.state();
    std::cerr << "PX4-Lite demo complete: time=" << finalState.simulationTimeS
              << "s, altitude=" << -finalState.positionNedM.z
              << "m, commander=" << stateToString(commander.state()) << '\n';
    return 0;
}
