#include "px4lite/simple_quad_simulator.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using px4lite::simulator::MotorCommands;
using px4lite::simulator::SimpleQuadSimulator;

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool near(double a, double b, double tolerance = 1e-8)
{
    return std::abs(a - b) <= tolerance;
}

void testFixedStepAndReset()
{
    SimpleQuadSimulator sim;
    require(near(sim.fixedDtSeconds(), 0.004), "default step must be 4 ms");

    MotorCommands hover{{0.4905, 0.4905, 0.4905, 0.4905}};
    require(sim.setMotorCommands(hover), "valid motor commands must be accepted");
    for (int i = 0; i < 250; ++i) {
        sim.step();
    }
    require(near(sim.state().simulationTimeS, 1.0), "simulation time must advance by fixed dt");

    sim.reset();
    require(near(sim.state().simulationTimeS, 0.0), "reset must clear simulation time");
    require(near(sim.state().attitudeNedFromBody.w, 1.0), "reset must restore identity attitude");
    require(sim.state().onGround, "reset state must be grounded");
}

void testMotorValidationAndClamp()
{
    SimpleQuadSimulator sim;
    require(sim.setMotorCommands(MotorCommands{{-0.5, 0.25, 1.25, 0.5}}),
            "finite motor commands must be accepted and clamped");
    sim.step();
    require(near(sim.state().motorOutput[0], 0.0), "low motor command must clamp to zero");
    require(sim.state().motorOutput[1] > 0.0, "motor lag must respond to a valid command");

    const auto priorTarget = sim.state().motorOutput;
    MotorCommands invalid{{0.2, 0.2, std::nan(""), 0.2}};
    require(!sim.setMotorCommands(invalid), "NaN motor command must be rejected");
    require(sim.state().motorOutput == priorTarget, "rejecting a command must not mutate state");
}

void testLiftAndGroundContact()
{
    SimpleQuadSimulator sim;
    require(sim.setMotorCommands(MotorCommands{{0.8, 0.8, 0.8, 0.8}}),
            "lift command must be accepted");
    for (int i = 0; i < 500; ++i) {
        sim.step();
    }
    require(sim.state().positionNedM.z < -0.1, "thrust above weight must climb (negative NED z)");

    require(sim.setMotorCommands(MotorCommands{}), "motor cut must be accepted");
    for (int i = 0; i < 3000; ++i) {
        sim.step();
    }
    require(sim.state().positionNedM.z <= 0.0, "ground contact must prevent passing below ground");
    require(sim.state().onGround, "landed state must be reported after impact");
}

void testLab04TorqueDirections()
{
    SimpleQuadSimulator rollSim;
    rollSim.setMotorCommands(MotorCommands{{0.3, 0.7, 0.7, 0.3}});
    for (int i = 0; i < 80; ++i) {
        rollSim.step();
    }
    require(rollSim.state().eulerRad.rollRad > 0.0,
            "Lab04 left-heavy motor pattern must produce positive roll");

    SimpleQuadSimulator pitchSim;
    pitchSim.setMotorCommands(MotorCommands{{0.3, 0.7, 0.3, 0.7}});
    for (int i = 0; i < 80; ++i) {
        pitchSim.step();
    }
    require(pitchSim.state().eulerRad.pitchRad < 0.0,
            "rear-heavy thrust must produce nose-down (negative FRD pitch)");

    SimpleQuadSimulator yawSim;
    yawSim.setMotorCommands(MotorCommands{{0.3, 0.3, 0.7, 0.7}});
    for (int i = 0; i < 80; ++i) {
        yawSim.step();
    }
    require(yawSim.state().eulerRad.yawRad > 0.0,
            "CCW-heavy motor pattern must produce positive NED yaw");
}

void testDeterministicReplay()
{
    SimpleQuadSimulator a;
    SimpleQuadSimulator b;
    for (int i = 0; i < 3000; ++i) {
        const double pulse = (i / 250) % 2 == 0 ? 0.62 : 0.44;
        const MotorCommands motors{{pulse, 0.51, 0.48, 0.50}};
        a.setMotorCommands(motors);
        b.setMotorCommands(motors);
        a.step();
        b.step();
    }
    const auto& sa = a.state();
    const auto& sb = b.state();
    require(sa.positionNedM.x == sb.positionNedM.x
            && sa.positionNedM.y == sb.positionNedM.y
            && sa.positionNedM.z == sb.positionNedM.z
            && sa.velocityNedMps.x == sb.velocityNedMps.x
            && sa.velocityNedMps.y == sb.velocityNedMps.y
            && sa.velocityNedMps.z == sb.velocityNedMps.z
            && sa.attitudeNedFromBody.w == sb.attitudeNedFromBody.w
            && sa.attitudeNedFromBody.x == sb.attitudeNedFromBody.x
            && sa.attitudeNedFromBody.y == sb.attitudeNedFromBody.y
            && sa.attitudeNedFromBody.z == sb.attitudeNedFromBody.z
            && sa.simulationTimeS == sb.simulationTimeS,
            "identical fixed-dt inputs must produce bit-identical state");
}

void testInvalidConfig()
{
    SimpleQuadSimulator::Config config;
    config.fixedDtSeconds = 0.0;
    bool threw = false;
    try {
        SimpleQuadSimulator sim(config);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    require(threw, "invalid fixed dt must be rejected");
}

} // namespace

int main()
{
    try {
        testFixedStepAndReset();
        testMotorValidationAndClamp();
        testLiftAndGroundContact();
        testLab04TorqueDirections();
        testDeterministicReplay();
        testInvalidConfig();
        std::cout << "SimpleQuadSimulator: 6 tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "SimpleQuadSimulator test failure: " << error.what() << '\n';
        return 1;
    }
}
