#include <iostream>
#include <cmath>

class QuadSimulator
{

    public:
    QuadSimulator()=default;
    ~QuadSimulator()=default;

    struct State
    {
        float roll=0.0f;
        float pitch=0.0f;
        float pitchRate = 0.0f;

        float rollRate = 0.0f;

        float z=0.0f;
        float vz=0.0f;
    };
    static float clampMotor(float v)
    {
        if(v>1.0f)
            return 1.0f;
        else if(v<0.0f)
            return 0.0f;
        return v;
    }
    void setMotors(float v1,float v2,float v3,float v4)
    {
        _motor[0]=clampMotor(v1);
        _motor[1]=clampMotor(v2);
        _motor[2]=clampMotor(v3);
        _motor[3]=clampMotor(v4);
    }
    void update(float dt)
    {
        if(dt<=0.0f)
            return ;

    float thrust1 = _motor[0] * _maxThrust;
    float thrust2 = _motor[1] * _maxThrust;
    float thrust3 = _motor[2] * _maxThrust;
    float thrust4 = _motor[3] * _maxThrust;

    float totalThrust =
        thrust1 +
        thrust2 +
        thrust3 +
        thrust4;

    float gravityForce =
    _mass * _gravity;

float verticalThrust =
    totalThrust
    * std::cos(_state.roll)
    * std::cos(_state.pitch);

float netForce =
    verticalThrust - gravityForce;

    float acceleration =
        netForce / _mass;

    _state.vz += acceleration * dt;

    _state.z += _state.vz * dt;

       float leftThrust =
    thrust1 + thrust4;

float rightThrust =
    thrust2 + thrust3;

float rollTorque =
    (leftThrust - rightThrust)
    * _armLength;

        float rollAcceleration =
    rollTorque / _Ixx;

_state.rollRate +=
    rollAcceleration * dt;

_state.roll +=
    _state.rollRate * dt;
        float frontThrust =
    thrust1 + thrust2;

float backThrust =
    thrust4 + thrust3;

float pitchTorque =
    (frontThrust - backThrust)
    * _armLength;

float pitchAcceleration =
    pitchTorque / _Iyy;

_state.pitchRate +=
    pitchAcceleration * dt;

_state.pitch +=
    _state.pitchRate * dt;

    }

    const State& state() const
    {
        return _state;
    }

    private:
    State _state;
    float _motor[4]{ 0.0,0.0,0.0,0.0   };
    float _maxThrust = 5.0f; // N
    float _mass = 1.0f;
    float _gravity = 9.81f;
    float _armLength = 0.2f;
    float _Ixx = 0.02f; // kg * m^2
    float _Iyy = 0.02f;
};

int main() {

     QuadSimulator sim;
    QuadSimulator sim2;
    QuadSimulator sim3;

    //up
   sim.setMotors(
        0.5,
        0.5,
        0.5,
        0.5
    );

    for (int i = 0; i < 1000; ++i)
    {
        sim.update(0.001);
    }

    const auto& state = sim.state();

    std::cout
        << "UP roll=" << state.roll
        << " pitch=" << state.pitch
        << " z=" << state.z
        << std::endl;

    //hover

    sim2.setMotors(
    0.4905f,
    0.4905f,
    0.4905f,
    0.4905f
);

    for (int i = 0; i < 1000; ++i)
    {
        sim2.update(0.001);
    }
     const auto& state2 = sim2.state();
    std::cout
        << "Hover roll=" << state2.roll
        << " pitch=" << state2.pitch
        << " z=" << state2.z
        << std::endl;
    //down

    sim3.setMotors(
    0.4f,
    0.4f,
    0.4f,
    0.4f
);

     for (int i = 0; i < 1000; ++i)
    {
        sim3.update(0.001);
    }

    const auto& state3 = sim3.state();
    std::cout
        << "Down roll=" << state3.roll
        << " pitch=" << state3.pitch
        << " z=" << state3.z
        << std::endl;
    std::cout << "bye "<<std::endl;


    QuadSimulator rollSim;

rollSim.setMotors(
    0.6f,   // M1 左前
    0.4f,   // M2 右前
    0.4f,   // M3 右後
    0.6f    // M4 左後
);

for (int i = 0; i < 1000; ++i)
{
    rollSim.update(0.001f);
}

const auto& rollState = rollSim.state();

std::cout
    << "roll=" << rollState.roll
    << " rollRate=" << rollState.rollRate
    << " z=" << rollState.z
    << std::endl;

    QuadSimulator pitchSim;

pitchSim.setMotors(
    0.6f,   // M1 前
    0.6f,   // M2 前
    0.4f,   // M3 後
    0.4f    // M4 後
);

for (int i = 0; i < 1000; ++i)
{
    pitchSim.update(0.001f);
}

const auto& pitchState = pitchSim.state();

std::cout
    << "pitch=" << pitchState.pitch
    << " pitchRate=" << pitchState.pitchRate
    << " roll=" << pitchState.roll
    << std::endl;

    QuadSimulator tiltSim;

tiltSim.setMotors(
    0.6f,
    0.4f,
    0.4f,
    0.6f
);

for (int i = 0; i < 1000; ++i)
{
    tiltSim.update(0.001f);
}

const auto& tiltState = tiltSim.state();

std::cout
    << "roll=" << tiltState.roll
    << " z=" << tiltState.z
    << std::endl;

    QuadSimulator a;
QuadSimulator b;

a.setMotors(0.6f, 0.4f, 0.4f, 0.6f);
b.setMotors(0.6f, 0.4f, 0.4f, 0.6f);

for (int i = 0; i < 1000; ++i)
{
    a.update(0.001f);
    b.update(0.001f);
}

    const auto& stateA = a.state();
const auto& stateB = b.state();

std::cout
    << "A roll=" << stateA.roll
    << " pitch=" << stateA.pitch
    << " z=" << stateA.z
    << std::endl;

std::cout
    << "B roll=" << stateB.roll
    << " pitch=" << stateB.pitch
    << " z=" << stateB.z
    << std::endl;

 if (stateA.roll == stateB.roll &&
    stateA.pitch == stateB.pitch &&
    stateA.rollRate == stateB.rollRate &&
    stateA.pitchRate == stateB.pitchRate &&
    stateA.z == stateB.z &&
    stateA.vz == stateB.vz)
{
    std::cout << "Deterministic test PASS\n";
}
else
{
    std::cout << "Deterministic test FAIL" << std::endl;
}

    return 0;


}
