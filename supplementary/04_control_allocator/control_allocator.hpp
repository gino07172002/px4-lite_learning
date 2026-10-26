#pragma once

#include <algorithm>

namespace px4lite::supplementary {

struct MotorOutputs {
    float m[4];
};

class NaiveMixer {
public:
    static MotorOutputs mix(float rollCmd, float pitchCmd, float yawCmd, float thrust) {
        MotorOutputs out;
        out.m[0] = thrust - rollCmd - pitchCmd - yawCmd;
        out.m[1] = thrust + rollCmd + pitchCmd - yawCmd;
        out.m[2] = thrust + rollCmd - pitchCmd + yawCmd;
        out.m[3] = thrust - rollCmd + pitchCmd + yawCmd;
        for (float& motor : out.m) {
            motor = std::max(0.0f, std::min(1.0f, motor));
        }
        return out;
    }
};

class PX4ControlAllocator {
public:
    static MotorOutputs allocate(float rollCmd, float pitchCmd, float yawCmd, float thrust) {
        MotorOutputs out;
        out.m[0] = thrust - rollCmd - pitchCmd - yawCmd;
        out.m[1] = thrust + rollCmd + pitchCmd - yawCmd;
        out.m[2] = thrust + rollCmd - pitchCmd + yawCmd;
        out.m[3] = thrust - rollCmd + pitchCmd + yawCmd;

        float maximum = out.m[0];
        float minimum = out.m[0];
        for (int i = 1; i < 4; ++i) {
            maximum = std::max(maximum, out.m[i]);
            minimum = std::min(minimum, out.m[i]);
        }
        if (maximum > 1.0f) {
            const float deltaThrust = maximum - 1.0f;
            for (float& motor : out.m) {
                motor -= deltaThrust;
            }
        } else if (minimum < 0.0f) {
            const float deltaThrust = -minimum;
            for (float& motor : out.m) {
                motor += deltaThrust;
            }
        }
        for (float& motor : out.m) {
            motor = std::max(0.0f, std::min(1.0f, motor));
        }
        return out;
    }
};

} // namespace px4lite::supplementary
