/**
 * @file main.cpp
 * @brief 補充模組 04: 控制分配器與優先級消飽和策略 (Control Allocator & Desaturation)
 * 
 * [對標 PX4 源碼]:
 * - src/modules/control_allocator
 * - src/modules/control_allocator/ControlAllocation/ControlAllocation.cpp
 */

#include <iostream>
#include <algorithm>
#include <cmath>
#include <cassert>

struct MotorOutputs {
    float m[4];
};

// 1. 一般未消飽和的簡單線性混控器 (直覺做法)
class NaiveMixer {
public:
    static MotorOutputs mix(float rollCmd, float pitchCmd, float yawCmd, float thrust) {
        MotorOutputs out;
        out.m[0] = thrust - rollCmd - pitchCmd - yawCmd; // M1 (右前)
        out.m[1] = thrust + rollCmd + pitchCmd - yawCmd; // M2 (左後)
        out.m[2] = thrust + rollCmd - pitchCmd + yawCmd; // M3 (左前)
        out.m[3] = thrust - rollCmd + pitchCmd + yawCmd; // M4 (右後)

        // 單純硬性截斷 [0, 1]
        for (int i = 0; i < 4; ++i) {
            out.m[i] = std::max(0.0f, std::min(1.0f, out.m[i]));
        }
        return out;
    }
};

// 2. PX4 工業級優先級消飽和分配器 (PX4 Priority Desaturation)
class PX4ControlAllocator {
public:
    static MotorOutputs allocate(float rollCmd, float pitchCmd, float yawCmd, float thrust) {
        MotorOutputs out;
        // 幾何分配矩陣 (Quad-X)
        out.m[0] = thrust - rollCmd - pitchCmd - yawCmd;
        out.m[1] = thrust + rollCmd + pitchCmd - yawCmd;
        out.m[2] = thrust + rollCmd - pitchCmd + yawCmd;
        out.m[3] = thrust - rollCmd + pitchCmd + yawCmd;

        // 1. 尋找極值
        float m_max = out.m[0];
        float m_min = out.m[0];
        for (int i = 1; i < 4; ++i) {
            if (out.m[i] > m_max) m_max = out.m[i];
            if (out.m[i] < m_min) m_min = out.m[i];
        }

        // 2. 優先級 1: 滾轉/俯仰優先 (Roll/Pitch Priority)
        // 若達到上限天花板 (m_max > 1.0)，優先犧牲部分垂直推力，將整體油門向下拉回
        if (m_max > 1.0f) {
            float deltaThrust = m_max - 1.0f;
            for (int i = 0; i < 4; ++i) {
                out.m[i] -= deltaThrust;
            }
        }
        // 若觸及下限地板 (m_min < 0.0)，集體上拉油門以保證糾偏力矩
        else if (m_min < 0.0f) {
            float deltaThrust = -m_min;
            for (int i = 0; i < 4; ++i) {
                out.m[i] += deltaThrust;
            }
        }

        // 3. 再次截斷保底
        for (int i = 0; i < 4; ++i) {
            out.m[i] = std::max(0.0f, std::min(1.0f, out.m[i]));
        }
        return out;
    }
};

int main() {
    std::cout << "=======================================================================\n";
    std::cout << " [補充模組 04]: 控制分配器與優先級消飽和策略 (Desaturation Benchmark)  \n";
    std::cout << "=======================================================================\n";

    // 工況設定:
    // 飛機正在 90% 大油門急速爬升 (thrust = 0.9)
    // 遭遇劇烈側風，姿態環要求 30% 滾轉力矩 (rollCmd = +0.3)
    float thrust = 0.90f;
    float rollCmd = 0.30f;
    float pitchCmd = 0.0f;
    float yawCmd = 0.0f;

    std::cout << "極限工況輸入: 爬升油門 = 90 %, 期望滾轉修正力矩 = 30 %\n\n";

    // 1. 測試傳統天真混控器 (Naive Mixer)
    auto naiveOut = NaiveMixer::mix(rollCmd, pitchCmd, yawCmd, thrust);
    float naiveDiff = naiveOut.m[1] - naiveOut.m[0]; // 左馬達 M2 - 右馬達 M1
    std::cout << "[傳統硬截斷混控器結果]:\n";
    std::cout << " - M2 (左後): " << naiveOut.m[1] << " (已被 1.0 截斷)\n";
    std::cout << " - M1 (右前): " << naiveOut.m[0] << "\n";
    std::cout << " - 實質滾轉推力差: " << naiveDiff << " (期望應為 0.60)\n";
    std::cout << " -> 損失了 " << (1.0f - naiveDiff / 0.60f) * 100.0f << " % 的姿態控制扭矩！(可能導致翻機！)\n\n";
    assert(naiveDiff < 0.50f);

    // 2. 測試 PX4 優先級消飽和分配器 (PX4 Desaturation Allocator)
    auto px4Out = PX4ControlAllocator::allocate(rollCmd, pitchCmd, yawCmd, thrust);
    float px4Diff = px4Out.m[1] - px4Out.m[0];
    std::cout << "[PX4 優先級消飽和分配器結果]:\n";
    std::cout << " - M2 (左後): " << px4Out.m[1] << "\n";
    std::cout << " - M1 (右前): " << px4Out.m[0] << "\n";
    std::cout << " - 實質滾轉推力差: " << px4Diff << " (完全達到期望 0.60！)\n";
    std::cout << " -> 成功動態下調集體推力，100% 完整保留滾轉姿態自穩能力！\n";
    assert(std::abs(px4Diff - 0.60f) < 1e-5f);

    std::cout << "\n✓ 驗收通過: 成功重現 PX4 核心消飽和防翻機物理分配架構！\n";
    return 0;
}
