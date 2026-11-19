#pragma once

#include "render/CameraMath.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace openbus::rendering {

enum class HumanBoneSlot : std::uint8_t {
    LeftUpperLeg = 0,
    RightUpperLeg = 1,
    LeftLowerLeg = 2,
    RightLowerLeg = 3,
    LeftUpperArm = 4,
    RightUpperArm = 5,
    LeftLowerArm = 6,
    RightLowerArm = 7,
    Hip = 8,
    Main = 9,
    Head = 10,
    LeftHand = 11,
    RightHand = 12,
    Count = 13,
};
inline constexpr std::size_t HUMAN_BONE_COUNT = static_cast<std::size_t>(HumanBoneSlot::Count);
inline constexpr std::size_t HUMAN_WEIGHT_COUNT = 16;

constexpr std::size_t humanBoneSlotIndex(HumanBoneSlot slot) noexcept {
    return static_cast<std::size_t>(slot);
}

using HumanVertexWeights = std::array<float, HUMAN_WEIGHT_COUNT>;
using HumanBonePalette = std::array<Matrix4, HUMAN_BONE_COUNT>;

struct HumanGaitParameters {
    double phaseRadians = 0.0;
    double weight = 0.0;
    double heightMeters = 0.0;
    double upperArmBetaDegrees = 66.0;
    double armSwing = 1.0;
    double hipSway = 1.0;
    double waistAnimation = 0.0;
    double shoulderXHeightFraction = 0.105;
    double shoulderZHeightFraction = 0.76;
    double hipXHeightFraction = 0.06;
    double hipZHeightFraction = 0.5;
    double kneeXHeightFraction = 0.08;
    double kneeYHeightFraction = 0.0;
    double kneeZHeightFraction = 0.25;
    double elbowXHeightFraction = 0.25;
    double elbowZHeightFraction = 0.76;
    double handXHeightFraction = 0.40;
    double handZHeightFraction = 0.76;
    double fingerXHeightFraction = 0.5;
    double fingerZHeightFraction = 0.76;
    double neckYHeightFraction = 0.0;
    double neckZHeightFraction = 0.85;
    double waistYHeightFraction = 0.0;
    double waistZHeightFraction = 0.6;
};

} // namespace openbus::rendering
