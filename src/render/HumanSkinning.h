#pragma once

#include "assets/ObjLoader.h"
#include "config/ModelConfigTypes.h"
#include "render/CoreRenderer.h"

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace openbus::rendering {

using HumanVertexWeights = std::array<float, HUMAN_WEIGHT_COUNT>;
using HumanBonePalette = std::array<Matrix4, HUMAN_BONE_COUNT>;

constexpr std::optional<HumanBoneSlot> humanBoneSlotFromOmsiIndex(int index) noexcept {
    constexpr int firstIndex = -2;
    constexpr int lastIndex = firstIndex - static_cast<int>(HUMAN_BONE_COUNT) + 1;
    if (index < lastIndex || index > firstIndex) {
        return std::nullopt;
    }
    return static_cast<HumanBoneSlot>(firstIndex - index);
}

struct HumanSkinWeights {
    std::vector<HumanVertexWeights> vertices;
    std::size_t invalidRecords = 0;
    std::size_t unboundBones = 0;
    std::size_t unweightedVertices = 0;
    bool hasWeights = false;
};

inline bool hasCompleteHumanSkinWeights(const HumanSkinWeights& skin) noexcept {
    return skin.hasWeights && skin.invalidRecords == 0 && skin.unboundBones == 0 &&
           skin.unweightedVertices == 0;
}

struct HumanJoints {
    std::array<double, 3> hip = {};
    std::array<double, 3> knee = {};
    std::array<double, 3> shoulder = {};
    std::array<double, 3> elbow = {};
    std::array<double, 3> hand = {};
    std::array<double, 3> waist = {};
    std::array<double, 3> neck = {};
};

HumanSkinWeights resolveHumanSkinWeights(const ParsedObj& mesh,
                                         const std::vector<ModelBoneBinding>& bindings);
HumanJoints approximateHumanJoints(const HumanGaitParameters& gait);
HumanVertexWeights inferHumanSkinWeights(const std::array<double, 3>& position,
                                         const HumanJoints& joints, double height);
HumanBonePalette makeHumanBonePalette(const HumanJoints& joints, const HumanGaitParameters& gait,
                                      double seatedFraction = 0.0);
double humanSeatAnchorOffset(double seatHeight, const HumanJoints& joints, double seatedFraction);
double humanConfiguredSeatHeightOffset(double seatHeight, const HumanJoints& joints,
                                       double seatedFraction);

} // namespace openbus::rendering
