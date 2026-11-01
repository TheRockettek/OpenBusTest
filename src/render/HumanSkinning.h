#pragma once

#include "assets/ObjLoader.h"
#include "config/ModelConfigTypes.h"
#include "render/CoreRenderer.h"

#include <array>
#include <cstddef>
#include <vector>

namespace openbus::rendering {

using HumanVertexWeights = std::array<float, HUMAN_WEIGHT_COUNT>;
using HumanBonePalette = std::array<Matrix4, HUMAN_BONE_COUNT>;

struct HumanSkinWeights {
    std::vector<HumanVertexWeights> vertices;
    std::size_t invalidRecords = 0;
    std::size_t unboundBones = 0;
    std::size_t unweightedVertices = 0;
    bool hasWeights = false;
};

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
HumanBonePalette makeHumanBonePalette(const HumanJoints& joints, const HumanGaitParameters& gait);

} // namespace openbus::rendering
