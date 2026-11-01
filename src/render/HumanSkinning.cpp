#include "render/HumanSkinning.h"

#include "render/MatrixMath.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <numbers>
#include <string>
#include <unordered_map>

namespace openbus::rendering {
namespace {

std::string boneKey(std::string name) {
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return name;
}

Matrix4 rotateAt(const std::array<double, 3>& pivot, double angle, double x, double y, double z) {
    const Matrix4 rotation = rotationMatrix(angle, x, y, z);
    return multiplyMatrix4(multiplyMatrix4(translationMatrix(pivot), rotation),
                           translationMatrix({-pivot[0], -pivot[1], -pivot[2]}));
}

std::array<double, 3> sideJoint(const std::array<double, 3>& joint, double side) {
    return {side * std::abs(joint[0]), joint[1], joint[2]};
}

} // namespace

HumanSkinWeights resolveHumanSkinWeights(const ParsedObj& mesh,
                                         const std::vector<ModelBoneBinding>& bindings) {
    HumanSkinWeights result;
    result.vertices.resize(mesh.positions.size());
    std::unordered_map<std::string, int> boneIndices;
    for (const ModelBoneBinding& binding : bindings) {
        const std::string name = boneKey(binding.name);
        const auto [it, inserted] = boneIndices.emplace(name, binding.index);
        if (!inserted && it->second != binding.index) {
            ++result.invalidRecords;
            it->second = 0;
        }
    }
    for (const ObjBone& bone : mesh.bones) {
        const auto binding = boneIndices.find(boneKey(bone.name));
        if (binding == boneIndices.end() || binding->second > -2 || binding->second < -14) {
            ++result.unboundBones;
            continue;
        }
        const std::size_t slot = static_cast<std::size_t>(-binding->second - 2);
        for (const ObjBoneWeight& weight : bone.weights) {
            if (weight.sourceIndex >= result.vertices.size() || !std::isfinite(weight.weight) ||
                weight.weight < 0.0F || weight.weight > 1.0F) {
                ++result.invalidRecords;
                continue;
            }
            result.vertices[weight.sourceIndex][slot] = weight.weight;
        }
    }
    for (HumanVertexWeights& weights : result.vertices) {
        float sum = 0.0F;
        for (std::size_t index = 0; index < HUMAN_BONE_COUNT; ++index) {
            sum += weights[index];
        }
        if (sum <= 0.0F) {
            ++result.unweightedVertices;
            continue;
        }
        result.hasWeights = true;
        for (std::size_t index = 0; index < HUMAN_BONE_COUNT; ++index) {
            weights[index] /= sum;
        }
    }
    return result;
}

HumanBonePalette makeHumanBonePalette(const HumanJoints& joints, const HumanGaitParameters& gait) {
    HumanBonePalette palette;
    const double weight = std::clamp(gait.weight, 0.0, 1.0);
    const double sway = std::sin(gait.phaseRadians) * weight;
    const Matrix4 hip =
        rotateAt({0.0, joints.hip[1], joints.hip[2]}, sway * 5.0 * gait.hipSway, 0, 1, 0);
    const Matrix4 chest =
        multiplyMatrix4(hip, rotateAt(joints.waist, sway * 2.0 * gait.waistAnimation, 1, 0, 0));
    palette[8] = hip;
    palette[9] = chest;
    palette[10] = multiplyMatrix4(chest, rotateAt(joints.neck, -sway * 2.0, 1, 0, 0));
    for (std::size_t index = 0; index < 2; ++index) {
        const double side = index == 0 ? -1.0 : 1.0;
        const double phase = gait.phaseRadians + (index == 0 ? std::numbers::pi : 0.0);
        const double swing = std::sin(phase) * weight;
        const double kneeBend = std::max(0.0, std::cos(phase)) * weight;
        const Matrix4 upperLeg =
            multiplyMatrix4(hip, rotateAt(sideJoint(joints.hip, side), swing * 24.0, 1, 0, 0));
        palette[index] = upperLeg;
        palette[2 + index] = multiplyMatrix4(
            upperLeg, rotateAt(sideJoint(joints.knee, side), -kneeBend * 28.0, 1, 0, 0));
        const Matrix4 shoulderSwing =
            rotateAt(sideJoint(joints.shoulder, side), -swing * 18.0 * gait.armSwing, 1, 0, 0);
        const Matrix4 shoulderLower =
            rotateAt(sideJoint(joints.shoulder, side), side * gait.upperArmBetaDegrees, 0, 1, 0);
        const Matrix4 upperArm =
            multiplyMatrix4(chest, multiplyMatrix4(shoulderSwing, shoulderLower));
        palette[4 + index] = upperArm;
        const Matrix4 lowerArm =
            multiplyMatrix4(upperArm, rotateAt(sideJoint(joints.elbow, side),
                                               swing * 8.0 * gait.armSwing, 1, 0, 0));
        palette[6 + index] = lowerArm;
        palette[11 + index] = multiplyMatrix4(
            lowerArm, rotateAt(sideJoint(joints.hand, side), swing * 4.0 * gait.armSwing, 1, 0, 0));
    }
    return palette;
}

} // namespace openbus::rendering
