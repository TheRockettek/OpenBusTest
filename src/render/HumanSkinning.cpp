#include "render/HumanSkinning.h"

#include "render/MatrixMath.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <numbers>
#include <stdexcept>
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

double smoothWeight(double lower, double upper, double value) {
    const double t = std::clamp((value - lower) / (upper - lower), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double distanceToSegment(const std::array<double, 3>& point,
                         const std::array<double, 3>& start,
                         const std::array<double, 3>& end) {
    std::array<double, 3> segment;
    std::array<double, 3> offset;
    double segmentLengthSquared = 0.0;
    double projection = 0.0;
    for (std::size_t axis = 0; axis < point.size(); ++axis) {
        segment[axis] = end[axis] - start[axis];
        offset[axis] = point[axis] - start[axis];
        segmentLengthSquared += segment[axis] * segment[axis];
        projection += offset[axis] * segment[axis];
    }
    const double t = segmentLengthSquared > 1.0e-12
                         ? std::clamp(projection / segmentLengthSquared, 0.0, 1.0)
                         : 0.0;
    double distanceSquared = 0.0;
    for (std::size_t axis = 0; axis < point.size(); ++axis) {
        const double difference = point[axis] - (start[axis] + segment[axis] * t);
        distanceSquared += difference * difference;
    }
    return std::sqrt(distanceSquared);
}

} // namespace

HumanSkinWeights resolveHumanSkinWeights(const ParsedObj& mesh,
                                         const std::vector<ModelBoneBinding>& bindings) {
    HumanSkinWeights result;
    result.vertices.resize(mesh.positions.size());
    std::unordered_map<std::string, int> boneIndices;
    std::unordered_map<std::string, std::unordered_map<std::uint32_t, float>> seenWeights;
    for (const ModelBoneBinding& binding : bindings) {
        const std::string name = boneKey(binding.name);
        const auto [it, inserted] = boneIndices.emplace(name, binding.index);
        if (!inserted && it->second != binding.index) {
            ++result.invalidRecords;
            it->second = 0;
        }
    }
    for (const ObjBone& bone : mesh.bones) {
        const std::string name = boneKey(bone.name);
        const auto binding = boneIndices.find(name);
        if (binding == boneIndices.end()) {
            ++result.unboundBones;
            continue;
        }
        const std::optional<HumanBoneSlot> slot = humanBoneSlotFromOmsiIndex(binding->second);
        if (!slot) {
            ++result.unboundBones;
            continue;
        }
        const std::size_t slotIndex = humanBoneSlotIndex(*slot);
        auto& seenForBone = seenWeights[name];
        seenForBone.reserve(seenForBone.size() + bone.weights.size());
        for (const ObjBoneWeight& weight : bone.weights) {
            if (weight.sourceIndex >= result.vertices.size() || !std::isfinite(weight.weight) ||
                weight.weight < 0.0F || weight.weight > 1.0F) {
                ++result.invalidRecords;
                continue;
            }
            const auto [previous, inserted] =
                seenForBone.emplace(weight.sourceIndex, weight.weight);
            if (!inserted) {
                if (previous->second != weight.weight) {
                    ++result.invalidRecords;
                }
                continue;
            }
            result.vertices[weight.sourceIndex][slotIndex] += weight.weight;
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

HumanJoints approximateHumanJoints(const HumanGaitParameters& gait) {
    const double height = gait.heightMeters;
    return {{gait.hipXHeightFraction * height, 0.0, gait.hipZHeightFraction * height},
            {gait.kneeXHeightFraction * height, gait.kneeYHeightFraction * height,
             gait.kneeZHeightFraction * height},
            {gait.shoulderXHeightFraction * height, 0.0, gait.shoulderZHeightFraction * height},
            {gait.elbowXHeightFraction * height, 0.0, gait.elbowZHeightFraction * height},
            {gait.handXHeightFraction * height, 0.0, gait.handZHeightFraction * height},
            {0.0, gait.waistYHeightFraction * height, gait.waistZHeightFraction * height},
            {0.0, gait.neckYHeightFraction * height, gait.neckZHeightFraction * height}};
}

HumanVertexWeights inferHumanSkinWeights(const std::array<double, 3>& position,
                                         const HumanJoints& joints, double height) {
    if (!std::isfinite(height) || height <= 0.0) {
        throw std::invalid_argument("Human skin inference requires a positive finite height");
    }
    HumanVertexWeights weights = {};
    const std::size_t side = position[0] < 0.0 ? 0 : 1;
    const HumanBoneSlot upperLeg =
        side == 0 ? HumanBoneSlot::LeftUpperLeg : HumanBoneSlot::RightUpperLeg;
    const HumanBoneSlot lowerLegSlot =
        side == 0 ? HumanBoneSlot::LeftLowerLeg : HumanBoneSlot::RightLowerLeg;
    const HumanBoneSlot upperArm =
        side == 0 ? HumanBoneSlot::LeftUpperArm : HumanBoneSlot::RightUpperArm;
    const HumanBoneSlot lowerArm =
        side == 0 ? HumanBoneSlot::LeftLowerArm : HumanBoneSlot::RightLowerArm;
    const double x = std::abs(position[0]);
    const double leg = 1.0 - smoothWeight(joints.hip[2] - 0.04 * height,
                                          joints.hip[2] + 0.04 * height, position[2]);
    const double lowerLeg = 1.0 - smoothWeight(joints.knee[2] - 0.03 * height,
                                               joints.knee[2] + 0.03 * height, position[2]);
    const double arm = (1.0 - leg) *
                       smoothWeight(std::abs(joints.shoulder[0]) + 0.02 * height,
                                    std::abs(joints.shoulder[0]) + 0.12 * height, x) *
                       smoothWeight(joints.hip[2], joints.hip[2] + 0.15 * height, position[2]);
    const double sideSign = side == 0 ? -1.0 : 1.0;
    const auto shoulder = sideJoint(joints.shoulder, sideSign);
    const auto elbow = sideJoint(joints.elbow, sideSign);
    const auto hand = sideJoint(joints.hand, sideSign);
    const double upperArmDistance = distanceToSegment(position, shoulder, elbow);
    const double lowerArmDistance = distanceToSegment(position, elbow, hand);
    const double upperArmAffinity =
        1.0 - smoothWeight(0.025 * height, 0.10 * height, upperArmDistance);
    const double lowerArmAffinity =
        1.0 - smoothWeight(0.025 * height, 0.10 * height, lowerArmDistance);
    const double armProximity = std::max(upperArmAffinity, lowerArmAffinity) * (1.0 - leg);
    const double armWeight = std::max(arm, armProximity);
    const double armAffinity = upperArmAffinity + lowerArmAffinity;
    const double upperArmFraction =
        armAffinity > 1.0e-6 ? upperArmAffinity / armAffinity : 1.0;
    weights[humanBoneSlotIndex(upperLeg)] = static_cast<float>(leg * (1.0 - lowerLeg));
    weights[humanBoneSlotIndex(lowerLegSlot)] = static_cast<float>(leg * lowerLeg);
    weights[humanBoneSlotIndex(upperArm)] = static_cast<float>(armWeight * upperArmFraction);
    weights[humanBoneSlotIndex(lowerArm)] =
        static_cast<float>(armWeight * (1.0 - upperArmFraction));
    const double torso = std::max(0.0, 1.0 - leg - armWeight);
    const double waistHeight = std::max(joints.hip[2], joints.waist[2]);
    double pelvisFraction = 1.0 - smoothWeight(waistHeight - 0.05 * height,
                                               waistHeight + 0.05 * height, position[2]);
    double neckFraction =
        smoothWeight(joints.neck[2] - 0.08 * height, joints.neck[2] + 0.02 * height,
                     position[2]);
    const double spineFraction = pelvisFraction + neckFraction;
    if (spineFraction > 1.0) {
        pelvisFraction /= spineFraction;
        neckFraction /= spineFraction;
    }
    weights[humanBoneSlotIndex(HumanBoneSlot::Hip)] = static_cast<float>(torso * pelvisFraction);
    weights[humanBoneSlotIndex(HumanBoneSlot::Main)] =
        static_cast<float>(torso * (1.0 - pelvisFraction - neckFraction));
    weights[humanBoneSlotIndex(HumanBoneSlot::Head)] = static_cast<float>(torso * neckFraction);
    return weights;
}

HumanBonePalette makeHumanBonePalette(const HumanJoints& joints, const HumanGaitParameters& gait,
                                      double seatedFraction) {
    HumanBonePalette palette;
    const double seated = std::clamp(seatedFraction, 0.0, 1.0);
    const double weight = std::clamp(gait.weight, 0.0, 1.0);
    const double sway = std::sin(gait.phaseRadians) * weight * (1.0 - seated);
    const Matrix4 hip =
        rotateAt({0.0, joints.hip[1], joints.hip[2]}, sway * 5.0 * gait.hipSway, 0, 1, 0);
    const Matrix4 chest =
        multiplyMatrix4(hip, rotateAt(joints.waist, sway * 2.0 * gait.waistAnimation, 1, 0, 0));
    palette[humanBoneSlotIndex(HumanBoneSlot::Hip)] = hip;
    palette[humanBoneSlotIndex(HumanBoneSlot::Main)] = chest;
    palette[humanBoneSlotIndex(HumanBoneSlot::Head)] =
        multiplyMatrix4(chest, rotateAt(joints.neck, -sway * 2.0, 1, 0, 0));
    for (std::size_t index = 0; index < 2; ++index) {
        const double side = index == 0 ? -1.0 : 1.0;
        const double phase = gait.phaseRadians + (index == 0 ? std::numbers::pi : 0.0);
        const HumanBoneSlot upperLegSlot =
            index == 0 ? HumanBoneSlot::LeftUpperLeg : HumanBoneSlot::RightUpperLeg;
        const HumanBoneSlot lowerLegSlot =
            index == 0 ? HumanBoneSlot::LeftLowerLeg : HumanBoneSlot::RightLowerLeg;
        const HumanBoneSlot upperArmSlot =
            index == 0 ? HumanBoneSlot::LeftUpperArm : HumanBoneSlot::RightUpperArm;
        const HumanBoneSlot lowerArmSlot =
            index == 0 ? HumanBoneSlot::LeftLowerArm : HumanBoneSlot::RightLowerArm;
        const HumanBoneSlot handSlot =
            index == 0 ? HumanBoneSlot::LeftHand : HumanBoneSlot::RightHand;
        const double swing = std::sin(phase) * weight * (1.0 - seated);
        const double kneeBend = std::max(0.0, std::cos(phase)) * weight * (1.0 - seated);
        const Matrix4 upperLeg = multiplyMatrix4(
            hip, rotateAt(sideJoint(joints.hip, side), swing * 24.0 + seated * 90.0, 1, 0, 0));
        palette[humanBoneSlotIndex(upperLegSlot)] = upperLeg;
        palette[humanBoneSlotIndex(lowerLegSlot)] =
            multiplyMatrix4(upperLeg, rotateAt(sideJoint(joints.knee, side),
                                               -kneeBend * 28.0 - seated * 90.0, 1, 0, 0));
        const Matrix4 shoulderSwing =
            rotateAt(sideJoint(joints.shoulder, side), -swing * 18.0 * gait.armSwing, 1, 0, 0);
        const Matrix4 shoulderLower =
            rotateAt(sideJoint(joints.shoulder, side), side * gait.upperArmBetaDegrees, 0, 1, 0);
        const Matrix4 upperArm =
            multiplyMatrix4(chest, multiplyMatrix4(shoulderSwing, shoulderLower));
        palette[humanBoneSlotIndex(upperArmSlot)] = upperArm;
        const Matrix4 lowerArm =
            multiplyMatrix4(upperArm, rotateAt(sideJoint(joints.elbow, side),
                                               swing * 8.0 * gait.armSwing, 1, 0, 0));
        palette[humanBoneSlotIndex(lowerArmSlot)] = lowerArm;
        palette[humanBoneSlotIndex(handSlot)] = multiplyMatrix4(
            lowerArm, rotateAt(sideJoint(joints.hand, side), swing * 4.0 * gait.armSwing, 1, 0, 0));
    }
    return palette;
}

double humanSeatAnchorOffset(double seatHeight, const HumanJoints& joints, double seatedFraction) {
    return (seatHeight - joints.hip[2]) * std::clamp(seatedFraction, 0.0, 1.0);
}

double humanConfiguredSeatHeightOffset(double seatHeight, const HumanJoints& joints,
                                       double seatedFraction) {
    if (!std::isfinite(seatHeight) || !std::isfinite(seatedFraction)) {
        throw std::invalid_argument("Human configured seat height must be finite");
    }
    return (joints.hip[2] - seatHeight) * std::clamp(seatedFraction, 0.0, 1.0);
}

} // namespace openbus::rendering
