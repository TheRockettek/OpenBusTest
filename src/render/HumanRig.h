#pragma once

#include "config/HumanConfigLoader.h"
#include "render/CoreRenderer.h"
#include "render/HumanSkinning.h"

#include <algorithm>
#include <cmath>
#include <optional>

namespace openbus::rendering {

inline std::optional<HumanJoints> makeHumanJoints(const openbus::config::HumanConfig& config) {
    for (const auto& record : config.numericRecords) {
        if (record.keyword != "links" || !record.valid || record.values.size() != 22 ||
            !std::all_of(record.values.begin(), record.values.end(),
                         [](double value) { return std::isfinite(value); })) {
            continue;
        }
        const auto& v = record.values;
        return HumanJoints{{v[0], v[1], v[2]},    {v[3], v[4], v[5]},    {v[8], v[9], v[10]},
                           {v[11], v[12], v[13]}, {v[16], v[17], v[18]}, {0.0, v[6], v[7]},
                           {0.0, v[14], v[15]}};
    }
    return std::nullopt;
}

inline HumanGaitParameters makeHumanGaitProfile(const openbus::config::HumanConfig& config,
                                                double heightMeters) {
    HumanGaitParameters gait;
    gait.heightMeters = heightMeters;
    gait.upperArmBetaDegrees = config.walkUpperArmBetaDegrees;
    gait.armSwing = config.walkArmSwing;
    gait.hipSway = config.walkHipSway;
    gait.waistAnimation = config.walkWaistAnimation;
    if (config.rightShoulderHeightFractions) {
        gait.shoulderXHeightFraction = (*config.rightShoulderHeightFractions)[0];
        gait.shoulderZHeightFraction = (*config.rightShoulderHeightFractions)[1];
    }
    if (config.rightHipHeightFractions) {
        gait.hipXHeightFraction = (*config.rightHipHeightFractions)[0];
        gait.hipZHeightFraction = (*config.rightHipHeightFractions)[1];
    }
    if (config.rightKneePositionFractions) {
        gait.kneeXHeightFraction = (*config.rightKneePositionFractions)[0];
        gait.kneeYHeightFraction = (*config.rightKneePositionFractions)[1];
        gait.kneeZHeightFraction = (*config.rightKneePositionFractions)[2];
    }
    if (config.rightElbowHeightFractions) {
        gait.elbowXHeightFraction = (*config.rightElbowHeightFractions)[0];
        gait.elbowZHeightFraction = (*config.rightElbowHeightFractions)[1];
    }
    if (config.rightHandHeightFractions) {
        gait.handXHeightFraction = (*config.rightHandHeightFractions)[0];
        gait.handZHeightFraction = (*config.rightHandHeightFractions)[1];
    }
    if (config.rightFingerHeightFractions) {
        gait.fingerXHeightFraction = (*config.rightFingerHeightFractions)[0];
        gait.fingerZHeightFraction = (*config.rightFingerHeightFractions)[1];
    }
    if (config.neckHeightFractions) {
        gait.neckYHeightFraction = (*config.neckHeightFractions)[0];
        gait.neckZHeightFraction = (*config.neckHeightFractions)[1];
    }
    if (config.waistHeightFractions) {
        gait.waistYHeightFraction = (*config.waistHeightFractions)[0];
        gait.waistZHeightFraction = (*config.waistHeightFractions)[1];
    }
    return gait;
}

} // namespace openbus::rendering
