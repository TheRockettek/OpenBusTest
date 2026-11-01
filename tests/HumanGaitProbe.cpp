#include "render/HumanGait.h"
#include "render/HumanRig.h"

#include <array>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <vector>

int main() {
    const auto requireProfile = [](bool condition) {
        if (!condition) {
            throw std::runtime_error("human gait profile did not preserve authored parameters");
        }
    };
    openbus::config::HumanConfig human;
    human.walkUpperArmBetaDegrees = 61.0;
    human.walkArmSwing = 0.4;
    human.walkHipSway = 0.7;
    human.walkWaistAnimation = 0.3;
    human.rightShoulderHeightFractions = {0.11, 0.77};
    human.rightHipHeightFractions = {0.06, 0.51};
    human.rightKneePositionFractions = {0.08, -0.02, 0.26};
    human.rightElbowHeightFractions = {0.24, 0.71};
    human.rightHandHeightFractions = {0.39, 0.69};
    human.rightFingerHeightFractions = {0.48, 0.67};
    human.neckHeightFractions = {-0.03, 0.86};
    human.waistHeightFractions = {-0.01, 0.61};
    const auto profile = openbus::rendering::makeHumanGaitProfile(human, 1.7);
    requireProfile(profile.heightMeters == 1.7 && profile.upperArmBetaDegrees == 61.0);
    requireProfile(profile.armSwing == 0.4 && profile.hipSway == 0.7 &&
                   profile.waistAnimation == 0.3);
    requireProfile(profile.shoulderXHeightFraction == 0.11 &&
                   profile.shoulderZHeightFraction == 0.77);
    requireProfile(profile.hipXHeightFraction == 0.06 && profile.hipZHeightFraction == 0.51);
    requireProfile(profile.kneeXHeightFraction == 0.08 && profile.kneeYHeightFraction == -0.02 &&
                   profile.kneeZHeightFraction == 0.26);
    requireProfile(profile.elbowXHeightFraction == 0.24 && profile.elbowZHeightFraction == 0.71);
    requireProfile(profile.handXHeightFraction == 0.39 && profile.handZHeightFraction == 0.69);
    requireProfile(profile.fingerXHeightFraction == 0.48 && profile.fingerZHeightFraction == 0.67);
    requireProfile(profile.neckYHeightFraction == -0.03 && profile.neckZHeightFraction == 0.86);
    requireProfile(profile.waistYHeightFraction == -0.01 && profile.waistZHeightFraction == 0.61);
    const auto fallback = openbus::rendering::makeHumanGaitProfile({}, 1.8);
    const openbus::rendering::HumanGaitParameters defaults;
    requireProfile(fallback.fingerXHeightFraction == defaults.fingerXHeightFraction &&
                   fallback.neckZHeightFraction == defaults.neckZHeightFraction &&
                   fallback.waistZHeightFraction == defaults.waistZHeightFraction);

    const std::vector<std::array<double, 3>> turningPath = {
        {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}};
    const auto beforeTurn =
        openbus::rendering::sampleHumanPath(turningPath, 0.25, -1.0, 2.0);
    assert(std::abs(beforeTurn.position[0] - 0.5) < 1.0e-9);
    assert(std::abs(beforeTurn.position[1]) < 1.0e-9);
    assert(std::abs(beforeTurn.headingRadians - std::numbers::pi / 2.0) < 1.0e-9);
    const auto afterTurn =
        openbus::rendering::sampleHumanPath(turningPath, 0.75, -1.0, 2.0);
    assert(std::abs(afterTurn.position[0] - 1.0) < 1.0e-9);
    assert(std::abs(afterTurn.position[1] - 0.5) < 1.0e-9);
    assert(std::abs(afterTurn.headingRadians) < 1.0e-9);
    const auto completedPath =
        openbus::rendering::sampleHumanPath(turningPath, 1.0, -1.0, 2.0);
    assert(completedPath.position == turningPath.back());
    assert(completedPath.headingRadians == 2.0);

    openbus::rendering::HumanGaitState gait;
    openbus::rendering::advanceHumanGait(gait, 0.0, 0.5);
    assert(gait.phaseRadians == 0.0);
    assert(gait.weight == 0.0);

    openbus::rendering::advanceHumanGait(gait, 0.35, 0.5);
    assert(std::abs(gait.phaseRadians - std::numbers::pi / 2.0) < 1.0e-9);
    assert(std::abs(gait.weight - (0.35 / 0.6)) < 1.0e-9);

    openbus::rendering::advanceHumanGait(gait, 0.35, 0.5);
    assert(std::abs(gait.phaseRadians - std::numbers::pi) < 1.0e-9);
    assert(std::abs(gait.weight - (0.35 / 0.6)) < 1.0e-9);

    openbus::rendering::advanceHumanGait(gait, 0.0, 0.5);
    assert(std::abs(gait.phaseRadians - std::numbers::pi) < 1.0e-9);
    assert(gait.weight == 0.0);

    openbus::rendering::HumanGaitState longStrideGait;
    openbus::rendering::advanceHumanGait(longStrideGait, 0.35, 0.5, 2.8);
    assert(std::abs(longStrideGait.phaseRadians - std::numbers::pi / 4.0) < 1.0e-9);
    bool rejectedInvalidStride = false;
    try {
        openbus::rendering::advanceHumanGait(longStrideGait, 0.1, 0.1, 0.0);
    } catch (const std::invalid_argument&) {
        rejectedInvalidStride = true;
    }
    assert(rejectedInvalidStride);
    return 0;
}
