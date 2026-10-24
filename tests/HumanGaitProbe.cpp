#include "render/HumanGait.h"

#include <array>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <vector>

int main() {
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
