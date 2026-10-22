#include "render/HumanGait.h"

#include <cassert>
#include <cmath>

int main() {
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
    return 0;
}
