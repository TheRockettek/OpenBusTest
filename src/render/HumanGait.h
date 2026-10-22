#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace openbus::rendering {

struct HumanGaitState {
    double phaseRadians = 0.0;
    double weight = 0.0;
};

inline void advanceHumanGait(HumanGaitState& gait, double distanceMeters, double elapsedSeconds) {
    if (!std::isfinite(distanceMeters) || distanceMeters < 0.0 || !std::isfinite(elapsedSeconds) ||
        elapsedSeconds < 0.0) {
        throw std::invalid_argument("Human gait requires finite, nonnegative movement");
    }
    if (elapsedSeconds == 0.0) {
        gait.weight = 0.0;
        return;
    }

    constexpr double strideLengthMeters = 1.4;
    constexpr double fullCycleRadians = 2.0 * std::numbers::pi;
    constexpr double fullWalkingSpeedMetersPerSecond = 1.2;
    gait.phaseRadians =
        std::fmod(gait.phaseRadians + distanceMeters * fullCycleRadians / strideLengthMeters,
                  fullCycleRadians);
    gait.weight =
        std::clamp(distanceMeters / (elapsedSeconds * fullWalkingSpeedMetersPerSecond), 0.0, 1.0);
}

} // namespace openbus::rendering
