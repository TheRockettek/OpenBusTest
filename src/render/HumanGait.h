#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace openbus::rendering {

struct HumanGaitState {
    double phaseRadians = 0.0;
    double weight = 0.0;
};

struct HumanPathPose {
    std::array<double, 3> position = {};
    double headingRadians = 0.0;
};

inline HumanPathPose sampleHumanPath(const std::vector<std::array<double, 3>>& path,
                                     double progress, double startHeadingRadians,
                                     double endHeadingRadians) {
    if (path.empty() || !std::isfinite(progress) || !std::isfinite(startHeadingRadians) ||
        !std::isfinite(endHeadingRadians)) {
        throw std::invalid_argument("Human path pose inputs are invalid");
    }
    for (const auto& point : path) {
        if (!std::all_of(point.begin(), point.end(),
                         [](double coordinate) { return std::isfinite(coordinate); })) {
            throw std::invalid_argument("Human path positions must be finite");
        }
    }

    HumanPathPose pose;
    pose.position = path.front();
    const double normalizedProgress = std::clamp(progress, 0.0, 1.0);
    double totalLength = 0.0;
    for (std::size_t index = 1; index < path.size(); ++index) {
        totalLength +=
            std::hypot(path[index][0] - path[index - 1][0], path[index][1] - path[index - 1][1],
                       path[index][2] - path[index - 1][2]);
    }
    if (totalLength <= 1.0e-9) {
        pose.headingRadians = normalizedProgress >= 1.0 ? endHeadingRadians : startHeadingRadians;
        return pose;
    }

    double remaining = totalLength * normalizedProgress;
    for (std::size_t index = 1; index < path.size(); ++index) {
        const auto& from = path[index - 1];
        const auto& to = path[index];
        const double dx = to[0] - from[0];
        const double dy = to[1] - from[1];
        const double dz = to[2] - from[2];
        const double segmentLength = std::hypot(dx, dy, dz);
        if (segmentLength <= 1.0e-9) {
            continue;
        }
        if (remaining <= segmentLength || index + 1 == path.size()) {
            const double segmentProgress = std::clamp(remaining / segmentLength, 0.0, 1.0);
            for (std::size_t axis = 0; axis < pose.position.size(); ++axis) {
                pose.position[axis] = from[axis] + (to[axis] - from[axis]) * segmentProgress;
            }
            const double horizontalLength = std::hypot(dx, dy);
            pose.headingRadians = normalizedProgress >= 1.0   ? endHeadingRadians
                                  : horizontalLength > 1.0e-9 ? std::atan2(dx, dy)
                                                              : startHeadingRadians;
            return pose;
        }
        remaining -= segmentLength;
    }
    pose.position = path.back();
    pose.headingRadians = endHeadingRadians;
    return pose;
}

inline void advanceHumanGait(HumanGaitState& gait, double distanceMeters, double elapsedSeconds,
                             double strideLengthMeters = 1.4) {
    if (!std::isfinite(distanceMeters) || distanceMeters < 0.0 || !std::isfinite(elapsedSeconds) ||
        elapsedSeconds < 0.0 || !std::isfinite(strideLengthMeters) || strideLengthMeters <= 0.0) {
        throw std::invalid_argument(
            "Human gait requires finite, nonnegative movement and a positive stride length");
    }
    if (elapsedSeconds == 0.0) {
        gait.weight = 0.0;
        return;
    }

    constexpr double fullCycleRadians = 2.0 * std::numbers::pi;
    constexpr double fullWalkingSpeedMetersPerSecond = 1.2;
    gait.phaseRadians =
        std::fmod(gait.phaseRadians + distanceMeters * fullCycleRadians / strideLengthMeters,
                  fullCycleRadians);
    gait.weight =
        std::clamp(distanceMeters / (elapsedSeconds * fullWalkingSpeedMetersPerSecond), 0.0, 1.0);
}

} // namespace openbus::rendering
