#pragma once

#include "map/MapTrafficRuntime.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>

namespace openbus::app {

inline constexpr double passengerApproachSpeedMetersPerSecond = 1.4;
inline constexpr double playerPassengerStopArrivalRadiusMeters = 20.0;
inline constexpr double playerPassengerStopReleaseRadiusMeters = 25.0;

inline std::array<double, 3> passengerCabinModelPoint(const std::array<double, 3>& point) {
    return {point[1], -point[0], point[2]};
}

inline std::array<double, 3> passengerCabinStandingPoint(const std::array<double, 3>& seat,
                                                         double seatHeight) {
    return {seat[0], seat[1], seat[2] - seatHeight};
}

inline openbus::map::MapTrafficPose
passengerDoorQueuePose(const openbus::map::MapTrafficPose& busPose, double entryForward,
                       double vehicleWidth, bool leftHandTraffic, std::size_t queueSlot) {
    const double side = openbus::map::mapTrafficPassengerSideSign(leftHandTraffic);
    const double lateral = vehicleWidth * 0.5 + 0.7 + 0.65 * static_cast<double>(queueSlot % 3);
    const double forward = entryForward - 0.7 * static_cast<double>(queueSlot / 3);
    const auto offset = openbus::map::mapTrafficPassengerWorldOffset(
        leftHandTraffic, busPose.headingRadians, lateral, forward);
    auto result = busPose;
    result.position[0] += offset[0];
    result.position[1] += offset[1];
    result.headingRadians -= side * std::numbers::pi * 0.5;
    return result;
}

inline double passengerApproachDuration(const openbus::map::MapTrafficPose& start,
                                        const openbus::map::MapTrafficPose& end) {
    return std::hypot(end.position[0] - start.position[0], end.position[1] - start.position[1]) /
           passengerApproachSpeedMetersPerSecond;
}

inline openbus::map::MapTrafficPose passengerApproachPose(const openbus::map::MapTrafficPose& start,
                                                          const openbus::map::MapTrafficPose& end,
                                                          double elapsedSeconds) {
    const double duration = passengerApproachDuration(start, end);
    const double progress =
        duration > 1.0e-6 ? std::clamp(elapsedSeconds / duration, 0.0, 1.0) : 1.0;
    auto result = end;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        result.position[axis] =
            start.position[axis] + (end.position[axis] - start.position[axis]) * progress;
    }
    if (progress < 1.0 && duration > 1.0e-6) {
        result.headingRadians =
            std::atan2(end.position[0] - start.position[0], end.position[1] - start.position[1]);
    }
    return result;
}

} // namespace openbus::app
