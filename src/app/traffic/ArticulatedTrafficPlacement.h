#pragma once

#include "map/MapTrafficRuntime.h"

#include <array>
#include <numbers>

namespace openbus::app {

inline VehiclePlacement
articulatedTrafficRearPlacement(const openbus::map::MapTrafficRoute& route,
                                double frontRouteDistanceMeters, double lateralOffsetMeters,
                                const VehiclePlacement& frontPlacement,
                                const std::array<double, 3>& frontRearCouplingPoint,
                                const std::array<double, 3>& rearFrontCouplingPoint) {
    const openbus::map::MapTrafficPose frontPose{
        frontPlacement.position, (90.0 - frontPlacement.yawDegrees) * std::numbers::pi / 180.0, 0,
        frontPlacement.pitchDegrees * std::numbers::pi / 180.0,
        frontPlacement.rollDegrees * std::numbers::pi / 180.0};
    const openbus::map::MapTrafficPose rearPose = openbus::map::mapTrafficArticulatedRearPose(
        route, frontRouteDistanceMeters, lateralOffsetMeters, frontPose, frontRearCouplingPoint,
        rearFrontCouplingPoint);
    return {rearPose.position,
            openbus::map::mapTrafficHeadingToVehicleYawDegrees(rearPose.headingRadians),
            rearPose.pitchRadians * 180.0 / std::numbers::pi,
            rearPose.rollRadians * 180.0 / std::numbers::pi};
}

} // namespace openbus::app
