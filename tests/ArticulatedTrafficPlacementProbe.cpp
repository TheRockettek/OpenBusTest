#include "app/traffic/ArticulatedTrafficPlacement.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

void near(double actual, double expected) {
    if (std::abs(actual - expected) > 1.0e-8) {
        throw std::runtime_error("Articulated traffic placement mismatch: actual=" +
                                 std::to_string(actual) +
                                 ", expected=" + std::to_string(expected));
    }
}

void verifyAnchorAlignment(const VehiclePlacement& front,
                           const std::array<double, 3>& frontAnchor,
                           const VehiclePlacement& rear,
                           const std::array<double, 3>& rearAnchor) {
    const auto rotate = [](const std::array<double, 3>& point, double yawDegrees) {
        const double yaw = yawDegrees * std::acos(-1.0) / 180.0;
        return std::array<double, 3>{std::cos(yaw) * point[0] - std::sin(yaw) * point[1],
                                     std::sin(yaw) * point[0] + std::cos(yaw) * point[1],
                                     point[2]};
    };
    const auto frontOffset = rotate(frontAnchor, front.yawDegrees);
    const auto rearOffset = rotate(rearAnchor, rear.yawDegrees);
    for (std::size_t axis = 0; axis < 3; ++axis) {
        near(front.position[axis] + frontOffset[axis],
             rear.position[axis] + rearOffset[axis]);
    }
}

} // namespace

int main() {
    try {
        openbus::map::MapTrafficRoute straightRoute;
        openbus::map::MapTrafficRouteSegment straightSegment;
        straightSegment.points = {{0.0, 0.0, 0.0}, {0.0, 20.0, 0.0}};
        straightRoute.segments.push_back(straightSegment);
        straightRoute.cumulativeEndDistances = {20.0};
        straightRoute.totalLength = 20.0;

        openbus::map::MapTrafficRoute cornerRoute;
        openbus::map::MapTrafficRouteSegment cornerSegment;
        cornerSegment.points = {{0.0, 0.0, 0.0}, {0.0, 10.0, 0.0}, {10.0, 10.0, 0.0}};
        cornerRoute.segments.push_back(cornerSegment);
        cornerRoute.cumulativeEndDistances = {20.0};
        cornerRoute.totalLength = 20.0;

        const std::array<double, 3> frontAnchor = {2.0, 0.0, 0.0};
        const std::array<double, 3> rearAnchor = {1.0, 0.0, 0.0};
        VehiclePlacement front{{0.0, 5.0, 0.0}, 90.0, 0.0, 0.0};
        const auto straightRear = openbus::app::articulatedTrafficRearPlacement(
            straightRoute, 5.0, 0.0, front, frontAnchor, rearAnchor);
        near(straightRear.yawDegrees, 90.0);
        near(straightRear.position[0], 0.0);
        near(straightRear.position[1], 6.0);
        verifyAnchorAlignment(front, frontAnchor, straightRear, rearAnchor);

        front = {{0.0, 9.0, 0.0}, 90.0, 0.0, 0.0};
        const auto cornerRear = openbus::app::articulatedTrafficRearPlacement(
            cornerRoute, 9.0, 0.0, front, frontAnchor, rearAnchor);
        const auto cornerPose = openbus::map::sampleMapTrafficRoute(cornerRoute, 11.0, 0.0);
        if (!cornerPose) {
            throw std::runtime_error("Could not sample the articulated corner route");
        }
        near(cornerRear.yawDegrees,
             openbus::map::mapTrafficHeadingToVehicleYawDegrees(cornerPose->headingRadians));
        if (cornerRear.yawDegrees == front.yawDegrees) {
            throw std::runtime_error("Articulated rear section did not follow the route corner");
        }
        verifyAnchorAlignment(front, frontAnchor, cornerRear, rearAnchor);

        bool rejected = false;
        try {
            auto invalidFront = front;
            invalidFront.yawDegrees = std::numeric_limits<double>::quiet_NaN();
            (void)openbus::app::articulatedTrafficRearPlacement(
                cornerRoute, 9.0, 0.0, invalidFront, frontAnchor, rearAnchor);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        if (!rejected) {
            throw std::runtime_error("Non-finite articulated traffic placement was accepted");
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
