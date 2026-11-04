#pragma once

#include "app/traffic/PassengerApproach.h"
#include "config/VehicleConfigLoader.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

namespace openbus::app {

inline openbus::map::MapTrafficPose
passengerCabinSeatPose(const openbus::map::MapTrafficPose& vehiclePose,
                       const PassengerCabinPosition& seat) {
    constexpr double degreesToRadians = std::numbers::pi / 180.0;
    const double yaw =
        openbus::map::mapTrafficHeadingToVehicleYawDegrees(vehiclePose.headingRadians) *
        degreesToRadians;
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    const auto point =
        passengerCabinModelPoint(passengerCabinStandingPoint(seat.position, seat.seatHeight));
    openbus::map::MapTrafficPose result;
    result.position = {vehiclePose.position[0] + point[0] * cosine - point[1] * sine,
                       vehiclePose.position[1] + point[0] * sine + point[1] * cosine,
                       vehiclePose.position[2] + point[2]};
    result.headingRadians = passengerCabinHeading(vehiclePose.headingRadians, seat.rotationDegrees);
    return result;
}

inline std::array<double, 3>
passengerCabinLocalPoint(const openbus::map::MapTrafficPose& vehiclePose,
                         const std::array<double, 3>& worldPosition) {
    constexpr double degreesToRadians = std::numbers::pi / 180.0;
    const double yaw =
        openbus::map::mapTrafficHeadingToVehicleYawDegrees(vehiclePose.headingRadians) *
        degreesToRadians;
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    const double offsetX = worldPosition[0] - vehiclePose.position[0];
    const double offsetY = worldPosition[1] - vehiclePose.position[1];
    return {offsetX * sine - offsetY * cosine, offsetX * cosine + offsetY * sine,
            worldPosition[2] - vehiclePose.position[2]};
}

struct PassengerCabinTravelPathData {
    std::vector<std::array<double, 3>> positions;
    std::vector<std::optional<std::size_t>> segmentLinkIndices;
};

inline PassengerCabinTravelPathData passengerCabinTravelPath(
    const VehicleConfig& cabin, const openbus::map::MapTrafficPose& vehiclePose,
    const std::array<double, 3>& startWorld, const std::array<double, 3>& endLocal, bool boarding,
    std::optional<std::size_t> doorIndex) {
    const auto& doors = boarding ? cabin.passengerEntryPathPoints : cabin.passengerExitPathPoints;
    if (cabin.passengerPathPoints.empty() || cabin.passengerPathLinks.empty() || !doorIndex ||
        *doorIndex >= doors.size()) {
        return {};
    }
    const auto pointIndex = [&](int id) -> std::optional<std::size_t> {
        const auto point = std::find_if(
            cabin.passengerPathPoints.begin(), cabin.passengerPathPoints.end(),
            [id](const PassengerPathPoint& candidate) { return candidate.index == id; });
        if (point == cabin.passengerPathPoints.end()) {
            return std::nullopt;
        }
        return static_cast<std::size_t>(std::distance(cabin.passengerPathPoints.begin(), point));
    };
    const auto linkIndex = [&](int from, int to) -> std::optional<std::size_t> {
        for (std::size_t index = 0; index < cabin.passengerPathLinks.size(); ++index) {
            const auto& link = cabin.passengerPathLinks[index];
            if ((link.from == from && link.to == to) ||
                (!link.oneWay && link.from == to && link.to == from)) {
                return index;
            }
        }
        return std::nullopt;
    };
    const std::array<double, 3> startLocal = passengerCabinLocalPoint(vehiclePose, startWorld);
    const auto closestPoint = [&](const std::array<double, 3>& position) {
        std::optional<std::size_t> closest;
        double closestDistance = std::numeric_limits<double>::infinity();
        for (std::size_t index = 0; index < cabin.passengerPathPoints.size(); ++index) {
            const auto& candidate = cabin.passengerPathPoints[index].position;
            const double dx = candidate[0] - position[0];
            const double dy = candidate[1] - position[1];
            const double dz = candidate[2] - position[2];
            const double distance = dx * dx + dy * dy + dz * dz;
            if (distance < closestDistance) {
                closest = index;
                closestDistance = distance;
            }
        }
        return closest;
    };
    const auto doorNode = pointIndex(doors[*doorIndex]);
    const auto startNode = boarding ? doorNode : closestPoint(startLocal);
    const auto endNode = boarding ? closestPoint(endLocal) : doorNode;
    if (!startNode || !endNode) {
        return {};
    }
    const auto route = passengerPathRoute(cabin.passengerPathPoints, cabin.passengerPathLinks,
                                          cabin.passengerPathPoints[*startNode].index,
                                          cabin.passengerPathPoints[*endNode].index);
    if (route.empty()) {
        return {};
    }
    PassengerCabinTravelPathData path;
    path.positions.reserve(route.size() + 2);
    path.segmentLinkIndices.reserve(route.size() + 1);
    path.positions.push_back(startLocal);
    const auto appendPoint = [&](const std::array<double, 3>& point,
                                 std::optional<std::size_t> link) {
        if (path.positions.back() != point) {
            path.segmentLinkIndices.push_back(link);
            path.positions.push_back(point);
        }
    };
    int previousPoint = -1;
    for (const int id : route) {
        const auto index = pointIndex(id);
        if (!index) {
            return {};
        }
        appendPoint(cabin.passengerPathPoints[*index].position,
                    previousPoint < 0 ? std::nullopt : linkIndex(previousPoint, id));
        previousPoint = id;
    }
    appendPoint(endLocal, std::nullopt);
    constexpr double degreesToRadians = std::numbers::pi / 180.0;
    const double yaw =
        openbus::map::mapTrafficHeadingToVehicleYawDegrees(vehiclePose.headingRadians) *
        degreesToRadians;
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    for (auto& point : path.positions) {
        const auto modelPoint = passengerCabinModelPoint(point);
        point = {vehiclePose.position[0] + modelPoint[0] * cosine - modelPoint[1] * sine,
                 vehiclePose.position[1] + modelPoint[0] * sine + modelPoint[1] * cosine,
                 vehiclePose.position[2] + modelPoint[2]};
    }
    return path;
}

inline void updatePassengerCabinPathOccupancy(openbus::map::MapPassengerServiceRuntime& service,
                                              const VehicleConfig& cabin,
                                              const openbus::map::MapTrafficPose& vehiclePose,
                                              const std::array<double, 3>& startWorld,
                                              const std::array<double, 3>& endLocal, bool boarding,
                                              std::size_t seatIndex,
                                              std::optional<std::size_t> doorIndex) {
    const auto path =
        passengerCabinTravelPath(cabin, vehiclePose, startWorld, endLocal, boarding, doorIndex);
    service.setPassengerPathLinkProgressProfile(passengerSeatPathLinkProfile(
        passengerPathLinkProgressProfile(path.positions, path.segmentLinkIndices), boarding,
        cabin.passengerPositions.at(seatIndex).seatHeight > 0.0));
}

} // namespace openbus::app
