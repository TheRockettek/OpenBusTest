#pragma once

#include "app/traffic/PassengerApproach.h"
#include "config/VehicleConfigLoader.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>
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

inline std::array<double, 3>
passengerCabinWorldPoint(const openbus::map::MapTrafficPose& vehiclePose,
                         const std::array<double, 3>& localPosition) {
    constexpr double degreesToRadians = std::numbers::pi / 180.0;
    const double yaw =
        openbus::map::mapTrafficHeadingToVehicleYawDegrees(vehiclePose.headingRadians) *
        degreesToRadians;
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    const auto modelPoint = passengerCabinModelPoint(localPosition);
    return {vehiclePose.position[0] + modelPoint[0] * cosine - modelPoint[1] * sine,
            vehiclePose.position[1] + modelPoint[0] * sine + modelPoint[1] * cosine,
            vehiclePose.position[2] + modelPoint[2]};
}

struct PassengerCabinTravelPathData {
    std::vector<std::array<double, 3>> positions;
    std::vector<std::optional<std::size_t>> segmentLinkIndices;
    std::optional<std::size_t> doorPointIndex;
};

struct PassengerCabinSectionPose {
    const VehicleConfig* configuration = nullptr;
    openbus::map::MapTrafficPose pose;
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
        if (id == doors[*doorIndex]) {
            path.doorPointIndex = path.positions.size() - 1;
        }
        previousPoint = id;
    }
    appendPoint(endLocal, std::nullopt);
    for (auto& point : path.positions) {
        point = passengerCabinWorldPoint(vehiclePose, point);
    }
    return path;
}

inline PassengerCabinTravelPathData passengerCabinTravelPath(
    const std::vector<PassengerCabinSectionPose>& sections, std::size_t startSectionIndex,
    const std::array<double, 3>& startWorld, std::size_t endSectionIndex,
    const std::array<double, 3>& endLocal, bool boarding, std::size_t doorSectionIndex,
    std::optional<std::size_t> doorIndex) {
    if (sections.empty() || startSectionIndex >= sections.size() ||
        endSectionIndex >= sections.size() || doorSectionIndex >= sections.size() ||
        std::ranges::any_of(sections, [](const PassengerCabinSectionPose& section) {
            return section.configuration == nullptr;
        }) ||
        !doorIndex) {
        return {};
    }

    struct Edge {
        std::size_t destination = 0;
        std::optional<std::size_t> linkIndex;
        double distance = 0.0;
    };
    std::vector<std::size_t> pointOffsets(sections.size() + 1, 0);
    std::vector<std::size_t> linkOffsets(sections.size() + 1, 0);
    for (std::size_t sectionIndex = 0; sectionIndex < sections.size(); ++sectionIndex) {
        const auto& cabin = *sections[sectionIndex].configuration;
        pointOffsets[sectionIndex + 1] =
            pointOffsets[sectionIndex] + cabin.passengerPathPoints.size();
        linkOffsets[sectionIndex + 1] =
            linkOffsets[sectionIndex] + cabin.passengerPathLinks.size();
    }
    if (pointOffsets.back() == 0) {
        return {};
    }

    const auto pointIndex = [&](std::size_t sectionIndex, int id)
        -> std::optional<std::size_t> {
        const auto& points = sections[sectionIndex].configuration->passengerPathPoints;
        const auto point = std::find_if(points.begin(), points.end(),
                                        [id](const PassengerPathPoint& candidate) {
                                            return candidate.index == id;
                                        });
        if (point == points.end()) {
            return std::nullopt;
        }
        return pointOffsets[sectionIndex] +
               static_cast<std::size_t>(std::distance(points.begin(), point));
    };
    const auto sectionForNode = [&](std::size_t nodeIndex) {
        return static_cast<std::size_t>(
            std::upper_bound(pointOffsets.begin(), pointOffsets.end(), nodeIndex) -
            pointOffsets.begin() - 1);
    };
    const auto worldPoint = [&](std::size_t nodeIndex) {
        const std::size_t sectionIndex = sectionForNode(nodeIndex);
        const std::size_t localIndex = nodeIndex - pointOffsets[sectionIndex];
        return passengerCabinWorldPoint(
            sections[sectionIndex].pose,
            sections[sectionIndex].configuration->passengerPathPoints[localIndex].position);
    };
    const auto targetWorld = passengerCabinWorldPoint(
        sections[endSectionIndex].pose, endLocal);
    std::vector<std::array<double, 3>> nodePositions(pointOffsets.back());
    for (std::size_t nodeIndex = 0; nodeIndex < nodePositions.size(); ++nodeIndex) {
        nodePositions[nodeIndex] = worldPoint(nodeIndex);
    }

    const auto distance = [](const std::array<double, 3>& first,
                             const std::array<double, 3>& second) {
        return std::hypot(second[0] - first[0], second[1] - first[1],
                          second[2] - first[2]);
    };
    std::vector<std::vector<Edge>> graph(pointOffsets.back());
    const auto addEdge = [&](std::size_t from, std::size_t to,
                             std::optional<std::size_t> linkIndex) {
        graph[from].push_back({to, linkIndex, distance(nodePositions[from], nodePositions[to])});
    };
    for (std::size_t sectionIndex = 0; sectionIndex < sections.size(); ++sectionIndex) {
        const auto& cabin = *sections[sectionIndex].configuration;
        for (std::size_t linkIndex = 0; linkIndex < cabin.passengerPathLinks.size(); ++linkIndex) {
            const auto& link = cabin.passengerPathLinks[linkIndex];
            const auto from = pointIndex(sectionIndex, link.from);
            const auto to = pointIndex(sectionIndex, link.to);
            if (!from || !to) {
                return {};
            }
            const std::size_t globalLinkIndex = linkOffsets[sectionIndex] + linkIndex;
            addEdge(*from, *to, globalLinkIndex);
            if (!link.oneWay) {
                addEdge(*to, *from, globalLinkIndex);
            }
        }
    }
    for (std::size_t sectionIndex = 0; sectionIndex + 1 < sections.size(); ++sectionIndex) {
        const auto& current = *sections[sectionIndex].configuration;
        const auto& next = *sections[sectionIndex + 1].configuration;
        const auto connect = [&](const std::optional<int>& currentPoint,
                                 const std::optional<int>& nextPoint) {
            if (!currentPoint || !nextPoint) {
                return;
            }
            const auto from = pointIndex(sectionIndex, *currentPoint);
            const auto to = pointIndex(sectionIndex + 1, *nextPoint);
            if (!from || !to) {
                return;
            }
            addEdge(*from, *to, std::nullopt);
            addEdge(*to, *from, std::nullopt);
        };
        connect(current.passengerPathPointToNextVehicle,
                next.passengerPathPointToPreviousVehicle);
        connect(current.passengerPathPointToPreviousVehicle,
                next.passengerPathPointToNextVehicle);
    }

    const auto closestNode = [&](std::size_t sectionIndex,
                                 const std::array<double, 3>& position)
        -> std::optional<std::size_t> {
        const std::size_t first = pointOffsets[sectionIndex];
        const std::size_t last = pointOffsets[sectionIndex + 1];
        if (first == last) {
            return std::nullopt;
        }
        std::size_t closest = first;
        double closestDistance = distance(nodePositions[first], position);
        for (std::size_t node = first + 1; node < last; ++node) {
            const double candidateDistance = distance(nodePositions[node], position);
            if (candidateDistance < closestDistance) {
                closest = node;
                closestDistance = candidateDistance;
            }
        }
        return closest;
    };
    const auto& doorCabin = *sections[doorSectionIndex].configuration;
    const auto& doors = boarding ? doorCabin.passengerEntryPathPoints
                                 : doorCabin.passengerExitPathPoints;
    if (*doorIndex >= doors.size()) {
        return {};
    }
    const auto doorNode = pointIndex(doorSectionIndex, doors[*doorIndex]);
    const auto startNode =
        boarding ? doorNode : closestNode(startSectionIndex, startWorld);
    const auto endNode = boarding ? closestNode(endSectionIndex, targetWorld) : doorNode;
    if (!startNode || !endNode) {
        return {};
    }

    std::vector<double> distances(graph.size(), std::numeric_limits<double>::infinity());
    std::vector<std::optional<std::size_t>> previousNodes(graph.size());
    std::vector<std::optional<std::size_t>> previousLinks(graph.size());
    std::vector<bool> visited(graph.size(), false);
    distances[*startNode] = 0.0;
    for (std::size_t iteration = 0; iteration < graph.size(); ++iteration) {
        std::optional<std::size_t> currentNode;
        for (std::size_t node = 0; node < graph.size(); ++node) {
            if (!visited[node] &&
                (!currentNode || distances[node] < distances[*currentNode])) {
                currentNode = node;
            }
        }
        if (!currentNode || !std::isfinite(distances[*currentNode])) {
            break;
        }
        if (*currentNode == *endNode) {
            break;
        }
        visited[*currentNode] = true;
        for (const Edge& edge : graph[*currentNode]) {
            const double candidateDistance = distances[*currentNode] + edge.distance;
            if (candidateDistance < distances[edge.destination]) {
                distances[edge.destination] = candidateDistance;
                previousNodes[edge.destination] = *currentNode;
                previousLinks[edge.destination] = edge.linkIndex;
            }
        }
    }
    if (!std::isfinite(distances[*endNode])) {
        return {};
    }
    std::vector<std::size_t> routeNodes;
    std::vector<std::optional<std::size_t>> routeLinks;
    for (std::size_t node = *endNode;; node = *previousNodes[node]) {
        routeNodes.push_back(node);
        if (node == *startNode) {
            break;
        }
        if (!previousNodes[node]) {
            return {};
        }
        routeLinks.push_back(previousLinks[node]);
    }
    std::ranges::reverse(routeNodes);
    std::ranges::reverse(routeLinks);

    PassengerCabinTravelPathData path;
    path.positions.push_back(startWorld);
    const auto appendPoint = [&](const std::array<double, 3>& point,
                                 std::optional<std::size_t> linkIndex) {
        if (path.positions.back() != point) {
            path.segmentLinkIndices.push_back(linkIndex);
            path.positions.push_back(point);
        }
        return path.positions.size() - 1;
    };
    for (std::size_t index = 0; index < routeNodes.size(); ++index) {
        const auto routePointIndex = appendPoint(
            nodePositions[routeNodes[index]], index == 0 ? std::nullopt : routeLinks[index - 1]);
        if (routeNodes[index] == *doorNode) {
            path.doorPointIndex = routePointIndex;
        }
    }
    appendPoint(targetWorld, std::nullopt);
    return path;
}

inline double passengerCabinTravelPathLength(const PassengerCabinTravelPathData& path,
                                             const std::array<double, 3>& startWorld,
                                             const std::array<double, 3>& endWorld) {
    double length = 0.0;
    const auto addSegment = [&](const auto& from, const auto& to) {
        const double segment = std::hypot(to[0] - from[0], to[1] - from[1], to[2] - from[2]);
        if (!std::isfinite(segment) || !std::isfinite(length + segment)) {
            throw std::invalid_argument("Passenger cabin path length must be finite");
        }
        length += segment;
    };
    if (path.positions.size() < 2) {
        addSegment(startWorld, endWorld);
    } else {
        for (std::size_t index = 1; index < path.positions.size(); ++index) {
            addSegment(path.positions[index - 1], path.positions[index]);
        }
    }
    return length;
}

inline double passengerCabinTravelDuration(const PassengerCabinTravelPathData& path,
                                           const std::array<double, 3>& startWorld,
                                           const std::array<double, 3>& endWorld, bool hasSeat) {
    const double length = passengerCabinTravelPathLength(path, startWorld, endWorld);
    const double walkingPortion = hasSeat ? 1.0 - passengerSeatTransitionPortion : 1.0;
    const double duration = length / (passengerApproachSpeedMetersPerSecond * walkingPortion);
    if (!std::isfinite(duration)) {
        throw std::invalid_argument("Passenger cabin travel duration must be finite");
    }
    return std::max(0.75, duration);
}

inline std::optional<double>
passengerAlightingDoorWaitProgress(const PassengerCabinTravelPathData& path, bool hasSeat) {
    if (!path.doorPointIndex || *path.doorPointIndex >= path.positions.size() ||
        path.positions.size() < 2) {
        return std::nullopt;
    }
    double length = 0.0;
    double distanceToDoor = 0.0;
    for (std::size_t index = 1; index < path.positions.size(); ++index) {
        const auto& from = path.positions[index - 1];
        const auto& to = path.positions[index];
        const double segment = std::hypot(to[0] - from[0], to[1] - from[1], to[2] - from[2]);
        if (!std::isfinite(segment) || !std::isfinite(length + segment)) {
            throw std::invalid_argument("Passenger cabin path length must be finite");
        }
        length += segment;
        if (index <= *path.doorPointIndex) {
            distanceToDoor = length;
        }
    }
    if (length <= 1.0e-9) {
        return std::nullopt;
    }
    constexpr double clearanceMeters = 0.5;
    const double walkingProgress = std::max(0.0, distanceToDoor - clearanceMeters) / length;
    const double progress = hasSeat ? passengerSeatTransitionPortion +
                                          (1.0 - passengerSeatTransitionPortion) * walkingProgress
                                    : walkingProgress;
    return std::min(progress, std::nextafter(1.0, 0.0));
}

inline double passengerCabinQueueProgress(const PassengerCabinTravelPathData& path,
                                          const std::array<double, 3>& startWorld,
                                          const std::array<double, 3>& endWorld, bool hasSeat,
                                          std::size_t queuePosition) {
    const double length = passengerCabinTravelPathLength(path, startWorld, endWorld);
    if (length <= 1.0e-9) {
        return 0.0;
    }
    constexpr double queueSpacingMeters = 0.6;
    const double nearDoorProgress =
        passengerAlightingDoorWaitProgress(path, hasSeat).value_or(std::nextafter(1.0, 0.0));
    const double walkingPortion = hasSeat ? 1.0 - passengerSeatTransitionPortion : 1.0;
    const double separation =
        walkingPortion * queueSpacingMeters * static_cast<double>(queuePosition) / length;
    return std::clamp(nearDoorProgress - separation, 0.0, std::nextafter(1.0, 0.0));
}

inline void updatePassengerCabinPathOccupancy(
    openbus::map::MapPassengerServiceRuntime& service, const VehicleConfig& cabin,
    const openbus::map::MapTrafficPose& vehiclePose, const std::array<double, 3>& startWorld,
    const std::array<double, 3>& endLocal, bool boarding, std::size_t seatIndex,
    std::optional<std::size_t> doorIndex,
    std::optional<std::array<double, 3>> fallbackVia = std::nullopt) {
    auto path =
        passengerCabinTravelPath(cabin, vehiclePose, startWorld, endLocal, boarding, doorIndex);
    const bool hasSeat = cabin.passengerPositions.at(seatIndex).seatHeight > 0.0;
    if (path.positions.empty() && fallbackVia) {
        path.positions = {startWorld, *fallbackVia,
                          passengerCabinWorldPoint(vehiclePose, endLocal)};
        path.segmentLinkIndices = {std::nullopt, std::nullopt};
    }
    const double duration = passengerCabinTravelDuration(
        path, startWorld, passengerCabinWorldPoint(vehiclePose, endLocal), hasSeat);
    if (boarding) {
        service.setPassengerAnimationDurationForSeat(seatIndex, duration);
    } else {
        service.setAlightingPassengerAnimationDuration(duration);
    }
    if (!boarding) {
        service.setAlightingDoorWaitProgress(passengerAlightingDoorWaitProgress(path, hasSeat));
    }
    auto profile = passengerSeatPathLinkProfile(
        passengerPathLinkProgressProfile(path.positions, path.segmentLinkIndices), boarding,
        hasSeat);
    if (boarding) {
        service.setPassengerPathLinkProgressProfileForSeat(seatIndex, std::move(profile));
    } else {
        service.setAlightingPassengerPathLinkProgressProfile(std::move(profile));
    }
}

inline void updatePassengerCabinPathOccupancy(
    openbus::map::MapPassengerServiceRuntime& service,
    const std::vector<PassengerCabinSectionPose>& sections, std::size_t startSectionIndex,
    const std::array<double, 3>& startWorld, std::size_t endSectionIndex,
    const std::array<double, 3>& endLocal, bool boarding, std::size_t seatIndex,
    bool hasSeat, std::size_t doorSectionIndex, std::optional<std::size_t> doorIndex,
    std::optional<std::array<double, 3>> fallbackVia = std::nullopt) {
    auto path = passengerCabinTravelPath(sections, startSectionIndex, startWorld,
                                         endSectionIndex, endLocal, boarding,
                                         doorSectionIndex, doorIndex);
    const auto endWorld = passengerCabinWorldPoint(sections.at(endSectionIndex).pose, endLocal);
    if (path.positions.empty() && fallbackVia) {
        path.positions = {startWorld, *fallbackVia, endWorld};
        path.segmentLinkIndices = {std::nullopt, std::nullopt};
    }
    const double duration =
        passengerCabinTravelDuration(path, startWorld, endWorld, hasSeat);
    if (boarding) {
        service.setPassengerAnimationDurationForSeat(seatIndex, duration);
    } else {
        service.setAlightingPassengerAnimationDuration(duration);
    }
    if (!boarding) {
        service.setAlightingDoorWaitProgress(passengerAlightingDoorWaitProgress(path, hasSeat));
    }
    auto profile = passengerSeatPathLinkProfile(
        passengerPathLinkProgressProfile(path.positions, path.segmentLinkIndices), boarding,
        hasSeat);
    if (boarding) {
        service.setPassengerPathLinkProgressProfileForSeat(seatIndex, std::move(profile));
    } else {
        service.setAlightingPassengerPathLinkProgressProfile(std::move(profile));
    }
}

} // namespace openbus::app
