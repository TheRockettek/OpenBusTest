#include "map/MapTrafficRuntime.h"

#include "map/MapSplineGeometry.h"
#include "map/MapTileWindow.h"

#include <algorithm>
#include <charconv>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <map>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <unordered_set>
#include <utility>

namespace openbus::map {

bool mapTrafficPoseOnActiveTile(const MapTrafficPose& pose,
                                const std::vector<std::pair<int, int>>& activeTiles) {
    const MapTileCoordinate tile{mapWorldTile(pose.position[0]), mapWorldTile(pose.position[1])};
    return std::find(activeTiles.begin(), activeTiles.end(), tile) != activeTiles.end();
}

namespace {

std::string_view trimRuleField(std::string_view field) {
    const auto whitespace = [](char value) {
        return value == ' ' || value == '\t' || value == '\r' || value == '\n';
    };
    while (!field.empty() && whitespace(field.front())) {
        field.remove_prefix(1);
    }
    while (!field.empty() && whitespace(field.back())) {
        field.remove_suffix(1);
    }
    return field;
}

std::string lowerRuleField(std::string_view field) {
    std::string value(trimRuleField(field));
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

template <typename Number> Number parseRuleNumber(std::string_view field, const char* name) {
    field = trimRuleField(field);
    Number value{};
    const auto parsed = std::from_chars(field.data(), field.data() + field.size(), value);
    if (field.empty() || parsed.ec != std::errc{} || parsed.ptr != field.data() + field.size()) {
        throw std::invalid_argument(std::string("Invalid map traffic rule ") + name + ": " +
                                    std::string(field));
    }
    if constexpr (std::is_floating_point_v<Number>) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument(std::string("Non-finite map traffic rule ") + name);
        }
    }
    return value;
}

const MapTrafficRouteSegment& routeSegmentAtDistance(const MapTrafficRoute& route,
                                                     double distanceMeters) {
    double routeDistance = distanceMeters;
    if (route.closed) {
        routeDistance = std::fmod(routeDistance, route.totalLength);
        if (routeDistance < 0.0) {
            routeDistance += route.totalLength;
        }
    } else if (route.loopStartDistance && routeDistance >= route.totalLength) {
        const double start = *route.loopStartDistance;
        routeDistance = start + std::fmod(routeDistance - start, route.totalLength - start);
    } else {
        routeDistance = std::clamp(routeDistance, 0.0, route.totalLength);
    }
    const auto segmentEnd = std::lower_bound(route.cumulativeEndDistances.begin(),
                                             route.cumulativeEndDistances.end(), routeDistance);
    const std::size_t segmentIndex =
        segmentEnd == route.cumulativeEndDistances.end()
            ? route.segments.size() - 1
            : static_cast<std::size_t>(segmentEnd - route.cumulativeEndDistances.begin());
    return route.segments[segmentIndex];
}

double mapTrafficRouteCurvatureAt(const MapTrafficRoute& route, double distanceMeters) {
    const double fromDistance = std::max(0.0, distanceMeters - 1.0);
    const double toDistance = route.closed || route.loopStartDistance
                                  ? distanceMeters + 1.0
                                  : std::min(route.totalLength, distanceMeters + 1.0);
    const auto from = sampleMapTrafficRoute(route, fromDistance, 0.0);
    const auto to = sampleMapTrafficRoute(route, toDistance, 0.0);
    if (!from || !to) {
        throw std::invalid_argument("Cannot sample traffic curve");
    }
    const double span =
        std::hypot(to->position[0] - from->position[0], to->position[1] - from->position[1]);
    const double angle =
        std::abs(std::remainder(to->headingRadians - from->headingRadians, 2.0 * std::numbers::pi));
    return angle / std::max(span, 0.01);
}

bool mapTrafficCurveFitsMinimumTurnRadius(const MapTrafficRoute& route, double distanceMeters,
                                          double minimumTurnRadiusMeters) {
    return minimumTurnRadiusMeters <= 0.0 ||
           mapTrafficRouteCurvatureAt(route, distanceMeters) * minimumTurnRadiusMeters <= 1.05;
}

} // namespace

bool mapTrafficRoutePositionOnJunctionPath(const MapTrafficRoute& route, double distanceMeters) {
    if (route.segments.empty() || route.segments.size() != route.cumulativeEndDistances.size() ||
        !std::isfinite(distanceMeters) || !std::isfinite(route.totalLength) ||
        route.totalLength <= 0.0) {
        throw std::invalid_argument("Traffic junction-path query inputs are invalid");
    }
    return !routeSegmentAtDistance(route, distanceMeters).points.empty();
}

bool mapTrafficActorOnJunctionPath(const MapTrafficActor& actor, double distanceMeters) {
    if (!std::isfinite(distanceMeters) || !std::isfinite(actor.lengthMeters) ||
        actor.lengthMeters <= 0.0) {
        throw std::invalid_argument("Traffic junction actor query inputs are invalid");
    }
    const double halfLength = actor.lengthMeters * 0.5;
    return mapTrafficRoutePositionOnJunctionPath(actor.route, distanceMeters) ||
           mapTrafficRoutePositionOnJunctionPath(actor.route, distanceMeters - halfLength) ||
           mapTrafficRoutePositionOnJunctionPath(actor.route, distanceMeters + halfLength);
}

double mapTrafficLaneOffsetMeters(bool leftHandTraffic, double distanceMeters) {
    if (!std::isfinite(distanceMeters) || distanceMeters < 0.0) {
        throw std::invalid_argument(
            "Map traffic lane offset must be a finite, non-negative distance");
    }
    return leftHandTraffic ? -distanceMeters : distanceMeters;
}

double mapTrafficHeadingToVehicleYawDegrees(double headingRadians) {
    if (!std::isfinite(headingRadians)) {
        throw std::invalid_argument("Map traffic heading must be finite");
    }
    return std::remainder(90.0 - headingRadians * 180.0 / std::numbers::pi, 360.0);
}

double mapTrafficHourAtElapsedSeconds(double startHour, double elapsedSeconds) {
    if (!std::isfinite(startHour) || !std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0) {
        throw std::invalid_argument(
            "Map traffic clock requires a finite start hour and elapsed time");
    }
    constexpr double hoursPerDay = 24.0;
    const double elapsedHours = std::fmod(elapsedSeconds / 3600.0, hoursPerDay);
    const double hour = std::fmod(std::fmod(startHour, hoursPerDay) + elapsedHours, hoursPerDay);
    return hour < 0.0 ? hour + hoursPerDay : hour;
}

double mapTrafficNextDepartureElapsedSeconds(double startHour, double elapsedSeconds,
                                             double departureMinutes) {
    if (!std::isfinite(startHour) || startHour < 0.0 || startHour >= 24.0 ||
        !std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0 ||
        !std::isfinite(departureMinutes) || departureMinutes < 0.0) {
        throw std::invalid_argument("Timetable departure clock inputs are invalid");
    }
    constexpr double secondsPerDay = 24.0 * 60.0 * 60.0;
    const double startSeconds = startHour * 60.0 * 60.0;
    const double currentSeconds = startSeconds + elapsedSeconds;
    const double dailyDepartureSeconds = std::fmod(departureMinutes * 60.0, secondsPerDay);
    double nextDepartureSeconds =
        std::floor(currentSeconds / secondsPerDay) * secondsPerDay + dailyDepartureSeconds;
    if (nextDepartureSeconds + 1.0e-9 < currentSeconds) {
        nextDepartureSeconds += secondsPerDay;
    }
    return nextDepartureSeconds - startSeconds;
}

bool mapTrafficScheduledDepartureDue(double elapsedSeconds, double departureElapsedSeconds) {
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0 ||
        !std::isfinite(departureElapsedSeconds) || departureElapsedSeconds < 0.0) {
        throw std::invalid_argument("Scheduled departure comparison inputs are invalid");
    }
    return elapsedSeconds + 1.0e-9 >= departureElapsedSeconds;
}

MapTrafficPathRules mapTrafficRulesForPath(const std::vector<MapTileRule>& rules,
                                           std::size_t pathIndex) {
    MapTrafficPathRules result;
    for (const MapTileRule& rule : rules) {
        const long long rulePathIndex = parseRuleNumber<long long>(rule.fields[0], "path index");
        if (rulePathIndex < 0) {
            throw std::invalid_argument("Map traffic rule path index cannot be negative");
        }
        if (static_cast<std::size_t>(rulePathIndex) != pathIndex) {
            continue;
        }
        const std::string kind = lowerRuleField(rule.fields[1]);
        if (kind != "speedlimit" && kind != "trafficdensity" && kind != "priority" &&
            kind != "no_cars" && kind != "bus" && kind != "trucks" && kind != "overtaking_prohib") {
            continue;
        }
        const long long groupIndex = parseRuleNumber<long long>(rule.fields[3], "AI group index");
        if (groupIndex < 0) {
            throw std::invalid_argument("Map traffic rule group index is invalid");
        }
        const auto existing = std::find_if(
            result.groups.begin(), result.groups.end(), [&](const MapTrafficGroupRules& group) {
                return group.groupIndex == static_cast<std::size_t>(groupIndex);
            });
        MapTrafficGroupRules* groupRules = nullptr;
        if (existing == result.groups.end()) {
            MapTrafficGroupRules group;
            group.groupIndex = static_cast<std::size_t>(groupIndex);
            result.groups.push_back(std::move(group));
            groupRules = &result.groups.back();
        } else {
            groupRules = &*existing;
        }
        if (rule.kill) {
            if (kind == "speedlimit") {
                groupRules->speedLimitKmh.reset();
            } else if (kind == "trafficdensity") {
                groupRules->density.reset();
            } else if (kind == "priority") {
                groupRules->priority.reset();
            } else if (kind == "no_cars") {
                groupRules->carsProhibited.reset();
            } else if (kind == "bus") {
                groupRules->busesAllowed.reset();
            } else if (kind == "trucks") {
                groupRules->trucksAllowed.reset();
            } else if (kind == "overtaking_prohib") {
                groupRules->overtakingProhibited.reset();
            }
            continue;
        }
        const double value = parseRuleNumber<double>(rule.fields[2], kind.c_str());
        if (kind == "speedlimit") {
            groupRules->speedLimitKmh = value > 0.0 ? std::optional<double>(value) : std::nullopt;
        } else if (kind == "trafficdensity") {
            if (value < 0.0) {
                throw std::invalid_argument("Map traffic density rule cannot be negative");
            }
            groupRules->density = value;
        } else if (kind == "priority") {
            if (value < 0.0 || value > 255.0) {
                throw std::invalid_argument("Map traffic priority must be in [0, 255]");
            }
            groupRules->priority = value;
        } else if (kind == "no_cars") {
            groupRules->carsProhibited = value != 0.0;
        } else if (kind == "bus") {
            groupRules->busesAllowed = value != 0.0;
        } else if (kind == "trucks") {
            groupRules->trucksAllowed = value != 0.0;
        } else if (kind == "overtaking_prohib") {
            groupRules->overtakingProhibited = value != 0.0;
        }
    }
    return result;
}

double mapTrafficPathDensityForVehicle(const MapTrafficPathRules& rules,
                                       const MapTrafficVehicleFilter& vehicle) noexcept {
    if (vehicle.unscheduledGroupIndex.has_value()) {
        const auto group = std::find_if(
            rules.groups.begin(), rules.groups.end(), [&](const MapTrafficGroupRules& candidate) {
                return candidate.groupIndex == *vehicle.unscheduledGroupIndex;
            });
        if (group != rules.groups.end() && group->density.has_value()) {
            return *group->density;
        }
    }
    return rules.density.value_or(1.0);
}

std::optional<double>
mapTrafficPathSpeedLimitForVehicle(const MapTrafficPathRules& rules,
                                   const MapTrafficVehicleFilter& vehicle) noexcept {
    if (vehicle.unscheduledGroupIndex.has_value()) {
        const auto group = std::find_if(
            rules.groups.begin(), rules.groups.end(), [&](const MapTrafficGroupRules& candidate) {
                return candidate.groupIndex == *vehicle.unscheduledGroupIndex;
            });
        if (group != rules.groups.end() && group->speedLimitKmh.has_value()) {
            return group->speedLimitKmh;
        }
    }
    return rules.speedLimitKmh;
}

double mapTrafficPathPriorityForVehicle(const MapTrafficPathRules& rules,
                                        const MapTrafficVehicleFilter& vehicle) noexcept {
    if (vehicle.unscheduledGroupIndex.has_value()) {
        const auto group = std::find_if(
            rules.groups.begin(), rules.groups.end(), [&](const MapTrafficGroupRules& candidate) {
                return candidate.groupIndex == *vehicle.unscheduledGroupIndex;
            });
        if (group != rules.groups.end() && group->priority.has_value()) {
            return *group->priority;
        }
    }
    return rules.priority;
}

bool mapTrafficPathAllowsVehicle(const MapTrafficPathRules& rules,
                                 const MapTrafficVehicleFilter& vehicle) {
    if (vehicle.scheduled) {
        return true;
    }
    const double density = mapTrafficPathDensityForVehicle(rules, vehicle);
    if (!std::isfinite(density) || density <= 0.0) {
        return false;
    }
    bool carsProhibited = rules.carsProhibited;
    bool busesAllowed = rules.busesAllowed;
    bool trucksAllowed = rules.trucksAllowed;
    if (vehicle.unscheduledGroupIndex.has_value()) {
        const auto group = std::find_if(
            rules.groups.begin(), rules.groups.end(), [&](const MapTrafficGroupRules& candidate) {
                return candidate.groupIndex == *vehicle.unscheduledGroupIndex;
            });
        if (group != rules.groups.end()) {
            carsProhibited = group->carsProhibited.value_or(carsProhibited);
            busesAllowed = group->busesAllowed.value_or(busesAllowed);
            trucksAllowed = group->trucksAllowed.value_or(trucksAllowed);
        }
    }
    switch (vehicle.vehicleType) {
    case 0:
        return !carsProhibited;
    case 1:
        return !carsProhibited || busesAllowed || trucksAllowed;
    case 2:
        return busesAllowed;
    case 3:
        return trucksAllowed;
    default:
        return false;
    }
}

std::vector<std::vector<double>>
mapTrafficPartitionScheduledDepartures(const std::vector<double>& departureMinutes,
                                       std::size_t partitionCount) {
    if (departureMinutes.empty() || partitionCount == 0 ||
        partitionCount > departureMinutes.size() ||
        std::any_of(departureMinutes.begin(), departureMinutes.end(),
                    [](double minute) { return !std::isfinite(minute) || minute < 0.0; })) {
        throw std::invalid_argument("Timetable departure partition is invalid");
    }
    std::vector<std::vector<double>> partitions(partitionCount);
    for (std::size_t index = 0; index < departureMinutes.size(); ++index) {
        partitions[index % partitionCount].push_back(departureMinutes[index]);
    }
    return partitions;
}

MapTrafficActor makeMapTrafficActor(MapTrafficRoute route, std::string modelPath,
                                    double speedMetersPerSecond, double laneOffsetMagnitudeMeters,
                                    bool leftHandTraffic,
                                    std::optional<std::string> driverModelPath) {
    if (modelPath.empty() || (driverModelPath.has_value() && driverModelPath->empty()) ||
        !std::isfinite(speedMetersPerSecond) || speedMetersPerSecond < 0.0 ||
        route.segments.empty() || route.cumulativeEndDistances.size() != route.segments.size() ||
        !std::isfinite(route.totalLength) || route.totalLength <= 0.0) {
        throw std::invalid_argument("Map traffic actor has invalid route, model path, or speed");
    }
    MapTrafficActor actor;
    actor.modelPath = std::move(modelPath);
    actor.driverModelPath = std::move(driverModelPath);
    actor.route = std::move(route);
    actor.speedMetersPerSecond = speedMetersPerSecond;
    actor.targetSpeedMetersPerSecond = speedMetersPerSecond;
    actor.cruiseSpeedMetersPerSecond = speedMetersPerSecond;
    actor.lateralOffsetMeters =
        mapTrafficLaneOffsetMeters(leftHandTraffic, laneOffsetMagnitudeMeters);
    return actor;
}

void resetMapTrafficActorForEntry(MapTrafficActor& actor) noexcept {
    actor.distanceMeters = 0.0;
    actor.speedMetersPerSecond = 0.0;
    actor.targetSpeedMetersPerSecond = 0.0;
    actor.junctionWaitSeconds = 0.0;
    actor.blockReason = MapTrafficBlockReason::None;
    actor.blockingActor = nullptr;
    actor.blockingObstacleIndex.reset();
    actor.junctionClaimEndMeters.reset();
}

const char* mapTrafficBlockReasonName(MapTrafficBlockReason reason) noexcept {
    switch (reason) {
    case MapTrafficBlockReason::None:
        return "none";
    case MapTrafficBlockReason::Leader:
        return "following leader";
    case MapTrafficBlockReason::Vehicle:
        return "AI vehicle ahead";
    case MapTrafficBlockReason::Obstacle:
        return "external obstacle";
    case MapTrafficBlockReason::CrossingReservation:
        return "crossing reservation";
    case MapTrafficBlockReason::JunctionTraffic:
        return "junction traffic";
    case MapTrafficBlockReason::Signal:
        return "traffic signal";
    case MapTrafficBlockReason::Collision:
        return "collision sweep";
    }
    return "unknown";
}

bool mapTrafficRoutesShareCompleteOrderedPath(const std::vector<int>& firstSplineIds,
                                              const std::vector<int>& secondSplineIds) {
    if (firstSplineIds.size() < 2 || secondSplineIds.size() < 2) {
        return false;
    }
    const auto contains = [](const std::vector<int>& route, const std::vector<int>& path) {
        return std::search(route.begin(), route.end(), path.begin(), path.end()) != route.end();
    };
    return contains(firstSplineIds, secondSplineIds) || contains(secondSplineIds, firstSplineIds);
}

std::vector<int> mapTrafficRoutePrefixMatchingTimetable(
    const std::vector<int>& timetableSplineIds, const std::vector<int>& routeSplineIds) {
    std::vector<int> longestPrefix;
    if (timetableSplineIds.size() < 2 || routeSplineIds.size() < 2) {
        return longestPrefix;
    }
    for (std::size_t timetableStart = 0; timetableStart + 1 < timetableSplineIds.size();
         ++timetableStart) {
        std::size_t matchingLength = 0;
        while (matchingLength < routeSplineIds.size() &&
               timetableStart + matchingLength < timetableSplineIds.size() &&
               routeSplineIds[matchingLength] ==
                   timetableSplineIds[timetableStart + matchingLength]) {
            ++matchingLength;
        }
        if (matchingLength > longestPrefix.size()) {
            longestPrefix.assign(routeSplineIds.begin(),
                                 routeSplineIds.begin() +
                                     static_cast<std::ptrdiff_t>(matchingLength));
        }
    }
    return longestPrefix.size() >= 2 ? longestPrefix : std::vector<int>{};
}

namespace {

bool placeMapTrafficActorNear(MapTrafficActor& actor, double startDistanceMeters, double centerX,
                              double centerY, double maximumDistanceMeters,
                              double endpointMarginMeters, bool loopOpenRoute) {
    if (!std::isfinite(startDistanceMeters) || startDistanceMeters < 0.0 ||
        !std::isfinite(centerX) || !std::isfinite(centerY) ||
        !std::isfinite(maximumDistanceMeters) || maximumDistanceMeters < 0.0 ||
        !std::isfinite(endpointMarginMeters) || endpointMarginMeters < 0.0 ||
        !std::isfinite(actor.route.totalLength) || actor.route.totalLength <= 0.0) {
        throw std::invalid_argument("Map traffic spawn position or route is invalid");
    }

    const double usableLength = std::max(0.0, actor.route.totalLength - endpointMarginMeters);
    if (loopOpenRoute && !actor.route.closed && usableLength <= 0.0) {
        return false;
    }
    const double distance =
        actor.route.closed ? std::fmod(startDistanceMeters, actor.route.totalLength)
        : loopOpenRoute && usableLength > 0.0 ? std::fmod(startDistanceMeters, usableLength)
                                              : std::min(startDistanceMeters, usableLength);
    const std::optional<MapTrafficPose> pose =
        sampleMapTrafficRoute(actor.route, distance, actor.lateralOffsetMeters);
    if (!pose.has_value()) {
        throw std::runtime_error("Could not sample the map traffic spawn route pose");
    }
    const double dx = pose->position[0] - centerX;
    const double dy = pose->position[1] - centerY;
    if (std::hypot(dx, dy) > maximumDistanceMeters) {
        return false;
    }
    actor.distanceMeters = distance;
    return true;
}

} // namespace

bool placeMapTrafficActorNear(MapTrafficActor& actor, double startDistanceMeters, double centerX,
                              double centerY, double maximumDistanceMeters,
                              double endpointMarginMeters) {
    return placeMapTrafficActorNear(actor, startDistanceMeters, centerX, centerY,
                                    maximumDistanceMeters, endpointMarginMeters, false);
}

bool placeMapTrafficActorNearLooping(MapTrafficActor& actor, double startDistanceMeters,
                                     double centerX, double centerY, double maximumDistanceMeters,
                                     double endpointMarginMeters) {
    return placeMapTrafficActorNear(actor, startDistanceMeters, centerX, centerY,
                                    maximumDistanceMeters, endpointMarginMeters, true);
}

std::optional<MapTrafficRoute> buildMapTrafficRoute(const MapSplinePlacementReference& start,
                                                    const MapSplineChainResolver& chainResolver,
                                                    std::size_t segmentLimit) {
    if (!start.placement.geometryValid || start.placement.length <= 0.0 || segmentLimit == 0) {
        return std::nullopt;
    }

    MapTrafficRoute route;
    std::unordered_set<int> visited;
    MapSplinePlacementReference current = start;
    bool reversed = false;
    visited.insert(current.placement.splineId);

    for (std::size_t index = 0; index < segmentLimit; ++index) {
        const MapSplinePlacement& spline = current.placement;
        if (!spline.geometryValid || !std::isfinite(spline.length) || spline.length <= 0.0) {
            break;
        }
        route.segments.push_back({current, reversed, 0.0, 0.0, {}, {}, {}});
        route.totalLength += spline.length;
        if (!std::isfinite(route.totalLength)) {
            return std::nullopt;
        }
        route.cumulativeEndDistances.push_back(route.totalLength);

        const int nextId = reversed ? spline.previousSplineId : spline.nextSplineId;
        if (nextId == 0 || !chainResolver) {
            break;
        }
        if (visited.contains(nextId)) {
            if (nextId == start.placement.splineId &&
                ((!reversed && start.placement.previousSplineId == spline.splineId) ||
                 (reversed && start.placement.nextSplineId == spline.splineId))) {
                route.closed = true;
            }
            break;
        }
        const std::optional<MapSplinePlacementReference> next = chainResolver(nextId);
        if (!next.has_value()) {
            break;
        }
        if (next->placement.previousSplineId == spline.splineId) {
            reversed = false;
        } else if (next->placement.nextSplineId == spline.splineId) {
            reversed = true;
        } else {
            break;
        }
        visited.insert(nextId);
        current = *next;
    }

    if (route.segments.empty() || route.totalLength <= 0.0) {
        return std::nullopt;
    }
    return route;
}

namespace {

double splineDistanceAtRouteStart(const MapTrafficRouteSegment& segment) {
    return segment.reversed ? segment.spline.placement.length : 0.0;
}

bool segmentsAreContinuousOnSameSpline(const MapTrafficRouteSegment& incoming,
                                       double incomingLength,
                                       const MapTrafficRouteSegment& outgoing) {
    if (!incoming.points.empty() || !outgoing.points.empty() ||
        incoming.spline.placement.splineId != outgoing.spline.placement.splineId ||
        incoming.spline.tileX != outgoing.spline.tileX ||
        incoming.spline.tileY != outgoing.spline.tileY ||
        incoming.reversed != outgoing.reversed) {
        return false;
    }

    const double expectedStart =
        splineDistanceAtRouteStart(incoming) + (incoming.reversed ? -incomingLength : incomingLength);
    return std::abs(expectedStart - splineDistanceAtRouteStart(outgoing)) <= 1.0e-6;
}

std::optional<MapTrafficPose> sampleMapTrafficRouteRaw(const MapTrafficRoute& route,
                                                       double distanceMeters,
                                                       double lateralOffsetMeters) {
    if (route.segments.empty() || route.segments.size() != route.cumulativeEndDistances.size() ||
        !std::isfinite(route.totalLength) || route.totalLength <= 0.0 ||
        !std::isfinite(distanceMeters) || !std::isfinite(lateralOffsetMeters)) {
        return std::nullopt;
    }

    double routeDistance = distanceMeters;
    if (route.closed) {
        routeDistance = std::fmod(routeDistance, route.totalLength);
        if (routeDistance < 0.0) {
            routeDistance += route.totalLength;
        }
    } else if (route.loopStartDistance && routeDistance >= route.totalLength) {
        const double start = *route.loopStartDistance;
        routeDistance = start + std::fmod(routeDistance - start, route.totalLength - start);
    } else {
        routeDistance = std::clamp(routeDistance, 0.0, route.totalLength);
    }

    const auto segmentEnd = std::lower_bound(route.cumulativeEndDistances.begin(),
                                             route.cumulativeEndDistances.end(), routeDistance);
    const std::size_t segmentIndex =
        segmentEnd == route.cumulativeEndDistances.end()
            ? route.segments.size() - 1
            : static_cast<std::size_t>(segmentEnd - route.cumulativeEndDistances.begin());
    const double segmentStart =
        segmentIndex == 0 ? 0.0 : route.cumulativeEndDistances[segmentIndex - 1];
    const double routeSegmentLength =
        route.cumulativeEndDistances[segmentIndex] - segmentStart;
    const MapTrafficRouteSegment& routeSegment = route.segments[segmentIndex];
    const MapSplinePlacement& spline = routeSegment.spline.placement;
    if (!routeSegment.points.empty()) {
        const auto edgeHeading = [&](std::size_t edge) {
            const auto& from = routeSegment.points[edge];
            const auto& to = routeSegment.points[edge + 1];
            return std::atan2(to[0] - from[0], to[1] - from[1]);
        };
        const auto vertexHeading = [&](std::size_t vertex) {
            const std::size_t count = routeSegment.points.size();
            if (count == 2) {
                return edgeHeading(0);
            }
            // Recover smooth tangents from tessellated arcs, including both endpoints.
            const std::size_t before = vertex == 0 ? 0 : vertex - 1;
            const std::size_t after = vertex + 1 >= count ? count - 2 : vertex;
            if (before == after) {
                const bool first = vertex == 0;
                const double heading = edgeHeading(before);
                const double difference =
                    first ? edgeHeading(1) - heading : heading - edgeHeading(before - 1);
                return heading +
                       (first ? -0.5 : 0.5) * std::remainder(difference, 2.0 * std::numbers::pi);
            }
            const double heading = edgeHeading(before);
            return heading +
                   0.5 * std::remainder(edgeHeading(after) - heading, 2.0 * std::numbers::pi);
        };
        double remaining = routeDistance - segmentStart;
        for (std::size_t index = 1; index < routeSegment.points.size(); ++index) {
            const auto& from = routeSegment.points[index - 1];
            const auto& to = routeSegment.points[index];
            const double dx = to[0] - from[0];
            const double dy = to[1] - from[1];
            const double dz = to[2] - from[2];
            const double length = std::hypot(dx, dy, dz);
            if (length <= 0.0) {
                continue;
            }
            if (remaining <= length || index + 1 == routeSegment.points.size()) {
                const double fraction = std::clamp(remaining / length, 0.0, 1.0);
                const double fromHeading = vertexHeading(index - 1);
                const double heading =
                    fromHeading + fraction * std::remainder(vertexHeading(index) - fromHeading,
                                                            2.0 * std::numbers::pi);
                const auto offset = mapSplineRightOffset(heading, lateralOffsetMeters);
                return MapTrafficPose{{from[0] + dx * fraction + offset[0],
                                       from[1] + dy * fraction + offset[1],
                                       from[2] + dz * fraction},
                                      heading,
                                      spline.splineId};
            }
            remaining -= length;
        }
        return std::nullopt;
    }
    if (!spline.geometryValid || spline.length <= 0.0) {
        return std::nullopt;
    }

    const double segmentDistance =
        std::clamp(routeDistance - segmentStart, 0.0, routeSegmentLength);
    const double splineDistance = splineDistanceAtRouteStart(routeSegment) +
                                  (routeSegment.reversed ? -segmentDistance : segmentDistance);
    if (splineDistance < -1.0e-6 || splineDistance > spline.length + 1.0e-6) {
        return std::nullopt;
    }
    const double boundedSplineDistance = std::clamp(splineDistance, 0.0, spline.length);
    const double tileOriginX =
        static_cast<double>(routeSegment.spline.tileX) * OMSI_TILE_SIZE_METERS;
    const double tileOriginY =
        static_cast<double>(routeSegment.spline.tileY) * OMSI_TILE_SIZE_METERS;
    const auto center =
        sampleMapSpline(tileOriginX + spline.localX, tileOriginY + spline.localY,
                        spline.rotationDegrees, spline.length, spline.radius,
                        boundedSplineDistance);
    if (!center.has_value()) {
        return std::nullopt;
    }

    const double heading =
        center->headingRadians + (routeSegment.reversed ? std::numbers::pi : 0.0);
    const double splineLateral =
        routeSegment.pathLateralMeters +
        (routeSegment.reversed ? -lateralOffsetMeters : lateralOffsetMeters);
    const std::array<double, 2> offset =
        mapSplineRightOffset(center->headingRadians, splineLateral);
    const double progress = boundedSplineDistance / spline.length;
    const double cant = spline.cantStart + (spline.cantEnd - spline.cantStart) * progress;
    const double elevation = mapSplineElevation(
        spline.elevation, spline.length, spline.gradientStart, spline.gradientEnd,
        spline.elevated ? std::optional<double>(spline.heightDelta) : std::nullopt,
        boundedSplineDistance);

    return MapTrafficPose{{center->x + offset[0], center->y + offset[1],
                           elevation + mapSplineCantHeightOffset(splineLateral, cant) +
                               routeSegment.pathHeightMeters},
                          std::remainder(heading, 2.0 * std::numbers::pi),
                          spline.splineId};
}

} // namespace

std::optional<MapTrafficPose> sampleMapTrafficRoute(const MapTrafficRoute& route,
                                                    double distanceMeters,
                                                    double lateralOffsetMeters) {
    const std::optional<MapTrafficPose> rawPose =
        sampleMapTrafficRouteRaw(route, distanceMeters, lateralOffsetMeters);
    if (!rawPose || route.segments.size() < 2) {
        return rawPose;
    }

    double routeDistance = distanceMeters;
    if (route.closed) {
        routeDistance = std::fmod(routeDistance, route.totalLength);
        if (routeDistance < 0.0) {
            routeDistance += route.totalLength;
        }
    } else if (route.loopStartDistance && routeDistance >= route.totalLength) {
        const double start = *route.loopStartDistance;
        routeDistance = start + std::fmod(routeDistance - start, route.totalLength - start);
    } else {
        routeDistance = std::clamp(routeDistance, 0.0, route.totalLength);
    }

    struct Transition {
        double distance = 0.0;
        double incomingLength = 0.0;
        double outgoingLength = 0.0;
        bool loopSeam = false;
    };
    std::vector<Transition> transitions;
    transitions.reserve(route.segments.size());
    const auto segmentLength = [&](std::size_t index) {
        const double start = index == 0 ? 0.0 : route.cumulativeEndDistances[index - 1];
        return route.cumulativeEndDistances[index] - start;
    };
    for (std::size_t index = 0; index + 1 < route.segments.size(); ++index) {
        if (route.segments[index].points.empty() &&
            route.segments[index + 1].points.empty()) {
            if (!segmentsAreContinuousOnSameSpline(route.segments[index], segmentLength(index),
                                                   route.segments[index + 1])) {
                transitions.push_back({route.cumulativeEndDistances[index], segmentLength(index),
                                       segmentLength(index + 1), false});
            }
        }
    }
    if (route.closed || route.loopStartDistance.has_value()) {
        std::size_t loopSegment = 0;
        if (route.loopStartDistance) {
            const auto end = std::lower_bound(route.cumulativeEndDistances.begin(),
                                              route.cumulativeEndDistances.end(),
                                              *route.loopStartDistance);
            loopSegment = end == route.cumulativeEndDistances.end()
                              ? route.segments.size() - 1
                              : static_cast<std::size_t>(end -
                                                         route.cumulativeEndDistances.begin());
        }
        if (route.segments.back().points.empty() && route.segments[loopSegment].points.empty()) {
            transitions.push_back({route.totalLength, segmentLength(route.segments.size() - 1),
                                   segmentLength(loopSegment), true});
        }
    }

    std::optional<Transition> selected;
    double selectedHalfWindow = 0.0;
    double transitionDistance = std::numeric_limits<double>::infinity();
    double queryDistance = routeDistance;
    for (const Transition& transition : transitions) {
        const double halfWindow =
            std::min({2.0, transition.incomingLength * 0.25,
                      transition.outgoingLength * 0.25});
        if (halfWindow < 0.1) {
            continue;
        }
        double unwrappedDistance = routeDistance;
        if (transition.loopSeam) {
            if (route.closed) {
                if (unwrappedDistance < halfWindow) {
                    unwrappedDistance += route.totalLength;
                }
            } else if (route.loopStartDistance &&
                       unwrappedDistance < *route.loopStartDistance + halfWindow) {
                unwrappedDistance += route.totalLength - *route.loopStartDistance;
            }
        }
        const double distanceToTransition = std::abs(unwrappedDistance - transition.distance);
        if (distanceToTransition <= halfWindow && distanceToTransition < transitionDistance) {
            selected = transition;
            selectedHalfWindow = halfWindow;
            transitionDistance = distanceToTransition;
            queryDistance = unwrappedDistance;
        }
    }
    if (!selected.has_value()) {
        return rawPose;
    }

    const double beginDistance = selected->distance - selectedHalfWindow;
    const double endDistance = selected->distance + selectedHalfWindow;
    const auto beginPose = sampleMapTrafficRouteRaw(route, beginDistance, lateralOffsetMeters);
    const auto endPose = sampleMapTrafficRouteRaw(route, endDistance, lateralOffsetMeters);
    const auto beforeJoin =
        sampleMapTrafficRouteRaw(route, selected->distance - 1.0e-4, lateralOffsetMeters);
    const auto afterJoin =
        sampleMapTrafficRouteRaw(route, selected->distance + 1.0e-4, lateralOffsetMeters);
    if (!beginPose || !endPose || !beforeJoin || !afterJoin ||
        std::hypot(beforeJoin->position[0] - afterJoin->position[0],
                   beforeJoin->position[1] - afterJoin->position[1],
                   beforeJoin->position[2] - afterJoin->position[2]) > 0.75) {
        return rawPose;
    }

    const double t = std::clamp((queryDistance - beginDistance) /
                                    (endDistance - beginDistance),
                                0.0, 1.0);
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    const double h10 = t3 - 2.0 * t2 + t;
    const double h01 = -2.0 * t3 + 3.0 * t2;
    const double h11 = t3 - t2;
    const double tangentLength = endDistance - beginDistance;
    MapTrafficPose result = *rawPose;
    result.position[0] =
        h00 * beginPose->position[0] +
        h10 * tangentLength * std::sin(beginPose->headingRadians) +
        h01 * endPose->position[0] +
        h11 * tangentLength * std::sin(endPose->headingRadians);
    result.position[1] =
        h00 * beginPose->position[1] +
        h10 * tangentLength * std::cos(beginPose->headingRadians) +
        h01 * endPose->position[1] +
        h11 * tangentLength * std::cos(endPose->headingRadians);
    result.position[2] = beginPose->position[2] +
                         (endPose->position[2] - beginPose->position[2]) * t;

    const double dh00 = 6.0 * t2 - 6.0 * t;
    const double dh10 = 3.0 * t2 - 4.0 * t + 1.0;
    const double dh01 = -6.0 * t2 + 6.0 * t;
    const double dh11 = 3.0 * t2 - 2.0 * t;
    const double tangentX =
        dh00 * beginPose->position[0] +
        dh10 * tangentLength * std::sin(beginPose->headingRadians) +
        dh01 * endPose->position[0] + dh11 * tangentLength * std::sin(endPose->headingRadians);
    const double tangentY =
        dh00 * beginPose->position[1] +
        dh10 * tangentLength * std::cos(beginPose->headingRadians) +
        dh01 * endPose->position[1] + dh11 * tangentLength * std::cos(endPose->headingRadians);
    if (std::hypot(tangentX, tangentY) > 1.0e-6) {
        result.headingRadians = std::atan2(tangentX, tangentY);
    } else {
        result.headingRadians =
            beginPose->headingRadians +
            t * std::remainder(endPose->headingRadians - beginPose->headingRadians,
                               2.0 * std::numbers::pi);
    }
    return result;
}

std::vector<MapTrafficSpawnCandidate> mapTrafficRouteSpawnCandidates(const MapTrafficRoute& route,
                                                                     double vehicleLengthMeters) {
    if (!std::isfinite(vehicleLengthMeters) || vehicleLengthMeters <= 0.0) {
        throw std::invalid_argument("Traffic spawn vehicle length must be positive and finite");
    }
    if (route.segments.size() != route.cumulativeEndDistances.size()) {
        throw std::invalid_argument("Traffic spawn route has inconsistent segment distances");
    }
    const double endpointMargin = std::max(8.0, vehicleLengthMeters * 0.5 + 4.0);
    const double spacing = std::max(12.0, vehicleLengthMeters + 6.0);
    std::vector<MapTrafficSpawnCandidate> candidates;
    for (std::size_t index = 0; index < route.segments.size(); ++index) {
        const MapTrafficRouteSegment& segment = route.segments[index];
        if (!segment.points.empty() || segment.spline.placement.splineId == 0 ||
            (index + 1 == route.segments.size() && !route.closed && !route.loopStartDistance)) {
            continue;
        }
        const double segmentStart = index == 0 ? 0.0 : route.cumulativeEndDistances[index - 1];
        const double segmentEnd = route.cumulativeEndDistances[index];
        if (!std::isfinite(segmentStart) || !std::isfinite(segmentEnd) ||
            segmentEnd <= segmentStart) {
            throw std::invalid_argument("Traffic spawn route has invalid segment distances");
        }
        for (double distance = segmentStart + endpointMargin;
             distance <= segmentEnd - endpointMargin; distance += spacing) {
            candidates.push_back({distance, segment.spline.tileX, segment.spline.tileY});
        }
    }
    return candidates;
}

bool advanceMapTrafficActor(MapTrafficActor& actor, double elapsedSeconds) {
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0 ||
        !std::isfinite(actor.speedMetersPerSecond) || actor.speedMetersPerSecond < 0.0 ||
        !std::isfinite(actor.distanceMeters) || !std::isfinite(actor.route.totalLength) ||
        actor.route.totalLength <= 0.0) {
        throw std::invalid_argument("Map traffic actor has invalid time, speed, or route state");
    }

    const double nextDistance = actor.distanceMeters + actor.speedMetersPerSecond * elapsedSeconds;
    if (!std::isfinite(nextDistance)) {
        throw std::invalid_argument("Map traffic actor distance overflowed");
    }
    if (actor.route.closed) {
        actor.distanceMeters = std::fmod(nextDistance, actor.route.totalLength);
        return true;
    }
    if (actor.route.loopStartDistance && nextDistance >= actor.route.totalLength) {
        const double loopStart = *actor.route.loopStartDistance;
        const double loopLength = actor.route.totalLength - loopStart;
        if (!std::isfinite(loopStart) || loopStart < 0.0 || loopLength <= 0.0) {
            throw std::invalid_argument("Map traffic route has an invalid loop start");
        }
        actor.distanceMeters = loopStart + std::fmod(nextDistance - loopStart, loopLength);
        return true;
    }
    if (nextDistance >= actor.route.totalLength) {
        actor.distanceMeters = actor.route.totalLength;
        return false;
    }
    actor.distanceMeters = nextDistance;
    return true;
}

bool advanceMapTrafficActorLooping(MapTrafficActor& actor, double elapsedSeconds) {
    const double previousDistance = actor.distanceMeters;
    if (advanceMapTrafficActor(actor, elapsedSeconds)) {
        return true;
    }
    const double overflowDistance =
        previousDistance + actor.speedMetersPerSecond * elapsedSeconds - actor.route.totalLength;
    actor.distanceMeters = std::fmod(std::max(0.0, overflowDistance), actor.route.totalLength);
    return true;
}

std::size_t advanceMapTrafficActors(std::vector<MapTrafficActor>& actors, double elapsedSeconds) {
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0) {
        throw std::invalid_argument("Map traffic update time must be finite and non-negative");
    }

    std::vector<MapTrafficActor> remaining;
    remaining.reserve(actors.size());
    std::size_t removedActors = 0;
    for (const MapTrafficActor& actor : actors) {
        MapTrafficActor advancedActor = actor;
        if (advanceMapTrafficActor(advancedActor, elapsedSeconds)) {
            remaining.push_back(std::move(advancedActor));
        } else {
            ++removedActors;
        }
    }
    actors.swap(remaining);
    return removedActors;
}

void MapTrafficRouteNetwork::addTileSplines(const std::vector<MapSplinePlacement>& splines,
                                            int tileX, int tileY) {
    if (finalized_) {
        throw std::logic_error("Cannot add splines to a finalized traffic route network");
    }
    for (const MapSplinePlacement& spline : splines) {
        if (spline.splineId == 0 || ambiguousSplineIds_.contains(spline.splineId)) {
            continue;
        }
        const auto [entry, inserted] = splinesById_.emplace(
            spline.splineId, MapSplinePlacementReference{spline, tileX, tileY});
        if (!inserted) {
            splinesById_.erase(entry);
            ambiguousSplineIds_.insert(spline.splineId);
        }
    }
}

void MapTrafficRouteNetwork::finalize() {
    finalized_ = true;
}

std::optional<MapTrafficRoute> MapTrafficRouteNetwork::buildRoute(int splineId,
                                                                  std::size_t segmentLimit) const {
    if (!finalized_) {
        throw std::logic_error("Traffic route network must be finalized before route lookup");
    }
    if (ambiguousSplineIds_.contains(splineId)) {
        return std::nullopt;
    }
    const auto start = splinesById_.find(splineId);
    if (start == splinesById_.end()) {
        return std::nullopt;
    }
    const MapSplineChainResolver resolver =
        [this](int linkedSplineId) -> std::optional<MapSplinePlacementReference> {
        if (ambiguousSplineIds_.contains(linkedSplineId)) {
            return std::nullopt;
        }
        const auto linked = splinesById_.find(linkedSplineId);
        return linked == splinesById_.end()
                   ? std::nullopt
                   : std::optional<MapSplinePlacementReference>(linked->second);
    };
    return buildMapTrafficRoute(start->second, resolver, segmentLimit);
}

std::vector<int> MapTrafficRouteNetwork::splineIds() const {
    if (!finalized_) {
        throw std::logic_error("Traffic route network must be finalized before listing splines");
    }
    std::vector<int> ids;
    ids.reserve(splinesById_.size());
    for (const auto& [splineId, spline] : splinesById_) {
        (void)spline;
        ids.push_back(splineId);
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

std::vector<int> MapTrafficRouteNetwork::routeStartSplineIds() const {
    if (!finalized_) {
        throw std::logic_error(
            "Traffic route network must be finalized before listing route starts");
    }

    const std::vector<int> ids = splineIds();
    std::vector<int> routeStarts;
    std::unordered_set<int> coveredSplineIds;
    const auto addRoute = [&](int splineId) {
        if (coveredSplineIds.contains(splineId)) {
            return;
        }
        const std::optional<MapTrafficRoute> route = buildRoute(splineId);
        if (!route.has_value()) {
            return;
        }
        routeStarts.push_back(splineId);
        for (const MapTrafficRouteSegment& segment : route->segments) {
            coveredSplineIds.insert(segment.spline.placement.splineId);
        }
    };

    for (const int splineId : ids) {
        const MapSplinePlacement& spline = splinesById_.at(splineId).placement;
        const auto previous = splinesById_.find(spline.previousSplineId);
        const bool hasLinkedPrevious = spline.previousSplineId != 0 &&
                                       !ambiguousSplineIds_.contains(spline.previousSplineId) &&
                                       previous != splinesById_.end() &&
                                       previous->second.placement.nextSplineId == splineId;
        if (!hasLinkedPrevious) {
            addRoute(splineId);
        }
    }

    // A fully linked cycle has no open start. Pick its lowest remaining spline ID so
    // route assignment stays stable across runs.
    for (const int splineId : ids) {
        addRoute(splineId);
    }
    std::sort(routeStarts.begin(), routeStarts.end());
    return routeStarts;
}

std::optional<MapTrafficSplineEndpointStatus>
MapTrafficRouteNetwork::endpointStatus(int splineId) const {
    if (!finalized_) {
        throw std::logic_error("Traffic route network must be finalized before endpoint lookup");
    }
    if (ambiguousSplineIds_.contains(splineId)) {
        return std::nullopt;
    }
    const auto spline = splinesById_.find(splineId);
    if (spline == splinesById_.end()) {
        return std::nullopt;
    }

    const auto isConnected = [&](int linkedSplineId) {
        if (linkedSplineId == 0 || ambiguousSplineIds_.contains(linkedSplineId)) {
            return false;
        }
        const auto linked = splinesById_.find(linkedSplineId);
        return linked != splinesById_.end() && linked->second.placement.geometryValid &&
               std::isfinite(linked->second.placement.length) &&
               linked->second.placement.length > 0.0 &&
               (linked->second.placement.previousSplineId == splineId ||
                linked->second.placement.nextSplineId == splineId);
    };
    return MapTrafficSplineEndpointStatus{isConnected(spline->second.placement.previousSplineId),
                                          isConnected(spline->second.placement.nextSplineId)};
}

void MapTrafficLaneNetwork::addSegment(MapTrafficRouteSegment segment) {
    if (finalized_) {
        throw std::logic_error("Cannot add lanes to a finalized traffic network");
    }
    double length = segment.spline.placement.length;
    if (segment.points.empty() && !segment.spline.placement.geometryValid) {
        throw std::invalid_argument("Traffic lane has invalid spline geometry");
    }
    if (!segment.points.empty()) {
        length = 0.0;
        for (const auto& point : segment.points) {
            if (!std::all_of(point.begin(), point.end(),
                             [](double value) { return std::isfinite(value); })) {
                throw std::invalid_argument("Traffic lane has non-finite points");
            }
        }
        for (std::size_t index = 1; index < segment.points.size(); ++index) {
            const auto& from = segment.points[index - 1];
            const auto& to = segment.points[index];
            length += std::hypot(to[0] - from[0], to[1] - from[1], to[2] - from[2]);
        }
    }
    if (!std::isfinite(length) || length <= 0.0) {
        throw std::invalid_argument("Traffic lane has invalid geometry");
    }
    lengths_.push_back(length);
    segments_.push_back(std::move(segment));
}

void MapTrafficLaneNetwork::finalize() {
    if (finalized_) {
        throw std::logic_error("Traffic lane network is already finalized");
    }
    struct Endpoints {
        MapTrafficPose start;
        MapTrafficPose end;
    };
    std::vector<Endpoints> endpoints;
    using Cell = std::pair<long long, long long>;
    constexpr double tolerance = 0.5;
    const auto cellFor = [](const std::array<double, 3>& position) -> Cell {
        return {static_cast<long long>(std::floor(position[0] / tolerance)),
                static_cast<long long>(std::floor(position[1] / tolerance))};
    };
    std::map<Cell, std::vector<std::size_t>> starts;
    for (std::size_t index = 0; index < segments_.size(); ++index) {
        MapTrafficRoute single;
        single.segments.push_back(segments_[index]);
        single.totalLength = lengths_[index];
        single.cumulativeEndDistances.push_back(single.totalLength);
        const auto start = sampleMapTrafficRoute(single, 0.0, 0.0);
        const auto end = sampleMapTrafficRoute(single, single.totalLength, 0.0);
        if (!start || !end) {
            throw std::runtime_error("Cannot sample traffic lane endpoints");
        }
        endpoints.push_back({*start, *end});
        double maximumCurvature = 0.0;
        for (double distance = 0.0; distance < single.totalLength; distance += 1.0) {
            const double endDistance = std::min(distance + 2.0, single.totalLength);
            const auto from = sampleMapTrafficRoute(single, distance, 0.0);
            const auto to = sampleMapTrafficRoute(single, endDistance, 0.0);
            const double span = std::hypot(to->position[0] - from->position[0],
                                           to->position[1] - from->position[1]);
            if (span > 1.0e-6) {
                maximumCurvature =
                    std::max(maximumCurvature,
                             std::abs(std::remainder(to->headingRadians - from->headingRadians,
                                                     2.0 * std::numbers::pi)) /
                                 span);
            }
        }
        maximumCurvatures_.push_back(maximumCurvature);
        starts[cellFor(start->position)].push_back(index);
        if (segments_[index].spline.placement.splineId != 0) {
            lanesBySpline_[segments_[index].spline.placement.splineId].push_back(index);
        }
    }
    next_.resize(segments_.size());
    incoming_.resize(segments_.size());
    for (std::size_t index = 0; index < segments_.size(); ++index) {
        const auto& end = endpoints[index].end;
        const Cell cell = cellFor(end.position);
        for (long long x = -1; x <= 1; ++x) {
            for (long long y = -1; y <= 1; ++y) {
                const auto bucket = starts.find({cell.first + x, cell.second + y});
                if (bucket == starts.end()) {
                    continue;
                }
                for (const std::size_t candidate : bucket->second) {
                    const auto& start = endpoints[candidate].start;
                    const double distance = std::hypot(end.position[0] - start.position[0],
                                                       end.position[1] - start.position[1]);
                    if (candidate == index &&
                        (distance > 1.0e-6 ||
                         std::abs(end.position[2] - start.position[2]) > 1.0e-6)) {
                        continue;
                    }
                    if (distance > tolerance ||
                        std::abs(end.position[2] - start.position[2]) > tolerance) {
                        continue;
                    }
                    const double alignment = std::cos(end.headingRadians - start.headingRadians);
                    if (alignment >= -0.01) {
                        next_[index].push_back(candidate);
                        ++incoming_[candidate];
                    }
                }
            }
        }
        std::stable_sort(
            next_[index].begin(), next_[index].end(), [&](std::size_t a, std::size_t b) {
                return std::cos(end.headingRadians - endpoints[a].start.headingRadians) >
                       std::cos(end.headingRadians - endpoints[b].start.headingRadians);
            });
    }
    // Report open ends that nearly meet an aligned lane start. These usually indicate
    // authoring gaps or height mismatches rather than intentional dead ends.
    constexpr double nearMissMeters = 3.0;
    stats_ = {segments_.size(), 0, 0};
    for (std::size_t index = 0; index < segments_.size(); ++index) {
        if (!next_[index].empty()) {
            continue;
        }
        ++stats_.deadEnds;
        const auto& end = endpoints[index].end;
        for (std::size_t candidate = 0; candidate < segments_.size(); ++candidate) {
            const auto& start = endpoints[candidate].start;
            if (candidate != index &&
                std::hypot(end.position[0] - start.position[0],
                           end.position[1] - start.position[1]) <= nearMissMeters &&
                std::abs(end.position[2] - start.position[2]) <= nearMissMeters &&
                std::cos(end.headingRadians - start.headingRadians) > 0.5) {
                ++stats_.nearMissDeadEnds;
                break;
            }
        }
    }
    for (const auto& [splineId, indexes] : lanesBySpline_) {
        for (std::size_t laneSelector = 0; laneSelector < indexes.size(); ++laneSelector) {
            const std::size_t index = indexes[laneSelector];
            if (segments_[index].points.empty() && !next_[index].empty()) {
                spawnLanes_.push_back({splineId, laneSelector});
            }
        }
    }
    std::sort(spawnLanes_.begin(), spawnLanes_.end(), [](const auto& a, const auto& b) {
        return std::pair{a.splineId, a.laneSelector} < std::pair{b.splineId, b.laneSelector};
    });
    finalized_ = true;
}

MapTrafficLaneNetworkStats MapTrafficLaneNetwork::stats() const {
    if (!finalized_) {
        throw std::logic_error("Traffic lane network must be finalized before reading stats");
    }
    return stats_;
}

std::size_t MapTrafficLaneNetwork::laneCount(int splineId) const {
    if (!finalized_) {
        throw std::logic_error("Traffic lane network must be finalized before lane lookup");
    }
    const auto lanes = lanesBySpline_.find(splineId);
    return lanes == lanesBySpline_.end() ? 0 : lanes->second.size();
}

std::optional<MapTrafficRoute>
MapTrafficLaneNetwork::buildRoute(int splineId, std::size_t laneSelector, std::size_t routeSeed,
                                  double minimumTurnRadiusMeters, MapTrafficVehicleFilter vehicle,
                                  const std::vector<int>& requiredSplineSequence) const {
    if (!finalized_) {
        throw std::logic_error("Traffic lane network must be finalized before route lookup");
    }
    if (!std::isfinite(minimumTurnRadiusMeters) || minimumTurnRadiusMeters < 0.0) {
        throw std::invalid_argument("Traffic route minimum turn radius is invalid");
    }
    const auto lanes = lanesBySpline_.find(splineId);
    if (lanes == lanesBySpline_.end() || lanes->second.empty()) {
        return std::nullopt;
    }
    std::size_t current = lanes->second[laneSelector % lanes->second.size()];
    if (!requiredSplineSequence.empty() &&
        segments_[current].spline.placement.splineId != requiredSplineSequence.front()) {
        return std::nullopt;
    }
    if (!mapTrafficPathAllowsVehicle(segments_[current].pathRules, vehicle)) {
        return std::nullopt;
    }
    if (minimumTurnRadiusMeters > 0.0 &&
        maximumCurvatures_[current] > 1.05 / minimumTurnRadiusMeters) {
        return std::nullopt;
    }
    MapTrafficRoute route;
    std::unordered_map<std::size_t, double> visited;
    const auto choicesFor = [&](std::size_t currentLane) {
        std::vector<std::size_t> choices;
        for (const auto candidate : next_[currentLane]) {
            if (!mapTrafficPathAllowsVehicle(segments_[candidate].pathRules, vehicle)) {
                continue;
            }
            if (minimumTurnRadiusMeters > 0.0) {
                if (maximumCurvatures_[candidate] > 1.05 / minimumTurnRadiusMeters) {
                    continue;
                }
                MapTrafficRoute join;
                join.segments = {segments_[currentLane], segments_[candidate]};
                join.cumulativeEndDistances = {lengths_[currentLane],
                                               lengths_[currentLane] + lengths_[candidate]};
                join.totalLength = join.cumulativeEndDistances.back();
                if (!mapTrafficCurveFitsMinimumTurnRadius(join, lengths_[currentLane],
                                                          minimumTurnRadiusMeters)) {
                    continue;
                }
            }
            choices.push_back(candidate);
        }
        if (!requiredSplineSequence.empty() &&
            route.segments.size() < requiredSplineSequence.size()) {
            const int requiredSplineId = requiredSplineSequence[route.segments.size()];
            std::erase_if(choices, [&](std::size_t candidate) {
                return segments_[candidate].spline.placement.splineId != requiredSplineId;
            });
        }
        // Seed zero retains straight-first routing for route inspection and timetable matching.
        if (!choices.empty() && routeSeed != 0) {
            // Mix lane identity into the seed so successive junctions do not share residues.
            std::uint64_t choiceSeed =
                static_cast<std::uint64_t>(routeSeed) +
                0x9e3779b97f4a7c15ULL * (static_cast<std::uint64_t>(currentLane) + 1U);
            choiceSeed = (choiceSeed ^ (choiceSeed >> 30U)) * 0xbf58476d1ce4e5b9ULL;
            choiceSeed = (choiceSeed ^ (choiceSeed >> 27U)) * 0x94d049bb133111ebULL;
            choiceSeed ^= choiceSeed >> 31U;
            double maximumDensity = 0.0;
            for (const std::size_t choice : choices) {
                maximumDensity =
                    std::max(maximumDensity,
                             mapTrafficPathDensityForVehicle(segments_[choice].pathRules, vehicle));
            }
            double totalWeight = 0.0;
            for (const std::size_t choice : choices) {
                totalWeight +=
                    mapTrafficPathDensityForVehicle(segments_[choice].pathRules, vehicle) /
                    maximumDensity;
            }
            double selection = static_cast<double>(
                static_cast<long double>(choiceSeed) /
                static_cast<long double>(std::numeric_limits<std::uint64_t>::max()) *
                static_cast<long double>(totalWeight));
            std::size_t choice = choices.size() - 1;
            for (std::size_t index = 0; index < choices.size(); ++index) {
                const double weight =
                    mapTrafficPathDensityForVehicle(segments_[choices[index]].pathRules, vehicle) /
                    maximumDensity;
                if (selection < weight) {
                    choice = index;
                    break;
                }
                selection -= weight;
            }
            std::rotate(choices.begin(), choices.begin() + choice, choices.end());
        }
        return choices;
    };
    struct Frame {
        std::size_t lane;
        std::vector<std::size_t> choices;
        std::size_t nextChoice = 0;
    };
    std::vector<Frame> stack;
    std::unordered_set<std::size_t> blocked;
    const auto append = [&](std::size_t lane) {
        visited.emplace(lane, route.totalLength);
        route.segments.push_back(segments_[lane]);
        route.totalLength += lengths_[lane];
        route.cumulativeEndDistances.push_back(route.totalLength);
        stack.push_back({lane, choicesFor(lane)});
    };
    append(current);
    while (!stack.empty()) {
        auto& frame = stack.back();
        if (next_[frame.lane].empty() && (requiredSplineSequence.empty() ||
                                          route.segments.size() >= requiredSplineSequence.size() ||
                                          route.segments.size() >= 2)) {
            return route;
        }
        if (frame.nextChoice == frame.choices.size()) {
            // Do not invent an exit where geometry continues but the vehicle cannot turn.
            blocked.insert(frame.lane);
            visited.erase(frame.lane);
            stack.pop_back();
            route.segments.pop_back();
            route.cumulativeEndDistances.pop_back();
            route.totalLength =
                route.cumulativeEndDistances.empty() ? 0.0 : route.cumulativeEndDistances.back();
            continue;
        }
        current = frame.choices[frame.nextChoice++];
        if (blocked.contains(current)) {
            continue;
        }
        if (visited.contains(current)) {
            if (!requiredSplineSequence.empty() &&
                route.segments.size() < requiredSplineSequence.size()) {
                continue;
            }
            route.loopStartDistance = visited.at(current);
            route.closed = *route.loopStartDistance == 0.0;
            return route;
        }
        append(current);
    }
    return std::nullopt;
}

std::vector<MapTrafficLaneStart> MapTrafficLaneNetwork::entryLanes() const {
    if (!finalized_) {
        throw std::logic_error("Traffic lane network must be finalized before entry lookup");
    }
    std::vector<MapTrafficLaneStart> entries;
    for (const auto& [splineId, lanes] : lanesBySpline_) {
        for (std::size_t lane = 0; lane < lanes.size(); ++lane) {
            if (incoming_[lanes[lane]] == 0 && !next_[lanes[lane]].empty()) {
                entries.push_back({splineId, lane});
            }
        }
    }
    std::sort(entries.begin(), entries.end(), [](const auto& a, const auto& b) {
        return std::pair{a.splineId, a.laneSelector} < std::pair{b.splineId, b.laneSelector};
    });
    return entries;
}

std::optional<MapTrafficLaneStart>
MapTrafficLaneNetwork::selectEntryLane(MapTrafficVehicleFilter vehicle,
                                       std::size_t selectionSeed) const {
    if (!finalized_) {
        throw std::logic_error("Traffic lane network must be finalized before entry lookup");
    }
    if (selectionSeed == 0) {
        throw std::invalid_argument("Traffic entry selection seed must be nonzero");
    }
    return selectLane(vehicle, selectionSeed, entryLanes());
}

std::vector<MapTrafficLaneStart> MapTrafficLaneNetwork::spawnLanes() const {
    if (!finalized_) {
        throw std::logic_error("Traffic lane network must be finalized before spawn lookup");
    }
    return spawnLanes_;
}

std::optional<MapTrafficLaneStart>
MapTrafficLaneNetwork::selectSpawnLane(MapTrafficVehicleFilter vehicle,
                                       std::size_t selectionSeed) const {
    if (!finalized_) {
        throw std::logic_error("Traffic lane network must be finalized before spawn lookup");
    }
    if (selectionSeed == 0) {
        throw std::invalid_argument("Traffic spawn selection seed must be nonzero");
    }
    return selectLane(vehicle, selectionSeed, spawnLanes_);
}

std::optional<MapTrafficLaneStart>
MapTrafficLaneNetwork::selectLane(MapTrafficVehicleFilter vehicle, std::size_t selectionSeed,
                                  const std::vector<MapTrafficLaneStart>& laneStarts) const {
    if (!finalized_) {
        throw std::logic_error("Traffic lane network must be finalized before lane selection");
    }
    struct WeightedEntry {
        MapTrafficLaneStart entry;
        double density;
    };
    std::vector<WeightedEntry> candidates;
    double maximumDensity = 0.0;
    for (const MapTrafficLaneStart& entry : laneStarts) {
        const auto laneGroup = lanesBySpline_.find(entry.splineId);
        if (laneGroup == lanesBySpline_.end() || laneGroup->second.empty()) {
            continue;
        }
        const MapTrafficRouteSegment& segment =
            segments_[laneGroup->second[entry.laneSelector % laneGroup->second.size()]];
        if (!mapTrafficPathAllowsVehicle(segment.pathRules, vehicle)) {
            continue;
        }
        const double density = mapTrafficPathDensityForVehicle(segment.pathRules, vehicle);
        maximumDensity = std::max(maximumDensity, density);
        candidates.push_back({entry, density});
    }
    if (candidates.empty()) {
        return std::nullopt;
    }
    double totalWeight = 0.0;
    for (const WeightedEntry& candidate : candidates) {
        totalWeight += candidate.density / maximumDensity;
    }
    std::uint64_t mixedSeed = static_cast<std::uint64_t>(selectionSeed);
    mixedSeed += 0x9e3779b97f4a7c15ULL;
    mixedSeed = (mixedSeed ^ (mixedSeed >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    mixedSeed = (mixedSeed ^ (mixedSeed >> 27U)) * 0x94d049bb133111ebULL;
    mixedSeed ^= mixedSeed >> 31U;
    double selection =
        static_cast<double>(static_cast<long double>(mixedSeed) /
                            static_cast<long double>(std::numeric_limits<std::uint64_t>::max()) *
                            static_cast<long double>(totalWeight));
    for (const WeightedEntry& candidate : candidates) {
        const double weight = candidate.density / maximumDensity;
        if (selection < weight) {
            return candidate.entry;
        }
        selection -= weight;
    }
    return candidates.back().entry;
}

namespace {

void validateFootprint(const MapTrafficObstacle& obstacle) {
    if (!std::all_of(obstacle.pose.position.begin(), obstacle.pose.position.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !std::isfinite(obstacle.pose.headingRadians) || !std::isfinite(obstacle.lengthMeters) ||
        obstacle.lengthMeters <= 0.0 || !std::isfinite(obstacle.widthMeters) ||
        obstacle.widthMeters <= 0.0) {
        throw std::invalid_argument("Traffic obstacle footprint is invalid");
    }
}

bool footprintsOverlap(const MapTrafficObstacle& a, const MapTrafficObstacle& b, double clearance) {
    if (std::abs(a.pose.position[2] - b.pose.position[2]) > 3.0) {
        return false;
    }
    const auto axes = [](double heading) {
        return std::array<std::array<double, 2>, 2>{
            std::array<double, 2>{std::sin(heading), std::cos(heading)},
            std::array<double, 2>{std::cos(heading), -std::sin(heading)}};
    };
    const auto aa = axes(a.pose.headingRadians);
    const auto ba = axes(b.pose.headingRadians);
    const double dx = b.pose.position[0] - a.pose.position[0];
    const double dy = b.pose.position[1] - a.pose.position[1];
    const auto dot = [](const auto& u, const auto& v) { return u[0] * v[0] + u[1] * v[1]; };
    for (const auto& axis : {aa[0], aa[1], ba[0], ba[1]}) {
        const double extent = (a.lengthMeters * std::abs(dot(axis, aa[0])) +
                               a.widthMeters * std::abs(dot(axis, aa[1])) +
                               b.lengthMeters * std::abs(dot(axis, ba[0])) +
                               b.widthMeters * std::abs(dot(axis, ba[1]))) *
                                  0.5 +
                              clearance;
        if (std::abs(dx * axis[0] + dy * axis[1]) >= extent) {
            return false;
        }
    }
    return true;
}

MapTrafficObstacle actorFootprint(const MapTrafficActor& actor, double distance) {
    const auto pose = sampleMapTrafficRoute(actor.route, distance, actor.lateralOffsetMeters);
    if (!pose) {
        throw std::invalid_argument("Cannot sample traffic driving route");
    }
    return {*pose, actor.lengthMeters, actor.widthMeters};
}

} // namespace

bool mapTrafficActorBehindObstacle(const MapTrafficActor& actor,
                                   const MapTrafficObstacle& obstacle,
                                   double maximumGapMeters) {
    if (!std::isfinite(maximumGapMeters) || maximumGapMeters < 0.0) {
        throw std::invalid_argument("Maximum traffic following gap must be finite and non-negative");
    }
    validateFootprint(obstacle);
    const MapTrafficObstacle actorPosition = actorFootprint(actor, actor.distanceMeters);
    validateFootprint(actorPosition);
    const double heading = actorPosition.pose.headingRadians;
    if (std::cos(heading - obstacle.pose.headingRadians) < 0.7) {
        return false;
    }
    const double deltaX = obstacle.pose.position[0] - actorPosition.pose.position[0];
    const double deltaY = obstacle.pose.position[1] - actorPosition.pose.position[1];
    const double bumperGap =
        deltaX * std::sin(heading) + deltaY * std::cos(heading) -
        (actor.lengthMeters + obstacle.lengthMeters) * 0.5;
    const double lateralGap =
        std::abs(deltaX * std::cos(heading) - deltaY * std::sin(heading)) -
        (actor.widthMeters + obstacle.widthMeters) * 0.5;
    return bumperGap >= 0.0 && bumperGap <= maximumGapMeters && lateralGap <= 0.75;
}

bool mapTrafficHasClearance(const MapTrafficObstacle& vehicle,
                            const std::vector<MapTrafficObstacle>& obstacles,
                            double clearanceMeters) {
    validateFootprint(vehicle);
    if (!std::isfinite(clearanceMeters) || clearanceMeters < 0.0) {
        throw std::invalid_argument("Traffic clearance must be finite and non-negative");
    }
    for (const auto& obstacle : obstacles) {
        validateFootprint(obstacle);
        if (footprintsOverlap(vehicle, obstacle, clearanceMeters)) {
            return false;
        }
    }
    return true;
}

namespace {

struct TrafficSignalApproach {
    const MapTrafficSignalSchedule* schedule = nullptr;
    double distanceToStopMeters = 0.0;
};

std::optional<TrafficSignalApproach> nextTrafficSignalApproach(const MapTrafficActor& actor) {
    const MapTrafficRoute& route = actor.route;
    if (route.segments.size() != route.cumulativeEndDistances.size() ||
        !std::isfinite(actor.distanceMeters) || actor.distanceMeters < 0.0) {
        return std::nullopt;
    }

    std::optional<TrafficSignalApproach> nearest;
    double segmentStart = 0.0;
    for (std::size_t index = 0; index < route.segments.size(); ++index) {
        const MapTrafficRouteSegment& segment = route.segments[index];
        const double segmentEnd = route.cumulativeEndDistances[index];
        if (!segment.trafficSignal.has_value() || !std::isfinite(segmentEnd) ||
            segmentEnd <= segmentStart) {
            segmentStart = segmentEnd;
            continue;
        }

        double distanceToSegmentStart = 0.0;
        if (segmentStart <= 1.0e-9 && actor.distanceMeters <= 1.0e-9) {
            distanceToSegmentStart = 0.0;
        } else if (actor.distanceMeters < segmentStart) {
            distanceToSegmentStart = segmentStart - actor.distanceMeters;
        } else if (actor.distanceMeters <= segmentEnd) {
            if (route.closed) {
                distanceToSegmentStart = route.totalLength - actor.distanceMeters + segmentStart;
            } else if (route.loopStartDistance.has_value() &&
                       segmentStart >= *route.loopStartDistance) {
                distanceToSegmentStart = route.totalLength - actor.distanceMeters + segmentStart -
                                         *route.loopStartDistance;
            } else {
                segmentStart = segmentEnd;
                continue;
            }
        } else if (route.closed) {
            distanceToSegmentStart = route.totalLength - actor.distanceMeters + segmentStart;
        } else if (route.loopStartDistance.has_value() &&
                   segmentStart >= *route.loopStartDistance) {
            distanceToSegmentStart =
                route.totalLength - actor.distanceMeters + segmentStart - *route.loopStartDistance;
        } else {
            segmentStart = segmentEnd;
            continue;
        }

        const double stopOffset = actor.lengthMeters * 0.5 + 0.75;
        const double distanceToStop = std::max(0.0, distanceToSegmentStart - stopOffset);
        if (!nearest.has_value() || distanceToStop < nearest->distanceToStopMeters) {
            nearest = TrafficSignalApproach{&*segment.trafficSignal, distanceToStop};
        }
        segmentStart = segmentEnd;
    }
    return nearest;
}

std::optional<int> trafficSignalStateAtTime(const MapTrafficSignalSchedule& schedule,
                                            double elapsedSeconds) {
    return mapTrafficSignalStateAtTime(schedule, elapsedSeconds);
}

} // namespace

bool mapTrafficActorWaitingAtSignal(const MapTrafficActor& actor, double signalClockSeconds) {
    if (!std::isfinite(signalClockSeconds) || signalClockSeconds < 0.0) {
        throw std::invalid_argument("Traffic signal clock must be finite and non-negative");
    }
    const std::optional<TrafficSignalApproach> approach = nextTrafficSignalApproach(actor);
    if (!approach.has_value() || approach->distanceToStopMeters > 1.0) {
        return false;
    }
    const std::optional<int> state =
        trafficSignalStateAtTime(*approach->schedule, signalClockSeconds);
    return !state.has_value() || *state < 6 || *state > 8;
}

std::optional<MapTrafficSignalStopDebugInfo>
mapTrafficNextSignalStopDebugInfo(const MapTrafficActor& actor, double signalClockSeconds) {
    if (!std::isfinite(signalClockSeconds) || signalClockSeconds < 0.0) {
        throw std::invalid_argument("Traffic signal clock must be finite and non-negative");
    }
    const std::optional<TrafficSignalApproach> approach = nextTrafficSignalApproach(actor);
    if (!approach.has_value()) {
        return std::nullopt;
    }
    const std::optional<MapTrafficPose> pose =
        sampleMapTrafficRoute(actor.route, actor.distanceMeters + approach->distanceToStopMeters,
                              actor.lateralOffsetMeters);
    if (!pose.has_value()) {
        return std::nullopt;
    }
    const std::optional<int> state =
        trafficSignalStateAtTime(*approach->schedule, signalClockSeconds);
    return MapTrafficSignalStopDebugInfo{*pose, approach->distanceToStopMeters,
                                         !state.has_value() || *state < 6 || *state > 8};
}

void advanceMapTrafficSignalControllers(
    const std::vector<std::shared_ptr<MapTrafficSignalController>>& controllers,
    const std::vector<MapTrafficActor*>& actors, double elapsedSeconds,
    double initialClockSeconds) {
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0 ||
        !std::isfinite(initialClockSeconds) || initialClockSeconds < 0.0) {
        throw std::invalid_argument("Traffic signal controller update is invalid");
    }
    std::unordered_set<MapTrafficSignalController*> uniqueControllers;
    for (const auto& controller : controllers) {
        if (controller == nullptr) {
            throw std::invalid_argument("Traffic signal controller is null");
        }
        if (uniqueControllers.insert(controller.get()).second) {
            controller->clearRequests();
        }
    }
    for (const MapTrafficActor* actor : actors) {
        if (actor == nullptr) {
            throw std::invalid_argument("Traffic signal actor is null");
        }
        const std::optional<TrafficSignalApproach> approach = nextTrafficSignalApproach(*actor);
        if (!approach.has_value() || approach->schedule->controller == nullptr) {
            continue;
        }
        const double approachDistance = approach->schedule->approachDistanceMeters.value_or(50.0);
        const bool stoppedAtLine =
            actor->speedMetersPerSecond < 0.5 && approach->distanceToStopMeters <= 3.0;
        if (stoppedAtLine ||
            (approachDistance > 0.0 && approach->distanceToStopMeters <= approachDistance)) {
            approach->schedule->controller->request(approach->schedule->lightIndex);
        }
    }
    for (const auto& controller : controllers) {
        if (uniqueControllers.erase(controller.get()) > 0) {
            controller->advance(elapsedSeconds, initialClockSeconds);
        }
    }
}

bool advanceMapTrafficActorLoopingAvoidingObstacles(
    MapTrafficActor& actor, const std::vector<MapTrafficObstacle>& obstacles, double lengthMeters,
    double widthMeters, double elapsedSeconds, double signalClockSeconds) {
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0 ||
        !std::isfinite(signalClockSeconds) || signalClockSeconds < 0.0 ||
        !std::isfinite(lengthMeters) || lengthMeters <= 0.0 || !std::isfinite(widthMeters) ||
        widthMeters <= 0.0 || !std::isfinite(actor.speedMetersPerSecond) ||
        actor.speedMetersPerSecond < 0.0 || !std::isfinite(actor.cruiseSpeedMetersPerSecond) ||
        actor.cruiseSpeedMetersPerSecond < 0.0 || !std::isfinite(actor.distanceMeters) ||
        actor.distanceMeters < 0.0 || !std::isfinite(actor.speedMetersPerSecond * elapsedSeconds) ||
        actor.speedMetersPerSecond * elapsedSeconds > 10000.0) {
        throw std::invalid_argument("Obstacle-aware actor movement inputs are invalid");
    }
    for (const MapTrafficObstacle& obstacle : obstacles) {
        validateFootprint(obstacle);
    }
    double remaining = elapsedSeconds;
    while (remaining > 1.0e-9) {
        const double currentSignalClockSeconds = signalClockSeconds + elapsedSeconds - remaining;
        const double step =
            std::min({remaining, 0.05, 0.1 / std::max(actor.speedMetersPerSecond, 0.1)});
        MapTrafficActor candidate = actor;
        const std::optional<TrafficSignalApproach> signalApproach =
            nextTrafficSignalApproach(actor);
        const std::optional<int> signalState =
            signalApproach.has_value()
                ? trafficSignalStateAtTime(*signalApproach->schedule, currentSignalClockSeconds)
                : std::nullopt;
        const bool blockedBySignal =
            signalApproach.has_value() &&
            (!signalState.has_value() || *signalState < 6 || *signalState > 8);
        candidate.speedMetersPerSecond = actor.cruiseSpeedMetersPerSecond;
        const double requestedTravel = candidate.speedMetersPerSecond * step;
        const bool reachesSignalStop =
            blockedBySignal && signalApproach->distanceToStopMeters <= requestedTravel + 1.0e-9;
        if (reachesSignalStop) {
            candidate.distanceMeters = actor.distanceMeters + signalApproach->distanceToStopMeters;
            if (candidate.route.closed) {
                candidate.distanceMeters =
                    std::fmod(candidate.distanceMeters, candidate.route.totalLength);
            } else if (candidate.route.loopStartDistance &&
                       candidate.distanceMeters >= candidate.route.totalLength) {
                const double loopStart = *candidate.route.loopStartDistance;
                const double loopLength = candidate.route.totalLength - loopStart;
                candidate.distanceMeters =
                    loopStart + std::fmod(candidate.distanceMeters - loopStart, loopLength);
            } else {
                candidate.distanceMeters =
                    std::min(candidate.distanceMeters, candidate.route.totalLength);
            }
            candidate.speedMetersPerSecond = 0.0;
        } else {
            advanceMapTrafficActorLooping(candidate, step);
        }
        const auto pose = sampleMapTrafficRoute(candidate.route, candidate.distanceMeters,
                                                candidate.lateralOffsetMeters);
        if (!pose) {
            throw std::runtime_error("Cannot sample an obstacle-aware traffic actor");
        }
        if (!mapTrafficHasClearance({*pose, lengthMeters, widthMeters}, obstacles, 0.05)) {
            return false;
        }
        actor = std::move(candidate);
        remaining -= step;
    }
    return true;
}

bool canSpawnMapTrafficEntry(const MapTrafficActor& actor, bool entryHidden,
                             const std::vector<MapTrafficObstacle>& obstacles) {
    return entryHidden && mapTrafficHasClearance(actorFootprint(actor, 0.0), obstacles, 2.0);
}

bool mapTrafficRouteEndReadyForUnload(const MapTrafficActor& actor) {
    if (!std::isfinite(actor.distanceMeters) || actor.distanceMeters < 0.0 ||
        !std::isfinite(actor.route.totalLength) || actor.route.totalLength < 0.0) {
        throw std::invalid_argument("Traffic route-end state is invalid");
    }
    return !actor.route.closed && !actor.route.loopStartDistance &&
           actor.distanceMeters >= actor.route.totalLength;
}

bool mapTrafficEntryHiddenFromCamera(const std::array<double, 3>& position, double radiusMeters,
                                     const std::array<double, 3>& cameraPosition,
                                     const std::array<double, 16>& viewProjection,
                                     double minimumDistanceMeters) {
    if (!std::isfinite(radiusMeters) || radiusMeters < 0.0 ||
        !std::isfinite(minimumDistanceMeters) || minimumDistanceMeters < 0.0) {
        throw std::invalid_argument("Traffic entry visibility inputs are invalid");
    }
    if (std::hypot(position[0] - cameraPosition[0], position[1] - cameraPosition[1]) <=
        minimumDistanceMeters + radiusMeters) {
        return false;
    }
    const auto row = [&](int index) {
        return std::array<double, 4>{viewProjection[index], viewProjection[index + 4],
                                     viewProjection[index + 8], viewProjection[index + 12]};
    };
    const auto w = row(3);
    for (int axis = 0; axis < 3; ++axis) {
        const auto r = row(axis);
        for (const double sign : {1.0, -1.0}) {
            const std::array<double, 4> plane = {w[0] + sign * r[0], w[1] + sign * r[1],
                                                 w[2] + sign * r[2], w[3] + sign * r[3]};
            const double length = std::hypot(plane[0], plane[1], plane[2]);
            const double distance =
                plane[0] * position[0] + plane[1] * position[1] + plane[2] * position[2] + plane[3];
            if (distance < -radiusMeters * length) {
                return true;
            }
        }
    }
    return false;
}

double mapTrafficCurveSpeedLimit(const MapTrafficRoute& route, double distanceMeters,
                                 double cruiseSpeedMetersPerSecond,
                                 double minimumTurnRadiusMeters) {
    if (!std::isfinite(distanceMeters) || !std::isfinite(cruiseSpeedMetersPerSecond) ||
        cruiseSpeedMetersPerSecond < 0.0 || !std::isfinite(minimumTurnRadiusMeters) ||
        minimumTurnRadiusMeters < 0.0) {
        throw std::invalid_argument("Traffic curve speed inputs are invalid");
    }
    const double curvature = mapTrafficRouteCurvatureAt(route, distanceMeters);
    // Comfortable lateral acceleration, rather than taking all bends at cruise speed.
    constexpr double lateralAcceleration = 1.8;
    constexpr double minimumCurveSpeedMetersPerSecond = 2.0;
    const double curveSpeed =
        curvature < 1.0e-6
            ? cruiseSpeedMetersPerSecond
            : std::max(minimumCurveSpeedMetersPerSecond,
                       std::sqrt(lateralAcceleration / curvature));
    return std::min(cruiseSpeedMetersPerSecond, curveSpeed);
}

void advanceMapTrafficDriving(const std::vector<MapTrafficActor*>& actors,
                              const std::vector<MapTrafficObstacle>& obstacles,
                              double elapsedSeconds, double signalClockSeconds) {
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0 ||
        !std::isfinite(signalClockSeconds) || signalClockSeconds < 0.0) {
        throw std::invalid_argument("Traffic driving timestep is invalid");
    }
    std::unordered_set<MapTrafficActor*> unique;
    for (auto* actor : actors) {
        if (actor == nullptr || !unique.insert(actor).second ||
            !std::isfinite(actor->distanceMeters) || actor->distanceMeters < 0.0 ||
            !std::isfinite(actor->speedMetersPerSecond) || actor->speedMetersPerSecond < 0.0 ||
            !std::isfinite(actor->targetSpeedMetersPerSecond) ||
            actor->targetSpeedMetersPerSecond < 0.0 ||
            !std::isfinite(actor->cruiseSpeedMetersPerSecond) ||
            actor->cruiseSpeedMetersPerSecond < 0.0 ||
            !std::isfinite(actor->minimumTurnRadiusMeters) ||
            actor->minimumTurnRadiusMeters < 0.0 || !std::isfinite(actor->junctionWaitSeconds) ||
            actor->junctionWaitSeconds < 0.0) {
            throw std::invalid_argument("Traffic driving actor state is invalid");
        }
        validateFootprint(actorFootprint(*actor, actor->distanceMeters));
    }
    for (const auto& obstacle : obstacles) {
        validateFootprint(obstacle);
    }
    constexpr double acceleration = 1.6;
    constexpr double braking = 3.0;
    constexpr double headwaySeconds = 1.5;
    constexpr double signalApproachDeceleration = 1.6;
    constexpr double followingResponseSeconds = 2.5;
    constexpr double standstillFollowingGapMeters = 2.0;
    constexpr double parkedVehicleFollowingGapMeters = 1.0;
    constexpr double movingLeaderSpeedMetersPerSecond = 1.0;
    // A short margin beyond the vehicles' predicted physical occupancy at a conflict.
    constexpr double priorityGapSeconds = 0.75;
    struct MovementReservation {
        MapTrafficObstacle footprint;
        const MapTrafficActor* actor = nullptr;
        double distanceMeters = 0.0;
        double arrivalSeconds = 0.0;
    };
    const auto sameDirectedSegment = [](const MapTrafficRouteSegment& first,
                                        const MapTrafficRouteSegment& second) {
        if (first.reversed != second.reversed ||
            first.pathLateralMeters != second.pathLateralMeters ||
            first.pathHeightMeters != second.pathHeightMeters) {
            return false;
        }
        if (first.points.empty() && second.points.empty()) {
            return first.spline.placement.splineId != 0 &&
                   first.spline.placement.splineId == second.spline.placement.splineId;
        }
        return first.points == second.points;
    };
    const auto sameDirectedRoute = [&](const MapTrafficActor& first,
                                       const MapTrafficActor& second) {
        if (first.lateralOffsetMeters != second.lateralOffsetMeters ||
            first.route.closed != second.route.closed ||
            first.route.loopStartDistance != second.route.loopStartDistance ||
            first.route.segments.size() != second.route.segments.size()) {
            return false;
        }
        for (std::size_t index = 0; index < first.route.segments.size(); ++index) {
            if (!sameDirectedSegment(first.route.segments[index], second.route.segments[index])) {
                return false;
            }
        }
        return true;
    };
    const auto isBehindOnRoute = [](const MapTrafficActor& candidate,
                                    const MapTrafficActor& other) {
        double cycleStart = 0.0;
        double cycleLength = candidate.route.totalLength;
        if (candidate.route.closed) {
            cycleStart = 0.0;
        } else if (candidate.route.loopStartDistance.has_value() &&
            candidate.distanceMeters >= *candidate.route.loopStartDistance &&
            other.distanceMeters >= *candidate.route.loopStartDistance) {
            cycleStart = *candidate.route.loopStartDistance;
            cycleLength -= cycleStart;
        } else {
            return other.distanceMeters < candidate.distanceMeters - 1.0e-6;
        }
        double gap = std::fmod(candidate.distanceMeters - other.distanceMeters, cycleLength);
        if (gap < 0.0) {
            gap += cycleLength;
        }
        return gap > 1.0e-6 && gap <= cycleLength * 0.5;
    };
    // Last segment of the junction group starting at segmentIndex. Junction objects linked by a
    // connector too short to hold this vehicle are one group, so nobody stops across the link.
    const auto junctionGroupEndIndex = [](const MapTrafficActor& candidate,
                                          std::size_t segmentIndex) {
        const auto& segments = candidate.route.segments;
        const auto& ends = candidate.route.cumulativeEndDistances;
        std::size_t endIndex = segmentIndex;
        while (true) {
            if (endIndex + 1 < segments.size() && !segments[endIndex + 1].points.empty()) {
                ++endIndex;
            } else if (endIndex + 2 < segments.size() && segments[endIndex + 1].points.empty() &&
                       !segments[endIndex + 2].points.empty() &&
                       ends[endIndex + 1] - ends[endIndex] < candidate.lengthMeters + 4.0) {
                endIndex += 2;
            } else {
                return endIndex;
            }
        }
    };
    const auto junctionClaimEnd = [&](const MapTrafficActor& candidate, std::size_t endIndex) {
        return candidate.route.cumulativeEndDistances[endIndex] + candidate.lengthMeters * 0.5 +
               2.0;
    };
    // True when other is queued behind candidate in candidate's lane, even on a different route:
    // other's current segment is one candidate has already driven, at an earlier position.
    const auto queuedBehindInLane = [&](const MapTrafficActor& candidate,
                                        const MapTrafficActor& other) {
        if (candidate.lateralOffsetMeters != other.lateralOffsetMeters) {
            return false;
        }
        const auto segmentStart = [](const MapTrafficRoute& route, std::size_t index) {
            return index == 0 ? 0.0 : route.cumulativeEndDistances[index - 1];
        };
        const MapTrafficRouteSegment& otherSegment =
            routeSegmentAtDistance(other.route, other.distanceMeters);
        const auto otherIndex =
            static_cast<std::size_t>(&otherSegment - other.route.segments.data());
        const double otherOffset = other.distanceMeters - segmentStart(other.route, otherIndex);
        const auto candidateIndex = static_cast<std::size_t>(
            &routeSegmentAtDistance(candidate.route, candidate.distanceMeters) -
            candidate.route.segments.data());
        for (std::size_t back = 0; back <= candidateIndex; ++back) {
            const std::size_t index = candidateIndex - back;
            if (candidate.distanceMeters - segmentStart(candidate.route, index) > 100.0) {
                break;
            }
            if (sameDirectedSegment(candidate.route.segments[index], otherSegment)) {
                return segmentStart(candidate.route, index) + otherOffset <
                       candidate.distanceMeters - 1.0e-6;
            }
        }
        return false;
    };
    const auto currentSegmentIndex = [](const MapTrafficActor& candidate) {
        return static_cast<std::size_t>(
            &routeSegmentAtDistance(candidate.route, candidate.distanceMeters) -
            candidate.route.segments.data());
    };
    // Authored priority of the lane that leads into the vehicle's next junction, so the
    // priority of the road a vehicle arrives on decides who gives way, not the junction path.
    const auto approachPriority = [&](const MapTrafficActor& candidate) {
        const auto& segments = candidate.route.segments;
        const MapTrafficVehicleFilter vehicle{
            candidate.vehicleType, candidate.unscheduledGroupIndex, candidate.scheduled};
        const auto explicitPriority = [&](std::size_t index) -> std::optional<double> {
            const MapTrafficPathRules& rules = segments[index].pathRules;
            if (vehicle.unscheduledGroupIndex.has_value()) {
                const auto group = std::find_if(
                    rules.groups.begin(), rules.groups.end(),
                    [&](const MapTrafficGroupRules& candidateGroup) {
                        return candidateGroup.groupIndex == *vehicle.unscheduledGroupIndex;
                    });
                if (group != rules.groups.end() && group->priority.has_value()) {
                    return group->priority;
                }
            }
            return std::nullopt;
        };
        const std::size_t current = currentSegmentIndex(candidate);
        if (const auto priority = explicitPriority(current)) {
            return *priority;
        }
        for (std::size_t back = 1; back <= std::min<std::size_t>(current, 3); ++back) {
            const std::size_t index = current - back;
            if (!segments[index].points.empty()) {
                break;
            }
            if (const auto priority = explicitPriority(index)) {
                return *priority;
            }
        }
        for (std::size_t index = current + 1; index < segments.size(); ++index) {
            if (!segments[index].points.empty() ||
                candidate.route.cumulativeEndDistances[index - 1] -
                        candidate.distanceMeters >
                    150.0) {
                break;
            }
            if (const auto priority = explicitPriority(index)) {
                return *priority;
            }
        }
        return mapTrafficPathPriorityForVehicle(segments[current].pathRules, vehicle);
    };
    // Seconds to cover a distance accelerating towards cruise speed from the current speed.
    const auto travelSeconds = [&](double distance, double speed, double cruise) {
        distance = std::max(0.0, distance);
        speed = std::max(0.0, speed);
        cruise = std::max(cruise, 0.1);
        if (speed > cruise) {
            const double brakingDistance = (speed * speed - cruise * cruise) / (2.0 * braking);
            if (distance <= brakingDistance) {
                const double endingSpeed =
                    std::sqrt(std::max(0.0, speed * speed - 2.0 * braking * distance));
                return (speed - endingSpeed) / braking;
            }
            return (speed - cruise) / braking + (distance - brakingDistance) / cruise;
        }
        const double accelerationDistance =
            (cruise * cruise - speed * speed) / (2.0 * acceleration);
        if (distance <= accelerationDistance) {
            return (std::sqrt(speed * speed + 2.0 * acceleration * distance) - speed) /
                   acceleration;
        }
        return (cruise - speed) / acceleration + (distance - accelerationDistance) / cruise;
    };
    const auto actorArrivalSeconds = [&](const MapTrafficActor& candidate, double distance) {
        const double travelDistance = std::max(0.0, distance - candidate.distanceMeters);
        const double curveSpeed = mapTrafficCurveSpeedLimit(
            candidate.route, distance, candidate.cruiseSpeedMetersPerSecond,
            candidate.minimumTurnRadiusMeters);
        return travelSeconds(travelDistance, candidate.speedMetersPerSecond,
                             std::min(candidate.cruiseSpeedMetersPerSecond, curveSpeed));
    };
    // The next junction group on an open route: its entry distance and claim end.
    const auto nextJunctionRange =
        [&](const MapTrafficActor& candidate) -> std::optional<std::pair<double, double>> {
        if (candidate.route.closed || candidate.route.loopStartDistance) {
            return std::nullopt;
        }
        for (std::size_t segmentIndex = 0; segmentIndex < candidate.route.segments.size();
             ++segmentIndex) {
            if (candidate.route.segments[segmentIndex].points.empty() ||
                candidate.route.cumulativeEndDistances[segmentIndex] <
                    candidate.distanceMeters) {
                continue;
            }
            const double segmentStart =
                segmentIndex == 0 ? 0.0
                                  : candidate.route.cumulativeEndDistances[segmentIndex - 1];
            return std::pair{segmentStart,
                             junctionClaimEnd(candidate,
                                              junctionGroupEndIndex(candidate, segmentIndex))};
        }
        return std::nullopt;
    };
    // Remaining junction path of an actor that is inside, or admitted into, a junction.
    const auto activeJunctionClaimRange =
        [&](const MapTrafficActor& candidate) -> std::optional<std::pair<double, double>> {
        if (mapTrafficActorOnJunctionPath(candidate, candidate.distanceMeters)) {
            const auto& ends = candidate.route.cumulativeEndDistances;
            std::size_t index = static_cast<std::size_t>(
                std::lower_bound(ends.begin(), ends.end(), candidate.distanceMeters) -
                ends.begin());
            index = std::min(index, ends.size() - 1);
            if (candidate.route.segments[index].points.empty() && index + 1 < ends.size()) {
                ++index;
            }
            double end = junctionClaimEnd(candidate, junctionGroupEndIndex(candidate, index));
            if (candidate.junctionClaimEndMeters.has_value()) {
                end = std::max(end, *candidate.junctionClaimEndMeters);
            }
            return std::pair{candidate.distanceMeters, end};
        }
        if (candidate.junctionClaimEndMeters.has_value() &&
            candidate.distanceMeters < *candidate.junctionClaimEndMeters) {
            return std::pair{candidate.distanceMeters, *candidate.junctionClaimEndMeters};
        }
        return std::nullopt;
    };
    struct JunctionClaim {
        const MapTrafficActor* actor = nullptr;
        std::size_t actorIndex = 0;
        std::vector<double> distances;
        std::vector<MapTrafficObstacle> footprints;
        std::vector<const MapTrafficRouteSegment*> segments;
        std::array<double, 4> bounds = {}; // minX, minY, maxX, maxY including body extents
    };
    const auto makeJunctionClaim = [&](const MapTrafficActor& candidate, double from, double to) {
        JunctionClaim claim;
        claim.actor = &candidate;
        if (!candidate.route.closed) {
            to = std::min(to, candidate.route.totalLength);
        }
        from = std::min(from, to);
        claim.bounds = {std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::infinity(),
                        -std::numeric_limits<double>::infinity(),
                        -std::numeric_limits<double>::infinity()};
        const double radius = std::hypot(candidate.lengthMeters, candidate.widthMeters) * 0.5;
        for (double distance = from;; distance = std::min(distance + 1.0, to)) {
            claim.footprints.push_back(actorFootprint(candidate, distance));
            claim.distances.push_back(distance);
            claim.segments.push_back(&routeSegmentAtDistance(candidate.route, distance));
            const auto& position = claim.footprints.back().pose.position;
            claim.bounds[0] = std::min(claim.bounds[0], position[0] - radius);
            claim.bounds[1] = std::min(claim.bounds[1], position[1] - radius);
            claim.bounds[2] = std::max(claim.bounds[2], position[0] + radius);
            claim.bounds[3] = std::max(claim.bounds[3], position[1] + radius);
            if (distance >= to) {
                break;
            }
        }
        return claim;
    };
    for (double remaining = elapsedSeconds; remaining > 1.0e-9;) {
        const double step = std::min(remaining, 0.05);
        remaining -= step;
        const double currentSignalClockSeconds = signalClockSeconds + elapsedSeconds - remaining;
        std::vector<MapTrafficObstacle> occupied = obstacles;
        for (const auto* actor : actors) {
            occupied.push_back(actorFootprint(*actor, actor->distanceMeters));
        }
        std::vector<MovementReservation> reservations;
        std::vector<JunctionClaim> junctionClaims;
        for (std::size_t candidateIndex = 0; candidateIndex < actors.size(); ++candidateIndex) {
            const MapTrafficActor& candidate = *actors[candidateIndex];
            if (const auto range = activeJunctionClaimRange(candidate)) {
                junctionClaims.push_back(makeJunctionClaim(candidate, range->first, range->second));
                junctionClaims.back().actorIndex = candidateIndex;
            }
        }
        // True when the paths cross or merge, ignoring parts of second beyond latestDistance.
        const auto claimsConflict =
            [&](const JunctionClaim& first, const JunctionClaim& second,
                double latestDistance = std::numeric_limits<double>::infinity()) {
                if (first.bounds[0] > second.bounds[2] || second.bounds[0] > first.bounds[2] ||
                    first.bounds[1] > second.bounds[3] || second.bounds[1] > first.bounds[3]) {
                    return false;
                }
                for (std::size_t mine = 0; mine < first.footprints.size(); ++mine) {
                    for (std::size_t theirs = 0; theirs < second.footprints.size() &&
                                                 second.distances[theirs] <= latestDistance;
                         ++theirs) {
                        // Sharing the same lane behind the claimant is ordinary following.
                        if (footprintsOverlap(first.footprints[mine], second.footprints[theirs],
                                              0.25) &&
                            !sameDirectedSegment(*first.segments[mine],
                                                 *second.segments[theirs])) {
                            return true;
                        }
                    }
                }
                return false;
            };
        const auto claimsConflictOnArrival =
            [&](const JunctionClaim& first, const MapTrafficActor& firstActor,
                const JunctionClaim& second, const MapTrafficActor& secondActor) {
                if (first.bounds[0] > second.bounds[2] || second.bounds[0] > first.bounds[2] ||
                    first.bounds[1] > second.bounds[3] || second.bounds[1] > first.bounds[3]) {
                    return false;
                }
                for (std::size_t mine = 0; mine < first.footprints.size(); ++mine) {
                    const double firstDistance = first.distances[mine];
                    const double firstTravelDistance =
                        std::max(0.0, firstDistance - firstActor.distanceMeters);
                    const double firstCurveSpeed = mapTrafficCurveSpeedLimit(
                        firstActor.route, firstDistance, firstActor.cruiseSpeedMetersPerSecond,
                        firstActor.minimumTurnRadiusMeters);
                    const double firstTargetSpeed =
                        std::min(firstActor.cruiseSpeedMetersPerSecond, firstCurveSpeed);
                    const double firstArrivalSeconds =
                        travelSeconds(firstTravelDistance, firstActor.speedMetersPerSecond,
                                      firstTargetSpeed);
                    const double firstSpeedAtArrival =
                        std::max(2.0, std::min(firstTargetSpeed,
                                               firstActor.speedMetersPerSecond +
                                                   acceleration * firstArrivalSeconds));
                    const double firstOccupancySeconds =
                        firstActor.lengthMeters * 0.5 / firstSpeedAtArrival;
                    for (std::size_t theirs = 0; theirs < second.footprints.size(); ++theirs) {
                        if (!footprintsOverlap(first.footprints[mine], second.footprints[theirs],
                                               0.25) ||
                            sameDirectedSegment(*first.segments[mine],
                                                *second.segments[theirs])) {
                            continue;
                        }
                        const double secondDistance = second.distances[theirs];
                        const double secondTravelDistance =
                            std::max(0.0, secondDistance - secondActor.distanceMeters);
                        const double secondCurveSpeed = mapTrafficCurveSpeedLimit(
                            secondActor.route, secondDistance,
                            secondActor.cruiseSpeedMetersPerSecond,
                            secondActor.minimumTurnRadiusMeters);
                        const double secondTargetSpeed =
                            std::min(secondActor.cruiseSpeedMetersPerSecond, secondCurveSpeed);
                        const double secondArrivalSeconds =
                            travelSeconds(secondTravelDistance,
                                          secondActor.speedMetersPerSecond, secondTargetSpeed);
                        const double secondSpeedAtArrival =
                            std::max(2.0, std::min(secondTargetSpeed,
                                                   secondActor.speedMetersPerSecond +
                                                       acceleration * secondArrivalSeconds));
                        const double secondOccupancySeconds =
                            secondActor.lengthMeters * 0.5 / secondSpeedAtArrival;
                        const double safeArrivalGap =
                            priorityGapSeconds + firstOccupancySeconds + secondOccupancySeconds;
                        if (std::abs(firstArrivalSeconds - secondArrivalSeconds) <=
                            safeArrivalGap) {
                            return true;
                        }
                    }
                }
                return false;
            };
        std::vector<double> priorities(actors.size());
        for (std::size_t candidateIndex = 0; candidateIndex < actors.size(); ++candidateIndex) {
            priorities[candidateIndex] = approachPriority(*actors[candidateIndex]);
        }
        // Authored right of way: a vehicle approaching a junction that can still stop at its
        // entry line gives way to every higher-priority vehicle whose path crosses or merges
        // into its junction movement before it could clear. Computed once per step so the
        // yielding vehicle and the priority vehicle agree on who goes first.
        std::vector<char> canStillYield(actors.size(), 0);
        std::vector<std::vector<std::size_t>> yieldsTo(actors.size());
        std::vector<std::optional<JunctionClaim>> prioritySweeps(actors.size());
        for (std::size_t lowIndex = 0; lowIndex < actors.size(); ++lowIndex) {
            const MapTrafficActor& low = *actors[lowIndex];
            if (mapTrafficActorOnJunctionPath(low, low.distanceMeters)) {
                continue;
            }
            const auto range = nextJunctionRange(low);
            if (!range.has_value()) {
                continue;
            }
            const double entryGap =
                range->first - low.lengthMeters * 0.5 - 0.5 - low.distanceMeters;
            if (entryGap > 80.0 ||
                entryGap < low.speedMetersPerSecond * low.speedMetersPerSecond / (2.0 * braking)) {
                continue;
            }
            canStillYield[lowIndex] = 1;
            std::optional<JunctionClaim> lowClaim;
            for (std::size_t highIndex = 0; highIndex < actors.size(); ++highIndex) {
                const MapTrafficActor& high = *actors[highIndex];
                const bool highHeld =
                    high.blockReason == MapTrafficBlockReason::Signal ||
                    (high.speedMetersPerSecond < 0.3 &&
                     (high.blockReason == MapTrafficBlockReason::Leader ||
                      high.blockReason == MapTrafficBlockReason::Vehicle ||
                      high.blockReason == MapTrafficBlockReason::Obstacle ||
                      high.blockReason == MapTrafficBlockReason::Collision));
                if (highIndex == lowIndex || priorities[highIndex] >= priorities[lowIndex] ||
                    highHeld || mapTrafficActorOnJunctionPath(high, high.distanceMeters) ||
                    sameDirectedRoute(low, high) || queuedBehindInLane(low, high) ||
                    queuedBehindInLane(high, low)) {
                    continue;
                }
                const auto& lowPosition = occupied[obstacles.size() + lowIndex].pose.position;
                const auto& highPosition = occupied[obstacles.size() + highIndex].pose.position;
                if (std::hypot(highPosition[0] - lowPosition[0],
                               highPosition[1] - lowPosition[1]) > 250.0) {
                    continue;
                }
                if (!lowClaim.has_value()) {
                    lowClaim = makeJunctionClaim(
                        low, std::max(low.distanceMeters, range->first - low.lengthMeters * 0.5),
                        range->second);
                }
                if (!prioritySweeps[highIndex].has_value()) {
                    prioritySweeps[highIndex] =
                        makeJunctionClaim(high, high.distanceMeters, high.distanceMeters + 200.0);
                }
                if (claimsConflictOnArrival(*lowClaim, low, *prioritySweeps[highIndex], high)) {
                    yieldsTo[lowIndex].push_back(highIndex);
                }
            }
        }
        const auto yieldsForPriority = [&](std::size_t lowIndex, std::size_t highIndex) {
            return canStillYield[lowIndex] &&
                   std::ranges::find(yieldsTo[lowIndex], highIndex) != yieldsTo[lowIndex].end();
        };
        std::vector<std::size_t> priorityOrder(actors.size());
        std::iota(priorityOrder.begin(), priorityOrder.end(), 0);
        std::stable_sort(
            priorityOrder.begin(), priorityOrder.end(), [&](std::size_t first, std::size_t second) {
                const auto isCommittedToJunction = [](const MapTrafficActor& actor) {
                    return mapTrafficActorOnJunctionPath(actor, actor.distanceMeters);
                };
                const bool firstOnJunction = isCommittedToJunction(*actors[first]);
                const bool secondOnJunction = isCommittedToJunction(*actors[second]);
                if (firstOnJunction != secondOnJunction) {
                    return firstOnJunction;
                }
                if (firstOnJunction &&
                    actors[first]->junctionWaitSeconds != actors[second]->junctionWaitSeconds) {
                    return actors[first]->junctionWaitSeconds > actors[second]->junctionWaitSeconds;
                }
                // OMSI assigns higher priority to lower values; waiting time only orders ties.
                if (priorities[first] != priorities[second]) {
                    return priorities[first] < priorities[second];
                }
                return actors[first]->junctionWaitSeconds > actors[second]->junctionWaitSeconds;
            });
        for (const std::size_t index : priorityOrder) {
            auto& actor = *actors[index];
            const std::size_t own = obstacles.size() + index;
            bool blockedByCrossingReservation = false;
            bool blockedByJunctionTraffic = false;
            const auto onJunctionPath = [](const MapTrafficActor& candidate, double distance) {
                return mapTrafficActorOnJunctionPath(candidate, distance);
            };
            const bool committedToJunction = onJunctionPath(actor, actor.distanceMeters);
            bool junctionInLookahead = committedToJunction;
            std::optional<std::pair<double, double>> upcomingJunctionRange;
            if (!committedToJunction) {
                upcomingJunctionRange = nextJunctionRange(actor);
            }
            const MapTrafficActor* priorityTraffic =
                canStillYield[index] && !yieldsTo[index].empty() ? actors[yieldsTo[index].front()]
                                                                 : nullptr;
            MapTrafficBlockReason clearBlockReason = MapTrafficBlockReason::None;
            // In-lane leaders that are still moving. The following model already controls the gap
            // to them, so the lookahead must not treat them as parked obstacles.
            std::vector<const MapTrafficActor*> movingLaneLeaders;
            std::vector<bool> followingObstacleLookahead(obstacles.size(), false);
            const MapTrafficActor* clearBlockingActor = nullptr;
            std::optional<std::size_t> clearBlockingObstacle;
            const auto clearAt = [&](double distance, double clearance,
                                     bool allowFollowingVehicleLookahead) {
                auto footprint = actorFootprint(actor, distance);
                if (clearance > 0.5) {
                    // Headway is ahead of the nose, not behind the tail of the leader.
                    const double extension = clearance - 0.25;
                    footprint.lengthMeters += extension;
                    footprint.pose.position[0] +=
                        std::sin(footprint.pose.headingRadians) * extension * 0.5;
                    footprint.pose.position[1] +=
                        std::cos(footprint.pose.headingRadians) * extension * 0.5;
                    clearance = 0.25;
                }
                for (std::size_t other = 0; other < occupied.size(); ++other) {
                    if (other != own && footprintsOverlap(footprint, occupied[other], clearance)) {
                        if (allowFollowingVehicleLookahead && other < obstacles.size() &&
                            followingObstacleLookahead[other]) {
                            continue;
                        }
                        clearBlockReason = MapTrafficBlockReason::Obstacle;
                        clearBlockingActor = nullptr;
                        clearBlockingObstacle.reset();
                        if (other < obstacles.size()) {
                            clearBlockingObstacle = other;
                        }
                        if (other >= obstacles.size()) {
                            const auto& otherActor = *actors[other - obstacles.size()];
                            if (allowFollowingVehicleLookahead &&
                                ((sameDirectedRoute(actor, otherActor) &&
                                  isBehindOnRoute(actor, otherActor)) ||
                                 std::ranges::find(movingLaneLeaders, &otherActor) !=
                                     movingLaneLeaders.end())) {
                                continue;
                            }
                            clearBlockingActor = &otherActor;
                            const bool crossing =
                                std::abs(std::cos(footprint.pose.headingRadians -
                                                  occupied[other].pose.headingRadians)) < 0.7;
                            const bool otherOnJunction =
                                onJunctionPath(otherActor, otherActor.distanceMeters);
                            if ((onJunctionPath(actor, distance) && crossing) || otherOnJunction) {
                                blockedByJunctionTraffic = true;
                                clearBlockReason = MapTrafficBlockReason::JunctionTraffic;
                            } else {
                                clearBlockReason = MapTrafficBlockReason::Vehicle;
                            }
                        }
                        return false;
                    }
                }
                for (const auto& reservation : reservations) {
                    const MapTrafficRouteSegment& candidateSegment =
                        routeSegmentAtDistance(actor.route, distance);
                    const MapTrafficRouteSegment& reservedSegment =
                        routeSegmentAtDistance(reservation.actor->route,
                                               reservation.distanceMeters);
                    const bool samePathGeometry =
                        candidateSegment.points.empty() && reservedSegment.points.empty()
                            ? candidateSegment.spline.placement.splineId != 0 &&
                                  candidateSegment.spline.placement.splineId ==
                                      reservedSegment.spline.placement.splineId
                            : candidateSegment.points == reservedSegment.points;
                    const bool sameDirectedPath =
                        candidateSegment.reversed == reservedSegment.reversed &&
                        candidateSegment.pathLateralMeters == reservedSegment.pathLateralMeters &&
                        candidateSegment.pathHeightMeters == reservedSegment.pathHeightMeters &&
                        actor.lateralOffsetMeters == reservation.actor->lateralOffsetMeters &&
                        samePathGeometry;
                    if (sameDirectedPath) {
                        continue;
                    }
                    if (sameDirectedRoute(actor, *reservation.actor)) {
                        continue;
                    }
                    // Following is handled by current footprints; reserve junction crossings.
                    if (std::abs(std::cos(footprint.pose.headingRadians -
                                          reservation.footprint.pose.headingRadians)) < 0.7 &&
                        footprintsOverlap(footprint, reservation.footprint, clearance)) {
                        const double candidateArrivalSeconds =
                            actorArrivalSeconds(actor, distance);
                        const double candidateTargetSpeed = std::min(
                            actor.cruiseSpeedMetersPerSecond,
                            mapTrafficCurveSpeedLimit(actor.route, distance,
                                                      actor.cruiseSpeedMetersPerSecond,
                                                      actor.minimumTurnRadiusMeters));
                        const double candidateArrivalSpeed = std::max(
                            2.0, std::min(candidateTargetSpeed,
                                          actor.speedMetersPerSecond +
                                              acceleration * candidateArrivalSeconds));
                        const double reservationArrivalSpeed = std::max(
                            2.0, std::min(reservation.actor->cruiseSpeedMetersPerSecond,
                                          reservation.actor->speedMetersPerSecond +
                                              acceleration * reservation.arrivalSeconds));
                        const double safeArrivalGap =
                            0.25 + actor.lengthMeters * 0.5 / candidateArrivalSpeed +
                            reservation.actor->lengthMeters * 0.5 / reservationArrivalSpeed;
                        if (std::abs(candidateArrivalSeconds - reservation.arrivalSeconds) <=
                            safeArrivalGap) {
                            blockedByCrossingReservation = true;
                            clearBlockReason = MapTrafficBlockReason::CrossingReservation;
                            clearBlockingActor = reservation.actor;
                            clearBlockingObstacle.reset();
                            return false;
                        }
                    }
                }
                return true;
            };
            double target = actor.cruiseSpeedMetersPerSecond;
            MapTrafficBlockReason targetReason = MapTrafficBlockReason::None;
            const MapTrafficActor* targetBlockingActor = nullptr;
            std::optional<std::size_t> targetBlockingObstacle;
            const auto limitTarget = [&](double limit, MapTrafficBlockReason reason,
                                         const MapTrafficActor* blockingActor = nullptr,
                                         std::optional<std::size_t> blockingObstacle = {}) {
                if (limit < target) {
                    target = limit;
                    targetReason = reason;
                    targetBlockingActor = blockingActor;
                    targetBlockingObstacle = blockingObstacle;
                }
            };
            double nearestFollowingGapMeters = std::numeric_limits<double>::infinity();
            double nearestFollowingSpeedMetersPerSecond = 0.0;
            const MapTrafficActor* nearestFollowingActor = nullptr;
            const MapTrafficObstacle& actorFootprintNow = occupied[own];
            const double actorHeading = actorFootprintNow.pose.headingRadians;
            const bool openRoute = !actor.route.closed && !actor.route.loopStartDistance;
            // Beyond this gap even a stopped leader cannot lower the following target.
            const double followingRangeMeters =
                standstillFollowingGapMeters + actor.speedMetersPerSecond * headwaySeconds +
                actor.speedMetersPerSecond * actor.speedMetersPerSecond / (2.0 * braking) +
                actor.cruiseSpeedMetersPerSecond * followingResponseSeconds + 5.0;
            for (std::size_t otherIndex = 0; otherIndex < actors.size(); ++otherIndex) {
                if (otherIndex == index) {
                    continue;
                }
                const auto& otherActor = *actors[otherIndex];
                const MapTrafficObstacle& otherFootprint =
                    occupied[obstacles.size() + otherIndex];
                const double halfLengths = (actor.lengthMeters + otherActor.lengthMeters) * 0.5;
                double alongRouteMeters = 0.0;
                if (sameDirectedRoute(actor, otherActor)) {
                    if (!isBehindOnRoute(otherActor, actor)) {
                        continue;
                    }
                    alongRouteMeters = otherActor.distanceMeters - actor.distanceMeters;
                    if (alongRouteMeters < 0.0) {
                        const double cycleStart = actor.route.loopStartDistance.value_or(0.0);
                        alongRouteMeters += actor.route.totalLength - cycleStart;
                    }
                } else {
                    // A leader must sit on this actor's own path, not merely straight ahead:
                    // project it onto the route so curves, adjacent lanes, side roads, and
                    // over/underpasses are not mistaken for the vehicle in front.
                    const double deltaX =
                        otherFootprint.pose.position[0] - actorFootprintNow.pose.position[0];
                    const double deltaY =
                        otherFootprint.pose.position[1] - actorFootprintNow.pose.position[1];
                    if (std::hypot(deltaX, deltaY) - halfLengths > followingRangeMeters ||
                        std::cos(actorHeading - otherFootprint.pose.headingRadians) < 0.0) {
                        continue;
                    }
                    alongRouteMeters =
                        deltaX * std::sin(actorHeading) + deltaY * std::cos(actorHeading);
                    if (alongRouteMeters <= 0.0) {
                        continue;
                    }
                    std::optional<MapTrafficPose> pathPose;
                    for (int refinement = 0; refinement < 3 && alongRouteMeters > 0.0;
                         ++refinement) {
                        const double pathDistance = actor.distanceMeters + alongRouteMeters;
                        if (openRoute && pathDistance > actor.route.totalLength) {
                            pathPose.reset();
                            break;
                        }
                        pathPose = sampleMapTrafficRoute(actor.route, pathDistance,
                                                         actor.lateralOffsetMeters);
                        if (!pathPose) {
                            break;
                        }
                        alongRouteMeters +=
                            (otherFootprint.pose.position[0] - pathPose->position[0]) *
                                std::sin(pathPose->headingRadians) +
                            (otherFootprint.pose.position[1] - pathPose->position[1]) *
                                std::cos(pathPose->headingRadians);
                    }
                    if (!pathPose || alongRouteMeters <= 0.0 ||
                        std::cos(pathPose->headingRadians - otherFootprint.pose.headingRadians) <
                            0.7 ||
                        std::abs(otherFootprint.pose.position[2] - pathPose->position[2]) > 3.0) {
                        continue;
                    }
                    const double pathLateralOffset = std::abs(
                        (otherFootprint.pose.position[0] - pathPose->position[0]) *
                            std::cos(pathPose->headingRadians) -
                        (otherFootprint.pose.position[1] - pathPose->position[1]) *
                            std::sin(pathPose->headingRadians));
                    // Bodies must overlap laterally by a margin to share this lane.
                    if (pathLateralOffset >
                        (actor.widthMeters + otherActor.widthMeters) * 0.5 - 0.3) {
                        continue;
                    }
                }
                const double longitudinalGap = alongRouteMeters - halfLengths;
                if (longitudinalGap >= 0.0 &&
                    otherActor.speedMetersPerSecond > movingLeaderSpeedMetersPerSecond) {
                    movingLaneLeaders.push_back(&otherActor);
                }
                if (longitudinalGap < 0.0 || longitudinalGap >= nearestFollowingGapMeters) {
                    continue;
                }
                nearestFollowingGapMeters = longitudinalGap;
                nearestFollowingSpeedMetersPerSecond = otherActor.speedMetersPerSecond;
                nearestFollowingActor = &otherActor;
            }
            for (std::size_t obstacleIndex = 0; obstacleIndex < obstacles.size();
                 ++obstacleIndex) {
                const MapTrafficObstacle& obstacle = obstacles[obstacleIndex];
                const double halfLengths = (actor.lengthMeters + obstacle.lengthMeters) * 0.5;
                const double deltaX =
                    obstacle.pose.position[0] - actorFootprintNow.pose.position[0];
                const double deltaY =
                    obstacle.pose.position[1] - actorFootprintNow.pose.position[1];
                if (std::hypot(deltaX, deltaY) - halfLengths > followingRangeMeters ||
                    std::cos(actorHeading - obstacle.pose.headingRadians) < 0.0) {
                    continue;
                }
                double alongRouteMeters =
                    deltaX * std::sin(actorHeading) + deltaY * std::cos(actorHeading);
                if (alongRouteMeters <= 0.0) {
                    continue;
                }
                std::optional<MapTrafficPose> pathPose;
                for (int refinement = 0; refinement < 3 && alongRouteMeters > 0.0;
                     ++refinement) {
                    const double pathDistance = actor.distanceMeters + alongRouteMeters;
                    if (openRoute && pathDistance > actor.route.totalLength) {
                        pathPose.reset();
                        break;
                    }
                    pathPose = sampleMapTrafficRoute(actor.route, pathDistance,
                                                     actor.lateralOffsetMeters);
                    if (!pathPose) {
                        break;
                    }
                    alongRouteMeters +=
                        (obstacle.pose.position[0] - pathPose->position[0]) *
                            std::sin(pathPose->headingRadians) +
                        (obstacle.pose.position[1] - pathPose->position[1]) *
                            std::cos(pathPose->headingRadians);
                }
                if (!pathPose || alongRouteMeters <= 0.0 ||
                    std::cos(pathPose->headingRadians - obstacle.pose.headingRadians) < 0.7 ||
                    std::abs(obstacle.pose.position[2] - pathPose->position[2]) > 3.0) {
                    continue;
                }
                const double pathLateralOffset = std::abs(
                    (obstacle.pose.position[0] - pathPose->position[0]) *
                        std::cos(pathPose->headingRadians) -
                    (obstacle.pose.position[1] - pathPose->position[1]) *
                        std::sin(pathPose->headingRadians));
                if (pathLateralOffset > (actor.widthMeters + obstacle.widthMeters) * 0.5 - 0.3) {
                    continue;
                }
                const double longitudinalGap = alongRouteMeters - halfLengths;
                if (longitudinalGap < 0.0) {
                    continue;
                }
                const double safeQueueDistance =
                    actor.distanceMeters + longitudinalGap - parkedVehicleFollowingGapMeters;
                if (!committedToJunction && upcomingJunctionRange.has_value() &&
                    safeQueueDistance + 1.0e-6 < upcomingJunctionRange->second) {
                    continue;
                }
                followingObstacleLookahead[obstacleIndex] = true;
                const double availableStoppingDistance =
                    std::max(0.0, longitudinalGap - parkedVehicleFollowingGapMeters);
                limitTarget(std::sqrt(2.0 * braking * availableStoppingDistance),
                            MapTrafficBlockReason::Obstacle, nullptr, obstacleIndex);
            }
            if (std::isfinite(nearestFollowingGapMeters)) {
                const double closingDistance = std::max(
                    0.0, (actor.speedMetersPerSecond * actor.speedMetersPerSecond -
                          nearestFollowingSpeedMetersPerSecond *
                              nearestFollowingSpeedMetersPerSecond) /
                             (2.0 * braking));
                const double desiredFollowingGap =
                    standstillFollowingGapMeters + actor.speedMetersPerSecond * headwaySeconds +
                    closingDistance;
                const double followingSpeed =
                    std::max(0.0, nearestFollowingSpeedMetersPerSecond +
                                      (nearestFollowingGapMeters - desiredFollowingGap) /
                                          followingResponseSeconds);
                limitTarget(followingSpeed, MapTrafficBlockReason::Leader, nearestFollowingActor);
            }
            double signalStopDistance = std::numeric_limits<double>::infinity();
            const std::optional<TrafficSignalApproach> signalApproach =
                nextTrafficSignalApproach(actor);
            if (signalApproach.has_value()) {
                const std::optional<int> signalState =
                    trafficSignalStateAtTime(*signalApproach->schedule, currentSignalClockSeconds);
                if (!signalState.has_value() || *signalState < 6 || *signalState > 8) {
                    signalStopDistance = signalApproach->distanceToStopMeters;
                }
            }
            if (std::isfinite(signalStopDistance)) {
                limitTarget(std::sqrt(2.0 * signalApproachDeceleration * signalStopDistance),
                            MapTrafficBlockReason::Signal);
            }
            const double lookahead = std::max(
                15.0, actor.speedMetersPerSecond * actor.speedMetersPerSecond / (2.0 * braking) +
                          actor.speedMetersPerSecond * headwaySeconds + actor.lengthMeters);
            const double junctionEntryStopDistance =
                upcomingJunctionRange.has_value()
                    ? upcomingJunctionRange->first - actor.lengthMeters * 0.5 - 0.5
                    : std::numeric_limits<double>::infinity();
            const double junctionEntryStopGap =
                std::max(0.0, junctionEntryStopDistance - actor.distanceMeters);
            double movementLookahead = lookahead;
            if (upcomingJunctionRange.has_value() && junctionEntryStopGap <= lookahead) {
                movementLookahead =
                    std::max(movementLookahead, upcomingJunctionRange->second - actor.distanceMeters);
            }
            double clearedThroughDistance = -std::numeric_limits<double>::infinity();
            for (double ahead = 0.0; ahead <= movementLookahead; ahead += 1.0) {
                if (ahead > signalStopDistance) {
                    break;
                }
                const double distance = actor.distanceMeters + ahead;
                if (!actor.route.closed && !actor.route.loopStartDistance &&
                    distance > actor.route.totalLength) {
                    break;
                }
                const bool junctionAtDistance = onJunctionPath(actor, distance);
                junctionInLookahead = junctionInLookahead || junctionAtDistance;
                const double curveLimit = mapTrafficCurveSpeedLimit(
                    actor.route, distance, actor.cruiseSpeedMetersPerSecond,
                    actor.minimumTurnRadiusMeters);
                limitTarget(std::sqrt(curveLimit * curveLimit +
                                      2.0 * braking * std::max(0.0, ahead - 2.0)),
                            MapTrafficBlockReason::None);
                const std::optional<double> speedLimit = mapTrafficPathSpeedLimitForVehicle(
                    routeSegmentAtDistance(actor.route, distance).pathRules,
                    {actor.vehicleType, actor.unscheduledGroupIndex, actor.scheduled});
                if (speedLimit.has_value()) {
                    const double limitMetersPerSecond = *speedLimit / 3.6;
                    limitTarget(std::sqrt(limitMetersPerSecond * limitMetersPerSecond +
                                          2.0 * braking * std::max(0.0, ahead - 2.0)),
                                MapTrafficBlockReason::None);
                }
                const double clearance = committedToJunction && junctionAtDistance ? 0.5 : 2.0;
                if (!clearAt(distance, clearance, true)) {
                    const double approachBuffer = clearance <= 0.5 ? 0.0 : 1.0;
                    double availableGapMeters = std::max(0.0, ahead - approachBuffer);
                    if (upcomingJunctionRange.has_value() &&
                        distance >= upcomingJunctionRange->first -
                                        actor.lengthMeters * 0.5 - clearance &&
                        distance <= upcomingJunctionRange->second) {
                        availableGapMeters =
                            std::min(availableGapMeters, junctionEntryStopGap);
                    }
                    // A time headway makes the required following distance grow with speed.
                    limitTarget(std::min(availableGapMeters / headwaySeconds,
                                         std::sqrt(2.0 * braking * availableGapMeters)),
                                clearBlockReason, clearBlockingActor, clearBlockingObstacle);
                    break;
                }
                clearedThroughDistance = distance;
            }
            bool junctionClaimHeld =
                actor.junctionClaimEndMeters.has_value() &&
                actor.distanceMeters < *actor.junctionClaimEndMeters;
            if (!committedToJunction && junctionClaimHeld &&
                (std::isfinite(signalStopDistance) || priorityTraffic != nullptr)) {
                // A red light ahead of the junction, or priority traffic arriving before this
                // vehicle could clear: release the unused claim and stop at the entry line.
                actor.junctionClaimEndMeters.reset();
                junctionClaimHeld = false;
            }
            if (!committedToJunction && !junctionClaimHeld &&
                upcomingJunctionRange.has_value() && junctionEntryStopGap <= lookahead) {
                // Enter only if no actor inside or admitted to a junction will cross this path,
                // so two vehicles never meet head-on or side-on inside the junction box.
                JunctionClaim wanted = makeJunctionClaim(
                    actor,
                    std::max(actor.distanceMeters,
                             upcomingJunctionRange->first - actor.lengthMeters * 0.5),
                    upcomingJunctionRange->second);
                wanted.actorIndex = index;
                const MapTrafficActor* conflictingActor = nullptr;
                for (const JunctionClaim& claim : junctionClaims) {
                    // A lower-priority claimant that can still stop gives way to this vehicle.
                    if (claim.actor == &actor || sameDirectedRoute(actor, *claim.actor) ||
                        queuedBehindInLane(actor, *claim.actor) ||
                        queuedBehindInLane(*claim.actor, actor) ||
                        yieldsForPriority(claim.actorIndex, index)) {
                        continue;
                    }
                    if (claimsConflict(wanted, claim)) {
                        conflictingActor = claim.actor;
                        break;
                    }
                }
                if (conflictingActor == nullptr) {
                    conflictingActor = priorityTraffic;
                }
                if (conflictingActor != nullptr) {
                    blockedByJunctionTraffic = true;
                    limitTarget(std::min(junctionEntryStopGap / headwaySeconds,
                                         std::sqrt(2.0 * braking * junctionEntryStopGap)),
                                MapTrafficBlockReason::JunctionTraffic, conflictingActor);
                } else {
                    const double admissionWindow =
                        actor.speedMetersPerSecond * actor.speedMetersPerSecond /
                            (2.0 * braking) +
                        actor.speedMetersPerSecond * 0.5 + 2.0;
                    const double junctionPathEnd =
                        actor.route.closed
                            ? upcomingJunctionRange->second
                            : std::min(upcomingJunctionRange->second, actor.route.totalLength);
                    // Claims follow queue order: a leader still short of the entry claims first.
                    const bool leaderStillQueuedAhead =
                        nearestFollowingActor != nullptr &&
                        nearestFollowingGapMeters < junctionEntryStopGap &&
                        !activeJunctionClaimRange(*nearestFollowingActor).has_value();
                    if (junctionEntryStopGap <= admissionWindow &&
                        clearedThroughDistance >= junctionPathEnd - 1.0 &&
                        !leaderStillQueuedAhead) {
                        actor.junctionClaimEndMeters = upcomingJunctionRange->second;
                        junctionClaims.push_back(wanted);
                    }
                }
            }
            actor.targetSpeedMetersPerSecond = target;
            const double oldSpeed = actor.speedMetersPerSecond;
            const double newSpeed = target < oldSpeed
                                        ? std::max(target, oldSpeed - braking * step)
                                        : std::min(target, oldSpeed + acceleration * step);
            const double requestedTravel = (oldSpeed + newSpeed) * 0.5 * step;
            const double travel = std::min(requestedTravel, signalStopDistance);
            double admitted = 0.0;
            bool blockedDuringMovement = false;
            // Sweep rather than testing just the final pose, including at low frame rates.
            for (double offset = std::min(0.2, travel); offset <= travel && travel > 0.0;
                 offset = std::min(offset + 0.2, travel)) {
                if (!clearAt(actor.distanceMeters + offset, 0.5, false)) {
                    blockedDuringMovement = true;
                    break;
                }
                admitted = offset;
                if (offset == travel) {
                    break;
                }
            }
            actor.speedMetersPerSecond =
                admitted + 1.0e-9 < travel || travel + 1.0e-9 < requestedTravel ? 0.0 : newSpeed;
            actor.blockReason = blockedDuringMovement
                                    ? clearBlockReason == MapTrafficBlockReason::Obstacle
                                          ? MapTrafficBlockReason::Obstacle
                                          : MapTrafficBlockReason::Collision
                                    : targetReason;
            actor.blockingActor = blockedDuringMovement ? clearBlockingActor : targetBlockingActor;
            actor.blockingObstacleIndex =
                blockedDuringMovement ? clearBlockingObstacle : targetBlockingObstacle;
            const bool inJunctionAfterMovement =
                onJunctionPath(actor, actor.distanceMeters + admitted);
            const bool junctionConstraintSlowedVehicle =
                (actor.blockReason == MapTrafficBlockReason::CrossingReservation ||
                 actor.blockReason == MapTrafficBlockReason::JunctionTraffic) &&
                newSpeed + 1.0e-9 < oldSpeed;
            const bool junctionConstraintStoppedVehicle =
                blockedDuringMovement &&
                (inJunctionAfterMovement ||
                 clearBlockReason == MapTrafficBlockReason::CrossingReservation ||
                 clearBlockReason == MapTrafficBlockReason::JunctionTraffic);
            if (junctionConstraintSlowedVehicle || junctionConstraintStoppedVehicle) {
                actor.junctionWaitSeconds += step;
            } else if (admitted > 1.0e-9 && !inJunctionAfterMovement && !junctionInLookahead) {
                actor.junctionWaitSeconds = 0.0;
            }
            const double distanceBeforeMovement = actor.distanceMeters;
            actor.distanceMeters += admitted;
            if (actor.route.closed) {
                actor.distanceMeters = std::fmod(actor.distanceMeters, actor.route.totalLength);
            } else if (actor.route.loopStartDistance &&
                       actor.distanceMeters >= actor.route.totalLength) {
                const double start = *actor.route.loopStartDistance;
                actor.distanceMeters = start + std::fmod(actor.distanceMeters - start,
                                                         actor.route.totalLength - start);
            } else {
                actor.distanceMeters = std::min(actor.distanceMeters, actor.route.totalLength);
            }
            occupied[own] = actorFootprint(actor, actor.distanceMeters);
            if (actor.distanceMeters < distanceBeforeMovement) {
                actor.junctionClaimEndMeters.reset();
            }
            if (const auto claimRange = activeJunctionClaimRange(actor)) {
                // Hold the claim until the tail has cleared the junction group.
                actor.junctionClaimEndMeters = claimRange->second;
            } else {
                actor.junctionClaimEndMeters.reset();
            }
            if (actor.speedMetersPerSecond > 0.1) {
                for (double ahead = 1.0; ahead < actor.speedMetersPerSecond * 3.0; ahead += 1.0) {
                    const double reservationDistance = actor.distanceMeters + ahead;
                    reservations.push_back(
                        {actorFootprint(actor, reservationDistance), &actor, reservationDistance,
                         step + actorArrivalSeconds(actor, reservationDistance)});
                }
            }
        }
    }
}

} // namespace openbus::map
