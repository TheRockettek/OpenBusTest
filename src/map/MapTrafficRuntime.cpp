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

} // namespace

bool mapTrafficRoutePositionOnJunctionPath(const MapTrafficRoute& route, double distanceMeters) {
    if (route.segments.empty() || route.segments.size() != route.cumulativeEndDistances.size() ||
        !std::isfinite(distanceMeters) || !std::isfinite(route.totalLength) ||
        route.totalLength <= 0.0) {
        throw std::invalid_argument("Traffic junction-path query inputs are invalid");
    }
    return !routeSegmentAtDistance(route, distanceMeters).points.empty();
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
    actor.cruiseSpeedMetersPerSecond = speedMetersPerSecond;
    actor.lateralOffsetMeters =
        mapTrafficLaneOffsetMeters(leftHandTraffic, laneOffsetMagnitudeMeters);
    return actor;
}

void resetMapTrafficActorForEntry(MapTrafficActor& actor) noexcept {
    actor.distanceMeters = 0.0;
    actor.speedMetersPerSecond = 0.0;
    actor.junctionWaitSeconds = 0.0;
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

std::optional<MapTrafficPose> sampleMapTrafficRoute(const MapTrafficRoute& route,
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

    const double segmentDistance = std::clamp(routeDistance - segmentStart, 0.0, spline.length);
    const double splineDistance =
        routeSegment.reversed ? spline.length - segmentDistance : segmentDistance;
    const double tileOriginX =
        static_cast<double>(routeSegment.spline.tileX) * OMSI_TILE_SIZE_METERS;
    const double tileOriginY =
        static_cast<double>(routeSegment.spline.tileY) * OMSI_TILE_SIZE_METERS;
    const auto center =
        sampleMapSpline(tileOriginX + spline.localX, tileOriginY + spline.localY,
                        spline.rotationDegrees, spline.length, spline.radius, splineDistance);
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
    const double progress = splineDistance / spline.length;
    const double cant = spline.cantStart + (spline.cantEnd - spline.cantStart) * progress;
    const double elevation = mapSplineElevation(
        spline.elevation, spline.length, spline.gradientStart, spline.gradientEnd,
        spline.elevated ? std::optional<double>(spline.heightDelta) : std::nullopt, splineDistance);

    return MapTrafficPose{{center->x + offset[0], center->y + offset[1],
                           elevation + mapSplineCantHeightOffset(splineLateral, cant) +
                               routeSegment.pathHeightMeters},
                          std::remainder(heading, 2.0 * std::numbers::pi),
                          spline.splineId};
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
                if (mapTrafficCurveSpeedLimit(join, lengths_[currentLane], 1.0,
                                              minimumTurnRadiusMeters) == 0.0) {
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
    struct WeightedEntry {
        MapTrafficLaneStart entry;
        double density;
    };
    std::vector<WeightedEntry> candidates;
    double maximumDensity = 0.0;
    for (const MapTrafficLaneStart& entry : entryLanes()) {
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
    if (schedule.controller != nullptr && schedule.hasConditionalOverrides) {
        return schedule.controller->state(schedule.lightIndex);
    }
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
    const double curvature = angle / std::max(span, 0.01);
    if (curvature * minimumTurnRadiusMeters > 1.05) {
        return 0.0;
    }
    // Comfortable lateral acceleration, rather than taking all bends at cruise speed.
    constexpr double lateralAcceleration = 1.8;
    return curvature < 1.0e-6
               ? cruiseSpeedMetersPerSecond
               : std::min(cruiseSpeedMetersPerSecond, std::sqrt(lateralAcceleration / curvature));
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
    for (double remaining = elapsedSeconds; remaining > 1.0e-9;) {
        const double step = std::min(remaining, 0.05);
        remaining -= step;
        const double currentSignalClockSeconds = signalClockSeconds + elapsedSeconds - remaining;
        std::vector<MapTrafficObstacle> occupied = obstacles;
        for (const auto* actor : actors) {
            occupied.push_back(actorFootprint(*actor, actor->distanceMeters));
        }
        std::vector<MapTrafficObstacle> reservations;
        std::vector<std::size_t> priorityOrder(actors.size());
        std::iota(priorityOrder.begin(), priorityOrder.end(), 0);
        std::stable_sort(
            priorityOrder.begin(), priorityOrder.end(), [&](std::size_t first, std::size_t second) {
                const auto isOnJunctionPath = [](const MapTrafficActor& actor) {
                    return mapTrafficRoutePositionOnJunctionPath(actor.route, actor.distanceMeters);
                };
                const bool firstOnJunction = isOnJunctionPath(*actors[first]);
                const bool secondOnJunction = isOnJunctionPath(*actors[second]);
                if (firstOnJunction != secondOnJunction) {
                    return firstOnJunction;
                }
                const auto authoredPriority = [](const MapTrafficActor& actor) {
                    const MapTrafficVehicleFilter vehicle{
                        actor.vehicleType, actor.unscheduledGroupIndex, actor.scheduled};
                    return mapTrafficPathPriorityForVehicle(
                        routeSegmentAtDistance(actor.route, actor.distanceMeters).pathRules,
                        vehicle);
                };
                const double firstPriority = authoredPriority(*actors[first]);
                const double secondPriority = authoredPriority(*actors[second]);
                if (firstPriority != secondPriority) {
                    return firstPriority > secondPriority;
                }
                return actors[first]->junctionWaitSeconds > actors[second]->junctionWaitSeconds;
            });
        for (const std::size_t index : priorityOrder) {
            auto& actor = *actors[index];
            const std::size_t own = obstacles.size() + index;
            bool blockedByCrossingReservation = false;
            const auto clearAt = [&](double distance, double clearance) {
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
                        return false;
                    }
                }
                for (const auto& reservation : reservations) {
                    // Following is handled by current footprints; reserve crossing movements.
                    if (std::abs(std::cos(footprint.pose.headingRadians -
                                          reservation.pose.headingRadians)) < 0.7 &&
                        footprintsOverlap(footprint, reservation, clearance)) {
                        blockedByCrossingReservation = true;
                        return false;
                    }
                }
                return true;
            };
            double target = actor.cruiseSpeedMetersPerSecond;
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
                target = std::min(target, std::sqrt(2.0 * braking * signalStopDistance));
            }
            const double lookahead = std::max(
                15.0, actor.speedMetersPerSecond * actor.speedMetersPerSecond / (2.0 * braking) +
                          actor.speedMetersPerSecond * headwaySeconds + actor.lengthMeters);
            for (double ahead = 0.0; ahead <= lookahead; ahead += 1.0) {
                if (ahead > signalStopDistance) {
                    break;
                }
                const double distance = actor.distanceMeters + ahead;
                if (!actor.route.closed && !actor.route.loopStartDistance &&
                    distance > actor.route.totalLength) {
                    break;
                }
                const double curveLimit = mapTrafficCurveSpeedLimit(
                    actor.route, distance, actor.cruiseSpeedMetersPerSecond,
                    actor.minimumTurnRadiusMeters);
                target = std::min(target, std::sqrt(curveLimit * curveLimit +
                                                    2.0 * braking * std::max(0.0, ahead - 2.0)));
                const std::optional<double> speedLimit = mapTrafficPathSpeedLimitForVehicle(
                    routeSegmentAtDistance(actor.route, distance).pathRules,
                    {actor.vehicleType, actor.unscheduledGroupIndex, actor.scheduled});
                if (speedLimit.has_value()) {
                    const double limitMetersPerSecond = *speedLimit / 3.6;
                    target =
                        std::min(target, std::sqrt(limitMetersPerSecond * limitMetersPerSecond +
                                                   2.0 * braking * std::max(0.0, ahead - 2.0)));
                }
                if (!clearAt(distance, 2.0)) {
                    const double gap = std::max(0.0, ahead - 1.0);
                    target =
                        std::min({target, gap / headwaySeconds, std::sqrt(2.0 * braking * gap)});
                    break;
                }
            }
            const double oldSpeed = actor.speedMetersPerSecond;
            const double newSpeed = target < oldSpeed
                                        ? std::max(target, oldSpeed - braking * step)
                                        : std::min(target, oldSpeed + acceleration * step);
            const double requestedTravel = (oldSpeed + newSpeed) * 0.5 * step;
            const double travel = std::min(requestedTravel, signalStopDistance);
            double admitted = 0.0;
            // Sweep rather than testing just the final pose, including at low frame rates.
            for (double offset = std::min(0.2, travel); offset <= travel && travel > 0.0;
                 offset = std::min(offset + 0.2, travel)) {
                if (!clearAt(actor.distanceMeters + offset, 0.5)) {
                    break;
                }
                admitted = offset;
                if (offset == travel) {
                    break;
                }
            }
            actor.speedMetersPerSecond =
                admitted + 1.0e-9 < travel || travel + 1.0e-9 < requestedTravel ? 0.0 : newSpeed;
            if (blockedByCrossingReservation) {
                actor.junctionWaitSeconds += step;
            } else if (admitted > 1.0e-9) {
                actor.junctionWaitSeconds = 0.0;
            }
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
            if (actor.speedMetersPerSecond > 0.1) {
                for (double ahead = 1.0; ahead < actor.speedMetersPerSecond * 3.0; ahead += 1.0) {
                    reservations.push_back(actorFootprint(actor, actor.distanceMeters + ahead));
                }
            }
        }
    }
}

} // namespace openbus::map
