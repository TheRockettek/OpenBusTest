#include "map/MapTrafficRuntime.h"

#include "map/MapSplineGeometry.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace openbus::map {

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
    actor.lateralOffsetMeters =
        mapTrafficLaneOffsetMeters(leftHandTraffic, laneOffsetMagnitudeMeters);
    return actor;
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
        route.segments.push_back({current, reversed});
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
    const std::array<double, 2> offset = mapSplineRightOffset(heading, lateralOffsetMeters);
    const double splineLateral = routeSegment.reversed ? -lateralOffsetMeters : lateralOffsetMeters;
    const double progress = splineDistance / spline.length;
    const double cant = spline.cantStart + (spline.cantEnd - spline.cantStart) * progress;
    const double elevation = mapSplineElevation(
        spline.elevation, spline.length, spline.gradientStart, spline.gradientEnd,
        spline.elevated ? std::optional<double>(spline.heightDelta) : std::nullopt, splineDistance);

    return MapTrafficPose{{center->x + offset[0], center->y + offset[1],
                           elevation + mapSplineCantHeightOffset(splineLateral, cant)},
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
    if (nextDistance >= actor.route.totalLength) {
        actor.distanceMeters = actor.route.totalLength;
        return false;
    }
    actor.distanceMeters = nextDistance;
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

} // namespace openbus::map
