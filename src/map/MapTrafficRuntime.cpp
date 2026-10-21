#include "map/MapTrafficRuntime.h"

#include "map/MapSplineGeometry.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <map>
#include <limits>
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
    const double distance = actor.route.closed
                                ? std::fmod(startDistanceMeters, actor.route.totalLength)
                                : loopOpenRoute && usableLength > 0.0
                                      ? std::fmod(startDistanceMeters, usableLength)
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
                                     double centerX, double centerY,
                                     double maximumDistanceMeters,
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
        route.segments.push_back({current, reversed, 0.0, 0.0, {}});
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
    if (!routeSegment.points.empty()) {
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
                const double heading = std::atan2(dx, dy);
                const auto offset = mapSplineRightOffset(heading, lateralOffsetMeters);
                return MapTrafficPose{
                    {from[0] + dx * fraction + offset[0],
                     from[1] + dy * fraction + offset[1], from[2] + dz * fraction},
                    heading, spline.splineId};
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
    const double splineLateral = routeSegment.pathLateralMeters +
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
        throw std::logic_error("Traffic route network must be finalized before listing route starts");
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
        const bool hasLinkedPrevious =
            spline.previousSplineId != 0 && !ambiguousSplineIds_.contains(spline.previousSplineId) &&
            previous != splinesById_.end() && previous->second.placement.nextSplineId == splineId;
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
    return MapTrafficSplineEndpointStatus{
        isConnected(spline->second.placement.previousSplineId),
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
        starts[cellFor(start->position)].push_back(index);
        if (segments_[index].spline.placement.splineId != 0) {
            lanesBySpline_[segments_[index].spline.placement.splineId].push_back(index);
        }
    }
    next_.resize(segments_.size());
    for (std::size_t index = 0; index < segments_.size(); ++index) {
        const auto& end = endpoints[index].end;
        const Cell cell = cellFor(end.position);
        double bestAlignment = -0.01;
        double bestDistance = std::numeric_limits<double>::infinity();
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
                    if (alignment > bestAlignment + 1.0e-8 ||
                        (std::abs(alignment - bestAlignment) <= 1.0e-8 &&
                         distance < bestDistance)) {
                        bestAlignment = alignment;
                        bestDistance = distance;
                        next_[index] = candidate;
                    }
                }
            }
        }
    }
    // Report open ends that nearly meet an aligned lane start. These usually indicate
    // authoring gaps or height mismatches rather than intentional dead ends.
    constexpr double nearMissMeters = 3.0;
    stats_ = {segments_.size(), 0, 0};
    for (std::size_t index = 0; index < segments_.size(); ++index) {
        if (next_[index]) {
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

std::optional<MapTrafficRoute> MapTrafficLaneNetwork::buildRoute(int splineId,
                                                                 std::size_t laneSelector) const {
    if (!finalized_) {
        throw std::logic_error("Traffic lane network must be finalized before route lookup");
    }
    const auto lanes = lanesBySpline_.find(splineId);
    if (lanes == lanesBySpline_.end() || lanes->second.empty()) {
        return std::nullopt;
    }
    std::size_t current = lanes->second[laneSelector % lanes->second.size()];
    MapTrafficRoute route;
    std::unordered_map<std::size_t, double> visited;
    while (!visited.contains(current)) {
        visited.emplace(current, route.totalLength);
        route.segments.push_back(segments_[current]);
        route.totalLength += lengths_[current];
        route.cumulativeEndDistances.push_back(route.totalLength);
        if (!next_[current]) {
            return route;
        }
        current = *next_[current];
    }
    route.loopStartDistance = visited.at(current);
    route.closed = *route.loopStartDistance == 0.0;
    return route;
}

} // namespace openbus::map
