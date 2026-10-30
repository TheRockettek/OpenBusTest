#include "map/MapTrafficRuntime.h"

#include "core/Environment.h"
#include "core/PerfTrace.h"
#include "map/MapSplineGeometry.h"
#include "map/MapTileWindow.h"

#include <algorithm>
#include <charconv>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <numbers>
#include <map>
#include <limits>
#include <numeric>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace openbus::map {

MapTrafficPose mapTrafficSceneryPassengerPose(const MapSceneryPose& sceneryPose,
                                              const std::array<double, 3>& localPosition,
                                              double seatHeight, double rotationDegrees) {
    const auto offset = transformMapSceneryVector(
        sceneryPose, {localPosition[0], localPosition[1], localPosition[2] + seatHeight});
    const double angle = rotationDegrees * std::numbers::pi / 180.0;
    const auto direction =
        transformMapSceneryVector(sceneryPose, {std::cos(angle), -std::sin(angle), 0.0});
    MapTrafficPose result;
    result.position = {sceneryPose.x + offset[0], sceneryPose.y + offset[1],
                       sceneryPose.z + offset[2]};
    result.headingRadians = -std::atan2(direction[1], direction[0]);
    return result;
}

bool mapTrafficPoseOnActiveTile(const MapTrafficPose& pose,
                                const std::vector<std::pair<int, int>>& activeTiles,
                                double tileSize) {
    const MapTileCoordinate tile{mapWorldTile(pose.position[0], tileSize),
                                 mapWorldTile(pose.position[1], tileSize)};
    return std::find(activeTiles.begin(), activeTiles.end(), tile) != activeTiles.end();
}

bool mapTrafficSpawnInsideForwardExclusion(const MapTrafficPose& playerPose,
                                           double playerLengthMeters, double playerWidthMeters,
                                           const MapTrafficPose& spawnPose, double spawnWidthMeters,
                                           double forwardDistanceMeters) noexcept {
    if (forwardDistanceMeters < 0.0) {
        return false;
    }
    const double forwardX = std::cos(playerPose.headingRadians);
    const double forwardY = std::sin(playerPose.headingRadians);
    const double relativeX = spawnPose.position[0] - playerPose.position[0];
    const double relativeY = spawnPose.position[1] - playerPose.position[1];
    const double forward = relativeX * forwardX + relativeY * forwardY;
    const double lateral = -relativeX * forwardY + relativeY * forwardX;
    const double playerFront = playerLengthMeters * 0.5;
    const double halfWidth = (playerWidthMeters + spawnWidthMeters) * 0.5 + 0.5;
    return forward >= playerFront && forward <= playerFront + forwardDistanceMeters &&
           std::abs(lateral) <= halfWidth;
}

double mapTrafficSpawnPriorityDistanceMeters(const MapTrafficPose& playerPose,
                                             const MapTrafficPose& spawnPose,
                                             double predictedForwardDistanceMeters) noexcept {
    const double forwardX = std::cos(playerPose.headingRadians);
    const double forwardY = std::sin(playerPose.headingRadians);
    const double relativeX = spawnPose.position[0] - playerPose.position[0];
    const double relativeY = spawnPose.position[1] - playerPose.position[1];
    const double forward = relativeX * forwardX + relativeY * forwardY;
    const double lateral = -relativeX * forwardY + relativeY * forwardX;
    const double pathEnd = std::max(0.0, predictedForwardDistanceMeters);
    const double closestForward = std::clamp(forward, 0.0, pathEnd);
    return std::hypot(forward - closestForward, lateral);
}

std::map<int, std::vector<MapTrafficPose>>
mapTrafficAssignPassengerPositions(const std::vector<MapPassengerStopAnchor>& stops,
                                   const std::vector<MapPassengerWaitingArea>& areas,
                                   const std::map<int, std::size_t>& requestedCounts) {
    constexpr double maximumDistanceMeters = 30.0;
    constexpr double maximumHeightDifferenceMeters = 2.0;
    std::vector<const MapPassengerWaitingArea*> orderedAreas;
    orderedAreas.reserve(areas.size());
    for (const auto& area : areas) {
        orderedAreas.push_back(&area);
    }
    std::sort(orderedAreas.begin(), orderedAreas.end(), [](const auto* first, const auto* second) {
        return std::tie(first->objectId, first->position) <
               std::tie(second->objectId, second->position);
    });
    std::map<int, std::vector<MapTrafficPose>> result;
    std::map<int, std::vector<const MapPassengerWaitingArea*>> assignedAreas;
    for (const auto* area : orderedAreas) {
        const MapPassengerStopAnchor* nearest = nullptr;
        double nearestDistance = maximumDistanceMeters;
        for (const auto& stop : stops) {
            if (stop.stopId == 0 ||
                std::abs(stop.position[2] - area->position[2]) > maximumHeightDifferenceMeters) {
                continue;
            }
            const double distance = std::hypot(stop.position[0] - area->position[0],
                                               stop.position[1] - area->position[1]);
            if (distance <= maximumDistanceMeters &&
                (nearest == nullptr || distance < nearestDistance ||
                 (distance == nearestDistance && stop.stopId < nearest->stopId))) {
                nearest = &stop;
                nearestDistance = distance;
            }
        }
        if (nearest != nullptr && !area->positions.empty()) {
            auto& positions = result[nearest->stopId];
            positions.insert(positions.end(), area->positions.begin(), area->positions.end());
            assignedAreas[nearest->stopId].push_back(area);
        }
    }

    for (const auto& [stopId, requestedCount] : requestedCounts) {
        auto positions = result.find(stopId);
        const auto areasForStop = assignedAreas.find(stopId);
        if (positions == result.end() || areasForStop == assignedAreas.end() ||
            positions->second.size() >= requestedCount) {
            continue;
        }
        std::size_t extraCount = requestedCount - positions->second.size();
        std::vector<std::vector<MapTrafficPose>> extrasByArea(areasForStop->second.size());
        for (std::size_t areaIndex = 0; areaIndex < areasForStop->second.size() && extraCount > 0;
             ++areaIndex) {
            const auto* area = areasForStop->second[areaIndex];
            const std::size_t share = (extraCount + areasForStop->second.size() - areaIndex - 1) /
                                      (areasForStop->second.size() - areaIndex);
            const auto& bounds = area->bounds;
            const std::array<double, 3> axisX = {bounds[1][0] - bounds[0][0],
                                                 bounds[1][1] - bounds[0][1],
                                                 bounds[1][2] - bounds[0][2]};
            const std::array<double, 3> axisY = {bounds[2][0] - bounds[0][0],
                                                 bounds[2][1] - bounds[0][1],
                                                 bounds[2][2] - bounds[0][2]};
            const double width = std::hypot(axisX[0], axisX[1], axisX[2]);
            const double depth = std::hypot(axisY[0], axisY[1], axisY[2]);
            if (width <= 1.0e-6 || depth <= 1.0e-6) {
                continue;
            }
            const double aspectRatio = width / depth;
            const std::size_t columns = std::max<std::size_t>(
                1, static_cast<std::size_t>(std::ceil(std::sqrt(share * aspectRatio))));
            const std::size_t rows = (share + columns - 1) / columns;
            auto& generated = extrasByArea[areaIndex];
            for (std::size_t passenger = 0; passenger < share; ++passenger) {
                const std::size_t row = passenger / columns;
                const std::size_t column = passenger % columns;
                const double u = (static_cast<double>(column) + 0.5) / static_cast<double>(columns);
                const double v = (static_cast<double>(row) + 0.5) / static_cast<double>(rows);
                const std::array<double, 3> selected = {bounds[0][0] + u * axisX[0] + v * axisY[0],
                                                        bounds[0][1] + u * axisX[1] + v * axisY[1],
                                                        bounds[0][2] + u * axisX[2] + v * axisY[2]};
                const auto nearestPosition = std::min_element(
                    area->positions.begin(), area->positions.end(),
                    [&](const MapTrafficPose& left, const MapTrafficPose& right) {
                        const auto distanceSquared = [&](const MapTrafficPose& pose) {
                            const double dx = pose.position[0] - selected[0];
                            const double dy = pose.position[1] - selected[1];
                            const double dz = pose.position[2] - selected[2];
                            return dx * dx + dy * dy + dz * dz;
                        };
                        return distanceSquared(left) < distanceSquared(right);
                    });
                MapTrafficPose generatedPose = *nearestPosition;
                generatedPose.position = selected;
                generated.push_back(generatedPose);
            }
            extraCount -= generated.size();
        }
        for (const auto& areaExtras : extrasByArea) {
            positions->second.insert(positions->second.end(), areaExtras.begin(), areaExtras.end());
        }
    }
    return result;
}

namespace {

constexpr double trafficBrakingDeceleration = 3.0;

template <typename... Values>
std::string trafficInputError(std::string_view message, Values&&... values) {
    std::ostringstream error;
    error << message << " (got " << std::setprecision(std::numeric_limits<double>::max_digits10);
    bool first = true;
    bool awaitingValue = false;
    const auto append = [&](const auto& value) {
        if (!first && !awaitingValue) {
            error << ", ";
        }
        error << value;
        first = false;
        if constexpr (std::is_convertible_v<decltype(value), std::string_view>) {
            const std::string_view text(value);
            awaitingValue = !text.empty() && text.back() == '=';
        } else {
            awaitingValue = false;
        }
    };
    (append(std::forward<Values>(values)), ...);
    error << ')';
    return error.str();
}

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
            throw std::invalid_argument(trafficInputError(
                std::string("Non-finite map traffic rule ") + name + ": ", field));
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

double mapTrafficRouteCurvatureFromPoses(const MapTrafficPose& from, const MapTrafficPose& to) {
    const double span =
        std::hypot(to.position[0] - from.position[0], to.position[1] - from.position[1]);
    const double angle =
        std::abs(std::remainder(to.headingRadians - from.headingRadians, 2.0 * std::numbers::pi));
    return angle / std::max(span, 0.01);
}

double mapTrafficRouteCurvatureAt(const MapTrafficRoute& route, double distanceMeters) {
    const double fromDistance = std::max(0.0, distanceMeters - 1.0);
    const double toDistance = route.closed || route.loopStartDistance
                                  ? distanceMeters + 1.0
                                  : std::min(route.totalLength, distanceMeters + 1.0);
    const auto from = sampleMapTrafficRoute(route, fromDistance, 0.0);
    const auto to = sampleMapTrafficRoute(route, toDistance, 0.0);
    if (!from || !to) {
        throw std::invalid_argument(
            trafficInputError("Cannot sample traffic curve", "distanceMeters=", distanceMeters,
                              "fromDistance=", fromDistance, "toDistance=", toDistance,
                              "totalLength=", route.totalLength, "fromSampled=", from.has_value(),
                              "toSampled=", to.has_value()));
    }
    return mapTrafficRouteCurvatureFromPoses(*from, *to);
}

double mapTrafficCurveSpeedLimitFromCurvature(double curvature, double cruiseSpeedMetersPerSecond) {
    constexpr double lateralAcceleration = 4.0;
    constexpr double minimumCurveSpeedMetersPerSecond = 2.0;
    const double curveSpeed = curvature < 1.0e-6
                                  ? cruiseSpeedMetersPerSecond
                                  : std::max(minimumCurveSpeedMetersPerSecond,
                                             std::sqrt(lateralAcceleration / curvature));
    return std::min(cruiseSpeedMetersPerSecond, curveSpeed);
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
        throw std::invalid_argument(trafficInputError(
            "Traffic junction-path query inputs are invalid", "segments=", route.segments.size(),
            "cumulativeEndDistances=", route.cumulativeEndDistances.size(),
            "distanceMeters=", distanceMeters, "totalLength=", route.totalLength));
    }
    const MapTrafficRouteSegment& segment = routeSegmentAtDistance(route, distanceMeters);
    return !segment.points.empty() || segment.pathRules.roundabout;
}

bool mapTrafficActorOnJunctionPath(const MapTrafficActor& actor, double distanceMeters) {
    if (!std::isfinite(distanceMeters) || !std::isfinite(actor.lengthMeters) ||
        actor.lengthMeters <= 0.0) {
        throw std::invalid_argument(trafficInputError(
            "Traffic junction actor query inputs are invalid", "distanceMeters=", distanceMeters,
            "lengthMeters=", actor.lengthMeters));
    }
    const double halfLength = actor.lengthMeters * 0.5;
    return mapTrafficRoutePositionOnJunctionPath(actor.route, distanceMeters) ||
           mapTrafficRoutePositionOnJunctionPath(actor.route, distanceMeters - halfLength) ||
           mapTrafficRoutePositionOnJunctionPath(actor.route, distanceMeters + halfLength);
}

double mapTrafficLaneOffsetMeters(bool leftHandTraffic, double distanceMeters) {
    if (!std::isfinite(distanceMeters) || distanceMeters < 0.0) {
        throw std::invalid_argument(
            trafficInputError("Map traffic lane offset must be a finite, non-negative distance",
                              "distanceMeters=", distanceMeters));
    }
    return leftHandTraffic ? -distanceMeters : distanceMeters;
}

double mapTrafficPassengerSideSign(bool leftHandTraffic) noexcept {
    return leftHandTraffic ? -1.0 : 1.0;
}

double mapTrafficPassengerCurbOffsetMeters(bool leftHandTraffic, double laneOffsetMeters,
                                           double vehicleWidthMeters, double clearanceMeters) {
    if (!std::isfinite(laneOffsetMeters) || !std::isfinite(vehicleWidthMeters) ||
        vehicleWidthMeters <= 0.0 || !std::isfinite(clearanceMeters) || clearanceMeters < 0.0) {
        throw std::invalid_argument(trafficInputError(
            "Passenger curb offset inputs are invalid", "laneOffsetMeters=", laneOffsetMeters,
            "vehicleWidthMeters=", vehicleWidthMeters, "clearanceMeters=", clearanceMeters));
    }
    return laneOffsetMeters + mapTrafficPassengerSideSign(leftHandTraffic) *
                                  (vehicleWidthMeters * 0.5 + clearanceMeters);
}

double mapTrafficHeadingToVehicleYawDegrees(double headingRadians) {
    if (!std::isfinite(headingRadians)) {
        throw std::invalid_argument(trafficInputError("Map traffic heading must be finite",
                                                      "headingRadians=", headingRadians));
    }
    return std::remainder(90.0 - headingRadians * 180.0 / std::numbers::pi, 360.0);
}

std::size_t mapScheduledPassengerQueueSize(std::size_t nominalCount, double humanScale,
                                           std::size_t safetyLimit) noexcept {
    if (nominalCount == 0 || safetyLimit == 0 || !std::isfinite(humanScale) || humanScale <= 0.0) {
        return 0;
    }
    const long double scaled =
        static_cast<long double>(nominalCount) * static_cast<long double>(humanScale);
    if (scaled >= static_cast<long double>(safetyLimit)) {
        return safetyLimit;
    }
    return static_cast<std::size_t>(std::floor(scaled + 0.5L));
}

std::size_t mapScheduledPassengerQueueAfterDeparture(std::size_t waitingPassengers,
                                                     std::size_t initialPassengers) noexcept {
    return std::max(waitingPassengers, initialPassengers);
}

std::size_t mapScheduledPassengerModelIdentity(int stopId, std::size_t queuePosition) {
    if (stopId <= 0) {
        throw std::invalid_argument(
            trafficInputError("Passenger model identity requires a positive stop ID",
                              "stopId=", stopId, "queuePosition=", queuePosition));
    }
    const std::size_t stopIdentity = static_cast<std::size_t>(stopId);
    if (queuePosition > (std::numeric_limits<std::size_t>::max)() - stopIdentity) {
        throw std::overflow_error(
            trafficInputError("Passenger model identity exceeds its supported range",
                              "stopId=", stopId, "queuePosition=", queuePosition,
                              "maximumIdentity=", (std::numeric_limits<std::size_t>::max)()));
    }
    return stopIdentity + queuePosition;
}

std::size_t mapScheduledPassengerQueueSlot(std::size_t initialCount, std::size_t passengerIndex) {
    if (passengerIndex >= initialCount) {
        throw std::invalid_argument(
            trafficInputError("Passenger queue index exceeds its initial count",
                              "initialCount=", initialCount, "passengerIndex=", passengerIndex));
    }
    return initialCount - passengerIndex - 1;
}

std::array<double, 2> mapTrafficPassengerWorldOffset(bool leftHandTraffic, double headingRadians,
                                                     double lateralOffsetMeters,
                                                     double longitudinalOffsetMeters) {
    if (!std::isfinite(headingRadians) || !std::isfinite(lateralOffsetMeters) ||
        !std::isfinite(longitudinalOffsetMeters)) {
        throw std::invalid_argument(trafficInputError(
            "Passenger world offset inputs must be finite", "headingRadians=", headingRadians,
            "lateralOffsetMeters=", lateralOffsetMeters,
            "longitudinalOffsetMeters=", longitudinalOffsetMeters));
    }
    const double sideOffset = lateralOffsetMeters * mapTrafficPassengerSideSign(leftHandTraffic);
    return {sideOffset * std::cos(headingRadians) +
                longitudinalOffsetMeters * std::sin(headingRadians),
            -sideOffset * std::sin(headingRadians) +
                longitudinalOffsetMeters * std::cos(headingRadians)};
}

MapTrafficPose mapTrafficPassengerQueuePose(const MapTrafficPose& stopPose, bool leftHandTraffic,
                                            std::size_t queueSlot) {
    const double row = static_cast<double>(queueSlot / 3);
    const double column = static_cast<double>(queueSlot % 3) - 1.0;
    const auto offset = mapTrafficPassengerWorldOffset(leftHandTraffic, stopPose.headingRadians,
                                                       2.0 + 0.65 * column, -0.8 * row);
    MapTrafficPose pose = stopPose;
    pose.position[0] += offset[0];
    pose.position[1] += offset[1];
    return pose;
}

std::vector<std::size_t> mapPassengerServiceUpdateOrder(const std::vector<int>& serviceStopIds,
                                                        std::size_t rotation) {
    std::map<int, std::vector<std::size_t>> activeIndicesByStop;
    std::vector<std::size_t> inactiveIndices;
    inactiveIndices.reserve(serviceStopIds.size());
    for (std::size_t index = 0; index < serviceStopIds.size(); ++index) {
        if (serviceStopIds[index] == 0) {
            inactiveIndices.push_back(index);
        } else {
            activeIndicesByStop[serviceStopIds[index]].push_back(index);
        }
    }
    std::vector<std::size_t> order;
    order.reserve(serviceStopIds.size());
    for (const auto& [stopId, activeIndices] : activeIndicesByStop) {
        (void)stopId;
        const std::size_t startIndex = rotation % activeIndices.size();
        order.insert(order.end(), activeIndices.begin() + static_cast<std::ptrdiff_t>(startIndex),
                     activeIndices.end());
        order.insert(order.end(), activeIndices.begin(),
                     activeIndices.begin() + static_cast<std::ptrdiff_t>(startIndex));
    }
    order.insert(order.end(), inactiveIndices.begin(), inactiveIndices.end());
    return order;
}

MapPassengerServiceRuntime::MapPassengerServiceRuntime(std::size_t seatCount)
    : seatDestinations_(seatCount), seatPassengerModelIndices_(seatCount) {}

void MapPassengerServiceRuntime::setWaitingPassengers(int stopId, std::size_t count) {
    if (stopId != 0) {
        waitingByStop_[stopId] = count;
    }
}

bool MapPassengerServiceRuntime::arrive(const MapPassengerStopService& stop) {
    if (dwelling_ || stop.stopId == 0 || !std::isfinite(stop.dwellSeconds) ||
        stop.dwellSeconds < 0.0) {
        return false;
    }
    currentStopId_ = stop.stopId;
    nextStopId_ = stop.nextStopId;
    queuedAlightingSeats_.clear();
    alightingSeat_.reset();
    alightingAccumulatorSeconds_ = 0.0;
    std::vector<bool> selectedSeats(seatDestinations_.size(), false);
    for (std::size_t seat = 0; seat < seatDestinations_.size(); ++seat) {
        if (seatDestinations_[seat] && *seatDestinations_[seat] == stop.stopId) {
            queuedAlightingSeats_.push_back(seat);
            selectedSeats[seat] = true;
        }
    }
    std::size_t remainingToAlight = stop.passengersAlighting > queuedAlightingSeats_.size()
                                        ? stop.passengersAlighting - queuedAlightingSeats_.size()
                                        : 0;
    for (std::size_t seat = seatDestinations_.size(); seat > 0 && remainingToAlight > 0; --seat) {
        const std::size_t seatIndex = seat - 1;
        if (seatDestinations_[seatIndex] && !selectedSeats[seatIndex]) {
            queuedAlightingSeats_.push_back(seatIndex);
            selectedSeats[seatIndex] = true;
            --remainingToAlight;
        }
    }
    if (!queuedAlightingSeats_.empty()) {
        alightingSeat_ = queuedAlightingSeats_.front();
        queuedAlightingSeats_.erase(queuedAlightingSeats_.begin());
    }
    std::size_t& waiting = waitingByStop_[stop.stopId];
    const std::size_t available = (std::numeric_limits<std::size_t>::max)() - waiting;
    waiting += std::min(stop.waitingPassengers, available);
    dwellRemainingSeconds_ = stop.dwellSeconds;
    boardingAccumulatorSeconds_ = 0.0;
    dwelling_ = stop.dwellSeconds > 0.0 || alightingSeat_.has_value();
    return true;
}

void MapPassengerServiceRuntime::advance(double elapsedSeconds) {
    if (!dwelling_ || !std::isfinite(elapsedSeconds) || elapsedSeconds <= 0.0) {
        return;
    }
    constexpr double boardingIntervalSeconds = 0.75;
    constexpr double passengerAnimationSeconds = 0.75;
    const double dwellAtStartSeconds = dwellRemainingSeconds_;
    double elapsedForAlightingSeconds = 0.0;
    double alightingSeconds = elapsedSeconds;
    while (alightingSeconds > 1.0e-9 && alightingSeat_) {
        const double timeToAlight = passengerAnimationSeconds - alightingAccumulatorSeconds_;
        const double alightingTime = std::min(alightingSeconds, timeToAlight);
        alightingAccumulatorSeconds_ += alightingTime;
        alightingSeconds -= alightingTime;
        elapsedForAlightingSeconds += alightingTime;
        if (alightingAccumulatorSeconds_ + 1.0e-9 >= passengerAnimationSeconds) {
            seatDestinations_[*alightingSeat_].reset();
            seatPassengerModelIndices_[*alightingSeat_].reset();
            alightingSeat_.reset();
            alightingAccumulatorSeconds_ = 0.0;
            if (!queuedAlightingSeats_.empty()) {
                alightingSeat_ = queuedAlightingSeats_.front();
                queuedAlightingSeats_.erase(queuedAlightingSeats_.begin());
            }
        }
    }

    const double dwellAfterAlightingSeconds =
        std::max(0.0, dwellAtStartSeconds - elapsedForAlightingSeconds);
    double activeSeconds = std::min(std::max(0.0, elapsedSeconds - elapsedForAlightingSeconds),
                                    dwellAfterAlightingSeconds);
    std::size_t& waiting = waitingByStop_[currentStopId_];
    while (activeSeconds > 1.0e-9) {
        if (!boardingSeat_) {
            if (nextStopId_ == 0 || waiting == 0) {
                break;
            }
            const auto seat = std::find_if(
                seatDestinations_.begin(), seatDestinations_.end(),
                [](const std::optional<int>& destination) { return !destination.has_value(); });
            if (seat == seatDestinations_.end()) {
                break;
            }
            boardingSeat_ =
                static_cast<std::size_t>(std::distance(seatDestinations_.begin(), seat));
            --waiting;
            boardingPassengerModelIndex_ =
                mapScheduledPassengerModelIdentity(currentStopId_, waiting);
            boardingAccumulatorSeconds_ = 0.0;
        }
        const double timeToBoard = boardingIntervalSeconds - boardingAccumulatorSeconds_;
        const double boardingTime = std::min(activeSeconds, timeToBoard);
        boardingAccumulatorSeconds_ += boardingTime;
        activeSeconds -= boardingTime;
        if (boardingAccumulatorSeconds_ + 1.0e-9 >= boardingIntervalSeconds) {
            seatDestinations_[*boardingSeat_] = nextStopId_;
            seatPassengerModelIndices_[*boardingSeat_] = boardingPassengerModelIndex_;
            boardingSeat_.reset();
            boardingPassengerModelIndex_.reset();
            boardingAccumulatorSeconds_ = 0.0;
        }
    }
    dwellRemainingSeconds_ =
        std::max(0.0, dwellAtStartSeconds - std::min(elapsedSeconds, dwellAtStartSeconds));
    const double overrunSeconds = elapsedSeconds - std::min(elapsedSeconds, dwellAtStartSeconds);
    if (boardingSeat_ && overrunSeconds > 0.0) {
        boardingAccumulatorSeconds_ += overrunSeconds;
        if (boardingAccumulatorSeconds_ + 1.0e-9 >= boardingIntervalSeconds) {
            seatDestinations_[*boardingSeat_] = nextStopId_;
            seatPassengerModelIndices_[*boardingSeat_] = boardingPassengerModelIndex_;
            boardingSeat_.reset();
            boardingPassengerModelIndex_.reset();
            boardingAccumulatorSeconds_ = 0.0;
        }
    }
    if (dwellRemainingSeconds_ <= 1.0e-9 && !boardingSeat_ && !alightingSeat_) {
        dwellRemainingSeconds_ = 0.0;
        dwelling_ = false;
    }
}

bool MapPassengerServiceRuntime::dwelling() const noexcept {
    return dwelling_;
}

int MapPassengerServiceRuntime::currentStopId() const noexcept {
    return currentStopId_;
}

bool MapPassengerServiceRuntime::boardingPassenger() const noexcept {
    return boardingSeat_.has_value();
}

double MapPassengerServiceRuntime::boardingProgress() const noexcept {
    return boardingSeat_ ? std::clamp(boardingAccumulatorSeconds_ / 0.75, 0.0, 1.0) : 0.0;
}

std::optional<std::size_t> MapPassengerServiceRuntime::boardingSeatIndex() const noexcept {
    return boardingSeat_;
}

bool MapPassengerServiceRuntime::alightingPassenger() const noexcept {
    return alightingSeat_.has_value();
}

double MapPassengerServiceRuntime::alightingProgress() const noexcept {
    return alightingSeat_ ? std::clamp(alightingAccumulatorSeconds_ / 0.75, 0.0, 1.0) : 0.0;
}

std::optional<std::size_t> MapPassengerServiceRuntime::alightingSeatIndex() const noexcept {
    return alightingSeat_;
}

std::optional<std::size_t>
MapPassengerServiceRuntime::boardingPassengerModelIndex() const noexcept {
    return boardingPassengerModelIndex_;
}

std::size_t MapPassengerServiceRuntime::waitingPassengers(int stopId) const noexcept {
    const auto found = waitingByStop_.find(stopId);
    return found == waitingByStop_.end() ? 0 : found->second;
}

std::size_t MapPassengerServiceRuntime::passengerCountOnSeat(std::size_t seatIndex) const noexcept {
    return seatIndex < seatDestinations_.size() && seatDestinations_[seatIndex].has_value() ? 1 : 0;
}

std::optional<std::size_t>
MapPassengerServiceRuntime::passengerModelIndexOnSeat(std::size_t seatIndex) const noexcept {
    return seatIndex < seatPassengerModelIndices_.size() ? seatPassengerModelIndices_[seatIndex]
                                                         : std::nullopt;
}

std::size_t MapPassengerServiceRuntime::onboardPassengerCount() const noexcept {
    return static_cast<std::size_t>(std::count_if(
        seatDestinations_.begin(), seatDestinations_.end(),
        [](const std::optional<int>& destination) { return destination.has_value(); }));
}

std::size_t MapPassengerServiceRuntime::seatCount() const noexcept {
    return seatDestinations_.size();
}

void advanceMapPassengerServiceWithSharedQueue(MapPassengerServiceRuntime& service, int stopId,
                                               std::size_t& sharedWaitingPassengers,
                                               double elapsedSeconds) {
    if (stopId == 0) {
        throw std::invalid_argument(trafficInputError(
            "Shared passenger queue requires a valid stop ID", "stopId=", stopId,
            "waitingPassengers=", sharedWaitingPassengers, "elapsedSeconds=", elapsedSeconds));
    }
    service.setWaitingPassengers(stopId, sharedWaitingPassengers);
    service.advance(elapsedSeconds);
    sharedWaitingPassengers = service.waitingPassengers(stopId);
}

double mapTrafficHourAtElapsedSeconds(double startHour, double elapsedSeconds) {
    if (!std::isfinite(startHour) || !std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0) {
        throw std::invalid_argument(
            trafficInputError("Map traffic clock requires a finite start hour and elapsed time",
                              "startHour=", startHour, "elapsedSeconds=", elapsedSeconds));
    }
    constexpr double hoursPerDay = 24.0;
    const double elapsedHours = std::fmod(elapsedSeconds / 3600.0, hoursPerDay);
    const double hour = std::fmod(std::fmod(startHour, hoursPerDay) + elapsedHours, hoursPerDay);
    return hour < 0.0 ? hour + hoursPerDay : hour;
}

int mapTrafficWeekdayMaskAtElapsedSeconds(int startWeekday, double startHour,
                                          double elapsedSeconds) {
    if (startWeekday < 0 || startWeekday > 6 || !std::isfinite(startHour) || startHour < 0.0 ||
        startHour >= 24.0 || !std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0) {
        throw std::invalid_argument(
            trafficInputError("Map AI weekday clock is invalid", "startWeekday=", startWeekday,
                              "startHour=", startHour, "elapsedSeconds=", elapsedSeconds));
    }
    constexpr double secondsPerDay = 86400.0;
    int elapsedDays = static_cast<int>(std::fmod(std::floor(elapsedSeconds / secondsPerDay), 7.0));
    if (std::fmod(elapsedSeconds, secondsPerDay) >= secondsPerDay - startHour * 3600.0) {
        ++elapsedDays;
    }
    const int weekday = (startWeekday + elapsedDays) % 7;
    if (weekday == 0) {
        return 4;
    }
    return weekday == 6 ? 2 : 1;
}

double mapTrafficNextDepartureElapsedSeconds(double startHour, double elapsedSeconds,
                                             double departureMinutes) {
    if (!std::isfinite(startHour) || startHour < 0.0 || startHour >= 24.0 ||
        !std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0 ||
        !std::isfinite(departureMinutes) || departureMinutes < 0.0) {
        throw std::invalid_argument(trafficInputError(
            "Timetable departure clock inputs are invalid", "startHour=", startHour,
            "elapsedSeconds=", elapsedSeconds, "departureMinutes=", departureMinutes));
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
    return std::max(0.0, nextDepartureSeconds - startSeconds);
}

std::size_t mapTrafficNextDepartureIndex(double startHour, double elapsedSeconds,
                                         const std::vector<double>& departureMinutes) {
    if (departureMinutes.empty()) {
        throw std::invalid_argument(trafficInputError("Timetable departure list cannot be empty",
                                                      "departureCount=", departureMinutes.size()));
    }
    std::size_t nextIndex = 0;
    double nextElapsedSeconds =
        mapTrafficNextDepartureElapsedSeconds(startHour, elapsedSeconds, departureMinutes.front());
    for (std::size_t index = 1; index < departureMinutes.size(); ++index) {
        const double candidateElapsedSeconds = mapTrafficNextDepartureElapsedSeconds(
            startHour, elapsedSeconds, departureMinutes[index]);
        if (candidateElapsedSeconds < nextElapsedSeconds) {
            nextIndex = index;
            nextElapsedSeconds = candidateElapsedSeconds;
        }
    }
    return nextIndex;
}

bool mapTrafficScheduledDepartureDue(double elapsedSeconds, double departureElapsedSeconds) {
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0 ||
        !std::isfinite(departureElapsedSeconds) || departureElapsedSeconds < 0.0) {
        throw std::invalid_argument(trafficInputError(
            "Scheduled departure comparison inputs are invalid", "elapsedSeconds=", elapsedSeconds,
            "departureElapsedSeconds=", departureElapsedSeconds));
    }
    return elapsedSeconds + 1.0e-9 >= departureElapsedSeconds;
}

namespace {

template <typename T>
std::optional<T>
mapTrafficGroupRuleForVehicle(const MapTrafficPathRules& rules,
                              const MapTrafficVehicleFilter& vehicle,
                              std::optional<T> MapTrafficGroupRules::* rule) noexcept {
    if (!vehicle.trafficRuleGroupIndex.has_value()) {
        return std::nullopt;
    }
    const auto configuredValue = [&](std::size_t groupIndex) -> std::optional<T> {
        const auto group = std::find_if(rules.groups.begin(), rules.groups.end(),
                                        [&](const MapTrafficGroupRules& candidate) {
                                            return candidate.groupIndex == groupIndex;
                                        });
        return group == rules.groups.end() ? std::nullopt : (*group).*rule;
    };
    if (const std::optional<T> specific = configuredValue(*vehicle.trafficRuleGroupIndex)) {
        return specific;
    }
    return *vehicle.trafficRuleGroupIndex == 0 ? std::nullopt : configuredValue(0);
}

} // namespace

MapTrafficPathRules mapTrafficRulesForPath(const std::vector<MapTileRule>& rules,
                                           std::size_t pathIndex) {
    MapTrafficPathRules result;
    for (const MapTileRule& rule : rules) {
        const long long rulePathIndex = parseRuleNumber<long long>(rule.fields[0], "path index");
        if (rulePathIndex < 0) {
            throw std::invalid_argument(
                trafficInputError("Map traffic rule path index cannot be negative",
                                  "pathIndex=", rulePathIndex, "requestedPathIndex=", pathIndex));
        }
        if (static_cast<std::size_t>(rulePathIndex) != pathIndex) {
            continue;
        }
        const std::string kind = lowerRuleField(rule.fields[1]);
        if (kind != "speedlimit" && kind != "trafficdensity" && kind != "priority" &&
            kind != "no_cars" && kind != "no_unsched_traffic" && kind != "bus" &&
            kind != "trucks" && kind != "overtaking_prohib") {
            continue;
        }
        const long long groupIndex = parseRuleNumber<long long>(rule.fields[3], "vehicle group");
        if (groupIndex < 0) {
            throw std::invalid_argument(trafficInputError(
                "Map traffic rule vehicle group cannot be negative", "vehicleGroup=", groupIndex));
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
            } else if (kind == "no_unsched_traffic") {
                groupRules->unscheduledTrafficProhibited.reset();
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
                throw std::invalid_argument(trafficInputError(
                    "Map traffic density rule cannot be negative", "density=", value));
            }
            groupRules->density = value;
        } else if (kind == "priority") {
            if (value < 0.0 || value > 256.0) {
                throw std::invalid_argument(trafficInputError(
                    "Map traffic priority must be in [0, 256]", "priority=", value));
            }
            groupRules->priority = value;
        } else if (kind == "no_cars") {
            groupRules->carsProhibited = value != 0.0;
        } else if (kind == "no_unsched_traffic") {
            groupRules->unscheduledTrafficProhibited = value != 0.0;
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
    if (mapTrafficGroupRuleForVehicle(rules, vehicle,
                                      &MapTrafficGroupRules::unscheduledTrafficProhibited)
            .value_or(false)) {
        return 0.0;
    }
    return mapTrafficGroupRuleForVehicle(rules, vehicle, &MapTrafficGroupRules::density)
        .value_or(rules.density.value_or(1.0));
}

std::optional<double>
mapTrafficPathSpeedLimitForVehicle(const MapTrafficPathRules& rules,
                                   const MapTrafficVehicleFilter& vehicle) noexcept {
    const std::optional<double> groupSpeedLimit =
        mapTrafficGroupRuleForVehicle(rules, vehicle, &MapTrafficGroupRules::speedLimitKmh);
    return groupSpeedLimit.has_value() ? groupSpeedLimit : rules.speedLimitKmh;
}

double mapTrafficSpawnSpeedMetersPerSecond(const MapTrafficActor& actor, double distanceMeters) {
    if (!std::isfinite(distanceMeters) || !std::isfinite(actor.cruiseSpeedMetersPerSecond) ||
        actor.cruiseSpeedMetersPerSecond < 0.0 || actor.route.segments.empty() ||
        actor.route.cumulativeEndDistances.size() != actor.route.segments.size()) {
        throw std::invalid_argument(trafficInputError(
            "Traffic spawn speed requires a valid actor and route position", "distanceMeters=",
            distanceMeters, "cruiseSpeedMetersPerSecond=", actor.cruiseSpeedMetersPerSecond,
            "segments=", actor.route.segments.size(),
            "cumulativeEndDistances=", actor.route.cumulativeEndDistances.size()));
    }
    const MapTrafficVehicleFilter vehicle{actor.vehicleType, actor.unscheduledGroupIndex,
                                          actor.scheduled, actor.trafficRuleGroupIndex};
    const auto speedLimitKmh = mapTrafficPathSpeedLimitForVehicle(
        routeSegmentAtDistance(actor.route, distanceMeters).pathRules, vehicle);
    if (!speedLimitKmh.has_value()) {
        return actor.cruiseSpeedMetersPerSecond;
    }
    return std::min(actor.cruiseSpeedMetersPerSecond, std::max(0.0, *speedLimitKmh / 3.6));
}

double mapTrafficPathPriorityForVehicle(const MapTrafficPathRules& rules,
                                        const MapTrafficVehicleFilter& vehicle) noexcept {
    if (rules.roundabout) {
        return 256.0;
    }
    return mapTrafficGroupRuleForVehicle(rules, vehicle, &MapTrafficGroupRules::priority)
        .value_or(rules.priority);
}

bool mapTrafficPathAllowsVehicle(const MapTrafficPathRules& rules,
                                 const MapTrafficVehicleFilter& vehicle) {
    if (rules.priority <= 0.0 ||
        mapTrafficGroupRuleForVehicle(rules, vehicle, &MapTrafficGroupRules::priority)
                .value_or(rules.priority) <= 0.0) {
        return false;
    }
    if (vehicle.scheduled && !vehicle.trafficRuleGroupIndex) {
        const bool hasGroupPriorities = std::any_of(
            rules.groups.begin(), rules.groups.end(),
            [](const MapTrafficGroupRules& group) { return group.priority.has_value(); });
        const bool everyGroupPriorityIsZero =
            hasGroupPriorities && std::all_of(rules.groups.begin(), rules.groups.end(),
                                              [](const MapTrafficGroupRules& group) {
                                                  return !group.priority.has_value() ||
                                                         *group.priority <= 0.0;
                                              });
        if (everyGroupPriorityIsZero) {
            return false;
        }
        const bool hasGroupDensities = std::any_of(
            rules.groups.begin(), rules.groups.end(),
            [](const MapTrafficGroupRules& group) { return group.density.has_value(); });
        const bool everyGroupDensityIsZero =
            hasGroupDensities && std::all_of(rules.groups.begin(), rules.groups.end(),
                                             [](const MapTrafficGroupRules& group) {
                                                 return group.density.has_value() &&
                                                        *group.density <= 0.0;
                                             });
        if ((rules.density.has_value() && *rules.density <= 0.0) || everyGroupDensityIsZero) {
            return false;
        }
    }
    if (!vehicle.scheduled) {
        const double density = mapTrafficPathDensityForVehicle(rules, vehicle);
        if (!std::isfinite(density) || density <= 0.0) {
            return false;
        }
    } else if (vehicle.vehicleType == 2) {
        bool hasBusAccessRule = false;
        bool busesAllowed = false;
        for (const MapTrafficGroupRules& group : rules.groups) {
            if (group.busesAllowed.has_value()) {
                hasBusAccessRule = true;
                busesAllowed = busesAllowed || *group.busesAllowed;
            }
        }
        if (hasBusAccessRule) {
            return busesAllowed;
        }
        return true;
    }
    const bool carsProhibited =
        mapTrafficGroupRuleForVehicle(rules, vehicle, &MapTrafficGroupRules::carsProhibited)
            .value_or(rules.carsProhibited);
    const bool busesAllowed =
        mapTrafficGroupRuleForVehicle(rules, vehicle, &MapTrafficGroupRules::busesAllowed)
            .value_or(rules.busesAllowed);
    const bool trucksAllowed =
        mapTrafficGroupRuleForVehicle(rules, vehicle, &MapTrafficGroupRules::trucksAllowed)
            .value_or(rules.trucksAllowed);
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
        const auto invalidDeparture =
            std::find_if(departureMinutes.begin(), departureMinutes.end(),
                         [](double minute) { return !std::isfinite(minute) || minute < 0.0; });
        const std::size_t invalidIndex =
            static_cast<std::size_t>(invalidDeparture - departureMinutes.begin());
        throw std::invalid_argument(trafficInputError(
            "Timetable departure partition is invalid", "departureCount=", departureMinutes.size(),
            "partitionCount=", partitionCount, "invalidDepartureIndex=",
            invalidDeparture == departureMinutes.end() ? (std::numeric_limits<std::size_t>::max)()
                                                       : invalidIndex,
            "invalidDepartureMinutes=",
            invalidDeparture == departureMinutes.end() ? 0.0 : *invalidDeparture));
    }
    std::vector<std::vector<double>> partitions(partitionCount);
    for (std::size_t index = 0; index < departureMinutes.size(); ++index) {
        partitions[index % partitionCount].push_back(departureMinutes[index]);
    }
    return partitions;
}

std::vector<std::size_t>
mapTrafficAllocateScheduledBuses(const std::vector<std::size_t>& departureCounts,
                                 std::size_t busPoolLimit) {
    std::vector<std::size_t> busesPerRoute(departureCounts.size(), 0);
    std::size_t allocatedBuses = 0;
    for (std::size_t routeIndex = 0;
         routeIndex < departureCounts.size() && allocatedBuses < busPoolLimit; ++routeIndex) {
        if (departureCounts[routeIndex] == 0) {
            continue;
        }
        busesPerRoute[routeIndex] = 1;
        ++allocatedBuses;
    }
    while (allocatedBuses < busPoolLimit) {
        std::size_t bestRoute = departureCounts.size();
        double bestDepartureLoad = 0.0;
        for (std::size_t routeIndex = 0; routeIndex < departureCounts.size(); ++routeIndex) {
            if (busesPerRoute[routeIndex] >= departureCounts[routeIndex]) {
                continue;
            }
            const double departureLoad = static_cast<double>(departureCounts[routeIndex]) /
                                         static_cast<double>(busesPerRoute[routeIndex] + 1);
            if (departureLoad > bestDepartureLoad) {
                bestDepartureLoad = departureLoad;
                bestRoute = routeIndex;
            }
        }
        if (bestRoute == departureCounts.size()) {
            break;
        }
        ++busesPerRoute[bestRoute];
        ++allocatedBuses;
    }
    return busesPerRoute;
}

MapTrafficActor makeMapTrafficActor(MapTrafficRoute route, std::string modelPath,
                                    double speedMetersPerSecond, double laneOffsetMagnitudeMeters,
                                    bool leftHandTraffic,
                                    std::optional<std::string> driverModelPath) {
    if (modelPath.empty() || (driverModelPath.has_value() && driverModelPath->empty()) ||
        !std::isfinite(speedMetersPerSecond) || speedMetersPerSecond < 0.0 ||
        route.segments.empty() || route.cumulativeEndDistances.size() != route.segments.size() ||
        !std::isfinite(route.totalLength) || route.totalLength <= 0.0) {
        throw std::invalid_argument(trafficInputError(
            "Map traffic actor has invalid route, model path, or speed",
            "modelPathEmpty=", modelPath.empty(),
            "driverModelPathEmpty=", driverModelPath.has_value() && driverModelPath->empty(),
            "speedMetersPerSecond=", speedMetersPerSecond, "segments=", route.segments.size(),
            "cumulativeEndDistances=", route.cumulativeEndDistances.size(),
            "totalLength=", route.totalLength));
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

MapTrafficRouteSegment reverseMapTrafficRouteSegment(MapTrafficRouteSegment segment,
                                                     double segmentLengthMeters) {
    if (!std::isfinite(segmentLengthMeters) || segmentLengthMeters <= 0.0) {
        throw std::invalid_argument(
            trafficInputError("Reversed traffic segment length must be positive and finite",
                              "segmentLengthMeters=", segmentLengthMeters));
    }
    if (!segment.points.empty()) {
        std::reverse(segment.points.begin(), segment.points.end());
    } else {
        const double startDistance = segment.splineDistanceAtRouteStartMeters.value_or(
            segment.reversed ? segment.spline.placement.length : 0.0);
        segment.splineDistanceAtRouteStartMeters =
            startDistance + (segment.reversed ? -segmentLengthMeters : segmentLengthMeters);
        segment.reversed = !segment.reversed;
    }
    return segment;
}

void resetMapTrafficActorForEntry(MapTrafficActor& actor) noexcept {
    actor.distanceMeters = 0.0;
    actor.speedMetersPerSecond = 0.0;
    actor.targetSpeedMetersPerSecond = 0.0;
    actor.junctionWaitSeconds = 0.0;
    actor.scheduledStopDistanceMeters.reset();
    actor.blockReason = MapTrafficBlockReason::None;
    actor.blockingActor = nullptr;
    actor.blockingObstacleIndex.reset();
    actor.junctionClaimEndMeters.reset();
    actor.committedSignalSegmentIndex.reset();
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

std::vector<int> mapTrafficRoutePrefixMatchingTimetable(const std::vector<int>& timetableSplineIds,
                                                        const std::vector<int>& routeSplineIds) {
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

struct TimetableRoutePrefixMatch {
    std::size_t routeSegments = 0;
    std::size_t timetableEntries = 0;
};

TimetableRoutePrefixMatch matchedTimetablePathLength(
    const std::vector<MapTrafficRouteRequirement>& timetablePath, std::size_t timetableStart,
    const std::vector<MapTrafficRouteRequirement>& route, std::size_t routeStart) {
    std::size_t timetableIndex = timetableStart;
    std::size_t routeIndex = routeStart;
    while (timetableIndex < timetablePath.size() && routeIndex < route.size()) {
        const MapTrafficRouteRequirement& timetableEntry = timetablePath[timetableIndex];
        const MapTrafficRouteRequirement& routeEntry = route[routeIndex];
        if (timetableEntry.splineId != routeEntry.splineId ||
            (routeEntry.sceneryPathIndex.has_value() &&
             timetableEntry.sceneryPathIndex != routeEntry.sceneryPathIndex)) {
            break;
        }
        const int matchedSplineId = timetableEntry.splineId;
        const std::optional<std::size_t> matchedPathIndex =
            routeEntry.sceneryPathIndex ? timetableEntry.sceneryPathIndex : std::nullopt;
        ++timetableIndex;
        while (timetableIndex < timetablePath.size() &&
               timetablePath[timetableIndex].splineId == matchedSplineId &&
               (!matchedPathIndex ||
                timetablePath[timetableIndex].sceneryPathIndex == matchedPathIndex)) {
            ++timetableIndex;
        }
        ++routeIndex;
    }
    return {routeIndex - routeStart, timetableIndex - timetableStart};
}

} // namespace

bool mapTrafficTimetablePathContainedInRoute(
    const std::vector<MapTrafficRouteRequirement>& timetablePath,
    const std::vector<MapTrafficRouteRequirement>& route) {
    if (timetablePath.size() < 2 || route.size() < 2) {
        return false;
    }
    for (std::size_t routeStart = 0; routeStart < route.size(); ++routeStart) {
        const TimetableRoutePrefixMatch match =
            matchedTimetablePathLength(timetablePath, 0, route, routeStart);
        if (match.timetableEntries == timetablePath.size() && match.routeSegments >= 2) {
            return true;
        }
    }
    return false;
}

bool mapTrafficTimetablePathContainedInRouteInOrder(
    const std::vector<MapTrafficRouteRequirement>& timetablePath,
    const std::vector<MapTrafficRouteRequirement>& route) {
    if (timetablePath.size() < 2 || route.size() < 2) {
        return false;
    }
    std::size_t timetableIndex = 0;
    for (const MapTrafficRouteRequirement& routeEntry : route) {
        if (timetableIndex >= timetablePath.size()) {
            break;
        }
        const MapTrafficRouteRequirement& timetableEntry = timetablePath[timetableIndex];
        if (timetableEntry.splineId != routeEntry.splineId ||
            (routeEntry.sceneryPathIndex.has_value() &&
             timetableEntry.sceneryPathIndex != routeEntry.sceneryPathIndex)) {
            continue;
        }
        const int matchedSplineId = timetableEntry.splineId;
        const std::optional<std::size_t> matchedPathIndex =
            routeEntry.sceneryPathIndex ? timetableEntry.sceneryPathIndex : std::nullopt;
        ++timetableIndex;
        while (timetableIndex < timetablePath.size() &&
               timetablePath[timetableIndex].splineId == matchedSplineId &&
               (!matchedPathIndex ||
                timetablePath[timetableIndex].sceneryPathIndex == matchedPathIndex)) {
            ++timetableIndex;
        }
    }
    return timetableIndex == timetablePath.size();
}

std::vector<MapTrafficRouteRequirement>
mapTrafficRoutePrefixMatchingTimetable(const std::vector<MapTrafficRouteRequirement>& timetablePath,
                                       const std::vector<MapTrafficRouteRequirement>& route) {
    return mapTrafficRoutePrefixMatch(timetablePath, route).routePrefix;
}

MapTrafficRoutePrefixMatch
mapTrafficRoutePrefixMatch(const std::vector<MapTrafficRouteRequirement>& timetablePath,
                           const std::vector<MapTrafficRouteRequirement>& route) {
    MapTrafficRoutePrefixMatch bestMatch;
    if (timetablePath.size() < 2 || route.size() < 2) {
        return bestMatch;
    }
    for (std::size_t timetableStart = 0; timetableStart < timetablePath.size(); ++timetableStart) {
        const TimetableRoutePrefixMatch match =
            matchedTimetablePathLength(timetablePath, timetableStart, route, 0);
        if (match.routeSegments > bestMatch.routePrefix.size()) {
            bestMatch.routePrefix.assign(
                route.begin(), route.begin() + static_cast<std::ptrdiff_t>(match.routeSegments));
            bestMatch.timetableStartIndex = timetableStart;
            bestMatch.timetableEntriesMatched = match.timetableEntries;
        }
    }
    if (bestMatch.routePrefix.size() < 2) {
        return {};
    }
    return bestMatch;
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
        throw std::invalid_argument(trafficInputError(
            "Map traffic spawn position or route is invalid",
            "startDistanceMeters=", startDistanceMeters, "centerX=", centerX, "centerY=", centerY,
            "maximumDistanceMeters=", maximumDistanceMeters,
            "endpointMarginMeters=", endpointMarginMeters,
            "routeTotalLength=", actor.route.totalLength, "segments=", actor.route.segments.size(),
            "cumulativeEndDistances=", actor.route.cumulativeEndDistances.size()));
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
        throw std::runtime_error(trafficInputError(
            "Could not sample the map traffic spawn route pose", "distanceMeters=", distance,
            "lateralOffsetMeters=", actor.lateralOffsetMeters,
            "routeTotalLength=", actor.route.totalLength));
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

namespace {

bool trafficRouteLooksLikeRoundabout(const MapTrafficRoute& route) {
    constexpr double minimumLengthMeters = 20.0;
    constexpr double maximumLengthMeters = 600.0;
    constexpr double minimumTurnRadians = 5.1;
    constexpr double maximumTurnRadians = 7.5;
    if (!route.closed || route.segments.empty() ||
        route.segments.size() != route.cumulativeEndDistances.size() ||
        !std::isfinite(route.totalLength) || route.totalLength < minimumLengthMeters ||
        route.totalLength > maximumLengthMeters) {
        return false;
    }

    const auto first = sampleMapTrafficRoute(route, 0.0, 0.0);
    if (!first) {
        return false;
    }
    double minX = first->position[0];
    double maxX = first->position[0];
    double minY = first->position[1];
    double maxY = first->position[1];
    double signedTurn = 0.0;
    double absoluteTurn = 0.0;
    double previousHeading = first->headingRadians;
    for (double distance = 1.0; distance < route.totalLength; distance += 1.0) {
        const auto sample = sampleMapTrafficRoute(route, distance, 0.0);
        if (!sample) {
            return false;
        }
        const double turn =
            std::remainder(sample->headingRadians - previousHeading, 2.0 * std::numbers::pi);
        signedTurn += turn;
        absoluteTurn += std::abs(turn);
        previousHeading = sample->headingRadians;
        minX = std::min(minX, sample->position[0]);
        maxX = std::max(maxX, sample->position[0]);
        minY = std::min(minY, sample->position[1]);
        maxY = std::max(maxY, sample->position[1]);
    }
    const auto last = sampleMapTrafficRoute(route, route.totalLength - 1.0e-4, 0.0);
    if (!last) {
        return false;
    }
    const double closingTurn =
        std::remainder(first->headingRadians - last->headingRadians, 2.0 * std::numbers::pi);
    signedTurn += closingTurn;
    absoluteTurn += std::abs(closingTurn);

    const double width = maxX - minX;
    const double height = maxY - minY;
    const double diagonal = std::hypot(width, height);
    const double smallerExtent = std::min(width, height);
    const double largerExtent = std::max(width, height);
    const double perimeterToDiagonal = route.totalLength / std::max(diagonal, 1.0e-6);
    return absoluteTurn > 1.0e-6 && std::abs(signedTurn) >= minimumTurnRadians &&
           std::abs(signedTurn) <= maximumTurnRadians &&
           std::abs(signedTurn) / absoluteTurn >= 0.8 && smallerExtent >= 5.0 &&
           largerExtent <= 200.0 && smallerExtent / largerExtent >= 0.55 &&
           perimeterToDiagonal >= 1.8 && perimeterToDiagonal <= 2.6;
}

void markRoundaboutRoute(MapTrafficRoute& route) {
    if (!trafficRouteLooksLikeRoundabout(route)) {
        return;
    }
    for (MapTrafficRouteSegment& segment : route.segments) {
        segment.pathRules.roundabout = true;
    }
}

void detectRoundaboutLanes(std::vector<MapTrafficRouteSegment>& segments,
                           const std::vector<double>& lengths,
                           const std::vector<std::vector<std::size_t>>& next,
                           const std::vector<MapTrafficPose>& starts,
                           const std::vector<MapTrafficPose>& ends) {
    constexpr std::size_t maximumCycleSegments = 64;
    constexpr double maximumCycleLengthMeters = 600.0;
    constexpr double minimumContinuationAlignment = 0.5;
    for (std::size_t start = 0; start < segments.size(); ++start) {
        if (segments[start].pathRules.roundabout) {
            continue;
        }
        std::vector<std::size_t> cycle;
        std::unordered_set<std::size_t> visited;
        std::size_t current = start;
        double cycleLength = 0.0;
        bool closed = false;
        for (std::size_t count = 0; count < maximumCycleSegments; ++count) {
            if (!visited.insert(current).second) {
                break;
            }
            cycle.push_back(current);
            cycleLength += lengths[current];
            if (cycleLength > maximumCycleLengthMeters || next[current].empty()) {
                break;
            }
            std::optional<std::size_t> continuation;
            double bestAlignment = minimumContinuationAlignment;
            for (const std::size_t candidate : next[current]) {
                const double alignment =
                    std::cos(ends[current].headingRadians - starts[candidate].headingRadians);
                if (alignment < bestAlignment ||
                    (visited.contains(candidate) && candidate != start)) {
                    continue;
                }
                if (!continuation.has_value() || alignment > bestAlignment) {
                    continuation = candidate;
                    bestAlignment = alignment;
                }
            }
            if (!continuation.has_value()) {
                break;
            }
            if (*continuation == start) {
                closed = true;
                break;
            }
            current = *continuation;
        }
        if (!closed) {
            continue;
        }

        MapTrafficRoute route;
        route.closed = true;
        route.segments.reserve(cycle.size());
        route.cumulativeEndDistances.reserve(cycle.size());
        for (const std::size_t lane : cycle) {
            route.segments.push_back(segments[lane]);
            route.totalLength += lengths[lane];
            route.cumulativeEndDistances.push_back(route.totalLength);
        }
        if (trafficRouteLooksLikeRoundabout(route)) {
            for (const std::size_t lane : cycle) {
                segments[lane].pathRules.roundabout = true;
            }
        }
    }
}

} // namespace

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
        route.segments.push_back({current, reversed, 0.0, 0.0, {}, {}, {}, {}, {}});
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
    markRoundaboutRoute(route);
    return route;
}

MapTrafficRoute subdivideMapTrafficRoute(const MapTrafficRoute& route) {
    constexpr double maximumPieceLengthMeters = 10.0;
    constexpr double mergeClearanceMeters = 5.0;
    constexpr double straightRadiusEpsilon = 1.0e-8;
    if (route.segments.size() != route.cumulativeEndDistances.size()) {
        throw std::invalid_argument(
            trafficInputError("Map traffic route segment distances are inconsistent",
                              "segments=", route.segments.size(),
                              "cumulativeEndDistances=", route.cumulativeEndDistances.size(),
                              "totalLength=", route.totalLength));
    }

    MapTrafficRoute subdivided = route;
    subdivided.segments.clear();
    subdivided.cumulativeEndDistances.clear();

    double segmentStart = 0.0;
    for (std::size_t segmentIndex = 0; segmentIndex < route.segments.size(); ++segmentIndex) {
        const double segmentEnd = route.cumulativeEndDistances[segmentIndex];
        const double segmentLength = segmentEnd - segmentStart;
        if (!std::isfinite(segmentEnd) || !std::isfinite(segmentLength) || segmentLength <= 0.0) {
            throw std::invalid_argument(
                trafficInputError("Map traffic route has an invalid segment distance",
                                  "segmentIndex=", segmentIndex, "segmentStart=", segmentStart,
                                  "segmentEnd=", segmentEnd, "segmentLength=", segmentLength));
        }

        const MapTrafficRouteSegment& source = route.segments[segmentIndex];
        const bool eligibleForSubdivision =
            source.points.empty() && !source.spline.placement.elevated &&
            std::isfinite(source.spline.placement.radius) &&
            std::abs(source.spline.placement.radius) <= straightRadiusEpsilon;
        std::vector<double> splitOffsets;
        if (eligibleForSubdivision) {
            for (double offset = maximumPieceLengthMeters;
                 offset + mergeClearanceMeters < segmentLength;
                 offset += maximumPieceLengthMeters) {
                splitOffsets.push_back(offset);
            }
        }

        const double sourceDistanceAtStart = source.splineDistanceAtRouteStartMeters.value_or(
            source.reversed ? source.spline.placement.length : 0.0);
        double pieceStartOffset = 0.0;
        for (std::size_t pieceIndex = 0; pieceIndex <= splitOffsets.size(); ++pieceIndex) {
            const double pieceEndOffset =
                pieceIndex == splitOffsets.size() ? segmentLength : splitOffsets[pieceIndex];
            MapTrafficRouteSegment piece = source;
            if (pieceIndex > 0) {
                piece.splineDistanceAtRouteStartMeters =
                    sourceDistanceAtStart +
                    (source.reversed ? -pieceStartOffset : pieceStartOffset);
                piece.trafficSignal.reset();
            }
            subdivided.segments.push_back(std::move(piece));
            subdivided.cumulativeEndDistances.push_back(segmentStart + pieceEndOffset);
            pieceStartOffset = pieceEndOffset;
        }
        segmentStart = segmentEnd;
    }
    return subdivided;
}

namespace {

double splineDistanceAtRouteStart(const MapTrafficRouteSegment& segment) {
    return segment.splineDistanceAtRouteStartMeters.value_or(
        segment.reversed ? segment.spline.placement.length : 0.0);
}

bool segmentsAreContinuousOnSameSpline(const MapTrafficRouteSegment& incoming,
                                       double incomingLength,
                                       const MapTrafficRouteSegment& outgoing) {
    if (!incoming.points.empty() || !outgoing.points.empty() ||
        incoming.spline.placement.splineId != outgoing.spline.placement.splineId ||
        incoming.spline.tileX != outgoing.spline.tileX ||
        incoming.spline.tileY != outgoing.spline.tileY || incoming.reversed != outgoing.reversed) {
        return false;
    }

    const double expectedStart = splineDistanceAtRouteStart(incoming) +
                                 (incoming.reversed ? -incomingLength : incomingLength);
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
    const double routeSegmentLength = route.cumulativeEndDistances[segmentIndex] - segmentStart;
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
                const double horizontalLength = std::hypot(dx, dy);
                const double pitch =
                    std::atan2(dz, horizontalLength > 1.0e-9 ? horizontalLength : length);
                return MapTrafficPose{{from[0] + dx * fraction + offset[0],
                                       from[1] + dy * fraction + offset[1],
                                       from[2] + dz * fraction},
                                      heading,
                                      spline.splineId,
                                      pitch,
                                      0.0};
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
    const auto transform =
        routeSegment.spline.coordinates.tile(routeSegment.spline.tileX, routeSegment.spline.tileY);
    const double tileOriginX = transform.originX;
    const double tileOriginY = transform.originY;
    const auto center = sampleMapSpline(tileOriginX + spline.localX, tileOriginY + spline.localY,
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
    const double skew = spline.skewStart + (spline.skewEnd - spline.skewStart) * progress;
    const double skewShift = splineLateral * skew;
    const double elevation = mapSplineElevation(
        spline.elevation, spline.length, spline.gradientStart, spline.gradientEnd,
        spline.elevated ? std::optional<double>(spline.heightDelta) : std::nullopt,
        boundedSplineDistance + skewShift);

    const auto sampleSurface = [&](double distance) -> std::optional<std::array<double, 3>> {
        const auto surfaceCenter =
            sampleMapSpline(tileOriginX + spline.localX, tileOriginY + spline.localY,
                            spline.rotationDegrees, spline.length, spline.radius, distance);
        if (!surfaceCenter.has_value()) {
            return std::nullopt;
        }
        const double surfaceProgress = distance / spline.length;
        const double surfaceCant =
            spline.cantStart + (spline.cantEnd - spline.cantStart) * surfaceProgress;
        const double surfaceSkew =
            spline.skewStart + (spline.skewEnd - spline.skewStart) * surfaceProgress;
        const double surfaceSkewShift = splineLateral * surfaceSkew;
        const double surfaceElevation = mapSplineElevation(
            spline.elevation, spline.length, spline.gradientStart, spline.gradientEnd,
            spline.elevated ? std::optional<double>(spline.heightDelta) : std::nullopt,
            distance + surfaceSkewShift);
        const auto surfaceOffset =
            mapSplineRightOffset(surfaceCenter->headingRadians, splineLateral);
        std::array<double, 3> point{surfaceCenter->x + surfaceOffset[0] +
                                        std::sin(surfaceCenter->headingRadians) * surfaceSkewShift,
                                    surfaceCenter->y + surfaceOffset[1] +
                                        std::cos(surfaceCenter->headingRadians) * surfaceSkewShift,
                                    surfaceElevation +
                                        mapSplineCantHeightOffset(splineLateral, surfaceCant) +
                                        routeSegment.pathHeightMeters};
        point[0] = tileOriginX + (point[0] - tileOriginX) * transform.scaleX;
        point[1] = tileOriginY + (point[1] - tileOriginY) * transform.scaleY;
        return point;
    };
    const double routeDirection = routeSegment.reversed ? -1.0 : 1.0;
    const double beforeDistance =
        std::clamp(boundedSplineDistance - routeDirection * 0.5, 0.0, spline.length);
    const double afterDistance =
        std::clamp(boundedSplineDistance + routeDirection * 0.5, 0.0, spline.length);
    const auto beforeSurface = sampleSurface(beforeDistance);
    const auto afterSurface = sampleSurface(afterDistance);
    double pitch = 0.0;
    if (beforeSurface.has_value() && afterSurface.has_value()) {
        const double horizontal = std::hypot((*afterSurface)[0] - (*beforeSurface)[0],
                                             (*afterSurface)[1] - (*beforeSurface)[1]);
        pitch = std::atan2((*afterSurface)[2] - (*beforeSurface)[2], horizontal);
    }
    const double bank = mapSplineCantBankRadians(splineLateral, cant);
    const double roll = routeSegment.reversed ? bank : -bank;

    return MapTrafficPose{
        {tileOriginX +
             (center->x - tileOriginX + offset[0] + std::sin(center->headingRadians) * skewShift) *
                 transform.scaleX,
         tileOriginY +
             (center->y - tileOriginY + offset[1] + std::cos(center->headingRadians) * skewShift) *
                 transform.scaleY,
         elevation + mapSplineCantHeightOffset(splineLateral, cant) +
             routeSegment.pathHeightMeters},
        std::atan2(std::sin(heading) * transform.scaleX, std::cos(heading) * transform.scaleY),
        spline.splineId,
        pitch,
        roll};
}

} // namespace

bool mapTrafficRouteSegmentClearsWater(const MapTrafficRouteSegment& segment,
                                       const WaterData& water, double tileSize,
                                       double clearanceMeters) {
    if (!std::isfinite(tileSize) || tileSize <= 0.0 || !std::isfinite(clearanceMeters) ||
        clearanceMeters < 0.0) {
        throw std::invalid_argument(trafficInputError("Water clearance inputs are invalid",
                                                      "tileSize=", tileSize,
                                                      "clearanceMeters=", clearanceMeters));
    }
    if (water.surfaces.empty()) {
        return true;
    }

    const double segmentLength = [&] {
        if (segment.points.empty()) {
            return segment.spline.placement.length;
        }
        double length = 0.0;
        for (std::size_t index = 1; index < segment.points.size(); ++index) {
            const auto& previous = segment.points[index - 1];
            const auto& current = segment.points[index];
            length += std::hypot(current[0] - previous[0], current[1] - previous[1],
                                 current[2] - previous[2]);
        }
        return length;
    }();
    if (!std::isfinite(segmentLength) || segmentLength <= 0.0) {
        return false;
    }

    MapTrafficRoute route;
    route.segments.push_back(segment);
    route.totalLength = segmentLength;
    route.cumulativeEndDistances.push_back(segmentLength);
    const MapTileTransform tile =
        segment.spline.coordinates.tile(segment.spline.tileX, segment.spline.tileY);
    const double tileWidth = tileSize * tile.scaleX;
    const double tileHeight = tileSize * tile.scaleY;
    constexpr double sampleSpacingMeters = 2.0;

    const auto clearsAt = [&](double distance) {
        const std::optional<MapTrafficPose> pose = sampleMapTrafficRouteRaw(route, distance, 0.0);
        if (!pose) {
            return false;
        }
        const double x = (pose->position[0] - tile.originX) / tileWidth;
        const double y = (pose->position[1] - tile.originY) / tileHeight;
        if (x < 0.0 || x > 1.0 || y < 0.0 || y > 1.0) {
            return true;
        }

        double waterHeight = -std::numeric_limits<double>::infinity();
        for (const WaterSurface& surface : water.surfaces) {
            const auto& h = surface.heights;
            if (std::any_of(h.begin(), h.end(),
                            [](float height) { return !std::isfinite(height); })) {
                throw std::invalid_argument("Water surface has a non-finite corner height");
            }
            const double height = x >= y ? h[0] * (1.0 - x) + h[1] * (x - y) + h[2] * y
                                         : h[0] * (1.0 - y) + h[2] * x + h[3] * (y - x);
            waterHeight = (std::max)(waterHeight, height);
        }
        return pose->position[2] >= waterHeight + clearanceMeters;
    };

    for (double distance = 0.0; distance < segmentLength; distance += sampleSpacingMeters) {
        if (!clearsAt(distance)) {
            return false;
        }
    }
    return clearsAt(segmentLength);
}

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
    const auto segmentLength = [&](std::size_t index) {
        const double start = index == 0 ? 0.0 : route.cumulativeEndDistances[index - 1];
        return route.cumulativeEndDistances[index] - start;
    };
    std::optional<Transition> selected;
    double selectedHalfWindow = 0.0;
    double transitionDistance = std::numeric_limits<double>::infinity();
    double queryDistance = routeDistance;
    const auto considerTransition = [&](const Transition& transition) {
        const double halfWindow =
            std::min({2.0, transition.incomingLength * 0.25, transition.outgoingLength * 0.25});
        if (halfWindow < 0.1) {
            return;
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
    };

    const double transitionSearchStart = std::max(0.0, routeDistance - 2.0);
    const double transitionSearchEnd = routeDistance + 2.0;
    auto boundary = std::lower_bound(route.cumulativeEndDistances.begin(),
                                     route.cumulativeEndDistances.end(), transitionSearchStart);
    for (std::size_t index =
             static_cast<std::size_t>(boundary - route.cumulativeEndDistances.begin());
         index + 1 < route.segments.size() &&
         route.cumulativeEndDistances[index] <= transitionSearchEnd;
         ++index) {
        if (route.segments[index].points.empty() && route.segments[index + 1].points.empty() &&
            !segmentsAreContinuousOnSameSpline(route.segments[index], segmentLength(index),
                                               route.segments[index + 1])) {
            considerTransition({route.cumulativeEndDistances[index], segmentLength(index),
                                segmentLength(index + 1), false});
        }
    }
    if ((route.closed || route.loopStartDistance.has_value()) &&
        route.segments.back().points.empty()) {
        std::size_t loopSegment = 0;
        if (route.loopStartDistance) {
            const auto end =
                std::lower_bound(route.cumulativeEndDistances.begin(),
                                 route.cumulativeEndDistances.end(), *route.loopStartDistance);
            loopSegment =
                end == route.cumulativeEndDistances.end()
                    ? route.segments.size() - 1
                    : static_cast<std::size_t>(end - route.cumulativeEndDistances.begin());
        }
        if (route.segments[loopSegment].points.empty()) {
            considerTransition({route.totalLength, segmentLength(route.segments.size() - 1),
                                segmentLength(loopSegment), true});
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

    const double t =
        std::clamp((queryDistance - beginDistance) / (endDistance - beginDistance), 0.0, 1.0);
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    const double h10 = t3 - 2.0 * t2 + t;
    const double h01 = -2.0 * t3 + 3.0 * t2;
    const double h11 = t3 - t2;
    const double tangentLength = endDistance - beginDistance;
    MapTrafficPose result = *rawPose;
    result.position[0] =
        h00 * beginPose->position[0] + h10 * tangentLength * std::sin(beginPose->headingRadians) +
        h01 * endPose->position[0] + h11 * tangentLength * std::sin(endPose->headingRadians);
    result.position[1] =
        h00 * beginPose->position[1] + h10 * tangentLength * std::cos(beginPose->headingRadians) +
        h01 * endPose->position[1] + h11 * tangentLength * std::cos(endPose->headingRadians);
    result.position[2] =
        beginPose->position[2] + (endPose->position[2] - beginPose->position[2]) * t;

    const double dh00 = 6.0 * t2 - 6.0 * t;
    const double dh10 = 3.0 * t2 - 4.0 * t + 1.0;
    const double dh01 = -6.0 * t2 + 6.0 * t;
    const double dh11 = 3.0 * t2 - 2.0 * t;
    const double tangentX =
        dh00 * beginPose->position[0] + dh10 * tangentLength * std::sin(beginPose->headingRadians) +
        dh01 * endPose->position[0] + dh11 * tangentLength * std::sin(endPose->headingRadians);
    const double tangentY =
        dh00 * beginPose->position[1] + dh10 * tangentLength * std::cos(beginPose->headingRadians) +
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

std::optional<MapTrafficRoutePosition>
mapTrafficRoutePositionOnSpline(const MapTrafficRoute& route, int splineId,
                                std::size_t sceneryPathIndex, double splinePositionMeters,
                                std::size_t firstSegmentIndex) {
    if (splineId == 0 || !std::isfinite(splinePositionMeters) || splinePositionMeters < 0.0 ||
        firstSegmentIndex > route.segments.size() ||
        route.cumulativeEndDistances.size() != route.segments.size()) {
        throw std::invalid_argument(trafficInputError(
            "Map traffic spline-position inputs are invalid", "splineId=", splineId,
            "sceneryPathIndex=", sceneryPathIndex, "splinePositionMeters=", splinePositionMeters,
            "firstSegmentIndex=", firstSegmentIndex, "segments=", route.segments.size(),
            "cumulativeEndDistances=", route.cumulativeEndDistances.size()));
    }
    constexpr double positionToleranceMeters = 1.0e-6;
    for (std::size_t segmentIndex = firstSegmentIndex; segmentIndex < route.segments.size();
         ++segmentIndex) {
        const MapTrafficRouteSegment& segment = route.segments[segmentIndex];
        if (segment.spline.placement.splineId != splineId ||
            segment.sceneryPathIndex != sceneryPathIndex) {
            continue;
        }
        const double routeSegmentStartMeters =
            segmentIndex == 0 ? 0.0 : route.cumulativeEndDistances[segmentIndex - 1];
        const double routeSegmentEndMeters = route.cumulativeEndDistances[segmentIndex];
        const double routeSegmentLengthMeters = routeSegmentEndMeters - routeSegmentStartMeters;
        const double splinePositionAtRouteStartMeters =
            segment.splineDistanceAtRouteStartMeters.value_or(
                segment.reversed ? segment.spline.placement.length : 0.0);
        const double routeOffsetMeters =
            segment.reversed ? splinePositionAtRouteStartMeters - splinePositionMeters
                             : splinePositionMeters - splinePositionAtRouteStartMeters;
        if (routeSegmentLengthMeters < 0.0 || !std::isfinite(routeSegmentLengthMeters) ||
            !std::isfinite(splinePositionAtRouteStartMeters) ||
            routeOffsetMeters < -positionToleranceMeters ||
            routeOffsetMeters > routeSegmentLengthMeters + positionToleranceMeters) {
            continue;
        }
        return MapTrafficRoutePosition{
            segmentIndex,
            routeSegmentStartMeters + std::clamp(routeOffsetMeters, 0.0, routeSegmentLengthMeters)};
    }
    return std::nullopt;
}

double mapTrafficRouteTravelHeadingRadians(const MapTrafficRoute& route, double distanceMeters,
                                           double lateralOffsetMeters) {
    if (!std::isfinite(distanceMeters) || !std::isfinite(lateralOffsetMeters) ||
        route.segments.empty() || !std::isfinite(route.totalLength) || route.totalLength <= 0.0) {
        throw std::invalid_argument(trafficInputError(
            "Map traffic travel heading inputs are invalid", "distanceMeters=", distanceMeters,
            "lateralOffsetMeters=", lateralOffsetMeters, "segments=", route.segments.size(),
            "totalLength=", route.totalLength));
    }
    const auto current = sampleMapTrafficRoute(route, distanceMeters, lateralOffsetMeters);
    if (!current.has_value()) {
        throw std::invalid_argument(trafficInputError(
            "Cannot sample map traffic travel heading", "distanceMeters=", distanceMeters,
            "lateralOffsetMeters=", lateralOffsetMeters, "totalLength=", route.totalLength));
    }
    constexpr double sampleOffsetMeters = 0.5;
    double fromDistance = distanceMeters - sampleOffsetMeters;
    double toDistance = distanceMeters + sampleOffsetMeters;
    if (!route.closed && !route.loopStartDistance.has_value()) {
        fromDistance = std::max(0.0, fromDistance);
        toDistance = std::min(route.totalLength, toDistance);
    }
    const auto from = sampleMapTrafficRoute(route, fromDistance, lateralOffsetMeters);
    const auto to = sampleMapTrafficRoute(route, toDistance, lateralOffsetMeters);
    if (!from.has_value() || !to.has_value()) {
        return current->headingRadians;
    }
    const double dx = to->position[0] - from->position[0];
    const double dy = to->position[1] - from->position[1];
    if (std::hypot(dx, dy) <= 1.0e-6) {
        return current->headingRadians;
    }
    return std::atan2(dx, dy);
}

std::optional<MapTrafficPose> sampleMapTrafficActorPose(const MapTrafficActor& actor,
                                                        double distanceMeters) {
    if (!std::isfinite(distanceMeters) || !std::isfinite(actor.rearAxleOffsetMeters)) {
        throw std::invalid_argument(trafficInputError(
            "Traffic actor pose inputs are invalid", "distanceMeters=", distanceMeters,
            "rearAxleOffsetMeters=", actor.rearAxleOffsetMeters));
    }
    const double axleDistance = distanceMeters + actor.rearAxleOffsetMeters;
    if (!std::isfinite(axleDistance)) {
        throw std::invalid_argument("Traffic rear axle route distance is not finite");
    }
    auto pose = sampleMapTrafficRoute(actor.route, axleDistance, actor.lateralOffsetMeters);
    if (!pose) {
        return std::nullopt;
    }
    if (actor.rearAxleOffsetMeters == 0.0) {
        return pose;
    }
    const double sampledDistance = !actor.route.closed && axleDistance < 0.0 ? 0.0
                                   : !actor.route.closed && !actor.route.loopStartDistance &&
                                           axleDistance > actor.route.totalLength
                                       ? actor.route.totalLength
                                       : axleDistance;
    pose->headingRadians = mapTrafficRouteTravelHeadingRadians(actor.route, sampledDistance,
                                                               actor.lateralOffsetMeters);
    const double extrapolation = axleDistance - sampledDistance;
    const double forwardOffset = extrapolation - actor.rearAxleOffsetMeters;
    const double horizontalOffset = forwardOffset * std::cos(pose->pitchRadians);
    pose->position[0] += horizontalOffset * std::sin(pose->headingRadians);
    pose->position[1] += horizontalOffset * std::cos(pose->headingRadians);
    pose->position[2] += forwardOffset * std::sin(pose->pitchRadians);
    return pose;
}

MapTrafficTurnSignal mapTrafficActorTurnSignal(const MapTrafficActor& actor,
                                               double activationDistanceMeters) {
    if (!std::isfinite(activationDistanceMeters) || activationDistanceMeters < 0.0) {
        throw std::invalid_argument(
            trafficInputError("Traffic turn-signal lookahead must be finite and non-negative",
                              "activationDistanceMeters=", activationDistanceMeters));
    }
    const MapTrafficRoute& route = actor.route;
    if (route.segments.empty() || route.segments.size() != route.cumulativeEndDistances.size() ||
        !std::isfinite(route.totalLength) || route.totalLength <= 0.0 ||
        !std::isfinite(actor.distanceMeters) || actor.distanceMeters < 0.0 ||
        (route.loopStartDistance.has_value() &&
         (!std::isfinite(*route.loopStartDistance) || *route.loopStartDistance < 0.0 ||
          *route.loopStartDistance >= route.totalLength))) {
        return MapTrafficTurnSignal::None;
    }
    double previousEnd = 0.0;
    for (const double segmentEnd : route.cumulativeEndDistances) {
        if (!std::isfinite(segmentEnd) || segmentEnd <= previousEnd ||
            segmentEnd > route.totalLength) {
            return MapTrafficTurnSignal::None;
        }
        previousEnd = segmentEnd;
    }
    if (std::abs(previousEnd - route.totalLength) > 1.0e-6) {
        return MapTrafficTurnSignal::None;
    }

    double currentDistance = actor.distanceMeters;
    const double loopStart = route.loopStartDistance.value_or(0.0);
    const double loopLength = route.totalLength - loopStart;
    if (route.closed) {
        currentDistance = std::fmod(currentDistance, route.totalLength);
    } else if (route.loopStartDistance.has_value() && currentDistance >= route.totalLength) {
        currentDistance = loopStart + std::fmod(currentDistance - loopStart, loopLength);
    } else if (currentDistance > route.totalLength) {
        return MapTrafficTurnSignal::None;
    }

    double nearestDistanceAhead = std::numeric_limits<double>::infinity();
    MapTrafficTurnSignal nearestSignal = MapTrafficTurnSignal::None;
    const auto considerDecision = [&](double decisionStart, double decisionEnd,
                                      MapTrafficTurnSignal signal) {
        if (signal == MapTrafficTurnSignal::None) {
            return;
        }
        double distanceAhead = decisionStart - currentDistance;
        const double clearDistance = std::max(0.0, actor.lengthMeters * 0.5);
        if (distanceAhead < 0.0 && currentDistance <= decisionEnd + clearDistance) {
            distanceAhead = 0.0;
        } else if (distanceAhead < 0.0 && (route.closed || route.loopStartDistance)) {
            distanceAhead += route.closed ? route.totalLength : loopLength;
        }
        if (distanceAhead >= 0.0 && distanceAhead <= activationDistanceMeters &&
            distanceAhead < nearestDistanceAhead) {
            nearestDistanceAhead = distanceAhead;
            nearestSignal = signal;
        }
    };
    const auto turnForHeadingChange = [](double from, double to, double minimumAngle) {
        const double difference = std::remainder(to - from, 2.0 * std::numbers::pi);
        if (std::abs(difference) < minimumAngle) {
            return MapTrafficTurnSignal::None;
        }
        return difference > 0.0 ? MapTrafficTurnSignal::Right : MapTrafficTurnSignal::Left;
    };
    const auto segmentStart = [&](std::size_t index) {
        return index == 0 ? 0.0 : route.cumulativeEndDistances[index - 1];
    };
    const auto directionAt = [&](double distance) -> std::optional<MapTrafficPose> {
        return sampleMapTrafficRouteRaw(route, distance, actor.lateralOffsetMeters);
    };
    constexpr double junctionMinimumTurnRadians = 5.0 * std::numbers::pi / 180.0;
    constexpr double roadTurnThresholdRadians = 30.0 * std::numbers::pi / 180.0;
    constexpr double sampleOffsetMeters = 0.01;

    for (std::size_t index = 0; index < route.segments.size();) {
        if (route.segments[index].points.empty()) {
            ++index;
            continue;
        }
        std::size_t lastJunctionSegment = index;
        while (lastJunctionSegment + 1 < route.segments.size() &&
               !route.segments[lastJunctionSegment + 1].points.empty()) {
            ++lastJunctionSegment;
        }
        if (index > 0 && lastJunctionSegment + 1 < route.segments.size()) {
            const double junctionStart = segmentStart(index);
            const double junctionEnd = route.cumulativeEndDistances[lastJunctionSegment];
            const auto incoming = directionAt(junctionStart - sampleOffsetMeters);
            const auto outgoing = directionAt(junctionEnd + sampleOffsetMeters);
            if (incoming && outgoing) {
                considerDecision(junctionStart, junctionEnd,
                                 turnForHeadingChange(incoming->headingRadians,
                                                      outgoing->headingRadians,
                                                      junctionMinimumTurnRadians));
            }
        }
        index = lastJunctionSegment + 1;
    }

    for (std::size_t index = 0; index + 1 < route.segments.size(); ++index) {
        const MapTrafficRouteSegment& incomingSegment = route.segments[index];
        const MapTrafficRouteSegment& outgoingSegment = route.segments[index + 1];
        if (!incomingSegment.points.empty() || !outgoingSegment.points.empty()) {
            continue;
        }
        const double boundary = route.cumulativeEndDistances[index];
        const double lateralChange =
            outgoingSegment.pathLateralMeters - incomingSegment.pathLateralMeters;
        if (incomingSegment.spline.placement.splineId ==
                outgoingSegment.spline.placement.splineId &&
            incomingSegment.spline.tileX == outgoingSegment.spline.tileX &&
            incomingSegment.spline.tileY == outgoingSegment.spline.tileY &&
            incomingSegment.reversed == outgoingSegment.reversed &&
            std::abs(lateralChange) >= 1.0) {
            const double travelRelativeLateralChange =
                incomingSegment.reversed ? -lateralChange : lateralChange;
            considerDecision(boundary, boundary,
                             travelRelativeLateralChange > 0.0 ? MapTrafficTurnSignal::Right
                                                               : MapTrafficTurnSignal::Left);
            continue;
        }
        const double incomingLength = route.cumulativeEndDistances[index] - segmentStart(index);
        if (segmentsAreContinuousOnSameSpline(incomingSegment, incomingLength, outgoingSegment)) {
            continue;
        }
        const auto incoming = directionAt(boundary - sampleOffsetMeters);
        const auto outgoing = directionAt(boundary + sampleOffsetMeters);
        if (incoming && outgoing) {
            considerDecision(boundary, boundary,
                             turnForHeadingChange(incoming->headingRadians,
                                                  outgoing->headingRadians,
                                                  roadTurnThresholdRadians));
        }
    }
    return nearestSignal;
}

std::vector<MapTrafficSpawnCandidate> mapTrafficRouteSpawnCandidates(const MapTrafficRoute& route,
                                                                     double vehicleLengthMeters) {
    if (!std::isfinite(vehicleLengthMeters) || vehicleLengthMeters <= 0.0) {
        throw std::invalid_argument(
            trafficInputError("Traffic spawn vehicle length must be positive and finite",
                              "vehicleLengthMeters=", vehicleLengthMeters));
    }
    if (route.segments.size() != route.cumulativeEndDistances.size()) {
        throw std::invalid_argument(
            trafficInputError("Traffic spawn route has inconsistent segment distances",
                              "segments=", route.segments.size(),
                              "cumulativeEndDistances=", route.cumulativeEndDistances.size(),
                              "totalLength=", route.totalLength));
    }
    const double endpointMargin = std::max(8.0, vehicleLengthMeters * 0.5 + 4.0);
    const double spacing = std::max(12.0, vehicleLengthMeters + 6.0);
    std::vector<MapTrafficSpawnCandidate> candidates;
    const auto appendSpanCandidates = [&](std::size_t segmentIndex, double spanStart,
                                          double spanEnd) {
        if (!std::isfinite(spanStart) || !std::isfinite(spanEnd) || spanEnd <= spanStart) {
            throw std::invalid_argument(trafficInputError(
                "Traffic spawn route has invalid segment distances", "segmentIndex=", segmentIndex,
                "spanStart=", spanStart, "spanEnd=", spanEnd));
        }
        for (double distance = spanStart + endpointMargin; distance <= spanEnd - endpointMargin;
             distance += spacing) {
            const auto pose = sampleMapTrafficRoute(route, distance, 0.0);
            if (!pose) {
                throw std::runtime_error("Could not sample a map traffic spawn candidate position");
            }
            const double tileSize = route.segments[segmentIndex].spline.coordinates.tileSize;
            candidates.push_back({distance, mapWorldTile(pose->position[0], tileSize),
                                  mapWorldTile(pose->position[1], tileSize)});
        }
    };
    for (std::size_t index = 0; index < route.segments.size();) {
        const MapTrafficRouteSegment& segment = route.segments[index];
        if (segment.spline.placement.splineId == 0) {
            ++index;
            continue;
        }

        const double segmentStart = index == 0 ? 0.0 : route.cumulativeEndDistances[index - 1];
        const double segmentEnd = route.cumulativeEndDistances[index];
        if (!segment.points.empty()) {
            const bool isOpenRouteEnd =
                index + 1 == route.segments.size() && !route.closed && !route.loopStartDistance;
            if (!isOpenRouteEnd && !segment.trafficSignal.has_value()) {
                appendSpanCandidates(index, segmentStart, segmentEnd);
            }
            ++index;
            continue;
        }

        std::size_t spanEndIndex = index;
        while (spanEndIndex + 1 < route.segments.size()) {
            const double currentStart =
                spanEndIndex == 0 ? 0.0 : route.cumulativeEndDistances[spanEndIndex - 1];
            const double currentLength = route.cumulativeEndDistances[spanEndIndex] - currentStart;
            if (!segmentsAreContinuousOnSameSpline(route.segments[spanEndIndex], currentLength,
                                                   route.segments[spanEndIndex + 1])) {
                break;
            }
            ++spanEndIndex;
        }

        const bool isOpenRouteEnd =
            spanEndIndex + 1 == route.segments.size() && !route.closed && !route.loopStartDistance;
        const bool signalizedSpan =
            std::any_of(route.segments.begin() + static_cast<std::ptrdiff_t>(index),
                        route.segments.begin() + static_cast<std::ptrdiff_t>(spanEndIndex + 1),
                        [](const MapTrafficRouteSegment& spanSegment) {
                            return spanSegment.trafficSignal.has_value();
                        });
        if (!isOpenRouteEnd && !signalizedSpan) {
            const double spanStart = index == 0 ? 0.0 : route.cumulativeEndDistances[index - 1];
            const double spanEnd = route.cumulativeEndDistances[spanEndIndex];
            appendSpanCandidates(index, spanStart, spanEnd);
        }
        index = spanEndIndex + 1;
    }
    return candidates;
}

bool mapTrafficSpawnCandidatesOverlapActiveTiles(
    const std::vector<MapTrafficSpawnCandidate>& candidates,
    const std::vector<std::pair<int, int>>& activeTiles) {
    return std::any_of(
        candidates.begin(), candidates.end(), [&](const MapTrafficSpawnCandidate& candidate) {
            return std::find(activeTiles.begin(), activeTiles.end(),
                             std::pair{candidate.tileX, candidate.tileY}) != activeTiles.end();
        });
}

bool advanceMapTrafficActor(MapTrafficActor& actor, double elapsedSeconds) {
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0 ||
        !std::isfinite(actor.speedMetersPerSecond) || actor.speedMetersPerSecond < 0.0 ||
        !std::isfinite(actor.distanceMeters) || !std::isfinite(actor.route.totalLength) ||
        actor.route.totalLength <= 0.0) {
        throw std::invalid_argument(trafficInputError(
            "Map traffic actor has invalid time, speed, or route state",
            "elapsedSeconds=", elapsedSeconds, "speedMetersPerSecond=", actor.speedMetersPerSecond,
            "distanceMeters=", actor.distanceMeters, "totalLength=", actor.route.totalLength));
    }

    const double nextDistance = actor.distanceMeters + actor.speedMetersPerSecond * elapsedSeconds;
    if (!std::isfinite(nextDistance)) {
        throw std::invalid_argument(trafficInputError(
            "Map traffic actor distance overflowed", "distanceMeters=", actor.distanceMeters,
            "speedMetersPerSecond=", actor.speedMetersPerSecond, "elapsedSeconds=", elapsedSeconds,
            "nextDistance=", nextDistance));
    }
    if (actor.route.closed) {
        actor.distanceMeters = std::fmod(nextDistance, actor.route.totalLength);
        return true;
    }
    if (actor.route.loopStartDistance && nextDistance >= actor.route.totalLength) {
        const double loopStart = *actor.route.loopStartDistance;
        const double loopLength = actor.route.totalLength - loopStart;
        if (!std::isfinite(loopStart) || loopStart < 0.0 || loopLength <= 0.0) {
            throw std::invalid_argument(trafficInputError(
                "Map traffic route has an invalid loop start", "loopStart=", loopStart,
                "totalLength=", actor.route.totalLength, "loopLength=", loopLength));
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
        throw std::invalid_argument(
            trafficInputError("Map traffic update time must be finite and non-negative",
                              "elapsedSeconds=", elapsedSeconds));
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
                                            int tileX, int tileY,
                                            const MapCoordinates& coordinates) {
    if (finalized_) {
        throw std::logic_error(trafficInputError(
            "Cannot add splines to a finalized traffic route network", "finalized=", finalized_,
            "splineCount=", splines.size(), "tileX=", tileX, "tileY=", tileY));
    }
    for (const MapSplinePlacement& spline : splines) {
        if (spline.splineId == 0 || ambiguousSplineIds_.contains(spline.splineId)) {
            continue;
        }
        const auto [entry, inserted] = splinesById_.emplace(
            spline.splineId, MapSplinePlacementReference{spline, tileX, tileY, coordinates});
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
        throw std::logic_error(trafficInputError(
            "Traffic route network must be finalized before route lookup", "finalized=", finalized_,
            "splineId=", splineId, "segmentLimit=", segmentLimit));
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
        throw std::logic_error(
            trafficInputError("Traffic route network must be finalized before listing splines",
                              "finalized=", finalized_));
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

std::vector<int>
MapTrafficRouteNetwork::splineIdsInTiles(const std::vector<std::pair<int, int>>& tiles) const {
    if (!finalized_) {
        throw std::logic_error(
            trafficInputError("Traffic route network must be finalized before listing splines",
                              "finalized=", finalized_));
    }
    std::vector<int> ids;
    for (const auto& [splineId, spline] : splinesById_) {
        if (std::find(tiles.begin(), tiles.end(), std::pair{spline.tileX, spline.tileY}) !=
            tiles.end()) {
            ids.push_back(splineId);
        }
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

std::vector<int> MapTrafficRouteNetwork::routeStartSplineIds() const {
    if (!finalized_) {
        throw std::logic_error(
            trafficInputError("Traffic route network must be finalized before listing route starts",
                              "finalized=", finalized_));
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
        throw std::logic_error(
            trafficInputError("Traffic route network must be finalized before endpoint lookup",
                              "finalized=", finalized_, "splineId=", splineId));
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
    double length = segment.spline.placement.length;
    if (segment.points.empty() && !segment.spline.placement.geometryValid) {
        throw std::invalid_argument(trafficInputError(
            "Traffic lane has invalid spline geometry",
            "splineId=", segment.spline.placement.splineId,
            "geometryValid=", segment.spline.placement.geometryValid, "length=", length));
    }
    if (!segment.points.empty()) {
        length = 0.0;
        for (std::size_t pointIndex = 0; pointIndex < segment.points.size(); ++pointIndex) {
            const auto& point = segment.points[pointIndex];
            if (!std::all_of(point.begin(), point.end(),
                             [](double value) { return std::isfinite(value); })) {
                throw std::invalid_argument(trafficInputError(
                    "Traffic lane has non-finite points", "pointIndex=", pointIndex, "x=", point[0],
                    "y=", point[1], "z=", point[2]));
            }
        }
        for (std::size_t index = 1; index < segment.points.size(); ++index) {
            const auto& from = segment.points[index - 1];
            const auto& to = segment.points[index];
            length += std::hypot(to[0] - from[0], to[1] - from[1], to[2] - from[2]);
        }
    }
    if (!std::isfinite(length) || length <= 0.0) {
        throw std::invalid_argument(trafficInputError(
            "Traffic lane has invalid geometry", "splineId=", segment.spline.placement.splineId,
            "length=", length, "pointCount=", segment.points.size()));
    }
    lengths_.push_back(length);
    segments_.push_back(std::move(segment));
    finalized_ = false;
}

void MapTrafficLaneNetwork::finalize() {
    if (finalized_) {
        throw std::logic_error(trafficInputError(
            "Traffic lane network is already finalized", "finalized=", finalized_,
            "segmentCount=", segments_.size(), "finalizedSegmentCount=", finalizedSegmentCount_));
    }
    if (finalizedSegmentCount_ != 0) {
        finalizeAppendedSegments();
        return;
    }
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
            throw std::runtime_error(trafficInputError(
                "Cannot sample traffic lane endpoints", "segmentIndex=", index,
                "totalLength=", single.totalLength, "startSampled=", start.has_value(),
                "endSampled=", end.has_value()));
        }
        laneStarts_.push_back(*start);
        laneEnds_.push_back(*end);
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
    laneStartCells_ = starts;
    for (std::size_t index = 0; index < laneEnds_.size(); ++index) {
        laneEndCells_[cellFor(laneEnds_[index].position)].push_back(index);
    }
    next_.resize(segments_.size());
    incoming_.resize(segments_.size());
    for (std::size_t index = 0; index < segments_.size(); ++index) {
        const auto& end = laneEnds_[index];
        const Cell cell = cellFor(end.position);
        for (long long x = -1; x <= 1; ++x) {
            for (long long y = -1; y <= 1; ++y) {
                const auto bucket = starts.find({cell.first + x, cell.second + y});
                if (bucket == starts.end()) {
                    continue;
                }
                for (const std::size_t candidate : bucket->second) {
                    const auto& start = laneStarts_[candidate];
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
        std::stable_sort(next_[index].begin(), next_[index].end(),
                         [&](std::size_t a, std::size_t b) {
                             return std::cos(end.headingRadians - laneStarts_[a].headingRadians) >
                                    std::cos(end.headingRadians - laneStarts_[b].headingRadians);
                         });
    }
    const std::size_t laneCount = segments_.size();
    const auto orientedStart = [&](std::size_t orientedLane) {
        if (orientedLane < laneCount) {
            return laneStarts_[orientedLane];
        }
        MapTrafficPose pose = laneEnds_[orientedLane - laneCount];
        pose.headingRadians += std::numbers::pi;
        return pose;
    };
    const auto orientedEnd = [&](std::size_t orientedLane) {
        if (orientedLane < laneCount) {
            return laneEnds_[orientedLane];
        }
        MapTrafficPose pose = laneStarts_[orientedLane - laneCount];
        pose.headingRadians += std::numbers::pi;
        return pose;
    };
    std::map<Cell, std::vector<std::size_t>> timetableStarts;
    timetableNext_.resize(laneCount * 2);
    for (std::size_t orientedLane = 0; orientedLane < laneCount * 2; ++orientedLane) {
        timetableStarts[cellFor(orientedStart(orientedLane).position)].push_back(orientedLane);
    }
    timetableStartCells_ = timetableStarts;
    for (std::size_t orientedLane = 0; orientedLane < laneCount * 2; ++orientedLane) {
        const MapTrafficPose end = orientedEnd(orientedLane);
        const Cell cell = cellFor(end.position);
        for (long long x = -1; x <= 1; ++x) {
            for (long long y = -1; y <= 1; ++y) {
                const auto bucket = timetableStarts.find({cell.first + x, cell.second + y});
                if (bucket == timetableStarts.end()) {
                    continue;
                }
                for (const std::size_t candidate : bucket->second) {
                    const MapTrafficPose start = orientedStart(candidate);
                    const double horizontal = std::hypot(end.position[0] - start.position[0],
                                                         end.position[1] - start.position[1]);
                    if (horizontal > tolerance ||
                        std::abs(end.position[2] - start.position[2]) > tolerance ||
                        std::cos(end.headingRadians - start.headingRadians) < -0.01) {
                        continue;
                    }
                    timetableNext_[orientedLane].push_back(candidate);
                }
            }
        }
        std::stable_sort(timetableNext_[orientedLane].begin(), timetableNext_[orientedLane].end(),
                         [&](std::size_t a, std::size_t b) {
                             return std::cos(end.headingRadians - orientedStart(a).headingRadians) >
                                    std::cos(end.headingRadians - orientedStart(b).headingRadians);
                         });
    }
    // Report open ends that nearly meet an aligned lane start. These usually indicate
    // authoring gaps or height mismatches rather than intentional dead ends.
    constexpr double nearMissMeters = 3.0;
    stats_ = {segments_.size(), 0, 0};
    nearMissByLane_.assign(segments_.size(), false);
    for (std::size_t index = 0; index < segments_.size(); ++index) {
        if (!next_[index].empty()) {
            continue;
        }
        ++stats_.deadEnds;
        const auto& end = laneEnds_[index];
        const Cell cell = cellFor(end.position);
        bool hasNearMiss = false;
        constexpr long long searchCells = static_cast<long long>(nearMissMeters / tolerance) + 1;
        for (long long x = -searchCells; x <= searchCells && !hasNearMiss; ++x) {
            for (long long y = -searchCells; y <= searchCells && !hasNearMiss; ++y) {
                const auto bucket = laneStartCells_.find({cell.first + x, cell.second + y});
                if (bucket == laneStartCells_.end()) {
                    continue;
                }
                for (const std::size_t candidate : bucket->second) {
                    const auto& start = laneStarts_[candidate];
                    if (candidate != index &&
                        std::hypot(end.position[0] - start.position[0],
                                   end.position[1] - start.position[1]) <= nearMissMeters &&
                        std::abs(end.position[2] - start.position[2]) <= nearMissMeters &&
                        std::cos(end.headingRadians - start.headingRadians) > 0.5) {
                        hasNearMiss = true;
                        break;
                    }
                }
            }
        }
        if (hasNearMiss) {
            nearMissByLane_[index] = true;
            ++stats_.nearMissDeadEnds;
        }
    }
    for (const auto& [splineId, indexes] : lanesBySpline_) {
        for (std::size_t laneSelector = 0; laneSelector < indexes.size(); ++laneSelector) {
            const std::size_t index = indexes[laneSelector];
            if (!next_[index].empty()) {
                spawnLanes_.push_back({splineId, laneSelector});
            }
        }
    }
    std::sort(spawnLanes_.begin(), spawnLanes_.end(), [](const auto& a, const auto& b) {
        return std::pair{a.splineId, a.laneSelector} < std::pair{b.splineId, b.laneSelector};
    });
    detectRoundaboutLanes(segments_, lengths_, next_, laneStarts_, laneEnds_);
    finalizedSegmentCount_ = segments_.size();
    finalized_ = true;
}

void MapTrafficLaneNetwork::finalizeAppendedSegments() {
    constexpr double tolerance = 0.5;
    constexpr double nearMissMeters = 3.0;
    constexpr long long nearMissSearchCells =
        static_cast<long long>(nearMissMeters / tolerance) + 1;
    const auto cellFor = [](const std::array<double, 3>& position) -> Cell {
        return {static_cast<long long>(std::floor(position[0] / tolerance)),
                static_cast<long long>(std::floor(position[1] / tolerance))};
    };
    const std::size_t firstNewSegment = finalizedSegmentCount_;
    const std::size_t segmentCount = segments_.size();
    const std::size_t orientedLaneCount = segmentCount * 2;
    next_.resize(segmentCount);
    incoming_.resize(segmentCount, 0);
    timetableNext_.assign(orientedLaneCount, {});
    std::vector<bool> touchedOutgoing(segmentCount, false);
    std::vector<bool> touchedTimetableOutgoing(orientedLaneCount, false);

    for (std::size_t index = firstNewSegment; index < segmentCount; ++index) {
        MapTrafficRoute single;
        single.segments.push_back(segments_[index]);
        single.totalLength = lengths_[index];
        single.cumulativeEndDistances.push_back(single.totalLength);
        const auto start = sampleMapTrafficRoute(single, 0.0, 0.0);
        const auto end = sampleMapTrafficRoute(single, single.totalLength, 0.0);
        if (!start || !end) {
            throw std::runtime_error(trafficInputError(
                "Cannot sample traffic lane endpoints", "segmentIndex=", index,
                "totalLength=", single.totalLength, "startSampled=", start.has_value(),
                "endSampled=", end.has_value()));
        }
        laneStarts_.push_back(*start);
        laneEnds_.push_back(*end);
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
        laneStartCells_[cellFor(start->position)].push_back(index);
        laneEndCells_[cellFor(end->position)].push_back(index);
        if (segments_[index].spline.placement.splineId != 0) {
            lanesBySpline_[segments_[index].spline.placement.splineId].push_back(index);
        }
    }

    const auto addConnection = [&](std::size_t from, std::size_t to) {
        const MapTrafficPose& end = laneEnds_[from];
        const MapTrafficPose& start = laneStarts_[to];
        const double horizontal =
            std::hypot(end.position[0] - start.position[0], end.position[1] - start.position[1]);
        if ((from == to &&
             (horizontal > 1.0e-6 || std::abs(end.position[2] - start.position[2]) > 1.0e-6)) ||
            horizontal > tolerance || std::abs(end.position[2] - start.position[2]) > tolerance ||
            std::cos(end.headingRadians - start.headingRadians) < -0.01) {
            return;
        }
        next_[from].push_back(to);
        ++incoming_[to];
        touchedOutgoing[from] = true;
    };
    const auto forNearbyCells = [](const Cell& cell, long long radius, const auto& callback) {
        for (long long x = -radius; x <= radius; ++x) {
            for (long long y = -radius; y <= radius; ++y) {
                callback(Cell{cell.first + x, cell.second + y});
            }
        }
    };
    for (std::size_t from = firstNewSegment; from < segmentCount; ++from) {
        forNearbyCells(cellFor(laneEnds_[from].position), 1, [&](const Cell& cell) {
            const auto bucket = laneStartCells_.find(cell);
            if (bucket == laneStartCells_.end()) {
                return;
            }
            for (const std::size_t to : bucket->second) {
                addConnection(from, to);
            }
        });
    }
    for (std::size_t to = firstNewSegment; to < segmentCount; ++to) {
        forNearbyCells(cellFor(laneStarts_[to].position), 1, [&](const Cell& cell) {
            const auto bucket = laneEndCells_.find(cell);
            if (bucket == laneEndCells_.end()) {
                return;
            }
            for (const std::size_t from : bucket->second) {
                if (from < firstNewSegment) {
                    addConnection(from, to);
                }
            }
        });
    }
    for (std::size_t from = 0; from < segmentCount; ++from) {
        if (!touchedOutgoing[from]) {
            continue;
        }
        const MapTrafficPose& end = laneEnds_[from];
        std::stable_sort(
            next_[from].begin(), next_[from].end(), [&](std::size_t first, std::size_t second) {
                return std::cos(end.headingRadians - laneStarts_[first].headingRadians) >
                       std::cos(end.headingRadians - laneStarts_[second].headingRadians);
            });
    }

    const auto orientedStart = [&](std::size_t orientedLane) {
        if (orientedLane < segmentCount) {
            return laneStarts_[orientedLane];
        }
        MapTrafficPose pose = laneEnds_[orientedLane - segmentCount];
        pose.headingRadians += std::numbers::pi;
        return pose;
    };
    const auto orientedEnd = [&](std::size_t orientedLane) {
        if (orientedLane < segmentCount) {
            return laneEnds_[orientedLane];
        }
        MapTrafficPose pose = laneStarts_[orientedLane - segmentCount];
        pose.headingRadians += std::numbers::pi;
        return pose;
    };
    timetableStartCells_.clear();
    for (std::size_t orientedLane = 0; orientedLane < orientedLaneCount; ++orientedLane) {
        timetableStartCells_[cellFor(orientedStart(orientedLane).position)].push_back(orientedLane);
    }
    const auto addTimetableConnection = [&](std::size_t from, std::size_t to) {
        const MapTrafficPose end = orientedEnd(from);
        const MapTrafficPose start = orientedStart(to);
        const double horizontal =
            std::hypot(end.position[0] - start.position[0], end.position[1] - start.position[1]);
        if (horizontal > tolerance || std::abs(end.position[2] - start.position[2]) > tolerance ||
            std::cos(end.headingRadians - start.headingRadians) < -0.01) {
            return;
        }
        timetableNext_[from].push_back(to);
        touchedTimetableOutgoing[from] = true;
    };
    for (std::size_t from = 0; from < orientedLaneCount; ++from) {
        const MapTrafficPose end = orientedEnd(from);
        forNearbyCells(cellFor(end.position), 1, [&](const Cell& cell) {
            const auto bucket = timetableStartCells_.find(cell);
            if (bucket == timetableStartCells_.end()) {
                return;
            }
            for (const std::size_t to : bucket->second) {
                addTimetableConnection(from, to);
            }
        });
    }
    for (std::size_t from = 0; from < orientedLaneCount; ++from) {
        if (!touchedTimetableOutgoing[from]) {
            continue;
        }
        const MapTrafficPose end = orientedEnd(from);
        std::stable_sort(
            timetableNext_[from].begin(), timetableNext_[from].end(),
            [&](std::size_t first, std::size_t second) {
                return std::cos(end.headingRadians - orientedStart(first).headingRadians) >
                       std::cos(end.headingRadians - orientedStart(second).headingRadians);
            });
    }

    std::vector<bool> nearMissCandidates(segmentCount, false);
    for (std::size_t index = firstNewSegment; index < segmentCount; ++index) {
        nearMissCandidates[index] = true;
        forNearbyCells(cellFor(laneStarts_[index].position), nearMissSearchCells,
                       [&](const Cell& cell) {
                           const auto bucket = laneEndCells_.find(cell);
                           if (bucket == laneEndCells_.end()) {
                               return;
                           }
                           for (const std::size_t from : bucket->second) {
                               if (from < firstNewSegment) {
                                   nearMissCandidates[from] = true;
                               }
                           }
                       });
    }
    for (std::size_t index = 0; index < segmentCount; ++index) {
        if (touchedOutgoing[index]) {
            nearMissCandidates[index] = true;
        }
    }
    nearMissByLane_.resize(segmentCount, false);
    for (std::size_t index = 0; index < segmentCount; ++index) {
        if (!nearMissCandidates[index]) {
            continue;
        }
        nearMissByLane_[index] = false;
        if (!next_[index].empty()) {
            continue;
        }
        const MapTrafficPose& end = laneEnds_[index];
        forNearbyCells(cellFor(end.position), nearMissSearchCells, [&](const Cell& cell) {
            if (nearMissByLane_[index]) {
                return;
            }
            const auto bucket = laneStartCells_.find(cell);
            if (bucket == laneStartCells_.end()) {
                return;
            }
            for (const std::size_t candidate : bucket->second) {
                const MapTrafficPose& start = laneStarts_[candidate];
                if (candidate != index &&
                    std::hypot(end.position[0] - start.position[0],
                               end.position[1] - start.position[1]) <= nearMissMeters &&
                    std::abs(end.position[2] - start.position[2]) <= nearMissMeters &&
                    std::cos(end.headingRadians - start.headingRadians) > 0.5) {
                    nearMissByLane_[index] = true;
                    break;
                }
            }
        });
    }
    stats_ = {segmentCount, 0, 0};
    for (std::size_t index = 0; index < segmentCount; ++index) {
        if (next_[index].empty()) {
            ++stats_.deadEnds;
            stats_.nearMissDeadEnds += nearMissByLane_[index] ? 1U : 0U;
        }
    }
    spawnLanes_.clear();
    for (const auto& [splineId, indexes] : lanesBySpline_) {
        for (std::size_t laneSelector = 0; laneSelector < indexes.size(); ++laneSelector) {
            const std::size_t index = indexes[laneSelector];
            if (!next_[index].empty()) {
                spawnLanes_.push_back({splineId, laneSelector});
            }
        }
    }
    std::sort(spawnLanes_.begin(), spawnLanes_.end(), [](const auto& a, const auto& b) {
        return std::pair{a.splineId, a.laneSelector} < std::pair{b.splineId, b.laneSelector};
    });
    detectRoundaboutLanes(segments_, lengths_, next_, laneStarts_, laneEnds_);
    finalizedSegmentCount_ = segmentCount;
    finalized_ = true;
}

MapTrafficLaneNetworkStats MapTrafficLaneNetwork::stats() const {
    if (!finalized_) {
        throw std::logic_error(
            trafficInputError("Traffic lane network must be finalized before reading stats",
                              "finalized=", finalized_, "segmentCount=", segments_.size()));
    }
    return stats_;
}

std::size_t MapTrafficLaneNetwork::laneCount(int splineId) const {
    if (!finalized_) {
        throw std::logic_error(
            trafficInputError("Traffic lane network must be finalized before lane lookup",
                              "finalized=", finalized_, "splineId=", splineId));
    }
    const auto lanes = lanesBySpline_.find(splineId);
    return lanes == lanesBySpline_.end() ? 0 : lanes->second.size();
}

std::optional<MapTrafficRoute> MapTrafficLaneNetwork::buildRoute(
    int splineId, std::size_t laneSelector, std::size_t routeSeed, double minimumTurnRadiusMeters,
    MapTrafficVehicleFilter vehicle, const std::vector<int>& requiredSplineSequence,
    const std::vector<std::optional<std::size_t>>& requiredSceneryPathSequence,
    bool stopAtRequiredSequenceEnd, bool allowIntermediarySegmentsBetweenRequired,
    bool allowReverseTraversalForRequired) const {
    if (!finalized_) {
        throw std::logic_error(trafficInputError(
            "Traffic lane network must be finalized before route lookup", "finalized=", finalized_,
            "splineId=", splineId, "laneSelector=", laneSelector, "routeSeed=", routeSeed));
    }
    if (!std::isfinite(minimumTurnRadiusMeters) || minimumTurnRadiusMeters < 0.0) {
        throw std::invalid_argument(
            trafficInputError("Traffic route minimum turn radius is invalid",
                              "minimumTurnRadiusMeters=", minimumTurnRadiusMeters));
    }
    const auto lanes = lanesBySpline_.find(splineId);
    if (lanes == lanesBySpline_.end() || lanes->second.empty()) {
        return std::nullopt;
    }
    std::size_t current = lanes->second[laneSelector % lanes->second.size()];
    if (!requiredSceneryPathSequence.empty() &&
        requiredSceneryPathSequence.size() != requiredSplineSequence.size()) {
        throw std::invalid_argument(trafficInputError(
            "Traffic route path-selector sequence has the wrong size",
            "requiredSceneryPathSequenceSize=", requiredSceneryPathSequence.size(),
            "requiredSplineSequenceSize=", requiredSplineSequence.size()));
    }
    std::vector<int> effectiveRequiredSplineSequence;
    std::vector<std::optional<std::size_t>> effectiveRequiredSceneryPathSequence;
    effectiveRequiredSplineSequence.reserve(requiredSplineSequence.size());
    effectiveRequiredSceneryPathSequence.reserve(requiredSplineSequence.size());
    for (std::size_t index = 0; index < requiredSplineSequence.size(); ++index) {
        const int requiredSplineId = requiredSplineSequence[index];
        const auto requiredLanes = lanesBySpline_.find(requiredSplineId);
        const bool hasSceneryPath =
            requiredLanes != lanesBySpline_.end() &&
            std::any_of(
                requiredLanes->second.begin(), requiredLanes->second.end(),
                [&](std::size_t lane) { return segments_[lane].sceneryPathIndex.has_value(); });
        if (!hasSceneryPath && !effectiveRequiredSplineSequence.empty() &&
            effectiveRequiredSplineSequence.back() == requiredSplineId) {
            continue;
        }
        effectiveRequiredSplineSequence.push_back(requiredSplineId);
        effectiveRequiredSceneryPathSequence.push_back(requiredSceneryPathSequence.empty()
                                                           ? std::nullopt
                                                           : requiredSceneryPathSequence[index]);
    }
    if (!effectiveRequiredSplineSequence.empty() &&
        segments_[current].spline.placement.splineId != effectiveRequiredSplineSequence.front()) {
        return std::nullopt;
    }
    if (!effectiveRequiredSceneryPathSequence.empty() &&
        effectiveRequiredSceneryPathSequence.front().has_value() &&
        segments_[current].sceneryPathIndex.has_value() &&
        segments_[current].sceneryPathIndex != effectiveRequiredSceneryPathSequence.front()) {
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
    const auto matchesRequiredLane = [&](std::size_t lane, std::size_t requiredIndex) {
        return requiredIndex < effectiveRequiredSplineSequence.size() &&
               segments_[lane].spline.placement.splineId ==
                   effectiveRequiredSplineSequence[requiredIndex] &&
               (!effectiveRequiredSceneryPathSequence[requiredIndex].has_value() ||
                !segments_[lane].sceneryPathIndex.has_value() ||
                segments_[lane].sceneryPathIndex ==
                    effectiveRequiredSceneryPathSequence[requiredIndex]);
    };
    const auto advanceRequiredIndex = [&](std::size_t lane, std::size_t requiredIndex) {
        while (matchesRequiredLane(lane, requiredIndex)) {
            ++requiredIndex;
        }
        return requiredIndex;
    };
    const auto choicesFor = [&](std::size_t currentLane, std::size_t nextRequiredIndex) {
        std::vector<std::size_t> choices;
        const bool followingRequiredSequence =
            nextRequiredIndex < effectiveRequiredSplineSequence.size();
        const int requiredSplineId =
            followingRequiredSequence ? effectiveRequiredSplineSequence[nextRequiredIndex] : 0;
        const std::optional<std::size_t> requiredPathIndex =
            followingRequiredSequence && !effectiveRequiredSceneryPathSequence.empty()
                ? effectiveRequiredSceneryPathSequence[nextRequiredIndex]
                : std::nullopt;
        std::vector<std::size_t> possibleCandidates = next_[currentLane];
        if (followingRequiredSequence && requiredPathIndex.has_value()) {
            const auto requiredLanes = lanesBySpline_.find(requiredSplineId);
            if (requiredLanes != lanesBySpline_.end()) {
                constexpr double horizontalToleranceMeters = 0.5;
                constexpr double verticalToleranceMeters = 2.5;
                for (const std::size_t candidate : requiredLanes->second) {
                    if (segments_[candidate].sceneryPathIndex != requiredPathIndex ||
                        std::find(possibleCandidates.begin(), possibleCandidates.end(),
                                  candidate) != possibleCandidates.end()) {
                        continue;
                    }
                    const MapTrafficPose& from = laneEnds_[currentLane];
                    const MapTrafficPose& to = laneStarts_[candidate];
                    const double horizontalGap = std::hypot(from.position[0] - to.position[0],
                                                            from.position[1] - to.position[1]);
                    const double verticalGap = std::abs(from.position[2] - to.position[2]);
                    if (horizontalGap <= horizontalToleranceMeters &&
                        verticalGap <= verticalToleranceMeters &&
                        std::cos(from.headingRadians - to.headingRadians) >= -0.01) {
                        possibleCandidates.push_back(candidate);
                    }
                }
            }
        }
        for (const auto candidate : possibleCandidates) {
            if (!allowIntermediarySegmentsBetweenRequired && followingRequiredSequence &&
                segments_[candidate].spline.placement.splineId != requiredSplineId) {
                continue;
            }
            if (allowIntermediarySegmentsBetweenRequired && followingRequiredSequence &&
                segments_[candidate].spline.placement.splineId == requiredSplineId &&
                !matchesRequiredLane(candidate, nextRequiredIndex)) {
                continue;
            }
            if ((!allowIntermediarySegmentsBetweenRequired ||
                 (followingRequiredSequence &&
                  segments_[candidate].spline.placement.splineId == requiredSplineId)) &&
                requiredPathIndex.has_value() &&
                segments_[candidate].sceneryPathIndex.has_value() &&
                segments_[candidate].sceneryPathIndex != requiredPathIndex) {
                continue;
            }
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
                const MapTrafficRoute subdividedJoin = subdivideMapTrafficRoute(join);
                if (!mapTrafficCurveFitsMinimumTurnRadius(subdividedJoin, lengths_[currentLane],
                                                          minimumTurnRadiusMeters)) {
                    continue;
                }
            }
            choices.push_back(candidate);
        }
        // Seed zero retains straight-first routing for route inspection and timetable matching.
        if (!choices.empty() && routeSeed != 0) {
            // A low-discrepancy sequence avoids lopsided branch counts in small fixed pools.
            std::uint64_t lanePhase =
                0x9e3779b97f4a7c15ULL * (static_cast<std::uint64_t>(currentLane) + 1U);
            lanePhase = (lanePhase ^ (lanePhase >> 30U)) * 0xbf58476d1ce4e5b9ULL;
            lanePhase = (lanePhase ^ (lanePhase >> 27U)) * 0x94d049bb133111ebULL;
            lanePhase ^= lanePhase >> 31U;
            const long double laneOffset =
                static_cast<long double>(lanePhase >> 11U) / 9007199254740992.0L;
            constexpr long double routeSeedStep = 0.61803398874989484820458683436563811772L;
            const long double selectionFraction = std::fmod(
                static_cast<long double>(routeSeed - 1U) * routeSeedStep + laneOffset, 1.0L);
            constexpr double straightHeadingToleranceRadians = std::numbers::pi / 12.0;
            constexpr double straightRouteWeight = 2;
            double maximumDensity = 0.0;
            for (const std::size_t choice : choices) {
                maximumDensity =
                    std::max(maximumDensity,
                             mapTrafficPathDensityForVehicle(segments_[choice].pathRules, vehicle));
            }
            const auto choiceWeight = [&](std::size_t candidate) {
                const double densityWeight =
                    maximumDensity > 0.0
                        ? mapTrafficPathDensityForVehicle(segments_[candidate].pathRules, vehicle) /
                              maximumDensity
                        : 1.0;
                const double headingDifference = std::abs(std::remainder(
                    laneEnds_[currentLane].headingRadians - laneEnds_[candidate].headingRadians,
                    2.0 * std::numbers::pi));
                return densityWeight * (headingDifference <= straightHeadingToleranceRadians
                                            ? straightRouteWeight
                                            : 1.0);
            };
            double totalWeight = 0.0;
            for (const std::size_t choice : choices) {
                totalWeight += choiceWeight(choice);
            }
            double selection =
                static_cast<double>(selectionFraction * static_cast<long double>(totalWeight));
            std::size_t choice = choices.size() - 1;
            for (std::size_t index = 0; index < choices.size(); ++index) {
                const double weight = choiceWeight(choices[index]);
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
        std::size_t nextRequiredIndex;
        std::vector<std::size_t> choices;
        std::size_t nextChoice = 0;
    };
    struct SearchState {
        std::size_t lane;
        std::size_t nextRequiredIndex;
        bool operator==(const SearchState&) const = default;
    };
    struct SearchStateHash {
        std::size_t operator()(const SearchState& state) const noexcept {
            const std::size_t laneHash = std::hash<std::size_t>{}(state.lane);
            const std::size_t progressHash = std::hash<std::size_t>{}(state.nextRequiredIndex);
            return laneHash ^ (progressHash + 0x9e3779b9U + (laneHash << 6U) + (laneHash >> 2U));
        }
    };
    if (allowIntermediarySegmentsBetweenRequired && !effectiveRequiredSplineSequence.empty()) {
        const std::size_t laneCount = segments_.size();
        const auto baseLaneFor = [&](std::size_t orientedLane) {
            return orientedLane < laneCount ? orientedLane : orientedLane - laneCount;
        };
        const auto orientedSegment = [&](std::size_t orientedLane) {
            MapTrafficRouteSegment segment = segments_[baseLaneFor(orientedLane)];
            if (orientedLane >= laneCount) {
                segment = reverseMapTrafficRouteSegment(std::move(segment),
                                                        lengths_[baseLaneFor(orientedLane)]);
            }
            return segment;
        };
        const auto timetableChoicesFor = [&](const SearchState& state) {
            if (!allowReverseTraversalForRequired) {
                return choicesFor(state.lane, state.nextRequiredIndex);
            }
            std::vector<std::size_t> possibleCandidates = timetableNext_[state.lane];
            if (state.nextRequiredIndex < effectiveRequiredSplineSequence.size() &&
                !effectiveRequiredSceneryPathSequence.empty() &&
                effectiveRequiredSceneryPathSequence[state.nextRequiredIndex].has_value()) {
                const int requiredSplineId =
                    effectiveRequiredSplineSequence[state.nextRequiredIndex];
                const std::optional<std::size_t> requiredPathIndex =
                    effectiveRequiredSceneryPathSequence[state.nextRequiredIndex];
                const auto requiredLanes = lanesBySpline_.find(requiredSplineId);
                if (requiredLanes != lanesBySpline_.end()) {
                    const auto directedStart = [&](std::size_t orientedLane) {
                        if (orientedLane < laneCount) {
                            return laneStarts_[orientedLane];
                        }
                        MapTrafficPose pose = laneEnds_[orientedLane - laneCount];
                        pose.headingRadians += std::numbers::pi;
                        return pose;
                    };
                    const auto directedEnd = [&](std::size_t orientedLane) {
                        if (orientedLane < laneCount) {
                            return laneEnds_[orientedLane];
                        }
                        MapTrafficPose pose = laneStarts_[orientedLane - laneCount];
                        pose.headingRadians += std::numbers::pi;
                        return pose;
                    };
                    const MapTrafficPose from = directedEnd(state.lane);
                    for (const std::size_t requiredLane : requiredLanes->second) {
                        if (segments_[requiredLane].sceneryPathIndex != requiredPathIndex) {
                            continue;
                        }
                        for (const std::size_t candidate :
                             {requiredLane, requiredLane + laneCount}) {
                            if (std::find(possibleCandidates.begin(), possibleCandidates.end(),
                                          candidate) != possibleCandidates.end()) {
                                continue;
                            }
                            const MapTrafficPose to = directedStart(candidate);
                            if (std::hypot(from.position[0] - to.position[0],
                                           from.position[1] - to.position[1]) <= 0.5 &&
                                std::abs(from.position[2] - to.position[2]) <= 2.5 &&
                                std::cos(from.headingRadians - to.headingRadians) >= -0.01) {
                                possibleCandidates.push_back(candidate);
                            }
                        }
                    }
                }
            }
            std::vector<std::size_t> choices;
            for (const std::size_t candidate : possibleCandidates) {
                const std::size_t candidateLane = baseLaneFor(candidate);
                const int candidateSplineId = segments_[candidateLane].spline.placement.splineId;
                if (state.nextRequiredIndex < effectiveRequiredSplineSequence.size() &&
                    candidateSplineId == effectiveRequiredSplineSequence[state.nextRequiredIndex] &&
                    !matchesRequiredLane(candidateLane, state.nextRequiredIndex)) {
                    continue;
                }
                if (!mapTrafficPathAllowsVehicle(segments_[candidateLane].pathRules, vehicle)) {
                    continue;
                }
                if (minimumTurnRadiusMeters > 0.0) {
                    if (maximumCurvatures_[candidateLane] > 1.05 / minimumTurnRadiusMeters) {
                        continue;
                    }
                    MapTrafficRoute join;
                    join.segments = {orientedSegment(state.lane), orientedSegment(candidate)};
                    join.cumulativeEndDistances = {lengths_[baseLaneFor(state.lane)],
                                                   lengths_[baseLaneFor(state.lane)] +
                                                       lengths_[candidateLane]};
                    join.totalLength = join.cumulativeEndDistances.back();
                    if (!mapTrafficCurveFitsMinimumTurnRadius(subdivideMapTrafficRoute(join),
                                                              lengths_[baseLaneFor(state.lane)],
                                                              minimumTurnRadiusMeters)) {
                        continue;
                    }
                }
                choices.push_back(candidate);
            }
            return choices;
        };
        const SearchState startState{current, advanceRequiredIndex(current, 0)};
        std::queue<SearchState> pending;
        std::unordered_map<SearchState, SearchState, SearchStateHash> predecessors;
        pending.push(startState);
        predecessors.emplace(startState, startState);
        std::optional<SearchState> completedState;
        while (!pending.empty()) {
            const SearchState state = pending.front();
            pending.pop();
            if (state.nextRequiredIndex >= effectiveRequiredSplineSequence.size()) {
                completedState = state;
                break;
            }
            for (const std::size_t candidate : timetableChoicesFor(state)) {
                const std::size_t candidateLane = baseLaneFor(candidate);
                const std::size_t nextRequiredIndex =
                    advanceRequiredIndex(candidateLane, state.nextRequiredIndex);
                const SearchState nextState{candidate, nextRequiredIndex};
                if (predecessors.contains(nextState)) {
                    continue;
                }
                predecessors.emplace(nextState, state);
                pending.push(nextState);
            }
        }
        if (!completedState.has_value()) {
            return std::nullopt;
        }
        std::vector<std::size_t> routeLaneIndexes;
        for (SearchState state = *completedState;; state = predecessors.at(state)) {
            routeLaneIndexes.push_back(state.lane);
            if (state == startState) {
                break;
            }
        }
        std::reverse(routeLaneIndexes.begin(), routeLaneIndexes.end());
        for (const std::size_t orientedLane : routeLaneIndexes) {
            const std::size_t lane = baseLaneFor(orientedLane);
            MapTrafficRouteSegment segment = orientedSegment(orientedLane);
            if (!route.segments.empty() && !route.segments.back().points.empty() &&
                !segment.points.empty()) {
                const std::array<double, 3>& previousEnd = route.segments.back().points.back();
                std::array<double, 3>& currentStart = segment.points.front();
                const double horizontalGap =
                    std::hypot(previousEnd[0] - currentStart[0], previousEnd[1] - currentStart[1]);
                const double verticalGap = std::abs(previousEnd[2] - currentStart[2]);
                if (horizontalGap <= 0.5 && verticalGap <= 2.5) {
                    const double verticalOffset = previousEnd[2] - currentStart[2];
                    for (auto& point : segment.points) {
                        point[2] += verticalOffset;
                    }
                    segment.points.front() = previousEnd;
                }
            }
            route.segments.push_back(std::move(segment));
            route.totalLength += lengths_[lane];
            route.cumulativeEndDistances.push_back(route.totalLength);
        }
        return route;
    }
    std::vector<Frame> stack;
    std::unordered_map<std::size_t, double> visitedLanes;
    std::unordered_set<std::size_t> blockedLanes;
    std::unordered_map<SearchState, double, SearchStateHash> visitedStates;
    std::unordered_set<SearchState, SearchStateHash> blockedStates;
    const auto append = [&](std::size_t lane, std::size_t nextRequiredIndex) {
        const double laneStartDistance = route.totalLength;
        MapTrafficRouteSegment segment = segments_[lane];
        if (!effectiveRequiredSplineSequence.empty() && !route.segments.empty() &&
            !route.segments.back().points.empty() && !segment.points.empty()) {
            const std::array<double, 3>& previousEnd = route.segments.back().points.back();
            std::array<double, 3>& currentStart = segment.points.front();
            const double horizontalGap =
                std::hypot(previousEnd[0] - currentStart[0], previousEnd[1] - currentStart[1]);
            const double verticalGap = std::abs(previousEnd[2] - currentStart[2]);
            if (horizontalGap <= 0.5 && verticalGap <= 2.5) {
                const double verticalOffset = previousEnd[2] - currentStart[2];
                for (auto& point : segment.points) {
                    point[2] += verticalOffset;
                }
                segment.points.front() = previousEnd;
            }
        }
        route.segments.push_back(std::move(segment));
        route.totalLength += lengths_[lane];
        route.cumulativeEndDistances.push_back(route.totalLength);
        nextRequiredIndex = advanceRequiredIndex(lane, nextRequiredIndex);
        const SearchState state{lane, nextRequiredIndex};
        if (allowIntermediarySegmentsBetweenRequired) {
            visitedStates.emplace(state, laneStartDistance);
        } else {
            visitedLanes.emplace(lane, laneStartDistance);
        }
        stack.push_back({lane, nextRequiredIndex, choicesFor(lane, nextRequiredIndex)});
    };
    append(current, 0);
    while (!stack.empty()) {
        if (stopAtRequiredSequenceEnd && !effectiveRequiredSplineSequence.empty() &&
            stack.back().nextRequiredIndex >= effectiveRequiredSplineSequence.size()) {
            return route;
        }
        auto& frame = stack.back();
        if (next_[frame.lane].empty() &&
            (effectiveRequiredSplineSequence.empty() ||
             frame.nextRequiredIndex >= effectiveRequiredSplineSequence.size())) {
            return route;
        }
        if (frame.nextChoice == frame.choices.size()) {
            // Do not invent an exit where geometry continues but the vehicle cannot turn.
            const SearchState state{frame.lane, frame.nextRequiredIndex};
            if (allowIntermediarySegmentsBetweenRequired) {
                blockedStates.insert(state);
                visitedStates.erase(state);
            } else {
                blockedLanes.insert(frame.lane);
                visitedLanes.erase(frame.lane);
            }
            stack.pop_back();
            route.segments.pop_back();
            route.cumulativeEndDistances.pop_back();
            route.totalLength =
                route.cumulativeEndDistances.empty() ? 0.0 : route.cumulativeEndDistances.back();
            continue;
        }
        current = frame.choices[frame.nextChoice++];
        const std::size_t candidateRequiredIndex =
            advanceRequiredIndex(current, frame.nextRequiredIndex);
        const SearchState candidateState{current, candidateRequiredIndex};
        if (allowIntermediarySegmentsBetweenRequired) {
            if (blockedStates.contains(candidateState)) {
                continue;
            }
            const auto visitedState = visitedStates.find(candidateState);
            if (visitedState != visitedStates.end()) {
                if (!effectiveRequiredSplineSequence.empty() &&
                    candidateRequiredIndex < effectiveRequiredSplineSequence.size()) {
                    continue;
                }
                route.loopStartDistance = visitedState->second;
                route.closed = *route.loopStartDistance == 0.0;
                return route;
            }
        } else {
            if (blockedLanes.contains(current)) {
                continue;
            }
            const auto visitedLane = visitedLanes.find(current);
            if (visitedLane != visitedLanes.end()) {
                if (!effectiveRequiredSplineSequence.empty() &&
                    route.segments.size() < effectiveRequiredSplineSequence.size()) {
                    continue;
                }
                route.loopStartDistance = visitedLane->second;
                route.closed = *route.loopStartDistance == 0.0;
                return route;
            }
        }
        append(current, frame.nextRequiredIndex);
    }
    return std::nullopt;
}

std::vector<MapTrafficLaneStart> MapTrafficLaneNetwork::entryLanes() const {
    if (!finalized_) {
        throw std::logic_error(
            trafficInputError("Traffic lane network must be finalized before entry lookup",
                              "finalized=", finalized_));
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

std::vector<MapTrafficLaneStart>
MapTrafficLaneNetwork::lanesForTimetableEntry(int splineId,
                                              std::optional<std::size_t> sceneryPathIndex) const {
    if (!finalized_) {
        throw std::logic_error(trafficInputError(
            "Traffic lane network must be finalized before lane lookup", "finalized=", finalized_,
            "splineId=", splineId, "hasSceneryPathIndex=", sceneryPathIndex.has_value(),
            "sceneryPathIndex=", sceneryPathIndex.value_or(0)));
    }
    const auto lanes = lanesBySpline_.find(splineId);
    if (lanes == lanesBySpline_.end()) {
        return {};
    }
    std::vector<MapTrafficLaneStart> starts;
    starts.reserve(lanes->second.size());
    for (std::size_t selector = 0; selector < lanes->second.size(); ++selector) {
        const MapTrafficRouteSegment& segment = segments_[lanes->second[selector]];
        if (sceneryPathIndex.has_value() && segment.sceneryPathIndex.has_value() &&
            segment.sceneryPathIndex != sceneryPathIndex) {
            continue;
        }
        starts.push_back({splineId, selector});
    }
    return starts;
}

std::optional<MapTrafficLaneConnectionGap> MapTrafficLaneNetwork::nearestConnectionGap(
    int fromSplineId, std::optional<std::size_t> fromPathIndex, int toSplineId,
    std::optional<std::size_t> toPathIndex) const {
    if (!finalized_) {
        throw std::logic_error(trafficInputError(
            "Traffic lane network must be finalized before lane lookup", "finalized=", finalized_,
            "fromSplineId=", fromSplineId, "toSplineId=", toSplineId, "hasFromPathIndex=",
            fromPathIndex.has_value(), "fromPathIndex=", fromPathIndex.value_or(0),
            "hasToPathIndex=", toPathIndex.has_value(), "toPathIndex=", toPathIndex.value_or(0)));
    }
    const auto fromLanes = lanesBySpline_.find(fromSplineId);
    const auto toLanes = lanesBySpline_.find(toSplineId);
    if (fromLanes == lanesBySpline_.end() || toLanes == lanesBySpline_.end()) {
        return std::nullopt;
    }
    std::optional<MapTrafficLaneConnectionGap> nearest;
    double nearestDistanceSquared = std::numeric_limits<double>::infinity();
    for (const std::size_t fromLane : fromLanes->second) {
        if (fromPathIndex.has_value() && segments_[fromLane].sceneryPathIndex.has_value() &&
            segments_[fromLane].sceneryPathIndex != fromPathIndex) {
            continue;
        }
        for (const std::size_t toLane : toLanes->second) {
            if (toPathIndex.has_value() && segments_[toLane].sceneryPathIndex.has_value() &&
                segments_[toLane].sceneryPathIndex != toPathIndex) {
                continue;
            }
            const MapTrafficPose& from = laneEnds_[fromLane];
            const MapTrafficPose& to = laneStarts_[toLane];
            const double horizontal =
                std::hypot(from.position[0] - to.position[0], from.position[1] - to.position[1]);
            const double vertical = std::abs(from.position[2] - to.position[2]);
            const double distanceSquared = horizontal * horizontal + vertical * vertical;
            if (distanceSquared < nearestDistanceSquared) {
                nearestDistanceSquared = distanceSquared;
                nearest = MapTrafficLaneConnectionGap{
                    horizontal, vertical, std::cos(from.headingRadians - to.headingRadians)};
            }
        }
    }
    return nearest;
}

std::optional<MapTrafficLaneStart>
MapTrafficLaneNetwork::selectEntryLane(MapTrafficVehicleFilter vehicle,
                                       std::size_t selectionSeed) const {
    if (!finalized_) {
        throw std::logic_error(
            trafficInputError("Traffic lane network must be finalized before entry lookup",
                              "finalized=", finalized_, "selectionSeed=", selectionSeed));
    }
    if (selectionSeed == 0) {
        throw std::invalid_argument(trafficInputError(
            "Traffic entry selection seed must be nonzero", "selectionSeed=", selectionSeed));
    }
    return selectLane(vehicle, selectionSeed, entryLanes());
}

std::vector<MapTrafficLaneStart> MapTrafficLaneNetwork::spawnLanes() const {
    if (!finalized_) {
        throw std::logic_error(
            trafficInputError("Traffic lane network must be finalized before spawn lookup",
                              "finalized=", finalized_));
    }
    return spawnLanes_;
}

std::optional<MapTrafficLaneStart>
MapTrafficLaneNetwork::selectSpawnLane(MapTrafficVehicleFilter vehicle,
                                       std::size_t selectionSeed) const {
    if (!finalized_) {
        throw std::logic_error(
            trafficInputError("Traffic lane network must be finalized before spawn lookup",
                              "finalized=", finalized_, "selectionSeed=", selectionSeed));
    }
    if (selectionSeed == 0) {
        throw std::invalid_argument(trafficInputError(
            "Traffic spawn selection seed must be nonzero", "selectionSeed=", selectionSeed));
    }
    return selectLane(vehicle, selectionSeed, spawnLanes_);
}

std::optional<MapTrafficLaneStart> MapTrafficLaneNetwork::selectSpawnLaneInTiles(
    MapTrafficVehicleFilter vehicle, std::size_t selectionSeed,
    const std::vector<std::pair<int, int>>& activeTiles) const {
    if (!finalized_) {
        throw std::logic_error("Traffic lane network must be finalized before spawn lookup");
    }
    if (selectionSeed == 0) {
        throw std::invalid_argument("Traffic spawn selection seed must be nonzero");
    }
    std::vector<MapTrafficLaneStart> candidates;
    for (const MapTrafficLaneStart& entry : spawnLanes_) {
        const auto& lanes = lanesBySpline_.at(entry.splineId);
        const std::size_t laneIndex = lanes[entry.laneSelector];
        const auto& segment = segments_[laneIndex];
        const double tileSize = segment.spline.coordinates.tileSize;
        const auto& start = laneStarts_[laneIndex].position;
        if (std::any_of(activeTiles.begin(), activeTiles.end(), [&](const auto& tile) {
                if (tile.first == segment.spline.tileX && tile.second == segment.spline.tileY) {
                    return true;
                }
                const double minX = static_cast<double>(tile.first) * tileSize;
                const double minY = static_cast<double>(tile.second) * tileSize;
                const double dx = std::max({minX - start[0], 0.0, start[0] - minX - tileSize});
                const double dy = std::max({minY - start[1], 0.0, start[1] - minY - tileSize});
                return std::hypot(dx, dy) <= lengths_[laneIndex] + 0.01;
            })) {
            candidates.push_back(entry);
        }
    }
    return selectLane(vehicle, selectionSeed, candidates);
}

std::optional<MapTrafficLaneStart>
MapTrafficLaneNetwork::selectLane(MapTrafficVehicleFilter vehicle, std::size_t selectionSeed,
                                  const std::vector<MapTrafficLaneStart>& laneStarts) const {
    if (!finalized_) {
        throw std::logic_error(
            trafficInputError("Traffic lane network must be finalized before lane selection",
                              "finalized=", finalized_, "selectionSeed=", selectionSeed,
                              "laneStartCount=", laneStarts.size()));
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
        const double density = mapTrafficPathDensityForVehicle(segment.pathRules, vehicle);
        if (!mapTrafficPathAllowsVehicle(segment.pathRules, vehicle)) {
            continue;
        }
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
        throw std::invalid_argument(trafficInputError(
            "Traffic obstacle footprint is invalid", "x=", obstacle.pose.position[0],
            "y=", obstacle.pose.position[1], "z=", obstacle.pose.position[2],
            "headingRadians=", obstacle.pose.headingRadians, "lengthMeters=", obstacle.lengthMeters,
            "widthMeters=", obstacle.widthMeters));
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

// Exact: rejects only pairs whose centres are too far apart for any SAT axis to overlap.
bool footprintsOverlapWithDiameters(const MapTrafficObstacle& first,
                                    const MapTrafficObstacle& second, double clearanceMeters,
                                    double firstDiameter, double secondDiameter) {
    if (std::abs(first.pose.position[2] - second.pose.position[2]) > 3.0) {
        return false;
    }
    const double deltaX = second.pose.position[0] - first.pose.position[0];
    const double deltaY = second.pose.position[1] - first.pose.position[1];
    // Clearance inflates both rectangles' sides, so their corners grow by sqrt(2).
    const double maximumCenterDistance = (firstDiameter + secondDiameter) * 0.5 +
                                         std::numbers::sqrt2 * std::max(0.0, clearanceMeters);
    if (deltaX * deltaX + deltaY * deltaY >= maximumCenterDistance * maximumCenterDistance) {
        return false;
    }
    return footprintsOverlap(first, second, clearanceMeters);
}

bool footprintsOverlapWithBroadPhase(const MapTrafficObstacle& first,
                                     const MapTrafficObstacle& second, double clearanceMeters) {
    if (std::abs(first.pose.position[2] - second.pose.position[2]) > 3.0) {
        return false;
    }
    return footprintsOverlapWithDiameters(first, second, clearanceMeters,
                                          std::hypot(first.lengthMeters, first.widthMeters),
                                          std::hypot(second.lengthMeters, second.widthMeters));
}

double footprintSeparation(const MapTrafficObstacle& a, const MapTrafficObstacle& b,
                           double clearance) {
    if (std::abs(a.pose.position[2] - b.pose.position[2]) > 3.0) {
        return std::numeric_limits<double>::infinity();
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
    double separation = -std::numeric_limits<double>::infinity();
    for (const auto& axis : {aa[0], aa[1], ba[0], ba[1]}) {
        const double extent = (a.lengthMeters * std::abs(dot(axis, aa[0])) +
                               a.widthMeters * std::abs(dot(axis, aa[1])) +
                               b.lengthMeters * std::abs(dot(axis, ba[0])) +
                               b.widthMeters * std::abs(dot(axis, ba[1]))) *
                                  0.5 +
                              clearance;
        separation = std::max(separation, std::abs(dx * axis[0] + dy * axis[1]) - extent);
    }
    return separation;
}

MapTrafficObstacle actorFootprint(const MapTrafficActor& actor, double distance) {
    const auto pose = sampleMapTrafficActorPose(actor, distance);
    if (!pose) {
        throw std::invalid_argument(trafficInputError(
            "Cannot sample traffic driving route", "distanceMeters=", distance,
            "lateralOffsetMeters=", actor.lateralOffsetMeters,
            "routeTotalLength=", actor.route.totalLength, "segments=", actor.route.segments.size(),
            "cumulativeEndDistances=", actor.route.cumulativeEndDistances.size()));
    }
    return {*pose, actor.lengthMeters, actor.widthMeters};
}

// Conservative: true only when sampleMapTrafficRoute cannot return nullopt for any finite
// distance, so footprint and curve-limit queries on this route can never throw.
bool mapTrafficRouteSamplingCannotFail(const MapTrafficRoute& route, double lateralOffsetMeters) {
    if (route.segments.empty() || route.segments.size() != route.cumulativeEndDistances.size() ||
        !std::isfinite(route.totalLength) || route.totalLength <= 0.0 ||
        !std::isfinite(lateralOffsetMeters)) {
        return false;
    }
    if (route.loopStartDistance &&
        (!std::isfinite(*route.loopStartDistance) || *route.loopStartDistance < 0.0 ||
         *route.loopStartDistance >= route.totalLength)) {
        return false;
    }
    double segmentStart = 0.0;
    for (std::size_t index = 0; index < route.segments.size(); ++index) {
        const double segmentEnd = route.cumulativeEndDistances[index];
        if (!std::isfinite(segmentEnd) || !(segmentEnd > segmentStart)) {
            return false;
        }
        const MapTrafficRouteSegment& segment = route.segments[index];
        if (!segment.points.empty()) {
            if (segment.points.size() < 2) {
                return false;
            }
            for (const auto& point : segment.points) {
                if (!std::all_of(point.begin(), point.end(),
                                 [](double value) { return std::isfinite(value); })) {
                    return false;
                }
            }
            // Sampling always resolves on the last edge when it has a positive length.
            const auto& from = segment.points[segment.points.size() - 2];
            const auto& to = segment.points.back();
            if (!(std::hypot(to[0] - from[0], to[1] - from[1], to[2] - from[2]) > 0.0)) {
                return false;
            }
        } else {
            const MapSplinePlacement& spline = segment.spline.placement;
            const double startX =
                static_cast<double>(segment.spline.tileX) * segment.spline.coordinates.tileSize +
                spline.localX;
            const double startY =
                static_cast<double>(segment.spline.tileY) * segment.spline.coordinates.tileSize +
                spline.localY;
            // Magnitudes this small keep every intermediate in sampleMapSpline finite
            // (distance / radius <= 1e23 because curved radii exceed 1e-8).
            constexpr double safeMagnitude = 1.0e15;
            if (!spline.geometryValid || !std::isfinite(spline.length) || spline.length <= 0.0 ||
                spline.length > safeMagnitude || !(std::abs(startX) <= safeMagnitude) ||
                !(std::abs(startY) <= safeMagnitude) ||
                !(std::abs(spline.rotationDegrees) <= safeMagnitude) ||
                !(std::abs(spline.radius) <= safeMagnitude)) {
                return false;
            }
            // Spline distance is monotonic in the clamped segment distance, so checking both
            // ends covers every sample in this segment.
            const double segmentLength = segmentEnd - segmentStart;
            const double startDistance = splineDistanceAtRouteStart(segment);
            const double endDistance =
                startDistance + (segment.reversed ? -segmentLength : segmentLength);
            const auto withinSpline = [&](double splineDistance) {
                return std::isfinite(splineDistance) && splineDistance >= -1.0e-6 &&
                       splineDistance <= spline.length + 1.0e-6;
            };
            if (!withinSpline(startDistance) || !withinSpline(endDistance)) {
                return false;
            }
        }
        segmentStart = segmentEnd;
    }
    return true;
}

} // namespace

bool mapTrafficActorBehindObstacle(const MapTrafficActor& actor, const MapTrafficObstacle& obstacle,
                                   double maximumGapMeters) {
    if (!std::isfinite(maximumGapMeters) || maximumGapMeters < 0.0) {
        throw std::invalid_argument(
            trafficInputError("Maximum traffic following gap must be finite and non-negative",
                              "maximumGapMeters=", maximumGapMeters));
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
    const double bumperGap = deltaX * std::sin(heading) + deltaY * std::cos(heading) -
                             (actor.lengthMeters + obstacle.lengthMeters) * 0.5;
    const double lateralGap = std::abs(deltaX * std::cos(heading) - deltaY * std::sin(heading)) -
                              (actor.widthMeters + obstacle.widthMeters) * 0.5;
    return bumperGap >= 0.0 && bumperGap <= maximumGapMeters && lateralGap <= 0.75;
}

bool mapTrafficHasClearance(const MapTrafficObstacle& vehicle,
                            const std::vector<MapTrafficObstacle>& obstacles,
                            double clearanceMeters) {
    validateFootprint(vehicle);
    if (!std::isfinite(clearanceMeters) || clearanceMeters < 0.0) {
        throw std::invalid_argument(
            trafficInputError("Traffic clearance must be finite and non-negative",
                              "clearanceMeters=", clearanceMeters));
    }
    for (const auto& obstacle : obstacles) {
        validateFootprint(obstacle);
        if (footprintsOverlapWithBroadPhase(vehicle, obstacle, clearanceMeters)) {
            return false;
        }
    }
    return true;
}

namespace {

struct TrafficSignalApproach {
    const MapTrafficSignalSchedule* schedule = nullptr;
    double distanceToStopMeters = 0.0;
    std::size_t segmentIndex = 0;
    double distanceToEntryMeters = 0.0;
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
            nearest = TrafficSignalApproach{&*segment.trafficSignal, distanceToStop, index,
                                            distanceToSegmentStart};
        }
        segmentStart = segmentEnd;
    }
    return nearest;
}

std::optional<int> trafficSignalStateAtTime(const MapTrafficSignalSchedule& schedule,
                                            double elapsedSeconds) {
    return mapTrafficSignalStateAtTime(schedule, elapsedSeconds);
}

bool trafficSignalBlocksActor(const MapTrafficActor& actor, const TrafficSignalApproach& approach,
                              double signalClockSeconds) {
    const auto state = trafficSignalStateAtTime(*approach.schedule, signalClockSeconds);
    if (!state.has_value()) {
        return true;
    }
    return (*state < 6 || *state > 8) && actor.committedSignalSegmentIndex != approach.segmentIndex;
}

bool updateTrafficSignalStopDecision(MapTrafficActor& actor,
                                     const std::optional<TrafficSignalApproach>& approach,
                                     double signalClockSeconds) {
    if (!approach.has_value()) {
        actor.committedSignalSegmentIndex.reset();
        return false;
    }
    if (actor.committedSignalSegmentIndex != approach->segmentIndex) {
        actor.committedSignalSegmentIndex.reset();
    }
    const auto state = trafficSignalStateAtTime(*approach->schedule, signalClockSeconds);
    if (!state.has_value()) {
        return true;
    }
    if (*state >= 6 && *state <= 8) {
        actor.committedSignalSegmentIndex.reset();
        return false;
    }
    const double stoppingDistance = actor.speedMetersPerSecond * actor.speedMetersPerSecond /
                                    (2.0 * trafficBrakingDeceleration);
    // Do not hard-clamp a moving vehicle at a newly blocking signal it cannot brake for.
    // Once committed, slowing for another vehicle must not revoke the entry decision.
    if (approach->distanceToEntryMeters > 0.0 && actor.speedMetersPerSecond > 0.5 &&
        approach->distanceToStopMeters + 1.0e-9 < stoppingDistance) {
        actor.committedSignalSegmentIndex = approach->segmentIndex;
    }
    return actor.committedSignalSegmentIndex != approach->segmentIndex;
}

} // namespace

bool mapTrafficActorWaitingAtSignal(const MapTrafficActor& actor, double signalClockSeconds) {
    if (!std::isfinite(signalClockSeconds) || signalClockSeconds < 0.0) {
        throw std::invalid_argument(
            trafficInputError("Traffic signal clock must be finite and non-negative",
                              "signalClockSeconds=", signalClockSeconds));
    }
    const std::optional<TrafficSignalApproach> approach = nextTrafficSignalApproach(actor);
    if (!approach.has_value() || approach->distanceToStopMeters > 1.0) {
        return false;
    }
    return trafficSignalBlocksActor(actor, *approach, signalClockSeconds);
}

std::optional<MapTrafficSignalStopDebugInfo>
mapTrafficNextSignalStopDebugInfo(const MapTrafficActor& actor, double signalClockSeconds) {
    if (!std::isfinite(signalClockSeconds) || signalClockSeconds < 0.0) {
        throw std::invalid_argument(
            trafficInputError("Traffic signal clock must be finite and non-negative",
                              "signalClockSeconds=", signalClockSeconds));
    }
    const std::optional<TrafficSignalApproach> approach = nextTrafficSignalApproach(actor);
    if (!approach.has_value()) {
        return std::nullopt;
    }
    const std::optional<MapTrafficPose> pose = sampleMapTrafficRoute(
        actor.route,
        actor.distanceMeters + approach->distanceToStopMeters + actor.lengthMeters * 0.5,
        actor.lateralOffsetMeters);
    if (!pose.has_value()) {
        return std::nullopt;
    }
    return MapTrafficSignalStopDebugInfo{
        *pose, approach->distanceToStopMeters,
        trafficSignalBlocksActor(actor, *approach, signalClockSeconds)};
}

void advanceMapTrafficSignalControllers(
    const std::vector<std::shared_ptr<MapTrafficSignalController>>& controllers,
    const std::vector<MapTrafficActor*>& actors, double elapsedSeconds, double initialClockSeconds,
    const std::vector<MapTrafficSignalBusPriorityRequest>& busPriorityRequests) {
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0 ||
        !std::isfinite(initialClockSeconds) || initialClockSeconds < 0.0) {
        throw std::invalid_argument(trafficInputError(
            "Traffic signal controller update is invalid", "elapsedSeconds=", elapsedSeconds,
            "initialClockSeconds=", initialClockSeconds, "controllerCount=", controllers.size(),
            "actorCount=", actors.size()));
    }
    std::unordered_set<MapTrafficSignalController*> uniqueControllers;
    for (std::size_t index = 0; index < controllers.size(); ++index) {
        const auto& controller = controllers[index];
        if (controller == nullptr) {
            throw std::invalid_argument(
                trafficInputError("Traffic signal controller is null", "controllerIndex=", index,
                                  "controllerIsNull=", controller == nullptr));
        }
        if (uniqueControllers.insert(controller.get()).second) {
            controller->clearRequests();
        }
    }
    for (std::size_t index = 0; index < actors.size(); ++index) {
        const MapTrafficActor* actor = actors[index];
        if (actor == nullptr) {
            throw std::invalid_argument(trafficInputError("Traffic signal actor is null",
                                                          "actorIndex=", index,
                                                          "actorIsNull=", actor == nullptr));
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
        if (actor->isBus && approach->schedule->hasBusPriority &&
            approach->schedule->program != nullptr &&
            std::ranges::any_of(approach->schedule->program->busPriorityRules,
                                [&](const MapTrafficSignalBusPriorityRule& rule) {
                                    return approach->distanceToStopMeters <=
                                           rule.triggerDistanceMeters;
                                })) {
            approach->schedule->controller->requestBusPriority(approach->schedule->lightIndex);
        }
    }
    for (const MapTrafficSignalBusPriorityRequest& request : busPriorityRequests) {
        if (request.controller != nullptr) {
            request.controller->requestBusPriority(request.lightIndex);
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
        throw std::invalid_argument(trafficInputError(
            "Obstacle-aware actor movement inputs are invalid", "lengthMeters=", lengthMeters,
            "widthMeters=", widthMeters, "elapsedSeconds=", elapsedSeconds, "signalClockSeconds=",
            signalClockSeconds, "speedMetersPerSecond=", actor.speedMetersPerSecond,
            "cruiseSpeedMetersPerSecond=", actor.cruiseSpeedMetersPerSecond,
            "distanceMeters=", actor.distanceMeters,
            "requestedTravelMeters=", actor.speedMetersPerSecond * elapsedSeconds));
    }
    for (const MapTrafficObstacle& obstacle : obstacles) {
        validateFootprint(obstacle);
    }
    double remaining = elapsedSeconds;
    while (remaining > 1.0e-9) {
        const double currentSignalClockSeconds = signalClockSeconds + elapsedSeconds - remaining;
        const double step =
            std::min({remaining, 0.05, 0.1 / std::max(actor.speedMetersPerSecond, 0.1)});
        const std::optional<TrafficSignalApproach> signalApproach =
            nextTrafficSignalApproach(actor);
        const std::optional<int> signalState =
            signalApproach.has_value()
                ? trafficSignalStateAtTime(*signalApproach->schedule, currentSignalClockSeconds)
                : std::nullopt;
        const bool blockedBySignal =
            signalApproach.has_value() &&
            (!signalState.has_value() || *signalState < 6 || *signalState > 8);
        // Only distance and speed change, so move the actor in place and restore them if the
        // step is rejected, instead of copying the whole route each substep.
        const double previousDistanceMeters = actor.distanceMeters;
        const double previousSpeedMetersPerSecond = actor.speedMetersPerSecond;
        const auto restore = [&] {
            actor.distanceMeters = previousDistanceMeters;
            actor.speedMetersPerSecond = previousSpeedMetersPerSecond;
        };
        try {
            actor.speedMetersPerSecond = actor.cruiseSpeedMetersPerSecond;
            const double requestedTravel = actor.speedMetersPerSecond * step;
            const bool reachesSignalStop =
                blockedBySignal && signalApproach->distanceToStopMeters <= requestedTravel + 1.0e-9;
            if (reachesSignalStop) {
                actor.distanceMeters =
                    previousDistanceMeters + signalApproach->distanceToStopMeters;
                if (actor.route.closed) {
                    actor.distanceMeters = std::fmod(actor.distanceMeters, actor.route.totalLength);
                } else if (actor.route.loopStartDistance &&
                           actor.distanceMeters >= actor.route.totalLength) {
                    const double loopStart = *actor.route.loopStartDistance;
                    const double loopLength = actor.route.totalLength - loopStart;
                    actor.distanceMeters =
                        loopStart + std::fmod(actor.distanceMeters - loopStart, loopLength);
                } else {
                    actor.distanceMeters = std::min(actor.distanceMeters, actor.route.totalLength);
                }
                actor.speedMetersPerSecond = 0.0;
            } else {
                advanceMapTrafficActorLooping(actor, step);
            }
            const auto pose = sampleMapTrafficActorPose(actor, actor.distanceMeters);
            if (!pose) {
                throw std::runtime_error(
                    trafficInputError("Cannot sample an obstacle-aware traffic actor",
                                      "distanceMeters=", actor.distanceMeters,
                                      "lateralOffsetMeters=", actor.lateralOffsetMeters,
                                      "routeTotalLength=", actor.route.totalLength));
            }
            if (!mapTrafficHasClearance({*pose, lengthMeters, widthMeters}, obstacles, 0.05)) {
                restore();
                return false;
            }
        } catch (...) {
            restore();
            throw;
        }
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
        throw std::invalid_argument(trafficInputError(
            "Traffic route-end state is invalid", "distanceMeters=", actor.distanceMeters,
            "routeTotalLength=", actor.route.totalLength, "closed=", actor.route.closed,
            "hasLoopStart=", actor.route.loopStartDistance.has_value(),
            "loopStart=", actor.route.loopStartDistance.value_or(0.0)));
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
        throw std::invalid_argument(trafficInputError(
            "Traffic entry visibility inputs are invalid", "radiusMeters=", radiusMeters,
            "minimumDistanceMeters=", minimumDistanceMeters, "positionX=", position[0],
            "positionY=", position[1], "positionZ=", position[2], "cameraX=", cameraPosition[0],
            "cameraY=", cameraPosition[1], "cameraZ=", cameraPosition[2]));
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

bool mapTrafficEntryHiddenFromCameras(const std::array<double, 3>& position, double radiusMeters,
                                      const MapTrafficCameraView& mainCamera,
                                      const std::vector<MapTrafficCameraView>& additionalCameras,
                                      double minimumDistanceMeters) {
    const auto hiddenFromCamera = [&](const MapTrafficCameraView& camera) {
        return mapTrafficEntryHiddenFromCamera(position, radiusMeters, camera.position,
                                               camera.viewProjection, minimumDistanceMeters);
    };
    return hiddenFromCamera(mainCamera) &&
           std::all_of(additionalCameras.begin(), additionalCameras.end(), hiddenFromCamera);
}

double mapTrafficCurveSpeedLimit(const MapTrafficRoute& route, double distanceMeters,
                                 double cruiseSpeedMetersPerSecond,
                                 double minimumTurnRadiusMeters) {
    if (!std::isfinite(distanceMeters) || !std::isfinite(cruiseSpeedMetersPerSecond) ||
        cruiseSpeedMetersPerSecond < 0.0 || !std::isfinite(minimumTurnRadiusMeters) ||
        minimumTurnRadiusMeters < 0.0) {
        throw std::invalid_argument(trafficInputError(
            "Traffic curve speed inputs are invalid", "distanceMeters=", distanceMeters,
            "cruiseSpeedMetersPerSecond=", cruiseSpeedMetersPerSecond,
            "minimumTurnRadiusMeters=", minimumTurnRadiusMeters));
    }
    return mapTrafficCurveSpeedLimitFromCurvature(mapTrafficRouteCurvatureAt(route, distanceMeters),
                                                  cruiseSpeedMetersPerSecond);
}

void advanceMapTrafficDriving(const std::vector<MapTrafficActor*>& actors,
                              const std::vector<MapTrafficObstacle>& obstacles,
                              double elapsedSeconds, double signalClockSeconds) {
    openbus::rendering::TraceScope trace("traffic", "MapTrafficRuntime::advanceMapTrafficDriving");
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0 ||
        !std::isfinite(signalClockSeconds) || signalClockSeconds < 0.0) {
        throw std::invalid_argument(trafficInputError("Traffic driving timestep is invalid",
                                                      "elapsedSeconds=", elapsedSeconds,
                                                      "signalClockSeconds=", signalClockSeconds));
    }
    std::unordered_set<MapTrafficActor*> unique;
    for (std::size_t index = 0; index < actors.size(); ++index) {
        auto* actor = actors[index];
        if (actor == nullptr) {
            throw std::invalid_argument(trafficInputError("Traffic driving actor state is invalid",
                                                          "actorIndex=", index,
                                                          "actorIsNull=", true));
        }
        if (!unique.insert(actor).second) {
            throw std::invalid_argument(
                trafficInputError("Traffic driving actor state is invalid", "actorIndex=", index,
                                  "duplicateActor=", static_cast<const void*>(actor)));
        }
        if (!std::isfinite(actor->distanceMeters) || actor->distanceMeters < 0.0 ||
            !std::isfinite(actor->speedMetersPerSecond) || actor->speedMetersPerSecond < 0.0 ||
            !std::isfinite(actor->targetSpeedMetersPerSecond) ||
            actor->targetSpeedMetersPerSecond < 0.0 ||
            !std::isfinite(actor->cruiseSpeedMetersPerSecond) ||
            actor->cruiseSpeedMetersPerSecond < 0.0 ||
            !std::isfinite(actor->minimumTurnRadiusMeters) ||
            actor->minimumTurnRadiusMeters < 0.0 || !std::isfinite(actor->junctionWaitSeconds) ||
            actor->junctionWaitSeconds < 0.0 ||
            (actor->scheduledStopDistanceMeters.has_value() &&
             !std::isfinite(*actor->scheduledStopDistanceMeters))) {
            throw std::invalid_argument(trafficInputError(
                "Traffic driving actor state is invalid", "actorIndex=", index, "distanceMeters=",
                actor->distanceMeters, "speedMetersPerSecond=", actor->speedMetersPerSecond,
                "targetSpeedMetersPerSecond=", actor->targetSpeedMetersPerSecond,
                "cruiseSpeedMetersPerSecond=", actor->cruiseSpeedMetersPerSecond,
                "minimumTurnRadiusMeters=", actor->minimumTurnRadiusMeters,
                "junctionWaitSeconds=", actor->junctionWaitSeconds,
                "hasScheduledStopDistance=", actor->scheduledStopDistanceMeters.has_value(),
                "scheduledStopDistanceMeters=", actor->scheduledStopDistanceMeters.value_or(0.0)));
        }
        validateFootprint(actorFootprint(*actor, actor->distanceMeters));
    }
    std::unordered_map<const MapTrafficActor*, std::size_t> actorIndexByPointer;
    actorIndexByPointer.reserve(actors.size());
    for (std::size_t index = 0; index < actors.size(); ++index) {
        actorIndexByPointer.emplace(actors[index], index);
    }
    // Some optimizations below skip or defer route-sampling queries that throw on malformed
    // routes. They are only enabled when no such query can fail; otherwise the queries run
    // eagerly in their original order so the same exception is raised at the same point.
    const bool deferrableSamplingQueries =
        std::all_of(actors.begin(), actors.end(), [](const MapTrafficActor* actor) {
            return mapTrafficRouteSamplingCannotFail(actor->route, actor->lateralOffsetMeters);
        });
    for (const auto& obstacle : obstacles) {
        validateFootprint(obstacle);
    }
    constexpr double acceleration = 1.6;
    constexpr double braking = trafficBrakingDeceleration;
    constexpr double headwaySeconds = 1.5;
    // Drivers keep a shorter time gap at crawling speeds, reaching the full headway at
    // urban cruising speed.
    const auto followingHeadwaySeconds = [&](double speedMetersPerSecond) {
        return std::clamp(0.5 + 0.075 * speedMetersPerSecond, 0.5, headwaySeconds);
    };
    constexpr double signalApproachDeceleration = 1.6;
    constexpr double followingResponseSeconds = 2.5;
    constexpr double standstillFollowingGapMeters = 2.0;
    constexpr double parkedVehicleFollowingGapMeters = 1.0;
    constexpr double junctionCheckSectionLengthMeters = 10.0;
    constexpr double trafficVerticalClearanceMeters = 3.0;
    constexpr double turningObstacleClearanceMeters = 0.15;
    // A short margin beyond the vehicles' predicted physical occupancy at a conflict.
    constexpr double priorityGapSeconds = 0.75;
    const auto trafficFootprintsOverlap = [](const MapTrafficObstacle& first,
                                             const MapTrafficObstacle& second,
                                             double clearanceMeters) {
        return footprintsOverlapWithBroadPhase(first, second, clearanceMeters);
    };
    struct MovementReservation {
        MapTrafficObstacle footprint;
        const MapTrafficActor* actor = nullptr;
        std::size_t actorIndex = 0;
        double distanceMeters = 0.0;
        // Computed on first use; the reserving actor does not change again this substep.
        std::optional<double> arrivalSeconds;
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
                   first.spline.placement.splineId == second.spline.placement.splineId &&
                   first.spline.tileX == second.spline.tileX &&
                   first.spline.tileY == second.spline.tileY;
        }
        return first.points == second.points;
    };
    const auto sameDirectedRouteValue = [&](const MapTrafficActor& first,
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
    const auto combineRouteHash = [](std::size_t& seed, std::size_t value) {
        seed ^= value + std::size_t{0x9e3779b9} + (seed << 6) + (seed >> 2);
    };
    // Arbitration revisits actor pairs across several passes; compare full routes once per pair.
    std::unordered_map<std::size_t, std::vector<std::size_t>> routeClassesByHash;
    std::vector<std::size_t> routeClassByActor(actors.size());
    routeClassesByHash.reserve(actors.size());
    openbus::rendering::TraceScope routeTrace(
        "traffic", "MapTrafficRuntime::advanceMapTrafficDriving.routeClassification");
    for (std::size_t actorIndex = 0; actorIndex < actors.size(); ++actorIndex) {
        const MapTrafficActor& actor = *actors[actorIndex];
        std::size_t routeHash = std::hash<double>{}(actor.lateralOffsetMeters);
        combineRouteHash(routeHash, std::hash<bool>{}(actor.route.closed));
        combineRouteHash(routeHash, std::hash<bool>{}(actor.route.loopStartDistance.has_value()));
        if (actor.route.loopStartDistance.has_value()) {
            combineRouteHash(routeHash, std::hash<double>{}(*actor.route.loopStartDistance));
        }
        combineRouteHash(routeHash, actor.route.segments.size());
        for (const MapTrafficRouteSegment& segment : actor.route.segments) {
            combineRouteHash(routeHash, std::hash<bool>{}(segment.reversed));
            combineRouteHash(routeHash, std::hash<double>{}(segment.pathLateralMeters));
            combineRouteHash(routeHash, std::hash<double>{}(segment.pathHeightMeters));
            combineRouteHash(routeHash, segment.points.empty() ? 0U : 1U);
            if (segment.points.empty()) {
                combineRouteHash(routeHash, std::hash<int>{}(segment.spline.placement.splineId));
                combineRouteHash(routeHash, std::hash<int>{}(segment.spline.tileX));
                combineRouteHash(routeHash, std::hash<int>{}(segment.spline.tileY));
            } else {
                for (const auto& point : segment.points) {
                    for (const double coordinate : point) {
                        combineRouteHash(routeHash, std::hash<double>{}(coordinate));
                    }
                }
            }
        }
        auto& matchingClasses = routeClassesByHash[routeHash];
        bool matchedExistingClass = false;
        for (const std::size_t representative : matchingClasses) {
            if (sameDirectedRouteValue(actor, *actors[representative])) {
                routeClassByActor[actorIndex] = routeClassByActor[representative];
                matchedExistingClass = true;
                break;
            }
        }
        if (!matchedExistingClass) {
            routeClassByActor[actorIndex] = actorIndex;
            matchingClasses.push_back(actorIndex);
        }
    }
    routeTrace.finish();
    std::vector<std::vector<std::size_t>> actorsByRouteClass(actors.size());
    for (std::size_t index = 0; index < actors.size(); ++index) {
        actorsByRouteClass[routeClassByActor[index]].push_back(index);
    }
    const auto sameDirectedFlow = [&](const MapTrafficActor& firstActor, double firstDistance,
                                      const MapTrafficRouteSegment& firstSegment,
                                      const MapTrafficPose& firstPose,
                                      const MapTrafficActor& secondActor, double secondDistance,
                                      const MapTrafficRouteSegment& secondSegment,
                                      const MapTrafficPose& secondPose) {
        if (firstActor.lateralOffsetMeters == secondActor.lateralOffsetMeters &&
            sameDirectedSegment(firstSegment, secondSegment)) {
            return true;
        }
        if (!firstSegment.points.empty() || !secondSegment.points.empty()) {
            return false;
        }
        const auto projectsToSameFlow = [&](const MapTrafficActor& pathActor, double pathDistance,
                                            const MapTrafficPose& pathOrigin,
                                            const MapTrafficActor& targetActor,
                                            const MapTrafficPose& targetPose) {
            if (std::abs(pathOrigin.position[2] - targetPose.position[2]) >
                trafficVerticalClearanceMeters) {
                return false;
            }
            double along = (targetPose.position[0] - pathOrigin.position[0]) *
                               std::sin(pathOrigin.headingRadians) +
                           (targetPose.position[1] - pathOrigin.position[1]) *
                               std::cos(pathOrigin.headingRadians);
            if (std::abs(along) > 30.0) {
                return false;
            }
            std::optional<MapTrafficPose> projected;
            for (int refinement = 0; refinement < 4; ++refinement) {
                const double projectedDistance = pathDistance + along;
                if ((!pathActor.route.closed && projectedDistance < 0.0) ||
                    (!pathActor.route.closed && !pathActor.route.loopStartDistance.has_value() &&
                     projectedDistance > pathActor.route.totalLength)) {
                    return false;
                }
                projected = sampleMapTrafficActorPose(pathActor, projectedDistance);
                if (!projected.has_value()) {
                    return false;
                }
                const double dx = targetPose.position[0] - projected->position[0];
                const double dy = targetPose.position[1] - projected->position[1];
                along += dx * std::sin(projected->headingRadians) +
                         dy * std::cos(projected->headingRadians);
            }
            const double finalDistance = pathDistance + along;
            if (std::abs(along) > 30.0 || (!pathActor.route.closed && finalDistance < 0.0) ||
                (!pathActor.route.closed && !pathActor.route.loopStartDistance.has_value() &&
                 finalDistance > pathActor.route.totalLength)) {
                return false;
            }
            projected = sampleMapTrafficActorPose(pathActor, finalDistance);
            if (!projected.has_value() ||
                std::cos(projected->headingRadians - targetPose.headingRadians) < 0.7) {
                return false;
            }
            const double dx = targetPose.position[0] - projected->position[0];
            const double dy = targetPose.position[1] - projected->position[1];
            const double lateral = std::abs(dx * std::cos(projected->headingRadians) -
                                            dy * std::sin(projected->headingRadians));
            return lateral <= (pathActor.widthMeters + targetActor.widthMeters) * 0.5 - 0.3;
        };
        return projectsToSameFlow(firstActor, firstDistance, firstPose, secondActor, secondPose) ||
               projectsToSameFlow(secondActor, secondDistance, secondPose, firstActor, firstPose);
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
        const auto isJunctionPath = [](const MapTrafficRouteSegment& segment) {
            return !segment.points.empty() || segment.pathRules.roundabout;
        };
        std::size_t endIndex = segmentIndex;
        while (true) {
            std::size_t next = endIndex + 1;
            while (next < segments.size() && !isJunctionPath(segments[next]) &&
                   ends[next] - ends[endIndex] < candidate.lengthMeters + 4.0) {
                ++next;
            }
            if (next == segments.size() || !isJunctionPath(segments[next])) {
                return endIndex;
            }
            endIndex = next;
        }
    };
    const auto junctionClaimEnd = [&](const MapTrafficActor& candidate, std::size_t endIndex) {
        constexpr double junctionExitClearanceMeters = 1.0;
        return candidate.route.cumulativeEndDistances[endIndex] + candidate.lengthMeters * 0.5 +
               junctionExitClearanceMeters;
    };
    // Queue order is based on the vehicles' current poses so source-spline boundaries do not
    // determine whether a junction path can be claimed.
    const auto queuedBehindInLane = [&](const MapTrafficActor& candidate,
                                        const MapTrafficObstacle& candidateFootprint,
                                        const MapTrafficActor& other,
                                        const MapTrafficObstacle& otherFootprint) {
        const double headingDifference =
            candidateFootprint.pose.headingRadians - otherFootprint.pose.headingRadians;
        if (std::cos(headingDifference) < 0.7 ||
            std::abs(candidateFootprint.pose.position[2] - otherFootprint.pose.position[2]) > 3.0) {
            return false;
        }
        const double dx = otherFootprint.pose.position[0] - candidateFootprint.pose.position[0];
        const double dy = otherFootprint.pose.position[1] - candidateFootprint.pose.position[1];
        const double heading = candidateFootprint.pose.headingRadians;
        const double longitudinal = dx * std::sin(heading) + dy * std::cos(heading);
        const double lateral = std::abs(dx * std::cos(heading) - dy * std::sin(heading));
        const double laneHalfWidth = (candidate.widthMeters + other.widthMeters) * 0.5 + 0.5;
        return longitudinal < -1.0e-6 && longitudinal >= -100.0 && lateral <= laneHalfWidth;
    };
    const auto currentSegmentIndex = [](const MapTrafficActor& candidate) {
        return static_cast<std::size_t>(
            &routeSegmentAtDistance(candidate.route, candidate.distanceMeters) -
            candidate.route.segments.data());
    };
    // Use the lowest authored priority in the vehicle's nearby route so a short connector
    // cannot hide a lower-priority merge from traffic arbitration.
    const auto approachPriority = [&](const MapTrafficActor& candidate) {
        const auto& segments = candidate.route.segments;
        const MapTrafficVehicleFilter vehicle{candidate.vehicleType,
                                              candidate.unscheduledGroupIndex, candidate.scheduled,
                                              candidate.trafficRuleGroupIndex};
        const std::size_t current = currentSegmentIndex(candidate);
        if (segments[current].pathRules.roundabout) {
            return 256.0;
        }
        constexpr double approachPriorityLookaheadMeters = 150.0;
        const auto priorityForSegment = [&](std::size_t index) -> std::optional<double> {
            const MapTrafficPathRules& rules = segments[index].pathRules;
            if (!segments[index].points.empty() || rules.roundabout) {
                return std::nullopt;
            }
            return mapTrafficPathPriorityForVehicle(rules, vehicle);
        };
        double priority = mapTrafficPathPriorityForVehicle(segments[current].pathRules, vehicle);
        for (std::size_t back = 1; back <= current; ++back) {
            const std::size_t index = current - back;
            if (!segments[index].points.empty() ||
                candidate.distanceMeters - candidate.route.cumulativeEndDistances[index] >
                    approachPriorityLookaheadMeters) {
                break;
            }
            if (const auto nearbyPriority = priorityForSegment(index)) {
                priority = std::min(priority, *nearbyPriority);
            }
        }
        for (std::size_t index = current + 1; index < segments.size(); ++index) {
            if (!segments[index].points.empty() ||
                candidate.route.cumulativeEndDistances[index - 1] - candidate.distanceMeters >
                    approachPriorityLookaheadMeters) {
                break;
            }
            if (const auto nearbyPriority = priorityForSegment(index)) {
                priority = std::min(priority, *nearbyPriority);
            }
        }
        return priority;
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
        const double curveSpeed = mapTrafficCurveSpeedLimit(candidate.route, distance,
                                                            candidate.cruiseSpeedMetersPerSecond,
                                                            candidate.minimumTurnRadiusMeters);
        return travelSeconds(travelDistance, candidate.speedMetersPerSecond,
                             std::min(candidate.cruiseSpeedMetersPerSecond, curveSpeed));
    };
    const auto previewDistance = [&](const MapTrafficActor& candidate) {
        const bool physicallyHeld = candidate.speedMetersPerSecond < 0.3 &&
                                    (candidate.blockReason == MapTrafficBlockReason::Signal ||
                                     candidate.blockReason == MapTrafficBlockReason::Obstacle ||
                                     candidate.blockReason == MapTrafficBlockReason::Leader ||
                                     candidate.blockReason == MapTrafficBlockReason::Vehicle ||
                                     candidate.blockReason == MapTrafficBlockReason::Collision);
        if (physicallyHeld || candidate.cruiseSpeedMetersPerSecond == 0.0) {
            return 0.0;
        }
        // A rolling conflict window, independent of authored spline/junction length.
        return std::max(
            {15.0, candidate.speedMetersPerSecond * 3.0 + candidate.lengthMeters,
             candidate.speedMetersPerSecond * candidate.speedMetersPerSecond / (2.0 * braking) +
                 candidate.lengthMeters + 2.0});
    };
    using JunctionEdge = std::pair<std::array<double, 3>, std::array<double, 3>>;
    // Remaining junction edges depend only on the route and these actor fields, so they are
    // reused across the many arbitration and admission pairs while the actor is unchanged.
    struct JunctionEdgeCache {
        bool valid = false;
        double distanceMeters = 0.0;
        double speedMetersPerSecond = 0.0;
        double cruiseSpeedMetersPerSecond = 0.0;
        double lengthMeters = 0.0;
        double lateralOffsetMeters = 0.0;
        double rearAxleOffsetMeters = 0.0;
        MapTrafficBlockReason blockReason = MapTrafficBlockReason::None;
        std::vector<JunctionEdge> edges;
        std::array<double, 4> bounds = {}; // minX, minY, maxX, maxY of the edge endpoints
        double positionUncertaintyMeters = 0.0;
        bool exactStraightPaths = true;
    };
    std::vector<JunctionEdgeCache> junctionEdgeCaches(actors.size());
    struct StraightActorRouteCache {
        std::size_t firstPath;
        std::size_t endPath;
        bool straight;
    };
    std::vector<std::optional<StraightActorRouteCache>> straightActorRoutes(actors.size());
    const auto remainingJunctionEdges =
        [&](const MapTrafficActor& candidate,
            JunctionEdgeCache& scratch) -> const JunctionEdgeCache& {
        const auto indexed = actorIndexByPointer.find(&candidate);
        JunctionEdgeCache& cache =
            indexed != actorIndexByPointer.end() ? junctionEdgeCaches[indexed->second] : scratch;
        if (cache.valid && cache.distanceMeters == candidate.distanceMeters &&
            cache.speedMetersPerSecond == candidate.speedMetersPerSecond &&
            cache.cruiseSpeedMetersPerSecond == candidate.cruiseSpeedMetersPerSecond &&
            cache.lengthMeters == candidate.lengthMeters &&
            cache.lateralOffsetMeters == candidate.lateralOffsetMeters &&
            cache.rearAxleOffsetMeters == candidate.rearAxleOffsetMeters &&
            cache.blockReason == candidate.blockReason) {
            return cache;
        }
        cache.valid = true;
        cache.distanceMeters = candidate.distanceMeters;
        cache.speedMetersPerSecond = candidate.speedMetersPerSecond;
        cache.cruiseSpeedMetersPerSecond = candidate.cruiseSpeedMetersPerSecond;
        cache.lengthMeters = candidate.lengthMeters;
        cache.lateralOffsetMeters = candidate.lateralOffsetMeters;
        cache.rearAxleOffsetMeters = candidate.rearAxleOffsetMeters;
        cache.blockReason = candidate.blockReason;
        auto& edges = cache.edges;
        edges.clear();
        cache.bounds = {
            std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
        cache.positionUncertaintyMeters = 0.0;
        cache.exactStraightPaths = true;
        const auto& route = candidate.route;
        const double tail =
            std::max(0.0, candidate.distanceMeters - candidate.lengthMeters * 0.5 - 0.5);
        const double previewEnd =
            candidate.distanceMeters + std::max(15.0, previewDistance(candidate));
        std::size_t firstPath = 0;
        while (firstPath < route.segments.size()) {
            const double start = firstPath == 0 ? 0.0 : route.cumulativeEndDistances[firstPath - 1];
            if (start > previewEnd) {
                return cache;
            }
            if (!route.segments[firstPath].points.empty() &&
                route.cumulativeEndDistances[firstPath] >= tail) {
                break;
            }
            ++firstPath;
        }
        if (firstPath == route.segments.size()) {
            return cache;
        }
        const std::size_t endPath = junctionGroupEndIndex(candidate, firstPath);
        const auto routeIsStraight = [&]() {
            if (candidate.rearAxleOffsetMeters == 0.0) {
                return true;
            }
            if (route.closed || route.loopStartDistance.has_value()) {
                return false;
            }
            const double corridorStart =
                firstPath == 0 ? 0.0 : route.cumulativeEndDistances[firstPath - 1];
            const double padding = std::abs(candidate.rearAxleOffsetMeters) + 2.0;
            const double rangeStart = std::max(0.0, corridorStart - padding);
            const double rangeEnd =
                std::min(route.totalLength, route.cumulativeEndDistances[endPath] + padding);
            const auto first = std::lower_bound(route.cumulativeEndDistances.begin(),
                                                route.cumulativeEndDistances.end(), rangeStart);
            const std::size_t firstSegment =
                static_cast<std::size_t>(first - route.cumulativeEndDistances.begin());
            const double firstStart =
                firstSegment == 0 ? 0.0 : route.cumulativeEndDistances[firstSegment - 1];
            const double firstEnd = route.cumulativeEndDistances[firstSegment];
            const double originDistance =
                firstStart + std::min(1.0e-4, (firstEnd - firstStart) * 0.25);
            const auto origin =
                sampleMapTrafficRouteRaw(route, originDistance, candidate.lateralOffsetMeters);
            if (!origin || std::abs(origin->pitchRadians) > 1.0e-8) {
                return false;
            }
            const double dx = std::sin(origin->headingRadians);
            const double dy = std::cos(origin->headingRadians);
            for (std::size_t index = firstSegment; index < route.segments.size(); ++index) {
                const auto& segment = route.segments[index];
                const auto& spline = segment.spline.placement;
                if ((!segment.points.empty() && segment.points.size() != 2) ||
                    (segment.points.empty() &&
                     (spline.radius != 0.0 || spline.gradientStart != 0.0 ||
                      spline.gradientEnd != 0.0 || spline.heightDelta != 0.0 ||
                      spline.skewStart != spline.skewEnd || spline.cantStart != spline.cantEnd))) {
                    return false;
                }
                const double start = index == 0 ? 0.0 : route.cumulativeEndDistances[index - 1];
                const double end = route.cumulativeEndDistances[index];
                if (start > rangeEnd) {
                    break;
                }
                const double inset = std::min(1.0e-4, (end - start) * 0.25);
                for (const double distance : {start + inset, (start + end) * 0.5, end - inset}) {
                    const auto pose =
                        sampleMapTrafficRouteRaw(route, distance, candidate.lateralOffsetMeters);
                    if (!pose || std::abs(pose->pitchRadians) > 1.0e-8 ||
                        std::abs(std::remainder(pose->headingRadians - origin->headingRadians,
                                                2.0 * std::numbers::pi)) > 1.0e-8 ||
                        std::hypot(pose->position[0] - origin->position[0] -
                                       dx * (distance - originDistance),
                                   pose->position[1] - origin->position[1] -
                                       dy * (distance - originDistance)) > 1.0e-5 ||
                        std::abs(pose->position[2] - origin->position[2]) > 1.0e-5) {
                        return false;
                    }
                }
            }
            return true;
        };
        bool straightActorRoute = candidate.rearAxleOffsetMeters == 0.0;
        if (!straightActorRoute) {
            if (indexed != actorIndexByPointer.end()) {
                auto& cached = straightActorRoutes[indexed->second];
                if (!cached.has_value() || cached->firstPath != firstPath ||
                    cached->endPath != endPath) {
                    cached = StraightActorRouteCache{firstPath, endPath, routeIsStraight()};
                }
                straightActorRoute = cached->straight;
            } else {
                straightActorRoute = routeIsStraight();
            }
        }
        for (std::size_t index = firstPath; index <= endPath; ++index) {
            const auto& points = route.segments[index].points;
            const bool exactStraight = points.size() == 2 && straightActorRoute;
            if (!exactStraight) {
                cache.exactStraightPaths = false;
                cache.positionUncertaintyMeters = std::abs(candidate.lateralOffsetMeters) +
                                                  2.0 * std::abs(candidate.rearAxleOffsetMeters);
            }
            double distance = index == 0 ? 0.0 : route.cumulativeEndDistances[index - 1];
            for (std::size_t point = 1; point < points.size(); ++point) {
                const auto& from = points[point - 1];
                const auto& to = points[point];
                const double length = std::hypot(to[0] - from[0], to[1] - from[1], to[2] - from[2]);
                if (length > 1.0e-6 && distance + length > tail) {
                    auto clipped = from;
                    auto adjustedTo = to;
                    const double fraction = std::clamp((tail - distance) / length, 0.0, 1.0);
                    for (std::size_t axis = 0; axis < clipped.size(); ++axis) {
                        clipped[axis] += (to[axis] - from[axis]) * fraction;
                    }
                    if (exactStraight) {
                        const double heading = std::atan2(to[0] - from[0], to[1] - from[1]);
                        const auto offset =
                            mapSplineRightOffset(heading, candidate.lateralOffsetMeters);
                        for (std::size_t axis = 0; axis < 2; ++axis) {
                            clipped[axis] += offset[axis];
                            adjustedTo[axis] += offset[axis];
                        }
                    }
                    edges.emplace_back(clipped, adjustedTo);
                    const auto& clippedFrom = clipped;
                    const auto& clippedTo = adjustedTo;
                    for (const auto* endpoint : {&clippedFrom, &clippedTo}) {
                        cache.bounds[0] = std::min(cache.bounds[0], (*endpoint)[0]);
                        cache.bounds[1] = std::min(cache.bounds[1], (*endpoint)[1]);
                        cache.bounds[2] = std::max(cache.bounds[2], (*endpoint)[0]);
                        cache.bounds[3] = std::max(cache.bounds[3], (*endpoint)[1]);
                    }
                }
                distance += length;
            }
        }
        return cache;
    };
    const auto opposingJunctionMovements = [&](const MapTrafficActor& first,
                                               const MapTrafficActor& second) {
        JunctionEdgeCache firstScratch;
        JunctionEdgeCache secondScratch;
        const JunctionEdgeCache& firstCache = remainingJunctionEdges(first, firstScratch);
        if (firstCache.edges.empty()) {
            return false;
        }
        const JunctionEdgeCache& secondCache = remainingJunctionEdges(second, secondScratch);
        if (secondCache.edges.empty()) {
            return false;
        }
        const double nonPassingWidth = (first.widthMeters + second.widthMeters) * 0.5 + 0.5 +
                                       firstCache.positionUncertaintyMeters +
                                       secondCache.positionUncertaintyMeters;
        // A shared corridor keeps both edges within nonPassingWidth of each other.
        if (firstCache.bounds[0] - nonPassingWidth > secondCache.bounds[2] ||
            secondCache.bounds[0] - nonPassingWidth > firstCache.bounds[2] ||
            firstCache.bounds[1] - nonPassingWidth > secondCache.bounds[3] ||
            secondCache.bounds[1] - nonPassingWidth > firstCache.bounds[3]) {
            return false;
        }
        const auto& firstEdges = firstCache.edges;
        const auto& secondEdges = secondCache.edges;
        for (const auto& [firstFrom, firstTo] : firstEdges) {
            const double dx = firstTo[0] - firstFrom[0];
            const double dy = firstTo[1] - firstFrom[1];
            const double length = std::hypot(dx, dy);
            if (length < 0.5) {
                continue;
            }
            const double ux = dx / length;
            const double uy = dy / length;
            for (const auto& [secondFrom, secondTo] : secondEdges) {
                const double sx = secondTo[0] - secondFrom[0];
                const double sy = secondTo[1] - secondFrom[1];
                const double secondLength = std::hypot(sx, sy);
                if (secondLength < 0.5) {
                    continue;
                }
                const double directionDot = (ux * sx + uy * sy) / secondLength;
                if (directionDot > -0.98) {
                    continue;
                }
                const double edgeWidth =
                    firstCache.exactStraightPaths && secondCache.exactStraightPaths &&
                            std::abs((ux * sy - uy * sx) / secondLength) < 1.0e-12
                        ? nonPassingWidth - 0.25
                        : nonPassingWidth;
                const auto lateral = [&](const auto& point) {
                    return std::abs((point[0] - firstFrom[0]) * uy -
                                    (point[1] - firstFrom[1]) * ux);
                };
                if (lateral(secondFrom) > edgeWidth || lateral(secondTo) > edgeWidth) {
                    continue;
                }
                const double from =
                    (secondFrom[0] - firstFrom[0]) * ux + (secondFrom[1] - firstFrom[1]) * uy;
                const double to =
                    (secondTo[0] - firstFrom[0]) * ux + (secondTo[1] - firstFrom[1]) * uy;
                const double overlapStart = std::max(0.0, std::min(from, to));
                const double overlapEnd = std::min(length, std::max(from, to));
                if (overlapEnd - overlapStart < 0.5) {
                    continue;
                }
                const double overlapMiddle = (overlapStart + overlapEnd) * 0.5;
                const double firstHeight =
                    firstFrom[2] + (firstTo[2] - firstFrom[2]) * overlapMiddle / length;
                const double secondHeight = secondFrom[2] + (secondTo[2] - secondFrom[2]) *
                                                                (overlapMiddle - from) /
                                                                (to - from);
                if (std::abs(firstHeight - secondHeight) <= trafficVerticalClearanceMeters) {
                    return true;
                }
            }
        }
        return false;
    };
    // Distances remain unwrapped while planning across a loop seam.
    const auto nextJunctionRange =
        [&](const MapTrafficActor& candidate) -> std::optional<std::pair<double, double>> {
        const bool loops = candidate.route.closed || candidate.route.loopStartDistance.has_value();
        const double loopStart = candidate.route.loopStartDistance.value_or(0.0);
        const double cycleLength = candidate.route.totalLength - loopStart;
        for (int cycle = 0; cycle < (loops ? 2 : 1); ++cycle) {
            const double base = cycle * cycleLength;
            for (std::size_t segmentIndex = 0; segmentIndex < candidate.route.segments.size();
                 ++segmentIndex) {
                const double segmentStart =
                    segmentIndex == 0 ? 0.0
                                      : candidate.route.cumulativeEndDistances[segmentIndex - 1];
                if ((candidate.route.segments[segmentIndex].points.empty() &&
                     !candidate.route.segments[segmentIndex].pathRules.roundabout) ||
                    (cycle > 0 && segmentStart < loopStart) ||
                    base + candidate.route.cumulativeEndDistances[segmentIndex] <
                        candidate.distanceMeters) {
                    continue;
                }
                return std::pair{base + segmentStart,
                                 base + junctionClaimEnd(candidate, junctionGroupEndIndex(
                                                                        candidate, segmentIndex))};
            }
        }
        return std::nullopt;
    };
    // Remaining junction path of an actor that is inside, or admitted into, a junction.
    const auto activeJunctionClaimRange =
        [&](const MapTrafficActor& candidate) -> std::optional<std::pair<double, double>> {
        if (mapTrafficActorOnJunctionPath(candidate, candidate.distanceMeters)) {
            for (double offset :
                 {0.0, candidate.lengthMeters * 0.5, -candidate.lengthMeters * 0.5}) {
                const double referenceDistance = candidate.distanceMeters + offset;
                const auto& segment = routeSegmentAtDistance(candidate.route, referenceDistance);
                if (segment.points.empty() && !segment.pathRules.roundabout) {
                    continue;
                }
                const std::size_t index =
                    static_cast<std::size_t>(&segment - candidate.route.segments.data());
                double cycleBase = 0.0;
                if (candidate.route.closed) {
                    cycleBase = std::floor(referenceDistance / candidate.route.totalLength) *
                                candidate.route.totalLength;
                } else if (candidate.route.loopStartDistance &&
                           referenceDistance >= candidate.route.totalLength) {
                    const double cycleLength =
                        candidate.route.totalLength - *candidate.route.loopStartDistance;
                    cycleBase =
                        std::floor((referenceDistance - *candidate.route.loopStartDistance) /
                                   cycleLength) *
                        cycleLength;
                }
                const double end =
                    std::min(cycleBase + junctionClaimEnd(candidate,
                                                          junctionGroupEndIndex(candidate, index)),
                             candidate.distanceMeters + previewDistance(candidate));
                return std::pair{candidate.distanceMeters, std::max(candidate.distanceMeters, end)};
            }
        }
        if (candidate.junctionClaimEndMeters.has_value() &&
            candidate.distanceMeters < *candidate.junctionClaimEndMeters) {
            return std::pair{candidate.distanceMeters,
                             std::min(*candidate.junctionClaimEndMeters,
                                      candidate.distanceMeters + previewDistance(candidate))};
        }
        return std::nullopt;
    };
    struct JunctionClaim {
        struct Section {
            std::size_t firstSample = 0;
            std::size_t sampleCount = 0;
            double startDistance = 0.0;
            std::array<double, 4> bounds = {};
        };

        const MapTrafficActor* actor = nullptr;
        std::size_t actorIndex = 0;
        std::vector<double> distances;
        std::vector<MapTrafficObstacle> footprints;
        std::vector<const MapTrafficRouteSegment*> segments;
        std::vector<Section> sections;
        std::array<double, 4> bounds = {}; // minX, minY, maxX, maxY including body extents
        // Lazily computed arrival time and half-body occupancy time for each sample.
        mutable std::vector<char> arrivalComputed;
        mutable std::vector<std::pair<double, double>> arrival;
    };
    const auto cachedClaimFootprint = [](const JunctionClaim* claim,
                                         double distance) -> const MapTrafficObstacle* {
        if (claim == nullptr) {
            return nullptr;
        }
        const auto found =
            std::lower_bound(claim->distances.begin(), claim->distances.end(), distance);
        if (found == claim->distances.end() || *found != distance) {
            return nullptr;
        }
        return &claim->footprints[static_cast<std::size_t>(found - claim->distances.begin())];
    };
    const auto makeJunctionClaim = [&](const MapTrafficActor& candidate, double from, double to,
                                       double clockSeconds, const JunctionClaim* cached = nullptr) {
        JunctionClaim claim;
        claim.actor = &candidate;
        if (!candidate.route.closed && !candidate.route.loopStartDistance) {
            to = std::min(to, candidate.route.totalLength);
        }
        if (const auto signal = nextTrafficSignalApproach(candidate)) {
            const auto state = trafficSignalStateAtTime(*signal->schedule, clockSeconds);
            if (!state.has_value() || *state < 6 || *state > 8) {
                to = std::min(to, candidate.distanceMeters + signal->distanceToStopMeters);
            }
        }
        from = std::min(from, to);
        claim.bounds = {
            std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
        const double radius = std::hypot(candidate.lengthMeters, candidate.widthMeters) * 0.5;
        for (double distance = from;; distance = std::min(distance + 1.0, to)) {
            if (claim.sections.empty() || distance - claim.sections.back().startDistance >=
                                              junctionCheckSectionLengthMeters) {
                JunctionClaim::Section section;
                section.firstSample = claim.footprints.size();
                section.startDistance = distance;
                section.bounds = {std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::infinity(),
                                  -std::numeric_limits<double>::infinity(),
                                  -std::numeric_limits<double>::infinity()};
                claim.sections.push_back(section);
            }
            JunctionClaim::Section& section = claim.sections.back();
            const auto* cachedFootprint = cached != nullptr && cached->actor == &candidate
                                              ? cachedClaimFootprint(cached, distance)
                                              : nullptr;
            claim.footprints.push_back(cachedFootprint != nullptr
                                           ? *cachedFootprint
                                           : actorFootprint(candidate, distance));
            claim.distances.push_back(distance);
            claim.segments.push_back(&routeSegmentAtDistance(candidate.route, distance));
            ++section.sampleCount;
            const auto& position = claim.footprints.back().pose.position;
            for (auto* bounds : {&claim.bounds, &section.bounds}) {
                (*bounds)[0] = std::min((*bounds)[0], position[0] - radius);
                (*bounds)[1] = std::min((*bounds)[1], position[1] - radius);
                (*bounds)[2] = std::max((*bounds)[2], position[0] + radius);
                (*bounds)[3] = std::max((*bounds)[3], position[1] + radius);
            }
            if (distance >= to) {
                break;
            }
        }
        return claim;
    };
    const auto boundsOverlap = [](const std::array<double, 4>& first,
                                  const std::array<double, 4>& second) {
        return first[0] <= second[2] && second[0] <= first[2] && first[1] <= second[3] &&
               second[1] <= first[3];
    };
    struct OccupancyCell {
        std::int64_t x = 0;
        std::int64_t y = 0;

        bool operator==(const OccupancyCell&) const = default;
    };
    struct OccupancyCellHash {
        std::size_t operator()(const OccupancyCell& cell) const noexcept {
            std::size_t seed = std::hash<std::int64_t>{}(cell.x);
            seed ^= std::hash<std::int64_t>{}(cell.y) + std::size_t{0x9e3779b9} + (seed << 6) +
                    (seed >> 2);
            return seed;
        }
    };
    constexpr double occupancyCellSizeMeters = 12.0;
    constexpr std::size_t routeDistanceIndexThreshold = 8;
    std::vector<std::vector<std::size_t>> routeActorsByDistance(actors.size());
    const auto cellForPosition = [](double x, double y,
                                    double cellSizeMeters) -> std::optional<OccupancyCell> {
        const double cellX = std::floor(x / cellSizeMeters);
        const double cellY = std::floor(y / cellSizeMeters);
        constexpr double minimumCell = -9.0e18;
        constexpr double maximumCell = 9.0e18;
        if (!std::isfinite(cellX) || !std::isfinite(cellY) || cellX < minimumCell ||
            cellX > maximumCell || cellY < minimumCell || cellY > maximumCell) {
            return std::nullopt;
        }
        return OccupancyCell{static_cast<std::int64_t>(cellX), static_cast<std::int64_t>(cellY)};
    };
    const auto occupancyCellForPosition = [&](double x, double y) {
        return cellForPosition(x, y, occupancyCellSizeMeters);
    };
    for (double remaining = elapsedSeconds; remaining > 1.0e-9;) {
        openbus::rendering::TraceScope stepTrace(
            "traffic", "MapTrafficRuntime::advanceMapTrafficDriving.substep");
        const double step = std::min(remaining, 0.05);
        remaining -= step;
        const double currentSignalClockSeconds = signalClockSeconds + elapsedSeconds - remaining;
        std::vector<MapTrafficObstacle> occupied = obstacles;
        std::vector<MovementReservation> reservations;
        std::vector<JunctionClaim> junctionClaims;
        openbus::rendering::TraceScope occupancyTrace(
            "traffic", "MapTrafficRuntime::advanceMapTrafficDriving.substepOccupancyAndClaims");
        openbus::rendering::TraceScope footprintTrace(
            "traffic", "MapTrafficRuntime::advanceMapTrafficDriving.substepFootprintBuild");
        for (const auto* actor : actors) {
            occupied.push_back(actorFootprint(*actor, actor->distanceMeters));
        }
        footprintTrace.finish();
        double maximumActorTravelMeters = 0.0;
        for (const MapTrafficActor* actor : actors) {
            maximumActorTravelMeters =
                std::max(maximumActorTravelMeters,
                         (actor->speedMetersPerSecond + acceleration * step) * step);
        }
        for (std::size_t routeClass = 0; routeClass < actorsByRouteClass.size(); ++routeClass) {
            auto& sortedActors = routeActorsByDistance[routeClass];
            sortedActors.clear();
            const auto& routeClassActors = actorsByRouteClass[routeClass];
            if (routeClassActors.size() < routeDistanceIndexThreshold) {
                continue;
            }
            const MapTrafficRoute& route = actors[routeClassActors.front()]->route;
            if (route.closed || route.loopStartDistance.has_value()) {
                continue;
            }
            sortedActors = routeClassActors;
            std::sort(sortedActors.begin(), sortedActors.end(),
                      [&](std::size_t first, std::size_t second) {
                          const double firstDistance = actors[first]->distanceMeters;
                          const double secondDistance = actors[second]->distanceMeters;
                          return firstDistance == secondDistance ? first < second
                                                                 : firstDistance < secondDistance;
                      });
        }
        openbus::rendering::TraceScope activeClaimsTrace(
            "traffic", "MapTrafficRuntime::advanceMapTrafficDriving.substepActiveClaims");
        for (std::size_t candidateIndex = 0; candidateIndex < actors.size(); ++candidateIndex) {
            const MapTrafficActor& candidate = *actors[candidateIndex];
            if (const auto range = activeJunctionClaimRange(candidate)) {
                junctionClaims.push_back(makeJunctionClaim(candidate, range->first, range->second,
                                                           currentSignalClockSeconds));
                junctionClaims.back().actorIndex = candidateIndex;
            }
        }
        activeClaimsTrace.finish();
        occupancyTrace.finish();
        std::unordered_map<OccupancyCell, std::vector<std::size_t>, OccupancyCellHash>
            occupancyCells;
        std::unordered_map<OccupancyCell, std::vector<std::size_t>, OccupancyCellHash>
            reservationCells;
        double largestReservationRadiusMeters = 0.0;
        bool reservationIndexValid = true;
        occupancyCells.reserve(occupied.size());
        std::vector<OccupancyCell> occupancyCellByIndex(occupied.size());
        std::vector<std::size_t> occupancySlotByIndex(occupied.size());
        // Actor dimensions and obstacle dimensions do not change during this substep.
        std::vector<double> occupiedDiameters;
        occupiedDiameters.reserve(occupied.size());
        for (const MapTrafficObstacle& footprint : occupied) {
            occupiedDiameters.push_back(std::hypot(footprint.lengthMeters, footprint.widthMeters));
        }
        double largestFootprintRadiusMeters = 0.0;
        bool occupancyIndexValid = true;
        {
            openbus::rendering::TraceScope occupancyIndexTrace(
                "traffic", "MapTrafficRuntime::advanceMapTrafficDriving.substepOccupancyIndex");
            for (std::size_t index = 0; index < occupied.size(); ++index) {
                const MapTrafficObstacle& footprint = occupied[index];
                largestFootprintRadiusMeters =
                    std::max(largestFootprintRadiusMeters, occupiedDiameters[index] * 0.5);
                const auto cell = occupancyCellForPosition(footprint.pose.position[0],
                                                           footprint.pose.position[1]);
                if (!cell) {
                    occupancyIndexValid = false;
                    break;
                }
                auto& members = occupancyCells[*cell];
                occupancyCellByIndex[index] = *cell;
                occupancySlotByIndex[index] = members.size();
                members.push_back(index);
            }
        }
        // Candidates are returned in index order so the first conflict, and any diagnostics
        // recorded while scanning, match a full linear scan.
        const auto collectNearbyIndexed =
            [&](const MapTrafficObstacle& footprint, double footprintDiameter,
                double clearanceMeters, const auto& cells, double largestRadiusMeters,
                bool indexValid, std::size_t count, std::vector<std::size_t>& out,
                bool radialClearance = false,
                std::optional<std::size_t> excludedReservationRoute = std::nullopt) {
                out.clear();
                const auto collectAll = [&] {
                    if (excludedReservationRoute) {
                        for (std::size_t index = 0; index < count; ++index) {
                            if (routeClassByActor[reservations[index].actorIndex] !=
                                *excludedReservationRoute) {
                                out.push_back(index);
                            }
                        }
                    } else {
                        out.resize(count);
                        std::iota(out.begin(), out.end(), 0);
                    }
                };
                if (!indexValid) {
                    collectAll();
                    return;
                }
                double radius =
                    footprintDiameter * 0.5 + largestRadiusMeters +
                    (radialClearance ? 1.0 : std::numbers::sqrt2) * std::max(0.0, clearanceMeters);
                if (radialClearance) {
                    radius *= 1.0 + 8.0 * std::numeric_limits<double>::epsilon();
                }
                const double x = footprint.pose.position[0];
                const double y = footprint.pose.position[1];
                const auto minimum = occupancyCellForPosition(x - radius, y - radius);
                const auto maximum = occupancyCellForPosition(x + radius, y + radius);
                if (!minimum || !maximum ||
                    static_cast<long double>(maximum->x) - minimum->x > 256.0L ||
                    static_cast<long double>(maximum->y) - minimum->y > 256.0L) {
                    collectAll();
                    return;
                }
                for (std::int64_t cellX = minimum->x; cellX <= maximum->x; ++cellX) {
                    for (std::int64_t cellY = minimum->y; cellY <= maximum->y; ++cellY) {
                        const auto cell = cells.find({cellX, cellY});
                        if (cell != cells.end()) {
                            if (excludedReservationRoute) {
                                for (const std::size_t index : cell->second) {
                                    if (routeClassByActor[reservations[index].actorIndex] !=
                                        *excludedReservationRoute) {
                                        out.push_back(index);
                                    }
                                }
                            } else {
                                out.insert(out.end(), cell->second.begin(), cell->second.end());
                            }
                        }
                    }
                }
                std::sort(out.begin(), out.end());
            };
        const auto updateOccupiedFootprint = [&](std::size_t index,
                                                 const MapTrafficObstacle& footprint) {
            const auto cell =
                occupancyCellForPosition(footprint.pose.position[0], footprint.pose.position[1]);
            if (!cell) {
                occupancyIndexValid = false;
                occupied[index] = footprint;
                return;
            }
            if (!occupancyIndexValid || *cell == occupancyCellByIndex[index]) {
                occupied[index] = footprint;
                return;
            }
            const auto oldCell = occupancyCells.find(occupancyCellByIndex[index]);
            if (oldCell == occupancyCells.end()) {
                throw std::logic_error(trafficInputError(
                    "Traffic occupancy index lost a footprint cell", "footprintIndex=", index,
                    "oldCellX=", occupancyCellByIndex[index].x,
                    "oldCellY=", occupancyCellByIndex[index].y, "newCellX=", cell->x,
                    "newCellY=", cell->y, "occupancyCellCount=", occupancyCells.size()));
            }
            auto& oldMembers = oldCell->second;
            const std::size_t slot = occupancySlotByIndex[index];
            const std::size_t lastIndex = oldMembers.back();
            oldMembers[slot] = lastIndex;
            occupancySlotByIndex[lastIndex] = slot;
            oldMembers.pop_back();
            if (oldMembers.empty()) {
                occupancyCells.erase(oldCell);
            }
            auto& newMembers = occupancyCells[*cell];
            occupancyCellByIndex[index] = *cell;
            occupancySlotByIndex[index] = newMembers.size();
            newMembers.push_back(index);
            occupied[index] = footprint;
        };
        // First overlapping footprint in the local windows, not an authored path endpoint.
        // Samples are in ascending distance order, so the first conflict found is the nearest.
        std::vector<const JunctionClaim::Section*> overlappingClaimSections;
        const auto collectOverlappingSections = [&](const JunctionClaim::Section& firstSection,
                                                    const JunctionClaim& second) {
            overlappingClaimSections.clear();
            for (const JunctionClaim::Section& secondSection : second.sections) {
                if (boundsOverlap(firstSection.bounds, secondSection.bounds)) {
                    overlappingClaimSections.push_back(&secondSection);
                }
            }
        };
        const auto claimsConflict = [&](const JunctionClaim& first,
                                        const JunctionClaim& second) -> std::optional<double> {
            if (!boundsOverlap(first.bounds, second.bounds)) {
                return std::nullopt;
            }
            for (const JunctionClaim::Section& firstSection : first.sections) {
                collectOverlappingSections(firstSection, second);
                const std::size_t firstEndSample =
                    firstSection.firstSample + firstSection.sampleCount;
                for (std::size_t mine = firstSection.firstSample; mine < firstEndSample; ++mine) {
                    for (const JunctionClaim::Section* secondSectionPtr :
                         overlappingClaimSections) {
                        const auto& secondSection = *secondSectionPtr;
                        const std::size_t secondEndSample =
                            secondSection.firstSample + secondSection.sampleCount;
                        for (std::size_t theirs = secondSection.firstSample;
                             theirs < secondEndSample; ++theirs) {
                            if (trafficFootprintsOverlap(first.footprints[mine],
                                                         second.footprints[theirs], 0.25) &&
                                !sameDirectedFlow(
                                    *first.actor, first.distances[mine], *first.segments[mine],
                                    first.footprints[mine].pose, *second.actor,
                                    second.distances[theirs], *second.segments[theirs],
                                    second.footprints[theirs].pose)) {
                                return first.distances[mine];
                            }
                        }
                    }
                }
            }
            return std::nullopt;
        };
        // Arrival estimates are cached on the claim; claims passed here are built and used
        // while their actors' states are unchanged.
        const auto claimArrival = [&](const JunctionClaim& claim, const MapTrafficActor& claimActor,
                                      std::size_t sample) -> const std::pair<double, double>& {
            if (claim.arrivalComputed.empty()) {
                claim.arrivalComputed.assign(claim.distances.size(), 0);
                claim.arrival.resize(claim.distances.size());
            }
            if (!claim.arrivalComputed[sample]) {
                const double distance = claim.distances[sample];
                const double travelDistance = std::max(0.0, distance - claimActor.distanceMeters);
                const double curveSpeed = mapTrafficCurveSpeedLimit(
                    claimActor.route, distance, claimActor.cruiseSpeedMetersPerSecond,
                    claimActor.minimumTurnRadiusMeters);
                const double targetSpeed =
                    std::min(claimActor.cruiseSpeedMetersPerSecond, curveSpeed);
                const double arrivalSeconds =
                    travelSeconds(travelDistance, claimActor.speedMetersPerSecond, targetSpeed);
                const double speedAtArrival =
                    std::max(2.0, std::min(targetSpeed, claimActor.speedMetersPerSecond +
                                                            acceleration * arrivalSeconds));
                claim.arrival[sample] = {arrivalSeconds,
                                         claimActor.lengthMeters * 0.5 / speedAtArrival};
                claim.arrivalComputed[sample] = 1;
            }
            return claim.arrival[sample];
        };
        const auto firstConflictOnArrival =
            [&](const JunctionClaim& first, const MapTrafficActor& firstActor,
                const JunctionClaim& second,
                const MapTrafficActor& secondActor) -> std::optional<double> {
            if (!boundsOverlap(first.bounds, second.bounds)) {
                return std::nullopt;
            }
            if (!deferrableSamplingQueries) {
                // Original evaluation order: every arrival query runs, so a malformed route
                // throws exactly where it did before the lazy/early-exit search below.
                std::optional<double> nearestConflict;
                for (const JunctionClaim::Section& firstSection : first.sections) {
                    for (const JunctionClaim::Section& secondSection : second.sections) {
                        if (!boundsOverlap(firstSection.bounds, secondSection.bounds)) {
                            continue;
                        }
                        const std::size_t firstEndSample =
                            firstSection.firstSample + firstSection.sampleCount;
                        const std::size_t secondEndSample =
                            secondSection.firstSample + secondSection.sampleCount;
                        for (std::size_t mine = firstSection.firstSample; mine < firstEndSample;
                             ++mine) {
                            const auto [firstArrivalSeconds, firstOccupancySeconds] =
                                claimArrival(first, firstActor, mine);
                            for (std::size_t theirs = secondSection.firstSample;
                                 theirs < secondEndSample; ++theirs) {
                                if (!trafficFootprintsOverlap(first.footprints[mine],
                                                              second.footprints[theirs], 0.25) ||
                                    sameDirectedFlow(
                                        firstActor, first.distances[mine], *first.segments[mine],
                                        first.footprints[mine].pose, secondActor,
                                        second.distances[theirs], *second.segments[theirs],
                                        second.footprints[theirs].pose)) {
                                    continue;
                                }
                                const auto [secondArrivalSeconds, secondOccupancySeconds] =
                                    claimArrival(second, secondActor, theirs);
                                if (std::abs(firstArrivalSeconds - secondArrivalSeconds) <=
                                    priorityGapSeconds + firstOccupancySeconds +
                                        secondOccupancySeconds) {
                                    nearestConflict =
                                        std::min(nearestConflict.value_or(
                                                     std::numeric_limits<double>::infinity()),
                                                 first.distances[mine]);
                                }
                            }
                        }
                    }
                }
                return nearestConflict;
            }
            // Samples are in ascending distance order, so the first conflict is the nearest.
            for (const JunctionClaim::Section& firstSection : first.sections) {
                collectOverlappingSections(firstSection, second);
                const std::size_t firstEndSample =
                    firstSection.firstSample + firstSection.sampleCount;
                for (std::size_t mine = firstSection.firstSample; mine < firstEndSample; ++mine) {
                    for (const JunctionClaim::Section* secondSectionPtr :
                         overlappingClaimSections) {
                        const auto& secondSection = *secondSectionPtr;
                        const std::size_t secondEndSample =
                            secondSection.firstSample + secondSection.sampleCount;
                        for (std::size_t theirs = secondSection.firstSample;
                             theirs < secondEndSample; ++theirs) {
                            if (!trafficFootprintsOverlap(first.footprints[mine],
                                                          second.footprints[theirs], 0.25)) {
                                continue;
                            }
                            if (sameDirectedFlow(firstActor, first.distances[mine],
                                                 *first.segments[mine], first.footprints[mine].pose,
                                                 secondActor, second.distances[theirs],
                                                 *second.segments[theirs],
                                                 second.footprints[theirs].pose)) {
                                continue;
                            }
                            const auto [firstArrivalSeconds, firstOccupancySeconds] =
                                claimArrival(first, firstActor, mine);
                            const auto [secondArrivalSeconds, secondOccupancySeconds] =
                                claimArrival(second, secondActor, theirs);
                            const double safeArrivalGap =
                                priorityGapSeconds + firstOccupancySeconds + secondOccupancySeconds;
                            if (std::abs(firstArrivalSeconds - secondArrivalSeconds) <=
                                safeArrivalGap) {
                                return first.distances[mine];
                            }
                        }
                    }
                }
            }
            return std::nullopt;
        };
        std::vector<double> priorities(actors.size());
        std::vector<double> junctionArrivals(actors.size(),
                                             std::numeric_limits<double>::infinity());
        std::vector<std::optional<std::pair<double, double>>> upcomingJunctions(actors.size());
        // A committed turn must not yield to traffic that has not entered the junction.
        std::vector<char> committedTurningInJunction(actors.size(), 0);
        // Track center occupancy separately so a front-edge commitment cannot steal priority
        // from higher-priority traffic already established on the junction path.
        std::vector<char> actorCenterInJunction(actors.size(), 0);
        std::vector<char> actorCommittedInJunction(actors.size(), 0);
        // Equal authored priority: straight-on movements go before bends, bends before turns,
        // and faster vehicles before slow or waiting ones that can more easily give way.
        // Discrete classes keep the ordering transitive, so yields can never form a cycle.
        std::vector<int> movementTurnClass(actors.size(), 0);
        std::vector<int> movementSpeedClass(actors.size(), 0);
        for (std::size_t candidateIndex = 0; candidateIndex < actors.size(); ++candidateIndex) {
            const MapTrafficActor& candidate = *actors[candidateIndex];
            priorities[candidateIndex] = approachPriority(candidate);
            const double prioritySpeedMetersPerSecond = deferrableSamplingQueries
                                                            ? candidate.cruiseSpeedMetersPerSecond
                                                            : candidate.speedMetersPerSecond;
            movementSpeedClass[candidateIndex] = prioritySpeedMetersPerSecond < 3.0   ? 0
                                                 : prioritySpeedMetersPerSecond < 8.0 ? 1
                                                                                      : 2;
            const auto range = nextJunctionRange(candidate);
            upcomingJunctions[candidateIndex] = range;
            if (range) {
                junctionArrivals[candidateIndex] = actorArrivalSeconds(candidate, range->first);
                const auto entry = sampleMapTrafficRoute(candidate.route, range->first, 0.0);
                const auto exit = sampleMapTrafficRoute(candidate.route, range->second, 0.0);
                if (entry && exit) {
                    const double turnRadians = std::abs(std::remainder(
                        exit->headingRadians - entry->headingRadians, 2.0 * std::numbers::pi));
                    constexpr double straightTurnRadians = 45.0 * std::numbers::pi / 180.0;
                    constexpr double bendTurnRadians = 100.0 * std::numbers::pi / 180.0;
                    movementTurnClass[candidateIndex] = turnRadians < straightTurnRadians ? 0
                                                        : turnRadians < bendTurnRadians   ? 1
                                                                                          : 2;
                }
            }
            actorCenterInJunction[candidateIndex] =
                mapTrafficRoutePositionOnJunctionPath(candidate.route, candidate.distanceMeters);
            actorCommittedInJunction[candidateIndex] =
                mapTrafficActorOnJunctionPath(candidate, candidate.distanceMeters);
            committedTurningInJunction[candidateIndex] =
                movementTurnClass[candidateIndex] > 0 && actorCommittedInJunction[candidateIndex];
        }
        // Compare local arrival windows even inside a long junction spline. A vehicle that
        // can still stop before the conflict yields; occupying a distant part of that spline
        // does not confer right of way at every later crossing.
        std::vector<char> canStillYield(actors.size(), 0);
        std::vector<double> priorityStopDistances(actors.size(),
                                                  std::numeric_limits<double>::infinity());
        std::vector<std::vector<std::size_t>> yieldsTo(actors.size());
        const double priorityYieldClearanceMeters = deferrableSamplingQueries ? 2.0 : 1.0;
        const double junctionYieldBufferMeters = deferrableSamplingQueries ? 1.5 : 0.0;
        std::vector<std::optional<JunctionClaim>> previewClaims(actors.size());
        const auto previewClaim = [&](std::size_t actorIndex) -> const JunctionClaim& {
            if (!previewClaims[actorIndex].has_value()) {
                const MapTrafficActor& candidate = *actors[actorIndex];
                previewClaims[actorIndex] = makeJunctionClaim(
                    candidate, candidate.distanceMeters,
                    candidate.distanceMeters + std::max(15.0, previewDistance(candidate)),
                    currentSignalClockSeconds);
            }
            return *previewClaims[actorIndex];
        };
        openbus::rendering::TraceScope priorityTrace(
            "traffic", "MapTrafficRuntime::advanceMapTrafficDriving.priorityPairArbitration");
        std::vector<char> priorityHeld(actors.size(), 0);
        std::vector<char> yieldingOnJunction(actors.size(), 0);
        for (std::size_t index = 0; index < actors.size(); ++index) {
            const MapTrafficActor& candidate = *actors[index];
            const bool junctionYielder =
                candidate.speedMetersPerSecond < 0.3 &&
                (candidate.blockReason == MapTrafficBlockReason::CrossingReservation ||
                 candidate.blockReason == MapTrafficBlockReason::JunctionTraffic);
            yieldingOnJunction[index] = actorCommittedInJunction[index] && junctionYielder;
            priorityHeld[index] = candidate.blockReason == MapTrafficBlockReason::Signal ||
                                  (candidate.speedMetersPerSecond < 0.3 &&
                                   (candidate.blockReason == MapTrafficBlockReason::Leader ||
                                    candidate.blockReason == MapTrafficBlockReason::Vehicle ||
                                    candidate.blockReason == MapTrafficBlockReason::Obstacle ||
                                    candidate.blockReason == MapTrafficBlockReason::Collision)) ||
                                  (!actorCommittedInJunction[index] && junctionYielder);
        }
        // The front-only turn check runs before the normal 250 m distance rejection.
        // Only move the rejection earlier when no pair can pass that check at 250 m.
        const bool canRejectDistantPriorityPairs =
            occupancyIndexValid && largestFootprintRadiusMeters * 2.0 + 4.0 < 249.0;
        // Same-route pairs are excluded from priority arbitration below.
        const bool singleRoutePriorityQueue =
            deferrableSamplingQueries && !actors.empty() &&
            actorsByRouteClass[routeClassByActor.front()].size() == actors.size();
        constexpr double priorityCellSizeMeters = 256.0;
        std::unordered_map<OccupancyCell, std::vector<std::size_t>, OccupancyCellHash>
            priorityCells;
        std::vector<OccupancyCell> priorityCellByActor(actors.size());
        bool priorityIndexValid = canRejectDistantPriorityPairs && !singleRoutePriorityQueue;
        if (priorityIndexValid) {
            priorityCells.reserve(actors.size());
            for (std::size_t index = 0; index < actors.size(); ++index) {
                const auto& position = occupied[obstacles.size() + index].pose.position;
                const auto cell = cellForPosition(position[0], position[1], priorityCellSizeMeters);
                if (!cell) {
                    priorityIndexValid = false;
                    break;
                }
                priorityCellByActor[index] = *cell;
                priorityCells[*cell].push_back(index);
            }
        }
        priorityIndexValid = priorityIndexValid && priorityCells.size() > 1;
        std::vector<std::size_t> priorityCandidates;
        for (std::size_t lowIndex = 0; !singleRoutePriorityQueue && lowIndex < actors.size();
             ++lowIndex) {
            const MapTrafficActor& low = *actors[lowIndex];
            const auto& lowPosition = occupied[obstacles.size() + lowIndex].pose.position;
            priorityCandidates.clear();
            if (priorityIndexValid) {
                const auto cell = priorityCellByActor[lowIndex];
                for (std::int64_t x = cell.x - 1; x <= cell.x + 1; ++x) {
                    for (std::int64_t y = cell.y - 1; y <= cell.y + 1; ++y) {
                        const auto found = priorityCells.find({x, y});
                        if (found != priorityCells.end()) {
                            if (deferrableSamplingQueries) {
                                for (const std::size_t highIndex : found->second) {
                                    if (highIndex != lowIndex && !priorityHeld[highIndex] &&
                                        routeClassByActor[lowIndex] !=
                                            routeClassByActor[highIndex]) {
                                        priorityCandidates.push_back(highIndex);
                                    }
                                }
                            } else {
                                priorityCandidates.insert(priorityCandidates.end(),
                                                          found->second.begin(),
                                                          found->second.end());
                            }
                        }
                    }
                }
                std::sort(priorityCandidates.begin(), priorityCandidates.end());
            }
            const std::size_t candidateCount =
                priorityIndexValid ? priorityCandidates.size() : actors.size();
            for (std::size_t candidateIndex = 0; candidateIndex < candidateCount;
                 ++candidateIndex) {
                const std::size_t highIndex =
                    priorityIndexValid ? priorityCandidates[candidateIndex] : candidateIndex;
                if (deferrableSamplingQueries &&
                    (highIndex == lowIndex || priorityHeld[highIndex] ||
                     routeClassByActor[lowIndex] == routeClassByActor[highIndex])) {
                    continue;
                }
                const auto& highPosition = occupied[obstacles.size() + highIndex].pose.position;
                if (canRejectDistantPriorityPairs &&
                    std::hypot(highPosition[0] - lowPosition[0], highPosition[1] - lowPosition[1]) >
                        250.0) {
                    continue;
                }
                const MapTrafficActor& high = *actors[highIndex];
                const bool tieBreaksInFavorOfHigh =
                    priorities[highIndex] == priorities[lowIndex] &&
                    (movementTurnClass[highIndex] < movementTurnClass[lowIndex] ||
                     (movementTurnClass[highIndex] == movementTurnClass[lowIndex] &&
                      (movementSpeedClass[highIndex] > movementSpeedClass[lowIndex] ||
                       (movementSpeedClass[highIndex] == movementSpeedClass[lowIndex] &&
                        (junctionArrivals[highIndex] < junctionArrivals[lowIndex] ||
                         (junctionArrivals[highIndex] == junctionArrivals[lowIndex] &&
                          highIndex < lowIndex))))));
                const bool highHasAuthoredPriority =
                    priorities[highIndex] > priorities[lowIndex] || tieBreaksInFavorOfHigh;
                // A long vehicle's nose can enter the junction while it is still alongside
                // through traffic on the approach. Treat that as a yieldable turn only if
                // its sampled footprint can stop before the conflict.
                const auto adjacentFrontOnlyTurn = [&](std::size_t turnIndex,
                                                       std::size_t throughIndex) {
                    if (!committedTurningInJunction[turnIndex] ||
                        actorCenterInJunction[turnIndex] || movementTurnClass[throughIndex] != 0) {
                        return false;
                    }
                    const auto& turn = occupied[obstacles.size() + turnIndex];
                    const auto& through = occupied[obstacles.size() + throughIndex];
                    const double heading = through.pose.headingRadians;
                    if (std::cos(turn.pose.headingRadians - heading) < 0.95) {
                        return false;
                    }
                    const double dx = turn.pose.position[0] - through.pose.position[0];
                    const double dy = turn.pose.position[1] - through.pose.position[1];
                    const double along = std::abs(dx * std::sin(heading) + dy * std::cos(heading));
                    const double across = std::abs(dx * std::cos(heading) - dy * std::sin(heading));
                    if (along > (turn.lengthMeters + through.lengthMeters) * 0.5 ||
                        across > (turn.widthMeters + through.widthMeters) * 0.5 + 4.0) {
                        return false;
                    }
                    const MapTrafficActor& turningActor = *actors[turnIndex];
                    const auto conflict =
                        firstConflictOnArrival(previewClaim(turnIndex), turningActor,
                                               previewClaim(throughIndex), *actors[throughIndex]);
                    return conflict.has_value() && *conflict - turningActor.distanceMeters >=
                                                       turningActor.speedMetersPerSecond *
                                                               turningActor.speedMetersPerSecond /
                                                               (2.0 * braking) +
                                                           priorityYieldClearanceMeters;
                };
                const bool lowIsAdjacentFrontOnlyTurn = adjacentFrontOnlyTurn(lowIndex, highIndex);
                const bool highIsAdjacentFrontOnlyTurn = adjacentFrontOnlyTurn(highIndex, lowIndex);
                const bool committedTurnPrecedenceDiffers =
                    committedTurningInJunction[highIndex] != committedTurningInJunction[lowIndex];
                const bool highPrecedesJunctionYielder =
                    actorCommittedInJunction[highIndex] && yieldingOnJunction[lowIndex];
                const bool highIsEstablishedInJunction =
                    actorCenterInJunction[highIndex] && !actorCenterInJunction[lowIndex] &&
                    movementTurnClass[lowIndex] > 0 && highHasAuthoredPriority;
                const bool highHasPriority =
                    lowIsAdjacentFrontOnlyTurn || highIsAdjacentFrontOnlyTurn
                        ? highHasAuthoredPriority
                    : highPrecedesJunctionYielder
                        ? highHasAuthoredPriority
                        : highIsEstablishedInJunction ||
                              (committedTurnPrecedenceDiffers
                                   ? committedTurningInJunction[highIndex] != 0
                                   : highHasAuthoredPriority);
                if (highIndex == lowIndex || !highHasPriority || priorityHeld[highIndex]) {
                    continue;
                }
                if (!canRejectDistantPriorityPairs &&
                    std::hypot(highPosition[0] - lowPosition[0], highPosition[1] - lowPosition[1]) >
                        250.0) {
                    continue;
                }
                // Same as sameDirectedRoute(low, high) without the pointer lookups.
                if (routeClassByActor[lowIndex] == routeClassByActor[highIndex] ||
                    queuedBehindInLane(low, occupied[obstacles.size() + lowIndex], high,
                                       occupied[obstacles.size() + highIndex]) ||
                    queuedBehindInLane(high, occupied[obstacles.size() + highIndex], low,
                                       occupied[obstacles.size() + lowIndex])) {
                    continue;
                }
                // Opposing turns that share a non-passing connector need a single winner,
                // even when their local arrival windows initially appear time-separated.
                if (!mapTrafficActorOnJunctionPath(low, low.distanceMeters) &&
                    opposingJunctionMovements(low, high)) {
                    const auto lowJunction = deferrableSamplingQueries ? upcomingJunctions[lowIndex]
                                                                       : nextJunctionRange(low);
                    if (lowJunction.has_value()) {
                        const double stopDistance = lowJunction->first - low.lengthMeters * 0.5 -
                                                    0.5 - junctionYieldBufferMeters;
                        if (stopDistance - low.distanceMeters >=
                            low.speedMetersPerSecond * low.speedMetersPerSecond / (2.0 * braking)) {
                            canStillYield[lowIndex] = 1;
                            priorityStopDistances[lowIndex] =
                                std::min(priorityStopDistances[lowIndex], stopDistance);
                            yieldsTo[lowIndex].push_back(highIndex);
                            continue;
                        }
                    }
                }
                const auto conflict = [&] {
                    // Build in a fixed order so a failing claim throws deterministically.
                    const JunctionClaim& lowPreview = previewClaim(lowIndex);
                    const JunctionClaim& highPreview = previewClaim(highIndex);
                    return firstConflictOnArrival(lowPreview, low, highPreview, high);
                }();
                // Once a turn has entered, do not assign it a priority stop inside the
                // junction; it must clear while physical collision checks remain active.
                if (conflict.has_value() && !committedTurningInJunction[lowIndex] &&
                    highIsEstablishedInJunction && movementTurnClass[lowIndex] > 0) {
                    const auto lowJunction = deferrableSamplingQueries ? upcomingJunctions[lowIndex]
                                                                       : nextJunctionRange(low);
                    if (lowJunction.has_value()) {
                        const double stopDistance = lowJunction->first - low.lengthMeters * 0.5 -
                                                    priorityYieldClearanceMeters -
                                                    junctionYieldBufferMeters;
                        if (stopDistance - low.distanceMeters >=
                            low.speedMetersPerSecond * low.speedMetersPerSecond / (2.0 * braking)) {
                            canStillYield[lowIndex] = 1;
                            priorityStopDistances[lowIndex] =
                                std::min(priorityStopDistances[lowIndex], stopDistance);
                            yieldsTo[lowIndex].push_back(highIndex);
                            continue;
                        }
                    }
                }
                const double conflictStopBuffer =
                    lowIsAdjacentFrontOnlyTurn
                        ? priorityYieldClearanceMeters
                        : low.lengthMeters * 0.5 + priorityYieldClearanceMeters;
                if (conflict.has_value() &&
                    (!committedTurningInJunction[lowIndex] || highPrecedesJunctionYielder ||
                     lowIsAdjacentFrontOnlyTurn) &&
                    *conflict - low.distanceMeters >=
                        low.speedMetersPerSecond * low.speedMetersPerSecond / (2.0 * braking) +
                            conflictStopBuffer) {
                    canStillYield[lowIndex] = 1;
                    priorityStopDistances[lowIndex] =
                        std::min(priorityStopDistances[lowIndex], *conflict - conflictStopBuffer);
                    yieldsTo[lowIndex].push_back(highIndex);
                }
            }
        }
        priorityTrace.finish();
        const auto yieldsForPriority = [&](std::size_t lowIndex, std::size_t highIndex) {
            return canStillYield[lowIndex] &&
                   std::ranges::find(yieldsTo[lowIndex], highIndex) != yieldsTo[lowIndex].end();
        };
        std::vector<std::size_t> priorityOrder(actors.size());
        std::iota(priorityOrder.begin(), priorityOrder.end(), 0);
        std::stable_sort(
            priorityOrder.begin(), priorityOrder.end(), [&](std::size_t first, std::size_t second) {
                if (actorCenterInJunction[first] != actorCenterInJunction[second]) {
                    return actorCenterInJunction[first] != 0;
                }
                if (actorCommittedInJunction[first] != actorCommittedInJunction[second]) {
                    return actorCommittedInJunction[first] != 0;
                }
                if (priorities[first] != priorities[second]) {
                    return priorities[first] > priorities[second];
                }
                if (deferrableSamplingQueries) {
                    if (movementTurnClass[first] != movementTurnClass[second]) {
                        return movementTurnClass[first] < movementTurnClass[second];
                    }
                    if (movementSpeedClass[first] != movementSpeedClass[second]) {
                        return movementSpeedClass[first] > movementSpeedClass[second];
                    }
                    if (junctionArrivals[first] != junctionArrivals[second]) {
                        return junctionArrivals[first] < junctionArrivals[second];
                    }
                }
                // Waiting time breaks ties only after established occupancy and authored priority.
                return actors[first]->junctionWaitSeconds > actors[second]->junctionWaitSeconds;
            });
        openbus::rendering::TraceScope movementTrace(
            "traffic", "MapTrafficRuntime::advanceMapTrafficDriving.actorMovementPass");
        // Reused across actors to avoid per-actor allocations.
        std::vector<std::size_t> nearbyOccupied;
        std::vector<std::size_t> nearbyReservations;
        const auto onJunctionPath = [](const MapTrafficActor& candidate, double distance) {
            return mapTrafficActorOnJunctionPath(candidate, distance);
        };
        // Other actors can move during this pass; invalidate their cached state on movement.
        std::vector<std::optional<bool>> currentJunctionOccupancy(actors.size());
        if (deferrableSamplingQueries) {
            for (std::size_t actorIndex = 0; actorIndex < actors.size(); ++actorIndex) {
                currentJunctionOccupancy[actorIndex] = actorCommittedInJunction[actorIndex] != 0;
            }
        }
        const auto actorCurrentlyOnJunction = [&](std::size_t actorIndex) {
            const MapTrafficActor& candidate = *actors[actorIndex];
            if (!deferrableSamplingQueries) {
                return onJunctionPath(candidate, candidate.distanceMeters);
            }
            auto& cached = currentJunctionOccupancy[actorIndex];
            if (!cached.has_value()) {
                cached = onJunctionPath(candidate, candidate.distanceMeters);
            }
            return *cached;
        };
        for (const std::size_t index : priorityOrder) {
            openbus::rendering::TraceScope actorTrace(
                "traffic", "MapTrafficRuntime::advanceMapTrafficDriving.actorMovement");
            auto& actor = *actors[index];
            const std::size_t own = obstacles.size() + index;
            const double physicalFootprintDiameter = occupiedDiameters[own];
            std::optional<double> lookaheadFootprintDiameter;
            bool blockedByCrossingReservation = false;
            bool blockedByJunctionTraffic = false;
            const bool committedToJunction = deferrableSamplingQueries
                                                 ? actorCommittedInJunction[index] != 0
                                                 : onJunctionPath(actor, actor.distanceMeters);
            bool junctionInLookahead = committedToJunction;
            std::optional<std::pair<double, double>> upcomingJunctionRange;
            if (!committedToJunction) {
                upcomingJunctionRange =
                    deferrableSamplingQueries ? upcomingJunctions[index] : nextJunctionRange(actor);
            }
            const MapTrafficActor* priorityTraffic =
                canStillYield[index] && !yieldsTo[index].empty() ? actors[yieldsTo[index].front()]
                                                                 : nullptr;
            MapTrafficBlockReason clearBlockReason = MapTrafficBlockReason::None;
            // In-lane leaders, moving or queued. The following model already controls the gap
            // to them, so the lookahead must not add its larger obstacle clearance on top.
            std::vector<const MapTrafficActor*> laneLeaders;
            std::vector<bool> followingObstacleLookahead(obstacles.size(), false);
            const MapTrafficActor* clearBlockingActor = nullptr;
            std::optional<std::size_t> clearBlockingObstacle;
            const auto clearAt = [&](double distance, double clearance,
                                     bool allowFollowingVehicleLookahead,
                                     std::optional<bool> knownJunctionAtDistance = std::nullopt) {
                const auto* cachedFootprint =
                    distance == actor.distanceMeters
                        ? &occupied[own]
                        : cachedClaimFootprint(
                              previewClaims[index] ? &*previewClaims[index] : nullptr, distance);
                auto footprint =
                    cachedFootprint != nullptr ? *cachedFootprint : actorFootprint(actor, distance);
                const MapTrafficObstacle physicalFootprint = footprint;
                double footprintDiameter = physicalFootprintDiameter;
                const auto actorOnJunctionAtDistance = [&] {
                    if (!deferrableSamplingQueries) {
                        return onJunctionPath(actor, distance);
                    }
                    if (!knownJunctionAtDistance.has_value()) {
                        knownJunctionAtDistance = onJunctionPath(actor, distance);
                    }
                    return *knownJunctionAtDistance;
                };
                const bool committedJunctionMovement =
                    committedTurningInJunction[index] && actorOnJunctionAtDistance();
                if (clearance > 0.5 && !committedJunctionMovement) {
                    // Headway is ahead of the nose, not behind the tail of the leader.
                    const double extension = clearance - 0.25;
                    footprint.lengthMeters += extension;
                    footprint.pose.position[0] +=
                        std::sin(footprint.pose.headingRadians) * extension * 0.5;
                    footprint.pose.position[1] +=
                        std::cos(footprint.pose.headingRadians) * extension * 0.5;
                    if (clearance == 2.0) {
                        if (!lookaheadFootprintDiameter.has_value()) {
                            lookaheadFootprintDiameter =
                                std::hypot(footprint.lengthMeters, footprint.widthMeters);
                        }
                        footprintDiameter = *lookaheadFootprintDiameter;
                    } else {
                        footprintDiameter =
                            std::hypot(footprint.lengthMeters, footprint.widthMeters);
                    }
                    clearance = 0.25;
                }
                const auto blocksAt = [&](std::size_t other) {
                    if (other == own) {
                        return false;
                    }
                    const bool otherIsActor = other >= obstacles.size();
                    const MapTrafficActor* otherActor =
                        otherIsActor ? actors[other - obstacles.size()] : nullptr;
                    const bool bothOnJunction = otherActor != nullptr && clearance <= 0.5 &&
                                                actorOnJunctionAtDistance() &&
                                                actorCurrentlyOnJunction(other - obstacles.size());
                    const bool otherYieldsToCommittedActor =
                        otherActor != nullptr && committedJunctionMovement &&
                        !actorCurrentlyOnJunction(other - obstacles.size()) &&
                        yieldsForPriority(other - obstacles.size(), index);
                    const double pairClearance =
                        bothOnJunction || otherYieldsToCommittedActor ? 0.1 : clearance;
                    const MapTrafficObstacle& candidateFootprint =
                        otherActor == nullptr ? physicalFootprint : footprint;
                    const double candidateClearance =
                        otherActor == nullptr
                            ? (committedJunctionMovement ? turningObstacleClearanceMeters : 0.5)
                            : pairClearance;
                    if (!footprintsOverlapWithDiameters(
                            candidateFootprint, occupied[other], candidateClearance,
                            otherActor == nullptr ? physicalFootprintDiameter : footprintDiameter,
                            occupiedDiameters[other])) {
                        return false;
                    }
                    if (allowFollowingVehicleLookahead && other < obstacles.size() &&
                        followingObstacleLookahead[other]) {
                        return false;
                    }
                    if (other < obstacles.size() &&
                        footprintSeparation(physicalFootprint, occupied[other], 0.5) + 1.0e-6 >=
                            footprintSeparation(occupied[own], occupied[other], 0.5)) {
                        return false;
                    }
                    if (otherActor != nullptr &&
                        queuedBehindInLane(actor, occupied[own], *otherActor, occupied[other])) {
                        const double currentSeparation =
                            footprintSeparation(occupied[own], occupied[other], 0.0);
                        const double proposedSeparation =
                            footprintSeparation(physicalFootprint, occupied[other], 0.0);
                        if (distance == actor.distanceMeters
                                ? proposedSeparation + 1.0e-9 >= currentSeparation
                                : proposedSeparation > currentSeparation + 1.0e-9) {
                            return false;
                        }
                    }
                    clearBlockReason = MapTrafficBlockReason::Obstacle;
                    clearBlockingActor = nullptr;
                    clearBlockingObstacle.reset();
                    if (other < obstacles.size()) {
                        clearBlockingObstacle = other;
                    }
                    if (other >= obstacles.size()) {
                        // Sustained junction blocks may move only when separation improves.
                        const bool collisionDeadlockRecovery =
                            actor.junctionWaitSeconds >= 1.0 &&
                            (actorCurrentlyOnJunction(index) ||
                             actorCurrentlyOnJunction(other - obstacles.size()));
                        if (collisionDeadlockRecovery &&
                            footprintSeparation(physicalFootprint, occupied[other], pairClearance) +
                                    1.0e-6 >=
                                footprintSeparation(occupied[own], occupied[other],
                                                    pairClearance)) {
                            return false;
                        }
                        if (allowFollowingVehicleLookahead &&
                            ((routeClassByActor[index] ==
                                  routeClassByActor[other - obstacles.size()] &&
                              isBehindOnRoute(actor, *otherActor)) ||
                             std::ranges::find(laneLeaders, otherActor) != laneLeaders.end())) {
                            return false;
                        }
                        clearBlockingActor = otherActor;
                        const bool crossing =
                            std::abs(std::cos(footprint.pose.headingRadians -
                                              occupied[other].pose.headingRadians)) < 0.7;
                        const bool otherOnJunction =
                            actorCurrentlyOnJunction(other - obstacles.size());
                        if ((actorOnJunctionAtDistance() && crossing) || otherOnJunction) {
                            blockedByJunctionTraffic = true;
                            clearBlockReason = MapTrafficBlockReason::JunctionTraffic;
                        } else {
                            clearBlockReason = MapTrafficBlockReason::Vehicle;
                        }
                    }
                    return true;
                };
                collectNearbyIndexed(footprint, footprintDiameter, clearance, occupancyCells,
                                     largestFootprintRadiusMeters, occupancyIndexValid,
                                     occupied.size(), nearbyOccupied);
                for (const std::size_t other : nearbyOccupied) {
                    if (blocksAt(other)) {
                        return false;
                    }
                }
                // Crossing reservations on the same route are ignored by the narrow phase.
                collectNearbyIndexed(footprint, footprintDiameter, clearance, reservationCells,
                                     largestReservationRadiusMeters, reservationIndexValid,
                                     reservations.size(), nearbyReservations, false,
                                     deferrableSamplingQueries
                                         ? std::optional<std::size_t>(routeClassByActor[index])
                                         : std::nullopt);
                for (const std::size_t reservationIndex : nearbyReservations) {
                    auto& reservation = reservations[reservationIndex];
                    // Most reserved samples are far away; reject them before route comparisons.
                    if (!footprintsOverlapWithDiameters(
                            footprint, reservation.footprint, clearance, footprintDiameter,
                            occupiedDiameters[obstacles.size() + reservation.actorIndex])) {
                        continue;
                    }
                    if (yieldsForPriority(reservation.actorIndex, index)) {
                        continue;
                    }
                    const MapTrafficRouteSegment& candidateSegment =
                        routeSegmentAtDistance(actor.route, distance);
                    const MapTrafficRouteSegment& reservedSegment = routeSegmentAtDistance(
                        reservation.actor->route, reservation.distanceMeters);
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
                    if (routeClassByActor[index] == routeClassByActor[reservation.actorIndex]) {
                        continue;
                    }
                    const bool sameFlow = sameDirectedFlow(
                        actor, distance, candidateSegment, footprint.pose, *reservation.actor,
                        reservation.distanceMeters, reservedSegment, reservation.footprint.pose);
                    // Following is handled separately; reserve crossing movements only.
                    if (!sameFlow &&
                        std::abs(std::cos(footprint.pose.headingRadians -
                                          reservation.footprint.pose.headingRadians)) < 0.7) {
                        if (!reservation.arrivalSeconds.has_value()) {
                            reservation.arrivalSeconds =
                                step +
                                actorArrivalSeconds(*reservation.actor, reservation.distanceMeters);
                        }
                        const double reservationArrivalSeconds = *reservation.arrivalSeconds;
                        const double candidateArrivalSeconds = actorArrivalSeconds(actor, distance);
                        const double candidateTargetSpeed =
                            std::min(actor.cruiseSpeedMetersPerSecond,
                                     mapTrafficCurveSpeedLimit(actor.route, distance,
                                                               actor.cruiseSpeedMetersPerSecond,
                                                               actor.minimumTurnRadiusMeters));
                        const double candidateArrivalSpeed =
                            std::max(2.0, std::min(candidateTargetSpeed,
                                                   actor.speedMetersPerSecond +
                                                       acceleration * candidateArrivalSeconds));
                        const double reservationArrivalSpeed =
                            std::max(2.0, std::min(reservation.actor->cruiseSpeedMetersPerSecond,
                                                   reservation.actor->speedMetersPerSecond +
                                                       acceleration * reservationArrivalSeconds));
                        const double safeArrivalGap =
                            0.25 + actor.lengthMeters * 0.5 / candidateArrivalSpeed +
                            reservation.actor->lengthMeters * 0.5 / reservationArrivalSpeed;
                        if (std::abs(candidateArrivalSeconds - reservationArrivalSeconds) <=
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
            const auto limitTargetFromSquaredSpeed = [&](double limitSquared) {
                if (limitSquared < target * target) {
                    limitTarget(std::sqrt(std::max(0.0, limitSquared)),
                                MapTrafficBlockReason::None);
                }
            };
            double nearestFollowingGapMeters = std::numeric_limits<double>::infinity();
            double nearestFollowingSpeedMetersPerSecond = 0.0;
            const MapTrafficActor* nearestFollowingActor = nullptr;
            const MapTrafficObstacle& actorFootprintNow = occupied[own];
            const double actorHeading = actorFootprintNow.pose.headingRadians;
            const double actorHeadingSin = std::sin(actorHeading);
            const double actorHeadingCos = std::cos(actorHeading);
            const bool openRoute = !actor.route.closed && !actor.route.loopStartDistance;
            // Beyond this gap even a stopped leader cannot lower the following target.
            const double followingRangeMeters =
                standstillFollowingGapMeters + actor.speedMetersPerSecond * headwaySeconds +
                actor.speedMetersPerSecond * actor.speedMetersPerSecond / (2.0 * braking) +
                actor.cruiseSpeedMetersPerSecond * followingResponseSeconds + 5.0;
            openbus::rendering::TraceScope followingTrace(
                "traffic", "MapTrafficRuntime::advanceMapTrafficDriving.actorFollowingSearch");
            openbus::rendering::TraceScope candidateTrace(
                "traffic",
                "MapTrafficRuntime::advanceMapTrafficDriving.actorFollowingCandidateCollection");
            collectNearbyIndexed(actorFootprintNow, physicalFootprintDiameter, followingRangeMeters,
                                 occupancyCells, largestFootprintRadiusMeters, occupancyIndexValid,
                                 occupied.size(), nearbyOccupied, true);
            // Same-route leaders are considered regardless of spatial distance, including
            // across loop seams and on routes that double back.
            const auto& routeActors = actorsByRouteClass[routeClassByActor[index]];
            if (routeActors.size() > 1) {
                const auto& sortedRouteActors = routeActorsByDistance[routeClassByActor[index]];
                if (sortedRouteActors.empty()) {
                    for (const std::size_t routeActor : routeActors) {
                        nearbyOccupied.push_back(obstacles.size() + routeActor);
                    }
                } else {
                    const double maximumFollowingGapMeters =
                        standstillFollowingGapMeters + actor.speedMetersPerSecond * headwaySeconds +
                        actor.speedMetersPerSecond * actor.speedMetersPerSecond / (2.0 * braking) +
                        actor.cruiseSpeedMetersPerSecond * (followingResponseSeconds + 1.0) + 1.0;
                    const double substepMovementMarginMeters = 2.0 * maximumActorTravelMeters;
                    const double minimumDistance =
                        actor.distanceMeters - substepMovementMarginMeters;
                    const double maximumDistance = actor.distanceMeters +
                                                   maximumFollowingGapMeters +
                                                   substepMovementMarginMeters;
                    const auto first = std::lower_bound(
                        sortedRouteActors.begin(), sortedRouteActors.end(), minimumDistance,
                        [&](std::size_t routeActor, double distance) {
                            return actors[routeActor]->distanceMeters < distance;
                        });
                    const auto last =
                        std::upper_bound(first, sortedRouteActors.end(), maximumDistance,
                                         [&](double distance, std::size_t routeActor) {
                                             return distance < actors[routeActor]->distanceMeters;
                                         });
                    for (auto routeActor = first; routeActor != last; ++routeActor) {
                        nearbyOccupied.push_back(obstacles.size() + *routeActor);
                    }
                }
                std::sort(nearbyOccupied.begin(), nearbyOccupied.end());
                nearbyOccupied.erase(std::unique(nearbyOccupied.begin(), nearbyOccupied.end()),
                                     nearbyOccupied.end());
            }
            candidateTrace.finish();
            openbus::rendering::TraceScope evaluationTrace(
                "traffic",
                "MapTrafficRuntime::advanceMapTrafficDriving.actorFollowingCandidateEvaluation");
            for (const std::size_t occupiedIndex : nearbyOccupied) {
                if (occupiedIndex < obstacles.size()) {
                    continue;
                }
                const std::size_t otherIndex = occupiedIndex - obstacles.size();
                if (otherIndex == index) {
                    continue;
                }
                const auto& otherActor = *actors[otherIndex];
                const MapTrafficObstacle& otherFootprint = occupied[obstacles.size() + otherIndex];
                const double halfLengths = (actor.lengthMeters + otherActor.lengthMeters) * 0.5;
                double alongRouteMeters = 0.0;
                if (routeClassByActor[index] == routeClassByActor[otherIndex]) {
                    if (!isBehindOnRoute(otherActor, actor)) {
                        continue;
                    }
                    alongRouteMeters = otherActor.distanceMeters - actor.distanceMeters;
                    if (alongRouteMeters < 0.0) {
                        const double cycleStart = actor.route.loopStartDistance.value_or(0.0);
                        alongRouteMeters += actor.route.totalLength - cycleStart;
                    }
                } else {
                    // A leader must sit on this actor's own path, not merely straight
                    // ahead: project it onto the route so curves, adjacent lanes, side
                    // roads, and over/underpasses are not mistaken for the vehicle in
                    // front.
                    const double deltaX =
                        otherFootprint.pose.position[0] - actorFootprintNow.pose.position[0];
                    const double deltaY =
                        otherFootprint.pose.position[1] - actorFootprintNow.pose.position[1];
                    alongRouteMeters = deltaX * actorHeadingSin + deltaY * actorHeadingCos;
                    if (alongRouteMeters <= 0.0 ||
                        std::hypot(deltaX, deltaY) - halfLengths > followingRangeMeters ||
                        std::cos(actorHeading - otherFootprint.pose.headingRadians) < 0.0) {
                        continue;
                    }
                    std::optional<MapTrafficPose> pathPose;
                    double pathHeadingSin = 0.0;
                    double pathHeadingCos = 0.0;
                    for (int refinement = 0; refinement < 3 && alongRouteMeters > 0.0;
                         ++refinement) {
                        const double pathDistance = actor.distanceMeters + alongRouteMeters;
                        if (openRoute && pathDistance > actor.route.totalLength) {
                            pathPose.reset();
                            break;
                        }
                        pathPose = sampleMapTrafficActorPose(actor, pathDistance);
                        if (!pathPose) {
                            break;
                        }
                        pathHeadingSin = std::sin(pathPose->headingRadians);
                        pathHeadingCos = std::cos(pathPose->headingRadians);
                        alongRouteMeters +=
                            (otherFootprint.pose.position[0] - pathPose->position[0]) *
                                pathHeadingSin +
                            (otherFootprint.pose.position[1] - pathPose->position[1]) *
                                pathHeadingCos;
                    }
                    if (!pathPose || alongRouteMeters <= 0.0 ||
                        std::cos(pathPose->headingRadians - otherFootprint.pose.headingRadians) <
                            0.7 ||
                        std::abs(otherFootprint.pose.position[2] - pathPose->position[2]) > 3.0) {
                        continue;
                    }
                    const double pathLateralOffset = std::abs(
                        (otherFootprint.pose.position[0] - pathPose->position[0]) * pathHeadingCos -
                        (otherFootprint.pose.position[1] - pathPose->position[1]) * pathHeadingSin);
                    // Bodies must overlap laterally by a margin to share this lane.
                    if (pathLateralOffset >
                        (actor.widthMeters + otherActor.widthMeters) * 0.5 - 0.3) {
                        continue;
                    }
                }
                const double longitudinalGap = alongRouteMeters - halfLengths;
                if (longitudinalGap >= 0.0) {
                    laneLeaders.push_back(&otherActor);
                }
                if (longitudinalGap < 0.0 || longitudinalGap >= nearestFollowingGapMeters) {
                    continue;
                }
                nearestFollowingGapMeters = longitudinalGap;
                nearestFollowingSpeedMetersPerSecond = otherActor.speedMetersPerSecond;
                nearestFollowingActor = &otherActor;
            }
            evaluationTrace.finish();
            followingTrace.finish();
            openbus::rendering::TraceScope obstacleTrace(
                "traffic", "MapTrafficRuntime::advanceMapTrafficDriving.actorObstacleProjection");
            for (std::size_t obstacleIndex = 0; obstacleIndex < obstacles.size(); ++obstacleIndex) {
                const MapTrafficObstacle& obstacle = obstacles[obstacleIndex];
                const double halfLengths = (actor.lengthMeters + obstacle.lengthMeters) * 0.5;
                const double deltaX =
                    obstacle.pose.position[0] - actorFootprintNow.pose.position[0];
                const double deltaY =
                    obstacle.pose.position[1] - actorFootprintNow.pose.position[1];
                double alongRouteMeters = deltaX * actorHeadingSin + deltaY * actorHeadingCos;
                if (alongRouteMeters <= 0.0 ||
                    std::hypot(deltaX, deltaY) - halfLengths > followingRangeMeters ||
                    std::cos(actorHeading - obstacle.pose.headingRadians) < 0.0) {
                    continue;
                }
                std::optional<MapTrafficPose> pathPose;
                for (int refinement = 0; refinement < 3 && alongRouteMeters > 0.0; ++refinement) {
                    const double pathDistance = actor.distanceMeters + alongRouteMeters;
                    if (openRoute && pathDistance > actor.route.totalLength) {
                        pathPose.reset();
                        break;
                    }
                    pathPose = sampleMapTrafficActorPose(actor, pathDistance);
                    if (!pathPose) {
                        break;
                    }
                    alongRouteMeters += (obstacle.pose.position[0] - pathPose->position[0]) *
                                            std::sin(pathPose->headingRadians) +
                                        (obstacle.pose.position[1] - pathPose->position[1]) *
                                            std::cos(pathPose->headingRadians);
                }
                if (!pathPose || alongRouteMeters <= 0.0 ||
                    std::cos(pathPose->headingRadians - obstacle.pose.headingRadians) < 0.7 ||
                    std::abs(obstacle.pose.position[2] - pathPose->position[2]) > 3.0) {
                    continue;
                }
                const double pathLateralOffset =
                    std::abs((obstacle.pose.position[0] - pathPose->position[0]) *
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
            obstacleTrace.finish();
            if (std::isfinite(nearestFollowingGapMeters)) {
                const double closingDistance =
                    std::max(0.0, (actor.speedMetersPerSecond * actor.speedMetersPerSecond -
                                   nearestFollowingSpeedMetersPerSecond *
                                       nearestFollowingSpeedMetersPerSecond) /
                                      (2.0 * braking));
                const double desiredFollowingGap =
                    standstillFollowingGapMeters +
                    actor.speedMetersPerSecond *
                        followingHeadwaySeconds(actor.speedMetersPerSecond) +
                    closingDistance;
                const double excessGap = nearestFollowingGapMeters - desiredFollowingGap;
                // Behind a stopped or crawling leader, close the last of the gap at a gentle
                // walking pace and settle within a small band, instead of creeping forever.
                constexpr double settleBandMeters = 0.2;
                const bool leaderNearlyStopped = nearestFollowingSpeedMetersPerSecond < 0.5;
                const double responseSeconds =
                    followingResponseSeconds +
                    std::clamp((actor.speedMetersPerSecond - 6.0) / 6.0, 0.0, 1.0);
                double closingSpeed = excessGap / responseSeconds;
                if (leaderNearlyStopped && excessGap > 0.0) {
                    closingSpeed =
                        excessGap > settleBandMeters
                            ? std::max(closingSpeed,
                                       std::min(0.5, std::sqrt(2.0 * 1.5 *
                                                               (excessGap - settleBandMeters))))
                            : 0.0;
                }
                const double followingSpeed =
                    std::max(0.0, nearestFollowingSpeedMetersPerSecond + closingSpeed);
                limitTarget(followingSpeed, MapTrafficBlockReason::Leader, nearestFollowingActor);
            }
            double signalStopDistance = std::numeric_limits<double>::infinity();
            const std::optional<TrafficSignalApproach> signalApproach =
                nextTrafficSignalApproach(actor);
            if (updateTrafficSignalStopDecision(actor, signalApproach, currentSignalClockSeconds)) {
                signalStopDistance = signalApproach->distanceToStopMeters;
            }
            if (std::isfinite(signalStopDistance)) {
                limitTarget(std::sqrt(2.0 * signalApproachDeceleration * signalStopDistance),
                            MapTrafficBlockReason::Signal);
            }
            double junctionAdmissionStopDistance = std::numeric_limits<double>::infinity();
            const double lookahead = std::max(
                15.0, actor.speedMetersPerSecond * actor.speedMetersPerSecond / (2.0 * braking) +
                          actor.speedMetersPerSecond * headwaySeconds + actor.lengthMeters);
            const double junctionEntryStopDistance =
                upcomingJunctionRange.has_value()
                    ? upcomingJunctionRange->first - actor.lengthMeters * 0.5 - 0.5
                    : std::numeric_limits<double>::infinity();
            bool yieldingTurnAcrossStraightTraffic =
                priorityTraffic != nullptr && !yieldsTo[index].empty() &&
                movementTurnClass[index] > movementTurnClass[yieldsTo[index].front()] &&
                std::isfinite(junctionArrivals[yieldsTo[index].front()]);
            if (!yieldingTurnAcrossStraightTraffic && deferrableSamplingQueries &&
                movementTurnClass[index] > 0 && upcomingJunctionRange.has_value()) {
                const JunctionClaim* approach = nullptr;
                for (const JunctionClaim& claim : junctionClaims) {
                    if (movementTurnClass[claim.actorIndex] >= movementTurnClass[index] ||
                        !mapTrafficActorOnJunctionPath(*claim.actor, claim.actor->distanceMeters)) {
                        continue;
                    }
                    if (approach == nullptr) {
                        approach = &previewClaim(index);
                    }
                    if (claimsConflict(*approach, claim).has_value()) {
                        yieldingTurnAcrossStraightTraffic = true;
                        break;
                    }
                }
            }
            const double actorJunctionYieldBufferMeters =
                deferrableSamplingQueries
                    ? (yieldingTurnAcrossStraightTraffic ? 4.0 : junctionYieldBufferMeters)
                    : 0.0;
            const double junctionYieldStopDistance =
                junctionEntryStopDistance - actorJunctionYieldBufferMeters;
            const double junctionEntryStopGap =
                std::max(0.0, junctionEntryStopDistance - actor.distanceMeters);
            const auto junctionApproachSpeed = [&](double gap) {
                constexpr double settleBandMeters = 0.25;
                constexpr double settleSpeedMetersPerSecond = 2.0 / 3.6;
                if (gap <= settleBandMeters &&
                    actor.speedMetersPerSecond <= settleSpeedMetersPerSecond) {
                    return 0.0;
                }
                return std::min(gap / headwaySeconds, std::sqrt(2.0 * braking * gap));
            };
            const bool compactJunction =
                upcomingJunctionRange.has_value() &&
                upcomingJunctionRange->second - upcomingJunctionRange->first <=
                    actor.lengthMeters * 2.0 + 15.0;
            if (priorityTraffic != nullptr) {
                const double stopDistance = compactJunction ? std::min(junctionYieldStopDistance,
                                                                       priorityStopDistances[index])
                                                            : priorityStopDistances[index];
                junctionAdmissionStopDistance =
                    std::min(junctionAdmissionStopDistance, stopDistance);
                const double gap = std::max(0.0, stopDistance - actor.distanceMeters);
                limitTarget(junctionApproachSpeed(gap), MapTrafficBlockReason::JunctionTraffic,
                            priorityTraffic);
            }
            const double movementLookahead =
                compactJunction && junctionEntryStopGap <= lookahead
                    ? std::max(lookahead, upcomingJunctionRange->second - actor.distanceMeters)
                    : lookahead;
            double clearedThroughDistance = -std::numeric_limits<double>::infinity();
            std::array<double, 4> cachedCurveSampleDistances;
            cachedCurveSampleDistances.fill(std::numeric_limits<double>::quiet_NaN());
            std::array<std::optional<MapTrafficPose>, 4> cachedCurveSamples;
            std::size_t nextCurveSampleCacheSlot = 0;
            const auto sampleCurvePose =
                [&](double sampleDistance) -> const std::optional<MapTrafficPose>& {
                for (std::size_t slot = 0; slot < cachedCurveSamples.size(); ++slot) {
                    if (cachedCurveSampleDistances[slot] == sampleDistance) {
                        return cachedCurveSamples[slot];
                    }
                }
                const std::size_t slot = nextCurveSampleCacheSlot;
                nextCurveSampleCacheSlot =
                    (nextCurveSampleCacheSlot + 1) % cachedCurveSamples.size();
                cachedCurveSampleDistances[slot] = sampleDistance;
                cachedCurveSamples[slot] = sampleMapTrafficRoute(actor.route, sampleDistance, 0.0);
                return cachedCurveSamples[slot];
            };
            const auto curveSpeedLimitAt = [&](double distance) {
                if (!deferrableSamplingQueries || !std::isfinite(distance)) {
                    return mapTrafficCurveSpeedLimit(actor.route, distance,
                                                     actor.cruiseSpeedMetersPerSecond,
                                                     actor.minimumTurnRadiusMeters);
                }
                const double fromDistance = std::max(0.0, distance - 1.0);
                const double toDistance = actor.route.closed || actor.route.loopStartDistance
                                              ? distance + 1.0
                                              : std::min(actor.route.totalLength, distance + 1.0);
                const auto from = sampleCurvePose(fromDistance);
                const auto to = sampleCurvePose(toDistance);
                if (!from || !to) {
                    throw std::invalid_argument(trafficInputError(
                        "Cannot sample traffic curve", "distanceMeters=", distance,
                        "fromDistance=", fromDistance, "toDistance=", toDistance,
                        "totalLength=", actor.route.totalLength, "fromSampled=", from.has_value(),
                        "toSampled=", to.has_value()));
                }
                return mapTrafficCurveSpeedLimitFromCurvature(
                    mapTrafficRouteCurvatureFromPoses(*from, *to),
                    actor.cruiseSpeedMetersPerSecond);
            };
            {
                openbus::rendering::TraceScope lookaheadTrace(
                    "traffic", "MapTrafficRuntime::advanceMapTrafficDriving.actorRouteLookahead");
#if OPENBUS_ENABLE_PERF_TRACE
                static const bool detailEnabled = openbus::rendering::parseEnabledFlag(
                    openbus::getEnvironment("OPENBUS_TRACE_TRAFFIC_DETAIL"));
                const bool traceDetail = detailEnabled && openbus::rendering::IsRecording();
#endif
                const MapTrafficRouteSegment* speedLimitSegment = nullptr;
                double speedLimitSegmentEnd = -std::numeric_limits<double>::infinity();
                std::optional<double> segmentSpeedLimit;
                const MapTrafficVehicleFilter vehicle{actor.vehicleType,
                                                      actor.unscheduledGroupIndex, actor.scheduled,
                                                      actor.trafficRuleGroupIndex};
                std::size_t lookaheadSampleIndex = 0;
                for (double ahead = 0.0; ahead <= movementLookahead;
                     ahead += 1.0, ++lookaheadSampleIndex) {
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
#if OPENBUS_ENABLE_PERF_TRACE
                    std::optional<openbus::rendering::TraceScope> speedTrace;
                    if (traceDetail) {
                        speedTrace.emplace(
                            "traffic",
                            "MapTrafficRuntime::advanceMapTrafficDriving.lookaheadSpeed");
                    }
#endif
                    // Curve limits never fall below this floor; skip route sampling when even
                    // the floor cannot lower the current target.
                    const double curveLimitFloor = std::min(actor.cruiseSpeedMetersPerSecond, 2.0);
                    const bool sampleCurveSpeed = !actor.reducedSpeedLookahead ||
                                                  !deferrableSamplingQueries ||
                                                  lookaheadSampleIndex % 2 == 0;
                    const double curveBrakingDistance =
                        std::max(0.0, ahead - 2.0 - (actor.reducedSpeedLookahead ? 1.0 : 0.0));
                    const double curveFloorSquared =
                        curveLimitFloor * curveLimitFloor + 2.0 * braking * curveBrakingDistance;
                    if (sampleCurveSpeed &&
                        (!deferrableSamplingQueries || curveFloorSquared < target * target)) {
                        const double curveLimit = curveSpeedLimitAt(distance);
                        limitTargetFromSquaredSpeed(curveLimit * curveLimit +
                                                    2.0 * braking * curveBrakingDistance);
                    }
                    if (!deferrableSamplingQueries || target > 0.0) {
                        const auto& segment = deferrableSamplingQueries && openRoute &&
                                                      speedLimitSegment != nullptr &&
                                                      distance <= speedLimitSegmentEnd
                                                  ? *speedLimitSegment
                                                  : routeSegmentAtDistance(actor.route, distance);
                        if (&segment != speedLimitSegment) {
                            speedLimitSegment = &segment;
                            const std::size_t segmentIndex =
                                static_cast<std::size_t>(&segment - actor.route.segments.data());
                            speedLimitSegmentEnd =
                                segmentIndex + 1 == actor.route.segments.size()
                                    ? std::numeric_limits<double>::infinity()
                                    : actor.route.cumulativeEndDistances[segmentIndex];
                            segmentSpeedLimit =
                                mapTrafficPathSpeedLimitForVehicle(segment.pathRules, vehicle);
                        }
                        if (segmentSpeedLimit.has_value()) {
                            const double limitMetersPerSecond = *segmentSpeedLimit / 3.6;
                            limitTargetFromSquaredSpeed(limitMetersPerSecond *
                                                            limitMetersPerSecond +
                                                        2.0 * braking * std::max(0.0, ahead - 2.0));
                        }
                    }
#if OPENBUS_ENABLE_PERF_TRACE
                    speedTrace.reset();
                    std::optional<openbus::rendering::TraceScope> clearanceTrace;
                    if (traceDetail) {
                        clearanceTrace.emplace(
                            "traffic",
                            "MapTrafficRuntime::advanceMapTrafficDriving.lookaheadClearance");
                    }
#endif
                    constexpr double clearance = 2.0;
                    if (!clearAt(distance, clearance, true,
                                 deferrableSamplingQueries ? std::optional<bool>(junctionAtDistance)
                                                           : std::nullopt)) {
                        const double approachBuffer =
                            committedToJunction && junctionAtDistance ? 0.0 : 1.0;
                        double availableGapMeters = std::max(0.0, ahead - approachBuffer);
                        if (upcomingJunctionRange.has_value() &&
                            distance >= upcomingJunctionRange->first - actor.lengthMeters * 0.5 -
                                            clearance &&
                            (compactJunction ? distance <= upcomingJunctionRange->second
                                             : distance <= upcomingJunctionRange->first +
                                                               actor.lengthMeters + 4.0)) {
                            availableGapMeters = std::min(availableGapMeters, junctionEntryStopGap);
                        }
                        // A time headway makes the required following distance grow with
                        // speed.
                        limitTarget(std::min(availableGapMeters / headwaySeconds,
                                             std::sqrt(2.0 * braking * availableGapMeters)),
                                    clearBlockReason, clearBlockingActor, clearBlockingObstacle);
                        break;
                    }
                    clearedThroughDistance = distance;
                }
            }
            {
                openbus::rendering::TraceScope admissionTrace(
                    "traffic",
                    "MapTrafficRuntime::advanceMapTrafficDriving.actorJunctionAdmission");
                bool junctionClaimHeld = actor.junctionClaimEndMeters.has_value() &&
                                         actor.distanceMeters < *actor.junctionClaimEndMeters;
                if (!committedToJunction && junctionClaimHeld &&
                    (std::isfinite(signalStopDistance) || priorityTraffic != nullptr)) {
                    // Signals and local priority constraints supersede unused admission.
                    actor.junctionClaimEndMeters.reset();
                    junctionClaimHeld = false;
                }
                if (!committedToJunction && !junctionClaimHeld &&
                    upcomingJunctionRange.has_value() && junctionEntryStopGap <= lookahead &&
                    (!std::isfinite(signalStopDistance) ||
                     actor.distanceMeters + signalStopDistance >=
                         upcomingJunctionRange->first - actor.lengthMeters * 0.5)) {
                    // Enter only if no actor inside or admitted to a junction will cross
                    // this path, so two vehicles never meet head-on or side-on inside the
                    // junction box.
                    JunctionClaim wanted = makeJunctionClaim(
                        actor,
                        std::max(actor.distanceMeters,
                                 upcomingJunctionRange->first - actor.lengthMeters * 0.5),
                        std::min(upcomingJunctionRange->second,
                                 actor.distanceMeters + std::max(15.0, previewDistance(actor))),
                        currentSignalClockSeconds,
                        previewClaims[index] ? &*previewClaims[index] : nullptr);
                    wanted.actorIndex = index;
                    const MapTrafficActor* conflictingActor = nullptr;
                    double conflictStopDistance = std::numeric_limits<double>::infinity();
                    for (const JunctionClaim& claim : junctionClaims) {
                        // A lower-priority claimant that can still stop gives way to this
                        // vehicle.
                        const std::size_t claimActorIndex = actorIndexByPointer.at(claim.actor);
                        if (claim.actor == &actor ||
                            routeClassByActor[index] == routeClassByActor[claim.actorIndex] ||
                            queuedBehindInLane(actor, occupied[own], *claim.actor,
                                               occupied[obstacles.size() + claimActorIndex]) ||
                            queuedBehindInLane(*claim.actor,
                                               occupied[obstacles.size() + claimActorIndex], actor,
                                               occupied[own])) {
                            continue;
                        }
                        // Opposing vehicles cannot share a non-passable corridor,
                        // regardless of its length. Hold only until the leader's tail
                        // clears the shared portion.
                        if (opposingJunctionMovements(actor, *claim.actor)) {
                            conflictingActor = claim.actor;
                            conflictStopDistance = junctionEntryStopDistance;
                            break;
                        }
                        if (yieldsForPriority(claim.actorIndex, index)) {
                            continue;
                        }
                        if (const auto conflict = claimsConflict(wanted, claim);
                            conflict.has_value() &&
                            *conflict - priorityYieldClearanceMeters < conflictStopDistance) {
                            conflictingActor = claim.actor;
                            conflictStopDistance = *conflict - priorityYieldClearanceMeters;
                        }
                    }
                    if (conflictingActor == nullptr) {
                        conflictingActor = priorityTraffic;
                        conflictStopDistance = priorityStopDistances[index];
                    }
                    if (conflictingActor != nullptr) {
                        blockedByJunctionTraffic = true;
                        junctionAdmissionStopDistance =
                            std::min(junctionAdmissionStopDistance,
                                     compactJunction
                                         ? std::min(junctionYieldStopDistance, conflictStopDistance)
                                         : conflictStopDistance);
                        const double conflictGap =
                            std::max(0.0, (compactJunction ? std::min(junctionYieldStopDistance,
                                                                      conflictStopDistance)
                                                           : conflictStopDistance) -
                                              actor.distanceMeters);
                        limitTarget(junctionApproachSpeed(conflictGap),
                                    MapTrafficBlockReason::JunctionTraffic, conflictingActor);
                    } else {
                        const double admissionWindow = actor.speedMetersPerSecond *
                                                           actor.speedMetersPerSecond /
                                                           (2.0 * braking) +
                                                       actor.speedMetersPerSecond * 0.5 + 2.0;
                        const double junctionPathEnd =
                            compactJunction ? (actor.route.closed || actor.route.loopStartDistance
                                                   ? upcomingJunctionRange->second
                                                   : std::min(upcomingJunctionRange->second,
                                                              actor.route.totalLength))
                                            : std::min(wanted.distances.back(),
                                                       actor.distanceMeters + movementLookahead);
                        // Claims follow queue order: a leader still short of the entry
                        // claims first.
                        const bool leaderStillQueuedAhead =
                            nearestFollowingActor != nullptr &&
                            nearestFollowingGapMeters < junctionEntryStopGap &&
                            !activeJunctionClaimRange(*nearestFollowingActor).has_value();
                        if (junctionEntryStopGap <= admissionWindow &&
                            clearedThroughDistance >= junctionPathEnd - 1.0 &&
                            !leaderStillQueuedAhead) {
                            actor.junctionClaimEndMeters = wanted.distances.back();
                            junctionClaims.push_back(wanted);
                        }
                    }
                }
            }
            const double scheduledStopGap =
                actor.scheduledStopDistanceMeters.has_value()
                    ? std::max(0.0, *actor.scheduledStopDistanceMeters - actor.distanceMeters)
                    : std::numeric_limits<double>::infinity();
            if (std::isfinite(scheduledStopGap)) {
                target = std::min(target, std::sqrt(2.0 * braking * scheduledStopGap));
            }
            actor.targetSpeedMetersPerSecond = target;
            const double oldSpeed = actor.speedMetersPerSecond;
            const double newSpeed = target < oldSpeed
                                        ? std::max(target, oldSpeed - braking * step)
                                        : std::min(target, oldSpeed + acceleration * step);
            const double requestedTravel = (oldSpeed + newSpeed) * 0.5 * step;
            const double junctionStopGap =
                !committedToJunction && std::isfinite(junctionAdmissionStopDistance)
                    ? std::max(0.0, junctionAdmissionStopDistance - actor.distanceMeters)
                    : std::numeric_limits<double>::infinity();
            const double travel =
                std::min({requestedTravel, signalStopDistance, junctionStopGap, scheduledStopGap});
            double admitted = 0.0;
            bool blockedDuringMovement = false;
            // Sweep rather than testing just the final pose, including at low frame rates.
            {
                openbus::rendering::TraceScope sweepTrace(
                    "traffic", "MapTrafficRuntime::advanceMapTrafficDriving.actorSweptMovement");
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
            if (actor.distanceMeters != distanceBeforeMovement) {
                updateOccupiedFootprint(own, actorFootprint(actor, actor.distanceMeters));
                currentJunctionOccupancy[index].reset();
            }
            if (actor.distanceMeters < distanceBeforeMovement) {
                actor.junctionClaimEndMeters.reset();
            }
            if (signalApproach.has_value() &&
                actor.committedSignalSegmentIndex == signalApproach->segmentIndex &&
                admitted + 1.0e-9 >= signalApproach->distanceToEntryMeters) {
                actor.committedSignalSegmentIndex.reset();
            }
            if (const auto claimRange = activeJunctionClaimRange(actor)) {
                // Hold the claim until the tail has cleared the junction group.
                actor.junctionClaimEndMeters = claimRange->second;
            } else {
                actor.junctionClaimEndMeters.reset();
            }
            if (actor.speedMetersPerSecond > 0.1) {
                openbus::rendering::TraceScope reservationTrace(
                    "traffic", "MapTrafficRuntime::advanceMapTrafficDriving.actorReservations");
                for (double ahead = 1.0; ahead < actor.speedMetersPerSecond * 3.0 &&
                                         ahead <= signalStopDistance - admitted;
                     ahead += 1.0) {
                    const double reservationDistance = actor.distanceMeters + ahead;
                    reservations.push_back(
                        {actorFootprint(actor, reservationDistance), &actor, index,
                         reservationDistance,
                         deferrableSamplingQueries
                             ? std::nullopt
                             : std::optional<double>(
                                   step + actorArrivalSeconds(actor, reservationDistance))});
                    const auto& footprint = reservations.back().footprint;
                    largestReservationRadiusMeters =
                        std::max(largestReservationRadiusMeters, physicalFootprintDiameter * 0.5);
                    if (const auto cell = occupancyCellForPosition(footprint.pose.position[0],
                                                                   footprint.pose.position[1])) {
                        reservationCells[*cell].push_back(reservations.size() - 1);
                    } else {
                        reservationIndexValid = false;
                    }
                }
            }
        }
        movementTrace.finish();
    }
}

} // namespace openbus::map
