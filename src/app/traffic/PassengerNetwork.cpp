#include "app/traffic/PassengerNetwork.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace openbus::app {

std::vector<std::string>
scheduledPlayerTimetableKeys(const openbus::map::MapAiTrafficData& traffic) {
    std::vector<std::string> keys;
    const auto add = [&](const auto& timetable) {
        if (!timetable.scheduledDepartureMinutes.empty() &&
            std::ranges::find(keys, timetable.timetableKey) == keys.end()) {
            keys.push_back(timetable.timetableKey);
        }
    };
    for (const auto& timetable : traffic.timetableRoutes)
        add(timetable);
    for (const auto& timetable : traffic.passengerTimetables)
        add(timetable);
    return keys;
}

PassengerStopRouteState makePlayerTimetableRoute(const openbus::map::MapAiTimetableRoute& route,
                                                 std::size_t waitingPassengers,
                                                 const PlayerStopPoseResolver& resolvePose) {
    PassengerStops::Container stops;
    std::vector<std::optional<openbus::map::MapTrafficPose>> poses;
    stops.reserve(route.stations.size());
    poses.reserve(route.stations.size());
    for (std::size_t index = 0; index < route.stations.size(); ++index) {
        const auto& station = route.stations[index];
        openbus::map::MapTrafficPassengerStop stop;
        stop.stopId = station.id;
        stop.name = station.name;
        stop.timetableStationIndex = index;
        stop.waitingPassengers = waitingPassengers;
        stop.dwellSeconds = 8.0;
        std::optional<openbus::map::MapTrafficPose> pose;
        const auto position = openbus::map::mapAiTimetableStationPosition(route, station);
        if (position && route.trackEntries[position->trackEntryIndex].pathIndex >= 0 &&
            station.id != 0) {
            const auto& entry = route.trackEntries[position->trackEntryIndex];
            stop.splineId = entry.splineId;
            stop.pathIndex = static_cast<std::size_t>(entry.pathIndex);
            stop.trackEntryIndex = position->trackEntryIndex;
            if (station.positionOnPreviousTrackEntryMeters) {
                stop.splinePositionMeters = position->offsetMeters;
            }
            pose = resolvePose(stop);
        }
        stops.push_back(std::move(stop));
        poses.push_back(std::move(pose));
    }
    for (std::size_t index = poses.size(); index > 0; --index) {
        if (poses[index - 1]) {
            stops[index - 1].passengersAlighting = (std::numeric_limits<std::size_t>::max)();
            break;
        }
    }
    return {{},
            PassengerStops(std::move(stops)),
            route.timetableKey,
            route.lineName,
            route.destinationName,
            route.tripName,
            std::move(poses),
            route.profiles};
}

PassengerRouteRegistrationReport
PassengerNetwork::registerTimetables(const openbus::map::MapAiTrafficData& traffic,
                                     std::size_t waitingPassengers,
                                     const PlayerStopPoseResolver& resolvePose) {
    PassengerRouteRegistrationReport report;
    const auto registerRoute = [&](PassengerStopRouteState route) {
        const auto missing =
            std::ranges::count_if(route.stopPoses, [](const auto& pose) { return !pose; });
        report.missingStops += static_cast<std::size_t>(missing);
        report.routesWithMissingStops += missing != 0 ? 1 : 0;
        if (missing == static_cast<std::ptrdiff_t>(route.stops.size())) {
            ++report.routesWithoutStops;
            if (report.routesWithoutStopExamples.size() < 500) {
                report.routesWithoutStopExamples +=
                    (report.routesWithoutStopExamples.empty() ? "" : ", ") + route.timetableKey;
            }
        }
        for (std::size_t index = 0; index < route.stops.size(); ++index) {
            const auto& stop = route.stops[index];
            if (!route.stopPoses[index]) {
                if (report.missingExamples.size() < 500) {
                    report.missingExamples +=
                        (report.missingExamples.empty() ? "" : "; ") + route.timetableKey + ": " +
                        stop.name + " (spline " + std::to_string(stop.splineId) + ", path " +
                        std::to_string(stop.pathIndex) + ", offset " +
                        std::to_string(stop.splinePositionMeters.value_or(0.0)) + ")";
                }
                continue;
            }
            initializeQueues(std::span(&stop, 1));
        }
        routes.push_back(std::move(route));
    };
    for (const auto& timetable : traffic.timetableRoutes) {
        if (timetable.scheduledDepartureMinutes.empty() || timetable.stations.empty()) {
            continue;
        }
        registerRoute(makePlayerTimetableRoute(timetable, waitingPassengers, resolvePose));
    }
    for (const auto& timetable : traffic.passengerTimetables) {
        if (timetable.scheduledDepartureMinutes.empty() || timetable.stationIds.empty() ||
            findRouteIndex(timetable.timetableKey)) {
            continue;
        }
        if (!timetable.stationRecords.empty() &&
            timetable.stationRecords.size() != timetable.stationIds.size()) {
            throw std::logic_error("Passenger timetable station records lost alignment");
        }
        const auto calls = openbus::map::mapAiPassengerStopCalls(traffic, timetable);
        PassengerStops::Container stops;
        std::vector<std::optional<openbus::map::MapTrafficPose>> poses;
        stops.reserve(calls.size());
        poses.reserve(calls.size());
        bool missingDefinition = false;
        for (std::size_t index = 0; index < calls.size(); ++index) {
            const auto& call = calls[index];
            const auto* authored =
                !timetable.stationRecords.empty() && timetable.stationRecords[index]
                    ? &*timetable.stationRecords[index]
                    : nullptr;
            if (!authored && !call.busStopIndex) {
                missingDefinition = true;
                if (report.missingStopDefinitionExamples.size() < 500) {
                    report.missingStopDefinitionExamples +=
                        (report.missingStopDefinitionExamples.empty() ? "" : "; ") +
                        timetable.timetableKey + ": stop ID " + std::to_string(call.stationId);
                }
                continue;
            }
            openbus::map::MapTrafficPassengerStop stop;
            stop.stopId = call.stationId;
            stop.name = authored ? authored->name : traffic.busStops[*call.busStopIndex].name;
            stop.timetableStationIndex = index;
            stop.waitingPassengers = waitingPassengers;
            stop.dwellSeconds = 8.0;
            std::optional<openbus::map::MapTrafficPose> pose;
            const openbus::map::MapAiStationLink::Entry* entry = nullptr;
            bool useIncoming = false;
            if (call.outgoingStationLinkIndex) {
                const auto& link = traffic.stationLinks[*call.outgoingStationLinkIndex];
                if (!link.entries.empty())
                    entry = &link.entries.front();
            }
            if (!entry && index > 0 && calls[index - 1].outgoingStationLinkIndex) {
                const auto& link = traffic.stationLinks[*calls[index - 1].outgoingStationLinkIndex];
                if (!link.entries.empty()) {
                    entry = &link.entries.back();
                    useIncoming = true;
                }
            }
            if (entry && entry->splineId != 0 && entry->pathIndex >= 0 &&
                (!useIncoming ||
                 (std::isfinite(entry->lengthMeters) && entry->lengthMeters >= 0))) {
                stop.splineId = entry->splineId;
                stop.pathIndex = static_cast<std::size_t>(entry->pathIndex);
                if (useIncoming)
                    stop.splinePositionMeters = entry->lengthMeters;
                pose = resolvePose(stop);
            }
            stops.push_back(std::move(stop));
            poses.push_back(std::move(pose));
        }
        if (missingDefinition) {
            ++report.routesMissingStopDefinitions;
            continue;
        }
        for (std::size_t index = poses.size(); index > 0; --index) {
            if (poses[index - 1]) {
                stops[index - 1].passengersAlighting = (std::numeric_limits<std::size_t>::max)();
                break;
            }
        }
        registerRoute({{},
                       PassengerStops(std::move(stops)),
                       timetable.timetableKey,
                       timetable.lineName,
                       timetable.destinationName,
                       timetable.tripName,
                       std::move(poses),
                       timetable.profiles});
    }
    return report;
}

std::optional<std::size_t> PassengerNetwork::findRouteIndex(const std::string& key) const {
    const auto found = std::ranges::find(routes, key, &PassengerStopRouteState::timetableKey);
    if (found == routes.end()) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(std::distance(routes.begin(), found));
}

void PassengerNetwork::initializeQueues(
    std::span<const openbus::map::MapTrafficPassengerStop> stops) {
    for (const auto& stop : stops) {
        const auto [queue, inserted] = queues.try_emplace(stop.stopId, stop.waitingPassengers);
        if (inserted) {
            initialQueues.emplace(stop.stopId, queue->second);
        }
    }
}

PassengerStops::Container mapPassengerStopsToTrafficRoute(
    const openbus::map::MapTrafficRoute& route, PassengerStops::Container stops,
    const std::function<void(const openbus::map::MapTrafficPassengerStop&)>& onUnmatched) {
    PassengerStops::Container mappedStops;
    std::size_t routeSegmentCursor = 0;
    for (auto& stop : stops) {
        if (stop.stopId == 0 || stop.dwellSeconds < 0.0 || !std::isfinite(stop.dwellSeconds) ||
            (stop.splinePositionMeters &&
             (!std::isfinite(*stop.splinePositionMeters) || *stop.splinePositionMeters < 0.0))) {
            throw std::invalid_argument("Scheduled map bus passenger stop is invalid");
        }
        if (stop.splinePositionMeters) {
            const auto position = openbus::map::mapTrafficRoutePositionOnSpline(
                route, stop.splineId, stop.pathIndex, *stop.splinePositionMeters,
                routeSegmentCursor);
            if (!position) {
                onUnmatched(stop);
                continue;
            }
            routeSegmentCursor = position->segmentIndex;
            stop.routeDistanceMeters = position->distanceMeters;
        } else {
            const auto segment = std::find_if(
                route.segments.begin() + static_cast<std::ptrdiff_t>(routeSegmentCursor),
                route.segments.end(), [&](const auto& candidate) {
                    return openbus::map::mapTrafficSegmentMatchesTimetablePath(
                        candidate, stop.splineId, stop.pathIndex);
                });
            if (segment == route.segments.end()) {
                onUnmatched(stop);
                continue;
            }
            routeSegmentCursor =
                static_cast<std::size_t>(std::distance(route.segments.begin(), segment));
            stop.routeDistanceMeters = routeSegmentCursor == 0
                                           ? 0.0
                                           : route.cumulativeEndDistances[routeSegmentCursor - 1];
        }
        mappedStops.push_back(std::move(stop));
    }
    if (!mappedStops.empty()) {
        mappedStops.back().passengersAlighting = (std::numeric_limits<std::size_t>::max)();
    }
    return mappedStops;
}

NearbyPassengerStop PassengerNetwork::findNearestStop(const std::array<double, 3>& position,
                                                      double headingRadians,
                                                      std::optional<std::size_t> selectedRouteIndex,
                                                      std::size_t firstStopIndex) const {
    NearbyPassengerStop nearest;
    for (std::size_t routeIndex = 0; routeIndex < routes.size(); ++routeIndex) {
        if (selectedRouteIndex && routeIndex != *selectedRouteIndex) {
            continue;
        }
        const auto& route = routes[routeIndex];
        for (std::size_t stopIndex = 0; stopIndex < route.stops.size(); ++stopIndex) {
            if (selectedRouteIndex && stopIndex < firstStopIndex) {
                continue;
            }
            const auto pose = route.stopPose(stopIndex);
            if (!pose) {
                continue;
            }
            const double distance =
                std::hypot(pose->position[0] - position[0], pose->position[1] - position[1]);
            const double difference =
                std::remainder(headingRadians - pose->headingRadians, 2.0 * std::numbers::pi);
            if (std::abs(difference) > std::numbers::pi / 3.0) {
                continue;
            }
            if (distance < nearest.distanceMeters) {
                nearest = {routeIndex, stopIndex, distance};
            }
        }
    }
    return nearest;
}

std::optional<std::size_t> nextLocatedPassengerStop(const PassengerStopRouteState& route,
                                                    std::size_t firstStopIndex) {
    for (std::size_t index = firstStopIndex; index < route.stops.size(); ++index) {
        if (route.stopPose(index)) {
            return index;
        }
    }
    return std::nullopt;
}

} // namespace openbus::app
