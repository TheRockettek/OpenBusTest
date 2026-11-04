#include "app/traffic/PassengerNetwork.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <set>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
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

std::optional<std::size_t> PassengerNetwork::routeCallIndexForBusStop(
    std::size_t routeIndex, const PassengerStops& busStops, std::size_t busStopIndex) const {
    if (routeIndex >= routes.size() || busStopIndex >= busStops.size()) {
        return std::nullopt;
    }
    const auto& routeStops = routes[routeIndex].stops.values();
    const auto& busStop = busStops[busStopIndex];
    if (routeStops == busStops.values()) {
        return busStopIndex;
    }
    std::optional<std::size_t> matchedIndex;
    for (std::size_t index = 0; index < routeStops.size(); ++index) {
        const auto& routeStop = routeStops[index];
        if (routeStop.stopId != busStop.stopId ||
            routeStop.timetableStationIndex != busStop.timetableStationIndex) {
            continue;
        }
        if (matchedIndex) {
            return std::nullopt;
        }
        matchedIndex = index;
    }
    if (matchedIndex) {
        return matchedIndex;
    }
    for (std::size_t index = 0; index < routeStops.size(); ++index) {
        if (routeStops[index].stopId != busStop.stopId) {
            continue;
        }
        if (matchedIndex) {
            return std::nullopt;
        }
        matchedIndex = index;
    }
    return matchedIndex;
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

void PassengerNetwork::syncQueue(int stopId) {
    const auto queue = queues.find(stopId);
    if (queue == queues.end()) {
        throw std::invalid_argument("Passenger queue stop is not registered");
    }
    auto& state = queueStates_[stopId];
    if (!state.initialized) {
        const auto initial = initialQueues.find(stopId);
        state.capacity = initial == initialQueues.end() ? queue->second
                                                        : std::max(queue->second, initial->second);
        state.passengers.reserve(queue->second);
        for (std::size_t index = 0; index < queue->second; ++index) {
            const std::size_t slot = state.capacity - index - 1;
            state.passengers.push_back(
                {openbus::map::mapScheduledPassengerModelIdentity(stopId, index), slot, slot, {}});
        }
        state.nextArrivalOrder = state.capacity;
        state.nextModelIndex = queue->second;
        state.initialized = true;
    }
    if (state.passengers.size() > queue->second) {
        throw std::logic_error("Passenger queue count is smaller than its waiting records");
    }
    if (state.passengers.size() == queue->second) {
        return;
    }
    std::unordered_set<std::size_t> occupied;
    for (const auto& passenger : state.passengers) {
        occupied.insert(passenger.queueSlot);
    }
    for (const auto& [service, reservation] : reservations_) {
        (void)service;
        if (reservation.stopId == stopId) {
            occupied.insert(reservation.passenger.queueSlot);
        }
    }
    std::size_t slot = 0;
    while (state.passengers.size() < queue->second) {
        while (slot < state.capacity && occupied.contains(slot)) {
            ++slot;
        }
        if (slot == state.capacity) {
            break;
        }
        if (state.nextArrivalOrder == (std::numeric_limits<std::size_t>::max)() ||
            state.nextModelIndex == (std::numeric_limits<std::size_t>::max)()) {
            throw std::overflow_error("Passenger queue identity space exhausted");
        }
        const auto model =
            openbus::map::mapScheduledPassengerModelIdentity(stopId, state.nextModelIndex);
        state.passengers.push_back({model, slot, state.nextArrivalOrder, {}});
        occupied.insert(slot);
        ++state.nextArrivalOrder;
        ++state.nextModelIndex;
    }
}

const std::vector<WaitingPassenger>& PassengerNetwork::waitingPassengers(int stopId) {
    syncQueue(stopId);
    auto& state = queueStates_.at(stopId);
    auto& passengers = state.passengers;
    if (std::ranges::all_of(passengers,
                            [](const auto& passenger) { return passenger.journey.has_value(); })) {
        return passengers;
    }
    if (state.candidateRouteCount != routes.size()) {
        std::vector<WaitingPassengerJourney> candidates;
        std::set<std::tuple<std::size_t, std::size_t, std::size_t>> seen;
        for (std::size_t routeIndex = 0; routeIndex < routes.size(); ++routeIndex) {
            const auto& route = routes[routeIndex];
            for (std::size_t origin = 0; origin < route.stops.size(); ++origin) {
                if (route.stops[origin].stopId != stopId || !route.stopPose(origin)) {
                    continue;
                }
                for (std::size_t destination = origin + 1; destination < route.stops.size();
                     ++destination) {
                    const int destinationId = route.stops[destination].stopId;
                    const auto destinationPose = route.stopPose(destination);
                    if (destinationId > 0 && destinationPose &&
                        seen.emplace(routeIndex, origin, destination).second) {
                        candidates.push_back(
                            {routeIndex, destinationId, destinationPose->position, false,
                             std::nullopt, origin, destination});
                    }
                }
            }
        }
        state.candidates = std::move(candidates);
        state.candidateRouteCount = routes.size();
    }
    if (!state.candidates.empty()) {
        std::vector<std::size_t> candidateRoutes;
        std::unordered_map<std::size_t, std::vector<std::size_t>> candidatesByRoute;
        for (std::size_t index = 0; index < state.candidates.size(); ++index) {
            const std::size_t routeIndex = state.candidates[index].routeIndex;
            auto& routeCandidates = candidatesByRoute[routeIndex];
            if (routeCandidates.empty()) {
                candidateRoutes.push_back(routeIndex);
            }
            routeCandidates.push_back(index);
        }
        for (auto& passenger : passengers) {
            if (!passenger.journey) {
                const std::size_t routeOrdinal = passenger.modelIndex % candidateRoutes.size();
                const std::size_t routeIndex = candidateRoutes[routeOrdinal];
                const auto& routeCandidates = candidatesByRoute.at(routeIndex);
                const std::size_t candidateOrdinal =
                    (passenger.modelIndex / candidateRoutes.size()) % routeCandidates.size();
                passenger.journey = state.candidates[routeCandidates[candidateOrdinal]];
            }
        }
    }
    return passengers;
}

bool PassengerNetwork::passengerMatchesService(
    const WaitingPassenger& passenger, std::size_t routeIndex,
    const openbus::map::MapPassengerServiceRuntime& service,
    std::optional<std::size_t> currentStopIndex) const {
    if (!service.acceptsBoarding() || !passenger.journey || routeIndex >= routes.size() ||
        passenger.journey->routeIndex >= routes.size()) {
        return false;
    }
    const auto& actual = routes[routeIndex];
    const bool sameService = passenger.journey->routeIndex == routeIndex;
    if (!sameService || !service.acceptsPassengerDestination(passenger.journey->destinationStopId)) {
        return false;
    }
    const std::size_t originIndex = passenger.journey->originStopIndex;
    const std::size_t destinationIndex = passenger.journey->destinationStopIndex;
    if (originIndex >= actual.stops.size() || destinationIndex >= actual.stops.size() ||
        destinationIndex <= originIndex ||
        actual.stops[originIndex].stopId != service.currentStopId() ||
        (currentStopIndex && *currentStopIndex != originIndex) ||
        actual.stops[destinationIndex].stopId != passenger.journey->destinationStopId) {
        return false;
    }
    const auto& destination = passenger.journey->pedestrianJourney
                                  ? passenger.journey->pedestrianJourney->destination
                                  : passenger.journey->destinationPosition;
    const auto pose = actual.stopPose(destinationIndex);
    return pose && std::hypot(pose->position[0] - destination[0],
                              pose->position[1] - destination[1]) <=
                       openbus::map::passengerStopRequestRadiusMeters;
}

std::optional<std::size_t>
PassengerNetwork::nextPassenger(int stopId, std::size_t routeIndex,
                                const openbus::map::MapPassengerServiceRuntime& service,
                                std::optional<std::size_t> currentStopIndex) {
    const auto& passengers = waitingPassengers(stopId);
    std::optional<std::size_t> result;
    for (std::size_t index = 0; index < passengers.size(); ++index) {
        if (passengerMatchesService(passengers[index], routeIndex, service, currentStopIndex) &&
            (!result || passengers[index].arrivalOrder < passengers[*result].arrivalOrder)) {
            result = index;
        }
    }
    return result;
}

void PassengerNetwork::resolvePassengerPedestrianJourney(
    int stopId, std::size_t passengerIndex,
    std::optional<openbus::map::MapPassengerPedestrianJourney> pedestrianJourney) {
    syncQueue(stopId);
    auto& passenger = queueStates_.at(stopId).passengers.at(passengerIndex);
    if (!passenger.journey || passenger.journey->pedestrianResolved) {
        throw std::logic_error("Passenger pedestrian journey is missing or already resolved");
    }
    passenger.journey->pedestrianJourney = std::move(pedestrianJourney);
    passenger.journey->pedestrianResolved = true;
}

void PassengerNetwork::advanceService(openbus::map::MapPassengerServiceRuntime& service,
                                      std::size_t routeIndex,
                                      std::optional<std::size_t> passengerIndex,
                                      double elapsedSeconds, bool allowBoarding,
                                      bool allowAlighting, bool holdAtStop, bool pauseDwellTimer,
                                      std::optional<std::size_t> currentStopIndex) {
    const int stopId = service.currentStopId();
    const auto& passengers = waitingPassengers(stopId);
    auto& count = queues.at(stopId);
    const bool wasBoarding = service.boardingPassenger();
    if (wasBoarding != reservations_.contains(&service)) {
        throw std::logic_error("Passenger boarding reservation does not match the service");
    }
    std::optional<openbus::map::MapPassengerBoardingSelection> selection;
    if (!wasBoarding && passengerIndex) {
        if (*passengerIndex >= passengers.size() ||
            !passengerMatchesService(passengers[*passengerIndex], routeIndex, service,
                                     currentStopIndex)) {
            throw std::invalid_argument("Selected waiting passenger does not match the service");
        }
        const auto& passenger = passengers[*passengerIndex];
        selection = openbus::map::MapPassengerBoardingSelection{
            passenger.modelIndex, passenger.queueSlot, passenger.journey->destinationStopId,
            passenger.journey->pedestrianJourney, passenger.journey->destinationPosition};
    }
    service.setWaitingPassengers(stopId, count);
    service.advance(elapsedSeconds, allowBoarding, allowAlighting, holdAtStop, true, nullptr, true,
                    selection ? &*selection : nullptr, true, pauseDwellTimer);
    auto& state = queueStates_.at(stopId);
    if (!wasBoarding && service.boardingPassenger()) {
        if (!passengerIndex || !selection) {
            throw std::logic_error("Passenger boarded without a journey reservation");
        }
        reservations_.emplace(&service, Reservation{stopId, state.passengers[*passengerIndex]});
        state.passengers.erase(state.passengers.begin() +
                               static_cast<std::ptrdiff_t>(*passengerIndex));
    } else if (wasBoarding && !service.boardingPassenger()) {
        reservations_.erase(&service);
    }
    count = service.waitingPassengers(stopId);
    std::size_t eligible = 0;
    for (const auto& passenger : state.passengers) {
        if (passengerMatchesService(passenger, routeIndex, service, currentStopIndex)) {
            ++eligible;
        }
    }
    service.setEligibleWaitingPassengers(eligible);
}

void PassengerNetwork::restoreReservation(openbus::map::MapPassengerServiceRuntime& service,
                                          bool endService) {
    const int stopId = service.currentStopId();
    syncQueue(stopId);
    auto& count = queues.at(stopId);
    const auto reservation = reservations_.find(&service);
    if (service.boardingPassenger() != (reservation != reservations_.end())) {
        throw std::logic_error("Passenger cancellation has no matching reservation");
    }
    service.setWaitingPassengers(stopId, count);
    if (endService) {
        service.endServiceAtCurrentStop();
    } else {
        service.depart();
    }
    if (reservation != reservations_.end()) {
        if (reservation->second.stopId != stopId ||
            service.waitingPassengers(stopId) != count + 1) {
            throw std::logic_error("Passenger cancellation did not return its reserved rider");
        }
        queueStates_.at(stopId).passengers.push_back(reservation->second.passenger);
        reservations_.erase(reservation);
    }
    count = service.waitingPassengers(stopId);
}

void PassengerNetwork::departService(openbus::map::MapPassengerServiceRuntime& service) {
    restoreReservation(service, false);
}

void PassengerNetwork::endService(openbus::map::MapPassengerServiceRuntime& service) {
    restoreReservation(service, true);
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
                                                      std::size_t firstStopIndex,
                                                      const std::function<std::optional<std::array<double, 3>>(
                                                          const openbus::map::MapTrafficPassengerStop&)>&
                                                          resolvePosition) const {
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
            const auto resolved = resolvePosition ? resolvePosition(route.stops[stopIndex])
                                                  : std::nullopt;
            const auto& stopPosition = resolved ? *resolved : pose->position;
            const double distance =
                std::hypot(stopPosition[0] - position[0], stopPosition[1] - position[1]);
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

std::optional<std::size_t> nextPassengerStopWithPosition(
    const PassengerStopRouteState& route, std::size_t firstStopIndex,
    const std::function<std::optional<std::array<double, 3>>(
        const openbus::map::MapTrafficPassengerStop&)>& resolveEditorPosition) {
    for (std::size_t index = firstStopIndex; index < route.stops.size(); ++index) {
        if (route.stopPose(index) ||
            (resolveEditorPosition && resolveEditorPosition(route.stops[index]))) {
            return index;
        }
    }
    return std::nullopt;
}

} // namespace openbus::app
