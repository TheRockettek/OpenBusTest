#include "app/traffic/ScheduledBusTiming.h"

#include <algorithm>
#include <cmath>

namespace openbus::app {

const openbus::map::MapAiTimetableProfile*
scheduledBusProfileForDeparture(const ScheduledBusState& bus, std::size_t departureIndex) {
    if (!bus.timetableProfiles || bus.timetableProfiles->empty() ||
        departureIndex >= bus.scheduledDepartureProfileIndices.size()) {
        return nullptr;
    }
    const int requestedProfile = std::max(0, bus.scheduledDepartureProfileIndices[departureIndex]);
    const std::size_t profileIndex =
        std::min(static_cast<std::size_t>(requestedProfile), bus.timetableProfiles->size() - 1);
    return &(*bus.timetableProfiles)[profileIndex];
}

double scheduledBusDelaySeconds(const ScheduledBusState& bus,
                                const openbus::map::MapTrafficActor& actor,
                                double simulationTimeSeconds) {
    if (!bus.scheduledTripActive || bus.passengerStops.empty()) {
        return 0.0;
    }
    const auto* profile = scheduledBusProfileForDeparture(bus, bus.activeTimetableDepartureIndex);
    if (profile == nullptr) {
        return 0.0;
    }
    const auto arrivalOffset = [&](std::size_t stopIndex) {
        const std::size_t stationIndex = bus.passengerStops[stopIndex].timetableStationIndex;
        return stationIndex < profile->arrivalOffsetSeconds.size()
                   ? profile->arrivalOffsetSeconds[stationIndex]
                   : 0.0;
    };
    const auto departureOffset = [&](std::size_t stopIndex) {
        const std::size_t stationIndex = bus.passengerStops[stopIndex].timetableStationIndex;
        return stationIndex < profile->departureOffsetSeconds.size()
                   ? profile->departureOffsetSeconds[stationIndex]
                   : 0.0;
    };
    const double elapsedSinceDeparture =
        std::max(0.0, simulationTimeSeconds - bus.activeScheduledDepartureElapsedSeconds);
    const double distance = actor.distanceMeters;
    constexpr double stopPositionToleranceMeters = 0.25;
    const auto dwellProgress = [&](std::size_t stopIndex) {
        return std::clamp(elapsedSinceDeparture, arrivalOffset(stopIndex),
                          departureOffset(stopIndex));
    };
    const double firstStopDistance = bus.passengerStops.front().routeDistanceMeters;
    if (distance <= firstStopDistance + stopPositionToleranceMeters) {
        const double expected =
            std::abs(distance - firstStopDistance) <= stopPositionToleranceMeters
                ? dwellProgress(0)
                : departureOffset(0);
        return elapsedSinceDeparture - expected;
    }
    for (std::size_t index = 0; index + 1 < bus.passengerStops.size(); ++index) {
        const double fromDistance = bus.passengerStops[index].routeDistanceMeters;
        const double toDistance = bus.passengerStops[index + 1].routeDistanceMeters;
        if (std::abs(distance - fromDistance) <= stopPositionToleranceMeters) {
            return elapsedSinceDeparture - dwellProgress(index);
        }
        if (distance < toDistance && toDistance > fromDistance) {
            const double fraction =
                std::clamp((distance - fromDistance) / (toDistance - fromDistance), 0.0, 1.0);
            const double expected = departureOffset(index) +
                                    (arrivalOffset(index + 1) - departureOffset(index)) * fraction;
            return elapsedSinceDeparture - expected;
        }
    }
    const std::size_t lastStopIndex = bus.passengerStops.size() - 1;
    const double lastStopDistance = bus.passengerStops[lastStopIndex].routeDistanceMeters;
    const double expected = std::abs(distance - lastStopDistance) <= stopPositionToleranceMeters
                                ? dwellProgress(lastStopIndex)
                                : departureOffset(lastStopIndex);
    return elapsedSinceDeparture - expected;
}

std::optional<double> playerTimetableDelaySeconds(
    const PassengerStopRouteState& route, const openbus::map::MapAiTimetableProfile& profile,
    double departureMinutes, double currentTimeOfDaySeconds, std::size_t firstStopIndex,
    std::size_t nextStopIndex, const std::array<double, 3>& vehiclePosition,
    std::optional<std::size_t> dwellingStopIndex) {
    if (!std::isfinite(departureMinutes) || !std::isfinite(currentTimeOfDaySeconds) ||
        route.stops.empty()) {
        return std::nullopt;
    }

    const auto targetStopIndex = nextLocatedPassengerStop(route, nextStopIndex);
    if (!targetStopIndex) {
        return std::nullopt;
    }
    const auto offset = [&](std::size_t stopIndex,
                            const std::vector<double>& offsets) -> std::optional<double> {
        const std::size_t stationIndex = route.stops[stopIndex].timetableStationIndex;
        if (stationIndex >= offsets.size() || !std::isfinite(offsets[stationIndex])) {
            return std::nullopt;
        }
        return offsets[stationIndex];
    };
    const auto arrivalOffset = offset(*targetStopIndex, profile.arrivalOffsetSeconds);
    const auto departureOffset = offset(*targetStopIndex, profile.departureOffsetSeconds);
    if (!arrivalOffset || !departureOffset || *departureOffset < *arrivalOffset) {
        return std::nullopt;
    }

    double elapsedSinceDeparture =
        currentTimeOfDaySeconds - std::fmod(departureMinutes * 60.0, 86400.0);
    if (elapsedSinceDeparture < -43200.0) {
        elapsedSinceDeparture += 86400.0;
    } else if (elapsedSinceDeparture > 43200.0) {
        elapsedSinceDeparture -= 86400.0;
    }

    double expected = *arrivalOffset;
    if (dwellingStopIndex == targetStopIndex) {
        expected = std::clamp(elapsedSinceDeparture, *arrivalOffset, *departureOffset);
    } else {
        std::optional<std::size_t> previousStopIndex;
        for (std::size_t index = *targetStopIndex; index > firstStopIndex; --index) {
            const std::size_t candidateIndex = index - 1;
            if (route.stopPose(candidateIndex)) {
                previousStopIndex = candidateIndex;
                break;
            }
        }
        if (previousStopIndex) {
            const auto previousDeparture =
                offset(*previousStopIndex, profile.departureOffsetSeconds);
            const auto previousPose = route.stopPose(*previousStopIndex);
            const auto targetPose = route.stopPose(*targetStopIndex);
            if (!previousDeparture || !previousPose || !targetPose) {
                return std::nullopt;
            }
            const double distanceFromPrevious =
                std::hypot(vehiclePosition[0] - previousPose->position[0],
                           vehiclePosition[1] - previousPose->position[1]);
            const double distanceToTarget =
                std::hypot(targetPose->position[0] - vehiclePosition[0],
                           targetPose->position[1] - vehiclePosition[1]);
            const double totalDistance = distanceFromPrevious + distanceToTarget;
            const double progress = totalDistance > 1.0e-6
                                        ? std::clamp(distanceFromPrevious / totalDistance, 0.0, 1.0)
                                        : 0.0;
            if (*arrivalOffset < *previousDeparture) {
                return std::nullopt;
            }
            expected = *previousDeparture + (*arrivalOffset - *previousDeparture) * progress;
        }
    }
    return elapsedSinceDeparture - expected;
}

} // namespace openbus::app
