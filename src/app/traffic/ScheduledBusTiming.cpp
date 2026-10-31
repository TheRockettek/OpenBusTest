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

} // namespace openbus::app
