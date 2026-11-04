#pragma once

#include "map/MapConfigLoader.h"
#include "map/MapTrafficRuntime.h"
#include "app/traffic/PassengerStops.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace openbus::app {

struct ScheduledDepartureSuppression {
    std::size_t departureIndex;
    double departureElapsedSeconds;
};

struct ScheduledBusState {
    std::vector<double> scheduledDepartureMinutes;
    std::vector<int> scheduledDepartureProfileIndices;
    std::shared_ptr<const std::vector<openbus::map::MapAiTimetableProfile>> timetableProfiles;
    PassengerStops passengerStops;
    std::optional<std::size_t> passengerRouteIndex;
    std::string timetableKey;
    std::string timetableLineName;
    std::string timetableDestinationName;
    std::size_t nextPassengerStopIndex = 0;
    bool passengerStopArrived = false;
    bool passengerStopDepartureRequested = false;
    double nextScheduledDepartureElapsedSeconds = 0.0;
    double lastScheduledDepartureMinutes = 0.0;
    std::size_t nextScheduledDepartureIndex = 0;
    std::size_t activeTimetableDepartureIndex = 0;
    double activeScheduledDepartureElapsedSeconds = 0.0;
    int timetableTerminusIndex = -1;
    bool scheduledTripActive = false;
    bool scheduledServiceCancelled = false;
    bool retirementDrivingToTerminus = false;
    std::optional<ScheduledDepartureSuppression> suppressedDeparture;
    unsigned lastScheduledAdmissionDiagnosticBlockerMask = 16;
    double lastScheduledAdmissionDiagnosticCameraDistanceMeters = -1.0;
    double lastScheduledProgressDiagnosticDistanceMeters = -1.0;
};

inline void rebaseScheduledBusClock(ScheduledBusState& bus, double startHour,
                                    double previousElapsedSeconds) {
    if (!std::isfinite(previousElapsedSeconds) || previousElapsedSeconds < 0.0) {
        throw std::invalid_argument("Previous scheduled bus clock time is invalid");
    }
    if (bus.scheduledTripActive) {
        bus.activeScheduledDepartureElapsedSeconds -= previousElapsedSeconds;
    }
    if (bus.scheduledDepartureMinutes.empty()) {
        bus.suppressedDeparture.reset();
        return;
    }
    bus.nextScheduledDepartureIndex =
        openbus::map::mapTrafficNextDepartureIndex(startHour, 0.0,
                                                   bus.scheduledDepartureMinutes);
    bus.nextScheduledDepartureElapsedSeconds = openbus::map::mapTrafficNextDepartureElapsedSeconds(
        startHour, 0.0, bus.scheduledDepartureMinutes[bus.nextScheduledDepartureIndex]);
    if (bus.suppressedDeparture) {
        bus.suppressedDeparture = ScheduledDepartureSuppression{
            bus.nextScheduledDepartureIndex, bus.nextScheduledDepartureElapsedSeconds};
    }
}

inline bool scheduledBusWaitingForPassengers(const ScheduledBusState& bus) {
    // Arrival remains latched through boarding, dwell, and the script's door-close handshake.
    return bus.scheduledTripActive && bus.passengerStopArrived;
}

inline openbus::map::MapPassengerStopService
scheduledBusStopService(const ScheduledBusState& bus,
                        const openbus::map::MapPassengerServiceRuntime& service) {
    if (bus.nextPassengerStopIndex >= bus.passengerStops.size()) {
        throw std::out_of_range("Scheduled bus has no next passenger stop");
    }
    const auto& stop = bus.passengerStops[bus.nextPassengerStopIndex];
    const std::size_t nextIndex = bus.nextPassengerStopIndex + 1;
    const auto destinations =
        bus.scheduledServiceCancelled
            ? std::vector<int>{}
            : onwardPassengerStopIds(bus.passengerStops, nextIndex, stop.stopId,
                                     [](std::size_t) { return true; });
    return {.stopId = stop.stopId,
            .waitingPassengers = 0,
            .passengersAlighting = bus.scheduledServiceCancelled ? service.onboardPassengerCount()
                                                                 : stop.passengersAlighting,
            .dwellSeconds = stop.dwellSeconds,
            .nextStopId = destinations.empty() ? 0 : destinations.front(),
            .onwardStopIds = destinations};
}

inline bool shouldSuppressScheduledDeparture(const ScheduledBusState& bus,
                                             double simulationTimeSeconds) {
    return bus.suppressedDeparture && !bus.scheduledTripActive &&
           bus.nextScheduledDepartureIndex == bus.suppressedDeparture->departureIndex &&
           bus.nextScheduledDepartureElapsedSeconds ==
               bus.suppressedDeparture->departureElapsedSeconds &&
           openbus::map::mapTrafficScheduledDepartureDue(simulationTimeSeconds,
                                                         bus.nextScheduledDepartureElapsedSeconds);
}

inline void advanceScheduledBusDeparture(ScheduledBusState& bus, double startHour,
                                         double afterElapsedSeconds) {
    if (bus.scheduledDepartureMinutes.empty()) {
        throw std::logic_error("Cannot advance an empty scheduled departure list");
    }
    bus.nextScheduledDepartureIndex =
        (bus.nextScheduledDepartureIndex + 1) % bus.scheduledDepartureMinutes.size();
    bus.nextScheduledDepartureElapsedSeconds = openbus::map::mapTrafficNextDepartureElapsedSeconds(
        startHour, afterElapsedSeconds + 1.0e-6,
        bus.scheduledDepartureMinutes[bus.nextScheduledDepartureIndex]);
}

inline bool consumeSuppressedDeparture(ScheduledBusState& bus, double startHour,
                                       double simulationTimeSeconds) {
    if (!shouldSuppressScheduledDeparture(bus, simulationTimeSeconds)) {
        return false;
    }
    advanceScheduledBusDeparture(bus, startHour, bus.nextScheduledDepartureElapsedSeconds);
    bus.suppressedDeparture.reset();
    return true;
}

inline void beginScheduledBusRetirementRun(ScheduledBusState& bus) {
    bus.passengerStopArrived = false;
    bus.passengerStopDepartureRequested = false;
    bus.retirementDrivingToTerminus = true;
    bus.nextPassengerStopIndex = bus.passengerStops.size();
}

inline std::optional<double> scheduledBusRetirementTerminus(const ScheduledBusState& bus) {
    if (!bus.retirementDrivingToTerminus || bus.passengerStops.empty()) {
        return std::nullopt;
    }
    return bus.passengerStops[bus.passengerStops.size() - 1].routeDistanceMeters;
}

inline bool scheduledBusReachedRetirementTerminus(const ScheduledBusState& bus,
                                                  double distanceMeters,
                                                  double speedMetersPerSecond,
                                                  double stoppedSpeedMetersPerSecond) {
    const auto terminus = scheduledBusRetirementTerminus(bus);
    return terminus && distanceMeters + 0.05 >= *terminus &&
           speedMetersPerSecond <= stoppedSpeedMetersPerSecond;
}

inline void finishScheduledBusTrip(ScheduledBusState& bus, double simulationTimeSeconds) {
    bus.scheduledTripActive = false;
    bus.scheduledServiceCancelled = false;
    bus.retirementDrivingToTerminus = false;
    if (bus.suppressedDeparture &&
        bus.suppressedDeparture->departureElapsedSeconds <= simulationTimeSeconds) {
        bus.suppressedDeparture.reset();
    }
}

} // namespace openbus::app
