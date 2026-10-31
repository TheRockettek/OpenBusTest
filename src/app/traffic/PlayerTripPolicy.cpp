#include "app/traffic/PlayerTripPolicy.h"

#include <limits>

namespace openbus::app {

void clearPlayerTripPolicy(std::span<const PlayerTripPolicyTarget> targets) {
    for (const PlayerTripPolicyTarget& target : targets) {
        ScheduledBusState& bus = target.bus.get();
        if (bus.suppressedDeparture) {
            bus.suppressedDeparture.reset();
            ++target.generation.get();
        }
    }
}

void applyPlayerTripPolicy(std::span<const PlayerTripPolicyTarget> targets,
                           const openbus::map::MapAiTimetableCourseTrip& trip, bool additionalBus) {
    if (additionalBus) {
        return;
    }
    const auto matches = [&](const ScheduledBusState& bus) {
        return openbus::map::mapTrafficPlayerTripMatchesScheduledRoute(
            trip.timetableKey, bus.timetableKey, additionalBus);
    };
    bool activeMatch = false;
    for (const PlayerTripPolicyTarget& target : targets) {
        ScheduledBusState& bus = target.bus.get();
        if (bus.scheduledTripActive && matches(bus)) {
            activeMatch = true;
            if (!bus.scheduledServiceCancelled) {
                bus.scheduledServiceCancelled = true;
                ++target.generation.get();
            }
        }
    }
    if (activeMatch) {
        return;
    }
    const PlayerTripPolicyTarget* selectedTarget = nullptr;
    double selectedTime = std::numeric_limits<double>::infinity();
    for (const PlayerTripPolicyTarget& target : targets) {
        ScheduledBusState& bus = target.bus.get();
        if (!bus.scheduledTripActive && matches(bus) &&
            bus.nextScheduledDepartureIndex < bus.scheduledDepartureMinutes.size() &&
            bus.nextScheduledDepartureElapsedSeconds < selectedTime) {
            selectedTarget = &target;
            selectedTime = bus.nextScheduledDepartureElapsedSeconds;
        }
    }
    if (selectedTarget != nullptr) {
        selectedTarget->bus.get().suppressedDeparture = ScheduledDepartureSuppression{
            selectedTarget->bus.get().nextScheduledDepartureIndex, selectedTime};
        ++selectedTarget->generation.get();
    }
}

} // namespace openbus::app
