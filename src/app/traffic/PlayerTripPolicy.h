#pragma once

#include "app/traffic/ScheduledBusState.h"

#include <cstdint>
#include <functional>
#include <span>

namespace openbus::app {

struct PlayerTripPolicyTarget {
    std::reference_wrapper<ScheduledBusState> bus;
    std::reference_wrapper<std::uint64_t> generation;
};

void clearPlayerTripPolicy(std::span<const PlayerTripPolicyTarget> targets);
void applyPlayerTripPolicy(std::span<const PlayerTripPolicyTarget> targets,
                           const openbus::map::MapAiTimetableCourseTrip& trip, bool additionalBus);

} // namespace openbus::app
