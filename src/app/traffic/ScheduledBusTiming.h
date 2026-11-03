#pragma once

#include "app/traffic/ScheduledBusState.h"
#include "app/traffic/PassengerNetwork.h"

#include <array>
#include <optional>

namespace openbus::app {

const openbus::map::MapAiTimetableProfile*
scheduledBusProfileForDeparture(const ScheduledBusState& bus, std::size_t departureIndex);

double scheduledBusDelaySeconds(const ScheduledBusState& bus,
                                const openbus::map::MapTrafficActor& actor,
                                double simulationTimeSeconds);

std::optional<double> playerTimetableDelaySeconds(
    const PassengerStopRouteState& route, const openbus::map::MapAiTimetableProfile& profile,
    double departureMinutes, double currentTimeOfDaySeconds, std::size_t firstStopIndex,
    std::size_t nextStopIndex, const std::array<double, 3>& vehiclePosition,
    std::optional<std::size_t> dwellingStopIndex);

} // namespace openbus::app
