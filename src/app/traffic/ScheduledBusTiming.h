#pragma once

#include "app/traffic/ScheduledBusState.h"

namespace openbus::app {

const openbus::map::MapAiTimetableProfile*
scheduledBusProfileForDeparture(const ScheduledBusState& bus, std::size_t departureIndex);

double scheduledBusDelaySeconds(const ScheduledBusState& bus,
                                const openbus::map::MapTrafficActor& actor,
                                double simulationTimeSeconds);

} // namespace openbus::app
