#pragma once

#include "app/traffic/PassengerNetwork.h"
#include "map/MapConfigLoader.h"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace openbus::app {

struct PlayerTimetableSelection {
    std::size_t passengerRouteIndex = 0;
    std::size_t courseIndex = 0;
    std::size_t courseTripIndex = 0;
    std::size_t firstStopIndex = 0;
    bool driveAsAdditionalBus = false;
};

struct PlayerTimetableState {
    PlayerTimetableSelection selection;
    std::size_t nextStopIndex = 0;
    std::string lineName;
    bool stopAfterCurrentTrip = false;
};

class PlayerTimetableController {
  public:
    std::optional<PlayerTimetableState> state;

    const openbus::map::MapAiTimetableCourseTrip*
    select(std::optional<PlayerTimetableSelection> selection,
           const std::vector<PassengerStopRouteState>& routes,
           const std::vector<openbus::map::MapAiTimetableCourse>& courses);

    const openbus::map::MapAiTimetableCourseTrip*
    advance(const std::vector<PassengerStopRouteState>& routes,
            const std::vector<openbus::map::MapAiTimetableCourse>& courses);
};

} // namespace openbus::app
