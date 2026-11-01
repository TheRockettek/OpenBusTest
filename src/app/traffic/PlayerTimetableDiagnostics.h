#pragma once

#include "app/traffic/PlayerTimetableController.h"

#include <string>
#include <vector>

namespace openbus::app {

void logPlayerTimetableCatalog(const PlayerTimetableCatalog& catalog,
                               const std::vector<PassengerStopRouteState>& routes,
                               const std::vector<openbus::map::MapAiTimetableCourse>& courses,
                               const std::vector<std::string>& mapKeys, const std::string& mapPath,
                               const std::string& hofPath, const openbus::config::HofData& hof,
                               const std::vector<std::string>& scriptErrors);

} // namespace openbus::app
