#pragma once

#include "app/traffic/PassengerStops.h"
#include "map/MapTrafficRuntime.h"

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace openbus::app {

struct PassengerStopRouteState {
    std::shared_ptr<const openbus::map::MapTrafficActor> actor;
    PassengerStops stops;
    std::string timetableKey;
    std::string lineName;
    std::string destinationName;
    std::string tripName;
};

struct PassengerNetwork {
    std::vector<PassengerStopRouteState> routes;
    std::unordered_map<int, std::size_t> queues;
    std::unordered_map<int, std::size_t> initialQueues;
    std::size_t updateOffset = 0;
};

} // namespace openbus::app
