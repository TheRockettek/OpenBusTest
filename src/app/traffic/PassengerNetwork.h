#pragma once

#include "app/traffic/PassengerStops.h"
#include "map/MapConfigLoader.h"
#include "map/MapTrafficRuntime.h"

#include <array>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

namespace openbus::app {

using PlayerStopPoseResolver = std::function<std::optional<openbus::map::MapTrafficPose>(
    const openbus::map::MapTrafficPassengerStop&)>;

struct PassengerStopRouteState {
    std::shared_ptr<const openbus::map::MapTrafficActor> actor;
    PassengerStops stops;
    std::string timetableKey;
    std::string lineName;
    std::string destinationName;
    std::string tripName;
    std::vector<std::optional<openbus::map::MapTrafficPose>> stopPoses = {};

    [[nodiscard]] std::optional<openbus::map::MapTrafficPose> stopPose(std::size_t index) const {
        if (!stopPoses.empty()) {
            return stopPoses.at(index);
        }
        if (!actor) {
            return std::nullopt;
        }
        return openbus::map::sampleMapTrafficRoute(actor->route, stops[index].routeDistanceMeters,
                                                   0.0);
    }
};

PassengerStopRouteState makePlayerTimetableRoute(const openbus::map::MapAiTimetableRoute& route,
                                                 std::size_t waitingPassengers,
                                                 const PlayerStopPoseResolver& resolvePose);

struct PassengerRouteRegistrationReport {
    std::size_t missingStops = 0;
    std::size_t routesWithMissingStops = 0;
    std::size_t routesWithoutStops = 0;
    std::string missingExamples;
    std::string routesWithoutStopExamples;
};

struct NearbyPassengerStop {
    std::size_t routeIndex = 0;
    std::size_t stopIndex = 0;
    double distanceMeters = std::numeric_limits<double>::infinity();
};

PassengerStops::Container mapPassengerStopsToTrafficRoute(
    const openbus::map::MapTrafficRoute& route, PassengerStops::Container stops,
    const std::function<void(const openbus::map::MapTrafficPassengerStop&)>& onUnmatched);

struct PassengerNetwork {
    std::vector<PassengerStopRouteState> routes;
    std::unordered_map<int, std::size_t> queues;
    std::unordered_map<int, std::size_t> initialQueues;
    std::size_t updateOffset = 0;

    PassengerRouteRegistrationReport
    registerTimetables(const openbus::map::MapAiTrafficData& traffic, std::size_t waitingPassengers,
                       const PlayerStopPoseResolver& resolvePose);
    [[nodiscard]] std::optional<std::size_t> findRouteIndex(const std::string& key) const;
    void initializeQueues(std::span<const openbus::map::MapTrafficPassengerStop> stops);
    [[nodiscard]] NearbyPassengerStop
    findNearestStop(const std::array<double, 3>& position, double headingRadians,
                    std::optional<std::size_t> routeIndex = std::nullopt,
                    std::size_t firstStopIndex = 0) const;
};

std::optional<std::size_t> nextLocatedPassengerStop(const PassengerStopRouteState& route,
                                                    std::size_t firstStopIndex);

} // namespace openbus::app
