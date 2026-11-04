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
    std::vector<openbus::map::MapAiTimetableProfile> timetableProfiles;

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
std::vector<std::string>
scheduledPlayerTimetableKeys(const openbus::map::MapAiTrafficData& traffic);

struct PassengerRouteRegistrationReport {
    std::size_t missingStops = 0;
    std::size_t routesWithMissingStops = 0;
    std::size_t routesWithoutStops = 0;
    std::size_t routesMissingStopDefinitions = 0;
    std::string missingExamples;
    std::string routesWithoutStopExamples;
    std::string missingStopDefinitionExamples;
};

struct NearbyPassengerStop {
    std::size_t routeIndex = 0;
    std::size_t stopIndex = 0;
    double distanceMeters = std::numeric_limits<double>::infinity();
};

struct WaitingPassengerJourney {
    std::size_t routeIndex = 0;
    int destinationStopId = 0;
    std::array<double, 3> destinationPosition{};
    bool pedestrianResolved = false;
    std::optional<openbus::map::MapPassengerPedestrianJourney> pedestrianJourney;
    std::size_t originStopIndex = 0;
    std::size_t destinationStopIndex = 0;
};

struct WaitingPassenger {
    std::size_t modelIndex = 0;
    std::size_t queueSlot = 0;
    std::size_t arrivalOrder = 0;
    std::optional<WaitingPassengerJourney> journey;
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
    [[nodiscard]] std::optional<std::size_t>
    routeCallIndexForBusStop(std::size_t routeIndex, const PassengerStops& busStops,
                             std::size_t busStopIndex) const;
    void initializeQueues(std::span<const openbus::map::MapTrafficPassengerStop> stops);
    const std::vector<WaitingPassenger>& waitingPassengers(int stopId);
    bool passengerMatchesService(const WaitingPassenger& passenger, std::size_t routeIndex,
                                 const openbus::map::MapPassengerServiceRuntime& service,
                                 std::optional<std::size_t> currentStopIndex = std::nullopt) const;
    std::optional<std::size_t>
    nextPassenger(int stopId, std::size_t routeIndex,
                  const openbus::map::MapPassengerServiceRuntime& service,
                  std::optional<std::size_t> currentStopIndex = std::nullopt);
    void resolvePassengerPedestrianJourney(
        int stopId, std::size_t passengerIndex,
        std::optional<openbus::map::MapPassengerPedestrianJourney> pedestrianJourney);
    void advanceService(openbus::map::MapPassengerServiceRuntime& service, std::size_t routeIndex,
                        std::optional<std::size_t> passengerIndex, double elapsedSeconds,
                        bool allowBoarding, bool allowAlighting, bool holdAtStop,
                        bool pauseDwellTimer = false,
                        std::optional<std::size_t> currentStopIndex = std::nullopt);
    void departService(openbus::map::MapPassengerServiceRuntime& service);
    void endService(openbus::map::MapPassengerServiceRuntime& service);
    [[nodiscard]] NearbyPassengerStop
    findNearestStop(const std::array<double, 3>& position, double headingRadians,
                    std::optional<std::size_t> routeIndex = std::nullopt,
                    std::size_t firstStopIndex = 0,
                    const std::function<std::optional<std::array<double, 3>>(
                        const openbus::map::MapTrafficPassengerStop&)>& resolvePosition = {}) const;

  private:
    struct QueueState {
        std::vector<WaitingPassenger> passengers;
        std::size_t capacity = 0;
        std::size_t nextArrivalOrder = 0;
        std::size_t nextModelIndex = 0;
        bool initialized = false;
        std::optional<std::size_t> candidateRouteCount;
        std::vector<WaitingPassengerJourney> candidates;
    };
    struct Reservation {
        int stopId = 0;
        WaitingPassenger passenger;
    };
    std::unordered_map<int, QueueState> queueStates_;
    std::unordered_map<const openbus::map::MapPassengerServiceRuntime*, Reservation> reservations_;
    void syncQueue(int stopId);
    void restoreReservation(openbus::map::MapPassengerServiceRuntime& service, bool endService);
};

std::optional<std::size_t> nextLocatedPassengerStop(const PassengerStopRouteState& route,
                                                    std::size_t firstStopIndex);
std::optional<std::size_t> nextPassengerStopWithPosition(
    const PassengerStopRouteState& route, std::size_t firstStopIndex,
    const std::function<std::optional<std::array<double, 3>>(
        const openbus::map::MapTrafficPassengerStop&)>& resolveEditorPosition);

} // namespace openbus::app
