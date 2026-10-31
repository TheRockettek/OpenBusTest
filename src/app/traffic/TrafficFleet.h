#pragma once

#include "app/traffic/ScheduledBusState.h"
#include "map/MapTrafficRuntime.h"

#include <cstddef>
#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

struct Vehicle;

namespace openbus::app {

class MapTrafficWorker;

struct TrafficVehicleState {
    Vehicle* vehicle = nullptr;
    std::future<std::unique_ptr<Vehicle>> pendingVehicle;
    std::unique_ptr<Vehicle> pendingInitializationVehicle;
    openbus::map::MapTrafficActor actor;
    std::shared_ptr<const openbus::map::MapTrafficActor> workerActorTemplate;
    std::optional<std::size_t> blockingActorStateIndex;
    std::optional<std::uint64_t> blockingActorGeneration;
    openbus::map::MapTrafficBlockReason debugDisplayedBlockReason =
        openbus::map::MapTrafficBlockReason::None;
    openbus::map::MapTrafficBlockReason debugCandidateBlockReason =
        openbus::map::MapTrafficBlockReason::None;
    double debugCandidateBlockSeconds = 0.0;
    double debugBlockReasonHoldSeconds = 0.0;
    std::optional<openbus::map::MapTrafficPose> cachedPose;
    double cachedPoseDistanceMeters = 0.0;
    double cachedPoseLateralOffsetMeters = 0.0;
    double cachedTravelHeadingRadians = 0.0;
    std::uint64_t cachedPoseGeneration = 0;
    std::shared_ptr<const std::vector<openbus::map::MapTrafficSpawnCandidate>> spawnCandidates;
    std::unique_ptr<ScheduledBusState> bus;
    bool populationEnabled = false;
    bool vehicleLoadFailed = false;
    bool reducedTrafficDetail = false;
    bool deadlocked = false;
    bool deadlockRemovalCommitted = false;
    std::optional<std::size_t> activeSpawnCandidateIndex;
    std::vector<std::size_t> rejectedSpawnCandidateIndices;
    double spawnedAtDistanceMeters = 0.0;
    double wheelTravelDistanceMeters = 0.0;
    double previousWheelRouteDistanceMeters = 0.0;
    bool wheelRouteDistanceInitialized = false;
    bool movedSinceSpawn = false;
    double lastProgressDistanceMeters = 0.0;
    double stoppedSecondsSinceProgress = 0.0;
    bool stuckDiagnosticLogged = false;
    std::uint64_t workerGeneration = 0;
    std::optional<std::uint64_t> inactiveResetGeneration;

    [[nodiscard]] bool isBus() const noexcept {
        return bus != nullptr;
    }
};

struct TrafficFleet {
    std::vector<TrafficVehicleState> vehicles;
    std::unordered_map<const openbus::map::MapTrafficActor*, const TrafficVehicleState*> byActor;
    const TrafficVehicleState* indexedVehicleData = nullptr;
    std::size_t indexedVehicleCount = 0;
    std::size_t targetRoadTrafficCount = 0;
    std::vector<openbus::map::MapAiRoadTrafficGroupPopulation> targetRoadGroupPopulations;
    bool useGroupRoadPopulation = false;
    std::size_t roadRouteRefreshCursor = 0;
    std::size_t roadRouteRefreshSeed = 0;
    double roadRouteRefreshElapsedSeconds = 0.0;
    std::size_t targetBusTrafficCount = 0;
    double mapTrafficStartHour = 12.0;
    bool mapTrafficSpawnInView = false;
    bool trafficSpawnDiagnosticsEnabled = false;
    double trafficSpawnDiagnosticElapsedSeconds = 0.0;
    std::size_t roadTrafficRecycledOutsideTiles = 0;
    std::size_t roadRouteRefreshAttempts = 0;
    std::size_t roadRouteRefreshSuccesses = 0;
    double aiSimulationRateHz = 0.0;
    double aiMaximumRateHz = 0.0;
    double aiRateDivisor = 1.0;
    double trafficWorkerInputAccumulatorSeconds = 0.0;
    double currentAiSimulationRateHz = 0.0;
    double configuredAiSimulationRateHz = 0.0;
    double currentAiWorkerTickMilliseconds = 0.0;
    std::unique_ptr<MapTrafficWorker> worker;
    bool populationPlanPending = false;
};

} // namespace openbus::app
