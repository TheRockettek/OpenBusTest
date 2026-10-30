#pragma once

#include "map/MapTrafficRuntime.h"
#include "map/MapConfigLoader.h"

#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

namespace openbus::app {

class MapTrafficWorker {
  public:
    struct ActorMotion {
        double distanceMeters = 0.0;
        double speedMetersPerSecond = 0.0;
        double targetSpeedMetersPerSecond = 0.0;
        double lateralOffsetMeters = 0.0;
        double junctionWaitSeconds = 0.0;
        std::optional<double> scheduledStopDistanceMeters;
        openbus::map::MapTrafficBlockReason blockReason = openbus::map::MapTrafficBlockReason::None;
        std::optional<std::size_t> blockingObstacleIndex;
        std::optional<double> junctionClaimEndMeters;
        std::optional<std::size_t> committedSignalSegmentIndex;
    };

    struct ActorInput {
        std::size_t stateIndex = 0;
        std::uint64_t generation = 0;
        std::shared_ptr<const openbus::map::MapTrafficActor> actorTemplate;
        ActorMotion motion;
        std::optional<std::size_t> blockingActorStateIndex;
        std::optional<std::uint64_t> blockingActorGeneration;
        bool reducedSpeedLookahead = false;
    };

    struct ActorResult {
        std::size_t stateIndex = 0;
        std::uint64_t generation = 0;
        ActorMotion motion;
        std::optional<std::size_t> blockingActorStateIndex;
        std::optional<std::uint64_t> blockingActorGeneration;
        openbus::map::MapTrafficPose pose;
        double travelHeadingRadians = 0.0;
    };

    struct HumanInput {
        std::size_t stateIndex = 0;
        std::uint64_t generation = 0;
        std::shared_ptr<const openbus::map::MapTrafficActor> actorTemplate;
        double distanceMeters = 0.0;
        double speedMetersPerSecond = 0.0;
        double feetDistanceMeters = 0.2;
        bool populationEnabled = true;
        bool advanceWhileInactive = false;
    };

    struct HumanResult {
        std::size_t stateIndex = 0;
        std::uint64_t generation = 0;
        double distanceMeters = 0.0;
        double speedMetersPerSecond = 0.0;
    };

    struct UpdateResults {
        std::vector<ActorResult> actors;
        std::vector<HumanResult> humans;
        double elapsedSeconds = 0.0;
    };

    struct PopulationSearchContext {
        std::vector<std::pair<int, int>> activeTiles;
        double tileSize = openbus::map::OMSI_TILE_SIZE_METERS;
        bool cameraValid = false;
        bool spawnInView = false;
        openbus::map::MapTrafficCameraView mainCamera;
        std::vector<openbus::map::MapTrafficCameraView> reflectionCameras;
        std::vector<openbus::map::MapTrafficObstacle> occupied;
        bool hasPlayerPose = false;
        openbus::map::MapTrafficPose playerPose;
        double predictedForwardDistanceMeters = 0.0;
        bool playerMoving = false;
        double playerLengthMeters = 0.0;
        double playerWidthMeters = 0.0;
        bool diagnostics = false;
    };

    struct PopulationSpawnSearch {
        std::shared_ptr<const openbus::map::MapTrafficActor> actor;
        std::shared_ptr<const std::vector<openbus::map::MapTrafficSpawnCandidate>> candidates;
        std::vector<std::size_t> rejectedCandidates;
        double lateralOffsetMeters = 0.0;
    };

    struct PopulationCandidate {
        std::size_t stateIndex = 0;
        std::uint64_t generation = 0;
        std::optional<std::size_t> groupIndex;
        bool isHuman = false;
        bool isBus = false;
        bool active = false;
        bool canSpawn = false;
        bool canDespawn = false;
        double spawnPriorityMeters = std::numeric_limits<double>::infinity();
        std::optional<PopulationSpawnSearch> spawnSearch = std::nullopt;
    };

    struct PopulationSpawn {
        double distanceMeters = 0.0;
        openbus::map::MapTrafficPose pose;
        std::size_t candidateIndex = 0;
        double priorityMeters = 0.0;
    };

    struct PopulationSearchDiagnostics {
        std::size_t vehiclesChecked = 0;
        std::size_t candidatesChecked = 0;
        std::size_t rejectedCandidates = 0;
        std::size_t inactiveCandidateTiles = 0;
        std::size_t posesOutsideActiveTiles = 0;
        std::size_t visibleCandidates = 0;
        std::size_t obstructedCandidates = 0;
        std::size_t clearCandidates = 0;
        std::size_t frontZoneRejectedCandidates = 0;
        std::size_t lowerPriorityCandidates = 0;
        std::optional<std::pair<int, int>> firstInactiveCandidateTile;
    };

    struct BusSearchDiagnostics {
        openbus::map::MapTrafficPose pose;
        bool hidden = false;
        bool clear = false;
        bool onActiveTile = false;
        bool frontBlocked = false;
    };

    struct PopulationDecision {
        std::size_t stateIndex = 0;
        std::uint64_t generation = 0;
        bool isHuman = false;
        bool spawn = false;
        bool despawn = false;
        bool searchedSpawn = false;
        std::optional<PopulationSpawn> spawnLocation = std::nullopt;
        PopulationSearchDiagnostics roadSearchDiagnostics = {};
        std::optional<BusSearchDiagnostics> busSearchDiagnostics = std::nullopt;
    };

    class PopulationBalance {
      public:
        PopulationBalance(
            const std::vector<PopulationCandidate>& candidates, std::size_t roadTarget,
            std::size_t busTarget, std::size_t humanTarget,
            const std::vector<openbus::map::MapAiRoadTrafficGroupPopulation>& groupTargets,
            bool useGroupPopulation);

        std::size_t targetFor(const PopulationCandidate& candidate) const;
        std::size_t countFor(const PopulationCandidate& candidate) const;
        std::size_t& countFor(const PopulationCandidate& candidate);
        bool needsSpawn(const PopulationCandidate& candidate) const;
        bool needsDespawn(const PopulationCandidate& candidate) const;

      private:
        std::size_t roadTarget_;
        std::size_t busTarget_;
        std::size_t humanTarget_;
        bool useGroupPopulation_;
        std::size_t activeRoadCount_ = 0;
        std::size_t activeBusCount_ = 0;
        std::size_t activeHumanCount_ = 0;
        std::unordered_map<std::size_t, std::size_t> activeCountsByGroup_;
        std::unordered_map<std::size_t, std::size_t> groupTargets_;
    };

    static std::optional<PopulationSpawn> revalidatePopulationSpawn(
        const PopulationCandidate& current, const PopulationBalance& balance,
        const PopulationDecision& decision, const openbus::map::MapTrafficActor& actor,
        const std::vector<openbus::map::MapTrafficSpawnCandidate>& candidates,
        const std::vector<std::size_t>& rejectedCandidates, const PopulationSearchContext& context,
        const std::vector<openbus::map::MapTrafficObstacle>& occupied, bool scheduledDepartureDue);

    MapTrafficWorker();
    ~MapTrafficWorker();

    MapTrafficWorker(const MapTrafficWorker&) = delete;
    MapTrafficWorker& operator=(const MapTrafficWorker&) = delete;

    void setSimulationRateHz(double rateHz);
    void
    submit(std::vector<ActorInput> actors, std::vector<HumanInput> humans,
           std::vector<openbus::map::MapTrafficObstacle> obstacles,
           std::vector<openbus::map::MapTrafficObstacle> humanObstacles,
           std::vector<std::shared_ptr<openbus::map::MapTrafficSignalController>> signalControllers,
           std::vector<openbus::map::MapTrafficSignalBusPriorityRequest> busPriorityRequests,
           double elapsedSeconds, double simulationTimeSeconds);
    void
    submitPopulationPlan(std::vector<PopulationCandidate> candidates, std::size_t roadTarget,
                         std::size_t busTarget, std::size_t humanTarget,
                         std::vector<openbus::map::MapAiRoadTrafficGroupPopulation> groupTargets,
                         bool useGroupPopulation,
                         std::optional<PopulationSearchContext> searchContext = std::nullopt);
    double averageTickProcessingSeconds() const;
    std::optional<UpdateResults> takeLatestResults();
    std::optional<std::vector<PopulationDecision>> takePopulationDecisions();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::app
