#pragma once

#include "map/MapTrafficRuntime.h"
#include "map/MapConfigLoader.h"

#include <cstddef>
#include <memory>
#include <optional>
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

    struct PopulationCandidate {
        std::size_t stateIndex = 0;
        std::uint64_t generation = 0;
        std::optional<std::size_t> groupIndex;
        bool isHuman = false;
        bool isBus = false;
        bool active = false;
        bool canSpawn = false;
        bool canDespawn = false;
    };

    struct PopulationDecision {
        std::size_t stateIndex = 0;
        std::uint64_t generation = 0;
        bool isHuman = false;
        bool spawn = false;
        bool despawn = false;
    };

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
                         bool useGroupPopulation);
    double averageTickProcessingSeconds() const;
    std::optional<UpdateResults> takeLatestResults();
    std::optional<std::vector<PopulationDecision>> takePopulationDecisions();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::app
