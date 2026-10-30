#include "app/MapTrafficWorker.h"

#include "core/PerfTrace.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

namespace openbus::app {

namespace {

constexpr double kDefaultTrafficSimulationRateHz = 60.0;

class SpawnCrowdingIndex {
  public:
    explicit SpawnCrowdingIndex(const std::vector<openbus::map::MapTrafficObstacle>& obstacles) {
        for (const auto& obstacle : obstacles) {
            add(obstacle);
        }
    }

    void add(const openbus::map::MapTrafficObstacle& obstacle) {
        cells_[cell(obstacle.pose)].push_back(obstacle.pose.position);
    }

    double penalty(const openbus::map::MapTrafficPose& pose) const {
        const auto [cellX, cellY] = cell(pose);
        double result = 0.0;
        for (int x = -1; x <= 1; ++x) {
            for (int y = -1; y <= 1; ++y) {
                const auto found = cells_.find({cellX + x, cellY + y});
                if (found == cells_.end()) {
                    continue;
                }
                for (const auto& position : found->second) {
                    if (std::abs(position[2] - pose.position[2]) > 4.0) {
                        continue;
                    }
                    const double distance =
                        std::hypot(position[0] - pose.position[0], position[1] - pose.position[1]);
                    result += 25.0 * std::max(0.0, 1.0 - distance / cellSizeMeters);
                }
            }
        }
        return std::min(result, 60.0);
    }

  private:
    static constexpr double cellSizeMeters = 50.0;

    static std::pair<int, int> cell(const openbus::map::MapTrafficPose& pose) {
        return {static_cast<int>(std::floor(pose.position[0] / cellSizeMeters)),
                static_cast<int>(std::floor(pose.position[1] / cellSizeMeters))};
    }

    std::map<std::pair<int, int>, std::vector<std::array<double, 3>>> cells_;
};

std::optional<MapTrafficWorker::PopulationSpawn>
findPopulationSpawn(const MapTrafficWorker::PopulationCandidate& candidate,
                    const MapTrafficWorker::PopulationSearchContext& context,
                    const std::set<std::pair<int, int>>& activeTiles,
                    const SpawnCrowdingIndex& crowding,
                    MapTrafficWorker::PopulationDecision& decision) {
    using namespace openbus::map;
    const auto& search = *candidate.spawnSearch;
    if (!search.actor || (!candidate.isBus && !search.candidates)) {
        throw std::invalid_argument("Population search is missing immutable route data");
    }
    const MapTrafficActor& actor = *search.actor;
    auto& diagnostics = decision.roadSearchDiagnostics;
    openbus::rendering::TraceScope trace("traffic", "MapTrafficWorker::populationSpawnSearch");
    if (candidate.isBus) {
        const auto pose = sampleMapTrafficActorPose(actor.route, 0.0, search.lateralOffsetMeters,
                                                    actor.rearAxleOffsetMeters);
        if (!pose) {
            throw std::runtime_error("Could not sample a scheduled traffic spawn position");
        }
        MapTrafficWorker::BusSearchDiagnostics busDiagnostics;
        busDiagnostics.pose = *pose;
        const double radius = std::hypot(actor.lengthMeters, actor.widthMeters) * 0.5 + 2.0;
        busDiagnostics.hidden =
            context.spawnInView ||
            (context.cameraValid &&
             mapTrafficEntryHiddenFromCameras(pose->position, radius, context.mainCamera,
                                              context.reflectionCameras, 50.0));
        busDiagnostics.frontBlocked =
            context.playerMoving && mapTrafficSpawnInsideForwardExclusion(
                                        context.playerPose, context.playerLengthMeters,
                                        context.playerWidthMeters, *pose, actor.widthMeters, 30.0);
        busDiagnostics.clear =
            !busDiagnostics.frontBlocked &&
            mapTrafficHasClearance({*pose, actor.lengthMeters, actor.widthMeters}, context.occupied,
                                   2.0);
        busDiagnostics.onActiveTile =
            mapTrafficPoseOnActiveTile(*pose, context.activeTiles, context.tileSize);
        if (!busDiagnostics.hidden || !busDiagnostics.clear || !busDiagnostics.onActiveTile) {
            decision.busSearchDiagnostics = std::move(busDiagnostics);
            return std::nullopt;
        }
        const double priority =
            context.hasPlayerPose
                ? mapTrafficSpawnPriorityDistanceMeters(context.playerPose, *pose,
                                                        context.predictedForwardDistanceMeters)
                : 0.0;
        return MapTrafficWorker::PopulationSpawn{0.0, *pose, 0, priority};
    }
    if (context.diagnostics) {
        ++diagnostics.vehiclesChecked;
    }
    std::optional<MapTrafficWorker::PopulationSpawn> best;
    const std::size_t count = search.candidates->size();
    for (std::size_t index = 0; index < count; ++index) {
        if (std::ranges::find(search.rejectedCandidates, index) !=
            search.rejectedCandidates.end()) {
            if (context.diagnostics) {
                ++diagnostics.rejectedCandidates;
            }
            continue;
        }
        const auto& spawn = (*search.candidates)[index];
        const double distance = spawn.distanceMeters;
        if (context.diagnostics) {
            ++diagnostics.candidatesChecked;
        }
        if (!activeTiles.contains({spawn.tileX, spawn.tileY})) {
            if (context.diagnostics) {
                ++diagnostics.inactiveCandidateTiles;
                if (!diagnostics.firstInactiveCandidateTile) {
                    diagnostics.firstInactiveCandidateTile = {spawn.tileX, spawn.tileY};
                }
            }
            continue;
        }
        const auto pose = sampleMapTrafficActorPose(
            actor.route, distance, search.lateralOffsetMeters, actor.rearAxleOffsetMeters);
        if (!pose) {
            throw std::runtime_error("Could not sample a traffic spawn position");
        }
        if (!mapTrafficPoseOnActiveTile(*pose, context.activeTiles, context.tileSize)) {
            if (context.diagnostics) {
                ++diagnostics.posesOutsideActiveTiles;
            }
            continue;
        }
        if (context.playerMoving &&
            mapTrafficSpawnInsideForwardExclusion(context.playerPose, context.playerLengthMeters,
                                                  context.playerWidthMeters, *pose,
                                                  actor.widthMeters, 30.0)) {
            if (context.diagnostics) {
                ++diagnostics.frontZoneRejectedCandidates;
            }
            continue;
        }
        const double proximity =
            context.hasPlayerPose
                ? mapTrafficSpawnPriorityDistanceMeters(context.playerPose, *pose,
                                                        context.predictedForwardDistanceMeters)
                : 0.0;
        const double priority = proximity + spawn.locationPenaltyMeters -
                                spawn.entryPreferenceMeters + crowding.penalty(*pose);
        if (best && !(priority < best->priorityMeters)) {
            if (context.diagnostics) {
                ++diagnostics.lowerPriorityCandidates;
            }
            continue;
        }
        const double radius = std::hypot(actor.lengthMeters, actor.widthMeters) * 0.5 + 2.0;
        if (!context.spawnInView &&
            (!context.cameraValid ||
             !mapTrafficEntryHiddenFromCameras(pose->position, radius, context.mainCamera,
                                               context.reflectionCameras, 50.0))) {
            if (context.diagnostics) {
                ++diagnostics.visibleCandidates;
            }
            continue;
        }
        if (!mapTrafficHasClearance({*pose, actor.lengthMeters, actor.widthMeters},
                                    context.occupied, 3.0)) {
            if (context.diagnostics) {
                ++diagnostics.obstructedCandidates;
            }
            continue;
        }
        if (context.diagnostics) {
            ++diagnostics.clearCandidates;
        }
        best = MapTrafficWorker::PopulationSpawn{distance, *pose, index, priority};
    }
    return best;
}

void applyMotion(openbus::map::MapTrafficActor& actor,
                 const MapTrafficWorker::ActorMotion& motion) {
    actor.distanceMeters = motion.distanceMeters;
    actor.speedMetersPerSecond = motion.speedMetersPerSecond;
    actor.targetSpeedMetersPerSecond = motion.targetSpeedMetersPerSecond;
    actor.lateralOffsetMeters = motion.lateralOffsetMeters;
    actor.junctionWaitSeconds = motion.junctionWaitSeconds;
    actor.scheduledStopDistanceMeters = motion.scheduledStopDistanceMeters;
    actor.blockReason = motion.blockReason;
    actor.blockingObstacleIndex = motion.blockingObstacleIndex;
    actor.junctionClaimEndMeters = motion.junctionClaimEndMeters;
    actor.committedSignalSegmentIndex = motion.committedSignalSegmentIndex;
}

MapTrafficWorker::ActorMotion captureMotion(const openbus::map::MapTrafficActor& actor) {
    return {actor.distanceMeters,
            actor.speedMetersPerSecond,
            actor.targetSpeedMetersPerSecond,
            actor.lateralOffsetMeters,
            actor.junctionWaitSeconds,
            actor.scheduledStopDistanceMeters,
            actor.blockReason,
            actor.blockingObstacleIndex,
            actor.junctionClaimEndMeters,
            actor.committedSignalSegmentIndex};
}

void preserveMovementState(openbus::map::MapTrafficActor& destination,
                           const openbus::map::MapTrafficActor& source) {
    destination.distanceMeters = source.distanceMeters;
    destination.speedMetersPerSecond = source.speedMetersPerSecond;
    destination.targetSpeedMetersPerSecond = source.targetSpeedMetersPerSecond;
    destination.junctionWaitSeconds = source.junctionWaitSeconds;
    destination.blockReason = source.blockReason;
    destination.blockingObstacleIndex = source.blockingObstacleIndex;
    destination.junctionClaimEndMeters = source.junctionClaimEndMeters;
    destination.committedSignalSegmentIndex = source.committedSignalSegmentIndex;
}

} // namespace

MapTrafficWorker::PopulationBalance::PopulationBalance(
    const std::vector<PopulationCandidate>& candidates, std::size_t roadTarget,
    std::size_t busTarget, std::size_t humanTarget,
    const std::vector<openbus::map::MapAiRoadTrafficGroupPopulation>& groupTargets,
    bool useGroupPopulation)
    : roadTarget_(roadTarget), busTarget_(busTarget), humanTarget_(humanTarget),
      useGroupPopulation_(useGroupPopulation) {
    if (useGroupPopulation_) {
        groupTargets_.reserve(groupTargets.size());
        for (const auto& group : groupTargets) {
            groupTargets_.emplace(group.groupIndex, group.populationCount);
        }
    }
    for (const auto& candidate : candidates) {
        if (candidate.active) {
            ++countFor(candidate);
        }
    }
}

std::size_t
MapTrafficWorker::PopulationBalance::targetFor(const PopulationCandidate& candidate) const {
    if (candidate.isHuman) {
        return humanTarget_;
    }
    if (candidate.isBus) {
        return busTarget_;
    }
    if (useGroupPopulation_) {
        if (!candidate.groupIndex) {
            return 0;
        }
        const auto target = groupTargets_.find(*candidate.groupIndex);
        return target == groupTargets_.end() ? 0 : target->second;
    }
    return roadTarget_;
}

std::size_t
MapTrafficWorker::PopulationBalance::countFor(const PopulationCandidate& candidate) const {
    if (candidate.isHuman) {
        return activeHumanCount_;
    }
    if (candidate.isBus) {
        return activeBusCount_;
    }
    if (useGroupPopulation_ && candidate.groupIndex) {
        const auto count = activeCountsByGroup_.find(*candidate.groupIndex);
        return count == activeCountsByGroup_.end() ? 0 : count->second;
    }
    return activeRoadCount_;
}

std::size_t& MapTrafficWorker::PopulationBalance::countFor(const PopulationCandidate& candidate) {
    if (candidate.isHuman) {
        return activeHumanCount_;
    }
    if (candidate.isBus) {
        return activeBusCount_;
    }
    if (useGroupPopulation_ && candidate.groupIndex) {
        return activeCountsByGroup_[*candidate.groupIndex];
    }
    return activeRoadCount_;
}

bool MapTrafficWorker::PopulationBalance::needsSpawn(const PopulationCandidate& candidate) const {
    return !candidate.active && countFor(candidate) < targetFor(candidate);
}

bool MapTrafficWorker::PopulationBalance::needsDespawn(const PopulationCandidate& candidate) const {
    return candidate.active && countFor(candidate) > targetFor(candidate);
}

std::optional<MapTrafficWorker::PopulationSpawn> MapTrafficWorker::revalidatePopulationSpawn(
    const PopulationCandidate& current, const PopulationBalance& balance,
    const PopulationDecision& decision, const openbus::map::MapTrafficActor& actor,
    const std::vector<openbus::map::MapTrafficSpawnCandidate>& candidates,
    const std::vector<std::size_t>& rejectedCandidates, const PopulationSearchContext& context,
    const std::vector<openbus::map::MapTrafficObstacle>& occupied, bool scheduledDepartureDue) {
    using namespace openbus::map;
    if (current.isHuman || decision.isHuman || current.stateIndex != decision.stateIndex ||
        current.generation != decision.generation || !decision.spawn || !decision.spawnLocation ||
        !balance.needsSpawn(current) || (current.isBus && !scheduledDepartureDue)) {
        return std::nullopt;
    }
    const std::size_t index = decision.spawnLocation->candidateIndex;
    double distance = 0.0;
    if (!current.isBus) {
        if (index >= candidates.size() ||
            std::ranges::find(rejectedCandidates, index) != rejectedCandidates.end()) {
            return std::nullopt;
        }
        const auto& candidate = candidates[index];
        if (std::ranges::find(context.activeTiles, std::pair{candidate.tileX, candidate.tileY}) ==
            context.activeTiles.end()) {
            return std::nullopt;
        }
        distance = candidate.distanceMeters;
    }
    const auto pose = sampleMapTrafficActorPose(actor, distance);
    if (!pose) {
        throw std::runtime_error("Could not sample a traffic spawn position during admission");
    }
    if (!mapTrafficPoseOnActiveTile(*pose, context.activeTiles, context.tileSize) ||
        (context.playerMoving && mapTrafficSpawnInsideForwardExclusion(
                                     context.playerPose, context.playerLengthMeters,
                                     context.playerWidthMeters, *pose, actor.widthMeters, 30.0))) {
        return std::nullopt;
    }
    const double radius = std::hypot(actor.lengthMeters, actor.widthMeters) * 0.5 + 2.0;
    if ((!context.spawnInView &&
         (!context.cameraValid ||
          !mapTrafficEntryHiddenFromCameras(pose->position, radius, context.mainCamera,
                                            context.reflectionCameras, 50.0))) ||
        !mapTrafficHasClearance({*pose, actor.lengthMeters, actor.widthMeters}, occupied,
                                current.isBus ? 2.0 : 3.0)) {
        return std::nullopt;
    }
    const double priority =
        context.hasPlayerPose
            ? mapTrafficSpawnPriorityDistanceMeters(context.playerPose, *pose,
                                                    context.predictedForwardDistanceMeters)
            : 0.0;
    return PopulationSpawn{distance, *pose, index, priority};
}

struct MapTrafficWorker::Impl {
    struct InputSnapshot {
        std::vector<ActorInput> actors;
        std::vector<HumanInput> humans;
        std::vector<openbus::map::MapTrafficObstacle> obstacles;
        std::vector<openbus::map::MapTrafficObstacle> humanObstacles;
        std::vector<std::shared_ptr<openbus::map::MapTrafficSignalController>> signalControllers;
        std::vector<openbus::map::MapTrafficSignalBusPriorityRequest> busPriorityRequests;
    };

    struct WorkingActor {
        std::size_t stateIndex = 0;
        std::uint64_t generation = 0;
        std::shared_ptr<const openbus::map::MapTrafficActor> actorTemplate;
        openbus::map::MapTrafficDrivingRouteMetadata routeMetadata;
        openbus::map::MapTrafficActor actor;
        std::optional<std::size_t> blockingActorStateIndex;
        std::optional<std::uint64_t> blockingActorGeneration;
    };

    struct WorkingHuman {
        std::size_t stateIndex = 0;
        std::uint64_t generation = 0;
        openbus::map::MapTrafficActor actor;
        double feetDistanceMeters = 0.2;
    };

    Impl() {
        worker = std::thread([this] { run(); });
    }

    ~Impl() {
        {
            std::lock_guard lock(mutex);
            stopping = true;
        }
        wake.notify_all();
        worker.join();
    }

    void run() {
        std::unique_lock lock(mutex);
        bool lastWasPopulation = false;
        double populationProcessingSeconds = 0.0;
        while (true) {
            wake.wait(lock, [this] {
                return stopping || hasPopulationRequest ||
                       (hasInput && pendingElapsedSeconds >= simulationStepSeconds);
            });
            if (stopping) {
                return;
            }
            if (hasPopulationRequest && (!lastWasPopulation || !hasInput ||
                                         pendingElapsedSeconds < simulationStepSeconds)) {
                auto candidates = std::move(populationCandidates);
                const std::size_t roadTarget = populationRoadTarget;
                const std::size_t busTarget = populationBusTarget;
                const std::size_t humanTarget = populationHumanTarget;
                auto groupTargets = std::move(populationGroupTargets);
                const bool useGroupPopulation = populationUseGroups;
                auto searchContext = std::move(populationSearchContext);
                hasPopulationRequest = false;
                lastWasPopulation = true;
                lock.unlock();
                const auto populationStart = std::chrono::steady_clock::now();
                std::vector<PopulationDecision> decisions;
                try {
                    openbus::rendering::TraceScope trace("traffic",
                                                         "MapTrafficWorker::populationPlan");
                    decisions.reserve(candidates.size());
                    PopulationBalance balance(candidates, roadTarget, busTarget, humanTarget,
                                              groupTargets, useGroupPopulation);
                    for (const PopulationCandidate& candidate : candidates) {
                        decisions.push_back({candidate.stateIndex, candidate.generation,
                                             candidate.isHuman, false, false});
                    }
                    std::set<std::pair<int, int>> activeTiles;
                    const std::vector<openbus::map::MapTrafficObstacle> noObstacles;
                    SpawnCrowdingIndex crowding(searchContext ? searchContext->occupied
                                                              : noObstacles);
                    if (searchContext) {
                        activeTiles.insert(searchContext->activeTiles.begin(),
                                           searchContext->activeTiles.end());
                        for (std::size_t index = 0; index < candidates.size(); ++index) {
                            PopulationCandidate& candidate = candidates[index];
                            if (candidate.spawnSearch && balance.needsSpawn(candidate)) {
                                decisions[index].searchedSpawn = true;
                                auto spawn =
                                    findPopulationSpawn(candidate, *searchContext, activeTiles,
                                                        crowding, decisions[index]);
                                candidate.canSpawn = spawn.has_value();
                                if (spawn) {
                                    candidate.spawnPriorityMeters = spawn->priorityMeters;
                                    decisions[index].spawnLocation = std::move(spawn);
                                }
                            }
                        }
                    }
                    for (std::size_t index = 0; index < candidates.size(); ++index) {
                        const PopulationCandidate& candidate = candidates[index];
                        if (!candidate.isHuman && candidate.canDespawn &&
                            balance.needsDespawn(candidate)) {
                            decisions[index].despawn = true;
                            --balance.countFor(candidate);
                        }
                    }
                    for (std::size_t index = candidates.size(); index > 0; --index) {
                        const std::size_t candidateIndex = index - 1;
                        const PopulationCandidate& candidate = candidates[candidateIndex];
                        if (candidate.isHuman && candidate.canDespawn &&
                            balance.needsDespawn(candidate)) {
                            decisions[candidateIndex].despawn = true;
                            --balance.countFor(candidate);
                        }
                    }
                    std::vector<std::size_t> spawnOrder;
                    spawnOrder.reserve(candidates.size());
                    for (std::size_t index = 0; index < candidates.size(); ++index) {
                        if (!candidates[index].active && candidates[index].canSpawn) {
                            spawnOrder.push_back(index);
                        }
                    }
                    std::stable_sort(spawnOrder.begin(), spawnOrder.end(),
                                     [&](std::size_t first, std::size_t second) {
                                         return candidates[first].spawnPriorityMeters <
                                                candidates[second].spawnPriorityMeters;
                                     });
                    bool reservedSpawn = false;
                    for (const std::size_t index : spawnOrder) {
                        const PopulationCandidate& candidate = candidates[index];
                        if (balance.needsSpawn(candidate)) {
                            if (reservedSpawn && searchContext && candidate.spawnSearch) {
                                decisions[index].roadSearchDiagnostics = {};
                                decisions[index].busSearchDiagnostics.reset();
                                decisions[index].spawnLocation =
                                    findPopulationSpawn(candidate, *searchContext, activeTiles,
                                                        crowding, decisions[index]);
                                if (!decisions[index].spawnLocation) {
                                    continue;
                                }
                            }
                            decisions[index].spawn = true;
                            ++balance.countFor(candidate);
                            if (searchContext && candidate.spawnSearch &&
                                decisions[index].spawnLocation) {
                                const auto& actor = *candidate.spawnSearch->actor;
                                openbus::map::MapTrafficObstacle reservation{
                                    decisions[index].spawnLocation->pose, actor.lengthMeters,
                                    actor.widthMeters};
                                searchContext->occupied.push_back(reservation);
                                crowding.add(reservation);
                                reservedSpawn = true;
                            }
                        }
                    }
                } catch (...) {
                    lock.lock();
                    workerException = std::current_exception();
                    pendingElapsedSeconds = 0.0;
                    hasInput = false;
                    hasPopulationRequest = false;
                    continue;
                }
                lock.lock();
                populationProcessingSeconds +=
                    std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                  populationStart)
                        .count();
                latestPopulationDecisions = std::move(decisions);
                hasLatestPopulationDecisions = true;
                continue;
            }
            const auto tickProcessingStart = std::chrono::steady_clock::now();
            lastWasPopulation = false;
            const std::shared_ptr<const InputSnapshot> tickInput = submittedInput;
            const double simulationStepSecondsForTick = simulationStepSeconds;
            pendingElapsedSeconds -= simulationStepSecondsForTick;
            const double signalClockSeconds = std::max(
                0.0, simulationTimeSeconds - pendingElapsedSeconds - simulationStepSecondsForTick);
            lock.unlock();

            try {
                const auto& tickActors = tickInput->actors;
                const auto& tickHumans = tickInput->humans;
                const auto& tickObstacles = tickInput->obstacles;
                const auto& humanObstacles = tickInput->humanObstacles;
                const auto& tickSignalControllers = tickInput->signalControllers;
                const auto& tickBusPriorityRequests = tickInput->busPriorityRequests;
                std::unordered_map<std::size_t, std::size_t> previousActorByState;
                previousActorByState.reserve(workingActors.size());
                for (std::size_t index = 0; index < workingActors.size(); ++index) {
                    previousActorByState.emplace(workingActors[index].stateIndex, index);
                }
                std::unordered_map<std::size_t, std::size_t> actorPositionByState;
                actorPositionByState.reserve(tickActors.size());
                bool uniqueActorStates = true;
                for (std::size_t index = 0; index < tickActors.size(); ++index) {
                    uniqueActorStates &=
                        actorPositionByState.emplace(tickActors[index].stateIndex, index).second;
                }
                struct Blocker {
                    std::optional<std::size_t> stateIndex;
                    std::optional<std::uint64_t> generation;
                };
                std::vector<Blocker> blockers;
                blockers.reserve(tickActors.size());
                std::vector<openbus::map::MapTrafficDrivingRouteMetadata> actorRouteMetadata;
                actorRouteMetadata.reserve(tickActors.size());
                std::vector<openbus::map::MapTrafficActor> runtimeActors;
                runtimeActors.reserve(tickActors.size());
                for (const ActorInput& actor : tickActors) {
                    const auto previous = previousActorByState.find(actor.stateIndex);
                    const bool sameTemplate =
                        previous != previousActorByState.end() &&
                        workingActors[previous->second].actorTemplate == actor.actorTemplate;
                    const bool hasPrevious =
                        previous != previousActorByState.end() &&
                        workingActors[previous->second].generation == actor.generation;
                    const bool reuseActor = hasPrevious && uniqueActorStates && sameTemplate;
                    actorRouteMetadata.push_back(sameTemplate
                                                     ? workingActors[previous->second].routeMetadata
                                                     : openbus::map::analyzeMapTrafficDrivingRoute(
                                                           actor.actorTemplate->route));
                    if (reuseActor) {
                        runtimeActors.push_back(std::move(workingActors[previous->second].actor));
                    } else {
                        runtimeActors.push_back(*actor.actorTemplate);
                    }
                    openbus::map::MapTrafficActor& runtimeActor = runtimeActors.back();
                    if (reuseActor) {
                        // These two motion fields come from each new snapshot, not the last tick.
                        runtimeActor.lateralOffsetMeters = actor.motion.lateralOffsetMeters;
                        runtimeActor.scheduledStopDistanceMeters =
                            actor.motion.scheduledStopDistanceMeters;
                    } else {
                        applyMotion(runtimeActor, actor.motion);
                    }
                    runtimeActor.reducedSpeedLookahead = actor.reducedSpeedLookahead;
                    if (hasPrevious) {
                        const WorkingActor& previousActor = workingActors[previous->second];
                        if (!reuseActor) {
                            preserveMovementState(runtimeActor, previousActor.actor);
                        }
                        blockers.push_back({previousActor.blockingActorStateIndex,
                                            previousActor.blockingActorGeneration});
                    } else {
                        blockers.push_back(
                            {actor.blockingActorStateIndex, actor.blockingActorGeneration});
                    }
                    runtimeActor.blockingActor = nullptr;
                }
                std::unordered_map<std::size_t, std::size_t> previousHumanByState;
                previousHumanByState.reserve(workingHumans.size());
                for (std::size_t index = 0; index < workingHumans.size(); ++index) {
                    previousHumanByState.emplace(workingHumans[index].stateIndex, index);
                }
                std::vector<WorkingHuman> runtimeHumans;
                runtimeHumans.reserve(tickHumans.size());
                for (const HumanInput& human : tickHumans) {
                    const auto previous = previousHumanByState.find(human.stateIndex);
                    if (previous != previousHumanByState.end() &&
                        workingHumans[previous->second].generation == human.generation) {
                        runtimeHumans.push_back(std::move(workingHumans[previous->second]));
                    } else {
                        if (human.actorTemplate == nullptr) {
                            throw std::invalid_argument(
                                "A traffic human is missing its immutable actor template");
                        }
                        openbus::map::MapTrafficActor actor = *human.actorTemplate;
                        actor.distanceMeters = human.distanceMeters;
                        actor.speedMetersPerSecond = human.speedMetersPerSecond;
                        runtimeHumans.push_back({human.stateIndex, human.generation,
                                                 std::move(actor), human.feetDistanceMeters});
                    }
                    WorkingHuman& runtimeHuman = runtimeHumans.back();
                    runtimeHuman.stateIndex = human.stateIndex;
                    runtimeHuman.generation = human.generation;
                    runtimeHuman.feetDistanceMeters = human.feetDistanceMeters;
                }
                openbus::rendering::TraceScope trace("traffic",
                                                     "MapTrafficWorker::advanceMapTrafficDriving");

                for (std::size_t index = 0; index < tickActors.size(); ++index) {
                    const auto blocker = blockers[index].stateIndex;
                    const auto blockerPosition = blocker.has_value()
                                                     ? actorPositionByState.find(*blocker)
                                                     : actorPositionByState.end();
                    if (blockerPosition != actorPositionByState.end() &&
                        tickActors[blockerPosition->second].generation ==
                            blockers[index].generation) {
                        runtimeActors[index].blockingActor =
                            &runtimeActors[blockerPosition->second];
                    }
                }

                std::vector<openbus::map::MapTrafficActor*> actorPointers;
                actorPointers.reserve(runtimeActors.size());
                std::unordered_map<const openbus::map::MapTrafficActor*, std::size_t>
                    actorPositionByActor;
                actorPositionByActor.reserve(runtimeActors.size());
                for (std::size_t index = 0; index < runtimeActors.size(); ++index) {
                    actorPointers.push_back(&runtimeActors[index]);
                    actorPositionByActor.emplace(&runtimeActors[index], index);
                }
                std::vector<openbus::map::MapTrafficActor*> signalActors = actorPointers;
                for (std::size_t index = 0; index < runtimeHumans.size(); ++index) {
                    if (tickHumans[index].populationEnabled) {
                        signalActors.push_back(&runtimeHumans[index].actor);
                    }
                }
                openbus::map::advanceMapTrafficSignalControllers(
                    tickSignalControllers, signalActors, simulationStepSecondsForTick,
                    signalClockSeconds, tickBusPriorityRequests);
                openbus::map::advanceMapTrafficDriving(actorPointers, tickObstacles,
                                                       simulationStepSecondsForTick,
                                                       signalClockSeconds, actorRouteMetadata);

                std::vector<HumanResult> advancedHumanResults;
                advancedHumanResults.reserve(runtimeHumans.size());
                for (std::size_t index = 0; index < runtimeHumans.size(); ++index) {
                    WorkingHuman& human = runtimeHumans[index];
                    const HumanInput& humanInput = tickHumans[index];
                    if (humanInput.populationEnabled) {
                        openbus::map::advanceMapTrafficActorLoopingAvoidingObstacles(
                            human.actor, humanObstacles, human.feetDistanceMeters * 1.25,
                            human.feetDistanceMeters * 2.1, simulationStepSecondsForTick,
                            signalClockSeconds);
                    } else if (humanInput.advanceWhileInactive) {
                        openbus::map::advanceMapTrafficActorLooping(human.actor,
                                                                    simulationStepSecondsForTick);
                    }
                    advancedHumanResults.push_back({human.stateIndex, human.generation,
                                                    human.actor.distanceMeters,
                                                    human.actor.speedMetersPerSecond});
                }

                std::vector<WorkingActor> advancedActors;
                advancedActors.reserve(tickActors.size());
                for (std::size_t index = 0; index < runtimeActors.size(); ++index) {
                    WorkingActor advanced;
                    advanced.stateIndex = tickActors[index].stateIndex;
                    advanced.generation = tickActors[index].generation;
                    advanced.actorTemplate = tickActors[index].actorTemplate;
                    advanced.routeMetadata = actorRouteMetadata[index];
                    advanced.actor = std::move(runtimeActors[index]);
                    if (advanced.actor.blockingActor != nullptr) {
                        const auto blocker =
                            actorPositionByActor.find(advanced.actor.blockingActor);
                        if (blocker != actorPositionByActor.end()) {
                            advanced.blockingActorStateIndex =
                                tickActors[blocker->second].stateIndex;
                            advanced.blockingActorGeneration =
                                tickActors[blocker->second].generation;
                        }
                    }
                    advanced.actor.blockingActor = nullptr;
                    advancedActors.push_back(std::move(advanced));
                }
                std::vector<openbus::map::MapTrafficPose> advancedPoses;
                advancedPoses.reserve(advancedActors.size());
                for (const WorkingActor& actor : advancedActors) {
                    const auto pose = openbus::map::sampleMapTrafficActorPose(
                        actor.actor, actor.actor.distanceMeters);
                    if (!pose) {
                        throw std::runtime_error("Could not sample an advanced map traffic actor");
                    }
                    advancedPoses.push_back(*pose);
                }
                const double tickProcessingSeconds =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                  tickProcessingStart)
                        .count() +
                    std::exchange(populationProcessingSeconds, 0.0);
                workingActors = std::move(advancedActors);
                std::vector<ActorResult> completedActorResults;
                completedActorResults.reserve(workingActors.size());
                for (std::size_t index = 0; index < workingActors.size(); ++index) {
                    const WorkingActor& actor = workingActors[index];
                    completedActorResults.push_back(
                        {actor.stateIndex, actor.generation, captureMotion(actor.actor),
                         actor.blockingActorStateIndex, actor.blockingActorGeneration,
                         advancedPoses[index], advancedPoses[index].headingRadians});
                }
                workingHumans = std::move(runtimeHumans);
                lock.lock();
                averageTickProcessingSeconds =
                    averageTickProcessingSeconds <= 0.0
                        ? tickProcessingSeconds
                        : averageTickProcessingSeconds * 0.9 + tickProcessingSeconds * 0.1;
                latestResults = std::move(completedActorResults);
                latestHumanResults = std::move(advancedHumanResults);
                hasLatestResults = true;
                latestResultsElapsedSeconds += simulationStepSecondsForTick;
            } catch (...) {
                if (!lock.owns_lock()) {
                    lock.lock();
                }
                workerException = std::current_exception();
                pendingElapsedSeconds = 0.0;
                hasInput = false;
                hasPopulationRequest = false;
            }
        }
    }

    std::mutex mutex;
    std::condition_variable wake;
    bool stopping = false;
    bool hasInput = false;
    double simulationStepSeconds = 1.0 / kDefaultTrafficSimulationRateHz;
    double pendingElapsedSeconds = 0.0;
    double simulationTimeSeconds = 0.0;
    std::shared_ptr<const InputSnapshot> submittedInput;
    std::vector<PopulationCandidate> populationCandidates;
    std::size_t populationRoadTarget = 0;
    std::size_t populationBusTarget = 0;
    std::size_t populationHumanTarget = 0;
    std::vector<openbus::map::MapAiRoadTrafficGroupPopulation> populationGroupTargets;
    bool populationUseGroups = false;
    std::optional<PopulationSearchContext> populationSearchContext;
    bool hasPopulationRequest = false;
    std::vector<WorkingActor> workingActors;
    std::vector<WorkingHuman> workingHumans;
    std::vector<ActorResult> latestResults;
    std::vector<HumanResult> latestHumanResults;
    bool hasLatestResults = false;
    double averageTickProcessingSeconds = 0.0;
    std::vector<PopulationDecision> latestPopulationDecisions;
    bool hasLatestPopulationDecisions = false;
    double latestResultsElapsedSeconds = 0.0;
    std::exception_ptr workerException;
    std::thread worker;
};

MapTrafficWorker::MapTrafficWorker() : impl_(std::make_unique<Impl>()) {}

MapTrafficWorker::~MapTrafficWorker() = default;

void MapTrafficWorker::setSimulationRateHz(double rateHz) {
    if (!std::isfinite(rateHz) || rateHz <= 0.0) {
        throw std::invalid_argument("Traffic worker rate must be finite and positive");
    }
    std::lock_guard lock(impl_->mutex);
    if (impl_->workerException) {
        std::rethrow_exception(impl_->workerException);
    }
    const double simulationStepSeconds = 1.0 / rateHz;
    if (impl_->simulationStepSeconds != simulationStepSeconds) {
        impl_->simulationStepSeconds = simulationStepSeconds;
    }
    impl_->wake.notify_one();
}

void MapTrafficWorker::submit(
    std::vector<ActorInput> actors, std::vector<HumanInput> humans,
    std::vector<openbus::map::MapTrafficObstacle> obstacles,
    std::vector<openbus::map::MapTrafficObstacle> humanObstacles,
    std::vector<std::shared_ptr<openbus::map::MapTrafficSignalController>> signalControllers,
    std::vector<openbus::map::MapTrafficSignalBusPriorityRequest> busPriorityRequests,
    double elapsedSeconds, double simulationTimeSeconds) {
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0 ||
        !std::isfinite(simulationTimeSeconds) || simulationTimeSeconds < 0.0) {
        throw std::invalid_argument("Map traffic worker received an invalid simulation time");
    }
    if (std::any_of(actors.begin(), actors.end(),
                    [](const ActorInput& actor) { return !actor.actorTemplate; })) {
        throw std::invalid_argument("Map traffic worker received no actor template");
    }
    auto input = std::make_shared<Impl::InputSnapshot>(Impl::InputSnapshot{
        std::move(actors), std::move(humans), std::move(obstacles), std::move(humanObstacles),
        std::move(signalControllers), std::move(busPriorityRequests)});
    std::lock_guard lock(impl_->mutex);
    if (impl_->workerException) {
        std::rethrow_exception(impl_->workerException);
    }
    impl_->submittedInput = std::move(input);
    impl_->pendingElapsedSeconds += elapsedSeconds;
    impl_->simulationTimeSeconds = simulationTimeSeconds;
    impl_->hasInput = true;
    impl_->wake.notify_one();
}

void MapTrafficWorker::submitPopulationPlan(
    std::vector<PopulationCandidate> candidates, std::size_t roadTarget, std::size_t busTarget,
    std::size_t humanTarget,
    std::vector<openbus::map::MapAiRoadTrafficGroupPopulation> groupTargets,
    bool useGroupPopulation, std::optional<PopulationSearchContext> searchContext) {
    std::lock_guard lock(impl_->mutex);
    if (impl_->workerException) {
        std::rethrow_exception(impl_->workerException);
    }
    impl_->populationCandidates = std::move(candidates);
    impl_->populationRoadTarget = roadTarget;
    impl_->populationBusTarget = busTarget;
    impl_->populationHumanTarget = humanTarget;
    impl_->populationGroupTargets = std::move(groupTargets);
    impl_->populationUseGroups = useGroupPopulation;
    impl_->populationSearchContext = std::move(searchContext);
    impl_->hasPopulationRequest = true;
    impl_->wake.notify_one();
}

std::optional<MapTrafficWorker::UpdateResults> MapTrafficWorker::takeLatestResults() {
    std::lock_guard lock(impl_->mutex);
    if (impl_->workerException) {
        std::rethrow_exception(impl_->workerException);
    }
    if (!impl_->hasLatestResults) {
        return std::nullopt;
    }
    impl_->hasLatestResults = false;
    UpdateResults results;
    results.actors = std::move(impl_->latestResults);
    results.humans = std::move(impl_->latestHumanResults);
    results.elapsedSeconds = std::exchange(impl_->latestResultsElapsedSeconds, 0.0);
    return results;
}

std::optional<std::vector<MapTrafficWorker::PopulationDecision>>
MapTrafficWorker::takePopulationDecisions() {
    std::lock_guard lock(impl_->mutex);
    if (impl_->workerException) {
        std::rethrow_exception(impl_->workerException);
    }
    if (!impl_->hasLatestPopulationDecisions) {
        return std::nullopt;
    }
    impl_->hasLatestPopulationDecisions = false;
    return std::move(impl_->latestPopulationDecisions);
}

double MapTrafficWorker::averageTickProcessingSeconds() const {
    std::lock_guard lock(impl_->mutex);
    if (impl_->workerException) {
        std::rethrow_exception(impl_->workerException);
    }
    return impl_->averageTickProcessingSeconds;
}

} // namespace openbus::app
