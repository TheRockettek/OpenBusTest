#include "app/MapTrafficWorker.h"

#include "core/PerfTrace.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

namespace openbus::app {

namespace {

constexpr double kDefaultTrafficSimulationRateHz = 60.0;

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
        while (true) {
            wake.wait(lock, [this] {
                return stopping || hasPopulationRequest ||
                       (hasInput && pendingElapsedSeconds >= simulationStepSeconds);
            });
            if (stopping) {
                return;
            }
            if (hasPopulationRequest) {
                auto candidates = std::move(populationCandidates);
                const std::size_t roadTarget = populationRoadTarget;
                const std::size_t busTarget = populationBusTarget;
                const std::size_t humanTarget = populationHumanTarget;
                auto groupTargets = std::move(populationGroupTargets);
                const bool useGroupPopulation = populationUseGroups;
                hasPopulationRequest = false;
                lock.unlock();
                std::vector<PopulationDecision> decisions;
                try {
                    decisions.reserve(candidates.size());
                    std::unordered_map<std::size_t, std::size_t> activeCountsByGroup;
                    std::size_t activeRoadCount = 0;
                    std::size_t activeBusCount = 0;
                    std::size_t activeHumanCount = 0;
                    const auto targetFor = [&](const PopulationCandidate& candidate) {
                        if (candidate.isHuman) {
                            return humanTarget;
                        }
                        if (candidate.isBus) {
                            return busTarget;
                        }
                        if (useGroupPopulation) {
                            if (!candidate.groupIndex.has_value()) {
                                return std::size_t{0};
                            }
                            const auto target = std::find_if(
                                groupTargets.begin(), groupTargets.end(), [&](const auto& group) {
                                    return group.groupIndex == *candidate.groupIndex;
                                });
                            return target == groupTargets.end() ? std::size_t{0}
                                                                : target->populationCount;
                        }
                        return roadTarget;
                    };
                    const auto countFor =
                        [&](const PopulationCandidate& candidate) -> std::size_t& {
                        if (candidate.isHuman) {
                            return activeHumanCount;
                        }
                        if (candidate.isBus) {
                            return activeBusCount;
                        }
                        if (useGroupPopulation && candidate.groupIndex.has_value()) {
                            return activeCountsByGroup[*candidate.groupIndex];
                        }
                        return activeRoadCount;
                    };
                    for (const PopulationCandidate& candidate : candidates) {
                        if (candidate.active) {
                            ++countFor(candidate);
                        }
                        decisions.push_back({candidate.stateIndex, candidate.generation,
                                             candidate.isHuman, false, false});
                    }
                    for (std::size_t index = 0; index < candidates.size(); ++index) {
                        const PopulationCandidate& candidate = candidates[index];
                        if (!candidate.isHuman && candidate.active && candidate.canDespawn &&
                            countFor(candidate) > targetFor(candidate)) {
                            decisions[index].despawn = true;
                            --countFor(candidate);
                        }
                    }
                    for (std::size_t index = candidates.size(); index > 0; --index) {
                        const std::size_t candidateIndex = index - 1;
                        const PopulationCandidate& candidate = candidates[candidateIndex];
                        if (candidate.isHuman && candidate.active && candidate.canDespawn &&
                            countFor(candidate) > targetFor(candidate)) {
                            decisions[candidateIndex].despawn = true;
                            --countFor(candidate);
                        }
                    }
                    for (std::size_t index = 0; index < candidates.size(); ++index) {
                        const PopulationCandidate& candidate = candidates[index];
                        if (!candidate.active && candidate.canSpawn &&
                            countFor(candidate) < targetFor(candidate)) {
                            decisions[index].spawn = true;
                            ++countFor(candidate);
                        }
                    }
                } catch (...) {
                    lock.lock();
                    workerException = std::current_exception();
                    pendingElapsedSeconds = 0.0;
                    hasInput = false;
                    continue;
                }
                lock.lock();
                latestPopulationDecisions = std::move(decisions);
                hasLatestPopulationDecisions = true;
                continue;
            }
            const auto tickProcessingStart = std::chrono::steady_clock::now();
            const std::shared_ptr<const InputSnapshot> tickInput = submittedInput;
            const double simulationStepSecondsForTick = simulationStepSeconds;
            pendingElapsedSeconds -= simulationStepSecondsForTick;
            const double signalClockSeconds = std::max(
                0.0, simulationTimeSeconds - pendingElapsedSeconds - simulationStepSecondsForTick);
            lock.unlock();

            try {
                std::vector<ActorInput> tickActors = tickInput->actors;
                std::vector<HumanInput> tickHumans = tickInput->humans;
                std::vector<openbus::map::MapTrafficObstacle> tickObstacles = tickInput->obstacles;
                std::vector<openbus::map::MapTrafficObstacle> humanObstacles =
                    tickInput->humanObstacles;
                auto tickSignalControllers = tickInput->signalControllers;
                auto tickBusPriorityRequests = tickInput->busPriorityRequests;
                std::unordered_map<std::size_t, std::size_t> previousActorByState;
                previousActorByState.reserve(workingActors.size());
                for (std::size_t index = 0; index < workingActors.size(); ++index) {
                    previousActorByState.emplace(workingActors[index].stateIndex, index);
                }
                std::vector<openbus::map::MapTrafficActor> runtimeActors;
                runtimeActors.reserve(tickActors.size());
                for (ActorInput& actor : tickActors) {
                    const auto previous = previousActorByState.find(actor.stateIndex);
                    const bool hasPrevious =
                        previous != previousActorByState.end() &&
                        workingActors[previous->second].generation == actor.generation;
                    runtimeActors.push_back(*actor.actorTemplate);
                    openbus::map::MapTrafficActor& runtimeActor = runtimeActors.back();
                    applyMotion(runtimeActor, actor.motion);
                    if (hasPrevious) {
                        const WorkingActor& previousActor = workingActors[previous->second];
                        preserveMovementState(runtimeActor, previousActor.actor);
                        actor.blockingActorStateIndex = previousActor.blockingActorStateIndex;
                        actor.blockingActorGeneration = previousActor.blockingActorGeneration;
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

                std::unordered_map<std::size_t, std::size_t> actorPositionByState;
                actorPositionByState.reserve(tickActors.size());
                for (std::size_t index = 0; index < tickActors.size(); ++index) {
                    actorPositionByState.emplace(tickActors[index].stateIndex, index);
                }
                for (std::size_t index = 0; index < tickActors.size(); ++index) {
                    const auto blocker = tickActors[index].blockingActorStateIndex;
                    const auto blockerPosition = blocker.has_value()
                                                     ? actorPositionByState.find(*blocker)
                                                     : actorPositionByState.end();
                    if (blockerPosition != actorPositionByState.end() &&
                        tickActors[blockerPosition->second].generation ==
                            tickActors[index].blockingActorGeneration) {
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
                openbus::map::advanceMapTrafficDriving(
                    actorPointers, tickObstacles, simulationStepSecondsForTick, signalClockSeconds);

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
                std::vector<double> advancedTravelHeadings;
                advancedPoses.reserve(advancedActors.size());
                advancedTravelHeadings.reserve(advancedActors.size());
                for (const WorkingActor& actor : advancedActors) {
                    const auto pose = openbus::map::sampleMapTrafficRoute(
                        actor.actor.route, actor.actor.distanceMeters,
                        actor.actor.lateralOffsetMeters);
                    if (!pose) {
                        throw std::runtime_error("Could not sample an advanced map traffic actor");
                    }
                    advancedPoses.push_back(*pose);
                    advancedTravelHeadings.push_back(
                        openbus::map::mapTrafficRouteTravelHeadingRadians(
                            actor.actor.route, actor.actor.distanceMeters,
                            actor.actor.lateralOffsetMeters));
                }

                std::unordered_map<std::size_t, std::uint64_t> activeStates;
                activeStates.reserve(tickActors.size());
                for (const ActorInput& actor : tickActors) {
                    activeStates.emplace(actor.stateIndex, actor.generation);
                }
                std::vector<WorkingActor> activeActors;
                std::vector<openbus::map::MapTrafficPose> activePoses;
                std::vector<double> activeTravelHeadings;
                activeActors.reserve(advancedActors.size());
                activePoses.reserve(advancedPoses.size());
                activeTravelHeadings.reserve(advancedTravelHeadings.size());
                for (std::size_t index = 0; index < advancedActors.size(); ++index) {
                    const WorkingActor& actor = advancedActors[index];
                    const auto active = activeStates.find(actor.stateIndex);
                    if (active == activeStates.end() || active->second != actor.generation) {
                        continue;
                    }
                    activeActors.push_back(std::move(advancedActors[index]));
                    activePoses.push_back(advancedPoses[index]);
                    activeTravelHeadings.push_back(advancedTravelHeadings[index]);
                }
                advancedActors = std::move(activeActors);
                const double tickProcessingSeconds =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                  tickProcessingStart)
                        .count();
                workingActors = std::move(advancedActors);
                std::vector<ActorResult> completedActorResults;
                completedActorResults.reserve(workingActors.size());
                for (std::size_t index = 0; index < workingActors.size(); ++index) {
                    const WorkingActor& actor = workingActors[index];
                    completedActorResults.push_back(
                        {actor.stateIndex, actor.generation, captureMotion(actor.actor),
                         actor.blockingActorStateIndex, actor.blockingActorGeneration,
                         activePoses[index], activeTravelHeadings[index]});
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
    bool useGroupPopulation) {
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
