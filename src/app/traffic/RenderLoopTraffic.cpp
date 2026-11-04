#include "app/RenderLoopInternal.h"
#include "app/MapTrafficWorker.h"
#include "app/traffic/ScheduledBusTiming.h"
#include "render/map/MapRenderer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace {

constexpr double MAP_TRAFFIC_STUCK_TIMEOUT_SECONDS = 15;
constexpr double MAP_TRAFFIC_PROGRESS_THRESHOLD_METERS = 0.25;
constexpr double MAP_TRAFFIC_STOPPED_SPEED_METERS_PER_SECOND = 0.1;

} // namespace

void RenderLoop::publishTrafficTimetableData(Vehicle& vehicle, const TrafficVehicleState& state,
                                             std::size_t departureIndex) const {
    const openbus::app::ScheduledBusState emptyBus;
    const auto& bus = state.bus ? *state.bus : emptyBus;
    vehicle.variables.setString("setlineto", bus.timetableLineName);
    vehicle.variables.set("ai_target_index", static_cast<float>(bus.timetableTerminusIndex));
    vehicle.variables.set("target_index_int", static_cast<float>(bus.timetableTerminusIndex));
    vehicle.variables.set("schedule_active", bus.scheduledTripActive ? 1.0F : 0.0F);
    vehicle.variables.setString("act_busstop",
                                bus.nextPassengerStopIndex < bus.passengerStops.size()
                                    ? bus.passengerStops[bus.nextPassengerStopIndex].name
                                    : std::string{});
    vehicle.variables.set("tt_terminus_index", static_cast<float>(bus.timetableTerminusIndex));
    vehicle.variables.set("tt_delay", 0.0F);
    vehicle.variables.set("tt_busstop_count", static_cast<float>(bus.passengerStops.size()));
    const std::size_t currentStopIndex =
        bus.passengerStops.empty()
            ? 0
            : std::min(bus.nextPassengerStopIndex, bus.passengerStops.size() - 1);
    vehicle.variables.set("tt_busstop_index", static_cast<float>(currentStopIndex));
    const openbus::map::MapAiTimetableProfile* profile =
        openbus::app::scheduledBusProfileForDeparture(bus, departureIndex);
    const double departureSeconds = departureIndex < bus.scheduledDepartureMinutes.size()
                                        ? bus.scheduledDepartureMinutes[departureIndex] * 60.0
                                        : 0.0;
    constexpr double secondsPerDay = 24.0 * 60.0 * 60.0;
    for (std::size_t stopIndex = 0; stopIndex < bus.passengerStops.size(); ++stopIndex) {
        const openbus::map::MapTrafficPassengerStop& stop = bus.passengerStops[stopIndex];
        vehicle.variables.setString("tt_busstop_name_" + std::to_string(stopIndex), stop.name);
        double arrivalSeconds = 0.0;
        double departureAtStopSeconds = 0.0;
        if (profile != nullptr &&
            stop.timetableStationIndex < profile->arrivalOffsetSeconds.size() &&
            stop.timetableStationIndex < profile->departureOffsetSeconds.size()) {
            arrivalSeconds = std::fmod(
                departureSeconds + profile->arrivalOffsetSeconds[stop.timetableStationIndex],
                secondsPerDay);
            departureAtStopSeconds = std::fmod(
                departureSeconds + profile->departureOffsetSeconds[stop.timetableStationIndex],
                secondsPerDay);
        }
        vehicle.variables.set("tt_busstop_arr_" + std::to_string(stopIndex),
                              static_cast<float>(arrivalSeconds));
        vehicle.variables.set("tt_busstop_dep_" + std::to_string(stopIndex),
                              static_cast<float>(departureAtStopSeconds));
    }
}

void RenderLoop::finishPendingTrafficVehicleLoads() {
    TraceScope trace("traffic", "RenderLoop::finishPendingTrafficVehicleLoads");
    using DeferredStep = Vehicle::DeferredTrafficInitializationStep;
    for (TrafficVehicleState& state : trafficFleet_.vehicles) {
        if (state.vehicle != nullptr || state.vehicleLoadFailed) {
            continue;
        }
        try {
            if (!state.pendingInitializationVehicle) {
                bool vehicleLoadReady = false;
                {
                    TraceScope pollTrace("traffic", "finishPendingTrafficVehicleLoads.poll");
                    vehicleLoadReady = state.pendingVehicle.valid() &&
                                       state.pendingVehicle.wait_for(std::chrono::seconds(0)) ==
                                           std::future_status::ready;
                }
                if (!vehicleLoadReady) {
                    continue;
                }
                {
                    TraceScope getTrace("traffic", "finishPendingTrafficVehicleLoads.get");
                    state.pendingInitializationVehicle = state.pendingVehicle.get();
                }
                if (!state.pendingInitializationVehicle) {
                    throw std::runtime_error("Traffic vehicle loader returned no vehicle");
                }
                {
                    TraceScope metadataTrace("traffic",
                                             "finishPendingTrafficVehicleLoads.metadata");
                    const openbus::app::ScheduledBusState emptyBus;
                    const auto& bus = state.bus ? *state.bus : emptyBus;
                    state.pendingInitializationVehicle->variables.setString("tt_line_string",
                                                                            bus.timetableLineName);
                    state.pendingInitializationVehicle->variables.setString(
                        "tt_busstop_name", bus.timetableDestinationName);
                    state.pendingInitializationVehicle->variables.set(
                        "tt_busstop_count", static_cast<float>(bus.passengerStops.size()));
                    state.pendingInitializationVehicle->variables.set("tt_busstop_index", 0.0F);
                    for (std::size_t stopIndex = 0; stopIndex < bus.passengerStops.size();
                         ++stopIndex) {
                        state.pendingInitializationVehicle->variables.setString(
                            "tt_busstop_name_" + std::to_string(stopIndex),
                            bus.passengerStops[stopIndex].name);
                    }
                    publishTrafficTimetableData(*state.pendingInitializationVehicle, state,
                                                bus.nextScheduledDepartureIndex);
                }
            }
            state.pendingInitializationVehicle->variables.updateEnvironment(
                simulationState_.sharedVariables());
            DeferredStep step;
            {
                TraceScope initializeTrace("traffic",
                                           "finishPendingTrafficVehicleLoads.initialize");
                step = state.pendingInitializationVehicle->finishDeferredTrafficInitialization();
            }
            if (step == DeferredStep::Complete) {
                {
                    TraceScope attachTrace("traffic", "finishPendingTrafficVehicleLoads.attach");
                    state.pendingInitializationVehicle->followsMapTrafficRoute = true;
                    state.pendingInitializationVehicle->mapTrafficRouteFinished = true;
                    state.pendingInitializationVehicle->mapTrafficTileActive = false;
                    Vehicle* const loadedVehicle = state.pendingInitializationVehicle.get();
                    vehicles_.push_back(std::move(state.pendingInitializationVehicle));
                    state.vehicle = loadedVehicle;
                    if (state.isBus() && !state.bus->passengerStops.empty() &&
                        loadedVehicle->scripts) {
                        loadedVehicle->scripts->invokeSystemTrigger("ai_scheduled_settarget");
                        if (state.bus->nextPassengerStopIndex < state.bus->passengerStops.size()) {
                            loadedVehicle->scripts->invokeSystemTrigger("ai_scheduled_setbusstop");
                        }
                    }
                }
                break;
            }
            if (step == DeferredStep::Progressed) {
                break;
            }
        } catch (const std::exception& error) {
            state.pendingInitializationVehicle.reset();
            state.vehicleLoadFailed = true;
            gameLog.Warning("Could not asynchronously load map traffic vehicle '" +
                            state.actor.modelPath + "': " + error.what());
        }
    }
}

void RenderLoop::updateTrafficDriving(const BusSimulation& simulation, double elapsedSeconds) {
    TraceScope trace("traffic", "RenderLoop::updateTrafficDriving");
    using namespace openbus::map;
    if (!trafficFleet_.worker) {
        trafficFleet_.worker = std::make_unique<openbus::app::MapTrafficWorker>();
        trafficFleet_.populationPlanPending = false;
    }
    TraceScope resultTrace("traffic",
                           "RenderLoop::updateTrafficDriving.workerSchedulingAndResultSync");
    double aiRateHz = trafficFleet_.aiSimulationRateHz;
    if (aiRateHz <= 0.0) {
        const double averageRenderFps = frameTiming_.averageRenderFps();
        aiRateHz = (averageRenderFps > 0.0 ? averageRenderFps : 60.0) / trafficFleet_.aiRateDivisor;
    }
    if (trafficFleet_.aiMaximumRateHz > 0.0) {
        aiRateHz = std::min(aiRateHz, trafficFleet_.aiMaximumRateHz);
    }
    constexpr double aiWorkerBudgetFraction = 0.8;
    const double averageWorkerTickSeconds = trafficFleet_.worker->averageTickProcessingSeconds();
    trafficFleet_.currentAiWorkerTickMilliseconds = averageWorkerTickSeconds * 1000.0;
    if (averageWorkerTickSeconds > 0.0) {
        aiRateHz = std::min(aiRateHz, aiWorkerBudgetFraction / averageWorkerTickSeconds);
    }
    const double requestedAiRateHz = std::clamp(aiRateHz, 0.01, 1000.0);
    if (trafficFleet_.currentAiSimulationRateHz <= 0.0) {
        trafficFleet_.currentAiSimulationRateHz = requestedAiRateHz;
    } else {
        constexpr double rateSmoothing = 0.2;
        trafficFleet_.currentAiSimulationRateHz +=
            (requestedAiRateHz - trafficFleet_.currentAiSimulationRateHz) * rateSmoothing;
    }
    if (trafficFleet_.configuredAiSimulationRateHz <= 0.0 ||
        std::abs(trafficFleet_.currentAiSimulationRateHz -
                 trafficFleet_.configuredAiSimulationRateHz) >=
            std::max(0.01, trafficFleet_.configuredAiSimulationRateHz * 0.01)) {
        trafficFleet_.worker->setSimulationRateHz(trafficFleet_.currentAiSimulationRateHz);
        trafficFleet_.configuredAiSimulationRateHz = trafficFleet_.currentAiSimulationRateHz;
    }
    trafficFleet_.trafficWorkerInputAccumulatorSeconds += elapsedSeconds;
    const double trafficInputIntervalSeconds = 1.0 / trafficFleet_.currentAiSimulationRateHz;
    const bool trafficInputDue =
        trafficFleet_.trafficWorkerInputAccumulatorSeconds >= trafficInputIntervalSeconds;
    double trafficInputElapsedSeconds = 0.0;
    if (trafficInputDue) {
        trafficInputElapsedSeconds = trafficFleet_.trafficWorkerInputAccumulatorSeconds;
        trafficFleet_.trafficWorkerInputAccumulatorSeconds = 0.0;
    }
    using PopulationDecision = openbus::app::MapTrafficWorker::PopulationDecision;
    std::vector<std::optional<PopulationDecision>> populationDecisionsByVehicle(
        trafficFleet_.vehicles.size());
    std::vector<std::optional<PopulationDecision>> populationDecisionsByHuman(
        humanTrafficActors_.size());
    if (trafficInputDue) {
        if (auto decisions = trafficFleet_.worker->takePopulationDecisions()) {
            trafficFleet_.populationPlanPending = false;
            for (const auto& decision : *decisions) {
                auto& decisionsByState =
                    decision.isHuman ? populationDecisionsByHuman : populationDecisionsByVehicle;
                if (decision.stateIndex < decisionsByState.size()) {
                    decisionsByState[decision.stateIndex] = decision;
                }
            }
        }
    }
    if (auto results = trafficFleet_.worker->takeLatestResults()) {
        for (const auto& result : results->actors) {
            if (result.stateIndex >= trafficFleet_.vehicles.size()) {
                continue;
            }
            TrafficVehicleState& state = trafficFleet_.vehicles[result.stateIndex];
            if (state.workerGeneration == result.generation && state.populationEnabled &&
                state.vehicle != nullptr && state.vehicle->mapTrafficTileActive) {
                state.actor.distanceMeters = result.motion.distanceMeters;
                state.actor.speedMetersPerSecond = result.motion.speedMetersPerSecond;
                state.actor.targetSpeedMetersPerSecond = result.motion.targetSpeedMetersPerSecond;
                state.actor.lateralOffsetMeters = result.motion.lateralOffsetMeters;
                state.actor.junctionWaitSeconds = result.motion.junctionWaitSeconds;
                state.actor.scheduledStopDistanceMeters = result.motion.scheduledStopDistanceMeters;
                state.actor.blockReason = result.motion.blockReason;
                state.actor.blockingObstacleIndex = result.motion.blockingObstacleIndex;
                state.actor.junctionClaimEndMeters = result.motion.junctionClaimEndMeters;
                state.actor.committedSignalSegmentIndex = result.motion.committedSignalSegmentIndex;
                state.cachedPose = result.pose;
                state.cachedPoseDistanceMeters = state.actor.distanceMeters;
                state.cachedPoseLateralOffsetMeters = state.actor.lateralOffsetMeters;
                state.cachedTravelHeadingRadians = result.travelHeadingRadians;
                state.cachedPoseGeneration = state.workerGeneration;
            }
        }
        for (const auto& result : results->actors) {
            if (result.stateIndex >= trafficFleet_.vehicles.size()) {
                continue;
            }
            TrafficVehicleState& state = trafficFleet_.vehicles[result.stateIndex];
            if (state.workerGeneration != result.generation || !state.populationEnabled ||
                state.vehicle == nullptr || !state.vehicle->mapTrafficTileActive) {
                continue;
            }
            const bool blockerGenerationMatches =
                result.blockingActorStateIndex.has_value() &&
                result.blockingActorGeneration.has_value() &&
                *result.blockingActorStateIndex < trafficFleet_.vehicles.size() &&
                trafficFleet_.vehicles[*result.blockingActorStateIndex].workerGeneration ==
                    *result.blockingActorGeneration;
            state.blockingActorStateIndex =
                blockerGenerationMatches ? result.blockingActorStateIndex : std::nullopt;
            state.blockingActorGeneration =
                blockerGenerationMatches ? result.blockingActorGeneration : std::nullopt;
            state.actor.blockingActor =
                state.blockingActorStateIndex.has_value()
                    ? &trafficFleet_.vehicles[*state.blockingActorStateIndex].actor
                    : nullptr;
        }
        for (const auto& result : results->humans) {
            if (result.stateIndex >= humanTrafficActors_.size()) {
                continue;
            }
            HumanTrafficState& human = humanTrafficActors_[result.stateIndex];
            if (human.workerGeneration != result.generation) {
                continue;
            }
            const double previousDistanceMeters = human.actor.distanceMeters;
            human.actor.distanceMeters = result.distanceMeters;
            human.actor.speedMetersPerSecond = result.speedMetersPerSecond;
            if (!human.populationEnabled) {
                continue;
            }
            double distanceMovedMeters = human.actor.distanceMeters - previousDistanceMeters;
            if (distanceMovedMeters < 0.0 && human.actor.route.closed) {
                const double loopStart = human.actor.route.loopStartDistance.value_or(0.0);
                distanceMovedMeters += human.actor.route.totalLength - loopStart;
            }
            openbus::rendering::advanceHumanGait(human.gait, std::max(0.0, distanceMovedMeters),
                                                 results->elapsedSeconds,
                                                 human.walkStrideLengthMeters);
        }
    }
    resultTrace.finish();
    finishPendingTrafficVehicleLoads();
    const auto trafficVehiclePose = [&](TrafficVehicleState& state) {
        if (state.cachedPose.has_value() && state.cachedPoseGeneration == state.workerGeneration &&
            state.cachedPoseDistanceMeters == state.actor.distanceMeters &&
            state.cachedPoseLateralOffsetMeters == state.actor.lateralOffsetMeters) {
            return state.cachedPose;
        }
        const auto pose = sampleMapTrafficActorPose(state.actor, state.actor.distanceMeters);
        if (pose) {
            state.cachedPose = pose;
            state.cachedPoseDistanceMeters = state.actor.distanceMeters;
            state.cachedPoseLateralOffsetMeters = state.actor.lateralOffsetMeters;
            state.cachedTravelHeadingRadians = pose->headingRadians;
            state.cachedPoseGeneration = state.workerGeneration;
        }
        return pose;
    };
    const auto poseOnActiveTile = [&](const MapTrafficPose& pose) {
        return mapRenderer_ != nullptr &&
               mapTrafficPoseOnActiveTile(pose, mapRenderer_->activeTileCoordinates(),
                                          mapRenderer_->tileSize());
    };
    {
        TraceScope tileTrace("traffic", "RenderLoop::updateTrafficDriving.activeTileSync");
        for (std::size_t index = 0; index < trafficFleet_.vehicles.size(); ++index) {
            TrafficVehicleState& state = trafficFleet_.vehicles[index];
            if (state.vehicle == nullptr) {
                continue;
            }
            if (!state.populationEnabled) {
                if (state.inactiveResetGeneration == state.workerGeneration) {
                    continue;
                }
                state.vehicle->mapTrafficTileActive = false;
                state.vehicle->variables.set("ai_blinker_l", 0.0);
                state.vehicle->variables.set("ai_blinker_r", 0.0);
                state.vehicle->variables.set("ai_brakelight", 0.0);
                state.lastProgressDistanceMeters = state.actor.distanceMeters;
                state.stoppedSecondsSinceProgress = 0.0;
                state.deadlocked = false;
                state.stuckDiagnosticLogged = false;
                state.inactiveResetGeneration = state.workerGeneration;
                continue;
            }
            const auto pose = trafficVehiclePose(state);
            if (!pose) {
                throw std::runtime_error("Could not sample a map traffic tile pose");
            }
            state.vehicle->mapTrafficTileActive = poseOnActiveTile(*pose);
            if (!state.vehicle->mapTrafficTileActive) {
                state.actor.speedMetersPerSecond = 0.0;
                if (!state.isBus()) {
                    state.populationEnabled = false;
                    state.vehicle->mapTrafficRouteFinished = true;
                    state.vehicle->variables.set("velocity", 0.0);
                    state.vehicle->variables.set("velocity_ground", 0.0);
                    state.vehicle->variables.set("ai_blinker_l", 0.0);
                    state.vehicle->variables.set("ai_blinker_r", 0.0);
                    state.vehicle->variables.set("ai_brakelight", 0.0);
                    state.activeSpawnCandidateIndex.reset();
                    state.deadlockRemovalCommitted = false;
                    ++state.workerGeneration;
                    if (trafficFleet_.trafficSpawnDiagnosticsEnabled) {
                        ++trafficFleet_.roadTrafficRecycledOutsideTiles;
                    }
                }
                state.lastProgressDistanceMeters = state.actor.distanceMeters;
                state.stoppedSecondsSinceProgress = 0.0;
                state.deadlocked = false;
                state.stuckDiagnosticLogged = false;
            }
        }
    }
    const double currentSimulationTime = mapTrafficElapsedSeconds(simulation.simulationTime());
    const double currentSignalClockSeconds =
        mapTrafficSignalClockSeconds(currentSimulationTime, trafficFleet_.mapTrafficStartHour);
    const auto advanceScheduledDeparture = [&](TrafficVehicleState& state,
                                               double afterElapsedSeconds) {
        openbus::app::advanceScheduledBusDeparture(*state.bus, trafficFleet_.mapTrafficStartHour,
                                                   afterElapsedSeconds);
    };
    const auto finishScheduledTrip = [&](TrafficVehicleState& state) {
        if (!state.isBus() || !state.bus->scheduledTripActive) {
            return;
        }
        if (state.bus->passengerStopArrived && state.vehicle && state.vehicle->passengers) {
            auto& service = state.vehicle->passengers->service;
            if (passengerNetwork_.queues.contains(service.currentStopId())) {
                passengerNetwork_.departService(service);
            } else {
                service.depart();
            }
            state.bus->passengerStopArrived = false;
            state.bus->passengerStopDepartureRequested = false;
        }
        openbus::app::finishScheduledBusTrip(*state.bus, currentSimulationTime);
        if (state.vehicle != nullptr) {
            state.vehicle->variables.set("schedule_active", 0.0F);
        }
        while (mapTrafficScheduledDepartureDue(currentSimulationTime,
                                               state.bus->nextScheduledDepartureElapsedSeconds)) {
            advanceScheduledDeparture(state, currentSimulationTime);
        }
    };
    for (TrafficVehicleState& state : trafficFleet_.vehicles) {
        if (state.bus &&
            openbus::app::consumeSuppressedDeparture(*state.bus, trafficFleet_.mapTrafficStartHour,
                                                     currentSimulationTime)) {
            ++state.workerGeneration;
        }
    }
    {
        TraceScope stopTrace("traffic", "RenderLoop::updateTrafficDriving.scheduledStopSync");
        for (TrafficVehicleState& state : trafficFleet_.vehicles) {
            if (!state.isBus() || state.vehicle == nullptr || !state.populationEnabled) {
                continue;
            }
            if (state.bus->passengerStopArrived) {
                if (state.bus->scheduledServiceCancelled) {
                    auto& service = state.vehicle->passengers->service;
                    const auto queue = passengerNetwork_.queues.find(service.currentStopId());
                    if (queue != passengerNetwork_.queues.end()) {
                        passengerNetwork_.endService(service);
                    } else {
                        service.endServiceAtCurrentStop();
                    }
                    if (state.bus->passengerStopDepartureRequested &&
                        state.vehicle->passengers->service.dwelling()) {
                        state.bus->passengerStopDepartureRequested = false;
                        state.vehicle->variables.set("ai_scheduled_atstation", 1.0F);
                    }
                }
            }
            const auto departureAction = openbus::map::mapTrafficPassengerStopDepartureAction(
                state.bus->passengerStopArrived, state.vehicle->passengers->service.dwelling(),
                state.bus->passengerStopDepartureRequested,
                state.vehicle->variables.get("ai_scheduled_atstation"),
                state.bus->scheduledServiceCancelled);
            if (departureAction ==
                openbus::map::MapTrafficPassengerStopDepartureAction::RequestDeparture) {
                state.vehicle->variables.set("ai_scheduled_atstation", -1.0F);
                state.bus->passengerStopDepartureRequested = true;
            } else if (departureAction ==
                       openbus::map::MapTrafficPassengerStopDepartureAction::BeginRetirementRun) {
                openbus::app::beginScheduledBusRetirementRun(*state.bus);
                state.vehicle->variables.setString("act_busstop", "");
                ++state.workerGeneration;
            } else if (departureAction ==
                       openbus::map::MapTrafficPassengerStopDepartureAction::ResumeRoute) {
                state.vehicle->passengers->service.rearmPassengerStopRequestsAfterStop(
                    state.bus->passengerStops[state.bus->nextPassengerStopIndex].stopId);
                state.bus->passengerStopArrived = false;
                state.bus->passengerStopDepartureRequested = false;
                ++state.bus->nextPassengerStopIndex;
                state.vehicle->variables.set(
                    "tt_busstop_index",
                    static_cast<float>(std::min(state.bus->nextPassengerStopIndex,
                                                state.bus->passengerStops.empty()
                                                    ? std::size_t{0}
                                                    : state.bus->passengerStops.size() - 1)));
                if (state.bus->nextPassengerStopIndex < state.bus->passengerStops.size()) {
                    state.vehicle->variables.setString(
                        "act_busstop",
                        state.bus->passengerStops[state.bus->nextPassengerStopIndex].name);
                    if (state.vehicle->scripts) {
                        state.vehicle->scripts->invokeSystemTrigger("ai_scheduled_setbusstop");
                    }
                } else {
                    state.vehicle->variables.setString("act_busstop", "");
                }
            }
            if (state.bus->passengerStopArrived) {
                state.actor.scheduledStopDistanceMeters = state.actor.distanceMeters;
            } else if (state.bus->retirementDrivingToTerminus) {
                state.actor.scheduledStopDistanceMeters =
                    openbus::app::scheduledBusRetirementTerminus(*state.bus);
            } else if (state.bus->nextPassengerStopIndex < state.bus->passengerStops.size()) {
                state.actor.scheduledStopDistanceMeters =
                    state.bus->passengerStops[state.bus->nextPassengerStopIndex]
                        .routeDistanceMeters;
            } else {
                state.actor.scheduledStopDistanceMeters.reset();
            }
        }
    }
    std::vector<MapTrafficObstacle> external;
    std::size_t externalVehicleObstacleCount = 0;
    std::optional<std::size_t> playerObstacleIndex;
    MapTrafficPose playerSpawnReferencePose;
    bool hasPlayerSpawnReferencePose = false;
    bool playerBusMoving = false;
    double playerPredictedForwardDistanceMeters = 0.0;
    if (playerVehicle_ != nullptr) {
        const auto chassis = simulation.chassisPose();
        playerSpawnReferencePose.position = chassis.position;
        playerSpawnReferencePose.headingRadians =
            std::atan2(chassis.rotation[0], chassis.rotation[3]);
        hasPlayerSpawnReferencePose = true;
        const double playerSpeedMetersPerSecond = std::abs(simulation.speed());
        playerBusMoving = playerSpeedMetersPerSecond > MAP_TRAFFIC_STOPPED_SPEED_METERS_PER_SECOND;
        playerPredictedForwardDistanceMeters = std::min(playerSpeedMetersPerSecond * 4.0, 60.0);
    } else {
        playerSpawnReferencePose.position = onFootState_.feetPosition;
        playerSpawnReferencePose.headingRadians = freeCameraPose_.yawRadians;
        hasPlayerSpawnReferencePose = true;
    }
    {
        TraceScope obstacleTrace("traffic",
                                 "RenderLoop::updateTrafficDriving.externalObstacleCollection");
        for (const auto& vehicle : vehicles_) {
            if (vehicle->followsMapTrafficRoute || vehicle->mapTrafficRouteFinished) {
                continue;
            }
            MapTrafficPose pose;
            if (vehicle.get() == playerVehicle_) {
                pose = playerSpawnReferencePose;
            } else {
                pose.position = vehicle->placement.position;
                pose.headingRadians =
                    (90.0 - vehicle->placement.yawDegrees) * std::numbers::pi / 180.0;
            }
            external.push_back({pose, vehicle->trafficLengthMeters, vehicle->trafficWidthMeters});
            if (vehicle.get() == playerVehicle_) {
                playerObstacleIndex = external.size() - 1;
            }
        }
        externalVehicleObstacleCount = external.size();
        if (playerVehicle_ == nullptr) {
            MapTrafficPose pose;
            pose.position = onFootState_.feetPosition;
            pose.headingRadians = freeCameraPose_.yawRadians;
            external.push_back({pose, 0.6, 0.6});
            playerObstacleIndex = external.size() - 1;
        }
    }
    const TrafficVehicleState* trafficVehicleData = trafficFleet_.vehicles.data();
    if (trafficFleet_.indexedVehicleData != trafficVehicleData ||
        trafficFleet_.indexedVehicleCount != trafficFleet_.vehicles.size()) {
        TraceScope actorIndexTrace("traffic", "RenderLoop::updateTrafficDriving.actorStateIndex");
        trafficFleet_.byActor.clear();
        trafficFleet_.byActor.reserve(trafficFleet_.vehicles.size());
        for (const TrafficVehicleState& state : trafficFleet_.vehicles) {
            trafficFleet_.byActor.emplace(&state.actor, &state);
        }
        trafficFleet_.indexedVehicleData = trafficVehicleData;
        trafficFleet_.indexedVehicleCount = trafficFleet_.vehicles.size();
    }
    const auto& trafficStateByActor = trafficFleet_.byActor;
    // Follow the actual blocker chain; proximity to an unrelated bus must not mask a deadlock.
    const auto legitimateStoppedWait = [&](const TrafficVehicleState& waitingVehicle) {
        if (waitingVehicle.actor.speedMetersPerSecond >
            MAP_TRAFFIC_STOPPED_SPEED_METERS_PER_SECOND) {
            return false;
        }
        std::unordered_set<const TrafficVehicleState*> visited;
        for (const TrafficVehicleState* current = &waitingVehicle;
             current != nullptr && visited.insert(current).second;) {
            const MapTrafficActor& actor = current->actor;
            if (current != &waitingVehicle &&
                actor.speedMetersPerSecond > MAP_TRAFFIC_STOPPED_SPEED_METERS_PER_SECOND) {
                return true;
            }
            if (mapTrafficActorWaitingAtSignal(actor, currentSignalClockSeconds) ||
                (current->bus && openbus::app::scheduledBusWaitingForPassengers(*current->bus)) ||
                (actor.blockingObstacleIndex.has_value() &&
                 actor.blockingObstacleIndex == playerObstacleIndex)) {
                return true;
            }
            const bool stoppedBusAtRouteEnd =
                current != &waitingVehicle && current->isBus() && !actor.route.closed &&
                !actor.route.loopStartDistance &&
                actor.distanceMeters >= actor.route.totalLength - 1.0e-6;
            if (stoppedBusAtRouteEnd) {
                return true;
            }
            const auto blocker = trafficStateByActor.find(actor.blockingActor);
            if (actor.blockingActor == nullptr || blocker == trafficStateByActor.end() ||
                !blocker->second->populationEnabled ||
                !current->blockingActorGeneration.has_value() ||
                blocker->second->workerGeneration != *current->blockingActorGeneration) {
                return false;
            }
            current = blocker->second;
        }
        return false;
    };
    const auto hiddenAtPose = [&](const MapTrafficActor& actor, const MapTrafficPose& pose) {
        if (!lastMainCameraPoseValid_) {
            return false;
        }
        // Pad the sphere so queued traffic never pops in at the screen edge.
        const double radius = std::hypot(actor.lengthMeters, actor.widthMeters) * 0.5 + 2.0;
        const MapTrafficCameraView mainCamera{lastMainCameraPose_.position,
                                              trafficCameraViewProjection_};
        return mapTrafficEntryHiddenFromCameras(pose.position, radius, mainCamera,
                                                trafficReflectionCameraViews_, 50.0);
    };
    const auto hidden = [&](const MapTrafficActor& actor, double distance) {
        const auto pose = sampleMapTrafficActorPose(actor, distance);
        if (!pose) {
            throw std::runtime_error("Cannot sample a queued traffic entry");
        }
        return hiddenAtPose(actor, *pose);
    };
    if (trafficInputDue) {
        TraceScope admissionTrace("traffic",
                                  "RenderLoop::updateTrafficDriving.populationAndAdmission");
        std::vector<MapTrafficObstacle> occupied = external;
        std::set<std::pair<int, int>> activeTiles;
        if (mapRenderer_ != nullptr) {
            const auto& coordinates = mapRenderer_->activeTileCoordinates();
            activeTiles.insert(coordinates.begin(), coordinates.end());
        }
        trafficFleet_.roadRouteRefreshElapsedSeconds += trafficInputElapsedSeconds;
        const auto activeRoadCount = static_cast<std::size_t>(
            std::count_if(trafficFleet_.vehicles.begin(), trafficFleet_.vehicles.end(),
                          [](const TrafficVehicleState& state) {
                              return !state.isBus() && state.populationEnabled;
                          }));
        if (mapRenderer_ != nullptr && !activeTiles.empty() &&
            activeRoadCount < trafficFleet_.targetRoadTrafficCount &&
            trafficFleet_.roadRouteRefreshElapsedSeconds >= 0.05 &&
            !trafficFleet_.vehicles.empty()) {
            trafficFleet_.roadRouteRefreshElapsedSeconds = 0.0;
            const auto& coordinates = mapRenderer_->activeTileCoordinates();
            for (std::size_t checked = 0; checked < trafficFleet_.vehicles.size(); ++checked) {
                const std::size_t index =
                    trafficFleet_.roadRouteRefreshCursor++ % trafficFleet_.vehicles.size();
                TrafficVehicleState& state = trafficFleet_.vehicles[index];
                if (state.isBus() || state.populationEnabled || state.vehicle == nullptr ||
                    state.vehicleLoadFailed || state.deadlockRemovalCommitted) {
                    continue;
                }
                bool hasUsableActiveCandidate = false;
                for (std::size_t candidateIndex = 0; candidateIndex < state.spawnCandidates->size();
                     ++candidateIndex) {
                    const auto& candidate = (*state.spawnCandidates)[candidateIndex];
                    if (activeTiles.contains({candidate.tileX, candidate.tileY}) &&
                        std::ranges::find(state.rejectedSpawnCandidateIndices, candidateIndex) ==
                            state.rejectedSpawnCandidateIndices.end()) {
                        const auto pose =
                            sampleMapTrafficActorPose(state.actor, candidate.distanceMeters);
                        if (!pose) {
                            throw std::runtime_error(
                                "Could not sample a road traffic spawn position");
                        }
                        if (poseOnActiveTile(*pose)) {
                            hasUsableActiveCandidate = true;
                            break;
                        }
                    }
                }
                if (hasUsableActiveCandidate) {
                    continue;
                }
                const MapTrafficVehicleFilter filter{state.actor.vehicleType,
                                                     state.actor.unscheduledGroupIndex, false,
                                                     state.actor.trafficRuleGroupIndex};
                if (++trafficFleet_.roadRouteRefreshSeed == 0) {
                    ++trafficFleet_.roadRouteRefreshSeed;
                }
                if (trafficFleet_.trafficSpawnDiagnosticsEnabled) {
                    ++trafficFleet_.roadRouteRefreshAttempts;
                }
                const auto lane = mapRenderer_->selectTrafficSpawnLaneInTiles(
                    filter, trafficFleet_.roadRouteRefreshSeed, coordinates);
                if (!lane) {
                    break;
                }
                auto route = mapRenderer_->buildDrivingRoute(
                    lane->splineId, lane->laneSelector, trafficFleet_.roadRouteRefreshSeed,
                    state.actor.minimumTurnRadiusMeters, filter);
                if (!route) {
                    break;
                }
                auto subdividedRoute = subdivideMapTrafficRoute(*route);
                auto candidates =
                    mapTrafficRouteSpawnCandidates(subdividedRoute, state.actor.lengthMeters);
                if (!mapTrafficSpawnCandidatesOverlapActiveTiles(candidates, coordinates)) {
                    break;
                }
                state.actor.route = std::move(subdividedRoute);
                resetMapTrafficActorForEntry(state.actor);
                state.spawnCandidates =
                    std::make_shared<const std::vector<MapTrafficSpawnCandidate>>(
                        std::move(candidates));
                state.rejectedSpawnCandidateIndices.clear();
                state.activeSpawnCandidateIndex.reset();
                state.cachedPose.reset();
                state.blockingActorStateIndex.reset();
                state.blockingActorGeneration.reset();
                state.workerActorTemplate = std::make_shared<const MapTrafficActor>(state.actor);
                ++state.workerGeneration;
                if (trafficFleet_.trafficSpawnDiagnosticsEnabled) {
                    ++trafficFleet_.roadRouteRefreshSuccesses;
                }
                break;
            }
        }
        for (auto& state : trafficFleet_.vehicles) {
            if (state.deadlockRemovalCommitted) {
                if (state.populationEnabled && state.vehicle != nullptr &&
                    legitimateStoppedWait(state)) {
                    state.deadlockRemovalCommitted = false;
                    state.lastProgressDistanceMeters = state.actor.distanceMeters;
                    state.stoppedSecondsSinceProgress = 0.0;
                    state.deadlocked = false;
                    state.stuckDiagnosticLogged = false;
                } else if (state.populationEnabled && state.vehicle != nullptr && !state.isBus() &&
                           !state.movedSinceSpawn && state.activeSpawnCandidateIndex.has_value() &&
                           *state.activeSpawnCandidateIndex < state.spawnCandidates->size() &&
                           state.rejectedSpawnCandidateIndices.size() + 1 <
                               state.spawnCandidates->size() &&
                           std::ranges::find(state.rejectedSpawnCandidateIndices,
                                             *state.activeSpawnCandidateIndex) ==
                               state.rejectedSpawnCandidateIndices.end()) {
                    state.rejectedSpawnCandidateIndices.push_back(*state.activeSpawnCandidateIndex);
                    state.vehicle->mapTrafficTileActive = false;
                    state.vehicle->mapTrafficRouteFinished = true;
                    state.vehicle->variables.set("velocity", 0.0);
                    state.vehicle->variables.set("velocity_ground", 0.0);
                    state.actor.speedMetersPerSecond = 0.0;
                    state.populationEnabled = false;
                    ++state.workerGeneration;
                    state.deadlockRemovalCommitted = false;
                    state.stoppedSecondsSinceProgress = 0.0;
                    state.deadlocked = false;
                    state.stuckDiagnosticLogged = false;
                    const auto spawnPose =
                        sampleMapTrafficRoute(state.actor.route, state.spawnedAtDistanceMeters,
                                              state.actor.lateralOffsetMeters);
                    gameLog.Warning(
                        "Map road AI vehicle made no route progress from spawn spline=" +
                        std::to_string(spawnPose ? spawnPose->splineId : 0) +
                        "; rejecting that spawn candidate and retrying elsewhere: " +
                        state.actor.modelPath);
                    state.activeSpawnCandidateIndex.reset();
                    continue;
                } else {
                    if (state.populationEnabled && state.vehicle != nullptr) {
                        state.vehicle->mapTrafficTileActive = false;
                        state.vehicle->mapTrafficRouteFinished = true;
                        state.vehicle->variables.set("velocity", 0.0);
                        state.vehicle->variables.set("velocity_ground", 0.0);
                        state.actor.speedMetersPerSecond = 0.0;
                        state.populationEnabled = false;
                        ++state.workerGeneration;
                        finishScheduledTrip(state);
                        state.stoppedSecondsSinceProgress = 0.0;
                        state.deadlocked = false;
                        state.stuckDiagnosticLogged = false;
                    }
                    continue;
                }
            }
            if (!state.populationEnabled || state.vehicle == nullptr) {
                continue;
            }
            if (!state.vehicle->mapTrafficTileActive) {
                continue;
            }
            const auto pose = trafficVehiclePose(state);
            if (!pose) {
                throw std::runtime_error("Cannot sample active traffic for entry admission");
            }
            occupied.push_back({*pose, state.actor.lengthMeters, state.actor.widthMeters});
        }

        const auto humanFootprint = [&](const HumanTrafficState& human) {
            const auto pose = sampleMapTrafficRoute(human.actor.route, human.actor.distanceMeters,
                                                    human.actor.lateralOffsetMeters);
            if (!pose) {
                throw std::runtime_error("Cannot sample a map human traffic actor");
            }
            return MapTrafficObstacle{*pose, human.feetDistanceMeters * 1.25,
                                      human.feetDistanceMeters * 2.1};
        };

        std::vector<openbus::app::MapTrafficWorker::PopulationCandidate> populationCandidates;
        populationCandidates.reserve(trafficFleet_.vehicles.size() + humanTrafficActors_.size());
        TraceScope humanAdmissionTrace("traffic",
                                       "RenderLoop::updateTrafficDriving.humanAdmission");
        std::size_t humanCount = 0;
        for (const HumanTrafficState& human : humanTrafficActors_) {
            humanCount += human.populationEnabled ? 1U : 0U;
        }
        for (std::size_t index = humanTrafficActors_.size();
             index > 0 && humanCount > targetHumanTrafficCount_; --index) {
            HumanTrafficState& human = humanTrafficActors_[index - 1];
            const auto& decision = populationDecisionsByHuman[index - 1];
            if (human.populationEnabled && decision.has_value() &&
                decision->generation == human.workerGeneration && decision->despawn &&
                hidden(human.actor, human.actor.distanceMeters)) {
                human.populationEnabled = false;
                ++human.workerGeneration;
                human.actor.speedMetersPerSecond = human.actor.cruiseSpeedMetersPerSecond;
                --humanCount;
                gameLog.Log("Map human traffic actor unloaded off-screen by population schedule: " +
                            human.actor.modelPath);
            }
        }

        for (HumanTrafficState& human : humanTrafficActors_) {
            human.advanceWhileInactive = false;
        }
        for (const HumanTrafficState& human : humanTrafficActors_) {
            if (human.populationEnabled) {
                occupied.push_back(humanFootprint(human));
            }
        }
        for (std::size_t index = 0; index < humanTrafficActors_.size(); ++index) {
            HumanTrafficState& human = humanTrafficActors_[index];
            bool canSpawn = false;
            if (!human.populationEnabled && humanCount < targetHumanTrafficCount_) {
                human.actor.speedMetersPerSecond = human.actor.cruiseSpeedMetersPerSecond;
                if (!hidden(human.actor, human.actor.distanceMeters)) {
                    human.advanceWhileInactive = true;
                } else {
                    const MapTrafficObstacle candidate = humanFootprint(human);
                    if (!mapTrafficHasClearance(candidate, occupied)) {
                        human.advanceWhileInactive = true;
                    } else {
                        canSpawn = true;
                        const auto& decision = populationDecisionsByHuman[index];
                        if (decision.has_value() &&
                            decision->generation == human.workerGeneration && decision->spawn) {
                            human.populationEnabled = true;
                            ++human.workerGeneration;
                            ++humanCount;
                            occupied.push_back(candidate);
                            gameLog.Log(
                                "Map human traffic actor spawned at a clear off-screen path: " +
                                human.actor.modelPath + ", spline=" +
                                std::to_string(candidate.pose.splineId) + ", camera distance=" +
                                std::to_string(std::hypot(
                                    candidate.pose.position[0] - lastMainCameraPose_.position[0],
                                    candidate.pose.position[1] - lastMainCameraPose_.position[1])) +
                                " m.");
                        }
                    }
                }
            }
            const bool canDespawn = human.populationEnabled &&
                                    humanCount > targetHumanTrafficCount_ &&
                                    hidden(human.actor, human.actor.distanceMeters);
            populationCandidates.push_back({index, human.workerGeneration, std::nullopt, true,
                                            false, human.populationEnabled, canSpawn, canDespawn});
        }
        for (const HumanTrafficState& human : humanTrafficActors_) {
            if (human.populationEnabled) {
                external.push_back(humanFootprint(human));
            }
        }
        humanAdmissionTrace.finish();

        struct RoadSpawnDiagnostics {
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
            std::size_t vehiclesAtTarget = 0;
            std::optional<std::pair<int, int>> firstInactiveCandidateTile;
        };
        const auto isInsidePlayerFrontExclusion = [&](const MapTrafficPose& pose,
                                                      double vehicleWidthMeters) {
            return playerVehicle_ != nullptr && playerBusMoving &&
                   openbus::map::mapTrafficSpawnInsideForwardExclusion(
                       playerSpawnReferencePose, playerVehicle_->trafficLengthMeters,
                       playerVehicle_->trafficWidthMeters, pose, vehicleWidthMeters, 30.0);
        };
        const auto makeSearchContext = [&] {
            openbus::app::MapTrafficWorker::PopulationSearchContext context;
            context.activeTiles.assign(activeTiles.begin(), activeTiles.end());
            if (mapRenderer_ != nullptr) {
                context.tileSize = mapRenderer_->tileSize();
            }
            context.cameraValid = lastMainCameraPoseValid_;
            context.spawnInView = trafficFleet_.mapTrafficSpawnInView;
            context.mainCamera = {lastMainCameraPose_.position, trafficCameraViewProjection_};
            context.reflectionCameras = trafficReflectionCameraViews_;
            context.hasPlayerPose = hasPlayerSpawnReferencePose;
            context.playerPose = playerSpawnReferencePose;
            context.predictedForwardDistanceMeters = playerPredictedForwardDistanceMeters;
            context.playerMoving = playerVehicle_ != nullptr && playerBusMoving;
            if (playerVehicle_ != nullptr) {
                context.playerLengthMeters = playerVehicle_->trafficLengthMeters;
                context.playerWidthMeters = playerVehicle_->trafficWidthMeters;
            }
            context.diagnostics = trafficFleet_.trafficSpawnDiagnosticsEnabled;
            return context;
        };
        const auto logBusAdmissionWaiting = [&](TrafficVehicleState& state,
                                                const MapTrafficPose& entryPose, bool entryHidden,
                                                bool entryClear, bool entryOnActiveTile,
                                                bool frontBlocked) {
            const unsigned blockerMask = (entryHidden ? 0u : 1u) | (entryClear ? 0u : 2u) |
                                         (entryOnActiveTile ? 0u : 4u) | (frontBlocked ? 8u : 0u);
            const double cameraDistanceMeters =
                lastMainCameraPoseValid_
                    ? std::hypot(
                          std::hypot(entryPose.position[0] - lastMainCameraPose_.position[0],
                                     entryPose.position[1] - lastMainCameraPose_.position[1]),
                          entryPose.position[2] - lastMainCameraPose_.position[2])
                    : 0.0;
            if (blockerMask != state.bus->lastScheduledAdmissionDiagnosticBlockerMask ||
                !lastMainCameraPoseValid_ ||
                std::abs(cameraDistanceMeters -
                         state.bus->lastScheduledAdmissionDiagnosticCameraDistanceMeters) >= 50.0) {
                gameLog.Debug("Scheduled map AI bus departure waiting: spline=" +
                              std::to_string(entryPose.splineId) +
                              ", hidden=" + std::to_string(entryHidden) +
                              ", clear=" + std::to_string(entryClear) +
                              ", frontBlocked=" + std::to_string(frontBlocked) +
                              ", activeTile=" + std::to_string(entryOnActiveTile) +
                              ", cameraValid=" + std::to_string(lastMainCameraPoseValid_) +
                              ", cameraDistance=" + std::to_string(cameraDistanceMeters) +
                              "m, simulationSeconds=" + std::to_string(currentSimulationTime) +
                              ", departureSeconds=" +
                              std::to_string(state.bus->nextScheduledDepartureElapsedSeconds) +
                              ": " + state.actor.modelPath);
                state.bus->lastScheduledAdmissionDiagnosticBlockerMask = blockerMask;
                state.bus->lastScheduledAdmissionDiagnosticCameraDistanceMeters =
                    cameraDistanceMeters;
            }
        };

        if (std::ranges::any_of(populationDecisionsByVehicle,
                                [](const auto& decision) { return decision.has_value(); })) {
            TraceScope applyPopulationTrace(
                "traffic", "RenderLoop::updateTrafficDriving.applyPopulationDecisions");
            auto admissionCandidates = populationCandidates;
            for (std::size_t index = 0; index < trafficFleet_.vehicles.size(); ++index) {
                const auto& state = trafficFleet_.vehicles[index];
                if (state.vehicle != nullptr && !state.vehicleLoadFailed &&
                    !state.deadlockRemovalCommitted &&
                    !(state.isBus() && state.bus->scheduledServiceCancelled)) {
                    admissionCandidates.push_back({index, state.workerGeneration,
                                                   state.actor.unscheduledGroupIndex, false,
                                                   state.isBus(), state.populationEnabled});
                }
            }
            openbus::app::MapTrafficWorker::PopulationBalance admissionBalance(
                admissionCandidates, trafficFleet_.targetRoadTrafficCount,
                trafficFleet_.targetBusTrafficCount, targetHumanTrafficCount_,
                trafficFleet_.targetRoadGroupPopulations, trafficFleet_.useGroupRoadPopulation);
            std::optional<openbus::app::MapTrafficWorker::PopulationSearchContext> admissionContext;
            std::vector<std::size_t> populationDecisionOrder;
            populationDecisionOrder.reserve(trafficFleet_.vehicles.size());
            for (std::size_t index = 0; index < trafficFleet_.vehicles.size(); ++index) {
                if (populationDecisionsByVehicle[index]) {
                    populationDecisionOrder.push_back(index);
                }
            }
            std::stable_sort(populationDecisionOrder.begin(), populationDecisionOrder.end(),
                             [&](std::size_t first, std::size_t second) {
                                 const auto priority = [&](std::size_t index) {
                                     const auto& decision = populationDecisionsByVehicle[index];
                                     return decision && decision->spawnLocation
                                                ? decision->spawnLocation->priorityMeters
                                                : std::numeric_limits<double>::infinity();
                                 };
                                 return priority(first) < priority(second);
                             });
            for (const std::size_t stateIndex : populationDecisionOrder) {
                auto& state = trafficFleet_.vehicles[stateIndex];
                if (state.vehicle == nullptr || state.vehicleLoadFailed ||
                    state.deadlockRemovalCommitted ||
                    (state.isBus() && state.bus->scheduledServiceCancelled)) {
                    continue;
                }
                const auto& decision = populationDecisionsByVehicle[stateIndex];
                const openbus::app::MapTrafficWorker::PopulationCandidate admissionCandidate{
                    stateIndex, state.workerGeneration, state.actor.unscheduledGroupIndex,
                    false,      state.isBus(),          state.populationEnabled};
                if (state.isBus() && !state.populationEnabled && decision &&
                    decision->generation == state.workerGeneration &&
                    decision->busSearchDiagnostics &&
                    mapTrafficScheduledDepartureDue(
                        currentSimulationTime, state.bus->nextScheduledDepartureElapsedSeconds)) {
                    const auto& diagnostics = *decision->busSearchDiagnostics;
                    logBusAdmissionWaiting(state, diagnostics.pose, diagnostics.hidden,
                                           diagnostics.clear, diagnostics.onActiveTile,
                                           diagnostics.frontBlocked);
                }
                if (state.populationEnabled && decision.has_value() &&
                    decision->generation == state.workerGeneration && decision->despawn &&
                    admissionBalance.needsDespawn(admissionCandidate) &&
                    hidden(state.actor, state.actor.distanceMeters)) {
                    --admissionBalance.countFor(admissionCandidate);
                    state.populationEnabled = false;
                    ++state.workerGeneration;
                    state.vehicle->mapTrafficRouteFinished = true;
                    finishScheduledTrip(state);
                    state.deadlocked = false;
                }
                if (state.populationEnabled || !decision.has_value() ||
                    decision->generation != state.workerGeneration || !decision->spawn ||
                    !decision->spawnLocation || !admissionBalance.needsSpawn(admissionCandidate)) {
                    continue;
                }
                if (!admissionContext) {
                    admissionContext = makeSearchContext();
                }
                const bool departureDue =
                    !state.isBus() ||
                    mapTrafficScheduledDepartureDue(
                        currentSimulationTime, state.bus->nextScheduledDepartureElapsedSeconds);
                const std::vector<MapTrafficSpawnCandidate> noSpawnCandidates;
                const auto proposal = openbus::app::MapTrafficWorker::revalidatePopulationSpawn(
                    admissionCandidate, admissionBalance, *decision, state.actor,
                    state.spawnCandidates ? *state.spawnCandidates : noSpawnCandidates,
                    state.rejectedSpawnCandidateIndices, *admissionContext, occupied, departureDue);
                if (!proposal) {
                    if (!state.isBus() || !departureDue) {
                        continue;
                    }
                    const auto entryPose = sampleMapTrafficActorPose(state.actor, 0.0);
                    if (!entryPose) {
                        throw std::runtime_error("Could not sample a map traffic entry pose");
                    }
                    const bool entryHidden = trafficFleet_.mapTrafficSpawnInView ||
                                             hiddenAtPose(state.actor, *entryPose);
                    const bool frontBlocked =
                        isInsidePlayerFrontExclusion(*entryPose, state.actor.widthMeters);
                    const bool entryClear =
                        !frontBlocked && canSpawnMapTrafficEntry(state.actor, true, occupied);
                    const bool entryOnActiveTile = poseOnActiveTile(*entryPose);
                    if (!entryHidden || !entryClear || !entryOnActiveTile) {
                        logBusAdmissionWaiting(state, *entryPose, entryHidden, entryClear,
                                               entryOnActiveTile, frontBlocked);
                    }
                    continue;
                }
                const auto& spawn = proposal;
                ++admissionBalance.countFor(admissionCandidate);
                resetMapTrafficActorForEntry(state.actor);
                state.actor.distanceMeters = spawn->distanceMeters;
                // Start from rest so the first traffic tick can react to nearby leaders and red
                // lights.
                state.actor.speedMetersPerSecond = 0.0;
                state.wheelTravelDistanceMeters = 0.0;
                state.previousWheelRouteDistanceMeters = state.actor.distanceMeters;
                state.wheelRouteDistanceInitialized = true;
                state.populationEnabled = true;
                ++state.workerGeneration;
                state.lastProgressDistanceMeters = state.actor.distanceMeters;
                state.spawnedAtDistanceMeters = state.actor.distanceMeters;
                if (state.bus) {
                    state.bus->lastScheduledProgressDiagnosticDistanceMeters =
                        state.actor.distanceMeters;
                }
                state.movedSinceSpawn = false;
                state.activeSpawnCandidateIndex =
                    state.isBus() ? std::nullopt
                                  : std::optional<std::size_t>(spawn->candidateIndex);
                state.stoppedSecondsSinceProgress = 0.0;
                state.deadlocked = false;
                state.stuckDiagnosticLogged = false;
                state.vehicle->mapTrafficRouteFinished = false;
                state.vehicle->mapTrafficTileActive = true;
                const MapTrafficRoute& spawnedRoute = state.actor.route;
                if (spawnedRoute.segments.empty() ||
                    spawnedRoute.cumulativeEndDistances.size() != spawnedRoute.segments.size()) {
                    throw std::logic_error("Spawned map traffic vehicle has an invalid route");
                }
                const auto routeSegment = std::upper_bound(
                    spawnedRoute.cumulativeEndDistances.begin(),
                    spawnedRoute.cumulativeEndDistances.end(), state.actor.distanceMeters);
                const std::size_t routeSegmentIndex =
                    routeSegment == spawnedRoute.cumulativeEndDistances.end()
                        ? spawnedRoute.segments.size() - 1
                        : static_cast<std::size_t>(routeSegment -
                                                   spawnedRoute.cumulativeEndDistances.begin());
                const MapTrafficRouteSegment& spawnedSegment =
                    spawnedRoute.segments[routeSegmentIndex];
                const MapTrafficVehicleFilter spawnedVehicleFilter{
                    state.actor.vehicleType, state.actor.unscheduledGroupIndex,
                    state.actor.scheduled, state.actor.trafficRuleGroupIndex};
                gameLog.Log(std::string(state.isBus() ? "Scheduled map AI bus spawned: "
                                                      : "Map road AI vehicle spawned: ") +
                            "vehicle=" + state.actor.modelPath + ", spline=" +
                            std::to_string(spawnedSegment.spline.placement.splineId) + ", path=" +
                            (spawnedSegment.sceneryPathIndex.has_value()
                                 ? std::to_string(*spawnedSegment.sceneryPathIndex)
                                 : std::string("none")) +
                            ", priority=" +
                            std::to_string(mapTrafficPathPriorityForVehicle(
                                spawnedSegment.pathRules, spawnedVehicleFilter)) +
                            ", density=" +
                            std::to_string(mapTrafficPathDensityForVehicle(spawnedSegment.pathRules,
                                                                           spawnedVehicleFilter)) +
                            ", unscheduledGroup=" +
                            (state.actor.unscheduledGroupIndex.has_value()
                                 ? std::to_string(*state.actor.unscheduledGroupIndex)
                                 : std::string("none")) +
                            ", trafficRuleGroup=" +
                            (state.actor.trafficRuleGroupIndex.has_value()
                                 ? std::to_string(*state.actor.trafficRuleGroupIndex)
                                 : std::string("none")));
                if (state.isBus()) {
                    state.bus->nextPassengerStopIndex = 0;
                    while (state.bus->nextPassengerStopIndex < state.bus->passengerStops.size() &&
                           state.bus->passengerStops[state.bus->nextPassengerStopIndex]
                                       .routeDistanceMeters +
                                   0.05 <
                               state.actor.distanceMeters) {
                        ++state.bus->nextPassengerStopIndex;
                    }
                    state.vehicle->variables.set(
                        "tt_busstop_index",
                        static_cast<float>(std::min(state.bus->nextPassengerStopIndex,
                                                    state.bus->passengerStops.empty()
                                                        ? std::size_t{0}
                                                        : state.bus->passengerStops.size() - 1)));
                    state.bus->passengerStopArrived = false;
                    state.bus->lastScheduledDepartureMinutes =
                        state.bus
                            ->scheduledDepartureMinutes[state.bus->nextScheduledDepartureIndex];
                    state.bus->scheduledTripActive = true;
                    state.vehicle->variables.set("ai_scheduled_atstation", 0.0F);
                    state.bus->activeTimetableDepartureIndex =
                        state.bus->nextScheduledDepartureIndex;
                    state.bus->activeScheduledDepartureElapsedSeconds =
                        state.bus->nextScheduledDepartureElapsedSeconds;
                    publishTrafficTimetableData(*state.vehicle, state,
                                                state.bus->activeTimetableDepartureIndex);
                    if (state.vehicle->scripts) {
                        state.vehicle->scripts->invokeSystemTrigger("ai_scheduled_settarget");
                        if (state.bus->nextPassengerStopIndex < state.bus->passengerStops.size()) {
                            state.vehicle->scripts->invokeSystemTrigger("ai_scheduled_setbusstop");
                        }
                    }
                    for (const MapTrafficPassengerStop& stop : state.bus->passengerStops) {
                        const auto queue = passengerNetwork_.queues.find(stop.stopId);
                        const auto initialQueue = passengerNetwork_.initialQueues.find(stop.stopId);
                        if (queue == passengerNetwork_.queues.end() ||
                            initialQueue == passengerNetwork_.initialQueues.end()) {
                            throw std::logic_error(
                                "Scheduled bus passenger stop has no initialized waiting queue");
                        }
                        queue->second = mapScheduledPassengerQueueAfterDeparture(
                            queue->second, initialQueue->second);
                    }
                    advanceScheduledDeparture(state,
                                              state.bus->nextScheduledDepartureElapsedSeconds);
                    gameLog.Log("Scheduled map AI bus departed from endpoint spline=" +
                                std::to_string(spawn->pose.splineId) + " at timetable minute=" +
                                std::to_string(state.bus->lastScheduledDepartureMinutes) + ": " +
                                state.actor.modelPath);
                }
                occupied.push_back(
                    {spawn->pose, state.actor.lengthMeters, state.actor.widthMeters});
            }
        }

        RoadSpawnDiagnostics roadSpawnDiagnostics;
        for (const auto& decision : populationDecisionsByVehicle) {
            if (!decision) {
                continue;
            }
            const auto& diagnostics = decision->roadSearchDiagnostics;
            roadSpawnDiagnostics.vehiclesChecked += diagnostics.vehiclesChecked;
            roadSpawnDiagnostics.candidatesChecked += diagnostics.candidatesChecked;
            roadSpawnDiagnostics.rejectedCandidates += diagnostics.rejectedCandidates;
            roadSpawnDiagnostics.inactiveCandidateTiles += diagnostics.inactiveCandidateTiles;
            roadSpawnDiagnostics.posesOutsideActiveTiles += diagnostics.posesOutsideActiveTiles;
            roadSpawnDiagnostics.visibleCandidates += diagnostics.visibleCandidates;
            roadSpawnDiagnostics.obstructedCandidates += diagnostics.obstructedCandidates;
            roadSpawnDiagnostics.clearCandidates += diagnostics.clearCandidates;
            roadSpawnDiagnostics.frontZoneRejectedCandidates +=
                diagnostics.frontZoneRejectedCandidates;
            roadSpawnDiagnostics.lowerPriorityCandidates += diagnostics.lowerPriorityCandidates;
            if (!roadSpawnDiagnostics.firstInactiveCandidateTile) {
                roadSpawnDiagnostics.firstInactiveCandidateTile =
                    diagnostics.firstInactiveCandidateTile;
            }
        }
        std::size_t loadedRoadVehicles = 0;
        std::size_t pendingRoadVehicleLoads = 0;
        std::size_t failedRoadVehicleLoads = 0;
        std::size_t activeRoadVehicles = 0;
        TraceScope candidateTrace("traffic",
                                  "RenderLoop::updateTrafficDriving.buildPopulationPlan");
        if (!trafficFleet_.populationPlanPending) {
            const std::size_t firstVehicleCandidate = populationCandidates.size();
            for (std::size_t stateIndex = 0; stateIndex < trafficFleet_.vehicles.size();
                 ++stateIndex) {
                TrafficVehicleState& state = trafficFleet_.vehicles[stateIndex];
                if (!state.isBus()) {
                    loadedRoadVehicles += state.vehicle != nullptr;
                    pendingRoadVehicleLoads += state.pendingVehicle.valid() ||
                                               state.pendingInitializationVehicle != nullptr;
                    failedRoadVehicleLoads += state.vehicleLoadFailed;
                    activeRoadVehicles += state.populationEnabled;
                }
                if (state.vehicle == nullptr || state.vehicleLoadFailed ||
                    state.deadlockRemovalCommitted) {
                    continue;
                }
                populationCandidates.push_back({stateIndex, state.workerGeneration,
                                                state.actor.unscheduledGroupIndex, false,
                                                state.isBus(), state.populationEnabled});
            }
            const openbus::app::MapTrafficWorker::PopulationBalance populationBalance(
                populationCandidates, trafficFleet_.targetRoadTrafficCount,
                trafficFleet_.targetBusTrafficCount, targetHumanTrafficCount_,
                trafficFleet_.targetRoadGroupPopulations, trafficFleet_.useGroupRoadPopulation);
            for (std::size_t candidateIndex = firstVehicleCandidate;
                 candidateIndex < populationCandidates.size(); ++candidateIndex) {
                auto& candidate = populationCandidates[candidateIndex];
                TrafficVehicleState& state = trafficFleet_.vehicles[candidate.stateIndex];
                if (populationBalance.needsSpawn(candidate)) {
                    if (!state.isBus() || mapTrafficScheduledDepartureDue(
                                              currentSimulationTime,
                                              state.bus->nextScheduledDepartureElapsedSeconds)) {
                        candidate.spawnSearch =
                            openbus::app::MapTrafficWorker::PopulationSpawnSearch{
                                state.workerActorTemplate, state.spawnCandidates,
                                state.rejectedSpawnCandidateIndices,
                                state.actor.lateralOffsetMeters};
                    }
                } else if (!state.populationEnabled && !state.isBus() &&
                           trafficFleet_.trafficSpawnDiagnosticsEnabled) {
                    ++roadSpawnDiagnostics.vehiclesAtTarget;
                }
                candidate.canDespawn = populationBalance.needsDespawn(candidate) &&
                                       hidden(state.actor, state.actor.distanceMeters);
            }
            std::optional<openbus::app::MapTrafficWorker::PopulationSearchContext> searchContext;
            if (std::ranges::any_of(populationCandidates, [](const auto& candidate) {
                    return candidate.spawnSearch.has_value();
                })) {
                searchContext = makeSearchContext();
                searchContext->occupied = occupied;
            }
            trafficFleet_.worker->submitPopulationPlan(
                std::move(populationCandidates), trafficFleet_.targetRoadTrafficCount,
                trafficFleet_.targetBusTrafficCount, targetHumanTrafficCount_,
                trafficFleet_.targetRoadGroupPopulations, trafficFleet_.useGroupRoadPopulation,
                std::move(searchContext));
            trafficFleet_.populationPlanPending = true;
        }
        candidateTrace.finish();
        if (trafficFleet_.trafficSpawnDiagnosticsEnabled && trafficFleet_.populationPlanPending &&
            loadedRoadVehicles == 0 && pendingRoadVehicleLoads == 0 &&
            failedRoadVehicleLoads == 0 && activeRoadVehicles == 0) {
            for (const auto& state : trafficFleet_.vehicles) {
                if (!state.isBus()) {
                    loadedRoadVehicles += state.vehicle != nullptr;
                    pendingRoadVehicleLoads += state.pendingVehicle.valid() ||
                                               state.pendingInitializationVehicle != nullptr;
                    failedRoadVehicleLoads += state.vehicleLoadFailed;
                    activeRoadVehicles += state.populationEnabled;
                }
            }
        }
        if (trafficFleet_.trafficSpawnDiagnosticsEnabled) {
            trafficFleet_.trafficSpawnDiagnosticElapsedSeconds += trafficInputElapsedSeconds;
            if (trafficFleet_.trafficSpawnDiagnosticElapsedSeconds >= 1.0) {
                std::string activeTileCoordinates;
                for (const auto& [tileX, tileY] : activeTiles) {
                    if (!activeTileCoordinates.empty()) {
                        activeTileCoordinates += ';';
                    }
                    activeTileCoordinates += std::to_string(tileX) + ',' + std::to_string(tileY);
                }
                const std::string firstInactiveCandidateTile =
                    roadSpawnDiagnostics.firstInactiveCandidateTile
                        ? std::to_string(roadSpawnDiagnostics.firstInactiveCandidateTile->first) +
                              ',' +
                              std::to_string(
                                  roadSpawnDiagnostics.firstInactiveCandidateTile->second)
                        : std::string("none");
                gameLog.Debug(
                    "Map road AI spawn admission: target=" +
                    std::to_string(trafficFleet_.targetRoadTrafficCount) +
                    ", queued=" + std::to_string(trafficFleet_.vehicles.size()) +
                    ", loaded=" + std::to_string(loadedRoadVehicles) +
                    ", pendingLoads=" + std::to_string(pendingRoadVehicleLoads) +
                    ", failedLoads=" + std::to_string(failedRoadVehicleLoads) +
                    ", active=" + std::to_string(activeRoadVehicles) + ", recycledOutsideTiles=" +
                    std::to_string(trafficFleet_.roadTrafficRecycledOutsideTiles) +
                    ", localRouteAttempts=" +
                    std::to_string(trafficFleet_.roadRouteRefreshAttempts) + ", localRoutesBuilt=" +
                    std::to_string(trafficFleet_.roadRouteRefreshSuccesses) +
                    ", activeTiles=" + std::to_string(activeTiles.size()) +
                    ", checkedVehicles=" + std::to_string(roadSpawnDiagnostics.vehiclesChecked) +
                    ", vehiclesAtTarget=" + std::to_string(roadSpawnDiagnostics.vehiclesAtTarget) +
                    ", candidates=" + std::to_string(roadSpawnDiagnostics.candidatesChecked) +
                    ", rejected=" + std::to_string(roadSpawnDiagnostics.rejectedCandidates) +
                    ", inactiveCandidateTiles=" +
                    std::to_string(roadSpawnDiagnostics.inactiveCandidateTiles) +
                    ", posesOutsideActiveTiles=" +
                    std::to_string(roadSpawnDiagnostics.posesOutsideActiveTiles) +
                    ", frontZoneRejected=" +
                    std::to_string(roadSpawnDiagnostics.frontZoneRejectedCandidates) +
                    ", lowerPriority=" +
                    std::to_string(roadSpawnDiagnostics.lowerPriorityCandidates) +
                    ", firstInactiveCandidateTile=" + firstInactiveCandidateTile +
                    ", activeTileCoordinates=" + activeTileCoordinates +
                    ", visible=" + std::to_string(roadSpawnDiagnostics.visibleCandidates) +
                    ", obstructed=" + std::to_string(roadSpawnDiagnostics.obstructedCandidates) +
                    ", clear=" + std::to_string(roadSpawnDiagnostics.clearCandidates));
                trafficFleet_.trafficSpawnDiagnosticElapsedSeconds = 0.0;
                trafficFleet_.roadTrafficRecycledOutsideTiles = 0;
                trafficFleet_.roadRouteRefreshAttempts = 0;
                trafficFleet_.roadRouteRefreshSuccesses = 0;
            }
        }
    }
    std::vector<std::shared_ptr<MapTrafficSignalController>> signalControllers;
    std::vector<MapTrafficSignalRequest> signalRequests;
    if (mapRenderer_ != nullptr) {
        if (trafficInputDue) {
            TraceScope signalTrace("traffic", "RenderLoop::updateTrafficDriving.signalSnapshot");
            signalControllers = mapRenderer_->trafficSignalControllers();
            const auto playerPosition = streamingPosition(simulation);
            signalRequests = mapRenderer_->trafficSignalRequests(
                std::array<double, 2>{playerPosition[0], playerPosition[1]}, simulation.speed());
        }
        mapRenderer_->updateTrafficSignalScripts(currentSignalClockSeconds);
    }
    {
        TraceScope movementTrace("traffic", "RenderLoop::updateTrafficDriving.vehicleMovement");
        if (trafficInputDue) {
            std::vector<openbus::app::MapTrafficWorker::ActorInput> actors;
            std::vector<openbus::app::MapTrafficWorker::HumanInput> humans;
            std::vector<MapTrafficObstacle> humanObstacles = external;
            {
                TraceScope actorCollectionTrace(
                    "traffic", "RenderLoop::updateTrafficDriving.vehicleMovement.actorCollection");
                {
                    TraceScope vehicleInputTrace("traffic",
                                                 "RenderLoop::updateTrafficDriving.vehicleMovement."
                                                 "actorCollection.vehicles");
                    for (std::size_t index = 0; index < trafficFleet_.vehicles.size(); ++index) {
                        auto& state = trafficFleet_.vehicles[index];
                        if (state.populationEnabled && state.vehicle != nullptr &&
                            state.vehicle->mapTrafficTileActive) {
                            const auto& route = state.actor.route;
                            if (!route.closed && !route.loopStartDistance &&
                                state.actor.distanceMeters >= route.totalLength) {
                                state.actor.speedMetersPerSecond = 0.0;
                            } else {
                                openbus::app::MapTrafficWorker::ActorMotion motion;
                                motion.distanceMeters = state.actor.distanceMeters;
                                motion.speedMetersPerSecond = state.actor.speedMetersPerSecond;
                                motion.targetSpeedMetersPerSecond =
                                    state.actor.targetSpeedMetersPerSecond;
                                motion.lateralOffsetMeters = state.actor.lateralOffsetMeters;
                                motion.junctionWaitSeconds = state.actor.junctionWaitSeconds;
                                motion.scheduledStopDistanceMeters =
                                    state.actor.scheduledStopDistanceMeters;
                                motion.blockReason = state.actor.blockReason;
                                motion.blockingObstacleIndex = state.actor.blockingObstacleIndex;
                                motion.junctionClaimEndMeters = state.actor.junctionClaimEndMeters;
                                motion.committedSignalSegmentIndex =
                                    state.actor.committedSignalSegmentIndex;
                                const bool blockerGenerationMatches =
                                    state.actor.blockingActor != nullptr &&
                                    state.blockingActorStateIndex.has_value() &&
                                    state.blockingActorGeneration.has_value() &&
                                    *state.blockingActorStateIndex <
                                        trafficFleet_.vehicles.size() &&
                                    trafficFleet_.vehicles[*state.blockingActorStateIndex]
                                            .workerGeneration == *state.blockingActorGeneration;
                                constexpr double reducedDetailEnterDistanceMeters = 500.0;
                                constexpr double reducedDetailExitDistanceMeters = 400.0;
                                if (lastMainCameraPoseValid_ && state.cachedPose.has_value()) {
                                    const double cameraDistanceMeters =
                                        std::hypot(std::hypot(state.cachedPose->position[0] -
                                                                  lastMainCameraPose_.position[0],
                                                              state.cachedPose->position[1] -
                                                                  lastMainCameraPose_.position[1]),
                                                   state.cachedPose->position[2] -
                                                       lastMainCameraPose_.position[2]);
                                    if (state.reducedTrafficDetail) {
                                        state.reducedTrafficDetail =
                                            cameraDistanceMeters >= reducedDetailExitDistanceMeters;
                                    } else {
                                        state.reducedTrafficDetail =
                                            cameraDistanceMeters >=
                                            reducedDetailEnterDistanceMeters;
                                    }
                                } else {
                                    state.reducedTrafficDetail = false;
                                }
                                actors.push_back(
                                    {index, state.workerGeneration, state.workerActorTemplate,
                                     motion,
                                     !blockerGenerationMatches ? std::nullopt
                                                               : state.blockingActorStateIndex,
                                     !blockerGenerationMatches ? std::nullopt
                                                               : state.blockingActorGeneration,
                                     state.reducedTrafficDetail});
                            }
                        }
                    }
                }
                {
                    TraceScope obstacleInputTrace("traffic",
                                                  "RenderLoop::updateTrafficDriving."
                                                  "vehicleMovement.actorCollection.humanObstacles");
                    for (std::size_t index = 0; index < trafficFleet_.vehicles.size(); ++index) {
                        TrafficVehicleState& state = trafficFleet_.vehicles[index];
                        if (!state.populationEnabled || state.vehicle == nullptr ||
                            !state.vehicle->mapTrafficTileActive) {
                            continue;
                        }
                        const auto pose = trafficVehiclePose(state);
                        if (!pose) {
                            throw std::runtime_error(
                                "Cannot sample an active AI vehicle for pedestrian avoidance");
                        }
                        humanObstacles.push_back(
                            {*pose, state.actor.lengthMeters, state.actor.widthMeters});
                    }
                }
                {
                    TraceScope humanInputTrace(
                        "traffic",
                        "RenderLoop::updateTrafficDriving.vehicleMovement.actorCollection.humans");
                    for (std::size_t index = 0; index < humanTrafficActors_.size(); ++index) {
                        const HumanTrafficState& human = humanTrafficActors_[index];
                        humans.push_back({index, human.workerGeneration, human.workerActorTemplate,
                                          human.actor.distanceMeters,
                                          human.actor.speedMetersPerSecond,
                                          human.feetDistanceMeters, human.populationEnabled,
                                          human.advanceWhileInactive});
                    }
                }
            }
            {
                TraceScope submitTrace("traffic",
                                       "RenderLoop::updateTrafficDriving.vehicleMovement.submit");
                trafficFleet_.worker->submit(
                    std::move(actors), std::move(humans), external, std::move(humanObstacles),
                    std::move(signalControllers), std::move(signalRequests),
                    trafficInputElapsedSeconds, currentSignalClockSeconds);
            }
        }
    }
    {
        TraceScope commitTrace("traffic",
                               "RenderLoop::updateTrafficDriving.vehiclePlacementAndStuckCheck");
        for (TrafficVehicleState& trafficVehicle : trafficFleet_.vehicles) {
            if (!trafficVehicle.populationEnabled) {
                continue;
            }
            if (trafficVehicle.isBus() && trafficVehicle.vehicle != nullptr &&
                !trafficVehicle.bus->passengerStopArrived &&
                trafficVehicle.bus->nextPassengerStopIndex <
                    trafficVehicle.bus->passengerStops.size()) {
                const MapTrafficPassengerStop& stop =
                    trafficVehicle.bus->passengerStops[trafficVehicle.bus->nextPassengerStopIndex];
                if (trafficVehicle.actor.distanceMeters + 0.05 >= stop.routeDistanceMeters &&
                    trafficVehicle.actor.speedMetersPerSecond <=
                        MAP_TRAFFIC_STOPPED_SPEED_METERS_PER_SECOND) {
                    const auto queue = passengerNetwork_.queues.find(stop.stopId);
                    const std::size_t waitingPassengers =
                        queue == passengerNetwork_.queues.end() ? 0 : queue->second;
                    trafficVehicle.vehicle->passengers->service.setWaitingPassengers(
                        stop.stopId, waitingPassengers);
                    auto stopService = openbus::app::scheduledBusStopService(
                        *trafficVehicle.bus, trafficVehicle.vehicle->passengers->service);
                    const auto sampledBus = openbus::map::sampleMapTrafficActorPose(
                        trafficVehicle.actor, trafficVehicle.actor.distanceMeters);
                    const auto stopPose = openbus::map::sampleMapTrafficRoute(
                        trafficVehicle.actor.route, stop.routeDistanceMeters, 0.0);
                    if (!sampledBus || !stopPose || !mapRenderer_) {
                        throw std::runtime_error("Cannot locate scheduled bus alighting positions");
                    }
                    auto busPose = *sampledBus;
                    busPose.position[2] += trafficVehicle.vehicle->aiDeltaHeight;
                    stopService.alightingPositions = passengerAlightingPositions(
                        *trafficVehicle.vehicle, busPose, *stopPose, mapRenderer_->leftHandTraffic(),
                        trafficVehicle.actor.lateralOffsetMeters, trafficVehicle.actor.widthMeters);
                    if (!trafficVehicle.vehicle->passengers->service.arrive(
                            stopService)) {
                        throw std::logic_error(
                            "Scheduled bus reached a passenger stop while already dwelling");
                    }
                    if (trafficVehicle.vehicle->passengers->service
                            .outOfRangeAssignedPassengers() > 0) {
                        gameLog.Warning(
                            "Scheduled bus reached stop " + std::to_string(stop.stopId) +
                            " with assigned passenger destinations more than 200 m from their exits." +
                            (stopService.passengersAlighting > 0
                                 ? " Forced alighting at this stop can override the distance check."
                                 : " Those passengers remain aboard."));
                    }
                    trafficVehicle.vehicle->passengers->service.setEligibleWaitingPassengers(0);
                    trafficVehicle.bus->passengerStopArrived = true;
                    trafficVehicle.bus->passengerStopDepartureRequested = false;
                    auto& approach = passengerApproachProgress_[trafficVehicle.vehicle];
                    approach = {};
                    approach.stopId = stop.stopId;
                    trafficVehicle.vehicle->variables.set("ai_scheduled_atstation", 1.0F);
                    trafficVehicle.actor.scheduledStopDistanceMeters =
                        trafficVehicle.actor.distanceMeters;
                }
            }
            const auto& route = trafficVehicle.actor.route;
            const bool atRetirementTerminus =
                trafficVehicle.isBus() &&
                openbus::app::scheduledBusReachedRetirementTerminus(
                    *trafficVehicle.bus, trafficVehicle.actor.distanceMeters,
                    trafficVehicle.actor.speedMetersPerSecond,
                    MAP_TRAFFIC_STOPPED_SPEED_METERS_PER_SECOND);
            const bool atUnconnectedRouteEnd =
                !route.closed && !route.loopStartDistance &&
                trafficVehicle.actor.distanceMeters >= route.totalLength;
            const bool passengerServiceComplete =
                !trafficVehicle.isBus() ||
                (!trafficVehicle.vehicle->passengers->service.dwelling() &&
                 !trafficVehicle.bus->passengerStopArrived);
            if (passengerServiceComplete &&
                (atRetirementTerminus || (atUnconnectedRouteEnd && mapTrafficRouteEndReadyForUnload(
                                                                       trafficVehicle.actor)))) {
                if (trafficVehicle.isBus() && trafficVehicle.bus->scheduledTripActive) {
                    gameLog.Log("Scheduled map AI bus reached route end after travelling " +
                                std::to_string(trafficVehicle.actor.distanceMeters -
                                               trafficVehicle.spawnedAtDistanceMeters) +
                                "m for timetable minute=" +
                                std::to_string(trafficVehicle.bus->lastScheduledDepartureMinutes) +
                                ": " + trafficVehicle.actor.modelPath);
                }
                trafficVehicle.vehicle->mapTrafficRouteFinished = true;
                trafficVehicle.vehicle->mapTrafficTileActive = false;
                trafficVehicle.vehicle->variables.set("ai_blinker_l", 0.0);
                trafficVehicle.vehicle->variables.set("ai_blinker_r", 0.0);
                trafficVehicle.vehicle->variables.set("ai_brakelight", 0.0);
                trafficVehicle.populationEnabled = false;
                ++trafficVehicle.workerGeneration;
                finishScheduledTrip(trafficVehicle);
                trafficVehicle.stoppedSecondsSinceProgress = 0.0;
                trafficVehicle.deadlocked = false;
                trafficVehicle.stuckDiagnosticLogged = false;
                continue;
            }
            const auto pose = trafficVehiclePose(trafficVehicle);
            if (!pose) {
                throw std::runtime_error("Could not sample an active map traffic route pose");
            }
            std::array<double, 3> trafficPosition = pose->position;
            trafficPosition[2] += trafficVehicle.vehicle->aiDeltaHeight;
            trafficVehicle.vehicle->placement.position = trafficPosition;
            trafficVehicle.vehicle->placement.yawDegrees =
                openbus::map::mapTrafficHeadingToVehicleYawDegrees(
                    trafficVehicle.cachedTravelHeadingRadians);
            trafficVehicle.vehicle->placement.pitchDegrees =
                pose->pitchRadians * 180.0 / std::numbers::pi;
            trafficVehicle.vehicle->placement.rollDegrees =
                pose->rollRadians * 180.0 / std::numbers::pi;
            if (!trafficVehicle.vehicle->mapTrafficTileActive) {
                trafficVehicle.actor.speedMetersPerSecond = 0.0;
            }
            const double speedKmh = trafficVehicle.actor.speedMetersPerSecond * 3.6;
            trafficVehicle.vehicle->variables.set("velocity", speedKmh);
            trafficVehicle.vehicle->variables.set("velocity_ground", speedKmh);
            if (!trafficVehicle.wheelRouteDistanceInitialized) {
                trafficVehicle.previousWheelRouteDistanceMeters =
                    trafficVehicle.actor.distanceMeters;
                trafficVehicle.wheelRouteDistanceInitialized = true;
            } else {
                double distanceDelta = trafficVehicle.actor.distanceMeters -
                                       trafficVehicle.previousWheelRouteDistanceMeters;
                if (distanceDelta < 0.0) {
                    if (route.closed) {
                        distanceDelta += route.totalLength;
                    } else if (route.loopStartDistance.has_value()) {
                        distanceDelta += route.totalLength - *route.loopStartDistance;
                    } else {
                        distanceDelta = 0.0;
                    }
                }
                trafficVehicle.wheelTravelDistanceMeters += std::max(0.0, distanceDelta);
                trafficVehicle.previousWheelRouteDistanceMeters =
                    trafficVehicle.actor.distanceMeters;
            }
            const openbus::map::MapTrafficTurnSignal turnSignal =
                trafficVehicle.vehicle->mapTrafficTileActive
                    ? openbus::map::mapTrafficActorTurnSignal(trafficVehicle.actor)
                    : openbus::map::MapTrafficTurnSignal::None;
            const bool indicatorBlinkOn = std::fmod(currentSimulationTime, 1.0) < 0.5;
            trafficVehicle.vehicle->variables.set(
                "ai_blinker_l",
                turnSignal == openbus::map::MapTrafficTurnSignal::Left && indicatorBlinkOn ? 1.0
                                                                                           : 0.0);
            trafficVehicle.vehicle->variables.set(
                "ai_blinker_r",
                turnSignal == openbus::map::MapTrafficTurnSignal::Right && indicatorBlinkOn ? 1.0
                                                                                            : 0.0);
            trafficVehicle.vehicle->variables.set(
                "ai_brakelight", trafficVehicle.actor.targetSpeedMetersPerSecond + 0.05 <
                                         trafficVehicle.actor.speedMetersPerSecond
                                     ? 1.0
                                     : 0.0);
            if (!trafficVehicle.vehicle->mapTrafficTileActive || atUnconnectedRouteEnd) {
                trafficVehicle.lastProgressDistanceMeters = trafficVehicle.actor.distanceMeters;
                trafficVehicle.stoppedSecondsSinceProgress = 0.0;
                trafficVehicle.deadlocked = false;
                trafficVehicle.stuckDiagnosticLogged = false;
                continue;
            }
            const double distanceSinceProgress = std::abs(
                trafficVehicle.actor.distanceMeters - trafficVehicle.lastProgressDistanceMeters);
            const bool progressed = distanceSinceProgress >= MAP_TRAFFIC_PROGRESS_THRESHOLD_METERS;
            if (trafficVehicle.activeSpawnCandidateIndex.has_value() &&
                std::abs(trafficVehicle.actor.distanceMeters -
                         trafficVehicle.spawnedAtDistanceMeters) >=
                    MAP_TRAFFIC_PROGRESS_THRESHOLD_METERS) {
                trafficVehicle.movedSinceSpawn = true;
            }
            const bool legitimateWait = legitimateStoppedWait(trafficVehicle);
            if (progressed) {
                trafficVehicle.lastProgressDistanceMeters = trafficVehicle.actor.distanceMeters;
                trafficVehicle.stoppedSecondsSinceProgress = 0.0;
                trafficVehicle.deadlocked = false;
                trafficVehicle.stuckDiagnosticLogged = false;
            } else if (legitimateWait) {
                trafficVehicle.lastProgressDistanceMeters = trafficVehicle.actor.distanceMeters;
                trafficVehicle.stoppedSecondsSinceProgress = 0.0;
                trafficVehicle.deadlocked = false;
                trafficVehicle.stuckDiagnosticLogged = false;
            } else if (trafficVehicle.actor.speedMetersPerSecond <=
                       MAP_TRAFFIC_STOPPED_SPEED_METERS_PER_SECOND) {
                trafficVehicle.stoppedSecondsSinceProgress += elapsedSeconds;
                trafficVehicle.deadlocked =
                    trafficVehicle.stoppedSecondsSinceProgress >= MAP_TRAFFIC_STUCK_TIMEOUT_SECONDS;
                if (trafficVehicle.deadlocked) {
                    trafficVehicle.deadlockRemovalCommitted = true;
                }
            } else {
                trafficVehicle.lastProgressDistanceMeters = trafficVehicle.actor.distanceMeters;
                trafficVehicle.stoppedSecondsSinceProgress = 0.0;
                trafficVehicle.deadlocked = false;
                trafficVehicle.stuckDiagnosticLogged = false;
            }
            if (trafficVehicle.isBus() && trafficVehicle.bus->scheduledTripActive &&
                trafficVehicle.actor.distanceMeters -
                        trafficVehicle.bus->lastScheduledProgressDiagnosticDistanceMeters >=
                    100.0) {
                gameLog.Debug("Scheduled map AI bus route progress: timetable minute=" +
                              std::to_string(trafficVehicle.bus->lastScheduledDepartureMinutes) +
                              ", route distance=" +
                              std::to_string(trafficVehicle.actor.distanceMeters -
                                             trafficVehicle.spawnedAtDistanceMeters) +
                              "m, spline=" + std::to_string(pose->splineId) + ": " +
                              trafficVehicle.actor.modelPath);
                trafficVehicle.bus->lastScheduledProgressDiagnosticDistanceMeters =
                    trafficVehicle.actor.distanceMeters;
            }
            if (trafficVehicle.deadlocked && !trafficVehicle.stuckDiagnosticLogged) {
                const auto segmentEnd = std::lower_bound(route.cumulativeEndDistances.begin(),
                                                         route.cumulativeEndDistances.end(),
                                                         trafficVehicle.actor.distanceMeters);
                const std::size_t segmentIndex =
                    static_cast<std::size_t>(segmentEnd - route.cumulativeEndDistances.begin());
                const int splineId = segmentIndex < route.segments.size()
                                         ? route.segments[segmentIndex].spline.placement.splineId
                                         : pose->splineId;
                const double curveSpeedLimit =
                    mapTrafficCurveSpeedLimit(route, trafficVehicle.actor.distanceMeters,
                                              trafficVehicle.actor.cruiseSpeedMetersPerSecond,
                                              trafficVehicle.actor.minimumTurnRadiusMeters);
                const bool onJunctionPath = mapTrafficActorOnJunctionPath(
                    trafficVehicle.actor, trafficVehicle.actor.distanceMeters);
                std::string blocker = "none";
                const auto& blockingObstacleIndex = trafficVehicle.actor.blockingObstacleIndex;
                if (blockingObstacleIndex.has_value() && *blockingObstacleIndex < external.size()) {
                    const auto& obstacle = external[*blockingObstacleIndex];
                    blocker = std::string(playerObstacleIndex == blockingObstacleIndex ? "player"
                                          : *blockingObstacleIndex < externalVehicleObstacleCount
                                              ? "static vehicle"
                                              : "pedestrian") +
                              " at (" + std::to_string(obstacle.pose.position[0]) + ", " +
                              std::to_string(obstacle.pose.position[1]) + ")";
                } else if (const auto found =
                               trafficStateByActor.find(trafficVehicle.actor.blockingActor);
                           trafficVehicle.actor.blockingActor != nullptr &&
                           found != trafficStateByActor.end()) {
                    const MapTrafficActor& other = found->second->actor;
                    const auto otherPose = sampleMapTrafficActorPose(other, other.distanceMeters);
                    blocker =
                        other.modelPath +
                        (otherPose ? " at (" + std::to_string(otherPose->position[0]) + ", " +
                                         std::to_string(otherPose->position[1]) + ")"
                                   : std::string()) +
                        ", speed=" + std::to_string(other.speedMetersPerSecond) +
                        " m/s, its block reason=" + mapTrafficBlockReasonName(other.blockReason);
                }
                gameLog.Warning(
                    std::string("Map ") + (trafficVehicle.isBus() ? "AI bus" : "road AI vehicle") +
                    " appears stuck after " +
                    std::to_string(trafficVehicle.stoppedSecondsSinceProgress) +
                    " seconds without route progress and will be removed from active traffic: " +
                    trafficVehicle.actor.modelPath + ", spline=" + std::to_string(splineId) +
                    ", route distance=" + std::to_string(trafficVehicle.actor.distanceMeters) +
                    " m, junction path=" + (onJunctionPath ? "yes" : "no") +
                    ", curve speed limit=" + std::to_string(curveSpeedLimit) +
                    " m/s, junction wait=" +
                    std::to_string(trafficVehicle.actor.junctionWaitSeconds) + " s, blocked by=" +
                    openbus::map::mapTrafficBlockReasonName(trafficVehicle.actor.blockReason) +
                    ", scheduled stop=" +
                    (trafficVehicle.actor.scheduledStopDistanceMeters
                         ? std::to_string(*trafficVehicle.actor.scheduledStopDistanceMeters)
                         : "none") +
                    ", passenger stop arrived=" +
                    (trafficVehicle.bus && trafficVehicle.bus->passengerStopArrived ? "yes"
                                                                                    : "no") +
                    ", position=(" + std::to_string(pose->position[0]) + ", " +
                    std::to_string(pose->position[1]) + "), heading=" +
                    std::to_string(pose->headingRadians) + " rad, blocker=" + blocker + ".");
                trafficVehicle.stuckDiagnosticLogged = true;
            }
        }
    }
    for (TrafficVehicleState& state : trafficFleet_.vehicles) {
        if (!state.isBus() || state.vehicle == nullptr) {
            continue;
        }
        const double delaySeconds =
            openbus::app::scheduledBusDelaySeconds(*state.bus, state.actor, currentSimulationTime);
        state.vehicle->variables.set(
            "tt_delay", static_cast<float>(std::clamp(
                            delaySeconds, -static_cast<double>((std::numeric_limits<float>::max)()),
                            static_cast<double>((std::numeric_limits<float>::max)()))));
    }
}
