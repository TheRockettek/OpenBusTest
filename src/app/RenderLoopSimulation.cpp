// Script ticking and player/physics variable synchronisation.

#include "app/RenderLoopInternal.h"
#include "render/map/MapRenderer.h"

#include <numbers>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace {

constexpr double MAP_TRAFFIC_STUCK_TIMEOUT_SECONDS = 15;
constexpr double MAP_TRAFFIC_PROGRESS_THRESHOLD_METERS = 0.25;
constexpr double MAP_TRAFFIC_STOPPED_SPEED_METERS_PER_SECOND = 0.1;

double mapTrafficRouteCurvatureAtRearAxle(const openbus::map::MapTrafficActor& actor,
                                          double wheelbaseMeters, double rearAxleOffsetMeters) {
    const auto& route = actor.route;
    const double sampleSpan = std::max(1.0, wheelbaseMeters);
    const double rearAxleDistance = actor.distanceMeters + rearAxleOffsetMeters;
    double fromDistance = rearAxleDistance - sampleSpan * 0.5;
    double toDistance = rearAxleDistance + sampleSpan * 0.5;
    if (!route.closed && !route.loopStartDistance) {
        fromDistance = std::max(0.0, fromDistance);
        toDistance = std::min(route.totalLength, toDistance);
    }
    const double distanceSpan = toDistance - fromDistance;
    if (distanceSpan <= 1.0e-6) {
        return 0.0;
    }
    const double fromHeading = openbus::map::mapTrafficRouteTravelHeadingRadians(
        route, fromDistance, actor.lateralOffsetMeters);
    const double toHeading = openbus::map::mapTrafficRouteTravelHeadingRadians(
        route, toDistance, actor.lateralOffsetMeters);
    return std::remainder(toHeading - fromHeading, 2.0 * std::numbers::pi) / distanceSpan;
}

void logVariableSet(const std::string& scope, const Variables& variables) {
    std::vector<std::string> numericNames;
    numericNames.reserve(variables.numericValues().size());
    for (const auto& [name, value] : variables.numericValues()) {
        (void)value;
        numericNames.push_back(name);
    }
    std::sort(numericNames.begin(), numericNames.end());
    gameLog.Log(scope + " numeric variables: " + std::to_string(numericNames.size()));
    for (const std::string& name : numericNames) {
        std::ostringstream line;
        line << scope << '.' << name << " = " << std::setprecision(12)
             << variables.numericValues().at(name);
        gameLog.Log(line.str());
    }

    std::vector<std::string> stringNames;
    stringNames.reserve(variables.stringValues().size());
    for (const auto& [name, value] : variables.stringValues()) {
        (void)value;
        stringNames.push_back(name);
    }
    std::sort(stringNames.begin(), stringNames.end());
    gameLog.Log(scope + " string variables: " + std::to_string(stringNames.size()));
    for (const std::string& name : stringNames) {
        gameLog.Log(scope + "." + name + " = " + variables.stringValues().at(name));
    }
}

} // namespace

void RenderLoop::logDiagnosticVariables() const {
    gameLog.Log("Variable dump requested (F9)");
    if (playerVehicle_ != nullptr) {
        logVariableSet("vehicle", playerVehicle_->variables);
    } else {
        gameLog.Log("No player vehicle is available for the variable dump");
    }
    logVariableSet("system", simulationState_.sharedVariables());
}

void RenderLoop::logDrivetrainDiagnosticVariables() const {
    if (playerVehicle_ == nullptr) {
        return;
    }
    static constexpr const char* variableNames[] = {"throttle",
                                                    "brake",
                                                    "velocity",
                                                    "engine_on",
                                                    "engine_injection_on",
                                                    "engine_n",
                                                    "engine_internal_throttle",
                                                    "engine_throttle_injection",
                                                    "engine_m",
                                                    "engine_m_restriction",
                                                    "antrieb_getr_gangwahl",
                                                    "antrieb_getr_aktugang",
                                                    "antrieb_getr_ratio",
                                                    "antrieb_getr_m_an",
                                                    "antrieb_gear_engaged_timer",
                                                    "m_wheel",
                                                    "n_wheel",
                                                    "bremse_feststell",
                                                    "bremse_feststell_sw",
                                                    "bremse_luft",
                                                    "bremse_kompressor",
                                                    "bremse_p_leitung01",
                                                    "bremse_p_leitung02",
                                                    "bremse_p_tank01",
                                                    "bremse_p_tank02",
                                                    "bremse_p_tank03",
                                                    "bremse_ueventil_3",
                                                    "bremse_a_ueventil_3",
                                                    "bremse_a_ueventil_1",
                                                    "bremse_a_ueventil_2",
                                                    "bremse_zieldruck_fba",
                                                    "bremse_p_brzyl_fba",
                                                    "bremse_a_brventil_fba",
                                                    "bremse_p_brzyl_ha",
                                                    "bremse_zieldruck_ha",
                                                    "axle_brakeforce_0_l",
                                                    "axle_brakeforce_0_r",
                                                    "axle_brakeforce_1_l",
                                                    "axle_brakeforce_1_r"};
    std::ostringstream line;
    line << "Native drivetrain variables" << std::setprecision(7);
    for (const char* name : variableNames) {
        line << ' ' << name << '=' << playerVehicle_->variables.get(name);
    }
    gameLog.Log(line.str());
}

void RenderLoop::updatePlayerVariables(const BusSimulation& simulation, double throttle,
                                       double steering, double brake) {
    TraceScope trace("frame", "RenderLoop::updatePlayerVariables");
    soundViewpoint_ =
        isExteriorView() ? RenderViewContext::PlayerExterior : RenderViewContext::PlayerInterior;
    const double steeringSmoothingRate = (mouseControlEnabled_ ? steeringSmoothingRate_ : 0.0);
    smoothedSteering_ = openbus::input::smoothSteeringInput(
        smoothedSteering_, steering, std::clamp(frameTimeStep_, 0.0, 0.25), steeringSmoothingRate);
    if (playerVehicle_ != nullptr) {
        playerVehicle_->updateSimulationVariables(simulation, throttle, smoothedSteering_, brake);
    }
    if (mapRenderer_ != nullptr) {
        TraceScope trafficActorTrace("traffic", "MapRenderer::updateTrafficActors");
        mapRenderer_->updateTrafficActors(std::clamp(frameTimeStep_, 0.0, 0.25));
    }
    updateTrafficDriving(simulation, std::clamp(frameTimeStep_, 0.0, 0.25));
    const double trafficTimeStep = std::clamp(frameTimeStep_, 0.0, 0.25);
    std::vector<openbus::map::MapTrafficObstacle> vehicleObstacles;
    {
        TraceScope obstacleTrace("traffic",
                                 "RenderLoop::updatePlayerVariables.vehicleObstacleCollection");
        for (const auto& vehicle : vehicles_) {
            if (vehicle->followsMapTrafficRoute || vehicle->mapTrafficRouteFinished) {
                continue;
            }
            openbus::map::MapTrafficPose pose;
            if (vehicle.get() == playerVehicle_) {
                const auto chassis = simulation.chassisPose();
                pose.position = chassis.position;
                pose.headingRadians = std::atan2(chassis.rotation[0], chassis.rotation[3]);
            } else {
                pose.position = vehicle->placement.position;
                pose.headingRadians =
                    (90.0 - vehicle->placement.yawDegrees) * std::numbers::pi / 180.0;
            }
            vehicleObstacles.push_back(
                {pose, vehicle->trafficLengthMeters, vehicle->trafficWidthMeters});
        }
        for (const TrafficVehicleState& trafficVehicle : trafficVehicles_) {
            if (!trafficVehicle.populationEnabled || trafficVehicle.vehicle == nullptr ||
                !trafficVehicle.vehicle->mapTrafficTileActive) {
                continue;
            }
            const auto pose = openbus::map::sampleMapTrafficRoute(
                trafficVehicle.actor.route, trafficVehicle.actor.distanceMeters,
                trafficVehicle.actor.lateralOffsetMeters);
            if (!pose) {
                throw std::runtime_error(
                    "Cannot sample an active AI vehicle for pedestrian avoidance");
            }
            vehicleObstacles.push_back(
                {*pose, trafficVehicle.actor.lengthMeters, trafficVehicle.actor.widthMeters});
        }
    }
    {
        TraceScope pedestrianTrace("traffic",
                                   "RenderLoop::updatePlayerVariables.pedestrianAvoidance");
        for (std::size_t index = 0; index < humanTrafficActors_.size(); ++index) {
            HumanTrafficState& human = humanTrafficActors_[index];
            if (!human.populationEnabled) {
                continue;
            }
            const double previousDistanceMeters = human.actor.distanceMeters;
            openbus::map::advanceMapTrafficActorLoopingAvoidingObstacles(
                human.actor, vehicleObstacles, human.feetDistanceMeters * 1.25,
                human.feetDistanceMeters * 2.1, trafficTimeStep,
                std::max(0.0, simulation.simulationTime() - trafficTimeStep));
            double distanceMovedMeters = human.actor.distanceMeters - previousDistanceMeters;
            if (distanceMovedMeters < 0.0 && human.actor.route.closed) {
                const double loopStart = human.actor.route.loopStartDistance.value_or(0.0);
                distanceMovedMeters += human.actor.route.totalLength - loopStart;
            }
            openbus::rendering::advanceHumanGait(human.gait, std::max(0.0, distanceMovedMeters),
                                                 trafficTimeStep, human.walkStrideLengthMeters);
        }
    }
    {
        const double passengerServiceTimeStep = std::clamp(frameTimeStep_, 0.0, 0.25);
        std::unordered_map<Vehicle*, int> sharedQueueStops;
        for (const TrafficVehicleState& state : trafficVehicles_) {
            if (state.vehicle != nullptr && state.passengerStopArrived) {
                sharedQueueStops.emplace(state.vehicle,
                                         state.vehicle->passengerService.currentStopId());
            }
        }
        if (playerVehicle_ != nullptr && activePlayerPassengerStop_) {
            sharedQueueStops.emplace(playerVehicle_, *activePlayerPassengerStop_);
        }
        std::vector<int> passengerServiceStopIds;
        passengerServiceStopIds.reserve(vehicles_.size());
        for (std::size_t vehicleIndex = 0; vehicleIndex < vehicles_.size(); ++vehicleIndex) {
            const auto activeStop = sharedQueueStops.find(vehicles_[vehicleIndex].get());
            passengerServiceStopIds.push_back(
                activeStop == sharedQueueStops.end() ? 0 : activeStop->second);
        }
        const std::vector<std::size_t> passengerServiceUpdateOrder =
            openbus::map::mapPassengerServiceUpdateOrder(passengerServiceStopIds,
                                                         passengerServiceUpdateOffset_);
        for (const std::size_t vehicleIndex : passengerServiceUpdateOrder) {
            const std::unique_ptr<Vehicle>& vehicle = vehicles_[vehicleIndex];
            const auto activeStop = sharedQueueStops.find(vehicle.get());
            auto queue = activeStop != sharedQueueStops.end()
                             ? scheduledPassengerQueues_.find(activeStop->second)
                             : scheduledPassengerQueues_.end();
            if (queue != scheduledPassengerQueues_.end()) {
                advanceMapPassengerServiceWithSharedQueue(vehicle->passengerService,
                                                          activeStop->second, queue->second,
                                                          passengerServiceTimeStep);
            } else {
                vehicle->passengerService.advance(passengerServiceTimeStep);
            }
        }
        if (std::any_of(passengerServiceStopIds.begin(), passengerServiceStopIds.end(),
                        [](int stopId) { return stopId != 0; })) {
            ++passengerServiceUpdateOffset_;
        }
    }
    updateScripts();
    for (TrafficVehicleState& trafficVehicle : trafficVehicles_) {
        if (!trafficVehicle.populationEnabled || trafficVehicle.vehicle == nullptr) {
            continue;
        }
        const auto rearAxle = std::min_element(trafficVehicle.vehicle->trafficAxles.begin(),
                                               trafficVehicle.vehicle->trafficAxles.end(),
                                               [](const BusAxle& left, const BusAxle& right) {
                                                   return left.position < right.position;
                                               });
        const double rearAxleOffsetMeters =
            rearAxle == trafficVehicle.vehicle->trafficAxles.end() ? 0.0 : rearAxle->position;
        trafficVehicle.vehicle->updateTrafficWheelAnimations(
            trafficVehicle.wheelTravelDistanceMeters,
            mapTrafficRouteCurvatureAtRearAxle(trafficVehicle.actor,
                                               trafficVehicle.vehicle->trafficWheelbaseMeters,
                                               rearAxleOffsetMeters),
            trafficVehicle.actor.speedMetersPerSecond);
    }
    if (drivetrainDiagnosticsEnabled_) {
        const double now = glfwGetTime();
        if (lastDrivetrainDiagnosticTime_ < 0.0 || now - lastDrivetrainDiagnosticTime_ >= 1.0) {
            lastDrivetrainDiagnosticTime_ = now;
            logDrivetrainDiagnosticVariables();
        }
    }
}

void RenderLoop::finishPendingTrafficVehicleLoads() {
    TraceScope trace("traffic", "RenderLoop::finishPendingTrafficVehicleLoads");
    for (TrafficVehicleState& state : trafficVehicles_) {
        if (state.vehicle != nullptr || state.vehicleLoadFailed || !state.pendingVehicle.valid() ||
            state.pendingVehicle.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            continue;
        }
        try {
            std::unique_ptr<Vehicle> vehicle = state.pendingVehicle.get();
            if (!vehicle) {
                throw std::runtime_error("Traffic vehicle loader returned no vehicle");
            }
            vehicle->variables.setString("tt_line_string", state.timetableLineName);
            vehicle->variables.setString("tt_busstop_name", state.timetableDestinationName);
            vehicle->finishDeferredTrafficInitialization();
            vehicle->followsMapTrafficRoute = true;
            vehicle->mapTrafficRouteFinished = true;
            vehicle->mapTrafficTileActive = false;
            Vehicle* const loadedVehicle = vehicle.get();
            vehicles_.push_back(std::move(vehicle));
            state.vehicle = loadedVehicle;
        } catch (const std::exception& error) {
            state.vehicleLoadFailed = true;
            gameLog.Log("Could not asynchronously load map traffic vehicle '" +
                        state.actor.modelPath + "': " + error.what());
        }
    }
}

void RenderLoop::updateTrafficDriving(const BusSimulation& simulation, double elapsedSeconds) {
    TraceScope trace("traffic", "RenderLoop::updateTrafficDriving");
    using namespace openbus::map;
    finishPendingTrafficVehicleLoads();
    const auto poseOnActiveTile = [&](const MapTrafficPose& pose) {
        return mapRenderer_ != nullptr &&
               mapTrafficPoseOnActiveTile(pose, mapRenderer_->activeTileCoordinates());
    };
    {
        TraceScope tileTrace("traffic", "RenderLoop::updateTrafficDriving.activeTileSync");
        for (TrafficVehicleState& state : trafficVehicles_) {
            if (state.vehicle == nullptr) {
                continue;
            }
            if (!state.populationEnabled) {
                state.vehicle->mapTrafficTileActive = false;
                state.vehicle->variables.set("ai_blinker_l", 0.0);
                state.vehicle->variables.set("ai_blinker_r", 0.0);
                state.vehicle->variables.set("ai_brakelight", 0.0);
                state.lastProgressDistanceMeters = state.actor.distanceMeters;
                state.stoppedSecondsSinceProgress = 0.0;
                state.deadlocked = false;
                state.stuckDiagnosticLogged = false;
                continue;
            }
            const auto pose = sampleMapTrafficRoute(state.actor.route, state.actor.distanceMeters,
                                                    state.actor.lateralOffsetMeters);
            if (!pose) {
                throw std::runtime_error("Could not sample a map traffic tile pose");
            }
            state.vehicle->mapTrafficTileActive = poseOnActiveTile(*pose);
            if (!state.vehicle->mapTrafficTileActive) {
                state.actor.speedMetersPerSecond = 0.0;
                state.lastProgressDistanceMeters = state.actor.distanceMeters;
                state.stoppedSecondsSinceProgress = 0.0;
                state.deadlocked = false;
                state.stuckDiagnosticLogged = false;
            }
        }
    }
    const double currentSimulationTime = simulation.simulationTime();
    const auto advanceScheduledDeparture = [&](TrafficVehicleState& state,
                                               double afterElapsedSeconds) {
        state.nextScheduledDepartureIndex =
            (state.nextScheduledDepartureIndex + 1) % state.scheduledDepartureMinutes.size();
        state.nextScheduledDepartureElapsedSeconds = mapTrafficNextDepartureElapsedSeconds(
            mapTrafficStartHour_, afterElapsedSeconds + 1.0e-6,
            state.scheduledDepartureMinutes[state.nextScheduledDepartureIndex]);
    };
    const auto finishScheduledTrip = [&](TrafficVehicleState& state) {
        if (!state.isBus || !state.scheduledTripActive) {
            return;
        }
        state.scheduledTripActive = false;
        while (mapTrafficScheduledDepartureDue(currentSimulationTime,
                                               state.nextScheduledDepartureElapsedSeconds)) {
            advanceScheduledDeparture(state, currentSimulationTime);
        }
    };
    for (TrafficVehicleState& state : trafficVehicles_) {
        if (!state.isBus || state.vehicle == nullptr || !state.populationEnabled) {
            continue;
        }
        if (state.passengerStopArrived) {
            const int completedStopId = state.vehicle->passengerService.currentStopId();
            const auto queue = scheduledPassengerQueues_.find(completedStopId);
            if (queue != scheduledPassengerQueues_.end()) {
                queue->second = state.vehicle->passengerService.waitingPassengers(completedStopId);
            }
        }
        if (state.passengerStopArrived && !state.vehicle->passengerService.dwelling()) {
            state.passengerStopArrived = false;
            ++state.nextPassengerStopIndex;
        }
        if (state.passengerStopArrived) {
            state.actor.scheduledStopDistanceMeters = state.actor.distanceMeters;
        } else if (state.nextPassengerStopIndex < state.passengerStops.size()) {
            state.actor.scheduledStopDistanceMeters =
                state.passengerStops[state.nextPassengerStopIndex].routeDistanceMeters;
        } else {
            state.actor.scheduledStopDistanceMeters.reset();
        }
    }
    std::vector<MapTrafficObstacle> external;
    std::size_t externalVehicleObstacleCount = 0;
    std::optional<std::size_t> playerBusObstacleIndex;
    {
        TraceScope obstacleTrace("traffic",
                                 "RenderLoop::updateTrafficDriving.externalObstacleCollection");
        for (const auto& vehicle : vehicles_) {
            if (vehicle->followsMapTrafficRoute || vehicle->mapTrafficRouteFinished) {
                continue;
            }
            MapTrafficPose pose;
            if (vehicle.get() == playerVehicle_) {
                const auto chassis = simulation.chassisPose();
                pose.position = chassis.position;
                pose.headingRadians = std::atan2(chassis.rotation[0], chassis.rotation[3]);
            } else {
                pose.position = vehicle->placement.position;
                pose.headingRadians =
                    (90.0 - vehicle->placement.yawDegrees) * std::numbers::pi / 180.0;
            }
            external.push_back({pose, vehicle->trafficLengthMeters, vehicle->trafficWidthMeters});
            if (vehicle.get() == playerVehicle_) {
                playerBusObstacleIndex = external.size() - 1;
            }
        }
        externalVehicleObstacleCount = external.size();
    }
    std::unordered_map<const MapTrafficActor*, const TrafficVehicleState*> trafficStateByActor;
    for (const TrafficVehicleState& state : trafficVehicles_) {
        trafficStateByActor.emplace(&state.actor, &state);
    }
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
            if (mapTrafficActorWaitingAtSignal(actor, currentSimulationTime) ||
                (actor.blockingObstacleIndex.has_value() &&
                 actor.blockingObstacleIndex == playerBusObstacleIndex)) {
                return true;
            }
            const bool stoppedBusAtRouteEnd =
                current != &waitingVehicle && current->isBus && !actor.route.closed &&
                !actor.route.loopStartDistance &&
                actor.distanceMeters >= actor.route.totalLength - 1.0e-6;
            if (stoppedBusAtRouteEnd) {
                return true;
            }
            const auto blocker = trafficStateByActor.find(actor.blockingActor);
            if (actor.blockingActor == nullptr || blocker == trafficStateByActor.end() ||
                !blocker->second->populationEnabled) {
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
        const auto pose = sampleMapTrafficRoute(actor.route, distance, 0.0);
        if (!pose) {
            throw std::runtime_error("Cannot sample a queued traffic entry");
        }
        return hiddenAtPose(actor, *pose);
    };
    {
        TraceScope admissionTrace("traffic",
                                  "RenderLoop::updateTrafficDriving.populationAndAdmission");
        std::vector<MapTrafficObstacle> occupied = external;
        std::set<std::pair<int, int>> activeTiles;
        if (mapRenderer_ != nullptr) {
            const auto& coordinates = mapRenderer_->activeTileCoordinates();
            activeTiles.insert(coordinates.begin(), coordinates.end());
        }
        std::size_t roadCount = 0;
        std::size_t busCount = 0;
        std::unordered_map<std::size_t, std::size_t> roadGroupCounts;
        for (auto& state : trafficVehicles_) {
            if (state.deadlockRemovalCommitted) {
                if (state.populationEnabled && state.vehicle != nullptr &&
                    legitimateStoppedWait(state)) {
                    state.deadlockRemovalCommitted = false;
                    state.lastProgressDistanceMeters = state.actor.distanceMeters;
                    state.stoppedSecondsSinceProgress = 0.0;
                    state.deadlocked = false;
                    state.stuckDiagnosticLogged = false;
                } else if (state.populationEnabled && state.vehicle != nullptr && !state.isBus &&
                           !state.movedSinceSpawn && state.activeSpawnCandidateIndex.has_value() &&
                           *state.activeSpawnCandidateIndex < state.spawnCandidates.size() &&
                           state.rejectedSpawnCandidateIndices.size() + 1 <
                               state.spawnCandidates.size() &&
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
                    state.deadlockRemovalCommitted = false;
                    state.stoppedSecondsSinceProgress = 0.0;
                    state.deadlocked = false;
                    state.stuckDiagnosticLogged = false;
                    const auto spawnPose =
                        sampleMapTrafficRoute(state.actor.route, state.spawnedAtDistanceMeters,
                                              state.actor.lateralOffsetMeters);
                    gameLog.Log("Map road AI vehicle made no route progress from spawn spline=" +
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
            if (state.isBus) {
                ++busCount;
            } else {
                ++roadCount;
                if (state.actor.unscheduledGroupIndex.has_value()) {
                    ++roadGroupCounts[*state.actor.unscheduledGroupIndex];
                }
            }
            if (!state.vehicle->mapTrafficTileActive) {
                continue;
            }
            const auto pose =
                sampleMapTrafficRoute(state.actor.route, state.actor.distanceMeters, 0.0);
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

        std::size_t humanCount = 0;
        for (const HumanTrafficState& human : humanTrafficActors_) {
            humanCount += human.populationEnabled ? 1U : 0U;
        }
        for (std::size_t index = humanTrafficActors_.size();
             index > 0 && humanCount > targetHumanTrafficCount_; --index) {
            HumanTrafficState& human = humanTrafficActors_[index - 1];
            if (human.populationEnabled && hidden(human.actor, human.actor.distanceMeters)) {
                human.populationEnabled = false;
                human.actor.speedMetersPerSecond = human.actor.cruiseSpeedMetersPerSecond;
                --humanCount;
                gameLog.Log("Map human traffic actor unloaded off-screen by population schedule: " +
                            human.actor.modelPath);
            }
        }

        for (const HumanTrafficState& human : humanTrafficActors_) {
            if (human.populationEnabled) {
                occupied.push_back(humanFootprint(human));
            }
        }
        for (HumanTrafficState& human : humanTrafficActors_) {
            if (human.populationEnabled || humanCount >= targetHumanTrafficCount_) {
                continue;
            }
            human.actor.speedMetersPerSecond = human.actor.cruiseSpeedMetersPerSecond;
            const double distance = human.actor.distanceMeters;
            if (!hidden(human.actor, distance)) {
                advanceMapTrafficActorLooping(human.actor, elapsedSeconds);
                continue;
            }
            const MapTrafficObstacle candidate = humanFootprint(human);
            if (!mapTrafficHasClearance(candidate, occupied)) {
                advanceMapTrafficActorLooping(human.actor, elapsedSeconds);
                continue;
            }
            human.populationEnabled = true;
            ++humanCount;
            occupied.push_back(candidate);
            gameLog.Log("Map human traffic actor spawned at a clear off-screen path: " +
                        human.actor.modelPath + ", spline=" +
                        std::to_string(candidate.pose.splineId) + ", camera distance=" +
                        std::to_string(std::hypot(
                            candidate.pose.position[0] - lastMainCameraPose_.position[0],
                            candidate.pose.position[1] - lastMainCameraPose_.position[1])) +
                        " m.");
        }
        for (const HumanTrafficState& human : humanTrafficActors_) {
            if (human.populationEnabled) {
                external.push_back(humanFootprint(human));
            }
        }

        struct RoadSpawnLocation {
            double distanceMeters = 0.0;
            MapTrafficPose pose;
            std::size_t candidateIndex = 0;
        };
        const auto findRoadSpawn =
            [&](const TrafficVehicleState& state) -> std::optional<RoadSpawnLocation> {
            const MapTrafficRoute& route = state.actor.route;
            for (std::size_t candidateIndex = 0; candidateIndex < state.spawnCandidates.size();
                 ++candidateIndex) {
                if (std::ranges::find(state.rejectedSpawnCandidateIndices, candidateIndex) !=
                    state.rejectedSpawnCandidateIndices.end()) {
                    continue;
                }
                const MapTrafficSpawnCandidate& spawn = state.spawnCandidates[candidateIndex];
                if (!activeTiles.contains({spawn.tileX, spawn.tileY})) {
                    continue;
                }
                const auto pose = sampleMapTrafficRoute(route, spawn.distanceMeters,
                                                        state.actor.lateralOffsetMeters);
                if (!pose) {
                    throw std::runtime_error("Could not sample a road traffic spawn position");
                }
                if (!poseOnActiveTile(*pose) ||
                    (!mapTrafficSpawnInView_ && !hiddenAtPose(state.actor, *pose))) {
                    continue;
                }
                const MapTrafficObstacle candidate{*pose, state.actor.lengthMeters,
                                                   state.actor.widthMeters};
                if (mapTrafficHasClearance(candidate, occupied, 3.0)) {
                    return RoadSpawnLocation{spawn.distanceMeters, *pose, candidateIndex};
                }
            }
            return std::nullopt;
        };

        for (auto& state : trafficVehicles_) {
            if (state.vehicle == nullptr || state.vehicleLoadFailed ||
                state.deadlockRemovalCommitted) {
                continue;
            }
            const bool countByGroup = !state.isBus && useGroupRoadPopulation_ &&
                                      state.actor.unscheduledGroupIndex.has_value();
            std::size_t* count = state.isBus ? &busCount
                                 : countByGroup
                                     ? &roadGroupCounts[*state.actor.unscheduledGroupIndex]
                                     : &roadCount;
            std::size_t target = state.isBus ? targetBusTrafficCount_ : targetRoadTrafficCount_;
            if (!state.isBus && useGroupRoadPopulation_) {
                target = 0;
                if (state.actor.unscheduledGroupIndex.has_value()) {
                    const auto groupPopulation = std::find_if(
                        targetRoadGroupPopulations_.begin(), targetRoadGroupPopulations_.end(),
                        [&](const MapAiRoadTrafficGroupPopulation& population) {
                            return population.groupIndex == *state.actor.unscheduledGroupIndex;
                        });
                    if (groupPopulation != targetRoadGroupPopulations_.end()) {
                        target = groupPopulation->populationCount;
                    }
                }
            }
            if (state.populationEnabled && *count > target &&
                hidden(state.actor, state.actor.distanceMeters)) {
                state.populationEnabled = false;
                state.vehicle->mapTrafficRouteFinished = true;
                finishScheduledTrip(state);
                state.deadlocked = false;
                --*count;
                if (countByGroup) {
                    --roadCount;
                }
            }
            if (state.populationEnabled || *count >= target) {
                continue;
            }
            std::optional<RoadSpawnLocation> spawn;
            if (state.isBus) {
                const auto entryPose = sampleMapTrafficRoute(state.actor.route, 0.0, 0.0);
                if (!entryPose) {
                    throw std::runtime_error("Could not sample a map traffic entry pose");
                }
                if (!mapTrafficScheduledDepartureDue(currentSimulationTime,
                                                     state.nextScheduledDepartureElapsedSeconds)) {
                    continue;
                }
                const bool entryHidden =
                    mapTrafficSpawnInView_ || hiddenAtPose(state.actor, *entryPose);
                const bool entryClear = canSpawnMapTrafficEntry(state.actor, true, occupied);
                const bool entryOnActiveTile = poseOnActiveTile(*entryPose);
                if (!entryHidden || !entryClear || !entryOnActiveTile) {
                    const unsigned blockerMask = (entryHidden ? 0u : 1u) | (entryClear ? 0u : 2u) |
                                                 (entryOnActiveTile ? 0u : 4u);
                    const double cameraDistanceMeters =
                        lastMainCameraPoseValid_
                            ? std::hypot(
                                  std::hypot(
                                      entryPose->position[0] - lastMainCameraPose_.position[0],
                                      entryPose->position[1] - lastMainCameraPose_.position[1]),
                                  entryPose->position[2] - lastMainCameraPose_.position[2])
                            : 0.0;
                    if (blockerMask != state.lastScheduledAdmissionDiagnosticBlockerMask ||
                        !lastMainCameraPoseValid_ ||
                        std::abs(cameraDistanceMeters -
                                 state.lastScheduledAdmissionDiagnosticCameraDistanceMeters) >=
                            50.0) {
                        gameLog.Log("Scheduled map AI bus departure waiting: spline=" +
                                    std::to_string(entryPose->splineId) +
                                    ", hidden=" + std::to_string(entryHidden) +
                                    ", clear=" + std::to_string(entryClear) +
                                    ", activeTile=" + std::to_string(entryOnActiveTile) +
                                    ", cameraValid=" + std::to_string(lastMainCameraPoseValid_) +
                                    ", cameraDistance=" + std::to_string(cameraDistanceMeters) +
                                    "m, simulationSeconds=" +
                                    std::to_string(currentSimulationTime) + ", departureSeconds=" +
                                    std::to_string(state.nextScheduledDepartureElapsedSeconds) +
                                    ": " + state.actor.modelPath);
                        state.lastScheduledAdmissionDiagnosticBlockerMask = blockerMask;
                        state.lastScheduledAdmissionDiagnosticCameraDistanceMeters =
                            cameraDistanceMeters;
                    }
                    continue;
                }
                spawn = RoadSpawnLocation{0.0, *entryPose, 0};
            } else {
                spawn = findRoadSpawn(state);
                if (!spawn.has_value()) {
                    continue;
                }
            }
            resetMapTrafficActorForEntry(state.actor);
            state.actor.distanceMeters = spawn->distanceMeters;
            state.actor.speedMetersPerSecond =
                mapTrafficSpawnSpeedMetersPerSecond(state.actor, state.actor.distanceMeters);
            state.wheelTravelDistanceMeters = 0.0;
            state.previousWheelRouteDistanceMeters = state.actor.distanceMeters;
            state.wheelRouteDistanceInitialized = true;
            state.populationEnabled = true;
            state.lastProgressDistanceMeters = state.actor.distanceMeters;
            state.spawnedAtDistanceMeters = state.actor.distanceMeters;
            state.lastScheduledProgressDiagnosticDistanceMeters = state.actor.distanceMeters;
            state.movedSinceSpawn = false;
            state.activeSpawnCandidateIndex =
                state.isBus ? std::nullopt : std::optional<std::size_t>(spawn->candidateIndex);
            state.stoppedSecondsSinceProgress = 0.0;
            state.deadlocked = false;
            state.stuckDiagnosticLogged = false;
            state.vehicle->mapTrafficRouteFinished = false;
            state.vehicle->mapTrafficTileActive = true;
            if (state.isBus) {
                state.nextPassengerStopIndex = 0;
                while (state.nextPassengerStopIndex < state.passengerStops.size() &&
                       state.passengerStops[state.nextPassengerStopIndex].routeDistanceMeters +
                               0.05 <
                           state.actor.distanceMeters) {
                    ++state.nextPassengerStopIndex;
                }
                state.passengerStopArrived = false;
                state.lastScheduledDepartureMinutes =
                    state.scheduledDepartureMinutes[state.nextScheduledDepartureIndex];
                state.scheduledTripActive = true;
                for (const MapTrafficPassengerStop& stop : state.passengerStops) {
                    const auto queue = scheduledPassengerQueues_.find(stop.stopId);
                    const auto initialQueue = scheduledPassengerInitialQueues_.find(stop.stopId);
                    if (queue == scheduledPassengerQueues_.end() ||
                        initialQueue == scheduledPassengerInitialQueues_.end()) {
                        throw std::logic_error(
                            "Scheduled bus passenger stop has no initialized waiting queue");
                    }
                    queue->second = mapScheduledPassengerQueueAfterDeparture(queue->second,
                                                                             initialQueue->second);
                }
                advanceScheduledDeparture(state, state.nextScheduledDepartureElapsedSeconds);
                gameLog.Log("Scheduled map AI bus departed from endpoint spline=" +
                            std::to_string(spawn->pose.splineId) + " at timetable minute=" +
                            std::to_string(state.lastScheduledDepartureMinutes) + ": " +
                            state.actor.modelPath);
            }
            occupied.push_back({spawn->pose, state.actor.lengthMeters, state.actor.widthMeters});
            ++*count;
            if (countByGroup) {
                ++roadCount;
            }
        }
    }
    if (mapRenderer_ != nullptr) {
        TraceScope signalTrace("traffic", "RenderLoop::updateTrafficDriving.signalControllers");
        std::vector<MapTrafficActor*> signalActors;
        signalActors.reserve(trafficVehicles_.size() + humanTrafficActors_.size());
        for (TrafficVehicleState& state : trafficVehicles_) {
            if (state.populationEnabled && state.vehicle != nullptr &&
                state.vehicle->mapTrafficTileActive) {
                signalActors.push_back(&state.actor);
            }
        }
        for (HumanTrafficState& human : humanTrafficActors_) {
            if (human.populationEnabled) {
                signalActors.push_back(&human.actor);
            }
        }
        const auto playerPosition = streamingPosition(simulation);
        mapRenderer_->updateTrafficSignalControllers(
            signalActors, elapsedSeconds, std::max(0.0, currentSimulationTime - elapsedSeconds),
            std::array<double, 2>{playerPosition[0], playerPosition[1]});
    }
    {
        TraceScope movementTrace("traffic", "RenderLoop::updateTrafficDriving.vehicleMovement");
        std::vector<MapTrafficActor*> actors;
        {
            TraceScope actorCollectionTrace(
                "traffic", "RenderLoop::updateTrafficDriving.vehicleMovement.actorCollection");
            for (auto& state : trafficVehicles_) {
                if (state.populationEnabled && state.vehicle != nullptr &&
                    state.vehicle->mapTrafficTileActive) {
                    const auto& route = state.actor.route;
                    if (!route.closed && !route.loopStartDistance &&
                        state.actor.distanceMeters >= route.totalLength) {
                        state.actor.speedMetersPerSecond = 0.0;
                    } else {
                        actors.push_back(&state.actor);
                    }
                }
            }
        }
        {
            TraceScope runtimeTrace(
                "traffic", "RenderLoop::updateTrafficDriving.vehicleMovement.runtimeAdvance");
            advanceMapTrafficDriving(actors, external, elapsedSeconds,
                                     std::max(0.0, currentSimulationTime - elapsedSeconds));
        }
    }
    {
        TraceScope commitTrace("traffic",
                               "RenderLoop::updateTrafficDriving.vehiclePlacementAndStuckCheck");
        for (TrafficVehicleState& trafficVehicle : trafficVehicles_) {
            if (!trafficVehicle.populationEnabled) {
                continue;
            }
            if (trafficVehicle.isBus && trafficVehicle.vehicle != nullptr &&
                !trafficVehicle.passengerStopArrived &&
                trafficVehicle.nextPassengerStopIndex < trafficVehicle.passengerStops.size()) {
                const MapTrafficPassengerStop& stop =
                    trafficVehicle.passengerStops[trafficVehicle.nextPassengerStopIndex];
                if (trafficVehicle.actor.distanceMeters + 0.05 >= stop.routeDistanceMeters &&
                    trafficVehicle.actor.speedMetersPerSecond <=
                        MAP_TRAFFIC_STOPPED_SPEED_METERS_PER_SECOND) {
                    const int nextStopId =
                        trafficVehicle.nextPassengerStopIndex + 1 <
                                trafficVehicle.passengerStops.size()
                            ? trafficVehicle
                                  .passengerStops[trafficVehicle.nextPassengerStopIndex + 1]
                                  .stopId
                            : 0;
                    const auto queue = scheduledPassengerQueues_.find(stop.stopId);
                    const std::size_t waitingPassengers =
                        queue == scheduledPassengerQueues_.end() || nextStopId == 0 ? 0
                                                                                    : queue->second;
                    trafficVehicle.vehicle->passengerService.setWaitingPassengers(
                        stop.stopId, waitingPassengers);
                    if (!trafficVehicle.vehicle->passengerService.arrive(
                            {.stopId = stop.stopId,
                             .waitingPassengers = 0,
                             .passengersAlighting = stop.passengersAlighting,
                             .dwellSeconds = stop.dwellSeconds,
                             .nextStopId = nextStopId})) {
                        throw std::logic_error(
                            "Scheduled bus reached a passenger stop while already dwelling");
                    }
                    trafficVehicle.passengerStopArrived = true;
                    trafficVehicle.actor.scheduledStopDistanceMeters =
                        trafficVehicle.actor.distanceMeters;
                }
            }
            const auto& route = trafficVehicle.actor.route;
            const bool atUnconnectedRouteEnd =
                !route.closed && !route.loopStartDistance &&
                trafficVehicle.actor.distanceMeters >= route.totalLength;
            const bool passengerServiceComplete =
                !trafficVehicle.isBus || !trafficVehicle.vehicle->passengerService.dwelling();
            if (atUnconnectedRouteEnd && passengerServiceComplete &&
                mapTrafficRouteEndReadyForUnload(trafficVehicle.actor)) {
                if (trafficVehicle.isBus && trafficVehicle.scheduledTripActive) {
                    gameLog.Log("Scheduled map AI bus reached route end after travelling " +
                                std::to_string(trafficVehicle.actor.distanceMeters -
                                               trafficVehicle.spawnedAtDistanceMeters) +
                                "m for timetable minute=" +
                                std::to_string(trafficVehicle.lastScheduledDepartureMinutes) +
                                ": " + trafficVehicle.actor.modelPath);
                }
                trafficVehicle.vehicle->mapTrafficRouteFinished = true;
                trafficVehicle.vehicle->mapTrafficTileActive = false;
                trafficVehicle.vehicle->variables.set("ai_blinker_l", 0.0);
                trafficVehicle.vehicle->variables.set("ai_blinker_r", 0.0);
                trafficVehicle.vehicle->variables.set("ai_brakelight", 0.0);
                trafficVehicle.populationEnabled = false;
                finishScheduledTrip(trafficVehicle);
                trafficVehicle.stoppedSecondsSinceProgress = 0.0;
                trafficVehicle.deadlocked = false;
                trafficVehicle.stuckDiagnosticLogged = false;
                continue;
            }
            const std::optional<openbus::map::MapTrafficPose> pose =
                openbus::map::sampleMapTrafficRoute(trafficVehicle.actor.route,
                                                    trafficVehicle.actor.distanceMeters,
                                                    trafficVehicle.actor.lateralOffsetMeters);
            if (!pose) {
                throw std::runtime_error("Could not sample an active map traffic route pose");
            }
            std::array<double, 3> trafficPosition = pose->position;
            trafficPosition[2] += trafficVehicle.vehicle->aiDeltaHeight;
            trafficVehicle.vehicle->placement.position = trafficPosition;
            trafficVehicle.vehicle->placement.yawDegrees =
                openbus::map::mapTrafficHeadingToVehicleYawDegrees(
                    openbus::map::mapTrafficRouteTravelHeadingRadians(
                        trafficVehicle.actor.route, trafficVehicle.actor.distanceMeters,
                        trafficVehicle.actor.lateralOffsetMeters));
            trafficVehicle.vehicle->placement.pitchDegrees =
                pose->pitchRadians * 180.0 / std::numbers::pi;
            trafficVehicle.vehicle->placement.rollDegrees =
                pose->rollRadians * 180.0 / std::numbers::pi;
            trafficVehicle.vehicle->mapTrafficTileActive = poseOnActiveTile(*pose);
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
                "ai_brakelight",
                trafficVehicle.actor.targetSpeedMetersPerSecond + 0.05 <
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
            if (trafficVehicle.isBus && trafficVehicle.scheduledTripActive &&
                trafficVehicle.actor.distanceMeters -
                        trafficVehicle.lastScheduledProgressDiagnosticDistanceMeters >=
                    100.0) {
                gameLog.Log("Scheduled map AI bus route progress: timetable minute=" +
                            std::to_string(trafficVehicle.lastScheduledDepartureMinutes) +
                            ", route distance=" +
                            std::to_string(trafficVehicle.actor.distanceMeters -
                                           trafficVehicle.spawnedAtDistanceMeters) +
                            "m, spline=" + std::to_string(pose->splineId) + ": " +
                            trafficVehicle.actor.modelPath);
                trafficVehicle.lastScheduledProgressDiagnosticDistanceMeters =
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
                    blocker =
                        std::string(playerBusObstacleIndex == blockingObstacleIndex ? "player bus"
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
                    const auto otherPose = sampleMapTrafficRoute(other.route, other.distanceMeters,
                                                                 other.lateralOffsetMeters);
                    blocker =
                        other.modelPath +
                        (otherPose ? " at (" + std::to_string(otherPose->position[0]) + ", " +
                                         std::to_string(otherPose->position[1]) + ")"
                                   : std::string()) +
                        ", speed=" + std::to_string(other.speedMetersPerSecond) +
                        " m/s, its block reason=" + mapTrafficBlockReasonName(other.blockReason);
                }
                gameLog.Log(
                    std::string("Map ") + (trafficVehicle.isBus ? "AI bus" : "road AI vehicle") +
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
                    ", position=(" + std::to_string(pose->position[0]) + ", " +
                    std::to_string(pose->position[1]) + "), heading=" +
                    std::to_string(pose->headingRadians) + " rad, blocker=" + blocker + ".");
                trafficVehicle.stuckDiagnosticLogged = true;
            }
        }
    }
}

void RenderLoop::updatePostPhysicsVariables(const BusSimulation& simulation) {
    simulationState_.sharedVariables().updateSimulationClock(simulation.simulationTime());
    if (playerVehicle_ != nullptr) {
        playerVehicle_->updateSimulationVariables(
            simulation, playerVehicle_->variables.get("throttle"),
            playerVehicle_->variables.get("steering"), playerVehicle_->variables.get("brake"));
        if (activePlayerPassengerStop_) {
            const auto queue = scheduledPassengerQueues_.find(*activePlayerPassengerStop_);
            if (queue != scheduledPassengerQueues_.end()) {
                queue->second =
                    playerVehicle_->passengerService.waitingPassengers(*activePlayerPassengerStop_);
            }
        }
        if (activePlayerPassengerStop_ && !playerVehicle_->passengerService.dwelling()) {
            activePlayerPassengerStop_.reset();
            activePlayerPassengerStopRouteIndex_.reset();
            activePlayerPassengerStopIndex_.reset();
        }

        struct NearbyPassengerStop {
            std::size_t routeIndex = 0;
            std::size_t stopIndex = 0;
            double distanceMeters = std::numeric_limits<double>::infinity();
        };
        NearbyPassengerStop nearestStop;
        const auto chassis = simulation.chassisPose();
        const double chassisHeading = std::atan2(chassis.rotation[0], chassis.rotation[3]);
        double latchedStopDistance = std::numeric_limits<double>::infinity();
        if (playerPassengerStopLatchRouteIndex_ && playerPassengerStopLatchIndex_ &&
            *playerPassengerStopLatchRouteIndex_ < passengerStopRoutes_.size()) {
            const PassengerStopRouteState& latchedRoute =
                passengerStopRoutes_[*playerPassengerStopLatchRouteIndex_];
            if (*playerPassengerStopLatchIndex_ < latchedRoute.stops.size()) {
                const auto latchedPose = openbus::map::sampleMapTrafficRoute(
                    latchedRoute.actor.route,
                    latchedRoute.stops[*playerPassengerStopLatchIndex_].routeDistanceMeters, 0.0);
                if (!latchedPose) {
                    throw std::runtime_error("Cannot sample the latched player passenger stop");
                }
                latchedStopDistance = std::hypot(latchedPose->position[0] - chassis.position[0],
                                                 latchedPose->position[1] - chassis.position[1]);
            }
        }
        for (std::size_t routeIndex = 0; routeIndex < passengerStopRoutes_.size(); ++routeIndex) {
            const PassengerStopRouteState& passengerRoute = passengerStopRoutes_[routeIndex];
            for (std::size_t stopIndex = 0; stopIndex < passengerRoute.stops.size(); ++stopIndex) {
                const openbus::map::MapTrafficPassengerStop& stop = passengerRoute.stops[stopIndex];
                const auto pose = openbus::map::sampleMapTrafficRoute(
                    passengerRoute.actor.route, stop.routeDistanceMeters, 0.0);
                if (!pose) {
                    throw std::runtime_error(
                        "Cannot sample a scheduled passenger stop for player service");
                }
                const double distance = std::hypot(pose->position[0] - chassis.position[0],
                                                   pose->position[1] - chassis.position[1]);
                const double headingDifference =
                    std::remainder(chassisHeading - pose->headingRadians, 2.0 * std::numbers::pi);
                if (std::abs(headingDifference) > std::numbers::pi / 3.0) {
                    continue;
                }
                if (distance < nearestStop.distanceMeters) {
                    nearestStop = {routeIndex, stopIndex, distance};
                }
            }
        }
        if (playerPassengerStopLatch_ && latchedStopDistance > 5.0) {
            const auto queue = scheduledPassengerQueues_.find(*playerPassengerStopLatch_);
            const auto initialQueue =
                scheduledPassengerInitialQueues_.find(*playerPassengerStopLatch_);
            if (queue == scheduledPassengerQueues_.end() ||
                initialQueue == scheduledPassengerInitialQueues_.end()) {
                throw std::logic_error(
                    "Player passenger stop latch has no initialized waiting queue");
            }
            queue->second = openbus::map::mapScheduledPassengerQueueAfterDeparture(
                queue->second, initialQueue->second);
            playerPassengerStopLatch_.reset();
            playerPassengerStopLatchRouteIndex_.reset();
            playerPassengerStopLatchIndex_.reset();
        }
        if (!activePlayerPassengerStop_ && !playerPassengerStopLatch_ &&
            nearestStop.distanceMeters <= 2.5 && std::abs(simulation.speed()) <= 0.35 &&
            playerVehicle_->passengerService.seatCount() > 0) {
            const openbus::map::MapTrafficPassengerStop& stop =
                passengerStopRoutes_[nearestStop.routeIndex].stops[nearestStop.stopIndex];
            const auto queue = scheduledPassengerQueues_.find(stop.stopId);
            if (queue != scheduledPassengerQueues_.end()) {
                const int nextStopId =
                    nearestStop.stopIndex + 1 <
                            passengerStopRoutes_[nearestStop.routeIndex].stops.size()
                        ? passengerStopRoutes_[nearestStop.routeIndex]
                              .stops[nearestStop.stopIndex + 1]
                              .stopId
                        : 0;
                playerVehicle_->passengerService.setWaitingPassengers(
                    stop.stopId, nextStopId == 0 ? 0 : queue->second);
                if (!playerVehicle_->passengerService.arrive(
                        {.stopId = stop.stopId,
                         .waitingPassengers = 0,
                         .passengersAlighting = stop.passengersAlighting,
                         .dwellSeconds = stop.dwellSeconds,
                         .nextStopId = nextStopId})) {
                    throw std::logic_error("Player bus reached a passenger stop while already "
                                           "dwelling");
                }
                activePlayerPassengerStop_ = stop.stopId;
                activePlayerPassengerStopRouteIndex_ = nearestStop.routeIndex;
                activePlayerPassengerStopIndex_ = nearestStop.stopIndex;
                playerPassengerStopLatch_ = stop.stopId;
                playerPassengerStopLatchRouteIndex_ = nearestStop.routeIndex;
                playerPassengerStopLatchIndex_ = nearestStop.stopIndex;
            }
        }
        playerVehicle_->soundBank.updateAmbient(soundEngine_, playerVehicle_->variables,
                                                soundViewpoint_);
    }
}

void RenderLoop::updateScripts() {
    TraceScope trace("script", "RenderLoop::updateScripts");
    const auto updateAllScripts = [&] {
        for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
            if (!vehicle->mapTrafficRouteFinished &&
                (!vehicle->followsMapTrafficRoute || vehicle->mapTrafficTileActive)) {
                vehicle->updateScripts(vehicle.get() != playerVehicle_);
            }
        }
        if (mapRenderer_ != nullptr) {
            mapRenderer_->updateScripts();
        }
    };
    if (scriptRateHz_ <= 0.0) {
        simulationState_.sharedVariables().set(
            "timegap", std::clamp(frameTimeStep_, 0.0, MAX_SCRIPT_FRAME_DELTA_SECONDS));
        updateAllScripts();
        return;
    }

    const double scriptTimeStep = 1.0 / scriptRateHz_;
    const ScriptTickSchedule schedule =
        scheduleScriptTicks(scriptAccumulator_, frameTimeStep_, scriptTimeStep);
    scriptAccumulator_ = schedule.remainderSeconds;
    scriptTicksSinceDiagnostic_ += schedule.ticks;
    droppedScriptTicksSinceDiagnostic_ += schedule.droppedTicks;
    droppedScriptTimeSinceDiagnostic_ += schedule.droppedSeconds;
    for (int tick = 0; tick < schedule.ticks; ++tick) {
        simulationState_.sharedVariables().set("timegap", scriptTimeStep);
        updateAllScripts();
    }

    const double diagnosticTime = glfwGetTime();
    if (lastScriptDiagnosticTime_ < 0.0) {
        lastScriptDiagnosticTime_ = diagnosticTime;
    } else if (diagnosticTime - lastScriptDiagnosticTime_ >= 1.0) {
        if (droppedScriptTicksSinceDiagnostic_ > 0) {
            gameLog.Log("Script scheduler executed " + std::to_string(scriptTicksSinceDiagnostic_) +
                        " ticks and dropped " + std::to_string(droppedScriptTicksSinceDiagnostic_) +
                        " ticks (" + std::to_string(droppedScriptTimeSinceDiagnostic_) +
                        " s) during the last second");
        }
        lastScriptDiagnosticTime_ = diagnosticTime;
        scriptTicksSinceDiagnostic_ = 0;
        droppedScriptTicksSinceDiagnostic_ = 0;
        droppedScriptTimeSinceDiagnostic_ = 0.0;
    }
    // Fixed script ticks must not leak their timestep into later mouse handlers.
    simulationState_.sharedVariables().set("timegap", frameTimeStep_);
}
