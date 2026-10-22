// Script ticking and player/physics variable synchronisation.

#include "app/RenderLoopInternal.h"
#include "render/map/MapRenderer.h"

#include <numbers>

namespace {

constexpr double MAP_TRAFFIC_STUCK_TIMEOUT_SECONDS = 45.0;
constexpr double MAP_TRAFFIC_PROGRESS_THRESHOLD_METERS = 0.25;
constexpr double MAP_TRAFFIC_STOPPED_SPEED_METERS_PER_SECOND = 0.1;

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
    const auto humanFootprint = [](const HumanTrafficState& human) {
        const auto pose = openbus::map::sampleMapTrafficRoute(
            human.actor.route, human.actor.distanceMeters, human.actor.lateralOffsetMeters);
        if (!pose) {
            throw std::runtime_error("Cannot sample an active human traffic obstacle");
        }
        return openbus::map::MapTrafficObstacle{*pose, human.widthMeters * 1.25,
                                                human.widthMeters * 2.1};
    };
    {
        TraceScope pedestrianTrace("traffic",
                                   "RenderLoop::updatePlayerVariables.pedestrianAvoidance");
        for (std::size_t index = 0; index < humanTrafficActors_.size(); ++index) {
            HumanTrafficState& human = humanTrafficActors_[index];
            if (!human.populationEnabled) {
                continue;
            }
            std::vector<openbus::map::MapTrafficObstacle> obstacles = vehicleObstacles;
            for (std::size_t otherIndex = 0; otherIndex < humanTrafficActors_.size();
                 ++otherIndex) {
                if (otherIndex != index && humanTrafficActors_[otherIndex].populationEnabled) {
                    obstacles.push_back(humanFootprint(humanTrafficActors_[otherIndex]));
                }
            }
            openbus::map::advanceMapTrafficActorLoopingAvoidingObstacles(
                human.actor, obstacles, human.widthMeters * 1.25, human.widthMeters * 2.1,
                trafficTimeStep, std::max(0.0, simulation.simulationTime() - trafficTimeStep));
        }
    }
    updateScripts();
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
                state.lastProgressDistanceMeters = state.actor.distanceMeters;
                state.stoppedSecondsSinceProgress = 0.0;
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
    std::vector<MapTrafficObstacle> external;
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
        }
    }
    const auto hidden = [&](const MapTrafficActor& actor, double distance) {
        if (!lastMainCameraPoseValid_) {
            return false;
        }
        const auto pose = sampleMapTrafficRoute(actor.route, distance, 0.0);
        if (!pose) {
            throw std::runtime_error("Cannot sample a queued traffic entry");
        }
        // Pad the sphere so queued traffic never pops in at the screen edge.
        const double radius = std::hypot(actor.lengthMeters, actor.widthMeters) * 0.5 + 2.0;
        return mapTrafficEntryHiddenFromCamera(pose->position, radius, lastMainCameraPose_.position,
                                               trafficCameraViewProjection_, 50.0);
    };
    {
        TraceScope admissionTrace("traffic",
                                  "RenderLoop::updateTrafficDriving.populationAndAdmission");
        std::vector<MapTrafficObstacle> occupied = external;
        std::size_t roadCount = 0;
        std::size_t busCount = 0;
        std::unordered_map<std::size_t, std::size_t> roadGroupCounts;
        for (const auto& state : trafficVehicles_) {
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
            return MapTrafficObstacle{*pose, human.widthMeters * 1.25, human.widthMeters * 2.1};
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

        for (auto& state : trafficVehicles_) {
            if (state.vehicle == nullptr || state.vehicleLoadFailed) {
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
                --*count;
                if (countByGroup) {
                    --roadCount;
                }
            }
            if (state.populationEnabled || *count >= target || !hidden(state.actor, 0.0) ||
                !canSpawnMapTrafficEntry(state.actor, true, occupied)) {
                continue;
            }
            const auto entryPose = sampleMapTrafficRoute(state.actor.route, 0.0, 0.0);
            if (!entryPose) {
                throw std::runtime_error("Could not sample a map traffic entry pose");
            }
            if (!poseOnActiveTile(*entryPose)) {
                continue;
            }
            if (state.isBus &&
                !mapTrafficScheduledDepartureDue(currentSimulationTime,
                                                 state.nextScheduledDepartureElapsedSeconds)) {
                continue;
            }
            resetMapTrafficActorForEntry(state.actor);
            state.populationEnabled = true;
            state.lastProgressDistanceMeters = state.actor.distanceMeters;
            state.stoppedSecondsSinceProgress = 0.0;
            state.stuckDiagnosticLogged = false;
            state.vehicle->mapTrafficRouteFinished = false;
            state.vehicle->mapTrafficTileActive = true;
            if (state.isBus) {
                state.lastScheduledDepartureMinutes =
                    state.scheduledDepartureMinutes[state.nextScheduledDepartureIndex];
                state.scheduledTripActive = true;
                advanceScheduledDeparture(state, state.nextScheduledDepartureElapsedSeconds);
            }
            const auto pose = sampleMapTrafficRoute(state.actor.route, 0.0, 0.0);
            occupied.push_back({*pose, state.actor.lengthMeters, state.actor.widthMeters});
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
        mapRenderer_->updateTrafficSignalControllers(
            signalActors, elapsedSeconds, std::max(0.0, currentSimulationTime - elapsedSeconds));
    }
    {
        TraceScope movementTrace("traffic", "RenderLoop::updateTrafficDriving.vehicleMovement");
        std::vector<MapTrafficActor*> actors;
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
        advanceMapTrafficDriving(actors, external, elapsedSeconds,
                                 std::max(0.0, currentSimulationTime - elapsedSeconds));
    }
    {
        TraceScope commitTrace("traffic",
                               "RenderLoop::updateTrafficDriving.vehiclePlacementAndStuckCheck");
        for (TrafficVehicleState& trafficVehicle : trafficVehicles_) {
            if (!trafficVehicle.populationEnabled) {
                continue;
            }
            const auto& route = trafficVehicle.actor.route;
            const bool atUnconnectedRouteEnd =
                !route.closed && !route.loopStartDistance &&
                trafficVehicle.actor.distanceMeters >= route.totalLength;
            if (atUnconnectedRouteEnd && mapTrafficRouteEndReadyForUnload(trafficVehicle.actor)) {
                trafficVehicle.vehicle->mapTrafficRouteFinished = true;
                trafficVehicle.vehicle->mapTrafficTileActive = false;
                trafficVehicle.populationEnabled = false;
                finishScheduledTrip(trafficVehicle);
                trafficVehicle.stoppedSecondsSinceProgress = 0.0;
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
                openbus::map::mapTrafficHeadingToVehicleYawDegrees(pose->headingRadians);
            trafficVehicle.vehicle->mapTrafficTileActive = poseOnActiveTile(*pose);
            if (!trafficVehicle.vehicle->mapTrafficTileActive) {
                trafficVehicle.actor.speedMetersPerSecond = 0.0;
            }
            const double speedKmh = trafficVehicle.actor.speedMetersPerSecond * 3.6;
            trafficVehicle.vehicle->variables.set("velocity", speedKmh);
            trafficVehicle.vehicle->variables.set("velocity_ground", speedKmh);
            const double distanceSinceProgress = std::abs(
                trafficVehicle.actor.distanceMeters - trafficVehicle.lastProgressDistanceMeters);
            const bool progressed = distanceSinceProgress >= MAP_TRAFFIC_PROGRESS_THRESHOLD_METERS;
            const bool waitingAtSignal =
                trafficVehicle.actor.speedMetersPerSecond <=
                    MAP_TRAFFIC_STOPPED_SPEED_METERS_PER_SECOND &&
                mapTrafficActorWaitingAtSignal(trafficVehicle.actor, currentSimulationTime);
            if (progressed) {
                trafficVehicle.lastProgressDistanceMeters = trafficVehicle.actor.distanceMeters;
                trafficVehicle.stoppedSecondsSinceProgress = 0.0;
                trafficVehicle.stuckDiagnosticLogged = false;
            } else if (waitingAtSignal) {
                trafficVehicle.lastProgressDistanceMeters = trafficVehicle.actor.distanceMeters;
                trafficVehicle.stoppedSecondsSinceProgress = 0.0;
                trafficVehicle.stuckDiagnosticLogged = false;
            } else if (trafficVehicle.actor.speedMetersPerSecond <=
                       MAP_TRAFFIC_STOPPED_SPEED_METERS_PER_SECOND) {
                trafficVehicle.stoppedSecondsSinceProgress += elapsedSeconds;
            } else {
                trafficVehicle.lastProgressDistanceMeters = trafficVehicle.actor.distanceMeters;
                trafficVehicle.stoppedSecondsSinceProgress = 0.0;
                trafficVehicle.stuckDiagnosticLogged = false;
            }
            if (!trafficVehicle.stuckDiagnosticLogged &&
                trafficVehicle.stoppedSecondsSinceProgress >= MAP_TRAFFIC_STUCK_TIMEOUT_SECONDS) {
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
                const bool onJunctionPath = segmentIndex < route.segments.size() &&
                                            !route.segments[segmentIndex].points.empty();
                gameLog.Log(
                    std::string("Map ") + (trafficVehicle.isBus ? "AI bus" : "road AI vehicle") +
                    " appears stuck after " +
                    std::to_string(trafficVehicle.stoppedSecondsSinceProgress) +
                    " seconds without route progress: " + trafficVehicle.actor.modelPath +
                    ", spline=" + std::to_string(splineId) +
                    ", route distance=" + std::to_string(trafficVehicle.actor.distanceMeters) +
                    " m, junction path=" + (onJunctionPath ? "yes" : "no") +
                    ", curve speed limit=" + std::to_string(curveSpeedLimit) +
                    " m/s, junction wait=" +
                    std::to_string(trafficVehicle.actor.junctionWaitSeconds) + " s.");
                trafficVehicle.stuckDiagnosticLogged = true;
            }
        }
    }
}

void RenderLoop::updatePostPhysicsVariables(const BusSimulation& simulation) {
    if (playerVehicle_ != nullptr) {
        playerVehicle_->updateSimulationVariables(
            simulation, playerVehicle_->variables.get("throttle"),
            playerVehicle_->variables.get("steering"), playerVehicle_->variables.get("brake"));
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
