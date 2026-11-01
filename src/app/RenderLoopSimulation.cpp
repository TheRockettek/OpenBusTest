// Script ticking and player/physics variable synchronisation.

#include "app/RenderLoopInternal.h"
#include "app/MapTrafficWorker.h"
#include "core/PassengerVariables.h"
#include "render/map/MapRenderer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace {

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
    gameLog.Debug(scope + " numeric variables: " + std::to_string(numericNames.size()));
    for (const std::string& name : numericNames) {
        std::ostringstream line;
        line << scope << '.' << name << " = " << std::setprecision(12)
             << variables.numericValues().at(name);
        gameLog.Debug(line.str());
    }

    std::vector<std::string> stringNames;
    stringNames.reserve(variables.stringValues().size());
    for (const auto& [name, value] : variables.stringValues()) {
        (void)value;
        stringNames.push_back(name);
    }
    std::sort(stringNames.begin(), stringNames.end());
    gameLog.Debug(scope + " string variables: " + std::to_string(stringNames.size()));
    for (const std::string& name : stringNames) {
        gameLog.Debug(scope + "." + name + " = " + variables.stringValues().at(name));
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
    gameLog.Debug(line.str());
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
    {
        const double passengerServiceTimeStep = std::clamp(frameTimeStep_, 0.0, 0.25);
        std::unordered_map<Vehicle*, int> sharedQueueStops;
        for (const TrafficVehicleState& state : trafficFleet_.vehicles) {
            if (state.vehicle != nullptr && state.bus && state.bus->passengerStopArrived) {
                sharedQueueStops.emplace(state.vehicle,
                                         state.vehicle->passengers->service.currentStopId());
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
                                                         passengerNetwork_.updateOffset);
        const auto hasOpenPassengerDoor = [](const Vehicle& vehicle,
                                             const std::vector<int>& pathPoints,
                                             const char* variablePrefix) {
            const std::size_t doorCount =
                std::min(pathPoints.size(), openbus::scripting::passengerDoorSlotCount);
            for (std::size_t doorIndex = 0; doorIndex < doorCount; ++doorIndex) {
                if (vehicle.variables.get(std::string(variablePrefix) + std::to_string(doorIndex) +
                                          "_open") > 0.5F) {
                    return true;
                }
            }
            return false;
        };
        for (const std::size_t vehicleIndex : passengerServiceUpdateOrder) {
            const std::unique_ptr<Vehicle>& vehicle = vehicles_[vehicleIndex];
            if (!vehicle->passengers) {
                continue;
            }
            const std::uint64_t stopRequestSequenceBefore =
                vehicle->passengers->service.passengerStopRequestSequence();
            const bool entryOpen = hasOpenPassengerDoor(
                *vehicle, vehicle->passengers->configuration().passengerEntryPathPoints,
                "pax_entry");
            const bool exitOpen = hasOpenPassengerDoor(
                *vehicle, vehicle->passengers->configuration().passengerExitPathPoints, "pax_exit");
            const auto activeStop = sharedQueueStops.find(vehicle.get());
            auto queue = activeStop != sharedQueueStops.end()
                             ? passengerNetwork_.queues.find(activeStop->second)
                             : passengerNetwork_.queues.end();
            if (queue != passengerNetwork_.queues.end()) {
                advanceMapPassengerServiceWithSharedQueue(
                    vehicle->passengers->service, activeStop->second, queue->second,
                    passengerServiceTimeStep, entryOpen, exitOpen);
            } else {
                vehicle->passengers->service.advance(passengerServiceTimeStep, entryOpen, exitOpen);
            }
            const std::uint64_t stopRequests =
                vehicle->passengers->service.passengerStopRequestSequence() -
                stopRequestSequenceBefore;
            if (stopRequests > 0 && vehicle->scripts &&
                vehicle->scripts->hasScriptEntryPoint("trigger_int_haltewunsch")) {
                for (std::uint64_t requestIndex = 0; requestIndex < stopRequests; ++requestIndex) {
                    vehicle->scripts->invokeSystemTrigger("int_haltewunsch");
                }
            }
        }
        if (std::any_of(passengerServiceStopIds.begin(), passengerServiceStopIds.end(),
                        [](int stopId) { return stopId != 0; })) {
            ++passengerNetwork_.updateOffset;
        }
    }
    updateScripts();
    for (TrafficVehicleState& trafficVehicle : trafficFleet_.vehicles) {
        if (!trafficVehicle.populationEnabled || trafficVehicle.vehicle == nullptr) {
            continue;
        }
        trafficVehicle.vehicle->updateTrafficWheelAnimations(
            trafficVehicle.wheelTravelDistanceMeters,
            mapTrafficRouteCurvatureAtRearAxle(trafficVehicle.actor,
                                               trafficVehicle.vehicle->trafficWheelbaseMeters,
                                               trafficVehicle.actor.rearAxleOffsetMeters),
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

void RenderLoop::updatePostPhysicsVariables(BusSimulation& simulation) {
    simulationState_.sharedVariables().updateSimulationClock(simulation.simulationTime());
    if (playerVehicle_ != nullptr) {
        if (activePlayerPassengerStop_ && !playerVehicle_->passengers) {
            throw std::logic_error("Active player passenger stop has no passenger service");
        }
        playerVehicle_->updateSimulationVariables(
            simulation, playerVehicle_->variables.get("throttle"),
            playerVehicle_->variables.get("steering"), playerVehicle_->variables.get("brake"));
        persistPlayerOdometer();
        if (const std::optional<BusCollisionEvent> collision = simulation.consumeCollisionEvent()) {
            playerVehicle_->variables.set("coll_pos_x", static_cast<float>(collision->position[0]));
            playerVehicle_->variables.set("coll_pos_y", static_cast<float>(collision->position[1]));
            playerVehicle_->variables.set("coll_pos_z", static_cast<float>(collision->position[2]));
            playerVehicle_->variables.set("coll_energy", static_cast<float>(collision->energy));
            if (playerVehicle_->scripts) {
                playerVehicle_->scripts->invokeSystemTrigger("collision");
            }
        }
        if (activePlayerPassengerStop_) {
            const auto queue = passengerNetwork_.queues.find(*activePlayerPassengerStop_);
            if (queue != passengerNetwork_.queues.end()) {
                queue->second = playerVehicle_->passengers->service.waitingPassengers(
                    *activePlayerPassengerStop_);
            }
        }
        if (activePlayerPassengerStop_ && !playerVehicle_->passengers->service.dwelling()) {
            const std::optional<std::size_t> completedRoute = activePlayerPassengerStopRouteIndex_;
            const std::optional<std::size_t> completedStop = activePlayerPassengerStopIndex_;
            if (activePlayerPassengerStopCompletesTimetable_ && playerTimetable_.state &&
                completedRoute && completedStop &&
                *completedRoute == playerTimetable_.state->selection.passengerRouteIndex) {
                const auto& completedPassengerRoute = passengerNetwork_.routes[*completedRoute];
                if (!openbus::app::nextLocatedPassengerStop(completedPassengerRoute,
                                                            *completedStop + 1)) {
                    advancePlayerTimetableTrip();
                } else {
                    playerTimetable_.state->nextStopIndex = *completedStop + 1;
                }
            }
            activePlayerPassengerStop_.reset();
            activePlayerPassengerStopRouteIndex_.reset();
            activePlayerPassengerStopIndex_.reset();
            activePlayerPassengerStopCompletesTimetable_ = false;
        }

        const auto chassis = simulation.chassisPose();
        const double chassisHeading = std::atan2(chassis.rotation[0], chassis.rotation[3]);
        const auto nearestStop =
            playerPassengerServiceEnabled_
                ? passengerNetwork_.findNearestStop(
                      chassis.position, chassisHeading,
                      playerTimetable_.state
                          ? std::optional<std::size_t>(
                                playerTimetable_.state->selection.passengerRouteIndex)
                          : std::nullopt,
                      playerTimetable_.state ? playerTimetable_.state->nextStopIndex : 0)
                : openbus::app::NearbyPassengerStop{};
        double latchedStopDistance = std::numeric_limits<double>::infinity();
        if (playerPassengerStopLatchRouteIndex_ && playerPassengerStopLatchIndex_ &&
            *playerPassengerStopLatchRouteIndex_ < passengerNetwork_.routes.size()) {
            const PassengerStopRouteState& latchedRoute =
                passengerNetwork_.routes[*playerPassengerStopLatchRouteIndex_];
            if (*playerPassengerStopLatchIndex_ < latchedRoute.stops.size()) {
                const auto latchedPose = latchedRoute.stopPose(*playerPassengerStopLatchIndex_);
                if (!latchedPose) {
                    throw std::runtime_error("Cannot sample the latched player passenger stop");
                }
                latchedStopDistance = std::hypot(latchedPose->position[0] - chassis.position[0],
                                                 latchedPose->position[1] - chassis.position[1]);
            }
        }
        if (playerPassengerStopLatch_ && latchedStopDistance > 5.0) {
            const auto queue = passengerNetwork_.queues.find(*playerPassengerStopLatch_);
            const auto initialQueue =
                passengerNetwork_.initialQueues.find(*playerPassengerStopLatch_);
            if (queue == passengerNetwork_.queues.end() ||
                initialQueue == passengerNetwork_.initialQueues.end()) {
                throw std::logic_error(
                    "Player passenger stop latch has no initialized waiting queue");
            }
            queue->second = openbus::map::mapScheduledPassengerQueueAfterDeparture(
                queue->second, initialQueue->second);
            playerPassengerStopLatch_.reset();
            playerPassengerStopLatchRouteIndex_.reset();
            playerPassengerStopLatchIndex_.reset();
        }
        if (playerPassengerServiceEnabled_ && !activePlayerPassengerStop_ &&
            !playerPassengerStopLatch_ && nearestStop.distanceMeters <= 2.5 &&
            std::abs(simulation.speed()) <= 0.35 && playerVehicle_->passengers &&
            playerVehicle_->passengers->service.seatCount() > 0) {
            const openbus::map::MapTrafficPassengerStop& stop =
                passengerNetwork_.routes[nearestStop.routeIndex].stops[nearestStop.stopIndex];
            const auto queue = passengerNetwork_.queues.find(stop.stopId);
            if (queue != passengerNetwork_.queues.end()) {
                const auto& route = passengerNetwork_.routes[nearestStop.routeIndex];
                const auto nextStop =
                    openbus::app::nextLocatedPassengerStop(route, nearestStop.stopIndex + 1);
                const int nextStopId = nextStop ? route.stops[*nextStop].stopId : 0;
                playerVehicle_->passengers->service.setWaitingPassengers(
                    stop.stopId, nextStopId == 0 ? 0 : queue->second);
                if (!playerVehicle_->passengers->service.arrive(
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
                activePlayerPassengerStopCompletesTimetable_ = playerTimetable_.state.has_value();
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
    const auto updatePlayerAndMapScripts = [&] {
        {
            TraceScope playerTrace("script", "RenderLoop::updateScripts.playerVehicle");
            for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                if (vehicle.get() == playerVehicle_ && !vehicle->mapTrafficRouteFinished &&
                    (!vehicle->followsMapTrafficRoute || vehicle->mapTrafficTileActive)) {
                    vehicle->updateScripts(false);
                }
            }
        }
        if (mapRenderer_ != nullptr) {
            TraceScope mapTrace("script", "RenderLoop::updateScripts.map");
            mapRenderer_->updateScripts();
        }
    };
    if (scriptRateHz_ <= 0.0) {
        simulationState_.sharedVariables().set(
            "timegap", std::clamp(frameTimeStep_, 0.0, MAX_SCRIPT_FRAME_DELTA_SECONDS));
        updatePlayerAndMapScripts();
    } else {
        const double scriptTimeStep = 1.0 / scriptRateHz_;
        const ScriptTickSchedule schedule =
            scheduleScriptTicks(scriptAccumulator_, frameTimeStep_, scriptTimeStep);
        scriptAccumulator_ = schedule.remainderSeconds;
        scriptTicksSinceDiagnostic_ += schedule.ticks;
        droppedScriptTicksSinceDiagnostic_ += schedule.droppedTicks;
        droppedScriptTimeSinceDiagnostic_ += schedule.droppedSeconds;
        for (int tick = 0; tick < schedule.ticks; ++tick) {
            simulationState_.sharedVariables().set("timegap", scriptTimeStep);
            updatePlayerAndMapScripts();
        }

        const double diagnosticTime = glfwGetTime();
        if (lastScriptDiagnosticTime_ < 0.0) {
            lastScriptDiagnosticTime_ = diagnosticTime;
        } else if (diagnosticTime - lastScriptDiagnosticTime_ >= 1.0) {
            if (droppedScriptTicksSinceDiagnostic_ > 0) {
                gameLog.Debug("Script scheduler executed " +
                              std::to_string(scriptTicksSinceDiagnostic_) + " ticks and dropped " +
                              std::to_string(droppedScriptTicksSinceDiagnostic_) + " ticks (" +
                              std::to_string(droppedScriptTimeSinceDiagnostic_) +
                              " s) during the last second");
            }
            lastScriptDiagnosticTime_ = diagnosticTime;
            scriptTicksSinceDiagnostic_ = 0;
            droppedScriptTicksSinceDiagnostic_ = 0;
            droppedScriptTimeSinceDiagnostic_ = 0.0;
        }
    }

    const double aiScriptTimeStep = 1.0 / AI_VEHICLE_SCRIPT_RATE_HZ;
    const ScriptTickSchedule aiSchedule =
        scheduleScriptTicks(aiScriptAccumulator_, frameTimeStep_, aiScriptTimeStep);
    aiScriptAccumulator_ = aiSchedule.remainderSeconds;
    for (int tick = 0; tick < aiSchedule.ticks; ++tick) {
        simulationState_.sharedVariables().set("timegap", aiScriptTimeStep);
        TraceScope aiTrace("script", "RenderLoop::updateAiVehicleScripts");
        {
            TraceScope aiVehiclesTrace("script", "RenderLoop::updateScripts.activeAiVehicles");
            for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                if (vehicle.get() != playerVehicle_ && !vehicle->mapTrafficRouteFinished &&
                    (!vehicle->followsMapTrafficRoute || vehicle->mapTrafficTileActive)) {
                    vehicle->updateScripts(true);
                    vehicle->soundBank.updateActiveSounds(soundEngine_, vehicle->variables,
                                                          soundViewpoint_);
                }
            }
        }
    }
    // Script ticks must not leak their timestep into later mouse handlers.
    simulationState_.sharedVariables().set(
        "timegap", scriptRateHz_ > 0.0
                       ? frameTimeStep_
                       : std::clamp(frameTimeStep_, 0.0, MAX_SCRIPT_FRAME_DELTA_SECONDS));
}
