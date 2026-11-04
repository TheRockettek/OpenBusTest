// Script ticking and player/physics variable synchronisation.

#include "app/RenderLoopInternal.h"
#include "app/MapTrafficWorker.h"
#include "app/vehicle/PassengerRequestVariables.h"
#include "core/PassengerVariables.h"
#include "render/map/MapRenderer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
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
                                                    "axle_brakeforce_1_r",
                                                    "axle_brakeforce_actual_0_l",
                                                    "axle_brakeforce_actual_0_r",
                                                    "axle_brakeforce_actual_1_l",
                                                    "axle_brakeforce_actual_1_r"};
    std::ostringstream line;
    line << "Native drivetrain variables" << std::setprecision(7);
    for (const char* name : variableNames) {
        line << ' ' << name << '=' << playerVehicle_->variables.get(name);
    }
    gameLog.Debug(line.str());
}

void RenderLoop::refreshPassengerWaitingPositions() {
    if (mapRenderer_ == nullptr) {
        return;
    }
    const auto& objects = mapRenderer_->editorPassengerDebugObjects();
    std::map<int, std::size_t> requestedPassengerCounts;
    for (const auto& [stopId, count] : passengerNetwork_.initialQueues) {
        requestedPassengerCounts.emplace(stopId, count);
    }
    for (const auto& [stopId, count] : passengerNetwork_.queues) {
        requestedPassengerCounts[stopId] = std::max(requestedPassengerCounts[stopId], count);
    }
    if (objects.size() == authoredPassengerObjectCount_ &&
        requestedPassengerCounts == authoredPassengerQueueCounts_) {
        return;
    }
    std::vector<openbus::map::MapPassengerStopAnchor> stops;
    std::vector<openbus::map::MapPassengerWaitingArea> areas;
    for (const auto& object : objects) {
        if (object.busStop && object.objectId.has_value()) {
            stops.push_back({*object.objectId, object.position});
        }
        if (object.passengerCabin && !object.passengerPositions.empty()) {
            areas.push_back({object.objectId.value_or(0), object.position,
                             object.passengerPositions, object.bounds});
        }
    }
    authoredPassengerPositions_ =
        openbus::map::mapTrafficAssignPassengerPositions(stops, areas, requestedPassengerCounts);
    authoredPassengerQueueCounts_ = std::move(requestedPassengerCounts);
    authoredPassengerObjectCount_ = objects.size();
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
    updateNonPlayerSoundListenerGain();
    if (mapRenderer_ != nullptr) {
        TraceScope trafficActorTrace("traffic", "MapRenderer::updateTrafficActors");
        mapRenderer_->updateTrafficActors(std::clamp(frameTimeStep_, 0.0, 0.25));
    }
    updateTrafficDriving(simulation, std::clamp(frameTimeStep_, 0.0, 0.25));
    refreshPassengerWaitingPositions();
    for (const auto& vehicle : vehicles_) {
        vehicle->soundViewContext =
            vehicle.get() == playerVehicle_ ? soundViewpoint_ : RenderViewContext::NonPlayer;
        if (vehicle->mapTrafficRouteFinished ||
            (vehicle->followsMapTrafficRoute && !vehicle->mapTrafficTileActive)) {
            vehicle->soundBank.stopAll(soundEngine_);
            continue;
        }
        BodyPose pose;
        if (vehicle.get() == playerVehicle_) {
            pose = simulation.chassisPose();
            pose.position = transformLocalPoint(pose, {0.0, 0.0, vehicle->modelOffsetZ});
        } else {
            VehiclePlacement placement = vehicle->placement;
            if (mapRenderer_ != nullptr && !vehicle->followsMapTrafficRoute) {
                placement.position[2] =
                    mapRenderer_->terrainHeightAt(placement.position[0], placement.position[1])
                        .value_or(placement.position[2]);
            }
            const auto rotation = multiplyMatrix4(
                rotationMatrix(placement.yawDegrees, 0.0, 0.0, 1.0),
                multiplyMatrix4(rotationMatrix(-placement.pitchDegrees, 0.0, 1.0, 0.0),
                                rotationMatrix(placement.rollDegrees, 1.0, 0.0, 0.0)));
            pose.position = makeModelRootPlacement(placement, vehicle->modelOffsetZ).position;
            pose.rotation = {rotation[0], rotation[4], rotation[8], rotation[1], rotation[5],
                             rotation[9], rotation[2], rotation[6], rotation[10]};
            pose.position = transformLocalPoint(pose, {0.0, 0.0, vehicle->modelOffsetZ});
        }
        vehicle->soundBank.setPose(pose);
        vehicle->soundBank.updateActiveSounds(soundEngine_, vehicle->variables,
                                              vehicle->soundViewContext);
    }
    {
        const double passengerServiceTimeStep = std::clamp(frameTimeStep_, 0.0, 0.25);
        std::unordered_map<Vehicle*, int> sharedQueueStops;
        std::unordered_map<Vehicle*, const TrafficVehicleState*> scheduledStopVehicles;
        for (const TrafficVehicleState& state : trafficFleet_.vehicles) {
            if (state.vehicle != nullptr && state.bus && state.bus->passengerStopArrived) {
                sharedQueueStops.emplace(state.vehicle,
                                         state.vehicle->passengers->service.currentStopId());
                scheduledStopVehicles.emplace(state.vehicle, &state);
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
        for (const std::size_t vehicleIndex : passengerServiceUpdateOrder) {
            const std::unique_ptr<Vehicle>& vehicle = vehicles_[vehicleIndex];
            if (!vehicle->passengers) {
                continue;
            }
            const std::uint64_t stopRequestSequenceBefore =
                vehicle->passengers->service.passengerStopRequestSequence();
            const auto activeStop = sharedQueueStops.find(vehicle.get());
            const bool playerServiceActive =
                vehicle.get() == playerVehicle_ && activeStop != sharedQueueStops.end();
            const bool entryOpen = openbus::app::vehicle::passengerDoorIsOpen(
                vehicle->variables, openbus::app::vehicle::PassengerDoorKind::Entry,
                vehicle->passengers->entryDoor());
            const bool exitOpen = openbus::app::vehicle::passengerDoorIsOpen(
                vehicle->variables, openbus::app::vehicle::PassengerDoorKind::Exit,
                vehicle->passengers->exitDoor());
            auto queue = activeStop != sharedQueueStops.end()
                             ? passengerNetwork_.queues.find(activeStop->second)
                             : passengerNetwork_.queues.end();
            bool passengerReady = true;
            if (playerServiceActive && queue != passengerNetwork_.queues.end()) {
                playerPassengerApproachSeconds_ += passengerServiceTimeStep;
                if (queue->second > 0 && !vehicle->passengers->service.boardingPassenger()) {
                    const auto initialQueue = passengerNetwork_.initialQueues.find(activeStop->second);
                    const std::size_t initialCount =
                        initialQueue == passengerNetwork_.initialQueues.end()
                            ? queue->second
                            : std::max(initialQueue->second, queue->second);
                    const std::size_t slot = openbus::map::mapScheduledPassengerQueueSlot(
                        initialCount, queue->second - 1);
                    const auto start = passengerQueuePose(activeStop->second, slot);
                    if (!start || mapRenderer_ == nullptr) {
                        throw std::logic_error("Cannot locate waiting player bus passengers");
                    }
                    const auto destination = playerPassengerDoorQueuePose(
                        *vehicle, simulation.chassisPose(), mapRenderer_->leftHandTraffic(), slot);
                    passengerReady = playerPassengerApproachSeconds_ >=
                                     passengerApproachDuration(*start, destination);
                }
            }
            if (queue != passengerNetwork_.queues.end()) {
                advanceMapPassengerServiceWithSharedQueue(
                    vehicle->passengers->service, activeStop->second, queue->second,
                    passengerNetwork_.queueIdentities, passengerServiceTimeStep,
                    entryOpen && passengerReady, exitOpen,
                    playerServiceActive && activePlayerPassengerStopServiceGeneration_ ==
                                               playerPassengerServiceGeneration_,
                    true);
            } else {
                vehicle->passengers->service.advance(passengerServiceTimeStep, entryOpen, exitOpen,
                                                     false, true);
            }
            auto& service = vehicle->passengers->service;
            if (service.boardingPassenger() || service.alightingPassenger()) {
                if (activeStop == sharedQueueStops.end() || mapRenderer_ == nullptr) {
                    service.setPassengerPathLinkProgressProfile({});
                } else {
                    openbus::map::MapTrafficPose busPose;
                    std::optional<openbus::map::MapTrafficPose> stopPose;
                    double laneOffset = 0.0;
                    double vehicleWidth = vehicle->trafficWidthMeters;
                    if (playerServiceActive) {
                        if (!activePlayerPassengerStopRouteIndex_ ||
                            *activePlayerPassengerStopRouteIndex_ >=
                                passengerNetwork_.routes.size() ||
                            !activePlayerPassengerStopIndex_) {
                            throw std::logic_error(
                                "Active player passenger stop has no mapped route call");
                        }
                        const auto& route =
                            passengerNetwork_.routes[*activePlayerPassengerStopRouteIndex_];
                        if (*activePlayerPassengerStopIndex_ >= route.stops.size() ||
                            route.stops[*activePlayerPassengerStopIndex_].stopId !=
                                activeStop->second) {
                            throw std::logic_error(
                                "Active player passenger stop does not match its route call");
                        }
                        const auto chassis = simulation.chassisPose();
                        busPose.position =
                            transformLocalPoint(chassis, {0.0, 0.0, vehicle->modelOffsetZ});
                        busPose.headingRadians =
                            std::atan2(chassis.rotation[0], chassis.rotation[3]);
                        stopPose = route.stopPose(*activePlayerPassengerStopIndex_);
                        if (stopPose) {
                            if (const auto editorPosition =
                                    mapRenderer_->editorOnlyBusStopPosition(activeStop->second)) {
                                stopPose->position = *editorPosition;
                            }
                        }
                    } else {
                        const auto scheduled = scheduledStopVehicles.find(vehicle.get());
                        if (scheduled == scheduledStopVehicles.end()) {
                            throw std::logic_error(
                                "Active scheduled passenger stop has no bus state");
                        }
                        const auto& state = *scheduled->second;
                        if (state.bus->nextPassengerStopIndex >= state.bus->passengerStops.size() ||
                            state.bus->passengerStops[state.bus->nextPassengerStopIndex].stopId !=
                                activeStop->second) {
                            throw std::logic_error(
                                "Scheduled passenger stop does not match its route call");
                        }
                        const auto sampled = openbus::map::sampleMapTrafficActorPose(
                            state.actor, state.actor.distanceMeters);
                        if (!sampled) {
                            throw std::runtime_error(
                                "Cannot sample a scheduled AI bus passenger pose");
                        }
                        busPose = *sampled;
                        busPose.position[2] += vehicle->aiDeltaHeight;
                        const auto& stop =
                            state.bus->passengerStops[state.bus->nextPassengerStopIndex];
                        stopPose = openbus::map::sampleMapTrafficRoute(
                            state.actor.route, stop.routeDistanceMeters, 0.0);
                        laneOffset = state.actor.lateralOffsetMeters;
                        vehicleWidth = state.actor.widthMeters;
                    }
                    if (!stopPose) {
                        throw std::runtime_error("Cannot sample a bus passenger boarding pose");
                    }
                    const auto curb = passengerCurbPose(*stopPose, mapRenderer_->leftHandTraffic(),
                                                        laneOffset, vehicleWidth);
                    const auto& cabin = vehicle->passengers->configuration();
                    const bool boarding = service.boardingPassenger();
                    const auto seat =
                        boarding ? service.boardingSeatIndex() : service.alightingSeatIndex();
                    if (!seat || *seat >= cabin.passengerPositions.size()) {
                        throw std::logic_error("Active passenger has no configured seat");
                    }
                    const auto& position = cabin.passengerPositions[*seat];
                    const auto seatPose = passengerCabinSeatPose(busPose, position);
                    if (boarding) {
                        const auto initialQueue =
                            passengerNetwork_.initialQueues.find(activeStop->second);
                        const std::size_t initialCount =
                            initialQueue == passengerNetwork_.initialQueues.end()
                                ? 0
                                : initialQueue->second;
                        const auto slot = service.boardingPassengerQueueSlot(initialCount);
                        if (!slot) {
                            throw std::logic_error("Boarding passenger has no reserved queue slot");
                        }
                        const auto outside =
                            playerServiceActive
                                ? playerPassengerDoorQueuePose(*vehicle, simulation.chassisPose(),
                                                               mapRenderer_->leftHandTraffic(),
                                                               *slot)
                                : passengerQueuePose(activeStop->second, *slot).value_or(curb);
                        updatePassengerCabinPathOccupancy(
                            service, cabin, busPose, outside.position,
                            passengerCabinStandingPoint(position.position, position.seatHeight),
                            true, *seat, vehicle->passengers->entryDoor());
                    } else {
                        const auto outside =
                            passengerConfiguredDoorQueuePose(
                                *vehicle, busPose, mapRenderer_->leftHandTraffic(),
                                cabin.passengerExitPathPoints, vehicle->passengers->exitDoor(), 0)
                                .value_or(curb);
                        updatePassengerCabinPathOccupancy(
                            service, cabin, busPose, seatPose.position,
                            passengerCabinLocalPoint(busPose, outside.position), false, *seat,
                            vehicle->passengers->exitDoor());
                    }
                }
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

void RenderLoop::updateNonPlayerSoundListenerGain() {
    const float outsideGain =
        playerVehicle_ != nullptr &&
                soundViewpoint_ == openbus::rendering::ViewpointContext::PlayerInterior
            ? std::clamp(playerVehicle_->variables.get("snd_outsidevol"), 0.0F, 1.0F)
            : 1.0F;
    for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
        if (vehicle.get() != playerVehicle_) {
            vehicle->soundBank.setListenerOutsideGain(outsideGain);
        }
    }
}

void RenderLoop::updatePostPhysicsVariables(BusSimulation& simulation) {
    const double simulationTime = simulation.simulationTime();
    if (hasPreviousCalendarSimulationTime_ && simulationTime >= previousCalendarSimulationTime_) {
        calendarElapsedSeconds_ += simulationTime - previousCalendarSimulationTime_;
    }
    previousCalendarSimulationTime_ = simulationTime;
    hasPreviousCalendarSimulationTime_ = true;
    simulationState_.sharedVariables().updateSimulationClock(calendarElapsedSeconds_);
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
        if (activePlayerPassengerStop_ && std::abs(simulation.speed()) > 0.35) {
            const auto queue = passengerNetwork_.queues.find(*activePlayerPassengerStop_);
            if (queue != passengerNetwork_.queues.end()) {
                openbus::map::departMapPassengerServiceWithSharedQueue(
                    playerVehicle_->passengers->service, queue->second,
                    passengerNetwork_.queueIdentities);
            } else {
                playerVehicle_->passengers->service.depart();
            }
        }
        if (activePlayerPassengerStop_ && !playerVehicle_->passengers->service.dwelling()) {
            const std::optional<std::size_t> completedRoute = activePlayerPassengerStopRouteIndex_;
            const std::optional<std::size_t> completedStop = activePlayerPassengerStopIndex_;
            if (activePlayerPassengerStopServiceGeneration_ == playerPassengerServiceGeneration_ &&
                completedRoute && completedStop &&
                playerPassengerServiceRouteIndex_ == completedRoute) {
                playerPassengerServiceNextStopIndex_ = *completedStop + 1;
            }
            activePlayerPassengerStop_.reset();
            activePlayerPassengerStopRouteIndex_.reset();
            activePlayerPassengerStopIndex_.reset();
            if (playerTimetable_.state &&
                playerPassengerServiceRouteIndex_ !=
                    playerTimetable_.state->selection.passengerRouteIndex) {
                playerPassengerServiceRouteIndex_ =
                    playerTimetable_.state->selection.passengerRouteIndex;
                playerPassengerServiceNextStopIndex_ =
                    playerTimetable_.state->selection.firstStopIndex;
                ++playerPassengerServiceGeneration_;
            }
        }

        const auto chassis = simulation.chassisPose();
        const double chassisHeading = std::atan2(chassis.rotation[0], chassis.rotation[3]);
        if (playerTimetable_.state) {
            const std::size_t routeIndex =
                playerTimetable_.state->selection.passengerRouteIndex;
            if (routeIndex >= passengerNetwork_.routes.size()) {
                throw std::logic_error("Active player timetable route is outside its catalog");
            }
            const PassengerStopRouteState& route = passengerNetwork_.routes[routeIndex];
            const auto resolveEditorPosition = [this](const auto& stop) {
                return mapRenderer_ ? mapRenderer_->editorOnlyBusStopPosition(stop.stopId)
                                    : std::nullopt;
            };
            const auto stopIndex = openbus::app::nextPassengerStopWithPosition(
                route, playerTimetable_.state->nextStopIndex, resolveEditorPosition);
            if (playerTimetableStopVisitLatchPosition_ &&
                std::hypot(playerTimetableStopVisitLatchPosition_->at(0) - chassis.position[0],
                           playerTimetableStopVisitLatchPosition_->at(1) - chassis.position[1]) >
                    PLAYER_TIMETABLE_STOP_VISIT_RADIUS_METERS) {
                playerTimetableStopVisitLatchPosition_.reset();
            }
            if (stopIndex && !playerTimetableStopVisitLatchPosition_) {
                const auto& stop = route.stops[*stopIndex];
                const auto pose = route.stopPose(*stopIndex);
                const auto editorStopPosition =
                    mapRenderer_ ? mapRenderer_->editorOnlyBusStopPosition(stop.stopId)
                                 : std::nullopt;
                const auto visitPosition =
                    editorStopPosition ? editorStopPosition
                                       : pose ? std::optional{pose->position} : std::nullopt;
                if (!visitPosition) {
                    throw std::logic_error("Located player timetable stop has no position");
                }
                const double visitDistance =
                    std::hypot((*visitPosition)[0] - chassis.position[0],
                               (*visitPosition)[1] - chassis.position[1]);
                if (visitDistance <= PLAYER_TIMETABLE_STOP_VISIT_RADIUS_METERS) {
                    playerTimetableStopVisitLatchPosition_ =
                        std::array<double, 2>{(*visitPosition)[0], (*visitPosition)[1]};
                    playerTimetable_.state->nextStopIndex = *stopIndex + 1;
                    if (!openbus::app::nextPassengerStopWithPosition(
                            route, playerTimetable_.state->nextStopIndex, resolveEditorPosition)) {
                        advancePlayerTimetableTrip();
                    }
                }
            }
        }

        openbus::app::NearbyPassengerStop nearestStop;
        if (playerPassengerServiceEnabled_) {
            if (playerPassengerServiceRouteIndex_) {
                if (*playerPassengerServiceRouteIndex_ >= passengerNetwork_.routes.size()) {
                    throw std::logic_error("Player passenger-service route is outside its catalog");
                }
                const PassengerStopRouteState& route =
                    passengerNetwork_.routes[*playerPassengerServiceRouteIndex_];
                const auto stopIndex = openbus::app::nextLocatedPassengerStop(
                    route, playerPassengerServiceNextStopIndex_);
                if (stopIndex) {
                    const auto pose = route.stopPose(*stopIndex);
                    if (!pose) {
                        throw std::logic_error("Located player passenger stop has no pose");
                    }
                    const auto& stop = route.stops[*stopIndex];
                    const auto editorStopPosition =
                        mapRenderer_ ? mapRenderer_->editorOnlyBusStopPosition(stop.stopId)
                                     : std::nullopt;
                    const auto& stopPosition = editorStopPosition.value_or(pose->position);
                    const double distance =
                        std::hypot(stopPosition[0] - chassis.position[0],
                                   stopPosition[1] - chassis.position[1]);
                    const double headingDifference =
                        std::remainder(chassisHeading - pose->headingRadians,
                                       2.0 * std::numbers::pi);
                    if (std::abs(headingDifference) <= std::numbers::pi / 3.0) {
                        nearestStop = {*playerPassengerServiceRouteIndex_, *stopIndex, distance};
                    }
                }
            } else if (!playerTimetable_.state) {
                nearestStop = passengerNetwork_.findNearestStop(
                    chassis.position, chassisHeading, std::nullopt, 0,
                    [this](const auto& stop) {
                        return mapRenderer_ ? mapRenderer_->editorOnlyBusStopPosition(stop.stopId)
                                            : std::nullopt;
                    });
            }
        }
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
                const int latchedStopId =
                    latchedRoute.stops[*playerPassengerStopLatchIndex_].stopId;
                const auto editorStopPosition =
                    mapRenderer_ ? mapRenderer_->editorOnlyBusStopPosition(latchedStopId)
                                 : std::nullopt;
                const auto& latchPosition = editorStopPosition.value_or(latchedPose->position);
                latchedStopDistance = std::hypot(latchPosition[0] - chassis.position[0],
                                                 latchPosition[1] - chassis.position[1]);
            }
        }
        const bool reachedDifferentStop =
            playerPassengerStopLatch_ &&
            nearestStop.distanceMeters <= PLAYER_PASSENGER_STOP_ARRIVAL_RADIUS_METERS &&
            passengerNetwork_.routes[nearestStop.routeIndex].stops[nearestStop.stopIndex].stopId !=
                *playerPassengerStopLatch_;
        if (playerPassengerStopLatch_ && !activePlayerPassengerStop_ &&
            (latchedStopDistance > PLAYER_PASSENGER_STOP_RELEASE_RADIUS_METERS ||
             reachedDifferentStop)) {
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
            !playerPassengerStopLatch_ &&
            nearestStop.distanceMeters <= PLAYER_PASSENGER_STOP_ARRIVAL_RADIUS_METERS &&
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
                playerPassengerApproachSeconds_ = 0.0;
                activePlayerPassengerStopRouteIndex_ = nearestStop.routeIndex;
                activePlayerPassengerStopIndex_ = nearestStop.stopIndex;
                activePlayerPassengerStopServiceGeneration_ = playerPassengerServiceGeneration_;
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

    updateNonPlayerSoundListenerGain();

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
                    vehicle->soundBank.updateAmbient(soundEngine_, vehicle->variables,
                                                     vehicle->soundViewContext);
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
