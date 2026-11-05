// Script ticking and player/physics variable synchronisation.

#include "app/RenderLoopInternal.h"
#include "app/MapTrafficWorker.h"
#include "app/vehicle/PassengerRequestVariables.h"
#include "core/PassengerVariables.h"
#include "render/map/MapRenderer.h"

#include <algorithm>
#include <cmath>
#include <functional>
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
    std::map<std::pair<int, std::size_t>, bool> waitingPassengerKeys;
    for (const auto& [stopId, count] : passengerNetwork_.queues) {
        (void)count;
        for (const auto& passenger : passengerNetwork_.waitingPassengers(stopId)) {
            waitingPassengerKeys.emplace(std::pair{stopId, passenger.modelIndex}, true);
        }
    }
    std::erase_if(passengerHeldQueuePositions_, [&](const auto& entry) {
        return !waitingPassengerKeys.contains(entry.first);
    });
    const bool pedestrianQueuePositionsReady = mapRenderer_->humanTrafficLaneNetworkReady();
    if (objects.size() == authoredPassengerObjectCount_ &&
        requestedPassengerCounts == authoredPassengerQueueCounts_ &&
        (!pedestrianQueuePositionsReady || passengerPedestrianQueuePositionsPrepared_)) {
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
    passengerPedestrianQueuePositions_.clear();
    if (pedestrianQueuePositionsReady) {
        for (const auto& [stopId, count] : authoredPassengerQueueCounts_) {
            const auto authored = authoredPassengerPositions_.find(stopId);
            if (count == 0 ||
                (authored != authoredPassengerPositions_.end() && !authored->second.empty())) {
                continue;
            }
            std::optional<std::array<double, 3>> stopPosition =
                mapRenderer_->editorOnlyBusStopPosition(stopId);
            if (!stopPosition) {
                for (const PassengerStopRouteState& route : passengerNetwork_.routes) {
                    for (std::size_t stopIndex = 0; stopIndex < route.stops.size(); ++stopIndex) {
                        if (route.stops[stopIndex].stopId == stopId) {
                            const auto pose = route.stopPose(stopIndex);
                            if (pose) {
                                stopPosition = pose->position;
                            }
                            break;
                        }
                    }
                    if (stopPosition) {
                        break;
                    }
                }
            }
            if (!stopPosition) {
                continue;
            }
            const auto pedestrian = mapRenderer_->passengerPedestrianJourneyNear(*stopPosition, 0);
            if (!pedestrian || !pedestrian->route || pedestrian->route->totalLength <= 0.0) {
                continue;
            }
            const double startDistance = pedestrian->startDistanceMeters;
            const double direction =
                pedestrian->route->totalLength - startDistance >= startDistance ? 1.0 : -1.0;
            const double maximumDistance = std::max(0.0, pedestrian->route->totalLength - 0.01);
            auto& positions = passengerPedestrianQueuePositions_[stopId];
            positions.reserve(count);
            for (std::size_t slot = 0; slot < count; ++slot) {
                const double distance = std::clamp(
                    startDistance + direction * 0.8 * static_cast<double>(slot), 0.0,
                    maximumDistance);
                const auto pose =
                    openbus::map::sampleMapTrafficRoute(
                        *pedestrian->route, distance, pedestrian->lateralOffsetMeters);
                if (!pose) {
                    throw std::runtime_error(
                        "Cannot sample pedestrian queue position near passenger stop");
                }
                positions.push_back(*pose);
            }
        }
        passengerPedestrianQueuePositionsPrepared_ = pedestrianQueuePositionsReady;
    }
}

void RenderLoop::cancelMovingPlayerPassengerService(const BusSimulation& simulation) {
    if (!activePlayerPassengerStop_) {
        return;
    }
    if (playerVehicle_ == nullptr || !playerVehicle_->passengers) {
        throw std::logic_error("Active player passenger stop has no passenger service");
    }
    if (std::abs(simulation.speed()) <= 0.35) {
        return;
    }
    const auto queue = passengerNetwork_.queues.find(*activePlayerPassengerStop_);
    if (queue != passengerNetwork_.queues.end()) {
        passengerNetwork_.departService(playerVehicle_->passengers->service);
    } else {
        playerVehicle_->passengers->service.depart();
    }
    if (mapRenderer_ != nullptr) {
        const auto queueApproaches = passengerQueueApproachProgress_.find(playerVehicle_);
        for (const auto& rider : passengerNetwork_.waitingPassengers(*activePlayerPassengerStop_)) {
            const auto key = std::pair{*activePlayerPassengerStop_, rider.modelIndex};
            std::optional<openbus::map::MapTrafficPose> current;
            const auto rendered = passengerRenderedQueuePositions_.find(key);
            if (rendered != passengerRenderedQueuePositions_.end()) {
                current = rendered->second;
            } else if (const auto held = passengerHeldQueuePositions_.find(key);
                       held != passengerHeldQueuePositions_.end()) {
                current = held->second;
            } else if (queueApproaches != passengerQueueApproachProgress_.end()) {
                const auto approach = queueApproaches->second.find(rider.modelIndex);
                const auto start = passengerQueuePose(*activePlayerPassengerStop_, rider);
                if (approach != queueApproaches->second.end() && start &&
                    approach->second.stopId == *activePlayerPassengerStop_) {
                    const auto chassis = simulation.chassisPose();
                    const auto doorQueue = playerPassengerDoorQueuePose(
                        *playerVehicle_, chassis, mapRenderer_->leftHandTraffic(),
                        rider.queueSlot);
                    current = openbus::app::passengerQueueApproachPose(
                        approach->second.roadApproach, *start, doorQueue,
                        approach->second.elapsedSeconds);
                }
            }
            if (current) {
                passengerHeldQueuePositions_[key] = *current;
            }
        }
    }
}

void RenderLoop::updatePlayerVariables(BusSimulation& simulation, double throttle,
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
    cancelMovingPlayerPassengerService(simulation);
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
        std::unordered_map<Vehicle*, const TrafficVehicleState*> scheduledVehicles;
        for (const TrafficVehicleState& state : trafficFleet_.vehicles) {
            if (state.vehicle != nullptr && state.bus) {
                scheduledVehicles.emplace(state.vehicle, &state);
                if (state.bus->passengerStopArrived) {
                    sharedQueueStops.emplace(state.vehicle,
                                             state.vehicle->passengers->service.currentStopId());
                    scheduledStopVehicles.emplace(state.vehicle, &state);
                }
            }
        }
        const auto queueDoorPose = [this, &simulation, &scheduledStopVehicles](
                                       Vehicle* vehicle, std::size_t queueSlot)
            -> std::optional<openbus::map::MapTrafficPose> {
            if (!mapRenderer_) {
                return std::nullopt;
            }
            std::optional<openbus::map::MapTrafficPose> destination;
            if (vehicle == playerVehicle_) {
                destination = playerPassengerDoorQueuePose(
                    *vehicle, simulation.chassisPose(), mapRenderer_->leftHandTraffic(),
                    queueSlot);
            } else {
                const auto scheduled = scheduledStopVehicles.find(vehicle);
                if (scheduled == scheduledStopVehicles.end()) {
                    return std::nullopt;
                }
                const auto& state = *scheduled->second;
                auto busPose = openbus::map::sampleMapTrafficActorPose(
                    state.actor, state.actor.distanceMeters);
                if (!busPose) {
                    throw std::runtime_error(
                        "Cannot sample scheduled pedestrian approach bus pose");
                }
                busPose->position[2] += vehicle->aiDeltaHeight;
                destination = passengerConfiguredDoorQueuePose(
                    *vehicle, *busPose, mapRenderer_->leftHandTraffic(),
                    vehicle->passengers->configuration().passengerEntryPathPoints,
                    vehicle->passengers->entryDoor(), queueSlot);
                if (!destination) {
                    destination = passengerDoorQueuePose(
                        *busPose, vehicle->trafficLengthMeters * 0.3,
                        vehicle->trafficWidthMeters, mapRenderer_->leftHandTraffic(),
                        queueSlot);
                }
            }
            return destination;
        };
        const auto updateQueueRoadApproach =
            [this, &queueDoorPose](
                Vehicle* vehicle, int stopId,
                const openbus::app::WaitingPassenger& rider,
                PassengerQueueApproachProgress& approach) {
                const auto destination = queueDoorPose(vehicle, rider.queueSlot);
                if (!destination) {
                    return;
                }
                const auto rendered =
                    passengerRenderedQueuePositions_.find({stopId, rider.modelIndex});
                const auto currentRenderedPose = [&]() {
                    if (rendered != passengerRenderedQueuePositions_.end()) {
                        return rendered->second;
                    }
                    if (approach.roadApproach) {
                        return openbus::app::passengerRoadApproachPose(
                            *approach.roadApproach, approach.elapsedSeconds);
                    }
                    return openbus::app::passengerApproachPose(
                        approach.originPose, approach.destinationPose,
                        approach.elapsedSeconds);
                };
                if (!approach.posesInitialized) {
                    const auto origin = passengerQueuePose(stopId, rider);
                    if (!origin) {
                        return;
                    }
                    approach.originPose = *origin;
                    approach.destinationPose = *destination;
                    approach.posesInitialized = true;
                } else if (openbus::app::passengerApproachTargetChanged(
                               approach.destinationPose, *destination)) {
                    approach.originPose = currentRenderedPose();
                    approach.destinationPose = *destination;
                    approach.elapsedSeconds = 0.0;
                    approach.roadApproach.reset();
                    approach.roadApproachAttempted = false;
                }
            };
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
        openbus::app::NearbyPassengerStop nearbyPlayerPassengerStop;
        if (playerVehicle_ != nullptr) {
            const auto chassis = simulation.chassisPose();
            const double heading = std::atan2(chassis.rotation[0], chassis.rotation[3]);
            nearbyPlayerPassengerStop = passengerNetwork_.findNearestStop(
                chassis.position, heading, playerPassengerServiceRouteIndex_, 0,
                [this](const auto& stop) {
                    return mapRenderer_ ? mapRenderer_->editorOnlyBusStopPosition(stop.stopId)
                                        : std::nullopt;
                });
        }
        const auto holdQueueApproachPose = [this](int stopId, std::size_t modelIndex) {
            const auto rendered =
                passengerRenderedQueuePositions_.find({stopId, modelIndex});
            if (rendered != passengerRenderedQueuePositions_.end()) {
                passengerHeldQueuePositions_[{stopId, modelIndex}] = rendered->second;
            }
        };
        for (const std::size_t vehicleIndex : passengerServiceUpdateOrder) {
            const std::unique_ptr<Vehicle>& vehicle = vehicles_[vehicleIndex];
            if (!vehicle->passengers) {
                continue;
            }
            const std::uint64_t stopRequestSequenceBefore =
                vehicle->passengers->service.passengerStopRequestSequence();
            auto& service = vehicle->passengers->service;
            if (service.onboardPassengerCount() > 0) {
                if (vehicle.get() == playerVehicle_) {
                    const auto position = simulation.chassisPose().position;
                    const auto requestRouteStops = [&](const PassengerStopRouteState& route,
                                                       std::size_t firstStop) {
                        openbus::app::requestPassengerStopsOnRoute(
                            service, route.stops, firstStop, position,
                            [&](std::size_t stopIndex) -> std::optional<std::array<double, 3>> {
                                const auto pose = route.stopPose(stopIndex);
                                if (!pose) {
                                    return std::nullopt;
                                }
                                const int stopId = route.stops[stopIndex].stopId;
                                const auto editorPosition =
                                    mapRenderer_ ? mapRenderer_->editorOnlyBusStopPosition(stopId)
                                                 : std::nullopt;
                                return editorPosition.value_or(pose->position);
                            });
                    };
                    if (playerPassengerServiceRouteIndex_) {
                        const auto& route =
                            passengerNetwork_.routes.at(*playerPassengerServiceRouteIndex_);
                        requestRouteStops(route, playerPassengerServiceNextStopIndex_);
                    } else {
                        for (const auto& route : passengerNetwork_.routes) {
                            requestRouteStops(route, 0);
                        }
                    }
                } else {
                    const auto scheduled = scheduledVehicles.find(vehicle.get());
                    if (scheduled != scheduledVehicles.end() &&
                        scheduled->second->bus->nextPassengerStopIndex <
                            scheduled->second->bus->passengerStops.size()) {
                        const auto& state = *scheduled->second;
                        const auto busPose = openbus::map::sampleMapTrafficActorPose(
                            state.actor, state.actor.distanceMeters);
                        if (!busPose) {
                            throw std::runtime_error(
                                "Cannot sample scheduled passenger stop-request bus pose");
                        }
                        openbus::app::requestPassengerStopsOnRoute(
                            service, state.bus->passengerStops, state.bus->nextPassengerStopIndex,
                            busPose->position,
                            [&](std::size_t index) -> std::optional<std::array<double, 3>> {
                                const auto& stop = state.bus->passengerStops[index];
                                const auto stopPose = openbus::map::sampleMapTrafficRoute(
                                    state.actor.route, stop.routeDistanceMeters, 0.0);
                                if (!stopPose) {
                                    throw std::runtime_error(
                                        "Cannot sample scheduled passenger stop-request pose");
                                }
                                return stopPose->position;
                            });
                    }
                }
            }
            const auto activeStop = sharedQueueStops.find(vehicle.get());
            const bool playerServiceActive =
                vehicle.get() == playerVehicle_ && activeStop != sharedQueueStops.end();
            const bool entryOpen = openbus::app::vehicle::passengerDoorIsOpen(
                vehicle->variables, openbus::app::vehicle::PassengerDoorKind::Entry,
                vehicle->passengers->entryDoor());
            bool exitOpen = openbus::app::vehicle::passengerDoorIsOpen(
                vehicle->variables, openbus::app::vehicle::PassengerDoorKind::Exit,
                vehicle->passengers->exitDoor());
            const auto alightingSeatBefore = service.alightingSeatIndex();
            struct AlightingPassengerSnapshot {
                std::size_t seatIndex;
                std::optional<std::size_t> modelIndex;
                std::optional<openbus::map::MapPassengerPedestrianJourney> pedestrianJourney;
            };
            std::vector<AlightingPassengerSnapshot> alightingPassengersBefore;
            if (alightingSeatBefore) {
                alightingPassengersBefore.push_back(
                    {*alightingSeatBefore, service.passengerModelIndexOnSeat(*alightingSeatBefore),
                     service.passengerPedestrianJourneyOnSeat(*alightingSeatBefore)});
            }
            for (const std::size_t seat : service.queuedAlightingSeatIndices()) {
                alightingPassengersBefore.push_back(
                    {seat, service.passengerModelIndexOnSeat(seat),
                     service.passengerPedestrianJourneyOnSeat(seat)});
            }
            if (exitOpen && playerServiceActive && alightingSeatBefore) {
                if (!mapRenderer_ || !activePlayerPassengerStopRouteIndex_ ||
                    !activePlayerPassengerStopIndex_) {
                    throw std::logic_error("Cannot locate the active player bus exit");
                }
                auto stopPose = passengerNetwork_.routes.at(*activePlayerPassengerStopRouteIndex_)
                                    .stopPose(*activePlayerPassengerStopIndex_);
                if (!stopPose) {
                    throw std::runtime_error("Cannot sample the player passenger stop pose");
                }
                if (const auto editorPosition =
                        mapRenderer_->editorOnlyBusStopPosition(activeStop->second)) {
                    stopPose->position = *editorPosition;
                }
                const auto chassis = simulation.chassisPose();
                openbus::map::MapTrafficPose busPose;
                busPose.position = transformLocalPoint(chassis, {0.0, 0.0, vehicle->modelOffsetZ});
                busPose.headingRadians = std::atan2(chassis.rotation[0], chassis.rotation[3]);
                const auto curb = passengerCurbPose(*stopPose, mapRenderer_->leftHandTraffic(), 0.0,
                                                    vehicle->trafficWidthMeters);
                const auto outside = passengerConfiguredDoorQueuePose(
                                         *vehicle, busPose, mapRenderer_->leftHandTraffic(),
                                         vehicle->passengers->configuration().passengerExitPathPoints,
                                         vehicle->passengers->exitDoorForSeat(*alightingSeatBefore), 0)
                                         .value_or(curb);
                exitOpen = service.canCurrentPassengerAlightAt(outside.position);
                if (!exitOpen && reportedPassengerDriftStops_.insert(activeStop->second).second) {
                    gameLog.Warning(
                        "Player bus moved its exit more than 200 m from a passenger destination "
                        "at stop " + std::to_string(activeStop->second) +
                        "; the passenger will wait aboard.");
                }
            }
            auto queue = activeStop != sharedQueueStops.end()
                             ? passengerNetwork_.queues.find(activeStop->second)
                             : passengerNetwork_.queues.end();
            std::optional<std::size_t> serviceRouteIndex;
            std::optional<std::size_t> serviceStopIndex;
            std::optional<std::size_t> nextPassenger;
            bool passengerDestinationPending = false;
            if (queue != passengerNetwork_.queues.end()) {
                if (playerServiceActive) {
                    serviceRouteIndex = activePlayerPassengerStopRouteIndex_;
                    serviceStopIndex = activePlayerPassengerStopIndex_;
                } else {
                    const auto scheduled = scheduledStopVehicles.find(vehicle.get());
                    if (scheduled != scheduledStopVehicles.end()) {
                        serviceRouteIndex = scheduled->second->bus->passengerRouteIndex;
                        if (serviceRouteIndex) {
                            serviceStopIndex = passengerNetwork_.routeCallIndexForBusStop(
                                *serviceRouteIndex, scheduled->second->bus->passengerStops,
                                scheduled->second->bus->nextPassengerStopIndex);
                        }
                    }
                }
                if (!serviceRouteIndex || *serviceRouteIndex >= passengerNetwork_.routes.size() ||
                    !serviceStopIndex) {
                    throw std::logic_error("Active passenger service has no registered route");
                }
                nextPassenger = passengerNetwork_.nextPassenger(activeStop->second,
                                                                *serviceRouteIndex, service,
                                                                serviceStopIndex);
                if (nextPassenger &&
                    !passengerQueuePose(
                        activeStop->second,
                        passengerNetwork_.waitingPassengers(activeStop->second)
                            .at(*nextPassenger))) {
                    nextPassenger.reset();
                }
                if (nextPassenger && !service.boardingPassenger()) {
                    const auto& rider =
                        passengerNetwork_.waitingPassengers(activeStop->second)[*nextPassenger];
                    if (!rider.journey) {
                        throw std::logic_error("Selected passenger has no journey");
                    }
                    if (!rider.journey->pedestrianResolved) {
                        if (!mapRenderer_ || !mapRenderer_->humanTrafficLaneNetworkReady()) {
                            passengerDestinationPending = service.canAcceptWaitingPassenger();
                            nextPassenger.reset();
                        } else {
                            const auto& route =
                                passengerNetwork_.routes.at(rider.journey->routeIndex);
                            if (rider.journey->originStopIndex != *serviceStopIndex ||
                                rider.journey->originStopIndex >= route.stops.size() ||
                                route.stops[rider.journey->originStopIndex].stopId !=
                                    activeStop->second ||
                                rider.journey->destinationStopIndex >= route.stops.size() ||
                                rider.journey->destinationStopIndex <=
                                    rider.journey->originStopIndex ||
                                route.stops[rider.journey->destinationStopIndex].stopId !=
                                    rider.journey->destinationStopId) {
                                throw std::logic_error(
                                    "Passenger journey stop-call indices do not match its service");
                            }
                            const auto destinationPose =
                                route.stopPose(rider.journey->destinationStopIndex);
                            if (!destinationPose) {
                                throw std::logic_error(
                                    "Passenger journey has no located downstream destination");
                            }
                            const auto key = std::pair{rider.journey->routeIndex,
                                                       rider.journey->destinationStopIndex};
                            auto cached = passengerPedestrianPaths_.find(key);
                            if (cached == passengerPedestrianPaths_.end()) {
                                auto candidate = mapRenderer_->passengerPedestrianJourneyNear(
                                    destinationPose->position, 0);
                                if (candidate) {
                                    cached = passengerPedestrianPaths_
                                                 .emplace(key, std::move(*candidate))
                                                 .first;
                                }
                            }
                            std::optional<openbus::map::MapPassengerPedestrianJourney> pedestrian;
                            if (cached != passengerPedestrianPaths_.end()) {
                                pedestrian = cached->second;
                                pedestrian->lateralOffsetMeters =
                                    openbus::map::mapTrafficPedestrianLateralOffset(
                                        *pedestrian->route, rider.modelIndex);
                                pedestrian->destinationDistanceMeters = std::min(
                                    pedestrian->route->totalLength - 0.01,
                                    pedestrian->startDistanceMeters + 15.0 +
                                        static_cast<double>(rider.modelIndex % 31));
                                const auto destination = openbus::map::sampleMapTrafficRoute(
                                    *pedestrian->route, pedestrian->destinationDistanceMeters,
                                    pedestrian->lateralOffsetMeters);
                                if (!destination) {
                                    throw std::runtime_error(
                                        "Cannot sample passenger pedestrian destination");
                                }
                                pedestrian->destination = destination->position;
                            }
                            if (!pedestrian &&
                                reportedPassengerPedestrianStops_
                                    .insert(rider.journey->destinationStopId)
                                    .second) {
                                gameLog.Warning(
                                    "No suitable pedestrian spline near passenger destination stop " +
                                    std::to_string(rider.journey->destinationStopId) +
                                    "; riders use the mapped stop and disappear after alighting.");
                            }
                            passengerNetwork_.resolvePassengerPedestrianJourney(
                                activeStop->second, *nextPassenger, std::move(pedestrian));
                            if (!passengerNetwork_.passengerMatchesService(
                                    passengerNetwork_.waitingPassengers(activeStop->second)
                                        [*nextPassenger],
                                    *serviceRouteIndex, service, serviceStopIndex)) {
                                nextPassenger.reset();
                            }
                        }
                    }
                }
                const auto& waiting = passengerNetwork_.waitingPassengers(activeStop->second);
                if (service.acceptsBoarding() &&
                    std::ranges::any_of(waiting,
                                        [](const auto& rider) { return !rider.journey; }) &&
                    reportedPassengerJourneyStops_.insert(activeStop->second).second) {
                    gameLog.Warning(
                        "No mapped downstream passenger journeys are available at stop " +
                        std::to_string(activeStop->second) +
                        "; waiting passengers cannot choose a service.");
                }
            }
            if (activeStop != sharedQueueStops.end() && queue != passengerNetwork_.queues.end() &&
                serviceRouteIndex && serviceStopIndex && !passengerDestinationPending &&
                !service.alightingPassenger()) {
                auto& approaches = passengerQueueApproachProgress_[vehicle.get()];
                std::unordered_set<std::size_t> eligibleModels;
                const auto& waiting = passengerNetwork_.waitingPassengers(activeStop->second);
                for (const auto& rider : waiting) {
                    if (!passengerNetwork_.passengerMatchesService(
                            rider, *serviceRouteIndex, service, serviceStopIndex) ||
                        !passengerQueuePose(activeStop->second, rider)) {
                        continue;
                    }
                    auto& approach = approaches[rider.modelIndex];
                    if (approach.stopId != activeStop->second ||
                        approach.routeIndex != *serviceRouteIndex ||
                        approach.stopCallIndex != *serviceStopIndex) {
                        approach = {activeStop->second, *serviceRouteIndex, *serviceStopIndex,
                                    0.0, std::nullopt, false, {}, {}, false};
                    }
                    updateQueueRoadApproach(vehicle.get(), activeStop->second, rider, approach);
                    approach.elapsedSeconds += passengerServiceTimeStep;
                    eligibleModels.insert(rider.modelIndex);
                }
                std::erase_if(approaches, [&](const auto& entry) {
                    if (eligibleModels.contains(entry.first)) {
                        return false;
                    }
                    holdQueueApproachPose(entry.second.stopId, entry.first);
                    return true;
                });
                if (approaches.empty()) {
                    passengerQueueApproachProgress_.erase(vehicle.get());
                }
            } else if (vehicle.get() == playerVehicle_ &&
                       nearbyPlayerPassengerStop.distanceMeters <=
                           PLAYER_PASSENGER_STOP_ARRIVAL_RADIUS_METERS) {
                const auto& route =
                    passengerNetwork_.routes.at(nearbyPlayerPassengerStop.routeIndex);
                const auto& stop = route.stops[nearbyPlayerPassengerStop.stopIndex];
                const int stopId = stop.stopId;
                const auto queue = passengerNetwork_.queues.find(stopId);
                if (queue != passengerNetwork_.queues.end()) {
                    auto& approaches = passengerQueueApproachProgress_[vehicle.get()];
                    std::unordered_set<std::size_t> eligibleModels;
                    const auto& waiting = passengerNetwork_.waitingPassengers(stopId);
                    for (const auto& rider : waiting) {
                        if (!rider.journey ||
                            rider.journey->routeIndex != nearbyPlayerPassengerStop.routeIndex ||
                            rider.journey->originStopIndex !=
                                nearbyPlayerPassengerStop.stopIndex ||
                            !passengerQueuePose(stopId, rider)) {
                            continue;
                        }
                        auto& approach = approaches[rider.modelIndex];
                        if (approach.stopId != stopId ||
                            approach.routeIndex != nearbyPlayerPassengerStop.routeIndex ||
                            approach.stopCallIndex != nearbyPlayerPassengerStop.stopIndex) {
                            approach = {stopId, nearbyPlayerPassengerStop.routeIndex,
                                        nearbyPlayerPassengerStop.stopIndex, 0.0, std::nullopt,
                                        false, {}, {}, false};
                        }
                        updateQueueRoadApproach(playerVehicle_, stopId, rider, approach);
                        approach.elapsedSeconds += passengerServiceTimeStep;
                        eligibleModels.insert(rider.modelIndex);
                    }
                    std::erase_if(approaches, [&](const auto& entry) {
                        if (eligibleModels.contains(entry.first)) {
                            return false;
                        }
                        holdQueueApproachPose(entry.second.stopId, entry.first);
                        return true;
                    });
                    if (approaches.empty()) {
                        passengerQueueApproachProgress_.erase(vehicle.get());
                    }
                } else {
                    const auto approaches = passengerQueueApproachProgress_.find(vehicle.get());
                    if (approaches != passengerQueueApproachProgress_.end()) {
                        for (const auto& [modelIndex, progress] : approaches->second) {
                            holdQueueApproachPose(progress.stopId, modelIndex);
                        }
                        passengerQueueApproachProgress_.erase(approaches);
                    }
                }
            } else {
                const auto approaches = passengerQueueApproachProgress_.find(vehicle.get());
                if (approaches != passengerQueueApproachProgress_.end()) {
                    for (const auto& [modelIndex, progress] : approaches->second) {
                        holdQueueApproachPose(progress.stopId, modelIndex);
                    }
                }
                passengerQueueApproachProgress_.erase(vehicle.get());
            }
            bool passengerReady = true;
            bool passengerTicketSalePending = false;
            if (activeStop != sharedQueueStops.end() && queue != passengerNetwork_.queues.end() &&
                nextPassenger && !service.boardingPassenger() &&
                !service.alightingPassenger()) {
                const auto& rider =
                    passengerNetwork_.waitingPassengers(activeStop->second)[*nextPassenger];
                auto& approach = passengerApproachProgress_[vehicle.get()];
                if (approach.stopId != activeStop->second ||
                    approach.modelIndex != rider.modelIndex ||
                    approach.routeIndex != *serviceRouteIndex ||
                    approach.stopCallIndex != *serviceStopIndex) {
                    approach = {};
                    approach.stopId = activeStop->second;
                    approach.modelIndex = rider.modelIndex;
                    approach.routeIndex = *serviceRouteIndex;
                    approach.stopCallIndex = *serviceStopIndex;
                }
                const auto vehicleApproaches =
                    passengerQueueApproachProgress_.find(vehicle.get());
                if (vehicleApproaches == passengerQueueApproachProgress_.end()) {
                    throw std::logic_error("Selected passenger has no queue approach state");
                }
                const auto riderApproach = vehicleApproaches->second.find(rider.modelIndex);
                if (riderApproach == vehicleApproaches->second.end()) {
                    throw std::logic_error("Selected passenger is not approaching the entry queue");
                }
                approach.elapsedSeconds = riderApproach->second.elapsedSeconds;
                const double approachSeconds = approach.elapsedSeconds;
                if (!playerServiceActive) {
                    const auto scheduled = scheduledStopVehicles.find(vehicle.get());
                    if (scheduled == scheduledStopVehicles.end() || !mapRenderer_) {
                        throw std::logic_error("Scheduled passenger approach has no active bus pose");
                    }
                }
                if (nextPassenger) {
                    const std::size_t slot =
                        passengerNetwork_.waitingPassengers(activeStop->second)[*nextPassenger]
                            .queueSlot;
                    std::optional<openbus::map::MapTrafficPose> start;
                    std::optional<openbus::map::MapTrafficPose> destination;
                    std::optional<openbus::map::MapTrafficPose> busPoseForTicket;
                    if (playerServiceActive) {
                        const auto chassis = simulation.chassisPose();
                        busPoseForTicket.emplace();
                        busPoseForTicket->position =
                            transformLocalPoint(chassis, {0.0, 0.0, vehicle->modelOffsetZ});
                        busPoseForTicket->headingRadians =
                            std::atan2(chassis.rotation[0], chassis.rotation[3]);
                        start = passengerQueuePose(activeStop->second, rider);
                        destination = playerPassengerDoorQueuePose(
                            *vehicle, chassis, mapRenderer_->leftHandTraffic(), slot);
                    } else {
                        const auto scheduled = scheduledStopVehicles.find(vehicle.get());
                        if (scheduled == scheduledStopVehicles.end() || !mapRenderer_) {
                            throw std::logic_error(
                                "Scheduled passenger approach has no mapped bus state");
                        }
                        const auto& state = *scheduled->second;
                        if (state.bus->nextPassengerStopIndex >= state.bus->passengerStops.size()) {
                            throw std::logic_error("Scheduled passenger approach has no stop call");
                        }
                        start = passengerQueuePose(activeStop->second, rider);
                        auto busPose = openbus::map::sampleMapTrafficActorPose(
                            state.actor, state.actor.distanceMeters);
                        if (!busPose) {
                            throw std::runtime_error(
                                "Cannot sample scheduled passenger approach bus pose");
                        }
                        busPose->position[2] += vehicle->aiDeltaHeight;
                        busPoseForTicket = *busPose;
                        destination = passengerConfiguredDoorQueuePose(
                            *vehicle, *busPose, mapRenderer_->leftHandTraffic(),
                            vehicle->passengers->configuration().passengerEntryPathPoints,
                            vehicle->passengers->entryDoor(), slot);
                        if (!destination) {
                            *destination = passengerDoorQueuePose(
                                *busPose, vehicle->trafficLengthMeters * 0.3,
                                vehicle->trafficWidthMeters, mapRenderer_->leftHandTraffic(), slot);
                        }
                    }
                    if (!start || !destination) {
                        throw std::logic_error("Cannot locate passenger entry queue poses");
                    }
                    const bool reachedEntry =
                        approachSeconds >= passengerQueueApproachDuration(
                                              riderApproach->second.roadApproach, *start,
                                              *destination);
                    passengerReady = reachedEntry;
                    const auto& cabin = vehicle->passengers->configuration();
                    if (entryOpen && reachedEntry && !approach.ticketSaleStarted) {
                        approach.ticketSaleStarted = true;
                        approach.ticketSaleComplete = true;
                        if (!cabin.passengerCabinNoTicketSale &&
                            passengerTicketPack_.ticketBuyProbability > 0.0 &&
                            !passengerTicketPack_.tickets.empty()) {
                            std::uniform_real_distribution<double> purchaseChance(0.0, 1.0);
                            if (purchaseChance(passengerTransactionRandom_) <
                                passengerTicketPack_.ticketBuyProbability) {
                                const auto& route = passengerNetwork_.routes.at(*serviceRouteIndex);
                                std::size_t originIndex = route.stops.size();
                                if (playerServiceActive && activePlayerPassengerStopIndex_) {
                                    originIndex = *activePlayerPassengerStopIndex_;
                                } else if (const auto scheduled =
                                               scheduledStopVehicles.find(vehicle.get());
                                           scheduled != scheduledStopVehicles.end()) {
                                    originIndex =
                                        scheduled->second->bus->nextPassengerStopIndex;
                                }
                                const auto& journey = *rider.journey;
                                const auto& destinationPosition =
                                    journey.pedestrianJourney
                                        ? journey.pedestrianJourney->destination
                                        : journey.destinationPosition;
                                std::optional<std::size_t> stationCount;
                                if (originIndex < route.stops.size()) {
                                    for (std::size_t index = originIndex + 1;
                                         index < route.stops.size(); ++index) {
                                        if (route.stops[index].stopId != journey.destinationStopId) {
                                            continue;
                                        }
                                        const auto pose = route.stopPose(index);
                                        if (pose &&
                                            std::hypot(pose->position[0] - destinationPosition[0],
                                                       pose->position[1] -
                                                           destinationPosition[1]) <=
                                                openbus::map::passengerStopRequestRadiusMeters) {
                                            stationCount = index - originIndex;
                                            break;
                                        }
                                    }
                                }
                                const std::optional<double> passengerAge =
                                    scheduledPassengerHumanAges_.empty()
                                        ? std::nullopt
                                        : scheduledPassengerHumanAges_[
                                              rider.modelIndex %
                                              scheduledPassengerHumanAges_.size()];
                                std::optional<std::size_t> ticketIndex;
                                for (std::size_t index = 0;
                                     index < passengerTicketPack_.tickets.size(); ++index) {
                                    const auto& ticket = passengerTicketPack_.tickets[index];
                                    if ((!stationCount && ticket.maxStations != 0) ||
                                        (stationCount && ticket.maxStations != 0 &&
                                         *stationCount >
                                             static_cast<std::size_t>(ticket.maxStations)) ||
                                        !passengerAge || *passengerAge < ticket.ageMin ||
                                        *passengerAge > ticket.ageMax) {
                                        continue;
                                    }
                                    if (!ticketIndex ||
                                        ticket.value <
                                            passengerTicketPack_.tickets[*ticketIndex].value) {
                                        ticketIndex = index;
                                    }
                                }
                                if (ticketIndex) {
                                    const auto& ticket =
                                        passengerTicketPack_.tickets[*ticketIndex];
                                    const bool hasPaymentCoins =
                                        ticket.value <= 0.0 || !passengerCurrency_.coins.empty();
                                    if (ticket.value > 0.0 && !hasPaymentCoins) {
                                        if (reportedPassengerTicketStops_.insert(activeStop->second)
                                                .second) {
                                            gameLog.Warning(
                                                "Passenger ticket sale is unavailable without map "
                                                "currency coins at stop " +
                                                std::to_string(activeStop->second) + ".");
                                        }
                                    } else {
                                        double payment = ticket.value;
                                        if (ticket.value > 0.0) {
                                            std::vector<double> coinValues;
                                            coinValues.reserve(passengerCurrency_.coins.size());
                                            for (const auto& coin : passengerCurrency_.coins) {
                                                coinValues.push_back(coin.value);
                                            }
                                            std::ranges::sort(coinValues, std::greater{});
                                            payment = 0.0;
                                            for (const double coinValue : coinValues) {
                                                const double remaining = ticket.value - payment;
                                                if (remaining <= 1.0e-8) {
                                                    break;
                                                }
                                                payment +=
                                                    std::floor((remaining + 1.0e-8) / coinValue) *
                                                    coinValue;
                                            }
                                            if (payment + 1.0e-8 < ticket.value) {
                                                const double smallestCoin =
                                                    std::ranges::min(coinValues);
                                                payment +=
                                                    std::ceil((ticket.value - payment - 1.0e-8) /
                                                              smallestCoin) *
                                                    smallestCoin;
                                            }
                                        }
                                        approach.ticketBought = true;
                                        approach.ticketPaymentRequired = ticket.value > 0.0;
                                        approach.changeDue =
                                            std::max(0.0, payment - ticket.value);
                                        approach.ticketSaleComplete = false;
                                        if (!cabin.passengerTicketSaleMachines.empty()) {
                                            approach.ticketMachineConfigurationIndex = 0;
                                        } else {
                                            approach.ticketPaymentStarted = true;
                                            approach.cashDispensedAtSaleStart =
                                                std::max(0.0, static_cast<double>(
                                                                  vehicle->variables.get(
                                                                      "cash_dispensed_total")));
                                        }
                                        vehicle->variables.set(
                                            "passenger_ticket_sale_active",
                                            approach.ticketPaymentStarted ? 1.0F : 0.0F);
                                        vehicle->variables.set("passenger_ticket_sale_price",
                                                               static_cast<float>(ticket.value));
                                        vehicle->variables.set("passenger_ticket_sale_paid_amount",
                                                               static_cast<float>(payment));
                                        vehicle->variables.set(
                                            "passenger_ticket_sale_change_due",
                                            static_cast<float>(approach.changeDue));
                                        vehicle->variables.set(
                                            "passenger_ticket_sale_machine_index",
                                            !approach.ticketMachineConfigurationIndex
                                                ? -1.0F
                                                : static_cast<float>(
                                                      cabin.passengerTicketSaleMachines[
                                                          *approach.ticketMachineConfigurationIndex]
                                                          .index));
                                        vehicle->variables.setString("passenger_ticket_sale_name",
                                                                     ticket.name);
                                        vehicle->variables.setString(
                                            "passenger_ticket_sale_display_string",
                                            ticket.displayString);
                                    }
                                }
                            }
                        }
                    }
                    if (entryOpen && reachedEntry && approach.ticketBought &&
                        approach.ticketMachineConfigurationIndex &&
                        !approach.ticketPaymentStarted) {
                        const auto machinePose = passengerVehicleLocalPose(
                            *busPoseForTicket,
                            cabin.passengerTicketSaleMachines[
                                *approach.ticketMachineConfigurationIndex]
                                .position);
                        approach.ticketMachineApproachSeconds += passengerServiceTimeStep;
                        if (approach.ticketMachineApproachSeconds >=
                            passengerApproachDuration(*destination, machinePose)) {
                            approach.ticketPaymentStarted = true;
                            approach.cashDispensedAtSaleStart =
                                std::max(0.0, static_cast<double>(
                                                  vehicle->variables.get(
                                                      "cash_dispensed_total")));
                            vehicle->variables.set("passenger_ticket_sale_active", 1.0F);
                        }
                    }
                    if (entryOpen && reachedEntry && approach.ticketBought &&
                        approach.ticketPaymentStarted && !approach.ticketPrinted) {
                        const auto origin = passengerTicketSaleOriginPose(
                            cabin, *busPoseForTicket, *destination,
                            approach.ticketMachineConfigurationIndex);
                        const auto moneyPoint = approach.ticketPaymentRequired
                                                    ? passengerTicketMoneyPointPose(
                                                          cabin, *busPoseForTicket, origin)
                                                    : origin;
                        const auto changePoint = passengerTicketChangePointPose(
                            cabin, *busPoseForTicket, moneyPoint, approach.changeDue);
                        if (!approach.ticketPaymentRequired ||
                            !cabin.passengerTicketMoneyPoint) {
                            approach.ticketMoneyPointReached = true;
                        } else if (!approach.ticketMoneyPointReached) {
                            approach.ticketMoneyPointApproachSeconds +=
                                passengerServiceTimeStep;
                            if (approach.ticketMoneyPointApproachSeconds >=
                                passengerApproachDuration(origin, moneyPoint)) {
                                approach.ticketMoneyPointReached = true;
                            }
                        }
                        if (approach.changeDue <= 0.0 ||
                            !cabin.passengerTicketChangePoint) {
                            approach.ticketChangePointReached = true;
                        } else if (approach.ticketMoneyPointReached &&
                                   !approach.ticketChangePointReached) {
                            approach.ticketChangePointApproachSeconds +=
                                passengerServiceTimeStep;
                            if (approach.ticketChangePointApproachSeconds >=
                                passengerApproachDuration(moneyPoint, changePoint)) {
                                approach.ticketChangePointReached = true;
                            }
                        }
                        if (approach.ticketMoneyPointReached && approach.ticketChangePointReached &&
                            std::max(0.0, static_cast<double>(vehicle->variables.get(
                                              "cash_dispensed_total")) -
                                              approach.cashDispensedAtSaleStart) +
                                    1.0e-4 >=
                                approach.changeDue) {
                            approach.ticketWaitSeconds += passengerServiceTimeStep;
                            if (approach.ticketWaitSeconds >= 0.75) {
                                approach.ticketPrinted = true;
                                vehicle->variables.set("passenger_ticket_sale_active", 0.0F);
                                std::uniform_real_distribution<double> stampChance(0.0, 1.0);
                                const bool stamped =
                                    !cabin.passengerCabinStampers.empty() &&
                                    stampChance(passengerTransactionRandom_) <
                                        passengerTicketPack_.stamperProbability;
                                approach.ticketStamped = stamped;
                                if (stamped) {
                                    approach.ticketStamperConfigurationIndex = 0;
                                }
                                vehicle->variables.set("passenger_ticket_last_stamped",
                                                       stamped ? 1.0F : 0.0F);
                                if (stamped) {
                                    vehicle->variables.set(
                                        "passenger_ticket_stamper_index",
                                        static_cast<float>(
                                            cabin.passengerCabinStampers.front().index));
                                }
                            }
                        }
                    }
                    if (entryOpen && reachedEntry && approach.ticketPrinted &&
                        !approach.ticketSaleComplete) {
                        const auto transactionPose = passengerTicketTransactionPose(
                            cabin, *busPoseForTicket, *destination,
                            approach.ticketMachineConfigurationIndex,
                            approach.ticketPaymentRequired, approach.changeDue);
                        if (approach.ticketStamped &&
                            approach.ticketStamperConfigurationIndex) {
                            const auto stamperPose = passengerVehicleLocalPose(
                                *busPoseForTicket,
                                cabin.passengerCabinStampers[
                                    *approach.ticketStamperConfigurationIndex]
                                    .position);
                            if (approach.ticketStamperApproachSeconds <
                                passengerApproachDuration(transactionPose, stamperPose)) {
                                approach.ticketStamperApproachSeconds +=
                                    passengerServiceTimeStep;
                            } else if (approach.ticketStamperWaitSeconds < 0.5) {
                                approach.ticketStamperWaitSeconds += passengerServiceTimeStep;
                            } else {
                                approach.ticketStamperReturnSeconds +=
                                    passengerServiceTimeStep;
                                if (approach.ticketStamperReturnSeconds >=
                                    passengerApproachDuration(stamperPose, *destination)) {
                                    approach.ticketSaleComplete = true;
                                }
                            }
                        } else {
                            approach.ticketMachineReturnSeconds += passengerServiceTimeStep;
                            if (approach.ticketMachineReturnSeconds >=
                                passengerApproachDuration(transactionPose, *destination)) {
                                approach.ticketSaleComplete = true;
                            }
                        }
                    }
                    passengerTicketSalePending =
                        approach.ticketBought && !approach.ticketSaleComplete;
                    passengerReady = reachedEntry && approach.ticketSaleComplete;
                }
            } else if (activeStop == sharedQueueStops.end()) {
                passengerApproachProgress_.erase(vehicle.get());
            }
            if (queue != passengerNetwork_.queues.end()) {
                passengerNetwork_.advanceService(
                    service, *serviceRouteIndex, nextPassenger, passengerServiceTimeStep,
                    entryOpen && passengerReady, exitOpen,
                    (playerServiceActive && activePlayerPassengerStopServiceGeneration_ ==
                                                playerPassengerServiceGeneration_) ||
                        passengerDestinationPending || passengerTicketSalePending,
                    passengerDestinationPending, serviceStopIndex);
            } else {
                vehicle->passengers->service.advance(passengerServiceTimeStep, entryOpen, exitOpen,
                                                     false, true, nullptr, true);
            }
            for (const auto& passenger : alightingPassengersBefore) {
                if (!passenger.modelIndex || !passenger.pedestrianJourney ||
                    service.passengerCountOnSeat(passenger.seatIndex) != 0) {
                    continue;
                }
                if (activeStop == sharedQueueStops.end() || !mapRenderer_) {
                    throw std::logic_error("Alighting pedestrian has no active mapped stop");
                }
                openbus::map::MapTrafficPose busPose;
                std::optional<openbus::map::MapTrafficPose> stopPose;
                double laneOffset = 0.0;
                double vehicleWidth = vehicle->trafficWidthMeters;
                if (playerServiceActive) {
                    if (!activePlayerPassengerStopRouteIndex_ || !activePlayerPassengerStopIndex_) {
                        throw std::logic_error("Alighting player passenger has no route call");
                    }
                    const auto chassis = simulation.chassisPose();
                    busPose.position =
                        transformLocalPoint(chassis, {0.0, 0.0, vehicle->modelOffsetZ});
                    busPose.headingRadians = std::atan2(chassis.rotation[0], chassis.rotation[3]);
                    stopPose = passengerNetwork_.routes.at(*activePlayerPassengerStopRouteIndex_)
                                   .stopPose(*activePlayerPassengerStopIndex_);
                    if (stopPose) {
                        if (const auto editorPosition =
                                mapRenderer_->editorOnlyBusStopPosition(activeStop->second)) {
                            stopPose->position = *editorPosition;
                        }
                    }
                } else {
                    const auto scheduled = scheduledStopVehicles.find(vehicle.get());
                    if (scheduled == scheduledStopVehicles.end()) {
                        throw std::logic_error("Alighting scheduled passenger has no bus state");
                    }
                    const auto& state = *scheduled->second;
                    const auto sampled = openbus::map::sampleMapTrafficActorPose(
                        state.actor, state.actor.distanceMeters);
                    if (!sampled || state.bus->nextPassengerStopIndex >=
                                        state.bus->passengerStops.size()) {
                        throw std::runtime_error("Cannot sample alighting bus pose");
                    }
                    busPose = *sampled;
                    busPose.position[2] += vehicle->aiDeltaHeight;
                    const auto& stop = state.bus->passengerStops[state.bus->nextPassengerStopIndex];
                    stopPose = openbus::map::sampleMapTrafficRoute(
                        state.actor.route, stop.routeDistanceMeters, 0.0);
                    laneOffset = state.actor.lateralOffsetMeters;
                    vehicleWidth = state.actor.widthMeters;
                }
                if (!stopPose) {
                    throw std::runtime_error("Cannot sample alighting stop pose");
                }
                const auto curb = passengerCurbPose(*stopPose, mapRenderer_->leftHandTraffic(),
                                                    laneOffset, vehicleWidth);
                const auto exitDoor = vehicle->passengers->exitDoorForSeat(passenger.seatIndex);
                const auto outside = passengerConfiguredDoorQueuePose(
                                         *vehicle, busPose, mapRenderer_->leftHandTraffic(),
                                         vehicle->passengers->configuration().passengerExitPathPoints,
                                         exitDoor, 0)
                                         .value_or(curb);
                auto handoff = openbus::app::makePassengerPedestrianHandoff(
                    *passenger.pedestrianJourney, outside, busPose,
                    mapRenderer_->leftHandTraffic(), *passenger.modelIndex);
                if (!handoff) {
                    const auto alternateJourney =
                        mapRenderer_->passengerPedestrianJourneyBetween(
                            outside.position, passenger.pedestrianJourney->destination,
                            *passenger.modelIndex, 0.5, 50.0);
                    if (alternateJourney) {
                        handoff = openbus::app::makePassengerPedestrianHandoff(
                            *alternateJourney, outside, busPose,
                            mapRenderer_->leftHandTraffic(), *passenger.modelIndex);
                    }
                }
                if (handoff) {
                    alightedPassengers_.push_back(std::move(*handoff));
                } else if (reportedPassengerHandoffStops_.insert(activeStop->second).second) {
                    gameLog.Warning(
                        "No pedestrian path can be reached safely from a bus exit at stop " +
                        std::to_string(activeStop->second) +
                        "; the alighting passenger was removed.");
                }
            }
            if (service.boardingPassenger() || service.alightingPassenger()) {
                if (activeStop == sharedQueueStops.end() || mapRenderer_ == nullptr) {
                    throw std::logic_error(
                        "Active passenger movement has no mapped stop or map renderer");
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
                            true, *seat, vehicle->passengers->entryDoor(), curb.position);
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
                        const auto& queuedSeats = service.queuedAlightingSeatIndices();
                        for (std::size_t queueIndex = 0; queueIndex < queuedSeats.size();
                             ++queueIndex) {
                            const std::size_t queuedSeat = queuedSeats[queueIndex];
                            if (queuedSeat >= cabin.passengerPositions.size()) {
                                throw std::logic_error(
                                    "Queued alighting passenger has no configured seat");
                            }
                            const auto queuedSeatPose = passengerCabinSeatPose(
                                busPose, cabin.passengerPositions[queuedSeat]);
                            const auto queuedExitDoor =
                                vehicle->passengers->exitDoorForSeat(queuedSeat);
                            const auto queuedOutside = passengerConfiguredDoorQueuePose(
                                                           *vehicle, busPose,
                                                           mapRenderer_->leftHandTraffic(),
                                                           cabin.passengerExitPathPoints,
                                                           queuedExitDoor, 0)
                                                           .value_or(curb);
                            const auto queuedEndLocal =
                                passengerCabinLocalPoint(busPose, queuedOutside.position);
                            auto queuedPath = passengerCabinTravelPath(
                                cabin, busPose, queuedSeatPose.position, queuedEndLocal, false,
                                queuedExitDoor);
                            if (queuedPath.positions.empty()) {
                                queuedPath.positions = {queuedSeatPose.position,
                                                        queuedOutside.position};
                                queuedPath.segmentLinkIndices = {std::nullopt};
                            }
                            const bool queuedHasSeat =
                                cabin.passengerPositions[queuedSeat].seatHeight > 0.0;
                            const double queuedDuration = passengerCabinTravelDuration(
                                queuedPath, queuedSeatPose.position, queuedOutside.position,
                                queuedHasSeat);
                            const double queueProgress = passengerCabinQueueProgress(
                                queuedPath, queuedSeatPose.position, queuedOutside.position,
                                queuedHasSeat, queueIndex + 1);
                            service.setQueuedAlightingPassengerAnimation(
                                queuedSeat, queuedDuration, queueProgress);
                        }
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
        std::erase_if(alightedPassengers_, [&](auto& passenger) {
            return !openbus::app::advancePassengerPedestrianHandoff(passenger,
                                                                     passengerServiceTimeStep);
        });
    }
    std::vector<BusPassengerMassPoint> passengerMassPoints;
    if (playerVehicle_ != nullptr && playerVehicle_->passengers) {
        const auto& service = playerVehicle_->passengers->service;
        const auto& cabin = playerVehicle_->passengers->configuration();
        for (std::size_t seatIndex = 0; seatIndex < service.seatCount(); ++seatIndex) {
            if (service.passengerCountOnSeat(seatIndex) == 0) {
                continue;
            }
            const auto modelIndex = service.passengerModelIndexOnSeat(seatIndex);
            if (!modelIndex || scheduledPassengerHumanMassesKg_.empty()) {
                continue;
            }
            const auto& passengerMass =
                scheduledPassengerHumanMassesKg_[*modelIndex %
                                                  scheduledPassengerHumanMassesKg_.size()];
            if (passengerMass && *passengerMass > 0.0) {
                if (seatIndex >= cabin.passengerPositions.size()) {
                    throw std::logic_error("Passenger mass point has no configured seat");
                }
                const auto& seat = cabin.passengerPositions[seatIndex];
                const auto standingPoint =
                    passengerCabinStandingPoint(seat.position, seat.seatHeight);
                const auto modelPoint = passengerCabinModelPoint(standingPoint);
                passengerMassPoints.push_back(
                    {seatIndex, *passengerMass,
                     {modelPoint[0], modelPoint[1],
                      modelPoint[2] + playerVehicle_->modelOffsetZ}});
            }
        }
    }
    simulation.setPassengerMassPoints(passengerMassPoints);
    updateScripts();
    const double simulationTime = simulation.simulationTime();
    std::erase_if(passengerCoinAnimations_, [simulationTime](const PassengerCoinAnimation& animation) {
        return simulationTime - animation.startedAtSeconds > 0.8;
    });
    std::unordered_set<Vehicle*> observedCoinVehicles;
    const auto observeDispensedCoins = [&](Vehicle& vehicle,
                                           const openbus::map::MapTrafficPose& busPose) {
        observedCoinVehicles.insert(&vehicle);
        auto& observedCounts = observedPassengerCoinCounts_[&vehicle];
        observedCounts.resize(passengerCurrency_.coins.size(), 0.0);
        std::array<double, 3> localCoinPosition = {};
        if (vehicle.passengers) {
            const auto& cabin = vehicle.passengers->configuration();
            if (cabin.passengerTicketChangePoint) {
                localCoinPosition = cabin.passengerTicketChangePoint->position;
            } else if (cabin.passengerTicketMoneyPoint) {
                localCoinPosition = cabin.passengerTicketMoneyPoint->position;
            } else if (!cabin.passengerTicketSaleMachines.empty()) {
                localCoinPosition = cabin.passengerTicketSaleMachines.front().position;
            }
        }
        const auto worldCoinPosition = passengerVehicleLocalPose(busPose, localCoinPosition).position;
        for (std::size_t coinIndex = 0; coinIndex < observedCounts.size(); ++coinIndex) {
            const std::string counterName =
                "cash_coin_dispensed_count_" + std::to_string(coinIndex);
            const double currentCount = vehicle.variables.get(counterName);
            const std::size_t visualBurst =
                openbus::app::passengerCoinVisualBurstCount(observedCounts[coinIndex], currentCount);
            if (currentCount < observedCounts[coinIndex]) {
                observedCounts[coinIndex] = currentCount;
                continue;
            }
            for (std::size_t burstIndex = 0; burstIndex < visualBurst; ++burstIndex) {
                if (passengerCoinAnimations_.size() >= 128) {
                    break;
                }
                auto position = worldCoinPosition;
                const double spread =
                    (static_cast<double>(burstIndex) -
                     (static_cast<double>(visualBurst) - 1.0) * 0.5) *
                    0.025;
                position[0] += std::cos(busPose.headingRadians) * spread;
                position[1] += std::sin(busPose.headingRadians) * spread;
                passengerCoinAnimations_.push_back(
                    {position, busPose.headingRadians, simulationTime, coinIndex});
            }
            observedCounts[coinIndex] = currentCount;
        }
    };
    if (playerVehicle_ != nullptr) {
        const BodyPose chassis = simulation.chassisPose();
        openbus::map::MapTrafficPose playerPose;
        playerPose.position =
            transformLocalPoint(chassis, {0.0, 0.0, playerVehicle_->modelOffsetZ});
        playerPose.headingRadians = std::atan2(chassis.rotation[0], chassis.rotation[3]);
        observeDispensedCoins(*playerVehicle_, playerPose);
    }
    for (const TrafficVehicleState& trafficVehicle : trafficFleet_.vehicles) {
        if (trafficVehicle.vehicle == nullptr || !trafficVehicle.bus) {
            continue;
        }
        const auto busPose = openbus::map::sampleMapTrafficActorPose(
            trafficVehicle.actor, trafficVehicle.actor.distanceMeters);
        if (!busPose) {
            throw std::runtime_error("Cannot sample bus pose for passenger coin animation");
        }
        auto adjustedBusPose = *busPose;
        adjustedBusPose.position[2] += trafficVehicle.vehicle->aiDeltaHeight;
        observeDispensedCoins(*trafficVehicle.vehicle, adjustedBusPose);
    }
    std::erase_if(observedPassengerCoinCounts_, [&observedCoinVehicles](const auto& entry) {
        return !observedCoinVehicles.contains(entry.first);
    });
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
        cancelMovingPlayerPassengerService(simulation);
        if (activePlayerPassengerStop_ && !playerVehicle_->passengers->service.dwelling()) {
            playerVehicle_->passengers->service.rearmPassengerStopRequestsAfterStop(
                *activePlayerPassengerStop_);
            const std::optional<std::size_t> completedRoute = activePlayerPassengerStopRouteIndex_;
            const std::optional<std::size_t> completedStop = activePlayerPassengerStopIndex_;
            if (activePlayerPassengerStopServiceGeneration_ == playerPassengerServiceGeneration_ &&
                completedRoute && completedStop &&
                playerPassengerServiceRouteIndex_ == completedRoute &&
                *completedStop >= playerPassengerServiceNextStopIndex_) {
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
            bool advancedTrip = false;
            const auto completedStop = openbus::app::releasePlayerTimetableStopVisit(
                playerTimetableStopVisitLatch_, playerTimetable_.state->nextStopIndex,
                chassis.position, PLAYER_TIMETABLE_STOP_VISIT_RADIUS_METERS);
            if (completedStop) {
                playerTimetable_.state->nextStopIndex = *completedStop + 1;
                if (!openbus::app::nextPassengerStopWithPosition(
                        route, playerTimetable_.state->nextStopIndex, resolveEditorPosition)) {
                    advancePlayerTimetableTrip();
                    advancedTrip = true;
                }
            }
            if (playerTimetable_.state && !advancedTrip && !playerTimetableStopVisitLatch_) {
                const std::size_t currentRouteIndex =
                    playerTimetable_.state->selection.passengerRouteIndex;
                const auto& currentRoute = passengerNetwork_.routes.at(currentRouteIndex);
                const auto stopIndex = openbus::app::nextPassengerStopWithPosition(
                    currentRoute, playerTimetable_.state->nextStopIndex, resolveEditorPosition);
                if (stopIndex) {
                    const auto& stop = currentRoute.stops[*stopIndex];
                    const auto pose = currentRoute.stopPose(*stopIndex);
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
                        playerTimetableStopVisitLatch_ = openbus::app::PlayerTimetableStopVisitLatch{
                            *stopIndex, {(*visitPosition)[0], (*visitPosition)[1]}};
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
                nearestStop = passengerNetwork_.findNearestStop(
                    chassis.position, chassisHeading, playerPassengerServiceRouteIndex_, 0,
                    [this](const auto& stop) {
                        return mapRenderer_ ? mapRenderer_->editorOnlyBusStopPosition(stop.stopId)
                                            : std::nullopt;
                    });
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
                const auto destinations = openbus::app::onwardPassengerStopIds(
                    route.stops, nearestStop.stopIndex + 1, stop.stopId,
                    [&](std::size_t index) { return route.stopPose(index).has_value(); });
                const int nextStopId = destinations.empty() ? 0 : destinations.front();
                auto stopPose = route.stopPose(nearestStop.stopIndex);
                if (!stopPose || !mapRenderer_) {
                    throw std::logic_error("Cannot locate player bus alighting positions");
                }
                if (const auto editorPosition = mapRenderer_->editorOnlyBusStopPosition(stop.stopId)) {
                    stopPose->position = *editorPosition;
                }
                const auto chassis = simulation.chassisPose();
                openbus::map::MapTrafficPose busPose;
                busPose.position =
                    transformLocalPoint(chassis, {0.0, 0.0, playerVehicle_->modelOffsetZ});
                busPose.headingRadians = std::atan2(chassis.rotation[0], chassis.rotation[3]);
                playerVehicle_->passengers->service.setWaitingPassengers(
                    stop.stopId, nextStopId == 0 ? 0 : queue->second);
                if (!playerVehicle_->passengers->service.arrive(
                        {.stopId = stop.stopId,
                         .waitingPassengers = 0,
                         .passengersAlighting = stop.passengersAlighting,
                         .dwellSeconds = stop.dwellSeconds,
                         .nextStopId = nextStopId,
                         .onwardStopIds = destinations,
                         .alightingPositions = passengerAlightingPositions(
                             *playerVehicle_, busPose, *stopPose, mapRenderer_->leftHandTraffic(),
                             0.0, playerVehicle_->trafficWidthMeters)})) {
                    throw std::logic_error("Player bus reached a passenger stop while already "
                                           "dwelling");
                }
                if (playerVehicle_->passengers->service.outOfRangeAssignedPassengers() > 0) {
                    gameLog.Warning(
                        "Player bus reached stop " + std::to_string(stop.stopId) +
                        " with assigned passenger destinations more than 200 m from their exits." +
                        (stop.passengersAlighting > 0
                             ? " Forced alighting at this stop can override the distance check."
                             : " Those passengers remain aboard."));
                }
                playerVehicle_->passengers->service.setEligibleWaitingPassengers(0);
                activePlayerPassengerStop_ = stop.stopId;
                passengerApproachProgress_.erase(playerVehicle_);
                passengerQueueApproachProgress_.erase(playerVehicle_);
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
