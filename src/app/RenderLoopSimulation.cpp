// Script ticking and player/physics variable synchronisation.

#include "app/RenderLoopInternal.h"
#include "app/MapTrafficWorker.h"
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

const openbus::map::MapAiTimetableProfile* RenderLoop::trafficTimetableProfileForDeparture(
    const TrafficVehicleState& state, std::size_t departureIndex) const {
    if (state.timetableProfiles.empty() ||
        departureIndex >= state.scheduledDepartureProfileIndices.size()) {
        return nullptr;
    }
    const int requestedProfile =
        std::max(0, state.scheduledDepartureProfileIndices[departureIndex]);
    const std::size_t profileIndex =
        std::min(static_cast<std::size_t>(requestedProfile), state.timetableProfiles.size() - 1);
    return &state.timetableProfiles[profileIndex];
}

void RenderLoop::publishTrafficTimetableData(Vehicle& vehicle, const TrafficVehicleState& state,
                                             std::size_t departureIndex) const {
    vehicle.variables.set("tt_terminus_index", static_cast<float>(state.timetableTerminusIndex));
    vehicle.variables.set("tt_delay", 0.0F);
    vehicle.variables.set("tt_busstop_count", static_cast<float>(state.passengerStops.size()));
    const std::size_t currentStopIndex =
        state.passengerStops.empty()
            ? 0
            : std::min(state.nextPassengerStopIndex, state.passengerStops.size() - 1);
    vehicle.variables.set("tt_busstop_index", static_cast<float>(currentStopIndex));
    const openbus::map::MapAiTimetableProfile* profile =
        trafficTimetableProfileForDeparture(state, departureIndex);
    const double departureSeconds =
        departureIndex < state.scheduledDepartureMinutes.size()
            ? state.scheduledDepartureMinutes[departureIndex] * 60.0
            : 0.0;
    constexpr double secondsPerDay = 24.0 * 60.0 * 60.0;
    for (std::size_t stopIndex = 0; stopIndex < state.passengerStops.size(); ++stopIndex) {
        const openbus::map::MapTrafficPassengerStop& stop = state.passengerStops[stopIndex];
        vehicle.variables.setString("tt_busstop_name_" + std::to_string(stopIndex), stop.name);
        double arrivalSeconds = 0.0;
        double departureAtStopSeconds = 0.0;
        if (profile != nullptr &&
            stop.timetableStationIndex < profile->arrivalOffsetSeconds.size() &&
            stop.timetableStationIndex < profile->departureOffsetSeconds.size()) {
            arrivalSeconds =
                std::fmod(departureSeconds +
                              profile->arrivalOffsetSeconds[stop.timetableStationIndex],
                          secondsPerDay);
            departureAtStopSeconds =
                std::fmod(departureSeconds +
                              profile->departureOffsetSeconds[stop.timetableStationIndex],
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
    for (TrafficVehicleState& state : trafficVehicles_) {
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
                    state.pendingInitializationVehicle->variables.setString(
                        "tt_line_string", state.timetableLineName);
                    state.pendingInitializationVehicle->variables.setString(
                        "tt_busstop_name", state.timetableDestinationName);
                    state.pendingInitializationVehicle->variables.set(
                        "tt_busstop_count", static_cast<float>(state.passengerStops.size()));
                    state.pendingInitializationVehicle->variables.set("tt_busstop_index", 0.0F);
                    for (std::size_t stopIndex = 0; stopIndex < state.passengerStops.size();
                         ++stopIndex) {
                        state.pendingInitializationVehicle->variables.setString(
                            "tt_busstop_name_" + std::to_string(stopIndex),
                            state.passengerStops[stopIndex].name);
                    }
                    publishTrafficTimetableData(*state.pendingInitializationVehicle, state,
                                                 state.nextScheduledDepartureIndex);
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
    const auto scheduledDelaySeconds = [&](const TrafficVehicleState& state,
                                           double simulationTime) {
        if (!state.scheduledTripActive || state.passengerStops.empty()) {
            return 0.0;
        }
        const MapAiTimetableProfile* profile =
            trafficTimetableProfileForDeparture(state, state.activeTimetableDepartureIndex);
        if (profile == nullptr) {
            return 0.0;
        }
        const auto arrivalOffset = [&](std::size_t stopIndex) {
            const std::size_t stationIndex =
                state.passengerStops[stopIndex].timetableStationIndex;
            return stationIndex < profile->arrivalOffsetSeconds.size()
                       ? profile->arrivalOffsetSeconds[stationIndex]
                       : 0.0;
        };
        const auto departureOffset = [&](std::size_t stopIndex) {
            const std::size_t stationIndex =
                state.passengerStops[stopIndex].timetableStationIndex;
            return stationIndex < profile->departureOffsetSeconds.size()
                       ? profile->departureOffsetSeconds[stationIndex]
                       : 0.0;
        };
        const double elapsedSinceDeparture =
            std::max(0.0, simulationTime - state.activeScheduledDepartureElapsedSeconds);
        const double distance = state.actor.distanceMeters;
        constexpr double stopPositionToleranceMeters = 0.25;
        const auto dwellProgress = [&](std::size_t stopIndex) {
            return std::clamp(elapsedSinceDeparture, arrivalOffset(stopIndex),
                              departureOffset(stopIndex));
        };
        const double firstStopDistance = state.passengerStops.front().routeDistanceMeters;
        if (distance <= firstStopDistance + stopPositionToleranceMeters) {
            const double expected =
                std::abs(distance - firstStopDistance) <= stopPositionToleranceMeters
                    ? dwellProgress(0)
                    : departureOffset(0);
            return elapsedSinceDeparture - expected;
        }
        for (std::size_t stopIndex = 0; stopIndex + 1 < state.passengerStops.size();
             ++stopIndex) {
            const double fromDistance =
                state.passengerStops[stopIndex].routeDistanceMeters;
            const double toDistance =
                state.passengerStops[stopIndex + 1].routeDistanceMeters;
            if (std::abs(distance - fromDistance) <= stopPositionToleranceMeters) {
                return elapsedSinceDeparture - dwellProgress(stopIndex);
            }
            if (distance < toDistance && toDistance > fromDistance) {
                const double fraction =
                    std::clamp((distance - fromDistance) / (toDistance - fromDistance), 0.0, 1.0);
                const double expected =
                    departureOffset(stopIndex) +
                    (arrivalOffset(stopIndex + 1) - departureOffset(stopIndex)) * fraction;
                return elapsedSinceDeparture - expected;
            }
        }
        const std::size_t lastStopIndex = state.passengerStops.size() - 1;
        const double lastStopDistance =
            state.passengerStops[lastStopIndex].routeDistanceMeters;
        const double expected =
            std::abs(distance - lastStopDistance) <= stopPositionToleranceMeters
                ? dwellProgress(lastStopIndex)
                : departureOffset(lastStopIndex);
        return elapsedSinceDeparture - expected;
    };
    if (!trafficWorker_) {
        trafficWorker_ = std::make_unique<openbus::app::MapTrafficWorker>();
    }
    TraceScope resultTrace("traffic",
                           "RenderLoop::updateTrafficDriving.workerSchedulingAndResultSync");
    double aiRateHz = aiSimulationRateHz_;
    if (aiRateHz <= 0.0) {
        aiRateHz = (averageRenderFpsHz_ > 0.0 ? averageRenderFpsHz_ : 60.0) / aiRateDivisor_;
    }
    if (aiMaximumRateHz_ > 0.0) {
        aiRateHz = std::min(aiRateHz, aiMaximumRateHz_);
    }
    constexpr double aiWorkerBudgetFraction = 0.8;
    const double averageWorkerTickSeconds = trafficWorker_->averageTickProcessingSeconds();
    currentAiWorkerTickMilliseconds_ = averageWorkerTickSeconds * 1000.0;
    if (averageWorkerTickSeconds > 0.0) {
        aiRateHz = std::min(aiRateHz, aiWorkerBudgetFraction / averageWorkerTickSeconds);
    }
    const double requestedAiRateHz = std::clamp(aiRateHz, 0.01, 1000.0);
    if (currentAiSimulationRateHz_ <= 0.0) {
        currentAiSimulationRateHz_ = requestedAiRateHz;
    } else {
        constexpr double rateSmoothing = 0.2;
        currentAiSimulationRateHz_ +=
            (requestedAiRateHz - currentAiSimulationRateHz_) * rateSmoothing;
    }
    if (configuredAiSimulationRateHz_ <= 0.0 ||
        std::abs(currentAiSimulationRateHz_ - configuredAiSimulationRateHz_) >=
            std::max(0.01, configuredAiSimulationRateHz_ * 0.01)) {
        trafficWorker_->setSimulationRateHz(currentAiSimulationRateHz_);
        configuredAiSimulationRateHz_ = currentAiSimulationRateHz_;
    }
    trafficWorkerInputAccumulatorSeconds_ += elapsedSeconds;
    const double trafficInputIntervalSeconds = 1.0 / currentAiSimulationRateHz_;
    const bool trafficInputDue =
        trafficWorkerInputAccumulatorSeconds_ >= trafficInputIntervalSeconds;
    double trafficInputElapsedSeconds = 0.0;
    if (trafficInputDue) {
        trafficInputElapsedSeconds = trafficWorkerInputAccumulatorSeconds_;
        trafficWorkerInputAccumulatorSeconds_ = 0.0;
    }
    using PopulationDecision = openbus::app::MapTrafficWorker::PopulationDecision;
    std::vector<std::optional<PopulationDecision>> populationDecisionsByVehicle(
        trafficVehicles_.size());
    std::vector<std::optional<PopulationDecision>> populationDecisionsByHuman(
        humanTrafficActors_.size());
    if (trafficInputDue) {
        if (auto decisions = trafficWorker_->takePopulationDecisions()) {
            for (const auto& decision : *decisions) {
                auto& decisionsByState =
                    decision.isHuman ? populationDecisionsByHuman : populationDecisionsByVehicle;
                if (decision.stateIndex < decisionsByState.size()) {
                    decisionsByState[decision.stateIndex] = decision;
                }
            }
        }
    }
    if (auto results = trafficWorker_->takeLatestResults()) {
        for (const auto& result : results->actors) {
            if (result.stateIndex >= trafficVehicles_.size()) {
                continue;
            }
            TrafficVehicleState& state = trafficVehicles_[result.stateIndex];
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
            if (result.stateIndex >= trafficVehicles_.size()) {
                continue;
            }
            TrafficVehicleState& state = trafficVehicles_[result.stateIndex];
            if (state.workerGeneration != result.generation || !state.populationEnabled ||
                state.vehicle == nullptr || !state.vehicle->mapTrafficTileActive) {
                continue;
            }
            const bool blockerGenerationMatches =
                result.blockingActorStateIndex.has_value() &&
                result.blockingActorGeneration.has_value() &&
                *result.blockingActorStateIndex < trafficVehicles_.size() &&
                trafficVehicles_[*result.blockingActorStateIndex].workerGeneration ==
                    *result.blockingActorGeneration;
            state.blockingActorStateIndex =
                blockerGenerationMatches ? result.blockingActorStateIndex : std::nullopt;
            state.blockingActorGeneration =
                blockerGenerationMatches ? result.blockingActorGeneration : std::nullopt;
            state.actor.blockingActor =
                state.blockingActorStateIndex.has_value()
                    ? &trafficVehicles_[*state.blockingActorStateIndex].actor
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
        for (std::size_t index = 0; index < trafficVehicles_.size(); ++index) {
            TrafficVehicleState& state = trafficVehicles_[index];
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
            const auto pose = trafficVehiclePose(state);
            if (!pose) {
                throw std::runtime_error("Could not sample a map traffic tile pose");
            }
            state.vehicle->mapTrafficTileActive = poseOnActiveTile(*pose);
            if (!state.vehicle->mapTrafficTileActive) {
                state.actor.speedMetersPerSecond = 0.0;
                if (!state.isBus) {
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
                    if (trafficSpawnDiagnosticsEnabled_) {
                        ++roadTrafficRecycledOutsideTiles_;
                    }
                }
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
    {
        TraceScope stopTrace("traffic", "RenderLoop::updateTrafficDriving.scheduledStopSync");
        for (TrafficVehicleState& state : trafficVehicles_) {
            if (!state.isBus || state.vehicle == nullptr || !state.populationEnabled) {
                continue;
            }
            if (state.passengerStopArrived) {
                const int completedStopId = state.vehicle->passengerService.currentStopId();
                const auto queue = scheduledPassengerQueues_.find(completedStopId);
                if (queue != scheduledPassengerQueues_.end()) {
                    queue->second =
                        state.vehicle->passengerService.waitingPassengers(completedStopId);
                }
            }
            if (state.passengerStopArrived && !state.vehicle->passengerService.dwelling()) {
                state.passengerStopArrived = false;
                ++state.nextPassengerStopIndex;
                state.vehicle->variables.set(
                    "tt_busstop_index",
                    static_cast<float>(std::min(state.nextPassengerStopIndex,
                                                state.passengerStops.empty()
                                                    ? std::size_t{0}
                                                    : state.passengerStops.size() - 1)));
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
        playerBusMoving =
            playerSpeedMetersPerSecond > MAP_TRAFFIC_STOPPED_SPEED_METERS_PER_SECOND;
        playerPredictedForwardDistanceMeters =
            std::min(playerSpeedMetersPerSecond * 4.0, 60.0);
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
    std::unordered_map<const MapTrafficActor*, const TrafficVehicleState*> trafficStateByActor;
    {
        TraceScope actorIndexTrace("traffic", "RenderLoop::updateTrafficDriving.actorStateIndex");
        trafficStateByActor.reserve(trafficVehicles_.size());
        for (const TrafficVehicleState& state : trafficVehicles_) {
            trafficStateByActor.emplace(&state.actor, &state);
        }
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
                 actor.blockingObstacleIndex == playerObstacleIndex)) {
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
        roadRouteRefreshElapsedSeconds_ += trafficInputElapsedSeconds;
        const auto activeRoadCount = static_cast<std::size_t>(std::count_if(
            trafficVehicles_.begin(), trafficVehicles_.end(), [](const TrafficVehicleState& state) {
                return !state.isBus && state.populationEnabled;
            }));
        if (mapRenderer_ != nullptr && !activeTiles.empty() &&
            activeRoadCount < targetRoadTrafficCount_ && roadRouteRefreshElapsedSeconds_ >= 0.05 &&
            !trafficVehicles_.empty()) {
            roadRouteRefreshElapsedSeconds_ = 0.0;
            const auto& coordinates = mapRenderer_->activeTileCoordinates();
            for (std::size_t checked = 0; checked < trafficVehicles_.size(); ++checked) {
                const std::size_t index = roadRouteRefreshCursor_++ % trafficVehicles_.size();
                TrafficVehicleState& state = trafficVehicles_[index];
                if (state.isBus || state.populationEnabled || state.vehicle == nullptr ||
                    state.vehicleLoadFailed || state.deadlockRemovalCommitted) {
                    continue;
                }
                bool hasUsableActiveCandidate = false;
                for (std::size_t candidateIndex = 0; candidateIndex < state.spawnCandidates.size();
                     ++candidateIndex) {
                    const auto& candidate = state.spawnCandidates[candidateIndex];
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
                if (++roadRouteRefreshSeed_ == 0) {
                    ++roadRouteRefreshSeed_;
                }
                if (trafficSpawnDiagnosticsEnabled_) {
                    ++roadRouteRefreshAttempts_;
                }
                const auto lane = mapRenderer_->selectTrafficSpawnLaneInTiles(
                    filter, roadRouteRefreshSeed_, coordinates);
                if (!lane) {
                    break;
                }
                auto route = mapRenderer_->buildDrivingRoute(
                    lane->splineId, lane->laneSelector, roadRouteRefreshSeed_,
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
                state.spawnCandidates = std::move(candidates);
                state.rejectedSpawnCandidateIndices.clear();
                state.activeSpawnCandidateIndex.reset();
                state.cachedPose.reset();
                state.blockingActorStateIndex.reset();
                state.blockingActorGeneration.reset();
                state.workerActorTemplate = std::make_shared<const MapTrafficActor>(state.actor);
                ++state.workerGeneration;
                if (trafficSpawnDiagnosticsEnabled_) {
                    ++roadRouteRefreshSuccesses_;
                }
                break;
            }
        }
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
            const auto pose = sampleMapTrafficActorPose(state.actor, state.actor.distanceMeters);
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
        populationCandidates.reserve(trafficVehicles_.size() + humanTrafficActors_.size());
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
            const bool canDespawn =
                human.populationEnabled && hidden(human.actor, human.actor.distanceMeters);
            populationCandidates.push_back({index, human.workerGeneration, std::nullopt, true,
                                            false, human.populationEnabled, canSpawn, canDespawn});
        }
        for (const HumanTrafficState& human : humanTrafficActors_) {
            if (human.populationEnabled) {
                external.push_back(humanFootprint(human));
            }
        }
        humanAdmissionTrace.finish();

        struct RoadSpawnLocation {
            double distanceMeters = 0.0;
            MapTrafficPose pose;
            std::size_t candidateIndex = 0;
            double priorityDistanceMeters = std::numeric_limits<double>::infinity();
        };
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
            std::optional<std::pair<int, int>> firstInactiveCandidateTile;
        };
        const auto isInsidePlayerFrontExclusion = [&](const MapTrafficPose& pose,
                                                       double vehicleWidthMeters) {
            return playerVehicle_ != nullptr && playerBusMoving &&
                   openbus::map::mapTrafficSpawnInsideForwardExclusion(
                       playerSpawnReferencePose, playerVehicle_->trafficLengthMeters,
                       playerVehicle_->trafficWidthMeters, pose, vehicleWidthMeters, 30.0);
        };
        const auto spawnPriorityForPose = [&](const MapTrafficPose& pose) {
            return hasPlayerSpawnReferencePose
                       ? openbus::map::mapTrafficSpawnPriorityDistanceMeters(
                             playerSpawnReferencePose, pose,
                             playerPredictedForwardDistanceMeters)
                       : 0.0;
        };
        const auto findRoadSpawn =
            [&](const TrafficVehicleState& state,
                RoadSpawnDiagnostics* diagnostics) -> std::optional<RoadSpawnLocation> {
            if (diagnostics != nullptr) {
                ++diagnostics->vehiclesChecked;
            }
            std::optional<RoadSpawnLocation> bestSpawn;
            for (std::size_t candidateIndex = 0; candidateIndex < state.spawnCandidates.size();
                 ++candidateIndex) {
                if (std::ranges::find(state.rejectedSpawnCandidateIndices, candidateIndex) !=
                    state.rejectedSpawnCandidateIndices.end()) {
                    if (diagnostics != nullptr) {
                        ++diagnostics->rejectedCandidates;
                    }
                    continue;
                }
                const MapTrafficSpawnCandidate& spawn = state.spawnCandidates[candidateIndex];
                if (diagnostics != nullptr) {
                    ++diagnostics->candidatesChecked;
                }
                if (!activeTiles.contains({spawn.tileX, spawn.tileY})) {
                    if (diagnostics != nullptr) {
                        ++diagnostics->inactiveCandidateTiles;
                        if (!diagnostics->firstInactiveCandidateTile) {
                            diagnostics->firstInactiveCandidateTile =
                                std::pair{spawn.tileX, spawn.tileY};
                        }
                    }
                    continue;
                }
                const auto pose = sampleMapTrafficActorPose(state.actor, spawn.distanceMeters);
                if (!pose) {
                    throw std::runtime_error("Could not sample a road traffic spawn position");
                }
                if (!poseOnActiveTile(*pose)) {
                    if (diagnostics != nullptr) {
                        ++diagnostics->posesOutsideActiveTiles;
                    }
                    continue;
                }
                if (isInsidePlayerFrontExclusion(*pose, state.actor.widthMeters)) {
                    if (diagnostics != nullptr) {
                        ++diagnostics->frontZoneRejectedCandidates;
                    }
                    continue;
                }
                if (!mapTrafficSpawnInView_ && !hiddenAtPose(state.actor, *pose)) {
                    if (diagnostics != nullptr) {
                        ++diagnostics->visibleCandidates;
                    }
                    continue;
                }
                const MapTrafficObstacle candidate{*pose, state.actor.lengthMeters,
                                                   state.actor.widthMeters};
                if (mapTrafficHasClearance(candidate, occupied, 3.0)) {
                    if (diagnostics != nullptr) {
                        ++diagnostics->clearCandidates;
                    }
                    const double priorityDistanceMeters = spawnPriorityForPose(*pose);
                    if (!bestSpawn.has_value() ||
                        priorityDistanceMeters < bestSpawn->priorityDistanceMeters) {
                        bestSpawn = RoadSpawnLocation{spawn.distanceMeters, *pose, candidateIndex,
                                                      priorityDistanceMeters};
                    }
                }
                if (diagnostics != nullptr) {
                    ++diagnostics->obstructedCandidates;
                }
            }
            return bestSpawn;
        };
        const auto logBusAdmissionWaiting = [&](TrafficVehicleState& state,
                                                const MapTrafficPose& entryPose, bool entryHidden,
                                                bool entryClear, bool entryOnActiveTile,
                                                bool frontBlocked) {
            const unsigned blockerMask =
                (entryHidden ? 0u : 1u) | (entryClear ? 0u : 2u) |
                (entryOnActiveTile ? 0u : 4u) | (frontBlocked ? 8u : 0u);
            const double cameraDistanceMeters =
                lastMainCameraPoseValid_
                    ? std::hypot(
                          std::hypot(entryPose.position[0] - lastMainCameraPose_.position[0],
                                     entryPose.position[1] - lastMainCameraPose_.position[1]),
                          entryPose.position[2] - lastMainCameraPose_.position[2])
                    : 0.0;
            if (blockerMask != state.lastScheduledAdmissionDiagnosticBlockerMask ||
                !lastMainCameraPoseValid_ ||
                std::abs(cameraDistanceMeters -
                         state.lastScheduledAdmissionDiagnosticCameraDistanceMeters) >= 50.0) {
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
                              std::to_string(state.nextScheduledDepartureElapsedSeconds) + ": " +
                              state.actor.modelPath);
                state.lastScheduledAdmissionDiagnosticBlockerMask = blockerMask;
                state.lastScheduledAdmissionDiagnosticCameraDistanceMeters = cameraDistanceMeters;
            }
        };

        {
            TraceScope applyPopulationTrace(
                "traffic", "RenderLoop::updateTrafficDriving.applyPopulationDecisions");
            std::vector<std::size_t> populationDecisionOrder;
            populationDecisionOrder.reserve(trafficVehicles_.size());
            for (std::size_t index = 0; index < trafficVehicles_.size(); ++index) {
                populationDecisionOrder.push_back(index);
            }
            std::stable_sort(populationDecisionOrder.begin(), populationDecisionOrder.end(),
                             [&](std::size_t first, std::size_t second) {
                                 return trafficVehicles_[first].spawnPriorityMeters <
                                        trafficVehicles_[second].spawnPriorityMeters;
                             });
            for (const std::size_t stateIndex : populationDecisionOrder) {
                auto& state = trafficVehicles_[stateIndex];
                if (state.vehicle == nullptr || state.vehicleLoadFailed ||
                    state.deadlockRemovalCommitted) {
                    continue;
                }
                const auto& decision = populationDecisionsByVehicle[stateIndex];
                if (state.populationEnabled && decision.has_value() &&
                    decision->generation == state.workerGeneration && decision->despawn &&
                    hidden(state.actor, state.actor.distanceMeters)) {
                    state.populationEnabled = false;
                    ++state.workerGeneration;
                    state.vehicle->mapTrafficRouteFinished = true;
                    finishScheduledTrip(state);
                    state.deadlocked = false;
                }
                if (state.populationEnabled || !decision.has_value() ||
                    decision->generation != state.workerGeneration || !decision->spawn) {
                    continue;
                }
                std::optional<RoadSpawnLocation> spawn;
                if (state.isBus) {
                    const auto entryPose = sampleMapTrafficActorPose(state.actor, 0.0);
                    if (!entryPose) {
                        throw std::runtime_error("Could not sample a map traffic entry pose");
                    }
                    if (!mapTrafficScheduledDepartureDue(
                            currentSimulationTime, state.nextScheduledDepartureElapsedSeconds)) {
                        continue;
                    }
                    const bool entryHidden =
                        mapTrafficSpawnInView_ || hiddenAtPose(state.actor, *entryPose);
                    const bool frontBlocked =
                        isInsidePlayerFrontExclusion(*entryPose, state.actor.widthMeters);
                    const bool entryClear =
                        !frontBlocked && canSpawnMapTrafficEntry(state.actor, true, occupied);
                    const bool entryOnActiveTile = poseOnActiveTile(*entryPose);
                    if (!entryHidden || !entryClear || !entryOnActiveTile) {
                        logBusAdmissionWaiting(state, *entryPose, entryHidden, entryClear,
                                               entryOnActiveTile, frontBlocked);
                        continue;
                    }
                    spawn = RoadSpawnLocation{0.0, *entryPose, 0,
                                              spawnPriorityForPose(*entryPose)};
                } else {
                    spawn = findRoadSpawn(state, nullptr);
                    if (!spawn.has_value()) {
                        continue;
                    }
                }
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
                state.lastScheduledProgressDiagnosticDistanceMeters = state.actor.distanceMeters;
                state.movedSinceSpawn = false;
                state.activeSpawnCandidateIndex =
                    state.isBus ? std::nullopt : std::optional<std::size_t>(spawn->candidateIndex);
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
                gameLog.Log(std::string(state.isBus ? "Scheduled map AI bus spawned: "
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
                if (state.isBus) {
                    state.nextPassengerStopIndex = 0;
                    while (state.nextPassengerStopIndex < state.passengerStops.size() &&
                           state.passengerStops[state.nextPassengerStopIndex].routeDistanceMeters +
                                   0.05 <
                               state.actor.distanceMeters) {
                        ++state.nextPassengerStopIndex;
                    }
                    state.vehicle->variables.set(
                        "tt_busstop_index",
                        static_cast<float>(std::min(state.nextPassengerStopIndex,
                                                    state.passengerStops.empty()
                                                        ? std::size_t{0}
                                                        : state.passengerStops.size() - 1)));
                    state.passengerStopArrived = false;
                    state.lastScheduledDepartureMinutes =
                        state.scheduledDepartureMinutes[state.nextScheduledDepartureIndex];
                    state.scheduledTripActive = true;
                    state.activeTimetableDepartureIndex =
                        state.nextScheduledDepartureIndex;
                    state.activeScheduledDepartureElapsedSeconds =
                        state.nextScheduledDepartureElapsedSeconds;
                    publishTrafficTimetableData(*state.vehicle, state,
                                                state.activeTimetableDepartureIndex);
                    for (const MapTrafficPassengerStop& stop : state.passengerStops) {
                        const auto queue = scheduledPassengerQueues_.find(stop.stopId);
                        const auto initialQueue =
                            scheduledPassengerInitialQueues_.find(stop.stopId);
                        if (queue == scheduledPassengerQueues_.end() ||
                            initialQueue == scheduledPassengerInitialQueues_.end()) {
                            throw std::logic_error(
                                "Scheduled bus passenger stop has no initialized waiting queue");
                        }
                        queue->second = mapScheduledPassengerQueueAfterDeparture(
                            queue->second, initialQueue->second);
                    }
                    advanceScheduledDeparture(state, state.nextScheduledDepartureElapsedSeconds);
                    gameLog.Log("Scheduled map AI bus departed from endpoint spline=" +
                                std::to_string(spawn->pose.splineId) + " at timetable minute=" +
                                std::to_string(state.lastScheduledDepartureMinutes) + ": " +
                                state.actor.modelPath);
                }
                occupied.push_back(
                    {spawn->pose, state.actor.lengthMeters, state.actor.widthMeters});
            }
        }

        RoadSpawnDiagnostics roadSpawnDiagnostics;
        std::size_t loadedRoadVehicles = 0;
        std::size_t pendingRoadVehicleLoads = 0;
        std::size_t failedRoadVehicleLoads = 0;
        std::size_t activeRoadVehicles = 0;
        TraceScope candidateTrace("traffic",
                                  "RenderLoop::updateTrafficDriving.buildPopulationPlan");
        for (std::size_t stateIndex = 0; stateIndex < trafficVehicles_.size(); ++stateIndex) {
            TrafficVehicleState& state = trafficVehicles_[stateIndex];
            state.spawnPriorityMeters = std::numeric_limits<double>::infinity();
            if (!state.isBus) {
                loadedRoadVehicles += state.vehicle != nullptr;
                pendingRoadVehicleLoads +=
                    state.pendingVehicle.valid() || state.pendingInitializationVehicle != nullptr;
                failedRoadVehicleLoads += state.vehicleLoadFailed;
                activeRoadVehicles += state.populationEnabled;
            }
            if (state.vehicle == nullptr || state.vehicleLoadFailed ||
                state.deadlockRemovalCommitted) {
                continue;
            }
            bool canSpawn = false;
            if (!state.populationEnabled) {
                if (state.isBus) {
                    const auto entryPose = sampleMapTrafficActorPose(state.actor, 0.0);
                    if (!entryPose) {
                        throw std::runtime_error("Could not sample a map traffic entry pose");
                    }
                    if (mapTrafficScheduledDepartureDue(
                            currentSimulationTime, state.nextScheduledDepartureElapsedSeconds)) {
                        const bool entryHidden =
                            mapTrafficSpawnInView_ || hiddenAtPose(state.actor, *entryPose);
                        const bool frontBlocked =
                            isInsidePlayerFrontExclusion(*entryPose, state.actor.widthMeters);
                        const bool entryClear = !frontBlocked &&
                                                canSpawnMapTrafficEntry(state.actor, true,
                                                                        occupied);
                        const bool entryOnActiveTile = poseOnActiveTile(*entryPose);
                        canSpawn = entryHidden && entryClear && entryOnActiveTile;
                        if (canSpawn) {
                            state.spawnPriorityMeters = spawnPriorityForPose(*entryPose);
                        }
                        if (!canSpawn) {
                            logBusAdmissionWaiting(state, *entryPose, entryHidden, entryClear,
                                                   entryOnActiveTile, frontBlocked);
                        }
                    }
                } else {
                    const auto roadSpawn =
                        findRoadSpawn(state, trafficSpawnDiagnosticsEnabled_ ? &roadSpawnDiagnostics
                                                                             : nullptr);
                    canSpawn = roadSpawn.has_value();
                    if (roadSpawn.has_value()) {
                        state.spawnPriorityMeters = roadSpawn->priorityDistanceMeters;
                    }
                }
            }
            const bool canDespawn =
                state.populationEnabled && hidden(state.actor, state.actor.distanceMeters);
            populationCandidates.push_back({stateIndex, state.workerGeneration,
                                            state.actor.unscheduledGroupIndex, false, state.isBus,
                                            state.populationEnabled, canSpawn, canDespawn,
                                            state.spawnPriorityMeters});
        }
        trafficWorker_->submitPopulationPlan(
            std::move(populationCandidates), targetRoadTrafficCount_, targetBusTrafficCount_,
            targetHumanTrafficCount_, targetRoadGroupPopulations_, useGroupRoadPopulation_);
        if (trafficSpawnDiagnosticsEnabled_) {
            trafficSpawnDiagnosticElapsedSeconds_ += trafficInputElapsedSeconds;
            if (trafficSpawnDiagnosticElapsedSeconds_ >= 1.0) {
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
                    std::to_string(targetRoadTrafficCount_) +
                    ", queued=" + std::to_string(trafficVehicles_.size()) +
                    ", loaded=" + std::to_string(loadedRoadVehicles) +
                    ", pendingLoads=" + std::to_string(pendingRoadVehicleLoads) +
                    ", failedLoads=" + std::to_string(failedRoadVehicleLoads) +
                    ", active=" + std::to_string(activeRoadVehicles) +
                    ", recycledOutsideTiles=" + std::to_string(roadTrafficRecycledOutsideTiles_) +
                    ", localRouteAttempts=" + std::to_string(roadRouteRefreshAttempts_) +
                    ", localRoutesBuilt=" + std::to_string(roadRouteRefreshSuccesses_) +
                    ", activeTiles=" + std::to_string(activeTiles.size()) +
                    ", checkedVehicles=" + std::to_string(roadSpawnDiagnostics.vehiclesChecked) +
                    ", candidates=" + std::to_string(roadSpawnDiagnostics.candidatesChecked) +
                    ", rejected=" + std::to_string(roadSpawnDiagnostics.rejectedCandidates) +
                    ", inactiveCandidateTiles=" +
                    std::to_string(roadSpawnDiagnostics.inactiveCandidateTiles) +
                    ", posesOutsideActiveTiles=" +
                    std::to_string(roadSpawnDiagnostics.posesOutsideActiveTiles) +
                    ", frontZoneRejected=" +
                    std::to_string(roadSpawnDiagnostics.frontZoneRejectedCandidates) +
                    ", firstInactiveCandidateTile=" + firstInactiveCandidateTile +
                    ", activeTileCoordinates=" + activeTileCoordinates +
                    ", visible=" + std::to_string(roadSpawnDiagnostics.visibleCandidates) +
                    ", obstructed=" + std::to_string(roadSpawnDiagnostics.obstructedCandidates) +
                    ", clear=" + std::to_string(roadSpawnDiagnostics.clearCandidates));
                trafficSpawnDiagnosticElapsedSeconds_ = 0.0;
                roadTrafficRecycledOutsideTiles_ = 0;
                roadRouteRefreshAttempts_ = 0;
                roadRouteRefreshSuccesses_ = 0;
            }
        }
    }
    std::vector<std::shared_ptr<MapTrafficSignalController>> signalControllers;
    std::vector<MapTrafficSignalBusPriorityRequest> busPriorityRequests;
    if (mapRenderer_ != nullptr) {
        if (trafficInputDue) {
            TraceScope signalTrace("traffic", "RenderLoop::updateTrafficDriving.signalSnapshot");
            signalControllers = mapRenderer_->trafficSignalControllers();
            const auto playerPosition = streamingPosition(simulation);
            busPriorityRequests = mapRenderer_->trafficSignalBusPriorityRequests(
                std::array<double, 2>{playerPosition[0], playerPosition[1]});
        }
        mapRenderer_->updateTrafficSignalScripts(currentSimulationTime);
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
                    for (std::size_t index = 0; index < trafficVehicles_.size(); ++index) {
                        auto& state = trafficVehicles_[index];
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
                                    *state.blockingActorStateIndex < trafficVehicles_.size() &&
                                    trafficVehicles_[*state.blockingActorStateIndex]
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
                    for (std::size_t index = 0; index < trafficVehicles_.size(); ++index) {
                        TrafficVehicleState& state = trafficVehicles_[index];
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
                trafficWorker_->submit(std::move(actors), std::move(humans), external,
                                       std::move(humanObstacles), std::move(signalControllers),
                                       std::move(busPriorityRequests), trafficInputElapsedSeconds,
                                       currentSimulationTime);
            }
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
            if (trafficVehicle.isBus && trafficVehicle.scheduledTripActive &&
                trafficVehicle.actor.distanceMeters -
                        trafficVehicle.lastScheduledProgressDiagnosticDistanceMeters >=
                    100.0) {
                gameLog.Debug("Scheduled map AI bus route progress: timetable minute=" +
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
    for (TrafficVehicleState& state : trafficVehicles_) {
        if (!state.isBus || state.vehicle == nullptr) {
            continue;
        }
        const double delaySeconds = scheduledDelaySeconds(state, currentSimulationTime);
        state.vehicle->variables.set(
            "tt_delay",
            static_cast<float>(std::clamp(
                delaySeconds, -static_cast<double>((std::numeric_limits<float>::max)()),
                static_cast<double>((std::numeric_limits<float>::max)()))));
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
    const auto updatePlayerAndMapScripts = [&] {
        for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
            if (vehicle.get() == playerVehicle_ && !vehicle->mapTrafficRouteFinished &&
                (!vehicle->followsMapTrafficRoute || vehicle->mapTrafficTileActive)) {
                vehicle->updateScripts(false);
            }
        }
        if (mapRenderer_ != nullptr) {
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
        for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
            if (vehicle.get() != playerVehicle_ && !vehicle->mapTrafficRouteFinished &&
                (!vehicle->followsMapTrafficRoute || vehicle->mapTrafficTileActive)) {
                vehicle->updateScripts(true);
            }
        }
    }
    // Script ticks must not leak their timestep into later mouse handlers.
    simulationState_.sharedVariables().set(
        "timegap", scriptRateHz_ > 0.0
                       ? frameTimeStep_
                       : std::clamp(frameTimeStep_, 0.0, MAX_SCRIPT_FRAME_DELTA_SECONDS));
}
