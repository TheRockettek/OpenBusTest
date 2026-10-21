// Script ticking and player/physics variable synchronisation.

#include "app/RenderLoopInternal.h"
#include "render/map/MapRenderer.h"

namespace {

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
        mapRenderer_->updateTrafficActors(std::clamp(frameTimeStep_, 0.0, 0.25));
    }
    const double trafficTimeStep = std::clamp(frameTimeStep_, 0.0, 0.25);
    for (TrafficVehicleState& trafficVehicle : trafficVehicles_) {
        if (trafficVehicle.vehicle->mapTrafficRouteFinished) {
            continue;
        }
        if (!openbus::map::advanceMapTrafficActor(trafficVehicle.actor, trafficTimeStep)) {
            trafficVehicle.vehicle->mapTrafficRouteFinished = true;
            trafficVehicle.routeCompleted = true;
            const int finalSplineId =
                trafficVehicle.actor.route.segments.back().spline.placement.splineId;
            gameLog.Log(std::string("Map ") +
                        (trafficVehicle.isBus ? "AI bus" : "road AI vehicle") +
                        " unloaded at unconnected lane end: " +
                        trafficVehicle.actor.modelPath +
                        ", final spline=" + std::to_string(finalSplineId));
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
    }
    for (HumanTrafficState& human : humanTrafficActors_) {
        if (human.populationEnabled) {
            openbus::map::advanceMapTrafficActorLooping(human.actor, trafficTimeStep);
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
            if (!vehicle->mapTrafficRouteFinished) {
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
            gameLog.Log("Script scheduler executed " +
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
    // Fixed script ticks must not leak their timestep into later mouse handlers.
    simulationState_.sharedVariables().set("timegap", frameTimeStep_);
}
