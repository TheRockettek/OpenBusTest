// Vehicle lifecycle, scripting and simulation state synchronisation.

#include "app/vehicle/Vehicle.h"
#include "app/vehicle/PassengerRequestVariables.h"

#include <numbers>

void Vehicle::updateFrameVariables(bool isAiVehicle, double timeStep) {
    TraceScope trace("frame", "Vehicle::updateFrameVariables");
    // Frame-scoped values are refreshed before simulation and rendering run.
    variables.updateFrame();
    variables.set("ai", isAiVehicle ? 1.0 : 0.0);
    animationTimeStep = timeStep;
}

void Vehicle::updateScripts(bool isAiVehicle) {
    TraceScope trace("script", "Vehicle::updateScripts");
    openbus::app::vehicle::PassengerRequestVariables requests;
    if (!passengers) {
        variables.set("passengers_onboard", 0.0F);
        variables.set("humans_count", 0.0F);
    } else {
        TraceScope passengerVariables("script", "Vehicle::updateScripts.passengerVariables");
        const auto& cabin = passengers->configuration();
        for (std::size_t seatIndex = 0; seatIndex < passengers->service.seatCount(); ++seatIndex) {
            variables.set("human_count_on_seat_" + std::to_string(seatIndex),
                          static_cast<float>(passengers->service.passengerCountOnSeat(seatIndex)));
        }
        for (std::size_t linkIndex = 0; linkIndex < cabin.passengerPathLinks.size(); ++linkIndex) {
            variables.set(
                "human_count_on_path_link_" + std::to_string(linkIndex),
                static_cast<float>(passengers->service.passengerCountOnPathLink(linkIndex)));
        }
        const float onboardPassengerCount =
            static_cast<float>(passengers->service.onboardPassengerCount());
        variables.set("passengers_onboard", onboardPassengerCount);
        variables.set("humans_count", onboardPassengerCount);
        const int stopId = passengers->service.currentStopId();
        requests = openbus::app::vehicle::passengerRequestVariables(
            cabin.passengerEntryPathPoints.size(), cabin.passengerExitPathPoints.size(),
            passengers->service.dwelling() && stopId != 0,
            passengers->service.acceptsBoarding() ? passengers->service.waitingPassengers(stopId)
                                                  : 0,
            passengers->service.boardingPassenger(), passengers->service.alightingPassenger());
    }
    openbus::app::vehicle::publishPassengerRequestVariables(variables, requests);
    {
        TraceScope runtime("script", "Vehicle::updateScripts.runtime");
        if (scripts) {
            const std::size_t scriptErrorCount = scripts->errors().size();
            scripts->update(isAiVehicle);
            for (std::size_t index = scriptErrorCount; index < scripts->errors().size(); ++index) {
                gameLog.Error("Lua frame error: " + scripts->errors()[index]);
            }
        }
    }
}

Vehicle::DeferredTrafficInitializationStep Vehicle::finishDeferredTrafficInitialization() {
    if (!deferredInitializationStarted) {
        if (!deferredDefinition) {
            throw std::logic_error("Deferred traffic vehicle initialization was not requested");
        }
        const std::shared_ptr<const SharedVehicleDefinition> definition =
            std::move(deferredDefinition);
        {
            TraceScope definitionTrace("traffic", "deferredTraffic.loadDefinition");
            for (const std::string& error : scripts->errors()) {
                gameLog.Error("Lua script error: " + error);
            }
            load(*definition);
        }
        {
            TraceScope scriptsTrace("traffic", "deferredTraffic.initializeScripts");
            scripts->initialize();
            for (const std::string& error : scripts->errors()) {
                gameLog.Error("Lua initialization error: " + error);
            }
            scripts->update(false);
        }

        if (hasSharedGeometry || loadingPolicy.modelMode != AssetLoadingMode::Eager) {
            if (hasSharedGeometry) {
                loggedAllObjectsLoaded = true;
            }
            soundEventsEnabled = true;
            deferredInitializationStarted = true;
            return DeferredTrafficInitializationStep::Complete;
        }
        if (deferredObjRequests.size() != pendingParts.size()) {
            throw std::logic_error("Deferred traffic OBJ requests do not match model parts");
        }
        for (std::size_t index = 0; index < pendingParts.size(); ++index) {
            pendingParts[index].objRequest = deferredObjRequests[index];
        }
        deferredInitializationStarted = true;
        return DeferredTrafficInitializationStep::Progressed;
    }

    if (nextDeferredPartIndex < pendingParts.size()) {
        Part& part = pendingParts[nextDeferredPartIndex];
        {
            TraceScope pollTrace("traffic", "deferredTraffic.pollObj");
            if (!part.objRequest || part.objRequest->future.wait_for(std::chrono::milliseconds(
                                        0)) != std::future_status::ready) {
                return DeferredTrafficInitializationStep::Waiting;
            }
        }
        {
            TraceScope loadTrace("traffic", "deferredTraffic.loadObj");
            loadObj(part, part.objRequest->future.get());
        }
        ++nextDeferredPartIndex;
        if (nextDeferredPartIndex < pendingParts.size()) {
            return DeferredTrafficInitializationStep::Progressed;
        }
    }

    {
        TraceScope finalizeTrace("traffic", "deferredTraffic.finishSpawn");
        finishSpawn();
    }
    {
        TraceScope cacheTrace("traffic", "deferredTraffic.cacheSharedDisplayLists");
        cacheSharedDisplayLists();
    }
    soundEventsEnabled = true;
    return DeferredTrafficInitializationStep::Complete;
}

void Vehicle::initializePlayerSystems() {
    if (!scripts) {
        return;
    }
}

void Vehicle::updateSimulationVariables(const BusSimulation& simulation, double throttle,
                                        double steering, double brake) {
    simulation.updateVariables(variables, throttle, steering, brake);
    const double currentX = simulation.positionX();
    const double currentY = simulation.positionY();
    if (!odometerPositionInitialized) {
        odometerPosition = {currentX, currentY};
        odometerPositionInitialized = true;
    } else {
        const double distance =
            std::hypot(currentX - odometerPosition[0], currentY - odometerPosition[1]);
        if (std::isfinite(distance) && distance <= 100.0) {
            odometerMetres += distance;
        }
        odometerPosition = {currentX, currentY};
    }
    odometerMetres = std::max(0.0, odometerMetres);
    variables.set("kmcounter_km", std::floor(odometerMetres / 1000.0));
    variables.set("kmcounter_m", std::fmod(odometerMetres, 1000.0));
}

void Vehicle::updateTrafficWheelAnimations(double traveledDistanceMeters,
                                           double routeCurvaturePerMeter,
                                           double speedMetersPerSecond) {
    constexpr double radiansPerSecondToRpm = 60.0 / (2.0 * std::numbers::pi);
    const double physicalCurvaturePerMeter = -routeCurvaturePerMeter;
    for (std::size_t axleIndex = 0; axleIndex < trafficAxles.size(); ++axleIndex) {
        const BusAxle& axle = trafficAxles[axleIndex];
        const double wheelRadius = axle.wheelDiameter * 0.5;
        if (!std::isfinite(wheelRadius) || wheelRadius <= 0.0) {
            continue;
        }
        const double wheelRotation = traveledDistanceMeters / wheelRadius;
        const double wheelSpeedRpm = speedMetersPerSecond / wheelRadius * radiansPerSecondToRpm;
        for (const char* side : {"l", "r"}) {
            const std::string variableSuffix = "_" + std::to_string(axleIndex) + "_" + side;
            variables.set("wheel_rotation" + variableSuffix, wheelRotation);
            variables.set("wheel_rotationspeed" + variableSuffix, wheelSpeedRpm);
            if (!trafficAxleSteerable[axleIndex]) {
                continue;
            }
            const double sideOffset =
                (side[0] == 'l' ? 1.0 : -1.0) *
                std::max(0.0, axle.trackWidth * 0.5 - trafficWheelHalfWidthMeters);
            const double steeringAngle =
                std::atan2(trafficWheelbaseMeters * physicalCurvaturePerMeter,
                           1.0 - physicalCurvaturePerMeter * sideOffset);
            variables.set("axle_steering" + variableSuffix, std::clamp(steeringAngle, -0.65, 0.65));
        }
    }
}

void Vehicle::setOdeSimulation(const BusSimulation& simulation) {
    odeSimulation = &simulation;
}

void Vehicle::joinTextureWorkers() {
    assets->join();
}

Vehicle::~Vehicle() {
    if (soundPlayback != nullptr) {
        soundBank.stopAll(*soundPlayback);
    }
    joinTextureWorkers();
    const auto deleteBuffers = [](const std::vector<DisplayPart>& parts) {
        for (const DisplayPart& part : parts) {
            for (const Batch& batch : part.batches) {
                if (batch.buffer && !batch.bufferTracked) {
                    openbus::rendering::deleteRendererBuffers(1, &batch.buffer);
                }
            }
        }
    };
    deleteBuffers(displayLists);
#ifdef _WIN32
    if (comInitialized) {
        CoUninitialize();
    }
#endif
}

bool Vehicle::isCaptureReady() const {
    return areObjectsLoaded() && areTexturesLoaded();
}

void Vehicle::spawn() {
    TraceScope trace("obj", "spawn");
    std::vector<std::pair<Part*, std::shared_future<std::shared_ptr<ParsedObj>>>> requests;
    requests.reserve(pendingParts.size());
    {
        TraceScope requestTrace("obj", "Vehicle::spawn.requestModels");
        for (Part& part : pendingParts) {
            requests.emplace_back(&part, parsedObjFuture(part.objPath, part.bundleEntry));
        }
    }
    {
        TraceScope buildTrace("obj", "Vehicle::spawn.waitAndBuildModels");
        for (const auto& request : requests) {
            loadObj(*request.first, request.second.get());
        }
    }
    {
        TraceScope finishTrace("obj", "Vehicle::spawn.finish");
        finishSpawn();
    }
}

void Vehicle::finishSpawn() {
    pendingParts.clear();
    {
        TraceScope orderTrace("obj", "Vehicle::finishSpawn.rebuildDisplayOrder");
        rebuildDisplayOrder();
    }
    {
        TraceScope textureTrace("obj", "Vehicle::finishSpawn.preloadTextures");
        preloadTextures();
    }
    loaded = !displayLists.empty();
    hasLoadedInitialView = true;
    if (!loggedAllObjectsLoaded && loaded) {
        gameLog.Log("All objects loaded. bodyParts=" + std::to_string(displayLists.size()) +
                    " wheelParts=generic");
        loggedAllObjectsLoaded = true;
    }
}

std::string Vehicle::trim(const std::string& value) {
    const std::size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const std::size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::string Vehicle::lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

double Vehicle::parseDouble(const std::string& value, double fallback) {
    try {
        return std::stod(trim(value));
    } catch (const std::exception&) {
        return fallback;
    }
}

void Vehicle::load(const SharedVehicleDefinition& definition) {
    TraceScope trace("startup", "Vehicle::load");
    const openbus::rendering::BusModelLoadResult& result = definition.modelConfiguration;
    if (scripts) {
        scripts->configureScriptTextures(result.scriptTextures);
        scripts->configureTextTextures(result.textTextures);
    }
    interiorLights = result.interiorLights;
    enhancedLights = result.enhancedLights;
    spotlights = result.spotlights;
    interiorLightControllers.clear();
    interiorLightControllers.reserve(interiorLights.size());
    for (const ModelInteriorLight& light : interiorLights) {
        char* parsedEnd = nullptr;
        const double numericValue = std::strtod(light.controller.c_str(), &parsedEnd);
        interiorLightControllers.push_back(
            {parsedEnd != light.controller.c_str() && *parsedEnd == '\0', numericValue});
    }
    enhancedLightControllers.clear();
    enhancedLightControllers.reserve(enhancedLights.size());
    for (const ModelEnhancedLight& light : enhancedLights) {
        char* parsedEnd = nullptr;
        const double numericValue = std::strtod(light.controller.c_str(), &parsedEnd);
        enhancedLightControllers.push_back(
            {parsedEnd != light.controller.c_str() && *parsedEnd == '\0', numericValue});
    }
    lodThresholds = result.lodThresholds;
    ctcTextureReplacements.clear();
    for (const ModelCtcTexture& texture : result.ctcTextures) {
        ctcTextureReplacements[lower(texture.slot)] = texture.textureName;
    }
    for (const auto& [slot, textureName] : setupOptions.ctcTextureReplacements) {
        ctcTextureReplacements[lower(slot)] = textureName;
    }
    if (loadSharedDisplayLists()) {
        return;
    }

    std::vector<Part> parts;
    parts.reserve(result.parts.size());
    for (const auto& source : result.parts) {
        Part part;
        static_cast<openbus::rendering::BusModelPart&>(part) = source;
        parts.push_back(std::move(part));
    }
    pendingParts = std::move(parts);
    loaded = !pendingParts.empty();
}
