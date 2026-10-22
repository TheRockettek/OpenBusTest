// Vehicle lifecycle, scripting and simulation state synchronisation.

#include "app/vehicle/Vehicle.h"

void Vehicle::updateFrameVariables(bool isAiVehicle, double timeStep) {
    TraceScope trace("frame", "Vehicle::updateFrameVariables");
    // Frame-scoped values are refreshed before simulation and rendering run.
    variables.updateFrame();
    variables.set("ai", isAiVehicle ? 1.0 : 0.0);
    animationTimeStep = timeStep;
}

void Vehicle::updateScripts(bool isAiVehicle) {
    TraceScope trace("script", "Vehicle::updateScripts");
    if (scripts) {
        const std::size_t scriptErrorCount = scripts->errors().size();
        scripts->update(isAiVehicle);
        for (std::size_t index = scriptErrorCount; index < scripts->errors().size(); ++index) {
            gameLog.Log("Lua frame error: " + scripts->errors()[index]);
        }
    }
}

void Vehicle::finishDeferredTrafficInitialization() {
    if (!deferredDefinition) {
        throw std::logic_error("Deferred traffic vehicle initialization was not requested");
    }
    const std::shared_ptr<const SharedVehicleDefinition> definition =
        std::move(deferredDefinition);
    for (const std::string& error : scripts->errors()) {
        gameLog.Log("Lua script error: " + error);
    }
    load(*definition);
    scripts->initialize();
    for (const std::string& error : scripts->errors()) {
        gameLog.Log("Lua initialization error: " + error);
    }
    scripts->update(false);
    if (!hasSharedGeometry && loadingPolicy.modelMode == AssetLoadingMode::Eager) {
        spawn();
        cacheSharedDisplayLists();
    } else if (hasSharedGeometry) {
        loggedAllObjectsLoaded = true;
    }
    soundEventsEnabled = true;
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
                    pglDeleteBuffers(1, &batch.buffer);
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
    for (Part& part : pendingParts) {
        requests.emplace_back(&part, parsedObjFuture(part.objPath, part.bundleEntry));
    }
    for (const auto& request : requests) {
        loadObj(*request.first, request.second.get());
    }
    pendingParts.clear();
    rebuildDisplayOrder();
    preloadTextures();
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
    const openbus::rendering::BusModelLoadResult& result = definition.modelConfiguration;
    if (scripts) {
        scripts->configureScriptTextures(result.scriptTextures);
        scripts->configureTextTextures(result.textTextures);
    }
    interiorLights = result.interiorLights;
    interiorLightControllers.clear();
    interiorLightControllers.reserve(interiorLights.size());
    for (const ModelInteriorLight& light : interiorLights) {
        char* parsedEnd = nullptr;
        const double numericValue = std::strtod(light.controller.c_str(), &parsedEnd);
        interiorLightControllers.push_back(
            {parsedEnd != light.controller.c_str() && *parsedEnd == '\0', numericValue});
    }
    lodThresholds = result.lodThresholds;
    ctcTextureReplacements.clear();
    for (const ModelCtcTexture& texture : result.ctcTextures) {
        ctcTextureReplacements[lower(texture.slot)] = texture.textureName;
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
