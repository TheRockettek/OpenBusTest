// Window lifecycle, vehicle/map registration and simple accessors.

#include "app/RenderLoopInternal.h"
#include "app/vehicle/VehicleDefinition.h"
#include "render/map/MapRenderer.h"
#include "map/MapSplineProfile.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <numbers>

namespace {

double calculateTrafficMinimumTurnRadius(const VehicleConfig& configuration, bool isBus) {
    double turnRadius = configuration.inverseMinimumTurnRadius > 0.0
                            ? 1.0 / configuration.inverseMinimumTurnRadius
                            : (isBus ? 10.0 : 4.5);
    if (configuration.inverseMinimumTurnRadius <= 0.0 && configuration.axles.size() >= 2) {
        const auto [rear, front] = std::minmax_element(
            configuration.axles.begin(), configuration.axles.end(),
            [](const auto& a, const auto& b) { return a.position < b.position; });
        turnRadius = std::max(turnRadius, (front->position - rear->position) /
                                              std::tan(35.0 * std::numbers::pi / 180.0));
    }
    return turnRadius;
}

} // namespace

RenderLoop::RenderLoop(int width, int height, const char* title)
    : window_(nullptr), assetRequestManager_(std::make_unique<AssetRequestManager>()),
      reflectionRenderer_(std::make_unique<openbus::rendering::ReflectionRenderer>()) {
    TraceScope trace("startup", "RenderLoop::RenderLoop");
    if (const char* diagnostics = openbus::getEnvironment("OPENBUS_SCRIPT_DIAGNOSTICS")) {
        drivetrainDiagnosticsEnabled_ = std::string(diagnostics) == "1";
    }
    if (drivetrainDiagnosticsEnabled_) {
        gameLog.Log("Native drivetrain variable diagnostics enabled (1 Hz)");
    }
    mouseLookEnabled_ = parseEnabledFlag(openbus::getEnvironment("OPENBUS_MOUSE_LOOK"));
    if (mouseLookEnabled_) {
        gameLog.Log("Mouse camera look enabled (F4 toggles; move the mouse to look around)");
    }
    if (const char* scriptRate = openbus::getEnvironment("OPENBUS_SCRIPT_HZ")) {
        try {
            std::size_t parsed = 0;
            const double requestedRate = std::stod(scriptRate, &parsed);
            if (!std::isfinite(requestedRate) || scriptRate[parsed] != '\0') {
                throw std::invalid_argument("Invalid OPENBUS_SCRIPT_HZ");
            }
            const double rate = std::clamp(requestedRate, 0.0, 1000.0);
            if (rate > 0.0 && !std::isfinite(1.0 / rate)) {
                throw std::invalid_argument("OPENBUS_SCRIPT_HZ is too small");
            }
            scriptRateHz_ = rate;
        } catch (const std::exception&) {
            scriptRateHz_ = 0.0;
            gameLog.Log("Invalid OPENBUS_SCRIPT_HZ; scripts will follow the render rate");
        }
    }
    if (const char* hoverRate = openbus::getEnvironment("OPENBUS_HOVER_HZ")) {
        try {
            std::size_t parsed = 0;
            const double rate = std::stod(hoverRate, &parsed);
            if (!std::isfinite(rate) || hoverRate[parsed] != '\0') {
                throw std::invalid_argument("Invalid hover rate");
            }
            const double rateHz = std::clamp(rate, 0.0, 1000.0);
            clickableHoverRefreshInterval_ = rateHz > 0.0 ? 1.0 / rateHz : 0.0;
        } catch (const std::invalid_argument&) {
            gameLog.Log("Invalid OPENBUS_HOVER_HZ; using the default 75 ms hover interval");
        } catch (const std::out_of_range&) {
            gameLog.Log("OPENBUS_HOVER_HZ out of range; using the default 75 ms hover interval");
        }
    }
    if (clickableHoverRefreshInterval_ > 0.0) {
        gameLog.Log("Stationary-pointer hover refresh limited to " +
                    std::to_string(1.0 / clickableHoverRefreshInterval_) + " Hz");
    } else {
        gameLog.Log("Hover refresh follows the render rate");
    }
    if (const char* steeringSmoothing = openbus::getEnvironment("OPENBUS_STEERING_SMOOTHING")) {
        try {
            steeringSmoothingRate_ = std::clamp(std::stod(steeringSmoothing), 0.0, 1000.0);
        } catch (const std::exception&) {
            steeringSmoothingRate_ = 0.0;
        }
    }
    gameLog.Log("Mouse steering smoothing: " +
                std::to_string(openbus::input::DEFAULT_MOUSE_STEERING_SMOOTHING_RATE) +
                " input units/s; keyboard steering remains unsmoothed");
    if (scriptRateHz_ > 0.0) {
        gameLog.Log("Script rate limited to " + std::to_string(scriptRateHz_) + " Hz");
    } else {
        gameLog.Log("Scripts follow the render rate");
    }
    if (!glfwInit()) {
        gameLog.Log("Failed to initialize GLFW");
        throw std::runtime_error("Failed to initialize GLFW");
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    const bool benchmarkMode = parseEnabledFlag(openbus::getEnvironment("OPENBUS_BENCHMARK"));
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif
    window_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!window_) {
        gameLog.Log("OpenGL 4.3 context unavailable; retrying 3.3 with CPU culling");
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        window_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
    }
    if (!window_) {
        gameLog.Log("Failed to create OpenGL window");
        glfwTerminate();
        throw std::runtime_error("Failed to create OpenGL window");
    }
    if (benchmarkMode) {
        gameLog.Log(
            std::string("Benchmark window is ") +
            (glfwGetWindowAttrib(window_, GLFW_VISIBLE) == GLFW_TRUE ? "visible" : "hidden"));
    }
    glfwMakeContextCurrent(window_);
    glfwGetFramebufferSize(window_, &framebufferWidth_, &framebufferHeight_);
    clickableCursor_ = glfwCreateStandardCursor(GLFW_HAND_CURSOR);
    mouseSteeringCursor_ = glfwCreateStandardCursor(GLFW_CROSSHAIR_CURSOR);
    glfwSetWindowUserPointer(window_, this);
    if (mouseLookEnabled_) {
        glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        glfwGetCursorPos(window_, &previousCursorX_, &previousCursorY_);
    }
    glfwSetScrollCallback(window_, &RenderLoop::scrollCallback);
    const char* vsyncSetting = openbus::getEnvironment("OPENBUS_VSYNC");
    const bool vsyncEnabled = vsyncSetting == nullptr || (std::string(vsyncSetting) != "0" &&
                                                          std::string(vsyncSetting) != "off" &&
                                                          std::string(vsyncSetting) != "false");
    glfwSwapInterval(vsyncEnabled ? 1 : 0);
    gameLog.Log(std::string("VSync ") + (vsyncEnabled ? "enabled" : "disabled"));
    if (!loadOpenGLFunctions()) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
        glfwTerminate();
        throw std::runtime_error("OpenGL VBO functions are unavailable");
    }
    if (!openbus::rendering::initializeCoreRenderer()) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
        glfwTerminate();
        throw std::runtime_error("Failed to initialize core OpenGL renderer");
    }
    pglActiveTexture(GL_TEXTURE0);
    glGenTextures(1, &statusHudTexture_);
    if (statusHudTexture_ != 0) {
        glBindTexture(GL_TEXTURE_2D, statusHudTexture_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        const std::vector<std::uint8_t> initialPixels =
            makeStatusHudPixels("X 0.0 Y 0.0 Z 0.0\nCHUNKS 0/0 TERRAIN 0/0 OBJECTS 0/0\n"
                                "ROADS 0/0 COLLISION TILES 0 AI VEHICLES 0");
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, STATUS_HUD_TEXTURE_WIDTH, STATUS_HUD_TEXTURE_HEIGHT,
                     0, GL_RGBA, GL_UNSIGNED_BYTE, initialPixels.data());
        openbus::rendering::invalidateTextureBindings();
        if (glGetError() != GL_NO_ERROR) {
            glDeleteTextures(1, &statusHudTexture_);
            statusHudTexture_ = 0;
            gameLog.Log("Status HUD texture initialization failed");
        }
    } else {
        gameLog.Log("Status HUD texture could not be allocated");
    }
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.45f, 0.65f, 0.88f, 1.0f);
    gameLog.Log("RenderLoop initialized");
}

RenderLoop::~RenderLoop() {
    gameLog.Log("RenderLoop shutting down");
    for (TrafficVehicleState& state : trafficVehicles_) {
        if (!state.pendingVehicle.valid()) {
            continue;
        }
        try {
            std::unique_ptr<Vehicle> pendingVehicle = state.pendingVehicle.get();
        } catch (const std::exception& error) {
            gameLog.Log("Map traffic vehicle loader failed during shutdown: " +
                        std::string(error.what()));
        } catch (...) {
            gameLog.Log("Map traffic vehicle loader failed during shutdown with an unknown error");
        }
    }
    if (window_ != nullptr && mouseLookEnabled_) {
        glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }
    reflectionRenderer_->destroy();
    mapRenderer_.reset();
    playerVehicle_ = nullptr;
    assetRequestManager_->join();
    vehicles_.clear();
    Vehicle::clearSharedDisplayCache();
    if (statusHudTexture_ != 0) {
        glDeleteTextures(1, &statusHudTexture_);
        statusHudTexture_ = 0;
        openbus::rendering::invalidateTextureBindings();
    }
    openbus::rendering::shutdownCoreRenderer();
    assetRequestManager_.reset();

    if (clickableCursor_ != nullptr) {
        glfwDestroyCursor(clickableCursor_);
        clickableCursor_ = nullptr;
    }
    if (mouseSteeringCursor_ != nullptr) {
        glfwDestroyCursor(mouseSteeringCursor_);
        mouseSteeringCursor_ = nullptr;
    }

    if (window_) {
        glfwDestroyWindow(window_);
    }

    glfwTerminate();
}

Vehicle* RenderLoop::AddVehicle(const std::filesystem::path& vehicleConfigPath,
                                const std::filesystem::path& modelConfigPath,
                                const VehiclePlacement& placement, ModelLoadingPolicy loadingPolicy,
                                bool isAiVehicle) {
    const std::filesystem::path resolvedVehicleConfigPath =
        busConfigurationPathFor(vehicleConfigPath);
    std::string extension = resolvedVehicleConfigPath.extension().string();
    std::transform(
        extension.begin(), extension.end(), extension.begin(),
        [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    const VehicleFileKind configKind =
        extension == ".ovh" ? VehicleFileKind::Vehicle : VehicleFileKind::Bus;
    const std::filesystem::path resolvedModelConfigPath =
        modelConfigPath.empty()
            ? (configKind == VehicleFileKind::Bus
                   ? modelConfigurationPathForBus(resolvedVehicleConfigPath)
                   : modelConfigurationPathForVehicle(resolvedVehicleConfigPath))
            : (configKind == VehicleFileKind::Bus
                   ? modelConfigurationPathForBus(resolvedVehicleConfigPath, modelConfigPath)
                   : modelConfigurationPathForVehicle(resolvedVehicleConfigPath, modelConfigPath));
    const std::shared_ptr<const SharedVehicleDefinition> definition =
        sharedVehicleDefinition(resolvedVehicleConfigPath, resolvedModelConfigPath, configKind);
    const VehicleConfig& vehicleConfiguration = definition->vehicleConfiguration;
    if (vehicleCameras_.empty()) {
        for (const VehicleCamera& camera : vehicleConfiguration.cameras) {
            if (camera.kind == VehicleCameraKind::Driver ||
                camera.kind == VehicleCameraKind::Passenger ||
                camera.kind == VehicleCameraKind::Reflexion ||
                camera.kind == VehicleCameraKind::Reflexion2) {
                vehicleCameras_.push_back(camera);
            }
        }
        if (!vehicleCameras_.empty()) {
            int driverIndex = 0;
            int selectedCamera = -1;
            for (std::size_t index = 0; index < vehicleCameras_.size(); ++index) {
                if (vehicleCameras_[index].kind != VehicleCameraKind::Driver) {
                    continue;
                }
                if (driverIndex == vehicleConfiguration.standardDriverCamera) {
                    selectedCamera = static_cast<int>(index);
                    break;
                }
                ++driverIndex;
            }
            cameraView_ = selectedCamera >= 0 ? selectedCamera + 1 : 1;
            reflectionReferenceCamera_ = selectedCamera;
            for (std::size_t index = 0;
                 reflectionReferenceCamera_ < 0 && index < vehicleCameras_.size(); ++index) {
                if (vehicleCameras_[index].kind == VehicleCameraKind::Driver) {
                    reflectionReferenceCamera_ = static_cast<int>(index);
                }
            }
        }
        reflectionRenderer_->initialize(vehicleCameras_);
    }
    const double modelOffsetZ = -vehicleConfiguration.centerOfGravityHeight;
    auto model = std::make_unique<Vehicle>(
        definition, placementForVehicle(vehicleConfiguration, placement, isAiVehicle), modelOffsetZ,
        loadingPolicy, *assetRequestManager_, simulationState_, soundEngine_, soundViewpoint_,
        isAiVehicle);
    Vehicle* result = model.get();
    vehicles_.push_back(std::move(model));
    return result;
}

bool RenderLoop::AddTrafficVehicle(
    int splineId, int pathTypeCode, const std::filesystem::path& busConfigPath,
    std::optional<std::string> driverModelPath, double speedMetersPerSecond,
    ModelLoadingPolicy loadingPolicy, bool isBus, std::size_t laneSelector, std::size_t routeSeed,
    std::vector<double> scheduledDepartureMinutes, std::optional<std::size_t> unscheduledGroupIndex,
    int defaultVehicleType, std::vector<int> requiredRouteSplineIds) {
    if (mapRenderer_ == nullptr) {
        throw std::logic_error("Cannot add map traffic before loading a map");
    }
    const auto laneStarts =
        isBus ? mapRenderer_->trafficEntryLanes() : mapRenderer_->trafficSpawnLanes();
    if (pathTypeCode != openbus::map::MAP_SPLINE_PATH_TYPE_ROAD ||
        std::find(laneStarts.begin(), laneStarts.end(),
                  openbus::map::MapTrafficLaneStart{splineId, laneSelector}) == laneStarts.end()) {
        return false;
    }
    const auto resolvedConfig = busConfigurationPathFor(busConfigPath);
    const auto definition =
        sharedVehicleDefinition(resolvedConfig, modelConfigurationPathForVehicle(resolvedConfig),
                                isBus ? VehicleFileKind::Bus : VehicleFileKind::Vehicle);
    const auto& configuration = definition->vehicleConfiguration;
    if (!configuration.hasBoundingBox || configuration.boundingBox[0] <= 0.0 ||
        configuration.boundingBox[1] <= 0.0) {
        throw std::runtime_error("Map traffic requires a valid vehicle bounding box: " +
                                 busConfigPath.string());
    }
    const double turnRadius = calculateTrafficMinimumTurnRadius(configuration, isBus);
    const int vehicleType = isBus ? 2 : configuration.vehicleType.value_or(defaultVehicleType);
    const openbus::map::MapTrafficVehicleFilter vehicleFilter{
        vehicleType, isBus ? std::nullopt : unscheduledGroupIndex, isBus};
    auto route = mapRenderer_->buildDrivingRoute(splineId, laneSelector, routeSeed, turnRadius,
                                                 vehicleFilter, requiredRouteSplineIds);
    if (!route) {
        return false;
    }
    *route = openbus::map::subdivideMapTrafficRoute(*route);
    auto actor = openbus::map::makeMapTrafficActor(std::move(*route), busConfigPath.string(),
                                                   speedMetersPerSecond, 0.0, false,
                                                   std::move(driverModelPath));
    if (isBus) {
        if (scheduledDepartureMinutes.empty()) {
            throw std::invalid_argument("Scheduled map buses require timetable departures");
        }
        if (std::any_of(scheduledDepartureMinutes.begin(), scheduledDepartureMinutes.end(),
                        [](double minute) { return !std::isfinite(minute) || minute < 0.0; })) {
            throw std::invalid_argument("Map bus timetable departures are invalid");
        }
        std::sort(scheduledDepartureMinutes.begin(), scheduledDepartureMinutes.end());
    } else if (!scheduledDepartureMinutes.empty()) {
        throw std::invalid_argument("Only map buses can have timetable departures");
    }
    actor.speedMetersPerSecond = 0.0;
    actor.minimumTurnRadiusMeters = turnRadius;
    actor.vehicleType = vehicleType;
    actor.unscheduledGroupIndex = isBus ? std::nullopt : unscheduledGroupIndex;
    actor.scheduled = isBus;
    actor.lengthMeters =
        configuration.boundingBox[1] + 2.0 * std::abs(configuration.boundingBox[4]);
    actor.widthMeters = configuration.boundingBox[0] + 2.0 * std::abs(configuration.boundingBox[3]);
    std::vector<openbus::map::MapTrafficSpawnCandidate> spawnCandidates;
    if (!isBus) {
        spawnCandidates =
            openbus::map::mapTrafficRouteSpawnCandidates(actor.route, actor.lengthMeters);
        if (spawnCandidates.empty()) {
            return false;
        }
    }

    const std::optional<openbus::map::MapTrafficPose> pose =
        openbus::map::sampleMapTrafficRoute(actor.route, 0.0, actor.lateralOffsetMeters);
    if (!pose.has_value()) {
        throw std::runtime_error("Could not sample the initial map traffic route pose");
    }
    VehiclePlacement placement;
    placement.position = pose->position;
    placement.yawDegrees = openbus::map::mapTrafficHeadingToVehicleYawDegrees(pose->headingRadians);
    const VehiclePlacement configuredPlacement =
        placementForVehicle(configuration, placement, true);
    const double modelOffsetZ = -configuration.centerOfGravityHeight;
    AssetRequestManager* const assetRequests = assetRequestManager_.get();
    SimulationState* const simulationState = &simulationState_;
    SoundEngine* const soundEngine = &soundEngine_;
    openbus::rendering::ViewpointContext* const soundViewpoint = &soundViewpoint_;
    std::future<std::unique_ptr<Vehicle>> pendingVehicle =
        assetRequests->submit([definition, configuredPlacement, modelOffsetZ, loadingPolicy,
                               assetRequests, simulationState, soundEngine, soundViewpoint] {
            return std::make_unique<Vehicle>(definition, configuredPlacement, modelOffsetZ,
                                             loadingPolicy, *assetRequests, *simulationState,
                                             *soundEngine, *soundViewpoint, true, true);
        });
    TrafficVehicleState trafficState;
    trafficState.pendingVehicle = std::move(pendingVehicle);
    trafficState.actor = std::move(actor);
    trafficState.spawnCandidates = std::move(spawnCandidates);
    trafficState.scheduledDepartureMinutes = std::move(scheduledDepartureMinutes);
    trafficState.isBus = isBus;
    if (isBus) {
        trafficState.nextScheduledDepartureElapsedSeconds =
            openbus::map::mapTrafficNextDepartureElapsedSeconds(
                mapTrafficStartHour_, 0.0, trafficState.scheduledDepartureMinutes.front());
    }
    trafficVehicles_.push_back(std::move(trafficState));
    return true;
}

int RenderLoop::trafficVehicleType(const std::filesystem::path& vehicleConfigPath,
                                   int defaultVehicleType) const {
    if (mapRenderer_ == nullptr) {
        throw std::logic_error("Cannot query map traffic vehicle types before loading a map");
    }
    const auto resolvedConfig = busConfigurationPathFor(vehicleConfigPath);
    const auto definition = sharedVehicleDefinition(
        resolvedConfig, modelConfigurationPathForVehicle(resolvedConfig), VehicleFileKind::Vehicle);
    return definition->vehicleConfiguration.vehicleType.value_or(defaultVehicleType);
}

void RenderLoop::SetMapTrafficStartHour(double startHour) {
    if (!std::isfinite(startHour) || startHour < 0.0 || startHour >= 24.0) {
        throw std::invalid_argument("Map traffic start hour must be between 0 and 24");
    }
    mapTrafficStartHour_ = startHour;
}

double RenderLoop::trafficMinimumTurnRadius(const std::filesystem::path& vehicleConfigPath,
                                            bool isBus) const {
    const std::filesystem::path resolvedConfig = busConfigurationPathFor(vehicleConfigPath);
    const VehicleFileKind configKind = isBus ? VehicleFileKind::Bus : VehicleFileKind::Vehicle;
    const std::filesystem::path resolvedModelConfig =
        isBus ? modelConfigurationPathForBus(resolvedConfig)
              : modelConfigurationPathForVehicle(resolvedConfig);
    const auto definition =
        sharedVehicleDefinition(resolvedConfig, resolvedModelConfig, configKind);
    return calculateTrafficMinimumTurnRadius(definition->vehicleConfiguration, isBus);
}

bool RenderLoop::AddHumanTrafficActor(int splineId, const std::filesystem::path& humanConfigPath,
                                      double feetDistanceMeters, double heightMeters,
                                      double speedMetersPerSecond, double startDistanceMeters,
                                      double centerX, double centerY,
                                      double maximumDistanceMeters) {
    if (mapRenderer_ == nullptr) {
        throw std::logic_error("Cannot add human traffic before loading a map");
    }
    if (humanConfigPath.empty() || !std::isfinite(feetDistanceMeters) ||
        feetDistanceMeters <= 0.0 || !std::isfinite(heightMeters) || heightMeters <= 0.0) {
        throw std::invalid_argument("Human traffic spawn configuration is invalid");
    }

    mapRenderer_->loadHumanTrafficModel(humanConfigPath);
    std::optional<openbus::map::MapTrafficActor> actor = mapRenderer_->createTrafficActor(
        splineId, humanConfigPath.string(), speedMetersPerSecond, 0.0, std::nullopt,
        openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN, humanTrafficActors_.size());
    if (!actor) {
        return false;
    }
    if (!openbus::map::placeMapTrafficActorNearLooping(*actor, startDistanceMeters, centerX,
                                                       centerY, maximumDistanceMeters, 0.01)) {
        return false;
    }
    actor->lengthMeters = feetDistanceMeters * 1.25;
    actor->widthMeters = feetDistanceMeters * 2.1;
    humanTrafficActors_.push_back({std::move(*actor), feetDistanceMeters, heightMeters, {}, false});
    return true;
}

void RenderLoop::SetMapTrafficPopulation(
    std::size_t roadVehicleCount, std::size_t busCount,
    std::vector<openbus::map::MapAiRoadTrafficGroupPopulation> roadGroupPopulations,
    bool useGroupRoadPopulation) {
    targetRoadTrafficCount_ = roadVehicleCount;
    targetRoadGroupPopulations_ = std::move(roadGroupPopulations);
    useGroupRoadPopulation_ = useGroupRoadPopulation;
    targetBusTrafficCount_ = busCount;
}

void RenderLoop::SetHumanTrafficPopulation(std::size_t count) {
    targetHumanTrafficCount_ = count;
}

std::vector<int> RenderLoop::trafficSplineIds(int pathTypeCode) const {
    return mapRenderer_ == nullptr ? std::vector<int>{}
                                   : mapRenderer_->trafficSplineIds(pathTypeCode);
}

std::vector<openbus::map::MapTrafficLaneStart> RenderLoop::trafficEntryLanes() const {
    return mapRenderer_ == nullptr ? std::vector<openbus::map::MapTrafficLaneStart>{}
                                   : mapRenderer_->trafficEntryLanes();
}

std::optional<openbus::map::MapTrafficLaneStart>
RenderLoop::selectTrafficEntryLane(openbus::map::MapTrafficVehicleFilter vehicle,
                                   std::size_t selectionSeed) const {
    return mapRenderer_ == nullptr ? std::nullopt
                                   : mapRenderer_->selectTrafficEntryLane(vehicle, selectionSeed);
}

std::vector<openbus::map::MapTrafficLaneStart> RenderLoop::trafficSpawnLanes() const {
    return mapRenderer_ == nullptr ? std::vector<openbus::map::MapTrafficLaneStart>{}
                                   : mapRenderer_->trafficSpawnLanes();
}

std::optional<openbus::map::MapTrafficLaneStart>
RenderLoop::selectTrafficSpawnLane(openbus::map::MapTrafficVehicleFilter vehicle,
                                   std::size_t selectionSeed) const {
    return mapRenderer_ == nullptr ? std::nullopt
                                   : mapRenderer_->selectTrafficSpawnLane(vehicle, selectionSeed);
}

std::vector<int> RenderLoop::trafficRouteStartSplineIds(int pathTypeCode) const {
    return mapRenderer_ == nullptr ? std::vector<int>{}
                                   : mapRenderer_->trafficRouteStartSplineIds(pathTypeCode);
}

std::vector<int>
RenderLoop::trafficRouteSplineIds(int splineId, int pathTypeCode, std::size_t laneSelector,
                                  double minimumTurnRadiusMeters, std::size_t routeSeed,
                                  const std::vector<int>& requiredSplineSequence,
                                  openbus::map::MapTrafficVehicleFilter vehicle) const {
    if (mapRenderer_ == nullptr) {
        return {};
    }
    const std::optional<openbus::map::MapTrafficRoute> route =
        pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_ROAD
            ? mapRenderer_->buildDrivingRoute(splineId, laneSelector, routeSeed,
                                              minimumTurnRadiusMeters, vehicle,
                                              requiredSplineSequence)
            : mapRenderer_->buildTrafficRoute(splineId, pathTypeCode);
    if (!route.has_value()) {
        return {};
    }
    std::vector<int> splineIds;
    splineIds.reserve(route->segments.size());
    for (const openbus::map::MapTrafficRouteSegment& segment : route->segments) {
        if (segment.spline.placement.splineId != 0) {
            splineIds.push_back(segment.spline.placement.splineId);
        }
    }
    return splineIds;
}

std::size_t RenderLoop::trafficVehicleCount() const {
    return static_cast<std::size_t>(std::count_if(
        trafficVehicles_.begin(), trafficVehicles_.end(), [](const TrafficVehicleState& state) {
            return state.populationEnabled && state.vehicle != nullptr &&
                   !state.vehicle->mapTrafficRouteFinished;
        }));
}

void RenderLoop::SetWeather(int precipitationType, double precipitationRate) {
    if (precipitationType < 0 || precipitationType > 2 || !std::isfinite(precipitationRate) ||
        precipitationRate < 0.0 || precipitationRate > 1.0) {
        throw std::invalid_argument("Weather precipitation settings are invalid");
    }
    simulationState_.sharedVariables().set("preciptype", static_cast<float>(precipitationType));
    simulationState_.sharedVariables().set("preciprate", static_cast<float>(precipitationRate));
}

void RenderLoop::SetPlayerVehicle(Vehicle* model) {
    playerVehicle_ = model;
    if (playerVehicle_ != nullptr) {
        playerVehicle_->initializePlayerSystems();
    }
}

void RenderLoop::SetMap(const openbus::map::MapDefinition& map, std::size_t spawnEntryPointIndex,
                        std::size_t groundTextureIndex, const std::filesystem::path& omsiRoot) {
    TraceScope trace("map", "RenderLoop::SetMap");
    if (spawnEntryPointIndex >= map.entryPoints.size()) {
        throw std::out_of_range("Selected map spawn entrypoint is outside [entrypoints]");
    }
    mapRenderer_ = std::make_unique<openbus::rendering::MapRenderer>(
        map, static_cast<std::size_t>(map.entryPoints[spawnEntryPointIndex].tileIndex),
        map.entryPoints[spawnEntryPointIndex].placement.position[0],
        map.entryPoints[spawnEntryPointIndex].placement.position[1], groundTextureIndex, omsiRoot,
        simulationState_, *assetRequestManager_);
    trafficSplineDebugBuilt_ = false;
    trafficSplineDebugTiles_.clear();
    ++trafficSplineDebugRevision_;
}

void RenderLoop::updateMapStreaming(double worldX, double worldY) {
    if (mapRenderer_ != nullptr) {
        mapRenderer_->updateStreaming(worldX, worldY);
    }
}

std::vector<std::pair<int, int>> RenderLoop::activeMapTiles() const {
    if (mapRenderer_ == nullptr) {
        return {};
    }
    return mapRenderer_->activeTileCoordinates();
}

std::vector<std::pair<int, int>> RenderLoop::loadedMapTiles() const {
    return activeMapTiles();
}

bool RenderLoop::shouldClose() const {
    return glfwWindowShouldClose(window_) != 0;
}

void RenderLoop::requestClose() {
    if (window_ != nullptr) {
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }
}

std::array<int, 2> RenderLoop::windowSize() const {
    int width = 1;
    int height = 1;
    glfwGetWindowSize(window_, &width, &height);
    return {width, height};
}

std::array<int, 2> RenderLoop::framebufferSize() const {
    return {framebufferWidth_, framebufferHeight_};
}

RenderBenchmarkInput RenderLoop::setBenchmarkFrame(RenderBenchmarkPhase phase, int frameInPhase,
                                                   int phaseFrameCount) {
    setFreeCameraEnabled(false);
    const int frameCount = std::max(phaseFrameCount, 1);
    const int frame = std::clamp(frameInPhase, 0, frameCount - 1);
    const double progress = frameCount <= 1 ? 0.0 : static_cast<double>(frame) / (frameCount - 1);
    benchmarkClickDiscoveryPending_ = false;
    benchmarkClickQueued_ = false;

    RenderBenchmarkInput input;
    switch (phase) {
    case RenderBenchmarkPhase::Baseline:
        cameraView_ = 0;
        cameraYaw_ = -2.3;
        cameraPitch_ = 0.45;
        cameraDistance_ = 24.0;
        viewLookYaw_ = 0.0;
        viewLookPitch_ = 0.0;
        fieldOfViewOffset_ = 0.0;
        break;
    case RenderBenchmarkPhase::CameraCycle: {
        const auto isUserCamera = [](const VehicleCamera& camera) {
            return camera.kind == VehicleCameraKind::Driver ||
                   camera.kind == VehicleCameraKind::Passenger;
        };
        const std::size_t cameraCount = static_cast<std::size_t>(
            std::count_if(vehicleCameras_.begin(), vehicleCameras_.end(), isUserCamera));
        if (cameraCount == 0) {
            cameraView_ = 0;
            break;
        }
        const std::size_t selectedCamera =
            std::min(cameraCount - 1, static_cast<std::size_t>(frame) * cameraCount /
                                          static_cast<std::size_t>(frameCount));
        std::size_t userCameraIndex = 0;
        for (std::size_t index = 0; index < vehicleCameras_.size(); ++index) {
            if (!isUserCamera(vehicleCameras_[index])) {
                continue;
            }
            if (userCameraIndex == selectedCamera) {
                cameraView_ = static_cast<int>(index + 1);
                break;
            }
            ++userCameraIndex;
        }
        break;
    }
    case RenderBenchmarkPhase::ThirdPersonZoom: {
        cameraView_ = 0;
        const double zoomCycle = 0.5 - 0.5 * std::cos(progress * 2.0 * 3.141592653589793);
        cameraDistance_ = 7.0 + 60.0 * zoomCycle;
        cameraYaw_ = -2.3 + 0.25 * std::sin(progress * 2.0 * 3.141592653589793);
        cameraPitch_ = 0.40 + 0.10 * std::sin(progress * 4.0 * 3.141592653589793);
        break;
    }
    case RenderBenchmarkPhase::DrivingControls:
        cameraView_ = 0;
        cameraDistance_ = 24.0;
        input.throttle = 0.15;
        input.steering = std::sin(progress * 4.0 * 3.141592653589793);
        input.brake = progress >= 0.75 ? 0.25 : 0.0;
        break;
    case RenderBenchmarkPhase::DashboardInteraction: {
        const auto driverCamera = std::find_if(
            vehicleCameras_.begin(), vehicleCameras_.end(),
            [](const VehicleCamera& camera) { return camera.kind == VehicleCameraKind::Driver; });
        cameraView_ =
            driverCamera == vehicleCameras_.end()
                ? 0
                : static_cast<int>(std::distance(vehicleCameras_.begin(), driverCamera)) + 1;
        viewLookYaw_ = 0.35 * std::sin(progress * 2.0 * 3.141592653589793);
        viewLookPitch_ = 0.12 * std::sin(progress * 4.0 * 3.141592653589793);
        fieldOfViewOffset_ = 8.0 * std::sin(progress * 2.0 * 3.141592653589793);
        benchmarkClickDiscoveryPending_ = !benchmarkClickDiscoveryComplete_;
        const int clickInterval = std::max(frameCount / 5, 1);
        benchmarkClickQueued_ = benchmarkClickTargetFound_ && frame % clickInterval == 0;
        break;
    }
    }
    return input;
}

bool RenderLoop::benchmarkClickTargetFound() const {
    return benchmarkClickTargetFound_;
}

bool RenderLoop::consumeCaptureRequest() {
    const bool requested = captureRequested_;
    captureRequested_ = false;
    return requested;
}

bool RenderLoop::isCaptureReady() const {
    return playerVehicle_ && playerVehicle_->isCaptureReady();
}

std::vector<KeyEvent> RenderLoop::consumeKeyEvents() {
    std::vector<KeyEvent> events;
    events.swap(keyEvents_);
    return events;
}

bool RenderLoop::consumeRaisePlayerRequest() {
    const bool requested = raisePlayerRequestPending_;
    raisePlayerRequestPending_ = false;
    return requested;
}
