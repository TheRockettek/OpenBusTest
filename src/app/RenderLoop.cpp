// Window lifecycle, vehicle/map registration and simple accessors.

#include "app/RenderLoopInternal.h"
#include "app/MapTrafficWorker.h"
#include "app/ImGuiLayer.h"
#include "app/vehicle/VehicleDefinition.h"
#include "config/HumanConfigLoader.h"
#include "core/AssetPath.h"
#include "render/map/MapRenderer.h"
#include "map/MapSplineProfile.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <numbers>

namespace {

VehicleFileKind vehicleFileKindForConfig(const std::filesystem::path& configPath) {
    std::string extension = configPath.extension().string();
    std::transform(
        extension.begin(), extension.end(), extension.begin(),
        [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    return extension == ".ovh" ? VehicleFileKind::Vehicle : VehicleFileKind::Bus;
}

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

double positiveEnvironmentRate(const char* name, double defaultValue) {
    const char* value = openbus::getEnvironment(name);
    if (value == nullptr) {
        return defaultValue;
    }
    try {
        std::size_t parsed = 0;
        const double rate = std::stod(value, &parsed);
        if (value[parsed] != '\0' || !std::isfinite(rate) || rate <= 0.0) {
            throw std::invalid_argument("Rate must be finite and positive");
        }
        return std::clamp(rate, 0.01, 1000.0);
    } catch (const std::exception&) {
        gameLog.Warning(std::string("Invalid ") + name + "; ignoring the setting");
        return defaultValue;
    }
}

double minimumObjectViewportFraction(const char* name) {
    const char* value = openbus::getEnvironment(name);
    if (value == nullptr || *value == '\0') {
        return 0.0;
    }
    try {
        std::size_t parsed = 0;
        const double percentage = std::stod(value, &parsed);
        if (value[parsed] != '\0' || !std::isfinite(percentage) || percentage < 0.0 ||
            percentage > 100.0) {
            throw std::invalid_argument("Expected a percentage from 0 to 100");
        }
        return percentage / 100.0;
    } catch (const std::exception&) {
        gameLog.Warning(std::string("Invalid ") + name +
                        "; minimum object size culling is disabled for this view");
        return 0.0;
    }
}

} // namespace

RenderLoop::RenderLoop(int width, int height, const char* title,
                       openbus::app::SteamPresence& steamPresence)
    : steamPresence_(steamPresence), window_(nullptr),
      assetRequestManager_(std::make_unique<AssetRequestManager>()),
      reflectionRenderer_(std::make_unique<openbus::rendering::ReflectionRenderer>()) {
    TraceScope trace("startup", "RenderLoop::RenderLoop");
    const char* odometerMode = openbus::getEnvironment("OPENBUS_ODOMETER_MODE");
    vehicleOdometerMode_ = odometerMode ? openbus::app::parseVehicleOdometerMode(odometerMode)
                                        : openbus::app::parseVehicleOdometerMode();
    try {
        vehicleRegistrationPreferences_ =
            openbus::app::loadVehicleRegistrationPreferences(vehicleRegistrationPreferencesPath_);
    } catch (const std::exception& error) {
        vehicleRegistrationPreferencesLoaded_ = false;
        vehicleRegistrationPreferencesError_ = error.what();
        gameLog.Warning("Vehicle registration preferences were not loaded: " +
                        vehicleRegistrationPreferencesError_);
    }
    try {
        vehicleOdometerPreferences_ =
            openbus::app::loadVehicleOdometerPreferences(vehicleOdometerPreferencesPath_);
    } catch (const std::exception& error) {
        vehicleOdometerPreferencesLoaded_ = false;
        vehicleOdometerPreferencesError_ = error.what();
        gameLog.Warning("Vehicle odometer preferences were not loaded: " +
                        vehicleOdometerPreferencesError_);
    }
    simulationState_.sharedVariables().setSoundAvailable(soundEngine_.isAvailable());
    if (const char* diagnostics = openbus::getEnvironment("OPENBUS_SCRIPT_DIAGNOSTICS")) {
        drivetrainDiagnosticsEnabled_ = std::string(diagnostics) == "1";
    }
    if (const char* diagnostics = openbus::getEnvironment("OPENBUS_MAP_AI_SPAWN_DIAGNOSTICS")) {
        trafficFleet_.trafficSpawnDiagnosticsEnabled = std::string(diagnostics) == "1";
    }
    if (trafficFleet_.trafficSpawnDiagnosticsEnabled) {
        gameLog.Log("Map AI spawn-admission diagnostics enabled (1 Hz)");
    }
    if (drivetrainDiagnosticsEnabled_) {
        gameLog.Log("Native drivetrain variable diagnostics enabled (1 Hz)");
    }
    trafficFleet_.aiSimulationRateHz = positiveEnvironmentRate("OPENBUS_AI_HZ", 0.0);
    trafficFleet_.aiMaximumRateHz = positiveEnvironmentRate("OPENBUS_MAX_FPS", 0.0);
    trafficFleet_.aiRateDivisor = positiveEnvironmentRate("OPENBUS_AI_HZ_DIVISOR", 1.0);
    const double renderFpsCapHz = positiveEnvironmentRate("OPENBUS_FPS_CAP", 0.0);
    frameTiming_.setFrameRateCap(renderFpsCapHz);
    mainViewMinimumObjectViewportFraction_ =
        minimumObjectViewportFraction("OPENBUS_MIN_OBJECT_SIZE_MAIN_VIEW_PERCENT");
    reflectionMinimumObjectViewportFraction_ =
        minimumObjectViewportFraction("OPENBUS_MIN_OBJECT_SIZE_MIRRORS_PERCENT");
    if (mainViewMinimumObjectViewportFraction_ > 0.0) {
        gameLog.Log("Main-view minimum object size culling set to " +
                    std::to_string(mainViewMinimumObjectViewportFraction_ * 100.0) + "%");
    }
    if (reflectionMinimumObjectViewportFraction_ > 0.0) {
        gameLog.Log("Mirror/camera minimum object size culling set to " +
                    std::to_string(reflectionMinimumObjectViewportFraction_ * 100.0) + "%");
    }
    gameLog.Log(trafficFleet_.aiSimulationRateHz > 0.0
                    ? "AI traffic simulation rate set to " +
                          std::to_string(trafficFleet_.aiSimulationRateHz) + " Hz"
                    : "AI traffic simulation rate follows average FPS divided by " +
                          std::to_string(trafficFleet_.aiRateDivisor) +
                          " (60 Hz until an FPS average is available)");
    if (trafficFleet_.aiMaximumRateHz > 0.0) {
        gameLog.Log("AI traffic rate capped at " + std::to_string(trafficFleet_.aiMaximumRateHz) +
                    " Hz");
    }
    if (renderFpsCapHz > 0.0) {
        gameLog.Log("Render frame rate capped at " + std::to_string(renderFpsCapHz) + " FPS");
    } else {
        gameLog.Log("Render frame rate is not capped");
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
            gameLog.Warning("Invalid OPENBUS_SCRIPT_HZ; scripts will follow the render rate");
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
            gameLog.Warning("Invalid OPENBUS_HOVER_HZ; using the default 75 ms hover interval");
        } catch (const std::out_of_range&) {
            gameLog.Warning(
                "OPENBUS_HOVER_HZ out of range; using the default 75 ms hover interval");
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
        gameLog.Error("Failed to initialize GLFW");
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
        gameLog.Warning("OpenGL 4.3 context unavailable; retrying 3.3 with CPU culling");
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        window_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
    }
    if (!window_) {
        gameLog.Error("Failed to create OpenGL window");
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
    vsyncEnabled_ = vsyncSetting == nullptr ||
                    (std::string(vsyncSetting) != "0" && std::string(vsyncSetting) != "off" &&
                     std::string(vsyncSetting) != "false");
    glfwSwapInterval(vsyncEnabled_ ? 1 : 0);
    gameLog.Log(std::string("VSync ") + (vsyncEnabled_ ? "enabled" : "disabled"));
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
    imguiLayer_ = std::make_unique<ImGuiLayer>(window_);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.45f, 0.65f, 0.88f, 1.0f);
    gameLog.Log("RenderLoop initialized");
}

RenderLoop::~RenderLoop() {
    gameLog.Log("RenderLoop shutting down");
    persistPlayerOdometer(true);
    imguiLayer_.reset();
    trafficFleet_.worker.reset();
    for (TrafficVehicleState& state : trafficFleet_.vehicles) {
        if (!state.pendingVehicle.valid()) {
            continue;
        }
        try {
            std::unique_ptr<Vehicle> pendingVehicle = state.pendingVehicle.get();
        } catch (const std::exception& error) {
            gameLog.Warning("Map traffic vehicle loader failed during shutdown: " +
                            std::string(error.what()));
        } catch (...) {
            gameLog.Warning(
                "Map traffic vehicle loader failed during shutdown with an unknown error");
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

void RenderLoop::persistPlayerOdometer(bool force) {
    if (playerVehicle_ == nullptr || !vehicleOdometerPreferencesLoaded_ ||
        playerVehicle_->setupOptions.vehicleConfigurationPath.empty()) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!force && now - lastOdometerPreferenceSave_ < std::chrono::seconds(30)) {
        return;
    }
    lastOdometerPreferenceSave_ = now;

    try {
        openbus::app::setVehicleOdometerReading(
            vehicleOdometerPreferences_, vehicleOdometerMode_,
            playerVehicle_->setupOptions.vehicleConfigurationPath,
            playerVehicle_->odometerInitialKilometres, playerVehicle_->odometerMetres);
        openbus::app::saveVehicleOdometerPreferences(vehicleOdometerPreferencesPath_,
                                                     vehicleOdometerPreferences_);
        vehicleOdometerPreferencesError_.clear();
    } catch (const std::exception& error) {
        vehicleOdometerPreferencesError_ = error.what();
        gameLog.Warning("Vehicle odometer preferences were not saved: " +
                        vehicleOdometerPreferencesError_);
    }
}

Vehicle* RenderLoop::AddVehicle(const std::filesystem::path& vehicleConfigPath,
                                const std::filesystem::path& modelConfigPath,
                                const VehiclePlacement& placement, ModelLoadingPolicy loadingPolicy,
                                bool isAiVehicle, VehicleSetupOptions setupOptions) {
    TraceScope trace("startup", "RenderLoop::AddVehicle");
    const std::filesystem::path resolvedVehicleConfigPath =
        busConfigurationPathFor(vehicleConfigPath);
    const VehicleFileKind configKind = vehicleFileKindForConfig(resolvedVehicleConfigPath);
    const std::filesystem::path resolvedModelConfigPath =
        modelConfigPath.empty()
            ? (configKind == VehicleFileKind::Bus
                   ? modelConfigurationPathForBus(resolvedVehicleConfigPath)
                   : modelConfigurationPathForVehicle(resolvedVehicleConfigPath))
            : (configKind == VehicleFileKind::Bus
                   ? modelConfigurationPathForBus(resolvedVehicleConfigPath, modelConfigPath)
                   : modelConfigurationPathForVehicle(resolvedVehicleConfigPath, modelConfigPath));
    std::shared_ptr<const SharedVehicleDefinition> definition;
    {
        TraceScope definitionTrace("startup", "RenderLoop::AddVehicle.sharedDefinition");
        definition =
            sharedVehicleDefinition(resolvedVehicleConfigPath, resolvedModelConfigPath, configKind);
    }
    const VehicleConfig& vehicleConfiguration = definition->vehicleConfiguration;
    setupOptions.vehicleConfigurationPath = resolvedVehicleConfigPath;
    if (!isAiVehicle) {
        vehicleCameras_.clear();
        for (const VehicleCamera& camera : vehicleConfiguration.cameras) {
            if (camera.kind == VehicleCameraKind::Driver ||
                camera.kind == VehicleCameraKind::Passenger ||
                camera.kind == VehicleCameraKind::Reflexion ||
                camera.kind == VehicleCameraKind::Reflexion2) {
                vehicleCameras_.push_back(camera);
            }
        }
        vehicleScheduleViews_ = vehicleConfiguration.scheduleViews;
        vehicleTicketSellingViews_ = vehicleConfiguration.ticketSellingViews;
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
    std::unique_ptr<Vehicle> model;
    {
        TraceScope vehicleTrace("startup", "RenderLoop::AddVehicle.constructVehicle");
        model = std::make_unique<Vehicle>(
            definition, placementForVehicle(vehicleConfiguration, placement, isAiVehicle),
            modelOffsetZ, loadingPolicy, *assetRequestManager_, simulationState_, soundEngine_,
            soundViewpoint_, isAiVehicle, false, std::move(setupOptions),
            configKind == VehicleFileKind::Bus);
    }
    Vehicle* result = model.get();
    vehicles_.push_back(std::move(model));
    return result;
}

void RenderLoop::RegisterPlayerTimetableRoutes(const openbus::map::MapAiTrafficData& traffic,
                                               double scheduledHumanScale) {
    if (!mapRenderer_) {
        throw std::logic_error("Cannot register player timetable routes before loading a map");
    }
    passengerTimetableCourses_ = traffic.timetableCourses;
    playerTimetableMapKeys_ = openbus::app::scheduledPlayerTimetableKeys(traffic);
    const auto report = passengerNetwork_.registerTimetables(
        traffic, openbus::map::mapScheduledPassengerQueueSize(4, scheduledHumanScale),
        [&](const openbus::map::MapTrafficPassengerStop& stop) {
            return mapRenderer_->timetableStopPose(stop.splineId, stop.pathIndex,
                                                   stop.splinePositionMeters.value_or(0.0));
        });
    updateAutomaticMapHofSelection();
    playerTimetableCatalog_.invalidate();
    gameLog.Log("Player timetable routes registered from map schedules: " +
                std::to_string(passengerNetwork_.routes.size()) +
                "; courses: " + std::to_string(passengerTimetableCourses_.size()));
    if (report.missingStops != 0) {
        gameLog.Warning(
            "Player timetable cannot locate " + std::to_string(report.missingStops) + " stops on " +
            std::to_string(report.routesWithMissingStops) + " routes, including " +
            std::to_string(report.routesWithoutStops) +
            " with no located stops; those stops cannot be serviced. Examples: " +
            report.missingExamples +
            (report.routesWithoutStopExamples.empty()
                 ? ""
                 : ". Routes with no located stops: " + report.routesWithoutStopExamples));
    }
    if (report.routesMissingStopDefinitions != 0) {
        gameLog.Warning("Player timetable cannot register " +
                        std::to_string(report.routesMissingStopDefinitions) +
                        " routes with missing or ambiguous stop definitions. Examples: " +
                        report.missingStopDefinitionExamples);
    }
}

std::optional<std::size_t> RenderLoop::playerTimetableRouteIndex(const std::string& key) const {
    return passengerNetwork_.findRouteIndex(key);
}

bool RenderLoop::AddTrafficVehicle(
    int splineId, int pathTypeCode, const std::filesystem::path& busConfigPath,
    std::optional<std::string> driverModelPath, double speedMetersPerSecond,
    ModelLoadingPolicy loadingPolicy, bool isBus, std::size_t laneSelector, std::size_t routeSeed,
    std::vector<double> scheduledDepartureMinutes, std::optional<std::size_t> unscheduledGroupIndex,
    int defaultVehicleType, std::vector<int> requiredRouteSplineIds,
    std::vector<std::optional<std::size_t>> requiredSceneryPathSequence,
    bool stopAtRequiredSequenceEnd, bool allowIntermediarySegmentsBetweenRequired,
    bool allowReverseTraversalForRequired,
    std::vector<openbus::map::MapTrafficPassengerStop> passengerStops,
    std::string timetableLineName, std::string timetableDestinationName,
    std::optional<std::size_t> trafficRuleGroupIndex, bool registerPassengerRouteOnly,
    std::optional<std::size_t> passengerRouteIndex, std::size_t* registeredPassengerRouteIndex,
    bool requireActiveTileSpawnCandidate, std::vector<int> scheduledDepartureProfileIndices,
    std::shared_ptr<const std::vector<openbus::map::MapAiTimetableProfile>> timetableProfiles,
    std::string timetableKey, std::string timetableTripName) {
    TraceScope trace("startup", "RenderLoop::AddTrafficVehicle");
    if (mapRenderer_ == nullptr) {
        throw std::logic_error("Cannot add map traffic before loading a map");
    }
    if (isBus && !registerPassengerRouteOnly &&
        (requiredRouteSplineIds.empty() || !stopAtRequiredSequenceEnd)) {
        throw std::invalid_argument("Scheduled map buses require a complete timetable route ending "
                                    "at its final checkpoint");
    }
    const auto laneStarts =
        isBus ? mapRenderer_->trafficEntryLanes() : mapRenderer_->trafficSpawnLanes();
    const openbus::map::MapTrafficLaneStart requestedStart{splineId, laneSelector};
    const bool isExistingEntry =
        std::find(laneStarts.begin(), laneStarts.end(), requestedStart) != laneStarts.end();
    const auto timetableEntryLanes = isBus && !requiredRouteSplineIds.empty()
                                         ? mapRenderer_->lanesForTimetableEntry(
                                               splineId, requiredSceneryPathSequence.empty()
                                                             ? std::nullopt
                                                             : requiredSceneryPathSequence.front())
                                         : std::vector<openbus::map::MapTrafficLaneStart>{};
    const bool isScheduledTimetableStart =
        std::find(timetableEntryLanes.begin(), timetableEntryLanes.end(), requestedStart) !=
        timetableEntryLanes.end();
    if (pathTypeCode != openbus::map::MAP_SPLINE_PATH_TYPE_ROAD ||
        (!isExistingEntry && !isScheduledTimetableStart)) {
        return false;
    }
    const auto resolvedConfig = busConfigurationPathFor(busConfigPath);
    const VehicleFileKind configKind = vehicleFileKindForConfig(resolvedConfig);
    const std::filesystem::path modelConfig =
        configKind == VehicleFileKind::Bus ? modelConfigurationPathForBusFromConfig(resolvedConfig)
                                           : modelConfigurationPathForVehicle(resolvedConfig);
    std::shared_ptr<const SharedVehicleDefinition> definition;
    {
        TraceScope definitionTrace("startup", "RenderLoop::AddTrafficVehicle.sharedDefinition");
        definition = sharedVehicleDefinition(resolvedConfig, modelConfig, configKind);
    }
    const auto& configuration = definition->vehicleConfiguration;
    if (!configuration.hasBoundingBox || configuration.boundingBox[0] <= 0.0 ||
        configuration.boundingBox[1] <= 0.0) {
        throw std::runtime_error("Map traffic requires a valid vehicle bounding box: " +
                                 busConfigPath.string());
    }
    const int vehicleType = isBus ? 2 : configuration.vehicleType.value_or(defaultVehicleType);
    const double turnRadius = calculateTrafficMinimumTurnRadius(configuration, vehicleType == 2);
    if (!isBus && !trafficRuleGroupIndex.has_value()) {
        throw std::invalid_argument("Unscheduled map traffic requires a vehicle-rule group");
    }
    const openbus::map::MapTrafficVehicleFilter vehicleFilter{
        vehicleType, isBus ? std::nullopt : unscheduledGroupIndex, isBus,
        isBus ? std::nullopt : trafficRuleGroupIndex};
    const double routeSearchTurnRadius =
        isBus && !requiredRouteSplineIds.empty() ? 0.0 : turnRadius;
    std::optional<openbus::map::MapTrafficRoute> route;
    {
        TraceScope routeTrace("startup", "RenderLoop::AddTrafficVehicle.buildRoute");
        route = mapRenderer_->buildDrivingRoute(
            splineId, laneSelector, routeSeed, routeSearchTurnRadius, vehicleFilter,
            requiredRouteSplineIds, requiredSceneryPathSequence, stopAtRequiredSequenceEnd,
            allowIntermediarySegmentsBetweenRequired, allowReverseTraversalForRequired);
        if (route) {
            *route = openbus::map::subdivideMapTrafficRoute(*route);
        }
    }
    if (!route) {
        return false;
    }
    auto actor = openbus::map::makeMapTrafficActor(std::move(*route), busConfigPath.string(),
                                                   speedMetersPerSecond, 0.0, false,
                                                   std::move(driverModelPath));
    if (isBus && !registerPassengerRouteOnly) {
        if (scheduledDepartureMinutes.empty()) {
            throw std::invalid_argument("Scheduled map buses require timetable departures");
        }
        if (std::any_of(scheduledDepartureMinutes.begin(), scheduledDepartureMinutes.end(),
                        [](double minute) { return !std::isfinite(minute) || minute < 0.0; })) {
            throw std::invalid_argument("Map bus timetable departures are invalid");
        }
        if (scheduledDepartureProfileIndices.empty()) {
            scheduledDepartureProfileIndices.resize(scheduledDepartureMinutes.size(), 0);
        } else if (scheduledDepartureProfileIndices.size() != scheduledDepartureMinutes.size()) {
            throw std::invalid_argument(
                "Map bus timetable departures and profiles are not aligned");
        }
        std::vector<std::pair<double, int>> scheduledTrips;
        scheduledTrips.reserve(scheduledDepartureMinutes.size());
        for (std::size_t index = 0; index < scheduledDepartureMinutes.size(); ++index) {
            scheduledTrips.emplace_back(scheduledDepartureMinutes[index],
                                        std::max(0, scheduledDepartureProfileIndices[index]));
        }
        std::stable_sort(
            scheduledTrips.begin(), scheduledTrips.end(),
            [](const auto& left, const auto& right) { return left.first < right.first; });
        for (std::size_t index = 0; index < scheduledTrips.size(); ++index) {
            scheduledDepartureMinutes[index] = scheduledTrips[index].first;
            scheduledDepartureProfileIndices[index] = scheduledTrips[index].second;
        }
        if (timetableProfiles) {
            for (const openbus::map::MapAiTimetableProfile& profile : *timetableProfiles) {
                const auto validOffsets = [](const std::vector<double>& offsets) {
                    return std::all_of(offsets.begin(), offsets.end(), [](double seconds) {
                        return std::isfinite(seconds) && seconds >= 0.0;
                    });
                };
                if (profile.arrivalOffsetSeconds.size() != profile.departureOffsetSeconds.size() ||
                    !validOffsets(profile.arrivalOffsetSeconds) ||
                    !validOffsets(profile.departureOffsetSeconds)) {
                    throw std::invalid_argument("Map bus timetable profile times are invalid");
                }
                for (std::size_t station = 0; station < profile.arrivalOffsetSeconds.size();
                     ++station) {
                    if (profile.departureOffsetSeconds[station] <
                        profile.arrivalOffsetSeconds[station]) {
                        throw std::invalid_argument(
                            "Map bus timetable departure offset precedes arrival offset");
                    }
                }
            }
        }
    } else if (!scheduledDepartureMinutes.empty() || !scheduledDepartureProfileIndices.empty() ||
               (timetableProfiles && !timetableProfiles->empty())) {
        throw std::invalid_argument("Only map buses can have timetable departures");
    }
    actor.speedMetersPerSecond = 0.0;
    actor.minimumTurnRadiusMeters = turnRadius;
    actor.vehicleType = vehicleType;
    actor.unscheduledGroupIndex = isBus ? std::nullopt : unscheduledGroupIndex;
    actor.trafficRuleGroupIndex = isBus ? std::nullopt : trafficRuleGroupIndex;
    actor.scheduled = isBus;
    actor.isBus = isBus;
    actor.lengthMeters =
        configuration.boundingBox[1] + 2.0 * std::abs(configuration.boundingBox[4]);
    actor.widthMeters = configuration.boundingBox[0] + 2.0 * std::abs(configuration.boundingBox[3]);
    actor.rearAxleOffsetMeters =
        configuration.axles.empty()
            ? -configuration.boundingBox[1] * 0.3
            : std::min_element(configuration.axles.begin(), configuration.axles.end(),
                               [](const BusAxle& left, const BusAxle& right) {
                                   return left.position < right.position;
                               })
                  ->position;
    std::vector<openbus::map::MapTrafficSpawnCandidate> spawnCandidates;
    if (!isBus) {
        spawnCandidates =
            openbus::map::mapTrafficRouteSpawnCandidates(actor.route, actor.lengthMeters);
        if (spawnCandidates.empty()) {
            return false;
        }
        if (requireActiveTileSpawnCandidate) {
            if (!openbus::map::mapTrafficSpawnCandidatesOverlapActiveTiles(
                    spawnCandidates, mapRenderer_->activeTileCoordinates())) {
                return false;
            }
        }
    }

    const std::optional<openbus::map::MapTrafficPose> pose =
        openbus::map::sampleMapTrafficActorPose(actor, 0.0);
    if (!pose.has_value()) {
        throw std::runtime_error("Could not sample the initial map traffic route pose");
    }
    VehiclePlacement placement;
    placement.position = pose->position;
    placement.yawDegrees = openbus::map::mapTrafficHeadingToVehicleYawDegrees(pose->headingRadians);
    placement.pitchDegrees = pose->pitchRadians * 180.0 / std::numbers::pi;
    placement.rollDegrees = pose->rollRadians * 180.0 / std::numbers::pi;
    const VehiclePlacement configuredPlacement =
        placementForVehicle(configuration, placement, true);
    const double modelOffsetZ = -configuration.centerOfGravityHeight;
    TrafficVehicleState trafficState;
    trafficState.actor = std::move(actor);
    trafficState.workerActorTemplate =
        std::make_shared<const openbus::map::MapTrafficActor>(trafficState.actor);
    if (!isBus) {
        trafficState.spawnCandidates =
            std::make_shared<const std::vector<openbus::map::MapTrafficSpawnCandidate>>(
                std::move(spawnCandidates));
    }
    if (!isBus && (!passengerStops.empty() || registerPassengerRouteOnly)) {
        throw std::invalid_argument("Only scheduled map buses can service passenger stops");
    }
    if (isBus) {
        trafficState.bus = std::make_unique<openbus::app::ScheduledBusState>();
        trafficState.bus->scheduledDepartureMinutes = std::move(scheduledDepartureMinutes);
        trafficState.bus->scheduledDepartureProfileIndices =
            std::move(scheduledDepartureProfileIndices);
        trafficState.bus->timetableProfiles = std::move(timetableProfiles);
        trafficState.bus->timetableKey = timetableKey;
        trafficState.bus->timetableLineName = std::move(timetableLineName);
        trafficState.bus->timetableDestinationName = std::move(timetableDestinationName);
        auto mappedStops = openbus::app::mapPassengerStopsToTrafficRoute(
            trafficState.actor.route, std::move(passengerStops),
            [&](const openbus::map::MapTrafficPassengerStop& stop) {
                gameLog.Warning(
                    "Scheduled passenger stop could not be matched to the bus route; "
                    "stopId=" +
                    std::to_string(stop.stopId) + " spline=" + std::to_string(stop.splineId) +
                    " path=" + std::to_string(stop.pathIndex) + " bus=" + busConfigPath.string());
            });
        if (!mappedStops.empty()) {
            trafficState.bus->timetableTerminusIndex = static_cast<int>(mappedStops.size() - 1);
        }
        if (!registerPassengerRouteOnly && passengerRouteIndex && !mappedStops.empty()) {
            if (*passengerRouteIndex >= passengerNetwork_.routes.size()) {
                throw std::invalid_argument("Passenger route catalog index is out of range");
            }
            trafficState.bus->passengerStops = openbus::app::PassengerStops::shareIfEqual(
                std::move(mappedStops), passengerNetwork_.routes[*passengerRouteIndex].stops);
        } else {
            trafficState.bus->passengerStops = openbus::app::PassengerStops(std::move(mappedStops));
        }
        if (registerPassengerRouteOnly) {
            if (!isBus || trafficState.bus->passengerStops.empty()) {
                throw std::invalid_argument(
                    "Passenger-route-only registration requires a bus route with stops and no "
                    "scheduled vehicle departures");
            }
            const std::size_t routeIndex = passengerNetwork_.routes.size();
            passengerNetwork_.routes.push_back({trafficState.workerActorTemplate,
                                                trafficState.bus->passengerStops,
                                                trafficState.bus->timetableKey,
                                                trafficState.bus->timetableLineName,
                                                trafficState.bus->timetableDestinationName,
                                                std::move(timetableTripName),
                                                {},
                                                {}});
            playerTimetableCatalog_.invalidate();
            if (registeredPassengerRouteIndex != nullptr) {
                *registeredPassengerRouteIndex = routeIndex;
            }
            trafficState.bus->passengerRouteIndex = routeIndex;
            passengerNetwork_.initializeQueues(trafficState.bus->passengerStops.values());
            return true;
        }
        if (isBus && !trafficState.bus->passengerStops.empty()) {
            if (passengerRouteIndex) {
                trafficState.bus->passengerRouteIndex = passengerRouteIndex;
            } else {
                trafficState.bus->passengerRouteIndex = passengerNetwork_.routes.size();
                passengerNetwork_.routes.push_back({trafficState.workerActorTemplate,
                                                    trafficState.bus->passengerStops,
                                                    trafficState.bus->timetableKey,
                                                    trafficState.bus->timetableLineName,
                                                    trafficState.bus->timetableDestinationName,
                                                    std::move(timetableTripName),
                                                    {},
                                                    {}});
                playerTimetableCatalog_.invalidate();
            }
        }
        passengerNetwork_.initializeQueues(trafficState.bus->passengerStops.values());
    }
    if (isBus) {
        trafficState.bus->nextScheduledDepartureIndex = openbus::map::mapTrafficNextDepartureIndex(
            trafficFleet_.mapTrafficStartHour, 0.0, trafficState.bus->scheduledDepartureMinutes);
        trafficState.bus->nextScheduledDepartureElapsedSeconds =
            openbus::map::mapTrafficNextDepartureElapsedSeconds(
                trafficFleet_.mapTrafficStartHour, 0.0,
                trafficState.bus
                    ->scheduledDepartureMinutes[trafficState.bus->nextScheduledDepartureIndex]);
    }
    AssetRequestManager* const assetRequests = assetRequestManager_.get();
    SimulationState* const simulationState = &simulationState_;
    SoundEngine* const soundEngine = &soundEngine_;
    openbus::rendering::ViewpointContext* const soundViewpoint = &soundViewpoint_;
    std::future<std::unique_ptr<Vehicle>> pendingVehicle =
        assetRequests->submit([definition, configuredPlacement, modelOffsetZ, loadingPolicy,
                               assetRequests, simulationState, soundEngine, soundViewpoint, isBus] {
            return std::make_unique<Vehicle>(definition, configuredPlacement, modelOffsetZ,
                                             loadingPolicy, *assetRequests, *simulationState,
                                             *soundEngine, *soundViewpoint, true, true,
                                             VehicleSetupOptions{}, isBus);
        });
    trafficState.pendingVehicle = std::move(pendingVehicle);
    trafficFleet_.vehicles.push_back(std::move(trafficState));
    return true;
}

int RenderLoop::trafficVehicleType(const std::filesystem::path& vehicleConfigPath,
                                   int defaultVehicleType) const {
    if (mapRenderer_ == nullptr) {
        throw std::logic_error("Cannot query map traffic vehicle types before loading a map");
    }
    const auto resolvedConfig = busConfigurationPathFor(vehicleConfigPath);
    const VehicleFileKind configKind = vehicleFileKindForConfig(resolvedConfig);
    const std::filesystem::path modelConfig =
        configKind == VehicleFileKind::Bus ? modelConfigurationPathForBusFromConfig(resolvedConfig)
                                           : modelConfigurationPathForVehicle(resolvedConfig);
    const auto definition = sharedVehicleDefinition(resolvedConfig, modelConfig, configKind);
    return definition->vehicleConfiguration.vehicleType.value_or(defaultVehicleType);
}

void RenderLoop::SetMapTrafficStartHour(double startHour) {
    if (!std::isfinite(startHour) || startHour < 0.0 || startHour >= 24.0) {
        throw std::invalid_argument("Map traffic start hour must be between 0 and 24");
    }
    trafficFleet_.mapTrafficStartHour = startHour;
    trafficFleet_.mapTrafficStartSimulationTime = 0.0;
}

void RenderLoop::SetMapCalendar(int year, int month, int day, double timeSeconds,
                                double simulationTime) {
    if (!std::isfinite(simulationTime) || simulationTime < 0.0) {
        throw std::invalid_argument("Simulation time is invalid");
    }
    simulationState_.sharedVariables().configureSimulationClock(year, month, day, timeSeconds);
    calendarElapsedSeconds_ = 0.0;
    previousCalendarSimulationTime_ = simulationTime;
    hasPreviousCalendarSimulationTime_ = true;
    rebaseMapTrafficClock(simulationTime, simulationTime, timeSeconds / 3600.0);
}

MapCalendarRequest RenderLoop::currentMapCalendar() const {
    const SystemVariables& variables = simulationState_.sharedVariables();
    return {static_cast<int>(variables.get("year")), static_cast<int>(variables.get("month")),
            static_cast<int>(variables.get("day")), variables.get("time")};
}

void RenderLoop::ResetMapClockEpoch(double simulationTime) {
    ResetMapClockEpoch(simulationTime, simulationTime);
}

void RenderLoop::ResetMapClockEpoch(double previousSimulationTime, double newSimulationTime) {
    if (!std::isfinite(previousSimulationTime) || previousSimulationTime < 0.0 ||
        !std::isfinite(newSimulationTime) || newSimulationTime < 0.0) {
        throw std::invalid_argument("Simulation time is invalid");
    }
    const MapCalendarRequest current = currentMapCalendar();
    simulationState_.sharedVariables().configureSimulationClock(
        current.year, current.month, current.day, current.timeSeconds);
    calendarElapsedSeconds_ = 0.0;
    previousCalendarSimulationTime_ = newSimulationTime;
    hasPreviousCalendarSimulationTime_ = true;
    rebaseMapTrafficClock(previousSimulationTime, newSimulationTime,
                          simulationState_.sharedVariables().get("time") / 3600.0);
}

double RenderLoop::mapTrafficElapsedSeconds(double simulationTime) const {
    return std::max(0.0, simulationTime - trafficFleet_.mapTrafficStartSimulationTime);
}

void RenderLoop::rebaseMapTrafficClock(double previousSimulationTime, double newSimulationTime,
                                       double newStartHour) {
    const double previousElapsedSeconds = mapTrafficElapsedSeconds(previousSimulationTime);
    clearPlayerTripAiPolicy();
    for (TrafficVehicleState& state : trafficFleet_.vehicles) {
        if (!state.bus) {
            continue;
        }
        openbus::app::rebaseScheduledBusClock(*state.bus, newStartHour,
                                              previousElapsedSeconds);
    }
    trafficFleet_.mapTrafficStartHour = newStartHour;
    trafficFleet_.mapTrafficStartSimulationTime = newSimulationTime;
    if (playerTimetable_.state &&
        playerTimetable_.state->selection.courseIndex < passengerTimetableCourses_.size()) {
        const auto& trips =
            passengerTimetableCourses_[playerTimetable_.state->selection.courseIndex].trips;
        const std::size_t tripIndex = playerTimetable_.state->selection.courseTripIndex;
        if (tripIndex < trips.size()) {
            applyPlayerTripAiPolicy(
                trips[tripIndex], playerTimetable_.state->selection.driveAsAdditionalBus);
        }
    }
}

void RenderLoop::SetMapTrafficSpawnInView(bool enabled) {
    trafficFleet_.mapTrafficSpawnInView = enabled;
}

double RenderLoop::trafficMinimumTurnRadius(const std::filesystem::path& vehicleConfigPath,
                                            bool isBus) const {
    const std::filesystem::path resolvedConfig = busConfigurationPathFor(vehicleConfigPath);
    const VehicleFileKind configKind = isBus ? VehicleFileKind::Bus : VehicleFileKind::Vehicle;
    const std::filesystem::path resolvedModelConfig =
        isBus ? modelConfigurationPathForBusFromConfig(resolvedConfig)
              : modelConfigurationPathForVehicle(resolvedConfig);
    const auto definition =
        sharedVehicleDefinition(resolvedConfig, resolvedModelConfig, configKind);
    return calculateTrafficMinimumTurnRadius(definition->vehicleConfiguration, isBus);
}

bool RenderLoop::AddHumanTrafficActor(int splineId, const std::filesystem::path& humanConfigPath,
                                      double feetDistanceMeters, double heightMeters,
                                      double walkStrideLengthMeters, double speedMetersPerSecond,
                                      double startDistanceMeters, double centerX, double centerY,
                                      double maximumDistanceMeters) {
    if (mapRenderer_ == nullptr) {
        throw std::logic_error("Cannot add human traffic before loading a map");
    }
    if (humanConfigPath.empty() || !std::isfinite(feetDistanceMeters) ||
        feetDistanceMeters <= 0.0 || !std::isfinite(heightMeters) || heightMeters <= 0.0 ||
        !std::isfinite(walkStrideLengthMeters) || walkStrideLengthMeters <= 0.0) {
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
    humanTrafficActors_.push_back({std::move(*actor),
                                   {},
                                   feetDistanceMeters,
                                   heightMeters,
                                   walkStrideLengthMeters,
                                   {},
                                   false});
    humanTrafficActors_.back().workerActorTemplate =
        std::make_shared<const openbus::map::MapTrafficActor>(humanTrafficActors_.back().actor);
    return true;
}

void RenderLoop::SetScheduledPassengerHumanModels(
    std::vector<std::filesystem::path> humanConfigPaths) {
    TraceScope trace("startup", "RenderLoop::SetScheduledPassengerHumanModels");
    if (mapRenderer_ == nullptr) {
        throw std::logic_error("Cannot load scheduled passenger models before loading a map");
    }
    scheduledPassengerHumanModels_.clear();
    scheduledPassengerHumanAges_.clear();
    scheduledPassengerHumanMassesKg_.clear();
    for (const std::filesystem::path& path : humanConfigPaths) {
        if (path.empty() || !std::filesystem::is_regular_file(path)) {
            continue;
        }
        mapRenderer_->loadHumanTrafficModel(path);
        scheduledPassengerHumanModels_.push_back(path);
        const auto human = openbus::config::loadHumanConfig(path);
        scheduledPassengerHumanAges_.push_back(human.age);
        scheduledPassengerHumanMassesKg_.push_back(
            human.mass ? std::optional<double>{*human.mass * 1000.0} : std::nullopt);
    }
}

void RenderLoop::SetMapTrafficPopulation(
    std::size_t roadVehicleCount, std::size_t busCount,
    std::vector<openbus::map::MapAiRoadTrafficGroupPopulation> roadGroupPopulations,
    bool useGroupRoadPopulation) {
    trafficFleet_.targetRoadTrafficCount = roadVehicleCount;
    trafficFleet_.targetRoadGroupPopulations = std::move(roadGroupPopulations);
    trafficFleet_.useGroupRoadPopulation = useGroupRoadPopulation;
    trafficFleet_.targetBusTrafficCount = busCount;
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

std::vector<openbus::map::MapTrafficLaneStart>
RenderLoop::trafficLanesForTimetableEntry(int splineId,
                                          std::optional<std::size_t> sceneryPathIndex) const {
    return mapRenderer_ == nullptr
               ? std::vector<openbus::map::MapTrafficLaneStart>{}
               : mapRenderer_->lanesForTimetableEntry(splineId, sceneryPathIndex);
}

std::optional<openbus::map::MapTrafficLaneConnectionGap>
RenderLoop::trafficLaneConnectionGap(int fromSplineId, std::optional<std::size_t> fromPathIndex,
                                     int toSplineId, std::optional<std::size_t> toPathIndex) const {
    return mapRenderer_ == nullptr ? std::nullopt
                                   : mapRenderer_->nearestTrafficLaneConnectionGap(
                                         fromSplineId, fromPathIndex, toSplineId, toPathIndex);
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

std::vector<openbus::map::MapTrafficRouteRequirement> RenderLoop::trafficRouteSegments(
    int splineId, int pathTypeCode, std::size_t laneSelector, double minimumTurnRadiusMeters,
    std::size_t routeSeed, const std::vector<int>& requiredSplineSequence,
    openbus::map::MapTrafficVehicleFilter vehicle,
    const std::vector<std::optional<std::size_t>>& requiredSceneryPathSequence,
    bool stopAtRequiredSequenceEnd, bool allowIntermediarySegmentsBetweenRequired,
    bool allowReverseTraversalForRequired) const {
    if (mapRenderer_ == nullptr) {
        return {};
    }
    const std::optional<openbus::map::MapTrafficRoute> route =
        pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_ROAD
            ? mapRenderer_->buildDrivingRoute(
                  splineId, laneSelector, routeSeed, minimumTurnRadiusMeters, vehicle,
                  requiredSplineSequence, requiredSceneryPathSequence, stopAtRequiredSequenceEnd,
                  allowIntermediarySegmentsBetweenRequired, allowReverseTraversalForRequired)
            : mapRenderer_->buildTrafficRoute(splineId, pathTypeCode);
    if (!route.has_value()) {
        return {};
    }
    std::vector<openbus::map::MapTrafficRouteRequirement> routeSegments;
    routeSegments.reserve(route->segments.size());
    for (const openbus::map::MapTrafficRouteSegment& segment : route->segments) {
        if (segment.spline.placement.splineId != 0) {
            routeSegments.push_back({segment.spline.placement.splineId, segment.sceneryPathIndex});
        }
    }
    return routeSegments;
}

std::size_t RenderLoop::trafficVehicleCount() const {
    return static_cast<std::size_t>(
        std::count_if(trafficFleet_.vehicles.begin(), trafficFleet_.vehicles.end(),
                      [](const TrafficVehicleState& state) {
                          return state.populationEnabled && state.vehicle != nullptr &&
                                 !state.vehicle->mapTrafficRouteFinished;
                      }));
}

void RenderLoop::SetWeather(int precipitationType, double precipitationRate) {
    simulationState_.sharedVariables().setPrecipitation(precipitationType, precipitationRate);
}

void RenderLoop::SetWeather(int precipitationType, double precipitationRate,
                            double temperatureCelsius, double absoluteHumidityGramsPerCubicMeter) {
    simulationState_.sharedVariables().setWeather(precipitationType, precipitationRate,
                                                  temperatureCelsius,
                                                  absoluteHumidityGramsPerCubicMeter);
}

void RenderLoop::SetAutoClutch(bool enabled) {
    simulationState_.sharedVariables().setAutoClutch(enabled);
}

void RenderLoop::SetSunAltitude(double altitudeDegrees) {
    simulationState_.sharedVariables().setSunAltitude(altitudeDegrees);
}

void RenderLoop::SetPlayerVehicle(Vehicle* model) {
    if (model != playerVehicle_) {
        persistPlayerOdometer(true);
        if (model != nullptr) {
            if (!model->setupOptions.vehicleConfigurationPath.empty()) {
                model->odometerMetres = openbus::app::vehicleOdometerReading(
                    vehicleOdometerPreferences_, vehicleOdometerMode_,
                    model->setupOptions.vehicleConfigurationPath, model->odometerInitialKilometres);
            }
            model->odometerPositionInitialized = false;
        }
    }
    playerTimetableCatalog_.invalidate();
    playerVehicle_ = model;
    if (playerVehicle_ != nullptr) {
        mapBusVehicle_ = playerVehicle_;
        playerPassengerServiceEnabled_ = true;
        freeCameraEnabled_ = false;
        if (mouseLookEnabled_) {
            glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            glfwGetCursorPos(window_, &previousCursorX_, &previousCursorY_);
        }
        playerVehicle_->initializePlayerSystems();
    }
}

void RenderLoop::SetParkedPlayerBus(Vehicle* vehicle) {
    if (vehicle == nullptr || playerVehicle_ != nullptr) {
        throw std::invalid_argument("A parked player bus requires an unoccupied vehicle");
    }
    mapBusVehicle_ = vehicle;
}

void RenderLoop::RemovePlayerVehicle() {
    Vehicle* removedVehicle = mapBusVehicle_ != nullptr ? mapBusVehicle_ : playerVehicle_;
    if (removedVehicle == nullptr) {
        return;
    }

    SetPlayerTimetableSelection(std::nullopt);
    SetPlayerVehicle(nullptr);
    mapBusVehicle_ = nullptr;
    mouseControlEnabled_ = false;
    mouseLookEnabled_ = false;
    glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    glfwSetCursor(window_, nullptr);
    vehicles_.erase(std::remove_if(vehicles_.begin(), vehicles_.end(),
                                  [removedVehicle](const std::unique_ptr<Vehicle>& vehicle) {
                                      return vehicle.get() == removedVehicle;
                                  }),
                    vehicles_.end());
    vehicleCameras_.clear();
    vehicleScheduleViews_.clear();
    vehicleTicketSellingViews_.clear();
    cameraView_ = 0;
    pauseMenuOpen_ = false;
    setPausedState(false);
}

void RenderLoop::ResetForStartupMenu() {
    if (playerTimetable_.state) {
        SetPlayerTimetableSelection(std::nullopt);
    }
    clearPlayerTripAiPolicy();
    trafficFleet_.worker.reset();
    trafficFleet_.vehicles.clear();
    trafficFleet_ = {};
    SetPlayerVehicle(nullptr);
    vehicles_.clear();
    humanTrafficActors_.clear();
    alightedPassengers_.clear();
    scheduledPassengerHumanModels_.clear();
    scheduledPassengerHumanAges_.clear();
    scheduledPassengerHumanMassesKg_.clear();
    passengerTicketPack_ = {};
    passengerCurrency_ = {};
    passengerNetwork_ = {};
    passengerTimetableCourses_.clear();
    playerTimetableCatalog_.invalidate();
    playerTimetableMapKeys_.clear();
    playerTimetableMapPath_.clear();
    playerTimetable_.state.reset();
    authoredPassengerPositions_.clear();
    authoredPassengerQueueCounts_.clear();
    reportedPassengerAreaStops_.clear();
    reportedPassengerJourneyStops_.clear();
    reportedPassengerPedestrianStops_.clear();
    reportedPassengerHandoffStops_.clear();
    reportedPassengerDriftStops_.clear();
    passengerPedestrianPaths_.clear();
    passengerApproachProgress_.clear();
    reportedPassengerTicketStops_.clear();
    playerPassengerStopLatch_.reset();
    playerPassengerStopLatchRouteIndex_.reset();
    playerPassengerStopLatchIndex_.reset();
    activePlayerPassengerStop_.reset();
    activePlayerPassengerStopRouteIndex_.reset();
    activePlayerPassengerStopIndex_.reset();
    playerPassengerServiceRouteIndex_.reset();
    playerPassengerServiceNextStopIndex_ = 0;
    playerTimetableStopVisitLatchPosition_.reset();
    ++playerPassengerServiceGeneration_;
    mapRenderer_.reset();
    vehicleCameras_.clear();
    vehicleScheduleViews_.clear();
    vehicleTicketSellingViews_.clear();
    mouseControlEnabled_ = false;
    mouseLookEnabled_ = false;
    freeCameraEnabled_ = false;
    clickableDebugOverlay_ = false;
    collisionDebugOverlay_ = false;
    tileBoundaryDebugOverlay_ = false;
    trafficSplineDebugOverlay_ = false;
    clickableHoverCacheValid_ = false;
    targetHumanTrafficCount_ = 0;
    authoredPassengerObjectCount_ = 0;
    playerPassengerServiceEnabled_ = true;
    spawnBusRequestPending_ = false;
    enterMapBusRequestPending_ = false;
    leaveMapBusRequestPending_ = false;
    mapBusTeleportRequestPending_.reset();
    removePlayerBusRequestPending_ = false;
    openSpawnPopup_ = false;
    openPlayerTimetablePopup_ = false;
    pauseMenuOpen_ = false;
    debugMenuOpen_ = false;
    debugCalendarInitialized_ = false;
    mapCalendarRequestPending_.reset();
    quitToMenuRequested_ = false;
    calendarElapsedSeconds_ = 0.0;
    hasPreviousCalendarSimulationTime_ = false;
    paused_ = false;
    simulationState_ = SimulationState{};
    mapEntryPoints_.clear();
    selectedMapSpawnEntryPointIndex_ = 0;
    mapSpawnInBus_ = true;
    mapBusVehicle_ = nullptr;
    onFootState_ = {};
    freeCameraPose_ = {};
    glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    glfwSetCursor(window_, nullptr);
}

void RenderLoop::SetMap(const openbus::map::MapDefinition& map, std::size_t spawnEntryPointIndex,
                        std::size_t groundTextureIndex, const std::filesystem::path& omsiRoot,
                        bool prepareInitialWindowAsync) {
    TraceScope trace("map", "RenderLoop::SetMap");
    auto& previousMapVariables = simulationState_.sharedVariables();
    for (std::size_t index = 0; index < passengerTicketPack_.tickets.size(); ++index) {
        previousMapVariables.setString("ticket_name_" + std::to_string(index), "");
        previousMapVariables.set("ticket_value_" + std::to_string(index), 0.0F);
    }
    for (std::size_t index = 0; index < passengerCurrency_.coins.size(); ++index) {
        previousMapVariables.set("cash_coin_value_" + std::to_string(index), 0.0F);
        previousMapVariables.setString("cash_coin_model_" + std::to_string(index), "");
    }
    passengerTicketPack_ = {};
    passengerCurrency_ = {};
    passengerApproachProgress_.clear();
    reportedPassengerTicketStops_.clear();
    if (spawnEntryPointIndex >= map.entryPoints.size()) {
        throw std::out_of_range("Selected map spawn entrypoint is outside [entrypoints]");
    }
    mapEntryPoints_ = map.entryPoints;
    selectedMapSpawnEntryPointIndex_ = spawnEntryPointIndex;
    passengerTimetableCourses_.clear();
    playerTimetableCatalog_.invalidate();
    playerTimetableMapPath_ = openbus::pathToUtf8(map.rootPath);
    playerTimetableMapKeys_.clear();
    if (prepareInitialWindowAsync) {
        mapRenderer_.reset();
    }
    if (!mapRenderer_) {
        TraceScope rendererTrace("map", "RenderLoop::SetMap.createMapRenderer");
        authoredPassengerPositions_.clear();
        authoredPassengerQueueCounts_.clear();
        authoredPassengerObjectCount_ = 0;
        reportedPassengerAreaStops_.clear();
        reportedPassengerPedestrianStops_.clear();
        reportedPassengerHandoffStops_.clear();
        reportedPassengerDriftStops_.clear();
        passengerPedestrianPaths_.clear();
        mapRenderer_ = std::make_unique<openbus::rendering::MapRenderer>(
            map, static_cast<std::size_t>(map.entryPoints[spawnEntryPointIndex].tileIndex),
            map.entryPoints[spawnEntryPointIndex].placement.position[0],
            map.entryPoints[spawnEntryPointIndex].placement.position[1], groundTextureIndex,
            omsiRoot, simulationState_, *assetRequestManager_, prepareInitialWindowAsync);
    } else if (!prepareInitialWindowAsync) {
        mapRenderer_->enableScripts();
    }
    const auto resolveMapAsset =
        [&](const std::string& configuredPath,
            std::string_view assetName) -> std::optional<std::filesystem::path> {
        if (configuredPath.empty()) {
            return std::nullopt;
        }
        std::string normalizedPath = configuredPath;
        std::replace(normalizedPath.begin(), normalizedPath.end(), '\\', '/');
        const std::filesystem::path resolvedPath =
            openbus::assets::resolveCaseInsensitivePath(omsiRoot / normalizedPath);
        if (!std::filesystem::is_regular_file(resolvedPath)) {
            gameLog.Warning("Map " + std::string(assetName) +
                            " configuration is missing: " + resolvedPath.generic_string());
            return std::nullopt;
        }
        return resolvedPath;
    };
    auto& globalVariables = simulationState_.sharedVariables();
    if (const auto ticketPackPath = resolveMapAsset(map.metadata.ticketPack, "ticket pack")) {
        TraceScope ticketTrace("map", "RenderLoop::SetMap.loadTicketPack");
        const auto ticketPack = openbus::map::loadMapTicketPackDefinition(*ticketPackPath);
        passengerTicketPack_ = ticketPack;
        for (std::size_t index = 0; index < ticketPack.tickets.size(); ++index) {
            const openbus::map::MapTicketDefinition& ticket = ticketPack.tickets[index];
            globalVariables.setString("ticket_name_" + std::to_string(index), ticket.name);
            globalVariables.set("ticket_value_" + std::to_string(index),
                                static_cast<float>(ticket.value));
        }
        for (const ConfigurationDiagnostic& diagnostic : ticketPack.diagnostics.entries) {
            gameLog.Log(diagnostic.severity == ConfigurationDiagnostic::Severity::Error
                            ? LogLevel::Error
                            : LogLevel::Warning,
                        "Map ticket-pack diagnostic line=" + std::to_string(diagnostic.line) +
                            " [" + diagnostic.keyword + "] " + diagnostic.message);
        }
    }
    if (const auto currencyPath = resolveMapAsset(map.metadata.moneySystem, "currency")) {
        const auto currency = openbus::map::loadMapCurrencyDefinition(*currencyPath);
        passengerCurrency_ = currency;
        globalVariables.setString("cash_currency_name", currency.name);
        for (std::size_t index = 0; index < currency.coins.size(); ++index) {
            globalVariables.set("cash_coin_value_" + std::to_string(index),
                                static_cast<float>(currency.coins[index].value));
            globalVariables.setString("cash_coin_model_" + std::to_string(index),
                                      currency.coins[index].modelPath);
        }
        for (const ConfigurationDiagnostic& diagnostic : currency.diagnostics.entries) {
            gameLog.Log(diagnostic.severity == ConfigurationDiagnostic::Severity::Error
                            ? LogLevel::Error
                            : LogLevel::Warning,
                        "Map currency diagnostic line=" + std::to_string(diagnostic.line) + " [" +
                            diagnostic.keyword + "] " + diagnostic.message);
        }
    }
    trafficSplineDebugBuilt_ = false;
    trafficSplineDebugMapRevision_ = 0;
    trafficSplineDebugTiles_.clear();
    ++trafficSplineDebugRevision_;
}

void RenderLoop::updateMapStreaming(double worldX, double worldY) {
    if (mapRenderer_ != nullptr) {
        mapRenderer_->updateStreaming(worldX, worldY);
    }
}

bool RenderLoop::initialMapWindowReady() const noexcept {
    return mapRenderer_ != nullptr && mapRenderer_->initialWindowReady();
}

void RenderLoop::updateMapPoleStates(const std::vector<MapPoleState>& states) {
    if (mapRenderer_ != nullptr) {
        mapRenderer_->updatePoleStates(states);
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

bool RenderLoop::consumeRaisePlayerRequest() {
    const bool requested = raisePlayerRequestPending_;
    raisePlayerRequestPending_ = false;
    return requested;
}
