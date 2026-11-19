// Window lifecycle, vehicle/map registration and simple accessors.

#include "app/RenderLoopInternal.h"
#include "app/MapTrafficWorker.h"
#include "app/PassengerCoinGeometry.h"
#include "app/vehicle/VehicleSpotlight.h"
#include "app/vehicle/VehicleDefinition.h"
#include "app/vehicle/SpotlightShadowScheduling.h"
#include "app/traffic/PassengerMotionKeys.h"
#include "config/HumanConfigLoader.h"
#include "core/AssetPath.h"
#include "render/map/MapRenderer.h"
#include "map/MapSplineProfile.h"
#if defined(OPENBUS_DILIGENT_RENDERER_AVAILABLE)
#include "app/DiligentCameraBridge.h"
#include "app/DiligentFrameRenderer.h"
#include "app/DiligentHumanBridge.h"
#include "app/DiligentMapBridge.h"
#include "app/DiligentWindowBridge.h"
#include "app/vehicle/DiligentVehicleBridge.h"
#include "app/vehicle/DiligentVehicleSceneBridge.h"
#include "render/diligent/DiligentMapTileAsset.h"
#endif

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
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

double graphicsFpsCap(double defaultValue) {
    const char* value = openbus::getEnvironment("OPENBUS_FPS_CAP");
    if (value == nullptr)
        return defaultValue;
    try {
        return openbus::core::parseGraphicsFpsCapOverride(value);
    } catch (const std::exception&) {
        gameLog.Warning("Invalid OPENBUS_FPS_CAP; ignoring the setting");
        return defaultValue;
    }
}

int positiveEnvironmentFrameInterval(const char* name, int defaultValue) {
    const char* value = openbus::getEnvironment(name);
    if (value == nullptr) {
        return defaultValue;
    }
    try {
        std::size_t parsed = 0;
        const long interval = std::stol(value, &parsed);
        if (value[parsed] != '\0' || interval < 1 || interval > 32) {
            throw std::invalid_argument("Expected an integer from 1 to 32");
        }
        return static_cast<int>(interval);
    } catch (const std::exception&) {
        gameLog.Warning(std::string("Invalid ") + name +
                        "; using the default AI spotlight interval");
        return defaultValue;
    }
}

double minimumObjectViewportFraction(const char* name, double defaultPercent) {
    const char* value = openbus::getEnvironment(name);
    if (value == nullptr || *value == '\0') {
        return defaultPercent / 100.0;
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
      assetRequestManager_(std::make_unique<AssetRequestManager>()) {
    TraceScope trace("startup", "RenderLoop::RenderLoop");
    try {
        graphicsPreferencesPath_ = openbus::core::graphicsPreferencesPath();
        graphicsPreferences_ = openbus::core::loadGraphicsPreferences(graphicsPreferencesPath_);
        graphicsPreferencesBaseline_ = graphicsPreferences_.values;
        activeGraphicsPreferences_ = graphicsPreferences_.values;
        graphicsPreferencesLoaded_ = true;
    } catch (const std::exception& error) {
        graphicsPreferencesError_ = error.what();
        gameLog.Warning("Graphics preferences were not loaded: " + graphicsPreferencesError_);
        activeGraphicsPreferences_ = graphicsPreferences_.values;
    }
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
    aiSpotlightScheduleConfig_.nearDistanceMeters =
        positiveEnvironmentRate("OPENBUS_AI_SPOTLIGHT_NEAR_METERS", 60.0);
    aiSpotlightScheduleConfig_.mediumDistanceMeters =
        positiveEnvironmentRate("OPENBUS_AI_SPOTLIGHT_MEDIUM_METERS", 180.0);
    aiSpotlightScheduleConfig_.nearIntervalFrames =
        positiveEnvironmentFrameInterval("OPENBUS_AI_SPOTLIGHT_NEAR_FRAMES", 2);
    aiSpotlightScheduleConfig_.mediumIntervalFrames =
        positiveEnvironmentFrameInterval("OPENBUS_AI_SPOTLIGHT_MEDIUM_FRAMES", 6);
    aiSpotlightScheduleConfig_.farIntervalFrames =
        positiveEnvironmentFrameInterval("OPENBUS_AI_SPOTLIGHT_FAR_FRAMES", 20);
    aiSpotlightScheduleConfig_.approachingSpeedMetersPerSecond =
        positiveEnvironmentRate("OPENBUS_AI_SPOTLIGHT_APPROACH_MPS", 5.0);
    aiSpotlightScheduleConfig_.highSpeedMetersPerSecond =
        positiveEnvironmentRate("OPENBUS_AI_SPOTLIGHT_HIGH_SPEED_MPS", 18.0);
    if (aiSpotlightScheduleConfig_.nearDistanceMeters >=
        aiSpotlightScheduleConfig_.mediumDistanceMeters) {
        gameLog.Warning("AI spotlight near distance must be less than medium distance; "
                        "using 60 m and 180 m defaults");
        aiSpotlightScheduleConfig_.nearDistanceMeters = 60.0;
        aiSpotlightScheduleConfig_.mediumDistanceMeters = 180.0;
    }
    if (aiSpotlightScheduleConfig_.nearIntervalFrames >
            aiSpotlightScheduleConfig_.mediumIntervalFrames ||
        aiSpotlightScheduleConfig_.mediumIntervalFrames >
            aiSpotlightScheduleConfig_.farIntervalFrames) {
        gameLog.Warning("AI spotlight intervals must increase with distance; "
                        "using 2/6/20 frame defaults");
        aiSpotlightScheduleConfig_.nearIntervalFrames = 2;
        aiSpotlightScheduleConfig_.mediumIntervalFrames = 6;
        aiSpotlightScheduleConfig_.farIntervalFrames = 20;
    }
    gameLog.Log("AI spotlight scheduling intervals are " +
                std::to_string(aiSpotlightScheduleConfig_.nearIntervalFrames) + "/" +
                std::to_string(aiSpotlightScheduleConfig_.mediumIntervalFrames) + "/" +
                std::to_string(aiSpotlightScheduleConfig_.farIntervalFrames) +
                " frames at <= " + std::to_string(aiSpotlightScheduleConfig_.nearDistanceMeters) +
                " m, <= " + std::to_string(aiSpotlightScheduleConfig_.mediumDistanceMeters) +
                " m and farther (approaching/high-speed lights update twice as often).");
    const double renderFpsCapHz = graphicsFpsCap(graphicsPreferences_.values.fpsCap);
    frameTiming_.setFrameRateCap(renderFpsCapHz);
    mainViewMinimumObjectViewportFraction_ =
        minimumObjectViewportFraction("OPENBUS_MIN_OBJECT_SIZE_MAIN_VIEW_PERCENT",
                                      graphicsPreferences_.values.minimumObjectSizeMainPercent);
    reflectionMinimumObjectViewportFraction_ =
        minimumObjectViewportFraction("OPENBUS_MIN_OBJECT_SIZE_MIRRORS_PERCENT",
                                      graphicsPreferences_.values.minimumObjectSizeMirrorsPercent);
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
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    const bool benchmarkMode = parseEnabledFlag(openbus::getEnvironment("OPENBUS_BENCHMARK"));
    int windowWidth = benchmarkMode ? width : graphicsPreferences_.values.width;
    int windowHeight = benchmarkMode ? height : graphicsPreferences_.values.height;
    GLFWmonitor* windowMonitor = nullptr;
    GLFWmonitor* selectedMonitor = nullptr;
    bool centerWindowOnMonitor = false;
    int refreshRate = GLFW_DONT_CARE;
    const auto windowMode = graphicsPreferences_.values.windowMode;
    if (!benchmarkMode) {
        int monitorCount = 0;
        GLFWmonitor** monitors = glfwGetMonitors(&monitorCount);
        const int monitorIndex = graphicsPreferences_.values.monitor;
        if (monitors == nullptr || monitorCount == 0) {
            gameLog.Warning("No display monitor is available; starting windowed.");
        } else {
            selectedMonitor = monitors[monitorIndex < monitorCount ? std::max(monitorIndex, 0) : 0];
            if (monitorIndex >= monitorCount) {
                gameLog.Warning("Configured monitor is unavailable; using the primary display.");
            }
            if (windowMode == "exclusive") {
                const GLFWvidmode* requestedMode = nullptr;
                int modeCount = 0;
                const GLFWvidmode* modes = glfwGetVideoModes(selectedMonitor, &modeCount);
                for (int index = 0; modes != nullptr && index < modeCount; ++index) {
                    if (modes[index].width == windowWidth && modes[index].height == windowHeight &&
                        modes[index].refreshRate == graphicsPreferences_.values.refreshRate) {
                        requestedMode = &modes[index];
                        break;
                    }
                }
                if (requestedMode == nullptr) {
                    requestedMode = glfwGetVideoMode(selectedMonitor);
                    if (requestedMode != nullptr) {
                        windowWidth = requestedMode->width;
                        windowHeight = requestedMode->height;
                        gameLog.Warning("Configured exclusive display mode is unavailable; using "
                                        "the monitor's current mode.");
                    }
                }
                if (requestedMode != nullptr) {
                    windowMonitor = selectedMonitor;
                    refreshRate = requestedMode->refreshRate;
                }
            } else if (windowMode == "borderless") {
                const GLFWvidmode* desktopMode = glfwGetVideoMode(selectedMonitor);
                if (desktopMode != nullptr) {
                    windowWidth = desktopMode->width;
                    windowHeight = desktopMode->height;
                    windowMonitor = nullptr;
                    glfwWindowHint(GLFW_REFRESH_RATE, GLFW_DONT_CARE);
                }
            } else {
                centerWindowOnMonitor = glfwGetVideoMode(selectedMonitor) != nullptr;
            }
        }
    }
    if (benchmarkMode) {
        glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);
    } else {
        glfwWindowHint(GLFW_DECORATED, windowMode == "borderless" ? GLFW_FALSE : GLFW_TRUE);
    }
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
    glfwWindowHint(GLFW_REFRESH_RATE, refreshRate);
    window_ = glfwCreateWindow(windowWidth, windowHeight, title, windowMonitor, nullptr);
    glfwWindowHint(GLFW_REFRESH_RATE, GLFW_DONT_CARE);
    if (!window_) {
        gameLog.Error("Failed to create Diligent no-API window");
        glfwTerminate();
        throw std::runtime_error("Failed to create Diligent no-API window");
    }
    if (benchmarkMode) {
        gameLog.Log(
            std::string("Benchmark window is ") +
            (glfwGetWindowAttrib(window_, GLFW_VISIBLE) == GLFW_TRUE ? "visible" : "hidden"));
    }
    if (!benchmarkMode && selectedMonitor != nullptr &&
        (windowMode == "borderless" || centerWindowOnMonitor)) {
        int monitorX = 0;
        int monitorY = 0;
        glfwGetMonitorPos(selectedMonitor, &monitorX, &monitorY);
        if (centerWindowOnMonitor) {
            const GLFWvidmode* desktopMode = glfwGetVideoMode(selectedMonitor);
            if (desktopMode != nullptr) {
                monitorX += (desktopMode->width - windowWidth) / 2;
                monitorY += (desktopMode->height - windowHeight) / 2;
            }
        }
        glfwSetWindowPos(window_, monitorX, monitorY);
    }
    glfwSetWindowUserPointer(window_, this);
    glfwSetScrollCallback(window_, &RenderLoop::scrollCallback);
#if defined(OPENBUS_DILIGENT_RENDERER_AVAILABLE)
    try {
        initializeDiligentBackend();
    } catch (...) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
        glfwTerminate();
        throw;
    }
#else
    glfwDestroyWindow(window_);
    window_ = nullptr;
    glfwTerminate();
    throw std::runtime_error("OpenBus requires the Diligent Vulkan renderer");
#endif
    clickableCursor_ = glfwCreateStandardCursor(GLFW_HAND_CURSOR);
    mouseSteeringCursor_ = glfwCreateStandardCursor(GLFW_CROSSHAIR_CURSOR);
    if (mouseLookEnabled_) {
        glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        glfwGetCursorPos(window_, &previousCursorX_, &previousCursorY_);
    }
    gameLog.Log("RenderLoop initialized");
}

#if defined(OPENBUS_DILIGENT_RENDERER_AVAILABLE)
void RenderLoop::initializeDiligentBackend() {
    if (window_ == nullptr) {
        throw std::logic_error("Diligent initialization requires a valid GLFW window");
    }
    diligentDebugView_ =
        openbus::app::diligentDebugViewForPreference(graphicsPreferences_.values.temporalDebugView);
    framePresenter_ =
        openbus::app::createDiligentFrameRenderer(*window_, graphicsPreferences_.values);
    glfwGetFramebufferSize(window_, &framebufferWidth_, &framebufferHeight_);
    const char* vsyncSetting = openbus::getEnvironment("OPENBUS_VSYNC");
    vsyncEnabled_ = vsyncSetting == nullptr
                        ? graphicsPreferences_.values.vsync
                        : (std::string(vsyncSetting) != "0" && std::string(vsyncSetting) != "off" &&
                           std::string(vsyncSetting) != "false");
    framePresenter_->setVerticalSync(vsyncEnabled_);
    auto* diligentRenderer =
        dynamic_cast<openbus::rendering::DiligentFrameRenderer*>(framePresenter_.get());
    if (diligentRenderer == nullptr) {
        throw std::logic_error("Diligent initialization did not create a Diligent frame renderer");
    }
    const auto maximumAnisotropy = diligentRenderer->maximumAnisotropy();
    textureAnisotropySupported_ = maximumAnisotropy > 1;
    maximumTextureAnisotropy_ = static_cast<float>(maximumAnisotropy);
    appliedTextureAnisotropy_ = static_cast<float>(
        std::min(static_cast<Diligent::Uint32>(graphicsPreferences_.values.anisotropicFiltering),
                 maximumAnisotropy));
    if (graphicsPreferences_.values.anisotropicFiltering > 1 && !textureAnisotropySupported_) {
        gameLog.Warning("Anisotropic filtering was requested but is not supported by this "
                        "Diligent device.");
    } else {
        gameLog.Log("Diligent material texture anisotropic filtering: " +
                    std::to_string(appliedTextureAnisotropy_) + "x (device maximum " +
                    std::to_string(maximumTextureAnisotropy_) + "x)");
    }
    gameLog.Log("Diligent Vulkan renderer initialized; VSync " +
                std::string(vsyncEnabled_ ? "enabled" : "disabled"));
}
#endif

void RenderLoop::shutdownRendererBackend() {
    framePresenter_.reset();
}

RenderLoop::~RenderLoop() {
    gameLog.Log("RenderLoop shutting down");
    persistPlayerOdometer(true);
    if (framePresenter_ != nullptr) {
        try {
            framePresenter_->waitForIdle();
        } catch (const std::exception& error) {
            gameLog.Error("Renderer shutdown wait failed: " + std::string(error.what()));
        } catch (...) {
            gameLog.Error("Renderer shutdown wait failed with an unknown error");
        }
    }
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
    mapRenderer_.reset();
    playerVehicle_ = nullptr;
    assetRequestManager_->join();
    vehicles_.clear();
    Vehicle::clearSharedDisplayCache();
    shutdownRendererBackend();
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
                                bool isAiVehicle, VehicleSetupOptions setupOptions,
                                bool configurePlayerView) {
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
    if (!isAiVehicle && configurePlayerView) {
        vehicleCameras_.clear();
        trafficReflectionViewsByIndex_.clear();
        trafficReflectionCameraViews_.clear();
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
    }
    const double modelOffsetZ = -vehicleConfiguration.centerOfGravityHeight;
    std::unique_ptr<Vehicle> model;
    {
        TraceScope vehicleTrace("startup", "RenderLoop::AddVehicle.constructVehicle");
        model = std::make_unique<Vehicle>(
            definition, placementForVehicle(vehicleConfiguration, placement, isAiVehicle),
            modelOffsetZ, loadingPolicy, *assetRequestManager_, simulationState_, soundEngine_,
            soundViewpoint_, isAiVehicle, false, std::move(setupOptions),
            configKind == VehicleFileKind::Bus, activeGraphicsPreferences_.vehicleTextureScale,
            activeGraphicsPreferences_.vehicleFrustumCulling, false);
    }
    Vehicle* result = model.get();
    vehicles_.push_back(std::move(model));
    return result;
}

void RenderLoop::RemoveCoupledVehicle(Vehicle* vehicle) {
    if (vehicle == nullptr) {
        return;
    }
    if (vehicle == playerVehicle_ || vehicle == mapBusVehicle_ || vehicle->followsMapTrafficRoute) {
        throw std::invalid_argument(
            "Coupled-section removal cannot remove a player or map-traffic vehicle");
    }
    const auto found = std::find_if(vehicles_.begin(), vehicles_.end(),
                                    [vehicle](const std::unique_ptr<Vehicle>& candidate) {
                                        return candidate.get() == vehicle;
                                    });
    if (found == vehicles_.end()) {
        throw std::invalid_argument("Coupled-section vehicle is not owned by the render loop");
    }
    vehicles_.erase(found);
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
    if (isBus && configuration.rearCouplingReference) {
        try {
            const BusConfiguration frontBusConfiguration =
                loadBusConfiguration(resolvedConfig, modelConfig);
            const std::filesystem::path rearConfigPath =
                busConfigurationPathFor(configuration.rearCouplingReference->vehicleConfigPath);
            const std::filesystem::path rearModelConfigPath =
                modelConfigurationPathForBusFromConfig(rearConfigPath);
            const auto rearDefinition =
                sharedVehicleDefinition(rearConfigPath, rearModelConfigPath, VehicleFileKind::Bus);
            const BusConfiguration rearBusConfiguration =
                loadBusConfiguration(rearConfigPath, rearModelConfigPath);
            const VehicleConfig& rearVehicleConfiguration = rearDefinition->vehicleConfiguration;
            if (!frontBusConfiguration.rearCouplingPoint.has_value() ||
                !rearBusConfiguration.frontCouplingPoint.has_value() ||
                !rearBusConfiguration.frontCouplingCharacter.has_value() ||
                rearBusConfiguration.frontCouplingCharacter->type != 1 ||
                !rearVehicleConfiguration.hasBoundingBox ||
                rearVehicleConfiguration.boundingBox[0] <= 0.0 ||
                rearVehicleConfiguration.boundingBox[1] <= 0.0) {
                gameLog.Warning(
                    "Scheduled map AI bus rear section was not paired: type-1 coupling anchors "
                    "or a valid rear-section footprint are unavailable for " +
                    busConfigPath.string());
            } else {
                trafficState.frontRearCouplingPoint = *frontBusConfiguration.rearCouplingPoint;
                trafficState.coupledFrontCouplingPoint = *rearBusConfiguration.frontCouplingPoint;
                trafficState.actor.articulatedRear = openbus::map::MapTrafficActor::ArticulatedRear{
                    *frontBusConfiguration.rearCouplingPoint,
                    *rearBusConfiguration.frontCouplingPoint,
                    rearVehicleConfiguration.boundingBox[1] +
                        2.0 * std::abs(rearVehicleConfiguration.boundingBox[4]),
                    rearVehicleConfiguration.boundingBox[0] +
                        2.0 * std::abs(rearVehicleConfiguration.boundingBox[3])};
                trafficState.workerActorTemplate =
                    std::make_shared<const openbus::map::MapTrafficActor>(trafficState.actor);
                const VehiclePlacement rearPlacement = articulatedRearPlacement(
                    frontBusConfiguration, configuredPlacement, rearBusConfiguration);
                VehicleSetupOptions rearSetupOptions;
                rearSetupOptions.vehicleConfigurationPath = rearConfigPath;
                const double rearModelOffsetZ =
                    -rearDefinition->vehicleConfiguration.centerOfGravityHeight;
                trafficState.pendingCoupledVehicle = assetRequests->submit(
                    [rearDefinition, rearPlacement, rearModelOffsetZ, loadingPolicy, assetRequests,
                     simulationState, soundEngine, soundViewpoint,
                     vehicleTextureScale = activeGraphicsPreferences_.vehicleTextureScale,
                     vehicleFrustumCulling = activeGraphicsPreferences_.vehicleFrustumCulling,
                     rearSetupOptions = std::move(rearSetupOptions)]() mutable {
                        return std::make_unique<Vehicle>(
                            rearDefinition, rearPlacement, rearModelOffsetZ, loadingPolicy,
                            *assetRequests, *simulationState, *soundEngine, *soundViewpoint, true,
                            true, std::move(rearSetupOptions), true, vehicleTextureScale,
                            vehicleFrustumCulling, false);
                    });
                trafficState.hasCoupledVehicle = true;
            }
        } catch (const std::exception& error) {
            gameLog.Warning("Scheduled map AI bus rear section could not be prepared for '" +
                            busConfigPath.string() + "': " + error.what());
        }
    }
    std::future<std::unique_ptr<Vehicle>> pendingVehicle = assetRequests->submit(
        [definition, configuredPlacement, modelOffsetZ, loadingPolicy, assetRequests,
         simulationState, soundEngine, soundViewpoint, isBus,
         vehicleTextureScale = activeGraphicsPreferences_.vehicleTextureScale,
         vehicleFrustumCulling = activeGraphicsPreferences_.vehicleFrustumCulling] {
            return std::make_unique<Vehicle>(
                definition, configuredPlacement, modelOffsetZ, loadingPolicy, *assetRequests,
                *simulationState, *soundEngine, *soundViewpoint, true, true, VehicleSetupOptions{},
                isBus, vehicleTextureScale, vehicleFrustumCulling, false);
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
    simulationState_.sharedVariables().configureSimulationClock(current.year, current.month,
                                                                current.day, current.timeSeconds);
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
        openbus::app::rebaseScheduledBusClock(*state.bus, newStartHour, previousElapsedSeconds);
    }
    trafficFleet_.mapTrafficStartHour = newStartHour;
    trafficFleet_.mapTrafficStartSimulationTime = newSimulationTime;
    if (playerTimetable_.state &&
        playerTimetable_.state->selection.courseIndex < passengerTimetableCourses_.size()) {
        const auto& trips =
            passengerTimetableCourses_[playerTimetable_.state->selection.courseIndex].trips;
        const std::size_t tripIndex = playerTimetable_.state->selection.courseTripIndex;
        if (tripIndex < trips.size()) {
            applyPlayerTripAiPolicy(trips[tripIndex],
                                    playerTimetable_.state->selection.driveAsAdditionalBus);
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
    actor->lateralOffsetMeters =
        openbus::map::mapTrafficPedestrianLateralOffset(actor->route, humanTrafficActors_.size());
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

void RenderLoop::SetWeather(const openbus::WeatherState& weather) {
    simulationState_.sharedVariables().setWeather(weather);
}

void RenderLoop::EnableMetarRefresh(const std::string& station,
                                    const openbus::WeatherState& initialWeather) {
    weatherSelection_.enableMetarRefresh(station, initialWeather);
}

void RenderLoop::SetAutoClutch(bool enabled) {
    simulationState_.sharedVariables().setAutoClutch(enabled);
}

void RenderLoop::SetSunAltitude(double altitudeDegrees) {
    simulationState_.sharedVariables().setSunAltitude(altitudeDegrees);
}

void RenderLoop::ConfigureDayNight(double latitudeDegrees) {
    simulationState_.sharedVariables().configureDayNight(latitudeDegrees);
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
    trafficReflectionViewsByIndex_.clear();
    trafficReflectionCameraViews_.clear();
    vehicleScheduleViews_.clear();
    vehicleTicketSellingViews_.clear();
    cameraView_ = 0;
    pauseMenuOpen_ = false;
    setPausedState(false);
}

void RenderLoop::ResetForStartupMenu() {
    invalidateTemporalHistory();
    weatherSelection_.discardPending();
    weatherUiMessage_.clear();
    weatherUiError_ = false;
    if (playerTimetable_.state) {
        SetPlayerTimetableSelection(std::nullopt);
    }
    clearPlayerTripAiPolicy();
    trafficFleet_.worker.reset();
    trafficFleet_.vehicles.clear();
    trafficFleet_ = {};
    controlCablePairingWarningsLogged_.clear();
    SetPlayerVehicle(nullptr);
    vehicles_.clear();
    humanTrafficActors_.clear();
    alightedPassengers_.clear();
    scheduledPassengerHumanModels_.clear();
    scheduledPassengerHumanAges_.clear();
    scheduledPassengerHumanMassesKg_.clear();
    passengerTicketPack_ = {};
    passengerCurrency_ = {};
    passengerCoinAnimations_.clear();
    observedPassengerCoinCounts_.clear();
    diligentHumanRenderSources_.clear();
    diligentHumanBoundingRadii_.clear();
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
    passengerQueueApproachProgress_.clear();
    passengerPedestrianQueuePositions_.clear();
    passengerPedestrianQueuePositionsPrepared_ = false;
    passengerHeldQueuePositions_.clear();
    passengerRenderedQueuePositions_.clear();
    reportedPassengerTicketStops_.clear();
    playerPassengerStopLatch_.reset();
    playerPassengerStopLatchRouteIndex_.reset();
    playerPassengerStopLatchIndex_.reset();
    activePlayerPassengerStop_.reset();
    activePlayerPassengerStopRouteIndex_.reset();
    activePlayerPassengerStopIndex_.reset();
    playerPassengerServiceRouteIndex_.reset();
    playerPassengerServiceNextStopIndex_ = 0;
    playerTimetableStopVisitLatch_.reset();
    ++playerPassengerServiceGeneration_;
    mapRenderer_.reset();
    diligentHumanRenderSources_.clear();
    diligentHumanBoundingRadii_.clear();
    vehicleCameras_.clear();
    trafficReflectionViewsByIndex_.clear();
    trafficReflectionCameraViews_.clear();
    vehicleScheduleViews_.clear();
    vehicleTicketSellingViews_.clear();
    mouseControlEnabled_ = false;
    mouseLookEnabled_ = false;
    freeCameraEnabled_ = false;
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
    passengerCoinAnimations_.clear();
    observedPassengerCoinCounts_.clear();
    passengerApproachProgress_.clear();
    passengerQueueApproachProgress_.clear();
    passengerPedestrianQueuePositions_.clear();
    passengerPedestrianQueuePositionsPrepared_ = false;
    passengerHeldQueuePositions_.clear();
    passengerRenderedQueuePositions_.clear();
    reportedPassengerTicketStops_.clear();
    if (spawnEntryPointIndex >= map.entryPoints.size()) {
        throw std::out_of_range("Selected map spawn entrypoint is outside [entrypoints]");
    }
    weatherPresetRoot_ = omsiRoot;
    weatherPresets_.clear();
    weatherPresetsLoaded_ = false;
    weatherPresetError_.clear();
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
            omsiRoot, simulationState_, *assetRequestManager_, prepareInitialWindowAsync, false);
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
#if defined(OPENBUS_DILIGENT_RENDERER_AVAILABLE)
        if (auto* diligentRenderer =
                dynamic_cast<openbus::rendering::DiligentFrameRenderer*>(framePresenter_.get())) {
            mapRenderer_->updateStreaming(worldX, worldY);
            static_cast<void>(openbus::app::updateDiligentMapTiles(*diligentRenderer, *mapRenderer_,
                                                                   worldX, worldY));
            return;
        }
#endif
        mapRenderer_->updateStreaming(worldX, worldY);
    }
}

void RenderLoop::prepareDiligentSceneAssets(const BusSimulation& simulation) {
#if defined(OPENBUS_DILIGENT_RENDERER_AVAILABLE)
    auto* diligentRenderer =
        dynamic_cast<openbus::rendering::DiligentFrameRenderer*>(framePresenter_.get());
    if (diligentRenderer == nullptr) {
        return;
    }
    static_cast<void>(diligentRenderer->configureSpotlightShadowMaps(256, 4));

    const BodyPose chassis = simulation.chassisPose();
    const auto view = buildCameraViewMatrix(simulation, chassis);
    Matrix4 inverseView = {};
    if (!openbus::rendering::invertAffineMatrix(view, inverseView)) {
        throw std::runtime_error("Diligent vehicle preparation requires an invertible camera view");
    }
    const std::array<double, 3> eye = {inverseView[12], inverseView[13], inverseView[14]};
    const RenderViewContext playerContext = renderingReflection_ || isExteriorView()
                                                ? RenderViewContext::PlayerExterior
                                                : RenderViewContext::PlayerInterior;
    std::vector<std::pair<double, Vehicle*>> loadOrder;
    loadOrder.reserve(vehicles_.size());
    for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
        if (vehicle->mapTrafficRouteFinished ||
            (vehicle->followsMapTrafficRoute && !vehicle->mapTrafficTileActive)) {
            continue;
        }
        const std::array<double, 3>& position =
            vehicle.get() == playerVehicle_
                ? chassis.position
                : vehicle->trafficRenderPlacement.value_or(vehicle->placement).position;
        const double dx = position[0] - eye[0];
        const double dy = position[1] - eye[1];
        const double dz = position[2] - eye[2];
        loadOrder.emplace_back(dx * dx + dy * dy + dz * dz, vehicle.get());
    }
    std::sort(loadOrder.begin(), loadOrder.end(),
              [](const auto& left, const auto& right) { return left.first < right.first; });
    const auto modelLoadDeadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(4);
    for (const auto& [distanceSquared, vehicle] : loadOrder) {
        const RenderViewContext context =
            vehicle == playerVehicle_ ? playerContext : RenderViewContext::NonPlayer;
        vehicle->ensureLoaded(context, std::sqrt(distanceSquared), modelLoadDeadline);
    }

    std::vector<Vehicle*> liveVehicles;
    liveVehicles.reserve(vehicles_.size());
    for (const auto& [distanceSquared, vehicle] : loadOrder) {
        static_cast<void>(distanceSquared);
        liveVehicles.push_back(vehicle);
    }
    static_cast<void>(
        openbus::app::vehicle::prepareDiligentVehicleAssets(liveVehicles, *diligentRenderer));

    if (mapRenderer_ == nullptr) {
        static_cast<void>(diligentRenderer->retainHumanAssets({}));
        return;
    }
    std::vector<std::string> liveHumanModelKeys;
    std::unordered_set<std::string> uniqueHumanModelKeys;
    const std::size_t maximumModelCount =
        scheduledPassengerHumanModels_.size() + humanTrafficActors_.size();
    liveHumanModelKeys.reserve(maximumModelCount);
    uniqueHumanModelKeys.reserve(maximumModelCount);
    const auto prepareHumanModel = [&](const std::filesystem::path& path) {
        const std::string key = openbus::pathToUtf8(path.lexically_normal());
        if (key.empty() || !uniqueHumanModelKeys.insert(key).second) {
            return;
        }
        liveHumanModelKeys.push_back(key);
        auto source = diligentHumanRenderSources_.find(key);
        if (source == diligentHumanRenderSources_.end()) {
            std::optional<openbus::rendering::HumanTrafficRenderSource> loadedSource =
                mapRenderer_->humanTrafficRenderSource(path);
            if (!loadedSource.has_value()) {
                return;
            }
            double radiusSquared = 0.0;
            for (const auto& [materialIndex, vertices] :
                 loadedSource->geometry.verticesByMaterial) {
                static_cast<void>(materialIndex);
                for (const auto& vertex : vertices) {
                    const double x = vertex.x;
                    const double y = vertex.y;
                    const double z = vertex.z;
                    radiusSquared = (std::max)(radiusSquared, x * x + y * y + z * z);
                }
            }
            diligentHumanBoundingRadii_.insert_or_assign(key, std::sqrt(radiusSquared));
            source =
                diligentHumanRenderSources_
                    .emplace(key,
                             std::make_shared<const openbus::rendering::HumanTrafficRenderSource>(
                                 std::move(*loadedSource)))
                    .first;
        }
        static_cast<void>(
            openbus::rendering::prepareDiligentHumanAsset(*diligentRenderer, key, *source->second));
    };
    for (const std::filesystem::path& path : scheduledPassengerHumanModels_) {
        prepareHumanModel(path);
    }
    for (const HumanTrafficState& human : humanTrafficActors_) {
        prepareHumanModel(std::filesystem::path(human.actor.modelPath));
    }
    static_cast<void>(diligentRenderer->retainHumanAssets(liveHumanModelKeys));
#else
    static_cast<void>(simulation);
#endif
}

void RenderLoop::drawDiligentScene(const BusSimulation& simulation, const World& world) {
#if defined(OPENBUS_DILIGENT_RENDERER_AVAILABLE)
    auto* diligentRenderer =
        dynamic_cast<openbus::rendering::DiligentFrameRenderer*>(framePresenter_.get());
    if (diligentRenderer == nullptr) {
        throw std::logic_error("Diligent scene submission requires the Diligent frame presenter");
    }
    if (diligentRenderer->isSuspended()) {
        diligentMotionHistory_.invalidate();
        diligentRenderer->invalidateHistory();
        diligentRenderer->cancelUiFrame();
        return;
    }
    if (framebufferWidth_ <= 0 || framebufferHeight_ <= 0) {
        throw std::logic_error("Diligent scene submission requires a nonzero framebuffer");
    }
    if (diligentFrameId_ == std::numeric_limits<std::uint64_t>::max()) {
        throw std::overflow_error("Diligent frame identifier space is exhausted");
    }

    const BodyPose chassis = simulation.chassisPose();
    const auto view = buildCameraViewMatrix(simulation, chassis);
    const auto projection = openbus::rendering::makePerspectiveMatrix(
        static_cast<double>(framebufferWidth_), static_cast<double>(framebufferHeight_),
        currentFieldOfView());
    const std::array<double, 3> cameraPosition = {
        -(view[0] * view[12] + view[1] * view[13] + view[2] * view[14]),
        -(view[4] * view[12] + view[5] * view[13] + view[6] * view[14]),
        -(view[8] * view[12] + view[9] * view[13] + view[10] * view[14])};
    const auto camera = openbus::app::makeDiligentCameraView(view, projection, cameraPosition,
                                                             framebufferWidth_, framebufferHeight_);
    const RenderViewContext playerContext = renderingReflection_ || isExteriorView()
                                                ? RenderViewContext::PlayerExterior
                                                : RenderViewContext::PlayerInterior;
    const auto vehicleWorldTransform = [&](const Vehicle& vehicle) {
        const Matrix4 localModelOffset =
            openbus::rendering::translationMatrix({0.0, 0.0, vehicle.modelOffsetZ});
        if (&vehicle == playerVehicle_) {
            return openbus::rendering::multiplyMatrix4(openbus::rendering::poseMatrix(chassis),
                                                       localModelOffset);
        }
        if (vehicle.odeSimulation != nullptr) {
            return openbus::rendering::multiplyMatrix4(
                openbus::rendering::poseMatrix(vehicle.odeSimulation->chassisPose()),
                localModelOffset);
        }

        VehiclePlacement placement = vehicle.trafficRenderPlacement.value_or(vehicle.placement);
        if (mapRenderer_ != nullptr && !vehicle.followsMapTrafficRoute) {
            placement.position[2] =
                mapRenderer_->terrainHeightAt(placement.position[0], placement.position[1])
                    .value_or(placement.position[2]);
        }
        const VehiclePlacement modelRoot =
            openbus::rendering::makeModelRootPlacement(placement, vehicle.modelOffsetZ);
        Matrix4 worldTransform = openbus::rendering::translationMatrix(modelRoot.position);
        worldTransform = openbus::rendering::multiplyMatrix4(
            worldTransform,
            openbus::rendering::rotationMatrix(placement.yawDegrees, 0.0, 0.0, 1.0));
        worldTransform = openbus::rendering::multiplyMatrix4(
            worldTransform,
            openbus::rendering::rotationMatrix(-placement.pitchDegrees, 0.0, 1.0, 0.0));
        worldTransform = openbus::rendering::multiplyMatrix4(
            worldTransform,
            openbus::rendering::rotationMatrix(placement.rollDegrees, 1.0, 0.0, 0.0));
        return openbus::rendering::multiplyMatrix4(worldTransform, localModelOffset);
    };
    std::optional<openbus::rendering::ViewFrustum> vehicleVisibilityFrustum =
        openbus::rendering::buildViewFrustum(projection);
    for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
        if (vehicle->mapTrafficRouteFinished ||
            (vehicle->followsMapTrafficRoute && !vehicle->mapTrafficTileActive) ||
            !vehicle->loaded || vehicle->displayLists.empty()) {
            continue;
        }
        const RenderViewContext context =
            vehicle.get() == playerVehicle_ ? playerContext : RenderViewContext::NonPlayer;
        const Matrix4 modelView =
            openbus::rendering::multiplyMatrix4(view, vehicleWorldTransform(*vehicle));
        const Matrix4 visibilityModelView = openbus::rendering::multiplyMatrix4(
            modelView, openbus::rendering::translationMatrix({0.0, 0.0, -vehicle->modelOffsetZ}));
        vehicle->prepareReflectionVisibility(
            context, framebufferWidth_, framebufferHeight_, mainViewMinimumObjectViewportFraction_,
            visibilityModelView, projection, &vehicleVisibilityFrustum);
    }

    std::vector<Diligent::Uint32> reflectionTargetSizes;
    reflectionTargetSizes.reserve(vehicleCameras_.size());
    const auto reflectionTargetSize = static_cast<Diligent::Uint32>(std::max(
        graphicsPreferences_.values.reflectionSize, openbus::rendering::kMinReflectionTargetSize));
    for (const VehicleCamera& cameraDefinition : vehicleCameras_) {
        if (cameraDefinition.kind != VehicleCameraKind::Reflexion &&
            cameraDefinition.kind != VehicleCameraKind::Reflexion2) {
            continue;
        }
        reflectionTargetSizes.push_back(reflectionTargetSize);
    }
    diligentRenderer->configureReflectionTargets(
        reflectionTargetSizes,
        static_cast<std::uint64_t>(graphicsPreferences_.values.reflectionInterval),
        graphicsPreferences_.values.reflectionMaxFps);

    const auto& sharedVariables = simulationState_.sharedVariables();
    const auto weatherBrightness = sharedVariables.environmentBrightness();
    const auto& roadWeather = sharedVariables.roadWeather();
    const openbus::rendering::DiligentMapWeather mapWeather{
        static_cast<float>(roadWeather.groundSnow), static_cast<float>(roadWeather.snow),
        static_cast<float>(roadWeather.water)};
    const double groundHeight =
        mapRenderer_ != nullptr
            ? mapRenderer_->terrainHeightAt(cameraPosition[0], cameraPosition[1]).value_or(0.0)
            : 0.0;
    const auto environment = openbus::rendering::buildDiligentSceneEnvironment(
        view, sharedVariables.sunDirection(), weatherBrightness, false,
        cameraPosition[2] - groundHeight, sharedVariables.effectiveWeatherVisibilityMeters(),
        sharedVariables.weatherFogTopAboveGroundMeters());
    openbus::app::DiligentMapSceneryUpdate sceneryUpdate;
    if (mapRenderer_ != nullptr) {
        sceneryUpdate = openbus::app::updateDiligentMapScenery(*diligentRenderer, *mapRenderer_);
    }
    int displayWidth = 0;
    int displayHeight = 0;
    glfwGetWindowSize(window_, &displayWidth, &displayHeight);
    const std::uint64_t frameId = ++diligentFrameId_;
    ++lightingFrameIndex_;
    if (lightingFrameIndex_ % 10 == 0) {
        spotlightShadowUpdateBudget_ = openbus::app::vehicle::adaptSpotlightShadowUpdateBudget(
            spotlightShadowUpdateBudget_, frameTiming_.averageRenderFps(),
            frameTiming_.frameRateCap());
    }
    std::vector<std::pair<std::pair<int, std::size_t>, openbus::map::MapTrafficPose>>
        diligentWaitingPassengerRenderedPoses;
    const bool rendered = diligentRenderer->renderFrame(
        diligentMotionHistory_, frameId, simulation.simulationTime(), camera, displayWidth,
        displayHeight, environment,
        [&](openbus::rendering::DiligentFrameRenderer& renderer) {
            static_cast<void>(renderer.drawWeatherClouds(
                sharedVariables.weatherCloudType(),
                sharedVariables.weatherCloudBaseAboveGroundMeters(),
                sharedVariables.weatherWindDirectionDegrees(),
                sharedVariables.weatherWindSpeedMetersPerSecond(), weatherBrightness,
                cameraPosition[2] - groundHeight, simulation.simulationTime()));
            if (mapRenderer_ != nullptr) {
                openbus::app::uploadDiligentMapLightsForEnvironment(
                    renderer, *mapRenderer_, view, cameraPosition, weatherBrightness,
                    static_cast<std::size_t>(graphicsPreferences_.values.sceneryLightLimit));
            }
            Diligent::ITextureView& fallbackTexture = renderer.fallbackTexture();
            std::vector<openbus::app::vehicle::DiligentVehicleSceneDrawState> vehicleDrawStates;
            vehicleDrawStates.reserve(vehicles_.size());
            std::vector<openbus::app::vehicle::SpotlightCandidate> vehicleSpotlightCandidates;
            vehicleSpotlightCandidates.reserve(vehicles_.size());
            const auto spotlightFrustum = openbus::rendering::buildViewFrustum(projection);
            struct DiligentSpotlightSource {
                openbus::rendering::VehicleSpotlight worldLight;
                double selection = -1.0;
                double distance = 0.0;
                bool player = false;
            };
            std::unordered_map<std::uint64_t, DiligentSpotlightSource> diligentSpotlightSources;
            diligentSpotlightSources.reserve(vehicles_.size());
            for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                if (vehicle->mapTrafficRouteFinished ||
                    (vehicle->followsMapTrafficRoute && !vehicle->mapTrafficTileActive) ||
                    !vehicle->loaded || vehicle->displayLists.empty()) {
                    continue;
                }
                const RenderViewContext context =
                    vehicle.get() == playerVehicle_ ? playerContext : RenderViewContext::NonPlayer;
                const Matrix4 modelView =
                    openbus::rendering::multiplyMatrix4(view, vehicleWorldTransform(*vehicle));
                if (!vehicle->spotlights.empty()) {
                    const double spotlightSelection = vehicle->variables.get("spot_select");
                    const auto spotlightStatus = openbus::app::vehicle::spotlightSelectionStatus(
                        vehicle->spotlights.size(), spotlightSelection);
                    if (spotlightStatus == openbus::app::vehicle::SpotlightStatus::Invalid &&
                        !vehicle->invalidSpotlightLogged) {
                        gameLog.Warning(
                            "Ignoring invalid vehicle spotlight selection or parameters");
                        vehicle->invalidSpotlightLogged = true;
                    } else if (spotlightStatus == openbus::app::vehicle::SpotlightStatus::Active) {
                        openbus::rendering::VehicleSpotlight spotlight;
                        openbus::rendering::VehicleSpotlight worldSpotlight;
                        if (openbus::app::vehicle::makeSpotlight(
                                vehicle->spotlights, spotlightSelection, modelView, spotlight) ==
                            openbus::app::vehicle::SpotlightStatus::Active) {
                            const std::array<double, 4> lightCenter = {spotlight.position[0],
                                                                       spotlight.position[1],
                                                                       spotlight.position[2], 1.0};
                            if (!openbus::rendering::sphereOutsideFrustum(
                                    spotlightFrustum, lightCenter, spotlight.range * 4.0)) {
                                vehicleSpotlightCandidates.push_back(
                                    {spotlight, vehicle.get() == playerVehicle_, 0.0,
                                     vehicle->motionObjectId});
                                if (openbus::app::vehicle::makeSpotlight(
                                        vehicle->spotlights, spotlightSelection,
                                        vehicleWorldTransform(*vehicle), worldSpotlight) ==
                                    openbus::app::vehicle::SpotlightStatus::Active) {
                                    diligentSpotlightSources.insert_or_assign(
                                        vehicle->motionObjectId,
                                        DiligentSpotlightSource{worldSpotlight, spotlightSelection,
                                                                std::hypot(spotlight.position[0],
                                                                           spotlight.position[1],
                                                                           spotlight.position[2]),
                                                                vehicle.get() == playerVehicle_});
                                }
                            }
                        } else if (!vehicle->invalidSpotlightLogged) {
                            gameLog.Warning(
                                "Ignoring invalid vehicle spotlight selection or parameters");
                            vehicle->invalidSpotlightLogged = true;
                        }
                    }
                }
                auto preparation = openbus::app::vehicle::prepareDiligentVehicleSceneDraw(
                    *vehicle, renderer, modelView, projection, context, environment.lighting);
                if (preparation.drawState.has_value()) {
                    vehicleDrawStates.push_back(std::move(*preparation.drawState));
                }
            }
            const auto vehicleSpotlights = openbus::app::vehicle::selectSpotlights(
                vehicleSpotlightCandidates, openbus::rendering::vehicleSpotlightSelectionLimit());
            struct PendingDiligentShadowUpdate {
                std::size_t slot = 0;
                openbus::rendering::SpotlightShadowCamera camera;
                openbus::rendering::Matrix4 clipFromView = {};
            };
            std::vector<PendingDiligentShadowUpdate> pendingShadowUpdates;
            pendingShadowUpdates.reserve(diligentSpotlightShadowSlots_.size());
            std::vector<openbus::rendering::DiligentSpotlightShadowBinding> spotlightShadowBindings(
                vehicleSpotlights.size());
            std::size_t shadowUpdatesRemaining =
                std::min(spotlightShadowUpdateBudget_, diligentSpotlightShadowSlots_.size());
            const auto minimumNdcZ = renderer.device().GetDeviceInfo().NDC.MinZ;
            struct DueDiligentShadowUpdate {
                std::size_t slot = 0;
                bool player = false;
                int interval = 1;
            };
            std::vector<DueDiligentShadowUpdate> dueShadowUpdates;
            std::vector<int> shadowUpdateIntervals(vehicleSpotlights.size(), 1);
            for (std::size_t index = 0;
                 index < vehicleSpotlights.size() && index < diligentSpotlightShadowSlots_.size();
                 ++index) {
                auto& slot = diligentSpotlightShadowSlots_[index];
                const auto source =
                    diligentSpotlightSources.find(vehicleSpotlightCandidates[index].vehicleId);
                if (source == diligentSpotlightSources.end()) {
                    slot = {};
                    continue;
                }
                const DiligentSpotlightSource& selected = source->second;
                if (slot.vehicleId != vehicleSpotlightCandidates[index].vehicleId ||
                    openbus::app::vehicle::spotlightSelectionChanged(slot.selection,
                                                                     selected.selection)) {
                    slot = {};
                    slot.vehicleId = vehicleSpotlightCandidates[index].vehicleId;
                    slot.selection = selected.selection;
                }
                const int interval = selected.player
                                         ? 1
                                         : openbus::app::vehicle::aiSpotlightUpdateIntervalFrames(
                                               {selected.distance, 0.0, 0.0, false, false},
                                               aiSpotlightScheduleConfig_);
                shadowUpdateIntervals[index] = interval;
                const bool updateDue = selected.player || !slot.valid ||
                                       lightingFrameIndex_ - slot.lastUpdateFrame >=
                                           static_cast<std::uint64_t>(interval);
                if (updateDue) {
                    dueShadowUpdates.push_back({index, selected.player, interval});
                }
            }
            std::stable_sort(
                dueShadowUpdates.begin(), dueShadowUpdates.end(),
                [&](const DueDiligentShadowUpdate& first, const DueDiligentShadowUpdate& second) {
                    return openbus::app::vehicle::spotlightShadowUpdateBefore(
                        first.player, diligentSpotlightShadowSlots_[first.slot].lastUpdateFrame,
                        second.player, diligentSpotlightShadowSlots_[second.slot].lastUpdateFrame,
                        spotlightShadowUpdateBudget_, lightingFrameIndex_);
                });
            for (const DueDiligentShadowUpdate& due : dueShadowUpdates) {
                if (shadowUpdatesRemaining == 0) {
                    break;
                }
                auto& slot = diligentSpotlightShadowSlots_[due.slot];
                const auto source =
                    diligentSpotlightSources.find(vehicleSpotlightCandidates[due.slot].vehicleId);
                if (source == diligentSpotlightSources.end()) {
                    continue;
                }
                const auto shadowCamera =
                    openbus::rendering::makeSpotlightShadowCamera(source->second.worldLight);
                if (!shadowCamera) {
                    slot.valid = false;
                    slot.camera.reset();
                    continue;
                }
                const auto clipFromView =
                    openbus::rendering::buildDiligentSpotlightShadowClipFromView(*shadowCamera,
                                                                                 view, minimumNdcZ);
                if (!clipFromView) {
                    slot.valid = false;
                    slot.camera.reset();
                    continue;
                }
                slot.valid = false;
                slot.camera.reset();
                pendingShadowUpdates.push_back({due.slot, *shadowCamera, *clipFromView});
                --shadowUpdatesRemaining;
            }
            std::vector<openbus::rendering::DiligentHumanPlacementSceneDrawState> humanDrawStates;
            humanDrawStates.reserve(humanTrafficActors_.size() + alightedPassengers_.size());
            const auto appendHumanPlacement = [&](const std::filesystem::path& modelPath,
                                                  const openbus::map::MapTrafficPose& pose,
                                                  double gaitPhaseRadians, double gaitWeight,
                                                  openbus::rendering::MotionObjectKey motionKey) {
                const std::string modelKey = openbus::pathToUtf8(modelPath.lexically_normal());
                const auto source = diligentHumanRenderSources_.find(modelKey);
                const auto radius = diligentHumanBoundingRadii_.find(modelKey);
                if (source == diligentHumanRenderSources_.end() ||
                    radius == diligentHumanBoundingRadii_.end()) {
                    return false;
                }
                auto placement = openbus::rendering::prepareDiligentHumanTrafficPlacement(
                    renderer, modelKey, *source->second, pose, gaitPhaseRadians, gaitWeight,
                    std::nullopt, radius->second, {motionKey.object, 0, motionKey.subObject},
                    motionKey, environment.lighting);
                if (!placement.has_value()) {
                    return false;
                }
                humanDrawStates.push_back(std::move(*placement));
                return true;
            };
            for (const HumanTrafficState& human : humanTrafficActors_) {
                if (!human.populationEnabled) {
                    continue;
                }
                const auto pose = openbus::map::sampleMapTrafficRoute(
                    human.actor.route, human.actor.distanceMeters, human.actor.lateralOffsetMeters);
                if (!pose.has_value()) {
                    throw std::runtime_error("Cannot sample a pedestrian render pose");
                }
                appendHumanPlacement(std::filesystem::path(human.actor.modelPath), *pose,
                                     human.gait.phaseRadians, human.gait.weight,
                                     {human.motionObjectId, 0});
            }
            if (!scheduledPassengerHumanModels_.empty() && lastMainCameraPoseValid_) {
                for (const auto& passenger : alightedPassengers_) {
                    const auto pose = openbus::app::passengerPedestrianHandoffPose(passenger);
                    if (std::hypot(pose.position[0] - lastMainCameraPose_.position[0],
                                   pose.position[1] - lastMainCameraPose_.position[1]) <= 75.0) {
                        const auto& model =
                            scheduledPassengerHumanModels_[passenger.modelIndex %
                                                           scheduledPassengerHumanModels_.size()];
                        appendHumanPlacement(
                            model, pose,
                            openbus::app::passengerWalkingGaitPhase(
                                passenger.travelledMeters /
                                openbus::app::passengerApproachSpeedMetersPerSecond),
                            1.0, {passenger.motionObjectId, 0});
                    }
                }
            }
            const auto appendSeatedPassengers = [&](Vehicle& bus,
                                                    const openbus::map::MapTrafficPose& busPose,
                                                    std::size_t modelOffset) {
                if (!bus.passengers || scheduledPassengerHumanModels_.empty() ||
                    !lastMainCameraPoseValid_ ||
                    std::hypot(busPose.position[0] - lastMainCameraPose_.position[0],
                               busPose.position[1] - lastMainCameraPose_.position[1]) > 75.0) {
                    return;
                }
                auto& service = bus.passengers->service;
                const auto cabinSections = passengerCabinSectionPoses(bus, busPose, true);
                std::vector<std::size_t> alightingSeats;
                if (const auto seat = service.alightingSeatIndex()) {
                    alightingSeats.push_back(*seat);
                }
                alightingSeats.insert(alightingSeats.end(),
                                      service.queuedAlightingSeatIndices().begin(),
                                      service.queuedAlightingSeatIndices().end());
                for (std::size_t seatIndex = 0; seatIndex < service.seatCount(); ++seatIndex) {
                    if (service.passengerCountOnSeat(seatIndex) == 0 ||
                        std::find(alightingSeats.begin(), alightingSeats.end(), seatIndex) !=
                            alightingSeats.end()) {
                        continue;
                    }
                    const std::size_t sectionIndex = bus.passengers->sectionIndexForSeat(seatIndex);
                    const auto& cabin = bus.passengers->sectionConfiguration(sectionIndex);
                    const auto& seat =
                        cabin.passengerPositions.at(bus.passengers->localSeatIndex(seatIndex));
                    const auto passengerPose =
                        passengerCabinSeatPose(cabinSections.at(sectionIndex).pose, seat);
                    const std::size_t modelIndex =
                        service.passengerModelIndexOnSeat(seatIndex).value_or(modelOffset +
                                                                              seatIndex) %
                        scheduledPassengerHumanModels_.size();
                    appendHumanPlacement(scheduledPassengerHumanModels_[modelIndex], passengerPose,
                                         0.0, 0.0,
                                         openbus::app::seatedPassengerMotionObjectKey(
                                             bus.motionObjectId, seatIndex));
                }
            };
            const auto appendPassengerTransitions =
                [&](Vehicle& bus, const openbus::map::MapTrafficPose& busPose,
                    std::size_t modelOffset, const openbus::map::MapTrafficActor* routeActor,
                    const openbus::map::MapTrafficPassengerStop* boardingStop,
                    const std::optional<openbus::map::MapTrafficPose>& stopPose,
                    const BodyPose* playerChassis) {
                    if (!bus.passengers || boardingStop == nullptr || !stopPose ||
                        scheduledPassengerHumanModels_.empty() || !lastMainCameraPoseValid_ ||
                        bus.passengers->service.currentStopId() != boardingStop->stopId ||
                        std::hypot(busPose.position[0] - lastMainCameraPose_.position[0],
                                   busPose.position[1] - lastMainCameraPose_.position[1]) > 75.0 ||
                        std::hypot(stopPose->position[0] - lastMainCameraPose_.position[0],
                                   stopPose->position[1] - lastMainCameraPose_.position[1]) >
                            75.0) {
                        return;
                    }
                    auto& service = bus.passengers->service;
                    const auto cabinSections = passengerCabinSectionPoses(bus, busPose, true);
                    const double laneOffset =
                        routeActor == nullptr ? 0.0 : routeActor->lateralOffsetMeters;
                    const double vehicleWidth =
                        routeActor == nullptr ? bus.trafficWidthMeters : routeActor->widthMeters;
                    const auto curbPose = passengerCurbPose(
                        *stopPose, mapRenderer_->leftHandTraffic(), laneOffset, vehicleWidth);
                    const auto submitTravelPath = [&](const PassengerCabinTravelPathData& path,
                                                      double progress, double startHeading,
                                                      double endHeading, std::size_t modelIndex,
                                                      double seatHeight, bool boarding,
                                                      std::size_t seatIndex) {
                        const bool hasSeat = seatHeight > 0.0;
                        const auto transition =
                            passengerSeatTransition(progress, boarding, hasSeat);
                        const auto sampled = openbus::rendering::sampleHumanPath(
                            path.positions, transition.pathProgress, startHeading, endHeading);
                        openbus::map::MapTrafficPose passengerPose;
                        passengerPose.position = sampled.position;
                        passengerPose.headingRadians = sampled.headingRadians;
                        double pathLength = 0.0;
                        for (std::size_t index = 1; index < path.positions.size(); ++index) {
                            pathLength +=
                                std::hypot(path.positions[index][0] - path.positions[index - 1][0],
                                           path.positions[index][1] - path.positions[index - 1][1],
                                           path.positions[index][2] - path.positions[index - 1][2]);
                        }
                        const double gaitPhase = openbus::app::passengerWalkingGaitPhase(
                            pathLength * transition.pathProgress /
                            openbus::app::passengerApproachSpeedMetersPerSecond);
                        const std::size_t selectedModel =
                            modelIndex % scheduledPassengerHumanModels_.size();
                        appendHumanPlacement(scheduledPassengerHumanModels_[selectedModel],
                                             passengerPose, gaitPhase,
                                             1.0 - transition.seatedFraction,
                                             openbus::app::seatedPassengerMotionObjectKey(
                                                 bus.motionObjectId, seatIndex));
                    };

                    std::vector<std::size_t> alightingSeats;
                    if (const auto seat = service.alightingSeatIndex()) {
                        alightingSeats.push_back(*seat);
                    }
                    alightingSeats.insert(alightingSeats.end(),
                                          service.queuedAlightingSeatIndices().begin(),
                                          service.queuedAlightingSeatIndices().end());
                    for (const std::size_t seatIndex : alightingSeats) {
                        if (seatIndex >= service.seatCount()) {
                            throw std::logic_error("Alighting passenger has no configured seat");
                        }
                        const std::size_t sectionIndex =
                            bus.passengers->sectionIndexForSeat(seatIndex);
                        const auto& cabin = bus.passengers->sectionConfiguration(sectionIndex);
                        const auto& seat =
                            cabin.passengerPositions.at(bus.passengers->localSeatIndex(seatIndex));
                        const auto& sectionPose = cabinSections.at(sectionIndex).pose;
                        const auto seatPose = passengerCabinSeatPose(sectionPose, seat);
                        const Vehicle* sectionVehicle =
                            bus.passengers->sectionVehicle(sectionIndex);
                        if (sectionVehicle == nullptr) {
                            throw std::logic_error(
                                "Alighting passenger has no owning vehicle section");
                        }
                        const auto exitDoor = bus.passengers->exitDoorForSeat(seatIndex);
                        const auto outsidePose =
                            passengerConfiguredDoorQueuePose(
                                *sectionVehicle, sectionPose, mapRenderer_->leftHandTraffic(),
                                cabin.passengerExitPathPoints, exitDoor, 0)
                                .value_or(curbPose);
                        PassengerCabinTravelPathData path = passengerCabinTravelPath(
                            cabinSections, sectionIndex, seatPose.position, sectionIndex,
                            passengerCabinLocalPoint(sectionPose, outsidePose.position), false,
                            sectionIndex, exitDoor);
                        if (path.positions.empty()) {
                            path.positions = {seatPose.position, outsidePose.position};
                            path.segmentLinkIndices = {std::nullopt};
                        }
                        const std::size_t modelIndex =
                            service.passengerModelIndexOnSeat(seatIndex).value_or(modelOffset +
                                                                                  seatIndex);
                        submitTravelPath(path, service.alightingProgressForSeat(seatIndex),
                                         seatPose.headingRadians, outsidePose.headingRadians,
                                         modelIndex, seat.seatHeight, false, seatIndex);
                    }

                    const auto initialQueue =
                        passengerNetwork_.initialQueues.find(boardingStop->stopId);
                    const std::size_t initialCount =
                        initialQueue == passengerNetwork_.initialQueues.end()
                            ? 0
                            : initialQueue->second;
                    for (const auto& boarding : service.boardingPassengers()) {
                        if (boarding.seatIndex >= service.seatCount()) {
                            throw std::logic_error("Boarding passenger has no configured seat");
                        }
                        const std::size_t sectionIndex =
                            bus.passengers->sectionIndexForSeat(boarding.seatIndex);
                        const auto& cabin = bus.passengers->sectionConfiguration(sectionIndex);
                        const auto& seat = cabin.passengerPositions.at(
                            bus.passengers->localSeatIndex(boarding.seatIndex));
                        const auto seatPose =
                            passengerCabinSeatPose(cabinSections.at(sectionIndex).pose, seat);
                        const auto queueSlot = service.boardingPassengerQueueSlotForSeat(
                            boarding.seatIndex, initialCount);
                        if (!queueSlot) {
                            throw std::logic_error("Boarding passenger has no reserved queue slot");
                        }
                        const auto outsidePose =
                            playerChassis != nullptr
                                ? playerPassengerDoorQueuePose(bus, *playerChassis,
                                                               mapRenderer_->leftHandTraffic(),
                                                               *queueSlot)
                                : passengerQueuePose(boardingStop->stopId, *queueSlot)
                                      .value_or(curbPose);
                        PassengerCabinTravelPathData path = passengerCabinTravelPath(
                            cabinSections, 0, outsidePose.position, sectionIndex,
                            passengerCabinStandingPoint(seat.position, seat.seatHeight), true, 0,
                            bus.passengers->entryDoor());
                        if (path.positions.empty()) {
                            path.positions = {outsidePose.position, curbPose.position,
                                              seatPose.position};
                            path.segmentLinkIndices = {std::nullopt, std::nullopt};
                        }
                        submitTravelPath(path, boarding.progress, outsidePose.headingRadians,
                                         seatPose.headingRadians, boarding.modelIndex,
                                         seat.seatHeight, true, boarding.seatIndex);
                    }
                };
            for (const TrafficVehicleState& trafficVehicle : trafficFleet_.vehicles) {
                if (!trafficVehicle.isBus() || trafficVehicle.vehicle == nullptr ||
                    !trafficVehicle.vehicle->passengers || !trafficVehicle.populationEnabled ||
                    !trafficVehicle.vehicle->mapTrafficTileActive) {
                    continue;
                }
                auto busPose = openbus::map::sampleMapTrafficActorPose(
                    trafficVehicle.actor, trafficVehicle.actor.distanceMeters);
                if (!busPose.has_value()) {
                    throw std::runtime_error("Cannot sample a scheduled AI bus passenger pose");
                }
                busPose->position[2] += trafficVehicle.vehicle->aiDeltaHeight;
                if (trafficVehicle.vehicle->trafficRenderPlacement) {
                    const VehiclePlacement& rendered =
                        *trafficVehicle.vehicle->trafficRenderPlacement;
                    busPose->position = rendered.position;
                    busPose->headingRadians =
                        (90.0 - rendered.yawDegrees) * std::numbers::pi / 180.0;
                    busPose->pitchRadians = rendered.pitchDegrees * std::numbers::pi / 180.0;
                    busPose->rollRadians = rendered.rollDegrees * std::numbers::pi / 180.0;
                }
                const std::size_t modelOffset =
                    trafficVehicle.bus->passengerStops.empty()
                        ? 0
                        : static_cast<std::size_t>(
                              trafficVehicle.bus->passengerStops.front().stopId);
                appendSeatedPassengers(*trafficVehicle.vehicle, *busPose, modelOffset);
                const openbus::map::MapTrafficPassengerStop* boardingStop = nullptr;
                std::optional<openbus::map::MapTrafficPose> stopPose;
                if (trafficVehicle.bus->passengerStopArrived &&
                    trafficVehicle.bus->nextPassengerStopIndex <
                        trafficVehicle.bus->passengerStops.size()) {
                    boardingStop =
                        &trafficVehicle.bus
                             ->passengerStops[trafficVehicle.bus->nextPassengerStopIndex];
                    if (boardingStop->stopId !=
                        trafficVehicle.vehicle->passengers->service.currentStopId()) {
                        throw std::logic_error(
                            "Scheduled bus passenger service stop index does not match its active "
                            "call");
                    }
                    stopPose = openbus::map::sampleMapTrafficRoute(
                        trafficVehicle.actor.route, boardingStop->routeDistanceMeters, 0.0);
                    if (!stopPose.has_value()) {
                        throw std::runtime_error(
                            "Cannot sample a scheduled bus passenger boarding pose");
                    }
                }
                appendPassengerTransitions(*trafficVehicle.vehicle, *busPose, modelOffset,
                                           &trafficVehicle.actor, boardingStop, stopPose, nullptr);
            }
            if (playerVehicle_ != nullptr && playerVehicle_->passengers) {
                openbus::map::MapTrafficPose playerPose;
                playerPose.position =
                    transformLocalPoint(chassis, {0.0, 0.0, playerVehicle_->modelOffsetZ});
                playerPose.headingRadians = std::atan2(chassis.rotation[0], chassis.rotation[3]);
                appendSeatedPassengers(*playerVehicle_, playerPose, 0);
                const openbus::map::MapTrafficPassengerStop* boardingStop = nullptr;
                std::optional<openbus::map::MapTrafficPose> stopPose;
                if (activePlayerPassengerStop_) {
                    if (!activePlayerPassengerStopRouteIndex_ ||
                        *activePlayerPassengerStopRouteIndex_ >= passengerNetwork_.routes.size() ||
                        !activePlayerPassengerStopIndex_) {
                        throw std::logic_error(
                            "Active player passenger stop has no mapped route call");
                    }
                    const auto& route =
                        passengerNetwork_.routes[*activePlayerPassengerStopRouteIndex_];
                    if (*activePlayerPassengerStopIndex_ >= route.stops.size()) {
                        throw std::logic_error(
                            "Active player passenger stop index is out of range");
                    }
                    boardingStop = &route.stops[*activePlayerPassengerStopIndex_];
                    if (boardingStop->stopId != *activePlayerPassengerStop_) {
                        throw std::logic_error(
                            "Active player passenger stop index does not match its ID");
                    }
                    stopPose = route.stopPose(*activePlayerPassengerStopIndex_);
                    if (!stopPose.has_value()) {
                        throw std::runtime_error(
                            "Cannot sample the player passenger boarding pose");
                    }
                }
                appendPassengerTransitions(*playerVehicle_, playerPose, 0, nullptr, boardingStop,
                                           stopPose, &chassis);
            }
            std::vector<openbus::rendering::PrimitiveVertex> diligentTicketVertices;
            struct TicketTransactionActorPose {
                openbus::map::MapTrafficPose pose;
                bool walking = false;
                double gaitTime = 0.0;
                bool ticketVisible = false;
            };
            const auto ticketTransactionActorPose =
                [&](int stopId, const openbus::app::WaitingPassenger& passenger)
                -> std::optional<TicketTransactionActorPose> {
                Vehicle* approachVehicle = nullptr;
                const PassengerApproachProgress* approach = nullptr;
                for (const auto& [vehicle, progress] : passengerApproachProgress_) {
                    if (progress.stopId == stopId && progress.modelIndex == passenger.modelIndex &&
                        (!passenger.journey ||
                         (progress.routeIndex == passenger.journey->routeIndex &&
                          progress.stopCallIndex == passenger.journey->originStopIndex))) {
                        approachVehicle = vehicle;
                        approach = &progress;
                        break;
                    }
                }
                if (approach == nullptr || !approach->ticketBought) {
                    return std::nullopt;
                }

                const VehicleConfig* cabin = nullptr;
                openbus::map::MapTrafficPose busPose;
                std::optional<openbus::map::MapTrafficPose> destination;
                if (approachVehicle == playerVehicle_ && playerVehicle_ != nullptr &&
                    playerVehicle_->passengers != nullptr && activePlayerPassengerStop_ == stopId &&
                    activePlayerPassengerStopRouteIndex_ == approach->routeIndex &&
                    activePlayerPassengerStopIndex_ == approach->stopCallIndex) {
                    cabin = &playerVehicle_->passengers->configuration();
                    busPose.position =
                        transformLocalPoint(chassis, {0.0, 0.0, playerVehicle_->modelOffsetZ});
                    busPose.headingRadians = std::atan2(chassis.rotation[0], chassis.rotation[3]);
                    destination = playerPassengerDoorQueuePose(*playerVehicle_, chassis,
                                                               mapRenderer_->leftHandTraffic(),
                                                               passenger.queueSlot);
                } else {
                    const auto vehicle =
                        std::find_if(trafficFleet_.vehicles.begin(), trafficFleet_.vehicles.end(),
                                     [&](const TrafficVehicleState& state) {
                                         return state.vehicle == approachVehicle && state.isBus() &&
                                                state.bus != nullptr &&
                                                state.vehicle->passengers != nullptr;
                                     });
                    if (vehicle == trafficFleet_.vehicles.end() ||
                        !passengerNetwork_.passengerMatchesService(
                            passenger, approach->routeIndex, vehicle->vehicle->passengers->service,
                            approach->stopCallIndex)) {
                        return std::nullopt;
                    }
                    auto sampledPose = openbus::map::sampleMapTrafficActorPose(
                        vehicle->actor, vehicle->actor.distanceMeters);
                    if (!sampledPose) {
                        throw std::runtime_error(
                            "Cannot sample scheduled passenger ticket transaction bus pose");
                    }
                    sampledPose->position[2] += vehicle->vehicle->aiDeltaHeight;
                    if (vehicle->vehicle->trafficRenderPlacement) {
                        const VehiclePlacement& rendered =
                            *vehicle->vehicle->trafficRenderPlacement;
                        sampledPose->position = rendered.position;
                        sampledPose->headingRadians =
                            (90.0 - rendered.yawDegrees) * std::numbers::pi / 180.0;
                        sampledPose->pitchRadians =
                            rendered.pitchDegrees * std::numbers::pi / 180.0;
                        sampledPose->rollRadians = rendered.rollDegrees * std::numbers::pi / 180.0;
                    }
                    busPose = *sampledPose;
                    cabin = &vehicle->vehicle->passengers->configuration();
                    destination = passengerConfiguredDoorQueuePose(
                        *vehicle->vehicle, busPose, mapRenderer_->leftHandTraffic(),
                        cabin->passengerEntryPathPoints, vehicle->vehicle->passengers->entryDoor(),
                        passenger.queueSlot);
                    if (!destination) {
                        destination = passengerDoorQueuePose(
                            busPose, vehicle->vehicle->trafficLengthMeters * 0.3,
                            vehicle->vehicle->trafficWidthMeters, mapRenderer_->leftHandTraffic(),
                            passenger.queueSlot);
                    }
                }
                if (cabin == nullptr || !destination) {
                    return std::nullopt;
                }

                const auto origin = passengerTicketSaleOriginPose(
                    *cabin, busPose, *destination, approach->ticketMachineConfigurationIndex);
                const auto moneyPoint = approach->ticketPaymentRequired
                                            ? passengerTicketMoneyPointPose(*cabin, busPose, origin)
                                            : origin;
                const auto changePoint = passengerTicketChangePointPose(*cabin, busPose, moneyPoint,
                                                                        approach->changeDue);
                const auto transactionPose = passengerTicketTransactionPose(
                    *cabin, busPose, *destination, approach->ticketMachineConfigurationIndex,
                    approach->ticketPaymentRequired, approach->changeDue);

                if (!approach->ticketPaymentStarted && approach->ticketMachineConfigurationIndex) {
                    const bool walking = approach->ticketMachineApproachSeconds <
                                         passengerApproachDuration(*destination, origin);
                    return TicketTransactionActorPose{
                        passengerApproachPose(*destination, origin,
                                              approach->ticketMachineApproachSeconds),
                        walking, approach->ticketMachineApproachSeconds};
                }
                if (!approach->ticketPrinted) {
                    const auto movement = passengerTicketTransactionActorPose(
                        origin, moneyPoint, changePoint, approach->ticketPaymentRequired,
                        approach->changeDue, approach->ticketMoneyPointReached,
                        approach->ticketChangePointReached,
                        approach->ticketMoneyPointApproachSeconds,
                        approach->ticketChangePointApproachSeconds);
                    return TicketTransactionActorPose{movement.pose, movement.walking,
                                                      approach->elapsedSeconds};
                }
                if (approach->ticketStamped && approach->ticketStamperConfigurationIndex) {
                    const auto& stamper = cabin->passengerCabinStampers.at(
                        *approach->ticketStamperConfigurationIndex);
                    const auto stamperPose = passengerVehicleLocalPose(busPose, stamper.position);
                    const auto movement = passengerStamperActorPose(
                        transactionPose, stamperPose, *destination,
                        approach->ticketStamperApproachSeconds, approach->ticketStamperWaitSeconds,
                        approach->ticketStamperReturnSeconds);
                    return TicketTransactionActorPose{movement.pose, movement.walking,
                                                      approach->elapsedSeconds,
                                                      approach->ticketPrinted};
                }
                const bool walking = approach->ticketMachineReturnSeconds <
                                     passengerApproachDuration(transactionPose, *destination);
                return TicketTransactionActorPose{
                    passengerApproachPose(transactionPose, *destination,
                                          approach->ticketMachineReturnSeconds),
                    walking, approach->ticketMachineReturnSeconds, approach->ticketPrinted};
            };
            if (!scheduledPassengerHumanModels_.empty() && lastMainCameraPoseValid_) {
                constexpr double waitingPassengerIdleGaitWeight = 0.08;
                constexpr double waitingPassengerIdleCycleRadiansPerSecond = 0.7;
                for (const auto& [stopId, waitingCount] : passengerNetwork_.queues) {
                    if (waitingCount == 0) {
                        continue;
                    }
                    const auto& passengers = passengerNetwork_.waitingPassengers(stopId);
                    for (std::size_t index = 0; index < passengers.size(); ++index) {
                        const auto& passenger = passengers[index];
                        auto pose = passengerQueuePose(stopId, passenger);
                        if (!pose.has_value()) {
                            continue;
                        }
                        const PassengerQueueApproachProgress* queueApproach = nullptr;
                        for (const auto& [approachVehicle, approaches] :
                             passengerQueueApproachProgress_) {
                            static_cast<void>(approachVehicle);
                            const auto approach = approaches.find(passenger.modelIndex);
                            if (approach != approaches.end() && approach->second.stopId == stopId &&
                                (!passenger.journey ||
                                 (approach->second.routeIndex == passenger.journey->routeIndex &&
                                  approach->second.stopCallIndex ==
                                      passenger.journey->originStopIndex))) {
                                queueApproach = &approach->second;
                                break;
                            }
                        }
                        double gaitWeight = waitingPassengerIdleGaitWeight;
                        double gaitPhase = simulation.simulationTime() *
                                               waitingPassengerIdleCycleRadiansPerSecond +
                                           static_cast<double>(index) * 1.7;
                        if (queueApproach != nullptr) {
                            const auto& approachStart =
                                queueApproach->posesInitialized ? queueApproach->originPose : *pose;
                            const auto& approachDestination = queueApproach->posesInitialized
                                                                  ? queueApproach->destinationPose
                                                                  : *pose;
                            if (queueApproach->returningToSpawn) {
                                const double duration =
                                    passengerApproachDuration(approachStart, approachDestination);
                                gaitWeight = queueApproach->elapsedSeconds < duration
                                                 ? 1.0
                                                 : waitingPassengerIdleGaitWeight;
                                pose = passengerApproachPose(approachStart, approachDestination,
                                                             queueApproach->elapsedSeconds);
                            } else {
                                const double duration = passengerQueueApproachDuration(
                                    queueApproach->roadApproach, approachStart,
                                    approachDestination);
                                gaitWeight = queueApproach->elapsedSeconds < duration
                                                 ? 1.0
                                                 : waitingPassengerIdleGaitWeight;
                                pose = passengerQueueApproachPose(
                                    queueApproach->roadApproach, approachStart, approachDestination,
                                    queueApproach->elapsedSeconds);
                            }
                            if (gaitWeight == 1.0) {
                                gaitPhase =
                                    passengerWalkingGaitPhase(queueApproach->elapsedSeconds);
                            }
                        }
                        bool ticketVisible = false;
                        if (const auto transactionPose =
                                ticketTransactionActorPose(stopId, passenger)) {
                            pose = transactionPose->pose;
                            ticketVisible = transactionPose->ticketVisible;
                            gaitWeight =
                                transactionPose->walking ? 1.0 : waitingPassengerIdleGaitWeight;
                            if (gaitWeight == 1.0) {
                                gaitPhase = passengerWalkingGaitPhase(transactionPose->gaitTime);
                            }
                        }
                        if (std::hypot(pose->position[0] - lastMainCameraPose_.position[0],
                                       pose->position[1] - lastMainCameraPose_.position[1]) >
                            75.0) {
                            continue;
                        }
                        const std::size_t modelIndex =
                            passenger.modelIndex % scheduledPassengerHumanModels_.size();
                        const auto motionKey = openbus::app::waitingPassengerMotionObjectKey(
                            stopId, passenger.arrivalOrder);
                        const bool actorSubmitted =
                            appendHumanPlacement(scheduledPassengerHumanModels_[modelIndex], *pose,
                                                 gaitPhase, gaitWeight, motionKey);
                        openbus::app::appendPassengerTicketCard(
                            diligentTicketVertices, pose->position, pose->headingRadians,
                            ticketVisible, actorSubmitted);
                        if (actorSubmitted) {
                            diligentWaitingPassengerRenderedPoses.push_back(
                                {{stopId, passenger.modelIndex}, *pose});
                        }
                    }
                }
            }
            for (const PendingDiligentShadowUpdate& update : pendingShadowUpdates) {
                std::vector<openbus::rendering::DiligentShadowCaster> shadowCasters;
                if (mapRenderer_ != nullptr) {
                    auto mapCasters = openbus::app::buildDiligentMapTileShadowCasters(
                        renderer.mapTiles(), mapRenderer_->mapDefinition().coordinates.tileSize,
                        view, update.clipFromView);
                    shadowCasters.insert(shadowCasters.end(), mapCasters.begin(), mapCasters.end());
                }
                for (const auto& placement : sceneryUpdate.placements) {
                    const auto* asset =
                        renderer.sceneryAsset(placement.configPath, placement.assetRoot);
                    if (asset == nullptr) {
                        continue;
                    }
                    const auto resolveSeasonalTexture = [&renderer,
                                                         &placement](const auto& seasonalPaths) {
                        if (!seasonalPaths) {
                            return static_cast<Diligent::ITextureView*>(nullptr);
                        }
                        return openbus::app::resolveDiligentMapSeasonalTexture(
                            placement.seasonIndex, *seasonalPaths,
                            [&renderer](const std::filesystem::path& path) {
                                const auto* texture = renderer.seasonalSceneryTexture(path);
                                return texture ? &texture->shaderResourceView() : nullptr;
                            });
                    };
                    std::vector<Diligent::ITextureView*> seasonalBatchTextures;
                    seasonalBatchTextures.reserve(asset->batches().size());
                    for (const auto& batch : asset->batches()) {
                        seasonalBatchTextures.push_back(
                            resolveSeasonalTexture(batch.source.seasonTexturePaths));
                    }
                    std::vector<Diligent::ITextureView*> seasonalTreeTextures;
                    seasonalTreeTextures.reserve(asset->trees().size());
                    for (const auto& tree : asset->trees()) {
                        seasonalTreeTextures.push_back(
                            resolveSeasonalTexture(tree.seasonTexturePaths));
                    }
                    const float visibilityFade = openbus::app::diligentMapSceneryVisibilityFade(
                        placement, view, projection, mainViewMinimumObjectViewportFraction_);
                    auto sceneryCasters = openbus::app::buildDiligentMapSceneryShadowCasters(
                        placement, *asset, view, update.clipFromView, seasonalBatchTextures,
                        seasonalTreeTextures, visibilityFade);
                    shadowCasters.insert(shadowCasters.end(), sceneryCasters.begin(),
                                         sceneryCasters.end());
                }
                for (const auto& vehicleState : vehicleDrawStates) {
                    const auto* asset = renderer.vehicleAsset(vehicleState.vehicleObjectId);
                    if (asset == nullptr) {
                        throw std::logic_error(
                            "Diligent vehicle shadow pass lost a prepared vehicle asset");
                    }
                    auto vehicleCasters = asset->shadowCasters(
                        update.clipFromView, vehicleState.modelView, vehicleState.groupTransforms,
                        vehicleState.materialOverrides.overrides);
                    shadowCasters.insert(shadowCasters.end(), vehicleCasters.begin(),
                                         vehicleCasters.end());
                }
                std::vector<openbus::rendering::DiligentHumanShadowDrawState> lightHumanShadows;
                lightHumanShadows.reserve(humanDrawStates.size());
                for (const auto& actor : humanDrawStates) {
                    if (!actor.visible || actor.asset == nullptr) {
                        continue;
                    }
                    const auto modelView =
                        openbus::rendering::multiplyMatrix4(view, actor.worldTransform);
                    auto humanCasters = actor.asset->shadowCasters(
                        openbus::rendering::multiplyMatrix4(update.clipFromView, modelView),
                        actor.currentPose, actor.seated);
                    shadowCasters.insert(shadowCasters.end(), humanCasters.begin(),
                                         humanCasters.end());
                }
                static_cast<void>(renderer.drawOpaqueSpotlightShadowLayer(
                    static_cast<Diligent::Uint32>(update.slot), shadowCasters));
                auto& slot = diligentSpotlightShadowSlots_[update.slot];
                slot.camera = update.camera;
                slot.lastUpdateFrame = lightingFrameIndex_;
                slot.valid = true;
            }
            for (std::size_t index = 0;
                 index < vehicleSpotlights.size() && index < diligentSpotlightShadowSlots_.size();
                 ++index) {
                auto& slot = diligentSpotlightShadowSlots_[index];
                const int interval = shadowUpdateIntervals[index];
                const bool fresh = slot.valid && slot.camera &&
                                   openbus::app::vehicle::spotlightShadowFresh(
                                       lightingFrameIndex_ - slot.lastUpdateFrame, interval);
                slot.visibility =
                    openbus::app::vehicle::advanceSpotlightShadowVisibility(slot.visibility, fresh);
                if (fresh && slot.visibility > 0.0F) {
                    spotlightShadowBindings[index] = {slot.camera,
                                                      static_cast<Diligent::Uint32>(index), 0.001F};
                }
            }
            const auto mainCameraLocalEye = [&] {
                const double x = cameraPosition[0] - chassis.position[0];
                const double y = cameraPosition[1] - chassis.position[1];
                const double z = cameraPosition[2] - chassis.position[2];
                return std::array<double, 3>{
                    chassis.rotation[0] * x + chassis.rotation[3] * y + chassis.rotation[6] * z,
                    chassis.rotation[1] * x + chassis.rotation[4] * y + chassis.rotation[7] * z,
                    chassis.rotation[2] * x + chassis.rotation[5] * y + chassis.rotation[8] * z};
            }();
            std::size_t reflectionIndex = 0;
            for (std::size_t cameraIndex = 0; cameraIndex < vehicleCameras_.size(); ++cameraIndex) {
                const VehicleCamera& reflectionCamera = vehicleCameras_[cameraIndex];
                if (reflectionCamera.kind != VehicleCameraKind::Reflexion &&
                    reflectionCamera.kind != VehicleCameraKind::Reflexion2) {
                    continue;
                }

                bool needed = false;
                for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                    if (!vehicle->mapTrafficRouteFinished &&
                        (!vehicle->followsMapTrafficRoute || vehicle->mapTrafficTileActive) &&
                        vehicle->needsReflectionTexture(reflectionIndex)) {
                        needed = true;
                        break;
                    }
                }
                if (needed) {
                    const int reflectionWidth =
                        static_cast<int>(reflectionTargetSizes.at(reflectionIndex));
                    const int reflectionHeight = reflectionWidth;
                    static_cast<void>(renderer.renderReflectionTarget(
                        reflectionIndex, frameId, true,
                        [&](openbus::rendering::DiligentReflectionTarget&,
                            openbus::rendering::DiligentTexturedMeshRenderer& reflectionMeshes) {
                            const int previousCameraView = cameraView_;
                            const int previousFramebufferWidth = framebufferWidth_;
                            const int previousFramebufferHeight = framebufferHeight_;
                            const double previousFovOffset = fieldOfViewOffset_;
                            const double previousLookYaw = viewLookYaw_;
                            const double previousLookPitch = viewLookPitch_;
                            const bool previousRenderingReflection = renderingReflection_;
                            const auto previousReflectionEye = reflectionViewerEyeLocal_;
                            const bool previousReflectionEyeValid = reflectionViewerEyeLocalValid_;
                            const auto restoreCameraState = [&] {
                                cameraView_ = previousCameraView;
                                framebufferWidth_ = previousFramebufferWidth;
                                framebufferHeight_ = previousFramebufferHeight;
                                fieldOfViewOffset_ = previousFovOffset;
                                viewLookYaw_ = previousLookYaw;
                                viewLookPitch_ = previousLookPitch;
                                renderingReflection_ = previousRenderingReflection;
                                reflectionViewerEyeLocal_ = previousReflectionEye;
                                reflectionViewerEyeLocalValid_ = previousReflectionEyeValid;
                            };

                            try {
                                framebufferWidth_ = reflectionWidth;
                                framebufferHeight_ = reflectionHeight;
                                cameraView_ = static_cast<int>(cameraIndex + 1);
                                fieldOfViewOffset_ = 0.0;
                                viewLookYaw_ = 0.0;
                                viewLookPitch_ = 0.0;
                                renderingReflection_ = true;
                                reflectionViewerEyeLocal_ = mainCameraLocalEye;
                                reflectionViewerEyeLocalValid_ = true;

                                const Matrix4 reflectionView =
                                    buildCameraViewMatrix(simulation, chassis);
                                const Matrix4 reflectionProjection =
                                    openbus::rendering::makePerspectiveMatrix(
                                        static_cast<double>(reflectionWidth),
                                        static_cast<double>(reflectionHeight),
                                        currentFieldOfView());
                                const std::array<double, 3> reflectionCameraPosition = {
                                    -(reflectionView[0] * reflectionView[12] +
                                      reflectionView[1] * reflectionView[13] +
                                      reflectionView[2] * reflectionView[14]),
                                    -(reflectionView[4] * reflectionView[12] +
                                      reflectionView[5] * reflectionView[13] +
                                      reflectionView[6] * reflectionView[14]),
                                    -(reflectionView[8] * reflectionView[12] +
                                      reflectionView[9] * reflectionView[13] +
                                      reflectionView[10] * reflectionView[14])};
                                const double reflectionGroundHeight =
                                    mapRenderer_ != nullptr
                                        ? mapRenderer_
                                              ->terrainHeightAt(reflectionCameraPosition[0],
                                                                reflectionCameraPosition[1])
                                              .value_or(0.0)
                                        : 0.0;
                                const auto reflectionEnvironment =
                                    openbus::rendering::buildDiligentSceneEnvironment(
                                        reflectionView, sharedVariables.sunDirection(),
                                        weatherBrightness, true,
                                        reflectionCameraPosition[2] - reflectionGroundHeight,
                                        sharedVariables.effectiveWeatherVisibilityMeters(),
                                        sharedVariables.weatherFogTopAboveGroundMeters(),
                                        openbus::rendering::configuredReflectionAmbientFloor());
                                Diligent::IDeviceContext& reflectionContext =
                                    renderer.immediateContext();
                                if (mapRenderer_ != nullptr) {
                                    static_cast<void>(
                                        openbus::app::uploadDiligentMapLightsInViewForEnvironment(
                                            reflectionMeshes, reflectionContext, *mapRenderer_,
                                            reflectionView, reflectionCameraPosition,
                                            weatherBrightness,
                                            static_cast<std::size_t>(
                                                graphicsPreferences_.values.sceneryLightLimit)));
                                }
                                const Matrix4 reflectionViewProjection =
                                    openbus::rendering::multiplyMatrix4(reflectionProjection,
                                                                        reflectionView);
                                static_cast<void>(renderer.drawWeatherCloudsInReflection(
                                    reflectionView, reflectionProjection, reflectionEnvironment.fog,
                                    sharedVariables.weatherCloudType(),
                                    sharedVariables.weatherCloudBaseAboveGroundMeters(),
                                    sharedVariables.weatherWindDirectionDegrees(),
                                    sharedVariables.weatherWindSpeedMetersPerSecond(),
                                    weatherBrightness,
                                    reflectionCameraPosition[2] - reflectionGroundHeight,
                                    simulation.simulationTime()));
                                const auto drawMapTilePass =
                                    [&](openbus::app::DiligentMapTileViewPass pass) {
                                        static_cast<void>(openbus::app::drawDiligentMapTilesInView(
                                            renderer.mapTiles(), reflectionMeshes,
                                            reflectionContext, fallbackTexture,
                                            reflectionViewProjection, pass, nullptr,
                                            reflectionEnvironment.lighting, &mapWeather));
                                    };
                                drawMapTilePass(openbus::app::DiligentMapTileViewPass::Terrain);
                                drawMapTilePass(openbus::app::DiligentMapTileViewPass::OpaqueRoads);
                                drawMapTilePass(openbus::app::DiligentMapTileViewPass::CutoutRoads);
                                static_cast<void>(
                                    openbus::app::drawDiligentMapSceneryPlacementsInView(
                                        renderer, sceneryUpdate.placements,
                                        openbus::rendering::DiligentSceneryPass::Opaque,
                                        reflectionMeshes, reflectionContext, fallbackTexture,
                                        reflectionView, reflectionProjection, {},
                                        reflectionEnvironment.lighting,
                                        reflectionMinimumObjectViewportFraction_));
                                for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                                    if (vehicle->mapTrafficRouteFinished ||
                                        (vehicle->followsMapTrafficRoute &&
                                         !vehicle->mapTrafficTileActive) ||
                                        !vehicle->loaded || vehicle->displayLists.empty()) {
                                        continue;
                                    }
                                    const auto viewpoint =
                                        vehicle.get() == playerVehicle_
                                            ? openbus::rendering::ViewpointContext::PlayerExterior
                                            : openbus::rendering::ViewpointContext::NonPlayer;
                                    static_cast<void>(
                                        openbus::app::vehicle::drawDiligentVehicleInView(
                                            *vehicle, renderer, reflectionMeshes, fallbackTexture,
                                            openbus::rendering::multiplyMatrix4(
                                                reflectionView, vehicleWorldTransform(*vehicle)),
                                            reflectionProjection, viewpoint,
                                            reflectionEnvironment.lighting));
                                }
                                for (const auto& actor : humanDrawStates) {
                                    if (!actor.visible || actor.asset == nullptr) {
                                        continue;
                                    }
                                    static_cast<void>(
                                        openbus::rendering::drawDiligentHumanAssetInView(
                                            *actor.asset, reflectionMeshes, reflectionContext,
                                            fallbackTexture, actor.worldTransform,
                                            actor.boundingRadius, reflectionView,
                                            reflectionProjection, actor.currentPose, actor.seated,
                                            reflectionEnvironment.lighting, actor.visible));
                                }
                                drawMapTilePass(openbus::app::DiligentMapTileViewPass::Water);
                                drawMapTilePass(
                                    openbus::app::DiligentMapTileViewPass::TransparentRoads);
                                static_cast<
                                    void>(openbus::app::drawDiligentMapSceneryPlacementsInView(
                                    renderer, sceneryUpdate.placements,
                                    openbus::rendering::DiligentSceneryPass::TransmapDepthPrepass,
                                    reflectionMeshes, reflectionContext, fallbackTexture,
                                    reflectionView, reflectionProjection, {},
                                    reflectionEnvironment.lighting,
                                    reflectionMinimumObjectViewportFraction_));
                                static_cast<void>(
                                    openbus::app::drawDiligentMapSceneryPlacementsInView(
                                        renderer, sceneryUpdate.placements,
                                        openbus::rendering::DiligentSceneryPass::Transparent,
                                        reflectionMeshes, reflectionContext, fallbackTexture,
                                        reflectionView, reflectionProjection, {},
                                        reflectionEnvironment.lighting,
                                        reflectionMinimumObjectViewportFraction_));
                            } catch (...) {
                                restoreCameraState();
                                throw;
                            }
                            restoreCameraState();
                        },
                        environment.clearColor));
                }
                ++reflectionIndex;
            }
            std::vector<Diligent::ITextureView*> reflectionTextures;
            reflectionTextures.reserve(reflectionTargetSizes.size());
            for (std::size_t index = 0; index < reflectionTargetSizes.size(); ++index) {
                reflectionTextures.push_back(&renderer.reflectionTarget(index).colorTexture());
            }
            std::vector<Diligent::ITextureView*> visibleReflectionTextures(
                reflectionTextures.size());
            for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                const auto drawState =
                    std::find_if(vehicleDrawStates.begin(), vehicleDrawStates.end(),
                                 [&vehicle](const auto& state) {
                                     return state.vehicleObjectId == vehicle->motionObjectId;
                                 });
                if (drawState != vehicleDrawStates.end()) {
                    std::copy(reflectionTextures.begin(), reflectionTextures.end(),
                              visibleReflectionTextures.begin());
                    for (std::size_t index = 0; index < visibleReflectionTextures.size(); ++index) {
                        if (!vehicle->needsReflectionTexture(index)) {
                            visibleReflectionTextures[index] = nullptr;
                        }
                    }
                    openbus::app::vehicle::bindDiligentVehicleReflectionTextures(
                        vehicle->displayLists, drawState->materialOverrides,
                        visibleReflectionTextures);
                }
            }
            renderer.setVehicleSpotLightsWithShadows(vehicleSpotlights, spotlightShadowBindings,
                                                     view, vehicleSpotlights.size(), minimumNdcZ);
            static_cast<void>(openbus::app::drawDiligentMapMainView(
                renderer, fallbackTexture, environment.lighting,
                openbus::app::DiligentMapMainViewPass::Opaque, &mapWeather));
            static_cast<void>(openbus::app::drawDiligentMapSceneryPlacements(
                renderer, sceneryUpdate.placements, openbus::rendering::DiligentSceneryPass::Opaque,
                fallbackTexture, view, {}, 0, true, environment.lighting, weatherBrightness,
                mainViewMinimumObjectViewportFraction_));
            static_cast<void>(openbus::app::vehicle::drawDiligentVehicleScenePass(
                vehicleDrawStates, renderer, fallbackTexture,
                openbus::rendering::DiligentVehicleRenderPass::Opaque));
            static_cast<void>(openbus::rendering::drawDiligentHumanPlacementScenePass(
                renderer, fallbackTexture, humanDrawStates,
                openbus::rendering::DiligentHumanRenderPass::Opaque));
            static_cast<void>(openbus::app::drawDiligentMapSceneryPlacements(
                renderer, sceneryUpdate.placements,
                openbus::rendering::DiligentSceneryPass::TransmapDepthPrepass, fallbackTexture,
                view, {}, 0, true, environment.lighting, weatherBrightness,
                mainViewMinimumObjectViewportFraction_));
            static_cast<void>(openbus::app::drawDiligentMapMainView(
                renderer, fallbackTexture, environment.lighting,
                openbus::app::DiligentMapMainViewPass::Transparent, &mapWeather));
            static_cast<void>(openbus::app::drawDiligentMapSceneryPlacements(
                renderer, sceneryUpdate.placements,
                openbus::rendering::DiligentSceneryPass::Transparent, fallbackTexture, view, {}, 0,
                true, environment.lighting, weatherBrightness,
                mainViewMinimumObjectViewportFraction_));
            static_cast<void>(openbus::app::vehicle::drawDiligentVehicleScenePass(
                vehicleDrawStates, renderer, fallbackTexture,
                openbus::rendering::DiligentVehicleRenderPass::Transparent));
            static_cast<void>(openbus::rendering::drawDiligentHumanPlacementScenePass(
                renderer, fallbackTexture, humanDrawStates,
                openbus::rendering::DiligentHumanRenderPass::Transparent));
            if (!diligentTicketVertices.empty()) {
                const auto viewProjection = openbus::rendering::multiplyMatrix4(projection, view);
                renderer.drawPrimitives(diligentTicketVertices, viewProjection, view);
            }
            if (!passengerCoinAnimations_.empty()) {
                constexpr std::size_t verticesPerCoin = 144;
                const double simulationTime = simulation.simulationTime();
                std::vector<openbus::rendering::PrimitiveVertex> coinVertices;
                coinVertices.reserve(passengerCoinAnimations_.size() * verticesPerCoin);
                for (const PassengerCoinAnimation& animation : passengerCoinAnimations_) {
                    const double age =
                        std::clamp(simulationTime - animation.startedAtSeconds, 0.0, 0.8);
                    const std::array<float, 3> color =
                        animation.coinIndex < passengerCurrency_.coins.size() &&
                                passengerCurrency_.coins[animation.coinIndex].value < 0.1
                            ? std::array<float, 3>{0.82F, 0.47F, 0.20F}
                            : std::array<float, 3>{0.92F, 0.76F, 0.28F};
                    std::vector<openbus::rendering::PrimitiveVertex> discVertices;
                    discVertices.reserve(verticesPerCoin);
                    openbus::app::appendPassengerCoinDisc(discVertices, 0.014, 0.0025, color);

                    const double angle = animation.headingRadians + age * 2.0 * std::numbers::pi;
                    const double cosine = std::cos(angle);
                    const double sine = std::sin(angle);
                    const double centerZ =
                        animation.position[2] + 0.025 + 0.035 * std::sin(age * 11.0) - 0.12 * age;
                    for (const auto& vertex : discVertices) {
                        coinVertices.push_back(
                            {static_cast<float>(animation.position[0] + cosine * vertex.x -
                                                sine * vertex.y),
                             static_cast<float>(animation.position[1] + sine * vertex.x +
                                                cosine * vertex.y),
                             static_cast<float>(centerZ + vertex.z), vertex.r, vertex.g, vertex.b});
                    }
                }
                const auto viewProjection = openbus::rendering::multiplyMatrix4(projection, view);
                renderer.drawPrimitives(coinVertices, viewProjection, view);
            }
            if (cameraView_ == 0 || freeCameraEnabled_) {
                const double precipitationType = sharedVariables.get("preciptype");
                if (!std::isfinite(precipitationType) || precipitationType < 0.0 ||
                    precipitationType > 2.0 || std::floor(precipitationType) != precipitationType) {
                    throw std::runtime_error("Weather system variable PrecipType is invalid");
                }
                static_cast<void>(renderer.drawWeatherPrecipitation(
                    static_cast<int>(precipitationType), sharedVariables.get("preciprate"),
                    simulation.simulationTime(), sharedVariables.weatherWindDirectionDegrees(),
                    sharedVariables.weatherWindSpeedMetersPerSecond(), weatherBrightness));
            }
            drawDiligentDebugOverlays(world, openbus::rendering::multiplyMatrix4(projection, view));
            if (imguiEnabled_) {
                drawOnFootUi();
                drawPlayerTimetableUi();
                drawPauseMenuUi();
                drawDebugMenuUi();
            }
        },
        diligentRenderer->defaultColorFilter(), diligentDebugView_);
    if (rendered && mapRenderer_ != nullptr) {
        passengerRenderedQueuePositions_.clear();
        for (const auto& [key, pose] : diligentWaitingPassengerRenderedPoses) {
            passengerRenderedQueuePositions_.insert_or_assign(key, pose);
        }
        mapRenderer_->finishFrame();
    }
#else
    static_cast<void>(simulation);
    throw std::logic_error("Diligent renderer support is not available in this build");
#endif
}

void RenderLoop::invalidateTemporalHistory() {
    if (framePresenter_ == nullptr) {
        return;
    }
#if defined(OPENBUS_DILIGENT_RENDERER_AVAILABLE)
    diligentMotionHistory_.invalidate();
    auto* diligentRenderer =
        dynamic_cast<openbus::rendering::DiligentFrameRenderer*>(framePresenter_.get());
    if (diligentRenderer == nullptr) {
        throw std::logic_error(
            "Diligent temporal invalidation requires the Diligent frame presenter");
    }
    diligentRenderer->invalidateHistory();
#else
    throw std::logic_error("Diligent renderer support is not available in this build");
#endif
}

void RenderLoop::beginUiFrame(bool manageMouseCursor) {
#if defined(OPENBUS_DILIGENT_RENDERER_AVAILABLE)
    auto* diligentRenderer =
        dynamic_cast<openbus::rendering::DiligentFrameRenderer*>(framePresenter_.get());
    if (diligentRenderer == nullptr) {
        throw std::logic_error("UI frame requires the Diligent frame presenter");
    }
    diligentRenderer->beginUiFrame(manageMouseCursor);
#else
    static_cast<void>(manageMouseCursor);
    throw std::logic_error("Diligent renderer support is not available in this build");
#endif
}

void RenderLoop::clearUiOnlyFrame(int framebufferWidth, int framebufferHeight) {
    static_cast<void>(framebufferWidth);
    static_cast<void>(framebufferHeight);
#if defined(OPENBUS_DILIGENT_RENDERER_AVAILABLE)
    if (framePresenter_ == nullptr || dynamic_cast<openbus::rendering::DiligentFrameRenderer*>(
                                          framePresenter_.get()) == nullptr) {
        throw std::logic_error("UI clearing requires the Diligent frame presenter");
    }
#else
    throw std::logic_error("Diligent renderer support is not available in this build");
#endif
}

void RenderLoop::presentUiOnlyFrame() {
#if defined(OPENBUS_DILIGENT_RENDERER_AVAILABLE)
    auto* diligentRenderer =
        dynamic_cast<openbus::rendering::DiligentFrameRenderer*>(framePresenter_.get());
    if (diligentRenderer == nullptr) {
        throw std::logic_error("UI presentation requires the Diligent frame presenter");
    }
    diligentRenderer->endUiOnlyFrame();
#else
    throw std::logic_error("Diligent renderer support is not available in this build");
#endif
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
