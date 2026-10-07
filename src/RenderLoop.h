#pragma once

#include "BusTypes.h"
#include "MapConfigLoader.h"
#include "CoreRenderer.h"
#include "SoundEngine.h"
#include "VehicleConfigLoader.h"
#include "Viewpoint.h"
#include "Variables.h"

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

struct GLFWwindow;
struct GLFWcursor;
struct Vehicle;
class BusSimulation;

namespace openbus::rendering {
class AssetRequestManager;
class MapRenderer;
class ReflectionRenderer;
} // namespace openbus::rendering

enum class AssetLoadingMode {
    Eager,
    Deferred,
};

struct ModelLoadingPolicy {
    AssetLoadingMode modelMode = AssetLoadingMode::Deferred;
    AssetLoadingMode textureMode = AssetLoadingMode::Deferred;
};

enum class RenderBenchmarkPhase {
    Baseline,
    CameraCycle,
    ThirdPersonZoom,
    DrivingControls,
    DashboardInteraction,
};

struct RenderBenchmarkInput {
    double throttle = 0.0;
    double steering = 0.0;
    double brake = 0.0;
};

class RenderLoop {
  public:
    RenderLoop(int width, int height, const char* title);
    ~RenderLoop();

    RenderLoop(const RenderLoop&) = delete;
    RenderLoop& operator=(const RenderLoop&) = delete;

    bool shouldClose() const;
    void requestClose();
    std::array<int, 2> windowSize() const;
    std::array<int, 2> framebufferSize() const;
    RenderBenchmarkInput setBenchmarkFrame(RenderBenchmarkPhase phase, int frameInPhase,
                                           int phaseFrameCount);
    bool benchmarkClickTargetFound() const;
    Vehicle* AddVehicle(const std::filesystem::path& busConfigPath,
                        const std::filesystem::path& modelConfigPath,
                        const VehiclePlacement& placement, ModelLoadingPolicy loadingPolicy = {});

    void SetPlayerVehicle(Vehicle* vehicle);
    void SetMap(const openbus::map::MapDefinition& map, std::size_t spawnEntryPointIndex,
                std::size_t groundTextureIndex, const std::filesystem::path& omsiRoot);

    void beginFrame(double fixedTimeStep = -1.0);
    void updatePlayerVariables(const BusSimulation& simulation, double throttle, double steering,
                               double brake);
    void updatePostPhysicsVariables(const BusSimulation& simulation);
    void draw(const BusSimulation& simulation);
    void endFrame();
    void captureViews(const BusSimulation& simulation, const std::filesystem::path& directory);
    bool consumeCaptureRequest();
    bool consumeRaisePlayerRequest();
    bool isCaptureReady() const;
    double throttle() const;
    double steering() const;
    double brake() const;
    double physicsThrottle() const;
    double physicsWheelTorque() const;
    double physicsSteering() const;
    std::vector<double> physicsWheelBrakeForces(std::size_t axleCount) const;
    std::vector<double> physicsAxleSpringFactors(std::size_t axleCount) const;
    std::vector<KeyEvent> consumeKeyEvents();

  private:
    static void scrollCallback(GLFWwindow* window, double xOffset, double yOffset);
    bool isExteriorView() const;
    const VehicleCamera* currentVehicleCamera() const;
    void selectVehicleCamera(int direction);
    void updateScripts();
    void logDiagnosticVariables() const;
    double currentFieldOfView() const;
    void renderReflectionViews(const BusSimulation& simulation);

    GLFWwindow* window_;
    double cameraYaw_ = -2.3;
    double cameraPitch_ = 0.45;
    double fieldOfViewOffset_ = 0.0;
    double viewLookYaw_ = 0.0;
    double viewLookPitch_ = 0.0;
    double lastStatsTitleTime_ = 0.0;
    double cameraDistance_ = 24.0;
    double previousVariableTime_ = 0.0;
    int cameraView_ = 0;
    double previousCursorX_ = 0.0;
    double previousCursorY_ = 0.0;
    bool draggingCamera_ = false;
    double previousFovCursorY_ = 0.0;
    bool draggingFov_ = false;
    bool mouseControlEnabled_ = false;
    double mouseThrottle_ = 0.0;
    double mouseSteering_ = 0.0;
    double mouseBrake_ = 0.0;
    std::vector<bool> previousVehicleKeyStates_;
    std::array<bool, 10> previousViewKeyStates_ = {};
    std::array<bool, 2> previousCameraNavigationStates_ = {};
    bool previousCaptureKeyState_ = false;
    bool previousRaisePlayerKeyState_ = false;
    bool raisePlayerRequestPending_ = false;
    bool previousLeftMouseState_ = false;
    bool leftMousePressed_ = false;
    bool previousClickableDebugKeyState_ = false;
    bool clickableDebugOverlay_ = false;
    bool previousCollisionDebugKeyState_ = false;
    bool collisionDebugOverlay_ = false;
    bool collisionWireframeBuilt_ = false;
    std::uint64_t collisionWireframeRevision_ = 0;
    bool clickableHoverCacheValid_ = false;
    bool clickableHoverCacheHit_ = false;
    double clickableHoverCacheX_ = 0.0;
    double clickableHoverCacheY_ = 0.0;
    int clickableHoverCacheWidth_ = 0;
    int clickableHoverCacheHeight_ = 0;
    int clickableHoverCacheContext_ = 0;
    int clickableHoverCacheCameraView_ = -1;
    double clickableHoverCacheCameraYaw_ = 0.0;
    double clickableHoverCacheCameraPitch_ = 0.0;
    double clickableHoverCacheLookYaw_ = 0.0;
    double clickableHoverCacheLookPitch_ = 0.0;
    std::uint64_t clickableHoverCacheRevision_ = 0;
    double clickableHoverCacheTimestamp_ = 0.0;
    bool renderingReflection_ = false;
    bool benchmarkClickDiscoveryPending_ = false;
    bool benchmarkClickDiscoveryComplete_ = false;
    bool benchmarkClickTargetFound_ = false;
    bool benchmarkClickQueued_ = false;
    double benchmarkClickFramebufferX_ = 0.0;
    double benchmarkClickFramebufferY_ = 0.0;
    bool hasPreviousVariableTime_ = false;
    bool captureRequested_ = false;
    bool captureMode_ = false;
    bool pendingMouseClick_ = false;
    double pendingMouseClickX_ = 0.0;
    double pendingMouseClickY_ = 0.0;
    std::string activeMouseEvent_;
    double previousMouseInteractionX_ = 0.0;
    double previousMouseInteractionY_ = 0.0;
    double frameTimeStep_ = 0.0;
    int framebufferWidth_ = 1;
    int framebufferHeight_ = 1;
    unsigned int coordinateHudTexture_ = 0;
    std::string coordinateHudText_;
    openbus::rendering::StaticPrimitiveBuffer collisionWireframeBuffer_;
    std::vector<openbus::rendering::PrimitiveVertex> collisionWireframeVertices_;
    std::array<int, 4> viewport_ = {0, 0, 1, 1};
    double scriptRateHz_ = 0.0;
    double scriptAccumulator_ = 0.0;
    double steeringSmoothingRate_ = 0.0;
    double smoothedSteering_ = 0.0;
    std::vector<KeyEvent> keyEvents_;
    std::vector<VehicleCamera> vehicleCameras_;
    SimulationState simulationState_;
    std::unique_ptr<openbus::rendering::AssetRequestManager> assetRequestManager_;
    std::unique_ptr<openbus::rendering::ReflectionRenderer> reflectionRenderer_;
    std::unique_ptr<openbus::rendering::MapRenderer> mapRenderer_;
    SoundEngine soundEngine_;
    openbus::rendering::ViewpointContext soundViewpoint_ =
        openbus::rendering::ViewpointContext::PlayerExterior;
    std::vector<std::unique_ptr<Vehicle>> vehicles_;
    Vehicle* playerVehicle_ = nullptr;
    GLFWcursor* clickableCursor_ = nullptr;
    GLFWcursor* mouseSteeringCursor_ = nullptr;
};
