#pragma once

#include "BusTypes.h"
#include "SoundEngine.h"
#include "VehicleConfigLoader.h"
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

class RenderLoop {
  public:
    RenderLoop(int width, int height, const char* title);
    ~RenderLoop();

    RenderLoop(const RenderLoop&) = delete;
    RenderLoop& operator=(const RenderLoop&) = delete;

    bool shouldClose() const;
    void requestClose();
    Vehicle* AddVehicle(const std::filesystem::path& busConfigPath,
                        const std::filesystem::path& modelConfigPath,
                        const VehiclePlacement& placement, ModelLoadingPolicy loadingPolicy = {});

    void SetPlayerVehicle(Vehicle* vehicle);

    void beginFrame();
    void updatePlayerVariables(const BusSimulation& simulation, double throttle, double steering,
                               double brake);
    void updatePostPhysicsVariables(const BusSimulation& simulation);
    void draw(const BusSimulation& simulation);
    void endFrame();
    void captureViews(const BusSimulation& simulation, const std::filesystem::path& directory);
    bool consumeCaptureRequest();
    bool isCaptureReady() const;
    double throttle() const;
    double steering() const;
    double brake() const;
    double physicsThrottle() const;
    double physicsWheelTorque() const;
    double physicsSteering() const;
    double physicsBrake() const;
    std::vector<KeyEvent> consumeKeyEvents();

  private:
    static void scrollCallback(GLFWwindow* window, double xOffset, double yOffset);
    bool isExteriorView() const;
    const VehicleCamera* currentVehicleCamera() const;
    void selectVehicleCamera(int direction);
    void updateScripts();
    double currentFieldOfView() const;
    void renderReflectionViews(const BusSimulation& simulation);
    void renderReflectionDebugOverlay();

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
    std::vector<bool> previousVehicleKeyStates_;
    std::array<bool, 10> previousViewKeyStates_ = {};
    std::array<bool, 2> previousCameraNavigationStates_ = {};
    bool previousCaptureKeyState_ = false;
    bool previousLeftMouseState_ = false;
    bool leftMousePressed_ = false;
    bool previousReflectionDebugKeyState_ = false;
    bool previousCollisionDebugKeyState_ = false;
    bool previousClickableDebugKeyState_ = false;
    bool reflectionDebugOverlay_ = false;
    bool collisionDebugOverlay_ = false;
    bool clickableDebugOverlay_ = false;
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
    std::array<double, 3> clickableHoverCachePosition_ = {};
    std::array<double, 9> clickableHoverCacheRotation_ = {};
    std::uint64_t clickableHoverCacheRevision_ = 0;
    bool renderingReflection_ = false;
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
    double scriptRateHz_ = 0.0;
    double scriptAccumulator_ = 0.0;
    std::vector<KeyEvent> keyEvents_;
    std::vector<VehicleCamera> vehicleCameras_;
    SimulationState simulationState_;
    std::unique_ptr<openbus::rendering::AssetRequestManager> assetRequestManager_;
    std::unique_ptr<openbus::rendering::ReflectionRenderer> reflectionRenderer_;
    SoundEngine soundEngine_;
    std::vector<std::unique_ptr<Vehicle>> vehicles_;
    Vehicle* playerVehicle_ = nullptr;
    GLFWcursor* clickableCursor_ = nullptr;
};
