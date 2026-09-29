#pragma once

#include "BusTypes.h"
#include "VehicleConfigLoader.h"
#include "Variables.h"

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <vector>

struct GLFWwindow;
struct Vehicle;
class BusSimulation;

namespace openbus::rendering {
class AssetRequestManager;
}

enum class AssetLoadingMode {
    Eager,
    Deferred,
};

struct ModelLoadingPolicy {
    AssetLoadingMode modelMode = AssetLoadingMode::Deferred;
    AssetLoadingMode textureMode = AssetLoadingMode::Deferred;
};

class Renderer {
  public:
    Renderer(int width, int height, const char* title);
    ~Renderer();

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    bool shouldClose() const;
    void requestClose();
    Vehicle* AddVehicle(BusVehicle vehicle, ModelLoadingPolicy loadingPolicy = {});
    Vehicle* AddVehicle(BusVehicle vehicle, const std::array<double, 3>& spawnPosition,
                        ModelLoadingPolicy loadingPolicy = {});

    Vehicle* AddBusModel(BusVehicle vehicle, ModelLoadingPolicy loadingPolicy = {}) {
        return AddVehicle(vehicle, loadingPolicy);
    }
    Vehicle* AddBusModel(BusVehicle vehicle, const std::array<double, 3>& spawnPosition,
                         ModelLoadingPolicy loadingPolicy = {}) {
        return AddVehicle(vehicle, spawnPosition, loadingPolicy);
    }

    void SetPlayerVehicle(Vehicle* vehicle);
    void SetPlayerBusModel(Vehicle* vehicle) {
        SetPlayerVehicle(vehicle);
    }

    void beginFrame();
    void updatePlayerVariables(const BusSimulation& simulation, double throttle, double steering,
                               double brake);
    void draw(const BusSimulation& simulation);
    void endFrame();
    void captureViews(const BusSimulation& simulation, const std::filesystem::path& directory);
    bool consumeCaptureRequest();
    bool isCaptureReady() const;
    double throttle() const;
    double steering() const;
    double brake() const;
    std::vector<KeyEvent> consumeKeyEvents();

  private:
    static void scrollCallback(GLFWwindow* window, double xOffset, double yOffset);
    bool isExteriorView() const;
    const VehicleCamera* currentVehicleCamera() const;
    void selectVehicleCamera(int direction);
    void updateScripts();
    double currentFieldOfView() const;
    void renderReflectionViews(const BusSimulation& simulation);
    void initializeReflectionTargets();
    void destroyReflectionTargets();

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
    std::array<bool, 4> previousKeyStates_ = {};
    std::array<bool, 10> previousViewKeyStates_ = {};
    std::array<bool, 2> previousCameraNavigationStates_ = {};
    bool previousCaptureKeyState_ = false;
    bool renderingReflection_ = false;
    bool hasPreviousVariableTime_ = false;
    bool captureRequested_ = false;
    bool captureMode_ = false;
    int reflectionSize_ = 256;
    int reflectionFrameInterval_ = 1;
    std::uint64_t reflectionFrameCounter_ = 0;
    double frameTimeStep_ = 0.0;
    double scriptRateHz_ = 0.0;
    double scriptAccumulator_ = 0.0;
    std::vector<KeyEvent> keyEvents_;
    std::vector<VehicleCamera> vehicleCameras_;
    struct ReflectionTarget {
        unsigned int framebuffer = 0;
        unsigned int texture = 0;
        unsigned int depthTexture = 0;
        int width = 0;
        int height = 0;
    };
    std::vector<ReflectionTarget> reflectionTargets_;
    SimulationState simulationState_;
    std::unique_ptr<openbus::rendering::AssetRequestManager> assetRequestManager_;
    std::vector<std::unique_ptr<Vehicle>> vehicles_;
    Vehicle* playerVehicle_ = nullptr;
};
