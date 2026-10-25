#pragma once

#include "core/BusTypes.h"
#include "render/CameraMath.h"
#include "map/MapConfigLoader.h"
#include "map/MapTrafficRuntime.h"
#include "input/MouseInteraction.h"
#include "render/CoreRenderer.h"
#include "render/HumanGait.h"
#include "audio/SoundEngine.h"
#include "config/VehicleConfigLoader.h"
#include "core/Viewpoint.h"
#include "core/Variables.h"
#include "physics/BusSimulation.h"

#include <array>
#include <cstdint>
#include <filesystem>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <unordered_map>
#include <vector>

struct GLFWwindow;
struct GLFWcursor;
class ImGuiLayer;
struct Vehicle;

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

struct VehicleSetupOptions {
    std::unordered_map<std::string, std::string> ctcTextureReplacements;
    std::string appearanceKey;
    std::optional<std::string> vehicleNumber;
    std::optional<std::string> registration;
    std::filesystem::path depotPath;
    std::string depotName;
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

struct StartupSelection {
    std::filesystem::path mapDirectory;
    openbus::map::MapDefinition mapDefinition;
    std::filesystem::path vehicleConfigPath;
    VehicleSetupOptions vehicleSetup;
    bool startInBus = false;
    std::size_t spawnEntryPoint = 0;
    int year = 0;
    int month = 0;
    int day = 0;
    double timeSeconds = 0.0;
};

class RenderLoop {
  public:
    RenderLoop(int width, int height, const char* title);
    ~RenderLoop();

    RenderLoop(const RenderLoop&) = delete;
    RenderLoop& operator=(const RenderLoop&) = delete;

    bool shouldClose() const;
    void updateOnFoot(const BusSimulation& simulation, double elapsedSeconds);
    std::array<double, 3> streamingPosition(const BusSimulation& simulation) const;
    void requestClose();
    std::array<int, 2> windowSize() const;
    std::array<int, 2> framebufferSize() const;
    RenderBenchmarkInput setBenchmarkFrame(RenderBenchmarkPhase phase, int frameInPhase,
                                           int phaseFrameCount);
    bool benchmarkClickTargetFound() const;
    Vehicle* AddVehicle(const std::filesystem::path& vehicleConfigPath,
                        const std::filesystem::path& modelConfigPath,
                        const VehiclePlacement& placement, ModelLoadingPolicy loadingPolicy = {},
                        bool isAiVehicle = false, VehicleSetupOptions setupOptions = {});
    bool AddTrafficVehicle(int splineId, int pathTypeCode,
                           const std::filesystem::path& busConfigPath,
                           std::optional<std::string> driverModelPath, double speedMetersPerSecond,
                           ModelLoadingPolicy loadingPolicy = {}, bool isBus = false,
                           std::size_t laneSelector = 0, std::size_t routeSeed = 1,
                           std::vector<double> scheduledDepartureMinutes = {},
                           std::optional<std::size_t> unscheduledGroupIndex = std::nullopt,
                           int defaultVehicleType = 0, std::vector<int> requiredRouteSplineIds = {},
                           std::vector<std::optional<std::size_t>> requiredSceneryPathSequence = {},
                           bool stopAtRequiredSequenceEnd = false,
                           bool allowIntermediarySegmentsBetweenRequired = false,
                           bool allowReverseTraversalForRequired = false,
                           std::vector<openbus::map::MapTrafficPassengerStop> passengerStops = {},
                           std::string timetableLineName = {},
                           std::string timetableDestinationName = {},
                           std::optional<std::size_t> trafficRuleGroupIndex = std::nullopt,
                           bool registerPassengerRouteOnly = false,
                           std::optional<std::size_t> passengerRouteIndex = std::nullopt,
                           std::size_t* registeredPassengerRouteIndex = nullptr);
    int trafficVehicleType(const std::filesystem::path& vehicleConfigPath,
                           int defaultVehicleType) const;
    double trafficMinimumTurnRadius(const std::filesystem::path& vehicleConfigPath,
                                    bool isBus) const;
    void SetMapCalendar(int year, int month, int day, double timeSeconds);
    void SetMapTrafficStartHour(double startHour);
    void SetMapTrafficSpawnInView(bool enabled);
    bool AddHumanTrafficActor(int splineId, const std::filesystem::path& humanConfigPath,
                              double feetDistanceMeters, double heightMeters,
                              double walkStrideLengthMeters, double speedMetersPerSecond,
                              double startDistanceMeters, double centerX, double centerY,
                              double maximumDistanceMeters);
    void SetScheduledPassengerHumanModels(std::vector<std::filesystem::path> humanConfigPaths);
    void SetMapTrafficPopulation(
        std::size_t roadVehicleCount, std::size_t busCount,
        std::vector<openbus::map::MapAiRoadTrafficGroupPopulation> roadGroupPopulations = {},
        bool useGroupRoadPopulation = false);
    void SetHumanTrafficPopulation(std::size_t count);
    std::vector<int> trafficSplineIds(int pathTypeCode) const;
    std::vector<openbus::map::MapTrafficLaneStart>
    trafficLanesForTimetableEntry(int splineId, std::optional<std::size_t> sceneryPathIndex) const;
    std::optional<openbus::map::MapTrafficLaneConnectionGap>
    trafficLaneConnectionGap(int fromSplineId, std::optional<std::size_t> fromPathIndex,
                             int toSplineId, std::optional<std::size_t> toPathIndex) const;
    std::vector<openbus::map::MapTrafficLaneStart> trafficEntryLanes() const;
    std::optional<openbus::map::MapTrafficLaneStart>
    selectTrafficEntryLane(openbus::map::MapTrafficVehicleFilter vehicle,
                           std::size_t selectionSeed) const;
    std::vector<openbus::map::MapTrafficLaneStart> trafficSpawnLanes() const;
    std::optional<openbus::map::MapTrafficLaneStart>
    selectTrafficSpawnLane(openbus::map::MapTrafficVehicleFilter vehicle,
                           std::size_t selectionSeed) const;
    std::vector<int> trafficRouteStartSplineIds(int pathTypeCode) const;
    std::vector<openbus::map::MapTrafficRouteRequirement> trafficRouteSegments(
        int splineId, int pathTypeCode, std::size_t laneSelector = 0,
        double minimumTurnRadiusMeters = 0.0, std::size_t routeSeed = 0,
        const std::vector<int>& requiredSplineSequence = {},
        openbus::map::MapTrafficVehicleFilter vehicle = {},
        const std::vector<std::optional<std::size_t>>& requiredSceneryPathSequence = {},
        bool stopAtRequiredSequenceEnd = false,
        bool allowIntermediarySegmentsBetweenRequired = false,
        bool allowReverseTraversalForRequired = false) const;
    std::size_t trafficVehicleCount() const;
    void SetWeather(int precipitationType, double precipitationRate);

    void SetPlayerVehicle(Vehicle* vehicle);
    bool runStartupMenu(const std::filesystem::path& omsiRoot, StartupSelection& selection);
    void SetNoBusSpawnPoint(const VehiclePlacement& placement);
    bool consumeSpawnBusRequest();
    void SetMap(const openbus::map::MapDefinition& map, std::size_t spawnEntryPointIndex,
                std::size_t groundTextureIndex, const std::filesystem::path& omsiRoot);
    void updateMapStreaming(double worldX, double worldY);
    void updateMapPoleStates(const std::vector<MapPoleState>& states);
    std::vector<std::pair<int, int>> loadedMapTiles() const;
    std::vector<std::pair<int, int>> activeMapTiles() const;

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

  private:
    static void scrollCallback(GLFWwindow* window, double xOffset, double yOffset);
    bool isExteriorView() const;
    const VehicleCamera* currentVehicleCamera() const;
    void selectVehicleCamera(int direction);
    void setFreeCameraEnabled(bool enabled);
    void updateScripts();
    void logDiagnosticVariables() const;
    void logDrivetrainDiagnosticVariables() const;
    double currentFieldOfView() const;
    void renderReflectionViews(const BusSimulation& simulation);
    void drawOnFootUi();

    GLFWwindow* window_;
    std::unique_ptr<ImGuiLayer> imguiLayer_;
    bool imguiEnabled_ = false;
    bool spawnBusRequestPending_ = false;
    double cameraYaw_ = -2.3;
    double cameraPitch_ = 0.45;
    double fieldOfViewOffset_ = 0.0;
    double viewLookYaw_ = 0.0;
    double viewLookPitch_ = 0.0;
    double lastStatsTitleTime_ = 0.0;
    double fpsSampleStartTime_ = 0.0;
    double displayedFps_ = 0.0;
    std::uint64_t fpsSampleFrameCount_ = 0;
    bool fpsSampleInitialized_ = false;
    double cameraDistance_ = 24.0;
    openbus::rendering::FreeCameraPose freeCameraPose_{};
    OnFootState onFootState_{};
    bool previousJumpKeyState_ = false;
    openbus::rendering::FreeCameraPose lastMainCameraPose_{};
    double freeCameraSpeed_ = 12.0;
    double previousVariableTime_ = 0.0;
    int cameraView_ = 0;
    double previousCursorX_ = 0.0;
    double previousCursorY_ = 0.0;
    bool draggingCamera_ = false;
    double previousFovCursorY_ = 0.0;
    bool draggingFov_ = false;
    bool freeCameraEnabled_ = false;
    bool mouseLookEnabled_ = false;
    bool lastMainCameraPoseValid_ = false;
    // Main-camera projection * view, used to admit traffic only at off-screen entries.
    openbus::rendering::Matrix4 trafficCameraViewProjection_ = {};
    std::vector<openbus::map::MapTrafficCameraView> trafficReflectionCameraViews_;
    // Main-view eye in vehicle-local space, used to aim viewer-following reflection cameras.
    std::array<double, 3> reflectionViewerEyeLocal_ = {};
    bool reflectionViewerEyeLocalValid_ = false;
    // Driver camera whose eye position the authored reflection directions are designed for.
    int reflectionReferenceCamera_ = -1;
    bool previousFreeCameraToggleState_ = false;
    bool previousProfileToggleState_ = false;
    bool previousMouseLookToggleState_ = false;
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
    bool previousTrafficSplineDebugKeyState_ = false;
    bool trafficSplineDebugOverlay_ = false;
    bool trafficSplineDebugBuilt_ = false;
    double trafficSplineDebugOriginX_ = 0.0;
    double trafficSplineDebugOriginY_ = 0.0;
    std::vector<std::pair<int, int>> trafficSplineDebugTiles_;
    std::uint64_t trafficSplineDebugRevision_ = 0;
    bool collisionWireframeBuilt_ = false;
    std::uint64_t collisionWireframeRevision_ = 0;
    double clickableHoverRefreshInterval_ = 0.075;
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
    bool mouseInteractionPending_ = false;
    bool pendingMouseClick_ = false;
    double pendingMouseClickX_ = 0.0;
    double pendingMouseClickY_ = 0.0;
    bool mouseDragDiagnosticLogged_ = false;
    openbus::input::MouseInteraction mouseInteraction_;
    double frameTimeStep_ = 0.0;
    double lastDrivetrainDiagnosticTime_ = -1.0;
    bool drivetrainDiagnosticsEnabled_ = false;
    double lastScriptDiagnosticTime_ = -1.0;
    int scriptTicksSinceDiagnostic_ = 0;
    int droppedScriptTicksSinceDiagnostic_ = 0;
    double droppedScriptTimeSinceDiagnostic_ = 0.0;
    int framebufferWidth_ = 1;
    int framebufferHeight_ = 1;
    openbus::rendering::StaticPrimitiveBuffer collisionWireframeBuffer_;
    std::vector<openbus::rendering::PrimitiveVertex> collisionWireframeVertices_;
    openbus::rendering::StaticPrimitiveBuffer trafficSplineDebugBuffer_;
    std::vector<openbus::rendering::PrimitiveVertex> trafficSplineDebugVertices_;
    openbus::rendering::StaticPrimitiveBuffer trafficBehaviorDebugBuffer_;
    std::uint64_t trafficBehaviorDebugRevision_ = 0;
    std::array<int, 4> viewport_ = {0, 0, 1, 1};
    double scriptRateHz_ = 0.0;
    double scriptAccumulator_ = 0.0;
    double steeringSmoothingRate_ = 0.0;
    bool steeringSmoothingOverride_ = false;
    double smoothedSteering_ = 0.0;
    std::vector<VehicleCamera> vehicleCameras_;
    SimulationState simulationState_;
    std::unique_ptr<openbus::rendering::AssetRequestManager> assetRequestManager_;
    std::unique_ptr<openbus::rendering::ReflectionRenderer> reflectionRenderer_;
    std::unique_ptr<openbus::rendering::MapRenderer> mapRenderer_;
    SoundEngine soundEngine_;
    openbus::rendering::ViewpointContext soundViewpoint_ =
        openbus::rendering::ViewpointContext::PlayerExterior;
    std::vector<std::unique_ptr<Vehicle>> vehicles_;
    struct TrafficVehicleState {
        Vehicle* vehicle = nullptr;
        std::future<std::unique_ptr<Vehicle>> pendingVehicle;
        openbus::map::MapTrafficActor actor;
        std::vector<openbus::map::MapTrafficSpawnCandidate> spawnCandidates;
        std::vector<double> scheduledDepartureMinutes;
        std::vector<openbus::map::MapTrafficPassengerStop> passengerStops;
        std::optional<std::size_t> passengerRouteIndex;
        std::string timetableLineName;
        std::string timetableDestinationName;
        std::size_t nextPassengerStopIndex = 0;
        bool passengerStopArrived = false;
        double nextScheduledDepartureElapsedSeconds = 0.0;
        double lastScheduledDepartureMinutes = 0.0;
        std::size_t nextScheduledDepartureIndex = 0;
        bool isBus = false;
        bool populationEnabled = false;
        bool vehicleLoadFailed = false;
        bool scheduledTripActive = false;
        unsigned lastScheduledAdmissionDiagnosticBlockerMask = 8;
        double lastScheduledAdmissionDiagnosticCameraDistanceMeters = -1.0;
        double lastScheduledProgressDiagnosticDistanceMeters = -1.0;
        bool deadlocked = false;
        bool deadlockRemovalCommitted = false;
        std::optional<std::size_t> activeSpawnCandidateIndex;
        std::vector<std::size_t> rejectedSpawnCandidateIndices;
        double spawnedAtDistanceMeters = 0.0;
        double wheelTravelDistanceMeters = 0.0;
        double previousWheelRouteDistanceMeters = 0.0;
        bool wheelRouteDistanceInitialized = false;
        bool movedSinceSpawn = false;
        double lastProgressDistanceMeters = 0.0;
        double stoppedSecondsSinceProgress = 0.0;
        bool stuckDiagnosticLogged = false;
    };
    struct PassengerStopRouteState {
        openbus::map::MapTrafficActor actor;
        std::vector<openbus::map::MapTrafficPassengerStop> stops;
    };
    std::vector<TrafficVehicleState> trafficVehicles_;
    std::vector<PassengerStopRouteState> passengerStopRoutes_;
    std::unordered_map<int, std::size_t> scheduledPassengerQueues_;
    std::unordered_map<int, std::size_t> scheduledPassengerInitialQueues_;
    std::size_t passengerServiceUpdateOffset_ = 0;
    std::optional<int> playerPassengerStopLatch_;
    std::optional<std::size_t> playerPassengerStopLatchRouteIndex_;
    std::optional<std::size_t> playerPassengerStopLatchIndex_;
    std::optional<int> activePlayerPassengerStop_;
    std::optional<std::size_t> activePlayerPassengerStopRouteIndex_;
    std::optional<std::size_t> activePlayerPassengerStopIndex_;
    std::size_t targetRoadTrafficCount_ = 0;
    std::vector<openbus::map::MapAiRoadTrafficGroupPopulation> targetRoadGroupPopulations_;
    bool useGroupRoadPopulation_ = false;
    std::size_t targetBusTrafficCount_ = 0;
    double mapTrafficStartHour_ = 12.0;
    bool mapTrafficSpawnInView_ = false;
    void finishPendingTrafficVehicleLoads();
    void updateTrafficDriving(const BusSimulation& simulation, double elapsedSeconds);
    struct HumanTrafficState {
        openbus::map::MapTrafficActor actor;
        double feetDistanceMeters = 0.2;
        double heightMeters = 1.7;
        double walkStrideLengthMeters = 1.4;
        openbus::rendering::HumanGaitState gait;
        bool populationEnabled = false;
    };
    std::vector<HumanTrafficState> humanTrafficActors_;
    std::vector<std::filesystem::path> scheduledPassengerHumanModels_;
    std::size_t targetHumanTrafficCount_ = 0;
    Vehicle* playerVehicle_ = nullptr;
    GLFWcursor* clickableCursor_ = nullptr;
    GLFWcursor* mouseSteeringCursor_ = nullptr;
};
