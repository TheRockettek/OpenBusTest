#pragma once

#include "core/BusTypes.h"
#include "app/FrameTiming.h"
#include "app/traffic/PassengerNetwork.h"
#include "app/traffic/PlayerTimetableController.h"
#include "app/traffic/TrafficFleet.h"
#include "render/CameraMath.h"
#include "map/MapConfigLoader.h"
#include "map/MapTrafficRuntime.h"
#include "input/MouseInteraction.h"
#include "render/CoreRenderer.h"
#include "render/HumanGait.h"
#include "audio/SoundEngine.h"
#include "config/VehicleConfigLoader.h"
#include "config/VehicleCatalog.h"
#include "app/vehicle/VehiclePreferences.h"
#include "core/Viewpoint.h"
#include "core/Variables.h"
#include "physics/BusSimulation.h"
#include "physics/World.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <future>
#include <limits>
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

namespace openbus::app {
class MapTrafficWorker;
class SteamPresence;
} // namespace openbus::app

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
    std::filesystem::path vehicleConfigurationPath;
    std::optional<double> initialOdometerMetres;
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
    std::filesystem::path initialSimulationBusConfigPath;
    std::vector<std::filesystem::path> simulationBusConfigCandidates;
    VehicleSetupOptions vehicleSetup;
    std::size_t spawnEntryPoint = 0;
    int year = 0;
    int month = 0;
    int day = 0;
    double timeSeconds = 0.0;
};

struct SpawnBusRequest {
    std::filesystem::path configurationPath;
    VehiclePlacement placement;
    VehicleSetupOptions setupOptions;
    bool spawnInBus = true;
    using TimetableSelection = openbus::app::PlayerTimetableSelection;
};

struct MapCalendarRequest {
    int year = 0;
    int month = 0;
    int day = 0;
    double timeSeconds = 0.0;
};

enum class MapHofSelectionMode { Automatic, None, Manual };

struct TrafficSplineDebugLabel {
    int splineId = 0;
    int pathTypeCode = 0;
    std::array<double, 3> position = {};
    bool hasBidirectionalPath = false;
};

class RenderLoop {
  public:
    RenderLoop(int width, int height, const char* title,
               openbus::app::SteamPresence& steamPresence);
    ~RenderLoop();

    RenderLoop(const RenderLoop&) = delete;
    RenderLoop& operator=(const RenderLoop&) = delete;

    bool shouldClose() const;
    void updateOnFoot(const World& world, double elapsedSeconds);
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
    bool AddTrafficVehicle(
        int splineId, int pathTypeCode, const std::filesystem::path& busConfigPath,
        std::optional<std::string> driverModelPath, double speedMetersPerSecond,
        ModelLoadingPolicy loadingPolicy = {}, bool isBus = false, std::size_t laneSelector = 0,
        std::size_t routeSeed = 1, std::vector<double> scheduledDepartureMinutes = {},
        std::optional<std::size_t> unscheduledGroupIndex = std::nullopt, int defaultVehicleType = 0,
        std::vector<int> requiredRouteSplineIds = {},
        std::vector<std::optional<std::size_t>> requiredSceneryPathSequence = {},
        bool stopAtRequiredSequenceEnd = false,
        bool allowIntermediarySegmentsBetweenRequired = false,
        bool allowReverseTraversalForRequired = false,
        std::vector<openbus::map::MapTrafficPassengerStop> passengerStops = {},
        std::string timetableLineName = {}, std::string timetableDestinationName = {},
        std::optional<std::size_t> trafficRuleGroupIndex = std::nullopt,
        bool registerPassengerRouteOnly = false,
        std::optional<std::size_t> passengerRouteIndex = std::nullopt,
        std::size_t* registeredPassengerRouteIndex = nullptr,
        bool requireActiveTileSpawnCandidate = false,
        std::vector<int> scheduledDepartureProfileIndices = {},
        std::shared_ptr<const std::vector<openbus::map::MapAiTimetableProfile>> timetableProfiles =
            {},
        std::string timetableKey = {}, std::string timetableTripName = {});
    void RegisterPlayerTimetableRoutes(const openbus::map::MapAiTrafficData& traffic,
                                       double scheduledHumanScale);
    std::optional<std::size_t> playerTimetableRouteIndex(const std::string& key) const;
    int trafficVehicleType(const std::filesystem::path& vehicleConfigPath,
                           int defaultVehicleType) const;
    double trafficMinimumTurnRadius(const std::filesystem::path& vehicleConfigPath,
                                    bool isBus) const;
    void SetMapCalendar(int year, int month, int day, double timeSeconds,
                        double simulationTime = 0.0);
    MapCalendarRequest currentMapCalendar() const;
    void ResetMapClockEpoch(double simulationTime);
    void ResetMapClockEpoch(double previousSimulationTime, double newSimulationTime);
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
    void SetWeather(int precipitationType, double precipitationRate, double temperatureCelsius,
                    double absoluteHumidityGramsPerCubicMeter);
    void SetAutoClutch(bool enabled);
    void SetSunAltitude(double altitudeDegrees);

    void SetPlayerVehicle(Vehicle* vehicle);
    void SetParkedPlayerBus(Vehicle* vehicle);
    void SetPlayerTimetableSelection(std::optional<SpawnBusRequest::TimetableSelection> selection);
    bool runStartupMenu(const std::filesystem::path& omsiRoot, StartupSelection& selection);
    void SetNoBusSpawnPoint(const VehiclePlacement& placement);
    std::optional<SpawnBusRequest> consumeSpawnBusRequest();
    bool consumeEnterMapBusRequest();
    bool consumeLeaveMapBusRequest();
    std::optional<VehiclePlacement> consumeMapBusTeleportRequest();
    bool consumeRemovePlayerBusRequest();
    std::optional<MapCalendarRequest> consumeMapCalendarRequest();
    bool consumeQuitToMenuRequest();
    void ResetForStartupMenu();
    void RemovePlayerVehicle();
    void SetMap(const openbus::map::MapDefinition& map, std::size_t spawnEntryPointIndex,
                std::size_t groundTextureIndex, const std::filesystem::path& omsiRoot,
                bool prepareInitialWindowAsync = false);
    void showStartupStatus(const std::string& status);
    void updateMapStreaming(double worldX, double worldY);
    bool initialMapWindowReady() const noexcept;
    void updateMapPoleStates(const std::vector<MapPoleState>& states);
    std::vector<std::pair<int, int>> loadedMapTiles() const;
    std::vector<std::pair<int, int>> activeMapTiles() const;

    void beginFrame(double fixedTimeStep = -1.0);
    bool isPaused() const noexcept {
        return paused_;
    }
    void updatePlayerVariables(const BusSimulation& simulation, double throttle, double steering,
                               double brake);
    void updatePostPhysicsVariables(BusSimulation& simulation);
    void draw(const BusSimulation& simulation, const World& world);
    void endFrame();
    void captureViews(const BusSimulation& simulation, const World& world,
                      const std::filesystem::path& directory);
    bool consumeCaptureRequest();
    bool consumeRaisePlayerRequest();
    bool isCaptureReady() const;
    void SetSpawnBusError(std::string error);
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
    void updateNonPlayerSoundListenerGain();
    const VehicleCamera* currentVehicleCamera() const;
    void selectVehicleCamera(int direction);
    bool selectDriverCameraView(int driverViewIndex);
    void setFreeCameraEnabled(bool enabled);
    void updateScripts();
    void logDiagnosticVariables() const;
    void logDrivetrainDiagnosticVariables() const;
    double currentFieldOfView() const;
    double mapTrafficElapsedSeconds(double simulationTime) const;
    std::optional<openbus::map::MapTrafficPose> passengerQueuePose(int stopId,
                                                                  std::size_t queueSlot) const;
    void rebaseMapTrafficClock(double previousSimulationTime, double newSimulationTime,
                               double newStartHour);
    void renderReflectionViews(const BusSimulation& simulation, const World& world);
    void drawOnFootUi();
    void drawPlayerTimetableUi();
    void drawPlayerTimetableSelectionUi();
    void drawPauseMenuUi();
    void drawDebugMenuUi();
    void setPausedState(bool paused);
    void applyPlayerTripAiPolicy(const openbus::map::MapAiTimetableCourseTrip& trip,
                                 bool additionalBus);
    void clearPlayerTripAiPolicy();
    void advancePlayerTimetableTrip();
    void loadMapVehicleOptions();
    void updateAutomaticMapHofSelection();
    bool mapBusIsNearby() const;
    void restoreMapVehiclePreference();
    void persistMapVehiclePreference();
    void persistPlayerOdometer(bool force = false);

    openbus::app::SteamPresence& steamPresence_;
    GLFWwindow* window_;
    bool vsyncEnabled_ = true;
    std::unique_ptr<ImGuiLayer> imguiLayer_;
    bool imguiEnabled_ = false;
    bool spawnBusRequestPending_ = false;
    bool enterMapBusRequestPending_ = false;
    bool leaveMapBusRequestPending_ = false;
    std::optional<VehiclePlacement> mapBusTeleportRequestPending_;
    bool removePlayerBusRequestPending_ = false;
    bool openSpawnPopup_ = false;
    bool openPlayerTimetablePopup_ = false;
    bool pauseMenuOpen_ = false;
    bool debugMenuOpen_ = false;
    bool debugCalendarInitialized_ = false;
    bool quitToMenuRequested_ = false;
    std::optional<MapCalendarRequest> mapCalendarRequestPending_;
    int debugCalendarYear_ = 0;
    int debugCalendarMonth_ = 1;
    int debugCalendarDay_ = 1;
    int debugCalendarHour_ = 12;
    int debugCalendarMinute_ = 0;
    std::string spawnBusError_;
    std::vector<openbus::config::VehicleCatalogEntry> vehicleCatalog_;
    std::filesystem::path vehiclesDirectory_;
    std::size_t selectedMapVehicleIndex_ = 0;
    std::array<char, 256> mapVehicleSearchInput_{};
    std::vector<openbus::map::MapEntryPoint> mapEntryPoints_;
    std::size_t selectedMapSpawnEntryPointIndex_ = 0;
    bool mapSpawnInBus_ = true;
    std::optional<std::size_t> loadedMapVehicleIndex_;
    std::optional<VehicleConfig> mapVehicleConfig_;
    std::vector<openbus::config::VehicleAppearance> mapVehicleAppearances_;
    std::vector<openbus::config::VehicleDepot> vehicleDepotCatalog_;
    std::size_t selectedMapAppearanceIndex_ = 0;
    MapHofSelectionMode mapHofSelectionMode_ = MapHofSelectionMode::Automatic;
    std::size_t selectedMapDepotIndex_ = 0;
    std::optional<std::size_t> autoMapDepotIndex_;
    std::size_t autoMapDepotMatchedRouteCount_ = 0;
    bool autoMapDepotEvaluated_ = false;
    std::size_t selectedMapRegistrationIndex_ = 0;
    std::array<char, 256> mapRegistrationInput_{};
    std::array<char, 256> mapVehicleNumberInput_{};
    std::string mapVehicleLoadError_;
    openbus::app::VehicleRegistrationPreferences vehicleRegistrationPreferences_;
    std::filesystem::path vehicleRegistrationPreferencesPath_ = "OpenBus_vehicle_preferences.txt";
    bool vehicleRegistrationPreferencesLoaded_ = true;
    std::string vehicleRegistrationPreferencesError_;
    openbus::app::VehicleOdometerPreferences vehicleOdometerPreferences_;
    openbus::app::VehicleOdometerMode vehicleOdometerMode_ =
        openbus::app::VehicleOdometerMode::Global;
    std::filesystem::path vehicleOdometerPreferencesPath_ = "OpenBus_odometer_preferences.txt";
    bool vehicleOdometerPreferencesLoaded_ = true;
    std::string vehicleOdometerPreferencesError_;
    std::chrono::steady_clock::time_point lastOdometerPreferenceSave_ =
        std::chrono::steady_clock::now();
    double cameraYaw_ = -2.3;
    double cameraPitch_ = 0.45;
    double fieldOfViewOffset_ = 0.0;
    double viewLookYaw_ = 0.0;
    double viewLookPitch_ = 0.0;
    double lastStatsTitleTime_ = 0.0;
    openbus::app::FrameTiming frameTiming_;
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
    bool previousPauseToggleState_ = false;
    bool previousPauseMenuToggleState_ = false;
    bool previousProfileToggleState_ = false;
    bool previousMouseLookToggleState_ = false;
    bool paused_ = false;
    double calendarElapsedSeconds_ = 0.0;
    double previousCalendarSimulationTime_ = 0.0;
    bool hasPreviousCalendarSimulationTime_ = false;
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
    bool previousTileBoundaryDebugKeyState_ = false;
    bool tileBoundaryDebugOverlay_ = false;
    bool tileBoundaryDebugBuilt_ = false;
    std::uint64_t tileBoundaryDebugRevision_ = 0;
    std::vector<std::pair<int, int>> tileBoundaryDebugTiles_;
    bool previousTrafficSplineDebugKeyState_ = false;
    bool trafficSplineDebugOverlay_ = false;
    Vehicle* trafficDebugHotVehicle_ = nullptr;
    double trafficDebugHotHoldSeconds_ = 0.0;
    bool trafficSplineDebugBuilt_ = false;
    std::uint64_t trafficSplineDebugMapRevision_ = 0;
    double trafficSplineDebugOriginX_ = 0.0;
    double trafficSplineDebugOriginY_ = 0.0;
    std::vector<std::pair<int, int>> trafficSplineDebugTiles_;
    std::vector<TrafficSplineDebugLabel> trafficSplineDebugLabels_;
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
    double mainViewMinimumObjectViewportFraction_ = 0.0;
    double reflectionMinimumObjectViewportFraction_ = 0.0;
    openbus::rendering::StaticPrimitiveBuffer collisionWireframeBuffer_;
    std::vector<openbus::rendering::PrimitiveVertex> collisionWireframeVertices_;
    openbus::rendering::StaticPrimitiveBuffer collisionSurfaceWireframeBuffer_;
    std::vector<openbus::rendering::PrimitiveVertex> collisionSurfaceWireframeVertices_;
    openbus::rendering::StaticPrimitiveBuffer tileBoundaryDebugBuffer_;
    std::vector<openbus::rendering::PrimitiveVertex> tileBoundaryDebugVertices_;
    openbus::rendering::StaticPrimitiveBuffer trafficSplineDebugBuffer_;
    std::vector<openbus::rendering::PrimitiveVertex> trafficSplineDebugVertices_;
    openbus::rendering::StaticPrimitiveBuffer trafficBehaviorDebugBuffer_;
    std::uint64_t trafficBehaviorDebugRevision_ = 0;
    std::array<int, 4> viewport_ = {0, 0, 1, 1};
    double scriptRateHz_ = 0.0;
    double scriptAccumulator_ = 0.0;
    double aiScriptAccumulator_ = 0.0;
    double steeringSmoothingRate_ = 0.0;
    bool steeringSmoothingOverride_ = false;
    double smoothedSteering_ = 0.0;
    std::vector<VehicleCamera> vehicleCameras_;
    std::vector<VehicleViewEntry> vehicleScheduleViews_;
    std::vector<VehicleViewEntry> vehicleTicketSellingViews_;
    SimulationState simulationState_;
    std::unique_ptr<openbus::rendering::AssetRequestManager> assetRequestManager_;
    std::unique_ptr<openbus::rendering::ReflectionRenderer> reflectionRenderer_;
    std::unique_ptr<openbus::rendering::MapRenderer> mapRenderer_;
    SoundEngine soundEngine_;
    openbus::rendering::ViewpointContext soundViewpoint_ =
        openbus::rendering::ViewpointContext::PlayerExterior;
    std::vector<std::unique_ptr<Vehicle>> vehicles_;
    using TrafficVehicleState = openbus::app::TrafficVehicleState;
    using PassengerStopRouteState = openbus::app::PassengerStopRouteState;
    openbus::app::TrafficFleet trafficFleet_;
    openbus::app::PassengerNetwork passengerNetwork_;
    std::vector<openbus::map::MapAiTimetableCourse> passengerTimetableCourses_;
    openbus::app::PlayerTimetableCatalog playerTimetableCatalog_;
    std::vector<std::string> playerTimetableMapKeys_;
    std::string playerTimetableMapPath_;
    std::string selectedPlayerLineName_;
    std::size_t selectedPlayerCourseIndex_ = 0;
    std::size_t selectedPlayerCourseTripIndex_ = 0;
    std::size_t selectedPlayerFirstStopIndex_ = 0;
    bool selectedPlayerDriveAsAdditionalBus_ = false;
    std::optional<SpawnBusRequest::TimetableSelection> selectedPlayerTimetableSelection_;
    using PlayerTimetableState = openbus::app::PlayerTimetableState;
    openbus::app::PlayerTimetableController playerTimetable_;
    bool playerPassengerServiceEnabled_ = true;
    std::map<int, std::vector<openbus::map::MapTrafficPose>> authoredPassengerPositions_;
    std::map<int, std::size_t> authoredPassengerQueueCounts_;
    std::size_t authoredPassengerObjectCount_ = 0;
    std::unordered_set<int> reportedPassengerAreaStops_;
    std::optional<int> playerPassengerStopLatch_;
    std::optional<std::size_t> playerPassengerStopLatchRouteIndex_;
    std::optional<std::size_t> playerPassengerStopLatchIndex_;
    std::optional<int> activePlayerPassengerStop_;
    std::optional<std::size_t> activePlayerPassengerStopRouteIndex_;
    std::optional<std::size_t> activePlayerPassengerStopIndex_;
    double playerPassengerApproachSeconds_ = 0.0;
    std::optional<std::size_t> playerPassengerServiceRouteIndex_;
    std::size_t playerPassengerServiceNextStopIndex_ = 0;
    std::optional<std::array<double, 2>> playerTimetableStopVisitLatchPosition_;
    std::uint64_t playerPassengerServiceGeneration_ = 0;
    std::uint64_t activePlayerPassengerStopServiceGeneration_ = 0;
    void publishTrafficTimetableData(Vehicle& vehicle, const TrafficVehicleState& state,
                                     std::size_t departureIndex) const;
    void finishPendingTrafficVehicleLoads();
    void updateTrafficDriving(const BusSimulation& simulation, double elapsedSeconds);
    struct HumanTrafficState {
        openbus::map::MapTrafficActor actor;
        std::shared_ptr<const openbus::map::MapTrafficActor> workerActorTemplate;
        double feetDistanceMeters = 0.2;
        double heightMeters = 1.7;
        double walkStrideLengthMeters = 1.4;
        openbus::rendering::HumanGaitState gait;
        bool populationEnabled = false;
        bool advanceWhileInactive = false;
        std::uint64_t workerGeneration = 0;
    };
    std::vector<HumanTrafficState> humanTrafficActors_;
    std::vector<std::filesystem::path> scheduledPassengerHumanModels_;
    std::size_t targetHumanTrafficCount_ = 0;
    Vehicle* playerVehicle_ = nullptr;
    Vehicle* mapBusVehicle_ = nullptr;
    GLFWcursor* clickableCursor_ = nullptr;
    GLFWcursor* mouseSteeringCursor_ = nullptr;
};
