#pragma once

#include "app/DiligentGraphicsPreferences.h"
#include "core/WeatherState.h"
#include "render/diligent/DiligentColorPresent.h"
#include "render/diligent/DiligentHumanAsset.h"
#include "render/diligent/DiligentMapLighting.h"
#include "render/diligent/DiligentMultisampling.h"
#include "render/diligent/DiligentReflectionTarget.h"
#include "render/diligent/DiligentSceneLighting.h"
#include "render/diligent/DiligentSceneFrame.h"
#include "render/diligent/DiligentSpotlightShadowMaps.h"
#include "render/diligent/DiligentVehicleAsset.h"
#include "render/diligent/DiligentVulkanFrame.h"
#include "render/PrimitiveVertex.h"
#include "render/RendererFramePresenter.h"
#include "render/VehicleSpotlightData.h"

#include <Graphics/GraphicsEngine/interface/SwapChain.h>
#include <Graphics/GraphicsEngineVulkan/interface/EngineFactoryVk.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

struct GLFWwindow;
class Variables;
struct Vehicle;

namespace openbus::map {
struct MapDefinition;
class MapRoadConnectionIndex;
} // namespace openbus::map

namespace openbus::app::vehicle {
struct DiligentVehicleMaterialOverrideSet;
}

namespace openbus::app {
struct DiligentCameraView;
}

namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
struct ITextureView;
} // namespace Diligent

class ScriptRuntime;

namespace openbus::rendering {

class DiligentSceneryAsset;
class DiligentSceneryAssetCache;
class DiligentTexturedMeshRenderer;
struct DiligentSceneryDrawResult;
class DiligentHumanPoseHistory;
class DiligentTexture2D;
class DiligentMapTileSet;
struct DiligentMapWeather;
struct DiligentSceneryBatch;
struct DiligentSceneryOpaqueDraw;
struct DiligentSceneryTreeBatch;
enum class DiligentMapRoadPass : std::uint8_t;
enum class DiligentSceneryPass : std::uint8_t;

class DiligentFrameRenderer final : public RendererFramePresenter {
  public:
    using ReflectionDrawCallback =
        std::function<void(DiligentReflectionTarget&, DiligentTexturedMeshRenderer&)>;

    DiligentFrameRenderer(GLFWwindow& window, const Diligent::NativeWindow& nativeWindow,
                          const Diligent::SwapChainDesc& swapChainDesc,
                          bool includeLinearDepth = true, DiligentMultisampling multisampling = {},
                          bool generateTextureMipmaps = false);
    DiligentFrameRenderer(GLFWwindow& window, const Diligent::NativeWindow& nativeWindow,
                          const Diligent::SwapChainDesc& swapChainDesc,
                          const openbus::app::DiligentGraphicsOptions& graphicsOptions,
                          bool includeLinearDepth = true);
    ~DiligentFrameRenderer();
    DiligentFrameRenderer(const DiligentFrameRenderer&) = delete;
    DiligentFrameRenderer& operator=(const DiligentFrameRenderer&) = delete;

    [[nodiscard]] RendererBackend backend() const noexcept override {
        return RendererBackend::Diligent;
    }
    [[nodiscard]] Diligent::IRenderDevice& device() const noexcept;
    [[nodiscard]] Diligent::IDeviceContext& immediateContext() const noexcept;
    [[nodiscard]] Diligent::ITextureView& fallbackTexture() const;
    [[nodiscard]] Diligent::Uint32 maximumAnisotropy() const noexcept;
    // Reconfiguration is outside scene submission; a changed configuration invalidates prior
    // pointers and views returned by this host.
    DiligentSpotlightShadowMaps& configureSpotlightShadowMaps(Diligent::Uint32 resolution,
                                                              Diligent::Uint32 capacity);
    [[nodiscard]] DiligentSpotlightShadowMaps* spotlightShadowMaps() noexcept;
    [[nodiscard]] const DiligentSpotlightShadowMaps* spotlightShadowMaps() const noexcept;
    // Configure reflection targets outside scene submission. Replacing the set waits for
    // immediate-context work; an unchanged size list preserves existing target contents.
    void configureReflectionTargets(std::span<const Diligent::Uint32> sizes,
                                    std::uint64_t frameInterval = 1, double maximumFrameRate = 0.0);
    [[nodiscard]] const DiligentReflectionTarget& reflectionTarget(std::size_t index) const;
    // Returns false when not needed or throttled. Successful draws restore the active scene
    // attachments and viewport. The callback supplies the reflection camera and scene geometry.
    bool renderReflectionTarget(std::size_t index, std::uint64_t frameId, bool needed,
                                const ReflectionDrawCallback& draw,
                                const std::array<float, 4>& clearColor = {0, 0, 0, 1},
                                bool forceRefresh = false);
    [[nodiscard]] DiligentTexturedMeshRenderer& meshes();
    void drawPrimitives(std::span<const PrimitiveVertex> vertices,
                        const Matrix4& modelViewProjection, const Matrix4& modelView);
    std::size_t drawOpaqueSpotlightShadowLayer(Diligent::Uint32 lightIndex,
                                               std::span<const DiligentShadowCaster> casters);
    bool drawWeatherClouds(std::optional<WeatherCloudType> type,
                           std::optional<double> baseAboveGroundMeters,
                           std::optional<double> windDirectionDegrees,
                           std::optional<double> windSpeedMetersPerSecond,
                           std::optional<double> brightness, double groundHeightMeters,
                           double simulationSeconds);
    bool drawWeatherCloudsInReflection(
        const Matrix4& view, const Matrix4& projection, const DiligentWeatherFog& fog,
        std::optional<WeatherCloudType> type, std::optional<double> baseAboveGroundMeters,
        std::optional<double> windDirectionDegrees, std::optional<double> windSpeedMetersPerSecond,
        std::optional<double> brightness, double groundHeightMeters, double simulationSeconds);
    bool drawWeatherPrecipitation(int type, double rate, double simulationSeconds,
                                  std::optional<double> windDirectionDegrees,
                                  std::optional<double> windSpeedMetersPerSecond,
                                  std::optional<double> brightness);
    [[nodiscard]] DiligentHumanPoseHistory& humanPoseHistory();
    void setMapLights(std::span<const MapPointLight> activeLights, const Matrix4& view,
                      const std::array<double, 3>& cameraWorldPosition, double strength,
                      std::size_t maximumLights);
    void setSpotLights(std::span<const DiligentSpotLight> activeLights);
    // Applies caller-scheduled shadow cameras/layers, then validates and uploads the lights.
    void setSpotLightsWithShadows(std::span<const DiligentSpotLight> activeLights,
                                  std::span<const DiligentSpotlightShadowBinding> shadowBindings,
                                  const Matrix4& cameraView, float minimumNdcZ);
    void setVehicleSpotLights(std::span<const VehicleSpotlight> viewSpaceLights,
                              std::size_t maximumLights);
    // Upload already-selected view-space vehicle spotlights with their aligned scheduled shadow
    // cameras/layers. Shadow bindings must match the selected count after maximumLights is applied.
    void
    setVehicleSpotLightsWithShadows(std::span<const VehicleSpotlight> viewSpaceLights,
                                    std::span<const DiligentSpotlightShadowBinding> shadowBindings,
                                    const Matrix4& cameraView, std::size_t maximumLights,
                                    float minimumNdcZ);
    bool requestSceneryAsset(const std::filesystem::path& configPath,
                             const std::filesystem::path& omsiRoot);
    bool requestSeasonalSceneryTexture(const std::filesystem::path& texturePath);
    [[nodiscard]] const DiligentSceneryAsset*
    sceneryAsset(const std::filesystem::path& configPath,
                 const std::filesystem::path& omsiRoot) const;
    [[nodiscard]] const DiligentTexture2D*
    seasonalSceneryTexture(const std::filesystem::path& texturePath) const;
    bool
    synchronizeMapSceneryRuntimeTextures(std::uint64_t motionObjectId,
                                         const DiligentSceneryAsset& asset, ScriptRuntime* scripts,
                                         std::vector<Diligent::ITextureView*>& dynamicBatchTextures,
                                         std::vector<Diligent::ITextureView*>& textBatchTextures);
    std::size_t retainMapSceneryRuntimeTextures(std::span<const std::uint64_t> motionObjectIds);
    std::size_t publishReadySceneryAssets();
    // Evicts scenery assets/textures not needed by the next streamed scene window.
    // Call outside scene submission and do not retain returned asset pointers across this call.
    std::size_t retainSceneryAssets(
        std::span<const std::pair<std::filesystem::path, std::filesystem::path>> assets,
        std::span<const std::filesystem::path> seasonalTextures);
    std::size_t requestMapTileWindow(
        const openbus::map::MapDefinition& map, double worldX, double worldY, int radius,
        const std::filesystem::path& omsiRoot, std::size_t baseGroundTextureIndex,
        std::shared_ptr<const openbus::map::MapRoadConnectionIndex> roadConnections = {});
    template <typename MapSource>
    std::size_t requestMapTileWindow(const MapSource& source, double worldX, double worldY,
                                     int radius) {
        auto roadConnections = source.roadConnectionIndex();
        if (!roadConnections) {
            return 0;
        }
        return requestMapTileWindow(source.mapDefinition(), worldX, worldY, radius,
                                    source.omsiRoot(), source.baseGroundTextureIndex(),
                                    std::move(roadConnections));
    }
    std::size_t publishReadyMapTiles();
    std::size_t removeInactiveMapTiles();
    [[nodiscard]] const DiligentMapTileSet& mapTiles() const;
    std::size_t drawMapTerrain(Diligent::ITextureView& fallbackTexture,
                               const Matrix4& viewProjection,
                               const DiligentMaterialState* materialOverride = nullptr,
                               const DiligentLightingParameters& lighting = {},
                               const DiligentMotionParameters* motion = nullptr,
                               const DiligentMapWeather* weather = nullptr);
    // Managed MotionHistory generates tile-local motion; raw frame submissions may pass motion.
    std::size_t drawMapWater(Diligent::ITextureView& fallbackTexture, const Matrix4& viewProjection,
                             const DiligentMaterialState* materialOverride = nullptr,
                             const DiligentLightingParameters& lighting = {},
                             const DiligentMotionParameters* motion = nullptr);
    std::size_t drawMapRoads(DiligentMapRoadPass pass, Diligent::ITextureView& fallbackTexture,
                             const Matrix4& viewProjection,
                             const DiligentMaterialState* materialOverride = nullptr,
                             const DiligentLightingParameters& lighting = {},
                             const DiligentMotionParameters* motion = nullptr,
                             const DiligentMapWeather* weather = nullptr);
    bool drawSceneryBatch(const DiligentSceneryBatch& batch, DiligentSceneryPass pass,
                          Diligent::ITextureView& fallbackTexture,
                          const Matrix4& modelViewProjection, const DiligentSceneryOpaqueDraw& draw,
                          const DiligentLightingParameters& lighting = {},
                          const DiligentMotionParameters* motion = nullptr);
    bool drawSceneryTreeBatch(const DiligentSceneryTreeBatch& batch,
                              Diligent::ITextureView& fallbackTexture,
                              const Matrix4& modelViewProjection,
                              const DiligentSceneryOpaqueDraw& draw,
                              const DiligentLightingParameters& lighting = {},
                              const DiligentMotionParameters* motion = nullptr);
    DiligentSceneryDrawResult drawSceneryAsset(
        const DiligentSceneryAsset& asset, DiligentSceneryPass pass,
        Diligent::ITextureView& fallbackTexture, const Matrix4& modelViewProjection,
        const Matrix4& modelView, std::span<const DiligentSceneryOpaqueDraw> batchDraws,
        std::span<const DiligentSceneryOpaqueDraw> treeDraws, MotionObjectKey motionKey = {},
        bool geometryHistoryValid = true, const DiligentLightingParameters& lighting = {},
        const ::Variables* variables = nullptr);
    DiligentSceneryDrawResult drawSceneryPlacement(
        const DiligentSceneryAsset& asset, const Matrix4& worldTransform, double boundingRadius,
        DiligentSceneryPass pass, Diligent::ITextureView& fallbackTexture,
        std::span<const DiligentSceneryOpaqueDraw> batchDraws,
        std::span<const DiligentSceneryOpaqueDraw> treeDraws, std::uint64_t motionObjectId,
        std::uint64_t viewId = 0, bool geometryHistoryValid = true,
        const DiligentLightingParameters& lighting = {}, const ::Variables* variables = nullptr);
    [[nodiscard]] bool isSceneryPlacementVisible(const Matrix4& worldTransform,
                                                 double boundingRadius) const;
    [[nodiscard]] bool isSphereVisibleInView(const Matrix4& modelView,
                                             const std::array<double, 3>& localCenter,
                                             double localRadius) const;
    void drawVehicle(const DiligentVehicleAsset& asset, Diligent::ITextureView& fallbackTexture,
                     const Matrix4& modelViewProjection, const Matrix4& modelView,
                     const DiligentLightingParameters& lighting = {},
                     const DiligentMotionParameters* motion = nullptr,
                     std::span<const DiligentVehicleMaterialOverride> materialOverrides = {},
                     Diligent::ITextureView* normalMap = nullptr,
                     std::span<const DiligentVehicleGroupTransform> groupTransforms = {});
    [[nodiscard]] std::size_t
    drawVehiclePass(const DiligentVehicleAsset& asset, Diligent::ITextureView& fallbackTexture,
                    const Matrix4& modelViewProjection, const Matrix4& modelView,
                    DiligentVehicleRenderPass pass, const DiligentLightingParameters& lighting = {},
                    const DiligentMotionParameters* motion = nullptr,
                    std::span<const DiligentVehicleMaterialOverride> materialOverrides = {},
                    Diligent::ITextureView* normalMap = nullptr,
                    std::span<const DiligentVehicleGroupTransform> groupTransforms = {});
    // Prepare GPU assets outside an active scene. An empty source span reuses a current cache
    // entry; otherwise it uploads the caller-prepared batch sources.
    void prepareVehicleAsset(std::uint64_t vehicleObjectId, std::uint64_t assetGeneration,
                             std::uint64_t materialSelectionGeneration,
                             std::span<const DiligentVehicleBatchSource> batches);
    [[nodiscard]] bool vehicleAssetCurrent(std::uint64_t vehicleObjectId,
                                           std::uint64_t assetGeneration) const noexcept;
    [[nodiscard]] const DiligentVehicleAsset*
    vehicleAsset(std::uint64_t vehicleObjectId) const noexcept;
    // Evict assets not present in the next live vehicle set. Call outside scene submission.
    std::size_t retainVehicleAssets(std::span<const std::uint64_t> vehicleObjectIds);
    // Synchronize script/text textures and build current material-state overrides in an active
    // frame. The output owns the material states referenced by its override entries.
    bool buildVehicleMaterialOverrides(
        Vehicle& vehicle, const Matrix4& vehicleRootModelView,
        openbus::app::vehicle::DiligentVehicleMaterialOverrideSet& overrides);
    [[nodiscard]] DiligentVehicleGroupTransform makeVehicleGroupTransform(
        std::size_t animationGroupIndex, std::uint64_t vehicleObjectId,
        const Matrix4& currentModelView, bool geometryHistoryValid = true, bool visible = true,
        double maximumTranslationDeltaMeters = std::numeric_limits<double>::infinity());
    void drawHuman(const DiligentHumanSubmesh& submesh, Diligent::ITextureView& fallbackTexture,
                   const Matrix4& modelViewProjection, const HumanBonePalette& currentPose,
                   const HumanBonePalette* previousPose, bool seated,
                   const DiligentLightingParameters& lighting = {},
                   const DiligentMotionParameters* motion = nullptr);
    void drawHumanAsset(const DiligentHumanAsset& asset, Diligent::ITextureView& fallbackTexture,
                        const Matrix4& modelViewProjection, const Matrix4& modelView,
                        const HumanBonePalette& currentPose, const HumanBonePalette* previousPose,
                        bool seated, const DiligentLightingParameters& lighting = {},
                        const DiligentMotionParameters* motion = nullptr);
    [[nodiscard]] std::size_t
    drawHumanAssetPass(const DiligentHumanAsset& asset, Diligent::ITextureView& fallbackTexture,
                       const Matrix4& modelViewProjection, const Matrix4& modelView,
                       const HumanBonePalette& currentPose, const HumanBonePalette* previousPose,
                       bool seated, DiligentHumanRenderPass pass,
                       const DiligentLightingParameters& lighting = {},
                       const DiligentMotionParameters* motion = nullptr);
    // Human assets are shared by instances using the same model key and source generation.
    // Upload or rebuild them outside scene submission.
    void prepareHumanAsset(std::string_view modelKey, std::uint64_t sourceGeneration,
                           const HumanMeshGeometry& geometry,
                           const std::unordered_map<int, std::shared_ptr<const Image>>& textures,
                           const std::unordered_map<int, DiligentMaterialState>& materials = {});
    [[nodiscard]] bool humanAssetCurrent(std::string_view modelKey,
                                         std::uint64_t sourceGeneration) const;
    [[nodiscard]] const DiligentHumanAsset* humanAsset(std::string_view modelKey) const;
    std::size_t retainHumanAssets(std::span<const std::string> modelKeys);
    [[nodiscard]] DiligentHumanDrawResult drawHumanPlacement(
        const DiligentHumanAsset& asset, const Matrix4& worldTransform, double boundingRadius,
        Diligent::ITextureView& fallbackTexture, DiligentHumanPoseKey poseKey,
        MotionObjectKey motionKey, const HumanBonePalette& currentPose, bool seated,
        const DiligentLightingParameters& lighting = {}, bool geometryHistoryValid = true,
        bool visible = true, DiligentHumanRenderPass pass = DiligentHumanRenderPass::All);
    [[nodiscard]] DiligentHumanDrawResult drawHumanAssetWithHistory(
        const DiligentHumanAsset& asset, Diligent::ITextureView& fallbackTexture,
        const Matrix4& modelViewProjection, const Matrix4& modelView, DiligentHumanPoseKey poseKey,
        MotionObjectKey motionKey, const HumanBonePalette& currentPose, bool seated,
        const DiligentLightingParameters& lighting = {}, bool geometryHistoryValid = true,
        DiligentHumanRenderPass pass = DiligentHumanRenderPass::All);
    [[nodiscard]] DiligentMotionRenderTargets& targets() noexcept;
    [[nodiscard]] DiligentColorFilter defaultColorFilter() const noexcept;
    // Updates the next frame's composition filter. The scene's MSAA sample count is fixed when
    // the renderer is created, so a newly requested MSAA mode falls back to FXAA until restart.
    void setDefaultColorFilter(DiligentColorFilter filter, bool multisamplingRequested);
    [[nodiscard]] const MotionDataFrame& frameData() const;
    // Returns the primary view environment supplied at frame start; auxiliary views should
    // build and pass their own lighting to their draw adapters.
    [[nodiscard]] const DiligentSceneEnvironment& sceneEnvironment() const;
    [[nodiscard]] bool isSuspended() const noexcept override;

    // Resizes the swap chain and scene attachments together. Zero dimensions suspend rendering.
    void resize(Diligent::Uint32 width, Diligent::Uint32 height) override;
    // Call before beginFrame after a camera cut or other discontinuity.
    void invalidateHistory();
    void setVerticalSync(bool enabled) noexcept override;
    void waitForIdle() override;
    void present() override;
    // Discard the active scene after a caller-side submission failure and invalidate temporal
    // history so partially rendered state cannot be committed by the next frame.
    void abortFrame() noexcept;

    // Returns false while the swap chain is suspended; callers must skip scene submission then.
    bool beginFrame(const MotionDataFrame& frame,
                    const std::array<float, 4>& clearColor = {0, 0, 0, 1});
    bool beginFrame(const MotionDataFrame& frame, const DiligentSceneEnvironment& environment);
    // Starts and commits MotionHistory around presentation. The referenced history must outlive
    // the active frame; host invalidation is applied before the next history frame begins.
    bool beginFrame(MotionHistory& history, std::uint64_t frameId, double timestampSeconds,
                    const openbus::app::DiligentCameraView& camera, int displayWidth = 0,
                    int displayHeight = 0, const std::array<float, 4>& clearColor = {0, 0, 0, 1});
    bool beginFrame(MotionHistory& history, std::uint64_t frameId, double timestampSeconds,
                    const openbus::app::DiligentCameraView& camera, int displayWidth,
                    int displayHeight, const DiligentSceneEnvironment& environment);
    bool beginFrame(MotionHistory& history, std::uint64_t frameId, double timestampSeconds,
                    int width, int height, const Matrix4& view, const Matrix4& projection,
                    int displayWidth = 0, int displayHeight = 0,
                    const std::array<float, 4>& clearColor = {0, 0, 0, 1});
    bool beginFrame(MotionHistory& history, std::uint64_t frameId, double timestampSeconds,
                    int width, int height, const Matrix4& view, const Matrix4& projection,
                    int displayWidth, int displayHeight,
                    const DiligentSceneEnvironment& environment);
    void beginUiFrame(bool manageMouseCursor = true);
    void endUiOnlyFrame(const std::array<float, 4>& clearColor = {0, 0, 0, 1});
    void cancelUiFrame() noexcept;
    // Capture the next successfully composed scene frame before swap-chain presentation.
    // GPU-to-CPU readback occurs only for this explicit one-shot request.
    void captureNextFrame(const std::filesystem::path& path);
    [[nodiscard]] bool hasPendingCapture() const noexcept;
    void cancelPendingCapture() noexcept;
    void endFrame();
    void endFrame(DiligentColorFilter filter);
    void endFrame(DiligentColorFilter filter, DiligentDebugView debugView);

    template <typename DrawScene>
    bool renderFrame(MotionHistory& history, std::uint64_t frameId, double timestampSeconds,
                     const openbus::app::DiligentCameraView& camera, int displayWidth,
                     int displayHeight, const DiligentSceneEnvironment& environment,
                     DrawScene&& drawScene, DiligentColorFilter filter = DiligentColorFilter::None,
                     DiligentDebugView debugView = DiligentDebugView::None) {
        if (!beginFrame(history, frameId, timestampSeconds, camera, displayWidth, displayHeight,
                        environment)) {
            return false;
        }
        try {
            std::forward<DrawScene>(drawScene)(*this);
            endFrame(filter, debugView);
        } catch (...) {
            abortFrame();
            throw;
        }
        return true;
    }

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::rendering
