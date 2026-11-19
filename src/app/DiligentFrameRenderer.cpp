#include "app/DiligentFrameRenderer.h"

#include "app/DiligentCameraBridge.h"
#include "app/DiligentImGuiLayer.h"
#include "app/DiligentScriptTexture.h"
#include "app/vehicle/DiligentVehicleBridge.h"
#include "app/vehicle/DiligentVehicleRuntimeTextures.h"
#include "app/vehicle/DiligentVehicleSourceBridge.h"
#include "assets/ScreenshotWriter.h"
#include "map/MapConfigLoader.h"
#include "render/diligent/DiligentHumanAsset.h"
#include "render/diligent/DiligentMapTileSet.h"
#include "render/diligent/DiligentSceneryAssetCache.h"
#include "render/diligent/DiligentTexture2D.h"
#include "render/diligent/DiligentWeatherClouds.h"
#include "render/diligent/DiligentWeatherPrecipitation.h"
#include "render/diligent/DiligentPrimitiveRenderer.h"
#include "render/MatrixMath.h"

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/DeviceContext.h>
#include <Graphics/GraphicsEngine/interface/RenderDevice.h>
#include <Graphics/GraphicsEngine/interface/Texture.h>
#include <Graphics/GraphicsEngine/interface/TextureView.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace openbus::rendering {
namespace {

const Diligent::NativeWindow& requireNoApiWindow(GLFWwindow& window,
                                                 const Diligent::NativeWindow& nativeWindow) {
    if (glfwGetWindowAttrib(&window, GLFW_CLIENT_API) != GLFW_NO_API) {
        throw std::invalid_argument("Diligent frame renderer requires a GLFW no-API window");
    }
    return nativeWindow;
}

std::string normalizeHumanAssetKey(std::string_view modelKey) {
    if (modelKey.empty()) {
        throw std::invalid_argument("Diligent human assets require a non-empty model key");
    }
    return std::filesystem::path(modelKey).lexically_normal().generic_string();
}

bool isValidHumanPass(DiligentHumanRenderPass pass) {
    return pass == DiligentHumanRenderPass::All || pass == DiligentHumanRenderPass::Opaque ||
           pass == DiligentHumanRenderPass::Transparent;
}

void validateSceneryPlacement(const Matrix4& worldTransform, double boundingRadius) {
    if (!std::isfinite(boundingRadius) || boundingRadius < 0.0) {
        throw std::invalid_argument(
            "Diligent scenery placement radius must be finite and non-negative");
    }
    if (!std::all_of(worldTransform.begin(), worldTransform.end(),
                     [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("Diligent scenery placement transform must be finite");
    }
}

} // namespace

struct DiligentFrameRenderer::Impl {
    struct VehicleAssetRecord {
        std::unique_ptr<DiligentVehicleAsset> asset;
        std::unique_ptr<openbus::app::vehicle::DiligentVehicleRuntimeTextures> runtimeTextures;
        std::uint64_t assetGeneration = 0;
        std::uint64_t materialSelectionGeneration = 0;
        ScriptRuntime* scriptRuntime = nullptr;
    };

    struct HumanAssetRecord {
        std::unique_ptr<DiligentHumanAsset> asset;
        std::uint64_t sourceGeneration = 0;
    };

    struct ReflectionPassResources {
        std::unique_ptr<DiligentReflectionTarget> target;
        std::unique_ptr<DiligentTexturedMeshRenderer> meshes;
        std::uint64_t lastRenderedFrame = 0;
        std::chrono::steady_clock::time_point lastRenderedTime = {};
        bool hasRendered = false;
    };

    struct MapSceneryRuntimeTextureRecord {
        ScriptRuntime* scriptRuntime = nullptr;
        std::map<std::pair<openbus::app::DiligentScriptTextureKind, int>,
                 std::unique_ptr<openbus::app::DiligentScriptTexture>>
            textures;
    };

    DiligentVulkanFrame frame;
    DiligentMultisamplingSelection multisamplingSelection;
    std::unique_ptr<DiligentTexture2D> fallbackTexture;
    DiligentSceneFrame scene;
    std::unique_ptr<DiligentPrimitiveRenderer> primitives;
    std::unique_ptr<DiligentWeatherClouds> weatherClouds;
    std::unique_ptr<DiligentWeatherClouds> reflectionWeatherClouds;
    std::unique_ptr<DiligentWeatherPrecipitation> weatherPrecipitation;
    DiligentImGuiLayer gui;
    DiligentHumanPoseHistory humanPoseHistory;
    DiligentSceneryAssetCache sceneryAssets;
    std::unique_ptr<DiligentSpotlightShadowMaps> spotlightShadowMaps;
    std::vector<ReflectionPassResources> reflectionPasses;
    std::uint64_t reflectionFrameInterval = 1;
    double reflectionMaximumFrameRate = 0.0;
    std::unordered_map<std::uint64_t, VehicleAssetRecord> vehicleAssets;
    std::unordered_map<std::uint64_t, MapSceneryRuntimeTextureRecord> mapSceneryRuntimeTextures;
    std::unordered_map<std::string, HumanAssetRecord> humanAssets;
    std::unique_ptr<DiligentMapTileSet> mapTiles;
    std::optional<openbus::map::MapCoordinates> mapCoordinates;
    std::filesystem::path mapRoot;
    std::filesystem::path omsiRoot;
    bool generateMipmaps = false;
    Diligent::Uint32 defaultAnisotropy = 0;
    DiligentColorFilter defaultColorFilter = DiligentColorFilter::None;
    bool sceneActive = false;
    bool reflectionPassActive = false;
    bool uiActive = false;
    std::optional<DiligentSceneEnvironment> activeEnvironment;
    std::optional<std::filesystem::path> capturePath;
    MotionHistory* activeMotionHistory = nullptr;
    bool invalidateMotionHistoryOnNextFrame = true;

    Impl(GLFWwindow& window, const Diligent::NativeWindow& nativeWindow,
         const Diligent::SwapChainDesc& swapChainDesc, bool includeLinearDepth,
         DiligentMultisampling multisampling, bool generateTextureMipmaps)
        : frame(requireNoApiWindow(window, nativeWindow), swapChainDesc),
          multisamplingSelection(
              selectDiligentMultisampling(frame.device(), includeLinearDepth, multisampling)),
          scene(frame.device(), swapChainDesc.Width, swapChainDesc.Height,
                swapChainDesc.ColorBufferFormat, includeLinearDepth,
                multisamplingSelection.multisampling),
          gui(window, frame.device(), swapChainDesc.ColorBufferFormat),
          sceneryAssets(frame.device(), generateTextureMipmaps),
          generateMipmaps(generateTextureMipmaps) {
        primitives = std::make_unique<DiligentPrimitiveRenderer>(
            frame.device(), Diligent::TEX_FORMAT_RGBA8_UNORM, Diligent::TEX_FORMAT_D32_FLOAT,
            multisamplingSelection.multisampling, includeLinearDepth);
        weatherClouds = std::make_unique<DiligentWeatherClouds>(
            frame.device(), Diligent::TEX_FORMAT_RGBA8_UNORM, multisamplingSelection.multisampling,
            includeLinearDepth);
        reflectionWeatherClouds = std::make_unique<DiligentWeatherClouds>(
            frame.device(), Diligent::TEX_FORMAT_RGBA8_UNORM, DiligentMultisampling{}, false, true);
        weatherPrecipitation = std::make_unique<DiligentWeatherPrecipitation>(
            frame.device(), Diligent::TEX_FORMAT_RGBA8_UNORM, multisamplingSelection.multisampling,
            includeLinearDepth);
        fallbackTexture =
            std::make_unique<DiligentTexture2D>(frame.device(), Image{1, 1, {255, 255, 255, 255}});
        if (multisamplingSelection.useFxaaFallback) {
            defaultColorFilter = DiligentColorFilter::Fxaa;
        }
    }

    void requireActiveScene(const char* operation) const {
        if (!sceneActive) {
            throw std::logic_error(std::string(operation) + " requires an active Diligent frame");
        }
    }

    void captureBackBuffer(const std::filesystem::path& path) {
        using namespace Diligent;
        ITextureView& target = frame.backBufferRenderTarget();
        ITexture* source = target.GetTexture();
        if (source == nullptr) {
            throw std::runtime_error("Diligent screenshot source texture is unavailable");
        }
        const TextureDesc& sourceDesc = source->GetDesc();
        if (sourceDesc.Format != TEX_FORMAT_RGBA8_UNORM || sourceDesc.SampleCount != 1 ||
            sourceDesc.Width == 0 || sourceDesc.Height == 0 ||
            sourceDesc.Width > static_cast<Uint32>(std::numeric_limits<int>::max()) ||
            sourceDesc.Height > static_cast<Uint32>(std::numeric_limits<int>::max())) {
            throw std::runtime_error("Diligent screenshot requires an RGBA8 single-sample target");
        }
        if (!captureStaging || captureStagingWidth != sourceDesc.Width ||
            captureStagingHeight != sourceDesc.Height) {
            TextureDesc stagingDesc;
            stagingDesc.Name = "OpenBus Diligent screenshot staging";
            stagingDesc.Type = RESOURCE_DIM_TEX_2D;
            stagingDesc.Width = sourceDesc.Width;
            stagingDesc.Height = sourceDesc.Height;
            stagingDesc.MipLevels = 1;
            stagingDesc.Format = sourceDesc.Format;
            stagingDesc.Usage = USAGE_STAGING;
            stagingDesc.BindFlags = BIND_NONE;
            stagingDesc.CPUAccessFlags = CPU_ACCESS_READ;
            frame.device().CreateTexture(stagingDesc, nullptr, &captureStaging);
            if (!captureStaging) {
                throw std::runtime_error("Failed to create Diligent screenshot staging texture");
            }
            captureStagingWidth = sourceDesc.Width;
            captureStagingHeight = sourceDesc.Height;
        }

        IDeviceContext& context = frame.immediateContext();
        context.SetRenderTargets(0, nullptr, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        CopyTextureAttribs copy;
        copy.pSrcTexture = source;
        copy.SrcTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
        copy.pDstTexture = captureStaging;
        copy.DstTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
        context.CopyTexture(copy);
        frame.waitForIdle();

        MappedTextureSubresource mapped;
        context.MapTextureSubresource(captureStaging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT, nullptr,
                                      mapped);
        if (mapped.pData == nullptr) {
            throw std::runtime_error("Failed to map Diligent screenshot staging texture");
        }
        struct UnmapTexture {
            IDeviceContext& context;
            ITexture& texture;
            ~UnmapTexture() {
                context.UnmapTextureSubresource(&texture, 0, 0);
            }
        } unmap{context, *captureStaging};

        const std::size_t rowBytes = static_cast<std::size_t>(sourceDesc.Width) * 4;
        if (mapped.Stride < rowBytes || static_cast<std::size_t>(sourceDesc.Height) >
                                            std::numeric_limits<std::size_t>::max() / rowBytes) {
            throw std::runtime_error("Diligent screenshot staging stride is invalid");
        }
        std::vector<std::uint8_t> pixels(rowBytes * sourceDesc.Height);
        const auto* sourcePixels = static_cast<const std::uint8_t*>(mapped.pData);
        for (Uint32 row = 0; row < sourceDesc.Height; ++row) {
            std::memcpy(pixels.data() + static_cast<std::size_t>(row) * rowBytes,
                        sourcePixels + static_cast<std::size_t>(row) * mapped.Stride, rowBytes);
        }
        if (!saveRgbaPng(path, static_cast<int>(sourceDesc.Width),
                         static_cast<int>(sourceDesc.Height), pixels, rowBytes)) {
            throw std::runtime_error("Failed to save Diligent screenshot: " + path.string());
        }
    }

    Diligent::RefCntAutoPtr<Diligent::ITexture> captureStaging;
    Diligent::Uint32 captureStagingWidth = 0;
    Diligent::Uint32 captureStagingHeight = 0;
};

DiligentFrameRenderer::DiligentFrameRenderer(GLFWwindow& window,
                                             const Diligent::NativeWindow& nativeWindow,
                                             const Diligent::SwapChainDesc& swapChainDesc,
                                             bool includeLinearDepth,
                                             DiligentMultisampling multisampling,
                                             bool generateTextureMipmaps)
    : impl_(std::make_unique<Impl>(window, nativeWindow, swapChainDesc, includeLinearDepth,
                                   multisampling, generateTextureMipmaps)) {}

DiligentFrameRenderer::DiligentFrameRenderer(
    GLFWwindow& window, const Diligent::NativeWindow& nativeWindow,
    const Diligent::SwapChainDesc& swapChainDesc,
    const openbus::app::DiligentGraphicsOptions& graphicsOptions, bool includeLinearDepth)
    : DiligentFrameRenderer(window, nativeWindow, swapChainDesc, includeLinearDepth,
                            graphicsOptions.multisampling, graphicsOptions.generateTextureMipmaps) {
    if (graphicsOptions.anisotropy != 1 && graphicsOptions.anisotropy != 2 &&
        graphicsOptions.anisotropy != 4 && graphicsOptions.anisotropy != 8 &&
        graphicsOptions.anisotropy != 16) {
        throw std::invalid_argument("Invalid Diligent graphics anisotropy option");
    }
    setVerticalSync(graphicsOptions.verticalSync);
    impl_->defaultAnisotropy = graphicsOptions.anisotropy;
    setDefaultColorFilter(graphicsOptions.colorFilter, graphicsOptions.multisampling.samples > 1);
}

DiligentFrameRenderer::~DiligentFrameRenderer() {
    if (impl_ && impl_->activeMotionHistory != nullptr) {
        impl_->activeMotionHistory->invalidate();
    }
}

Diligent::IRenderDevice& DiligentFrameRenderer::device() const noexcept {
    return impl_->frame.device();
}

Diligent::IDeviceContext& DiligentFrameRenderer::immediateContext() const noexcept {
    return impl_->frame.immediateContext();
}

Diligent::Uint32 DiligentFrameRenderer::maximumAnisotropy() const noexcept {
    return impl_->scene.maximumAnisotropy();
}

Diligent::ITextureView& DiligentFrameRenderer::fallbackTexture() const {
    return impl_->fallbackTexture->shaderResourceView();
}

DiligentSpotlightShadowMaps&
DiligentFrameRenderer::configureSpotlightShadowMaps(Diligent::Uint32 resolution,
                                                    Diligent::Uint32 capacity) {
    if (impl_->sceneActive) {
        throw std::logic_error(
            "Cannot configure Diligent spotlight shadow maps during scene drawing");
    }
    if (impl_->spotlightShadowMaps && impl_->spotlightShadowMaps->resolution() == resolution &&
        impl_->spotlightShadowMaps->capacity() == capacity) {
        return *impl_->spotlightShadowMaps;
    }
    auto shadowMaps =
        std::make_unique<DiligentSpotlightShadowMaps>(impl_->frame.device(), resolution, capacity);
    impl_->spotlightShadowMaps = std::move(shadowMaps);
    return *impl_->spotlightShadowMaps;
}

DiligentSpotlightShadowMaps* DiligentFrameRenderer::spotlightShadowMaps() noexcept {
    return impl_->spotlightShadowMaps.get();
}

const DiligentSpotlightShadowMaps* DiligentFrameRenderer::spotlightShadowMaps() const noexcept {
    return impl_->spotlightShadowMaps.get();
}

void DiligentFrameRenderer::configureReflectionTargets(std::span<const Diligent::Uint32> sizes,
                                                       std::uint64_t frameInterval,
                                                       double maximumFrameRate) {
    if (impl_->sceneActive) {
        throw std::logic_error("Cannot configure Diligent reflection targets during scene drawing");
    }
    if (frameInterval == 0 || !std::isfinite(maximumFrameRate) || maximumFrameRate < 0.0 ||
        maximumFrameRate > 1000.0) {
        throw std::invalid_argument("Diligent reflection scheduling options are invalid");
    }
    impl_->reflectionFrameInterval = frameInterval;
    impl_->reflectionMaximumFrameRate = maximumFrameRate;
    if (sizes.size() == impl_->reflectionPasses.size() &&
        std::equal(sizes.begin(), sizes.end(), impl_->reflectionPasses.begin(),
                   [](Diligent::Uint32 size, const Impl::ReflectionPassResources& resources) {
                       return size == resources.target->size();
                   })) {
        return;
    }

    std::vector<Impl::ReflectionPassResources> replacement;
    replacement.reserve(sizes.size());
    for (const Diligent::Uint32 size : sizes) {
        Impl::ReflectionPassResources resources;
        resources.target = std::make_unique<DiligentReflectionTarget>(
            impl_->frame.device(), size, Diligent::TEX_FORMAT_RGBA8_UNORM,
            Diligent::TEX_FORMAT_D32_FLOAT);
        resources.meshes = std::make_unique<DiligentTexturedMeshRenderer>(
            impl_->frame.device(), Diligent::TEX_FORMAT_RGBA8_UNORM,
            Diligent::TEX_FORMAT_D32_FLOAT);
        replacement.push_back(std::move(resources));
    }
    if (!impl_->reflectionPasses.empty()) {
        impl_->frame.immediateContext().WaitForIdle();
    }
    impl_->reflectionPasses = std::move(replacement);
}

const DiligentReflectionTarget& DiligentFrameRenderer::reflectionTarget(std::size_t index) const {
    if (index >= impl_->reflectionPasses.size()) {
        throw std::out_of_range("Diligent reflection target index is not configured");
    }
    return *impl_->reflectionPasses[index].target;
}

bool DiligentFrameRenderer::renderReflectionTarget(std::size_t index, std::uint64_t frameId,
                                                   bool needed, const ReflectionDrawCallback& draw,
                                                   const std::array<float, 4>& clearColor,
                                                   bool forceRefresh) {
    impl_->requireActiveScene("Reflection pass");
    if (impl_->reflectionPassActive) {
        throw std::logic_error("Diligent reflection passes cannot be nested");
    }
    if (index >= impl_->reflectionPasses.size()) {
        throw std::out_of_range("Diligent reflection target index is not configured");
    }
    if (!draw) {
        throw std::invalid_argument("Diligent reflection pass requires a draw callback");
    }
    if (frameId != impl_->scene.frameData().frameId ||
        !std::all_of(clearColor.begin(), clearColor.end(),
                     [](float value) { return std::isfinite(value); })) {
        throw std::invalid_argument("Diligent reflection frame or clear color is invalid");
    }

    auto& resources = impl_->reflectionPasses[index];
    if (resources.hasRendered && frameId < resources.lastRenderedFrame) {
        throw std::invalid_argument("Diligent reflection frame identifier moved backwards");
    }
    if (!needed) {
        return false;
    }
    const auto renderStart = std::chrono::steady_clock::now();
    if (resources.hasRendered && !forceRefresh) {
        if (frameId - resources.lastRenderedFrame < impl_->reflectionFrameInterval) {
            return false;
        }
        if (impl_->reflectionMaximumFrameRate > 0.0 &&
            std::chrono::duration<double>(renderStart - resources.lastRenderedTime).count() <
                1.0 / impl_->reflectionMaximumFrameRate) {
            return false;
        }
    }

    Diligent::IDeviceContext& context = impl_->frame.immediateContext();
    DiligentTexturedMeshRenderer& meshes = *resources.meshes;
    meshes.setPointLights(context, {});
    meshes.setSpotLights(context, {});
    meshes.setSpotlightShadowMapView(nullptr);
    if (impl_->defaultAnisotropy != 0) {
        meshes.setDefaultAnisotropy(impl_->defaultAnisotropy);
    }

    resources.target->clear(context, clearColor);
    impl_->reflectionPassActive = true;
    const auto restoreSceneTarget = [&] {
        try {
            impl_->scene.targets().bind(context);
            const auto width = impl_->scene.targets().width();
            const auto height = impl_->scene.targets().height();
            const Diligent::Viewport viewport{
                0.0F, 0.0F, static_cast<float>(width), static_cast<float>(height), 0.0F, 1.0F};
            context.SetViewports(1, &viewport, width, height);
        } catch (...) {
            impl_->reflectionPassActive = false;
            throw;
        }
        impl_->reflectionPassActive = false;
    };
    try {
        draw(*resources.target, meshes);
    } catch (...) {
        const std::exception_ptr drawError = std::current_exception();
        restoreSceneTarget();
        std::rethrow_exception(drawError);
    }
    restoreSceneTarget();
    resources.lastRenderedFrame = frameId;
    resources.lastRenderedTime = renderStart;
    resources.hasRendered = true;
    return true;
}

DiligentTexturedMeshRenderer& DiligentFrameRenderer::meshes() {
    impl_->requireActiveScene("Mesh access");
    return impl_->scene.meshes();
}

void DiligentFrameRenderer::drawPrimitives(std::span<const PrimitiveVertex> vertices,
                                           const Matrix4& modelViewProjection,
                                           const Matrix4& modelView) {
    impl_->requireActiveScene("Primitive draw");
    impl_->primitives->draw(impl_->frame.immediateContext(), vertices, modelViewProjection,
                            modelView);
}

std::size_t DiligentFrameRenderer::drawOpaqueSpotlightShadowLayer(
    Diligent::Uint32 lightIndex, std::span<const DiligentShadowCaster> casters) {
    impl_->requireActiveScene("Spotlight shadow pass");
    if (!impl_->spotlightShadowMaps) {
        throw std::logic_error("Diligent spotlight shadow maps are not configured");
    }
    return impl_->scene.meshes().drawOpaqueShadowLayer(
        impl_->frame.immediateContext(), impl_->scene.targets(), *impl_->spotlightShadowMaps,
        lightIndex, casters);
}

bool DiligentFrameRenderer::drawWeatherClouds(std::optional<WeatherCloudType> type,
                                              std::optional<double> baseAboveGroundMeters,
                                              std::optional<double> windDirectionDegrees,
                                              std::optional<double> windSpeedMetersPerSecond,
                                              std::optional<double> brightness,
                                              double groundHeightMeters, double simulationSeconds) {
    impl_->requireActiveScene("Weather cloud draw");
    if (!impl_->activeEnvironment) {
        throw std::logic_error("Diligent weather clouds require an active scene environment");
    }
    const MotionDataFrame& frame = impl_->scene.frameData();
    return impl_->weatherClouds->draw(
        impl_->frame.immediateContext(), frame.currentView, frame.currentProjection,
        impl_->activeEnvironment->fog, type, baseAboveGroundMeters, windDirectionDegrees,
        windSpeedMetersPerSecond, brightness, groundHeightMeters, simulationSeconds);
}

bool DiligentFrameRenderer::drawWeatherCloudsInReflection(
    const Matrix4& view, const Matrix4& projection, const DiligentWeatherFog& fog,
    std::optional<WeatherCloudType> type, std::optional<double> baseAboveGroundMeters,
    std::optional<double> windDirectionDegrees, std::optional<double> windSpeedMetersPerSecond,
    std::optional<double> brightness, double groundHeightMeters, double simulationSeconds) {
    impl_->requireActiveScene("Reflection weather cloud draw");
    if (!impl_->reflectionPassActive) {
        throw std::logic_error("Reflection weather clouds require an active reflection target");
    }
    return impl_->reflectionWeatherClouds->draw(impl_->frame.immediateContext(), view, projection,
                                                fog, type, baseAboveGroundMeters,
                                                windDirectionDegrees, windSpeedMetersPerSecond,
                                                brightness, groundHeightMeters, simulationSeconds);
}

bool DiligentFrameRenderer::drawWeatherPrecipitation(int type, double rate,
                                                     double simulationSeconds,
                                                     std::optional<double> windDirectionDegrees,
                                                     std::optional<double> windSpeedMetersPerSecond,
                                                     std::optional<double> brightness) {
    impl_->requireActiveScene("Weather precipitation draw");
    if (!impl_->activeEnvironment) {
        throw std::logic_error("Diligent precipitation requires an active scene environment");
    }
    const MotionDataFrame& frame = impl_->scene.frameData();
    return impl_->weatherPrecipitation->draw(
        impl_->frame.immediateContext(), frame.currentView, frame.currentProjection,
        static_cast<Diligent::Uint32>(frame.width), static_cast<Diligent::Uint32>(frame.height),
        impl_->activeEnvironment->fog, type, rate, simulationSeconds, windDirectionDegrees,
        windSpeedMetersPerSecond, brightness);
}

DiligentHumanPoseHistory& DiligentFrameRenderer::humanPoseHistory() {
    impl_->requireActiveScene("Human pose history access");
    return impl_->humanPoseHistory;
}

void DiligentFrameRenderer::setMapLights(std::span<const MapPointLight> activeLights,
                                         const Matrix4& view,
                                         const std::array<double, 3>& cameraWorldPosition,
                                         double strength, std::size_t maximumLights) {
    impl_->requireActiveScene("Map-light update");
    const std::vector<DiligentPointLight> lights = buildDiligentMapPointLights(
        activeLights, view, cameraWorldPosition, strength, maximumLights);
    impl_->scene.meshes().setPointLights(impl_->frame.immediateContext(), lights);
}

void DiligentFrameRenderer::setSpotLights(std::span<const DiligentSpotLight> activeLights) {
    impl_->requireActiveScene("Spotlight update");
    if (std::any_of(activeLights.begin(), activeLights.end(), [&](const auto& light) {
            return light.shadowValid &&
                   (impl_->spotlightShadowMaps == nullptr ||
                    light.shadowMapIndex >= impl_->spotlightShadowMaps->capacity());
        })) {
        throw std::invalid_argument("Diligent spotlight refers to an unavailable shadow-map layer");
    }
    auto& meshes = impl_->scene.meshes();
    meshes.setSpotlightShadowMapView(impl_->spotlightShadowMaps != nullptr
                                         ? &impl_->spotlightShadowMaps->shaderResourceView()
                                         : nullptr);
    meshes.setSpotLights(impl_->frame.immediateContext(), activeLights);
}

void DiligentFrameRenderer::setSpotLightsWithShadows(
    std::span<const DiligentSpotLight> activeLights,
    std::span<const DiligentSpotlightShadowBinding> shadowBindings, const Matrix4& cameraView,
    float minimumNdcZ) {
    impl_->requireActiveScene("Shadowed spotlight update");
    std::vector<DiligentSpotLight> lights(activeLights.begin(), activeLights.end());
    applyDiligentSpotlightShadowBindings(lights, shadowBindings, cameraView, minimumNdcZ);
    setSpotLights(lights);
}

void DiligentFrameRenderer::setVehicleSpotLights(std::span<const VehicleSpotlight> viewSpaceLights,
                                                 std::size_t maximumLights) {
    impl_->requireActiveScene("Vehicle spotlight update");
    const std::vector<DiligentSpotLight> lights =
        openbus::app::vehicle::makeDiligentVehicleSpotLights(viewSpaceLights, maximumLights);
    setSpotLights(lights);
}

void DiligentFrameRenderer::setVehicleSpotLightsWithShadows(
    std::span<const VehicleSpotlight> viewSpaceLights,
    std::span<const DiligentSpotlightShadowBinding> shadowBindings, const Matrix4& cameraView,
    std::size_t maximumLights, float minimumNdcZ) {
    impl_->requireActiveScene("Shadowed vehicle spotlight update");
    const std::vector<DiligentSpotLight> lights =
        openbus::app::vehicle::makeDiligentVehicleSpotLights(viewSpaceLights, maximumLights);
    setSpotLightsWithShadows(lights, shadowBindings, cameraView, minimumNdcZ);
}

bool DiligentFrameRenderer::requestSceneryAsset(const std::filesystem::path& configPath,
                                                const std::filesystem::path& omsiRoot) {
    return impl_->sceneryAssets.request(configPath, omsiRoot);
}

bool DiligentFrameRenderer::requestSeasonalSceneryTexture(
    const std::filesystem::path& texturePath) {
    return impl_->sceneryAssets.requestSeasonalTexture(texturePath);
}

const DiligentSceneryAsset*
DiligentFrameRenderer::sceneryAsset(const std::filesystem::path& configPath,
                                    const std::filesystem::path& omsiRoot) const {
    return impl_->sceneryAssets.find(configPath, omsiRoot);
}

const DiligentTexture2D*
DiligentFrameRenderer::seasonalSceneryTexture(const std::filesystem::path& texturePath) const {
    return impl_->sceneryAssets.seasonalTexture(texturePath);
}

bool DiligentFrameRenderer::synchronizeMapSceneryRuntimeTextures(
    std::uint64_t motionObjectId, const DiligentSceneryAsset& asset, ScriptRuntime* scripts,
    std::vector<Diligent::ITextureView*>& dynamicBatchTextures,
    std::vector<Diligent::ITextureView*>& textBatchTextures) {
    impl_->requireActiveScene("Map scenery runtime texture update");
    if (motionObjectId == 0) {
        throw std::invalid_argument("Map scenery runtime textures require a stable object ID");
    }

    dynamicBatchTextures.clear();
    textBatchTextures.clear();
    const auto batches = asset.batches();
    const bool usesRuntimeTextures =
        std::any_of(batches.begin(), batches.end(), [](const DiligentSceneryBatch& batch) {
            return batch.source.batch.scriptTextureIndex >= 0 ||
                   batch.source.batch.textTextureIndex >= 0;
        });
    if (!usesRuntimeTextures) {
        return true;
    }
    if (scripts == nullptr) {
        return false;
    }

    Impl::MapSceneryRuntimeTextureRecord& record = impl_->mapSceneryRuntimeTextures[motionObjectId];
    if (record.scriptRuntime != scripts) {
        record.textures.clear();
        record.scriptRuntime = scripts;
    }

    std::vector<Diligent::ITextureView*> synchronizedDynamicTextures(batches.size(), nullptr);
    std::vector<Diligent::ITextureView*> synchronizedTextTextures(batches.size(), nullptr);
    bool hasDynamicTextures = false;
    bool hasTextTextures = false;
    const auto synchronizeSlot = [&](openbus::app::DiligentScriptTextureKind kind, int index) {
        const auto key = std::pair{kind, index};
        auto texture = record.textures.find(key);
        if (texture == record.textures.end()) {
            texture = record.textures
                          .emplace(key, std::make_unique<openbus::app::DiligentScriptTexture>(
                                            impl_->frame.device(), kind, index))
                          .first;
        }
        openbus::app::DiligentScriptTexture& scriptTexture = *texture->second;
        const bool synchronized =
            scriptTexture.synchronize(*scripts, impl_->frame.immediateContext());
        if (!synchronized && !scriptTexture.texture().initialized()) {
            return static_cast<Diligent::ITextureView*>(nullptr);
        }
        return scriptTexture.texture().shaderResourceView();
    };

    for (std::size_t index = 0; index < batches.size(); ++index) {
        const openbus::map::MapSceneryBatch& source = batches[index].source.batch;
        if (source.scriptTextureIndex >= 0) {
            hasDynamicTextures = true;
            synchronizedDynamicTextures[index] = synchronizeSlot(
                openbus::app::DiligentScriptTextureKind::Script, source.scriptTextureIndex);
            if (synchronizedDynamicTextures[index] == nullptr) {
                return false;
            }
        }
        if (source.textTextureIndex >= 0) {
            hasTextTextures = true;
            synchronizedTextTextures[index] = synchronizeSlot(
                openbus::app::DiligentScriptTextureKind::Text, source.textTextureIndex);
            if (synchronizedTextTextures[index] == nullptr) {
                return false;
            }
        }
    }

    if (hasDynamicTextures) {
        dynamicBatchTextures = std::move(synchronizedDynamicTextures);
    }
    if (hasTextTextures) {
        textBatchTextures = std::move(synchronizedTextTextures);
    }
    return true;
}

std::size_t DiligentFrameRenderer::retainMapSceneryRuntimeTextures(
    std::span<const std::uint64_t> motionObjectIds) {
    if (impl_->sceneActive) {
        throw std::logic_error(
            "Cannot evict Diligent map scenery runtime textures during scene drawing");
    }
    std::unordered_set<std::uint64_t> retained;
    retained.reserve(motionObjectIds.size());
    for (const std::uint64_t objectId : motionObjectIds) {
        if (objectId == 0) {
            throw std::invalid_argument("Map scenery runtime texture IDs must be non-zero");
        }
        retained.insert(objectId);
    }

    std::size_t removed = 0;
    for (auto record = impl_->mapSceneryRuntimeTextures.begin();
         record != impl_->mapSceneryRuntimeTextures.end();) {
        if (!retained.contains(record->first)) {
            record = impl_->mapSceneryRuntimeTextures.erase(record);
            ++removed;
        } else {
            ++record;
        }
    }
    return removed;
}

std::size_t DiligentFrameRenderer::publishReadySceneryAssets() {
    if (impl_->sceneActive) {
        throw std::logic_error("Cannot upload Diligent scenery assets during scene drawing");
    }
    return impl_->sceneryAssets.publishReady();
}

std::size_t DiligentFrameRenderer::retainSceneryAssets(
    std::span<const std::pair<std::filesystem::path, std::filesystem::path>> assets,
    std::span<const std::filesystem::path> seasonalTextures) {
    if (impl_->sceneActive) {
        throw std::logic_error("Cannot evict Diligent scenery assets during scene drawing");
    }
    return impl_->sceneryAssets.retain(assets, seasonalTextures);
}

std::size_t DiligentFrameRenderer::requestMapTileWindow(
    const openbus::map::MapDefinition& map, double worldX, double worldY, int radius,
    const std::filesystem::path& omsiRoot, std::size_t baseGroundTextureIndex,
    std::shared_ptr<const openbus::map::MapRoadConnectionIndex> roadConnections) {
    if (impl_->sceneActive) {
        throw std::logic_error("Cannot update Diligent map tiles during scene drawing");
    }
    const std::filesystem::path mapRoot = map.rootPath.lexically_normal();
    const std::filesystem::path normalizedOmsiRoot = omsiRoot.lexically_normal();
    const bool sourceChanged =
        !impl_->mapTiles || !impl_->mapCoordinates.has_value() ||
        impl_->mapCoordinates->tileSize != map.coordinates.tileSize ||
        impl_->mapCoordinates->worldCoordinates != map.coordinates.worldCoordinates ||
        impl_->mapRoot != mapRoot || impl_->omsiRoot != normalizedOmsiRoot;
    if (sourceChanged) {
        auto replacement = std::make_unique<DiligentMapTileSet>(
            impl_->frame.device(), map.coordinates, impl_->generateMipmaps);
        const std::size_t scheduled =
            replacement->requestTerrainWindow(map, worldX, worldY, radius, omsiRoot,
                                              baseGroundTextureIndex, std::move(roadConnections));
        impl_->mapTiles = std::move(replacement);
        impl_->mapCoordinates = map.coordinates;
        impl_->mapRoot = mapRoot;
        impl_->omsiRoot = normalizedOmsiRoot;
        return scheduled;
    }
    return impl_->mapTiles->requestTerrainWindow(
        map, worldX, worldY, radius, omsiRoot, baseGroundTextureIndex, std::move(roadConnections));
}

std::size_t DiligentFrameRenderer::publishReadyMapTiles() {
    if (impl_->sceneActive) {
        throw std::logic_error("Cannot upload Diligent map tiles during scene drawing");
    }
    return impl_->mapTiles ? impl_->mapTiles->publishReadyTerrainTiles() : 0;
}

std::size_t DiligentFrameRenderer::removeInactiveMapTiles() {
    if (impl_->sceneActive) {
        throw std::logic_error("Cannot evict Diligent map tiles during scene drawing");
    }
    return impl_->mapTiles ? impl_->mapTiles->removeInactiveTiles() : 0;
}

const DiligentMapTileSet& DiligentFrameRenderer::mapTiles() const {
    impl_->requireActiveScene("Map tile access");
    if (!impl_->mapTiles) {
        throw std::logic_error("Diligent map tiles have not been requested");
    }
    return *impl_->mapTiles;
}

std::size_t DiligentFrameRenderer::drawMapTerrain(Diligent::ITextureView& fallbackTexture,
                                                  const Matrix4& viewProjection,
                                                  const DiligentMaterialState* materialOverride,
                                                  const DiligentLightingParameters& lighting,
                                                  const DiligentMotionParameters* motion,
                                                  const DiligentMapWeather* weather) {
    const DiligentMapTileSet& tiles = mapTiles();
    const MotionDataFrame* motionFrame =
        impl_->activeMotionHistory != nullptr ? &impl_->scene.frameData() : nullptr;
    return tiles.drawTerrain(impl_->scene.meshes(), impl_->frame.immediateContext(),
                             fallbackTexture, viewProjection, materialOverride, lighting, motion,
                             motionFrame, impl_->activeMotionHistory, weather);
}

std::size_t DiligentFrameRenderer::drawMapWater(Diligent::ITextureView& fallbackTexture,
                                                const Matrix4& viewProjection,
                                                const DiligentMaterialState* materialOverride,
                                                const DiligentLightingParameters& lighting,
                                                const DiligentMotionParameters* motion) {
    const DiligentMapTileSet& tiles = mapTiles();
    const MotionDataFrame* motionFrame =
        impl_->activeMotionHistory != nullptr ? &impl_->scene.frameData() : nullptr;
    return tiles.drawWater(impl_->scene.meshes(), impl_->frame.immediateContext(), fallbackTexture,
                           viewProjection, materialOverride, lighting, motion, motionFrame,
                           impl_->activeMotionHistory);
}

std::size_t DiligentFrameRenderer::drawMapRoads(DiligentMapRoadPass pass,
                                                Diligent::ITextureView& fallbackTexture,
                                                const Matrix4& viewProjection,
                                                const DiligentMaterialState* materialOverride,
                                                const DiligentLightingParameters& lighting,
                                                const DiligentMotionParameters* motion,
                                                const DiligentMapWeather* weather) {
    const DiligentMapTileSet& tiles = mapTiles();
    const MotionDataFrame* motionFrame =
        impl_->activeMotionHistory != nullptr ? &impl_->scene.frameData() : nullptr;
    return tiles.drawRoads(pass, impl_->scene.meshes(), impl_->frame.immediateContext(),
                           fallbackTexture, viewProjection, materialOverride, lighting, motion,
                           motionFrame, impl_->activeMotionHistory, weather);
}

bool DiligentFrameRenderer::drawSceneryBatch(const DiligentSceneryBatch& batch,
                                             DiligentSceneryPass pass,
                                             Diligent::ITextureView& fallbackTexture,
                                             const Matrix4& modelViewProjection,
                                             const DiligentSceneryOpaqueDraw& draw,
                                             const DiligentLightingParameters& lighting,
                                             const DiligentMotionParameters* motion) {
    impl_->requireActiveScene("Scenery batch draw");
    switch (pass) {
    case DiligentSceneryPass::Opaque:
        return batch.drawOpaque(impl_->scene.meshes(), impl_->frame.immediateContext(),
                                fallbackTexture, modelViewProjection, draw, lighting, motion);
    case DiligentSceneryPass::Transparent:
        return batch.drawTransparent(impl_->scene.meshes(), impl_->frame.immediateContext(),
                                     fallbackTexture, modelViewProjection, draw, lighting, motion);
    case DiligentSceneryPass::TransmapDepthPrepass:
        return batch.drawTransmapDepthPrepass(impl_->scene.meshes(),
                                              impl_->frame.immediateContext(), fallbackTexture,
                                              modelViewProjection, draw, lighting);
    }
    throw std::invalid_argument("Unknown Diligent scenery render pass");
}

bool DiligentFrameRenderer::drawSceneryTreeBatch(const DiligentSceneryTreeBatch& batch,
                                                 Diligent::ITextureView& fallbackTexture,
                                                 const Matrix4& modelViewProjection,
                                                 const DiligentSceneryOpaqueDraw& draw,
                                                 const DiligentLightingParameters& lighting,
                                                 const DiligentMotionParameters* motion) {
    impl_->requireActiveScene("Scenery tree draw");
    return batch.draw(impl_->scene.meshes(), impl_->frame.immediateContext(), fallbackTexture,
                      modelViewProjection, draw, lighting, motion);
}

DiligentSceneryDrawResult DiligentFrameRenderer::drawSceneryAsset(
    const DiligentSceneryAsset& asset, DiligentSceneryPass pass,
    Diligent::ITextureView& fallbackTexture, const Matrix4& modelViewProjection,
    const Matrix4& modelView, std::span<const DiligentSceneryOpaqueDraw> batchDraws,
    std::span<const DiligentSceneryOpaqueDraw> treeDraws, MotionObjectKey motionKey,
    bool geometryHistoryValid, const DiligentLightingParameters& lighting,
    const ::Variables* variables) {
    impl_->requireActiveScene("Scenery asset draw");
    const auto batches = asset.batches();
    const auto trees = asset.trees();
    const bool transparentTrees =
        pass == DiligentSceneryPass::Transparent &&
        (std::any_of(trees.begin(), trees.end(),
                     [](const auto& tree) { return tree.source.noZwrite; }) ||
         (treeDraws.size() == trees.size() &&
          std::any_of(treeDraws.begin(), treeDraws.end(),
                      [](const auto& draw) { return draw.visibilityFade < 1.0F; })));
    if (batchDraws.size() != batches.size() ||
        ((pass == DiligentSceneryPass::Opaque || transparentTrees) &&
         treeDraws.size() != trees.size())) {
        throw std::invalid_argument("Diligent scenery draw state count does not match the asset");
    }

    DiligentSceneryDrawResult result;
    try {
        std::vector<std::size_t> order;
        order.reserve(batches.size());
        for (std::size_t index = 0; index < batches.size(); ++index) {
            const auto& batch = batches[index].source.batch;
            const DiligentSceneryOpaqueDraw& draw = batchDraws[index];
            if (!std::isfinite(draw.visibilityFade) || draw.visibilityFade < 0.0F ||
                draw.visibilityFade > 1.0F) {
                throw std::invalid_argument(
                    "Diligent scenery visibility fade must be finite and in [0, 1]");
            }
            const bool usesTextTexture = batch.textTextureIndex >= 0;
            const bool fadingOpaqueBatch = !usesTextTexture && !batch.noZwrite &&
                                           draw.visibilityFade < 1.0F &&
                                           (batch.alphaMode == 0 || batch.alphaMode == 1);
            const bool matchesPass =
                (pass == DiligentSceneryPass::Opaque && !usesTextTexture && !batch.noZwrite &&
                 (batch.alphaMode == 0 || batch.alphaMode == 1) && !fadingOpaqueBatch) ||
                (pass == DiligentSceneryPass::Transparent &&
                 (usesTextTexture || batch.noZwrite || batch.alphaMode == 2 ||
                  batch.alphaMode == 3 || fadingOpaqueBatch)) ||
                (pass == DiligentSceneryPass::TransmapDepthPrepass && batch.alphaMode == 2);
            if (!matchesPass) {
                continue;
            }
            const ::Variables* selectedVariables =
                variables != nullptr ? variables : draw.variables;
            if (draw.visibilityFade == 0.0F ||
                (!batch.visibilityVariable.empty() &&
                 (selectedVariables != nullptr
                      ? selectedVariables->get(batch.visibilityVariable)
                      : 0.0F) != static_cast<float>(batch.visibilityValue))) {
                continue;
            }
            order.push_back(index);
        }
        bool drawsTrees = false;
        std::vector<std::size_t> transparentTreeOrder;
        if (pass == DiligentSceneryPass::Opaque || transparentTrees) {
            for (std::size_t index = 0; index < trees.size(); ++index) {
                const float visibilityFade = treeDraws[index].visibilityFade;
                if (!std::isfinite(visibilityFade) || visibilityFade < 0.0F ||
                    visibilityFade > 1.0F) {
                    throw std::invalid_argument(
                        "Diligent tree visibility fade must be finite and in [0, 1]");
                }
                const bool transparentTree = trees[index].source.noZwrite || visibilityFade < 1.0F;
                if (transparentTree != (pass == DiligentSceneryPass::Transparent)) {
                    continue;
                }
                drawsTrees = drawsTrees || (visibilityFade > 0.0F && trees[index].mesh != nullptr);
                if (pass == DiligentSceneryPass::Transparent && visibilityFade > 0.0F &&
                    trees[index].mesh != nullptr) {
                    transparentTreeOrder.push_back(index);
                }
            }
        }
        std::vector<DiligentScenerySubmission> transparentOrder;
        if (pass == DiligentSceneryPass::Transparent) {
            transparentOrder = orderDiligentTransparentScenery(batches, order, trees,
                                                               transparentTreeOrder, modelView);
        }
        if (order.empty() && !drawsTrees) {
            return result;
        }

        std::optional<DiligentMotionParameters> motion;
        if (impl_->activeMotionHistory != nullptr) {
            const MotionTransform transform = impl_->activeMotionHistory->previousModelView(
                modelView, motionKey, geometryHistoryValid);
            motion = DiligentMotionParameters::fromHistory(impl_->scene.frameData(), transform);
            result.motionHistoryValid = motion->historyValid;
        }

        const auto drawBatch = [&](std::size_t index) {
            DiligentSceneryOpaqueDraw draw = batchDraws[index];
            if (variables != nullptr) {
                draw.variables = variables;
            }
            result.batchesDrawn +=
                drawSceneryBatch(batches[index], pass, fallbackTexture, modelViewProjection, draw,
                                 lighting, motion ? &*motion : nullptr)
                    ? 1U
                    : 0U;
        };
        const auto drawTree = [&](std::size_t index) {
            DiligentSceneryOpaqueDraw draw = treeDraws[index];
            if (variables != nullptr) {
                draw.variables = variables;
            }
            result.treesDrawn +=
                drawSceneryTreeBatch(trees[index], fallbackTexture, modelViewProjection, draw,
                                     lighting, motion ? &*motion : nullptr)
                    ? 1U
                    : 0U;
        };
        if (pass == DiligentSceneryPass::Transparent) {
            for (const DiligentScenerySubmission& submission : transparentOrder) {
                if (submission.kind == DiligentScenerySubmissionKind::Batch) {
                    drawBatch(submission.index);
                } else {
                    drawTree(submission.index);
                }
            }
        } else {
            for (const std::size_t index : order) {
                drawBatch(index);
            }
        }
        if (pass == DiligentSceneryPass::Opaque) {
            for (std::size_t index = 0; index < trees.size(); ++index) {
                if (trees[index].source.noZwrite || treeDraws[index].visibilityFade < 1.0F) {
                    continue;
                }
                drawTree(index);
            }
        }
    } catch (...) {
        if (impl_->activeMotionHistory != nullptr) {
            impl_->activeMotionHistory->invalidate();
            impl_->activeMotionHistory = nullptr;
            impl_->invalidateMotionHistoryOnNextFrame = true;
        }
        throw;
    }
    return result;
}

DiligentSceneryDrawResult DiligentFrameRenderer::drawSceneryPlacement(
    const DiligentSceneryAsset& asset, const Matrix4& worldTransform, double boundingRadius,
    DiligentSceneryPass pass, Diligent::ITextureView& fallbackTexture,
    std::span<const DiligentSceneryOpaqueDraw> batchDraws,
    std::span<const DiligentSceneryOpaqueDraw> treeDraws, std::uint64_t motionObjectId,
    std::uint64_t viewId, bool geometryHistoryValid, const DiligentLightingParameters& lighting,
    const ::Variables* variables) {
    if (!std::isfinite(boundingRadius) || boundingRadius < 0.0) {
        throw std::invalid_argument(
            "Diligent scenery placement radius must be finite and non-negative");
    }
    if (motionObjectId == 0) {
        throw std::invalid_argument(
            "Diligent scenery placement requires a stable motion object identifier");
    }
    const MotionDataFrame& frame = frameData();
    validateSceneryPlacement(worldTransform, boundingRadius);
    const Matrix4 modelView = multiplyMatrix4(frame.currentView, worldTransform);
    if (!isSphereVisibleInView(modelView, {0.0, 0.0, 0.0}, boundingRadius)) {
        DiligentSceneryDrawResult result;
        result.culled = true;
        return result;
    }
    const Matrix4 modelViewProjection = multiplyMatrix4(frame.currentProjection, modelView);
    return drawSceneryAsset(asset, pass, fallbackTexture, modelViewProjection, modelView,
                            batchDraws, treeDraws, {motionObjectId, viewId}, geometryHistoryValid,
                            lighting, variables);
}

bool DiligentFrameRenderer::isSceneryPlacementVisible(const Matrix4& worldTransform,
                                                      double boundingRadius) const {
    impl_->requireActiveScene("Scenery placement visibility test");
    const MotionDataFrame& frame = frameData();
    validateSceneryPlacement(worldTransform, boundingRadius);
    return isSphereVisibleInView(multiplyMatrix4(frame.currentView, worldTransform),
                                 {0.0, 0.0, 0.0}, boundingRadius);
}

bool DiligentFrameRenderer::isSphereVisibleInView(const Matrix4& modelView,
                                                  const std::array<double, 3>& localCenter,
                                                  double localRadius) const {
    impl_->requireActiveScene("View-space sphere visibility test");
    if (!std::isfinite(localRadius) || localRadius < 0.0 ||
        !std::all_of(
            modelView.begin(), modelView.end(),
            [](double value) { return std::isfinite(value); }) ||
        !std::all_of(
            localCenter.begin(), localCenter.end(),
            [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("Diligent view-space sphere bounds must be finite");
    }
    const std::array<double, 4> center =
        transformPoint(modelView, {localCenter[0], localCenter[1], localCenter[2], 1.0});
    const double transformScale =
        (std::max)({std::hypot(modelView[0], modelView[1], modelView[2]),
                    std::hypot(modelView[4], modelView[5], modelView[6]),
                    std::hypot(modelView[8], modelView[9], modelView[10])});
    const double viewRadius = localRadius * transformScale;
    if (!std::isfinite(transformScale) || !std::isfinite(viewRadius) ||
        !std::all_of(center.begin(), center.end(),
                     [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("Diligent view-space sphere exceeds numeric range");
    }
    return !sphereOutsideFrustum(buildViewFrustum(frameData().currentProjection), center,
                                 viewRadius);
}

void DiligentFrameRenderer::prepareVehicleAsset(
    std::uint64_t vehicleObjectId, std::uint64_t assetGeneration,
    std::uint64_t materialSelectionGeneration,
    std::span<const DiligentVehicleBatchSource> batches) {
    if (impl_->sceneActive) {
        throw std::logic_error("Cannot prepare a Diligent vehicle asset during scene drawing");
    }
    if (vehicleObjectId == 0) {
        throw std::invalid_argument("Diligent vehicle assets require a stable object identifier");
    }

    auto cached = impl_->vehicleAssets.find(vehicleObjectId);
    if (cached != impl_->vehicleAssets.end() && cached->second.asset &&
        cached->second.assetGeneration == assetGeneration) {
        cached->second.materialSelectionGeneration = materialSelectionGeneration;
        return;
    }
    if (batches.empty()) {
        throw std::invalid_argument("Diligent vehicle asset preparation requires batch sources");
    }
    auto asset = std::make_unique<DiligentVehicleAsset>(impl_->frame.device(), batches,
                                                        impl_->generateMipmaps);

    if (cached == impl_->vehicleAssets.end()) {
        Impl::VehicleAssetRecord record;
        record.asset = std::move(asset);
        record.runtimeTextures =
            std::make_unique<openbus::app::vehicle::DiligentVehicleRuntimeTextures>(
                impl_->frame.device());
        record.assetGeneration = assetGeneration;
        record.materialSelectionGeneration = materialSelectionGeneration;
        impl_->vehicleAssets.emplace(vehicleObjectId, std::move(record));
    } else {
        cached->second.asset = std::move(asset);
        cached->second.assetGeneration = assetGeneration;
        cached->second.materialSelectionGeneration = materialSelectionGeneration;
        if (!cached->second.runtimeTextures) {
            cached->second.runtimeTextures =
                std::make_unique<openbus::app::vehicle::DiligentVehicleRuntimeTextures>(
                    impl_->frame.device());
        }
    }
}

bool DiligentFrameRenderer::vehicleAssetCurrent(std::uint64_t vehicleObjectId,
                                                std::uint64_t assetGeneration) const noexcept {
    const auto cached = impl_->vehicleAssets.find(vehicleObjectId);
    return cached != impl_->vehicleAssets.end() && cached->second.asset &&
           cached->second.assetGeneration == assetGeneration;
}

const DiligentVehicleAsset*
DiligentFrameRenderer::vehicleAsset(std::uint64_t vehicleObjectId) const noexcept {
    const auto cached = impl_->vehicleAssets.find(vehicleObjectId);
    return cached == impl_->vehicleAssets.end() ? nullptr : cached->second.asset.get();
}

std::size_t
DiligentFrameRenderer::retainVehicleAssets(std::span<const std::uint64_t> vehicleObjectIds) {
    if (impl_->sceneActive) {
        throw std::logic_error("Cannot evict Diligent vehicle assets during scene drawing");
    }
    std::unordered_set<std::uint64_t> retained;
    retained.reserve(vehicleObjectIds.size());
    for (const std::uint64_t vehicleObjectId : vehicleObjectIds) {
        if (vehicleObjectId == 0) {
            throw std::invalid_argument(
                "Diligent vehicle retention requires valid object identifiers");
        }
        retained.insert(vehicleObjectId);
    }
    std::size_t removed = 0;
    for (auto vehicle = impl_->vehicleAssets.begin(); vehicle != impl_->vehicleAssets.end();) {
        if (retained.contains(vehicle->first)) {
            ++vehicle;
        } else {
            vehicle = impl_->vehicleAssets.erase(vehicle);
            ++removed;
        }
    }
    return removed;
}

bool DiligentFrameRenderer::buildVehicleMaterialOverrides(
    Vehicle& vehicle, const Matrix4& vehicleRootModelView,
    openbus::app::vehicle::DiligentVehicleMaterialOverrideSet& overrides) {
    impl_->requireActiveScene("Vehicle material overrides");
    const auto cached = impl_->vehicleAssets.find(vehicle.motionObjectId);
    if (cached == impl_->vehicleAssets.end() || !cached->second.asset ||
        cached->second.assetGeneration != vehicle.diligentAssetGeneration ||
        cached->second.materialSelectionGeneration != vehicle.materialSelectionGeneration) {
        throw std::logic_error(
            "Diligent vehicle assets must be prepared for the current material generation");
    }

    Impl::VehicleAssetRecord& record = cached->second;
    ScriptRuntime* const scriptRuntime = vehicle.scripts.get();
    if (record.scriptRuntime != scriptRuntime) {
        record.runtimeTextures =
            std::make_unique<openbus::app::vehicle::DiligentVehicleRuntimeTextures>(
                impl_->frame.device());
        record.scriptRuntime = scriptRuntime;
    }

    std::vector<DiligentVehicleMaterialOverride> runtimeOverrides;
    if (scriptRuntime != nullptr) {
        if (!record.runtimeTextures->synchronize(vehicle.displayLists, *scriptRuntime,
                                                 impl_->frame.immediateContext(),
                                                 runtimeOverrides)) {
            overrides = {};
            return false;
        }
    } else {
        const bool hasRuntimeTextures =
            std::any_of(vehicle.displayLists.begin(), vehicle.displayLists.end(),
                        [](const Vehicle::DisplayPart& part) {
                            return std::any_of(part.batches.begin(), part.batches.end(),
                                               [](const Vehicle::Batch& batch) {
                                                   return batch.scriptTextureIndex >= 0 ||
                                                          batch.textTextureIndex >= 0;
                                               });
                        });
        if (hasRuntimeTextures) {
            overrides = {};
            return false;
        }
    }

    overrides = openbus::app::vehicle::buildDiligentVehicleMaterialOverrides(
        vehicle, vehicleRootModelView, runtimeOverrides);
    return true;
}

void DiligentFrameRenderer::drawVehicle(
    const DiligentVehicleAsset& asset, Diligent::ITextureView& fallbackTexture,
    const Matrix4& modelViewProjection, const Matrix4& modelView,
    const DiligentLightingParameters& lighting, const DiligentMotionParameters* motion,
    std::span<const DiligentVehicleMaterialOverride> materialOverrides,
    Diligent::ITextureView* normalMap,
    std::span<const DiligentVehicleGroupTransform> groupTransforms) {
    impl_->requireActiveScene("Vehicle draw");
    asset.draw(impl_->scene.meshes(), impl_->frame.immediateContext(), fallbackTexture,
               modelViewProjection, modelView, lighting, motion, materialOverrides, normalMap,
               groupTransforms);
}

std::size_t DiligentFrameRenderer::drawVehiclePass(
    const DiligentVehicleAsset& asset, Diligent::ITextureView& fallbackTexture,
    const Matrix4& modelViewProjection, const Matrix4& modelView, DiligentVehicleRenderPass pass,
    const DiligentLightingParameters& lighting, const DiligentMotionParameters* motion,
    std::span<const DiligentVehicleMaterialOverride> materialOverrides,
    Diligent::ITextureView* normalMap,
    std::span<const DiligentVehicleGroupTransform> groupTransforms) {
    impl_->requireActiveScene("Vehicle draw pass");
    return asset.drawPass(impl_->scene.meshes(), impl_->frame.immediateContext(), fallbackTexture,
                          modelViewProjection, modelView, pass, lighting, motion, materialOverrides,
                          normalMap, groupTransforms);
}

DiligentVehicleGroupTransform DiligentFrameRenderer::makeVehicleGroupTransform(
    std::size_t animationGroupIndex, std::uint64_t vehicleObjectId, const Matrix4& currentModelView,
    bool geometryHistoryValid, bool visible, double maximumTranslationDeltaMeters) {
    impl_->requireActiveScene("Vehicle group transform");
    if (vehicleObjectId == 0) {
        throw std::invalid_argument("Diligent vehicle motion requires a stable object identifier");
    }
    if (visible && impl_->activeMotionHistory == nullptr) {
        throw std::logic_error("Diligent vehicle group motion requires managed frame history");
    }
    const MotionDataFrame& frame = impl_->scene.frameData();
    if (!visible) {
        DiligentVehicleGroupTransform transform;
        transform.animationGroupIndex = animationGroupIndex;
        transform.modelView = currentModelView;
        transform.modelViewProjection = multiplyMatrix4(frame.currentProjection, currentModelView);
        transform.visible = false;
        return transform;
    }
    return DiligentVehicleGroupTransform::fromMotionHistory(
        animationGroupIndex, frame, *impl_->activeMotionHistory,
        {vehicleObjectId, static_cast<std::uint64_t>(animationGroupIndex)}, currentModelView,
        geometryHistoryValid, maximumTranslationDeltaMeters);
}

void DiligentFrameRenderer::drawHuman(const DiligentHumanSubmesh& submesh,
                                      Diligent::ITextureView& fallbackTexture,
                                      const Matrix4& modelViewProjection,
                                      const HumanBonePalette& currentPose,
                                      const HumanBonePalette* previousPose, bool seated,
                                      const DiligentLightingParameters& lighting,
                                      const DiligentMotionParameters* motion) {
    impl_->requireActiveScene("Human draw");
    submesh.draw(impl_->scene.meshes(), impl_->frame.immediateContext(), fallbackTexture,
                 modelViewProjection, currentPose, previousPose, seated, lighting, motion);
}

void DiligentFrameRenderer::drawHumanAsset(
    const DiligentHumanAsset& asset, Diligent::ITextureView& fallbackTexture,
    const Matrix4& modelViewProjection, const Matrix4& modelView,
    const HumanBonePalette& currentPose, const HumanBonePalette* previousPose, bool seated,
    const DiligentLightingParameters& lighting, const DiligentMotionParameters* motion) {
    impl_->requireActiveScene("Human asset draw");
    asset.draw(impl_->scene.meshes(), impl_->frame.immediateContext(), fallbackTexture,
               modelViewProjection, modelView, currentPose, previousPose, seated, lighting, motion);
}

std::size_t DiligentFrameRenderer::drawHumanAssetPass(
    const DiligentHumanAsset& asset, Diligent::ITextureView& fallbackTexture,
    const Matrix4& modelViewProjection, const Matrix4& modelView,
    const HumanBonePalette& currentPose, const HumanBonePalette* previousPose, bool seated,
    DiligentHumanRenderPass pass, const DiligentLightingParameters& lighting,
    const DiligentMotionParameters* motion) {
    impl_->requireActiveScene("Human asset pass");
    return asset.drawPass(impl_->scene.meshes(), impl_->frame.immediateContext(), fallbackTexture,
                          modelViewProjection, modelView, currentPose, previousPose, seated, pass,
                          lighting, motion);
}

void DiligentFrameRenderer::prepareHumanAsset(
    std::string_view modelKey, std::uint64_t sourceGeneration, const HumanMeshGeometry& geometry,
    const std::unordered_map<int, std::shared_ptr<const Image>>& textures,
    const std::unordered_map<int, DiligentMaterialState>& materials) {
    if (impl_->sceneActive) {
        throw std::logic_error("Cannot prepare a Diligent human asset during scene drawing");
    }
    std::string key = normalizeHumanAssetKey(modelKey);
    auto existing = impl_->humanAssets.find(key);
    if (existing != impl_->humanAssets.end() &&
        existing->second.sourceGeneration == sourceGeneration) {
        return;
    }

    auto asset = std::make_unique<DiligentHumanAsset>(impl_->frame.device(), geometry, textures,
                                                      materials, sourceGeneration);
    Impl::HumanAssetRecord record;
    record.asset = std::move(asset);
    record.sourceGeneration = sourceGeneration;
    impl_->humanAssets.insert_or_assign(std::move(key), std::move(record));
}

bool DiligentFrameRenderer::humanAssetCurrent(std::string_view modelKey,
                                              std::uint64_t sourceGeneration) const {
    const auto existing = impl_->humanAssets.find(normalizeHumanAssetKey(modelKey));
    return existing != impl_->humanAssets.end() &&
           existing->second.sourceGeneration == sourceGeneration;
}

const DiligentHumanAsset* DiligentFrameRenderer::humanAsset(std::string_view modelKey) const {
    const auto existing = impl_->humanAssets.find(normalizeHumanAssetKey(modelKey));
    return existing != impl_->humanAssets.end() ? existing->second.asset.get() : nullptr;
}

std::size_t DiligentFrameRenderer::retainHumanAssets(std::span<const std::string> modelKeys) {
    if (impl_->sceneActive) {
        throw std::logic_error("Cannot evict Diligent human assets during scene drawing");
    }
    std::unordered_set<std::string> retainedKeys;
    retainedKeys.reserve(modelKeys.size());
    for (const std::string& modelKey : modelKeys) {
        retainedKeys.insert(normalizeHumanAssetKey(modelKey));
    }
    const std::size_t previousSize = impl_->humanAssets.size();
    std::erase_if(impl_->humanAssets, [&retainedKeys](const auto& entry) {
        return !retainedKeys.contains(entry.first);
    });
    return previousSize - impl_->humanAssets.size();
}

DiligentHumanDrawResult DiligentFrameRenderer::drawHumanPlacement(
    const DiligentHumanAsset& asset, const Matrix4& worldTransform, double boundingRadius,
    Diligent::ITextureView& fallbackTexture, DiligentHumanPoseKey poseKey,
    MotionObjectKey motionKey, const HumanBonePalette& currentPose, bool seated,
    const DiligentLightingParameters& lighting, bool geometryHistoryValid, bool visible,
    DiligentHumanRenderPass pass) {
    impl_->requireActiveScene("Human placement draw");
    if (!isValidHumanPass(pass)) {
        throw std::invalid_argument("Diligent human render pass is invalid");
    }
    if (poseKey.objectId == 0 || motionKey.isStatic() || poseKey.objectId != motionKey.object ||
        poseKey.subObjectId != motionKey.subObject) {
        throw std::invalid_argument(
            "Diligent human pose and motion keys must identify the same object");
    }
    if (!std::isfinite(boundingRadius) || boundingRadius < 0.0) {
        throw std::invalid_argument(
            "Diligent human placement radius must be finite and non-negative");
    }
    if (!std::all_of(worldTransform.begin(), worldTransform.end(),
                     [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("Diligent human placement transform must be finite");
    }
    if (!visible || asset.submeshes().empty()) {
        return {};
    }
    const MotionDataFrame& frame = impl_->scene.frameData();
    const Matrix4 modelView = multiplyMatrix4(frame.currentView, worldTransform);
    const std::array<double, 4> center = transformPoint(modelView, {0.0, 0.0, 0.0, 1.0});
    const double transformScale =
        (std::max)({std::hypot(worldTransform[0], worldTransform[1], worldTransform[2]),
                    std::hypot(worldTransform[4], worldTransform[5], worldTransform[6]),
                    std::hypot(worldTransform[8], worldTransform[9], worldTransform[10])});
    const double viewRadius = boundingRadius * transformScale;
    if (!std::isfinite(transformScale) || !std::isfinite(viewRadius) ||
        !std::all_of(center.begin(), center.end(),
                     [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("Diligent human placement bounds exceed numeric range");
    }
    if (sphereOutsideFrustum(buildViewFrustum(frame.currentProjection), center, viewRadius)) {
        DiligentHumanDrawResult result;
        result.culled = true;
        return result;
    }
    const Matrix4 modelViewProjection = multiplyMatrix4(frame.currentProjection, modelView);
    return drawHumanAssetWithHistory(asset, fallbackTexture, modelViewProjection, modelView,
                                     poseKey, motionKey, currentPose, seated, lighting,
                                     geometryHistoryValid, pass);
}

DiligentHumanDrawResult DiligentFrameRenderer::drawHumanAssetWithHistory(
    const DiligentHumanAsset& asset, Diligent::ITextureView& fallbackTexture,
    const Matrix4& modelViewProjection, const Matrix4& modelView, DiligentHumanPoseKey poseKey,
    MotionObjectKey motionKey, const HumanBonePalette& currentPose, bool seated,
    const DiligentLightingParameters& lighting, bool geometryHistoryValid,
    DiligentHumanRenderPass pass) {
    impl_->requireActiveScene("Human asset history draw");
    if (!isValidHumanPass(pass)) {
        throw std::invalid_argument("Diligent human render pass is invalid");
    }
    DiligentHumanDrawResult result;
    if (asset.submeshes().empty()) {
        return result;
    }
    if (!asset.hasSubmeshesForPass(pass)) {
        return result;
    }
    try {
        const DiligentHumanPoseHistoryResult poseHistory =
            impl_->humanPoseHistory.recordRenderedPose(poseKey, currentPose, geometryHistoryValid,
                                                       seated, asset.sourceGeneration());
        result.poseHistoryValid = poseHistory.historyValid;
        std::optional<DiligentMotionParameters> motion;
        if (impl_->activeMotionHistory != nullptr) {
            MotionTransform transform = impl_->activeMotionHistory->previousModelView(
                modelView, motionKey, geometryHistoryValid);
            transform.valid = transform.valid && poseHistory.historyValid;
            motion = DiligentMotionParameters::fromHistory(impl_->scene.frameData(), transform);
            result.motionHistoryValid = motion->historyValid;
        }
        const std::size_t submeshesDrawn =
            asset.drawPass(impl_->scene.meshes(), impl_->frame.immediateContext(), fallbackTexture,
                           modelViewProjection, modelView, currentPose, poseHistory.previousPose,
                           seated, pass, lighting, motion ? &*motion : nullptr);
        result.drawn = submeshesDrawn != 0;
    } catch (...) {
        impl_->humanPoseHistory.invalidate();
        if (impl_->activeMotionHistory != nullptr) {
            impl_->activeMotionHistory->invalidate();
        }
        impl_->invalidateMotionHistoryOnNextFrame = true;
        throw;
    }
    return result;
}

DiligentMotionRenderTargets& DiligentFrameRenderer::targets() noexcept {
    return impl_->scene.targets();
}

DiligentColorFilter DiligentFrameRenderer::defaultColorFilter() const noexcept {
    return impl_->defaultColorFilter;
}

void DiligentFrameRenderer::setDefaultColorFilter(DiligentColorFilter filter,
                                                  bool multisamplingRequested) {
    switch (filter) {
    case DiligentColorFilter::None:
    case DiligentColorFilter::Fxaa:
    case DiligentColorFilter::Smaa:
        break;
    default:
        throw std::invalid_argument("Invalid Diligent default color filter");
    }
    const bool multisamplingUnavailable =
        multisamplingRequested && impl_->scene.targets().samples() == 1;
    impl_->defaultColorFilter = multisamplingUnavailable ? DiligentColorFilter::Fxaa : filter;
}

const MotionDataFrame& DiligentFrameRenderer::frameData() const {
    impl_->requireActiveScene("Frame metadata access");
    return impl_->scene.frameData();
}

const DiligentSceneEnvironment& DiligentFrameRenderer::sceneEnvironment() const {
    impl_->requireActiveScene("Scene environment access");
    if (!impl_->activeEnvironment.has_value()) {
        throw std::logic_error(
            "The active Diligent frame was not started with a scene environment");
    }
    return *impl_->activeEnvironment;
}

bool DiligentFrameRenderer::isSuspended() const noexcept {
    return impl_->frame.isSuspended();
}

void DiligentFrameRenderer::resize(Diligent::Uint32 width, Diligent::Uint32 height) {
    if (impl_->sceneActive) {
        throw std::logic_error("Cannot resize the Diligent frame renderer during scene drawing");
    }
    const bool wasSuspended = impl_->frame.isSuspended();
    const bool dimensionsChanged =
        width != impl_->scene.targets().width() || height != impl_->scene.targets().height();
    impl_->frame.resize(width, height);
    if (width > 0 && height > 0) {
        impl_->scene.resize(impl_->frame.immediateContext(), width, height);
    }
    if (dimensionsChanged || wasSuspended || width == 0 || height == 0) {
        impl_->scene.invalidateHistory();
        impl_->humanPoseHistory.invalidate();
        impl_->invalidateMotionHistoryOnNextFrame = true;
    }
}

void DiligentFrameRenderer::invalidateHistory() {
    if (impl_->sceneActive) {
        throw std::logic_error("Cannot invalidate Diligent frame history during scene drawing");
    }
    impl_->scene.invalidateHistory();
    impl_->humanPoseHistory.invalidate();
    impl_->invalidateMotionHistoryOnNextFrame = true;
}

void DiligentFrameRenderer::setVerticalSync(bool enabled) noexcept {
    impl_->frame.setVerticalSync(enabled);
}

void DiligentFrameRenderer::waitForIdle() {
    impl_->frame.waitForIdle();
}

bool DiligentFrameRenderer::beginFrame(const MotionDataFrame& frame,
                                       const std::array<float, 4>& clearColor) {
    if (impl_->sceneActive) {
        throw std::logic_error("The previous Diligent scene frame has not been completed");
    }
    if (impl_->frame.isSuspended()) {
        impl_->activeEnvironment.reset();
        impl_->humanPoseHistory.invalidate();
        impl_->invalidateMotionHistoryOnNextFrame = true;
        return false;
    }
    impl_->humanPoseHistory.beginFrame(frame);
    impl_->activeEnvironment.reset();
    try {
        Diligent::IDeviceContext& context = impl_->frame.immediateContext();
        impl_->scene.beginScene(context, frame, clearColor);
        if (impl_->defaultAnisotropy != 0) {
            impl_->scene.meshes().setDefaultAnisotropy(impl_->defaultAnisotropy);
        }
        impl_->scene.meshes().setSpotlightShadowMapView(
            impl_->spotlightShadowMaps != nullptr
                ? &impl_->spotlightShadowMaps->shaderResourceView()
                : nullptr);
        impl_->scene.meshes().setPointLights(context, {});
        impl_->scene.meshes().setSpotLights(context, {});
    } catch (...) {
        impl_->humanPoseHistory.invalidate();
        throw;
    }
    impl_->sceneActive = true;
    return true;
}

bool DiligentFrameRenderer::beginFrame(const MotionDataFrame& frame,
                                       const DiligentSceneEnvironment& environment) {
    if (!beginFrame(frame, environment.clearColor)) {
        return false;
    }
    impl_->activeEnvironment = environment;
    return true;
}

bool DiligentFrameRenderer::beginFrame(MotionHistory& history, std::uint64_t frameId,
                                       double timestampSeconds,
                                       const openbus::app::DiligentCameraView& camera,
                                       int displayWidth, int displayHeight,
                                       const std::array<float, 4>& clearColor) {
    return beginFrame(history, frameId, timestampSeconds, camera.viewportWidth,
                      camera.viewportHeight, camera.view, camera.projection, displayWidth,
                      displayHeight, clearColor);
}

bool DiligentFrameRenderer::beginFrame(MotionHistory& history, std::uint64_t frameId,
                                       double timestampSeconds,
                                       const openbus::app::DiligentCameraView& camera,
                                       int displayWidth, int displayHeight,
                                       const DiligentSceneEnvironment& environment) {
    return beginFrame(history, frameId, timestampSeconds, camera.viewportWidth,
                      camera.viewportHeight, camera.view, camera.projection, displayWidth,
                      displayHeight, environment);
}

bool DiligentFrameRenderer::beginFrame(MotionHistory& history, std::uint64_t frameId,
                                       double timestampSeconds, int width, int height,
                                       const Matrix4& view, const Matrix4& projection,
                                       int displayWidth, int displayHeight,
                                       const std::array<float, 4>& clearColor) {
    if (impl_->sceneActive) {
        throw std::logic_error("The previous Diligent scene frame has not been completed");
    }
    if (impl_->frame.isSuspended()) {
        history.invalidate();
        impl_->invalidateMotionHistoryOnNextFrame = true;
        return false;
    }
    if (impl_->invalidateMotionHistoryOnNextFrame) {
        history.invalidate();
    }
    history.beginFrame(frameId, timestampSeconds, width, height, view, projection, displayWidth,
                       displayHeight);
    try {
        if (!beginFrame(history.frame(), clearColor)) {
            history.invalidate();
            impl_->invalidateMotionHistoryOnNextFrame = true;
            return false;
        }
    } catch (...) {
        history.invalidate();
        impl_->invalidateMotionHistoryOnNextFrame = true;
        throw;
    }
    impl_->activeMotionHistory = &history;
    impl_->invalidateMotionHistoryOnNextFrame = false;
    return true;
}

bool DiligentFrameRenderer::beginFrame(MotionHistory& history, std::uint64_t frameId,
                                       double timestampSeconds, int width, int height,
                                       const Matrix4& view, const Matrix4& projection,
                                       int displayWidth, int displayHeight,
                                       const DiligentSceneEnvironment& environment) {
    if (!beginFrame(history, frameId, timestampSeconds, width, height, view, projection,
                    displayWidth, displayHeight, environment.clearColor)) {
        return false;
    }
    impl_->activeEnvironment = environment;
    return true;
}

void DiligentFrameRenderer::beginUiFrame(bool manageMouseCursor) {
    if (impl_->uiActive) {
        throw std::logic_error("Diligent UI frame has already started");
    }
    impl_->gui.beginFrame(manageMouseCursor);
    impl_->uiActive = true;
}

void DiligentFrameRenderer::endUiOnlyFrame(const std::array<float, 4>& clearColor) {
    if (impl_->sceneActive) {
        throw std::logic_error("Cannot present a UI-only frame during scene drawing");
    }
    if (!impl_->uiActive) {
        throw std::logic_error("Diligent UI-only frame has not been started");
    }
    if (impl_->frame.isSuspended()) {
        cancelUiFrame();
        return;
    }
    if (!std::all_of(clearColor.begin(), clearColor.end(),
                     [](float value) { return std::isfinite(value); })) {
        cancelUiFrame();
        throw std::invalid_argument("Diligent UI clear color must be finite");
    }

    try {
        Diligent::IDeviceContext& context = impl_->frame.immediateContext();
        auto& target = impl_->frame.backBufferRenderTarget();
        context.ClearRenderTarget(&target, clearColor.data(),
                                  Diligent::RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        impl_->gui.render(context, target);
        impl_->frame.present();
    } catch (...) {
        cancelUiFrame();
        throw;
    }
    impl_->uiActive = false;
}

void DiligentFrameRenderer::cancelUiFrame() noexcept {
    if (impl_ && impl_->uiActive) {
        impl_->gui.finishFrameWithoutRendering();
        impl_->uiActive = false;
    }
}

void DiligentFrameRenderer::captureNextFrame(const std::filesystem::path& path) {
    if (path.empty()) {
        throw std::invalid_argument("Diligent screenshot path must not be empty");
    }
    if (impl_->sceneActive || impl_->reflectionPassActive) {
        throw std::logic_error("Diligent screenshots must be requested before scene submission");
    }
    if (impl_->capturePath.has_value()) {
        throw std::logic_error("A Diligent screenshot is already pending");
    }
    impl_->capturePath = path;
}

bool DiligentFrameRenderer::hasPendingCapture() const noexcept {
    return impl_ != nullptr && impl_->capturePath.has_value();
}

void DiligentFrameRenderer::cancelPendingCapture() noexcept {
    if (impl_) {
        impl_->capturePath.reset();
    }
}

void DiligentFrameRenderer::present() {
    endFrame();
}

void DiligentFrameRenderer::abortFrame() noexcept {
    if (!impl_) {
        return;
    }
    if (!impl_->sceneActive) {
        cancelUiFrame();
        return;
    }
    if (impl_->uiActive) {
        impl_->gui.finishFrameWithoutRendering();
    }
    impl_->scene.abortFrame();
    impl_->sceneActive = false;
    impl_->reflectionPassActive = false;
    impl_->uiActive = false;
    impl_->activeEnvironment.reset();
    impl_->capturePath.reset();
    impl_->humanPoseHistory.invalidate();
    if (impl_->activeMotionHistory != nullptr) {
        impl_->activeMotionHistory->invalidate();
        impl_->activeMotionHistory = nullptr;
    }
    impl_->invalidateMotionHistoryOnNextFrame = true;
}

void DiligentFrameRenderer::endFrame() {
    endFrame(impl_->defaultColorFilter);
}

void DiligentFrameRenderer::endFrame(DiligentColorFilter filter) {
    endFrame(filter, DiligentDebugView::None);
}

void DiligentFrameRenderer::endFrame(DiligentColorFilter filter, DiligentDebugView debugView) {
    impl_->requireActiveScene("Frame completion");
    try {
        Diligent::IDeviceContext& context = impl_->frame.immediateContext();
        impl_->scene.finishScene(context, impl_->frame.backBufferRenderTarget(), filter, debugView);
        if (impl_->uiActive) {
            impl_->gui.render(context, impl_->frame.backBufferRenderTarget());
        }
        if (impl_->capturePath.has_value()) {
            const std::filesystem::path path = std::move(*impl_->capturePath);
            impl_->capturePath.reset();
            impl_->captureBackBuffer(path);
        }
        impl_->scene.completeFrame();
        impl_->frame.present();
        impl_->humanPoseHistory.completeFrame();
        if (impl_->activeMotionHistory != nullptr) {
            impl_->activeMotionHistory->completeFrame();
            impl_->activeMotionHistory = nullptr;
        }
    } catch (...) {
        abortFrame();
        throw;
    }
    impl_->uiActive = false;
    impl_->sceneActive = false;
    impl_->activeEnvironment.reset();
}

} // namespace openbus::rendering
