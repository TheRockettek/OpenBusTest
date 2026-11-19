#include "render/diligent/DiligentMapTileSet.h"
#include "render/diligent/DiligentSceneryAsset.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#define GLFW_INCLUDE_NONE
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>
#include <imgui.h>
#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/DeviceContext.h>
#include <Graphics/GraphicsEngine/interface/RenderDevice.h>
#include <Graphics/GraphicsEngine/interface/Texture.h>
#include <Graphics/GraphicsEngine/interface/TextureView.h>

#include "app/DiligentFrameRenderer.h"
#include "app/DiligentCameraBridge.h"
#include "app/DiligentGraphicsPreferences.h"
#include "app/DiligentHumanBridge.h"
#include "app/DiligentMapBridge.h"
#include "app/PassengerCoinGeometry.h"
#include "app/DiligentSceneEnvironmentBridge.h"
#include "app/DiligentWindowBridge.h"
#include "app/traffic/PassengerMotionKeys.h"
#include "app/vehicle/DiligentVehicleBridge.h"
#include "app/vehicle/DiligentVehicleSceneBridge.h"
#include "config/ModelConfigTypes.h"
#include "config/VehicleConfigLoader.h"
#include "core/Variables.h"
#include "core/Viewpoint.h"
#include "map/MapConfigLoader.h"
#include "map/MapRoadGeometry.h"
#include "render/AssetRequestManager.h"
#include "render/map/MapRenderer.h"
#include "render/diligent/DiligentHumanAsset.h"
#include "render/diligent/DiligentMapLighting.h"
#include "render/diligent/DiligentSceneLighting.h"
#include "render/diligent/DiligentDynamicTexture2D.h"
#include "render/diligent/DiligentMapTileAsset.h"
#include "render/diligent/DiligentReflectionTarget.h"
#include "render/diligent/DiligentSpotlightShadowMaps.h"
#include "render/diligent/DiligentTexture2D.h"
#include "render/diligent/DiligentTexture2DArray.h"
#include "render/diligent/DiligentTexturedMesh.h"
#include "render/diligent/DiligentVehicleAsset.h"
#include "render/SpotlightShadowCamera.h"
#include "scripting/ScriptRuntime.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>

void runDiligentVehicleSourceBridgeProbe(Diligent::ITextureView& reflectionTexture);

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

struct ScrollCallbackState {
    int calls = 0;
    double horizontal = 0.0;
    double vertical = 0.0;
};

void recordScrollCallback(GLFWwindow* window, double horizontal, double vertical) {
    auto* state = static_cast<ScrollCallbackState*>(glfwGetWindowUserPointer(window));
    if (state != nullptr) {
        ++state->calls;
        state->horizontal = horizontal;
        state->vertical = vertical;
    }
}

void verifyAlphaOnlyVehicleSampling(Diligent::IRenderDevice& device,
                                    Diligent::IDeviceContext& context) {
    using namespace Diligent;
    using namespace openbus::rendering;
    constexpr Uint32 size = 16;
    TextureDesc colorDesc;
    colorDesc.Name = "Diligent alpha-only vehicle probe target";
    colorDesc.Type = RESOURCE_DIM_TEX_2D;
    colorDesc.Width = size;
    colorDesc.Height = size;
    colorDesc.MipLevels = 1;
    colorDesc.Format = TEX_FORMAT_RGBA8_UNORM;
    colorDesc.BindFlags = BIND_RENDER_TARGET;
    RefCntAutoPtr<ITexture> colorTarget;
    device.CreateTexture(colorDesc, nullptr, &colorTarget);
    require(static_cast<bool>(colorTarget), "Failed to create alpha-only probe color target");

    TextureDesc depthDesc = colorDesc;
    depthDesc.Name = "Diligent alpha-only vehicle probe depth";
    depthDesc.Format = TEX_FORMAT_D32_FLOAT;
    depthDesc.BindFlags = BIND_DEPTH_STENCIL;
    RefCntAutoPtr<ITexture> depthTarget;
    device.CreateTexture(depthDesc, nullptr, &depthTarget);
    require(static_cast<bool>(depthTarget), "Failed to create alpha-only probe depth target");

    TextureDesc stagingDesc = colorDesc;
    stagingDesc.Name = "Diligent alpha-only vehicle probe readback";
    stagingDesc.Usage = USAGE_STAGING;
    stagingDesc.BindFlags = BIND_NONE;
    stagingDesc.CPUAccessFlags = CPU_ACCESS_READ;
    RefCntAutoPtr<ITexture> staging;
    device.CreateTexture(stagingDesc, nullptr, &staging);
    require(static_cast<bool>(staging), "Failed to create alpha-only probe staging texture");

    auto* rtv = colorTarget->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET);
    auto* dsv = depthTarget->GetDefaultView(TEXTURE_VIEW_DEPTH_STENCIL);
    require(rtv != nullptr && dsv != nullptr, "Alpha-only probe target view is missing");
    const std::array<ModelVertex, 6> vertices = {{
        {-0.8F, -0.8F, 0.5F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {0.8F, -0.8F, 0.5F, 1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {0.8F, 0.8F, 0.5F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {-0.8F, -0.8F, 0.5F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {0.8F, 0.8F, 0.5F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {-0.8F, 0.8F, 0.5F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F},
    }};
    DiligentTexturedMesh mesh(device, vertices);
    DiligentTexturedMeshRenderer meshRenderer(device, colorDesc.Format, depthDesc.Format);
    DiligentTexture2D texture(device, Image{1, 1, {255, 0, 0, 255}});
    DiligentMaterialState material;
    material.baseColor = {0.0F, 1.0F, 1.0F, 1.0F};
    material.textureAlphaOnly = true;

    context.SetRenderTargets(1, &rtv, dsv, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    const float clear[] = {0.0F, 0.0F, 0.0F, 1.0F};
    context.ClearRenderTarget(rtv, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(dsv, CLEAR_DEPTH_FLAG, 1.0F, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    const Viewport viewport{0.0F, 0.0F, static_cast<float>(size), static_cast<float>(size)};
    context.SetViewports(1, &viewport, 0, 0);
    meshRenderer.draw(context, mesh, texture.shaderResourceView(), identityMatrix(), material);
    context.SetRenderTargets(0, nullptr, nullptr, RESOURCE_STATE_TRANSITION_MODE_NONE);

    CopyTextureAttribs copy;
    copy.pSrcTexture = colorTarget;
    copy.SrcTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
    copy.pDstTexture = staging;
    copy.DstTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
    context.CopyTexture(copy);
    context.WaitForIdle();
    MappedTextureSubresource mapped;
    context.MapTextureSubresource(staging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT, nullptr, mapped);
    require(mapped.pData != nullptr, "Failed to read alpha-only probe pixels");
    const auto* pixel =
        static_cast<const std::uint8_t*>(mapped.pData) + 8 * mapped.Stride + 8 * 4;
    const bool colorPreserved = pixel[0] < 35 && pixel[1] > 220 && pixel[2] > 220;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(colorPreserved, "Diligent alpha-only vehicle texture did not preserve base RGB");
}

void runProbe(GLFWwindow& window, ScrollCallbackState& scrollState) {
    using namespace Diligent;
    using namespace openbus::rendering;

    const std::array<MapPointLight, 1> mapLightPolicyInput = {
        MapPointLight{{1.0, 2.0, 3.0}, {0.4, 0.5, 0.6}, 20.0}};
    const auto duskMapLights = buildDiligentMapPointLights(
        mapLightPolicyInput, identityMatrix(), {}, mapLightStrengthForEnvironmentBrightness(0.25),
        4);
    require(duskMapLights.size() == 1 &&
                std::abs(duskMapLights[0].intensity - 0.75F) < 1e-6F &&
                mapLightStrengthForEnvironmentBrightness(std::nullopt) == 0.0 &&
                mapLightStrengthForEnvironmentBrightness(1.0) == 0.0 &&
                mapLightStrengthForEnvironmentBrightness(0.0) == 1.0 &&
                mapLightStrengthForEnvironmentBrightness(1.5) == 0.0 &&
                mapLightStrengthForEnvironmentBrightness(-0.5) == 1.0,
            "Diligent map-light intensity did not match production weather brightness");

    DiligentMapWeather mapWeather{0.25F, 0.5F, 0.8F};
    DiligentMaterialState roadMaterial;
    const DiligentMaterialState wetRoadMaterial =
        applyDiligentRoadWeather(roadMaterial, true, true, mapWeather);
    require(std::abs(wetRoadMaterial.snowCoverage - 0.425F) < 1e-6F &&
                std::abs(wetRoadMaterial.baseColor[0] - 0.885F) < 1e-6F &&
                std::abs(wetRoadMaterial.specularColor[0] - 0.0828F) < 1e-6F &&
                wetRoadMaterial.specularPower == 48.0F,
            "Diligent road weather did not preserve production snow, moisture, and specular policy");
    const DiligentMaterialState dryClassifiedRoad =
        applyDiligentRoadWeather(roadMaterial, false, true, mapWeather);
    require(dryClassifiedRoad.snowCoverage == 0.0F &&
                std::abs(dryClassifiedRoad.baseColor[0] - 0.8F) < 1e-6F,
            "Unclassified Diligent road incorrectly received snow coverage");
    const DiligentMaterialState snowOnlyRoad =
        applyDiligentRoadWeather(roadMaterial, true, false, mapWeather);
    require(snowOnlyRoad.snowCoverage == 0.425F && snowOnlyRoad.baseColor[0] == 1.0F &&
                snowOnlyRoad.specularColor[0] == 0.0F,
            "Diligent road without moisture support received wet-surface shading");
    bool invalidRoadWeatherRejected = false;
    try {
        mapWeather.roadWater = 1.1F;
        static_cast<void>(applyDiligentRoadWeather(roadMaterial, true, true, mapWeather));
    } catch (const std::invalid_argument&) {
        invalidRoadWeatherRejected = true;
    }
    require(invalidRoadWeatherRejected,
            "Diligent road weather adapter accepted an out-of-range fraction");

    constexpr double reflectionNearPlane = 0.1;
    constexpr double reflectionFarPlane = 100.0;
    openbus::map::MapTrafficCameraView glReflectionView;
    glReflectionView.position = {1.0, 2.0, 3.0};
    glReflectionView.view = identityMatrix();
    glReflectionView.projection = {};
    glReflectionView.projection[0] = 1.5;
    glReflectionView.projection[5] = 2.0;
    glReflectionView.projection[10] =
        -(reflectionFarPlane + reflectionNearPlane) /
        (reflectionFarPlane - reflectionNearPlane);
    glReflectionView.projection[11] = -1.0;
    glReflectionView.projection[14] =
        -(2.0 * reflectionFarPlane * reflectionNearPlane) /
        (reflectionFarPlane - reflectionNearPlane);
    glReflectionView.viewProjection =
        multiplyMatrix4(glReflectionView.projection, glReflectionView.view);
    glReflectionView.viewportWidth = 640;
    glReflectionView.viewportHeight = 360;

    const auto diligentReflectionView = openbus::app::makeDiligentCameraView(glReflectionView);
    const auto nearClip = transformPoint(
        diligentReflectionView.projection, {0.0, 0.0, -reflectionNearPlane, 1.0});
    const auto farClip = transformPoint(
        diligentReflectionView.projection, {0.0, 0.0, -reflectionFarPlane, 1.0});
    require(std::abs(nearClip[2] / nearClip[3]) < 1e-9 &&
                std::abs(farClip[2] / farClip[3] - 1.0) < 1e-9,
            "Diligent reflection projection did not map OpenGL depth into [0, 1]");
    require(diligentReflectionView.projection[5] == glReflectionView.projection[5] &&
                diligentReflectionView.viewportWidth == glReflectionView.viewportWidth &&
                diligentReflectionView.viewportHeight == glReflectionView.viewportHeight &&
                diligentReflectionView.cameraPosition == glReflectionView.position,
            "Diligent reflection camera conversion changed Y projection or view metadata");
    require(diligentReflectionView.viewProjection ==
                multiplyMatrix4(diligentReflectionView.projection,
                                diligentReflectionView.view),
            "Diligent reflection camera conversion returned an inconsistent view-projection");

    bool incompleteReflectionViewRejected = false;
    glReflectionView.viewportWidth = 0;
    try {
        (void)openbus::app::makeDiligentCameraView(glReflectionView);
    } catch (const std::invalid_argument&) {
        incompleteReflectionViewRejected = true;
    }
    require(incompleteReflectionViewRejected,
            "Diligent reflection camera conversion accepted an incomplete viewport");

    openbus::map::MapTrafficCameraView identityGlReflectionView;
    identityGlReflectionView.view = identityMatrix();
    identityGlReflectionView.projection = identityMatrix();
    identityGlReflectionView.viewProjection = identityMatrix();
    identityGlReflectionView.viewportWidth = 64;
    identityGlReflectionView.viewportHeight = 64;
    const auto diligentAdapterReflectionView =
        openbus::app::makeDiligentCameraView(identityGlReflectionView);
    const auto diligentMainView = openbus::app::makeDiligentCameraView(
        identityGlReflectionView.view, identityGlReflectionView.projection,
        identityGlReflectionView.position, identityGlReflectionView.viewportWidth,
        identityGlReflectionView.viewportHeight);
    require(diligentMainView.viewProjection ==
                diligentAdapterReflectionView.viewProjection &&
                diligentMainView.viewportWidth == diligentAdapterReflectionView.viewportWidth &&
                diligentMainView.viewportHeight == diligentAdapterReflectionView.viewportHeight,
            "Diligent main-view conversion diverged from the production camera snapshot adapter");

    const auto poseCamera = openbus::app::makeDiligentCameraViewFromPose(
        {10.0, 20.0, 30.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, 320, 240, 60.0);
    const auto poseOrigin =
        transformPoint(poseCamera.view, {10.0, 20.0, 30.0, 1.0});
    const auto poseForward =
        transformPoint(poseCamera.view, {10.0, 21.0, 30.0, 1.0});
    const auto poseNearClip =
        transformPoint(poseCamera.projection, {0.0, 0.0, -0.1, 1.0});
    const auto poseFarClip =
        transformPoint(poseCamera.projection, {0.0, 0.0, -10000.0, 1.0});
    require(std::abs(poseOrigin[0]) < 1e-9 && std::abs(poseOrigin[1]) < 1e-9 &&
                std::abs(poseOrigin[2]) < 1e-9 && std::abs(poseForward[2] + 1.0) < 1e-9 &&
                std::abs(poseNearClip[2] / poseNearClip[3]) < 1e-9 &&
                std::abs(poseFarClip[2] / poseFarClip[3] - 1.0) < 1e-9 &&
                poseCamera.viewportWidth == 320 && poseCamera.viewportHeight == 240,
            "Backend-neutral camera pose did not produce a valid Diligent view");

    openbus::core::GraphicsPreferences graphicsPreferences;
    const auto defaultDiligentOptions =
        openbus::app::makeDiligentGraphicsOptions(graphicsPreferences);
    require(defaultDiligentOptions.multisampling.samples == 1 &&
                defaultDiligentOptions.colorFilter == DiligentColorFilter::None &&
                defaultDiligentOptions.anisotropy == 1 &&
                defaultDiligentOptions.verticalSync &&
                !defaultDiligentOptions.generateTextureMipmaps,
            "Diligent graphics options did not preserve default preferences");
    graphicsPreferences.antiAliasingMode = "fxaa";
    require(openbus::app::makeDiligentGraphicsOptions(graphicsPreferences).colorFilter ==
                DiligentColorFilter::Fxaa,
            "Diligent graphics options did not map FXAA");
    graphicsPreferences.antiAliasingMode = "smaa";
    require(openbus::app::makeDiligentGraphicsOptions(graphicsPreferences).colorFilter ==
                DiligentColorFilter::Smaa,
            "Diligent graphics options did not map SMAA");
    graphicsPreferences.antiAliasingMode = "msaa";
    graphicsPreferences.anisotropicFiltering = 16;
    graphicsPreferences.generateTextureMipmaps = true;
    graphicsPreferences.vsync = false;
    const auto msaaDiligentOptions =
        openbus::app::makeDiligentGraphicsOptions(graphicsPreferences);
    require(msaaDiligentOptions.multisampling.samples == 4 &&
                msaaDiligentOptions.anisotropy == 16 &&
                msaaDiligentOptions.generateTextureMipmaps &&
                !msaaDiligentOptions.verticalSync,
            "Diligent graphics options did not map MSAA and quality preferences");
    const auto selectedTwoSampleFallback =
        openbus::rendering::selectDiligentMultisamplingFromSupport(1U | 2U, {4});
    const auto selectedFxaaFallback =
        openbus::rendering::selectDiligentMultisamplingFromSupport(1U, {4});
    require(selectedTwoSampleFallback.multisampling.samples == 2 &&
                !selectedTwoSampleFallback.useFxaaFallback &&
                selectedFxaaFallback.multisampling.samples == 1 &&
                selectedFxaaFallback.useFxaaFallback,
            "Diligent MSAA fallback did not select 2x before FXAA");
    graphicsPreferences.antiAliasingMode = "invalid";
    bool invalidDiligentAaRejected = false;
    try {
        (void)openbus::app::makeDiligentGraphicsOptions(graphicsPreferences);
    } catch (const std::invalid_argument&) {
        invalidDiligentAaRejected = true;
    }
    require(invalidDiligentAaRejected,
            "Diligent graphics options accepted an unsupported anti-aliasing preference");
    graphicsPreferences.antiAliasingMode = "off";
    graphicsPreferences.anisotropicFiltering = 3;
    bool invalidDiligentAnisotropyRejected = false;
    try {
        (void)openbus::app::makeDiligentGraphicsOptions(graphicsPreferences);
    } catch (const std::invalid_argument&) {
        invalidDiligentAnisotropyRejected = true;
    }
    require(invalidDiligentAnisotropyRejected,
            "Diligent graphics options accepted an unsupported anisotropy preference");

    bool invalidVehicleAlphaRejected = false;
    require(openbus::app::vehicle::toDiligentVehicleAlphaMode(0) ==
                    DiligentAlphaMode::Opaque &&
                openbus::app::vehicle::toDiligentVehicleAlphaMode(1) ==
                    DiligentAlphaMode::Cutout &&
                openbus::app::vehicle::toDiligentVehicleAlphaMode(2) ==
                    DiligentAlphaMode::Blend &&
                openbus::app::vehicle::toDiligentVehicleAlphaMode(3) ==
                    DiligentAlphaMode::NearOpaque,
            "Production vehicle alpha modes did not map to Diligent material modes");
    try {
        (void)openbus::app::vehicle::toDiligentVehicleAlphaMode(4);
    } catch (const std::invalid_argument&) {
        invalidVehicleAlphaRejected = true;
    }
    require(invalidVehicleAlphaRejected,
            "Production vehicle batch bridge accepted an unsupported alpha mode");

    constexpr Matrix4 identity = {1, 0, 0, 0, 0, 1, 0, 0,
                                  0, 0, 1, 0, 0, 0, 0, 1};
    std::vector<openbus::rendering::PrimitiveVertex> ticketVertices;
    openbus::app::appendPassengerTicketCard(ticketVertices, {1.0, 2.0, 3.0}, 0.0, true, true);
    require(ticketVertices.size() == 12 &&
                std::abs(ticketVertices.front().x - 1.0F) < 1.0e-6F &&
                std::abs(ticketVertices.front().y - 2.153F) < 1.0e-6F &&
                std::abs(ticketVertices.front().z - 4.002F) < 1.0e-6F &&
                ticketVertices.front().r > ticketVertices[6].r,
            "Passenger ticket geometry lost its holder transform or printed-ticket details");
    constexpr Matrix4 translatedView = {1, 0, 0, 0, 0, 1, 0, 0,
                                        0, 0, 1, 0, -10, 0, 0, 1};
    const SpotlightShadowCamera worldShadowCamera{identity, identity, 0.05, 100.0};
    const auto identityShadowTransform =
        buildDiligentSpotlightShadowClipFromView(worldShadowCamera, identity, 0.0F);
    const auto translatedShadowTransform =
        buildDiligentSpotlightShadowClipFromView(worldShadowCamera, translatedView, 0.0F);
    require(identityShadowTransform.has_value() && translatedShadowTransform.has_value() &&
                *identityShadowTransform != *translatedShadowTransform,
            "Diligent spotlight shadow bridge did not include the active camera view");
    constexpr Matrix4 singularShadowView = {};
    require(!buildDiligentSpotlightShadowClipFromView(worldShadowCamera, singularShadowView,
                                                      0.0F)
                 .has_value(),
            "Diligent spotlight shadow bridge accepted a singular camera view");
    std::array<DiligentSpotLight, 1> shadowBoundLights;
    std::array<DiligentSpotlightShadowBinding, 1> shadowBindings = {
        DiligentSpotlightShadowBinding{worldShadowCamera, 7, 0.002F}};
    applyDiligentSpotlightShadowBindings(shadowBoundLights, shadowBindings, translatedView, 0.0F);
    require(shadowBoundLights[0].shadowValid && shadowBoundLights[0].shadowMapIndex == 7 &&
                shadowBoundLights[0].shadowBias == 0.002F &&
                shadowBoundLights[0].shadowClipFromView == *translatedShadowTransform,
            "Diligent spotlight shadow bindings did not preserve the scheduled layer and transform");
    shadowBindings[0].camera.reset();
    applyDiligentSpotlightShadowBindings(shadowBoundLights, shadowBindings, translatedView, 0.0F);
    require(!shadowBoundLights[0].shadowValid,
            "Diligent spotlight shadow binding retained a missing shadow camera");
    shadowBindings[0].camera = worldShadowCamera;
    applyDiligentSpotlightShadowBindings(shadowBoundLights, shadowBindings, singularShadowView,
                                         0.0F);
    require(!shadowBoundLights[0].shadowValid,
            "Diligent spotlight shadow binding remained valid for a singular view");
    bool mismatchedDiligentShadowBindingsRejected = false;
    try {
        applyDiligentSpotlightShadowBindings(shadowBoundLights, {}, translatedView, 0.0F);
    } catch (const std::invalid_argument&) {
        mismatchedDiligentShadowBindingsRejected = true;
    }
    require(mismatchedDiligentShadowBindingsRejected,
            "Diligent spotlight shadow binding accepted a mismatched light count");
    const auto daylight = buildDiligentSceneLighting(identity, std::nullopt, std::nullopt, false);
    require(daylight.directionalEnabled && daylight.lightDirectionView ==
                std::array<float, 3>{-0.35F, 0.5F, 0.8F} &&
                std::abs(daylight.ambientIntensity - 0.58F) < 1.0e-6F &&
                std::abs(daylight.diffuseIntensity - 0.42F) < 1.0e-6F &&
                daylight.weatherBrightness == 1.0F,
            "Diligent scene-light bridge did not preserve the production daylight policy");
    const auto nightLighting =
        buildDiligentSceneLighting(translatedView, std::array<double, 3>{0.0, 0.0, 1.0}, 0.0,
                                  false);
    require(nightLighting.lightDirectionView == std::array<float, 3>{0.0F, 0.0F, 1.0F} &&
                nightLighting.ambientIntensity == 0.0F &&
                nightLighting.diffuseIntensity == 0.0F &&
                nightLighting.weatherBrightness == 0.0F,
            "Diligent scene-light bridge did not preserve the production night policy");
    const auto reflectionLighting =
        buildDiligentSceneLighting(identity, std::array<double, 3>{1.0, 0.0, 0.0}, 0.0, true,
                                  0.3);
    require(reflectionLighting.ambientIntensity == 0.3F &&
                reflectionLighting.diffuseIntensity == 0.0F,
            "Diligent reflection lighting did not apply its ambient floor");
    bool invalidDiligentSunRejected = false;
    try {
        (void)buildDiligentSceneLighting(identity, std::array<double, 3>{0.0, 0.0, 0.0}, 1.0,
                                         false);
    } catch (const std::invalid_argument&) {
        invalidDiligentSunRejected = true;
    }
    require(invalidDiligentSunRejected,
            "Diligent scene-light bridge accepted a zero sun direction");
    bool invalidDiligentBrightnessRejected = false;
    try {
        (void)buildDiligentSceneLighting(identity, std::nullopt, 1.1, false);
    } catch (const std::invalid_argument&) {
        invalidDiligentBrightnessRejected = true;
    }
    require(invalidDiligentBrightnessRejected,
            "Diligent scene-light bridge accepted out-of-range weather brightness");
    bool invalidDiligentViewSunRejected = false;
    try {
        constexpr Matrix4 zeroView = {};
        (void)buildDiligentSceneLighting(zeroView, std::nullopt, 1.0, false);
    } catch (const std::invalid_argument&) {
        invalidDiligentViewSunRejected = true;
    }
    require(invalidDiligentViewSunRejected,
            "Diligent scene-light bridge accepted a view that collapses the sun direction");
    const auto clearWeatherFog =
        buildDiligentWeatherFog(identity, 0.0, std::nullopt, std::nullopt, std::nullopt);
    require(clearWeatherFog.density == 0.0F && clearWeatherFog.layerHeight == -1.0F &&
                clearWeatherFog.color == std::array<float, 3>{0.45F, 0.65F, 0.88F},
            "Diligent weather-fog bridge did not preserve the clear-weather defaults");
    Matrix4 rotatedFogView = identity;
    rotatedFogView[8] = 1.0;
    rotatedFogView[10] = 0.0;
    const auto layeredWeatherFog =
        buildDiligentWeatherFog(rotatedFogView, 25.0, 0.5, 100.0, 10.0);
    require(std::abs(layeredWeatherFog.density - 0.03912023F) < 1.0e-6F &&
                layeredWeatherFog.layerHeight == 10.0F &&
                layeredWeatherFog.cameraHeight == 25.0F &&
                layeredWeatherFog.worldUpView == std::array<float, 3>{1.0F, 0.0F, 0.0F} &&
                layeredWeatherFog.color == std::array<float, 3>{0.225F, 0.325F, 0.44F},
            "Diligent weather-fog bridge did not match production layer and view parameters");
    const auto sceneEnvironment = buildDiligentSceneEnvironment(
        identity, std::array<double, 3>{0.0, 0.0, 1.0}, 0.5, false, 25.0,
        100.0, 10.0);
    require(sceneEnvironment.lighting.lightDirectionView ==
                std::array<float, 3>{0.0F, 0.0F, 1.0F} &&
                sceneEnvironment.fog.worldUpView ==
                    std::array<float, 3>{0.0F, 0.0F, 1.0F} &&
                std::abs(sceneEnvironment.lighting.weatherFog.density -
                         sceneEnvironment.fog.density) < 1.0e-6F &&
                sceneEnvironment.clearColor ==
                    std::array<float, 4>{0.225F, 0.325F, 0.44F, 1.0F},
            "Diligent scene environment did not keep per-view lighting, fog, and background aligned");
    SystemVariables productionWeather;
    openbus::WeatherState productionWeatherState;
    productionWeatherState.brightness = 0.5;
    productionWeatherState.visibilityMeters = 100.0;
    productionWeatherState.fogTopAboveGroundMeters = 10.0;
    productionWeather.setWeather(productionWeatherState);
    productionWeather.configureDayNight(35.0);
    productionWeather.setSunAltitude(15.0);
    const auto productionCamera = openbus::app::makeDiligentCameraView(
        identity, identity, {0.0, 0.0, 30.0}, 320, 240);
    const auto productionSceneEnvironment = openbus::app::buildDiligentSceneEnvironment(
        productionCamera, productionWeather, 25.0, false);
    const auto productionSunDirection = productionWeather.sunDirection();
    require(productionSunDirection.has_value() &&
                productionSceneEnvironment.lighting.lightDirectionView ==
                    std::array<float, 3>{
                        static_cast<float>((*productionSunDirection)[0]),
                        static_cast<float>((*productionSunDirection)[1]),
                        static_cast<float>((*productionSunDirection)[2])},
            "Diligent production environment ignored the configured solar direction");
    require(productionSceneEnvironment.clearColor == sceneEnvironment.clearColor &&
                productionSceneEnvironment.fog.density == sceneEnvironment.fog.density &&
                productionSceneEnvironment.fog.layerHeight == sceneEnvironment.fog.layerHeight &&
                productionSceneEnvironment.lighting.weatherFog.density ==
                    productionSceneEnvironment.fog.density,
            "Diligent app weather bridge diverged from production weather values");
    const auto reflectionSceneEnvironment = openbus::app::buildDiligentSceneEnvironment(
        productionCamera, productionWeather, 25.0, true);
    require(reflectionSceneEnvironment.lighting.ambientIntensity >= 0.25F &&
                reflectionSceneEnvironment.clearColor == productionSceneEnvironment.clearColor,
            "Diligent reflection environment did not preserve the ambient floor and weather sky");
    bool invalidProductionCameraEnvironmentRejected = false;
    try {
        auto invalidCamera = productionCamera;
        invalidCamera.viewportHeight = 0;
        (void)openbus::app::buildDiligentSceneEnvironment(
            invalidCamera, productionWeather, 25.0, false);
    } catch (const std::invalid_argument&) {
        invalidProductionCameraEnvironmentRejected = true;
    }
    require(invalidProductionCameraEnvironmentRejected,
            "Diligent app weather bridge accepted an invalid camera viewport");
    bool invalidDiligentVisibilityRejected = false;
    try {
        (void)buildDiligentWeatherFog(identity, 0.0, 1.0, -1.0, std::nullopt);
    } catch (const std::invalid_argument&) {
        invalidDiligentVisibilityRejected = true;
    }
    require(invalidDiligentVisibilityRejected,
            "Diligent weather-fog bridge accepted negative visibility");
    const std::array<MapPointLight, 2> mapLights = {
        MapPointLight{{10.0, 2.0, 3.0}, {0.5, 0.75, 1.0}, 12.0},
        MapPointLight{{20.0, 2.0, 3.0}, {1.0, 0.5, 0.25}, 8.0}};
    const auto diligentMapLights =
        buildDiligentMapPointLights(mapLights, translatedView, {10.0, 0.0, 0.0}, 0.25, 1);
    require(diligentMapLights.size() == 1 && diligentMapLights[0].positionView ==
                std::array<float, 3>{0.0F, 2.0F, 3.0F} &&
                diligentMapLights[0].color == std::array<float, 3>{0.5F, 0.75F, 1.0F} &&
                diligentMapLights[0].radius == 12.0F && diligentMapLights[0].intensity == 0.25F,
            "Diligent map-light bridge did not select and transform the nearest light");
    bool invalidDiligentMapLightRejected = false;
    try {
        const std::array<MapPointLight, 1> invalidLights = {
            MapPointLight{{}, {1.0, 1.0, 1.0}, 0.0}};
        (void)buildDiligentMapPointLights(invalidLights, translatedView, {}, 1.0, 1);
    } catch (const std::invalid_argument&) {
        invalidDiligentMapLightRejected = true;
    }
    require(invalidDiligentMapLightRejected,
            "Diligent map-light bridge accepted a non-positive radius");
    const std::array<DiligentWorldSpotLight, 2> worldSpotLights = {{
        {{10.0, 2.0, 3.0}, {0.0, 0.0, -1.0}, {0.5, 0.75, 1.0},
         20.0, 1.5, 10.0, 25.0},
        {{30.0, 2.0, 3.0}, {1.0, 0.0, 0.0}, {1.0, 0.5, 0.25},
         15.0, 1.0, 5.0, 20.0},
    }};
    const auto diligentSpotLights =
        buildDiligentSpotLights(worldSpotLights, translatedView, {10.0, 0.0, 0.0}, 1);
    require(diligentSpotLights.size() == 1 &&
                diligentSpotLights[0].positionView == std::array<float, 3>{0.0F, 2.0F, 3.0F} &&
                diligentSpotLights[0].directionView == std::array<float, 3>{0.0F, 0.0F, -1.0F} &&
                diligentSpotLights[0].color == std::array<float, 3>{0.5F, 0.75F, 1.0F} &&
                diligentSpotLights[0].range == 20.0F &&
                diligentSpotLights[0].intensity == 1.5F &&
                diligentSpotLights[0].innerAngleDegrees == 10.0F &&
                diligentSpotLights[0].outerAngleDegrees == 25.0F,
            "Diligent spotlight bridge did not select and transform the nearest light");
    Matrix4 rotatedView = identity;
    rotatedView[0] = 0.0;
    rotatedView[1] = 1.0;
    rotatedView[4] = -1.0;
    rotatedView[5] = 0.0;
    const std::array<DiligentWorldSpotLight, 1> directionalSpot = {{
        {{}, {1.0, 0.0, 0.0}, {1.0, 1.0, 1.0}, 5.0, 1.0, 10.0, 20.0},
    }};
    const auto rotatedSpot =
        buildDiligentSpotLights(directionalSpot, rotatedView, {}, 1);
    require(rotatedSpot[0].directionView == std::array<float, 3>{0.0F, 1.0F, 0.0F},
            "Diligent spotlight bridge did not rotate beam direction into view space");
    bool invalidDiligentSpotLightRejected = false;
    try {
        const std::array<DiligentWorldSpotLight, 1> invalidLights = {{
            {{}, {0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}, 1.0, 1.0, 10.0, 20.0},
        }};
        (void)buildDiligentSpotLights(invalidLights, translatedView, {}, 1);
    } catch (const std::invalid_argument&) {
        invalidDiligentSpotLightRejected = true;
    }
    require(invalidDiligentSpotLightRejected,
            "Diligent spotlight bridge accepted a zero direction");
    const std::array<VehicleSpotlight, 2> viewSpaceVehicleLights = {{
        {{1.0, 2.0, 3.0}, {0.0, 0.0, -2.0}, {0.25, 0.5, 0.75}, 18.0, 8.0, 22.0},
        {{4.0, 5.0, 6.0}, {1.0, 0.0, 0.0}, {1.0, 1.0, 1.0}, 12.0, 5.0, 18.0},
    }};
    const auto diligentVehicleLights =
        openbus::app::vehicle::makeDiligentVehicleSpotLights(viewSpaceVehicleLights, 1);
    require(diligentVehicleLights.size() == 1 &&
                diligentVehicleLights[0].positionView == std::array<float, 3>{1.0F, 2.0F, 3.0F} &&
                diligentVehicleLights[0].directionView == std::array<float, 3>{0.0F, 0.0F, -2.0F} &&
                diligentVehicleLights[0].color == std::array<float, 3>{0.25F, 0.5F, 0.75F} &&
                diligentVehicleLights[0].range == 18.0F &&
                diligentVehicleLights[0].innerAngleDegrees == 8.0F &&
                diligentVehicleLights[0].outerAngleDegrees == 22.0F,
            "Diligent vehicle-light bridge did not preserve the selected view-space light");
    bool invalidDiligentVehicleLightRejected = false;
    try {
        const std::array<VehicleSpotlight, 1> invalidLights = {{
            {{}, {0.0, 0.0, 1.0}, {1.1, 1.0, 1.0}, 1.0, 5.0, 10.0},
        }};
        (void)openbus::app::vehicle::makeDiligentVehicleSpotLights(invalidLights, 1);
    } catch (const std::invalid_argument&) {
        invalidDiligentVehicleLightRejected = true;
    }
    require(invalidDiligentVehicleLightRejected,
            "Diligent vehicle-light bridge accepted a color outside the production range");
    bool outOfRangeDiligentMapLightRejected = false;
    try {
        const std::array<MapPointLight, 1> outOfRangeLights = {
            MapPointLight{{}, {1.0e100, 1.0, 1.0}, 1.0}};
        (void)buildDiligentMapPointLights(outOfRangeLights, translatedView, {}, 1.0, 1);
    } catch (const std::invalid_argument&) {
        outOfRangeDiligentMapLightRejected = true;
    }
    require(outOfRangeDiligentMapLightRejected,
            "Diligent map-light bridge returned values outside GPU float range");

    std::array<ModelAnimation, 1> rootAnimations = {};
    rootAnimations[0].type = "anim_trans";
    std::array<ModelAnimation, 1> childAnimations = {};
    childAnimations[0].type = "anim_trans";
    const std::array<double, 1> rootAmounts = {2.0};
    std::array<double, 1> childAmounts = {3.0};
    std::array<DiligentVehicleAnimationGroupInput, 2> animationGroups = {{
        {0, std::nullopt, rootAnimations, rootAmounts, true},
        {1, 0, childAnimations, childAmounts, true},
    }};
    MotionHistory vehicleMotionHistory;
    vehicleMotionHistory.beginFrame(1, 1.0, 320, 240, identity, identity, 320, 240);
    auto groupTransforms = buildDiligentVehicleGroupTransforms(
        animationGroups, vehicleMotionHistory.frame(), vehicleMotionHistory, 42, identity);
    require(groupTransforms.size() == 2 && groupTransforms[0].modelView[13] == -2.0 &&
                groupTransforms[1].modelView[13] == -5.0 &&
                groupTransforms[0].motion.has_value() &&
                !groupTransforms[0].motion->historyValid &&
                !groupTransforms[1].motion->historyValid,
            "Diligent vehicle group hierarchy produced incorrect first-frame transforms");
    vehicleMotionHistory.completeFrame();
    childAmounts[0] = 4.0;
    animationGroups[1].geometryHistoryValid = false;
    vehicleMotionHistory.beginFrame(2, 1.008, 320, 240, identity, identity, 320, 240);
    groupTransforms = buildDiligentVehicleGroupTransforms(
        animationGroups, vehicleMotionHistory.frame(), vehicleMotionHistory, 42, identity);
    require(groupTransforms[0].motion->historyValid &&
                !groupTransforms[1].motion->historyValid &&
                groupTransforms[1].modelView[13] == -6.0,
            "Diligent vehicle group history did not preserve per-group geometry validity");
    vehicleMotionHistory.completeFrame();
    animationGroups[0].parentAnimationGroupIndex = 1;
    vehicleMotionHistory.beginFrame(3, 1.016, 320, 240, identity, identity, 320, 240);
    bool animationCycleRejected = false;
    try {
        (void)buildDiligentVehicleGroupTransforms(
            animationGroups, vehicleMotionHistory.frame(), vehicleMotionHistory, 42, identity);
    } catch (const std::invalid_argument&) {
        animationCycleRejected = true;
    }
    require(animationCycleRejected, "Diligent vehicle animation group cycle was not rejected");
    vehicleMotionHistory.invalidate();

    DiligentHumanPoseHistory visibilityHistory;
    HumanBonePalette visibilityPose = {};
    for (Matrix4& bone : visibilityPose) {
        bone = identity;
    }
    MotionDataFrame poseFrame;
    poseFrame.frameId = 1;
    poseFrame.width = 320;
    poseFrame.height = 240;
    visibilityHistory.beginFrame(poseFrame);
    require(!visibilityHistory.recordRenderedPose({88, 0}, visibilityPose).historyValid,
            "A newly visible Diligent pedestrian received valid pose history");
    visibilityHistory.completeFrame();
    poseFrame.frameId = 2;
    poseFrame.historyValid = true;
    visibilityHistory.beginFrame(poseFrame);
    require(visibilityHistory.recordRenderedPose({88, 0}, visibilityPose).historyValid &&
                !visibilityHistory.recordRenderedPose({88, 1}, visibilityPose).historyValid,
            "Diligent pedestrian pose history was not scoped to its rendering view");
    visibilityHistory.completeFrame();
    poseFrame.frameId = 3;
    visibilityHistory.beginFrame(poseFrame);
    require(visibilityHistory.recordRenderedPose({88, 1}, visibilityPose).historyValid,
            "Diligent pedestrian lost pose history in a continuously rendered view");
    visibilityHistory.completeFrame();
    poseFrame.frameId = 4;
    visibilityHistory.beginFrame(poseFrame);
    require(!visibilityHistory.recordRenderedPose({88, 0}, visibilityPose).historyValid,
            "Diligent pedestrian pose history survived a frame in which it was not rendered");
    visibilityHistory.completeFrame();

    const auto windowConfiguration =
        openbus::app::makeDiligentWindowConfiguration(window, 320, 240);
    require(windowConfiguration.nativeWindow.hWnd != nullptr &&
                windowConfiguration.swapChainDesc.Width == 320 &&
                windowConfiguration.swapChainDesc.Height == 240 &&
                windowConfiguration.swapChainDesc.ColorBufferFormat == TEX_FORMAT_RGBA8_UNORM,
            "Diligent window bridge did not preserve the drawable handle and dimensions");
    bool zeroSizedDiligentSwapChainRejected = false;
    try {
        (void)openbus::app::makeDiligentWindowConfiguration(window, 0, 240);
    } catch (const std::invalid_argument&) {
        zeroSizedDiligentSwapChainRejected = true;
    }
    require(zeroSizedDiligentSwapChainRejected,
            "Diligent window bridge accepted a zero-sized initial swap chain");
    openbus::core::GraphicsPreferences framePreferences;
    framePreferences.antiAliasingMode = "msaa";
    framePreferences.anisotropicFiltering = 16;
    framePreferences.vsync = false;
    auto rendererOwner =
        openbus::app::createDiligentFrameRenderer(window, framePreferences, true);
    auto& renderer = *rendererOwner;
    require(renderer.backend() == openbus::rendering::RendererBackend::Diligent,
            "Diligent frame host reported the wrong presenter backend");
    const auto selectedMultisampling = openbus::rendering::selectDiligentMultisampling(
        renderer.device(), true, msaaDiligentOptions.multisampling);
    require(renderer.targets().samples() == selectedMultisampling.multisampling.samples &&
                renderer.defaultColorFilter() ==
                    (selectedMultisampling.useFxaaFallback ? DiligentColorFilter::Fxaa
                                                          : DiligentColorFilter::None),
            "Diligent frame host did not apply supported MSAA or the FXAA fallback");
    renderer.setDefaultColorFilter(DiligentColorFilter::Fxaa, false);
    require(renderer.defaultColorFilter() == DiligentColorFilter::Fxaa,
            "Diligent frame host did not apply a runtime FXAA preference");
    renderer.setDefaultColorFilter(DiligentColorFilter::Smaa, false);
    require(renderer.defaultColorFilter() == DiligentColorFilter::Smaa,
            "Diligent frame host did not apply a runtime SMAA preference");
    renderer.setDefaultColorFilter(DiligentColorFilter::None, true);
    require(renderer.defaultColorFilter() ==
                (renderer.targets().samples() == 1 ? DiligentColorFilter::Fxaa
                                                   : DiligentColorFilter::None),
            "Diligent frame host did not preserve the runtime MSAA fallback");
    renderer.setDefaultColorFilter(DiligentColorFilter::None, false);
    require(renderer.defaultColorFilter() == DiligentColorFilter::None,
            "Diligent frame host did not apply a runtime unfiltered preference");
    renderer.setDefaultColorFilter(DiligentColorFilter::None, true);
    renderer.configureSpotlightShadowMaps(32, 2);
    int expectedDrawableWidth = 0;
    int expectedDrawableHeight = 0;
    glfwGetFramebufferSize(&window, &expectedDrawableWidth, &expectedDrawableHeight);
    require(expectedDrawableWidth > 0 && expectedDrawableHeight > 0 &&
                renderer.targets().width() == static_cast<Uint32>(expectedDrawableWidth) &&
                renderer.targets().height() == static_cast<Uint32>(expectedDrawableHeight),
            "Diligent frame-renderer factory did not use the GLFW drawable size");
    const std::array<Uint32, 2> reflectionSizes = {64, 96};
    renderer.configureReflectionTargets(reflectionSizes, 2);
    require(renderer.reflectionTarget(0).size() == 64 &&
                renderer.reflectionTarget(1).size() == 96,
            "Diligent frame host did not configure indexed reflection targets");
    runDiligentVehicleSourceBridgeProbe(
        renderer.reflectionTarget(0).colorTexture());
    DiligentReflectionTarget reflectionTarget(renderer.device(), 64);
    require(reflectionTarget.size() == 64 &&
                reflectionTarget.colorFormat() == TEX_FORMAT_RGBA8_UNORM &&
                reflectionTarget.depthFormat() == TEX_FORMAT_D32_FLOAT,
            "Diligent reflection target did not preserve its configured dimensions or formats");
    reflectionTarget.clear(renderer.immediateContext());
    require(reflectionTarget.colorTexture().GetTexture()->GetDesc().Width == 64 &&
                reflectionTarget.colorTexture().GetTexture()->GetDesc().Height == 64 &&
                reflectionTarget.depthTexture().GetTexture()->GetDesc().Width == 64 &&
                reflectionTarget.depthTexture().GetTexture()->GetDesc().Height == 64,
            "Diligent reflection target did not expose correctly sized sampleable attachments");
    reflectionTarget.resize(96);
    reflectionTarget.clear(renderer.immediateContext());
    require(reflectionTarget.size() == 96 &&
                reflectionTarget.colorTexture().GetTexture()->GetDesc().Width == 96 &&
                reflectionTarget.depthTexture().GetTexture()->GetDesc().Height == 96,
            "Diligent reflection target did not resize its attachments");
    bool zeroSizedReflectionTargetRejected = false;
    try {
        reflectionTarget.resize(0);
    } catch (const std::invalid_argument&) {
        zeroSizedReflectionTargetRejected = true;
    }
    require(zeroSizedReflectionTargetRejected && reflectionTarget.size() == 96,
            "Diligent reflection target accepted an invalid resize or lost its current size");

    std::array<DiligentSceneryBatch, 2> orderedSceneryBatches = {};
    orderedSceneryBatches[0].localCenter = {0.0, 0.0, -2.0};
    orderedSceneryBatches[1].localCenter = {0.0, 0.0, -4.0};
    std::array<DiligentSceneryTreeBatch, 2> orderedSceneryTrees = {};
    orderedSceneryTrees[0].localCenter = {0.0, 0.0, -3.0};
    orderedSceneryTrees[1].localCenter = {0.0, 0.0, -1.0};
    const std::array<std::size_t, 2> orderedBatchIndices = {0, 1};
    const std::array<std::size_t, 2> orderedTreeIndices = {0, 1};
    const auto orderedTransparentScenery =
        orderDiligentTransparentScenery(orderedSceneryBatches, orderedBatchIndices,
                                        orderedSceneryTrees, orderedTreeIndices, identity);
    require(orderedTransparentScenery.size() == 4 &&
                orderedTransparentScenery[0].kind == DiligentScenerySubmissionKind::Batch &&
                orderedTransparentScenery[0].index == 1 &&
                orderedTransparentScenery[1].kind == DiligentScenerySubmissionKind::Tree &&
                orderedTransparentScenery[1].index == 0 &&
                orderedTransparentScenery[2].kind == DiligentScenerySubmissionKind::Batch &&
                orderedTransparentScenery[2].index == 0 &&
                orderedTransparentScenery[3].kind == DiligentScenerySubmissionKind::Tree &&
                orderedTransparentScenery[3].index == 1,
            "Diligent transparent scenery did not interleave trees and batches back-to-front");
    const DiligentSceneryBatch sceneryBatch;
    const DiligentSceneryTreeBatch sceneryTreeBatch;
    const DiligentSceneryOpaqueDraw sceneryDraw;
    bool sceneryBatchDrawRejectedOutsideScene = false;
    try {
        renderer.drawSceneryBatch(sceneryBatch, DiligentSceneryPass::Opaque,
                                  renderer.targets().colorTexture(), {}, sceneryDraw);
    } catch (const std::logic_error&) {
        sceneryBatchDrawRejectedOutsideScene = true;
    }
    require(sceneryBatchDrawRejectedOutsideScene,
            "Diligent frame host allowed scenery batch submission outside a scene");
    bool sceneryTreeDrawRejectedOutsideScene = false;
    try {
        renderer.drawSceneryTreeBatch(sceneryTreeBatch, renderer.targets().colorTexture(), {},
                                      sceneryDraw);
    } catch (const std::logic_error&) {
        sceneryTreeDrawRejectedOutsideScene = true;
    }
    require(sceneryTreeDrawRejectedOutsideScene,
            "Diligent frame host allowed scenery tree submission outside a scene");

    auto vehicleTextureImage = std::make_shared<Image>();
    vehicleTextureImage->width = 1;
    vehicleTextureImage->height = 1;
    vehicleTextureImage->rgba = {255, 255, 255, 255};
    const std::shared_ptr<const Image> immutableVehicleTexture = vehicleTextureImage;
    const std::array<ModelVertex, 3> vehicleVertices = {{
        {-0.5F, -0.5F, 0.5F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {0.5F, -0.5F, 0.5F, 1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {0.0F, 0.5F, 0.5F, 0.5F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F},
    }};
    DiligentVehicleBatchSource vehicleBatch;
    vehicleBatch.vertices = vehicleVertices;
    vehicleBatch.texture = immutableVehicleTexture;
    const std::array<DiligentVehicleBatchSource, 1> vehicleBatches = {vehicleBatch};
    DiligentVehicleBatchSource transparentVehicleBatch = vehicleBatch;
    transparentVehicleBatch.materialIndex = 1;
    transparentVehicleBatch.sourceBatchIndex = 1;
    transparentVehicleBatch.material.alphaMode = DiligentAlphaMode::Blend;
    const std::array<DiligentVehicleBatchSource, 2> mixedVehicleBatches = {
        vehicleBatch, transparentVehicleBatch};
    DiligentVehicleAsset passVehicleAsset(renderer.device(), mixedVehicleBatches);
    constexpr std::uint64_t vehicleObjectId = 7001;
    renderer.prepareVehicleAsset(vehicleObjectId, 1, 1, vehicleBatches);
    constexpr std::uint64_t farSceneVehicleId = 7002;
    constexpr std::uint64_t nearSceneVehicleId = 7003;
    renderer.prepareVehicleAsset(farSceneVehicleId, 1, 1, mixedVehicleBatches);
    renderer.prepareVehicleAsset(nearSceneVehicleId, 1, 1, mixedVehicleBatches);
    const DiligentVehicleAsset* cachedVehicleAsset = renderer.vehicleAsset(vehicleObjectId);
    require(cachedVehicleAsset != nullptr && renderer.vehicleAssetCurrent(vehicleObjectId, 1),
            "Diligent frame host did not publish the prepared vehicle asset");
    renderer.prepareVehicleAsset(vehicleObjectId, 1, 2, {});
    require(renderer.vehicleAsset(vehicleObjectId) == cachedVehicleAsset,
            "Diligent frame host rebuilt an unchanged vehicle asset");
    renderer.prepareVehicleAsset(vehicleObjectId, 2, 2, vehicleBatches);
    require(renderer.vehicleAssetCurrent(vehicleObjectId, 2) &&
                renderer.vehicleAsset(vehicleObjectId) != nullptr,
            "Diligent frame host did not rebuild a changed vehicle asset generation");
    bool missingVehicleSourcesRejected = false;
    try {
        renderer.prepareVehicleAsset(vehicleObjectId, 3, 2, {});
    } catch (const std::invalid_argument&) {
        missingVehicleSourcesRejected = true;
    }
    require(missingVehicleSourcesRejected && renderer.vehicleAssetCurrent(vehicleObjectId, 2),
            "Diligent frame host discarded a valid vehicle asset after incomplete preparation");
    const DiligentVehicleAsset& vehicleAsset = *renderer.vehicleAsset(vehicleObjectId);
    DiligentTexture2D vehicleFallback(renderer.device(), *immutableVehicleTexture);
    openbus::map::MapTerrainMesh reflectionTerrain;
    reflectionTerrain.vertices.assign(vehicleVertices.begin(), vehicleVertices.end());
    reflectionTerrain.indices = {0, 1, 2};
    DiligentMapTerrainLayerSource reflectionTerrainLayer;
    reflectionTerrainLayer.texture = immutableVehicleTexture;
    const std::array<DiligentMapTerrainLayerSource, 1> reflectionTerrainLayers = {
        reflectionTerrainLayer};
    DiligentMapTileSet reflectionMapTiles(renderer.device(), 300.0);
    reflectionMapTiles.addTile(0, 0, &reflectionTerrain, reflectionTerrainLayers, nullptr, {}, {});
    reflectionMapTiles.updateActiveTiles(150.0, 150.0, 0);
    require(reflectionMapTiles.activeTiles().size() == 1,
            "Diligent reflection map-tile fixture did not activate its terrain tile");
    DiligentMapTerrainLayerSource opaqueShadowTerrain = reflectionTerrainLayer;
    DiligentMapTerrainLayerSource cutoutShadowTerrain = reflectionTerrainLayer;
    cutoutShadowTerrain.material.alphaMode = DiligentAlphaMode::Cutout;
    DiligentMapTerrainLayerSource transparentShadowTerrain = reflectionTerrainLayer;
    transparentShadowTerrain.material.alphaMode = DiligentAlphaMode::Blend;
    const std::array<DiligentMapTerrainLayerSource, 3> shadowTerrainLayers = {
        opaqueShadowTerrain, cutoutShadowTerrain, transparentShadowTerrain};
    DiligentMapTileSet shadowMapTiles(renderer.device(), 300.0);
    shadowMapTiles.addTile(2, 1, &reflectionTerrain, shadowTerrainLayers, nullptr, {}, {});
    shadowMapTiles.updateActiveTiles(750.0, 450.0, 0);
    const auto mapShadowCasters = openbus::app::buildDiligentMapTileShadowCasters(
        shadowMapTiles, 300.0, identity, identity);
    require(shadowMapTiles.activeTiles().size() == 1 && mapShadowCasters.size() == 2 &&
                mapShadowCasters[0].modelViewProjection[12] == 600.0 &&
                mapShadowCasters[0].modelViewProjection[13] == 300.0 &&
                mapShadowCasters[1].alphaTexture != nullptr &&
                mapShadowCasters[1].alphaCutoff == 0.5F,
            "Diligent map shadow caster collection lost tile transforms or alpha filtering");
    openbus::app::vehicle::DiligentVehicleViewGroupInput reflectionViewGroup;
    reflectionViewGroup.localAnimationTransform = identity;
    reflectionViewGroup.localCenter = {0.0, -0.1667, 0.5};
    reflectionViewGroup.localRadius = 1.0;
    const std::array<openbus::app::vehicle::DiligentVehicleViewGroupInput, 1>
        reflectionViewGroups = {
        reflectionViewGroup};
    openbus::rendering::SharedMapTextureImages::SeasonalPaths seasonalPaths = {
        "summer.dds", "spring.dds", "fall.dds", "winter.dds", "winter_snow.dds"};
    std::vector<std::filesystem::path> seasonalLookupAttempts;
    const auto lookupWinterSnow = [&seasonalLookupAttempts, &vehicleFallback,
                                   &seasonalPaths](const std::filesystem::path& path)
        -> ITextureView* {
        seasonalLookupAttempts.push_back(path);
        return path == seasonalPaths[4] ? &vehicleFallback.shaderResourceView() : nullptr;
    };
    ITextureView* resolvedSeasonalTexture =
        openbus::app::resolveDiligentMapSeasonalTexture(4, seasonalPaths, lookupWinterSnow);
    require(resolvedSeasonalTexture == &vehicleFallback.shaderResourceView() &&
                seasonalLookupAttempts == std::vector<std::filesystem::path>{seasonalPaths[4]},
            "Diligent map scenery did not prefer an available winter-snow texture");
    seasonalLookupAttempts.clear();
    const auto lookupWinterFallback = [&seasonalLookupAttempts, &vehicleFallback,
                                       &seasonalPaths](const std::filesystem::path& path)
        -> ITextureView* {
        seasonalLookupAttempts.push_back(path);
        return path == seasonalPaths[3] ? &vehicleFallback.shaderResourceView() : nullptr;
    };
    resolvedSeasonalTexture =
        openbus::app::resolveDiligentMapSeasonalTexture(4, seasonalPaths, lookupWinterFallback);
    require(resolvedSeasonalTexture == &vehicleFallback.shaderResourceView() &&
                seasonalLookupAttempts ==
                    std::vector<std::filesystem::path>{seasonalPaths[4], seasonalPaths[3]},
            "Diligent map scenery did not fall back from winter-snow to winter");
    bool invalidScenerySeasonRejected = false;
    try {
        (void)openbus::app::resolveDiligentMapSeasonalTexture(
            seasonalPaths.size(), seasonalPaths, lookupWinterFallback);
    } catch (const std::invalid_argument&) {
        invalidScenerySeasonRejected = true;
    }
    require(invalidScenerySeasonRejected,
            "Diligent map scenery accepted an out-of-range placement season");
    PreparedSceneryAsset preparedScenery;
    PreparedSceneryMesh preparedSceneryMesh;
    preparedSceneryMesh.verticesByMaterial.emplace(
        0, std::vector<ModelVertex>(vehicleVertices.begin(), vehicleVertices.end()));
    preparedScenery.meshes.emplace("synthetic", std::move(preparedSceneryMesh));
    PreparedSceneryBatch opaqueScenerySource;
    opaqueScenerySource.meshKey = "synthetic";
    opaqueScenerySource.batch.vertexCount = vehicleVertices.size();
    PreparedSceneryBatch transparentScenerySource = opaqueScenerySource;
    transparentScenerySource.batch.alphaMode = 2;
    preparedScenery.batches = {opaqueScenerySource, transparentScenerySource};
    DiligentSceneryAsset sceneryAsset(renderer.device(), preparedScenery);
    PreparedSceneryAsset cutoutScenery = preparedScenery;
    cutoutScenery.batches = {opaqueScenerySource};
    cutoutScenery.batches.front().batch.alphaMode = 1;
    DiligentSceneryAsset cutoutSceneryAsset(renderer.device(), cutoutScenery);
    PreparedSceneryAsset variableScenery = preparedScenery;
    for (PreparedSceneryBatch& batch : variableScenery.batches) {
        batch.batch.visibilityVariable = "placement_visible";
        batch.batch.visibilityValue = 1;
    }
    DiligentSceneryAsset variableSceneryAsset(renderer.device(), variableScenery);
    PreparedSceneryAsset scriptedScenery = preparedScenery;
    scriptedScenery.batches.front().batch.scriptTextureIndex = 0;
    scriptedScenery.batches.back().batch.textTextureIndex = 0;
    DiligentSceneryAsset scriptedSceneryAsset(renderer.device(), scriptedScenery);
    auto sceneryShadowImage = std::make_shared<Image>();
    sceneryShadowImage->width = 1;
    sceneryShadowImage->height = 1;
    sceneryShadowImage->rgba = {255, 255, 255, 255};
    PreparedSceneryAsset preparedShadowScenery = preparedScenery;
    preparedShadowScenery.decodedTextures.emplace("synthetic-shadow.png",
                                                   sceneryShadowImage);
    PreparedSceneryBatch cutoutShadowSource = opaqueScenerySource;
    cutoutShadowSource.texturePath = "synthetic-shadow.png";
    cutoutShadowSource.batch.alphaMode = 1;
    cutoutShadowSource.batch.material.flipTextureY = true;
    cutoutShadowSource.batch.texcoordTransYVariable = "scenery_uv_y";
    cutoutShadowSource.batch.alphaScaleVariable = "scenery_alpha";
    PreparedSceneryBatch hiddenShadowSource = opaqueScenerySource;
    hiddenShadowSource.batch.visibilityVariable = "placement_visible";
    hiddenShadowSource.batch.visibilityValue = 0;
    PreparedSceneryBatch textShadowSource = opaqueScenerySource;
    textShadowSource.batch.textTextureIndex = 0;
    preparedShadowScenery.batches = {opaqueScenerySource, cutoutShadowSource,
                                     transparentScenerySource, hiddenShadowSource,
                                     textShadowSource};
    PreparedSceneryTreeBatch cutoutShadowTree;
    cutoutShadowTree.vertices.assign(vehicleVertices.begin(), vehicleVertices.end());
    cutoutShadowTree.batch.vertexCount = vehicleVertices.size();
    cutoutShadowTree.batch.alphaMode = 1;
    cutoutShadowTree.texturePath = "synthetic-shadow.png";
    preparedShadowScenery.treeBatches = {cutoutShadowTree};
    DiligentSceneryAsset shadowSceneryAsset(renderer.device(), preparedShadowScenery);
    const std::array<DiligentSceneryOpaqueDraw, 2> sceneryDrawStates = {};
    const std::array<DiligentSceneryOpaqueDraw, 1> cutoutSceneryDrawStates = {};
    const std::array<DiligentSceneryOpaqueDraw, 0> noTreeDrawStates = {};
    Variables placementVariables;
    placementVariables.set("placement_visible", 1.0F);
    placementVariables.set("scenery_uv_y", 0.25F);
    placementVariables.set("scenery_alpha", 0.8F);
    openbus::app::DiligentMapSceneryPlacement mapSceneryPlacement;
    mapSceneryPlacement.motionObjectId = 905;
    mapSceneryPlacement.worldTransform = identity;
    mapSceneryPlacement.boundingRadius = 0.1;
    mapSceneryPlacement.variables = &placementVariables;
    auto shadowSceneryPlacement = mapSceneryPlacement;
    shadowSceneryPlacement.worldTransform[12] = 3.0;
    const auto sceneryShadowCasters =
        openbus::app::buildDiligentMapSceneryShadowCasters(
            shadowSceneryPlacement, shadowSceneryAsset, translatedView, identity);
    require(sceneryShadowCasters.size() == 3 &&
                sceneryShadowCasters[0].alphaCutoff == 0.0F &&
                sceneryShadowCasters[1].alphaTexture != nullptr &&
                std::abs(sceneryShadowCasters[1].alphaCutoff - 0.625F) < 1.0e-6F &&
                sceneryShadowCasters[1].textureTransform ==
                    std::array<float, 4>{1.0F, -1.0F, 0.0F, 1.25F} &&
                sceneryShadowCasters[2].alphaTexture != nullptr &&
                sceneryShadowCasters[2].alphaCutoff == 0.5F &&
                sceneryShadowCasters[0].modelViewProjection[12] == -7.0,
            "Diligent scenery shadows lost transforms, cutout textures, or visibility filtering");
    bool sceneryShadowTextureMismatchRejected = false;
    const std::array<ITextureView*, 1> mismatchedSceneryShadowTextures = {nullptr};
    try {
        (void)openbus::app::buildDiligentMapSceneryShadowCasters(
            shadowSceneryPlacement, shadowSceneryAsset, identity, identity,
            mismatchedSceneryShadowTextures);
    } catch (const std::invalid_argument&) {
        sceneryShadowTextureMismatchRejected = true;
    }
    require(sceneryShadowTextureMismatchRejected,
            "Diligent scenery shadow bridge accepted mismatched seasonal texture state");
    HumanMeshGeometry humanGeometry;
    humanGeometry.verticesByMaterial.emplace(0, std::vector<ModelVertex>(
        vehicleVertices.begin(), vehicleVertices.end()));
    humanGeometry.verticesByMaterial.emplace(1, std::vector<ModelVertex>(
        vehicleVertices.begin(), vehicleVertices.end()));
    constexpr std::string_view humanModelKey = "synthetic/human";
    const std::unordered_map<int, std::shared_ptr<const Image>> humanTextures = {
        {0, immutableVehicleTexture}, {1, immutableVehicleTexture}};
    DiligentMaterialState transparentHumanDrawMaterial;
    transparentHumanDrawMaterial.alphaMode = DiligentAlphaMode::Blend;
    const std::unordered_map<int, DiligentMaterialState> humanDrawMaterials = {
        {1, transparentHumanDrawMaterial}};
    renderer.prepareHumanAsset(humanModelKey, 1, humanGeometry, humanTextures,
                               humanDrawMaterials);
    const DiligentHumanAsset* initialHumanAsset = renderer.humanAsset(humanModelKey);
    require(initialHumanAsset != nullptr &&
                renderer.humanAssetCurrent(humanModelKey, 1),
            "Diligent frame host did not publish a prepared human asset");
    HumanMeshGeometry unusedHumanGeometry;
    renderer.prepareHumanAsset(humanModelKey, 1, unusedHumanGeometry, {});
    require(renderer.humanAsset(humanModelKey) == initialHumanAsset,
            "Diligent frame host rebuilt an unchanged human asset generation");
    renderer.prepareHumanAsset(humanModelKey, 2, humanGeometry, humanTextures,
                               humanDrawMaterials);
    const DiligentHumanAsset* humanAsset = renderer.humanAsset(humanModelKey);
    require(humanAsset != nullptr && humanAsset != initialHumanAsset &&
                !renderer.humanAssetCurrent(humanModelKey, 1) &&
                renderer.humanAssetCurrent(humanModelKey, 2),
            "Diligent frame host did not replace a changed human asset generation");
    HumanMeshGeometry transparentOnlyHumanGeometry;
    transparentOnlyHumanGeometry.verticesByMaterial.emplace(
        1, std::vector<ModelVertex>(vehicleVertices.begin(), vehicleVertices.end()));
    constexpr std::string_view transparentOnlyHumanModelKey = "synthetic/transparent-human";
    const std::unordered_map<int, std::shared_ptr<const Image>> transparentOnlyHumanTextures = {
        {1, immutableVehicleTexture}};
    renderer.prepareHumanAsset(transparentOnlyHumanModelKey, 1, transparentOnlyHumanGeometry,
                               transparentOnlyHumanTextures, humanDrawMaterials);
    const DiligentHumanAsset* transparentOnlyHumanAsset =
        renderer.humanAsset(transparentOnlyHumanModelKey);
    require(transparentOnlyHumanAsset != nullptr &&
                !transparentOnlyHumanAsset->hasSubmeshesForPass(
                    DiligentHumanRenderPass::Opaque) &&
                transparentOnlyHumanAsset->hasSubmeshesForPass(
                    DiligentHumanRenderPass::Transparent),
            "Diligent human asset pass availability did not match its material alpha mode");
    HumanTrafficRenderSource humanRenderSource;
    humanRenderSource.sourceGeneration = 2;
    HumanBonePalette vehicleTestPose = {};
    for (Matrix4& bone : vehicleTestPose) {
        bone = identity;
    }

    DiligentMapTileAsset motionTile(renderer.device(), 2, 1, nullptr,
                                    std::shared_ptr<const Image>{}, nullptr,
                                    DiligentMapWaterSource{},
                                    std::span<const DiligentMapRoadSource>{}, false);
    MotionHistory mapMotionHistory;
    mapMotionHistory.beginFrame(1, 1.0, 320, 240, identity, identity, 320, 240);
    require(!motionTile.motionParameters(mapMotionHistory.frame(), mapMotionHistory, 300.0)
                 .historyValid,
            "Diligent map tile motion was valid without a previous rendered frame");
    mapMotionHistory.completeFrame();
    Matrix4 movedView = identity;
    movedView[12] = -5.0;
    movedView[13] = -7.0;
    mapMotionHistory.beginFrame(2, 1.008, 320, 240, movedView, identity, 320, 240);
    const DiligentMotionParameters tileMotion =
        motionTile.motionParameters(mapMotionHistory.frame(), mapMotionHistory, 300.0);
    require(tileMotion.historyValid &&
                tileMotion.previousModelViewProjection[12] == 600.0 &&
                tileMotion.previousModelViewProjection[13] == 300.0,
            "Diligent map tile motion did not include its world origin and prior camera view");
    mapMotionHistory.completeFrame();

    openbus::map::MapDefinition map;
    map.coordinates.tileSize = 300.0;
    map.groundTextures.emplace_back();
    openbus::map::MapDefinition failingMap;
    failingMap.coordinates.tileSize = 300.0;
    failingMap.groundTextures.emplace_back();
    openbus::map::MapTileReference failingTile;
    failingTile.terrainPath = "missing-diligent-terrain-for-retry-probe.ter";
    failingTile.hasTerrainFile = true;
    failingMap.tiles.push_back(failingTile);
    openbus::rendering::DiligentMapTileSet failingMapTiles(
        renderer.device(), failingMap.coordinates, false);
    require(failingMapTiles.requestTerrainWindow(failingMap, 0.0, 0.0, 0, {}, 0) == 1,
            "Diligent map tile set did not schedule an invalid terrain source");
    const auto waitForTerrainPreparationFailure = [](
                                                      openbus::rendering::DiligentMapTileSet&
                                                          tileSet) {
        for (int attempt = 0; attempt < 100; ++attempt) {
            try {
                static_cast<void>(tileSet.publishReadyTerrainTiles());
            } catch (const std::exception&) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    };
    require(waitForTerrainPreparationFailure(failingMapTiles),
            "Diligent map tile preparation did not surface a failed terrain source");
    require(failingMapTiles.requestTerrainWindow(failingMap, 0.0, 0.0, 0, {}, 0) == 0,
            "Diligent map tile set retried a failed terrain source before its backoff expired");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    require(failingMapTiles.requestTerrainWindow(failingMap, 0.0, 0.0, 0, {}, 0) == 1,
            "Diligent map tile set did not retry a failed terrain source after its backoff");
    require(waitForTerrainPreparationFailure(failingMapTiles),
            "Diligent map tile retry did not surface its preparation failure");
    failingMap.tiles.front().terrainPath = "changed-missing-diligent-terrain-probe.ter";
    require(failingMapTiles.requestTerrainWindow(failingMap, 0.0, 0.0, 0, {}, 0) == 1,
            "Diligent map tile set kept a failed-source backoff after its source changed");
    openbus::rendering::DiligentMapTileSet inactiveMapTiles(
        renderer.device(), failingMap.coordinates, false);
    require(inactiveMapTiles.requestTerrainWindow(failingMap, 0.0, 0.0, 0, {}, 0) == 1 &&
                waitForTerrainPreparationFailure(inactiveMapTiles),
            "Diligent map tile set did not surface an inactive-window probe failure");
    require(inactiveMapTiles.requestTerrainWindow(map, 0.0, 0.0, 0, {}, 0) == 0 &&
                inactiveMapTiles.requestTerrainWindow(failingMap, 0.0, 0.0, 0, {}, 0) == 1,
            "Diligent map tile set retained failure backoff after leaving the active window");
    require(renderer.requestMapTileWindow(map, 0.0, 0.0, 0, {}, 0) == 0,
            "Diligent frame host scheduled tiles for an empty map");
    struct TestMapSource {
        const openbus::map::MapDefinition& definition;
        std::filesystem::path assetRoot;
        std::shared_ptr<const openbus::map::MapRoadConnectionIndex> roadConnections;
        std::size_t baseTextureIndex = 1;
        std::vector<MapPointLight> lights;
        int tileRadius = 0;
        std::size_t streamingUpdates = 0;
        double lastStreamingX = 0.0;
        double lastStreamingY = 0.0;
        bool streamingAdvanced = false;
        void updateStreaming(double worldX, double worldY) {
            ++streamingUpdates;
            lastStreamingX = worldX;
            lastStreamingY = worldY;
            streamingAdvanced = true;
        }
        const openbus::map::MapDefinition& mapDefinition() const noexcept {
            return definition;
        }
        const std::filesystem::path& omsiRoot() const noexcept {
            return assetRoot;
        }
        std::size_t baseGroundTextureIndex() const noexcept {
            return baseTextureIndex;
        }
        std::shared_ptr<const openbus::map::MapRoadConnectionIndex>
        roadConnectionIndex() const noexcept {
            return roadConnections;
        }
        std::vector<MapPointLight> activeSceneryLights() const {
            return lights;
        }
        int activeTileRadius() const noexcept {
            return tileRadius;
        }
        std::size_t activeTreeSeason() const noexcept {
            return seasonIndex;
        }
        struct ScenerySources {
            std::vector<std::pair<std::filesystem::path, std::filesystem::path>> assets;
            std::vector<std::filesystem::path> seasonalTextures;
        };
        std::vector<openbus::app::DiligentMapSceneryPlacement> placements;
        std::size_t seasonIndex = 0;
        ScenerySources activeSceneryAssetSources() const {
            return {};
        }
        std::vector<openbus::app::DiligentMapSceneryPlacement> activeSceneryPlacements() const {
            return streamingAdvanced ? placements
                                     : std::vector<openbus::app::DiligentMapSceneryPlacement>{};
        }
    };
    require(renderer.requestMapTileWindow(TestMapSource{map, {}, {}}, 0.0, 0.0, 0) == 0,
            "Diligent frame host requested map tiles before the road index was finalized");
    auto readyRoadIndex =
        std::make_shared<openbus::map::MapRoadConnectionIndex>(map.coordinates);
    readyRoadIndex->finalize();
    bool readyMapSourceValidated = false;
    try {
        renderer.requestMapTileWindow(TestMapSource{map, {}, readyRoadIndex}, 0.0, 0.0, 0);
    } catch (const std::out_of_range&) {
        readyMapSourceValidated = true;
    }
    require(readyMapSourceValidated,
            "Diligent frame host did not pass the finalized map source to tile validation");
    require(renderer.publishReadyMapTiles() == 0,
            "Diligent frame host published tiles for an empty map");
    require(renderer.removeInactiveMapTiles() == 0,
            "Diligent frame host evicted tiles from an empty map");
    const auto waitingMapUpdate =
        openbus::app::updateDiligentMapTiles(renderer, TestMapSource{map, {}, {}}, 0.0, 0.0, 0);
    require(!waitingMapUpdate.roadIndexReady && waitingMapUpdate.scheduledTiles == 0 &&
                waitingMapUpdate.publishedTiles == 0 && waitingMapUpdate.removedTiles == 0,
            "Diligent map bridge changed streaming state before road indexing completed");
    TestMapSource waitingMapSource{map, {}, {}};
    waitingMapSource.placements.push_back(
        {5000, "objects\\pending.sco", identity, {0.0, 0.0, 0.0}, 2.0, 0, 0});
    const auto waitingMapFrameUpdate = openbus::app::updateDiligentMapFrameResourcesAtPosition(
        renderer, waitingMapSource, 12.5, -8.0);
    require(!waitingMapFrameUpdate.tiles.roadIndexReady &&
                waitingMapSource.streamingUpdates == 1 &&
                waitingMapSource.lastStreamingX == 12.5 &&
                waitingMapSource.lastStreamingY == -8.0 &&
                waitingMapFrameUpdate.tiles.scheduledTiles == 0 &&
                waitingMapFrameUpdate.scenery.placements.empty() &&
                waitingMapFrameUpdate.scenery.requestedAssets == 0 &&
                waitingMapFrameUpdate.scenery.requestedSeasonalTextures == 0,
            "Diligent map-frame update mishandled an unfinished map index");
    const auto readyMapUpdate = openbus::app::updateDiligentMapTiles(
        renderer, TestMapSource{map, {}, readyRoadIndex, 0}, 0.0, 0.0, 0);
    require(readyMapUpdate.roadIndexReady && readyMapUpdate.scheduledTiles == 0 &&
                readyMapUpdate.publishedTiles == 0 && readyMapUpdate.removedTiles == 0,
            "Diligent map bridge did not update an empty finalized map safely");
    const auto productionRadiusUpdate = openbus::app::updateDiligentMapTiles(
        renderer, TestMapSource{map, {}, readyRoadIndex, 0}, 0.0, 0.0);
    require(productionRadiusUpdate.roadIndexReady &&
                productionRadiusUpdate.scheduledTiles == 0 &&
                productionRadiusUpdate.publishedTiles == 0 &&
                productionRadiusUpdate.removedTiles == 0,
            "Diligent map bridge could not use the production tile-radius accessor");
    const auto emptySceneryUpdate =
        openbus::app::updateDiligentMapScenery(renderer, TestMapSource{map, {}, readyRoadIndex, 0});
    require(emptySceneryUpdate.requestedAssets == 0 &&
                emptySceneryUpdate.requestedSeasonalTextures == 0 &&
                emptySceneryUpdate.publishedResources == 0 &&
                emptySceneryUpdate.removedResources == 0 &&
                emptySceneryUpdate.placements.empty(),
            "Diligent map bridge changed scenery resources for an empty active window");
    TestMapSource placedScenerySource{map, {}, readyRoadIndex, 0};
    placedScenerySource.assetRoot = "test/assets";
    placedScenerySource.seasonIndex = 4;
    Matrix4 placedSceneryTransform = identity;
    placedSceneryTransform[12] = 45.0;
    Variables placedSceneryVariables;
    placedScenerySource.placements.push_back(
        {5001, "objects\\tree.sco", placedSceneryTransform, {45.0, 0.0, 0.0}, 6.0, 0, 0,
         &placedSceneryVariables});
    const auto placedMapFrameUpdate =
        openbus::app::updateDiligentMapFrameResourcesAtPosition(
            renderer, placedScenerySource, 24.0, 36.0);
    const auto& placedSceneryUpdate = placedMapFrameUpdate.scenery;
    require(placedSceneryUpdate.placements.size() == 1 &&
                placedMapFrameUpdate.tiles.roadIndexReady &&
                placedScenerySource.streamingUpdates == 1 &&
                placedScenerySource.streamingAdvanced &&
                placedScenerySource.lastStreamingX == 24.0 &&
                placedScenerySource.lastStreamingY == 36.0 &&
                placedSceneryUpdate.placements.front().motionObjectId == 5001 &&
                placedSceneryUpdate.placements.front().worldTransform[12] == 45.0 &&
                placedSceneryUpdate.placements.front().variables == &placedSceneryVariables &&
                placedSceneryUpdate.placements.front().seasonIndex == 4 &&
                placedSceneryUpdate.placements.front().assetRoot == "test/assets",
            "Diligent map bridge lost active placement state or season selection");
    std::array<openbus::app::DiligentMapSceneryPlacement, 3> transparentPlacements = {};
    transparentPlacements[0].motionObjectId = 501;
    transparentPlacements[1].motionObjectId = 502;
    transparentPlacements[2].motionObjectId = 503;
    for (auto& placement : transparentPlacements) {
        placement.configPath = "missing.sco";
        placement.assetRoot = "test/assets";
    }
    transparentPlacements[0].worldPosition = {0.0, 0.0, -2.0};
    transparentPlacements[1].worldPosition = {0.0, 0.0, -12.0};
    transparentPlacements[2].worldPosition = {0.0, 0.0, -12.0};
    require(openbus::app::orderDiligentMapSceneryPlacements(
                transparentPlacements, identity, DiligentSceneryPass::Transparent) ==
                    std::vector<std::size_t>{1, 2, 0} &&
                openbus::app::orderDiligentMapSceneryPlacements(
                    transparentPlacements, identity, DiligentSceneryPass::Opaque) ==
                    std::vector<std::size_t>{0, 1, 2},
            "Diligent map scenery did not stably order transparent placements back-to-front");
    Matrix4 invalidSceneryView = identity;
    invalidSceneryView[14] = std::numeric_limits<double>::quiet_NaN();
    bool invalidSceneryViewRejected = false;
    try {
        (void)openbus::app::orderDiligentMapSceneryPlacements(
            transparentPlacements, invalidSceneryView, DiligentSceneryPass::Transparent);
    } catch (const std::invalid_argument&) {
        invalidSceneryViewRejected = true;
    }
    require(invalidSceneryViewRejected,
            "Diligent map scenery accepted a non-finite transparent view depth");
    const auto transparentSubmissions = openbus::app::drawDiligentMapSceneryPlacements(
        renderer, transparentPlacements, DiligentSceneryPass::Transparent,
        vehicleFallback.shaderResourceView(), identity);
    require(transparentSubmissions.size() == 3 &&
                transparentSubmissions[0].motionObjectId == 502 &&
                transparentSubmissions[1].motionObjectId == 503 &&
                transparentSubmissions[2].motionObjectId == 501 &&
                std::all_of(transparentSubmissions.begin(), transparentSubmissions.end(),
                            [](const auto& submission) {
                                return submission.status ==
                                       openbus::app::DiligentMapSceneryPlacementDrawResult::Status::
                                           AssetPending;
                            }) &&
                std::all_of(transparentSubmissions.begin(), transparentSubmissions.end(),
                            [](const auto& submission) { return !submission.drawResult; }),
            "Diligent map scenery orchestration lost transparent order or pending asset state");
    auto offscreenPlacement = transparentPlacements[0];
    offscreenPlacement.motionObjectId = 504;
    offscreenPlacement.worldTransform = identity;
    offscreenPlacement.worldTransform[12] = 1000.0;
    offscreenPlacement.worldPosition = {1000.0, 0.0, 0.0};
    const std::array<openbus::app::DiligentMapSceneryPlacement, 1> offscreenPlacements = {
        offscreenPlacement};
    const auto offscreenSubmissions = openbus::app::drawDiligentMapSceneryPlacements(
        renderer, offscreenPlacements, DiligentSceneryPass::Opaque,
        vehicleFallback.shaderResourceView(), identity);
    require(offscreenSubmissions.size() == 1 &&
                offscreenSubmissions[0].status ==
                    openbus::app::DiligentMapSceneryPlacementDrawResult::Status::AssetPending &&
                !offscreenSubmissions[0].drawResult,
            "Diligent map scenery changed pending-asset reporting for offscreen placements");
    const std::array<openbus::app::DiligentMapSceneryRuntimeState, 1> incompleteRuntimeStates = {};
    bool incompleteSceneryRuntimeStatesRejected = false;
    try {
        (void)openbus::app::drawDiligentMapSceneryPlacements(
            renderer, transparentPlacements, DiligentSceneryPass::Opaque,
            vehicleFallback.shaderResourceView(), identity, incompleteRuntimeStates);
    } catch (const std::invalid_argument&) {
        incompleteSceneryRuntimeStatesRejected = true;
    }
    require(incompleteSceneryRuntimeStatesRejected,
            "Diligent map scenery orchestration accepted misaligned runtime texture state");
    require(!openbus::app::drawCachedDiligentMapSceneryPlacement(
                 renderer, placedSceneryUpdate.placements.front(), DiligentSceneryPass::Opaque,
                 vehicleFallback.shaderResourceView())
                 .has_value(),
            "Diligent map scenery reported a draw before its asset was published");
    bool mapTileAccessRejectedOutsideScene = false;
    try {
        (void)renderer.mapTiles();
    } catch (const std::logic_error&) {
        mapTileAccessRejectedOutsideScene = true;
    }
    require(mapTileAccessRejectedOutsideScene,
            "Diligent frame host exposed map tiles outside scene submission");
    bool mapDrawRejectedOutsideScene = false;
    try {
        renderer.drawMapTerrain(renderer.targets().colorTexture(), {});
    } catch (const std::logic_error&) {
        mapDrawRejectedOutsideScene = true;
    }
    require(mapDrawRejectedOutsideScene,
            "Diligent frame host allowed map drawing outside scene submission");
    bool mapLightUpdateRejectedOutsideScene = false;
    try {
        renderer.setMapLights({}, {}, {}, 0.0, 0);
    } catch (const std::logic_error&) {
        mapLightUpdateRejectedOutsideScene = true;
    }
    require(mapLightUpdateRejectedOutsideScene,
            "Diligent frame host allowed map-light updates outside scene submission");
    bool sceneEnvironmentAccessRejectedOutsideScene = false;
    try {
        (void)renderer.sceneEnvironment();
    } catch (const std::logic_error&) {
        sceneEnvironmentAccessRejectedOutsideScene = true;
    }
    require(sceneEnvironmentAccessRejectedOutsideScene,
            "Diligent frame host exposed scene environment outside scene submission");
    bool vehicleDrawRejectedOutsideScene = false;
    try {
        renderer.drawVehicle(vehicleAsset, vehicleFallback.shaderResourceView(), identity, identity);
    } catch (const std::logic_error&) {
        vehicleDrawRejectedOutsideScene = true;
    }
    require(vehicleDrawRejectedOutsideScene,
            "Diligent frame host allowed vehicle draws outside scene submission");
    bool humanDrawRejectedOutsideScene = false;
    try {
        renderer.drawHumanAsset(*humanAsset, vehicleFallback.shaderResourceView(), identity,
                                identity, vehicleTestPose, nullptr, false);
    } catch (const std::logic_error&) {
        humanDrawRejectedOutsideScene = true;
    }
    require(humanDrawRejectedOutsideScene,
            "Diligent frame host allowed human draws outside scene submission");
    bool spotlightUpdateRejectedOutsideScene = false;
    try {
        renderer.setSpotLights({});
    } catch (const std::logic_error&) {
        spotlightUpdateRejectedOutsideScene = true;
    }
    require(spotlightUpdateRejectedOutsideScene,
            "Diligent frame host allowed spotlight updates outside scene submission");
    bool vehicleSpotlightUpdateRejectedOutsideScene = false;
    try {
        renderer.setVehicleSpotLights({}, 0);
    } catch (const std::logic_error&) {
        vehicleSpotlightUpdateRejectedOutsideScene = true;
    }
    require(vehicleSpotlightUpdateRejectedOutsideScene,
            "Diligent frame host allowed vehicle spotlight updates outside scene submission");

    MotionHistory history;
    const auto beginHistoryFrame = [&](std::uint64_t frameId, double timestamp, int width,
                                       int height) {
        const auto camera = openbus::app::makeDiligentCameraView(
            identity, identity, {0.0, 0.0, 0.0}, width, height);
        const auto environment = openbus::app::buildDiligentSceneEnvironment(
            camera, productionWeather, 0.0, false);
        const bool begun = renderer.beginFrame(history, frameId, timestamp, camera, width, height,
                                               environment);
        if (begun) {
            require(renderer.sceneEnvironment().clearColor == environment.clearColor &&
                        renderer.sceneEnvironment().fog.density == environment.fog.density &&
                        renderer.sceneEnvironment().lighting.lightDirectionView ==
                            environment.lighting.lightDirectionView,
                    "Diligent frame host did not retain the active view's scene environment");
        }
        return begun;
    };
    require(beginHistoryFrame(1, 1.0, 320, 240),
            "Diligent frame host rejected an active frame");
    const std::size_t opaqueVehicleBatchCount = renderer.drawVehiclePass(
        passVehicleAsset, vehicleFallback.shaderResourceView(), identity, identity,
        DiligentVehicleRenderPass::Opaque);
    const std::size_t transparentVehicleBatchCount = renderer.drawVehiclePass(
        passVehicleAsset, vehicleFallback.shaderResourceView(), identity, identity,
        DiligentVehicleRenderPass::Transparent);
    require(opaqueVehicleBatchCount == 1 && transparentVehicleBatchCount == 1,
            "Diligent vehicle pass routing did not separate opaque and blended batches");
    bool invalidVehiclePassRejected = false;
    try {
        (void)renderer.drawVehiclePass(
            passVehicleAsset, vehicleFallback.shaderResourceView(), identity, identity,
            static_cast<DiligentVehicleRenderPass>(255));
    } catch (const std::invalid_argument&) {
        invalidVehiclePassRejected = true;
    }
    require(invalidVehiclePassRejected,
            "Diligent vehicle pass routing accepted an unsupported pass");
    openbus::app::vehicle::DiligentVehicleSceneDrawState nearSceneVehicle;
    nearSceneVehicle.vehicleObjectId = nearSceneVehicleId;
    nearSceneVehicle.assetGeneration = 1;
    nearSceneVehicle.modelViewProjection = identity;
    nearSceneVehicle.modelView = identity;
    nearSceneVehicle.modelView[14] = -1.0;
    openbus::app::vehicle::DiligentVehicleSceneDrawState farSceneVehicle;
    farSceneVehicle.vehicleObjectId = farSceneVehicleId;
    farSceneVehicle.assetGeneration = 1;
    farSceneVehicle.modelViewProjection = identity;
    farSceneVehicle.modelView = identity;
    farSceneVehicle.modelView[14] = -3.0;
    const std::array<openbus::app::vehicle::DiligentVehicleSceneDrawState, 2>
        vehicleSceneDraws = {std::move(nearSceneVehicle), std::move(farSceneVehicle)};
    const auto transparentVehicleScenePass =
        openbus::app::vehicle::drawDiligentVehicleScenePass(
            vehicleSceneDraws, renderer, vehicleFallback.shaderResourceView(),
            DiligentVehicleRenderPass::Transparent);
    require(transparentVehicleScenePass.vehiclesDrawn == 2 &&
                transparentVehicleScenePass.batchesDrawn == 2 &&
                transparentVehicleScenePass.submissionOrder ==
                    std::vector<std::uint64_t>{farSceneVehicleId, nearSceneVehicleId},
            "Diligent vehicle scene pass did not sort visible blended vehicles back-to-front");
    const auto opaqueVehicleScenePass =
        openbus::app::vehicle::drawDiligentVehicleScenePass(
            vehicleSceneDraws, renderer, vehicleFallback.shaderResourceView(),
            DiligentVehicleRenderPass::Opaque);
    require(opaqueVehicleScenePass.vehiclesDrawn == 2 &&
                opaqueVehicleScenePass.batchesDrawn == 2 &&
                opaqueVehicleScenePass.submissionOrder ==
                    std::vector<std::uint64_t>{nearSceneVehicleId, farSceneVehicleId},
            "Diligent vehicle scene pass changed caller ordering for opaque vehicles");
    const auto vehicleShadowScenePass =
        openbus::app::vehicle::drawDiligentVehicleShadowSceneLayer(
            vehicleSceneDraws, renderer, 0, identity);
    require(vehicleShadowScenePass.vehiclesWithCasters == 2 &&
                vehicleShadowScenePass.castersDrawn == 4 &&
                vehicleShadowScenePass.submissionOrder ==
                    std::vector<std::uint64_t>{nearSceneVehicleId, farSceneVehicleId},
            "Diligent vehicle shadow scene pass did not preserve source alpha-mode caster policy");
    bool activeReflectionReconfigurationRejected = false;
    try {
        renderer.configureReflectionTargets(reflectionSizes);
    } catch (const std::logic_error&) {
        activeReflectionReconfigurationRejected = true;
    }
    require(activeReflectionReconfigurationRejected,
            "Diligent frame host allowed reflection target replacement during a scene");
    require(renderer.renderReflectionTarget(
                0, 1, true, [&](DiligentReflectionTarget& target,
                                DiligentTexturedMeshRenderer& reflectionMeshes) {
            require(target.size() == 64 &&
                        target.colorTexture().GetTexture()->GetDesc().Width == 64,
                    "Diligent reflection callback received an invalid target");
            require(renderer.drawWeatherCloudsInReflection(
                        diligentAdapterReflectionView.view, diligentAdapterReflectionView.projection,
                        renderer.sceneEnvironment().fog, openbus::WeatherCloudType::Cumulus2,
                        1200.0, 45.0, 3.0, 0.6, 0.0, 2.0),
                    "Diligent reflection target did not submit its camera-specific cloud pass");
            const auto groupTransforms =
                openbus::app::vehicle::buildDiligentVehicleViewGroupTransforms(
                    reflectionViewGroups, diligentAdapterReflectionView.view,
                    diligentAdapterReflectionView.projection,
                    openbus::rendering::ViewpointContext::PlayerExterior, -1);
            require(groupTransforms.size() == 1 && groupTransforms[0].visible &&
                        !groupTransforms[0].motion.has_value(),
                    "Diligent reflection view did not produce visible, history-free vehicle state");
            const std::array<MapPointLight, 1> reflectionMapLights = {
                MapPointLight{{1.0, 2.0, 3.0}, {0.4, 0.5, 0.6}, 20.0}};
            const TestMapSource reflectionLightSource{
                map, {}, readyRoadIndex, 0,
                std::vector<MapPointLight>(reflectionMapLights.begin(),
                                           reflectionMapLights.end())};
            require(openbus::app::uploadDiligentMapLightsInViewForEnvironment(
                        reflectionMeshes, renderer.immediateContext(), reflectionLightSource,
                        diligentAdapterReflectionView.view, {}, 0.5, 4) == 1,
                    "Diligent reflection view did not upload brightness-scaled map lights");
            require(openbus::app::drawDiligentMapTilesInView(
                        reflectionMapTiles, reflectionMeshes, renderer.immediateContext(),
                        vehicleFallback.shaderResourceView(),
                        diligentAdapterReflectionView.viewProjection,
                        openbus::app::DiligentMapTileViewPass::Terrain) == 1 &&
                        openbus::app::drawDiligentMapTilesInView(
                            reflectionMapTiles, reflectionMeshes, renderer.immediateContext(),
                            vehicleFallback.shaderResourceView(),
                            diligentAdapterReflectionView.viewProjection,
                            openbus::app::DiligentMapTileViewPass::OpaqueRoads) == 0 &&
                        openbus::app::drawDiligentMapTilesInView(
                            reflectionMapTiles, reflectionMeshes, renderer.immediateContext(),
                            vehicleFallback.shaderResourceView(),
                            diligentAdapterReflectionView.viewProjection,
                            openbus::app::DiligentMapTileViewPass::CutoutRoads) == 0 &&
                        openbus::app::drawDiligentMapTilesInView(
                            reflectionMapTiles, reflectionMeshes, renderer.immediateContext(),
                            vehicleFallback.shaderResourceView(),
                            diligentAdapterReflectionView.viewProjection,
                            openbus::app::DiligentMapTileViewPass::Water) == 0 &&
                        openbus::app::drawDiligentMapTilesInView(
                            reflectionMapTiles, reflectionMeshes, renderer.immediateContext(),
                            vehicleFallback.shaderResourceView(),
                            diligentAdapterReflectionView.viewProjection,
                            openbus::app::DiligentMapTileViewPass::TransparentRoads) == 0,
                    "Diligent secondary-view map-tile adapter did not route terrain and road/water passes");
            const std::array<DiligentSceneryOpaqueDraw, 2> reflectionSceneryDraws = {};
            openbus::app::DiligentMapSceneryPlacement reflectionSceneryPlacement;
            reflectionSceneryPlacement.seasonIndex = 0;
            const auto opaqueReflectionScenery =
                openbus::app::drawDiligentMapSceneryPlacementInViewWithResources(
                    renderer, reflectionSceneryPlacement, sceneryAsset, DiligentSceneryPass::Opaque,
                    reflectionMeshes, renderer.immediateContext(),
                    vehicleFallback.shaderResourceView(),
                    diligentAdapterReflectionView.viewProjection,
                    diligentAdapterReflectionView.view);
            const auto transparentReflectionScenery =
                openbus::app::drawDiligentMapSceneryInView(
                    sceneryAsset, DiligentSceneryPass::Transparent, reflectionMeshes,
                    renderer.immediateContext(), vehicleFallback.shaderResourceView(),
                    diligentAdapterReflectionView.viewProjection,
                    diligentAdapterReflectionView.view, reflectionSceneryDraws);
            require(opaqueReflectionScenery.batchesDrawn == 1 &&
                        transparentReflectionScenery.batchesDrawn == 1 &&
                        !opaqueReflectionScenery.motionHistoryValid &&
                        !transparentReflectionScenery.motionHistoryValid,
                    "Diligent reflection pass did not submit scenery batches without main-view motion history");
            auto pendingReflectionPlacement = reflectionSceneryPlacement;
            pendingReflectionPlacement.motionObjectId = 8101;
            pendingReflectionPlacement.configPath = "unpublished-reflection.sco";
            pendingReflectionPlacement.assetRoot = "test/assets";
            pendingReflectionPlacement.worldPosition = {0.0, 0.0, 0.5};
            pendingReflectionPlacement.worldTransform = identity;
            pendingReflectionPlacement.boundingRadius = 0.1;
            const std::array<openbus::app::DiligentMapSceneryPlacement, 1>
                pendingReflectionPlacements = {pendingReflectionPlacement};
            const auto pendingReflectionDraws =
                openbus::app::drawDiligentMapSceneryPlacementsInView(
                    renderer, pendingReflectionPlacements, DiligentSceneryPass::Opaque,
                    reflectionMeshes, renderer.immediateContext(),
                    vehicleFallback.shaderResourceView(), diligentAdapterReflectionView.view,
                    diligentAdapterReflectionView.projection);
            require(pendingReflectionDraws.size() == 1 &&
                        pendingReflectionDraws[0].status ==
                            openbus::app::DiligentMapSceneryPlacementDrawResult::Status::
                                AssetPending,
                    "Diligent secondary-view orchestration did not report an unpublished asset");
            pendingReflectionPlacement.motionObjectId = 8102;
            pendingReflectionPlacement.worldPosition = {1000.0, 0.0, 0.5};
            pendingReflectionPlacement.worldTransform[12] = 1000.0;
            const std::array<openbus::app::DiligentMapSceneryPlacement, 1>
                culledReflectionPlacements = {pendingReflectionPlacement};
            const auto culledReflectionDraws =
                openbus::app::drawDiligentMapSceneryPlacementsInView(
                    renderer, culledReflectionPlacements, DiligentSceneryPass::Opaque,
                    reflectionMeshes, renderer.immediateContext(),
                    vehicleFallback.shaderResourceView(), diligentAdapterReflectionView.view,
                    diligentAdapterReflectionView.projection);
            require(culledReflectionDraws.size() == 1 &&
                        culledReflectionDraws[0].status ==
                            openbus::app::DiligentMapSceneryPlacementDrawResult::Status::Culled &&
                        culledReflectionDraws[0].drawResult.has_value(),
                    "Diligent secondary-view orchestration did not cull an off-frustum placement");
            bool missingSecondaryViewScriptTextureRejected = false;
            try {
                (void)openbus::app::drawDiligentMapSceneryPlacementInViewWithResources(
                    renderer, reflectionSceneryPlacement, scriptedSceneryAsset,
                    DiligentSceneryPass::Opaque, reflectionMeshes, renderer.immediateContext(),
                    vehicleFallback.shaderResourceView(),
                    diligentAdapterReflectionView.viewProjection,
                    diligentAdapterReflectionView.view);
            } catch (const std::invalid_argument&) {
                missingSecondaryViewScriptTextureRejected = true;
            }
            require(missingSecondaryViewScriptTextureRejected,
                    "Diligent secondary-view scenery silently accepted missing script or text textures");
            const auto reflectedHuman = drawDiligentHumanAssetInView(
                *humanAsset, reflectionMeshes, renderer.immediateContext(),
                vehicleFallback.shaderResourceView(), identity, 1.0,
                diligentAdapterReflectionView.view, diligentAdapterReflectionView.projection,
                vehicleTestPose, false);
            require(reflectedHuman.drawn && !reflectedHuman.culled &&
                        !reflectedHuman.poseHistoryValid && !reflectedHuman.motionHistoryValid,
                    "Diligent reflection pass did not draw a pedestrian without main-view history");
            Matrix4 outsideReflectionTransform = identity;
            outsideReflectionTransform[12] = 1000.0;
            const auto culledReflectedHuman = drawDiligentHumanAssetInView(
                *humanAsset, reflectionMeshes, renderer.immediateContext(),
                vehicleFallback.shaderResourceView(), outsideReflectionTransform, 1.0,
                diligentAdapterReflectionView.view, diligentAdapterReflectionView.projection,
                vehicleTestPose, false);
            require(culledReflectedHuman.culled && !culledReflectedHuman.drawn,
                    "Diligent secondary-view pedestrian culling ignored the supplied frustum");
            const auto reflectedHumanActor = drawDiligentHumanTrafficActorInView(
                renderer, humanModelKey, humanRenderSource, openbus::map::MapTrafficPose{}, 0.0,
                0.0, std::nullopt, 1.0, reflectionMeshes, renderer.immediateContext(),
                vehicleFallback.shaderResourceView(), diligentAdapterReflectionView.view,
                diligentAdapterReflectionView.projection);
            require(reflectedHumanActor.drawn && !reflectedHumanActor.poseHistoryValid &&
                        !reflectedHumanActor.motionHistoryValid,
                    "Diligent reflection actor adapter did not reuse traffic pose construction");
            vehicleAsset.draw(reflectionMeshes, renderer.immediateContext(),
                              vehicleFallback.shaderResourceView(),
                              diligentAdapterReflectionView.viewProjection,
                              diligentAdapterReflectionView.view, {}, nullptr, {}, nullptr,
                              groupTransforms);
        }),
            "Diligent reflection pass did not render its initial visible target");
    require(renderer.targets().width() == 320 && renderer.targets().height() == 240,
            "Diligent reflection pass did not restore the active scene attachments");
    bool reflectionDrawFailurePropagated = false;
    try {
        renderer.renderReflectionTarget(
            1, 1, true, [](DiligentReflectionTarget&, DiligentTexturedMeshRenderer&) {
                throw std::runtime_error("intentional reflection probe failure");
            });
    } catch (const std::runtime_error&) {
        reflectionDrawFailurePropagated = true;
    }
    require(reflectionDrawFailurePropagated && renderer.targets().width() == 320 &&
                renderer.targets().height() == 240,
            "Diligent reflection failure was hidden or did not restore the scene attachments");
    (void)renderer.meshes();

    bool invalidDefaultAnisotropyRejected = false;
    try {
        renderer.meshes().setDefaultAnisotropy(3);
    } catch (const std::invalid_argument&) {
        invalidDefaultAnisotropyRejected = true;
    }
    require(invalidDefaultAnisotropyRejected,
            "Diligent mesh renderer accepted an unsupported default anisotropy level");
    require(renderer.maximumAnisotropy() >= 1,
            "Diligent frame renderer did not expose a valid anisotropy limit");
    renderer.setSpotLights(diligentSpotLights);
    renderer.setVehicleSpotLights(viewSpaceVehicleLights, 1);
    const auto firstManagedVehicleGroup =
        renderer.makeVehicleGroupTransform(3, 777, identity);
    require(firstManagedVehicleGroup.animationGroupIndex == 3 &&
                firstManagedVehicleGroup.motion.has_value() &&
                !firstManagedVehicleGroup.motion->historyValid,
            "Diligent frame host did not invalidate first-frame vehicle group motion");
    const auto hiddenVehicleGroup = renderer.makeVehicleGroupTransform(0, 778, identity, true,
                                                                        false);
    require(!hiddenVehicleGroup.visible && !hiddenVehicleGroup.motion.has_value(),
            "Diligent frame host recorded motion for a vehicle group that was not visible");
    const std::array<DiligentVehicleGroupTransform, 1> hiddenVehicleGroups = {
        hiddenVehicleGroup};
    renderer.drawVehicle(vehicleAsset, vehicleFallback.shaderResourceView(), identity, identity);
    renderer.drawVehicle(vehicleAsset, vehicleFallback.shaderResourceView(), identity, identity,
                         {}, nullptr, {}, nullptr, hiddenVehicleGroups);
    require(renderer.drawHumanAssetPass(
                *humanAsset, vehicleFallback.shaderResourceView(), identity, identity,
                vehicleTestPose, nullptr, false, DiligentHumanRenderPass::Opaque) == 1 &&
                renderer.drawHumanAssetPass(
                    *humanAsset, vehicleFallback.shaderResourceView(), identity, identity,
                    vehicleTestPose, nullptr, false,
                    DiligentHumanRenderPass::Transparent) == 1 &&
                renderer.drawHumanAssetPass(
                    *humanAsset, vehicleFallback.shaderResourceView(), identity, identity,
                    vehicleTestPose, nullptr, false, DiligentHumanRenderPass::All) == 2,
            "Diligent human asset pass selection did not separate opaque and transparent batches");
    bool invalidHumanPassRejected = false;
    try {
        (void)renderer.drawHumanAssetPass(
            *humanAsset, vehicleFallback.shaderResourceView(), identity, identity,
            vehicleTestPose, nullptr, false, static_cast<DiligentHumanRenderPass>(255));
    } catch (const std::invalid_argument&) {
        invalidHumanPassRejected = true;
    }
    require(invalidHumanPassRejected,
            "Diligent human asset accepted an invalid render pass");
    Matrix4 farHumanModelView = identity;
    farHumanModelView[14] = -20.0;
    Matrix4 nearHumanModelView = identity;
    nearHumanModelView[14] = -5.0;
    const std::array<DiligentHumanSceneDrawState, 2> humanSceneActors = {{
        {610, humanAsset, identity, nearHumanModelView, &vehicleTestPose},
        {611, humanAsset, identity, farHumanModelView, &vehicleTestPose},
    }};
    const auto opaqueHumanScene = drawDiligentHumanScenePass(
        renderer, vehicleFallback.shaderResourceView(), humanSceneActors,
        DiligentHumanRenderPass::Opaque);
    const auto transparentHumanScene = drawDiligentHumanScenePass(
        renderer, vehicleFallback.shaderResourceView(), humanSceneActors,
        DiligentHumanRenderPass::Transparent);
    require(opaqueHumanScene.actorsDrawn == 2 && opaqueHumanScene.submeshesDrawn == 2 &&
                opaqueHumanScene.submissionOrder == std::vector<std::uint64_t>{610, 611} &&
                transparentHumanScene.actorsDrawn == 2 &&
                transparentHumanScene.submeshesDrawn == 2 &&
                transparentHumanScene.submissionOrder == std::vector<std::uint64_t>{611, 610},
            "Diligent human scene passes did not preserve opaque order and sort transparency");
    auto duplicateHumanSceneActors = humanSceneActors;
    duplicateHumanSceneActors[1].objectId = duplicateHumanSceneActors[0].objectId;
    bool duplicateHumanSceneIdsRejected = false;
    try {
        static_cast<void>(drawDiligentHumanScenePass(
            renderer, vehicleFallback.shaderResourceView(), duplicateHumanSceneActors,
            DiligentHumanRenderPass::Opaque));
    } catch (const std::invalid_argument&) {
        duplicateHumanSceneIdsRejected = true;
    }
    require(duplicateHumanSceneIdsRejected,
            "Diligent human scene pass accepted duplicate actor IDs");
    Matrix4 firstManagedHumanTransform = identity;
    firstManagedHumanTransform[12] = -0.1;
    Matrix4 secondManagedHumanTransform = identity;
    secondManagedHumanTransform[12] = 0.1;
    std::array<DiligentHumanPlacementSceneDrawState, 2> managedHumanSceneActors = {};
    const auto firstPassengerMotionKey = openbus::app::waitingPassengerMotionObjectKey(12, 0);
    const auto secondPassengerMotionKey = openbus::app::waitingPassengerMotionObjectKey(12, 1);
    managedHumanSceneActors[0].poseKey =
        makeDiligentHumanPoseKey(firstPassengerMotionKey, 0);
    managedHumanSceneActors[0].motionKey = firstPassengerMotionKey;
    managedHumanSceneActors[0].asset = humanAsset;
    managedHumanSceneActors[0].worldTransform = firstManagedHumanTransform;
    managedHumanSceneActors[0].boundingRadius = 0.02;
    managedHumanSceneActors[0].currentPose = vehicleTestPose;
    managedHumanSceneActors[1].poseKey =
        makeDiligentHumanPoseKey(secondPassengerMotionKey, 0);
    managedHumanSceneActors[1].motionKey = secondPassengerMotionKey;
    managedHumanSceneActors[1].asset = humanAsset;
    managedHumanSceneActors[1].worldTransform = secondManagedHumanTransform;
    managedHumanSceneActors[1].boundingRadius = 0.02;
    managedHumanSceneActors[1].currentPose = vehicleTestPose;
    const auto managedHumanOpaquePass = drawDiligentHumanPlacementScenePass(
        renderer, vehicleFallback.shaderResourceView(), managedHumanSceneActors,
        DiligentHumanRenderPass::Opaque);
    const auto managedHumanTransparentPass = drawDiligentHumanPlacementScenePass(
        renderer, vehicleFallback.shaderResourceView(), managedHumanSceneActors,
        DiligentHumanRenderPass::Transparent);
    require(managedHumanOpaquePass.actorsDrawn == 2 &&
                    managedHumanOpaquePass.poseSubmissionOrder ==
                        std::vector<DiligentHumanPoseKey>{
                            makeDiligentHumanPoseKey(firstPassengerMotionKey, 0),
                            makeDiligentHumanPoseKey(secondPassengerMotionKey, 0)} &&
                    managedHumanTransparentPass.actorsDrawn == 2 &&
                    managedHumanTransparentPass.poseSubmissionOrder ==
                        managedHumanOpaquePass.poseSubmissionOrder &&
                    managedHumanOpaquePass.actorResults.size() == 2 &&
                    std::all_of(managedHumanOpaquePass.actorResults.begin(),
                                managedHumanOpaquePass.actorResults.end(),
                                [](const DiligentHumanPlacementActorResult& actor) {
                                    return actor.drawn && !actor.poseHistoryValid &&
                                           !actor.motionHistoryValid;
                                }),
                "Diligent managed human scene passes did not preserve distinct passenger identities");
    auto mismatchedHumanSceneActors = managedHumanSceneActors;
    mismatchedHumanSceneActors[1].motionKey = firstPassengerMotionKey;
    bool rejectedMismatchedHumanKeys = false;
    try {
        static_cast<void>(drawDiligentHumanPlacementScenePass(
            renderer, vehicleFallback.shaderResourceView(), mismatchedHumanSceneActors,
            DiligentHumanRenderPass::Opaque));
    } catch (const std::invalid_argument&) {
        rejectedMismatchedHumanKeys = true;
    }
    require(rejectedMismatchedHumanKeys,
            "Diligent human scene passes must reject mismatched pose and motion identities");
    const auto cutoutSceneryDraw = renderer.drawSceneryAsset(
        cutoutSceneryAsset, DiligentSceneryPass::Opaque,
        vehicleFallback.shaderResourceView(), identity, identity,
        cutoutSceneryDrawStates, noTreeDrawStates, {904, 0});
    require(cutoutSceneryDraw.batchesDrawn == 1,
            "Diligent cutout scenery did not submit through its depth-biased pipeline");
    bool sceneryDrawStateCountRejected = false;
    try {
        const std::array<DiligentSceneryOpaqueDraw, 1> incompleteDrawStates = {};
        (void)renderer.drawSceneryAsset(
            sceneryAsset, DiligentSceneryPass::Opaque,
            vehicleFallback.shaderResourceView(), identity, identity, incompleteDrawStates,
            noTreeDrawStates, {900, 0});
    } catch (const std::invalid_argument&) {
        sceneryDrawStateCountRejected = true;
    }
    require(sceneryDrawStateCountRejected,
            "Diligent scenery asset submission accepted incomplete per-batch draw state");
    const std::array<Diligent::ITextureView*, 1> incompleteSeasonalTextures = {
        &vehicleFallback.shaderResourceView()};
    bool sceneryResourceCountRejected = false;
    try {
        static_cast<void>(openbus::app::drawDiligentMapSceneryPlacementWithResources(
            renderer, mapSceneryPlacement, variableSceneryAsset,
            DiligentSceneryPass::Transparent, vehicleFallback.shaderResourceView(),
            incompleteSeasonalTextures));
    } catch (const std::invalid_argument&) {
        sceneryResourceCountRejected = true;
    }
    require(sceneryResourceCountRejected,
            "Diligent map scenery adapter accepted incomplete per-batch seasonal resources");
    bool missingSceneryScriptTextureRejected = false;
    try {
        static_cast<void>(openbus::app::drawDiligentMapSceneryPlacementWithResources(
            renderer, mapSceneryPlacement, scriptedSceneryAsset,
            DiligentSceneryPass::Transparent, vehicleFallback.shaderResourceView()));
    } catch (const std::invalid_argument&) {
        missingSceneryScriptTextureRejected = true;
    }
    require(missingSceneryScriptTextureRejected,
            "Diligent map scenery silently omitted a required script texture");
    const std::array<Diligent::ITextureView*, 2> sceneryRuntimeTextures = {
        &vehicleFallback.shaderResourceView(), &vehicleFallback.shaderResourceView()};
    bool missingSceneryTextTextureRejected = false;
    try {
        static_cast<void>(openbus::app::drawDiligentMapSceneryPlacementWithResources(
            renderer, mapSceneryPlacement, scriptedSceneryAsset,
            DiligentSceneryPass::Transparent, vehicleFallback.shaderResourceView(),
            {}, sceneryRuntimeTextures));
    } catch (const std::invalid_argument&) {
        missingSceneryTextTextureRejected = true;
    }
    require(missingSceneryTextTextureRejected,
            "Diligent map scenery silently omitted a required text texture");
    const auto variablePlacementDraw =
        openbus::app::drawDiligentMapSceneryPlacementWithResources(
            renderer, mapSceneryPlacement, variableSceneryAsset,
            DiligentSceneryPass::Transparent, vehicleFallback.shaderResourceView());
    placementVariables.set("placement_visible", 0.0F);
    const auto hiddenVariablePlacementDraw =
        openbus::app::drawDiligentMapSceneryPlacementWithResources(
            renderer, mapSceneryPlacement, variableSceneryAsset,
            DiligentSceneryPass::Transparent, vehicleFallback.shaderResourceView());
    require(variablePlacementDraw.batchesDrawn == 1 &&
                hiddenVariablePlacementDraw.batchesDrawn == 0 &&
                !hiddenVariablePlacementDraw.motionHistoryValid,
            "Diligent map placement did not apply per-instance visibility before motion history");
    placementVariables.set("placement_visible", 1.0F);
    const auto firstPlacementDraw = renderer.drawSceneryPlacement(
        sceneryAsset, identity, 0.1, DiligentSceneryPass::Transparent,
        vehicleFallback.shaderResourceView(), sceneryDrawStates, noTreeDrawStates, 900);
    auto hiddenSceneryDrawStates = sceneryDrawStates;
    for (DiligentSceneryOpaqueDraw& draw : hiddenSceneryDrawStates) {
        draw.visibilityFade = 0.0F;
    }
    const auto filteredOpaqueSceneryDraw = renderer.drawSceneryAsset(
        sceneryAsset, DiligentSceneryPass::Opaque, vehicleFallback.shaderResourceView(), identity,
        identity, hiddenSceneryDrawStates, noTreeDrawStates, {903, 0});
    Matrix4 culledPlacementTransform = identity;
    culledPlacementTransform[12] = 4.0;
    const auto culledPlacementDraw = renderer.drawSceneryPlacement(
        sceneryAsset, culledPlacementTransform, 0.1, DiligentSceneryPass::Opaque,
        vehicleFallback.shaderResourceView(), sceneryDrawStates, noTreeDrawStates, 902);
    require(firstPlacementDraw.batchesDrawn == 1 && culledPlacementDraw.culled &&
                culledPlacementDraw.batchesDrawn == 0 &&
                filteredOpaqueSceneryDraw.batchesDrawn == 0 &&
                !firstPlacementDraw.motionHistoryValid && !culledPlacementDraw.motionHistoryValid,
            "Diligent scenery placement submission did not cull or filter its render pass correctly");
    DiligentSceneryOpaqueDraw hiddenSceneryDraw;
    hiddenSceneryDraw.visibilityFade = 0.0F;
    require(!renderer.drawSceneryBatch(
                sceneryBatch, DiligentSceneryPass::Opaque,
                vehicleFallback.shaderResourceView(), identity, hiddenSceneryDraw) &&
                !renderer.drawSceneryBatch(
                    sceneryBatch, DiligentSceneryPass::Transparent,
                    vehicleFallback.shaderResourceView(), identity, hiddenSceneryDraw) &&
                !renderer.drawSceneryBatch(
                    sceneryBatch, DiligentSceneryPass::TransmapDepthPrepass,
                    vehicleFallback.shaderResourceView(), identity, hiddenSceneryDraw) &&
                !renderer.drawSceneryTreeBatch(sceneryTreeBatch,
                                               vehicleFallback.shaderResourceView(), identity,
                                               hiddenSceneryDraw),
            "Diligent scenery frame-host forwarding did not preserve the hidden-batch gate");
    openbus::map::MapTrafficPose invalidHumanTrafficPose;
    invalidHumanTrafficPose.headingRadians =
        std::numeric_limits<double>::quiet_NaN();
    bool invalidHumanTrafficPoseRejected = false;
    try {
        static_cast<void>(drawDiligentHumanTrafficActor(
            renderer, humanModelKey, humanRenderSource, invalidHumanTrafficPose, 0.0, 0.0,
            std::nullopt, 0.1, vehicleFallback.shaderResourceView(), {100, 0}, {100, 0}));
    } catch (const std::invalid_argument&) {
        invalidHumanTrafficPoseRejected = true;
    }
    require(invalidHumanTrafficPoseRejected,
            "Diligent human traffic adapter accepted a non-finite pose");
    openbus::map::MapTrafficPose initialHumanTrafficPose;
    const auto standingHumanPlacement = prepareDiligentHumanTrafficPlacement(
        renderer, humanModelKey, humanRenderSource, initialHumanTrafficPose, 0.0, 0.0,
        std::nullopt, 0.1, {630, 0}, {630, 0});
    const auto seatedHumanPlacement = prepareDiligentHumanTrafficPlacement(
        renderer, humanModelKey, humanRenderSource, initialHumanTrafficPose, 0.5, 0.25,
        HumanSeatPose{0.6, 0.75}, 0.1, {631, 0}, {631, 0});
    HumanTrafficRenderSource unpublishedHumanSource = humanRenderSource;
    unpublishedHumanSource.sourceGeneration = 3;
    const auto pendingHumanPlacement = prepareDiligentHumanTrafficPlacement(
        renderer, humanModelKey, unpublishedHumanSource, initialHumanTrafficPose, 0.0, 0.0,
        std::nullopt, 0.1, {632, 0}, {632, 0});
    require(standingHumanPlacement.has_value() && seatedHumanPlacement.has_value() &&
                !standingHumanPlacement->seated && seatedHumanPlacement->seated &&
                !pendingHumanPlacement,
            "Diligent human traffic preparation did not build standing/seated or pending states");
    const std::array<DiligentHumanPlacementSceneDrawState, 2> preparedHumanPlacements = {
        *standingHumanPlacement, *seatedHumanPlacement};
    const auto preparedHumanOpaquePass = drawDiligentHumanPlacementScenePass(
        renderer, vehicleFallback.shaderResourceView(), preparedHumanPlacements,
        DiligentHumanRenderPass::Opaque);
    const auto preparedHumanTransparentPass = drawDiligentHumanPlacementScenePass(
        renderer, vehicleFallback.shaderResourceView(), preparedHumanPlacements,
        DiligentHumanRenderPass::Transparent);
    require(preparedHumanOpaquePass.actorsDrawn == 2 &&
                preparedHumanTransparentPass.actorsDrawn == 2,
            "Diligent human traffic states did not submit through both scene material passes");
    const auto firstHumanPlacementDraw = drawDiligentHumanTrafficActor(
        renderer, humanModelKey, humanRenderSource, initialHumanTrafficPose, 0.0, 0.0,
        std::nullopt, 0.1, vehicleFallback.shaderResourceView(), {100, 0}, {100, 0});
    Matrix4 hiddenHumanTransform = identity;
    hiddenHumanTransform[12] = 4.0;
    const auto culledHumanPlacementDraw = renderer.drawHumanPlacement(
        *humanAsset, hiddenHumanTransform, 0.1, vehicleFallback.shaderResourceView(), {101, 0},
        {101, 0}, vehicleTestPose, false);
    const auto hiddenHumanPlacementDraw = renderer.drawHumanPlacement(
        *humanAsset, identity, 0.1, vehicleFallback.shaderResourceView(), {102, 0}, {102, 0},
        vehicleTestPose, false, {}, true, false);
    require(firstHumanPlacementDraw.drawn && !firstHumanPlacementDraw.motionHistoryValid &&
                culledHumanPlacementDraw.culled && !culledHumanPlacementDraw.drawn &&
                !hiddenHumanPlacementDraw.drawn && !hiddenHumanPlacementDraw.poseHistoryValid,
            "Diligent human placement did not preserve visibility, culling, or first-frame history");
    const auto unmatchedHumanPassDraw = renderer.drawHumanPlacement(
        *transparentOnlyHumanAsset, identity, 0.1, vehicleFallback.shaderResourceView(),
        {103, 0}, {103, 0}, vehicleTestPose, false, {}, true, true,
        DiligentHumanRenderPass::Opaque);
    HumanBonePalette transparentOnlyHumanPose = vehicleTestPose;
    transparentOnlyHumanPose[humanBoneSlotIndex(HumanBoneSlot::Main)][12] = 0.5;
    const auto matchingHumanPassDraw = renderer.drawHumanPlacement(
        *transparentOnlyHumanAsset, identity, 0.1, vehicleFallback.shaderResourceView(),
        {103, 0}, {103, 0}, transparentOnlyHumanPose, false, {}, true, true,
        DiligentHumanRenderPass::Transparent);
    require(!unmatchedHumanPassDraw.drawn && !unmatchedHumanPassDraw.poseHistoryValid &&
                matchingHumanPassDraw.drawn && !matchingHumanPassDraw.poseHistoryValid,
            "A human material pass with no matching geometry polluted rendered pose history");
    HumanBonePalette firstHumanPose = {};
    for (Matrix4& bone : firstHumanPose) {
        bone = identity;
    }
    firstHumanPose[humanBoneSlotIndex(HumanBoneSlot::Main)][12] = 1.25;
    const auto firstHumanHistory =
        renderer.humanPoseHistory().recordRenderedPose({77, 0}, firstHumanPose);
    require(!firstHumanHistory.historyValid && firstHumanHistory.previousPose == nullptr,
            "Diligent human pose history was valid on its first rendered frame");
    const DiligentMapTileSet* originalMapTiles = &renderer.mapTiles();
    Diligent::ITextureView& fallbackTexture = renderer.fallbackTexture();
    require(fallbackTexture.GetTexture()->GetDesc().Width == 1 &&
                fallbackTexture.GetTexture()->GetDesc().Height == 1 &&
                fallbackTexture.GetTexture()->GetDesc().Format == Diligent::TEX_FORMAT_RGBA8_UNORM,
            "Diligent frame host did not create its opaque white fallback texture");
    const auto emptyMainViewMap = openbus::app::drawDiligentMapMainView(
        renderer, fallbackTexture);
    const auto emptyOpaqueMainViewMap = openbus::app::drawDiligentMapMainView(
        renderer, fallbackTexture, {},
        openbus::app::DiligentMapMainViewPass::Opaque);
    const auto emptyTransparentMainViewMap = openbus::app::drawDiligentMapMainView(
        renderer, fallbackTexture, {},
        openbus::app::DiligentMapMainViewPass::Transparent);
    require(emptyMainViewMap.terrainBatches == 0 &&
                emptyMainViewMap.opaqueRoadBatches == 0 &&
                emptyMainViewMap.cutoutRoadBatches == 0 &&
                emptyMainViewMap.waterBatches == 0 &&
                emptyMainViewMap.transparentRoadBatches == 0 &&
                emptyOpaqueMainViewMap.terrainBatches == 0 &&
                emptyOpaqueMainViewMap.opaqueRoadBatches == 0 &&
                emptyOpaqueMainViewMap.cutoutRoadBatches == 0 &&
                emptyOpaqueMainViewMap.waterBatches == 0 &&
                emptyOpaqueMainViewMap.transparentRoadBatches == 0 &&
                emptyTransparentMainViewMap.terrainBatches == 0 &&
                emptyTransparentMainViewMap.opaqueRoadBatches == 0 &&
                emptyTransparentMainViewMap.cutoutRoadBatches == 0 &&
                emptyTransparentMainViewMap.waterBatches == 0 &&
                emptyTransparentMainViewMap.transparentRoadBatches == 0,
            "Diligent main-view map orchestration submitted batches for an empty map");
    const std::array<MapPointLight, 1> frameMapLights = {
        MapPointLight{{0.0, 0.0, 0.0}, {0.4, 0.5, 0.6}, 20.0}};
    openbus::app::uploadDiligentMapLights(
        renderer, TestMapSource{map, {}, readyRoadIndex, 0,
                                std::vector<MapPointLight>(frameMapLights.begin(),
                                                           frameMapLights.end())},
        identity, {}, 0.5, 4);
    bool mapTileUpdateRejectedDuringScene = false;
    try {
        renderer.requestMapTileWindow(map, 0.0, 0.0, 0, {}, 0);
    } catch (const std::logic_error&) {
        mapTileUpdateRejectedDuringScene = true;
    }
    require(mapTileUpdateRejectedDuringScene,
            "Diligent frame host allowed map streaming updates during scene drawing");
    bool mapTileUploadRejectedDuringScene = false;
    try {
        renderer.publishReadyMapTiles();
    } catch (const std::logic_error&) {
        mapTileUploadRejectedDuringScene = true;
    }
    require(mapTileUploadRejectedDuringScene,
            "Diligent frame host allowed map tile uploads during scene drawing");
    bool mapTileEvictionRejectedDuringScene = false;
    try {
        renderer.removeInactiveMapTiles();
    } catch (const std::logic_error&) {
        mapTileEvictionRejectedDuringScene = true;
    }
    require(mapTileEvictionRejectedDuringScene,
            "Diligent frame host allowed map tile eviction during scene drawing");
    bool sceneryUploadRejectedDuringScene = false;
    try {
        renderer.publishReadySceneryAssets();
    } catch (const std::logic_error&) {
        sceneryUploadRejectedDuringScene = true;
    }
    require(sceneryUploadRejectedDuringScene,
            "Diligent frame host allowed scenery resource uploads during scene drawing");
    bool sceneryEvictionRejectedDuringScene = false;
    try {
        renderer.retainSceneryAssets({}, {});
    } catch (const std::logic_error&) {
        sceneryEvictionRejectedDuringScene = true;
    }
    require(sceneryEvictionRejectedDuringScene,
            "Diligent frame host allowed scenery asset eviction during scene drawing");
    renderer.beginUiFrame(false);
    require((ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_NoMouseCursorChange) != 0,
            "Diligent frame host did not preserve the UI cursor policy");
    GLFWscrollfun diligentScrollCallback = glfwSetScrollCallback(&window, nullptr);
    require(diligentScrollCallback != nullptr,
            "Diligent ImGui platform did not install its scroll callback");
    glfwSetScrollCallback(&window, diligentScrollCallback);
    diligentScrollCallback(&window, 0.5, 1.0);
    require(scrollState.calls == 1 && scrollState.horizontal == 0.5 &&
                scrollState.vertical == 1.0,
            "Diligent ImGui platform did not chain the application scroll callback");
    ImGui::GetForegroundDrawList()->AddText({8.0F, 8.0F}, IM_COL32_WHITE,
                                            "Diligent frame host");
    openbus::rendering::RendererFramePresenter& presenter = renderer;
    presenter.present();
    require(history.frame().frameId == 1,
            "Diligent frame host did not retain the successfully submitted frame metadata");
    bool completedSceneEnvironmentAccessRejected = false;
    try {
        (void)renderer.sceneEnvironment();
    } catch (const std::logic_error&) {
        completedSceneEnvironmentAccessRejected = true;
    }
    require(completedSceneEnvironmentAccessRejected,
            "Diligent frame host retained scene environment after presentation");
    renderer.beginUiFrame(true);
    require((ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_NoMouseCursorChange) == 0,
            "Diligent frame host did not grant the UI cursor policy");
    require(ImGui::GetIO().MouseWheelH == 0.5F && ImGui::GetIO().MouseWheel == 1.0F,
            "Diligent ImGui platform did not forward scroll input to its UI context");
    ImGui::GetForegroundDrawList()->AddText({8.0F, 24.0F}, IM_COL32_WHITE,
                                            "Diligent UI-only frame");
    renderer.endUiOnlyFrame();
    bool duplicateUiOnlyFrameRejected = false;
    try {
        renderer.endUiOnlyFrame();
    } catch (const std::logic_error&) {
        duplicateUiOnlyFrameRejected = true;
    }
    require(duplicateUiOnlyFrameRejected,
            "Diligent frame host allowed UI-only presentation without an active UI frame");

    auto invalidMapReplacement = map;
    invalidMapReplacement.rootPath = "different-map";
    bool invalidMapReplacementRejected = false;
    try {
        renderer.requestMapTileWindow(invalidMapReplacement, 0.0, 0.0, 0, {}, 1);
    } catch (const std::out_of_range&) {
        invalidMapReplacementRejected = true;
    }
    require(invalidMapReplacementRejected,
            "Diligent frame host accepted an invalid ground-texture index");
    const auto roadFixtureDirectory =
        std::filesystem::temp_directory_path() /
        ("OpenBusDiligentRoadBridgeProbe-" +
         std::to_string(reinterpret_cast<std::uintptr_t>(&renderer)));
    std::filesystem::create_directories(roadFixtureDirectory / "Splines");
    const auto roadTexturePath = roadFixtureDirectory / "Splines" / "road.tga";
    const std::array<std::uint8_t, 22> roadTextureBytes = {
        0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 32, 40, 255, 255, 255, 255};
    {
        std::ofstream output(roadTexturePath, std::ios::binary);
        output.write(reinterpret_cast<const char*>(roadTextureBytes.data()),
                     static_cast<std::streamsize>(roadTextureBytes.size()));
        require(static_cast<bool>(output), "Failed to write Diligent bridge road texture fixture");
    }
    const auto groundTexturePath = roadFixtureDirectory / "road.tga";
    std::filesystem::copy_file(roadTexturePath, groundTexturePath,
                               std::filesystem::copy_options::overwrite_existing);
    auto roadSurfaceConfigPath = roadTexturePath;
    roadSurfaceConfigPath += ".cfg";
    {
        std::ofstream output(roadSurfaceConfigPath);
        output << "[moisture]\n";
        require(static_cast<bool>(output),
                "Failed to write Diligent road moisture configuration fixture");
    }
    const auto roadProfilePath = roadFixtureDirectory / "Splines" / "road.sli";
    {
        std::ofstream output(roadProfilePath);
        output << "[texture]\n"
               << roadTexturePath.string() << "\n"
                  "[matl_alpha]\n1\n"
                  "[profile]\n0\n"
                  "[profilepnt]\n-1\n0\n0\n0\n"
                  "[profilepnt]\n1\n0\n1\n0\n";
        require(static_cast<bool>(output), "Failed to write Diligent bridge road profile fixture");
    }
    const auto roadMapTextPath = roadFixtureDirectory / "road.map";
    {
        std::ofstream output(roadMapTextPath);
        output << "Diligent bridge road tile\n[version]\n14\n[terrain]\n"
               << "[spline]\n0\n"
               << roadProfilePath.string()
               << "\n1\n0\n2\n0\n0\n290\n0\n"
                  "10\n0\n0\n0\n0\n0\n0\n0\n0\n\n";
        require(static_cast<bool>(output), "Failed to write Diligent bridge map fixture");
    }
    const auto neighborRoadMapTextPath = roadFixtureDirectory / "road-neighbor.map";
    {
        std::ofstream output(neighborRoadMapTextPath);
        output << "Diligent bridge neighboring road tile\n[version]\n14\n[terrain]\n"
               << "[spline]\n0\n"
               << roadProfilePath.string()
               << "\n2\n1\n0\n0\n0\n0\n90\n"
                  "10\n0\n0\n0\n0\n0\n0\n0\n0\n\n";
        require(static_cast<bool>(output),
                "Failed to write Diligent neighboring road map fixture");
    }
    openbus::map::MapDefinition roadBridgeMap;
    roadBridgeMap.rootPath = roadFixtureDirectory;
    roadBridgeMap.coordinates.tileSize = 300.0;
    roadBridgeMap.groundTextures.push_back({groundTexturePath.string(), "", {}});
    openbus::map::MapTileReference roadBridgeTile;
    roadBridgeTile.terrainPath = roadFixtureDirectory / "road.terrain";
    roadBridgeTile.hasTerrainFile = true;
    roadBridgeTile.textPath = roadMapTextPath;
    {
        const std::array<std::uint8_t, 20> terrainBytes = {1};
        std::ofstream output(roadBridgeTile.terrainPath, std::ios::binary);
        output.write(reinterpret_cast<const char*>(terrainBytes.data()),
                     static_cast<std::streamsize>(terrainBytes.size()));
        require(static_cast<bool>(output), "Failed to write Diligent bridge terrain fixture");
    }
    roadBridgeTile.waterPath = roadFixtureDirectory / "road.water";
    roadBridgeTile.hasWaterFile = true;
    {
        const std::array<std::uint8_t, 20> waterBytes = {1};
        std::ofstream output(roadBridgeTile.waterPath, std::ios::binary);
        output.write(reinterpret_cast<const char*>(waterBytes.data()),
                     static_cast<std::streamsize>(waterBytes.size()));
        require(static_cast<bool>(output), "Failed to write Diligent bridge water fixture");
    }
    openbus::map::MapTileReference neighborRoadBridgeTile = roadBridgeTile;
    neighborRoadBridgeTile.y = 1;
    neighborRoadBridgeTile.terrainPath = roadFixtureDirectory / "road-neighbor.terrain";
    neighborRoadBridgeTile.textPath = neighborRoadMapTextPath;
    {
        const std::array<std::uint8_t, 20> terrainBytes = {1};
        std::ofstream output(neighborRoadBridgeTile.terrainPath, std::ios::binary);
        output.write(reinterpret_cast<const char*>(terrainBytes.data()),
                     static_cast<std::streamsize>(terrainBytes.size()));
        require(static_cast<bool>(output),
                "Failed to write Diligent neighboring terrain fixture");
    }
    roadBridgeMap.tiles.push_back(roadBridgeTile);
    roadBridgeMap.tiles.push_back(neighborRoadBridgeTile);
    const openbus::map::MapTileData roadBridgeMapData =
        openbus::map::loadMapTile(roadBridgeTile);
    const openbus::map::MapTileData neighborRoadBridgeMapData =
        openbus::map::loadMapTile(neighborRoadBridgeTile);
    require(roadBridgeMapData.splines.size() == 1 &&
                roadBridgeMapData.splines.front().geometryValid &&
                neighborRoadBridgeMapData.splines.size() == 1 &&
                neighborRoadBridgeMapData.splines.front().geometryValid,
            "Diligent bridge road fixtures did not parse valid splines");
    const auto roadBridgeProfile = openbus::map::loadMapSplineProfile(roadProfilePath);
    auto roadBridgeIndex = std::make_shared<openbus::map::MapRoadConnectionIndex>(
        roadBridgeMap.coordinates);
    roadBridgeIndex->addSpline(
        roadBridgeMapData.splines.front(), roadBridgeTile.x, roadBridgeTile.y,
        openbus::map::mapSplineProfileCantHalfWidth(roadBridgeProfile));
    roadBridgeIndex->addSpline(
        neighborRoadBridgeMapData.splines.front(), neighborRoadBridgeTile.x,
        neighborRoadBridgeTile.y,
        openbus::map::mapSplineProfileCantHalfWidth(roadBridgeProfile));
    roadBridgeIndex->finalize();
    TestMapSource roadBridgeSource{roadBridgeMap, {}, roadBridgeIndex, 0};
    const auto waitingRoadBridgeUpdate = openbus::app::updateDiligentMapTiles(
        renderer, TestMapSource{roadBridgeMap, {}, {}, 0}, 0.0, 0.0, 0);
    require(!waitingRoadBridgeUpdate.roadIndexReady &&
                waitingRoadBridgeUpdate.scheduledTiles == 0 &&
                waitingRoadBridgeUpdate.publishedTiles == 0 &&
                waitingRoadBridgeUpdate.removedTiles == 0,
            "Diligent map bridge scheduled a road-bearing tile before indexing completed");
    std::size_t roadBridgePublished = 0;
    bool roadGeometryPrepared = false;
    bool roadDrawSubmitted = false;
    require(beginHistoryFrame(2, 1.008, 320, 240),
            "Diligent frame host rejected a frame after invalid map replacement");
    bool throttledReflectionDrawCalled = false;
    require(!renderer.renderReflectionTarget(
                0, 2, true,
                [&](DiligentReflectionTarget&, DiligentTexturedMeshRenderer&) {
                    throttledReflectionDrawCalled = true;
                }) &&
                !throttledReflectionDrawCalled,
            "Diligent reflection frame-interval throttle did not preserve the prior target");
    require(renderer.isSphereVisibleInView(identity, {0.0, 0.0, 0.0}, 0.1) &&
                !renderer.isSceneryPlacementVisible(offscreenPlacement.worldTransform,
                                                    offscreenPlacement.boundingRadius),
            "Diligent frame host returned incorrect view-space sphere visibility");
    require(history.frame().historyValid,
            "Diligent frame host did not commit history for the next consecutive frame");
    Matrix4 movedVehicleGroupModelView = identity;
    movedVehicleGroupModelView[12] = 2.0;
    const auto secondManagedVehicleGroup =
        renderer.makeVehicleGroupTransform(3, 777, movedVehicleGroupModelView);
    require(secondManagedVehicleGroup.motion.has_value() &&
                secondManagedVehicleGroup.motion->historyValid &&
                secondManagedVehicleGroup.motion->previousModelViewProjection[12] == 0.0,
            "Diligent frame host did not retain previous rendered vehicle group transforms");
    const auto newlyVisibleVehicleGroup =
        renderer.makeVehicleGroupTransform(4, 777, identity);
    const auto restoredVehicleGroup = renderer.makeVehicleGroupTransform(0, 778, identity);
    require(newlyVisibleVehicleGroup.motion.has_value() &&
                !newlyVisibleVehicleGroup.motion->historyValid &&
                restoredVehicleGroup.motion.has_value() &&
                !restoredVehicleGroup.motion->historyValid,
            "Diligent frame host reused stale motion history for a newly visible vehicle group");
    Matrix4 movedSceneryModelView = identity;
    movedSceneryModelView[12] = 0.25;
    const auto secondPlacementDraw = renderer.drawSceneryPlacement(
        sceneryAsset, movedSceneryModelView, 0.1, DiligentSceneryPass::Transparent,
        vehicleFallback.shaderResourceView(), sceneryDrawStates, noTreeDrawStates, 900);
    require(secondPlacementDraw.motionHistoryValid,
            "Diligent scenery placement submission did not reuse the previous rendered transform");
    Matrix4 movedVariablePlacementTransform = identity;
    movedVariablePlacementTransform[12] = 0.125;
    mapSceneryPlacement.worldTransform = movedVariablePlacementTransform;
    const auto secondVariablePlacementDraw = openbus::app::drawDiligentMapSceneryPlacement(
        renderer, mapSceneryPlacement, variableSceneryAsset, DiligentSceneryPass::Transparent,
        vehicleFallback.shaderResourceView(), sceneryDrawStates, noTreeDrawStates);
    require(secondVariablePlacementDraw.motionHistoryValid,
            "Diligent map placement did not retain motion from its prior visible frame");
    HumanBonePalette secondHumanPose = firstHumanPose;
    secondHumanPose[humanBoneSlotIndex(HumanBoneSlot::Main)][12] = 2.5;
    const auto secondHumanHistory =
        renderer.humanPoseHistory().recordRenderedPose({77, 0}, secondHumanPose);
    const auto restoredCulledHumanHistory =
        renderer.humanPoseHistory().recordRenderedPose({101, 0}, secondHumanPose);
    const auto restoredHiddenHumanHistory =
        renderer.humanPoseHistory().recordRenderedPose({102, 0}, secondHumanPose);
    require(secondHumanHistory.historyValid && secondHumanHistory.previousPose != nullptr &&
                (*secondHumanHistory.previousPose)
                        [humanBoneSlotIndex(HumanBoneSlot::Main)][12] == 1.25 &&
                !restoredCulledHumanHistory.historyValid &&
                !restoredHiddenHumanHistory.historyValid,
            "Diligent frame host did not retain the last successfully rendered human pose");
    HumanBonePalette secondVehicleTestPose = vehicleTestPose;
    secondVehicleTestPose[humanBoneSlotIndex(HumanBoneSlot::Main)][12] = 0.25;
    Matrix4 movedHumanModelView = identity;
    movedHumanModelView[12] = 1.0;
    const auto secondHumanPlacementDraw = renderer.drawHumanPlacement(
        *humanAsset, movedHumanModelView, 0.1, vehicleFallback.shaderResourceView(), {100, 0},
        {100, 0}, secondVehicleTestPose, false);
    const auto secondVehiclePoseHistory =
        renderer.humanPoseHistory().recordRenderedPose(
            {100, 0}, secondVehicleTestPose, true, false, humanAsset->sourceGeneration());
    require(secondHumanPlacementDraw.drawn && secondHumanPlacementDraw.poseHistoryValid &&
                secondHumanPlacementDraw.motionHistoryValid &&
                secondVehiclePoseHistory.historyValid &&
                secondVehiclePoseHistory.previousPose != nullptr &&
                (*secondVehiclePoseHistory.previousPose)
                        [humanBoneSlotIndex(HumanBoneSlot::Main)][12] == 0.0,
            "Diligent frame host did not bind the previous rendered pose to the human draw");
    const std::array<openbus::rendering::DiligentHumanShadowDrawState, 2> humanShadowActors = {{
        {humanAsset, identity, &secondVehicleTestPose, false},
        {humanAsset, identity, &secondVehicleTestPose, true},
    }};
    const auto humanShadowScenePass =
        openbus::rendering::drawDiligentHumanShadowSceneLayer(renderer, 0, humanShadowActors);
    require(humanShadowScenePass.actorsWithCasters == 2 &&
                humanShadowScenePass.castersDrawn == 2,
            "Diligent human shadow scene pass did not gather casters across actors");
    require(&renderer.mapTiles() == originalMapTiles,
            "Invalid Diligent map replacement discarded the existing tile set");
    TestMapSource primaryMapLightSource{map, {}, readyRoadIndex};
    primaryMapLightSource.lights.assign(mapLightPolicyInput.begin(), mapLightPolicyInput.end());
    openbus::app::uploadDiligentMapLightsForEnvironment(
        renderer, primaryMapLightSource, identity, {}, 0.25, 4);
    openbus::app::uploadDiligentMapLightsForEnvironment(
        renderer, primaryMapLightSource, identity, {}, std::nullopt, 4);
    renderer.endFrame();
    const auto initialRoadBridgeUpdate = openbus::app::updateDiligentMapTiles(
        renderer, roadBridgeSource, 0.0, 0.0, 0);
    require(initialRoadBridgeUpdate.roadIndexReady &&
                initialRoadBridgeUpdate.scheduledTiles == 1,
            "Diligent map bridge did not schedule a road-bearing tile with its finalized index");
    roadBridgePublished = initialRoadBridgeUpdate.publishedTiles;
    const auto roadBridgeDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (roadBridgePublished == 0 &&
           std::chrono::steady_clock::now() < roadBridgeDeadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        roadBridgePublished += openbus::app::updateDiligentMapTiles(
                                   renderer, roadBridgeSource, 0.0, 0.0, 0)
                                   .publishedTiles;
    }
    require(roadBridgePublished == 1,
            "Diligent production map bridge did not publish the initial road tile");
    auto changedRoadSpline = roadBridgeMapData.splines.front();
    changedRoadSpline.length += 1.0;
    auto changedRoadBridgeIndex = std::make_shared<openbus::map::MapRoadConnectionIndex>(
        roadBridgeMap.coordinates);
    changedRoadBridgeIndex->addSpline(
        changedRoadSpline, roadBridgeTile.x, roadBridgeTile.y,
        openbus::map::mapSplineProfileCantHalfWidth(roadBridgeProfile));
    changedRoadBridgeIndex->addSpline(
        neighborRoadBridgeMapData.splines.front(), neighborRoadBridgeTile.x,
        neighborRoadBridgeTile.y,
        openbus::map::mapSplineProfileCantHalfWidth(roadBridgeProfile));
    changedRoadBridgeIndex->finalize();
    roadBridgeSource.roadConnections = changedRoadBridgeIndex;
    const auto changedRoadBridgeUpdate = openbus::app::updateDiligentMapTiles(
        renderer, roadBridgeSource, 0.0, 0.0, 0);
    require(changedRoadBridgeUpdate.roadIndexReady &&
                changedRoadBridgeUpdate.scheduledTiles == 1,
            "Diligent production map bridge did not invalidate tiles when the road index changed");
    std::size_t changedRoadBridgePublished = changedRoadBridgeUpdate.publishedTiles;
    const auto changedRoadBridgeDeadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (changedRoadBridgePublished == 0 &&
           std::chrono::steady_clock::now() < changedRoadBridgeDeadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        changedRoadBridgePublished += openbus::app::updateDiligentMapTiles(
                                          renderer, roadBridgeSource, 0.0, 0.0, 0)
                                          .publishedTiles;
    }

    renderer.resize(0, 240);
    require(renderer.isSuspended(), "Diligent frame host did not suspend at zero width");
    require(!beginHistoryFrame(3, 1.016, 320, 240),
            "Diligent frame host accepted scene work while suspended");
    renderer.resize(320, 240);
    require(!renderer.isSuspended(), "Diligent frame host did not resume at its prior size");
    require(beginHistoryFrame(3, 1.016, 320, 240),
            "Diligent frame host rejected a frame after same-size restoration");
    const auto roadBridgeTiles = renderer.mapTiles().activeTiles();
    roadGeometryPrepared =
        changedRoadBridgePublished == 1 && roadBridgeTiles.size() == 1 &&
        roadBridgeTiles.front()->roads().size() == 1 &&
        roadBridgeTiles.front()->roads().front().texture != nullptr &&
        roadBridgeTiles.front()->roads().front().alphaMode ==
            static_cast<int>(openbus::rendering::DiligentAlphaMode::Cutout) &&
        roadBridgeTiles.front()->roads().front().supportsMoisture &&
        roadBridgeTiles.front()->roads().front().weatherClassified;
    const auto roadBridgeDraw =
        openbus::app::drawDiligentMapMainView(renderer, renderer.fallbackTexture());
    roadDrawSubmitted = roadBridgeDraw.cutoutRoadBatches == 1;
    require(!history.frame().historyValid,
            "Diligent frame host left caller motion history valid after suspension");
    require(!renderer.frameData().historyValid,
            "Diligent frame host retained stale motion history after suspension");
    require(renderer.renderReflectionTarget(
                0, 3, true,
                [](DiligentReflectionTarget&, DiligentTexturedMeshRenderer&) {}),
            "Diligent reflection target did not refresh after its configured frame interval");
    const auto restoredHumanPose =
        renderer.humanPoseHistory().recordRenderedPose({77, 0}, secondHumanPose);
    require(!restoredHumanPose.historyValid && restoredHumanPose.previousPose == nullptr,
            "Diligent frame host retained human-pose history across suspension");
    renderer.endFrame();
    const auto outOfWindowRoadBridgeUpdate =
        openbus::app::updateDiligentMapTiles(renderer, roadBridgeSource, 1000.0, 0.0, 0);
    require(outOfWindowRoadBridgeUpdate.roadIndexReady &&
                outOfWindowRoadBridgeUpdate.removedTiles == 1,
            "Diligent production map bridge did not evict the out-of-window road tile");
    require(roadGeometryPrepared && roadDrawSubmitted,
            "Diligent production map bridge did not publish and submit cutout road geometry");

    {
        SimulationState mapSimulationState;
        openbus::rendering::AssetRequestManager mapAssetRequests;
        openbus::rendering::MapRenderer noGlMapRenderer(
            roadBridgeMap, 0, 0.0, 0.0, 0, roadFixtureDirectory, mapSimulationState,
            mapAssetRequests, false, false);
        const auto productionRoadIndex = noGlMapRenderer.roadConnectionIndex();
        require(noGlMapRenderer.initialWindowReady() && productionRoadIndex != nullptr,
                "Diligent map renderer did not initialize CPU map resources without OpenGL");
        const auto tileRoadJoins = productionRoadIndex->joinsFor(
            roadBridgeMapData.splines.front(), roadBridgeTile.x, roadBridgeTile.y);
        const auto neighborRoadJoins = productionRoadIndex->joinsFor(
            neighborRoadBridgeMapData.splines.front(), neighborRoadBridgeTile.x,
            neighborRoadBridgeTile.y);
        require(tileRoadJoins.end.has_value() && neighborRoadJoins.start.has_value(),
                "Production map loader did not connect road geometry across adjacent tiles");
        const auto initialNoGlMapUpdate =
            openbus::app::updateDiligentMapTiles(renderer, noGlMapRenderer, 0.0, 0.0);
        require(initialNoGlMapUpdate.roadIndexReady &&
                    initialNoGlMapUpdate.scheduledTiles == 2,
                "Diligent map renderer did not schedule its real road-bearing map tile");
        std::size_t noGlMapPublished = initialNoGlMapUpdate.publishedTiles;
        const auto noGlMapDeadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (noGlMapPublished == 0 &&
               std::chrono::steady_clock::now() < noGlMapDeadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            noGlMapPublished +=
                openbus::app::updateDiligentMapTiles(renderer, noGlMapRenderer, 0.0, 0.0)
                    .publishedTiles;
        }
        require(noGlMapPublished == 2,
                "Diligent map renderer did not publish its real road-bearing tile");

        renderer.invalidateHistory();
        require(beginHistoryFrame(4, 1.032, 320, 240),
                "Diligent frame host rejected a frame after explicit history invalidation");
        require(!history.frame().historyValid && !renderer.frameData().historyValid,
                "Diligent frame host did not apply explicit history invalidation");
        const auto noGlMapTiles = renderer.mapTiles().activeTiles();
        require(noGlMapTiles.size() == 2 &&
                    std::all_of(noGlMapTiles.begin(), noGlMapTiles.end(),
                                [](const auto* tile) {
                                    return tile->roads().size() == 1 &&
                                           tile->roads().front().supportsMoisture &&
                                       tile->roads().front().weatherClassified &&
                                       tile->water() != nullptr;
                                }),
                "Diligent map renderer did not preserve road and water resources without OpenGL");
        const auto noGlMapDraw =
            openbus::app::drawDiligentMapMainView(renderer, renderer.fallbackTexture());
        require(noGlMapDraw.terrainBatches == 2 && noGlMapDraw.cutoutRoadBatches == 2 &&
                    noGlMapDraw.waterBatches == 2,
                "Diligent map renderer did not submit terrain, water, and roads across adjacent "
                "tiles");
        renderer.endFrame();
    }

    std::error_code roadFixtureCleanupError;
    std::filesystem::remove_all(roadFixtureDirectory, roadFixtureCleanupError);
    require(!roadFixtureCleanupError,
            "Diligent bridge probe could not remove its road fixture directory");

    renderer.resize(0, 240);
    require(renderer.isSuspended(), "Diligent frame host did not suspend before resize");
    renderer.resize(160, 120);
    require(!renderer.isSuspended(), "Diligent frame host did not resume after resize");
    require(renderer.targets().width() == 160 && renderer.targets().height() == 120,
            "Diligent frame host did not resize scene attachments with the swap chain");

    require(beginHistoryFrame(5, 1.048, 160, 120),
            "Diligent frame host rejected a frame after resize");
    require(!renderer.frameData().historyValid,
            "Diligent frame host retained motion history after a resolution change");
    renderer.endFrame(DiligentColorFilter::Fxaa);

    require(beginHistoryFrame(6, 1.064, 160, 120),
            "Diligent frame host rejected a frame before testing failed composition");
    HumanBonePalette failedFrameHumanPose = secondHumanPose;
    failedFrameHumanPose[humanBoneSlotIndex(HumanBoneSlot::Main)][12] = 3.75;
    static_cast<void>(
        renderer.humanPoseHistory().recordRenderedPose({77, 0}, failedFrameHumanPose));
    bool invalidFilterRejected = false;
    try {
        renderer.endFrame(static_cast<DiligentColorFilter>(255));
    } catch (const std::invalid_argument&) {
        invalidFilterRejected = true;
    }
    require(invalidFilterRejected, "Diligent frame host accepted an invalid color filter");
    require(history.frame().frameId == 0,
            "Failed Diligent composition did not invalidate motion history");
    require(beginHistoryFrame(6, 1.064, 160, 120),
            "Diligent frame host did not recover from failed color composition");
    require(!history.frame().historyValid,
            "Diligent frame host retained external motion validity after failed composition");
    require(!renderer.frameData().historyValid,
            "Diligent frame host retained history after failed color composition");
    const auto recoveredHumanPose =
        renderer.humanPoseHistory().recordRenderedPose({77, 0}, failedFrameHumanPose);
    require(!recoveredHumanPose.historyValid && recoveredHumanPose.previousPose == nullptr,
            "Diligent frame host retained human-pose data from failed composition");
    renderer.endFrame();
    require(beginHistoryFrame(7, 1.080, 160, 120),
            "Diligent frame host rejected a frame before testing explicit abort");
    renderer.beginUiFrame(false);
    ImGui::GetForegroundDrawList()->AddText({8.0F, 8.0F}, IM_COL32_WHITE,
                                            "Aborted Diligent UI frame");
    static_cast<void>(
        renderer.humanPoseHistory().recordRenderedPose({77, 0}, secondHumanPose));
    renderer.abortFrame();
    require(history.frame().frameId == 0,
            "Diligent frame host did not invalidate motion history on explicit abort");
    require(beginHistoryFrame(7, 1.080, 160, 120) && !history.frame().historyValid &&
                !renderer.frameData().historyValid,
            "Diligent frame host did not invalidate temporal state after explicit abort");
    renderer.beginUiFrame(false);
    ImGui::GetForegroundDrawList()->AddText({8.0F, 8.0F}, IM_COL32_WHITE,
                                            "Recovered Diligent UI frame");
    const auto recoveredAfterAbort =
        renderer.humanPoseHistory().recordRenderedPose({77, 0}, secondHumanPose);
    require(!recoveredAfterAbort.historyValid && recoveredAfterAbort.previousPose == nullptr,
            "Diligent frame host retained human-pose history from an explicitly aborted frame");
    renderer.endFrame();
    const auto submissionCamera = openbus::app::makeDiligentCameraView(
        identity, identity, {0.0, 0.0, 0.0}, 160, 120);
    const auto submissionEnvironment = openbus::app::buildDiligentSceneEnvironment(
        submissionCamera, productionWeather, 0.0, false);
    require(renderer.renderFrame(
                history, 8, 1.096, submissionCamera, 160, 120, submissionEnvironment,
                [&](DiligentFrameRenderer& activeRenderer) {
                    require(&activeRenderer == &renderer &&
                                activeRenderer.frameData().frameId == 8 &&
                                activeRenderer.sceneEnvironment().fog.density ==
                                    submissionEnvironment.fog.density,
                            "Diligent frame transaction did not expose its active scene state");
                    const std::array<PrimitiveVertex, 3> primitiveVertices = {{
                        {-0.5F, -0.5F, 0.0F, 1.0F, 0.0F, 0.0F},
                        {0.5F, -0.5F, 0.0F, 0.0F, 1.0F, 0.0F},
                        {0.0F, 0.5F, 0.0F, 0.0F, 0.0F, 1.0F},
                    }};
                    activeRenderer.drawPrimitives(primitiveVertices, identity, identity);
                    activeRenderer.drawPrimitives(ticketVertices, identity, identity);
                },
                renderer.defaultColorFilter(), DiligentDebugView::MotionValidity),
            "Diligent frame transaction did not submit and present a valid scene");
    bool sceneSubmissionFailurePropagated = false;
    try {
        (void)renderer.renderFrame(
            history, 9, 1.112, submissionCamera, 160, 120, submissionEnvironment,
            [](DiligentFrameRenderer&) {
                throw std::runtime_error("intentional scene submission failure");
            });
    } catch (const std::runtime_error&) {
        sceneSubmissionFailurePropagated = true;
    }
    require(sceneSubmissionFailurePropagated && history.frame().frameId == 0,
            "Diligent frame transaction hid a scene exception or retained failed history");
    require(renderer.renderFrame(
                history, 10, 1.128, submissionCamera, 160, 120, submissionEnvironment,
                [&](DiligentFrameRenderer& activeRenderer) {
                    require(!history.frame().historyValid &&
                                !activeRenderer.frameData().historyValid,
                            "Diligent frame transaction did not invalidate failed temporal data");
                    require(activeRenderer.drawWeatherClouds(
                                openbus::WeatherCloudType::Overcast1, 1500.0, 0.0, 4.0, 0.6,
                                0.0, 2.0),
                            "Diligent frame host did not submit the procedural weather-cloud pass");
                    require(activeRenderer.drawWeatherPrecipitation(1, 0.001, 2.0, 180.0, 8.0,
                                                                   0.6) &&
                                activeRenderer.drawWeatherPrecipitation(
                                    2, 0.001, 2.0, 180.0, 8.0, 0.6),
                            "Diligent frame host did not submit rain and snow precipitation");
                }),
            "Diligent frame transaction did not recover after scene submission failure");
    const auto screenshotPath =
        std::filesystem::temp_directory_path() /
        ("OpenBusDiligentFrameRendererProbe-" +
         std::to_string(reinterpret_cast<std::uintptr_t>(&renderer)) + ".png");
    renderer.captureNextFrame(screenshotPath);
    require(renderer.renderFrame(history, 11, 1.144, submissionCamera, 160, 120,
                                 submissionEnvironment,
                                 [](DiligentFrameRenderer&) {}),
            "Diligent frame transaction did not capture a composed frame");
    require(!renderer.hasPendingCapture(),
            "Diligent frame host retained a screenshot request after capture");
    std::ifstream screenshot(screenshotPath, std::ios::binary);
    std::array<unsigned char, 8> pngSignature = {};
    screenshot.read(reinterpret_cast<char*>(pngSignature.data()),
                    static_cast<std::streamsize>(pngSignature.size()));
    const bool validPng =
        screenshot.gcount() == static_cast<std::streamsize>(pngSignature.size()) &&
        pngSignature == std::array<unsigned char, 8>{137, 80, 78, 71, 13, 10, 26, 10};
    screenshot.close();
    std::error_code screenshotRemovalError;
    std::filesystem::remove(screenshotPath, screenshotRemovalError);
    require(!screenshotRemovalError, "Diligent frame capture test could not remove its image");
    require(validPng, "Diligent frame capture did not write a valid PNG image");
    const std::array<std::string, 1> retainedHumanModels = {
        std::string(humanModelKey)};
    require(renderer.retainHumanAssets(retainedHumanModels) == 1 &&
                renderer.humanAsset(humanModelKey) != nullptr,
            "Diligent frame host did not retain the active human asset or evict the inactive one");
    const std::array<std::string, 0> noHumanModels = {};
    require(renderer.retainHumanAssets(noHumanModels) == 1 &&
                renderer.humanAsset(humanModelKey) == nullptr,
            "Diligent frame host did not evict an inactive human asset");
    const std::array<std::uint64_t, 3> retainedVehicles = {
        vehicleObjectId, farSceneVehicleId, nearSceneVehicleId};
    require(renderer.retainVehicleAssets(retainedVehicles) == 0 &&
                renderer.vehicleAssetCurrent(vehicleObjectId, 2),
            "Diligent frame host evicted a retained vehicle asset");
    const std::array<std::uint64_t, 0> noVehicles = {};
    require(renderer.retainVehicleAssets(noVehicles) == 3 &&
                renderer.vehicleAsset(vehicleObjectId) == nullptr &&
                renderer.vehicleAsset(farSceneVehicleId) == nullptr &&
                renderer.vehicleAsset(nearSceneVehicleId) == nullptr,
            "Diligent frame host did not evict an inactive vehicle asset");
    renderer.waitForIdle();
}

void runDiligentScriptTextureProbe(GLFWwindow& window) {
    using namespace Diligent;
    using namespace openbus::rendering;

    VehicleConfig configuration;
    Variables localState(ScriptObjectKind::SceneryObject);
    SimulationState sharedState;
    ScriptRuntime scripts(configuration, localState, sharedState);
    scripts.configureScriptTextures({ModelScriptTexture{0, 1, 1, {}}});
    Variables replacementLocalState(ScriptObjectKind::SceneryObject);
    SimulationState replacementSharedState;
    ScriptRuntime replacementScripts(configuration, replacementLocalState, replacementSharedState);
    replacementScripts.configureScriptTextures({ModelScriptTexture{0, 1, 1, {}}});

    openbus::core::GraphicsPreferences preferences;
    auto rendererOwner =
        openbus::app::createDiligentFrameRenderer(window, preferences, false);
    auto& renderer = *rendererOwner;
    verifyAlphaOnlyVehicleSampling(renderer.device(), renderer.immediateContext());
    auto& shadowMaps = renderer.configureSpotlightShadowMaps(32, 3);
    require(shadowMaps.resolution() == 32 && shadowMaps.capacity() == 3 &&
                renderer.spotlightShadowMaps() == &shadowMaps &&
                &shadowMaps.depthLayer(0) != &shadowMaps.depthLayer(2),
            "Diligent frame host did not retain the expected shadow-map array resource");
    require(&renderer.configureSpotlightShadowMaps(32, 3) == &shadowMaps,
            "Diligent frame host replaced an unchanged spotlight shadow-map resource");
    bool invalidShadowDimensionsRejected = false;
    try {
        (void)renderer.configureSpotlightShadowMaps(0, 3);
    } catch (const std::invalid_argument&) {
        invalidShadowDimensionsRejected = true;
    }
    require(invalidShadowDimensionsRejected && renderer.spotlightShadowMaps() == &shadowMaps,
            "Diligent frame host did not preserve shadow maps after invalid reconfiguration");
    (void)shadowMaps.shaderResourceView();
    shadowMaps.clearLayer(renderer.immediateContext(), 0);
    shadowMaps.clearLayer(renderer.immediateContext(), 2);
    bool invalidShadowLayerRejected = false;
    try {
        shadowMaps.clearLayer(renderer.immediateContext(), shadowMaps.capacity());
    } catch (const std::out_of_range&) {
        invalidShadowLayerRejected = true;
    }
    require(invalidShadowLayerRejected,
            "Diligent spotlight shadow maps accepted an out-of-range layer");
    bool invalidShadowClearRejected = false;
    try {
        shadowMaps.clearLayer(renderer.immediateContext(), 1, 1.1F);
    } catch (const std::invalid_argument&) {
        invalidShadowClearRejected = true;
    }
    require(invalidShadowClearRejected,
            "Diligent spotlight shadow maps accepted an invalid depth clear value");

    constexpr Matrix4 identity = {1, 0, 0, 0, 0, 1, 0, 0,
                                  0, 0, 1, 0, 0, 0, 0, 1};
    const auto sceneEnvironment = buildDiligentSceneEnvironment(
        identity, std::nullopt, std::nullopt, false, 0.0, std::nullopt, std::nullopt);
    const std::array<ModelVertex, 3> shadowVertices = {
        ModelVertex{-0.5F, -0.5F, 0.5F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        ModelVertex{0.5F, -0.5F, 0.5F, 1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        ModelVertex{0.0F, 0.5F, 0.5F, 0.5F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F},
    };
    DiligentTexturedMesh shadowMesh(renderer.device(), shadowVertices);
    std::array<HumanVertexWeights, 3> shadowWeights{};
    for (HumanVertexWeights& weights : shadowWeights) {
        weights[0] = 1.0F;
    }
    DiligentTexturedMesh skinnedShadowMesh(renderer.device(), shadowVertices, shadowWeights);
    HumanBonePalette shadowPose;
    for (Matrix4& bone : shadowPose) {
        bone = identity;
    }
    HumanMeshGeometry shadowHumanGeometry;
    shadowHumanGeometry.verticesByMaterial.emplace(
        0, std::vector<ModelVertex>(shadowVertices.begin(), shadowVertices.end()));
    auto shadowHumanImage = std::make_shared<Image>();
    shadowHumanImage->width = 1;
    shadowHumanImage->height = 1;
    shadowHumanImage->rgba = {255, 255, 255, 255};
    const std::unordered_map<int, std::shared_ptr<const Image>> shadowHumanTextures = {
        {0, shadowHumanImage}};
    DiligentHumanAsset shadowHumanAsset(renderer.device(), shadowHumanGeometry,
                                        shadowHumanTextures);
    DiligentMaterialState transparentHumanMaterial;
    transparentHumanMaterial.alphaMode = DiligentAlphaMode::Blend;
    const std::unordered_map<int, DiligentMaterialState> transparentHumanMaterials = {
        {0, transparentHumanMaterial}};
    DiligentHumanAsset transparentShadowHumanAsset(
        renderer.device(), shadowHumanGeometry, shadowHumanTextures, transparentHumanMaterials);
    require(transparentShadowHumanAsset.shadowCasters(identity, shadowPose, false).empty(),
            "Diligent human shadow bridge submitted a blended submesh as an opaque caster");
    const std::array<std::uint8_t, 4> alphaPixel = {255, 255, 255, 255};
    DiligentDynamicTexture2D alphaMask(renderer.device(), renderer.immediateContext(), 1, 1,
                                       alphaPixel);
    std::array<Image, 2> alphaArrayLayers;
    for (Image& layer : alphaArrayLayers) {
        layer.width = 1;
        layer.height = 1;
        layer.rgba = {255, 255, 255, 255};
    }
    DiligentTexture2DArray alphaArray(renderer.device(), alphaArrayLayers);
    auto vehicleArrayLayer0 = std::make_shared<Image>();
    auto vehicleArrayLayer1 = std::make_shared<Image>();
    for (const auto& layer : {vehicleArrayLayer0, vehicleArrayLayer1}) {
        layer->width = 1;
        layer->height = 1;
        layer->rgba = {255, 255, 255, 255};
    }
    const std::array<std::shared_ptr<const Image>, 2> vehicleArrayLayers = {
        vehicleArrayLayer0, vehicleArrayLayer1};
    std::array<ModelVertex, 3> vehicleArrayVertices = shadowVertices;
    for (ModelVertex& vertex : vehicleArrayVertices) {
        vertex.layer = 1.0F;
    }
    DiligentVehicleBatchSource vehicleCutoutBatch;
    vehicleCutoutBatch.vertices = vehicleArrayVertices;
    vehicleCutoutBatch.textureArrayLayers = vehicleArrayLayers;
    vehicleCutoutBatch.material.alphaMode = DiligentAlphaMode::Cutout;
    vehicleCutoutBatch.animationGroupIndex = 0;
    vehicleCutoutBatch.sourceBatchIndex = 0;
    DiligentVehicleBatchSource vehicleTransparentBatch = vehicleCutoutBatch;
    vehicleTransparentBatch.material.alphaMode = DiligentAlphaMode::NearOpaqueBlend;
    vehicleTransparentBatch.sourceBatchIndex = 1;
    DiligentVehicleBatchSource vehicleBlendedShadowBatch = vehicleCutoutBatch;
    vehicleBlendedShadowBatch.material.alphaMode = DiligentAlphaMode::Blend;
    vehicleBlendedShadowBatch.sourceBatchIndex = 3;
    DiligentVehicleBatchSource hiddenVehicleBatch;
    hiddenVehicleBatch.vertices = shadowVertices;
    hiddenVehicleBatch.animationGroupIndex = 1;
    hiddenVehicleBatch.sourceBatchIndex = 2;
    const std::array<DiligentVehicleBatchSource, 4> vehicleShadowBatches = {
        vehicleCutoutBatch, vehicleTransparentBatch, hiddenVehicleBatch,
        vehicleBlendedShadowBatch};
    DiligentVehicleAsset vehicleShadowAsset(renderer.device(), vehicleShadowBatches);
    DiligentVehicleGroupTransform visibleVehicleGroup;
    visibleVehicleGroup.animationGroupIndex = 0;
    visibleVehicleGroup.modelView = identity;
    visibleVehicleGroup.modelView[12] = 0.25;
    DiligentVehicleGroupTransform hiddenVehicleGroup;
    hiddenVehicleGroup.animationGroupIndex = 1;
    hiddenVehicleGroup.visible = false;
    const std::array<DiligentVehicleGroupTransform, 2> vehicleShadowTransforms = {
        visibleVehicleGroup, hiddenVehicleGroup};
    const std::vector<DiligentShadowCaster> vehicleShadowCasters =
        vehicleShadowAsset.shadowCasters(identity, identity, vehicleShadowTransforms);
    require(vehicleShadowCasters.size() == 2 &&
                vehicleShadowCasters[0].useVertexArrayLayer &&
                vehicleShadowCasters[0].alphaArrayLayer == 0 &&
                vehicleShadowCasters[0].alphaCutoff == 0.5F &&
                vehicleShadowCasters[0].modelViewProjection[12] == 0.25 &&
                vehicleShadowCasters[1].alphaCutoff == 0.0F &&
                vehicleShadowCasters[1].alphaTexture == nullptr,
            "Diligent vehicle shadow policy lost animation, cutout, or production blend behavior");
    DiligentMaterialState fadedCutoutMaterial = vehicleCutoutBatch.material;
    fadedCutoutMaterial.alphaMode = DiligentAlphaMode::Blend;
    fadedCutoutMaterial.baseColor[3] = 0.8F;
    fadedCutoutMaterial.visibilityFade = 0.75F;
    DiligentVehicleMaterialOverride fadedCutoutOverride;
    fadedCutoutOverride.material = &fadedCutoutMaterial;
    fadedCutoutOverride.animationGroupIndex = 0;
    fadedCutoutOverride.sourceBatchIndex = 0;
    const std::array<DiligentVehicleMaterialOverride, 1> fadedCutoutOverrides = {
        fadedCutoutOverride};
    const std::vector<DiligentShadowCaster> fadedVehicleShadowCasters =
        vehicleShadowAsset.shadowCasters(identity, identity, vehicleShadowTransforms,
                                          fadedCutoutOverrides);
    require(fadedVehicleShadowCasters.size() == 2 &&
                std::abs(fadedVehicleShadowCasters[0].alphaCutoff - (0.5F / 0.6F)) <
                    1.0e-6F &&
                fadedVehicleShadowCasters[1].alphaCutoff == 0.0F,
            "Diligent vehicle shadow cutouts did not apply runtime alpha and visibility fade");
    const std::array<DiligentShadowCaster, 4> shadowCasters = {
        DiligentShadowCaster{&shadowMesh, identity},
        DiligentShadowCaster{&shadowMesh, identity, &alphaMask.shaderResourceView(),
                             {1.0F, 1.0F, 0.0F, 0.0F}, 0.5F},
        DiligentShadowCaster{&shadowMesh, identity, &alphaArray.shaderResourceView(),
                             {1.0F, 1.0F, 0.0F, 0.0F}, 0.5F, 1},
        DiligentShadowCaster{&skinnedShadowMesh, identity, &alphaArray.shaderResourceView(),
                             {1.0F, 1.0F, 0.0F, 0.0F}, 0.5F, 1, &shadowPose}};
    PreparedSceneryAsset preparedScenery;
    PreparedSceneryMesh preparedMesh;
    preparedMesh.verticesByMaterial.emplace(
        0, std::vector<ModelVertex>{{-0.5F, -0.5F, 0.5F, 0.0F, 1.0F,
                                     0.0F, 0.0F, 0.0F, 1.0F},
                                    {0.5F, -0.5F, 0.5F, 1.0F, 1.0F,
                                     0.0F, 0.0F, 0.0F, 1.0F},
                                    {0.0F, 0.5F, 0.5F, 0.5F, 0.0F,
                                     0.0F, 0.0F, 0.0F, 1.0F}});
    preparedScenery.meshes.emplace("synthetic", std::move(preparedMesh));
    PreparedSceneryBatch scriptedBatch;
    scriptedBatch.meshKey = "synthetic";
    scriptedBatch.batch.vertexCount = 3;
    scriptedBatch.batch.scriptTextureIndex = 0;
    preparedScenery.batches.push_back(std::move(scriptedBatch));
    DiligentSceneryAsset sceneryAsset(renderer.device(), preparedScenery);

    MotionHistory history;
    require(renderer.beginFrame(history, 1, 1.0, 320, 240, identity, identity, 320, 240,
                                sceneEnvironment),
            "Diligent script texture probe could not begin a frame");
    bool activeShadowReconfigurationRejected = false;
    try {
        (void)renderer.configureSpotlightShadowMaps(64, 3);
    } catch (const std::logic_error&) {
        activeShadowReconfigurationRejected = true;
    }
    require(activeShadowReconfigurationRejected,
            "Diligent frame host reconfigured shadow resources during scene drawing");
    bool missingShadowAlphaRejected = false;
    const std::array<DiligentShadowCaster, 1> invalidShadowCasters = {
        DiligentShadowCaster{&shadowMesh, identity, nullptr, {1.0F, 1.0F, 0.0F, 0.0F}, 0.5F}};
    try {
        (void)renderer.drawOpaqueSpotlightShadowLayer(1, invalidShadowCasters);
    } catch (const std::invalid_argument&) {
        missingShadowAlphaRejected = true;
    }
    require(missingShadowAlphaRejected,
            "Diligent spotlight shadow pass accepted a cutout caster without an alpha texture");
    bool invalidAlphaArrayLayerRejected = false;
    const std::array<DiligentShadowCaster, 1> invalidArrayLayerCasters = {
        DiligentShadowCaster{&shadowMesh, identity, &alphaArray.shaderResourceView(),
                             {1.0F, 1.0F, 0.0F, 0.0F}, 0.5F, 2}};
    try {
        (void)renderer.drawOpaqueSpotlightShadowLayer(1, invalidArrayLayerCasters);
    } catch (const std::invalid_argument&) {
        invalidAlphaArrayLayerRejected = true;
    }
    require(invalidAlphaArrayLayerRejected,
            "Diligent spotlight shadow pass accepted an alpha-array layer outside its view");
    bool missingShadowPoseRejected = false;
    const std::array<DiligentShadowCaster, 1> missingShadowPoseCasters = {
        DiligentShadowCaster{&skinnedShadowMesh, identity}};
    try {
        (void)renderer.drawOpaqueSpotlightShadowLayer(1, missingShadowPoseCasters);
    } catch (const std::invalid_argument&) {
        missingShadowPoseRejected = true;
    }
    require(missingShadowPoseRejected,
            "Diligent skinned shadow caster accepted a missing current pose");
    require(renderer.drawOpaqueSpotlightShadowLayer(1, shadowCasters) == 4,
            "Diligent frame host did not render opaque, 2D-cutout, array-cutout, and skinned casters");
    require(renderer.drawOpaqueSpotlightShadowLayer(1, vehicleShadowCasters) == 2,
            "Diligent vehicle shadow bridge did not submit cutout and legacy solid-blend casters");
    require(drawDiligentHumanShadowLayer(renderer, shadowHumanAsset, 1, identity, shadowPose, false) ==
                1,
            "Diligent human shadow bridge did not submit the current skinned pose");
    DiligentSpotLight shadowedLight;
    shadowedLight.positionView = {0.0F, 0.0F, 0.0F};
    shadowedLight.directionView = {0.0F, 0.0F, 1.0F};
    const std::array<DiligentSpotLight, 1> shadowedLights = {shadowedLight};
    const SpotlightShadowCamera activeShadowCamera{identity, identity, 0.05, 100.0};
    const std::array<DiligentSpotlightShadowBinding, 1> activeShadowBindings = {
        DiligentSpotlightShadowBinding{activeShadowCamera, 1, 0.001F}};
    renderer.setSpotLightsWithShadows(shadowedLights, activeShadowBindings, identity, 0.0F);
    const std::array<VehicleSpotlight, 2> selectedShadowVehicleLights = {{
        {{1.0, 2.0, 3.0}, {0.0, 0.0, -2.0}, {0.25, 0.5, 0.75}, 18.0, 8.0, 22.0},
        {{4.0, 5.0, 6.0}, {1.0, 0.0, 0.0}, {1.0, 1.0, 1.0}, 12.0, 5.0, 18.0},
    }};
    renderer.setVehicleSpotLightsWithShadows(
        selectedShadowVehicleLights, activeShadowBindings, identity, 1, 0.0F);
    bool mismatchedVehicleShadowBindingsRejected = false;
    try {
        renderer.setVehicleSpotLightsWithShadows(
            selectedShadowVehicleLights, {}, identity, 1, 0.0F);
    } catch (const std::invalid_argument&) {
        mismatchedVehicleShadowBindingsRejected = true;
    }
    require(mismatchedVehicleShadowBindingsRejected,
            "Diligent vehicle spotlight upload accepted shadow bindings for another light set");
    renderer.meshes().draw(renderer.immediateContext(), shadowMesh,
                           alphaMask.shaderResourceView(), identity);
    bool unavailableShadowLayerRejected = false;
    const std::array<DiligentSpotlightShadowBinding, 1> invalidShadowBindings = {
        DiligentSpotlightShadowBinding{activeShadowCamera, shadowMaps.capacity(), 0.001F}};
    try {
        renderer.setSpotLightsWithShadows(shadowedLights, invalidShadowBindings, identity, 0.0F);
    } catch (const std::invalid_argument&) {
        unavailableShadowLayerRejected = true;
    }
    require(unavailableShadowLayerRejected,
            "Diligent frame host accepted a spotlight shadow layer outside its depth array");
    std::vector<ITextureView*> dynamicBatchTextures;
    std::vector<ITextureView*> textBatchTextures;
    require(!renderer.synchronizeMapSceneryRuntimeTextures(
                7101, sceneryAsset, nullptr, dynamicBatchTextures, textBatchTextures) &&
                dynamicBatchTextures.empty() && textBatchTextures.empty(),
            "Diligent map scenery accepted missing script runtime data");
    bool zeroSceneryMotionIdRejected = false;
    try {
        (void)renderer.synchronizeMapSceneryRuntimeTextures(
            0, sceneryAsset, &scripts, dynamicBatchTextures, textBatchTextures);
    } catch (const std::invalid_argument&) {
        zeroSceneryMotionIdRejected = true;
    }
    require(zeroSceneryMotionIdRejected,
            "Diligent map scenery accepted a zero runtime-texture placement ID");
    require(renderer.synchronizeMapSceneryRuntimeTextures(
                7101, sceneryAsset, &scripts, dynamicBatchTextures, textBatchTextures) &&
                dynamicBatchTextures.size() == 1 &&
                dynamicBatchTextures.front() != nullptr && textBatchTextures.empty(),
            "Diligent map scenery did not publish the configured script texture slot");
    RefCntAutoPtr<ITextureView> firstScriptTexture{dynamicBatchTextures.front()};
    renderer.endFrame();

    require(renderer.beginFrame(history, 2, 1.008, 320, 240, identity, identity, 320, 240),
            "Diligent script texture probe could not begin a second frame");
    require(renderer.synchronizeMapSceneryRuntimeTextures(
                7101, sceneryAsset, &scripts, dynamicBatchTextures, textBatchTextures) &&
                dynamicBatchTextures.size() == 1 &&
                dynamicBatchTextures.front() == firstScriptTexture.RawPtr(),
            "Diligent map scenery did not reuse its unchanged script texture slot");
    renderer.endFrame();

    require(renderer.beginFrame(history, 3, 1.016, 320, 240, identity, identity, 320, 240),
            "Diligent script texture probe could not begin a replacement frame");
    require(renderer.synchronizeMapSceneryRuntimeTextures(
                7101, sceneryAsset, &replacementScripts, dynamicBatchTextures,
                textBatchTextures) &&
                dynamicBatchTextures.size() == 1 &&
                dynamicBatchTextures.front() != firstScriptTexture.RawPtr(),
            "Diligent map scenery reused a prior runtime's script texture after replacement");
    renderer.endFrame();

    renderer.waitForIdle();
    const std::array<std::uint64_t, 1> retainedScenery = {7101};
    require(renderer.retainMapSceneryRuntimeTextures(retainedScenery) == 0,
            "Diligent frame host evicted a retained scenery script texture cache");
    bool zeroRetainedSceneryIdRejected = false;
    const std::array<std::uint64_t, 1> zeroSceneryId = {0};
    try {
        (void)renderer.retainMapSceneryRuntimeTextures(zeroSceneryId);
    } catch (const std::invalid_argument&) {
        zeroRetainedSceneryIdRejected = true;
    }
    require(zeroRetainedSceneryIdRejected,
            "Diligent frame host accepted a zero scenery runtime-texture retention ID");
    const std::array<std::uint64_t, 0> inactiveScenery = {};
    require(renderer.retainMapSceneryRuntimeTextures(inactiveScenery) == 1,
            "Diligent frame host did not evict an inactive scenery script texture cache");
}

void runHumanPoseSubobjectHistoryProbe() {
    using openbus::app::seatedPassengerMotionObjectKey;
    using openbus::app::waitingPassengerMotionObjectKey;
    using openbus::rendering::DiligentHumanPoseHistory;
    using openbus::rendering::HumanBonePalette;
    using openbus::rendering::Matrix4;
    using openbus::rendering::makeDiligentHumanPoseKey;

    const auto waitingA = waitingPassengerMotionObjectKey(12, 0);
    const auto waitingB = waitingPassengerMotionObjectKey(12, 1);
    const auto waitingPoseA = makeDiligentHumanPoseKey(waitingA, 0);
    const auto waitingPoseB = makeDiligentHumanPoseKey(waitingB, 0);
    require(waitingPoseA.objectId == waitingPoseB.objectId &&
                waitingPoseA.subObjectId != waitingPoseB.subObjectId,
            "Diligent waiting-passenger pose keys lost their arrival identity");

    const auto seatedA = seatedPassengerMotionObjectKey(42, 0);
    const auto seatedB = seatedPassengerMotionObjectKey(42, 1);
    require(makeDiligentHumanPoseKey(seatedA, 0).subObjectId !=
                makeDiligentHumanPoseKey(seatedB, 0).subObjectId,
            "Diligent seated-passenger pose keys lost their seat identity");

    const auto identity = [] {
        Matrix4 result{};
        result[0] = 1.0;
        result[5] = 1.0;
        result[10] = 1.0;
        result[15] = 1.0;
        return result;
    };
    HumanBonePalette previousA{};
    HumanBonePalette previousB{};
    for (Matrix4& bone : previousA) {
        bone = identity();
    }
    for (Matrix4& bone : previousB) {
        bone = identity();
    }
    previousA[0][12] = 1.0;
    previousB[0][12] = 2.0;

    DiligentHumanPoseHistory history;
    openbus::rendering::MotionDataFrame frame;
    frame.frameId = 1;
    frame.width = 320;
    frame.height = 240;
    history.beginFrame(frame);
    require(!history.recordRenderedPose(waitingPoseA, previousA, true, false, 10).historyValid &&
                !history.recordRenderedPose(waitingPoseB, previousB, true, false, 10).historyValid,
            "first-visible waiting passengers must not have valid pose history");
    history.completeFrame();

    frame.frameId = 2;
    frame.historyValid = true;
    history.beginFrame(frame);
    const auto currentA = history.recordRenderedPose(waitingPoseA, previousB, true, false, 10);
    const auto currentB = history.recordRenderedPose(waitingPoseB, previousA, true, false, 10);
    require(currentA.historyValid && currentB.historyValid &&
                currentA.previousPose != nullptr && currentB.previousPose != nullptr &&
                (*currentA.previousPose)[0][12] == 1.0 &&
                (*currentB.previousPose)[0][12] == 2.0,
            "waiting passengers sharing a stop must retain independent previous bone palettes");
    history.completeFrame();

    frame.frameId = 3;
    history.beginFrame(frame);
    const auto standingToSeated =
        history.recordRenderedPose(waitingPoseA, previousA, true, true, 10);
    require(!standingToSeated.historyValid && standingToSeated.previousPose == nullptr,
            "standing-to-seated transitions must invalidate the previous pose");
    history.completeFrame();

    frame.frameId = 4;
    history.beginFrame(frame);
    const auto stableSeated = history.recordRenderedPose(waitingPoseA, previousB, true, true, 10);
    require(stableSeated.historyValid && stableSeated.previousPose != nullptr &&
                (*stableSeated.previousPose)[0][12] == 1.0,
            "seated pose history should become valid after the transition frame");
    history.completeFrame();

    frame.frameId = 5;
    history.beginFrame(frame);
    const auto reloadedAsset = history.recordRenderedPose(waitingPoseA, previousA, true, true, 11);
    require(!reloadedAsset.historyValid && reloadedAsset.previousPose == nullptr,
            "human asset reloads must invalidate previous bone palettes");
    history.completeFrame();

    frame.frameId = 6;
    history.beginFrame(frame);
    const auto stableReloadedAsset =
        history.recordRenderedPose(waitingPoseA, previousB, true, true, 11);
    require(stableReloadedAsset.historyValid && stableReloadedAsset.previousPose != nullptr,
            "human pose history should recover after a stable asset-generation frame");
    history.completeFrame();
}

} // namespace

void runDiligentDebugViewPreferenceProbe() {
    using openbus::rendering::DiligentDebugView;
    const auto checkRoundTrip = [](std::string_view preference, DiligentDebugView view) {
        require(openbus::app::diligentDebugViewForPreference(preference) == view,
                "Diligent temporal debug preference mapped to the wrong view");
        require(openbus::app::preferenceForDiligentDebugView(view) == preference,
                "Diligent temporal debug view did not map back to its preference");
    };
    checkRoundTrip("off", DiligentDebugView::None);
    checkRoundTrip("motion_vectors", DiligentDebugView::MotionVectors);
    checkRoundTrip("motion_magnitude", DiligentDebugView::MotionMagnitude);
    checkRoundTrip("motion_validity", DiligentDebugView::MotionValidity);
    checkRoundTrip("linear_depth", DiligentDebugView::LinearDepth);

    bool invalidPreferenceRejected = false;
    try {
        static_cast<void>(openbus::app::diligentDebugViewForPreference("unknown"));
    } catch (const std::invalid_argument&) {
        invalidPreferenceRejected = true;
    }
    require(invalidPreferenceRejected, "unknown Diligent debug preference was accepted");
}

int main() {
    try {
        runDiligentDebugViewPreferenceProbe();
        runHumanPoseSubobjectHistoryProbe();
    } catch (const std::exception& error) {
        std::cerr << "Diligent human pose history probe failed: " << error.what() << '\n';
        return 1;
    }

    if (glfwInit() != GLFW_TRUE || glfwVulkanSupported() != GLFW_TRUE) {
        std::cerr << "GLFW could not initialize Vulkan for the Diligent frame-host probe.\n";
        glfwTerminate();
        return 77;
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(320, 240, "Diligent frame renderer probe", nullptr,
                                          nullptr);
    if (window == nullptr) {
        glfwTerminate();
        std::cerr << "GLFW could not create a Vulkan-compatible window.\n";
        return 77;
    }

    int result = 0;
    ScrollCallbackState scrollState;
    glfwSetWindowUserPointer(window, &scrollState);
    glfwSetScrollCallback(window, &recordScrollCallback);
    try {
        runProbe(*window, scrollState);
        runDiligentScriptTextureProbe(*window);
        std::cout << "Diligent-only frame lifecycle, UI composition, resize, suspension, "
                     "restoration, presentation, staging screenshot capture, and script texture "
                     "synchronization passed.\n";
    } catch (const std::exception& error) {
        std::cerr << "Diligent frame renderer probe failed: " << error.what() << '\n';
        result = 1;
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    return result;
}
