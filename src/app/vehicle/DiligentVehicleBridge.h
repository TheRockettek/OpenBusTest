#pragma once

#include "render/VehicleSpotlightData.h"
#include "render/diligent/DiligentVehicleAsset.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <stdexcept>
#include <stdexcept>
#include <vector>

struct Vehicle;

namespace openbus::rendering {
class DiligentFrameRenderer;
enum class ViewpointContext : int;
} // namespace openbus::rendering

namespace openbus::app::vehicle {

struct DiligentVehicleViewGroupInput {
    std::size_t animationGroupIndex = 0;
    openbus::rendering::Matrix4 localAnimationTransform = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                                           0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::array<double, 3> localCenter = {};
    double localRadius = 0.0;
    int viewpointMask = 0;
    int lodIndex = -1;
    bool variableVisible = true;
};

// Builds auxiliary-view transforms without touching temporal history. The caller supplies
// current animation matrices and visibility state evaluated for that view.
[[nodiscard]] std::vector<openbus::rendering::DiligentVehicleGroupTransform>
buildDiligentVehicleViewGroupTransforms(std::span<const DiligentVehicleViewGroupInput> groups,
                                        const openbus::rendering::Matrix4& modelView,
                                        const openbus::rendering::Matrix4& projection,
                                        openbus::rendering::ViewpointContext context,
                                        int activeLod);

[[nodiscard]] inline openbus::rendering::DiligentAlphaMode
toDiligentVehicleAlphaMode(int alphaMode) {
    switch (alphaMode) {
    case 0:
        return openbus::rendering::DiligentAlphaMode::Opaque;
    case 1:
        return openbus::rendering::DiligentAlphaMode::Cutout;
    case 2:
        return openbus::rendering::DiligentAlphaMode::Blend;
    case 3:
        return openbus::rendering::DiligentAlphaMode::NearOpaque;
    default:
        throw std::invalid_argument("Vehicle batch has an unsupported alpha mode");
    }
}

// Input spotlights are selected vehicle lights already transformed into the active view space.
[[nodiscard]] inline std::vector<openbus::rendering::DiligentSpotLight>
makeDiligentVehicleSpotLights(std::span<const openbus::rendering::VehicleSpotlight> viewSpaceLights,
                              std::size_t maximumLights) {
    const std::size_t count = (std::min)(maximumLights, viewSpaceLights.size());
    std::vector<openbus::rendering::DiligentSpotLight> result;
    result.reserve(count);
    const auto toFloat = [](double value) {
        if (!std::isfinite(value) ||
            std::abs(value) > static_cast<double>((std::numeric_limits<float>::max)())) {
            throw std::invalid_argument("Vehicle spotlight exceeds Diligent float range");
        }
        return static_cast<float>(value);
    };
    for (std::size_t index = 0; index < count; ++index) {
        const auto& source = viewSpaceLights[index];
        const auto isFinite = [](double value) { return std::isfinite(value); };
        if (!std::all_of(source.position.begin(), source.position.end(), isFinite) ||
            !std::all_of(source.direction.begin(), source.direction.end(), isFinite) ||
            !std::all_of(source.color.begin(), source.color.end(), isFinite) ||
            std::any_of(source.color.begin(), source.color.end(),
                        [](double value) { return value < 0.0 || value > 1.0; }) ||
            !isFinite(source.range) || source.range < (std::numeric_limits<float>::min)() ||
            !isFinite(source.innerAngleDegrees) || !isFinite(source.outerAngleDegrees) ||
            source.innerAngleDegrees < 0.0 || source.innerAngleDegrees > source.outerAngleDegrees ||
            source.outerAngleDegrees > 180.0 ||
            std::hypot(source.direction[0], source.direction[1], source.direction[2]) < 1.0e-12) {
            throw std::invalid_argument("Vehicle spotlight contains invalid parameters");
        }

        openbus::rendering::DiligentSpotLight light;
        for (std::size_t component = 0; component < 3; ++component) {
            light.positionView[component] = toFloat(source.position[component]);
            light.directionView[component] = toFloat(source.direction[component]);
            light.color[component] = toFloat(source.color[component]);
        }
        light.range = toFloat(source.range);
        light.innerAngleDegrees = toFloat(source.innerAngleDegrees);
        light.outerAngleDegrees = toFloat(source.outerAngleDegrees);
        if (light.range <= 0.0F || std::hypot(light.directionView[0], light.directionView[1],
                                              light.directionView[2]) <= 0.000001F) {
            throw std::invalid_argument("Vehicle spotlight is invalid in Diligent view space");
        }
        result.push_back(light);
    }
    return result;
}

// Call after the vehicle's animation state has been updated. Group indices match displayLists.
[[nodiscard]] bool
prepareDiligentVehicleAsset(Vehicle& vehicle,
                            openbus::rendering::DiligentFrameRenderer& frameRenderer);

struct DiligentVehiclePreparationSummary {
    std::size_t readyAssets = 0;
    std::size_t pendingAssets = 0;
    std::size_t evictedAssets = 0;
};

// Call outside scene submission with the live vehicle set. Asset failures propagate; incomplete
// asynchronous texture requests are reported as pending and retained for the next update.
[[nodiscard]] DiligentVehiclePreparationSummary
prepareDiligentVehicleAssets(std::span<Vehicle* const> vehicles,
                             openbus::rendering::DiligentFrameRenderer& frameRenderer);

// Draw a prepared vehicle during an active Diligent scene. Returns false while a referenced
// script/text texture has no drawable snapshot yet.
[[nodiscard]] bool drawDiligentVehicle(
    Vehicle& vehicle, openbus::rendering::DiligentFrameRenderer& frameRenderer,
    Diligent::ITextureView& fallbackTexture, const openbus::rendering::Matrix4& modelViewProjection,
    const openbus::rendering::Matrix4& modelView, openbus::rendering::ViewpointContext context,
    const openbus::rendering::DiligentLightingParameters& lighting = {},
    bool geometryHistoryValid = true);

// Submit a selected material pass using transforms/material overrides prepared once for this
// vehicle. Reuse the same spans for opaque and transparent passes to avoid recording motion twice.
[[nodiscard]] std::size_t drawDiligentVehiclePass(
    Vehicle& vehicle, openbus::rendering::DiligentFrameRenderer& frameRenderer,
    Diligent::ITextureView& fallbackTexture, const openbus::rendering::Matrix4& modelViewProjection,
    const openbus::rendering::Matrix4& modelView,
    openbus::rendering::DiligentVehicleRenderPass pass,
    std::span<const openbus::rendering::DiligentVehicleGroupTransform> groupTransforms,
    std::span<const openbus::rendering::DiligentVehicleMaterialOverride> materialOverrides,
    const openbus::rendering::DiligentLightingParameters& lighting = {});

// Submits a vehicle into an auxiliary view such as a reflection. This uses the supplied
// view/projection for animation transforms and culling, and intentionally does not write
// main-view motion history.
[[nodiscard]] bool drawDiligentVehicleInView(
    Vehicle& vehicle, openbus::rendering::DiligentFrameRenderer& frameRenderer,
    openbus::rendering::DiligentTexturedMeshRenderer& meshRenderer,
    Diligent::ITextureView& fallbackTexture, const openbus::rendering::Matrix4& modelView,
    const openbus::rendering::Matrix4& projection, openbus::rendering::ViewpointContext context,
    const openbus::rendering::DiligentLightingParameters& lighting = {});

[[nodiscard]] std::size_t drawDiligentVehicleShadowLayer(
    openbus::rendering::DiligentFrameRenderer& frameRenderer,
    const openbus::rendering::DiligentVehicleAsset& asset, Diligent::Uint32 lightIndex,
    const openbus::rendering::Matrix4& lightClipFromView,
    const openbus::rendering::Matrix4& modelView,
    std::span<const openbus::rendering::DiligentVehicleGroupTransform> groupTransforms = {},
    std::span<const openbus::rendering::DiligentVehicleMaterialOverride> materialOverrides = {});

[[nodiscard]] bool drawDiligentVehicleShadowLayer(
    Vehicle& vehicle, openbus::rendering::DiligentFrameRenderer& frameRenderer,
    Diligent::Uint32 lightIndex, const openbus::rendering::Matrix4& lightClipFromView,
    const openbus::rendering::Matrix4& modelView, openbus::rendering::ViewpointContext context,
    bool geometryHistoryValid = true);

void prepareDiligentVehicleGroupTransforms(
    const Vehicle& vehicle, const openbus::rendering::MotionDataFrame& frame,
    openbus::rendering::MotionHistory& history, const openbus::rendering::Matrix4& vehicleModelView,
    std::vector<openbus::rendering::DiligentVehicleGroupTransform>& transforms,
    bool geometryHistoryValid = true);

[[nodiscard]] std::vector<openbus::rendering::DiligentVehicleGroupTransform>
prepareDiligentVehicleGroupTransforms(const Vehicle& vehicle,
                                      const openbus::rendering::MotionDataFrame& frame,
                                      openbus::rendering::MotionHistory& history,
                                      const openbus::rendering::Matrix4& vehicleModelView,
                                      bool geometryHistoryValid = true);

// Uses the frame host's transaction-managed MotionHistory after current animation evaluation.
void prepareDiligentVehicleGroupTransforms(
    const Vehicle& vehicle, openbus::rendering::DiligentFrameRenderer& frameRenderer,
    const openbus::rendering::Matrix4& vehicleModelView,
    std::vector<openbus::rendering::DiligentVehicleGroupTransform>& transforms,
    openbus::rendering::ViewpointContext context, bool geometryHistoryValid = true);

} // namespace openbus::app::vehicle
