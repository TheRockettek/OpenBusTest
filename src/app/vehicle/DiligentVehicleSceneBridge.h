#pragma once

#include "app/DiligentFrameRenderer.h"
#include "app/vehicle/DiligentVehicleSourceBridge.h"
#include "render/diligent/DiligentVehicleAsset.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace openbus::app::vehicle {

struct DiligentVehicleSceneDrawState {
    std::uint64_t vehicleObjectId = 0;
    std::uint64_t assetGeneration = 0;
    openbus::rendering::Matrix4 modelViewProjection = {};
    openbus::rendering::Matrix4 modelView = {};
    openbus::rendering::DiligentLightingParameters lighting{};
    DiligentVehicleMaterialOverrideSet materialOverrides;
    std::vector<openbus::rendering::DiligentVehicleGroupTransform> groupTransforms;
};

struct DiligentVehicleSceneDrawPreparation {
    enum class Status { AssetPending, MaterialPending, Ready } status = Status::AssetPending;
    std::optional<DiligentVehicleSceneDrawState> drawState;
};

struct DiligentVehicleScenePassResult {
    std::size_t vehiclesDrawn = 0;
    std::size_t batchesDrawn = 0;
    std::vector<std::uint64_t> submissionOrder;
};

struct DiligentVehicleShadowPassResult {
    std::size_t vehiclesWithCasters = 0;
    std::size_t castersDrawn = 0;
    std::vector<std::uint64_t> submissionOrder;
};

// Call once per vehicle per rendered frame during an active scene, even while its asset is pending.
// This updates animation and material state and records motion transforms once; reuse the returned
// state across scene passes.
[[nodiscard]] DiligentVehicleSceneDrawPreparation prepareDiligentVehicleSceneDraw(
    Vehicle& vehicle, openbus::rendering::DiligentFrameRenderer& frameRenderer,
    const openbus::rendering::Matrix4& modelView, const openbus::rendering::Matrix4& projection,
    openbus::rendering::ViewpointContext context,
    const openbus::rendering::DiligentLightingParameters& lighting = {},
    bool geometryHistoryValid = true);

// Draw a scene-wide vehicle pass. Transparent vehicles are submitted back-to-front by view-space
// origin; each asset further orders its own blended material batches.
[[nodiscard]] DiligentVehicleScenePassResult
drawDiligentVehicleScenePass(std::span<const DiligentVehicleSceneDrawState> drawStates,
                             openbus::rendering::DiligentFrameRenderer& frameRenderer,
                             Diligent::ITextureView& fallbackTexture,
                             openbus::rendering::DiligentVehicleRenderPass pass);

// Gather all vehicle casters before submitting the layer so the shared shadow depth slice is
// cleared once per light, not once per vehicle.
[[nodiscard]] DiligentVehicleShadowPassResult
drawDiligentVehicleShadowSceneLayer(std::span<const DiligentVehicleSceneDrawState> drawStates,
                                    openbus::rendering::DiligentFrameRenderer& frameRenderer,
                                    Diligent::Uint32 lightIndex,
                                    const openbus::rendering::Matrix4& lightClipFromView);

} // namespace openbus::app::vehicle
