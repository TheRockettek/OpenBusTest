#include "app/vehicle/DiligentVehicleSceneBridge.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace openbus::app::vehicle {
namespace {

bool matrixIsFinite(const openbus::rendering::Matrix4& matrix) {
    return std::all_of(matrix.begin(), matrix.end(),
                       [](double value) { return std::isfinite(value); });
}

} // namespace

DiligentVehicleScenePassResult
drawDiligentVehicleScenePass(std::span<const DiligentVehicleSceneDrawState> drawStates,
                             openbus::rendering::DiligentFrameRenderer& frameRenderer,
                             Diligent::ITextureView& fallbackTexture,
                             openbus::rendering::DiligentVehicleRenderPass pass) {
    if (pass != openbus::rendering::DiligentVehicleRenderPass::All &&
        pass != openbus::rendering::DiligentVehicleRenderPass::Opaque &&
        pass != openbus::rendering::DiligentVehicleRenderPass::Transparent) {
        throw std::invalid_argument("Diligent vehicle scene pass is invalid");
    }

    DiligentVehicleScenePassResult result;
    std::vector<std::size_t> order(drawStates.size());
    std::unordered_set<std::uint64_t> uniqueIds;
    uniqueIds.reserve(drawStates.size());
    for (std::size_t index = 0; index < drawStates.size(); ++index) {
        const DiligentVehicleSceneDrawState& state = drawStates[index];
        if (state.vehicleObjectId == 0 || !uniqueIds.insert(state.vehicleObjectId).second) {
            throw std::invalid_argument(
                "Diligent vehicle scene pass requires unique stable vehicle IDs");
        }
        if (!matrixIsFinite(state.modelViewProjection) || !matrixIsFinite(state.modelView)) {
            throw std::invalid_argument("Diligent vehicle scene draw has invalid matrices");
        }
        if (!frameRenderer.vehicleAssetCurrent(state.vehicleObjectId, state.assetGeneration)) {
            throw std::logic_error(
                "Diligent vehicle scene pass references a stale or unpublished asset");
        }
        order[index] = index;
    }

    if (pass == openbus::rendering::DiligentVehicleRenderPass::Transparent) {
        std::stable_sort(order.begin(), order.end(), [&](std::size_t left, std::size_t right) {
            return drawStates[left].modelView[14] < drawStates[right].modelView[14];
        });
    }

    result.submissionOrder.reserve(order.size());
    for (const std::size_t index : order) {
        const DiligentVehicleSceneDrawState& state = drawStates[index];
        const openbus::rendering::DiligentVehicleAsset* asset =
            frameRenderer.vehicleAsset(state.vehicleObjectId);
        if (asset == nullptr) {
            throw std::logic_error("Diligent vehicle asset disappeared during scene submission");
        }
        const std::size_t batches = frameRenderer.drawVehiclePass(
            *asset, fallbackTexture, state.modelViewProjection, state.modelView, pass,
            state.lighting, nullptr, state.materialOverrides.overrides, nullptr,
            state.groupTransforms);
        result.submissionOrder.push_back(state.vehicleObjectId);
        result.batchesDrawn += batches;
        if (batches != 0) {
            ++result.vehiclesDrawn;
        }
    }
    return result;
}

DiligentVehicleShadowPassResult
drawDiligentVehicleShadowSceneLayer(std::span<const DiligentVehicleSceneDrawState> drawStates,
                                    openbus::rendering::DiligentFrameRenderer& frameRenderer,
                                    Diligent::Uint32 lightIndex,
                                    const openbus::rendering::Matrix4& lightClipFromView) {
    if (!matrixIsFinite(lightClipFromView)) {
        throw std::invalid_argument("Diligent vehicle shadow matrix must be finite");
    }

    DiligentVehicleShadowPassResult result;
    std::unordered_set<std::uint64_t> uniqueIds;
    uniqueIds.reserve(drawStates.size());
    std::vector<openbus::rendering::DiligentShadowCaster> casters;
    result.submissionOrder.reserve(drawStates.size());
    for (const DiligentVehicleSceneDrawState& state : drawStates) {
        if (state.vehicleObjectId == 0 || !uniqueIds.insert(state.vehicleObjectId).second) {
            throw std::invalid_argument(
                "Diligent vehicle shadow pass requires unique stable vehicle IDs");
        }
        if (!matrixIsFinite(state.modelView)) {
            throw std::invalid_argument(
                "Diligent vehicle shadow draw has an invalid model-view matrix");
        }
        if (!frameRenderer.vehicleAssetCurrent(state.vehicleObjectId, state.assetGeneration)) {
            throw std::logic_error(
                "Diligent vehicle shadow pass references a stale or unpublished asset");
        }
        const openbus::rendering::DiligentVehicleAsset* asset =
            frameRenderer.vehicleAsset(state.vehicleObjectId);
        if (asset == nullptr) {
            throw std::logic_error("Diligent vehicle asset disappeared during shadow submission");
        }
        auto vehicleCasters =
            asset->shadowCasters(lightClipFromView, state.modelView, state.groupTransforms,
                                 state.materialOverrides.overrides);
        if (!vehicleCasters.empty()) {
            ++result.vehiclesWithCasters;
            result.submissionOrder.push_back(state.vehicleObjectId);
            casters.insert(casters.end(), vehicleCasters.begin(), vehicleCasters.end());
        }
    }

    result.castersDrawn = frameRenderer.drawOpaqueSpotlightShadowLayer(lightIndex, casters);
    return result;
}

} // namespace openbus::app::vehicle
