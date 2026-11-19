#include "app/vehicle/DiligentVehicleBridge.h"

#include "app/DiligentFrameRenderer.h"
#include "app/vehicle/Vehicle.h"
#include "app/vehicle/DiligentVehicleSourceBridge.h"
#include "render/MatrixMath.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <type_traits>
#include <unordered_set>

namespace openbus::app::vehicle {
namespace {

double maximumVehicleMotionDeltaMeters(const openbus::rendering::MotionDataFrame& frame) noexcept {
    constexpr double maximumVehicleSpeedMetersPerSecond = 80.0;
    constexpr double displacementSlackMeters = 5.0;
    return displacementSlackMeters + maximumVehicleSpeedMetersPerSecond * frame.deltaSeconds;
}

} // namespace

bool prepareDiligentVehicleAsset(Vehicle& vehicle,
                                 openbus::rendering::DiligentFrameRenderer& frameRenderer) {
    if (!vehicle.prepareDiligentTextureRequests()) {
        return false;
    }

    if (frameRenderer.vehicleAssetCurrent(vehicle.motionObjectId,
                                          vehicle.diligentAssetGeneration)) {
        frameRenderer.prepareVehicleAsset(vehicle.motionObjectId, vehicle.diligentAssetGeneration,
                                          vehicle.materialSelectionGeneration, {});
        return true;
    }

    DiligentVehicleBatchSourceSet sources;
    if (!makeDiligentVehicleBatchSources(vehicle.displayLists, sources)) {
        return false;
    }
    frameRenderer.prepareVehicleAsset(vehicle.motionObjectId, vehicle.diligentAssetGeneration,
                                      vehicle.materialSelectionGeneration, sources.batches);
    return true;
}

DiligentVehiclePreparationSummary
prepareDiligentVehicleAssets(std::span<Vehicle* const> vehicles,
                             openbus::rendering::DiligentFrameRenderer& frameRenderer) {
    DiligentVehiclePreparationSummary summary;
    std::vector<std::uint64_t> liveVehicleIds;
    std::unordered_set<std::uint64_t> uniqueVehicleIds;
    liveVehicleIds.reserve(vehicles.size());
    uniqueVehicleIds.reserve(vehicles.size());
    for (Vehicle* vehicle : vehicles) {
        if (vehicle == nullptr || vehicle->motionObjectId == 0) {
            throw std::invalid_argument(
                "Diligent vehicle preparation requires non-null vehicles with stable IDs");
        }
        if (!uniqueVehicleIds.insert(vehicle->motionObjectId).second) {
            throw std::invalid_argument(
                "Diligent vehicle preparation received a duplicate vehicle ID");
        }
        liveVehicleIds.push_back(vehicle->motionObjectId);
    }

    for (Vehicle* vehicle : vehicles) {
        if (prepareDiligentVehicleAsset(*vehicle, frameRenderer)) {
            ++summary.readyAssets;
        } else {
            ++summary.pendingAssets;
        }
    }
    summary.evictedAssets = frameRenderer.retainVehicleAssets(liveVehicleIds);
    return summary;
}

std::size_t drawDiligentVehiclePass(
    Vehicle& vehicle, openbus::rendering::DiligentFrameRenderer& frameRenderer,
    Diligent::ITextureView& fallbackTexture, const openbus::rendering::Matrix4& modelViewProjection,
    const openbus::rendering::Matrix4& modelView,
    openbus::rendering::DiligentVehicleRenderPass pass,
    std::span<const openbus::rendering::DiligentVehicleGroupTransform> groupTransforms,
    std::span<const openbus::rendering::DiligentVehicleMaterialOverride> materialOverrides,
    const openbus::rendering::DiligentLightingParameters& lighting) {
    const openbus::rendering::DiligentVehicleAsset* asset =
        frameRenderer.vehicleAsset(vehicle.motionObjectId);
    if (asset == nullptr || !frameRenderer.vehicleAssetCurrent(vehicle.motionObjectId,
                                                               vehicle.diligentAssetGeneration)) {
        throw std::logic_error("Diligent vehicle pass requires a prepared current asset");
    }
    return frameRenderer.drawVehiclePass(*asset, fallbackTexture, modelViewProjection, modelView,
                                         pass, lighting, nullptr, materialOverrides, nullptr,
                                         groupTransforms);
}

bool drawDiligentVehicle(Vehicle& vehicle, openbus::rendering::DiligentFrameRenderer& frameRenderer,
                         Diligent::ITextureView& fallbackTexture,
                         const openbus::rendering::Matrix4& modelViewProjection,
                         const openbus::rendering::Matrix4& modelView,
                         openbus::rendering::ViewpointContext context,
                         const openbus::rendering::DiligentLightingParameters& lighting,
                         bool geometryHistoryValid) {
    const openbus::rendering::DiligentVehicleAsset* asset =
        frameRenderer.vehicleAsset(vehicle.motionObjectId);
    if (asset == nullptr || !frameRenderer.vehicleAssetCurrent(vehicle.motionObjectId,
                                                               vehicle.diligentAssetGeneration)) {
        throw std::logic_error("Diligent vehicle draw requires a prepared current asset");
    }

    DiligentVehicleMaterialOverrideSet materialOverrides;
    if (!frameRenderer.buildVehicleMaterialOverrides(vehicle, modelView, materialOverrides)) {
        return false;
    }
    std::vector<openbus::rendering::DiligentVehicleGroupTransform> groupTransforms;
    prepareDiligentVehicleGroupTransforms(vehicle, frameRenderer, modelView, groupTransforms,
                                          context, geometryHistoryValid);
    (void)drawDiligentVehiclePass(vehicle, frameRenderer, fallbackTexture, modelViewProjection,
                                  modelView, openbus::rendering::DiligentVehicleRenderPass::All,
                                  groupTransforms, materialOverrides.overrides, lighting);
    return true;
}

bool drawDiligentVehicleInView(Vehicle& vehicle,
                               openbus::rendering::DiligentFrameRenderer& frameRenderer,
                               openbus::rendering::DiligentTexturedMeshRenderer& meshRenderer,
                               Diligent::ITextureView& fallbackTexture,
                               const openbus::rendering::Matrix4& modelView,
                               const openbus::rendering::Matrix4& projection,
                               openbus::rendering::ViewpointContext context,
                               const openbus::rendering::DiligentLightingParameters& lighting) {
    using namespace openbus::rendering;
    const DiligentVehicleAsset* asset = frameRenderer.vehicleAsset(vehicle.motionObjectId);
    if (asset == nullptr || !frameRenderer.vehicleAssetCurrent(vehicle.motionObjectId,
                                                               vehicle.diligentAssetGeneration)) {
        throw std::logic_error("Diligent auxiliary vehicle draw requires a prepared current asset");
    }

    const auto matrixIsFinite = [](const Matrix4& matrix) {
        return std::all_of(matrix.begin(), matrix.end(),
                           [](double value) { return std::isfinite(value); });
    };
    if (!matrixIsFinite(modelView) || !matrixIsFinite(projection)) {
        throw std::invalid_argument("Diligent auxiliary vehicle view matrices must be finite");
    }
    const Matrix4 viewProjection = multiplyMatrix4(projection, modelView);
    std::vector<DiligentVehicleViewGroupInput> viewGroups;
    viewGroups.reserve(vehicle.displayLists.size());
    for (std::size_t groupIndex = 0; groupIndex < vehicle.displayLists.size(); ++groupIndex) {
        const Vehicle::DisplayPart& part = vehicle.displayLists[groupIndex];
        viewGroups.push_back(
            {groupIndex, vehicle.animationTransformForPart(part), part.center, part.radius,
             part.viewpoint, part.lodIndex,
             part.visibleVariable.empty() || vehicle.variables.get(part.visibleVariable) ==
                                                 static_cast<double>(part.visibleValue)});
    }

    std::vector<DiligentVehicleGroupTransform> groupTransforms =
        buildDiligentVehicleViewGroupTransforms(viewGroups, modelView, projection, context,
                                                vehicle.activeLod);
    DiligentVehicleMaterialOverrideSet materialOverrides;
    if (!frameRenderer.buildVehicleMaterialOverrides(vehicle, modelView, materialOverrides)) {
        return false;
    }
    asset->draw(meshRenderer, frameRenderer.immediateContext(), fallbackTexture, viewProjection,
                modelView, lighting, nullptr, materialOverrides.overrides, nullptr,
                groupTransforms);
    return true;
}

std::size_t drawDiligentVehicleShadowLayer(
    openbus::rendering::DiligentFrameRenderer& frameRenderer,
    const openbus::rendering::DiligentVehicleAsset& asset, Diligent::Uint32 lightIndex,
    const openbus::rendering::Matrix4& lightClipFromView,
    const openbus::rendering::Matrix4& modelView,
    std::span<const openbus::rendering::DiligentVehicleGroupTransform> groupTransforms,
    std::span<const openbus::rendering::DiligentVehicleMaterialOverride> materialOverrides) {
    const std::vector<openbus::rendering::DiligentShadowCaster> casters =
        asset.shadowCasters(lightClipFromView, modelView, groupTransforms, materialOverrides);
    return frameRenderer.drawOpaqueSpotlightShadowLayer(lightIndex, casters);
}

bool drawDiligentVehicleShadowLayer(Vehicle& vehicle,
                                    openbus::rendering::DiligentFrameRenderer& frameRenderer,
                                    Diligent::Uint32 lightIndex,
                                    const openbus::rendering::Matrix4& lightClipFromView,
                                    const openbus::rendering::Matrix4& modelView,
                                    openbus::rendering::ViewpointContext context,
                                    bool geometryHistoryValid) {
    const openbus::rendering::DiligentVehicleAsset* asset =
        frameRenderer.vehicleAsset(vehicle.motionObjectId);
    if (asset == nullptr || !frameRenderer.vehicleAssetCurrent(vehicle.motionObjectId,
                                                               vehicle.diligentAssetGeneration)) {
        throw std::logic_error("Diligent vehicle shadow pass requires a prepared current asset");
    }

    DiligentVehicleMaterialOverrideSet materialOverrides;
    if (!frameRenderer.buildVehicleMaterialOverrides(vehicle, modelView, materialOverrides)) {
        return false;
    }
    std::vector<openbus::rendering::DiligentVehicleGroupTransform> groupTransforms;
    prepareDiligentVehicleGroupTransforms(vehicle, frameRenderer, modelView, groupTransforms,
                                          context, geometryHistoryValid);
    (void)drawDiligentVehicleShadowLayer(frameRenderer, *asset, lightIndex, lightClipFromView,
                                         modelView, groupTransforms, materialOverrides.overrides);
    return true;
}

void prepareDiligentVehicleGroupTransforms(
    const Vehicle& vehicle, const openbus::rendering::MotionDataFrame& frame,
    openbus::rendering::MotionHistory& history, const openbus::rendering::Matrix4& vehicleModelView,
    std::vector<openbus::rendering::DiligentVehicleGroupTransform>& transforms,
    bool geometryHistoryValid) {
    transforms.clear();
    transforms.reserve(vehicle.displayLists.size());
    for (std::size_t groupIndex = 0; groupIndex < vehicle.displayLists.size(); ++groupIndex) {
        const Vehicle::DisplayPart& part = vehicle.displayLists[groupIndex];
        const openbus::rendering::Matrix4 modelView = openbus::rendering::multiplyMatrix4(
            vehicleModelView, vehicle.animationTransformForPart(part));
        transforms.push_back(openbus::rendering::DiligentVehicleGroupTransform::fromMotionHistory(
            groupIndex, frame, history, {vehicle.motionObjectId, groupIndex}, modelView,
            geometryHistoryValid, maximumVehicleMotionDeltaMeters(frame)));
    }
}

std::vector<openbus::rendering::DiligentVehicleGroupTransform>
prepareDiligentVehicleGroupTransforms(const Vehicle& vehicle,
                                      const openbus::rendering::MotionDataFrame& frame,
                                      openbus::rendering::MotionHistory& history,
                                      const openbus::rendering::Matrix4& vehicleModelView,
                                      bool geometryHistoryValid) {
    std::vector<openbus::rendering::DiligentVehicleGroupTransform> transforms;
    prepareDiligentVehicleGroupTransforms(vehicle, frame, history, vehicleModelView, transforms,
                                          geometryHistoryValid);
    return transforms;
}

void prepareDiligentVehicleGroupTransforms(
    const Vehicle& vehicle, openbus::rendering::DiligentFrameRenderer& frameRenderer,
    const openbus::rendering::Matrix4& vehicleModelView,
    std::vector<openbus::rendering::DiligentVehicleGroupTransform>& transforms,
    openbus::rendering::ViewpointContext context, bool geometryHistoryValid) {
    transforms.clear();
    transforms.reserve(vehicle.displayLists.size());
    for (std::size_t groupIndex = 0; groupIndex < vehicle.displayLists.size(); ++groupIndex) {
        const Vehicle::DisplayPart& part = vehicle.displayLists[groupIndex];
        const openbus::rendering::Matrix4 modelView = openbus::rendering::multiplyMatrix4(
            vehicleModelView, vehicle.animationTransformForPart(part));
        const bool variableAndViewVisible =
            (part.visibleVariable.empty() || vehicle.variables.get(part.visibleVariable) ==
                                                 static_cast<double>(part.visibleValue)) &&
            openbus::rendering::viewpointMatches(part.viewpoint, context) &&
            (vehicle.activeLod < 0 || part.lodIndex < 0 || part.lodIndex == vehicle.activeLod);
        const bool visible = variableAndViewVisible && frameRenderer.isSphereVisibleInView(
                                                           modelView, part.center, part.radius);
        transforms.push_back(frameRenderer.makeVehicleGroupTransform(
            groupIndex, vehicle.motionObjectId, modelView, geometryHistoryValid, visible,
            maximumVehicleMotionDeltaMeters(frameRenderer.frameData())));
    }
}

} // namespace openbus::app::vehicle
