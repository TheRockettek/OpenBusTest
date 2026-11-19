#include "app/vehicle/DiligentVehicleSceneBridge.h"

#include "app/DiligentFrameRenderer.h"
#include "app/vehicle/DiligentVehicleBridge.h"
#include "render/MatrixMath.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace openbus::app::vehicle {
namespace {

bool matrixIsFinite(const openbus::rendering::Matrix4& matrix) {
    return std::all_of(matrix.begin(), matrix.end(),
                       [](double value) { return std::isfinite(value); });
}

} // namespace

DiligentVehicleSceneDrawPreparation prepareDiligentVehicleSceneDraw(
    Vehicle& vehicle, openbus::rendering::DiligentFrameRenderer& frameRenderer,
    const openbus::rendering::Matrix4& modelView, const openbus::rendering::Matrix4& projection,
    openbus::rendering::ViewpointContext context,
    const openbus::rendering::DiligentLightingParameters& lighting, bool geometryHistoryValid) {
    if (vehicle.motionObjectId == 0) {
        throw std::invalid_argument("Diligent vehicle scene draws require a stable object ID");
    }
    if (!matrixIsFinite(modelView) || !matrixIsFinite(projection)) {
        throw std::invalid_argument("Diligent vehicle scene matrices must be finite");
    }
    vehicle.updateAnimationStates();
    if (!frameRenderer.vehicleAssetCurrent(vehicle.motionObjectId,
                                           vehicle.diligentAssetGeneration)) {
        return {DiligentVehicleSceneDrawPreparation::Status::AssetPending, std::nullopt};
    }

    DiligentVehicleSceneDrawState state;
    state.vehicleObjectId = vehicle.motionObjectId;
    state.assetGeneration = vehicle.diligentAssetGeneration;
    state.modelView = modelView;
    state.modelViewProjection = openbus::rendering::multiplyMatrix4(projection, modelView);
    state.lighting = lighting;
    if (!frameRenderer.buildVehicleMaterialOverrides(vehicle, modelView, state.materialOverrides)) {
        return {DiligentVehicleSceneDrawPreparation::Status::MaterialPending, std::nullopt};
    }
    prepareDiligentVehicleGroupTransforms(vehicle, frameRenderer, modelView, state.groupTransforms,
                                          context, geometryHistoryValid);
    return {DiligentVehicleSceneDrawPreparation::Status::Ready, std::move(state)};
}

} // namespace openbus::app::vehicle
