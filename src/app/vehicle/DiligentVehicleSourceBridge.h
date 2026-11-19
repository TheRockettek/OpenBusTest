#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_NONE
#endif

#include "app/vehicle/Vehicle.h"
#include "render/diligent/DiligentVehicleAsset.h"

#include <cstddef>
#include <memory>
#include <span>

namespace openbus::app::vehicle {

struct DiligentVehicleBatchTextures {
    openbus::rendering::DiligentVehicleTextureSource base;
    std::span<const std::shared_ptr<const Image>> textureArrayLayers;
    std::shared_ptr<const gli::texture> compressedTextureArray;
    openbus::rendering::DiligentVehicleAuxiliaryTextureSources auxiliary;
    bool baseIsTextureArray = false;
};

struct DiligentVehicleBatchSourceSet {
    std::vector<std::vector<std::shared_ptr<const Image>>> textureArrayLayerStorage;
    std::vector<openbus::rendering::DiligentVehicleBatchSource> batches;
};

struct DiligentVehicleMaterialOverrideSet {
    DiligentVehicleMaterialOverrideSet() = default;
    DiligentVehicleMaterialOverrideSet(const DiligentVehicleMaterialOverrideSet&) = delete;
    DiligentVehicleMaterialOverrideSet&
    operator=(const DiligentVehicleMaterialOverrideSet&) = delete;
    DiligentVehicleMaterialOverrideSet(DiligentVehicleMaterialOverrideSet&&) noexcept = default;
    DiligentVehicleMaterialOverrideSet&
    operator=(DiligentVehicleMaterialOverrideSet&&) noexcept = default;

    std::vector<openbus::rendering::DiligentMaterialState> materials;
    std::vector<openbus::rendering::DiligentVehicleMaterialOverride> overrides;
};

// CPU geometry remains owned by Vehicle. Texture inputs must outlive construction of the
// DiligentVehicleAsset that consumes the returned source.
[[nodiscard]] openbus::rendering::DiligentVehicleBatchSource
makeDiligentVehicleBatchSource(const Vehicle::DisplayPart& part, std::size_t displayPartIndex,
                               std::size_t batchIndex,
                               const DiligentVehicleBatchTextures& textures = {});

[[nodiscard]] openbus::rendering::DiligentMaterialState
makeDiligentVehicleMaterialState(const Vehicle::Batch& batch, const Variables& variables,
                                 double alpha, double visibilityFade = 1.0);
void populateDiligentVehicleInteriorLights(
    openbus::rendering::DiligentMaterialState& material, const Vehicle::Batch& batch,
    std::span<const ModelInteriorLight> lights,
    std::span<const Vehicle::InteriorLightController> controllers, const Variables& variables,
    const openbus::rendering::Matrix4& vehicleRootModelView);

// Call after production material selection has updated the batches. Runtime texture overrides
// are merged by stable display-part and per-part batch identity.
[[nodiscard]] DiligentVehicleMaterialOverrideSet buildDiligentVehicleMaterialOverrides(
    std::span<const Vehicle::DisplayPart> displayParts, const Variables& variables,
    std::span<const openbus::rendering::DiligentVehicleMaterialOverride> runtimeTextures = {});
[[nodiscard]] DiligentVehicleMaterialOverrideSet buildDiligentVehicleMaterialOverrides(
    Vehicle& vehicle, const openbus::rendering::Matrix4& vehicleRootModelView,
    std::span<const openbus::rendering::DiligentVehicleMaterialOverride> runtimeTextures = {});

void bindDiligentVehicleReflectionTextures(
    std::span<const Vehicle::DisplayPart> displayParts,
    DiligentVehicleMaterialOverrideSet& overrides,
    std::span<Diligent::ITextureView* const> reflectionTextures);

// Returns false while any referenced production texture request is still decoding.
// The returned set owns image-array layers and must outlive DiligentVehicleAsset construction.
[[nodiscard]] bool makeDiligentVehicleBatchSources(const Vehicle::DisplayPart& part,
                                                   std::size_t displayPartIndex,
                                                   DiligentVehicleBatchSourceSet& result);
[[nodiscard]] bool
makeDiligentVehicleBatchSources(std::span<const Vehicle::DisplayPart> displayParts,
                                DiligentVehicleBatchSourceSet& result);

} // namespace openbus::app::vehicle
