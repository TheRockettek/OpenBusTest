#include "app/vehicle/DiligentVehicleRuntimeTextures.h"

#include "scripting/ScriptRuntime.h"

namespace openbus::app::vehicle {

DiligentVehicleRuntimeTextures::DiligentVehicleRuntimeTextures(Diligent::IRenderDevice& device)
    : device_(&device) {}

bool DiligentVehicleRuntimeTextures::synchronize(
    const Vehicle::DisplayPart& part, std::size_t displayPartIndex, ScriptRuntime& scripts,
    Diligent::IDeviceContext& immediateContext,
    std::vector<openbus::rendering::DiligentVehicleMaterialOverride>& overrides) {
    overrides.clear();
    std::vector<openbus::rendering::DiligentVehicleMaterialOverride> synchronizedOverrides;
    synchronizedOverrides.reserve(part.batches.size());

    for (std::size_t batchIndex = 0; batchIndex < part.batches.size(); ++batchIndex) {
        const Vehicle::Batch& batch = part.batches[batchIndex];
        const bool usesTextTexture = batch.textTextureIndex >= 0;
        const int textureIndex =
            usesTextTexture ? batch.textTextureIndex : batch.scriptTextureIndex;
        if (textureIndex < 0) {
            continue;
        }

        const DiligentScriptTextureKind kind =
            usesTextTexture ? DiligentScriptTextureKind::Text : DiligentScriptTextureKind::Script;
        const TextureKey key{kind, textureIndex};
        auto texture = textures_.find(key);
        if (texture == textures_.end()) {
            texture = textures_
                          .emplace(key, std::make_unique<DiligentScriptTexture>(*device_, kind,
                                                                                textureIndex))
                          .first;
        }
        DiligentScriptTexture& scriptTexture = *texture->second;
        const bool updated = scriptTexture.synchronize(scripts, immediateContext);
        if (!updated && !scriptTexture.texture().initialized()) {
            overrides.clear();
            return false;
        }

        openbus::rendering::DiligentVehicleMaterialOverride override;
        override.materialIndex = batch.baseMaterial.materialIndex;
        override.animationGroupIndex = displayPartIndex;
        override.sourceBatchIndex = batchIndex;
        if (usesTextTexture) {
            override.textMapTexture = &scriptTexture.texture();
        } else {
            override.freeMapTexture = &scriptTexture.texture();
        }
        synchronizedOverrides.push_back(std::move(override));
    }

    overrides = std::move(synchronizedOverrides);
    return true;
}

bool DiligentVehicleRuntimeTextures::synchronize(
    std::span<const Vehicle::DisplayPart> displayParts, ScriptRuntime& scripts,
    Diligent::IDeviceContext& immediateContext,
    std::vector<openbus::rendering::DiligentVehicleMaterialOverride>& overrides) {
    overrides.clear();
    std::vector<openbus::rendering::DiligentVehicleMaterialOverride> synchronizedOverrides;
    for (std::size_t displayPartIndex = 0; displayPartIndex < displayParts.size();
         ++displayPartIndex) {
        std::vector<openbus::rendering::DiligentVehicleMaterialOverride> partOverrides;
        if (!synchronize(displayParts[displayPartIndex], displayPartIndex, scripts,
                         immediateContext, partOverrides)) {
            overrides.clear();
            return false;
        }
        for (auto& override : partOverrides) {
            synchronizedOverrides.push_back(std::move(override));
        }
    }

    overrides = std::move(synchronizedOverrides);
    return true;
}

} // namespace openbus::app::vehicle
