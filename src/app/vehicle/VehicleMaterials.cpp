// Material state updates: CTC textures, material changes, interior lights.

#include "app/vehicle/Vehicle.h"
#include "app/vehicle/VehicleMaterialSelection.h"

namespace {

template <typename IncludeLight>
void populateInteriorLightMaterial(openbus::rendering::ModelMaterial& material,
                                   const std::vector<ModelInteriorLight>& lights,
                                   const std::vector<Vehicle::InteriorLightController>& controllers,
                                   const Variables& variables, const Matrix4& rootModelView,
                                   IncludeLight includeLight) {
    material.interiorLightCount = 0;
    material.interiorLightViewPositions = {};
    material.interiorLightColors = {};
    material.interiorLightStrengths = {};
    for (std::size_t index = 0; index < lights.size(); ++index) {
        if (!includeLight(index) || index >= controllers.size() ||
            material.interiorLightCount >=
                static_cast<int>(openbus::rendering::MAX_INTERIOR_LIGHTS)) {
            continue;
        }
        const ModelInteriorLight& light = lights[index];
        const Vehicle::InteriorLightController& controller = controllers[index];
        const double brightness = std::max(
            0.0, controller.isNumeric ? controller.numericValue : variables.get(light.controller));
        const double strength = brightness * std::max(0.0, light.intensity);
        if (strength <= 0.0) {
            continue;
        }
        const int outputIndex = material.interiorLightCount++;
        for (std::size_t channel = 0; channel < 3; ++channel) {
            material.interiorLightColors[outputIndex][channel] =
                static_cast<float>(std::clamp(light.color[channel] / 255.0, 0.0, 1.0));
        }
        const std::array<double, 3> viewPosition =
            openbus::rendering::interiorLightPositionInViewSpace(rootModelView, light.position);
        material.interiorLightViewPositions[outputIndex] = {static_cast<float>(viewPosition[0]),
                                                            static_cast<float>(viewPosition[1]),
                                                            static_cast<float>(viewPosition[2])};
        material.interiorLightStrengths[outputIndex] = static_cast<float>(strength);
    }
    material.useInteriorLight = material.interiorLightCount > 0;
}

} // namespace

std::string Vehicle::resolveCtcTextureName(const std::string& textureName) const {
    return openbus::app::vehicle::resolveCtcTextureName(textureName, ctcTextureReplacements);
}

void Vehicle::updateMaterialChange(Batch& batch, bool updateGlBuffer) {
    if (batch.materialSelectionGeneration == materialSelectionGeneration) {
        return;
    }
    if (batch.materialSelectionGeneration != 0 && batch.textureChanges.empty()) {
        batch.materialSelectionGeneration = materialSelectionGeneration;
        return;
    }
    TraceScope phase("texture", "updateMaterialChange");
    const MaterialState& selected = selectModelMaterial(
        batch.baseMaterial, [this](const std::string& name) { return variables.get(name); });
    const bool auxiliaryTextureChanged = batch.lightmap.name != selected.lightmapTextureName ||
                                         batch.nightmap.name != selected.nightmapTextureName ||
                                         batch.transmap.name != selected.transmapTextureName ||
                                         batch.bumpmap.name != selected.bumpmapTextureName;
    const bool environmentTextureChanged =
        batch.environmentTextureName != selected.environmentTextureName;
    const bool freeTextureChanged = batch.freeTextureName != selected.freeTextureName ||
                                    batch.freeTextureVariable != selected.freeTextureVariable ||
                                    batch.scriptTextureIndex != selected.scriptTextureIndex ||
                                    batch.textTextureIndex != selected.textTextureIndex;
    batch.selectedMaterialItem = &selected != &batch.baseMaterial;
    const auto auxiliary = [](AuxiliaryTexture& texture, const std::string& name) {
        if (texture.name != name) {
            texture = {};
            texture.name = name;
        }
    };
    auxiliary(batch.lightmap, selected.lightmapTextureName);
    auxiliary(batch.nightmap, selected.nightmapTextureName);
    auxiliary(batch.transmap, selected.transmapTextureName);
    auxiliary(batch.bumpmap, selected.bumpmapTextureName);
    batch.bumpmapStrength = selected.bumpmapStrength;
    if (batch.environmentTextureName != selected.environmentTextureName) {
        batch.environmentTexture = 0;
        batch.environmentTextureCacheEntry.reset();
        batch.environmentLoadAttempted = false;
    }
    batch.environmentTextureName = selected.environmentTextureName;
    batch.environmentStrength = selected.environmentStrength;
    batch.lightmapStrengthVariable = selected.lightmapStrengthVariable;
    if (batch.freeTextureName != selected.freeTextureName ||
        batch.freeTextureVariable != selected.freeTextureVariable ||
        batch.scriptTextureIndex != selected.scriptTextureIndex ||
        batch.textTextureIndex != selected.textTextureIndex) {
        // Text, script and file textures can reuse the same slot number,
        // but they are different sources. Never retain a previous binding.
        batch.freeTexture = 0;
        batch.freeTextureIndex = -1;
        batch.freeTextureRevision = 0;
        batch.freeTextureAsset = {};
    }
    batch.freeTextureName = selected.freeTextureName;
    batch.freeTextureVariable = selected.freeTextureVariable;
    batch.scriptTextureIndex = selected.scriptTextureIndex;
    batch.textTextureIndex = selected.textTextureIndex;
    batch.texcoordTransXVariable = selected.texcoordTransXVariable;
    batch.texcoordTransYVariable = selected.texcoordTransYVariable;
    batch.alphaMode = selected.alphaMode;
    batch.noZwrite = selected.noZwrite;
    batch.noZcheck = selected.noZcheck;
    batch.alphaScaleVariable = selected.alphaScaleVariable;
    batch.textureAddressS = selected.textureAddressS;
    batch.textureAddressT = selected.textureAddressT;
    batch.textureWrapS = textureAddressModeToGl(selected.textureAddressS);
    batch.textureWrapT = textureAddressModeToGl(selected.textureAddressT);

    // matl_change's index identifies a texture occurrence in the mesh,
    // not a DDS array layer or a replacement texture.
    const std::filesystem::path texturePath = batch.baseTexturePath;
    const std::string textureName = batch.baseTextureName;
    const int requestedLayer = batch.baseTextureLayer;
    const int layer = openbus::app::vehicle::resolveTextureArrayLayer(
        batch.textureArray, requestedLayer, batch.textureArrayLayers);
    const bool sameRequestedTexture =
        textureName == batch.textureName &&
        (texturePath == batch.texturePath || batch.textureLoadAttempted);
    if (sameRequestedTexture && layer == batch.textureLayer) {
        if (auxiliaryTextureChanged || environmentTextureChanged || freeTextureChanged) {
            invalidateDiligentAsset();
        }
        batch.materialSelectionGeneration = materialSelectionGeneration;
        return;
    }
    invalidateDiligentAsset();
    batch.textureName = textureName;
    batch.textureCacheEntry.reset();
    batch.textureRequest.reset();
    batch.texture = 0;
    batch.textured = false;
    batch.textureLoadAttempted = false;
    batch.textureLoadStarted = false;
    if (layer != batch.textureLayer) {
        batch.textureLayer = layer;
        for (auto& cachedSort : batch.transparentSortCache) {
            cachedSort.valid = false;
        }
        for (Vertex& vertex : batch.vertices) {
            vertex.layer = static_cast<float>(layer);
        }
        if (updateGlBuffer && !batch.vertices.empty()) {
            pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
            pglBufferData(GL_ARRAY_BUFFER,
                          static_cast<std::ptrdiff_t>(batch.vertices.size() * sizeof(Vertex)),
                          batch.vertices.data(), GL_STATIC_DRAW);
            pglBindBuffer(GL_ARRAY_BUFFER, 0);
        }
    }
    batch.materialSelectionGeneration = materialSelectionGeneration;
}

float Vehicle::alphaScale(const Batch& batch) const {
    if (batch.alphaScaleVariable.empty()) {
        return 1.0F;
    }
    return std::clamp(variables.get(batch.alphaScaleVariable), 0.0F, 1.0F);
}

void Vehicle::applyInteriorLightMaterial(const Batch& batch,
                                         openbus::rendering::ModelMaterial& material) const {
    populateInteriorLightMaterial(
        material, interiorLights, interiorLightControllers, variables, interiorLightRootModelView,
        [&batch](std::size_t index) {
            return std::find(batch.interiorLightIndexes.begin(), batch.interiorLightIndexes.end(),
                             static_cast<int>(index)) != batch.interiorLightIndexes.end();
        });
}

void Vehicle::applyPassengerInteriorLights(openbus::rendering::ModelMaterial& material) const {
    populateInteriorLightMaterial(material, interiorLights, interiorLightControllers, variables,
                                  interiorLightRootModelView, [](std::size_t) { return true; });
}
