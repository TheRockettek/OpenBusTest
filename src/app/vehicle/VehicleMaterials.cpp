// Material state updates: CTC textures, material changes, interior lights.

#include "app/vehicle/Vehicle.h"

std::string Vehicle::resolveCtcTextureName(const std::string& textureName) const {
    const auto found = ctcTextureReplacements.find(lower(textureName));
    return found == ctcTextureReplacements.end() ? textureName : found->second;
}

void Vehicle::updateMaterialChange(Batch& batch) {
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
    batch.textureWrapS = textureAddressModeToGl(selected.textureAddressS);
    batch.textureWrapT = textureAddressModeToGl(selected.textureAddressT);

    // matl_change's index identifies a texture occurrence in the mesh,
    // not a DDS array layer or a replacement texture.
    const std::filesystem::path texturePath = batch.baseTexturePath;
    const std::string textureName = batch.baseTextureName;
    const int requestedLayer = batch.baseTextureLayer;
    const int layer =
        batch.textureArray
            ? std::clamp(requestedLayer, 0, static_cast<int>(batch.textureArrayLayers) - 1)
            : 0;
    const bool sameRequestedTexture =
        textureName == batch.textureName &&
        (texturePath == batch.texturePath || batch.textureLoadAttempted);
    if (sameRequestedTexture && layer == batch.textureLayer) {
        batch.materialSelectionGeneration = materialSelectionGeneration;
        return;
    }
    batch.textureName = textureName;
    batch.textureCacheEntry.reset();
    batch.textureRequest.reset();
    batch.texture = 0;
    batch.textured = false;
    batch.textureLoadAttempted = false;
    batch.textureLoadStarted = false;
    if (layer != batch.textureLayer) {
        batch.textureLayer = layer;
        for (Vertex& vertex : batch.vertices) {
            vertex.layer = static_cast<float>(layer);
        }
        if (!batch.vertices.empty()) {
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
    material.interiorLightCount = 0;
    material.interiorLightViewPositions = {};
    material.interiorLightColors = {};
    material.interiorLightStrengths = {};
    for (const int lightIndex : batch.interiorLightIndexes) {
        if (lightIndex < 0 || static_cast<std::size_t>(lightIndex) >= interiorLights.size()) {
            continue;
        }
        const std::size_t index = static_cast<std::size_t>(lightIndex);
        const ModelInteriorLight& light = interiorLights[index];
        const InteriorLightController& controller = interiorLightControllers[index];
        const double brightness = std::max(
            0.0, controller.isNumeric ? controller.numericValue : variables.get(light.controller));
        const double strength = brightness * std::max(0.0, light.intensity);
        if (strength <= 0.0 || material.interiorLightCount >=
                                   static_cast<int>(openbus::rendering::MAX_INTERIOR_LIGHTS)) {
            continue;
        }
        const int outputIndex = material.interiorLightCount++;
        for (std::size_t channel = 0; channel < 3; ++channel) {
            material.interiorLightColors[outputIndex][channel] =
                static_cast<float>(std::clamp(light.color[channel] / 255.0, 0.0, 1.0));
        }
        // Light locations are model-root positions, independent of the
        // receiving mesh's animation. Transform to view space once from
        // the vehicle root so the shader can compare them to view-space
        // fragment positions without inheriting a door/panel animation.
        const std::array<double, 3> viewPosition =
            openbus::rendering::interiorLightPositionInViewSpace(interiorLightRootModelView,
                                                                 light.position);
        material.interiorLightViewPositions[outputIndex] = {static_cast<float>(viewPosition[0]),
                                                            static_cast<float>(viewPosition[1]),
                                                            static_cast<float>(viewPosition[2])};
        material.interiorLightStrengths[outputIndex] = static_cast<float>(strength);
    }
    material.useInteriorLight = material.interiorLightCount > 0;
}
