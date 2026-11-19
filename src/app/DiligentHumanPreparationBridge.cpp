#include "app/DiligentHumanBridge.h"

#include "app/DiligentFrameRenderer.h"
#include "core/AssetPath.h"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace openbus::rendering {
namespace {

DiligentAlphaMode toDiligentAlphaMode(int alphaMode) {
    switch (alphaMode) {
    case 0:
        return DiligentAlphaMode::Opaque;
    case 1:
        return DiligentAlphaMode::Cutout;
    case 2:
        return DiligentAlphaMode::Blend;
    case 3:
        return DiligentAlphaMode::NearOpaque;
    default:
        throw std::invalid_argument("Human material has an unsupported alpha mode");
    }
}

std::unordered_map<int, DiligentMaterialState>
makeDiligentHumanMaterials(const HumanTrafficRenderSource& source) {
    std::unordered_map<int, DiligentMaterialState> materials;
    materials.reserve(source.materials.size());
    for (const auto& [materialIndex, sourceMaterial] : source.materials) {
        DiligentMaterialState material;
        for (std::size_t component = 0; component < sourceMaterial.color.size(); ++component) {
            material.baseColor[component] = static_cast<float>(sourceMaterial.color[component]);
            material.specularColor[component] = sourceMaterial.specularColor[component];
            material.emissionColor[component] = sourceMaterial.emissionColor[component];
        }
        material.baseColor[3] = static_cast<float>(sourceMaterial.alpha);
        material.alphaMode = toDiligentAlphaMode(sourceMaterial.alphaMode);
        material.specularPower = sourceMaterial.specularPower;
        material.addressU = sourceMaterial.textureAddressS;
        material.addressV = sourceMaterial.textureAddressT;
        material.depthMode =
            sourceMaterial.noZcheck ? DiligentDepthMode::Disabled : DiligentDepthMode::Less;
        material.depthWrite = !sourceMaterial.noZwrite;
        material.blendEnabled = sourceMaterial.noZwrite;
        material.cullBackFaces = sourceMaterial.cullBackFaces;
        materials.emplace(materialIndex, material);
    }
    return materials;
}

} // namespace

bool prepareDiligentHumanAsset(MapRenderer& mapRenderer, DiligentFrameRenderer& frameRenderer,
                               const std::filesystem::path& humanConfigPath) {
    const std::string modelKey = openbus::pathToUtf8(humanConfigPath.lexically_normal());
    std::optional<HumanTrafficRenderSource> source =
        mapRenderer.humanTrafficRenderSource(humanConfigPath);
    if (!source.has_value()) {
        return false;
    }
    return prepareDiligentHumanAsset(frameRenderer, modelKey, *source);
}

bool prepareDiligentHumanAsset(DiligentFrameRenderer& frameRenderer, std::string_view modelKey,
                               const HumanTrafficRenderSource& source) {
    if (frameRenderer.humanAssetCurrent(modelKey, source.sourceGeneration)) {
        return true;
    }
    const auto materials = makeDiligentHumanMaterials(source);
    frameRenderer.prepareHumanAsset(modelKey, source.sourceGeneration, source.geometry,
                                    source.textures, materials);
    return true;
}

} // namespace openbus::rendering
