#include "render/map/PreparedMapScenery.h"

#include "assets/O3DLoader.h"
#include "assets/XLoader.h"
#include "config/ModelConfigLoader.h"
#include "core/AssetPath.h"
#include "core/Utf8Path.h"
#include "map/MapSceneryGeometry.h"
#include "map/SceneryMaterialBinding.h"
#include "render/MaterialConstants.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace openbus::rendering {
namespace {

double vertexBoundingRadius(const std::vector<ModelVertex>& vertices) {
    double radiusSquared = 0.0;
    for (const ModelVertex& vertex : vertices) {
        const double x = vertex.x;
        const double y = vertex.y;
        const double z = vertex.z;
        radiusSquared = std::max(radiusSquared, x * x + y * y + z * z);
    }
    return std::sqrt(radiusSquared);
}

std::vector<ModelVertex> makeTreePreview(double height, double width) {
    // OMSI's SCO supplies a texture and height/ratio ranges; this preview uses
    // crossed cutout planes because its editor-only helper mesh is often .x.
    const float halfWidth = static_cast<float>(width * 0.5);
    const float top = static_cast<float>(height);
    constexpr float low = 0.0F;
    constexpr float high = 1.0F;
    const auto vertex = [](float x, float y, float z, float u, float v) {
        return ModelVertex{x, y, z, u, v, 0.0F, 0.0F, 0.0F, 1.0F};
    };
    const ModelVertex xLow = vertex(-halfWidth, 0.0F, low, 0.0F, 0.0F);
    const ModelVertex xHigh = vertex(halfWidth, 0.0F, low, 1.0F, 0.0F);
    const ModelVertex xTopHigh = vertex(halfWidth, 0.0F, top, 1.0F, high);
    const ModelVertex xTopLow = vertex(-halfWidth, 0.0F, top, 0.0F, high);
    const ModelVertex yLow = vertex(0.0F, -halfWidth, low, 0.0F, 0.0F);
    const ModelVertex yHigh = vertex(0.0F, halfWidth, low, 1.0F, 0.0F);
    const ModelVertex yTopHigh = vertex(0.0F, halfWidth, top, 1.0F, high);
    const ModelVertex yTopLow = vertex(0.0F, -halfWidth, top, 0.0F, high);
    return {xLow, xHigh, xTopHigh, xLow, xTopHigh, xTopLow,
            yLow, yHigh, yTopHigh, yLow, yTopHigh, yTopLow};
}

PreparedSceneryMesh prepareSceneryMesh(const std::filesystem::path& meshPath);

} // namespace

PreparedSceneryAsset prepareSceneryAsset(const std::filesystem::path& configPath,
                                         const std::filesystem::path& omsiRootPath,
                                         SharedMapTextureImages& textureImages) {
    PreparedSceneryAsset prepared;
    prepared.objectConfiguration = loadSceneryObjectFile(configPath);
    prepared.modelConfigPath = resolveSceneryObjectModelConfigPath(prepared.objectConfiguration);
    prepared.usesExternalModelConfig = !prepared.objectConfiguration.modelPath.empty();
    prepared.externalModelConfigPath = prepared.modelConfigPath;
    if (prepared.usesExternalModelConfig &&
        !std::filesystem::is_regular_file(prepared.modelConfigPath)) {
        prepared.externalModelConfigMissing = true;
        prepared.modelConfigPath = configPath;
        prepared.usesExternalModelConfig = false;
    }
    prepared.modelRoot = prepared.usesExternalModelConfig ? prepared.modelConfigPath.parent_path()
                                                          : configPath.parent_path() / "model";
    if (!prepared.objectConfiguration.onlyEditor || !prepared.objectConfiguration.trees.empty()) {
        if (prepared.objectConfiguration.trees.empty()) {
            prepared.modelConfiguration =
                loadModelConfig(prepared.modelConfigPath, prepared.modelRoot,
                                ModelConfigKind::SceneryObject, prepared.variables);
            prepared.hasModelConfiguration = true;
            for (const ModelPart& part : prepared.modelConfiguration.parts) {
                if (part.objPath.empty()) {
                    continue;
                }
                std::string extension = part.objPath.extension().string();
                std::transform(
                    extension.begin(), extension.end(), extension.begin(),
                    [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
                if (extension == ".o3d" || extension == ".x") {
                    const std::string meshKey =
                        openbus::pathToUtf8(part.objPath.lexically_normal());
                    if (!prepared.meshes.contains(meshKey)) {
                        PreparedSceneryMesh mesh;
                        try {
                            mesh = prepareSceneryMesh(part.objPath);
                        } catch (...) {
                            mesh.preparationError = std::current_exception();
                        }
                        prepared.meshes.emplace(meshKey, std::move(mesh));
                    }
                }
            }
        }
    }
    const auto decodeTexture = [&](const std::string& textureName) {
        if (textureName.empty()) {
            return;
        }
        ++prepared.textureLookups;
        if (prepared.resolvedTexturePaths.contains(textureName)) {
            return;
        }
        ++prepared.textureResolutions;
        const std::filesystem::path texturePath = openbus::assets::findSceneryTexture(
            configPath.parent_path(), prepared.modelRoot, omsiRootPath, textureName);
        if (texturePath.empty()) {
            prepared.resolvedTexturePaths.try_emplace(textureName, texturePath);
            return;
        }
        const std::string key = openbus::pathToUtf8(texturePath.lexically_normal());
        if (prepared.decodedTextures.contains(key)) {
            prepared.resolvedTexturePaths.try_emplace(textureName, texturePath);
            return;
        }
        auto image = textureImages.getOrDecode(key, [&texturePath] {
            auto decoded = std::make_shared<Image>();
            if (!TextureLoader::readImage(texturePath, *decoded)) {
                throw std::runtime_error("Could not decode scenery texture: " +
                                         openbus::pathToUtf8(texturePath));
            }
            return decoded;
        });
        prepared.decodedTextures.emplace(key, std::move(image));
        prepared.resolvedTexturePaths.try_emplace(textureName, texturePath);
    };
    const auto decodeOptionalTexture = [&](const std::string& textureName) {
        if (prepared.auxiliaryTexturePreparationErrors.contains(textureName)) {
            return;
        }
        try {
            decodeTexture(textureName);
        } catch (const std::runtime_error& error) {
            prepared.auxiliaryTexturePreparationErrors.try_emplace(textureName, error.what());
        }
    };
    for (const SceneryTreeDefinition& tree : prepared.objectConfiguration.trees) {
        decodeTexture(tree.texturePath);
    }
    for (const auto& [meshKey, mesh] : prepared.meshes) {
        static_cast<void>(meshKey);
        if (!mesh.parsed) {
            continue;
        }
        for (const auto& [materialName, material] : mesh.parsed->materials) {
            static_cast<void>(materialName);
            decodeTexture(material.textureName);
        }
    }
    for (const ModelPart& part : prepared.modelConfiguration.parts) {
        for (const ModelMaterialState& state : part.materialStatesInOrder) {
            decodeTexture(state.textureName);
            decodeTexture(state.transmapTextureName);
            decodeOptionalTexture(state.lightmapTextureName);
            decodeOptionalTexture(state.nightmapTextureName);
            for (const auto& change : state.textureChanges) {
                for (const auto& item : change.items) {
                    if (item) {
                        decodeOptionalTexture(item->lightmapTextureName);
                        decodeOptionalTexture(item->nightmapTextureName);
                    }
                }
            }
            decodeOptionalTexture(state.bumpmapTextureName);
            if (state.environmentStrength > 0.0) {
                decodeOptionalTexture(state.environmentTextureName);
            }
            decodeTexture(state.freeTextureName);
        }
    }
    const auto resolvedTexture = [&prepared](const std::string& textureName) {
        if (textureName.empty()) {
            return std::filesystem::path{};
        }
        const auto texture = prepared.resolvedTexturePaths.find(textureName);
        if (texture == prepared.resolvedTexturePaths.end()) {
            throw std::logic_error("Scenery texture path was not prepared: " + textureName);
        }
        return texture->second;
    };
    const auto preparedAuxTexture = [&](const std::string& textureName) {
        PreparedSceneryAuxTexture result;
        result.name = textureName;
        const auto error = prepared.auxiliaryTexturePreparationErrors.find(textureName);
        if (error != prepared.auxiliaryTexturePreparationErrors.end()) {
            result.preparationError = error->second;
        } else {
            result.path = resolvedTexture(textureName);
        }
        return result;
    };
    for (const SceneryTreeDefinition& tree : prepared.objectConfiguration.trees) {
        const std::filesystem::path texturePath = resolvedTexture(tree.texturePath);
        if (texturePath.empty()) {
            PreparedSceneryTreeBatch batch;
            batch.textureName = tree.texturePath;
            prepared.treeBatches.push_back(std::move(batch));
            continue;
        }
        const double height = (tree.minimumHeight + tree.maximumHeight) * 0.5;
        const double ratio = (tree.minimumRatio + tree.maximumRatio) * 0.5;
        PreparedSceneryTreeBatch batch;
        batch.vertices = makeTreePreview(height, height * 0.55 * ratio);
        batch.textureName = tree.texturePath;
        batch.texturePath = texturePath;
        batch.seasonTexturePaths = textureImages.seasonalPaths(texturePath);
        batch.batch.vertexCount = batch.vertices.size();
        batch.batch.boundingRadius = vertexBoundingRadius(batch.vertices);
        batch.batch.renderType = prepared.objectConfiguration.renderType;
        batch.batch.material.textured = true;
        batch.batch.material.flipTextureY = true;
        batch.batch.alphaMode = 1;
        prepared.treeBatches.push_back(std::move(batch));
    }
    for (const ModelPart& part : prepared.modelConfiguration.parts) {
        if (part.objPath.empty()) {
            continue;
        }
        std::string meshExtension = part.objPath.extension().string();
        std::transform(meshExtension.begin(), meshExtension.end(), meshExtension.begin(),
                       [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
        if (meshExtension != ".o3d" && meshExtension != ".x") {
            continue;
        }
        const std::string meshKey = openbus::pathToUtf8(part.objPath.lexically_normal());
        const auto preparedMesh = prepared.meshes.find(meshKey);
        if (preparedMesh == prepared.meshes.end() || !preparedMesh->second.parsed ||
            preparedMesh->second.preparationError) {
            continue;
        }
        const PreparedSceneryMesh& mesh = preparedMesh->second;
        const ParsedObj& parsed = *mesh.parsed;
        const auto statesByMaterial = openbus::map::resolveSceneryMaterialStates(parsed, part);
        for (const auto& [materialIndex, vertices] : mesh.verticesByMaterial) {
            if (vertices.empty()) {
                continue;
            }
            const auto sourceMaterial =
                parsed.materials.find("matl_" + std::to_string(materialIndex));
            const auto stateFound = statesByMaterial.find(materialIndex);
            const ModelMaterialState* state =
                stateFound == statesByMaterial.end() ? nullptr : stateFound->second;
            std::string textureName = sourceMaterial == parsed.materials.end()
                                          ? std::string{}
                                          : sourceMaterial->second.textureName;
            std::array<double, 3> color = sourceMaterial == parsed.materials.end()
                                              ? std::array<double, 3>{1.0, 1.0, 1.0}
                                              : sourceMaterial->second.color;
            if (state != nullptr && !state->textureName.empty()) {
                textureName = state->textureName;
                const std::string stateTexture =
                    openbus::pathToUtf8(std::filesystem::path(state->textureName).filename());
                const auto materialForTexture = std::find_if(
                    parsed.materials.begin(), parsed.materials.end(), [&](const auto& entry) {
                        return std::filesystem::path(entry.second.textureName).filename() ==
                               stateTexture;
                    });
                if (materialForTexture != parsed.materials.end()) {
                    color = materialForTexture->second.color;
                }
            }
            PreparedSceneryBatch preparedBatch;
            preparedBatch.meshKey = meshKey;
            preparedBatch.materialIndex = materialIndex;
            preparedBatch.textureName = textureName;
            preparedBatch.texturePath = resolvedTexture(textureName);
            preparedBatch.seasonTexturePaths =
                textureImages.seasonalPaths(preparedBatch.texturePath);
            if (state != nullptr) {
                preparedBatch.transmapTextureName = state->transmapTextureName;
                preparedBatch.transmapTexturePath = resolvedTexture(state->transmapTextureName);
                preparedBatch.lightmap = preparedAuxTexture(state->lightmapTextureName);
                preparedBatch.nightmap = preparedAuxTexture(state->nightmapTextureName);
                for (const auto& change : state->textureChanges) {
                    for (const auto& item : change.items) {
                        if (item) {
                            if (!item->lightmapTextureName.empty()) {
                                preparedBatch.itemLightmaps.push_back(
                                    preparedAuxTexture(item->lightmapTextureName));
                            }
                            if (!item->nightmapTextureName.empty()) {
                                preparedBatch.itemNightmaps.push_back(
                                    preparedAuxTexture(item->nightmapTextureName));
                            }
                        }
                    }
                }
                preparedBatch.bumpmap = preparedAuxTexture(state->bumpmapTextureName);
                if (state->environmentStrength > 0.0) {
                    preparedBatch.environment = preparedAuxTexture(state->environmentTextureName);
                }
                preparedBatch.freeTextureName = state->freeTextureName;
                preparedBatch.freeTexturePath = resolvedTexture(state->freeTextureName);
                preparedBatch.hasMaterialState = true;
                preparedBatch.stateAlphaMode = state->alphaMode;
            }
            openbus::map::MapSceneryBatch& batch = preparedBatch.batch;
            batch.material.weatherLitDynamicTexture = true;
            batch.material.whiteAmbient = !part.isShadow;
            batch.cullBackFaces = true;
            batch.vertexCount = vertices.size();
            batch.boundingRadius = vertexBoundingRadius(vertices);
            batch.sceneryLodIndex =
                part.lodIndex < 0 ? 0U : static_cast<std::size_t>(part.lodIndex);
            if (sourceMaterial != parsed.materials.end()) {
                for (std::size_t index = 0; index < 3; ++index) {
                    batch.material.specularColor[index] =
                        static_cast<float>(sourceMaterial->second.specular[index]);
                    batch.material.emissionColor[index] =
                        static_cast<float>(sourceMaterial->second.emission[index]);
                }
                batch.material.specularPower =
                    static_cast<float>(sourceMaterial->second.specularPower);
                batch.alpha = std::clamp(sourceMaterial->second.alpha, 0.0, 1.0);
            }
            batch.color = color;
            batch.renderType =
                resolveSceneryRenderType(part, prepared.objectConfiguration.renderType);
            batch.visibilityVariable = part.visibleVariable;
            batch.visibilityValue = part.visibleValue;
            if (state != nullptr) {
                batch.scriptTextureIndex = state->scriptTextureIndex;
                batch.textTextureIndex = state->textTextureIndex;
                batch.freeTextureName = state->freeTextureName;
                batch.freeTextureVariable = state->freeTextureVariable;
                batch.texcoordTransXVariable = state->texcoordTransXVariable;
                batch.texcoordTransYVariable = state->texcoordTransYVariable;
                batch.lightmapStrengthVariable = state->lightmapStrengthVariable;
                batch.alphaScaleVariable = state->alphaScaleVariable;
                if (!preparedBatch.itemLightmaps.empty() || !preparedBatch.itemNightmaps.empty()) {
                    batch.auxiliaryMaterialSelection = *state;
                }
                batch.material.lightmapStrength =
                    state->lightmapStrengthVariable.empty() ? 1.0F : 0.0F;
                batch.material.bumpmapStrength =
                    static_cast<float>(std::clamp(state->bumpmapStrength, 0.0, 1.0));
                batch.material.environmentStrength = static_cast<float>(
                    std::clamp(state->environmentStrength * ENVIRONMENT_MAP_OPACITY, 0.0, 1.0));
                batch.textureObjectRoot = configPath.parent_path();
                batch.textureModelRoot = prepared.modelRoot;
                batch.material.addressU = state->textureAddressS;
                batch.material.addressV = state->textureAddressT;
                batch.alphaMode = state->alphaMode;
                batch.noZwrite = state->noZwrite;
                batch.noZcheck = state->noZcheck;
            }
            if (batch.alphaMode == 1 && batch.alpha == 0.0) {
                batch.alpha = 1.0;
            }
            if (batch.alphaMode == 0 && batch.alpha < 1.0 - 1.0e-6) {
                batch.alphaMode = 2;
            }
            prepared.batches.push_back(std::move(preparedBatch));
        }
    }
    return prepared;
}

namespace {

PreparedSceneryMesh prepareSceneryMesh(const std::filesystem::path& meshPath) {
    PreparedSceneryMesh prepared;
    std::string extension = meshPath.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    prepared.parsed = extension == ".x" ? XLoader::parse(meshPath) : O3DLoader::parse(meshPath);
    if (!prepared.parsed) {
        return prepared;
    }
    auto geometry = openbus::map::prepareMapSceneryGeometry(*prepared.parsed);
    prepared.verticesByMaterial = std::move(geometry.verticesByMaterial);
    prepared.invalidPositionIndices = geometry.invalidPositionIndices;
    return prepared;
}

} // namespace

} // namespace openbus::rendering
