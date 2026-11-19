#include "render/diligent/DiligentSceneryAsset.h"

#include "core/Utf8Path.h"
#include "map/MapSceneryPlacement.h"
#include "render/diligent/DiligentTexture2D.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace openbus::rendering {
namespace {

std::array<double, 3> geometryCenter(std::span<const ModelVertex> vertices) {
    if (vertices.empty()) {
        return {};
    }
    std::array<double, 3> minimum = {vertices.front().x, vertices.front().y, vertices.front().z};
    std::array<double, 3> maximum = minimum;
    for (const ModelVertex& vertex : vertices.subspan(1)) {
        const std::array<double, 3> position = {vertex.x, vertex.y, vertex.z};
        for (std::size_t component = 0; component < position.size(); ++component) {
            minimum[component] = (std::min)(minimum[component], position[component]);
            maximum[component] = (std::max)(maximum[component], position[component]);
        }
    }
    return {(minimum[0] + maximum[0]) * 0.5, (minimum[1] + maximum[1]) * 0.5,
            (minimum[2] + maximum[2]) * 0.5};
}

} // namespace

openbus::map::SceneryMaterialRuntimeState
DiligentSceneryBatch::resolveRuntime(const Variables* variables,
                                     std::optional<double> weatherBrightness) const {
    const auto& batch = source.batch;
    const openbus::map::SceneryMaterialRuntimeConfig config{
        batch.lightmapStrengthVariable,
        batch.visibilityVariable,
        batch.texcoordTransXVariable,
        batch.texcoordTransYVariable,
        batch.alphaScaleVariable,
        batch.auxiliaryMaterialSelection ? &*batch.auxiliaryMaterialSelection : nullptr,
        batch.material.lightmapStrength,
        0.0F};
    return openbus::map::resolveSceneryMaterialRuntime(config, variables, weatherBrightness);
}

bool DiligentSceneryBatch::drawOpaque(DiligentTexturedMeshRenderer& renderer,
                                      Diligent::IDeviceContext& context,
                                      Diligent::ITextureView& fallbackWhite,
                                      const Matrix4& modelViewProjection,
                                      const DiligentSceneryOpaqueDraw& draw,
                                      const DiligentLightingParameters& lighting,
                                      const DiligentMotionParameters* motion) const {
    return drawPass(DiligentSceneryPass::Opaque, renderer, context, fallbackWhite,
                    modelViewProjection, draw, lighting, motion);
}

bool DiligentSceneryBatch::drawTransparent(DiligentTexturedMeshRenderer& renderer,
                                           Diligent::IDeviceContext& context,
                                           Diligent::ITextureView& fallbackWhite,
                                           const Matrix4& modelViewProjection,
                                           const DiligentSceneryOpaqueDraw& draw,
                                           const DiligentLightingParameters& lighting,
                                           const DiligentMotionParameters* motion) const {
    return drawPass(DiligentSceneryPass::Transparent, renderer, context, fallbackWhite,
                    modelViewProjection, draw, lighting, motion);
}

bool DiligentSceneryBatch::drawTransmapDepthPrepass(
    DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
    Diligent::ITextureView& fallbackWhite, const Matrix4& modelViewProjection,
    const DiligentSceneryOpaqueDraw& draw, const DiligentLightingParameters& lighting) const {
    return drawPass(DiligentSceneryPass::TransmapDepthPrepass, renderer, context, fallbackWhite,
                    modelViewProjection, draw, lighting, nullptr);
}

bool DiligentSceneryBatch::drawPass(
    DiligentSceneryPass pass, DiligentTexturedMeshRenderer& renderer,
    Diligent::IDeviceContext& context, Diligent::ITextureView& fallbackWhite,
    const Matrix4& modelViewProjection, const DiligentSceneryOpaqueDraw& draw,
    const DiligentLightingParameters& lighting, const DiligentMotionParameters* motion) const {
    const auto& batch = source.batch;
    if (!std::isfinite(draw.visibilityFade) || draw.visibilityFade < 0.0F ||
        draw.visibilityFade > 1.0F) {
        throw std::invalid_argument(
            "Diligent scenery visibility fade must be finite and in [0, 1]");
    }
    if (draw.visibilityFade == 0.0F) {
        return false;
    }
    if (draw.sceneryLodIndex && batch.sceneryLodIndex != *draw.sceneryLodIndex) {
        return false;
    }
    if (!batch.visibilityVariable.empty() &&
        (draw.variables != nullptr ? draw.variables->get(batch.visibilityVariable) : 0.0F) !=
            static_cast<float>(batch.visibilityValue)) {
        return false;
    }
    if (!mesh) {
        throw std::logic_error("Diligent scenery batch has no mesh");
    }
    switch (pass) {
    case DiligentSceneryPass::Opaque:
        if ((batch.alphaMode != 0 && batch.alphaMode != 1) || batch.textTextureIndex >= 0) {
            throw std::invalid_argument(
                "Diligent opaque pass received a transparent scenery or text batch");
        }
        if (draw.visibilityFade < 1.0F) {
            return false;
        }
        break;
    case DiligentSceneryPass::Transparent:
        if (batch.textTextureIndex < 0 && !batch.noZwrite && batch.alphaMode != 2 &&
            batch.alphaMode != 3 &&
            !(draw.visibilityFade < 1.0F && (batch.alphaMode == 0 || batch.alphaMode == 1))) {
            throw std::invalid_argument("Diligent transparent pass requires alpha mode 2 or 3, "
                                        "no-Z-write scenery, or text");
        }
        break;
    case DiligentSceneryPass::TransmapDepthPrepass:
        if (batch.alphaMode != 2) {
            throw std::invalid_argument(
                "Diligent transmap depth prepass requires alpha mode 2 scenery");
        }
        if (!textures.transmap || batch.noZwrite || batch.textTextureIndex >= 0) {
            return false;
        }
        break;
    }
    const bool usesTextTexture = batch.textTextureIndex >= 0;
    const bool usesDynamicTexture =
        batch.scriptTextureIndex >= 0 || !batch.freeTextureVariable.empty();
    if (usesTextTexture && draw.textTexture == nullptr) {
        throw std::logic_error("Diligent scenery text batch requires its runtime text texture");
    }
    if (usesDynamicTexture && draw.dynamicTexture == nullptr) {
        throw std::logic_error("Diligent scenery dynamic batch requires its runtime texture");
    }
    const auto runtime = resolveRuntime(draw.variables, draw.weatherBrightness);
    const auto textureView = [](const std::shared_ptr<DiligentTexture2D>& texture) {
        return texture ? &texture->shaderResourceView() : nullptr;
    };
    const auto itemTexture =
        [&](const std::string& name, const std::vector<PreparedSceneryAuxTexture>& sources,
            const std::vector<std::shared_ptr<DiligentTexture2D>>& texturesByIndex) {
            if (sources.size() != texturesByIndex.size()) {
                throw std::logic_error("Diligent scenery item texture metadata is inconsistent");
            }
            const auto found = std::find_if(sources.begin(), sources.end(),
                                            [&](const auto& entry) { return entry.name == name; });
            return found == sources.end()
                       ? static_cast<Diligent::ITextureView*>(nullptr)
                       : textureView(
                             texturesByIndex[static_cast<std::size_t>(found - sources.begin())]);
        };
    DiligentMaterialState material;
    switch (pass) {
    case DiligentSceneryPass::Opaque:
        material.alphaMode =
            batch.alphaMode == 1 ? DiligentAlphaMode::Cutout : DiligentAlphaMode::Opaque;
        break;
    case DiligentSceneryPass::Transparent:
        if (usesTextTexture || batch.alphaMode == 2 ||
            (draw.visibilityFade < 1.0F && (batch.alphaMode == 0 || batch.alphaMode == 1))) {
            material.alphaMode = DiligentAlphaMode::Blend;
        } else if (batch.alphaMode == 3) {
            material.alphaMode = DiligentAlphaMode::NearOpaqueBlend;
        } else {
            material.alphaMode =
                batch.alphaMode == 1 ? DiligentAlphaMode::Cutout : DiligentAlphaMode::Opaque;
            material.blendEnabled = batch.noZwrite;
        }
        break;
    case DiligentSceneryPass::TransmapDepthPrepass:
        material.alphaMode = DiligentAlphaMode::NearOpaque;
        material.colorWriteEnabled = false;
        break;
    }
    material.baseColor = {static_cast<float>(batch.color[0]), static_cast<float>(batch.color[1]),
                          static_cast<float>(batch.color[2]),
                          static_cast<float>(batch.alpha * runtime.alphaScale)};
    material.textureOffset = {runtime.texcoordOffsetX.value_or(0.0F),
                              runtime.texcoordOffsetY.value_or(0.0F)};
    material.specularColor = batch.material.specularColor;
    material.emissionColor = batch.material.emissionColor;
    for (float& value : material.emissionColor) {
        value = std::clamp(value, 0.0F, 1.0F);
    }
    material.specularPower = std::max(batch.material.specularPower, 1.0F);
    material.normalMapStrength = batch.material.bumpmapStrength;
    material.lightMap = runtime.selectedItem
                            ? itemTexture(runtime.selectedItem->lightmapTextureName,
                                          source.itemLightmaps, textures.itemLightmaps)
                            : textureView(textures.lightmap);
    material.nightMap = runtime.selectedItem
                            ? itemTexture(runtime.selectedItem->nightmapTextureName,
                                          source.itemNightmaps, textures.itemNightmaps)
                            : textureView(textures.nightmap);
    material.lightMapStrength = runtime.lightmapStrength;
    material.nightMapStrength = runtime.nightmapStrength;
    material.transMap = textureView(textures.transmap);
    material.environmentMap = textureView(textures.environment);
    material.environmentStrength = batch.material.environmentStrength;
    material.environmentBrightness = static_cast<float>(draw.weatherBrightness.value_or(1.0));
    material.freeMap = usesDynamicTexture ? draw.dynamicTexture : textureView(textures.freeTexture);
    material.textMap = usesTextTexture ? draw.textTexture : nullptr;
    material.weatherLitDynamicTexture = batch.material.weatherLitDynamicTexture;
    material.dynamicTextureBrightness = material.environmentBrightness;
    material.whiteAmbient = batch.material.whiteAmbient;
    material.cullBackFaces = batch.cullBackFaces && !draw.reverseWinding;
    material.useVertexColor = false;
    material.flipTextureY = batch.material.flipTextureY;
    material.addressU = batch.material.addressU;
    material.addressV = batch.material.addressV;
    material.visibilityFade = draw.visibilityFade;
    material.depthWrite = pass == DiligentSceneryPass::TransmapDepthPrepass ||
                          (pass == DiligentSceneryPass::Opaque && !batch.noZwrite);
    material.blendEnabled = material.blendEnabled || batch.noZwrite;
    if (pass == DiligentSceneryPass::Transparent) {
        material.depthMode = DiligentDepthMode::LessEqual;
        material.depthWrite = usesTextTexture || (batch.alphaMode == 2 && !batch.noZwrite) ||
                              (draw.visibilityFade < 1.0F && !batch.noZwrite &&
                               (batch.alphaMode == 0 || batch.alphaMode == 1));
    } else if (pass == DiligentSceneryPass::Opaque && batch.noZcheck && !batch.noZwrite &&
               batch.alphaMode == 0) {
        material.depthMode = DiligentDepthMode::Disabled;
    }
    Diligent::ITextureView* base =
        draw.seasonalTexture ? draw.seasonalTexture : textureView(textures.base);
    if (base == nullptr) {
        base = &fallbackWhite;
    }
    auto* normal = textureView(textures.bumpmap);
    if (motion != nullptr) {
        renderer.drawMotion(context, *mesh, *base, modelViewProjection, *motion, material, lighting,
                            normal);
    } else {
        renderer.draw(context, *mesh, *base, modelViewProjection, material, lighting, normal);
    }
    return true;
}

bool DiligentSceneryTreeBatch::draw(DiligentTexturedMeshRenderer& renderer,
                                    Diligent::IDeviceContext& context,
                                    Diligent::ITextureView& fallbackWhite,
                                    const Matrix4& modelViewProjection,
                                    const DiligentSceneryOpaqueDraw& draw,
                                    const DiligentLightingParameters& lighting,
                                    const DiligentMotionParameters* motion) const {
    if (!std::isfinite(draw.visibilityFade) || draw.visibilityFade < 0.0F ||
        draw.visibilityFade > 1.0F) {
        throw std::invalid_argument("Diligent tree visibility fade must be finite and in [0, 1]");
    }
    if (draw.visibilityFade == 0.0F ||
        (draw.sceneryLodIndex && source.sceneryLodIndex != *draw.sceneryLodIndex) || !mesh) {
        return false;
    }
    if (source.alphaMode != 0 && source.alphaMode != 1) {
        throw std::invalid_argument("Diligent tree batches require opaque or cutout alpha mode");
    }
    Diligent::ITextureView* base =
        draw.seasonalTexture ? draw.seasonalTexture
                             : (texture ? &texture->shaderResourceView() : &fallbackWhite);
    DiligentMaterialState material;
    material.alphaMode =
        draw.visibilityFade < 1.0F
            ? DiligentAlphaMode::Blend
            : (source.alphaMode == 1 ? DiligentAlphaMode::Cutout : DiligentAlphaMode::Opaque);
    material.baseColor = {static_cast<float>(source.color[0]), static_cast<float>(source.color[1]),
                          static_cast<float>(source.color[2]), static_cast<float>(source.alpha)};
    material.whiteAmbient = source.material.whiteAmbient;
    material.cullBackFaces = source.cullBackFaces && !draw.reverseWinding;
    material.useVertexColor = false;
    material.flipTextureY = source.material.flipTextureY;
    material.addressU = source.material.addressU;
    material.addressV = source.material.addressV;
    material.visibilityFade = draw.visibilityFade;
    material.depthWrite = !source.noZwrite;
    material.blendEnabled = source.noZwrite || draw.visibilityFade < 1.0F;
    if (source.noZcheck && !source.noZwrite && source.alphaMode == 0) {
        material.depthMode = DiligentDepthMode::Disabled;
    }
    if (motion != nullptr) {
        renderer.drawMotion(context, *mesh, *base, modelViewProjection, *motion, material,
                            lighting);
    } else {
        renderer.draw(context, *mesh, *base, modelViewProjection, material, lighting);
    }
    return true;
}

DiligentSceneryAsset::DiligentSceneryAsset(Diligent::IRenderDevice& device,
                                           const PreparedSceneryAsset& prepared,
                                           bool generateMipmaps) {
    lodThresholds_ = prepared.modelConfiguration.lodThresholds;
    if (prepared.preparationError) {
        std::rethrow_exception(prepared.preparationError);
    }

    for (const auto& [key, mesh] : prepared.meshes) {
        if (mesh.preparationError) {
            try {
                std::rethrow_exception(mesh.preparationError);
            } catch (const std::exception& error) {
                diagnostics_.push_back("Scenery mesh " + key + ": " + error.what());
            }
        } else if (!mesh.parsed) {
            diagnostics_.push_back("Scenery mesh could not be parsed: " + key);
        }
        if (mesh.invalidPositionIndices != 0) {
            diagnostics_.push_back("Scenery mesh " + key + " has " +
                                   std::to_string(mesh.invalidPositionIndices) +
                                   " invalid position indices");
        }
    }

    std::unordered_map<std::string, std::shared_ptr<DiligentTexture2D>> textures;
    const auto uploadTexture = [&](const std::filesystem::path& path) {
        if (path.empty()) {
            return std::shared_ptr<DiligentTexture2D>{};
        }
        const std::string key = openbus::pathToUtf8(path.lexically_normal());
        const auto cached = textures.find(key);
        if (cached != textures.end()) {
            return cached->second;
        }
        const auto image = prepared.decodedTextures.find(key);
        if (image == prepared.decodedTextures.end() || !image->second) {
            throw std::invalid_argument("Prepared scenery texture has no decoded image: " + key);
        }
        auto texture = std::make_shared<DiligentTexture2D>(device, *image->second, generateMipmaps);
        textures.emplace(key, texture);
        return texture;
    };
    const auto missingTexture = [&](const std::string& kind, const std::string& name,
                                    const std::filesystem::path& path,
                                    const std::string& preparationError) {
        if (!preparationError.empty()) {
            diagnostics_.push_back("Scenery " + kind + " texture " + name + ": " +
                                   preparationError);
        } else if (!name.empty() && path.empty()) {
            diagnostics_.push_back("Scenery " + kind + " texture not found: " + name);
        }
    };
    const auto uploadAuxTexture = [&](const std::string& kind,
                                      const PreparedSceneryAuxTexture& texture) {
        missingTexture(kind, texture.name, texture.path, texture.preparationError);
        return uploadTexture(texture.path);
    };
    std::unordered_map<std::string, std::unordered_map<int, std::shared_ptr<DiligentTexturedMesh>>>
        meshes;
    batches_.reserve(prepared.batches.size());
    for (const PreparedSceneryBatch& source : prepared.batches) {
        const auto preparedMesh = prepared.meshes.find(source.meshKey);
        if (preparedMesh == prepared.meshes.end()) {
            throw std::invalid_argument("Prepared scenery batch references a missing mesh: " +
                                        source.meshKey);
        }
        if (preparedMesh->second.preparationError) {
            std::rethrow_exception(preparedMesh->second.preparationError);
        }
        const auto vertices = preparedMesh->second.verticesByMaterial.find(source.materialIndex);
        if (vertices == preparedMesh->second.verticesByMaterial.end() ||
            vertices->second.size() != source.batch.vertexCount) {
            throw std::invalid_argument(
                "Prepared scenery batch has missing or inconsistent geometry: " + source.meshKey);
        }
        auto& materialMeshes = meshes[source.meshKey];
        auto mesh = materialMeshes.find(source.materialIndex);
        if (mesh == materialMeshes.end()) {
            mesh = materialMeshes
                       .emplace(source.materialIndex,
                                std::make_shared<DiligentTexturedMesh>(device, vertices->second))
                       .first;
        }
        DiligentSceneryBatch batch;
        batch.source = source;
        batch.mesh = mesh->second;
        batch.localCenter = geometryCenter(vertices->second);
        missingTexture("base", source.textureName, source.texturePath, {});
        batch.textures.base = uploadTexture(source.texturePath);
        missingTexture("transmap", source.transmapTextureName, source.transmapTexturePath, {});
        batch.textures.transmap = uploadTexture(source.transmapTexturePath);
        batch.textures.lightmap = uploadAuxTexture("lightmap", source.lightmap);
        batch.textures.nightmap = uploadAuxTexture("nightmap", source.nightmap);
        batch.textures.bumpmap = uploadAuxTexture("bumpmap", source.bumpmap);
        batch.textures.environment = uploadAuxTexture("environment", source.environment);
        missingTexture("free", source.freeTextureName, source.freeTexturePath, {});
        batch.textures.freeTexture = uploadTexture(source.freeTexturePath);
        batch.textures.itemLightmaps.reserve(source.itemLightmaps.size());
        for (const auto& texture : source.itemLightmaps) {
            batch.textures.itemLightmaps.push_back(uploadAuxTexture("item lightmap", texture));
        }
        batch.textures.itemNightmaps.reserve(source.itemNightmaps.size());
        for (const auto& texture : source.itemNightmaps) {
            batch.textures.itemNightmaps.push_back(uploadAuxTexture("item nightmap", texture));
        }
        batches_.push_back(std::move(batch));
    }

    trees_.reserve(prepared.treeBatches.size());
    for (const PreparedSceneryTreeBatch& source : prepared.treeBatches) {
        DiligentSceneryTreeBatch tree;
        tree.source = source.batch;
        tree.localCenter = geometryCenter(source.vertices);
        tree.textureName = source.textureName;
        tree.texturePath = source.texturePath;
        tree.seasonTexturePaths = source.seasonTexturePaths;
        if (source.texturePath.empty()) {
            diagnostics_.push_back("Scenery tree texture not found: " + source.textureName);
        }
        if (!source.texturePath.empty()) {
            if (source.vertices.size() != source.batch.vertexCount) {
                throw std::invalid_argument("Prepared scenery tree has inconsistent geometry");
            }
            tree.mesh = std::make_shared<DiligentTexturedMesh>(device, source.vertices);
            tree.texture = uploadTexture(source.texturePath);
        }
        trees_.push_back(std::move(tree));
    }
}

std::span<const DiligentSceneryBatch> DiligentSceneryAsset::batches() const noexcept {
    return batches_;
}

std::span<const DiligentSceneryTreeBatch> DiligentSceneryAsset::trees() const noexcept {
    return trees_;
}

std::size_t DiligentSceneryAsset::selectLod(double distance) const noexcept {
    return openbus::map::selectMapSceneryLod(lodThresholds_, distance);
}

std::span<const std::string> DiligentSceneryAsset::diagnostics() const noexcept {
    return diagnostics_;
}

std::vector<DiligentScenerySubmission> orderDiligentTransparentScenery(
    std::span<const DiligentSceneryBatch> batches, std::span<const std::size_t> batchIndices,
    std::span<const DiligentSceneryTreeBatch> trees, std::span<const std::size_t> treeIndices,
    const Matrix4& modelView) {
    std::vector<std::pair<double, DiligentScenerySubmission>> ordered;
    ordered.reserve(batchIndices.size() + treeIndices.size());
    const auto append = [&](const std::array<double, 3>& center, DiligentScenerySubmissionKind kind,
                            std::size_t index) {
        const double depth = modelView[2] * center[0] + modelView[6] * center[1] +
                             modelView[10] * center[2] + modelView[14];
        if (!std::isfinite(depth)) {
            throw std::invalid_argument("Diligent transparent scenery has invalid view depth");
        }
        ordered.emplace_back(depth, DiligentScenerySubmission{kind, index});
    };
    for (const std::size_t index : batchIndices) {
        if (index >= batches.size()) {
            throw std::out_of_range("Diligent transparent scenery batch index is out of range");
        }
        append(batches[index].localCenter, DiligentScenerySubmissionKind::Batch, index);
    }
    for (const std::size_t index : treeIndices) {
        if (index >= trees.size()) {
            throw std::out_of_range("Diligent transparent scenery tree index is out of range");
        }
        append(trees[index].localCenter, DiligentScenerySubmissionKind::Tree, index);
    }
    std::stable_sort(ordered.begin(), ordered.end(),
                     [](const auto& left, const auto& right) { return left.first < right.first; });

    std::vector<DiligentScenerySubmission> submissions;
    submissions.reserve(ordered.size());
    for (const auto& [depth, submission] : ordered) {
        (void)depth;
        submissions.push_back(submission);
    }
    return submissions;
}

} // namespace openbus::rendering
