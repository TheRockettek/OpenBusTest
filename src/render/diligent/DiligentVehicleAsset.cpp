#include "render/diligent/DiligentVehicleAsset.h"

#include "render/diligent/DiligentTexture2D.h"
#include "render/diligent/DiligentTexture2DArray.h"
#include "render/diligent/DiligentRuntimeTexture2D.h"
#include "render/MatrixMath.h"
#include "config/ModelConfigTypes.h"

#include <Graphics/GraphicsEngine/interface/TextureView.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <unordered_map>
#include <type_traits>
#include <utility>

namespace openbus::rendering {
namespace {

bool hasTextureSource(const DiligentVehicleTextureSource& source) {
    return std::visit(
        [](const auto& value) {
            using Value = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<Value, std::monostate>) {
                return false;
            } else {
                return static_cast<bool>(value);
            }
        },
        source);
}

void bindAuxiliaryTextures(DiligentMaterialState& material,
                           const DiligentVehicleAuxiliaryTextures& textures) {
    if (material.lightMap == nullptr && textures.lightMap) {
        material.lightMap = &textures.lightMap->shaderResourceView();
    }
    if (material.nightMap == nullptr && textures.nightMap) {
        material.nightMap = &textures.nightMap->shaderResourceView();
    }
    if (material.transMap == nullptr && textures.transMap) {
        material.transMap = &textures.transMap->shaderResourceView();
    }
    if (material.detailMap == nullptr && textures.detailMap) {
        material.detailMap = &textures.detailMap->shaderResourceView();
    }
    if (material.freeMap == nullptr && textures.freeMap) {
        material.freeMap = &textures.freeMap->shaderResourceView();
    }
    if (material.textMap == nullptr && textures.textMap) {
        material.textMap = &textures.textMap->shaderResourceView();
    }
    if (material.environmentMap == nullptr && textures.environmentMap) {
        material.environmentMap = &textures.environmentMap->shaderResourceView();
    }
}

} // namespace

DiligentVehicleGroupTransform DiligentVehicleGroupTransform::fromMotionHistory(
    std::size_t animationGroupIndex, const MotionDataFrame& frame, MotionHistory& history,
    MotionObjectKey objectKey, const Matrix4& currentModelView, bool geometryHistoryValid,
    double maximumTranslationDeltaMeters) {
    DiligentVehicleGroupTransform transform;
    transform.animationGroupIndex = animationGroupIndex;
    transform.modelView = currentModelView;
    transform.modelViewProjection = multiplyMatrix4(frame.currentProjection, currentModelView);
    const MotionTransform previous = history.previousModelView(
        currentModelView, objectKey, geometryHistoryValid, maximumTranslationDeltaMeters);
    transform.motion = DiligentMotionParameters::fromHistory(frame, previous);
    return transform;
}

DiligentVehicleGroupTransform DiligentVehicleGroupTransform::fromAnimationsAndHistory(
    std::size_t animationGroupIndex, const MotionDataFrame& frame, MotionHistory& history,
    MotionObjectKey objectKey, const Matrix4& parentModelView,
    std::span<const ModelAnimation> animations, std::span<const double> animationAmounts,
    bool geometryHistoryValid, double maximumTranslationDeltaMeters) {
    if (animations.size() != animationAmounts.size()) {
        throw std::invalid_argument(
            "Diligent vehicle animation definitions and values must have matching sizes");
    }
    ModelAnimationTransformBuilder animationTransform;
    for (std::size_t index = 0; index < animations.size(); ++index) {
        animationTransform.append(animations[index], animationAmounts[index]);
    }
    const Matrix4 currentModelView = multiplyMatrix4(parentModelView, animationTransform.matrix());
    return fromMotionHistory(animationGroupIndex, frame, history, objectKey, currentModelView,
                             geometryHistoryValid, maximumTranslationDeltaMeters);
}

std::vector<DiligentVehicleGroupTransform>
buildDiligentVehicleGroupTransforms(std::span<const DiligentVehicleAnimationGroupInput> groups,
                                    const MotionDataFrame& frame, MotionHistory& history,
                                    std::uint64_t vehicleObjectId, const Matrix4& parentModelView,
                                    double maximumTranslationDeltaMeters) {
    if (vehicleObjectId == 0) {
        throw std::invalid_argument("Diligent vehicle motion requires a stable object identifier");
    }

    std::unordered_map<std::size_t, std::size_t> groupPositions;
    groupPositions.reserve(groups.size());
    for (std::size_t index = 0; index < groups.size(); ++index) {
        if (!groupPositions.emplace(groups[index].animationGroupIndex, index).second) {
            throw std::invalid_argument(
                "Diligent vehicle animation group identifiers must be unique");
        }
        if (groups[index].animations.size() != groups[index].animationAmounts.size()) {
            throw std::invalid_argument(
                "Diligent vehicle animation definitions and values must have matching sizes");
        }
    }

    std::vector<Matrix4> modelViews(groups.size());
    std::vector<bool> effectiveHistoryValidity(groups.size(), false);
    std::vector<std::uint8_t> states(groups.size(), 0);
    const std::function<void(std::size_t)> resolveGroup = [&](std::size_t position) {
        if (states[position] == 2) {
            return;
        }
        if (states[position] == 1) {
            throw std::invalid_argument("Diligent vehicle animation groups contain a parent cycle");
        }
        states[position] = 1;
        const DiligentVehicleAnimationGroupInput& group = groups[position];
        Matrix4 groupParentModelView = parentModelView;
        bool parentHistoryValid = true;
        if (group.parentAnimationGroupIndex.has_value()) {
            const auto parent = groupPositions.find(*group.parentAnimationGroupIndex);
            if (parent == groupPositions.end()) {
                throw std::invalid_argument(
                    "Diligent vehicle animation group references a missing parent");
            }
            resolveGroup(parent->second);
            groupParentModelView = modelViews[parent->second];
            parentHistoryValid = effectiveHistoryValidity[parent->second];
        }

        ModelAnimationTransformBuilder localTransform;
        for (std::size_t index = 0; index < group.animations.size(); ++index) {
            localTransform.append(group.animations[index], group.animationAmounts[index]);
        }
        modelViews[position] = multiplyMatrix4(groupParentModelView, localTransform.matrix());
        effectiveHistoryValidity[position] = group.geometryHistoryValid && parentHistoryValid;
        states[position] = 2;
    };

    for (std::size_t index = 0; index < groups.size(); ++index) {
        resolveGroup(index);
    }

    std::vector<DiligentVehicleGroupTransform> transforms;
    transforms.reserve(groups.size());
    for (std::size_t index = 0; index < groups.size(); ++index) {
        const DiligentVehicleAnimationGroupInput& group = groups[index];
        transforms.push_back(DiligentVehicleGroupTransform::fromMotionHistory(
            group.animationGroupIndex, frame, history,
            {vehicleObjectId, static_cast<std::uint64_t>(group.animationGroupIndex)},
            modelViews[index], effectiveHistoryValidity[index], maximumTranslationDeltaMeters));
    }
    return transforms;
}

void DiligentVehicleSubmesh::draw(
    DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
    Diligent::ITextureView& fallbackTexture, const Matrix4& modelViewProjection,
    const DiligentMaterialState* materialOverride, Diligent::ITextureView* baseTextureOverride,
    const DiligentLightingParameters& lighting, const DiligentMotionParameters* motion,
    Diligent::ITextureView* normalMap) const {
    if (!mesh) {
        throw std::logic_error("Diligent vehicle submesh has no mesh");
    }
    DiligentMaterialState selectedMaterial =
        materialOverride == nullptr ? material : *materialOverride;
    bindAuxiliaryTextures(selectedMaterial, auxiliaryTextures);
    Diligent::ITextureView* selectedNormalMap =
        normalMap != nullptr          ? normalMap
        : auxiliaryTextures.normalMap ? &auxiliaryTextures.normalMap->shaderResourceView()
                                      : nullptr;
    if (baseTextureOverride != nullptr) {
        if (motion != nullptr) {
            renderer.drawMotion(context, *mesh, *baseTextureOverride, modelViewProjection, *motion,
                                selectedMaterial, lighting, selectedNormalMap);
        } else {
            renderer.draw(context, *mesh, *baseTextureOverride, modelViewProjection,
                          selectedMaterial, lighting, selectedNormalMap);
        }
    } else if (textureArray) {
        if (motion != nullptr) {
            renderer.drawMotion(context, *mesh, *textureArray, modelViewProjection, *motion,
                                selectedMaterial, lighting, selectedNormalMap);
        } else {
            renderer.draw(context, *mesh, *textureArray, modelViewProjection, selectedMaterial,
                          lighting, selectedNormalMap);
        }
    } else if (texture) {
        if (motion != nullptr) {
            renderer.drawMotion(context, *mesh, *texture, modelViewProjection, *motion,
                                selectedMaterial, lighting, selectedNormalMap);
        } else {
            renderer.draw(context, *mesh, *texture, modelViewProjection, selectedMaterial, lighting,
                          selectedNormalMap);
        }
    } else {
        if (motion != nullptr) {
            renderer.drawMotion(context, *mesh, fallbackTexture, modelViewProjection, *motion,
                                selectedMaterial, lighting, selectedNormalMap);
        } else {
            renderer.draw(context, *mesh, fallbackTexture, modelViewProjection, selectedMaterial,
                          lighting, selectedNormalMap);
        }
    }
}

namespace {

struct TextureArrayKeyHash {
    std::size_t operator()(const std::vector<const Image*>& layers) const noexcept {
        std::size_t hash = layers.size();
        for (const Image* image : layers) {
            const std::size_t value = std::hash<const Image*>{}(image);
            hash ^= value + 0x9e3779b9U + (hash << 6U) + (hash >> 2U);
        }
        return hash;
    }
};

} // namespace

DiligentVehicleAsset::DiligentVehicleAsset(Diligent::IRenderDevice& device,
                                           std::span<const DiligentVehicleBatchSource> batches,
                                           bool generateMipmaps) {
    if (batches.empty()) {
        throw std::invalid_argument("Diligent vehicle asset requires at least one batch");
    }

    for (const auto& batch : batches) {
        if (batch.vertices.empty() || batch.vertices.size() % 3 != 0) {
            throw std::invalid_argument("Diligent vehicle batches must contain complete triangles");
        }
    }

    std::unordered_map<const Image*, std::shared_ptr<DiligentTexture2D>> uploadedTextures;
    std::unordered_map<const gli::texture*, std::shared_ptr<DiligentTexture2D>>
        uploadedCompressedTextures;
    std::unordered_map<std::vector<const Image*>, std::shared_ptr<DiligentTexture2DArray>,
                       TextureArrayKeyHash>
        uploadedTextureArrays;
    std::unordered_map<const gli::texture*, std::shared_ptr<DiligentTexture2DArray>>
        uploadedCompressedTextureArrays;
    std::unordered_map<const CompressedDds*, std::shared_ptr<DiligentTexture2D>>
        uploadedCompressedDds;
    const auto uploadTexture = [&](const DiligentVehicleTextureSource& source) {
        if (const auto* image = std::get_if<std::shared_ptr<const Image>>(&source)) {
            if (!*image) {
                return std::shared_ptr<DiligentTexture2D>{};
            }
            auto [texture, inserted] = uploadedTextures.try_emplace(image->get());
            if (inserted) {
                texture->second =
                    std::make_shared<DiligentTexture2D>(device, **image, generateMipmaps);
            }
            return texture->second;
        }
        if (const auto* image = std::get_if<std::shared_ptr<const CompressedDds>>(&source)) {
            if (!*image) {
                return std::shared_ptr<DiligentTexture2D>{};
            }
            auto [texture, inserted] = uploadedCompressedDds.try_emplace(image->get());
            if (inserted) {
                texture->second = std::make_shared<DiligentTexture2D>(device, **image);
            }
            return texture->second;
        }
        if (const auto* image = std::get_if<std::shared_ptr<const gli::texture>>(&source)) {
            if (!*image) {
                return std::shared_ptr<DiligentTexture2D>{};
            }
            auto [texture, inserted] = uploadedCompressedTextures.try_emplace(image->get());
            if (inserted) {
                texture->second = std::make_shared<DiligentTexture2D>(device, **image);
            }
            return texture->second;
        }
        return std::shared_ptr<DiligentTexture2D>{};
    };
    submeshes_.reserve(batches.size());
    for (const auto& batch : batches) {
        const bool hasBaseTexture = hasTextureSource(batch.texture);
        const bool hasDecodedTextureArray = !batch.textureArrayLayers.empty();
        const bool hasCompressedTextureArray = static_cast<bool>(batch.compressedTextureArray);
        if ((hasDecodedTextureArray && hasCompressedTextureArray) ||
            (hasBaseTexture && (hasDecodedTextureArray || hasCompressedTextureArray))) {
            throw std::invalid_argument(
                "Diligent vehicle batch cannot combine multiple base texture sources");
        }
        DiligentVehicleSubmesh submesh;
        submesh.materialIndex = batch.materialIndex;
        submesh.sourceBatchIndex = batch.sourceBatchIndex;
        submesh.animationGroupIndex = batch.animationGroupIndex;
        std::array<double, 3> center = {};
        for (const ModelVertex& vertex : batch.vertices) {
            center[0] += vertex.x;
            center[1] += vertex.y;
            center[2] += vertex.z;
        }
        const double inverseVertexCount = 1.0 / static_cast<double>(batch.vertices.size());
        for (double& coordinate : center) {
            coordinate *= inverseVertexCount;
        }
        submesh.localCenter = center;
        submesh.mesh = std::make_unique<DiligentTexturedMesh>(device, batch.vertices);
        submesh.material = batch.material;
        submesh.texture = uploadTexture(batch.texture);
        submesh.auxiliaryTextures.lightMap = uploadTexture(batch.auxiliaryTextures.lightMap);
        submesh.auxiliaryTextures.nightMap = uploadTexture(batch.auxiliaryTextures.nightMap);
        submesh.auxiliaryTextures.transMap = uploadTexture(batch.auxiliaryTextures.transMap);
        submesh.auxiliaryTextures.detailMap = uploadTexture(batch.auxiliaryTextures.detailMap);
        submesh.auxiliaryTextures.freeMap = uploadTexture(batch.auxiliaryTextures.freeMap);
        submesh.auxiliaryTextures.textMap = uploadTexture(batch.auxiliaryTextures.textMap);
        submesh.auxiliaryTextures.environmentMap =
            uploadTexture(batch.auxiliaryTextures.environmentMap);
        submesh.auxiliaryTextures.normalMap = uploadTexture(batch.auxiliaryTextures.normalMap);
        bindAuxiliaryTextures(submesh.material, submesh.auxiliaryTextures);
        if (!batch.textureArrayLayers.empty()) {
            std::vector<const Image*> key;
            key.reserve(batch.textureArrayLayers.size());
            for (const auto& layer : batch.textureArrayLayers) {
                if (!layer) {
                    throw std::invalid_argument(
                        "Diligent vehicle texture array contains a null layer");
                }
                key.push_back(layer.get());
            }
            auto [textureArray, inserted] = uploadedTextureArrays.try_emplace(key);
            if (inserted) {
                textureArray->second =
                    std::make_shared<DiligentTexture2DArray>(device, batch.textureArrayLayers);
            }
            submesh.textureArray = textureArray->second;
        } else if (batch.compressedTextureArray) {
            auto [textureArray, inserted] =
                uploadedCompressedTextureArrays.try_emplace(batch.compressedTextureArray.get());
            if (inserted) {
                textureArray->second =
                    std::make_shared<DiligentTexture2DArray>(device, *batch.compressedTextureArray);
            }
            submesh.textureArray = textureArray->second;
        }
        submeshes_.push_back(std::move(submesh));
    }
}

std::span<const DiligentVehicleSubmesh> DiligentVehicleAsset::submeshes() const noexcept {
    return submeshes_;
}

std::vector<DiligentShadowCaster> DiligentVehicleAsset::shadowCasters(
    const Matrix4& lightClipFromView, const Matrix4& modelView,
    std::span<const DiligentVehicleGroupTransform> groupTransforms,
    std::span<const DiligentVehicleMaterialOverride> materialOverrides) const {
    std::unordered_map<std::size_t, const DiligentVehicleGroupTransform*> transformsByGroup;
    transformsByGroup.reserve(groupTransforms.size());
    for (const DiligentVehicleGroupTransform& transform : groupTransforms) {
        if (!transformsByGroup.emplace(transform.animationGroupIndex, &transform).second) {
            throw std::invalid_argument("Diligent vehicle animation group has multiple transforms");
        }
    }
    for (const DiligentVehicleMaterialOverride& override : materialOverrides) {
        if (override.animationGroupIndex.has_value() != override.sourceBatchIndex.has_value()) {
            throw std::invalid_argument(
                "Diligent vehicle batch override must specify both source indices");
        }
    }

    std::vector<DiligentShadowCaster> casters;
    casters.reserve(submeshes_.size());
    for (const DiligentVehicleSubmesh& submesh : submeshes_) {
        const DiligentVehicleGroupTransform* transform = nullptr;
        if (!groupTransforms.empty()) {
            const auto group = transformsByGroup.find(submesh.animationGroupIndex);
            if (group == transformsByGroup.end()) {
                throw std::invalid_argument(
                    "Diligent vehicle submesh has no animation group transform");
            }
            transform = group->second;
            if (!transform->visible) {
                continue;
            }
        }

        const auto batchOverride =
            std::find_if(materialOverrides.begin(), materialOverrides.end(),
                         [&](const DiligentVehicleMaterialOverride& candidate) {
                             return candidate.animationGroupIndex.has_value() &&
                                    *candidate.animationGroupIndex == submesh.animationGroupIndex &&
                                    *candidate.sourceBatchIndex == submesh.sourceBatchIndex;
                         });
        const auto materialOverride =
            std::find_if(materialOverrides.begin(), materialOverrides.end(),
                         [&](const DiligentVehicleMaterialOverride& candidate) {
                             return !candidate.animationGroupIndex.has_value() &&
                                    candidate.materialIndex == submesh.materialIndex;
                         });
        const DiligentVehicleMaterialOverride* selectedOverride =
            batchOverride != materialOverrides.end()      ? &*batchOverride
            : materialOverride != materialOverrides.end() ? &*materialOverride
                                                          : nullptr;
        const DiligentMaterialState& material =
            selectedOverride != nullptr && selectedOverride->material != nullptr
                ? *selectedOverride->material
                : submesh.material;
        if (!std::isfinite(material.baseColor[3]) || material.baseColor[3] < 0.0F ||
            material.baseColor[3] > 1.0F || !std::isfinite(material.visibilityFade) ||
            material.visibilityFade < 0.0F || material.visibilityFade > 1.0F) {
            throw std::invalid_argument("Diligent vehicle shadow alpha scale is invalid");
        }
        if (material.blendEnabled) {
            continue;
        }
        const DiligentAlphaMode shadowAlphaMode = submesh.material.alphaMode;
        float alphaCutoff = 0.0F;
        if (shadowAlphaMode == DiligentAlphaMode::Cutout) {
            alphaCutoff = 0.5F;
        } else if (shadowAlphaMode == DiligentAlphaMode::NearOpaque) {
            alphaCutoff = 0.99F;
        } else if (shadowAlphaMode == DiligentAlphaMode::NearOpaqueBlend) {
            continue;
        }
        if (alphaCutoff > 0.0F) {
            const float alphaScale = material.baseColor[3] * material.visibilityFade;
            if (alphaScale < alphaCutoff) {
                continue;
            }
            alphaCutoff /= alphaScale;
        }

        Diligent::ITextureView* alphaTexture =
            selectedOverride != nullptr ? selectedOverride->baseTexture : nullptr;
        if (selectedOverride != nullptr && selectedOverride->runtimeTexture != nullptr) {
            if (alphaTexture != nullptr) {
                throw std::invalid_argument(
                    "Diligent vehicle override cannot specify both runtime and base textures");
            }
            if (!selectedOverride->runtimeTexture->initialized() ||
                selectedOverride->runtimeTexture->shaderResourceView() == nullptr) {
                throw std::logic_error("Diligent vehicle runtime base texture is not initialized");
            }
            alphaTexture = selectedOverride->runtimeTexture->shaderResourceView();
        }
        if (alphaTexture == nullptr) {
            if (submesh.textureArray) {
                alphaTexture = &submesh.textureArray->shaderResourceView();
            } else if (submesh.texture) {
                alphaTexture = &submesh.texture->shaderResourceView();
            }
        }

        DiligentShadowCaster caster;
        caster.mesh = submesh.mesh.get();
        const Matrix4& selectedModelView = transform != nullptr ? transform->modelView : modelView;
        caster.modelViewProjection = multiplyMatrix4(lightClipFromView, selectedModelView);
        caster.alphaTexture = alphaCutoff > 0.0F ? alphaTexture : nullptr;
        caster.alphaCutoff = caster.alphaTexture != nullptr ? alphaCutoff : 0.0F;
        caster.textureTransform = {material.textureScale[0], material.textureScale[1],
                                   material.textureOffset[0], material.textureOffset[1]};
        if (caster.alphaTexture != nullptr &&
            caster.alphaTexture->GetDesc().TextureDim == Diligent::RESOURCE_DIM_TEX_2D_ARRAY) {
            caster.useVertexArrayLayer = true;
        }
        casters.push_back(caster);
    }
    return casters;
}

void DiligentVehicleAsset::draw(DiligentTexturedMeshRenderer& renderer,
                                Diligent::IDeviceContext& context,
                                Diligent::ITextureView& fallbackTexture,
                                const Matrix4& modelViewProjection, const Matrix4& modelView,
                                const DiligentLightingParameters& lighting,
                                const DiligentMotionParameters* motion,
                                std::span<const DiligentVehicleMaterialOverride> materialOverrides,
                                Diligent::ITextureView* normalMap,
                                std::span<const DiligentVehicleGroupTransform> groupTransforms,
                                DiligentVehicleRenderPass pass) const {
    (void)drawPass(renderer, context, fallbackTexture, modelViewProjection, modelView, pass,
                   lighting, motion, materialOverrides, normalMap, groupTransforms);
}

std::size_t DiligentVehicleAsset::drawPass(
    DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
    Diligent::ITextureView& fallbackTexture, const Matrix4& modelViewProjection,
    const Matrix4& modelView, DiligentVehicleRenderPass pass,
    const DiligentLightingParameters& lighting, const DiligentMotionParameters* motion,
    std::span<const DiligentVehicleMaterialOverride> materialOverrides,
    Diligent::ITextureView* normalMap,
    std::span<const DiligentVehicleGroupTransform> groupTransforms) const {
    if (pass != DiligentVehicleRenderPass::All && pass != DiligentVehicleRenderPass::Opaque &&
        pass != DiligentVehicleRenderPass::Transparent) {
        throw std::invalid_argument("Diligent vehicle draw pass is invalid");
    }
    std::vector<std::size_t> opaqueOrder;
    std::vector<std::size_t> transparentOrder;
    std::vector<const DiligentVehicleMaterialOverride*> selectedOverrides(submeshes_.size(),
                                                                          nullptr);
    std::vector<const DiligentVehicleGroupTransform*> selectedTransforms(submeshes_.size(),
                                                                         nullptr);
    std::unordered_map<std::size_t, const DiligentVehicleGroupTransform*> transformsByGroup;
    for (const DiligentVehicleMaterialOverride& override : materialOverrides) {
        if (override.animationGroupIndex.has_value() != override.sourceBatchIndex.has_value()) {
            throw std::invalid_argument(
                "Diligent vehicle batch override must specify both source indices");
        }
    }
    transformsByGroup.reserve(groupTransforms.size());
    for (const DiligentVehicleGroupTransform& transform : groupTransforms) {
        if (!transformsByGroup.emplace(transform.animationGroupIndex, &transform).second) {
            throw std::invalid_argument("Diligent vehicle animation group has multiple transforms");
        }
    }
    opaqueOrder.reserve(submeshes_.size());
    transparentOrder.reserve(submeshes_.size());
    for (std::size_t index = 0; index < submeshes_.size(); ++index) {
        const DiligentVehicleSubmesh& submesh = submeshes_[index];
        if (!groupTransforms.empty()) {
            const auto group = transformsByGroup.find(submesh.animationGroupIndex);
            if (group == transformsByGroup.end()) {
                throw std::invalid_argument(
                    "Diligent vehicle submesh has no animation group transform");
            }
            selectedTransforms[index] = group->second;
            if (!group->second->visible) {
                continue;
            }
        }
        const auto batchOverride =
            std::find_if(materialOverrides.begin(), materialOverrides.end(),
                         [&](const DiligentVehicleMaterialOverride& candidate) {
                             return candidate.animationGroupIndex.has_value() &&
                                    *candidate.animationGroupIndex == submesh.animationGroupIndex &&
                                    *candidate.sourceBatchIndex == submesh.sourceBatchIndex;
                         });
        if (batchOverride != materialOverrides.end()) {
            selectedOverrides[index] = &*batchOverride;
        } else {
            const auto materialOverride =
                std::find_if(materialOverrides.begin(), materialOverrides.end(),
                             [&](const DiligentVehicleMaterialOverride& candidate) {
                                 return !candidate.animationGroupIndex.has_value() &&
                                        candidate.materialIndex == submesh.materialIndex;
                             });
            if (materialOverride != materialOverrides.end()) {
                selectedOverrides[index] = &*materialOverride;
            }
        }
        const DiligentVehicleMaterialOverride* selectedOverride = selectedOverrides[index];
        const DiligentMaterialState& selectedMaterial =
            selectedOverride != nullptr && selectedOverride->material != nullptr
                ? *selectedOverride->material
                : submesh.material;
        if (diligentMaterialBlends(selectedMaterial)) {
            transparentOrder.push_back(index);
        } else {
            opaqueOrder.push_back(index);
        }
    }

    std::vector<double> viewDepths(submeshes_.size(), 0.0);
    for (const std::size_t index : transparentOrder) {
        const auto& center = submeshes_[index].localCenter;
        const DiligentVehicleGroupTransform* transform = selectedTransforms[index];
        const Matrix4& selectedModelView = transform != nullptr ? transform->modelView : modelView;
        const double depth = selectedModelView[2] * center[0] + selectedModelView[6] * center[1] +
                             selectedModelView[10] * center[2] + selectedModelView[14];
        if (!std::isfinite(depth)) {
            throw std::invalid_argument(
                "Diligent vehicle transparent batch has invalid view depth");
        }
        viewDepths[index] = depth;
    }
    std::stable_sort(
        transparentOrder.begin(), transparentOrder.end(),
        [&](std::size_t left, std::size_t right) { return viewDepths[left] < viewDepths[right]; });

    const auto drawSubmesh = [&](std::size_t index) {
        const DiligentVehicleSubmesh& submesh = submeshes_[index];
        const DiligentVehicleMaterialOverride* override = selectedOverrides[index];
        const DiligentVehicleGroupTransform* transform = selectedTransforms[index];
        const DiligentMaterialState* materialOverride =
            override != nullptr ? override->material : nullptr;
        Diligent::ITextureView* textureOverride =
            override != nullptr ? override->baseTexture : nullptr;
        DiligentMaterialState runtimeMaterial;
        const bool hasRuntimeBase = override != nullptr && override->runtimeTexture != nullptr;
        const bool hasRuntimeFreeMap = override != nullptr && override->freeMapTexture != nullptr;
        const bool hasRuntimeTextMap = override != nullptr && override->textMapTexture != nullptr;
        if (hasRuntimeBase || hasRuntimeFreeMap || hasRuntimeTextMap) {
            if (textureOverride != nullptr) {
                if (hasRuntimeBase) {
                    throw std::invalid_argument(
                        "Diligent vehicle override cannot specify both runtime and base textures");
                }
            }
            runtimeMaterial = materialOverride != nullptr ? *materialOverride : submesh.material;
            if (hasRuntimeBase) {
                if (!override->runtimeTexture->initialized() ||
                    override->runtimeTexture->shaderResourceView() == nullptr) {
                    throw std::logic_error(
                        "Diligent vehicle runtime base texture is not initialized");
                }
                runtimeMaterial.nearestBaseTexture = !override->runtimeTexture->filtered();
                textureOverride = override->runtimeTexture->shaderResourceView();
            }
            if (hasRuntimeFreeMap) {
                if (!override->freeMapTexture->initialized() ||
                    override->freeMapTexture->shaderResourceView() == nullptr) {
                    throw std::logic_error(
                        "Diligent vehicle runtime free texture is not initialized");
                }
                runtimeMaterial.freeMap = override->freeMapTexture->shaderResourceView();
                runtimeMaterial.nearestFreeTexture = !override->freeMapTexture->filtered();
            }
            if (hasRuntimeTextMap) {
                if (!override->textMapTexture->initialized() ||
                    override->textMapTexture->shaderResourceView() == nullptr) {
                    throw std::logic_error(
                        "Diligent vehicle runtime text texture is not initialized");
                }
                runtimeMaterial.textMap = override->textMapTexture->shaderResourceView();
                runtimeMaterial.nearestTextTexture = !override->textMapTexture->filtered();
            }
            materialOverride = &runtimeMaterial;
        }
        const Matrix4& selectedModelViewProjection =
            transform != nullptr ? transform->modelViewProjection : modelViewProjection;
        const DiligentLightingParameters& selectedLighting =
            transform != nullptr && transform->lighting.has_value() ? *transform->lighting
                                                                    : lighting;
        const DiligentMotionParameters* selectedMotion =
            transform != nullptr && transform->motion.has_value() ? &*transform->motion : motion;
        submesh.draw(renderer, context, fallbackTexture, selectedModelViewProjection,
                     materialOverride, textureOverride, selectedLighting, selectedMotion,
                     normalMap);
    };
    std::size_t submitted = 0;
    if (pass != DiligentVehicleRenderPass::Transparent) {
        for (const std::size_t index : opaqueOrder) {
            drawSubmesh(index);
            ++submitted;
        }
    }
    if (pass != DiligentVehicleRenderPass::Opaque) {
        for (const std::size_t index : transparentOrder) {
            drawSubmesh(index);
            ++submitted;
        }
    }
    return submitted;
}

} // namespace openbus::rendering
