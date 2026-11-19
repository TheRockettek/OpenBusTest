#include "render/diligent/DiligentHumanAsset.h"

#include "render/diligent/DiligentTexture2D.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace openbus::rendering {
namespace {

bool isValidHumanPass(DiligentHumanRenderPass pass) {
    return pass == DiligentHumanRenderPass::All || pass == DiligentHumanRenderPass::Opaque ||
           pass == DiligentHumanRenderPass::Transparent;
}

bool isTransparentHumanMaterial(const DiligentMaterialState& material) {
    return diligentMaterialBlends(material);
}

bool matchesHumanPass(const DiligentMaterialState& material, DiligentHumanRenderPass pass) {
    if (pass == DiligentHumanRenderPass::All) {
        return true;
    }
    return isTransparentHumanMaterial(material) ? pass == DiligentHumanRenderPass::Transparent
                                                : pass == DiligentHumanRenderPass::Opaque;
}

bool hasMaterial(const HumanMeshGeometry& geometry, int materialIndex) {
    return geometry.verticesByMaterial.find(materialIndex) != geometry.verticesByMaterial.end();
}

void validateMaterialMetadata(const HumanMeshGeometry& geometry,
                              const std::unordered_map<int, std::shared_ptr<const Image>>& textures,
                              const std::unordered_map<int, DiligentMaterialState>& materials) {
    if (geometry.verticesByMaterial.empty()) {
        throw std::invalid_argument("Diligent human asset requires at least one material batch");
    }

    for (const auto& [materialIndex, vertices] : geometry.verticesByMaterial) {
        if (vertices.empty() || vertices.size() % 3 != 0) {
            throw std::invalid_argument(
                "Diligent human material geometry must contain complete triangles");
        }
        const auto weights = geometry.weightsByMaterial.find(materialIndex);
        if (weights != geometry.weightsByMaterial.end() &&
            weights->second.size() != vertices.size()) {
            throw std::invalid_argument("Diligent human skin weights do not match vertex count");
        }
        const auto seatedWeights = geometry.seatedWeightsByMaterial.find(materialIndex);
        if (seatedWeights != geometry.seatedWeightsByMaterial.end() &&
            seatedWeights->second.size() != vertices.size()) {
            throw std::invalid_argument(
                "Diligent seated human skin weights do not match vertex count");
        }
    }

    for (const auto& [materialIndex, weights] : geometry.weightsByMaterial) {
        static_cast<void>(weights);
        if (!hasMaterial(geometry, materialIndex)) {
            throw std::invalid_argument("Diligent human skin weights have no matching geometry");
        }
    }
    for (const auto& [materialIndex, weights] : geometry.seatedWeightsByMaterial) {
        static_cast<void>(weights);
        if (!hasMaterial(geometry, materialIndex)) {
            throw std::invalid_argument(
                "Diligent seated human skin weights have no matching geometry");
        }
    }
    for (const auto& [materialIndex, image] : textures) {
        if (!hasMaterial(geometry, materialIndex) || !image) {
            throw std::invalid_argument("Diligent human texture has no matching material geometry");
        }
    }
    for (const auto& [materialIndex, material] : materials) {
        static_cast<void>(material);
        if (!hasMaterial(geometry, materialIndex)) {
            throw std::invalid_argument("Diligent human material state has no matching geometry");
        }
    }
}

std::vector<HumanVertexWeights> makeMainBoneWeights(std::size_t vertexCount) {
    HumanVertexWeights mainBoneWeight = {};
    mainBoneWeight[humanBoneSlotIndex(HumanBoneSlot::Main)] = 1.0F;
    return std::vector<HumanVertexWeights>(vertexCount, mainBoneWeight);
}

} // namespace

std::size_t DiligentHumanPoseKeyHash::operator()(const DiligentHumanPoseKey& key) const noexcept {
    std::size_t seed = std::hash<std::uint64_t>{}(key.objectId);
    const auto combine = [&seed](std::uint64_t value) {
        const std::size_t valueHash = std::hash<std::uint64_t>{}(value);
        seed ^= valueHash + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    };
    combine(key.viewId);
    combine(key.subObjectId);
    return seed;
}

void DiligentHumanPoseHistory::beginFrame(const MotionDataFrame& frame) {
    if (frameActive_) {
        throw std::logic_error("Diligent human pose frame was begun before completion");
    }
    if (frame.frameId == 0 || frame.width <= 0 || frame.height <= 0) {
        throw std::invalid_argument("Diligent human pose history requires a rendered frame");
    }
    if (hasPreviousFrame_ && frame.frameId <= previousFrameId_) {
        throw std::invalid_argument("Diligent human pose frame IDs must increase");
    }
    activeFrameId_ = frame.frameId;
    frameHistoryValid_ = frame.historyValid && hasPreviousFrame_ &&
                         previousFrameId_ != UINT64_MAX && frame.frameId == previousFrameId_ + 1;
    currentPoses_.clear();
    frameActive_ = true;
}

DiligentHumanPoseHistoryResult DiligentHumanPoseHistory::recordRenderedPose(
    DiligentHumanPoseKey key, const HumanBonePalette& currentPose, bool geometryHistoryValid,
    bool seated, std::uint64_t assetGeneration) {
    if (!frameActive_) {
        throw std::logic_error("Diligent human pose was recorded outside an active frame");
    }
    if (key.objectId == 0) {
        throw std::invalid_argument("Diligent human pose requires a stable object identifier");
    }
    for (const Matrix4& bone : currentPose) {
        if (!std::all_of(bone.begin(), bone.end(),
                         [](double value) { return std::isfinite(value); })) {
            throw std::invalid_argument("Diligent human pose matrices must be finite");
        }
    }
    const auto [current, inserted] =
        currentPoses_.try_emplace(key, PoseSnapshot{currentPose, seated, assetGeneration});
    if (!inserted && (current->second.pose != currentPose || current->second.seated != seated ||
                      current->second.assetGeneration != assetGeneration)) {
        throw std::invalid_argument(
            "Diligent human pose, seating state, or asset changed within the same frame");
    }

    const auto previous = previousPoses_.find(key);
    const bool valid = frameHistoryValid_ && geometryHistoryValid &&
                       previous != previousPoses_.end() && previous->second.seated == seated &&
                       previous->second.assetGeneration == assetGeneration;
    return {valid ? &previous->second.pose : nullptr, valid};
}

void DiligentHumanPoseHistory::completeFrame() {
    if (!frameActive_) {
        throw std::logic_error("Diligent human pose frame completed without being active");
    }
    previousPoses_.swap(currentPoses_);
    currentPoses_.clear();
    previousFrameId_ = activeFrameId_;
    hasPreviousFrame_ = true;
    frameHistoryValid_ = false;
    frameActive_ = false;
}

void DiligentHumanPoseHistory::invalidate() noexcept {
    previousPoses_.clear();
    currentPoses_.clear();
    activeFrameId_ = 0;
    previousFrameId_ = 0;
    hasPreviousFrame_ = false;
    frameHistoryValid_ = false;
    frameActive_ = false;
}

void DiligentHumanSubmesh::draw(
    DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
    Diligent::ITextureView& fallbackTexture, const Matrix4& modelViewProjection,
    const HumanBonePalette& currentPose, const HumanBonePalette* previousPose, bool seated,
    const DiligentLightingParameters& lighting, const DiligentMotionParameters* motion) const {
    if (!mesh) {
        throw std::logic_error("Diligent human submesh has no standing mesh");
    }
    const auto& selectedMesh = seated && seatedMesh ? *seatedMesh : *mesh;
    Diligent::ITextureView& selectedTexture =
        texture ? texture->shaderResourceView() : fallbackTexture;
    const DiligentSkinDraw skin{currentPose, previousPose};
    if (motion) {
        renderer.drawSkinnedMotion(context, selectedMesh, selectedTexture, modelViewProjection,
                                   *motion, skin, material, lighting);
    } else {
        renderer.drawSkinned(context, selectedMesh, selectedTexture, modelViewProjection, skin,
                             material, lighting);
    }
}

DiligentHumanAsset::DiligentHumanAsset(
    Diligent::IRenderDevice& device, const HumanMeshGeometry& geometry,
    const std::unordered_map<int, std::shared_ptr<const Image>>& textures,
    const std::unordered_map<int, DiligentMaterialState>& materials, std::uint64_t sourceGeneration)
    : sourceGeneration_(sourceGeneration) {
    validateMaterialMetadata(geometry, textures, materials);

    std::unordered_map<const Image*, std::shared_ptr<DiligentTexture2D>> uploadedTextures;
    std::vector<int> materialIndexes;
    materialIndexes.reserve(geometry.verticesByMaterial.size());
    for (const auto& [materialIndex, vertices] : geometry.verticesByMaterial) {
        static_cast<void>(vertices);
        materialIndexes.push_back(materialIndex);
    }
    std::sort(materialIndexes.begin(), materialIndexes.end());

    submeshes_.reserve(materialIndexes.size());
    for (const int materialIndex : materialIndexes) {
        const auto& vertices = geometry.verticesByMaterial.at(materialIndex);
        const auto seatedWeights = geometry.seatedWeightsByMaterial.find(materialIndex);
        const auto sourceWeights = geometry.weightsByMaterial.find(materialIndex);
        const std::vector<HumanVertexWeights> defaultWeights =
            sourceWeights == geometry.weightsByMaterial.end() &&
                    seatedWeights == geometry.seatedWeightsByMaterial.end()
                ? makeMainBoneWeights(vertices.size())
                : std::vector<HumanVertexWeights>{};
        const auto& standingWeights =
            sourceWeights != geometry.weightsByMaterial.end()
                ? sourceWeights->second
                : (seatedWeights != geometry.seatedWeightsByMaterial.end() ? seatedWeights->second
                                                                           : defaultWeights);
        const auto& selectedSeatedWeights = seatedWeights != geometry.seatedWeightsByMaterial.end()
                                                ? seatedWeights->second
                                                : standingWeights;

        DiligentHumanSubmesh submesh;
        submesh.materialIndex = materialIndex;
        for (const ModelVertex& vertex : vertices) {
            submesh.localCenter[0] += vertex.x;
            submesh.localCenter[1] += vertex.y;
            submesh.localCenter[2] += vertex.z;
        }
        const double inverseVertexCount = 1.0 / static_cast<double>(vertices.size());
        for (double& coordinate : submesh.localCenter) {
            coordinate *= inverseVertexCount;
        }
        submesh.mesh = std::make_unique<DiligentTexturedMesh>(device, vertices, standingWeights);
        if (selectedSeatedWeights != standingWeights) {
            submesh.seatedMesh =
                std::make_unique<DiligentTexturedMesh>(device, vertices, selectedSeatedWeights);
        }

        const auto texture = textures.find(materialIndex);
        if (texture != textures.end()) {
            const Image* image = texture->second.get();
            auto [uploaded, inserted] = uploadedTextures.try_emplace(image);
            if (inserted) {
                uploaded->second = std::make_shared<DiligentTexture2D>(device, *image);
            }
            submesh.texture = uploaded->second;
        }
        const auto material = materials.find(materialIndex);
        if (material != materials.end()) {
            submesh.material = material->second;
        }
        submeshes_.push_back(std::move(submesh));
    }
}

std::span<const DiligentHumanSubmesh> DiligentHumanAsset::submeshes() const noexcept {
    return submeshes_;
}

std::uint64_t DiligentHumanAsset::sourceGeneration() const noexcept {
    return sourceGeneration_;
}

bool DiligentHumanAsset::hasSubmeshesForPass(DiligentHumanRenderPass pass) const {
    if (!isValidHumanPass(pass)) {
        throw std::invalid_argument("Diligent human render pass is invalid");
    }
    return std::any_of(submeshes_.begin(), submeshes_.end(), [pass](const auto& submesh) {
        return matchesHumanPass(submesh.material, pass);
    });
}

std::vector<DiligentShadowCaster>
DiligentHumanAsset::shadowCasters(const Matrix4& modelViewProjection,
                                  const HumanBonePalette& currentPose, bool seated) const {
    std::vector<DiligentShadowCaster> casters;
    casters.reserve(submeshes_.size());
    for (const DiligentHumanSubmesh& submesh : submeshes_) {
        if (diligentMaterialBlends(submesh.material)) {
            continue;
        }
        float alphaCutoff = 0.0F;
        switch (submesh.material.alphaMode) {
        case DiligentAlphaMode::Opaque:
            break;
        case DiligentAlphaMode::Cutout:
            alphaCutoff = 0.5F;
            break;
        case DiligentAlphaMode::NearOpaque:
            alphaCutoff = 0.99F;
            break;
        case DiligentAlphaMode::Blend:
        case DiligentAlphaMode::NearOpaqueBlend:
            continue;
        }

        const DiligentTexturedMesh* mesh =
            seated && submesh.seatedMesh ? submesh.seatedMesh.get() : submesh.mesh.get();
        if (mesh == nullptr) {
            throw std::logic_error("Diligent human submesh has no shadow mesh");
        }
        Diligent::ITextureView* alphaTexture = alphaCutoff > 0.0F && submesh.texture
                                                   ? &submesh.texture->shaderResourceView()
                                                   : nullptr;
        if (alphaTexture == nullptr) {
            alphaCutoff = 0.0F;
        }
        casters.push_back({mesh,
                           modelViewProjection,
                           alphaTexture,
                           {1.0F, 1.0F, 0.0F, 0.0F},
                           alphaCutoff,
                           0,
                           &currentPose});
    }
    return casters;
}

void DiligentHumanAsset::draw(DiligentTexturedMeshRenderer& renderer,
                              Diligent::IDeviceContext& context,
                              Diligent::ITextureView& fallbackTexture,
                              const Matrix4& modelViewProjection, const Matrix4& modelView,
                              const HumanBonePalette& currentPose,
                              const HumanBonePalette* previousPose, bool seated,
                              const DiligentLightingParameters& lighting,
                              const DiligentMotionParameters* motion) const {
    (void)drawPass(renderer, context, fallbackTexture, modelViewProjection, modelView, currentPose,
                   previousPose, seated, DiligentHumanRenderPass::All, lighting, motion);
}

std::size_t DiligentHumanAsset::drawPass(
    DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
    Diligent::ITextureView& fallbackTexture, const Matrix4& modelViewProjection,
    const Matrix4& modelView, const HumanBonePalette& currentPose,
    const HumanBonePalette* previousPose, bool seated, DiligentHumanRenderPass pass,
    const DiligentLightingParameters& lighting, const DiligentMotionParameters* motion) const {
    if (!isValidHumanPass(pass)) {
        throw std::invalid_argument("Diligent human render pass is invalid");
    }

    std::vector<std::size_t> opaqueOrder;
    std::vector<std::size_t> transparentOrder;
    opaqueOrder.reserve(submeshes_.size());
    transparentOrder.reserve(submeshes_.size());
    for (std::size_t index = 0; index < submeshes_.size(); ++index) {
        const DiligentMaterialState& material = submeshes_[index].material;
        if (matchesHumanPass(material, pass)) {
            (isTransparentHumanMaterial(material) ? transparentOrder : opaqueOrder)
                .push_back(index);
        }
    }

    const auto viewDepth = [&](std::size_t index) {
        const auto& center = submeshes_[index].localCenter;
        const double depth = modelView[2] * center[0] + modelView[6] * center[1] +
                             modelView[10] * center[2] + modelView[14];
        if (!std::isfinite(depth)) {
            throw std::invalid_argument("Diligent human submesh has invalid view depth");
        }
        return depth;
    };
    std::vector<double> transparentDepths(submeshes_.size(), 0.0);
    for (const std::size_t index : transparentOrder) {
        transparentDepths[index] = viewDepth(index);
    }
    std::stable_sort(transparentOrder.begin(), transparentOrder.end(),
                     [&](std::size_t first, std::size_t second) {
                         return transparentDepths[first] < transparentDepths[second];
                     });

    const auto drawSubmesh = [&](std::size_t index) {
        submeshes_[index].draw(renderer, context, fallbackTexture, modelViewProjection, currentPose,
                               previousPose, seated, lighting, motion);
    };
    for (const std::size_t index : opaqueOrder) {
        drawSubmesh(index);
    }
    for (const std::size_t index : transparentOrder) {
        drawSubmesh(index);
    }
    return opaqueOrder.size() + transparentOrder.size();
}

} // namespace openbus::rendering
