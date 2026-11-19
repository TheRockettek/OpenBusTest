#include "render/diligent/DiligentObjModel.h"
#include "render/diligent/DiligentTexture2D.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace openbus::rendering {
namespace {

struct ObjVertexKey {
    int position = 0;
    int texCoord = 0;
    int normal = 0;
    std::size_t face = 0;

    bool operator==(const ObjVertexKey&) const = default;
};

struct ObjVertexKeyHash {
    std::size_t operator()(const ObjVertexKey& key) const noexcept {
        std::size_t seed = std::hash<int>{}(key.position);
        const auto combine = [&seed](std::size_t value) {
            seed ^= value + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        };
        combine(std::hash<int>{}(key.texCoord));
        combine(std::hash<int>{}(key.normal));
        combine(std::hash<std::size_t>{}(key.face));
        return seed;
    }
};

struct MeshBatch {
    std::string materialName;
    ObjMaterial material;
    std::vector<ModelVertex> vertices;
    std::vector<std::uint32_t> indices;
    std::unordered_map<ObjVertexKey, std::uint32_t, ObjVertexKeyHash> vertexLookup;
};

float toFiniteFloat(double value, const char* fieldName) {
    const float converted = static_cast<float>(value);
    if (!std::isfinite(value) || !std::isfinite(converted)) {
        throw std::invalid_argument(std::string("OBJ model has non-finite or out-of-range ") +
                                    fieldName);
    }
    return converted;
}

std::size_t checkedIndex(int oneBasedIndex, std::size_t count, const char* fieldName,
                         bool optional) {
    if (oneBasedIndex == 0 && optional) {
        return std::numeric_limits<std::size_t>::max();
    }
    if (oneBasedIndex <= 0 || static_cast<std::size_t>(oneBasedIndex) > count) {
        throw std::invalid_argument(std::string("OBJ model has invalid ") + fieldName + " index");
    }
    return static_cast<std::size_t>(oneBasedIndex - 1);
}

std::array<double, 3> faceNormal(const std::array<ObjPosition, 3>& positions) {
    const double edgeOneX = positions[1].x - positions[0].x;
    const double edgeOneY = positions[1].y - positions[0].y;
    const double edgeOneZ = positions[1].z - positions[0].z;
    const double edgeTwoX = positions[2].x - positions[0].x;
    const double edgeTwoY = positions[2].y - positions[0].y;
    const double edgeTwoZ = positions[2].z - positions[0].z;
    const double normalX = edgeOneY * edgeTwoZ - edgeOneZ * edgeTwoY;
    const double normalY = edgeOneZ * edgeTwoX - edgeOneX * edgeTwoZ;
    const double normalZ = edgeOneX * edgeTwoY - edgeOneY * edgeTwoX;
    const double length = std::hypot(normalX, normalY, normalZ);
    if (!std::isfinite(length) || length <= 1.0e-12) {
        return {};
    }
    return {normalX / length, normalY / length, normalZ / length};
}

ModelVertex makeVertex(const ParsedObj& model, const ObjIndex& index,
                       const std::array<double, 3>& generatedNormal,
                       const DiligentObjImportOptions& options) {
    const std::size_t positionIndex =
        checkedIndex(index.position, model.positions.size(), "position", false);
    const std::size_t texCoordIndex =
        checkedIndex(index.texCoord, model.texCoords.size(), "texture coordinate", true);
    const std::size_t normalIndex =
        checkedIndex(index.normal, model.normals.size(), "normal", true);
    const ObjPosition& position = model.positions[positionIndex];

    ModelVertex vertex{};
    const bool vehicleFrame = options.coordinateFrame == DiligentObjCoordinateFrame::Vehicle;
    vertex.x = toFiniteFloat(vehicleFrame ? position.y : position.x, "position");
    vertex.y = toFiniteFloat(vehicleFrame ? -position.x : position.y, "position");
    vertex.z = toFiniteFloat(position.z, "position");
    if (texCoordIndex != std::numeric_limits<std::size_t>::max()) {
        const ObjTexCoord& texCoord = model.texCoords[texCoordIndex];
        vertex.u = toFiniteFloat(texCoord.u, "texture coordinate");
        vertex.v = toFiniteFloat(options.flipTextureV ? 1.0 - texCoord.v : texCoord.v,
                                 "texture coordinate");
    }
    if (normalIndex != std::numeric_limits<std::size_t>::max()) {
        const ObjNormal& normal = model.normals[normalIndex];
        vertex.nx = toFiniteFloat(vehicleFrame ? normal.y : normal.x, "normal");
        vertex.ny = toFiniteFloat(vehicleFrame ? -normal.x : normal.y, "normal");
        vertex.nz = toFiniteFloat(normal.z, "normal");
    } else {
        vertex.nx = toFiniteFloat(vehicleFrame ? generatedNormal[1] : generatedNormal[0],
                                  "generated normal");
        vertex.ny = toFiniteFloat(vehicleFrame ? -generatedNormal[0] : generatedNormal[1],
                                  "generated normal");
        vertex.nz = toFiniteFloat(generatedNormal[2], "generated normal");
    }
    if (!model.vertexColors.empty()) {
        const ObjColor& color = model.vertexColors[positionIndex];
        vertex.r = toFiniteFloat(color[0], "vertex color");
        vertex.g = toFiniteFloat(color[1], "vertex color");
        vertex.b = toFiniteFloat(color[2], "vertex color");
        vertex.a = toFiniteFloat(color[3], "vertex alpha");
    }
    return vertex;
}

} // namespace

DiligentMaterialState DiligentObjSubmesh::materialState() const {
    DiligentMaterialState state;
    for (std::size_t channel = 0; channel < material.color.size(); ++channel) {
        state.baseColor[channel] = toFiniteFloat(material.color[channel], "material diffuse color");
        state.specularColor[channel] =
            toFiniteFloat(material.specular[channel], "material specular color");
        if (state.specularColor[channel] < 0.0F) {
            throw std::invalid_argument("OBJ material specular color cannot be negative");
        }
    }

    state.baseColor[3] = toFiniteFloat(material.alpha, "material alpha");
    if (state.baseColor[3] < 0.0F || state.baseColor[3] > 1.0F) {
        throw std::invalid_argument("OBJ material alpha must be between zero and one");
    }
    state.specularPower = toFiniteFloat(material.specularPower, "material specular power");
    if (state.specularPower <= 0.0F) {
        throw std::invalid_argument("OBJ material specular power must be positive");
    }
    if (state.baseColor[3] < 1.0F) {
        state.alphaMode = DiligentAlphaMode::Blend;
        state.depthWrite = false;
    }
    return state;
}

void DiligentObjSubmesh::draw(DiligentTexturedMeshRenderer& renderer,
                              Diligent::IDeviceContext& context,
                              Diligent::ITextureView& fallbackTexture,
                              const Matrix4& modelViewProjection,
                              const DiligentLightingParameters& lighting) const {
    if (!mesh) {
        throw std::logic_error("Cannot draw an OBJ submesh without a Diligent mesh");
    }
    Diligent::ITextureView& texture =
        baseTexture ? baseTexture->shaderResourceView() : fallbackTexture;
    renderer.draw(context, *mesh, texture, modelViewProjection, materialState(), lighting);
}

DiligentObjModel::DiligentObjModel(Diligent::IRenderDevice& device, const ParsedObj& model,
                                   const std::filesystem::path& textureRoot,
                                   DiligentObjImportOptions options)
    : hasTransform_(model.hasTransform), transform_(model.transform) {
    if (options.coordinateFrame != DiligentObjCoordinateFrame::Parsed &&
        options.coordinateFrame != DiligentObjCoordinateFrame::Vehicle) {
        throw std::invalid_argument("Invalid Diligent OBJ coordinate frame");
    }
    if (model.triangles.empty()) {
        throw std::invalid_argument("Diligent OBJ model requires at least one triangle");
    }
    if (!model.vertexColors.empty() && model.vertexColors.size() != model.positions.size()) {
        throw std::invalid_argument("OBJ vertex colors must match the position count");
    }

    std::vector<MeshBatch> batches;
    const std::size_t estimatedBatchCount =
        std::min(model.triangles.size(), std::max<std::size_t>(1, model.materials.size()));
    batches.reserve(estimatedBatchCount);
    std::unordered_map<std::string, std::size_t> batchLookup;
    batchLookup.reserve(estimatedBatchCount);
    for (std::size_t faceIndex = 0; faceIndex < model.triangles.size(); ++faceIndex) {
        const ObjTriangle& triangle = model.triangles[faceIndex];
        auto [batchIt, inserted] = batchLookup.try_emplace(triangle.material, batches.size());
        if (inserted) {
            MeshBatch batch;
            batch.materialName = triangle.material;
            const auto materialIt = model.materials.find(triangle.material);
            if (materialIt != model.materials.end()) {
                batch.material = materialIt->second;
            }
            batches.push_back(std::move(batch));
        }
        MeshBatch& batch = batches[batchIt->second];

        std::array<ObjPosition, 3> positions;
        for (std::size_t corner = 0; corner < triangle.indices.size(); ++corner) {
            const std::size_t positionIndex = checkedIndex(
                triangle.indices[corner].position, model.positions.size(), "position", false);
            positions[corner] = model.positions[positionIndex];
        }
        const auto generatedNormal = faceNormal(positions);
        for (const ObjIndex& objIndex : triangle.indices) {
            const std::size_t faceKey = objIndex.normal == 0 ? faceIndex : 0;
            const ObjVertexKey key{objIndex.position, objIndex.texCoord, objIndex.normal, faceKey};
            const auto vertexIt = batch.vertexLookup.find(key);
            if (vertexIt != batch.vertexLookup.end()) {
                batch.indices.push_back(vertexIt->second);
                continue;
            }
            if (batch.vertices.size() >= std::numeric_limits<std::uint32_t>::max()) {
                throw std::length_error("Diligent OBJ mesh exceeds 32-bit index capacity");
            }
            const auto vertexIndex = static_cast<std::uint32_t>(batch.vertices.size());
            batch.vertices.push_back(makeVertex(model, objIndex, generatedNormal, options));
            batch.vertexLookup.emplace(key, vertexIndex);
            batch.indices.push_back(vertexIndex);
        }
    }

    submeshes_.reserve(batches.size());
    std::unordered_map<std::filesystem::path, std::shared_ptr<DiligentTexture2D>> textureCache;
    textureCache.reserve(estimatedBatchCount);
    for (MeshBatch& batch : batches) {
        DiligentObjSubmesh submesh;
        submesh.materialName = std::move(batch.materialName);
        submesh.material = std::move(batch.material);
        const std::filesystem::path textureReference =
            !submesh.material.texturePath.empty()
                ? submesh.material.texturePath
                : std::filesystem::path(submesh.material.textureName);
        if (!textureRoot.empty() && !textureReference.empty()) {
            const std::filesystem::path texturePath =
                (textureReference.is_absolute() ? textureReference : textureRoot / textureReference)
                    .lexically_normal();
            auto textureIt = textureCache.find(texturePath);
            if (textureIt == textureCache.end()) {
                std::shared_ptr<DiligentTexture2D> texture =
                    DiligentTexture2D::load(device, texturePath);
                textureIt = textureCache.emplace(texturePath, std::move(texture)).first;
            }
            submesh.baseTexture = textureIt->second;
        }
        submesh.mesh =
            std::make_unique<DiligentTexturedMesh>(device, batch.vertices, batch.indices);
        submeshes_.push_back(std::move(submesh));
    }
}

DiligentObjModel::~DiligentObjModel() = default;

std::span<const DiligentObjSubmesh> DiligentObjModel::submeshes() const noexcept {
    return submeshes_;
}

bool DiligentObjModel::hasTransform() const noexcept {
    return hasTransform_;
}

const std::array<double, 16>& DiligentObjModel::transform() const noexcept {
    return transform_;
}

} // namespace openbus::rendering
