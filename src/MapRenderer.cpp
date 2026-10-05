#include "MapRenderer.h"

#include "CoreRenderer.h"
#include "MapSplineGeometry.h"
#include "ModelConfigLoader.h"
#include "O3DLoader.h"
#include "OpenGLFunctions.h"
#include "SceneryObjectConfigLoader.h"
#include "TextureLoader.h"
#include "Variables.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace openbus::rendering {

namespace {

constexpr int TERRAIN_TILE_RADIUS = 1;
constexpr double SCENERY_DRAW_DISTANCE = 320.0;
constexpr double ROAD_HALF_WIDTH_METERS = 3.25;

struct ModelVertex {
    float x;
    float y;
    float z;
    float u;
    float v;
    float layer;
    float nx;
    float ny;
    float nz;
};

static_assert(sizeof(ModelVertex) == 9 * sizeof(float));

struct TerrainVertex {
    float x;
    float y;
    float z;
    float u;
    float v;
    float layer;
    float nx;
    float ny;
    float nz;
};

static_assert(sizeof(TerrainVertex) == 9 * sizeof(float));

std::filesystem::path groundTexturePath(const openbus::map::MapDefinition& map,
                                        const std::filesystem::path& omsiRoot,
                                        const std::string& configuredPath) {
    std::string normalized = configuredPath;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    const std::filesystem::path relative(normalized);
    const std::array<std::filesystem::path, 2> candidates = {
        map.rootPath / relative, omsiRoot / relative};
    for (const std::filesystem::path& candidate : candidates) {
        if (std::filesystem::is_regular_file(candidate)) {
            return candidate;
        }
    }
    throw std::runtime_error("Could not resolve map ground texture: " + configuredPath);
}

GLuint loadTexture(const std::filesystem::path& path) {
    Image image;
    if (!TextureLoader::readImage(path, image)) {
        throw std::runtime_error("Could not decode map ground texture: " + path.string());
    }
    GLuint texture = 0;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.width, image.height, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, image.rgba.data());
    openbus::rendering::invalidateTextureBindings();
    if (glGetError() != GL_NO_ERROR) {
        glDeleteTextures(1, &texture);
        throw std::runtime_error("OpenGL failed to upload map ground texture: " + path.string());
    }
    return texture;
}

std::filesystem::path normalizedAssetPath(const std::string& value) {
    std::string normalized = value;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    return std::filesystem::path(normalized);
}

std::filesystem::path findSceneryTexture(const std::filesystem::path& objectRoot,
                                         const std::filesystem::path& modelRoot,
                                         const std::filesystem::path& omsiRoot,
                                         const std::string& textureName) {
    const std::filesystem::path relative = normalizedAssetPath(textureName);
    const std::array<std::filesystem::path, 5> candidates = {
        objectRoot / "texture" / relative, objectRoot / relative, modelRoot / relative,
        omsiRoot / relative, omsiRoot / "Texture" / relative};
    for (const std::filesystem::path& candidate : candidates) {
        if (std::filesystem::is_regular_file(candidate)) {
            return candidate;
        }
    }
    return {};
}

std::uint64_t tileKey(int x, int y) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
           static_cast<std::uint32_t>(y);
}

bool supportedPlacementRotation(const openbus::map::MapSceneryPlacement& object) {
    constexpr double tolerance = 1.0e-6;
    return object.transformValid &&
           std::abs(object.rotationDegrees[1]) <= tolerance &&
           std::abs(object.rotationDegrees[2]) <= tolerance;
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

void appendRoadSegment(std::vector<ModelVertex>& vertices,
                       const openbus::map::MapSplinePlacement& spline,
                       const std::vector<openbus::map::MapSplineSample>& centerline) {
    constexpr double profileHeight = 0.1;
    constexpr double textureRepeatPerMeter = 0.167;
    const auto heightAt = [&](double distance) {
        // The selected sample segments use values around -5. Treating these as
        // percent grades is a rendering approximation, not a format guarantee.
        const double t = std::clamp(distance / spline.length, 0.0, 1.0);
        const double averageGradient =
            spline.gradientStart + (spline.gradientEnd - spline.gradientStart) * t * 0.5;
        return spline.elevation + profileHeight + averageGradient * distance / 100.0;
    };
    const auto vertex = [](double x, double y, double z, float u, float v) {
        return ModelVertex{static_cast<float>(x), static_cast<float>(y),
                           static_cast<float>(z), u, v, 0.0F,
                           0.0F, 0.0F, 1.0F};
    };
    for (std::size_t index = 1; index < centerline.size(); ++index) {
        const openbus::map::MapSplineSample& start = centerline[index - 1];
        const openbus::map::MapSplineSample& end = centerline[index];
        const double startDirectionX = std::sin(start.headingRadians);
        const double startDirectionY = std::cos(start.headingRadians);
        const double endDirectionX = std::sin(end.headingRadians);
        const double endDirectionY = std::cos(end.headingRadians);
        const double startSideX = -startDirectionY;
        const double startSideY = startDirectionX;
        const double endSideX = -endDirectionY;
        const double endSideY = endDirectionX;
        const float startV = static_cast<float>(start.distance * textureRepeatPerMeter);
        const float endV = static_cast<float>(end.distance * textureRepeatPerMeter);
        const ModelVertex startLeft = vertex(
            start.x - ROAD_HALF_WIDTH_METERS * startSideX,
            start.y - ROAD_HALF_WIDTH_METERS * startSideY,
            heightAt(start.distance), 0.995F, startV);
        const ModelVertex endLeft = vertex(
            end.x - ROAD_HALF_WIDTH_METERS * endSideX,
            end.y - ROAD_HALF_WIDTH_METERS * endSideY,
            heightAt(end.distance), 0.995F, endV);
        const ModelVertex endRight = vertex(
            end.x + ROAD_HALF_WIDTH_METERS * endSideX,
            end.y + ROAD_HALF_WIDTH_METERS * endSideY,
            heightAt(end.distance), 0.005F, endV);
        const ModelVertex startRight = vertex(
            start.x + ROAD_HALF_WIDTH_METERS * startSideX,
            start.y + ROAD_HALF_WIDTH_METERS * startSideY,
            heightAt(start.distance), 0.005F, startV);
        vertices.insert(vertices.end(), {startLeft, endLeft, endRight,
                                         startLeft, endRight, startRight});
    }
}

} // namespace

struct MapRenderer::Impl {
    struct TileBuffer {
        GLuint buffer = 0;
        std::size_t vertexCount = 0;
    };

    struct ModelBatch {
        GLuint buffer = 0;
        std::size_t vertexCount = 0;
        ModelMaterial material;
        std::array<double, 3> color = {1.0, 1.0, 1.0};
        int alphaMode = 0;
    };

    struct SceneryInstance {
        std::size_t batchIndex = 0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double yawDegrees = 0.0;
    };

    GLuint texture = 0;
    std::vector<TileBuffer> terrainBuffers;
    std::vector<ModelBatch> sceneryBatches;
    std::vector<SceneryInstance> sceneryInstances;
    std::vector<ModelBatch> roadBatches;
    std::vector<GLuint> ownedTextures;

    ~Impl() {
        for (const TileBuffer& tile : terrainBuffers) {
            if (tile.buffer != 0 && pglDeleteBuffers != nullptr) {
                pglDeleteBuffers(1, &tile.buffer);
            }
        }
        for (const ModelBatch& batch : sceneryBatches) {
            if (batch.buffer != 0 && pglDeleteBuffers != nullptr) {
                pglDeleteBuffers(1, &batch.buffer);
            }
        }
        for (const ModelBatch& batch : roadBatches) {
            if (batch.buffer != 0 && pglDeleteBuffers != nullptr) {
                pglDeleteBuffers(1, &batch.buffer);
            }
        }
        for (const GLuint ownedTexture : ownedTextures) {
            if (ownedTexture != 0) {
                glDeleteTextures(1, &ownedTexture);
            }
        }
        if (texture != 0) {
            glDeleteTextures(1, &texture);
        }
    }
};

MapRenderer::MapRenderer(const openbus::map::MapDefinition& map, std::size_t centerTileIndex,
                         const std::filesystem::path& omsiRoot)
    : impl_(std::make_unique<Impl>()) {
    if (map.groundTextures.empty()) {
        throw std::runtime_error("Map has no [groundtex] records");
    }
    if (centerTileIndex >= map.tiles.size()) {
        throw std::runtime_error("Map spawn references an invalid tile index");
    }
    if (pglGenBuffers == nullptr || pglBindBuffer == nullptr || pglBufferData == nullptr) {
        throw std::runtime_error("OpenGL buffer functions are unavailable for map terrain");
    }

    const std::filesystem::path texturePath =
        groundTexturePath(map, omsiRoot, map.groundTextures.front().texturePath);
    impl_->texture = loadTexture(texturePath);

    std::unordered_map<std::uint64_t, const openbus::map::MapTileReference*> tileByCoordinate;
    std::unordered_map<std::uint64_t, openbus::map::TerrainGrid> terrainByCoordinate;
    tileByCoordinate.reserve(map.tiles.size());
    for (const openbus::map::MapTileReference& tile : map.tiles) {
        tileByCoordinate.emplace(tileKey(tile.x, tile.y), &tile);
    }
    const openbus::map::MapTileReference& center = map.tiles[centerTileIndex];
    for (int tileY = center.y - TERRAIN_TILE_RADIUS; tileY <= center.y + TERRAIN_TILE_RADIUS;
         ++tileY) {
        for (int tileX = center.x - TERRAIN_TILE_RADIUS; tileX <= center.x + TERRAIN_TILE_RADIUS;
             ++tileX) {
            const std::uint64_t key = tileKey(tileX, tileY);
            const auto found = tileByCoordinate.find(key);
            if (found == tileByCoordinate.end() || !found->second->hasTerrainFile) {
                continue;
            }
            const openbus::map::TerrainGrid grid =
                openbus::map::loadTerrainGrid(found->second->terrainPath);
            terrainByCoordinate.emplace(key, grid);
            const std::size_t side = grid.intervals + 1;
            const double step = openbus::map::OMSI_TILE_SIZE_METERS / grid.intervals;
            std::vector<TerrainVertex> vertices;
            vertices.reserve(grid.intervals * grid.intervals * 6);
            const auto makeVertex = [&](std::size_t row, std::size_t column) {
                const double x = static_cast<double>(tileX) * openbus::map::OMSI_TILE_SIZE_METERS +
                                 static_cast<double>(column) * step;
                const double y = static_cast<double>(tileY) * openbus::map::OMSI_TILE_SIZE_METERS +
                                 static_cast<double>(row) * step;
                const std::size_t index = row * side + column;
                return TerrainVertex{static_cast<float>(x), static_cast<float>(y),
                                     grid.heights[index], static_cast<float>(column),
                                     static_cast<float>(row), 0.0F, 0.0F, 0.0F, 1.0F};
            };
            for (std::size_t row = 0; row < grid.intervals; ++row) {
                for (std::size_t column = 0; column < grid.intervals; ++column) {
                    const TerrainVertex topLeft = makeVertex(row, column);
                    const TerrainVertex topRight = makeVertex(row, column + 1);
                    const TerrainVertex bottomRight = makeVertex(row + 1, column + 1);
                    const TerrainVertex bottomLeft = makeVertex(row + 1, column);
                    vertices.insert(vertices.end(), {topLeft, topRight, bottomRight,
                                                     topLeft, bottomRight, bottomLeft});
                }
            }
            Impl::TileBuffer buffer;
            buffer.vertexCount = vertices.size();
            pglGenBuffers(1, &buffer.buffer);
            pglBindBuffer(GL_ARRAY_BUFFER, buffer.buffer);
            pglBufferData(GL_ARRAY_BUFFER,
                          static_cast<std::ptrdiff_t>(vertices.size() * sizeof(TerrainVertex)),
                          vertices.data(), GL_STATIC_DRAW);
            if (buffer.buffer == 0 || glGetError() != GL_NO_ERROR) {
                if (buffer.buffer != 0) {
                    pglDeleteBuffers(1, &buffer.buffer);
                }
                throw std::runtime_error("Could not upload terrain mesh for " +
                                         found->second->textPath.filename().string());
            }
            impl_->terrainBuffers.push_back(buffer);
        }
    }
    if (impl_->terrainBuffers.empty()) {
        throw std::runtime_error("No terrain files found around the selected map entrypoint");
    }

    std::unordered_map<std::string, GLuint> textureCache;
    const auto getTexture = [&](const std::filesystem::path& path) {
        const std::string key = path.lexically_normal().generic_string();
        const auto found = textureCache.find(key);
        if (found != textureCache.end()) {
            return found->second;
        }
        const GLuint texture = loadTexture(path);
        textureCache.emplace(key, texture);
        impl_->ownedTextures.push_back(texture);
        return texture;
    };

    const auto sampleTerrainHeight = [&](int tileX, int tileY, double localX,
                                         double localY) -> std::optional<double> {
        const auto found = terrainByCoordinate.find(tileKey(tileX, tileY));
        if (found == terrainByCoordinate.end() || found->second.intervals == 0) {
            return std::nullopt;
        }
        const openbus::map::TerrainGrid& grid = found->second;
        const std::size_t side = grid.intervals + 1;
        const double gridX = std::clamp(localX / openbus::map::OMSI_TILE_SIZE_METERS *
                                            static_cast<double>(grid.intervals),
                                        0.0, static_cast<double>(grid.intervals));
        const double gridY = std::clamp(localY / openbus::map::OMSI_TILE_SIZE_METERS *
                                            static_cast<double>(grid.intervals),
                                        0.0, static_cast<double>(grid.intervals));
        const std::size_t column = static_cast<std::size_t>(gridX);
        const std::size_t row = static_cast<std::size_t>(gridY);
        const std::size_t nextColumn = std::min(column + 1, grid.intervals);
        const std::size_t nextRow = std::min(row + 1, grid.intervals);
        const double fractionX = gridX - static_cast<double>(column);
        const double fractionY = gridY - static_cast<double>(row);
        const auto heightAt = [&](std::size_t sampleRow, std::size_t sampleColumn) {
            return static_cast<double>(grid.heights[sampleRow * side + sampleColumn]);
        };
        const double lower = std::lerp(heightAt(row, column), heightAt(row, nextColumn), fractionX);
        const double upper = std::lerp(heightAt(nextRow, column),
                                       heightAt(nextRow, nextColumn), fractionX);
        return std::lerp(lower, upper, fractionY);
    };

    std::unordered_map<std::string, std::vector<std::size_t>> sceneryAssetBatches;
    const auto loadSceneryAsset = [&](const std::filesystem::path& configPath)
        -> const std::vector<std::size_t>& {
        const std::string cacheKey = configPath.lexically_normal().generic_string();
        const auto cached = sceneryAssetBatches.find(cacheKey);
        if (cached != sceneryAssetBatches.end()) {
            return cached->second;
        }
        std::vector<std::size_t> batchIndexes;
        try {
            const std::filesystem::path objectRoot = configPath.parent_path();
            const std::filesystem::path modelRoot = objectRoot / "model";
            const SceneryObjectConfig objectConfiguration = loadSceneryObjectFile(configPath);
            if (!objectConfiguration.trees.empty()) {
                for (const SceneryTreeDefinition& tree : objectConfiguration.trees) {
                    const std::filesystem::path treeTexture =
                        findSceneryTexture(objectRoot, modelRoot, omsiRoot, tree.texturePath);
                    if (treeTexture.empty()) {
                        continue;
                    }
                    const double height = (tree.minimumHeight + tree.maximumHeight) * 0.5;
                    const double ratio = (tree.minimumRatio + tree.maximumRatio) * 0.5;
                    std::vector<ModelVertex> vertices = makeTreePreview(height, height * 0.55 * ratio);
                    Impl::ModelBatch batch;
                    batch.vertexCount = vertices.size();
                    batch.material.texture = getTexture(treeTexture);
                    batch.material.textured = true;
                    batch.material.flipTextureY = true;
                    batch.alphaMode = 1;
                    pglGenBuffers(1, &batch.buffer);
                    pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
                    pglBufferData(
                        GL_ARRAY_BUFFER,
                        static_cast<std::ptrdiff_t>(vertices.size() * sizeof(ModelVertex)),
                        vertices.data(), GL_STATIC_DRAW);
                    if (batch.buffer == 0 || glGetError() != GL_NO_ERROR) {
                        if (batch.buffer != 0) {
                            pglDeleteBuffers(1, &batch.buffer);
                        }
                        continue;
                    }
                    batchIndexes.push_back(impl_->sceneryBatches.size());
                    impl_->sceneryBatches.push_back(batch);
                }
                return sceneryAssetBatches.emplace(cacheKey, std::move(batchIndexes))
                    .first->second;
            }
            openbus::scripting::SceneryObject variables;
            const ModelConfig configuration =
                loadModelConfig(configPath, modelRoot, ModelConfigKind::SceneryObject, variables);
            for (const ModelPart& part : configuration.parts) {
                if (part.lodIndex > 0 || part.objPath.empty() ||
                    part.objPath.extension() != ".o3d") {
                    continue;
                }
                const std::shared_ptr<ParsedObj> parsed = O3DLoader::parse(part.objPath);
                if (!parsed) {
                    continue;
                }
                std::unordered_map<int, std::vector<ModelVertex>> verticesByMaterial;
                std::unordered_map<int, const ModelMaterialState*> statesByMaterial;
                for (const ModelMaterialState& state : part.materialStatesInOrder) {
                    statesByMaterial.emplace(state.materialIndex, &state);
                }
                for (const ObjTriangle& triangle : parsed->triangles) {
                    const auto sourceMaterial = parsed->materials.find(triangle.material);
                    const int materialIndex = sourceMaterial == parsed->materials.end()
                                                  ? 0
                                                  : sourceMaterial->second.materialIndex;
                    const auto validIndex = [](int index, std::size_t count) {
                        return index > 0 && static_cast<std::size_t>(index) <= count;
                    };
                    for (const ObjIndex& index : triangle.indices) {
                        if (!validIndex(index.position, parsed->positions.size())) {
                            continue;
                        }
                        const ObjPosition& sourcePosition =
                            parsed->positions[static_cast<std::size_t>(index.position - 1)];
                        std::array<double, 3> normal = {0.0, 0.0, 1.0};
                        if (validIndex(index.normal, parsed->normals.size())) {
                            const ObjNormal& sourceNormal =
                                parsed->normals[static_cast<std::size_t>(index.normal - 1)];
                            normal = {sourceNormal.y, -sourceNormal.x, sourceNormal.z};
                            const double length = std::sqrt(normal[0] * normal[0] +
                                                            normal[1] * normal[1] +
                                                            normal[2] * normal[2]);
                            if (length > 1.0e-8) {
                                for (double& component : normal) {
                                    component /= length;
                                }
                            }
                        }
                        float u = 0.0F;
                        float v = 0.0F;
                        if (validIndex(index.texCoord, parsed->texCoords.size())) {
                            const ObjTexCoord& coordinate =
                                parsed->texCoords[static_cast<std::size_t>(index.texCoord - 1)];
                            u = static_cast<float>(coordinate.u);
                            v = static_cast<float>(1.0 - coordinate.v);
                        }
                        verticesByMaterial[materialIndex].push_back(
                            {static_cast<float>(sourcePosition.y),
                             static_cast<float>(-sourcePosition.x),
                             static_cast<float>(sourcePosition.z), u, v, 0.0F,
                             static_cast<float>(normal[0]), static_cast<float>(normal[1]),
                             static_cast<float>(normal[2])});
                    }
                }

                for (auto& [materialIndex, vertices] : verticesByMaterial) {
                    if (vertices.empty()) {
                        continue;
                    }
                    const auto sourceMaterial = parsed->materials.find("matl_" +
                                                                        std::to_string(materialIndex));
                    const auto stateFound = statesByMaterial.find(materialIndex);
                    const ModelMaterialState* state =
                        stateFound == statesByMaterial.end() ? nullptr : stateFound->second;
                    std::string textureName = sourceMaterial == parsed->materials.end()
                                                  ? std::string{}
                                                  : sourceMaterial->second.textureName;
                    std::array<double, 3> color = sourceMaterial == parsed->materials.end()
                                                      ? std::array<double, 3>{1.0, 1.0, 1.0}
                                                      : sourceMaterial->second.color;
                    if (state != nullptr && !state->textureName.empty()) {
                        textureName = state->textureName;
                    }
                    if (state != nullptr && !state->textureName.empty()) {
                        const std::string stateTexture =
                            std::filesystem::path(state->textureName).filename().string();
                        const auto materialForTexture = std::find_if(
                            parsed->materials.begin(), parsed->materials.end(),
                            [&](const auto& entry) {
                                return std::filesystem::path(entry.second.textureName)
                                           .filename() == stateTexture;
                            });
                        if (materialForTexture != parsed->materials.end()) {
                            color = materialForTexture->second.color;
                        }
                    }
                    ModelMaterial material;
                    const std::filesystem::path materialTexture =
                        findSceneryTexture(objectRoot, modelRoot, omsiRoot, textureName);
                    if (!materialTexture.empty()) {
                        material.texture = getTexture(materialTexture);
                        material.textured = true;
                    }
                    Impl::ModelBatch batch;
                    batch.vertexCount = vertices.size();
                    batch.material = material;
                    batch.color = color;
                    batch.alphaMode = state == nullptr ? 0 : state->alphaMode;
                    pglGenBuffers(1, &batch.buffer);
                    pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
                    pglBufferData(GL_ARRAY_BUFFER,
                                  static_cast<std::ptrdiff_t>(vertices.size() * sizeof(ModelVertex)),
                                  vertices.data(), GL_STATIC_DRAW);
                    if (batch.buffer == 0 || glGetError() != GL_NO_ERROR) {
                        if (batch.buffer != 0) {
                            pglDeleteBuffers(1, &batch.buffer);
                        }
                        continue;
                    }
                    batchIndexes.push_back(impl_->sceneryBatches.size());
                    impl_->sceneryBatches.push_back(batch);
                }
            }
        } catch (const std::exception&) {
            // Unsupported scenery configuration syntax/assets are skipped; they must not prevent
            // the map terrain and supported static scenery from rendering.
        }
        return sceneryAssetBatches.emplace(cacheKey, std::move(batchIndexes)).first->second;
    };

    std::unordered_map<std::string, std::vector<ModelVertex>> roadVertices;
    const std::array<std::pair<std::string, std::string>, 2> supportedRoadProfiles = {{
        {"splines/freyfurt/6,5meter_gw0_asphalt.sli", "str_asphdrk.bmp"},
        {"splines/freyfurt/6,5meter_gw0_asphalt_line.sli", "str_asphdrk_1line.bmp"},
    }};
    const double spawnX = map.entryPoints.empty() ? 0.0 :
                          map.entryPoints.front().placement.position[0];
    const double spawnY = map.entryPoints.empty() ? 0.0 :
                          map.entryPoints.front().placement.position[1];
    for (int tileY = center.y - TERRAIN_TILE_RADIUS; tileY <= center.y + TERRAIN_TILE_RADIUS;
         ++tileY) {
        for (int tileX = center.x - TERRAIN_TILE_RADIUS; tileX <= center.x + TERRAIN_TILE_RADIUS;
             ++tileX) {
            const auto tileFound = tileByCoordinate.find(tileKey(tileX, tileY));
            if (tileFound == tileByCoordinate.end()) {
                continue;
            }
            const openbus::map::MapTileData tile = openbus::map::loadMapTile(*tileFound->second);
            for (const openbus::map::MapSceneryPlacement& object : tile.sceneryObjects) {
                if (!supportedPlacementRotation(object)) {
                    continue;
                }
                const double worldX = static_cast<double>(tileX) *
                                          openbus::map::OMSI_TILE_SIZE_METERS +
                                      object.localPosition[0];
                const double worldY = static_cast<double>(tileY) *
                                          openbus::map::OMSI_TILE_SIZE_METERS +
                                      object.localPosition[1];
                const double dx = worldX - spawnX;
                const double dy = worldY - spawnY;
                if (dx * dx + dy * dy > SCENERY_DRAW_DISTANCE * SCENERY_DRAW_DISTANCE) {
                    continue;
                }
                const std::optional<double> ground =
                    sampleTerrainHeight(tileX, tileY, object.localPosition[0],
                                        object.localPosition[1]);
                if (!ground) {
                    continue;
                }
                const std::filesystem::path relative = normalizedAssetPath(object.assetPath);
                const std::array<std::filesystem::path, 2> candidates = {
                    omsiRoot / relative, map.rootPath / relative};
                std::filesystem::path configPath;
                for (const std::filesystem::path& candidate : candidates) {
                    if (std::filesystem::is_regular_file(candidate)) {
                        configPath = candidate;
                        break;
                    }
                }
                if (configPath.empty()) {
                    continue;
                }
                for (const std::size_t batchIndex : loadSceneryAsset(configPath)) {
                    impl_->sceneryInstances.push_back(
                        {batchIndex, worldX, worldY, *ground + object.localPosition[2],
                         object.rotationDegrees[0]});
                }
            }
            for (const openbus::map::MapSplinePlacement& spline : tile.splines) {
                if (!spline.geometryValid || std::abs(spline.gradientStart) > 10.0 ||
                    std::abs(spline.gradientEnd) > 10.0 || spline.length <= 0.0 ||
                    spline.length > 60.0) {
                    continue;
                }
                std::string splinePath = normalizedAssetPath(spline.assetPath).generic_string();
                std::transform(splinePath.begin(), splinePath.end(), splinePath.begin(),
                               [](unsigned char character) {
                                   return static_cast<char>(std::tolower(character));
                               });
                    const auto profile = std::find_if(
                        supportedRoadProfiles.begin(), supportedRoadProfiles.end(),
                        [&](const auto& candidate) { return splinePath == candidate.first; });
                    if (profile == supportedRoadProfiles.end()) {
                        continue;
                    }
                    const double startX = static_cast<double>(tileX) *
                                              openbus::map::OMSI_TILE_SIZE_METERS +
                                          spline.localX;
                    const double startY = static_cast<double>(tileY) *
                                              openbus::map::OMSI_TILE_SIZE_METERS +
                                          spline.localY;
                    const std::vector<openbus::map::MapSplineSample> centerline =
                        openbus::map::tessellateMapSpline(startX, startY, spline.rotationDegrees,
                                                           spline.length, spline.radius);
                    if (centerline.size() < 2) {
                        continue;
                    }
                    const double cullingDistance = SCENERY_DRAW_DISTANCE + spline.length * 0.5 +
                                                   ROAD_HALF_WIDTH_METERS;
                    const double cullingDistanceSquared = cullingDistance * cullingDistance;
                    const bool intersectsDrawDistance = std::any_of(
                        centerline.begin(), centerline.end(), [&](const auto& sample) {
                            const double dx = sample.x - spawnX;
                            const double dy = sample.y - spawnY;
                            return dx * dx + dy * dy <= cullingDistanceSquared;
                        });
                    if (!intersectsDrawDistance) {
                        continue;
                }
                    const std::string texturePath =
                        (omsiRoot / "Splines" / "Freyfurt" / "texture" / profile->second)
                            .lexically_normal().generic_string();
                    appendRoadSegment(roadVertices[texturePath], spline, centerline);
            }
        }
    }

    for (auto& [texturePath, vertices] : roadVertices) {
        const std::filesystem::path path(texturePath);
        if (vertices.empty() || !std::filesystem::is_regular_file(path)) {
            continue;
        }
        Impl::ModelBatch batch;
        batch.vertexCount = vertices.size();
        batch.material.texture = getTexture(path);
        batch.material.textured = true;
        pglGenBuffers(1, &batch.buffer);
        pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
        pglBufferData(GL_ARRAY_BUFFER,
                      static_cast<std::ptrdiff_t>(vertices.size() * sizeof(ModelVertex)),
                      vertices.data(), GL_STATIC_DRAW);
        if (batch.buffer == 0 || glGetError() != GL_NO_ERROR) {
            if (batch.buffer != 0) {
                pglDeleteBuffers(1, &batch.buffer);
            }
            continue;
        }
        impl_->roadBatches.push_back(batch);
    }
}

MapRenderer::~MapRenderer() = default;

void MapRenderer::draw() const {
    ModelMaterial material;
    material.texture = impl_->texture;
    material.textured = true;
    for (const Impl::TileBuffer& tile : impl_->terrainBuffers) {
        drawModelBatch(tile.buffer, tile.vertexCount, material, {1.0, 1.0, 1.0}, 1.0, 0);
    }
    for (const Impl::ModelBatch& road : impl_->roadBatches) {
        drawModelBatch(road.buffer, road.vertexCount, road.material, road.color, 1.0,
                       road.alphaMode);
    }
    for (const Impl::SceneryInstance& instance : impl_->sceneryInstances) {
        if (instance.batchIndex >= impl_->sceneryBatches.size()) {
            continue;
        }
        const Impl::ModelBatch& batch = impl_->sceneryBatches[instance.batchIndex];
        const bool blended = batch.alphaMode == 2 || batch.alphaMode == 3;
        if (blended) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);
        }
        pushMatrix();
        translate(instance.x, instance.y, instance.z);
        rotate(instance.yawDegrees, 0.0, 0.0, 1.0);
        drawModelBatch(batch.buffer, batch.vertexCount, batch.material, batch.color, 1.0,
                       batch.alphaMode);
        popMatrix();
        if (blended) {
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
        }
    }
}

} // namespace openbus::rendering