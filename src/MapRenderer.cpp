#include "MapRenderer.h"

#include "CoreRenderer.h"
#include "Logger.h"
#include "MapSceneryPlacement.h"
#include "MapSplineGeometry.h"
#include "MapSplineProfile.h"
#include "ModelConfigLoader.h"
#include "O3DLoader.h"
#include "OpenGLFunctions.h"
#include "PerfTrace.h"
#include "SceneryObjectConfigLoader.h"
#include "TextureLoader.h"
#include "Variables.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

extern Logger gameLog;

namespace openbus::rendering {

namespace {

constexpr std::size_t MAX_SKIP_EXAMPLES = 3;
constexpr double MAP_VISIBILITY_RADIUS_METERS = 500.0;
constexpr double MAP_VISIBILITY_RADIUS_SQUARED =
    MAP_VISIBILITY_RADIUS_METERS * MAP_VISIBILITY_RADIUS_METERS;

double squaredDistanceToBounds(double x, double y, double minX, double minY, double maxX,
                               double maxY) {
    const double dx = x < minX ? minX - x : (x > maxX ? x - maxX : 0.0);
    const double dy = y < minY ? minY - y : (y > maxY ? y - maxY : 0.0);
    return dx * dx + dy * dy;
}

struct SkipSummary {
    std::size_t count = 0;
    std::vector<std::string> examples;
};

using SkipSummaries = std::map<std::string, SkipSummary>;

long long elapsedMilliseconds(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                 start)
        .count();
}

std::string tileLabel(int x, int y) {
    return "tile(" + std::to_string(x) + "," + std::to_string(y) + ")";
}

void recordSkipped(SkipSummaries& summaries, const std::string& reason, const std::string& example,
                   std::size_t count = 1) {
    SkipSummary& summary = summaries[reason];
    summary.count += count;
    if (!example.empty() && summary.examples.size() < MAX_SKIP_EXAMPLES) {
        summary.examples.push_back(example);
    }
}

void logSkipped(const SkipSummaries& summaries) {
    for (const auto& [reason, summary] : summaries) {
        std::string message = "Skipped " + std::to_string(summary.count) + " " + reason;
        if (!summary.examples.empty()) {
            message += " (examples: ";
            for (std::size_t index = 0; index < summary.examples.size(); ++index) {
                if (index != 0) {
                    message += "; ";
                }
                message += summary.examples[index];
            }
            message += ")";
        }
        gameLog.Log(message);
    }
}

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

struct RoadMeshGeometry {
    std::vector<ModelVertex> vertices;
    std::vector<std::uint32_t> indices;
    std::filesystem::path texturePath;
    double minX = std::numeric_limits<double>::infinity();
    double minY = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();
    bool textured = false;
};

std::filesystem::path groundTexturePath(const openbus::map::MapDefinition& map,
                                        const std::filesystem::path& omsiRoot,
                                        const std::string& configuredPath) {
    std::string normalized = configuredPath;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    const std::filesystem::path relative(normalized);
    const std::array<std::filesystem::path, 2> candidates = {map.rootPath / relative,
                                                             omsiRoot / relative};
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
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.width, image.height, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 image.rgba.data());
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

std::filesystem::path resolveSplineProfile(const openbus::map::MapDefinition& map,
                                           const std::filesystem::path& omsiRoot,
                                           const std::string& assetPath) {
    const std::filesystem::path relative = normalizedAssetPath(assetPath);
    const std::array<std::filesystem::path, 2> candidates = {omsiRoot / relative,
                                                             map.rootPath / relative};
    for (const std::filesystem::path& candidate : candidates) {
        if (std::filesystem::is_regular_file(candidate)) {
            return candidate;
        }
    }
    return {};
}

std::filesystem::path findSplineTexture(const std::filesystem::path& profilePath,
                                        const std::filesystem::path& omsiRoot,
                                        const std::string& textureName) {
    const std::filesystem::path relative = normalizedAssetPath(textureName);
    const std::filesystem::path profileRoot = profilePath.parent_path();
    const std::array<std::filesystem::path, 5> candidates = {
        profileRoot / "texture" / relative, profileRoot / relative, omsiRoot / "Texture" / relative,
        omsiRoot / "texture" / relative, omsiRoot / relative};
    for (const std::filesystem::path& candidate : candidates) {
        if (std::filesystem::is_regular_file(candidate)) {
            return candidate;
        }
    }
    return {};
}

void appendRoadProfile(RoadMeshGeometry& road, const openbus::map::MapSplinePlacement& spline,
                       const std::vector<openbus::map::MapSplineSample>& centerline,
                       const openbus::map::MapSplineProfileSection& section) {
    const auto heightAt = [&](double distance) {
        return openbus::map::mapSplineElevation(
            spline.elevation, spline.length, spline.gradientStart, spline.gradientEnd,
            spline.elevated ? std::optional<double>(spline.heightDelta) : std::nullopt, distance);
    };
    const auto vertex = [&](const openbus::map::MapSplineSample& sample,
                            const openbus::map::MapSplineProfilePoint& point) {
        const auto lateral =
            openbus::map::mapSplineRightOffset(sample.headingRadians, point.lateral);
        return ModelVertex{static_cast<float>(sample.x + lateral[0]),
                           static_cast<float>(sample.y + lateral[1]),
                           static_cast<float>(heightAt(sample.distance) + point.height),
                           static_cast<float>(point.textureU),
                           static_cast<float>(sample.distance * point.textureVPerMeter),
                           0.0F,
                           0.0F,
                           0.0F,
                           1.0F};
    };
    const std::uint32_t baseVertex = static_cast<std::uint32_t>(road.vertices.size());
    for (const openbus::map::MapSplineSample& sample : centerline) {
        for (const openbus::map::MapSplineProfilePoint& point : section.points) {
            const ModelVertex generated = vertex(sample, point);
            road.minX = std::min(road.minX, static_cast<double>(generated.x));
            road.minY = std::min(road.minY, static_cast<double>(generated.y));
            road.maxX = std::max(road.maxX, static_cast<double>(generated.x));
            road.maxY = std::max(road.maxY, static_cast<double>(generated.y));
            road.vertices.push_back(generated);
        }
    }
    const std::uint32_t pointsPerSample = static_cast<std::uint32_t>(section.points.size());
    for (std::size_t sampleIndex = 1; sampleIndex < centerline.size(); ++sampleIndex) {
        const std::uint32_t startBase =
            baseVertex + static_cast<std::uint32_t>(sampleIndex - 1) * pointsPerSample;
        const std::uint32_t endBase = startBase + pointsPerSample;
        for (std::uint32_t pointIndex = 1; pointIndex < pointsPerSample; ++pointIndex) {
            const std::uint32_t startLeft = startBase + pointIndex - 1;
            const std::uint32_t endLeft = endBase + pointIndex - 1;
            const std::uint32_t endRight = endBase + pointIndex;
            const std::uint32_t startRight = startBase + pointIndex;
            road.indices.insert(road.indices.end(),
                                {startLeft, endLeft, endRight, startLeft, endRight, startRight});
        }
    }
}

} // namespace

struct MapRenderer::Impl {
    struct TileBuffer {
        GLuint buffer = 0;
        GLuint indexBuffer = 0;
        std::size_t indexCount = 0;
        int tileX = 0;
        int tileY = 0;
    };

    struct ModelBatch {
        GLuint buffer = 0;
        GLuint indexBuffer = 0;
        std::size_t vertexCount = 0;
        std::size_t indexCount = 0;
        ModelMaterial material;
        std::array<double, 3> color = {1.0, 1.0, 1.0};
        int alphaMode = 0;
        bool hasVisibilityBounds = false;
        double minX = 0.0;
        double minY = 0.0;
        double maxX = 0.0;
        double maxY = 0.0;
    };

    struct SceneryInstance {
        std::size_t batchIndex = 0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        std::array<double, 6> rotationDegrees = {};
        std::size_t rotationCount = 3;
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
            if (tile.indexBuffer != 0 && pglDeleteBuffers != nullptr) {
                pglDeleteBuffers(1, &tile.indexBuffer);
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
            if (batch.indexBuffer != 0 && pglDeleteBuffers != nullptr) {
                pglDeleteBuffers(1, &batch.indexBuffer);
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
    TraceScope trace("map", "MapRenderer::MapRenderer");
    const auto mapLoadStart = std::chrono::steady_clock::now();
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
    {
        TraceScope phase("map", "MapRenderer.loadGroundTexture");
        impl_->texture = loadTexture(texturePath);
    }

    std::unordered_map<std::uint64_t, openbus::map::TerrainGrid> terrainByCoordinate;
    terrainByCoordinate.reserve(map.tiles.size());
    const openbus::map::MapTileReference& center = map.tiles[centerTileIndex];
    SkipSummaries skipped;
    gameLog.Log("Map loading started: root=" + map.rootPath.generic_string() +
                ", tiles=" + std::to_string(map.tiles.size()) +
                ", full-map loading enabled, render visibility=" +
                std::to_string(static_cast<int>(MAP_VISIBILITY_RADIUS_METERS)) +
                " m, spawn=" + tileLabel(center.x, center.y) + ".");
    const auto terrainLoadStart = std::chrono::steady_clock::now();
    std::size_t terrainTilesVisited = 0;
    {
        TraceScope phase("map", "MapRenderer.loadAllTerrainTiles");
        for (const openbus::map::MapTileReference& tile : map.tiles) {
            ++terrainTilesVisited;
            if (terrainTilesVisited % 100 == 0 || terrainTilesVisited == map.tiles.size()) {
                gameLog.Log("Map terrain progress: " + std::to_string(terrainTilesVisited) + "/" +
                            std::to_string(map.tiles.size()) + " tiles visited; " +
                            std::to_string(impl_->terrainBuffers.size()) +
                            " terrain meshes uploaded.");
            }
            if (!tile.hasTerrainFile) {
                recordSkipped(skipped, "listed map tiles without a .terrain sidecar",
                              tileLabel(tile.x, tile.y) + " " + tile.textPath.filename().string());
                continue;
            }
            const std::uint64_t key = tileKey(tile.x, tile.y);
            const openbus::map::TerrainGrid grid = openbus::map::loadTerrainGrid(tile.terrainPath);
            terrainByCoordinate.emplace(key, grid);
            const std::size_t side = grid.intervals + 1;
            const double step = openbus::map::OMSI_TILE_SIZE_METERS / grid.intervals;
            std::vector<TerrainVertex> vertices;
            vertices.reserve(side * side);
            for (std::size_t row = 0; row < side; ++row) {
                for (std::size_t column = 0; column < side; ++column) {
                    const double x =
                        static_cast<double>(tile.x) * openbus::map::OMSI_TILE_SIZE_METERS +
                        static_cast<double>(column) * step;
                    const double y =
                        static_cast<double>(tile.y) * openbus::map::OMSI_TILE_SIZE_METERS +
                        static_cast<double>(row) * step;
                    vertices.push_back({static_cast<float>(x), static_cast<float>(y),
                                        grid.heights[row * side + column],
                                        static_cast<float>(column), static_cast<float>(row), 0.0F,
                                        0.0F, 0.0F, 1.0F});
                }
            }
            std::vector<std::uint32_t> indices;
            indices.reserve(grid.intervals * grid.intervals * 6);
            for (std::size_t row = 0; row < grid.intervals; ++row) {
                for (std::size_t column = 0; column < grid.intervals; ++column) {
                    const auto topLeft = static_cast<std::uint32_t>(row * side + column);
                    const auto topRight = static_cast<std::uint32_t>(row * side + column + 1);
                    const auto bottomRight =
                        static_cast<std::uint32_t>((row + 1) * side + column + 1);
                    const auto bottomLeft = static_cast<std::uint32_t>((row + 1) * side + column);
                    indices.insert(indices.end(), {topLeft, topRight, bottomRight, topLeft,
                                                   bottomRight, bottomLeft});
                }
            }
            Impl::TileBuffer buffer;
            buffer.indexCount = indices.size();
            buffer.tileX = tile.x;
            buffer.tileY = tile.y;
            pglGenBuffers(1, &buffer.buffer);
            pglBindBuffer(GL_ARRAY_BUFFER, buffer.buffer);
            pglBufferData(GL_ARRAY_BUFFER,
                          static_cast<std::ptrdiff_t>(vertices.size() * sizeof(TerrainVertex)),
                          vertices.data(), GL_STATIC_DRAW);
            pglGenBuffers(1, &buffer.indexBuffer);
            pglBindBuffer(GL_ARRAY_BUFFER, buffer.indexBuffer);
            pglBufferData(GL_ARRAY_BUFFER,
                          static_cast<std::ptrdiff_t>(indices.size() * sizeof(std::uint32_t)),
                          indices.data(), GL_STATIC_DRAW);
            if (buffer.buffer == 0 || buffer.indexBuffer == 0 || glGetError() != GL_NO_ERROR) {
                if (buffer.buffer != 0) {
                    pglDeleteBuffers(1, &buffer.buffer);
                }
                if (buffer.indexBuffer != 0) {
                    pglDeleteBuffers(1, &buffer.indexBuffer);
                }
                throw std::runtime_error("Could not upload terrain mesh for " +
                                         tile.textPath.filename().string());
            }
            impl_->terrainBuffers.push_back(buffer);
        }
    }
    gameLog.Log("Map terrain phase complete: " + std::to_string(impl_->terrainBuffers.size()) +
                " meshes from " + std::to_string(map.tiles.size()) + " map tiles in " +
                std::to_string(elapsedMilliseconds(terrainLoadStart)) + " ms.");
    if (impl_->terrainBuffers.empty()) {
        logSkipped(skipped);
        throw std::runtime_error("No terrain files found in the selected map");
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
        const double upper =
            std::lerp(heightAt(nextRow, column), heightAt(nextRow, nextColumn), fractionX);
        return std::lerp(lower, upper, fractionY);
    };

    std::unordered_map<std::string, std::vector<std::size_t>> sceneryAssetBatches;
    const auto loadSceneryAsset =
        [&](const std::filesystem::path& configPath) -> const std::vector<std::size_t>& {
        const std::string cacheKey = configPath.lexically_normal().generic_string();
        const auto cached = sceneryAssetBatches.find(cacheKey);
        if (cached != sceneryAssetBatches.end()) {
            return cached->second;
        }
        TraceScope trace("map", "MapRenderer.loadSceneryAsset");
        std::vector<std::size_t> batchIndexes;
        try {
            const std::filesystem::path objectRoot = configPath.parent_path();
            const std::filesystem::path modelRoot = objectRoot / "model";
            const SceneryObjectConfig objectConfiguration = loadSceneryObjectFile(configPath);
            for (const ConfigurationDiagnostic& diagnostic :
                 objectConfiguration.diagnostics.entries) {
                const std::string severity =
                    diagnostic.severity == ConfigurationDiagnostic::Severity::Error ? "error"
                                                                                    : "warning";
                recordSkipped(skipped, "scenery config diagnostics",
                              configPath.filename().string() +
                                  " line=" + std::to_string(diagnostic.line) + " [" +
                                  diagnostic.keyword + "] " + severity + ": " + diagnostic.message);
            }
            if (!objectConfiguration.trees.empty()) {
                for (const SceneryTreeDefinition& tree : objectConfiguration.trees) {
                    const std::filesystem::path treeTexture =
                        findSceneryTexture(objectRoot, modelRoot, omsiRoot, tree.texturePath);
                    if (treeTexture.empty()) {
                        recordSkipped(skipped, "tree scenery definitions with missing textures",
                                      configPath.filename().string() +
                                          " texture=" + tree.texturePath);
                        continue;
                    }
                    const double height = (tree.minimumHeight + tree.maximumHeight) * 0.5;
                    const double ratio = (tree.minimumRatio + tree.maximumRatio) * 0.5;
                    std::vector<ModelVertex> vertices =
                        makeTreePreview(height, height * 0.55 * ratio);
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
                        recordSkipped(skipped, "tree scenery batches rejected by OpenGL upload",
                                      configPath.filename().string());
                        continue;
                    }
                    batchIndexes.push_back(impl_->sceneryBatches.size());
                    impl_->sceneryBatches.push_back(batch);
                }
                return sceneryAssetBatches.emplace(cacheKey, std::move(batchIndexes)).first->second;
            }
            openbus::scripting::SceneryObject variables;
            const ModelConfig configuration =
                loadModelConfig(configPath, modelRoot, ModelConfigKind::SceneryObject, variables);
            for (const ConfigurationDiagnostic& diagnostic : configuration.diagnostics.entries) {
                const std::string severity =
                    diagnostic.severity == ConfigurationDiagnostic::Severity::Error ? "error"
                                                                                    : "warning";
                recordSkipped(skipped, "scenery model config diagnostics",
                              configPath.filename().string() +
                                  " line=" + std::to_string(diagnostic.line) + " [" +
                                  diagnostic.keyword + "] " + severity + ": " + diagnostic.message);
            }
            for (const ModelPart& part : configuration.parts) {
                if (part.lodIndex > 0) {
                    recordSkipped(skipped, "scenery model parts in nonzero LODs (unsupported)",
                                  configPath.filename().string() + " " +
                                      part.objPath.generic_string());
                    continue;
                }
                if (part.objPath.empty()) {
                    recordSkipped(skipped, "scenery model parts with no resolved mesh path",
                                  configPath.filename().string());
                    continue;
                }
                if (part.objPath.extension() != ".o3d") {
                    recordSkipped(skipped, "scenery model parts with unsupported mesh formats",
                                  configPath.filename().string() + " " +
                                      part.objPath.generic_string());
                    continue;
                }
                const std::shared_ptr<ParsedObj> parsed = O3DLoader::parse(part.objPath);
                if (!parsed) {
                    recordSkipped(skipped, "scenery O3D meshes that failed to parse",
                                  configPath.filename().string() + " " +
                                      part.objPath.filename().string());
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
                            recordSkipped(skipped,
                                          "scenery mesh vertices with invalid position indices",
                                          part.objPath.filename().string());
                            continue;
                        }
                        const ObjPosition& sourcePosition =
                            parsed->positions[static_cast<std::size_t>(index.position - 1)];
                        std::array<double, 3> normal = {0.0, 0.0, 1.0};
                        if (validIndex(index.normal, parsed->normals.size())) {
                            const ObjNormal& sourceNormal =
                                parsed->normals[static_cast<std::size_t>(index.normal - 1)];
                            normal = {sourceNormal.y, -sourceNormal.x, sourceNormal.z};
                            const double length =
                                std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] +
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
                        recordSkipped(skipped, "empty scenery material batches",
                                      part.objPath.filename().string());
                        continue;
                    }
                    const auto sourceMaterial =
                        parsed->materials.find("matl_" + std::to_string(materialIndex));
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
                                return std::filesystem::path(entry.second.textureName).filename() ==
                                       stateTexture;
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
                    pglBufferData(
                        GL_ARRAY_BUFFER,
                        static_cast<std::ptrdiff_t>(vertices.size() * sizeof(ModelVertex)),
                        vertices.data(), GL_STATIC_DRAW);
                    if (batch.buffer == 0 || glGetError() != GL_NO_ERROR) {
                        if (batch.buffer != 0) {
                            pglDeleteBuffers(1, &batch.buffer);
                        }
                        recordSkipped(skipped, "scenery model batches rejected by OpenGL upload",
                                      part.objPath.filename().string());
                        continue;
                    }
                    batchIndexes.push_back(impl_->sceneryBatches.size());
                    impl_->sceneryBatches.push_back(batch);
                }
            }
        } catch (const std::exception& error) {
            // Unsupported scenery configuration syntax/assets are skipped; they must not prevent
            // the map terrain and supported static scenery from rendering.
            recordSkipped(skipped, "scenery assets rejected while loading",
                          configPath.filename().string() + ": " + error.what());
        }
        return sceneryAssetBatches.emplace(cacheKey, std::move(batchIndexes)).first->second;
    };

    std::map<std::string, RoadMeshGeometry> roadVertices;
    std::unordered_map<std::string, openbus::map::MapSplineProfile> splineProfiles;
    std::unordered_set<std::string> failedSplineProfiles;
    std::unordered_set<std::string> editorOnlySplineProfiles;
    const auto mapContentStart = std::chrono::steady_clock::now();
    std::size_t tilesProcessed = 0;
    std::size_t sceneryPlacementsParsed = 0;
    std::size_t splinePlacementsParsed = 0;
    TraceScope sceneBuildPhase("map", "MapRenderer.buildSceneGeometryAndBatches");
    for (const openbus::map::MapTileReference& tileReference : map.tiles) {
        const int tileX = tileReference.x;
        const int tileY = tileReference.y;
        TraceScope tileTrace("map", "MapRenderer.processTile");
        const openbus::map::MapTileData tile = openbus::map::loadMapTile(tileReference);
        ++tilesProcessed;
        sceneryPlacementsParsed += tile.sceneryObjects.size() + tile.splineAttachments.size();
        splinePlacementsParsed += tile.splines.size();
        const std::string tileName = tileLabel(tileX, tileY);
        if (tile.attachedObjectCount != 0) {
            recordSkipped(skipped, "[attachObj] sections (not rendered)",
                          tileName + " " + tileReference.textPath.filename().string(),
                          tile.attachedObjectCount);
        }
        const auto addSceneryInstance =
            [&](const std::string& assetPath, const std::string& description, double worldX,
                double worldY, double worldZ, const std::array<double, 6>& rotationDegrees,
                std::size_t rotationCount) {
                const std::filesystem::path relative = normalizedAssetPath(assetPath);
                const std::array<std::filesystem::path, 2> candidates = {omsiRoot / relative,
                                                                         map.rootPath / relative};
                std::filesystem::path configPath;
                for (const std::filesystem::path& candidate : candidates) {
                    if (std::filesystem::is_regular_file(candidate)) {
                        configPath = candidate;
                        break;
                    }
                }
                if (configPath.empty()) {
                    recordSkipped(skipped, "scenery placements whose .sco config was not found",
                                  description);
                    return;
                }
                const std::vector<std::size_t>& batchIndexes = loadSceneryAsset(configPath);
                if (batchIndexes.empty()) {
                    recordSkipped(skipped, "scenery placements with no renderable asset batches",
                                  description + " config=" + configPath.filename().string());
                }
                for (const std::size_t batchIndex : batchIndexes) {
                    impl_->sceneryInstances.push_back(
                        {batchIndex, worldX, worldY, worldZ, rotationDegrees, rotationCount});
                }
            };

        for (const openbus::map::MapSceneryPlacement& object : tile.sceneryObjects) {
            const std::string objectDescription = tileName + " object=" + object.label +
                                                  " asset=" + object.assetPath + " rotationDeg=(" +
                                                  std::to_string(object.rotationDegrees[0]) + "," +
                                                  std::to_string(object.rotationDegrees[1]) + "," +
                                                  std::to_string(object.rotationDegrees[2]) + ")";
            if (!object.transformValid) {
                recordSkipped(
                    skipped, "[object] sections with invalid transforms",
                    objectDescription + " rawTransform=(" + object.rawTransformFields[0] + "," +
                        object.rawTransformFields[1] + "," + object.rawTransformFields[2] + "," +
                        object.rawTransformFields[3] + "," + object.rawTransformFields[4] + "," +
                        object.rawTransformFields[5] + ")");
                continue;
            }
            const double worldX = static_cast<double>(tileX) * openbus::map::OMSI_TILE_SIZE_METERS +
                                  object.localPosition[0];
            const double worldY = static_cast<double>(tileY) * openbus::map::OMSI_TILE_SIZE_METERS +
                                  object.localPosition[1];
            const int terrainTileX =
                static_cast<int>(std::floor(worldX / openbus::map::OMSI_TILE_SIZE_METERS));
            const int terrainTileY =
                static_cast<int>(std::floor(worldY / openbus::map::OMSI_TILE_SIZE_METERS));
            const std::optional<double> ground = sampleTerrainHeight(
                terrainTileX, terrainTileY,
                worldX - static_cast<double>(terrainTileX) * openbus::map::OMSI_TILE_SIZE_METERS,
                worldY - static_cast<double>(terrainTileY) * openbus::map::OMSI_TILE_SIZE_METERS);
            std::array<double, 6> rotationDegrees = {};
            std::copy(object.rotationDegrees.begin(), object.rotationDegrees.end(),
                      rotationDegrees.begin());
            addSceneryInstance(object.assetPath, objectDescription, worldX, worldY,
                               ground.value_or(0.0) + object.localPosition[2], rotationDegrees, 3);
        }
        for (const openbus::map::MapSplineAttachment& attachment : tile.splineAttachments) {
            const std::string description =
                tileName + (attachment.repeater ? " splineRepeater=" : " splineAttachment=") +
                attachment.label + " asset=" + attachment.assetPath +
                " splineIndex=" + std::to_string(attachment.splineIndex);
            if (!attachment.transformValid) {
                recordSkipped(skipped, "spline attachment records with invalid transforms",
                              description);
                continue;
            }
            const std::vector<openbus::map::MapSceneryPose> poses =
                openbus::map::placeMapSplineAttachment(attachment, tile.splines, tileX, tileY);
            if (poses.empty()) {
                recordSkipped(skipped, "spline attachment records with no local placement",
                              description);
                continue;
            }
            for (const openbus::map::MapSceneryPose& pose : poses) {
                addSceneryInstance(attachment.assetPath, description, pose.x, pose.y, pose.z,
                                   pose.rotationDegrees, pose.rotationCount);
            }
        }
        for (const openbus::map::MapSplinePlacement& spline : tile.splines) {
            const std::string splineDescription =
                tileName + (spline.elevated ? " [spline_h] asset=" : " [spline] asset=") +
                spline.assetPath;
            if (!spline.geometryValid) {
                recordSkipped(skipped, "[spline] sections with invalid numeric geometry",
                              splineDescription);
                continue;
            }
            const std::filesystem::path profilePath =
                resolveSplineProfile(map, omsiRoot, spline.assetPath);
            if (profilePath.empty()) {
                recordSkipped(skipped, "spline profile files not found", splineDescription);
                continue;
            }
            const std::string profileKey = profilePath.lexically_normal().generic_string();
            if (failedSplineProfiles.contains(profileKey)) {
                continue;
            }
            auto profileFound = splineProfiles.find(profileKey);
            if (profileFound == splineProfiles.end()) {
                try {
                    profileFound =
                        splineProfiles
                            .emplace(profileKey, openbus::map::loadMapSplineProfile(profilePath))
                            .first;
                } catch (const std::exception& error) {
                    failedSplineProfiles.insert(profileKey);
                    recordSkipped(skipped, "spline profile files that failed to parse",
                                  profilePath.filename().string() + ": " + error.what());
                    continue;
                }
            }
            const openbus::map::MapSplineProfile& profile = profileFound->second;
            if (profile.editorOnly) {
                if (editorOnlySplineProfiles.insert(profileKey).second) {
                    recordSkipped(skipped, "editor-only spline profiles",
                                  profilePath.filename().string());
                }
                continue;
            }
            if (profile.sections.empty()) {
                recordSkipped(skipped, "spline profiles with no graphical cross-section strips",
                              profilePath.filename().string());
                continue;
            }
            const double startX =
                static_cast<double>(tileX) * openbus::map::OMSI_TILE_SIZE_METERS + spline.localX;
            const double startY =
                static_cast<double>(tileY) * openbus::map::OMSI_TILE_SIZE_METERS + spline.localY;
            const std::vector<openbus::map::MapSplineSample> centerline =
                openbus::map::tessellateMapSpline(startX, startY, spline.rotationDegrees,
                                                  spline.length, spline.radius);
            if (centerline.size() < 2) {
                recordSkipped(skipped, "[spline] sections that failed geometry tessellation",
                              splineDescription);
                continue;
            }
            for (const openbus::map::MapSplineProfileSection& section : profile.sections) {
                if (section.points.size() < 2) {
                    recordSkipped(skipped, "spline profile strips with fewer than two points",
                                  profilePath.filename().string());
                    continue;
                }
                if (section.textureIndex < 0 ||
                    static_cast<std::size_t>(section.textureIndex) >= profile.textures.size()) {
                    recordSkipped(skipped, "spline profile strips with invalid texture indices",
                                  profilePath.filename().string() +
                                      " index=" + std::to_string(section.textureIndex));
                    continue;
                }
                const std::string& textureName =
                    profile.textures[static_cast<std::size_t>(section.textureIndex)];
                const std::filesystem::path roadTexture =
                    findSplineTexture(profilePath, omsiRoot, textureName);
                const bool textured = !roadTexture.empty();
                if (!textured) {
                    recordSkipped(
                        skipped, "spline profile textures not found (geometry rendered untextured)",
                        profilePath.filename().string() + " texture=" + textureName);
                }
                const std::string tileBatchPrefix =
                    std::to_string(tileX) + "," + std::to_string(tileY) + "|";
                const std::string batchKey =
                    tileBatchPrefix + (textured ? roadTexture.lexically_normal().generic_string()
                                                : "untextured:" + profileKey + "#" +
                                                      std::to_string(section.textureIndex));
                RoadMeshGeometry& road = roadVertices[batchKey];
                if (road.vertices.empty()) {
                    road.texturePath = roadTexture;
                    road.textured = textured;
                }
                appendRoadProfile(road, spline, centerline, section);
            }
        }
        if (tilesProcessed % 100 == 0 || tilesProcessed == map.tiles.size()) {
            gameLog.Log("Map content progress: " + std::to_string(tilesProcessed) + "/" +
                        std::to_string(map.tiles.size()) +
                        " tiles processed; scenery=" + std::to_string(sceneryPlacementsParsed) +
                        ", splines=" + std::to_string(splinePlacementsParsed) +
                        ", cached spline profiles=" + std::to_string(splineProfiles.size()) + ".");
        }
    }
    gameLog.Log("Map content phase complete: " + std::to_string(tilesProcessed) + " tiles, " +
                std::to_string(sceneryPlacementsParsed) + " scenery placements, " +
                std::to_string(splinePlacementsParsed) + " splines, " +
                std::to_string(splineProfiles.size()) + " unique profiles in " +
                std::to_string(elapsedMilliseconds(mapContentStart)) + " ms.");

    const auto roadUploadStart = std::chrono::steady_clock::now();
    TraceScope roadUploadPhase("map", "MapRenderer.uploadRoadBatches");
    for (auto& [batchKey, geometry] : roadVertices) {
        if (geometry.vertices.empty()) {
            continue;
        }
        Impl::ModelBatch batch;
        batch.vertexCount = geometry.vertices.size();
        batch.indexCount = geometry.indices.size();
        batch.hasVisibilityBounds = true;
        batch.minX = geometry.minX;
        batch.minY = geometry.minY;
        batch.maxX = geometry.maxX;
        batch.maxY = geometry.maxY;
        if (geometry.textured) {
            try {
                batch.material.texture = getTexture(geometry.texturePath);
                batch.material.textured = true;
            } catch (const std::exception& error) {
                recordSkipped(skipped, "spline profile textures that failed to load",
                              geometry.texturePath.generic_string() + ": " + error.what());
                batch.color = {0.45, 0.45, 0.45};
            }
        } else {
            batch.color = {0.45, 0.45, 0.45};
        }
        pglGenBuffers(1, &batch.buffer);
        pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
        pglBufferData(GL_ARRAY_BUFFER,
                      static_cast<std::ptrdiff_t>(geometry.vertices.size() * sizeof(ModelVertex)),
                      geometry.vertices.data(), GL_STATIC_DRAW);
        pglGenBuffers(1, &batch.indexBuffer);
        pglBindBuffer(GL_ARRAY_BUFFER, batch.indexBuffer);
        pglBufferData(GL_ARRAY_BUFFER,
                      static_cast<std::ptrdiff_t>(geometry.indices.size() * sizeof(std::uint32_t)),
                      geometry.indices.data(), GL_STATIC_DRAW);
        if (batch.buffer == 0 || batch.indexBuffer == 0 || glGetError() != GL_NO_ERROR) {
            if (batch.buffer != 0) {
                pglDeleteBuffers(1, &batch.buffer);
            }
            if (batch.indexBuffer != 0) {
                pglDeleteBuffers(1, &batch.indexBuffer);
            }
            recordSkipped(skipped, "road batches rejected by OpenGL upload", batchKey);
            continue;
        }
        impl_->roadBatches.push_back(batch);
    }
    gameLog.Log("Map road upload complete: geometry groups=" + std::to_string(roadVertices.size()) +
                ", uploaded batches=" + std::to_string(impl_->roadBatches.size()) + " in " +
                std::to_string(elapsedMilliseconds(roadUploadStart)) + " ms.");

    gameLog.Log("Map render result: terrain tiles=" + std::to_string(impl_->terrainBuffers.size()) +
                ", scenery instances=" + std::to_string(impl_->sceneryInstances.size()) +
                ", scenery batches=" + std::to_string(impl_->sceneryBatches.size()) +
                ", road batches=" + std::to_string(impl_->roadBatches.size()) +
                ", cached textures=" + std::to_string(textureCache.size()) + ".");
    logSkipped(skipped);
    gameLog.Log("Map loading complete in " + std::to_string(elapsedMilliseconds(mapLoadStart)) +
                " ms.");
}

MapRenderer::~MapRenderer() = default;

void MapRenderer::draw() const {
    TraceScope trace("map", "MapRenderer::draw");
    const Matrix4& view = modelViewMatrix();
    const double cameraX = -(view[0] * view[12] + view[1] * view[13] + view[2] * view[14]);
    const double cameraY = -(view[4] * view[12] + view[5] * view[13] + view[6] * view[14]);
    const auto outsideVisibilityBounds = [&](double minX, double minY, double maxX, double maxY) {
        return squaredDistanceToBounds(cameraX, cameraY, minX, minY, maxX, maxY) >
               MAP_VISIBILITY_RADIUS_SQUARED;
    };

    ModelMaterial material;
    material.texture = impl_->texture;
    material.textured = true;
    for (const Impl::TileBuffer& tile : impl_->terrainBuffers) {
        const double minX = static_cast<double>(tile.tileX) * openbus::map::OMSI_TILE_SIZE_METERS;
        const double minY = static_cast<double>(tile.tileY) * openbus::map::OMSI_TILE_SIZE_METERS;
        if (outsideVisibilityBounds(minX, minY, minX + openbus::map::OMSI_TILE_SIZE_METERS,
                                    minY + openbus::map::OMSI_TILE_SIZE_METERS)) {
            continue;
        }
        drawIndexedModelBatch(tile.buffer, tile.indexBuffer, tile.indexCount, material,
                              {1.0, 1.0, 1.0}, 1.0, 0);
    }
    for (const Impl::ModelBatch& road : impl_->roadBatches) {
        if (road.hasVisibilityBounds &&
            outsideVisibilityBounds(road.minX, road.minY, road.maxX, road.maxY)) {
            continue;
        }
        drawIndexedModelBatch(road.buffer, road.indexBuffer, road.indexCount, road.material,
                              road.color, 1.0, road.alphaMode);
    }
    for (const Impl::SceneryInstance& instance : impl_->sceneryInstances) {
        if (outsideVisibilityBounds(instance.x, instance.y, instance.x, instance.y)) {
            continue;
        }
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
        std::size_t ownRotationOffset = 0;
        if (instance.rotationCount >= 6) {
            rotate(-instance.rotationDegrees[0], 0.0, 0.0, 1.0);
            rotate(instance.rotationDegrees[2], 0.0, 1.0, 0.0);
            rotate(instance.rotationDegrees[1], 1.0, 0.0, 0.0);
            ownRotationOffset = 3;
        }
        rotate(-instance.rotationDegrees[ownRotationOffset], 0.0, 0.0, 1.0);
        rotate(-instance.rotationDegrees[ownRotationOffset + 2], 0.0, 1.0, 0.0);
        rotate(-instance.rotationDegrees[ownRotationOffset + 1], 1.0, 0.0, 0.0);
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