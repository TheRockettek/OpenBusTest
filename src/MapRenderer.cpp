#include "MapRenderer.h"

#include "CoreRenderer.h"
#include "Environment.h"
#include "Logger.h"
#include "MapRoadGeometry.h"
#include "MapSceneryPlacement.h"
#include "MapSplineGeometry.h"
#include "MapSplineProfile.h"
#include "ModelConfigLoader.h"
#include "ObjLoader.h"
#include "O3DLoader.h"
#include "OpenGLFunctions.h"
#include "PerfTrace.h"
#include "SceneryObjectConfigLoader.h"
#include "TextureLoader.h"
#include "Variables.h"
#include "XLoader.h"

#include <algorithm>
#include <array>
#include <charconv>
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
#include <system_error>
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
        gameLog.Log("Map skip encountered: " + reason + "; example: " + example);
    }
}

void logSkipped(const SkipSummaries& summaries) {
    for (const auto& [reason, summary] : summaries) {
        std::string message = "Map skip summary: skipped " + std::to_string(summary.count) +
                              " " + reason;
        if (!summary.examples.empty()) {
            message += " (context logged when encountered)";
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
    double minZ = std::numeric_limits<double>::infinity();
    double maxZ = -std::numeric_limits<double>::infinity();
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

GLuint loadTexture(const std::filesystem::path& path, bool repeat = true) {
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
    const GLint wrapMode = repeat ? GL_REPEAT : GL_CLAMP_TO_EDGE;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapMode);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapMode);
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

std::string lowercasePathComponent(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

std::filesystem::path findCaseInsensitiveFile(const std::filesystem::path& path) {
    if (std::filesystem::is_regular_file(path)) {
        return path;
    }
    std::filesystem::path resolved = path.root_path();
    for (const std::filesystem::path& component : path.relative_path()) {
        const std::filesystem::path exact = resolved / component;
        if (std::filesystem::exists(exact)) {
            resolved = exact;
            continue;
        }
        std::error_code error;
        if (!std::filesystem::is_directory(resolved, error) || error) {
            return {};
        }
        const std::string targetName = lowercasePathComponent(component.string());
        bool matched = false;
        for (std::filesystem::directory_iterator entry(resolved, error), end;
             !error && entry != end; entry.increment(error)) {
            if (lowercasePathComponent(entry->path().filename().string()) == targetName) {
                resolved = entry->path();
                matched = true;
                break;
            }
        }
        if (!matched) {
            return {};
        }
    }
    return std::filesystem::is_regular_file(resolved) ? resolved : std::filesystem::path{};
}

std::filesystem::path findSceneryTexture(const std::filesystem::path& objectRoot,
                                         const std::filesystem::path& modelRoot,
                                         const std::filesystem::path& omsiRoot,
                                         const std::string& textureName) {
    const std::filesystem::path relative = normalizedAssetPath(textureName);
    const std::filesystem::path packageRoot = objectRoot.parent_path();
    const std::filesystem::path modelPackageRoot = modelRoot.parent_path();
    const std::array<std::filesystem::path, 9> candidates = {
        objectRoot / "texture" / relative,
        objectRoot / relative,
        modelRoot / relative,
        packageRoot / "texture" / relative,
        packageRoot / relative,
        modelPackageRoot / "texture" / relative,
        modelPackageRoot / relative,
        omsiRoot / relative,
        omsiRoot / "Texture" / relative};
    for (const std::filesystem::path& candidate : candidates) {
        const std::filesystem::path exact = findCaseInsensitiveFile(candidate);
        if (!exact.empty()) {
            return exact;
        }
        std::string extension = lowercasePathComponent(candidate.extension().string());
        constexpr std::array<const char*, 6> textureExtensions = {
            ".dds", ".bmp", ".tga", ".png", ".jpg", ".jpeg"};
        const bool supportedExtension = std::any_of(
            textureExtensions.begin(), textureExtensions.end(),
            [&extension](const char* supported) { return extension == supported; });
        if (supportedExtension) {
            for (const char* alternativeExtension : textureExtensions) {
                if (extension == alternativeExtension) {
                    continue;
                }
                std::filesystem::path alternate = candidate;
                alternate.replace_extension(alternativeExtension);
                const std::filesystem::path alternateMatch = findCaseInsensitiveFile(alternate);
                if (!alternateMatch.empty()) {
                    return alternateMatch;
                }
            }
        }
    }
    return {};
}

std::uint64_t tileKey(int x, int y) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
           static_cast<std::uint32_t>(y);
}

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

using MapFrustum = std::array<std::array<double, 4>, 6>;

MapFrustum buildMapFrustum(const Matrix4& projection) {
    const std::array<std::array<double, 4>, 6> signs = {{{{1.0, 0.0, 0.0, 1.0}},
                                                         {{-1.0, 0.0, 0.0, 1.0}},
                                                         {{0.0, 1.0, 0.0, 1.0}},
                                                         {{0.0, -1.0, 0.0, 1.0}},
                                                         {{0.0, 0.0, 1.0, 1.0}},
                                                         {{0.0, 0.0, -1.0, 1.0}}}};
    MapFrustum planes = {};
    for (std::size_t plane = 0; plane < signs.size(); ++plane) {
        for (std::size_t column = 0; column < 4; ++column) {
            for (std::size_t row = 0; row < 4; ++row) {
                planes[plane][column] += signs[plane][row] * projection[row + column * 4];
            }
        }
    }
    return planes;
}

bool outsideMapFrustum(const MapFrustum& planes, const Matrix4& modelView,
                       const std::array<double, 3>& center, double radius) {
    std::array<double, 4> eye = {modelView[0] * center[0] + modelView[4] * center[1] +
                                     modelView[8] * center[2] + modelView[12],
                                 modelView[1] * center[0] + modelView[5] * center[1] +
                                     modelView[9] * center[2] + modelView[13],
                                 modelView[2] * center[0] + modelView[6] * center[1] +
                                     modelView[10] * center[2] + modelView[14],
                                 1.0};
    for (const std::array<double, 4>& plane : planes) {
        const double normalLength =
            std::sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
        const double distance =
            plane[0] * eye[0] + plane[1] * eye[1] + plane[2] * eye[2] + plane[3];
        if (distance < -radius * normalLength) {
            return true;
        }
    }
    return false;
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
    const openbus::map::MapRoadSectionGeometry geometry =
        openbus::map::buildMapRoadSectionGeometry(spline, centerline, section);
    const std::uint32_t baseVertex = static_cast<std::uint32_t>(road.vertices.size());
    for (const openbus::map::MapRoadVertex& vertex : geometry.vertices) {
        const ModelVertex generated{static_cast<float>(vertex.x),
                                    static_cast<float>(vertex.y),
                                    static_cast<float>(vertex.z),
                                    static_cast<float>(vertex.textureU),
                                    static_cast<float>(vertex.textureV),
                                    0.0F,
                                    0.0F,
                                    0.0F,
                                    1.0F};
        road.minX = std::min(road.minX, vertex.x);
        road.minY = std::min(road.minY, vertex.y);
        road.maxX = std::max(road.maxX, vertex.x);
        road.maxY = std::max(road.maxY, vertex.y);
        road.minZ = std::min(road.minZ, vertex.z);
        road.maxZ = std::max(road.maxZ, vertex.z);
        road.vertices.push_back(generated);
    }
    for (const int index : geometry.indices) {
        road.indices.push_back(baseVertex + static_cast<std::uint32_t>(index));
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
        double minZ = 0.0;
        double maxZ = 0.0;
        std::vector<ModelMaterial> groundTextureLayers;
    };

    struct ModelBatch {
        GLuint buffer = 0;
        GLuint indexBuffer = 0;
        std::size_t vertexCount = 0;
        std::size_t indexCount = 0;
        ModelMaterial material;
        std::array<double, 3> color = {1.0, 1.0, 1.0};
        double alpha = 1.0;
        int alphaMode = 0;
        bool hasVisibilityBounds = false;
        double minX = 0.0;
        double minY = 0.0;
        double maxX = 0.0;
        double maxY = 0.0;
        double minZ = 0.0;
        double maxZ = 0.0;
        double boundingRadius = 0.0;
    };

    struct SceneryInstance {
        std::size_t batchIndex = 0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double boundingRadius = 0.0;
        std::array<double, 6> rotationDegrees = {};
        std::size_t rotationCount = 3;
    };

    struct SceneryChunk {
        std::vector<SceneryInstance> instances;
        double minX = std::numeric_limits<double>::infinity();
        double minY = std::numeric_limits<double>::infinity();
        double minZ = std::numeric_limits<double>::infinity();
        double maxX = -std::numeric_limits<double>::infinity();
        double maxY = -std::numeric_limits<double>::infinity();
        double maxZ = -std::numeric_limits<double>::infinity();
    };

    GLuint texture = 0;
    std::vector<TileBuffer> terrainBuffers;
    std::vector<ModelBatch> sceneryBatches;
    std::unordered_map<std::uint64_t, SceneryChunk> sceneryChunks;
    std::size_t sceneryInstanceCount = 0;
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
                         std::size_t groundTextureIndex, const std::filesystem::path& omsiRoot)
    : impl_(std::make_unique<Impl>()) {
    TraceScope trace("map", "MapRenderer::MapRenderer");
    const auto mapLoadStart = std::chrono::steady_clock::now();
    if (map.groundTextures.empty()) {
        throw std::runtime_error("Map has no [groundtex] records");
    }
    if (centerTileIndex >= map.tiles.size()) {
        throw std::runtime_error("Map spawn references an invalid tile index");
    }
    if (groundTextureIndex >= map.groundTextures.size()) {
        throw std::runtime_error("Map ground-texture preview references an invalid [groundtex] "
                                 "index");
    }
    if (pglGenBuffers == nullptr || pglBindBuffer == nullptr || pglBufferData == nullptr) {
        throw std::runtime_error("OpenGL buffer functions are unavailable for map terrain");
    }

    const char* groundTextureOverride = openbus::getEnvironment("OPENBUS_MAP_GROUNDTEX");
    const bool uniformGroundTexturePreview =
        groundTextureOverride != nullptr && *groundTextureOverride != '\0';
    const std::size_t baseGroundTextureIndex = uniformGroundTexturePreview ? groundTextureIndex : 0;
    const std::filesystem::path texturePath = groundTexturePath(
        map, omsiRoot, map.groundTextures[baseGroundTextureIndex].texturePath);
    if (uniformGroundTexturePreview) {
        gameLog.Log("Map base-ground-texture preview override: [groundtex] index=" +
                    std::to_string(groundTextureIndex) + ", path=" + texturePath.generic_string() +
                    "; applied uniformly.");
    } else {
        gameLog.Log("Map base-ground-texture: [groundtex] index=0, path=" +
                    texturePath.generic_string() + "; per-tile ground-texture masks enabled.");
    }
    {
        TraceScope phase("map", "MapRenderer.loadGroundTexture");
        impl_->texture = loadTexture(texturePath);
    }

    std::unordered_map<std::uint64_t, openbus::map::TerrainGrid> terrainByCoordinate;
    terrainByCoordinate.reserve(map.tiles.size());
    std::unordered_map<std::string, GLuint> textureCache;
    const auto getTexture = [&](const std::filesystem::path& path, bool repeat = true) {
        const std::string key = path.lexically_normal().generic_string() +
                                (repeat ? "|repeat" : "|clamp");
        const auto found = textureCache.find(key);
        if (found != textureCache.end()) {
            return found->second;
        }
        const GLuint texture = loadTexture(path, repeat);
        textureCache.emplace(key, texture);
        impl_->ownedTextures.push_back(texture);
        return texture;
    };
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
            if (!uniformGroundTexturePreview) {
                std::map<std::size_t, const openbus::map::MapGroundTextureSidecar*> layersByIndex;
                for (const openbus::map::MapGroundTextureSidecar& sidecar :
                     tile.groundTextureSidecars) {
                    std::size_t textureIndex = 0;
                    const char* suffixBegin = sidecar.ordinalSuffix.data();
                    const char* suffixEnd = suffixBegin + sidecar.ordinalSuffix.size();
                    const auto parsed = std::from_chars(suffixBegin, suffixEnd, textureIndex);
                    if (parsed.ec != std::errc{} || parsed.ptr != suffixEnd || textureIndex == 0 ||
                        textureIndex >= map.groundTextures.size()) {
                        recordSkipped(skipped, "ground-texture sidecars with invalid groundtex indices",
                                      tile.textPath.filename().string() + " sidecar=" +
                                          sidecar.path.filename().string());
                        continue;
                    }
                    if (!layersByIndex.emplace(textureIndex, &sidecar).second) {
                        recordSkipped(skipped, "duplicate ground-texture sidecar indices",
                                      tile.textPath.filename().string() + " index=" +
                                          std::to_string(textureIndex));
                    }
                }
                for (const auto& [textureIndex, sidecar] : layersByIndex) {
                    try {
                        ModelMaterial layer;
                        const std::filesystem::path layerTexturePath = groundTexturePath(
                            map, omsiRoot, map.groundTextures[textureIndex].texturePath);
                        layer.texture = getTexture(layerTexturePath);
                        layer.textured = true;
                        layer.transmap = getTexture(sidecar->path, false);
                        layer.useTransmap = true;
                        // OMSI ground-texture DDS rows run opposite the terrain mesh's map northing.
                        layer.flipTransmapY = true;
                        const float inverseIntervals = 1.0F / static_cast<float>(grid.intervals);
                        layer.transmapScale = {inverseIntervals, inverseIntervals};
                        buffer.groundTextureLayers.push_back(layer);
                    } catch (const std::exception& error) {
                        recordSkipped(skipped, "ground-texture layers that failed to load",
                                      sidecar->path.filename().string() + ": " + error.what());
                    }
                }
            }
            const auto [minimumHeight, maximumHeight] =
                std::minmax_element(grid.heights.begin(), grid.heights.end());
            buffer.minZ = *minimumHeight;
            buffer.maxZ = *maximumHeight;
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
    std::unordered_set<std::string> editorOnlySceneryAssets;
    std::unordered_set<std::string> absoluteHeightSceneryAssets;
    const bool verboseAlphaMaterials =
        parseEnabledFlag(openbus::getEnvironment("OPENBUS_VERBOSE_TEXTURE_READ"));
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
            const SceneryObjectConfig objectConfiguration = loadSceneryObjectFile(configPath);
            // Tree SCOs may mark their helper mesh [onlyeditor] while their [tree]
            // definition is runtime scenery. The tree preview below replaces that mesh.
            if (objectConfiguration.onlyEditor && objectConfiguration.trees.empty()) {
                editorOnlySceneryAssets.insert(cacheKey);
                return sceneryAssetBatches.emplace(cacheKey, std::move(batchIndexes))
                    .first->second;
            }
            std::filesystem::path modelConfigPath =
                resolveSceneryObjectModelConfigPath(objectConfiguration);
            bool usesExternalModelConfig = !objectConfiguration.modelPath.empty();
            if (usesExternalModelConfig && !std::filesystem::is_regular_file(modelConfigPath)) {
                recordSkipped(skipped, "scenery external model configs not found",
                              configPath.filename().string() + " model=" +
                                  modelConfigPath.generic_string());
                modelConfigPath = configPath;
                usesExternalModelConfig = false;
            }
            const std::filesystem::path modelRoot =
                usesExternalModelConfig ? modelConfigPath.parent_path() : objectRoot / "model";
            if (objectConfiguration.absoluteHeight) {
                absoluteHeightSceneryAssets.insert(cacheKey);
            }
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
                    batch.boundingRadius = vertexBoundingRadius(vertices);
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
                loadModelConfig(modelConfigPath, modelRoot, ModelConfigKind::SceneryObject,
                                variables);
            if (configuration.absoluteHeight) {
                absoluteHeightSceneryAssets.insert(cacheKey);
            }
            for (const ConfigurationDiagnostic& diagnostic : configuration.diagnostics.entries) {
                const std::string severity =
                    diagnostic.severity == ConfigurationDiagnostic::Severity::Error ? "error"
                                                                                    : "warning";
                recordSkipped(skipped, "scenery model config diagnostics",
                              modelConfigPath.filename().string() +
                                  " line=" + std::to_string(diagnostic.line) + " [" +
                                  diagnostic.keyword + "] " + severity + ": " + diagnostic.message);
            }
            for (const ModelPart& part : configuration.parts) {
                if (part.lodIndex > 0) {
                    recordSkipped(skipped, "scenery model parts in nonzero LODs (unsupported)",
                                  modelConfigPath.filename().string() + " " +
                                      part.objPath.generic_string());
                    continue;
                }
                if (part.objPath.empty()) {
                    recordSkipped(skipped, "scenery model parts with no resolved mesh path",
                                  modelConfigPath.filename().string());
                    continue;
                }
                std::string meshExtension = part.objPath.extension().string();
                std::transform(meshExtension.begin(), meshExtension.end(), meshExtension.begin(),
                               [](unsigned char value) {
                                   return static_cast<char>(std::tolower(value));
                               });
                if (meshExtension != ".o3d" && meshExtension != ".x") {
                    recordSkipped(skipped, "scenery model parts with unsupported mesh formats",
                                  modelConfigPath.filename().string() + " " +
                                      part.objPath.generic_string());
                    continue;
                }
                const std::shared_ptr<ParsedObj> parsed =
                    meshExtension == ".x" ? XLoader::parse(part.objPath)
                                          : O3DLoader::parse(part.objPath);
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
                            // O3DLoader already swaps the file's up/forward axes and
                            // adjusts handedness. Do not apply the OBJ vehicle-frame
                            // conversion a second time to map scenery normals.
                            normal = {sourceNormal.x, sourceNormal.y, sourceNormal.z};
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
                            // Text .x foliage assets may author a negative V range that already
                            // establishes the intended orientation. O3D's stored coordinate has
                            // already been converted by O3DLoader, so retain the map renderer's
                            // existing second conversion only for that format.
                            v = static_cast<float>(meshExtension == ".x" ? coordinate.v
                                                                         : 1.0 - coordinate.v);
                        }
                        verticesByMaterial[materialIndex].push_back(
                            {static_cast<float>(sourcePosition.x),
                             static_cast<float>(sourcePosition.y),
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
                    const ModelMaterialState* state = stateFound == statesByMaterial.end()
                                                          ? nullptr
                                                          : stateFound->second;
                    // Some OMSI O3D meshes number their material slots differently from the
                    // SCO's [matl] index. When the numeric slot misses, safely bind a unique
                    // matching texture override by basename so [matl_alpha] and friends survive.
                    if (state == nullptr && sourceMaterial != parsed->materials.end()) {
                        const std::string sourceTextureName = lowercasePathComponent(
                            std::filesystem::path(sourceMaterial->second.textureName)
                                .filename()
                                .string());
                        if (!sourceTextureName.empty()) {
                            const ModelMaterialState* matchingState = nullptr;
                            bool ambiguousMatch = false;
                            for (const ModelMaterialState& candidate : part.materialStatesInOrder) {
                                const std::string candidateTextureName = lowercasePathComponent(
                                    std::filesystem::path(candidate.textureName)
                                        .filename()
                                        .string());
                                if (candidateTextureName != sourceTextureName) {
                                    continue;
                                }
                                if (matchingState != nullptr) {
                                    ambiguousMatch = true;
                                    break;
                                }
                                matchingState = &candidate;
                            }
                            if (!ambiguousMatch) {
                                state = matchingState;
                            }
                        }
                    }
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
                    } else if (!textureName.empty()) {
                        recordSkipped(skipped, "scenery material textures not found",
                                      modelConfigPath.filename().string() +
                                          " mesh=" + part.objPath.filename().string() +
                                          " texture=" + textureName);
                    }
                    if (state != nullptr && !state->transmapTextureName.empty()) {
                        const std::filesystem::path transmapTexture = findSceneryTexture(
                            objectRoot, modelRoot, omsiRoot, state->transmapTextureName);
                        if (transmapTexture.empty()) {
                            recordSkipped(skipped, "scenery transmap textures not found",
                                          modelConfigPath.filename().string() +
                                              " mesh=" + part.objPath.filename().string() +
                                              " transmap=" + state->transmapTextureName);
                        } else {
                            material.transmap = getTexture(transmapTexture);
                            material.useTransmap = material.transmap != 0;
                        }
                    }
                    Impl::ModelBatch batch;
                    batch.vertexCount = vertices.size();
                    batch.boundingRadius = vertexBoundingRadius(vertices);
                    batch.material = material;
                    batch.color = color;
                    batch.alpha = sourceMaterial == parsed->materials.end()
                                      ? 1.0
                                      : std::clamp(sourceMaterial->second.alpha, 0.0, 1.0);
                    batch.alphaMode = state == nullptr ? 0 : state->alphaMode;
                    if (batch.alphaMode == 0 && batch.alpha < 1.0 - 1.0e-6) {
                        batch.alphaMode = 2;
                    }
                    if (verboseAlphaMaterials && batch.alphaMode != 0) {
                        gameLog.Log("Map scenery alpha material: asset=" +
                                    modelConfigPath.filename().string() + " mesh=" +
                                    part.objPath.filename().string() + " index=" +
                                    std::to_string(materialIndex) + " mode=" +
                                    std::to_string(batch.alphaMode) + " state=" +
                                    (state == nullptr ? "none" : std::to_string(state->alphaMode)) +
                                    " texture=" + materialTexture.generic_string());
                    }
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
        sceneryPlacementsParsed += tile.sceneryObjects.size() + tile.attachedObjects.size() +
                                   tile.splineAttachments.size();
        splinePlacementsParsed += tile.splines.size();
        const std::string tileName = tileLabel(tileX, tileY);
        for (const ConfigurationDiagnostic& diagnostic : tile.diagnostics.entries) {
            recordSkipped(skipped, "unimplemented map tile sections [" + diagnostic.keyword + "]",
                          tileName + " line=" + std::to_string(diagnostic.line));
        }
        for (const openbus::map::MapAttachedObject& attachedObject : tile.attachedObjects) {
            recordSkipped(skipped,
                          "[attachObj] placements parsed but not rendered (parent anchor semantics "
                          "unresolved)",
                          tileName + " id=" + std::to_string(attachedObject.id) +
                              " parent=" + std::to_string(attachedObject.attachedToObjectId) + " " +
                              attachedObject.assetPath);
        }
        const auto reportVariableParent = [&](int objectId, const std::optional<int>& parentId) {
            if (parentId.has_value()) {
                recordSkipped(skipped,
                              "[varparent] references parsed but parent transforms not applied",
                              tileName + " id=" + std::to_string(objectId) +
                                  " parent=" + std::to_string(*parentId));
            }
        };
        for (const openbus::map::MapSceneryPlacement& object : tile.sceneryObjects) {
            reportVariableParent(object.id, object.variableParentId);
        }
        for (const openbus::map::MapAttachedObject& object : tile.attachedObjects) {
            reportVariableParent(object.id, object.variableParentId);
        }
        for (const openbus::map::MapSplineAttachment& attachment : tile.splineAttachments) {
            reportVariableParent(attachment.id, attachment.variableParentId);
        }
        const auto reportTerrainAlignment = [&](int objectId, bool enabled) {
            if (enabled) {
                recordSkipped(skipped,
                              "[spline_terrain_align] markers parsed but alignment not applied",
                              tileName + " id=" + std::to_string(objectId));
            }
        };
        const auto reportRules = [&](int objectId,
                                     const std::vector<openbus::map::MapTileRule>& rules) {
            for (const openbus::map::MapTileRule& rule : rules) {
                recordSkipped(skipped,
                              rule.kill ? "[kill_rule] records parsed but not applied"
                                        : "[rule] records parsed but not applied",
                              tileName + " id=" + std::to_string(objectId) +
                                  " first-field=" + rule.fields[0]);
            }
        };
        for (const openbus::map::MapSceneryPlacement& object : tile.sceneryObjects) {
            reportTerrainAlignment(object.id, object.splineTerrainAlign);
            reportRules(object.id, object.rules);
        }
        for (const openbus::map::MapAttachedObject& object : tile.attachedObjects) {
            reportTerrainAlignment(object.id, object.splineTerrainAlign);
            reportRules(object.id, object.rules);
        }
        for (const openbus::map::MapSplineAttachment& attachment : tile.splineAttachments) {
            reportTerrainAlignment(attachment.id, attachment.splineTerrainAlign);
            reportRules(attachment.id, attachment.rules);
        }
        for (const openbus::map::MapSplinePlacement& spline : tile.splines) {
            if (spline.splineTerrainAlign2.has_value()) {
                recordSkipped(skipped,
                              "[spline_terrain_align_2] values parsed but alignment not applied",
                              tileName + " spline=" + std::to_string(spline.splineId));
            }
            reportRules(spline.splineId, spline.rules);
        }
        if (tile.hasWaterMarker) {
            if (!tileReference.hasWaterFile) {
                recordSkipped(skipped, "[water] markers without a .map.water sidecar",
                              tileName + " " + tileReference.textPath.filename().string());
            } else {
                try {
                    const openbus::map::WaterData water =
                        openbus::map::loadWaterData(tileReference.waterPath);
                    recordSkipped(
                        skipped,
                        "water surfaces parsed but not rendered (spatial ordering unresolved)",
                        tileName + " surfaces=" + std::to_string(water.surfaces.size()));
                } catch (const std::exception& error) {
                    recordSkipped(skipped, "invalid .map.water sidecars",
                                  tileName + ": " + error.what());
                }
            }
        }
        if (tile.hasVariableTerrainMarker) {
            recordSkipped(skipped, "[variable_terrain] tiles (dynamic terrain not rendered)",
                          tileName + " " + tileReference.textPath.filename().string());
        }
        if (tile.hasVariableTerrainLightmapMarker) {
            recordSkipped(
                skipped,
                "[variable_terrainlightmap] tiles (dynamic terrain lightmaps not rendered)",
                tileName + " " + tileReference.textPath.filename().string());
        }
        const auto addSceneryInstance =
            [&](const std::string& assetPath, const std::string& description, double worldX,
                double worldY, double localZ, double terrainHeightOffset,
                const std::array<double, 6>& rotationDegrees, std::size_t rotationCount) {
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
                if (editorOnlySceneryAssets.contains(
                        configPath.lexically_normal().generic_string())) {
                    recordSkipped(skipped, "map scenery placements marked [onlyeditor]",
                                  description + " config=" + configPath.filename().string());
                    return;
                }
                const bool absoluteHeight = absoluteHeightSceneryAssets.contains(
                    configPath.lexically_normal().generic_string());
                const double worldZ = openbus::map::mapSceneryWorldHeight(
                    localZ, terrainHeightOffset, absoluteHeight);
                if (batchIndexes.empty()) {
                    recordSkipped(skipped, "scenery placements with no renderable asset batches",
                                  description + " config=" + configPath.filename().string());
                }
                for (const std::size_t batchIndex : batchIndexes) {
                    if (batchIndex >= impl_->sceneryBatches.size()) {
                        continue;
                    }
                    const double radius = impl_->sceneryBatches[batchIndex].boundingRadius;
                    const Impl::SceneryInstance instance{
                        batchIndex, worldX, worldY, worldZ, radius, rotationDegrees, rotationCount};
                    const int chunkX =
                        static_cast<int>(std::floor(worldX / openbus::map::OMSI_TILE_SIZE_METERS));
                    const int chunkY =
                        static_cast<int>(std::floor(worldY / openbus::map::OMSI_TILE_SIZE_METERS));
                    Impl::SceneryChunk& chunk = impl_->sceneryChunks[tileKey(chunkX, chunkY)];
                    chunk.instances.push_back(instance);
                    chunk.minX = std::min(chunk.minX, worldX - radius);
                    chunk.minY = std::min(chunk.minY, worldY - radius);
                    chunk.minZ = std::min(chunk.minZ, worldZ - radius);
                    chunk.maxX = std::max(chunk.maxX, worldX + radius);
                    chunk.maxY = std::max(chunk.maxY, worldY + radius);
                    chunk.maxZ = std::max(chunk.maxZ, worldZ + radius);
                    ++impl_->sceneryInstanceCount;
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
            const std::optional<openbus::map::MapSceneryPose> pose =
                openbus::map::placeMapSceneryObject(object, tileX, tileY);
            if (!pose.has_value()) {
                recordSkipped(skipped, "[object] sections with invalid world placement",
                              objectDescription);
                continue;
            }
            const double worldX = pose->x;
            const double worldY = pose->y;
            const int terrainTileX =
                static_cast<int>(std::floor(worldX / openbus::map::OMSI_TILE_SIZE_METERS));
            const int terrainTileY =
                static_cast<int>(std::floor(worldY / openbus::map::OMSI_TILE_SIZE_METERS));
            const std::optional<double> ground = sampleTerrainHeight(
                terrainTileX, terrainTileY,
                worldX - static_cast<double>(terrainTileX) * openbus::map::OMSI_TILE_SIZE_METERS,
                worldY - static_cast<double>(terrainTileY) * openbus::map::OMSI_TILE_SIZE_METERS);
            addSceneryInstance(object.assetPath, objectDescription, worldX, worldY, pose->z,
                               ground.value_or(0.0), pose->rotationDegrees, pose->rotationCount);
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
                addSceneryInstance(attachment.assetPath, description, pose.x, pose.y, pose.z, 0.0,
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
        batch.minZ = geometry.minZ;
        batch.maxZ = geometry.maxZ;
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
                ", scenery instances=" + std::to_string(impl_->sceneryInstanceCount) +
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
    const MapFrustum frustum = buildMapFrustum(projectionMatrix());
    const double cameraX = -(view[0] * view[12] + view[1] * view[13] + view[2] * view[14]);
    const double cameraY = -(view[4] * view[12] + view[5] * view[13] + view[6] * view[14]);
    const auto outsideVisibilityBounds = [&](double minX, double minY, double maxX, double maxY) {
        return squaredDistanceToBounds(cameraX, cameraY, minX, minY, maxX, maxY) >
               MAP_VISIBILITY_RADIUS_SQUARED;
    };
    const auto outsideFrustumBounds = [&](double minX, double minY, double minZ, double maxX,
                                          double maxY, double maxZ) {
        const std::array<double, 3> center = {(minX + maxX) * 0.5, (minY + maxY) * 0.5,
                                              (minZ + maxZ) * 0.5};
        const double halfX = (maxX - minX) * 0.5;
        const double halfY = (maxY - minY) * 0.5;
        const double halfZ = (maxZ - minZ) * 0.5;
        return outsideMapFrustum(frustum, view, center,
                                 std::sqrt(halfX * halfX + halfY * halfY + halfZ * halfZ));
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
        if (outsideFrustumBounds(minX, minY, tile.minZ, minX + openbus::map::OMSI_TILE_SIZE_METERS,
                                 minY + openbus::map::OMSI_TILE_SIZE_METERS, tile.maxZ)) {
            continue;
        }
        drawIndexedModelBatch(tile.buffer, tile.indexBuffer, tile.indexCount, material,
                              {1.0, 1.0, 1.0}, 1.0, 0);
        if (!tile.groundTextureLayers.empty()) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthFunc(GL_LEQUAL);
            glDepthMask(GL_FALSE);
            for (const ModelMaterial& layer : tile.groundTextureLayers) {
                drawIndexedModelBatch(tile.buffer, tile.indexBuffer, tile.indexCount, layer,
                                      {1.0, 1.0, 1.0}, 1.0, 2);
            }
            glDepthMask(GL_TRUE);
            glDepthFunc(GL_LESS);
            glDisable(GL_BLEND);
        }
    }
    for (const Impl::ModelBatch& road : impl_->roadBatches) {
        if (road.hasVisibilityBounds &&
            outsideVisibilityBounds(road.minX, road.minY, road.maxX, road.maxY)) {
            continue;
        }
        if (road.hasVisibilityBounds && outsideFrustumBounds(road.minX, road.minY, road.minZ,
                                                             road.maxX, road.maxY, road.maxZ)) {
            continue;
        }
        drawIndexedModelBatch(road.buffer, road.indexBuffer, road.indexCount, road.material,
                              road.color, 1.0, road.alphaMode);
    }
    for (const auto& [key, chunk] : impl_->sceneryChunks) {
        static_cast<void>(key);
        if (chunk.instances.empty() ||
            outsideVisibilityBounds(chunk.minX, chunk.minY, chunk.maxX, chunk.maxY) ||
            outsideFrustumBounds(chunk.minX, chunk.minY, chunk.minZ, chunk.maxX, chunk.maxY,
                                 chunk.maxZ)) {
            continue;
        }
        for (const Impl::SceneryInstance& instance : chunk.instances) {
            if (instance.batchIndex >= impl_->sceneryBatches.size() ||
                outsideVisibilityBounds(
                    instance.x - instance.boundingRadius, instance.y - instance.boundingRadius,
                    instance.x + instance.boundingRadius, instance.y + instance.boundingRadius) ||
                outsideMapFrustum(frustum, view, {instance.x, instance.y, instance.z},
                                  instance.boundingRadius)) {
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
            drawModelBatch(batch.buffer, batch.vertexCount, batch.material, batch.color, batch.alpha,
                           batch.alphaMode);
            popMatrix();
            if (blended) {
                glDepthMask(GL_TRUE);
                glDisable(GL_BLEND);
            }
        }
    }
}

} // namespace openbus::rendering