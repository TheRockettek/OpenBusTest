#include "MapRenderer.h"

#include "AssetPath.h"
#include "CoreRenderer.h"
#include "ComputeCulling.h"
#include "Environment.h"
#include "Logger.h"
#include "MapRoadGeometry.h"
#include "MapSceneryPlacement.h"
#include "MapSplineGeometry.h"
#include "MapSplineProfile.h"
#include "MapWaterGeometry.h"
#include "MapTileWindow.h"
#include "ModelConfigLoader.h"
#include "ObjLoader.h"
#include "O3DLoader.h"
#include "OpenGLFunctions.h"
#include "PerfTrace.h"
#include "SceneryObjectConfigLoader.h"
#include "SceneryMaterialBinding.h"
#include "ScriptRuntime.h"
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

#ifndef GL_CLAMP_TO_BORDER
#define GL_CLAMP_TO_BORDER 0x812D
#endif
#ifndef GL_MIRRORED_REPEAT
#define GL_MIRRORED_REPEAT 0x8370
#endif
#ifndef GL_MIRROR_CLAMP_TO_EDGE
#define GL_MIRROR_CLAMP_TO_EDGE 0x8743
#endif

extern Logger gameLog;

namespace openbus::rendering {

namespace {

constexpr std::size_t MAX_SKIP_EXAMPLES = 3;
constexpr double MAP_VISIBILITY_RADIUS_METERS = 10000.0;
constexpr int MAP_ACTIVE_TILE_RADIUS = 2;
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
        std::string message =
            "Map skip summary: skipped " + std::to_string(summary.count) + " " + reason;
        if (!summary.examples.empty()) {
            message += " (context logged when encountered)";
        }
        gameLog.Log(message);
    }
}

GLenum textureAddressModeToGl(TextureAddressMode mode) {
    switch (mode) {
    case TextureAddressMode::Clamp:
        return GL_CLAMP_TO_EDGE;
    case TextureAddressMode::Border:
        return GL_CLAMP_TO_BORDER;
    case TextureAddressMode::Mirror:
        return GL_MIRRORED_REPEAT;
    case TextureAddressMode::MirrorOnce:
        return GL_MIRROR_CLAMP_TO_EDGE;
    case TextureAddressMode::Repeat:
    default:
        return GL_REPEAT;
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
    int tileX = 0;
    int tileY = 0;
    int alphaMode = 0;
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
        const std::filesystem::path resolved =
            openbus::assets::resolveCaseInsensitivePath(candidate);
        if (std::filesystem::is_regular_file(resolved)) {
            return resolved;
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
        constexpr std::array<const char*, 6> textureExtensions = {".dds", ".bmp", ".tga",
                                                                  ".png", ".jpg", ".jpeg"};
        const bool supportedExtension =
            std::any_of(textureExtensions.begin(), textureExtensions.end(),
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

struct MapFrustum {
    std::array<std::array<double, 4>, 6> planes = {};
    std::array<double, 6> planeLengths = {};
};

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
                planes.planes[plane][column] += signs[plane][row] * projection[row + column * 4];
            }
        }
        const auto& coefficients = planes.planes[plane];
        planes.planeLengths[plane] =
            std::sqrt(coefficients[0] * coefficients[0] + coefficients[1] * coefficients[1] +
                      coefficients[2] * coefficients[2]);
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
    for (std::size_t index = 0; index < planes.planes.size(); ++index) {
        const std::array<double, 4>& plane = planes.planes[index];
        const double distance =
            plane[0] * eye[0] + plane[1] * eye[1] + plane[2] * eye[2] + plane[3];
        if (distance < -radius * planes.planeLengths[index]) {
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
                       const openbus::map::MapSplineProfileSection& section,
                       const openbus::map::MapRoadSplineJoins& joins,
                       std::optional<double> cantHalfWidth) {
    const openbus::map::MapRoadSectionGeometry geometry = openbus::map::buildMapRoadSectionGeometry(
        spline, centerline, section, joins, cantHalfWidth);
    const std::vector<std::array<double, 3>> normals =
        openbus::map::computeMapRoadVertexNormals(geometry);
    const std::uint32_t baseVertex = static_cast<std::uint32_t>(road.vertices.size());
    for (std::size_t vertexIndex = 0; vertexIndex < geometry.vertices.size(); ++vertexIndex) {
        const openbus::map::MapRoadVertex& vertex = geometry.vertices[vertexIndex];
        const std::array<double, 3>& normal = normals[vertexIndex];
        const ModelVertex generated{
            static_cast<float>(vertex.x),        static_cast<float>(vertex.y),
            static_cast<float>(vertex.z),        static_cast<float>(vertex.textureU),
            static_cast<float>(vertex.textureV), 0.0F,
            static_cast<float>(normal[0]),       static_cast<float>(normal[1]),
            static_cast<float>(normal[2])};
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
    struct DynamicSceneryTexture {
        GLuint texture = 0;
        int width = 0;
        int height = 0;
        std::uint64_t revision = 0;
        bool filtered = false;
        GLenum wrapS = 0;
        GLenum wrapT = 0;
    };

    struct TileBuffer {
        GLuint buffer = 0;
        GLuint indexBuffer = 0;
        std::size_t indexCount = 0;
        int tileX = 0;
        int tileY = 0;
        double minZ = 0.0;
        double maxZ = 0.0;
        std::array<float, 2> baseTextureScale = {1.0F, 1.0F};
        std::array<float, 2> baseDetailTextureScale = {1.0F, 1.0F};
        std::vector<ModelMaterial> groundTextureLayers;
    };

    struct WaterBatch {
        GLuint buffer = 0;
        GLuint indexBuffer = 0;
        std::size_t indexCount = 0;
        int tileX = 0;
        int tileY = 0;
        double minZ = 0.0;
        double maxZ = 0.0;
        double meanZ = 0.0;
    };

    struct ModelBatch {
        GLuint buffer = 0;
        GLuint indexBuffer = 0;
        std::size_t vertexCount = 0;
        std::size_t indexCount = 0;
        ModelMaterial material;
        int scriptTextureIndex = -1;
        int textTextureIndex = -1;
        std::string freeTextureName;
        std::string freeTextureVariable;
        std::filesystem::path textureObjectRoot;
        std::filesystem::path textureModelRoot;
        std::array<double, 3> color = {1.0, 1.0, 1.0};
        double alpha = 1.0;
        int alphaMode = 0;
        int renderType = 2;
        bool noZwrite = false;
        bool noZcheck = false;
        std::string visibilityVariable;
        int visibilityValue = 0;
        bool hasVisibilityBounds = false;
        double minX = 0.0;
        double minY = 0.0;
        double maxX = 0.0;
        double maxY = 0.0;
        double minZ = 0.0;
        double maxZ = 0.0;
        double boundingRadius = 0.0;
        bool cullBackFaces = false;
        int tileX = 0;
        int tileY = 0;
        std::size_t sceneryLodIndex = 0;
    };

    struct SceneryInstance {
        std::size_t batchIndex = 0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double boundingRadius = 0.0;
        std::array<double, 6> rotationDegrees = {};
        std::size_t rotationCount = 3;
        std::array<double, 9> orientationMatrix = {};
        bool hasOrientationMatrix = false;
        std::size_t scriptStateIndex = std::numeric_limits<std::size_t>::max();
        std::size_t lodThresholdSetIndex = 0;
        int sourceTileX = 0;
        int sourceTileY = 0;
    };

    struct SceneryRuntimeDefinition {
        VehicleConfig configuration;
        std::vector<ModelScriptTexture> scriptTextures;
        std::vector<ModelTextTexture> textTextures;
        std::vector<std::string> floatVariables;
        std::vector<std::string> stringVariables;
        bool hasRuntime = false;
    };

    struct SceneryScriptState {
        openbus::scripting::SceneryObject variables;
        std::unique_ptr<ScriptRuntime> runtime;
        std::shared_ptr<SceneryRuntimeDefinition> definition;
        std::map<std::pair<bool, int>, DynamicSceneryTexture> dynamicTextures;
        std::vector<std::string> placementLabels;
        int ownerTileX = 0;
        int ownerTileY = 0;
        bool runtimeInitialized = false;
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
    GLuint baseDetailTexture = 0;
    std::vector<TileBuffer> terrainBuffers;
    std::unordered_map<std::uint64_t, openbus::map::TerrainGrid> terrainByCoordinate;
    std::vector<WaterBatch> waterBatches;
    std::vector<ModelBatch> sceneryBatches;
    std::vector<std::vector<double>> sceneryLodThresholdSets;
    std::vector<std::unique_ptr<SceneryScriptState>> sceneryScriptStates;
    std::vector<std::size_t> activeSceneryScriptStateIndices;
    std::size_t sceneryScriptRuntimeCount = 0;
    std::unordered_map<std::uint64_t, SceneryChunk> sceneryChunks;
    std::size_t sceneryInstanceCount = 0;
    std::size_t sceneryObjectCount = 0;
    std::vector<ModelBatch> roadBatches;
    std::unordered_map<std::uint64_t, std::size_t> terrainTilesByTile;
    std::unordered_map<std::uint64_t, std::size_t> sceneryObjectsByTile;
    std::unordered_map<std::uint64_t, std::size_t> roadBatchesByTile;
    MapRenderStats renderStats;
    std::vector<GLuint> ownedTextures;
    std::filesystem::path omsiRoot;
    std::unordered_map<std::string, GLuint> dynamicSceneryTextures;
    std::unordered_set<std::string> unavailableSceneryTextures;
    std::vector<openbus::map::MapTileCoordinate> mapTileCoordinates;
    std::vector<openbus::map::MapTileCoordinate> activeTileCoordinates;
    std::unordered_set<std::uint64_t> activeTileKeys;
    std::optional<openbus::map::MapTileCoordinate> activeCenterTile;
    SimulationState* simulationState = nullptr;

    ~Impl() {
        for (const TileBuffer& tile : terrainBuffers) {
            if (tile.buffer != 0 && pglDeleteBuffers != nullptr) {
                pglDeleteBuffers(1, &tile.buffer);
            }
            if (tile.indexBuffer != 0 && pglDeleteBuffers != nullptr) {
                pglDeleteBuffers(1, &tile.indexBuffer);
            }
        }
        for (const WaterBatch& water : waterBatches) {
            if (water.buffer != 0 && pglDeleteBuffers != nullptr) {
                pglDeleteBuffers(1, &water.buffer);
            }
            if (water.indexBuffer != 0 && pglDeleteBuffers != nullptr) {
                pglDeleteBuffers(1, &water.indexBuffer);
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
                         double spawnWorldX, double spawnWorldY, std::size_t groundTextureIndex,
                         const std::filesystem::path& omsiRoot, SimulationState& simulationState)
    : impl_(std::make_unique<Impl>()) {
    TraceScope trace("map", "MapRenderer::MapRenderer");
    impl_->simulationState = &simulationState;
    impl_->omsiRoot = omsiRoot;
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
    impl_->mapTileCoordinates.reserve(map.tiles.size());
    for (const openbus::map::MapTileReference& tile : map.tiles) {
        impl_->mapTileCoordinates.emplace_back(tile.x, tile.y);
    }

    const char* groundTextureOverride = openbus::getEnvironment("OPENBUS_MAP_GROUNDTEX");
    const bool uniformGroundTexturePreview =
        groundTextureOverride != nullptr && *groundTextureOverride != '\0';
    const std::size_t baseGroundTextureIndex = uniformGroundTexturePreview ? groundTextureIndex : 0;
    const double baseTextureRepeatsPerTile =
        openbus::map::mapGroundTextureTiling(map.groundTextures[baseGroundTextureIndex])
            .textureRepeatsPerTile;
    const std::filesystem::path texturePath =
        groundTexturePath(map, omsiRoot, map.groundTextures[baseGroundTextureIndex].texturePath);
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

    auto& terrainByCoordinate = impl_->terrainByCoordinate;
    terrainByCoordinate.reserve(map.tiles.size());
    SkipSummaries skipped;
    std::unordered_map<std::string, GLuint> textureCache;
    const auto getTexture = [&](const std::filesystem::path& path, bool repeat = true) {
        const std::string key =
            path.lexically_normal().generic_string() + (repeat ? "|repeat" : "|clamp");
        const auto found = textureCache.find(key);
        if (found != textureCache.end()) {
            return found->second;
        }
        const GLuint texture = loadTexture(path, repeat);
        textureCache.emplace(key, texture);
        impl_->ownedTextures.push_back(texture);
        return texture;
    };
    if (!uniformGroundTexturePreview) {
        const std::string& detailTextureName =
            map.groundTextures[baseGroundTextureIndex].detailTexturePath;
        if (!detailTextureName.empty()) {
            try {
                impl_->baseDetailTexture =
                    getTexture(groundTexturePath(map, omsiRoot, detailTextureName));
            } catch (const std::exception& error) {
                recordSkipped(skipped, "base ground-texture detail textures that failed to load",
                              detailTextureName + ": " + error.what());
            }
        }
    }
    const openbus::map::MapTileReference& center = map.tiles[centerTileIndex];
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
                    // Match MapRenderer's GL_CW front-face convention so terrain can
                    // use the same one-sided shader rule as other model geometry.
                    indices.insert(indices.end(), {topLeft, bottomRight, topRight, topLeft,
                                                   bottomLeft, bottomRight});
                }
            }
            Impl::TileBuffer buffer;
            buffer.indexCount = indices.size();
            buffer.tileX = tile.x;
            buffer.tileY = tile.y;
            const float inverseIntervals = 1.0F / static_cast<float>(grid.intervals);
            const float baseTextureScale =
                static_cast<float>(baseTextureRepeatsPerTile) * inverseIntervals;
            buffer.baseTextureScale = {baseTextureScale, baseTextureScale};
            if (impl_->baseDetailTexture != 0) {
                const double baseDetailRepeatsPerTile =
                    openbus::map::mapGroundTextureTiling(map.groundTextures[baseGroundTextureIndex])
                        .detailTextureRepeatsPerTile;
                const float baseDetailScale =
                    static_cast<float>(baseDetailRepeatsPerTile) * inverseIntervals;
                buffer.baseDetailTextureScale = {baseDetailScale, baseDetailScale};
            }
            if (!uniformGroundTexturePreview) {
                std::map<std::size_t, const openbus::map::MapGroundTextureSidecar*> layersByIndex;
                for (const openbus::map::MapGroundTextureSidecar& sidecar :
                     tile.groundTextureSidecars) {
                    const std::optional<std::size_t> resolvedIndex =
                        openbus::map::resolveMapGroundTextureSidecarIndex(
                            sidecar.ordinalSuffix, map.groundTextures.size());
                    if (!resolvedIndex) {
                        recordSkipped(skipped,
                                      "ground-texture sidecars with invalid groundtex indices",
                                      tile.textPath.filename().string() +
                                          " sidecar=" + sidecar.path.filename().string());
                        continue;
                    }
                    const std::size_t textureIndex = *resolvedIndex;
                    if (!layersByIndex.emplace(textureIndex, &sidecar).second) {
                        recordSkipped(skipped, "duplicate ground-texture sidecar indices",
                                      tile.textPath.filename().string() +
                                          " index=" + std::to_string(textureIndex));
                    }
                }
                for (const auto& [textureIndex, sidecar] : layersByIndex) {
                    try {
                        ModelMaterial layer;
                        const std::filesystem::path layerTexturePath = groundTexturePath(
                            map, omsiRoot, map.groundTextures[textureIndex].texturePath);
                        layer.texture = getTexture(layerTexturePath);
                        layer.textured = true;
                        const openbus::map::MapGroundTextureTiling tiling =
                            openbus::map::mapGroundTextureTiling(map.groundTextures[textureIndex]);
                        const double layerRepeatsPerTile = tiling.textureRepeatsPerTile;
                        const float layerTextureScale =
                            static_cast<float>(layerRepeatsPerTile) * inverseIntervals;
                        layer.textureScale = {layerTextureScale, layerTextureScale};
                        const std::string& detailTextureName =
                            map.groundTextures[textureIndex].detailTexturePath;
                        if (!detailTextureName.empty()) {
                            try {
                                const std::filesystem::path detailTexturePath =
                                    groundTexturePath(map, omsiRoot, detailTextureName);
                                layer.detailTexture = getTexture(detailTexturePath);
                                layer.useDetailTexture = true;
                                const float detailTextureScale =
                                    static_cast<float>(tiling.detailTextureRepeatsPerTile) *
                                    inverseIntervals;
                                layer.detailTextureScale = {detailTextureScale, detailTextureScale};
                            } catch (const std::exception& error) {
                                recordSkipped(
                                    skipped, "ground-texture detail textures that failed to load",
                                    sidecar->path.filename().string() +
                                        " detail=" + detailTextureName + ": " + error.what());
                            }
                        }
                        layer.transmap = getTexture(sidecar->path, false);
                        layer.useTransmap = true;
                        // OMSI ground-texture DDS rows run opposite the terrain mesh's map
                        // northing.
                        layer.flipTransmapY = true;
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
            ++impl_->terrainTilesByTile[tileKey(buffer.tileX, buffer.tileY)];
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
        if (found == terrainByCoordinate.end()) {
            return std::nullopt;
        }
        return openbus::map::sampleMapTerrainHeight(found->second, localX, localY);
    };

    const auto sampleTerrainNormal = [&](double worldX,
                                         double worldY) -> std::optional<std::array<double, 3>> {
        const int tileX =
            static_cast<int>(std::floor(worldX / openbus::map::OMSI_TILE_SIZE_METERS));
        const int tileY =
            static_cast<int>(std::floor(worldY / openbus::map::OMSI_TILE_SIZE_METERS));
        const auto found = terrainByCoordinate.find(tileKey(tileX, tileY));
        if (found == terrainByCoordinate.end()) {
            return std::nullopt;
        }
        const double localX =
            worldX - static_cast<double>(tileX) * openbus::map::OMSI_TILE_SIZE_METERS;
        const double localY =
            worldY - static_cast<double>(tileY) * openbus::map::OMSI_TILE_SIZE_METERS;
        return openbus::map::sampleMapTerrainNormal(found->second, localX, localY);
    };

    std::unordered_map<std::string, std::vector<std::size_t>> sceneryAssetBatches;
    std::unordered_map<std::string, std::size_t> sceneryLodThresholdSetByAsset;
    std::unordered_map<std::string, std::vector<ModelAttachmentPoint>> sceneryAttachmentPoints;
    std::unordered_map<std::string, VehicleConfig> sceneryScriptConfigurations;
    std::unordered_map<std::string, std::vector<ModelScriptTexture>> sceneryModelScriptTextures;
    std::unordered_map<std::string, std::vector<ModelTextTexture>> sceneryModelTextTextures;
    std::unordered_map<std::string, std::vector<std::string>> sceneryModelFloatVariables;
    std::unordered_map<std::string, std::vector<std::string>> sceneryModelStringVariables;
    std::unordered_map<std::string, std::vector<std::string>> sceneryFreeTextureVariables;
    std::unordered_map<std::string, std::shared_ptr<Impl::SceneryRuntimeDefinition>>
        sceneryRuntimeDefinitions;
    std::unordered_set<std::string> sceneryAssetsNeedingVariables;
    std::unordered_set<std::string> editorOnlySceneryAssets;
    std::unordered_set<std::string> absoluteHeightSceneryAssets;
    const bool verboseAlphaMaterials =
        parseEnabledFlag(openbus::getEnvironment("OPENBUS_VERBOSE_TEXTURE_READ"));
    const auto resolveSceneryConfigPath = [&](const std::string& assetPath) {
        const std::filesystem::path relative = normalizedAssetPath(assetPath);
        const std::array<std::filesystem::path, 2> candidates = {omsiRoot / relative,
                                                                 map.rootPath / relative};
        for (const std::filesystem::path& candidate : candidates) {
            const std::filesystem::path resolved =
                openbus::assets::resolveCaseInsensitivePath(candidate);
            if (std::filesystem::is_regular_file(resolved)) {
                return resolved;
            }
        }
        return std::filesystem::path{};
    };
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
            sceneryAttachmentPoints[cacheKey] = objectConfiguration.attachmentPoints;
            sceneryScriptConfigurations[cacheKey] = objectConfiguration.scriptConfiguration;
            if (!objectConfiguration.scriptConfiguration.scripts.empty()) {
                sceneryAssetsNeedingVariables.insert(cacheKey);
            }
            // Tree SCOs may mark their helper mesh [onlyeditor] while their [tree]
            // definition is runtime scenery. The tree preview below replaces that mesh.
            if (objectConfiguration.onlyEditor && objectConfiguration.trees.empty()) {
                editorOnlySceneryAssets.insert(cacheKey);
                return sceneryAssetBatches.emplace(cacheKey, std::move(batchIndexes)).first->second;
            }
            std::filesystem::path modelConfigPath =
                resolveSceneryObjectModelConfigPath(objectConfiguration);
            bool usesExternalModelConfig = !objectConfiguration.modelPath.empty();
            if (usesExternalModelConfig && !std::filesystem::is_regular_file(modelConfigPath)) {
                recordSkipped(skipped, "scenery external model configs not found",
                              configPath.filename().string() +
                                  " model=" + modelConfigPath.generic_string());
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
                    batch.renderType = objectConfiguration.renderType;
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
            const ModelConfig configuration = loadModelConfig(
                modelConfigPath, modelRoot, ModelConfigKind::SceneryObject, variables);
            const std::size_t lodThresholdSetIndex = impl_->sceneryLodThresholdSets.size();
            impl_->sceneryLodThresholdSets.push_back(configuration.lodThresholds);
            sceneryLodThresholdSetByAsset.emplace(cacheKey, lodThresholdSetIndex);
            for (const auto& [name, value] : variables.numericValues()) {
                static_cast<void>(value);
                sceneryModelFloatVariables[cacheKey].push_back(name);
            }
            for (const auto& [name, value] : variables.stringValues()) {
                static_cast<void>(value);
                sceneryModelStringVariables[cacheKey].push_back(name);
            }
            sceneryModelScriptTextures[cacheKey] = configuration.scriptTextures;
            sceneryModelTextTextures[cacheKey] = configuration.textTextures;
            if (!configuration.scriptTextures.empty() || !configuration.textTextures.empty()) {
                sceneryAssetsNeedingVariables.insert(cacheKey);
            }
            if (sceneryAttachmentPoints[cacheKey].empty()) {
                sceneryAttachmentPoints[cacheKey] = configuration.attachmentPoints;
            }
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
                if (!part.visibleVariable.empty()) {
                    sceneryAssetsNeedingVariables.insert(cacheKey);
                }
                for (const ModelMaterialState& state : part.materialStatesInOrder) {
                    if (state.scriptTextureIndex >= 0 || state.textTextureIndex >= 0) {
                        sceneryAssetsNeedingVariables.insert(cacheKey);
                    }
                    if (!state.freeTextureVariable.empty()) {
                        sceneryAssetsNeedingVariables.insert(cacheKey);
                        auto& stringVariables = sceneryModelStringVariables[cacheKey];
                        if (std::find(stringVariables.begin(), stringVariables.end(),
                                      state.freeTextureVariable) == stringVariables.end()) {
                            stringVariables.push_back(state.freeTextureVariable);
                        }
                        sceneryFreeTextureVariables[cacheKey].push_back(state.freeTextureVariable);
                    }
                }
                if (part.objPath.empty()) {
                    recordSkipped(skipped, "scenery model parts with no resolved mesh path",
                                  modelConfigPath.filename().string());
                    continue;
                }
                std::string meshExtension = part.objPath.extension().string();
                std::transform(
                    meshExtension.begin(), meshExtension.end(), meshExtension.begin(),
                    [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
                if (meshExtension != ".o3d" && meshExtension != ".x") {
                    recordSkipped(skipped, "scenery model parts with unsupported mesh formats",
                                  modelConfigPath.filename().string() + " " +
                                      part.objPath.generic_string());
                    continue;
                }
                const std::shared_ptr<ParsedObj> parsed = meshExtension == ".x"
                                                              ? XLoader::parse(part.objPath)
                                                              : O3DLoader::parse(part.objPath);
                if (!parsed) {
                    recordSkipped(skipped, "scenery O3D meshes that failed to parse",
                                  configPath.filename().string() + " " +
                                      part.objPath.filename().string());
                    continue;
                }
                std::unordered_map<int, std::vector<ModelVertex>> verticesByMaterial;
                const auto statesByMaterial =
                    openbus::map::resolveSceneryMaterialStates(*parsed, part);
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
                            v = static_cast<float>(1.0 - coordinate.v);
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
                    if (state != nullptr && !state->freeTextureName.empty()) {
                        const std::filesystem::path freeTexture = findSceneryTexture(
                            objectRoot, modelRoot, omsiRoot, state->freeTextureName);
                        if (freeTexture.empty()) {
                            recordSkipped(skipped, "scenery free textures not found",
                                          modelConfigPath.filename().string() +
                                              " mesh=" + part.objPath.filename().string() +
                                              " texture=" + state->freeTextureName);
                        } else {
                            material.freeTexture = getTexture(freeTexture);
                            material.useFreeTexture = material.freeTexture != 0;
                        }
                    }
                    Impl::ModelBatch batch;
                    batch.cullBackFaces = true;
                    batch.vertexCount = vertices.size();
                    batch.boundingRadius = vertexBoundingRadius(vertices);
                    batch.sceneryLodIndex =
                        part.lodIndex < 0 ? 0U : static_cast<std::size_t>(part.lodIndex);
                    batch.material = material;
                    if (state != nullptr) {
                        batch.scriptTextureIndex = state->scriptTextureIndex;
                        batch.textTextureIndex = state->textTextureIndex;
                        batch.freeTextureName = state->freeTextureName;
                        batch.freeTextureVariable = state->freeTextureVariable;
                        batch.textureObjectRoot = objectRoot;
                        batch.textureModelRoot = modelRoot;
                        batch.material.textureWrapS =
                            textureAddressModeToGl(state->textureAddressS);
                        batch.material.textureWrapT =
                            textureAddressModeToGl(state->textureAddressT);
                    }
                    batch.color = color;
                    batch.alpha = sourceMaterial == parsed->materials.end()
                                      ? 1.0
                                      : std::clamp(sourceMaterial->second.alpha, 0.0, 1.0);
                    batch.alphaMode = state == nullptr ? 0 : state->alphaMode;
                    batch.noZwrite = state != nullptr && state->noZwrite;
                    batch.noZcheck = state != nullptr && state->noZcheck;
                    batch.renderType = resolveSceneryRenderType(part, objectConfiguration.renderType);
                    batch.visibilityVariable = part.visibleVariable;
                    batch.visibilityValue = part.visibleValue;
                    if (batch.alphaMode == 0 && batch.alpha < 1.0 - 1.0e-6) {
                        batch.alphaMode = 2;
                    }
                    if (verboseAlphaMaterials && batch.alphaMode != 0) {
                        gameLog.Log("Map scenery alpha material: asset=" +
                                    modelConfigPath.filename().string() +
                                    " mesh=" + part.objPath.filename().string() +
                                    " index=" + std::to_string(materialIndex) +
                                    " mode=" + std::to_string(batch.alphaMode) + " state=" +
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
    const auto cachedSplineProfile =
        [&](const std::filesystem::path& path) -> const openbus::map::MapSplineProfile* {
        if (path.empty()) {
            return nullptr;
        }
        const std::string key = path.lexically_normal().generic_string();
        if (failedSplineProfiles.contains(key)) {
            return nullptr;
        }
        auto found = splineProfiles.find(key);
        if (found == splineProfiles.end()) {
            try {
                found = splineProfiles.emplace(key, openbus::map::loadMapSplineProfile(path)).first;
            } catch (const std::exception& error) {
                failedSplineProfiles.insert(key);
                recordSkipped(skipped, "spline profile files that failed to parse",
                              path.filename().string() + ": " + error.what());
                return nullptr;
            }
        }
        return &found->second;
    };
    // Share profile metadata across join indexing, row/parent placement, and road generation.
    // Each asset is resolved/parsed once, including absent and rejected assets.
    std::unordered_map<std::string, std::optional<double>> splineCantWidths;
    const openbus::map::MapSplineCantWidthResolver splineCantWidth =
        [&](const openbus::map::MapSplinePlacement& spline) -> std::optional<double> {
        const auto found = splineCantWidths.find(spline.assetPath);
        if (found != splineCantWidths.end()) {
            return found->second;
        }
        const auto* profile =
            cachedSplineProfile(resolveSplineProfile(map, omsiRoot, spline.assetPath));
        const std::optional<double> width =
            profile != nullptr ? openbus::map::mapSplineProfileCantHalfWidth(*profile)
                               : std::nullopt;
        splineCantWidths.emplace(spline.assetPath, width);
        return width;
    };
    const auto mapContentStart = std::chrono::steady_clock::now();
    std::size_t tilesProcessed = 0;
    std::size_t sceneryPlacementsParsed = 0;
    std::size_t splinePlacementsParsed = 0;
    TraceScope sceneBuildPhase("map", "MapRenderer.buildSceneGeometryAndBatches");

    struct IndexedSceneryObject {
        std::string assetPath;
        openbus::map::MapSceneryPose pose;
        bool transformValid = false;
        bool splineTerrainAlign = false;
    };
    struct IndexedAttachedObject {
        std::string assetPath;
        int parentId = 0;
        int attachmentIndex = 0;
        std::array<double, 3> rotationDegrees = {};
        bool transformValid = false;
        bool splineTerrainAlign = false;
    };
    std::unordered_map<int, IndexedSceneryObject> sceneryObjectsById;
    std::unordered_map<int, IndexedAttachedObject> attachedObjectsById;
    std::unordered_map<int, std::size_t> splineAttachmentTileById;
    std::unordered_map<int, openbus::map::MapSplinePlacementReference> mapSplinesById;
    std::unordered_set<int> ambiguousMapSplineIds;
    std::unordered_map<std::uint64_t, std::unordered_set<int>> editorObjectNumbersByTile;
    openbus::map::MapRoadConnectionIndex roadConnections;
    for (std::size_t tileReferenceIndex = 0; tileReferenceIndex < map.tiles.size();
         ++tileReferenceIndex) {
        const openbus::map::MapTileReference& tileReference = map.tiles[tileReferenceIndex];
        const openbus::map::MapTileData tile = openbus::map::loadMapTile(tileReference);
        const std::uint64_t currentTileKey = tileKey(tileReference.x, tileReference.y);
        for (const auto& spline : tile.splines) {
            roadConnections.addSpline(spline, tileReference.x, tileReference.y,
                                      spline.geometryValid ? splineCantWidth(spline)
                                                           : std::nullopt);
            if (spline.splineId != 0 && !ambiguousMapSplineIds.contains(spline.splineId)) {
                const auto [existing, inserted] = mapSplinesById.try_emplace(
                    spline.splineId, openbus::map::MapSplinePlacementReference{
                                         spline, tileReference.x, tileReference.y});
                if (!inserted) {
                    mapSplinesById.erase(existing);
                    ambiguousMapSplineIds.insert(spline.splineId);
                }
            }
        }
        for (const openbus::map::MapSceneryPlacement& object : tile.sceneryObjects) {
            IndexedSceneryObject indexed;
            indexed.assetPath = object.assetPath;
            indexed.transformValid = object.transformValid;
            indexed.splineTerrainAlign = object.splineTerrainAlign;
            if (const auto pose =
                    openbus::map::placeMapSceneryObject(object, tileReference.x, tileReference.y)) {
                indexed.pose = *pose;
            }
            if (!sceneryObjectsById.try_emplace(object.id, std::move(indexed)).second) {
                recordSkipped(skipped, "map placements with duplicate object IDs",
                              tileLabel(tileReference.x, tileReference.y) +
                                  " object ID=" + std::to_string(object.id));
            }
            if (object.editorObjectNumber.has_value()) {
                editorObjectNumbersByTile[currentTileKey].insert(*object.editorObjectNumber);
            }
        }
        for (const openbus::map::MapAttachedObject& object : tile.attachedObjects) {
            IndexedAttachedObject indexed;
            indexed.assetPath = object.assetPath;
            indexed.parentId = object.attachedToObjectId;
            indexed.attachmentIndex = object.attachPointIndexValue;
            indexed.rotationDegrees = object.rotationDegrees;
            indexed.transformValid = object.transformValid;
            indexed.splineTerrainAlign = object.splineTerrainAlign;
            if (!attachedObjectsById.try_emplace(object.id, std::move(indexed)).second) {
                recordSkipped(skipped, "map placements with duplicate object IDs",
                              tileLabel(tileReference.x, tileReference.y) +
                                  " attached object ID=" + std::to_string(object.id));
            }
            if (object.editorObjectNumber.has_value()) {
                editorObjectNumbersByTile[currentTileKey].insert(*object.editorObjectNumber);
            }
        }
        for (const openbus::map::MapSplineAttachment& attachment : tile.splineAttachments) {
            if (!splineAttachmentTileById.try_emplace(attachment.id, tileReferenceIndex).second) {
                recordSkipped(skipped, "map placements with duplicate object IDs",
                              tileLabel(tileReference.x, tileReference.y) +
                                  " spline attachment ID=" + std::to_string(attachment.id));
            }
            if (attachment.editorObjectNumber.has_value()) {
                editorObjectNumbersByTile[currentTileKey].insert(*attachment.editorObjectNumber);
            }
        }
    }

    const openbus::map::MapSplineChainResolver mapSplineResolver =
        [&mapSplinesById](
            int splineId) -> std::optional<openbus::map::MapSplinePlacementReference> {
        const auto found = mapSplinesById.find(splineId);
        if (found == mapSplinesById.end()) {
            return std::nullopt;
        }
        return found->second;
    };

    struct ResolvedSceneryPlacement {
        std::string assetPath;
        openbus::map::MapSceneryPose pose;
    };
    roadConnections.finalize();
    std::unordered_map<int, ResolvedSceneryPlacement> resolvedPlacements;
    std::unordered_map<std::size_t, openbus::map::MapTileData> splineAttachmentParentTiles;
    std::unordered_set<int> resolvingPlacementIds;
    const auto applyTerrainAlignment = [&](openbus::map::MapSceneryPose& pose, bool requested,
                                           const std::string& description) {
        if (!requested) {
            return;
        }
        const std::optional<std::array<double, 3>> normal = sampleTerrainNormal(pose.x, pose.y);
        if (!normal || !openbus::map::alignMapSceneryPoseToTerrainNormal(pose, *normal)) {
            recordSkipped(skipped,
                          "[spline_terrain_align] placements without a usable terrain normal",
                          description);
        }
    };
    const auto resolveMapSceneryPlacement =
        [&](int objectId, auto&& resolveSelf) -> std::optional<ResolvedSceneryPlacement> {
        const auto cached = resolvedPlacements.find(objectId);
        if (cached != resolvedPlacements.end()) {
            return cached->second;
        }
        if (!resolvingPlacementIds.insert(objectId).second) {
            recordSkipped(skipped, "[attachObj] parent cycles",
                          "object ID=" + std::to_string(objectId));
            return std::nullopt;
        }
        struct ResolveGuard {
            std::unordered_set<int>& active;
            int id;
            ~ResolveGuard() {
                active.erase(id);
            }
        } guard{resolvingPlacementIds, objectId};

        const auto ordinary = sceneryObjectsById.find(objectId);
        if (ordinary != sceneryObjectsById.end()) {
            const IndexedSceneryObject& object = ordinary->second;
            if (!object.transformValid) {
                return std::nullopt;
            }
            openbus::map::MapSceneryPose pose = object.pose;
            const int terrainTileX =
                static_cast<int>(std::floor(pose.x / openbus::map::OMSI_TILE_SIZE_METERS));
            const int terrainTileY =
                static_cast<int>(std::floor(pose.y / openbus::map::OMSI_TILE_SIZE_METERS));
            const std::optional<double> ground = sampleTerrainHeight(
                terrainTileX, terrainTileY,
                pose.x - static_cast<double>(terrainTileX) * openbus::map::OMSI_TILE_SIZE_METERS,
                pose.y - static_cast<double>(terrainTileY) * openbus::map::OMSI_TILE_SIZE_METERS);
            const std::filesystem::path configPath = resolveSceneryConfigPath(object.assetPath);
            bool absoluteHeight = false;
            if (!configPath.empty()) {
                static_cast<void>(loadSceneryAsset(configPath));
                absoluteHeight = absoluteHeightSceneryAssets.contains(
                    configPath.lexically_normal().generic_string());
            }
            pose.z =
                openbus::map::mapSceneryWorldHeight(pose.z, ground.value_or(0.0), absoluteHeight);
            applyTerrainAlignment(pose, object.splineTerrainAlign,
                                  "object ID=" + std::to_string(objectId));
            ResolvedSceneryPlacement resolved{object.assetPath, pose};
            resolvedPlacements.emplace(objectId, resolved);
            return resolved;
        }

        const auto attached = attachedObjectsById.find(objectId);
        if (attached != attachedObjectsById.end()) {
            if (!attached->second.transformValid) {
                return std::nullopt;
            }
            const IndexedAttachedObject& object = attached->second;
            const std::optional<ResolvedSceneryPlacement> parent =
                resolveSelf(object.parentId, resolveSelf);
            if (!parent.has_value()) {
                return std::nullopt;
            }
            ModelAttachmentPoint attachment;
            if (object.attachmentIndex < 0) {
                return std::nullopt;
            }
            const std::filesystem::path parentConfig = resolveSceneryConfigPath(parent->assetPath);
            if (!parentConfig.empty()) {
                static_cast<void>(loadSceneryAsset(parentConfig));
                const auto attachmentDefinitions =
                    sceneryAttachmentPoints.find(parentConfig.lexically_normal().generic_string());
                if (attachmentDefinitions != sceneryAttachmentPoints.end() &&
                    !attachmentDefinitions->second.empty()) {
                    const std::optional<std::size_t> attachmentOffset =
                        openbus::map::mapSceneryAttachmentPointOffset(
                            object.attachmentIndex, attachmentDefinitions->second.size());
                    if (!attachmentOffset.has_value()) {
                        return std::nullopt;
                    }
                    attachment = attachmentDefinitions->second[*attachmentOffset];
                } else if (object.attachmentIndex != 0) {
                    return std::nullopt;
                }
            } else if (object.attachmentIndex != 0) {
                return std::nullopt;
            }
            ResolvedSceneryPlacement resolved;
            resolved.assetPath = object.assetPath;
            resolved.pose = openbus::map::composeMapAttachedObjectPose(
                parent->pose, attachment.translation, attachment.rotationDegrees,
                object.rotationDegrees);
            applyTerrainAlignment(resolved.pose, object.splineTerrainAlign,
                                  "attached object ID=" + std::to_string(objectId));
            resolvedPlacements.emplace(objectId, resolved);
            return resolved;
        }

        const auto splineAttachment = splineAttachmentTileById.find(objectId);
        if (splineAttachment == splineAttachmentTileById.end() ||
            splineAttachment->second >= map.tiles.size()) {
            return std::nullopt;
        }
        auto parentTile = splineAttachmentParentTiles.find(splineAttachment->second);
        if (parentTile == splineAttachmentParentTiles.end()) {
            const std::size_t tileIndex = splineAttachment->second;
            parentTile = splineAttachmentParentTiles
                             .emplace(tileIndex, openbus::map::loadMapTile(map.tiles[tileIndex]))
                             .first;
        }
        const auto& splineAttachmentRecord = parentTile->second.splineAttachments;
        const auto record =
            std::find_if(splineAttachmentRecord.begin(), splineAttachmentRecord.end(),
                         [objectId](const openbus::map::MapSplineAttachment& candidate) {
                             return candidate.id == objectId;
                         });
        if (record == splineAttachmentRecord.end()) {
            return std::nullopt;
        }
        const std::optional<openbus::map::MapSceneryPose> parentPose =
            openbus::map::placeMapSplineAttachmentParent(
                *record, parentTile->second.splines, parentTile->second.reference.x,
                parentTile->second.reference.y, mapSplineResolver, splineCantWidth);
        if (!parentPose.has_value()) {
            return std::nullopt;
        }
        ResolvedSceneryPlacement resolved{record->assetPath, *parentPose};
        applyTerrainAlignment(resolved.pose, record->splineTerrainAlign,
                              "spline attachment parent ID=" + std::to_string(objectId));
        resolvedPlacements.emplace(objectId, resolved);
        return resolved;
    };

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
        const auto reportVariableParent = [&](int objectId, int objectTileX, int objectTileY,
                                              const std::optional<int>& parentNumber) {
            if (parentNumber.has_value()) {
                const auto tileObjects =
                    editorObjectNumbersByTile.find(tileKey(objectTileX, objectTileY));
                const bool resolved = tileObjects != editorObjectNumbersByTile.end() &&
                                      tileObjects->second.contains(*parentNumber);
                recordSkipped(
                    skipped,
                    resolved ? "[varparent] Object Nr. links resolved; scenery script-variable "
                               "inheritance is unsupported"
                             : "[varparent] references to missing same-tile Object Nr. records",
                    tileName + " id=" + std::to_string(objectId) +
                        " parentObjectNumber=" + std::to_string(*parentNumber));
            }
        };
        for (const openbus::map::MapSceneryPlacement& object : tile.sceneryObjects) {
            reportVariableParent(object.id, tileX, tileY, object.variableParentId);
        }
        for (const openbus::map::MapAttachedObject& object : tile.attachedObjects) {
            reportVariableParent(object.id, tileX, tileY, object.variableParentId);
        }
        for (const openbus::map::MapSplineAttachment& attachment : tile.splineAttachments) {
            reportVariableParent(attachment.id, tileX, tileY, attachment.variableParentId);
        }
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
            reportRules(object.id, object.rules);
        }
        for (const openbus::map::MapAttachedObject& object : tile.attachedObjects) {
            reportRules(object.id, object.rules);
        }
        for (const openbus::map::MapSplineAttachment& attachment : tile.splineAttachments) {
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
                    std::vector<ModelVertex> vertices;
                    std::vector<std::uint32_t> indices;
                    vertices.reserve(water.surfaces.size() * 4);
                    indices.reserve(water.surfaces.size() * 6);
                    double minimumHeight = std::numeric_limits<double>::infinity();
                    double maximumHeight = -std::numeric_limits<double>::infinity();
                    double heightSum = 0.0;
                    std::size_t heightCount = 0;
                    for (const openbus::map::WaterSurface& surface : water.surfaces) {
                        const auto corners =
                            openbus::map::buildMapWaterSurfaceVertices(tileX, tileY, surface);
                        const std::uint32_t base = static_cast<std::uint32_t>(vertices.size());
                        for (std::size_t corner = 0; corner < corners.size(); ++corner) {
                            const openbus::map::MapWaterVertex& vertex = corners[corner];
                            vertices.push_back({static_cast<float>(vertex.x),
                                                static_cast<float>(vertex.y),
                                                static_cast<float>(vertex.z),
                                                corner == 1 || corner == 2 ? 1.0F : 0.0F,
                                                corner >= 2 ? 1.0F : 0.0F, 0.0F, 0.0F, 0.0F, 1.0F});
                            minimumHeight = (std::min)(minimumHeight, vertex.z);
                            maximumHeight = (std::max)(maximumHeight, vertex.z);
                            heightSum += vertex.z;
                            ++heightCount;
                        }
                        // SW, SE, NE, NW gives upward-facing triangles in the map XY plane.
                        indices.insert(indices.end(),
                                       {base, base + 1, base + 2, base, base + 2, base + 3});
                    }
                    if (!vertices.empty()) {
                        Impl::WaterBatch batch;
                        batch.indexCount = indices.size();
                        batch.tileX = tileX;
                        batch.tileY = tileY;
                        batch.minZ = minimumHeight;
                        batch.maxZ = maximumHeight;
                        batch.meanZ = heightSum / static_cast<double>(heightCount);
                        pglGenBuffers(1, &batch.buffer);
                        pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
                        pglBufferData(
                            GL_ARRAY_BUFFER,
                            static_cast<std::ptrdiff_t>(vertices.size() * sizeof(ModelVertex)),
                            vertices.data(), GL_STATIC_DRAW);
                        pglGenBuffers(1, &batch.indexBuffer);
                        pglBindBuffer(GL_ARRAY_BUFFER, batch.indexBuffer);
                        pglBufferData(
                            GL_ARRAY_BUFFER,
                            static_cast<std::ptrdiff_t>(indices.size() * sizeof(std::uint32_t)),
                            indices.data(), GL_STATIC_DRAW);
                        if (batch.buffer == 0 || batch.indexBuffer == 0 ||
                            glGetError() != GL_NO_ERROR) {
                            if (batch.buffer != 0) {
                                pglDeleteBuffers(1, &batch.buffer);
                            }
                            if (batch.indexBuffer != 0) {
                                pglDeleteBuffers(1, &batch.indexBuffer);
                            }
                            recordSkipped(skipped, "water surfaces rejected by OpenGL upload",
                                          tileName);
                        } else {
                            impl_->waterBatches.push_back(batch);
                        }
                    }
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
        const auto addSceneryInstance = [&](const std::string& assetPath,
                                            const std::string& description, double worldX,
                                            double worldY, double localZ,
                                            double terrainHeightOffset,
                                            const std::array<double, 6>& rotationDegrees,
                                            std::size_t rotationCount,
                                            const std::array<double, 9>* orientationMatrix =
                                                nullptr,
                                            std::vector<std::string> placementLabels = {}) {
            const std::filesystem::path configPath = resolveSceneryConfigPath(assetPath);
            if (configPath.empty()) {
                recordSkipped(skipped, "scenery placements whose .sco config was not found",
                              description);
                return;
            }
            const std::vector<std::size_t>& batchIndexes = loadSceneryAsset(configPath);
            if (editorOnlySceneryAssets.contains(configPath.lexically_normal().generic_string())) {
                recordSkipped(skipped, "map scenery placements marked [onlyeditor]",
                              description + " config=" + configPath.filename().string());
                return;
            }
            const std::string assetKey = configPath.lexically_normal().generic_string();
            auto lodSet = sceneryLodThresholdSetByAsset.find(assetKey);
            if (lodSet == sceneryLodThresholdSetByAsset.end()) {
                const std::size_t index = impl_->sceneryLodThresholdSets.size();
                impl_->sceneryLodThresholdSets.emplace_back();
                lodSet = sceneryLodThresholdSetByAsset.emplace(assetKey, index).first;
            }
            const std::size_t lodThresholdSetIndex = lodSet->second;
            std::size_t scriptStateIndex = std::numeric_limits<std::size_t>::max();
            if (sceneryAssetsNeedingVariables.contains(assetKey)) {
                auto scriptState = std::make_unique<Impl::SceneryScriptState>();
                scriptState->ownerTileX = tileX;
                scriptState->ownerTileY = tileY;
                scriptState->placementLabels = std::move(placementLabels);
                for (const std::string& variable : sceneryModelFloatVariables[assetKey]) {
                    scriptState->variables.declare(variable);
                }
                for (const std::string& variable : sceneryModelStringVariables[assetKey]) {
                    scriptState->variables.declareString(variable);
                }
                const auto scriptConfiguration = sceneryScriptConfigurations.find(assetKey);
                auto definition = sceneryRuntimeDefinitions.find(assetKey);
                if (definition == sceneryRuntimeDefinitions.end()) {
                    auto sharedDefinition = std::make_shared<Impl::SceneryRuntimeDefinition>();
                    if (scriptConfiguration != sceneryScriptConfigurations.end()) {
                        sharedDefinition->configuration = scriptConfiguration->second;
                    }
                    sharedDefinition->scriptTextures = sceneryModelScriptTextures[assetKey];
                    sharedDefinition->textTextures = sceneryModelTextTextures[assetKey];
                    sharedDefinition->floatVariables = sceneryModelFloatVariables[assetKey];
                    sharedDefinition->stringVariables = sceneryModelStringVariables[assetKey];
                    sharedDefinition->hasRuntime =
                        (scriptConfiguration != sceneryScriptConfigurations.end() &&
                         !scriptConfiguration->second.scripts.empty()) ||
                        !sharedDefinition->scriptTextures.empty() ||
                        !sharedDefinition->textTextures.empty() ||
                        !sceneryFreeTextureVariables[assetKey].empty();
                    definition =
                        sceneryRuntimeDefinitions.emplace(assetKey, std::move(sharedDefinition))
                            .first;
                }
                scriptState->definition = definition->second;
                scriptStateIndex = impl_->sceneryScriptStates.size();
                impl_->sceneryScriptStates.push_back(std::move(scriptState));
            }
            const bool absoluteHeight = absoluteHeightSceneryAssets.contains(assetKey);
            const double worldZ =
                openbus::map::mapSceneryWorldHeight(localZ, terrainHeightOffset, absoluteHeight);
            if (batchIndexes.empty()) {
                recordSkipped(skipped, "scenery placements with no renderable asset batches",
                              description + " config=" + configPath.filename().string());
            } else {
                ++impl_->sceneryObjectCount;
                ++impl_->sceneryObjectsByTile[tileKey(tileX, tileY)];
            }
            for (const std::size_t batchIndex : batchIndexes) {
                if (batchIndex >= impl_->sceneryBatches.size()) {
                    continue;
                }
                const double radius = impl_->sceneryBatches[batchIndex].boundingRadius;
                Impl::SceneryInstance instance;
                instance.batchIndex = batchIndex;
                instance.x = worldX;
                instance.y = worldY;
                instance.z = worldZ;
                instance.boundingRadius = radius;
                instance.rotationDegrees = rotationDegrees;
                instance.rotationCount = rotationCount;
                instance.scriptStateIndex = scriptStateIndex;
                instance.lodThresholdSetIndex = lodThresholdSetIndex;
                instance.sourceTileX = tileX;
                instance.sourceTileY = tileY;
                if (orientationMatrix != nullptr) {
                    instance.orientationMatrix = *orientationMatrix;
                    instance.hasOrientationMatrix = true;
                }
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
            const std::optional<ResolvedSceneryPlacement> placement =
                resolveMapSceneryPlacement(object.id, resolveMapSceneryPlacement);
            if (!placement.has_value()) {
                recordSkipped(skipped, "[object] sections with invalid world placement",
                              objectDescription);
                continue;
            }
            const openbus::map::MapSceneryPose& pose = placement->pose;
            addSceneryInstance(placement->assetPath, objectDescription, pose.x, pose.y, pose.z, 0.0,
                               pose.rotationDegrees, pose.rotationCount,
                               pose.hasOrientationMatrix ? &pose.orientationMatrix : nullptr,
                               object.labels);
        }
        for (const openbus::map::MapAttachedObject& object : tile.attachedObjects) {
            const std::string description = tileName +
                                            " [attachObj] id=" + std::to_string(object.id) +
                                            " parent=" + std::to_string(object.attachedToObjectId) +
                                            " asset=" + object.assetPath;
            if (!object.transformValid) {
                recordSkipped(skipped, "[attachObj] records with invalid rotation or anchor index",
                              description);
                continue;
            }
            const std::optional<ResolvedSceneryPlacement> placement =
                resolveMapSceneryPlacement(object.id, resolveMapSceneryPlacement);
            if (!placement.has_value()) {
                recordSkipped(
                    skipped,
                    "[attachObj] records with missing parents, attachment points, or cycles",
                    description + " anchor=" + std::to_string(object.attachPointIndexValue));
                continue;
            }
            addSceneryInstance(placement->assetPath, description, placement->pose.x,
                               placement->pose.y, placement->pose.z, 0.0,
                               placement->pose.rotationDegrees, placement->pose.rotationCount,
                               &placement->pose.orientationMatrix, object.labels);
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
                openbus::map::placeMapSplineAttachment(attachment, tile.splines, tileX, tileY,
                                                       mapSplineResolver, splineCantWidth);
            if (poses.empty()) {
                recordSkipped(skipped, "spline attachment records with no local placement",
                              description);
                continue;
            }
            for (const openbus::map::MapSceneryPose& pose : poses) {
                openbus::map::MapSceneryPose renderedPose = pose;
                applyTerrainAlignment(renderedPose, attachment.splineTerrainAlign, description);
                addSceneryInstance(
                    attachment.assetPath, description, renderedPose.x, renderedPose.y,
                    renderedPose.z, 0.0, renderedPose.rotationDegrees, renderedPose.rotationCount,
                    renderedPose.hasOrientationMatrix ? &renderedPose.orientationMatrix : nullptr);
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
            const auto* loadedProfile = cachedSplineProfile(profilePath);
            if (loadedProfile == nullptr) {
                continue;
            }
            const openbus::map::MapSplineProfile& profile = *loadedProfile;
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
            const std::optional<double> cantHalfWidth =
                openbus::map::mapSplineProfileCantHalfWidth(profile);
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
                const int alphaMode =
                    static_cast<std::size_t>(section.textureIndex) <
                            profile.textureAlphaModes.size()
                        ? profile.textureAlphaModes[static_cast<std::size_t>(section.textureIndex)]
                        : 0;
                const bool textured = !roadTexture.empty();
                if (!textured) {
                    recordSkipped(
                        skipped, "spline profile textures not found (geometry rendered untextured)",
                        profilePath.filename().string() + " texture=" + textureName);
                }
                const std::string tileBatchPrefix =
                    std::to_string(tileX) + "," + std::to_string(tileY) + "|";
                const std::string batchKey =
                    tileBatchPrefix +
                    (textured ? roadTexture.lexically_normal().generic_string()
                              : "untextured:" + profileKey + "#" +
                                    std::to_string(section.textureIndex)) +
                    "|alpha:" + std::to_string(alphaMode);
                RoadMeshGeometry& road = roadVertices[batchKey];
                if (road.vertices.empty()) {
                    road.texturePath = roadTexture;
                    road.textured = textured;
                    road.tileX = tileX;
                    road.tileY = tileY;
                    road.alphaMode = alphaMode;
                }
                appendRoadProfile(road, spline, centerline, section,
                                  roadConnections.joinsFor(spline, tileX, tileY), cantHalfWidth);
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
        batch.tileX = geometry.tileX;
        batch.tileY = geometry.tileY;
        batch.alphaMode = geometry.alphaMode;
        batch.cullBackFaces = true;
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
        ++impl_->roadBatchesByTile[tileKey(batch.tileX, batch.tileY)];
    }
    gameLog.Log("Map road upload complete: geometry groups=" + std::to_string(roadVertices.size()) +
                ", uploaded batches=" + std::to_string(impl_->roadBatches.size()) + " in " +
                std::to_string(elapsedMilliseconds(roadUploadStart)) + " ms.");

    updateStreaming(spawnWorldX, spawnWorldY);
    gameLog.Log("Map render result: terrain tiles=" + std::to_string(impl_->terrainBuffers.size()) +
                ", scenery instances=" + std::to_string(impl_->sceneryInstanceCount) +
                ", scenery batches=" + std::to_string(impl_->sceneryBatches.size()) +
                ", scenery variable states=" + std::to_string(impl_->sceneryScriptStates.size()) +
                ", scenery script runtimes=" + std::to_string(impl_->sceneryScriptRuntimeCount) +
                ", road batches=" + std::to_string(impl_->roadBatches.size()) +
                ", water batches=" + std::to_string(impl_->waterBatches.size()) +
                ", cached textures=" + std::to_string(textureCache.size()) + ".");
    logSkipped(skipped);
    gameLog.Log("Map loading complete in " + std::to_string(elapsedMilliseconds(mapLoadStart)) +
                " ms.");
}

MapRenderer::~MapRenderer() = default;

void MapRenderer::updateScripts() {
    TraceScope trace("script", "MapRenderer::updateScripts");
    initializeActiveScripts();
    for (const std::size_t stateIndex : impl_->activeSceneryScriptStateIndices) {
        Impl::SceneryScriptState& scriptState = *impl_->sceneryScriptStates[stateIndex];
        if (!scriptState.runtime || !scriptState.runtimeInitialized) {
            continue;
        }
        const std::size_t previousErrorCount = scriptState.runtime->errors().size();
        scriptState.runtime->update(false);
        const std::vector<std::string>& errors = scriptState.runtime->errors();
        for (std::size_t index = previousErrorCount; index < errors.size(); ++index) {
            gameLog.Log("Scenery script frame error: " + errors[index]);
        }
    }
}

void MapRenderer::initializeActiveScripts() {
    for (const std::size_t stateIndex : impl_->activeSceneryScriptStateIndices) {
        Impl::SceneryScriptState& scriptState = *impl_->sceneryScriptStates[stateIndex];
        if (!scriptState.definition || !scriptState.definition->hasRuntime) {
            continue;
        }
        if (!scriptState.runtime) {
            scriptState.runtime =
                std::make_unique<ScriptRuntime>(scriptState.definition->configuration,
                                                scriptState.variables, *impl_->simulationState);
            scriptState.runtime->configureScriptTextures(scriptState.definition->scriptTextures);
            scriptState.runtime->configureTextTextures(scriptState.definition->textTextures,
                                                       scriptState.placementLabels);
            ++impl_->sceneryScriptRuntimeCount;
        }
        if (scriptState.runtimeInitialized) {
            continue;
        }
        const std::size_t previousErrorCount = scriptState.runtime->errors().size();
        scriptState.runtime->initialize();
        scriptState.runtimeInitialized = true;
        scriptState.runtime->update(false);
        const std::vector<std::string>& errors = scriptState.runtime->errors();
        for (std::size_t index = previousErrorCount; index < errors.size(); ++index) {
            gameLog.Log("Scenery script initialization error: " + errors[index]);
        }
    }
}

void MapRenderer::updateStreaming(double worldX, double worldY) {
    const int centerX = openbus::map::mapWorldTile(worldX);
    const int centerY = openbus::map::mapWorldTile(worldY);
    const openbus::map::MapTileCoordinate center{centerX, centerY};
    if (impl_->activeCenterTile == center) {
        return;
    }
    impl_->activeCenterTile = center;
    impl_->activeTileCoordinates = openbus::map::selectMapTilesWithinRadius(
        impl_->mapTileCoordinates, worldX, worldY, MAP_ACTIVE_TILE_RADIUS);
    impl_->activeTileKeys.clear();
    impl_->activeTileKeys.reserve(impl_->activeTileCoordinates.size());
    MapRenderStats stats;
    stats.activeTiles = impl_->activeTileCoordinates.size();
    stats.totalTiles = impl_->mapTileCoordinates.size();
    stats.totalTerrainTiles = impl_->terrainBuffers.size();
    stats.totalSceneryObjects = impl_->sceneryObjectCount;
    stats.totalRoadBatches = impl_->roadBatches.size();
    const auto countForTile = [](const auto& counts, std::uint64_t key) {
        const auto found = counts.find(key);
        return found == counts.end() ? std::size_t{0} : found->second;
    };
    for (const auto& [tileX, tileY] : impl_->activeTileCoordinates) {
        const std::uint64_t key = tileKey(tileX, tileY);
        impl_->activeTileKeys.insert(key);
        stats.activeTerrainTiles += countForTile(impl_->terrainTilesByTile, key);
        stats.activeSceneryObjects += countForTile(impl_->sceneryObjectsByTile, key);
        stats.activeRoadBatches += countForTile(impl_->roadBatchesByTile, key);
    }
    impl_->renderStats = stats;
    impl_->activeSceneryScriptStateIndices.clear();
    impl_->activeSceneryScriptStateIndices.reserve(impl_->sceneryScriptStates.size());
    for (std::size_t stateIndex = 0; stateIndex < impl_->sceneryScriptStates.size(); ++stateIndex) {
        const Impl::SceneryScriptState& scriptState = *impl_->sceneryScriptStates[stateIndex];
        if (impl_->activeTileKeys.contains(
                tileKey(scriptState.ownerTileX, scriptState.ownerTileY))) {
            impl_->activeSceneryScriptStateIndices.push_back(stateIndex);
        }
    }
    initializeActiveScripts();
    gameLog.Log("Map active render window: center=" + tileLabel(centerX, centerY) +
                ", active tiles=" + std::to_string(impl_->activeTileCoordinates.size()) +
                ", radius=" + std::to_string(MAP_ACTIVE_TILE_RADIUS) + " tiles.");
}

const std::vector<std::pair<int, int>>& MapRenderer::activeTileCoordinates() const {
    return impl_->activeTileCoordinates;
}

MapRenderStats MapRenderer::renderStats() const {
    return impl_->renderStats;
}

std::optional<double> MapRenderer::terrainHeightAt(double worldX, double worldY) const {
    if (!std::isfinite(worldX) || !std::isfinite(worldY)) {
        return std::nullopt;
    }
    const double tileXValue = std::floor(worldX / openbus::map::OMSI_TILE_SIZE_METERS);
    const double tileYValue = std::floor(worldY / openbus::map::OMSI_TILE_SIZE_METERS);
    if (tileXValue < static_cast<double>(std::numeric_limits<int>::min()) ||
        tileXValue > static_cast<double>(std::numeric_limits<int>::max()) ||
        tileYValue < static_cast<double>(std::numeric_limits<int>::min()) ||
        tileYValue > static_cast<double>(std::numeric_limits<int>::max())) {
        return std::nullopt;
    }
    const int tileX = static_cast<int>(tileXValue);
    const int tileY = static_cast<int>(tileYValue);
    const auto found = impl_->terrainByCoordinate.find(tileKey(tileX, tileY));
    if (found == impl_->terrainByCoordinate.end()) {
        return std::nullopt;
    }
    const double localX = worldX - tileXValue * openbus::map::OMSI_TILE_SIZE_METERS;
    const double localY = worldY - tileYValue * openbus::map::OMSI_TILE_SIZE_METERS;
    return openbus::map::sampleMapTerrainHeight(found->second, localX, localY);
}

void MapRenderer::draw(bool renderTypeFourOnly) const {
    TraceScope trace("map", "MapRenderer::draw");
    IndirectCullingBatch indirectBatch;
    IndirectCullingScope indirectScope;
    bool indirect = false;
    GLint previousFrontFace = GL_CCW;
    glGetIntegerv(GL_FRONT_FACE, &previousFrontFace);
    glFrontFace(GL_CW);
    struct FrontFaceRestore {
        GLint frontFace;
        ~FrontFaceRestore() {
            glFrontFace(static_cast<GLenum>(frontFace));
        }
    } frontFaceRestore{previousFrontFace};
    const Matrix4& view = modelViewMatrix();
    const MapFrustum frustum = buildMapFrustum(projectionMatrix());
    const double cameraX = -(view[0] * view[12] + view[1] * view[13] + view[2] * view[14]);
    const double cameraY = -(view[4] * view[12] + view[5] * view[13] + view[6] * view[14]);
    const double cameraZ = -(view[8] * view[12] + view[9] * view[13] + view[10] * view[14]);
    std::vector<CullingItem> cullingItems;
    std::vector<float> lodBoundaries;
    std::vector<std::uint32_t> visibility;
    std::unordered_map<const void*, std::size_t> cullingIndices;
    bool gpuCulling = false;
    if (computeCullingAvailable() || indirectCullingAvailable()) {
        TraceScope phase("map", "MapRenderer::draw.computeCulling");
        std::vector<std::uint32_t> lodOffsets;
        for (const auto& thresholds : impl_->sceneryLodThresholdSets) {
            lodOffsets.push_back(appendCullingLodBoundaries(lodBoundaries, thresholds));
        }
        const auto addSphere = [&](const void* key, const std::array<double, 3>& center,
                                   double radius, double minX, double minY, double maxX,
                                   double maxY, std::size_t count) -> CullingItem& {
            cullingIndices.emplace(key, cullingItems.size());
            CullingItem item = makeCullingItem(view, center, radius);
            item.draw[0] = count <= static_cast<std::size_t>(std::numeric_limits<GLsizei>::max())
                               ? static_cast<std::uint32_t>(count)
                               : std::numeric_limits<std::uint32_t>::max();
            item.metadata[0] |= CullMapDistance;
            item.distanceBounds = {
                static_cast<float>(minX - cameraX), static_cast<float>(minY - cameraY),
                static_cast<float>(maxX - cameraX), static_cast<float>(maxY - cameraY)};
            cullingItems.push_back(item);
            return cullingItems.back();
        };
        const auto addBounds = [&](const void* key, double minX, double minY, double minZ,
                                   double maxX, double maxY, double maxZ, std::size_t count) {
            addSphere(key, {(minX + maxX) * 0.5, (minY + maxY) * 0.5, (minZ + maxZ) * 0.5},
                      std::hypot(maxX - minX, maxY - minY, maxZ - minZ) * 0.5, minX, minY, maxX,
                      maxY, count);
        };
        const auto addTile = [&](const auto& tile) {
            if (!impl_->activeTileKeys.contains(tileKey(tile.tileX, tile.tileY)))
                return;
            const double x = tile.tileX * openbus::map::OMSI_TILE_SIZE_METERS;
            const double y = tile.tileY * openbus::map::OMSI_TILE_SIZE_METERS;
            addBounds(&tile, x, y, tile.minZ, x + openbus::map::OMSI_TILE_SIZE_METERS,
                      y + openbus::map::OMSI_TILE_SIZE_METERS, tile.maxZ, tile.indexCount);
        };
        if (!renderTypeFourOnly) {
            for (const auto& tile : impl_->terrainBuffers)
                addTile(tile);
            for (const auto& water : impl_->waterBatches)
                addTile(water);
            for (const auto& road : impl_->roadBatches) {
                if (road.hasVisibilityBounds &&
                    impl_->activeTileKeys.contains(tileKey(road.tileX, road.tileY))) {
                    addBounds(&road, road.minX, road.minY, road.minZ, road.maxX, road.maxY,
                              road.maxZ, road.indexCount);
                }
            }
        }
        for (const auto& [key, chunk] : impl_->sceneryChunks) {
            static_cast<void>(key);
            // Broad-phase chunk rejection avoids CPU material/resource work for
            // entire distant groups; individual sphere/LOD tests remain on GPU.
            if (indirectCullingAvailable() &&
                (squaredDistanceToBounds(cameraX, cameraY, chunk.minX, chunk.minY, chunk.maxX,
                                         chunk.maxY) > MAP_VISIBILITY_RADIUS_SQUARED ||
                 outsideMapFrustum(frustum, view,
                                   {(chunk.minX + chunk.maxX) * 0.5,
                                    (chunk.minY + chunk.maxY) * 0.5,
                                    (chunk.minZ + chunk.maxZ) * 0.5},
                                   std::hypot(chunk.maxX - chunk.minX, chunk.maxY - chunk.minY,
                                              chunk.maxZ - chunk.minZ) *
                                       0.5)))
                continue;
            for (const auto& instance : chunk.instances) {
                if (!impl_->activeTileKeys.contains(
                        tileKey(instance.sourceTileX, instance.sourceTileY)) ||
                    instance.batchIndex >= impl_->sceneryBatches.size())
                    continue;
                const auto& batch = impl_->sceneryBatches[instance.batchIndex];
                if (renderTypeFourOnly ? batch.renderType != 4 : batch.renderType == 4)
                    continue;
                auto& item = addSphere(
                    &instance, {instance.x, instance.y, instance.z}, instance.boundingRadius,
                    instance.x - instance.boundingRadius, instance.y - instance.boundingRadius,
                    instance.x + instance.boundingRadius, instance.y + instance.boundingRadius,
                    batch.vertexCount);
                item.metadata[0] |= CullLod;
                item.metadata[1] = static_cast<std::uint32_t>(batch.sceneryLodIndex);
                if (instance.lodThresholdSetIndex < lodOffsets.size()) {
                    item.metadata[2] = lodOffsets[instance.lodThresholdSetIndex];
                    item.metadata[3] = static_cast<std::uint32_t>(
                        impl_->sceneryLodThresholdSets[instance.lodThresholdSetIndex].size());
                }
                // Map assets without thresholds still select LOD zero.
                if (item.metadata[3] == 0 && batch.sceneryLodIndex != 0) {
                    item.metadata[0] |= CullReject;
                }
            }
        }
        if (indirectCullingAvailable()) {
            indirect = indirectBatch.dispatch(cullingItems, makeCullingPlanes(projectionMatrix()),
                                              lodBoundaries);
            gpuCulling = indirect;
        } else {
            gpuCulling = computeCull(cullingItems, makeCullingPlanes(projectionMatrix()),
                                     lodBoundaries, visibility);
        }
    }
    const auto gpuVisible = [&](const void* key) {
        const auto found = cullingIndices.find(key);
        return found != cullingIndices.end() && (indirect || visibility[found->second] != 0);
    };
    const auto selectIndirect = [&](const void* key) {
        if (!indirect)
            return;
        const auto found = cullingIndices.find(key);
        indirectBatch.select(found != cullingIndices.end()
                                 ? found->second
                                 : std::numeric_limits<std::size_t>::max());
    };
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
    const auto refreshSceneryTexture = [&](Impl::SceneryScriptState& state, bool textTexture,
                                           int index, const ModelMaterial& material) -> GLuint {
        if (!state.runtime || index < 0) {
            return 0;
        }
        Impl::DynamicSceneryTexture& texture = state.dynamicTextures[{textTexture, index}];
        ScriptRuntime::ScriptTextureSnapshot snapshot;
        const bool copied =
            texture.texture == 0
                ? (textTexture ? state.runtime->copyTextTexture(index, snapshot)
                               : state.runtime->copyScriptTexture(index, snapshot))
                : (textTexture
                       ? state.runtime->copyTextTextureIfChanged(index, texture.revision, snapshot)
                       : state.runtime->copyScriptTextureIfChanged(index, texture.revision,
                                                                   snapshot));
        if (!copied || snapshot.width <= 0 || snapshot.height <= 0) {
            return 0;
        }

        const bool dimensionsChanged =
            texture.width != snapshot.width || texture.height != snapshot.height;
        const bool needsUpload = texture.texture == 0 || dimensionsChanged ||
                                 texture.revision != snapshot.revision ||
                                 texture.filtered != snapshot.filtered;
        const bool wrapChanged =
            texture.wrapS != material.textureWrapS || texture.wrapT != material.textureWrapT;
        if (!needsUpload && !wrapChanged) {
            return texture.texture;
        }
        if (needsUpload) {
            const std::size_t expectedBytes = static_cast<std::size_t>(snapshot.width) *
                                              static_cast<std::size_t>(snapshot.height) * 4U;
            if (snapshot.pixels.size() != expectedBytes) {
                return 0;
            }
        }
        const GLenum textureUnit = GL_TEXTURE0 + (textTexture ? 5 : 4);
        pglActiveTexture(textureUnit);
        if (texture.texture == 0) {
            glGenTextures(1, &texture.texture);
            if (texture.texture == 0) {
                openbus::rendering::invalidateTextureBindings();
                return 0;
            }
            impl_->ownedTextures.push_back(texture.texture);
        }
        glBindTexture(GL_TEXTURE_2D, texture.texture);
        const GLint filter = snapshot.filtered ? GL_LINEAR : GL_NEAREST;
        if (needsUpload) {
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
        }
        if (wrapChanged) {
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, material.textureWrapS);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, material.textureWrapT);
        }
        if (needsUpload) {
            if (dimensionsChanged || texture.width == 0 || texture.height == 0) {
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, snapshot.width, snapshot.height, 0, GL_RGBA,
                             GL_UNSIGNED_BYTE, snapshot.pixels.data());
            } else {
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, snapshot.width, snapshot.height, GL_RGBA,
                                GL_UNSIGNED_BYTE, snapshot.pixels.data());
            }
            if (glGetError() != GL_NO_ERROR) {
                openbus::rendering::invalidateTextureBindings();
                return 0;
            }
            texture.width = snapshot.width;
            texture.height = snapshot.height;
            texture.revision = snapshot.revision;
            texture.filtered = snapshot.filtered;
        }
        if (wrapChanged) {
            texture.wrapS = material.textureWrapS;
            texture.wrapT = material.textureWrapT;
        }
        openbus::rendering::invalidateTextureBindings();
        return texture.texture;
    };

    const auto drawTerrain = [&]() {
        ModelMaterial material;
        material.texture = impl_->texture;
        material.textured = true;
        material.detailTexture = impl_->baseDetailTexture;
        material.useDetailTexture = impl_->baseDetailTexture != 0;
        for (const Impl::TileBuffer& tile : impl_->terrainBuffers) {
            if (!impl_->activeTileKeys.contains(tileKey(tile.tileX, tile.tileY))) {
                continue;
            }
            const double minX =
                static_cast<double>(tile.tileX) * openbus::map::OMSI_TILE_SIZE_METERS;
            const double minY =
                static_cast<double>(tile.tileY) * openbus::map::OMSI_TILE_SIZE_METERS;
            if (gpuCulling && !gpuVisible(&tile))
                continue;
            if (!gpuCulling &&
                outsideVisibilityBounds(minX, minY, minX + openbus::map::OMSI_TILE_SIZE_METERS,
                                        minY + openbus::map::OMSI_TILE_SIZE_METERS)) {
                continue;
            }
            if (!gpuCulling &&
                outsideFrustumBounds(minX, minY, tile.minZ,
                                     minX + openbus::map::OMSI_TILE_SIZE_METERS,
                                     minY + openbus::map::OMSI_TILE_SIZE_METERS, tile.maxZ)) {
                continue;
            }
            material.textureScale = tile.baseTextureScale;
            material.detailTextureScale = tile.baseDetailTextureScale;
            selectIndirect(&tile);
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
    };
    const auto drawWater = [&]() {
        std::vector<std::pair<const Impl::WaterBatch*, double>> visibleWater;
        for (const Impl::WaterBatch& water : impl_->waterBatches) {
            if (!impl_->activeTileKeys.contains(tileKey(water.tileX, water.tileY))) {
                continue;
            }
            const double minX =
                static_cast<double>(water.tileX) * openbus::map::OMSI_TILE_SIZE_METERS;
            const double minY =
                static_cast<double>(water.tileY) * openbus::map::OMSI_TILE_SIZE_METERS;
            const double maxX = minX + openbus::map::OMSI_TILE_SIZE_METERS;
            const double maxY = minY + openbus::map::OMSI_TILE_SIZE_METERS;
            if (gpuCulling
                    ? !gpuVisible(&water)
                    : (outsideVisibilityBounds(minX, minY, maxX, maxY) ||
                       outsideFrustumBounds(minX, minY, water.minZ, maxX, maxY, water.maxZ))) {
                continue;
            }
            const double centerX = (minX + maxX) * 0.5;
            const double centerY = (minY + maxY) * 0.5;
            const double viewDepth =
                -(view[2] * centerX + view[6] * centerY + view[10] * water.meanZ + view[14]);
            visibleWater.emplace_back(&water, viewDepth);
        }
        if (visibleWater.empty()) {
            return;
        }
        std::stable_sort(
            visibleWater.begin(), visibleWater.end(),
            [](const auto& first, const auto& second) { return first.second > second.second; });
        ModelMaterial material;
        const std::array<double, 3> waterColor = {0.24, 0.48, 0.60};
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        for (const auto& [water, depth] : visibleWater) {
            static_cast<void>(depth);
            selectIndirect(water);
            drawIndexedModelBatch(water->buffer, water->indexBuffer, water->indexCount, material,
                                  waterColor, 0.62, 0);
        }
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);
        glDisable(GL_BLEND);
    };
    const auto drawSplines = [&]() {
        for (const Impl::ModelBatch& road : impl_->roadBatches) {
            if (!impl_->activeTileKeys.contains(tileKey(road.tileX, road.tileY))) {
                continue;
            }
            if (gpuCulling && road.hasVisibilityBounds && !gpuVisible(&road))
                continue;
            if (!gpuCulling && road.hasVisibilityBounds &&
                outsideVisibilityBounds(road.minX, road.minY, road.maxX, road.maxY)) {
                continue;
            }
            if (!gpuCulling && road.hasVisibilityBounds &&
                outsideFrustumBounds(road.minX, road.minY, road.minZ, road.maxX, road.maxY,
                                     road.maxZ)) {
                continue;
            }
            selectIndirect(&road);
            drawIndexedModelBatch(road.buffer, road.indexBuffer, road.indexCount, road.material,
                                  road.color, 1.0, road.alphaMode, road.cullBackFaces);
        }
    };
    struct SceneryDraw {
        const Impl::SceneryInstance* instance = nullptr;
        const Impl::ModelBatch* batch = nullptr;
        ModelMaterial material;
        int alphaMode = 0;
        double viewDepth = 0.0;
    };
    std::vector<SceneryDraw> sceneryDraws;
    sceneryDraws.reserve(impl_->renderStats.activeSceneryObjects);
    const auto drawSceneryInstance = [&](const Impl::SceneryInstance& instance,
                                         const Impl::ModelBatch& batch,
                                         const ModelMaterial& sceneryMaterial, int alphaMode) {
        selectIndirect(&instance);
        pushMatrix();
        translate(instance.x, instance.y, instance.z);
        if (instance.hasOrientationMatrix) {
            const std::array<double, 16> transform = {instance.orientationMatrix[0],
                                                      instance.orientationMatrix[1],
                                                      instance.orientationMatrix[2],
                                                      0.0,
                                                      instance.orientationMatrix[3],
                                                      instance.orientationMatrix[4],
                                                      instance.orientationMatrix[5],
                                                      0.0,
                                                      instance.orientationMatrix[6],
                                                      instance.orientationMatrix[7],
                                                      instance.orientationMatrix[8],
                                                      0.0,
                                                      0.0,
                                                      0.0,
                                                      0.0,
                                                      1.0};
            multiplyMatrix(transform);
        } else {
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
        }
        drawModelBatch(batch.buffer, batch.vertexCount, sceneryMaterial, batch.color, batch.alpha,
                       alphaMode, batch.cullBackFaces);
        popMatrix();
    };
    for (const auto& [key, chunk] : impl_->sceneryChunks) {
        static_cast<void>(key);
        if (chunk.instances.empty() ||
            (!gpuCulling &&
             (outsideVisibilityBounds(chunk.minX, chunk.minY, chunk.maxX, chunk.maxY) ||
              outsideFrustumBounds(chunk.minX, chunk.minY, chunk.minZ, chunk.maxX, chunk.maxY,
                                   chunk.maxZ)))) {
            continue;
        }
        for (const Impl::SceneryInstance& instance : chunk.instances) {
            if (!impl_->activeTileKeys.contains(
                    tileKey(instance.sourceTileX, instance.sourceTileY)) ||
                instance.batchIndex >= impl_->sceneryBatches.size() ||
                (gpuCulling
                     ? !gpuVisible(&instance)
                     : (outsideVisibilityBounds(instance.x - instance.boundingRadius,
                                                instance.y - instance.boundingRadius,
                                                instance.x + instance.boundingRadius,
                                                instance.y + instance.boundingRadius) ||
                        outsideMapFrustum(frustum, view, {instance.x, instance.y, instance.z},
                                          instance.boundingRadius)))) {
                continue;
            }
            const Impl::ModelBatch& batch = impl_->sceneryBatches[instance.batchIndex];
            if (renderTypeFourOnly ? batch.renderType != 4 : batch.renderType == 4) {
                continue;
            }
            const std::vector<double>* lodThresholds =
                instance.lodThresholdSetIndex < impl_->sceneryLodThresholdSets.size()
                    ? &impl_->sceneryLodThresholdSets[instance.lodThresholdSetIndex]
                    : nullptr;
            static const std::vector<double> noLodThresholds;
            const std::size_t selectedLod =
                gpuCulling ? batch.sceneryLodIndex
                           : openbus::map::selectMapSceneryLod(
                                 lodThresholds != nullptr ? *lodThresholds : noLodThresholds,
                                 std::hypot(instance.x - cameraX, instance.y - cameraY,
                                            instance.z - cameraZ));
            if (!gpuCulling && batch.sceneryLodIndex != selectedLod) {
                continue;
            }
            if (!batch.visibilityVariable.empty()) {
                const bool hasScriptState =
                    instance.scriptStateIndex < impl_->sceneryScriptStates.size();
                const float value =
                    hasScriptState
                        ? impl_->sceneryScriptStates[instance.scriptStateIndex]->variables.get(
                              batch.visibilityVariable)
                        : 0.0F;
                if (value != static_cast<float>(batch.visibilityValue)) {
                    continue;
                }
            }
            ModelMaterial sceneryMaterial = batch.material;
            int alphaMode = batch.alphaMode;
            if (instance.scriptStateIndex < impl_->sceneryScriptStates.size()) {
                Impl::SceneryScriptState& scriptState =
                    *impl_->sceneryScriptStates[instance.scriptStateIndex];
                if (batch.textTextureIndex >= 0) {
                    sceneryMaterial.textTexture = refreshSceneryTexture(
                        scriptState, true, batch.textTextureIndex, sceneryMaterial);
                    sceneryMaterial.useTextTexture = sceneryMaterial.textTexture != 0;
                    if (sceneryMaterial.useTextTexture) {
                        // Text glyphs include partial alpha for antialiasing and a
                        // transparent background, so draw them as a blended overlay.
                        alphaMode = 2;
                    }
                } else if (batch.scriptTextureIndex >= 0) {
                    sceneryMaterial.freeTexture = refreshSceneryTexture(
                        scriptState, false, batch.scriptTextureIndex, sceneryMaterial);
                    sceneryMaterial.useFreeTexture = sceneryMaterial.freeTexture != 0;
                } else if (!batch.freeTextureVariable.empty()) {
                    const std::string selectedTexture =
                        scriptState.variables.getString(batch.freeTextureVariable);
                    if (!selectedTexture.empty()) {
                        const std::filesystem::path texturePath =
                            findSceneryTexture(batch.textureObjectRoot, batch.textureModelRoot,
                                               impl_->omsiRoot, selectedTexture);
                        const std::string textureKey =
                            texturePath.empty()
                                ? lowercasePathComponent(normalizedAssetPath(selectedTexture)
                                                             .lexically_normal()
                                                             .generic_string())
                                : texturePath.lexically_normal().generic_string();
                        if (!impl_->unavailableSceneryTextures.contains(textureKey)) {
                            if (texturePath.empty()) {
                                impl_->unavailableSceneryTextures.insert(textureKey);
                                gameLog.Log("Map scenery free texture not found: " +
                                            selectedTexture);
                            } else {
                                auto texture = impl_->dynamicSceneryTextures.find(textureKey);
                                if (texture == impl_->dynamicSceneryTextures.end()) {
                                    try {
                                        const GLuint loadedTexture = loadTexture(texturePath);
                                        impl_->ownedTextures.push_back(loadedTexture);
                                        texture = impl_->dynamicSceneryTextures
                                                      .emplace(textureKey, loadedTexture)
                                                      .first;
                                    } catch (const std::exception& error) {
                                        impl_->unavailableSceneryTextures.insert(textureKey);
                                        gameLog.Log("Could not load map scenery free texture " +
                                                    texturePath.generic_string() + ": " +
                                                    error.what());
                                    }
                                }
                                if (texture != impl_->dynamicSceneryTextures.end()) {
                                    sceneryMaterial.freeTexture = texture->second;
                                    sceneryMaterial.useFreeTexture = texture->second != 0;
                                }
                            }
                        }
                    }
                }
            }
            const double viewDepth =
                -(view[2] * instance.x + view[6] * instance.y + view[10] * instance.z + view[14]);
            sceneryDraws.push_back(
                {&instance, &batch, std::move(sceneryMaterial), alphaMode, viewDepth});
        }
    }

    const auto drawSceneryTypes = [&](auto matchesRenderType) {
        std::vector<const SceneryDraw*> opaqueScenery;
        std::vector<const SceneryDraw*> transparentScenery;
        for (const SceneryDraw& scenery : sceneryDraws) {
            if (!matchesRenderType(scenery.batch->renderType)) {
                continue;
            }
            if (scenery.alphaMode == 2 || scenery.alphaMode == 3) {
                transparentScenery.push_back(&scenery);
            } else {
                opaqueScenery.push_back(&scenery);
            }
        }
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        for (const SceneryDraw* scenery : opaqueScenery) {
            glDepthMask(scenery->batch->noZwrite ? GL_FALSE : GL_TRUE);
            // OMSI shadow overlays commonly request both flags: they must not
            // write depth, but still need depth testing so they cannot bleed
            // through foreground scenery or vehicles.
            const bool skipDepthCheck = scenery->batch->noZcheck && !scenery->batch->noZwrite;
            if (skipDepthCheck) {
                glDisable(GL_DEPTH_TEST);
            }
            drawSceneryInstance(*scenery->instance, *scenery->batch, scenery->material,
                                scenery->alphaMode);
            if (skipDepthCheck) {
                glEnable(GL_DEPTH_TEST);
            }
        }
        std::stable_sort(transparentScenery.begin(), transparentScenery.end(),
                         [](const SceneryDraw* first, const SceneryDraw* second) {
                             return first->viewDepth > second->viewDepth;
                         });
        if (transparentScenery.empty()) {
            glDepthMask(GL_TRUE);
            glDepthFunc(GL_LESS);
            glEnable(GL_DEPTH_TEST);
            return;
        }

        const bool hasTransmapDepthPrepass = std::any_of(
            transparentScenery.begin(), transparentScenery.end(), [](const SceneryDraw* scenery) {
                return scenery->alphaMode == 2 && scenery->material.useTransmap &&
                       !scenery->material.useTextTexture && !scenery->batch->noZwrite &&
                       !scenery->batch->noZcheck;
            });
        if (hasTransmapDepthPrepass) {
            GLboolean previousColorMask[4] = {GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE};
            glGetBooleanv(GL_COLOR_WRITEMASK, previousColorMask);
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
            for (const SceneryDraw* scenery : transparentScenery) {
                if (scenery->alphaMode != 2 || !scenery->material.useTransmap ||
                    scenery->material.useTextTexture || scenery->batch->noZwrite ||
                    scenery->batch->noZcheck) {
                    continue;
                }
                // The shader's alpha-mode-3 threshold keeps only fully opaque
                // transmap texels. Record those surfaces in depth before the
                // blended pass, matching the vehicle renderer's treatment.
                drawSceneryInstance(*scenery->instance, *scenery->batch, scenery->material, 3);
            }
            glColorMask(previousColorMask[0], previousColorMask[1], previousColorMask[2],
                        previousColorMask[3]);
        }
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        for (const SceneryDraw* scenery : transparentScenery) {
            // Text textures are transparent overlays: write depth only where the
            // shader retains visible glyph pixels, leaving the cleared background
            // transparent for the rest of the scene.
            const bool hasOpaqueTransmapDepth =
                scenery->alphaMode == 2 && scenery->material.useTransmap &&
                !scenery->material.useTextTexture && !scenery->batch->noZwrite &&
                !scenery->batch->noZcheck;
            glDepthFunc(hasOpaqueTransmapDepth ? GL_LEQUAL : GL_LESS);
            glDepthMask(scenery->material.useTextTexture && !scenery->batch->noZwrite ? GL_TRUE
                                                                                      : GL_FALSE);
            const bool skipDepthCheck = scenery->batch->noZcheck && !scenery->batch->noZwrite;
            if (skipDepthCheck) {
                glDisable(GL_DEPTH_TEST);
            }
            drawSceneryInstance(*scenery->instance, *scenery->batch, scenery->material,
                                scenery->alphaMode);
            if (skipDepthCheck) {
                glEnable(GL_DEPTH_TEST);
            }
        }
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);
        glDisable(GL_BLEND);
    };

    if (renderTypeFourOnly) {
        drawSceneryTypes([](int renderType) { return renderType == 4; });
    } else {
        drawSceneryTypes([](int renderType) { return renderType == -1; });
        drawTerrain();
        drawWater();
        drawSceneryTypes([](int renderType) { return renderType == -2; });
        drawSplines();
        drawSceneryTypes([](int renderType) { return renderType == -3; });
        drawSceneryTypes([](int renderType) { return renderType == 1; });
        drawSceneryTypes([](int renderType) {
            return renderType != -1 && renderType != -2 && renderType != -3 && renderType != 1 &&
                   renderType != 3 && renderType != 4;
        });
        drawSceneryTypes([](int renderType) { return renderType == 3; });
    }
}

} // namespace openbus::rendering