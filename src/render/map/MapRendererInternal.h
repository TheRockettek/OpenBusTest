#pragma once

// Internal to the map renderer: shared helper types and the MapRenderer::Impl
// state. Included only by render/map/*.cpp.

#include "assets/O3DLoader.h"
#include "assets/ObjLoader.h"
#include "assets/TextureLoader.h"
#include "assets/XLoader.h"
#include "config/ModelConfigLoader.h"
#include "config/SceneryObjectConfigLoader.h"
#include "core/AssetPath.h"
#include "core/Environment.h"
#include "core/Logger.h"
#include "core/PerfTrace.h"
#include "core/Variables.h"
#include "map/MapRoadGeometry.h"
#include "map/MapSceneryPlacement.h"
#include "map/MapSplineGeometry.h"
#include "map/MapSplineProfile.h"
#include "map/MapTrafficRuntime.h"
#include "map/MapTileWindow.h"
#include "map/MapWaterGeometry.h"
#include "render/ComputeCulling.h"
#include "render/CoreRenderer.h"
#include "render/GlCompat.h"
#include "render/MatrixMath.h"
#include "render/OpenGLFunctions.h"
#include "render/RenderPrimitives.h"
#include "render/SceneryMaterialBinding.h"
#include "render/map/MapRenderer.h"
#include "scripting/ScriptRuntime.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <future>
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

namespace openbus::rendering {

namespace map_detail {

inline constexpr std::size_t MAX_SKIP_EXAMPLES = 3;
inline constexpr double MAP_VISIBILITY_RADIUS_METERS = 10000.0;
inline constexpr int MAP_ACTIVE_TILE_RADIUS = 2;
inline constexpr double MAP_VISIBILITY_RADIUS_SQUARED =
    MAP_VISIBILITY_RADIUS_METERS * MAP_VISIBILITY_RADIUS_METERS;

double squaredDistanceToBounds(double x, double y, double minX, double minY, double maxX,
                               double maxY);

struct SkipSummary {
    std::size_t count = 0;
    std::vector<std::string> examples;
};

using SkipSummaries = std::map<std::string, SkipSummary>;

long long elapsedMilliseconds(std::chrono::steady_clock::time_point start);

std::string tileLabel(int x, int y);

void recordSkipped(SkipSummaries& summaries, const std::string& reason, const std::string& example,
                   std::size_t count = 1);

void logSkipped(const SkipSummaries& summaries);

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
    float r = 1.0F;
    float g = 1.0F;
    float b = 1.0F;
    float a = 1.0F;
};

static_assert(sizeof(ModelVertex) == 13 * sizeof(float));

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
    float r = 1.0F;
    float g = 1.0F;
    float b = 1.0F;
    float a = 1.0F;
};

static_assert(sizeof(TerrainVertex) == 13 * sizeof(float));

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
                                        const std::string& configuredPath);

GLuint loadTexture(const std::filesystem::path& path, bool repeat = true);
GLuint uploadTextureImage(const Image& image, bool repeat = true);

std::filesystem::path normalizedAssetPath(const std::string& value);

std::string lowercasePathComponent(std::string value);

std::filesystem::path findCaseInsensitiveFile(const std::filesystem::path& path);

std::filesystem::path findSceneryTexture(const std::filesystem::path& objectRoot,
                                         const std::filesystem::path& modelRoot,
                                         const std::filesystem::path& omsiRoot,
                                         const std::string& textureName);

std::uint64_t tileKey(int x, int y);

double vertexBoundingRadius(const std::vector<ModelVertex>& vertices);

// Tests a world-space bounding sphere against a frustum built with buildViewFrustum().
[[nodiscard]] bool outsideMapFrustum(const ViewFrustum& frustum, const Matrix4& modelView,
                                     const std::array<double, 3>& center, double radius);

std::vector<ModelVertex> makeTreePreview(double height, double width);

std::filesystem::path resolveSplineProfile(const openbus::map::MapDefinition& map,
                                           const std::filesystem::path& omsiRoot,
                                           const std::string& assetPath);

std::filesystem::path findSplineTexture(const std::filesystem::path& profilePath,
                                        const std::filesystem::path& omsiRoot,
                                        const std::string& textureName);

void appendRoadProfile(RoadMeshGeometry& road, const openbus::map::MapSplinePlacement& spline,
                       const std::vector<openbus::map::MapSplineSample>& centerline,
                       const openbus::map::MapSplineProfileSection& section,
                       const openbus::map::MapRoadSplineJoins& joins,
                       std::optional<double> cantHalfWidth);

Matrix4 makeSceneryInstanceTransform(double x, double y, double z,
                                     const std::array<double, 6>& rotationDegrees,
                                     std::size_t rotationCount,
                                     const std::array<double, 9>* orientationMatrix);

} // namespace map_detail

using namespace map_detail;

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

    struct HumanModelAsset {
        std::vector<std::size_t> batchIndexes;
        std::vector<std::filesystem::path> texturePaths;
        std::vector<std::shared_future<std::shared_ptr<Image>>> textureImages;
        double heightMeters = 0.0;
    };

    struct HumanModelConfiguration {
        std::filesystem::path modelConfigPath;
        ModelConfig configuration;
        double heightMeters = 0.0;
    };

    struct PendingHumanModel {
        std::filesystem::path humanConfigPath;
        std::future<HumanModelConfiguration> configurationRequest;
        std::optional<HumanModelConfiguration> configuration;
        std::vector<std::shared_future<std::shared_ptr<ParsedObj>>> meshRequests;
    };

    struct SceneryInstance {
        std::size_t batchIndex = 0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double boundingRadius = 0.0;
        // Static world transform (translation and rotation), precomputed at load.
        Matrix4 transform = {};
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
        std::optional<openbus::map::MapTrafficSignalSchedule> trafficSignal;
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

    // Instances of one chunk that belong to active tiles, split by whether the
    // batch is render type 4 so each draw pass only visits its own candidates.
    struct ActiveSceneryChunk {
        const SceneryChunk* chunk = nullptr;
        std::vector<const SceneryInstance*> instances;
    };

    struct SceneryDraw {
        const SceneryInstance* instance = nullptr;
        const ModelBatch* batch = nullptr;
        ModelMaterial material;
        int alphaMode = 0;
        double viewDepth = 0.0;
    };

    static constexpr std::size_t SCENERY_DRAW_GROUP_COUNT = 7;

    GLuint texture = 0;
    GLuint baseDetailTexture = 0;
    std::vector<TileBuffer> terrainBuffers;
    std::unordered_map<std::uint64_t, openbus::map::TerrainGrid> terrainByCoordinate;
    std::vector<WaterBatch> waterBatches;
    std::vector<ModelBatch> sceneryBatches;
    std::unordered_map<std::string, HumanModelAsset> humanTrafficModels;
    std::unordered_map<std::string, PendingHumanModel> pendingHumanTrafficModels;
    std::unordered_set<std::string> failedHumanTrafficModels;
    std::unordered_map<std::string, std::shared_future<std::shared_ptr<Image>>>
        humanTextureRequests;
    std::vector<std::vector<double>> sceneryLodThresholdSets;
    std::vector<std::unique_ptr<SceneryScriptState>> sceneryScriptStates;
    std::vector<std::size_t> activeSceneryScriptStateIndices;
    std::size_t sceneryScriptRuntimeCount = 0;
    std::unordered_map<std::uint64_t, SceneryChunk> sceneryChunks;
    std::size_t sceneryInstanceCount = 0;
    std::size_t sceneryObjectCount = 0;
    std::vector<ModelBatch> roadBatches;
    openbus::map::MapTrafficRouteNetwork trafficRouteNetwork;
    openbus::map::MapTrafficRouteNetwork roadTrafficRouteNetwork;
    openbus::map::MapTrafficLaneNetwork roadTrafficLaneNetwork;
    openbus::map::MapTrafficRouteNetwork humanTrafficRouteNetwork;
    openbus::map::MapTrafficLaneNetwork humanTrafficLaneNetwork;
    std::vector<TrafficPathDebugSegment> sceneryTrafficPathSegments;
    std::vector<std::shared_ptr<openbus::map::MapTrafficSignalController>> trafficSignalControllers;
    std::unordered_map<int, std::vector<openbus::map::MapSplinePathOffset>>
        roadTrafficSplinePathOffsets;
    std::unordered_map<int, std::vector<openbus::map::MapSplinePathOffset>>
        humanTrafficSplinePathOffsets;
    std::vector<openbus::map::MapTrafficActor> trafficActors;
    std::unordered_map<std::uint64_t, std::size_t> terrainTilesByTile;
    std::unordered_map<std::uint64_t, std::size_t> sceneryObjectsByTile;
    std::unordered_map<std::uint64_t, std::size_t> roadBatchesByTile;
    MapRenderStats renderStats;
    std::vector<GLuint> ownedTextures;
    std::filesystem::path omsiRoot;
    std::unordered_map<std::string, GLuint> dynamicSceneryTextures;
    std::unordered_set<std::string> unavailableSceneryTextures;
    // Per scenery batch: selected free-texture name -> texture to apply (nullopt
    // leaves the batch material unchanged), so steady-state frames never touch
    // the filesystem.
    std::vector<std::unordered_map<std::string, std::optional<GLuint>>> resolvedFreeTexturesByBatch;
    std::array<std::vector<ActiveSceneryChunk>, 2> activeSceneryChunks;
    std::vector<SceneryDraw> sceneryDrawScratch;
    std::array<std::vector<const SceneryDraw*>, SCENERY_DRAW_GROUP_COUNT> opaqueSceneryScratch;
    std::array<std::vector<const SceneryDraw*>, SCENERY_DRAW_GROUP_COUNT> transparentSceneryScratch;
    std::vector<openbus::map::MapTileCoordinate> mapTileCoordinates;
    std::vector<openbus::map::MapTileCoordinate> activeTileCoordinates;
    std::unordered_set<std::uint64_t> activeTileKeys;
    std::optional<openbus::map::MapTileCoordinate> activeCenterTile;
    bool leftHandTraffic = false;
    SimulationState* simulationState = nullptr;

    ~Impl();
};

} // namespace openbus::rendering
