#pragma once

// Builds static map content into a MapRenderer::Impl, loading render assets as
// their tiles enter the active window.

#include "render/map/MapRendererInternal.h"
#include "render/map/PreparedMapTextures.h"
#include "render/map/PreparedTerrainTexturePaths.h"
#include "render/map/SharedMapTextureImages.h"
#include "render/AssetRequestManager.h"
#include "config/SceneryObjectConfigLoader.h"

#include <exception>
#include <future>
#include <tuple>

namespace openbus::rendering {

class MapSceneLoader {
  public:
    enum class TileLoadStatus {
        Pending,
        Ready,
        Updated,
    };

    MapSceneLoader(const openbus::map::MapDefinition& map, std::size_t centerTileIndex,
                   std::size_t groundTextureIndex, const std::filesystem::path& omsiRoot,
                   MapRenderer::Impl& impl, AssetRequestManager& assetRequests);
    ~MapSceneLoader();

    // Loads map-wide indexes and shared textures.
    void load();
    bool finishLoad(bool waitForCompletion);
    bool roadTrafficLaneNetworkReady() const noexcept {
        return loadFinished_ && !roadTrafficLaneNetworkDirty &&
               !pendingRoadTrafficLaneNetworkFinalization_.valid();
    }
    bool humanTrafficLaneNetworkReady() const noexcept {
        return loadFinished_ && !humanTrafficLaneNetworkDirty &&
               !pendingHumanTrafficLaneNetworkFinalization_.valid();
    }
    // Loads render assets for active tiles that have not been loaded yet.
    TileLoadStatus loadActiveTiles(const std::vector<std::pair<int, int>>& tileCoordinates,
                                   const openbus::map::MapTileCoordinate& priorityCenter,
                                   bool waitForCompletion = false,
                                   bool failOnExhaustedRetries = false);
    // Logs totals and skip summaries; call after the initial streaming update.
    void logSummary() const;

  private:
    using Impl = MapRenderer::Impl;

    struct PreparedTerrainTile {
        openbus::map::TerrainGrid grid;
        std::vector<TerrainVertex> vertices;
        std::vector<std::uint32_t> indices;
        std::unordered_map<std::string, std::shared_ptr<Image>> decodedTextures;
        std::unordered_map<std::string, std::string> textureErrors;
        PreparedTerrainTexturePaths texturePaths;
        float minimumHeight = 0.0F;
        float maximumHeight = 0.0F;
    };

    struct PreparedSceneryMesh {
        std::shared_ptr<ParsedObj> parsed;
        std::unordered_map<int, std::vector<ModelVertex>> verticesByMaterial;
        std::size_t invalidPositionIndices = 0;
        std::exception_ptr preparationError;
    };

    struct PreparedSceneryAuxTexture {
        std::string name;
        std::filesystem::path path;
        std::string preparationError;
    };

    struct PreparedSceneryBatch {
        std::string meshKey;
        int materialIndex = 0;
        Impl::ModelBatch batch;
        std::string textureName;
        std::string transmapTextureName;
        std::string freeTextureName;
        std::filesystem::path texturePath;
        std::filesystem::path transmapTexturePath;
        PreparedSceneryAuxTexture lightmap;
        PreparedSceneryAuxTexture nightmap;
        std::vector<PreparedSceneryAuxTexture> itemLightmaps;
        std::vector<PreparedSceneryAuxTexture> itemNightmaps;
        PreparedSceneryAuxTexture bumpmap;
        PreparedSceneryAuxTexture environment;
        std::filesystem::path freeTexturePath;
        bool hasMaterialState = false;
        int stateAlphaMode = 0;
    };

    struct PreparedSceneryTreeBatch {
        std::vector<ModelVertex> vertices;
        std::string textureName;
        std::filesystem::path texturePath;
        Impl::ModelBatch batch;
    };

    struct PreparedSceneryAsset {
        SceneryObjectConfig objectConfiguration;
        ModelConfig modelConfiguration;
        openbus::scripting::SceneryObject variables;
        std::unordered_map<std::string, PreparedSceneryMesh> meshes;
        std::vector<PreparedSceneryBatch> batches;
        std::vector<PreparedSceneryTreeBatch> treeBatches;
        std::unordered_map<std::string, std::filesystem::path> resolvedTexturePaths;
        std::unordered_map<std::string, std::string> auxiliaryTexturePreparationErrors;
        std::unordered_map<std::string, std::shared_ptr<Image>> decodedTextures;
        std::size_t textureLookups = 0;
        std::size_t textureResolutions = 0;
        std::filesystem::path modelConfigPath;
        std::filesystem::path modelRoot;
        std::filesystem::path externalModelConfigPath;
        bool usesExternalModelConfig = false;
        bool externalModelConfigMissing = false;
        bool hasModelConfiguration = false;
        std::exception_ptr preparationError;
    };

    struct PendingSceneryAsset {
        std::string key;
        std::shared_future<std::shared_ptr<PreparedSceneryAsset>> result;
    };

    struct TilePreparationFailure {
        std::size_t attempts = 0;
        std::chrono::steady_clock::time_point retryAfter;
        bool exhausted = false;
        bool terrainPreparation = false;
        std::string diagnostic;
    };

    struct PreparedRoadTile {
        std::map<std::string, RoadMeshGeometry> geometryByBatch;
        std::vector<std::tuple<std::string, std::string, std::size_t>> skipped;
        std::unordered_set<std::string> editorOnlyProfileKeys;
        std::unordered_map<std::string, std::shared_ptr<Image>> decodedTextures;
        std::unordered_map<std::string, std::string> textureErrors;
        std::vector<ModelVertex> waterVertices;
        std::vector<std::uint32_t> waterIndices;
        double waterMinimumHeight = 0.0;
        double waterMaximumHeight = 0.0;
        double waterMeanHeight = 0.0;
        std::string waterError;
        std::optional<openbus::map::WaterData> waterData;
    };

    struct RoadSplineInput {
        openbus::map::MapSplinePlacement spline;
        openbus::map::MapRoadSplineJoins joins;
        std::filesystem::path profilePath;
        std::optional<openbus::map::MapSplineProfile> profile;
    };

    struct IndexedSceneryObject {
        std::string assetPath;
        openbus::map::MapSceneryPose pose;
        std::vector<openbus::map::MapTileRule> rules;
        int tileX = 0;
        int tileY = 0;
        bool transformValid = false;
        bool splineTerrainAlign = false;
    };
    struct IndexedAttachedObject {
        std::string assetPath;
        int parentId = 0;
        int attachmentIndex = 0;
        std::array<double, 3> rotationDegrees = {};
        std::vector<openbus::map::MapTileRule> rules;
        int tileX = 0;
        int tileY = 0;
        bool transformValid = false;
        bool splineTerrainAlign = false;
    };
    enum class MapObjectRecordKind {
        Scenery,
        Attached,
        SplineAttachment,
    };
    struct ResolvedSceneryPlacement {
        std::string assetPath;
        openbus::map::MapSceneryPose pose;
        bool worldHeightResolved = true;
    };

    // Phases, in execution order.
    void configureGroundTextures();
    void loadGroundTextures();
    bool loadTerrainTiles(const std::unordered_set<std::uint64_t>& tileKeys,
                          const openbus::map::MapTileCoordinate& priorityCenter,
                          bool waitForCompletion);
    void indexMapContent();
    void indexSceneryTrafficPathsMapWide();
    bool finishTrafficLaneNetworkPreparation(bool waitForCompletion);
    void buildTileContent(const openbus::map::MapTileReference& tileReference,
                          const openbus::map::MapTileData& tile, PreparedRoadTile& preparedRoads);
    bool requestTileRoadPreparation(const openbus::map::MapTileData& tile,
                                    const openbus::map::MapTileReference& tileReference,
                                    std::uint64_t tileKeyValue);
    static PreparedRoadTile prepareRoadTile(const std::vector<RoadSplineInput>& splines,
                                            const std::filesystem::path& omsiRoot, int tileX,
                                            int tileY,
                                            const openbus::map::MapCoordinates& coordinates,
                                            bool hasWaterMarker,
                                            const std::filesystem::path& waterPath);
    std::vector<std::string> collectTileSceneryAssets(const openbus::map::MapTileData& tile);
    void requestTileSceneryAssets(const std::vector<std::string>& assetKeys,
                                  std::uint64_t tileKeyValue);
    bool completeSceneryAssetPreparation(std::uint64_t tileKeyValue, bool waitForCompletion);
    bool sceneryAssetPreparationsReady(const std::vector<std::string>& assetKeys) const;
    bool tilePreparationRetryReady(std::uint64_t tileKeyValue) const;
    void recordTilePreparationFailure(std::uint64_t tileKeyValue, bool terrainPreparation,
                                      const std::string& stage, const std::filesystem::path& path,
                                      std::exception_ptr error, bool waitForCompletion);
    static PreparedSceneryAsset prepareSceneryAsset(const std::filesystem::path& configPath,
                                                    const std::filesystem::path& omsiRoot,
                                                    SharedMapTextureImages& textureImages);
    static PreparedSceneryMesh prepareSceneryMesh(const std::filesystem::path& meshPath);
    void resolveTrafficLightVariableParents();
    bool uploadRoadBatches(bool waitForCompletion);

    void rememberPreparedTexture(const std::string& key, std::shared_ptr<Image> image);
    GLuint uploadPreparedTexture(const std::filesystem::path& path, bool repeat);
    GLuint getTexture(const std::filesystem::path& path, bool repeat = true);
    [[nodiscard]] std::optional<double> sampleTerrainHeight(int tileX, int tileY, double localX,
                                                            double localY) const;
    [[nodiscard]] std::optional<std::array<double, 3>> sampleTerrainNormal(double worldX,
                                                                           double worldY) const;
    [[nodiscard]] std::filesystem::path
    resolveSceneryConfigPath(const std::string& assetPath) const;
    const SceneryObjectConfig* loadSceneryObjectMetadata(const std::string& assetPath);
    const std::vector<std::string>& loadCarparkVehicleList(int listIndex);
    const std::vector<std::size_t>& loadSceneryAsset(const std::filesystem::path& configPath);
    const openbus::map::MapSplineProfile* cachedSplineProfile(const std::filesystem::path& path);
    void applyTerrainAlignment(openbus::map::MapSceneryPose& pose, bool requested,
                               const std::string& description);
    std::optional<ResolvedSceneryPlacement> resolveMapSceneryPlacement(int objectId,
                                                                       bool useCache = true);

    const openbus::map::MapDefinition& map;
    std::size_t centerTileIndex;
    std::size_t groundTextureIndex;
    std::filesystem::path omsiRoot;
    Impl* impl_;
    AssetRequestManager* assetRequests_;
    std::unordered_map<std::uint64_t, openbus::map::TerrainGrid>& terrainByCoordinate;
    std::unordered_map<std::uint64_t, std::future<PreparedTerrainTile>> pendingTerrainRequests;
    std::unordered_map<std::uint64_t, std::future<openbus::map::MapTileData>>
        pendingTileDataRequests;
    std::unordered_map<std::uint64_t, openbus::map::MapTileData> pendingTileContentData;
    std::unordered_map<std::uint64_t, std::vector<std::string>> pendingTileAssetKeys;
    std::unordered_map<std::uint64_t, std::future<PreparedRoadTile>> pendingTileRoadRequests;
    std::unordered_map<std::uint64_t, PreparedRoadTile> preparedTileRoads;
    std::unordered_map<std::uint64_t, std::vector<PendingSceneryAsset>> pendingTileSceneryRequests;
    std::unordered_map<std::string, std::shared_future<std::shared_ptr<PreparedSceneryAsset>>>
        pendingSceneryAssetRequests;
    std::unordered_map<std::string, std::shared_ptr<PreparedSceneryAsset>>
        preparedSceneryAssetConfigs;
    std::unordered_map<std::uint64_t, TilePreparationFailure> tilePreparationFailures;

    std::chrono::steady_clock::time_point mapLoadStart = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point lastTileProgressLog{};
    bool uniformGroundTexturePreview = false;
    std::size_t baseGroundTextureIndex = 0;
    double baseTextureRepeatsPerTile = 1.0;
    SkipSummaries skipped;
    std::unordered_map<std::string, GLuint> textureCache;
    PreparedMapTextures preparedTextureImages;
    std::shared_ptr<SharedMapTextureImages> sharedSceneryTextureImages_ =
        std::make_shared<SharedMapTextureImages>();
    std::unordered_map<std::string, std::string> preparedTextureErrors;
    std::filesystem::path baseTexturePath_;
    std::future<Image> baseTextureRequest_;
    std::future<void> mapIndexRequest_;
    bool loadStarted_ = false;
    bool loadFinished_ = false;

    std::unordered_map<std::string, std::vector<std::size_t>> sceneryAssetBatches;
    std::unordered_map<std::string, SceneryObjectConfig> sceneryObjectMetadata;
    std::unordered_map<int, std::vector<std::string>> carparkVehicleLists_;
    std::unordered_map<std::string, std::size_t> sceneryLodThresholdSetByAsset;
    std::unordered_map<std::string, std::vector<ModelAttachmentPoint>> sceneryAttachmentPoints;
    std::unordered_map<std::string, std::vector<SceneryObjectTrafficPath>> sceneryTrafficPaths;
    struct EditorPassengerAsset {
        bool busStop = false;
        bool passengerCabin = false;
        bool hasBoundingBox = false;
        std::array<double, 6> boundingBox = {};
        std::vector<PassengerCabinPosition> positions;
    };
    std::unordered_map<std::string, EditorPassengerAsset> editorPassengerAssets;
    std::unordered_map<std::string,
                       std::vector<std::optional<openbus::map::MapTrafficSignalSchedule>>>
        sceneryTrafficLightSchedules;
    std::unordered_map<std::string, VehicleConfig> sceneryScriptConfigurations;
    std::unordered_map<std::string, std::vector<ModelScriptTexture>> sceneryModelScriptTextures;
    std::unordered_map<std::string, std::vector<ModelTextTexture>> sceneryModelTextTextures;
    std::unordered_map<std::string, std::vector<std::string>> sceneryModelFloatVariables;
    std::unordered_map<std::string, std::vector<std::string>> sceneryModelStringVariables;
    std::unordered_map<std::string, std::vector<std::string>> sceneryFreeTextureVariables;
    std::unordered_map<std::string, std::shared_ptr<Impl::SceneryRuntimeDefinition>>
        sceneryRuntimeDefinitions;
    std::unordered_set<std::string> sceneryAssetsNeedingVariables;
    std::unordered_set<std::string> sceneryTrafficLightAssets;
    std::unordered_set<std::string> poleCrashSceneryAssets;
    std::unordered_set<std::string> editorOnlySceneryAssets;
    std::unordered_set<std::string> absoluteHeightSceneryAssets;
    bool verboseAlphaMaterials = false;
    double carParkChancePercent_ = 50.0;

    std::map<std::string, RoadMeshGeometry> roadVertices;
    std::unordered_map<std::string, openbus::map::MapSplineProfile> splineProfiles;
    std::unordered_set<std::string> failedSplineProfiles;
    std::unordered_set<std::string> editorOnlySplineProfiles;
    std::unordered_map<std::string, std::optional<double>> splineCantWidths;
    openbus::map::MapSplineCantWidthResolver splineCantWidth;

    std::size_t tilesProcessed = 0;
    std::size_t sceneryPlacementsParsed = 0;
    std::size_t sceneryAssetCollectionPasses = 0;
    std::size_t sceneryAssetPathResolutions = 0;
    std::size_t sceneryAssetRequestBatches = 0;
    std::size_t sceneryTextureLookups = 0;
    std::size_t sceneryTextureResolutions = 0;
    std::size_t terrainTextureResolutions = 0;
    std::size_t terrainTexturePathReuses = 0;
    std::size_t splinePlacementsParsed = 0;
    bool roadTrafficLaneNetworkDirty = false;
    bool humanTrafficLaneNetworkDirty = false;
    std::future<openbus::map::MapTrafficLaneNetwork> pendingRoadTrafficLaneNetworkFinalization_;
    std::future<openbus::map::MapTrafficLaneNetwork> pendingHumanTrafficLaneNetworkFinalization_;
    bool terrainLoadPhaseLogged = false;
    std::unordered_map<int, IndexedSceneryObject> sceneryObjectsById;
    std::unordered_map<int, IndexedAttachedObject> attachedObjectsById;
    std::unordered_map<int, std::size_t> splineAttachmentTileById;
    std::unordered_map<int, std::unordered_set<std::size_t>> mapWideRoadSceneryPathIndexes;
    std::unordered_map<int, std::unordered_set<std::size_t>> mapWideHumanSceneryPathIndexes;
    std::unordered_map<std::uint64_t, openbus::map::WaterData> indexedTrafficWaterByTile;
    std::unordered_map<int, MapObjectRecordKind> mapObjectRecordKindsById;
    std::unordered_set<int> ambiguousMapObjectIds;
    std::unordered_map<int, std::uint64_t> mapObjectTileKeysById;
    std::unordered_map<int, std::string> mapObjectAssetPathsById;
    std::unordered_map<int, int> mapObjectVariableParentsById;
    std::unordered_map<int, openbus::map::MapSplinePlacementReference> mapSplinesById;
    std::unordered_set<int> ambiguousMapSplineIds;
    openbus::map::MapRoadConnectionIndex roadConnections;
    openbus::map::MapSplineChainResolver mapSplineResolver;
    std::unordered_map<int, ResolvedSceneryPlacement> resolvedPlacements;
    std::unordered_map<std::size_t, openbus::map::MapTileData> splineAttachmentParentTiles;
    std::unordered_set<int> resolvingPlacementIds;
    std::unordered_set<std::uint64_t> loadedTerrainTileKeys;
    std::unordered_set<std::uint64_t> loadedContentTileKeys;
};

} // namespace openbus::rendering
