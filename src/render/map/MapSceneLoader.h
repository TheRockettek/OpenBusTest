#pragma once

// Builds all static map content (terrain, scenery, splines/roads) into a
// MapRenderer::Impl. The loader's members hold intermediate caches and indexes
// that are shared between load phases; it is discarded once loading finishes.

#include "render/map/MapRendererInternal.h"

namespace openbus::rendering {

class MapSceneLoader {
  public:
    MapSceneLoader(const openbus::map::MapDefinition& map, std::size_t centerTileIndex,
                   std::size_t groundTextureIndex, const std::filesystem::path& omsiRoot,
                   MapRenderer::Impl& impl);

    // Runs every load phase. Throws std::runtime_error when the map is unusable.
    void load();
    // Logs totals and skip summaries; call after the initial streaming update.
    void logSummary() const;

  private:
    using Impl = MapRenderer::Impl;

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
    struct ResolvedSceneryPlacement {
        std::string assetPath;
        openbus::map::MapSceneryPose pose;
    };

    // Phases, in execution order.
    void loadGroundTextures();
    void loadTerrainTiles();
    void indexMapContent();
    void buildTileContent();
    void uploadRoadBatches();

    GLuint getTexture(const std::filesystem::path& path, bool repeat = true);
    [[nodiscard]] std::optional<double> sampleTerrainHeight(int tileX, int tileY, double localX,
                                                            double localY) const;
    [[nodiscard]] std::optional<std::array<double, 3>> sampleTerrainNormal(double worldX,
                                                                           double worldY) const;
    [[nodiscard]] std::filesystem::path
    resolveSceneryConfigPath(const std::string& assetPath) const;
    const std::vector<std::size_t>& loadSceneryAsset(const std::filesystem::path& configPath);
    const openbus::map::MapSplineProfile* cachedSplineProfile(const std::filesystem::path& path);
    void applyTerrainAlignment(openbus::map::MapSceneryPose& pose, bool requested,
                               const std::string& description);
    std::optional<ResolvedSceneryPlacement> resolveMapSceneryPlacement(int objectId);

    const openbus::map::MapDefinition& map;
    std::size_t centerTileIndex;
    std::size_t groundTextureIndex;
    const std::filesystem::path& omsiRoot;
    Impl* impl_;
    std::unordered_map<std::uint64_t, openbus::map::TerrainGrid>& terrainByCoordinate;

    std::chrono::steady_clock::time_point mapLoadStart = std::chrono::steady_clock::now();
    bool uniformGroundTexturePreview = false;
    std::size_t baseGroundTextureIndex = 0;
    double baseTextureRepeatsPerTile = 1.0;
    SkipSummaries skipped;
    std::unordered_map<std::string, GLuint> textureCache;

    std::unordered_map<std::string, std::vector<std::size_t>> sceneryAssetBatches;
    std::unordered_map<std::string, std::size_t> sceneryLodThresholdSetByAsset;
    std::unordered_map<std::string, std::vector<ModelAttachmentPoint>> sceneryAttachmentPoints;
    std::unordered_map<std::string, std::vector<SceneryObjectTrafficPath>> sceneryTrafficPaths;
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
    bool verboseAlphaMaterials = false;

    std::map<std::string, RoadMeshGeometry> roadVertices;
    std::unordered_map<std::string, openbus::map::MapSplineProfile> splineProfiles;
    std::unordered_set<std::string> failedSplineProfiles;
    std::unordered_set<std::string> editorOnlySplineProfiles;
    std::unordered_map<std::string, std::optional<double>> splineCantWidths;
    openbus::map::MapSplineCantWidthResolver splineCantWidth;

    std::chrono::steady_clock::time_point mapContentStart;
    std::size_t tilesProcessed = 0;
    std::size_t sceneryPlacementsParsed = 0;
    std::size_t splinePlacementsParsed = 0;
    std::unordered_map<int, IndexedSceneryObject> sceneryObjectsById;
    std::unordered_map<int, IndexedAttachedObject> attachedObjectsById;
    std::unordered_map<int, std::size_t> splineAttachmentTileById;
    std::unordered_map<int, openbus::map::MapSplinePlacementReference> mapSplinesById;
    std::unordered_set<int> ambiguousMapSplineIds;
    std::unordered_map<std::uint64_t, std::unordered_set<int>> editorObjectNumbersByTile;
    openbus::map::MapRoadConnectionIndex roadConnections;
    openbus::map::MapSplineChainResolver mapSplineResolver;
    std::unordered_map<int, ResolvedSceneryPlacement> resolvedPlacements;
    std::unordered_map<std::size_t, openbus::map::MapTileData> splineAttachmentParentTiles;
    std::unordered_set<int> resolvingPlacementIds;
};

} // namespace openbus::rendering
