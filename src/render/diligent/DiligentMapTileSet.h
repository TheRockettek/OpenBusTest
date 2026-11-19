#pragma once

#include "map/MapCoordinates.h"
#include "map/MapRoadGeometry.h"
#include "map/MapTileWindow.h"
#include "render/diligent/DiligentMapTileAsset.h"

#include <deque>
#include <chrono>
#include <filesystem>
#include <future>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace Diligent {
struct IRenderDevice;
}

namespace openbus::rendering {

class DiligentMapTileSet final {
  public:
    DiligentMapTileSet(Diligent::IRenderDevice& device, double tileSize,
                       bool generateMipmaps = false);
    DiligentMapTileSet(Diligent::IRenderDevice& device, openbus::map::MapCoordinates coordinates,
                       bool generateMipmaps = false);
    ~DiligentMapTileSet();
    DiligentMapTileSet(const DiligentMapTileSet&) = delete;
    DiligentMapTileSet& operator=(const DiligentMapTileSet&) = delete;

    void addTile(int tileX, int tileY, const openbus::map::MapTerrainMesh* terrain,
                 std::span<const DiligentMapTerrainLayerSource> terrainLayers,
                 const openbus::map::MapWaterMesh* water, DiligentMapWaterSource waterSource,
                 std::span<const DiligentMapRoadSource> roads);
    void addTerrainTileFromFiles(
        const openbus::map::MapDefinition& map, const openbus::map::MapTileReference& tile,
        const std::filesystem::path& omsiRoot, std::size_t baseGroundTextureIndex,
        std::shared_ptr<const openbus::map::MapRoadConnectionIndex> roadConnections = {});
    std::size_t updateTerrainWindow(
        const openbus::map::MapDefinition& map, double worldX, double worldY, int radius,
        const std::filesystem::path& omsiRoot, std::size_t baseGroundTextureIndex,
        std::shared_ptr<const openbus::map::MapRoadConnectionIndex> roadConnections = {});
    std::size_t requestTerrainWindow(
        const openbus::map::MapDefinition& map, double worldX, double worldY, int radius,
        const std::filesystem::path& omsiRoot, std::size_t baseGroundTextureIndex,
        std::shared_ptr<const openbus::map::MapRoadConnectionIndex> roadConnections = {});
    // Publishes completed CPU preparations on the calling thread, then fills free slots in the
    // four-job worker limit. Call regularly to keep the requested window moving. Uploads therefore
    // stay on the renderer thread, and preparation failures are rethrown to the caller.
    std::size_t publishReadyTerrainTiles();
    void removeTile(int tileX, int tileY);
    void updateActiveTiles(double worldX, double worldY, int radius);
    std::size_t removeInactiveTiles();

    [[nodiscard]] std::size_t tileCount() const noexcept;
    [[nodiscard]] std::span<const DiligentMapTileAsset* const> activeTiles() const noexcept;

    // Draws terrain in tile/layer order and sorts transparent water/road batches by projected
    // depth. Supplying both motionFrame and motionHistory derives previous clip transforms per
    // tile.
    std::size_t drawTerrain(DiligentTexturedMeshRenderer& renderer,
                            Diligent::IDeviceContext& context,
                            Diligent::ITextureView& fallbackTexture, const Matrix4& viewProjection,
                            const DiligentMaterialState* materialOverride = nullptr,
                            const DiligentLightingParameters& lighting = {},
                            const DiligentMotionParameters* motion = nullptr,
                            const MotionDataFrame* motionFrame = nullptr,
                            MotionHistory* motionHistory = nullptr,
                            const DiligentMapWeather* weather = nullptr) const;
    std::size_t drawWater(DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
                          Diligent::ITextureView& fallbackTexture, const Matrix4& viewProjection,
                          const DiligentMaterialState* materialOverride = nullptr,
                          const DiligentLightingParameters& lighting = {},
                          const DiligentMotionParameters* motion = nullptr,
                          const MotionDataFrame* motionFrame = nullptr,
                          MotionHistory* motionHistory = nullptr) const;
    std::size_t drawRoads(DiligentMapRoadPass pass, DiligentTexturedMeshRenderer& renderer,
                          Diligent::IDeviceContext& context,
                          Diligent::ITextureView& fallbackTexture, const Matrix4& viewProjection,
                          const DiligentMaterialState* materialOverride = nullptr,
                          const DiligentLightingParameters& lighting = {},
                          const DiligentMotionParameters* motion = nullptr,
                          const MotionDataFrame* motionFrame = nullptr,
                          MotionHistory* motionHistory = nullptr,
                          const DiligentMapWeather* weather = nullptr) const;

  private:
    struct PreparedTerrainTile {
        int tileX = 0;
        int tileY = 0;
        std::string sourceSignature;
        openbus::map::MapTerrainMesh geometry;
        std::vector<DiligentMapTerrainLayerSource> terrainLayers;
        std::optional<openbus::map::MapWaterMesh> water;
        DiligentMapWaterSource waterSource;
        std::vector<openbus::map::RoadMeshGeometry> roadGeometry;
    };

    struct TerrainWindowState {
        double worldX = 0.0;
        double worldY = 0.0;
        int radius = 0;
        std::vector<std::pair<openbus::map::MapTileCoordinate, std::string>> sourceSignatures;
    };

    struct PendingTerrainTile {
        std::string sourceSignature;
        std::future<PreparedTerrainTile> preparation;
    };

    struct TerrainPreparationFailure {
        std::string sourceSignature;
        unsigned attempts = 0;
        std::chrono::steady_clock::time_point retryAfter{};
    };

    struct TerrainPreparationRequest {
        openbus::map::MapTileCoordinate coordinate;
        std::string sourceSignature;
        openbus::map::MapCoordinates coordinates;
        std::filesystem::path mapRoot;
        openbus::map::MapTileReference tile;
        std::filesystem::path omsiRoot;
        std::shared_ptr<const std::vector<openbus::map::GroundTextureDefinition>> groundTextures;
        std::shared_ptr<const openbus::map::MapRoadConnectionIndex> roadConnections;
        std::size_t baseGroundTextureIndex = 0;
    };

    struct TileKeyHash {
        std::size_t operator()(const openbus::map::MapTileCoordinate& tile) const noexcept;
    };

    [[nodiscard]] static PreparedTerrainTile prepareTerrainTile(
        const openbus::map::MapCoordinates& coordinates, const std::filesystem::path& mapRoot,
        const openbus::map::MapTileReference& tile, const std::filesystem::path& omsiRoot,
        std::span<const openbus::map::GroundTextureDefinition> groundTextures,
        std::size_t baseGroundTextureIndex,
        const openbus::map::MapRoadConnectionIndex* roadConnections);
    void addPreparedTerrainTile(PreparedTerrainTile&& prepared);
    std::size_t schedulePendingTerrainTiles();

    Diligent::IRenderDevice& device_;
    openbus::map::MapCoordinates coordinates_;
    bool generateMipmaps_;
    std::unordered_map<openbus::map::MapTileCoordinate, std::unique_ptr<DiligentMapTileAsset>,
                       TileKeyHash>
        tiles_;
    std::vector<const DiligentMapTileAsset*> activeTiles_;
    std::unordered_map<openbus::map::MapTileCoordinate, PendingTerrainTile, TileKeyHash>
        pendingTerrainTiles_;
    std::deque<TerrainPreparationRequest> queuedTerrainTiles_;
    std::unordered_map<openbus::map::MapTileCoordinate, std::string, TileKeyHash>
        terrainSourceSignatures_;
    std::unordered_map<openbus::map::MapTileCoordinate, TerrainPreparationFailure, TileKeyHash>
        terrainPreparationFailures_;
    std::optional<TerrainWindowState> terrainWindow_;
};

} // namespace openbus::rendering
