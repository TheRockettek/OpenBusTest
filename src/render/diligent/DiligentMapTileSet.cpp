#include "render/diligent/DiligentMapTileSet.h"

#include "core/AssetPath.h"
#include "core/Logger.h"
#include "core/PerfTrace.h"
#include "core/Utf8Path.h"
#include "config/TextureSurfaceConfigLoader.h"
#include "map/MapRoadRenderGeometry.h"
#include "render/MaterialConstants.h"
#include "render/MatrixMath.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>

namespace openbus::rendering {
namespace {

std::filesystem::path resolveMapTexturePath(const std::filesystem::path& mapRoot,
                                            const std::filesystem::path& omsiRoot,
                                            const std::string& configuredPath) {
    TraceScope trace("filesystem", "DiligentMapTileSet.resolveMapTexturePath");
    std::string normalized = configuredPath;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    const std::filesystem::path relative = openbus::pathFromUtf8(normalized);
    std::filesystem::path mapRelative;
    auto component = relative.begin();
    if (component != relative.end() && openbus::pathToUtf8(*component) == "maps") {
        ++component;
        if (component != relative.end()) {
            ++component;
        }
        for (; component != relative.end(); ++component) {
            mapRelative /= *component;
        }
    }
    const std::array<std::filesystem::path, 3> candidates = {
        mapRoot / relative, omsiRoot / relative,
        mapRelative.empty() ? std::filesystem::path{} : mapRoot / mapRelative};
    for (const std::filesystem::path& candidate : candidates) {
        if (candidate.empty()) {
            continue;
        }
        const std::filesystem::path resolved = openbus::assets::resolveTexturePath(candidate);
        if (std::filesystem::is_regular_file(resolved)) {
            return resolved;
        }
    }
    throw std::runtime_error("Could not resolve Diligent map ground texture: " + configuredPath);
}

std::filesystem::path resolveOptionalMapTexturePath(const std::filesystem::path& path) {
    TraceScope trace("filesystem", "DiligentMapTileSet.resolveOptionalMapTexturePath");
    if (path.empty()) {
        return {};
    }
    const std::filesystem::path resolved = openbus::assets::resolveTexturePath(path);
    return std::filesystem::is_regular_file(resolved) ? resolved : std::filesystem::path{};
}

std::filesystem::path resolveSplineProfilePath(const std::filesystem::path& mapRoot,
                                               const std::filesystem::path& omsiRoot,
                                               const std::string& assetPath) {
    TraceScope trace("filesystem", "DiligentMapTileSet.resolveSplineProfilePath");
    std::string normalized = assetPath;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    const std::filesystem::path relative = openbus::pathFromUtf8(normalized);
    const std::array<std::filesystem::path, 2> candidates = {omsiRoot / relative,
                                                             mapRoot / relative};
    for (const std::filesystem::path& candidate : candidates) {
        if (std::filesystem::is_regular_file(candidate)) {
            return candidate;
        }
    }
    return {};
}

double projectedSortDepth(const Matrix4& viewProjection,
                          const std::array<double, 3>& worldPosition) {
    if (!std::all_of(worldPosition.begin(), worldPosition.end(),
                     [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("Diligent map transparent geometry has invalid world position");
    }
    const double clipZ = viewProjection[2] * worldPosition[0] +
                         viewProjection[6] * worldPosition[1] +
                         viewProjection[10] * worldPosition[2] + viewProjection[14];
    const double clipW = viewProjection[3] * worldPosition[0] +
                         viewProjection[7] * worldPosition[1] +
                         viewProjection[11] * worldPosition[2] + viewProjection[15];
    if (!std::isfinite(clipZ) || !std::isfinite(clipW) || std::abs(clipW) <= 1.0e-12) {
        throw std::invalid_argument("Diligent map transparent geometry has invalid clip depth");
    }
    const double depth = clipZ / clipW;
    if (!std::isfinite(depth)) {
        throw std::invalid_argument(
            "Diligent map transparent geometry has invalid projected depth");
    }
    return depth;
}

std::string
terrainSourceSignature(const std::filesystem::path& mapRoot, const std::filesystem::path& omsiRoot,
                       const openbus::map::MapTileReference& tile,
                       std::span<const openbus::map::GroundTextureDefinition> groundTextures,
                       std::size_t baseGroundTextureIndex,
                       const openbus::map::MapRoadConnectionIndex* roadConnections) {
    std::string signature;
    const auto append = [&signature](const std::string& value) {
        signature += std::to_string(value.size());
        signature.push_back(':');
        signature += value;
    };
    const auto appendPath = [&append](const std::filesystem::path& path) {
        append(openbus::pathToUtf8(path.lexically_normal()));
    };
    appendPath(mapRoot);
    appendPath(omsiRoot);
    append(std::to_string(tile.hasTerrainFile));
    appendPath(tile.terrainPath);
    append(std::to_string(tile.hasWaterFile));
    appendPath(tile.waterPath);
    append(std::to_string(baseGroundTextureIndex));
    if (roadConnections != nullptr) {
        appendPath(tile.textPath);
        append(std::to_string(tile.chronoOverlayPaths.size()));
        for (const std::filesystem::path& overlayPath : tile.chronoOverlayPaths) {
            appendPath(overlayPath);
        }
        append(std::to_string(roadConnections->sourceSignature()));
    }
    for (const openbus::map::GroundTextureDefinition& groundTexture : groundTextures) {
        append(groundTexture.texturePath);
        append(groundTexture.detailTexturePath);
        for (const std::string& parameter : groundTexture.parameters) {
            append(parameter);
        }
    }
    for (const openbus::map::MapGroundTextureSidecar& sidecar : tile.groundTextureSidecars) {
        append(sidecar.ordinalSuffix);
        appendPath(sidecar.path);
    }
    return signature;
}

void validateMotionHistoryPair(const MotionDataFrame* frame, const MotionHistory* history) {
    if ((frame == nullptr) != (history == nullptr)) {
        throw std::invalid_argument(
            "Diligent map motion requires both frame data and motion history");
    }
}

} // namespace

DiligentMapTileSet::DiligentMapTileSet(Diligent::IRenderDevice& device, double tileSize,
                                       bool generateMipmaps)
    : DiligentMapTileSet(device, openbus::map::MapCoordinates{tileSize, false}, generateMipmaps) {}

DiligentMapTileSet::DiligentMapTileSet(Diligent::IRenderDevice& device,
                                       openbus::map::MapCoordinates coordinates,
                                       bool generateMipmaps)
    : device_(device), coordinates_(coordinates), generateMipmaps_(generateMipmaps) {
    if (!std::isfinite(coordinates_.tileSize) || coordinates_.tileSize <= 0.0) {
        throw std::invalid_argument("Map tile set size must be finite and positive");
    }
}

DiligentMapTileSet::~DiligentMapTileSet() = default;

std::size_t DiligentMapTileSet::TileKeyHash::operator()(
    const openbus::map::MapTileCoordinate& tile) const noexcept {
    const std::size_t xHash = std::hash<int>{}(tile.first);
    const std::size_t yHash = std::hash<int>{}(tile.second);
    return xHash ^ (yHash + static_cast<std::size_t>(0x9e3779b9U) + (xHash << 6U) + (xHash >> 2U));
}

void DiligentMapTileSet::addTile(int tileX, int tileY, const openbus::map::MapTerrainMesh* terrain,
                                 std::span<const DiligentMapTerrainLayerSource> terrainLayers,
                                 const openbus::map::MapWaterMesh* water,
                                 DiligentMapWaterSource waterSource,
                                 std::span<const DiligentMapRoadSource> roads) {
    const openbus::map::MapTileCoordinate coordinate{tileX, tileY};
    if (tiles_.contains(coordinate)) {
        throw std::invalid_argument("Map tile is already present in the Diligent tile set");
    }
    auto asset =
        std::make_unique<DiligentMapTileAsset>(device_, tileX, tileY, terrain, terrainLayers, water,
                                               std::move(waterSource), roads, generateMipmaps_);
    tiles_.emplace(coordinate, std::move(asset));
}

DiligentMapTileSet::PreparedTerrainTile DiligentMapTileSet::prepareTerrainTile(
    const openbus::map::MapCoordinates& coordinates, const std::filesystem::path& mapRoot,
    const openbus::map::MapTileReference& tile, const std::filesystem::path& omsiRoot,
    std::span<const openbus::map::GroundTextureDefinition> groundTextures,
    std::size_t baseGroundTextureIndex,
    const openbus::map::MapRoadConnectionIndex* roadConnections) {
    TraceScope trace("filesystem", "DiligentMapTileSet.prepareTerrainTile");
    if (!tile.hasTerrainFile || tile.terrainPath.empty()) {
        throw std::invalid_argument("Diligent map terrain tile has no terrain file");
    }
    if (baseGroundTextureIndex >= groundTextures.size()) {
        throw std::out_of_range("Diligent map base ground-texture index is out of range");
    }

    const openbus::map::TerrainGrid grid = openbus::map::loadTerrainGrid(tile.terrainPath);
    PreparedTerrainTile prepared;
    prepared.tileX = tile.x;
    prepared.tileY = tile.y;
    prepared.sourceSignature = terrainSourceSignature(mapRoot, omsiRoot, tile, groundTextures,
                                                      baseGroundTextureIndex, roadConnections);
    prepared.geometry = openbus::map::prepareMapTerrainMesh(grid, coordinates.tileSize);
    const float inverseIntervals = 1.0F / static_cast<float>(grid.intervals);
    prepared.terrainLayers.reserve(tile.groundTextureSidecars.size() + 1);

    const auto makeLayer = [&](std::size_t textureIndex) {
        const openbus::map::GroundTextureDefinition& groundTexture = groundTextures[textureIndex];
        const openbus::map::MapGroundTextureTiling tiling =
            openbus::map::mapGroundTextureTiling(groundTexture);
        DiligentMapTerrainLayerSource layer;
        layer.texturePath = resolveMapTexturePath(mapRoot, omsiRoot, groundTexture.texturePath);
        layer.material.textureScale = {
            static_cast<float>(tiling.textureRepeatsPerTile) * inverseIntervals,
            static_cast<float>(tiling.textureRepeatsPerTile) * inverseIntervals};
        if (!groundTexture.detailTexturePath.empty()) {
            layer.detailMapPath =
                resolveMapTexturePath(mapRoot, omsiRoot, groundTexture.detailTexturePath);
            layer.material.detailMapScale = {
                static_cast<float>(tiling.detailTextureRepeatsPerTile) * inverseIntervals,
                static_cast<float>(tiling.detailTextureRepeatsPerTile) * inverseIntervals};
        }
        return layer;
    };

    prepared.terrainLayers.push_back(makeLayer(baseGroundTextureIndex));
    std::map<std::size_t, const openbus::map::MapGroundTextureSidecar*> sidecarsByIndex;
    for (const openbus::map::MapGroundTextureSidecar& sidecar : tile.groundTextureSidecars) {
        const std::optional<std::size_t> textureIndex =
            openbus::map::resolveMapGroundTextureSidecarIndex(sidecar.ordinalSuffix,
                                                              groundTextures.size());
        if (!textureIndex) {
            throw std::invalid_argument("Diligent map terrain has an invalid ground-texture "
                                        "sidecar index: " +
                                        sidecar.ordinalSuffix);
        }
        if (!sidecarsByIndex.emplace(*textureIndex, &sidecar).second) {
            throw std::invalid_argument("Diligent map terrain has duplicate ground-texture "
                                        "sidecar indices");
        }
    }
    for (const auto& [textureIndex, sidecar] : sidecarsByIndex) {
        DiligentMapTerrainLayerSource layer = makeLayer(textureIndex);
        layer.transMapPath = sidecar->path;
        layer.material.alphaMode = DiligentAlphaMode::Blend;
        layer.material.transMapScale = {inverseIntervals, inverseIntervals};
        layer.material.flipTransmapY = true;
        prepared.terrainLayers.push_back(std::move(layer));
    }

    if (tile.hasWaterFile) {
        const openbus::map::WaterData waterData = openbus::map::loadWaterData(tile.waterPath);
        prepared.water =
            openbus::map::prepareMapWaterMesh(waterData, tile.x, tile.y, coordinates.tileSize);
        prepared.waterSource.material.alphaMode = DiligentAlphaMode::Blend;
        prepared.waterSource.material.baseColor = {1.0F, 1.0F, 1.0F, 0.62F};
        prepared.waterSource.material.depthMode = DiligentDepthMode::LessEqual;
        prepared.waterSource.material.depthWrite = false;
        prepared.waterSource.material.normalMapStrength = 0.35F;
        prepared.waterSource.material.environmentStrength =
            static_cast<float>(openbus::rendering::ENVIRONMENT_MAP_OPACITY);
        prepared.waterSource.texturePath =
            resolveOptionalMapTexturePath(mapRoot / "texture" / "water.tga");
        prepared.waterSource.normalMapPath =
            resolveOptionalMapTexturePath(mapRoot / "texture" / "water_bump.bmp");
        prepared.waterSource.environmentMapPath =
            resolveOptionalMapTexturePath(mapRoot / "texture" / "water_envmap.bmp");
        if (prepared.waterSource.texturePath.empty()) {
            prepared.waterSource.material.baseColor = {0.24F, 0.48F, 0.60F, 0.62F};
        }
    }
    if (roadConnections != nullptr) {
        const openbus::map::MapTileData mapTile = openbus::map::loadMapTile(tile);
        std::vector<openbus::map::MapSplineProfile> profiles;
        std::vector<openbus::map::MapRoadSplineRenderInput> roadInputs;
        profiles.reserve(mapTile.splines.size());
        roadInputs.reserve(mapTile.splines.size());
        for (const openbus::map::MapSplinePlacement& spline : mapTile.splines) {
            if (!spline.geometryValid) {
                continue;
            }
            const std::filesystem::path profilePath =
                resolveSplineProfilePath(mapRoot, omsiRoot, spline.assetPath);
            if (profilePath.empty()) {
                throw std::runtime_error("Could not resolve Diligent map spline profile: " +
                                         spline.assetPath);
            }
            profiles.push_back(openbus::map::loadMapSplineProfile(profilePath));
            const openbus::map::MapSplineProfile& profile = profiles.back();
            roadInputs.push_back(
                {spline, roadConnections->joinsFor(spline, tile.x, tile.y), profilePath, &profile});
        }
        std::map<std::string, openbus::map::RoadMeshGeometry> roadBatches =
            openbus::map::prepareMapRoadRenderBatches(tile.x, tile.y, coordinates, omsiRoot,
                                                      roadInputs);
        prepared.roadGeometry.reserve(roadBatches.size());
        for (auto& [batchKey, road] : roadBatches) {
            (void)batchKey;
            if (road.textured) {
                const auto surface =
                    openbus::config::loadTextureSurfaceConfiguration(road.texturePath);
                for (const auto& diagnostic : surface.diagnostics.entries) {
                    gameLog.Warning("Diligent road texture surface " +
                                    openbus::pathToUtf8(road.texturePath) + ":" +
                                    std::to_string(diagnostic.line) + ": " + diagnostic.message);
                }
                if (!surface.diagnostics.hasErrors()) {
                    road.supportsMoisture = surface.supportsMoisture;
                    road.weatherClassified = surface.configFound;
                }
            }
            prepared.roadGeometry.push_back(std::move(road));
        }
    }
    return prepared;
}

void DiligentMapTileSet::addPreparedTerrainTile(PreparedTerrainTile&& prepared) {
    const openbus::map::MapTileCoordinate coordinate{prepared.tileX, prepared.tileY};
    std::vector<DiligentMapRoadSource> roadSources;
    roadSources.reserve(prepared.roadGeometry.size());
    for (const openbus::map::RoadMeshGeometry& geometry : prepared.roadGeometry) {
        DiligentMapRoadSource source;
        source.geometry = &geometry;
        source.texturePath = geometry.texturePath;
        roadSources.push_back(std::move(source));
    }
    auto asset = std::make_unique<DiligentMapTileAsset>(
        device_, prepared.tileX, prepared.tileY, &prepared.geometry, prepared.terrainLayers,
        prepared.water ? &*prepared.water : nullptr, std::move(prepared.waterSource), roadSources,
        generateMipmaps_);
    const auto existing = tiles_.find(coordinate);
    if (existing == tiles_.end()) {
        tiles_.emplace(coordinate, std::move(asset));
    } else {
        if (!terrainSourceSignatures_.contains(coordinate)) {
            throw std::invalid_argument("Cannot replace a manually added Diligent map tile");
        }
        existing->second = std::move(asset);
    }
    terrainSourceSignatures_[coordinate] = std::move(prepared.sourceSignature);
    terrainPreparationFailures_.erase(coordinate);
}

void DiligentMapTileSet::addTerrainTileFromFiles(
    const openbus::map::MapDefinition& map, const openbus::map::MapTileReference& tile,
    const std::filesystem::path& omsiRoot, std::size_t baseGroundTextureIndex,
    std::shared_ptr<const openbus::map::MapRoadConnectionIndex> roadConnections) {
    if (map.coordinates.worldCoordinates != coordinates_.worldCoordinates ||
        map.coordinates.tileSize != coordinates_.tileSize) {
        throw std::invalid_argument(
            "Diligent map tile set and map definition use different coordinate systems");
    }
    PreparedTerrainTile prepared =
        prepareTerrainTile(map.coordinates, map.rootPath, tile, omsiRoot, map.groundTextures,
                           baseGroundTextureIndex, roadConnections.get());
    addPreparedTerrainTile(std::move(prepared));
}

std::size_t DiligentMapTileSet::updateTerrainWindow(
    const openbus::map::MapDefinition& map, double worldX, double worldY, int radius,
    const std::filesystem::path& omsiRoot, std::size_t baseGroundTextureIndex,
    std::shared_ptr<const openbus::map::MapRoadConnectionIndex> roadConnections) {
    if (map.coordinates.worldCoordinates != coordinates_.worldCoordinates ||
        map.coordinates.tileSize != coordinates_.tileSize) {
        throw std::invalid_argument(
            "Diligent map tile set and map definition use different coordinate systems");
    }

    std::vector<openbus::map::MapTileCoordinate> mapCoordinates;
    mapCoordinates.reserve(map.tiles.size());
    std::unordered_map<openbus::map::MapTileCoordinate, const openbus::map::MapTileReference*,
                       TileKeyHash>
        references;
    references.reserve(map.tiles.size());
    for (const openbus::map::MapTileReference& tile : map.tiles) {
        const openbus::map::MapTileCoordinate coordinate{tile.x, tile.y};
        if (!references.emplace(coordinate, &tile).second) {
            throw std::invalid_argument(
                "Diligent map definition contains duplicate tile coordinates");
        }
        mapCoordinates.push_back(coordinate);
    }
    const std::vector<openbus::map::MapTileCoordinate> selected =
        openbus::map::selectMapTilesWithinRadius(mapCoordinates, worldX, worldY, radius,
                                                 coordinates_.tileSize);
    std::vector<std::pair<openbus::map::MapTileCoordinate, std::string>> requestedSignatures;
    requestedSignatures.reserve(selected.size());
    for (const openbus::map::MapTileCoordinate& coordinate : selected) {
        const openbus::map::MapTileReference& tile = *references.at(coordinate);
        if (tile.hasTerrainFile) {
            requestedSignatures.emplace_back(
                coordinate, terrainSourceSignature(map.rootPath, omsiRoot, tile, map.groundTextures,
                                                   baseGroundTextureIndex, roadConnections.get()));
        }
    }
    for (auto failure = terrainPreparationFailures_.begin();
         failure != terrainPreparationFailures_.end();) {
        const auto requested =
            std::find_if(requestedSignatures.begin(), requestedSignatures.end(),
                         [&failure](const auto& entry) { return entry.first == failure->first; });
        if (requested == requestedSignatures.end() ||
            requested->second != failure->second.sourceSignature) {
            failure = terrainPreparationFailures_.erase(failure);
        } else {
            ++failure;
        }
    }
    queuedTerrainTiles_.clear();
    terrainWindow_ = TerrainWindowState{worldX, worldY, radius, std::move(requestedSignatures)};
    std::size_t added = 0;
    for (const openbus::map::MapTileCoordinate& coordinate : selected) {
        const openbus::map::MapTileReference& tile = *references.at(coordinate);
        if (!tile.hasTerrainFile) {
            if (terrainSourceSignatures_.contains(coordinate)) {
                removeTile(coordinate.first, coordinate.second);
            }
            continue;
        }
        const std::string sourceSignature =
            terrainSourceSignature(map.rootPath, omsiRoot, tile, map.groundTextures,
                                   baseGroundTextureIndex, roadConnections.get());
        const auto loadedSignature = terrainSourceSignatures_.find(coordinate);
        if (tiles_.contains(coordinate) && (loadedSignature == terrainSourceSignatures_.end() ||
                                            loadedSignature->second == sourceSignature)) {
            continue;
        }
        addTerrainTileFromFiles(map, tile, omsiRoot, baseGroundTextureIndex, roadConnections);
        ++added;
    }
    updateActiveTiles(worldX, worldY, radius);
    return added;
}

std::size_t DiligentMapTileSet::requestTerrainWindow(
    const openbus::map::MapDefinition& map, double worldX, double worldY, int radius,
    const std::filesystem::path& omsiRoot, std::size_t baseGroundTextureIndex,
    std::shared_ptr<const openbus::map::MapRoadConnectionIndex> roadConnections) {
    if (map.coordinates.worldCoordinates != coordinates_.worldCoordinates ||
        map.coordinates.tileSize != coordinates_.tileSize) {
        throw std::invalid_argument(
            "Diligent map tile set and map definition use different coordinate systems");
    }
    if (baseGroundTextureIndex >= map.groundTextures.size()) {
        throw std::out_of_range("Diligent map base ground-texture index is out of range");
    }

    std::vector<openbus::map::MapTileCoordinate> mapCoordinates;
    mapCoordinates.reserve(map.tiles.size());
    std::unordered_map<openbus::map::MapTileCoordinate, const openbus::map::MapTileReference*,
                       TileKeyHash>
        references;
    references.reserve(map.tiles.size());
    for (const openbus::map::MapTileReference& tile : map.tiles) {
        const openbus::map::MapTileCoordinate coordinate{tile.x, tile.y};
        if (!references.emplace(coordinate, &tile).second) {
            throw std::invalid_argument(
                "Diligent map definition contains duplicate tile coordinates");
        }
        mapCoordinates.push_back(coordinate);
    }
    const std::vector<openbus::map::MapTileCoordinate> selected =
        openbus::map::selectMapTilesWithinRadius(mapCoordinates, worldX, worldY, radius,
                                                 coordinates_.tileSize);
    const auto groundTextures =
        std::make_shared<const std::vector<openbus::map::GroundTextureDefinition>>(
            map.groundTextures);

    queuedTerrainTiles_.clear();
    std::vector<std::pair<openbus::map::MapTileCoordinate, std::string>> requestedSignatures;
    for (const openbus::map::MapTileCoordinate& coordinate : selected) {
        const openbus::map::MapTileReference& tile = *references.at(coordinate);
        if (!tile.hasTerrainFile) {
            if (terrainSourceSignatures_.contains(coordinate)) {
                removeTile(coordinate.first, coordinate.second);
            }
            continue;
        }
        std::string sourceSignature =
            terrainSourceSignature(map.rootPath, omsiRoot, tile, *groundTextures,
                                   baseGroundTextureIndex, roadConnections.get());
        requestedSignatures.emplace_back(coordinate, sourceSignature);
        const auto loadedSignature = terrainSourceSignatures_.find(coordinate);
        if (tiles_.contains(coordinate) && (loadedSignature == terrainSourceSignatures_.end() ||
                                            loadedSignature->second == sourceSignature)) {
            continue;
        }
        const auto pending = pendingTerrainTiles_.find(coordinate);
        if (pending != pendingTerrainTiles_.end() &&
            pending->second.sourceSignature == sourceSignature) {
            continue;
        }
        queuedTerrainTiles_.push_back({coordinate, std::move(sourceSignature), map.coordinates,
                                       map.rootPath, tile, omsiRoot, groundTextures,
                                       roadConnections, baseGroundTextureIndex});
    }
    for (auto failure = terrainPreparationFailures_.begin();
         failure != terrainPreparationFailures_.end();) {
        const auto requested =
            std::find_if(requestedSignatures.begin(), requestedSignatures.end(),
                         [&failure](const auto& entry) { return entry.first == failure->first; });
        if (requested == requestedSignatures.end() ||
            requested->second != failure->second.sourceSignature) {
            failure = terrainPreparationFailures_.erase(failure);
        } else {
            ++failure;
        }
    }
    terrainWindow_ = TerrainWindowState{worldX, worldY, radius, std::move(requestedSignatures)};
    const std::size_t scheduled = schedulePendingTerrainTiles();
    updateActiveTiles(worldX, worldY, radius);
    return scheduled;
}

std::size_t DiligentMapTileSet::schedulePendingTerrainTiles() {
    constexpr std::size_t maxConcurrentPreparations = 4;
    std::size_t scheduled = 0;
    std::size_t deferred = 0;
    while (!queuedTerrainTiles_.empty() &&
           pendingTerrainTiles_.size() < maxConcurrentPreparations) {
        TerrainPreparationRequest request = queuedTerrainTiles_.front();
        const auto pending = pendingTerrainTiles_.find(request.coordinate);
        if (pending != pendingTerrainTiles_.end()) {
            if (pending->second.sourceSignature == request.sourceSignature) {
                queuedTerrainTiles_.pop_front();
                deferred = 0;
            } else {
                queuedTerrainTiles_.push_back(std::move(queuedTerrainTiles_.front()));
                queuedTerrainTiles_.pop_front();
                if (++deferred >= queuedTerrainTiles_.size()) {
                    break;
                }
            }
            continue;
        }
        auto failure = terrainPreparationFailures_.find(request.coordinate);
        if (failure != terrainPreparationFailures_.end()) {
            if (failure->second.sourceSignature != request.sourceSignature) {
                terrainPreparationFailures_.erase(failure);
            } else if (std::chrono::steady_clock::now() < failure->second.retryAfter) {
                queuedTerrainTiles_.push_back(std::move(queuedTerrainTiles_.front()));
                queuedTerrainTiles_.pop_front();
                if (++deferred >= queuedTerrainTiles_.size()) {
                    break;
                }
                continue;
            }
        }
        const auto loadedSignature = terrainSourceSignatures_.find(request.coordinate);
        if (tiles_.contains(request.coordinate) &&
            (loadedSignature == terrainSourceSignatures_.end() ||
             loadedSignature->second == request.sourceSignature)) {
            queuedTerrainTiles_.pop_front();
            deferred = 0;
            continue;
        }
        std::future<PreparedTerrainTile> preparation =
            std::async(std::launch::async, [request = std::move(request)]() mutable {
                return prepareTerrainTile(request.coordinates, request.mapRoot, request.tile,
                                          request.omsiRoot, *request.groundTextures,
                                          request.baseGroundTextureIndex,
                                          request.roadConnections.get());
            });
        const auto coordinate = queuedTerrainTiles_.front().coordinate;
        std::string sourceSignature = queuedTerrainTiles_.front().sourceSignature;
        pendingTerrainTiles_.emplace(
            coordinate, PendingTerrainTile{std::move(sourceSignature), std::move(preparation)});
        queuedTerrainTiles_.pop_front();
        ++scheduled;
        deferred = 0;
    }
    return scheduled;
}

std::size_t DiligentMapTileSet::publishReadyTerrainTiles() {
    std::unordered_map<openbus::map::MapTileCoordinate, std::string, TileKeyHash>
        requestedSignatures;
    if (terrainWindow_) {
        requestedSignatures.insert(terrainWindow_->sourceSignatures.begin(),
                                   terrainWindow_->sourceSignatures.end());
    }

    std::size_t published = 0;
    for (auto pending = pendingTerrainTiles_.begin(); pending != pendingTerrainTiles_.end();) {
        if (pending->second.preparation.wait_for(std::chrono::seconds(0)) !=
            std::future_status::ready) {
            ++pending;
            continue;
        }
        const openbus::map::MapTileCoordinate coordinate = pending->first;
        const std::string sourceSignature = pending->second.sourceSignature;
        std::future<PreparedTerrainTile> preparation = std::move(pending->second.preparation);
        pending = pendingTerrainTiles_.erase(pending);
        PreparedTerrainTile prepared;
        try {
            prepared = preparation.get();
        } catch (...) {
            TerrainPreparationFailure& failure = terrainPreparationFailures_[coordinate];
            if (failure.sourceSignature != sourceSignature) {
                failure = TerrainPreparationFailure{sourceSignature};
            }
            failure.attempts = (std::min)(failure.attempts + 1U, 5U);
            const unsigned exponent = (std::min)(failure.attempts - 1U, 3U);
            const auto retryDelay = std::chrono::milliseconds(250U * (1U << exponent));
            failure.retryAfter = std::chrono::steady_clock::now() + retryDelay;
            throw;
        }
        const auto requestedSignature = requestedSignatures.find(coordinate);
        const auto loadedSignature = terrainSourceSignatures_.find(coordinate);
        if (requestedSignature != requestedSignatures.end() &&
            requestedSignature->second == prepared.sourceSignature &&
            (!tiles_.contains(coordinate) ||
             (loadedSignature != terrainSourceSignatures_.end() &&
              loadedSignature->second != prepared.sourceSignature))) {
            addPreparedTerrainTile(std::move(prepared));
            ++published;
        }
    }
    schedulePendingTerrainTiles();
    if (terrainWindow_) {
        updateActiveTiles(terrainWindow_->worldX, terrainWindow_->worldY, terrainWindow_->radius);
    }
    return published;
}

void DiligentMapTileSet::removeTile(int tileX, int tileY) {
    const openbus::map::MapTileCoordinate coordinate{tileX, tileY};
    terrainPreparationFailures_.erase(coordinate);
    const auto found = tiles_.find(coordinate);
    if (found == tiles_.end()) {
        return;
    }
    const DiligentMapTileAsset* removed = found->second.get();
    tiles_.erase(found);
    terrainSourceSignatures_.erase(coordinate);
    terrainPreparationFailures_.erase(coordinate);
    std::erase(activeTiles_, removed);
}

void DiligentMapTileSet::updateActiveTiles(double worldX, double worldY, int radius) {
    std::vector<openbus::map::MapTileCoordinate> loadedCoordinates;
    loadedCoordinates.reserve(tiles_.size());
    for (const auto& [coordinate, asset] : tiles_) {
        (void)asset;
        loadedCoordinates.push_back(coordinate);
    }
    const auto selected = openbus::map::selectMapTilesWithinRadius(
        loadedCoordinates, worldX, worldY, radius, coordinates_.tileSize);
    activeTiles_.clear();
    activeTiles_.reserve(selected.size());
    for (const auto& coordinate : selected) {
        activeTiles_.push_back(tiles_.at(coordinate).get());
    }
}

std::size_t DiligentMapTileSet::removeInactiveTiles() {
    std::unordered_set<const DiligentMapTileAsset*> activeAssets(activeTiles_.begin(),
                                                                 activeTiles_.end());
    std::size_t removed = 0;
    for (auto tile = tiles_.begin(); tile != tiles_.end();) {
        if (activeAssets.contains(tile->second.get())) {
            ++tile;
        } else {
            terrainSourceSignatures_.erase(tile->first);
            tile = tiles_.erase(tile);
            ++removed;
        }
    }
    return removed;
}

std::size_t DiligentMapTileSet::tileCount() const noexcept {
    return tiles_.size();
}

std::span<const DiligentMapTileAsset* const> DiligentMapTileSet::activeTiles() const noexcept {
    return activeTiles_;
}

std::size_t DiligentMapTileSet::drawTerrain(
    DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
    Diligent::ITextureView& fallbackTexture, const Matrix4& viewProjection,
    const DiligentMaterialState* materialOverride, const DiligentLightingParameters& lighting,
    const DiligentMotionParameters* motion, const MotionDataFrame* motionFrame,
    MotionHistory* motionHistory, const DiligentMapWeather* weather) const {
    validateMotionHistoryPair(motionFrame, motionHistory);
    if (weather != nullptr && (!std::isfinite(weather->groundSnow) || weather->groundSnow < 0.0F ||
                               weather->groundSnow > 1.0F)) {
        throw std::invalid_argument("Diligent ground-snow coverage must be a finite fraction");
    }
    std::size_t submitted = 0;
    for (const DiligentMapTileAsset* tile : activeTiles_) {
        const Matrix4 modelViewProjection =
            tile->modelViewProjection(viewProjection, coordinates_.tileSize);
        const std::optional<DiligentMotionParameters> tileMotion =
            motionFrame != nullptr ? std::optional<DiligentMotionParameters>(tile->motionParameters(
                                         *motionFrame, *motionHistory, coordinates_.tileSize))
                                   : std::nullopt;
        const DiligentMotionParameters* selectedMotion =
            tileMotion.has_value() ? &*tileMotion : motion;
        for (const DiligentMapMeshBatch& layer : tile->terrainLayers()) {
            DiligentMaterialState weatherMaterial;
            const DiligentMaterialState* selectedMaterial = materialOverride;
            if (weather != nullptr) {
                weatherMaterial = materialOverride != nullptr ? *materialOverride : layer.material;
                weatherMaterial.snowCoverage = 0.85F * weather->groundSnow;
                selectedMaterial = &weatherMaterial;
            }
            if (layer.draw(renderer, context, fallbackTexture, modelViewProjection,
                           selectedMaterial, nullptr, lighting, selectedMotion)) {
                ++submitted;
            }
        }
    }
    return submitted;
}

std::size_t DiligentMapTileSet::drawWater(
    DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
    Diligent::ITextureView& fallbackTexture, const Matrix4& viewProjection,
    const DiligentMaterialState* materialOverride, const DiligentLightingParameters& lighting,
    const DiligentMotionParameters* motion, const MotionDataFrame* motionFrame,
    MotionHistory* motionHistory) const {
    validateMotionHistoryPair(motionFrame, motionHistory);

    struct WaterTile {
        const DiligentMapTileAsset* tile = nullptr;
        double projectedDepth = 0.0;
    };
    std::vector<WaterTile> visibleWater;
    visibleWater.reserve(activeTiles_.size());
    for (const DiligentMapTileAsset* tile : activeTiles_) {
        if (tile->water() == nullptr) {
            continue;
        }
        const double centerX = (static_cast<double>(tile->tileX()) + 0.5) * coordinates_.tileSize;
        const double centerY = (static_cast<double>(tile->tileY()) + 0.5) * coordinates_.tileSize;
        const double depth =
            projectedSortDepth(viewProjection, {centerX, centerY, tile->water()->meanHeight});
        visibleWater.push_back({tile, depth});
    }
    std::sort(visibleWater.begin(), visibleWater.end(),
              [](const WaterTile& first, const WaterTile& second) {
                  if (first.projectedDepth != second.projectedDepth) {
                      return first.projectedDepth > second.projectedDepth;
                  }
                  return std::pair{first.tile->tileX(), first.tile->tileY()} <
                         std::pair{second.tile->tileX(), second.tile->tileY()};
              });

    std::size_t submitted = 0;
    for (const WaterTile& water : visibleWater) {
        const Matrix4 modelViewProjection =
            water.tile->modelViewProjection(viewProjection, coordinates_.tileSize);
        const std::optional<DiligentMotionParameters> tileMotion =
            motionFrame != nullptr
                ? std::optional<DiligentMotionParameters>(water.tile->motionParameters(
                      *motionFrame, *motionHistory, coordinates_.tileSize))
                : std::nullopt;
        const DiligentMotionParameters* selectedMotion =
            tileMotion.has_value() ? &*tileMotion : motion;
        if (water.tile->water()->draw(renderer, context, fallbackTexture, modelViewProjection,
                                      materialOverride, nullptr, lighting, selectedMotion)) {
            ++submitted;
        }
    }
    return submitted;
}

std::size_t DiligentMapTileSet::drawRoads(
    DiligentMapRoadPass pass, DiligentTexturedMeshRenderer& renderer,
    Diligent::IDeviceContext& context, Diligent::ITextureView& fallbackTexture,
    const Matrix4& viewProjection, const DiligentMaterialState* materialOverride,
    const DiligentLightingParameters& lighting, const DiligentMotionParameters* motion,
    const MotionDataFrame* motionFrame, MotionHistory* motionHistory,
    const DiligentMapWeather* weather) const {
    validateMotionHistoryPair(motionFrame, motionHistory);

    struct RoadSubmission {
        const DiligentMapTileAsset* tile = nullptr;
        const DiligentMapMeshBatch* batch = nullptr;
        double projectedDepth = 0.0;
        std::size_t sourceOrder = 0;
    };
    std::vector<RoadSubmission> submissions;
    std::size_t sourceOrder = 0;
    for (const DiligentMapTileAsset* tile : activeTiles_) {
        for (const DiligentMapMeshBatch& road : tile->roads()) {
            const bool passMatches = (pass == DiligentMapRoadPass::Opaque &&
                                      road.material.alphaMode == DiligentAlphaMode::Opaque) ||
                                     (pass == DiligentMapRoadPass::Cutout &&
                                      road.material.alphaMode == DiligentAlphaMode::Cutout) ||
                                     (pass == DiligentMapRoadPass::Transparent &&
                                      road.material.alphaMode == DiligentAlphaMode::Blend);
            if (!passMatches || !road.mesh) {
                ++sourceOrder;
                continue;
            }
            double projectedDepth = 0.0;
            if (pass == DiligentMapRoadPass::Transparent) {
                const double fallbackX =
                    (static_cast<double>(tile->tileX()) + 0.5) * coordinates_.tileSize;
                const double fallbackY =
                    (static_cast<double>(tile->tileY()) + 0.5) * coordinates_.tileSize;
                const double centerX = road.hasSortCenter ? road.sortCenter[0] : fallbackX;
                const double centerY = road.hasSortCenter ? road.sortCenter[1] : fallbackY;
                const double centerZ = road.hasSortCenter ? road.sortCenter[2] : road.meanHeight;
                projectedDepth = projectedSortDepth(viewProjection, {centerX, centerY, centerZ});
            }
            submissions.push_back({tile, &road, projectedDepth, sourceOrder++});
        }
    }
    if (pass == DiligentMapRoadPass::Transparent) {
        std::sort(
            submissions.begin(), submissions.end(),
            [](const RoadSubmission& first, const RoadSubmission& second) {
                if (first.projectedDepth != second.projectedDepth) {
                    return first.projectedDepth > second.projectedDepth;
                }
                return std::tuple{first.tile->tileX(), first.tile->tileY(), first.sourceOrder} <
                       std::tuple{second.tile->tileX(), second.tile->tileY(), second.sourceOrder};
            });
    }

    std::size_t submitted = 0;
    DiligentMaterialState selectedMaterial;
    for (const RoadSubmission& road : submissions) {
        const DiligentMaterialState* material = materialOverride;
        if (materialOverride != nullptr || weather != nullptr) {
            selectedMaterial =
                materialOverride != nullptr ? *materialOverride : road.batch->material;
            if (weather != nullptr) {
                selectedMaterial = applyDiligentRoadWeather(std::move(selectedMaterial),
                                                            road.batch->weatherClassified,
                                                            road.batch->supportsMoisture, *weather);
            }
            selectedMaterial.alphaMode = road.batch->material.alphaMode;
            material = &selectedMaterial;
        }
        const Matrix4 modelViewProjection =
            road.tile->modelViewProjection(viewProjection, coordinates_.tileSize);
        const std::optional<DiligentMotionParameters> tileMotion =
            motionFrame != nullptr
                ? std::optional<DiligentMotionParameters>(road.tile->motionParameters(
                      *motionFrame, *motionHistory, coordinates_.tileSize))
                : std::nullopt;
        const DiligentMotionParameters* selectedMotion =
            tileMotion.has_value() ? &*tileMotion : motion;
        if (road.batch->draw(renderer, context, fallbackTexture, modelViewProjection, material,
                             nullptr, lighting, selectedMotion)) {
            ++submitted;
        }
    }
    return submitted;
}

} // namespace openbus::rendering
