#include "MapCollisionBuilder.h"

#include "AssetPath.h"
#include "MapSceneryPlacement.h"
#include "MapRoadGeometry.h"
#include "MapSplineGeometry.h"
#include "MapSplineProfile.h"
#include "ModelConfigLoader.h"
#include "O3DLoader.h"
#include "SceneryObjectConfigLoader.h"
#include "Variables.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace openbus::map {
namespace {

constexpr std::size_t MAX_VERTICES_PER_TILE = 1'000'000;
std::filesystem::path normalizedAssetPath(const std::string& source) {
    std::string normalized = source;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    return std::filesystem::path(normalized);
}

std::filesystem::path resolveProfile(const MapDefinition& map,
                                     const std::filesystem::path& omsiRoot,
                                     const std::string& assetPath) {
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
    return {};
}

std::filesystem::path resolveAsset(const std::filesystem::path& relative,
                                   const std::filesystem::path& preferredRoot,
                                   const std::filesystem::path& omsiRoot,
                                   const std::filesystem::path& mapRoot) {
    if (relative.is_absolute() && std::filesystem::is_regular_file(relative)) {
        return relative;
    }
    for (const std::filesystem::path& candidate :
         {preferredRoot / relative, omsiRoot / relative, mapRoot / relative}) {
        const std::filesystem::path resolved =
            openbus::assets::resolveCaseInsensitivePath(candidate);
        if (std::filesystem::is_regular_file(resolved)) {
            return resolved.lexically_normal();
        }
    }
    return {};
}

std::uint64_t tileKey(int x, int y) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
           static_cast<std::uint32_t>(y);
}

double sampleTerrainHeight(const MapDefinition& map, double worldX, double worldY,
                           std::unordered_map<std::uint64_t, TerrainGrid>& terrainCache) {
    const int tileX = static_cast<int>(std::floor(worldX / OMSI_TILE_SIZE_METERS));
    const int tileY = static_cast<int>(std::floor(worldY / OMSI_TILE_SIZE_METERS));
    const std::uint64_t key = tileKey(tileX, tileY);
    auto found = terrainCache.find(key);
    if (found == terrainCache.end()) {
        const auto reference =
            std::find_if(map.tiles.begin(), map.tiles.end(), [&](const auto& tile) {
                return tile.x == tileX && tile.y == tileY && tile.hasTerrainFile;
            });
        if (reference == map.tiles.end()) {
            return 0.0;
        }
        try {
            found = terrainCache.emplace(key, loadTerrainGrid(reference->terrainPath)).first;
        } catch (const std::exception&) {
            return 0.0;
        }
    }
    const TerrainGrid& grid = found->second;
    if (grid.intervals == 0 || grid.heights.size() != (grid.intervals + 1) * (grid.intervals + 1)) {
        return 0.0;
    }
    const std::size_t side = grid.intervals + 1;
    const double localX = worldX - static_cast<double>(tileX) * OMSI_TILE_SIZE_METERS;
    const double localY = worldY - static_cast<double>(tileY) * OMSI_TILE_SIZE_METERS;
    const double gridX = std::clamp(localX / OMSI_TILE_SIZE_METERS * grid.intervals, 0.0,
                                    static_cast<double>(grid.intervals));
    const double gridY = std::clamp(localY / OMSI_TILE_SIZE_METERS * grid.intervals, 0.0,
                                    static_cast<double>(grid.intervals));
    const std::size_t x0 = static_cast<std::size_t>(gridX);
    const std::size_t y0 = static_cast<std::size_t>(gridY);
    const std::size_t x1 = std::min(x0 + 1, grid.intervals);
    const std::size_t y1 = std::min(y0 + 1, grid.intervals);
    const double fx = gridX - x0;
    const double fy = gridY - y0;
    const auto at = [&](std::size_t y, std::size_t x) {
        return static_cast<double>(grid.heights[y * side + x]);
    };
    return std::lerp(std::lerp(at(y0, x0), at(y0, x1), fx), std::lerp(at(y1, x0), at(y1, x1), fx),
                     fy);
}

std::array<double, 3> transformSceneryVertex(const openbus::rendering::ObjPosition& source,
                                             const MapSceneryPose& pose, double worldZ) {
    const double radians = std::numbers::pi / 180.0;
    const double ax = -pose.rotationDegrees[1] * radians;
    const double ay = -pose.rotationDegrees[2] * radians;
    const double az = -pose.rotationDegrees[0] * radians;
    const double x1 = source.x;
    const double y1 = source.y * std::cos(ax) - source.z * std::sin(ax);
    const double z1 = source.y * std::sin(ax) + source.z * std::cos(ax);
    const double x2 = x1 * std::cos(ay) + z1 * std::sin(ay);
    const double y2 = y1;
    const double z2 = -x1 * std::sin(ay) + z1 * std::cos(ay);
    return {pose.x + x2 * std::cos(az) - y2 * std::sin(az),
            pose.y + x2 * std::sin(az) + y2 * std::cos(az), worldZ + z2};
}

std::vector<std::array<double, 3>> clipPolygonToTile(std::vector<std::array<double, 3>> polygon,
                                                     int tileX, int tileY) {
    const double minX = static_cast<double>(tileX) * OMSI_TILE_SIZE_METERS;
    const double minY = static_cast<double>(tileY) * OMSI_TILE_SIZE_METERS;
    const double maxX = minX + OMSI_TILE_SIZE_METERS;
    const double maxY = minY + OMSI_TILE_SIZE_METERS;
    const auto clip = [](const std::vector<std::array<double, 3>>& input, std::size_t axis,
                         double boundary, bool keepGreater) {
        std::vector<std::array<double, 3>> output;
        if (input.empty()) {
            return output;
        }
        const auto inside = [=](const std::array<double, 3>& point) {
            return keepGreater ? point[axis] >= boundary : point[axis] <= boundary;
        };
        auto previous = input.back();
        bool previousInside = inside(previous);
        for (const auto& current : input) {
            const bool currentInside = inside(current);
            if (currentInside != previousInside) {
                const double denominator = current[axis] - previous[axis];
                if (std::abs(denominator) > 1.0e-12) {
                    const double amount = (boundary - previous[axis]) / denominator;
                    std::array<double, 3> intersection{};
                    for (std::size_t component = 0; component < 3; ++component) {
                        intersection[component] =
                            std::lerp(previous[component], current[component], amount);
                    }
                    intersection[axis] = boundary;
                    output.push_back(intersection);
                }
            }
            if (currentInside) {
                output.push_back(current);
            }
            previous = current;
            previousInside = currentInside;
        }
        return output;
    };
    polygon = clip(polygon, 0, minX, true);
    polygon = clip(polygon, 0, maxX, false);
    polygon = clip(polygon, 1, minY, true);
    return clip(polygon, 1, maxY, false);
}

void appendSceneryCollisionForTile(StaticCollisionMesh& tileMesh,
                                   const MapSceneryCollisionIndex& index, int tileX, int tileY,
                                   MapRoadCollisionResult& result) {
    const auto found = index.objectsByTile.find(tileKey(tileX, tileY));
    if (found == index.objectsByTile.end()) {
        return;
    }
    bool objectAdded = false;
    const double minX = static_cast<double>(tileX) * OMSI_TILE_SIZE_METERS;
    const double minY = static_cast<double>(tileY) * OMSI_TILE_SIZE_METERS;
    const double maxX = minX + OMSI_TILE_SIZE_METERS;
    const double maxY = minY + OMSI_TILE_SIZE_METERS;
    for (const std::size_t objectIndex : found->second) {
        if (objectIndex >= index.objects.size()) {
            continue;
        }
        const MapSceneryCollisionObject& object = index.objects[objectIndex];
        for (const auto& parsed : object.meshes) {
            if (!parsed) {
                continue;
            }
            std::vector<std::array<double, 3>> worldPositions;
            worldPositions.reserve(parsed->positions.size());
            for (const auto& vertex : parsed->positions) {
                worldPositions.push_back(
                    transformSceneryVertex(vertex, object.pose, object.worldZ));
            }
            const bool meshFitsTile =
                std::all_of(worldPositions.begin(), worldPositions.end(), [&](const auto& point) {
                    return point[0] >= minX && point[0] <= maxX && point[1] >= minY &&
                           point[1] <= maxY;
                });
            if (meshFitsTile) {
                const std::size_t currentVertexCount = tileMesh.vertices.size() / 3;
                if (worldPositions.size() > MAX_VERTICES_PER_TILE - currentVertexCount ||
                    currentVertexCount + worldPositions.size() >
                        static_cast<std::size_t>(std::numeric_limits<int>::max())) {
                    ++result.sceneryCollisionMeshesMissing;
                    continue;
                }
                const int baseVertex = static_cast<int>(currentVertexCount);
                for (const auto& point : worldPositions) {
                    tileMesh.vertices.insert(tileMesh.vertices.end(),
                                             {point[0], point[1], point[2]});
                }
                for (const auto& triangle : parsed->triangles) {
                    for (const std::size_t corner : {0U, 2U, 1U}) {
                        const auto& vertex = triangle.indices[corner];
                        if (vertex.position > 0 &&
                            static_cast<std::size_t>(vertex.position) <= worldPositions.size()) {
                            tileMesh.indices.push_back(baseVertex + vertex.position - 1);
                        }
                    }
                }
                objectAdded = true;
                continue;
            }
            for (const auto& triangle : parsed->triangles) {
                std::vector<std::array<double, 3>> polygon;
                polygon.reserve(3);
                for (const std::size_t corner : {0U, 2U, 1U}) {
                    const auto& vertex = triangle.indices[corner];
                    if (vertex.position <= 0 ||
                        static_cast<std::size_t>(vertex.position) > worldPositions.size()) {
                        polygon.clear();
                        break;
                    }
                    polygon.push_back(
                        worldPositions[static_cast<std::size_t>(vertex.position - 1)]);
                }
                polygon = clipPolygonToTile(std::move(polygon), tileX, tileY);
                if (polygon.size() < 3) {
                    continue;
                }
                for (std::size_t corner = 1; corner + 1 < polygon.size(); ++corner) {
                    const std::size_t currentVertexCount = tileMesh.vertices.size() / 3;
                    if (currentVertexCount + 3 > MAX_VERTICES_PER_TILE ||
                        currentVertexCount + 3 >
                            static_cast<std::size_t>(std::numeric_limits<int>::max())) {
                        ++result.sceneryCollisionMeshesMissing;
                        break;
                    }
                    const int baseVertex = static_cast<int>(currentVertexCount);
                    for (const auto& point : {polygon[0], polygon[corner], polygon[corner + 1]}) {
                        tileMesh.vertices.insert(tileMesh.vertices.end(),
                                                 {point[0], point[1], point[2]});
                    }
                    tileMesh.indices.insert(tileMesh.indices.end(),
                                            {baseVertex, baseVertex + 1, baseVertex + 2});
                    objectAdded = true;
                }
            }
        }
        if (objectAdded) {
            ++result.sceneryCollisionObjectsAdded;
            objectAdded = false;
        }
    }
}

std::vector<std::filesystem::path>
readSceneryCollisionMeshes(const std::filesystem::path& configPath,
                           const std::filesystem::path& omsiRoot,
                           const std::filesystem::path& mapRoot, bool& noCollision,
                           bool& onlyEditor, bool& absoluteHeight) {
    std::vector<std::filesystem::path> meshes;
    const SceneryObjectConfig scenery = loadSceneryObjectFile(configPath);
    noCollision = scenery.noCollision;
    onlyEditor = scenery.onlyEditor;
    absoluteHeight = scenery.absoluteHeight;
    const std::filesystem::path objectRoot = configPath.parent_path();
    if (!scenery.collisionMesh.empty()) {
        const std::filesystem::path collisionRelative =
            normalizedAssetPath(scenery.collisionMesh.generic_string());
        std::filesystem::path direct =
            resolveAsset(collisionRelative, objectRoot / "model", omsiRoot, mapRoot);
        if (direct.empty()) {
            direct = resolveAsset(collisionRelative, objectRoot, omsiRoot, mapRoot);
        }
        if (!direct.empty()) {
            meshes.push_back(direct);
        }
    }

    const std::filesystem::path modelConfigPath = resolveSceneryObjectModelConfigPath(scenery);
    if (modelConfigPath.empty() || !std::filesystem::is_regular_file(modelConfigPath)) {
        return meshes;
    }
    const std::filesystem::path modelRoot =
        scenery.modelPath.empty() ? objectRoot / "model" : modelConfigPath.parent_path();
    openbus::scripting::SceneryObject variables;
    const ModelConfig model =
        loadModelConfig(modelConfigPath, modelRoot, ModelConfigKind::SceneryObject, variables);
    absoluteHeight = absoluteHeight || model.absoluteHeight;
    for (const ModelCollisionMesh& collision : model.collisionMeshes) {
        if (collision.hasPart && collision.partIndex < model.parts.size() &&
            (model.parts[collision.partIndex].noCollision ||
             model.parts[collision.partIndex].isShadow)) {
            continue;
        }
        std::filesystem::path resolved = collision.resolvedPath;
        if (!std::filesystem::is_regular_file(resolved)) {
            resolved = resolveAsset(normalizedAssetPath(collision.sourcePath.generic_string()),
                                    modelRoot, omsiRoot, mapRoot);
        }
        if (!resolved.empty()) {
            resolved = resolved.lexically_normal();
            if (std::none_of(meshes.begin(), meshes.end(), [&](const auto& existing) {
                    return existing.lexically_normal() == resolved;
                })) {
                meshes.push_back(std::move(resolved));
            }
        }
    }
    return meshes;
}

} // namespace

std::shared_ptr<const MapSceneryCollisionIndex>
buildMapSceneryCollisionIndex(const MapDefinition& map, const std::filesystem::path& omsiRoot) {
    struct ConfigCollision {
        std::vector<std::filesystem::path> paths;
        bool noCollision = false;
        bool onlyEditor = false;
        bool absoluteHeight = false;
    };
    auto index = std::make_shared<MapSceneryCollisionIndex>();
    std::unordered_map<std::string, ConfigCollision> configCache;
    std::unordered_map<std::string, std::shared_ptr<openbus::rendering::ParsedObj>> meshCache;
    std::unordered_map<std::string, std::array<double, 6>> meshLocalBoundsCache;
    std::unordered_map<std::uint64_t, TerrainGrid> terrainCache;
    std::unordered_set<std::uint64_t> mapTileKeys;
    mapTileKeys.reserve(map.tiles.size());
    for (const MapTileReference& tile : map.tiles) {
        mapTileKeys.insert(tileKey(tile.x, tile.y));
    }

    for (const MapTileReference& tileReference : map.tiles) {
        MapTileData tile;
        try {
            tile = loadMapTile(tileReference);
        } catch (const std::exception&) {
            continue;
        }
        for (const MapSceneryPlacement& placement : tile.sceneryObjects) {
            if (!placement.transformValid) {
                continue;
            }
            const std::optional<MapSceneryPose> pose =
                placeMapSceneryObject(placement, tileReference.x, tileReference.y);
            if (!pose) {
                continue;
            }
            const std::filesystem::path configPath = resolveAsset(
                normalizedAssetPath(placement.assetPath), map.rootPath, omsiRoot, map.rootPath);
            if (configPath.empty()) {
                continue;
            }
            const std::string configKey = configPath.lexically_normal().generic_string();
            auto config = configCache.find(configKey);
            if (config == configCache.end()) {
                ConfigCollision collision;
                collision.paths = readSceneryCollisionMeshes(
                    configPath, omsiRoot, map.rootPath, collision.noCollision, collision.onlyEditor,
                    collision.absoluteHeight);
                config = configCache.emplace(configKey, std::move(collision)).first;
            }
            const ConfigCollision& collision = config->second;
            if (collision.noCollision || collision.onlyEditor || collision.paths.empty()) {
                continue;
            }

            MapSceneryCollisionObject object;
            object.pose = *pose;
            const double ground = collision.absoluteHeight
                                      ? 0.0
                                      : sampleTerrainHeight(map, pose->x, pose->y, terrainCache);
            object.worldZ = mapSceneryWorldHeight(pose->z, ground, collision.absoluteHeight);
            object.minX = object.minY = std::numeric_limits<double>::max();
            object.maxX = object.maxY = std::numeric_limits<double>::lowest();
            for (const std::filesystem::path& meshPath : collision.paths) {
                const std::string meshKey = meshPath.lexically_normal().generic_string();
                auto mesh = meshCache.find(meshKey);
                if (mesh == meshCache.end()) {
                    try {
                        mesh = meshCache
                                   .emplace(meshKey, openbus::rendering::O3DLoader::parse(meshPath))
                                   .first;
                    } catch (const std::exception&) {
                        mesh = meshCache.emplace(meshKey, nullptr).first;
                    }
                }
                if (!mesh->second || mesh->second->triangles.empty()) {
                    continue;
                }
                object.meshes.push_back(mesh->second);
                auto bounds = meshLocalBoundsCache.find(meshKey);
                if (bounds == meshLocalBoundsCache.end()) {
                    std::array<double, 6> localBounds = {std::numeric_limits<double>::max(),
                                                         std::numeric_limits<double>::max(),
                                                         std::numeric_limits<double>::max(),
                                                         std::numeric_limits<double>::lowest(),
                                                         std::numeric_limits<double>::lowest(),
                                                         std::numeric_limits<double>::lowest()};
                    for (const auto& vertex : mesh->second->positions) {
                        localBounds[0] = std::min(localBounds[0], vertex.x);
                        localBounds[1] = std::min(localBounds[1], vertex.y);
                        localBounds[2] = std::min(localBounds[2], vertex.z);
                        localBounds[3] = std::max(localBounds[3], vertex.x);
                        localBounds[4] = std::max(localBounds[4], vertex.y);
                        localBounds[5] = std::max(localBounds[5], vertex.z);
                    }
                    bounds = meshLocalBoundsCache.emplace(meshKey, localBounds).first;
                }
                const auto& localBounds = bounds->second;
                for (int corner = 0; corner < 8; ++corner) {
                    const openbus::rendering::ObjPosition vertex{
                        (corner & 1) == 0 ? localBounds[0] : localBounds[3],
                        (corner & 2) == 0 ? localBounds[1] : localBounds[4],
                        (corner & 4) == 0 ? localBounds[2] : localBounds[5]};
                    const auto world = transformSceneryVertex(vertex, object.pose, object.worldZ);
                    object.minX = std::min(object.minX, world[0]);
                    object.minY = std::min(object.minY, world[1]);
                    object.maxX = std::max(object.maxX, world[0]);
                    object.maxY = std::max(object.maxY, world[1]);
                }
            }
            if (object.meshes.empty()) {
                continue;
            }
            const std::size_t objectIndex = index->objects.size();
            index->objects.push_back(std::move(object));
            const MapSceneryCollisionObject& stored = index->objects.back();
            const int minTileX = static_cast<int>(std::floor(stored.minX / OMSI_TILE_SIZE_METERS));
            const int minTileY = static_cast<int>(std::floor(stored.minY / OMSI_TILE_SIZE_METERS));
            const int maxTileX = static_cast<int>(std::floor(stored.maxX / OMSI_TILE_SIZE_METERS));
            const int maxTileY = static_cast<int>(std::floor(stored.maxY / OMSI_TILE_SIZE_METERS));
            for (int tileY = minTileY; tileY <= maxTileY; ++tileY) {
                for (int tileX = minTileX; tileX <= maxTileX; ++tileX) {
                    const std::uint64_t key = tileKey(tileX, tileY);
                    if (mapTileKeys.contains(key)) {
                        index->objectsByTile[key].push_back(objectIndex);
                    }
                }
            }
        }
    }
    return index;
}

static MapRoadCollisionResult buildMapRoadCollisionImpl(const MapDefinition& map,
                                                        const std::filesystem::path& omsiRoot,
                                                        const MapTileReference* selectedTile,
                                                        const MapRoadConnectionIndex& connections,
                                                        const MapSceneryCollisionIndex* scenery) {
    MapRoadCollisionResult result;
    std::unordered_map<std::string, MapSplineProfile> profiles;
    std::unordered_set<std::string> failedProfiles;

    for (const MapTileReference& tileReference : map.tiles) {
        if (selectedTile != nullptr &&
            (tileReference.x != selectedTile->x || tileReference.y != selectedTile->y)) {
            continue;
        }
        ++result.tilesVisited;
        const MapTileData tile = loadMapTile(tileReference);
        StaticCollisionMesh tileMesh;

        for (const MapSplinePlacement& spline : tile.splines) {
            if (!spline.geometryValid) {
                ++result.skippedSplines;
                continue;
            }
            const std::filesystem::path profilePath =
                resolveProfile(map, omsiRoot, spline.assetPath);
            if (profilePath.empty()) {
                ++result.skippedProfiles;
                continue;
            }
            const std::string profileKey = profilePath.lexically_normal().generic_string();
            if (failedProfiles.contains(profileKey)) {
                ++result.skippedProfiles;
                continue;
            }
            auto profileIt = profiles.find(profileKey);
            if (profileIt == profiles.end()) {
                try {
                    profileIt =
                        profiles.emplace(profileKey, loadMapSplineProfile(profilePath)).first;
                } catch (const std::exception&) {
                    failedProfiles.insert(profileKey);
                    ++result.skippedProfiles;
                    continue;
                }
            }
            const MapSplineProfile& profile = profileIt->second;
            if (profile.editorOnly || profile.noCollision) {
                continue;
            }

            const double startX =
                static_cast<double>(tileReference.x) * OMSI_TILE_SIZE_METERS + spline.localX;
            const double startY =
                static_cast<double>(tileReference.y) * OMSI_TILE_SIZE_METERS + spline.localY;
            const std::vector<MapSplineSample> centerline = tessellateMapSpline(
                startX, startY, spline.rotationDegrees, spline.length, spline.radius);
            if (centerline.size() < 2) {
                ++result.skippedSplines;
                continue;
            }
            const std::optional<double> cantHalfWidth = mapSplineProfileCantHalfWidth(profile);
            for (const MapSplineProfileSection& section : profile.sections) {
                // The renderer only submits strips with a valid texture reference. Missing image
                // files still render untextured, so they do not remove collision geometry.
                if (section.points.size() < 2 || section.textureIndex < 0 ||
                    static_cast<std::size_t>(section.textureIndex) >= profile.textures.size()) {
                    continue;
                }
                MapRoadSectionGeometry sectionGeometry;
                try {
                    sectionGeometry = buildMapRoadSectionGeometry(
                        spline, centerline, section,
                        connections.joinsFor(spline, tileReference.x, tileReference.y),
                        cantHalfWidth);
                } catch (const std::exception&) {
                    ++result.skippedSplines;
                    continue;
                }
                if (sectionGeometry.vertices.empty() || sectionGeometry.indices.empty()) {
                    continue;
                }
                const std::size_t currentVertexCount = tileMesh.vertices.size() / 3;
                if (sectionGeometry.vertices.size() > MAX_VERTICES_PER_TILE - currentVertexCount ||
                    currentVertexCount + sectionGeometry.vertices.size() >
                        static_cast<std::size_t>(std::numeric_limits<int>::max())) {
                    ++result.skippedSplines;
                    continue;
                }
                const int baseVertex = static_cast<int>(currentVertexCount);
                for (const MapRoadVertex& vertex : sectionGeometry.vertices) {
                    tileMesh.vertices.insert(tileMesh.vertices.end(),
                                             {vertex.x, vertex.y, vertex.z});
                }
                // The renderer's road-strip winding is clockwise from above. ODE trimesh
                // contacts need the opposite orientation so surface normals point upward.
                for (std::size_t index = 0; index < sectionGeometry.indices.size(); index += 3) {
                    tileMesh.indices.push_back(baseVertex + sectionGeometry.indices[index]);
                    tileMesh.indices.push_back(baseVertex + sectionGeometry.indices[index + 2]);
                    tileMesh.indices.push_back(baseVertex + sectionGeometry.indices[index + 1]);
                }
                ++result.splineSectionsAdded;
            }
        }

        if (scenery != nullptr) {
            appendSceneryCollisionForTile(tileMesh, *scenery, tileReference.x, tileReference.y,
                                          result);
        }
        if (!tileMesh.indices.empty()) {
            result.meshes.push_back(std::move(tileMesh));
        }
    }
    return result;
}

MapRoadCollisionResult buildMapRoadCollision(const MapDefinition& map,
                                             const std::filesystem::path& omsiRoot) {
    const std::shared_ptr<const MapSceneryCollisionIndex> scenery =
        buildMapSceneryCollisionIndex(map, omsiRoot);
    const auto connections = loadMapRoadConnections(map, omsiRoot);
    return buildMapRoadCollisionImpl(map, omsiRoot, nullptr, connections, scenery.get());
}

MapRoadCollisionResult buildMapRoadCollisionTile(const MapDefinition& map,
                                                 const std::filesystem::path& omsiRoot, int tileX,
                                                 int tileY,
                                                 const MapRoadConnectionIndex* connections,
                                                 const MapSceneryCollisionIndex* scenery) {
    const auto tile = std::find_if(map.tiles.begin(), map.tiles.end(), [&](const auto& reference) {
        return reference.x == tileX && reference.y == tileY;
    });
    if (tile == map.tiles.end()) {
        return {};
    }
    const auto ownedScenery = scenery == nullptr
                                  ? buildMapSceneryCollisionIndex(map, omsiRoot)
                                  : std::shared_ptr<const MapSceneryCollisionIndex>{};
    const auto ownedConnections =
        connections == nullptr
            ? std::optional<MapRoadConnectionIndex>(loadMapRoadConnections(map, omsiRoot))
            : std::nullopt;
    return buildMapRoadCollisionImpl(map, omsiRoot, &*tile,
                                     connections == nullptr ? *ownedConnections : *connections,
                                     scenery == nullptr ? ownedScenery.get() : scenery);
}

MapRoadCollisionResult buildMapSplineCollisionTile(const MapDefinition& map,
                                                   const std::filesystem::path& omsiRoot, int tileX,
                                                   int tileY,
                                                   const MapRoadConnectionIndex* connections) {
    const auto tile = std::find_if(map.tiles.begin(), map.tiles.end(), [&](const auto& reference) {
        return reference.x == tileX && reference.y == tileY;
    });
    if (tile == map.tiles.end()) {
        return {};
    }
    const auto ownedConnections =
        connections == nullptr
            ? std::optional<MapRoadConnectionIndex>(loadMapRoadConnections(map, omsiRoot))
            : std::nullopt;
    return buildMapRoadCollisionImpl(
        map, omsiRoot, &*tile, connections == nullptr ? *ownedConnections : *connections, nullptr);
}

MapRoadConnectionIndex loadMapRoadConnections(const MapDefinition& map,
                                              const std::filesystem::path& omsiRoot) {
    MapRoadConnectionIndex connections;
    std::unordered_map<std::string, std::optional<double>> cantHalfWidths;
    for (const auto& reference : map.tiles) {
        try {
            const auto tile = loadMapTile(reference);
            for (const auto& spline : tile.splines) {
                const std::string assetKey =
                    normalizedAssetPath(spline.assetPath).lexically_normal().generic_string();
                const auto [width, inserted] = cantHalfWidths.try_emplace(assetKey, std::nullopt);
                if (inserted) {
                    // Cache unavailable widths too: a bad asset must not be reparsed for
                    // every placement, nor prevent later splines in this tile registering.
                    try {
                        const auto profilePath = resolveProfile(map, omsiRoot, spline.assetPath);
                        if (!profilePath.empty()) {
                            width->second =
                                mapSplineProfileCantHalfWidth(loadMapSplineProfile(profilePath));
                        }
                    } catch (const std::exception&) {
                        // Missing/malformed profiles retain legacy unbounded cant metadata.
                    }
                }
                connections.addSpline(spline, reference.x, reference.y, width->second);
            }
        } catch (const std::exception&) {
            // An unrelated missing/malformed tile must not prevent a selected tile loading.
            // Its own collision build remains strict and reports the original parse failure.
        }
    }
    connections.finalize();
    return connections;
}

} // namespace openbus::map
