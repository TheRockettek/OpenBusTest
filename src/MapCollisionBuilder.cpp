#include "MapCollisionBuilder.h"

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
        if (std::filesystem::is_regular_file(candidate)) {
            return candidate;
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
        if (std::filesystem::is_regular_file(candidate)) {
            return candidate.lexically_normal();
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
        const auto reference = std::find_if(map.tiles.begin(), map.tiles.end(), [&](const auto& tile) {
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
    const double gridX = std::clamp(localX / OMSI_TILE_SIZE_METERS * grid.intervals,
                                    0.0, static_cast<double>(grid.intervals));
    const double gridY = std::clamp(localY / OMSI_TILE_SIZE_METERS * grid.intervals,
                                    0.0, static_cast<double>(grid.intervals));
    const std::size_t x0 = static_cast<std::size_t>(gridX);
    const std::size_t y0 = static_cast<std::size_t>(gridY);
    const std::size_t x1 = std::min(x0 + 1, grid.intervals);
    const std::size_t y1 = std::min(y0 + 1, grid.intervals);
    const double fx = gridX - x0;
    const double fy = gridY - y0;
    const auto at = [&](std::size_t y, std::size_t x) {
        return static_cast<double>(grid.heights[y * side + x]);
    };
    return std::lerp(std::lerp(at(y0, x0), at(y0, x1), fx),
                     std::lerp(at(y1, x0), at(y1, x1), fx), fy);
}

std::array<double, 3> transformSceneryVertex(const openbus::rendering::ObjPosition& source,
                                              const MapSceneryPose& pose, double worldZ) {
    const double radians = std::numbers::pi / 180.0;
    const double ax = -pose.rotationDegrees[1] * radians;
    const double ay = pose.rotationDegrees[2] * radians;
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

std::vector<std::filesystem::path> readSceneryCollisionMeshes(
    const std::filesystem::path& configPath, const std::filesystem::path& omsiRoot,
    const std::filesystem::path& mapRoot, bool& noCollision, bool& onlyEditor,
    bool& absoluteHeight) {
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

    const std::filesystem::path modelConfigPath =
        resolveSceneryObjectModelConfigPath(scenery);
    if (modelConfigPath.empty() || !std::filesystem::is_regular_file(modelConfigPath)) {
        return meshes;
    }
    const std::filesystem::path modelRoot = scenery.modelPath.empty()
                                                ? objectRoot / "model"
                                                : modelConfigPath.parent_path();
    openbus::scripting::SceneryObject variables;
    const ModelConfig model = loadModelConfig(modelConfigPath, modelRoot,
                                              ModelConfigKind::SceneryObject, variables);
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

MapRoadCollisionResult buildMapRoadCollision(const MapDefinition& map,
                                             const std::filesystem::path& omsiRoot) {
    MapRoadCollisionResult result;
    std::unordered_map<std::string, MapSplineProfile> profiles;
    std::unordered_set<std::string> failedProfiles;
    std::unordered_map<std::uint64_t, TerrainGrid> terrainCache;

    for (const MapTileReference& tileReference : map.tiles) {
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
            if (profile.editorOnly) {
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
            for (const MapSplineProfileSection& section : profile.sections) {
                // The renderer only submits strips with a valid texture reference. Missing image
                // files still render untextured, so they do not remove collision geometry.
                if (section.points.size() < 2 || section.textureIndex < 0 ||
                    static_cast<std::size_t>(section.textureIndex) >= profile.textures.size()) {
                    continue;
                }
                MapRoadSectionGeometry sectionGeometry;
                try {
                    sectionGeometry = buildMapRoadSectionGeometry(spline, centerline, section);
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

        for (const MapSceneryPlacement& object : tile.sceneryObjects) {
            if (!object.transformValid) {
                continue;
            }
            const std::optional<MapSceneryPose> pose =
                placeMapSceneryObject(object, tileReference.x, tileReference.y);
            if (!pose) {
                continue;
            }
            const std::filesystem::path configPath = resolveAsset(
                normalizedAssetPath(object.assetPath), map.rootPath, omsiRoot, map.rootPath);
            if (configPath.empty()) {
                continue;
            }
            bool noCollision = false;
            bool onlyEditor = false;
            bool absoluteHeight = false;
            const std::vector<std::filesystem::path> collisionPaths = readSceneryCollisionMeshes(
                configPath, omsiRoot, map.rootPath, noCollision, onlyEditor, absoluteHeight);
            if (noCollision || onlyEditor || collisionPaths.empty()) {
                continue;
            }
            const double ground = absoluteHeight
                                      ? 0.0
                                      : sampleTerrainHeight(map, pose->x, pose->y, terrainCache);
            const double worldZ = mapSceneryWorldHeight(pose->z, ground, absoluteHeight);
            bool objectAdded = false;
            for (const std::filesystem::path& collisionPath : collisionPaths) {
                const std::shared_ptr<openbus::rendering::ParsedObj> parsed =
                    openbus::rendering::O3DLoader::parse(collisionPath);
                if (!parsed || parsed->triangles.empty()) {
                    ++result.sceneryCollisionMeshesMissing;
                    continue;
                }
                const std::size_t currentVertexCount = tileMesh.vertices.size() / 3;
                if (parsed->positions.size() > MAX_VERTICES_PER_TILE - currentVertexCount ||
                    currentVertexCount + parsed->positions.size() >
                        static_cast<std::size_t>(std::numeric_limits<int>::max())) {
                    ++result.skippedSplines;
                    continue;
                }
                const int baseVertex = static_cast<int>(currentVertexCount);
                for (const openbus::rendering::ObjPosition& vertex : parsed->positions) {
                    const auto world = transformSceneryVertex(vertex, *pose, worldZ);
                    tileMesh.vertices.insert(tileMesh.vertices.end(),
                                             {world[0], world[1], world[2]});
                }
                for (const openbus::rendering::ObjTriangle& triangle : parsed->triangles) {
                    // O3DLoader maps the file's right/up/forward basis to
                    // east/north/up with (x, y, z) -> (x, z, y), which flips
                    // handedness. Reverse each face so ODE receives the same
                    // upward-facing surface orientation as the map renderer.
                    for (const std::size_t corner : {0U, 2U, 1U}) {
                        const openbus::rendering::ObjIndex& index = triangle.indices[corner];
                        if (index.position <= 0 ||
                            static_cast<std::size_t>(index.position) > parsed->positions.size()) {
                            continue;
                        }
                        tileMesh.indices.push_back(baseVertex + index.position - 1);
                    }
                    objectAdded = true;
                }
            }
            if (objectAdded) {
                ++result.sceneryCollisionObjectsAdded;
            }
        }
        if (!tileMesh.indices.empty()) {
            result.meshes.push_back(std::move(tileMesh));
        }
    }
    return result;
}

} // namespace openbus::map
