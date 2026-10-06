#include "MapCollisionBuilder.h"

#include "MapRoadGeometry.h"
#include "MapSplineGeometry.h"
#include "MapSplineProfile.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace openbus::map {
namespace {

constexpr std::size_t MAX_VERTICES_PER_TILE = 1'000'000;
constexpr int SPAWN_TILE_RADIUS = 1;

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

} // namespace

MapRoadCollisionResult buildSpawnRoadCollision(const MapDefinition& map,
                                               std::size_t centerTileIndex,
                                               const std::filesystem::path& omsiRoot) {
    if (centerTileIndex >= map.tiles.size()) {
        throw std::out_of_range("Selected map spawn references an invalid tile");
    }
    const MapTileReference& center = map.tiles[centerTileIndex];
    MapRoadCollisionResult result;
    std::unordered_map<std::string, MapSplineProfile> profiles;
    std::unordered_set<std::string> failedProfiles;

    for (const MapTileReference& tileReference : map.tiles) {
        const long long deltaX = static_cast<long long>(tileReference.x) - center.x;
        const long long deltaY = static_cast<long long>(tileReference.y) - center.y;
        if (std::abs(deltaX) > SPAWN_TILE_RADIUS || std::abs(deltaY) > SPAWN_TILE_RADIUS) {
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
        if (!tileMesh.indices.empty()) {
            result.meshes.push_back(std::move(tileMesh));
        }
    }
    return result;
}

} // namespace openbus::map
