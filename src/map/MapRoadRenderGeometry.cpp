#include "map/MapRoadRenderGeometry.h"

#include "core/AssetPath.h"
#include "core/Utf8Path.h"

#include <algorithm>
#include <array>
#include <stdexcept>

namespace openbus::map {
namespace {

std::filesystem::path normalizedAssetPath(const std::string& value) {
    std::string normalized = value;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    return openbus::pathFromUtf8(normalized);
}

std::filesystem::path findSplineTexture(const std::filesystem::path& profilePath,
                                        const std::filesystem::path& omsiRoot,
                                        const std::string& textureName) {
    const std::filesystem::path relative = normalizedAssetPath(textureName);
    const std::filesystem::path profileRoot = profilePath.parent_path();
    const std::array<std::filesystem::path, 6> candidates = {
        profileRoot / "texture" / relative,
        profileRoot / relative,
        openbus::assets::textureSubdirectoryPath(profileRoot, relative),
        omsiRoot / "Texture" / relative,
        omsiRoot / "texture" / relative,
        omsiRoot / relative};
    for (const std::filesystem::path& candidate : candidates) {
        if (!candidate.empty() && std::filesystem::is_regular_file(candidate)) {
            return candidate;
        }
    }
    return {};
}

} // namespace

std::map<std::string, RoadMeshGeometry>
prepareMapRoadRenderBatches(int tileX, int tileY, const MapCoordinates& coordinates,
                            const std::filesystem::path& omsiRoot,
                            std::span<const MapRoadSplineRenderInput> splines) {
    std::map<std::string, RoadMeshGeometry> batches;
    for (const MapRoadSplineRenderInput& input : splines) {
        const MapSplinePlacement& spline = input.spline;
        if (!spline.geometryValid || input.profile == nullptr) {
            continue;
        }
        const MapSplineProfile& profile = *input.profile;
        if (profile.editorOnly || profile.sections.empty()) {
            continue;
        }
        const double startX = static_cast<double>(tileX) * coordinates.tileSize + spline.localX;
        const double startY = static_cast<double>(tileY) * coordinates.tileSize + spline.localY;
        const std::vector<MapSplineSample> centerline = tessellateMapSpline(
            startX, startY, spline.rotationDegrees, spline.length, spline.radius);
        if (centerline.size() < 2) {
            continue;
        }
        const std::optional<double> cantHalfWidth = mapSplineProfileCantHalfWidth(profile);
        const std::string profileKey = openbus::pathToUtf8(input.profilePath.lexically_normal());
        for (const MapSplineProfileSection& section : profile.sections) {
            if (section.points.size() < 2 || section.textureIndex < 0 ||
                static_cast<std::size_t>(section.textureIndex) >= profile.textures.size()) {
                continue;
            }
            const std::string& textureName =
                profile.textures[static_cast<std::size_t>(section.textureIndex)];
            const std::filesystem::path texturePath =
                findSplineTexture(input.profilePath, omsiRoot, textureName);
            const int alphaMode =
                static_cast<std::size_t>(section.textureIndex) < profile.textureAlphaModes.size()
                    ? profile.textureAlphaModes[static_cast<std::size_t>(section.textureIndex)]
                    : 0;
            const bool textured = !texturePath.empty();
            const std::string batchKey =
                std::to_string(tileX) + "," + std::to_string(tileY) + "|" +
                (textured
                     ? openbus::pathToUtf8(texturePath.lexically_normal())
                     : "untextured:" + profileKey + "#" + std::to_string(section.textureIndex)) +
                "|alpha:" + std::to_string(alphaMode);
            RoadMeshGeometry& road = batches[batchKey];
            if (road.vertices.empty()) {
                road.texturePath = texturePath;
                road.textured = textured;
                road.tileX = tileX;
                road.tileY = tileY;
                road.alphaMode = alphaMode;
            }
            appendMapRoadProfile(road, spline, centerline, section, input.joins, cantHalfWidth,
                                 coordinates);
        }
    }
    return batches;
}

void appendMapRoadProfile(RoadMeshGeometry& road, const MapSplinePlacement& spline,
                          const std::vector<MapSplineSample>& centerline,
                          const MapSplineProfileSection& section, const MapRoadSplineJoins& joins,
                          std::optional<double> cantHalfWidth, const MapCoordinates& coordinates) {
    const MapRoadSectionGeometry geometry =
        buildMapRoadSectionGeometry(spline, centerline, section, joins, cantHalfWidth,
                                    coordinates.tile(road.tileX, road.tileY));
    const auto normals = computeMapRoadVertexNormals(geometry);
    constexpr std::size_t maximum = std::numeric_limits<std::uint32_t>::max();
    if (road.vertices.size() > maximum ||
        geometry.vertices.size() > maximum - road.vertices.size()) {
        throw std::length_error("Road mesh exceeds the indexed geometry limit");
    }
    for (const int index : geometry.indices) {
        if (index < 0 || static_cast<std::size_t>(index) >= geometry.vertices.size()) {
            throw std::invalid_argument("Road geometry contains an invalid vertex index");
        }
    }
    const auto baseVertex = static_cast<std::uint32_t>(road.vertices.size());
    const double tileOriginX = static_cast<double>(road.tileX) * coordinates.tileSize;
    const double tileOriginY = static_cast<double>(road.tileY) * coordinates.tileSize;
    for (std::size_t vertexIndex = 0; vertexIndex < geometry.vertices.size(); ++vertexIndex) {
        const MapRoadVertex& vertex = geometry.vertices[vertexIndex];
        const auto& normal = normals[vertexIndex];
        road.vertices.push_back(
            {static_cast<float>(vertex.x - tileOriginX), static_cast<float>(vertex.y - tileOriginY),
             static_cast<float>(vertex.z), static_cast<float>(vertex.textureU),
             static_cast<float>(vertex.textureV), 0.0F, static_cast<float>(normal[0]),
             static_cast<float>(normal[1]), static_cast<float>(normal[2])});
        road.minX = std::min(road.minX, vertex.x);
        road.minY = std::min(road.minY, vertex.y);
        road.maxX = std::max(road.maxX, vertex.x);
        road.maxY = std::max(road.maxY, vertex.y);
        road.minZ = std::min(road.minZ, vertex.z);
        road.maxZ = std::max(road.maxZ, vertex.z);
    }
    for (const int index : geometry.indices) {
        road.indices.push_back(baseVertex + static_cast<std::uint32_t>(index));
    }
}

} // namespace openbus::map
