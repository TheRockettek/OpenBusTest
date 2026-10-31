#include "map/MapCollisionBuilder.h"

#include "core/AssetPath.h"
#include "map/MapSceneryPlacement.h"
#include "map/MapRoadGeometry.h"
#include "map/MapSplineGeometry.h"
#include "map/MapSplineProfile.h"
#include "assets/TextureLoader.h"
#include "config/ModelConfigLoader.h"
#include "assets/O3DLoader.h"
#include "config/SceneryObjectConfigLoader.h"
#include "core/Variables.h"
#include "core/Logger.h"
#include "core/PerfTrace.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace openbus::map {
namespace {

constexpr std::size_t MAX_VERTICES_PER_TILE = 1'000'000;
constexpr std::size_t MAX_SURFACE_SUBDIVISIONS = 16;

std::filesystem::path normalizedAssetPath(const std::string& source) {
    std::string normalized = source;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    return std::filesystem::path(normalized);
}

bool isO3DPath(const std::filesystem::path& path) {
    std::string extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    return extension == ".o3d";
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

std::filesystem::path resolveSurfaceMap(const MapDefinition& map,
                                        const std::filesystem::path& omsiRoot,
                                        const std::filesystem::path& profilePath,
                                        const std::string& texturePath) {
    const std::filesystem::path relative = normalizedAssetPath(texturePath);
    const std::filesystem::path profileRoot = profilePath.parent_path();
    for (const std::filesystem::path& textureCandidate :
         {profileRoot / "texture" / relative, profileRoot / relative,
          omsiRoot / "Texture" / relative, omsiRoot / "texture" / relative, omsiRoot / relative,
          map.rootPath / relative}) {
        std::filesystem::path surfaceCandidate = textureCandidate;
        surfaceCandidate += ".surf";
        const std::filesystem::path resolved =
            openbus::assets::resolveCaseInsensitivePath(surfaceCandidate);
        if (std::filesystem::is_regular_file(resolved)) {
            return resolved.lexically_normal();
        }
    }
    return {};
}

std::size_t surfaceSubdivisions(double uvSpan, int pixels) {
    const double requested = std::ceil(std::abs(uvSpan) * static_cast<double>(pixels) * 2.0);
    if (!std::isfinite(requested) || requested >= static_cast<double>(MAX_SURFACE_SUBDIVISIONS)) {
        return MAX_SURFACE_SUBDIVISIONS;
    }
    return std::max<std::size_t>(1, static_cast<std::size_t>(requested));
}

std::uint8_t sampleSurfaceRed(const openbus::rendering::Image& image, double u, double v) {
    const double wrappedU = u - std::floor(u);
    const double wrappedV = v - std::floor(v);
    const double pixelX = wrappedU * static_cast<double>(image.width) - 0.5;
    const double pixelY = wrappedV * static_cast<double>(image.height) - 0.5;
    const int x0 = static_cast<int>(std::floor(pixelX));
    const int y0 = static_cast<int>(std::floor(pixelY));
    const double xBlend = pixelX - static_cast<double>(x0);
    const double yBlend = pixelY - static_cast<double>(y0);
    const auto wrap = [](int value, int size) {
        const int remainder = value % size;
        return remainder < 0 ? remainder + size : remainder;
    };
    const auto redAt = [&](int x, int y) {
        const std::size_t offset = (static_cast<std::size_t>(wrap(y, image.height)) *
                                        static_cast<std::size_t>(image.width) +
                                    static_cast<std::size_t>(wrap(x, image.width))) *
                                   4;
        return static_cast<double>(image.rgba[offset]);
    };
    const double top = std::lerp(redAt(x0, y0), redAt(x0 + 1, y0), xBlend);
    const double bottom = std::lerp(redAt(x0, y0 + 1), redAt(x0 + 1, y0 + 1), xBlend);
    return static_cast<std::uint8_t>(
        std::clamp(std::lround(std::lerp(top, bottom, yBlend)), 0L, 255L));
}

bool applySurfaceHeightMap(MapRoadSectionGeometry& geometry, std::size_t rowCount,
                           std::size_t columnCount, bool mirrored,
                           const openbus::rendering::Image& image) {
    if (image.width <= 0 || image.height <= 0 ||
        image.rgba.size() !=
            static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height) * 4 ||
        geometry.vertices.size() != rowCount * columnCount || rowCount < 2 || columnCount < 2) {
        return false;
    }

    std::vector<MapRoadVertex> vertices;
    std::vector<int> indices;
    for (std::size_t row = 0; row + 1 < rowCount; ++row) {
        for (std::size_t column = 0; column + 1 < columnCount; ++column) {
            const MapRoadVertex& topLeft = geometry.vertices[row * columnCount + column];
            const MapRoadVertex& topRight = geometry.vertices[row * columnCount + column + 1];
            const MapRoadVertex& bottomLeft = geometry.vertices[(row + 1) * columnCount + column];
            const MapRoadVertex& bottomRight =
                geometry.vertices[(row + 1) * columnCount + column + 1];
            const std::size_t horizontal =
                surfaceSubdivisions(std::max(std::abs(topRight.textureU - topLeft.textureU),
                                             std::abs(bottomRight.textureU - bottomLeft.textureU)),
                                    image.width);
            const std::size_t longitudinal =
                surfaceSubdivisions(std::max(std::abs(bottomLeft.textureV - topLeft.textureV),
                                             std::abs(bottomRight.textureV - topRight.textureV)),
                                    image.height);
            if (horizontal + 1 > MAX_VERTICES_PER_TILE / (longitudinal + 1) ||
                vertices.size() > MAX_VERTICES_PER_TILE - (horizontal + 1) * (longitudinal + 1)) {
                throw std::length_error("Road surface height map exceeds collision vertex limit");
            }
            const std::size_t base = vertices.size();
            for (std::size_t y = 0; y <= longitudinal; ++y) {
                const double fy = static_cast<double>(y) / static_cast<double>(longitudinal);
                for (std::size_t x = 0; x <= horizontal; ++x) {
                    const double fx = static_cast<double>(x) / static_cast<double>(horizontal);
                    const auto interpolate = [&](double a, double b, double c, double d) {
                        return std::lerp(std::lerp(a, b, fx), std::lerp(c, d, fx), fy);
                    };
                    MapRoadVertex vertex;
                    vertex.x = interpolate(topLeft.x, topRight.x, bottomLeft.x, bottomRight.x);
                    vertex.y = interpolate(topLeft.y, topRight.y, bottomLeft.y, bottomRight.y);
                    vertex.z = interpolate(topLeft.z, topRight.z, bottomLeft.z, bottomRight.z);
                    vertex.textureU = interpolate(topLeft.textureU, topRight.textureU,
                                                  bottomLeft.textureU, bottomRight.textureU);
                    vertex.textureV = interpolate(topLeft.textureV, topRight.textureV,
                                                  bottomLeft.textureV, bottomRight.textureV);
                    const double heightOffset = (static_cast<double>(sampleSurfaceRed(
                                                     image, vertex.textureU, vertex.textureV)) /
                                                     255.0 -
                                                 0.5) *
                                                0.04;
                    vertex.z += heightOffset;
                    vertices.push_back(vertex);
                }
            }
            for (std::size_t y = 0; y < longitudinal; ++y) {
                for (std::size_t x = 0; x < horizontal; ++x) {
                    const std::size_t topLeftIndex = base + y * (horizontal + 1) + x;
                    const std::size_t topRightIndex = topLeftIndex + 1;
                    const std::size_t bottomLeftIndex = topLeftIndex + horizontal + 1;
                    const std::size_t bottomRightIndex = bottomLeftIndex + 1;
                    if (bottomRightIndex >
                        static_cast<std::size_t>(std::numeric_limits<int>::max())) {
                        throw std::length_error(
                            "Road surface height map exceeds collision index limit");
                    }
                    if (mirrored) {
                        indices.insert(
                            indices.end(),
                            {static_cast<int>(topLeftIndex), static_cast<int>(bottomRightIndex),
                             static_cast<int>(bottomLeftIndex), static_cast<int>(topLeftIndex),
                             static_cast<int>(topRightIndex), static_cast<int>(bottomRightIndex)});
                    } else {
                        indices.insert(
                            indices.end(),
                            {static_cast<int>(topLeftIndex), static_cast<int>(bottomLeftIndex),
                             static_cast<int>(bottomRightIndex), static_cast<int>(topLeftIndex),
                             static_cast<int>(bottomRightIndex), static_cast<int>(topRightIndex)});
                    }
                }
            }
        }
    }
    geometry.vertices = std::move(vertices);
    geometry.indices = std::move(indices);
    return true;
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
    const int tileX = static_cast<int>(std::floor(worldX / map.coordinates.tileSize));
    const int tileY = static_cast<int>(std::floor(worldY / map.coordinates.tileSize));
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
            openbus::rendering::TraceScope trace(
                "map_collision", "MapCollisionBuilder.loadSceneryCollisionTerrainGrid");
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
    const double localX = worldX - static_cast<double>(tileX) * map.coordinates.tileSize;
    const double localY = worldY - static_cast<double>(tileY) * map.coordinates.tileSize;
    const double gridX = std::clamp(localX / map.coordinates.tileSize * grid.intervals, 0.0,
                                    static_cast<double>(grid.intervals));
    const double gridY = std::clamp(localY / map.coordinates.tileSize * grid.intervals, 0.0,
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
    const auto offset = transformMapSceneryVector(pose, {source.x, source.y, source.z});
    return {pose.x + offset[0], pose.y + offset[1], worldZ + offset[2]};
}

std::array<double, 6> parsedMeshBounds(const openbus::rendering::ParsedObj& mesh) {
    std::array<double, 6> bounds = {
        std::numeric_limits<double>::max(),    std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),    std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()};
    for (const auto& vertex : mesh.positions) {
        bounds[0] = std::min(bounds[0], vertex.x);
        bounds[1] = std::min(bounds[1], vertex.y);
        bounds[2] = std::min(bounds[2], vertex.z);
        bounds[3] = std::max(bounds[3], vertex.x);
        bounds[4] = std::max(bounds[4], vertex.y);
        bounds[5] = std::max(bounds[5], vertex.z);
    }
    return bounds;
}

void expandSceneryCollisionObjectBounds(MapSceneryCollisionObject& object,
                                        const std::array<double, 6>& localBounds) {
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

bool hasUsableCollisionTriangles(const openbus::rendering::ParsedObj& mesh) {
    return std::any_of(mesh.triangles.begin(), mesh.triangles.end(), [&](const auto& triangle) {
        return std::all_of(
            triangle.indices.begin(), triangle.indices.end(), [&](const auto& index) {
                return index.position > 0 &&
                       static_cast<std::size_t>(index.position) <= mesh.positions.size();
            });
    });
}

std::shared_ptr<openbus::rendering::ParsedObj>
makeBoundingBoxCollisionMesh(const std::array<double, 6>& boundingBox) {
    auto mesh = std::make_shared<openbus::rendering::ParsedObj>();
    const double halfX = boundingBox[0] * 0.5;
    const double halfY = boundingBox[1] * 0.5;
    const double halfZ = boundingBox[2] * 0.5;
    for (int corner = 0; corner < 8; ++corner) {
        const std::array<double, 3> position = {
            boundingBox[3] + ((corner & 1) == 0 ? -halfX : halfX),
            boundingBox[4] + ((corner & 2) == 0 ? -halfY : halfY),
            boundingBox[5] + ((corner & 4) == 0 ? -halfZ : halfZ)};
        mesh->positions.push_back({position[0], position[1], position[2]});
    }
    constexpr std::array<std::array<int, 3>, 12> triangles = {{
        {{0, 2, 3}},
        {{0, 3, 1}},
        {{4, 5, 7}},
        {{4, 7, 6}},
        {{0, 1, 5}},
        {{0, 5, 4}},
        {{2, 6, 7}},
        {{2, 7, 3}},
        {{0, 4, 6}},
        {{0, 6, 2}},
        {{1, 3, 7}},
        {{1, 7, 5}},
    }};
    for (const auto& indices : triangles) {
        openbus::rendering::ObjTriangle triangle;
        for (std::size_t corner = 0; corner < indices.size(); ++corner) {
            triangle.indices[corner].position = indices[2 - corner] + 1;
        }
        mesh->triangles.push_back(std::move(triangle));
    }
    return mesh;
}

std::vector<std::array<double, 3>> clipPolygonToTile(std::vector<std::array<double, 3>> polygon,
                                                     int tileX, int tileY,
                                                     const MapDefinition& map) {
    const double minX = static_cast<double>(tileX) * map.coordinates.tileSize;
    const double minY = static_cast<double>(tileY) * map.coordinates.tileSize;
    const double maxX = minX + map.coordinates.tileSize;
    const double maxY = minY + map.coordinates.tileSize;
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

std::optional<MapPoleCollision> makeMapPoleCollision(const MapSceneryCollisionObject& object,
                                                     const MapDefinition& map, int tileX,
                                                     int tileY) {
    if (!object.poleCrash.has_value()) {
        return std::nullopt;
    }
    // Lamp arms and signs can extend far beyond the shaft. Fit the horizontal
    // center and width to the lowest fifth of the collision geometry.
    const double lowerLimit = std::lerp(object.localBounds[2], object.localBounds[5], 0.2);
    std::array<double, 4> shaftBounds = {
        std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
        std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()};
    for (const auto& mesh : object.meshes) {
        for (const auto& vertex : mesh->positions) {
            if (vertex.z <= lowerLimit) {
                shaftBounds[0] = std::min(shaftBounds[0], vertex.x);
                shaftBounds[1] = std::min(shaftBounds[1], vertex.y);
                shaftBounds[2] = std::max(shaftBounds[2], vertex.x);
                shaftBounds[3] = std::max(shaftBounds[3], vertex.y);
            }
        }
    }
    const std::array<double, 3> localBottom = {(shaftBounds[0] + shaftBounds[2]) * 0.5,
                                               (shaftBounds[1] + shaftBounds[3]) * 0.5,
                                               object.localBounds[2]};
    const std::array<double, 3> localTop = {localBottom[0], localBottom[1], object.localBounds[5]};
    const auto bottomOffset = transformMapSceneryVector(object.pose, localBottom);
    const auto topOffset = transformMapSceneryVector(object.pose, localTop);
    MapPoleCollision pole;
    pole.mapObjectId = object.mapObjectId;
    pole.tileX = object.sourceTileX;
    pole.tileY = object.sourceTileY;
    pole.basePosition = {object.pose.x + bottomOffset[0], object.pose.y + bottomOffset[1],
                         object.worldZ + bottomOffset[2]};
    pole.topPosition = {object.pose.x + topOffset[0], object.pose.y + topOffset[1],
                        object.worldZ + topOffset[2]};
    const double horizontalWidth = shaftBounds[2] - shaftBounds[0];
    const double horizontalDepth = shaftBounds[3] - shaftBounds[1];
    const double narrowestWidth = std::min(horizontalWidth, horizontalDepth) > 1.0e-4
                                      ? std::min(horizontalWidth, horizontalDepth)
                                      : std::max(horizontalWidth, horizontalDepth);
    const auto xAxis = transformMapSceneryVector(object.pose, {1.0, 0.0, 0.0});
    const auto yAxis = transformMapSceneryVector(object.pose, {0.0, 1.0, 0.0});
    const double boundRadius = 0.5 * narrowestWidth *
                               std::max(std::hypot(xAxis[0], xAxis[1], xAxis[2]),
                                        std::hypot(yAxis[0], yAxis[1], yAxis[2]));
    const double poleLength = std::hypot(pole.topPosition[0] - pole.basePosition[0],
                                         pole.topPosition[1] - pole.basePosition[1],
                                         pole.topPosition[2] - pole.basePosition[2]);
    pole.radiusMeters = std::min(boundRadius, std::max(0.0, (poleLength - 0.01) * 0.5));
    pole.resistanceRadiansPerTonneMeterPerSecond =
        object.poleCrash->resistanceRadiansPerTonneMeterPerSecond;
    pole.breakAngleRadians = object.poleCrash->breakAngleRadians;
    const int baseTileX =
        static_cast<int>(std::floor(pole.basePosition[0] / map.coordinates.tileSize));
    const int baseTileY =
        static_cast<int>(std::floor(pole.basePosition[1] / map.coordinates.tileSize));
    if (baseTileX != tileX || baseTileY != tileY ||
        std::hypot(pole.topPosition[0] - pole.basePosition[0],
                   pole.topPosition[1] - pole.basePosition[1],
                   pole.topPosition[2] - pole.basePosition[2]) <= 0.1 ||
        !std::isfinite(pole.radiusMeters) || pole.radiusMeters <= 0.0) {
        return std::nullopt;
    }
    return pole;
}

void appendSceneryCollisionForTile(StaticCollisionMesh& tileMesh,
                                   const MapSceneryCollisionIndex& index, int tileX, int tileY,
                                   MapRoadCollisionResult& result, const MapDefinition& map) {
    const auto found = index.objectsByTile.find(tileKey(tileX, tileY));
    if (found == index.objectsByTile.end()) {
        return;
    }
    bool objectAdded = false;
    const double minX = static_cast<double>(tileX) * map.coordinates.tileSize;
    const double minY = static_cast<double>(tileY) * map.coordinates.tileSize;
    const double maxX = minX + map.coordinates.tileSize;
    const double maxY = minY + map.coordinates.tileSize;
    for (const std::size_t objectIndex : found->second) {
        if (objectIndex >= index.objects.size()) {
            continue;
        }
        const MapSceneryCollisionObject& object = index.objects[objectIndex];
        if (object.poleCrash.has_value()) {
            if (auto pole = makeMapPoleCollision(object, map, tileX, tileY)) {
                result.poles.push_back(std::move(*pole));
            }
            continue;
        }
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
                polygon = clipPolygonToTile(std::move(polygon), tileX, tileY, map);
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

struct SceneryCollisionDefinition {
    std::vector<std::filesystem::path> meshes;
    std::vector<std::filesystem::path> modelGeometryMeshes;
    bool noCollision = false;
    bool onlyEditor = false;
    bool surface = false;
    bool absoluteHeight = false;
    std::optional<std::array<double, 6>> boundingBox;
    std::optional<SceneryPoleCrashConfiguration> poleCrash;
};

SceneryCollisionDefinition readSceneryCollisionDefinition(const std::filesystem::path& configPath,
                                                          const std::filesystem::path& omsiRoot,
                                                          const std::filesystem::path& mapRoot) {
    openbus::rendering::TraceScope trace("map_collision",
                                         "MapCollisionBuilder.readSceneryCollisionDefinition");
    SceneryCollisionDefinition definition;
    const SceneryObjectConfig scenery = loadSceneryObjectFile(configPath);
    definition.noCollision = scenery.noCollision;
    definition.onlyEditor = scenery.onlyEditor;
    definition.surface = scenery.surface;
    definition.absoluteHeight = scenery.absoluteHeight;
    definition.poleCrash = scenery.poleCrash;
    if (scenery.hasBoundingBox) {
        definition.boundingBox = scenery.boundingBox;
    }
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
            definition.meshes.push_back(direct);
        }
    }

    const std::filesystem::path modelConfigPath = resolveSceneryObjectModelConfigPath(scenery);
    if (modelConfigPath.empty() || !std::filesystem::is_regular_file(modelConfigPath)) {
        return definition;
    }
    const std::filesystem::path modelRoot =
        scenery.modelPath.empty() ? objectRoot / "model" : modelConfigPath.parent_path();
    openbus::scripting::SceneryObject variables;
    const ModelConfig model =
        loadModelConfig(modelConfigPath, modelRoot, ModelConfigKind::SceneryObject, variables);
    const bool surfaceOverridesNoCollision = scenery.surface;
    definition.absoluteHeight = definition.absoluteHeight || model.absoluteHeight;
    if (!definition.boundingBox && model.hasBoundingBox) {
        definition.boundingBox = model.boundingBox;
    }
    for (const ModelCollisionMesh& collision : model.collisionMeshes) {
        if (collision.hasPart && collision.partIndex < model.parts.size() &&
            ((!surfaceOverridesNoCollision && model.parts[collision.partIndex].noCollision) ||
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
            if (std::none_of(definition.meshes.begin(), definition.meshes.end(),
                             [&](const auto& existing) {
                                 return existing.lexically_normal() == resolved;
                             })) {
                definition.meshes.push_back(std::move(resolved));
            }
        }
    }

    for (const ModelPart& part : model.parts) {
        if ((!surfaceOverridesNoCollision && part.noCollision) || part.isShadow ||
            part.lodIndex > 0) {
            continue;
        }
        std::filesystem::path meshPath = part.objPath;
        if (!isO3DPath(meshPath)) {
            meshPath = resolveAsset(normalizedAssetPath(part.sourceMeshPath.generic_string()),
                                    modelRoot, omsiRoot, mapRoot);
        }
        if (!isO3DPath(meshPath) || !std::filesystem::is_regular_file(meshPath)) {
            continue;
        }
        meshPath = meshPath.lexically_normal();
        if (std::none_of(
                definition.modelGeometryMeshes.begin(), definition.modelGeometryMeshes.end(),
                [&](const auto& existing) { return existing.lexically_normal() == meshPath; })) {
            definition.modelGeometryMeshes.push_back(std::move(meshPath));
        }
    }
    return definition;
}

} // namespace

std::shared_ptr<const MapSceneryCollisionIndex>
buildMapSceneryCollisionIndex(const MapDefinition& map, const std::filesystem::path& omsiRoot) {
    openbus::rendering::TraceScope trace("map_collision",
                                         "MapCollisionBuilder.buildMapSceneryCollisionIndex");
    struct ConfigCollision {
        std::vector<std::filesystem::path> paths;
        std::vector<std::filesystem::path> modelGeometryPaths;
        std::shared_ptr<openbus::rendering::ParsedObj> boundingBoxMesh;
        bool noCollision = false;
        bool onlyEditor = false;
        bool surface = false;
        bool absoluteHeight = false;
        std::optional<SceneryPoleCrashConfiguration> poleCrash;
    };
    auto index = std::make_shared<MapSceneryCollisionIndex>();
    std::unordered_map<std::string, ConfigCollision> configCache;
    std::unordered_map<std::string, std::filesystem::path> sceneryConfigPathCache;
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
        openbus::rendering::TraceScope tileTrace(
            "map_collision", "MapCollisionBuilder.processSceneryTilePlacements");
        for (const MapSceneryPlacement& placement : tile.sceneryObjects) {
            if (!placement.transformValid) {
                continue;
            }
            const std::optional<MapSceneryPose> pose =
                placeMapSceneryObject(placement, tileReference.x, tileReference.y, map.coordinates);
            if (!pose) {
                continue;
            }
            std::filesystem::path configPath;
            {
                openbus::rendering::TraceScope resolveConfigTrace(
                    "map_collision", "MapCollisionBuilder.resolveSceneryConfigPath");
                const std::filesystem::path normalizedPath =
                    normalizedAssetPath(placement.assetPath);
                const std::string assetKey = normalizedPath.generic_string();
                const auto cachedPath = sceneryConfigPathCache.find(assetKey);
                if (cachedPath != sceneryConfigPathCache.end()) {
                    configPath = cachedPath->second;
                } else {
                    openbus::rendering::TraceScope filesystemResolveTrace(
                        "map_collision", "MapCollisionBuilder.resolveSceneryConfigPathFilesystem");
                    configPath = resolveAsset(normalizedPath, map.rootPath, omsiRoot, map.rootPath);
                    sceneryConfigPathCache.emplace(assetKey, configPath);
                }
            }
            if (configPath.empty()) {
                continue;
            }
            const std::string configKey = configPath.lexically_normal().generic_string();
            auto config = configCache.find(configKey);
            if (config == configCache.end()) {
                SceneryCollisionDefinition definition =
                    readSceneryCollisionDefinition(configPath, omsiRoot, map.rootPath);
                ConfigCollision collision;
                collision.paths = std::move(definition.meshes);
                collision.modelGeometryPaths = std::move(definition.modelGeometryMeshes);
                collision.noCollision = definition.noCollision;
                collision.onlyEditor = definition.onlyEditor;
                collision.surface = definition.surface;
                collision.absoluteHeight = definition.absoluteHeight;
                collision.poleCrash = definition.poleCrash;
                if (definition.boundingBox.has_value()) {
                    collision.boundingBoxMesh =
                        makeBoundingBoxCollisionMesh(*definition.boundingBox);
                }
                config = configCache.emplace(configKey, std::move(collision)).first;
            }
            const ConfigCollision& collision = config->second;
            if ((collision.noCollision && !collision.surface) || collision.onlyEditor ||
                (collision.paths.empty() && collision.modelGeometryPaths.empty() &&
                 !collision.boundingBoxMesh)) {
                continue;
            }

            MapSceneryCollisionObject object;
            object.mapObjectId = placement.id;
            object.sourceTileX = tileReference.x;
            object.sourceTileY = tileReference.y;
            object.pose = *pose;
            object.poleCrash = collision.poleCrash;
            const double ground = collision.absoluteHeight
                                      ? 0.0
                                      : sampleTerrainHeight(map, pose->x, pose->y, terrainCache);
            object.worldZ = mapSceneryWorldHeight(pose->z, ground, collision.absoluteHeight);
            object.minX = object.minY = std::numeric_limits<double>::max();
            object.maxX = object.maxY = std::numeric_limits<double>::lowest();
            object.localBounds = {
                std::numeric_limits<double>::max(),    std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max(),    std::numeric_limits<double>::lowest(),
                std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()};
            const auto addMesh = [&](const std::shared_ptr<openbus::rendering::ParsedObj>& parsed,
                                     const std::array<double, 6>& localBounds) {
                object.meshes.push_back(parsed);
                object.localBounds[0] = std::min(object.localBounds[0], localBounds[0]);
                object.localBounds[1] = std::min(object.localBounds[1], localBounds[1]);
                object.localBounds[2] = std::min(object.localBounds[2], localBounds[2]);
                object.localBounds[3] = std::max(object.localBounds[3], localBounds[3]);
                object.localBounds[4] = std::max(object.localBounds[4], localBounds[4]);
                object.localBounds[5] = std::max(object.localBounds[5], localBounds[5]);
                expandSceneryCollisionObjectBounds(object, localBounds);
            };
            const auto addMeshesFromPaths = [&](const std::vector<std::filesystem::path>& paths) {
                for (const std::filesystem::path& meshPath : paths) {
                    const std::string meshKey = meshPath.lexically_normal().generic_string();
                    auto mesh = meshCache.find(meshKey);
                    if (mesh == meshCache.end()) {
                        try {
                            openbus::rendering::TraceScope parseMeshTrace(
                                "map_collision", "MapCollisionBuilder.parseSceneryCollisionMesh");
                            mesh = meshCache
                                       .emplace(meshKey,
                                                openbus::rendering::O3DLoader::parse(meshPath))
                                       .first;
                        } catch (const std::exception&) {
                            mesh = meshCache.emplace(meshKey, nullptr).first;
                        }
                    }
                    if (!mesh->second || !hasUsableCollisionTriangles(*mesh->second)) {
                        continue;
                    }
                    auto bounds = meshLocalBoundsCache.find(meshKey);
                    if (bounds == meshLocalBoundsCache.end()) {
                        bounds =
                            meshLocalBoundsCache.emplace(meshKey, parsedMeshBounds(*mesh->second))
                                .first;
                    }
                    addMesh(mesh->second, bounds->second);
                }
            };
            addMeshesFromPaths(collision.paths);
            if (object.meshes.empty()) {
                if (collision.boundingBoxMesh) {
                    addMesh(collision.boundingBoxMesh,
                            parsedMeshBounds(*collision.boundingBoxMesh));
                } else {
                    addMeshesFromPaths(collision.modelGeometryPaths);
                }
            }
            if (object.meshes.empty()) {
                continue;
            }
            const std::size_t objectIndex = index->objects.size();
            index->objects.push_back(std::move(object));
            const MapSceneryCollisionObject& stored = index->objects.back();
            const int minTileX =
                static_cast<int>(std::floor(stored.minX / map.coordinates.tileSize));
            const int minTileY =
                static_cast<int>(std::floor(stored.minY / map.coordinates.tileSize));
            const int maxTileX =
                static_cast<int>(std::floor(stored.maxX / map.coordinates.tileSize));
            const int maxTileY =
                static_cast<int>(std::floor(stored.maxY / map.coordinates.tileSize));
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
    openbus::rendering::TraceScope trace(
        "map_collision", selectedTile == nullptr
                             ? "MapCollisionBuilder.buildAllRoadCollision"
                             : "MapCollisionBuilder.buildSelectedTileRoadCollision");
    MapRoadCollisionResult result;
    std::unordered_map<std::string, MapSplineProfile> profiles;
    std::unordered_set<std::string> failedProfiles;
    std::unordered_map<std::string, std::optional<std::filesystem::path>> surfaceMapPaths;
    std::unordered_map<std::string, std::optional<openbus::rendering::Image>> surfaceMaps;

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
                static_cast<double>(tileReference.x) * map.coordinates.tileSize + spline.localX;
            const double startY =
                static_cast<double>(tileReference.y) * map.coordinates.tileSize + spline.localY;
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
                        cantHalfWidth, map.coordinates.tile(tileReference.x, tileReference.y));
                } catch (const std::exception&) {
                    ++result.skippedSplines;
                    continue;
                }
                const std::string& texturePath =
                    profile.textures[static_cast<std::size_t>(section.textureIndex)];
                bool surfaceHeightMapApplied = false;
                const std::string surfaceRequestKey = profileKey + '\0' + texturePath;
                auto surfacePath = surfaceMapPaths.find(surfaceRequestKey);
                if (surfacePath == surfaceMapPaths.end()) {
                    std::filesystem::path resolved =
                        resolveSurfaceMap(map, omsiRoot, profilePath, texturePath);
                    surfacePath =
                        surfaceMapPaths
                            .emplace(surfaceRequestKey, resolved.empty()
                                                            ? std::nullopt
                                                            : std::optional<std::filesystem::path>(
                                                                  std::move(resolved)))
                            .first;
                }
                if (surfacePath->second.has_value()) {
                    const std::string surfaceKey =
                        surfacePath->second->lexically_normal().generic_string();
                    auto surface = surfaceMaps.find(surfaceKey);
                    if (surface == surfaceMaps.end()) {
                        openbus::rendering::Image image;
                        if (openbus::rendering::TextureLoader::readImage(*surfacePath->second,
                                                                         image)) {
                            surface = surfaceMaps.emplace(surfaceKey, std::move(image)).first;
                        } else {
                            gameLog.Warning("Map road surface height map could not be decoded: " +
                                            surfacePath->second->string());
                            surface = surfaceMaps.emplace(surfaceKey, std::nullopt).first;
                        }
                    }
                    if (surface->second.has_value()) {
                        surfaceHeightMapApplied = applySurfaceHeightMap(
                            sectionGeometry, centerline.size(), section.points.size(),
                            spline.mirrored, *surface->second);
                        if (!surfaceHeightMapApplied) {
                            gameLog.Warning(
                                "Map road surface height map has invalid image dimensions: " +
                                surfacePath->second->string());
                        }
                    }
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
                    const std::array<int, 3> triangle = {
                        baseVertex + sectionGeometry.indices[index],
                        baseVertex + sectionGeometry.indices[index + 2],
                        baseVertex + sectionGeometry.indices[index + 1]};
                    tileMesh.indices.insert(tileMesh.indices.end(), triangle.begin(),
                                            triangle.end());
                    if (surfaceHeightMapApplied) {
                        tileMesh.surfaceHeightMapIndices.insert(
                            tileMesh.surfaceHeightMapIndices.end(), triangle.begin(),
                            triangle.end());
                    }
                }
                ++result.splineSectionsAdded;
            }
        }

        if (scenery != nullptr) {
            appendSceneryCollisionForTile(tileMesh, *scenery, tileReference.x, tileReference.y,
                                          result, map);
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
    openbus::rendering::TraceScope trace("map_collision",
                                         "MapCollisionBuilder.buildMapRoadCollisionTile");
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

std::vector<MapPoleCollision> buildMapPoleCollisionTile(const MapDefinition& map,
                                                        const std::filesystem::path& omsiRoot,
                                                        int tileX, int tileY,
                                                        const MapSceneryCollisionIndex* scenery) {
    const auto tile = std::find_if(map.tiles.begin(), map.tiles.end(), [&](const auto& reference) {
        return reference.x == tileX && reference.y == tileY;
    });
    if (tile == map.tiles.end()) {
        return {};
    }
    const std::shared_ptr<const MapSceneryCollisionIndex> ownedScenery =
        scenery == nullptr ? buildMapSceneryCollisionIndex(map, omsiRoot) : nullptr;
    const MapSceneryCollisionIndex& collisionIndex = scenery == nullptr ? *ownedScenery : *scenery;
    const auto found = collisionIndex.objectsByTile.find(tileKey(tileX, tileY));
    if (found == collisionIndex.objectsByTile.end()) {
        return {};
    }
    std::vector<MapPoleCollision> poles;
    for (const std::size_t index : found->second) {
        if (index >= collisionIndex.objects.size()) {
            continue;
        }
        if (auto pole = makeMapPoleCollision(collisionIndex.objects[index], map, tileX, tileY)) {
            poles.push_back(std::move(*pole));
        }
    }
    return poles;
}

MapRoadConnectionIndex loadMapRoadConnections(const MapDefinition& map,
                                              const std::filesystem::path& omsiRoot) {
    openbus::rendering::TraceScope trace("map_collision",
                                         "MapCollisionBuilder.loadMapRoadConnections");
    MapRoadConnectionIndex connections(map.coordinates);
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
