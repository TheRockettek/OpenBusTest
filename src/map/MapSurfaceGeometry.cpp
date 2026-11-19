#include "map/MapSurfaceGeometry.h"

#include "map/MapWaterGeometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace openbus::map {

MapTerrainMesh prepareMapTerrainMesh(const TerrainGrid& grid, double tileSize) {
    if (grid.intervals == 0 || grid.intervals > 1024 || !std::isfinite(tileSize) ||
        tileSize <= 0.0 || tileSize > std::numeric_limits<float>::max()) {
        throw std::invalid_argument("Invalid terrain grid intervals or tile size");
    }
    const std::size_t side = grid.intervals + 1;
    if (grid.heights.size() != side * side ||
        !std::all_of(grid.heights.begin(), grid.heights.end(),
                     [](float height) { return std::isfinite(height); })) {
        throw std::invalid_argument("Terrain heights do not match the grid or are non-finite");
    }
    MapTerrainMesh mesh;
    const double step = tileSize / grid.intervals;
    mesh.vertices.reserve(side * side);
    for (std::size_t row = 0; row < side; ++row) {
        for (std::size_t column = 0; column < side; ++column) {
            mesh.vertices.push_back({static_cast<float>(column * step),
                                     static_cast<float>(row * step),
                                     grid.heights[row * side + column], static_cast<float>(column),
                                     static_cast<float>(row), 0.0F, 0.0F, 0.0F, 1.0F});
        }
    }
    mesh.indices.reserve(grid.intervals * grid.intervals * 6);
    for (std::size_t row = 0; row < grid.intervals; ++row) {
        for (std::size_t column = 0; column < grid.intervals; ++column) {
            const auto topLeft = static_cast<std::uint32_t>(row * side + column);
            const auto topRight = static_cast<std::uint32_t>(row * side + column + 1);
            const auto bottomRight = static_cast<std::uint32_t>((row + 1) * side + column + 1);
            const auto bottomLeft = static_cast<std::uint32_t>((row + 1) * side + column);
            mesh.indices.insert(mesh.indices.end(),
                                {topLeft, bottomRight, topRight, topLeft, bottomLeft, bottomRight});
        }
    }
    const auto [minimum, maximum] = std::minmax_element(grid.heights.begin(), grid.heights.end());
    mesh.minimumHeight = *minimum;
    mesh.maximumHeight = *maximum;
    return mesh;
}

MapWaterMesh prepareMapWaterMesh(const WaterData& water, int tileX, int tileY, double tileSize) {
    if (!std::isfinite(tileSize) || tileSize <= 0.0 ||
        tileSize > std::numeric_limits<float>::max()) {
        throw std::invalid_argument("Water tile size must be finite and positive");
    }
    if (water.surfaces.size() > std::numeric_limits<std::uint32_t>::max() / 4 ||
        water.surfaces.size() > std::numeric_limits<std::size_t>::max() / 6) {
        throw std::length_error("Water mesh exceeds the indexed geometry limit");
    }
    MapWaterMesh mesh;
    mesh.vertices.reserve(water.surfaces.size() * 4);
    mesh.indices.reserve(water.surfaces.size() * 6);
    double minimumHeight = std::numeric_limits<double>::infinity();
    double maximumHeight = -std::numeric_limits<double>::infinity();
    double heightSum = 0.0;
    const double originX = static_cast<double>(tileX) * tileSize;
    const double originY = static_cast<double>(tileY) * tileSize;
    for (const WaterSurface& surface : water.surfaces) {
        if (!std::all_of(surface.heights.begin(), surface.heights.end(),
                         [](float height) { return std::isfinite(height); })) {
            throw std::invalid_argument("Water heights must be finite");
        }
        const auto corners = buildMapWaterSurfaceVertices(tileX, tileY, surface, tileSize);
        const auto base = static_cast<std::uint32_t>(mesh.vertices.size());
        for (std::size_t corner = 0; corner < corners.size(); ++corner) {
            const MapWaterVertex& vertex = corners[corner];
            mesh.vertices.push_back(
                {static_cast<float>(vertex.x - originX), static_cast<float>(vertex.y - originY),
                 static_cast<float>(vertex.z), corner == 1 || corner == 2 ? 1.0F : 0.0F,
                 corner >= 2 ? 1.0F : 0.0F, 0.0F, 0.0F, 0.0F, 1.0F});
            minimumHeight = std::min(minimumHeight, vertex.z);
            maximumHeight = std::max(maximumHeight, vertex.z);
            heightSum += vertex.z;
        }
        mesh.indices.insert(mesh.indices.end(),
                            {base, base + 1, base + 2, base, base + 2, base + 3});
    }
    if (!mesh.vertices.empty()) {
        mesh.minimumHeight = minimumHeight;
        mesh.maximumHeight = maximumHeight;
        mesh.meanHeight = heightSum / static_cast<double>(mesh.vertices.size());
    }
    return mesh;
}

} // namespace openbus::map
