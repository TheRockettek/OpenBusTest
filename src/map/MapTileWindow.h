#pragma once

#include "map/MapConfigLoader.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace openbus::map {

using MapTileCoordinate = std::pair<int, int>;

inline int mapWorldTile(double coordinate) {
    if (!std::isfinite(coordinate)) {
        throw std::invalid_argument("Map tile position must be finite");
    }
    const double tile = std::floor(coordinate / OMSI_TILE_SIZE_METERS);
    if (tile < static_cast<double>(std::numeric_limits<int>::min()) ||
        tile > static_cast<double>(std::numeric_limits<int>::max())) {
        throw std::out_of_range("Map position exceeds tile coordinates");
    }
    return static_cast<int>(tile);
}

inline bool mapTileWithinRadius(int x, int y, int centerX, int centerY, int radius) {
    return radius >= 0 && std::abs(static_cast<long long>(x) - centerX) <= radius &&
           std::abs(static_cast<long long>(y) - centerY) <= radius;
}

inline std::vector<MapTileCoordinate>
selectMapTilesWithinRadius(const std::vector<MapTileCoordinate>& mapTiles, double worldX,
                           double worldY, int radius) {
    if (radius < 0) {
        throw std::invalid_argument("Map tile window radius must not be negative");
    }
    const int centerX = mapWorldTile(worldX);
    const int centerY = mapWorldTile(worldY);
    std::vector<MapTileCoordinate> selected;
    selected.reserve(mapTiles.size());
    for (const MapTileCoordinate& tile : mapTiles) {
        if (mapTileWithinRadius(tile.first, tile.second, centerX, centerY, radius)) {
            selected.push_back(tile);
        }
    }
    std::sort(selected.begin(), selected.end());
    selected.erase(std::unique(selected.begin(), selected.end()), selected.end());
    return selected;
}

} // namespace openbus::map
