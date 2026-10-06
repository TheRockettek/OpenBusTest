#include "BusSimulation.h"
#include "MapCollisionStreamer.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

void writeTile(const std::filesystem::path& path) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary);
    if (!output || !(output << "[version]\n12\n")) {
        throw std::runtime_error("failed to write map collision streaming fixture");
    }
}

BusConfiguration probeConfiguration() {
    BusConfiguration configuration{};
    configuration.mass = 10000.0;
    configuration.length = 10.0;
    configuration.width = 2.5;
    configuration.bodyHalfLength = 5.0;
    configuration.bodyHalfWidth = 1.25;
    configuration.bodyHalfHeight = 1.4;
    configuration.wheelRadius = 0.48;
    configuration.wheelHalfWidth = 0.14;
    configuration.collisionLength = 10.0;
    configuration.collisionWidth = 2.5;
    configuration.collisionHeight = 2.8;
    configuration.centerOfGravityHeight = 1.2;
    configuration.axles = {{3.0, 2.4, 2.4, 1.8, 0.96, 200000.0, 80000.0,
                            18000.0, true, false},
                           {-3.0, 2.4, 2.4, 1.8, 0.96, 200000.0, 80000.0,
                            18000.0, false, true}};
    return configuration;
}

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void runProbe() {
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "OpenBusMapCollisionStreamerProbe";
    std::filesystem::remove_all(root);
    const std::filesystem::path mapRoot = root / "map";
    openbus::map::MapDefinition map;
    map.rootPath = mapRoot;
    for (int tileX = -3; tileX <= 3; ++tileX) {
        openbus::map::MapTileReference tile;
        tile.x = tileX;
        tile.y = 0;
        tile.textPath = mapRoot / ("tile_" + std::to_string(tileX) + ".map");
        writeTile(tile.textPath);
        map.tiles.push_back(std::move(tile));
    }

    BusSimulation simulation(probeConfiguration(), VehiclePlacement{{5.0, 5.0, 0.0}, 0.0},
                             60.0, 8, 0.0, {}, {}, false);
    openbus::map::MapCollisionStreamer streamer(map, root / "omsi", simulation);
    streamer.update(5.0, 5.0);
    require(streamer.loadedTileCount() == 3 && simulation.loadedMapCollisionTileCount() == 3,
            "startup should load only the 3x3 neighborhood intersecting available tiles");

    streamer.update(605.0, 5.0);
    require(streamer.loadedTileCount() == 4 && simulation.loadedMapCollisionTileCount() == 4,
            "tiles should load ahead while the previous tile remains in the hysteresis ring");

    streamer.update(905.0, 5.0);
    require(streamer.loadedTileCount() == 3 && simulation.loadedMapCollisionTileCount() == 3,
            "tiles outside the unload radius should be removed as the bus advances");

    streamer.update(-0.01, 5.0);
    require(streamer.loadedTileCount() == 4 && simulation.loadedMapCollisionTileCount() == 4,
            "negative world positions should use floor-based tile selection");
    std::filesystem::remove_all(root);
}

} // namespace

int main() {
    try {
        runProbe();
    } catch (const std::exception& error) {
        std::cerr << "map collision streamer probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
