#include "physics/BusSimulation.h"
#include "physics/World.h"
#include "map/MapCollisionStreamer.h"
#include "ProbeTempDirectory.h"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace {

void writeU16(std::ofstream& output, std::uint16_t value) {
    output.put(static_cast<char>(value & 0xffU));
    output.put(static_cast<char>((value >> 8U) & 0xffU));
}

void writeU32(std::ofstream& output, std::uint32_t value) {
    for (int byte = 0; byte < 4; ++byte) {
        output.put(static_cast<char>((value >> (byte * 8)) & 0xffU));
    }
}

void writeF32(std::ofstream& output, float value) {
    writeU32(output, std::bit_cast<std::uint32_t>(value));
}

void writeText(const std::filesystem::path& path, const std::string& content) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary);
    if (!output || !(output << content)) {
        throw std::runtime_error("failed to write map collision streaming fixture");
    }
}

void writeStationCollisionMesh(const std::filesystem::path& path) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("failed to write station collision mesh fixture");
    }
    output.put(static_cast<char>(0x84));
    output.put(static_cast<char>(0x19));
    output.put(static_cast<char>(3));
    output.put(static_cast<char>(0x17));
    writeU16(output, 3);
    const std::array<std::array<float, 3>, 3> positions = {
        std::array<float, 3>{-1.0F, 0.0F, -1.0F},
        std::array<float, 3>{1.0F, 0.0F, -1.0F},
        std::array<float, 3>{0.0F, 0.0F, 1.0F}};
    for (const auto& position : positions) {
        for (const float value : position) {
            writeF32(output, value);
        }
        for (int value = 0; value < 5; ++value) {
            writeF32(output, value == 2 ? 1.0F : 0.0F);
        }
    }
    output.put(static_cast<char>(0x49));
    writeU16(output, 1);
    writeU16(output, 0);
    writeU16(output, 2);
    writeU16(output, 1);
    writeU16(output, 0);
}

void writeTerrain(const std::filesystem::path& path) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("failed to write map collision terrain fixture");
    }
    writeU32(output, 1);
    for (int index = 0; index < 4; ++index) {
        writeU32(output, std::bit_cast<std::uint32_t>(0.0F));
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
        openbus::probe::makeProbeDirectory("OpenBusMapCollisionStreamerProbe");
    const std::filesystem::path mapRoot = root / "map";
    const std::filesystem::path omsiRoot = root / "omsi";
    openbus::map::MapDefinition map;
    map.rootPath = mapRoot;
    for (int tileX = -3; tileX <= 3; ++tileX) {
        openbus::map::MapTileReference tile;
        tile.x = tileX;
        tile.y = 0;
        tile.textPath = mapRoot / ("tile_" + std::to_string(tileX) + ".map");
        const std::string sceneryObject =
            tileX == 0
                ? "[object]\n0\nSceneryobjects\\Test\\Station.sco\n77\n5\n5\n0\n0\n0\n0\n0\n"
                : "";
        writeText(tile.textPath, "[version]\n12\n[terrain]\n" + sceneryObject);
        if (tileX == 0) {
            tile.hasTerrainFile = true;
            tile.terrainPath = mapRoot / "tile_0.terrain";
            writeTerrain(tile.terrainPath);
        }
        map.tiles.push_back(std::move(tile));
    }
    writeText(omsiRoot / "Sceneryobjects" / "Test" / "Station.sco",
              "[model]\nstation.cfg\n");
    writeText(omsiRoot / "Sceneryobjects" / "Test" / "station.cfg",
              "[collision_mesh]\nstation.o3d\n");
    writeStationCollisionMesh(omsiRoot / "Sceneryobjects" / "Test" / "station.o3d");
    std::vector<openbus::map::MapTileCoordinate> mapTileCoordinates;
    for (const openbus::map::MapTileReference& tile : map.tiles) {
        mapTileCoordinates.emplace_back(tile.x, tile.y);
    }

    World world(0.0, {}, {}, false);
    BusSimulation simulation(world, probeConfiguration(),
                             VehiclePlacement{{5.0, 5.0, 0.0}, 0.0});
    openbus::map::MapCollisionStreamer streamer(map, omsiRoot, world);
    const auto updateUntilSettled = [&](double worldX, double worldY,
                                        const std::vector<openbus::map::MapTileCoordinate>& tiles,
                                        std::size_t expectedCount, const char* message) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        do {
            streamer.update(worldX, worldY, tiles);
            streamer.waitForPendingBuilds();
            streamer.update(worldX, worldY, tiles);
            if (streamer.loadedTileCount() == expectedCount &&
                world.loadedMapCollisionTileCount() == expectedCount) {
                return;
            }
            std::this_thread::yield();
        } while (std::chrono::steady_clock::now() < deadline);
        require(false, message);
    };

    updateUntilSettled(5.0, 5.0, {{0, 0}}, 1,
                       "collision must load only geometry-resident map tiles");
    for (int attempt = 0; attempt < 4 && world.collisionDebugMeshes().empty(); ++attempt) {
        streamer.update(5.0, 5.0, {{0, 0}});
        streamer.waitForPendingBuilds();
    }
    require(!world.collisionDebugMeshes().empty(),
            "terrain collision should publish independently while static collision builds");
    const auto hasStationCollision = [&]() {
        const std::vector<StaticCollisionMesh> collisionMeshes = world.collisionDebugMeshes();
        return std::any_of(collisionMeshes.begin(), collisionMeshes.end(),
                           [](const StaticCollisionMesh& mesh) {
                               return mesh.vertices.size() == 9 && mesh.indices.size() == 3;
                           });
    };
    const auto collisionDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!hasStationCollision() && std::chrono::steady_clock::now() < collisionDeadline) {
        streamer.update(5.0, 5.0, {{0, 0}});
        streamer.waitForPendingBuilds();
    }
    require(hasStationCollision(),
            "streamed SCO model collision meshes are published to physics");
    updateUntilSettled(
        605.0, 5.0,
        openbus::map::selectMapTilesWithinRadius(mapTileCoordinates, 605.0, 5.0, 2), 4,
        "tiles should load ahead while the previous tile remains in the hysteresis ring");
    updateUntilSettled(
        905.0, 5.0,
        openbus::map::selectMapTilesWithinRadius(mapTileCoordinates, 905.0, 5.0, 2), 3,
        "tiles outside the unload radius should be removed as the bus advances");
    updateUntilSettled(
        -0.01, 5.0,
        openbus::map::selectMapTilesWithinRadius(mapTileCoordinates, -0.01, 5.0, 2), 4,
        "negative world positions should use floor-based tile selection");
    updateUntilSettled(-0.01, 5.0, {{-1, 0}}, 1,
                       "collision residency must follow geometry changes without center movement");
    openbus::probe::removeProbeDirectory(root);
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
