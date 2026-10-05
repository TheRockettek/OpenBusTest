#include "MapConfigLoader.h"

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const char* description) {
    if (!condition) {
        throw std::runtime_error(description);
    }
}

void writeUtf16Le(const std::filesystem::path& path, const std::string& text) {
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("could not create UTF-16 fixture");
    }
    output.put(static_cast<char>(0xff));
    output.put(static_cast<char>(0xfe));
    for (const unsigned char character : text) {
        output.put(static_cast<char>(character));
        output.put('\0');
    }
}

void writeU32(std::ofstream& output, std::uint32_t value) {
    for (int byte = 0; byte < 4; ++byte) {
        output.put(static_cast<char>((value >> (byte * 8)) & 0xffU));
    }
}

void runFixtureProbe() {
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "openbus-map-config-probe";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    try {
        writeUtf16Le(root / "global.cfg",
                     "[name]\nFixture map\n[groundtex]\nTexture\\grass.bmp\n"
                     "Texture\\detail.bmp\n0\n1\n60\n[entrypoints]\n1\n"
                     "2\n11\n0\n10\n20\n0\n0\n0\n0\n1\n0\nDepot\n"
                     "[map]\n0\n0\ntile_0_0.map\n");
        writeUtf16Le(root / "tile_0_0.map",
                     "Fixture tile\n[version]\n14\n[terrain]\n\n"
                     "[spline]\n0\nSplines\\road.sli\n30273\n30272\n0\n10\n20\n30\n"
                     "0\n50\n0\n0\n0\n0\n0\n0\n0\n0\n\n"
                     "Object Nr. 4\n[object]\n0\n"
                     "Scenery\\tree.sco\n5\n1\n2\n0\n0\n0\n0\n0\n"
                     "[attachObj]\n");
        {
            std::ofstream terrain(root / "tile_0_0.map.terrain", std::ios::binary);
            writeU32(terrain, 2);
            for (std::uint32_t value = 1; value <= 9; ++value) {
                writeU32(terrain, std::bit_cast<std::uint32_t>(static_cast<float>(value)));
            }
        }

        const openbus::map::MapDefinition definition = openbus::map::loadMapDefinition(root);
        require(definition.groundTextures.size() == 1, "groundtex record parsed");
        require(definition.groundTextures[0].texturePath == "Texture\\grass.bmp",
                "ground texture path preserved");
        require(definition.entryPoints.size() == 1, "entrypoint parsed");
        require(definition.entryPoints[0].tileIndex == 0 &&
                    definition.entryPoints[0].name == "Depot",
                "entrypoint fields mapped");
        require(definition.entryPoints[0].placement.position[0] == 10.0 &&
                    definition.entryPoints[0].placement.position[1] == 0.0 &&
                    definition.entryPoints[0].placement.position[2] == 20.0,
                "entrypoint placement maps tile-local coordinates into map coordinates");
        require(definition.tiles.size() == 1 && definition.tiles[0].hasTerrainFile,
                "tile manifest and sidecar resolved");

        const openbus::map::MapTileData tile = openbus::map::loadMapTile(definition.tiles[0]);
        require(tile.version == "14" && tile.hasTerrainMarker, "tile header parsed");
        require(tile.splineCount == 1 && tile.objectCount == 1 &&
                    tile.attachedObjectCount == 1,
                "tile record families counted");
        require(tile.splines.size() == 1 && tile.splines[0].geometryValid &&
                    tile.splines[0].assetPath == "Splines\\road.sli" &&
                    tile.splines[0].localX == 10.0 && tile.splines[0].elevation == 20.0 &&
                    tile.splines[0].localY == 30.0 && tile.splines[0].length == 50.0,
                "fixed-width spline geometry parsed while raw fields remain available");
        require(tile.sceneryObjects.size() == 1 && tile.sceneryObjects[0].id == 5 &&
                    tile.sceneryObjects[0].label == "Object Nr. 4" &&
                    tile.sceneryObjects[0].line1 == "0" &&
                    tile.sceneryObjects[0].assetPath == "Scenery\\tree.sco" &&
                    tile.sceneryObjects[0].transformValid &&
                    tile.sceneryObjects[0].localPosition[0] == 1.0 &&
                    tile.sceneryObjects[0].localPosition[1] == 2.0,
                "scenery object placement fields preserved");

        const openbus::map::TerrainGrid terrain =
            openbus::map::loadTerrainGrid(definition.tiles[0].terrainPath);
        require(terrain.intervals == 2 && terrain.heights.size() == 9 &&
                    terrain.heights.front() == 1.0F && terrain.heights.back() == 9.0F,
                "little-endian terrain grid decoded");
    } catch (...) {
        std::filesystem::remove_all(root);
        throw;
    }
    std::filesystem::remove_all(root);
}

void runInstalledMapProbe(const std::filesystem::path& root) {
    const openbus::map::MapDefinition definition = openbus::map::loadMapDefinition(root);
    std::size_t terrainCount = 0;
    std::size_t splineCount = 0;
    std::size_t objectCount = 0;
    std::size_t sceneryPlacementCount = 0;
    for (const openbus::map::MapTileReference& reference : definition.tiles) {
        const openbus::map::MapTileData tile = openbus::map::loadMapTile(reference);
        splineCount += tile.splineCount + tile.elevatedSplineCount;
        sceneryPlacementCount += tile.sceneryObjects.size();
        objectCount += tile.objectCount + tile.attachedObjectCount +
                       tile.splineAttachmentCount + tile.splineRepeaterCount;
        if (reference.hasTerrainFile) {
            const openbus::map::TerrainGrid terrain =
                openbus::map::loadTerrainGrid(reference.terrainPath);
            require(terrain.heights.size() == (terrain.intervals + 1) * (terrain.intervals + 1),
                    "terrain vertex count matches interval header");
            ++terrainCount;
        }
    }
    std::cout << "tiles=" << definition.tiles.size()
              << " groundtex=" << definition.groundTextures.size()
              << " entrypoints=" << definition.entryPoints.size()
              << " terrain=" << terrainCount << " spline_records=" << splineCount
              << " object_records=" << objectCount
              << " scenery_placements=" << sceneryPlacementCount << '\n';
    if (root.filename() == "Grande Porto 2022") {
        const VehiclePlacement& spawn = definition.entryPoints.front().placement;
        require(definition.entryPoints.front().name == "Porto (Boavista-Bom Sucesso)",
                "Grande Porto first entrypoint selected as the default spawn");
        require(std::abs(spawn.position[0] - -2009.882) < 1.0e-6 &&
                    std::abs(spawn.position[1] - 3586.663) < 1.0e-6 &&
                    std::abs(spawn.position[2] - 64.540) < 1.0e-6,
                "Grande Porto default spawn uses tile-local x/z/y fields");
        std::cout << "default_spawn=" << definition.entryPoints.front().name
                  << " position=" << spawn.position[0] << ',' << spawn.position[1] << ','
                  << spawn.position[2] << " yaw=" << spawn.yawDegrees << '\n';
    }
}

} // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 1) {
            runFixtureProbe();
        } else if (argc == 2) {
            runInstalledMapProbe(argv[1]);
        } else {
            std::cerr << "usage: OpenBusMapConfigProbe [map-directory]\n";
            return 2;
        }
    } catch (const std::exception& error) {
        std::cerr << "map config probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
