#include "MapCollisionBuilder.h"

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void writeText(const std::filesystem::path& path, const std::string& content) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary);
    if (!output || !(output << content)) {
        throw std::runtime_error("failed to write map collision fixture: " + path.string());
    }
}

void writeU16(std::ofstream& output, std::uint16_t value) {
    output.put(static_cast<char>(value & 0xff));
    output.put(static_cast<char>((value >> 8) & 0xff));
}

void writeU32(std::ofstream& output, std::uint32_t value) {
    for (unsigned int byte = 0; byte < 4; ++byte) {
        output.put(static_cast<char>((value >> (byte * 8)) & 0xff));
    }
}

void writeF32(std::ofstream& output, float value) {
    writeU32(output, std::bit_cast<std::uint32_t>(value));
}

void writeJunctionO3D(const std::filesystem::path& path) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("failed to write junction O3D fixture");
    }
    output.put(static_cast<char>(0x84));
    output.put(static_cast<char>(0x19));
    output.put(static_cast<char>(3));
    output.put(static_cast<char>(0x17)); // Vertex list
    writeU16(output, 3);
    const std::array<std::array<float, 3>, 3> positions = {
        std::array<float, 3>{0.0F, 0.0F, 0.0F},
        std::array<float, 3>{1.0F, 0.0F, 0.0F},
        std::array<float, 3>{0.0F, 0.0F, 1.0F}};
    for (const auto& position : positions) {
        for (float value : position) {
            writeF32(output, value);
        }
        for (int value = 0; value < 5; ++value) {
            writeF32(output, value == 2 ? 1.0F : 0.0F);
        }
    }
    output.put(static_cast<char>(0x49)); // Triangle list
    writeU16(output, 1);
    // This winding is upward in the O3D right/up/forward frame. O3DLoader's
    // axis swap flips it; the map collision builder must reverse it again.
    writeU16(output, 0);
    writeU16(output, 2);
    writeU16(output, 1);
    writeU16(output, 0);
}

std::string splineMapText() {
    return "[version]\n12\n[terrain]\n[spline]\n"
           "test spline\n"
           "Splines/Test.sli\n"
           "1\n0\n0\n"
           "10\n2\n20\n0\n10\n0\n0\n0\n0\n0\n0\n0\n0\n0\n";
}

void runProbe() {
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "OpenBusMapCollisionBuilderProbe";
    std::filesystem::remove_all(root);
    const std::filesystem::path mapRoot = root / "map";
    const std::filesystem::path omsiRoot = root / "omsi";
    writeText(omsiRoot / "Splines" / "Test.sli",
              "Test profile\n[texture]\nroad.bmp\n[profile]\n0\n"
              "[profilepnt]\n-2\n0\n0\n0\n"
              "[profilepnt]\n2\n0\n1\n0\n");

    openbus::map::MapDefinition map;
    map.rootPath = mapRoot;
    const auto addTile = [&](int x, int y, const std::string& name) {
        openbus::map::MapTileReference tile;
        tile.x = x;
        tile.y = y;
        tile.textPath = mapRoot / name;
        writeText(tile.textPath, splineMapText());
        map.tiles.push_back(std::move(tile));
    };
    addTile(-1, -1, "center.map");
    addTile(0, -1, "neighbor.map");
    addTile(1, -1, "outside.map");
    writeText(map.tiles[1].textPath,
              splineMapText() + "[object]\n0\nSceneryobjects\\Test\\Junction.sco\n"
                                  "9\n10\n20\n4\n90\n0\n0\n0\n"
                                  "[object]\n0\nSceneryobjects\\Test\\NoCollision.sco\n"
                                  "10\n20\n20\n4\n0\n0\n0\n0\n");
    writeText(omsiRoot / "Sceneryobjects" / "Test" / "Junction.sco",
              "[collision_mesh]\njunction.o3d\n");
    writeText(omsiRoot / "Sceneryobjects" / "Test" / "NoCollision.sco",
              "[collision_mesh]\njunction.o3d\n[nocollision]\n");
    writeJunctionO3D(omsiRoot / "Sceneryobjects" / "Test" / "junction.o3d");

    const openbus::map::MapRoadCollisionResult result =
        openbus::map::buildMapRoadCollision(map, omsiRoot);
    require(result.tilesVisited == 3, "collision includes tiles beyond the spawn neighborhood");
    require(result.splineSectionsAdded == 3, "road strips are collected from every map tile");
    require(result.meshes.size() == 3, "collision meshes cover every fixture tile");
        require(result.sceneryCollisionObjectsAdded == 1,
            "authored SCO collision meshes are included and [nocollision] is respected");
    require(result.skippedProfiles == 0 && result.skippedSplines == 0,
            "valid fixture roads are not skipped");
    require(result.meshes.front().vertices.size() >= 6,
            "selected tile road mesh contains world-space vertices");
    require(std::abs(result.meshes.front().vertices[0] - (-292.0)) < 1.0e-8,
            "negative tile coordinates are included in the road world transform");
        const StaticCollisionMesh& junctionMesh = result.meshes[1];
        require(junctionMesh.vertices.size() >= 18 && junctionMesh.indices.size() >= 9,
            "junction O3D collision triangles are appended to the tile road mesh");
        const std::size_t junctionVertex = junctionMesh.vertices.size() / 3 - 3;
        require(std::abs(junctionMesh.vertices[junctionVertex * 3] - 10.0) < 1.0e-8 &&
            std::abs(junctionMesh.vertices[junctionVertex * 3 + 1] + 280.0) < 1.0e-8 &&
            std::abs(junctionMesh.vertices[junctionVertex * 3 + 2] - 4.0) < 1.0e-8,
            "junction vertices receive tile, height, and authored rotation transforms");
        const std::size_t junctionIndex = junctionMesh.indices.size() - 3;
        const auto point = [&](std::size_t index) {
            const std::size_t offset = static_cast<std::size_t>(junctionMesh.indices[index]) * 3;
            return std::array<double, 3>{junctionMesh.vertices[offset],
                                         junctionMesh.vertices[offset + 1],
                                         junctionMesh.vertices[offset + 2]};
        };
        const auto junctionFirst = point(junctionIndex);
        const auto junctionSecond = point(junctionIndex + 1);
        const auto junctionThird = point(junctionIndex + 2);
        require((junctionSecond[0] - junctionFirst[0]) *
                            (junctionThird[1] - junctionFirst[1]) -
                        (junctionSecond[1] - junctionFirst[1]) *
                            (junctionThird[0] - junctionFirst[0]) >
                    0.0,
                "junction O3D winding produces upward-facing ODE contact normals");
        const StaticCollisionMesh& mesh = result.meshes.front();
        const int first = mesh.indices[0];
        const int second = mesh.indices[1];
        const int third = mesh.indices[2];
        const double firstEdgeX = mesh.vertices[static_cast<std::size_t>(second) * 3] -
                      mesh.vertices[static_cast<std::size_t>(first) * 3];
        const double firstEdgeY = mesh.vertices[static_cast<std::size_t>(second) * 3 + 1] -
                      mesh.vertices[static_cast<std::size_t>(first) * 3 + 1];
        const double secondEdgeX = mesh.vertices[static_cast<std::size_t>(third) * 3] -
                       mesh.vertices[static_cast<std::size_t>(first) * 3];
        const double secondEdgeY = mesh.vertices[static_cast<std::size_t>(third) * 3 + 1] -
                       mesh.vertices[static_cast<std::size_t>(first) * 3 + 1];
        require(firstEdgeX * secondEdgeY - firstEdgeY * secondEdgeX > 0.0,
            "ODE road triangle winding produces upward-facing normals");

    std::filesystem::remove_all(root);
}

} // namespace

int main() {
    try {
        runProbe();
    } catch (const std::exception& error) {
        std::cerr << "map collision builder probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
