#include "MapCollisionBuilder.h"

#include <cmath>
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

    const openbus::map::MapRoadCollisionResult result =
        openbus::map::buildSpawnRoadCollision(map, 0, omsiRoot);
    require(result.tilesVisited == 2, "only the spawn tile and immediate neighbor are visited");
    require(result.splineSectionsAdded == 2, "one road strip is collected from each selected tile");
    require(result.meshes.size() == 2, "collision meshes are bounded by selected tiles");
    require(result.skippedProfiles == 0 && result.skippedSplines == 0,
            "valid fixture roads are not skipped");
    require(result.meshes.front().vertices.size() >= 6,
            "selected tile road mesh contains world-space vertices");
    require(std::abs(result.meshes.front().vertices[0] - (-292.0)) < 1.0e-8,
            "negative tile coordinates are included in the road world transform");
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
