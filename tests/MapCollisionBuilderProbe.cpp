#include "MapCollisionBuilder.h"
#include "MapRoadGeometry.h"
#include "MapSplineProfile.h"

#include <ode/ode.h>

#include <algorithm>
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

void writeLongBridgeO3D(const std::filesystem::path& path) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("failed to write long bridge O3D fixture");
    }
    output.put(static_cast<char>(0x84));
    output.put(static_cast<char>(0x19));
    output.put(static_cast<char>(3));
    output.put(static_cast<char>(0x17));
    writeU16(output, 3);
    const std::array<std::array<float, 3>, 3> positions = {
        std::array<float, 3>{50.0F, 0.0F, 50.0F},
        std::array<float, 3>{650.0F, 0.0F, 50.0F},
        std::array<float, 3>{50.0F, 0.0F, 250.0F}};
    for (const auto& position : positions) {
        for (float value : position) {
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

std::string splineMapText() {
    return "[version]\n12\n[terrain]\n[spline]\n"
           "test spline\n"
           "Splines/Test.sli\n"
           "1\n0\n0\n"
           "10\n2\n20\n0\n10\n0\n0\n0\n0\n0\n0\n0\n0\n0\n";
}

void runLinkedProbe() {
    using namespace openbus::map;
    const auto root = std::filesystem::temp_directory_path() / "OpenBusMapLinkedCollisionProbe";
    std::filesystem::remove_all(root);
        const auto mapRoot = root / "map";
        const auto omsiRoot = root / "omsi";
        const std::string crossSection =
          "[texture]\nroad.bmp\n[profile]\n0\n"
          "[profilepnt]\n-2\n0.25\n0\n0.2\n[profilepnt]\n2\n0.25\n1\n0.2\n";
                writeText(omsiRoot / "Splines" / "Test.sli",
              "[halfcantwidth]\n0.06\n[heightprofile]\n-0.2\n0.2\n0\n0\n" +
                                    crossSection);
                writeText(omsiRoot / "Splines" / "Wide.sli",
              "[halfcantwidth]\n0.12\n[heightprofile]\n-0.3\n0.3\n0\n0\n" +
                                    crossSection);
    MapDefinition map;
        map.rootPath = mapRoot;
    MapTileReference aTile;
        aTile.textPath = mapRoot / "tile_0_0.map";
    MapTileReference bTile;
    bTile.x = 1;
        bTile.textPath = mapRoot / "tile_1_0.map";
    writeText(aTile.textPath,
          "[version]\n14\n[terrain]\n[spline]\n0\nSplines/Test.sli\n"
          "10\n0\n20\n290\n0\n0\n90\n10\n0\n12\n12\n200\n200\n0.5\n0.5\n0\n\n");
    writeText(bTile.textPath,
            "[version]\n14\n[terrain]\n[spline]\n0\nSplines/Wide.sli\n"
          "20\n10\n0\n0\n1.25\n0\n0\n10\n0\n-11.2\n-11.2\n-120\n-120\n-0.25\n0\n10\n\n");
    map.tiles = {aTile, bTile};
        const auto full = buildMapRoadCollision(map, omsiRoot);
        const auto first = buildMapRoadCollisionTile(map, omsiRoot, 0, 0);
        const auto second = buildMapRoadCollisionTile(map, omsiRoot, 1, 0);
    require(full.meshes.size() == 2 && first.meshes.size() == 1 && second.meshes.size() == 1,
        "linked cross-tile roads remain partitioned for independent streaming");
    require(full.meshes[0].vertices == first.meshes[0].vertices &&
        full.meshes[1].vertices == second.meshes[0].vertices &&
        full.meshes[0].indices == first.meshes[0].indices &&
        full.meshes[1].indices == second.meshes[0].indices,
        "stitched collision is identical for full-map and independent tile builds");
    const auto& firstVertices = first.meshes[0].vertices;
    const auto& secondVertices = second.meshes[0].vertices;
    require(firstVertices.size() >= 6 && secondVertices.size() >= 6,
            "linked roads contain both seam edge vertices");
    for (std::size_t coordinate = 0; coordinate < 6; ++coordinate) {
        require(std::abs(firstVertices[firstVertices.size() - 6 + coordinate] -
                         secondVertices[coordinate]) < 1.0e-8,
                "cross-tile skew/cant/gradient seam has no collision crack");
    }
    // Shared center 1.225, grade/skew lift +/-0.276, profile height 0.25,
    // and separately clamped mean cant lift +/-(-0.06 + 0.072).
    require(std::abs(secondVertices[2] - 1.187) < 1.0e-8 &&
                std::abs(secondVertices[5] - 1.763) < 1.0e-8,
            "seam height averages each authored width's cant displacement independently");
    const auto connections = loadMapRoadConnections(map, omsiRoot);
    const std::array<const MapTileReference*, 2> references = {&aTile, &bTile};
    const std::array<const StaticCollisionMesh*, 2> meshes = {
        &first.meshes[0], &second.meshes[0]};
    for (std::size_t tileIndex = 0; tileIndex < references.size(); ++tileIndex) {
        const auto& reference = *references[tileIndex];
        const auto& mesh = *meshes[tileIndex];
        const auto cached = buildMapRoadCollisionTile(map, omsiRoot, reference.x, reference.y,
                                                      &connections);
        require(cached.meshes.size() == 1 && cached.meshes[0].vertices == mesh.vertices &&
                    cached.meshes[0].indices == mesh.indices,
                "retained hydrated connections match independently loaded tile metadata");
        const auto tile = loadMapTile(reference);
        const auto& spline = tile.splines.front();
        const auto profile = loadMapSplineProfile(omsiRoot / spline.assetPath);
        const auto cantHalfWidth = mapSplineProfileCantHalfWidth(profile);
        const double expectedWidth = tileIndex == 0 ? 0.06 : 0.12;
        require(cantHalfWidth.has_value() &&
                    std::abs(*cantHalfWidth - expectedWidth) < 1.0e-9,
                "collision profiles expose distinct authored cant half-widths");
        const auto samples = tessellateMapSpline(
            reference.x * OMSI_TILE_SIZE_METERS + spline.localX,
            reference.y * OMSI_TILE_SIZE_METERS + spline.localY,
            spline.rotationDegrees, spline.length, spline.radius);
        const auto rendered = buildMapRoadSectionGeometry(
            spline, samples, profile.sections[0],
            connections.joinsFor(spline, reference.x, reference.y), cantHalfWidth);
        require(rendered.vertices.size() * 3 == mesh.vertices.size(),
                "collision retains shared renderer road vertices");
        for (std::size_t vertex = 0; vertex < rendered.vertices.size(); ++vertex) {
            require(rendered.vertices[vertex].x == mesh.vertices[vertex * 3] &&
                        rendered.vertices[vertex].y == mesh.vertices[vertex * 3 + 1] &&
                        rendered.vertices[vertex].z == mesh.vertices[vertex * 3 + 2],
                    "collision and render skew/stitching use identical XYZ geometry");
        }
        require(rendered.indices.size() == mesh.indices.size(),
                "collision retains shared renderer road triangles");
        for (std::size_t index = 0; index < rendered.indices.size(); index += 3) {
            require(rendered.indices[index] == mesh.indices[index] &&
                        rendered.indices[index + 1] == mesh.indices[index + 2] &&
                        rendered.indices[index + 2] == mesh.indices[index + 1],
                    "collision matches renderer indices with only ODE winding reversed");
        }
    }
    MapTileReference missing;
    missing.x = 99;
    missing.textPath = mapRoot / "missing.map";
    map.tiles.push_back(missing);
    require(buildMapRoadCollisionTile(map, omsiRoot, 0, 0).meshes[0].vertices == firstVertices,
        "missing unrelated tile cannot break selected-tile seam metadata loading");
    std::filesystem::remove_all(root);
}

void runNoCollisionProbe() {
    using namespace openbus::map;
    const auto root = std::filesystem::temp_directory_path() / "OpenBusMapNoCollisionProbe";
    std::filesystem::remove_all(root);
    const auto mapRoot = root / "map";
    const auto omsiRoot = root / "omsi";
    writeText(omsiRoot / "Splines" / "NoCollision.sli",
              "[nocollision]\n[texture]\nroad.bmp\n[profile]\n0\n"
              "[profilepnt]\n-2\n0\n0\n0\n"
              "[profilepnt]\n2\n0\n1\n0\n");

    MapDefinition map;
    map.rootPath = mapRoot;
    MapTileReference tile;
    tile.textPath = mapRoot / "tile_0_0.map";
    std::string tileText = splineMapText();
    const std::string oldAsset = "Splines/Test.sli";
    const std::size_t assetOffset = tileText.find(oldAsset);
    require(assetOffset != std::string::npos, "collision fixture contains its spline asset path");
    tileText.replace(assetOffset, oldAsset.size(), "Splines/NoCollision.sli");
    writeText(tile.textPath, tileText);
    map.tiles.push_back(tile);

    const MapSplineProfile profile = loadMapSplineProfile(omsiRoot / "Splines" / "NoCollision.sli");
    require(profile.noCollision && profile.sections.size() == 1 &&
                profile.sections.front().points.size() == 2,
            "[nocollision] profile remains parsed with its visible cross-section");
    const MapRoadCollisionResult result = buildMapSplineCollisionTile(map, omsiRoot, 0, 0);
    require(result.tilesVisited == 1 && result.skippedProfiles == 0 &&
                result.skippedSplines == 0 && result.splineSectionsAdded == 0 &&
                result.meshes.empty(),
            "[nocollision] spline profile contributes no road collision geometry");
    std::filesystem::remove_all(root);
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
    writeText(map.tiles[2].textPath,
              splineMapText() + "[object]\n0\nSceneryobjects\\Test\\Junction.sco\n"
                                  "11\n50\n20\n4\n0\n0\n10\n0\n"
                                  "[object]\n0\nSceneryobjects\\Test\\Junction.sco\n"
                                  "12\n80\n20\n4\n0\n10\n0\n0\n");
    writeText(omsiRoot / "Sceneryobjects" / "Test" / "Junction.sco",
              "[collision_mesh]\njunction.o3d\n");
    writeText(omsiRoot / "Sceneryobjects" / "Test" / "NoCollision.sco",
              "[collision_mesh]\njunction.o3d\n[nocollision]\n");
    writeJunctionO3D(omsiRoot / "Sceneryobjects" / "Test" / "junction.o3d");

    const openbus::map::MapRoadCollisionResult result =
        openbus::map::buildMapRoadCollision(map, omsiRoot);
    const openbus::map::MapRoadCollisionResult selectedTile =
        openbus::map::buildMapRoadCollisionTile(map, omsiRoot, 0, -1);
    const openbus::map::MapRoadCollisionResult selectedSplineTile =
        openbus::map::buildMapSplineCollisionTile(map, omsiRoot, 0, -1);
    require(selectedTile.tilesVisited == 1 && selectedTile.meshes.size() == 1,
            "streaming collision builder processes only the requested tile");
    require(selectedSplineTile.tilesVisited == 1 && selectedSplineTile.meshes.size() == 1 &&
                selectedSplineTile.meshes[0].vertices.size() <=
                    selectedTile.meshes[0].vertices.size() &&
                selectedSplineTile.meshes[0].indices.size() <=
                    selectedTile.meshes[0].indices.size() &&
                std::equal(selectedSplineTile.meshes[0].vertices.begin(),
                           selectedSplineTile.meshes[0].vertices.end(),
                           selectedTile.meshes[0].vertices.begin()) &&
                std::equal(selectedSplineTile.meshes[0].indices.begin(),
                           selectedSplineTile.meshes[0].indices.end(),
                           selectedTile.meshes[0].indices.begin()),
            "spline-only streaming preserves the exact renderer-matched road collision");
    require(openbus::map::buildMapRoadCollisionTile(map, omsiRoot, 99, 99).tilesVisited == 0,
            "streaming collision builder ignores coordinates absent from the map manifest");
    require(result.tilesVisited == 3, "collision includes tiles beyond the spawn neighborhood");
    require(result.splineSectionsAdded == 3, "road strips are collected from every map tile");
    require(result.meshes.size() == 3, "collision meshes cover every fixture tile");
        require(result.sceneryCollisionObjectsAdded == 3,
            "authored SCO collision meshes include rotated objects and respect [nocollision]");
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
            const StaticCollisionMesh& rotatedJunctionMesh = result.meshes[2];
            const std::size_t rotatedJunctionVertex = rotatedJunctionMesh.vertices.size() / 3 - 6;
            const double bankRadians = 10.0 * std::acos(-1.0) / 180.0;
            const std::size_t bankedEastVertex = rotatedJunctionVertex + 1;
            require(std::abs(rotatedJunctionMesh.vertices[bankedEastVertex * 3] -
                     (350.0 + std::cos(bankRadians))) < 1.0e-8 &&
                    std::abs(rotatedJunctionMesh.vertices[bankedEastVertex * 3 + 1] + 280.0) <
                    1.0e-8 &&
                    std::abs(rotatedJunctionMesh.vertices[bankedEastVertex * 3 + 2] -
                         (4.0 + std::sin(bankRadians))) < 1.0e-8,
                "collision bank uses the same Y-axis sign as rendered scenery");
            const std::size_t pitchedJunctionVertex = rotatedJunctionMesh.vertices.size() / 3 - 3;
            const std::size_t pitchedNorthVertex = pitchedJunctionVertex + 2;
            require(std::abs(rotatedJunctionMesh.vertices[pitchedNorthVertex * 3] - 380.0) < 1.0e-8 &&
                    std::abs(rotatedJunctionMesh.vertices[pitchedNorthVertex * 3 + 1] -
                         (-280.0 + std::cos(bankRadians))) < 1.0e-8 &&
                    std::abs(rotatedJunctionMesh.vertices[pitchedNorthVertex * 3 + 2] -
                         (4.0 - std::sin(bankRadians))) < 1.0e-8,
                "collision pitch remains around X and matches rendered scenery");
        require(dInitODE2(0) != 0, "ODE initializes for the junction contact regression");
        require(dAllocateODEDataForThread(dAllocateFlagCollisionData) != 0,
            "ODE allocates collision data for the junction contact regression");
        dTriMeshDataID junctionData = dGeomTriMeshDataCreate();
        const std::vector<double> junctionContactVertices = {
            junctionFirst[0], junctionFirst[1], junctionFirst[2],
            junctionSecond[0], junctionSecond[1], junctionSecond[2],
            junctionThird[0], junctionThird[1], junctionThird[2]};
        const std::vector<int> junctionContactIndices = {0, 1, 2};
        dGeomTriMeshDataBuildDouble(junctionData, junctionContactVertices.data(), 3 * sizeof(double), 3,
                                    junctionContactIndices.data(), 3, 3 * sizeof(int));
        dGeomID junctionGeometry = dCreateTriMesh(nullptr, junctionData, nullptr, nullptr, nullptr);
        dGeomID probeSphere = dCreateSphere(nullptr, 0.25);
        dGeomSetPosition(probeSphere, (junctionFirst[0] + junctionSecond[0] + junctionThird[0]) / 3.0,
                         (junctionFirst[1] + junctionSecond[1] + junctionThird[1]) / 3.0, 4.1);
        dContactGeom contacts[4] = {};
        const int contactCount = dCollide(probeSphere, junctionGeometry, 4, contacts,
                                          sizeof(dContactGeom));
        require(contactCount > 0,
                "ODE detects a body above the transformed authored junction collision surface");
        dGeomDestroy(probeSphere);
        dGeomDestroy(junctionGeometry);
        dGeomTriMeshDataDestroy(junctionData);
        dCloseODE();
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

        const std::filesystem::path longRoot = root / "long_bridge";
        const std::filesystem::path longMapRoot = longRoot / "map";
        const std::filesystem::path longOmsiRoot = longRoot / "omsi";
        using namespace openbus::map;
        MapDefinition longMap;
        longMap.rootPath = longMapRoot;
        for (int tileX = 0; tileX < 3; ++tileX) {
            MapTileReference tile;
            tile.x = tileX;
            tile.textPath = longMapRoot / ("tile_" + std::to_string(tileX) + "_0.map");
            writeText(tile.textPath, "[version]\n14\n[terrain]\n");
            longMap.tiles.push_back(std::move(tile));
        }
        writeText(longMap.tiles[0].textPath,
                  "[version]\n14\n[terrain]\n[object]\n0\n"
                  "Sceneryobjects\\Test\\Bridge.sco\n1\n10\n10\n0\n0\n0\n0\n0\n");
        writeText(longOmsiRoot / "Sceneryobjects" / "Test" / "Bridge.sco",
                  "[model]\nbridge.cfg\n[absheight]\n");
        writeText(longOmsiRoot / "Sceneryobjects" / "Test" / "bridge.cfg",
                  "[collision_mesh]\nbridge.o3d\n");
        writeLongBridgeO3D(longOmsiRoot / "Sceneryobjects" / "Test" / "bridge.o3d");
        const auto longScenery = buildMapSceneryCollisionIndex(longMap, longOmsiRoot);
        require(longScenery->objectsByTile.contains((std::uint64_t{1} << 32U) | 0U) &&
                    longScenery->objectsByTile.contains((std::uint64_t{2} << 32U) | 0U),
                "long bridge collision is indexed into every tile crossed by its mesh");
        const auto bridgeTile = buildMapRoadCollisionTile(longMap, longOmsiRoot, 1, 0, nullptr,
                                                           longScenery.get());
        require(bridgeTile.meshes.size() == 1 && !bridgeTile.meshes[0].indices.empty(),
                "streamed collision tile receives the bridge surface beyond its placement tile");
        for (std::size_t vertex = 0; vertex + 2 < bridgeTile.meshes[0].vertices.size(); vertex += 3) {
            require(bridgeTile.meshes[0].vertices[vertex] >= 300.0 - 1.0e-8 &&
                        bridgeTile.meshes[0].vertices[vertex] <= 600.0 + 1.0e-8 &&
                        bridgeTile.meshes[0].vertices[vertex + 1] >= -1.0e-8 &&
                        bridgeTile.meshes[0].vertices[vertex + 1] <= 300.0 + 1.0e-8,
                    "streamed bridge collision triangles are clipped to their owning tile");
        }

    std::filesystem::remove_all(root);
}

} // namespace

int main() {
    try {
        runProbe();
        runLinkedProbe();
        runNoCollisionProbe();
    } catch (const std::exception& error) {
        std::cerr << "map collision builder probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
