#include "map/MapCollisionBuilder.h"
#include "map/MapRoadGeometry.h"
#include "map/MapSplineProfile.h"
#include "ProbeTempDirectory.h"

#include <ode/ode.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
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

void writeSurfaceBitmap(const std::filesystem::path& path, bool constant = false) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("failed to write surface-height fixture");
    }
    const auto writeU16 = [&](std::uint16_t value) {
        output.put(static_cast<char>(value & 0xffU));
        output.put(static_cast<char>((value >> 8) & 0xffU));
    };
    const auto writeU32 = [&](std::uint32_t value) {
        for (unsigned int byte = 0; byte < 4; ++byte) {
            output.put(static_cast<char>((value >> (byte * 8)) & 0xffU));
        }
    };
    output.put('B');
    output.put('M');
    writeU32(62);
    writeU16(0);
    writeU16(0);
    writeU32(54);
    writeU32(40);
    writeU32(2);
    writeU32(1);
    writeU16(1);
    writeU16(24);
    writeU32(0);
    writeU32(8);
    writeU32(0);
    writeU32(0);
    writeU32(0);
    writeU32(0);
    const std::array<unsigned char, 8> row =
        constant ? std::array<unsigned char, 8>{0, 0, 0, 0, 0, 0, 0, 0}
                 : std::array<unsigned char, 8>{0, 0, 0, 255, 255, 255, 0, 0};
    output.write(reinterpret_cast<const char*>(row.data()),
                 static_cast<std::streamsize>(row.size()));
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

void writePoleO3D(const std::filesystem::path& path) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("failed to write pole O3D fixture");
    }
    output.put(static_cast<char>(0x84));
    output.put(static_cast<char>(0x19));
    output.put(static_cast<char>(3));
    output.put(static_cast<char>(0x17));
    writeU16(output, 5);
    const std::array<std::array<float, 3>, 5> positions = {
        std::array<float, 3>{0.9F, 0.5F, 0.3F},
        std::array<float, 3>{1.1F, 0.5F, 0.3F},
        std::array<float, 3>{1.0F, 4.5F, 0.3F},
        std::array<float, 3>{4.0F, 4.5F, 0.3F},
        std::array<float, 3>{4.0F, 4.0F, 0.3F}};
    for (const auto& position : positions) {
        for (float value : position) {
            writeF32(output, value);
        }
        for (int value = 0; value < 5; ++value) {
            writeF32(output, value == 2 ? 1.0F : 0.0F);
        }
    }
    output.put(static_cast<char>(0x49));
    writeU16(output, 2);
    writeU16(output, 0);
    writeU16(output, 2);
    writeU16(output, 1);
    writeU16(output, 0);
    writeU16(output, 2);
    writeU16(output, 3);
    writeU16(output, 4);
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
    const auto root = openbus::probe::makeProbeDirectory("OpenBusMapLinkedCollisionProbe");
        const auto mapRoot = root / "map";
        const auto omsiRoot = root / "omsi";
        const std::string crossSection =
          "[TEXTURE]\nroad.bmp\n-<DISABLED>-\n[nocollision]\n-<ENABLED>-\n[PROFILE]\n0\n"
          "[PROFILEPNT]\n-2\n0.25\n0\n0.2\n[PROFILEPNT]\n2\n0.25\n1\n0.2\n";
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
    openbus::probe::removeProbeDirectory(root);
}

void runNoCollisionProbe() {
    using namespace openbus::map;
    const auto root = openbus::probe::makeProbeDirectory("OpenBusMapNoCollisionProbe");
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
    openbus::probe::removeProbeDirectory(root);
}

void runSurfaceHeightMapProbe() {
    using namespace openbus::map;
    const auto root = openbus::probe::makeProbeDirectory("OpenBusMapSurfaceHeightMapProbe");
    const auto mapRoot = root / "map";
    const auto omsiRoot = root / "omsi";
    const auto profilePath = omsiRoot / "Splines" / "Markings.sli";
    const auto texturePath = omsiRoot / "Texture" / "Markings" / "White.dds";
    const auto surfacePath = omsiRoot / "Texture" / "Markings" / "White.dds.surf";
    writeText(profilePath, "[texture]\nMarkings/White.dds\n[profile]\n0\n"
                           "[profilepnt]\n-2\n0\n0\n0.1\n"
                           "[profilepnt]\n2\n0\n1\n0.1\n");
    writeText(texturePath, "fixture texture");
    writeSurfaceBitmap(surfacePath);

    MapDefinition map;
    map.rootPath = mapRoot;
    MapTileReference tile;
    tile.textPath = mapRoot / "tile_0_0.map";
    std::string tileText = splineMapText();
    const std::string originalProfile = "Splines/Test.sli";
    const std::size_t profileOffset = tileText.find(originalProfile);
    require(profileOffset != std::string::npos, "surface fixture has a spline profile path");
    tileText.replace(profileOffset, originalProfile.size(), "Splines/Markings.sli");
    writeText(tile.textPath, tileText);
    map.tiles.push_back(tile);

    const MapRoadCollisionResult result = buildMapSplineCollisionTile(map, omsiRoot, 0, 0);
    require(result.splineSectionsAdded == 1 && result.meshes.size() == 1,
            "surface height map retains road marking collision");
    const StaticCollisionMesh& mesh = result.meshes.front();
    double minimumHeight = std::numeric_limits<double>::infinity();
    double maximumHeight = -std::numeric_limits<double>::infinity();
    for (std::size_t index = 2; index < mesh.vertices.size(); index += 3) {
        minimumHeight = std::min(minimumHeight, mesh.vertices[index]);
        maximumHeight = std::max(maximumHeight, mesh.vertices[index]);
    }
    require(mesh.vertices.size() > 3 * (11 * 2),
            "surface height map subdivides road collision to sample texture details");
    require(!mesh.surfaceHeightMapIndices.empty() && mesh.surfaceHeightMapIndices.size() % 3 == 0 &&
                std::all_of(mesh.surfaceHeightMapIndices.begin(),
                            mesh.surfaceHeightMapIndices.end(),
                            [&](int index) {
                                return index >= 0 &&
                                       static_cast<std::size_t>(index) < mesh.vertices.size() / 3;
                            }),
            "surface-height-map triangles are separately identified for the collision outline");
    if (!(minimumHeight < 1.985 && maximumHeight > 2.015)) {
        throw std::runtime_error("surface-map collision heights are " +
                                 std::to_string(minimumHeight) + " to " +
                                 std::to_string(maximumHeight));
    }

    writeText(tile.textPath,
              "[version]\n12\n[terrain]\n[spline]\ntest spline\nSplines/Markings.sli\n"
              "1\n0\n0\n10\n2\n20\n0\n10\n0\n0\n0\n0\n0\n0\nmirror\n");
    const MapRoadCollisionResult mirroredResult = buildMapSplineCollisionTile(map, omsiRoot, 0, 0);
    require(mirroredResult.meshes.size() == 1 &&
                !mirroredResult.meshes.front().surfaceHeightMapIndices.empty(),
            "mirrored surface-height-map road keeps tagged collision geometry");
    const StaticCollisionMesh& mirroredMesh = mirroredResult.meshes.front();
    for (std::size_t index = 0; index + 2 < mirroredMesh.indices.size(); index += 3) {
        const std::size_t first = static_cast<std::size_t>(mirroredMesh.indices[index]) * 3;
        const std::size_t second = static_cast<std::size_t>(mirroredMesh.indices[index + 1]) * 3;
        const std::size_t third = static_cast<std::size_t>(mirroredMesh.indices[index + 2]) * 3;
        const double firstEdgeX = mirroredMesh.vertices[second] - mirroredMesh.vertices[first];
        const double firstEdgeY =
            mirroredMesh.vertices[second + 1] - mirroredMesh.vertices[first + 1];
        const double secondEdgeX = mirroredMesh.vertices[third] - mirroredMesh.vertices[first];
        const double secondEdgeY =
            mirroredMesh.vertices[third + 1] - mirroredMesh.vertices[first + 1];
        require(firstEdgeX * secondEdgeY - firstEdgeY * secondEdgeX > 0.0,
                "mirrored .surf collision winding remains upward-facing for ODE");
    }
    writeSurfaceBitmap(surfacePath, true);
    writeText(tile.textPath, tileText);
    const MapRoadCollisionResult constantResult = buildMapSplineCollisionTile(map, omsiRoot, 0, 0);
    require(constantResult.meshes.size() == 1, "constant surface road retains collision");
    const auto& constantMesh = constantResult.meshes.front();
    require(constantMesh.vertices.size() == 3 * 12 &&
                constantMesh.indices.size() == 5 * 6 &&
                constantMesh.surfaceHeightMapIndices == constantMesh.indices,
            "constant surface uses the original road tessellation and remains tagged");
    for (std::size_t index = 2; index < constantMesh.vertices.size(); index += 3) {
        require(std::abs(constantMesh.vertices[index] - 1.98) < 1.0e-9,
                "constant surface retains its exact height offset");
    }
    writeText(tile.textPath,
              "[version]\n12\n[terrain]\n[spline]\ntest spline\nSplines/Markings.sli\n"
              "1\n0\n0\n10\n2\n20\n0\n10\n20\n0\n0\n0\n0\n0\n0\n0\n0\n");
    const auto curvedResult = buildMapSplineCollisionTile(map, omsiRoot, 0, 0);
    require(curvedResult.meshes.size() == 1 &&
                curvedResult.meshes.front().vertices.size() > constantMesh.vertices.size(),
            "constant surface keeps subdivisions when road quads are not affine");
    openbus::probe::removeProbeDirectory(root);
}

void runDenseTileCapacityProbe() {
    using namespace openbus::map;
    const auto root = openbus::probe::makeProbeDirectory("OpenBusDenseCollisionTileProbe");
    const auto mapRoot = root / "map";
    const auto omsiRoot = root / "omsi";
    const auto profilePath = omsiRoot / "Splines" / "Dense.sli";
    writeSurfaceBitmap(omsiRoot / "Splines" / "texture" / "road.bmp.surf");
    std::string profile = "[texture]\nroad.bmp\n";
    for (int section = 0; section < 7; ++section) {
        profile += "[profile]\n0\n[profilepnt]\n-2\n0\n0\n5\n"
                   "[profilepnt]\n2\n0\n1\n5\n";
    }
    writeText(profilePath, profile);
    const std::string spline =
        "[spline]\ntest spline\nSplines/Dense.sli\n"
        "1\n0\n0\n10\n2\n20\n0\n4096\n0\n0\n0\n0\n0\n0\n0\n0\n0\n";
    MapDefinition map;
    map.rootPath = mapRoot;
    MapTileReference tile;
    tile.textPath = mapRoot / "tile_0_0.map";
    writeText(tile.textPath, "[version]\n12\n[terrain]\n" + spline);
    map.tiles.push_back(tile);

    const auto result = buildMapSplineCollisionTile(map, omsiRoot, 0, 0);
    std::size_t vertices = 0;
    std::size_t triangles = 0;
    for (const auto& mesh : result.meshes) {
        require(mesh.vertices.size() / 3 <= 1'000'000,
                "each streamed road mesh respects the vertex limit");
        require(mesh.surfaceHeightMapIndices == mesh.indices,
                "split meshes retain their surface-height debug triangles");
        vertices += mesh.vertices.size() / 3;
        triangles += mesh.indices.size() / 3;
    }
    require(result.meshes.size() >= 2 && vertices > 1'000'000 &&
                result.splineSectionsAdded == 7 && result.skippedSplines == 0 &&
                triangles == 7 * 2048 * 4 * 16 * 2,
            "dense tile retains every road section when exceeding one mesh's vertex limit");
    openbus::probe::removeProbeDirectory(root);
}

void runProbe() {
    const std::filesystem::path root =
        openbus::probe::makeProbeDirectory("OpenBusMapCollisionBuilderProbe");
    const std::filesystem::path mapRoot = root / "map";
    const std::filesystem::path omsiRoot = root / "omsi";
    writeText(omsiRoot / "Splines" / "Test.sli", "Test profile\n[texture]\nroad.bmp\n[profile]\n0\n"
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

        MapDefinition poleMap;
        poleMap.rootPath = mapRoot;
        MapTileReference poleTile;
        poleTile.x = 0;
        poleTile.y = 0;
        poleTile.textPath = mapRoot / "pole.map";
        writeText(poleTile.textPath,
                  "[version]\n12\n[terrain]\n[object]\n0\n"
                  "Sceneryobjects\\Test\\Pole.sco\n77\n50\n50\n5\n0\n0\n0\n0\n"
                  "[object]\n0\nSceneryobjects\\Test\\BoundingBoxPole.sco\n"
                  "78\n60\n50\n5\n0\n0\n0\n0\n"
                  "[object]\n0\nSceneryobjects\\Test\\Pole.sco\n"
                  "79\n299.5\n50\n5\n0\n0\n0\n0\n"
                  "[object]\n0\nSceneryobjects\\Test\\BoundingBoxPole.sco\n"
                  "80\n90\n50\n5\n90\n0\n0\n0\n");
        poleMap.tiles.push_back(poleTile);
        MapTileReference adjacentPoleTile;
        adjacentPoleTile.x = 1;
        adjacentPoleTile.y = 0;
        adjacentPoleTile.textPath = mapRoot / "pole_adjacent.map";
        writeText(adjacentPoleTile.textPath, "[version]\n12\n[terrain]\n");
        poleMap.tiles.push_back(adjacentPoleTile);
        writeText(omsiRoot / "Sceneryobjects" / "Test" / "Pole.sco",
                  "[model]\npole.cfg\n[crashmode_pole]\n0.25\n1.2\n");
        writeText(omsiRoot / "Sceneryobjects" / "Test" / "BoundingBoxPole.sco",
                  "[model]\nboundingboxpole.cfg\n[crashmode_pole]\n0.25\n1.2\n");
        writeText(omsiRoot / "Sceneryobjects" / "Test" / "boundingboxpole.cfg",
                  "[boundingbox]\n0.4\n0.4\n4\n0.8\n-0.5\n2\n");
        writeText(omsiRoot / "Sceneryobjects" / "Test" / "pole.cfg",
                  "[collision_mesh]\npole.o3d\n");
        writePoleO3D(omsiRoot / "Sceneryobjects" / "Test" / "pole.o3d");
        const auto poleCollisions = buildMapPoleCollisionTile(poleMap, omsiRoot, 0, 0);
        require(
            poleCollisions.size() == 3 && poleCollisions.front().mapObjectId == 77 &&
                poleCollisions.front().radiusMeters > 0.0 &&
                poleCollisions.front().radiusMeters <= 0.12 &&
                std::hypot(
                    poleCollisions.front().topPosition[0] - poleCollisions.front().basePosition[0],
                    poleCollisions.front().topPosition[1] - poleCollisions.front().basePosition[1],
                    poleCollisions.front().topPosition[2] -
                        poleCollisions.front().basePosition[2]) > 0.1 &&
                poleCollisions.front().resistanceRadiansPerTonneMeterPerSecond == 0.25 &&
                poleCollisions.front().breakAngleRadians == 1.2 &&
                poleCollisions[1].mapObjectId == 78 &&
                poleCollisions[1].radiusMeters > 0.0 &&
                std::hypot(
                    poleCollisions[1].topPosition[0] - poleCollisions[1].basePosition[0],
                    poleCollisions[1].topPosition[1] - poleCollisions[1].basePosition[1],
                    poleCollisions[1].topPosition[2] - poleCollisions[1].basePosition[2]) >
                    0.1,
            "[crashmode_pole] produces streamed collisions from mesh or bounding-box bounds");
        const auto& meshPole = poleCollisions.front();
        const auto& boxPole = poleCollisions[1];
        require(std::abs(meshPole.basePosition[0] - 51.0) < 1.0e-5 &&
                    std::abs(meshPole.basePosition[1] - 50.3) < 1.0e-5 &&
                    std::abs(meshPole.basePosition[2] - 5.5) < 1.0e-5 &&
                    std::abs(meshPole.topPosition[0] - 51.0) < 1.0e-5 &&
                    std::abs(meshPole.topPosition[2] - 9.5) < 1.0e-5 &&
                    std::abs(meshPole.radiusMeters - 0.1) < 1.0e-5,
                "pole shaft center and radius ignore the offset lamp arm");
        require(std::abs(boxPole.basePosition[0] - 60.8) < 1.0e-8 &&
                    std::abs(boxPole.basePosition[1] - 49.5) < 1.0e-8 &&
                    std::abs(boxPole.basePosition[2] - 5.0) < 1.0e-8 &&
                    std::abs(boxPole.topPosition[2] - 9.0) < 1.0e-8 &&
                    std::abs(boxPole.radiusMeters - 0.2) < 1.0e-8,
                "authored bounding box uses z as vertical and retains its offset");
        const auto& rotatedPole = poleCollisions.back();
        require(rotatedPole.mapObjectId == 80 &&
                    std::abs(rotatedPole.basePosition[0] - 89.5) < 1.0e-8 &&
                    std::abs(rotatedPole.basePosition[1] - 49.2) < 1.0e-8 &&
                    std::abs(rotatedPole.basePosition[2] - 5.0) < 1.0e-8 &&
                    std::abs(rotatedPole.topPosition[2] - 9.0) < 1.0e-8,
                "rotated pole bounds use the same placement transform as scenery");
        const auto adjacentPoles = buildMapPoleCollisionTile(poleMap, omsiRoot, 1, 0);
        require(adjacentPoles.size() == 1 && adjacentPoles.front().mapObjectId == 79 &&
                    adjacentPoles.front().tileX == 0 && adjacentPoles.front().tileY == 0 &&
                    std::abs(adjacentPoles.front().basePosition[0] - 300.5) < 1.0e-5,
                "cross-tile pole keeps its source placement identity for rendering");
        const auto poleTileCollision = buildMapRoadCollisionTile(poleMap, omsiRoot, 0, 0);
        require(poleTileCollision.poles.size() == 3 &&
                    poleTileCollision.sceneryCollisionObjectsAdded == 0 &&
                    poleTileCollision.meshes.empty(),
                "mesh and bounding-box poles stay dynamic instead of entering the static mesh");

        openbus::probe::removeProbeDirectory(root);
}

void runSceneryGeometryFallbackProbe() {
    using namespace openbus::map;
    const auto root = openbus::probe::makeProbeDirectory("OpenBusSceneryGeometryFallbackProbe");
    const auto mapRoot = root / "map";
    const auto omsiRoot = root / "omsi";
    const auto sceneryRoot = omsiRoot / "Sceneryobjects" / "Test";
    writeJunctionO3D(sceneryRoot / "model" / "model.o3d");

    writeText(sceneryRoot / "SurfaceFallback.sco",
              "[mesh]\nmodel.o3d\n[nocollision]\n[surface]\n");
    writeText(sceneryRoot / "GeometryFallback.sco", "[mesh]\nmodel.o3d\n");
    writeText(sceneryRoot / "BoundingBox.sco",
              "[boundingbox]\n2\n4\n6\n1\n0\n0\n[mesh]\nmodel.o3d\n");
    writeText(sceneryRoot / "NoSurfaceNoCollision.sco",
              "[nocollision]\n[mesh]\nmodel.o3d\n");
    writeText(sceneryRoot / "BrokenCollisionFallback.sco",
              "[collision_mesh]\nbroken.o3d\n[mesh]\nmodel.o3d\n");
    writeText(sceneryRoot / "model" / "broken.o3d", "not an O3D model");

    MapDefinition map;
    map.rootPath = mapRoot;
    MapTileReference tile;
    tile.textPath = mapRoot / "tile_0_0.map";
    writeText(tile.textPath,
              "[version]\n12\n[terrain]\n"
              "[object]\n0\nSceneryobjects\\Test\\SurfaceFallback.sco\n"
              "1\n50\n50\n4\n0\n0\n0\n0\n"
              "[object]\n0\nSceneryobjects\\Test\\GeometryFallback.sco\n"
              "2\n60\n50\n4\n0\n0\n0\n0\n"
              "[object]\n0\nSceneryobjects\\Test\\BoundingBox.sco\n"
              "3\n70\n50\n4\n0\n0\n0\n0\n"
              "[object]\n0\nSceneryobjects\\Test\\NoSurfaceNoCollision.sco\n"
              "4\n80\n50\n4\n0\n0\n0\n0\n"
              "[object]\n0\nSceneryobjects\\Test\\BrokenCollisionFallback.sco\n"
              "5\n90\n50\n4\n0\n0\n0\n0\n");
    map.tiles.push_back(tile);

    const auto index = buildMapSceneryCollisionIndex(map, omsiRoot);
    require(index->objects.size() == 4,
            "surface and ordinary scenery use model or box fallback while no-surface nocollision is honored");
    const auto box = std::find_if(index->objects.begin(), index->objects.end(),
                                  [](const MapSceneryCollisionObject& object) {
                                      return object.mapObjectId == 3;
                                  });
    require(box != index->objects.end() &&
                box->localBounds == std::array<double, 6>{0.0, -2.0, -3.0, 2.0, 2.0, 3.0},
            "static bounding-box collision uses map axes");
    const auto result = buildMapRoadCollisionTile(map, omsiRoot, 0, 0, nullptr, index.get());
    require(result.sceneryCollisionObjectsAdded == 4 && result.meshes.size() == 1 &&
                result.meshes.front().indices.size() == 45,
            "fallback model meshes and authored bounding boxes contribute scenery collision, including after a bad collision mesh");
    require(dInitODE2(0) != 0, "ODE initializes for bounding-box contact regression");
    require(dAllocateODEDataForThread(dAllocateFlagCollisionData) != 0,
            "ODE allocates bounding-box collision data");
    const auto& collision = result.meshes.front();
    dTriMeshDataID data = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildDouble(data, collision.vertices.data(), 3 * sizeof(double),
                                static_cast<int>(collision.vertices.size() / 3),
                                collision.indices.data(),
                                static_cast<int>(collision.indices.size()), 3 * sizeof(int));
    dGeomID geometry = dCreateTriMesh(nullptr, data, nullptr, nullptr, nullptr);
    dGeomID sphere = dCreateSphere(nullptr, 0.25);
    dGeomSetPosition(sphere, 71.0, 50.0, 7.1);
    dContactGeom contacts[4] = {};
    require(dCollide(sphere, geometry, 4, contacts, sizeof(dContactGeom)) > 0,
            "bounding-box top face has outward collision winding");
    dGeomDestroy(sphere);
    dGeomDestroy(geometry);
    dGeomTriMeshDataDestroy(data);
    dCloseODE();
    openbus::probe::removeProbeDirectory(root);
}

} // namespace

int main() {
    try {
        runProbe();
        runLinkedProbe();
        runNoCollisionProbe();
        runSurfaceHeightMapProbe();
        runDenseTileCapacityProbe();
        runSceneryGeometryFallbackProbe();
    } catch (const std::exception& error) {
        std::cerr << "map collision builder probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
