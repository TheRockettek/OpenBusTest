#include "map/MapRoadRenderGeometry.h"
#include "map/MapSurfaceGeometry.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void near(double actual, double expected, const char* message) {
    require(std::isfinite(actual) && std::abs(actual - expected) < 1.0e-4, message);
}

template <typename Function> void requireInvalid(Function function) {
    try {
        function();
    } catch (const std::invalid_argument&) {
        return;
    }
    throw std::runtime_error("Invalid surface input was accepted");
}

} // namespace

int main() {
    try {
        using namespace openbus::map;
        const auto terrain = prepareMapTerrainMesh({2, {-2, -1, 0, 1, 2, 3, 4, 5, 6}}, 10);
        require(terrain.vertices.size() == 9 && terrain.indices.size() == 24,
                "Terrain grid dimensions changed");
        require(terrain.indices == std::vector<std::uint32_t>{0, 4, 1, 0, 3, 4, 1, 5, 2, 1, 4, 5,
                                                              3, 7, 4, 3, 6, 7, 4, 8, 5, 4, 7, 8},
                "Terrain winding or indexing changed");
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t column = 0; column < 3; ++column) {
                const auto& v = terrain.vertices[row * 3 + column];
                near(v.x, column * 5.0, "Terrain X");
                near(v.y, row * 5.0, "Terrain Y");
                near(v.z, static_cast<double>(row * 3 + column) - 2.0, "Terrain height");
                near(v.u, static_cast<double>(column), "Terrain U");
                near(v.v, static_cast<double>(row), "Terrain V");
                near(v.nx, 0, "Terrain normal X");
                near(v.ny, 0, "Terrain normal Y");
                near(v.nz, 1, "Terrain normal Z");
            }
        }
        near(terrain.minimumHeight, -2, "Terrain minimum");
        near(terrain.maximumHeight, 6, "Terrain maximum");
        requireInvalid([] { (void)prepareMapTerrainMesh({0, {}}, 1); });
        requireInvalid([] { (void)prepareMapTerrainMesh({1, {0, 0, 0}}, 1); });
        requireInvalid([] { (void)prepareMapTerrainMesh({1, {0, 0, 0, 0}}, 0); });
        requireInvalid([] {
            (void)prepareMapTerrainMesh({1, {0, 0, 0, std::numeric_limits<float>::infinity()}}, 1);
        });

        const WaterData water{{WaterSurface{{1, 2, 3, 4}}, WaterSurface{{-4, -3, -2, -1}}}};
        const auto mesh = prepareMapWaterMesh(water, -2, 3, 10);
        require(mesh.vertices.size() == 8 &&
                    mesh.indices == std::vector<std::uint32_t>{0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7},
                "Water indexing changed");
        constexpr double x[] = {0, 10, 10, 0};
        constexpr double y[] = {0, 0, 10, 10};
        constexpr double z[] = {1, 2, 4, 3};
        for (std::size_t i = 0; i < 4; ++i) {
            const auto& v = mesh.vertices[i];
            near(v.x, x[i], "Water local X");
            near(v.y, y[i], "Water local Y");
            near(v.z, z[i], "Water corner height");
            near(v.u, x[i] / 10, "Water U");
            near(v.v, y[i] / 10, "Water V");
            near(v.nz, 1, "Water normal");
        }
        near(mesh.minimumHeight, -4, "Water minimum");
        near(mesh.maximumHeight, 4, "Water maximum");
        near(mesh.meanHeight, 0, "Water mean");
        const auto empty = prepareMapWaterMesh({}, 0, 0, 10);
        require(empty.vertices.empty() && empty.indices.empty(), "Empty water is not empty");
        requireInvalid([] { (void)prepareMapWaterMesh({}, 0, 0, -1); });
        requireInvalid([] {
            (void)prepareMapWaterMesh(
                {{WaterSurface{{0, 0, 0, std::numeric_limits<float>::quiet_NaN()}}}}, 0, 0, 1);
        });

        MapSplinePlacement spline;
        spline.geometryValid = true;
        spline.length = 10;
        MapSplineProfileSection section;
        section.points = {{-2, 0, 0, 0.1}, {2, 0, 1, 0.1}};
        MapCoordinates coordinates;
        RoadMeshGeometry road;
        road.tileX = -2;
        road.tileY = 3;
        const auto centerline = tessellateMapSpline(-600, 900, 0, spline.length, 0);
        const auto source = buildMapRoadSectionGeometry(spline, centerline, section, {},
                                                        std::nullopt, coordinates.tile(-2, 3));
        const auto normals = computeMapRoadVertexNormals(source);
        appendMapRoadProfile(road, spline, centerline, section, {}, std::nullopt, coordinates);
        require(!road.vertices.empty() && road.vertices.size() == source.vertices.size() &&
                    road.indices.size() == source.indices.size(),
                "Road geometry dimensions changed");
        for (std::size_t i = 0; i < source.vertices.size(); ++i) {
            const auto& v = road.vertices[i];
            const auto& s = source.vertices[i];
            near(v.x, s.x + 600, "Road local X");
            near(v.y, s.y - 900, "Road local Y");
            near(v.z, s.z, "Road height");
            near(v.u, s.textureU, "Road U");
            near(v.v, s.textureV, "Road V");
            near(v.nx, normals[i][0], "Road normal X");
            near(v.ny, normals[i][1], "Road normal Y");
            near(v.nz, normals[i][2], "Road normal Z");
            require(s.x >= road.minX && s.x <= road.maxX && s.y >= road.minY && s.y <= road.maxY &&
                        s.z >= road.minZ && s.z <= road.maxZ,
                    "Road world bounds do not contain vertex");
        }
        const auto vertexCount = road.vertices.size();
        const auto indexCount = road.indices.size();
        appendMapRoadProfile(road, spline, centerline, section, {}, std::nullopt, coordinates);
        require(road.vertices.size() == 2 * vertexCount && road.indices.size() == 2 * indexCount,
                "Road append did not preserve existing geometry");
        for (std::size_t i = 0; i < indexCount; ++i) {
            require(road.indices[i] == static_cast<std::uint32_t>(source.indices[i]) &&
                        road.indices[indexCount + i] == road.indices[i] + vertexCount,
                    "Road append index offsets changed");
        }
    } catch (const std::exception& error) {
        std::cerr << "map surface geometry probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
