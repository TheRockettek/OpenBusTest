#include "map/MapCollisionBuilder.h"
#include "map/MapTileWindow.h"
#include "map/MapTrafficRuntime.h"
#include "map/MapWaterGeometry.h"
#include "ProbeTempDirectory.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

using namespace openbus::map;

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void near(double actual, double expected, const char* message, double tolerance = 1.0e-8) {
    require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}

void verifyCoordinates() {
    const auto plain = MapCoordinates::forRows(false, {10940, -10940});
    near(plain.tileSize, 300.0, "plain maps retain 300 m tiles");
    const auto plainPoint = plain.tile(-2, 3).toWorld(12.5, -7.0);
    near(plainPoint[0], -587.5, "plain X coordinates unchanged");
    near(plainPoint[1], 893.0, "plain Y coordinates unchanged");
    near(mapWorldRowWidth(0), 611.496226348877, "equatorial zoom-16 tile width");
    near(mapWorldRowWidth(10940), 381.621, "Bristol projection scale", 0.001);
    near(mapWorldRowWidth(11281), 371.9, "Spandau has a different projection scale", 0.05);
    near(mapWorldRowWidth(-10940), mapWorldRowWidth(10940), "hemisphere symmetry");
    require(mapWorldRowWidth(10941) < mapWorldRowWidth(10940),
            "northern rows narrow with increasing latitude");

    const auto grid = MapCoordinates::forRows(true, {10941, 10937, 10940});
    const auto ordered = MapCoordinates::forRows(true, {10937, 10940, 10941});
    near(grid.tileSize, ordered.tileSize, "grid selection is independent of tile order");
    const auto tile = grid.tile(-494, 10940);
    const double width = (mapWorldRowWidth(10940) + mapWorldRowWidth(10941)) * 0.5;
    const double height = mapWorldRowWidth(10941);
    const auto upper = tile.toWorld(width, height);
    near(upper[0], -493 * grid.tileSize, "tile's metric width maps onto the next column");
    near(upper[1], 10941 * grid.tileSize, "upper-edge width is the tile height");
    const auto position = tile.toWorld(125.125, 270.375);
    const auto local = tile.toLocal(position[0], position[1]);
    near(local[0], 125.125, "negative-X tile inverse transform");
    near(local[1], 270.375, "large northing inverse transform");
    require(mapWorldTile(position[0], grid.tileSize) == -494 &&
                mapWorldTile(position[1], grid.tileSize) == 10940,
            "streaming uses the same grid as placement");
    require(selectMapTilesWithinRadius({{-494, 10940}, {-495, 10940}, {-490, 10940}}, position[0],
                                       position[1], 1, grid.tileSize)
                    .size() == 2,
            "active tile window uses the projected grid");
    require(position[1] > 4000000.0, "world coordinates are not rebased");

    for (const double invalid :
         {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        bool rejected = false;
        try {
            static_cast<void>(mapWorldRowWidth(invalid));
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "invalid projection rows are rejected");
    }
}

void verifyGeometryAndTraffic() {
    const auto grid = MapCoordinates::forRows(true, {10937, 10940, 10941});
    const auto aTile = grid.tile(-493, 10937);
    const auto bTile = grid.tile(-493, 10938);
    MapSplinePlacement a;
    a.geometryValid = true;
    a.splineId = 1;
    a.nextSplineId = 2;
    a.localX = 74.8923205569217;
    a.localY = 376.299161592529;
    a.elevation = 184.390797217497;
    a.rotationDegrees = 19.999999686135;
    a.length = 9.99999984306749;
    a.radius = -499.999992153375;
    MapSplinePlacement b = a;
    b.splineId = 2;
    b.previousSplineId = 1;
    b.nextSplineId = 0;
    b.localX = 78.2183432309441;
    b.localY = 4.05172714895049;
    b.rotationDegrees = 18.8540800777322;
    b.length = 19.999999686135;
    b.radius = -74.9999971050193;

    const auto aSamples = tessellateMapSpline(aTile.originX + a.localX, aTile.originY + a.localY,
                                              a.rotationDegrees, a.length, a.radius);
    const auto bSamples = tessellateMapSpline(bTile.originX + b.localX, bTile.originY + b.localY,
                                              b.rotationDegrees, b.length, b.radius);
    MapRoadConnectionIndex connections(grid);
    connections.addSpline(a, -493, 10937);
    connections.addSpline(b, -493, 10938);
    connections.finalize();
    require(connections.joinsFor(a, -493, 10937).end.has_value() &&
                connections.joinsFor(b, -493, 10938).start.has_value(),
            "Bristol cross-row links join instead of leaving an 81 m gap");
    MapSplineProfileSection section;
    section.points = {{-2.0, 0.25, 0.0, 0.2}, {2.0, 0.25, 1.0, 0.2}};
    const auto first = buildMapRoadSectionGeometry(
        a, aSamples, section, connections.joinsFor(a, -493, 10937), std::nullopt, aTile);
    const auto second = buildMapRoadSectionGeometry(
        b, bSamples, section, connections.joinsFor(b, -493, 10938), std::nullopt, bTile);
    for (std::size_t point = 0; point < 2; ++point) {
        const auto& left = first.vertices[first.vertices.size() - 2 + point];
        const auto& right = second.vertices[point];
        near(left.x, right.x, "projected seam X is identical");
        near(left.y, right.y, "projected seam Y is identical");
        near(left.z, right.z, "projected seam height is identical");
        const double gpuX = aTile.originX + static_cast<float>(left.x - aTile.originX);
        const double gpuY = aTile.originY + static_cast<float>(left.y - aTile.originY);
        near(gpuX, left.x, "tile-local GPU X retains sub-millimetre precision", 0.0001);
        near(gpuY, left.y, "tile-local GPU Y retains sub-millimetre precision", 0.0001);
    }

    MapTrafficRouteNetwork network;
    network.addTileSplines({a}, -493, 10937, grid);
    network.finalize();
    const auto route = network.buildRoute(a.splineId);
    require(route.has_value(), "projected traffic route can be built");
    const auto pose = sampleMapTrafficRoute(*route, 0.0, 0.0);
    require(pose.has_value(), "projected traffic route can be sampled");
    const auto expected = aTile.toWorld(a.localX, a.localY);
    near(pose->position[0], expected[0], "traffic shares scenery/road X transform");
    near(pose->position[1], expected[1], "traffic shares scenery/road Y transform");
    require(mapTrafficPoseOnActiveTile(*pose, {{-493, 10937}}, grid.tileSize),
            "projected traffic remains on its active tile");

    MapSceneryPlacement object;
    object.transformValid = true;
    object.localPosition = {a.localX, a.localY, a.elevation};
    const auto scenery = placeMapSceneryObject(object, -493, 10937, grid);
    require(scenery.has_value(), "projected scenery placement");
    near(scenery->x, expected[0], "scenery shares traffic X transform");
    near(scenery->y, expected[1], "scenery shares traffic Y transform");
    const auto offset = transformMapSceneryVector(*scenery, {2.0, 3.0, 4.0});
    near(offset[0], 2.0 * aTile.scaleX, "scenery mesh X scale");
    near(offset[1], 3.0 * aTile.scaleY, "scenery mesh Y scale");
    near(offset[2], 4.0, "projection does not scale elevation");
    const auto child = composeMapAttachedObjectPose(*scenery, {2.0, 3.0, 4.0}, {}, {});
    near(child.x, scenery->x + offset[0], "attachment inherits projection");
    near(child.y, scenery->y + offset[1], "attachment inherits projection Y");

    const TerrainGrid terrain{1, {0.0F, 2.0F, 4.0F, 6.0F}};
    const auto ground =
        sampleMapTerrainHeight(terrain, grid.tileSize * 0.5, grid.tileSize * 0.5, grid.tileSize);
    require(ground.has_value(), "projected terrain sampling");
    near(*ground, 3.0, "terrain samples the normalized tile grid");
    const auto water = buildMapWaterSurfaceVertices(-493, 10937, {}, grid.tileSize);
    near(water[0].x, aTile.originX, "water and terrain share tile origins");
    near(water[2].y, aTile.originY + grid.tileSize, "water fills projected tile height");
}

void verifyCollision() {
    const auto root = openbus::probe::makeProbeDirectory("openbus-map-coordinates-probe");
    struct Cleanup {
        std::filesystem::path path;
        ~Cleanup() {
            openbus::probe::removeProbeDirectory(path);
        }
    } cleanup{root};
    {
        std::ofstream profile(root / "Test.sli");
        profile << "[texture]\nroad.bmp\n[profile]\n0\n"
                   "[profilepnt]\n-2\n0.25\n0\n0.2\n[profilepnt]\n2\n0.25\n1\n0.2\n";
        require(static_cast<bool>(profile), "write projection collision profile");
    }
    MapDefinition map;
    map.rootPath = root;
    map.coordinates = MapCoordinates::forRows(true, {10940});
    MapTileReference tile;
    tile.x = -494;
    tile.y = 10940;
    tile.textPath = root / "tile.map";
    {
        std::ofstream output(tile.textPath);
        output << "[version]\n14\n[terrain]\n[spline]\n0\nTest.sli\n1\n0\n0\n"
                  "20\n5\n30\n0\n10\n0\n0\n0\n0\n0\n0\n0\n0\n";
        require(static_cast<bool>(output), "write projection collision tile");
    }
    map.tiles.push_back(tile);
    const auto collision = buildMapSplineCollisionTile(map, root, tile.x, tile.y);
    require(collision.splineSectionsAdded == 1 && collision.meshes.size() == 1,
            "projected spline collision is generated");
    const auto transform = map.coordinates.tile(tile.x, tile.y);
    const auto expected = transform.toWorld(18.0, 30.0);
    near(collision.meshes[0].vertices[0], expected[0], "collision uses projected road X");
    near(collision.meshes[0].vertices[1], expected[1], "collision uses projected road Y");
    near(collision.meshes[0].vertices[2], 5.25, "collision preserves authored road height");
}

} // namespace

int main() {
    try {
        verifyCoordinates();
        verifyGeometryAndTraffic();
        verifyCollision();
    } catch (const std::exception& error) {
        std::cerr << "map coordinates probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
