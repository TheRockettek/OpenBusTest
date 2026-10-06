#include "MapSceneryPlacement.h"

#include <cmath>
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

void requireNear(double actual, double expected, double tolerance, const char* description) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(std::string(description) + ": expected " +
                                 std::to_string(expected) + ", got " +
                                 std::to_string(actual));
    }
}

openbus::map::MapSplinePlacement spline(int id, int previous, int next, double x, double y,
                                        double heading, double length) {
    openbus::map::MapSplinePlacement result;
    result.splineId = id;
    result.previousSplineId = previous;
    result.nextSplineId = next;
    result.localX = x;
    result.localY = y;
    result.rotationDegrees = heading;
    result.length = length;
    result.geometryValid = true;
    result.chainOffsetValid = true;
    return result;
}

openbus::map::MapSplineAttachment row(int splineIndex, double lateral, double height,
                                      double distance, double interval, double range) {
    openbus::map::MapSplineAttachment result;
    result.splineIndex = splineIndex;
    result.offset = {lateral, height, distance};
    result.interval = interval;
    result.range = range;
    result.transformValid = true;
    return result;
}

void runProbe() {
    using namespace openbus::map;

    MapSceneryPlacement ordinaryObject;
    ordinaryObject.localPosition = {12.5, -3.25, 4.75};
    ordinaryObject.rotationDegrees = {17.0, -23.0, 41.0};
    ordinaryObject.transformValid = true;
    const auto ordinaryPose = placeMapSceneryObject(ordinaryObject, -2, 3);
    require(ordinaryPose.has_value(), "valid ordinary scenery transform produces a pose");
    requireNear(ordinaryPose->x, -587.5, 1e-9, "object X uses tile origin plus authored X");
    requireNear(ordinaryPose->y, 896.75, 1e-9, "object Y uses tile origin plus authored Y");
    requireNear(ordinaryPose->z, 4.75, 1e-9, "object authored Z is preserved before terrain offset");
    require(ordinaryPose->rotationCount == 3 &&
                ordinaryPose->rotationDegrees ==
                    std::array<double, 6>{17.0, -23.0, 41.0, 0.0, 0.0, 0.0},
            "ordinary object rotation order is preserved");
    requireNear(mapSceneryWorldHeight(ordinaryPose->z, 12.0, false), 16.75, 1e-9,
                "terrain-relative ordinary scenery adds sampled terrain height");
    requireNear(mapSceneryWorldHeight(ordinaryPose->z, 12.0, true), 4.75, 1e-9,
                "absolute-height ordinary scenery ignores terrain height");
        TerrainGrid slopedTerrain;
        slopedTerrain.intervals = 1;
        slopedTerrain.heights = {0.0F, 30.0F, 60.0F, 90.0F};
        const auto terrainNormal = sampleMapTerrainNormal(slopedTerrain, 150.0, 150.0);
        const double normalScale = 1.0 / std::sqrt(1.05);
        require(terrainNormal.has_value(), "valid terrain grid yields a normal");
        requireNear((*terrainNormal)[0], -0.1 * normalScale, 1e-9,
            "terrain normal includes the eastward terrain gradient");
        requireNear((*terrainNormal)[1], -0.2 * normalScale, 1e-9,
            "terrain normal includes the northward terrain gradient");
        requireNear((*terrainNormal)[2], normalScale, 1e-9,
            "terrain normal is normalized with positive up");
        require(!sampleMapTerrainNormal(TerrainGrid{}, 0.0, 0.0).has_value(),
            "empty terrain grid has no terrain normal");
    const MapSceneryPose identityParent{};
    const MapSceneryPose anchored = composeMapAttachedObjectPose(
        identityParent, {1.0, 2.0, 3.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
    requireNear(anchored.x, 1.0, 1e-9, "attachment translation offsets parent X");
    requireNear(anchored.y, 2.0, 1e-9, "attachment translation offsets parent Y");
    requireNear(anchored.z, 3.0, 1e-9, "attachment translation offsets parent Z");
    requireNear(transformMapSceneryVector(anchored, {1.0, 0.0, 0.0})[0], 1.0, 1e-9,
                "identity attached orientation preserves local X");

    MapSceneryPose rotatedParent;
    rotatedParent.rotationDegrees[0] = 90.0;
    rotatedParent.rotationCount = 3;
    const MapSceneryPose rotatedAnchor = composeMapAttachedObjectPose(
        rotatedParent, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
    requireNear(rotatedAnchor.x, 0.0, 1e-9, "parent heading rotates anchor translation in X");
    requireNear(rotatedAnchor.y, -1.0, 1e-9, "parent heading rotates anchor translation in Y");

    const MapSceneryPose childRotated = composeMapAttachedObjectPose(
        identityParent, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {90.0, 0.0, 0.0});
    const std::array<double, 3> childAxis =
        transformMapSceneryVector(childRotated, {1.0, 0.0, 0.0});
    requireNear(childAxis[0], 0.0, 1e-9, "child yaw rotates local X");
    requireNear(childAxis[1], -1.0, 1e-9, "child yaw follows map renderer handedness");

    MapSceneryPose terrainAligned;
    terrainAligned.rotationDegrees[0] = 30.0;
    terrainAligned.rotationCount = 3;
    const std::array<double, 3> slopeNormal = {0.0, -0.6, 0.8};
    require(alignMapSceneryPoseToTerrainNormal(terrainAligned, slopeNormal),
            "finite terrain normal produces an aligned scenery pose");
    const std::array<double, 3> alignedUp =
        transformMapSceneryVector(terrainAligned, {0.0, 0.0, 1.0});
    requireNear(alignedUp[0], slopeNormal[0], 1e-9,
                "terrain-aligned local up matches sampled terrain normal X");
    requireNear(alignedUp[1], slopeNormal[1], 1e-9,
                "terrain-aligned local up matches sampled terrain normal Y");
    requireNear(alignedUp[2], slopeNormal[2], 1e-9,
                "terrain-aligned local up matches sampled terrain normal Z");
    requireNear(terrainAligned.x, 0.0, 1e-9,
                "terrain alignment preserves the scenery placement position");
    require(!alignMapSceneryPoseToTerrainNormal(terrainAligned, {0.0, 0.0, 0.0}),
            "degenerate terrain normal is rejected");

    ordinaryObject.transformValid = false;
    require(!placeMapSceneryObject(ordinaryObject, -2, 3).has_value(),
            "invalid ordinary scenery transform does not produce a pose");

    // A northbound attachment row: right is east, height is spline-relative, and
    // the row interval/range are measured from its authored chain distance.
    MapSplinePlacement north = spline(1, 0, 0, 100.0, 200.0, 0.0, 100.0);
    north.elevation = 10.0;
    const auto lamps = placeMapSplineAttachment(row(0, 3.0, 2.0, 10.0, 20.0, 45.0),
                                                {north}, 0, 0);
    require(lamps.size() == 3, "row obeys interval and inclusive range");
    for (std::size_t index = 0; index < lamps.size(); ++index) {
        requireNear(lamps[index].x, 103.0, 1e-9, "northbound right offset goes east");
        requireNear(lamps[index].y, 210.0 + 20.0 * index, 1e-9,
                    "attachment follows distance along spline");
        requireNear(lamps[index].z, 12.0, 1e-9, "attachment height is relative to spline");
        require(lamps[index].rotationCount == 6, "spline and object orientations are retained");
    }

    MapSplinePlacement east = spline(2, 0, 0, 0.0, 0.0, 90.0, 30.0);
    const auto eastLamp = placeMapSplineAttachment(row(0, 3.0, 0.0, 5.0, 0.0, 0.0),
                                                  {east}, 0, 0);
    require(eastLamp.size() == 1, "zero interval makes exactly one object");
    requireNear(eastLamp[0].x, 5.0, 1e-9, "eastbound spline sample X");
    requireNear(eastLamp[0].y, -3.0, 1e-9, "right of eastbound spline points south");

    MapSplinePlacement first = spline(10, 0, 11, 0.0, 0.0, 0.0, 10.0);
    MapSplinePlacement second = spline(11, 10, 0, 0.0, 10.0, 0.0, 20.0);
    const auto chained = placeMapSplineAttachment(row(0, 0.0, 0.0, 5.0, 10.0, 25.0),
                                                 {first, second}, 0, 0);
    require(chained.size() == 3, "master row continues along linked splines in its tile");
    requireNear(chained[0].y, 5.0, 1e-9, "first row object lies on master spline");
    requireNear(chained[1].y, 15.0, 1e-9, "row continues on next spline");
    requireNear(chained[2].y, 25.0, 1e-9, "chain distance remains continuous");

    MapSplinePlacement reverseStart = spline(20, 0, 21, 0.0, 0.0, 0.0, 10.0);
    MapSplinePlacement reverseSegment = spline(21, 22, 20, 0.0, 30.0, 180.0, 20.0);
    const auto reversed = placeMapSplineAttachment(row(0, 3.0, 0.0, 15.0, 0.0, 0.0),
                                                  {reverseStart, reverseSegment}, 0, 0);
    require(reversed.size() == 1, "row places one object on a reverse-linked segment");
    requireNear(reversed[0].x, 3.0, 1e-9,
                "right side remains consistent when a spline runs backwards in the chain");
    requireNear(reversed[0].y, 15.0, 1e-9, "reverse-linked segment samples from its far end");
    requireNear(reversed[0].rotationDegrees[3], 180.0, 1e-9,
                "reverse-linked row receives a half-turn");

    MapSplinePlacement repeaterSpline = spline(30, 29, 0, 0.0, 10.0, 0.0, 20.0);
    repeaterSpline.chainOffset = 10.0;
    MapSplineAttachment repeater = row(0, 0.0, 0.0, 5.0, 5.0, 15.0);
    repeater.repeater = true;
    repeater.repeaterFirstObjectIndex = 1;
    const auto repeated = placeMapSplineAttachment(repeater, {repeaterSpline}, 0, 0);
    require(repeated.size() == 3, "repeater resumes at its encoded first object index");
    requireNear(repeated[0].y, 10.0, 1e-9, "repeater places its first object at tile-spline start");
    requireNear(repeated[1].y, 15.0, 1e-9, "repeater retains row spacing");
    requireNear(repeated[2].y, 20.0, 1e-9, "repeater remains bounded by its spline");

    MapSplinePlacement ramp = spline(40, 0, 0, 0.0, 0.0, 0.0, 73.86);
    ramp.elevation = -5.79;
    ramp.gradientStart = 6.19;
    ramp.elevated = true;
    ramp.heightDelta = 5.85;
    MapSplineAttachment tilted = row(0, 0.0, 0.25, 73.86, 0.0, 0.0);
    tilted.tilt = true;
    const auto rampEnd = placeMapSplineAttachment(tilted, {ramp}, 0, 0);
    require(rampEnd.size() == 1, "tilted attachment can be placed at elevated spline end");
    requireNear(rampEnd[0].z, 0.31, 1e-9, "elevated spline height delta is honored");
    requireNear(rampEnd[0].rotationDegrees[1], 0.0, 1e-9,
                "elevated spline reaches its authored end gradient");
}

} // namespace

int main() {
    try {
        runProbe();
    } catch (const std::exception& error) {
        std::cerr << "map scenery placement probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
