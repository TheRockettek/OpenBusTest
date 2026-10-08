#include "MapSceneryPlacement.h"

#include <cmath>
#include <iostream>
#include <numbers>
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
    require(mapSceneryAttachmentPointOffset(1, 3) == std::optional<std::size_t>(0) &&
                mapSceneryAttachmentPointOffset(3, 3) == std::optional<std::size_t>(2),
            "positive map attachment numbers select one-based configured points");
    require(!mapSceneryAttachmentPointOffset(0, 3).has_value() &&
                !mapSceneryAttachmentPointOffset(4, 3).has_value(),
            "origin anchor zero and out-of-range anchors have no configured-point offset");
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
    requireNear(anchored.x, 2.0, 1e-9,
                "attachment longitudinal translation maps to world X");
    requireNear(anchored.y, -1.0, 1e-9,
                "attachment lateral translation maps to negative world Y");
    requireNear(anchored.z, 3.0, 1e-9, "attachment translation offsets parent Z");
    requireNear(transformMapSceneryVector(anchored, {1.0, 0.0, 0.0})[0], 1.0, 1e-9,
                "identity attached orientation preserves local X");
    const MapSceneryPose originAnchored = composeMapAttachedObjectPose(
        identityParent, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
    requireNear(originAnchored.x, identityParent.x, 1e-9,
                "anchor zero leaves the child at the parent origin");
    requireNear(originAnchored.y, identityParent.y, 1e-9,
                "anchor zero does not select the first configured point");

    const std::array<std::array<double, 3>, 3> poleAnchors = {
        std::array<double, 3>{0.0, -0.04, 0.3},
        std::array<double, 3>{0.0, 0.04, 0.3},
        std::array<double, 3>{0.0, 0.0, 0.3}};
    const std::size_t thirdAnchor = *mapSceneryAttachmentPointOffset(3, poleAnchors.size());
    const MapSceneryPose attachedButton = composeMapAttachedObjectPose(
        identityParent, poleAnchors[thirdAnchor], {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
    requireNear(attachedButton.z, 0.3, 1e-9,
                "anchor number 3 attaches to the third point on a three-anchor pole");

    MapSceneryPose rotatedParent;
    rotatedParent.rotationDegrees[0] = 90.0;
    rotatedParent.rotationCount = 3;
    const MapSceneryPose rotatedAnchor = composeMapAttachedObjectPose(
        rotatedParent, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
    requireNear(rotatedAnchor.x, 0.0, 1e-9, "parent heading rotates anchor translation in X");
    requireNear(rotatedAnchor.y, -1.0, 1e-9, "parent heading rotates anchor translation in Y");

    MapSceneryPose pitchedParent;
    pitchedParent.rotationDegrees[1] = 90.0;
    pitchedParent.rotationCount = 3;
    const std::array<double, 3> pitchedForward =
        transformMapSceneryVector(pitchedParent, {0.0, 1.0, 0.0});
    requireNear(pitchedForward[1], 0.0, 1e-9,
                "parent pitch follows the ordinary scenery X-axis rotation sign");
    requireNear(pitchedForward[2], -1.0, 1e-9,
                "parent pitch rotates the anchor frame like rendered scenery");

    MapSceneryPose bankedParent;
    bankedParent.rotationDegrees[2] = 90.0;
    bankedParent.rotationCount = 3;
    const std::array<double, 3> bankedRight =
        transformMapSceneryVector(bankedParent, {1.0, 0.0, 0.0});
    requireNear(bankedRight[0], 0.0, 1e-9,
                "parent bank follows the ordinary scenery Y-axis rotation sign");
    requireNear(bankedRight[2], 1.0, 1e-9,
                "parent bank rotates the anchor frame like rendered scenery");

    MapSceneryPose splineFrame;
    splineFrame.rotationDegrees[1] = 90.0;
    splineFrame.rotationCount = 6;
    const std::array<double, 3> splineFrameForward =
        transformMapSceneryVector(splineFrame, {0.0, 1.0, 0.0});
    requireNear(splineFrameForward[2], 1.0, 1e-9,
                "spline-frame pitch matches MapRenderer's positive X rotation");
    MapSceneryPose splineObjectRotation;
    splineObjectRotation.rotationDegrees[4] = 90.0;
    splineObjectRotation.rotationCount = 6;
    const std::array<double, 3> splineObjectUp =
        transformMapSceneryVector(splineObjectRotation, {0.0, 0.0, 1.0});
    requireNear(splineObjectUp[1], 1.0, 1e-9,
                "spline attachment's own pitch matches the renderer's negative X rotation");

    const MapSceneryPose configRotatedX = composeMapAttachedObjectPose(
        identityParent, {0.0, 0.0, 0.0}, {90.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
    const std::array<double, 3> configXUp =
        transformMapSceneryVector(configRotatedX, {0.0, 0.0, 1.0});
    requireNear(configXUp[0], 1.0, 1e-9,
                "attachment config X rotation maps to the converted map Y axis");
    const MapSceneryPose configRotatedY = composeMapAttachedObjectPose(
        identityParent, {0.0, 0.0, 0.0}, {0.0, 90.0, 0.0}, {0.0, 0.0, 0.0});
    const std::array<double, 3> configYUp =
        transformMapSceneryVector(configRotatedY, {0.0, 0.0, 1.0});
    requireNear(configYUp[1], 1.0, 1e-9,
                "attachment config Y rotation maps to the converted map X axis");

    const MapSceneryPose childRotated = composeMapAttachedObjectPose(
        identityParent, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {90.0, 0.0, 0.0});
    const std::array<double, 3> childAxis =
        transformMapSceneryVector(childRotated, {1.0, 0.0, 0.0});
    requireNear(childAxis[0], 0.0, 1e-9, "child yaw rotates local X");
    requireNear(childAxis[1], -1.0, 1e-9, "child yaw follows map renderer handedness");

    const MapSceneryPose childPitched = composeMapAttachedObjectPose(
        identityParent, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 90.0, 0.0});
    const std::array<double, 3> childUp =
        transformMapSceneryVector(childPitched, {0.0, 0.0, 1.0});
    requireNear(childUp[1], 1.0, 1e-9,
                "attached-child pitch uses the ordinary scenery rotation sign");
    requireNear(childUp[2], 0.0, 1e-9,
                "attached-child pitch rotates its orientation consistently");

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

    MapSplinePlacement bankedHost = spline(3, 0, 0, 0.0, 0.0, 0.0, 20.0);
    bankedHost.cantStart = bankedHost.cantEnd = 5.0;
    MapSplineAttachment bankedRow = row(0, 10.0, 0.0, 0.0, 0.0, 0.0);
    bankedRow.tilt = true;
    const auto bankedLamp = placeMapSplineAttachment(bankedRow, {bankedHost}, 0, 0);
    require(bankedLamp.size() == 1, "banked spline row places its single object");
    requireNear(bankedLamp[0].z, -0.5, 1e-9,
                "5% cant lowers a row object 0.5 m at a 10 m lateral offset");
    requireNear(bankedLamp[0].rotationDegrees[2],
                std::atan(0.05) * 180.0 / std::numbers::pi, 1e-9,
                "tilted row orientation converts 5% crossfall to its bank angle");
    const auto farBankedLamp = placeMapSplineAttachment(
        row(0, 20.0, 0.0, 0.0, 0.0, 0.0), {bankedHost}, 0, 0);
    require(farBankedLamp.size() == 1, "wide-offset spline row places its object");
    requireNear(farBankedLamp[0].z, -1.0, 1e-9,
                "cant height remains proportional beyond a 10 m lateral offset");

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

    MapSplinePlacement attachmentHost = spline(50, 0, 0, 100.0, 200.0, 0.0, 100.0);
    attachmentHost.elevation = 8.0;
    const MapSplineAttachment attachmentParent = row(0, 0.0, 0.0, 5.0, 10.0, 0.0);
    const auto attachmentParentPose = placeMapSplineAttachmentParent(
        attachmentParent, {attachmentHost}, 0, 0);
    require(attachmentParentPose.has_value(),
            "a single spline attachment can resolve as an [attachObj] parent");
    requireNear(attachmentParentPose->x, 100.0, 1e-9,
                "spline attachment parent retains its local spline X");
    requireNear(attachmentParentPose->y, 205.0, 1e-9,
                "spline attachment parent uses its authored chain distance");
    requireNear(attachmentParentPose->z, 8.0, 1e-9,
                "spline attachment parent retains spline elevation");
    const MapSceneryPose attachedChild = composeMapAttachedObjectPose(
        *attachmentParentPose, {0.04, -0.186, 2.545}, {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0});
    requireNear(attachedChild.x, 99.814, 1e-9,
                "child attachment offsets a spline-generated parent in map X");
    requireNear(attachedChild.y, 204.96, 1e-9,
                "child attachment offsets a spline-generated parent in map Y");
    requireNear(attachedChild.z, 10.545, 1e-9,
                "child attachment height composes with spline-parent elevation");
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
