#include "map/MapRoadGeometry.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireNear(double actual, double expected, const char* message) {
    if (!std::isfinite(actual) || !std::isfinite(expected) || std::abs(actual - expected) > 1.0e-8) {
        throw std::runtime_error(std::string(message) + ": expected " +
                                 std::to_string(expected) + ", got " + std::to_string(actual));
    }
}

void requireSamePosition(const openbus::map::MapRoadVertex& left,
                         const openbus::map::MapRoadVertex& right) {
    requireNear(left.x, right.x, "stitched endpoint X agrees");
    requireNear(left.y, right.y, "stitched endpoint Y agrees");
    requireNear(left.z, right.z, "stitched endpoint height agrees");
}

void runSkewProbe() {
    using namespace openbus::map;
    MapSplinePlacement spline;
    spline.geometryValid = true;
    spline.length = 10.0;
    spline.gradientStart = spline.gradientEnd = 12.0;
    spline.cantStart = spline.cantEnd = 5.0;
    spline.skewStart = 1.0;
    spline.skewEnd = -0.5;
    spline.chainOffsetValid = true;
    spline.chainOffset = 20.0;
    MapSplineProfileSection section;
    section.points = {{-2.0, 0.25, 0.0, 0.2}, {0.0, 0.25, 0.5, 0.2},
                      {2.0, 0.25, 1.0, 0.2}};
    const auto samples = tessellateMapSpline(0.0, 0.0, 0.0, 10.0, 0.0);
    const auto geometry = buildMapRoadSectionGeometry(spline, samples, section);
    requireNear(geometry.vertices[0].y, -2.0, "positive start skew cuts the negative edge backwards");
    requireNear(geometry.vertices[2].y, 2.0, "positive start skew cuts the positive edge forwards");
    requireNear(geometry.vertices[1].y, 0.0, "skew leaves the centerline unchanged");
    requireNear(geometry.vertices[0].z, 0.11,
                "skew overhang continues grade and applies 5% cant");
    requireNear(geometry.vertices[2].textureV, 4.4, "skew UVs include chain phase and forward shift");
    requireNear(geometry.vertices.back().y, 9.0, "negative end skew reverses the cut direction");
    requireNear(geometry.vertices[geometry.vertices.size() - 3].y, 11.0,
                "skew can extend beyond the centerline end without clamping");
    spline.mirrored = true;
    const auto mirrored = buildMapRoadSectionGeometry(spline, samples, section);
    requireNear(mirrored.vertices[0].y, geometry.vertices[2].y,
                "mirroring reflects skew together with lateral offset");
    requireNear(mirrored.vertices[0].z, geometry.vertices[2].z,
                "mirrored skew keeps grade/cant in the reflected physical frame");
    require(mirrored.indices[1] == geometry.indices[2], "skew retains mirrored winding policy");

    spline.mirrored = false;
    spline.radius = 20.0;
    const auto curve = tessellateMapSpline(0.0, 0.0, 0.0, spline.length, spline.radius);
    const auto curved = buildMapRoadSectionGeometry(spline, curve, section);
    const auto& end = curve.back();
    const auto right = mapSplineRightOffset(end.headingRadians, 2.0);
    requireNear(curved.vertices.back().x, end.x + right[0] - std::sin(end.headingRadians),
                "curved skew shears along the local tangent X");
    requireNear(curved.vertices.back().y, end.y + right[1] - std::cos(end.headingRadians),
                "curved skew shears along the local tangent Y");
    spline.skewEnd = std::numeric_limits<double>::infinity();
    bool rejected = false;
    try {
        buildMapRoadSectionGeometry(spline, curve, section);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "non-finite skew is rejected before upload/ODE publication");
    spline.length = 0.0;
    require(buildMapRoadSectionGeometry(spline, curve, section).vertices.empty(),
            "zero-length splines do not divide by zero");
}

void runJoinProbe() {
    using namespace openbus::map;
    MapSplinePlacement a;
    a.geometryValid = true;
    a.splineId = 10;
    a.nextSplineId = 20;
    a.localY = 290.0;
    a.length = 10.0;
    MapSplinePlacement b = a;
    b.splineId = 20;
    b.previousSplineId = 10;
    b.nextSplineId = 0;
    b.localY = 0.0;
    b.rotationDegrees = 90.0;
    b.chainOffsetValid = true;
    b.chainOffset = 10.0;
    MapSplineProfileSection section;
    section.points = {{-2.0, 0.25, 0.0, 0.2}, {0.0, 0.25, 0.5, 0.2},
                      {2.0, 0.25, 1.0, 0.2}};
    const auto aSamples = tessellateMapSpline(0.0, 290.0, 0.0, 10.0, 0.0);
    const auto bSamples = tessellateMapSpline(0.0, 300.0, 90.0, 10.0, 0.0);
    const auto joinsForPair = [&](const MapSplinePlacement& first, const MapSplinePlacement& second) {
        MapRoadConnectionIndex index;
        index.addSpline(first, 0, 0);
        index.addSpline(second, 0, 1);
        index.finalize();
        return index;
    };
    auto index = joinsForPair(a, b);
    require(index.joinsFor(a, 0, 0).end.has_value() && index.joinsFor(b, 0, 1).start.has_value(),
            "reciprocal links resolve across tile boundaries");
    const auto first = buildMapRoadSectionGeometry(a, aSamples, section, index.joinsFor(a, 0, 0));
    const auto second = buildMapRoadSectionGeometry(b, bSamples, section, index.joinsFor(b, 0, 1));
    for (std::size_t p = 0; p < section.points.size(); ++p) {
        requireSamePosition(first.vertices[first.vertices.size() - 3 + p], second.vertices[p]);
    }
    requireNear(first.vertices.back().x, 2.0, "90-degree miter preserves the road edge X");
    requireNear(first.vertices.back().y, 298.0, "90-degree miter intersects both offset edges");
    const auto independent = buildMapRoadSectionGeometry(a, aSamples, section);
    require(first.indices == independent.indices, "stitching retains strip topology/winding");
    requireSamePosition(first.vertices[6], independent.vertices[6]);

    // Resolve the same pair in reverse insertion order; no traversal/load order dependency.
    MapRoadConnectionIndex reverseIndex;
    reverseIndex.addSpline(b, 0, 1);
    reverseIndex.addSpline(a, 0, 0);
    reverseIndex.finalize();
    const auto reverse = buildMapRoadSectionGeometry(a, aSamples, section,
                                                    reverseIndex.joinsFor(a, 0, 0));
    requireSamePosition(first.vertices.back(), reverse.vertices.back());

    a.gradientStart = 12.0;
    a.gradientEnd = -11.2;
    a.cantStart = a.cantEnd = 200.0;
    a.skewEnd = 1.0;
    b.elevation = 0.04;
    b.gradientStart = b.gradientEnd = -11.2;
    b.cantStart = b.cantEnd = -120.0;
    b.skewStart = -0.5;
    index = joinsForPair(a, b);
    const auto bankedA = buildMapRoadSectionGeometry(a, aSamples, section, index.joinsFor(a, 0, 0));
    const auto bankedB = buildMapRoadSectionGeometry(b, bSamples, section, index.joinsFor(b, 0, 1));
    for (std::size_t p = 0; p < 3; ++p) {
        requireSamePosition(bankedA.vertices[bankedA.vertices.size() - 3 + p], bankedB.vertices[p]);
    }

    // Same-end links reverse the physical right axis; mirror maps the authored profile back.
    a.gradientStart = a.gradientEnd = 0.0;
    a.cantStart = a.cantEnd = 0.0;
    a.skewEnd = 0.25;
    b.elevation = 0.0;
    b.localY = 10.0;
    b.rotationDegrees = 180.0;
    b.previousSplineId = 0;
    b.nextSplineId = a.splineId;
    b.gradientStart = b.gradientEnd = 0.0;
    b.cantStart = b.cantEnd = 0.0;
    b.skewEnd = 0.25;
    b.mirrored = true;
    const auto reversedSamples = tessellateMapSpline(0.0, 310.0, 180.0, 10.0, 0.0);
    index = joinsForPair(a, b);
    require(index.joinsFor(b, 0, 1).end.has_value(), "end-to-end links are supported");
    const auto reversedA = buildMapRoadSectionGeometry(a, aSamples, section, index.joinsFor(a, 0, 0));
    const auto reversedB = buildMapRoadSectionGeometry(b, reversedSamples, section,
                                                      index.joinsFor(b, 0, 1));
    for (std::size_t p = 0; p < 3; ++p) {
        requireSamePosition(reversedA.vertices[reversedA.vertices.size() - 3 + p],
                            reversedB.vertices[reversedB.vertices.size() - 3 + p]);
    }

    // Without mirroring the opposite physical sides meet, not equal point ordinals.
    b.mirrored = false;
    index = joinsForPair(a, b);
    const auto unmirroredB = buildMapRoadSectionGeometry(b, reversedSamples, section,
                                                        index.joinsFor(b, 0, 1));
    for (std::size_t p = 0; p < 3; ++p) {
        requireSamePosition(reversedA.vertices[reversedA.vertices.size() - 3 + p],
                            unmirroredB.vertices[unmirroredB.vertices.size() - 1 - p]);
    }

    MapSplinePlacement startA = a;
    startA.localY = 0.0;
    startA.previousSplineId = 20;
    startA.nextSplineId = 0;
    MapSplinePlacement startB = b;
    startB.localY = -300.0;
    startB.previousSplineId = 10;
    startB.nextSplineId = 0;
    startB.mirrored = true;
    const auto startASamples = tessellateMapSpline(0.0, 0.0, 0.0, 10.0, 0.0);
    const auto startBSamples = tessellateMapSpline(0.0, 0.0, 180.0, 10.0, 0.0);
    index = joinsForPair(startA, startB);
    require(index.joinsFor(startA, 0, 0).start.has_value() &&
                index.joinsFor(startB, 0, 1).start.has_value(), "start-to-start links are supported");
    const auto startFirst = buildMapRoadSectionGeometry(startA, startASamples, section,
                                                       index.joinsFor(startA, 0, 0));
    const auto startSecond = buildMapRoadSectionGeometry(startB, startBSamples, section,
                                                        index.joinsFor(startB, 0, 1));
    for (std::size_t p = 0; p < 3; ++p) {
        requireSamePosition(startFirst.vertices[p], startSecond.vertices[p]);
    }

    // A curved endpoint and a short differently tessellated continuation share exactly one seam.
    MapSplinePlacement arc = a;
    arc.radius = -20.0;
    arc.skewEnd = -0.3;
    const auto arcSamples = tessellateMapSpline(0.0, 290.0, 0.0, arc.length, arc.radius);
    MapSplinePlacement continuation = a;
    continuation.splineId = 20;
    continuation.previousSplineId = 10;
    continuation.nextSplineId = 0;
    continuation.localX = arcSamples.back().x;
    continuation.localY = arcSamples.back().y - 300.0;
    continuation.rotationDegrees = arcSamples.back().headingRadians * 180.0 / std::numbers::pi;
    continuation.length = 0.5;
    continuation.skewStart = -0.3;
    continuation.skewEnd = 0.0;
    const auto shortSamples = tessellateMapSpline(arcSamples.back().x, arcSamples.back().y,
                                                 continuation.rotationDegrees, 0.5, 0.0);
    index = joinsForPair(arc, continuation);
    const auto arcGeometry = buildMapRoadSectionGeometry(arc, arcSamples, section,
                                                         index.joinsFor(arc, 0, 0));
    const auto shortGeometry = buildMapRoadSectionGeometry(continuation, shortSamples, section,
                                                          index.joinsFor(continuation, 0, 1));
    for (std::size_t p = 0; p < 3; ++p) {
        requireSamePosition(arcGeometry.vertices[arcGeometry.vertices.size() - 3 + p],
                            shortGeometry.vertices[p]);
    }

    // A straight continuation with matching skew preserves chain texture phase across the cut.
    a.skewEnd = 0.5;
    b = a;
    b.splineId = 20;
    b.localY = 0.0;
    b.previousSplineId = 10;
    b.nextSplineId = 0;
    b.skewStart = 0.5;
    b.skewEnd = 0.0;
    b.chainOffsetValid = true;
    b.chainOffset = 10.0;
    index = joinsForPair(a, b);
    const auto straightB = tessellateMapSpline(0.0, 300.0, 0.0, 10.0, 0.0);
    const auto phaseA = buildMapRoadSectionGeometry(a, aSamples, section, index.joinsFor(a, 0, 0));
    const auto phaseB = buildMapRoadSectionGeometry(b, straightB, section, index.joinsFor(b, 0, 1));
    for (std::size_t p = 0; p < 3; ++p) {
        requireSamePosition(phaseA.vertices[phaseA.vertices.size() - 3 + p], phaseB.vertices[p]);
        requireNear(phaseA.vertices[phaseA.vertices.size() - 3 + p].textureV,
                    phaseB.vertices[p].textureV, "skewed stitched chains retain texture phase");
    }
    b.previousSplineId = 0;
    require(!joinsForPair(a, b).joinsFor(a, 0, 0).end, "one-way stale links are not stitched");
    b.previousSplineId = 10;
    b.localY = 2.0;
    require(!joinsForPair(a, b).joinsFor(a, 0, 0).end, "distant linked endpoints are left separate");
    b.localY = 0.0;
    b.elevation = 2.0;
    require(!joinsForPair(a, b).joinsFor(a, 0, 0).end, "grade-separated roads are not welded");
    b.elevation = 0.0;
    b.rotationDegrees = 180.0;
    require(!joinsForPair(a, b).joinsFor(a, 0, 0).end, "near U-turn miter spikes are rejected");
    b.rotationDegrees = 0.0;
    auto duplicates = joinsForPair(a, b);
    duplicates.addSpline(a, 1, 0);
    duplicates.finalize();
    require(!duplicates.joinsFor(a, 0, 0).end && !duplicates.joinsFor(b, 0, 1).start,
            "duplicate spline IDs disable ambiguous joins on both sides");

    // Curved, elevated segment joining a straight continuation with two active seam ends.
    a.localY = 290.0;
    a.previousSplineId = 0;
    a.nextSplineId = 20;
    a.radius = 20.0;
    a.elevated = true;
    a.heightDelta = 1.0;
    a.gradientStart = 0.0;
    a.gradientEnd = 12.0;
    a.skewEnd = 0.25;
    const auto curvedSamples = tessellateMapSpline(0.0, 290.0, 0.0, 10.0, 20.0);
    const auto& curvedEnd = curvedSamples.back();
    b.localX = curvedEnd.x;
    b.localY = curvedEnd.y - 300.0;
    b.elevation = 1.0;
    b.rotationDegrees = curvedEnd.headingRadians * 180.0 / std::numbers::pi;
    b.previousSplineId = 10;
    b.nextSplineId = 30;
    b.gradientStart = b.gradientEnd = 12.0;
    b.skewStart = 0.25;
    const auto middleSamples = tessellateMapSpline(curvedEnd.x, curvedEnd.y, b.rotationDegrees,
                                                   b.length, 0.0);
    MapSplinePlacement c = b;
    c.splineId = 30;
    c.previousSplineId = 20;
    c.nextSplineId = 0;
    c.localX = middleSamples.back().x;
    c.localY = middleSamples.back().y - 300.0;
    c.elevation = 2.2;
    c.skewStart = b.skewEnd;
    const auto lastSamples = tessellateMapSpline(c.localX, c.localY + 300.0, c.rotationDegrees,
                                                 c.length, c.radius);
    index = joinsForPair(a, b);
    index.addSpline(c, 0, 1);
    index.finalize();
    const auto curvedJoined = buildMapRoadSectionGeometry(a, curvedSamples, section,
                                                         index.joinsFor(a, 0, 0));
    const auto middleJoined = buildMapRoadSectionGeometry(b, middleSamples, section,
                                                         index.joinsFor(b, 0, 1));
    const auto lastJoined = buildMapRoadSectionGeometry(c, lastSamples, section,
                                                       index.joinsFor(c, 0, 1));
    require(index.joinsFor(b, 0, 1).start && index.joinsFor(b, 0, 1).end,
            "chain middle segments can stitch both ends independently");
    for (std::size_t p = 0; p < 3; ++p) {
        requireSamePosition(curvedJoined.vertices[curvedJoined.vertices.size() - 3 + p],
                            middleJoined.vertices[p]);
        requireSamePosition(middleJoined.vertices[middleJoined.vertices.size() - 3 + p],
                            lastJoined.vertices[p]);
    }

    a.radius = 0.0;
    a.elevated = false;
    a.gradientStart = a.gradientEnd = 0.0;
    a.length = 0.1;
    a.localY = 300.0;
    a.previousSplineId = a.nextSplineId = 20;
    b.localX = 0.0;
    b.localY = 0.05;
    b.elevation = 0.0;
    b.rotationDegrees = 0.0;
    b.previousSplineId = 10;
    b.nextSplineId = 0;
    index = joinsForPair(a, b);
    require(!index.joinsFor(a, 0, 0).start && !index.joinsFor(a, 0, 0).end,
            "ambiguous short loops cannot weld two endpoints onto one neighbor endpoint");
}

void runAuthoredSkewJoinProbe() {
    using namespace openbus::map;
    MapSplinePlacement first;
    first.geometryValid = true;
    first.splineId = 518;
    first.previousSplineId = 517;
    first.nextSplineId = 519;
    first.localX = 72.2001113607307;
    first.elevation = -0.00999999228392505;
    first.localY = 105.651410888032;
    first.rotationDegrees = 139.684998029509;
    first.length = 3.999999937227;
    first.skewEnd = 1.02286673378804;

    MapSplinePlacement second;
    second.geometryValid = true;
    second.splineId = 519;
    second.previousSplineId = 518;
    second.nextSplineId = 520;
    second.localX = 74.7880662387655;
    second.elevation = -0.00999999159673029;
    second.localY = 102.601408582255;
    second.rotationDegrees = 48.3896957169828;
    second.length = 3.50000011687231;
    second.skewStart = -1.02286673378804;

    MapRoadConnectionIndex connections;
    connections.addSpline(first, 0, 0);
    connections.addSpline(second, 0, 0);
    connections.finalize();
    const MapRoadSplineJoins firstJoins = connections.joinsFor(first, 0, 0);
    const MapRoadSplineJoins secondJoins = connections.joinsFor(second, 0, 0);
    require(firstJoins.end.has_value() && secondJoins.start.has_value(),
            "Cotterell's reciprocal skewed right-angle splines are joined");

    MapSplineProfileSection section;
    section.points = {{0.0, 0.0, 0.0, 0.0}, {3.0, 0.0, 1.0, 0.0}};
    const auto firstSamples = tessellateMapSpline(first.localX, first.localY,
                                                  first.rotationDegrees, first.length, 0.0);
    const auto secondSamples = tessellateMapSpline(second.localX, second.localY,
                                                   second.rotationDegrees, second.length, 0.0);
    const auto firstGeometry =
        buildMapRoadSectionGeometry(first, firstSamples, section, firstJoins);
    const auto secondGeometry =
        buildMapRoadSectionGeometry(second, secondSamples, section, secondJoins);
    requireSamePosition(firstGeometry.vertices.back(), secondGeometry.vertices[1]);

    const MapSplineSample& firstEndpoint = firstSamples.back();
    const MapSplineSample& secondEndpoint = secondSamples.front();
    const auto firstRight = mapSplineRightOffset(firstEndpoint.headingRadians, 1.0);
    const auto secondRight = mapSplineRightOffset(secondEndpoint.headingRadians, 1.0);
    const std::array<double, 2> firstAxis = {
        firstRight[0] + first.skewEnd * std::sin(firstEndpoint.headingRadians),
        firstRight[1] + first.skewEnd * std::cos(firstEndpoint.headingRadians)};
    const std::array<double, 2> secondAxis = {
        secondRight[0] + second.skewStart * std::sin(secondEndpoint.headingRadians),
        secondRight[1] + second.skewStart * std::cos(secondEndpoint.headingRadians)};
    requireNear(firstJoins.end->lateralAxis[0], 0.5 * (firstAxis[0] + secondAxis[0]),
                "aligned skew seam uses the mean authored parallelogram axis (x)");
    requireNear(firstJoins.end->lateralAxis[1], 0.5 * (firstAxis[1] + secondAxis[1]),
                "aligned skew seam uses the mean authored parallelogram axis (y)");
    requireNear(firstGeometry.vertices.back().x,
                firstJoins.end->center[0] + 3.0 * firstJoins.end->lateralAxis[0],
                "first skewed strip ends on its shared cross-section (x)");
    requireNear(firstGeometry.vertices.back().y,
                firstJoins.end->center[1] + 3.0 * firstJoins.end->lateralAxis[1],
                "first skewed strip ends on its shared cross-section (y)");
}

void runProbe() {
    openbus::map::MapSplinePlacement spline;
    spline.geometryValid = true;
    spline.localX = 100.0;
    spline.localY = 200.0;
    spline.elevation = 2.0;
    spline.length = 10.0;
    spline.gradientStart = 0.0;
    spline.gradientEnd = 10.0;

    openbus::map::MapSplineProfileSection section;
    section.textureIndex = 0;
    section.points = {{-2.0, 0.25, 0.0, 0.2}, {2.0, 0.25, 1.0, 0.2}};
    const auto straightCenterline =
        openbus::map::tessellateMapSpline(100.0, 200.0, 0.0, spline.length, 0.0);
    const auto straight =
        openbus::map::buildMapRoadSectionGeometry(spline, straightCenterline, section);
    require(straight.vertices.size() == straightCenterline.size() * 2,
            "straight cross-section keeps both profile points at every station");
    require(straight.indices.size() == (straightCenterline.size() - 1) * 6,
            "straight cross-section produces two triangles per segment");
    requireNear(straight.vertices[0].x, 98.0, "right-offset west profile edge");
    requireNear(straight.vertices[1].x, 102.0, "right-offset east profile edge");
    requireNear(straight.vertices.front().z, 2.25, "profile height is added to spline elevation");
    requireNear(straight.vertices[straight.vertices.size() - 1].z, 2.75,
                "spline gradient is reflected at the road endpoint");
    requireNear(straight.vertices.back().textureV, 2.0,
                "road texture V follows spline distance and profile scale");

    openbus::map::MapSplineProfileSection lengthScaledSection = section;
    lengthScaledSection.scaleTextureByLength = true;
    lengthScaledSection.points = {{-2.0, 0.25, 0.0, 1.0}, {2.0, 0.25, 1.0, 1.0}};
    const auto lengthScaled = openbus::map::buildMapRoadSectionGeometry(
        spline, straightCenterline, lengthScaledSection);
    requireNear(lengthScaled.vertices.front().textureV, 0.0,
                "length-scaled texture phase starts at zero on each spline segment");
    requireNear(lengthScaled.vertices.back().textureV, 1.0,
                "scaleTexByLength maps one texture span across the literal segment length");

    spline.chainOffsetValid = true;
    spline.chainOffset = spline.length;
    const auto continued =
        openbus::map::buildMapRoadSectionGeometry(spline, straightCenterline, section);
    requireNear(continued.vertices.front().textureV, straight.vertices.back().textureV,
                "chain offset continues texture phase at a linked spline start");
    requireNear(continued.vertices.back().textureV, 4.0,
                "texture phase advances from the authored chain offset");
    const auto continuedLengthScaled = openbus::map::buildMapRoadSectionGeometry(
        spline, straightCenterline, lengthScaledSection);
    requireNear(continuedLengthScaled.vertices.front().textureV, 0.0,
                "length-scaled texture phase remains segment-local despite chain offsets");
    requireNear(continuedLengthScaled.vertices.back().textureV, 1.0,
                "length-scaled texture span remains one repeat on linked segments");
    require(continued.indices == straight.indices &&
                continued.vertices.size() == straight.vertices.size(),
            "chain texture offset does not alter road topology");
    for (std::size_t vertexIndex = 0; vertexIndex < straight.vertices.size(); ++vertexIndex) {
        requireNear(continued.vertices[vertexIndex].x, straight.vertices[vertexIndex].x,
                    "chain texture offset leaves road x unchanged");
        requireNear(continued.vertices[vertexIndex].y, straight.vertices[vertexIndex].y,
                    "chain texture offset leaves road y unchanged");
        requireNear(continued.vertices[vertexIndex].z, straight.vertices[vertexIndex].z,
                    "chain texture offset leaves road height unchanged");
    }

    openbus::map::MapSplinePlacement bankedSpline = spline;
    bankedSpline.length = 10.0;
    bankedSpline.elevation = 0.0;
    bankedSpline.gradientStart = 0.0;
    bankedSpline.gradientEnd = 0.0;
    bankedSpline.cantStart = 2.0;
    bankedSpline.cantEnd = 4.0;
    openbus::map::MapSplineProfileSection bankedSection;
    bankedSection.points = {{-2.0, 0.0, 0.0, 0.0}, {2.0, 0.0, 1.0, 0.0}};
    const std::vector<openbus::map::MapSplineSample> bankSamples = {
        {0.0, 0.0, 0.0, 0.0}, {0.0, 5.0, 5.0, 0.0}, {0.0, 10.0, 10.0, 0.0}};
    const auto banked = openbus::map::buildMapRoadSectionGeometry(
        bankedSpline, bankSamples, bankedSection);
    requireNear(banked.vertices[0].z, 0.04,
                "positive cant raises the negative-lateral profile edge at spline start");
    requireNear(banked.vertices[1].z, -0.04,
                "positive cant lowers the positive-lateral profile edge at spline start");
    requireNear(banked.vertices[2].z, 0.06,
                "cant is interpolated linearly at the spline midpoint");
    requireNear(banked.vertices[3].z, -0.06,
                "interpolated cant is applied symmetrically across the section");
    requireNear(banked.vertices[4].z, 0.08,
                "end cant reaches the negative-lateral endpoint height");
    requireNear(banked.vertices[5].z, -0.08,
                "end cant reaches the positive-lateral endpoint height");

    openbus::map::MapSplinePlacement fivePercentSpline = bankedSpline;
    fivePercentSpline.cantStart = fivePercentSpline.cantEnd = 5.0;
    openbus::map::MapSplineProfileSection threeMeterSection;
    threeMeterSection.points = {{-1.5, 0.0, 0.0, 0.0}, {1.5, 0.0, 1.0, 0.0}};
    const auto fivePercent = openbus::map::buildMapRoadSectionGeometry(
        fivePercentSpline, bankSamples, threeMeterSection);
    requireNear(fivePercent.vertices[0].z, 0.075,
                "5% cant raises the negative-lateral edge by 0.075 m at 1.5 m");
    requireNear(fivePercent.vertices[1].z, -0.075,
                "5% cant lowers the positive-lateral edge by 0.075 m at 1.5 m");
    requireNear(fivePercent.vertices[1].z - fivePercent.vertices[0].z, -0.15,
                "5% cant changes height by 0.15 m across a 3 m road width");
    const auto fivePercentNormals = computeMapRoadVertexNormals(fivePercent);
    const double expectedNormalLength = std::sqrt(1.0 + 0.05 * 0.05);
    requireNear(fivePercentNormals[0][0], 0.05 / expectedNormalLength,
                "render normal tilts with positive cant across the road");
    requireNear(fivePercentNormals[0][1], 0.0,
                "straight-road cant does not tilt the normal along the road");
    requireNear(fivePercentNormals[0][2], 1.0 / expectedNormalLength,
                "banked-road normal remains unit length and upward-facing");
    openbus::map::MapSplinePlacement fortyFiveDegreeSpline = bankedSpline;
    fortyFiveDegreeSpline.cantStart = fortyFiveDegreeSpline.cantEnd = -100.0;
    openbus::map::MapSplineProfileSection fullRoadWidthSection;
    fullRoadWidthSection.points = {{0.0, 0.0, 0.0, 0.0}, {4.55, 0.0, 1.0, 0.0}};
    const auto fortyFiveDegreeBank = openbus::map::buildMapRoadSectionGeometry(
        fortyFiveDegreeSpline, bankSamples, fullRoadWidthSection);
    requireNear(fortyFiveDegreeBank.vertices[1].z - fortyFiveDegreeBank.vertices[0].z, 4.55,
                "-100% cant raises the far road edge by one full road width");
    const auto fortyFiveDegreeNormals = computeMapRoadVertexNormals(fortyFiveDegreeBank);
    requireNear(fortyFiveDegreeNormals[0][0], -std::sqrt(0.5),
                "-100% cant produces the expected 45-degree normal tilt");
    requireNear(fortyFiveDegreeNormals[0][2], std::sqrt(0.5),
                "-100% cant retains an upward-facing 45-degree normal");
    openbus::map::MapSplineProfileSection hundredMeterSection;
    hundredMeterSection.points = {{-50.0, 0.0, 0.0, 0.0}, {50.0, 0.0, 1.0, 0.0}};
    const auto hundredMeter = openbus::map::buildMapRoadSectionGeometry(
        fivePercentSpline, bankSamples, hundredMeterSection);
    requireNear(hundredMeter.vertices[1].z - hundredMeter.vertices[0].z, -5.0,
                "5% cant changes height by 5 m across a 100 m spline width");

    openbus::map::MapSplinePlacement curvedTransition = bankedSpline;
    curvedTransition.length = 100.0;
    curvedTransition.radius = 40.0;
    curvedTransition.gradientStart = 0.0;
    curvedTransition.gradientEnd = 10.0;
    curvedTransition.cantStart = 0.0;
    curvedTransition.cantEnd = 10.0;
    openbus::map::MapSplineProfileSection curvedTransitionSection;
    curvedTransitionSection.points = {{-2.0, 0.0, 0.0, 0.0}, {2.0, 0.0, 1.0, 0.0}};
    std::vector<openbus::map::MapSplineSample> curvedTransitionSamples;
    for (const double distance : {0.0, 25.0, 50.0, 75.0, 100.0}) {
        const auto sample = openbus::map::sampleMapSpline(
            0.0, 0.0, 0.0, curvedTransition.length, curvedTransition.radius, distance);
        require(sample.has_value(), "curved spline sample is valid at each length-based station");
        curvedTransitionSamples.push_back(*sample);
    }
    const auto curvedTransitionGeometry = openbus::map::buildMapRoadSectionGeometry(
        curvedTransition, curvedTransitionSamples, curvedTransitionSection);
    for (std::size_t station = 0; station < curvedTransitionSamples.size(); ++station) {
        const double distance = curvedTransitionSamples[station].distance;
        const double t = distance / curvedTransition.length;
        const double expectedCant = std::lerp(curvedTransition.cantStart,
                                              curvedTransition.cantEnd, t);
        const std::size_t firstVertex = station * curvedTransitionSection.points.size();
        const double edgeHeightDifference =
            curvedTransitionGeometry.vertices[firstVertex + 1].z -
            curvedTransitionGeometry.vertices[firstVertex].z;
        requireNear(edgeHeightDifference, -4.0 * expectedCant / 100.0,
                    "cant transition uses segment distance divided directly by spline length");
        const double centerHeight =
            (curvedTransitionGeometry.vertices[firstVertex + 1].z +
             curvedTransitionGeometry.vertices[firstVertex].z) * 0.5;
        requireNear(centerHeight,
                    openbus::map::mapSplineElevation(0.0, curvedTransition.length,
                                                     curvedTransition.gradientStart,
                                                     curvedTransition.gradientEnd,
                                                     std::nullopt, distance),
                    "gradient transition follows the same spline-length fraction");
    }
    openbus::map::MapSplinePlacement shallowerCurve = curvedTransition;
    shallowerCurve.radius = 200.0;
    std::vector<openbus::map::MapSplineSample> shallowerCurveSamples;
    for (const openbus::map::MapSplineSample& sample : curvedTransitionSamples) {
        const auto alternate = openbus::map::sampleMapSpline(
            0.0, 0.0, 0.0, shallowerCurve.length, shallowerCurve.radius, sample.distance);
        require(alternate.has_value(), "alternate-radius spline sample is valid");
        shallowerCurveSamples.push_back(*alternate);
    }
    const auto shallowerCurveGeometry = openbus::map::buildMapRoadSectionGeometry(
        shallowerCurve, shallowerCurveSamples, curvedTransitionSection);
    for (std::size_t vertex = 0; vertex < curvedTransitionGeometry.vertices.size(); ++vertex) {
        requireNear(shallowerCurveGeometry.vertices[vertex].z,
                    curvedTransitionGeometry.vertices[vertex].z,
                    "cant and gradient transitions are not rescaled by curve radius");
    }

    spline.chainOffsetValid = false;
    spline.chainOffset = 0.0;
    spline.radius = 20.0;
    spline.length = 10.0;
    const auto curvedCenterline =
        openbus::map::tessellateMapSpline(100.0, 200.0, 0.0, spline.length, spline.radius);
    const auto curved =
        openbus::map::buildMapRoadSectionGeometry(spline, curvedCenterline, section);
    require(curved.vertices.size() == curvedCenterline.size() * 2,
            "curved cross-section follows each centerline sample");
    require(std::abs(curved.vertices.back().x - straight.vertices.back().x) > 0.1,
            "curved road vertices follow the curved centerline");

    openbus::map::MapSplinePlacement mirroredSpline = spline;
    mirroredSpline.mirrored = true;
    const auto mirroredCurved = openbus::map::buildMapRoadSectionGeometry(
        mirroredSpline, curvedCenterline, section);
    require(mirroredCurved.vertices.size() == curved.vertices.size(),
            "mirroring preserves curved cross-section vertex count");
    for (std::size_t sampleIndex = 0; sampleIndex < curvedCenterline.size(); ++sampleIndex) {
        const std::size_t profileBase = sampleIndex * section.points.size();
        for (std::size_t pointIndex = 0; pointIndex < section.points.size(); ++pointIndex) {
            const openbus::map::MapRoadVertex& original =
                curved.vertices[profileBase + pointIndex];
            const openbus::map::MapRoadVertex& reflected =
                mirroredCurved.vertices[profileBase + pointIndex];
            requireNear((original.x + reflected.x) * 0.5, curvedCenterline[sampleIndex].x,
                        "mirrored curve reflects profile across centerline X");
            requireNear((original.y + reflected.y) * 0.5, curvedCenterline[sampleIndex].y,
                        "mirrored curve reflects profile across centerline Y");
            requireNear(reflected.z, original.z,
                        "mirroring leaves authored profile height unchanged");
            requireNear(reflected.textureU, original.textureU,
                        "mirroring preserves authored cross-section texture coordinates");
        }
    }
    require(mirroredCurved.indices.size() == curved.indices.size() &&
                mirroredCurved.indices[0] == curved.indices[0] &&
                mirroredCurved.indices[1] == curved.indices[2] &&
                mirroredCurved.indices[2] == curved.indices[1],
            "mirrored strips reverse triangle winding to preserve surface orientation");

    spline.elevated = true;
    spline.heightDelta = 4.0;
    spline.gradientStart = 0.0;
    spline.gradientEnd = 0.0;
    const auto elevated =
        openbus::map::buildMapRoadSectionGeometry(spline, curvedCenterline, section);
    requireNear(elevated.vertices.back().z, 6.25,
                "elevated spline height profile is reflected in road geometry");

    spline.geometryValid = false;
    require(openbus::map::buildMapRoadSectionGeometry(spline, curvedCenterline, section)
                .vertices.empty(),
            "invalid spline placement does not create road geometry");
}

void runHeightProfileCantProbe() {
    using namespace openbus::map;
    constexpr double cantHalfWidth = 0.06;
    MapSplinePlacement ramp;
    ramp.geometryValid = true;
    ramp.length = 3.0;
    ramp.cantStart = ramp.cantEnd = -120.0;
    MapSplineProfileSection section;
    section.points = {{-1.35, 0.0, 0.0, 0.0}, {1.5, 0.0, 1.0, 0.0}};
    const auto rampSamples = tessellateMapSpline(0.0, 1.0, 0.0, ramp.length, 0.0);
    const auto clampedRamp =
        buildMapRoadSectionGeometry(ramp, rampSamples, section, {}, cantHalfWidth);
    for (std::size_t station = 0; station < rampSamples.size(); ++station) {
        const std::size_t firstVertex = station * section.points.size();
        requireNear(clampedRamp.vertices[firstVertex].z, -0.072,
                    "-120% cant is limited to 7.2 cm at the negative profile edge");
        requireNear(clampedRamp.vertices[firstVertex + 1].z, 0.072,
                    "-120% cant is limited to 7.2 cm at the positive profile edge");
    }
    const auto unboundedRamp = buildMapRoadSectionGeometry(ramp, rampSamples, section);
    requireNear(unboundedRamp.vertices[0].z, -1.62,
                "profiles without a height-profile width retain the legacy crossfall");
    requireNear(unboundedRamp.vertices[1].z, 1.8,
                "unbounded crossfall remains distinct from the height-profile clamp");

    MapSplinePlacement previous = ramp;
    previous.splineId = 1;
    previous.nextSplineId = 2;
    previous.localY = 0.0;
    previous.length = 1.0;
    previous.cantStart = 0.0;
    MapSplinePlacement linkedRamp = ramp;
    linkedRamp.splineId = 2;
    linkedRamp.previousSplineId = 1;
    linkedRamp.nextSplineId = 3;
    linkedRamp.localY = 1.0;
    MapSplinePlacement next = ramp;
    next.splineId = 3;
    next.previousSplineId = 2;
    next.localY = 4.0;
    next.length = 1.0;
    next.cantEnd = 0.0;

    MapRoadConnectionIndex connections;
    connections.addSpline(previous, 0, 0, cantHalfWidth);
    connections.addSpline(linkedRamp, 0, 0, cantHalfWidth);
    connections.addSpline(next, 0, 0, cantHalfWidth);
    connections.finalize();
    const auto previousSamples = tessellateMapSpline(0.0, 0.0, 0.0, 1.0, 0.0);
    const auto nextSamples = tessellateMapSpline(0.0, 4.0, 0.0, 1.0, 0.0);
    const auto previousGeometry = buildMapRoadSectionGeometry(
        previous, previousSamples, section, connections.joinsFor(previous, 0, 0), cantHalfWidth);
    const auto rampGeometry = buildMapRoadSectionGeometry(
        linkedRamp, rampSamples, section, connections.joinsFor(linkedRamp, 0, 0), cantHalfWidth);
    const auto nextGeometry = buildMapRoadSectionGeometry(
        next, nextSamples, section, connections.joinsFor(next, 0, 0), cantHalfWidth);
    for (std::size_t point = 0; point < section.points.size(); ++point) {
        requireSamePosition(previousGeometry.vertices[previousGeometry.vertices.size() -
                                                       section.points.size() + point],
                            rampGeometry.vertices[point]);
        requireSamePosition(rampGeometry.vertices[rampGeometry.vertices.size() -
                                                   section.points.size() + point],
                            nextGeometry.vertices[point]);
    }
}

void runMixedWidthJoinProbe() {
    using namespace openbus::map;
    MapSplinePlacement a;
    a.geometryValid = true;
    a.splineId = 1;
    a.nextSplineId = 2;
    a.length = 10.0;
    a.cantStart = a.cantEnd = -120.0;
    MapSplinePlacement b = a;
    b.splineId = 2;
    b.previousSplineId = 1;
    b.nextSplineId = 0;
    b.localY = 10.0;
    MapSplineProfileSection section;
    section.points = {{-1.5, 0.25, 0.0, 0.2}, {-0.08, 0.25, 0.3, 0.2},
                      {0.0, 0.25, 0.5, 0.2}, {0.08, 0.25, 0.7, 0.2},
                      {1.5, 0.25, 1.0, 0.2}};
    const auto aSamples = tessellateMapSpline(0.0, 0.0, 0.0, a.length, 0.0);
    for (const bool reversed : {false, true}) {
        b.previousSplineId = reversed ? 0 : 1;
        b.nextSplineId = reversed ? 1 : 0;
        b.localY = reversed ? 20.0 : 10.0;
        b.rotationDegrees = reversed ? 180.0 : 0.0;
        b.mirrored = reversed;
        b.cantStart = b.cantEnd = reversed ? 120.0 : -120.0;
        const auto bSamples = tessellateMapSpline(0.0, b.localY, b.rotationDegrees, b.length, 0.0);
        for (const std::optional<double> otherWidth :
             {std::optional<double>{0.12}, std::optional<double>{0.0}, std::optional<double>{}}) {
            MapRoadConnectionIndex index;
            index.addSpline(a, 0, 0, 0.06);
            index.addSpline(b, 0, 0, otherWidth);
            index.finalize();
            const auto aJoins = index.joinsFor(a, 0, 0);
            const auto bJoins = index.joinsFor(b, 0, 0);
            require(aJoins.end.has_value() && (reversed ? bJoins.end : bJoins.start).has_value(),
                    "mixed-width joins resolve in both traversal directions");
            const auto first = buildMapRoadSectionGeometry(a, aSamples, section, aJoins, 0.06);
            const auto second = buildMapRoadSectionGeometry(b, bSamples, section, bJoins, otherWidth);
            const std::size_t aBase = first.vertices.size() - section.points.size();
            const std::size_t bBase = reversed ? second.vertices.size() - section.points.size() : 0;
            for (std::size_t p = 0; p < section.points.size(); ++p) {
                requireSamePosition(first.vertices[aBase + p], second.vertices[bBase + p]);
                const double x = section.points[p].lateral;
                const double expected = section.points[p].height +
                    0.5 * (mapSplineCantHeightOffset(x, -120.0, 0.06) +
                           mapSplineCantHeightOffset(x, -120.0, otherWidth));
                requireNear(first.vertices[aBase + p].z, expected,
                            "seam averages the two independently clamped displacements");
            }
            // No load-order dependence, and no change outside the seam fade.
            MapRoadConnectionIndex reverseIndex;
            reverseIndex.addSpline(b, 0, 0, otherWidth);
            reverseIndex.addSpline(a, 0, 0, 0.06);
            reverseIndex.finalize();
            const auto reorder = buildMapRoadSectionGeometry(
                a, aSamples, section, reverseIndex.joinsFor(a, 0, 0), 0.06);
            requireSamePosition(first.vertices.back(), reorder.vertices.back());
            const auto independent = buildMapRoadSectionGeometry(a, aSamples, section, {}, 0.06);
            requireSamePosition(first.vertices[2 * section.points.size()],
                                independent.vertices[2 * section.points.size()]);
        }
    }
    for (const double badWidth : {-1.0, std::numeric_limits<double>::infinity(),
                                 std::numeric_limits<double>::quiet_NaN()}) {
        bool rejected = false;
        try {
            MapRoadConnectionIndex index;
            index.addSpline(a, 0, 0, badWidth);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "connection metadata rejects negative/non-finite cant widths");
    }
}

} // namespace

int main() {
    try {
        runProbe();
        runSkewProbe();
        runJoinProbe();
        runAuthoredSkewJoinProbe();
        runHeightProfileCantProbe();
        runMixedWidthJoinProbe();
    } catch (const std::exception& error) {
        std::cerr << "map road geometry probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
