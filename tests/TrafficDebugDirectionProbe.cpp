#include "app/TrafficDebugDirection.h"
#include "map/MapConfigLoader.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireNear(double actual, double expected, const char* message) {
    require(std::abs(actual - expected) < 1.0e-8, message);
}

void checkPath(const openbus::map::MapTrafficRouteSegment& source,
               const openbus::map::MapSplinePathOffset& path, double expectedX,
               double expectedYDirection) {
    const auto route = openbus::app::trafficDebugSplineRoute(source, path);
    const auto first = openbus::map::sampleMapTrafficRoute(route, 0.0, 0.0);
    const auto next = openbus::map::sampleMapTrafficRoute(route, 1.0, 0.0);
    require(first && next, "lane can be sampled at spawn");
    requireNear(first->position[0], expectedX, "debug path retains authored lane position");
    requireNear(first->position[2], path.heightMeters, "debug path applies height exactly once");
    require((next->position[1] - first->position[1]) * expectedYDirection > 0.0,
            "lane advances in the authored direction");
    const double heading = openbus::map::mapTrafficRouteTravelHeadingRadians(route, 0.0, 0.0);
    const double yaw = openbus::map::mapTrafficHeadingToVehicleYawDegrees(heading);
    const double forwardX = std::cos(yaw * std::numbers::pi / 180.0);
    const double forwardY = std::sin(yaw * std::numbers::pi / 180.0);
    require((next->position[0] - first->position[0]) * forwardX +
                (next->position[1] - first->position[1]) * forwardY >
            0.0,
            "spawn yaw faces increasing route distance");
    const auto arrow = openbus::app::trafficDirectionArrow(first->position, heading, 2.0);
    require(arrow.has_value(), "valid route produces an arrow");
    require(((*arrow)[1][1] - (*arrow)[0][1]) * expectedYDirection > 0.0,
            "arrow points along vehicle travel");
}

void checkOppositeCarriageways(const openbus::map::MapSplineProfile& profile,
                              const openbus::map::MapTrafficRouteSegment& normal,
                              const openbus::map::MapTrafficRouteSegment& mirrored) {
    const auto normalRoad = openbus::map::mapSplineProfilePathOffsets(profile, 0, false);
    const auto mirroredRoad = openbus::map::mapSplineProfilePathOffsets(profile, 0, true);
    const auto authoredRoad = openbus::map::mapSplineProfilePathOffsets(
        profile, 0, true, openbus::map::MirroredRoadDirectionPolicy::Authored);
    require(normalRoad.size() == 3 && mirroredRoad.size() == 3 && authoredRoad.size() == 3,
            "parallel carriageways each have three road lanes");
    for (std::size_t index = 0; index < normalRoad.size(); ++index) {
        require(!normalRoad[index].reversed && mirroredRoad[index].reversed &&
                    !authoredRoad[index].reversed,
                "mirrored one-way road lane reverses unless authored mode is selected");
        const auto normalRoute = openbus::app::trafficDebugSplineRoute(normal, normalRoad[index]);
        const auto mirroredRoute =
            openbus::app::trafficDebugSplineRoute(mirrored, mirroredRoad[index]);
        const double normalHeading =
            openbus::map::mapTrafficRouteTravelHeadingRadians(normalRoute, 1.0, 0.0);
        const double mirroredHeading =
            openbus::map::mapTrafficRouteTravelHeadingRadians(mirroredRoute, 1.0, 0.0);
        require(std::cos(normalHeading - mirroredHeading) < -0.999999,
                "normal and mirrored road lanes face opposite directions");
    }
    const auto normalHuman = openbus::map::mapSplineProfilePathOffsets(profile, 1, false);
    const auto mirroredHuman = openbus::map::mapSplineProfilePathOffsets(profile, 1, true);
    require(normalHuman.size() == 1 && mirroredHuman.size() == 1 &&
                openbus::app::trafficDebugHasBidirectionalPath(normalHuman) &&
                openbus::app::trafficDebugHasBidirectionalPath(mirroredHuman),
            "both pedestrian paths are labeled bidirectional");
    require(!openbus::app::trafficDebugHasBidirectionalPath(normalRoad),
            "one-way road paths are not labeled bidirectional");
    require(!normalHuman[0].reversed && !mirroredHuman[0].reversed &&
                normalHuman[0].lateralMeters == -mirroredHuman[0].lateralMeters,
            "pedestrian paths remain bidirectional and mirror their offsets");
}

} // namespace

int main(int argc, char** argv) {
    try {
        openbus::map::MapTrafficRouteSegment source;
        source.spline.placement.length = 20.0;
        source.spline.placement.geometryValid = true;
        source.spline.placement.splineId = 1;
        source.pathLateralMeters = 99.0;
        source.pathHeightMeters = 99.0;
        source.splineDistanceAtRouteStartMeters = 5.0;
        const openbus::map::MapSplinePathOffset north{0, 1.625, 0.1, false, false};
        const openbus::map::MapSplinePathOffset south{1, -1.625, 0.1, true, false};
        checkPath(source, north, 1.625, 1.0);
        checkPath(source, south, -1.625, -1.0);
        openbus::map::MapSplineProfile profile;
        profile.paths = {
            {{"0", "-1.625", "0.1", "3", "1"}, {0.0, -1.625, 0.1, 3.0, 1.0}, true, 0},
            {{"0", "1.625", "0.1", "3", "0"}, {0.0, 1.625, 0.1, 3.0, 0.0}, true, 0},
        };
        const auto mirroredPaths = openbus::map::mapSplineProfilePathOffsets(profile, 0, true);
        checkPath(source, mirroredPaths[0], 1.625, 1.0);
        checkPath(source, mirroredPaths[1], -1.625, -1.0);
        openbus::map::MapTrafficLaneNetwork joinedLanes;
        auto firstSegment = openbus::app::trafficDebugSplineRoute(source, north).segments.front();
        auto secondSegment =
            openbus::app::trafficDebugSplineRoute(source, mirroredPaths[0]).segments.front();
        secondSegment.spline.placement.splineId = 2;
        secondSegment.spline.placement.localY = 20.0;
        joinedLanes.addSegment(firstSegment);
        joinedLanes.addSegment(secondSegment);
        joinedLanes.finalize();
        const auto joinedRoute = joinedLanes.buildRoute(1);
        require(joinedRoute && joinedRoute->segments.size() == 2,
                "corrected mirrored lane joins an unmirrored continuation");
        auto bidirectional = north;
        bidirectional.bidirectional = true;
        const auto forward = openbus::app::trafficDebugSplineRoute(source, bidirectional);
        const auto reversed = openbus::map::reverseMapTrafficRouteSegment(
            forward.segments.front(), forward.totalLength);
        auto opposite = forward;
        opposite.segments.front() = reversed;
        const auto start = openbus::map::sampleMapTrafficRoute(opposite, 0.0, 0.0);
        const auto next = openbus::map::sampleMapTrafficRoute(opposite, 1.0, 0.0);
        require(start && next && next->position[1] < start->position[1],
                "bidirectional opposite route retains the same lane and reverses travel");
        requireNear(start->position[0], 1.625, "bidirectional reverse keeps lane position");
        require(!openbus::app::trafficDirectionArrow({0, 0, 0}, 0, 0) &&
                    !openbus::app::trafficDirectionArrow(
                        {0, 0, 0}, std::numeric_limits<double>::quiet_NaN(), 1),
                "invalid arrow geometry is rejected");
        openbus::map::MapSplineProfile carriagewayProfile;
        carriagewayProfile.paths = {
            {{"0", "1.45", "0.1", "2.929", "0"}, {0, 1.45, 0.1, 2.929, 0}, true, 0},
            {{"0", "4.4", "0.1", "3.03", "0"}, {0, 4.4, 0.1, 3.03, 0}, true, 0},
            {{"0", "7.4", "0.1", "3.03", "0"}, {0, 7.4, 0.1, 3.03, 0}, true, 0},
            {{"1", "11.9", "0.25", "1.664", "2"}, {1, 11.9, 0.25, 1.664, 2}, true, 1},
        };
        auto mirroredSource = source;
        mirroredSource.spline.placement.mirrored = true;
        checkOppositeCarriageways(carriagewayProfile, source, mirroredSource);
        if (argc == 4 && std::string(argv[1]) == "--brt-wilhelm") {
            openbus::map::MapTileReference tileReference;
            tileReference.x = -13;
            tileReference.y = 12;
            tileReference.textPath = argv[2];
            const auto tile = openbus::map::loadMapTile(tileReference);
            const auto findSpline = [&](int id) {
                return std::find_if(tile.splines.begin(), tile.splines.end(),
                                    [id](const auto& spline) { return spline.splineId == id; });
            };
            const auto normalSpline = findSpline(8299355);
            const auto mirroredSpline = findSpline(8299357);
            require(normalSpline != tile.splines.end() && mirroredSpline != tile.splines.end(),
                    "both reported BRT splines exist in the supplied tile");
            require(!normalSpline->mirrored && mirroredSpline->mirrored,
                    "only spline 8299357 is mirrored");
            auto normalSegment = source;
            normalSegment.spline.placement = *normalSpline;
            auto mirroredSegment = source;
            mirroredSegment.spline.placement = *mirroredSpline;
            const auto installedProfile = openbus::map::loadMapSplineProfile(argv[3]);
            checkOppositeCarriageways(installedProfile, normalSegment, mirroredSegment);
        } else if (argc == 2) {
            const auto installedProfile = openbus::map::loadMapSplineProfile(argv[1]);
            const auto paths = openbus::map::mapSplineProfilePathOffsets(installedProfile, 0);
            require(paths.size() == 2, "representative BRT road has two vehicle lanes");
            require(paths[0].reversed && !paths[1].reversed,
                    "representative BRT road has opposing authored directions");
            checkPath(source, paths[0], -1.625, -1.0);
            checkPath(source, paths[1], 1.625, 1.0);
            const auto installedMirroredPaths =
                openbus::map::mapSplineProfilePathOffsets(installedProfile, 0, true);
            checkPath(source, installedMirroredPaths[0], 1.625, 1.0);
            checkPath(source, installedMirroredPaths[1], -1.625, -1.0);
        }
    } catch (const std::exception& error) {
        std::cerr << "Traffic direction probe: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
