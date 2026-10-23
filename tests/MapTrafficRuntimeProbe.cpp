#include "map/MapTrafficRuntime.h"
#include "map/MapSplineGeometry.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

openbus::map::MapSplinePlacement makeSpline(int id, int previousId, int nextId, double startY) {
    openbus::map::MapSplinePlacement spline;
    spline.splineId = id;
    spline.previousSplineId = previousId;
    spline.nextSplineId = nextId;
    spline.localX = 5.0;
    spline.localY = startY;
    spline.rotationDegrees = 0.0;
    spline.length = 10.0;
    spline.elevation = 2.0;
    spline.geometryValid = true;
    return spline;
}

void requireNear(double actual, double expected, const char* message) {
    if (std::abs(actual - expected) > 1.0e-8) {
        throw std::runtime_error(message);
    }
}

void verifyDrivingBehavior() {
    using namespace openbus::map;
    const auto straight = [](int id, double x, double y, double heading, double length) {
        MapTrafficRouteSegment segment{{makeSpline(id, 0, 0, y), 0, 0}};
        segment.spline.placement.localX = x;
        segment.spline.placement.elevation = 0.0;
        segment.spline.placement.rotationDegrees = heading;
        segment.spline.placement.length = length;
        return segment;
    };
    const auto oneSegment = [](const MapTrafficRouteSegment& segment) {
        MapTrafficRoute route;
        route.segments = {segment};
        route.totalLength = segment.spline.placement.length;
        route.cumulativeEndDistances = {route.totalLength};
        return route;
    };
    MapTrafficRouteSegment longStraight = straight(1899, 12.0, -7.0, 23.0, 35.0);
    longStraight.trafficSignal = MapTrafficSignalSchedule{10.0, {{0, 5.0}, {6, 5.0}}};
    const MapTrafficRoute unsplitStraightRoute = oneSegment(longStraight);
    const MapTrafficRoute splitStraightRoute = subdivideMapTrafficRoute(unsplitStraightRoute);
    require(splitStraightRoute.segments.size() == 3 &&
                splitStraightRoute.cumulativeEndDistances[0] == 10.0 &&
                splitStraightRoute.cumulativeEndDistances[1] == 20.0 &&
                splitStraightRoute.segments.front().trafficSignal.has_value() &&
                !splitStraightRoute.segments[1].trafficSignal.has_value() &&
                !splitStraightRoute.segments[2].trafficSignal.has_value(),
            "long straight splines split at 10 m intervals with signals kept at the route start");
    for (const double distance : {0.0, 3.0, 10.0, 20.0, 28.0, 35.0}) {
        const auto originalPose = sampleMapTrafficRoute(unsplitStraightRoute, distance, 0.75);
        const auto splitPose = sampleMapTrafficRoute(splitStraightRoute, distance, 0.75);
        require(originalPose && splitPose &&
                    std::hypot(originalPose->position[0] - splitPose->position[0],
                               originalPose->position[1] - splitPose->position[1],
                               originalPose->position[2] - splitPose->position[2]) < 1.0e-8,
                "safe straight-spline subdivisions preserve their original positions");
        require(std::abs(std::remainder(originalPose->headingRadians - splitPose->headingRadians,
                                        2.0 * std::acos(-1.0))) < 1.0e-8,
                "safe straight-spline subdivisions preserve their original headings");
    }
    MapTrafficRouteSegment curvedSpline = straight(1898, 0.0, 0.0, 0.0, 35.0);
    curvedSpline.spline.placement.radius = 50.0;
    require(subdivideMapTrafficRoute(oneSegment(curvedSpline)).segments.size() == 1,
            "curved splines remain intact to avoid artificial heading transitions");
    MapTrafficRouteSegment junctionPath;
    junctionPath.points = {{0.0, 0.0, 0.0}, {0.0, 4.0, 0.0}};
    MapTrafficRouteSegment divergingStraight = straight(1897, 0.0, 0.0, 90.0, 30.0);
    MapTrafficRoute mergeRoute;
    mergeRoute.segments = {longStraight, junctionPath, divergingStraight};
    mergeRoute.cumulativeEndDistances = {35.0, 39.0, 69.0};
    mergeRoute.totalLength = 69.0;
    const MapTrafficRoute safeMergeRoute = subdivideMapTrafficRoute(mergeRoute);
    require(safeMergeRoute.segments.size() == 7 &&
                safeMergeRoute.cumulativeEndDistances[1] == 20.0 &&
                safeMergeRoute.cumulativeEndDistances[2] == 35.0 &&
                safeMergeRoute.segments[3].points.size() == 2 &&
                safeMergeRoute.cumulativeEndDistances[4] == 49.0 &&
                safeMergeRoute.cumulativeEndDistances[5] == 59.0,
            "merge approaches stay unsplit within 5 m while downstream diverging roads may split");
    const auto originalRouteSpawnCandidates = mapTrafficRouteSpawnCandidates(mergeRoute, 4.5);
    const auto subdividedRouteSpawnCandidates = mapTrafficRouteSpawnCandidates(safeMergeRoute, 4.5);
    require(originalRouteSpawnCandidates.size() == 2 &&
                subdividedRouteSpawnCandidates.size() == originalRouteSpawnCandidates.size(),
            "route validity still finds entry candidates after safe subdivision");
    for (std::size_t index = 0; index < originalRouteSpawnCandidates.size(); ++index) {
        requireNear(subdividedRouteSpawnCandidates[index].distanceMeters,
                    originalRouteSpawnCandidates[index].distanceMeters,
                    "subdivision preserves spawn candidates on the original spline span");
        require(subdividedRouteSpawnCandidates[index].tileX ==
                        originalRouteSpawnCandidates[index].tileX &&
                    subdividedRouteSpawnCandidates[index].tileY ==
                        originalRouteSpawnCandidates[index].tileY,
                "subdivision preserves spawn-candidate tile ownership");
    }
    const auto footprint = [](const MapTrafficActor& actor) {
        return MapTrafficObstacle{
            *sampleMapTrafficRoute(actor.route, actor.distanceMeters, actor.lateralOffsetMeters),
            actor.lengthMeters, actor.widthMeters};
    };
    require(mapTrafficPoseOnActiveTile({{10.0, 10.0, 0.0}, 0.0, 0}, {{0, 0}}) &&
                !mapTrafficPoseOnActiveTile({{OMSI_TILE_SIZE_METERS * 2.1, 10.0, 0.0}, 0.0, 0},
                                            {{0, 0}}),
            "map traffic tile gating admits only poses in the active tile set");
    MapTrafficSignalSchedule cycleSignal;
    cycleSignal.cycleDurationSeconds = 10.0;
    cycleSignal.phases = {{0, 3.0}, {6, 5.0}, {9, 2.0}};
    require(mapTrafficSignalStateAtTime(cycleSignal, 0.0) == 0 &&
                mapTrafficSignalStateAtTime(cycleSignal, 3.0) == 6 &&
                mapTrafficSignalStateAtTime(cycleSignal, 8.0) == 9 &&
                mapTrafficSignalStateAtTime(cycleSignal, 10.0) == 0,
            "signal phase schedule wraps at the configured cycle duration");
    require(!mapTrafficSignalStateAtTime(MapTrafficSignalSchedule{9.0, {{0, 3.0}, {6, 5.0}}, false},
                                         0.0)
                 .has_value(),
            "signal schedules whose phases do not cover the cycle are rejected");
    MapTrafficRouteSegment signalWaitSegment = straight(1901, 0.0, 0.0, 0.0, 10.0);
    signalWaitSegment.trafficSignal = cycleSignal;
    MapTrafficActor signalWaiter;
    signalWaiter.route = oneSegment(signalWaitSegment);
    require(mapTrafficActorWaitingAtSignal(signalWaiter, 0.0) &&
                !mapTrafficActorWaitingAtSignal(signalWaiter, 3.0),
            "stuck diagnostics distinguish red-signal waits from stopped traffic");
    const auto redSignalStop = mapTrafficNextSignalStopDebugInfo(signalWaiter, 0.0);
    const auto greenSignalStop = mapTrafficNextSignalStopDebugInfo(signalWaiter, 3.0);
    require(redSignalStop.has_value() && redSignalStop->blocked && greenSignalStop.has_value() &&
                !greenSignalStop->blocked,
            "traffic signal debug data exposes the stop-line pose and active block state");
    auto heldSignalProgram = std::make_shared<MapTrafficSignalProgram>();
    heldSignalProgram->cycleDurationSeconds = 10.0;
    heldSignalProgram->lights = {{{6, 5.0}, {0, 5.0}}};
    heldSignalProgram->approachDistancesMeters = {std::nullopt};
    heldSignalProgram->controlPoints = {{0, 3.0, true, std::nullopt}};
    MapTrafficSignalController heldSignal(heldSignalProgram);
    heldSignal.advance(4.0, 0.0);
    requireNear(heldSignal.clockSeconds(), 3.0,
                "a conditional stop holds the clock when no vehicle requests its light");
    heldSignal.clearRequests();
    heldSignal.request(0);
    heldSignal.advance(1.0, 0.0);
    requireNear(heldSignal.clockSeconds(), 4.0,
                "a conditional stop releases when a vehicle requests its light");
    auto requestingController = std::make_shared<MapTrafficSignalController>(heldSignalProgram);
    MapTrafficSignalSchedule requestedSchedule;
    requestedSchedule.cycleDurationSeconds = 10.0;
    requestedSchedule.phases = heldSignalProgram->lights[0];
    requestedSchedule.hasConditionalOverrides = true;
    requestedSchedule.approachDistanceMeters = 0.0;
    requestedSchedule.program = heldSignalProgram;
    requestedSchedule.controller = requestingController;
    MapTrafficRouteSegment requestedSignalPath = straight(1900, 0.0, 0.0, 0.0, 10.0);
    requestedSignalPath.trafficSignal = requestedSchedule;
    auto requestingActor =
        makeMapTrafficActor(oneSegment(requestedSignalPath), "requesting-car.ovh", 0.0, 0.0, false);
    advanceMapTrafficSignalControllers({requestingController}, {&requestingActor}, 4.0, 0.0);
    requireNear(requestingController->clockSeconds(), 4.0,
                "a stopped actor at the linked signal line submits a traffic-light request");
    require(mapTrafficSignalStateAtTime(requestedSchedule, 4.0) == 6,
            "the active conditional phase is exposed for scenery signal rendering");

    auto jumpedSignalProgram = std::make_shared<MapTrafficSignalProgram>();
    jumpedSignalProgram->cycleDurationSeconds = 10.0;
    jumpedSignalProgram->lights = {{{6, 9.0}, {0, 1.0}}};
    jumpedSignalProgram->approachDistancesMeters = {std::nullopt};
    jumpedSignalProgram->controlPoints = {{0, 4.0, true, 8.0}};
    MapTrafficSignalController jumpedSignal(jumpedSignalProgram);
    jumpedSignal.advance(5.0, 0.0);
    requireNear(jumpedSignal.clockSeconds(), 9.0,
                "a conditional jump skips to its target and consumes the remaining frame time");
    MapTrafficSignalController requestedJump(jumpedSignalProgram);
    requestedJump.request(0);
    requestedJump.advance(5.0, 0.0);
    requireNear(requestedJump.clockSeconds(), 5.0,
                "an if-request jump is skipped while a vehicle requests its light");

    MapTrafficLaneNetwork junction;
    junction.addSegment(straight(1001, 0.0, -40.0, 0.0, 40.0));
    junction.addSegment(straight(1002, 0.0, 0.0, 0.0, 40.0));
    for (const double radius : {-10.0, 10.0}) {
        MapTrafficRouteSegment turn;
        const double length = std::acos(-1.0) * 5.0;
        for (const auto& sample : tessellateMapSpline(0.0, 0.0, 0.0, length, radius)) {
            turn.points.push_back({sample.x, sample.y, 0.0});
        }
        junction.addSegment(turn);
        junction.addSegment(
            straight(radius < 0 ? 1003 : 1004, radius, 10.0, radius < 0 ? -90.0 : 90.0, 60.0));
    }
    junction.finalize();
    MapTrafficLaneNetwork splineTransitions;
    splineTransitions.addSegment(straight(1020, 0.0, -20.0, 0.0, 20.0));
    splineTransitions.addSegment(straight(1021, 0.2, 0.0, 10.0, 20.0));
    splineTransitions.finalize();
    const auto transitionRoute = splineTransitions.buildRoute(1020, 0, 0, 4.5);
    require(transitionRoute && transitionRoute->segments.size() == 2,
            "slightly misaligned but feasible spline endpoints remain connected");
    const auto beforeTransition = sampleMapTrafficRoute(*transitionRoute, 19.99, 0.0);
    const auto afterTransition = sampleMapTrafficRoute(*transitionRoute, 20.01, 0.0);
    require(beforeTransition && afterTransition &&
                std::hypot(beforeTransition->position[0] - afterTransition->position[0],
                           beforeTransition->position[1] - afterTransition->position[1]) < 0.05,
            "traffic position remains continuous across adjacent splines");
    const double headingChange = std::abs(std::remainder(
        afterTransition->headingRadians - beforeTransition->headingRadians, 2.0 * std::acos(-1.0)));
    require(headingChange < 0.02 &&
                std::abs(std::remainder(
                    mapTrafficHeadingToVehicleYawDegrees(afterTransition->headingRadians) -
                        mapTrafficHeadingToVehicleYawDegrees(beforeTransition->headingRadians),
                    360.0)) < 1.0,
            "vehicle heading and rendered yaw rotate smoothly across adjacent splines");
    require(junction.entryLanes() == std::vector<MapTrafficLaneStart>{{1001, 0}},
            "only an unconnected incoming lane is an entry, not internal or exit splines");
    MapTrafficLaneNetwork isolatedLane;
    isolatedLane.addSegment(straight(1005, 20.0, 20.0, 0.0, 20.0));
    isolatedLane.finalize();
    require(isolatedLane.entryLanes().empty(),
            "a spline with no connected continuation is never a vehicle spawn entry");
    require(isolatedLane.spawnLanes().empty() &&
                !isolatedLane.selectSpawnLane({0, std::nullopt, false}, 1).has_value(),
            "a spline with no connected continuation is never a road traffic spawn lane");
    MapTrafficLaneNetwork connectedSpawnLanes;
    connectedSpawnLanes.addSegment(straight(1010, 0.0, 0.0, 0.0, 20.0));
    connectedSpawnLanes.addSegment(straight(1011, 0.0, 20.0, 0.0, 20.0));
    connectedSpawnLanes.addSegment(straight(1012, 0.0, 40.0, 0.0, 20.0));
    connectedSpawnLanes.finalize();
    require(connectedSpawnLanes.entryLanes() == std::vector<MapTrafficLaneStart>{{1010, 0}} &&
                connectedSpawnLanes.spawnLanes() ==
                    std::vector<MapTrafficLaneStart>{{1010, 0}, {1011, 0}},
            "road traffic may start on connected internal lanes but not terminal dead ends");
    bool selectedInternalSpawnLane = false;
    for (std::size_t seed = 1; seed <= 64; ++seed) {
        selectedInternalSpawnLane |=
            connectedSpawnLanes.selectSpawnLane({0, std::nullopt, false}, seed) ==
            std::optional<MapTrafficLaneStart>{{1011, 0}};
    }
    require(selectedInternalSpawnLane,
            "seeded road spawn selection can choose a connected internal lane");
    const auto connectedSpawnRoute = connectedSpawnLanes.buildRoute(1010);
    require(connectedSpawnRoute.has_value(),
            "a connected road lane builds a route for interior spawn candidates");
    const auto spawnCandidates = mapTrafficRouteSpawnCandidates(*connectedSpawnRoute, 4.5);
    require(spawnCandidates.size() == 2,
            "road spawn candidates include connected route interiors, not terminal dead ends");
    requireNear(spawnCandidates[0].distanceMeters, 8.0,
                "road spawn candidates keep a vehicle-sized margin from the route start");
    requireNear(spawnCandidates[1].distanceMeters, 28.0,
                "road spawn candidates include interior positions beyond the first lane");
    require(spawnCandidates[0].tileX == 0 && spawnCandidates[0].tileY == 0 &&
                spawnCandidates[1].tileX == 0 && spawnCandidates[1].tileY == 0,
            "road spawn candidates retain their owning tile for streaming admission");
    MapTrafficLaneNetwork sidewalkLane;
    auto sidewalk = straight(1006, 0.0, 0.0, 0.0, 20.0);
    sidewalk.pathLateralMeters = 2.5;
    sidewalk.pathHeightMeters = 0.2;
    sidewalkLane.addSegment(sidewalk);
    sidewalkLane.finalize();
    const auto sidewalkRoute = sidewalkLane.buildRoute(1006);
    const std::optional<MapTrafficPose> sidewalkPose =
        sidewalkRoute ? sampleMapTrafficRoute(*sidewalkRoute, 0.0, 0.0) : std::nullopt;
    require(sidewalkPose.has_value(),
            "pedestrian routes can be built from authored human-path lanes");
    requireNear(sidewalkPose->position[0], 2.5,
                "pedestrian routes preserve the authored sidewalk lateral offset");
    requireNear(sidewalkPose->position[2], 0.2,
                "pedestrian routes preserve the authored path height offset");
    std::optional<MapTrafficRoute> left;
    std::optional<MapTrafficRoute> right;
    bool foundStraight = false;
    for (std::size_t seed = 1; seed <= 12; ++seed) {
        const auto route = junction.buildRoute(1001, 0, seed, 4.5);
        require(route.has_value(), "a normal car can route through an authored junction");
        for (double distance = 0.0; distance < route->totalLength; distance += 0.25) {
            require(mapTrafficCurveSpeedLimit(*route, distance, 12.0, 4.5) > 0.0,
                    "a route accepted for a vehicle never produces a permanent zero curve speed");
        }
        const int exit = route->segments.back().spline.placement.splineId;
        if (exit == 1003)
            left = route;
        if (exit == 1004)
            right = route;
        if (exit == 1002)
            foundStraight = true;
        const auto largeVehicle = junction.buildRoute(1001, 0, seed, 20.0);
        require(largeVehicle && largeVehicle->segments.back().spline.placement.splineId == 1002,
                "vehicles with a 20 m minimum radius reject the 10 m junction turns");
        const auto exactRadius = junction.buildRoute(1001, 0, seed, 10.0);
        require(exactRadius && exactRadius->segments.back().spline.placement.splineId == exit,
                "a vehicle with a 10 m minimum radius can use authored 10 m turns");
    }
    require(left && right && foundStraight,
            "seeded junction choices include left, right, and straight movements");
    MapTrafficRouteSegment sharpCurveSegment;
    for (const auto& sample : tessellateMapSpline(0.0, 0.0, 0.0, std::acos(-1.0) * 0.25, 0.5)) {
        sharpCurveSegment.points.push_back({sample.x, sample.y, 0.0});
    }
    double sharpCurveLength = 0.0;
    for (std::size_t index = 1; index < sharpCurveSegment.points.size(); ++index) {
        const auto& from = sharpCurveSegment.points[index - 1];
        const auto& to = sharpCurveSegment.points[index];
        sharpCurveLength += std::hypot(to[0] - from[0], to[1] - from[1]);
    }
    MapTrafficRoute sharpCurveRoute;
    sharpCurveRoute.segments.push_back(std::move(sharpCurveSegment));
    sharpCurveRoute.cumulativeEndDistances = {sharpCurveLength};
    sharpCurveRoute.totalLength = sharpCurveLength;
    require(mapTrafficCurveSpeedLimit(sharpCurveRoute, sharpCurveLength * 0.5, 8.0, 4.5) >= 2.0,
            "tight-curve speed limits retain a 2 m/s floor rather than freezing traffic");
    for (const auto* route : {&*left, &*right}) {
        auto actor = makeMapTrafficActor(*route, "car.ovh", 12.0, 0.0, false);
        actor.minimumTurnRadiusMeters = 10.0;
        actor.speedMetersPerSecond = 0.0;
        bool sampledTurn = false;
        bool reachedExit = false;
        for (int frame = 0; frame < 400; ++frame) {
            const double oldSpeed = actor.speedMetersPerSecond;
            advanceMapTrafficDriving({&actor}, {}, 0.1);
            require(actor.speedMetersPerSecond <= oldSpeed + 0.160001 &&
                        actor.speedMetersPerSecond >= oldSpeed - 0.300001,
                    "free driving respects acceleration and comfortable braking limits");
            if (actor.distanceMeters > 40.0 && actor.distanceMeters < 55.0) {
                sampledTurn = true;
                require(actor.speedMetersPerSecond <= 4.5,
                        "10 m radius junction turns are traversed below 4.5 m/s");
            }
            if (actor.distanceMeters > 60.0) {
                reachedExit = true;
                break;
            }
        }
        require(sampledTurn && reachedExit,
                "vehicles actually traverse left/right scenery curves and enter their exit lane");
        const auto exitPose = sampleMapTrafficRoute(actor.route, actor.distanceMeters, 0.0);
        require(std::abs(std::abs(exitPose->headingRadians) - std::acos(-1.0) * 0.5) < 0.01,
                "vehicles leave the turn aligned with the perpendicular exit lane");
    }
    MapTrafficLaneNetwork radiusBlocked;
    radiusBlocked.addSegment(straight(1500, 0.0, -40.0, 0.0, 40.0));
    radiusBlocked.addSegment(straight(1501, 0.0, 0.0, 0.0, 10.0));
    for (const auto& [radius, startY] : {std::pair{5.0, 10.0}, std::pair{40.0, 0.0}}) {
        MapTrafficRouteSegment turn;
        for (const auto& sample :
             tessellateMapSpline(0.0, startY, 0.0, radius * std::acos(-1.0) * 0.5, radius)) {
            turn.points.push_back({sample.x, sample.y, 0.0});
        }
        radiusBlocked.addSegment(turn);
        radiusBlocked.addSegment(
            straight(radius == 5.0 ? 1502 : 1503, radius, startY + radius, 90.0, 60.0));
    }
    radiusBlocked.finalize();
    const auto alternative = radiusBlocked.buildRoute(1500, 0, 0, 20.0);
    require(alternative && alternative->segments.back().spline.placement.splineId == 1503,
            "routing backtracks from an impossible later bend to a feasible branch");
    require(!radiusBlocked.buildRoute(1500, 0, 0, 60.0),
            "no route is spawned when every branch needs an impossible turning radius");
    const auto road = oneSegment(straight(1100, 0.0, 0.0, 0.0, 500.0));
    auto stoppedFollower = makeMapTrafficActor(road, "car.ovh", 12.0, 0.0, false);
    const MapTrafficObstacle playerBus{{{0.0, 60.0, 0.0}, 0.0, 0}, 12.0, 2.5};
    int slowApproachFrames = 0;
    for (int frame = 0; frame < 300; ++frame) {
        const double bumperGap = playerBus.pose.position[1] -
                                 (stoppedFollower.distanceMeters +
                                  (stoppedFollower.lengthMeters + playerBus.lengthMeters) * 0.5);
        if (stoppedFollower.speedMetersPerSecond < 0.5 && bumperGap > 3.0) {
            ++slowApproachFrames;
        }
        advanceMapTrafficDriving({&stoppedFollower}, {playerBus}, 0.1);
        require(mapTrafficHasClearance(footprint(stoppedFollower), {playerBus}, 0.49),
                "a follower never overlaps a stopped player bus");
    }
    const double stoppedBusBumperGap =
        playerBus.pose.position[1] -
        (stoppedFollower.distanceMeters +
         (stoppedFollower.lengthMeters + playerBus.lengthMeters) * 0.5);
    require(stoppedFollower.speedMetersPerSecond < 0.01 && stoppedBusBumperGap >= 0.5 &&
                stoppedBusBumperGap <= 1.5 && slowApproachFrames < 30,
            "a car brakes smoothly to a close, collision-safe standstill behind the bus");
    require(mapTrafficActorBehindObstacle(stoppedFollower, playerBus, 25.0),
            "a stopped traffic actor behind the bus is recognized as a legitimate queue");
    MapTrafficObstacle adjacentBus = playerBus;
    adjacentBus.pose.position[0] = 5.0;
    require(!mapTrafficActorBehindObstacle(stoppedFollower, adjacentBus, 25.0),
            "a bus in an adjacent lane does not excuse a stopped traffic actor");
    requireNear(playerBus.pose.position[1], 60.0,
                "traffic avoidance never changes the player bus state");
    MapTrafficRoute busAtJunctionRoute;
    MapTrafficRouteSegment busAtJunctionConnector;
    busAtJunctionConnector.points = {{0.0, 0.0, 0.0}, {0.0, 10.0, 0.0}};
    busAtJunctionRoute.segments = {
        straight(1101, 0.0, -40.0, 0.0, 40.0), busAtJunctionConnector,
        straight(1102, 0.0, 10.0, 0.0, 100.0)};
    busAtJunctionRoute.cumulativeEndDistances = {40.0, 50.0, 150.0};
    busAtJunctionRoute.totalLength = 150.0;
    auto joiningBusLane =
        makeMapTrafficActor(busAtJunctionRoute, "joining-bus-lane.ovh", 8.0, 0.0, false);
    const MapTrafficObstacle busOnExitLane{{{0.0, 22.5, 0.0}, 0.0, 0}, 12.0, 2.5};
    for (int frame = 0; frame < 500; ++frame) {
        advanceMapTrafficDriving({&joiningBusLane}, {busOnExitLane}, 0.1);
        require(mapTrafficHasClearance(footprint(joiningBusLane), {busOnExitLane}, 0.49),
                "a vehicle entering a bus-occupied road maintains physical clearance");
    }
    const double junctionBusBumperGap =
        busOnExitLane.pose.position[1] -
        (joiningBusLane.distanceMeters - 40.0 +
         (joiningBusLane.lengthMeters + busOnExitLane.lengthMeters) * 0.5);
    require(joiningBusLane.distanceMeters > 52.5 && junctionBusBumperGap >= 0.5 &&
                junctionBusBumperGap <= 1.5,
            "a vehicle clears a junction with one meter of rear clearance and queues close behind a bus");
    MapTrafficActor blockedAtJunction =
        makeMapTrafficActor(busAtJunctionRoute, "blocked-at-junction.ovh", 8.0, 0.0, false);
    const MapTrafficObstacle busBlockingExit{{{0.0, 18.0, 0.0}, 0.0, 0}, 12.0, 2.5};
    for (int frame = 0; frame < 200; ++frame) {
        advanceMapTrafficDriving({&blockedAtJunction}, {busBlockingExit}, 0.1);
    }
    require(blockedAtJunction.distanceMeters < 40.0 &&
                mapTrafficHasClearance(footprint(blockedAtJunction), {busBlockingExit}, 0.49),
            "a vehicle waits before the junction when a stopped bus leaves no room to clear it");
    auto leader = makeMapTrafficActor(road, "slow.ovh", 6.0, 0.0, false);
    auto follower = makeMapTrafficActor(road, "fast.ovh", 12.0, 0.0, false);
    leader.distanceMeters = 30.0;
    for (int frame = 0; frame < 300; ++frame) {
        advanceMapTrafficDriving({&follower, &leader}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(follower), {footprint(leader)}, 0.49),
                "a faster vehicle cannot drive through a slower leader");
        require(follower.distanceMeters < leader.distanceMeters,
                "same-lane vehicles retain their ordering");
    }
    require(follower.distanceMeters > 100.0 && follower.speedMetersPerSecond <= 6.5,
            "following traffic makes progress at the leader's speed");
    const auto steadyFollowingGap = [&](double leaderSpeed) {
        auto speedLeader = makeMapTrafficActor(road, "speed-leader.ovh", leaderSpeed, 0.0, false);
        auto speedFollower = makeMapTrafficActor(road, "speed-follower.ovh", 12.0, 0.0, false);
        speedLeader.distanceMeters = 30.0;
        for (int frame = 0; frame < 300; ++frame) {
            advanceMapTrafficDriving({&speedFollower, &speedLeader}, {}, 0.1);
            require(
                mapTrafficHasClearance(footprint(speedFollower), {footprint(speedLeader)}, 0.49),
                "speed-dependent following gaps retain physical clearance");
        }
        return speedLeader.distanceMeters - speedFollower.distanceMeters;
    };
    const double slowLeaderGap = steadyFollowingGap(3.0);
    const double fastLeaderGap = steadyFollowingGap(8.0);
    require(fastLeaderGap > slowLeaderGap + 4.0,
            "faster traffic maintains a larger following gap than slower traffic");
    const auto followingSpeedAfterLeaderSlows = [&](double centerGapMeters) {
        auto slowingLeader = makeMapTrafficActor(road, "slowing-leader.ovh", 6.0, 0.0, false);
        auto adaptingFollower = makeMapTrafficActor(road, "adapting-follower.ovh", 10.0, 0.0, false);
        slowingLeader.distanceMeters = centerGapMeters;
        adaptingFollower.speedMetersPerSecond = 8.0;
        slowingLeader.speedMetersPerSecond = 6.0;
        for (int frame = 0; frame < 5; ++frame) {
            advanceMapTrafficDriving({&adaptingFollower, &slowingLeader}, {}, 0.1);
            require(mapTrafficHasClearance(footprint(adaptingFollower),
                                           {footprint(slowingLeader)}, 0.49),
                    "adaptive following retains physical clearance while matching a slower leader");
        }
        return adaptingFollower.speedMetersPerSecond;
    };
    const double closeFollowingSpeed = followingSpeedAfterLeaderSlows(20.0);
    const double distantFollowingSpeed = followingSpeedAfterLeaderSlows(36.0);
    require(closeFollowingSpeed + 1.0 < distantFollowingSpeed &&
                closeFollowingSpeed > 0.0,
            "following speed adapts smoothly to a slowing leader according to the available gap");
    {
        const auto adjacentLane = oneSegment(straight(1104, 2.4, 0.0, 0.0, 500.0));
        auto adjacentStoppedCar =
            makeMapTrafficActor(adjacentLane, "adjacent-stopped.ovh", 0.0, 0.0, false);
        adjacentStoppedCar.distanceMeters = 40.0;
        auto passingCar = makeMapTrafficActor(road, "passing.ovh", 10.0, 0.0, false);
        for (int frame = 0; frame < 80; ++frame) {
            advanceMapTrafficDriving({&passingCar, &adjacentStoppedCar}, {}, 0.1);
        }
        require(passingCar.distanceMeters > 60.0 && passingCar.speedMetersPerSecond > 9.0,
                "a stopped vehicle in a narrow adjacent lane is not followed as a leader");
        require(passingCar.blockReason == MapTrafficBlockReason::None,
                "free-flowing traffic reports no blocking constraint");
    }
    {
        auto stoppedLeader = makeMapTrafficActor(road, "stopped-leader.ovh", 0.0, 0.0, false);
        stoppedLeader.distanceMeters = 40.0;
        auto queuedCar = makeMapTrafficActor(road, "queued.ovh", 10.0, 0.0, false);
        for (int frame = 0; frame < 150; ++frame) {
            advanceMapTrafficDriving({&queuedCar, &stoppedLeader}, {}, 0.1);
        }
        require(queuedCar.speedMetersPerSecond < 0.01 &&
                    (queuedCar.blockReason == MapTrafficBlockReason::Leader ||
                     queuedCar.blockReason == MapTrafficBlockReason::Vehicle),
                "a vehicle queued behind a stopped leader reports the leader as its constraint");
    }
    auto closeLeader = makeMapTrafficActor(road, "leader.ovh", 10.0, 0.0, false);
    auto closeFollower = makeMapTrafficActor(road, "follower.ovh", 10.0, 0.0, false);
    closeLeader.distanceMeters = 5.5;
    closeLeader.speedMetersPerSecond = 0.0;
    closeFollower.speedMetersPerSecond = 0.0;
    for (int frame = 0; frame < 150; ++frame) {
        advanceMapTrafficDriving({&closeFollower, &closeLeader}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(closeLeader), {footprint(closeFollower)}, 0.49),
                "a close queue restarts without losing physical clearance");
    }
    require(closeLeader.distanceMeters > 50.0 && closeFollower.distanceMeters > 40.0,
            "a close follower cannot immobilize its leader after a blockage clears");
    auto cutIn = makeMapTrafficActor(road, "car.ovh", 20.0, 0.0, false);
    const MapTrafficObstacle suddenObstacle{{{0.0, 10.0, 0.0}, 0.0, 0}, 4.5, 1.8};
    advanceMapTrafficDriving({&cutIn}, {suddenObstacle}, 2.0);
    require(mapTrafficHasClearance(footprint(cutIn), {suddenObstacle}, 0.49),
            "swept movement prevents tunnelling during a long frame or sudden blockage");
    require(cutIn.speedMetersPerSecond == 0.0, "emergency blockage stops the actor");
    auto pedestrianApproach = makeMapTrafficActor(road, "car.ovh", 20.0, 0.0, false);
    pedestrianApproach.speedMetersPerSecond = 20.0;
    const MapTrafficObstacle pedestrian{{{0.0, 10.0, 0.0}, 0.0, 0}, 0.5, 0.8};
    advanceMapTrafficDriving({&pedestrianApproach}, {pedestrian}, 2.0);
    require(mapTrafficHasClearance(footprint(pedestrianApproach), {pedestrian}, 0.0) &&
                pedestrianApproach.speedMetersPerSecond == 0.0,
            "road AI yields to a pedestrian footprint instead of driving through it");
    auto pedestrianWalker = makeMapTrafficActor(road, "walker.hum", 1.2, 0.0, false);
    const MapTrafficObstacle parkedVehicle{{{0.0, 5.0, 0.0}, 0.0, 0}, 4.5, 1.8};
    require(!advanceMapTrafficActorLoopingAvoidingObstacles(pedestrianWalker, {parkedVehicle}, 0.5,
                                                            0.8, 3.0) &&
                pedestrianWalker.distanceMeters > 0.0 && pedestrianWalker.distanceMeters < 2.6,
            "pedestrians stop before an occupied vehicle footprint");
    require(advanceMapTrafficActorLoopingAvoidingObstacles(pedestrianWalker, {}, 0.5, 0.8, 1.0) &&
                pedestrianWalker.distanceMeters > 2.4,
            "pedestrians resume along their route after the vehicle clears");
    auto pedestrianSignalPath = straight(1199, 20.0, 0.0, 0.0, 20.0);
    MapTrafficSignalSchedule pedestrianRedThenGreen;
    pedestrianRedThenGreen.cycleDurationSeconds = 200.0;
    pedestrianRedThenGreen.phases = {{0, 100.0}, {6, 100.0}};
    pedestrianSignalPath.trafficSignal = pedestrianRedThenGreen;
    MapTrafficLaneNetwork pedestrianSignalNetwork;
    pedestrianSignalNetwork.addSegment(pedestrianSignalPath);
    pedestrianSignalNetwork.finalize();
    const auto pedestrianSignalRoute = pedestrianSignalNetwork.buildRoute(1199);
    require(pedestrianSignalRoute && pedestrianSignalRoute->segments.front().trafficSignal,
            "pedestrian routes retain schedules attached to authored crossing paths");
    auto signalWaitingWalker =
        makeMapTrafficActor(*pedestrianSignalRoute, "walker.hum", 1.2, 0.0, false);
    signalWaitingWalker.lengthMeters = 0.25;
    signalWaitingWalker.widthMeters = 0.42;
    advanceMapTrafficActorLoopingAvoidingObstacles(signalWaitingWalker, {}, 0.25, 0.42, 1.0, 0.0);
    require(signalWaitingWalker.distanceMeters == 0.0 &&
                signalWaitingWalker.speedMetersPerSecond == 0.0,
            "pedestrians wait at a red signal on their authored crossing path");
    advanceMapTrafficActorLoopingAvoidingObstacles(signalWaitingWalker, {}, 0.25, 0.42, 1.0, 100.0);
    require(signalWaitingWalker.distanceMeters > 0.0,
            "pedestrians resume when their crossing path signal turns green");
    auto adjacent = makeMapTrafficActor(road, "car.ovh", 10.0, 0.0, false);
    const MapTrafficObstacle oncoming{{{3.8, 10.0, 0.0}, std::acos(-1.0), 0}, 12.0, 2.5};
    advanceMapTrafficDriving({&adjacent}, {oncoming}, 1.0);
    requireNear(adjacent.speedMetersPerSecond, 10.0,
                "an oncoming bus in the adjacent lane does not falsely block this lane");
    auto north = makeMapTrafficActor(oneSegment(straight(1200, 0.0, -40.0, 0.0, 100.0)),
                                     "north.ovh", 8.0, 0.0, false);
    auto east = makeMapTrafficActor(oneSegment(straight(1201, -40.0, 0.0, 90.0, 100.0)), "east.ovh",
                                    8.0, 0.0, false);
    bool crossingYieldObserved = false;
    for (int frame = 0; frame < 250; ++frame) {
        advanceMapTrafficDriving({&north, &east}, {}, 0.1);
        crossingYieldObserved = crossingYieldObserved || north.junctionWaitSeconds > 0.0 ||
                                east.junctionWaitSeconds > 0.0;
        require(mapTrafficHasClearance(footprint(north), {footprint(east)}, 0.49),
                "conflicting junction movements remain separated");
    }
    require(crossingYieldObserved && north.distanceMeters > 60.0 && east.distanceMeters > 60.0,
            "waiting vehicles gain junction priority and both conflicting movements traverse");
    MapTrafficGroupRules committedPathRules;
    committedPathRules.groupIndex = 0;
    committedPathRules.priority = 192.0;
    MapTrafficGroupRules approachingPathRules;
    approachingPathRules.groupIndex = 0;
    approachingPathRules.priority = 64.0;
    MapTrafficRoute committedJunctionRoute = oneSegment(straight(1204, 0.0, -40.0, 0.0, 100.0));
    committedJunctionRoute.segments.front().points = {{0.0, 0.0, 0.0}, {0.0, 10.0, 0.0}};
    MapTrafficRoute junctionClassificationRoute;
    MapTrafficRouteSegment approachSegment = straight(1206, 0.0, -40.0, 0.0, 40.0);
    MapTrafficRouteSegment junctionSegment = straight(1207, 0.0, 0.0, 0.0, 10.0);
    junctionSegment.points = {{0.0, 0.0, 0.0}, {0.0, 10.0, 0.0}};
    junctionClassificationRoute.segments = {approachSegment, junctionSegment};
    junctionClassificationRoute.cumulativeEndDistances = {40.0, 50.0};
    junctionClassificationRoute.totalLength = 50.0;
    require(mapTrafficRoutePositionOnJunctionPath(junctionClassificationRoute, 45.0) &&
                !mapTrafficRoutePositionOnJunctionPath(junctionClassificationRoute, 10.0),
            "junction debug classification follows the actor's current route segment");
    auto partiallyCommitted = makeMapTrafficActor(
        junctionClassificationRoute, "partially-committed.ovh", 8.0, 0.0, false);
    partiallyCommitted.distanceMeters = 39.0;
    require(!mapTrafficRoutePositionOnJunctionPath(partiallyCommitted.route,
                                                   partiallyCommitted.distanceMeters) &&
                !mapTrafficRoutePositionOnJunctionPath(
                    partiallyCommitted.route,
                    partiallyCommitted.distanceMeters - partiallyCommitted.lengthMeters * 0.5) &&
                mapTrafficActorOnJunctionPath(partiallyCommitted,
                                              partiallyCommitted.distanceMeters),
            "a vehicle commits when its front, not only its center or rear, enters the junction");
    committedJunctionRoute.segments.front().pathRules.groups.push_back(committedPathRules);
    MapTrafficRoute approachingJunctionRoute = oneSegment(straight(1205, -40.0, 0.0, 90.0, 100.0));
    approachingJunctionRoute.segments.front().pathRules.groups.push_back(approachingPathRules);
    auto committedJunctionActor = makeMapTrafficActor(std::move(committedJunctionRoute),
                                                      "inside-junction.ovh", 8.0, 0.0, false);
    auto highPriorityApproach = makeMapTrafficActor(std::move(approachingJunctionRoute),
                                                    "approaching.ovh", 8.0, 0.0, false);
    committedJunctionActor.unscheduledGroupIndex = 0;
    highPriorityApproach.unscheduledGroupIndex = 0;
    committedJunctionActor.distanceMeters = 45.0;
    highPriorityApproach.distanceMeters = 35.0;
    for (int frame = 0; frame < 300; ++frame) {
        advanceMapTrafficDriving({&highPriorityApproach, &committedJunctionActor}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(highPriorityApproach),
                                       {footprint(committedJunctionActor)}, 0.49),
                "an approaching high-priority vehicle does not collide with junction traffic");
    }
    require(committedJunctionActor.distanceMeters > 60.0 &&
                highPriorityApproach.distanceMeters > 60.0,
            "a vehicle already committed to a junction clears before new priority traffic enters");
    const auto throughJunction = [&](int firstId, double startX, double startY, double heading,
                                     double directionX, double directionY) {
        MapTrafficRoute route;
        auto junctionPath = straight(firstId + 1, startX + directionX * 34.0,
                                     startY + directionY * 34.0, heading, 12.0);
        junctionPath.points = {{startX + directionX * 34.0, startY + directionY * 34.0, 0.0},
                               {startX + directionX * 46.0, startY + directionY * 46.0, 0.0}};
        route.segments = {straight(firstId, startX, startY, heading, 34.0), junctionPath,
                          straight(firstId + 2, startX + directionX * 46.0,
                                   startY + directionY * 46.0, heading, 40.0)};
        route.cumulativeEndDistances = {34.0, 46.0, 86.0};
        route.totalLength = 86.0;
        return route;
    };
    auto longJunctionRoute = throughJunction(1600, 0.0, -40.0, 0.0, 0.0, 1.0);
    longJunctionRoute.segments[1].points = {{0.0, -6.0, 0.0}, {0.0, 194.0, 0.0}};
    longJunctionRoute.segments[2] = straight(1602, 0.0, 194.0, 0.0, 40.0);
    longJunctionRoute.cumulativeEndDistances = {34.0, 234.0, 274.0};
    longJunctionRoute.totalLength = 274.0;
    auto distantQueueApproach = makeMapTrafficActor(
        longJunctionRoute, "distant-junction-queue.ovh", 8.0, 0.0, false);
    distantQueueApproach.distanceMeters = 20.0;
    distantQueueApproach.speedMetersPerSecond = 8.0;
    const MapTrafficObstacle distantQueue{{{0.0, 110.0, 0.0}, 0.0, 0}, 12.0, 2.5};
    for (int frame = 0; frame < 120; ++frame) {
        advanceMapTrafficDriving({&distantQueueApproach}, {distantQueue}, 0.1);
        require(mapTrafficHasClearance(footprint(distantQueueApproach), {distantQueue}, 0.49),
                "rolling junction lookahead preserves clearance from a distant queue");
    }
    require(distantQueueApproach.distanceMeters > 90.0 &&
                distantQueueApproach.speedMetersPerSecond > 4.0,
            "a distant queue on a long junction path cannot hold traffic at the path start");
    auto stoppedLongPath = makeMapTrafficActor(
        longJunctionRoute, "stopped-long-path.ovh", 8.0, 0.0, false);
    stoppedLongPath.distanceMeters = 40.0;
    stoppedLongPath.blockReason = MapTrafficBlockReason::Obstacle;
    auto distantLongPathCrossing = makeMapTrafficActor(
        throughJunction(1610, -40.0, 80.0, 90.0, 1.0, 0.0),
        "distant-crossing.ovh", 8.0, 0.0, false);
    distantLongPathCrossing.distanceMeters = 20.0;
    distantLongPathCrossing.speedMetersPerSecond = 8.0;
    const MapTrafficObstacle holdingLongPath{{{0.0, 5.5, 0.0}, 0.0, 0}, 4.5, 1.8};
    for (int frame = 0; frame < 120; ++frame) {
        advanceMapTrafficDriving({&stoppedLongPath, &distantLongPathCrossing}, {holdingLongPath}, 0.1);
    }
    require(distantLongPathCrossing.distanceMeters > 70.0,
            "a stopped vehicle does not reserve a distant crossing on its remaining spline");
    auto longPathPriorityRoute = longJunctionRoute;
    for (auto& segment : longPathPriorityRoute.segments) {
        segment.pathRules.priority = 200.0;
    }
    auto priorityCrossingRoute = throughJunction(1620, -40.0, 80.0, 90.0, 1.0, 0.0);
    for (auto& segment : priorityCrossingRoute.segments) {
        segment.pathRules.priority = 1.0;
    }
    auto lowOnLongPath = makeMapTrafficActor(
        longPathPriorityRoute, "low-on-long-path.ovh", 8.0, 0.0, false);
    auto highAtLocalCrossing = makeMapTrafficActor(
        priorityCrossingRoute, "high-local-crossing.ovh", 8.0, 0.0, false);
    lowOnLongPath.distanceMeters = 100.0;
    highAtLocalCrossing.distanceMeters = 20.0;
    lowOnLongPath.speedMetersPerSecond = 8.0;
    highAtLocalCrossing.speedMetersPerSecond = 8.0;
    bool highCrossedFirst = false;
    for (int frame = 0; frame < 200; ++frame) {
        advanceMapTrafficDriving({&lowOnLongPath, &highAtLocalCrossing}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(lowOnLongPath),
                                       {footprint(highAtLocalCrossing)}, 0.49),
                "local priority arbitration retains physical clearance on a long junction path");
        highCrossedFirst = highCrossedFirst ||
                           (highAtLocalCrossing.distanceMeters > 45.0 &&
                            lowOnLongPath.distanceMeters < 120.0);
    }
    require(highCrossedFirst && lowOnLongPath.distanceMeters > 150.0,
            "being on a long junction spline does not outrank priority at a later crossing");
    auto splitLongPathRoute = longPathPriorityRoute;
    splitLongPathRoute.segments.clear();
    splitLongPathRoute.cumulativeEndDistances.clear();
    splitLongPathRoute.segments.push_back(longPathPriorityRoute.segments.front());
    splitLongPathRoute.cumulativeEndDistances.push_back(34.0);
    for (int piece = 0; piece < 50; ++piece) {
        auto path = longPathPriorityRoute.segments[1];
        path.points = {{0.0, -6.0 + piece * 4.0, 0.0},
                       {0.0, -2.0 + piece * 4.0, 0.0}};
        splitLongPathRoute.segments.push_back(std::move(path));
        splitLongPathRoute.cumulativeEndDistances.push_back(38.0 + piece * 4.0);
    }
    splitLongPathRoute.segments.push_back(longPathPriorityRoute.segments.back());
    splitLongPathRoute.cumulativeEndDistances.push_back(274.0);
    const auto compareLongPathPartition = [&](const MapTrafficRoute& route) {
        auto low = makeMapTrafficActor(route, "partition-low.ovh", 8.0, 0.0, false);
        auto high = makeMapTrafficActor(
            priorityCrossingRoute, "partition-high.ovh", 8.0, 0.0, false);
        low.distanceMeters = 100.0;
        high.distanceMeters = 20.0;
        low.speedMetersPerSecond = 8.0;
        high.speedMetersPerSecond = 8.0;
        std::vector<std::array<double, 4>> trace;
        for (int frame = 0; frame < 140; ++frame) {
            advanceMapTrafficDriving({&low, &high}, {}, 0.1);
            trace.push_back({low.distanceMeters, high.distanceMeters,
                             low.speedMetersPerSecond, high.speedMetersPerSecond});
        }
        return trace;
    };
    const auto continuousPathTrace = compareLongPathPartition(longPathPriorityRoute);
    const auto splitPathTrace = compareLongPathPartition(splitLongPathRoute);
    for (std::size_t frame = 0; frame < continuousPathTrace.size(); ++frame) {
        for (std::size_t value = 0; value < continuousPathTrace[frame].size(); ++value) {
            requireNear(splitPathTrace[frame][value], continuousPathTrace[frame][value],
                        "junction priority and speed are invariant under harmless path splits");
        }
    }
    auto redPriorityRoute = priorityCrossingRoute;
    redPriorityRoute.segments[1].trafficSignal =
        MapTrafficSignalSchedule{120.0, {{0, 20.0}, {6, 100.0}}};
    auto redPriorityVehicle = makeMapTrafficActor(
        redPriorityRoute, "red-priority.ovh", 8.0, 0.0, false);
    auto permissiveLongPath = makeMapTrafficActor(
        longPathPriorityRoute, "permissive-long-path.ovh", 8.0, 0.0, false);
    redPriorityVehicle.distanceMeters = 20.0;
    permissiveLongPath.distanceMeters = 100.0;
    redPriorityVehicle.speedMetersPerSecond = 8.0;
    permissiveLongPath.speedMetersPerSecond = 8.0;
    for (int frame = 0; frame < 150; ++frame) {
        advanceMapTrafficDriving({&permissiveLongPath, &redPriorityVehicle}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(permissiveLongPath),
                                       {footprint(redPriorityVehicle)}, 0.49),
                "red-signal priority traffic remains separated from the permitted movement");
    }
    require(permissiveLongPath.distanceMeters > 200.0 &&
                redPriorityVehicle.distanceMeters <
                    34.0 - redPriorityVehicle.lengthMeters * 0.5 &&
                redPriorityVehicle.blockReason == MapTrafficBlockReason::Signal,
            "a vehicle held at red cannot reserve a crossing beyond its stop line");
    for (int frame = 0; frame < 120; ++frame) {
        advanceMapTrafficDriving({&permissiveLongPath, &redPriorityVehicle}, {}, 0.1,
                                 20.0 + frame * 0.1);
        require(mapTrafficHasClearance(footprint(permissiveLongPath),
                                       {footprint(redPriorityVehicle)}, 0.49),
                "priority traffic resumes safely when its signal turns green");
    }
    require(redPriorityVehicle.distanceMeters > 70.0,
            "signal-clipped local reservations are rebuilt when green permits entry");
    MapTrafficRoute longMergeRoute;
    auto longMergePath = straight(1631, 0.0, -6.0, 45.0, 1.0);
    longMergePath.points = {{0.0, -6.0, 0.0}, {6.0, 0.0, 0.0}, {206.0, 0.0, 0.0}};
    longMergeRoute.segments = {straight(1630, 0.0, -40.0, 0.0, 34.0), longMergePath,
                               straight(1632, 206.0, 0.0, 90.0, 40.0)};
    const double longMergeLength = std::sqrt(72.0) + 200.0;
    longMergeRoute.cumulativeEndDistances =
        {34.0, 34.0 + longMergeLength, 74.0 + longMergeLength};
    longMergeRoute.totalLength = longMergeRoute.cumulativeEndDistances.back();
    auto localMerger =
        makeMapTrafficActor(longMergeRoute, "local-merger.ovh", 8.0, 0.0, false);
    auto remoteMergedLaneVehicle = makeMapTrafficActor(
        oneSegment(straight(1633, 0.0, 0.0, 90.0, 250.0)),
        "remote-merged-lane.ovh", 0.0, 0.0, false);
    localMerger.distanceMeters = 20.0;
    localMerger.speedMetersPerSecond = 8.0;
    remoteMergedLaneVehicle.distanceMeters = 120.0;
    for (int frame = 0; frame < 180; ++frame) {
        advanceMapTrafficDriving({&localMerger, &remoteMergedLaneVehicle}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(localMerger),
                                       {footprint(remoteMergedLaneVehicle)}, 0.49),
                "a local merge into a long occupied spline preserves vehicle clearance");
    }
    require(localMerger.distanceMeters > 90.0,
            "a remote vehicle on a long downstream spline cannot hold a clear local merge");
    const auto singleLaneCorridor = [&](int id, bool reverse) {
        MapTrafficRoute route;
        const double direction = reverse ? -1.0 : 1.0;
        const double startY = reverse ? 240.0 : -40.0;
        const double heading = reverse ? 180.0 : 0.0;
        auto shared = straight(id + 1, 0.0, startY + direction * 40.0, heading, 200.0);
        shared.points = {{0.0, startY + direction * 40.0, 0.0},
                         {0.0, startY + direction * 220.0, 0.0},
                         {direction * 20.0, startY + direction * 240.0, 0.0}};
        const double sharedLength = 180.0 + std::sqrt(800.0);
        route.segments = {straight(id, 0.0, startY, heading, 40.0), shared,
                          straight(id + 2, direction * 20.0, startY + direction * 240.0,
                                   reverse ? 270.0 : 90.0, 40.0)};
        route.cumulativeEndDistances = {40.0, 40.0 + sharedLength, 80.0 + sharedLength};
        route.totalLength = route.cumulativeEndDistances.back();
        return route;
    };
    auto northCorridor =
        makeMapTrafficActor(singleLaneCorridor(1640, false), "corridor-north.ovh", 8.0, 0.0, false);
    auto southCorridor =
        makeMapTrafficActor(singleLaneCorridor(1650, true), "corridor-south.ovh", 8.0, 0.0, false);
    northCorridor.distanceMeters = 20.0;
    southCorridor.distanceMeters = 20.0;
    northCorridor.speedMetersPerSecond = 8.0;
    southCorridor.speedMetersPerSecond = 8.0;
    for (const bool northStartsAtRed : {false, true}) {
        northCorridor.distanceMeters = 20.0;
        southCorridor.distanceMeters = 20.0;
        northCorridor.speedMetersPerSecond = 8.0;
        southCorridor.speedMetersPerSecond = 8.0;
        northCorridor.junctionClaimEndMeters.reset();
        southCorridor.junctionClaimEndMeters.reset();
        northCorridor.blockReason = MapTrafficBlockReason::None;
        southCorridor.blockReason = MapTrafficBlockReason::None;
        if (northStartsAtRed) {
            northCorridor.route.segments[1].trafficSignal =
                MapTrafficSignalSchedule{120.0, {{0, 20.0}, {6, 100.0}}};
        }
        for (int frame = 0; frame < 800; ++frame) {
            advanceMapTrafficDriving({&northCorridor, &southCorridor}, {}, 0.1, frame * 0.1);
            require(mapTrafficHasClearance(footprint(northCorridor), {footprint(southCorridor)},
                                           0.49),
                    "opposing single-lane corridor traffic retains physical clearance");
            if (northStartsAtRed && frame == 120) {
                require(northCorridor.distanceMeters < 40.0 &&
                            southCorridor.distanceMeters > 100.0,
                        "a red-held vehicle cannot claim an unentered single-lane corridor");
            }
        }
        require(northCorridor.distanceMeters > 260.0 && southCorridor.distanceMeters > 260.0,
                "exclusive opposing-corridor admission prevents a head-on deadlock on long paths");
    }
    auto claimNorth = makeMapTrafficActor(throughJunction(1300, 0.0, -40.0, 0.0, 0.0, 1.0),
                                          "claim-north.ovh", 8.0, 0.0, false);
    auto claimEast = makeMapTrafficActor(throughJunction(1310, -40.0, 0.0, 90.0, 1.0, 0.0),
                                         "claim-east.ovh", 8.0, 0.0, false);
    claimNorth.distanceMeters = 20.0;
    claimEast.distanceMeters = 20.0;
    claimNorth.speedMetersPerSecond = 8.0;
    claimEast.speedMetersPerSecond = 8.0;
    bool junctionClaimWaitObserved = false;
    double slowestCommittedSpeed = std::numeric_limits<double>::infinity();
    for (int frame = 0; frame < 400; ++frame) {
        advanceMapTrafficDriving({&claimNorth, &claimEast}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(claimNorth), {footprint(claimEast)}, 0.49),
                "junction claims keep simultaneously arriving crossing vehicles separated");
        for (const MapTrafficActor* actor : {&claimNorth, &claimEast}) {
            junctionClaimWaitObserved =
                junctionClaimWaitObserved ||
                actor->blockReason == MapTrafficBlockReason::JunctionTraffic;
            if (actor->distanceMeters < 70.0 &&
                mapTrafficActorOnJunctionPath(*actor, actor->distanceMeters)) {
                slowestCommittedSpeed =
                    std::min(slowestCommittedSpeed, actor->speedMetersPerSecond);
            }
        }
    }
    require(junctionClaimWaitObserved,
            "a vehicle waits outside a junction claimed by a conflicting vehicle");
    require(slowestCommittedSpeed > 1.0,
            "vehicles admitted into a claimed junction never stop inside it");
    require(claimNorth.distanceMeters > 70.0 && claimEast.distanceMeters > 70.0,
            "both conflicting junction movements traverse after claims are released");
    for (const bool closedLoop : {false, true}) {
        auto loopingNorthRoute = throughJunction(1660, 0.0, -40.0, 0.0, 0.0, 1.0);
        auto loopingEastRoute = throughJunction(1670, -40.0, 0.0, 90.0, 1.0, 0.0);
        loopingNorthRoute.segments.insert(
            loopingNorthRoute.segments.end(),
            {straight(1663, 0.0, 46.0, 90.0, 40.0),
             straight(1664, 40.0, 46.0, 180.0, 86.0),
             straight(1665, 40.0, -40.0, 270.0, 40.0)});
        loopingEastRoute.segments.insert(
            loopingEastRoute.segments.end(),
            {straight(1673, 46.0, 0.0, 0.0, 40.0),
             straight(1674, 46.0, 40.0, 270.0, 86.0),
             straight(1675, -40.0, 40.0, 180.0, 40.0)});
        for (auto* route : {&loopingNorthRoute, &loopingEastRoute}) {
            route->cumulativeEndDistances.insert(route->cumulativeEndDistances.end(),
                                                 {126.0, 212.0, 252.0});
            route->totalLength = 252.0;
            route->closed = closedLoop;
            if (!closedLoop) {
                route->loopStartDistance = 0.0;
            }
        }
        auto loopingNorth =
            makeMapTrafficActor(loopingNorthRoute, "looping-north.ovh", 8.0, 0.0, false);
        auto loopingEast =
            makeMapTrafficActor(loopingEastRoute, "looping-east.ovh", 8.0, 0.0, false);
        loopingNorth.distanceMeters = 20.0;
        loopingEast.distanceMeters = 20.0;
        loopingNorth.speedMetersPerSecond = 8.0;
        loopingEast.speedMetersPerSecond = 8.0;
        double slowestLoopJunctionSpeed = std::numeric_limits<double>::infinity();
        for (int frame = 0; frame < 140; ++frame) {
            advanceMapTrafficDriving({&loopingNorth, &loopingEast}, {}, 0.1);
            require(mapTrafficHasClearance(footprint(loopingNorth), {footprint(loopingEast)},
                                           0.49),
                    "looping-route junction admission preserves crossing clearance");
            for (const auto* actor : {&loopingNorth, &loopingEast}) {
                if (mapTrafficActorOnJunctionPath(*actor, actor->distanceMeters)) {
                    slowestLoopJunctionSpeed =
                        std::min(slowestLoopJunctionSpeed, actor->speedMetersPerSecond);
                }
            }
        }
        require(loopingNorth.distanceMeters > 60.0 && loopingEast.distanceMeters > 60.0 &&
                    slowestLoopJunctionSpeed > 1.0,
                "closed and entry-loop routes use junction admission rather than stopping inside");
    }
    auto committedPerpendicularNorth = makeMapTrafficActor(
        throughJunction(1350, 0.0, -40.0, 0.0, 0.0, 1.0),
        "committed-perpendicular-north.ovh", 8.0, 0.0, false);
    auto committedPerpendicularEast = makeMapTrafficActor(
        throughJunction(1360, -40.0, 0.0, 90.0, 1.0, 0.0),
        "committed-perpendicular-east.ovh", 8.0, 0.0, false);
    committedPerpendicularNorth.distanceMeters = 35.0;
    committedPerpendicularEast.distanceMeters = 35.0;
    committedPerpendicularNorth.speedMetersPerSecond = 8.0;
    committedPerpendicularEast.speedMetersPerSecond = 8.0;
    for (int frame = 0; frame < 250; ++frame) {
        advanceMapTrafficDriving(
            {&committedPerpendicularNorth, &committedPerpendicularEast}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(committedPerpendicularNorth),
                                       {footprint(committedPerpendicularEast)}, 0.49),
                "committed perpendicular vehicles retain physical clearance");
    }
    require(committedPerpendicularNorth.distanceMeters > 60.0 &&
                committedPerpendicularEast.distanceMeters > 60.0,
            "one committed perpendicular vehicle clears before the other proceeds");
    const auto junctionRouteFromPose = [&](int id, double x, double y, double headingDegrees) {
        constexpr double routeLength = 30.0;
        const double headingRadians = headingDegrees * std::acos(-1.0) / 180.0;
        auto path = straight(id, x, y, headingDegrees, routeLength);
        path.points = {{x, y, 0.0},
                       {x + std::sin(headingRadians) * routeLength,
                        y + std::cos(headingRadians) * routeLength, 0.0}};
        return oneSegment(path);
    };
    auto collisionBlockedEast = makeMapTrafficActor(
        junctionRouteFromPose(1362, 0.0, 0.0, 90.0), "collision-blocked-east.ovh", 8.0, 0.0,
        false);
    auto collisionBlockedNorth = makeMapTrafficActor(
        junctionRouteFromPose(1363, 3.2, -3.2, 0.0), "collision-blocked-north.ovh", 8.0, 0.0,
        false);
    collisionBlockedEast.junctionWaitSeconds = 18.0;
    collisionBlockedNorth.junctionWaitSeconds = 18.0;
    collisionBlockedEast.blockReason = MapTrafficBlockReason::Collision;
    collisionBlockedNorth.blockReason = MapTrafficBlockReason::Collision;
    collisionBlockedEast.blockingActor = &collisionBlockedNorth;
    collisionBlockedNorth.blockingActor = &collisionBlockedEast;
    require(mapTrafficHasClearance(footprint(collisionBlockedEast),
                                   {footprint(collisionBlockedNorth)}, 0.0),
            "deadlock recovery starts from non-overlapping vehicle bodies");
    for (int frame = 0; frame < 200; ++frame) {
        advanceMapTrafficDriving({&collisionBlockedEast, &collisionBlockedNorth}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(collisionBlockedEast),
                                       {footprint(collisionBlockedNorth)}, 0.0),
                "junction collision recovery does not introduce body overlap");
    }
    require(collisionBlockedEast.distanceMeters > 5.0,
            "the collision-blocked vehicle with a clear perpendicular path continues");
    const auto makeShallowMergeRoute = [&](int approachId, double headingDegrees) {
        constexpr double approachLength = 18.0;
        const double headingRadians = headingDegrees * std::acos(-1.0) / 180.0;
        MapTrafficRoute route;
        route.segments = {
            straight(approachId, -std::sin(headingRadians) * approachLength,
                     -std::cos(headingRadians) * approachLength, headingDegrees, approachLength),
            straight(1344, 0.0, 0.0, 90.0, 70.0)};
        route.cumulativeEndDistances = {approachLength, approachLength + 70.0};
        route.totalLength = route.cumulativeEndDistances.back();
        return route;
    };
    auto shallowMergeFirst = makeMapTrafficActor(
        makeShallowMergeRoute(1342, 72.0), "shallow-merge-first.ovh", 8.0, 0.0, false);
    auto shallowMergeSecond = makeMapTrafficActor(
        makeShallowMergeRoute(1343, 90.0), "shallow-merge-second.ovh", 8.0, 0.0, false);
    shallowMergeFirst.speedMetersPerSecond = 8.0;
    shallowMergeSecond.speedMetersPerSecond = 8.0;
    for (int frame = 0; frame < 300; ++frame) {
        advanceMapTrafficDriving({&shallowMergeFirst, &shallowMergeSecond}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(shallowMergeFirst),
                                       {footprint(shallowMergeSecond)}, 0.49),
                "shallow-angle same-direction merges retain clearance without join reservations");
    }
    require(shallowMergeFirst.distanceMeters > 60.0 &&
                shallowMergeSecond.distanceMeters > 60.0,
            "vehicles continue through shared road flow without a spline-boundary deadlock");
    MapTrafficRoute seamRoute;
    MapTrafficRoute seamLeaderRoute;
    for (int piece = 0; piece < 30; ++piece) {
        seamRoute.segments.push_back(straight(1700 + piece, 500.0, piece * 10.0, 0.0, 10.0));
        seamRoute.cumulativeEndDistances.push_back((piece + 1) * 10.0);
    }
    seamRoute.totalLength = 300.0;
    seamLeaderRoute = seamRoute;
    auto seamFollower = makeMapTrafficActor(seamRoute, "seam-follower.ovh", 8.0, 0.0, false);
    auto seamLeader = makeMapTrafficActor(seamLeaderRoute, "seam-leader.ovh", 8.0, 0.0, false);
    seamFollower.distanceMeters = 5.0;
    seamFollower.speedMetersPerSecond = 8.0;
    seamLeader.distanceMeters = 45.0;
    seamLeader.speedMetersPerSecond = 8.0;
    for (int frame = 0; frame < 250; ++frame) {
        advanceMapTrafficDriving({&seamFollower, &seamLeader}, {}, 0.1);
        require(seamFollower.speedMetersPerSecond > 7.9 && seamLeader.speedMetersPerSecond > 7.9,
                "same-direction traffic never slows at ordinary spline starts or ends");
    }
    auto slowTurnPath = straight(1321, 0.0, -40.0, 0.0, 80.0);
    slowTurnPath.points = {{0.0, -40.0, 0.0}, {0.0, 40.0, 0.0}};
    auto slowTurnaround =
        makeMapTrafficActor(oneSegment(slowTurnPath), "slow-turnaround.ovh", 2.0, 0.0, false);
    slowTurnaround.speedMetersPerSecond = 2.0;
    auto turnaroundCrossingApproach = straight(1322, -40.0, 0.0, 90.0, 30.0);
    auto turnaroundCrossingPath = straight(1323, -10.0, 0.0, 90.0, 20.0);
    turnaroundCrossingPath.points = {{-10.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
    MapTrafficRoute turnaroundCrossingRoute;
    turnaroundCrossingRoute.segments = {
        turnaroundCrossingApproach, turnaroundCrossingPath,
        straight(1324, 10.0, 0.0, 90.0, 40.0)};
    turnaroundCrossingRoute.cumulativeEndDistances = {30.0, 50.0, 90.0};
    turnaroundCrossingRoute.totalLength = 90.0;
    auto turnaroundCrossing = makeMapTrafficActor(
        std::move(turnaroundCrossingRoute), "turnaround-crossing.ovh", 8.0, 0.0, false);
    turnaroundCrossing.distanceMeters = 20.0;
    turnaroundCrossing.speedMetersPerSecond = 8.0;
    bool turnaroundCrossingWaitedAtEntry = false;
    for (int frame = 0; frame < 450; ++frame) {
        advanceMapTrafficDriving({&turnaroundCrossing, &slowTurnaround}, {}, 0.1);
        turnaroundCrossingWaitedAtEntry =
            turnaroundCrossingWaitedAtEntry ||
            (turnaroundCrossing.distanceMeters < 30.0 &&
             turnaroundCrossing.blockReason == MapTrafficBlockReason::JunctionTraffic);
        require(mapTrafficHasClearance(footprint(turnaroundCrossing), {footprint(slowTurnaround)},
                                       0.49),
                "a crossing vehicle clears before a slow junction movement reaches the crossing");
    }
    require(!turnaroundCrossingWaitedAtEntry && turnaroundCrossing.distanceMeters > 70.0 &&
                slowTurnaround.distanceMeters > 70.0,
            "a moving vehicle's distant future junction path does not block entry for its full route");
    auto stoppedTurnaround =
        makeMapTrafficActor(oneSegment(slowTurnPath), "stopped-turnaround.ovh", 2.0, 0.0, false);
    stoppedTurnaround.distanceMeters = 40.0;
    stoppedTurnaround.speedMetersPerSecond = 0.0;
    stoppedTurnaround.blockReason = MapTrafficBlockReason::Obstacle;
    auto stoppedCrossingRoute = turnaroundCrossing.route;
    auto stoppedCrossing =
        makeMapTrafficActor(std::move(stoppedCrossingRoute), "stopped-crossing.ovh", 8.0, 0.0,
                            false);
    stoppedCrossing.distanceMeters = 20.0;
    stoppedCrossing.speedMetersPerSecond = 8.0;
    advanceMapTrafficDriving({&stoppedCrossing, &stoppedTurnaround}, {}, 0.1);
    require(stoppedCrossing.distanceMeters < 30.0 - stoppedCrossing.lengthMeters * 0.5 &&
                stoppedCrossing.blockReason == MapTrafficBlockReason::JunctionTraffic,
            "a stopped vehicle occupying the crossing keeps conflicting approaches out");
    auto groundCrossingPath = straight(1330, 0.0, -40.0, 0.0, 100.0);
    groundCrossingPath.points = {{0.0, -40.0, 0.0}, {0.0, 60.0, 0.0}};
    auto elevatedCrossingPath = straight(1331, -40.0, 0.0, 90.0, 100.0);
    elevatedCrossingPath.points = {{-40.0, 0.0, 4.0}, {60.0, 0.0, 4.0}};
    auto groundCrossing =
        makeMapTrafficActor(oneSegment(groundCrossingPath), "ground-crossing.ovh", 8.0, 0.0,
                            false);
    auto elevatedCrossing =
        makeMapTrafficActor(oneSegment(elevatedCrossingPath), "elevated-crossing.ovh", 8.0, 0.0,
                            false);
    groundCrossing.distanceMeters = 35.0;
    elevatedCrossing.distanceMeters = 35.0;
    groundCrossing.speedMetersPerSecond = 8.0;
    elevatedCrossing.speedMetersPerSecond = 8.0;
    for (int frame = 0; frame < 100; ++frame) {
        advanceMapTrafficDriving({&groundCrossing, &elevatedCrossing}, {}, 0.1);
    }
    require(groundCrossing.distanceMeters > 60.0 && elevatedCrossing.distanceMeters > 60.0 &&
                groundCrossing.junctionWaitSeconds == 0.0 &&
                elevatedCrossing.junctionWaitSeconds == 0.0,
            "vertically separated spline overlaps do not block junction movement");
    auto claimLeader = makeMapTrafficActor(throughJunction(1320, 0.0, -40.0, 0.0, 0.0, 1.0),
                                           "claim-leader.ovh", 8.0, 0.0, false);
    auto claimFollower = makeMapTrafficActor(throughJunction(1320, 0.0, -40.0, 0.0, 0.0, 1.0),
                                             "claim-follower.ovh", 8.0, 0.0, false);
    claimLeader.distanceMeters = 30.0;
    claimFollower.distanceMeters = 16.0;
    claimLeader.speedMetersPerSecond = 8.0;
    claimFollower.speedMetersPerSecond = 8.0;
    bool followerOverlappedLeaderClaim = false;
    double slowestFollowerSpeed = std::numeric_limits<double>::infinity();
    for (int frame = 0; frame < 200; ++frame) {
        advanceMapTrafficDriving({&claimLeader, &claimFollower}, {}, 0.1);
        if (claimFollower.distanceMeters < 70.0) {
            slowestFollowerSpeed =
                std::min(slowestFollowerSpeed, claimFollower.speedMetersPerSecond);
        }
        followerOverlappedLeaderClaim =
            followerOverlappedLeaderClaim ||
            (claimLeader.junctionClaimEndMeters.has_value() &&
             claimFollower.junctionClaimEndMeters.has_value());
    }
    require(slowestFollowerSpeed > 4.0,
            "a same-lane follower is not stopped at the entry by its leader's junction claim");
    require(followerOverlappedLeaderClaim && claimFollower.distanceMeters > 70.0,
            "a same-lane follower enters the junction while its leader still holds a claim");
    MapTrafficRoute differentExitLeaderRoute;
    auto differentExitApproach = straight(1325, 0.0, -40.0, 0.0, 34.0);
    auto differentExitTurn = straight(1326, 0.0, -6.0, 45.0, std::sqrt(72.0));
    differentExitTurn.points = {{0.0, -6.0, 0.0}, {6.0, 0.0, 0.0}};
    differentExitLeaderRoute.segments = {differentExitApproach, differentExitTurn,
                                         straight(1327, 6.0, 0.0, 90.0, 40.0)};
    differentExitLeaderRoute.cumulativeEndDistances = {34.0, 34.0 + std::sqrt(72.0),
                                                        74.0 + std::sqrt(72.0)};
    differentExitLeaderRoute.totalLength = differentExitLeaderRoute.cumulativeEndDistances.back();
    MapTrafficRoute differentExitFollowerRoute;
    auto differentExitFollowerApproach = differentExitApproach;
    auto differentExitThrough = straight(1328, 0.0, -6.0, 0.0, 12.0);
    differentExitThrough.points = {{0.0, -6.0, 0.0}, {0.0, 6.0, 0.0}};
    differentExitFollowerRoute.segments = {differentExitFollowerApproach, differentExitThrough,
                                           straight(1329, 0.0, 6.0, 0.0, 40.0)};
    differentExitFollowerRoute.cumulativeEndDistances = {34.0, 46.0, 86.0};
    differentExitFollowerRoute.totalLength = 86.0;
    auto differentExitLeader = makeMapTrafficActor(
        differentExitLeaderRoute, "different-exit-leader.ovh", 8.0, 0.0, false);
    auto differentExitFollower = makeMapTrafficActor(
        differentExitFollowerRoute, "different-exit-follower.ovh", 8.0, 0.0, false);
    differentExitLeader.distanceMeters = 32.5;
    differentExitFollower.distanceMeters = 20.0;
    differentExitLeader.speedMetersPerSecond = 8.0;
    differentExitFollower.speedMetersPerSecond = 8.0;
    bool differentExitFollowerEnteredDuringClaim = false;
    double slowestDifferentExitFollower = std::numeric_limits<double>::infinity();
    for (int frame = 0; frame < 160; ++frame) {
        advanceMapTrafficDriving({&differentExitLeader, &differentExitFollower}, {}, 0.1);
        slowestDifferentExitFollower =
            std::min(slowestDifferentExitFollower, differentExitFollower.speedMetersPerSecond);
        differentExitFollowerEnteredDuringClaim =
            differentExitFollowerEnteredDuringClaim ||
            (differentExitLeader.junctionClaimEndMeters.has_value() &&
             differentExitFollower.junctionClaimEndMeters.has_value());
        require(mapTrafficHasClearance(footprint(differentExitLeader),
                                       {footprint(differentExitFollower)}, 0.49),
                "different-exit queue movements preserve vehicle clearance");
    }
    require(differentExitFollowerEnteredDuringClaim &&
                slowestDifferentExitFollower > 2.0 &&
                differentExitFollower.distanceMeters > 70.0,
            "a follower with a different exit does not wait for its leader to clear the junction");
    MapTrafficRoute segmentedLeaderRoute;
    segmentedLeaderRoute.segments.push_back(straight(1350, 0.0, -40.0, 0.0, 30.0));
    segmentedLeaderRoute.cumulativeEndDistances.push_back(30.0);
    for (int segmentIndex = 0; segmentIndex < 6; ++segmentIndex) {
        const double segmentStartY = -10.0 + segmentIndex * 1.5;
        auto segment = straight(1351 + segmentIndex, 0.0, segmentStartY, 0.0, 1.5);
        segment.points = {{0.0, segmentStartY, 0.0},
                          {0.0, segmentStartY + 1.5, 0.0}};
        segmentedLeaderRoute.segments.push_back(std::move(segment));
        segmentedLeaderRoute.cumulativeEndDistances.push_back(30.0 + (segmentIndex + 1) * 1.5);
    }
    segmentedLeaderRoute.segments.push_back(straight(1357, 0.0, -1.0, 0.0, 40.0));
    segmentedLeaderRoute.cumulativeEndDistances.push_back(79.0);
    segmentedLeaderRoute.totalLength = 79.0;
    MapTrafficRoute segmentedFollowerRoute;
    segmentedFollowerRoute.segments.push_back(straight(1350, 0.0, -40.0, 0.0, 30.0));
    auto segmentedFollowerTurn = straight(1358, 0.0, -10.0, 45.0, std::sqrt(200.0));
    segmentedFollowerTurn.points = {{0.0, -10.0, 0.0}, {10.0, 0.0, 0.0}};
    segmentedFollowerRoute.segments = {
        segmentedFollowerRoute.segments.front(), segmentedFollowerTurn,
        straight(1359, 10.0, 0.0, 90.0, 40.0)};
    segmentedFollowerRoute.cumulativeEndDistances = {30.0, 30.0 + std::sqrt(200.0),
                                                      70.0 + std::sqrt(200.0)};
    segmentedFollowerRoute.totalLength = segmentedFollowerRoute.cumulativeEndDistances.back();
    auto segmentedLeader = makeMapTrafficActor(
        segmentedLeaderRoute, "segmented-leader.ovh", 8.0, 0.0, false);
    auto segmentedFollower = makeMapTrafficActor(
        segmentedFollowerRoute, "segmented-follower.ovh", 8.0, 0.0, false);
    segmentedLeader.distanceMeters = 35.0;
    segmentedFollower.distanceMeters = 25.0;
    segmentedLeader.speedMetersPerSecond = 8.0;
    segmentedFollower.speedMetersPerSecond = 8.0;
    double slowestSegmentedFollowerSpeed = std::numeric_limits<double>::infinity();
    bool segmentedLeaderKeptStaleClaim = false;
    int segmentedFollowerJunctionWaitFrames = 0;
    for (int frame = 0; frame < 120; ++frame) {
        advanceMapTrafficDriving({&segmentedLeader, &segmentedFollower}, {}, 0.1);
        if (segmentedFollower.blockReason == MapTrafficBlockReason::JunctionTraffic) {
            ++segmentedFollowerJunctionWaitFrames;
        }
        if (segmentedFollower.distanceMeters < 60.0) {
            slowestSegmentedFollowerSpeed =
                std::min(slowestSegmentedFollowerSpeed, segmentedFollower.speedMetersPerSecond);
        }
        segmentedLeaderKeptStaleClaim =
            segmentedLeaderKeptStaleClaim ||
            (segmentedLeader.distanceMeters > 43.0 &&
             segmentedLeader.junctionClaimEndMeters.has_value());
        require(mapTrafficHasClearance(footprint(segmentedLeader),
                                       {footprint(segmentedFollower)}, 0.49),
                "a follower with many junction connectors keeps clearance from its leader");
    }
    require(!segmentedLeaderKeptStaleClaim && slowestSegmentedFollowerSpeed > 2.0 &&
                segmentedFollower.distanceMeters > 60.0 &&
                segmentedFollowerJunctionWaitFrames < 5,
            "a follower still shares its approach after its leader crosses several junction "
            "connectors");
    MapTrafficRoute sharedLaneLeaderRoute;
    sharedLaneLeaderRoute.segments = {straight(1340, 0.0, 0.0, 0.0, 100.0),
                                      straight(1341, 0.0, 100.0, 0.0, 50.0)};
    sharedLaneLeaderRoute.cumulativeEndDistances = {100.0, 150.0};
    sharedLaneLeaderRoute.totalLength = 150.0;
    auto movingSharedLaneLeader = makeMapTrafficActor(std::move(sharedLaneLeaderRoute),
                                                      "shared-lane-leader.ovh", 8.0, 0.0, false);
    auto sharedLaneFollower = makeMapTrafficActor(oneSegment(straight(1340, 0.0, 0.0, 0.0, 100.0)),
                                                  "shared-lane-follower.ovh", 8.0, 0.0, false);
    movingSharedLaneLeader.distanceMeters = 25.0;
    sharedLaneFollower.distanceMeters = 10.0;
    movingSharedLaneLeader.speedMetersPerSecond = 8.0;
    sharedLaneFollower.speedMetersPerSecond = 8.0;
    double slowestSharedLaneFollower = std::numeric_limits<double>::infinity();
    for (int frame = 0; frame < 80; ++frame) {
        advanceMapTrafficDriving({&movingSharedLaneLeader, &sharedLaneFollower}, {}, 0.1);
        slowestSharedLaneFollower =
            std::min(slowestSharedLaneFollower, sharedLaneFollower.speedMetersPerSecond);
        require(mapTrafficHasClearance(footprint(sharedLaneFollower),
                                       {footprint(movingSharedLaneLeader)}, 0.49),
                "a follower on a different route keeps clear of its shared-lane leader");
    }
    require(slowestSharedLaneFollower > 6.0,
            "a moving shared-lane leader is followed instead of being braked for as parked");
    const auto approachThen = [&](int approachId, MapTrafficRouteSegment junctionPath,
                                  MapTrafficRouteSegment exit) {
        MapTrafficRoute route;
        const double junctionLength = junctionPath.spline.placement.length;
        route.segments = {straight(approachId, 0.0, -40.0, 0.0, 34.0), std::move(junctionPath),
                          std::move(exit)};
        route.cumulativeEndDistances = {34.0, 34.0 + junctionLength, 74.0 + junctionLength};
        route.totalLength = 74.0 + junctionLength;
        return route;
    };
    auto straightOn = straight(1351, 0.0, -6.0, 0.0, 12.0);
    straightOn.points = {{0.0, -6.0, 0.0}, {0.0, 6.0, 0.0}};
    auto turnOff = straight(1352, 0.0, -6.0, 45.0, std::sqrt(72.0));
    turnOff.points = {{0.0, -6.0, 0.0}, {6.0, 0.0, 0.0}};
    auto slowQueueLeader =
        makeMapTrafficActor(approachThen(1350, straightOn, straight(1353, 0.0, 6.0, 0.0, 40.0)),
                            "queue-leader.ovh", 8.0, 0.0, false);
    auto fastQueueFollower =
        makeMapTrafficActor(approachThen(1350, turnOff, straight(1354, 6.0, 0.0, 90.0, 40.0)),
                            "queue-follower.ovh", 8.0, 0.0, false);
    // The follower claimed the junction before the leader merged in ahead of it.
    slowQueueLeader.distanceMeters = 31.0;
    fastQueueFollower.distanceMeters = 23.0;
    fastQueueFollower.junctionClaimEndMeters = 34.0 + std::sqrt(72.0) + 4.25;
    for (int frame = 0; frame < 600; ++frame) {
        advanceMapTrafficDriving({&slowQueueLeader, &fastQueueFollower}, {}, 0.1);
        require(slowQueueLeader.blockingActor != &fastQueueFollower,
                "a vehicle is never held at a junction by the vehicle queued behind it");
    }
    require(slowQueueLeader.distanceMeters > 50.0 && fastQueueFollower.distanceMeters > 50.0,
            "a queue whose vehicles take different junction movements clears in order");
    MapTrafficGroupRules mainRoadRules;
    mainRoadRules.groupIndex = 0;
    mainRoadRules.priority = 64.0;
    MapTrafficGroupRules sideRoadRules;
    sideRoadRules.groupIndex = 0;
    sideRoadRules.priority = 192.0;
    for (const bool mainRoadUsesJunctionPath : {true, false}) {
        // Main road runs east through x=-6..6 at y=0; the side road joins from the south.
        MapTrafficRoute mainRoute;
        auto mainApproach = straight(1360, -80.0, 0.0, 90.0, 74.0);
        mainApproach.pathRules.groups.push_back(mainRoadRules);
        auto mainThrough = straight(1361, -6.0, 0.0, 90.0, 12.0);
        if (mainRoadUsesJunctionPath) {
            mainThrough.points = {{-6.0, 0.0, 0.0}, {6.0, 0.0, 0.0}};
        }
        mainThrough.pathRules.groups.push_back(sideRoadRules);
        mainRoute.segments = {mainApproach, mainThrough, straight(1362, 6.0, 0.0, 90.0, 60.0)};
        mainRoute.cumulativeEndDistances = {74.0, 86.0, 146.0};
        mainRoute.totalLength = 146.0;
        MapTrafficRoute sideRoute;
        auto sideApproach = straight(1363, 0.0, -40.0, 0.0, 34.0);
        sideApproach.pathRules.groups.push_back(sideRoadRules);
        auto sideTurn = straight(1364, 0.0, -6.0, 45.0, std::sqrt(72.0));
        sideTurn.points = {{0.0, -6.0, 0.0}, {6.0, 0.0, 0.0}};
        sideRoute.segments = {sideApproach, sideTurn, straight(1365, 6.0, 0.0, 90.0, 60.0)};
        sideRoute.cumulativeEndDistances = {34.0, 34.0 + std::sqrt(72.0),
                                            94.0 + std::sqrt(72.0)};
        sideRoute.totalLength = 94.0 + std::sqrt(72.0);
        auto mainRoadCar = makeMapTrafficActor(std::move(mainRoute), "main-road.ovh", 10.0, 0.0,
                                               false);
        auto sideRoadCar = makeMapTrafficActor(std::move(sideRoute), "side-road.ovh", 8.0, 0.0,
                                               false);
        mainRoadCar.unscheduledGroupIndex = 0;
        sideRoadCar.unscheduledGroupIndex = 0;
        // The side-road car reaches its entry line first, with the main-road car a few
        // seconds away, and has already been waiting long enough to have aged.
        mainRoadCar.distanceMeters = 40.0;
        mainRoadCar.speedMetersPerSecond = 10.0;
        sideRoadCar.distanceMeters = 26.0;
        sideRoadCar.speedMetersPerSecond = 0.0;
        sideRoadCar.junctionWaitSeconds = 30.0;
        double slowestMainRoad = std::numeric_limits<double>::infinity();
        bool sideRoadYieldedToMainRoad = false;
        for (int frame = 0; frame < 400; ++frame) {
            advanceMapTrafficDriving({&sideRoadCar, &mainRoadCar}, {}, 0.1);
            if (mainRoadCar.distanceMeters < 90.0) {
                slowestMainRoad = std::min(slowestMainRoad, mainRoadCar.speedMetersPerSecond);
            }
            sideRoadYieldedToMainRoad = sideRoadYieldedToMainRoad ||
                                        (sideRoadCar.blockingActor == &mainRoadCar &&
                                         sideRoadCar.distanceMeters < 34.0);
            require(mapTrafficHasClearance(footprint(mainRoadCar), {footprint(sideRoadCar)},
                                           0.49),
                    "priority arbitration keeps main and side road vehicles separated");
        }
        require(slowestMainRoad > 9.0,
                "a vehicle on a higher-priority spline does not give way to a lower-priority one");
        require(sideRoadYieldedToMainRoad,
                "a vehicle on a lower-priority spline gives way to approaching priority traffic");
        require(mainRoadCar.distanceMeters > 100.0 && sideRoadCar.distanceMeters > 60.0,
                "the lower-priority vehicle enters after the priority vehicle has passed");
    }
    auto straightTLeader = makeMapTrafficActor(
        throughJunction(1370, 0.0, -40.0, 0.0, 0.0, 1.0), "straight-t-leader.ovh", 8.0,
        0.0, false);
    auto straightTFollower = makeMapTrafficActor(
        throughJunction(1370, 0.0, -40.0, 0.0, 0.0, 1.0), "straight-t-follower.ovh", 8.0,
        0.0, false);
    straightTLeader.distanceMeters = 25.0;
    straightTFollower.distanceMeters = 12.0;
    straightTLeader.speedMetersPerSecond = 8.0;
    straightTFollower.speedMetersPerSecond = 8.0;
    for (int frame = 0; frame < 180; ++frame) {
        advanceMapTrafficDriving({&straightTLeader, &straightTFollower}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(straightTLeader), {footprint(straightTFollower)},
                                       0.49),
                "straight-through T-junction traffic maintains lane clearance");
    }
    require(straightTLeader.distanceMeters > 70.0 &&
                straightTFollower.distanceMeters > 55.0 &&
                straightTLeader.junctionWaitSeconds == 0.0 &&
                straightTFollower.junctionWaitSeconds == 0.0,
            "straight-through vehicles don't wait when nobody is turning across their path");
    auto acceleratingTLeader = makeMapTrafficActor(
        throughJunction(1372, 0.0, -40.0, 0.0, 0.0, 1.0), "accelerating-t-leader.ovh", 8.0,
        0.0, false);
    auto acceleratingTFollower = makeMapTrafficActor(
        throughJunction(1372, 0.0, -40.0, 0.0, 0.0, 1.0), "accelerating-t-follower.ovh", 8.0,
        0.0, false);
    acceleratingTLeader.distanceMeters = 30.0;
    acceleratingTFollower.distanceMeters = 18.0;
    acceleratingTLeader.speedMetersPerSecond = 5.5;
    acceleratingTFollower.speedMetersPerSecond = 8.0;
    double slowestAcceleratingTFollower = std::numeric_limits<double>::infinity();
    for (int frame = 0; frame < 80; ++frame) {
        advanceMapTrafficDriving({&acceleratingTFollower, &acceleratingTLeader}, {}, 0.1);
        if (acceleratingTFollower.distanceMeters < 70.0) {
            slowestAcceleratingTFollower =
                std::min(slowestAcceleratingTFollower, acceleratingTFollower.speedMetersPerSecond);
        }
    }
    require(slowestAcceleratingTFollower > 2.0 &&
                acceleratingTFollower.distanceMeters > 55.0,
            "a straight-through T-junction follower keeps moving behind an accelerating leader");
    MapTrafficRoute queuedMainRoad = throughJunction(1380, -40.0, 0.0, 90.0, 1.0, 0.0);
    queuedMainRoad.segments.front().pathRules.groups.push_back(mainRoadRules);
    queuedMainRoad.segments[1].pathRules.groups.push_back(sideRoadRules);
    MapTrafficRoute queuedSideTurn;
    auto queuedSideApproach = straight(1383, 0.0, -40.0, 0.0, 34.0);
    queuedSideApproach.pathRules.groups.push_back(sideRoadRules);
    auto queuedSideConnector = straight(1384, 0.0, -6.0, 45.0, std::sqrt(72.0));
    queuedSideConnector.points = {{0.0, -6.0, 0.0}, {6.0, 0.0, 0.0}};
    queuedSideTurn.segments = {queuedSideApproach, queuedSideConnector,
                               straight(1385, 6.0, 0.0, 90.0, 60.0)};
    queuedSideTurn.cumulativeEndDistances = {34.0, 34.0 + std::sqrt(72.0),
                                             94.0 + std::sqrt(72.0)};
    queuedSideTurn.totalLength = queuedSideTurn.cumulativeEndDistances.back();
    auto queuedMainLeader =
        makeMapTrafficActor(queuedMainRoad, "queued-main-leader.ovh", 10.0, 0.0, false);
    auto queuedMainFollower =
        makeMapTrafficActor(queuedMainRoad, "queued-main-follower.ovh", 10.0, 0.0, false);
    auto queuedTTurn =
        makeMapTrafficActor(queuedSideTurn, "queued-t-turn.ovh", 8.0, 0.0, false);
    queuedMainLeader.unscheduledGroupIndex = 0;
    queuedMainFollower.unscheduledGroupIndex = 0;
    queuedTTurn.unscheduledGroupIndex = 0;
    queuedMainLeader.distanceMeters = 28.0;
    queuedMainFollower.distanceMeters = 14.0;
    queuedTTurn.distanceMeters = 26.0;
    queuedMainLeader.speedMetersPerSecond = 8.0;
    queuedMainFollower.speedMetersPerSecond = 8.0;
    queuedTTurn.speedMetersPerSecond = 0.0;
    double slowestQueuedMainFollower = std::numeric_limits<double>::infinity();
    bool sideTurnYielded = false;
    for (int frame = 0; frame < 80; ++frame) {
        advanceMapTrafficDriving({&queuedMainFollower, &queuedMainLeader, &queuedTTurn}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(queuedTTurn),
                                       {footprint(queuedMainLeader), footprint(queuedMainFollower)},
                                       0.49),
                "a local T-junction merge keeps clear of both priority-road vehicles");
        if (queuedMainFollower.distanceMeters < 75.0) {
            slowestQueuedMainFollower =
                std::min(slowestQueuedMainFollower, queuedMainFollower.speedMetersPerSecond);
        }
        sideTurnYielded = sideTurnYielded ||
                          ((queuedTTurn.blockingActor == &queuedMainLeader ||
                            queuedTTurn.blockingActor == &queuedMainFollower) &&
                           queuedTTurn.distanceMeters < 34.0);
    }
    require(slowestQueuedMainFollower > 2.0 && queuedMainLeader.distanceMeters > 70.0 &&
                queuedMainFollower.distanceMeters > 45.0,
            "main-road straight traffic and its follower continue through a T junction");
    require(sideTurnYielded,
            "a T-junction turn yields to main-road traffic without stopping its follower");
    auto stoppedLeader = makeMapTrafficActor(oneSegment(straight(1330, 0.0, 0.0, 0.0, 100.0)),
                                             "stopped-leader.ovh", 0.0, 0.0, false);
    auto blockedFollower = makeMapTrafficActor(oneSegment(straight(1330, 0.0, 0.0, 0.0, 100.0)),
                                               "blocked-follower.ovh", 8.0, 0.0, false);
    stoppedLeader.distanceMeters = 30.0;
    blockedFollower.distanceMeters = 10.0;
    for (int frame = 0; frame < 100; ++frame) {
        advanceMapTrafficDriving({&stoppedLeader, &blockedFollower}, {}, 0.1);
    }
    require(blockedFollower.blockingActor == &stoppedLeader &&
                !blockedFollower.blockingObstacleIndex.has_value(),
            "a queued vehicle records the leader that blocks it");
    auto obstacleBlocked = makeMapTrafficActor(oneSegment(straight(1331, 0.0, 0.0, 0.0, 100.0)),
                                               "obstacle-blocked.ovh", 8.0, 0.0, false);
    obstacleBlocked.distanceMeters = 10.0;
    const MapTrafficObstacle farObstacle{{{30.0, 30.0, 0.0}, 0.0, 0}, 4.0, 2.0};
    const MapTrafficObstacle parkedAhead{{{0.0, 30.0, 0.0}, 0.0, 0}, 4.0, 2.0};
    for (int frame = 0; frame < 100; ++frame) {
        advanceMapTrafficDriving({&obstacleBlocked}, {farObstacle, parkedAhead}, 0.1);
    }
    require(obstacleBlocked.blockReason == MapTrafficBlockReason::Obstacle &&
                obstacleBlocked.blockingActor == nullptr &&
                obstacleBlocked.blockingObstacleIndex == std::optional<std::size_t>{1},
            "a vehicle records which external obstacle blocks it");
    MapTrafficRoute lowTurningRoute;
    auto lowTurningApproach = straight(1220, 0.0, -40.0, 0.0, 40.0);
    auto lowTurningConnector = straight(1221, 0.0, 0.0, 0.0, 10.0);
    lowTurningConnector.points = {{0.0, 0.0, 0.0}, {0.0, 10.0, 0.0}};
    lowTurningRoute.segments = {lowTurningApproach, lowTurningConnector,
                                straight(1222, 0.0, 10.0, 0.0, 60.0)};
    lowTurningRoute.cumulativeEndDistances = {40.0, 50.0, 110.0};
    lowTurningRoute.totalLength = 110.0;
    lowTurningRoute.segments.front().pathRules.groups.push_back(committedPathRules);
    MapTrafficRoute highPriorityTurnApproach =
        oneSegment(straight(1223, -40.0, 0.0, 90.0, 100.0));
    highPriorityTurnApproach.segments.front().pathRules.groups.push_back(approachingPathRules);
    auto lowTurning = makeMapTrafficActor(std::move(lowTurningRoute), "low-turn.ovh", 8.0,
                                          0.0, false);
    auto highPriorityTurn =
        makeMapTrafficActor(std::move(highPriorityTurnApproach), "high-turn.ovh", 8.0, 0.0, false);
    lowTurning.unscheduledGroupIndex = 0;
    highPriorityTurn.unscheduledGroupIndex = 0;
    lowTurning.distanceMeters = 39.0;
    highPriorityTurn.distanceMeters = 35.0;
    lowTurning.speedMetersPerSecond = 0.0;
    highPriorityTurn.speedMetersPerSecond = 0.0;
    lowTurning.junctionWaitSeconds = 10.0;
    require(!mapTrafficRoutePositionOnJunctionPath(lowTurning.route, lowTurning.distanceMeters) &&
                !mapTrafficRoutePositionOnJunctionPath(
                    lowTurning.route, lowTurning.distanceMeters - lowTurning.lengthMeters * 0.5) &&
                mapTrafficRoutePositionOnJunctionPath(
                    lowTurning.route, lowTurning.distanceMeters + lowTurning.lengthMeters * 0.5),
            "the front of a vehicle can commit it to a junction before its center arrives");
    require(mapTrafficHasClearance(footprint(lowTurning), {footprint(highPriorityTurn)}, 0.49),
            "a partially committed turning vehicle begins clear of crossing traffic");
    advanceMapTrafficDriving({&highPriorityTurn, &lowTurning}, {}, 0.1);
    require(lowTurning.distanceMeters > 39.0,
            "a turning vehicle whose front entered the junction is not preempted by an approach");
    requireNear(highPriorityTurn.distanceMeters, 35.0,
                "an approach yields while a turning vehicle occupies the junction boundary");
    for (int frame = 0; frame < 300; ++frame) {
        advanceMapTrafficDriving({&highPriorityTurn, &lowTurning}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(lowTurning), {footprint(highPriorityTurn)}, 0.49),
                "a committed turn preserves physical footprint clearance");
    }
    require(lowTurning.distanceMeters > 60.0 && highPriorityTurn.distanceMeters > 60.0,
            "committed low-priority turns clear before higher-priority approaches");
    MapTrafficRouteSegment northboundSegment = straight(1210, 1.2, -40.0, 0.0, 100.0);
    MapTrafficRouteSegment southboundSegment = straight(1211, -1.2, 60.0, 180.0, 100.0);
    MapTrafficRouteSegment eastboundSegment = straight(1212, -40.0, -1.2, 90.0, 100.0);
    MapTrafficRouteSegment westboundSegment = straight(1213, 60.0, 1.2, 270.0, 100.0);
    auto fourWayNorth =
        makeMapTrafficActor(oneSegment(northboundSegment), "north.ovh", 8.0, 0.0, false);
    auto fourWaySouth =
        makeMapTrafficActor(oneSegment(southboundSegment), "south.ovh", 8.0, 0.0, false);
    auto fourWayEast =
        makeMapTrafficActor(oneSegment(eastboundSegment), "east.ovh", 8.0, 0.0, false);
    auto fourWayWest =
        makeMapTrafficActor(oneSegment(westboundSegment), "west.ovh", 8.0, 0.0, false);
    const std::array<MapTrafficActor*, 4> fourWayActors = {&fourWayNorth, &fourWaySouth,
                                                           &fourWayEast, &fourWayWest};
    const std::vector<MapTrafficActor*> fourWayTraffic(fourWayActors.begin(), fourWayActors.end());
    for (int frame = 0; frame < 400; ++frame) {
        advanceMapTrafficDriving(fourWayTraffic, {}, 0.1);
        for (std::size_t first = 0; first < fourWayActors.size(); ++first) {
            for (std::size_t second = first + 1; second < fourWayActors.size(); ++second) {
                require(mapTrafficHasClearance(footprint(*fourWayActors[first]),
                                               {footprint(*fourWayActors[second])}, 0.49),
                        "four-way junction reservations keep conflicting vehicles separated");
            }
        }
    }
    for (const MapTrafficActor* actor : fourWayActors) {
        require(actor->distanceMeters > 60.0,
                "all four approaches eventually clear a busy junction");
    }
    MapTrafficRoute lowPriorityRoute = oneSegment(straight(1202, 0.0, -40.0, 0.0, 100.0));
    MapTrafficRoute highPriorityRoute = oneSegment(straight(1203, -40.0, 0.0, 90.0, 100.0));
    lowPriorityRoute.segments.front().pathRules = mapTrafficRulesForPath(
        {{false, {"0", "priority", "192", "0"}}}, 0);
    highPriorityRoute.segments.front().pathRules = mapTrafficRulesForPath(
        {{false, {"0", "priority", "64", "0"}}}, 0);
    auto lowPriority = makeMapTrafficActor(std::move(lowPriorityRoute), "low.ovh", 8.0, 0.0, false);
    auto highPriority =
        makeMapTrafficActor(std::move(highPriorityRoute), "high.ovh", 8.0, 0.0, false);
    lowPriority.unscheduledGroupIndex = 0;
    highPriority.unscheduledGroupIndex = 0;
    lowPriority.distanceMeters = 35.0;
    highPriority.distanceMeters = 35.0;
    lowPriority.junctionWaitSeconds = 5.0;
    bool lowPriorityYielded = false;
    bool highPriorityWasServedFirst = false;
    for (int frame = 0; frame < 300; ++frame) {
        advanceMapTrafficDriving({&lowPriority, &highPriority}, {}, 0.1);
        if (lowPriority.junctionWaitSeconds > 0.0) {
            lowPriorityYielded = true;
            highPriorityWasServedFirst = highPriorityWasServedFirst ||
                                         highPriority.distanceMeters > lowPriority.distanceMeters;
        }
    }
    require(lowPriorityYielded && highPriorityWasServedFirst && lowPriority.distanceMeters > 60.0 &&
                highPriority.distanceMeters > 60.0,
            "lower-valued authored priority wins before wait aging closes the priority gap");
    MapTrafficRoute lowPriorityTurningRoute = oneSegment(straight(1208, 0.0, -40.0, 0.0, 100.0));
    lowPriorityTurningRoute.segments.front().points = {{0.0, -40.0, 0.0}, {0.0, 60.0, 0.0}};
    lowPriorityTurningRoute.segments.front().pathRules.groups.push_back(committedPathRules);
    MapTrafficRoute highPriorityJunctionRoute = oneSegment(straight(1209, -40.0, 0.0, 90.0, 100.0));
    highPriorityJunctionRoute.segments.front().points = {{-40.0, 0.0, 0.0}, {60.0, 0.0, 0.0}};
    highPriorityJunctionRoute.segments.front().pathRules.groups.push_back(approachingPathRules);
    auto lowPriorityTurning = makeMapTrafficActor(std::move(lowPriorityTurningRoute),
                                                  "low-priority-turn.ovh", 8.0, 0.0, false);
    auto highPriorityInJunction = makeMapTrafficActor(
        std::move(highPriorityJunctionRoute), "high-priority-junction.ovh", 8.0, 0.0, false);
    lowPriorityTurning.unscheduledGroupIndex = 0;
    highPriorityInJunction.unscheduledGroupIndex = 0;
    lowPriorityTurning.distanceMeters = 35.0;
    highPriorityInJunction.distanceMeters = 35.0;
    lowPriorityTurning.speedMetersPerSecond = 8.0;
    highPriorityInJunction.speedMetersPerSecond = 8.0;
    lowPriorityTurning.junctionWaitSeconds = 30.0;
    bool lowerPriorityClearedConflictFirst = false;
    for (int frame = 0; frame < 300; ++frame) {
        advanceMapTrafficDriving({&highPriorityInJunction, &lowPriorityTurning}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(lowPriorityTurning),
                                       {footprint(highPriorityInJunction)}, 0.49),
                "committed low-priority turns retain physical clearance while clearing");
        lowerPriorityClearedConflictFirst =
            lowerPriorityClearedConflictFirst || (lowPriorityTurning.distanceMeters > 45.0 &&
                                                  highPriorityInJunction.distanceMeters < 60.0);
    }
    require(lowerPriorityClearedConflictFirst && lowPriorityTurning.distanceMeters > 60.0 &&
                highPriorityInJunction.distanceMeters > 60.0,
            "waiting low-priority junction traffic clears before a higher-priority actor already "
            "in the junction");
    MapTrafficRoute rightTurnRoute;
    auto rightTurnApproach = straight(1214, 0.0, -40.0, 0.0, 30.0);
    rightTurnApproach.pathRules.groups.push_back(committedPathRules);
    MapTrafficRouteSegment rightTurnConnector = straight(1215, 0.0, -10.0, 0.0, 1.0);
    constexpr int rightTurnSteps = 16;
    const double quarterTurn = std::acos(-1.0) * 0.5;
    for (int index = 0; index <= rightTurnSteps; ++index) {
        const double angle =
            std::acos(-1.0) - quarterTurn * static_cast<double>(index) / rightTurnSteps;
        rightTurnConnector.points.push_back(
            {10.0 + 10.0 * std::cos(angle), -10.0 + 10.0 * std::sin(angle), 0.0});
    }
    rightTurnConnector.pathRules.groups.push_back(committedPathRules);
    const double connectorLength = [&] {
        double length = 0.0;
        for (std::size_t index = 1; index < rightTurnConnector.points.size(); ++index) {
            const auto& from = rightTurnConnector.points[index - 1];
            const auto& to = rightTurnConnector.points[index];
            length += std::hypot(to[0] - from[0], to[1] - from[1]);
        }
        return length;
    }();
    auto rightTurnExit = straight(1216, 10.0, 0.0, 90.0, 60.0);
    rightTurnExit.pathRules.groups.push_back(committedPathRules);
    rightTurnRoute.segments = {rightTurnApproach, rightTurnConnector, rightTurnExit};
    rightTurnRoute.cumulativeEndDistances = {30.0, 30.0 + connectorLength, 90.0 + connectorLength};
    rightTurnRoute.totalLength = rightTurnRoute.cumulativeEndDistances.back();
    auto queuedMergeVehicle = makeMapTrafficActor(rightTurnRoute, "queued-merge.ovh", 8.0,
                                                  0.0, false);
    queuedMergeVehicle.speedMetersPerSecond = 8.0;
    const MapTrafficObstacle queuedMergeExit{{{14.0, 0.0, 0.0}, std::acos(-1.0) * 0.5, 0},
                                             4.5, 1.8};
    for (int frame = 0; frame < 200; ++frame) {
        advanceMapTrafficDriving({&queuedMergeVehicle}, {queuedMergeExit}, 0.1);
    }
    require(queuedMergeVehicle.distanceMeters <
                30.0 - queuedMergeVehicle.lengthMeters * 0.5 &&
                !mapTrafficActorOnJunctionPath(queuedMergeVehicle,
                                               queuedMergeVehicle.distanceMeters),
            "a vehicle waits before a T-junction when queued traffic blocks its merge exit");
    require(queuedMergeVehicle.distanceMeters >
                30.0 - queuedMergeVehicle.lengthMeters * 0.5 - 1.0,
            "a vehicle waiting for a blocked turn stops close to the junction entry");
    for (int frame = 0; frame < 300; ++frame) {
        advanceMapTrafficDriving({&queuedMergeVehicle}, {}, 0.1);
    }
    require(queuedMergeVehicle.distanceMeters > 60.0,
            "a vehicle completes its turn and clears the T-junction after the exit opens");
    auto unobstructedTurn =
        makeMapTrafficActor(rightTurnRoute, "unobstructed-turn.ovh", 8.0, 0.0, false);
    unobstructedTurn.speedMetersPerSecond = 8.0;
    double minimumUnobstructedTurnSpeed = std::numeric_limits<double>::infinity();
    for (int frame = 0; frame < 160; ++frame) {
        advanceMapTrafficDriving({&unobstructedTurn}, {}, 0.1);
        if (unobstructedTurn.distanceMeters >= 27.0 &&
            unobstructedTurn.distanceMeters <= 30.0 + connectorLength) {
            minimumUnobstructedTurnSpeed =
                std::min(minimumUnobstructedTurnSpeed, unobstructedTurn.speedMetersPerSecond);
        }
    }
    require(unobstructedTurn.distanceMeters > 60.0 &&
                minimumUnobstructedTurnSpeed > 1.5 &&
                unobstructedTurn.junctionWaitSeconds == 0.0,
            "an unblocked turning request crosses the spline boundary without stopping");
    auto tightTurnFollower =
        makeMapTrafficActor(rightTurnRoute, "tight-turn-follower.ovh", 8.0, 0.0, false);
    auto tightTurnLeader =
        makeMapTrafficActor(rightTurnRoute, "tight-turn-leader.ovh", 8.0, 0.0, false);
    tightTurnFollower.distanceMeters = 33.0;
    tightTurnLeader.distanceMeters = 43.0;
    tightTurnFollower.speedMetersPerSecond = 4.0;
    tightTurnLeader.speedMetersPerSecond = 4.0;
    const double tightTurnFollowerStart = tightTurnFollower.distanceMeters;
    const double tightTurnLeaderStart = tightTurnLeader.distanceMeters;
    for (int frame = 0; frame < 120; ++frame) {
        advanceMapTrafficDriving({&tightTurnFollower, &tightTurnLeader}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(tightTurnFollower), {footprint(tightTurnLeader)},
                                       0.49),
                "vehicles following the same tight turn retain physical clearance");
    }
    require(tightTurnFollower.distanceMeters > tightTurnFollowerStart + 10.0 &&
                tightTurnLeader.distanceMeters > tightTurnLeaderStart + 10.0,
            "a follower's curved-path reservation cannot deadlock the turning leader");
    auto splineBoundaryFollower =
        makeMapTrafficActor(rightTurnRoute, "spline-boundary-follower.ovh", 8.0, 0.0, false);
    auto splineBoundaryLeader =
        makeMapTrafficActor(rightTurnRoute, "spline-boundary-leader.ovh", 8.0, 0.0, false);
    splineBoundaryFollower.distanceMeters = 35.0;
    splineBoundaryLeader.distanceMeters = 47.0;
    splineBoundaryFollower.speedMetersPerSecond = 4.0;
    splineBoundaryLeader.speedMetersPerSecond = 4.0;
    splineBoundaryFollower.junctionWaitSeconds = 20.0;
    for (int frame = 0; frame < 200; ++frame) {
        advanceMapTrafficDriving({&splineBoundaryFollower, &splineBoundaryLeader}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(splineBoundaryFollower),
                                       {footprint(splineBoundaryLeader)}, 0.49),
                "a follower remains separated while traversing a multi-spline turn");
    }
    require(splineBoundaryLeader.distanceMeters > 60.0 &&
                splineBoundaryFollower.distanceMeters > 50.0,
            "vehicles following through a spline boundary keep moving around the turn");
    const auto makeSplitRoadCurve = [&](int firstSplineId) {
        MapTrafficRoute route;
        route.segments = {straight(firstSplineId, 0.0, -30.0, 0.0, 30.0),
                          straight(firstSplineId + 1, 0.0, 0.0, 90.0, 40.0)};
        route.cumulativeEndDistances = {30.0, 70.0};
        route.totalLength = 70.0;
        return route;
    };
    auto differentlySegmentedCurveLeader =
        makeMapTrafficActor(makeSplitRoadCurve(1510), "segmented-curve-leader.ovh", 6.0, 0.0,
                            false);
    auto differentlySegmentedCurveFollower =
        makeMapTrafficActor(makeSplitRoadCurve(1512), "segmented-curve-follower.ovh", 6.0, 0.0,
                            false);
    differentlySegmentedCurveLeader.distanceMeters = 26.0;
    differentlySegmentedCurveFollower.distanceMeters = 14.0;
    differentlySegmentedCurveLeader.speedMetersPerSecond = 6.0;
    differentlySegmentedCurveFollower.speedMetersPerSecond = 6.0;
    bool curveWasTreatedAsCrossing = false;
    for (int frame = 0; frame < 240; ++frame) {
        advanceMapTrafficDriving(
            {&differentlySegmentedCurveLeader, &differentlySegmentedCurveFollower}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(differentlySegmentedCurveLeader),
                                       {footprint(differentlySegmentedCurveFollower)}, 0.49),
                "different spline segmentations preserve clearance on the same curved road");
        curveWasTreatedAsCrossing =
            curveWasTreatedAsCrossing ||
            differentlySegmentedCurveLeader.blockReason ==
                MapTrafficBlockReason::CrossingReservation ||
            differentlySegmentedCurveFollower.blockReason ==
                MapTrafficBlockReason::CrossingReservation;
    }
    require(!curveWasTreatedAsCrossing &&
                differentlySegmentedCurveLeader.distanceMeters > 60.0 &&
                differentlySegmentedCurveFollower.distanceMeters > 50.0,
            "different spline boundaries do not create crossing blocks on a shared road");
    {
        const auto minimumTurnSpeed = [&](bool withStraightAheadCar) {
            auto turningCar =
                makeMapTrafficActor(rightTurnRoute, "turning-car.ovh", 10.0, 0.0, false);
            auto straightAheadCar = makeMapTrafficActor(
                oneSegment(straight(1230, 0.0, 10.0, 0.0, 50.0)), "straight-ahead.ovh", 0.0,
                0.0, false);
            double minimumSpeed = std::numeric_limits<double>::infinity();
            for (int frame = 0; frame < 120; ++frame) {
                if (withStraightAheadCar) {
                    advanceMapTrafficDriving({&turningCar, &straightAheadCar}, {}, 0.1);
                } else {
                    advanceMapTrafficDriving({&turningCar}, {}, 0.1);
                }
                if (turningCar.distanceMeters > 5.0 && turningCar.distanceMeters < 40.0) {
                    minimumSpeed = std::min(minimumSpeed, turningCar.speedMetersPerSecond);
                }
            }
            require(turningCar.distanceMeters > 60.0,
                    "a turning car completes its turn past a car on the straight-ahead road");
            return minimumSpeed;
        };
        require(minimumTurnSpeed(true) >= minimumTurnSpeed(false) - 0.1,
                "a stopped car straight ahead on another road does not slow a turning car");
    }
    MapTrafficRoute highPriorityCrossingRoute =
        oneSegment(straight(1217, -40.0, -5.0, 90.0, 100.0));
    highPriorityCrossingRoute.segments.front().pathRules.groups.push_back(approachingPathRules);
    auto waitingRightTurn =
        makeMapTrafficActor(rightTurnRoute, "waiting-right-turn.ovh", 8.0, 0.0, false);
    auto highPriorityCrossing = makeMapTrafficActor(highPriorityCrossingRoute,
                                                    "high-priority-crossing.ovh", 8.0, 0.0, false);
    waitingRightTurn.unscheduledGroupIndex = 0;
    highPriorityCrossing.unscheduledGroupIndex = 0;
    waitingRightTurn.distanceMeters = 25.0;
    highPriorityCrossing.distanceMeters = 25.0;
    advanceMapTrafficDriving({&waitingRightTurn, &highPriorityCrossing}, {}, 0.1);
    require(waitingRightTurn.junctionWaitSeconds > 0.0,
            "a right-turning approach accrues wait time when lookahead sees crossing traffic");
    auto lateRightTurn =
        makeMapTrafficActor(rightTurnRoute, "late-right-turn.ovh", 8.0, 0.0, false);
    auto passedPriorityCrossing = makeMapTrafficActor(
        highPriorityCrossingRoute, "passed-priority-crossing.ovh", 30.0, 0.0, false);
    lateRightTurn.unscheduledGroupIndex = 0;
    passedPriorityCrossing.unscheduledGroupIndex = 0;
    passedPriorityCrossing.distanceMeters = 20.0;
    passedPriorityCrossing.speedMetersPerSecond = 30.0;
    double lateTurnMinimumSpeed = std::numeric_limits<double>::infinity();
    for (int frame = 0; frame < 12; ++frame) {
        advanceMapTrafficDriving({&lateRightTurn, &passedPriorityCrossing}, {}, 0.1);
        lateTurnMinimumSpeed =
            std::min(lateTurnMinimumSpeed, lateRightTurn.speedMetersPerSecond);
    }
    require(lateRightTurn.distanceMeters > 8.0 && lateTurnMinimumSpeed > 7.0,
            "a turn continues when priority traffic clears before it reaches the crossing");
    MapTrafficRoute distantCrossingRoute =
        oneSegment(straight(1330, -40.0, -5.0, 90.0, 100.0));
    distantCrossingRoute.segments.front().points = {{-40.0, -5.0, 0.0}, {60.0, -5.0, 0.0}};
    auto rightTurnPastDistantClaim =
        makeMapTrafficActor(rightTurnRoute, "right-turn-past-distant-claim.ovh", 8.0,
                            0.0, false);
    auto distantCrossing =
        makeMapTrafficActor(distantCrossingRoute, "distant-crossing.ovh", 8.0, 0.0, false);
    rightTurnPastDistantClaim.distanceMeters = 25.0;
    distantCrossing.distanceMeters = 5.0;
    rightTurnPastDistantClaim.speedMetersPerSecond = 8.0;
    distantCrossing.speedMetersPerSecond = 8.0;
    bool rightTurnEnteredWhileDistantClaimHeld = false;
    for (int frame = 0; frame < 100; ++frame) {
        advanceMapTrafficDriving({&rightTurnPastDistantClaim, &distantCrossing}, {}, 0.1);
        rightTurnEnteredWhileDistantClaimHeld =
            rightTurnEnteredWhileDistantClaimHeld ||
            (distantCrossing.junctionClaimEndMeters.has_value() &&
             rightTurnPastDistantClaim.distanceMeters > 40.0 &&
             distantCrossing.distanceMeters < 35.0);
    }
    require(rightTurnEnteredWhileDistantClaimHeld,
            "a right turn uses a safe arrival gap instead of waiting for a distant crossing "
            "vehicle to clear the whole junction");
    MapTrafficRoute distantHorizontalRoute;
    distantHorizontalRoute.segments = {
        straight(1331, -40.0, 0.0, 90.0, 30.0),
        straight(1332, -10.0, 0.0, 90.0, 20.0),
        straight(1333, 10.0, 0.0, 90.0, 30.0)};
    distantHorizontalRoute.segments[1].points = {{-10.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
    distantHorizontalRoute.cumulativeEndDistances = {30.0, 50.0, 80.0};
    distantHorizontalRoute.totalLength = 80.0;
    MapTrafficRoute nearVerticalRoute;
    nearVerticalRoute.segments = {
        straight(1334, 0.0, -40.0, 0.0, 30.0),
        straight(1335, 0.0, -10.0, 0.0, 20.0),
        straight(1336, 0.0, 10.0, 0.0, 30.0)};
    nearVerticalRoute.segments[1].points = {{0.0, -10.0, 0.0}, {0.0, 10.0, 0.0}};
    nearVerticalRoute.cumulativeEndDistances = {30.0, 50.0, 80.0};
    nearVerticalRoute.totalLength = 80.0;
    auto distantHorizontal = makeMapTrafficActor(
        distantHorizontalRoute, "distant-horizontal.ovh", 8.0, 0.0, false);
    auto nearVertical = makeMapTrafficActor(
        nearVerticalRoute, "near-vertical.ovh", 8.0, 0.0, false);
    distantHorizontal.distanceMeters = 12.0;
    nearVertical.distanceMeters = 27.0;
    distantHorizontal.speedMetersPerSecond = 8.0;
    nearVertical.speedMetersPerSecond = 8.0;
    bool verticalClearedBeforeHorizontalEntered = false;
    for (int frame = 0; frame < 80; ++frame) {
        advanceMapTrafficDriving({&distantHorizontal, &nearVertical}, {}, 0.1);
        verticalClearedBeforeHorizontalEntered =
            verticalClearedBeforeHorizontalEntered ||
            (nearVertical.distanceMeters > 42.0 && distantHorizontal.distanceMeters < 30.0 &&
             distantHorizontal.junctionClaimEndMeters.has_value());
        require(mapTrafficHasClearance(footprint(distantHorizontal), {footprint(nearVertical)},
                                       0.49),
                "time-separated crossing traffic keeps physical clearance");
    }
    require(verticalClearedBeforeHorizontalEntered,
            "straight-through traffic can clear before a distant crossing claimant arrives");
    auto agedRightTurn =
        makeMapTrafficActor(rightTurnRoute, "aged-right-turn.ovh", 8.0, 0.0, false);
    auto agedCrossing =
        makeMapTrafficActor(highPriorityCrossingRoute, "aged-crossing.ovh", 8.0, 0.0, false);
    agedRightTurn.unscheduledGroupIndex = 0;
    agedCrossing.unscheduledGroupIndex = 0;
    agedRightTurn.distanceMeters = 25.0;
    agedCrossing.distanceMeters = 25.0;
    agedRightTurn.speedMetersPerSecond = 8.0;
    agedCrossing.speedMetersPerSecond = 8.0;
    agedRightTurn.junctionWaitSeconds = 20.0;
    bool agedRightTurnEnteredFirst = false;
    for (int frame = 0; frame < 300; ++frame) {
        advanceMapTrafficDriving({&agedRightTurn, &agedCrossing}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(agedRightTurn), {footprint(agedCrossing)}, 0.49),
                "a right-turning vehicle keeps physical clearance after gaining junction priority");
        agedRightTurnEnteredFirst =
            agedRightTurnEnteredFirst ||
            (agedRightTurn.distanceMeters > 36.0 && agedCrossing.distanceMeters < 60.0);
    }
    require(
        agedRightTurnEnteredFirst && agedRightTurn.distanceMeters > 70.0 &&
            agedCrossing.distanceMeters > 60.0,
        "a persistently waiting lower-priority right turn eventually gets a safe junction slot");
    auto entryActor = makeMapTrafficActor(road, "entry.ovh", 12.0, 0.0, false);
    require(!canSpawnMapTrafficEntry(entryActor, false, {}),
            "vehicles do not materialize at a visible endpoint");
    require(!canSpawnMapTrafficEntry(entryActor, true, {footprint(entryActor)}),
            "occupied endpoint entries queue rather than overlapping spawns");
    require(canSpawnMapTrafficEntry(entryActor, true, {}),
            "an off-screen, clear endpoint admits a spawn at distance zero");
    requireNear(entryActor.distanceMeters, 0.0,
                "entry admission does not offset actors into a route");
    require(mapTrafficRoutesShareCompleteOrderedPath({1, 2, 3, 4}, {2, 3}),
            "a complete timetable path can be a contiguous part of a longer endpoint route");
    require(mapTrafficRoutesShareCompleteOrderedPath({2, 3}, {1, 2, 3, 4}),
            "a complete endpoint route can be a contiguous part of a longer timetable path");
    require(!mapTrafficRoutesShareCompleteOrderedPath({1, 2, 3}, {2, 4}),
            "route matching rejects partial or out-of-order spline overlaps");
    require(!mapTrafficRoutesShareCompleteOrderedPath({1}, {1, 2}),
            "route matching requires at least two ordered spline IDs");
    require(mapTrafficRoutePrefixMatchingTimetable({1, 2, 3, 4}, {2, 3, 8}) ==
                std::vector<int>{2, 3},
            "an endpoint route matches a consecutive timetable section from its first spline");
    require(mapTrafficRoutePrefixMatchingTimetable({1, 2, 3, 4}, {9, 2, 3}).empty(),
            "timetable overlap after the route endpoint does not qualify a bus route");
    entryActor.distanceMeters = entryActor.route.totalLength;
    require(mapTrafficRouteEndReadyForUnload(entryActor),
            "vehicles unload immediately when they reach an open route end");
    entryActor.speedMetersPerSecond = 4.0;
    entryActor.junctionWaitSeconds = 8.0;
    resetMapTrafficActorForEntry(entryActor);
    requireNear(entryActor.distanceMeters, 0.0,
                "recycled traffic re-enters at route distance zero");
    requireNear(entryActor.speedMetersPerSecond, 0.0,
                "recycled traffic starts from rest rather than retaining route speed");
    requireNear(entryActor.junctionWaitSeconds, 0.0,
                "recycled traffic does not retain junction wait priority from its prior trip");
    require(!mapTrafficRouteEndReadyForUnload(entryActor),
            "vehicles do not unload before reaching an open route end");
    {
        // Camera at the origin looking along +Y with a 90 degree, square perspective frustum.
        constexpr double nearPlane = 0.1;
        constexpr double farPlane = 1000.0;
        std::array<double, 16> viewProjection = {};
        viewProjection[0] = 1.0;
        viewProjection[9] = 1.0;
        viewProjection[6] = (farPlane + nearPlane) / (farPlane - nearPlane);
        viewProjection[14] = -2.0 * farPlane * nearPlane / (farPlane - nearPlane);
        viewProjection[7] = 1.0;
        const std::array<double, 3> camera = {0.0, 0.0, 0.0};
        require(
            !mapTrafficEntryHiddenFromCamera({0.0, 100.0, 0.0}, 3.0, camera, viewProjection, 50.0),
            "an entry in front of the camera is visible, so nothing spawns there");
        require(
            mapTrafficEntryHiddenFromCamera({0.0, -100.0, 0.0}, 3.0, camera, viewProjection, 50.0),
            "an entry behind the camera is hidden");
        require(
            mapTrafficEntryHiddenFromCamera({200.0, 100.0, 0.0}, 3.0, camera, viewProjection, 50.0),
            "an entry beside the view frustum is hidden");
        require(!mapTrafficEntryHiddenFromCamera({103.0, 100.0, 0.0}, 5.0, camera, viewProjection,
                                                 50.0),
                "a vehicle footprint straddling the screen edge counts as visible");
        require(
            !mapTrafficEntryHiddenFromCamera({0.0, -30.0, 0.0}, 3.0, camera, viewProjection, 50.0),
            "entries near the camera never admit spawns even when behind it");
        require(
            mapTrafficEntryHiddenFromCamera({0.0, 1500.0, 0.0}, 3.0, camera, viewProjection, 50.0),
            "entries beyond the far plane are hidden");
    }
    MapTrafficLaneNetwork twoWay;
    auto forward = straight(1300, 0.0, 0.0, 0.0, 20.0);
    auto reverse = forward;
    reverse.reversed = true;
    reverse.pathLateralMeters = 3.5;
    auto continuation = forward;
    continuation.spline.placement = straight(1301, 0.0, 20.0, 0.0, 20.0).spline.placement;
    auto reverseContinuation = continuation;
    reverseContinuation.reversed = true;
    reverseContinuation.pathLateralMeters = 3.5;
    for (const auto& lane : {forward, reverse, continuation, reverseContinuation}) {
        twoWay.addSegment(lane);
    }
    twoWay.finalize();
    require(twoWay.entryLanes() == std::vector<MapTrafficLaneStart>{{1300, 0}, {1301, 1}},
            "reverse lanes enter at the far open end, never at an internal spline join");
    MapTrafficLaneNetwork circular;
    auto circle = straight(1400, 0.0, 0.0, 0.0, 40.0 * std::acos(-1.0));
    circle.spline.placement.radius = 20.0;
    circular.addSegment(circle);
    circular.finalize();
    require(circular.entryLanes().empty(), "a closed circuit has no endpoint spawn candidates");
    const auto circleRoute = circular.buildRoute(1400);
    auto circulating = makeMapTrafficActor(*circleRoute, "car.ovh", 5.0, 0.0, false);
    circulating.distanceMeters = circleRoute->totalLength - 1.0;
    advanceMapTrafficDriving({&circulating}, {}, 1.0);
    require(circulating.distanceMeters < 5.0,
            "the driving controller preserves genuine loops without endpoint teleportation");
    auto tightCurveSegment =
        straight(1401, 0.0, 0.0, 0.0, 10.0 * std::acos(-1.0));
    tightCurveSegment.spline.placement.radius = 5.0;
    auto tightCurveRoute = oneSegment(tightCurveSegment);
    tightCurveRoute.closed = true;
    auto tightCurveFollower =
        makeMapTrafficActor(tightCurveRoute, "tight-curve-follower.ovh", 4.0, 0.0, false);
    auto tightCurveLeader =
        makeMapTrafficActor(tightCurveRoute, "tight-curve-leader.ovh", 4.0, 0.0, false);
    tightCurveFollower.distanceMeters = 1.0;
    tightCurveLeader.distanceMeters = 7.5;
    tightCurveFollower.speedMetersPerSecond = 2.5;
    tightCurveLeader.speedMetersPerSecond = 2.5;
    require(mapTrafficHasClearance(footprint(tightCurveFollower), {footprint(tightCurveLeader)},
                                   0.49),
            "close vehicles on a tight curve begin with physical clearance");
    double tightCurveFollowerTravel = 0.0;
    double tightCurveLeaderTravel = 0.0;
    for (int frame = 0; frame < 200; ++frame) {
        const double previousFollowerDistance = tightCurveFollower.distanceMeters;
        const double previousLeaderDistance = tightCurveLeader.distanceMeters;
        advanceMapTrafficDriving({&tightCurveFollower, &tightCurveLeader}, {}, 0.1);
        double followerStep = tightCurveFollower.distanceMeters - previousFollowerDistance;
        double leaderStep = tightCurveLeader.distanceMeters - previousLeaderDistance;
        if (followerStep < 0.0) {
            followerStep += tightCurveRoute.totalLength;
        }
        if (leaderStep < 0.0) {
            leaderStep += tightCurveRoute.totalLength;
        }
        tightCurveFollowerTravel += followerStep;
        tightCurveLeaderTravel += leaderStep;
        require(mapTrafficHasClearance(footprint(tightCurveFollower), {footprint(tightCurveLeader)},
                                       0.49),
                "vehicles on a tight curve retain physical clearance while moving");
    }
    require(tightCurveFollowerTravel > 20.0,
            "a follower on a tight curve keeps progressing with physical clearance");
    require(tightCurveLeaderTravel > 20.0,
            "a leader on a tight curve keeps progressing with physical clearance");

    MapTrafficRoute signalRoute;
    MapTrafficRouteSegment approach = straight(1600, 0.0, -20.0, 0.0, 20.0);
    MapTrafficRouteSegment signalPath = straight(1601, 0.0, 0.0, 0.0, 4.0);
    MapTrafficSignalSchedule redThenGreen;
    redThenGreen.cycleDurationSeconds = 200.0;
    redThenGreen.phases = {{0, 100.0}, {6, 100.0}};
    signalPath.trafficSignal = redThenGreen;
    MapTrafficRouteSegment exit = straight(1602, 0.0, 4.0, 0.0, 80.0);
    signalRoute.segments = {approach, signalPath, exit};
    signalRoute.cumulativeEndDistances = {20.0, 24.0, 104.0};
    signalRoute.totalLength = 104.0;
    auto signalActor = makeMapTrafficActor(signalRoute, "car.ovh", 8.0, 0.0, false);
    advanceMapTrafficDriving({&signalActor}, {}, 0.1);
    require(signalActor.speedMetersPerSecond < 8.0 &&
                signalActor.speedMetersPerSecond > 0.0,
            "AI traffic eases off speed while approaching a red signal");
    for (int frame = 0; frame < 299; ++frame) {
        advanceMapTrafficDriving({&signalActor}, {}, 0.1);
    }
    require(signalActor.distanceMeters >= 16.8 && signalActor.distanceMeters <= 17.001 &&
                signalActor.speedMetersPerSecond < 1.0e-6,
            "AI traffic stops before entering a signalized path on red");
    advanceMapTrafficDriving({&signalActor}, {}, 3.0, 100.0);
    require(signalActor.distanceMeters > 20.0,
            "AI traffic proceeds through the signalized path on green");

    MapTrafficRoute signalStartRoute;
    signalStartRoute.segments = {signalPath, exit};
    signalStartRoute.cumulativeEndDistances = {4.0, 84.0};
    signalStartRoute.totalLength = 84.0;
    auto signalStartActor = makeMapTrafficActor(signalStartRoute, "car.ovh", 8.0, 0.0, false);
    advanceMapTrafficDriving({&signalStartActor}, {}, 1.0);
    require(signalStartActor.distanceMeters == 0.0 &&
                signalStartActor.speedMetersPerSecond < 1.0e-6,
            "AI traffic starting on a signalized path waits at route entry on red");
    advanceMapTrafficDriving({&signalStartActor}, {}, 1.0, 100.0);
    require(signalStartActor.distanceMeters > 0.0,
            "AI traffic starting on a signalized path proceeds on green");

    MapTrafficRoute loopSignalRoute = *circleRoute;
    loopSignalRoute.segments.front().trafficSignal = redThenGreen;
    auto loopSignalActor = makeMapTrafficActor(loopSignalRoute, "car.ovh", 8.0, 0.0, false);
    loopSignalActor.distanceMeters = loopSignalRoute.totalLength - 5.0;
    for (int frame = 0; frame < 100; ++frame) {
        advanceMapTrafficDriving({&loopSignalActor}, {}, 0.05);
    }
    require(loopSignalActor.distanceMeters < loopSignalRoute.totalLength - 2.0 &&
                loopSignalActor.speedMetersPerSecond < 1.0e-6,
            "AI traffic approaching a signal across a loop wrap stops before red");
    advanceMapTrafficDriving({&loopSignalActor}, {}, 3.0, 100.0);
    require(loopSignalActor.distanceMeters < 10.0,
            "AI traffic resumes across the loop wrap when the signal turns green");

    bool rejected = false;
    try {
        advanceMapTrafficDriving({&entryActor}, {}, -1.0);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "invalid driving time is explicitly rejected");
}

void verifyTrafficRules() {
    using namespace openbus::map;
    const std::vector<MapTileRule> authoredRules = {
        {false, {"0", "speedlimit", "36", "0"}},
        {false, {"0", "trafficdensity", "0.5", "0"}},
        {false, {"0", "trafficdensity", "0.1", "1"}},
        {false, {"0", "trafficdensity", "0", "1"}},
        {false, {"0", "priority", "192", "0"}},
        {false, {"0", "no_cars", "1", "0"}},
        {false, {"0", "bus", "1", "0"}},
        {false, {"0", "trucks", "1", "0"}},
        {false, {"0", "overtaking_prohib", "1", "0"}},
        {false, {"1", "speedlimit", "not-a-number", "0"}},
    };
    const MapTrafficPathRules rules = mapTrafficRulesForPath(authoredRules, 0);
    require(mapTrafficPathSpeedLimitForVehicle(rules, {0, 0, false}) == 36.0 &&
                mapTrafficPathPriorityForVehicle(rules, {0, 0, false}) == 192.0 &&
                !mapTrafficPathSpeedLimitForVehicle(rules, {0, 1, false}).has_value() &&
                mapTrafficPathPriorityForVehicle(rules, {0, 1, false}) == 128.0,
            "speed and priority rules are scoped to their authored AI group");
    require(rules.groups.size() == 2 && rules.groups[0].carsProhibited == true &&
                rules.groups[0].busesAllowed == true && rules.groups[0].trucksAllowed == true &&
                rules.groups[0].overtakingProhibited == true,
            "access and overtaking rules decode as group-specific overrides");
    requireNear(mapTrafficPathDensityForVehicle(rules, {0, 0, false}), 0.5,
                "traffic density resolves for its matching unscheduled group");
    requireNear(mapTrafficPathDensityForVehicle(rules, {0, 1, false}), 0.0,
                "the last matching group density rule takes effect");
    require(
        mapTrafficPathAllowsVehicle(rules, {1, 0, false}) &&
            mapTrafficPathAllowsVehicle(rules, {2, 0, false}) &&
            mapTrafficPathAllowsVehicle(rules, {3, 0, false}) &&
            !mapTrafficPathAllowsVehicle(rules, {0, 0, false}) &&
            !mapTrafficPathAllowsVehicle(rules, {3, 1, false}) &&
            mapTrafficPathAllowsVehicle(rules, {0, 2, false}) &&
            mapTrafficPathAllowsVehicle(rules, {2, std::nullopt, true}),
        "path access rules enforce cars, cabs, buses, trucks, density and scheduled exemptions");
    const MapTrafficPathRules afterKill =
        mapTrafficRulesForPath({{false, {"0", "no_cars", "1", "0"}},
                                {true, {"0", "no_cars", "1", "0"}},
                                {false, {"0", "trafficdensity", "0", "1"}},
                                {true, {"0", "trafficdensity", "0", "1"}}},
                               0);
    require(!afterKill.groups[0].carsProhibited.has_value() &&
                mapTrafficPathDensityForVehicle(afterKill, {0, 1, false}) == 1.0,
            "kill rules remove their matching path restrictions and group density override");

    bool malformedRuleRejected = false;
    try {
        static_cast<void>(mapTrafficRulesForPath(
            std::vector<MapTileRule>{{false, {"0", "speedlimit", "fast", "0"}}}, 0));
    } catch (const std::invalid_argument&) {
        malformedRuleRejected = true;
    }
    require(malformedRuleRejected,
            "malformed values for recognized map traffic rules fail explicitly");
    bool malformedPathRejected = false;
    try {
        static_cast<void>(mapTrafficRulesForPath(
            std::vector<MapTileRule>{{false, {"path", "speedlimit", "30", "0"}}}, 0));
    } catch (const std::invalid_argument&) {
        malformedPathRejected = true;
    }
    require(malformedPathRejected, "malformed path indices fail explicitly");

    MapTrafficRouteSegment restrictedSegment;
    restrictedSegment.spline = {makeSpline(1300, 0, 0, 0.0), 0, 0};
    restrictedSegment.pathRules = rules;
    MapTrafficLaneNetwork restrictedNetwork;
    restrictedNetwork.addSegment(restrictedSegment);
    restrictedNetwork.finalize();
    require(
        !restrictedNetwork.buildRoute(1300, 0, 0, 0.0, {0, 0, false}) &&
            restrictedNetwork.buildRoute(1300, 0, 0, 0.0, {2, 0, false}) &&
            !restrictedNetwork.buildRoute(1300, 0, 0, 0.0, {3, 1, false}) &&
            restrictedNetwork.buildRoute(1300, 0, 0, 0.0, {2, std::nullopt, true}),
        "route construction rejects prohibited car/group lanes but preserves permitted traffic");

    const auto straightSegment = [](int id, double x, double y) {
        MapTrafficRouteSegment segment{{makeSpline(id, 0, 0, y), 0, 0}};
        segment.spline.placement.localX = x;
        segment.spline.placement.elevation = 0.0;
        segment.spline.placement.length = 10.0;
        return segment;
    };
    MapTrafficLaneNetwork densityNetwork;
    auto lowDensityEntry = straightSegment(1310, 0.0, 0.0);
    MapTrafficGroupRules lowDensityRules;
    lowDensityRules.groupIndex = 0;
    lowDensityRules.density = 0.1;
    lowDensityEntry.pathRules.groups.push_back(lowDensityRules);
    densityNetwork.addSegment(lowDensityEntry);
    densityNetwork.addSegment(straightSegment(1311, 0.0, 10.0));
    auto highDensityEntry = straightSegment(1312, 10.0, 0.0);
    MapTrafficGroupRules highDensityRules;
    highDensityRules.groupIndex = 0;
    highDensityRules.density = 1.0;
    highDensityEntry.pathRules.groups.push_back(highDensityRules);
    densityNetwork.addSegment(highDensityEntry);
    densityNetwork.addSegment(straightSegment(1313, 10.0, 10.0));
    auto closedEntry = straightSegment(1314, 20.0, 0.0);
    MapTrafficGroupRules closedRules;
    closedRules.groupIndex = 0;
    closedRules.density = 0.0;
    closedEntry.pathRules.groups.push_back(closedRules);
    densityNetwork.addSegment(closedEntry);
    densityNetwork.addSegment(straightSegment(1315, 20.0, 10.0));
    densityNetwork.finalize();
    std::size_t lowDensitySelections = 0;
    std::size_t highDensitySelections = 0;
    for (std::size_t seed = 1; seed <= 1000; ++seed) {
        const auto entry = densityNetwork.selectEntryLane({0, 0, false}, seed);
        require(entry.has_value() && entry->splineId != 1314,
                "zero-density paths are excluded from unscheduled entry selection");
        if (entry->splineId == 1310) {
            ++lowDensitySelections;
        } else if (entry->splineId == 1312) {
            ++highDensitySelections;
        }
    }
    require(highDensitySelections > lowDensitySelections * 5,
            "positive path density scales the relative selection frequency for its AI group");

    MapTrafficRouteSegment speedSegment;
    speedSegment.spline = {makeSpline(1301, 0, 0, 0.0), 0, 0};
    speedSegment.spline.placement.length = 100.0;
    MapTrafficGroupRules speedRules;
    speedRules.groupIndex = 0;
    speedRules.speedLimitKmh = 18.0;
    speedSegment.pathRules.groups.push_back(speedRules);
    MapTrafficRoute speedRoute;
    speedRoute.segments = {speedSegment};
    speedRoute.totalLength = 100.0;
    speedRoute.cumulativeEndDistances = {100.0};
    MapTrafficActor speedCapped =
        makeMapTrafficActor(std::move(speedRoute), "speed.ovh", 15.0, 0.0, false);
    speedCapped.speedMetersPerSecond = 0.0;
    speedCapped.unscheduledGroupIndex = 0;
    advanceMapTrafficDriving({&speedCapped}, {}, 4.0);
    require(speedCapped.speedMetersPerSecond <= 5.0 + 1.0e-8,
            "authored speed limits cap vehicle speed in km/h converted to m/s");
}

} // namespace

int main() {
    std::size_t verifiedAiSpawns = 0;
    std::size_t verifiedHumanSpawns = 0;
    std::size_t verifiedAiPopulationSpawns = 0;
    std::size_t verifiedHumanPopulationSpawns = 0;
    try {
        using namespace openbus::map;
        verifyDrivingBehavior();
        verifyTrafficRules();
        const auto spawnPopulation = [](const MapTrafficRouteNetwork& routeNetwork,
                                        const std::vector<int>& splineIds,
                                        const std::string& modelPath, double speedMetersPerSecond,
                                        const std::array<double, 3>& spawnCenter,
                                        std::size_t targetCount, double spacingMeters,
                                        double maximumDistanceMeters, double endpointMarginMeters) {
            std::vector<MapTrafficActor> actors;
            for (std::size_t candidate = 0;
                 candidate < targetCount * 4 && actors.size() < targetCount; ++candidate) {
                if (splineIds.empty()) {
                    break;
                }
                const auto route = routeNetwork.buildRoute(splineIds[candidate % splineIds.size()]);
                if (!route) {
                    continue;
                }
                MapTrafficActor actor =
                    makeMapTrafficActor(*route, modelPath, speedMetersPerSecond, 0.0, false);
                if (placeMapTrafficActorNearLooping(
                        actor, static_cast<double>(candidate) * spacingMeters, spawnCenter[0],
                        spawnCenter[1], maximumDistanceMeters, endpointMarginMeters)) {
                    actors.push_back(std::move(actor));
                }
            }
            return actors;
        };

        MapTrafficRouteNetwork network;
        network.addTileSplines({makeSpline(10, 0, 11, 6.0)}, 1, 2);
        network.addTileSplines({makeSpline(11, 10, 0, 16.0)}, 1, 2);
        network.finalize();
        require(network.splineIds() == std::vector<int>({10, 11}),
                "traffic route start IDs are returned in deterministic order");
        require(network.routeStartSplineIds() == std::vector<int>({10}),
                "connected splines expose one deterministic start for their full route");
        const auto firstEndpoints = network.endpointStatus(10);
        const auto lastEndpoints = network.endpointStatus(11);
        require(firstEndpoints.has_value() && !firstEndpoints->startConnected &&
                    firstEndpoints->endConnected && lastEndpoints.has_value() &&
                    lastEndpoints->startConnected && !lastEndpoints->endConnected,
                "only the unjoined ends of an open connected route are marked disconnected");
        const auto route = network.buildRoute(10);
        require(route.has_value() && route->segments.size() == 2 && !route->closed,
                "route follows reciprocal spline links and stops at the open end");
        requireNear(route->totalLength, 20.0, "route length sums linked splines");
        require(
            std::none_of(route->segments.begin(), route->segments.end(),
                         [](const MapTrafficRouteSegment& segment) { return segment.reversed; }),
            "a normal connected route follows its authored spline direction");
        MapTrafficLaneNetwork lanes;
        MapTrafficRouteSegment firstLane{{makeSpline(40, 0, 0, 0.0), 0, 0}};
        firstLane.pathLateralMeters = -2.0;
        firstLane.pathHeightMeters = 0.1;
        MapTrafficRouteSegment secondLane{{makeSpline(41, 0, 0, 20.0), 0, 0}};
        secondLane.pathLateralMeters = -2.0;
        secondLane.pathHeightMeters = 0.1;
        MapTrafficRouteSegment junction;
        junction.points = {{3.0, 10.0, 2.1}, {3.0, 15.0, 2.1}, {3.0, 20.0, 2.1}};
        MapTrafficRouteSegment wrongWay = firstLane;
        wrongWay.reversed = true;
        lanes.addSegment(firstLane);
        lanes.addSegment(secondLane);
        lanes.addSegment(junction);
        lanes.addSegment(wrongWay);
        lanes.finalize();
        const auto drivingRoute = lanes.buildRoute(40);
        require(drivingRoute && drivingRoute->segments.size() == 3 && !drivingRoute->closed,
                "authored lanes join through scenery even without reciprocal spline IDs");
        MapTrafficActor drivingActor =
            makeMapTrafficActor(*drivingRoute, "Vehicles\\regular.ovh", 2.0, 0.0, false);
        for (int step = 0; step < 29; ++step) {
            require(advanceMapTrafficActor(drivingActor, 0.5),
                    "vehicle stays active until the final unconnected lane end");
            const auto pose =
                sampleMapTrafficRoute(drivingActor.route, drivingActor.distanceMeters, 0.0);
            require(pose.has_value(), "driving lane pose is valid");
            requireNear(pose->position[0], 3.0, "vehicle uses F2 lane lateral position");
            requireNear(pose->position[1], static_cast<double>(step + 1),
                        "vehicle travels forward without bouncing or teleporting");
            requireNear(pose->position[2], 2.1, "vehicle uses F2 lane height");
            requireNear(pose->headingRadians, 0.0, "vehicle follows authored direction");
        }
        require(!advanceMapTrafficActor(drivingActor, 0.5),
                "vehicle retires only at the final unconnected end without reversing");
        MapTrafficLaneNetwork reversedLanes;
        firstLane.reversed = true;
        secondLane = firstLane;
        secondLane.spline.placement = makeSpline(42, 0, 0, -10.0);
        reversedLanes.addSegment(firstLane);
        reversedLanes.addSegment(secondLane);
        reversedLanes.finalize();
        const auto reverseDrivingRoute = reversedLanes.buildRoute(40);
        require(reverseDrivingRoute && reverseDrivingRoute->segments.size() == 2,
                "reverse-direction authored lanes join end to start");
        const auto reverseDrivingPose = sampleMapTrafficRoute(*reverseDrivingRoute, 12.0, 0.0);
        require(reverseDrivingPose.has_value(), "reverse-direction lane can be sampled");
        requireNear(reverseDrivingPose->position[0], 3.0,
                    "reverse lane keeps authored lateral offset rather than mirroring it");
        requireNear(reverseDrivingPose->position[1], -2.0,
                    "reverse lane crosses into the preceding spline");
        requireNear(std::abs(reverseDrivingPose->headingRadians), std::acos(-1.0),
                    "reverse lane vehicle heading follows authored travel");
        MapTrafficLaneNetwork disconnectedLanes;
        firstLane.reversed = false;
        secondLane = firstLane;
        secondLane.spline.placement = makeSpline(43, 40, 0, 10.6);
        firstLane.spline.placement.nextSplineId = 43;
        disconnectedLanes.addSegment(firstLane);
        disconnectedLanes.addSegment(secondLane);
        disconnectedLanes.finalize();
        const auto disconnectedRoute = disconnectedLanes.buildRoute(40);
        require(disconnectedRoute && disconnectedRoute->segments.size() == 1,
                "authored ID links alone do not bridge spatially disconnected lanes");
        const MapTrafficLaneNetworkStats disconnectedStats = disconnectedLanes.stats();
        require(disconnectedStats.lanes == 2 && disconnectedStats.deadEnds == 2 &&
                    disconnectedStats.nearMissDeadEnds == 1,
                "lane stats report open ends and gaps just beyond the join tolerance");
        MapTrafficLaneNetwork elevatedLanes;
        secondLane.spline.placement.localY = 10.0;
        secondLane.spline.placement.elevation += 3.0;
        elevatedLanes.addSegment(firstLane);
        elevatedLanes.addSegment(secondLane);
        elevatedLanes.finalize();
        const auto elevatedRoute = elevatedLanes.buildRoute(40);
        require(elevatedRoute && elevatedRoute->segments.size() == 1,
                "overlapping roads at different elevations are not connected");
        MapTrafficLaneNetwork cyclicLanes;
        MapTrafficRouteSegment approach;
        approach.spline.placement.splineId = 50;
        approach.points = {{0.0, -10.0, 0.0}, {0.0, 0.0, 0.0}};
        MapTrafficRouteSegment cycleA;
        cycleA.points = {{0.0, 0.0, 0.0}, {0.0, 10.0, 0.0}, {10.0, 10.0, 0.0}};
        MapTrafficRouteSegment cycleB;
        cycleB.points = {{10.0, 10.0, 0.0}, {10.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
        cyclicLanes.addSegment(approach);
        cyclicLanes.addSegment(cycleA);
        cyclicLanes.addSegment(cycleB);
        cyclicLanes.finalize();
        const auto cyclicRoute = cyclicLanes.buildRoute(50);
        require(cyclicRoute && cyclicRoute->loopStartDistance == 10.0 && !cyclicRoute->closed,
                "a route entering an authored loop retains its one-way approach");
        MapTrafficActor cyclicActor =
            makeMapTrafficActor(*cyclicRoute, "Vehicles\\regular.ovh", 2.0, 0.0, false);
        cyclicActor.distanceMeters = cyclicRoute->totalLength - 1.0;
        require(advanceMapTrafficActor(cyclicActor, 1.0),
                "an actor entering a real loop is not retired or reversed");
        requireNear(cyclicActor.distanceMeters, 11.0,
                    "an actor loops at the authored join rather than teleporting to its approach");
        MapTrafficLaneNetwork singleLoop;
        MapTrafficRouteSegment loopLane;
        loopLane.spline.placement.splineId = 51;
        loopLane.points = {{0.0, 0.0, 0.0},    {0.0, 10.0, 0.0},  {10.0, 10.0, 0.0},
                           {10.0, -10.0, 0.0}, {0.0, -10.0, 0.0}, {0.0, 0.0, 0.0}};
        singleLoop.addSegment(loopLane);
        singleLoop.finalize();
        const auto singleLoopRoute = singleLoop.buildRoute(51);
        require(singleLoopRoute && singleLoopRoute->closed && singleLoopRoute->segments.size() == 1,
                "a single authored closed path remains a loop without artificial reversal");
        MapTrafficLaneNetwork branchingLanes;
        MapTrafficRouteSegment turningLane;
        turningLane.points = {{3.0, 10.0, 2.1}, {13.0, 10.0, 2.1}};
        secondLane = firstLane;
        secondLane.spline.placement = makeSpline(44, 0, 0, 10.0);
        branchingLanes.addSegment(firstLane);
        branchingLanes.addSegment(turningLane);
        branchingLanes.addSegment(secondLane);
        branchingLanes.addSegment(wrongWay);
        branchingLanes.finalize();
        const auto straightRoute = branchingLanes.buildRoute(40);
        require(straightRoute && straightRoute->segments.size() == 2 &&
                    straightRoute->segments.back().spline.placement.splineId == 44,
                "branch selection prefers the forward lane, not a turn or a U-turn");
        require(branchingLanes.laneCount(40) == 2 && branchingLanes.laneCount(0) == 0,
                "authored lanes are indexed per spline, excluding scenery junction paths");
        const auto oppositeRoute = branchingLanes.buildRoute(40, 1);
        require(oppositeRoute && oppositeRoute->segments.front().reversed &&
                    oppositeRoute->segments.size() == 1,
                "a lane selector spawns on the opposite-direction lane of the same spline");
        const auto wrappedSelector = branchingLanes.buildRoute(40, 2);
        require(wrappedSelector && !wrappedSelector->segments.front().reversed,
                "lane selectors wrap across the spline's authored lanes");
        require(!branchingLanes.buildRoute(999), "unknown splines have no driving lane");
        MapTrafficLaneNetwork timetableLanes;
        MapTrafficRouteSegment timetableApproach;
        timetableApproach.spline.placement.splineId = 60;
        timetableApproach.points = {{0.0, -10.0, 0.0}, {0.0, 0.0, 0.0}};
        MapTrafficRouteSegment timetableStraight;
        timetableStraight.spline.placement.splineId = 61;
        timetableStraight.points = {{0.0, 0.0, 0.0}, {0.0, 10.0, 0.0}};
        MapTrafficRouteSegment timetableTurn;
        timetableTurn.spline.placement.splineId = 62;
        timetableTurn.points = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
        timetableLanes.addSegment(timetableApproach);
        timetableLanes.addSegment(timetableStraight);
        timetableLanes.addSegment(timetableTurn);
        timetableLanes.finalize();
        const auto timetableGuidedRoute = timetableLanes.buildRoute(60, 0, 0, 0.0, {}, {60, 62});
        require(timetableGuidedRoute && timetableGuidedRoute->segments.size() == 2 &&
                    timetableGuidedRoute->segments.back().spline.placement.splineId == 62,
                "a timetable spline sequence guides route choice through a non-default turn");
        require(!timetableLanes.buildRoute(60, 0, 0, 0.0, {}, {60, 63}),
                "a timetable-guided route rejects a spline sequence absent from the lane network");
        MapTrafficActor oneWayActor =
            makeMapTrafficActor(*route, "Vehicles\\one_way.bus", 2.0, 0.0, false);
        oneWayActor.distanceMeters = 19.5;
        require(!advanceMapTrafficActor(oneWayActor, 0.5) &&
                    oneWayActor.distanceMeters == route->totalLength,
                "an open AI route completes at its endpoint instead of reversing");
        const auto acrossJoin = sampleMapTrafficRoute(*route, 12.0, 0.0);
        require(acrossJoin.has_value() && acrossJoin->splineId == 11,
                "sampling crosses into the next linked spline");
        requireNear(acrossJoin->position[0], 305.0, "tile X origin is applied");
        requireNear(acrossJoin->position[1], 618.0, "tile Y origin is applied");
        requireNear(acrossJoin->position[2], 2.0, "spline elevation is sampled");
        requireNear(mapTrafficLaneOffsetMeters(false, 1.5), 1.5,
                    "right-hand traffic uses the right side of travel");
        requireNear(mapTrafficLaneOffsetMeters(true, 1.5), -1.5,
                    "left-hand traffic uses the left side of travel");
        requireNear(mapTrafficHeadingToVehicleYawDegrees(0.0), 90.0,
                    "northbound spline heading aligns the vehicle's local forward axis");
        requireNear(mapTrafficHeadingToVehicleYawDegrees(std::acos(-1.0) / 2.0), 0.0,
                    "eastbound spline heading maps to zero vehicle yaw");
        requireNear(mapTrafficHourAtElapsedSeconds(12.0, 0.0), 12.0,
                    "map traffic clock starts at the configured hour");
        requireNear(mapTrafficHourAtElapsedSeconds(12.0, 7200.0), 14.0,
                    "map traffic clock advances with simulation time");
        requireNear(mapTrafficHourAtElapsedSeconds(23.5, 3600.0), 0.5,
                    "map traffic clock wraps across midnight");
        requireNear(mapTrafficHourAtElapsedSeconds(12.0, 86400.0), 12.0,
                    "map traffic clock wraps after a full day");
        requireNear(mapTrafficNextDepartureElapsedSeconds(6.0, 0.0, 405.0), 2700.0,
                    "scheduled departure is measured from the configured start hour");
        requireNear(mapTrafficNextDepartureElapsedSeconds(23.5, 0.0, 30.0), 3600.0,
                    "a departure after midnight is scheduled on the next clock day");
        requireNear(mapTrafficNextDepartureElapsedSeconds(6.75, 0.0, 405.0), 0.0,
                    "departure at the configured start hour is immediately due");
        requireNear(mapTrafficNextDepartureElapsedSeconds(6.75, 1.0, 405.0), 86400.0,
                    "a missed departure rolls to the following day");
        require(!mapTrafficScheduledDepartureDue(2699.0, 2700.0),
                "a queued bus is held before its departure time");
        require(mapTrafficScheduledDepartureDue(2700.0, 2700.0) &&
                    mapTrafficScheduledDepartureDue(2701.0, 2700.0),
                "a queued bus becomes eligible at and after its departure time");
        const auto departurePartitions =
            mapTrafficPartitionScheduledDepartures({360.0, 370.0, 390.0, 410.0, 440.0}, 2);
        require(departurePartitions.size() == 2 &&
                    departurePartitions[0] == std::vector<double>({360.0, 390.0, 440.0}) &&
                    departurePartitions[1] == std::vector<double>({370.0, 410.0}),
                "overlapping bus assignments partition departures without duplication");
        MapTrafficActor createdActor = makeMapTrafficActor(
            *route, "Vehicles\\traffic.bus", 8.0, 1.5, true, std::string("Humans\\driver.hum"));
        require(createdActor.modelPath == "Vehicles\\traffic.bus" &&
                    createdActor.driverModelPath == "Humans\\driver.hum" &&
                    createdActor.speedMetersPerSecond == 8.0 &&
                    createdActor.lateralOffsetMeters == -1.5,
                "actor factory retains assets, movement speed, and map-side lane offset");
        const auto roadSpawnCenter = sampleMapTrafficRoute(createdActor.route, 7.0, -1.5);
        require(roadSpawnCenter.has_value() &&
                    placeMapTrafficActorNear(createdActor, 7.0, roadSpawnCenter->position[0],
                                             roadSpawnCenter->position[1], 0.0, 1.0),
                "AI vehicle spawns on a road route at its requested map position");
        ++verifiedAiSpawns;
        requireNear(createdActor.distanceMeters, 7.0,
                    "AI vehicle spawn retains the initialized route distance");
        MapTrafficActor repeatedRoadSpawn =
            makeMapTrafficActor(*route, "Vehicles\\open_traffic.bus", 8.0, 0.0, false);
        const auto openRoadSpawnCenter = sampleMapTrafficRoute(*route, 7.0, 0.0);
        require(openRoadSpawnCenter.has_value(), "open route spawn center can be sampled");
        require(placeMapTrafficActorNearLooping(repeatedRoadSpawn, 42.0,
                                                openRoadSpawnCenter->position[0],
                                                openRoadSpawnCenter->position[1], 5.0, 1.0),
                "AI vehicle population can place repeated actors on an open route");
        ++verifiedAiSpawns;
        requireNear(repeatedRoadSpawn.distanceMeters, 4.0,
                    "looping AI spawn wraps before the open route endpoint");
        require(repeatedRoadSpawn.distanceMeters != createdActor.distanceMeters,
                "repeated AI spawns do not pile up at the open route endpoint");
        require(advanceMapTrafficActorLooping(createdActor, 1.0),
                "spawned AI vehicle remains active while traversing its route");
        const auto movedRoadPose = sampleMapTrafficRoute(
            createdActor.route, createdActor.distanceMeters, createdActor.lateralOffsetMeters);
        require(movedRoadPose.has_value(),
                "spawned AI vehicle advances across its continuous road route");
        const std::vector<MapTrafficActor> aiPopulation =
            spawnPopulation(network, network.routeStartSplineIds(), "Vehicles\\traffic.bus", 8.0,
                            roadSpawnCenter->position, 4, 5.0, 20.0, 1.0);
        verifiedAiPopulationSpawns = aiPopulation.size();
        require(verifiedAiPopulationSpawns == 4,
                "bounded route candidate selection fills the requested AI population");

        MapTrafficRouteNetwork humanNetwork;
        humanNetwork.addTileSplines({makeSpline(20, 0, 0, 6.0)}, 1, 2);
        humanNetwork.finalize();
        const auto humanRoute = humanNetwork.buildRoute(20);
        require(humanRoute.has_value() && humanRoute->segments.size() == 1,
                "human-path network builds a route independently from road traffic");
        MapTrafficActor humanActor =
            makeMapTrafficActor(*humanRoute, "Humans\\pedestrian.hum", 1.2, 0.0, false);
        const auto humanSpawnCenter = sampleMapTrafficRoute(humanActor.route, 2.0, 0.0);
        require(humanSpawnCenter.has_value() &&
                    placeMapTrafficActorNear(humanActor, 2.0, humanSpawnCenter->position[0],
                                             humanSpawnCenter->position[1], 0.0, 0.01),
                "human actor spawns on its configured human path");
        ++verifiedHumanSpawns;
        requireNear(humanActor.distanceMeters, 2.0,
                    "human spawn retains its initial route distance");
        MapTrafficActor repeatedHumanSpawn = humanActor;
        require(placeMapTrafficActorNearLooping(repeatedHumanSpawn, 12.0,
                                                humanSpawnCenter->position[0],
                                                humanSpawnCenter->position[1], 12.0, 0.01),
                "human population can place repeated actors on an open route");
        ++verifiedHumanSpawns;
        requireNear(repeatedHumanSpawn.distanceMeters, 2.01,
                    "human spawn wraps into its endpoint-safe route range");
        require(placeMapTrafficActorNearLooping(repeatedHumanSpawn, 22.0,
                                                humanSpawnCenter->position[0],
                                                humanSpawnCenter->position[1], 12.0, 0.01),
                "human spawn cycles route distance after passing its endpoint");
        requireNear(repeatedHumanSpawn.distanceMeters, 2.02,
                    "looping human spawn wraps before the open route endpoint");
        require(advanceMapTrafficActorLooping(humanActor, 1.0),
                "spawned human actor remains active during route traversal");
        const auto movedHumanPose = sampleMapTrafficRoute(
            humanActor.route, humanActor.distanceMeters, humanActor.lateralOffsetMeters);
        require(movedHumanPose.has_value() && movedHumanPose->splineId == 20,
                "spawned human actor advances on a human route");
        const std::vector<MapTrafficActor> humanPopulation =
            spawnPopulation(humanNetwork, humanNetwork.splineIds(), "Humans\\pedestrian.hum", 1.2,
                            humanSpawnCenter->position, 4, 2.5, 6.0, 0.01);
        verifiedHumanPopulationSpawns = humanPopulation.size();
        require(verifiedHumanPopulationSpawns == 4,
                "bounded route candidate selection fills the requested human population");
        const double previousHumanDistance = humanActor.distanceMeters;
        require(!placeMapTrafficActorNear(humanActor, 4.0, 10000.0, 10000.0, 1.0, 0.01),
                "actors outside the configured spawn radius are rejected");
        requireNear(humanActor.distanceMeters, previousHumanDistance,
                    "rejected spawn leaves existing actor state unchanged");
        MapTrafficActor endClampedHuman = humanActor;
        require(placeMapTrafficActorNear(endClampedHuman, 10.0, humanSpawnCenter->position[0],
                                         humanSpawnCenter->position[1], 100.0, 0.01),
                "human actor spawn accepts a route endpoint margin");
        requireNear(endClampedHuman.distanceMeters, 9.99,
                    "human actor spawn remains just before an open route endpoint");
        MapTrafficActor unusableShortRoute = humanActor;
        unusableShortRoute.distanceMeters = 3.0;
        require(!placeMapTrafficActorNearLooping(
                    unusableShortRoute, 4.0, humanSpawnCenter->position[0],
                    humanSpawnCenter->position[1], 100.0, unusableShortRoute.route.totalLength),
                "open routes fully consumed by endpoint margins are rejected");
        requireNear(unusableShortRoute.distanceMeters, 3.0,
                    "rejected short-route spawn leaves actor state unchanged");
        const double previousSpawnDistance = humanActor.distanceMeters;
        bool invalidSpawnRejected = false;
        try {
            static_cast<void>(placeMapTrafficActorNear(
                humanActor, std::numeric_limits<double>::quiet_NaN(), humanSpawnCenter->position[0],
                humanSpawnCenter->position[1], 1.0));
        } catch (const std::invalid_argument&) {
            invalidSpawnRejected = true;
        }
        require(invalidSpawnRejected && humanActor.distanceMeters == previousSpawnDistance,
                "invalid spawn values are rejected without changing actor state");

        MapTrafficRouteNetwork reverseNetwork;
        reverseNetwork.addTileSplines({makeSpline(30, 0, 31, 0.0)}, 0, 0);
        reverseNetwork.addTileSplines({makeSpline(31, 0, 30, 10.0)}, 0, 0);
        reverseNetwork.finalize();
        const auto reverseRoute = reverseNetwork.buildRoute(30);
        require(reverseRoute.has_value() && reverseRoute->segments.size() == 2 &&
                    reverseRoute->segments[1].reversed,
                "route records reversed traversal of a reciprocal spline");
        const auto reversePose =
            sampleMapTrafficRoute(*reverseRoute, 12.0, mapTrafficLaneOffsetMeters(false, 1.0));
        require(reversePose.has_value(), "reverse-traversed actor pose is sampled");
        requireNear(reversePose->headingRadians, std::acos(-1.0),
                    "actor heading follows reversed spline traversal");
        requireNear(reversePose->position[0], 4.0,
                    "positive lane offset remains right of reversed travel");
        const auto leftHandReversePose =
            sampleMapTrafficRoute(*reverseRoute, 12.0, mapTrafficLaneOffsetMeters(true, 1.0));
        require(leftHandReversePose.has_value(), "left-hand reverse actor pose is sampled");
        requireNear(leftHandReversePose->position[0], 6.0,
                    "left-hand lane offset uses the opposite side of reversed travel");

        MapTrafficActor actor;
        actor.route = *route;
        actor.distanceMeters = 9.0;
        actor.speedMetersPerSecond = 4.0;
        require(advanceMapTrafficActor(actor, 1.0), "actor remains on an open route");
        requireNear(actor.distanceMeters, 13.0, "actor advances by speed times elapsed time");
        const auto advancedPose =
            sampleMapTrafficRoute(actor.route, actor.distanceMeters, actor.lateralOffsetMeters);
        require(advancedPose.has_value() && advancedPose->splineId == 11,
                "advanced actor pose remains on the linked route");
        requireNear(advancedPose->position[1], 619.0, "advanced actor position follows the spline");
        MapTrafficActor endingActor = actor;
        endingActor.distanceMeters = 18.0;
        endingActor.speedMetersPerSecond = 1.5;
        std::vector<MapTrafficActor> population{actor, endingActor};
        require(advanceMapTrafficActors(population, 0.5) == 0 && population.size() == 2,
                "population update advances actors without removing those still on route");
        requireNear(population.front().distanceMeters, 15.0,
                    "population update advances each actor by elapsed time");
        require(advanceMapTrafficActors(population, 1.0) == 1 && population.size() == 1,
                "population update removes actors that reach open-route endpoints");
        requireNear(population.front().distanceMeters, 19.0,
                    "remaining population actor retains its independently advanced position");
        const double positionBeforeInvalidUpdate = population.front().distanceMeters;
        bool invalidPopulationElapsedRejected = false;
        try {
            static_cast<void>(advanceMapTrafficActors(population, -0.1));
        } catch (const std::invalid_argument&) {
            invalidPopulationElapsedRejected = true;
        }
        require(invalidPopulationElapsedRejected &&
                    population.front().distanceMeters == positionBeforeInvalidUpdate,
                "invalid population time is rejected without changing actor state");
        MapTrafficActor invalidPopulationActor = actor;
        invalidPopulationActor.speedMetersPerSecond = -1.0;
        std::vector<MapTrafficActor> invalidPopulation{actor, invalidPopulationActor};
        const double firstActorDistance = invalidPopulation.front().distanceMeters;
        bool invalidActorRejected = false;
        try {
            static_cast<void>(advanceMapTrafficActors(invalidPopulation, 0.5));
        } catch (const std::invalid_argument&) {
            invalidActorRejected = true;
        }
        require(invalidActorRejected && invalidPopulation.size() == 2 &&
                    invalidPopulation.front().distanceMeters == firstActorDistance,
                "invalid actor state rejects population update without partial advancement");
        actor.distanceMeters = 19.0;
        actor.speedMetersPerSecond = 3.0;
        require(!advanceMapTrafficActor(actor, 1.0),
                "actor reports leaving an open route at its endpoint");
        requireNear(actor.distanceMeters, 20.0, "open route clamps at its endpoint");
        actor.route = *route;
        actor.distanceMeters = 18.0;
        actor.speedMetersPerSecond = 5.0;
        require(advanceMapTrafficActorLooping(actor, 1.0),
                "looping actor remains active after reaching an open-route endpoint");
        requireNear(actor.distanceMeters, 3.0,
                    "generic open-route looping retains its existing wrap behavior");
        bool invalidElapsedRejected = false;
        try {
            static_cast<void>(advanceMapTrafficActor(actor, -0.1));
        } catch (const std::invalid_argument&) {
            invalidElapsedRejected = true;
        }
        require(invalidElapsedRejected, "negative actor update time is rejected");

        MapSplinePlacement first = makeSpline(20, 21, 21, 0.0);
        MapSplinePlacement second = makeSpline(21, 20, 20, 10.0);
        std::unordered_map<int, MapSplinePlacementReference> loopSplines{{20, {first, 0, 0}},
                                                                         {21, {second, 0, 0}}};
        const MapSplineChainResolver loopResolver =
            [&loopSplines](int id) -> std::optional<MapSplinePlacementReference> {
            const auto found = loopSplines.find(id);
            return found == loopSplines.end()
                       ? std::nullopt
                       : std::optional<MapSplinePlacementReference>(found->second);
        };
        const auto loop = buildMapTrafficRoute(loopSplines.at(20), loopResolver);
        require(loop.has_value() && loop->closed && loop->segments.size() == 2,
                "reciprocal cycle closes without duplicating its first segment");
        MapTrafficRouteNetwork closedNetwork;
        closedNetwork.addTileSplines({first, second}, 0, 0);
        closedNetwork.finalize();
        require(closedNetwork.routeStartSplineIds() == std::vector<int>({20}),
                "closed routes select one stable spline ID instead of every segment");
        const auto closedEndpoints = closedNetwork.endpointStatus(20);
        require(closedEndpoints.has_value() && closedEndpoints->startConnected &&
                    closedEndpoints->endConnected,
                "closed routes have no unjoined endpoint markers");
        actor.route = *loop;
        actor.distanceMeters = 18.0;
        actor.speedMetersPerSecond = 5.0;
        require(advanceMapTrafficActor(actor, 1.0), "actor remains on a closed route");
        requireNear(actor.distanceMeters, 3.0, "closed route wraps actor distance");
        const auto closedSpawnCenter = sampleMapTrafficRoute(*loop, 2.0, 0.0);
        MapTrafficActor closedSpawnActor =
            makeMapTrafficActor(*loop, "Humans\\closed_path.hum", 1.2, 0.0, false);
        require(closedSpawnCenter.has_value() &&
                    placeMapTrafficActorNearLooping(closedSpawnActor, 22.0,
                                                    closedSpawnCenter->position[0],
                                                    closedSpawnCenter->position[1], 0.0, 1.0),
                "actors spawn near the requested center on closed routes");
        requireNear(closedSpawnActor.distanceMeters, 2.0,
                    "closed route spawn distances wrap by total route length");

        MapTrafficRouteNetwork ambiguousNetwork;
        ambiguousNetwork.addTileSplines({makeSpline(10, 0, 11, 6.0)}, 1, 2);
        ambiguousNetwork.addTileSplines({makeSpline(11, 10, 0, 16.0)}, 1, 2);
        ambiguousNetwork.addTileSplines({makeSpline(11, 10, 0, 16.0)}, 1, 3);
        ambiguousNetwork.finalize();
        require(!ambiguousNetwork.buildRoute(11),
                "ambiguous spline IDs cannot be selected as route starts");
        const auto truncatedRoute = ambiguousNetwork.buildRoute(10);
        require(truncatedRoute.has_value() && truncatedRoute->segments.size() == 1,
                "ambiguous linked IDs terminate a route safely");
        const auto ambiguousEndpoints = ambiguousNetwork.endpointStatus(10);
        require(ambiguousEndpoints.has_value() && !ambiguousEndpoints->endConnected,
                "an ambiguous linked spline is exposed as an unjoined endpoint");
        std::cout << "MAP_TRAFFIC_SPAWN_PROBE=PASS AI_ROUTE_SPAWNS=" << verifiedAiSpawns
                  << " HUMAN_ROUTE_SPAWNS=" << verifiedHumanSpawns
                  << " AI_POPULATION_SPAWNS=" << verifiedAiPopulationSpawns
                  << " HUMAN_POPULATION_SPAWNS=" << verifiedHumanPopulationSpawns << '\n';
    } catch (const std::exception& error) {
        std::cerr << "map traffic runtime probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
