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
    const auto footprint = [](const MapTrafficActor& actor) {
        return MapTrafficObstacle{
            *sampleMapTrafficRoute(actor.route, actor.distanceMeters, actor.lateralOffsetMeters),
            actor.lengthMeters, actor.widthMeters};
    };
    MapTrafficSignalSchedule cycleSignal;
    cycleSignal.cycleDurationSeconds = 10.0;
    cycleSignal.phases = {{0, 3.0}, {6, 5.0}, {9, 2.0}};
    require(mapTrafficSignalStateAtTime(cycleSignal, 0.0) == 0 &&
                mapTrafficSignalStateAtTime(cycleSignal, 3.0) == 6 &&
                mapTrafficSignalStateAtTime(cycleSignal, 8.0) == 9 &&
                mapTrafficSignalStateAtTime(cycleSignal, 10.0) == 0,
            "signal phase schedule wraps at the configured cycle duration");
    require(!mapTrafficSignalStateAtTime(
                 MapTrafficSignalSchedule{9.0, {{0, 3.0}, {6, 5.0}}, false}, 0.0)
                 .has_value(),
            "signal schedules whose phases do not cover the cycle are rejected");

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
        junction.addSegment(straight(radius < 0 ? 1003 : 1004, radius, 10.0,
                                    radius < 0 ? -90.0 : 90.0, 60.0));
    }
    junction.finalize();
    require(junction.entryLanes() == std::vector<MapTrafficLaneStart>{{1001, 0}},
            "only an unconnected incoming lane is an entry, not internal or exit splines");
    std::optional<MapTrafficRoute> left;
    std::optional<MapTrafficRoute> right;
    bool foundStraight = false;
    for (std::size_t seed = 1; seed <= 12; ++seed) {
        const auto route = junction.buildRoute(1001, 0, seed, 4.5);
        require(route.has_value(), "a normal car can route through an authored junction");
        const int exit = route->segments.back().spline.placement.splineId;
        if (exit == 1003) left = route;
        if (exit == 1004) right = route;
        if (exit == 1002) foundStraight = true;
        const auto largeVehicle = junction.buildRoute(1001, 0, seed, 20.0);
        require(largeVehicle && largeVehicle->segments.back().spline.placement.splineId == 1002,
                "vehicles with a 20 m minimum radius reject the 10 m junction turns");
        const auto exactRadius = junction.buildRoute(1001, 0, seed, 10.0);
        require(exactRadius &&
                    exactRadius->segments.back().spline.placement.splineId == exit,
                "a vehicle with a 10 m minimum radius can use authored 10 m turns");
    }
    require(left && right && foundStraight,
            "seeded junction choices include left, right, and straight movements");
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
        for (const auto& sample : tessellateMapSpline(
                 0.0, startY, 0.0, radius * std::acos(-1.0) * 0.5, radius)) {
            turn.points.push_back({sample.x, sample.y, 0.0});
        }
        radiusBlocked.addSegment(turn);
        radiusBlocked.addSegment(straight(radius == 5.0 ? 1502 : 1503, radius,
                                         startY + radius, 90.0, 60.0));
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
    for (int frame = 0; frame < 300; ++frame) {
        advanceMapTrafficDriving({&stoppedFollower}, {playerBus}, 0.1);
        require(mapTrafficHasClearance(footprint(stoppedFollower), {playerBus}, 0.49),
                "a follower never overlaps a stopped player bus");
    }
    require(stoppedFollower.speedMetersPerSecond < 0.01 &&
                stoppedFollower.distanceMeters < 51.0 && stoppedFollower.distanceMeters > 40.0,
            "a car brakes to rest behind the bus with a standstill gap");
    requireNear(playerBus.pose.position[1], 60.0,
                "traffic avoidance never changes the player bus state");
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
    auto pedestrianWalker =
        makeMapTrafficActor(road, "walker.hum", 1.2, 0.0, false);
    const MapTrafficObstacle parkedVehicle{{{0.0, 5.0, 0.0}, 0.0, 0}, 4.5, 1.8};
    require(!advanceMapTrafficActorLoopingAvoidingObstacles(
                pedestrianWalker, {parkedVehicle}, 0.5, 0.8, 3.0) &&
                pedestrianWalker.distanceMeters > 0.0 &&
                pedestrianWalker.distanceMeters < 2.6,
            "pedestrians stop before an occupied vehicle footprint");
    require(advanceMapTrafficActorLoopingAvoidingObstacles(
                pedestrianWalker, {}, 0.5, 0.8, 1.0) &&
                pedestrianWalker.distanceMeters > 2.4,
            "pedestrians resume along their route after the vehicle clears");
    auto adjacent = makeMapTrafficActor(road, "car.ovh", 10.0, 0.0, false);
    const MapTrafficObstacle oncoming{{{3.8, 10.0, 0.0}, std::acos(-1.0), 0}, 12.0, 2.5};
    advanceMapTrafficDriving({&adjacent}, {oncoming}, 1.0);
    requireNear(adjacent.speedMetersPerSecond, 10.0,
                "an oncoming bus in the adjacent lane does not falsely block this lane");
    auto north = makeMapTrafficActor(
        oneSegment(straight(1200, 0.0, -40.0, 0.0, 100.0)), "north.ovh", 8.0, 0.0, false);
    auto east = makeMapTrafficActor(
        oneSegment(straight(1201, -40.0, 0.0, 90.0, 100.0)), "east.ovh", 8.0, 0.0, false);
    bool crossingYieldObserved = false;
    for (int frame = 0; frame < 250; ++frame) {
        advanceMapTrafficDriving({&north, &east}, {}, 0.1);
        crossingYieldObserved = crossingYieldObserved ||
                                north.junctionWaitSeconds > 0.0 ||
                                east.junctionWaitSeconds > 0.0;
        require(mapTrafficHasClearance(footprint(north), {footprint(east)}, 0.49),
                "conflicting junction movements remain separated");
    }
    require(crossingYieldObserved && north.distanceMeters > 60.0 && east.distanceMeters > 60.0,
            "waiting vehicles gain junction priority and both conflicting movements traverse");
    auto entryActor = makeMapTrafficActor(road, "entry.ovh", 12.0, 0.0, false);
    require(!canSpawnMapTrafficEntry(entryActor, false, {}),
            "vehicles do not materialize at a visible endpoint");
    require(!canSpawnMapTrafficEntry(entryActor, true, {footprint(entryActor)}),
            "occupied endpoint entries queue rather than overlapping spawns");
    require(canSpawnMapTrafficEntry(entryActor, true, {}),
            "an off-screen, clear endpoint admits a spawn at distance zero");
    requireNear(entryActor.distanceMeters, 0.0, "entry admission does not offset actors into a route");
    require(mapTrafficRoutesShareCompleteOrderedPath({1, 2, 3, 4}, {2, 3}),
            "a complete timetable path can be a contiguous part of a longer endpoint route");
    require(mapTrafficRoutesShareCompleteOrderedPath({2, 3}, {1, 2, 3, 4}),
            "a complete endpoint route can be a contiguous part of a longer timetable path");
    require(!mapTrafficRoutesShareCompleteOrderedPath({1, 2, 3}, {2, 4}),
            "route matching rejects partial or out-of-order spline overlaps");
    require(!mapTrafficRoutesShareCompleteOrderedPath({1}, {1, 2}),
            "route matching requires at least two ordered spline IDs");
    entryActor.distanceMeters = entryActor.route.totalLength;
    require(!mapTrafficRouteEndReadyForUnload(entryActor, false),
            "vehicles wait at visible unconnected route ends instead of unloading");
    require(mapTrafficRouteEndReadyForUnload(entryActor, true),
            "vehicles unload after their unconnected route end becomes hidden");
    entryActor.distanceMeters = 0.0;
    require(!mapTrafficRouteEndReadyForUnload(entryActor, true),
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
        require(!mapTrafficEntryHiddenFromCamera({0.0, 100.0, 0.0}, 3.0, camera, viewProjection,
                                                 50.0),
                "an entry in front of the camera is visible, so nothing spawns there");
        require(mapTrafficEntryHiddenFromCamera({0.0, -100.0, 0.0}, 3.0, camera, viewProjection,
                                                50.0),
                "an entry behind the camera is hidden");
        require(mapTrafficEntryHiddenFromCamera({200.0, 100.0, 0.0}, 3.0, camera, viewProjection,
                                                50.0),
                "an entry beside the view frustum is hidden");
        require(!mapTrafficEntryHiddenFromCamera({103.0, 100.0, 0.0}, 5.0, camera,
                                                 viewProjection, 50.0),
                "a vehicle footprint straddling the screen edge counts as visible");
        require(!mapTrafficEntryHiddenFromCamera({0.0, -30.0, 0.0}, 3.0, camera, viewProjection,
                                                 50.0),
                "entries near the camera never admit spawns even when behind it");
        require(mapTrafficEntryHiddenFromCamera({0.0, 1500.0, 0.0}, 3.0, camera, viewProjection,
                                                50.0),
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
    for (int frame = 0; frame < 300; ++frame) {
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
    auto signalStartActor = makeMapTrafficActor(
        signalStartRoute, "car.ovh", 8.0, 0.0, false);
    advanceMapTrafficDriving({&signalStartActor}, {}, 1.0);
    require(signalStartActor.distanceMeters == 0.0 &&
                signalStartActor.speedMetersPerSecond < 1.0e-6,
            "AI traffic starting on a signalized path waits at route entry on red");
    advanceMapTrafficDriving({&signalStartActor}, {}, 1.0, 100.0);
    require(signalStartActor.distanceMeters > 0.0,
            "AI traffic starting on a signalized path proceeds on green");

    MapTrafficRoute loopSignalRoute = *circleRoute;
    loopSignalRoute.segments.front().trafficSignal = redThenGreen;
    auto loopSignalActor = makeMapTrafficActor(
        loopSignalRoute, "car.ovh", 8.0, 0.0, false);
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

} // namespace

int main() {
    std::size_t verifiedAiSpawns = 0;
    std::size_t verifiedHumanSpawns = 0;
    std::size_t verifiedAiPopulationSpawns = 0;
    std::size_t verifiedHumanPopulationSpawns = 0;
    try {
        using namespace openbus::map;
        verifyDrivingBehavior();
        const auto spawnPopulation =
            [](const MapTrafficRouteNetwork& routeNetwork, const std::vector<int>& splineIds,
               const std::string& modelPath, double speedMetersPerSecond,
               const std::array<double, 3>& spawnCenter, std::size_t targetCount,
               double spacingMeters, double maximumDistanceMeters,
               double endpointMarginMeters) {
                std::vector<MapTrafficActor> actors;
                for (std::size_t candidate = 0;
                     candidate < targetCount * 4 && actors.size() < targetCount; ++candidate) {
                    if (splineIds.empty()) {
                        break;
                    }
                    const auto route = routeNetwork.buildRoute(
                        splineIds[candidate % splineIds.size()]);
                    if (!route) {
                        continue;
                    }
                    MapTrafficActor actor =
                        makeMapTrafficActor(*route, modelPath, speedMetersPerSecond, 0.0, false);
                    if (placeMapTrafficActorNearLooping(
                            actor, static_cast<double>(candidate) * spacingMeters,
                            spawnCenter[0], spawnCenter[1], maximumDistanceMeters,
                            endpointMarginMeters)) {
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
        require(std::none_of(route->segments.begin(), route->segments.end(),
                             [](const MapTrafficRouteSegment& segment) {
                                 return segment.reversed;
                             }),
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
            const auto pose = sampleMapTrafficRoute(
                drivingActor.route, drivingActor.distanceMeters, 0.0);
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
        require(cyclicRoute && cyclicRoute->loopStartDistance == 10.0 &&
                    !cyclicRoute->closed,
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
        loopLane.points = {{0.0, 0.0, 0.0}, {0.0, 10.0, 0.0}, {10.0, 10.0, 0.0},
                           {10.0, -10.0, 0.0}, {0.0, -10.0, 0.0}, {0.0, 0.0, 0.0}};
        singleLoop.addSegment(loopLane);
        singleLoop.finalize();
        const auto singleLoopRoute = singleLoop.buildRoute(51);
        require(singleLoopRoute && singleLoopRoute->closed &&
                    singleLoopRoute->segments.size() == 1,
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
            *route, "Vehicles\\traffic.bus", 8.0, 1.5, true,
            std::string("Humans\\driver.hum"));
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
        require(placeMapTrafficActorNearLooping(
                    repeatedRoadSpawn, 42.0, openRoadSpawnCenter->position[0],
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
        const std::vector<MapTrafficActor> aiPopulation = spawnPopulation(
            network, network.routeStartSplineIds(), "Vehicles\\traffic.bus", 8.0,
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
        require(placeMapTrafficActorNearLooping(
                    repeatedHumanSpawn, 12.0, humanSpawnCenter->position[0],
                    humanSpawnCenter->position[1], 12.0, 0.01),
                "human population can place repeated actors on an open route");
        ++verifiedHumanSpawns;
        requireNear(repeatedHumanSpawn.distanceMeters, 2.01,
                    "human spawn wraps into its endpoint-safe route range");
        require(placeMapTrafficActorNearLooping(
                    repeatedHumanSpawn, 22.0, humanSpawnCenter->position[0],
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
        const std::vector<MapTrafficActor> humanPopulation = spawnPopulation(
            humanNetwork, humanNetwork.splineIds(), "Humans\\pedestrian.hum", 1.2,
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
        std::unordered_map<int, MapSplinePlacementReference> loopSplines{
            {20, {first, 0, 0}}, {21, {second, 0, 0}}};
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
                    placeMapTrafficActorNearLooping(
                        closedSpawnActor, 22.0, closedSpawnCenter->position[0],
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
