#include "map/MapTrafficRuntime.h"
#include "map/MapSplineGeometry.h"
#include "app/traffic/ScheduledBusState.h"
#include "app/traffic/PlayerTripPolicy.h"
#include "app/traffic/PassengerNetwork.h"
#include "app/traffic/PassengerApproach.h"
#include "app/traffic/PassengerCabinPath.h"
#include "app/traffic/ScheduledBusTiming.h"
#include "app/traffic/PlayerTimetableController.h"
#include "app/vehicle/PassengerState.h"
#include "app/vehicle/PassengerRequestVariables.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string_view>
#include <tuple>
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

void verifyRearAxleTrafficPose() {
    using namespace openbus::map;
    for (const double direction : {-1.0, 1.0}) {
        MapTrafficRouteSegment approach{{makeSpline(3200, 0, 0, -40.0), 0, 0}};
        approach.spline.placement.localX = 0.0;
        approach.spline.placement.elevation = 0.0;
        approach.spline.placement.length = 40.0;
        MapTrafficRouteSegment turn;
        constexpr double radius = 8.0;
        for (int step = 0; step <= 64; ++step) {
            const double angle = std::numbers::pi * 0.5 * step / 64.0;
            turn.points.push_back({direction * radius * (1.0 - std::cos(angle)),
                                   radius * std::sin(angle), 0.0});
        }
        double turnLength = 0.0;
        for (std::size_t index = 1; index < turn.points.size(); ++index) {
            turnLength += std::hypot(turn.points[index][0] - turn.points[index - 1][0],
                                     turn.points[index][1] - turn.points[index - 1][1]);
        }
        MapTrafficRouteSegment exit{{makeSpline(3201, 0, 0, radius), 0, 0}};
        exit.spline.placement.localX = direction * radius;
        exit.spline.placement.elevation = 0.0;
        exit.spline.placement.rotationDegrees = direction * 90.0;
        exit.spline.placement.length = 40.0;
        MapTrafficRoute route;
        route.segments = {approach, turn, exit};
        route.cumulativeEndDistances = {40.0, 40.0 + turnLength, 80.0 + turnLength};
        route.totalLength = route.cumulativeEndDistances.back();
        auto actor = makeMapTrafficActor(route, "rear-axle.ovh", 4.0, 0.0, false);
        actor.lengthMeters = 12.0;
        actor.widthMeters = 2.4;
        actor.rearAxleOffsetMeters = -3.0;
        for (double distance : {0.0, 20.0, 40.0, 44.0, 48.0, 53.0, 60.0}) {
            const auto body = sampleMapTrafficActorPose(actor, distance);
            require(body.has_value(), "rear-axle vehicle pose can be sampled");
            if (distance >= 3.0) {
                const auto axle = sampleMapTrafficRoute(route, distance - 3.0, 0.0);
                require(axle.has_value(), "rear axle route pose can be sampled");
                requireNear(body->position[0] - 3.0 * std::sin(body->headingRadians),
                            axle->position[0], "rear axle follows the route horizontally");
                requireNear(body->position[1] - 3.0 * std::cos(body->headingRadians),
                            axle->position[1], "rear axle follows the route longitudinally");
                requireNear(body->headingRadians,
                            mapTrafficRouteTravelHeadingRadians(route, distance - 3.0, 0.0),
                            "body heading follows the rear axle tangent");
            } else {
                const auto origin = sampleMapTrafficRoute(route, distance, 0.0);
                require(origin.has_value(), "route origin can be sampled");
                requireNear(body->position[0], origin->position[0],
                            "rear axle extrapolates before an open route");
                requireNear(body->position[1], origin->position[1],
                            "open-route entry retains the vehicle origin");
            }
        }
        const auto body = sampleMapTrafficActorPose(actor, 48.0);
        const auto centerline = sampleMapTrafficRoute(route, 48.0, 0.0);
        require(body && centerline &&
                    std::abs(std::remainder(body->headingRadians - centerline->headingRadians,
                                            2.0 * std::numbers::pi)) > 0.25,
                "a long vehicle pivots around its rear axle rather than its route center");

        actor.distanceMeters = 44.0;
        actor.speedMetersPerSecond = 4.0;
        const auto start = sampleMapTrafficActorPose(actor, actor.distanceMeters);
        const auto future = sampleMapTrafficActorPose(actor, actor.distanceMeters + 2.0);
        const auto oldFuture = sampleMapTrafficRoute(route, actor.distanceMeters + 2.0, 0.0);
        require(start && future && oldFuture, "turning collision test samples body poses");
        const MapTrafficObstacle startBody{*start, actor.lengthMeters, actor.widthMeters};
        const MapTrafficObstacle futureBody{*future, actor.lengthMeters, actor.widthMeters};
        const MapTrafficObstacle oldFutureBody{*oldFuture, actor.lengthMeters, actor.widthMeters};
        std::optional<MapTrafficObstacle> obstacle;
        for (double x = -12.0; x <= 12.0 && !obstacle; x += 0.25) {
            for (double y = -8.0; y <= 16.0; y += 0.25) {
                MapTrafficPose obstaclePose;
                obstaclePose.position = {x, y, 0.0};
                const MapTrafficObstacle candidate{obstaclePose, 0.25, 0.25};
                if (mapTrafficHasClearance(startBody, {candidate}, 0.1) &&
                    !mapTrafficHasClearance(futureBody, {candidate}, 0.0) &&
                    mapTrafficHasClearance(oldFutureBody, {candidate}, 0.0)) {
                    obstacle = candidate;
                    break;
                }
            }
        }
        require(obstacle.has_value(), "turning vehicle has a swept-body obstacle");
        for (int frame = 0; frame < 60; ++frame) {
            advanceMapTrafficDriving({&actor}, {*obstacle}, 0.05);
            const auto pose = sampleMapTrafficActorPose(actor, actor.distanceMeters);
            require(pose && mapTrafficHasClearance({*pose, actor.lengthMeters, actor.widthMeters},
                                                   {*obstacle}, 0.0),
                    "rear-axle turn preserves physical body clearance");
        }
        require(actor.distanceMeters < 46.0,
                "swept collision stops the rear-axle vehicle before its body reaches an obstacle");
    }
    MapTrafficRouteSegment loop;
    constexpr double radius = 20.0;
    for (int step = 0; step <= 128; ++step) {
        const double angle = 2.0 * std::numbers::pi * step / 128.0;
        loop.points.push_back({radius * (1.0 - std::cos(angle)), radius * std::sin(angle), 0.0});
    }
    double length = 0.0;
    for (std::size_t index = 1; index < loop.points.size(); ++index) {
        length += std::hypot(loop.points[index][0] - loop.points[index - 1][0],
                             loop.points[index][1] - loop.points[index - 1][1]);
    }
    MapTrafficRoute loopRoute;
    loopRoute.segments = {loop};
    loopRoute.cumulativeEndDistances = {length};
    loopRoute.totalLength = length;
    loopRoute.closed = true;
    auto loopActor = makeMapTrafficActor(loopRoute, "loop-rear-axle.ovh", 4.0, 0.0, false);
    loopActor.rearAxleOffsetMeters = -3.0;
    const auto body = sampleMapTrafficActorPose(loopActor, 1.0);
    const auto axle = sampleMapTrafficRoute(loopRoute, length - 2.0, 0.0);
    require(body && axle, "rear axle samples across a closed route seam");
    requireNear(body->position[0] - 3.0 * std::sin(body->headingRadians), axle->position[0],
                "rear axle wraps across the loop horizontally");
    requireNear(body->position[1] - 3.0 * std::cos(body->headingRadians), axle->position[1],
                "rear axle wraps across the loop longitudinally");
    loopActor.rearAxleOffsetMeters = std::numeric_limits<double>::quiet_NaN();
    bool rejectedInvalidOffset = false;
    try {
        (void)sampleMapTrafficActorPose(loopActor, 1.0);
    } catch (const std::invalid_argument&) {
        rejectedInvalidOffset = true;
    }
    require(rejectedInvalidOffset, "non-finite rear axle offsets are rejected");
}

void verifyPassengerSideOffsets() {
    using namespace openbus::map;
    requireNear(mapTrafficPassengerSideSign(false), 1.0,
                "right-hand traffic boards on the right side of travel");
    requireNear(mapTrafficPassengerSideSign(true), -1.0,
                "left-hand traffic boards on the left side of travel");
    requireNear(mapTrafficPassengerCurbOffsetMeters(false, 0.2, 2.0, 0.35), 1.55,
                "right-hand passenger curb offset includes lane and vehicle width");
    requireNear(mapTrafficPassengerCurbOffsetMeters(true, -0.2, 2.0, 0.35), -1.55,
                "left-hand passenger curb offset includes lane and vehicle width");
    const auto northboundRight = mapSplineRightOffset(0.0, 1.0);
    requireNear(northboundRight[0], 1.0,
                "northbound passenger curb offset points east on map coordinates");
    requireNear(northboundRight[1], 0.0,
                "northbound passenger curb offset does not shift along the road");
    const auto eastboundRight = mapSplineRightOffset(std::numbers::pi / 2.0, 1.0);
    requireNear(eastboundRight[0], 0.0,
                "eastbound passenger curb offset does not shift along the road");
    requireNear(eastboundRight[1], -1.0,
                "eastbound passenger curb offset points south on map coordinates");
    const auto eastboundCurbOffset =
        mapTrafficPassengerWorldOffset(false, std::numbers::pi / 2.0, 1.0, 0.0);
    requireNear(eastboundCurbOffset[0], 0.0, "eastbound curb offset does not shift along the road");
    requireNear(eastboundCurbOffset[1], -1.0, "eastbound right-side curb offset points south");
    {
        MapSceneryPose scenery;
        scenery.x = 100.0;
        scenery.y = 200.0;
        scenery.z = 3.0;
        scenery.rotationDegrees[0] = 90.0;
        scenery.rotationCount = 3;
        const auto pose = mapTrafficSceneryPassengerPose(scenery, {1.0, 2.0, 0.5}, 0.25, 30.0);
        requireNear(pose.position[0], 102.0, "passenger position follows editor object rotation");
        requireNear(pose.position[1], 199.0, "passenger position follows editor object origin");
        requireNear(pose.position[2], 3.75, "passenger position includes authored seat height");
        requireNear(pose.headingRadians, 2.0 * std::numbers::pi / 3.0,
                    "passenger heading combines editor and authored rotations");
        scenery.rotationDegrees[1] = 90.0;
        const auto pitched = mapTrafficSceneryPassengerPose(scenery, {0.0, 0.0, 0.0}, 1.0, 0.0);
        const auto transformedUp = transformMapSceneryVector(scenery, {0.0, 0.0, 1.0});
        requireNear(pitched.position[0], scenery.x + transformedUp[0],
                    "seat height follows tilted editor object x axis");
        requireNear(pitched.position[1], scenery.y + transformedUp[1],
                    "seat height follows tilted editor object y axis");
        requireNear(pitched.position[2], scenery.z + transformedUp[2],
                    "seat height follows tilted editor object z axis");
    }
    {
        MapTrafficPose firstPosition;
        firstPosition.position = {36.9, 191.8, 0.25};
        MapTrafficPose secondPosition;
        secondPosition.position = {15.1, 195.8, 0.25};
        MapTrafficPose thirdPosition;
        thirdPosition.position = {24.9, 170.0, 0.25};
        const std::vector<MapPassengerStopAnchor> stops = {
            {8318690, {33.79, 192.28, 0.1}}, {8318798, {16.41, 197.22, 0.1}},
            {8318801, {26.43, 175.47, 0.1}}, {100, {100.0, 100.0, 20.0}}};
        const std::vector<MapPassengerWaitingArea> areas = {
            {8318803, {24.86, 170.03, 0.25}, {thirdPosition}},
            {8318692, {36.98, 191.76, 0.25}, {firstPosition}},
            {8318802, {15.07, 195.78, 0.25}, {secondPosition}},
            {999, {100.0, 100.0, 0.0}, {firstPosition}},
            {1000, {200.0, 200.0, 0.0}, {firstPosition}}};
        const auto assigned = mapTrafficAssignPassengerPositions(stops, areas);
        require(assigned.size() == 3 && assigned.at(8318690)[0].position == firstPosition.position &&
                    assigned.at(8318798)[0].position == secondPosition.position &&
                    assigned.at(8318801)[0].position == thirdPosition.position,
                "BRT waiting areas belong to the nearest nearby stop on the same level");
        require(mapTrafficAssignPassengerPositions({{42, {0.0, 0.0, 0.0}}}, {}).empty(),
                "a stop without an authored waiting area has no generated passenger positions");
        const auto ordered = mapTrafficAssignPassengerPositions(
            {{20, {0.0, 0.0, 0.0}}, {10, {2.0, 0.0, 0.0}}},
            {{4, {1.0, 0.0, 0.0}, {secondPosition}},
             {3, {1.0, 0.0, 0.0}, {firstPosition}}});
        require(ordered.size() == 1 && ordered.at(10).size() == 2 &&
                    ordered.at(10)[0].position == firstPosition.position &&
                    ordered.at(10)[1].position == secondPosition.position,
                "equidistant stops and multiple waiting areas are assigned deterministically");
        const std::array<std::array<double, 3>, 8> areaBounds = {{
            {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {0.0, 4.0, 0.0}, {2.0, 4.0, 0.0},
            {0.0, 0.0, 2.0}, {2.0, 0.0, 2.0}, {0.0, 4.0, 2.0}, {2.0, 4.0, 2.0}}};
        const std::vector<MapTrafficPose> authored = {
            {.position = {0.5, 0.5, 0.0}, .headingRadians = 0.2},
            {.position = {1.5, 3.5, 0.0}, .headingRadians = 0.4}};
        const auto expanded = mapTrafficAssignPassengerPositions(
            {{55, {10.0, 10.0, 0.0}}},
            {{8, {10.1, 10.0, 0.0}, authored, areaBounds}}, {{55, 12}});
        require(expanded.size() == 1 && expanded.at(55).size() == 12 &&
                    expanded.at(55)[0].position == authored[0].position &&
                    expanded.at(55)[1].position == authored[1].position,
                "waiting areas provide authored and excess passenger positions");
        for (std::size_t index = authored.size(); index < expanded.at(55).size(); ++index) {
            const auto& position = expanded.at(55)[index].position;
            require(position[0] >= 0.0 && position[0] <= 2.0 && position[1] >= 0.0 &&
                        position[1] <= 4.0 && position[2] == 0.0,
                    "excess waiting passengers stay within the authored area bounds");
            require(std::none_of(expanded.at(55).begin(),
                                 expanded.at(55).begin() + static_cast<std::ptrdiff_t>(index),
                                 [&](const MapTrafficPose& existing) {
                                     return existing.position == position;
                                 }),
                    "generated passenger positions do not duplicate occupied slots");
        }
    }
}

void verifyPassengerDoorGating() {
    using namespace openbus::map;
    MapPassengerServiceRuntime service(1);
    service.setWaitingPassengers(10, 1);
    require(service.arrive({.stopId = 10, .dwellSeconds = 1.5, .nextStopId = 20}),
            "door-gating fixture starts passenger service");
    service.advance(0.75, false, true);
    require(service.waitingPassengers(10) == 1 && service.onboardPassengerCount() == 0 &&
                service.passengerStopRequestSequence() == 0,
            "closed entry doors prevent boarding and stop requests");
    service.advance(0.75, true, true);
    require(service.waitingPassengers(10) == 0 && service.onboardPassengerCount() == 1 &&
                service.passengerStopRequestSequence() == 0,
            "opening an entry door allows the queued passenger to board");
    service.requestStopForPassengers(20, 200.01);
    service.requestStopForPassengers(30, 0.0);
    require(service.passengerStopRequestSequence() == 0,
            "a passenger does not request a distant or unrelated stop");
    service.requestStopForPassengers(20, 200.0);
    service.requestStopForPassengers(20, 0.0);
    require(service.passengerStopRequestSequence() == 1,
            "a passenger requests their stop once within the 200-metre boundary");
    bool rejectedInvalidDistance = false;
    try {
        service.requestStopForPassengers(20, -1.0);
    } catch (const std::invalid_argument&) {
        rejectedInvalidDistance = true;
    }
    require(rejectedInvalidDistance, "stop requests reject invalid distance inputs");

    require(service.arrive({.stopId = 20, .dwellSeconds = 0.75, .nextStopId = 30}),
            "door-gating fixture reaches the passenger destination");
    service.advance(0.75, true, false);
    require(service.alightingPassenger() && service.onboardPassengerCount() == 1 &&
                service.dwelling(),
            "closed exit doors pause alighting and keep the vehicle at the stop");
    service.advance(0.75, true, true);
    require(!service.alightingPassenger() && service.onboardPassengerCount() == 0 &&
                !service.dwelling(),
            "opening an exit door resumes and completes alighting");

    MapPassengerServiceRuntime queuedAlighting(3);
    queuedAlighting.setWaitingPassengers(40, 3);
    require(queuedAlighting.arrive({.stopId = 40, .dwellSeconds = 2.25, .nextStopId = 50}),
            "queued alighting fixture boards its passengers");
    queuedAlighting.advance(2.25);
    require(queuedAlighting.arrive({.stopId = 50, .dwellSeconds = 5.0, .nextStopId = 60}),
            "queued alighting fixture reaches their destination");
    require(queuedAlighting.alightingSeatIndex() == 0 &&
                queuedAlighting.queuedAlightingSeatIndices() ==
                    std::vector<std::size_t>{1, 2},
            "alighters retain FIFO seat order");
    queuedAlighting.setPassengerAnimationDuration(3.0);
    queuedAlighting.setAlightingDoorWaitProgress(0.8);
    queuedAlighting.setQueuedAlightingPassengerAnimation(1, 3.0, 0.2);
    queuedAlighting.setQueuedAlightingPassengerAnimation(2, 3.0, 0.4);
    queuedAlighting.advance(0.3, true, false, false, true, nullptr, true);
    require(queuedAlighting.alightingProgressForSeat(1) > 0.0 &&
                queuedAlighting.alightingProgressForSeat(1) < 0.2 &&
                queuedAlighting.alightingProgressForSeat(2) > 0.0 &&
                queuedAlighting.alightingProgressForSeat(2) < 0.4,
            "queued alighters walk toward the exit while waiting for the closed door");
    queuedAlighting.setQueuedAlightingPassengerAnimation(1, 3.0, 0.1);
    queuedAlighting.advance(1.0, true, false, false, true, nullptr, true);
    const double firstQueuedProgress = queuedAlighting.alightingProgressForSeat(1);
    require(std::abs(firstQueuedProgress - 0.2) < 1.0e-9 &&
                std::abs(queuedAlighting.alightingProgressForSeat(2) - 0.4) < 1.0e-9,
            "queued alighters wait at their spaced positions until the exit opens");
    queuedAlighting.advance(1.7, true, true, false, true, nullptr, true);
    require(queuedAlighting.alightingSeatIndex() == 1 &&
                std::abs(queuedAlighting.alightingProgress() - firstQueuedProgress) < 1.0e-9 &&
                queuedAlighting.queuedAlightingSeatIndices() ==
                    std::vector<std::size_t>{2},
            "the first queued alighter is promoted with their progress preserved");
}

void verifyPassengerServiceRetirement() {
    using namespace openbus::map;
    MapPassengerServiceRuntime service(2);
    service.setWaitingPassengers(10, 1);
    require(service.arrive({.stopId = 10, .dwellSeconds = 0.75, .nextStopId = 20}),
            "retirement fixture begins the first stop");
    service.advance(0.75);
    require(service.onboardPassengerCount() == 1, "retirement fixture boards a passenger");
    service.setWaitingPassengers(15, 2);
    require(service.arrive({.stopId = 15, .dwellSeconds = 2.0, .nextStopId = 30}),
            "retirement fixture begins the next stop");
    service.advance(0.25);
    require(service.boardingPassenger() && service.waitingPassengers(15) == 1,
            "a passenger is boarding when retirement begins");
    service.endServiceAtCurrentStop();
    require(!service.acceptsBoarding() && !service.boardingPassenger() &&
                service.waitingPassengers(15) == 2 && service.alightingPassenger(),
            "retirement returns the boarding passenger to the queue and unloads onboard riders");
    service.endServiceAtCurrentStop();
    service.advance(1.0, true, false);
    require(service.onboardPassengerCount() == 1 && service.waitingPassengers(15) == 2,
            "retirement respects closed exit doors and does not board waiting passengers");
    service.advance(2.0, true, true);
    require(service.onboardPassengerCount() == 0 && service.waitingPassengers(15) == 2 &&
                !service.dwelling(),
            "retirement unloads all passengers without consuming the shared queue");

    MapPassengerServiceRuntime enRoute(2);
    enRoute.setWaitingPassengers(10, 2);
    require(enRoute.arrive({.stopId = 10, .dwellSeconds = 1.5, .nextStopId = 20}),
            "en-route retirement fixture begins boarding");
    enRoute.advance(1.5);
    openbus::map::MapTrafficPassengerStop nextStop;
    nextStop.stopId = 15;
    nextStop.dwellSeconds = 1.0;
    openbus::map::MapTrafficPassengerStop terminus;
    terminus.stopId = 30;
    openbus::app::ScheduledBusState bus;
    bus.passengerStops = openbus::app::PassengerStops({nextStop, terminus});
    const auto normalStop = openbus::app::scheduledBusStopService(bus, enRoute);
    require(normalStop.nextStopId == 30 && normalStop.passengersAlighting == 0,
            "non-retiring service keeps its normal stop behavior");
    bus.scheduledServiceCancelled = true;
    const auto retirementStop = openbus::app::scheduledBusStopService(bus, enRoute);
    require(retirementStop.nextStopId == 0 && retirementStop.passengersAlighting == 2,
            "retiring service unloads every onboard passenger at the next stop");
    enRoute.setWaitingPassengers(15, 3);
    require(enRoute.arrive(retirementStop), "retiring bus arrives at the next stop");
    enRoute.advance(3.0);
    require(enRoute.onboardPassengerCount() == 0 && enRoute.waitingPassengers(15) == 3 &&
                !enRoute.dwelling(),
            "en-route retirement unloads everyone without boarding from the stop queue");
}

void verifyPassengerStopDepartureHandshake() {
    {
        openbus::app::ScheduledBusState bus;
        bus.scheduledTripActive = true;
        require(!openbus::app::scheduledBusWaitingForPassengers(bus),
                "a bus stopped between passenger calls is still eligible for stuck cleanup");
        bus.passengerStopArrived = true;
        require(openbus::app::scheduledBusWaitingForPassengers(bus),
                "passenger service must not count toward the deadlock timeout");
        openbus::map::MapPassengerServiceRuntime service(1);
        require(service.arrive({.stopId = 10, .dwellSeconds = 20.0}),
                "long-dwell fixture arrives at a passenger stop");
        for (int second = 0; second < 20; ++second) {
            require(service.dwelling() &&
                        openbus::app::scheduledBusWaitingForPassengers(bus),
                    "a valid dwell longer than the 15-second stuck timeout stays exempt");
            service.advance(1.0);
        }
        require(!service.dwelling(), "long passenger dwell eventually completes");
        bus.passengerStopDepartureRequested = true;
        require(openbus::app::scheduledBusWaitingForPassengers(bus),
                "waiting for the door-close handshake remains a legitimate stop");
        openbus::app::beginScheduledBusRetirementRun(bus);
        require(!openbus::app::scheduledBusWaitingForPassengers(bus),
                "a retirement run no longer inherits the passenger-wait exemption");
        bus.passengerStopArrived = true;
        openbus::app::finishScheduledBusTrip(bus, 0.0);
        require(!openbus::app::scheduledBusWaitingForPassengers(bus),
                "an inactive trip cannot suppress stuck detection");
    }
    using namespace openbus::map;
    require(mapTrafficPassengerStopDepartureAction(false, false, false, 1.0) ==
                    MapTrafficPassengerStopDepartureAction::None &&
                mapTrafficPassengerStopDepartureAction(true, true, false, 1.0) ==
                    MapTrafficPassengerStopDepartureAction::None,
            "departure is not requested before arrival or while passenger service is active");
    require(mapTrafficPassengerStopDepartureAction(true, false, false, 1.0) ==
                MapTrafficPassengerStopDepartureAction::RequestDeparture,
            "completed passenger service requests script-controlled door closure");
    require(mapTrafficPassengerStopDepartureAction(true, false, true, -1.0) ==
                    MapTrafficPassengerStopDepartureAction::None &&
                mapTrafficPassengerStopDepartureAction(true, false, true, 1.0) ==
                    MapTrafficPassengerStopDepartureAction::None,
            "the bus remains at the stop until its script clears the departure state");
    require(mapTrafficPassengerStopDepartureAction(true, false, true, 0.0) ==
                MapTrafficPassengerStopDepartureAction::ResumeRoute,
            "the bus resumes only after its script signals that departure can proceed");
    require(mapTrafficPassengerStopDepartureAction(true, false, true, -1.0, true) ==
                    MapTrafficPassengerStopDepartureAction::None &&
                mapTrafficPassengerStopDepartureAction(true, false, true, 1.0, true) ==
                    MapTrafficPassengerStopDepartureAction::None &&
                mapTrafficPassengerStopDepartureAction(true, false, true, 0.0, true) ==
                    MapTrafficPassengerStopDepartureAction::BeginRetirementRun,
            "a conflicting AI bus leaves the stop after the script closes its doors");
    require(mapTrafficPlayerTripMatchesScheduledRoute("line12", "line12", false) &&
                !mapTrafficPlayerTripMatchesScheduledRoute("line12", "other", false) &&
                !mapTrafficPlayerTripMatchesScheduledRoute("", "", false) &&
                !mapTrafficPlayerTripMatchesScheduledRoute("line12", "line12", true),
            "takeover matches the selected trip route without departure-time checks");
    openbus::app::ScheduledBusState bus;
    bus.suppressedDeparture = openbus::app::ScheduledDepartureSuppression{1, 120.0};
    bus.nextScheduledDepartureIndex = 0;
    bus.nextScheduledDepartureElapsedSeconds = 60.0;
    require(!openbus::app::shouldSuppressScheduledDeparture(bus, 60.0),
            "an earlier unrelated departure is not suppressed");
    bus.nextScheduledDepartureIndex = 1;
    bus.nextScheduledDepartureElapsedSeconds = 120.0;
    require(!openbus::app::shouldSuppressScheduledDeparture(bus, 119.0) &&
                openbus::app::shouldSuppressScheduledDeparture(bus, 120.0),
            "only the selected departure is suppressed when it is due");
    bus.scheduledTripActive = true;
    require(!openbus::app::shouldSuppressScheduledDeparture(bus, 120.0),
            "an active bus cannot dispatch a second trip");
    bus.scheduledTripActive = false;
    bus.nextScheduledDepartureElapsedSeconds += 86400.0;
    require(!openbus::app::shouldSuppressScheduledDeparture(bus, 86520.0),
            "suppression does not repeat on the following day");
    bus.suppressedDeparture.reset();
    bus.nextScheduledDepartureElapsedSeconds = 120.0;
    require(!openbus::app::shouldSuppressScheduledDeparture(bus, 120.0),
            "clearing takeover restores an unconsumed departure");
    bus.scheduledTripActive = true;
    bus.scheduledServiceCancelled = true;
    bus.suppressedDeparture = openbus::app::ScheduledDepartureSuppression{1, 120.0};
    openbus::app::finishScheduledBusTrip(bus, 119.0);
    require(!bus.scheduledTripActive && !bus.scheduledServiceCancelled &&
                bus.suppressedDeparture.has_value(),
            "retiring a bus releases cancellation while preserving a future suppression");
    bus.scheduledTripActive = true;
    openbus::app::finishScheduledBusTrip(bus, 120.0);
    require(!bus.suppressedDeparture,
            "finishing an overlapping trip clears an already missed suppression");
}

void verifyPassengerComponentAllocation() {
    auto definition = std::make_shared<SharedVehicleDefinition>();
    require(!openbus::app::vehicle::makePassengerState(definition, false),
            "a car without passenger configuration has no passenger component");
    const auto emptyBus = openbus::app::vehicle::makePassengerState(definition, true);
    require(emptyBus && emptyBus->service.seatCount() == 0,
            "a scheduled bus without seats still has passenger service");
    definition->vehicleConfiguration.passengerPositions.emplace_back();
    const auto first = openbus::app::vehicle::makePassengerState(definition, false);
    const auto second = openbus::app::vehicle::makePassengerState(definition, false);
    require(first && second && first->service.seatCount() == 1 &&
                &first->configuration() == &second->configuration(),
            "passenger-capable vehicles share immutable cabin geometry");
    require(&first->service != &second->service,
            "each passenger-capable vehicle has independent service state");
}

void verifyPassengerStopSharing() {
    openbus::map::MapTrafficPassengerStop stop;
    stop.stopId = 12;
    stop.routeDistanceMeters = 35.0;
    const openbus::app::PassengerStops catalog({stop});
    const auto matching = openbus::app::PassengerStops::shareIfEqual({stop}, catalog);
    require(&matching.values() == &catalog.values(),
            "identical mapped stops share catalog storage");
    stop.routeDistanceMeters = 36.0;
    const auto different = openbus::app::PassengerStops::shareIfEqual({stop}, catalog);
    require(&different.values() != &catalog.values() &&
                different[0].routeDistanceMeters == 36.0,
            "route-specific stop distances retain separate storage");
    auto actor = std::make_shared<const openbus::map::MapTrafficActor>();
    const std::weak_ptr<const openbus::map::MapTrafficActor> snapshot = actor;
    openbus::app::PassengerStopRouteState route{actor, catalog, {}, {}, {}, {}};
    actor.reset();
    require(!snapshot.expired() && route.actor == snapshot.lock(),
            "passenger routes retain the immutable worker actor snapshot");

    openbus::map::MapTrafficRoute trafficRoute;
    trafficRoute.segments = {
        {{makeSpline(10, 0, 11, 0.0), 0, 0}},
        {{makeSpline(11, 10, 0, 10.0), 0, 0}},
    };
    trafficRoute.cumulativeEndDistances = {10.0, 20.0};
    trafficRoute.totalLength = 20.0;
    openbus::map::MapTrafficPassengerStop first;
    first.stopId = 1;
    first.splineId = 10;
    first.splinePositionMeters = 5.0;
    openbus::map::MapTrafficPassengerStop unmatched;
    unmatched.stopId = 2;
    unmatched.splineId = 99;
    openbus::map::MapTrafficPassengerStop last;
    last.stopId = 3;
    last.splineId = 11;
    std::vector<int> unmatchedIds;
    const auto mapped = openbus::app::mapPassengerStopsToTrafficRoute(
        trafficRoute, {first, unmatched, last},
        [&](const auto& missed) { unmatchedIds.push_back(missed.stopId); });
    require(mapped.size() == 2 && mapped[0].routeDistanceMeters == 5.0 &&
                mapped[1].routeDistanceMeters == 10.0 &&
                mapped[1].passengersAlighting == (std::numeric_limits<std::size_t>::max)() &&
                unmatchedIds == std::vector<int>{2},
            "AI stop projection preserves offsets, order, terminus, and unmatched diagnostics");
    bool invalidRejected = false;
    try {
        first.stopId = 0;
        openbus::app::mapPassengerStopsToTrafficRoute(trafficRoute, {first},
                                                      [](const auto&) {});
    } catch (const std::invalid_argument&) {
        invalidRejected = true;
    }
    require(invalidRejected, "AI stop projection rejects invalid stops");
}

void verifyPlayerTripPolicyLifecycle() {
    using namespace openbus::app;
    ScheduledBusState first;
    ScheduledBusState second;
    first.timetableKey = second.timetableKey = "route";
    first.scheduledDepartureMinutes = second.scheduledDepartureMinutes = {480.0, 490.0};
    first.scheduledDepartureProfileIndices = second.scheduledDepartureProfileIndices = {1, 1};
    first.nextScheduledDepartureElapsedSeconds = second.nextScheduledDepartureElapsedSeconds =
        28800.0;
    std::uint64_t firstGeneration = 0;
    std::uint64_t secondGeneration = 0;
    const std::array targets{PlayerTripPolicyTarget{first, firstGeneration},
                             PlayerTripPolicyTarget{second, secondGeneration}};
    const openbus::map::MapAiTimetableCourseTrip trip{"route", 1, 480.0};

    applyPlayerTripPolicy(targets, trip, true);
    require(!first.suppressedDeparture && !second.suppressedDeparture && firstGeneration == 0,
            "additional buses leave AI departures unchanged");
    applyPlayerTripPolicy(targets, trip, false);
    require(first.suppressedDeparture && !second.suppressedDeparture && firstGeneration == 1,
            "one matching duplicate departure is selected deterministically");
    clearPlayerTripPolicy(targets);
    require(!first.suppressedDeparture && firstGeneration == 2,
            "releasing a pending takeover invalidates worker decisions");
    applyPlayerTripPolicy(targets, trip, false);
    require(first.suppressedDeparture &&
                first.suppressedDeparture->departureElapsedSeconds == 28800.0,
            "an overdue pending departure can still be taken over");
    require(consumeSuppressedDeparture(first, 0.0, 28801.0) &&
                first.nextScheduledDepartureIndex == 1 &&
                first.nextScheduledDepartureElapsedSeconds == 29400.0 &&
                !first.suppressedDeparture,
            "the selected departure is consumed once and the next trip remains available");
    const std::array firstTarget{PlayerTripPolicyTarget{first, firstGeneration}};
    applyPlayerTripPolicy(firstTarget, trip, false);
    require(first.suppressedDeparture && first.suppressedDeparture->departureIndex == 1,
            "pending takeover matches the route despite a different departure time");
    clearPlayerTripPolicy(firstTarget);
    advanceScheduledBusDeparture(first, 0.0, 29400.0);
    require(first.nextScheduledDepartureIndex == 0 &&
                first.nextScheduledDepartureElapsedSeconds == 115200.0 &&
                !consumeSuppressedDeparture(first, 0.0, 115200.0),
            "the same departure runs normally on the following day");
    first.scheduledTripActive = true;
    first.activeTimetableDepartureIndex = 1;
    first.activeScheduledDepartureElapsedSeconds = 1.0;
    first.scheduledDepartureProfileIndices[1] = 3;
    second.scheduledTripActive = true;
    second.activeTimetableDepartureIndex = 0;
    second.timetableKey = "other";
    applyPlayerTripPolicy(targets, trip, false);
    require(first.scheduledServiceCancelled && !second.scheduledServiceCancelled,
            "active buses on unrelated trip routes continue normally");
    second.timetableKey = "route";
    applyPlayerTripPolicy(targets, trip, false);
    require(first.scheduledServiceCancelled && second.scheduledServiceCancelled &&
                !second.suppressedDeparture,
            "active matching buses retire regardless of their departure time or profile");
    clearPlayerTripPolicy(targets);
    require(first.scheduledServiceCancelled,
            "an already active takeover remains committed when the player advances");
    openbus::map::MapTrafficPassengerStop firstStop;
    firstStop.routeDistanceMeters = 50.0;
    openbus::map::MapTrafficPassengerStop lastStop;
    lastStop.routeDistanceMeters = 200.0;
    first.passengerStops = PassengerStops({firstStop, lastStop});
    first.passengerStopArrived = true;
    first.passengerStopDepartureRequested = true;
    beginScheduledBusRetirementRun(first);
    require(first.retirementDrivingToTerminus && !first.passengerStopArrived &&
                !first.passengerStopDepartureRequested &&
                first.nextPassengerStopIndex == first.passengerStops.size() &&
                scheduledBusRetirementTerminus(first) == 200.0,
            "after unloading the bus skips intermediate stops and targets the terminus");
    require(!scheduledBusReachedRetirementTerminus(first, 199.9, 0.0, 0.2) &&
                !scheduledBusReachedRetirementTerminus(first, 200.0, 0.5, 0.2) &&
                scheduledBusReachedRetirementTerminus(first, 199.96, 0.0, 0.2),
            "retiring buses despawn only after stopping at the terminus");
    finishScheduledBusTrip(first, 115210.0);
    require(!first.scheduledTripActive && !first.scheduledServiceCancelled &&
                !first.retirementDrivingToTerminus,
            "retirement releases the bus for later departures");
}

void verifyScheduledBusTiming() {
    openbus::app::ScheduledBusState bus;
    openbus::map::MapTrafficActor actor;
    openbus::map::MapTrafficPassengerStop first;
    openbus::map::MapTrafficPassengerStop second;
    first.timetableStationIndex = 0;
    second.timetableStationIndex = 1;
    second.routeDistanceMeters = 100.0;
    bus.passengerStops = openbus::app::PassengerStops({first, second});
    bus.scheduledDepartureProfileIndices = {0};
    openbus::map::MapAiTimetableProfile profile;
    profile.arrivalOffsetSeconds = {0.0, 300.0};
    profile.departureOffsetSeconds = {60.0, 320.0};
    bus.timetableProfiles =
        std::make_shared<const std::vector<openbus::map::MapAiTimetableProfile>>(
            std::vector{profile});
    requireNear(openbus::app::scheduledBusDelaySeconds(bus, actor, 30.0), 0.0,
                "inactive scheduled trips have no delay");
    bus.scheduledTripActive = true;
    requireNear(openbus::app::scheduledBusDelaySeconds(bus, actor, 30.0), 0.0,
                "scheduled dwell time does not count as delay");
    actor.distanceMeters = 50.0;
    requireNear(openbus::app::scheduledBusDelaySeconds(bus, actor, 170.0), -10.0,
                "scheduled travel delay interpolates between stop times");
    actor.distanceMeters = 100.0;
    requireNear(openbus::app::scheduledBusDelaySeconds(bus, actor, 340.0), 20.0,
                "delay after the final stop reflects its departure time");
    bus.activeTimetableDepartureIndex = 1;
    requireNear(openbus::app::scheduledBusDelaySeconds(bus, actor, 340.0), 0.0,
                "missing departure profiles preserve the existing zero delay");

    openbus::app::PassengerStopRouteState playerRoute;
    openbus::map::MapTrafficPassengerStop playerFirst;
    playerFirst.timetableStationIndex = 0;
    openbus::map::MapTrafficPassengerStop playerSecond;
    playerSecond.timetableStationIndex = 1;
    playerRoute.stops = openbus::app::PassengerStops({playerFirst, playerSecond});
    playerRoute.stopPoses = {
        openbus::map::MapTrafficPose{.position = {0.0, 0.0, 0.0}},
        openbus::map::MapTrafficPose{.position = {100.0, 0.0, 0.0}},
    };
    const std::array<double, 3> halfwayPosition = {50.0, 0.0, 0.0};
    const auto playerDelay = openbus::app::playerTimetableDelaySeconds(
        playerRoute, profile, 480.0, 480.0 * 60.0 + 170.0, 0, 1, halfwayPosition, std::nullopt);
    require(playerDelay.has_value(), "player timetable delay requires a valid next stop");
    requireNear(*playerDelay, -10.0, "player trip delay interpolates between stop schedules");
    const auto dwellingDelay = openbus::app::playerTimetableDelaySeconds(
        playerRoute, profile, 480.0, 480.0 * 60.0 + 340.0, 0, 1, halfwayPosition, 1);
    require(dwellingDelay.has_value(), "player timetable dwell delay is available");
    requireNear(*dwellingDelay, 20.0, "player trip delay uses scheduled dwell time");
}

void verifyPlayerTimetableController() {
    using namespace openbus::app;
    openbus::map::MapTrafficPassengerStop stop;
    const PassengerStops stops({stop, stop});
    const auto actor = std::make_shared<const openbus::map::MapTrafficActor>();
    const std::vector<PassengerStopRouteState> routes{
        {actor, stops, "outbound", "line A", {}, {}},
        {actor, stops, "return", "line B", {}, {}},
    };
    const std::vector<openbus::map::MapAiTimetableCourse> courses{
        {"course", {{"outbound", 0, 480.0}, {"unavailable", 0, 500.0},
                    {"return", 0, 520.0}}},
    };
    PlayerTimetableController controller;
    const PlayerTimetableSelection selection{0, 0, 0, 1, true};
    const auto* trip = controller.select(selection, routes, courses);
    require(trip && trip->timetableKey == "outbound" && controller.state &&
                controller.state->nextStopIndex == 1 &&
                controller.state->selection.driveAsAdditionalBus,
            "selection retains the requested course, first stop, and additional-bus mode");
    std::optional<PlayerTimetableStopVisitLatch> stopVisitLatch{
        PlayerTimetableStopVisitLatch{1, {10.0, 20.0}}};
    require(!releasePlayerTimetableStopVisit(stopVisitLatch, 1, {60.0, 20.0, 0.0}, 50.0) &&
                stopVisitLatch.has_value() &&
                !releasePlayerTimetableStopVisit(stopVisitLatch, 1, {59.99, 20.0, 0.0}, 50.0),
            "timetable stop remains current while the bus stays within the stop radius");
    require(releasePlayerTimetableStopVisit(stopVisitLatch, 1, {60.01, 20.0, 0.0}, 50.0) == 1 &&
                !stopVisitLatch,
            "timetable stop releases only after the bus drives away");
    trip = controller.advance(routes, courses);
    require(trip && trip->timetableKey == "return" && controller.state &&
                controller.state->selection.courseTripIndex == 2 &&
                controller.state->selection.passengerRouteIndex == 1 &&
                controller.state->nextStopIndex == 0 &&
                controller.state->selection.driveAsAdditionalBus,
            "terminus progression selects the next available course trip");
    controller.state->stopAfterCurrentTrip = true;
    require(!controller.advance(routes, courses) && !controller.state,
            "stop-after-trip ends player timetable service");
    controller.select(selection, routes, courses);
    controller.advance(routes, courses);
    require(!controller.advance(routes, courses) && !controller.state,
            "course end clears player timetable service");
    bool rejectedInvalidStop = false;
    try {
        controller.select(PlayerTimetableSelection{0, 0, 0, 2, false}, routes, courses);
    } catch (const std::out_of_range&) {
        rejectedInvalidStop = true;
    }
    require(rejectedInvalidStop && !controller.state,
            "invalid first stops cannot start timetable service");
    bool rejectedMismatchedRoute = false;
    try {
        controller.select(PlayerTimetableSelection{1, 0, 0, 0, false}, routes, courses);
    } catch (const std::invalid_argument&) {
        rejectedMismatchedRoute = true;
    }
    require(rejectedMismatchedRoute && !controller.state,
            "a selected trip must match its passenger route");
    controller.select(selection, routes, courses);
    require(!controller.select(std::nullopt, routes, courses) && !controller.state,
            "cancellation immediately clears an active timetable selection");

    auto hofStops = stops.values();
    hofStops[0].name = "Central Station";
    hofStops[1].name = "STOP_B";
    const PassengerStopRouteState hofRoute{
        actor, PassengerStops(hofStops), "hof-trip", "0100", {}, {}};
    openbus::config::HofData hof;
    hof.busstops = {{"STOP_A", {"Central Station"}}, {"STOP_B", {"Airport"}}};
    hof.routes = {{"1001", 0, false, {"STOP_A", "STOP_B"}, "Outbound", "100"}};
    openbus::map::MapAiTimetableRoute scheduledRoute;
    scheduledRoute.timetableKey = "hof-trip";
    scheduledRoute.lineName = "100";
    scheduledRoute.scheduledDepartureMinutes = {480.0};
    scheduledRoute.trackEntries = {{1001, 0, 0, 10.0}, {1002, 0, 0, 10.0}};
    openbus::map::MapAiTimetableRoute::Station firstStation;
    firstStation.id = 1;
    firstStation.name = "Central Station";
    openbus::map::MapAiTimetableRoute::Station secondStation;
    secondStation.id = 2;
    secondStation.trackEntryIndex = 1;
    secondStation.name = "STOP_B";
    scheduledRoute.stations = {firstStation, secondStation};
    const auto independentRoute = makePlayerTimetableRoute(
        scheduledRoute, 4, [](const auto& stop) -> std::optional<openbus::map::MapTrafficPose> {
            if (stop.stopId == 2) {
                return std::nullopt;
            }
            return openbus::map::MapTrafficPose{};
        });
    require(!independentRoute.actor && independentRoute.stops.size() == 2 &&
                independentRoute.stopPose(0) && !independentRoute.stopPose(1) &&
                passengerRouteMatchesHof(independentRoute, hof),
            "map schedule and HOF remain selectable without an AI route or all stop geometry");
    require(matchingPlayerTimetableRouteCount({independentRoute}, hof) == 1,
            "HOF auto-matching scores distinct map timetable routes that it can serve");
    openbus::config::HofData unrelatedHof;
    unrelatedHof.routes = {{"9001", 0, false, {"UNRELATED_A", "UNRELATED_B"}, "Other", "900"}};
    require(matchingPlayerTimetableRouteCount({independentRoute}, unrelatedHof) == 0,
            "HOF auto-matching rejects files with no compatible map timetable route");
    const std::vector<openbus::map::MapAiTimetableCourse> independentCourses{
        {"course", {{"hof-trip", 0, 480.0}}}};
    PlayerTimetableCatalog independentCatalog;
    independentCatalog.update({independentRoute}, independentCourses, hof);
    require(independentCatalog.lines.size() == 1 &&
                independentCatalog.lines.front().courses.front().trips.size() == 1,
            "the player catalog includes scheduled trips independent of AI route construction");
    PassengerNetwork network;
    openbus::map::MapAiTrafficData traffic;
    traffic.timetableRoutes.push_back(scheduledRoute);
    const auto report = network.registerTimetables(
        traffic, 4, [](const auto& stop) -> std::optional<openbus::map::MapTrafficPose> {
            if (stop.stopId == 2) {
                return std::nullopt;
            }
            return openbus::map::MapTrafficPose{};
        });
    require(report.missingStops == 1 && report.routesWithMissingStops == 1 &&
                report.routesWithoutStops == 0 && network.findRouteIndex("hof-trip") == 0 &&
                network.queues.at(1) == 4 && !network.queues.contains(2),
            "registration reports missing locations and initializes only located stops");
    openbus::map::MapAiTrafficData stopOnlyTraffic;
    stopOnlyTraffic.busStops = {{"Alpha", 0, 10, {}}, {"Beta", 0, 11, {}},
                                {"Gamma", 0, 12, {}}};
    openbus::map::MapAiPassengerTimetable stopOnlyTimetable;
    stopOnlyTimetable.timetableKey = "stop-only";
    stopOnlyTimetable.lineName = "42";
    stopOnlyTimetable.stationIds = {10, 11, 12};
    stopOnlyTimetable.scheduledDepartureMinutes = {480.0};
    stopOnlyTraffic.passengerTimetables.push_back(stopOnlyTimetable);
    auto sharedDrivingTimetable = scheduledRoute;
    sharedDrivingTimetable.timetableKey = "stop-only";
    stopOnlyTraffic.timetableRoutes.push_back(sharedDrivingTimetable);
    require(scheduledPlayerTimetableKeys(stopOnlyTraffic) == std::vector<std::string>{"stop-only"},
            "scheduled route keys include passenger timetables without duplicating driving routes");
    stopOnlyTraffic.timetableRoutes.clear();
    openbus::map::MapAiStationLink partialLink;
    partialLink.fromStopId = 10;
    partialLink.toStopId = 11;
    partialLink.entries = {{100, 1, 0, 20.0, {}}};
    stopOnlyTraffic.stationLinks.push_back(partialLink);
    PassengerNetwork stopOnlyNetwork;
    const auto stopOnlyReport = stopOnlyNetwork.registerTimetables(
        stopOnlyTraffic, 4, [](const auto&) -> std::optional<openbus::map::MapTrafficPose> {
            return openbus::map::MapTrafficPose{};
        });
    require(stopOnlyNetwork.routes.size() == 1 &&
                stopOnlyNetwork.routes[0].stops.size() == 3 &&
                stopOnlyNetwork.routes[0].stops[0].name == "Alpha" &&
                stopOnlyNetwork.routes[0].stops[2].name == "Gamma" &&
                stopOnlyNetwork.routes[0].stopPose(0) && stopOnlyNetwork.routes[0].stopPose(1) &&
                !stopOnlyNetwork.routes[0].stopPose(2) &&
                stopOnlyNetwork.routes[0].stops[1].splinePositionMeters == 20.0 &&
                stopOnlyReport.missingStops == 1 && stopOnlyReport.routesWithoutStops == 0,
            "station-only trips register without complete links and use each available link");
    const auto firstStopPose = stopOnlyNetwork.routes[0].stopPose(0);
    require(firstStopPose.has_value(), "first line stop has a route pose");
    const auto anyLineStop = stopOnlyNetwork.findNearestStop(
        firstStopPose->position, firstStopPose->headingRadians, 0, 0);
    const auto nextScheduledStop = stopOnlyNetwork.findNearestStop(
        firstStopPose->position, firstStopPose->headingRadians, 0, 1);
    require(anyLineStop.stopIndex == 0 && nextScheduledStop.stopIndex == 1,
            "proximity lookup can consider earlier line stops independently of service order");
    stopOnlyTraffic.stationLinks.clear();
    PassengerNetwork unlocatedNetwork;
    const auto unlocatedReport = unlocatedNetwork.registerTimetables(
        stopOnlyTraffic, 4, [](const auto&) -> std::optional<openbus::map::MapTrafficPose> {
            throw std::logic_error("an unlocated stop must not invoke the pose resolver");
        });
    openbus::config::HofData stopOnlyHof;
    stopOnlyHof.routes = {{"4201", 0, false, {"Alpha", "Beta", "Gamma"}, "Outbound", "42"}};
    const std::vector<openbus::map::MapAiTimetableCourse> stopOnlyCourses{
        {"course", {{"stop-only", 0, 480.0}}}};
    PlayerTimetableCatalog stopOnlyCatalog;
    stopOnlyCatalog.update(unlocatedNetwork.routes, stopOnlyCourses, stopOnlyHof);
    PlayerTimetableController stopOnlyController;
    require(unlocatedReport.routesWithoutStops == 1 && unlocatedNetwork.routes.size() == 1 &&
                stopOnlyCatalog.lines.size() == 1 &&
                stopOnlyController.select({PlayerTimetableSelection{0, 0, 0, 2, false}},
                                          unlocatedNetwork.routes, stopOnlyCourses),
            "trips with existing but unlocated stops remain selectable");
    auto missingStopTraffic = stopOnlyTraffic;
    missingStopTraffic.busStops.pop_back();
    PassengerNetwork missingStopNetwork;
    const auto missingStopReport = missingStopNetwork.registerTimetables(
        missingStopTraffic, 4, [](const auto&) -> std::optional<openbus::map::MapTrafficPose> {
            return std::nullopt;
        });
    require(missingStopNetwork.routes.empty() && missingStopReport.routesMissingStopDefinitions == 1 &&
                missingStopReport.missingStopDefinitionExamples.find("stop ID 12") !=
                    std::string::npos,
            "missing stop definitions are diagnosed instead of inventing a stop name");
    auto ambiguousStopTraffic = stopOnlyTraffic;
    ambiguousStopTraffic.busStops.push_back(ambiguousStopTraffic.busStops.back());
    PassengerNetwork ambiguousStopNetwork;
    require(ambiguousStopNetwork.registerTimetables(
                ambiguousStopTraffic, 4,
                [](const auto&) -> std::optional<openbus::map::MapTrafficPose> {
                    return std::nullopt;
                }).routesMissingStopDefinitions == 1 &&
                ambiguousStopNetwork.routes.empty(),
            "ambiguous stop IDs are not resolved arbitrarily");
    auto authoredStopTraffic = stopOnlyTraffic;
    authoredStopTraffic.busStops.clear();
    auto& authoredStops = authoredStopTraffic.passengerTimetables[0].stationRecords;
    authoredStops = {openbus::map::MapAiTimetableRoute::Station{10, 0, "Alpha", 0, {}, {}},
                     openbus::map::MapAiTimetableRoute::Station{11, 0, "Beta", 0, {}, {}},
                     openbus::map::MapAiTimetableRoute::Station{12, 0, "Gamma", 0, {}, {}}};
    PassengerNetwork authoredStopNetwork;
    require(authoredStopNetwork.registerTimetables(
                authoredStopTraffic, 4,
                [](const auto&) -> std::optional<openbus::map::MapTrafficPose> {
                    return std::nullopt;
                }).routesWithoutStops == 1 &&
                authoredStopNetwork.routes.size() == 1,
            "explicit timetable stops register without Busstops.cfg or road links");
    network.queues.at(1) = 2;
    network.initializeQueues(independentRoute.stops.values());
    require(network.queues.at(1) == 2 && network.initialQueues.at(1) == 4 &&
                network.queues.at(2) == 4,
            "shared stops preserve existing passenger queues");
    const auto nearby = network.findNearestStop({0.0, 0.0, 0.0}, 0.0, 0, 0);
    require(nearby.routeIndex == 0 && nearby.stopIndex == 0 && nearby.distanceMeters == 0.0,
            "nearest stop honors route and heading");
    const auto editorNearby = network.findNearestStop(
        {10.0, 0.0, 0.0}, 0.0, std::nullopt, 0,
        [](const auto& stop) -> std::optional<std::array<double, 3>> {
            return stop.stopId == 1 ? std::optional{std::array<double, 3>{10.0, 0.0, 0.0}}
                                    : std::nullopt;
        });
    require(editorNearby.stopIndex == 0 && editorNearby.distanceMeters == 0.0,
            "nearest stop uses the editor position when available");
    require(!std::isfinite(network.findNearestStop({0.0, 0.0, 0.0}, std::numbers::pi, 0, 0)
                               .distanceMeters) &&
                !std::isfinite(network.findNearestStop({0.0, 0.0, 0.0}, 0.0, 0, 1)
                                   .distanceMeters) &&
                !nextLocatedPassengerStop(network.routes[0], 1),
            "wrong directions, earlier stops, and unlocated stops are excluded");
    auto editorOnlyRoute = network.routes[0];
    editorOnlyRoute.stopPoses.assign(editorOnlyRoute.stops.size(), std::nullopt);
    const auto editorOnlyStopIndex = nextPassengerStopWithPosition(
        editorOnlyRoute, 1, [](const auto& stop) -> std::optional<std::array<double, 3>> {
            return stop.stopId == 2 ? std::optional{std::array<double, 3>{10.0, 0.0, 0.0}}
                                    : std::nullopt;
        });
    require(editorOnlyStopIndex == std::optional<std::size_t>{1},
            "editor-only bus-stop positions locate stops without route poses");
    auto placeholderHof = hof;
    placeholderHof.routes.front().line = "XXX";
    PlayerTimetableCatalog placeholderCatalog;
    placeholderCatalog.update({independentRoute}, independentCourses, placeholderHof);
    require(placeholderCatalog.lines.size() == 1 &&
                placeholderCatalog.lines.front().name == "100",
            "an HOF placeholder line displays the map's public line instead of XXX");
    auto tripKeyOnlyRoute = independentRoute;
    tripKeyOnlyRoute.lineName = tripKeyOnlyRoute.timetableKey;
    require(passengerRouteDisplayLine(tripKeyOnlyRoute, placeholderHof.routes.front()) == "XXX",
            "an internal trip key is not displayed as a public line");
    require(passengerRouteMatchesHof(hofRoute, hof),
            "HOF matching uses the line and ordered stop identifiers or display names");
    auto tripKeyLine = hofRoute;
    tripKeyLine.lineName = "outbound-trip-key";
    require(passengerRouteHofRoute(tripKeyLine, hof) &&
                passengerRouteHofRoute(tripKeyLine, hof)->line == "100",
            "HOF supplies the public line when a map uses a trip key as its line label");
    auto ambiguousHof = hof;
    ambiguousHof.routes.push_back(
        {"1011", 0, false, {"STOP_A", "STOP_B"}, "Other line", "101"});
    require(!passengerRouteMatchesHof(tripKeyLine, ambiguousHof),
            "ambiguous HOF lines cannot be inferred from a trip key");
    require(passengerRouteMatchesHof(hofRoute, ambiguousHof),
            "an authored line disambiguates identical HOF stop sequences");
    auto extendedHof = hof;
    extendedHof.busstops.push_back({"STOP_MID", {"Midtown"}});
    extendedHof.routes[0].busstopIdents = {"STOP_A", "STOP_A", "STOP_MID", "STOP_B"};
    require(passengerRouteMatchesHof(hofRoute, extendedHof),
            "a scheduled partial route matches an ordered subset of its HOF stops");
    auto reversePartialRoute = hofRoute;
    auto reversePartialStops = hofStops;
    std::ranges::reverse(reversePartialStops);
    reversePartialRoute.stops = PassengerStops(std::move(reversePartialStops));
    require(!passengerRouteMatchesHof(reversePartialRoute, extendedHof),
            "partial HOF stop matching still distinguishes route direction");
    openbus::config::HofData destinationOnlyHof;
    destinationOnlyHof.termini = {{7, {"Central Hub"}}};
    destinationOnlyHof.routes = {{"7001", 7, true, {}, "Outbound", "7"}};
    auto destinationOnlyRoute = hofRoute;
    destinationOnlyRoute.lineName = "7";
    destinationOnlyRoute.destinationName = "7 Central Hub";
    require(passengerRouteMatchesHof(destinationOnlyRoute, destinationOnlyHof),
            "a stop-list-free HOF route matches by its line and exact terminus");
    destinationOnlyRoute.destinationName = "7 Other Hub";
    require(!passengerRouteMatchesHof(destinationOnlyRoute, destinationOnlyHof),
            "a stop-list-free HOF route does not match by line alone");
    const auto grundorfRoute = [&](std::vector<std::string> names) {
        auto mapStops = stops.values();
        mapStops.clear();
        for (std::string& name : names) {
            auto stop = stops.front();
            stop.name = std::move(name);
            mapStops.push_back(std::move(stop));
        }
        return PassengerStopRouteState{actor, PassengerStops(std::move(mapStops)),
                                       "grundorf-trip", "76", {}, {}};
    };
    openbus::config::HofData grundorfHof;
    grundorfHof.routes = {
        {"7601", 0, false,
         {"Bauernhof", "Nordspitze", "Elsterplatz", "Gaussdorf", "Einsteindorf Ausbau",
          "Einsteindorf Marktpl", "Krankenhaus"},
         "Bauernhof-Krankenhaus", "TML"},
        {"7602", 0, false,
         {"Krankenhaus", "Einsteindorf Ausbau", "Gaussdorf", "Elsterplatz", "Nordspitze",
          "Bauernhof"},
         "Krankenhaus-Bauernhof", "76"},
    };
    const auto grundorfOutbound = grundorfRoute(
        {"Bauernhof", "Bauernhof", "Nordspitze", "Elsterplatz", "Gaussdorf",
         "Einsteindorf Ausbau", "Einsteindorf Marktpl", "Krankenhaus"});
    require(passengerRouteMatchesHof(grundorfOutbound, grundorfHof),
            "Grundorf outbound HOF route matches when map has an additional terminal stop");
    const auto grundorfInbound = grundorfRoute(
        {"Krankenhaus", "Einsteindorf Marktpl", "Einsteindorf Ausbau", "Gaussdorf",
         "Elsterplatz", "Nordspitze", "Bauernhof"});
    require(passengerRouteMatchesHof(grundorfInbound, grundorfHof),
            "Grundorf return HOF route matches when the map has an intermediate extra stop");
    const auto wrongGrundorfDirection = grundorfRoute(
        {"Bauernhof", "Nordspitze", "Elsterplatz", "Gaussdorf", "Einsteindorf Ausbau",
         "Krankenhaus"});
    require(passengerRouteMatchesHof(wrongGrundorfDirection, grundorfHof),
            "a partial map timetable matches when its stops remain ordered in the HOF");
    const auto unrelatedGrundorfRoute = grundorfRoute(
        {"Bauernhof", "Nordspitze", "Unlisted stop", "Gaussdorf", "Einsteindorf Ausbau",
         "Einsteindorf Marktpl", "Krankenhaus"});
    require(!passengerRouteMatchesHof(unrelatedGrundorfRoute, grundorfHof),
            "HOF matching rejects a route containing a stop outside the HOF sequence");
    const auto brtRoute = [&](std::vector<std::string> names) {
        std::vector<openbus::map::MapTrafficPassengerStop> mapStops;
        for (auto& name : names) {
            auto stop = stops.front();
            stop.name = std::move(name);
            mapStops.push_back(std::move(stop));
        }
        return PassengerStopRouteState{actor, PassengerStops(std::move(mapStops)),
                                       "626-trip", "626", {}, {}};
    };
    openbus::config::HofData brtHof;
    brtHof.routes = {
        {"626005", 0, false,
         {"Waldschaenke", "Gruener Weg", "See", "HOF detour", "Neissestr.",
          "Mitte", "Heinersdorfer Weg"},
         "Outbound", "XXX"},
    };
    const auto brtOutbound = brtRoute(
        {"Waldsch\xc3\xa4nke 3 Terminus", "Gr\xfcner Weg", "Map detour", "See",
         "Nei\xc3\x9f" "estr.", "Mitte", "Heinersdorfer Weg/Reha"});
    require(passengerRouteMatchesHof(brtOutbound, brtHof),
            "placeholder HOF routes match stop aliases and different ordered detours");
    auto reversedBrtStops = brtOutbound.stops.values();
    std::ranges::reverse(reversedBrtStops);
    auto reversedBrt = brtOutbound;
    reversedBrt.stops = PassengerStops(std::move(reversedBrtStops));
    require(!passengerRouteMatchesHof(reversedBrt, brtHof),
            "relaxed matching rejects the opposite direction");
    const auto unrelatedBrt = brtRoute(
        {"Waldsch\xc3\xa4nke 3", "Other 1", "Other 2", "See", "Other 3",
         "Other 4", "Mitte", "Heinersdorfer Weg/Reha"});
    require(!passengerRouteMatchesHof(unrelatedBrt, brtHof),
            "shared endpoints and a few stops do not match an unrelated itinerary");
    auto wrongLineHof = brtHof;
    wrongLineHof.routes[0].line = "999";
    require(!passengerRouteMatchesHof(brtOutbound, wrongLineHof),
            "relaxed matching does not override an explicit different HOF line");
    auto tripKeyBrt = brtOutbound;
    tripKeyBrt.lineName = tripKeyBrt.timetableKey;
    require(!passengerRouteMatchesHof(tripKeyBrt, brtHof),
            "relaxed matching cannot infer a public line from an internal trip key");
    openbus::config::HofData x10Hof;
    x10Hof.routes = {
        {"510230", 0, false, {"Middle A", "Middle B"}, "Short route", "XXX"},
        {"510002", 0, false,
         {"Teltow Bahnhof", "Middle A", "Middle B", "Third", "Hertzallee*T13"},
         "Outbound", "XXX"},
    };
    auto x10 = brtRoute({"Bahnhof Terminus", "Teltow Bahnhof", "Middle A", "Middle B",
                         "Third", "Hertzallee X10"});
    x10.lineName = "X10";
    require(passengerRouteHofRoute(x10, x10Hof) &&
                passengerRouteHofRoute(x10, x10Hof)->code == "510002",
            "line-qualified and extra terminus stops match the full X10 HOF route");
    x10Hof.routes.pop_back();
    require(!passengerRouteMatchesHof(x10, x10Hof),
            "a two-stop shared corridor cannot stand in for a longer HOF itinerary");
    openbus::config::HofData suffixedHof;
    suffixedHof.busstops = {{"Tel_Start", {"Start"}}, {"Tel_Middle1", {"Middle 1"}},
                            {"Tel_Middle2", {"Middle 2"}}, {"Tel_End", {"End"}}};
    suffixedHof.routes = {
        {"4201", 0, false, {"Tel_Start", "Tel_Middle1", "Tel_Middle2", "Tel_End*T13"},
         "Outbound", "XXX"}};
    require(passengerRouteMatchesHof(brtRoute({"Start", "Middle 1", "Middle 2", "End"}),
                                     suffixedHof),
            "HOF stop timing suffixes retain the underlying stop's display aliases");
    std::vector<std::string> longHofStops{"Start"};
    std::vector<std::string> longMapStops{"Start"};
    for (int index = 0; index < 20; ++index) {
        const std::string name = "Common stop" + std::to_string(index);
        longHofStops.push_back(name);
        longMapStops.push_back(name);
    }
    longHofStops.push_back("HOF-only detour");
    longHofStops.push_back("End");
    for (int index = 0; index < 18; ++index)
        longMapStops.push_back("Map-only detour" + std::to_string(index));
    longMapStops.push_back("End");
    openbus::config::HofData longHof;
    longHof.routes = {{"912002", 0, false, longHofStops, "Outbound", "XXX"}};
    require(passengerRouteMatchesHof(brtRoute(longMapStops), longHof),
            "relaxed matching accepts 55 percent ordered coverage of the longer itinerary");
    longMapStops.insert(longMapStops.end() - 1, "Another map-only stop");
    require(!passengerRouteMatchesHof(brtRoute(longMapStops), longHof),
            "relaxed matching rejects longer itineraries below the coverage threshold");
    std::ranges::reverse(hofStops);
    auto wrongDirection = hofRoute;
    wrongDirection.stops = PassengerStops(hofStops);
    require(!passengerRouteMatchesHof(wrongDirection, hof),
            "HOF stop sequence matching distinguishes route direction");
    require(!passengerRouteMatchesHof(hofRoute, {}),
            "an absent or empty HOF does not expose unrelated map routes");
    const std::vector<PassengerStopRouteState> hofRoutes{hofRoute, routes[1]};
    const std::vector<openbus::map::MapAiTimetableCourse> hofCourses{
        {"HOF course", {{"hof-trip", 0, 480.0}, {"return", 0, 520.0}}}};
    PlayerTimetableCatalog catalog;
    require(catalog.update(hofRoutes, hofCourses, hof) && catalog.routes.size() == 2 &&
                catalog.routes[0].status == PassengerHofMatchStatus::Matched &&
                catalog.routes[1].status == PassengerHofMatchStatus::NoMatchingSequence &&
                catalog.lines.size() == 1 && catalog.lines[0].name == "100" &&
                catalog.lines[0].courses.size() == 1 &&
                catalog.lines[0].courses[0].courseIndex == 0 &&
                catalog.lines[0].courses[0].trips.size() == 1 &&
                catalog.lines[0].courses[0].trips[0].tripIndex == 0 &&
                catalog.lines[0].courses[0].trips[0].routeIndex == 0,
            "catalog caches only HOF-compatible course trips");
    require(!catalog.update(hofRoutes, hofCourses, {}) && catalog.lines.size() == 1,
            "repeated frames reuse the catalog without matching the HOF again");
    catalog.invalidate();
    require(catalog.update(hofRoutes, hofCourses, {}) && catalog.lines.empty() &&
                catalog.routes[0].status == PassengerHofMatchStatus::NoMatchingSequence,
            "catalog invalidation rebuilds the results for changed input");
    auto appendedRoute = hofRoute;
    appendedRoute.timetableKey = "second-trip";
    const std::vector<PassengerStopRouteState> expandedRoutes{hofRoute, appendedRoute};
    const std::vector<openbus::map::MapAiTimetableCourse> expandedCourses{
        {"updated", {{"second-trip", 0, 520.0}, {"hof-trip", 0, 480.0}}}};
    catalog.invalidate();
    require(catalog.update(expandedRoutes, expandedCourses, hof) &&
                catalog.lines.size() == 1 && catalog.lines[0].courses[0].trips.size() == 2 &&
                catalog.lines[0].courses[0].trips[0].tripIndex == 1 &&
                catalog.lines[0].courses[0].trips[0].routeIndex == 0 &&
                catalog.lines[0].courses[0].trips[1].tripIndex == 0 &&
                catalog.lines[0].courses[0].trips[1].routeIndex == 1,
            "catalog sorts matched course trips by departure time");
    const auto ambiguousMatch = analyzePassengerRouteHof(tripKeyLine, ambiguousHof);
    require(ambiguousMatch.status == PassengerHofMatchStatus::AmbiguousLines &&
                ambiguousMatch.matchingSequences == 2,
            "diagnostics distinguish ambiguous matching HOF lines");
    auto oneStopRoute = hofRoute;
    oneStopRoute.stops = PassengerStops({hofRoute.stops.front()});
    require(analyzePassengerRouteHof(oneStopRoute, hof).status ==
                PassengerHofMatchStatus::TooFewStops,
            "diagnostics identify routes with too few registered stops");
    controller.select(PlayerTimetableSelection{0, 0, 0, 0, false}, hofRoutes, hofCourses);
    require(!controller.advance(hofRoutes, hofCourses, &hof) && !controller.state,
            "course progression cannot switch to a trip outside the bus's HOF");
}

void verifyPassengerServiceBehavior() {
    using namespace openbus::map;
    MapPassengerQueueIdentities queueIdentities;
    {
        MapTrafficRoute curvedPedestrianRoute;
        MapTrafficRouteSegment curvedPedestrianSegment;
        curvedPedestrianSegment.points = {{0.0, 0.0, 2.0}, {5.0, 5.0, 2.0}, {10.0, 0.0, 2.0}};
        const double curvedPedestrianLength = 2.0 * std::hypot(5.0, 5.0);
        curvedPedestrianRoute.segments.push_back(curvedPedestrianSegment);
        curvedPedestrianRoute.cumulativeEndDistances.push_back(curvedPedestrianLength);
        curvedPedestrianRoute.totalLength = curvedPedestrianLength;
        const MapPassengerPedestrianJourney curvedJourney{
            std::make_shared<const MapTrafficRoute>(curvedPedestrianRoute), 0.0,
            curvedPedestrianLength, {10.0, 0.0, 2.0}};
        MapTrafficPose approachStart;
        approachStart.position = {0.0, 0.0, 2.0};
        MapTrafficPose approachEnd;
        approachEnd.position = {10.0, 0.0, 2.0};
        const auto roadApproach = openbus::app::makePassengerRoadApproach(
            curvedJourney, approachStart, approachEnd);
        require(roadApproach.has_value(),
                "nearby queue and door poses connect to a pedestrian path");
        const auto roadApproachMidpoint = openbus::app::passengerQueueApproachPose(
            roadApproach, approachStart, approachEnd,
            openbus::app::passengerQueueApproachDuration(roadApproach, approachStart,
                                                        approachEnd) *
                0.5);
        require(roadApproachMidpoint.position[1] > 4.9,
                "passenger approach follows the authored pedestrian path instead of crossing it");
        MapTrafficPose smallDoorShift = approachEnd;
        smallDoorShift.position[0] += 0.5;
        require(!openbus::app::passengerApproachTargetChanged(approachEnd, smallDoorShift),
                "small door movements do not restart passenger approaches");
        MapTrafficPose movedDoor = approachEnd;
        movedDoor.position[0] += 1.0;
        require(openbus::app::passengerApproachTargetChanged(approachEnd, movedDoor),
                "material door movement triggers passenger approach replanning");
        movedDoor = approachEnd;
        movedDoor.headingRadians = 0.2;
        require(openbus::app::passengerApproachTargetChanged(approachEnd, movedDoor),
                "material door rotation triggers passenger approach replanning");
        MapTrafficPose unsafeEnd = approachEnd;
        unsafeEnd.position[1] = 3.0;
        require(!openbus::app::makePassengerRoadApproach(curvedJourney, approachStart, unsafeEnd),
                "road approach is rejected when a safe path connector is not nearby");

        MapTrafficLaneNetwork lanes;
        auto spline = makeSpline(9901, 0, 0, 0.0);
        spline.length = 100.0;
        lanes.addSegment({{spline, 0, 0}});
        lanes.finalize();
        const auto journey = lanes.passengerJourneyNear({9901}, {5.0, 0.0, 2.0}, 5);
        require(journey && journey->route && journey->destinationDistanceMeters >
                                                 journey->startDistanceMeters,
                "nearby directed human lane produces a forward pedestrian destination");
        require(std::hypot(journey->destination[0] - 5.0, journey->destination[1]) <= 200.0,
                "generated pedestrian destination is within 200 metres of the stop");
        require(!lanes.passengerJourneyNear({9901}, {500.0, 0.0, 2.0}, 5),
                "a distant pedestrian lane is not treated as nearby");
        MapTrafficLaneNetwork destinationLanes;
        auto nearestLane = makeSpline(9910, 0, 0, 0.0);
        nearestLane.length = 20.0;
        destinationLanes.addSegment({{nearestLane, 0, 0}});
        MapTrafficRouteSegment alternateLane{{makeSpline(9911, 0, 0, 1.5), 0, 0}};
        alternateLane.points = {{5.0, 1.5, 2.0}, {10.0, 6.0, 2.0}, {15.0, 10.0, 2.0}};
        destinationLanes.addSegment(std::move(alternateLane));
        destinationLanes.finalize();
        const auto connectedJourney = destinationLanes.passengerJourneyBetween(
            {9910, 9911}, {5.0, 0.0, 2.0}, {15.0, 10.0, 2.0}, 4);
        require(connectedJourney && connectedJourney->route &&
                    connectedJourney->route->segments.front().spline.placement.splineId == 9911,
                "door-aware pedestrian lookup tries another nearby route when the nearest cannot connect");
        require(std::hypot(connectedJourney->destination[0] - 15.0,
                           connectedJourney->destination[1] - 10.0) <= 2.0,
                "alternate pedestrian route reaches a safe connector near the door");
        MapTrafficPose alternateApproachStart;
        alternateApproachStart.position = {5.0, 0.0, 2.0};
        MapTrafficPose alternateApproachEnd;
        alternateApproachEnd.position = {15.0, 10.0, 2.0};
        const auto alternateApproach = openbus::app::makePassengerRoadApproach(
            *connectedJourney, alternateApproachStart, alternateApproachEnd);
        require(alternateApproach.has_value(),
                "door-aware alternate route builds a complete queue-to-door approach");
        const auto alternateApproachMidpoint = openbus::app::passengerRoadApproachPose(
            *alternateApproach,
            openbus::app::passengerRoadApproachDuration(*alternateApproach) * 0.5);
        require(alternateApproachMidpoint.position[1] > 5.2,
                "alternate queue-to-door movement follows the selected pedestrian lane");
        const auto strictJourney = destinationLanes.passengerJourneyBetween(
            {9910, 9911}, {5.0, 1.1, 2.0}, {15.0, 10.0, 2.0}, 4, 0.5, 50.0);
        require(strictJourney && strictJourney->route &&
                    strictJourney->route->segments.front().spline.placement.splineId == 9911,
                "strict alternate-route search finds a path within the alighting connector limit");
        const auto strictStart = sampleMapTrafficRoute(
            *strictJourney->route, strictJourney->startDistanceMeters, 0.0);
        require(strictStart.has_value(), "strict alternate-route start can be sampled");
        MapTrafficPose alternateBusPose;
        alternateBusPose.position = {5.0, 0.0, 2.0};
        require(openbus::app::makePassengerPedestrianHandoff(
                    *strictJourney, *strictStart, alternateBusPose, false, 22)
                    .has_value(),
                "alternate route passes the existing conservative curb-side handoff checks");
        require(!destinationLanes.passengerJourneyBetween(
                    {9910, 9911}, {5.0, 0.0, 2.0}, {15.0, 10.0, 2.0}, 4, 0.5, 50.0),
                "strict handoff lookup rejects a connector outside its safety limit");
        require(!destinationLanes.passengerJourneyBetween(
                    {9910, 9911}, {5.0, 0.0, 2.0}, {15.0, 15.0, 2.0}, 4),
                "door-aware lookup rejects routes without a safe destination connector");
        const auto laneStart = sampleMapTrafficRoute(*journey->route, journey->startDistanceMeters, 0.0);
        require(laneStart.has_value(), "pedestrian handoff start can be sampled");
        const auto side = mapTrafficPassengerWorldOffset(false, 0.0, 1.0, 0.0);
        MapTrafficPose busPose = *laneStart;
        MapTrafficPose exitPose = *laneStart;
        for (std::size_t axis = 0; axis < 2; ++axis) {
            busPose.position[axis] -= 2.0 * side[axis];
            exitPose.position[axis] -= 0.25 * side[axis];
        }
        auto handoff = openbus::app::makePassengerPedestrianHandoff(*journey, exitPose, busPose,
                                                                    false, 17);
        require(handoff && handoff->modelIndex == 17,
                "nearby forward pedestrian route accepts the alighter and preserves its model");
        const auto initialPose = openbus::app::passengerPedestrianHandoffPose(*handoff);
        requireNear(initialPose.position[0], exitPose.position[0],
                    "handoff begins at the rendered exit position");
        requireNear(initialPose.position[1], exitPose.position[1],
                    "handoff begins at the rendered exit position");
        require(openbus::app::advancePassengerPedestrianHandoff(*handoff, 1.0),
                "handoff advances onto the pedestrian route");
        const auto walkingPose = openbus::app::passengerPedestrianHandoffPose(*handoff);
        require(std::hypot(walkingPose.position[0] - exitPose.position[0],
                           walkingPose.position[1] - exitPose.position[1]) > 0.25,
                "handoff continues forward after the connector");
        require(!openbus::app::advancePassengerPedestrianHandoff(*handoff, 100.0),
                "handoff completes at its destination");
        const auto longJourney = lanes.passengerJourneyNear({9901}, {5.0, 0.0, 2.0}, 30);
        require(longJourney.has_value(), "long handoff fixture has a pedestrian route");
        const auto fartherLanePose = sampleMapTrafficRoute(
            *longJourney->route, longJourney->startDistanceMeters + 30.0, 0.0);
        require(fartherLanePose.has_value(), "farther handoff point can be sampled");
        auto fartherBusPose = *fartherLanePose;
        auto fartherExitPose = *fartherLanePose;
        for (std::size_t axis = 0; axis < 2; ++axis) {
            fartherBusPose.position[axis] -= 2.0 * side[axis];
            fartherExitPose.position[axis] -= 0.25 * side[axis];
        }
        require(openbus::app::makePassengerPedestrianHandoff(
                    *longJourney, fartherExitPose, fartherBusPose, false, 18)
                    .has_value(),
                "handoff can use a farther point along the assigned sidewalk route");
        auto unsafeExit = exitPose;
        unsafeExit.position[2] += 1.0;
        require(!openbus::app::makePassengerPedestrianHandoff(*journey, unsafeExit, busPose,
                                                               false, 17),
                "handoff rejects vertically separated paths");
        unsafeExit = exitPose;
        for (std::size_t axis = 0; axis < 2; ++axis) {
            unsafeExit.position[axis] += side[axis] * 0.75;
        }
        require(!openbus::app::makePassengerPedestrianHandoff(*journey, unsafeExit, busPose,
                                                               false, 17),
                "handoff rejects connectors toward the bus");
        unsafeExit = exitPose;
        unsafeExit.position[1] += 40.0;
        require(!openbus::app::makePassengerPedestrianHandoff(*journey, unsafeExit, busPose,
                                                               false, 17),
                "handoff rejects distant paths and points beyond the destination");
        auto missingRoute = *journey;
        missingRoute.route.reset();
        require(!openbus::app::makePassengerPedestrianHandoff(missingRoute, exitPose, busPose,
                                                               false, 17),
                "missing pedestrian routes do not create a handoff");
        MapPassengerServiceRuntime loadingService(1);
        require(loadingService.arrive({.stopId = 100, .waitingPassengers = 1,
                                       .dwellSeconds = 0.1, .nextStopId = 200}),
                "destination-loading fixture starts dwelling");
        loadingService.advance(1.0, false, true, true, false, nullptr, false, nullptr, false, true);
        require(loadingService.dwelling(),
                "a held service does not depart while a destination is being prepared");
        loadingService.advance(0.1, true, true);
        require(loadingService.boardingPassenger(),
                "destination preparation preserves enough dwell to start boarding");
        loadingService.advance(1.0, true, true);
        require(!loadingService.dwelling(),
                "service can depart once the destination preparation hold is released");
        openbus::app::PassengerNetwork network;
        openbus::app::PassengerStopRouteState route;
        route.timetableKey = "walk";
        route.stops = openbus::app::PassengerStops(
            {{.stopId = 100, .waitingPassengers = 1}, {.stopId = 200}});
        route.stopPoses = {MapTrafficPose{}, MapTrafficPose{}};
        network.routes.push_back(route);
        network.initializeQueues(route.stops.values());
        MapPassengerServiceRuntime service(1);
        require(service.arrive({.stopId = 100, .dwellSeconds = 5.0, .nextStopId = 200}),
                "pedestrian destination fixture starts boarding");
        const auto passenger = network.nextPassenger(100, 0, service);
        require(passenger.has_value(), "pedestrian destination fixture has a waiting rider");
        network.resolvePassengerPedestrianJourney(100, *passenger, journey);
        network.advanceService(service, 0, passenger, 0.1, true, true, true);
        network.departService(service);
        require(network.waitingPassengers(100)[0].journey->pedestrianJourney.has_value(),
                "cancellation retains the rider's pedestrian destination");
        require(service.arrive({.stopId = 100, .dwellSeconds = 5.0, .nextStopId = 200}),
                "the rider can reboard after cancellation");
        network.advanceService(service, 0, network.nextPassenger(100, 0, service), 0.1, true,
                               true, true);
        service.setPassengerAnimationDuration(0.75);
        network.advanceService(service, 0, std::nullopt, 0.75, true, true, true);
        require(service.passengerPedestrianJourneyOnSeat(0).has_value(),
                "the pedestrian destination follows the rider to their seat");
        const auto destination = journey->destination;
        service.requestStopForPassengers(
            200, {destination[0] - 200.01, destination[1], destination[2]}, {5.0, 0.0, 2.0});
        require(service.passengerStopRequestSequence() == 0,
                "a destination just beyond 200 metres does not request the stop");
        service.requestStopForPassengers(
            200, {destination[0] - 200.0, destination[1], destination[2]}, {5.0, 0.0, 2.0});
        require(service.passengerStopRequestSequence() == 1,
                "the 200-metre request uses the pedestrian destination");
        service.depart();
        require(service.arrive({.stopId = 200, .dwellSeconds = 1.0, .nextStopId = 300,
                                .alightingPositions = {{destination[0] - 200.01,
                                                        destination[1], destination[2]}}}) &&
                    !service.alightingPassenger() &&
                    service.outOfRangeAssignedPassengers() == 1,
                "a repeated stop ID beyond 200 metres does not unload the rider");
        service.depart();
        service.rearmPassengerStopRequestsAfterStop(200);
        service.requestStopForPassengers(
            200, {destination[0] - 200.0, destination[1], destination[2]}, {5.0, 0.0, 2.0});
        require(service.passengerStopRequestSequence() == 2,
                "a retained rider can request the later call with the same stop ID");
        require(service.arrive({.stopId = 200, .dwellSeconds = 1.0, .nextStopId = 300,
                                .alightingPositions = {{destination[0] - 200.0,
                                                        destination[1], destination[2]}}}) &&
                    service.alightingPassenger(),
                "a rider may alight when the exit is exactly 200 metres from the destination");
        require(service.canCurrentPassengerAlightAt(
                    {destination[0] - 200.0, destination[1], destination[2]}) &&
                    !service.canCurrentPassengerAlightAt(
                        {destination[0] - 200.01, destination[1], destination[2]}),
                "an alighter waits if the bus drifts beyond the destination radius");
        service.setPassengerAnimationDuration(1.0);
        service.setAlightingDoorWaitProgress(0.5);
        service.advance(1.0, true,
                        service.canCurrentPassengerAlightAt(
                            {destination[0] - 200.01, destination[1], destination[2]}));
        require(service.alightingPassenger() && service.alightingProgress() == 0.5 &&
                    service.passengerCountOnSeat(0) == 1,
                "drift blocks the alighter at the door without removing the rider");
        service.advance(0.5, true,
                        service.canCurrentPassengerAlightAt(
                            {destination[0] - 200.0, destination[1], destination[2]}));
        require(!service.alightingPassenger() && service.passengerCountOnSeat(0) == 0,
                "alighting resumes when the exit returns within 200 metres");
    }
    {
        openbus::app::PassengerNetwork network;
        openbus::app::PassengerStopRouteState route;
        route.timetableKey = "fallback-position";
        route.stops = openbus::app::PassengerStops(
            {{.stopId = 101, .waitingPassengers = 1}, {.stopId = 201}});
        MapTrafficPose destinationPose;
        destinationPose.position = {1000.0, 0.0, 2.0};
        route.stopPoses = {MapTrafficPose{}, destinationPose};
        network.routes.push_back(route);
        network.initializeQueues(route.stops.values());
        MapPassengerServiceRuntime service(1);
        require(service.arrive({.stopId = 101, .dwellSeconds = 5.0, .nextStopId = 201}),
                "mapped destination fixture starts boarding");
        const auto rider = network.nextPassenger(101, 0, service);
        require(rider.has_value(), "mapped destination fixture selects a rider");
        network.resolvePassengerPedestrianJourney(101, *rider, std::nullopt);
        network.advanceService(service, 0, rider, 0.1, true, true, true);
        service.setPassengerAnimationDuration(0.75);
        network.advanceService(service, 0, std::nullopt, 0.75, true, true, true);
        service.requestStopForPassengers(201, {0.0, 0.0, 2.0}, {0.0, 0.0, 2.0});
        require(service.passengerStopRequestSequence() == 0,
                "fallback requests use the rider's original stop position, not a repeated ID");
        service.requestStopForPassengers(201, {800.0, 0.0, 2.0}, {0.0, 0.0, 2.0});
        require(service.passengerStopRequestSequence() == 1,
                "fallback requests trigger at the original position's 200-metre boundary");
        service.depart();
        require(service.arrive({.stopId = 201, .dwellSeconds = 1.0, .nextStopId = 301,
                                .alightingPositions = {{{0.0, 0.0, 2.0}}}}) &&
                    !service.alightingPassenger(),
                "mapped-stop fallback retains its original position for repeated stop IDs");
        service.depart();
        require(service.arrive({.stopId = 201, .passengersAlighting = 1,
                                .dwellSeconds = 1.0, .nextStopId = 301,
                                .alightingPositions = {{{0.0, 0.0, 2.0}}}}) &&
                    service.alightingPassenger() &&
                    service.canCurrentPassengerAlightAt({0.0, 0.0, 2.0}),
                "explicit forced unloading retains its existing distance override");
        service.depart();
        require(service.arrive({.stopId = 201, .dwellSeconds = 1.0, .nextStopId = 301,
                                .alightingPositions = {{{800.0, 0.0, 2.0}}}}) &&
                    service.alightingPassenger(),
                "mapped-stop fallback allows alighting at the 200-metre boundary");
    }
    {
        MapPassengerServiceRuntime service(1);
        require(service.arrive({.stopId = 101, .waitingPassengers = 1, .dwellSeconds = 5.0,
                                .nextStopId = 201}),
                "cancelled-service fixture starts boarding");
        const openbus::map::MapPassengerBoardingSelection passenger{
            .modelIndex = 0,
            .queueSlot = 0,
            .destinationStopId = 201,
            .destinationPosition = std::array<double, 3>{1000.0, 0.0, 2.0}};
        service.advance(0.1, true, true, true, false, nullptr, true, &passenger, true);
        service.setPassengerAnimationDuration(0.75);
        service.advance(0.75, true, true, true, false, nullptr, true, nullptr, true);
        require(service.onboardPassengerCount() == 1,
                "cancelled-service fixture boards a rider with a distant destination");
        service.depart();
        require(service.arrive({.stopId = 201, .dwellSeconds = 1.0, .nextStopId = 301,
                                .alightingPositions = {{{0.0, 0.0, 2.0}}}}) &&
                    !service.alightingPassenger(),
                "normal service does not unload the rider outside the destination radius");
        service.endServiceAtCurrentStop();
        require(service.alightingPassenger() &&
                    service.canCurrentPassengerAlightAt({0.0, 0.0, 2.0}),
                "cancelled service forces alighting beyond the normal destination radius");
        service.advance(0.75, true, true);
        require(service.onboardPassengerCount() == 0,
                "cancelled-service forced alighting completes");
    }
    {
        openbus::app::PassengerNetwork network;
        openbus::app::PassengerStopRouteState route;
        route.timetableKey = "unlocated";
        route.stops =
            openbus::app::PassengerStops({{.stopId = 90, .waitingPassengers = 1}, {.stopId = 91}});
        route.stopPoses = {MapTrafficPose{}, std::nullopt};
        network.routes.push_back(route);
        network.initializeQueues(route.stops.values());
        MapPassengerServiceRuntime service(1);
        require(service.arrive({.stopId = 90, .dwellSeconds = 5.0, .nextStopId = 91}) &&
                    !network.waitingPassengers(90)[0].journey &&
                    !network.nextPassenger(90, 0, service),
                "an unlocated destination does not create a fabricated journey");
        network.routes.push_back(route);
        network.routes.back().stopPoses[1] = MapTrafficPose{};
        require(network.waitingPassengers(90)[0].journey &&
                    network.waitingPassengers(90)[0].journey->routeIndex == 1,
                "unassigned riders can choose a newly registered located route");
    }
    {
        openbus::app::PassengerNetwork network;
        openbus::app::PassengerStopRouteState route;
        route.timetableKey = "repeated-calls";
        route.stops = openbus::app::PassengerStops(
            {{.stopId = 100, .waitingPassengers = 4, .timetableStationIndex = 0},
             {.stopId = 200, .timetableStationIndex = 1},
             {.stopId = 100, .timetableStationIndex = 2},
             {.stopId = 200, .timetableStationIndex = 3}});
        route.stopPoses = {MapTrafficPose{{0.0, 0.0, 0.0}, 0.0},
                           MapTrafficPose{{10.0, 0.0, 0.0}, 0.0},
                           MapTrafficPose{{20.0, 0.0, 0.0}, 0.0},
                           MapTrafficPose{{30.0, 0.0, 0.0}, 0.0}};
        network.routes.push_back(route);
        network.initializeQueues(route.stops.values());
        const openbus::app::PassengerStops sparseBusStops(
            {route.stops.values()[0], route.stops.values()[2], route.stops.values()[3]});
        require(network.routeCallIndexForBusStop(0, sparseBusStops, 1) == 2 &&
                    network.routeCallIndexForBusStop(0, sparseBusStops, 2) == 3,
                "scheduled call indices map through preserved timetable station indices");
        const auto& passengers = network.waitingPassengers(100);
        require(std::ranges::any_of(passengers, [](const auto& passenger) {
                    return passenger.journey && passenger.journey->originStopIndex == 0 &&
                           passenger.journey->destinationStopIndex == 1;
                }) &&
                    std::ranges::any_of(passengers, [](const auto& passenger) {
                        return passenger.journey && passenger.journey->originStopIndex == 0 &&
                               passenger.journey->destinationStopIndex == 3;
                    }) &&
                    std::ranges::any_of(passengers, [](const auto& passenger) {
                        return passenger.journey && passenger.journey->originStopIndex == 2 &&
                               passenger.journey->destinationStopIndex == 3;
                    }),
                "repeated stop IDs retain separate origin and destination call indices");
        MapPassengerServiceRuntime firstCall(1);
        MapPassengerServiceRuntime repeatedCall(1);
        require(firstCall.arrive({.stopId = 100, .dwellSeconds = 5.0, .nextStopId = 200}) &&
                    repeatedCall.arrive(
                        {.stopId = 100, .dwellSeconds = 5.0, .nextStopId = 200}),
                "both occurrences of a repeated stop can begin service");
        const auto firstPassenger = network.nextPassenger(100, 0, firstCall, 0);
        const auto repeatedPassenger = network.nextPassenger(100, 0, repeatedCall, 2);
        require(firstPassenger &&
                    passengers[*firstPassenger].journey->originStopIndex == 0 &&
                    repeatedPassenger &&
                    passengers[*repeatedPassenger].journey->originStopIndex == 2 &&
                    network.passengerMatchesService(passengers[*firstPassenger], 0, firstCall, 0) &&
                    network.passengerMatchesService(passengers[*repeatedPassenger], 0,
                                                    repeatedCall, 2),
                "boarding selects riders for the exact repeated origin call");
    }
    {
        openbus::app::PassengerNetwork network;
        openbus::app::PassengerStopRouteState manyDestinations;
        manyDestinations.timetableKey = "many-destinations";
        manyDestinations.stops = openbus::app::PassengerStops(
            {{.stopId = 100, .waitingPassengers = 8},
             {.stopId = 201},
             {.stopId = 202},
             {.stopId = 203},
             {.stopId = 204}});
        manyDestinations.stopPoses.assign(5, MapTrafficPose{});
        openbus::app::PassengerStopRouteState oneDestination;
        oneDestination.timetableKey = "one-destination";
        oneDestination.stops = openbus::app::PassengerStops(
            {{.stopId = 100, .waitingPassengers = 8}, {.stopId = 300}});
        oneDestination.stopPoses.assign(2, MapTrafficPose{});
        network.routes.push_back(std::move(manyDestinations));
        network.routes.push_back(std::move(oneDestination));
        network.initializeQueues(network.routes[0].stops.values());
        const auto& passengers = network.waitingPassengers(100);
        const auto manyDestinationRiders = std::ranges::count_if(
            passengers, [](const auto& passenger) {
                return passenger.journey && passenger.journey->routeIndex == 0;
            });
        const auto oneDestinationRiders = std::ranges::count_if(
            passengers, [](const auto& passenger) {
                return passenger.journey && passenger.journey->routeIndex == 1;
            });
        require(manyDestinationRiders == 4 && oneDestinationRiders == 4,
                "shared-stop riders are balanced across service routes independent of destination count");
    }
    {
        openbus::app::PassengerNetwork network;
        const auto route = [](const std::string& key, int destination) {
            openbus::app::PassengerStopRouteState result;
            result.timetableKey = key;
            result.stops = openbus::app::PassengerStops(
                {{.stopId = 100, .waitingPassengers = 4}, {.stopId = destination}});
            result.stopPoses = {MapTrafficPose{}, MapTrafficPose{}};
            return result;
        };
        network.routes.push_back(route("line-a", 200));
        network.routes.push_back(route("line-b", 300));
        network.initializeQueues(network.routes[0].stops.values());
        const auto original = network.waitingPassengers(100);
        require(original.size() == 4 && original[2].journey->routeIndex == 0 &&
                    original[2].queueSlot == 1 && original[3].journey->routeIndex == 1 &&
                    original[3].queueSlot == 0,
                "waiting riders have persistent service and destination choices");

        MapPassengerServiceRuntime busA(1);
        MapPassengerServiceRuntime busB(1);
        require(busA.arrive({.stopId = 100, .dwellSeconds = 5.0, .nextStopId = 200}) &&
                    busB.arrive({.stopId = 100, .dwellSeconds = 5.0, .nextStopId = 300}),
                "different services arrive at the shared queue");
        const auto indexA = network.nextPassenger(100, 0, busA);
        require(indexA == 2, "a matching rider can board past a rider for another service");
        network.advanceService(busA, 0, indexA, 0.1, true, true, true);
        const auto indexB = network.nextPassenger(100, 1, busB);
        require(indexB && network.waitingPassengers(100)[*indexB].modelIndex == 103,
                "the first bus leaves the other service's front rider in place");
        network.advanceService(busB, 1, indexB, 0.1, true, true, true);
        require(network.queues.at(100) == 2 && busA.boardingPassengerModelIndex() == 102 &&
                    busA.boardingPassengerQueueSlot(4) == 1 &&
                    busA.boardingPassengerDestinationStopId() == 200 &&
                    busB.boardingPassengerModelIndex() == 103 &&
                    busB.boardingPassengerQueueSlot(4) == 0 &&
                    busB.boardingPassengerDestinationStopId() == 300 &&
                    network.waitingPassengers(100)[0].queueSlot == 3 &&
                    network.waitingPassengers(100)[1].queueSlot == 2,
                "concurrent service reservations preserve identities, destinations and slots");
        network.departService(busA);
        network.endService(busB);
        require(network.queues.at(100) == 4, "cancelled reservations return to the shared queue");
        network.routes.push_back(route("line-c", 400));
        const auto& returned = network.waitingPassengers(100);
        const auto riderA = std::ranges::find(returned, std::size_t{102},
                                              &openbus::app::WaitingPassenger::modelIndex);
        const auto riderB = std::ranges::find(returned, std::size_t{103},
                                              &openbus::app::WaitingPassenger::modelIndex);
        require(riderA != returned.end() && riderB != returned.end() && riderA->queueSlot == 1 &&
                    riderA->journey->routeIndex == 0 && riderA->journey->destinationStopId == 200 &&
                    riderB->queueSlot == 0 && riderB->journey->routeIndex == 1 &&
                    riderB->journey->destinationStopId == 300,
                "returned journeys and positions do not change when routes are added");
        MapPassengerServiceRuntime unrelated(1);
        require(unrelated.arrive({.stopId = 100, .dwellSeconds = 5.0, .nextStopId = 400}),
                "newly registered service can reach the same stop");
        require(!network.nextPassenger(100, 2, unrelated),
                "existing riders do not switch to a newly registered service");
        network.advanceService(unrelated, 2, std::nullopt, 0.1, true, true, true);
        require(!unrelated.boardingPassenger() && unrelated.eligibleWaitingPassengers() == 0 &&
                    !openbus::app::vehicle::passengerServiceRequestVariables(0, 0, unrelated)
                         .entries[0],
                "unmatched passengers neither board nor request the entry door");
        network.routes.push_back(route("line-a", 400));
        require(!network.nextPassenger(100, 3, unrelated),
                "a same-key route variant cannot board a rider for an unavailable destination");
        auto otherLine = route("line-a", 200);
        otherLine.lineName = "another line";
        network.routes.push_back(std::move(otherLine));
        MapPassengerServiceRuntime otherLineBus(1);
        require(otherLineBus.arrive({.stopId = 100, .dwellSeconds = 5.0, .nextStopId = 200}) &&
                    !network.nextPassenger(100, 4, otherLineBus),
                "a shared timetable key does not make a different line the expected service");
        auto distantVariant = route("line-a", 200);
        distantVariant.stopPoses[1]->position[0] = 1000.0;
        network.routes.push_back(std::move(distantVariant));
        require(!network.passengerMatchesService(*riderA, 5, otherLineBus),
                "a same-key service cannot board a rider for a distant stop with the same ID");
        network.routes.push_back(route("line-a", 200));
        MapPassengerServiceRuntime sameCourseDifferentRoute(1);
        require(sameCourseDifferentRoute.arrive(
                    {.stopId = 100, .dwellSeconds = 5.0, .nextStopId = 200}) &&
                    !network.nextPassenger(100, 6, sameCourseDifferentRoute),
                "a route variant cannot board riders assigned to another route identity");
        MapPassengerServiceRuntime replacement(1);
        require(replacement.arrive({.stopId = 100, .dwellSeconds = 5.0, .nextStopId = 200}),
                "replacement bus can serve returned passengers");
        const auto replacementIndex = network.nextPassenger(100, 0, replacement);
        require(replacementIndex &&
                    network.waitingPassengers(100)[*replacementIndex].modelIndex == 102,
                "the same service selects its oldest returned rider");
        network.advanceService(replacement, 0, replacementIndex, 0.1, true, true, true);
        replacement.setPassengerAnimationDuration(0.75);
        network.advanceService(replacement, 0, std::nullopt, 0.75, true, true, true);
        require(replacement.passengerModelIndexOnSeat(0) == 102 && network.queues.at(100) == 3,
                "the reserved passenger keeps their model through seating");
        network.queues.at(100) = 4;
        const auto& replenished = network.waitingPassengers(100);
        std::unordered_set<std::size_t> occupiedSlots;
        for (const auto& passenger : replenished) {
            occupiedSlots.insert(passenger.queueSlot);
        }
        require(replenished.size() == 4 && occupiedSlots.size() == 4 &&
                    std::ranges::any_of(replenished,
                                        [](const auto& passenger) {
                                            return passenger.modelIndex == 104 &&
                                                   passenger.journey.has_value();
                                        }),
                "replenishment creates a new journey without overlapping waiting slots");
        require(replacement.arrive({.stopId = 200, .dwellSeconds = 1.0}) == false,
                "a held service cannot arrive at another stop");
    }
    {
        openbus::app::PassengerNetwork network;
        openbus::app::PassengerStopRouteState route;
        route.timetableKey = "single";
        route.stops = openbus::app::PassengerStops(
            {{.stopId = 100, .waitingPassengers = 1}, {.stopId = 200}});
        route.stopPoses = {MapTrafficPose{}, MapTrafficPose{}};
        network.routes.push_back(route);
        network.initializeQueues(route.stops.values());
        MapPassengerServiceRuntime first(1);
        require(first.arrive({.stopId = 100, .dwellSeconds = 5.0, .nextStopId = 200}),
                "replenishment fixture starts at a stop");
        network.advanceService(first, 0, network.nextPassenger(100, 0, first), 0.1, true, true,
                               true);
        network.queues.at(100) = 1;
        require(network.waitingPassengers(100).empty(),
                "new demand waits for a slot reserved by a boarding passenger");
        network.departService(first);
        require(network.queues.at(100) == 2 && network.waitingPassengers(100).size() == 1 &&
                    network.waitingPassengers(100).front().modelIndex == 100,
                "a cancelled reservation returns without overlapping replenished demand");
        MapPassengerServiceRuntime second(1);
        require(second.arrive({.stopId = 100, .dwellSeconds = 5.0, .nextStopId = 200}),
                "a replacement bus serves the restored rider");
        network.advanceService(second, 0, network.nextPassenger(100, 0, second), 0.1, true, true,
                               true);
        second.setPassengerAnimationDuration(0.75);
        network.advanceService(second, 0, std::nullopt, 0.75, true, true, true);
        require(network.queues.at(100) == 1 && network.waitingPassengers(100).size() == 1 &&
                    network.waitingPassengers(100).front().modelIndex == 101 &&
                    network.waitingPassengers(100).front().queueSlot == 0,
                "deferred demand enters the free slot after the rider boards");
    }
    {
        openbus::app::PassengerStops stops({{.stopId = 100},
                                            {.stopId = 200},
                                            {.stopId = 200},
                                            {.stopId = 300},
                                            {.stopId = 100},
                                            {.stopId = 400}});
        const auto destinations = openbus::app::onwardPassengerStopIds(
            stops, 1, 100, [](std::size_t index) { return index != 3; });
        require(destinations == std::vector<int>({200, 400}),
                "onward destinations exclude unavailable, repeated and origin stop IDs");
        openbus::app::ScheduledBusState scheduled;
        scheduled.passengerStops = stops;
        MapPassengerServiceRuntime emptyService(0);
        const auto scheduledStop = openbus::app::scheduledBusStopService(scheduled, emptyService);
        require(scheduledStop.nextStopId == 200 &&
                    scheduledStop.onwardStopIds == std::vector<int>({200, 300, 400}),
                "scheduled buses expose all distinct downstream destination stops");
        MapPassengerServiceRuntime service(3);
        service.setWaitingPassengers(100, 3);
        require(service.arrive({.stopId = 100,
                                .dwellSeconds = 3.0,
                                .nextStopId = 200,
                                .onwardStopIds = {200, 300, 400}}),
                "multiple-destination fixture starts boarding");
        service.advance(2.25, true, true, true);
        require(service.onboardPassengerCount() == 3,
                "multiple-destination fixture boards three passengers");
        service.requestStopForPassengers(200, 200.01);
        require(service.passengerStopRequestSequence() == 0,
                "passengers do not request their destinations beyond 200 metres");
        service.requestStopForPassengers(200, 200.0);
        require(service.passengerStopRequestSequence() == 1,
                "only the passenger assigned to the approaching stop requests it");
        service.depart();
        require(service.arrive({.stopId = 200, .dwellSeconds = 1.0, .nextStopId = 300}),
                "multiple-destination fixture reaches its first destination");
        require(service.alightingSeatIndex() == 0,
                "only the rider assigned to the first stop alights");
        service.advance(0.75, false, true, true);
        require(service.onboardPassengerCount() == 2, "riders for later stops remain on board");
        service.depart();
        require(service.arrive({.stopId = 300, .dwellSeconds = 1.0, .nextStopId = 400}) &&
                    service.alightingSeatIndex() == 2,
                "the next assigned rider alights at the second destination");
        service.advance(0.75, false, true, true);
        service.depart();
        require(service.arrive({.stopId = 400, .dwellSeconds = 1.0}) &&
                    service.alightingSeatIndex() == 1,
                "the last assigned rider continues to the third destination");
        service.advance(0.75, false, true, true);
        require(service.onboardPassengerCount() == 0,
                "all riders alight at their assigned destinations");
    }
    {
        const openbus::app::PassengerStops stops(
            {{.stopId = 100}, {.stopId = 200}, {.stopId = 300}, {.stopId = 400}});
        MapPassengerServiceRuntime service(3);
        service.setWaitingPassengers(100, 3);
        require(service.arrive({.stopId = 100,
                                .dwellSeconds = 3.0,
                                .nextStopId = 200,
                                .onwardStopIds = {200, 300, 400}}),
                "future-stop request fixture starts boarding");
        service.advance(2.25, true, true, true);
        service.depart();
        const auto position = [](std::size_t index) -> std::optional<std::array<double, 3>> {
            if (index == 1) {
                return std::array<double, 3>{250.0, 0.0, 0.0};
            }
            if (index == 3) {
                return std::array<double, 3>{200.0, 0.0, 0.0};
            }
            return std::nullopt;
        };
        openbus::app::requestPassengerStopsOnRoute(service, stops, 1, {0.0, 0.0, 0.0}, position);
        require(service.passengerStopRequestSequence() == 1 &&
                    !service.hasUnrequestedPassengerStop(400) &&
                    service.hasUnrequestedPassengerStop(200) &&
                    service.hasUnrequestedPassengerStop(300),
                "a later stop at 200 metres is requested despite an intervening stop");
        openbus::app::requestPassengerStopsOnRoute(service, stops, 2, {50.0, 0.0, 0.0}, position);
        require(service.passengerStopRequestSequence() == 1,
                "past stops and missing positions do not generate requests");
        openbus::app::requestPassengerStopsOnRoute(service, stops, 1, {49.99, 0.0, 0.0}, position);
        require(service.passengerStopRequestSequence() == 1,
                "an unrequested stop just over 200 metres away remains pending");
        openbus::app::requestPassengerStopsOnRoute(service, stops, 1, {50.0, 0.0, 0.0}, position);
        openbus::app::requestPassengerStopsOnRoute(service, stops, 1, {50.0, 0.0, 0.0}, position);
        require(service.passengerStopRequestSequence() == 2,
                "each rider requests their stop only once at the 200-metre boundary");
        service.rearmPassengerStopRequestsAfterStop(200);
        require(service.hasUnrequestedPassengerStop(400) &&
                    !service.hasUnrequestedPassengerStop(200),
                "an intermediate stop rearms later riders without rearming its own riders");
        openbus::app::requestPassengerStopsOnRoute(service, stops, 2, {50.0, 0.0, 0.0}, position);
        openbus::app::requestPassengerStopsOnRoute(service, stops, 2, {50.0, 0.0, 0.0}, position);
        require(service.passengerStopRequestSequence() == 3,
                "a later rider can request again after the intermediate stop resets the bell");
    }
    {
        MapPassengerQueueIdentities identities;
        MapPassengerServiceRuntime service(1);
        std::size_t waiting = 2;
        const MapPassengerStopService stop{.stopId = 720,
                                           .dwellSeconds = 2.0,
                                           .nextStopId = 721,
                                           .onwardStopIds = {721, 722, 723}};
        require(service.arrive(stop), "returning rider fixture starts boarding");
        advanceMapPassengerServiceWithSharedQueue(service, 720, waiting, identities, 0.25);
        const auto identity = service.boardingPassengerModelIndex();
        const auto destination = service.boardingPassengerDestinationStopId();
        require(identity && destination, "a reserved rider has a model and destination");
        departMapPassengerServiceWithSharedQueue(service, waiting, identities);
        require(waiting == 2 && service.arrive(stop), "an interrupted rider returns to the queue");
        advanceMapPassengerServiceWithSharedQueue(service, 720, waiting, identities, 0.25);
        require(service.boardingPassengerModelIndex() == identity &&
                    service.boardingPassengerDestinationStopId() == destination,
                "reboarding on the same route preserves the rider's destination");

        for (const std::vector<int>& destinations :
             std::vector<std::vector<int>>{{721, 721}, {721, 720}, {721, 0}, {722, 721}}) {
            MapPassengerServiceRuntime invalid(1);
            bool rejected = false;
            try {
                invalid.arrive({.stopId = 720,
                                .dwellSeconds = 1.0,
                                .nextStopId = 721,
                                .onwardStopIds = destinations});
            } catch (const std::invalid_argument&) {
                rejected = true;
            }
            require(rejected && !invalid.dwelling(),
                    "invalid destination lists are rejected before the stop state changes");
        }
    }
    {
        require(openbus::app::playerPassengerStopArrivalRadiusMeters == 20.0 &&
                    openbus::app::playerPassengerStopReleaseRadiusMeters >
                        openbus::app::playerPassengerStopArrivalRadiusMeters,
                "player stop allows a 20-metre approach with a larger release radius");
        const auto floor = openbus::app::passengerCabinStandingPoint(
            {-0.8875, 4.3534, 3.0557}, 0.45);
        const auto modelPoint = openbus::app::passengerCabinModelPoint(floor);
        requireNear(modelPoint[0], 4.3534, "seat forward coordinate follows the vehicle mesh");
        requireNear(modelPoint[1], 0.8875, "seat lateral coordinate follows the vehicle mesh");
        requireNear(modelPoint[2], 2.6057, "standing model feet are at the cabin floor");
        requireNear(openbus::app::passengerCabinHeading(std::numbers::pi / 2.0, 0.0),
                    std::numbers::pi / 2.0, "forward-facing seats follow the bus heading");
        requireNear(openbus::app::passengerCabinHeading(0.0, 180.0), std::numbers::pi,
                    "rear-facing seats turn passengers around");
        const auto boardingWalk = openbus::app::passengerSeatTransition(0.4, true, true);
        const auto boardingSit = openbus::app::passengerSeatTransition(0.9, true, true);
        const auto alightingStand = openbus::app::passengerSeatTransition(0.1, false, true);
        const auto alightingWalk = openbus::app::passengerSeatTransition(0.6, false, true);
        const auto standingWalk = openbus::app::passengerSeatTransition(0.5, true, false);
        requireNear(boardingWalk.pathProgress, 0.5, "boarding walks before sitting");
        requireNear(boardingWalk.seatedFraction, 0.0, "boarding does not sit in the aisle");
        requireNear(boardingSit.pathProgress, 1.0, "boarding reaches its seat before sitting");
        requireNear(boardingSit.seatedFraction, 0.5, "boarding blends into a seated pose");
        requireNear(alightingStand.pathProgress, 0.0, "alighting stands up before walking");
        requireNear(alightingStand.seatedFraction, 0.5, "alighting blends out of sitting");
        requireNear(alightingWalk.pathProgress, 0.5, "alighting walks after standing up");
        requireNear(standingWalk.pathProgress, 0.5, "standing positions retain the walking timing");
        requireNear(standingWalk.seatedFraction, 0.0, "standing positions never sit down");
        const std::vector<std::pair<double, std::optional<std::size_t>>> links{
            {0.5, std::size_t{2}}, {1.0, std::size_t{3}}};
        const auto boardingLinks = openbus::app::passengerSeatPathLinkProfile(links, true, true);
        const auto alightingLinks = openbus::app::passengerSeatPathLinkProfile(links, false, true);
        require(boardingLinks.size() == 3 && alightingLinks.size() == 3 &&
                    !boardingLinks.back().second && !alightingLinks.front().second,
                "seating transitions do not claim a cabin path link");
        requireNear(boardingLinks[0].first, 0.4, "boarding link timing follows walking progress");
        requireNear(alightingLinks[1].first, 0.6, "alighting link timing follows walking progress");
        require(openbus::app::passengerSeatPathLinkProfile(links, true, false) == links,
                "standing positions retain the original path timing");
        require(openbus::app::passengerCoinVisualBurstCount(0.0, 0.0) == 0 &&
                    openbus::app::passengerCoinVisualBurstCount(2.0, 5.0) == 3 &&
                    openbus::app::passengerCoinVisualBurstCount(5.0, 2.0) == 0 &&
                    openbus::app::passengerCoinVisualBurstCount(0.0, 20.0) == 4,
                "coin visuals track increments, resets, and a bounded burst");
        bool rejectedInvalidCoinCount = false;
        try {
            (void)openbus::app::passengerCoinVisualBurstCount(
                0.0, std::numeric_limits<double>::quiet_NaN());
        } catch (const std::invalid_argument&) {
            rejectedInvalidCoinCount = true;
        }
        require(rejectedInvalidCoinCount, "invalid coin counters are rejected");
        const MapTrafficPose bus{{10.0, 20.0, 1.0}, 0.0};
        const auto right = openbus::app::passengerDoorQueuePose(bus, 3.0, 2.4, false, 0);
        const auto left = openbus::app::passengerDoorQueuePose(bus, 3.0, 2.4, true, 0);
        const auto firstCallQueue = openbus::app::passengerStopCallQueuePose(
            MapTrafficPose{{0.0, 0.0, 1.0}, 0.0}, false, 0);
        const auto repeatedCallQueue = openbus::app::passengerStopCallQueuePose(
            MapTrafficPose{{20.0, 0.0, 1.0}, 0.0}, false, 0);
        requireNear(std::hypot(firstCallQueue.position[0] - repeatedCallQueue.position[0],
                               firstCallQueue.position[1] - repeatedCallQueue.position[1]),
                    20.0, "repeated stop calls keep separate world queue locations");
        auto previousCallQueue = firstCallQueue;
        for (std::size_t slot = 1; slot < 5; ++slot) {
            const auto firstCallSlot = openbus::app::passengerStopCallQueuePose(
                MapTrafficPose{{0.0, 0.0, 1.0}, 0.0}, false, slot);
            const auto repeatedCallSlot = openbus::app::passengerStopCallQueuePose(
                MapTrafficPose{{20.0, 0.0, 1.0}, 0.0}, false, slot);
            requireNear(std::hypot(firstCallSlot.position[0] - previousCallQueue.position[0],
                                   firstCallSlot.position[1] - previousCallQueue.position[1]),
                        0.8, "successive riders retain consistent spacing at a stop call");
            requireNear(std::hypot(firstCallSlot.position[0] - repeatedCallSlot.position[0],
                                   firstCallSlot.position[1] - repeatedCallSlot.position[1]),
                        20.0, "each repeated stop call retains its own spaced queue");
            previousCallQueue = firstCallSlot;
        }
        requireNear(right.position[0], 11.9, "right-hand traffic queues at the right door");
        requireNear(left.position[0], 8.1, "left-hand traffic queues at the left door");
        requireNear(right.position[1], 23.0, "queue follows the actual front entry");
        for (std::size_t slot = 1; slot < 5; ++slot) {
            const auto rightQueued =
                openbus::app::passengerDoorQueuePose(bus, 3.0, 2.4, false, slot);
            const auto leftQueued = openbus::app::passengerDoorQueuePose(bus, 3.0, 2.4, true, slot);
            requireNear(rightQueued.position[0], right.position[0],
                        "right-hand queue stays in a single line beside the bus");
            requireNear(leftQueued.position[0], left.position[0],
                        "left-hand queue stays in a single line beside the bus");
            requireNear(rightQueued.position[1], right.position[1] - 0.7 * slot,
                        "each passenger queues behind the previous passenger");
            requireNear(leftQueued.position[1], rightQueued.position[1],
                        "both traffic directions preserve the same queue order");
            requireNear(rightQueued.headingRadians, bus.headingRadians,
                        "queued passengers face the front of the line");
        }
        const MapTrafficPose start{{11.9, 9.0, 1.0}, 0.0};
        const double duration = openbus::app::passengerApproachDuration(start, right);
        requireNear(duration, 10.0, "approach time follows walking distance");
        requireNear(openbus::app::passengerApproachPose(start, right, 5.0).position[1], 16.0,
                    "waiting passengers walk halfway in half the travel time");
        const auto returnStart = openbus::app::passengerApproachPose(right, start, 0.0);
        const auto returnMidpoint = openbus::app::passengerApproachPose(right, start, 5.0);
        const auto returnDestination = openbus::app::passengerApproachPose(right, start, 10.0);
        require(returnStart.position == right.position &&
                    returnDestination.position == start.position &&
                    returnMidpoint.position[1] < right.position[1] &&
                    returnMidpoint.position[1] > start.position[1],
                "passengers walk back to their queue position instead of teleporting after departure");
        auto secondStart = start;
        secondStart.position[1] -= 0.7;
        const auto secondDestination =
            openbus::app::passengerDoorQueuePose(bus, 3.0, 2.4, false, 1);
        const double secondDuration =
            openbus::app::passengerApproachDuration(secondStart, secondDestination);
        const auto firstConcurrentPose =
            openbus::app::passengerApproachPose(start, right, duration * 0.5);
        const auto secondConcurrentPose =
            openbus::app::passengerApproachPose(secondStart, secondDestination,
                                                secondDuration * 0.5);
        require(std::hypot(firstConcurrentPose.position[0] - secondConcurrentPose.position[0],
                           firstConcurrentPose.position[1] - secondConcurrentPose.position[1]) >
                    0.5 &&
                    openbus::app::passengerApproachPose(start, right, duration).position ==
                        right.position &&
                    openbus::app::passengerApproachPose(secondStart, secondDestination,
                                                        secondDuration)
                            .position == secondDestination.position,
                "multiple passengers keep independent progress toward distinct door queue slots");
        requireNear(openbus::app::passengerApproachPose(start, right, 20.0).position[1], 23.0,
                    "waiting passengers stop at the entrance");
        const MapTrafficPose stamper{{11.9, 26.0, 1.0}, 0.0};
        const MapTrafficPose destination{{11.9, 29.0, 1.0}, 0.0};
        const double stampApproachDuration =
            openbus::app::passengerApproachDuration(right, stamper);
        const auto walkingToStamper = openbus::app::passengerStamperActorPose(
            right, stamper, destination, stampApproachDuration * 0.5, 0.0, 0.0);
        require(walkingToStamper.walking,
                "ticket passenger walks from the machine toward the configured stamper");
        requireNear(walkingToStamper.pose.position[1], 24.5,
                    "ticket passenger follows the timed stamper approach");
        const auto waitingAtStamper = openbus::app::passengerStamperActorPose(
            right, stamper, destination, stampApproachDuration, 0.2, 0.0);
        require(!waitingAtStamper.walking,
                "ticket passenger pauses at the stamper before returning");
        requireNear(waitingAtStamper.pose.position[1], 26.0,
                    "ticket passenger waits at the stamper position");
        const auto returningFromStamper = openbus::app::passengerStamperActorPose(
            right, stamper, destination, stampApproachDuration, 0.5, 1.0);
        require(returningFromStamper.walking,
                "ticket passenger returns to the entry after stamping");
        const MapTrafficPose moneyPoint{{11.9, 24.0, 1.0}, 0.0};
        const MapTrafficPose changePoint{{11.9, 27.0, 1.0}, 0.0};
        const auto walkingToMoneyPoint = openbus::app::passengerTicketTransactionActorPose(
            right, moneyPoint, changePoint, true, 0.25, false, false,
            openbus::app::passengerApproachDuration(right, moneyPoint) * 0.5, 0.0);
        require(walkingToMoneyPoint.walking,
                "ticket passenger walks to the configured money point");
        requireNear(walkingToMoneyPoint.pose.position[1], 23.5,
                    "money-point movement follows the walking duration");
        const auto walkingToChangePoint = openbus::app::passengerTicketTransactionActorPose(
            right, moneyPoint, changePoint, true, 0.25, true, false, 1.0,
            openbus::app::passengerApproachDuration(moneyPoint, changePoint) * 0.5);
        require(walkingToChangePoint.walking,
                "ticket passenger walks from the money point to the change point");
        requireNear(walkingToChangePoint.pose.position[1], 25.5,
                    "change-point movement follows the walking duration");
        const auto waitingForChange = openbus::app::passengerTicketTransactionActorPose(
            right, moneyPoint, changePoint, true, 0.25, true, true, 1.0, 2.0);
        require(!waitingForChange.walking,
                "ticket passenger waits at the change point for the transaction");
        requireNear(waitingForChange.pose.position[1], 27.0,
                    "ticket passenger waits at the configured change point");
    }
    {
        VehicleConfig cabin;
        cabin.passengerPositions = {{{0.0, 2.0, 0.0}}};
        cabin.passengerPathPoints = {
            {0, {0.0, 0.0, 0.0}}, {1, {0.0, 1.0, 0.0}}, {2, {0.0, 2.0, 0.0}}};
        cabin.passengerPathLinks = {{0, 1, false}, {1, 2, false}};
        cabin.passengerEntryPathPoints = {0};
        cabin.passengerExitPathPoints = {0};
        const MapTrafficPose bus{{10.0, 20.0, 1.0}, 0.3};
        PassengerCabinPosition doorPosition;
        const auto door = openbus::app::passengerCabinSeatPose(bus, doorPosition);
        const auto seat = openbus::app::passengerCabinSeatPose(bus, cabin.passengerPositions[0]);
        const auto seatLocal = openbus::app::passengerCabinStandingPoint(
            cabin.passengerPositions[0].position, cabin.passengerPositions[0].seatHeight);
        const openbus::app::PassengerCabinTravelPathData directPath;
        requireNear(openbus::app::passengerCabinTravelDuration(
                        directPath, {0.0, 0.0, 0.0}, {0.0, 14.0, 0.0}, false),
                    10.0, "fallback walking time scales with distance");
        requireNear(openbus::app::passengerCabinTravelDuration(
                        directPath, {0.0, 0.0, 0.0}, {0.0, 14.0, 0.0}, true),
                    12.5, "seated walking time includes the sitting transition");
        requireNear(openbus::app::passengerCabinTravelDuration(
                        directPath, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, false),
                    0.75, "zero-length paths retain a bounded transition time");
        openbus::app::PassengerCabinTravelPathData bentPath;
        bentPath.positions = {{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, {3.0, 4.0, 0.0}};
        requireNear(openbus::app::passengerCabinTravelDuration(
                        bentPath, bentPath.positions.front(), bentPath.positions.back(), false),
                    5.0, "cabin walking time follows the full path rather than its chord");
        MapPassengerServiceRuntime service(1);
        service.setWaitingPassengers(901, 1);
        require(service.arrive({.stopId = 901, .dwellSeconds = 1.0, .nextStopId = 902}),
                "headless cabin occupancy fixture starts boarding");
        service.advance(0.225, true, true, true);
        openbus::app::updatePassengerCabinPathOccupancy(service, cabin, bus, door.position,
                                                        seatLocal, true, 0, 0);
        require(service.passengerCountOnPathLink(0) == 1 &&
                    service.passengerCountOnPathLink(1) == 0,
                "simulation computes the first boarding link without a render pass");
        service.advance(0.55, true, true, true);
        require(service.passengerCountOnPathLink(0) == 0 &&
                    service.passengerCountOnPathLink(1) == 1,
                "simulation progress moves boarding occupancy to the next link");
        service.advance(0.8, true, true, true);
        require(service.passengerCountOnPathLink(1) == 0 && service.onboardPassengerCount() == 1,
                "boarding completion clears path occupancy");
        service.depart();
        require(service.arrive({.stopId = 902, .dwellSeconds = 1.0}),
                "headless cabin occupancy fixture starts alighting");
        service.advance(0.225, false, true, true);
        openbus::app::updatePassengerCabinPathOccupancy(
            service, cabin, bus, seat.position,
            openbus::app::passengerCabinLocalPoint(bus, door.position), false, 0, 0);
        require(service.passengerCountOnPathLink(1) == 1 &&
                    service.passengerCountOnPathLink(0) == 0,
                "simulation computes the reverse alighting route without rendering");
        service.advance(0.225, false, false, true);
        require(service.passengerCountOnPathLink(1) == 1,
                "closed exit doors leave alighting occupancy on the current link");
        service.advance(0.55, false, true, true);
        require(service.passengerCountOnPathLink(1) == 0 &&
                    service.passengerCountOnPathLink(0) == 1,
                "simulation advances alighting occupancy to the selected exit");
        cabin.passengerPathLinks[0].oneWay = true;
        openbus::app::updatePassengerCabinPathOccupancy(
            service, cabin, bus, seat.position,
            openbus::app::passengerCabinLocalPoint(bus, door.position), false, 0, 0);
        require(service.passengerCountOnPathLink(0) == 0 &&
                    service.passengerCountOnPathLink(1) == 0,
                "an unreachable selected exit clears occupancy for the direct-path fallback");
        service.depart();
        require(service.passengerCountOnPathLink(0) == 0,
                "departure clears active cabin path occupancy");

        cabin.passengerPathLinks[0].oneWay = false;
        cabin.passengerPositions[0].position[2] = 0.45;
        cabin.passengerPositions[0].seatHeight = 0.45;
        MapPassengerServiceRuntime seatedService(1);
        seatedService.setWaitingPassengers(903, 1);
        require(seatedService.arrive({.stopId = 903, .dwellSeconds = 1.0, .nextStopId = 904}),
                "seated cabin occupancy fixture starts boarding");
        seatedService.advance(0.375, true, true, true);
        openbus::app::updatePassengerCabinPathOccupancy(
            seatedService, cabin, bus, door.position, seatLocal, true, 0, 0);
        seatedService.advance(0.7, true, true, true);
        require(seatedService.passengerCountOnPathLink(1) == 1,
                "a seated passenger occupies the cabin link while walking to the seat");
        seatedService.advance(0.55, true, true, true);
        require(seatedService.boardingPassenger() &&
                    seatedService.passengerCountOnPathLink(0) == 0 &&
                    seatedService.passengerCountOnPathLink(1) == 0,
                "the sitting transition does not occupy a cabin link");
        seatedService.advance(0.2, true, true, true);
        seatedService.depart();
        require(seatedService.arrive({.stopId = 904, .dwellSeconds = 1.0}),
                "seated cabin occupancy fixture starts alighting");
        seatedService.advance(0.075, false, true, true);
        openbus::app::updatePassengerCabinPathOccupancy(
            seatedService, cabin, bus, seat.position,
            openbus::app::passengerCabinLocalPoint(bus, door.position), false, 0, 0);
        require(seatedService.passengerCountOnPathLink(0) == 0 &&
                    seatedService.passengerCountOnPathLink(1) == 0,
                "the standing transition does not occupy a cabin link");
        seatedService.advance(0.55, false, true, true);
        require(seatedService.passengerCountOnPathLink(1) == 1,
                "alighting occupies the selected path after standing");
    }
    {
        MapPassengerServiceRuntime service(2);
        service.setWaitingPassengers(905, 2);
        require(service.arrive({.stopId = 905, .dwellSeconds = 10.0, .nextStopId = 906}),
                "distance-based animation fixture starts boarding");
        service.advance(0.25, true, true, true, true, nullptr, true);
        require(service.boardingPassenger() && service.boardingProgress() == 0.0,
                "boarding waits until its path duration is configured");
        service.setPassengerAnimationDuration(2.0);
        service.advance(0.75, true, true, true, true, nullptr, true);
        requireNear(service.boardingProgress(), 0.375,
                    "boarding progression uses the configured walking duration");
        service.setPassengerAnimationDuration(10.0);
        service.advance(1.0, false, true, true, true, nullptr, true);
        requireNear(service.boardingProgress(), 0.375,
                    "closing the door pauses distance-based boarding");
        service.advance(1.25, true, true, true, true, nullptr, true);
        require(service.onboardPassengerCount() == 1 && !service.boardingPassenger(),
                "the first passenger completes after its original duration");
        service.advance(0.25, true, true, true, true, nullptr, true);
        service.setPassengerAnimationDuration(3.0);
        service.advance(0.25, true, true, true, true, nullptr, true);
        requireNear(service.boardingProgress(), 0.25 / 3.0,
                    "the next passenger receives a fresh path duration");
        service.advance(2.75, true, true, true, true, nullptr, true);
        service.depart();
        require(service.arrive({.stopId = 906, .dwellSeconds = 1.0}),
                "distance-based animation fixture starts alighting");
        service.advance(0.25, false, true, true, true, nullptr, true);
        require(service.alightingPassenger() && service.alightingProgress() == 0.0,
                "alighting waits until its path duration is configured");
        service.setPassengerAnimationDuration(2.0);
        service.advance(1.0, false, true, true, true, nullptr, true);
        requireNear(service.alightingProgress(), 0.5,
                    "alighting progression uses the configured walking duration");
        service.advance(1.0, false, true, true, true, nullptr, true);
        require(service.alightingPassenger() && service.alightingProgress() == 0.0,
                "the next alighter starts with no inherited animation duration");
        service.advance(0.25, false, true, true, true, nullptr, true);
        require(service.alightingProgress() == 0.0,
                "the next alighter also waits for its own path duration");
        service.setPassengerAnimationDuration(3.0);
        service.advance(1.5, false, true, true, true, nullptr, true);
        requireNear(service.alightingProgress(), 0.5,
                    "the next alighter uses its own path duration");
    }
    {
        VehicleConfig cabin;
        cabin.passengerPositions = {{{0.0, 0.0, 0.0}}};
        const MapTrafficPose bus{{0.0, 0.0, 1.0}, 0.3};
        MapPassengerServiceRuntime service(1);
        service.setWaitingPassengers(907, 1);
        require(service.arrive({.stopId = 907, .dwellSeconds = 10.0, .nextStopId = 908}),
                "fallback animation fixture starts boarding");
        service.advance(0.25, true, true, true, true, nullptr, true);
        openbus::app::updatePassengerCabinPathOccupancy(
            service, cabin, bus, {0.0, 0.0, 0.0},
            openbus::app::passengerCabinLocalPoint(bus, {6.0, 0.0, 0.0}), true, 0,
            std::nullopt, std::array<double, 3>{3.0, 4.0, 0.0});
        service.advance(1.0, true, true, true, true, nullptr, true);
        requireNear(service.boardingProgress(), 0.14,
                    "fallback duration includes the rendered intermediate curb point");
        require(service.passengerCountOnPathLink(0) == 0,
                "fallback movement does not occupy an authored cabin link");
    }
    {
        openbus::app::PassengerCabinTravelPathData path;
        path.positions = {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}};
        path.doorPointIndex = 1;
        const auto standingWait = openbus::app::passengerAlightingDoorWaitProgress(path, false);
        const auto seatedWait = openbus::app::passengerAlightingDoorWaitProgress(path, true);
        require(standingWait && seatedWait, "a connected exit has a door waiting position");
        requireNear(*standingWait, 0.5, "waiting position is half a metre inside the exit");
        requireNear(*seatedWait, 0.6, "exit approach includes the standing transition");
        path.doorPointIndex.reset();
        require(!openbus::app::passengerAlightingDoorWaitProgress(path, true),
                "an unknown exit does not invent a safe waiting position");

        MapPassengerServiceRuntime service(2);
        service.setWaitingPassengers(909, 2);
        require(service.arrive({.stopId = 909, .dwellSeconds = 2.0, .nextStopId = 910}),
                "exit waiting fixture starts boarding");
        service.advance(1.5, true, true, true);
        service.depart();
        require(service.arrive({.stopId = 910, .dwellSeconds = 1.0}),
                "exit waiting fixture reaches its destination");
        service.advance(0.25, false, false, true, true, nullptr, true);
        requireNear(service.alightingProgress(), 0.0,
                    "closed exit waits for its path to be configured");
        service.setPassengerAnimationDuration(3.0);
        service.setAlightingDoorWaitProgress(seatedWait);
        service.advance(10.0, false, false, true, true, nullptr, true);
        requireNear(service.alightingProgress(), 0.6,
                    "alighter approaches but does not pass the closed door");
        require(service.onboardPassengerCount() == 2 &&
                    openbus::app::vehicle::passengerServiceRequestVariables(0, 1, service)
                        .exits[1],
                "waiting alighter keeps its seat reservation and requests the selected exit");
        service.advance(1.0, false, false, true, true, nullptr, true);
        requireNear(service.alightingProgress(), 0.6,
                    "alighter remains at the closed door");
        service.advance(0.6, false, true, true, true, nullptr, true);
        requireNear(service.alightingProgress(), 0.8,
                    "an open door lets the alighter continue past the waiting point");
        service.advance(1.0, false, false, true, true, nullptr, true);
        requireNear(service.alightingProgress(), 0.8,
                    "closing the door again pauses without moving the alighter backwards");
        service.advance(0.6, false, true, true, true, nullptr, true);
        require(service.onboardPassengerCount() == 1 && service.alightingPassenger() &&
                    service.alightingProgress() == 0.0,
                "opening the door releases the first alighter and selects the next");
        service.setPassengerAnimationDuration(3.0);
        service.advance(1.0, false, false, true, true, nullptr, true);
        requireNear(service.alightingProgress(), 0.0,
                    "the next alighter does not inherit the previous door waiting position");
    }
    {
        VehicleConfig cabin;
        cabin.passengerPositions = {{{0.0, 2.0, 0.45}, 0.45, 0.0}};
        cabin.passengerPathPoints = {
            {0, {0.0, 0.0, 0.0}}, {1, {0.0, 1.0, 0.0}}, {2, {0.0, 2.0, 0.0}}};
        cabin.passengerPathLinks = {{0, 1, false}, {1, 2, false}};
        cabin.passengerExitPathPoints = {0};
        const MapTrafficPose bus{{10.0, 20.0, 1.0}, 0.3};
        const auto seat = openbus::app::passengerCabinSeatPose(bus, cabin.passengerPositions[0]);
        const std::array<double, 3> outsideLocal{0.0, -1.0, 0.0};
        MapPassengerServiceRuntime service(1);
        service.setWaitingPassengers(911, 1);
        require(service.arrive({.stopId = 911, .dwellSeconds = 1.0, .nextStopId = 912}),
                "connected exit approach fixture starts boarding");
        service.advance(0.75, true, true, true);
        service.depart();
        require(service.arrive({.stopId = 912, .dwellSeconds = 1.0}),
                "connected exit approach fixture reaches its destination");
        service.advance(0.25, false, false, true, true, nullptr, true);
        openbus::app::updatePassengerCabinPathOccupancy(service, cabin, bus, seat.position,
                                                        outsideLocal, false, 0, 0);
        service.advance(10.0, false, false, true, true, nullptr, true);
        requireNear(service.alightingProgress(), 0.6,
                    "the authored exit path stops a closed-door alighter before the exit point");
        require(service.onboardPassengerCount() == 1,
                "a closed door cannot complete an authored exit path");
    }
    {
        MapPassengerServiceRuntime player(1);
        std::size_t queue = 1;
        require(player.arrive({.stopId = 50, .dwellSeconds = 1.0, .nextStopId = 51}),
                "player stop starts");
        advanceMapPassengerServiceWithSharedQueue(player, 50, queue, queueIdentities, 10.0, false,
                                                  false, true);
        require(player.dwelling() && queue == 1 && player.onboardPassengerCount() == 0,
                "closed doors keep a player stop open without boarding past its nominal dwell");
        advanceMapPassengerServiceWithSharedQueue(player, 50, queue, queueIdentities, 0.25, true,
                                                  true, true);
        require(player.boardingPassenger() && queue == 0,
                "an open door reserves a passenger after the wait");
        const double progress = player.boardingProgress();
        advanceMapPassengerServiceWithSharedQueue(player, 50, queue, queueIdentities, 2.0, false,
                                                  false, true);
        require(player.boardingPassenger() && player.boardingProgress() == progress,
                "closing the door pauses an in-progress boarding passenger");
        player.depart();
        require(!player.dwelling() && player.waitingPassengers(50) == 1 &&
                    player.onboardPassengerCount() == 0,
                "pulling away returns an unfinished boarder to the queue");
        require(player.arrive({.stopId = 50, .dwellSeconds = 1.0, .nextStopId = 51}),
                "a departed service can arrive again");
        player.advance(0.75, true, true, true);
        require(player.onboardPassengerCount() == 1, "boarding completes while a stop is held");
        player.depart();
        require(player.onboardPassengerCount() == 1,
                "pulling away preserves passengers already on board");
    }
    for (const std::size_t waiting : {1U, 3U, 7U, 10U}) {
        constexpr std::size_t initial = 10;
        constexpr std::size_t capacity = 3;
        const std::size_t visible = std::min(waiting, capacity);
        std::unordered_set<std::size_t> occupied;
        for (std::size_t passenger = waiting - visible; passenger < waiting; ++passenger) {
            occupied.insert(mapScheduledPassengerQueueSlot(initial, passenger) % capacity);
        }

        require(occupied.size() == visible,
                "visible waiting passengers occupy distinct authored slots");
        const auto nextBoardingSlot = mapScheduledPassengerQueueSlot(initial, waiting - 1);
        require(occupied.contains(nextBoardingSlot % capacity),
                "next boarding passenger starts in a visible waiting slot");
    }
    bool rejectedInvalidQueueIndex = false;
    try {
        (void)mapScheduledPassengerQueueSlot(3, 3);
    } catch (const std::invalid_argument&) {
        rejectedInvalidQueueIndex = true;
    }
    require(rejectedInvalidQueueIndex, "queue slots reject out-of-range passenger indices");
    MapPassengerServiceRuntime boardingQueue(1);
    boardingQueue.setWaitingPassengers(42, 4);
    require(boardingQueue.arrive({.stopId = 42, .dwellSeconds = 2.0, .nextStopId = 43}),
            "boarding continuity test arrives at its stop");
    const std::size_t beforeBoarding = boardingQueue.waitingPassengers(42);
    const auto visibleSlot = mapScheduledPassengerQueueSlot(4, beforeBoarding - 1);
    boardingQueue.advance(0.01);
    const std::size_t afterBoarding = boardingQueue.waitingPassengers(42);
    require(boardingQueue.boardingPassenger() && afterBoarding + 1 == beforeBoarding &&
                mapScheduledPassengerQueueSlot(4, afterBoarding) == visibleSlot &&
                boardingQueue.boardingPassengerQueueSlot(4) == visibleSlot &&
                boardingQueue.boardingPassengerModelIndex() ==
                    mapScheduledPassengerModelIdentity(42, beforeBoarding - 1),
            "boarding retains the visible passenger's slot and model after queue decrement");
    require(mapScheduledPassengerModelIdentity(400, 0) == 400 &&
                mapScheduledPassengerModelIdentity(400, 1) == 401,
            "visible passenger model identities follow deterministic queue positions");
    bool rejectedInvalidPassengerStop = false;
    try {
        (void)mapScheduledPassengerModelIdentity(0, 0);
    } catch (const std::invalid_argument& error) {
        rejectedInvalidPassengerStop =
            std::string_view(error.what()).find("stopId=0") != std::string_view::npos &&
            std::string_view(error.what()).find("queuePosition=0") != std::string_view::npos;
    }
    bool rejectedPassengerIdentityOverflow = false;
    try {
        (void)mapScheduledPassengerModelIdentity(1, (std::numeric_limits<std::size_t>::max)());
    } catch (const std::overflow_error& error) {
        rejectedPassengerIdentityOverflow =
            std::string_view(error.what()).find("stopId=1") != std::string_view::npos &&
            std::string_view(error.what()).find("queuePosition=") != std::string_view::npos;
    }
    require(rejectedInvalidPassengerStop && rejectedPassengerIdentityOverflow,
            "passenger identity rejects invalid stop IDs and overflowing queue positions");
    require(mapScheduledPassengerQueueSize(10, 0.5) == 5 &&
                mapScheduledPassengerQueueSize(1, 0.5) == 1 &&
                mapScheduledPassengerQueueSize(10, 0.0) == 0 &&
                mapScheduledPassengerQueueSize(10, -1.0) == 0 &&
                mapScheduledPassengerQueueSize(10, 2.0, 12) == 12 &&
                mapScheduledPassengerQueueSize((std::numeric_limits<std::size_t>::max)(), 1.0e100,
                                               50) == 50 &&
                mapScheduledPassengerQueueAfterDeparture(1, 4) == 4 &&
                mapScheduledPassengerQueueAfterDeparture(5, 4) == 5 &&
                mapScheduledPassengerQueueAfterDeparture(0, 0) == 0 &&
                mapPassengerServiceUpdateOrder({0, 40, 0, 40}, 0) ==
                    std::vector<std::size_t>{1, 3, 0, 2} &&
                mapPassengerServiceUpdateOrder({0, 40, 0, 40}, 1) ==
                    std::vector<std::size_t>{3, 1, 0, 2} &&
                mapPassengerServiceUpdateOrder({0, 40, 0, 40}, 2) ==
                    std::vector<std::size_t>{1, 3, 0, 2} &&
                mapPassengerServiceUpdateOrder({0, 60, 60, 40, 40}, 1) ==
                    std::vector<std::size_t>{4, 3, 2, 1, 0} &&
                mapPassengerServiceUpdateOrder({0, 60, 60, 40, 40}, 2) ==
                    std::vector<std::size_t>{3, 4, 1, 2, 0} &&
                mapPassengerServiceUpdateOrder({0, 0}, 5) == std::vector<std::size_t>{0, 1} &&
                mapPassengerServiceUpdateOrder({}, 5).empty(),
            "passenger-service priority rotates independently for each shared stop queue");

    MapPassengerServiceRuntime service(2);
    require(service.seatCount() == 2, "vehicle passenger seats define service capacity");
    service.setWaitingPassengers(100, 4);
    require(service.arrive({.stopId = 100, .dwellSeconds = 2.0, .nextStopId = 200}) &&
                service.dwelling() && service.currentStopId() == 100 &&
                service.onboardPassengerCount() == 0,
            "arrival opens a stop dwell and alights no more than occupied seats");
    service.advance(0.5);
    require(service.passengerStopRequestSequence() == 0,
            "no passenger stop request is generated during boarding");
    service.advance(0.25);
    require(service.passengerCountOnSeat(0) == 1 && service.passengerCountOnSeat(1) == 0 &&
                service.waitingPassengers(100) == 3 &&
                service.passengerStopRequestSequence() == 0,
            "boarding assigns a destination without requesting the stop immediately");
    service.advance(1.25);
    require(!service.dwelling() && service.onboardPassengerCount() == 2 &&
                service.waitingPassengers(100) == 2 &&
                service.passengerStopRequestSequence() == 0,
            "boarding multiple passengers does not request the destination prematurely");
    service.requestStopForPassengers(200, 200.0);
    require(service.passengerStopRequestSequence() == 2,
            "each passenger requests the destination once when it is within 200 metres");
    service.setWaitingPassengers(200, 2);
    require(service.arrive({.stopId = 200, .dwellSeconds = 1.0, .nextStopId = 300}) &&
                service.onboardPassengerCount() == 2 && service.alightingPassenger(),
            "passengers reserve a timed alighting animation at their assigned destination");
    service.advance(2.0);
    require(!service.dwelling() && service.onboardPassengerCount() == 0 &&
                !service.boardingPassenger() && service.waitingPassengers(200) == 2,
            "alighting that outlasts the dwell does not start new boarding");
    require(!service.arrive({.stopId = 0, .dwellSeconds = 1.0}) &&
                !service.arrive({.stopId = 201, .dwellSeconds = -1.0}),
            "invalid stops and dwell durations are rejected");

    MapPassengerServiceRuntime finalStopService(2);
    finalStopService.setWaitingPassengers(850, 2);
    require(finalStopService.arrive({.stopId = 850, .dwellSeconds = 2.0, .nextStopId = 900}),
            "final-stop passenger fixture boards its passengers");
    finalStopService.advance(1.5);
    require(finalStopService.onboardPassengerCount() == 2,
            "final-stop passenger fixture has two seated riders");
    finalStopService.advance(0.5);
    require(!finalStopService.dwelling(),
            "final-stop passenger fixture completes the prior stop dwell");
    require(finalStopService.arrive({.stopId = 900, .dwellSeconds = 0.75}),
            "final stop starts its configured dwell and passenger exit");
    finalStopService.advance(0.75);
    require(
        finalStopService.dwelling() && finalStopService.onboardPassengerCount() == 1 &&
            finalStopService.alightingPassenger(),
        "final-stop service remains active when the configured dwell ends before all riders exit");
    finalStopService.advance(0.75);
    require(!finalStopService.dwelling() && finalStopService.onboardPassengerCount() == 0,
            "final-stop service completes after the last passenger has exited");

    MapPassengerServiceRuntime doorSwitchService(2);
    doorSwitchService.setWaitingPassengers(870, 2);
    require(doorSwitchService.arrive({.stopId = 870, .dwellSeconds = 2.0, .nextStopId = 871}),
            "multi-door fixture starts boarding");
    doorSwitchService.advance(2.0);
    require(doorSwitchService.onboardPassengerCount() == 2 &&
                doorSwitchService.arrive({.stopId = 871, .dwellSeconds = 2.0}),
            "multi-door fixture has two alighting passengers");
    doorSwitchService.advance(0.7, false, true, true, true);
    doorSwitchService.advance(0.2, false, true, true, true);
    require(doorSwitchService.alightingSeatIndex() == 1 &&
                doorSwitchService.alightingProgress() == 0.0 &&
                doorSwitchService.onboardPassengerCount() == 1,
            "next alighter waits for its own door to be selected after a previous exit");
    doorSwitchService.advance(0.25, false, false, true, true);
    require(doorSwitchService.alightingProgress() == 0.0,
            "an open previous door cannot move the next passenger through a closed exit");

    MapPassengerServiceRuntime alightingService(1);
    alightingService.setWaitingPassengers(800, 1);
    require(alightingService.arrive({.stopId = 800, .dwellSeconds = 2.0, .nextStopId = 900}),
            "alighting animation fixture starts at a valid stop");
    alightingService.advance(0.75);
    alightingService.advance(1.25);
    alightingService.setWaitingPassengers(900, 1);
    require(alightingService.passengerCountOnSeat(0) == 1 &&
                alightingService.arrive({.stopId = 900, .dwellSeconds = 2.0, .nextStopId = 1000}),
            "a seated passenger arrives at their assigned destination");
    alightingService.advance(0.375);
    require(alightingService.alightingPassenger() && alightingService.alightingSeatIndex() == 0 &&
                std::abs(alightingService.alightingProgress() - 0.5) < 1.0e-9 &&
                alightingService.passengerCountOnSeat(0) == 1 &&
                !alightingService.boardingPassenger() &&
                alightingService.waitingPassengers(900) == 1,
            "alighting starts first and prevents boarding through the same cabin exit");
    alightingService.setPassengerPathLinkProgressProfile(
        {{0.7, std::size_t{2}}, {1.0, std::nullopt}});
    require(alightingService.passengerCountOnPathLink(2) == 1,
            "an active alighting passenger occupies its current cabin pathlink");
    alightingService.advance(0.15);
    require(alightingService.passengerCountOnPathLink(2) == 0,
            "simulation progress moves an alighting passenger off the previous pathlink");
    alightingService.advance(0.35);
    require(!alightingService.alightingPassenger() &&
                alightingService.passengerCountOnSeat(0) == 0 &&
                alightingService.boardingPassenger() &&
                alightingService.passengerCountOnPathLink(2) == 0 &&
                std::abs(alightingService.boardingProgress() - (1.0 / 6.0)) < 1.0e-9,
            "boarding begins after alighting clears its pathlink occupancy and shared exit");

    MapPassengerServiceRuntime firstBus(1);
    MapPassengerServiceRuntime secondBus(1);
    require(firstBus.arrive({.stopId = 400, .dwellSeconds = 2.0, .nextStopId = 500}) &&
                secondBus.arrive({.stopId = 400, .dwellSeconds = 2.0, .nextStopId = 500}),
            "multiple buses can dwell at a shared stop");
    std::size_t sharedWaitingPassengers = 2;
    advanceMapPassengerServiceWithSharedQueue(firstBus, 400, sharedWaitingPassengers,
                                              queueIdentities, 0.375);
    advanceMapPassengerServiceWithSharedQueue(secondBus, 400, sharedWaitingPassengers,
                                              queueIdentities, 0.375);
    require(
        sharedWaitingPassengers == 0 &&
            firstBus.boardingPassengerModelIndex() == mapScheduledPassengerModelIdentity(400, 1) &&
            secondBus.boardingPassengerModelIndex() == mapScheduledPassengerModelIdentity(400, 0) &&
            firstBus.boardingPassengerQueueSlot(2) == 0 &&
            secondBus.boardingPassengerQueueSlot(2) == 1,
        "buses at a shared stop reserve different passengers from the visible queue");
    advanceMapPassengerServiceWithSharedQueue(firstBus, 400, sharedWaitingPassengers,
                                              queueIdentities, 0.1);
    require(firstBus.waitingPassengers(400) == 0 && firstBus.boardingPassengerQueueSlot(2) == 0 &&
                secondBus.boardingPassengerQueueSlot(2) == 1,
            "other reservations do not move an already boarding passenger to a different slot");
    advanceMapPassengerServiceWithSharedQueue(firstBus, 400, sharedWaitingPassengers,
                                              queueIdentities, 0.275);
    advanceMapPassengerServiceWithSharedQueue(secondBus, 400, sharedWaitingPassengers,
                                              queueIdentities, 0.375);
    require(firstBus.passengerModelIndexOnSeat(0) == 401 &&
                secondBus.passengerModelIndexOnSeat(0) == 400 &&
                !firstBus.boardingPassengerQueueSlot(2) && !secondBus.boardingPassengerQueueSlot(2),
            "shared queue passenger identities persist into each bus seat");

    MapPassengerServiceRuntime departingBus(1);
    MapPassengerServiceRuntime retiringBus(1);
    require(departingBus.arrive({.stopId = 410, .dwellSeconds = 2.0, .nextStopId = 420}) &&
                retiringBus.arrive({.stopId = 410, .dwellSeconds = 2.0, .nextStopId = 420}),
            "shared cancellation fixture starts two buses at the same stop");
    std::size_t cancellationQueue = 2;
    advanceMapPassengerServiceWithSharedQueue(departingBus, 410, cancellationQueue, queueIdentities,
                                              0.25);
    advanceMapPassengerServiceWithSharedQueue(retiringBus, 410, cancellationQueue, queueIdentities,
                                              0.25);
    require(cancellationQueue == 0 && departingBus.waitingPassengers(410) == 1,
            "the first bus has a stale local queue after the second reservation");
    departMapPassengerServiceWithSharedQueue(departingBus, cancellationQueue, queueIdentities);
    require(cancellationQueue == 1 && !departingBus.boardingPassenger() &&
                !departingBus.boardingPassengerQueueSlot(2) &&
                queueIdentities.modelIdentity(410, 0) == 411 &&
                retiringBus.boardingPassengerModelIndex() == 410,
            "departure returns its original model without replacing another reserved rider");
    endMapPassengerServiceWithSharedQueue(retiringBus, cancellationQueue, queueIdentities);
    require(cancellationQueue == 2 && !retiringBus.boardingPassenger() &&
                !retiringBus.boardingPassengerQueueSlot(2) &&
                queueIdentities.modelIdentity(410, 0) == 411 &&
                queueIdentities.modelIdentity(410, 1) == 410,
            "retirement preserves both returned models in the shared queue");
    endMapPassengerServiceWithSharedQueue(retiringBus, cancellationQueue, queueIdentities);
    require(cancellationQueue == 2 && queueIdentities.modelIdentity(410, 1) == 410,
            "repeated retirement does not return or replace a rider twice");
    MapPassengerServiceRuntime reboardingBus(2);
    require(reboardingBus.arrive({.stopId = 410, .dwellSeconds = 2.0, .nextStopId = 420}),
            "returned passengers can board a later bus");
    advanceMapPassengerServiceWithSharedQueue(reboardingBus, 410, cancellationQueue,
                                              queueIdentities, 1.5);
    require(cancellationQueue == 0 && reboardingBus.passengerModelIndexOnSeat(0) == 410 &&
                reboardingBus.passengerModelIndexOnSeat(1) == 411,
            "multiple returned passengers keep their models when boarded in one update");
    require(queueIdentities.modelIdentity(410, 0) == 410 &&
                queueIdentities.modelIdentity(410, 1) == 411,
            "boarding consumes overrides so later queue replenishment uses default models");
    reboardingBus.advance(0.5);
    require(reboardingBus.arrive({.stopId = 420, .dwellSeconds = 2.0}) &&
                reboardingBus.alightingPassenger() &&
                reboardingBus.passengerModelIndexOnSeat(0) == 410 &&
                reboardingBus.passengerModelIndexOnSeat(1) == 411,
            "reboarded passenger models persist through to alighting");

    MapPassengerServiceRuntime interruptedBus(1);
    MapPassengerServiceRuntime concurrentBus(1);
    MapPassengerServiceRuntime replacementBus(1);
    require(interruptedBus.arrive({.stopId = 415, .dwellSeconds = 2.0, .nextStopId = 420}) &&
                concurrentBus.arrive({.stopId = 415, .dwellSeconds = 2.0, .nextStopId = 420}) &&
                replacementBus.arrive({.stopId = 415, .dwellSeconds = 2.0, .nextStopId = 420}),
            "replacement fixture starts three buses at the same stop");
    std::size_t replacementQueue = 2;
    advanceMapPassengerServiceWithSharedQueue(interruptedBus, 415, replacementQueue,
                                              queueIdentities, 0.25);
    advanceMapPassengerServiceWithSharedQueue(concurrentBus, 415, replacementQueue, queueIdentities,
                                              0.25);
    departMapPassengerServiceWithSharedQueue(interruptedBus, replacementQueue, queueIdentities);
    advanceMapPassengerServiceWithSharedQueue(replacementBus, 415, replacementQueue,
                                              queueIdentities, 0.25, false);
    require(replacementQueue == 1 && queueIdentities.modelIdentity(415, 0) == 416,
            "a closed door does not consume the returned rider's model identity");
    advanceMapPassengerServiceWithSharedQueue(replacementBus, 415, replacementQueue,
                                              queueIdentities, 0.75);
    advanceMapPassengerServiceWithSharedQueue(concurrentBus, 415, replacementQueue, queueIdentities,
                                              0.5);
    require(replacementQueue == 0 && replacementBus.passengerModelIndexOnSeat(0) == 416 &&
                concurrentBus.passengerModelIndexOnSeat(0) == 415,
            "a returned rider can reboard while the other reserved rider finishes boarding");
    require(queueIdentities.modelIdentity(410, 0) == 410,
            "passenger identity overrides are isolated by stop");
    MapPassengerQueueIdentities independentIdentities;
    independentIdentities.returnPassenger(700, 0, 707);
    independentIdentities.returnPassenger(701, 0, 709);
    require(independentIdentities.take(700, 0) == 707 &&
                independentIdentities.modelIdentity(701, 0) == 709 &&
                independentIdentities.modelIdentity(700, 0) == 700,
            "consuming an override leaves other stops intact and restores the default slot");
    bool rejectedDuplicateQueuePosition = false;
    try {
        independentIdentities.returnPassenger(701, 0, 701);
    } catch (const std::logic_error&) {
        rejectedDuplicateQueuePosition = true;
    }
    require(rejectedDuplicateQueuePosition && independentIdentities.modelIdentity(701, 0) == 709,
            "an already occupied returned-passenger slot is not silently replaced");
    bool rejectedMismatchedQueue = false;
    try {
        advanceMapPassengerServiceWithSharedQueue(concurrentBus, 416, replacementQueue,
                                                  queueIdentities, 0.1);
    } catch (const std::invalid_argument&) {
        rejectedMismatchedQueue = true;
    }
    require(rejectedMismatchedQueue && replacementQueue == 0,
            "a service cannot consume another stop's shared queue");

    MapPassengerServiceRuntime finishedBus(1);
    MapPassengerServiceRuntime activeBus(1);
    require(finishedBus.arrive({.stopId = 430, .dwellSeconds = 2.0, .nextStopId = 440}) &&
                activeBus.arrive({.stopId = 430, .dwellSeconds = 2.0, .nextStopId = 440}),
            "shared completed-boarding fixture starts two buses");
    std::size_t completedQueue = 2;
    advanceMapPassengerServiceWithSharedQueue(finishedBus, 430, completedQueue, queueIdentities,
                                              0.75);
    advanceMapPassengerServiceWithSharedQueue(activeBus, 430, completedQueue, queueIdentities,
                                              0.25);
    require(completedQueue == 0 && finishedBus.waitingPassengers(430) == 1,
            "a completed bus may have a stale snapshot after another reservation");
    departMapPassengerServiceWithSharedQueue(finishedBus, completedQueue, queueIdentities);
    require(completedQueue == 0 && finishedBus.onboardPassengerCount() == 1,
            "a completed bus cannot restore its stale queue snapshot on departure");
    advanceMapPassengerServiceWithSharedQueue(activeBus, 430, completedQueue, queueIdentities, 0.5);
    require(completedQueue == 0 && activeBus.onboardPassengerCount() == 1,
            "the second reservation completes without creating extra passengers");

    MapPassengerServiceRuntime terminalBus(1);
    require(terminalBus.arrive({.stopId = 450, .dwellSeconds = 0.75, .nextStopId = 0}),
            "terminal bus opens a stop dwell without an onward passenger destination");
    std::size_t terminalStopQueue = 3;
    advanceMapPassengerServiceWithSharedQueue(terminalBus, 450, terminalStopQueue, queueIdentities,
                                              0.75);
    require(terminalStopQueue == 3 && terminalBus.onboardPassengerCount() == 0 &&
                !terminalBus.boardingPassenger(),
            "terminal calls preserve shared waiting passengers and do not board riders without a "
            "destination");

    MapPassengerServiceRuntime capacityService(1);
    const auto capacityRequests = [&] {
        return openbus::app::vehicle::passengerServiceRequestVariables(0, 0, capacityService);
    };
    capacityService.setWaitingPassengers(460, 2);
    require(capacityService.arrive({.stopId = 460, .dwellSeconds = 4.0, .nextStopId = 470}) &&
                capacityRequests().entries[0],
            "waiting passengers request entry when a seat is available");
    capacityService.advance(0.375, true, true, true);
    require(!capacityService.canAcceptWaitingPassenger() && capacityRequests().entries[0] &&
                capacityService.waitingPassengers(460) == 1,
            "a reserved last seat cannot accept another rider but keeps its entry request");
    capacityService.advance(0.375, true, true, true);
    require(capacityService.onboardPassengerCount() == 1 && !capacityRequests().entries[0] &&
                capacityService.waitingPassengers(460) == 1,
            "a full bus clears the entry request without consuming the remaining queue");
    capacityService.depart();
    capacityService.setWaitingPassengers(470, 1);
    require(capacityService.arrive({.stopId = 470, .dwellSeconds = 4.0, .nextStopId = 480}) &&
                capacityRequests().entries[0] && capacityRequests().exits[0],
            "a pending alighter makes room for a waiting passenger and requests both doors");
    capacityService.advance(0.375, false, false, true);
    require(capacityRequests().entries[0] && capacityRequests().exits[0],
            "closed doors retain requests while the occupied seat is due to be freed");
    capacityService.advance(0.75, false, true, true);
    require(capacityRequests().entries[0] && !capacityRequests().exits[0] &&
                capacityService.onboardPassengerCount() == 0,
            "entry stays requested once alighting frees the seat");
    capacityService.advance(0.75, true, true, true);
    require(!capacityRequests().entries[0] && capacityService.onboardPassengerCount() == 1,
            "entry request clears after the newly available seat is filled");
    capacityService.depart();
    capacityService.setWaitingPassengers(480, 1);
    require(capacityService.arrive({.stopId = 480, .dwellSeconds = 1.0}) &&
                !capacityRequests().entries[0] && capacityRequests().exits[0],
            "terminal service requests only alighting even when a seat will become available");
    MapPassengerServiceRuntime noSeatService(0);
    noSeatService.setWaitingPassengers(490, 1);
    require(noSeatService.arrive({.stopId = 490, .dwellSeconds = 1.0, .nextStopId = 500}) &&
                !openbus::app::vehicle::passengerServiceRequestVariables(0, 0, noSeatService)
                     .entries[0],
            "a vehicle with no seats does not request boarding");

    MapPassengerServiceRuntime boardingService(1);
    boardingService.setWaitingPassengers(600, 3);
    require(boardingService.arrive({.stopId = 600, .dwellSeconds = 0.75, .nextStopId = 700}),
            "boarding animation service starts at a valid stop");
    boardingService.advance(0.375);
    require(boardingService.boardingPassenger() &&
                std::abs(boardingService.boardingProgress() - 0.5) < 1.0e-9 &&
                boardingService.boardingSeatIndex() == 0 &&
                boardingService.boardingPassengerModelIndex() == 602 &&
                boardingService.passengerCountOnSeat(0) == 0 &&
                boardingService.waitingPassengers(600) == 2,
            "a reserved passenger has visible intermediate boarding progress");
    boardingService.setPassengerPathLinkProgressProfile(
        {{0.8, std::size_t{1}}, {1.0, std::size_t{2}}});
    require(boardingService.passengerCountOnPathLink(0) == 0 &&
                boardingService.passengerCountOnPathLink(1) == 1,
            "an active boarding passenger occupies only its current cabin pathlink");
    boardingService.advance(0.15);
    require(boardingService.passengerCountOnPathLink(1) == 1,
            "simulation progress retains occupancy within the current pathlink");
    boardingService.advance(0.08);
    require(boardingService.passengerCountOnPathLink(1) == 0 &&
                boardingService.passengerCountOnPathLink(2) == 1,
            "simulation progress advances a boarding passenger to the next pathlink");
    boardingService.advance(0.145);
    require(!boardingService.boardingPassenger() && !boardingService.boardingSeatIndex() &&
                boardingService.passengerCountOnSeat(0) == 1 &&
                boardingService.passengerCountOnPathLink(1) == 0 &&
                boardingService.passengerModelIndexOnSeat(0) == 602,
            "boarding clears pathlink occupancy and preserves the passenger identity at the seat");
    require(boardingService.arrive({.stopId = 700, .dwellSeconds = 0.75, .nextStopId = 0}),
            "passenger destination opens an alighting animation");
    boardingService.advance(0.375);
    require(boardingService.alightingPassenger() &&
                boardingService.passengerModelIndexOnSeat(0) == 602,
            "alighting preserves the seated passenger identity in motion");
    boardingService.advance(0.375);
    require(!boardingService.alightingPassenger() && !boardingService.passengerModelIndexOnSeat(0),
            "passenger identity is released when alighting completes");
}

void verifyIntegratedPassengerJourney() {
    using namespace openbus::map;
    MapTrafficLaneNetwork lanes;
    auto spline = makeSpline(9902, 0, 0, 0.0);
    spline.length = 100.0;
    lanes.addSegment({{spline, 0, 0}});
    lanes.finalize();
    const auto pedestrian = lanes.passengerJourneyNear({9902}, {5.0, 0.0, 2.0}, 7);
    require(pedestrian && pedestrian->route,
            "integrated passenger fixture resolves a directed pedestrian journey");
    const auto destinationPose = sampleMapTrafficRoute(
        *pedestrian->route, pedestrian->destinationDistanceMeters, 0.0);
    require(destinationPose.has_value(), "integrated passenger destination can be sampled");

    openbus::app::PassengerNetwork network;
    openbus::app::PassengerStopRouteState route;
    route.timetableKey = "integrated-passenger";
    route.stops =
        openbus::app::PassengerStops({{.stopId = 710, .waitingPassengers = 1},
                                      {.stopId = 720}});
    route.stopPoses = {MapTrafficPose{{5.0, 0.0, 2.0}, 0.0}, *destinationPose};
    network.routes.push_back(route);
    network.initializeQueues(route.stops.values());

    MapPassengerServiceRuntime service(1);
    require(service.arrive({.stopId = 710, .dwellSeconds = 5.0, .nextStopId = 720}),
            "integrated passenger service opens at the journey origin");
    const auto rider = network.nextPassenger(710, 0, service, 0);
    require(rider.has_value(), "integrated passenger service selects its assigned rider");
    network.resolvePassengerPedestrianJourney(710, *rider, pedestrian);
    network.advanceService(service, 0, rider, 0.1, true, true, true);
    service.setPassengerAnimationDuration(0.75);
    network.advanceService(service, 0, std::nullopt, 0.75, true, true, true);
    require(service.passengerPedestrianJourneyOnSeat(0).has_value(),
            "boarding preserves the rider's pedestrian destination");
    const auto seatedModelIndex = service.passengerModelIndexOnSeat(0);
    require(seatedModelIndex.has_value(), "integrated rider retains its model identity");

    const auto& destination = pedestrian->destination;
    service.requestStopForPassengers(
        720, {destination[0] - 200.01, destination[1], destination[2]}, {5.0, 0.0, 2.0});
    require(service.passengerStopRequestSequence() == 0,
            "integrated rider does not request the stop just beyond 200 metres");
    service.requestStopForPassengers(
        720, {destination[0] - 200.0, destination[1], destination[2]}, {5.0, 0.0, 2.0});
    require(service.passengerStopRequestSequence() == 1,
            "integrated rider requests the stop at the 200-metre boundary");
    network.departService(service);

    const auto side = mapTrafficPassengerWorldOffset(
        false, destinationPose->headingRadians, 1.0, 0.0);
    MapTrafficPose busPose = *destinationPose;
    MapTrafficPose exitPose = *destinationPose;
    for (std::size_t axis = 0; axis < 2; ++axis) {
        busPose.position[axis] -= 2.0 * side[axis];
        exitPose.position[axis] -= 0.25 * side[axis];
    }
    require(service.arrive({.stopId = 720,
                            .dwellSeconds = 1.0,
                            .alightingPositions = {exitPose.position}}) &&
                service.alightingPassenger(),
            "the destination stop starts normal alighting at the actual exit position");
    service.setPassengerAnimationDuration(0.75);
    service.advance(0.75, false, true);
    require(service.onboardPassengerCount() == 0,
            "integrated rider leaves the bus at the assigned destination");

    auto handoff = openbus::app::makePassengerPedestrianHandoff(
        *pedestrian, exitPose, busPose, false, *seatedModelIndex);
    require(handoff && !openbus::app::advancePassengerPedestrianHandoff(*handoff, 100.0),
            "the alighted rider completes the directed pedestrian handoff");
}

void verifyTrafficWaterClearance() {
    using namespace openbus::map;
    MapTrafficRouteSegment segment;
    segment.spline.placement = makeSpline(1, 0, 0, 10.0);
    segment.spline.placement.elevation = -0.1;
    segment.pathHeightMeters = 0.1;

    WaterData water;
    water.surfaces.push_back({{0.0F, 0.0F, 0.0F, 0.0F}});
    require(!mapTrafficRouteSegmentClearsWater(segment, water, OMSI_TILE_SIZE_METERS),
            "traffic lane at water level must not be used");

    segment.pathHeightMeters = 0.5;
    require(mapTrafficRouteSegmentClearsWater(segment, water, OMSI_TILE_SIZE_METERS),
            "traffic lane above water must remain usable");
    require(mapTrafficRouteSegmentClearsWater(segment, WaterData{}, OMSI_TILE_SIZE_METERS),
            "traffic lane in a tile without water must remain usable");

    segment.spline.placement.elevation = 0.5;
    segment.pathHeightMeters = 0.0;
    segment.spline.placement.gradientStart = 0.0;
    segment.spline.placement.gradientEnd = -20.0;
    require(!mapTrafficRouteSegmentClearsWater(segment, water, OMSI_TILE_SIZE_METERS),
            "traffic lane that crosses below water must be excluded");

    MapTrafficRouteSegment junctionPath;
    junctionPath.spline.tileX = 0;
    junctionPath.spline.tileY = 0;
    junctionPath.points = {{{2.0, 10.0, -0.5}}, {{12.0, 10.0, -0.5}}};
    require(!mapTrafficRouteSegmentClearsWater(junctionPath, water, OMSI_TILE_SIZE_METERS),
            "scenery junction paths below water must also be excluded");
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
    {
        MapTrafficRouteSegment approach = straight(2300, 0.0, -20.0, 0.0, 20.0);
        MapTrafficRouteSegment junction;
        junction.points = {{0.0, 0.0, 0.0}, {0.0, 4.0, 0.0}};
        const auto routeWithExit = [&](double outgoingHeading) {
            MapTrafficRoute route;
            route.segments = {approach, junction, straight(2301, 0.0, 4.0, outgoingHeading, 30.0)};
            route.cumulativeEndDistances = {20.0, 24.0, 54.0};
            route.totalLength = 54.0;
            return route;
        };
        MapTrafficActor shallowRight;
        shallowRight.route = routeWithExit(15.0);
        require(mapTrafficActorTurnSignal(shallowRight) == MapTrafficTurnSignal::Right &&
                    mapTrafficActorTurnSignal(shallowRight, 10.0) == MapTrafficTurnSignal::None,
                "a shallow selected junction exit signals right within the approach lookahead");
        shallowRight.distanceMeters = 21.0;
        require(mapTrafficActorTurnSignal(shallowRight) == MapTrafficTurnSignal::Right,
                "a junction turn signal remains active while the vehicle crosses the junction");
        shallowRight.distanceMeters = 27.0;
        require(mapTrafficActorTurnSignal(shallowRight) == MapTrafficTurnSignal::None,
                "a junction turn signal clears after the vehicle exits the junction");

        MapTrafficActor shallowLeft;
        shallowLeft.route = routeWithExit(345.0);
        require(mapTrafficActorTurnSignal(shallowLeft) == MapTrafficTurnSignal::Left,
                "a shallow selected junction exit signals left");

        MapTrafficRoute directTurnRoute;
        directTurnRoute.segments = {approach, straight(2302, 0.0, 0.0, 90.0, 30.0)};
        directTurnRoute.cumulativeEndDistances = {20.0, 50.0};
        directTurnRoute.totalLength = 50.0;
        MapTrafficActor directTurn;
        directTurn.route = std::move(directTurnRoute);
        require(mapTrafficActorTurnSignal(directTurn) == MapTrafficTurnSignal::Right,
                "a sharp outgoing road signals by its selected route direction");
        MapTrafficRouteSegment thresholdExit = straight(2305, 0.0, 0.0, 35.0, 30.0);
        MapTrafficRoute thresholdRoute;
        thresholdRoute.segments = {approach, thresholdExit};
        thresholdRoute.cumulativeEndDistances = {20.0, 50.0};
        thresholdRoute.totalLength = 50.0;
        MapTrafficActor thresholdTurn;
        thresholdTurn.route = std::move(thresholdRoute);
        require(mapTrafficActorTurnSignal(thresholdTurn) == MapTrafficTurnSignal::Right,
                "a distinct outgoing road signals once its direction change exceeds 30 degrees");
        MapTrafficRouteSegment slightExit = straight(2306, 0.0, 0.0, 20.0, 30.0);
        MapTrafficRoute slightRoute;
        slightRoute.segments = {approach, slightExit};
        slightRoute.cumulativeEndDistances = {20.0, 50.0};
        slightRoute.totalLength = 50.0;
        MapTrafficActor slightTurn;
        slightTurn.route = std::move(slightRoute);
        require(mapTrafficActorTurnSignal(slightTurn) == MapTrafficTurnSignal::None,
                "a shallow non-junction spline transition does not trigger a turn signal");

        MapTrafficRoute continuousRoute;
        continuousRoute.segments = {approach, straight(2303, 0.0, 0.0, 0.0, 30.0)};
        continuousRoute.cumulativeEndDistances = {20.0, 50.0};
        continuousRoute.totalLength = 50.0;
        MapTrafficActor continuousBend;
        continuousBend.route = std::move(continuousRoute);
        require(mapTrafficActorTurnSignal(continuousBend) == MapTrafficTurnSignal::None,
                "a smooth road continuation does not trigger a turn signal");
        MapTrafficRouteSegment curvedSignalSegment = straight(2307, 0.0, 0.0, 0.0, 35.0);
        curvedSignalSegment.spline.placement.radius = 50.0;
        MapTrafficRoute curvedRoute = oneSegment(curvedSignalSegment);
        MapTrafficActor curvedRoad;
        curvedRoad.route = std::move(curvedRoute);
        require(mapTrafficActorTurnSignal(curvedRoad) == MapTrafficTurnSignal::None,
                "a continuous bend within one spline does not trigger a turn signal");

        MapTrafficRouteSegment adjacentLane = straight(2304, 0.0, 0.0, 0.0, 30.0);
        adjacentLane.pathLateralMeters = 3.5;
        adjacentLane.splineDistanceAtRouteStartMeters = 20.0;
        MapTrafficRoute laneChangeRoute;
        laneChangeRoute.segments = {straight(2304, 0.0, -20.0, 0.0, 20.0), adjacentLane};
        laneChangeRoute.cumulativeEndDistances = {20.0, 50.0};
        laneChangeRoute.totalLength = 50.0;
        MapTrafficActor laneChange;
        laneChange.route = std::move(laneChangeRoute);
        require(mapTrafficActorTurnSignal(laneChange) == MapTrafficTurnSignal::Right,
                "crossing into an adjacent route lane signals even without a heading change");
    }
    MapTrafficRouteSegment longStraight = straight(1899, 12.0, -7.0, 23.0, 35.0);
    longStraight.sceneryPathIndex = 0;
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
    MapTrafficRoute closedSignalRoute = splitStraightRoute;
    closedSignalRoute.closed = true;
    require(mapTrafficRouteSpawnCandidates(closedSignalRoute, 4.5).empty(),
            "road traffic does not spawn inside a subdivided signalized spline");
    const auto splitStopPosition =
        mapTrafficRoutePositionOnSpline(splitStraightRoute, 1899, 0, 15.0);
    require(splitStopPosition && splitStopPosition->segmentIndex == 1,
            "passenger stops project onto the correct subdivided spline segment");
    requireNear(splitStopPosition->distanceMeters, 15.0,
                "passenger stop route distance includes its within-spline offset");
    MapTrafficRouteSegment ordinarySpline = longStraight;
    ordinarySpline.sceneryPathIndex.reset();
    require(mapTrafficSegmentMatchesTimetablePath(ordinarySpline, 1899, 3) &&
                !mapTrafficSegmentMatchesTimetablePath(longStraight, 1899, 3),
            "passenger stop segment lookup distinguishes spline and scenery paths");
    const auto ordinaryStopPosition =
        mapTrafficRoutePositionOnSpline(oneSegment(ordinarySpline), 1899, 3, 15.0);
    require(ordinaryStopPosition && ordinaryStopPosition->distanceMeters == 15.0,
            "ordinary spline passenger stops do not require a scenery path index");
    require(!mapTrafficRoutePositionOnSpline(unsplitStraightRoute, 1899, 3, 15.0),
            "scenery passenger stops require the matching scenery path index");
    MapTrafficRouteSegment reversedSpline = longStraight;
    reversedSpline.reversed = true;
    const auto reversedStopPosition =
        mapTrafficRoutePositionOnSpline(oneSegment(reversedSpline), 1899, 0, 15.0);
    require(reversedStopPosition && reversedStopPosition->segmentIndex == 0,
            "passenger stops project onto reversed route segments");
    requireNear(reversedStopPosition->distanceMeters, 20.0,
                "reversed passenger stop offsets follow route travel direction");
    require(!mapTrafficRoutePositionOnSpline(splitStraightRoute, 1899, 0, 36.0),
            "passenger stop projection rejects positions outside the spline");
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
    MapTrafficRouteSegment unsignalizedLongStraight = longStraight;
    unsignalizedLongStraight.trafficSignal.reset();
    MapTrafficRoute mergeRoute;
    mergeRoute.segments = {unsignalizedLongStraight, junctionPath, divergingStraight};
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
    const MapTrafficPose playerSpawnPose{{0.0, 0.0, 0.0}, 0.0, 0};
    require(mapTrafficSpawnInsideForwardExclusion(
                playerSpawnPose, 4.0, 2.0, {{10.0, 0.0, 0.0}, 0.0, 0}, 2.0, 30.0) &&
                !mapTrafficSpawnInsideForwardExclusion(
                    playerSpawnPose, 4.0, 2.0, {{10.0, 4.0, 0.0}, 0.0, 0}, 2.0, 30.0) &&
                !mapTrafficSpawnInsideForwardExclusion(
                    playerSpawnPose, 4.0, 2.0, {{-5.0, 0.0, 0.0}, 0.0, 0}, 2.0, 30.0),
            "the player-front spawn exclusion covers only the forward corridor");
    requireNear(mapTrafficSpawnPriorityDistanceMeters(
                    playerSpawnPose, {{10.0, 3.0, 0.0}, 0.0, 0}, 20.0),
                3.0, "spawn priority measures distance to the predicted travel path");
    requireNear(mapTrafficSpawnPriorityDistanceMeters(
                    playerSpawnPose, {{-5.0, 0.0, 0.0}, 0.0, 0}, 20.0),
                5.0, "spawn priority also includes locations behind the player");
    MapTrafficSignalSchedule cycleSignal;
    cycleSignal.cycleDurationSeconds = 10.0;
    cycleSignal.phases = {{0, 3.0}, {6, 5.0}, {9, 2.0}};
    require(mapTrafficSignalStateAtTime(cycleSignal, 0.0) == 0 &&
                mapTrafficSignalStateAtTime(cycleSignal, 3.0) == 6 &&
                mapTrafficSignalStateAtTime(cycleSignal, 8.0) == 9 &&
                mapTrafficSignalStateAtTime(cycleSignal, 10.0) == 0,
            "signal phase schedule wraps at the configured cycle duration");
    const MapTrafficSignalSchedule signalWithCycleRemainder{10.0, {{0, 3.0}, {6, 5.0}}, false};
    require(mapTrafficSignalStateAtTime(signalWithCycleRemainder, 7.9) == 6 &&
                mapTrafficSignalStateAtTime(signalWithCycleRemainder, 8.0) == 6 &&
                mapTrafficSignalStateAtTime(signalWithCycleRemainder, 10.0) == 0,
            "the final configured traffic-light phase extends through the cycle remainder");
    const MapTrafficSignalSchedule explicitDarkPhase{10.0, {{0, 3.0}, {6, 5.0}, {12, 2.0}}, false};
    require(mapTrafficSignalStateAtTime(explicitDarkPhase, 8.0) == 12,
            "an explicitly configured OMSI off phase remains dark");
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
    MapTrafficRoute signalStopLineRoute;
    signalStopLineRoute.segments = {straight(1899, 0.0, 0.0, 0.0, 10.0),
                                    straight(1898, 0.0, 10.0, 0.0, 20.0)};
    signalStopLineRoute.segments.back().trafficSignal = cycleSignal;
    signalStopLineRoute.cumulativeEndDistances = {10.0, 30.0};
    signalStopLineRoute.totalLength = 30.0;
    auto shortSignalVehicle =
        makeMapTrafficActor(signalStopLineRoute, "short.ovh", 0.0, 0.0, false);
    auto longSignalVehicle = makeMapTrafficActor(signalStopLineRoute, "long.ovh", 0.0, 0.0, false);
    shortSignalVehicle.distanceMeters = 2.0;
    longSignalVehicle.distanceMeters = 2.0;
    shortSignalVehicle.lengthMeters = 4.0;
    longSignalVehicle.lengthMeters = 12.0;
    const auto shortSignalLine = mapTrafficNextSignalStopDebugInfo(shortSignalVehicle, 0.0);
    const auto longSignalLine = mapTrafficNextSignalStopDebugInfo(longSignalVehicle, 0.0);
    const auto expectedSignalLine = sampleMapTrafficRoute(signalStopLineRoute, 9.25, 0.0);
    require(shortSignalLine.has_value() && longSignalLine.has_value() &&
                expectedSignalLine.has_value() &&
                std::hypot(shortSignalLine->pose.position[0] - longSignalLine->pose.position[0],
                           shortSignalLine->pose.position[1] - longSignalLine->pose.position[1]) <
                    1.0e-6 &&
                std::hypot(shortSignalLine->pose.position[0] - expectedSignalLine->position[0],
                           shortSignalLine->pose.position[1] - expectedSignalLine->position[1]) <
                    1.0e-6,
            "signal stopline debug position stays fixed regardless of vehicle length");
    auto heldSignalProgram = std::make_shared<MapTrafficSignalProgram>();
    heldSignalProgram->cycleDurationSeconds = 10.0;
    heldSignalProgram->lights = {{{6, 5.0}, {0, 5.0}}};
    heldSignalProgram->approachDistancesMeters = {std::nullopt};
    heldSignalProgram->controlPoints = {{0, 3.0, true}};
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

    auto jumpProgram = std::make_shared<MapTrafficSignalProgram>();
    jumpProgram->cycleDurationSeconds = 10.0;
    jumpProgram->lights = {{{0, 2.0}, {6, 3.0}, {9, 5.0}}};
    jumpProgram->approachDistancesMeters = {std::nullopt};
    jumpProgram->controlPoints = {{0, 1.0, true, 5.0}};
    MapTrafficSignalController unrequestedJump(jumpProgram);
    unrequestedJump.advance(2.0, 0.0);
    requireNear(unrequestedJump.clockSeconds(), 6.0,
                "an if_request jump skips to its target when the light has no request");
    MapTrafficSignalController requestedJump(jumpProgram);
    requestedJump.request(0);
    requestedJump.advance(2.0, 0.0);
    requireNear(requestedJump.clockSeconds(), 2.0,
                "an if_request jump does not fire while a vehicle requests its light");

    auto rewindProgram = std::make_shared<MapTrafficSignalProgram>();
    rewindProgram->cycleDurationSeconds = 10.0;
    rewindProgram->lights = {{{0, 2.0}, {6, 3.0}, {0, 5.0}}};
    rewindProgram->approachDistancesMeters = {std::nullopt};
    rewindProgram->controlPoints = {{0, 5.0, false, 1.0}};
    MapTrafficSignalController requestedRewind(rewindProgram);
    requestedRewind.request(0);
    requestedRewind.advance(5.5, 0.0);
    requireNear(requestedRewind.clockSeconds(), 1.5,
                "a requested backward jump rewinds the shared cycle clock");
    requestedRewind.advance(0.5, 0.0);
    requireNear(requestedRewind.clockSeconds(), 2.0,
                "a backward jump does not immediately retrigger while replaying its interval");

    auto changeoverStopProgram = std::make_shared<MapTrafficSignalProgram>();
    changeoverStopProgram->cycleDurationSeconds = 10.0;
    changeoverStopProgram->lights = {{{4, 5.0}, {6, 5.0}}};
    changeoverStopProgram->approachDistancesMeters = {std::nullopt};
    changeoverStopProgram->controlPoints = {{0, 2.0, true}};
    MapTrafficSignalController changeoverStop(changeoverStopProgram);
    changeoverStop.advance(3.0, 0.0);
    requireNear(changeoverStop.clockSeconds(), 3.0,
                "a conditional stop does not hold a red-yellow changeover");

    auto boundaryJumpProgram = std::make_shared<MapTrafficSignalProgram>();
    boundaryJumpProgram->cycleDurationSeconds = 10.0;
    boundaryJumpProgram->lights = {{{0, 5.0}, {6, 5.0}}};
    boundaryJumpProgram->approachDistancesMeters = {std::nullopt};
    boundaryJumpProgram->controlPoints = {{0, 0.0, true, 10.0}};
    MapTrafficSignalController boundaryJump(boundaryJumpProgram);
    boundaryJump.advance(0.1, 0.0);
    requireNear(boundaryJump.clockSeconds(), 0.1,
                "a jump to the cycle boundary does not freeze the signal clock");

    MapTrafficSignalController dayTimeAligned(jumpProgram);
    dayTimeAligned.advance(0.0, 12.0 * 60.0 * 60.0 + 2.5);
    requireNear(dayTimeAligned.clockSeconds(), 2.5,
                "signal cycle position initializes from the selected time of day");

    MapTrafficSignalSchedule darkSignalSchedule;
    darkSignalSchedule.cycleDurationSeconds = 10.0;
    darkSignalSchedule.phases = {{12, 10.0}};
    MapTrafficRouteSegment darkSignalPath = straight(1901, 0.0, 0.0, 0.0, 10.0);
    darkSignalPath.trafficSignal = darkSignalSchedule;
    auto darkSignalActor =
        makeMapTrafficActor(oneSegment(darkSignalPath), "dark-signal-car.ovh", 0.0, 0.0, false);
    require(!mapTrafficActorWaitingAtSignal(darkSignalActor, 0.0),
            "a dark traffic signal is permissive for vehicles");
    darkSignalActor.speedMetersPerSecond = 2.0;
    darkSignalActor.cruiseSpeedMetersPerSecond = 2.0;
    advanceMapTrafficDriving({&darkSignalActor}, {}, 0.5, 0.0);
    require(darkSignalActor.distanceMeters > 0.0,
            "vehicles continue across the stopline during the dark phase");

    MapTrafficLaneNetwork junction;
    for (int index = 0; index < 4; ++index) {
        junction.addSegment(straight(1090 + index, 1000.0 + index * 20.0, 1000.0, 0.0, 5.0));
    }
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
    MapTrafficLaneNetwork requiredSequence;
    requiredSequence.addSegment(straight(1022, 0.0, 0.0, 0.0, 20.0));
    requiredSequence.addSegment(straight(1023, 0.0, 20.0, 0.0, 20.0));
    requiredSequence.finalize();
    require(!requiredSequence.buildRoute(1022, 0, 0, 0.0, {}, {1022, 1023, 1024}),
            "a required timetable path cannot succeed after reaching a shorter dead end");
    const auto disconnectedStop = requiredSequence.timetableStopPose(1023, 0, 5.0);
    require(disconnectedStop.has_value(),
            "a player stop can be sampled even when the complete AI route cannot be built");
    requireNear(disconnectedStop->position[1], 25.0,
                "a player stop uses its authored position on the individual spline");
    const auto roundedEndpoint = requiredSequence.timetableStopPose(1023, 0, 20.00001);
    require(roundedEndpoint.has_value(),
            "rounded timetable lengths still locate stops at a spline endpoint");
    require(!requiredSequence.timetableStopPose(1023, 0, 20.1),
            "a materially out-of-range stop offset is not clamped to the endpoint");
    require(!requiredSequence.timetableStopPose(1024, 0, 0.0),
            "missing player stop geometry is reported rather than fabricated");
    const auto completeRequiredSequence =
        requiredSequence.buildRoute(1022, 0, 0, 0.0, {}, {1022, 1023});
    require(completeRequiredSequence && completeRequiredSequence->segments.size() == 2,
            "an endpoint route succeeds when it completes the required timetable sequence");
    MapTrafficLaneNetwork intermediaryRequiredSequence;
    intermediaryRequiredSequence.addSegment(straight(1030, 0.0, 0.0, 0.0, 20.0));
    intermediaryRequiredSequence.addSegment(straight(1031, 0.0, 20.0, 0.0, 20.0));
    intermediaryRequiredSequence.addSegment(straight(1032, 0.0, 40.0, 0.0, 20.0));
    intermediaryRequiredSequence.finalize();
    require(!intermediaryRequiredSequence.buildRoute(1030, 0, 0, 0.0, {}, {1030, 1032}),
            "timetable checkpoints remain adjacent by default");
    const auto intermediaryMatchedRoute =
        intermediaryRequiredSequence.buildRoute(1030, 0, 0, 0.0, {}, {1030, 1032}, {}, true, true);
    require(intermediaryMatchedRoute && intermediaryMatchedRoute->segments.size() == 3 &&
                mapTrafficTimetablePathContainedInRouteInOrder(
                    {{1030, std::nullopt}, {1032, std::nullopt}},
                    {{1030, std::nullopt}, {1031, std::nullopt}, {1032, std::nullopt}}),
            "scheduled path search traverses connected intermediary lanes between checkpoints");
    const auto repeatedIntermediaryCheckpointRoute = intermediaryRequiredSequence.buildRoute(
        1030, 0, 0, 0.0, {}, {1030, 1030, 1032}, {}, true, true);
    require(repeatedIntermediaryCheckpointRoute &&
                repeatedIntermediaryCheckpointRoute->segments.size() == 3,
            "connected route search consumes consecutive duplicate timetable checkpoints");
    MapTrafficLaneNetwork outOfOrderWaypointNetwork;
    outOfOrderWaypointNetwork.addSegment(straight(1040, 0.0, 0.0, 0.0, 20.0));
    outOfOrderWaypointNetwork.addSegment(straight(1042, 0.0, 20.0, 0.0, 20.0));
    outOfOrderWaypointNetwork.addSegment(straight(1041, 0.0, 40.0, 0.0, 20.0));
    outOfOrderWaypointNetwork.addSegment(straight(1042, 0.0, 60.0, 0.0, 20.0));
    outOfOrderWaypointNetwork.addSegment(straight(1043, 0.0, 80.0, 0.0, 20.0));
    outOfOrderWaypointNetwork.finalize();
    const auto waypointConnectionGap =
        outOfOrderWaypointNetwork.nearestConnectionGap(1040, std::nullopt, 1041, std::nullopt);
    require(waypointConnectionGap && waypointConnectionGap->horizontalMeters == 20.0 &&
                waypointConnectionGap->verticalMeters == 0.0 &&
                waypointConnectionGap->headingAlignment > 0.99,
            "connection diagnostics report the closest directed endpoint gap and alignment");
    const auto outOfOrderWaypointRoute = outOfOrderWaypointNetwork.buildRoute(
        1040, 0, 0, 0.0, {}, {1040, 1041, 1042, 1043}, {}, true, true);
    require(outOfOrderWaypointRoute && outOfOrderWaypointRoute->segments.size() == 5 &&
                mapTrafficTimetablePathContainedInRouteInOrder(
                    {{1040, std::nullopt},
                     {1041, std::nullopt},
                     {1042, std::nullopt},
                     {1043, std::nullopt}},
                    [&] {
                        std::vector<MapTrafficRouteRequirement> routeRequirements;
                        for (const auto& segment : outOfOrderWaypointRoute->segments) {
                            routeRequirements.push_back(
                                {segment.spline.placement.splineId, segment.sceneryPathIndex});
                        }
                        return routeRequirements;
                    }()),
            "waypoint search can pass a later checkpoint before matching the required order");
    MapTrafficLaneNetwork timetableReverseRouteNetwork;
    timetableReverseRouteNetwork.addSegment(straight(1050, 0.0, 0.0, 0.0, 20.0));
    timetableReverseRouteNetwork.addSegment(straight(1051, 0.0, 40.0, 180.0, 20.0));
    timetableReverseRouteNetwork.addSegment(straight(1052, 0.0, 60.0, 180.0, 20.0));
    timetableReverseRouteNetwork.finalize();
    require(!timetableReverseRouteNetwork.buildRoute(1050, 0, 0, 0.0, {}, {1050, 1051, 1052}, {},
                                                     true, true),
            "schedule-directed reverse traversal is disabled by default");
    const auto timetableReverseRoute = timetableReverseRouteNetwork.buildRoute(
        1050, 0, 0, 0.0, {}, {1050, 1051, 1052}, {}, true, true, true);
    require(
        timetableReverseRoute && timetableReverseRoute->segments.size() == 3 &&
            timetableReverseRoute->segments[1].reversed &&
            timetableReverseRoute->segments[2].reversed,
        "scheduled route requirements can authorize reverse travel through a linked spline chain");
    const auto reverseRouteStart = sampleMapTrafficRoute(*timetableReverseRoute, 20.0, 0.0);
    const auto reverseRouteMiddle = sampleMapTrafficRoute(*timetableReverseRoute, 40.0, 0.0);
    require(reverseRouteStart && reverseRouteMiddle &&
                std::abs(reverseRouteStart->position[1] - 20.0) < 1.0e-6 &&
                std::abs(reverseRouteMiddle->position[1] - 40.0) < 1.0e-6,
            "schedule-directed reversed spline segments sample continuously at their joins");
    MapTrafficLaneNetwork repeatedTimetableSpline;
    repeatedTimetableSpline.addSegment(straight(1027, 0.0, 0.0, 0.0, 20.0));
    repeatedTimetableSpline.addSegment(straight(1028, 0.0, 20.0, 0.0, 20.0));
    repeatedTimetableSpline.addSegment(straight(1029, 0.0, 40.0, 0.0, 20.0));
    repeatedTimetableSpline.finalize();
    const auto repeatedSplineRoute = repeatedTimetableSpline.buildRoute(
        1027, 0, 0, 0.0, {}, {1027, 1027, 1028, 1028, 1029}, {47, 46, 24, 23, 0}, true);
    require(repeatedSplineRoute && repeatedSplineRoute->segments.size() == 3 &&
                repeatedSplineRoute->segments.back().spline.placement.splineId == 1029,
            "repeated timetable checkpoints on ordinary splines collapse to one route segment");
    MapTrafficLaneNetwork scheduledHeightStep;
    MapTrafficRouteSegment scheduledIncoming;
    scheduledIncoming.spline.placement.splineId = 1025;
    scheduledIncoming.points = {{0.0, 0.0, 0.0}, {0.0, 10.0, 0.0}};
    scheduledIncoming.sceneryPathIndex = 3;
    MapTrafficRouteSegment scheduledOutgoing;
    scheduledOutgoing.spline.placement.splineId = 1026;
    scheduledOutgoing.points = {{0.1, 10.0, 1.98}, {0.1, 20.0, 2.18}};
    scheduledOutgoing.sceneryPathIndex = 7;
    scheduledHeightStep.addSegment(std::move(scheduledIncoming));
    scheduledHeightStep.addSegment(std::move(scheduledOutgoing));
    scheduledHeightStep.finalize();
    const auto unconstrainedHeightStepRoute = scheduledHeightStep.buildRoute(1025);
    require(unconstrainedHeightStepRoute && unconstrainedHeightStepRoute->segments.size() == 1,
            "ordinary road routing does not connect lanes with a large vertical step");
    const auto scheduledHeightStepRoute =
        scheduledHeightStep.buildRoute(1025, 0, 0, 0.0, {}, {1025, 1026}, {3, 7}, true);
    require(scheduledHeightStepRoute && scheduledHeightStepRoute->segments.size() == 2 &&
                scheduledHeightStepRoute->segments[1].points.front() ==
                    scheduledHeightStepRoute->segments[0].points.back() &&
                std::abs(scheduledHeightStepRoute->segments[1].points.back()[2] - 0.2) < 1.0e-8,
            "an exact timetable path selector may bridge a small authored vertical step");
    MapTrafficLaneNetwork sceneryTimetableRoute;
    auto invisibleRoadSpline = straight(15155, 0.0, 0.0, 0.0, 20.0);
    auto placedJunctionPath = straight(10129, 0.0, 20.0, 0.0, 20.0);
    sceneryTimetableRoute.addSegment(std::move(invisibleRoadSpline));
    sceneryTimetableRoute.addSegment(std::move(placedJunctionPath));
    sceneryTimetableRoute.finalize();
    const auto sceneryTimetableMatch =
        sceneryTimetableRoute.buildRoute(15155, 0, 0, 0.0, {}, {15155, 10129});
    require(sceneryTimetableMatch && sceneryTimetableMatch->segments.size() == 2 &&
                sceneryTimetableMatch->segments[1].spline.placement.splineId == 10129,
            "a placed scenery traffic path retains its map object ID for timetable matching");
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
    require(connectedSpawnLanes.selectSpawnLaneInTiles({0, std::nullopt, false}, 1, {{0, 0}})
                .has_value() &&
                !connectedSpawnLanes
                     .selectSpawnLaneInTiles({0, std::nullopt, false}, 1, {{5, 5}})
                     .has_value(),
            "local spawn lane selection excludes roads outside the streamed tile window");
    MapTrafficLaneNetwork streamedSpawnLanes;
    const auto addStreamedLanes = [&](int tile) {
        auto incoming = straight(1100 + tile * 2, 0.0, 0.0, 0.0, 20.0);
        auto outgoing = straight(1101 + tile * 2, 0.0, 20.0, 0.0, 20.0);
        incoming.spline.tileX = tile;
        outgoing.spline.tileX = tile;
        streamedSpawnLanes.addSegment(std::move(incoming));
        streamedSpawnLanes.addSegment(std::move(outgoing));
    };
    addStreamedLanes(0);
    streamedSpawnLanes.finalize();
    const auto oldLocalLane =
        streamedSpawnLanes.selectSpawnLaneInTiles({0, std::nullopt, false}, 1, {{0, 0}});
    require(!streamedSpawnLanes.selectSpawnLaneInTiles({0, std::nullopt, false}, 1, {{3, 0}}),
            "an unloaded tile has no local spawn lane");
    addStreamedLanes(3);
    streamedSpawnLanes.finalize();
    const auto newLocalLane =
        streamedSpawnLanes.selectSpawnLaneInTiles({0, std::nullopt, false}, 1, {{3, 0}});
    require(oldLocalLane == std::optional<MapTrafficLaneStart>{{1100, 0}} &&
                newLocalLane == std::optional<MapTrafficLaneStart>{{1106, 0}},
            "streaming to a distant tile selects that tile's connected road lane");
    const auto newLocalRoute =
        streamedSpawnLanes.buildRoute(newLocalLane->splineId, newLocalLane->laneSelector);
    require(newLocalRoute &&
                mapTrafficSpawnCandidatesOverlapActiveTiles(
                    mapTrafficRouteSpawnCandidates(*newLocalRoute, 4.5), {{3, 0}}),
            "the replacement route provides a spawn candidate in the new active tile");
    const auto connectedSpawnRoute = connectedSpawnLanes.buildRoute(1010);
    require(connectedSpawnRoute.has_value(),
            "a connected road lane builds a route for interior spawn candidates");
    const auto spawnCandidates = mapTrafficRouteSpawnCandidates(*connectedSpawnRoute, 4.5);
    require(spawnCandidates.size() == 2 && connectedSpawnRoute->startsAtEntryLane,
            "road spawn candidates include connected route interiors, not terminal dead ends");
    requireNear(spawnCandidates[0].distanceMeters, 8.0,
                "road spawn candidates keep a vehicle-sized margin from the route start");
    requireNear(spawnCandidates[1].distanceMeters, 28.0,
                "road spawn candidates include interior positions beyond the first lane");
    const auto subdividedEntryCandidates =
        mapTrafficRouteSpawnCandidates(subdivideMapTrafficRoute(*connectedSpawnRoute), 4.5);
    const auto internalRoute = connectedSpawnLanes.buildRoute(1011);
    require(internalRoute && !internalRoute->startsAtEntryLane &&
                spawnCandidates.front().entryPreferenceMeters == 10.0 &&
                subdividedEntryCandidates.front().entryPreferenceMeters == 10.0 &&
                mapTrafficRouteSpawnCandidates(*internalRoute, 4.5).front().entryPreferenceMeters ==
                    0.0,
            "only actual network entry lanes receive a spawn preference, including after subdivision");
    require(spawnCandidates[0].tileX == 0 && spawnCandidates[0].tileY == 0 &&
                spawnCandidates[1].tileX == 0 && spawnCandidates[1].tileY == 0,
            "road spawn candidates report their sampled world tile");
    MapTrafficLaneNetwork sceneryRoadLanes;
    MapTrafficRouteSegment sceneryRoadStart;
    sceneryRoadStart.spline.placement.splineId = 1020;
    sceneryRoadStart.spline.coordinates.tileSize = OMSI_TILE_SIZE_METERS;
    sceneryRoadStart.points = {{0.0, 0.0, 0.0}, {0.0, 30.0, 0.0}};
    MapTrafficRouteSegment sceneryRoadEnd;
    sceneryRoadEnd.spline.placement.splineId = 1021;
    sceneryRoadEnd.spline.coordinates.tileSize = OMSI_TILE_SIZE_METERS;
    sceneryRoadEnd.points = {{0.0, 30.0, 0.0}, {0.0, 60.0, 0.0}};
    sceneryRoadLanes.addSegment(std::move(sceneryRoadStart));
    sceneryRoadLanes.addSegment(std::move(sceneryRoadEnd));
    sceneryRoadLanes.finalize();
    require(sceneryRoadLanes.spawnLanes() == std::vector<MapTrafficLaneStart>{{1020, 0}},
            "connected point-backed scenery roads are eligible traffic spawn lanes");
    const auto sceneryRoadRoute = sceneryRoadLanes.buildRoute(1020);
    require(sceneryRoadRoute.has_value(),
            "point-backed scenery road lanes build routes for spawn candidates");
    const auto scenerySpawnCandidates =
        mapTrafficRouteSpawnCandidates(*sceneryRoadRoute, 4.5);
    require(scenerySpawnCandidates.size() == 2 &&
                scenerySpawnCandidates[0].tileX == 0 && scenerySpawnCandidates[0].tileY == 0 &&
                scenerySpawnCandidates[1].tileX == 0 && scenerySpawnCandidates[1].tileY == 0,
            "point-backed scenery road interiors produce world-tile spawn candidates");
    openbus::map::MapTrafficRoute signalizedSpawnRoute;
    openbus::map::MapTrafficRouteSegment signalizedSpawnPath;
    signalizedSpawnPath.spline.placement.splineId = 1022;
    signalizedSpawnPath.spline.coordinates.tileSize = OMSI_TILE_SIZE_METERS;
    signalizedSpawnPath.sceneryPathIndex = 0;
    signalizedSpawnPath.points = {{0.0, 0.0, 0.0}, {0.0, 60.0, 0.0}};
    signalizedSpawnPath.trafficSignal = openbus::map::MapTrafficSignalSchedule{};
    openbus::map::MapTrafficRouteSegment postSignalSpawnPath;
    postSignalSpawnPath.spline.placement.splineId = 1022;
    postSignalSpawnPath.spline.coordinates.tileSize = OMSI_TILE_SIZE_METERS;
    postSignalSpawnPath.sceneryPathIndex = 1;
    postSignalSpawnPath.points = {{0.0, 60.0, 0.0}, {0.0, 120.0, 0.0}};
    signalizedSpawnRoute.segments = {signalizedSpawnPath, postSignalSpawnPath};
    signalizedSpawnRoute.cumulativeEndDistances = {60.0, 120.0};
    signalizedSpawnRoute.totalLength = 120.0;
    signalizedSpawnRoute.closed = true;
    const auto signalSafeSpawnCandidates =
        mapTrafficRouteSpawnCandidates(signalizedSpawnRoute, 4.5);
    require(!signalSafeSpawnCandidates.empty() &&
                std::all_of(signalSafeSpawnCandidates.begin(), signalSafeSpawnCandidates.end(),
                            [](const openbus::map::MapTrafficSpawnCandidate& candidate) {
                                return candidate.distanceMeters >= 60.0;
                            }),
            "road traffic never spawns inside a signalized SCO path beyond its stopline");
    require(std::all_of(signalSafeSpawnCandidates.begin(), signalSafeSpawnCandidates.end(),
                        [](const openbus::map::MapTrafficSpawnCandidate& candidate) {
                            return candidate.locationPenaltyMeters >= 20.0;
                        }),
            "SCO junction paths are less preferred than ordinary road interiors");
    require(signalSafeSpawnCandidates.back().locationPenaltyMeters == 29.0,
            "signal approach penalties account for a signal across a closed route boundary");
    auto approachRoute = signalizedSpawnRoute;
    approachRoute.segments[0].trafficSignal.reset();
    approachRoute.segments[0].sceneryPathIndex.reset();
    approachRoute.segments[1].trafficSignal = openbus::map::MapTrafficSignalSchedule{};
    const auto approachCandidates = mapTrafficRouteSpawnCandidates(approachRoute, 4.5);
    require(approachCandidates.size() == 4 &&
                approachCandidates.front().locationPenaltyMeters == 0.0 &&
                approachCandidates.back().locationPenaltyMeters == 9.0,
            "candidates close to an upcoming signal stopline receive a graded penalty");
    MapTrafficLaneNetwork crossTileSpawnLanes;
    crossTileSpawnLanes.addSegment(
        straight(1013, 0.0, OMSI_TILE_SIZE_METERS - 60.0, 0.0, 120.0));
    crossTileSpawnLanes.addSegment(
        straight(1014, 0.0, OMSI_TILE_SIZE_METERS + 60.0, 0.0, 30.0));
    crossTileSpawnLanes.finalize();
    const auto crossTileSpawnRoute = crossTileSpawnLanes.buildRoute(1013);
    require(crossTileSpawnRoute.has_value(),
            "a connected route can cross an active-tile boundary");
    const auto crossTileSpawnCandidates =
        mapTrafficRouteSpawnCandidates(*crossTileSpawnRoute, 4.5);
    require(std::all_of(crossTileSpawnCandidates.begin(), crossTileSpawnCandidates.end(),
                         [&](const auto& candidate) {
                             const auto pose = sampleMapTrafficRoute(
                                 *crossTileSpawnRoute, candidate.distanceMeters, 0.0);
                             return pose && mapTrafficPoseOnActiveTile(
                                                *pose, {{candidate.tileX, candidate.tileY}},
                                                OMSI_TILE_SIZE_METERS);
                         }),
            "spawn candidate tiles match their sampled world positions");
    require(std::any_of(crossTileSpawnCandidates.begin(), crossTileSpawnCandidates.end(),
                         [](const auto& candidate) {
                             return candidate.tileY == 1;
                         }),
            "spawn candidates crossing a tile boundary use the tile containing their pose");
    require(mapTrafficSpawnCandidatesOverlapActiveTiles(crossTileSpawnCandidates,
                                                        {{0, 0}, {0, 1}}) &&
                !mapTrafficSpawnCandidatesOverlapActiveTiles(crossTileSpawnCandidates, {{2, 2}}),
            "active route admission follows candidate positions rather than source spline tiles");
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
    for (std::size_t seed = 1; seed <= 12; ++seed) {
        const auto route = junction.buildRoute(1001, 0, seed, 4.5);
        require(route.has_value(), "a normal car can route through an authored junction");
        for (double distance = 0.0; distance < route->totalLength; distance += 0.25) {
            require(mapTrafficCurveSpeedLimit(*route, distance, 12.0, 4.5) > 0.0,
                    "a route accepted for a vehicle never produces a permanent zero curve speed");
        }
        const int exit = route->segments.back().spline.placement.splineId;
        const auto largeVehicle = junction.buildRoute(1001, 0, seed, 20.0);
        require(largeVehicle && largeVehicle->segments.back().spline.placement.splineId == 1002,
                "vehicles with a 20 m minimum radius reject the 10 m junction turns");
        const auto exactRadius = junction.buildRoute(1001, 0, seed, 10.0);
        require(exactRadius && exactRadius->segments.back().spline.placement.splineId == exit,
                "a vehicle with a 10 m minimum radius can use authored 10 m turns");
    }
    std::optional<MapTrafficRoute> left;
    std::optional<MapTrafficRoute> right;
    std::size_t straightSelections = 0;
    std::size_t leftSelections = 0;
    std::size_t rightSelections = 0;
    std::size_t smallPoolStraightSelections = 0;
    std::size_t smallPoolLeftSelections = 0;
    std::size_t smallPoolRightSelections = 0;
    for (std::size_t seed = 1; seed <= 1200; ++seed) {
        const auto route = junction.buildRoute(1001, 0, seed, 4.5);
        require(route.has_value(), "a seeded vehicle route is available through the junction");
        switch (route->segments.back().spline.placement.splineId) {
        case 1002:
            ++straightSelections;
            smallPoolStraightSelections += seed <= 8 ? 1U : 0U;
            break;
        case 1003:
            ++leftSelections;
            smallPoolLeftSelections += seed <= 8 ? 1U : 0U;
            left = route;
            break;
        case 1004:
            ++rightSelections;
            smallPoolRightSelections += seed <= 8 ? 1U : 0U;
            right = route;
            break;
        default:
            require(false, "junction route selects one of its three authored exits");
        }
    }
    require(straightSelections > leftSelections * 3 / 2 &&
                straightSelections > rightSelections * 3 / 2 && leftSelections > 0 &&
                rightSelections > 0 && left.has_value() && right.has_value(),
            "equal-density junction choices favor straight movement over each turn");
    require(
        smallPoolStraightSelections >= smallPoolLeftSelections &&
            smallPoolStraightSelections >= smallPoolRightSelections &&
            smallPoolLeftSelections > 0 && smallPoolRightSelections > 0,
        "a small fixed vehicle pool samples straight, left, and right exits without a branch skew");
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
    {
        auto scheduledBus = makeMapTrafficActor(oneSegment(straight(2299, 0.0, 0.0, 0.0, 100.0)),
                                                "scheduled.bus", 10.0, 0.0, false);
        scheduledBus.isBus = true;
        scheduledBus.scheduledStopDistanceMeters = 20.0;
        for (int frame = 0; frame < 200 && scheduledBus.speedMetersPerSecond > 0.0; ++frame) {
            advanceMapTrafficDriving({&scheduledBus}, {}, 0.1);
        }
        requireNear(scheduledBus.distanceMeters, 20.0,
                    "scheduled bus reaches the configured passenger-stop position");
        requireNear(scheduledBus.speedMetersPerSecond, 0.0,
                    "scheduled bus holds safely at a passenger stop");
    }
    {
        auto busPastScheduledStop = makeMapTrafficActor(
            oneSegment(straight(2298, 0.0, 0.0, 0.0, 100.0)), "scheduled.bus", 11.0, 0.0, false);
        busPastScheduledStop.distanceMeters = 0.0307274;
        busPastScheduledStop.speedMetersPerSecond = 11.0;
        busPastScheduledStop.targetSpeedMetersPerSecond = 11.0;
        busPastScheduledStop.cruiseSpeedMetersPerSecond = 11.0;
        busPastScheduledStop.scheduledStopDistanceMeters = 0.0;
        advanceMapTrafficDriving({&busPastScheduledStop}, {}, 0.1);
        requireNear(busPastScheduledStop.distanceMeters, 0.0307274,
                    "a bus already past a scheduled stop remains safely held");
        requireNear(busPastScheduledStop.speedMetersPerSecond, 0.0,
                    "a bus already past a scheduled stop brakes to a hold");
    }
    for (const auto* route : {&*left, &*right}) {
        auto actor = makeMapTrafficActor(*route, "car.ovh", 12.0, 0.0, false);
        actor.minimumTurnRadiusMeters = 10.0;
        actor.speedMetersPerSecond = 0.0;
        bool sampledTurn = false;
        bool reachedExit = false;
        double minimumTurnSpeed = std::numeric_limits<double>::infinity();
        for (int frame = 0; frame < 400; ++frame) {
            const double oldSpeed = actor.speedMetersPerSecond;
            advanceMapTrafficDriving({&actor}, {}, 0.1);
            require(actor.speedMetersPerSecond <= oldSpeed + 0.160001 &&
                        actor.speedMetersPerSecond >= oldSpeed - 0.300001,
                    "free driving respects acceleration and comfortable braking limits");
            if (actor.distanceMeters > 40.0 && actor.distanceMeters < 55.0) {
                sampledTurn = true;
                minimumTurnSpeed = std::min(minimumTurnSpeed, actor.speedMetersPerSecond);
                require(actor.speedMetersPerSecond <= 7.0,
                        "10 m radius junction turns stay below the relaxed safe-speed limit");
            }
            if (actor.distanceMeters > 60.0) {
                reachedExit = true;
                break;
            }
        }
        require(sampledTurn && reachedExit,
                "vehicles actually traverse left/right scenery curves and enter their exit lane");
        require(minimumTurnSpeed > 5.2,
                "a moderate-radius turn does not impose the previous overly low speed cap");
        const auto firstJunctionSegment = std::find_if(
            route->segments.begin(), route->segments.end(),
            [](const MapTrafficRouteSegment& segment) { return !segment.points.empty(); });
        require(firstJunctionSegment != route->segments.end(),
                "the sampled junction route contains its authored turn");
        const std::size_t junctionSegmentIndex =
            static_cast<std::size_t>(firstJunctionSegment - route->segments.begin());
        const double junctionStartDistance =
            junctionSegmentIndex == 0 ? 0.0
                                      : route->cumulativeEndDistances[junctionSegmentIndex - 1];
        const double junctionEndDistance = route->cumulativeEndDistances[junctionSegmentIndex];
        double safeTurnDistance = junctionStartDistance;
        while (safeTurnDistance < junctionEndDistance &&
               mapTrafficCurveSpeedLimit(*route, safeTurnDistance, 12.0, 4.5) < 4.0) {
            safeTurnDistance += 0.25;
        }
        require(safeTurnDistance < junctionEndDistance,
                "the turn contains a curve section safe for a slow-moving vehicle");
        auto slowTurningActor =
            makeMapTrafficActor(*route, "slow-turning-car.ovh", 12.0, 0.0, false);
        slowTurningActor.distanceMeters = safeTurnDistance;
        slowTurningActor.speedMetersPerSecond = 3.5;
        advanceMapTrafficDriving({&slowTurningActor}, {}, 0.1);
        require(slowTurningActor.speedMetersPerSecond >= 3.5,
                "a turn does not slow a vehicle already below its curvature-based safe speed");
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
    busAtJunctionRoute.segments = {straight(1101, 0.0, -40.0, 0.0, 40.0), busAtJunctionConnector,
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
            "a vehicle clears a junction with one meter of rear clearance and queues close behind "
            "a bus");
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
        auto adaptingFollower =
            makeMapTrafficActor(road, "adapting-follower.ovh", 10.0, 0.0, false);
        slowingLeader.distanceMeters = centerGapMeters;
        adaptingFollower.speedMetersPerSecond = 8.0;
        slowingLeader.speedMetersPerSecond = 6.0;
        for (int frame = 0; frame < 5; ++frame) {
            advanceMapTrafficDriving({&adaptingFollower, &slowingLeader}, {}, 0.1);
            require(mapTrafficHasClearance(footprint(adaptingFollower), {footprint(slowingLeader)},
                                           0.49),
                    "adaptive following retains physical clearance while matching a slower leader");
        }
        return adaptingFollower.speedMetersPerSecond;
    };
    const double closeFollowingSpeed = followingSpeedAfterLeaderSlows(20.0);
    const double distantFollowingSpeed = followingSpeedAfterLeaderSlows(36.0);
    require(closeFollowingSpeed + 1.0 < distantFollowingSpeed && closeFollowingSpeed > 0.0,
            "following speed adapts smoothly to a slowing leader according to the available gap");
    auto reducedDetailLeader =
        makeMapTrafficActor(road, "reduced-detail-leader.ovh", 5.0, 0.0, false);
    auto reducedDetailFollower =
        makeMapTrafficActor(road, "reduced-detail-follower.ovh", 12.0, 0.0, false);
    reducedDetailLeader.distanceMeters = 30.0;
    reducedDetailFollower.reducedSpeedLookahead = true;
    for (int frame = 0; frame < 20; ++frame) {
        advanceMapTrafficDriving({&reducedDetailFollower, &reducedDetailLeader}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(reducedDetailFollower),
                                       {footprint(reducedDetailLeader)}, 0.49),
                "reduced curve-speed sampling retains full-rate leader clearance checks");
    }
    require(reducedDetailFollower.blockReason == MapTrafficBlockReason::Leader,
            "reduced-detail vehicles still adapt speed to a slower leader");
    std::vector<MapTrafficActor> indexedFollowingActors;
    std::vector<MapTrafficActor*> indexedFollowingPointers;
    indexedFollowingActors.reserve(8);
    indexedFollowingPointers.reserve(8);
    for (std::size_t index = 0; index < 8; ++index) {
        auto indexedActor =
            makeMapTrafficActor(road, "indexed-following-" + std::to_string(index) + ".ovh",
                                12.0, 0.0, false);
        indexedActor.distanceMeters =
            index == 0 ? 100.0 : index == 1 ? 130.0 : 250.0 + (index - 2) * 45.0;
        indexedActor.speedMetersPerSecond = index == 1 ? 0.0 : 10.0;
        indexedFollowingActors.push_back(std::move(indexedActor));
    }
    for (auto& indexedActor : indexedFollowingActors) {
        indexedFollowingPointers.push_back(&indexedActor);
    }
    std::vector<MapTrafficActor> exhaustiveFollowingActors(indexedFollowingActors.begin(),
                                                           indexedFollowingActors.end() - 1);
    std::vector<MapTrafficActor*> exhaustiveFollowingPointers;
    exhaustiveFollowingPointers.reserve(exhaustiveFollowingActors.size());
    for (auto& exhaustiveActor : exhaustiveFollowingActors) {
        exhaustiveFollowingPointers.push_back(&exhaustiveActor);
    }
    for (int frame = 0; frame < 20; ++frame) {
        advanceMapTrafficDriving(indexedFollowingPointers, {}, 0.1);
        advanceMapTrafficDriving(exhaustiveFollowingPointers, {}, 0.1);
        require(indexedFollowingActors[0].blockReason == MapTrafficBlockReason::Leader,
                "large shared routes retain the nearest same-route leader for speed control");
        require(mapTrafficHasClearance(footprint(indexedFollowingActors[0]),
                                      {footprint(indexedFollowingActors[1])}, 0.49),
                "large shared routes retain full collision clearance");
        requireNear(indexedFollowingActors[0].distanceMeters,
                    exhaustiveFollowingActors[0].distanceMeters,
                    "the route-distance index matches exhaustive same-route following");
        requireNear(indexedFollowingActors[0].targetSpeedMetersPerSecond,
                    exhaustiveFollowingActors[0].targetSpeedMetersPerSecond,
                    "the route-distance index preserves the exhaustive speed target");
    }
    auto stoppedFollowingLeader =
        makeMapTrafficActor(road, "stopped-following-leader.ovh", 0.0, 0.0, false);
    auto approachingFollower =
        makeMapTrafficActor(road, "approaching-follower.ovh", 12.0, 0.0, false);
    stoppedFollowingLeader.distanceMeters = 60.0;
    advanceMapTrafficDriving({&approachingFollower, &stoppedFollowingLeader}, {}, 0.1);
    require(approachingFollower.targetSpeedMetersPerSecond < 4.5 &&
                approachingFollower.blockReason == MapTrafficBlockReason::Leader,
            "a fast follower reduces its target speed early when approaching a stopped leader");
    {
        auto waitingLeader = makeMapTrafficActor(road, "waiting-leader.ovh", 0.0, 0.0, false);
        auto waitingFollower = makeMapTrafficActor(road, "waiting-follower.ovh", 6.0, 0.0, false);
        waitingLeader.distanceMeters = 24.0;
        for (int frame = 0; frame < 20; ++frame) {
            advanceMapTrafficDriving({&waitingFollower, &waitingLeader}, {}, 0.1);
        }
        requireNear(waitingLeader.distanceMeters, 24.0,
                    "a stationary leader retains its occupied position across substeps");
        waitingLeader.cruiseSpeedMetersPerSecond = 6.0;
        for (int frame = 0; frame < 60; ++frame) {
            advanceMapTrafficDriving({&waitingFollower, &waitingLeader}, {}, 0.1);
            require(mapTrafficHasClearance(footprint(waitingFollower), {footprint(waitingLeader)},
                                           0.49),
                    "a follower keeps clearance when a stationary leader starts moving");
        }
        require(waitingLeader.distanceMeters > 36.0 && waitingFollower.distanceMeters > 5.0,
                "vehicles resume safely when the leader crosses an occupancy cell boundary");
    }
    for (const double startDistance : {11.9, 12.1}) {
        for (const bool reverseOrder : {false, true}) {
            auto followerOnRoute =
                makeMapTrafficActor(road, "projected-follower.ovh", 12.0, 0.0, false);
            auto otherRoute = road;
            otherRoute.segments.front().spline.placement.splineId += 1;
            auto leaderOnOtherRoute =
                makeMapTrafficActor(otherRoute, "projected-leader.ovh", 0.0, 0.0, false);
            followerOnRoute.distanceMeters = startDistance;
            followerOnRoute.speedMetersPerSecond = 8.0;
            leaderOnOtherRoute.distanceMeters = startDistance + 20.0;
            const std::vector<MapTrafficActor*> ordered =
                reverseOrder ? std::vector<MapTrafficActor*>{&leaderOnOtherRoute, &followerOnRoute}
                             : std::vector<MapTrafficActor*>{&followerOnRoute, &leaderOnOtherRoute};
            advanceMapTrafficDriving(ordered, {}, 0.05);
            require(followerOnRoute.blockReason == MapTrafficBlockReason::Leader &&
                        followerOnRoute.blockingActor == &leaderOnOtherRoute,
                    "following finds leaders on different routes across occupancy cell boundaries");
        }
    }
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
    auto pedestrianBehindCar = makeMapTrafficActor(road, "car.ovh", 8.0, 0.0, false);
    pedestrianBehindCar.distanceMeters = 20.0;
    pedestrianBehindCar.speedMetersPerSecond = 8.0;
    const MapTrafficObstacle pedestrianNearRear{{{0.0, 17.6, 0.0}, 0.0, 0}, 0.25, 0.42};
    for (int frame = 0; frame < 30; ++frame) {
        advanceMapTrafficDriving({&pedestrianBehindCar}, {pedestrianNearRear}, 0.1);
    }
    require(pedestrianBehindCar.distanceMeters > 24.0 &&
                pedestrianBehindCar.speedMetersPerSecond > 5.0,
            "a pedestrian behind a car does not force it to stop while it moves away");
    auto pedestrianWalker = makeMapTrafficActor(road, "walker.hum", 1.2, 0.0, false);
    const MapTrafficObstacle parkedVehicle{{{0.0, 5.0, 0.0}, 0.0, 0}, 4.5, 1.8};
    require(!advanceMapTrafficActorLoopingAvoidingObstacles(pedestrianWalker, {parkedVehicle}, 0.5,
                                                            0.8, 3.0) &&
                pedestrianWalker.distanceMeters > 0.0 && pedestrianWalker.distanceMeters < 2.6,
            "pedestrians stop before an occupied vehicle footprint");
    require(advanceMapTrafficActorLoopingAvoidingObstacles(pedestrianWalker, {}, 0.5, 0.8, 1.0) &&
                pedestrianWalker.distanceMeters > 2.4,
            "pedestrians resume along their route after the vehicle clears");
    MapTrafficRouteSegment forwardPedestrianPath = straight(1198, 0.0, 0.0, 0.0, 30.0);
    MapTrafficRouteSegment reversePedestrianPath = forwardPedestrianPath;
    reversePedestrianPath.reversed = true;
    auto pedestrianGoingForward = makeMapTrafficActor(oneSegment(forwardPedestrianPath),
                                                      "forward-walker.hum", 1.2, 0.0, false);
    auto pedestrianGoingBackward = makeMapTrafficActor(oneSegment(reversePedestrianPath),
                                                       "backward-walker.hum", 1.2, 0.0, false);
    pedestrianGoingForward.lengthMeters = pedestrianGoingBackward.lengthMeters = 0.25;
    pedestrianGoingForward.widthMeters = pedestrianGoingBackward.widthMeters = 0.42;
    pedestrianGoingForward.distanceMeters = 10.0;
    pedestrianGoingBackward.distanceMeters = 10.0;
    const auto pedestrianStartPose = sampleMapTrafficRoute(
        pedestrianGoingForward.route, pedestrianGoingForward.distanceMeters, 0.0);
    const auto pedestrianOppositeStartPose = sampleMapTrafficRoute(
        pedestrianGoingBackward.route, pedestrianGoingBackward.distanceMeters, 0.0);
    require(pedestrianStartPose && pedestrianOppositeStartPose &&
                std::hypot(
                    pedestrianStartPose->position[0] - pedestrianOppositeStartPose->position[0],
                    pedestrianStartPose->position[1] - pedestrianOppositeStartPose->position[1]) >
                    5.0,
            "opposing pedestrians start apart on the same path");
    for (int frame = 0; frame < 50; ++frame) {
        require(advanceMapTrafficActorLoopingAvoidingObstacles(pedestrianGoingForward, {}, 0.25,
                                                               0.42, 0.1) &&
                    advanceMapTrafficActorLoopingAvoidingObstacles(pedestrianGoingBackward, {},
                                                                   0.25, 0.42, 0.1),
                "opposing pedestrians are not blocked by one another");
    }
    const auto pedestrianEndPose = sampleMapTrafficRoute(
        pedestrianGoingForward.route, pedestrianGoingForward.distanceMeters, 0.0);
    const auto pedestrianOppositeEndPose = sampleMapTrafficRoute(
        pedestrianGoingBackward.route, pedestrianGoingBackward.distanceMeters, 0.0);
    require(pedestrianEndPose && pedestrianOppositeEndPose &&
                pedestrianGoingForward.distanceMeters > 15.0 &&
                pedestrianGoingBackward.distanceMeters > 15.0 &&
                std::hypot(pedestrianEndPose->position[0] - pedestrianOppositeEndPose->position[0],
                           pedestrianEndPose->position[1] -
                               pedestrianOppositeEndPose->position[1]) < 5.0,
            "opposing pedestrians pass through each other and keep moving");
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
    for (const double offset : {-12.01, -0.01, 11.99}) {
        auto shiftedNorth =
            makeMapTrafficActor(oneSegment(straight(1200, offset, offset - 40.0, 0.0, 100.0)),
                                "north.ovh", 8.0, 0.0, false);
        auto shiftedEast =
            makeMapTrafficActor(oneSegment(straight(1201, offset - 40.0, offset, 90.0, 100.0)),
                                "east.ovh", 8.0, 0.0, false);
        shiftedNorth.lengthMeters = 18.0;
        bool shiftedYieldObserved = false;
        for (int frame = 0; frame < 250; ++frame) {
            advanceMapTrafficDriving({&shiftedNorth, &shiftedEast}, {}, 0.1);
            shiftedYieldObserved = shiftedYieldObserved || shiftedNorth.junctionWaitSeconds > 0.0 ||
                                   shiftedEast.junctionWaitSeconds > 0.0;
            require(
                mapTrafficHasClearance(footprint(shiftedNorth), {footprint(shiftedEast)}, 0.49),
                "cross-cell reservations protect long vehicles across negative cell boundaries");
        }
        require(shiftedYieldObserved && shiftedNorth.distanceMeters > 60.0 &&
                    shiftedEast.distanceMeters > 60.0,
                "cross-cell reservation checks let both crossing vehicles traverse");
    }
    MapTrafficGroupRules committedPathRules;
    committedPathRules.groupIndex = 0;
    committedPathRules.priority = 64.0;
    MapTrafficGroupRules approachingPathRules;
    approachingPathRules.groupIndex = 0;
    approachingPathRules.priority = 192.0;
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
    auto partiallyCommitted = makeMapTrafficActor(junctionClassificationRoute,
                                                  "partially-committed.ovh", 8.0, 0.0, false);
    partiallyCommitted.distanceMeters = 39.0;
    require(
        !mapTrafficRoutePositionOnJunctionPath(partiallyCommitted.route,
                                               partiallyCommitted.distanceMeters) &&
            !mapTrafficRoutePositionOnJunctionPath(partiallyCommitted.route,
                                                   partiallyCommitted.distanceMeters -
                                                       partiallyCommitted.lengthMeters * 0.5) &&
            mapTrafficActorOnJunctionPath(partiallyCommitted, partiallyCommitted.distanceMeters),
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
    committedJunctionActor.trafficRuleGroupIndex = 0;
    highPriorityApproach.trafficRuleGroupIndex = 0;
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
    MapTrafficRoute turningBusRoute;
    MapTrafficRouteSegment turningBusJunction;
    turningBusJunction.points = {{0.0, 0.0, 0.0}, {0.0, 4.0, 0.0}};
    turningBusRoute.segments = {straight(1590, 0.0, -20.0, 0.0, 20.0), turningBusJunction,
                                straight(1591, 0.0, 4.0, 90.0, 30.0)};
    turningBusRoute.cumulativeEndDistances = {20.0, 24.0, 54.0};
    turningBusRoute.totalLength = 54.0;
    auto turningBus = makeMapTrafficActor(turningBusRoute, "turning-bus.ovh", 8.0, 0.0, false);
    turningBus.isBus = true;
    turningBus.distanceMeters = 22.5;
    auto turningCar = makeMapTrafficActor(turningBusRoute, "turning-car.ovh", 8.0, 0.0, false);
    turningCar.distanceMeters = 22.5;
    const MapTrafficObstacle nearbyVehicle{{{3.7, 6.4, 0.0}, std::numbers::pi / 2.0, 0}, 2.5, 2.5};
    for (int frame = 0; frame < 80; ++frame) {
        advanceMapTrafficDriving({&turningBus}, {nearbyVehicle}, 0.1);
        advanceMapTrafficDriving({&turningCar}, {nearbyVehicle}, 0.1);
        require(mapTrafficHasClearance(footprint(turningBus), {nearbyVehicle}, 0.0) &&
                    mapTrafficHasClearance(footprint(turningCar), {nearbyVehicle}, 0.0),
                "vehicle-box leeway never permits actual footprint overlap");
    }
    require(turningBus.distanceMeters > 27.5 && turningCar.distanceMeters > 27.5,
            "committed bus and car turns tolerate small vehicle-box clearance");

    auto longJunctionRoute = throughJunction(1600, 0.0, -40.0, 0.0, 0.0, 1.0);
    longJunctionRoute.segments[1].points = {{0.0, -6.0, 0.0}, {0.0, 194.0, 0.0}};
    longJunctionRoute.segments[2] = straight(1602, 0.0, 194.0, 0.0, 40.0);
    longJunctionRoute.cumulativeEndDistances = {34.0, 234.0, 274.0};
    longJunctionRoute.totalLength = 274.0;
    auto distantQueueApproach =
        makeMapTrafficActor(longJunctionRoute, "distant-junction-queue.ovh", 8.0, 0.0, false);
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
    auto stoppedLongPath =
        makeMapTrafficActor(longJunctionRoute, "stopped-long-path.ovh", 8.0, 0.0, false);
    stoppedLongPath.distanceMeters = 40.0;
    stoppedLongPath.blockReason = MapTrafficBlockReason::Obstacle;
    auto distantLongPathCrossing =
        makeMapTrafficActor(throughJunction(1610, -40.0, 80.0, 90.0, 1.0, 0.0),
                            "distant-crossing.ovh", 8.0, 0.0, false);
    distantLongPathCrossing.distanceMeters = 20.0;
    distantLongPathCrossing.speedMetersPerSecond = 8.0;
    const MapTrafficObstacle holdingLongPath{{{0.0, 5.5, 0.0}, 0.0, 0}, 4.5, 1.8};
    for (int frame = 0; frame < 120; ++frame) {
        advanceMapTrafficDriving({&stoppedLongPath, &distantLongPathCrossing}, {holdingLongPath},
                                 0.1);
    }
    require(distantLongPathCrossing.distanceMeters > 70.0,
            "a stopped vehicle does not reserve a distant crossing on its remaining spline");
    auto longPathPriorityRoute = longJunctionRoute;
    for (auto& segment : longPathPriorityRoute.segments) {
        segment.pathRules.priority = 1.0;
    }
    auto priorityCrossingRoute = throughJunction(1620, -40.0, 80.0, 90.0, 1.0, 0.0);
    for (auto& segment : priorityCrossingRoute.segments) {
        segment.pathRules.priority = 200.0;
    }
    auto lowOnLongPath =
        makeMapTrafficActor(longPathPriorityRoute, "low-on-long-path.ovh", 8.0, 0.0, false);
    auto highAtLocalCrossing =
        makeMapTrafficActor(priorityCrossingRoute, "high-local-crossing.ovh", 8.0, 0.0, false);
    lowOnLongPath.distanceMeters = 100.0;
    highAtLocalCrossing.distanceMeters = 20.0;
    lowOnLongPath.speedMetersPerSecond = 8.0;
    highAtLocalCrossing.speedMetersPerSecond = 8.0;
    bool highCrossedFirst = false;
    for (int frame = 0; frame < 200; ++frame) {
        advanceMapTrafficDriving({&lowOnLongPath, &highAtLocalCrossing}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(lowOnLongPath), {footprint(highAtLocalCrossing)},
                                       0.49),
                "local priority arbitration retains physical clearance on a long junction path");
        highCrossedFirst = highCrossedFirst || (highAtLocalCrossing.distanceMeters > 45.0 &&
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
        path.points = {{0.0, -6.0 + piece * 4.0, 0.0}, {0.0, -2.0 + piece * 4.0, 0.0}};
        splitLongPathRoute.segments.push_back(std::move(path));
        splitLongPathRoute.cumulativeEndDistances.push_back(38.0 + piece * 4.0);
    }
    splitLongPathRoute.segments.push_back(longPathPriorityRoute.segments.back());
    splitLongPathRoute.cumulativeEndDistances.push_back(274.0);
    const auto compareLongPathPartition = [&](const MapTrafficRoute& route) {
        auto low = makeMapTrafficActor(route, "partition-low.ovh", 8.0, 0.0, false);
        auto high =
            makeMapTrafficActor(priorityCrossingRoute, "partition-high.ovh", 8.0, 0.0, false);
        low.distanceMeters = 100.0;
        high.distanceMeters = 20.0;
        low.speedMetersPerSecond = 8.0;
        high.speedMetersPerSecond = 8.0;
        std::vector<std::array<double, 4>> trace;
        for (int frame = 0; frame < 140; ++frame) {
            advanceMapTrafficDriving({&low, &high}, {}, 0.1);
            trace.push_back({low.distanceMeters, high.distanceMeters, low.speedMetersPerSecond,
                             high.speedMetersPerSecond});
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
    auto redPriorityVehicle =
        makeMapTrafficActor(redPriorityRoute, "red-priority.ovh", 8.0, 0.0, false);
    auto permissiveLongPath =
        makeMapTrafficActor(longPathPriorityRoute, "permissive-long-path.ovh", 8.0, 0.0, false);
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
                redPriorityVehicle.distanceMeters < 34.0 - redPriorityVehicle.lengthMeters * 0.5 &&
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
    longMergeRoute.cumulativeEndDistances = {34.0, 34.0 + longMergeLength, 74.0 + longMergeLength};
    longMergeRoute.totalLength = longMergeRoute.cumulativeEndDistances.back();
    auto localMerger = makeMapTrafficActor(longMergeRoute, "local-merger.ovh", 8.0, 0.0, false);
    auto remoteMergedLaneVehicle =
        makeMapTrafficActor(oneSegment(straight(1633, 0.0, 0.0, 90.0, 250.0)),
                            "remote-merged-lane.ovh", 0.0, 0.0, false);
    localMerger.distanceMeters = 20.0;
    localMerger.speedMetersPerSecond = 8.0;
    remoteMergedLaneVehicle.distanceMeters = 120.0;
    for (int frame = 0; frame < 180; ++frame) {
        advanceMapTrafficDriving({&localMerger, &remoteMergedLaneVehicle}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(localMerger), {footprint(remoteMergedLaneVehicle)},
                                       0.49),
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
            require(
                mapTrafficHasClearance(footprint(northCorridor), {footprint(southCorridor)}, 0.49),
                "opposing single-lane corridor traffic retains physical clearance");
            if (northStartsAtRed && frame == 120) {
                require(northCorridor.distanceMeters < 40.0 && southCorridor.distanceMeters > 100.0,
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
        loopingNorthRoute.segments.insert(loopingNorthRoute.segments.end(),
                                          {straight(1663, 0.0, 46.0, 90.0, 40.0),
                                           straight(1664, 40.0, 46.0, 180.0, 86.0),
                                           straight(1665, 40.0, -40.0, 270.0, 40.0)});
        loopingEastRoute.segments.insert(loopingEastRoute.segments.end(),
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
            require(mapTrafficHasClearance(footprint(loopingNorth), {footprint(loopingEast)}, 0.49),
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
    auto committedPerpendicularNorth =
        makeMapTrafficActor(throughJunction(1350, 0.0, -40.0, 0.0, 0.0, 1.0),
                            "committed-perpendicular-north.ovh", 8.0, 0.0, false);
    auto committedPerpendicularEast =
        makeMapTrafficActor(throughJunction(1360, -40.0, 0.0, 90.0, 1.0, 0.0),
                            "committed-perpendicular-east.ovh", 8.0, 0.0, false);
    committedPerpendicularNorth.distanceMeters = 35.0;
    committedPerpendicularEast.distanceMeters = 35.0;
    committedPerpendicularNorth.speedMetersPerSecond = 8.0;
    committedPerpendicularEast.speedMetersPerSecond = 8.0;
    for (int frame = 0; frame < 250; ++frame) {
        advanceMapTrafficDriving({&committedPerpendicularNorth, &committedPerpendicularEast}, {},
                                 0.1);
        require(mapTrafficHasClearance(footprint(committedPerpendicularNorth),
                                       {footprint(committedPerpendicularEast)}, 0.49),
                "committed perpendicular vehicles retain crossing clearance");
    }
    require(committedPerpendicularNorth.distanceMeters > 60.0 &&
                committedPerpendicularEast.distanceMeters > 60.0,
            "one committed perpendicular vehicle clears before the other proceeds");
    const auto junctionRouteFromPose = [&](int id, double x, double y, double headingDegrees) {
        constexpr double approachLength = 20.0;
        constexpr double connectorLength = 20.0;
        constexpr double exitLength = 30.0;
        const double headingRadians = headingDegrees * std::acos(-1.0) / 180.0;
        auto approach =
            straight(id, x - std::sin(headingRadians) * approachLength,
                     y - std::cos(headingRadians) * approachLength, headingDegrees, approachLength);
        auto connector = straight(id + 1, x, y, headingDegrees, connectorLength);
        connector.points = {{x, y, 0.0},
                            {x + std::sin(headingRadians) * connectorLength,
                             y + std::cos(headingRadians) * connectorLength, 0.0}};
        auto exit =
            straight(id + 2, x + std::sin(headingRadians) * connectorLength,
                     y + std::cos(headingRadians) * connectorLength, headingDegrees, exitLength);
        MapTrafficRoute route;
        route.segments = {approach, connector, exit};
        route.cumulativeEndDistances = {approachLength, approachLength + connectorLength,
                                        approachLength + connectorLength + exitLength};
        route.totalLength = route.cumulativeEndDistances.back();
        return route;
    };
    auto collisionBlockedEast = makeMapTrafficActor(junctionRouteFromPose(1362, 0.0, 0.0, 90.0),
                                                    "collision-blocked-east.ovh", 8.0, 0.0, false);
    auto collisionBlockedNorth =
        makeMapTrafficActor(junctionRouteFromPose(1363, 5.0, -5.0, 0.0),
                            "collision-blocked-north.ovh", 8.0, 0.0, false);
    collisionBlockedEast.distanceMeters = 20.2;
    collisionBlockedNorth.distanceMeters = 20.2;
    collisionBlockedEast.speedMetersPerSecond = 8.0;
    collisionBlockedNorth.speedMetersPerSecond = 8.0;
    collisionBlockedEast.junctionWaitSeconds = 18.0;
    collisionBlockedNorth.junctionWaitSeconds = 18.0;
    collisionBlockedEast.blockReason = MapTrafficBlockReason::Collision;
    collisionBlockedNorth.blockReason = MapTrafficBlockReason::Collision;
    collisionBlockedEast.blockingActor = &collisionBlockedNorth;
    collisionBlockedNorth.blockingActor = &collisionBlockedEast;
    require(mapTrafficHasClearance(footprint(collisionBlockedEast),
                                   {footprint(collisionBlockedNorth)}, 0.0),
            "collision recovery begins before overlapping vehicle bodies");
    bool collisionPairCleared = false;
    double previousEastDistance = collisionBlockedEast.distanceMeters;
    double previousNorthDistance = collisionBlockedNorth.distanceMeters;
    for (int frame = 0; frame < 600; ++frame) {
        advanceMapTrafficDriving({&collisionBlockedEast, &collisionBlockedNorth}, {}, 0.1);
        const bool hasClearance = mapTrafficHasClearance(footprint(collisionBlockedEast),
                                                         {footprint(collisionBlockedNorth)}, 0.0);
        require(!collisionPairCleared || hasClearance,
                "collision recovery preserves clearance after the vehicles separate");
        require(collisionBlockedEast.distanceMeters + 1.0e-9 >= previousEastDistance &&
                    collisionBlockedNorth.distanceMeters + 1.0e-9 >= previousNorthDistance,
                "collision-blocked junction vehicles never retreat along their routes");
        collisionPairCleared = collisionPairCleared || hasClearance;
        previousEastDistance = collisionBlockedEast.distanceMeters;
        previousNorthDistance = collisionBlockedNorth.distanceMeters;
    }
    require(collisionPairCleared && collisionBlockedEast.distanceMeters > 40.0 &&
                collisionBlockedNorth.distanceMeters > 40.0,
            "collision-blocked crossing vehicles break the deadlock and both clear");
    const auto parallelJunctionRoute = [&](int splineId, double y) {
        auto path = straight(splineId, 0.0, y, 90.0, 30.0);
        path.points = {{0.0, y, 0.0}, {30.0, y, 0.0}};
        return oneSegment(path);
    };
    auto closeTurningLaneA = makeMapTrafficActor(parallelJunctionRoute(1365, 0.0),
                                                 "close-turning-a.ovh", 8.0, 0.0, false);
    auto closeTurningLaneB = makeMapTrafficActor(parallelJunctionRoute(1366, 2.1),
                                                 "close-turning-b.ovh", 8.0, 0.0, false);
    closeTurningLaneA.distanceMeters = 5.0;
    closeTurningLaneB.distanceMeters = 5.0;
    for (int frame = 0; frame < 50; ++frame) {
        advanceMapTrafficDriving({&closeTurningLaneA, &closeTurningLaneB}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(closeTurningLaneA), {footprint(closeTurningLaneB)},
                                       0.0),
                "relaxed in-junction collision checks never allow actual body overlap");
        require(closeTurningLaneA.blockReason != MapTrafficBlockReason::Collision &&
                    closeTurningLaneB.blockReason != MapTrafficBlockReason::Collision,
                "vehicles with tight but physically clear turning paths are not collision-blocked");
    }
    require(closeTurningLaneA.distanceMeters > 10.0 && closeTurningLaneB.distanceMeters > 10.0,
            "vehicles with physically clear paths continue through the junction");
    const auto makeShallowMergeRoute = [&](int approachId, double headingDegrees) {
        constexpr double approachLength = 18.0;
        const double headingRadians = headingDegrees * std::acos(-1.0) / 180.0;
        MapTrafficRoute route;
        route.segments = {straight(approachId, -std::sin(headingRadians) * approachLength,
                                   -std::cos(headingRadians) * approachLength, headingDegrees,
                                   approachLength),
                          straight(1344, 0.0, 0.0, 90.0, 70.0)};
        route.cumulativeEndDistances = {approachLength, approachLength + 70.0};
        route.totalLength = route.cumulativeEndDistances.back();
        return route;
    };
    auto shallowMergeFirst = makeMapTrafficActor(makeShallowMergeRoute(1342, 72.0),
                                                 "shallow-merge-first.ovh", 8.0, 0.0, false);
    auto shallowMergeSecond = makeMapTrafficActor(makeShallowMergeRoute(1343, 90.0),
                                                  "shallow-merge-second.ovh", 8.0, 0.0, false);
    shallowMergeFirst.speedMetersPerSecond = 8.0;
    shallowMergeSecond.speedMetersPerSecond = 8.0;
    for (int frame = 0; frame < 300; ++frame) {
        advanceMapTrafficDriving({&shallowMergeFirst, &shallowMergeSecond}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(shallowMergeFirst),
                                       {footprint(shallowMergeSecond)}, 0.49),
                "shallow-angle same-direction merges retain clearance without join reservations");
    }
    require(shallowMergeFirst.distanceMeters > 60.0 && shallowMergeSecond.distanceMeters > 60.0,
            "vehicles continue through shared road flow without a spline-boundary deadlock");
    const auto makeSharedExitRoute = [&](int approachId, double headingDegrees) {
        constexpr double approachLength = 30.0;
        constexpr double exitLength = 70.0;
        const double headingRadians = headingDegrees * std::acos(-1.0) / 180.0;
        auto approach =
            straight(approachId, -std::sin(headingRadians) * approachLength,
                     -std::cos(headingRadians) * approachLength, headingDegrees, approachLength);
        auto exit = straight(1390, 0.0, 0.0, 90.0, exitLength);
        MapTrafficRoute route;
        route.segments = {approach, exit};
        route.cumulativeEndDistances = {approachLength, approachLength + exitLength};
        route.totalLength = route.cumulativeEndDistances.back();
        return route;
    };
    auto diagonalMerge =
        makeMapTrafficActor(makeSharedExitRoute(1391, 60.0), "diagonal-merge.ovh", 8.0, 0.0, false);
    auto straightMerge =
        makeMapTrafficActor(makeSharedExitRoute(1392, 90.0), "straight-merge.ovh", 8.0, 0.0, false);
    diagonalMerge.speedMetersPerSecond = 8.0;
    straightMerge.speedMetersPerSecond = 8.0;
    for (int frame = 0; frame < 300; ++frame) {
        advanceMapTrafficDriving({&diagonalMerge, &straightMerge}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(diagonalMerge), {footprint(straightMerge)}, 0.49),
                "moving-block merge arbitration preserves clearance on a shared exit spline");
    }
    require(diagonalMerge.distanceMeters > 60.0 && straightMerge.distanceMeters > 60.0,
            "both streams progress through a shared-exit merge after yielding as needed");
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
    turnaroundCrossingRoute.segments = {turnaroundCrossingApproach, turnaroundCrossingPath,
                                        straight(1324, 10.0, 0.0, 90.0, 40.0)};
    turnaroundCrossingRoute.cumulativeEndDistances = {30.0, 50.0, 90.0};
    turnaroundCrossingRoute.totalLength = 90.0;
    auto turnaroundCrossing = makeMapTrafficActor(std::move(turnaroundCrossingRoute),
                                                  "turnaround-crossing.ovh", 8.0, 0.0, false);
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
    require(
        !turnaroundCrossingWaitedAtEntry && turnaroundCrossing.distanceMeters > 70.0 &&
            slowTurnaround.distanceMeters > 70.0,
        "a moving vehicle's distant future junction path does not block entry for its full route");
    auto stoppedTurnaround =
        makeMapTrafficActor(oneSegment(slowTurnPath), "stopped-turnaround.ovh", 2.0, 0.0, false);
    stoppedTurnaround.distanceMeters = 40.0;
    stoppedTurnaround.speedMetersPerSecond = 0.0;
    stoppedTurnaround.blockReason = MapTrafficBlockReason::Obstacle;
    auto stoppedCrossingRoute = turnaroundCrossing.route;
    auto stoppedCrossing = makeMapTrafficActor(std::move(stoppedCrossingRoute),
                                               "stopped-crossing.ovh", 8.0, 0.0, false);
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
        makeMapTrafficActor(oneSegment(groundCrossingPath), "ground-crossing.ovh", 8.0, 0.0, false);
    auto elevatedCrossing = makeMapTrafficActor(oneSegment(elevatedCrossingPath),
                                                "elevated-crossing.ovh", 8.0, 0.0, false);
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
            followerOverlappedLeaderClaim || (claimLeader.junctionClaimEndMeters.has_value() &&
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
    auto differentExitLeader =
        makeMapTrafficActor(differentExitLeaderRoute, "different-exit-leader.ovh", 8.0, 0.0, false);
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
    require(differentExitFollowerEnteredDuringClaim && slowestDifferentExitFollower > 2.0 &&
                differentExitFollower.distanceMeters > 70.0,
            "a follower with a different exit does not wait for its leader to clear the junction");
    MapTrafficRoute segmentedLeaderRoute;
    segmentedLeaderRoute.segments.push_back(straight(1350, 0.0, -40.0, 0.0, 30.0));
    segmentedLeaderRoute.cumulativeEndDistances.push_back(30.0);
    for (int segmentIndex = 0; segmentIndex < 6; ++segmentIndex) {
        const double segmentStartY = -10.0 + segmentIndex * 1.5;
        auto segment = straight(1351 + segmentIndex, 0.0, segmentStartY, 0.0, 1.5);
        segment.points = {{0.0, segmentStartY, 0.0}, {0.0, segmentStartY + 1.5, 0.0}};
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
    segmentedFollowerRoute.segments = {segmentedFollowerRoute.segments.front(),
                                       segmentedFollowerTurn,
                                       straight(1359, 10.0, 0.0, 90.0, 40.0)};
    segmentedFollowerRoute.cumulativeEndDistances = {30.0, 30.0 + std::sqrt(200.0),
                                                     70.0 + std::sqrt(200.0)};
    segmentedFollowerRoute.totalLength = segmentedFollowerRoute.cumulativeEndDistances.back();
    auto segmentedLeader =
        makeMapTrafficActor(segmentedLeaderRoute, "segmented-leader.ovh", 8.0, 0.0, false);
    auto segmentedFollower =
        makeMapTrafficActor(segmentedFollowerRoute, "segmented-follower.ovh", 8.0, 0.0, false);
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
            segmentedLeaderKeptStaleClaim || (segmentedLeader.distanceMeters > 43.0 &&
                                              segmentedLeader.junctionClaimEndMeters.has_value());
        require(mapTrafficHasClearance(footprint(segmentedLeader), {footprint(segmentedFollower)},
                                       0.49),
                "a follower with many junction connectors keeps clearance from its leader");
    }
    require(!segmentedLeaderKeptStaleClaim && slowestSegmentedFollowerSpeed > 2.0 &&
                segmentedFollower.distanceMeters > 60.0 && segmentedFollowerJunctionWaitFrames < 5,
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
    mainRoadRules.priority = 192.0;
    MapTrafficGroupRules sideRoadRules;
    sideRoadRules.groupIndex = 0;
    sideRoadRules.priority = 64.0;
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
        sideTurn.pathRules.groups.push_back(mainRoadRules);
        sideRoute.segments = {sideApproach, sideTurn, straight(1365, 6.0, 0.0, 90.0, 60.0)};
        sideRoute.cumulativeEndDistances = {34.0, 34.0 + std::sqrt(72.0), 94.0 + std::sqrt(72.0)};
        sideRoute.totalLength = 94.0 + std::sqrt(72.0);
        auto mainRoadCar =
            makeMapTrafficActor(std::move(mainRoute), "main-road.ovh", 10.0, 0.0, false);
        auto sideRoadCar =
            makeMapTrafficActor(std::move(sideRoute), "side-road.ovh", 8.0, 0.0, false);
        mainRoadCar.unscheduledGroupIndex = 0;
        sideRoadCar.unscheduledGroupIndex = 0;
        mainRoadCar.trafficRuleGroupIndex = 0;
        sideRoadCar.trafficRuleGroupIndex = 0;
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
            sideRoadYieldedToMainRoad =
                sideRoadYieldedToMainRoad ||
                (sideRoadCar.blockingActor == &mainRoadCar && sideRoadCar.distanceMeters < 34.0);
            require(mapTrafficHasClearance(footprint(mainRoadCar), {footprint(sideRoadCar)}, 0.49),
                    "priority arbitration keeps main and side road vehicles separated");
        }
        require(slowestMainRoad > 9.0,
                "a vehicle on a higher-priority spline does not give way to a lower-priority one");
        require(sideRoadYieldedToMainRoad,
                "a vehicle on a lower-priority spline gives way to approaching priority traffic");
        require(mainRoadCar.distanceMeters > 100.0 && sideRoadCar.distanceMeters > 60.0,
                "the lower-priority vehicle enters after the priority vehicle has passed");
    }
    auto straightTLeader = makeMapTrafficActor(throughJunction(1370, 0.0, -40.0, 0.0, 0.0, 1.0),
                                               "straight-t-leader.ovh", 8.0, 0.0, false);
    auto straightTFollower = makeMapTrafficActor(throughJunction(1370, 0.0, -40.0, 0.0, 0.0, 1.0),
                                                 "straight-t-follower.ovh", 8.0, 0.0, false);
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
    require(straightTLeader.distanceMeters > 70.0 && straightTFollower.distanceMeters > 55.0 &&
                straightTLeader.junctionWaitSeconds == 0.0 &&
                straightTFollower.junctionWaitSeconds == 0.0,
            "straight-through vehicles don't wait when nobody is turning across their path");
    auto acceleratingTLeader = makeMapTrafficActor(throughJunction(1372, 0.0, -40.0, 0.0, 0.0, 1.0),
                                                   "accelerating-t-leader.ovh", 8.0, 0.0, false);
    auto acceleratingTFollower =
        makeMapTrafficActor(throughJunction(1372, 0.0, -40.0, 0.0, 0.0, 1.0),
                            "accelerating-t-follower.ovh", 8.0, 0.0, false);
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
    require(slowestAcceleratingTFollower > 2.0 && acceleratingTFollower.distanceMeters > 55.0,
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
    queuedSideTurn.cumulativeEndDistances = {34.0, 34.0 + std::sqrt(72.0), 94.0 + std::sqrt(72.0)};
    queuedSideTurn.totalLength = queuedSideTurn.cumulativeEndDistances.back();
    auto queuedMainLeader =
        makeMapTrafficActor(queuedMainRoad, "queued-main-leader.ovh", 10.0, 0.0, false);
    auto queuedMainFollower =
        makeMapTrafficActor(queuedMainRoad, "queued-main-follower.ovh", 10.0, 0.0, false);
    auto queuedTTurn = makeMapTrafficActor(queuedSideTurn, "queued-t-turn.ovh", 8.0, 0.0, false);
    queuedMainLeader.unscheduledGroupIndex = 0;
    queuedMainFollower.unscheduledGroupIndex = 0;
    queuedTTurn.unscheduledGroupIndex = 0;
    queuedMainLeader.trafficRuleGroupIndex = 0;
    queuedMainFollower.trafficRuleGroupIndex = 0;
    queuedTTurn.trafficRuleGroupIndex = 0;
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
        sideTurnYielded = sideTurnYielded || ((queuedTTurn.blockingActor == &queuedMainLeader ||
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
    MapTrafficRoute highPriorityTurnApproach = oneSegment(straight(1223, -40.0, 0.0, 90.0, 100.0));
    highPriorityTurnApproach.segments.front().pathRules.groups.push_back(approachingPathRules);
    auto lowTurning =
        makeMapTrafficActor(std::move(lowTurningRoute), "low-turn.ovh", 8.0, 0.0, false);
    auto highPriorityTurn =
        makeMapTrafficActor(std::move(highPriorityTurnApproach), "high-turn.ovh", 8.0, 0.0, false);
    lowTurning.unscheduledGroupIndex = 0;
    highPriorityTurn.unscheduledGroupIndex = 0;
    lowTurning.trafficRuleGroupIndex = 0;
    highPriorityTurn.trafficRuleGroupIndex = 0;
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
    MapTrafficRoute globalPriorityTurnRoute;
    auto globalPriorityTurnApproach = straight(1224, 0.0, -40.0, 0.0, 40.0);
    globalPriorityTurnApproach.pathRules.priority = 64.0;
    const double diagonalConnectorLength = 15.0 * std::sqrt(2.0);
    auto globalPriorityTurnConnector =
        straight(1225, 0.0, 0.0, 0.0, 15.0 + diagonalConnectorLength);
    globalPriorityTurnConnector.points = {{0.0, 0.0, 0.0}, {0.0, 15.0, 0.0}, {15.0, 30.0, 0.0}};
    globalPriorityTurnRoute.segments = {globalPriorityTurnApproach, globalPriorityTurnConnector,
                                        straight(1226, 15.0, 30.0, 90.0, 50.0)};
    globalPriorityTurnRoute.cumulativeEndDistances = {40.0, 40.0 + 15.0 + diagonalConnectorLength,
                                                      90.0 + 15.0 + diagonalConnectorLength};
    globalPriorityTurnRoute.totalLength = globalPriorityTurnRoute.cumulativeEndDistances.back();
    auto defaultPriorityCrossingRoute = oneSegment(straight(1227, -40.0, 15.0, 90.0, 100.0));
    auto globalPriorityTurn =
        makeMapTrafficActor(globalPriorityTurnRoute, "global-priority-turn.ovh", 4.0, 0.0, false);
    auto defaultPriorityCrossing = makeMapTrafficActor(
        defaultPriorityCrossingRoute, "default-priority-crossing.ovh", 8.0, 0.0, false);
    globalPriorityTurn.distanceMeters = 45.0;
    defaultPriorityCrossing.distanceMeters = 30.0;
    globalPriorityTurn.speedMetersPerSecond = 4.0;
    defaultPriorityCrossing.speedMetersPerSecond = 8.0;
    for (int frame = 0; frame < 30; ++frame) {
        advanceMapTrafficDriving({&globalPriorityTurn, &defaultPriorityCrossing}, {}, 0.1);
    }
    require(globalPriorityTurn.junctionWaitSeconds > 0.0 &&
                defaultPriorityCrossing.distanceMeters > 30.0 &&
                globalPriorityTurn.distanceMeters < 55.0,
            "a turning vehicle keeps its incoming road priority while on a long junction path");
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
    lowPriorityRoute.segments.front().pathRules =
        mapTrafficRulesForPath({{false, {"0", "priority", "64", "0"}}}, 0);
    highPriorityRoute.segments.front().pathRules =
        mapTrafficRulesForPath({{false, {"0", "priority", "256", "0"}}}, 0);
    auto lowPriority = makeMapTrafficActor(std::move(lowPriorityRoute), "low.ovh", 8.0, 0.0, false);
    auto highPriority =
        makeMapTrafficActor(std::move(highPriorityRoute), "high.ovh", 8.0, 0.0, false);
    lowPriority.unscheduledGroupIndex = 0;
    highPriority.unscheduledGroupIndex = 0;
    lowPriority.trafficRuleGroupIndex = 0;
    highPriority.trafficRuleGroupIndex = 0;
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
            "priority 256 wins over priority 64 before wait aging closes the priority gap");
    auto blockedPriorityRoute = oneSegment(straight(1260, -40.0, 0.0, 90.0, 100.0));
    auto blockingSegment = straight(1261, -10.0, -10.0, 0.0, 20.0);
    blockingSegment.points = {{-10.0, -10.0, 0.0}, {-10.0, 10.0, 0.0}};
    auto blockingRoute = oneSegment(blockingSegment);
    auto laterCrossingRoute = oneSegment(straight(1262, -4.0, -60.0, 0.0, 100.0));
    blockedPriorityRoute.segments.front().pathRules.priority = 128.0;
    blockingRoute.segments.front().pathRules.priority = 256.0;
    laterCrossingRoute.segments.front().pathRules.priority = 64.0;
    auto blockedPriority =
        makeMapTrafficActor(blockedPriorityRoute, "blocked-priority.ovh", 8.0, 0.0, false);
    auto intersectionBlocker =
        makeMapTrafficActor(blockingRoute, "intersection-blocker.ovh", 0.0, 0.0, false);
    auto laterCrossing =
        makeMapTrafficActor(laterCrossingRoute, "later-crossing.ovh", 8.0, 0.0, false);
    blockedPriority.distanceMeters = 20.0;
    blockedPriority.speedMetersPerSecond = 8.0;
    intersectionBlocker.distanceMeters = 10.0;
    laterCrossing.distanceMeters = 46.0;
    laterCrossing.speedMetersPerSecond = 8.0;
    bool blockedPriorityYielded = false;
    bool laterCrossingPassedBlockedVehicle = false;
    bool downstreamYieldedWhileFrontWasStopped = false;
    for (int frame = 0; frame < 160; ++frame) {
        advanceMapTrafficDriving({&blockedPriority, &intersectionBlocker, &laterCrossing}, {}, 0.1);
        blockedPriorityYielded =
            blockedPriorityYielded ||
            (blockedPriority.blockReason == MapTrafficBlockReason::JunctionTraffic &&
             blockedPriority.speedMetersPerSecond < 0.3);
        laterCrossingPassedBlockedVehicle =
            laterCrossingPassedBlockedVehicle ||
            (laterCrossing.distanceMeters > 60.0 && blockedPriority.distanceMeters < 50.0);
        downstreamYieldedWhileFrontWasStopped =
            downstreamYieldedWhileFrontWasStopped ||
            (blockedPriority.speedMetersPerSecond < 0.3 &&
             blockedPriority.blockReason == MapTrafficBlockReason::JunctionTraffic &&
             laterCrossing.blockingActor == &blockedPriority &&
             laterCrossing.blockReason == MapTrafficBlockReason::JunctionTraffic);
    }
    require(blockedPriorityYielded, "the middle-priority vehicle stops for an occupied junction");
    require(!downstreamYieldedWhileFrontWasStopped,
            "a stopped, uncommitted junction yielder does not block a downstream crossing");
    require(laterCrossingPassedBlockedVehicle,
            "a vehicle waiting to yield does not make other traffic yield to it downstream");
    MapTrafficRoute lowPriorityTurningRoute;
    auto lowPriorityApproach = straight(1208, 0.0, -40.0, 0.0, 40.0);
    lowPriorityApproach.pathRules.groups.push_back(committedPathRules);
    auto lowPriorityConnector = straight(1217, 0.0, 0.0, 0.0, 1.0);
    lowPriorityConnector.points = {
        {0.0, 0.0, 0.0}, {0.0, 4.0, 0.0}, {4.0, 8.0, 0.0}, {10.0, 8.0, 0.0}};
    lowPriorityConnector.pathRules.groups.push_back(committedPathRules);
    const double lowPriorityConnectorLength = 4.0 + std::hypot(4.0, 4.0) + 6.0;
    lowPriorityTurningRoute.segments = {lowPriorityApproach, lowPriorityConnector,
                                        straight(1218, 10.0, 8.0, 90.0, 60.0)};
    lowPriorityTurningRoute.cumulativeEndDistances = {40.0, 40.0 + lowPriorityConnectorLength,
                                                      100.0 + lowPriorityConnectorLength};
    lowPriorityTurningRoute.totalLength = lowPriorityTurningRoute.cumulativeEndDistances.back();
    MapTrafficRoute highPriorityJunctionRoute;
    auto highJunctionApproach = straight(1209, -40.0, 0.0, 90.0, 20.0);
    highJunctionApproach.pathRules.groups.push_back(approachingPathRules);
    auto highPriorityThrough = straight(1219, -20.0, 0.0, 90.0, 40.0);
    highPriorityThrough.points = {{-20.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
    highPriorityThrough.pathRules.groups.push_back(approachingPathRules);
    highPriorityJunctionRoute.segments = {highJunctionApproach, highPriorityThrough,
                                          straight(1220, 20.0, 0.0, 90.0, 60.0)};
    highPriorityJunctionRoute.cumulativeEndDistances = {20.0, 60.0, 120.0};
    highPriorityJunctionRoute.totalLength = 120.0;
    auto lowPriorityTurning = makeMapTrafficActor(std::move(lowPriorityTurningRoute),
                                                  "low-priority-turn.ovh", 8.0, 0.0, false);
    auto highPriorityInJunction = makeMapTrafficActor(
        std::move(highPriorityJunctionRoute), "high-priority-junction.ovh", 8.0, 0.0, false);
    lowPriorityTurning.unscheduledGroupIndex = 0;
    highPriorityInJunction.unscheduledGroupIndex = 0;
    lowPriorityTurning.trafficRuleGroupIndex = 0;
    highPriorityInJunction.trafficRuleGroupIndex = 0;
    lowPriorityTurning.distanceMeters = 24.0;
    highPriorityInJunction.distanceMeters = 30.0;
    lowPriorityTurning.speedMetersPerSecond = 8.0;
    highPriorityInJunction.speedMetersPerSecond = 8.0;
    lowPriorityTurning.junctionWaitSeconds = 30.0;
    require(!mapTrafficRoutePositionOnJunctionPath(lowPriorityTurning.route,
                                                   lowPriorityTurning.distanceMeters) &&
                !mapTrafficRoutePositionOnJunctionPath(lowPriorityTurning.route,
                                                       lowPriorityTurning.distanceMeters +
                                                           lowPriorityTurning.lengthMeters * 0.5) &&
                mapTrafficRoutePositionOnJunctionPath(highPriorityInJunction.route,
                                                      highPriorityInJunction.distanceMeters),
            "a lower-priority turn approaches while the priority vehicle is already established "
            "in the junction");
    bool lowerEnteredBeforePriorityCleared = false;
    for (int frame = 0; frame < 80; ++frame) {
        advanceMapTrafficDriving({&highPriorityInJunction, &lowPriorityTurning}, {}, 0.1);
        lowerEnteredBeforePriorityCleared =
            lowerEnteredBeforePriorityCleared || (lowPriorityTurning.distanceMeters >= 40.0 &&
                                                  highPriorityInJunction.distanceMeters <= 60.0);
        require(mapTrafficHasClearance(footprint(lowPriorityTurning),
                                       {footprint(highPriorityInJunction)}, 0.49),
                "a priority vehicle in the junction clears physically separated lower-priority "
                "turning traffic");
    }
    require(highPriorityInJunction.distanceMeters > 50.0 && !lowerEnteredBeforePriorityCleared,
            "a lower-priority turn does not commit ahead of higher-priority traffic already in "
            "the junction");
    auto yieldingLowerTurn = makeMapTrafficActor(
        lowPriorityTurning.route, "yielding-lower-turn.ovh", 8.0, 0.0, false);
    auto yieldingHigherThrough = makeMapTrafficActor(
        highPriorityInJunction.route, "yielding-higher-through.ovh", 8.0, 0.0, false);
    yieldingLowerTurn.unscheduledGroupIndex = 0;
    yieldingHigherThrough.unscheduledGroupIndex = 0;
    yieldingLowerTurn.trafficRuleGroupIndex = 0;
    yieldingHigherThrough.trafficRuleGroupIndex = 0;
    yieldingLowerTurn.distanceMeters = 42.0;
    yieldingHigherThrough.distanceMeters = 30.0;
    yieldingLowerTurn.speedMetersPerSecond = 0.0;
    yieldingHigherThrough.speedMetersPerSecond = 0.0;
    yieldingLowerTurn.blockReason = MapTrafficBlockReason::JunctionTraffic;
    yieldingHigherThrough.blockReason = MapTrafficBlockReason::JunctionTraffic;
    require(mapTrafficActorOnJunctionPath(yieldingLowerTurn, yieldingLowerTurn.distanceMeters) &&
                mapTrafficActorOnJunctionPath(yieldingHigherThrough,
                                              yieldingHigherThrough.distanceMeters),
            "both yielders are on their junction paths");
    const double yieldingLowerStart = yieldingLowerTurn.distanceMeters;
    const double yieldingHigherStart = yieldingHigherThrough.distanceMeters;
    for (int frame = 0; frame < 30; ++frame) {
        advanceMapTrafficDriving({&yieldingLowerTurn, &yieldingHigherThrough}, {}, 0.1);
    }
    const double lowerYieldingProgress = yieldingLowerTurn.distanceMeters - yieldingLowerStart;
    const double higherPriorityProgress =
        yieldingHigherThrough.distanceMeters - yieldingHigherStart;
    require(higherPriorityProgress > lowerYieldingProgress + 1.0,
            "higher-priority traffic continues when both conflicting vehicles wait on junction "
            "paths");
    for (int frame = 0; frame < 300; ++frame) {
        advanceMapTrafficDriving({&highPriorityInJunction, &lowPriorityTurning}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(lowPriorityTurning),
                                       {footprint(highPriorityInJunction)}, 0.49),
                "lower-priority traffic preserves clearance after the priority vehicle passes");
    }
    require(lowPriorityTurning.distanceMeters > 60.0 &&
                highPriorityInJunction.distanceMeters > 60.0,
            "a lower-priority turn enters after higher-priority junction traffic clears");
    auto committedLowerTurn =
        makeMapTrafficActor(lowPriorityTurning.route, "committed-lower-turn.ovh", 8.0, 0.0, false);
    auto establishedMainTraffic = makeMapTrafficActor(
        highPriorityInJunction.route, "established-main-traffic.ovh", 8.0, 0.0, false);
    committedLowerTurn.unscheduledGroupIndex = 0;
    establishedMainTraffic.unscheduledGroupIndex = 0;
    committedLowerTurn.trafficRuleGroupIndex = 0;
    establishedMainTraffic.trafficRuleGroupIndex = 0;
    committedLowerTurn.distanceMeters = 39.0;
    establishedMainTraffic.distanceMeters = 55.0;
    committedLowerTurn.speedMetersPerSecond = 0.0;
    establishedMainTraffic.speedMetersPerSecond = 8.0;
    require(mapTrafficActorOnJunctionPath(committedLowerTurn, committedLowerTurn.distanceMeters) &&
                !mapTrafficRoutePositionOnJunctionPath(committedLowerTurn.route,
                                                       committedLowerTurn.distanceMeters),
            "the turning vehicle is committed by its front before its center enters");
    const double committedTurnStart = committedLowerTurn.distanceMeters;
    for (int frame = 0; frame < 200; ++frame) {
        advanceMapTrafficDriving({&establishedMainTraffic, &committedLowerTurn}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(committedLowerTurn),
                                       {footprint(establishedMainTraffic)}, 0.49),
                "an already-committed turn preserves physical clearance from main-road traffic");
    }
    require(committedLowerTurn.distanceMeters > committedTurnStart + 10.0 &&
                establishedMainTraffic.distanceMeters > 60.0,
            "an already-committed lower-priority turn clears instead of stopping mid-junction");
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
    auto queuedMergeVehicle =
        makeMapTrafficActor(rightTurnRoute, "queued-merge.ovh", 8.0, 0.0, false);
    queuedMergeVehicle.speedMetersPerSecond = 8.0;
    const MapTrafficObstacle queuedMergeExit{
        {{14.0, 0.0, 0.0}, std::acos(-1.0) * 0.5, 0}, 4.5, 1.8};
    for (int frame = 0; frame < 200; ++frame) {
        advanceMapTrafficDriving({&queuedMergeVehicle}, {queuedMergeExit}, 0.1);
    }
    require(
        queuedMergeVehicle.distanceMeters < 30.0 - queuedMergeVehicle.lengthMeters * 0.5 &&
            !mapTrafficActorOnJunctionPath(queuedMergeVehicle, queuedMergeVehicle.distanceMeters),
        "a vehicle waits before a T-junction when queued traffic blocks its merge exit");
    require(queuedMergeVehicle.distanceMeters > 30.0 - queuedMergeVehicle.lengthMeters * 0.5 - 1.0,
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
    require(unobstructedTurn.distanceMeters > 60.0 && minimumUnobstructedTurnSpeed > 1.5 &&
                unobstructedTurn.junctionWaitSeconds == 0.0,
            "an unblocked turning request crosses the spline boundary without stopping");
    auto closePassingTurn =
        makeMapTrafficActor(rightTurnRoute, "close-passing-turn.ovh", 8.0, 0.0, false);
    closePassingTurn.speedMetersPerSecond = 8.0;
    const auto nearbyTurnPose =
        sampleMapTrafficRoute(rightTurnRoute, 30.0 + connectorLength * 0.5, 0.0);
    require(nearbyTurnPose.has_value(), "sample a turning vehicle's path for clearance test");
    MapTrafficPose nearbyBusPose = *nearbyTurnPose;
    nearbyBusPose.position[0] += std::cos(nearbyBusPose.headingRadians) * 3.5;
    nearbyBusPose.position[1] -= std::sin(nearbyBusPose.headingRadians) * 3.5;
    const MapTrafficObstacle nearbyBus{nearbyBusPose, 4.5, 1.8};
    double minimumClosePassingTurnSpeed = std::numeric_limits<double>::infinity();
    for (int frame = 0; frame < 160; ++frame) {
        advanceMapTrafficDriving({&closePassingTurn}, {nearbyBus}, 0.1);
        require(mapTrafficHasClearance(footprint(closePassingTurn), {nearbyBus}, 0.49),
                "a turn maintains physical clearance from a nearby but non-conflicting vehicle");
        if (closePassingTurn.distanceMeters >= 27.0 &&
            closePassingTurn.distanceMeters <= 30.0 + connectorLength) {
            minimumClosePassingTurnSpeed =
                std::min(minimumClosePassingTurnSpeed, closePassingTurn.speedMetersPerSecond);
        }
    }
    require(closePassingTurn.distanceMeters > 60.0 &&
                minimumClosePassingTurnSpeed >= minimumUnobstructedTurnSpeed - 0.1,
            "a nearby vehicle with physical clearance does not cause extra turn slowdown");
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
    auto differentlySegmentedCurveLeader = makeMapTrafficActor(
        makeSplitRoadCurve(1510), "segmented-curve-leader.ovh", 6.0, 0.0, false);
    auto differentlySegmentedCurveFollower = makeMapTrafficActor(
        makeSplitRoadCurve(1512), "segmented-curve-follower.ovh", 6.0, 0.0, false);
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
        curveWasTreatedAsCrossing = curveWasTreatedAsCrossing ||
                                    differentlySegmentedCurveLeader.blockReason ==
                                        MapTrafficBlockReason::CrossingReservation ||
                                    differentlySegmentedCurveFollower.blockReason ==
                                        MapTrafficBlockReason::CrossingReservation;
    }
    require(!curveWasTreatedAsCrossing && differentlySegmentedCurveLeader.distanceMeters > 60.0 &&
                differentlySegmentedCurveFollower.distanceMeters > 50.0,
            "different spline boundaries do not create crossing blocks on a shared road");
    {
        const auto minimumTurnSpeed = [&](bool withStraightAheadCar) {
            auto turningCar =
                makeMapTrafficActor(rightTurnRoute, "turning-car.ovh", 10.0, 0.0, false);
            auto straightAheadCar =
                makeMapTrafficActor(oneSegment(straight(1230, 0.0, 10.0, 0.0, 50.0)),
                                    "straight-ahead.ovh", 0.0, 0.0, false);
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
    waitingRightTurn.trafficRuleGroupIndex = 0;
    highPriorityCrossing.trafficRuleGroupIndex = 0;
    waitingRightTurn.distanceMeters = 25.0;
    highPriorityCrossing.distanceMeters = 25.0;
    advanceMapTrafficDriving({&waitingRightTurn, &highPriorityCrossing}, {}, 0.1);
    require(waitingRightTurn.junctionWaitSeconds > 0.0,
            "a right-turning approach accrues wait time when lookahead sees crossing traffic");
    MapTrafficRoute priorityTurnExitRoute = oneSegment(straight(1250, -40.0, -2.0, 90.0, 100.0));
    priorityTurnExitRoute.segments.front().pathRules.groups.push_back(approachingPathRules);
    auto yieldingRightTurn =
        makeMapTrafficActor(rightTurnRoute, "yielding-right-turn.ovh", 8.0, 0.0, false);
    auto approachingPriorityExit = makeMapTrafficActor(
        priorityTurnExitRoute, "approaching-priority-exit.ovh", 8.0, 0.0, false);
    yieldingRightTurn.unscheduledGroupIndex = 0;
    approachingPriorityExit.unscheduledGroupIndex = 0;
    yieldingRightTurn.trafficRuleGroupIndex = 0;
    approachingPriorityExit.trafficRuleGroupIndex = 0;
    yieldingRightTurn.distanceMeters = 15.0;
    approachingPriorityExit.distanceMeters = 20.0;
    yieldingRightTurn.speedMetersPerSecond = 8.0;
    approachingPriorityExit.speedMetersPerSecond = 8.0;
    double yieldingTurnMinimumSpeed = std::numeric_limits<double>::infinity();
    bool yieldingTurnWaitedBeforeEntry = false;
    for (int frame = 0; frame < 120; ++frame) {
        advanceMapTrafficDriving({&yieldingRightTurn, &approachingPriorityExit}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(yieldingRightTurn),
                                       {footprint(approachingPriorityExit)}, 0.49),
                "a lower-priority turn and an approaching exit vehicle remain separated");
        if (yieldingRightTurn.distanceMeters >= 30.0 &&
            yieldingRightTurn.distanceMeters <= 30.0 + connectorLength) {
            yieldingTurnMinimumSpeed =
                std::min(yieldingTurnMinimumSpeed, yieldingRightTurn.speedMetersPerSecond);
        }
        yieldingTurnWaitedBeforeEntry =
            yieldingTurnWaitedBeforeEntry || (yieldingRightTurn.junctionWaitSeconds > 0.0 &&
                                              yieldingRightTurn.distanceMeters < 30.0);
    }
    require(yieldingRightTurn.distanceMeters > 60.0 &&
                approachingPriorityExit.distanceMeters > 60.0,
            "a lower-priority turn and approaching exit vehicle eventually clear");
    require(yieldingTurnWaitedBeforeEntry,
            "a lower-priority turn waits before entering the junction");
    require(yieldingTurnMinimumSpeed > 1.5,
            "a lower-priority turn does not stop mid-turn after yielding");
    auto shortConnectorRightTurn =
        makeMapTrafficActor(rightTurnRoute, "short-connector-right-turn.ovh", 8.0, 0.0, false);
    auto nearPriorityCrossing = makeMapTrafficActor(highPriorityCrossingRoute,
                                                    "near-priority-crossing.ovh", 8.0, 0.0, false);
    shortConnectorRightTurn.unscheduledGroupIndex = 0;
    nearPriorityCrossing.unscheduledGroupIndex = 0;
    shortConnectorRightTurn.trafficRuleGroupIndex = 0;
    nearPriorityCrossing.trafficRuleGroupIndex = 0;
    shortConnectorRightTurn.distanceMeters = 27.0;
    nearPriorityCrossing.distanceMeters = 32.0;
    shortConnectorRightTurn.speedMetersPerSecond = 6.0;
    bool shortTurnYieldedBeforeEnteringConflict = false;
    for (int frame = 0; frame < 300; ++frame) {
        advanceMapTrafficDriving({&shortConnectorRightTurn, &nearPriorityCrossing}, {}, 0.1);
        shortTurnYieldedBeforeEnteringConflict =
            shortTurnYieldedBeforeEnteringConflict ||
            ((shortConnectorRightTurn.blockReason == MapTrafficBlockReason::JunctionTraffic ||
              shortConnectorRightTurn.blockReason == MapTrafficBlockReason::CrossingReservation) &&
             shortConnectorRightTurn.blockingActor == &nearPriorityCrossing &&
             shortConnectorRightTurn.junctionWaitSeconds > 0.0 &&
             shortConnectorRightTurn.distanceMeters < 35.0);
        require(mapTrafficHasClearance(footprint(shortConnectorRightTurn),
                                       {footprint(nearPriorityCrossing)}, 0.49),
                "a lower-priority turn remains clear of nearby priority traffic");
    }
    require(shortTurnYieldedBeforeEnteringConflict,
            "a lower-priority short turn yields before entering the conflict");
    require(shortConnectorRightTurn.distanceMeters > 50.0,
            "the lower-priority turn eventually clears after yielding");
    MapTrafficRoute secondRightTurnRoute;
    auto secondRightTurnApproach = straight(1240, -40.0, 0.0, 90.0, 30.0);
    auto secondRightTurnConnector = straight(1241, -10.0, 0.0, 135.0, std::sqrt(200.0));
    for (int index = 0; index <= rightTurnSteps; ++index) {
        const double angle = quarterTurn * (1.0 - static_cast<double>(index) / rightTurnSteps);
        secondRightTurnConnector.points.push_back(
            {-10.0 + 10.0 * std::cos(angle), -10.0 + 10.0 * std::sin(angle), 0.0});
    }
    const double secondConnectorLength = [&] {
        double length = 0.0;
        for (std::size_t index = 1; index < secondRightTurnConnector.points.size(); ++index) {
            const auto& from = secondRightTurnConnector.points[index - 1];
            const auto& to = secondRightTurnConnector.points[index];
            length += std::hypot(to[0] - from[0], to[1] - from[1]);
        }
        return length;
    }();
    secondRightTurnRoute.segments = {secondRightTurnApproach, secondRightTurnConnector,
                                     straight(1242, 0.0, -10.0, 180.0, 60.0)};
    secondRightTurnRoute.cumulativeEndDistances = {30.0, 30.0 + secondConnectorLength,
                                                   90.0 + secondConnectorLength};
    secondRightTurnRoute.totalLength = secondRightTurnRoute.cumulativeEndDistances.back();
    auto simultaneousFirstTurn =
        makeMapTrafficActor(rightTurnRoute, "simultaneous-first-turn.ovh", 8.0, 0.0, false);
    auto simultaneousSecondTurn =
        makeMapTrafficActor(secondRightTurnRoute, "simultaneous-second-turn.ovh", 8.0, 0.0, false);
    simultaneousFirstTurn.distanceMeters = 15.0;
    simultaneousSecondTurn.distanceMeters = 15.0;
    simultaneousFirstTurn.speedMetersPerSecond = 8.0;
    simultaneousSecondTurn.speedMetersPerSecond = 8.0;
    for (int frame = 0; frame < 500; ++frame) {
        advanceMapTrafficDriving({&simultaneousFirstTurn, &simultaneousSecondTurn}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(simultaneousFirstTurn),
                                       {footprint(simultaneousSecondTurn)}, 0.49),
                "simultaneous turning vehicles retain physical clearance");
    }
    require(simultaneousFirstTurn.distanceMeters > 70.0 &&
                simultaneousSecondTurn.distanceMeters > 70.0,
            "simultaneous right turns clear the junction without deadlocking");
    MapTrafficGroupRules mergePriorityRules;
    mergePriorityRules.groupIndex = 0;
    mergePriorityRules.priority = 192.0;
    MapTrafficGroupRules mergeYieldRules;
    mergeYieldRules.groupIndex = 0;
    mergeYieldRules.priority = 64.0;
    MapTrafficRoute priorityMergeRoute;
    auto priorityMergeApproach = straight(1244, -40.0, 0.0, 90.0, 34.0);
    priorityMergeApproach.pathRules.groups.push_back(mergePriorityRules);
    auto priorityMergeConnector = straight(1245, -6.0, 0.0, 45.0, std::sqrt(72.0));
    priorityMergeConnector.points = {{-6.0, 0.0, 0.0}, {0.0, 6.0, 0.0}};
    priorityMergeRoute.segments = {priorityMergeApproach, priorityMergeConnector,
                                   straight(1246, 0.0, 6.0, 0.0, 60.0)};
    priorityMergeRoute.cumulativeEndDistances = {34.0, 34.0 + std::sqrt(72.0),
                                                 94.0 + std::sqrt(72.0)};
    priorityMergeRoute.totalLength = priorityMergeRoute.cumulativeEndDistances.back();
    MapTrafficRoute yieldingMergeRoute;
    auto yieldingMergeApproach = straight(1247, 0.0, -40.0, 0.0, 34.0);
    yieldingMergeApproach.pathRules.groups.push_back(mergeYieldRules);
    auto yieldingMergeConnector = straight(1248, 0.0, -6.0, 0.0, 12.0);
    yieldingMergeConnector.points = {{0.0, -6.0, 0.0}, {0.0, 6.0, 0.0}};
    yieldingMergeRoute.segments = {yieldingMergeApproach, yieldingMergeConnector,
                                   straight(1249, 0.0, 6.0, 0.0, 60.0)};
    yieldingMergeRoute.cumulativeEndDistances = {34.0, 46.0, 106.0};
    yieldingMergeRoute.totalLength = 106.0;
    auto priorityMergeVehicle =
        makeMapTrafficActor(priorityMergeRoute, "priority-merge.ovh", 8.0, 0.0, false);
    auto yieldingMergeVehicle =
        makeMapTrafficActor(yieldingMergeRoute, "yielding-merge.ovh", 8.0, 0.0, false);
    priorityMergeVehicle.unscheduledGroupIndex = 0;
    yieldingMergeVehicle.unscheduledGroupIndex = 0;
    priorityMergeVehicle.trafficRuleGroupIndex = 0;
    yieldingMergeVehicle.trafficRuleGroupIndex = 0;
    priorityMergeVehicle.distanceMeters = 26.0;
    yieldingMergeVehicle.distanceMeters = 26.0;
    double firstMergePriorityDistance = -1.0;
    double firstMergeYieldDistance = -1.0;
    bool priorityMergeEnteredFirst = false;
    for (int frame = 0; frame < 400; ++frame) {
        advanceMapTrafficDriving({&yieldingMergeVehicle, &priorityMergeVehicle}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(priorityMergeVehicle),
                                       {footprint(yieldingMergeVehicle)}, 0.49),
                "priority arbitration keeps merging vehicles physically separated");
        priorityMergeEnteredFirst =
            priorityMergeEnteredFirst || (priorityMergeVehicle.distanceMeters > 42.0 &&
                                          yieldingMergeVehicle.distanceMeters < 40.0);
        if (firstMergePriorityDistance < 0.0 && priorityMergeVehicle.distanceMeters > 40.0) {
            firstMergePriorityDistance = priorityMergeVehicle.distanceMeters;
        }
        if (firstMergeYieldDistance < 0.0 && yieldingMergeVehicle.distanceMeters > 40.0) {
            firstMergeYieldDistance = yieldingMergeVehicle.distanceMeters;
        }
    }
    require(priorityMergeEnteredFirst && firstMergePriorityDistance > 40.0 &&
                firstMergeYieldDistance > 40.0 && priorityMergeVehicle.distanceMeters > 70.0 &&
                yieldingMergeVehicle.distanceMeters > 70.0,
            "the authored higher-priority spline claims a shared merge before the yielding spline");
    auto lateRightTurn =
        makeMapTrafficActor(rightTurnRoute, "late-right-turn.ovh", 8.0, 0.0, false);
    auto passedPriorityCrossing = makeMapTrafficActor(
        highPriorityCrossingRoute, "passed-priority-crossing.ovh", 30.0, 0.0, false);
    lateRightTurn.unscheduledGroupIndex = 0;
    passedPriorityCrossing.unscheduledGroupIndex = 0;
    lateRightTurn.trafficRuleGroupIndex = 0;
    passedPriorityCrossing.trafficRuleGroupIndex = 0;
    passedPriorityCrossing.distanceMeters = 20.0;
    passedPriorityCrossing.speedMetersPerSecond = 30.0;
    double lateTurnMinimumSpeed = std::numeric_limits<double>::infinity();
    for (int frame = 0; frame < 12; ++frame) {
        advanceMapTrafficDriving({&lateRightTurn, &passedPriorityCrossing}, {}, 0.1);
        lateTurnMinimumSpeed = std::min(lateTurnMinimumSpeed, lateRightTurn.speedMetersPerSecond);
    }
    require(lateRightTurn.distanceMeters > 8.0 && lateTurnMinimumSpeed > 7.0,
            "a turn continues when priority traffic clears before it reaches the crossing");
    MapTrafficRoute distantCrossingRoute = oneSegment(straight(1330, -40.0, -5.0, 90.0, 100.0));
    distantCrossingRoute.segments.front().points = {{-40.0, -5.0, 0.0}, {60.0, -5.0, 0.0}};
    auto rightTurnPastDistantClaim =
        makeMapTrafficActor(rightTurnRoute, "right-turn-past-distant-claim.ovh", 8.0, 0.0, false);
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
    distantHorizontalRoute.segments = {straight(1331, -40.0, 0.0, 90.0, 30.0),
                                       straight(1332, -10.0, 0.0, 90.0, 20.0),
                                       straight(1333, 10.0, 0.0, 90.0, 30.0)};
    distantHorizontalRoute.segments[1].points = {{-10.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
    distantHorizontalRoute.cumulativeEndDistances = {30.0, 50.0, 80.0};
    distantHorizontalRoute.totalLength = 80.0;
    MapTrafficRoute nearVerticalRoute;
    nearVerticalRoute.segments = {straight(1334, 0.0, -40.0, 0.0, 30.0),
                                  straight(1335, 0.0, -10.0, 0.0, 20.0),
                                  straight(1336, 0.0, 10.0, 0.0, 30.0)};
    nearVerticalRoute.segments[1].points = {{0.0, -10.0, 0.0}, {0.0, 10.0, 0.0}};
    nearVerticalRoute.cumulativeEndDistances = {30.0, 50.0, 80.0};
    nearVerticalRoute.totalLength = 80.0;
    auto distantHorizontal =
        makeMapTrafficActor(distantHorizontalRoute, "distant-horizontal.ovh", 8.0, 0.0, false);
    auto nearVertical =
        makeMapTrafficActor(nearVerticalRoute, "near-vertical.ovh", 8.0, 0.0, false);
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
        require(
            mapTrafficHasClearance(footprint(distantHorizontal), {footprint(nearVertical)}, 0.49),
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
    agedRightTurn.trafficRuleGroupIndex = 0;
    agedCrossing.trafficRuleGroupIndex = 0;
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
    const std::vector<MapTrafficRouteRequirement> repeatedMapSplineTimetable{
        {20, 47}, {20, 46}, {21, 24}, {21, 23}, {22, 0}};
    const std::vector<MapTrafficRouteRequirement> mapSplineRoute{
        {20, std::nullopt}, {21, std::nullopt}, {22, std::nullopt}};
    require(mapTrafficTimetablePathContainedInRoute(repeatedMapSplineTimetable, mapSplineRoute),
            "repeated checkpoints on ordinary map splines match one route segment each");
    require(!mapTrafficTimetablePathContainedInRoute(
                repeatedMapSplineTimetable,
                std::vector<MapTrafficRouteRequirement>{{20, std::nullopt}, {21, std::nullopt}}),
            "a route missing the timetable suffix remains a partial match");
    const std::vector<MapTrafficRouteRequirement> repeatedSceneryPathTimetable{
        {30, 13}, {30, 12}, {31, 0}};
    const std::vector<MapTrafficRouteRequirement> sceneryPathRoute{
        {30, 13}, {30, 12}, {31, std::nullopt}};
    require(mapTrafficTimetablePathContainedInRoute(repeatedSceneryPathTimetable, sceneryPathRoute),
            "repeated placed-scenery IDs retain their distinct timetable path selectors");
    require(
        !mapTrafficTimetablePathContainedInRoute({{30, 13}, {30, 11}, {31, 0}}, sceneryPathRoute),
        "a scenery timetable match rejects a different authored path selector");
    const auto repeatedPrefix = mapTrafficRoutePrefixMatch(
        repeatedMapSplineTimetable,
        std::vector<MapTrafficRouteRequirement>{{21, std::nullopt}, {22, std::nullopt}});
    require(repeatedPrefix.routePrefix.size() == 2 &&
                repeatedPrefix.routePrefix.back().splineId == 22 &&
                repeatedPrefix.timetableStartIndex == 2 &&
                repeatedPrefix.timetableEntriesMatched == 3,
            "partial matching collapses repeated map-spline checkpoints and reports its position");
    entryActor.distanceMeters = entryActor.route.totalLength;
    require(mapTrafficRouteEndReadyForUnload(entryActor),
            "vehicles unload immediately when they reach an open route end");
    entryActor.speedMetersPerSecond = 4.0;
    entryActor.junctionWaitSeconds = 8.0;
    entryActor.committedSignalSegmentIndex = 1;
    resetMapTrafficActorForEntry(entryActor);
    requireNear(entryActor.distanceMeters, 0.0,
                "recycled traffic re-enters at route distance zero");
    requireNear(entryActor.speedMetersPerSecond, 0.0,
                "recycled traffic starts from rest rather than retaining route speed");
    requireNear(entryActor.junctionWaitSeconds, 0.0,
                "recycled traffic does not retain junction wait priority from its prior trip");
    require(!entryActor.committedSignalSegmentIndex.has_value(),
            "recycled traffic does not retain permission to pass a red signal");
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
        auto mirrorViewProjection = viewProjection;
        mirrorViewProjection[7] = -1.0;
        mirrorViewProjection[6] = -viewProjection[6];
        const MapTrafficCameraView mainCamera{camera, viewProjection};
        const std::vector<MapTrafficCameraView> mirrorCameras = {{camera, mirrorViewProjection}};
        require(mapTrafficEntryHiddenFromCamera({0.0, -100.0, 0.0}, 3.0, camera, viewProjection,
                                                50.0) &&
                    !mapTrafficEntryHiddenFromCameras({0.0, -100.0, 0.0}, 3.0, mainCamera,
                                                      mirrorCameras, 50.0),
                "traffic hidden from the forward view remains visible to a mirror camera");
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
    auto tightCurveSegment = straight(1401, 0.0, 0.0, 0.0, 10.0 * std::acos(-1.0));
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
    require(
        mapTrafficHasClearance(footprint(tightCurveFollower), {footprint(tightCurveLeader)}, 0.49),
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
    require(signalActor.speedMetersPerSecond < 8.0 && signalActor.speedMetersPerSecond > 0.0,
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

    {
        auto closeActor = makeMapTrafficActor(signalRoute, "close.ovh", 8.0, 0.0, false);
        closeActor.distanceMeters = 12.0;
        advanceMapTrafficDriving({&closeActor}, {}, 0.05, 199.9);
        const double beforeRed = closeActor.distanceMeters;
        advanceMapTrafficDriving({&closeActor}, {}, 0.05, 200.0);
        require(closeActor.committedSignalSegmentIndex == 1 &&
                    closeActor.speedMetersPerSecond >= 8.0 && closeActor.distanceMeters > beforeRed,
                "a vehicle inside its braking distance when green turns red commits to passing");
        const auto debug = mapTrafficNextSignalStopDebugInfo(closeActor, 200.05);
        require(debug.has_value() && !debug->blocked,
                "signal stop overlay reflects the committed pass decision");
        advanceMapTrafficDriving({&closeActor}, {}, 1.0, 200.05);
        require(closeActor.distanceMeters > 20.0 &&
                    !closeActor.committedSignalSegmentIndex.has_value(),
                "a committed vehicle crosses red and clears its decision after the entry");
        closeActor.route.segments[2].trafficSignal = redThenGreen;
        closeActor.speedMetersPerSecond = 0.0;
        advanceMapTrafficDriving({&closeActor}, {}, 3.0, 201.05);
        require(closeActor.distanceMeters <= 21.001 &&
                    !closeActor.committedSignalSegmentIndex.has_value() &&
                    mapTrafficActorWaitingAtSignal(closeActor, 204.05),
                "passing one red signal does not grant permission at the next signal");

        auto slowedActor = makeMapTrafficActor(signalRoute, "slowed.ovh", 8.0, 0.0, false);
        slowedActor.distanceMeters = 16.0;
        advanceMapTrafficDriving({&slowedActor}, {}, 0.05);
        slowedActor.speedMetersPerSecond = 0.0;
        require(!mapTrafficActorWaitingAtSignal(slowedActor, 0.05),
                "a committed vehicle slowed by other traffic is not classified as waiting at red");
        advanceMapTrafficDriving({&slowedActor}, {}, 3.0, 0.05);
        require(slowedActor.distanceMeters > 20.0,
                "a committed pass survives slowing below the original braking threshold");

        auto stoppedActor = makeMapTrafficActor(signalRoute, "stopped.ovh", 8.0, 0.0, false);
        stoppedActor.distanceMeters = 17.0;
        stoppedActor.speedMetersPerSecond = 0.0;
        advanceMapTrafficDriving({&stoppedActor}, {}, 2.0);
        requireNear(stoppedActor.distanceMeters, 17.0,
                    "a vehicle already stopped at red does not acquire a pass commitment");
        require(mapTrafficActorWaitingAtSignal(stoppedActor, 2.0),
                "a vehicle stopped at red remains classified as waiting");

        auto boundaryActor = makeMapTrafficActor(signalRoute, "boundary.ovh", 6.0, 0.0, false);
        boundaryActor.distanceMeters = 11.0;
        advanceMapTrafficDriving({&boundaryActor}, {}, 0.05);
        require(!boundaryActor.committedSignalSegmentIndex.has_value() &&
                    boundaryActor.speedMetersPerSecond < 6.0,
                "a vehicle exactly one braking distance from red still brakes");

        auto blockedActor = makeMapTrafficActor(signalRoute, "blocked.ovh", 8.0, 0.0, false);
        blockedActor.distanceMeters = 12.0;
        const MapTrafficObstacle obstruction{*sampleMapTrafficRoute(signalRoute, 21.0, 0.0), 4.5,
                                             1.8};
        advanceMapTrafficDriving({&blockedActor}, {obstruction}, 3.0);
        require(blockedActor.distanceMeters < 17.0 &&
                    mapTrafficHasClearance(footprint(blockedActor), {obstruction}, 0.49),
                "permission to pass red never bypasses obstacle clearance");

        auto invalidActor = makeMapTrafficActor(signalRoute, "invalid.ovh", 8.0, 0.0, false);
        invalidActor.distanceMeters = 12.0;
        invalidActor.route.segments[1].trafficSignal->cycleDurationSeconds = 0.0;
        advanceMapTrafficDriving({&invalidActor}, {}, 2.0);
        require(invalidActor.distanceMeters <= 17.001 &&
                    !invalidActor.committedSignalSegmentIndex.has_value(),
                "an invalid signal schedule remains blocking even inside braking distance");
    }

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
    loopSignalActor.speedMetersPerSecond = 0.0;
    for (int frame = 0; frame < 100; ++frame) {
        advanceMapTrafficDriving({&loopSignalActor}, {}, 0.05);
    }
    require(loopSignalActor.distanceMeters < loopSignalRoute.totalLength - 2.0 &&
                loopSignalActor.speedMetersPerSecond < 1.0e-6,
            "AI traffic approaching a signal across a loop wrap stops before red");
    advanceMapTrafficDriving({&loopSignalActor}, {}, 3.0, 100.0);
    require(loopSignalActor.distanceMeters < 10.0,
            "AI traffic resumes across the loop wrap when the signal turns green");
    auto committedLoopActor =
        makeMapTrafficActor(loopSignalRoute, "committed-loop.ovh", 8.0, 0.0, false);
    committedLoopActor.distanceMeters = loopSignalRoute.totalLength - 5.0;
    advanceMapTrafficDriving({&committedLoopActor}, {}, 1.0);
    require(
        committedLoopActor.distanceMeters < 10.0 &&
            !committedLoopActor.committedSignalSegmentIndex.has_value(),
        "a too-close vehicle passes a red signal across a loop wrap without retaining permission");
    committedLoopActor.distanceMeters = loopSignalRoute.totalLength - 20.0;
    advanceMapTrafficDriving({&committedLoopActor}, {}, 15.0);
    require(committedLoopActor.distanceMeters >= loopSignalRoute.totalLength - 3.2 &&
                committedLoopActor.distanceMeters <= loopSignalRoute.totalLength - 2.999 &&
                committedLoopActor.speedMetersPerSecond < 1.0e-6,
            "the same looped vehicle stops at red on its next approach with room to brake");

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
    const MapTrafficPathRules unrestrictedRoad;
    require(mapTrafficPathAllowsVehicle(unrestrictedRoad, {0, std::nullopt, false}) &&
                mapTrafficPathAllowsVehicle(unrestrictedRoad, {1, std::nullopt, false}) &&
                mapTrafficPathAllowsVehicle(unrestrictedRoad, {3, std::nullopt, false}),
            "unmarked road paths admit cars, commercial vehicles and trucks by default");
    MapTrafficPathRules noTrafficPriorityRoad;
    noTrafficPriorityRoad.priority = 0.0;
    require(!mapTrafficPathAllowsVehicle(noTrafficPriorityRoad, {0, 0, false, 0}) &&
                !mapTrafficPathAllowsVehicle(noTrafficPriorityRoad, {2, std::nullopt, true}),
            "priority-zero paths exclude unscheduled and scheduled traffic");
    MapTrafficPathRules groupPriorityRoad;
    groupPriorityRoad.groups = {{0, std::nullopt, std::nullopt, 0.0},
                                {1, std::nullopt, std::nullopt, 0.0}};
    require(!mapTrafficPathAllowsVehicle(groupPriorityRoad, {0, 0, false, 0}) &&
                !mapTrafficPathAllowsVehicle(groupPriorityRoad, {0, 1, false, 1}) &&
                !mapTrafficPathAllowsVehicle(groupPriorityRoad, {2, std::nullopt, true}),
            "group priority zero excludes that group and paths closed to all scheduled traffic");
    groupPriorityRoad.groups[1].priority = 64.0;
    require(mapTrafficPathAllowsVehicle(groupPriorityRoad, {0, 1, false, 1}) &&
                mapTrafficPathAllowsVehicle(groupPriorityRoad, {2, std::nullopt, true}),
            "positive group priority keeps eligible traffic paths available");
    MapTrafficPathRules truckRestrictedRoad;
    truckRestrictedRoad.trucksAllowed = false;
    require(!mapTrafficPathAllowsVehicle(truckRestrictedRoad, {3, std::nullopt, false}),
            "an explicit truck restriction still excludes trucks");

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
        {false, {"0", "no_unsched_traffic", "1", "3"}},
        {false, {"0", "trafficdensity", "0.25", "7"}},
        {false, {"1", "speedlimit", "not-a-number", "0"}},
    };
    const MapTrafficPathRules rules = mapTrafficRulesForPath(authoredRules, 0);
    require(mapTrafficPathSpeedLimitForVehicle(rules, {0, 1, false, 0}) == 36.0 &&
                mapTrafficPathPriorityForVehicle(rules, {0, 1, false, 0}) == 192.0 &&
                mapTrafficPathSpeedLimitForVehicle(rules, {0, 0, false, 1}) == 36.0 &&
                mapTrafficPathPriorityForVehicle(rules, {0, 0, false, 1}) == 192.0,
            "group rules use the traffic-rule index, with missing properties falling back to group "
            "zero");
    const MapTrafficPathRules groupZeroFallbackRules =
        mapTrafficRulesForPath({{false, {"0", "speedlimit", "36", "0"}},
                                {false, {"0", "trafficdensity", "0.5", "0"}},
                                {false, {"0", "priority", "192", "0"}},
                                {false, {"0", "no_cars", "1", "0"}},
                                {false, {"0", "no_unsched_traffic", "1", "0"}},
                                {false, {"0", "bus", "1", "0"}},
                                {false, {"0", "trucks", "1", "0"}},
                                {false, {"0", "trafficdensity", "0.25", "4"}},
                                {false, {"0", "no_cars", "0", "4"}},
                                {false, {"0", "no_unsched_traffic", "0", "4"}}},
                               0);
    require(
        mapTrafficPathSpeedLimitForVehicle(groupZeroFallbackRules, {0, 0, false, 4}) == 36.0 &&
            mapTrafficPathPriorityForVehicle(groupZeroFallbackRules, {0, 0, false, 4}) == 192.0 &&
            mapTrafficPathDensityForVehicle(groupZeroFallbackRules, {0, 0, false, 4}) == 0.25 &&
            mapTrafficPathAllowsVehicle(groupZeroFallbackRules, {0, 0, false, 4}) &&
            mapTrafficPathSpeedLimitForVehicle(groupZeroFallbackRules, {0, 0, false, 5}) == 36.0 &&
            mapTrafficPathPriorityForVehicle(groupZeroFallbackRules, {0, 0, false, 5}) == 192.0 &&
            mapTrafficPathDensityForVehicle(groupZeroFallbackRules, {0, 0, false, 5}) == 0.0 &&
            !mapTrafficPathAllowsVehicle(groupZeroFallbackRules, {0, 0, false, 5}),
        "groups inherit group-zero path properties unless their own rule overrides that property, "
        "including explicit zeros and prohibitions");
    const MapTrafficPathRules groupZeroAccessRules =
        mapTrafficRulesForPath({{false, {"0", "no_cars", "1", "0"}},
                                {false, {"0", "no_cars", "0", "4"}},
                                {false, {"0", "bus", "1", "0"}},
                                {false, {"0", "trucks", "1", "0"}}},
                               0);
    require(mapTrafficPathAllowsVehicle(groupZeroAccessRules, {0, 0, false, 4}) &&
                !mapTrafficPathAllowsVehicle(groupZeroAccessRules, {0, 0, false, 5}) &&
                mapTrafficPathAllowsVehicle(groupZeroAccessRules, {1, 0, false, 5}) &&
                mapTrafficPathAllowsVehicle(groupZeroAccessRules, {3, 0, false, 5}),
            "access restrictions and permissions inherit group-zero rules while explicit group "
            "rules win");
    const MapTrafficPathRules zeroDensityRules = mapTrafficRulesForPath(
        {{false, {"0", "trafficdensity", "0", "0"}}, {false, {"0", "trafficdensity", "0", "1"}}},
        0);
    require(!mapTrafficPathAllowsVehicle(zeroDensityRules, {0, 0, false, 0}) &&
                !mapTrafficPathAllowsVehicle(zeroDensityRules, {0, 1, false, 1}) &&
                !mapTrafficPathAllowsVehicle(zeroDensityRules, {2, std::nullopt, true}),
            "zero density for every traffic group excludes scheduled and unscheduled vehicles");
    const MapTrafficPathRules groupSpecificZeroDensity = mapTrafficRulesForPath(
        {{false, {"0", "trafficdensity", "0", "0"}}, {false, {"0", "trafficdensity", "0.5", "1"}}},
        0);
    require(!mapTrafficPathAllowsVehicle(groupSpecificZeroDensity, {0, 0, false, 0}) &&
                mapTrafficPathAllowsVehicle(groupSpecificZeroDensity, {0, 1, false, 1}) &&
                mapTrafficPathAllowsVehicle(groupSpecificZeroDensity, {2, std::nullopt, true}),
            "group-specific zero density blocks its group without closing positive-density routes");
    const MapTrafficPathRules singleGroupDensityRule =
        mapTrafficRulesForPath({{false, {"0", "trafficdensity", "0", "0"}}}, 0);
    require(!mapTrafficPathAllowsVehicle(singleGroupDensityRule, {2, std::nullopt, true}),
            "scheduled buses are excluded when all configured density rules are zero");
    MapTrafficLaneNetwork zeroDensityLaneNetwork;
    MapTrafficRouteSegment zeroDensityLane;
    zeroDensityLane.spline.placement = makeSpline(1200, 0, 1201, 0.0);
    zeroDensityLane.pathRules = zeroDensityRules;
    zeroDensityLaneNetwork.addSegment(std::move(zeroDensityLane));
    MapTrafficRouteSegment densityContinuationLane;
    densityContinuationLane.spline.placement = makeSpline(1201, 1200, 0, 10.0);
    zeroDensityLaneNetwork.addSegment(std::move(densityContinuationLane));
    zeroDensityLaneNetwork.finalize();
    require(!zeroDensityLaneNetwork.selectSpawnLane({0, 0, false, 0}, 1) &&
                !zeroDensityLaneNetwork.selectSpawnLane({2, std::nullopt, true}, 2) &&
                !zeroDensityLaneNetwork.buildRoute(1200, 0, 0, 0.0, {0, 0, false, 0}) &&
                !zeroDensityLaneNetwork.buildRoute(1200, 0, 0, 0.0, {2, std::nullopt, true}),
            "zero-density lanes are neither spawn points nor traversable route segments");
    require(rules.groups.size() == 4 && rules.groups[0].carsProhibited == true &&
                rules.groups[0].busesAllowed == true && rules.groups[0].trucksAllowed == true &&
                rules.groups[0].overtakingProhibited == true && rules.groups[2].groupIndex == 3 &&
                rules.groups[2].unscheduledTrafficProhibited == true &&
                rules.groups[3].groupIndex == 7,
            "access, overtaking and no-unscheduled-traffic rules decode as group overrides");
    requireNear(mapTrafficPathDensityForVehicle(rules, {0, 1, false, 0}), 0.5,
                "traffic density resolves for its matching vehicle group");
    requireNear(mapTrafficPathDensityForVehicle(rules, {0, 0, false, 1}), 0.0,
                "the last matching group density rule takes effect");
    requireNear(mapTrafficPathDensityForVehicle(rules, {0, 0, false, 3}), 0.0,
                "no_unsched_traffic removes the matching group's effective path density");
    requireNear(mapTrafficPathDensityForVehicle(rules, {0, 0, false, 7}), 0.25,
                "traffic rule groups above three parse and match without truncation");
    MapTrafficRoute spawnSpeedRoute;
    MapTrafficRouteSegment spawnSpeedSegment;
    spawnSpeedSegment.points = {{0.0, 0.0, 0.0}, {0.0, 100.0, 0.0}};
    spawnSpeedRoute.segments.push_back(std::move(spawnSpeedSegment));
    spawnSpeedRoute.cumulativeEndDistances = {100.0};
    spawnSpeedRoute.totalLength = 100.0;
    auto spawnSpeedActor =
        makeMapTrafficActor(std::move(spawnSpeedRoute), "car.ovh", 12.0, 0.0, false);
    spawnSpeedActor.vehicleType = 0;
    spawnSpeedActor.trafficRuleGroupIndex = 0;
    spawnSpeedActor.route.segments.front().pathRules.speedLimitKmh = 30.0;
    requireNear(mapTrafficSpawnSpeedMetersPerSecond(spawnSpeedActor, 0.0), 30.0 / 3.6,
                "traffic actors spawn at the posted speed limit when it is below cruise speed");
    spawnSpeedActor.route.segments.front().pathRules.speedLimitKmh = 60.0;
    requireNear(mapTrafficSpawnSpeedMetersPerSecond(spawnSpeedActor, 0.0), 12.0,
                "traffic spawn speed never exceeds the configured cruise speed");
    spawnSpeedActor.route.segments.front().pathRules.speedLimitKmh.reset();
    requireNear(mapTrafficSpawnSpeedMetersPerSecond(spawnSpeedActor, 0.0), 12.0,
                "traffic without a posted limit uses its configured cruise speed");
    require(mapTrafficPathAllowsVehicle(rules, {1, 1, false, 0}) &&
                mapTrafficPathAllowsVehicle(rules, {2, 1, false, 0}) &&
                mapTrafficPathAllowsVehicle(rules, {3, 1, false, 0}) &&
                !mapTrafficPathAllowsVehicle(rules, {0, 1, false, 0}) &&
                !mapTrafficPathAllowsVehicle(rules, {3, 0, false, 1}) &&
                !mapTrafficPathAllowsVehicle(rules, {0, 1, false, 2}) &&
                !mapTrafficPathAllowsVehicle(rules, {0, 0, false, 3}) &&
                mapTrafficPathAllowsVehicle(rules, {2, std::nullopt, true}),
            "path rules enforce unscheduled access and density while preserving scheduled buses");
    const MapTrafficPathRules scheduledBusRestrictedRoad =
        mapTrafficRulesForPath({{false, {"0", "bus", "0", "0"}},
                                {false, {"0", "trafficdensity", "0", "0"}},
                                {false, {"0", "no_unsched_traffic", "1", "0"}}},
                               0);
    require(!mapTrafficPathAllowsVehicle(scheduledBusRestrictedRoad, {2, std::nullopt, true}),
            "scheduled buses respect explicit bus access restrictions");
    const MapTrafficPathRules scheduledBusAllowedRoad =
        mapTrafficRulesForPath({{false, {"0", "bus", "1", "0"}},
                                {false, {"0", "trafficdensity", "0", "0"}},
                                {false, {"0", "no_unsched_traffic", "1", "0"}}},
                               0);
    require(!mapTrafficPathAllowsVehicle(scheduledBusAllowedRoad, {2, std::nullopt, true}),
            "an explicit bus allowance cannot override a path with zero traffic density");
    const MapTrafficPathRules afterKill =
        mapTrafficRulesForPath({{false, {"0", "no_cars", "1", "0"}},
                                {true, {"0", "no_cars", "1", "0"}},
                                {false, {"0", "trafficdensity", "0", "1"}},
                                {true, {"0", "trafficdensity", "0", "1"}},
                                {false, {"0", "no_unsched_traffic", "1", "2"}},
                                {true, {"0", "no_unsched_traffic", "1", "2"}}},
                               0);
    require(!afterKill.groups[0].carsProhibited.has_value() &&
                mapTrafficPathDensityForVehicle(afterKill, {0, 0, false, 1}) == 1.0 &&
                !afterKill.groups[1].unscheduledTrafficProhibited.has_value() &&
                mapTrafficPathAllowsVehicle(afterKill, {0, 0, false, 2}),
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
    bool negativeVehicleGroupRejected = false;
    try {
        static_cast<void>(mapTrafficRulesForPath(
            std::vector<MapTileRule>{{false, {"0", "trafficdensity", "1", "-1"}}}, 0));
    } catch (const std::invalid_argument&) {
        negativeVehicleGroupRejected = true;
    }
    require(negativeVehicleGroupRejected, "negative traffic rule vehicle groups are rejected");

    MapTrafficRouteSegment restrictedSegment;
    restrictedSegment.spline = {makeSpline(1300, 0, 0, 0.0), 0, 0};
    restrictedSegment.pathRules = rules;
    MapTrafficLaneNetwork restrictedNetwork;
    restrictedNetwork.addSegment(restrictedSegment);
    restrictedNetwork.finalize();
    require(
        !restrictedNetwork.buildRoute(1300, 0, 0, 0.0, {0, 1, false, 0}) &&
            restrictedNetwork.buildRoute(1300, 0, 0, 0.0, {2, 1, false, 0}) &&
            !restrictedNetwork.buildRoute(1300, 0, 0, 0.0, {3, 1, false, 1}) &&
            !restrictedNetwork.buildRoute(1300, 0, 0, 0.0, {0, 1, false, 3}) &&
            restrictedNetwork.buildRoute(1300, 0, 0, 0.0, {2, std::nullopt, true}),
        "route construction rejects prohibited lanes and preserves allowed scheduled bus lanes");
    MapTrafficRouteSegment scheduledRestrictedSegment;
    scheduledRestrictedSegment.spline = {makeSpline(1301, 0, 0, 0.0), 0, 0};
    scheduledRestrictedSegment.pathRules = scheduledBusRestrictedRoad;
    MapTrafficLaneNetwork scheduledRestrictedNetwork;
    scheduledRestrictedNetwork.addSegment(scheduledRestrictedSegment);
    scheduledRestrictedNetwork.finalize();
    require(!scheduledRestrictedNetwork.buildRoute(1301, 0, 0, 0.0, {2, std::nullopt, true}),
            "route construction rejects scheduled buses on explicitly prohibited paths");

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
        const auto entry = densityNetwork.selectEntryLane({0, 7, false, 0}, seed);
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
    require(!densityNetwork.buildRoute(1314, 0, 0, 0.0, {0, 7, false, 0}),
            "an unscheduled vehicle cannot start a route on a zero-density path");
    for (std::size_t seed = 1; seed <= 1000; ++seed) {
        const auto spawn = densityNetwork.selectSpawnLane({0, 7, false, 0}, seed);
        require(spawn.has_value() && spawn->splineId != 1314,
                "zero-density paths are excluded from unscheduled spawn selection");
        require(densityNetwork.selectSpawnLaneInTiles({0, 7, false, 0}, seed, {{0, 0}}) == spawn,
                "local spawn lane selection retains weighted vehicle-specific path rules");
    }

    MapTrafficLaneNetwork zeroDensityContinuation;
    zeroDensityContinuation.addSegment(straightSegment(1320, 0.0, 0.0));
    auto zeroDensityPath = straightSegment(1321, 0.0, 10.0);
    MapTrafficGroupRules zeroDensityPathRules;
    zeroDensityPathRules.groupIndex = 0;
    zeroDensityPathRules.density = 0.0;
    zeroDensityPath.pathRules.groups.push_back(zeroDensityPathRules);
    zeroDensityContinuation.addSegment(zeroDensityPath);
    zeroDensityContinuation.finalize();
    const auto routeBeforeZeroDensityPath =
        zeroDensityContinuation.buildRoute(1320, 0, 0, 0.0, {0, 7, false, 0});
    require(!routeBeforeZeroDensityPath,
            "an unscheduled route with only a zero-density continuation cannot be built");

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
    speedCapped.trafficRuleGroupIndex = 0;
    MapTrafficActor reducedSpeedCapped = speedCapped;
    reducedSpeedCapped.reducedSpeedLookahead = true;
    advanceMapTrafficDriving({&speedCapped}, {}, 4.0);
    advanceMapTrafficDriving({&reducedSpeedCapped}, {}, 4.0);
    require(speedCapped.speedMetersPerSecond <= 5.0 + 1.0e-8,
            "authored speed limits cap vehicle speed in km/h converted to m/s");
    requireNear(reducedSpeedCapped.targetSpeedMetersPerSecond,
                speedCapped.targetSpeedMetersPerSecond,
                "reduced curve sampling retains exact authored speed-limit decisions");

    MapTrafficRouteSegment firstSpeedSegment;
    firstSpeedSegment.spline = {makeSpline(1302, 0, 0, 0.0), 0, 0};
    firstSpeedSegment.spline.placement.length = 20.0;
    MapTrafficRouteSegment nextSpeedSegment;
    nextSpeedSegment.spline = {makeSpline(1303, 0, 0, 20.0), 0, 0};
    nextSpeedSegment.spline.placement.length = 100.0;
    nextSpeedSegment.pathRules.groups.push_back(speedRules);
    MapTrafficRoute changingSpeedRoute;
    changingSpeedRoute.segments = {firstSpeedSegment, nextSpeedSegment};
    changingSpeedRoute.cumulativeEndDistances = {20.0, 120.0};
    changingSpeedRoute.totalLength = 120.0;
    auto approachingSpeedLimit =
        makeMapTrafficActor(changingSpeedRoute, "changing-speed.ovh", 12.0, 0.0, false);
    approachingSpeedLimit.distanceMeters = 12.0;
    approachingSpeedLimit.speedMetersPerSecond = 8.0;
    approachingSpeedLimit.trafficRuleGroupIndex = 0;
    advanceMapTrafficDriving({&approachingSpeedLimit}, {}, 0.05);
    requireNear(approachingSpeedLimit.targetSpeedMetersPerSecond, std::sqrt(67.0),
                "lookahead applies a new group speed limit at the route segment boundary");

    changingSpeedRoute.closed = true;
    changingSpeedRoute.segments.front().pathRules.groups.push_back(speedRules);
    changingSpeedRoute.segments.back().pathRules.groups.clear();
    auto approachingLoopSpeedLimit =
        makeMapTrafficActor(changingSpeedRoute, "loop-speed.ovh", 12.0, 0.0, false);
    approachingLoopSpeedLimit.distanceMeters = 112.0;
    approachingLoopSpeedLimit.speedMetersPerSecond = 8.0;
    approachingLoopSpeedLimit.trafficRuleGroupIndex = 0;
    advanceMapTrafficDriving({&approachingLoopSpeedLimit}, {}, 0.05);
    requireNear(approachingLoopSpeedLimit.targetSpeedMetersPerSecond, std::sqrt(61.0),
                "lookahead applies a group speed limit across a closed route seam");
}

void benchmarkDrivingLookahead(std::size_t actorCount = 69, double spacingMeters = 100.0,
                               bool reducedSpeedLookahead = false,
                               bool sharedDirectedRoute = false) {
    using namespace openbus::map;
    std::vector<MapTrafficActor> actors(actorCount);
    std::vector<MapTrafficActor*> pointers;
    for (std::size_t index = 0; index < actors.size(); ++index) {
        auto& actor = actors[index];
        for (int segmentIndex = 0; segmentIndex < 20; ++segmentIndex) {
            MapTrafficRouteSegment segment;
            segment.spline.placement = makeSpline(
                10000 + static_cast<int>(sharedDirectedRoute ? 0 : index) * 20 + segmentIndex, 0,
                0, segmentIndex * (sharedDirectedRoute ? 5000.0 : 10.0));
            segment.spline.placement.length = sharedDirectedRoute ? 5000.0 : 10.0;
            segment.spline.placement.localX =
                sharedDirectedRoute ? 0.0 : static_cast<double>(index) * spacingMeters;
            actor.route.segments.push_back(segment);
            actor.route.cumulativeEndDistances.push_back(
                (segmentIndex + 1) * (sharedDirectedRoute ? 5000.0 : 10.0));
        }
        actor.route.totalLength = sharedDirectedRoute ? 100000.0 : 200.0;
        actor.distanceMeters = 20.0 + (sharedDirectedRoute ? index * spacingMeters : 0.0);
        actor.speedMetersPerSecond = 10.0;
        actor.cruiseSpeedMetersPerSecond = 12.0;
        actor.reducedSpeedLookahead = reducedSpeedLookahead;
        pointers.push_back(&actor);
    }
    // Warm up outside the timed region; actors have distinct, non-conflicting routes.
    advanceMapTrafficDriving(pointers, {}, 0.016);
    const auto start = std::chrono::steady_clock::now();
    constexpr int updates = 30;
    for (int update = 0; update < updates; ++update) {
        advanceMapTrafficDriving(pointers, {}, 0.016);
    }
    const double milliseconds =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    double distanceSum = 0.0;
    double speedSum = 0.0;
    for (const auto& actor : actors) {
        require(actor.blockReason == MapTrafficBlockReason::None,
                "distant movement reservations must not block free-flowing traffic");
        requireNear(actor.speedMetersPerSecond, 10.0 + 31.0 * 0.016 * 1.6,
                    "free-flowing benchmark actors accelerate without reservation constraints");
        distanceSum += actor.distanceMeters;
        speedSum += actor.speedMetersPerSecond;
    }
    std::cout << std::setprecision(17) << "TRAFFIC_LOOKAHEAD_BENCHMARK actors=" << actors.size()
              << " updates=" << updates << " reduced_speed_lookahead="
              << reducedSpeedLookahead << " shared_route=" << sharedDirectedRoute
              << " ms_per_update=" << milliseconds / updates
              << " distance_sum=" << distanceSum << " speed_sum=" << speedSum << '\n';
}

void benchmarkDrivingQueue(bool verify = false, bool mixedDimensions = false) {
    using namespace openbus::map;
    MapTrafficRouteSegment segment;
    segment.spline.placement = makeSpline(99000, 0, 0, 0.0);
    segment.spline.placement.localX = 0.0;
    segment.spline.placement.elevation = 0.0;
    segment.spline.placement.length = 1000.0;
    MapTrafficRoute route;
    route.segments = {segment};
    route.cumulativeEndDistances = {1000.0};
    route.totalLength = 1000.0;
    std::vector<MapTrafficActor> actors(104);
    std::vector<MapTrafficActor*> pointers;
    for (std::size_t index = 0; index < actors.size(); ++index) {
        auto& actor = actors[index];
        actor.route = route;
        actor.distanceMeters = 20.0 + index * 7.0;
        actor.speedMetersPerSecond = 3.0;
        actor.cruiseSpeedMetersPerSecond = 10.0;
        if (mixedDimensions) {
            actor.lengthMeters = index % 3 == 0 ? 6.0 : 4.0;
            actor.widthMeters = index % 2 == 0 ? 2.4 : 1.8;
            actor.rearAxleOffsetMeters = index % 4 == 0 ? -1.0 : 0.0;
        }
        pointers.push_back(&actor);
    }
    actors.back().speedMetersPerSecond = 0.0;
    actors.back().cruiseSpeedMetersPerSecond = 0.0;
    std::uint64_t stateHash = 1469598103934665603ULL;
    const auto mix = [&](std::uint64_t value) {
        stateHash ^= value;
        stateHash *= 1099511628211ULL;
    };
    const auto mixDouble = [&](double value) {
        std::uint64_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        mix(bits);
    };
    constexpr int updates = 120;
    double milliseconds = 0.0;
    for (int update = 0; update < updates; ++update) {
        const auto start = std::chrono::steady_clock::now();
        advanceMapTrafficDriving(pointers, {}, 0.05, update * 0.05);
        milliseconds += std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - start)
                            .count();
        if (verify) {
            for (std::size_t index = 1; index < actors.size(); ++index) {
                const double gap = actors[index].distanceMeters - actors[index - 1].distanceMeters -
                                   (actors[index].lengthMeters + actors[index - 1].lengthMeters) *
                                       0.5;
                require(gap >= -1.0e-8, "dense same-route queue preserves physical clearance");
            }
        }
        for (const auto& actor : actors) {
            mixDouble(actor.distanceMeters);
            mixDouble(actor.speedMetersPerSecond);
            mixDouble(actor.targetSpeedMetersPerSecond);
            mixDouble(actor.junctionWaitSeconds);
            mix(static_cast<std::uint64_t>(actor.blockReason));
            mix(actor.blockingActor == nullptr
                    ? actors.size()
                    : static_cast<std::size_t>(actor.blockingActor - actors.data()));
            mixDouble(actor.junctionClaimEndMeters.value_or(-1.0));
        }
    }
    if (verify && (actors.back().speedMetersPerSecond != 0.0 ||
                   actors[actors.size() - 2].speedMetersPerSecond > 1.0)) {
        throw std::runtime_error(
            "dense queue did not slow behind its leader: leader=" +
            std::to_string(actors.back().speedMetersPerSecond) + " follower=" +
            std::to_string(actors[actors.size() - 2].speedMetersPerSecond));
    }
    std::cout << std::setprecision(17) << "TRAFFIC_QUEUE_BENCHMARK actors=" << actors.size()
              << " updates=" << updates << " mixed_dimensions=" << mixedDimensions
              << " ms_per_update=" << milliseconds / updates
              << " state_hash=" << std::hex << stateHash << std::dec << '\n';
}

openbus::map::MapTrafficRoute makeOppositeLaneRoute(bool reverse, double x) {
    using namespace openbus::map;
    const double startY = reverse ? 120.0 : -40.0;
    const double direction = reverse ? -1.0 : 1.0;
    const double heading = reverse ? 180.0 : 0.0;
    const auto spline = [&](int id, double y) {
        MapTrafficRouteSegment segment;
        segment.spline.placement = makeSpline(id, 0, 0, y);
        segment.spline.placement.localX = x;
        segment.spline.placement.elevation = 0.0;
        segment.spline.placement.rotationDegrees = heading;
        segment.spline.placement.length = 40.0;
        return segment;
    };
    auto junction = spline(reverse ? 99101 : 99100, startY + direction * 40.0);
    junction.points = {{x, startY + direction * 40.0, 0.0},
                       {x, startY + direction * 120.0, 0.0}};
    MapTrafficRoute result;
    result.segments = {spline(reverse ? 99103 : 99102, startY), junction,
                       spline(reverse ? 99105 : 99104, startY + direction * 120.0)};
    result.cumulativeEndDistances = {40.0, 120.0, 160.0};
    result.totalLength = 160.0;
    return result;
}

void verifyLeaderWithCloseFollower() {
    using namespace openbus::map;
    for (const bool differentRoutes : {false, true}) {
        for (const bool reverseOrder : {false, true}) {
            for (const double initialSpeed : {0.0, 4.0}) {
                for (const double startDistance : {30.0, 60.0}) {
                    auto route = makeOppositeLaneRoute(false, 0.0);
                    auto followerRoute = route;
                    if (differentRoutes) {
                        followerRoute.segments.back().spline.placement.splineId += 100;
                    }
                    auto leader =
                        makeMapTrafficActor(route, "close-leader.ovh", 4.0, 0.0, false);
                    auto follower =
                        makeMapTrafficActor(followerRoute, "close-follower.ovh", 4.0, 0.0, false);
                    leader.rearAxleOffsetMeters = follower.rearAxleOffsetMeters = -1.35;
                    follower.lengthMeters = 5.1;
                    leader.distanceMeters = startDistance;
                    follower.distanceMeters =
                        startDistance - (leader.lengthMeters + follower.lengthMeters) * 0.5 - 0.05;
                    leader.speedMetersPerSecond = follower.speedMetersPerSecond = initialSpeed;
                    const std::vector<MapTrafficActor*> actors =
                        reverseOrder ? std::vector<MapTrafficActor*>{&follower, &leader}
                                     : std::vector<MapTrafficActor*>{&leader, &follower};
                    for (int step = 0; step < 60; ++step) {
                        advanceMapTrafficDriving(actors, {}, 0.05, step * 0.05);
                        if (leader.blockingActor == &follower) {
                            throw std::runtime_error(
                                "a leader yields to the follower behind it: step=" +
                                std::to_string(step) + " reason=" +
                                std::string(mapTrafficBlockReasonName(leader.blockReason)) +
                                " leader=" + std::to_string(leader.distanceMeters) +
                                " follower=" + std::to_string(follower.distanceMeters));
                        }
                        const auto leaderPose =
                            sampleMapTrafficActorPose(leader, leader.distanceMeters);
                        const auto followerPose =
                            sampleMapTrafficActorPose(follower, follower.distanceMeters);
                        require(leaderPose && followerPose &&
                                    mapTrafficHasClearance(
                                        {*leaderPose, leader.lengthMeters, leader.widthMeters},
                                        {{*followerPose, follower.lengthMeters, follower.widthMeters}},
                                        0.0),
                                "a leader moving away from a close follower maintains physical clearance");
                    }
                    require(leader.distanceMeters > startDistance + 5.0 &&
                                leader.speedMetersPerSecond > 3.0,
                            "a close follower cannot stop a clear leader");
                }
            }
        }
    }
    const auto route = makeOppositeLaneRoute(false, 0.0);
    auto front = makeMapTrafficActor(route, "stationary-front.ovh", 0.0, 0.0, false);
    auto leader = makeMapTrafficActor(route, "constrained-leader.ovh", 4.0, 0.0, false);
    auto follower = makeMapTrafficActor(route, "close-follower.ovh", 4.0, 0.0, false);
    front.distanceMeters = 66.5;
    leader.distanceMeters = 60.0;
    follower.distanceMeters = 55.45;
    leader.speedMetersPerSecond = follower.speedMetersPerSecond = 0.0;
    for (int step = 0; step < 60; ++step) {
        advanceMapTrafficDriving({&follower, &leader, &front}, {}, 0.05, step * 0.05);
        const auto frontPose = sampleMapTrafficActorPose(front, front.distanceMeters);
        const auto leaderPose = sampleMapTrafficActorPose(leader, leader.distanceMeters);
        require(frontPose && leaderPose &&
                    mapTrafficHasClearance({*leaderPose, leader.lengthMeters, leader.widthMeters},
                                           {{*frontPose, front.lengthMeters, front.widthMeters}},
                                           0.0),
                "a close follower does not weaken clearance from the vehicle ahead");
    }
    require(leader.distanceMeters < 62.0 && leader.speedMetersPerSecond < 0.1 &&
                leader.blockingActor == &front,
            "a leader still stops for traffic ahead when it has a close follower");
}

void verifyOppositeLaneTraffic() {
    using namespace openbus::map;
    for (const bool offsetCenterline : {false, true}) {
        for (const double rearAxle : {0.0, -1.35}) {
            for (const bool reverseOrder : {false, true}) {
                auto north = makeMapTrafficActor(
                    makeOppositeLaneRoute(false, offsetCenterline ? 0.0 : 1.1),
                    "opposite-north.ovh", 4.0, 0.0, false);
                auto south = makeMapTrafficActor(
                    makeOppositeLaneRoute(true, offsetCenterline ? 0.0 : -1.1),
                    "opposite-south.ovh", 0.0, 0.0, false);
                north.lateralOffsetMeters = south.lateralOffsetMeters =
                    offsetCenterline ? 1.3 : 0.0;
                north.rearAxleOffsetMeters = south.rearAxleOffsetMeters = rearAxle;
                north.widthMeters = 1.6;
                south.widthMeters = 2.0;
                north.distanceMeters = 30.0;
                north.speedMetersPerSecond = 4.0;
                south.distanceMeters = 65.0;
                const std::vector<MapTrafficActor*> actors =
                    reverseOrder ? std::vector<MapTrafficActor*>{&south, &north}
                                 : std::vector<MapTrafficActor*>{&north, &south};
                double lowestSpeed = north.speedMetersPerSecond;
                for (int step = 0; step < 80; ++step) {
                    advanceMapTrafficDriving(actors, {}, 0.05, step * 0.05);
                    lowestSpeed = std::min(lowestSpeed, north.speedMetersPerSecond);
                    const auto northPose = sampleMapTrafficActorPose(north, north.distanceMeters);
                    const auto southPose = sampleMapTrafficActorPose(south, south.distanceMeters);
                    require(northPose && southPose &&
                                mapTrafficHasClearance(
                                    {*northPose, north.lengthMeters, north.widthMeters},
                                    {{*southPose, south.lengthMeters, south.widthMeters}}, 0.0),
                            "opposite-lane vehicles maintain physical clearance");
                }
                require(lowestSpeed > 3.9 && north.distanceMeters > 45.0,
                        "straight traffic does not yield to a stationary vehicle in a clear opposite lane");
            }
        }
        auto north = makeMapTrafficActor(makeOppositeLaneRoute(false, 0.0),
                                         "narrow-north.ovh", 4.0, 0.0, false);
        auto south = makeMapTrafficActor(makeOppositeLaneRoute(true, 0.0),
                                         "narrow-south.ovh", 0.0, 0.0, false);
        north.lateralOffsetMeters = south.lateralOffsetMeters = 0.8;
        north.distanceMeters = 30.0;
        north.speedMetersPerSecond = 4.0;
        south.distanceMeters = 65.0;
        for (int step = 0; step < 100; ++step) {
            advanceMapTrafficDriving({&north, &south}, {}, 0.05, step * 0.05);
            const auto northPose = sampleMapTrafficActorPose(north, north.distanceMeters);
            const auto southPose = sampleMapTrafficActorPose(south, south.distanceMeters);
            require(northPose && southPose &&
                        mapTrafficHasClearance({*northPose, north.lengthMeters, north.widthMeters},
                                               {{*southPose, south.lengthMeters, south.widthMeters}},
                                               0.0),
                    "overlapping opposing lanes retain physical clearance");
        }
        require(north.distanceMeters < 40.0 && north.speedMetersPerSecond < 0.05,
                "overlapping opposing lanes remain exclusive");
    }
}

void verifyLowSpeedFollowingStop() {
    using namespace openbus::map;
    MapTrafficRouteSegment segment;
    segment.spline.placement = makeSpline(99200, 0, 0, 0.0);
    segment.spline.placement.localX = 0.0;
    segment.spline.placement.elevation = 0.0;
    segment.spline.placement.length = 100.0;
    MapTrafficRoute route;
    route.segments = {segment};
    route.cumulativeEndDistances = {100.0};
    route.totalLength = 100.0;
    auto leader = makeMapTrafficActor(route, "stopped-leader.ovh", 0.0, 0.0, false);
    auto follower = makeMapTrafficActor(route, "slow-follower.ovh", 8.0, 0.0, false);
    leader.distanceMeters = 50.0;
    follower.distanceMeters = 50.0 - 4.5 - 2.7;
    follower.speedMetersPerSecond = 0.5;
    for (int step = 0; step < 60; ++step) {
        const double previousSpeed = follower.speedMetersPerSecond;
        advanceMapTrafficDriving({&follower, &leader}, {}, 0.05, step * 0.05);
        const double gap = leader.distanceMeters - follower.distanceMeters -
                           (leader.lengthMeters + follower.lengthMeters) * 0.5;
        require(gap >= 2.0, "low-speed following maintains the standstill gap");
        require(follower.speedMetersPerSecond + 0.15 + 1.0e-8 >= previousSpeed,
                "low-speed settling respects maximum deceleration");
    }
    require(follower.speedMetersPerSecond < 0.05,
            "a vehicle below 2 km/h settles promptly behind a stopped leader");
    leader.cruiseSpeedMetersPerSecond = 0.5;
    leader.speedMetersPerSecond = 0.5;
    follower.distanceMeters = leader.distanceMeters - 4.5 - 2.7;
    follower.speedMetersPerSecond = 0.5;
    const double movingFollowerStart = follower.distanceMeters;
    for (int step = 0; step < 60; ++step) {
        advanceMapTrafficDriving({&follower, &leader}, {}, 0.05, 3.0 + step * 0.05);
    }
    require(follower.distanceMeters > movingFollowerStart + 0.9 &&
                follower.speedMetersPerSecond > 0.3,
            "a vehicle continues following a slowly moving leader");

    auto approaching = makeMapTrafficActor(makeOppositeLaneRoute(false, 0.0),
                                            "slow-junction.ovh", 0.5, 0.0, false);
    auto opposing = makeMapTrafficActor(makeOppositeLaneRoute(true, 0.0),
                                         "junction-blocker.ovh", 0.0, 0.0, false);
    approaching.distanceMeters = 36.0;
    approaching.speedMetersPerSecond = 0.5;
    opposing.distanceMeters = 65.0;
    for (int step = 0; step < 60; ++step) {
        const double previousSpeed = approaching.speedMetersPerSecond;
        advanceMapTrafficDriving({&approaching, &opposing}, {}, 0.05, step * 0.05);
        require(approaching.distanceMeters <= 37.25 + 1.0e-8,
                "low-speed junction traffic stops before the shared corridor");
        require(approaching.speedMetersPerSecond + 0.15 + 1.0e-8 >= previousSpeed,
                "low-speed junction settling respects maximum deceleration");
    }
    if (approaching.speedMetersPerSecond >= 0.05) {
        throw std::runtime_error(
            "a vehicle below 2 km/h takes too long to settle at a junction: speed=" +
            std::to_string(approaching.speedMetersPerSecond) + " distance=" +
            std::to_string(approaching.distanceMeters));
    }
    const double stoppedDistance = approaching.distanceMeters;
    for (int step = 0; step < 60; ++step) {
        advanceMapTrafficDriving({&approaching}, {}, 0.05, 3.0 + step * 0.05);
    }
    require(approaching.distanceMeters > stoppedDistance + 1.0 &&
                approaching.speedMetersPerSecond > 0.4,
            "a vehicle resumes after its junction blocker clears");
}

// Crossing traffic on a 3x3 junction grid, so arbitration, admission, and reservations run
// together.
std::vector<openbus::map::MapTrafficActor> makeJunctionGridActors() {
    using namespace openbus::map;
    constexpr double pi = std::numbers::pi;
    constexpr double junctionHalfLength = 6.0;
    constexpr double laneOffset = 1.75;
    constexpr std::array<double, 3> crossings = {0.0, 80.0, 160.0};
    int nextSplineId = 50000;
    const auto laneRoute = [&](double startX, double startY, double headingDegrees) {
        const double headingRadians = headingDegrees * pi / 180.0;
        const double directionX = std::sin(headingRadians);
        const double directionY = std::cos(headingRadians);
        MapTrafficRoute route;
        const auto addSpline = [&](double from, double to) {
            for (double pieceStart = from; pieceStart < to - 1.0e-9;) {
                const double pieceEnd = std::min(to, pieceStart + 17.0);
                MapTrafficRouteSegment segment;
                segment.spline.placement = makeSpline(nextSplineId++, 0, 0, 0.0);
                segment.spline.placement.localX = startX + directionX * pieceStart;
                segment.spline.placement.localY = startY + directionY * pieceStart;
                segment.spline.placement.elevation = 0.0;
                segment.spline.placement.rotationDegrees = headingDegrees;
                segment.spline.placement.length = pieceEnd - pieceStart;
                route.segments.push_back(segment);
                route.totalLength += pieceEnd - pieceStart;
                route.cumulativeEndDistances.push_back(route.totalLength);
                pieceStart = pieceEnd;
            }
        };
        double along = 0.0;
        for (const double crossing : {40.0, 120.0, 200.0}) {
            addSpline(along, crossing - junctionHalfLength);
            MapTrafficRouteSegment junction;
            junction.spline.placement = makeSpline(nextSplineId++, 0, 0, 0.0);
            junction.spline.placement.length = junctionHalfLength * 2.0;
            const double from = crossing - junctionHalfLength;
            const double to = crossing + junctionHalfLength;
            junction.points = {{startX + directionX * from, startY + directionY * from, 0.0},
                               {startX + directionX * to, startY + directionY * to, 0.0}};
            route.segments.push_back(junction);
            route.totalLength += junctionHalfLength * 2.0;
            route.cumulativeEndDistances.push_back(route.totalLength);
            along = to;
        }
        addSpline(along, 240.0);
        return route;
    };
    std::vector<MapTrafficRoute> lanes;
    for (const double crossing : crossings) {
        lanes.push_back(laneRoute(-40.0, crossing - laneOffset, 90.0));
        lanes.push_back(laneRoute(200.0, crossing + laneOffset, 270.0));
        lanes.push_back(laneRoute(crossing + laneOffset, -40.0, 0.0));
        lanes.push_back(laneRoute(crossing - laneOffset, 200.0, 180.0));
    }
    std::vector<MapTrafficActor> actors;
    actors.reserve(lanes.size() * 6);
    for (std::size_t lane = 0; lane < lanes.size(); ++lane) {
        for (int slot = 0; slot < 6; ++slot) {
            MapTrafficActor actor;
            actor.modelPath = "grid.ovh";
            actor.route = lanes[lane];
            actor.distanceMeters = 5.0 + slot * 18.0 + static_cast<double>(lane % 4) * 3.0;
            actor.speedMetersPerSecond = 6.0;
            actor.cruiseSpeedMetersPerSecond = 10.0 + static_cast<double>((lane + slot) % 3);
            actors.push_back(std::move(actor));
        }
    }
    return actors;
}

void verifyJunctionSubstepEquivalence() {
    using namespace openbus::map;
    auto combinedActors = makeJunctionGridActors();
    auto separateActors = combinedActors;
    std::vector<MapTrafficActor*> combined;
    std::vector<MapTrafficActor*> separate;
    for (std::size_t index = 0; index < combinedActors.size(); ++index) {
        combined.push_back(&combinedActors[index]);
        separate.push_back(&separateActors[index]);
    }
    const auto blockerIndex = [](const std::vector<MapTrafficActor>& actors,
                                 const MapTrafficActor* blocker) {
        for (std::size_t index = 0; index < actors.size(); ++index) {
            if (&actors[index] == blocker) {
                return index;
            }
        }
        return actors.size();
    };
    for (int update = 0; update < 8; ++update) {
        const double startTime = update * 0.2;
        advanceMapTrafficDriving(combined, {}, 0.2, startTime);
        for (int step = 0; step < 4; ++step) {
            advanceMapTrafficDriving(separate, {}, 0.05, startTime + step * 0.05);
        }
        for (std::size_t index = 0; index < combinedActors.size(); ++index) {
            const auto& actual = combinedActors[index];
            const auto& expected = separateActors[index];
            require(std::abs(actual.distanceMeters - expected.distanceMeters) < 1.0e-8 &&
                        std::abs(actual.speedMetersPerSecond - expected.speedMetersPerSecond) <
                            1.0e-8 &&
                        std::abs(actual.targetSpeedMetersPerSecond -
                                 expected.targetSpeedMetersPerSecond) < 1.0e-8 &&
                        actual.blockReason == expected.blockReason &&
                        blockerIndex(combinedActors, actual.blockingActor) ==
                            blockerIndex(separateActors, expected.blockingActor) &&
                        actual.junctionClaimEndMeters.has_value() ==
                            expected.junctionClaimEndMeters.has_value() &&
                        (!actual.junctionClaimEndMeters ||
                         std::abs(*actual.junctionClaimEndMeters -
                                  *expected.junctionClaimEndMeters) < 1.0e-8),
                    "combined junction substeps preserve movement, priorities, and claims");
        }
    }
}

void benchmarkDrivingJunctionGrid(bool multiSubstep = false) {
    using namespace openbus::map;
    std::vector<MapTrafficActor> actors = makeJunctionGridActors();
    // Covers every actor's state after every update for before/after checks.
    std::uint64_t stateHash = 1469598103934665603ULL;
    const auto mix = [&](std::uint64_t value) {
        stateHash ^= value;
        stateHash *= 1099511628211ULL;
    };
    const auto mixDouble = [&](double value) {
        std::uint64_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        mix(bits);
    };
    const int updates = multiSubstep ? 80 : 600;
    const double stepSeconds = multiSubstep ? 0.2 : 1.0 / 60.0;
    double movingMilliseconds = 0.0;
    for (int update = 0; update < updates; ++update) {
        std::vector<MapTrafficActor*> pointers;
        for (auto& actor : actors) {
            if (actor.distanceMeters < actor.route.totalLength) {
                pointers.push_back(&actor);
            } else {
                actor.speedMetersPerSecond = 0.0;
            }
        }
        const auto start = std::chrono::steady_clock::now();
        advanceMapTrafficDriving(pointers, {}, stepSeconds, update * stepSeconds);
        movingMilliseconds +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                .count();
        for (const auto& actor : actors) {
            mixDouble(actor.distanceMeters);
            mixDouble(actor.speedMetersPerSecond);
            mixDouble(actor.junctionWaitSeconds);
            mix(static_cast<std::uint64_t>(actor.blockReason));
            mixDouble(actor.junctionClaimEndMeters.value_or(-1.0));
        }
    }
    double distanceSum = 0.0;
    std::size_t finished = 0;
    for (const auto& actor : actors) {
        distanceSum += actor.distanceMeters;
        finished += actor.distanceMeters >= actor.route.totalLength ? 1U : 0U;
    }
    std::cout << std::setprecision(17) << "TRAFFIC_JUNCTION_BENCHMARK actors=" << actors.size()
              << " updates=" << updates << " ms_per_update=" << movingMilliseconds / updates
              << " step_seconds=" << stepSeconds
              << " finished=" << finished << " distance_sum=" << distanceSum
              << " state_hash=" << std::hex << stateHash << std::dec << '\n';
}

void verifyAutomaticRoundaboutPriority() {
    using namespace openbus::map;
    MapTrafficRouteSegment ring;
    ring.spline.placement.splineId = 9500;
    ring.points.reserve(65);
    constexpr double radius = 12.0;
    for (int index = 0; index <= 64; ++index) {
        const double angle = 2.0 * std::numbers::pi * index / 64.0;
        ring.points.push_back({radius * std::cos(angle), radius * std::sin(angle), 0.0});
    }
    MapTrafficGroupRules lowGroupPriority;
    lowGroupPriority.groupIndex = 0;
    lowGroupPriority.priority = 0.0;
    ring.pathRules.groups.push_back(lowGroupPriority);

    MapTrafficLaneNetwork network;
    network.addSegment(ring);
    network.finalize();
    const auto route = network.buildRoute(9500);
    require(route.has_value() && route->closed && route->segments.size() == 1 &&
                route->segments.front().pathRules.roundabout &&
                mapTrafficRoutePositionOnJunctionPath(*route, 10.0),
            "a closed circular lane is automatically detected as a roundabout");
    MapTrafficVehicleFilter vehicle;
    vehicle.trafficRuleGroupIndex = 0;
    require(mapTrafficPathPriorityForVehicle(route->segments.front().pathRules, vehicle) == 256.0,
            "roundabout priority overrides lower vehicle-group priorities");

    MapTrafficRouteSegment square;
    square.spline.placement.splineId = 9501;
    square.points = {
        {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {10.0, 10.0, 0.0}, {0.0, 10.0, 0.0}, {0.0, 0.0, 0.0}};
    MapTrafficLaneNetwork nonRoundaboutNetwork;
    nonRoundaboutNetwork.addSegment(square);
    nonRoundaboutNetwork.finalize();
    const auto squareRoute = nonRoundaboutNetwork.buildRoute(9501);
    require(squareRoute.has_value() && squareRoute->closed &&
                !squareRoute->segments.front().pathRules.roundabout,
            "a closed square loop is not misclassified as a roundabout");
}

// Deferred/skipped sampling queries are only allowed on routes that cannot fail to sample, so
// malformed routes must still raise the same exception, from the same update, as eager evaluation.
void verifyMalformedRouteErrors() {
    using namespace openbus::map;
    const auto drivingError = [](const std::vector<MapTrafficActor*>& actors, int updates,
                                 double elapsedSeconds) -> std::pair<int, std::string> {
        for (int update = 0; update < updates; ++update) {
            std::vector<MapTrafficActor*> active;
            for (MapTrafficActor* actor : actors) {
                if (actor->distanceMeters < actor->route.totalLength) {
                    active.push_back(actor);
                }
            }
            try {
                advanceMapTrafficDriving(active, {}, elapsedSeconds, update * elapsedSeconds);
            } catch (const std::exception& error) {
                return {update, error.what()};
            }
        }
        return {updates, ""};
    };
    for (const double startMeters : {4.5, 10.0}) {
        MapTrafficRouteSegment valid{{makeSpline(9001, 0, 9002, 0.0), 0, 0}};
        valid.spline.placement.localX = 0.0;
        valid.spline.placement.length = 20.0;
        MapTrafficRouteSegment malformed{{makeSpline(9002, 9001, 0, 0.0), 0, 0}};
        malformed.spline.placement.localX = 20.0;
        malformed.spline.placement.length = 20.0;
        malformed.spline.placement.geometryValid = false;
        MapTrafficRoute route;
        route.segments = {valid, malformed};
        route.cumulativeEndDistances = {20.0, 40.0};
        route.totalLength = 40.0;
        // Slow enough that the lookahead curve query could be skipped on a well-formed route.
        MapTrafficActor actor = makeMapTrafficActor(route, "malformed.ovh", 1.0, 0.0, false);
        actor.distanceMeters = startMeters;
        actor.speedMetersPerSecond = 1.0;
        const auto [update, message] = drivingError({&actor}, 1, 0.05);
        require(update == 0 && message.find("Cannot sample traffic curve") == 0 &&
                    message.find("distanceMeters=") != std::string::npos &&
                    message.find("fromDistance=") != std::string::npos &&
                    message.find("toSampled=0") != std::string::npos,
                "slow traffic on a malformed route reports the curve sampling failure");
    }
    // Lane 0 segment 8 exercises the lookahead curve query; lane 2 segment 8 exercises the
    // junction arrival-window queries (deferring them moves the failure to a later update).
    for (const auto& [lane, segment, expectedUpdate] :
         {std::tuple<std::size_t, std::size_t, int>{0, 8, 0}, {2, 8, 9}}) {
        std::vector<MapTrafficActor> actors = makeJunctionGridActors();
        for (std::size_t slot = 0; slot < 6; ++slot) {
            actors[lane * 6 + slot].route.segments[segment].spline.placement.geometryValid = false;
        }
        std::vector<MapTrafficActor*> pointers;
        for (MapTrafficActor& actor : actors) {
            pointers.push_back(&actor);
        }
        const auto [update, message] = drivingError(pointers, 60, 1.0 / 60.0);
        require(update == expectedUpdate && message.find("Cannot sample traffic curve") == 0 &&
                    message.find("distanceMeters=") != std::string::npos &&
                    message.find("toDistance=") != std::string::npos &&
                    message.find("toSampled=0") != std::string::npos,
                "malformed junction traffic fails at the same update with the same error");
    }
}

void verifyRealisticTrafficBehavior() {
    using namespace openbus::map;
    constexpr double frameSeconds = 1.0 / 60.0;
    constexpr double carLengthMeters = 4.5;

    MapTrafficRoute laneRoute;
    for (int piece = 0; piece < 200; ++piece) {
        MapTrafficRouteSegment segment{{makeSpline(70000 + piece, 0, 0, piece * 10.0), 0, 0}};
        segment.spline.placement.localX = 0.0;
        segment.spline.placement.elevation = 0.0;
        laneRoute.segments.push_back(segment);
        laneRoute.totalLength += 10.0;
        laneRoute.cumulativeEndDistances.push_back(laneRoute.totalLength);
    }
    struct QueueResult {
        std::vector<double> cruiseGaps;
        std::vector<double> stoppedGaps;
        std::vector<double> stoppedSpeeds;
        double minimumGap = 1.0e9;
    };
    // A leader cruises at leaderCruise with five cars behind it, then stops.
    const auto runQueue = [&](double leaderCruise) {
        std::vector<MapTrafficActor> actors;
        for (int car = 0; car < 6; ++car) {
            MapTrafficActor actor;
            actor.modelPath = "queue.ovh";
            actor.route = laneRoute;
            actor.distanceMeters = 150.0 - car * 25.0;
            actor.cruiseSpeedMetersPerSecond = car == 0 ? leaderCruise : 13.0;
            actor.speedMetersPerSecond = actor.cruiseSpeedMetersPerSecond;
            actors.push_back(actor);
        }
        std::vector<MapTrafficActor*> pointers;
        for (MapTrafficActor& actor : actors) {
            pointers.push_back(&actor);
        }
        const auto gaps = [&] {
            std::vector<double> result;
            for (std::size_t car = 1; car < actors.size(); ++car) {
                result.push_back(actors[car - 1].distanceMeters - actors[car].distanceMeters -
                                 carLengthMeters);
            }
            return result;
        };
        QueueResult result;
        for (int update = 0; update < 60 * 70; ++update) {
            if (update == 60 * 25) {
                result.cruiseGaps = gaps();
                actors[0].cruiseSpeedMetersPerSecond = 0.0;
            }
            advanceMapTrafficDriving(pointers, {}, frameSeconds, update * frameSeconds);
            if (update > 60 * 5) {
                for (const double gap : gaps()) {
                    result.minimumGap = std::min(result.minimumGap, gap);
                }
            }
        }
        result.stoppedGaps = gaps();
        for (std::size_t car = 1; car < actors.size(); ++car) {
            result.stoppedSpeeds.push_back(actors[car].speedMetersPerSecond);
        }
        return result;
    };

    const QueueResult crawling = runQueue(2.0);
    require(crawling.cruiseGaps.front() > 2.8 && crawling.cruiseGaps.front() < 4.0,
            "traffic following a 2 m/s leader closes up to a short urban gap");
    const QueueResult fast = runQueue(13.0);
    require(fast.cruiseGaps.front() > 18.0 && fast.cruiseGaps.front() < 23.0,
            "traffic following a 13 m/s leader keeps a time-headway gap");
    require(fast.minimumGap > 2.0, "a braking queue never closes inside the standstill gap");
    for (std::size_t car = 0; car < fast.stoppedGaps.size(); ++car) {
        require(fast.stoppedGaps[car] > 2.0 && fast.stoppedGaps[car] < 2.4 &&
                    fast.stoppedSpeeds[car] < 0.01,
                "a stopped queue settles bumper to bumper at the standstill gap");
    }

    // Equal authored priority: a car waiting at the line to turn left across oncoming traffic
    // must let a 12 m/s straight-through car pass instead of forcing it to brake.
    int nextId = 80000;
    const auto spline = [&](double x, double y, double heading, double length) {
        MapTrafficRouteSegment segment{{makeSpline(nextId++, 0, 0, y), 0, 0}};
        segment.spline.placement.localX = x;
        segment.spline.placement.elevation = 0.0;
        segment.spline.placement.rotationDegrees = heading;
        segment.spline.placement.length = length;
        return segment;
    };
    const auto routeOf = [](std::vector<MapTrafficRouteSegment> segments,
                            const std::vector<double>& lengths) {
        MapTrafficRoute route;
        route.segments = std::move(segments);
        for (const double length : lengths) {
            route.totalLength += length;
            route.cumulativeEndDistances.push_back(route.totalLength);
        }
        return route;
    };
    MapTrafficRouteSegment straightJunction;
    straightJunction.points = {{1.75, -6.0, 0.0}, {1.75, 6.0, 0.0}};
    const MapTrafficRoute straightRoute =
        routeOf({spline(1.75, -60.0, 0.0, 54.0), straightJunction, spline(1.75, 6.0, 0.0, 54.0)},
                {54.0, 12.0, 54.0});
    MapTrafficRouteSegment turnJunction;
    constexpr double turnRadius = 7.75;
    for (int step = 0; step <= 12; ++step) {
        const double angle = std::numbers::pi * (1.0 + 0.5 * step / 12.0);
        turnJunction.points.push_back(
            {6.0 + turnRadius * std::cos(angle), 6.0 + turnRadius * std::sin(angle), 0.0});
    }
    const MapTrafficRoute turnRoute =
        routeOf({spline(-1.75, 60.0, 180.0, 54.0), turnJunction, spline(6.0, -1.75, 90.0, 54.0)},
                {54.0, turnRadius * std::numbers::pi * 0.5, 54.0});
    for (const auto& [straightStart, turnerStart, turnerSpeed] :
         {std::tuple<double, double, double>{4.0, 51.0, 0.0},
          {14.0, 51.0, 0.0},
          {24.0, 51.0, 0.0},
          {15.0, 35.0, 9.0},
          {20.0, 38.0, 9.0}}) {
        MapTrafficActor straight;
        straight.modelPath = "straight.ovh";
        straight.route = straightRoute;
        straight.distanceMeters = straightStart;
        straight.speedMetersPerSecond = 12.0;
        straight.cruiseSpeedMetersPerSecond = 12.0;
        MapTrafficActor turner;
        turner.modelPath = "turner.ovh";
        turner.route = turnRoute;
        turner.distanceMeters = turnerStart;
        turner.speedMetersPerSecond = turnerSpeed;
        turner.cruiseSpeedMetersPerSecond = 10.0;
        double straightEnter = -1.0;
        double turnerEnter = -1.0;
        double straightMinimumSpeed = 1.0e9;
        for (int update = 0; update < 60 * 12; ++update) {
            advanceMapTrafficDriving({&straight, &turner}, {}, frameSeconds, update * frameSeconds);
            straightMinimumSpeed = std::min(straightMinimumSpeed, straight.speedMetersPerSecond);
            if (straightEnter < 0.0 && straight.distanceMeters > 54.0) {
                straightEnter = update * frameSeconds;
            }
            if (turnerEnter < 0.0 && turner.distanceMeters > 54.0) {
                turnerEnter = update * frameSeconds;
            }
        }
        require(straightMinimumSpeed > 11.9,
                "straight-through traffic keeps its speed past a car turning across it");
        require(straightEnter >= 0.0 && turnerEnter > straightEnter,
                "a turning car waits for oncoming straight-through traffic before turning");
    }

    MapTrafficRouteSegment rightTurnJunction;
    for (int step = 0; step <= 12; ++step) {
        const double angle =
            std::numbers::pi - std::numbers::pi * 0.5 * static_cast<double>(step) / 12.0;
        rightTurnJunction.points.push_back(
            {9.5 + turnRadius * std::cos(angle), 6.0 + turnRadius * std::sin(angle), 0.0});
    }
    const MapTrafficRoute rightTurnRoute =
        routeOf({spline(1.75, -60.0, 0.0, 66.0), rightTurnJunction, spline(9.5, 13.75, 90.0, 60.0)},
                {66.0, turnRadius * std::numbers::pi * 0.5, 60.0});
    MapTrafficRouteSegment crossingJunction;
    crossingJunction.points = {{-6.0, 10.0, 0.0}, {6.0, 10.0, 0.0}};
    const MapTrafficRoute crossingRoute =
        routeOf({spline(-60.0, 10.0, 90.0, 54.0), crossingJunction, spline(6.0, 10.0, 90.0, 60.0)},
                {54.0, 12.0, 60.0});
    MapTrafficActor rightTurner;
    rightTurner.modelPath = "right-turn.ovh";
    rightTurner.route = rightTurnRoute;
    rightTurner.distanceMeters = 55.0;
    rightTurner.speedMetersPerSecond = 10.0;
    rightTurner.cruiseSpeedMetersPerSecond = 10.0;
    MapTrafficActor crossing;
    crossing.modelPath = "crossing.ovh";
    crossing.route = crossingRoute;
    crossing.distanceMeters = 60.0;
    crossing.speedMetersPerSecond = 8.0;
    crossing.cruiseSpeedMetersPerSecond = 8.0;
    double minimumRightTurnSpeed = std::numeric_limits<double>::infinity();
    bool yieldedToPassingTraffic = false;
    double maximumTurnerDistanceWhileYielding = 0.0;
    int yieldStateChanges = 0;
    bool previousYieldState = false;
    int crossingClearUpdate = -1;
    int rightTurnEntryUpdate = -1;
    int rightTurnExitUpdate = -1;
    double recoveredRightTurnSpeed = 0.0;
    const double rightTurnStopLine = 66.0 - rightTurner.lengthMeters * 0.5 - 0.5;
    constexpr double crossingConflictClearDistance = 68.0;
    constexpr double rightTurnEnd = 66.0 + turnRadius * std::numbers::pi * 0.5;
    for (int update = 0; update < 60 * 20; ++update) {
        advanceMapTrafficDriving({&rightTurner, &crossing}, {}, frameSeconds,
                                 update * frameSeconds);
        if (rightTurner.blockReason == MapTrafficBlockReason::JunctionTraffic) {
            yieldedToPassingTraffic = true;
        }
        const bool currentlyYielding =
            rightTurner.blockReason == MapTrafficBlockReason::JunctionTraffic;
        if (currentlyYielding) {
            maximumTurnerDistanceWhileYielding =
                std::max(maximumTurnerDistanceWhileYielding, rightTurner.distanceMeters);
        }
        if (currentlyYielding != previousYieldState) {
            ++yieldStateChanges;
        }
        previousYieldState = currentlyYielding;
        if (rightTurnEntryUpdate < 0 && rightTurner.distanceMeters >= 66.0) {
            rightTurnEntryUpdate = update;
        }
        if (rightTurner.distanceMeters >= 66.0 && rightTurner.distanceMeters <= rightTurnEnd) {
            minimumRightTurnSpeed =
                std::min(minimumRightTurnSpeed, rightTurner.speedMetersPerSecond);
        }
        if (crossingClearUpdate < 0 && crossing.distanceMeters > crossingConflictClearDistance) {
            crossingClearUpdate = update;
        }
        if (rightTurnExitUpdate < 0 && rightTurner.distanceMeters > rightTurnEnd) {
            rightTurnExitUpdate = update;
        } else if (rightTurnExitUpdate >= 0 && update == rightTurnExitUpdate + 180) {
            recoveredRightTurnSpeed = rightTurner.speedMetersPerSecond;
            break;
        }
    }
    require(yieldedToPassingTraffic && crossingClearUpdate >= 0,
            "a right turn yields to a passing vehicle and proceeds after it clears");
    require(maximumTurnerDistanceWhileYielding <= rightTurnStopLine - 3.5 &&
                rightTurnEntryUpdate > crossingClearUpdate,
            "a yielding right turn stays clear of the crossing path until passing traffic clears");
    require(yieldStateChanges <= 2,
            "a yielding vehicle does not repeatedly toggle between blocked and moving");
    require(minimumRightTurnSpeed > 3.5,
            "a right turn does not crawl through the junction after yielding");
    require(rightTurnExitUpdate >= 0 && recoveredRightTurnSpeed > 9.4,
            "a right-turning vehicle accelerates back to cruise speed after the junction clears");
}

void verifyAdjacentStraightAndRightTurn() {
    using namespace openbus::map;
    const auto spline = [](int id, double x, double y, double heading, double length) {
        MapTrafficRouteSegment segment{{makeSpline(id, 0, 0, y), 0, 0}};
        segment.spline.placement.localX = x;
        segment.spline.placement.elevation = 0.0;
        segment.spline.placement.rotationDegrees = heading;
        segment.spline.placement.length = length;
        return segment;
    };
    const auto route = [](std::vector<MapTrafficRouteSegment> segments,
                          const std::vector<double>& lengths) {
        MapTrafficRoute result;
        result.segments = std::move(segments);
        for (const double length : lengths) {
            result.totalLength += length;
            result.cumulativeEndDistances.push_back(result.totalLength);
        }
        return result;
    };
    MapTrafficRouteSegment throughJunction;
    throughJunction.points = {{3.4, 0.0, 0.0}, {3.4, 20.0, 0.0}};
    throughJunction.trafficSignal = MapTrafficSignalSchedule{120.0, {{6, 120.0}}};
    const auto throughRoute =
        route({spline(3100, 3.4, -40.0, 0.0, 40.0), throughJunction,
               spline(3101, 3.4, 20.0, 0.0, 60.0)},
              {40.0, 20.0, 60.0});
    MapTrafficRouteSegment turningJunction;
    constexpr double radius = 8.0;
    constexpr int steps = 32;
    for (int step = 0; step <= steps; ++step) {
        const double angle = std::numbers::pi -
                             std::numbers::pi * 0.5 * static_cast<double>(step) / steps;
        turningJunction.points.push_back(
            {radius + radius * std::cos(angle), radius * std::sin(angle), 0.0});
    }
    turningJunction.trafficSignal = MapTrafficSignalSchedule{120.0, {{6, 120.0}}};
    double turnLength = 0.0;
    for (std::size_t index = 1; index < turningJunction.points.size(); ++index) {
        const auto& first = turningJunction.points[index - 1];
        const auto& second = turningJunction.points[index];
        turnLength += std::hypot(second[0] - first[0], second[1] - first[1]);
    }
    const auto turnRoute =
        route({spline(3102, 0.0, -40.0, 0.0, 40.0), turningJunction,
               spline(3103, radius, radius, 90.0, 60.0)},
              {40.0, turnLength, 60.0});
    for (const auto& [startDistance, reverseOrder, redFirst, turnLengthMeters] :
         {std::tuple{35.0, false, false, 4.5}, std::tuple{35.0, true, false, 4.5},
          std::tuple{38.0, false, false, 4.5}, std::tuple{38.0, true, false, 4.5},
          std::tuple{39.0, false, false, 4.5}, std::tuple{39.0, true, false, 4.5},
          std::tuple{35.0, false, true, 4.5}, std::tuple{35.0, true, true, 4.5},
          std::tuple{38.0, false, true, 4.5}, std::tuple{38.0, true, true, 4.5},
          std::tuple{35.0, false, false, 12.0}, std::tuple{35.0, true, false, 12.0},
          std::tuple{35.0, false, true, 12.0}, std::tuple{35.0, true, true, 12.0},
          std::tuple{35.0, false, false, 18.0}, std::tuple{35.0, true, false, 18.0},
          std::tuple{35.0, false, true, 18.0}, std::tuple{35.0, true, true, 18.0}}) {
        auto through = makeMapTrafficActor(throughRoute, "adjacent-through.ovh", 8.0, 0.0, false);
        auto turn = makeMapTrafficActor(turnRoute, "adjacent-turn.ovh", 8.0, 0.0, false);
        turn.lengthMeters = turnLengthMeters;
        if (turnLengthMeters > 4.5) {
            turn.rearAxleOffsetMeters = -3.0;
        }
        if (redFirst) {
            const MapTrafficSignalSchedule redThenGreen{120.0, {{0, 5.0}, {6, 115.0}}};
            through.route.segments[1].trafficSignal = redThenGreen;
            turn.route.segments[1].trafficSignal = redThenGreen;
        }
        through.distanceMeters = startDistance;
        turn.distanceMeters = startDistance;
        through.speedMetersPerSecond = 0.0;
        turn.speedMetersPerSecond = 0.0;
        bool turnYielded = false;
        int throughYieldFrames = 0;
        double minimumThroughYieldSpeed = std::numeric_limits<double>::infinity();
        std::string throughYieldDetails;
        int throughClearedFrame = -1;
        int turnEnteredFrame = -1;
        for (int frame = 0; frame < 300; ++frame) {
            const std::vector<MapTrafficActor*> actors =
                reverseOrder ? std::vector<MapTrafficActor*>{&turn, &through}
                             : std::vector<MapTrafficActor*>{&through, &turn};
            advanceMapTrafficDriving(actors, {}, 0.1, frame * 0.1);
            const auto throughPose = sampleMapTrafficActorPose(through, through.distanceMeters);
            const auto turnPose = sampleMapTrafficActorPose(turn, turn.distanceMeters);
            require(throughPose && turnPose, "adjacent vehicles retain valid route poses");
            require(mapTrafficHasClearance({*throughPose, through.lengthMeters, through.widthMeters},
                                          {{*turnPose, turn.lengthMeters, turn.widthMeters}}, 0.0),
                    "adjacent through and turning vehicles do not collide");
            turnYielded |= (!redFirst || frame >= 50) && turn.blockingActor == &through &&
                           (turn.blockReason == MapTrafficBlockReason::JunctionTraffic ||
                            turn.blockReason == MapTrafficBlockReason::CrossingReservation);
            if ((!redFirst || frame >= 50) && through.blockingActor == &turn &&
                (through.blockReason == MapTrafficBlockReason::JunctionTraffic ||
                 through.blockReason == MapTrafficBlockReason::CrossingReservation)) {
                ++throughYieldFrames;
                minimumThroughYieldSpeed =
                    std::min(minimumThroughYieldSpeed, through.speedMetersPerSecond);
                throughYieldDetails += " frame=" + std::to_string(frame) +
                                       " through=" + std::to_string(through.distanceMeters) +
                                       " turn=" + std::to_string(turn.distanceMeters) +
                                       " reason=" + mapTrafficBlockReasonName(through.blockReason);
            }
            if ((!redFirst || frame >= 50) && throughClearedFrame < 0 &&
                through.distanceMeters > 50.0) {
                throughClearedFrame = frame;
            }
            if ((!redFirst || frame >= 50) && turnEnteredFrame < 0 && turn.distanceMeters > 40.0) {
                turnEnteredFrame = frame;
            }
        }
        if (!(throughClearedFrame >= 0 && throughClearedFrame < (redFirst ? 150 : 100) &&
              turnYielded &&
              (throughYieldFrames == 0 ||
               (turnLengthMeters > 4.5 && minimumThroughYieldSpeed > 1.0)) &&
              turnEnteredFrame > throughClearedFrame && turn.distanceMeters > 60.0)) {
            throw std::runtime_error(
                "adjacent straight/right-turn priority failed: start=" +
                std::to_string(startDistance) + ", reverseOrder=" + std::to_string(reverseOrder) +
                ", redFirst=" + std::to_string(redFirst) +
                ", turnLength=" + std::to_string(turnLengthMeters) +
                ", throughClearedFrame=" + std::to_string(throughClearedFrame) +
                ", turnEnteredFrame=" + std::to_string(turnEnteredFrame) +
                ", turnYielded=" + std::to_string(turnYielded) +
                ", throughYieldFrames=" + std::to_string(throughYieldFrames) +
                ", minimumThroughYieldSpeed=" + std::to_string(minimumThroughYieldSpeed) +
                ", throughYieldDetails=" + throughYieldDetails +
                ", throughDistance=" + std::to_string(through.distanceMeters) +
                ", turnDistance=" + std::to_string(turn.distanceMeters));
        }
    }
    for (const bool reverseOrder : {false, true}) {
        auto through = makeMapTrafficActor(throughRoute, "lower-priority-through.ovh", 8.0, 0.0,
                                           false);
        auto turn = makeMapTrafficActor(turnRoute, "higher-priority-turn.ovh", 8.0, 0.0, false);
        for (auto& segment : through.route.segments) {
            segment.pathRules.priority = 64.0;
        }
        for (auto& segment : turn.route.segments) {
            segment.pathRules.priority = 192.0;
        }
        turn.lengthMeters = 12.0;
        turn.rearAxleOffsetMeters = -3.0;
        through.distanceMeters = turn.distanceMeters = 35.0;
        bool throughYielded = false;
        for (int frame = 0; frame < 300; ++frame) {
            const std::vector<MapTrafficActor*> actors =
                reverseOrder ? std::vector<MapTrafficActor*>{&turn, &through}
                             : std::vector<MapTrafficActor*>{&through, &turn};
            advanceMapTrafficDriving(actors, {}, 0.1, frame * 0.1);
            const auto throughPose = sampleMapTrafficActorPose(through, through.distanceMeters);
            const auto turnPose = sampleMapTrafficActorPose(turn, turn.distanceMeters);
            require(throughPose && turnPose, "priority vehicles retain valid route poses");
            require(mapTrafficHasClearance({*throughPose, through.lengthMeters, through.widthMeters},
                                          {{*turnPose, turn.lengthMeters, turn.widthMeters}}, 0.0),
                    "authored-priority vehicles do not collide");
            throughYielded |= through.blockingActor == &turn &&
                              (through.blockReason == MapTrafficBlockReason::JunctionTraffic ||
                               through.blockReason == MapTrafficBlockReason::CrossingReservation);
        }
        require(throughYielded && through.distanceMeters > 60.0 && turn.distanceMeters > 60.0,
                "an adjacent long turn with higher authored priority goes before through traffic");
    }
}

void verifyCommittedLeftTurnPriority() {
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
    MapTrafficGroupRules committedRules;
    committedRules.groupIndex = 0;
    committedRules.priority = 192.0;
    MapTrafficGroupRules approachRules;
    approachRules.groupIndex = 0;
    approachRules.priority = 64.0;

    MapTrafficRoute committedLeftTurnRoute;
    MapTrafficRouteSegment committedLeftTurnPath;
    committedLeftTurnPath.points = {{0.0, 0.0, 0.0}, {0.0, 8.0, 0.0}, {-8.0, 8.0, 0.0}};
    committedLeftTurnPath.pathRules.groups.push_back(committedRules);
    committedLeftTurnRoute.segments = {committedLeftTurnPath,
                                       straight(1208, -8.0, 8.0, 270.0, 40.0)};
    committedLeftTurnRoute.cumulativeEndDistances = {16.0, 56.0};
    committedLeftTurnRoute.totalLength = 56.0;
    MapTrafficRoute higherPriorityApproachRoute =
        oneSegment(straight(1209, -40.0, 4.0, 90.0, 80.0));
    higherPriorityApproachRoute.segments.front().pathRules.groups.push_back(approachRules);
    auto committedLeftTurn =
        makeMapTrafficActor(committedLeftTurnRoute, "committed-left-turn.ovh", 3.0, 0.0, false);
    auto higherPriorityCrossing = makeMapTrafficActor(
        higherPriorityApproachRoute, "higher-priority-crossing.ovh", 8.0, 0.0, false);
    committedLeftTurn.unscheduledGroupIndex = 0;
    committedLeftTurn.trafficRuleGroupIndex = 0;
    higherPriorityCrossing.unscheduledGroupIndex = 0;
    higherPriorityCrossing.trafficRuleGroupIndex = 0;
    higherPriorityCrossing.distanceMeters = 20.0;

    bool committedTurnPassedFirst = false;
    for (int frame = 0; frame < 100; ++frame) {
        advanceMapTrafficDriving({&committedLeftTurn, &higherPriorityCrossing}, {}, 0.1);
        require(mapTrafficHasClearance(footprint(committedLeftTurn),
                                       {footprint(higherPriorityCrossing)}, 0.49),
                "a committed left turn and an approaching vehicle remain separated");
        committedTurnPassedFirst =
            committedTurnPassedFirst || (committedLeftTurn.distanceMeters > 8.0 &&
                                         higherPriorityCrossing.distanceMeters < 40.0);
    }
    require(committedTurnPassedFirst,
            "a vehicle already turning through a junction keeps priority over a higher-priority "
            "approach");
}

void verifyTJunctionTraffic() {
    using namespace openbus::map;
    constexpr double laneOffset = 1.75;
    constexpr double frameSeconds = 0.1;
    constexpr int updates = 300;
    int nextSplineId = 90000;
    const auto spline = [&](double x, double y, double heading, double length) {
        MapTrafficRouteSegment segment{{makeSpline(nextSplineId++, 0, 0, y), 0, 0}};
        segment.spline.placement.localX = x;
        segment.spline.placement.elevation = 0.0;
        segment.spline.placement.rotationDegrees = heading;
        segment.spline.placement.length = length;
        return segment;
    };
    const auto routeOf = [](std::vector<MapTrafficRouteSegment> segments,
                            const std::vector<double>& lengths) {
        MapTrafficRoute route;
        route.segments = std::move(segments);
        for (const double length : lengths) {
            route.totalLength += length;
            route.cumulativeEndDistances.push_back(route.totalLength);
        }
        return route;
    };
    const auto throughRoute = [&](bool westbound) {
        const double startX = westbound ? 60.0 : -60.0;
        const double laneY = westbound ? laneOffset : -laneOffset;
        const double heading = westbound ? 270.0 : 90.0;
        MapTrafficRouteSegment junction;
        junction.points =
            westbound ? std::vector<std::array<double, 3>>{{6.0, laneY, 0.0}, {-6.0, laneY, 0.0}}
                      : std::vector<std::array<double, 3>>{{-6.0, laneY, 0.0}, {6.0, laneY, 0.0}};
        return routeOf({spline(startX, laneY, heading, 54.0), junction,
                        spline(westbound ? -6.0 : 6.0, laneY, heading, 100.0)},
                       {54.0, 12.0, 100.0});
    };
    const MapTrafficRoute eastboundRoute = throughRoute(false);
    const MapTrafficRoute westboundRoute = throughRoute(true);

    constexpr double turnRadius = 7.75;
    const double turnArcLength = turnRadius * std::numbers::pi * 0.5;
    MapTrafficRouteSegment turnJunction;
    for (int step = 0; step <= 12; ++step) {
        const double angle =
            -std::numbers::pi * 0.5 + std::numbers::pi * 0.5 * static_cast<double>(step) / 12.0;
        turnJunction.points.push_back(
            {-6.0 + turnRadius * std::cos(angle), 6.0 + turnRadius * std::sin(angle), 0.0});
    }
    const MapTrafficRoute turnRoute = routeOf({spline(-60.0, -laneOffset, 90.0, 54.0), turnJunction,
                                               spline(turnRadius - 6.0, 6.0, 0.0, 100.0)},
                                              {54.0, turnArcLength, 100.0});

    const auto makeActor = [](MapTrafficRoute route, const char* model, double distance,
                              double speed) {
        auto actor = makeMapTrafficActor(std::move(route), model, speed, 0.0, false);
        actor.distanceMeters = distance;
        actor.speedMetersPerSecond = speed;
        actor.cruiseSpeedMetersPerSecond = speed;
        return actor;
    };
    std::vector<MapTrafficActor> actors;
    actors.push_back(makeActor(turnRoute, "t-junction-turn.ovh", 30.0, 9.0));
    actors.push_back(makeActor(eastboundRoute, "t-junction-east.ovh", 0.0, 10.0));
    actors.push_back(makeActor(westboundRoute, "t-junction-west-a.ovh", 30.0, 8.0));
    actors.push_back(makeActor(westboundRoute, "t-junction-west-b.ovh", 5.0, 8.0));
    std::vector<MapTrafficActor*> actorPointers;
    for (MapTrafficActor& actor : actors) {
        actorPointers.push_back(&actor);
    }
    const auto footprint = [](const MapTrafficActor& actor) {
        const auto pose =
            sampleMapTrafficRoute(actor.route, actor.distanceMeters, actor.lateralOffsetMeters);
        require(pose.has_value(), "T-junction vehicles retain a valid route pose");
        return MapTrafficObstacle{*pose, actor.lengthMeters, actor.widthMeters};
    };
    std::vector<double> previousDistances;
    std::vector<double> lastProgressSeconds(actors.size(), 0.0);
    const std::vector<double> clearanceDistances = {54.0 + turnArcLength + 20.0, 100.0, 100.0,
                                                    100.0};
    bool turnerYieldedToThroughTraffic = false;
    for (const MapTrafficActor& actor : actors) {
        previousDistances.push_back(actor.distanceMeters);
    }
    for (int update = 0; update < updates; ++update) {
        const double elapsed = (update + 1) * frameSeconds;
        advanceMapTrafficDriving(actorPointers, {}, frameSeconds, update * frameSeconds);
        turnerYieldedToThroughTraffic =
            turnerYieldedToThroughTraffic ||
            actors[0].blockReason == MapTrafficBlockReason::JunctionTraffic;
        std::vector<MapTrafficObstacle> footprints;
        footprints.reserve(actors.size());
        for (const MapTrafficActor& actor : actors) {
            footprints.push_back(footprint(actor));
        }
        for (std::size_t first = 0; first < footprints.size(); ++first) {
            for (std::size_t second = first + 1; second < footprints.size(); ++second) {
                require(mapTrafficHasClearance(footprints[first], {footprints[second]}, 0.49),
                        "T-junction turns and through traffic preserve vehicle clearance");
            }
        }
        for (std::size_t index = 0; index < actors.size(); ++index) {
            const MapTrafficActor& actor = actors[index];
            if (actor.distanceMeters > previousDistances[index] + 0.01) {
                lastProgressSeconds[index] = elapsed;
            }
            previousDistances[index] = actor.distanceMeters;
            if (actor.distanceMeters < clearanceDistances[index]) {
                require(elapsed - lastProgressSeconds[index] < 8.0,
                        "every T-junction vehicle continues making progress without deadlock");
            }
        }
    }
    require(turnerYieldedToThroughTraffic,
            "the turning vehicle yields to conflicting major-road through traffic");
    require(actors[0].distanceMeters > clearanceDistances[0],
            "the turning vehicle clears the minor-road junction exit");
    require(actors[1].distanceMeters > clearanceDistances[1] &&
                actors[2].distanceMeters > clearanceDistances[2] &&
                actors[3].distanceMeters > clearanceDistances[3],
            "all straight-through vehicles clear the T-junction");
    std::cout << "T_JUNCTION_PROBE updates=" << updates
              << " simulated_seconds=" << updates * frameSeconds << " vehicles=" << actors.size()
              << " all_clear=1\n";
}

void verifyIncrementalLaneNetwork() {
    using namespace openbus::map;
    const auto straight = [](int id, double y) {
        MapTrafficRouteSegment segment;
        segment.spline.placement = makeSpline(id, 0, 0, y);
        segment.spline.placement.length = 20.0;
        segment.spline.placement.elevation = 0.0;
        return segment;
    };

    MapTrafficLaneNetwork network;
    const MapTrafficRouteSegment first = straight(1024, -20.0);
    const MapTrafficRouteSegment second = straight(1025, 0.0);
    network.addSegment(first);
    network.finalize();
    network.addSegment(second);
    network.finalize();
    const auto route = network.buildRoute(1024);
    require(route && route->segments.size() == 2,
            "lane networks can be extended and finalized as map tiles load");
    MapTrafficLaneNetwork batchNetwork;
    batchNetwork.addSegment(first);
    batchNetwork.addSegment(second);
    batchNetwork.finalize();
    const MapTrafficLaneNetworkStats incrementalStats = network.stats();
    const MapTrafficLaneNetworkStats batchStats = batchNetwork.stats();
    require(incrementalStats.lanes == batchStats.lanes &&
                incrementalStats.deadEnds == batchStats.deadEnds &&
                incrementalStats.nearMissDeadEnds == batchStats.nearMissDeadEnds &&
                network.spawnLanes() == batchNetwork.spawnLanes(),
            "incremental lane finalization matches a batch-built network");
}

void verifyDefaultRightOfWay() {
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
    auto fromSouth = makeMapTrafficActor(
        oneSegment(straight(1200, 0.0, -40.0, 0.0, 100.0)), "south-approach.ovh", 8.0, 0.0,
        false);
    auto fromEast = makeMapTrafficActor(
        oneSegment(straight(1201, 40.0, 0.0, 270.0, 100.0)), "east-approach.ovh", 8.0, 0.0,
        false);
    fromSouth.distanceMeters = 35.0;
    fromEast.distanceMeters = 35.0;
    bool southYieldedToRight = false;
    bool eastWasServedFirst = false;
    for (int frame = 0; frame < 20; ++frame) {
        advanceMapTrafficDriving({&fromSouth, &fromEast}, {}, 0.1);
        southYieldedToRight =
            southYieldedToRight ||
            (fromSouth.speedMetersPerSecond < 0.3 && fromEast.distanceMeters > 35.1);
        eastWasServedFirst =
            eastWasServedFirst || fromEast.distanceMeters > fromSouth.distanceMeters + 0.1;
    }
    require(southYieldedToRight && eastWasServedFirst,
            "an unmarked crossing yields to the vehicle approaching from the right");
}

void verifyAuthoredPriorityOrder() {
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
    verifyDefaultRightOfWay();

    MapTrafficRoute lowPriorityRoute = oneSegment(straight(1202, 0.0, -40.0, 0.0, 100.0));
    MapTrafficRoute highPriorityRoute = oneSegment(straight(1203, -40.0, 0.0, 90.0, 100.0));
    lowPriorityRoute.segments.front().pathRules =
        mapTrafficRulesForPath({{false, {"0", "priority", "64", "0"}}}, 0);
    highPriorityRoute.segments.front().pathRules =
        mapTrafficRulesForPath({{false, {"0", "priority", "256", "0"}}}, 0);
    auto lowPriority = makeMapTrafficActor(std::move(lowPriorityRoute), "low.ovh", 8.0, 0.0, false);
    auto highPriority =
        makeMapTrafficActor(std::move(highPriorityRoute), "high.ovh", 8.0, 0.0, false);
    lowPriority.unscheduledGroupIndex = 0;
    highPriority.unscheduledGroupIndex = 0;
    lowPriority.trafficRuleGroupIndex = 0;
    highPriority.trafficRuleGroupIndex = 0;
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
            "priority 256 wins over priority 64 before wait aging closes the priority gap");

    MapTrafficRoute shortLowPriorityRoute;
    auto highPriorityApproach = straight(1206, 0.0, -40.0, 0.0, 30.0);
    highPriorityApproach.pathRules.groups.push_back(
        MapTrafficGroupRules{.groupIndex = 0, .priority = 192.0});
    auto shortLowPriorityConnector = straight(1207, 0.0, -10.0, 0.0, 10.0);
    shortLowPriorityConnector.pathRules.groups.push_back(
        MapTrafficGroupRules{.groupIndex = 0, .priority = 64.0});
    shortLowPriorityRoute.segments = {highPriorityApproach, shortLowPriorityConnector,
                                      straight(1208, 0.0, 0.0, 0.0, 100.0)};
    shortLowPriorityRoute.cumulativeEndDistances = {30.0, 40.0, 140.0};
    shortLowPriorityRoute.totalLength = 140.0;
    MapTrafficRoute defaultPriorityCrossingRoute =
        oneSegment(straight(1209, -40.0, 0.0, 90.0, 100.0));
    auto vehicleApproachingShortLowPriority =
        makeMapTrafficActor(shortLowPriorityRoute, "short-low-priority.ovh", 8.0, 0.0, false);
    auto defaultPriorityCrossing = makeMapTrafficActor(
        defaultPriorityCrossingRoute, "default-priority-crossing.ovh", 8.0, 0.0, false);
    vehicleApproachingShortLowPriority.unscheduledGroupIndex = 0;
    defaultPriorityCrossing.unscheduledGroupIndex = 0;
    vehicleApproachingShortLowPriority.trafficRuleGroupIndex = 0;
    defaultPriorityCrossing.trafficRuleGroupIndex = 0;
    vehicleApproachingShortLowPriority.distanceMeters = 20.0;
    defaultPriorityCrossing.distanceMeters = 35.0;
    bool shortLowPriorityVehicleYielded = false;
    bool defaultPriorityTrafficWasServedFirst = false;
    for (int frame = 0; frame < 300; ++frame) {
        advanceMapTrafficDriving({&vehicleApproachingShortLowPriority, &defaultPriorityCrossing},
                                 {}, 0.1);
        if (vehicleApproachingShortLowPriority.junctionWaitSeconds > 0.0) {
            shortLowPriorityVehicleYielded = true;
            defaultPriorityTrafficWasServedFirst =
                defaultPriorityTrafficWasServedFirst ||
                defaultPriorityCrossing.distanceMeters >
                    vehicleApproachingShortLowPriority.distanceMeters;
        }
    }
    require(shortLowPriorityVehicleYielded && defaultPriorityTrafficWasServedFirst &&
                vehicleApproachingShortLowPriority.distanceMeters > 40.0 &&
                defaultPriorityCrossing.distanceMeters > 60.0,
            "a short low-priority connector lowers the approaching vehicle's junction priority");
}

} // namespace

int main(int argc, char** argv) {
    std::size_t verifiedAiSpawns = 0;
    std::size_t verifiedHumanSpawns = 0;
    std::size_t verifiedAiPopulationSpawns = 0;
    std::size_t verifiedHumanPopulationSpawns = 0;
    try {
        if (argc == 2 && std::string_view(argv[1]) == "--benchmark-lookahead") {
            benchmarkDrivingLookahead();
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--benchmark-traffic-scale") {
            benchmarkDrivingLookahead(333);
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--benchmark-traffic-scale-reduced") {
            benchmarkDrivingLookahead(333, 100.0, true);
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--benchmark-same-route") {
            benchmarkDrivingLookahead(1000, 100.0, false, true);
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--benchmark-traffic-dense") {
            benchmarkDrivingLookahead(333, 6.0);
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--benchmark-junctions") {
            benchmarkDrivingJunctionGrid();
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--benchmark-junction-substeps") {
            benchmarkDrivingJunctionGrid(true);
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--junction-substep-equivalence") {
            verifyJunctionSubstepEquivalence();
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--benchmark-queue") {
            benchmarkDrivingQueue();
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--dense-queue") {
            benchmarkDrivingQueue(true);
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--mixed-queue") {
            benchmarkDrivingQueue(true, true);
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--opposite-lanes") {
            verifyOppositeLaneTraffic();
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--close-follower") {
            verifyLeaderWithCloseFollower();
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--low-speed-stop") {
            verifyLowSpeedFollowingStop();
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--t-junction") {
            verifyTJunctionTraffic();
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--incremental-lane-network") {
            verifyIncrementalLaneNetwork();
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--default-right-of-way") {
            verifyDefaultRightOfWay();
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--authored-priority-order") {
            verifyAuthoredPriorityOrder();
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--passenger-service") {
            verifyPassengerSideOffsets();
            verifyPassengerStopDepartureHandshake();
            verifyPassengerDoorGating();
            verifyPassengerServiceRetirement();
            verifyPassengerServiceBehavior();
            verifyIntegratedPassengerJourney();
            verifyPassengerComponentAllocation();
            verifyPassengerStopSharing();
            verifyPlayerTripPolicyLifecycle();
            verifyScheduledBusTiming();
            verifyPlayerTimetableController();
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--committed-left-turn") {
            verifyCommittedLeftTurnPriority();
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--adjacent-straight-right-turn") {
            verifyAdjacentStraightAndRightTurn();
            return 0;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--rear-axle") {
            verifyRearAxleTrafficPose();
            return 0;
        }
        using namespace openbus::map;
        verifyRearAxleTrafficPose();
        verifyPassengerSideOffsets();
        verifyPassengerStopDepartureHandshake();
        verifyPassengerDoorGating();
        verifyPassengerServiceRetirement();
        verifyPassengerServiceBehavior();
        verifyIntegratedPassengerJourney();
        verifyPassengerComponentAllocation();
        verifyPassengerStopSharing();
        verifyPlayerTripPolicyLifecycle();
        verifyScheduledBusTiming();
        verifyPlayerTimetableController();
        verifyTrafficWaterClearance();
        verifyDrivingBehavior();
        verifyTrafficRules();
        verifyAutomaticRoundaboutPriority();
        verifyMalformedRouteErrors();
        verifyRealisticTrafficBehavior();
        verifyAdjacentStraightAndRightTurn();
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
        require(network.splineIdsInTiles({{1, 2}}) == std::vector<int>({10, 11}) &&
                    network.splineIdsInTiles({{1, 3}}).empty() &&
                    network.splineIdsInTiles({}).empty(),
                "traffic spline filtering uses the owning active tiles");
        MapTrafficRouteNetwork tileFilteredNetwork;
        bool unfinalizedTileLookupRejected = false;
        try {
            static_cast<void>(tileFilteredNetwork.splineIdsInTiles({{1, 2}}));
        } catch (const std::logic_error&) {
            unfinalizedTileLookupRejected = true;
        }
        require(unfinalizedTileLookupRejected, "tile filtering requires a finalized network");
        auto invalidSpline = makeSpline(35, 0, 0, 0.0);
        invalidSpline.geometryValid = false;
        tileFilteredNetwork.addTileSplines(
            {makeSpline(32, 0, 0, 0.0), makeSpline(30, 0, 0, 0.0), invalidSpline}, 1, 2);
        tileFilteredNetwork.addTileSplines({makeSpline(31, 0, 0, 0.0),
                                             makeSpline(33, 0, 0, 0.0)}, -2, 4);
        tileFilteredNetwork.addTileSplines({makeSpline(33, 0, 0, 0.0),
                                             makeSpline(34, 0, 0, 0.0)}, 8, 9);
        tileFilteredNetwork.finalize();
        const std::vector<std::pair<int, int>> debugTiles = {{-2, 4}, {1, 2}, {-2, 4}};
        const auto filteredIds = tileFilteredNetwork.splineIdsInTiles(debugTiles);
        require(filteredIds == std::vector<int>({30, 31, 32, 35}),
                "tile filtering retains sorted unique unambiguous IDs");
        std::vector<int> originalDebugIds;
        for (const int id : tileFilteredNetwork.splineIds()) {
            const auto candidate = tileFilteredNetwork.buildRoute(id);
            if (candidate && !candidate->segments.empty()) {
                const auto& spline = candidate->segments.front().spline;
                if (std::find(debugTiles.begin(), debugTiles.end(),
                              std::pair{spline.tileX, spline.tileY}) != debugTiles.end()) {
                    originalDebugIds.push_back(id);
                }
            }
        }
        std::vector<int> filteredDebugIds;
        for (const int id : filteredIds) {
            const auto candidate = tileFilteredNetwork.buildRoute(id);
            if (candidate && !candidate->segments.empty()) {
                filteredDebugIds.push_back(id);
            }
        }
        require(filteredDebugIds == originalDebugIds,
                "pre-filtering preserves the debug routes selected by the original algorithm");
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
        MapTrafficRouteSegment bidirectionalLane = firstLane;
        bidirectionalLane.reversed = true;
        bidirectionalLane.spline.placement = makeSpline(44, 0, 0, 0.0);
        MapTrafficLaneNetwork bidirectionalLanes;
        bidirectionalLanes.addSegment(bidirectionalLane);
        bidirectionalLanes.addSegment(reverseMapTrafficRouteSegment(
            bidirectionalLane, bidirectionalLane.spline.placement.length));
        bidirectionalLanes.finalize();
        const auto authoredReverseRoute = bidirectionalLanes.buildRoute(44, 0);
        const auto oppositeDirectionRoute = bidirectionalLanes.buildRoute(44, 1);
        require(authoredReverseRoute && oppositeDirectionRoute,
                "bidirectional roads retain both authored and opposite travel directions");
        const auto authoredReversePose = sampleMapTrafficRoute(*authoredReverseRoute, 2.0, 0.0);
        const auto oppositePose = sampleMapTrafficRoute(*oppositeDirectionRoute, 2.0, 0.0);
        require(authoredReversePose && oppositePose &&
                    std::abs(std::remainder(authoredReversePose->headingRadians - std::numbers::pi,
                                            2.0 * std::numbers::pi)) < 1.0e-8 &&
                    std::abs(oppositePose->headingRadians) < 1.0e-8,
                "bidirectional lane expansion flips the spline travel direction exactly once");
        requireNear(mapTrafficHeadingToVehicleYawDegrees(authoredReversePose->headingRadians),
                    -90.0, "reverse lane vehicle yaw faces its spline travel direction");
        requireNear(mapTrafficHeadingToVehicleYawDegrees(oppositePose->headingRadians), 90.0,
                    "opposite lane vehicle yaw faces its spline travel direction");
        MapTrafficRouteSegment gradedSegment;
        gradedSegment.spline.placement = makeSpline(45, 0, 0, 0.0);
        gradedSegment.spline.placement.elevation = 0.0;
        gradedSegment.spline.placement.gradientStart = 10.0;
        gradedSegment.spline.placement.gradientEnd = 10.0;
        gradedSegment.spline.placement.cantStart = 5.0;
        gradedSegment.spline.placement.cantEnd = 5.0;
        const auto makeGradedRoute = [](const MapTrafficRouteSegment& segment) {
            MapTrafficRoute route;
            route.segments = {segment};
            route.cumulativeEndDistances = {segment.spline.placement.length};
            route.totalLength = segment.spline.placement.length;
            return route;
        };
        const auto gradedRoute = makeGradedRoute(gradedSegment);
        const auto gradedPose = sampleMapTrafficRoute(gradedRoute, 5.0, 0.0);
        require(gradedPose.has_value(), "graded road pose includes full surface orientation");
        requireNear(gradedPose->pitchRadians, std::atan(0.1),
                    "vehicle pitch follows authored uphill gradient");
        requireNear(gradedPose->rollRadians, -std::atan(0.05),
                    "vehicle roll follows authored road cant");
        gradedSegment.spline.placement.skewStart = 0.5;
        gradedSegment.spline.placement.skewEnd = 0.5;
        gradedSegment.pathLateralMeters = 2.0;
        const auto skewedPose = sampleMapTrafficRoute(makeGradedRoute(gradedSegment), 5.0, 0.0);
        require(skewedPose.has_value(), "skewed road pose is valid");
        requireNear(skewedPose->position[1], 6.0,
                    "traffic lane follows the road surface's lateral skew");
        requireNear(skewedPose->position[2], 0.5,
                    "traffic lane elevation follows the skew-adjusted road surface");
        gradedSegment.pathLateralMeters = 0.0;
        gradedSegment.spline.placement.skewStart = 0.0;
        gradedSegment.spline.placement.skewEnd = 0.0;
        gradedSegment.reversed = true;
        gradedSegment.splineDistanceAtRouteStartMeters = gradedSegment.spline.placement.length;
        const auto downhillPose = sampleMapTrafficRoute(makeGradedRoute(gradedSegment), 5.0, 0.0);
        require(downhillPose.has_value(), "reversed graded road pose is valid");
        requireNear(downhillPose->pitchRadians, -std::atan(0.1),
                    "reversed vehicle pitch follows downhill travel");
        requireNear(downhillPose->rollRadians, std::atan(0.05),
                    "reversed vehicle roll follows the opposite road edge");
        MapTrafficRouteSegment sceneryGrade;
        sceneryGrade.points = {{0.0, 0.0, 0.0}, {0.0, 10.0, 1.0}};
        MapTrafficRoute sceneryGradeRoute;
        sceneryGradeRoute.segments = {sceneryGrade};
        sceneryGradeRoute.cumulativeEndDistances = {10.0};
        sceneryGradeRoute.totalLength = 10.0;
        const auto sceneryGradePose = sampleMapTrafficRoute(sceneryGradeRoute, 5.0, 0.0);
        require(sceneryGradePose.has_value(), "scenery road grade pose is valid");
        requireNear(sceneryGradePose->pitchRadians, std::atan(0.1),
                    "vehicle pitch follows scenery-path elevation");
        const auto assertHeadingFollowsRoute = [](const MapTrafficRoute& route, double distance,
                                                  const char* message) {
            const double heading = mapTrafficRouteTravelHeadingRadians(route, distance, 0.0);
            const auto before = sampleMapTrafficRoute(route, distance - 0.5, 0.0);
            const auto after = sampleMapTrafficRoute(route, distance + 0.5, 0.0);
            require(before.has_value() && after.has_value(), message);
            const double dx = after->position[0] - before->position[0];
            const double dy = after->position[1] - before->position[1];
            require(dx * std::sin(heading) + dy * std::cos(heading) > 0.0, message);
        };
        assertHeadingFollowsRoute(*authoredReverseRoute, 2.0,
                                  "reverse-lane vehicle yaw follows increasing route distance");
        assertHeadingFollowsRoute(*oppositeDirectionRoute, 2.0,
                                  "opposite-lane vehicle yaw follows increasing route distance");
        requireNear(mapTrafficRouteTravelHeadingRadians(*authoredReverseRoute, 2.0, 0.0),
                    std::acos(-1.0), "reverse-lane travel heading points along route movement");
        requireNear(mapTrafficRouteTravelHeadingRadians(*oppositeDirectionRoute, 2.0, 0.0), 0.0,
                    "opposite-lane travel heading points along route movement");
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
        MapTrafficRouteSegment timetableAfterDestination;
        timetableAfterDestination.spline.placement.splineId = 63;
        timetableAfterDestination.points = {{10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
        timetableLanes.addSegment(timetableApproach);
        timetableLanes.addSegment(timetableStraight);
        timetableLanes.addSegment(timetableTurn);
        timetableLanes.addSegment(timetableAfterDestination);
        timetableLanes.finalize();
        const auto authoredEntryLanes = timetableLanes.entryLanes();
        const auto timetableStartLanes = timetableLanes.lanesForTimetableEntry(61, std::nullopt);
        require(std::find(authoredEntryLanes.begin(), authoredEntryLanes.end(),
                          MapTrafficLaneStart{61, 0}) == authoredEntryLanes.end() &&
                    timetableStartLanes == std::vector<MapTrafficLaneStart>{{61, 0}},
                "timetable starts can select their authored route entry without a mid-route spawn");
        const auto timetableGuidedRoute = timetableLanes.buildRoute(60, 0, 0, 0.0, {}, {60, 62});
        require(timetableGuidedRoute && timetableGuidedRoute->segments.size() == 3 &&
                    timetableGuidedRoute->segments[1].spline.placement.splineId == 62 &&
                    timetableGuidedRoute->segments.back().spline.placement.splineId == 63,
                "a timetable spline sequence guides route choice through a non-default turn");
        const auto timetableEndingRoute =
            timetableLanes.buildRoute(60, 0, 0, 0.0, {}, {60, 62}, {}, true);
        require(timetableEndingRoute && timetableEndingRoute->segments.size() == 2 &&
                    timetableEndingRoute->segments.back().spline.placement.splineId == 62,
                "a scheduled route can end at the timetable destination instead of continuing");
        require(!timetableLanes.buildRoute(60, 0, 0, 0.0, {}, {60, 63}),
                "a timetable-guided route rejects a spline sequence absent from the lane network");
        MapTrafficLaneNetwork indexedSceneryPaths;
        MapTrafficRouteSegment sceneryPathA;
        sceneryPathA.spline.placement.splineId = 70;
        sceneryPathA.sceneryPathIndex = 0;
        sceneryPathA.points = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
        MapTrafficRouteSegment sceneryPathB;
        sceneryPathB.spline.placement.splineId = 70;
        sceneryPathB.sceneryPathIndex = 1;
        sceneryPathB.points = {{10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
        MapTrafficRouteSegment sceneryExit;
        sceneryExit.spline.placement.splineId = 71;
        sceneryExit.points = {{20.0, 0.0, 0.0}, {30.0, 0.0, 0.0}};
        indexedSceneryPaths.addSegment(sceneryPathA);
        indexedSceneryPaths.addSegment(sceneryPathB);
        indexedSceneryPaths.addSegment(sceneryExit);
        indexedSceneryPaths.finalize();
        const auto indexedTimetableRoute = indexedSceneryPaths.buildRoute(
            70, 0, 0, 0.0, {}, {70, 70, 71}, {std::size_t{0}, std::size_t{1}, std::nullopt});
        require(indexedTimetableRoute && indexedTimetableRoute->segments.size() == 3 &&
                    indexedTimetableRoute->segments[1].sceneryPathIndex == 1,
                "timetable path indices disambiguate successive paths of the same SCO");
        require(!indexedSceneryPaths.buildRoute(70, 0, 0, 0.0, {}, {70, 70, 71},
                                                {std::size_t{0}, std::size_t{0}, std::nullopt}),
                "a timetable path index rejects the wrong SCO junction movement");
        MapTrafficLaneNetwork elevatedTimetablePaths;
        MapTrafficRouteSegment elevatedPathA;
        elevatedPathA.spline.placement.splineId = 72;
        elevatedPathA.sceneryPathIndex = 0;
        elevatedPathA.points = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
        MapTrafficRouteSegment elevatedPathB;
        elevatedPathB.spline.placement.splineId = 72;
        elevatedPathB.sceneryPathIndex = 1;
        elevatedPathB.points = {{20.0, 0.0, 1.7424}, {10.0, 0.0, 1.7424}};
        elevatedTimetablePaths.addSegment(elevatedPathA);
        elevatedTimetablePaths.addSegment(elevatedPathB);
        elevatedTimetablePaths.finalize();
        const auto elevatedReverseTimetableRoute = elevatedTimetablePaths.buildRoute(
            72, 0, 0, 0.0, {}, {72, 72}, {std::size_t{0}, std::size_t{1}}, true, true, true);
        require(elevatedReverseTimetableRoute &&
                    elevatedReverseTimetableRoute->segments.size() == 2 &&
                    elevatedReverseTimetableRoute->segments[1].points.front() ==
                        elevatedReverseTimetableRoute->segments[0].points.back(),
                "a required reverse SCO path joins across the permitted 2.5 m elevation gap");
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
        require(mapTrafficWeekdayMaskAtElapsedSeconds(5, 23.5, 0.0) == 1 &&
                    mapTrafficWeekdayMaskAtElapsedSeconds(5, 23.5, 1799.999) == 1 &&
                    mapTrafficWeekdayMaskAtElapsedSeconds(5, 23.5, 1800.0) == 2 &&
                    mapTrafficWeekdayMaskAtElapsedSeconds(5, 23.5, 1800.0 + 86400.0) == 4 &&
                    mapTrafficWeekdayMaskAtElapsedSeconds(5, 23.5, 1800.0 + 2.0 * 86400.0) == 1 &&
                    mapTrafficWeekdayMaskAtElapsedSeconds(5, 23.5, 7.0 * 86400.0) == 1 &&
                    mapTrafficWeekdayMaskAtElapsedSeconds(6, 0.0, 0.0) == 2 &&
                    mapTrafficWeekdayMaskAtElapsedSeconds(6, 0.0, 86400.0) == 4,
                "unscheduled density weekday class changes at calendar midnight and wraps weekly");
        bool rejectedInvalidWeekdayClock = false;
        try {
            static_cast<void>(mapTrafficWeekdayMaskAtElapsedSeconds(5, 24.0, 0.0));
        } catch (const std::invalid_argument&) {
            rejectedInvalidWeekdayClock = true;
        }
        require(rejectedInvalidWeekdayClock, "weekday clock rejects invalid start hours");
        MapAiTrafficData dayTraffic;
        dayTraffic.hasUnscheduledTrafficDensityFile = true;
        dayTraffic.unscheduledTrafficDensityGroups = {
            {"Cars", 1.0, {{1, {{0.0, 0.9}}}, {2, {{0.0, 0.5}}}, {4, {{0.0, 0.2}}}}}};
        MapAiRoadVehicle dayVehicle;
        dayVehicle.groupName = "Cars";
        const auto densityAt = [&](double elapsed) {
            return mapAiRoadTrafficDensityAtHour(
                dayTraffic, {dayVehicle}, mapTrafficHourAtElapsedSeconds(23.5, elapsed),
                mapTrafficWeekdayMaskAtElapsedSeconds(5, 23.5, elapsed));
        };
        require(densityAt(1799.0).has_value() && densityAt(1800.0).has_value() &&
                    densityAt(88200.0).has_value(),
                "Friday, Saturday, and Sunday density schedules resolve");
        requireNear(*densityAt(1799.0), 0.9, "Friday density remains active before midnight");
        requireNear(*densityAt(1800.0), 0.5, "Saturday density activates at midnight");
        requireNear(*densityAt(88200.0), 0.2, "Sunday density activates at the next midnight");
        requireNear(mapTrafficNextDepartureElapsedSeconds(6.0, 0.0, 405.0), 2700.0,
                    "scheduled departure is measured from the configured start hour");
        requireNear(mapTrafficNextDepartureElapsedSeconds(23.5, 0.0, 30.0), 3600.0,
                    "a departure after midnight is scheduled on the next clock day");
        requireNear(mapTrafficNextDepartureElapsedSeconds(6.75, 0.0, 405.0), 0.0,
                    "departure at the configured start hour is immediately due");
        requireNear(mapTrafficNextDepartureElapsedSeconds(8.05, 0.0, 483.0), 0.0,
                    "a decimal start hour matching a timetable departure is immediately due");
        require(
            mapTrafficScheduledDepartureDue(
                0.0, mapTrafficNextDepartureElapsedSeconds(8.05, 0.0, 483.0)),
            "a timetable departure at a decimal start hour does not produce a negative due time");
        requireNear(mapTrafficNextDepartureElapsedSeconds(6.75, 1.0, 405.0), 86400.0,
                    "a missed departure rolls to the following day");
        require(mapTrafficNextDepartureIndex(12.0, 0.0, {483.0, 739.0, 780.0}) == 1,
                "the initial bus departure is the next one today, not the first timetable time");
        requireNear(mapTrafficNextDepartureElapsedSeconds(12.0, 0.0, 739.0), 1140.0,
                    "the selected noon-start timetable departure is due in 19 minutes");
        require(mapTrafficNextDepartureIndex(14.0, 0.0, {483.0, 739.0, 780.0}) == 0,
                "after today's final departure, buses roll over to the first time tomorrow");
        openbus::app::ScheduledBusState rebasedBus;
        rebasedBus.scheduledDepartureMinutes = {390.0, 780.0};
        rebasedBus.scheduledTripActive = true;
        rebasedBus.activeScheduledDepartureElapsedSeconds = 1800.0;
        openbus::app::rebaseScheduledBusClock(rebasedBus, 8.0, 3600.0);
        require(rebasedBus.nextScheduledDepartureIndex == 1,
                "rebasing selects the next departure at the updated map time");
        requireNear(rebasedBus.nextScheduledDepartureElapsedSeconds, 18000.0,
                    "rebased next departure uses the new clock epoch");
        requireNear(rebasedBus.activeScheduledDepartureElapsedSeconds, -1800.0,
                    "rebasing preserves the elapsed time of an active trip");
        openbus::app::ScheduledBusState rebasedSuppression;
        rebasedSuppression.scheduledDepartureMinutes = {390.0, 780.0};
        rebasedSuppression.suppressedDeparture =
            openbus::app::ScheduledDepartureSuppression{0, 1800.0};
        openbus::app::rebaseScheduledBusClock(rebasedSuppression, 8.0, 3600.0);
        require(rebasedSuppression.suppressedDeparture.has_value() &&
                    rebasedSuppression.suppressedDeparture->departureIndex == 1 &&
                    rebasedSuppression.suppressedDeparture->departureElapsedSeconds == 18000.0,
                "rebasing keeps player-trip suppression aligned with its next departure");
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
        require(
            mapTrafficAllocateScheduledBuses({5, 2, 1}, 2) == std::vector<std::size_t>({1, 1, 0}) &&
                mapTrafficAllocateScheduledBuses({5, 2, 1}, 5) ==
                    std::vector<std::size_t>({3, 1, 1}) &&
                mapTrafficAllocateScheduledBuses({0, 5, 1}, 1) ==
                    std::vector<std::size_t>({0, 1, 0}) &&
                mapTrafficAllocateScheduledBuses({0, 0}, 4) == std::vector<std::size_t>({0, 0}),
            "scheduled bus pools skip routes without departures and balance eligible route load");
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
