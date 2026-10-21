#include "map/MapTrafficRuntime.h"

#include <cmath>
#include <iostream>
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

} // namespace

int main() {
    try {
        using namespace openbus::map;
        MapTrafficRouteNetwork network;
        network.addTileSplines({makeSpline(10, 0, 11, 6.0)}, 1, 2);
        network.addTileSplines({makeSpline(11, 10, 0, 16.0)}, 1, 2);
        network.finalize();
        require(network.splineIds() == std::vector<int>({10, 11}),
                "traffic route start IDs are returned in deterministic order");
        const auto route = network.buildRoute(10);
        require(route.has_value() && route->segments.size() == 2 && !route->closed,
                "route follows reciprocal spline links and stops at the open end");
        requireNear(route->totalLength, 20.0, "route length sums linked splines");
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
        MapTrafficActor createdActor = makeMapTrafficActor(
            *route, "Vehicles\\traffic.bus", 8.0, 1.5, true,
            std::string("Humans\\driver.hum"));
        require(createdActor.modelPath == "Vehicles\\traffic.bus" &&
                    createdActor.driverModelPath == "Humans\\driver.hum" &&
                    createdActor.speedMetersPerSecond == 8.0 &&
                    createdActor.lateralOffsetMeters == -1.5,
                "actor factory retains assets, movement speed, and map-side lane offset");

        MapTrafficRouteNetwork reverseNetwork;
        reverseNetwork.addTileSplines({makeSpline(30, 0, 31, 0.0)}, 0, 0);
        reverseNetwork.addTileSplines({makeSpline(31, 0, 30, 10.0)}, 0, 0);
        reverseNetwork.finalize();
        const auto reverseRoute = reverseNetwork.buildRoute(30);
        require(reverseRoute.has_value() && reverseRoute->segments.size() == 2 &&
                    reverseRoute->segments[1].reversed,
                "route records reversed traversal of a reciprocal spline");
        const auto reversePose = sampleMapTrafficRoute(
            *reverseRoute, 12.0, mapTrafficLaneOffsetMeters(false, 1.0));
        require(reversePose.has_value(), "reverse-traversed actor pose is sampled");
        requireNear(reversePose->headingRadians, std::acos(-1.0),
                    "actor heading follows reversed spline traversal");
        requireNear(reversePose->position[0], 4.0,
                    "positive lane offset remains right of reversed travel");
        const auto leftHandReversePose = sampleMapTrafficRoute(
            *reverseRoute, 12.0, mapTrafficLaneOffsetMeters(true, 1.0));
        require(leftHandReversePose.has_value(), "left-hand reverse actor pose is sampled");
        requireNear(leftHandReversePose->position[0], 6.0,
                    "left-hand lane offset uses the opposite side of reversed travel");

        MapTrafficActor actor;
        actor.route = *route;
        actor.distanceMeters = 9.0;
        actor.speedMetersPerSecond = 4.0;
        require(advanceMapTrafficActor(actor, 1.0), "actor remains on an open route");
        requireNear(actor.distanceMeters, 13.0, "actor advances by speed times elapsed time");
        const auto advancedPose = sampleMapTrafficRoute(
            actor.route, actor.distanceMeters, actor.lateralOffsetMeters);
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
        actor.route = *loop;
        actor.distanceMeters = 18.0;
        actor.speedMetersPerSecond = 5.0;
        require(advanceMapTrafficActor(actor, 1.0), "actor remains on a closed route");
        requireNear(actor.distanceMeters, 3.0, "closed route wraps actor distance");

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
    } catch (const std::exception& error) {
        std::cerr << "map traffic runtime probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
