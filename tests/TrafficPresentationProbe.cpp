#include "app/traffic/TrafficPresentation.h"
#include "map/MapTrafficRuntime.h"

#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>

namespace {

void near(double actual, double expected) {
    if (std::abs(actual - expected) > 1.0e-9) {
        throw std::runtime_error("Unexpected interpolated traffic sample");
    }
}

} // namespace

int main() {
    try {
        openbus::app::TrafficPresentation presentation;
        presentation.reset(1, {10.0, 0.0});
        if (presentation.retarget(1, {20.0, 2.0}, 1.0, 20.0, 100.0, std::nullopt)) {
            throw std::runtime_error("Ordinary traffic update unexpectedly snapped");
        }
        near(presentation.sample().distanceMeters, 10.0);
        presentation.advance(0.25);
        near(presentation.sample().distanceMeters, 12.5);
        near(presentation.sample().lateralOffsetMeters, 0.5);
        presentation.retarget(1, {30.0, 4.0}, 1.0, 20.0, 100.0, std::nullopt);
        near(presentation.sample().distanceMeters, 12.5);
        presentation.advance(0.5);
        near(presentation.sample().distanceMeters, 21.25);
        presentation.advance(10.0);
        near(presentation.sample().distanceMeters, 30.0);

        presentation.reset(6, {10.0, 0.0});
        presentation.retarget(6, {20.0, 0.0}, 1.0, 20.0, 100.0, std::nullopt);
        double wheelDistance = 0.0;
        double previousDistance = presentation.sample().distanceMeters;
        for (int frame = 0; frame < 10; ++frame) {
            presentation.advance(0.1);
            const double distance = presentation.sample().distanceMeters;
            const double travel = openbus::app::trafficPresentationTravel(
                previousDistance, distance, 100.0, std::nullopt);
            near(travel, 1.0);
            wheelDistance += travel;
            previousDistance = distance;
        }
        near(wheelDistance, 10.0);
        near(openbus::app::trafficPresentationTravel(99.0, 1.0, 100.0, 0.0), 2.0);
        near(openbus::app::trafficPresentationTravel(259.0, 21.0, 100.0, 20.0), 2.0);
        near(openbus::app::trafficPresentationTravel(20.0, 10.0, 100.0, std::nullopt), 0.0);
        near(openbus::app::trafficPresentationTravel(std::nextafter(20.0, 21.0), 20.0,
                                                     100.0, 0.0), 0.0);
        near(openbus::app::trafficPresentationTravel(std::nextafter(100.0, 101.0), 0.0,
                                                     100.0, 0.0), 0.0);

        presentation.reset(2, {98.0, 0.0});
        presentation.retarget(2, {2.0, 0.0}, 1.0, 10.0, 100.0, 0.0);
        presentation.advance(0.5);
        near(presentation.sample().distanceMeters, 100.0);
        presentation.advance(0.5);
        near(presentation.sample().distanceMeters, 2.0);
        presentation.retarget(2, {3.0, 0.0}, 1.0, 10.0, 100.0, 0.0);
        presentation.advance(1.0);
        near(presentation.sample().distanceMeters, 3.0);

        presentation.reset(2, {98.0, 0.0});
        presentation.retarget(2, {22.0, 0.0}, 1.0, 10.0, 100.0, 20.0);
        presentation.advance(0.5);
        near(presentation.sample().distanceMeters, 100.0);
        presentation.retarget(2, {24.0, 0.0}, 1.0, 10.0, 100.0, 20.0);
        near(presentation.sample().distanceMeters, 100.0);
        presentation.advance(1.0);
        near(presentation.sample().distanceMeters, 24.0);

        if (!presentation.retarget(3, {50.0, 0.0}, 1.0, 100.0, 100.0, std::nullopt)) {
            throw std::runtime_error("Traffic generation change did not report a snap");
        }
        near(presentation.sample().distanceMeters, 50.0);
        presentation.retarget(3, {5.0, 0.0}, 1.0, 100.0, 100.0, std::nullopt);
        near(presentation.sample().distanceMeters, 5.0);
        if (!presentation.retarget(3, {80.0, 0.0}, 1.0, 10.0, 100.0, std::nullopt)) {
            throw std::runtime_error("Traffic discontinuity did not report a snap");
        }
        near(presentation.sample().distanceMeters, 80.0);
        presentation.retarget(3, {81.0, 0.0}, 0.0, 10.0, 100.0, std::nullopt);
        near(presentation.sample().distanceMeters, 81.0);

        openbus::map::MapTrafficRoute route;
        openbus::map::MapTrafficRouteSegment segment;
        segment.points = {{0.0, 0.0, 0.0}, {0.0, 10.0, 0.0}, {10.0, 10.0, 0.0}};
        route.segments.push_back(segment);
        route.cumulativeEndDistances = {20.0};
        route.totalLength = 20.0;
        presentation.reset(4, {5.0, 0.0});
        presentation.retarget(4, {15.0, 0.0}, 1.0, 20.0, route.totalLength, std::nullopt);
        presentation.advance(0.5);
        const auto corner = openbus::map::sampleMapTrafficActorPose(
            route, presentation.sample().distanceMeters, 0.0, 0.0);
        if (!corner) {
            throw std::runtime_error("Could not sample interpolated route");
        }
        near(corner->position[0], 0.0);
        near(corner->position[1], 10.0);

        const double edgeLength = std::hypot(0.1, 10.0);
        route.segments[0].points = {{0.0, 0.0, 0.0}, {0.1, -10.0, 0.0}, {0.0, -20.0, 0.0}};
        route.totalLength = edgeLength * 2.0;
        route.cumulativeEndDistances = {route.totalLength};
        presentation.reset(5, {edgeLength - 1.0, 0.0});
        presentation.retarget(5, {edgeLength + 1.0, 0.0}, 1.0, 3.0, route.totalLength,
                              std::nullopt);
        presentation.advance(0.5);
        const auto wrapHeading = openbus::map::sampleMapTrafficActorPose(
            route, presentation.sample().distanceMeters, 0.0, 0.0);
        if (!wrapHeading || std::abs(std::remainder(wrapHeading->headingRadians - std::numbers::pi,
                                                    2.0 * std::numbers::pi)) > 0.02) {
            throw std::runtime_error("Interpolated route heading did not cross the angle wrap");
        }

        bool rejected = false;
        try {
            presentation.advance(-1.0);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        if (!rejected) {
            throw std::runtime_error("Negative presentation time was not rejected");
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
