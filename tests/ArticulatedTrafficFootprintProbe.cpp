#include "map/MapTrafficRuntime.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void near(double actual, double expected) {
    if (std::abs(actual - expected) > 1.0e-8) {
        throw std::runtime_error("Articulated traffic footprint position did not match");
    }
}

} // namespace

int main() {
    try {
        openbus::map::MapTrafficRoute route;
        openbus::map::MapTrafficRouteSegment segment;
        segment.points = {{0.0, 0.0, 0.0}, {0.0, 100.0, 0.0}};
        route.segments.push_back(segment);
        route.cumulativeEndDistances = {100.0};
        route.totalLength = 100.0;

        openbus::map::MapTrafficActor actor;
        actor.route = route;
        actor.distanceMeters = 20.0;
        actor.lengthMeters = 4.0;
        actor.widthMeters = 2.0;
        actor.articulatedRear = openbus::map::MapTrafficActor::ArticulatedRear{
            {-2.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 4.0, 2.0};

        const auto footprints = openbus::map::mapTrafficActorFootprints(actor, 20.0);
        if (footprints.size() != 2) {
            throw std::runtime_error("Articulated traffic actor did not produce two footprints");
        }
        near(footprints[0].pose.position[1], 20.0);
        near(footprints[1].pose.position[1], 17.0);

        openbus::map::MapTrafficObstacle rearOnlyObstacle;
        rearOnlyObstacle.pose.position = {0.0, 16.0, 0.0};
        rearOnlyObstacle.pose.headingRadians = 0.0;
        rearOnlyObstacle.lengthMeters = 1.0;
        rearOnlyObstacle.widthMeters = 1.0;
        const std::vector<openbus::map::MapTrafficObstacle> obstacles = {rearOnlyObstacle};
        if (!openbus::map::mapTrafficHasClearance(footprints.front(), obstacles, 0.0)) {
            throw std::runtime_error("Lead footprint incorrectly overlaps the rear-only obstacle");
        }
        if (openbus::map::mapTrafficActorHasClearance(actor, 20.0, obstacles, 0.0)) {
            throw std::runtime_error("Articulated rear footprint ignored an obstacle");
        }
        openbus::map::MapTrafficObstacle spawnObstacle = rearOnlyObstacle;
        spawnObstacle.pose.position[1] = -4.5;
        if (openbus::map::canSpawnMapTrafficEntry(
                actor, true, std::vector<openbus::map::MapTrafficObstacle>{spawnObstacle})) {
            throw std::runtime_error("Articulated actor spawned through an occupied rear section");
        }

        auto articulatedLead = openbus::map::makeMapTrafficActor(route, "articulated.bus", 0.0,
                                                                 0.0, false);
        articulatedLead.distanceMeters = 20.0;
        articulatedLead.lengthMeters = 4.0;
        articulatedLead.widthMeters = 2.0;
        articulatedLead.articulatedRear = actor.articulatedRear;
        auto ordinaryLead = articulatedLead;
        ordinaryLead.articulatedRear.reset();
        auto articulatedFollower =
            openbus::map::makeMapTrafficActor(route, "follower.ovh", 5.0, 0.0, false);
        auto ordinaryFollower = articulatedFollower;
        articulatedFollower.distanceMeters = 12.0;
        ordinaryFollower.distanceMeters = 12.0;
        articulatedFollower.speedMetersPerSecond = 0.0;
        ordinaryFollower.speedMetersPerSecond = 0.0;
        for (int step = 0; step < 40; ++step) {
            openbus::map::advanceMapTrafficDriving({&articulatedFollower, &articulatedLead}, {},
                                                   0.1);
            openbus::map::advanceMapTrafficDriving({&ordinaryFollower, &ordinaryLead}, {}, 0.1);
        }
        if (!(articulatedFollower.distanceMeters + 0.5 < ordinaryFollower.distanceMeters)) {
            throw std::runtime_error(
                "Traffic collision sweep did not account for the articulated rear section");
        }

        auto stoppedArticulatedLead =
            openbus::map::makeMapTrafficActor(route, "stopped-articulated.bus", 0.0, 0.0, false);
        stoppedArticulatedLead.distanceMeters = 25.0;
        stoppedArticulatedLead.articulatedRear = actor.articulatedRear;
        auto stoppedOrdinaryLead = stoppedArticulatedLead;
        stoppedOrdinaryLead.articulatedRear.reset();
        auto followingArticulated =
            openbus::map::makeMapTrafficActor(route, "following-articulated.ovh", 2.0, 0.0, false);
        auto followingOrdinary = followingArticulated;
        followingArticulated.distanceMeters = 16.0;
        followingOrdinary.distanceMeters = 16.0;
        for (int step = 0; step < 40; ++step) {
            openbus::map::advanceMapTrafficDriving(
                {&followingArticulated, &stoppedArticulatedLead}, {}, 0.1);
            openbus::map::advanceMapTrafficDriving(
                {&followingOrdinary, &stoppedOrdinaryLead}, {}, 0.1);
        }
        if (!(followingArticulated.distanceMeters + 0.1 < followingOrdinary.distanceMeters)) {
            throw std::runtime_error(
                "Following-distance control did not account for the articulated rear section");
        }

        actor.articulatedRear->widthMeters = 0.0;
        bool rejected = false;
        try {
            (void)openbus::map::mapTrafficActorFootprints(actor, 20.0);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        if (!rejected) {
            throw std::runtime_error("Invalid articulated rear footprint was accepted");
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
