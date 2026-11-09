#include "physics/BusSimulation.h"
#include "physics/World.h"
#include "core/Variables.h"

#include <cmath>
#include <iostream>

namespace {

BusConfiguration articulatedSectionConfiguration() {
    BusConfiguration configuration{};
    configuration.mass = 12620.0;
    configuration.length = 12.6;
    configuration.width = 2.546;
    configuration.bodyHalfLength = 6.3;
    configuration.bodyHalfWidth = 1.273;
    configuration.bodyHalfHeight = 1.55;
    configuration.wheelRadius = 0.47825;
    configuration.wheelHalfWidth = 0.145;
    configuration.collisionLength = 12.6;
    configuration.collisionWidth = 2.546;
    configuration.collisionHeight = 3.1;
    configuration.collisionOffsetZ = 0.8;
    configuration.centerOfGravityHeight = 1.3;
    configuration.momentOfInertia = {300.0, 80.0, 300.0};
    configuration.axles = {{3.2921, 2.4, 2.4, 1.85, 0.9565, 240000.0, 90000.0,
                            20000.0, true, false},
                           {-3.0008, 2.45, 2.45, 1.25, 0.9565, 280000.0, 116000.0,
                            20000.0, false, true}};
    return configuration;
}

std::array<double, 3> couplingAnchor(const BusSimulation& simulation,
                                     const BusConfiguration& configuration,
                                     const std::array<double, 3>& localPoint) {
    const BodyPose pose = simulation.chassisPose();
    const std::array<double, 3> localOffset = {
        localPoint[0], localPoint[1], localPoint[2] - configuration.centerOfGravityHeight};
    return {pose.position[0] + pose.rotation[0] * localOffset[0] +
                pose.rotation[1] * localOffset[1] + pose.rotation[2] * localOffset[2],
            pose.position[1] + pose.rotation[3] * localOffset[0] +
                pose.rotation[4] * localOffset[1] + pose.rotation[5] * localOffset[2],
            pose.position[2] + pose.rotation[6] * localOffset[0] +
                pose.rotation[7] * localOffset[1] + pose.rotation[8] * localOffset[2]};
}

double distance(const std::array<double, 3>& first, const std::array<double, 3>& second) {
    const double dx = first[0] - second[0];
    const double dy = first[1] - second[1];
    const double dz = first[2] - second[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace

int main() {
    {
        World crowdedWorld;
        const BusConfiguration configuration = articulatedSectionConfiguration();
        BusSimulation front(crowdedWorld, configuration, VehiclePlacement{{0.0, 0.0, 0.0}, 0.0});
        BusSimulation rear(crowdedWorld, configuration, VehiclePlacement{{30.0, 0.0, 0.0}, 0.0});
        BusSimulation unrelated(crowdedWorld, configuration,
                                VehiclePlacement{{60.0, 0.0, 0.0}, 0.0});
        bool rejected = false;
        try {
            BusSimulation::stepTogether(front, {}, rear, {});
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        if (!rejected || front.simulationTime() != 0.0 || rear.simulationTime() != 0.0 ||
            unrelated.simulationTime() != 0.0) {
            std::cerr << "Paired stepping did not reject an uncoordinated third bus simulation\n";
            return 1;
        }
    }

    World world;
    const BusConfiguration configuration = articulatedSectionConfiguration();
    BusSimulation front(world, configuration, VehiclePlacement{{0.0, 0.0, 0.0}, 0.0});
    BusSimulation rear(world, configuration, VehiclePlacement{{30.0, 0.0, 0.0}, 0.0});

    BusConfiguration trailerConfiguration = configuration;
    trailerConfiguration.axles.resize(1);
    World trailerWorld(0.0, {}, {}, false);
    BusSimulation trailer(trailerWorld, trailerConfiguration,
                          VehiclePlacement{{0.0, 0.0, 0.0}, 0.0});
    openbus::scripting::Vehicle rearVariables;
    rearVariables.set("wheel_rotation_2_l", -1.0F);
    rearVariables.set("wheel_rotationspeed_2_r", -1.0F);
    rearVariables.set("axle_suspension_2_l", -1.0F);
    rearVariables.set("axle_steering_2_r", -1.0F);
    rearVariables.set("axle_brakeforce_actual_2_l", -1.0F);
    rearVariables.set("wheel_rotation_3_l", 123.0F);
    rearVariables.set("axle_suspension_0_l", 123.0F);
    trailer.updateVariables(rearVariables, 0.0, 0.0, 0.0, front.axleCount());
    if (rearVariables.get("wheel_rotation_2_l") == -1.0F ||
        rearVariables.get("wheel_rotationspeed_2_r") == -1.0F ||
        rearVariables.get("axle_suspension_2_l") == -1.0F ||
        rearVariables.get("axle_steering_2_r") == -1.0F ||
        rearVariables.get("axle_brakeforce_actual_2_l") == -1.0F ||
        rearVariables.get("wheel_rotation_3_l") != 123.0F ||
        rearVariables.get("axle_suspension_0_l") != 123.0F) {
        std::cerr << "Articulated section physics variables did not use the combined axle indices\n";
        return 1;
    }

    dBodyID stepCounter = dBodyCreate(world.physicsWorld());
    dMass mass;
    dMassSetSphereTotal(&mass, 1.0, 1.0);
    dBodySetMass(stepCounter, &mass);
    dBodySetPosition(stepCounter, 10000.0, 10000.0, 100.0);

    BusSimulation::stepTogether(front, {}, rear, {});
    const double velocityAfterOneStep = dBodyGetLinearVel(stepCounter)[2];
    if (std::abs(velocityAfterOneStep + 9.81 / 60.0) > 1.0e-6 ||
        std::abs(front.simulationTime() - 1.0 / 60.0) > 1.0e-9 ||
        std::abs(rear.simulationTime() - 1.0 / 60.0) > 1.0e-9) {
        std::cerr << "Articulated step did not advance both sections in one shared world step\n";
        dBodyDestroy(stepCounter);
        return 1;
    }

    dBodyDestroy(stepCounter);
    BusSimulation::updateTogether(front, {}, rear, {}, 3.0 / 60.0);
    if (front.lastStepCount() != 3 || rear.lastStepCount() != 3 ||
        std::abs(front.simulationTime() - 4.0 / 60.0) > 1.0e-9 ||
        std::abs(rear.simulationTime() - 4.0 / 60.0) > 1.0e-9) {
        std::cerr << "Articulated update did not synchronize accumulated fixed steps\n";
        return 1;
    }
    for (int step = 0; step < 240; ++step) {
        BusSimulation::stepTogether(front, {}, rear, {});
    }
    if (std::abs(front.chassisPose().position[2] - rear.chassisPose().position[2]) > 0.1 ||
        front.chassisPose().position[2] < 0.8 || rear.chassisPose().position[2] < 0.8) {
        std::cerr << "Shared collision dispatch did not keep both sections supported by the ground\n";
        return 1;
    }

    BusConfiguration frontConfiguration = articulatedSectionConfiguration();
    frontConfiguration.articulated = true;
    frontConfiguration.rearCouplingPoint = std::array<double, 3>{0.0, -4.331, 0.315};
    BusConfiguration rearConfiguration = articulatedSectionConfiguration();
    rearConfiguration.articulated = true;
    rearConfiguration.frontCouplingPoint = std::array<double, 3>{0.0, 4.169, 0.315};
    rearConfiguration.frontCouplingCharacter = BusCouplingCharacter{52.5, -20.0, 20.0, 1};
    const VehiclePlacement frontPlacement{{100.0, 0.0, 50.0}, 37.0};
    const VehiclePlacement rearPlacement =
        articulatedRearPlacement(frontConfiguration, frontPlacement, rearConfiguration);
    World couplingWorld(0.0, {}, {}, false);
    BusSimulation couplingFront(couplingWorld, frontConfiguration, frontPlacement);
    BusSimulation couplingRear(couplingWorld, rearConfiguration, rearPlacement);
    ArticulatedBusCoupling coupling =
        BusSimulation::coupleRearSection(couplingFront, couplingRear);
    couplingRear.translateVertically(1.0);
    for (int step = 0; step < 180; ++step) {
        BusSimulation::stepTogether(couplingFront, {}, couplingRear, {});
    }
    const double anchorError =
        distance(couplingAnchor(couplingFront, frontConfiguration,
                                *frontConfiguration.rearCouplingPoint),
                 couplingAnchor(couplingRear, rearConfiguration,
                                *rearConfiguration.frontCouplingPoint));
    if (!std::isfinite(anchorError) || anchorError > 0.05) {
        std::cerr << "Type 1 articulated coupling did not hold the authored anchors together: "
                  << anchorError << " m\n";
        return 1;
    }

    BusConfiguration unsupportedRearConfiguration = rearConfiguration;
    unsupportedRearConfiguration.frontCouplingCharacter->type = 0;
    World unsupportedWorld(0.0, {}, {}, false);
    BusSimulation unsupportedFront(unsupportedWorld, frontConfiguration, frontPlacement);
    BusSimulation unsupportedRear(
        unsupportedWorld, unsupportedRearConfiguration,
        articulatedRearPlacement(frontConfiguration, frontPlacement,
                                 unsupportedRearConfiguration));
    bool unsupportedTypeRejected = false;
    try {
        auto unsupportedCoupling =
            BusSimulation::coupleRearSection(unsupportedFront, unsupportedRear);
        (void)unsupportedCoupling;
    } catch (const std::invalid_argument&) {
        unsupportedTypeRejected = true;
    }
    if (!unsupportedTypeRejected) {
        std::cerr << "Unsupported type 0 truck coupling was not rejected\n";
        return 1;
    }
    return 0;
}
