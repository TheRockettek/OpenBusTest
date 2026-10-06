#include "BusSimulation.h"

#include <cmath>
#include <iostream>
#include <string>

namespace {

BusConfiguration caetanoConfiguration(bool useConfiguredInertia) {
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
    configuration.collisionOffsetX = -0.05;
    configuration.collisionOffsetZ = 0.8;
    configuration.centerOfGravityHeight = 1.3;
    configuration.inverseMinimumTurnRadius = 0.16812;
    if (useConfiguredInertia) {
        configuration.momentOfInertia = {300.0, 80.0, 300.0};
    }
    configuration.axles = {{3.2921, 2.4, 2.4, 1.85, 0.9565, 240000.0, 90000.0,
                            20000.0, true, false},
                           {-3.0008, 2.45, 2.45, 1.25, 0.9565, 280000.0, 116000.0,
                            20000.0, false, true}};
    return configuration;
}

double rollAngle(const BodyPose& pose) {
    return std::atan2(pose.rotation[7], pose.rotation[8]);
}

double pitchAngle(const BodyPose& pose) {
    return std::atan2(-pose.rotation[6],
                      std::sqrt(pose.rotation[0] * pose.rotation[0] +
                                pose.rotation[3] * pose.rotation[3]));
}

} // namespace

int main(int argc, char** argv) {
    TerrainCollisionGrid terrain;
    terrain.tileX = -1;
    terrain.tileY = 2;
    terrain.intervals = 2;
    terrain.heights = {3.0F, 3.5F, 4.0F, 3.0F, 3.5F, 4.0F, 3.0F, 3.5F, 4.0F};
    BusSimulation terrainSimulation(caetanoConfiguration(true),
                                    VehiclePlacement{{-150.0, 750.0, 3.5}, 0.0}, 60.0, 8, 0.0,
                                    {terrain});
    const std::vector<StaticCollisionMesh> terrainDebugMeshes =
        terrainSimulation.collisionDebugMeshes();
    if (terrainDebugMeshes.size() != 1 || terrainDebugMeshes[0].vertices.size() != 27 ||
        terrainDebugMeshes[0].indices.size() != 24 ||
        terrainDebugMeshes[0].vertices[0] != -300.0 ||
        terrainDebugMeshes[0].vertices[1] != 600.0 ||
        terrainDebugMeshes[0].vertices[2] != 3.0) {
        std::cerr << "Terrain collision debug mesh did not expose the active ODE geometry\n";
        return 1;
    }
    for (int step = 0; step < 600; ++step) {
        terrainSimulation.step(0.0, 0.0, 1.0);
    }
    if (std::abs(terrainSimulation.chassisPose().position[2] - 4.8) > 0.25) {
        std::cerr << "Bus did not settle on the nonzero map terrain collision surface\n";
        return 1;
    }

    StaticCollisionMesh raisedRoad;
    raisedRoad.vertices = {-162.0, 735.0, 4.0, -138.0, 735.0, 4.0,
                           -138.0, 765.0, 4.0, -162.0, 765.0, 4.0};
    raisedRoad.indices = {0, 1, 2, 0, 2, 3};
    BusSimulation roadSimulation(caetanoConfiguration(true),
                                 VehiclePlacement{{-150.0, 750.0, 4.0}, 0.0}, 60.0, 8, 0.0,
                                 {terrain}, {raisedRoad});
    const std::vector<StaticCollisionMesh> roadDebugMeshes = roadSimulation.collisionDebugMeshes();
    if (roadDebugMeshes.size() != 2 || roadDebugMeshes[1].vertices != raisedRoad.vertices ||
        roadDebugMeshes[1].indices != raisedRoad.indices) {
        std::cerr << "Map road collision debug mesh did not expose the active ODE geometry\n";
        return 1;
    }
    for (int step = 0; step < 600; ++step) {
        roadSimulation.step(0.0, 0.0, 1.0);
    }
    if (std::abs(roadSimulation.chassisPose().position[2] - 5.3) > 0.3) {
        std::cerr << "Bus did not settle on the raised static map road above terrain\n";
        return 1;
    }

    BusConfiguration meshCollisionConfiguration = caetanoConfiguration(true);
    meshCollisionConfiguration.hasCollisionMesh = true;
    meshCollisionConfiguration.collisionMeshVertices = {-1.0, -1.0, 0.0,
                                                         1.0, -1.0, 0.0,
                                                         0.0, 1.0, 0.0};
    meshCollisionConfiguration.collisionMeshIndices = {0, 1, 2};
    BusSimulation meshCollisionSimulation(meshCollisionConfiguration,
                                           VehiclePlacement{{-1000.0, 0.0, 0.0}, 0.0});
    const ChassisCollisionBox meshCollision = meshCollisionSimulation.chassisCollisionBox();
    if (!meshCollision.enabled || !meshCollision.mesh) {
        std::cerr << "Configured triangle mesh was not selected as the chassis collider\n";
        return 1;
    }

    const bool useConfiguredInertia = argc <= 1 || std::string(argv[1]) != "box";
    const int steeringArgument = useConfiguredInertia ? 1 : 2;
    const double steeringCommand =
        argc > steeringArgument ? std::stod(argv[steeringArgument]) : 1.0;
    BusSimulation simulation(caetanoConfiguration(useConfiguredInertia),
                             VehiclePlacement{{-1000.0, 0.0, 0.0}, 0.0});
    BusSimulation scriptTorqueSimulation(caetanoConfiguration(useConfiguredInertia),
                                          VehiclePlacement{{-1000.0, 0.0, 0.0}, 0.0});
    const double scriptTorqueStartX = scriptTorqueSimulation.positionX();
    for (int step = 0; step < 600; ++step) {
        scriptTorqueSimulation.stepWithWheelTorque(25000.0, 0.0, 0.0);
    }
    if (scriptTorqueSimulation.positionX() - scriptTorqueStartX < 0.5) {
        std::cerr << "Script-produced wheel torque did not move the vehicle\n";
        return 1;
    }
    double settledRoll = 0.0;
    double settledPitch = 0.0;
    double straightRoll = 0.0;
    double straightPitch = 0.0;
    double turnRoll = 0.0;
    double turnPitch = 0.0;
    double maximumSpeed = 0.0;
    for (int step = 0; step < 600; ++step) {
        simulation.step(0.0, 0.0, 1.0);
        settledRoll = std::max(settledRoll, std::abs(rollAngle(simulation.chassisPose())));
        settledPitch = std::max(settledPitch, std::abs(pitchAngle(simulation.chassisPose())));
    }
    for (int step = 0; step < 900; ++step) {
        simulation.step(1.0, 0.0, 0.0);
        straightRoll = std::max(straightRoll, std::abs(rollAngle(simulation.chassisPose())));
        straightPitch = std::max(straightPitch, std::abs(pitchAngle(simulation.chassisPose())));
        maximumSpeed = std::max(maximumSpeed, simulation.speed());
    }
    for (int step = 0; step < 300; ++step) {
        simulation.step(0.0, steeringCommand, 0.0);
        turnRoll = std::max(turnRoll, std::abs(rollAngle(simulation.chassisPose())));
        turnPitch = std::max(turnPitch, std::abs(pitchAngle(simulation.chassisPose())));
        maximumSpeed = std::max(maximumSpeed, simulation.speed());
    }
    const BodyPose finalPose = simulation.chassisPose();
    std::cout << "inertia=" << (useConfiguredInertia ? "configured" : "box")
              << " maximumSpeed=" << maximumSpeed
              << " settledRoll=" << settledRoll << " settledPitch=" << settledPitch
              << " straightRoll=" << straightRoll << " straightPitch=" << straightPitch
              << " turnRoll=" << turnRoll << " turnPitch=" << turnPitch
              << " finalZ=" << finalPose.position[2] << " finalPitch=" << pitchAngle(finalPose)
              << '\n';
    if (maximumSpeed < 20.0 || turnRoll > 0.2 || std::abs(finalPose.position[2] - 1.3) > 0.25) {
        std::cerr << "Flat-ground stability exceeded the expected envelope\n";
        return 1;
    }
    return 0;
}