#pragma once

#include "BusConfiguration.h"

#include <memory>
#include <vector>

namespace openbus::scripting {
class Vehicle;
}

class BusSimulation {
  public:
    explicit BusSimulation(BusConfiguration configuration, VehiclePlacement placement,
                           double physicsHz = 60.0, int maxCatchUpSteps = 8,
                           double groundPlaneZ = 0.0,
                           std::vector<TerrainCollisionGrid> terrain = {},
                           std::vector<StaticCollisionMesh> staticCollision = {});
    ~BusSimulation();

    BusSimulation(const BusSimulation&) = delete;
    BusSimulation& operator=(const BusSimulation&) = delete;

    // Accumulates wall-clock time and advances ODE in fixed-size steps.
    void update(double elapsedSeconds, double throttle, double steering = 0.0, double brake = 0.0);
    // Applies script-produced total wheel torque while retaining ODE tire/contact physics.
    void updateWithWheelTorque(double elapsedSeconds, double wheelTorque, double steering = 0.0,
                               double brake = 0.0);
    // Applies script-produced wheel torque and per-wheel brake forces (N).
    void updateWithWheelTorqueAndBrakeForces(double elapsedSeconds, double wheelTorque,
                                             double steering,
                                             const std::vector<double>& wheelBrakeForces,
                                             const std::vector<double>& wheelSpringFactors = {});
    void step(double throttle, double steering = 0.0, double brake = 0.0);
    void stepWithWheelTorque(double wheelTorque, double steering = 0.0, double brake = 0.0);
    void stepWithWheelTorqueAndBrakeForces(double wheelTorque, double steering,
                                           const std::vector<double>& wheelBrakeForces,
                                           const std::vector<double>& wheelSpringFactors = {});
    void updateVariables(openbus::scripting::Vehicle& variables, double throttle, double steering,
                         double brake) const;
    double positionX() const;
    double positionY() const;
    double positionZ() const;
    std::vector<StaticCollisionMesh> collisionDebugMeshes() const;
    double yaw() const;
    double steeringAngle() const;
    std::array<double, 3> centerOfGravity() const;
    std::array<double, 3> outsideCameraCenter() const;
    BodyPose chassisPose() const;
    ChassisCollisionBox chassisCollisionBox() const;
    BodyPose wheelPose(std::size_t index) const;
    BodyPose wheelMountPose(std::size_t index) const;
    double wheelSteeringAngle(std::size_t index) const;
    double wheelRotation(std::size_t index) const;
    double wheelSuspensionCompression(std::size_t index) const;
    double wheelSuspensionSliderTravel(std::size_t index) const;
    double wheelRadius() const;
    double wheelRadius(std::size_t index) const;
    double wheelHalfWidth() const;
    double speed() const;
    double physicsHz() const;
    int lastStepCount() const;
    bool droppedTime() const;
    std::size_t axleCount() const;
    std::size_t wheelCount() const;
    const std::vector<RoadBump>& roadBumps() const;
    BusAxle axle(std::size_t index) const;
    double wheelLocalZ(std::size_t index) const;
    double simulationTime() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
