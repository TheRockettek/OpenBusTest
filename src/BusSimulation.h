#pragma once

#include "BusConfiguration.h"

#include <memory>

namespace openbus::scripting {
class Vehicle;
}

class BusSimulation {
  public:
    explicit BusSimulation(BusConfiguration configuration, VehiclePlacement placement,
                           double physicsHz = 60.0, int maxCatchUpSteps = 8);
    ~BusSimulation();

    BusSimulation(const BusSimulation&) = delete;
    BusSimulation& operator=(const BusSimulation&) = delete;

    // Accumulates wall-clock time and advances ODE in fixed-size steps.
    void update(double elapsedSeconds, double throttle, double steering = 0.0, double brake = 0.0);
    void step(double throttle, double steering = 0.0, double brake = 0.0);
    void updateVariables(openbus::scripting::Vehicle& variables, double throttle, double steering,
                         double brake) const;
    double positionX() const;
    double positionY() const;
    double positionZ() const;
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
