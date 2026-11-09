#pragma once

#include "physics/BusConfiguration.h"
#include "physics/World.h"
#include "core/WeatherState.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace openbus::scripting {
class Vehicle;
}

struct WheelStaticContactSettings {
    // A lower ERP and higher CFM make wheel contact with static map surfaces more compliant.
    double erp = 0.65;
    double cfm = 1.0e-5;
};

struct BusCollisionEvent {
    std::array<double, 3> position;
    double energy = 0.0;
};

struct BusPassengerMassPoint {
    std::size_t seatIndex = 0;
    double massKg = 0.0;
    // Position in chassis-local coordinates, relative to the chassis body origin.
    std::array<double, 3> localPosition = {};
};

struct BusStepInput {
    // Engine throttle when scriptDriven is false; wheel torque when true.
    double driveCommand = 0.0;
    double steering = 0.0;
    std::vector<double> wheelBrakeForces;
    std::vector<double> wheelSpringFactors;
    bool scriptDriven = false;
};

class ArticulatedBusCoupling;
class ArticulatedBusCouplingState;

class BusSimulation {
  public:
    explicit BusSimulation(World& world, BusConfiguration configuration, VehiclePlacement placement,
                           double physicsHz = 60.0, int maxCatchUpSteps = 8,
                           WheelStaticContactSettings wheelStaticContact = {});
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
    // Advances two simulations by one fixed step in their shared physics world.
    static void stepTogether(BusSimulation& first, const BusStepInput& firstInput,
                             BusSimulation& second, const BusStepInput& secondInput);
    // Accumulates elapsed time and advances both simulations in synchronized fixed steps.
    static void updateTogether(BusSimulation& first, const BusStepInput& firstInput,
                               BusSimulation& second, const BusStepInput& secondInput,
                               double elapsedSeconds);
    static ArticulatedBusCoupling coupleRearSection(BusSimulation& front, BusSimulation& rear);
    void setFrozen(bool frozen);
    void setRoadWeather(const openbus::RoadWeatherConditions& conditions);
    void setPassengerMassPoints(const std::vector<BusPassengerMassPoint>& points);
    double totalMassKg() const;
    void updateVariables(openbus::scripting::Vehicle& variables, double throttle, double steering,
                         double brake, std::size_t axleIndexOffset = 0) const;
    double positionX() const;
    double positionY() const;
    double positionZ() const;
    void translateVertically(double meters);
    double yaw() const;
    double steeringAngle() const;
    std::array<double, 3> centerOfGravity() const;
    std::array<double, 3> outsideCameraCenter() const;
    BodyPose chassisPose() const;
    std::optional<BusCollisionEvent> consumeCollisionEvent();
    std::optional<double> heightAbovePoint(double x, double y, double z,
                                           double modelOffsetZ = 0.0) const;
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
    static void validatePair(const BusSimulation& first, const BusSimulation& second);
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class ArticulatedBusCoupling {
  public:
    ArticulatedBusCoupling(const ArticulatedBusCoupling&) = delete;
    ArticulatedBusCoupling& operator=(const ArticulatedBusCoupling&) = delete;
    ArticulatedBusCoupling(ArticulatedBusCoupling&& other) noexcept;
    ArticulatedBusCoupling& operator=(ArticulatedBusCoupling&& other) noexcept;
    ~ArticulatedBusCoupling();

  private:
    explicit ArticulatedBusCoupling(std::shared_ptr<ArticulatedBusCouplingState> state);
    std::shared_ptr<ArticulatedBusCouplingState> state_;
    friend class BusSimulation;
};
