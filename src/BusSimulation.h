#pragma once

#include <array>
#include <ostream>
#include <string>
#include <vector>

struct KeyEvent {
    std::string key;
    bool pressed;
    double wallTimeSeconds;
};

enum class BusVehicle {
    ManDl05,
    SpE400Mmc,
};

struct BusAxle {
    double position;
    double trackWidth;
    double maxWidth = 0.0;
    double minWidth = 0.0;
    double wheelDiameter = 0.0;
    double springRate = 250000.0;
    double maxForce = 0.0;
    double damperRate = 16000.0;
    bool steerable;
    bool driven;
};

struct BusConfiguration {
    double mass = 11500.0;
    double length = 12.0;
    double width = 2.55;
    double bodyHalfLength = 5.60;
    double bodyHalfWidth = 1.15;
    double bodyHalfHeight = 1.35;
    double wheelRadius = 0.505;
    double wheelHalfWidth = 0.145;
    double collisionLength = 13.56;
    double collisionWidth = 2.52;
    double collisionHeight = 3.75;
    double collisionOffsetZ = 1.253;
    bool articulated = false;
    std::vector<BusAxle> axles;

    static BusConfiguration lionCity12();
    static BusConfiguration manDl05();
    static BusConfiguration spE400Mmc();
};

BusVehicle busVehicleFromEnvironment();
BusConfiguration busConfigurationFor(BusVehicle vehicle);

enum class RoadFeatureType {
    Hump,
    Incline,
    Bridge,
    Barrier,
};

struct RoadBump {
    double centerX;
    double centerY;
    double length;
    double width;
    double height;
    RoadFeatureType type = RoadFeatureType::Hump;
    double flatLength = 0.0;
    double railHeight = 0.0;
    double railWidth = 0.0;
};

struct BodyPose {
    std::array<double, 3> position;
    std::array<double, 9> rotation;
};

struct ChassisCollisionBox {
    double length;
    double width;
    double height;
    double offsetZ;
};

double roadFeatureHeightAt(const RoadBump& feature, double localX);

class BusSimulation {
  public:
    explicit BusSimulation(BusConfiguration configuration = BusConfiguration::manDl05(),
                           double physicsHz = 60.0, int maxCatchUpSteps = 8);
    ~BusSimulation();

    BusSimulation(const BusSimulation&) = delete;
    BusSimulation& operator=(const BusSimulation&) = delete;

    // Accumulates wall-clock time and advances ODE in fixed-size steps.
    void update(double elapsedSeconds, double throttle, double steering = 0.0, double brake = 0.0);
    void step(double seconds, double throttle, double steering = 0.0, double brake = 0.0);
    double positionX() const;
    double positionY() const;
    double positionZ() const;
    double yaw() const;
    double steeringAngle() const;
    std::array<double, 3> centerOfGravity() const;
    BodyPose chassisPose() const;
    ChassisCollisionBox chassisCollisionBox() const;
    BodyPose wheelPose(std::size_t index) const;
    double wheelRadius() const;
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
    void writeDiagnosticsJson(std::ostream& output, bool firstSample,
                              const std::vector<KeyEvent>& keyEvents) const;
    void writeWheelRotationLog(std::ostream& output) const;

  private:
    struct Impl;
    Impl* impl_;
};
