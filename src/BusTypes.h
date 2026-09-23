#pragma once

#include <array>
#include <cstddef>
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