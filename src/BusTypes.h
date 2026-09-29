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
using VehicleType = BusVehicle;

struct BusAxle {
    double position;
    double trackWidth;
    double maxWidth;
    double minWidth;
    double wheelDiameter;
    double springRate;
    double maxForce;
    double damperRate;
    bool steerable;
    bool driven;
};

struct BusConfiguration {
    double mass;
    double length;
    double width;
    double bodyHalfLength;
    double bodyHalfWidth;
    double bodyHalfHeight;
    double wheelRadius;
    double wheelHalfWidth;
    double collisionLength;
    double collisionWidth;
    double collisionHeight;
    double collisionOffsetX;
    double collisionOffsetY;
    double collisionOffsetZ;
    std::array<double, 3> outsideCameraCenter = {};
    bool hasOutsideCameraCenter = false;
    double centerOfGravityHeight;
    bool articulated;
    std::vector<BusAxle> axles;
    double inverseMinimumTurnRadius = 0.0;
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
    double offsetX;
    double offsetY;
    double offsetZ;
};
using Axle = BusAxle;
using VehicleConfiguration = BusConfiguration;