#include "physics/BusSimulation.h"
#include "physics/World.h"
#include "render/MatrixMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

struct TestBump {
    double centerX;
    double centerY;
    double length;
    double width;
    double height;
};

BusConfiguration twoAxleConfiguration() {
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

BusConfiguration threeAxleConfiguration() {
    BusConfiguration configuration{};
    configuration.mass = 15275.0;
    configuration.length = 13.5;
    configuration.width = 2.5;
    configuration.bodyHalfLength = 6.75;
    configuration.bodyHalfWidth = 1.25;
    configuration.bodyHalfHeight = 1.83;
    configuration.wheelRadius = 0.4755;
    configuration.wheelHalfWidth = 0.145;
    configuration.collisionLength = 13.5;
    configuration.collisionWidth = 2.5;
    configuration.collisionHeight = 3.66;
    // Keep the chassis collider clear of the low terrain humps; wheels still contact them.
    configuration.collisionOffsetZ = 1.4;
    configuration.centerOfGravityHeight = 0.9;
    configuration.inverseMinimumTurnRadius = 0.13;
    configuration.axles = {{4.05, 2.402, 2.402, 1.734, 0.951, 308000.0, 82000.0,
                            20000.0, true, false},
                           {-1.733, 2.402, 2.402, 1.18, 0.951, 314000.0, 115000.0,
                            20000.0, false, true},
                           {-3.406, 2.402, 2.402, 1.18, 0.951, 179000.0, 73000.0,
                            20000.0, false, false}};
    return configuration;
}

std::vector<TestBump> makeBumps() {
    constexpr double roadCenterY = -72.0;
        return {{25.0, roadCenterY, 2.5, 144.0, 0.10},
            {33.0, roadCenterY, 2.5, 144.0, 0.14},
            {41.0, roadCenterY, 2.5, 144.0, 0.08},
            {36.0, roadCenterY + 1.1, 1.8, 1.6, 0.12},
            {40.0, roadCenterY - 1.1, 1.8, 1.6, 0.16},
            {44.0, roadCenterY + 1.1, 1.8, 1.6, 0.10},
            {55.0, roadCenterY, 2.5, 144.0, 0.12},
            {62.0, roadCenterY, 2.5, 144.0, 0.10},
            {110.0, roadCenterY, 2.5, 144.0, 0.12},
            {114.0, roadCenterY, 2.5, 144.0, 0.16},
            {118.0, roadCenterY, 2.5, 144.0, 0.10}};
}

TerrainCollisionGrid makeTerrain() {
    constexpr std::size_t intervals = 288;
    constexpr double tileSize = 144.0;
    constexpr double spacing = tileSize / intervals;
    const std::vector<TestBump> bumps = makeBumps();

    TerrainCollisionGrid terrain;
    terrain.tileX = 0;
    terrain.tileY = -1;
    terrain.tileSizeMeters = tileSize;
    terrain.intervals = intervals;
    terrain.heights.reserve((intervals + 1) * (intervals + 1));
    for (std::size_t row = 0; row <= intervals; ++row) {
        const double y = -tileSize + static_cast<double>(row) * spacing;
        for (std::size_t column = 0; column <= intervals; ++column) {
            const double x = static_cast<double>(column) * spacing;
            double height = 0.0;
            for (const TestBump& bump : bumps) {
                const double xWeight =
                    std::max(0.0, 1.0 - std::abs(x - bump.centerX) / (bump.length * 0.5));
                const double yWeight = bump.width >= tileSize
                                           ? 1.0
                                           : std::max(0.0, 1.0 -
                                                               std::abs(y - bump.centerY) /
                                                                   (bump.width * 0.5));
                height = std::max(height, bump.height * xWeight * yWeight);
            }
            terrain.heights.push_back(static_cast<float>(height));
        }
    }
    return terrain;
}

bool finitePose(const BodyPose& pose) {
    return std::all_of(pose.position.begin(), pose.position.end(),
                       [](double value) { return std::isfinite(value); }) &&
           std::all_of(pose.rotation.begin(), pose.rotation.end(),
                       [](double value) { return std::isfinite(value); });
}

double wheelPivotError(const BusConfiguration& configuration, const BodyPose& chassis,
                       const BodyPose& wheel, std::size_t wheelIndex) {
    const double deltaX = wheel.position[0] - chassis.position[0];
    const double deltaY = wheel.position[1] - chassis.position[1];
    const double deltaZ = wheel.position[2] - chassis.position[2];
    const double localX =
        chassis.rotation[0] * deltaX + chassis.rotation[3] * deltaY + chassis.rotation[6] * deltaZ;
    const double localY =
        chassis.rotation[1] * deltaX + chassis.rotation[4] * deltaY + chassis.rotation[7] * deltaZ;
    const BusAxle& axle = configuration.axles[wheelIndex / 2];
    const double expectedY =
        (wheelIndex % 2 == 0 ? 1.0 : -1.0) * (axle.trackWidth * 0.5 - configuration.wheelHalfWidth);
    return std::hypot(localX - axle.position, localY - expectedY);
}

double wheelRenderCenterError(const BodyPose& chassis, const BodyPose& wheel) {
    const double deltaX = wheel.position[0] - chassis.position[0];
    const double deltaY = wheel.position[1] - chassis.position[1];
    const double deltaZ = wheel.position[2] - chassis.position[2];
    const std::array<double, 3> expectedCenter = {
        chassis.rotation[0] * deltaX + chassis.rotation[3] * deltaY +
            chassis.rotation[6] * deltaZ,
        chassis.rotation[1] * deltaX + chassis.rotation[4] * deltaY +
            chassis.rotation[7] * deltaZ,
        chassis.rotation[2] * deltaX + chassis.rotation[5] * deltaY +
            chassis.rotation[8] * deltaZ + 0.35,
    };
    constexpr std::array<double, 3> modelCenter = {0.13, -0.87, 0.41};
    constexpr double modelOffsetZ = -0.35;
    const openbus::rendering::Matrix4 transform =
        openbus::rendering::makeOdeWheelAnimationTransform(chassis, wheel, modelCenter,
                                                            modelOffsetZ);
    const std::array<double, 4> renderedCenter = openbus::rendering::transformPoint(
        transform, {modelCenter[0], modelCenter[1], modelCenter[2], 1.0});
    return std::hypot(std::hypot(renderedCenter[0] - expectedCenter[0],
                                 renderedCenter[1] - expectedCenter[1]),
                      renderedCenter[2] - expectedCenter[2]);
}

double rotationDelta(const BodyPose& previous, const BodyPose& current) {
    double trace = 0.0;
    for (std::size_t index = 0; index < previous.rotation.size(); ++index) {
        trace += previous.rotation[index] * current.rotation[index];
    }
    return std::acos(std::clamp((trace - 1.0) * 0.5, -1.0, 1.0));
}

struct StressMetrics {
    double maximumChassisSpeed = 0.0;
    double maximumChassisAngularSpeed = 0.0;
    double maximumWheelSpeed = 0.0;
    double maximumWheelAngularSpeed = 0.0;
    double maximumSuspensionTravel = 0.0;
    double maximumPitch = 0.0;
    double maximumRoll = 0.0;
    double settledWheelPivotError = 0.0;
    double maximumWheelPivotError = 0.0;
    double maximumWheelRenderCenterError = 0.0;
    double maximumStraightPairDifference = 0.0;
    double speedAtHighSpeedBump = 0.0;
    double speedAtBrakeStart = 0.0;
    double speedAfterBraking = 0.0;
    double finalX = 0.0;
    double finalY = 0.0;
    double finalZ = 0.0;
    double finalPlanarSpeed = 0.0;
};

bool runStressCase(const std::string& name, BusConfiguration configuration,
                   StressMetrics& metrics) {
    const BusConfiguration alignmentConfiguration = configuration;
    World simulationWorld(0.0, {makeTerrain()});
    BusSimulation simulation(simulationWorld, std::move(configuration), VehiclePlacement{{15.0, -72.0, 0.0}, 0.0}, 60.0, 8);
    constexpr double fixedStep = 1.0 / 60.0;
    BodyPose previousChassis = simulation.chassisPose();
    std::vector<BodyPose> previousWheels;
    std::vector<double> previousWheelRotation;
    for (std::size_t wheel = 0; wheel < simulation.wheelCount(); ++wheel) {
        previousWheels.push_back(simulation.wheelPose(wheel));
        previousWheelRotation.push_back(simulation.wheelRotation(wheel));
    }

    for (int step = 0; step < 240; ++step) {
        simulation.step(0.0, 0.0, 1.0);
    }
    previousChassis = simulation.chassisPose();
    for (std::size_t wheel = 0; wheel < simulation.wheelCount(); ++wheel) {
        previousWheels[wheel] = simulation.wheelPose(wheel);
        previousWheelRotation[wheel] = simulation.wheelRotation(wheel);
    }
    const BodyPose settledPose = simulation.chassisPose();
    if (!finitePose(settledPose) || settledPose.position[2] > 2.0 || simulation.speed() > 1.0) {
        std::cerr << name << ": did not settle onto the flat spawn terrain\n";
        return false;
    }
    for (std::size_t wheel = 0; wheel < simulation.wheelCount(); ++wheel) {
        metrics.settledWheelPivotError = std::max(
            metrics.settledWheelPivotError, wheelPivotError(alignmentConfiguration, settledPose,
                                                            simulation.wheelPose(wheel), wheel));
    }

    bool sawBrakingPhase = false;
    int previousSteeringCommand = 0;
    int steeringReversals = 0;
    int brakingSteps = 0;
    for (int step = 0; step < 1800; ++step) {
        const double x = simulation.positionX();
        double throttle = 1.0;
        double steering = 0.0;
        double brake = 0.0;
        if (x >= 95.0 && x < 105.0) {
            steering = (step / 15) % 2 == 0 ? 1.0 : -1.0;
            const int steeringCommand = steering > 0.0 ? 1 : -1;
            if (previousSteeringCommand != 0 &&
                steeringCommand != previousSteeringCommand) {
                ++steeringReversals;
            }
            previousSteeringCommand = steeringCommand;
        } else if (x >= 105.0) {
            throttle = 0.0;
            brake = 1.0;
            if (!sawBrakingPhase) {
                metrics.speedAtBrakeStart = simulation.speed();
                sawBrakingPhase = true;
            }
            ++brakingSteps;
        }

        simulation.step(throttle, steering, brake);
        const BodyPose chassis = simulation.chassisPose();
        if (!finitePose(chassis) || !std::isfinite(simulation.speed())) {
            std::cerr << name << ": non-finite chassis pose or speed\n";
            return false;
        }
        const double chassisSpeed = std::hypot(
            std::hypot(chassis.position[0] - previousChassis.position[0],
                       chassis.position[1] - previousChassis.position[1]),
            chassis.position[2] - previousChassis.position[2]) /
                                   fixedStep;
        const double chassisAngularSpeed = rotationDelta(previousChassis, chassis) / fixedStep;
        if (!std::isfinite(chassisSpeed) || !std::isfinite(chassisAngularSpeed)) {
            std::cerr << name << ": non-finite derived chassis velocity\n";
            return false;
        }
        metrics.maximumChassisSpeed = std::max(metrics.maximumChassisSpeed, chassisSpeed);
        metrics.maximumChassisAngularSpeed =
            std::max(metrics.maximumChassisAngularSpeed, chassisAngularSpeed);
        metrics.maximumPitch = std::max(
            metrics.maximumPitch, std::abs(std::asin(std::clamp(-chassis.rotation[6], -1.0, 1.0))));
        metrics.maximumRoll = std::max(
            metrics.maximumRoll, std::abs(std::atan2(chassis.rotation[7], chassis.rotation[8])));
        previousChassis = chassis;

        for (std::size_t wheel = 0; wheel < simulation.wheelCount(); ++wheel) {
            const BodyPose pose = simulation.wheelPose(wheel);
            const double wheelRotation = simulation.wheelRotation(wheel);
            const double suspensionTravel = std::abs(simulation.wheelSuspensionSliderTravel(wheel));
            if (!finitePose(pose) || !std::isfinite(wheelRotation) ||
                !std::isfinite(suspensionTravel)) {
                std::cerr << name << ": non-finite wheel pose or telemetry\n";
                return false;
            }
            const double wheelSpeed =
                std::hypot(std::hypot(pose.position[0] - previousWheels[wheel].position[0],
                                      pose.position[1] - previousWheels[wheel].position[1]),
                           pose.position[2] - previousWheels[wheel].position[2]) /
                fixedStep;
            const double wheelAngularSpeed =
                std::abs(wheelRotation - previousWheelRotation[wheel]) / fixedStep;
            if (!std::isfinite(wheelSpeed) || !std::isfinite(wheelAngularSpeed)) {
                std::cerr << name << ": non-finite derived wheel velocity\n";
                return false;
            }
            metrics.maximumWheelSpeed = std::max(metrics.maximumWheelSpeed, wheelSpeed);
            metrics.maximumWheelAngularSpeed =
                std::max(metrics.maximumWheelAngularSpeed, wheelAngularSpeed);
            metrics.maximumSuspensionTravel =
                std::max(metrics.maximumSuspensionTravel, suspensionTravel);
            metrics.maximumWheelPivotError = std::max(
                metrics.maximumWheelPivotError,
                wheelPivotError(alignmentConfiguration, chassis, pose, wheel));
            const double renderCenterError = wheelRenderCenterError(chassis, pose);
            if (!std::isfinite(renderCenterError)) {
                std::cerr << name << ": non-finite rendered wheel center\n";
                return false;
            }
            metrics.maximumWheelRenderCenterError =
                std::max(metrics.maximumWheelRenderCenterError, renderCenterError);
            previousWheels[wheel] = pose;
            previousWheelRotation[wheel] = wheelRotation;
        }

        if (x < 30.0) {
            for (std::size_t axle = 0; axle < simulation.axleCount(); ++axle) {
                const double difference = std::abs(
                    simulation.wheelSuspensionSliderTravel(axle * 2) -
                    simulation.wheelSuspensionSliderTravel(axle * 2 + 1));
                metrics.maximumStraightPairDifference =
                    std::max(metrics.maximumStraightPairDifference, difference);
            }
        }
        if (x >= 53.0 && x <= 58.0) {
            metrics.speedAtHighSpeedBump =
                std::max(metrics.speedAtHighSpeedBump, simulation.speed());
        }
        if (brakingSteps == 240) {
            metrics.speedAfterBraking = simulation.speed();
        }
        if (brakingSteps >= 240 && simulation.speed() < 0.5) {
            break;
        }
    }

    metrics.finalX = simulation.positionX();
    metrics.finalY = simulation.positionY();
    metrics.finalZ = simulation.positionZ();
    metrics.finalPlanarSpeed = simulation.speed();
    std::cout << name << " maxSpeed=" << metrics.maximumChassisSpeed
              << " maxAngularSpeed=" << metrics.maximumChassisAngularSpeed
              << " maxWheelSpeed=" << metrics.maximumWheelSpeed
              << " maxWheelSpin=" << metrics.maximumWheelAngularSpeed
              << " maxSuspensionTravel=" << metrics.maximumSuspensionTravel
              << " maxPitch=" << metrics.maximumPitch << " maxRoll=" << metrics.maximumRoll
              << " settledWheelPivotError=" << metrics.settledWheelPivotError
              << " maxWheelPivotError=" << metrics.maximumWheelPivotError
              << " maxWheelRenderCenterError=" << metrics.maximumWheelRenderCenterError
              << " straightPairDifference=" << metrics.maximumStraightPairDifference
              << " speedAtBump=" << metrics.speedAtHighSpeedBump
              << " finalPosition=" << metrics.finalX << ',' << metrics.finalY << ','
              << metrics.finalZ << " finalPlanarSpeed=" << metrics.finalPlanarSpeed
              << " brake=" << metrics.speedAtBrakeStart << "->" << metrics.speedAfterBraking
              << '\n';

    if (steeringReversals == 0 || !sawBrakingPhase || brakingSteps < 240 ||
        metrics.maximumPitch < 0.001 || metrics.maximumRoll < 0.001 ||
        metrics.maximumSuspensionTravel < 0.01 || metrics.settledWheelPivotError > 0.02 ||
        metrics.maximumChassisSpeed > 45.0 || metrics.maximumChassisAngularSpeed > 2.5 ||
        metrics.maximumWheelSpeed > 60.0 || metrics.maximumWheelAngularSpeed > 120.0 ||
        metrics.maximumSuspensionTravel > 0.28 || metrics.maximumWheelPivotError > 0.15 ||
        metrics.maximumWheelRenderCenterError > 1.0e-9 ||
        metrics.maximumStraightPairDifference > 0.12 || metrics.speedAtHighSpeedBump < 12.0 ||
        metrics.speedAtBrakeStart <= 1.0 ||
        metrics.speedAfterBraking >= metrics.speedAtBrakeStart * 0.75) {
        std::cerr << name << ": deterministic bump/steering/braking envelope exceeded\n";
        return false;
    }
    return true;
}

bool sameMetrics(const StressMetrics& first, const StressMetrics& second) {
    constexpr double tolerance = 1.0e-9;
    const std::array<double, 18> firstValues = {
        first.maximumChassisSpeed,
        first.maximumChassisAngularSpeed,
        first.maximumWheelSpeed,
        first.maximumWheelAngularSpeed,
        first.maximumSuspensionTravel,
        first.maximumPitch,
        first.maximumRoll,
        first.settledWheelPivotError,
        first.maximumWheelPivotError,
        first.maximumWheelRenderCenterError,
        first.maximumStraightPairDifference,
        first.speedAtHighSpeedBump,
        first.speedAtBrakeStart,
        first.speedAfterBraking,
        first.finalX,
        first.finalY,
        first.finalZ,
        first.finalPlanarSpeed,
    };
    const std::array<double, firstValues.size()> secondValues = {
        second.maximumChassisSpeed,
        second.maximumChassisAngularSpeed,
        second.maximumWheelSpeed,
        second.maximumWheelAngularSpeed,
        second.maximumSuspensionTravel,
        second.maximumPitch,
        second.maximumRoll,
        second.settledWheelPivotError,
        second.maximumWheelPivotError,
        second.maximumWheelRenderCenterError,
        second.maximumStraightPairDifference,
        second.speedAtHighSpeedBump,
        second.speedAtBrakeStart,
        second.speedAfterBraking,
        second.finalX,
        second.finalY,
        second.finalZ,
        second.finalPlanarSpeed,
    };
    for (std::size_t index = 0; index < firstValues.size(); ++index) {
        if (std::abs(firstValues[index] - secondValues[index]) > tolerance) {
            return false;
        }
    }
    return true;
}

} // namespace

int main() {
    StressMetrics twoAxleMetrics;
    StressMetrics threeAxleMetrics;
    StressMetrics repeatedTwoAxleMetrics;
    StressMetrics repeatedThreeAxleMetrics;
    if (!runStressCase("two-axle", twoAxleConfiguration(), twoAxleMetrics) ||
        !runStressCase("three-axle", threeAxleConfiguration(), threeAxleMetrics) ||
        !runStressCase("two-axle-repeat", twoAxleConfiguration(), repeatedTwoAxleMetrics) ||
        !runStressCase("three-axle-repeat", threeAxleConfiguration(),
                       repeatedThreeAxleMetrics)) {
        return 1;
    }
    if (!sameMetrics(twoAxleMetrics, repeatedTwoAxleMetrics) ||
        !sameMetrics(threeAxleMetrics, repeatedThreeAxleMetrics)) {
        std::cerr << "stress probe produced different metrics on repeated runs\n";
        return 1;
    }
    return 0;
}