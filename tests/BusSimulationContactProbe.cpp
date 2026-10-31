#include "physics/BusSimulation.h"
#include "physics/World.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

BusConfiguration testConfiguration() {
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

StaticCollisionMesh makeRaisedMarking() {
    // A 5 mm raised, transverse strip, contacted by the positive-Y wheels only.
    constexpr double centerX = 16.0;
    constexpr double halfLength = 0.075;
    constexpr double minY = 0.0;
    constexpr double maxY = 8.0;
    constexpr double height = 0.005;
    StaticCollisionMesh marking;
    marking.vertices = {centerX - halfLength, minY, height,
                        centerX + halfLength, minY, height,
                        centerX + halfLength, maxY, height,
                        centerX - halfLength, maxY, height};
    marking.indices = {0, 1, 2, 0, 2, 3};
    return marking;
}

StaticCollisionMesh makeCurb() {
    constexpr double minX = 16.0;
    constexpr double maxX = 16.5;
    constexpr double minY = 0.0;
    constexpr double maxY = 8.0;
    constexpr double height = 0.25;
    StaticCollisionMesh curb;
    curb.vertices = {minX, minY, height, maxX, minY, height,
                     maxX, maxY, height, minX, maxY, height};
    curb.indices = {0, 1, 2, 0, 2, 3};
    return curb;
}

void checkTessellatedRoadContact(double spacing, bool rough) {
    constexpr int width = 8;
    constexpr int length = 32;
    const int columns = static_cast<int>(2.0 * width / spacing);
    const int rows = static_cast<int>(2.0 * length / spacing);
    StaticCollisionMesh road;
    for (int row = 0; row <= rows; ++row) {
        for (int column = 0; column <= columns; ++column) {
            const double height =
                rough ? 0.01 * std::sin(row * 0.71) * std::cos(column * 0.63) : 0.0;
            road.vertices.insert(road.vertices.end(),
                                 {-length + row * spacing, -width + column * spacing, height});
        }
    }
    for (int row = 0; row < rows; ++row) {
        for (int column = 0; column < columns; ++column) {
            const int a = row * (columns + 1) + column;
            const int b = a + columns + 1;
            road.indices.insert(road.indices.end(), {a, b, b + 1, a, b + 1, a + 1});
        }
    }
    World world(0.0, {}, {std::move(road)}, false);
    BusSimulation simulation(world, testConfiguration(), VehiclePlacement{{0.0, 0.0, 0.0}, 0.0});
    for (int step = 0; step < 480; ++step) {
        simulation.step(step < 300 ? 0.0 : 0.4, 0.0, step < 300 ? 1.0 : 0.0);
        if (step < 180) {
            continue;
        }
        for (std::size_t wheel = 0; wheel < simulation.wheelCount(); ++wheel) {
            const double height = simulation.wheelPose(wheel).position[2];
            if (!std::isfinite(height) || height < 0.38 || height > 0.56) {
                throw std::runtime_error("wheel left tessellated road: spacing=" +
                                         std::to_string(spacing) + ", height=" +
                                         std::to_string(height));
            }
        }
    }
    if (simulation.positionX() < 3.0) {
        throw std::runtime_error("bus did not traverse the tessellated road");
    }
}

void checkFastTessellatedRoadContact(double phase) {
    constexpr double spacing = 0.125;
    constexpr int rows = 1600;
    constexpr int columns = 128;
    StaticCollisionMesh road;
    road.vertices.reserve(static_cast<std::size_t>(rows + 1) *
                          static_cast<std::size_t>(columns + 1) * 3);
    for (int row = 0; row <= rows; ++row) {
        for (int column = 0; column <= columns; ++column) {
            const double height =
                0.01 * std::sin(row * 0.71 + phase) * std::cos(column * 0.63 + phase);
            road.vertices.insert(road.vertices.end(),
                                 {-10.0 + row * spacing, -8.0 + column * spacing, height});
        }
    }
    road.indices.reserve(static_cast<std::size_t>(rows) *
                         static_cast<std::size_t>(columns) * 6);
    for (int row = 0; row < rows; ++row) {
        for (int column = 0; column < columns; ++column) {
            const int a = row * (columns + 1) + column;
            const int b = a + columns + 1;
            road.indices.insert(road.indices.end(), {a, b, b + 1, a, b + 1, a + 1});
        }
    }

    World world(0.0, {}, {std::move(road)}, false);
    BusSimulation simulation(world, testConfiguration(), VehiclePlacement{{0.0, 0.0, 0.0}, 0.0});
    for (int step = 0; step < 300; ++step) {
        simulation.step(0.0, 0.0, 1.0);
    }

    double maximumSpeed = 0.0;
    double minimumWheelBottom = std::numeric_limits<double>::infinity();
    double maximumWheelBottom = -std::numeric_limits<double>::infinity();
    double maximumWheelVerticalSpeed = 0.0;
    std::array<double, 4> previousWheelHeight{};
    for (std::size_t wheel = 0; wheel < previousWheelHeight.size(); ++wheel) {
        previousWheelHeight[wheel] = simulation.wheelPose(wheel).position[2];
    }
    int highSpeedSamples = 0;
    for (int step = 0; step < 600; ++step) {
        const double driveTorque = simulation.speed() < 12.0 ? 12000.0 : 0.0;
        simulation.stepWithWheelTorque(driveTorque, 0.0, 0.0);
        maximumSpeed = std::max(maximumSpeed, simulation.speed());
        highSpeedSamples += simulation.speed() >= 40.0 / 3.6 ? 1 : 0;
        for (std::size_t wheel = 0; wheel < simulation.wheelCount(); ++wheel) {
            const auto pose = simulation.wheelPose(wheel);
            if (!std::isfinite(pose.position[0]) || !std::isfinite(pose.position[1]) ||
                !std::isfinite(pose.position[2]) || pose.position[0] < -9.0 ||
                pose.position[0] > 189.0 ||
                std::abs(pose.position[1]) > 7.0) {
                throw std::runtime_error("wheel left the fast-road test surface");
            }
            const double bottom = pose.position[2] - simulation.wheelRadius(wheel);
            minimumWheelBottom = std::min(minimumWheelBottom, bottom);
            maximumWheelBottom = std::max(maximumWheelBottom, bottom);
            maximumWheelVerticalSpeed = std::max(
                maximumWheelVerticalSpeed,
                std::abs(pose.position[2] - previousWheelHeight[wheel]) * 60.0);
            previousWheelHeight[wheel] = pose.position[2];
        }
    }
    std::cout << "dense road: max speed=" << maximumSpeed
              << ", high-speed samples=" << highSpeedSamples
              << ", wheel bottom range=" << minimumWheelBottom << " to " << maximumWheelBottom
              << ", max wheel vertical speed=" << maximumWheelVerticalSpeed << '\n';
    if (maximumSpeed < 40.0 / 3.6 || highSpeedSamples < 30 ||
        minimumWheelBottom < -0.06 || maximumWheelBottom > 0.08 ||
        maximumWheelVerticalSpeed > 5.0) {
        throw std::runtime_error("wheel lost support on dense road at speed: maximum speed=" +
                                 std::to_string(maximumSpeed) +
                                 " m/s, high-speed samples=" +
                                 std::to_string(highSpeedSamples) +
                                 ", minimum wheel bottom=" +
                                 std::to_string(minimumWheelBottom) +
                                 " m, maximum wheel bottom=" +
                                 std::to_string(maximumWheelBottom) +
                                 " m, maximum wheel vertical speed=" +
                                 std::to_string(maximumWheelVerticalSpeed) + " m/s");
    }
}

struct ContactMetrics {
    double maximumYawRate = 0.0;
    double maximumSteeringHingeAngle = 0.0;
    double maximumLeftRightSuspensionDifference = 0.0;
    double maximumPerStepSuspensionChange = 0.0;
    double finalX = 0.0;
};

ContactMetrics runCrossing(WheelStaticContactSettings settings) {
    World world(0.0, {}, {makeRaisedMarking()});
    BusSimulation simulation(world, testConfiguration(),
                             VehiclePlacement{{0.0, 0.0, 0.0}, 0.0}, 60.0, 8, settings);
    constexpr double fixedStep = 1.0 / 60.0;
    for (int step = 0; step < 300; ++step) {
        simulation.step(0.0, 0.0, 1.0);
    }

    ContactMetrics metrics;
    double previousYaw = simulation.yaw();
    std::array<double, 4> previousTravel{};
    for (std::size_t wheel = 0; wheel < previousTravel.size(); ++wheel) {
        previousTravel[wheel] = simulation.wheelSuspensionSliderTravel(wheel);
    }
    for (int step = 0; step < 900; ++step) {
        simulation.step(0.55, 0.0, 0.0);
        const double x = simulation.positionX();
        const double yaw = simulation.yaw();
        const double yawRate = std::remainder(yaw - previousYaw, 2.0 * std::acos(-1.0)) /
                               fixedStep;
        previousYaw = yaw;
        if (!std::isfinite(yawRate) || !std::isfinite(x)) {
            throw std::runtime_error("non-finite vehicle response during marking traversal");
        }
        if (x >= 12.0 && x <= 21.0) {
            metrics.maximumYawRate = std::max(metrics.maximumYawRate, std::abs(yawRate));
            for (std::size_t wheel = 0; wheel < previousTravel.size(); ++wheel) {
                const double travel = simulation.wheelSuspensionSliderTravel(wheel);
                if (!std::isfinite(travel)) {
                    throw std::runtime_error("non-finite suspension travel during marking traversal");
                }
                metrics.maximumPerStepSuspensionChange = std::max(
                    metrics.maximumPerStepSuspensionChange,
                    std::abs(travel - previousTravel[wheel]) / fixedStep);
                previousTravel[wheel] = travel;
            }
            metrics.maximumSteeringHingeAngle =
                std::max({metrics.maximumSteeringHingeAngle,
                          std::abs(simulation.wheelSteeringAngle(0)),
                          std::abs(simulation.wheelSteeringAngle(1))});
            metrics.maximumLeftRightSuspensionDifference = std::max(
                metrics.maximumLeftRightSuspensionDifference,
                std::abs(simulation.wheelSuspensionSliderTravel(0) -
                         simulation.wheelSuspensionSliderTravel(1)));
        } else {
            for (std::size_t wheel = 0; wheel < previousTravel.size(); ++wheel) {
                previousTravel[wheel] = simulation.wheelSuspensionSliderTravel(wheel);
            }
        }
    }
    metrics.finalX = simulation.positionX();
    return metrics;
}

ContactMetrics runCurbCrossing(WheelStaticContactSettings settings) {
    World world(0.0, {}, {makeCurb()});
    BusSimulation simulation(world, testConfiguration(),
                             VehiclePlacement{{0.0, 0.0, 0.0}, 0.0}, 60.0, 8, settings);
    for (int step = 0; step < 300; ++step) {
        simulation.step(0.0, 0.0, 1.0);
    }

    ContactMetrics metrics;
    double previousYaw = simulation.yaw();
    std::array<double, 4> previousTravel{};
    for (std::size_t wheel = 0; wheel < previousTravel.size(); ++wheel) {
        previousTravel[wheel] = simulation.wheelSuspensionSliderTravel(wheel);
    }
    for (int step = 0; step < 900; ++step) {
        simulation.step(0.25, 0.0, 0.0);
        const double x = simulation.positionX();
        const double yaw = simulation.yaw();
        const double yawRate = std::remainder(yaw - previousYaw, 2.0 * std::acos(-1.0)) /
                               (1.0 / 60.0);
        previousYaw = yaw;
        if (x >= 12.0 && x <= 23.0) {
            metrics.maximumYawRate = std::max(metrics.maximumYawRate, std::abs(yawRate));
            for (std::size_t wheel = 0; wheel < previousTravel.size(); ++wheel) {
                const double travel = simulation.wheelSuspensionSliderTravel(wheel);
                metrics.maximumPerStepSuspensionChange = std::max(
                    metrics.maximumPerStepSuspensionChange,
                    std::abs(travel - previousTravel[wheel]) * 60.0);
                previousTravel[wheel] = travel;
            }
            metrics.maximumLeftRightSuspensionDifference = std::max(
                metrics.maximumLeftRightSuspensionDifference,
                std::abs(simulation.wheelSuspensionSliderTravel(0) -
                         simulation.wheelSuspensionSliderTravel(1)));
        } else {
            for (std::size_t wheel = 0; wheel < previousTravel.size(); ++wheel) {
                previousTravel[wheel] = simulation.wheelSuspensionSliderTravel(wheel);
            }
        }
    }
    metrics.finalX = simulation.positionX();
    return metrics;
}

struct PoleImpactResult {
    double initialAngleRadians = 0.0;
    bool contactAtFirstBend = false;
    MapPoleState finalState;
};

void checkOverlappingPolesDoNotReportBusContact() {
    World world;
    BusSimulation simulation(world, testConfiguration(), VehiclePlacement{{0.0, 0.0, 0.0}, 0.0});
    MapPoleCollision pole;
    pole.mapObjectId = 101;
    pole.basePosition = {20.0, 0.0, 0.0};
    pole.topPosition = {20.0, 0.0, 4.0};
    pole.radiusMeters = 0.25;
    MapPoleCollision overlappingPole = pole;
    overlappingPole.mapObjectId = 102;
    world.addMapTileCollision(0, 0, std::nullopt, {}, {pole, overlappingPole});

    std::array<dGeomID, 2> poleGeometries{};
    std::size_t count = 0;
    const dSpaceID space = world.collisionSpace();
    for (int index = 0; index < dSpaceGetNumGeoms(space); ++index) {
        const dGeomID geom = dSpaceGetGeom(space, index);
        if (world.isMapPoleGeometry(geom) && count < poleGeometries.size()) {
            poleGeometries[count++] = geom;
        }
    }
    dContactGeom contact{};
    if (count != 2 || dCollide(poleGeometries[0], poleGeometries[1], 1, &contact,
                               sizeof(contact)) == 0) {
        throw std::runtime_error("overlapping-pole fixture did not produce an ODE contact");
    }
    simulation.step(0.0, 0.0, 1.0);
    const auto states = world.mapPoleStates();
    if (states.size() != 2 || states[0].touchingBus || states[1].touchingBus ||
        states[0].angleRadians != 0.0 || states[1].angleRadians != 0.0) {
        throw std::runtime_error("overlapping poles incorrectly reported a bus collision");
    }
}

PoleImpactResult runPoleImpact(double resistance, double breakAngle) {
    World world;
    BusSimulation simulation(world, testConfiguration(), VehiclePlacement{{0.0, 0.0, 0.0}, 0.0});
    MapPoleCollision pole;
    pole.mapObjectId = 101;
    pole.tileX = 0;
    pole.basePosition = {12.0, 0.0, 0.0};
    pole.topPosition = {12.0, 0.0, 4.0};
    pole.radiusMeters = 0.25;
    pole.resistanceRadiansPerTonneMeterPerSecond = resistance;
    pole.breakAngleRadians = breakAngle;
    world.addMapTileCollision(0, 0, std::nullopt, {}, {pole});
    PoleImpactResult result;
    int stepsSinceImpact = 0;
    for (int step = 0; step < 300; ++step) {
        simulation.step(0.0, 0.0, 1.0);
    }
    for (int step = 0; step < 900; ++step) {
        simulation.step(0.55, 0.0, 0.0);
        const auto states = world.mapPoleStates();
        if (states.empty()) {
            continue;
        }
        if (result.initialAngleRadians == 0.0 && states.front().angleRadians > 0.0) {
            result.initialAngleRadians = states.front().angleRadians;
            result.contactAtFirstBend =
                states.front().touchingBus &&
                std::abs(states.front().radiusMeters - pole.radiusMeters) < 1.0e-9 &&
                std::abs(
                    std::hypot(states.front().topPosition[0] - states.front().basePosition[0],
                               states.front().topPosition[1] - states.front().basePosition[1],
                               states.front().topPosition[2] - states.front().basePosition[2]) -
                    4.0) < 1.0e-9;
        }
        if (result.initialAngleRadians > 0.0) {
            ++stepsSinceImpact;
            if (states.front().broken || stepsSinceImpact >= 90) {
                return {result.initialAngleRadians, result.contactAtFirstBend, states.front()};
            }
        }
    }
    throw std::runtime_error("bus did not bend the configured map pole");
}

void printMetrics(const char* label, const ContactMetrics& metrics) {
    std::cout << label << " maxYawRate=" << metrics.maximumYawRate
              << " maxSteeringHinge=" << metrics.maximumSteeringHingeAngle
              << " maxLeftRightSuspension=" << metrics.maximumLeftRightSuspensionDifference
              << " maxSuspensionChange=" << metrics.maximumPerStepSuspensionChange
              << " finalX=" << metrics.finalX << '\n';
}

} // namespace

int main() {
    try {
        checkTessellatedRoadContact(0.125, false);
        checkTessellatedRoadContact(0.125, true);
        checkFastTessellatedRoadContact(0.0);
        checkFastTessellatedRoadContact(0.6);
        checkOverlappingPolesDoNotReportBusContact();
        const ContactMetrics firm = runCrossing({0.8, 1.0e-6});
        const ContactMetrics compliant = runCrossing({0.65, 1.0e-5});
        const ContactMetrics firmCurb = runCurbCrossing({0.8, 1.0e-6});
        const ContactMetrics compliantCurb = runCurbCrossing({0.65, 1.0e-5});
        const PoleImpactResult bentPole = runPoleImpact(0.0001, 1.2);
        const PoleImpactResult brokenPole = runPoleImpact(0.25, 0.01);
        const PoleImpactResult highThresholdBrokenPole = runPoleImpact(0.25, 2.0);
        printMetrics("firm", firm);
        printMetrics("compliant", compliant);
        printMetrics("firm-curb", firmCurb);
        printMetrics("compliant-curb", compliantCurb);
        if (bentPole.finalState.broken || bentPole.finalState.angleRadians <= 0.0 ||
            !bentPole.contactAtFirstBend ||
            !brokenPole.finalState.broken ||
            brokenPole.finalState.angleRadians < std::acos(-1.0) * 0.5 ||
            brokenPole.initialAngleRadians >= brokenPole.finalState.angleRadians ||
            !highThresholdBrokenPole.finalState.broken ||
            highThresholdBrokenPole.finalState.angleRadians < 2.0 ||
            highThresholdBrokenPole.initialAngleRadians >=
                highThresholdBrokenPole.finalState.angleRadians) {
            std::cerr << "pole impact behavior unexpected: bent="
                      << bentPole.initialAngleRadians << "->"
                      << bentPole.finalState.angleRadians << " broken="
                      << brokenPole.initialAngleRadians << "->"
                      << brokenPole.finalState.angleRadians << " ("
                      << brokenPole.finalState.broken << ") high-threshold="
                      << highThresholdBrokenPole.initialAngleRadians << "->"
                      << highThresholdBrokenPole.finalState.angleRadians << " ("
                      << highThresholdBrokenPole.finalState.broken << ")\n";
            return 1;
        }
        if (firm.finalX < 21.0 || compliant.finalX < 21.0) {
            std::cerr << "bus did not traverse the raised-marking test strip\n";
            return 1;
        }
        if (compliant.maximumYawRate > firm.maximumYawRate * 1.10 + 1.0e-5 ||
            compliant.maximumSteeringHingeAngle > firm.maximumSteeringHingeAngle * 1.10 + 1.0e-5 ||
            compliant.maximumLeftRightSuspensionDifference >
                firm.maximumLeftRightSuspensionDifference * 1.10 + 1.0e-5 ||
            compliant.maximumPerStepSuspensionChange > firm.maximumPerStepSuspensionChange * 1.10 + 1.0e-5) {
            std::cerr << "compliant wheel-static contacts excessively worsened marking response\n";
            return 1;
        }
        if (compliant.maximumYawRate >= firm.maximumYawRate * 0.98 &&
            compliant.maximumSteeringHingeAngle >= firm.maximumSteeringHingeAngle * 0.98 &&
            compliant.maximumPerStepSuspensionChange >= firm.maximumPerStepSuspensionChange * 0.98) {
            std::cerr << "compliant wheel-static contacts did not measurably reduce the marking response\n";
            return 1;
        }
        if (firmCurb.finalX < 16.75 || compliantCurb.finalX < 16.75 ||
            compliantCurb.maximumLeftRightSuspensionDifference < 0.05 ||
            compliantCurb.maximumLeftRightSuspensionDifference > 0.28) {
            std::cerr << "compliant wheel-static contacts removed the curb response or produced excessive suspension travel\n";
            return 1;
        }
    } catch (const std::exception& error) {
        std::cerr << "wheel-static contact probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
