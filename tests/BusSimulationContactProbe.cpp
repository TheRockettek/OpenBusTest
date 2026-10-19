#include "physics/BusSimulation.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
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

struct ContactMetrics {
    double maximumYawRate = 0.0;
    double maximumSteeringHingeAngle = 0.0;
    double maximumLeftRightSuspensionDifference = 0.0;
    double maximumPerStepSuspensionChange = 0.0;
    double finalX = 0.0;
};

ContactMetrics runCrossing(WheelStaticContactSettings settings) {
    BusSimulation simulation(testConfiguration(), VehiclePlacement{{0.0, 0.0, 0.0}, 0.0},
                             60.0, 8, 0.0, {}, {makeRaisedMarking()}, true, settings);
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
    BusSimulation simulation(testConfiguration(), VehiclePlacement{{0.0, 0.0, 0.0}, 0.0},
                             60.0, 8, 0.0, {}, {makeCurb()}, true, settings);
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
        const ContactMetrics firm = runCrossing({0.8, 1.0e-6});
        const ContactMetrics compliant = runCrossing({0.65, 1.0e-5});
        const ContactMetrics firmCurb = runCurbCrossing({0.8, 1.0e-6});
        const ContactMetrics compliantCurb = runCurbCrossing({0.65, 1.0e-5});
        printMetrics("firm", firm);
        printMetrics("compliant", compliant);
        printMetrics("firm-curb", firmCurb);
        printMetrics("compliant-curb", compliantCurb);
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
