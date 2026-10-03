#include "BusSimulation.h"
#include "MouseControlMapping.h"
#include "Variables.h"

#include <cmath>
#include <iostream>

namespace {

BusConfiguration testConfiguration() {
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

double steeringHeading(const BodyPose& pose) {
    return std::atan2(-pose.rotation[2], pose.rotation[5]);
}

} // namespace

int main() {
    const auto centerMouse = openbus::input::mouseControlInputs(400.0, 300.0, 800.0, 600.0);
    const auto topMouse = openbus::input::mouseControlInputs(400.0, 0.0, 800.0, 600.0);
    const auto bottomMouse = openbus::input::mouseControlInputs(400.0, 600.0, 800.0, 600.0);
    const auto leftMouse = openbus::input::mouseControlInputs(0.0, 300.0, 800.0, 600.0);
    const auto rightMouse = openbus::input::mouseControlInputs(800.0, 300.0, 800.0, 600.0);
    const auto partialMouse = openbus::input::mouseControlInputs(600.0, 150.0, 800.0, 600.0);
    const auto invalidMouse = openbus::input::mouseControlInputs(400.0, 300.0, 0.0, 600.0);
    const double steeringStep = openbus::input::slewSteeringInput(0.0, 1.0, 0.1, 6.0);
    const double steeringReversal = openbus::input::slewSteeringInput(1.0, -1.0, 0.1, 6.0);
    if (centerMouse.throttle != 0.0 || centerMouse.steering != 0.0 ||
        centerMouse.brake != 0.0 || topMouse.throttle != 1.0 || topMouse.brake != 0.0 ||
        bottomMouse.throttle != 0.0 || bottomMouse.brake != 1.0 || leftMouse.steering != -1.0 ||
        rightMouse.steering != 1.0 || partialMouse.throttle != 0.5 ||
        partialMouse.steering != 0.5 || invalidMouse.throttle != 0.0 ||
        invalidMouse.steering != 0.0 || invalidMouse.brake != 0.0 ||
        std::abs(steeringStep - 0.6) > 1.0e-9 ||
        std::abs(steeringReversal - 0.4) > 1.0e-9) {
        std::cerr << "input mapping or steering rate limiting produced an unexpected value\n";
        return 1;
    }

    BusSimulation simulation(testConfiguration(), VehiclePlacement{{0.0, 0.0, 0.0}, 0.0});
    const BodyPose rearStartPose = simulation.wheelPose(2);
    for (int step = 0; step < 180; ++step) {
        simulation.step(0.0, 0.0, 0.0);
    }
    for (int step = 0; step < 100; ++step) {
        simulation.step(0.6, 1.0, 0.0);
    }
    const BodyPose rearDrivenPose = simulation.wheelPose(2);
    double rearPoseChange = 0.0;
    for (std::size_t index = 0; index < rearStartPose.rotation.size(); ++index) {
        rearPoseChange += std::abs(rearDrivenPose.rotation[index] - rearStartPose.rotation[index]);
    }
    double previousHeading = steeringHeading(simulation.wheelPose(0));
    int headingDirectionChanges = 0;
    int previousDirection = 0;
    for (int step = 0; step < 180; ++step) {
        simulation.step(0.6, -1.0, 0.0);
        const double heading = steeringHeading(simulation.wheelPose(0));
        const double delta = heading - previousHeading;
        const int direction = delta > 1.0e-5 ? 1 : delta < -1.0e-5 ? -1 : 0;
        if (direction != 0 && previousDirection != 0 && direction != previousDirection) {
            ++headingDirectionChanges;
        }
        if (direction != 0) {
            previousDirection = direction;
        }
        previousHeading = heading;
    }

    const double leftHeading = steeringHeading(simulation.wheelPose(0));
    const double rightHeading = steeringHeading(simulation.wheelPose(1));
    const double leftHeight = simulation.wheelPose(0).position[2];
    openbus::scripting::Vehicle variables;
    simulation.updateVariables(variables, 0.6, -1.0, 0.0);
    const double expectedSpeedKmh = simulation.speed() * 3.6;
    if (std::abs(variables.get("velocity") - expectedSpeedKmh) > 1.0e-9 ||
        std::abs(variables.get("velocity_ground") - expectedSpeedKmh) > 1.0e-9) {
        std::cerr << "script velocity variables were not expressed in km/h\n";
        return 1;
    }
    if (!variables.has("axle_steering_0_l") ||
        std::abs(variables.get("axle_steering_0_l")) < 1.0e-3) {
        std::cerr << "axle_steering_0_l was not populated from the steering joint\n";
        return 1;
    }
    if (std::abs(variables.get("wheel_rotation_1_l")) < 1.0e-3 ||
        std::abs(variables.get("wheel_rotation_1_r")) < 1.0e-3) {
        std::cerr << "Rear wheel rotation variables were not populated\n";
        return 1;
    }
    std::cout << "target=" << simulation.steeringAngle() << " left=" << leftHeading
              << " right=" << rightHeading
              << " leftHeight=" << leftHeight
              << " axleSteering0L=" << variables.get("axle_steering_0_l")
              << " wheelRotation1L=" << variables.get("wheel_rotation_1_l")
              << " wheelRotation1R=" << variables.get("wheel_rotation_1_r")
              << " rearPoseChange=" << rearPoseChange
              << " headingDirectionChanges=" << headingDirectionChanges << '\n';
    return 0;
}
