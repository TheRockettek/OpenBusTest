#include "BusSimulation.h"
#include "MouseControlMapping.h"
#include "Variables.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

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
    for (const double invalidRate : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::infinity(),
                                     std::numeric_limits<double>::denorm_min()}) {
        bool rejected = false;
        try {
            BusSimulation invalidSimulation(testConfiguration(),
                                            VehiclePlacement{{0.0, 0.0, 0.0}, 0.0},
                                            invalidRate);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        if (!rejected) {
            std::cerr << "invalid physics rate was accepted: " << invalidRate << '\n';
            return 1;
        }
    }

    const auto centerMouse = openbus::input::mouseControlInputs(400.0, 300.0, 800.0, 600.0);
    const auto topMouse = openbus::input::mouseControlInputs(400.0, 0.0, 800.0, 600.0);
    const auto bottomMouse = openbus::input::mouseControlInputs(400.0, 600.0, 800.0, 600.0);
    const auto leftMouse = openbus::input::mouseControlInputs(0.0, 300.0, 800.0, 600.0);
    const auto rightMouse = openbus::input::mouseControlInputs(800.0, 300.0, 800.0, 600.0);
    const auto partialMouse = openbus::input::mouseControlInputs(600.0, 150.0, 800.0, 600.0);
    const auto slightlyLeftOfCenter =
        openbus::input::mouseControlInputs(396.0, 300.0, 800.0, 600.0);
    const auto slightlyRightOfCenter =
        openbus::input::mouseControlInputs(404.0, 300.0, 800.0, 600.0);
    const auto invalidMouse = openbus::input::mouseControlInputs(400.0, 300.0, 0.0, 600.0);
    const double smoothedSteering = openbus::input::smoothSteeringInput(0.0, 1.0, 0.1, 6.0);
    const double unsmoothedSteering = openbus::input::smoothSteeringInput(0.0, 1.0, 0.1, 0.0);
    if (centerMouse.throttle != 0.0 || centerMouse.steering != 0.0 ||
        centerMouse.brake != 0.0 || topMouse.throttle != 1.0 || topMouse.brake != 0.0 ||
        bottomMouse.throttle != 0.0 || bottomMouse.brake != 1.0 || leftMouse.steering != -1.0 ||
        rightMouse.steering != 1.0 || partialMouse.throttle != 0.5 ||
        partialMouse.steering != 0.5 || invalidMouse.throttle != 0.0 ||
        invalidMouse.steering != 0.0 || invalidMouse.brake != 0.0 ||
        std::abs(slightlyLeftOfCenter.steering + 0.01) > 1.0e-9 ||
        std::abs(slightlyRightOfCenter.steering - 0.01) > 1.0e-9 ||
        std::abs(smoothedSteering - 0.6) > 1.0e-9 || unsmoothedSteering != 1.0) {
        std::cerr << "mouse mapping or steering smoothing produced an unexpected value\n";
        return 1;
    }

    BusSimulation simulation(testConfiguration(), VehiclePlacement{{0.0, 0.0, 0.0}, 0.0});
    BusConfiguration wheelContactConfiguration = testConfiguration();
    wheelContactConfiguration.collisionEnabled = false;
    BusSimulation softSuspension(wheelContactConfiguration,
                                 VehiclePlacement{{-1000.0, 0.0, 0.0}, 0.0});
    BusSimulation stiffSuspension(wheelContactConfiguration,
                                  VehiclePlacement{{-1000.0, 0.0, 0.0}, 0.0});
    const std::vector<double> noWheelBrakes(softSuspension.wheelCount(), 0.0);
    const std::vector<double> softSpringFactors(softSuspension.wheelCount(), 0.5);
    const std::vector<double> stiffSpringFactors(softSuspension.wheelCount(), 2.0);
    double maximumVerticalResponseDifference = 0.0;
    for (int step = 0; step < 600; ++step) {
        softSuspension.stepWithWheelTorqueAndBrakeForces(0.0, 0.0, noWheelBrakes,
                                                         softSpringFactors);
        stiffSuspension.stepWithWheelTorqueAndBrakeForces(0.0, 0.0, noWheelBrakes,
                                                          stiffSpringFactors);
        maximumVerticalResponseDifference =
            std::max(maximumVerticalResponseDifference,
                     std::abs(softSuspension.positionZ() - stiffSuspension.positionZ()));
    }
    if (maximumVerticalResponseDifference <= 1.0e-5) {
        std::cerr << "Axle_Springfactor did not change ODE suspension response: max vertical "
                     "difference="
                  << maximumVerticalResponseDifference << '\n';
        return 1;
    }

    BusSimulation lowFactorGroundCheck(wheelContactConfiguration,
                                       VehiclePlacement{{-1000.0, 0.0, 0.0}, 0.0});
    const std::vector<double> lowSpringFactors(lowFactorGroundCheck.wheelCount(), 0.3);
    const std::vector<double> groundCheckBrakes(lowFactorGroundCheck.wheelCount(), 0.0);
    double minimumWheelGroundClearance = std::numeric_limits<double>::max();
    double maximumSliderCompression = 0.0;
    std::vector<double> finalAnimationDisplacements(lowFactorGroundCheck.wheelCount(), 0.0);
    const double restSuspensionLength = 0.35;
    const double centerOfGravityHeight = testConfiguration().centerOfGravityHeight;
    for (int step = 0; step < 900; ++step) {
        lowFactorGroundCheck.stepWithWheelTorqueAndBrakeForces(
            0.0, 0.0, groundCheckBrakes, lowSpringFactors);
        const BodyPose chassisPose = lowFactorGroundCheck.chassisPose();
        for (std::size_t wheel = 0; wheel < lowFactorGroundCheck.wheelCount(); ++wheel) {
            const BodyPose wheelPose = lowFactorGroundCheck.wheelPose(wheel);
            const double wheelRadius = lowFactorGroundCheck.wheelRadius(wheel);
            minimumWheelGroundClearance =
                std::min(minimumWheelGroundClearance, wheelPose.position[2] - wheelRadius);
            const double deltaX = wheelPose.position[0] - chassisPose.position[0];
            const double deltaY = wheelPose.position[1] - chassisPose.position[1];
            const double deltaZ = wheelPose.position[2] - chassisPose.position[2];
            const double wheelLocalZ = chassisPose.rotation[2] * deltaX +
                                       chassisPose.rotation[5] * deltaY +
                                       chassisPose.rotation[8] * deltaZ;
            const double anchorLocalZ = wheelRadius + restSuspensionLength - centerOfGravityHeight;
            const double rawCompression =
                restSuspensionLength - (anchorLocalZ - wheelLocalZ);
            finalAnimationDisplacements[wheel] = rawCompression;
            maximumSliderCompression =
                std::max(maximumSliderCompression,
                         lowFactorGroundCheck.wheelSuspensionSliderTravel(wheel));
        }
    }
    openbus::scripting::Vehicle lowFactorVariables;
    lowFactorGroundCheck.updateVariables(lowFactorVariables, 0.0, 0.0, 0.0);
    bool animationVariableMatchesPose = true;
    for (std::size_t wheel = 0; wheel < lowFactorGroundCheck.wheelCount(); ++wheel) {
        const std::string variableName = "axle_suspension_" + std::to_string(wheel / 2) +
                                         (wheel % 2 == 0 ? "_l" : "_r");
        const double expectedDisplacement =
            std::clamp(-finalAnimationDisplacements[wheel], -0.25, 0.25);
        animationVariableMatchesPose =
            animationVariableMatchesPose &&
            std::abs(lowFactorVariables.get(variableName) - expectedDisplacement) < 1.0e-5;
    }
    if (minimumWheelGroundClearance < -0.005 || maximumSliderCompression > 0.28 ||
        !animationVariableMatchesPose) {
        std::cerr << "ODE wheel contact/travel exceeded its expected range: ground clearance="
                  << minimumWheelGroundClearance
                  << " m, maximum slider compression=" << maximumSliderCompression
                  << " m, animation variable matches pose="
                  << (animationVariableMatchesPose ? "yes" : "no") << '\n';
        return 1;
    }

    const BusConfiguration steeringLimitConfiguration = testConfiguration();
    const double expectedMaximumSteeringAngle =
        std::atan(steeringLimitConfiguration.inverseMinimumTurnRadius *
                  std::abs(steeringLimitConfiguration.axles[0].position -
                           steeringLimitConfiguration.axles[2].position));
    BusSimulation steeringLimitCheck(steeringLimitConfiguration,
                                     VehiclePlacement{{0.0, 0.0, 0.0}, 0.0});
    for (int step = 0; step < 600; ++step) {
        steeringLimitCheck.step(0.0, 1.0, 0.0);
    }
    if (std::abs(steeringLimitCheck.steeringAngle() - expectedMaximumSteeringAngle) > 1.0e-5) {
        std::cerr << "inverse minimum turn radius did not set the live steering limit: expected "
                  << expectedMaximumSteeringAngle << ", got "
                  << steeringLimitCheck.steeringAngle() << '\n';
        return 1;
    }

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
    if (variables.get("velocity") != static_cast<float>(expectedSpeedKmh) ||
        variables.get("velocity_ground") != static_cast<float>(expectedSpeedKmh)) {
        std::cerr << "script velocity variables were not expressed in km/h\n";
        return 1;
    }

    BusSimulation wheelSpeedCheck(testConfiguration(), VehiclePlacement{{0.0, 0.0, 0.0}, 0.0});
    for (int step = 0; step < 600; ++step) {
        wheelSpeedCheck.stepWithWheelTorque(0.0, 0.0, 0.0);
    }
    for (int step = 0; step < 4; ++step) {
        wheelSpeedCheck.stepWithWheelTorque(1000.0, 0.0, 0.0);
    }
    const double rotationBeforeFinalStep = wheelSpeedCheck.wheelRotation(2);
    wheelSpeedCheck.stepWithWheelTorque(1000.0, 0.0, 0.0);
    openbus::scripting::Vehicle wheelSpeedVariables;
    wheelSpeedCheck.updateVariables(wheelSpeedVariables, 0.0, 0.0, 0.0);
    const double expectedWheelSpeedRpm =
        std::abs(wheelSpeedCheck.wheelRotation(2) - rotationBeforeFinalStep) *
        wheelSpeedCheck.physicsHz() * 60.0 /
        (2.0 * 3.141592653589793);
    const double actualWheelSpeedRpm = wheelSpeedVariables.get("wheel_rotationspeed_1_l");
    const double averageDrivenWheelRpm =
        (wheelSpeedVariables.get("wheel_rotationspeed_1_l") +
         wheelSpeedVariables.get("wheel_rotationspeed_1_r")) /
        2.0;
    if (expectedWheelSpeedRpm < 1.0 || actualWheelSpeedRpm <= 0.0 ||
        std::abs(actualWheelSpeedRpm - expectedWheelSpeedRpm) >
            std::max(0.5, expectedWheelSpeedRpm * 0.1) ||
        std::abs(wheelSpeedVariables.get("n_wheel") - averageDrivenWheelRpm) > 1.0e-4) {
        std::cerr << "script wheel speeds were inconsistent with integrated forward rotation: "
                     "expected about "
                  << expectedWheelSpeedRpm << " rpm, got " << actualWheelSpeedRpm << " rpm\n";
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

    BusSimulation brakeCheck(testConfiguration(), VehiclePlacement{{0.0, 0.0, 0.0}, 0.0});
    for (int step = 0; step < 600; ++step) {
        brakeCheck.stepWithWheelTorque(0.0, 0.0, 0.0);
    }
    for (int step = 0; step < 600; ++step) {
        brakeCheck.stepWithWheelTorque(25000.0, 0.0, 0.0);
    }
    const double speedBeforeBraking = brakeCheck.speed();
    for (int step = 0; step < 180; ++step) {
        brakeCheck.stepWithWheelTorque(0.0, 0.0, 1.0);
    }
    const double speedDuringBraking = brakeCheck.speed();
    if (speedBeforeBraking <= 0.01 || speedDuringBraking >= speedBeforeBraking * 0.8) {
        std::cerr << "brake input did not materially reduce speed after drive torque was released: "
                  << speedBeforeBraking << " -> " << speedDuringBraking << '\n';
        return 1;
    }

    BusSimulation parkingBrakeCheck(testConfiguration(), VehiclePlacement{{0.0, 0.0, 0.0}, 0.0});
    for (int step = 0; step < 600; ++step) {
        parkingBrakeCheck.stepWithWheelTorque(0.0, 0.0, 0.0);
    }
    for (int step = 0; step < 600; ++step) {
        parkingBrakeCheck.stepWithWheelTorque(25000.0, 0.0, 0.0);
    }
    const double speedBeforeParkingBrake = parkingBrakeCheck.speed();
    std::vector<double> parkingBrakeForces(parkingBrakeCheck.wheelCount(), 0.0);
    parkingBrakeForces[2] = 40000.0;
    parkingBrakeForces[3] = 40000.0;
    for (int step = 0; step < 180; ++step) {
        parkingBrakeCheck.stepWithWheelTorqueAndBrakeForces(25000.0, 0.0,
                                                            parkingBrakeForces);
    }
    const double speedDuringParkingBrake = parkingBrakeCheck.speed();
    if (speedBeforeParkingBrake <= 0.01 ||
        speedDuringParkingBrake >= speedBeforeParkingBrake * 0.8) {
        std::cerr << "script parking-brake forces did not override drive torque: "
                  << speedBeforeParkingBrake << " -> " << speedDuringParkingBrake << '\n';
        return 1;
    }
    std::cout << "target=" << simulation.steeringAngle() << " left=" << leftHeading
              << " right=" << rightHeading
              << " leftHeight=" << leftHeight
              << " axleSteering0L=" << variables.get("axle_steering_0_l")
              << " wheelRotation1L=" << variables.get("wheel_rotation_1_l")
              << " wheelRotation1R=" << variables.get("wheel_rotation_1_r")
              << " minLowFactorGroundClearance=" << minimumWheelGroundClearance
              << " maxLowFactorSliderCompression=" << maximumSliderCompression
              << " rearPoseChange=" << rearPoseChange
              << " headingDirectionChanges=" << headingDirectionChanges << '\n';
    return 0;
}
