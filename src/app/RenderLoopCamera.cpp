#include "app/RenderLoopInternal.h"

openbus::rendering::Matrix4 RenderLoop::buildCameraViewMatrix(const BusSimulation& simulation,
                                                              const BodyPose& chassis) const {
    using openbus::rendering::makeLookDirectionMatrix;
    using openbus::rendering::transformLocalPoint;

    constexpr std::array<double, 3> worldUp = {0.0, 0.0, 1.0};
    if (freeCameraEnabled_ && !renderingReflection_ && !captureMode_) {
        const auto forward = openbus::rendering::freeCameraForward(freeCameraPose_.yawRadians,
                                                                   freeCameraPose_.pitchRadians);
        return makeLookDirectionMatrix(freeCameraPose_.position, forward, worldUp);
    }

    if (cameraView_ == 0) {
        const auto target = transformLocalPoint(chassis, simulation.outsideCameraCenter());
        const double horizontalDistance = cameraDistance_ * std::cos(cameraPitch_);
        const double orbitYaw = simulation.yaw() + cameraYaw_;
        const std::array<double, 3> eye = {target[0] + horizontalDistance * std::cos(orbitYaw),
                                           target[1] + horizontalDistance * std::sin(orbitYaw),
                                           target[2] + cameraDistance_ * std::sin(cameraPitch_)};
        return makeLookDirectionMatrix(
            eye, {target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]}, worldUp);
    }

    if (const VehicleCamera* camera = currentVehicleCamera(); camera != nullptr) {
        constexpr double degreesToRadians = 3.141592653589793 / 180.0;
        const bool reflectionCamera = camera->kind == VehicleCameraKind::Reflexion ||
                                      camera->kind == VehicleCameraKind::Reflexion2;
        const double panSign = reflectionCamera ? 1.0 : -1.0;
        double pan = panSign * camera->pan * degreesToRadians + viewLookYaw_;
        double tilt = camera->tilt * degreesToRadians + viewLookPitch_;
        const double modelOffsetZ = playerVehicle_ != nullptr ? playerVehicle_->modelOffsetZ : 0.0;
        const std::array<double, 3> centerLocal = {camera->position[1], -camera->position[0],
                                                   camera->position[2] + modelOffsetZ};
        if (reflectionCamera && camera->followsViewer && renderingReflection_ &&
            reflectionViewerEyeLocalValid_ && reflectionReferenceCamera_ >= 0 &&
            static_cast<std::size_t>(reflectionReferenceCamera_) < vehicleCameras_.size()) {
            const VehicleCamera& reference =
                vehicleCameras_[static_cast<std::size_t>(reflectionReferenceCamera_)];
            const double referencePan = -reference.pan * degreesToRadians;
            const double referenceTilt = reference.tilt * degreesToRadians;
            const std::array<double, 3> referenceEye = {
                reference.position[1] -
                    std::cos(referenceTilt) * std::cos(referencePan) * reference.orbitDistance,
                -reference.position[0] -
                    std::cos(referenceTilt) * std::sin(referencePan) * reference.orbitDistance,
                reference.position[2] + modelOffsetZ -
                    std::sin(referenceTilt) * reference.orbitDistance};
            const auto reflected = openbus::rendering::mirrorReflectionDirection(
                centerLocal,
                {std::cos(tilt) * std::cos(pan), std::cos(tilt) * std::sin(pan), std::sin(tilt)},
                referenceEye, reflectionViewerEyeLocal_);
            pan = std::atan2(reflected[1], reflected[0]);
            tilt = std::asin(std::clamp(reflected[2], -1.0, 1.0));
        }

        const std::array<double, 3> direction = {std::cos(tilt) * std::cos(pan),
                                                 std::cos(tilt) * std::sin(pan), std::sin(tilt)};
        const std::array<double, 3> upLocal = {-std::sin(tilt) * std::cos(pan),
                                               -std::sin(tilt) * std::sin(pan), std::cos(tilt)};
        const std::array<double, 3> eyeLocal = {
            centerLocal[0] - direction[0] * camera->orbitDistance,
            centerLocal[1] - direction[1] * camera->orbitDistance,
            centerLocal[2] - direction[2] * camera->orbitDistance};
        const std::array<double, 3> targetLocal = {eyeLocal[0] + direction[0] * 3.0,
                                                   eyeLocal[1] + direction[1] * 3.0,
                                                   eyeLocal[2] + direction[2] * 3.0};
        const auto eye = transformLocalPoint(chassis, eyeLocal);
        const auto target = transformLocalPoint(chassis, targetLocal);
        const std::array<double, 3> up = {
            chassis.rotation[0] * upLocal[0] + chassis.rotation[1] * upLocal[1] +
                chassis.rotation[2] * upLocal[2],
            chassis.rotation[3] * upLocal[0] + chassis.rotation[4] * upLocal[1] +
                chassis.rotation[5] * upLocal[2],
            chassis.rotation[6] * upLocal[0] + chassis.rotation[7] * upLocal[1] +
                chassis.rotation[8] * upLocal[2]};
        return makeLookDirectionMatrix(
            eye, {target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]}, up);
    }

    std::array<double, 3> eyeLocal = {5.65, 0.70, 0.70};
    std::array<double, 3> targetLocal = {8.65, 0.70, 0.72};
    switch (cameraView_) {
    case 2:
        eyeLocal = {4.25, -0.55, 0.78};
        targetLocal = {7.25, -0.55, 0.80};
        break;
    case 3:
        eyeLocal = {-2.75, 0.0, 0.72};
        targetLocal = {0.25, 0.0, 0.74};
        break;
    case 4:
        eyeLocal = {5.35, 1.05, 0.86};
        targetLocal = {5.35, 4.0, 0.82};
        break;
    case 5:
        eyeLocal = {5.35, -1.05, 0.86};
        targetLocal = {5.35, -4.0, 0.82};
        break;
    case 6:
        eyeLocal = {6.85, 0.0, 0.80};
        targetLocal = {3.85, 0.0, 0.80};
        break;
    case 7:
        eyeLocal = {-6.15, 0.0, 0.80};
        targetLocal = {-3.15, 0.0, 0.80};
        break;
    case 8:
        eyeLocal = {0.0, 1.05, 1.30};
        targetLocal = {0.0, 4.0, 1.30};
        break;
    case 9:
        eyeLocal = {0.0, -1.05, 1.30};
        targetLocal = {0.0, -4.0, 1.30};
        break;
    default:
        break;
    }

    const auto eye = transformLocalPoint(chassis, eyeLocal);
    const double directionX = targetLocal[0] - eyeLocal[0];
    const double directionY = targetLocal[1] - eyeLocal[1];
    const double directionZ = targetLocal[2] - eyeLocal[2];
    const double distance =
        std::sqrt(directionX * directionX + directionY * directionY + directionZ * directionZ);
    const double baseYaw = std::atan2(directionY, directionX);
    const double basePitch = std::atan2(directionZ, std::hypot(directionX, directionY));
    const double lookYaw = baseYaw + viewLookYaw_;
    const double lookPitch = basePitch + viewLookPitch_;
    const auto target = transformLocalPoint(
        chassis, {eyeLocal[0] + distance * std::cos(lookPitch) * std::cos(lookYaw),
                  eyeLocal[1] + distance * std::cos(lookPitch) * std::sin(lookYaw),
                  eyeLocal[2] + distance * std::sin(lookPitch)});
    return makeLookDirectionMatrix(
        eye, {target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]}, worldUp);
}
