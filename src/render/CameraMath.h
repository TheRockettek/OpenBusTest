#pragma once

#include "core/BusTypes.h"

#include <array>

namespace openbus::rendering {

using Matrix4 = std::array<double, 16>;

struct FreeCameraPose {
    std::array<double, 3> position = {};
    double yawRadians = 0.0;
    double pitchRadians = 0.0;
};

void setPerspective(double width, double height, double fieldOfView);
void lookAt(double eyeX, double eyeY, double eyeZ, double targetX, double targetY, double targetZ);
void lookAt(double eyeX, double eyeY, double eyeZ, double targetX, double targetY, double targetZ,
            double upX, double upY, double upZ);
void applyPose(const BodyPose& pose);
// Converts a ground-referenced static placement to the model root height
// required before Vehicle::draw applies modelOffsetZ.
VehiclePlacement makeModelRootPlacement(const VehiclePlacement& groundPlacement,
                                        double modelOffsetZ);
// Returns a local Z translation for a yaw-only vehicle's fake shadow after
// modelOffsetZ has been applied.
Matrix4 makeGroundShadowTransform(const VehiclePlacement& modelRootPlacement, double modelOffsetZ,
                                  double groundPlaneZ);
void pushMatrix();
void popMatrix();
void translate(double x, double y, double z);
void rotate(double angleDegrees, double x, double y, double z);
void multiplyMatrix(const Matrix4& matrix);
void scale(double x, double y, double z);
const Matrix4& modelViewMatrix();
const Matrix4& projectionMatrix();
void setModelViewMatrix(const Matrix4& matrix);
void setProjectionMatrix(const Matrix4& matrix);
std::array<double, 3> transformLocalPoint(const BodyPose& pose, const std::array<double, 3>& local);
// Treats a reflection camera as a planar mirror at `mirror` that shows `authoredDirection` to
// `referenceEye`, and returns the direction it shows to `viewerEye`. Returns the normalized
// authored direction when the mirror plane is undefined or the viewer is behind it.
std::array<double, 3> mirrorReflectionDirection(const std::array<double, 3>& mirror,
                                                const std::array<double, 3>& authoredDirection,
                                                const std::array<double, 3>& referenceEye,
                                                const std::array<double, 3>& viewerEye);
std::array<double, 3> freeCameraForward(double yawRadians, double pitchRadians);
FreeCameraPose moveFreeCamera(const FreeCameraPose& pose, double forwardInput, double rightInput,
                              double verticalInput, double deltaSeconds,
                              double speedMetersPerSecond);

} // namespace openbus::rendering
