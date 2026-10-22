#pragma once

// Column-major 4x4 matrix helpers shared by the vehicle renderer and render loop.

#include "core/BusTypes.h"
#include "render/CameraMath.h"

#include <array>

namespace openbus::rendering {

[[nodiscard]] Matrix4 identityMatrix();

[[nodiscard]] Matrix4 multiplyMatrix4(const Matrix4& left, const Matrix4& right);

[[nodiscard]] std::array<double, 4> transformPoint(const Matrix4& matrix,
                                                   const std::array<double, 4>& point);

using FrustumPlanes = std::array<std::array<double, 4>, 6>;
using FrustumPlaneLengths = std::array<double, 6>;

struct ViewFrustum {
    FrustumPlanes planes = {};
    FrustumPlaneLengths planeLengths = {};
};

[[nodiscard]] ViewFrustum buildViewFrustum(const Matrix4& projection);

[[nodiscard]] bool sphereOutsideFrustum(const ViewFrustum& frustum,
                                        const std::array<double, 4>& eye, double radius);

[[nodiscard]] Matrix4 translationMatrix(const std::array<double, 3>& value);

[[nodiscard]] Matrix4 poseMatrix(const BodyPose& pose);

[[nodiscard]] Matrix4 makeOdeWheelAnimationTransform(const BodyPose& chassis, const BodyPose& wheel,
                                                     const std::array<double, 3>& modelCenter,
                                                     double modelOffsetZ);

[[nodiscard]] Matrix4 rotationMatrix(double angleDegrees, double x, double y, double z);

[[nodiscard]] bool invertAffineMatrix(const Matrix4& matrix, Matrix4& inverse);

[[nodiscard]] Matrix4 makeMeshTransform(const std::array<double, 9>& meshRotation,
                                        const std::array<double, 3>& origin);

} // namespace openbus::rendering
