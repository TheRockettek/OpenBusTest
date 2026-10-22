#include "render/MatrixMath.h"

#include <cmath>

namespace openbus::rendering {

namespace {

std::array<double, 3> applyMeshAxis(const std::array<double, 3>& renderAxis,
                                    const std::array<double, 9>& meshRotation) {
    const std::array<double, 3> sourceAxis = {-renderAxis[1], renderAxis[0], renderAxis[2]};
    const std::array<double, 3> rotatedSourceAxis = {
        meshRotation[0] * sourceAxis[0] + meshRotation[3] * sourceAxis[1] +
            meshRotation[6] * sourceAxis[2],
        meshRotation[1] * sourceAxis[0] + meshRotation[4] * sourceAxis[1] +
            meshRotation[7] * sourceAxis[2],
        meshRotation[2] * sourceAxis[0] + meshRotation[5] * sourceAxis[1] +
            meshRotation[8] * sourceAxis[2]};
    return {rotatedSourceAxis[1], -rotatedSourceAxis[0], rotatedSourceAxis[2]};
}

} // namespace

Matrix4 identityMatrix() {
    return {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
}

Matrix4 multiplyMatrix4(const Matrix4& left, const Matrix4& right) {
    Matrix4 result = {};
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            for (int inner = 0; inner < 4; ++inner) {
                result[row + column * 4] += left[row + inner * 4] * right[inner + column * 4];
            }
        }
    }
    return result;
}

std::array<double, 4> transformPoint(const Matrix4& matrix, const std::array<double, 4>& point) {
    std::array<double, 4> result = {};
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            result[row] += matrix[row + column * 4] * point[column];
        }
    }
    return result;
}

ViewFrustum buildViewFrustum(const Matrix4& projection) {
    const std::array<std::array<double, 4>, 6> planeSigns = {{{{1.0, 0.0, 0.0, 1.0}},
                                                              {{-1.0, 0.0, 0.0, 1.0}},
                                                              {{0.0, 1.0, 0.0, 1.0}},
                                                              {{0.0, -1.0, 0.0, 1.0}},
                                                              {{0.0, 0.0, 1.0, 1.0}},
                                                              {{0.0, 0.0, -1.0, 1.0}}}};
    ViewFrustum frustum;
    for (std::size_t plane = 0; plane < planeSigns.size(); ++plane) {
        double lengthSquared = 0.0;
        for (int column = 0; column < 4; ++column) {
            double coefficient = 0.0;
            for (int row = 0; row < 4; ++row) {
                coefficient += planeSigns[plane][row] * projection[row + column * 4];
            }
            frustum.planes[plane][column] = coefficient;
            if (column < 3) {
                lengthSquared += coefficient * coefficient;
            }
        }
        frustum.planeLengths[plane] = std::sqrt(lengthSquared);
    }
    return frustum;
}

bool sphereOutsideFrustum(const ViewFrustum& frustum, const std::array<double, 4>& eye,
                          double radius) {
    for (std::size_t plane = 0; plane < frustum.planes.size(); ++plane) {
        const double planeDistance = frustum.planes[plane][0] * eye[0] +
                                     frustum.planes[plane][1] * eye[1] +
                                     frustum.planes[plane][2] * eye[2] + frustum.planes[plane][3];
        if (planeDistance < -radius * frustum.planeLengths[plane]) {
            return true;
        }
    }
    return false;
}

Matrix4 translationMatrix(const std::array<double, 3>& value) {
    Matrix4 result = identityMatrix();
    result[12] = value[0];
    result[13] = value[1];
    result[14] = value[2];
    return result;
}

Matrix4 poseMatrix(const BodyPose& pose) {
    return {pose.rotation[0], pose.rotation[3], pose.rotation[6], 0.0,
            pose.rotation[1], pose.rotation[4], pose.rotation[7], 0.0,
            pose.rotation[2], pose.rotation[5], pose.rotation[8], 0.0,
            pose.position[0], pose.position[1], pose.position[2], 1.0};
}

Matrix4 makeOdeWheelAnimationTransform(const BodyPose& chassis, const BodyPose& wheel,
                                       const std::array<double, 3>& modelCenter,
                                       double modelOffsetZ) {
    Matrix4 relative = identityMatrix();
    const std::array<double, 3> delta = {wheel.position[0] - chassis.position[0],
                                         wheel.position[1] - chassis.position[1],
                                         wheel.position[2] - chassis.position[2]};
    for (int row = 0; row < 3; ++row) {
        const double odeLocalPosition = chassis.rotation[row] * delta[0] +
                                        chassis.rotation[3 + row] * delta[1] +
                                        chassis.rotation[6 + row] * delta[2];
        const double modelLocalPosition = modelCenter[row] + (row == 2 ? modelOffsetZ : 0.0);
        relative[12 + row] = odeLocalPosition - modelLocalPosition;
        for (int column = 0; column < 3; ++column) {
            relative[column * 4 + row] = chassis.rotation[row] * wheel.rotation[column] +
                                         chassis.rotation[3 + row] * wheel.rotation[3 + column] +
                                         chassis.rotation[6 + row] * wheel.rotation[6 + column];
        }
    }
    Matrix4 base = identityMatrix();
    base[5] = 0.0;
    base[6] = -1.0;
    base[9] = 1.0;
    base[10] = 0.0;
    Matrix4 inverseBase = identityMatrix();
    static_cast<void>(invertAffineMatrix(base, inverseBase));
    relative = multiplyMatrix4(relative, inverseBase);
    const Matrix4 toOrigin = translationMatrix({-modelCenter[0], -modelCenter[1], -modelCenter[2]});
    const Matrix4 fromOrigin = translationMatrix({modelCenter[0], modelCenter[1], modelCenter[2]});
    return multiplyMatrix4(fromOrigin, multiplyMatrix4(relative, toOrigin));
}

Matrix4 rotationMatrix(double angleDegrees, double x, double y, double z) {
    const double length = std::sqrt(x * x + y * y + z * z);
    if (length <= 1.0e-12) {
        return identityMatrix();
    }
    x /= length;
    y /= length;
    z /= length;
    const double angle = angleDegrees * 3.141592653589793 / 180.0;
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    const double inverseCosine = 1.0 - cosine;
    return {cosine + x * x * inverseCosine,
            y * x * inverseCosine + z * sine,
            z * x * inverseCosine - y * sine,
            0.0,
            x * y * inverseCosine - z * sine,
            cosine + y * y * inverseCosine,
            z * y * inverseCosine + x * sine,
            0.0,
            x * z * inverseCosine + y * sine,
            y * z * inverseCosine - x * sine,
            cosine + z * z * inverseCosine,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0};
}

bool invertAffineMatrix(const Matrix4& matrix, Matrix4& inverse) {
    const double a = matrix[0];
    const double b = matrix[4];
    const double c = matrix[8];
    const double d = matrix[1];
    const double e = matrix[5];
    const double f = matrix[9];
    const double g = matrix[2];
    const double h = matrix[6];
    const double i = matrix[10];
    const double determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    if (std::abs(determinant) <= 1.0e-12) {
        inverse = identityMatrix();
        return false;
    }
    const double scale = 1.0 / determinant;
    inverse = identityMatrix();
    inverse[0] = (e * i - f * h) * scale;
    inverse[4] = (c * h - b * i) * scale;
    inverse[8] = (b * f - c * e) * scale;
    inverse[1] = (f * g - d * i) * scale;
    inverse[5] = (a * i - c * g) * scale;
    inverse[9] = (c * d - a * f) * scale;
    inverse[2] = (d * h - e * g) * scale;
    inverse[6] = (b * g - a * h) * scale;
    inverse[10] = (a * e - b * d) * scale;
    inverse[12] = -(inverse[0] * matrix[12] + inverse[4] * matrix[13] + inverse[8] * matrix[14]);
    inverse[13] = -(inverse[1] * matrix[12] + inverse[5] * matrix[13] + inverse[9] * matrix[14]);
    inverse[14] = -(inverse[2] * matrix[12] + inverse[6] * matrix[13] + inverse[10] * matrix[14]);
    return true;
}

Matrix4 makeMeshTransform(const std::array<double, 9>& meshRotation,
                          const std::array<double, 3>& origin) {
    Matrix4 result = identityMatrix();
    const std::array<std::array<double, 3>, 3> axes = {std::array<double, 3>{1.0, 0.0, 0.0},
                                                       std::array<double, 3>{0.0, 1.0, 0.0},
                                                       std::array<double, 3>{0.0, 0.0, 1.0}};
    for (int column = 0; column < 3; ++column) {
        const std::array<double, 3> transformed = applyMeshAxis(axes[column], meshRotation);
        result[column * 4] = transformed[0];
        result[column * 4 + 1] = transformed[1];
        result[column * 4 + 2] = transformed[2];
    }
    result[12] = origin[0];
    result[13] = origin[1];
    result[14] = origin[2];
    return result;
}

} // namespace openbus::rendering
