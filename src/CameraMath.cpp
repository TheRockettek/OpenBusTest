#include "CameraMath.h"

#include <cmath>
#include <vector>

namespace openbus::rendering {

namespace {

Matrix4 cachedModelViewMatrix = {};
Matrix4 cachedProjectionMatrix = {};
std::vector<Matrix4> modelViewStack;

Matrix4 multiply(const Matrix4& left, const Matrix4& right) {
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

} // namespace

const Matrix4& modelViewMatrix() {
    return cachedModelViewMatrix;
}

const Matrix4& projectionMatrix() {
    return cachedProjectionMatrix;
}

void setPerspective(double width, double height, double fieldOfView) {
    const double aspect = width / height;
    const double nearPlane = 0.1;
    const double farPlane = 500.0;
    const double fov = fieldOfView * 3.141592653589793 / 180.0;
    const double top = nearPlane * std::tan(fov * 0.5);
    const double right = top * aspect;
    cachedProjectionMatrix = {nearPlane / right,
                              0.0,
                              0.0,
                              0.0,
                              0.0,
                              nearPlane / top,
                              0.0,
                              0.0,
                              0.0,
                              0.0,
                              -(farPlane + nearPlane) / (farPlane - nearPlane),
                              -1.0,
                              0.0,
                              0.0,
                              -(2.0 * farPlane * nearPlane) / (farPlane - nearPlane),
                              0.0};
}

void lookAt(double eyeX, double eyeY, double eyeZ, double targetX, double targetY, double targetZ) {
    double forwardX = targetX - eyeX;
    double forwardY = targetY - eyeY;
    double forwardZ = targetZ - eyeZ;
    const double forwardLength =
        std::sqrt(forwardX * forwardX + forwardY * forwardY + forwardZ * forwardZ);
    forwardX /= forwardLength;
    forwardY /= forwardLength;
    forwardZ /= forwardLength;

    double sideX = forwardY;
    double sideY = -forwardX;
    const double sideLength = std::sqrt(sideX * sideX + sideY * sideY);
    sideX /= sideLength;
    sideY /= sideLength;
    const double upX = sideY * forwardZ;
    const double upY = -sideX * forwardZ;
    const double upZ = sideX * forwardY - sideY * forwardX;

    const Matrix4 rotation = {sideX, upX, -forwardX, 0.0, sideY, upY, -forwardY, 0.0,
                              0.0,   upZ, -forwardZ, 0.0, 0.0,   0.0, 0.0,       1.0};
    const Matrix4 translation = {1.0, 0.0, 0.0, 0.0, 0.0,   1.0,   0.0,   0.0,
                                 0.0, 0.0, 1.0, 0.0, -eyeX, -eyeY, -eyeZ, 1.0};
    cachedModelViewMatrix = multiply(rotation, translation);
}

void applyPose(const BodyPose& pose) {
    const Matrix4 matrix = {pose.rotation[0], pose.rotation[3], pose.rotation[6], 0.0,
                            pose.rotation[1], pose.rotation[4], pose.rotation[7], 0.0,
                            pose.rotation[2], pose.rotation[5], pose.rotation[8], 0.0,
                            pose.position[0], pose.position[1], pose.position[2], 1.0};
    cachedModelViewMatrix = multiply(cachedModelViewMatrix, matrix);
}

void pushMatrix() {
    modelViewStack.push_back(cachedModelViewMatrix);
}

void popMatrix() {
    if (modelViewStack.empty()) {
        return;
    }
    cachedModelViewMatrix = modelViewStack.back();
    modelViewStack.pop_back();
}

void translate(double x, double y, double z) {
    cachedModelViewMatrix =
        multiply(cachedModelViewMatrix,
                 Matrix4{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, x, y, z, 1.0});
}

void rotate(double angleDegrees, double x, double y, double z) {
    const double length = std::sqrt(x * x + y * y + z * z);
    if (length <= 1.0e-12) {
        return;
    }
    x /= length;
    y /= length;
    z /= length;
    const double angle = angleDegrees * 3.141592653589793 / 180.0;
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    const double inverseCosine = 1.0 - cosine;
    cachedModelViewMatrix =
        multiply(cachedModelViewMatrix,
                 Matrix4{cosine + x * x * inverseCosine, y * x * inverseCosine + z * sine,
                         z * x * inverseCosine - y * sine, 0.0, x * y * inverseCosine - z * sine,
                         cosine + y * y * inverseCosine, z * y * inverseCosine + x * sine, 0.0,
                         x * z * inverseCosine + y * sine, y * z * inverseCosine - x * sine,
                         cosine + z * z * inverseCosine, 0.0, 0.0, 0.0, 0.0, 1.0});
}

void scale(double x, double y, double z) {
    cachedModelViewMatrix =
        multiply(cachedModelViewMatrix,
                 Matrix4{x, 0.0, 0.0, 0.0, 0.0, y, 0.0, 0.0, 0.0, 0.0, z, 0.0, 0.0, 0.0, 0.0, 1.0});
}
std::array<double, 3> transformLocalPoint(const BodyPose& pose,
                                          const std::array<double, 3>& local) {
    return {pose.position[0] + pose.rotation[0] * local[0] + pose.rotation[1] * local[1] +
                pose.rotation[2] * local[2],
            pose.position[1] + pose.rotation[3] * local[0] + pose.rotation[4] * local[1] +
                pose.rotation[5] * local[2],
            pose.position[2] + pose.rotation[6] * local[0] + pose.rotation[7] * local[1] +
                pose.rotation[8] * local[2]};
}

} // namespace openbus::rendering
