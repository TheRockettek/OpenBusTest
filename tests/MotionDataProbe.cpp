#include "render/MotionData.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

using openbus::rendering::Matrix4;
using openbus::rendering::MotionHistory;
using openbus::rendering::MotionObjectKey;

Matrix4 identity() {
    return {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
}

Matrix4 translation(double x, double y = 0.0, double z = 0.0) {
    Matrix4 result = identity();
    result[12] = x;
    result[13] = y;
    result[14] = z;
    return result;
}

Matrix4 multiply(const Matrix4& left, const Matrix4& right) {
    Matrix4 result = {};
    for (std::size_t column = 0; column < 4; ++column) {
        for (std::size_t row = 0; row < 4; ++row) {
            for (std::size_t inner = 0; inner < 4; ++inner) {
                result[row + column * 4] +=
                    left[row + inner * 4] * right[inner + column * 4];
            }
        }
    }
    return result;
}

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void testCameraAndObjectMotion() {
    MotionHistory history;
    const Matrix4 identityMatrix = identity();
    const MotionObjectKey movingObject{17, 4};

    history.beginFrame(1, 1.0, 1280, 720, identityMatrix, identityMatrix);
    const auto first = history.previousModelView(translation(0.0), movingObject);
    require(!first.valid, "the first rendered frame must have invalid history");
    history.completeFrame();

    const Matrix4 currentView = translation(-1.0);
    const Matrix4 currentWorld = translation(3.0);
    history.beginFrame(2, 1.016, 1280, 720, currentView, identityMatrix);
    require(history.frame().historyValid, "continuous camera history should remain valid");
    require(std::abs(history.frame().deltaSeconds - 0.016) < 1.0e-9,
            "frame timing should be measured between rendered frames");

    const auto moving = history.previousModelView(multiply(currentView, currentWorld), movingObject);
    require(moving.valid, "a continuously rendered object should have valid history");
    require(std::abs(moving.previousModelView[12]) < 1.0e-9,
            "previous model-view must include the object's previous rendered transform");

    const auto stationary = history.previousModelView(
        multiply(currentView, translation(5.0)), MotionObjectKey{});
    require(stationary.valid, "static geometry should use previous camera history");
    require(std::abs(stationary.previousModelView[12] - 5.0) < 1.0e-9,
            "static geometry must reuse its current world transform with the previous view");
    history.completeFrame();

    history.beginFrame(3, 1.032, 1280, 720, currentView, identityMatrix);
    history.completeFrame();
    history.beginFrame(4, 1.048, 1280, 720, currentView, identityMatrix);
    const auto reappeared =
        history.previousModelView(multiply(currentView, translation(4.0)), movingObject);
    require(!reappeared.valid, "an object missing for a rendered frame must lose its history");
    history.completeFrame();
}

void testHistoryResetConditions() {
    MotionHistory history;
    const Matrix4 identityMatrix = identity();
    history.beginFrame(1, 1.0, 800, 600, identityMatrix, identityMatrix);
    history.completeFrame();

    history.beginFrame(2, 1.016, 1024, 600, identityMatrix, identityMatrix);
    require(!history.frame().historyValid, "resolution changes must invalidate history");
    history.completeFrame();

    history.beginFrame(3, 1.032, 1024, 600, translation(-150.0), identityMatrix);
    require(!history.frame().historyValid, "camera teleports must invalidate history");
    history.completeFrame();

    history.beginFrame(4, 1.048, 1024, 600, translation(-151.0), identityMatrix);
    require(history.frame().historyValid, "history should recover after a reset frame");
    history.completeFrame();
}

void testDisplayResolutionReset() {
    MotionHistory history;
    const Matrix4 identityMatrix = identity();
    history.beginFrame(1, 1.0, 400, 300, identityMatrix, identityMatrix, 800, 600);
    require(history.frame().width == 400 && history.frame().height == 300 &&
                history.frame().displayWidth == 800 && history.frame().displayHeight == 600,
            "render and display resolutions must be recorded independently");
    history.completeFrame();

    history.beginFrame(2, 1.016, 400, 300, identityMatrix, identityMatrix, 1600, 1200);
    require(!history.frame().historyValid,
            "display resolution changes must invalidate history even when render size is stable");
    history.completeFrame();
}

} // namespace

int main() {
    try {
        testCameraAndObjectMotion();
        testHistoryResetConditions();
        testDisplayResolutionReset();
        std::cout << "Motion data history checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Motion data history probe failed: " << error.what() << '\n';
        return 1;
    }
}
