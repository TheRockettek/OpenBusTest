#include "render/MotionData.h"
#include "app/traffic/PassengerMotionKeys.h"

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

    history.invalidate();
    history.beginFrame(5, 0.5, 1024, 600, identityMatrix, identityMatrix);
    require(!history.frame().historyValid,
            "history invalidation must permit a new simulation clock epoch");
    history.completeFrame();
}

void testVehicleTransformDiscontinuity() {
    MotionHistory history;
    const Matrix4 identityMatrix = identity();
    const MotionObjectKey vehicleGroup{27, 2};

    history.beginFrame(1, 1.0, 800, 600, identityMatrix, identityMatrix);
    const auto first = history.previousModelView(translation(0.0), vehicleGroup, true, 5.0);
    require(!first.valid, "a vehicle's first rendered transform must have invalid history");
    history.completeFrame();

    history.beginFrame(2, 1.016, 800, 600, identityMatrix, identityMatrix);
    const auto normalMovement =
        history.previousModelView(translation(4.0), vehicleGroup, true, 5.0);
    require(normalMovement.valid,
            "vehicle movement within the displacement threshold must retain history");
    history.completeFrame();

    history.beginFrame(3, 1.032, 800, 600, identityMatrix, identityMatrix);
    const auto teleported =
        history.previousModelView(translation(20.0), vehicleGroup, true, 5.0);
    require(!teleported.valid,
            "vehicle movement beyond the displacement threshold must invalidate history");
    require(std::abs(teleported.previousModelView[12] - 20.0) < 1.0e-9,
            "a discontinuity must use the current transform as its previous-transform fallback");
    history.completeFrame();

    history.beginFrame(4, 1.048, 800, 600, identityMatrix, identityMatrix);
    const auto recovered =
        history.previousModelView(translation(21.0), vehicleGroup, true, 5.0);
    require(recovered.valid,
            "vehicle history must recover from the discontinuity using its new baseline");
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

void testFrameIdsMustIncrease() {
    MotionHistory history;
    const Matrix4 identityMatrix = identity();
    history.beginFrame(5, 1.0, 800, 600, identityMatrix, identityMatrix);
    history.completeFrame();

    bool duplicateRejected = false;
    try {
        history.beginFrame(5, 1.016, 800, 600, identityMatrix, identityMatrix);
    } catch (const std::invalid_argument&) {
        duplicateRejected = true;
    }
    require(duplicateRejected, "duplicate rendered frame IDs must be rejected");

    bool backwardRejected = false;
    try {
        history.beginFrame(4, 1.016, 800, 600, identityMatrix, identityMatrix);
    } catch (const std::invalid_argument&) {
        backwardRejected = true;
    }
    require(backwardRejected, "backward rendered frame IDs must be rejected");

    history.beginFrame(7, 1.032, 800, 600, identityMatrix, identityMatrix);
    require(!history.frame().historyValid,
            "a skipped rendered frame ID must invalidate temporal history");
    history.completeFrame();
}

void testPassengerMotionKeys() {
    using openbus::app::seatedPassengerMotionObjectKey;
    using openbus::app::waitingPassengerMotionObjectKey;

    const auto waiting = waitingPassengerMotionObjectKey(12, 4);
    require(waiting == waitingPassengerMotionObjectKey(12, 4) &&
                waiting != waitingPassengerMotionObjectKey(13, 4) &&
                waiting != waitingPassengerMotionObjectKey(12, 5) && !waiting.isStatic(),
            "waiting passenger motion keys must be stable and unique per stop/arrival");

    const auto seated = seatedPassengerMotionObjectKey(42, 3);
    require(seated == seatedPassengerMotionObjectKey(42, 3) &&
                seated != seatedPassengerMotionObjectKey(42, 4) &&
                seated != seatedPassengerMotionObjectKey(43, 3) && !seated.isStatic(),
            "seated passenger motion keys must be stable and unique per vehicle/seat");
}

} // namespace

int main() {
    try {
        testCameraAndObjectMotion();
        testHistoryResetConditions();
        testVehicleTransformDiscontinuity();
        testDisplayResolutionReset();
        testFrameIdsMustIncrease();
        testPassengerMotionKeys();
        std::cout << "Motion data history checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Motion data history probe failed: " << error.what() << '\n';
        return 1;
    }
}
