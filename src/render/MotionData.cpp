#include "render/MotionData.h"

#include <cmath>
#include <atomic>
#include <stdexcept>

namespace openbus::rendering {
namespace {

std::atomic<std::uint64_t> nextMotionObjectId{1};

Matrix4 identity() {
    return {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
}

Matrix4 multiply(const Matrix4& left, const Matrix4& right) {
    Matrix4 result = {};
    for (std::size_t column = 0; column < 4; ++column) {
        for (std::size_t row = 0; row < 4; ++row) {
            for (std::size_t inner = 0; inner < 4; ++inner) {
                result[row + column * 4] += left[row + inner * 4] * right[inner + column * 4];
            }
        }
    }
    return result;
}

Matrix4 inverseRigidTransform(const Matrix4& matrix) {
    Matrix4 inverse = identity();
    for (std::size_t column = 0; column < 3; ++column) {
        for (std::size_t row = 0; row < 3; ++row) {
            inverse[row + column * 4] = matrix[column + row * 4];
        }
    }
    for (std::size_t row = 0; row < 3; ++row) {
        inverse[row + 12] = -(inverse[row] * matrix[12] + inverse[row + 4] * matrix[13] +
                              inverse[row + 8] * matrix[14]);
    }
    return inverse;
}

bool finiteMatrix(const Matrix4& matrix) {
    for (const double value : matrix) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}

std::array<double, 3> cameraPosition(const Matrix4& view) {
    const Matrix4 inverse = inverseRigidTransform(view);
    return {inverse[12], inverse[13], inverse[14]};
}

std::array<double, 3> cameraForward(const Matrix4& view) {
    return {-view[2], -view[6], -view[10]};
}

bool cameraCut(const Matrix4& previousView, const Matrix4& currentView) {
    constexpr double maximumCameraStepMeters = 100.0;
    constexpr double minimumForwardDot = 0.5;
    const auto previousPosition = cameraPosition(previousView);
    const auto currentPosition = cameraPosition(currentView);
    const double dx = currentPosition[0] - previousPosition[0];
    const double dy = currentPosition[1] - previousPosition[1];
    const double dz = currentPosition[2] - previousPosition[2];
    if (std::hypot(dx, dy, dz) > maximumCameraStepMeters) {
        return true;
    }
    const auto previousForward = cameraForward(previousView);
    const auto currentForward = cameraForward(currentView);
    const double directionDot = previousForward[0] * currentForward[0] +
                                previousForward[1] * currentForward[1] +
                                previousForward[2] * currentForward[2];
    return directionDot < minimumForwardDot;
}

} // namespace

std::uint64_t allocateMotionObjectId() noexcept {
    std::uint64_t id = nextMotionObjectId.fetch_add(1, std::memory_order_relaxed);
    if (id == 0) {
        id = nextMotionObjectId.fetch_add(1, std::memory_order_relaxed);
    }
    return id;
}

std::size_t
MotionHistory::MotionObjectKeyHash::operator()(const MotionObjectKey& key) const noexcept {
    const std::size_t objectHash = std::hash<std::uint64_t>{}(key.object);
    const std::size_t subObjectHash = std::hash<std::uint64_t>{}(key.subObject);
    return objectHash ^ (subObjectHash + 0x9e3779b9U + (objectHash << 6U) + (objectHash >> 2U));
}

void MotionHistory::beginFrame(std::uint64_t frameId, double timestampSeconds, int width,
                               int height, const Matrix4& view, const Matrix4& projection,
                               int displayWidth, int displayHeight) {
    if (frameActive_) {
        throw std::logic_error(
            "Motion history frame was begun before the previous frame completed");
    }
    if (frameId == 0 || !std::isfinite(timestampSeconds) || width <= 0 || height <= 0 ||
        displayWidth < 0 || displayHeight < 0 || (displayWidth == 0) != (displayHeight == 0) ||
        !finiteMatrix(view) || !finiteMatrix(projection)) {
        throw std::invalid_argument("Motion history frame requires finite matrices and dimensions");
    }
    if (hasPreviousFrame_ && timestampSeconds < previousTimestampSeconds_) {
        throw std::invalid_argument("Motion history timestamps must be monotonic");
    }

    frame_ = {};
    frame_.frameId = frameId;
    frame_.timestampSeconds = timestampSeconds;
    frame_.deltaSeconds = hasPreviousFrame_ ? timestampSeconds - previousTimestampSeconds_ : 0.0;
    frame_.width = width;
    frame_.height = height;
    frame_.displayWidth = displayWidth > 0 ? displayWidth : width;
    frame_.displayHeight = displayHeight > 0 ? displayHeight : height;
    frame_.currentView = view;
    frame_.currentProjection = projection;
    currentViewInverse_ = inverseRigidTransform(view);
    frame_.previousView = hasPreviousFrame_ ? previousView_ : view;
    frame_.previousProjection = hasPreviousFrame_ ? previousProjection_ : projection;
    frame_.historyValid =
        hasPreviousFrame_ && frameId == previousFrameId_ + 1 && width == previousWidth_ &&
        height == previousHeight_ && frame_.displayWidth == previousDisplayWidth_ &&
        frame_.displayHeight == previousDisplayHeight_ && !cameraCut(previousView_, view);
    if (!frame_.historyValid) {
        previousObjects_.clear();
        frame_.previousView = view;
        frame_.previousProjection = projection;
    }
    currentObjects_.clear();
    frameActive_ = true;
}

MotionTransform MotionHistory::previousModelView(const Matrix4& currentModelView,
                                                 MotionObjectKey key, bool geometryHistoryValid) {
    if (!frameActive_) {
        throw std::logic_error("Motion history transform requested outside an active frame");
    }
    if (!finiteMatrix(currentModelView)) {
        throw std::invalid_argument("Motion history transform must be finite");
    }
    const Matrix4 currentWorld = multiply(currentViewInverse_, currentModelView);
    MotionTransform result;
    if (key.isStatic()) {
        result.previousModelView = multiply(frame_.previousView, currentWorld);
        result.valid = frame_.historyValid && geometryHistoryValid;
        return result;
    }

    const auto previous = previousObjects_.find(key);
    const bool objectHistoryValid = frame_.historyValid && previous != previousObjects_.end() &&
                                    previous->second.frameId + 1 == frame_.frameId &&
                                    geometryHistoryValid;
    const Matrix4& previousWorld =
        objectHistoryValid ? previous->second.worldTransform : currentWorld;
    result.previousModelView = multiply(frame_.previousView, previousWorld);
    result.valid = objectHistoryValid;
    currentObjects_.insert_or_assign(key, ObjectHistory{currentWorld, frame_.frameId});
    return result;
}

void MotionHistory::completeFrame() {
    if (!frameActive_) {
        throw std::logic_error("Motion history frame completed without being active");
    }
    previousView_ = frame_.currentView;
    previousProjection_ = frame_.currentProjection;
    previousTimestampSeconds_ = frame_.timestampSeconds;
    previousWidth_ = frame_.width;
    previousHeight_ = frame_.height;
    previousDisplayWidth_ = frame_.displayWidth;
    previousDisplayHeight_ = frame_.displayHeight;
    previousFrameId_ = frame_.frameId;
    hasPreviousFrame_ = true;
    previousObjects_.swap(currentObjects_);
    currentObjects_.clear();
    frameActive_ = false;
}

void MotionHistory::invalidate() noexcept {
    hasPreviousFrame_ = false;
    frameActive_ = false;
    previousObjects_.clear();
    currentObjects_.clear();
    frame_ = {};
}

} // namespace openbus::rendering
