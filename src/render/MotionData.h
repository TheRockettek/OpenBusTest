#pragma once

#include "render/CameraMath.h"

#include <cstdint>
#include <limits>
#include <unordered_map>

namespace openbus::rendering {

struct MotionObjectKey {
    std::uint64_t object = 0;
    std::uint64_t subObject = 0;

    [[nodiscard]] bool isStatic() const noexcept {
        return object == 0;
    }
    bool operator==(const MotionObjectKey&) const = default;
};

[[nodiscard]] std::uint64_t allocateMotionObjectId() noexcept;

struct MotionTransform {
    Matrix4 previousModelView = {};
    bool valid = false;
};

struct MotionDataFrame {
    std::uint64_t frameId = 0;
    double timestampSeconds = 0.0;
    double deltaSeconds = 0.0;
    int width = 0;
    int height = 0;
    int displayWidth = 0;
    int displayHeight = 0;
    Matrix4 currentView = {};
    Matrix4 currentProjection = {};
    Matrix4 previousView = {};
    Matrix4 previousProjection = {};
    bool historyValid = false;
};

class MotionHistory {
  public:
    void beginFrame(std::uint64_t frameId, double timestampSeconds, int width, int height,
                    const Matrix4& view, const Matrix4& projection, int displayWidth = 0,
                    int displayHeight = 0);
    [[nodiscard]] MotionTransform previousModelView(
        const Matrix4& currentModelView, MotionObjectKey key = {}, bool geometryHistoryValid = true,
        double maximumTranslationDeltaMeters = std::numeric_limits<double>::infinity());
    void completeFrame();
    void invalidate() noexcept;

    [[nodiscard]] const MotionDataFrame& frame() const noexcept {
        return frame_;
    }

  private:
    struct ObjectHistory {
        Matrix4 worldTransform = {};
        std::uint64_t frameId = 0;
    };

    struct MotionObjectKeyHash {
        std::size_t operator()(const MotionObjectKey& key) const noexcept;
    };

    MotionDataFrame frame_;
    Matrix4 currentViewInverse_ = {};
    Matrix4 previousView_ = {};
    Matrix4 previousProjection_ = {};
    double previousTimestampSeconds_ = 0.0;
    int previousWidth_ = 0;
    int previousHeight_ = 0;
    int previousDisplayWidth_ = 0;
    int previousDisplayHeight_ = 0;
    std::uint64_t previousFrameId_ = 0;
    bool hasPreviousFrame_ = false;
    bool frameActive_ = false;
    std::unordered_map<MotionObjectKey, ObjectHistory, MotionObjectKeyHash> previousObjects_;
    std::unordered_map<MotionObjectKey, ObjectHistory, MotionObjectKeyHash> currentObjects_;
};

} // namespace openbus::rendering
