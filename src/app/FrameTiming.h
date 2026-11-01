#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <optional>

namespace openbus::app {

class FrameTiming {
  public:
    using Clock = std::chrono::steady_clock;

    void setFrameRateCap(double framesPerSecond);
    void sampleFrameStart(double timeSeconds);
    void sampleDisplayFrame(double timeSeconds);
    [[nodiscard]] double averageRenderFps() const noexcept {
        return averageRenderFps_;
    }
    [[nodiscard]] double displayedFps() const noexcept {
        return displayedFps_;
    }
    [[nodiscard]] std::optional<Clock::time_point> deadlineAfterFrame(Clock::time_point now);

  private:
    double frameRateCap_ = 0.0;
    std::deque<double> frameStartTimes_;
    double averageRenderFps_ = 0.0;
    double displaySampleStart_ = 0.0;
    double displayedFps_ = 0.0;
    std::uint64_t displayFrameCount_ = 0;
    bool displaySampleInitialized_ = false;
    Clock::time_point nextDeadline_{};
    bool deadlineInitialized_ = false;
};

} // namespace openbus::app
