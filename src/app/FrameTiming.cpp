#include "app/FrameTiming.h"

namespace openbus::app {

void FrameTiming::setFrameRateCap(double framesPerSecond) {
    frameRateCap_ = framesPerSecond;
}

void FrameTiming::sampleFrameStart(double timeSeconds) {
    frameStartTimes_.push_back(timeSeconds);
    const double windowStart = timeSeconds - 3.0;
    while (frameStartTimes_.size() > 1 && frameStartTimes_.front() < windowStart) {
        frameStartTimes_.pop_front();
    }
    if (frameStartTimes_.size() >= 2) {
        const double duration = frameStartTimes_.back() - frameStartTimes_.front();
        if (duration > 0.0) {
            averageRenderFps_ = static_cast<double>(frameStartTimes_.size() - 1) / duration;
        }
    } else {
        averageRenderFps_ = 0.0;
    }
}

void FrameTiming::sampleDisplayFrame(double timeSeconds) {
    if (!displaySampleInitialized_) {
        displaySampleStart_ = timeSeconds;
        displayFrameCount_ = 0;
        displaySampleInitialized_ = true;
    }
    ++displayFrameCount_;
    const double duration = timeSeconds - displaySampleStart_;
    if (duration >= 1.0) {
        displayedFps_ = static_cast<double>(displayFrameCount_) / duration;
        displayFrameCount_ = 0;
        displaySampleStart_ = timeSeconds;
    }
}

std::optional<FrameTiming::Clock::time_point>
FrameTiming::deadlineAfterFrame(Clock::time_point now) {
    if (frameRateCap_ <= 0.0) {
        return std::nullopt;
    }
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / frameRateCap_));
    if (!deadlineInitialized_) {
        nextDeadline_ = now;
        deadlineInitialized_ = true;
    }
    nextDeadline_ += period;
    if (nextDeadline_ > now) {
        return nextDeadline_;
    }
    nextDeadline_ = now;
    return std::nullopt;
}

} // namespace openbus::app
