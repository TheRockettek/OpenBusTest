#include "app/FrameTiming.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

} // namespace

int main() {
    try {
        openbus::app::FrameTiming timing;
        timing.sampleFrameStart(0.0);
        require(timing.averageRenderFps() == 0.0, "one frame has no measurable rate");
        timing.sampleFrameStart(0.5);
        timing.sampleFrameStart(1.0);
        require(timing.averageRenderFps() == 2.0, "rolling frame rate counts intervals");
        timing.sampleFrameStart(4.5);
        require(timing.averageRenderFps() == 0.0, "expired samples leave no measurable rate");
        timing.sampleFrameStart(5.0);
        require(timing.averageRenderFps() == 2.0, "rolling frame rate recovers after a gap");

        timing.sampleDisplayFrame(10.0);
        timing.sampleDisplayFrame(10.5);
        require(timing.displayedFps() == 0.0, "display rate waits for a one-second sample");
        timing.sampleDisplayFrame(11.0);
        require(timing.displayedFps() == 3.0, "display rate counts frames in its sample");

        using Clock = openbus::app::FrameTiming::Clock;
        const Clock::time_point start{};
        require(!timing.deadlineAfterFrame(start), "uncapped frames have no deadline");
        timing.setFrameRateCap(10.0);
        const auto first = timing.deadlineAfterFrame(start);
        require(first && *first == start + std::chrono::milliseconds(100),
                "first frame deadline is one period ahead");
        const auto second = timing.deadlineAfterFrame(start + std::chrono::milliseconds(100));
        require(second && *second == start + std::chrono::milliseconds(200),
                "frame deadlines remain aligned");
        require(!timing.deadlineAfterFrame(start + std::chrono::milliseconds(500)),
                "late frames do not sleep");
        const auto resumed = timing.deadlineAfterFrame(start + std::chrono::milliseconds(500));
        require(resumed && *resumed == start + std::chrono::milliseconds(600),
                "late frames restart pacing from the current time");
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "frame timing probe failed: " << error.what() << '\n';
        return 1;
    }
}
