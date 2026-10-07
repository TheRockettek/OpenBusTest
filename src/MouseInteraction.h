#pragma once

#include <string>
#include <utility>

namespace openbus::input {

// One captured mesh owns press/held-drag/release, even after the pointer leaves it.
// The host calls update once per input frame, not once per rendered view.
class MouseInteraction {
  public:
    template <class Press, class Release>
    void press(std::string event, double cursorX, double cursorY, Press&& onPress,
               Release&& onRelease) {
        cancel(onRelease);
        event_ = std::move(event);
        previousX_ = cursorX;
        previousY_ = cursorY;
        if (!event_.empty()) {
            onPress(event_);
        }
    }

    template <class Drag, class Release>
    void update(bool enabled, bool buttonDown, double cursorX, double cursorY, Drag&& onDrag,
                Release&& onRelease) {
        if (event_.empty()) {
            return;
        }
        if (!enabled || !buttonDown) {
            cancel(onRelease);
            return;
        }
        const double deltaX = cursorX - previousX_;
        const double deltaY = cursorY - previousY_;
        previousX_ = cursorX;
        previousY_ = cursorY;
        // Stationary holding is still a held interaction, with zero motion.
        onDrag(event_, deltaX, deltaY, cursorX, cursorY);
    }

    template <class Release> void cancel(Release&& onRelease) {
        if (!event_.empty()) {
            std::string released = std::exchange(event_, {});
            onRelease(released);
        }
    }

  private:
    std::string event_;
    double previousX_ = 0.0;
    double previousY_ = 0.0;
};

} // namespace openbus::input