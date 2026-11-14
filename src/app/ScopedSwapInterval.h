#pragma once

namespace openbus::app {

class ScopedSwapInterval {
  public:
    using Setter = void (*)(int);

    ScopedSwapInterval(Setter setter, int interval, const bool& restoreVsync)
        : setter_(setter), restoreVsync_(&restoreVsync) {
        setter_(interval);
    }

    ~ScopedSwapInterval() {
        setter_(*restoreVsync_ ? 1 : 0);
    }

    ScopedSwapInterval(const ScopedSwapInterval&) = delete;
    ScopedSwapInterval& operator=(const ScopedSwapInterval&) = delete;

  private:
    Setter setter_;
    const bool* restoreVsync_;
};

} // namespace openbus::app
