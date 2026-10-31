#pragma once

namespace openbus::app {

class ScopedSwapInterval {
  public:
    using Setter = void (*)(int);

    ScopedSwapInterval(Setter setter, int interval, int restoreInterval)
        : setter_(setter), restoreInterval_(restoreInterval) {
        setter_(interval);
    }

    ~ScopedSwapInterval() {
        setter_(restoreInterval_);
    }

    ScopedSwapInterval(const ScopedSwapInterval&) = delete;
    ScopedSwapInterval& operator=(const ScopedSwapInterval&) = delete;

  private:
    Setter setter_;
    int restoreInterval_;
};

} // namespace openbus::app
