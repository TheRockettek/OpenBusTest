#pragma once

#include "map/MapTrafficRuntime.h"

#include <cstddef>
#include <optional>
#include <vector>

namespace openbus::app {

class ReflectionTrafficViews {
  public:
    void setNeeded(std::size_t index, bool needed) {
        if (index >= views_.size()) {
            views_.resize(index + 1);
        }
        views_[index].needed = needed;
    }

    void setView(std::size_t index, const openbus::map::MapTrafficCameraView& view) {
        views_.at(index).view = view;
    }

    std::vector<openbus::map::MapTrafficCameraView> snapshot() const {
        std::vector<openbus::map::MapTrafficCameraView> result;
        result.reserve(views_.size());
        for (const auto& entry : views_) {
            if (entry.needed && entry.view) {
                result.push_back(*entry.view);
            }
        }
        return result;
    }

    void clear() {
        views_.clear();
    }

  private:
    struct Entry {
        bool needed = false;
        std::optional<openbus::map::MapTrafficCameraView> view;
    };
    std::vector<Entry> views_;
};

} // namespace openbus::app
