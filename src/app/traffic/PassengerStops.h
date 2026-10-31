#pragma once

#include "map/MapTrafficRuntime.h"

#include <memory>
#include <utility>
#include <vector>

namespace openbus::app {

class PassengerStops {
  public:
    using Stop = openbus::map::MapTrafficPassengerStop;
    using Container = std::vector<Stop>;

    PassengerStops() : stops_(emptyStops()) {}
    explicit PassengerStops(Container stops)
        : stops_(std::make_shared<const Container>(std::move(stops))) {}

    [[nodiscard]] const Container& values() const {
        return *stops_;
    }
    [[nodiscard]] bool empty() const {
        return stops_->empty();
    }
    [[nodiscard]] std::size_t size() const {
        return stops_->size();
    }
    [[nodiscard]] const Stop& operator[](std::size_t index) const {
        return (*stops_)[index];
    }
    [[nodiscard]] const Stop& front() const {
        return stops_->front();
    }
    [[nodiscard]] const Stop& back() const {
        return stops_->back();
    }
    [[nodiscard]] auto begin() const {
        return stops_->begin();
    }
    [[nodiscard]] auto end() const {
        return stops_->end();
    }

    [[nodiscard]] static PassengerStops shareIfEqual(const Container& stops,
                                                     const PassengerStops& existing) {
        return stops == existing.values() ? existing : PassengerStops(stops);
    }

    [[nodiscard]] static PassengerStops shareIfEqual(Container&& stops,
                                                     const PassengerStops& existing) {
        return stops == existing.values() ? existing : PassengerStops(std::move(stops));
    }

  private:
    static const std::shared_ptr<const Container>& emptyStops() {
        static const auto empty = std::make_shared<const Container>();
        return empty;
    }
    std::shared_ptr<const Container> stops_;
};

} // namespace openbus::app
