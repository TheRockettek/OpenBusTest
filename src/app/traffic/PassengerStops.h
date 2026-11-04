#pragma once

#include "map/MapTrafficRuntime.h"

#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <unordered_set>
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

template <typename IncludeStop>
std::vector<int> onwardPassengerStopIds(const PassengerStops& stops, std::size_t firstIndex,
                                        int originStopId, IncludeStop includeStop) {
    std::vector<int> result;
    std::unordered_set<int> seen;
    for (std::size_t index = firstIndex; index < stops.size(); ++index) {
        if (!includeStop(index)) {
            continue;
        }
        const int stopId = stops[index].stopId;
        if (stopId <= 0) {
            throw std::invalid_argument("Passenger destination stop must have a positive ID");
        }
        if (stopId != originStopId && seen.insert(stopId).second) {
            result.push_back(stopId);
        }
    }
    return result;
}

template <typename ResolvePosition>
void requestPassengerStopsOnRoute(openbus::map::MapPassengerServiceRuntime& service,
                                  const PassengerStops& stops, std::size_t firstIndex,
                                  const std::array<double, 3>& busPosition,
                                  ResolvePosition resolvePosition) {
    if (firstIndex > stops.size()) {
        throw std::invalid_argument("Passenger stop request index is outside the route");
    }
    for (std::size_t index = firstIndex; index < stops.size(); ++index) {
        const int stopId = stops[index].stopId;
        if (!service.hasUnrequestedPassengerStop(stopId)) {
            continue;
        }
        const auto position = resolvePosition(index);
        if (position) {
            service.requestStopForPassengers(stopId, busPosition, *position);
        }
    }
}

} // namespace openbus::app
