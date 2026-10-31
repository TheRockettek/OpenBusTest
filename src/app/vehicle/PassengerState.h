#pragma once

#include "app/vehicle/VehicleDefinition.h"
#include "map/MapTrafficRuntime.h"

#include <memory>
#include <utility>

namespace openbus::app::vehicle {

struct PassengerState {
    explicit PassengerState(std::shared_ptr<const SharedVehicleDefinition> vehicleDefinition)
        : definition(std::move(vehicleDefinition)),
          service(definition->vehicleConfiguration.passengerPositions.size()) {}

    [[nodiscard]] const VehicleConfig& configuration() const {
        return definition->vehicleConfiguration;
    }

    std::shared_ptr<const SharedVehicleDefinition> definition;
    openbus::map::MapPassengerServiceRuntime service;
};

inline std::unique_ptr<PassengerState>
makePassengerState(const std::shared_ptr<const SharedVehicleDefinition>& definition,
                   bool passengerServiceRequired) {
    const VehicleConfig& config = definition->vehicleConfiguration;
    if (!passengerServiceRequired && config.passengerPositions.empty() &&
        config.passengerPathPoints.empty() && config.passengerPathLinks.empty() &&
        config.passengerEntryPathPoints.empty() && config.passengerExitPathPoints.empty()) {
        return nullptr;
    }
    return std::make_unique<PassengerState>(definition);
}

} // namespace openbus::app::vehicle
