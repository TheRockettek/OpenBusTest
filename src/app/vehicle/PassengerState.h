#pragma once

#include "app/vehicle/VehicleDefinition.h"
#include "core/PassengerVariables.h"
#include "map/MapTrafficRuntime.h"

#include <memory>
#include <utility>

namespace openbus::app::vehicle {

struct PassengerState {
    explicit PassengerState(std::shared_ptr<const SharedVehicleDefinition> vehicleDefinition)
        : definition(std::move(vehicleDefinition)),
          service(definition->vehicleConfiguration.passengerPositions.size()) {
        const auto& config = configuration();
        exitDoorsBySeat.reserve(config.passengerPositions.size());
        for (std::size_t index = 0; index < config.passengerPositions.size(); ++index) {
            exitDoorsBySeat.push_back(passengerExitDoorForSeat(
                config, index, openbus::scripting::passengerDoorSlotCount));
        }
    }

    [[nodiscard]] const VehicleConfig& configuration() const {
        return definition->vehicleConfiguration;
    }
    [[nodiscard]] std::optional<std::size_t> entryDoor() const {
        return configuration().passengerEntryPathPoints.empty() ? std::nullopt
                                                                : std::optional<std::size_t>{0};
    }
    [[nodiscard]] std::optional<std::size_t> exitDoor() const {
        const auto seat = service.alightingSeatIndex();
        return seat ? exitDoorsBySeat.at(*seat) : std::nullopt;
    }

    std::shared_ptr<const SharedVehicleDefinition> definition;
    openbus::map::MapPassengerServiceRuntime service;
    std::vector<std::optional<std::size_t>> exitDoorsBySeat;
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
