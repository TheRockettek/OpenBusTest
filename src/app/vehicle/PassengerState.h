#pragma once

#include "app/vehicle/VehicleDefinition.h"
#include "core/PassengerVariables.h"
#include "map/MapTrafficRuntime.h"

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

struct Vehicle;

namespace openbus::app::vehicle {

struct PassengerCabinSection {
    std::shared_ptr<const SharedVehicleDefinition> definition;
    Vehicle* vehicle = nullptr;
    std::size_t firstSeatIndex = 0;
    std::size_t firstPathLinkIndex = 0;
};

struct PassengerState {
    explicit PassengerState(std::shared_ptr<const SharedVehicleDefinition> vehicleDefinition)
        : definition(std::move(vehicleDefinition)),
          service(definition->vehicleConfiguration.passengerPositions.size()) {
        sections.push_back({definition, nullptr, 0, 0});
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
    [[nodiscard]] const VehicleConfig& sectionConfiguration(std::size_t sectionIndex) const {
        return sections.at(sectionIndex).definition->vehicleConfiguration;
    }
    [[nodiscard]] Vehicle* sectionVehicle(std::size_t sectionIndex) const {
        return sections.at(sectionIndex).vehicle;
    }
    [[nodiscard]] std::size_t sectionIndexForSeat(std::size_t seatIndex) const {
        if (seatIndex >= service.seatCount()) {
            throw std::out_of_range("Passenger seat index is outside the articulated cabin");
        }
        for (std::size_t sectionIndex = 0; sectionIndex < sections.size(); ++sectionIndex) {
            const auto& section = sections[sectionIndex];
            const std::size_t sectionSeatCount =
                section.definition->vehicleConfiguration.passengerPositions.size();
            if (seatIndex >= section.firstSeatIndex &&
                seatIndex - section.firstSeatIndex < sectionSeatCount) {
                return sectionIndex;
            }
        }
        throw std::logic_error("Passenger seat has no owning cabin section");
    }
    [[nodiscard]] std::size_t localSeatIndex(std::size_t seatIndex) const {
        const auto& section = sections.at(sectionIndexForSeat(seatIndex));
        return seatIndex - section.firstSeatIndex;
    }
    [[nodiscard]] std::size_t globalPathLinkIndex(std::size_t sectionIndex,
                                                  std::size_t localLinkIndex) const {
        const auto& section = sections.at(sectionIndex);
        if (localLinkIndex >= section.definition->vehicleConfiguration.passengerPathLinks.size()) {
            throw std::out_of_range("Passenger path link index is outside its cabin section");
        }
        return section.firstPathLinkIndex + localLinkIndex;
    }
    void bindSectionVehicle(std::size_t sectionIndex, Vehicle* vehicle) {
        sections.at(sectionIndex).vehicle = vehicle;
    }
    void appendSection(std::shared_ptr<const SharedVehicleDefinition> sectionDefinition,
                       Vehicle* sectionVehicle) {
        if (!sectionDefinition) {
            throw std::invalid_argument("Passenger cabin section requires a vehicle definition");
        }
        const std::size_t firstSeatIndex = service.seatCount();
        const std::size_t firstPathLinkIndex =
            sections.back().firstPathLinkIndex +
            sections.back().definition->vehicleConfiguration.passengerPathLinks.size();
        const auto& sectionConfig = sectionDefinition->vehicleConfiguration;
        service.appendSeats(sectionConfig.passengerPositions.size());
        sections.push_back(
            {std::move(sectionDefinition), sectionVehicle, firstSeatIndex, firstPathLinkIndex});
        for (std::size_t index = 0; index < sectionConfig.passengerPositions.size(); ++index) {
            exitDoorsBySeat.push_back(passengerExitDoorForSeat(
                sectionConfig, index, openbus::scripting::passengerDoorSlotCount));
        }
    }
    [[nodiscard]] std::optional<std::size_t> entryDoor() const {
        return configuration().passengerEntryPathPoints.empty() ? std::nullopt
                                                                : std::optional<std::size_t>{0};
    }
    [[nodiscard]] std::optional<std::size_t> exitDoor() const {
        const auto seat = service.alightingSeatIndex();
        return seat ? exitDoorsBySeat.at(*seat) : std::nullopt;
    }
    [[nodiscard]] std::optional<std::size_t> exitDoorForSeat(std::size_t seatIndex) const {
        return exitDoorsBySeat.at(seatIndex);
    }
    [[nodiscard]] std::size_t exitSectionIndexForSeat(std::size_t seatIndex) const {
        return sectionIndexForSeat(seatIndex);
    }
    [[nodiscard]] std::optional<std::size_t>
    entryDoorForSection(std::size_t sectionIndex) const {
        return sectionConfiguration(sectionIndex).passengerEntryPathPoints.empty()
                   ? std::nullopt
                   : std::optional<std::size_t>{0};
    }

    std::shared_ptr<const SharedVehicleDefinition> definition;
    openbus::map::MapPassengerServiceRuntime service;
    std::vector<std::optional<std::size_t>> exitDoorsBySeat;
    std::vector<PassengerCabinSection> sections;
};

inline std::shared_ptr<PassengerState>
makePassengerState(const std::shared_ptr<const SharedVehicleDefinition>& definition,
                   bool passengerServiceRequired) {
    const VehicleConfig& config = definition->vehicleConfiguration;
    if (!passengerServiceRequired && config.passengerPositions.empty() &&
        config.passengerPathPoints.empty() && config.passengerPathLinks.empty() &&
        config.passengerEntryPathPoints.empty() && config.passengerExitPathPoints.empty()) {
        return {};
    }
    return std::make_shared<PassengerState>(definition);
}

} // namespace openbus::app::vehicle
