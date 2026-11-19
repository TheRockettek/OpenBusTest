#pragma once

#include "render/MotionData.h"

#include <cstddef>
#include <cstdint>

namespace openbus::app {

inline constexpr std::uint64_t passengerMotionSubobjectBit = std::uint64_t{1} << 63U;

[[nodiscard]] inline openbus::rendering::MotionObjectKey
waitingPassengerMotionObjectKey(int stopId, std::size_t arrivalOrder) noexcept {
    const auto stopKey = static_cast<std::uint32_t>(stopId);
    return {passengerMotionSubobjectBit | stopKey, static_cast<std::uint64_t>(arrivalOrder)};
}

[[nodiscard]] inline openbus::rendering::MotionObjectKey
seatedPassengerMotionObjectKey(std::uint64_t vehicleObjectId, std::size_t seatIndex) noexcept {
    return {vehicleObjectId, passengerMotionSubobjectBit | static_cast<std::uint64_t>(seatIndex)};
}

} // namespace openbus::app
