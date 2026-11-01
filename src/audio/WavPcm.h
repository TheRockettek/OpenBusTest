#pragma once

#include <bit>
#include <cstdint>

namespace openbus::audio::detail {

inline std::int32_t signedPcm32HighWord(std::uint32_t sample) {
    return std::bit_cast<std::int16_t>(static_cast<std::uint16_t>(sample >> 16));
}

} // namespace openbus::audio::detail
