#pragma once

#include <bit>
#include <cstdint>

namespace openbus::audio::detail {

inline std::int32_t signedPcm32HighWord(std::uint32_t sample) {
    return std::bit_cast<std::int16_t>(static_cast<std::uint16_t>(sample >> 16));
}

inline std::int16_t stereoPcm16ToMono(std::int16_t left, std::int16_t right) {
    return static_cast<std::int16_t>((static_cast<std::int32_t>(left) + right) / 2);
}

} // namespace openbus::audio::detail
