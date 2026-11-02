#include "audio/WavPcm.h"

#include <array>
#include <cstdint>
#include <iostream>
#include <utility>

int main() {
    constexpr std::array<std::pair<std::uint32_t, std::int32_t>, 6> samples = {{
        {0x80000000U, -32768},
        {0x8000ffffU, -32768},
        {0xffff0000U, -1},
        {0x00000000U, 0},
        {0x0000ffffU, 0},
        {0x7fff0000U, 32767},
    }};
    for (const auto& [input, expected] : samples) {
        if (openbus::audio::detail::signedPcm32HighWord(input) != expected) {
            std::cerr << "32-bit WAV PCM sign conversion failed for " << input << '\n';
            return 1;
        }
        constexpr std::array<std::array<std::int16_t, 3>, 5> stereoSamples = {{
            {32767, 32767, 32767},
            {-32768, -32768, -32768},
            {-20000, 20000, 0},
            {12000, 0, 6000},
            {0, -12000, -6000},
        }};
        for (const auto& sample : stereoSamples) {
            if (openbus::audio::detail::stereoPcm16ToMono(sample[0], sample[1]) != sample[2]) {
                std::cerr << "Stereo WAV downmix for positional playback failed\n";
                return 1;
            }
        }
    }
    return 0;
}
