#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <vector>

namespace openbus::assets {

struct DecodedWav {
    std::uint32_t sampleRate = 0;
    std::vector<std::int16_t> monoSamples;
};

std::optional<DecodedWav> decodeWavFile(const std::filesystem::path& path);

} // namespace openbus::assets
