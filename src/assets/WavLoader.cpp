#include "assets/WavLoader.h"

#include "audio/WavPcm.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>

namespace openbus::assets {
namespace {

std::uint32_t read32(const std::vector<std::uint8_t>& data, std::size_t offset) {
    return static_cast<std::uint32_t>(data[offset]) |
           (static_cast<std::uint32_t>(data[offset + 1]) << 8) |
           (static_cast<std::uint32_t>(data[offset + 2]) << 16) |
           (static_cast<std::uint32_t>(data[offset + 3]) << 24);
}

std::uint16_t read16(const std::vector<std::uint8_t>& data, std::size_t offset) {
    return static_cast<std::uint16_t>(data[offset]) |
           (static_cast<std::uint16_t>(data[offset + 1]) << 8);
}

} // namespace

std::optional<DecodedWav> decodeWavFile(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return std::nullopt;
    }
    const std::vector<std::uint8_t> data((std::istreambuf_iterator<char>(input)), {});
    if (input.bad() || data.size() < 12 || std::memcmp(data.data(), "RIFF", 4) != 0 ||
        std::memcmp(data.data() + 8, "WAVE", 4) != 0) {
        return std::nullopt;
    }

    std::uint16_t format = 0;
    std::uint16_t channels = 0;
    std::uint32_t rate = 0;
    std::uint16_t bits = 0;
    std::size_t dataOffset = 0;
    std::size_t dataSize = 0;
    for (std::size_t offset = 12; offset + 8 <= data.size();) {
        const std::uint32_t chunkSize = read32(data, offset + 4);
        const std::size_t payload = offset + 8;
        if (payload > data.size() || chunkSize > data.size() - payload) {
            return std::nullopt;
        }
        if (std::memcmp(data.data() + offset, "fmt ", 4) == 0 && chunkSize >= 16) {
            format = read16(data, payload);
            channels = read16(data, payload + 2);
            rate = read32(data, payload + 4);
            bits = read16(data, payload + 14);
        } else if (std::memcmp(data.data() + offset, "data", 4) == 0) {
            dataOffset = payload;
            dataSize = chunkSize;
        }
        const std::size_t paddedSize = static_cast<std::size_t>(chunkSize) + (chunkSize & 1u);
        if (paddedSize > data.size() - payload) {
            return std::nullopt;
        }
        offset = payload + paddedSize;
    }

    if ((format != 1 && format != 3) || (channels != 1 && channels != 2) || rate == 0 ||
        (format == 1 && bits != 8 && bits != 16 && bits != 24 && bits != 32) ||
        (format == 3 && bits != 32) || dataOffset == 0) {
        return std::nullopt;
    }
    const std::size_t bytesPerSample = bits / 8;
    const std::size_t frameBytes = bytesPerSample * channels;
    if (frameBytes == 0 || dataSize < frameBytes) {
        return std::nullopt;
    }
    const std::size_t frameCount = dataSize / frameBytes;
    if (frameCount > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max()) ||
        frameCount > (std::numeric_limits<std::size_t>::max)() / channels) {
        return std::nullopt;
    }

    DecodedWav decoded;
    decoded.sampleRate = rate;
    decoded.monoSamples.resize(frameCount * channels);
    for (std::size_t sample = 0; sample < decoded.monoSamples.size(); ++sample) {
        const std::size_t sampleOffset = dataOffset + sample * bytesPerSample;
        std::int32_t integer = 0;
        if (format == 3) {
            float value = 0.0f;
            std::memcpy(&value, data.data() + sampleOffset, sizeof(value));
            decoded.monoSamples[sample] =
                static_cast<std::int16_t>(std::lround(std::clamp(value, -1.0f, 1.0f) * 32767.0f));
            continue;
        }
        if (bits == 8) {
            integer = (static_cast<std::int32_t>(data[sampleOffset]) - 128) << 8;
        } else if (bits == 16) {
            integer = static_cast<std::int16_t>(read16(data, sampleOffset));
        } else if (bits == 24) {
            integer = static_cast<std::int32_t>(data[sampleOffset]) |
                      (static_cast<std::int32_t>(data[sampleOffset + 1]) << 8) |
                      (static_cast<std::int32_t>(data[sampleOffset + 2]) << 16);
            if ((integer & 0x00800000) != 0) {
                integer |= ~0x00ffffff;
            }
            integer >>= 8;
        } else {
            integer = audio::detail::signedPcm32HighWord(read32(data, sampleOffset));
        }
        decoded.monoSamples[sample] = static_cast<std::int16_t>(std::clamp(integer, -32768, 32767));
    }
    if (channels == 2) {
        for (std::size_t frame = 0; frame < frameCount; ++frame) {
            decoded.monoSamples[frame] = audio::detail::stereoPcm16ToMono(
                decoded.monoSamples[frame * 2], decoded.monoSamples[frame * 2 + 1]);
        }
        decoded.monoSamples.resize(frameCount);
    }
    return decoded;
}

} // namespace openbus::assets
