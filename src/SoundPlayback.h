#pragma once

#include <array>
#include <cstdint>
#include <filesystem>

using SoundPlaybackHandle = std::uint64_t;

struct SoundPlaybackParameters {
    float gain = 1.0F;
    float pitch = 1.0F;
    std::array<double, 3> position = {};
    double maxDistance = 0.0;
};

class SoundPlayback {
  public:
    virtual ~SoundPlayback() = default;

    virtual SoundPlaybackHandle play(const std::filesystem::path& path, bool looped,
                                     const SoundPlaybackParameters& parameters) = 0;
    virtual void updateLoop(SoundPlaybackHandle handle,
                            const SoundPlaybackParameters& parameters) = 0;
    virtual void stopLoop(SoundPlaybackHandle handle) = 0;
};
