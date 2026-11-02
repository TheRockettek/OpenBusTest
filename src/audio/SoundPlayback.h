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
    float rolloffFactor = 1.0F;
    float referenceDistance = 1.0F;
};

class SoundPlayback {
  public:
    virtual ~SoundPlayback() = default;

    virtual SoundPlaybackHandle play(const std::filesystem::path& path, bool looped,
                                     const SoundPlaybackParameters& parameters) = 0;
    virtual bool isPlaying(SoundPlaybackHandle handle) = 0;
    virtual void update(SoundPlaybackHandle handle, const SoundPlaybackParameters& parameters) = 0;
    virtual void stop(SoundPlaybackHandle handle) = 0;

    // Compatibility wrappers for callers that own loop handles.
    virtual void updateLoop(SoundPlaybackHandle handle, const SoundPlaybackParameters& params) {
        update(handle, params);
    }
    virtual void stopLoop(SoundPlaybackHandle handle) {
        stop(handle);
    }
};
