#pragma once

#include "SoundPlayback.h"

#include <array>
#include <filesystem>
#include <memory>

class SoundEngine : public SoundPlayback {
  public:
    SoundEngine();
    ~SoundEngine();
    SoundEngine(const SoundEngine&) = delete;
    SoundEngine& operator=(const SoundEngine&) = delete;

    void setListenerPose(const std::array<double, 3>& position,
                         const std::array<double, 3>& forward, const std::array<double, 3>& up);
    SoundPlaybackHandle play(const std::filesystem::path& path, bool looped,
                 const SoundPlaybackParameters& parameters) override;
    void updateLoop(SoundPlaybackHandle handle,
            const SoundPlaybackParameters& parameters) override;
    void stopLoop(SoundPlaybackHandle handle) override;

  private:
    struct Backend;
    std::unique_ptr<Backend> backend_;
};