#include "ProbeTempDirectory.h"
#include "audio/SoundEngine.h"
#include "audio/VehicleSoundBank.h"
#include "core/Logger.h"
#include "core/Utf8Path.h"
#include "core/Variables.h"

#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

class RecordingPlayback final : public SoundPlayback {
  public:
    explicit RecordingPlayback(SoundEngine& engine) : engine_(engine) {}

    SoundPlaybackHandle play(const std::filesystem::path& path, bool looped,
                             const SoundPlaybackParameters& parameters) override {
        ++attempts;
        lastHandle = engine_.play(path, looped, parameters);
        return lastHandle;
    }
    bool isPlaying(SoundPlaybackHandle handle) override { return engine_.isPlaying(handle); }
    void update(SoundPlaybackHandle handle, const SoundPlaybackParameters& parameters) override {
        engine_.update(handle, parameters);
    }
    void stop(SoundPlaybackHandle handle) override { engine_.stop(handle); }

    std::size_t attempts = 0;
    SoundPlaybackHandle lastHandle = 0;

  private:
    SoundEngine& engine_;
};

int main() {
    const auto root = openbus::probe::makeProbeDirectory("openbus-audio-recovery", false);
    const auto path = root / openbus::pathFromUtf8("recover-r\xc3\xb6hr-\xe6\xb8\xac.wav");
    std::ostringstream logs;
    auto* previous = std::cout.rdbuf(logs.rdbuf());
    bool passed = true;
    bool available = false;
    {
        SoundEngine sound;
        available = sound.isAvailable();
        if (available) {
            SoundPlaybackParameters parameters;
            for (int attempt = 0; attempt < 20; ++attempt) {
                passed = passed && sound.play(path, true, parameters) == 0;
            }
            {
                std::ofstream invalid(path, std::ios::binary);
                invalid << "not a wave";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1100));
            passed = passed && sound.play(path, true, parameters) == 0;
            constexpr std::array<unsigned char, 46> wav = {
                'R', 'I', 'F', 'F', 38, 0, 0, 0, 'W', 'A', 'V', 'E',
                'f', 'm', 't', ' ', 16, 0, 0, 0, 1, 0, 1, 0,
                0x80, 0xbb, 0, 0, 0, 0x77, 1, 0, 2, 0, 16, 0,
                'd', 'a', 't', 'a', 2, 0, 0, 0, 0, 0};
            {
                std::ofstream valid(path, std::ios::binary);
                valid.write(reinterpret_cast<const char*>(wav.data()), wav.size());
                passed = passed && static_cast<bool>(valid);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1100));
            const auto handle = sound.play(path, true, parameters);
            passed = passed && handle != 0;
            if (handle != 0) {
                sound.stop(handle);
            }
            const auto loopPath = root / "PENDING_LOOP.wav";
            const auto configPath = root / "pending.cfg";
            {
                std::ofstream config(configPath);
                config << "[sound]\npending_loop.wav\n1\n[trigger]\nev_pending\n";
                passed = passed && static_cast<bool>(config);
            }
            VehicleSoundBank bank;
            bank.load(configPath);
            Variables variables;
            RecordingPlayback playback(sound);
            constexpr auto viewpoint = openbus::rendering::ViewpointContext::PlayerExterior;
            bank.trigger(playback, "ev_pending", {}, 0.0F, variables, viewpoint);
            passed = passed && playback.attempts == 1 && playback.lastHandle == 0;
            {
                std::ofstream valid(loopPath, std::ios::binary);
                valid.write(reinterpret_cast<const char*>(wav.data()), wav.size());
                passed = passed && static_cast<bool>(valid);
            }
            bank.updateActiveSounds(playback, variables, viewpoint);
            passed = passed && playback.attempts == 1 && playback.lastHandle == 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(1100));
            bank.updateActiveSounds(playback, variables, viewpoint);
            passed = passed && playback.attempts == 2 && playback.lastHandle != 0 &&
                     sound.isPlaying(playback.lastHandle);
            bank.stopAll(sound);
        }
    }
    Logger::Flush();
    std::cout.rdbuf(previous);
    openbus::probe::removeProbeDirectory(root);
    if (!available) {
        std::cerr << "OpenAL is unavailable; audio recovery test skipped\n";
        return 77;
    }
    const std::string output = logs.str();
    const std::string warning = "Unable to decode sound:";
    const auto first = output.find(warning);
    const auto second =
        first == std::string::npos ? std::string::npos : output.find(warning, first + warning.size());
    if (!passed || first == std::string::npos ||
        output.find(openbus::pathToUtf8(path), first) == std::string::npos ||
        second == std::string::npos ||
        output.find(warning, second + warning.size()) != std::string::npos) {
        std::cerr << "Missing or invalid sounds did not recover with one warning per path:\n"
                  << output;
        return 1;
    }
    return 0;
}
