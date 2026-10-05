#include "VehicleSoundBank.h"
#include "Variables.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

struct RecordedPlayback {
    SoundPlaybackHandle handle = 0;
    std::filesystem::path path;
    bool looped = false;
    SoundPlaybackParameters parameters;
};

class FakePlayback final : public SoundPlayback {
  public:
    SoundPlaybackHandle play(const std::filesystem::path& path, bool looped,
                             const SoundPlaybackParameters& parameters) override {
        const SoundPlaybackHandle handle = ++nextHandle;
        started.push_back({handle, path, looped, parameters});
        return handle;
    }

    void updateLoop(SoundPlaybackHandle handle,
                    const SoundPlaybackParameters& parameters) override {
        updated.push_back(handle);
        for (RecordedPlayback& playback : started) {
            if (playback.handle == handle) {
                playback.parameters = parameters;
            }
        }
    }

    void stopLoop(SoundPlaybackHandle handle) override { stopped.push_back(handle); }

    SoundPlaybackHandle nextHandle = 0;
    std::vector<RecordedPlayback> started;
    std::vector<SoundPlaybackHandle> updated;
    std::vector<SoundPlaybackHandle> stopped;
};

int main() {
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "openbus_vehicle_sound_probe";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    const std::filesystem::path configPath = root / "sound.cfg";
    {
        std::ofstream config(configPath);
        config << "[sound]\nHorn_loop.wav\n0.8\n"
                  "[trigger]\nev_horn\n"
                  "[sound]\nAmbient_loop.wav\n1\n"
                  "[volcurve]\nAmbientLevel\n"
                  "[pnt]\n0\n0\n"
                  "[pnt]\n1\n1\n"
                  "[sound]\nExplicitOneShot.wav\n0.4\n"
                  "[noloop]\n"
                  "[trigger]\nev_one_shot\n";
    }

    VehicleSoundBank firstVehicle;
    VehicleSoundBank secondVehicle;
    firstVehicle.load(configPath);
    secondVehicle.load(configPath);
    FakePlayback playback;
    Variables firstVariables;
    Variables secondVariables;
    firstVariables.set("ambientlevel", 1.0F);
    secondVariables.set("ambientlevel", 1.0F);
    constexpr auto exterior = openbus::rendering::ViewpointContext::PlayerExterior;

    firstVehicle.trigger(playback, "ev_horn", {}, 0.0F, firstVariables, exterior);
    secondVehicle.trigger(playback, "ev_horn", {}, 0.0F, secondVariables, exterior);
    firstVehicle.updateAmbient(playback, firstVariables, exterior);
    secondVehicle.updateAmbient(playback, secondVariables, exterior);

    const bool bothVehiclesStartedTheirOwnLoops =
        playback.started.size() == 4 && playback.started[0].looped &&
        playback.started[1].looped && playback.started[2].looped && playback.started[3].looped &&
        playback.started[0].path == playback.started[1].path &&
        playback.started[2].path == playback.started[3].path;

    firstVehicle.stop(playback, "ev_horn");
    const bool triggerStopIsScopedToFirstVehicle = playback.stopped.size() == 1 &&
                                                    playback.stopped[0] == playback.started[0].handle;

    firstVariables.set("ambientlevel", 0.0F);
    firstVehicle.updateAmbient(playback, firstVariables, exterior);
    const bool ambientStopIsScopedToFirstVehicle =
        playback.stopped.size() == 2 && playback.stopped[1] == playback.started[2].handle;

    firstVehicle.stopAllLoops(playback);
    const bool stopAllLeavesSecondVehicleLoopsAlone =
        playback.stopped.size() == 2 &&
        std::find(playback.stopped.begin(), playback.stopped.end(), playback.started[1].handle) ==
            playback.stopped.end() &&
        std::find(playback.stopped.begin(), playback.stopped.end(), playback.started[3].handle) ==
            playback.stopped.end();

    const bool triggerLookupIsLocal = firstVehicle.hasTrigger("ev_horn") &&
                                      !VehicleSoundBank{}.hasTrigger("ev_horn");
    firstVehicle.trigger(playback, "ev_one_shot", {}, 0.0F, firstVariables, exterior);
    const bool oneShotPlaybackWorks = playback.started.size() == 5 &&
                                      !playback.started.back().looped &&
                                      playback.started.back().parameters.gain == 0.4F;

    std::filesystem::remove_all(root);
    if (!bothVehiclesStartedTheirOwnLoops || !triggerStopIsScopedToFirstVehicle ||
        !ambientStopIsScopedToFirstVehicle || !stopAllLeavesSecondVehicleLoopsAlone ||
        !triggerLookupIsLocal || !oneShotPlaybackWorks) {
        std::cerr << "vehicle sound ownership or loop cancellation was not isolated\n";
        return 1;
    }
    return 0;
}
