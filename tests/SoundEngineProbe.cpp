#include "VehicleSoundBank.h"
#include "Variables.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_set>
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
        ++playAttempts;
        if (failuresRemaining != 0) {
            --failuresRemaining;
            return 0;
        }
        const SoundPlaybackHandle handle = ++nextHandle;
        started.push_back({handle, path, looped, parameters});
        active.insert(handle);
        return handle;
    }

    bool isPlaying(SoundPlaybackHandle handle) override {
        queried.push_back(handle);
        return active.contains(handle);
    }

    void update(SoundPlaybackHandle handle,
                const SoundPlaybackParameters& parameters) override {
        if (!active.contains(handle)) {
            return;
        }
        updated.push_back(handle);
        for (RecordedPlayback& playback : started) {
            if (playback.handle == handle) {
                playback.parameters = parameters;
            }
        }
    }

    void stop(SoundPlaybackHandle handle) override {
        if (active.erase(handle) != 0) {
            stopped.push_back(handle);
        }
    }

    // Completion is explicit: no sleeping, wall-clock windows or real audio device.
    void complete(SoundPlaybackHandle handle) { active.erase(handle); }

    SoundPlaybackHandle nextHandle = 0;
    std::size_t playAttempts = 0;
    std::size_t failuresRemaining = 0;
    std::unordered_set<SoundPlaybackHandle> active;
    std::vector<RecordedPlayback> started;
    std::vector<SoundPlaybackHandle> queried;
    std::vector<SoundPlaybackHandle> updated;
    std::vector<SoundPlaybackHandle> stopped;
};

namespace {

constexpr auto exterior = openbus::rendering::ViewpointContext::PlayerExterior;
constexpr auto interior = openbus::rendering::ViewpointContext::PlayerInterior;

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
    }
    return condition;
}

bool close(float actual, float expected) {
    return std::abs(actual - expected) < 0.0001F;
}

bool checkOneShotLifecycle(const std::filesystem::path& configPath) {
    VehicleSoundBank first;
    VehicleSoundBank second;
    first.load(configPath);
    second.load(configPath);
    Variables variables;
    FakePlayback playback;
    first.trigger(playback, "ev_one_shot", {}, 0.0F, variables, exterior);
    for (int repeat = 0; repeat < 32; ++repeat) {
        first.trigger(playback, "ev_one_shot", {}, 0.0F, variables, exterior);
    }
    if (!expect(playback.started.size() == 1 && playback.active.size() == 1 &&
                    playback.updated.size() == 32 && playback.stopped.empty(),
                "same live clip must remain bounded to one updated voice")) {
        return false;
    }
    const SoundPlaybackHandle firstHandle = playback.started[0].handle;
    second.trigger(playback, "ev_one_shot", {}, 0.0F, variables, exterior);
    if (!expect(playback.started.size() == 2 && playback.active.size() == 2 &&
                    playback.started[0].path == playback.started[1].path &&
                    playback.started[0].handle != playback.started[1].handle,
                "one-shot ownership must be per vehicle, not global")) {
        return false;
    }
    const SoundPlaybackHandle secondHandle = playback.started[1].handle;

    playback.complete(firstHandle);
    first.trigger(playback, "ev_one_shot", {}, 0.0F, variables, exterior);
    bool passed = expect(playback.started.size() == 3 && playback.active.size() == 2 &&
                             playback.active.contains(secondHandle) && playback.stopped.empty(),
                         "completed one-shot must replay immediately without a timer");
    for (int repeat = 0; repeat < 8; ++repeat) {
        first.trigger(playback, "ev_one_shot", {}, 0.0F, variables, exterior,
                      SoundRetriggerPolicy::Restart);
    }
    passed &= expect(playback.started.size() == 11 && playback.stopped.size() == 8 &&
                         playback.active.size() == 2 && playback.active.contains(secondHandle),
                     "explicit Restart must restart every rapid same-file request");
    first.stop(playback, "ev_one_shot");
    passed &= expect(playback.stopped.size() == 9 && playback.active.size() == 1 &&
                         playback.active.contains(secondHandle),
                     "stop(name) must cancel one-shots without touching another vehicle");

    first.trigger(playback, "ev_one_shot", {}, 0.0F, variables, exterior);
    const SoundPlaybackHandle completedHandle = playback.started.back().handle;
    playback.complete(completedHandle);
    const std::size_t queriesBeforePrune = playback.queried.size();
    first.updateAmbient(playback, variables, exterior);
    passed &= expect(playback.queried.size() == queriesBeforePrune + 1 &&
                         playback.queried.back() == completedHandle,
                     "ambient update must inspect completed one-shot ownership");
    const std::size_t queriesAfterPrune = playback.queried.size();
    first.updateAmbient(playback, variables, exterior);
    first.trigger(playback, "ev_one_shot", {}, 0.0F, variables, exterior);
    passed &= expect(playback.queried.size() == queriesAfterPrune &&
                         playback.started.size() == 13 && playback.active.size() == 2,
                     "pruned handles must not be retained or delay immediate replay");
    first.stopAll(playback);
    second.stopAll(playback);
    return passed && expect(playback.active.empty(), "lifecycle cleanup must stop all owned voices");
}

bool checkDefinitionOwnership(const std::filesystem::path& configPath) {
    VehicleSoundBank bank;
    bank.load(configPath);
    Variables variables;
    FakePlayback playback;
    bank.trigger(playback, "ev_shared_a", {}, 0.0F, variables, exterior);
    bank.trigger(playback, "ev_shared_b", {}, 0.0F, variables, exterior);
    if (!expect(playback.started.size() == 2 && playback.active.size() == 2 &&
                    playback.started[0].path == playback.started[1].path &&
                    close(playback.started[0].parameters.gain, 0.25F) &&
                    close(playback.started[1].parameters.gain, 0.75F),
                "separate sound definitions sharing a filename must stay independent")) {
        return false;
    }
    const SoundPlaybackHandle sharedA = playback.started[0].handle;
    const SoundPlaybackHandle sharedB = playback.started[1].handle;
    bank.trigger(playback, "ev_shared_a", {}, 0.0F, variables, exterior);
    bank.stop(playback, "ev_shared_a");
    bool passed = expect(playback.started.size() == 2 && playback.updated.size() == 1 &&
                             playback.updated[0] == sharedA && playback.active.size() == 1 &&
                             playback.active.contains(sharedB),
                         "same-file definition update/stop must not affect its neighbor");
    bank.stopAll(playback);

    bank.trigger(playback, "ev_multiple", {}, 0.0F, variables, exterior);
    if (!expect(playback.started.size() == 4 && playback.active.size() == 2 &&
                    close(playback.started[2].parameters.gain, 0.2F) &&
                    close(playback.started[3].parameters.gain, 0.7F) &&
                    playback.started[2].parameters.position == std::array<double, 3>{} &&
                    playback.started[3].parameters.position == std::array<double, 3>{2, -1, 3} &&
                    playback.started[3].parameters.maxDistance == 25,
                "multiple same-file blocks on one trigger must retain distinct metadata and IDs")) {
        return false;
    }
    const SoundPlaybackHandle multipleA = playback.started[2].handle;
    const SoundPlaybackHandle multipleB = playback.started[3].handle;
    for (int repeat = 0; repeat < 16; ++repeat) {
        bank.trigger(playback, "ev_multiple", {}, 0.0F, variables, exterior);
        bank.trigger(playback, "ev_multiple_alias", {}, 0.0F, variables, exterior);
    }
    passed &= expect(playback.started.size() == 4 && playback.active.size() == 2,
                     "multiple definitions and aliases must not accumulate voices");
    bank.stop(playback, "ev_multiple_alias");
    passed &= expect(playback.active.size() == 1 && playback.active.contains(multipleA) &&
                         !playback.active.contains(multipleB),
                     "alias stop must cancel only the block owning that alias");
    bank.trigger(playback, "ev_multiple_alias", {}, 0.0F, variables, exterior);
    bank.trigger(playback, "ev_multiple", {}, 0.0F, variables, exterior);
    passed &= expect(playback.started.size() == 5 && playback.active.size() == 2,
                     "triggering through an alias and primary name must share block ownership");
    bank.stop(playback, "ev_multiple");
    passed &= expect(playback.active.empty(), "stop(name) must cancel all definitions on that name");

    bank.trigger(playback, "ev_alias_a", {}, 0.0F, variables, exterior);
    const SoundPlaybackHandle aliasHandle = playback.started.back().handle;
    const std::size_t startsBeforeAlias = playback.started.size();
    bank.trigger(playback, "EV_ALIAS_B", {}, 0.0F, variables, exterior);
    passed &= expect(playback.started.size() == startsBeforeAlias &&
                         playback.updated.back() == aliasHandle && playback.active.size() == 1,
                     "two case-insensitive aliases on a sound block must share one live voice");
    bank.stop(playback, "ev_alias_b");
    passed &= expect(playback.active.empty(), "either alias must stop the shared one-shot");
    bank.trigger(playback, "ev_alias_b", {}, 0.0F, variables, exterior);
    bank.trigger(playback, "ev_alias_a", {}, 0.0F, variables, exterior);
    passed &= expect(playback.started.size() == startsBeforeAlias + 1 &&
                         playback.active.size() == 1,
                     "alias ownership must work regardless of which name triggers first");
    bank.stopAll(playback);
    return passed;
}

bool checkFileOverrides(const std::filesystem::path& configPath) {
    VehicleSoundBank bank;
    bank.load(configPath);
    Variables variables;
    FakePlayback playback;
    variables.set("enabled", 1.0F);
    bank.trigger(playback, "ev_configured", "replacement_loop.wav", 0.5F, variables, exterior);
    variables.set("enabled", 0.0F);
    bank.trigger(playback, "ev_configured", "replacement_loop.wav", 0.5F, variables, interior);
    bool passed = expect(playback.playAttempts == 0,
                         "T.F must preserve viewpoint and condition gating");
    variables.set("enabled", 1.0F);
    bank.trigger(playback, "ev_configured", {}, 0.0F, variables, interior);
    passed &= expect(playback.playAttempts == 0, "silent new one-shots must remain skipped");
    bank.trigger(playback, "ev_configured", {}, 0.5F, variables, interior);
    if (!expect(playback.started.size() == 1 &&
                    close(playback.started[0].parameters.gain, 0.4F),
                "configured one-shot must apply base gain and volume curve")) {
        return false;
    }
    const SoundPlaybackHandle originalHandle = playback.started[0].handle;
    bank.trigger(playback, "ev_configured", "replacement_loop.wav", 0.25F, variables, interior);
    if (!expect(playback.started.size() == 2 && playback.stopped.size() == 1 &&
                    playback.stopped[0] == originalHandle && playback.active.size() == 1 &&
                    playback.started[1].path == configPath.parent_path() / "replacement_loop.wav" &&
                    !playback.started[1].looped &&
                    close(playback.started[1].parameters.gain, 0.2F) &&
                    playback.started[1].parameters.position == std::array<double, 3>{2, -1, 3} &&
                    playback.started[1].parameters.maxDistance == 40,
                "T.F must replace its owned voice and retain gain, curve, 3d and one-shot mode")) {
        return false;
    }
    const SoundPlaybackHandle replacementHandle = playback.started[1].handle;
    bank.trigger(playback, "ev_configured_alias", "./replacement_loop.wav", 0.75F, variables,
                 interior);
    passed &= expect(playback.started.size() == 2 && playback.updated.size() == 1 &&
                         playback.updated[0] == replacementHandle &&
                         close(playback.started[1].parameters.gain, 0.6F),
                     "same normalized override through an alias must update the stable-ID voice");
    variables.set("enabled", 0.0F);
    bank.trigger(playback, "ev_configured_alias", "blocked.wav", 0.5F, variables, interior);
    variables.set("enabled", 1.0F);
    bank.trigger(playback, "ev_configured", "blocked.wav", 0.5F, variables, exterior);
    passed &= expect(playback.started.size() == 2 && playback.stopped.size() == 1 &&
                         playback.active.contains(replacementHandle),
                     "ineligible override must not replace a playing configured voice");
    bank.trigger(playback, "ev_configured_alias", "replacement_loop.wav", 0.0F, variables, interior);
    passed &= expect(playback.started.size() == 2 && playback.active.contains(replacementHandle) &&
                         close(playback.started[1].parameters.gain, 0.0F),
                     "KeepPlaying must update a live one-shot even when its gain becomes zero");
    bank.trigger(playback, "ev_configured_alias", "replacement_loop.wav", 0.75F, variables, interior);
    passed &= expect(playback.started.size() == 2 && playback.active.contains(replacementHandle) &&
                         close(playback.started[1].parameters.gain, 0.6F),
                     "KeepPlaying must restore gain on the same still-active voice without restarting");
    bank.trigger(playback, "ev_configured", "replacement_loop.wav", 0.25F, variables, interior,
                 SoundRetriggerPolicy::Restart);
    passed &= expect(playback.started.size() == 3 && playback.stopped.size() == 2 &&
                         playback.active.size() == 1,
                     "explicit Restart must also restart a same-file T.F override");
    bank.trigger(playback, "ev_configured", {}, 0.5F, variables, interior);
    passed &= expect(playback.started.size() == 4 && playback.stopped.size() == 3 &&
                         playback.started.back().path == configPath.parent_path() / "Configured.wav" &&
                         close(playback.started.back().parameters.gain, 0.4F),
                     "returning to the configured file must replace the override on the same ID");
    bank.stop(playback, "ev_configured_alias");
    passed &= expect(playback.active.empty(), "alias stop must cancel an overridden one-shot");

    const std::size_t startsBeforeMultiple = playback.started.size();
    bank.trigger(playback, "ev_multiple", "multi_override_loop.wav", 0.0F, variables, exterior);
    if (!expect(playback.started.size() == startsBeforeMultiple + 2 && playback.active.size() == 2,
                "T.F on a multi-definition trigger must preserve all configured channels")) {
        return false;
    }
    const RecordedPlayback& first = playback.started[startsBeforeMultiple];
    const RecordedPlayback& second = playback.started[startsBeforeMultiple + 1];
    passed &= expect(!first.looped && !second.looped && first.path == second.path &&
                         close(first.parameters.gain, 0.2F) && close(second.parameters.gain, 0.7F) &&
                         second.parameters.maxDistance == 25,
                     "same override file must not merge definitions or infer a loop from its name");
    bank.trigger(playback, "ev_multiple", "multi_override_loop.wav", 0.0F, variables, exterior);
    passed &= expect(playback.started.size() == startsBeforeMultiple + 2 &&
                         playback.active.size() == 2,
                     "same active multi-definition override must stay bounded");
    bank.stopAll(playback);
    return passed;
}

bool checkTriggeredLoops(const std::filesystem::path& configPath) {
    VehicleSoundBank bank;
    bank.load(configPath);
    Variables variables;
    FakePlayback playback;
    bank.trigger(playback, "ev_loop_a", {}, 0.0F, variables, exterior);
    bank.trigger(playback, "ev_loop_b", {}, 0.0F, variables, exterior);
    if (!expect(playback.started.size() == 2 && playback.active.size() == 2 &&
                    playback.started[0].looped && playback.started[1].looped &&
                    playback.started[0].path == playback.started[1].path &&
                    close(playback.started[0].parameters.gain, 0.3F) &&
                    close(playback.started[1].parameters.gain, 0.6F),
                "triggered loop definitions sharing a file must own independent handles")) {
        return false;
    }
    const SoundPlaybackHandle firstLoop = playback.started[0].handle;
    const SoundPlaybackHandle secondLoop = playback.started[1].handle;
    bank.trigger(playback, "ev_loop_a_alias", {}, 0.0F, variables, exterior);
    bool passed = expect(playback.started.size() == 2 && playback.updated.size() == 1 &&
                             playback.updated[0] == firstLoop,
                         "triggered loop aliases must update the same definition");
    bank.trigger(playback, "ev_loop_a_alias", "replacement.wav", 0.0F, variables, exterior);
    passed &= expect(playback.started.size() == 3 && playback.stopped.size() == 1 &&
                         playback.stopped[0] == firstLoop && playback.started.back().looped &&
                         playback.active.size() == 2 && playback.active.contains(secondLoop),
                     "loop file change must stop only its old source and preserve configured looping");
    bank.trigger(playback, "ev_loop_a", "replacement.wav", 0.0F, variables, exterior,
                 SoundRetriggerPolicy::Restart);
    passed &= expect(playback.started.size() == 3 && playback.stopped.size() == 1,
                     "one-shot Restart policy must not change existing same-file loop behavior");
    bank.stop(playback, "ev_loop_a");
    passed &= expect(playback.active.size() == 1 && playback.active.contains(secondLoop),
                     "stop(name) must find an overridden loop by ID rather than configured filename");
    bank.stopAllLoops(playback);
    passed &= expect(playback.active.empty(), "stopAllLoops must cancel remaining triggered loops");

    bank.trigger(playback, "ev_explicit_loop", {}, 0.0F, variables, exterior);
    const SoundPlaybackHandle explicitLoop = playback.started.back().handle;
    bank.trigger(playback, "ev_explicit_loop", {}, 0.0F, variables, exterior);
    passed &= expect(playback.started.size() == 4 && playback.started.back().looped &&
                         playback.updated.back() == explicitLoop,
                     "explicit loopsound blocks must also have stable ownership IDs");
    bank.trigger(playback, "ev_explicit_loop", "changed.wav", 0.0F, variables, exterior);
    passed &= expect(playback.started.size() == 5 && playback.started.back().looped &&
                         playback.active.size() == 1 && !playback.active.contains(explicitLoop),
                     "explicit loopsound file override must replace the old source without changing mode");
    bank.stopAll(playback);
    return passed;
}

bool checkFailuresAndUnknownTriggers(const std::filesystem::path& configPath) {
    VehicleSoundBank bank;
    bank.load(configPath);
    Variables variables;
    FakePlayback playback;
    for (int index = 0; index < 32; ++index) {
        bank.trigger(playback, "unknown_" + std::to_string(index), "unconfigured.wav", 1.0F,
                     variables, exterior);
    }
    bank.trigger(playback, "unknown", {}, 1.0F, variables, exterior);
    bool passed = expect(playback.playAttempts == 0 && playback.active.empty() &&
                             playback.queried.empty(),
                         "unknown triggers must be skipped even with a supplied file");

    playback.failuresRemaining = 2;
    bank.trigger(playback, "ev_shared_a", {}, 0.0F, variables, exterior);
    bank.trigger(playback, "ev_shared_a", {}, 0.0F, variables, exterior);
    passed &= expect(playback.playAttempts == 2 && playback.started.empty() &&
                         playback.queried.empty() && playback.updated.empty(),
                     "failed one-shot handles must not be retained or queried");
    bank.trigger(playback, "ev_shared_a", {}, 0.0F, variables, exterior);
    if (!expect(playback.playAttempts == 3 && playback.started.size() == 1 &&
                    playback.active.size() == 1,
                "failed one-shot playback must retry immediately")) {
        return false;
    }
    const SoundPlaybackHandle original = playback.started[0].handle;
    playback.failuresRemaining = 1;
    bank.trigger(playback, "ev_shared_a", "replacement.wav", 0.0F, variables, exterior);
    passed &= expect(playback.playAttempts == 4 && playback.started.size() == 1 &&
                         playback.active.empty() && playback.stopped.size() == 1 &&
                         playback.stopped[0] == original,
                     "failed replacement must release the old voice without retaining handle zero");
    bank.trigger(playback, "ev_shared_a", "replacement.wav", 0.0F, variables, exterior);
    passed &= expect(playback.playAttempts == 5 && playback.started.size() == 2 &&
                         playback.active.size() == 1,
                     "failed T.F replacement must retry immediately");

    playback.failuresRemaining = 2;
    bank.trigger(playback, "ev_loop_a", {}, 0.0F, variables, exterior);
    bank.trigger(playback, "ev_loop_a_alias", {}, 0.0F, variables, exterior);
    bank.trigger(playback, "ev_loop_a", {}, 0.0F, variables, exterior);
    passed &= expect(playback.playAttempts == 8 && playback.started.size() == 3 &&
                         playback.started.back().looped && playback.active.size() == 2,
                     "failed triggered loops must retry through aliases without storing handle zero");
    bank.trigger(playback, "unknown_active", "replacement.wav", 0.0F, variables, exterior);
    bank.stop(playback, "unknown_active");
    passed &= expect(playback.playAttempts == 8 && playback.active.size() == 2 &&
                         std::none_of(playback.queried.begin(), playback.queried.end(),
                                      [](SoundPlaybackHandle handle) { return handle == 0; }),
                     "unknown file triggers must not affect configured voices or create ownership");
    bank.stopAll(playback);
    return passed && expect(playback.active.empty(), "failure probe must leave no active sources");
}

bool checkScopedStopAll(const std::filesystem::path& configPath) {
    VehicleSoundBank first;
    VehicleSoundBank second;
    first.load(configPath);
    second.load(configPath);
    Variables variables;
    variables.set("ambientlevel", 1.0F);
    FakePlayback playback;
    first.trigger(playback, "ev_horn", {}, 0.0F, variables, exterior);
    second.trigger(playback, "ev_horn", {}, 0.0F, variables, exterior);
    first.updateAmbient(playback, variables, exterior);
    second.updateAmbient(playback, variables, exterior);
    first.trigger(playback, "ev_one_shot", {}, 0.0F, variables, exterior);
    second.trigger(playback, "ev_one_shot", {}, 0.0F, variables, exterior);
    if (!expect(playback.started.size() == 6 && playback.active.size() == 6,
                "scoped cancellation fixture must start all three source types for each vehicle")) {
        return false;
    }
    const SoundPlaybackHandle firstOneShot = playback.started[4].handle;
    const SoundPlaybackHandle secondTriggered = playback.started[1].handle;
    const SoundPlaybackHandle secondAmbient = playback.started[3].handle;
    const SoundPlaybackHandle secondOneShot = playback.started[5].handle;
    first.stopAllLoops(playback);
    bool passed = expect(playback.active.size() == 4 && playback.active.contains(firstOneShot) &&
                             playback.active.contains(secondTriggered) &&
                             playback.active.contains(secondAmbient) &&
                             playback.active.contains(secondOneShot),
                         "stopAllLoops must leave one-shots and other vehicles alone");
    first.stopAll(playback);
    passed &= expect(playback.active.size() == 3 && playback.stopped.size() == 3 &&
                         playback.active.contains(secondTriggered) &&
                         playback.active.contains(secondAmbient) &&
                         playback.active.contains(secondOneShot),
                     "stopAll must cancel only this vehicle's remaining owned sources");
    first.stopAll(playback);
    passed &= expect(playback.stopped.size() == 3, "stopAll must be idempotent");
    second.stopAll(playback);
    passed &= expect(playback.active.empty() && playback.stopped.size() == 6,
                     "stopAll must cancel triggered loops, ambient loops and one-shots");
    // Follow the reload contract: stopAll first, since load has no playback argument.
    first.load(configPath);
    first.trigger(playback, "ev_one_shot", {}, 0.0F, variables, exterior);
    passed &= expect(playback.started.size() == 7 && playback.active.size() == 1,
                     "reload after explicit cancellation must start fresh ownership");
    first.stopAll(playback);
    return passed;
}

bool checkFilenameHeuristics(const std::filesystem::path& configPath) {
    VehicleSoundBank bank;
    bank.load(configPath);
    Variables variables;
    FakePlayback playback;
    // Preserve OpenBus filename heuristics without asserting native OMSI parity.
    bank.trigger(playback, "ev_motor_start", {}, 0.0F, variables, exterior);
    bank.updateAmbient(playback, variables, exterior);
    if (!expect(playback.started.size() == 3 && !playback.started[0].looped &&
                    playback.started[1].looped && playback.started[2].looped,
                "existing start-family and ambient loop heuristics must remain available")) {
        return false;
    }
    const SoundPlaybackHandle inferredLoop = playback.started[1].handle;
    const SoundPlaybackHandle ambientLoop = playback.started[2].handle;
    bank.trigger(playback, "ev_motor_end", {}, 0.0F, variables, exterior);
    bool passed = expect(playback.started.size() == 4 && playback.stopped.size() == 2 &&
                             !playback.active.contains(inferredLoop) &&
                             !playback.active.contains(ambientLoop),
                         "end-family heuristic must cancel ID-owned inferred and ambient loops");
    bank.stop(playback, "ev_motor_start");
    bank.stop(playback, "ev_motor_end");
    passed &= expect(playback.active.empty(), "family trigger stops must cancel owned one-shots too");

    const std::size_t startsBeforeOverride = playback.started.size();
    bank.trigger(playback, "ev_motor_start", "different.wav", 0.0F, variables, exterior);
    passed &= expect(playback.started.size() == startsBeforeOverride + 1 &&
                         !playback.started.back().looped && playback.active.size() == 1,
                     "T.F must not infer additional start-family loops");
    bank.stop(playback, "ev_motor_start");
    bank.trigger(playback, "ev_motor_start", {}, 0.0F, variables, exterior);
    passed &= expect(playback.active.size() == 2, "ordinary start trigger must still infer its loop");
    bank.stop(playback, "ev_motor_start");
    return passed && expect(playback.active.empty(), "stop(start) must cancel its inferred ID-owned loop");
}

} // namespace

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

    const std::filesystem::path ownershipConfigPath = root / "ownership.cfg";
    {
        std::ofstream config(ownershipConfigPath);
        config << "[sound]\nShared.wav\n0.25\n[trigger]\nev_shared_a\n"
                  "[sound]\nShared.wav\n0.75\n[trigger]\nev_shared_b\n"
                  "[sound]\nMultiple.wav\n0.2\n[trigger]\nev_multiple\n"
                  "[sound]\nMultiple.wav\n0.7\n[trigger]\nev_multiple\n"
                  "[trigger]\nev_multiple_alias\n[viewpoint]\n1\n[3d]\n1\n2\n3\n25\n"
                  "[sound]\nAlias.wav\n0.6\n[trigger]\nev_alias_a\n[trigger]\nev_alias_b\n"
                  "[sound]\nConfigured.wav\n0.8\n[trigger]\nev_configured\n"
                  "[trigger]\nev_configured_alias\n[viewpoint]\n2\n[3d]\n1\n2\n3\n40\n"
                  "[conditionSingle]\nenabled\n1\n1\n"
                  "[volcurve]\ncontrol\n[pnt]\n0\n0\n[pnt]\n1\n1\n"
                  "[loopsound]\nExplicitLoop.wav\n1\ncontrol\n1\n1\n"
                  "[trigger]\nev_explicit_loop\n"
                  "[sound]\nShared_loop.wav\n0.3\n[trigger]\nev_loop_a\n"
                  "[trigger]\nev_loop_a_alias\n"
                  "[sound]\nShared_loop.wav\n0.6\n[trigger]\nev_loop_b\n";
    }
    const std::filesystem::path heuristicConfigPath = root / "heuristics.cfg";
    {
        std::ofstream config(heuristicConfigPath);
        config << "[sound]\nmotor_start.wav\n0.5\n[trigger]\nev_motor_start\n"
                  "[sound]\nmotor_loop.wav\n0.5\n"
                  "[sound]\nmotor_end.wav\n0.5\n[trigger]\nev_motor_end\n";
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
    firstVehicle.trigger(playback, "ev_one_shot", {}, 0.0F, firstVariables, exterior);
    const bool repeatedOneShotIsCoalesced = playback.started.size() == 5;
    secondVehicle.trigger(playback, "ev_one_shot", {}, 0.0F, secondVariables, exterior);
    const bool oneShotStateIsVehicleLocal = playback.started.size() == 6;

    bool extendedChecksPassed = checkOneShotLifecycle(configPath);
    extendedChecksPassed &= checkDefinitionOwnership(ownershipConfigPath);
    extendedChecksPassed &= checkFileOverrides(ownershipConfigPath);
    extendedChecksPassed &= checkTriggeredLoops(ownershipConfigPath);
    extendedChecksPassed &= checkFailuresAndUnknownTriggers(ownershipConfigPath);
    extendedChecksPassed &= checkScopedStopAll(configPath);
    extendedChecksPassed &= checkFilenameHeuristics(heuristicConfigPath);
    firstVehicle.stopAll(playback);
    secondVehicle.stopAll(playback);

    std::filesystem::remove_all(root);
    if (!bothVehiclesStartedTheirOwnLoops || !triggerStopIsScopedToFirstVehicle ||
        !ambientStopIsScopedToFirstVehicle || !stopAllLeavesSecondVehicleLoopsAlone ||
        !triggerLookupIsLocal || !oneShotPlaybackWorks || !repeatedOneShotIsCoalesced ||
        !oneShotStateIsVehicleLocal || !extendedChecksPassed) {
        std::cerr << "vehicle sound source ownership, cancellation, or retrigger policy failed\n";
        return 1;
    }
    return 0;
}
