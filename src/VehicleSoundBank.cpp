#include "VehicleSoundBank.h"

#include "ConfigurationParser.h"
#include "Logger.h"
#include "PerfTrace.h"
#include "Variables.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <string_view>
#include <utility>

namespace {

Logger soundLog = Logger("Sound");

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

std::filesystem::path resolve(const std::filesystem::path& base, std::string value) {
    std::replace(value.begin(), value.end(), '\\', '/');
    return base / std::filesystem::path(value);
}

std::string pathKey(const std::filesystem::path& path) {
    return path.lexically_normal().generic_string();
}

bool endsWith(const std::string& value, std::string_view suffix) {
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool isLoopSoundFile(const std::filesystem::path& path) {
    const std::string stem = lower(path.stem().string());
    return stem == "loop" || endsWith(stem, "_loop") || endsWith(stem, "-loop");
}

bool isEndSoundFile(const std::filesystem::path& path) {
    return endsWith(lower(path.stem().string()), "_end");
}

bool belongsToSoundFamily(const std::filesystem::path& loopPath,
                          const std::filesystem::path& endPath) {
    const std::string loopStem = lower(loopPath.stem().string());
    const std::string endStem = lower(endPath.stem().string());
    const std::string family = endStem.substr(0, endStem.size() - 4);
    return loopPath.parent_path() == endPath.parent_path() &&
           (loopStem == family + "_loop" || loopStem == family + "-loop");
}

bool belongsToStartFamily(const std::filesystem::path& loopPath,
                          const std::filesystem::path& startPath) {
    const std::string loopStem = lower(loopPath.stem().string());
    const std::string startStem = lower(startPath.stem().string());
    constexpr std::string_view suffix = "_start";
    if (startStem.size() <= suffix.size() ||
        startStem.compare(startStem.size() - suffix.size(), suffix.size(), suffix) != 0) {
        return false;
    }
    const std::string family = startStem.substr(0, startStem.size() - suffix.size());
    return loopPath.parent_path() == startPath.parent_path() &&
           (loopStem == family + "_loop" || loopStem == family + "-loop");
}

float evaluateCurve(const std::vector<SoundCurvePoint>& points, float value) {
    if (points.empty()) {
        return 1.0F;
    }
    if (value <= points.front().x) {
        return points.front().y;
    }
    for (std::size_t index = 1; index < points.size(); ++index) {
        if (value <= points[index].x) {
            const SoundCurvePoint& left = points[index - 1];
            const SoundCurvePoint& right = points[index];
            const float range = right.x - left.x;
            if (range == 0.0F) {
                return right.y;
            }
            const float fraction = (value - left.x) / range;
            return left.y + fraction * (right.y - left.y);
        }
    }
    return points.back().y;
}

} // namespace

void VehicleSoundBank::load(const std::filesystem::path& configPath) {
    if (configPath.empty()) {
        return;
    }
    basePath_ = configPath.parent_path();
    triggers_.clear();
    untriggeredLoopSounds_.clear();
    activeTriggeredLoops_.clear();
    activeAmbientLoops_.clear();
    activeOneShots_.clear();

    openbus::config::Reader reader(configPath);
    if (!reader.isOpen()) {
        soundLog.Log("Unable to open sound configuration: " + configPath.string());
        return;
    }

    SoundDefinitionId nextDefinitionId = 1;
    SoundTriggerDefinition current;
    bool hasSound = false;
    std::vector<std::string> currentTriggerNames;
    const auto updateCurrentTriggers = [&]() {
        for (const std::string& name : currentTriggerNames) {
            std::vector<SoundTriggerDefinition>& definitions = triggers_[lower(name)];
            const auto existing = std::find_if(
                definitions.begin(), definitions.end(),
                [&](const SoundTriggerDefinition& value) { return value.id == current.id; });
            if (existing == definitions.end()) {
                definitions.push_back(current);
            } else {
                *existing = current;
            }
        }
    };
    const auto retainUntriggeredLoop = [&]() {
        if (!hasSound || !currentTriggerNames.empty()) {
            return;
        }
        const bool curveDrivenAmbient =
            !current.loopDisabled && (!current.volumeCurves.empty() || !current.conditions.empty());
        if (current.loop || curveDrivenAmbient) {
            current.loop = true;
            untriggeredLoopSounds_.push_back(current);
        }
    };
    openbus::config::Line line;
    while (reader.next(line)) {
        if (!line.isKeyword()) {
            continue;
        }
        const std::string keyword = line.keyword();
        if (keyword == "sound" || keyword == "loopsound") {
            retainUntriggeredLoop();
            openbus::config::Line value;
            ConfigurationDiagnostics diagnostics;
            if (!reader.readPayload(value, diagnostics, keyword)) {
                continue;
            }
            current = {};
            current.id = nextDefinitionId++;
            current.file = resolve(configPath.parent_path(), openbus::config::trim(value.text));
            current.loop = keyword == "loopsound" || isLoopSoundFile(current.file);
            if (keyword == "sound") {
                openbus::config::Line gainLine;
                if (reader.next(gainLine)) {
                    if (gainLine.isKeyword()) {
                        reader.pushBack(std::move(gainLine));
                    } else {
                        double baseGain = 1.0;
                        if (openbus::config::parseDouble(gainLine.text, baseGain)) {
                            current.baseGain = (std::max)(0.0, baseGain);
                        }
                    }
                }
            }
            if (keyword == "loopsound") {
                std::vector<std::string> loopValues;
                openbus::config::Line loopValue;
                while (reader.next(loopValue)) {
                    if (loopValue.isKeyword()) {
                        reader.pushBack(std::move(loopValue));
                        break;
                    }
                    loopValues.push_back(loopValue.text);
                    if (loopValues.size() == 4) {
                        break;
                    }
                }
                if (loopValues.size() >= 3) {
                    current.controlVariable = lower(openbus::config::trim(loopValues[1]));
                    openbus::config::parseFloat(loopValues[2], current.controlCenter);
                }
            }
            currentTriggerNames.clear();
            hasSound = true;
            continue;
        }
        if (keyword == "trigger") {
            openbus::config::Line value;
            ConfigurationDiagnostics diagnostics;
            if (hasSound && reader.readPayload(value, diagnostics, keyword)) {
                currentTriggerNames.push_back(openbus::config::trim(value.text));
                updateCurrentTriggers();
            }
            continue;
        }
        if (keyword == "noloop") {
            if (hasSound) {
                current.loop = false;
                current.loopDisabled = true;
                updateCurrentTriggers();
            }
            continue;
        }
        if (keyword == "loop" || keyword == "looped") {
            if (hasSound) {
                current.loop = true;
                current.loopDisabled = false;
                updateCurrentTriggers();
            }
            continue;
        }
        if (keyword == "viewpoint") {
            openbus::config::Line value;
            ConfigurationDiagnostics diagnostics;
            int viewpoint = 0;
            if (hasSound && reader.readPayload(value, diagnostics, keyword) &&
                openbus::config::parseInt(value.text, viewpoint)) {
                current.viewpoint = viewpoint;
                updateCurrentTriggers();
            }
            continue;
        }
        if (keyword == "3d") {
            std::vector<std::string> values;
            ConfigurationDiagnostics diagnostics;
            double maxDistance = 0.0;
            if (hasSound && reader.readPayloads(4, values, diagnostics, keyword) &&
                openbus::config::parseDouble(values[3], maxDistance)) {
                double sourceX = 0.0;
                double sourceY = 0.0;
                openbus::config::parseDouble(values[0], sourceX);
                openbus::config::parseDouble(values[1], sourceY);
                current.position = {sourceY, -sourceX, 0.0};
                openbus::config::parseDouble(values[2], current.position[2]);
                current.maxDistance = (std::max)(0.0, maxDistance);
                updateCurrentTriggers();
            }
            continue;
        }
        if (keyword == "volcurve") {
            SoundVolumeCurve curve;
            openbus::config::Line curveVariable;
            if (reader.next(curveVariable) && !curveVariable.isKeyword()) {
                curve.variable = lower(openbus::config::trim(curveVariable.text));
            } else if (curveVariable.isKeyword()) {
                reader.pushBack(std::move(curveVariable));
            }
            while (reader.next(line)) {
                if (line.isKeyword() && line.keyword() != "pnt") {
                    reader.pushBack(std::move(line));
                    break;
                }
                if (line.isKeyword()) {
                    std::vector<std::string> values;
                    ConfigurationDiagnostics diagnostics;
                    if (!reader.readPayloads(2, values, diagnostics, "pnt")) {
                        break;
                    }
                    SoundCurvePoint point;
                    if (openbus::config::parseFloat(values[0], point.x) &&
                        openbus::config::parseFloat(values[1], point.y)) {
                        curve.points.push_back(point);
                    }
                    continue;
                }
                std::istringstream values(line.text);
                SoundCurvePoint point;
                if (values >> point.x >> point.y) {
                    curve.points.push_back(point);
                }
            }
            if (!curve.points.empty()) {
                if (current.volumeCurve.empty()) {
                    current.volumeCurve = curve.points;
                }
                current.volumeCurves.push_back(std::move(curve));
            }
            updateCurrentTriggers();
            continue;
        }
        if (keyword == "conditionSingle") {
            if (!hasSound) {
                continue;
            }
            std::vector<std::string> values;
            ConfigurationDiagnostics diagnostics;
            SoundCondition condition;
            if (reader.readPayloads(3, values, diagnostics, keyword) &&
                openbus::config::parseFloat(values[1], condition.referenceValue) &&
                openbus::config::parseInt(values[2], condition.comparison) &&
                condition.comparison >= 0 && condition.comparison <= 5) {
                condition.variable = lower(openbus::config::trim(values[0]));
                if (!condition.variable.empty()) {
                    current.conditions.push_back(std::move(condition));
                    updateCurrentTriggers();
                    continue;
                }
            }
            current.conditions.push_back({{}, 0.0F, -1});
            soundLog.Log("Invalid [conditionSingle] in " + configPath.string());
            updateCurrentTriggers();
        }
    }
    retainUntriggeredLoop();
}

bool VehicleSoundBank::conditionsAllow(const SoundTriggerDefinition& definition,
                                       const Variables& variables) {
    for (const SoundCondition& condition : definition.conditions) {
        if (condition.variable.empty() || condition.comparison < 0 || condition.comparison > 5) {
            return false;
        }
        const float value = variables.get(condition.variable);
        bool matches = false;
        switch (condition.comparison) {
        case 0:
            matches = value != condition.referenceValue;
            break;
        case 1:
            matches = value == condition.referenceValue;
            break;
        case 2:
            matches = value < condition.referenceValue;
            break;
        case 3:
            matches = value > condition.referenceValue;
            break;
        case 4:
            matches = value <= condition.referenceValue;
            break;
        case 5:
            matches = value >= condition.referenceValue;
            break;
        default:
            return false;
        }
        if (!matches) {
            return false;
        }
    }
    return true;
}

std::filesystem::path
VehicleSoundBank::resolvedFile(const SoundTriggerDefinition& definition) const {
    return definition.file.is_absolute() ? definition.file : basePath_ / definition.file;
}

bool VehicleSoundBank::hasTrigger(const std::string& name) const {
    return triggers_.find(lower(name)) != triggers_.end();
}

void VehicleSoundBank::stopTriggeredLoop(SoundPlayback& playback, SoundDefinitionId id) {
    const auto loop = activeTriggeredLoops_.find(id);
    if (loop == activeTriggeredLoops_.end()) {
        return;
    }
    playback.stopLoop(loop->second.handle);
    activeTriggeredLoops_.erase(loop);
}

void VehicleSoundBank::stopOneShot(SoundPlayback& playback, SoundDefinitionId id) {
    const auto active = activeOneShots_.find(id);
    if (active == activeOneShots_.end()) {
        return;
    }
    playback.stop(active->second.handle);
    activeOneShots_.erase(active);
}

void VehicleSoundBank::stopAmbientLoopFile(SoundPlayback& playback,
                                           const std::filesystem::path& path) {
    const std::string key = pathKey(path);
    const auto loop = activeAmbientLoops_.find(key);
    if (loop == activeAmbientLoops_.end()) {
        return;
    }
    playback.stopLoop(loop->second);
    activeAmbientLoops_.erase(loop);
}

void VehicleSoundBank::trigger(SoundPlayback& playback, const std::string& name,
                               const std::filesystem::path& overrideFile, float controlValue,
                               const Variables& variables,
                               openbus::rendering::ViewpointContext viewpoint,
                               SoundRetriggerPolicy retriggerPolicy) {
    openbus::rendering::TraceScope trace("sound", "VehicleSoundBank::trigger");
    const auto found = triggers_.find(lower(name));
    if (found == triggers_.end()) {
        soundLog.Log("Skipping unknown sound trigger: " + name);
        return;
    }
    std::vector<SoundTriggerDefinition> definitions = found->second;
    if (overrideFile.empty()) {
        // Retain the existing filename-family heuristic, not verified native OMSI behavior.
        std::vector<SoundTriggerDefinition> inferredLoops;
        for (const SoundTriggerDefinition& definition : definitions) {
            if (definition.loop) {
                continue;
            }
            for (const SoundTriggerDefinition& loop : untriggeredLoopSounds_) {
                if (belongsToStartFamily(loop.file, definition.file) &&
                    std::none_of(definitions.begin(), definitions.end(),
                                 [&loop](const SoundTriggerDefinition& existing) {
                                     return existing.id == loop.id;
                                 }) &&
                    std::none_of(inferredLoops.begin(), inferredLoops.end(),
                                 [&loop](const SoundTriggerDefinition& existing) {
                                     return existing.id == loop.id;
                                 })) {
                    inferredLoops.push_back(loop);
                }
            }
        }
        definitions.insert(definitions.end(), inferredLoops.begin(), inferredLoops.end());
    }
    if (!overrideFile.empty()) {
        // T.F changes the clip, not the configured channel, metadata or loop mode.
        for (SoundTriggerDefinition& definition : definitions) {
            definition.file = overrideFile;
        }
    }
    definitions.erase(std::remove_if(definitions.begin(), definitions.end(),
                                     [viewpoint, &variables](const auto& definition) {
                                         return !openbus::rendering::viewpointMatches(
                                                    definition.viewpoint, viewpoint) ||
                                                (!definition.conditions.empty() &&
                                                 !conditionsAllow(definition, variables));
                                     }),
                      definitions.end());
    if (definitions.empty()) {
        return;
    }
    if (overrideFile.empty()) {
        for (const SoundTriggerDefinition& definition : definitions) {
            if (!isEndSoundFile(definition.file)) {
                continue;
            }
            for (const auto& entry : triggers_) {
                for (const SoundTriggerDefinition& candidate : entry.second) {
                    if (candidate.loop && belongsToSoundFamily(candidate.file, definition.file)) {
                        stopTriggeredLoop(playback, candidate.id);
                    }
                }
            }
            for (const SoundTriggerDefinition& candidate : untriggeredLoopSounds_) {
                if (belongsToSoundFamily(candidate.file, definition.file)) {
                    stopTriggeredLoop(playback, candidate.id);
                    stopAmbientLoopFile(playback, resolvedFile(candidate));
                }
            }
        }
    }
    for (const SoundTriggerDefinition& definition : definitions) {
        const std::filesystem::path file = resolvedFile(definition);
        double gain = definition.baseGain;
        if (!definition.volumeCurve.empty()) {
            gain *= evaluateCurve(definition.volumeCurve, controlValue);
        }
        gain = std::clamp(gain, 0.0, 1.0);
        // Keep the existing silent-loop behavior. A live KeepPlaying one-shot must
        // still receive parameter changes, including a gain that has reached zero.
        if (gain <= 0.0 && definition.loop) {
            soundLog.Log("Skipping silent sound: trigger=" + name +
                         " gain=" + std::to_string(gain));
            continue;
        }
        const SoundPlaybackParameters parameters{static_cast<float>(gain), 1.0F,
                                                 definition.position, definition.maxDistance};
        auto& ownedSources = definition.loop ? activeTriggeredLoops_ : activeOneShots_;
        const auto active = ownedSources.find(definition.id);
        if (active != ownedSources.end()) {
            if (!definition.loop && !playback.isPlaying(active->second.handle)) {
                ownedSources.erase(active);
            } else if (pathKey(active->second.file) == pathKey(file) &&
                       (definition.loop || retriggerPolicy == SoundRetriggerPolicy::KeepPlaying)) {
                // OpenBus KeepPlaying policy: update a live voice instead of restarting it.
                // Exact native OMSI same-file retrigger behavior is still unverified.
                if (definition.loop) {
                    playback.updateLoop(active->second.handle, parameters);
                } else {
                    playback.update(active->second.handle, parameters);
                }
                continue;
            } else {
                if (definition.loop) {
                    playback.stopLoop(active->second.handle);
                } else {
                    playback.stop(active->second.handle);
                }
                ownedSources.erase(active);
            }
        }
        if (gain <= 0.0) {
            soundLog.Log("Skipping silent sound: trigger=" + name +
                         " gain=" + std::to_string(gain));
            continue;
        }
        soundLog.Log("Sound playback requested: trigger=" + name + " file=" + file.string() +
                     " gain=" + std::to_string(gain) +
                     " loop=" + (definition.loop ? "true" : "false"));
        const SoundPlaybackHandle handle = playback.play(file, definition.loop, parameters);
        if (handle != 0) {
            ownedSources.emplace(definition.id, ActiveSound{file, handle});
        }
    }
}

void VehicleSoundBank::stop(SoundPlayback& playback, const std::string& name) {
    const auto found = triggers_.find(lower(name));
    if (found == triggers_.end()) {
        return;
    }
    for (const SoundTriggerDefinition& definition : found->second) {
        stopOneShot(playback, definition.id);
        stopTriggeredLoop(playback, definition.id);
    }
    for (const SoundTriggerDefinition& definition : untriggeredLoopSounds_) {
        for (const SoundTriggerDefinition& trigger : found->second) {
            if (belongsToStartFamily(definition.file, trigger.file)) {
                stopTriggeredLoop(playback, definition.id);
                stopAmbientLoopFile(playback, resolvedFile(definition));
            }
        }
    }
}

void VehicleSoundBank::updateAmbient(SoundPlayback& playback, const Variables& variables,
                                     openbus::rendering::ViewpointContext viewpoint) {
    openbus::rendering::TraceScope trace("sound", "VehicleSoundBank::updateAmbient");
    for (auto active = activeOneShots_.begin(); active != activeOneShots_.end();) {
        if (!playback.isPlaying(active->second.handle)) {
            active = activeOneShots_.erase(active);
        } else {
            ++active;
        }
    }
    struct DesiredLoop {
        std::filesystem::path path;
        SoundPlaybackParameters parameters;
    };
    std::unordered_map<std::string, DesiredLoop> desired;
    desired.reserve(untriggeredLoopSounds_.size());
    for (const SoundTriggerDefinition& definition : untriggeredLoopSounds_) {
        const std::filesystem::path file = resolvedFile(definition);
        if (!openbus::rendering::viewpointMatches(definition.viewpoint, viewpoint) ||
            !conditionsAllow(definition, variables)) {
            continue;
        }
        double gain = definition.baseGain;
        if (!definition.volumeCurves.empty()) {
            for (const SoundVolumeCurve& curve : definition.volumeCurves) {
                gain *= evaluateCurve(curve.points, variables.get(curve.variable));
            }
        } else if (!definition.controlVariable.empty() && definition.controlCenter > 0.0) {
            gain *= std::clamp(variables.get(definition.controlVariable) / definition.controlCenter,
                               0.0F, 1.0F);
        }
        gain = std::clamp(gain, 0.0, 1.0);
        if (gain <= 0.0) {
            continue;
        }
        double pitch = 1.0;
        if (!definition.controlVariable.empty() && definition.controlCenter > 0.0) {
            pitch = std::clamp(variables.get(definition.controlVariable) / definition.controlCenter,
                               0.5F, 2.0F);
        }
        desired.emplace(pathKey(file),
                        DesiredLoop{file,
                                    {static_cast<float>(gain), static_cast<float>(pitch),
                                     definition.position, definition.maxDistance}});
    }

    for (auto active = activeAmbientLoops_.begin(); active != activeAmbientLoops_.end();) {
        if (desired.find(active->first) != desired.end()) {
            ++active;
            continue;
        }
        playback.stopLoop(active->second);
        active = activeAmbientLoops_.erase(active);
    }
    for (const auto& [key, loop] : desired) {
        const auto active = activeAmbientLoops_.find(key);
        if (active != activeAmbientLoops_.end()) {
            playback.updateLoop(active->second, loop.parameters);
            continue;
        }
        const SoundPlaybackHandle handle = playback.play(loop.path, true, loop.parameters);
        if (handle != 0) {
            activeAmbientLoops_.emplace(key, handle);
        }
    }
}

void VehicleSoundBank::stopAllLoops(SoundPlayback& playback) {
    for (const auto& [id, source] : activeTriggeredLoops_) {
        (void)id;
        playback.stopLoop(source.handle);
    }
    for (const auto& [key, handle] : activeAmbientLoops_) {
        (void)key;
        playback.stopLoop(handle);
    }
    activeTriggeredLoops_.clear();
    activeAmbientLoops_.clear();
}

void VehicleSoundBank::stopAll(SoundPlayback& playback) {
    stopAllLoops(playback);
    for (const auto& [id, source] : activeOneShots_) {
        (void)id;
        playback.stop(source.handle);
    }
    activeOneShots_.clear();
}
