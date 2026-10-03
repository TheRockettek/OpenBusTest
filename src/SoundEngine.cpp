#include "SoundEngine.h"

#include "ConfigurationParser.h"
#include "Logger.h"
#include "PerfTrace.h"
#include "Variables.h"

#include <AL/al.h>
#include <AL/alc.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

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

double evaluateCurve(const std::vector<SoundCurvePoint>& points, double value) {
    if (points.empty()) {
        return 1.0;
    }
    if (value <= points.front().x) {
        return points.front().y;
    }
    for (std::size_t index = 1; index < points.size(); ++index) {
        if (value <= points[index].x) {
            const SoundCurvePoint& left = points[index - 1];
            const SoundCurvePoint& right = points[index];
            const double range = right.x - left.x;
            if (range == 0.0) {
                return right.y;
            }
            const double fraction = (value - left.x) / range;
            return left.y + fraction * (right.y - left.y);
        }
    }
    return points.back().y;
}

} // namespace

struct SoundEngine::Backend {
    struct Clip {
        std::vector<std::int16_t> samples;
        ALenum format = AL_FORMAT_MONO16;
        ALsizei rate = 48000;
        ALuint buffer = 0;
    };

    struct ActiveSource {
        std::filesystem::path path;
        ALuint source = 0;
        bool loop = false;
    };

    struct LoopUpdate {
        std::filesystem::path path;
        float gain = 0.0f;
        float pitch = 1.0f;
        std::array<double, 3> position = {};
        double maxDistance = 0.0;
    };

    ALCdevice* device = nullptr;
    ALCcontext* context = nullptr;
    std::mutex mutex;
    std::unordered_map<std::string, std::shared_ptr<Clip>> clips;
    std::vector<ActiveSource> active;
    std::unordered_map<std::string, ALuint> activeLoops;

    static std::string sourceKey(const std::filesystem::path& path) {
        return path.lexically_normal().string();
    }

    static std::uint32_t read32(const std::vector<std::uint8_t>& data, std::size_t offset) {
        return static_cast<std::uint32_t>(data[offset]) |
               (static_cast<std::uint32_t>(data[offset + 1]) << 8) |
               (static_cast<std::uint32_t>(data[offset + 2]) << 16) |
               (static_cast<std::uint32_t>(data[offset + 3]) << 24);
    }

    static std::uint16_t read16(const std::vector<std::uint8_t>& data, std::size_t offset) {
        return static_cast<std::uint16_t>(data[offset]) |
               (static_cast<std::uint16_t>(data[offset + 1]) << 8);
    }

    static std::shared_ptr<Clip> decodeWav(const std::filesystem::path& path) {
        std::ifstream input(path, std::ios::binary);
        if (!input) {
            return {};
        }
        std::vector<std::uint8_t> data((std::istreambuf_iterator<char>(input)), {});
        if (data.size() < 12 || std::memcmp(data.data(), "RIFF", 4) != 0 ||
            std::memcmp(data.data() + 8, "WAVE", 4) != 0) {
            return {};
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
                return {};
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
                return {};
            }
            offset = payload + paddedSize;
        }

        if ((format != 1 && format != 3) || (channels != 1 && channels != 2) || rate == 0 ||
            (format == 1 && bits != 8 && bits != 16 && bits != 24 && bits != 32) ||
            (format == 3 && bits != 32) || dataOffset == 0) {
            return {};
        }
        const std::size_t bytesPerSample = bits / 8;
        const std::size_t frameBytes = bytesPerSample * channels;
        if (frameBytes == 0 || dataSize < frameBytes) {
            return {};
        }
        const std::size_t frameCount = dataSize / frameBytes;
        if (frameCount > static_cast<std::size_t>(std::numeric_limits<ALsizei>::max())) {
            return {};
        }

        auto clip = std::make_shared<Clip>();
        clip->format = channels == 1 ? AL_FORMAT_MONO16 : AL_FORMAT_STEREO16;
        clip->rate = static_cast<ALsizei>(rate);
        clip->samples.resize(frameCount * channels);
        for (std::size_t sample = 0; sample < clip->samples.size(); ++sample) {
            const std::size_t sampleOffset = dataOffset + sample * bytesPerSample;
            std::int32_t integer = 0;
            if (format == 3) {
                float value = 0.0f;
                std::memcpy(&value, data.data() + sampleOffset, sizeof(value));
                clip->samples[sample] = static_cast<std::int16_t>(
                    std::lround(std::clamp(value, -1.0f, 1.0f) * 32767.0f));
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
                integer = static_cast<std::int32_t>(read32(data, sampleOffset) >> 16);
            }
            clip->samples[sample] = static_cast<std::int16_t>(std::clamp(integer, -32768, 32767));
        }
        return clip;
    }

    void cleanupStoppedSourcesLocked() {
        for (auto source = active.begin(); source != active.end();) {
            if (source->loop) {
                ++source;
                continue;
            }
            ALint state = AL_STOPPED;
            alGetSourcei(source->source, AL_SOURCE_STATE, &state);
            if (state == AL_STOPPED) {
                alDeleteSources(1, &source->source);
                source = active.erase(source);
            } else {
                ++source;
            }
        }
    }

    std::shared_ptr<Clip> loadClipLocked(const std::filesystem::path& path) {
        openbus::rendering::TraceScope trace("sound", "Backend::loadClipLocked");
        const std::string key = path.lexically_normal().string();
        const auto found = clips.find(key);
        if (found != clips.end()) {
            return found->second;
        }
        auto clip = decodeWav(path);
        if (!clip) {
            return {};
        }
        alGenBuffers(1, &clip->buffer);
        alBufferData(clip->buffer, clip->format, clip->samples.data(),
                     static_cast<ALsizei>(clip->samples.size() * sizeof(std::int16_t)), clip->rate);
        if (alGetError() != AL_NO_ERROR) {
            alDeleteBuffers(1, &clip->buffer);
            return {};
        }
        clips.emplace(key, clip);
        return clip;
    }

    Backend() {
        device = alcOpenDevice(nullptr);
        if (device == nullptr) {
            soundLog.Log("Unable to open an OpenAL audio device");
            return;
        }
        context = alcCreateContext(device, nullptr);
        if (context == nullptr || alcMakeContextCurrent(context) == ALC_FALSE) {
            soundLog.Log("Unable to create an OpenAL audio context");
            if (context != nullptr) {
                alcDestroyContext(context);
                context = nullptr;
            }
            alcCloseDevice(device);
            device = nullptr;
            return;
        }
        // Do not clamp at AL_MAX_DISTANCE: OMSI's 3D field is not an audio cutoff,
        // and clamping here makes distant sources retain a constant audible level.
        alDistanceModel(AL_INVERSE_DISTANCE);
        alDopplerFactor(0.0f);
        alListenerf(AL_GAIN, 1.0f);
    }

    ~Backend() {
        std::lock_guard<std::mutex> lock(mutex);
        for (const ActiveSource& source : active) {
            alSourceStop(source.source);
            alDeleteSources(1, &source.source);
        }
        active.clear();
        for (const auto& entry : clips) {
            alDeleteBuffers(1, &entry.second->buffer);
        }
        clips.clear();
        if (context != nullptr) {
            alcMakeContextCurrent(nullptr);
            alcDestroyContext(context);
        }
        if (device != nullptr) {
            alcCloseDevice(device);
        }
    }

    void setListenerPose(const std::array<double, 3>& position,
                         const std::array<double, 3>& forward, const std::array<double, 3>& up) {
        openbus::rendering::TraceScope trace("sound", "Backend::setListenerPose");
        if (context == nullptr) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex);
        const std::array<float, 6> orientation = {
            static_cast<float>(forward[0]), static_cast<float>(forward[1]),
            static_cast<float>(forward[2]), static_cast<float>(up[0]),
            static_cast<float>(up[1]),      static_cast<float>(up[2])};
        alListener3f(AL_POSITION, static_cast<float>(position[0]), static_cast<float>(position[1]),
                     static_cast<float>(position[2]));
        alListenerfv(AL_ORIENTATION, orientation.data());
    }

    void createSourceLocked(const std::filesystem::path& path, bool looped, float gain, float pitch,
                            const std::array<double, 3>& position, double maxDistance) {
        const std::shared_ptr<Clip> clip = loadClipLocked(path);
        if (!clip) {
            soundLog.Log("Unable to decode sound: " + path.string());
            return;
        }
        const std::string key = sourceKey(path);
        if (looped && activeLoops.contains(key)) {
            return;
        }
        ALuint source = 0;
        alGenSources(1, &source);
        if (source == 0) {
            soundLog.Log("Unable to create OpenAL sound source");
            return;
        }
        alSourcei(source, AL_BUFFER, static_cast<ALint>(clip->buffer));
        alSourcei(source, AL_LOOPING, looped ? AL_TRUE : AL_FALSE);
        alSourcef(source, AL_GAIN, gain);
        alSourcef(source, AL_PITCH, pitch);
        alSourcei(source, AL_SOURCE_RELATIVE, AL_FALSE);
        alSource3f(source, AL_POSITION, static_cast<float>(position[0]),
                   static_cast<float>(position[1]), static_cast<float>(position[2]));
        alSourcef(source, AL_REFERENCE_DISTANCE, 1.0f);
        alSourcef(source, AL_ROLLOFF_FACTOR, 1.0f);
        if (maxDistance > 0.0) {
            alSourcef(source, AL_MAX_DISTANCE, static_cast<float>(maxDistance));
        }
        alSourcePlay(source);
        if (alGetError() != AL_NO_ERROR) {
            alDeleteSources(1, &source);
            soundLog.Log("Unable to start OpenAL sound source: " + path.string());
            return;
        }
        active.push_back({path, source, looped});
        if (looped) {
            activeLoops.emplace(key, source);
        }
    }

    void play(const std::filesystem::path& path, bool looped, float gain, float pitch,
              const std::array<double, 3>& position, double maxDistance) {
        openbus::rendering::TraceScope trace("sound", "Backend::play");
        if (context == nullptr) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex);
        cleanupStoppedSourcesLocked();
        createSourceLocked(path, looped, gain, pitch, position, maxDistance);
    }

    void stop(const std::filesystem::path& path) {
        if (context == nullptr) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex);
        const auto loop = activeLoops.find(sourceKey(path));
        if (loop == activeLoops.end()) {
            return;
        }
        const ALuint sourceId = loop->second;
        alSourceStop(sourceId);
        alDeleteSources(1, &sourceId);
        activeLoops.erase(loop);
        active.erase(std::remove_if(active.begin(), active.end(), [sourceId](const ActiveSource& s) {
                         return s.source == sourceId;
                     }),
                     active.end());
    }

    void updateLoops(const std::vector<LoopUpdate>& updates) {
        openbus::rendering::TraceScope trace("sound", "Backend::updateLoops");
        if (context == nullptr) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex);
        cleanupStoppedSourcesLocked();
        std::unordered_map<std::string, const LoopUpdate*> desired;
        desired.reserve(updates.size());
        for (const LoopUpdate& update : updates) {
            desired.emplace(sourceKey(update.path), &update);
        }

        for (auto source = active.begin(); source != active.end();) {
            if (!source->loop) {
                ++source;
                continue;
            }
            const std::string key = sourceKey(source->path);
            const auto target = desired.find(key);
            if (target == desired.end()) {
                alSourceStop(source->source);
                alDeleteSources(1, &source->source);
                activeLoops.erase(key);
                source = active.erase(source);
                continue;
            }
            const LoopUpdate& update = *target->second;
            alSourcef(source->source, AL_GAIN, update.gain);
            alSourcef(source->source, AL_PITCH, update.pitch);
            alSource3f(source->source, AL_POSITION, static_cast<float>(update.position[0]),
                       static_cast<float>(update.position[1]),
                       static_cast<float>(update.position[2]));
            desired.erase(target);
            ++source;
        }
        for (const auto& [key, update] : desired) {
            (void)key;
            createSourceLocked(update->path, true, update->gain, update->pitch,
                               update->position, update->maxDistance);
        }
    }
};

SoundEngine::SoundEngine() : backend_(std::make_unique<Backend>()) {}

SoundEngine::~SoundEngine() = default;

void SoundEngine::load(const std::filesystem::path& configPath) {
    if (configPath.empty()) {
        return;
    }
    basePath_ = configPath.parent_path();
    triggers_.clear();
    untriggeredLoopSounds_.clear();

    openbus::config::Reader reader(configPath);
    if (!reader.isOpen()) {
        soundLog.Log("Unable to open sound configuration: " + configPath.string());
        return;
    }

    SoundTriggerDefinition current;
    bool hasSound = false;
    std::vector<std::string> currentTriggerNames;
    const auto updateCurrentTriggers = [&]() {
        for (const std::string& name : currentTriggerNames) {
            std::vector<SoundTriggerDefinition>& definitions = triggers_[lower(name)];
            const auto existing = std::find_if(
                definitions.begin(), definitions.end(),
                [&](const SoundTriggerDefinition& value) { return value.file == current.file; });
            if (existing == definitions.end()) {
                definitions.push_back(current);
            } else {
                *existing = current;
            }
        }
    };
    openbus::config::Line line;
    while (reader.next(line)) {
        if (!line.isKeyword()) {
            continue;
        }
        const std::string keyword = line.keyword();
        if (keyword == "sound" || keyword == "loopsound") {
            if (hasSound && current.loop && currentTriggerNames.empty()) {
                untriggeredLoopSounds_.push_back(current);
            }
            openbus::config::Line value;
            ConfigurationDiagnostics diagnostics;
            if (!reader.readPayload(value, diagnostics, keyword)) {
                continue;
            }
            current = {};
            current.file = resolve(configPath.parent_path(), openbus::config::trim(value.text));
            current.loop = keyword == "loopsound" || isLoopSoundFile(current.file);
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
                    openbus::config::parseDouble(loopValues[2], current.controlCenter);
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
                updateCurrentTriggers();
            }
            continue;
        }
        if (keyword == "loop" || keyword == "looped") {
            if (hasSound) {
                current.loop = true;
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
                    if (openbus::config::parseDouble(values[0], point.x) &&
                        openbus::config::parseDouble(values[1], point.y)) {
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
        }
    }
    if (hasSound && current.loop && currentTriggerNames.empty()) {
        untriggeredLoopSounds_.push_back(current);
    }
}

void SoundEngine::setListenerPose(const std::array<double, 3>& position,
                                  const std::array<double, 3>& forward,
                                  const std::array<double, 3>& up) {
    backend_->setListenerPose(position, forward, up);
}

void SoundEngine::updateLoops(const Variables& variables, int viewpoint) {
    openbus::rendering::TraceScope trace("sound", "SoundEngine::updateLoops");
    std::vector<Backend::LoopUpdate> updates;
    updates.reserve(untriggeredLoopSounds_.size());
    for (const SoundTriggerDefinition& definition : untriggeredLoopSounds_) {
        const std::filesystem::path file =
            definition.file.is_absolute() ? definition.file : basePath_ / definition.file;
        if (definition.viewpoint != 0 && definition.viewpoint != viewpoint) {
            continue;
        }

        double gain = 1.0;
        if (!definition.volumeCurves.empty()) {
            for (const SoundVolumeCurve& curve : definition.volumeCurves) {
                gain *= evaluateCurve(curve.points, variables.get(curve.variable));
            }
        } else if (!definition.controlVariable.empty() && definition.controlCenter > 0.0) {
            gain = std::clamp(variables.get(definition.controlVariable) /
                                  definition.controlCenter,
                              0.0, 1.0);
        }
        gain = std::clamp(gain, 0.0, 1.0);
        if (gain <= 0.0) {
            continue;
        }
        double pitch = 1.0;
        if (!definition.controlVariable.empty() && definition.controlCenter > 0.0) {
            pitch = std::clamp(variables.get(definition.controlVariable) /
                                   definition.controlCenter,
                               0.5, 2.0);
        }
        updates.push_back({file, static_cast<float>(gain), static_cast<float>(pitch),
                           definition.position, definition.maxDistance});
    }
    backend_->updateLoops(updates);
}

void SoundEngine::trigger(const std::string& name, const std::filesystem::path& overrideFile,
                          double controlValue) {
    openbus::rendering::TraceScope trace("sound", "SoundEngine::trigger");
    const auto found = triggers_.find(lower(name));
    if (found == triggers_.end() && overrideFile.empty()) {
        soundLog.Log("Skipping unknown sound trigger: " + name);
        return;
    }
    std::vector<SoundTriggerDefinition> definitions;
    if (found != triggers_.end()) {
        definitions = found->second;
    }
    if (overrideFile.empty()) {
        std::vector<SoundTriggerDefinition> inferredLoops;
        for (const SoundTriggerDefinition& definition : definitions) {
            if (definition.loop) {
                continue;
            }
            for (const SoundTriggerDefinition& loop : untriggeredLoopSounds_) {
                if (belongsToStartFamily(loop.file, definition.file) &&
                    std::none_of(definitions.begin(), definitions.end(),
                                 [&loop](const SoundTriggerDefinition& existing) {
                                     return existing.file == loop.file;
                                 }) &&
                    std::none_of(inferredLoops.begin(), inferredLoops.end(),
                                 [&loop](const SoundTriggerDefinition& existing) {
                                     return existing.file == loop.file;
                                 })) {
                    inferredLoops.push_back(loop);
                }
            }
        }
        definitions.insert(definitions.end(), inferredLoops.begin(), inferredLoops.end());
    }
    if (definitions.empty() && overrideFile.empty()) {
        soundLog.Log("Skipping unknown sound trigger: " + name);
        return;
    }
    if (!overrideFile.empty()) {
        definitions = {SoundTriggerDefinition{overrideFile, false, 0, 0.0, {}, {}, 0.0, {}, {}}};
    }
    if (overrideFile.empty()) {
        for (const SoundTriggerDefinition& definition : definitions) {
            if (!isEndSoundFile(definition.file)) {
                continue;
            }
            for (const auto& entry : triggers_) {
                for (const SoundTriggerDefinition& candidate : entry.second) {
                    if (candidate.loop && belongsToSoundFamily(candidate.file, definition.file)) {
                        backend_->stop(candidate.file);
                    }
                }
            }
            for (const SoundTriggerDefinition& candidate : untriggeredLoopSounds_) {
                if (belongsToSoundFamily(candidate.file, definition.file)) {
                    backend_->stop(candidate.file);
                }
            }
        }
    }
    for (const SoundTriggerDefinition& definition : definitions) {
        const std::filesystem::path file =
            definition.file.is_absolute() ? definition.file : basePath_ / definition.file;
        double gain = 1.0;
        if (!definition.volumeCurve.empty()) {
            gain = evaluateCurve(definition.volumeCurve, controlValue);
        }
        gain = std::clamp(gain, 0.0, 1.0);
        if (gain <= 0.0) {
            soundLog.Log("Skipping silent sound: trigger=" + name +
                         " gain=" + std::to_string(gain));
            continue;
        }
        soundLog.Log("Attempting sound playback: trigger=" + name + " file=" + file.string() +
                     " gain=" + std::to_string(gain) +
                     " loop=" + (definition.loop ? "true" : "false"));
        backend_->play(file, definition.loop, static_cast<float>(gain), 1.0f,
                   definition.position, definition.maxDistance);
    }
}

void SoundEngine::stop(const std::string& name) {
    const auto found = triggers_.find(lower(name));
    if (found == triggers_.end()) {
        return;
    }
    for (const SoundTriggerDefinition& definition : found->second) {
        if (definition.loop) {
            backend_->stop(definition.file);
        }
    }
    for (const SoundTriggerDefinition& definition : untriggeredLoopSounds_) {
        for (const SoundTriggerDefinition& trigger : found->second) {
            if (belongsToStartFamily(definition.file, trigger.file)) {
                backend_->stop(definition.file);
            }
        }
    }
}
