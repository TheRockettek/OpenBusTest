#include "SoundEngine.h"

#include "ConfigurationParser.h"
#include "Logger.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <mutex>
#include <sstream>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#include <mmsystem.h>
#elif defined(OPENBUS_HAS_PIPEWIRE)
#include <pipewire/pipewire.h>
#include <spa/param/audio/format-utils.h>
#include <spa/param/props.h>
#include <spa/utils/defs.h>
#endif

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

} // namespace

#if defined(OPENBUS_HAS_PIPEWIRE)

struct SoundEngine::Backend {
    struct Clip {
        std::vector<float> samples;
        std::uint32_t channels = 2;
        std::uint32_t rate = 48000;
    };

    struct ActiveClip {
        std::shared_ptr<Clip> clip;
        std::size_t frame = 0;
        bool loop = false;
        float gain = 1.0f;
    };

    pw_thread_loop* loop = nullptr;
    pw_context* context = nullptr;
    pw_core* core = nullptr;
    pw_stream* stream = nullptr;
    spa_hook streamListener = {};
    std::mutex mutex;
    std::vector<ActiveClip> active;

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

    static std::shared_ptr<Clip> loadWav(const std::filesystem::path& path) {
        std::ifstream input(path, std::ios::binary);
        if (!input) {
            return {};
        }
        std::vector<std::uint8_t> data((std::istreambuf_iterator<char>(input)), {});
        if (data.size() < 44 || std::memcmp(data.data(), "RIFF", 4) != 0 ||
            std::memcmp(data.data() + 8, "WAVE", 4) != 0) {
            return {};
        }

        std::uint16_t format = 0;
        std::uint16_t channels = 0;
        std::uint32_t rate = 0;
        std::uint16_t bits = 0;
        std::size_t dataOffset = 0;
        std::size_t dataSize = 0;
        std::size_t offset = 12;
        while (offset + 8 <= data.size()) {
            const std::uint32_t size = read32(data, offset + 4);
            const std::size_t payload = offset + 8;
            if (payload + size > data.size()) {
                return {};
            }
            if (std::memcmp(data.data() + offset, "fmt ", 4) == 0 && size >= 16) {
                format = read16(data, payload);
                channels = read16(data, payload + 2);
                rate = read32(data, payload + 4);
                bits = read16(data, payload + 14);
            } else if (std::memcmp(data.data() + offset, "data", 4) == 0) {
                dataOffset = payload;
                dataSize = size;
            }
            offset = payload + size + (size & 1u);
        }
        if ((format != 1 && format != 3) || channels == 0 || rate == 0 ||
            (bits != 8 && bits != 16 && bits != 24 && bits != 32) || dataOffset == 0) {
            return {};
        }

        const std::size_t bytesPerSample = bits / 8;
        const std::size_t frameBytes = bytesPerSample * channels;
        if (frameBytes == 0 || dataSize < frameBytes) {
            return {};
        }
        auto clip = std::make_shared<Clip>();
        clip->channels = channels;
        clip->rate = rate;
        const std::size_t frames = dataSize / frameBytes;
        clip->samples.resize(frames * channels);
        for (std::size_t sample = 0; sample < clip->samples.size(); ++sample) {
            const std::size_t sampleOffset = dataOffset + sample * bytesPerSample;
            float value = 0.0f;
            if (format == 3 && bits == 32) {
                std::memcpy(&value, data.data() + sampleOffset, sizeof(value));
            } else if (bits == 8) {
                value = (static_cast<float>(data[sampleOffset]) - 128.0f) / 128.0f;
            } else if (bits == 16) {
                value = static_cast<float>(static_cast<std::int16_t>(read16(data, sampleOffset))) /
                        32768.0f;
            } else if (bits == 24) {
                std::int32_t integer = static_cast<std::int32_t>(data[sampleOffset]) |
                                       (static_cast<std::int32_t>(data[sampleOffset + 1]) << 8) |
                                       (static_cast<std::int32_t>(data[sampleOffset + 2]) << 16);
                if ((integer & 0x00800000) != 0) {
                    integer |= ~0x00ffffff;
                }
                value = static_cast<float>(integer) / 8388608.0f;
            } else {
                std::int32_t integer = static_cast<std::int32_t>(read32(data, sampleOffset));
                value = static_cast<float>(integer) / 2147483648.0f;
            }
            clip->samples[sample] = std::clamp(value, -1.0f, 1.0f);
        }
        return clip;
    }

    static void process(void* object) {
        auto* backend = static_cast<Backend*>(object);
        pw_buffer* buffer = pw_stream_dequeue_buffer(backend->stream);
        if (buffer == nullptr) {
            return;
        }
        spa_data& data = buffer->buffer->datas[0];
        const std::size_t frames = data.maxsize / (sizeof(float) * 2);
        auto* output = static_cast<float*>(data.data);
        std::fill(output, output + frames * 2, 0.0f);
        {
            std::lock_guard<std::mutex> lock(backend->mutex);
            for (std::size_t frame = 0; frame < frames; ++frame) {
                for (auto active = backend->active.begin(); active != backend->active.end();) {
                    if (active->frame >= active->clip->samples.size() / active->clip->channels) {
                        if (active->loop) {
                            active->frame = 0;
                        } else {
                            active = backend->active.erase(active);
                            continue;
                        }
                    }
                    const std::size_t sourceFrame = active->frame++;
                    const auto& clip = *active->clip;
                    const float left = clip.samples[sourceFrame * clip.channels];
                    const float right =
                        clip.channels > 1 ? clip.samples[sourceFrame * clip.channels + 1] : left;
                    output[frame * 2] += left * active->gain;
                    output[frame * 2 + 1] += right * active->gain;
                    ++active;
                }
            }
        }
        for (std::size_t sample = 0; sample < frames * 2; ++sample) {
            output[sample] = std::clamp(output[sample], -1.0f, 1.0f);
        }
        pw_stream_queue_buffer(backend->stream, buffer);
    }

    Backend() {
        pw_init(nullptr, nullptr);
        loop = pw_thread_loop_new("openbus-audio", nullptr);
        context = pw_context_new(pw_thread_loop_get_loop(loop), nullptr, 0);
        core = pw_context_connect(context, nullptr, 0);
        stream = pw_stream_new(core, "OpenBus",
                               pw_properties_new(PW_KEY_MEDIA_TYPE, "Audio", PW_KEY_MEDIA_CATEGORY,
                                                 "Playback", PW_KEY_MEDIA_ROLE, "Game", nullptr));
        static const pw_stream_events events = [] {
            pw_stream_events value = {};
            value.version = PW_VERSION_STREAM_EVENTS;
            value.process = process;
            return value;
        }();
        pw_stream_add_listener(stream, &streamListener, &events, this);
        pw_thread_loop_start(loop);
        std::uint8_t buffer[1024] = {};
        spa_pod_builder builder = SPA_POD_BUILDER_INIT(buffer, sizeof(buffer));
        const spa_pod* params[1] = {spa_pod_builder_add_object(
            &builder, SPA_TYPE_OBJECT_Format, SPA_PARAM_EnumFormat, SPA_FORMAT_mediaType,
            SPA_POD_Id(SPA_MEDIA_TYPE_audio), SPA_FORMAT_mediaSubtype,
            SPA_POD_Id(SPA_MEDIA_SUBTYPE_raw), SPA_FORMAT_AUDIO_format,
            SPA_POD_Id(SPA_AUDIO_FORMAT_F32), SPA_FORMAT_AUDIO_rate, SPA_POD_Int(48000),
            SPA_FORMAT_AUDIO_channels, SPA_POD_Int(2))};
        pw_stream_connect(
            stream, PW_DIRECTION_OUTPUT, PW_ID_ANY,
            static_cast<pw_stream_flags>(PW_STREAM_FLAG_AUTOCONNECT | PW_STREAM_FLAG_MAP_BUFFERS),
            params, 1);
    }

    ~Backend() {
        if (loop != nullptr) {
            pw_thread_loop_stop(loop);
        }
        if (stream != nullptr)
            pw_stream_destroy(stream);
        if (core != nullptr)
            pw_core_disconnect(core);
        if (context != nullptr)
            pw_context_destroy(context);
        if (loop != nullptr)
            pw_thread_loop_destroy(loop);
        pw_deinit();
    }

    void play(const std::filesystem::path& path, bool looped, float gain) {
        std::shared_ptr<Clip> clip = loadWav(path);
        if (!clip) {
            soundLog.Log("Unable to decode sound: " + path.string());
            return;
        }
        std::lock_guard<std::mutex> lock(mutex);
        active.push_back({std::move(clip), 0, looped, gain});
    }
};

#else

struct SoundEngine::Backend {};

#endif

SoundEngine::SoundEngine()
#if defined(OPENBUS_HAS_PIPEWIRE)
    : backend_(std::make_unique<Backend>())
#endif
{
}

SoundEngine::~SoundEngine() = default;

void SoundEngine::load(const std::filesystem::path& configPath) {
    triggers_.clear();
    basePath_ = configPath.parent_path();
    if (configPath.empty()) {
        return;
    }

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
            triggers_[name] = current;
        }
    };
    openbus::config::Line line;
    while (reader.next(line)) {
        if (!line.isKeyword()) {
            continue;
        }
        const std::string keyword = (line.keyword());
        if (keyword == "sound" || keyword == "loopsound") {
            openbus::config::Line value;
            ConfigurationDiagnostics diagnostics;
            if (!reader.readPayload(value, diagnostics, keyword)) {
                continue;
            }
            current = {};
            current.loop = keyword == "loopsound";
            current.file = resolve(configPath.parent_path(), openbus::config::trim(value.text));
            currentTriggerNames.clear();
            hasSound = true;
            continue;
        }
        if (keyword == "trigger") {
            openbus::config::Line value;
            ConfigurationDiagnostics diagnostics;
            if (hasSound && reader.readPayload(value, diagnostics, keyword)) {
                currentTriggerNames.push_back((openbus::config::trim(value.text)));
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
                current.maxDistance = (std::max)(0.0, maxDistance);
                updateCurrentTriggers();
            }
            continue;
        }
        if (keyword == "volcurve") {
            while (reader.next(line)) {
                if (line.isKeyword() && (line.keyword()) != "pnt") {
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
                        current.volumeCurve.push_back(point);
                    }
                    continue;
                }
                std::istringstream values(line.text);
                SoundCurvePoint point;
                if (values >> point.x >> point.y) {
                    current.volumeCurve.push_back(point);
                }
            }
            updateCurrentTriggers();
        }
    }
}

void SoundEngine::setListenerDistance(double distance) {
    listenerDistance_ = (std::max)(0.0, distance);
}

void SoundEngine::trigger(const std::string& name, const std::filesystem::path& overrideFile,
                          double controlValue) {
    const auto found = triggers_.find((name));
    if (found == triggers_.end() && overrideFile.empty()) {
        return;
    }
    const bool loop = found != triggers_.end() && found->second.loop && overrideFile.empty();
    double gain = 1.0;
    if (found != triggers_.end() && !found->second.volumeCurve.empty()) {
        const auto& points = found->second.volumeCurve;
        if (controlValue <= points.front().x) {
            gain = points.front().y;
        } else {
            gain = points.back().y;
            for (std::size_t index = 1; index < points.size(); ++index) {
                if (controlValue <= points[index].x) {
                    const auto& left = points[index - 1];
                    const auto& right = points[index];
                    const double fraction = (controlValue - left.x) / (right.x - left.x);
                    gain = left.y + fraction * (right.y - left.y);
                    break;
                }
            }
        }
        gain = std::clamp(gain, 0.0, 1.0);
    }
    if (found != triggers_.end() && found->second.maxDistance > 0.0) {
        gain *= std::clamp(1.0 - listenerDistance_ / found->second.maxDistance, 0.0, 1.0);
    }
    if (gain <= 0.0) {
        return;
    }
    const std::filesystem::path file =
        overrideFile.empty()
            ? found->second.file
            : (overrideFile.is_absolute() ? overrideFile : basePath_ / overrideFile);
#ifdef _WIN32
    const std::wstring widePath = file.wstring();
    const DWORD flags = SND_FILENAME | SND_ASYNC | SND_NODEFAULT | (loop ? SND_LOOP : 0);
    if (!PlaySoundW(widePath.c_str(), nullptr, flags)) {
        soundLog.Log("Unable to play sound: " + file.string());
    }
#else
#if defined(OPENBUS_HAS_PIPEWIRE)
    backend_->play(file, loop, static_cast<float>(gain));
#else
    static_cast<void>(file);
#endif
#endif
}