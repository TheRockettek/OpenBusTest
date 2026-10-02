#include "SoundEngine.h"

#include "ConfigurationParser.h"
#include "Logger.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <mutex>
#include <sstream>
#include <string_view>
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
    const std::string suffix = "_start";
    if (startStem.size() <= suffix.size() ||
        startStem.compare(startStem.size() - suffix.size(), suffix.size(), suffix) != 0) {
        return false;
    }
    const std::string family = startStem.substr(0, startStem.size() - suffix.size());
    return loopPath.parent_path() == startPath.parent_path() &&
           (loopStem == family + "_loop" || loopStem == family + "-loop");
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
        std::filesystem::path path;
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
        if (looped && std::any_of(active.begin(), active.end(), [&path](const ActiveClip& voice) {
                return voice.loop && voice.path == path;
            })) {
            return;
        }
        active.push_back({path, std::move(clip), 0, looped, gain});
    }

    void stop(const std::filesystem::path& path) {
        std::lock_guard<std::mutex> lock(mutex);
        active.erase(std::remove_if(active.begin(), active.end(),
                                    [&path](const ActiveClip& voice) {
                                        return voice.loop && voice.path == path;
                                    }),
                     active.end());
    }
};

#elif defined(_WIN32)

struct SoundEngine::Backend {
    struct Clip {
        std::vector<float> samples;
        std::uint32_t channels = 2;
        std::uint32_t rate = 48000;
    };

    struct ActiveClip {
        std::filesystem::path path;
        std::shared_ptr<Clip> clip;
        double frame = 0.0;
        bool loop = false;
        float gain = 1.0f;
    };

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
        for (std::size_t offset = 12; offset + 8 <= data.size();) {
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
        clip->samples.resize((dataSize / frameBytes) * channels);
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
                value = static_cast<float>(static_cast<std::int32_t>(read32(data, sampleOffset))) /
                        2147483648.0f;
            }
            clip->samples[sample] = std::clamp(value, -1.0f, 1.0f);
        }
        return clip;
    }

    static constexpr std::size_t kBufferFrames = 2048;
    static constexpr std::size_t kBufferCount = 3;
    struct WaveBuffer {
        WAVEHDR header = {};
        std::array<std::int16_t, kBufferFrames * 2> samples = {};
    };

    HWAVEOUT output = nullptr;
    std::array<WaveBuffer, kBufferCount> buffers = {};
    std::mutex mutex;
    std::vector<ActiveClip> active;
    std::atomic<bool> shuttingDown = false;

    static void CALLBACK callback(HWAVEOUT, UINT message, DWORD_PTR instance, DWORD_PTR parameter,
                                  DWORD_PTR) {
        if (message == WOM_DONE && instance != 0) {
            auto* backend = static_cast<Backend*>(reinterpret_cast<void*>(instance));
            if (!backend->shuttingDown.load(std::memory_order_acquire)) {
                backend->refill(reinterpret_cast<WAVEHDR*>(parameter));
            }
        }
    }

    void refill(WAVEHDR* completed) {
        WaveBuffer* buffer = nullptr;
        for (WaveBuffer& candidate : buffers) {
            if (&candidate.header == completed) {
                buffer = &candidate;
                break;
            }
        }
        if (buffer == nullptr) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex);
            std::fill(buffer->samples.begin(), buffer->samples.end(), 0);
            for (std::size_t frame = 0; frame < kBufferFrames; ++frame) {
                float left = 0.0f;
                float right = 0.0f;
                for (auto voice = active.begin(); voice != active.end();) {
                    const std::size_t sourceFrames =
                        voice->clip->samples.size() / voice->clip->channels;
                    if (voice->frame >= static_cast<double>(sourceFrames)) {
                        if (voice->loop) {
                            voice->frame = 0.0;
                        } else {
                            voice = active.erase(voice);
                            continue;
                        }
                    }
                    const std::size_t sourceFrame = static_cast<std::size_t>(voice->frame);
                    const Clip& clip = *voice->clip;
                    left += clip.samples[sourceFrame * clip.channels] * voice->gain;
                    right += (clip.channels > 1 ? clip.samples[sourceFrame * clip.channels + 1]
                                                : clip.samples[sourceFrame * clip.channels]) *
                             voice->gain;
                    voice->frame += static_cast<double>(clip.rate) / 48000.0;
                    ++voice;
                }
                buffer->samples[frame * 2] = static_cast<std::int16_t>(
                    std::lround(std::clamp(left, -1.0f, 1.0f) * 32767.0f));
                buffer->samples[frame * 2 + 1] = static_cast<std::int16_t>(
                    std::lround(std::clamp(right, -1.0f, 1.0f) * 32767.0f));
            }
        }
        if (waveOutWrite(output, &buffer->header, sizeof(buffer->header)) != MMSYSERR_NOERROR) {
            soundLog.Log("Unable to queue mixed audio buffer");
        }
    }

    Backend() {
        WAVEFORMATEX format = {};
        format.wFormatTag = WAVE_FORMAT_PCM;
        format.nChannels = 2;
        format.nSamplesPerSec = 48000;
        format.wBitsPerSample = 16;
        format.nBlockAlign = format.nChannels * format.wBitsPerSample / 8;
        format.nAvgBytesPerSec = format.nSamplesPerSec * format.nBlockAlign;
        if (waveOutOpen(&output, WAVE_MAPPER, &format,
                        reinterpret_cast<DWORD_PTR>(&Backend::callback),
                        reinterpret_cast<DWORD_PTR>(this), CALLBACK_FUNCTION) != MMSYSERR_NOERROR) {
            output = nullptr;
            return;
        }
        for (WaveBuffer& buffer : buffers) {
            buffer.header.lpData = reinterpret_cast<LPSTR>(buffer.samples.data());
            buffer.header.dwBufferLength = static_cast<DWORD>(sizeof(buffer.samples));
            waveOutPrepareHeader(output, &buffer.header, sizeof(buffer.header));
            refill(&buffer.header);
        }
    }

    ~Backend() {
        if (output == nullptr) {
            return;
        }
        shuttingDown.store(true, std::memory_order_release);
        waveOutReset(output);
        for (WaveBuffer& buffer : buffers) {
            waveOutUnprepareHeader(output, &buffer.header, sizeof(buffer.header));
        }
        waveOutClose(output);
    }

    void play(const std::filesystem::path& path, bool looped, float gain) {
        if (output == nullptr) {
            soundLog.Log("Windows audio mixer is unavailable");
            return;
        }
        std::shared_ptr<Clip> clip = loadWav(path);
        if (!clip) {
            soundLog.Log("Unable to decode sound: " + path.string());
            return;
        }
        std::lock_guard<std::mutex> lock(mutex);
        if (looped && std::any_of(active.begin(), active.end(), [&path](const ActiveClip& voice) {
                return voice.loop && voice.path == path;
            })) {
            return;
        }
        active.push_back({path, std::move(clip), 0.0, looped, gain});
    }

    void stop(const std::filesystem::path& path) {
        std::lock_guard<std::mutex> lock(mutex);
        active.erase(std::remove_if(active.begin(), active.end(),
                                    [&path](const ActiveClip& voice) {
                                        return voice.loop && voice.path == path;
                                    }),
                     active.end());
    }
};

#else

struct SoundEngine::Backend {};

#endif

SoundEngine::SoundEngine()
#if defined(OPENBUS_HAS_PIPEWIRE) || defined(_WIN32)
    : backend_(std::make_unique<Backend>())
#endif
{
}

SoundEngine::~SoundEngine() = default;

void SoundEngine::load(const std::filesystem::path& configPath) {
    if (configPath.empty()) {
        return;
    }
    basePath_ = configPath.parent_path();

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
        const std::string keyword = (line.keyword());
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
    if (hasSound && current.loop && currentTriggerNames.empty()) {
        untriggeredLoopSounds_.push_back(current);
    }
}

void SoundEngine::setListenerDistance(double distance) {
    listenerDistance_ = (std::max)(0.0, distance);
}

void SoundEngine::trigger(const std::string& name, const std::filesystem::path& overrideFile,
                          double controlValue) {
    const auto found = triggers_.find(lower(name));
    if (found == triggers_.end() && overrideFile.empty()) {
        soundLog.Log("Skipping unknown sound trigger: " + name);
        return;
    }
    std::vector<SoundTriggerDefinition> definitions;
    std::vector<std::filesystem::path> inferredLoopFiles;
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
                    inferredLoopFiles.push_back(loop.file);
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
        definitions = {SoundTriggerDefinition{overrideFile, false, 0, 0.0, {}}};
    }
#if defined(OPENBUS_HAS_PIPEWIRE) || defined(_WIN32)
    if (overrideFile.empty()) {
        for (const SoundTriggerDefinition& definition : definitions) {
            if (isEndSoundFile(definition.file)) {
                for (const auto& entry : triggers_) {
                    for (const SoundTriggerDefinition& candidate : entry.second) {
                        if (candidate.loop &&
                            belongsToSoundFamily(candidate.file, definition.file)) {
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
    }
#endif
    for (const SoundTriggerDefinition& definition : definitions) {
        const bool loop = definition.loop;
        const bool inferredLoop = std::find(inferredLoopFiles.begin(), inferredLoopFiles.end(),
                                            definition.file) != inferredLoopFiles.end();
        const std::filesystem::path file =
            definition.file.is_absolute() ? definition.file : basePath_ / definition.file;
        double gain = 1.0;
        // A start-triggered loop is commonly paired with a volume curve driven by a
        // script variable. The trigger itself carries no value, so evaluating that
        // curve at zero would suppress the loop permanently; its level cannot be
        // updated until another sound event is emitted. Start the inferred loop at
        // full volume and let the configured end trigger stop it.
        if (!definition.volumeCurve.empty() && !(inferredLoop && controlValue <= 0.0)) {
            const auto& points = definition.volumeCurve;
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
        }
        gain = std::clamp(gain, 0.0, 1.0);
        const double maxDistance = definition.maxDistance;
        if (maxDistance > 0.0) {
            if (listenerDistance_ >= maxDistance) {
                soundLog.Log("Skipping sound because listener is too far: trigger=" + name +
                             " distance=" + std::to_string(listenerDistance_) +
                             " maxDistance=" + std::to_string(maxDistance));
                continue;
            }
            gain *= std::clamp(1.0 - listenerDistance_ / maxDistance, 0.0, 1.0);
        }
        if (gain <= 0.0) {
            soundLog.Log("Skipping silent sound: trigger=" + name +
                         " distance=" + std::to_string(listenerDistance_) + " maxDistance=" +
                         std::to_string(maxDistance) + " gain=" + std::to_string(gain));
            continue;
        }
        soundLog.Log("Attempting sound playback: trigger=" + name + " file=" + file.string() +
                     " distance=" + std::to_string(listenerDistance_) +
                     " maxDistance=" + std::to_string(maxDistance) +
                     " gain=" + std::to_string(gain) + " loop=" + (loop ? "true" : "false"));
#ifdef _WIN32
        backend_->play(file, loop, static_cast<float>(gain));
#else
#if defined(OPENBUS_HAS_PIPEWIRE)
        backend_->play(file, loop, static_cast<float>(gain));
#else
        static_cast<void>(file);
#endif
#endif
    }
}

void SoundEngine::stop(const std::string& name) {
    const auto found = triggers_.find(lower(name));
    if (found == triggers_.end()) {
        return;
    }
#if defined(OPENBUS_HAS_PIPEWIRE) || defined(_WIN32)
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
#endif
}