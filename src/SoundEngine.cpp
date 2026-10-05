#include "SoundEngine.h"

#include "Environment.h"
#include "Logger.h"
#include "PerfTrace.h"

#include <AL/al.h>
#include <AL/alc.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

Logger soundLog = Logger("Sound");

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

bool dopplerEnabledFromEnvironment() {
    const char* setting = openbus::getEnvironment("OPENBUS_DOPPLER");
    if (setting == nullptr) {
        return false;
    }
    const std::string normalized = lower(setting);
    return normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on";
}

std::array<float, 3> velocityBetween(const std::array<double, 3>& current,
                                     const std::array<double, 3>& previous, double seconds) {
    constexpr double maximumSpeed = 100.0;
    if (!std::isfinite(seconds) || seconds < 0.001 || seconds > 1.0) {
        return {};
    }
    std::array<double, 3> velocity = {};
    double speedSquared = 0.0;
    for (std::size_t axis = 0; axis < velocity.size(); ++axis) {
        velocity[axis] = (current[axis] - previous[axis]) / seconds;
        speedSquared += velocity[axis] * velocity[axis];
    }
    const double speed = std::sqrt(speedSquared);
    if (!std::isfinite(speed)) {
        return {};
    }
    if (speed > maximumSpeed) {
        const double scale = maximumSpeed / speed;
        for (double& component : velocity) {
            component *= scale;
        }
    }
    return {static_cast<float>(velocity[0]), static_cast<float>(velocity[1]),
            static_cast<float>(velocity[2])};
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
        SoundPlaybackHandle handle = 0;
        ALuint source = 0;
        bool loop = false;
        std::array<double, 3> lastPosition = {};
        std::chrono::steady_clock::time_point lastPositionUpdate = {};
    };

    ALCdevice* device = nullptr;
    ALCcontext* context = nullptr;
    std::mutex mutex;
    std::unordered_map<std::string, std::shared_ptr<Clip>> clips;
    std::vector<ActiveSource> active;
    SoundPlaybackHandle nextHandle = 1;
    bool dopplerEnabled = false;
    bool hasListenerPosition = false;
    std::array<double, 3> lastListenerPosition = {};
    std::chrono::steady_clock::time_point lastListenerPositionUpdate = {};

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
        dopplerEnabled = dopplerEnabledFromEnvironment();
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
        alDistanceModel(AL_INVERSE_DISTANCE);
        alDopplerFactor(dopplerEnabled ? 1.0f : 0.0f);
        alDopplerVelocity(343.3f);
        alListenerf(AL_GAIN, 1.0f);
        soundLog.Log(std::string("OpenAL Doppler ") +
                     (dopplerEnabled ? "enabled (OPENBUS_DOPPLER)" : "disabled"));
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
        const auto now = std::chrono::steady_clock::now();
        std::array<float, 3> velocity = {};
        if (dopplerEnabled && hasListenerPosition) {
            const double elapsed =
                std::chrono::duration<double>(now - lastListenerPositionUpdate).count();
            velocity = velocityBetween(position, lastListenerPosition, elapsed);
        }
        const std::array<float, 6> orientation = {
            static_cast<float>(forward[0]), static_cast<float>(forward[1]),
            static_cast<float>(forward[2]), static_cast<float>(up[0]),
            static_cast<float>(up[1]), static_cast<float>(up[2])};
        alListener3f(AL_POSITION, static_cast<float>(position[0]), static_cast<float>(position[1]),
                     static_cast<float>(position[2]));
        alListener3f(AL_VELOCITY, velocity[0], velocity[1], velocity[2]);
        alListenerfv(AL_ORIENTATION, orientation.data());
        lastListenerPosition = position;
        lastListenerPositionUpdate = now;
        hasListenerPosition = true;
    }

    SoundPlaybackHandle createSourceLocked(const std::filesystem::path& path, bool looped,
                                           const SoundPlaybackParameters& parameters) {
        const std::shared_ptr<Clip> clip = loadClipLocked(path);
        if (!clip) {
            soundLog.Log("Unable to decode sound: " + path.string());
            return 0;
        }
        ALuint source = 0;
        alGenSources(1, &source);
        if (source == 0) {
            soundLog.Log("Unable to create OpenAL sound source");
            return 0;
        }
        alSourcei(source, AL_BUFFER, static_cast<ALint>(clip->buffer));
        alSourcei(source, AL_LOOPING, looped ? AL_TRUE : AL_FALSE);
        alSourcef(source, AL_GAIN, parameters.gain);
        alSourcef(source, AL_PITCH, parameters.pitch);
        alSourcei(source, AL_SOURCE_RELATIVE, AL_FALSE);
        alSource3f(source, AL_POSITION, static_cast<float>(parameters.position[0]),
                   static_cast<float>(parameters.position[1]),
                   static_cast<float>(parameters.position[2]));
        alSourcef(source, AL_REFERENCE_DISTANCE, 1.0f);
        alSourcef(source, AL_ROLLOFF_FACTOR, 1.0f);
        if (parameters.maxDistance > 0.0) {
            alSourcef(source, AL_MAX_DISTANCE, static_cast<float>(parameters.maxDistance));
        }
        alSourcePlay(source);
        if (alGetError() != AL_NO_ERROR) {
            alDeleteSources(1, &source);
            soundLog.Log("Unable to start OpenAL sound source: " + path.string());
            return 0;
        }
        SoundPlaybackHandle handle = nextHandle++;
        if (handle == 0) {
            handle = nextHandle++;
        }
        active.push_back({handle, source, looped, parameters.position,
                          std::chrono::steady_clock::now()});
        soundLog.Log("OpenAL playback started: file=" + path.string() +
                     " loop=" + (looped ? "true" : "false") +
                     " gain=" + std::to_string(parameters.gain) +
                     " pitch=" + std::to_string(parameters.pitch));
        return handle;
    }

    SoundPlaybackHandle play(const std::filesystem::path& path, bool looped,
                             const SoundPlaybackParameters& parameters) {
        openbus::rendering::TraceScope trace("sound", "Backend::play");
        if (context == nullptr) {
            return 0;
        }
        std::lock_guard<std::mutex> lock(mutex);
        cleanupStoppedSourcesLocked();
        return createSourceLocked(path, looped, parameters);
    }

    void updateLoop(SoundPlaybackHandle handle, const SoundPlaybackParameters& parameters) {
        openbus::rendering::TraceScope trace("sound", "Backend::updateLoop");
        if (context == nullptr || handle == 0) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex);
        const auto found = std::find_if(active.begin(), active.end(), [handle](const auto& source) {
            return source.handle == handle && source.loop;
        });
        if (found == active.end()) {
            return;
        }
        alSourcef(found->source, AL_GAIN, parameters.gain);
        alSourcef(found->source, AL_PITCH, parameters.pitch);
        alSource3f(found->source, AL_POSITION, static_cast<float>(parameters.position[0]),
                   static_cast<float>(parameters.position[1]),
                   static_cast<float>(parameters.position[2]));
        if (parameters.maxDistance > 0.0) {
            alSourcef(found->source, AL_MAX_DISTANCE,
                      static_cast<float>(parameters.maxDistance));
        }
        const auto now = std::chrono::steady_clock::now();
        std::array<float, 3> velocity = {};
        if (dopplerEnabled) {
            const double elapsed =
                std::chrono::duration<double>(now - found->lastPositionUpdate).count();
            velocity = velocityBetween(parameters.position, found->lastPosition, elapsed);
        }
        alSource3f(found->source, AL_VELOCITY, velocity[0], velocity[1], velocity[2]);
        found->lastPosition = parameters.position;
        found->lastPositionUpdate = now;
    }

    void stopLoop(SoundPlaybackHandle handle) {
        if (context == nullptr || handle == 0) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex);
        const auto found = std::find_if(active.begin(), active.end(), [handle](const auto& source) {
            return source.handle == handle && source.loop;
        });
        if (found == active.end()) {
            return;
        }
        alSourceStop(found->source);
        alDeleteSources(1, &found->source);
        active.erase(found);
    }
};

SoundEngine::SoundEngine() : backend_(std::make_unique<Backend>()) {}

SoundEngine::~SoundEngine() = default;

void SoundEngine::setListenerPose(const std::array<double, 3>& position,
                                  const std::array<double, 3>& forward,
                                  const std::array<double, 3>& up) {
    backend_->setListenerPose(position, forward, up);
}

SoundPlaybackHandle SoundEngine::play(const std::filesystem::path& path, bool looped,
                                      const SoundPlaybackParameters& parameters) {
    return backend_->play(path, looped, parameters);
}

void SoundEngine::updateLoop(SoundPlaybackHandle handle,
                             const SoundPlaybackParameters& parameters) {
    backend_->updateLoop(handle, parameters);
}

void SoundEngine::stopLoop(SoundPlaybackHandle handle) {
    backend_->stopLoop(handle);
}
