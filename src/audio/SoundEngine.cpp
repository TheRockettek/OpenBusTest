#include "audio/SoundEngine.h"

#include "assets/WavLoader.h"
#include "core/Environment.h"
#include "core/Logger.h"
#include "core/PerfTrace.h"
#include "core/Utf8Path.h"

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
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
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

std::string formatWorldPosition(const std::array<double, 3>& position) {
    std::ostringstream result;
    result << std::fixed << std::setprecision(2) << '(' << position[0] << ", " << position[1]
           << ", " << position[2] << ')';
    return result.str();
}

double distanceBetween(const std::array<double, 3>& first, const std::array<double, 3>& second) {
    return std::hypot(first[0] - second[0], first[1] - second[1], first[2] - second[2]);
}

bool withinLoggingRange(double distance, double maxDistance) {
    return maxDistance <= 0.0 || distance <= maxDistance;
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
        std::filesystem::path path;
        bool loop = false;
        std::array<double, 3> lastPosition = {};
        std::array<double, 3> lastLoggedPosition = {};
        std::chrono::steady_clock::time_point lastPositionUpdate = {};
    };

    ALCdevice* device = nullptr;
    ALCcontext* context = nullptr;
    std::mutex mutex;
    std::unordered_map<std::string, std::shared_ptr<Clip>> clips;
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> failedClips;
    std::vector<ActiveSource> active;
    SoundPlaybackHandle nextHandle = 1;
    bool dopplerEnabled = false;
    bool hasListenerPosition = false;
    bool loggedUnavailableContext = false;
    std::array<double, 3> lastListenerPosition = {};
    std::chrono::steady_clock::time_point lastListenerPositionUpdate = {};

    static std::shared_ptr<Clip> decodeWav(const std::filesystem::path& path) {
        auto decoded = openbus::assets::decodeWavFile(path);
        if (!decoded) {
            return {};
        }
        auto clip = std::make_shared<Clip>();
        clip->samples = std::move(decoded->monoSamples);
        clip->rate = static_cast<ALsizei>(decoded->sampleRate);
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
        const std::string key = openbus::pathToUtf8(path.lexically_normal().make_preferred());
        const auto found = clips.find(key);
        if (found != clips.end()) {
            return found->second;
        }
        const auto now = std::chrono::steady_clock::now();
        const auto failed = failedClips.find(key);
        if (failed != failedClips.end() && now < failed->second) {
            return {};
        }
        auto clip = decodeWav(path);
        if (!clip) {
            if (failed == failedClips.end()) {
                soundLog.Warning("Unable to decode sound: " + openbus::pathToUtf8(path) +
                                 " (retrying periodically)");
            }
            failedClips.insert_or_assign(key, now + std::chrono::seconds(1));
            return {};
        }
        failedClips.erase(key);
        alGenBuffers(1, &clip->buffer);
        alBufferData(clip->buffer, clip->format, clip->samples.data(),
                     static_cast<ALsizei>(clip->samples.size() * sizeof(std::int16_t)), clip->rate);
        if (alGetError() != AL_NO_ERROR) {
            alDeleteBuffers(1, &clip->buffer);
            soundLog.Warning("Unable to create OpenAL buffer for sound: " +
                             openbus::pathToUtf8(path));
            return {};
        }
        clips.emplace(key, clip);
        return clip;
    }

    Backend() {
        dopplerEnabled = dopplerEnabledFromEnvironment();
        device = alcOpenDevice(nullptr);
        if (device == nullptr) {
            soundLog.Error("Unable to open an OpenAL audio device");
            return;
        }
        context = alcCreateContext(device, nullptr);
        if (context == nullptr || alcMakeContextCurrent(context) == ALC_FALSE) {
            soundLog.Error("Unable to create an OpenAL audio context");
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
        cleanupStoppedSourcesLocked();
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
            static_cast<float>(up[1]),      static_cast<float>(up[2])};
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
            return 0;
        }
        ALuint source = 0;
        alGenSources(1, &source);
        if (source == 0) {
            soundLog.Warning("Unable to create OpenAL sound source");
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
        alSourcef(source, AL_REFERENCE_DISTANCE, parameters.referenceDistance);
        alSourcef(source, AL_ROLLOFF_FACTOR, parameters.rolloffFactor);
        if (parameters.maxDistance > 0.0) {
            alSourcef(source, AL_MAX_DISTANCE, static_cast<float>(parameters.maxDistance));
        }
        alSourcePlay(source);
        if (alGetError() != AL_NO_ERROR) {
            alDeleteSources(1, &source);
            soundLog.Warning("Unable to start OpenAL sound source: " + openbus::pathToUtf8(path));
            return 0;
        }
        SoundPlaybackHandle handle = nextHandle++;
        if (handle == 0) {
            handle = nextHandle++;
        }
        active.push_back({handle, source, path, looped, parameters.position, parameters.position,
                          std::chrono::steady_clock::now()});
        if (hasListenerPosition) {
            const double distance = distanceBetween(parameters.position, lastListenerPosition);
            if (withinLoggingRange(distance, parameters.maxDistance)) {
                std::ostringstream message;
                message << "OpenAL source started: handle=" << handle
                        << " file=" << openbus::pathToUtf8(path)
                        << " loop=" << (looped ? "true" : "false")
                        << " source_world=" << formatWorldPosition(parameters.position)
                        << " gain=" << std::fixed << std::setprecision(3) << parameters.gain
                        << " max_distance=" << parameters.maxDistance
                        << " reference_distance=" << parameters.referenceDistance
                        << " rolloff=" << parameters.rolloffFactor
                        << " listener_world=" << formatWorldPosition(lastListenerPosition)
                        << " listener_distance=" << std::setprecision(2) << distance << 'm';
                soundLog.Trace(message.str());
            }
        }
        return handle;
    }

    SoundPlaybackHandle play(const std::filesystem::path& path, bool looped,
                             const SoundPlaybackParameters& parameters) {
        openbus::rendering::TraceScope trace("sound", "Backend::play");
        if (context == nullptr) {
            if (!loggedUnavailableContext) {
                soundLog.Warning(
                    "Skipping sound playback because the OpenAL context is unavailable: file=" +
                    openbus::pathToUtf8(path) +
                    " source_world=" + formatWorldPosition(parameters.position));
                loggedUnavailableContext = true;
            }
            return 0;
        }
        std::lock_guard<std::mutex> lock(mutex);
        cleanupStoppedSourcesLocked();
        return createSourceLocked(path, looped, parameters);
    }

    bool isPlaying(SoundPlaybackHandle handle) {
        if (context == nullptr || handle == 0) {
            return false;
        }
        std::lock_guard<std::mutex> lock(mutex);
        cleanupStoppedSourcesLocked();
        const auto found = std::find_if(active.begin(), active.end(), [handle](const auto& source) {
            return source.handle == handle;
        });
        if (found == active.end()) {
            return false;
        }
        ALint state = AL_STOPPED;
        alGetSourcei(found->source, AL_SOURCE_STATE, &state);
        // A one-shot can finish after the cleanup pass. Keep loop ownership unchanged.
        if (state == AL_STOPPED && !found->loop) {
            alDeleteSources(1, &found->source);
            active.erase(found);
        }
        return state == AL_PLAYING;
    }

    void update(SoundPlaybackHandle handle, const SoundPlaybackParameters& parameters,
                bool loopsOnly = false) {
        openbus::rendering::TraceScope trace("sound", "Backend::update");
        if (context == nullptr || handle == 0) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex);
        cleanupStoppedSourcesLocked();
        const auto found =
            std::find_if(active.begin(), active.end(), [handle, loopsOnly](const auto& source) {
                return source.handle == handle && (!loopsOnly || source.loop);
            });
        if (found == active.end()) {
            return;
        }
        alSourcef(found->source, AL_GAIN, parameters.gain);
        alSourcef(found->source, AL_PITCH, parameters.pitch);
        alSourcef(found->source, AL_REFERENCE_DISTANCE, parameters.referenceDistance);
        alSourcef(found->source, AL_ROLLOFF_FACTOR, parameters.rolloffFactor);
        alSource3f(found->source, AL_POSITION, static_cast<float>(parameters.position[0]),
                   static_cast<float>(parameters.position[1]),
                   static_cast<float>(parameters.position[2]));
        if (parameters.maxDistance > 0.0) {
            alSourcef(found->source, AL_MAX_DISTANCE, static_cast<float>(parameters.maxDistance));
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
        if (hasListenerPosition &&
            distanceBetween(parameters.position, found->lastLoggedPosition) >= 5.0) {
            const double distance = distanceBetween(parameters.position, lastListenerPosition);
            if (withinLoggingRange(distance, parameters.maxDistance)) {
                found->lastLoggedPosition = parameters.position;
                std::ostringstream message;
                message << "OpenAL source moved: handle=" << handle
                        << " file=" << openbus::pathToUtf8(found->path)
                        << " source_world=" << formatWorldPosition(parameters.position)
                        << " gain=" << std::fixed << std::setprecision(3) << parameters.gain
                        << " rolloff=" << parameters.rolloffFactor
                        << " reference_distance=" << parameters.referenceDistance
                        << " listener_world=" << formatWorldPosition(lastListenerPosition)
                        << " listener_distance=" << std::setprecision(2) << distance << 'm';
                soundLog.Trace(message.str());
            }
        }
    }

    void stop(SoundPlaybackHandle handle, bool loopsOnly = false) {
        if (context == nullptr || handle == 0) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex);
        const auto found =
            std::find_if(active.begin(), active.end(), [handle, loopsOnly](const auto& source) {
                return source.handle == handle && (!loopsOnly || source.loop);
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

bool SoundEngine::isAvailable() const {
    return backend_->context != nullptr;
}

void SoundEngine::setListenerPose(const std::array<double, 3>& position,
                                  const std::array<double, 3>& forward,
                                  const std::array<double, 3>& up) {
    backend_->setListenerPose(position, forward, up);
}

SoundPlaybackHandle SoundEngine::play(const std::filesystem::path& path, bool looped,
                                      const SoundPlaybackParameters& parameters) {
    return backend_->play(path, looped, parameters);
}

bool SoundEngine::isPlaying(SoundPlaybackHandle handle) {
    return backend_->isPlaying(handle);
}

void SoundEngine::update(SoundPlaybackHandle handle, const SoundPlaybackParameters& parameters) {
    backend_->update(handle, parameters);
}

void SoundEngine::stop(SoundPlaybackHandle handle) {
    backend_->stop(handle);
}

void SoundEngine::updateLoop(SoundPlaybackHandle handle,
                             const SoundPlaybackParameters& parameters) {
    // Preserve the engine's loop-only compatibility API; update() accepts any source.
    backend_->update(handle, parameters, true);
}

void SoundEngine::stopLoop(SoundPlaybackHandle handle) {
    backend_->stop(handle, true);
}
