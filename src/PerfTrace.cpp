#include "PerfTrace.h"

#include "Environment.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <iostream>
#include <string>

#if OPENBUS_ENABLE_PERF_TRACE
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>
#endif

namespace openbus::rendering {
#if OPENBUS_ENABLE_PERF_TRACE
namespace {

struct TraceEvent {
    char phase = 'X';
    std::string category;
    std::string name;
    long long timestampUs = 0;
    long long durationUs = 0;
    unsigned int threadId = 0;
    std::uint64_t sequence = 0;
};

class PerfTraceState {
  public:
    PerfTraceState()
        : maxEvents(parseMaxEvents(openbus::getEnvironment("OPENBUS_TRACE_MAX_EVENTS"))),
          minDurationUs(parseMinDuration(openbus::getEnvironment("OPENBUS_TRACE_MIN_US"))),
          start(std::chrono::steady_clock::now()) {
        events.reserve(std::min<std::size_t>(maxEvents, 8192));
        if (parseEnabledFlag(openbus::getEnvironment("OPENBUS_TRACE"))) {
            setRecording(true);
        }
    }

    std::uint64_t recordingId() const {
        return activeRecordingId.load(std::memory_order_acquire);
    }

    bool setRecording(bool recording) {
        std::lock_guard<std::mutex> controlLock(controlMutex);
        if (flushed) {
            return false;
        }
        if (recording == (recordingId() != 0)) {
            return recording;
        }
        if (!recording) {
            {
                std::lock_guard<std::mutex> lock(mutex);
                activeRecordingId.store(0, std::memory_order_release);
            }
            stopTraceWriter();
            return false;
        }
        if (!traceOutput.is_open()) {
            traceOutputPath = configuredTracePath();
            traceOutput.open(traceOutputPath, std::ios::trunc);
            if (traceOutput) {
                traceOutput << "{\n  \"traceEvents\": [\n";
                writeTraceFooter();
            } else {
                std::cerr << "Failed to open performance trace output: " << traceOutputPath.string()
                          << '\n';
                return false;
            }
        }
        if (!traceOutput) {
            std::cerr << "Performance trace output is not writable: " << traceOutputPath.string()
                      << '\n';
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(mutex);
            writerStopRequested = false;
        }
        traceWriter = std::thread(&PerfTraceState::traceWriterLoop, this);
        activeRecordingId.store(++nextRecordingId, std::memory_order_release);
        return true;
    }

    std::int64_t nowUs() const {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now() - start)
            .count();
    }

    void pushDuration(const char* category, const char* name, std::int64_t startTimestampUs,
                      std::uint64_t scopeRecordingId) {
        if (recordingId() != scopeRecordingId) {
            return;
        }
        const std::int64_t endTimestampUs = nowUs();
        const std::int64_t durationUs =
            std::max<std::int64_t>(0, endTimestampUs - startTimestampUs);
        if (durationUs < minDurationUs) {
            return;
        }
        TraceEvent event;
        event.category = category != nullptr ? category : "";
        event.name = name != nullptr ? name : "";
        event.timestampUs = startTimestampUs;
        event.durationUs = durationUs;
        thread_local const unsigned int threadId = static_cast<unsigned int>(
            std::hash<std::thread::id>{}(std::this_thread::get_id()) & 0xFFFFFFFFU);
        event.threadId = threadId;
        std::lock_guard<std::mutex> lock(mutex);
        // Scopes crossing a pause/resume boundary must not include time spent navigating.
        if (recordingId() != scopeRecordingId || acceptedEvents >= maxEvents) {
            return;
        }
        ++acceptedEvents;
        event.sequence = nextSequence++;
        events.push_back(std::move(event));
    }

    ~PerfTraceState() {
        flush();
    }

    void flush() {
        std::lock_guard<std::mutex> controlLock(controlMutex);
        if (flushed) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex);
            activeRecordingId.store(0, std::memory_order_release);
        }
        stopTraceWriter();
        flushed = true;

        if (traceOutput.is_open()) {
            traceOutput.close();
        }
    }

  private:
    static std::filesystem::path configuredTracePath() {
        if (const char* configuredPath = openbus::getEnvironment("OPENBUS_TRACE_FILE")) {
            if (*configuredPath != '\0') {
                return configuredPath;
            }
        }
        return "openbus_trace.json";
    }

    void traceWriterLoop() {
        std::vector<TraceEvent> chunk;
        for (;;) {
            std::unique_lock<std::mutex> lock(mutex);
            eventCondition.wait_for(lock, std::chrono::milliseconds(100),
                                    [this] { return writerStopRequested || !events.empty(); });
            const bool stopRequested = writerStopRequested;
            if (events.empty() && stopRequested) {
                break;
            }
            chunk.clear();
            chunk.swap(events);
            lock.unlock();
            writeTraceChunk(chunk);
        }
    }

    void writeTraceChunk(const std::vector<TraceEvent>& chunk) {
        std::lock_guard<std::mutex> lock(traceOutputMutex);
        if (!traceOutput) {
            return;
        }
        if (chunk.empty()) {
            return;
        }
        traceOutput.seekp(traceFooterPosition);
        for (const TraceEvent& event : chunk) {
            if (!traceFirstEvent) {
                traceOutput << ",\n";
            }
            traceFirstEvent = false;
            traceOutput << "    {\"name\":\"" << event.name << "\",\"cat\":\"" << event.category
                        << "\",\"ph\":\"" << event.phase << "\",\"ts\":" << event.timestampUs
                        << ",\"dur\":" << event.durationUs
                        << ",\"pid\":1,\"tid\":" << event.threadId << "}";
        }
        writeTraceFooter();
    }

    void writeTraceFooter() {
        // Overwrite this footer with the next chunk so paused captures remain valid JSON.
        traceFooterPosition = traceOutput.tellp();
        traceOutput << "\n  ],\n  \"displayTimeUnit\": \"ms\"\n}\n";
        traceOutput.flush();
        if (!traceOutput) {
            std::cerr << "Failed to write performance trace output: " << traceOutputPath.string()
                      << '\n';
        }
    }

    void stopTraceWriter() {
        {
            std::lock_guard<std::mutex> lock(mutex);
            writerStopRequested = true;
        }
        eventCondition.notify_one();
        if (traceWriter.joinable()) {
            traceWriter.join();
        }
    }

    static std::size_t parseMaxEvents(const char* value) {
        if (value == nullptr || *value == '\0') {
            return 1000000;
        }
        char* end = nullptr;
        const unsigned long long parsed = std::strtoull(value, &end, 10);
        if (end == value || *end != '\0' || parsed == 0) {
            return 1000000;
        }
        return static_cast<std::size_t>(std::min<unsigned long long>(
            parsed, static_cast<unsigned long long>(std::numeric_limits<std::size_t>::max())));
    }

    static std::int64_t parseMinDuration(const char* value) {
        if (value == nullptr || *value == '\0') {
            return 1;
        }
        char* end = nullptr;
        const long long parsed = std::strtoll(value, &end, 10);
        if (end == value || *end != '\0') {
            return 1;
        }
        return std::max<std::int64_t>(1, parsed);
    }

    std::atomic<std::uint64_t> activeRecordingId = 0;
    std::uint64_t nextRecordingId = 0;
    std::size_t maxEvents = 1000000;
    std::int64_t minDurationUs = 1;
    std::chrono::steady_clock::time_point start;
    std::mutex mutex;
    std::mutex controlMutex;
    std::condition_variable eventCondition;
    std::mutex traceOutputMutex;
    std::vector<TraceEvent> events;
    std::size_t acceptedEvents = 0;
    std::filesystem::path traceOutputPath;
    std::ofstream traceOutput;
    std::streampos traceFooterPosition = 0;
    std::thread traceWriter;
    bool traceFirstEvent = true;
    bool writerStopRequested = false;
    std::uint64_t nextSequence = 0;
    bool flushed = false;
};

PerfTraceState& perfTrace() {
    static PerfTraceState trace;
    return trace;
}

} // namespace
#endif

bool parseEnabledFlag(const char* value) {
    if (value == nullptr) {
        return false;
    }
    std::string lowered = value;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return lowered == "1" || lowered == "true" || lowered == "on" || lowered == "yes";
}

#if OPENBUS_ENABLE_PERF_TRACE
void Flush() {
    perfTrace().flush();
}

bool IsRecording() {
    return perfTrace().recordingId() != 0;
}

bool SetRecording(bool recording) {
    return perfTrace().setRecording(recording);
}

TraceScope::TraceScope(const char* category, const char* name)
    : category_(category), name_(name), recordingId_(perfTrace().recordingId()) {
    if (recordingId_ != 0) {
        startTimestampUs_ = perfTrace().nowUs();
    }
}

TraceScope::~TraceScope() {
    if (recordingId_ != 0) {
        perfTrace().pushDuration(category_, name_, startTimestampUs_, recordingId_);
    }
}
#endif

} // namespace openbus::rendering
