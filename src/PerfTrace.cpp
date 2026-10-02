#include "PerfTrace.h"

#include "Environment.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
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
        : traceEnabled(parseEnabledFlag(openbus::getEnvironment("OPENBUS_TRACE"))),
          enabled(traceEnabled),
          maxEvents(parseMaxEvents(openbus::getEnvironment("OPENBUS_TRACE_MAX_EVENTS"))),
          minDurationUs(parseMinDuration(openbus::getEnvironment("OPENBUS_TRACE_MIN_US"))),
          start(std::chrono::steady_clock::now()) {
        events.reserve(std::min<std::size_t>(maxEvents, 8192));
        if (traceEnabled) {
            traceOutputPath = configuredTracePath();
            traceOutput.open(traceOutputPath, std::ios::trunc);
            if (traceOutput) {
                traceOutput << "{\n  \"traceEvents\": [\n";
                traceWriter = std::thread(&PerfTraceState::traceWriterLoop, this);
            }
        }
    }

    bool isEnabled() const {
        return enabled;
    }

    std::int64_t nowUs() const {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now() - start)
            .count();
    }

    void pushDuration(const char* category, const char* name, std::int64_t startTimestampUs) {
        if (!enabled) {
            return;
        }
        const std::int64_t endTimestampUs = nowUs();
        const std::int64_t durationUs =
            std::max<std::int64_t>(0, endTimestampUs - startTimestampUs);
        if (durationUs < minDurationUs) {
            return;
        }
        const std::size_t eventSlot = acceptedEvents.fetch_add(1, std::memory_order_relaxed);
        if (eventSlot >= maxEvents) {
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
        event.sequence = nextSequence++;
        events.push_back(std::move(event));
    }

    ~PerfTraceState() {
        flush();
    }

    void flush() {
        stopTraceWriter();
        std::lock_guard<std::mutex> flushLock(flushMutex);
        if (flushed || !enabled) {
            return;
        }
        flushed = true;

        if (traceEnabled && traceOutput) {
            traceOutput << "\n  ],\n  \"displayTimeUnit\": \"ms\"\n}\n";
            traceOutput.flush();
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
        traceOutput.flush();
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

    bool traceEnabled = false;
    bool enabled = false;
    std::size_t maxEvents = 1000000;
    std::int64_t minDurationUs = 1;
    std::chrono::steady_clock::time_point start;
    std::mutex mutex;
    std::mutex flushMutex;
    std::condition_variable eventCondition;
    std::mutex traceOutputMutex;
    std::vector<TraceEvent> events;
    std::atomic<std::size_t> acceptedEvents = 0;
    std::filesystem::path traceOutputPath;
    std::ofstream traceOutput;
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

TraceScope::TraceScope(const char* category, const char* name)
    : category_(category), name_(name), active_(perfTrace().isEnabled()) {
    if (active_) {
        startTimestampUs_ = perfTrace().nowUs();
    }
}

TraceScope::~TraceScope() {
    if (active_) {
        perfTrace().pushDuration(category_, name_, startTimestampUs_);
    }
}
#endif

} // namespace openbus::rendering
