#include "PerfTrace.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace openbus::rendering {
namespace {

struct TraceEvent {
    char phase = 'B';
    const char* category = "";
    const char* name = "";
    long long timestampUs = 0;
    unsigned int threadId = 0;
};

class PerfTraceState {
  public:
    PerfTraceState()
        : enabled(parseEnabledFlag(std::getenv("OPENBUS_TRACE"))),
          start(std::chrono::steady_clock::now()) {}

    bool isEnabled() const {
        return enabled;
    }

    void push(char phase, const char* category, const char* name) {
        if (!enabled) {
            return;
        }
        TraceEvent event;
        event.phase = phase;
        event.category = category;
        event.name = name;
        event.timestampUs = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::steady_clock::now() - start)
                                .count();
        event.threadId = static_cast<unsigned int>(
            std::hash<std::thread::id>{}(std::this_thread::get_id()) & 0xFFFFFFFFU);
        std::lock_guard<std::mutex> lock(mutex);
        events.push_back(event);
    }

    ~PerfTraceState() {
        if (!enabled) {
            return;
        }
        std::filesystem::path outputPath = "openbus_trace.json";
        if (const char* configuredPath = std::getenv("OPENBUS_TRACE_FILE")) {
            if (*configuredPath != '\0') {
                outputPath = configuredPath;
            }
        }
        std::vector<TraceEvent> snapshot;
        {
            std::lock_guard<std::mutex> lock(mutex);
            snapshot = events;
        }
        std::ofstream output(outputPath, std::ios::trunc);
        if (!output) {
            return;
        }
        output << "{\n  \"traceEvents\": [\n";
        for (std::size_t index = 0; index < snapshot.size(); ++index) {
            const TraceEvent& event = snapshot[index];
            output << "    {\"name\":\"" << event.name << "\",\"cat\":\""
                   << event.category << "\",\"ph\":\"" << event.phase
                   << "\",\"ts\":" << event.timestampUs
                   << ",\"pid\":1,\"tid\":" << event.threadId << "}";
            if (index + 1 < snapshot.size()) {
                output << ",";
            }
            output << "\n";
        }
        output << "  ],\n  \"displayTimeUnit\": \"ms\"\n}\n";
    }

  private:
    bool enabled = false;
    std::chrono::steady_clock::time_point start;
    std::mutex mutex;
    std::vector<TraceEvent> events;
};

PerfTraceState& perfTrace() {
    static PerfTraceState trace;
    return trace;
}

}  // namespace

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

TraceScope::TraceScope(const char* category, const char* name)
    : category_(category), name_(name), active_(perfTrace().isEnabled()) {
    if (active_) {
        perfTrace().push('B', category_, name_);
    }
}

TraceScope::~TraceScope() {
    if (active_) {
        perfTrace().push('E', category_, name_);
    }
}

}  // namespace openbus::rendering
