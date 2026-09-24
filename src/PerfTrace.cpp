#include "PerfTrace.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace openbus::rendering {
namespace {

struct TraceEvent {
    char phase = 'B';
    const char* category = "";
    const char* name = "";
    long long timestampUs = 0;
    unsigned int threadId = 0;
    std::uint64_t sequence = 0;
};

class PerfTraceState {
  public:
    PerfTraceState()
        : traceEnabled(parseEnabledFlag(std::getenv("OPENBUS_TRACE"))),
          collapsedEnabled(traceEnabled ||
                           parseEnabledFlag(std::getenv("OPENBUS_TRACE_COLLAPSED"))),
          enabled(traceEnabled || collapsedEnabled), start(std::chrono::steady_clock::now()) {}

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
        event.sequence = nextSequence++;
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
        if (traceEnabled) {
            writeTraceJson(snapshot, outputPath);
        }
        if (collapsedEnabled) {
            std::filesystem::path collapsedPath = "openbus_trace.collapsed";
            if (const char* configuredPath = std::getenv("OPENBUS_TRACE_COLLAPSED_FILE")) {
                if (*configuredPath != '\0') {
                    collapsedPath = configuredPath;
                }
            }
            writeCollapsedStacks(snapshot, collapsedPath);
        }
    }

  private:
    static std::string frameName(const TraceEvent& event) {
        std::string frame;
        if (event.category != nullptr && *event.category != '\0') {
            frame = event.category;
            frame += ':';
        }
        if (event.name != nullptr) {
            frame += event.name;
        }
        for (char& character : frame) {
            if (character == ';' || character == '\n' || character == '\r') {
                character = '_';
            }
        }
        return frame.empty() ? "unknown" : frame;
    }

    static void writeTraceJson(const std::vector<TraceEvent>& snapshot,
                               const std::filesystem::path& outputPath) {
        std::ofstream output(outputPath, std::ios::trunc);
        if (!output) {
            return;
        }
        output << "{\n  \"traceEvents\": [\n";
        for (std::size_t index = 0; index < snapshot.size(); ++index) {
            const TraceEvent& event = snapshot[index];
            output << "    {\"name\":\"" << event.name << "\",\"cat\":\"" << event.category
                   << "\",\"ph\":\"" << event.phase << "\",\"ts\":" << event.timestampUs
                   << ",\"pid\":1,\"tid\":" << event.threadId << "}";
            if (index + 1 < snapshot.size()) {
                output << ",";
            }
            output << "\n";
        }
        output << "  ],\n  \"displayTimeUnit\": \"ms\"\n}\n";
    }

    static void writeCollapsedStacks(const std::vector<TraceEvent>& snapshot,
                                     const std::filesystem::path& outputPath) {
        std::unordered_map<unsigned int, std::vector<TraceEvent>> eventsByThread;
        for (const TraceEvent& event : snapshot) {
            eventsByThread[event.threadId].push_back(event);
        }

        std::map<std::string, std::uint64_t> foldedDurations;
        for (auto& threadEvents : eventsByThread) {
            std::vector<TraceEvent>& events = threadEvents.second;
            std::sort(events.begin(), events.end(),
                      [](const TraceEvent& first, const TraceEvent& second) {
                          if (first.timestampUs != second.timestampUs) {
                              return first.timestampUs < second.timestampUs;
                          }
                          return first.sequence < second.sequence;
                      });

            std::vector<std::string> stack;
            long long previousTimestamp = events.empty() ? 0 : events.front().timestampUs;
            const std::string threadFrame = "thread_" + std::to_string(threadEvents.first);
            for (const TraceEvent& event : events) {
                const long long timestamp = std::max(previousTimestamp, event.timestampUs);
                if (timestamp > previousTimestamp && !stack.empty()) {
                    std::string folded = threadFrame;
                    for (const std::string& frame : stack) {
                        folded += ';';
                        folded += frame;
                    }
                    foldedDurations[folded] +=
                        static_cast<std::uint64_t>(timestamp - previousTimestamp);
                }
                previousTimestamp = timestamp;

                if (event.phase == 'B') {
                    stack.push_back(frameName(event));
                    continue;
                }
                if (stack.empty()) {
                    continue;
                }
                const std::string closingFrame = frameName(event);
                const auto matching = std::find(stack.rbegin(), stack.rend(), closingFrame);
                if (matching == stack.rend()) {
                    stack.clear();
                } else {
                    stack.erase(matching.base() - 1, stack.end());
                }
            }
        }

        std::ofstream output(outputPath, std::ios::trunc);
        if (!output) {
            return;
        }
        for (const auto& folded : foldedDurations) {
            output << folded.first << ' ' << folded.second << '\n';
        }
    }

    bool traceEnabled = false;
    bool collapsedEnabled = false;
    bool enabled = false;
    std::chrono::steady_clock::time_point start;
    std::mutex mutex;
    std::vector<TraceEvent> events;
    std::uint64_t nextSequence = 0;
};

PerfTraceState& perfTrace() {
    static PerfTraceState trace;
    return trace;
}

} // namespace

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

} // namespace openbus::rendering
