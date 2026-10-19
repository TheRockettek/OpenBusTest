#include "core/PerfTrace.h"
#include "ProbeTempDirectory.h"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <thread>
#include <vector>

namespace {

bool setEnvironment(const char* name, const std::string& value) {
#ifdef _WIN32
    return _putenv_s(name, value.c_str()) == 0;
#else
    return setenv(name, value.c_str(), 1) == 0;
#endif
}

void record(const char* name) {
    openbus::rendering::TraceScope scope("probe", name);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

std::string readTrace(const std::filesystem::path& path) {
    std::ifstream input(path);
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

bool savedTrace(const std::string& contents) {
    return contents.starts_with("{\n  \"traceEvents\": [\n") &&
           contents.ends_with("\n  ],\n  \"displayTimeUnit\": \"ms\"\n}\n");
}

} // namespace

int main(int argc, char** argv) {
    using namespace openbus::rendering;
    const std::string mode = argc > 1 ? argv[1] : "toggle";
    const auto path = argc > 2 ? std::filesystem::path(argv[2]) :
                      std::filesystem::temp_directory_path() /
                      ("openbus_trace_probe_" +
                       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
                       ".json");
    const auto outputPath = mode == "failure" ? path / "missing" / "trace.json" : path;
    if (!setEnvironment("OPENBUS_TRACE", mode == "autostart" ? "1" : "") ||
        !setEnvironment("OPENBUS_TRACE_FILE", outputPath.string()) ||
        !setEnvironment("OPENBUS_TRACE_MAX_EVENTS", mode == "cap" ? "2" : "10000") ||
        !setEnvironment("OPENBUS_TRACE_MIN_US", "1")) {
        std::cerr << "Could not configure trace probe environment\n";
        return 1;
    }

    bool passed = true;
#if OPENBUS_ENABLE_PERF_TRACE
    if (mode == "failure") {
        passed = !SetRecording(true) && !IsRecording() && !std::filesystem::exists(path);
    } else if (mode == "autostart") {
        passed = IsRecording();
        record("autostart");
        SetRecording(false);
        passed = passed && savedTrace(readTrace(path)) &&
                 readTrace(path).find("\"name\":\"autostart\"") != std::string::npos;
    } else if (mode == "cap") {
        SetRecording(true);
        for (int i = 0; i < 5; ++i) {
            record("limited");
        }
        SetRecording(false);
        const std::string contents = readTrace(path);
        const std::string marker = "\"name\":\"limited\"";
        std::size_t count = 0;
        for (auto pos = contents.find(marker); pos != std::string::npos;
             pos = contents.find(marker, pos + marker.size())) {
            ++count;
        }
        passed = savedTrace(contents) && count == 2;
    } else {
        passed = !IsRecording();
        record("before-start");
        passed = passed && !std::filesystem::exists(path) && SetRecording(true);
        passed = SetRecording(true) && passed;
        record("first");
        {
            TraceScope spanningPause("probe", "spanning-pause");
            SetRecording(false);
            passed = passed && !IsRecording() && savedTrace(readTrace(path));
            record("paused");
            passed = SetRecording(true) && passed;
        }
        record("second");
        std::vector<std::thread> workers;
        for (int i = 0; i < 4; ++i) {
            workers.emplace_back([] { record("worker"); });
        }
        for (auto& worker : workers) {
            worker.join();
        }
        SetRecording(false);
        SetRecording(false);
        const std::string contents = readTrace(path);
        passed = passed && savedTrace(contents) &&
                 contents.find("\"name\":\"first\"") != std::string::npos &&
                 contents.find("\"name\":\"second\"") != std::string::npos &&
                 contents.find("\"name\":\"worker\"") != std::string::npos &&
                 contents.find("before-start") == std::string::npos &&
                 contents.find("\"name\":\"paused\"") == std::string::npos &&
                 contents.find("spanning-pause") == std::string::npos;
    }
#else
    record("disabled-build");
    passed = !IsRecording() && !SetRecording(true) && !std::filesystem::exists(path);
#endif
    Flush();
    Flush();
    passed = !IsRecording() && !SetRecording(true) && passed;
    if (argc <= 2) {
        openbus::probe::removeProbeFile(path);
    }
    if (!passed) {
        std::cerr << "Performance trace probe failed: " << mode << '\n';
    }
    return passed ? 0 : 1;
}
