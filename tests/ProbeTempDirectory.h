#pragma once

// Scratch directories for probes. Antivirus scanners and search indexers often hold
// freshly written files open for a moment, so cleanup must never throw (an uncaught
// filesystem_error would abort the probe) and each run gets its own directory so a
// locked leftover from an earlier run cannot interfere.

#include <chrono>
#include <filesystem>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>

#ifdef _WIN32
#include <process.h>
#define OPENBUS_PROBE_PID() _getpid()
#else
#include <unistd.h>
#define OPENBUS_PROBE_PID() getpid()
#endif

namespace openbus::probe {

// Best-effort recursive removal; retries briefly while another process holds a file.
inline void removeProbeDirectory(const std::filesystem::path& path) noexcept {
    for (int attempt = 0; attempt < 10; ++attempt) {
        std::error_code error;
        std::filesystem::remove_all(path, error);
        if (!error) {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

// Best-effort single-file removal for probes that write directly into the temp path.
inline void removeProbeFile(const std::filesystem::path& path) noexcept {
    std::error_code error;
    std::filesystem::remove(path, error);
}

// Creates an empty directory under the system temp path that is unique to this process,
// first sweeping (best-effort) directories left behind by earlier runs of the same probe.
[[nodiscard]] inline std::filesystem::path makeProbeDirectory(std::string_view name) {
    const std::filesystem::path tempRoot = std::filesystem::temp_directory_path();
    const std::string prefix = std::string(name) + "-";
    const std::filesystem::path::string_type nativePrefix = std::filesystem::path(prefix).native();
    std::error_code iterateError;
    for (std::filesystem::directory_iterator entry(tempRoot, iterateError), end;
         !iterateError && entry != end; entry.increment(iterateError)) {
        if (entry->path().filename().native().starts_with(nativePrefix)) {
            removeProbeDirectory(entry->path());
        }
    }
    const std::filesystem::path path = tempRoot / (prefix + std::to_string(OPENBUS_PROBE_PID()));
    removeProbeDirectory(path);
    std::filesystem::create_directories(path);
    return path;
}

} // namespace openbus::probe

#undef OPENBUS_PROBE_PID
