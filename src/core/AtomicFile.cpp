#include "core/AtomicFile.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <system_error>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace openbus::core {
namespace {

struct TemporaryFile {
    std::filesystem::path path;
    bool committed = false;

    ~TemporaryFile() {
        if (!committed) {
            std::error_code ignored;
            std::filesystem::remove(path, ignored);
        }
    }
};

} // namespace

void writeFileAtomically(const std::filesystem::path& path,
                         const std::function<void(std::ostream&)>& writeContent) {
    static std::atomic<std::uint64_t> sequence = 0;
    std::filesystem::path temporaryPath = path;
    temporaryPath += ".tmp-" +
                     std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
                     "-" + std::to_string(sequence.fetch_add(1, std::memory_order_relaxed));
    TemporaryFile temporary{temporaryPath};

    std::ofstream output(temporaryPath, std::ios::binary | std::ios::trunc);
    if (!output) {
        throw std::runtime_error("Unable to write temporary file: " + temporaryPath.string());
    }
    writeContent(output);
    output.flush();
    if (!output) {
        throw std::runtime_error("Failed while writing temporary file: " + temporaryPath.string());
    }
    output.close();
    if (!output) {
        throw std::runtime_error("Failed to close temporary file: " + temporaryPath.string());
    }

#ifdef _WIN32
    if (MoveFileExW(temporaryPath.c_str(), path.c_str(),
                    MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) == 0) {
        throw std::system_error(static_cast<int>(GetLastError()), std::system_category(),
                                "Unable to replace file " + path.string());
    }
#else
    std::error_code error;
    std::filesystem::rename(temporaryPath, path, error);
    if (error) {
        throw std::system_error(error, "Unable to replace file " + path.string());
    }
#endif
    temporary.committed = true;
}

} // namespace openbus::core
