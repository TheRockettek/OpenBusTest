#include "core/Environment.h"
#include "core/Logger.h"
#include "core/Utf8Path.h"
#include "map/MapAssetValidation.h"

#include <cstdlib>
#include <cstddef>
#include <exception>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

namespace {

void setDefaultEnvironment(const char* name, const char* value) {
    if (!openbus::environmentValue(name).empty()) {
        return;
    }
#ifdef _WIN32
    if (_putenv_s(name, value) != 0) {
#else
    if (setenv(name, value, 1) != 0) {
#endif
        throw std::runtime_error(std::string("Could not configure ") + name);
    }
}

} // namespace

#ifdef _WIN32
int wmain(int argc, wchar_t** argv) {
#else
int main(int argc, char** argv) {
#endif
    if (argc != 3) {
        std::cerr << "Usage: OpenBusMapAssetCheck <map-directory> <OMSI-root>\n";
        return 2;
    }
    bool loggingConfigured = false;
    try {
        setDefaultEnvironment("OPENBUS_LOG_LEVEL", "WARNING");
#ifdef _WIN32
        setDefaultEnvironment("OPENBUS_LOG_FILE", "NUL");
#else
        setDefaultEnvironment("OPENBUS_LOG_FILE", "/dev/null");
#endif
        loggingConfigured = true;
#ifdef _WIN32
        const auto report = openbus::map::validateMapAssets(argv[1], argv[2]);
#else
        const auto report = openbus::map::validateMapAssets(openbus::pathFromUtf8(argv[1]),
                                                            openbus::pathFromUtf8(argv[2]));
#endif
        Logger::Flush();
        std::map<std::string, std::size_t> findingCounts;
        for (const auto& finding : report.findings) {
            ++findingCounts[finding.category];
            if (finding.unsupported) {
                std::cout << "UNSUPPORTED ";
            }
            std::cout << finding.category << ": " << finding.detail;
            if (!finding.reference.empty()) {
                std::cout << " - " << finding.reference;
            }
            std::cout << " (referenced by " << openbus::pathToUtf8(finding.source) << ")";
            if (!finding.context.empty()) {
                std::cout << " (object directory " << openbus::pathToUtf8(finding.context) << ")";
            }
            std::cout << '\n';
        }
        if (!findingCounts.empty()) {
            std::cout << "Findings by category:\n";
            for (const auto& [category, count] : findingCounts) {
                std::cout << "  " << category << ": " << count << '\n';
            }
        }
        std::cout << "Scanned " << report.tilesScanned << " tiles; " << report.findings.size()
                  << " finding(s).\n";
        if (report.chronoTilesScanned != 0) {
            std::cout << "Scanned " << report.chronoTilesScanned
                      << " independent Chrono tile variant(s).\n";
        }
        if (report.chronoCombinedTilesScanned != 0) {
            std::cout << "Scanned " << report.chronoCombinedTilesScanned
                      << " date-selected combined Chrono tile variant(s).\n";
        }
        if (report.chronoTimetableDatesScanned != 0) {
            std::cout << "Checked " << report.chronoTimetableDatesScanned
                      << " date-selected Chrono timetable state(s).\n";
        }
        if (!report.complete) {
            std::cout << "INCOMPLETE: some transitive assets, runtime-selected variants, and "
                         "OMSI formats are not fully checked; this report cannot certify a map "
                         "for baking.\n";
        }
        return report.findings.empty() && report.complete ? 0 : report.findings.empty() ? 2 : 1;
    } catch (const std::exception& error) {
        if (loggingConfigured) {
            Logger::Flush();
        }
        std::cerr << "Map validation failed: " << error.what() << '\n';
        return 2;
    }
}
