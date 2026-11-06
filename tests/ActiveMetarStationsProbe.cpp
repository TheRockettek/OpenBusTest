#include "core/ActiveMetarStations.h"
#include "core/MetarNoaa.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

std::string row(const char* code, const char* date) {
    return std::string("<tr><td><a href=\"") + code + ".TXT\">" + code +
           ".TXT</a></td><td align=\"right\">" + date +
           "  </td><td align=\"right\">100 </td></tr>\n";
}

} // namespace

int main(int argc, char** argv) {
    using namespace std::chrono;
    if (argc == 2 && std::string(argv[1]) == "--live") {
        const auto stations = openbus::parseRecentMetarStations(
            openbus::fetchNoaaMetarDirectory(), system_clock::now());
        std::cout << "Recently updated NOAA stations: " << stations.size() << '\n';
        return stations.empty() ? 1 : 0;
    }
    bool passed = true;
    const auto check = [&](bool condition, const char* message) {
        if (!condition) {
            std::cerr << message << '\n';
            passed = false;
        }
    };
    const auto now = sys_days{year{2026}/November/6} + hours{22} + minutes{45};
    const std::string html = "<html><table>\n" +
        row("KJFK", "06-Nov-2026 22:30") +
        row("EGLL", "06-Nov-2026 21:45") +
        row("YSSY", "06-Nov-2026 21:44") +
        row("ABCD", "06-Nov-2026 22:50") +
        row("EFGH", "06-Nov-2026 22:51") + "</table></html>";
    const auto active = openbus::parseRecentMetarStations(html, now);
    check(active == std::vector<std::string>{"ABCD", "EGLL", "KJFK"},
          "directory time window or ordering was incorrect");
    const auto compact = row("EGLL", "06-Nov-2026 22:30") +
                         row("KJFK", "06-Nov-2026 22:30");
    std::string singleLine = compact;
    singleLine.erase(std::remove(singleLine.begin(), singleLine.end(), '\n'), singleLine.end());
    check(openbus::parseRecentMetarStations(singleLine, now) ==
              std::vector<std::string>{"EGLL", "KJFK"},
          "station rows on the same line were not parsed");
    const auto matched = openbus::matchMetarStations(
        {{"EGLL", "Heathrow", "United Kingdom"}, {"KJFK", "Kennedy", "United States"}}, active);
    check(matched.size() == 3 && matched[0].code == "ABCD" && matched[0].name.empty() &&
              matched[1].name == "Heathrow" && matched[2].country == "United States",
          "directory stations were not matched to metadata, including unknown stations");
    check(openbus::parseRecentMetarStations(row("KJFK", "05-Nov-2026 22:00"), now).empty(),
          "an entirely stale directory should have no active stations");
    for (const std::string invalid : {
             std::string("<html>not an index</html>"),
             row("KJFK", "32-Nov-2026 22:30"),
             row("KJFK", "06-Nox-2026 22:30"),
             row("KJFK", "06-Nov-2026 24:30"),
             row("KJFK", "06-Nov-2026 22:30") + row("KJFK", "06-Nov-2026 22:30"),
             std::string("<tr><td><a href=\"KJFK.TXT\">KJFK.TXT</a></td><td>bad</td></tr>")}) {
        bool rejected = false;
        try {
            static_cast<void>(openbus::parseRecentMetarStations(invalid, now));
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        check(rejected, "malformed NOAA directory was accepted");
    }

    auto simulatedNow = openbus::ActiveMetarStations::Clock::time_point{};
    std::atomic<int> calls{0};
    openbus::ActiveMetarStations selection(
        [&]() -> std::vector<std::string> {
            if (++calls == 2)
                throw std::runtime_error("network unavailable");
            return {"EGLL", "KJFK"};
        }, [&] { return simulatedNow; });
    const auto waitFor = [&]() {
        std::optional<std::vector<std::string>> result;
        const auto deadline = steady_clock::now() + seconds{3};
        while (selection.pending() && steady_clock::now() < deadline) {
            if (auto codes = selection.poll())
                result = std::move(codes);
            std::this_thread::sleep_for(milliseconds{1});
        }
        return result;
    };
    check(!selection.refreshIfDue(), "directory refresh started before it was requested");
    selection.request();
    check(waitFor().has_value() && calls == 1 && !selection.failed(),
          "initial directory request failed");
    simulatedNow += minutes{10} - milliseconds{1};
    check(!selection.refreshIfDue(), "directory refreshed before ten minutes");
    simulatedNow += milliseconds{1};
    check(selection.refreshIfDue() && !selection.refreshIfDue(),
          "directory refresh timing or overlap was incorrect");
    check(!waitFor() && selection.failed() &&
              selection.status().find("network unavailable") != std::string::npos,
          "directory error was not surfaced");
    simulatedNow += minutes{10};
    check(selection.refreshIfDue() && waitFor().has_value() && !selection.failed() && calls == 3,
          "directory refresh did not retry after an error");
    return passed ? 0 : 1;
}
