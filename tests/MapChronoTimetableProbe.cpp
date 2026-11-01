#include "map/MapConfigLoader.h"
#include "ProbeTempDirectory.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

void write(const std::filesystem::path& path, const std::string& text) {
    std::ofstream output(path, std::ios::binary);
    if (!output || !(output << text))
        throw std::runtime_error("could not write Chrono timetable fixture");
}

void runProbe() {
    const auto root = openbus::probe::makeProbeDirectory("openbus-chrono-timetable-probe");
    try {
        const auto base = root / "TTData";
        const auto earlier = root / "Chrono" / "z_earlier";
        const auto later = root / "Chrono" / "a_later";
        std::filesystem::create_directories(base);
        std::filesystem::create_directories(earlier / "TTData");
        std::filesystem::create_directories(later / "TTData");
        const auto trip = [](const std::string& destination) {
            return "[trip]\nBase\n" + destination +
                   "\n12\n[station]\n1\n0\nFirst\n0\n0\n0\n0\n0\n"
                   "[station]\n2\n0\nLast\n0\n0\n0\n0\n0\n";
        };
        write(base / "Base.ttp", trip("Base destination"));
        write(base / "Base.ttr", "[track_entry]\n1\n0\n0\n0\n10\n0\n"
                                "[track_entry]\n2\n0\n0\n0\n10\n0\n");
        write(base / "Base.ttl", "[addtrip]\nBase\n0\n100\n");
        write(base / "Keep.ttl", "[addtrip]\nBase\n0\n200\n");
        write(base / "Busstops.cfg", "[busstop]\nOld First\n0\n1\n0\n0\n0\n"
                                      "[busstop]\nOld Last\n0\n2\n0\n0\n0\n");
        write(base / "StnLinks.cfg", "[StnLink]\n10\n1\n2\n0\n0\n0\n0\n0\n0\n");
        write(earlier / "Chrono.cfg",
              "[startdate]\n20200101\n[enddate]\n20201231\n[deactivate_lines]\nBase\n");
        write(earlier / "TTData" / "Base.ttp", trip("Earlier destination"));
        write(earlier / "TTData" / "base.TTR",
              "[track_entry]\n1\n0\n0\n0\n25\n0\n"
              "[track_entry]\n2\n0\n0\n0\n30\n0\n");
        write(earlier / "TTData" / "New.ttl",
              "[addtrip]\nBase\n1\n300\n[addtrip]\nMissing\n0\n400\n");
        write(earlier / "TTData" / "Busstops.cfg",
              "[busstop]\nNew First\n0\n1\n0\n0\n0\n");
        write(earlier / "TTData" / "StnLinks.cfg",
              "[StnLink]\n20\n1\n2\n0\n0\n0\n0\n0\n0\n");
        write(later / "Chrono.cfg",
              "[startdate]\n20200601\n[enddate]\n20200630\n[deactivate_lines]\nNew\n");
        write(later / "TTData" / "Base.ttp", trip("Later destination"));
        write(later / "TTData" / "Final.ttl", "[addtrip]\nBase\n2\n500\n");
        write(later / "TTData" / "keep.TTL", "[addtrip]\nBase\n0\n210\n");
        write(later / "TTData" / "Busstops.cfg",
              "[busstop]\nNew Last\n0\n2\n0\n0\n0\n");
        write(later / "TTData" / "StnLinks.cfg", "");

        openbus::map::MapDefinition map;
        map.rootPath = root;
        map.aiTraffic.humanModelPaths = {"preserved"};
        const auto check = [&](int date, std::size_t active, const std::string& destination,
                               const std::vector<double>& departures) {
            require(openbus::map::applyMapChronoTimetables(map, date) == active,
                    "active timetable scenario count");
            require(map.aiTraffic.timetableRoutes.size() == 1 &&
                        map.aiTraffic.timetableRoutes[0].destinationName == destination &&
                        map.aiTraffic.timetableRoutes[0].scheduledDepartureMinutes == departures,
                    "timetable overlays, deactivation, and departure ordering");
            require(map.aiTraffic.humanModelPaths == std::vector<std::string>{"preserved"},
                    "applying timetable scenarios preserves unrelated traffic configuration");
        };
        check(20191231, 0, "Base destination", {100, 200});
        check(20200101, 1, "Earlier destination", {200, 300});
        require(map.aiTraffic.timetableRoutes[0].trackEntries[0].lengthMeters == 25,
                "Chrono track files override base files case-insensitively");
        require(map.aiTraffic.busStops.size() == 2 &&
                    map.aiTraffic.busStops[0].name == "Old Last" &&
                    map.aiTraffic.busStops[1].name == "New First" &&
                    map.aiTraffic.stationLinks.size() == 1 &&
                    map.aiTraffic.stationLinks[0].fields[0] == "20",
                "partial stop and station-link files override matching records and retain others");
        require(std::ranges::any_of(map.diagnostics.entries, [](const auto& diagnostic) {
                    return diagnostic.keyword == "addtrip" &&
                           diagnostic.message.find("missing trip Missing") != std::string::npos;
                }),
                "missing Chrono trip references produce a diagnostic");
        check(20200601, 2, "Later destination", {210, 500});
        require(map.aiTraffic.busStops.size() == 2 &&
                    map.aiTraffic.busStops[0].name == "New First" &&
                    map.aiTraffic.busStops[1].name == "New Last" &&
                    map.aiTraffic.stationLinks.size() == 1,
                "later partial and empty stop files preserve previous records");
        check(20200630, 2, "Later destination", {210, 500});
        check(20200701, 1, "Earlier destination", {200, 300});
        check(20201231, 1, "Earlier destination", {200, 300});
        check(20210101, 0, "Base destination", {100, 200});
        require(map.aiTraffic.timetableRoutes[0].trackEntries[0].lengthMeters == 10,
                "base track files return after the Chrono period");
        bool invalidDateRejected = false;
        try {
            openbus::map::applyMapChronoTimetables(map, 20200230);
        } catch (const std::invalid_argument&) {
            invalidDateRejected = true;
        }
        require(invalidDateRejected, "invalid calendar dates are rejected");
    } catch (...) {
        openbus::probe::removeProbeDirectory(root);
        throw;
    }
    openbus::probe::removeProbeDirectory(root);
}

} // namespace

int main() {
    try {
        runProbe();
    } catch (const std::exception& error) {
        std::cerr << "map Chrono timetable probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
