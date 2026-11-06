#include "core/ActiveMetarStations.h"

#include "core/MetarNoaa.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <regex>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace openbus {

std::vector<std::string> parseRecentMetarStations(std::string_view html,
                                                  std::chrono::system_clock::time_point now) {
    using namespace std::chrono;
    static const std::regex stationLink(R"re(href="([A-Z][A-Z0-9]{3})\.TXT")re");
    static const std::regex dateCell(
        R"re(</a></td><td[^>]*>\s*([0-9]{2})-([A-Za-z]{3})-([0-9]{4})\s+([0-9]{2}):([0-9]{2})\s*</td>)re");
    constexpr std::array<std::string_view, 12> months = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                                         "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    std::unordered_set<std::string> seen;
    std::vector<std::string> active;
    std::size_t offset = 0;
    while (true) {
        const auto start = html.find("<tr", offset);
        if (start == std::string_view::npos)
            break;
        const auto end = html.find("</tr>", start);
        if (end == std::string_view::npos)
            throw std::invalid_argument("NOAA METAR directory has an unterminated table row");
        const std::string line(html.substr(start, end - start));
        offset = end + 5;
        std::smatch link;
        if (!std::regex_search(line, link, stationLink))
            continue;
        const std::string code = link[1].str();
        if (!seen.insert(code).second)
            throw std::invalid_argument("NOAA METAR directory has duplicate station " + code);
        std::smatch date;
        if (!std::regex_search(line, date, dateCell))
            throw std::invalid_argument("NOAA METAR directory has an invalid timestamp for " +
                                        code);
        const std::string monthText = date[2].str();
        const auto monthIt = std::find(months.begin(), months.end(), monthText);
        if (monthIt == months.end())
            throw std::invalid_argument("NOAA METAR directory has an invalid month for " + code);
        const year_month_day day{year{std::stoi(date[3].str())},
                                 month{static_cast<unsigned>(monthIt - months.begin() + 1)},
                                 std::chrono::day{static_cast<unsigned>(std::stoi(date[1].str()))}};
        const int hour = std::stoi(date[4].str());
        const int minute = std::stoi(date[5].str());
        if (!day.ok() || hour > 23 || minute > 59)
            throw std::invalid_argument("NOAA METAR directory has an invalid date for " + code);
        const auto modified = sys_days{day} + hours{hour} + minutes{minute};
        if (modified >= now - hours{1} && modified <= now + minutes{5})
            active.push_back(code);
    }
    if (seen.empty())
        throw std::invalid_argument("NOAA METAR directory has no recognizable station entries");
    std::sort(active.begin(), active.end());
    return active;
}

std::vector<MetarStation> matchMetarStations(const std::vector<MetarStation>& catalogue,
                                             const std::vector<std::string>& codes) {
    std::unordered_map<std::string, const MetarStation*> lookup;
    for (const auto& station : catalogue)
        lookup.emplace(station.code, &station);
    std::vector<MetarStation> result;
    result.reserve(codes.size());
    for (const auto& code : codes) {
        const auto found = lookup.find(code);
        result.push_back(found == lookup.end() ? MetarStation{code, {}, {}} : *found->second);
    }
    return result;
}

ActiveMetarStations::ActiveMetarStations(Fetch fetch, Now now)
    : fetch_(std::move(fetch)), now_(std::move(now)) {
    if (!fetch_)
        fetch_ = [] {
            const auto directory = fetchNoaaMetarDirectory();
            return parseRecentMetarStations(directory, std::chrono::system_clock::now());
        };
    if (!now_)
        now_ = [] { return Clock::now(); };
}

void ActiveMetarStations::request() {
    if (request_.valid())
        throw std::logic_error("A METAR station directory request is already in progress");
    auto request = std::async(std::launch::async, fetch_);
    request_ = std::move(request);
    started_ = true;
    nextRefresh_ = now_() + std::chrono::minutes(10);
    failed_ = false;
    status_ = "Loading recently updated NOAA stations...";
}

bool ActiveMetarStations::refreshIfDue() {
    if (!started_ || request_.valid() || now_() < nextRefresh_)
        return false;
    try {
        request();
    } catch (const std::exception& error) {
        nextRefresh_ = now_() + std::chrono::minutes(10);
        failed_ = true;
        status_ = "Could not start NOAA station directory refresh: " + std::string(error.what());
    }
    return true;
}

bool ActiveMetarStations::pending() const {
    return request_.valid();
}

std::optional<std::vector<std::string>> ActiveMetarStations::poll() {
    if (!request_.valid() ||
        request_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        return std::nullopt;
    try {
        auto stations = request_.get();
        failed_ = false;
        status_ = "Showing " + std::to_string(stations.size()) +
                  " NOAA stations updated within the last hour.";
        return stations;
    } catch (const std::exception& error) {
        failed_ = true;
        status_ = "Could not load NOAA station directory: " + std::string(error.what());
        return std::nullopt;
    }
}

bool ActiveMetarStations::failed() const {
    return failed_;
}

const std::string& ActiveMetarStations::status() const {
    return status_;
}

} // namespace openbus
