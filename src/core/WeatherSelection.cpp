#include "core/WeatherSelection.h"

#include "core/MetarNoaa.h"
#include "MetarStations.h"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace openbus {
namespace {

std::vector<std::vector<std::string>> parseCsv(std::string_view csv) {
    if (csv.starts_with("\xEF\xBB\xBF"))
        csv.remove_prefix(3);
    std::vector<std::vector<std::string>> rows;
    std::vector<std::string> row;
    std::string field;
    bool quoted = false;
    bool closed = false;
    bool started = false;
    for (std::size_t i = 0; i < csv.size(); ++i) {
        const char c = csv[i];
        if (quoted) {
            if (c == '"') {
                if (i + 1 < csv.size() && csv[i + 1] == '"') {
                    field += '"';
                    ++i;
                } else {
                    quoted = false;
                    closed = true;
                }
            } else {
                field += c;
            }
        } else if (c == ',' || c == '\n' || c == '\r') {
            row.push_back(std::move(field));
            field.clear();
            started = closed = false;
            if (c != ',') {
                if (c == '\r' && i + 1 < csv.size() && csv[i + 1] == '\n')
                    ++i;
                rows.push_back(std::move(row));
                row.clear();
            }
        } else if (c == '"' && !started && !closed) {
            quoted = true;
            started = true;
        } else {
            if (closed || c == '"')
                throw std::invalid_argument("Invalid METAR station CSV quoting");
            field += c;
            started = true;
        }
    }
    if (quoted)
        throw std::invalid_argument("Unterminated quoted METAR station CSV field");
    if (!row.empty() || started || closed) {
        row.push_back(std::move(field));
        rows.push_back(std::move(row));
    }
    return rows;
}

} // namespace

std::vector<MetarStation> parseMetarStations(std::string_view csv) {
    const auto rows = parseCsv(csv);
    if (rows.empty())
        throw std::invalid_argument("METAR station CSV is empty");
    const auto& header = rows.front();
    const auto column = [&](std::string_view name) {
        const auto found = std::find(header.begin(), header.end(), name);
        if (found == header.end()) {
            throw std::invalid_argument("METAR station CSV is missing the " + std::string(name) +
                                        " column");
        }
        return static_cast<std::size_t>(found - header.begin());
    };
    const auto code = column("code");
    const auto name = column("name");
    const auto country = column("country");
    std::unordered_set<std::string> seen;
    std::vector<MetarStation> stations;
    for (std::size_t i = 1; i < rows.size(); ++i) {
        const auto& row = rows[i];
        if (row.size() == 1 && row[0].empty())
            continue;
        if (row.size() != header.size()) {
            throw std::invalid_argument("METAR station CSV has an invalid row " +
                                        std::to_string(i + 1));
        }
        const std::string station = noaaMetarStation(row[code]);
        if (!seen.insert(station).second) {
            throw std::invalid_argument("METAR station CSV has a duplicate " + station);
        }
        stations.push_back({station, row[name], row[country]});
    }
    std::sort(stations.begin(), stations.end(),
              [](const auto& a, const auto& b) { return a.code < b.code; });
    return stations;
}

std::vector<MetarStation> metarStations() {
    std::string csv;
    for (const char* chunk : openbusMetarStationChunks)
        csv += chunk;
    auto stations = parseMetarStations(csv);
    if (stations.empty())
        throw std::runtime_error("No METAR stations are available");
    return stations;
}

WeatherSelection::WeatherSelection(Fetch fetch, Now now)
    : fetch_(std::move(fetch)), now_(std::move(now)) {
    if (!now_)
        now_ = [] { return Clock::now(); };
    if (!fetch_) {
        fetch_ = [](const std::string& station) {
            const auto body = fetchNoaaMetar(station);
            return weatherFromMetar(
                parseNoaaMetar(station, body, std::chrono::system_clock::now()).observation);
        };
    }
}

void WeatherSelection::requestMetar(const std::string& station) {
    if (request_.valid())
        throw std::logic_error("A METAR request is already in progress");
    const std::string normalized = noaaMetarStation(station);
    startRequest(normalized);
    activeStation_ = normalized;
    startupOverrides_.reset();
    nextRefresh_ = now_() + std::chrono::minutes(10);
}

void WeatherSelection::startRequest(const std::string& normalized) {
    auto request = std::async(std::launch::async, fetch_, normalized);
    request_ = std::move(request);
    requestedStation_ = normalized;
    discarded_ = false;
    failed_ = false;
    status_ = "Fetching " + normalized + " from NOAA...";
}

void WeatherSelection::enableMetarRefresh(const std::string& station,
                                          const WeatherState& initialWeather) {
    if (request_.valid() && !discarded_)
        throw std::logic_error("A METAR request is already in progress");
    const std::string normalized = noaaMetarStation(station);
    validateWeatherState(initialWeather);
    activeStation_ = normalized;
    startupOverrides_ = initialWeather;
    nextRefresh_ = now_() + std::chrono::minutes(10);
    failed_ = false;
    status_ = "Automatic NOAA METAR updates enabled for " + normalized + ".";
}

bool WeatherSelection::refreshIfDue() {
    const auto now = now_();
    if (!activeStation_ || request_.valid() || now < nextRefresh_)
        return false;
    nextRefresh_ = now + std::chrono::minutes(10);
    try {
        startRequest(*activeStation_);
    } catch (const std::exception& error) {
        failed_ = true;
        status_ = "METAR " + *activeStation_ + " could not start: " + error.what();
    }
    return true;
}

void WeatherSelection::discardPending() {
    if (request_.valid())
        discarded_ = true;
    status_.clear();
    failed_ = false;
    activeStation_.reset();
    startupOverrides_.reset();
}

bool WeatherSelection::pending() const {
    return request_.valid();
}

std::optional<WeatherState> WeatherSelection::poll() {
    if (!request_.valid() ||
        request_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        return std::nullopt;
    try {
        WeatherState weather = request_.get();
        if (discarded_)
            return std::nullopt;
        if (startupOverrides_) {
            weather.brightness = startupOverrides_->brightness;
            weather.cloudType = startupOverrides_->cloudType;
            weather.fogTopAboveGroundMeters = startupOverrides_->fogTopAboveGroundMeters;
        }
        validateWeatherState(weather);
        status_ = "Applied NOAA METAR for " + requestedStation_ + ".";
        failed_ = false;
        return weather;
    } catch (const std::exception& error) {
        if (!discarded_) {
            status_ = "METAR " + requestedStation_ + " failed: " + error.what();
            failed_ = true;
        }
        return std::nullopt;
    }
}

const std::string& WeatherSelection::status() const {
    return status_;
}

bool WeatherSelection::failed() const {
    return failed_;
}

const std::optional<std::string>& WeatherSelection::activeStation() const {
    return activeStation_;
}

} // namespace openbus
