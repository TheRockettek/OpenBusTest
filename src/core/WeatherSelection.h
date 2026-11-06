#pragma once

#include "core/WeatherState.h"

#include <chrono>
#include <functional>
#include <future>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace openbus {

struct MetarStation {
    std::string code;
    std::string name;
    std::string country;
};

std::vector<MetarStation> parseMetarStations(std::string_view csv);
std::vector<MetarStation> metarStations();

class WeatherSelection {
  public:
    using Fetch = std::function<WeatherState(const std::string&)>;
    using Clock = std::chrono::steady_clock;
    using Now = std::function<Clock::time_point()>;
    explicit WeatherSelection(Fetch fetch = {}, Now now = {});

    void requestMetar(const std::string& station);
    void enableMetarRefresh(const std::string& station, const WeatherState& initialWeather);
    bool refreshIfDue();
    void discardPending();
    bool pending() const;
    std::optional<WeatherState> poll();
    const std::string& status() const;
    bool failed() const;
    const std::optional<std::string>& activeStation() const;

  private:
    void startRequest(const std::string& station);
    Fetch fetch_;
    Now now_;
    std::future<WeatherState> request_;
    std::optional<std::string> activeStation_;
    std::optional<WeatherState> startupOverrides_;
    Clock::time_point nextRefresh_{};
    std::string requestedStation_;
    std::string status_;
    bool discarded_ = false;
    bool failed_ = false;
};

} // namespace openbus
