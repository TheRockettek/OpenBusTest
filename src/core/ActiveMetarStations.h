#pragma once

#include "core/WeatherSelection.h"

#include <chrono>
#include <functional>
#include <future>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace openbus {

std::vector<std::string> parseRecentMetarStations(std::string_view html,
                                                  std::chrono::system_clock::time_point now);
std::vector<MetarStation> matchMetarStations(const std::vector<MetarStation>& catalogue,
                                             const std::vector<std::string>& codes);

class ActiveMetarStations {
  public:
    using Fetch = std::function<std::vector<std::string>()>;
    using Clock = std::chrono::steady_clock;
    using Now = std::function<Clock::time_point()>;
    explicit ActiveMetarStations(Fetch fetch = {}, Now now = {});

    void request();
    bool refreshIfDue();
    bool pending() const;
    std::optional<std::vector<std::string>> poll();
    bool failed() const;
    const std::string& status() const;

  private:
    Fetch fetch_;
    Now now_;
    std::future<std::vector<std::string>> request_;
    Clock::time_point nextRefresh_{};
    bool started_ = false;
    bool failed_ = false;
    std::string status_;
};

} // namespace openbus
