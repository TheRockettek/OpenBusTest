#include "Variables.h"

#include <algorithm>
#include <cctype>
#include <ctime>
#include <mutex>
#include <shared_mutex>

namespace {

std::string keyFor(const std::string& name) {
    // Script variable names are case-insensitive throughout the loaders and
    // renderer, so storage uses a normalized lowercase key.
    std::string key = name;
    std::transform(key.begin(), key.end(), key.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return key;
}

} // namespace

Variables::Variables() {
    // Seed variables used by configuration files before model-specific names
    // are declared by the parsers.
    const std::time_t now = std::time(nullptr);
    std::tm localDate = {};
    if (const std::tm* currentDate = std::localtime(&now)) {
        localDate = *currentDate;
    }

    values_ = {
        {keyFor("Timegap"), 0.0},
        {keyFor("GetTime"), 0.0},
        {keyFor("NoSound"), 1.0},
        {keyFor("Pause"), 0.0},
        {keyFor("Time"), 43200.0},
        {keyFor("Day"), static_cast<double>(localDate.tm_mday)},
        {keyFor("Month"), static_cast<double>(localDate.tm_mon + 1)},
        {keyFor("Year"), static_cast<double>(localDate.tm_year + 1900)},
        {keyFor("DayOfYear"), static_cast<double>(localDate.tm_yday)},
        {keyFor("mouse_x"), 0.0},
        {keyFor("mouse_y"), 0.0},
        {keyFor("PrecipType"), 0.0},
        {keyFor("PrecipRate"), 0.0},
        {keyFor("Weather_Temperature"), 20.0},
        {keyFor("Weather_AbsHum"), 10.0},
        {keyFor("Envir_Brightness"), 1.0},
        {keyFor("AutoClutch"), 0.0},
        {keyFor("SunAlt"), 60.0},
    };
}

void Variables::declare(const std::string& name) {
    const std::string key = keyFor(name);
    if (key.empty()) {
        return;
    }
    std::unique_lock<std::shared_mutex> lock(mutex_);
    // Do not overwrite a value that may have been assigned before declaration.
    values_.try_emplace(key, 0.0);
}

double Variables::get(const std::string& name) const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    const auto found = values_.find(keyFor(name));
    // Unknown variables follow the simulator's numeric default of zero.
    return found == values_.end() ? 0.0 : found->second;
}

void Variables::set(const std::string& name, double value) {
    const std::string key = keyFor(name);
    if (key.empty()) {
        return;
    }
    std::unique_lock<std::shared_mutex> lock(mutex_);
    values_[key] = value;
}

void Variables::updateFrame(double timegap, double getTime, double mouseX, double mouseY) {
    // These values are refreshed as one snapshot at the start of each frame.
    std::unique_lock<std::shared_mutex> lock(mutex_);
    values_[keyFor("Timegap")] = timegap;
    values_[keyFor("GetTime")] = getTime;
    values_[keyFor("mouse_x")] = mouseX;
    values_[keyFor("mouse_y")] = mouseY;
}