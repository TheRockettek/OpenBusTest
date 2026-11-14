#include "core/GraphicsPreferences.h"
#include "core/AtomicFile.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <system_error>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif defined(__APPLE__)
#include <mach-o/dyld.h>
#endif

namespace openbus::core {
namespace {

constexpr std::array<std::string_view, 19> keys = {"display.mode",
                                                   "display.width",
                                                   "display.height",
                                                   "display.refresh_rate",
                                                   "display.monitor",
                                                   "display.vsync",
                                                   "display.fps_cap",
                                                   "reflections.size",
                                                   "reflections.interval",
                                                   "reflections.max_fps",
                                                   "visibility.main_minimum_object_percent",
                                                   "visibility.mirrors_minimum_object_percent",
                                                   "textures.anisotropic_filtering",
                                                   "antialiasing.mode",
                                                   "rendering.scale",
                                                   "textures.vehicle_texture_scale",
                                                   "advanced.vehicle_frustum_culling",
                                                   "textures.generate_mipmaps",
                                                   "textures.map_texture_scale"};

std::string_view trim(std::string_view value) {
    const auto first = value.find_first_not_of(" \t\r");
    if (first == std::string_view::npos) {
        return {};
    }
    return value.substr(first, value.find_last_not_of(" \t\r") - first + 1);
}

double parseNumber(std::string_view value) {
    double result = 0.0;
    const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
    if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() ||
        !std::isfinite(result)) {
        throw std::invalid_argument("expected a finite number");
    }
    return result;
}

void assign(GraphicsPreferences& preferences, std::string_view key, std::string_view value) {
    if (key == keys[0]) {
        if (value != "windowed" && value != "borderless" && value != "exclusive") {
            throw std::invalid_argument("mode must be windowed, borderless or exclusive");
        }
        preferences.windowMode = std::string(value);
        return;
    }
    if (key == keys[5] || key == keys[16] || key == keys[17]) {
        bool* setting = key == keys[5]    ? &preferences.vsync
                        : key == keys[16] ? &preferences.vehicleFrustumCulling
                                          : &preferences.generateTextureMipmaps;
        if (value == "true" || value == "1") {
            *setting = true;
        } else if (value == "false" || value == "0") {
            *setting = false;
        } else {
            throw std::invalid_argument("expected true or false");
        }
        return;
    }
    if (key == keys[13]) {
        if (value != "off" && value != "fxaa") {
            throw std::invalid_argument("mode must be off or fxaa; MSAA is not supported");
        }
        preferences.antiAliasingMode = std::string(value);
        return;
    }
    const double number = parseNumber(value);
    if (key == keys[1] || key == keys[2] || key == keys[3] || key == keys[4]) {
        if (std::floor(number) != number) {
            throw std::invalid_argument("display dimensions and refresh rate must be integers");
        }
        if (key == keys[1] || key == keys[2]) {
            if (number < 640.0 || number > 16384.0) {
                throw std::invalid_argument("display dimensions must be between 640 and 16384");
            }
            (key == keys[1] ? preferences.width : preferences.height) = static_cast<int>(number);
        } else if (key == keys[3]) {
            if (number < 24.0 || number > 1000.0) {
                throw std::invalid_argument("refresh rate must be between 24 and 1000 Hz");
            }
            preferences.refreshRate = static_cast<int>(number);
        } else {
            if (number < 0.0 || number > 16.0) {
                throw std::invalid_argument("monitor index must be between 0 and 16");
            }
            preferences.monitor = static_cast<int>(number);
        }
        return;
    }
    if (key == keys[6] || key == keys[9]) {
        if (number < 0.0 || number > 1000.0 ||
            (number > 0.0 && number < (key == keys[6] ? 0.01 : 1.0))) {
            throw std::invalid_argument("frame rate is outside its supported range");
        }
        (key == keys[6] ? preferences.fpsCap : preferences.reflectionMaxFps) = number;
    } else if (key == keys[12]) {
        if (number != 1.0 && number != 2.0 && number != 4.0 && number != 8.0 && number != 16.0) {
            throw std::invalid_argument("anisotropic filtering must be 1, 2, 4, 8 or 16");
        }
        preferences.anisotropicFiltering = static_cast<int>(number);
    } else if (key == keys[7]) {
        if (number != 256.0 && number != 512.0 && number != 1024.0) {
            throw std::invalid_argument("reflection size must be 256, 512 or 1024");
        }
        preferences.reflectionSize = static_cast<int>(number);
    } else if (key == keys[8]) {
        if (number < 1.0 || number > 10000.0 || std::floor(number) != number) {
            throw std::invalid_argument("reflection interval must be an integer from 1 to 10000");
        }
        preferences.reflectionInterval = static_cast<int>(number);
    } else if (key == keys[14]) {
        if (number < 0.5 || number > 1.0) {
            throw std::invalid_argument("render scale must be between 0.5 and 1.0");
        }
        preferences.renderScale = number;
    } else if (key == keys[15]) {
        if (number < 0.25 || number > 1.0) {
            throw std::invalid_argument("vehicle texture scale must be between 0.25 and 1.0");
        }
        preferences.vehicleTextureScale = number;
    } else if (key == keys[18]) {
        if (number < 0.25 || number > 1.0) {
            throw std::invalid_argument("map texture scale must be between 0.25 and 1.0");
        }
        preferences.mapTextureScale = number;
    } else {
        if (number < 0.0 || number > 100.0) {
            throw std::invalid_argument("object size percentage must be between 0 and 100");
        }
        (key == keys[10] ? preferences.minimumObjectSizeMainPercent
                         : preferences.minimumObjectSizeMirrorsPercent) = number;
    }
}

std::string valueFor(const GraphicsPreferences& preferences, std::string_view key) {
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream << std::setprecision(std::numeric_limits<double>::max_digits10);
    if (key == keys[0])
        stream << preferences.windowMode;
    if (key == keys[1])
        stream << preferences.width;
    if (key == keys[2])
        stream << preferences.height;
    if (key == keys[3])
        stream << preferences.refreshRate;
    if (key == keys[4])
        stream << preferences.monitor;
    if (key == keys[5])
        stream << (preferences.vsync ? "true" : "false");
    if (key == keys[6])
        stream << preferences.fpsCap;
    if (key == keys[7])
        stream << preferences.reflectionSize;
    if (key == keys[8])
        stream << preferences.reflectionInterval;
    if (key == keys[9])
        stream << preferences.reflectionMaxFps;
    if (key == keys[10])
        stream << preferences.minimumObjectSizeMainPercent;
    if (key == keys[11])
        stream << preferences.minimumObjectSizeMirrorsPercent;
    if (key == keys[12])
        stream << preferences.anisotropicFiltering;
    if (key == keys[13])
        stream << preferences.antiAliasingMode;
    if (key == keys[14])
        stream << preferences.renderScale;
    if (key == keys[15])
        stream << preferences.vehicleTextureScale;
    if (key == keys[16])
        stream << (preferences.vehicleFrustumCulling ? "true" : "false");
    if (key == keys[17])
        stream << (preferences.generateTextureMipmaps ? "true" : "false");
    if (key == keys[18])
        stream << preferences.mapTextureScale;
    return stream.str();
}

bool knownKey(std::string_view key) {
    for (const auto candidate : keys) {
        if (key == candidate)
            return true;
    }
    return false;
}

std::string fullKey(std::string_view section, std::string_view name) {
    return std::string(section) + "." + std::string(name);
}

} // namespace

double parseGraphicsFpsCapOverride(std::string_view value) {
    std::size_t parsed = 0;
    const std::string input(value);
    const double rate = std::stod(input, &parsed);
    if (parsed != input.size() || !std::isfinite(rate) || rate < 0.0) {
        throw std::invalid_argument("FPS cap must be finite and non-negative");
    }
    return rate == 0.0 ? 0.0 : std::clamp(rate, 0.01, 1000.0);
}

std::filesystem::path graphicsPreferencesPath() {
#ifdef _WIN32
    std::vector<wchar_t> buffer(512);
    while (buffer.size() <= 32768) {
        const DWORD length =
            GetModuleFileNameW(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
        if (length == 0) {
            throw std::system_error(static_cast<int>(GetLastError()), std::system_category(),
                                    "Unable to locate the executable");
        }
        if (length < buffer.size()) {
            return std::filesystem::path(std::wstring(buffer.data(), length)).parent_path() /
                   "OpenBus_graphics.ini";
        }
        buffer.resize(buffer.size() * 2);
    }
    throw std::runtime_error("Executable path exceeds the supported length");
#elif defined(__APPLE__)
    std::uint32_t size = 0;
    _NSGetExecutablePath(nullptr, &size);
    std::vector<char> buffer(size);
    if (_NSGetExecutablePath(buffer.data(), &size) != 0) {
        throw std::runtime_error("Unable to locate the executable");
    }
    return std::filesystem::canonical(buffer.data()).parent_path() / "OpenBus_graphics.ini";
#else
    std::error_code error;
    const auto executable = std::filesystem::read_symlink("/proc/self/exe", error);
    if (error) {
        throw std::system_error(error, "Unable to locate the executable");
    }
    return executable.parent_path() / "OpenBus_graphics.ini";
#endif
}

GraphicsPreferencesFile loadGraphicsPreferences(const std::filesystem::path& path) {
    GraphicsPreferencesFile result;
    std::error_code error;
    const bool exists = std::filesystem::exists(path, error);
    if (error)
        throw std::system_error(error, "Unable to inspect graphics preferences");
    if (!exists)
        return result;
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("Unable to open graphics preferences: " + path.string());
    std::string section;
    std::string line;
    std::set<std::string> seen;
    while (std::getline(input, line)) {
        result.lines.push_back(line);
        std::string_view lineView = line;
        if (result.lines.size() == 1 && lineView.starts_with("\xEF\xBB\xBF")) {
            lineView.remove_prefix(3);
        }
        const auto text = trim(lineView);
        if (text.empty() || text.front() == ';' || text.front() == '#')
            continue;
        if (text.front() == '[') {
            if (text.back() != ']' || text.size() < 3) {
                throw std::runtime_error("Invalid graphics INI section at line " +
                                         std::to_string(result.lines.size()));
            }
            section = std::string(trim(text.substr(1, text.size() - 2)));
            continue;
        }
        const auto equals = text.find('=');
        if (section.empty() || equals == std::string_view::npos ||
            trim(text.substr(0, equals)).empty()) {
            throw std::runtime_error("Invalid graphics INI entry at line " +
                                     std::to_string(result.lines.size()));
        }
        const std::string key = fullKey(section, trim(text.substr(0, equals)));
        if (!knownKey(key))
            continue;
        if (!seen.insert(key).second) {
            throw std::runtime_error("Duplicate graphics INI key: " + key);
        }
        try {
            assign(result.values, key, trim(text.substr(equals + 1)));
        } catch (const std::invalid_argument& errorDetail) {
            throw std::runtime_error("Invalid graphics INI value for " + key + ": " +
                                     errorDetail.what());
        }
    }
    if (input.bad())
        throw std::runtime_error("Failed while reading graphics preferences");
    return result;
}

GraphicsPreferences mergeGraphicsPreferences(const GraphicsPreferences& baseline,
                                             const GraphicsPreferences& edited,
                                             GraphicsPreferences disk) {
    if (edited.windowMode != baseline.windowMode)
        disk.windowMode = edited.windowMode;
    if (edited.width != baseline.width)
        disk.width = edited.width;
    if (edited.height != baseline.height)
        disk.height = edited.height;
    if (edited.refreshRate != baseline.refreshRate)
        disk.refreshRate = edited.refreshRate;
    if (edited.monitor != baseline.monitor)
        disk.monitor = edited.monitor;
    if (edited.anisotropicFiltering != baseline.anisotropicFiltering)
        disk.anisotropicFiltering = edited.anisotropicFiltering;
    if (edited.antiAliasingMode != baseline.antiAliasingMode)
        disk.antiAliasingMode = edited.antiAliasingMode;
    if (edited.renderScale != baseline.renderScale)
        disk.renderScale = edited.renderScale;
    if (edited.vehicleTextureScale != baseline.vehicleTextureScale)
        disk.vehicleTextureScale = edited.vehicleTextureScale;
    if (edited.vehicleFrustumCulling != baseline.vehicleFrustumCulling)
        disk.vehicleFrustumCulling = edited.vehicleFrustumCulling;
    if (edited.vsync != baseline.vsync)
        disk.vsync = edited.vsync;
    if (edited.fpsCap != baseline.fpsCap)
        disk.fpsCap = edited.fpsCap;
    if (edited.reflectionSize != baseline.reflectionSize)
        disk.reflectionSize = edited.reflectionSize;
    if (edited.reflectionInterval != baseline.reflectionInterval)
        disk.reflectionInterval = edited.reflectionInterval;
    if (edited.reflectionMaxFps != baseline.reflectionMaxFps)
        disk.reflectionMaxFps = edited.reflectionMaxFps;
    if (edited.minimumObjectSizeMainPercent != baseline.minimumObjectSizeMainPercent)
        disk.minimumObjectSizeMainPercent = edited.minimumObjectSizeMainPercent;
    if (edited.minimumObjectSizeMirrorsPercent != baseline.minimumObjectSizeMirrorsPercent)
        disk.minimumObjectSizeMirrorsPercent = edited.minimumObjectSizeMirrorsPercent;
    return disk;
}

void saveGraphicsPreferences(const std::filesystem::path& path,
                             const GraphicsPreferencesFile& preferences) {
    GraphicsPreferences validated;
    for (const auto key : keys)
        assign(validated, key, valueFor(preferences.values, key));
    std::set<std::string> written;
    std::string section;
    writeFileAtomically(path, [&](std::ostream& output) {
        bool firstLine = true;
        for (const auto& line : preferences.lines) {
            std::string_view lineView = line;
            const bool hasBom = firstLine && lineView.starts_with("\xEF\xBB\xBF");
            firstLine = false;
            if (hasBom)
                lineView.remove_prefix(3);
            const auto text = trim(lineView);
            if (!text.empty() && text.front() == '[' && text.back() == ']') {
                section = std::string(trim(text.substr(1, text.size() - 2)));
            } else if (!text.empty() && text.front() != ';' && text.front() != '#') {
                const auto equals = text.find('=');
                if (equals != std::string_view::npos) {
                    const std::string key = fullKey(section, trim(text.substr(0, equals)));
                    if (knownKey(key)) {
                        if (hasBom)
                            output << "\xEF\xBB\xBF";
                        output << trim(text.substr(0, equals)) << " = "
                               << valueFor(preferences.values, key) << '\n';
                        written.insert(key);
                        continue;
                    }
                }
            }
            output << line << '\n';
        }
        for (const auto key : keys) {
            if (written.contains(std::string(key)))
                continue;
            const auto dot = key.find('.');
            output << "\n[" << key.substr(0, dot) << "]\n"
                   << key.substr(dot + 1) << " = " << valueFor(preferences.values, key) << '\n';
        }
    });
}

} // namespace openbus::core
