#pragma once

#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace openbus {

// Returns a thread-local snapshot that remains valid until the next call on
// the same thread. This avoids the deprecated CRT getenv API on Windows while
// preserving the existing const char* call-site contract.
// Values are read on every call; cache parsed settings at hot call sites, not
// the returned pointer, which a subsequent call can invalidate.
inline const char* getEnvironment(const char* name) {
#ifdef _WIN32
    char* rawValue = nullptr;
    std::size_t valueLength = 0;
    thread_local std::string value;
    if (_dupenv_s(&rawValue, &valueLength, name) != 0 || rawValue == nullptr) {
        std::free(rawValue);
        value.clear();
        return nullptr;
    }
    value.assign(rawValue, valueLength > 0 ? valueLength - 1 : 0);
    std::free(rawValue);
    return value.c_str();
#else
    return std::getenv(name);
#endif
}

inline std::string environmentValue(const char* name) {
    const char* value = getEnvironment(name);
    return value == nullptr ? std::string{} : std::string(value);
}

inline double finiteEnvironmentDouble(const char* name, double fallback) {
    const std::string configuredValue = environmentValue(name);
    if (configuredValue.empty()) {
        return fallback;
    }
    char* end = nullptr;
    const double value = std::strtod(configuredValue.c_str(), &end);
    if (end == configuredValue.c_str() || *end != '\0' || !std::isfinite(value)) {
        throw std::runtime_error(std::string(name) + " must be a finite number");
    }
    return value;
}

} // namespace openbus
