#pragma once

#include <cstdlib>
#include <string>

namespace openbus {

// Returns a thread-local snapshot that remains valid until the next call on
// the same thread. This avoids the deprecated CRT getenv API on Windows while
// preserving the existing const char* call-site contract.
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

} // namespace openbus
