#pragma once

#ifndef OPENBUS_ENABLE_PERF_TRACE
#define OPENBUS_ENABLE_PERF_TRACE 0
#endif

#include <cstdint>

namespace openbus::rendering {

bool parseEnabledFlag(const char* value);
#if OPENBUS_ENABLE_PERF_TRACE
void Flush();
bool IsRecording();
bool SetRecording(bool recording);

class TraceScope {
  public:
    TraceScope(const char* category, const char* name);
    ~TraceScope();

  private:
    const char* category_;
    const char* name_;
    std::int64_t startTimestampUs_ = 0;
    std::uint64_t recordingId_ = 0;
};
#else
inline void Flush() noexcept {}
inline bool IsRecording() noexcept {
    return false;
}
inline bool SetRecording(bool) noexcept {
    return false;
}

class TraceScope {
  public:
    constexpr TraceScope(const char*, const char*) noexcept {}
    ~TraceScope() noexcept {}
};
#endif

} // namespace openbus::rendering
