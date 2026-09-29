#pragma once

#include <cstdint>

namespace openbus::rendering {

bool parseEnabledFlag(const char* value);
void Flush();

class TraceScope {
  public:
    TraceScope(const char* category, const char* name);
    ~TraceScope();

  private:
    const char* category_;
    const char* name_;
    std::int64_t startTimestampUs_ = 0;
    bool active_;
};

} // namespace openbus::rendering
