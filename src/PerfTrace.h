#pragma once

namespace openbus::rendering {

bool parseEnabledFlag(const char* value);

class TraceScope {
  public:
    TraceScope(const char* category, const char* name);
    ~TraceScope();

  private:
    const char* category_;
    const char* name_;
    bool active_;
};

} // namespace openbus::rendering
