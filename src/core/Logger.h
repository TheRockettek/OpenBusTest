#pragma once

#include <string>

class Logger {
  public:
    explicit Logger(const std::string& moduleName);
    void Log(const std::string& message);
    static void Flush();

  private:
    std::string moduleName_;
};

// Shared channels used across modules. Module-private channels belong in an
// anonymous namespace in the owning translation unit.
extern Logger gameLog;
extern Logger textureLog;