#pragma once

#include <string>

enum class LogLevel { Trace, Debug, Info, Warning, Error };

class Logger {
  public:
    explicit Logger(const std::string& moduleName);
    void Log(const std::string& message);
    void Log(LogLevel level, const std::string& message);
    void Trace(const std::string& message);
    void Debug(const std::string& message);
    void Info(const std::string& message);
    void Warning(const std::string& message);
    void Error(const std::string& message);
    static void Flush();

  private:
    std::string moduleName_;
};

// Shared channels used across modules. Module-private channels belong in an
// anonymous namespace in the owning translation unit.
extern Logger gameLog;
extern Logger textureLog;