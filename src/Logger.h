#pragma once

#include <string>

class Logger {
  public:
    Logger(const std::string& moduleName);
    void Log(const std::string& message);
    static void Flush();

  private:
    std::string moduleName_;
};