#pragma once

#include <string>

class Logger {
  public:
    Logger(const std::string& moduleName);
    void Log(const std::string& message);

  private:
    std::string moduleName_;
};