#pragma once

#include <fstream>
#include <mutex>
#include <string>

class Logger {
  public:
    std::string ModuleName;
    std::ofstream LogFile;
    std::mutex LogMutex;
    Logger(const std::string& moduleName);
    void Log(const std::string& message);
};