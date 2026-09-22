#pragma once

#include <fstream>
#include <string>

class Logger {
  public:
    std::string ModuleName;
    std::ofstream LogFile;
    Logger(const std::string& moduleName);
    void Log(const std::string& message);
};