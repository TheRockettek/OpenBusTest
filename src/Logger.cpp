#include "Logger.h"

#include <chrono>
#include <iostream>

using namespace std::chrono;

int startTime = static_cast<int>(system_clock::now().time_since_epoch().count());

Logger::Logger(const std::string& moduleName) : ModuleName(moduleName) {
    LogFile.open("game.log", std::ios::app);
}

void Logger::Log(const std::string& message) {
    std::lock_guard<std::mutex> lock(LogMutex);
    std::string logMessage =
        "[" +
        std::to_string(static_cast<int>(system_clock::now().time_since_epoch().count()) -
                       startTime) +
        "] " + this->ModuleName + " " + message;
    std::cout << logMessage << std::endl;

    if (LogFile.is_open()) {
        LogFile << logMessage << std::endl;
    }
}