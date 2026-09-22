#include "Logger.h"

#include <iostream>

Logger::Logger(const std::string& moduleName) : ModuleName(moduleName) {
    LogFile.open("game.log", std::ios::app);
}

void Logger::Log(const std::string& message) {
    time_t now = time(nullptr);
    char buf[20];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", localtime(&now));
    std::string logMessage = "[" + std::string(buf) + "] " + this->ModuleName + " " + message;
    std::cout << logMessage << std::endl;

    if (LogFile.is_open()) {
        LogFile << logMessage << std::endl;
    }
}