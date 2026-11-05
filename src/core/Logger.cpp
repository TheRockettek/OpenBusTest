#include "core/Logger.h"

#include "core/Environment.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>

using namespace std::chrono;

namespace {

const auto startTime = system_clock::now();

LogLevel configuredMinimumLogLevel() {
    const char* configuredLevel = openbus::getEnvironment("OPENBUS_LOG_LEVEL");
    if (configuredLevel == nullptr || *configuredLevel == '\0') {
        return LogLevel::Debug;
    }
    std::string level(configuredLevel);
    std::transform(level.begin(), level.end(), level.begin(),
                   [](unsigned char character) {
                       return static_cast<char>(std::toupper(character));
                   });
    if (level == "TRACE") {
        return LogLevel::Trace;
    }
    if (level == "DEBUG") {
        return LogLevel::Debug;
    }
    if (level == "INFO") {
        return LogLevel::Info;
    }
    if (level == "WARNING") {
        return LogLevel::Warning;
    }
    if (level == "ERROR") {
        return LogLevel::Error;
    }
    std::cerr << "[WARNING] Logger invalid OPENBUS_LOG_LEVEL '" << configuredLevel
              << "'; expected TRACE, DEBUG, INFO, WARNING, or ERROR. Using DEBUG.\n";
    return LogLevel::Debug;
}

struct LogEntry {
    std::string message;
};

std::string logFilePath() {
    if (const char* configuredPath = openbus::getEnvironment("OPENBUS_LOG_FILE");
        configuredPath != nullptr && *configuredPath != '\0') {
        return configuredPath;
    }
    return "game.log";
}

class LoggerState {
  public:
    LoggerState()
        : logPath(logFilePath()), logFile(logPath, std::ios::out | std::ios::trunc),
          minimumLevel(configuredMinimumLogLevel()), worker(&LoggerState::run, this) {
        if (!logFile.is_open()) {
            std::cerr << "[ERROR] Logger could not open log file: " << logPath << '\n';
        }
    }

    ~LoggerState() {
        flush();
    }

    void flush() {
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (stopping) {
                return;
            }
            stopping = true;
        }
        condition.notify_one();
        if (worker.joinable()) {
            worker.join();
        }
    }

    void enqueue(std::string message) {
        {
            std::lock_guard<std::mutex> lock(mutex);
            entries.push_back({std::move(message)});
        }
        condition.notify_one();
    }

    bool shouldLog(LogLevel level) const {
        return static_cast<int>(level) >= static_cast<int>(minimumLevel);
    }

  private:
    void run() {
        std::unique_lock<std::mutex> lock(mutex);
        while (!stopping || !entries.empty()) {
            condition.wait(lock, [this] { return stopping || !entries.empty(); });
            while (!entries.empty()) {
                LogEntry entry = std::move(entries.front());
                entries.pop_front();
                lock.unlock();
                std::cout << entry.message << '\n';
                if (logFile.is_open()) {
                    logFile << entry.message << '\n';
                    logFile.flush();
                    if (!logFile) {
                        std::cerr << "[ERROR] Logger could not write log file: " << logPath << '\n';
                        logFile.close();
                    }
                }
                lock.lock();
            }
        }
        if (logFile.is_open()) {
            logFile.flush();
            logFile.close();
        }
    }

    std::string logPath;
    std::ofstream logFile;
    LogLevel minimumLevel;
    std::mutex mutex;
    std::condition_variable condition;
    std::deque<LogEntry> entries;
    bool stopping = false;
    std::thread worker;
};

LoggerState& loggerState() {
    static LoggerState state;
    return state;
}

const char* levelName(LogLevel level) {
    switch (level) {
    case LogLevel::Trace:
        return "TRACE";
    case LogLevel::Debug:
        return "DEBUG";
    case LogLevel::Info:
        return "INFO";
    case LogLevel::Warning:
        return "WARNING";
    case LogLevel::Error:
        return "ERROR";
    }
    return "UNKNOWN";
}

} // namespace

Logger gameLog("Game");
Logger textureLog("Texture");

Logger::Logger(const std::string& moduleName) : moduleName_(moduleName) {}

void Logger::Flush() {
    loggerState().flush();
}

void Logger::Log(const std::string& message) {
    Log(LogLevel::Info, message);
}

void Logger::Log(LogLevel level, const std::string& message) {
    LoggerState& state = loggerState();
    if (!state.shouldLog(level)) {
        return;
    }
    const auto elapsed = duration_cast<milliseconds>(system_clock::now() - startTime).count();
    state.enqueue("[" + std::to_string(elapsed) + "] [" + levelName(level) + "] " + moduleName_ +
                  " " + message);
}

void Logger::Trace(const std::string& message) {
    Log(LogLevel::Trace, message);
}

void Logger::Debug(const std::string& message) {
    Log(LogLevel::Debug, message);
}

void Logger::Info(const std::string& message) {
    Log(LogLevel::Info, message);
}

void Logger::Warning(const std::string& message) {
    Log(LogLevel::Warning, message);
}

void Logger::Error(const std::string& message) {
    Log(LogLevel::Error, message);
}