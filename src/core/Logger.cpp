#include "core/Logger.h"

#include "core/Environment.h"

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
        : logFile(logFilePath(), std::ios::out | std::ios::trunc), worker(&LoggerState::run, this) {
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
                }
                lock.lock();
            }
        }
        if (logFile.is_open()) {
            logFile.flush();
        }
    }

    std::ofstream logFile;
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
    const auto elapsed = duration_cast<milliseconds>(system_clock::now() - startTime).count();
    loggerState().enqueue("[" + std::to_string(elapsed) + "] [" + levelName(level) + "] " +
                          moduleName_ + " " + message);
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