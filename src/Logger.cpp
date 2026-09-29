#include "Logger.h"

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

class LoggerState {
  public:
    LoggerState()
        : logFile("game.log", std::ios::out | std::ios::trunc), worker(&LoggerState::run, this) {}

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

} // namespace

Logger::Logger(const std::string& moduleName) : moduleName_(moduleName) {}

void Logger::Flush() {
    loggerState().flush();
}

void Logger::Log(const std::string& message) {
    const auto elapsed = duration_cast<milliseconds>(system_clock::now() - startTime).count();
    loggerState().enqueue("[" + std::to_string(elapsed) + "] " + moduleName_ + " " + message);
}