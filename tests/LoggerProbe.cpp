#include "core/Logger.h"
#include "ProbeTempDirectory.h"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

int main() {
    const auto directory = openbus::probe::makeProbeDirectory("openbus_logger_probe");
    const auto path = directory / "test.log";
#ifdef _WIN32
    const int environmentResult = _putenv_s("OPENBUS_LOG_FILE", path.string().c_str());
#else
    const int environmentResult = setenv("OPENBUS_LOG_FILE", path.string().c_str(), 1);
#endif
    if (environmentResult != 0) {
        std::cerr << "Could not configure log path\n";
        openbus::probe::removeProbeDirectory(directory);
        return 1;
    }

    std::ostringstream console;
    std::streambuf* previous = std::cout.rdbuf(console.rdbuf());
    Logger logger("Probe");
    logger.Debug("debug message");
    logger.Info("info message");
    logger.Warning("warning message");
    logger.Error("error message");
    logger.Log("legacy message");
    logger.Log(LogLevel::Warning, "explicit warning");

    std::vector<std::thread> writers;
    for (int writer = 0; writer < 4; ++writer) {
        writers.emplace_back([&logger, writer] {
            for (int entry = 0; entry < 20; ++entry) {
                logger.Debug("worker " + std::to_string(writer) + ":" + std::to_string(entry));
            }
        });
    }
    for (auto& writer : writers) {
        writer.join();
    }
    Logger::Flush();
    std::cout.rdbuf(previous);

    std::ifstream file(path);
    const std::string contents{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
    bool passed = file.is_open() && contents == console.str();
    const std::regex prefix(R"(^\[[0-9]+\] \[(DEBUG|INFO|WARNING|ERROR)\] Probe .+$)");
    std::istringstream lines(contents);
    std::string line;
    int count = 0;
    while (std::getline(lines, line)) {
        passed = passed && std::regex_match(line, prefix);
        ++count;
    }
    for (const std::string expected : {
             "[DEBUG] Probe debug message", "[INFO] Probe info message",
             "[WARNING] Probe warning message", "[ERROR] Probe error message",
             "[INFO] Probe legacy message", "[WARNING] Probe explicit warning"}) {
        passed = passed && contents.find(expected + '\n') != std::string::npos;
    }
    for (int writer = 0; writer < 4; ++writer) {
        for (int entry = 0; entry < 20; ++entry) {
            const std::string expected = "[DEBUG] Probe worker " + std::to_string(writer) + ":" +
                                         std::to_string(entry) + '\n';
            const auto position = contents.find(expected);
            passed = passed && position != std::string::npos &&
                     contents.find(expected, position + expected.size()) == std::string::npos;
        }
    }
    passed = passed && count == 86;
    file.close();
    openbus::probe::removeProbeDirectory(directory);
    if (!passed) {
        std::cerr << "Log severity, concurrent output, or flush validation failed\n";
        return 1;
    }
    return 0;
}
