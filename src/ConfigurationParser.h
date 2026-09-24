#pragma once

#include "ConfigurationTypes.h"

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace openbus::config {

struct Line {
    std::size_t number = 0;
    std::string raw;
    std::string text;

    bool isKeyword() const;
    std::string keyword() const;
};

class Reader {
  public:
    explicit Reader(const std::filesystem::path& path);

    bool isOpen() const;
    bool next(Line& line);
    void pushBack(Line line);

    bool readPayload(Line& line, ConfigurationDiagnostics& diagnostics,
                     const std::string& ownerKeyword);
    bool readPayloads(std::size_t count, std::vector<std::string>& values,
                      ConfigurationDiagnostics& diagnostics, const std::string& ownerKeyword);

  private:
    bool readRaw(Line& line);

    std::ifstream input_;
    std::size_t nextLineNumber_ = 0;
    bool disabled_ = false;
    bool hasPushback_ = false;
    Line pushback_;
};

std::string trim(const std::string& value);
std::string lower(std::string value);
bool parseInt(const std::string& value, int& result);
bool parseDouble(const std::string& value, double& result);

} // namespace openbus::config
