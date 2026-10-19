#pragma once

#include <cstddef>
#include <string>
#include <vector>

struct ConfigurationDiagnostic {
    enum class Severity { Warning, Error };

    Severity severity = Severity::Warning;
    std::size_t line = 0;
    std::string keyword;
    std::string message;
};

struct ConfigurationDiagnostics {
    std::vector<ConfigurationDiagnostic> entries;

    void warning(std::size_t line, const std::string& keyword, const std::string& message) {
        entries.push_back({ConfigurationDiagnostic::Severity::Warning, line, keyword, message});
    }

    void error(std::size_t line, const std::string& keyword, const std::string& message) {
        entries.push_back({ConfigurationDiagnostic::Severity::Error, line, keyword, message});
    }

    bool hasErrors() const {
        for (const ConfigurationDiagnostic& diagnostic : entries) {
            if (diagnostic.severity == ConfigurationDiagnostic::Severity::Error) {
                return true;
            }
        }
        return false;
    }
};
