#include "ConfigurationParser.h"

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace openbus::config {

std::string trim(const std::string& value) {
    // CFG payloads are line-oriented, so whitespace is removed before parsing
    // keywords, numbers, and variable names.
    const std::size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const std::size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::string lower(std::string value) {
    for (char& character : value) {
        character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
    }
    return value;
}

bool parseInt(const std::string& value, int& result) {
    // Require the entire trimmed payload to be an integer; partial conversions
    // such as "12px" are rejected.
    const std::string normalized = trim(value);
    if (normalized.empty()) {
        return false;
    }
    char* end = nullptr;
    errno = 0;
    const long parsed = std::strtol(normalized.c_str(), &end, 10);
    if (errno != 0 || end == normalized.c_str() || *end != '\0' ||
        parsed < std::numeric_limits<int>::min() || parsed > std::numeric_limits<int>::max()) {
        return false;
    }
    result = static_cast<int>(parsed);
    return true;
}

bool parseDouble(const std::string& value, double& result) {
    // As with integers, reject trailing characters instead of accepting a
    // partially parsed numeric value.
    const std::string normalized = trim(value);
    if (normalized.empty()) {
        return false;
    }
    char* end = nullptr;
    errno = 0;
    const double parsed = std::strtod(normalized.c_str(), &end);
    if (errno != 0 || end == normalized.c_str() || *end != '\0' || !std::isfinite(parsed)) {
        return false;
    }
    result = parsed;
    return true;
}

bool Line::isKeyword() const {
    // Keywords are preserved exactly as bracketed lines so ordinary payloads
    // beginning with similar text are not misclassified.
    return raw.size() >= 3 && raw.front() == '[' && raw.back() == ']' && raw == text;
}

std::string Line::keyword() const {
    if (!isKeyword()) {
        return {};
    }
    return text.substr(1, text.size() - 2);
}

Reader::Reader(const std::filesystem::path& path) : input_(path) {}

bool Reader::isOpen() const {
    return input_.is_open();
}

bool Reader::readRaw(Line& line) {
    // Skip blank lines and disabled sections while retaining source line numbers
    // for useful diagnostics.
    std::string raw;
    while (std::getline(input_, raw)) {
        ++nextLineNumber_;
        if (!raw.empty() && raw.back() == '\r') {
            raw.pop_back();
        }
        const std::string text = trim(raw);
        if (text == "-<DISABLED>-") {
            disabled_ = true;
            continue;
        }
        if (text == "-<ENABLED>-") {
            disabled_ = false;
            continue;
        }
        if (disabled_ || text.empty()) {
            continue;
        }
        line = {nextLineNumber_, raw, text};
        return true;
    }
    return false;
}

bool Reader::next(Line& line) {
    if (hasPushback_) {
        line = pushback_;
        hasPushback_ = false;
        return true;
    }
    return readRaw(line);
}

void Reader::pushBack(Line line) {
    pushback_ = std::move(line);
    hasPushback_ = true;
}

bool Reader::readPayload(Line& line, ConfigurationDiagnostics& diagnostics,
                         const std::string& ownerKeyword) {
    // A keyword where a payload is expected belongs to the next record, so it is
    // pushed back after reporting the malformed current record.
    if (!next(line)) {
        diagnostics.error(nextLineNumber_, ownerKeyword, "missing value at end of file");
        return false;
    }
    if (line.isKeyword()) {
        diagnostics.error(line.number, ownerKeyword, "expected a value before " + line.text);
        pushBack(std::move(line));
        return false;
    }
    return true;
}

bool Reader::readPayloads(std::size_t count, std::vector<std::string>& values,
                          ConfigurationDiagnostics& diagnostics, const std::string& ownerKeyword) {
    values.clear();
    values.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        Line line;
        if (!readPayload(line, diagnostics, ownerKeyword)) {
            return false;
        }
        values.push_back(line.text);
    }
    return true;
}

} // namespace openbus::config
