#include "ConfigurationParser.h"

#include <charconv>
#include <cctype>
#include <cmath>

namespace openbus::config {

namespace {
constexpr std::size_t MAX_CONFIG_RECORDS = 1'000'000;
}

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
    const std::string normalized = trim(value);
    if (normalized.empty()) {
        return false;
    }

    const char* begin = normalized.data();
    const char* end = begin + normalized.size();
    if (*begin == '+') {
        ++begin;
    }
    if (begin == end) {
        return false;
    }

    int parsed = 0;
    const auto conversion = std::from_chars(begin, end, parsed, 10);
    if (conversion.ec != std::errc() || conversion.ptr != end) {
        return false;
    }
    result = parsed;
    return true;
}

bool parseDouble(const std::string& value, double& result) {
    const std::string normalized = trim(value);
    if (normalized.empty()) {
        return false;
    }

    const char* begin = normalized.data();
    const char* end = begin + normalized.size();
    if (*begin == '+') {
        ++begin;
    }
    if (begin == end) {
        return false;
    }

    double parsed = 0.0;
    const auto conversion = std::from_chars(begin, end, parsed);
    if (conversion.ec != std::errc() || conversion.ptr != end || !std::isfinite(parsed)) {
        return false;
    }
    result = parsed;
    return true;
}

bool parseFloat(const std::string& value, float& result) {
    double parsed = 0.0;
    if (!parseDouble(value, parsed)) {
        return false;
    }
    const float singlePrecision = static_cast<float>(parsed);
    if (!std::isfinite(singlePrecision)) {
        return false;
    }
    result = singlePrecision;
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

bool Reader::readRaw(Line& line, bool skipBlank) {
    // Skip disabled sections and, for ordinary records, blank lines while
    // retaining source line numbers for useful diagnostics.
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
        if (disabled_ || (skipBlank && text.empty())) {
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

bool Reader::nextIncludingBlank(Line& line) {
    if (hasPushback_) {
        line = pushback_;
        hasPushback_ = false;
        return true;
    }
    return readRaw(line, false);
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
    if (count > MAX_CONFIG_RECORDS) {
        diagnostics.error(nextLineNumber_, ownerKeyword, "entry count is too large");
        return false;
    }
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
