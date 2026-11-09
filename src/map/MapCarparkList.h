#pragma once

#include <charconv>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace openbus::map {

inline std::optional<int> carparkListIndex(const std::vector<std::string>& labels) {
    if (labels.empty() || labels.front().empty()) {
        return 0;
    }
    int index = 0;
    const std::string& selector = labels.front();
    const auto [end, error] =
        std::from_chars(selector.data(), selector.data() + selector.size(), index);
    if (error != std::errc{} || end != selector.data() + selector.size() || index < 0) {
        return std::nullopt;
    }
    return index;
}

inline std::string carparkListFilename(int index) {
    return index == 0 ? "parklist_p.txt" : "parklist_p_" + std::to_string(index) + ".txt";
}

struct CarparkListReadResult {
    bool opened = false;
    bool complete = false;
    std::vector<std::string> entries;
};

template <typename AcceptEntry>
CarparkListReadResult readCarparkList(const std::filesystem::path& path, AcceptEntry accept) {
    CarparkListReadResult result;
    std::ifstream input(path);
    if (!input) {
        return result;
    }
    result.opened = true;
    std::string line;
    while (std::getline(input, line)) {
        if (result.entries.empty() && line.size() >= 3 &&
            static_cast<unsigned char>(line[0]) == 0xEF &&
            static_cast<unsigned char>(line[1]) == 0xBB &&
            static_cast<unsigned char>(line[2]) == 0xBF) {
            line.erase(0, 3);
        }
        const std::size_t first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos || line[first] == ';' || line[first] == '#') {
            continue;
        }
        const std::size_t last = line.find_last_not_of(" \t\r\n");
        std::string assetPath = line.substr(first, last - first + 1);
        if (accept(assetPath)) {
            result.entries.push_back(std::move(assetPath));
        }
    }
    result.complete = !input.bad();
    return result;
}

} // namespace openbus::map
