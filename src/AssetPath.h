#pragma once

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <string>
#include <system_error>
#include <utility>

namespace openbus::assets {

inline std::filesystem::path resolveCaseInsensitivePath(const std::filesystem::path& requestedPath) {
    const std::filesystem::path requested = requestedPath.lexically_normal();
    std::error_code error;
    if (std::filesystem::exists(requested, error) && !error) {
        return requested;
    }

    const auto lowercase = [](std::string value) {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
        return value;
    };

    std::filesystem::path resolved = requested.root_path();
    for (const std::filesystem::path& component : requested.relative_path()) {
        const std::filesystem::path exact = resolved / component;
        error.clear();
        if (std::filesystem::exists(exact, error) && !error) {
            resolved = exact;
            continue;
        }

        error.clear();
        if (!std::filesystem::is_directory(resolved, error) || error) {
            return requested;
        }
        const std::string targetName = lowercase(component.string());
        std::filesystem::path matchedPath;
        std::filesystem::directory_iterator entry(resolved, error), end;
        for (; !error && entry != end; entry.increment(error)) {
            if (lowercase(entry->path().filename().string()) != targetName) {
                continue;
            }
            const std::string candidateName = entry->path().filename().string();
            if (matchedPath.empty() || candidateName < matchedPath.filename().string()) {
                matchedPath = entry->path();
            }
        }
        if (error || matchedPath.empty()) {
            return requested;
        }
        resolved = std::move(matchedPath);
    }

    error.clear();
    return std::filesystem::exists(resolved, error) && !error ? resolved : requested;
}

inline std::filesystem::path resolveReferencedPath(const std::filesystem::path& configPath,
                                                   const std::string& referencedPath) {
    std::string normalized = referencedPath;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    return resolveCaseInsensitivePath(configPath.parent_path() / normalized);
}

} // namespace openbus::assets
