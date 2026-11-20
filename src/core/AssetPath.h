#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <filesystem>
#include <string>
#include <system_error>
#include <utility>

#include "core/PerfTrace.h"
#include "core/Utf8Path.h"

namespace openbus::assets {

inline std::filesystem::path textureSubdirectoryPath(const std::filesystem::path& root,
                                                     const std::filesystem::path& relative) {
    if (relative.parent_path().empty()) {
        return {};
    }
    return root / relative.parent_path() / "texture" / relative.filename();
}

inline std::filesystem::path resolveCaseInsensitivePath(const std::filesystem::path& requestedPath,
                                                        bool strictAccess = false) {
    openbus::rendering::TraceScope trace("filesystem", "AssetPath.resolveCaseInsensitivePath");
    const std::filesystem::path requested = requestedPath.lexically_normal();
    std::error_code error;
    if (std::filesystem::exists(requested, error) && !error) {
        return requested;
    }
    const auto checkError = [&](const std::filesystem::path& path) {
        if (strictAccess && error && error != std::errc::no_such_file_or_directory) {
            throw std::filesystem::filesystem_error("Could not resolve path", path, error);
        }
    };
    checkError(requested);

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
        checkError(exact);

        error.clear();
        const bool directory = std::filesystem::is_directory(resolved, error);
        checkError(resolved);
        if (!directory || error) {
            return requested;
        }
        const std::string targetName = lowercase(openbus::pathToUtf8(component));
        std::filesystem::path matchedPath;
        std::filesystem::directory_iterator entry(resolved, error), end;
        for (; !error && entry != end; entry.increment(error)) {
            if (lowercase(openbus::pathToUtf8(entry->path().filename())) != targetName) {
                continue;
            }
            const std::string candidateName = openbus::pathToUtf8(entry->path().filename());
            if (matchedPath.empty() ||
                candidateName < openbus::pathToUtf8(matchedPath.filename())) {
                matchedPath = entry->path();
            }
        }
        checkError(resolved);
        if (error || matchedPath.empty()) {
            return requested;
        }
        resolved = std::move(matchedPath);
    }

    error.clear();
    const bool exists = std::filesystem::exists(resolved, error);
    checkError(resolved);
    return exists && !error ? resolved : requested;
}

inline std::filesystem::path resolveTexturePath(const std::filesystem::path& requestedPath) {
    openbus::rendering::TraceScope trace("filesystem", "AssetPath.resolveTexturePath");
    const std::filesystem::path resolved = resolveCaseInsensitivePath(requestedPath);
    if (std::filesystem::is_regular_file(resolved)) {
        return resolved;
    }

    std::string extension = requestedPath.extension().string();
    std::transform(
        extension.begin(), extension.end(), extension.begin(),
        [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    constexpr std::array<const char*, 6> textureExtensions = {".dds", ".bmp", ".tga",
                                                              ".png", ".jpg", ".jpeg"};
    const bool supportedExtension =
        std::any_of(textureExtensions.begin(), textureExtensions.end(),
                    [&extension](const char* supported) { return extension == supported; });
    if (!supportedExtension) {
        return requestedPath;
    }

    for (const char* alternativeExtension : textureExtensions) {
        if (extension == alternativeExtension) {
            continue;
        }
        std::filesystem::path alternatePath = requestedPath;
        alternatePath.replace_extension(alternativeExtension);
        const std::filesystem::path alternate = resolveCaseInsensitivePath(alternatePath);
        if (std::filesystem::is_regular_file(alternate)) {
            return alternate;
        }
    }
    return requestedPath;
}

inline std::array<std::filesystem::path, 13>
sceneryTextureCandidates(const std::filesystem::path& objectRoot,
                         const std::filesystem::path& modelRoot,
                         const std::filesystem::path& omsiRoot, const std::string& textureName) {
    std::string normalized = textureName;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    const std::filesystem::path relative = openbus::pathFromUtf8(normalized);
    const auto packageRoot = objectRoot.parent_path();
    const auto modelPackageRoot = modelRoot.parent_path();
    return {objectRoot / "texture" / relative,
            objectRoot / relative,
            modelRoot / relative,
            packageRoot / "texture" / relative,
            packageRoot / relative,
            modelPackageRoot / "texture" / relative,
            modelPackageRoot / relative,
            textureSubdirectoryPath(objectRoot, relative),
            textureSubdirectoryPath(modelRoot, relative),
            textureSubdirectoryPath(packageRoot, relative),
            textureSubdirectoryPath(modelPackageRoot, relative),
            omsiRoot / relative,
            omsiRoot / "Texture" / relative};
}

inline std::filesystem::path findSceneryTexture(const std::filesystem::path& objectRoot,
                                                const std::filesystem::path& modelRoot,
                                                const std::filesystem::path& omsiRoot,
                                                const std::string& textureName) {
    openbus::rendering::TraceScope trace("filesystem", "AssetPath.findSceneryTexture");
    const auto candidates = sceneryTextureCandidates(objectRoot, modelRoot, omsiRoot, textureName);
    for (const auto& candidate : candidates) {
        if (!candidate.empty()) {
            const auto resolved = resolveTexturePath(candidate);
            if (std::filesystem::is_regular_file(resolved)) {
                return resolved;
            }
        }
    }
    return {};
}

inline std::filesystem::path resolveReferencedPath(const std::filesystem::path& configPath,
                                                   const std::string& referencedPath,
                                                   bool strictAccess = false) {
    openbus::rendering::TraceScope trace("filesystem", "AssetPath.resolveReferencedPath");
    std::string normalized = referencedPath;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    return resolveCaseInsensitivePath(configPath.parent_path() / openbus::pathFromUtf8(normalized),
                                      strictAccess);
}

} // namespace openbus::assets
