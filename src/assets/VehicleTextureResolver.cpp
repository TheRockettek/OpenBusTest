#include "assets/VehicleTextureResolver.h"

#include "core/AssetPath.h"
#include "core/Environment.h"
#include "core/Logger.h"
#include "core/PerfTrace.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <system_error>
#include <vector>

namespace openbus::assets {
namespace {

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return value;
}

std::string trim(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
}

bool isImageExtension(const std::string& extension) {
    constexpr std::array<const char*, 6> extensions = {".tga", ".bmp", ".png",
                                                       ".dds", ".jpg", ".jpeg"};
    return std::find(extensions.begin(), extensions.end(), extension) != extensions.end();
}

int textureFormatPriority(const std::filesystem::path& path) {
    const std::string extension = lower(path.extension().string());
    return extension == ".dds" ? 3 : extension == ".tga" ? 2 : 1;
}

std::string normalizedPathKey(const std::filesystem::path& path) {
    return lower(std::filesystem::absolute(path).lexically_normal().generic_string());
}

} // namespace

std::filesystem::path VehicleTextureResolver::find(const std::filesystem::path& root,
                                                   const std::string& name) {
    rendering::TraceScope trace("texture", "findTexture");
    const std::string cleaned = trim(name);
    if (cleaned.empty()) {
        return {};
    }
    static const bool verbose =
        rendering::parseEnabledFlag(openbus::getEnvironment("OPENBUS_VERBOSE_TEXTURE_LOOKUP"));
    const auto normalizedRequest = std::filesystem::path(cleaned).lexically_normal();
    const std::string resolutionKey =
        normalizedPathKey(root) + "|" + lower(normalizedRequest.generic_string());
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto cached = resolutions_.find(resolutionKey);
        if (cached != resolutions_.end()) {
            return cached->second;
        }
    }
    if (verbose) {
        gameLog.Debug("Resolving texture with name: " + name +
                      " in root: " + root.generic_string());
    }
    const auto cacheResult = [&](const std::filesystem::path& path) {
        std::lock_guard<std::mutex> lock(mutex_);
        resolutions_.emplace(resolutionKey, path);
        return path;
    };
    const auto preferCompressedSibling = [&](const std::filesystem::path& path) {
        if (lower(path.extension().string()) == ".dds") {
            return path;
        }
        std::filesystem::path compressed = path;
        compressed.replace_extension(".dds");
        return std::filesystem::exists(compressed) ? compressed : path;
    };
    const auto getTextureIndex = [&](const std::filesystem::path& textureRoot) {
        const std::string key = normalizedPathKey(textureRoot);
        std::shared_ptr<TextureIndex> index;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto found = textureIndexes_.find(key);
            if (found != textureIndexes_.end()) {
                index = found->second;
            } else {
                index = std::make_shared<TextureIndex>();
                textureIndexes_.emplace(key, index);
            }
        }
        {
            std::lock_guard<std::mutex> lock(index->mutex);
            if (!index->built) {
                if (accessPolicy_ == VehicleTextureAccessPolicy::Strict) {
                    index->byFileName.clear();
                    index->byStem.clear();
                }
                const auto options =
                    accessPolicy_ == VehicleTextureAccessPolicy::Strict
                        ? std::filesystem::directory_options::none
                        : std::filesystem::directory_options::skip_permission_denied;
                for (const auto& entry :
                     std::filesystem::recursive_directory_iterator(textureRoot, options)) {
                    if (!entry.is_regular_file() ||
                        !isImageExtension(lower(entry.path().extension().string()))) {
                        continue;
                    }
                    const std::string fileName = lower(entry.path().filename().string());
                    const std::string stem = lower(entry.path().stem().string());
                    index->byFileName.emplace(fileName, entry.path());
                    const auto existing = index->byStem.find(stem);
                    if (existing == index->byStem.end() ||
                        textureFormatPriority(entry.path()) >
                            textureFormatPriority(existing->second)) {
                        index->byStem[stem] = entry.path();
                    }
                }
                index->built = true;
            }
        }
        return index;
    };
    const auto getRootIndex = [&](const std::filesystem::path& textureRoot) {
        const std::string key = normalizedPathKey(textureRoot);
        std::shared_ptr<RootIndex> index;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto found = rootIndexes_.find(key);
            if (found != rootIndexes_.end()) {
                index = found->second;
            } else {
                index = std::make_shared<RootIndex>();
                rootIndexes_.emplace(key, index);
            }
        }
        {
            std::lock_guard<std::mutex> lock(index->mutex);
            if (!index->built) {
                if (accessPolicy_ == VehicleTextureAccessPolicy::Strict) {
                    index->byStem.clear();
                }
                std::error_code error;
                const auto options =
                    accessPolicy_ == VehicleTextureAccessPolicy::Strict
                        ? std::filesystem::directory_options::none
                        : std::filesystem::directory_options::skip_permission_denied;
                std::filesystem::directory_iterator entry(textureRoot, options, error);
                const std::filesystem::directory_iterator end;
                for (; !error && entry != end; entry.increment(error)) {
                    std::error_code statusError;
                    const bool regularFile = entry->is_regular_file(statusError);
                    if (statusError && accessPolicy_ == VehicleTextureAccessPolicy::Strict) {
                        throw std::filesystem::filesystem_error("Could not inspect texture entry",
                                                                entry->path(), statusError);
                    }
                    if (!regularFile || statusError ||
                        !isImageExtension(lower(entry->path().extension().string()))) {
                        continue;
                    }
                    const std::string stem = lower(entry->path().stem().string());
                    const auto existing = index->byStem.find(stem);
                    if (existing == index->byStem.end() ||
                        textureFormatPriority(entry->path()) >
                            textureFormatPriority(existing->second)) {
                        index->byStem[stem] = entry->path();
                    }
                }
                if (error && accessPolicy_ == VehicleTextureAccessPolicy::Strict) {
                    throw std::filesystem::filesystem_error("Could not enumerate texture directory",
                                                            textureRoot, error);
                }
                index->built = true;
            }
        }
        return index;
    };

    std::filesystem::path direct = root / cleaned;
    if (std::filesystem::exists(direct)) {
        return cacheResult(preferCompressedSibling(direct));
    }
    std::vector<std::filesystem::path> textureRoots;
    for (std::filesystem::path ancestor = root; !ancestor.empty();
         ancestor = ancestor.parent_path()) {
        const auto textureRoot = resolveCaseInsensitivePath(
            ancestor / "Texture", accessPolicy_ == VehicleTextureAccessPolicy::Strict);
        if (std::filesystem::exists(textureRoot)) {
            textureRoots.push_back(textureRoot);
            direct = textureRoot / cleaned;
            if (std::filesystem::exists(direct)) {
                return cacheResult(preferCompressedSibling(direct));
            }
        }
        if (ancestor.parent_path() == ancestor) {
            break;
        }
    }

    const std::string basename = lower(std::filesystem::path(cleaned).filename().string());
    const std::string stem = lower(std::filesystem::path(cleaned).stem().string());
    if (std::filesystem::exists(root)) {
        const auto index = getRootIndex(root);
        std::lock_guard<std::mutex> lock(index->mutex);
        const auto found = index->byStem.find(stem);
        if (found != index->byStem.end()) {
            if (verbose) {
                gameLog.Debug("Found texture directly at path: " + found->second.generic_string());
            }
            return cacheResult(found->second);
        }
    }
    for (const auto& textureRoot : textureRoots) {
        const auto index = getTextureIndex(textureRoot);
        std::lock_guard<std::mutex> lock(index->mutex);
        const auto byName = index->byFileName.find(basename);
        if (byName != index->byFileName.end()) {
            if (verbose) {
                gameLog.Debug("Found texture at indexed filename path: " +
                              byName->second.generic_string());
            }
            return cacheResult(byName->second);
        }
        const auto byStem = index->byStem.find(stem);
        if (byStem != index->byStem.end()) {
            if (verbose) {
                gameLog.Debug("Found texture at indexed stem path: " +
                              byStem->second.generic_string());
            }
            return cacheResult(byStem->second);
        }
    }
    if (logMissing_) {
        textureLog.Warning("missing: " + cleaned);
    }
    return cacheResult({});
}

} // namespace openbus::assets
