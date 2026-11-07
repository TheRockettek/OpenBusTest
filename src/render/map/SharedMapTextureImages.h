#pragma once

#include "assets/TextureLoader.h"
#include "core/AssetPath.h"
#include "render/map/TreeSeason.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace openbus::rendering {

class SharedMapTextureImages {
  public:
    using SeasonalPaths = std::array<std::filesystem::path, map_detail::treeSeasonDirectories.size()>;

    struct Stats {
        std::size_t decodeAttempts;
        std::size_t sharedImageUses;
    };

    Stats stats() const {
        return {decodeAttempts_.load(std::memory_order_relaxed),
                sharedImageUses_.load(std::memory_order_relaxed)};
    }

    template <typename Decode>
    std::shared_ptr<Image> getOrDecode(const std::string& key, Decode&& decode) {
        std::shared_ptr<Entry> entry;
        {
            std::lock_guard lock(mutex_);
            auto position = entries_.find(key);
            const bool inserted = position == entries_.end();
            if (inserted) {
                position = entries_.emplace(key, std::make_shared<Entry>()).first;
            }
            entry = position->second;
            if (inserted && ++insertionsSinceCleanup_ == 128) {
                insertionsSinceCleanup_ = 0;
                for (auto it = entries_.begin(); it != entries_.end();) {
                    // A checked-out entry may be decoding or have waiting callers.
                    if (it->second.use_count() == 1 && it->second->image.expired()) {
                        it = entries_.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
        }

        std::lock_guard lock(entry->mutex);
        if (auto image = entry->image.lock()) {
            sharedImageUses_.fetch_add(1, std::memory_order_relaxed);
            return image;
        }
        decodeAttempts_.fetch_add(1, std::memory_order_relaxed);
        auto image = std::forward<Decode>(decode)();
        if (!image) {
            throw std::runtime_error("Texture decoder returned no image for " + key);
        }
        entry->image = image;
        return image;
    }

    std::shared_ptr<const SeasonalPaths> seasonalPaths(const std::filesystem::path& texture) {
        if (texture.empty()) {
            return {};
        }
        const std::string textureKey = openbus::pathToUtf8(texture.lexically_normal());
        SeasonalPaths folders;
        {
            std::lock_guard lock(seasonalMutex_);
            const auto cached = seasonalPaths_.find(textureKey);
            if (cached != seasonalPaths_.end()) {
                return cached->second;
            }
            const auto parent = texture.parent_path();
            const std::string folderKey = openbus::pathToUtf8(parent.lexically_normal());
            auto found = seasonalFolders_.find(folderKey);
            if (found == seasonalFolders_.end()) {
                SeasonalPaths discovered;
                for (std::size_t index = 0; index < discovered.size(); ++index) {
                    const auto candidate = openbus::assets::resolveCaseInsensitivePath(
                        parent / map_detail::treeSeasonDirectories[index]);
                    if (std::filesystem::is_directory(candidate)) {
                        discovered[index] = candidate;
                    }
                }
                found = seasonalFolders_.emplace(folderKey, std::move(discovered)).first;
            }
            folders = found->second;
        }
        SeasonalPaths discovered;
        bool any = false;
        for (std::size_t index = 0; index < folders.size(); ++index) {
            if (folders[index].empty()) {
                continue;
            }
            const auto candidate =
                openbus::assets::resolveTexturePath(folders[index] / texture.filename());
            if (std::filesystem::is_regular_file(candidate)) {
                discovered[index] = candidate;
                any = true;
            }
        }
        std::shared_ptr<const SeasonalPaths> result;
        if (any) {
            result = std::make_shared<SeasonalPaths>(std::move(discovered));
        }
        std::lock_guard lock(seasonalMutex_);
        return seasonalPaths_.emplace(textureKey, std::move(result)).first->second;
    }

  private:
    struct Entry {
        std::mutex mutex;
        std::weak_ptr<Image> image;
    };

    std::mutex mutex_;
    std::unordered_map<std::string, std::shared_ptr<Entry>> entries_;
    std::mutex seasonalMutex_;
    std::unordered_map<std::string, SeasonalPaths> seasonalFolders_;
    std::unordered_map<std::string, std::shared_ptr<const SeasonalPaths>> seasonalPaths_;
    std::size_t insertionsSinceCleanup_ = 0;
    std::atomic<std::size_t> decodeAttempts_ = 0;
    std::atomic<std::size_t> sharedImageUses_ = 0;
};

} // namespace openbus::rendering
