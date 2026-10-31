#pragma once

#include "assets/TextureLoader.h"

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

  private:
    struct Entry {
        std::mutex mutex;
        std::weak_ptr<Image> image;
    };

    std::mutex mutex_;
    std::unordered_map<std::string, std::shared_ptr<Entry>> entries_;
    std::size_t insertionsSinceCleanup_ = 0;
    std::atomic<std::size_t> decodeAttempts_ = 0;
    std::atomic<std::size_t> sharedImageUses_ = 0;
};

} // namespace openbus::rendering
