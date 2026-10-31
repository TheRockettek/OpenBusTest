#pragma once

#include "assets/TextureLoader.h"

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

namespace openbus::rendering {

class PreparedMapTextures {
  public:
    void remember(const std::string& key, std::shared_ptr<Image> image,
                  bool alreadyUploaded = false) {
        if (!alreadyUploaded) {
            images_.try_emplace(key, std::move(image));
        }
    }

    bool contains(const std::string& key) const {
        return images_.contains(key);
    }

    void discard(const std::string& key) {
        images_.erase(key);
    }

    template <typename Upload> auto upload(const std::string& key, Upload&& uploadImage) {
        const auto image = images_.find(key);
        auto result = std::forward<Upload>(uploadImage)(
            image == images_.end() ? nullptr : image->second.get());
        if (image != images_.end()) {
            images_.erase(image);
        }
        return result;
    }

    std::size_t count() const {
        return images_.size();
    }

    std::size_t bytes() const {
        std::size_t total = 0;
        for (const auto& [key, image] : images_) {
            static_cast<void>(key);
            if (image) {
                total += image->rgba.capacity();
            }
        }
        return total;
    }

  private:
    std::unordered_map<std::string, std::shared_ptr<Image>> images_;
};

} // namespace openbus::rendering
