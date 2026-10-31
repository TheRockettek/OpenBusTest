#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <utility>

namespace openbus::rendering {

class PreparedTerrainTexturePaths {
  public:
    template <typename Resolve>
    std::filesystem::path resolve(const std::string& name, Resolve&& resolvePath) {
        const auto cached = paths_.find(name);
        if (cached != paths_.end() && std::filesystem::is_regular_file(cached->second)) {
            ++reuses_;
            return cached->second;
        }
        ++resolutions_;
        std::filesystem::path path = std::forward<Resolve>(resolvePath)();
        if (!path.empty()) {
            paths_.insert_or_assign(name, path);
        }
        return path;
    }

    std::size_t resolutions() const {
        return resolutions_;
    }

    std::size_t reuses() const {
        return reuses_;
    }

  private:
    std::unordered_map<std::string, std::filesystem::path> paths_;
    std::size_t resolutions_ = 0;
    std::size_t reuses_ = 0;
};

} // namespace openbus::rendering
