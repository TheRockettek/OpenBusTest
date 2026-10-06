#pragma once

#include "MapConfigLoader.h"

#include <filesystem>
#include <memory>

namespace openbus::rendering {

class MapRenderer {
  public:
    MapRenderer(const openbus::map::MapDefinition& map, std::size_t centerTileIndex,
                std::size_t groundTextureIndex, const std::filesystem::path& omsiRoot);
    ~MapRenderer();

    MapRenderer(const MapRenderer&) = delete;
    MapRenderer& operator=(const MapRenderer&) = delete;

    void draw() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::rendering