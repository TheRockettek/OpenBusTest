#pragma once

#include <cstddef>
#include <type_traits>

namespace openbus::rendering {

struct ModelVertex {
    float x, y, z, u, v, layer, nx, ny, nz;
    float r = 1.0F, g = 1.0F, b = 1.0F, a = 1.0F;
};

static_assert(std::is_standard_layout_v<ModelVertex>);
static_assert(offsetof(ModelVertex, u) == 3 * sizeof(float));
static_assert(offsetof(ModelVertex, nx) == 6 * sizeof(float));
static_assert(offsetof(ModelVertex, r) == 9 * sizeof(float));
static_assert(sizeof(ModelVertex) == 13 * sizeof(float));

} // namespace openbus::rendering
