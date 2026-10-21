#pragma once

#include "render/CoreRenderer.h"

#include <vector>

namespace openbus::rendering {

enum class PrecipitationType {
    None = 0,
    Rain = 1,
    Snow = 2,
};

std::vector<PrimitiveVertex>
makePrecipitationVertices(PrecipitationType type, double rate, double elapsedSeconds);

} // namespace openbus::rendering
