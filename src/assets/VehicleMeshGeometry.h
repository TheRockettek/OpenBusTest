#pragma once

#include "render/ModelVertex.h"

#include <span>
#include <vector>

namespace openbus::rendering {
struct ObjTriangle;
struct ParsedObj;

// Prepares the selected triangles in their input order using the vehicle coordinate and UV
// conventions. Invalid position indices are skipped, matching the production vehicle loader.
[[nodiscard]] std::vector<ModelVertex>
prepareVehicleMeshGeometry(const ParsedObj& model, std::span<const ObjTriangle* const> triangles);

} // namespace openbus::rendering
