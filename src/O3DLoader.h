#pragma once

#include "ObjLoader.h"

#include <array>
#include <filesystem>
#include <memory>

namespace openbus::rendering {

// O3D vertices are right/up/forward; map geometry is east/north/up.
std::array<double, 3> convertO3DPositionToMapAxes(double x, double y, double z);
// O3D map-axis vertices are converted again by the vehicle renderer to its local frame.
std::array<double, 3> convertO3DMapPositionToVehicleAxes(double east, double north, double up);

class O3DLoader {
  public:
    static std::shared_ptr<ParsedObj> parse(const std::filesystem::path& path);
};

} // namespace openbus::rendering