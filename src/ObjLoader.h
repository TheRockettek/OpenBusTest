#pragma once

#include <array>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace openbus::rendering {

struct ObjPosition {
    double x;
    double y;
    double z;
};

struct ObjTexCoord {
    double u;
    double v;
};

struct ObjIndex {
    int position = 0;
    int texCoord = 0;
};

struct ObjTriangle {
    std::array<ObjIndex, 3> indices;
    std::string material;
};

struct ObjMaterial {
    std::filesystem::path texturePath;
    std::string textureName;
    std::array<double, 3> color = {0.65, 0.65, 0.65};
};

struct ParsedObj {
    std::vector<ObjPosition> positions;
    std::vector<ObjTexCoord> texCoords;
    std::vector<ObjTriangle> triangles;
    std::unordered_map<std::string, ObjMaterial> materials;
    std::array<double, 3> boundsCenter = {};
    std::array<double, 3> boundsSize = {};
    double boundsRadius = 0.0;
};

class ObjLoader {
  public:
    static std::shared_ptr<ParsedObj> parse(const std::filesystem::path& path);
};

} // namespace openbus::rendering
