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

using ObjNormal = ObjPosition;

struct ObjTexCoord {
    double u;
    double v;
};

struct ObjIndex {
    int position = 0;
    int texCoord = 0;
    int normal = 0;
};

struct ObjTriangle {
    std::array<ObjIndex, 3> indices;
    std::string material;
};

struct ObjMaterial {
    int materialIndex = -1;
    std::filesystem::path texturePath;
    std::string textureName;
    std::array<double, 3> color = {0.65, 0.65, 0.65};
    std::array<double, 3> specular = {};
    std::array<double, 3> emission = {};
    double specularPower = 8.0;
    double alpha = 1.0;
};

struct ParsedObj {
    std::vector<ObjPosition> positions;
    std::vector<ObjNormal> normals;
    std::vector<ObjTexCoord> texCoords;
    std::vector<ObjTriangle> triangles;
    std::unordered_map<std::string, ObjMaterial> materials;
    std::array<double, 16> transform = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    bool hasTransform = false;
    bool backFaceCulling = false;
    std::array<double, 3> boundsCenter = {};
    std::array<double, 3> boundsSize = {};
    double boundsRadius = 0.0;
};

class ObjLoader {
  public:
    static std::shared_ptr<ParsedObj> parse(const std::filesystem::path& path,
                                            const std::string& bundleEntry = {});
    static bool writeBundle(const std::filesystem::path& output, const std::filesystem::path& root,
                            const std::vector<std::filesystem::path>& objects);
};

} // namespace openbus::rendering
