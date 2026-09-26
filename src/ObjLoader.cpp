#include "ObjLoader.h"

#include "PerfTrace.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace openbus::rendering {
namespace {

std::string trim(const std::string& value) {
    const std::size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const std::size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

int parseInt(const std::string& value, int fallback) {
    try {
        return std::stoi(trim(value));
    } catch (const std::exception&) {
        return fallback;
    }
}

std::vector<ObjIndex> parseFace(const std::string& value, int positionCount, int texCoordCount,
                                int normalCount) {
    std::vector<ObjIndex> result;
    std::istringstream stream(value);
    std::string token;
    while (stream >> token) {
        ObjIndex index;
        const std::size_t firstSlash = token.find('/');
        const std::size_t secondSlash =
            firstSlash == std::string::npos ? std::string::npos : token.find('/', firstSlash + 1);
        const std::string positionText = token.substr(0, firstSlash);
        index.position = parseInt(positionText, 0);
        if (firstSlash != std::string::npos) {
            const std::string texCoordText = token.substr(
                firstSlash + 1, secondSlash == std::string::npos ? std::string::npos
                                                                 : secondSlash - firstSlash - 1);
            index.texCoord = parseInt(texCoordText, 0);
        }
        if (secondSlash != std::string::npos) {
            index.normal = parseInt(token.substr(secondSlash + 1), 0);
        }
        if (index.position < 0) {
            index.position += positionCount + 1;
        }
        if (index.texCoord < 0) {
            index.texCoord += texCoordCount + 1;
        }
        if (index.normal < 0) {
            index.normal += normalCount + 1;
        }
        result.push_back(index);
    }
    return result;
}

} // namespace

std::shared_ptr<ParsedObj> ObjLoader::parse(const std::filesystem::path& path) {
    TraceScope trace("obj", "parseObj");
    auto result = std::make_shared<ParsedObj>();
    std::ifstream input(path);
    if (!input) {
        return {};
    }
    std::filesystem::path materialLibrary;
    std::string currentMaterial;
    std::string line;
    while (std::getline(input, line)) {
        std::istringstream stream(line);
        std::string type;
        stream >> type;
        if (type == "v") {
            ObjPosition position = {};
            stream >> position.x >> position.y >> position.z;
            result->positions.push_back(position);
        } else if (type == "vn") {
            ObjNormal normal = {};
            stream >> normal.x >> normal.y >> normal.z;
            result->normals.push_back(normal);
        } else if (type == "vt") {
            ObjTexCoord texCoord = {};
            stream >> texCoord.u >> texCoord.v;
            result->texCoords.push_back(texCoord);
        } else if (type == "mtllib") {
            std::string materialPath;
            stream >> materialPath;
            std::replace(materialPath.begin(), materialPath.end(), '\\', '/');
            materialLibrary = path.parent_path() / materialPath;
        } else if (type == "usemtl") {
            stream >> currentMaterial;
        } else if (type == "f") {
            const std::size_t typeEnd = line.find(' ');
            if (typeEnd == std::string::npos) {
                continue;
            }
            const std::vector<ObjIndex> face =
                parseFace(line.substr(typeEnd + 1), static_cast<int>(result->positions.size()),
                          static_cast<int>(result->texCoords.size()),
                          static_cast<int>(result->normals.size()));
            for (std::size_t index = 2; index < face.size(); ++index) {
                result->triangles.push_back(
                    {{{face[0], face[index - 1], face[index]}}, currentMaterial});
            }
        }
    }
    if (result->triangles.empty()) {
        return {};
    }
    std::array<double, 3> boundsMin = {std::numeric_limits<double>::max(),
                                       std::numeric_limits<double>::max(),
                                       std::numeric_limits<double>::max()};
    std::array<double, 3> boundsMax = {std::numeric_limits<double>::lowest(),
                                       std::numeric_limits<double>::lowest(),
                                       std::numeric_limits<double>::lowest()};
    for (const ObjPosition& position : result->positions) {
        const std::array<double, 3> converted = {position.z, -position.x, position.y};
        for (int axis = 0; axis < 3; ++axis) {
            boundsMin[axis] = std::min(boundsMin[axis], converted[axis]);
            boundsMax[axis] = std::max(boundsMax[axis], converted[axis]);
        }
    }
    for (int axis = 0; axis < 3; ++axis) {
        result->boundsCenter[axis] = (boundsMin[axis] + boundsMax[axis]) * 0.5;
        result->boundsSize[axis] = boundsMax[axis] - boundsMin[axis];
    }
    for (const ObjPosition& position : result->positions) {
        const double x = position.z - result->boundsCenter[0];
        const double y = -position.x - result->boundsCenter[1];
        const double z = position.y - result->boundsCenter[2];
        result->boundsRadius = std::max(result->boundsRadius, std::sqrt(x * x + y * y + z * z));
    }
    if (!materialLibrary.empty()) {
        std::ifstream materialInput(materialLibrary);
        std::string materialLine;
        std::string materialName;
        while (std::getline(materialInput, materialLine)) {
            std::istringstream materialStream(materialLine);
            std::string materialType;
            materialStream >> materialType;
            if (materialType == "newmtl") {
                materialStream >> materialName;
                    ObjMaterial material;
                    if (materialName.rfind("matl_", 0) == 0) {
                        try {
                            material.materialIndex = std::stoi(materialName.substr(5));
                        } catch (const std::exception&) {
                            material.materialIndex = -1;
                        }
                    }
                    result->materials[materialName] = material;
            } else if (materialType == "map_Kd" && !materialName.empty()) {
                std::string textureName;
                std::getline(materialStream, textureName);
                result->materials[materialName].textureName = trim(textureName);
            } else if (materialType == "Kd" && !materialName.empty()) {
                materialStream >> result->materials[materialName].color[0] >>
                    result->materials[materialName].color[1] >>
                    result->materials[materialName].color[2];
            } else if (materialType == "Ks" && !materialName.empty()) {
                materialStream >> result->materials[materialName].specular[0] >>
                    result->materials[materialName].specular[1] >>
                    result->materials[materialName].specular[2];
            } else if (materialType == "Ke" && !materialName.empty()) {
                materialStream >> result->materials[materialName].emission[0] >>
                    result->materials[materialName].emission[1] >>
                    result->materials[materialName].emission[2];
            } else if (materialType == "Ns" && !materialName.empty()) {
                materialStream >> result->materials[materialName].specularPower;
            } else if (materialType == "d" && !materialName.empty()) {
                materialStream >> result->materials[materialName].alpha;
            } else if (materialType == "Tr" && !materialName.empty()) {
                double transparency = 0.0;
                materialStream >> transparency;
                result->materials[materialName].alpha = 1.0 - transparency;
            }
        }
    }
    return result;
}

} // namespace openbus::rendering
