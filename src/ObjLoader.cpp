#include "ObjLoader.h"

#include "PerfTrace.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace {

std::array<double, 3> convertSourcePosition(double x, double y, double z) {
    return {x, z, y};
}

std::array<double, 16> convertSourceTransform(const std::array<double, 16>& source) {
    std::array<double, 16> converted = source;
    const auto sourceAxis = [](int axis) { return axis == 1 ? 2 : axis == 2 ? 1 : axis; };
    for (int column = 0; column < 3; ++column) {
        for (int row = 0; row < 3; ++row) {
            converted[row + column * 4] = source[sourceAxis(row) + sourceAxis(column) * 4];
        }
    }
    converted[13] = source[14];
    converted[14] = source[13];
    return converted;
}

double transformDeterminant(const std::array<double, 16>& transform) {
    const double a = transform[0];
    const double b = transform[4];
    const double c = transform[8];
    const double d = transform[1];
    const double e = transform[5];
    const double f = transform[9];
    const double g = transform[2];
    const double h = transform[6];
    const double i = transform[10];
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
}

std::array<double, 3> transformVector(const std::array<double, 16>& transform,
                                      const std::array<double, 3>& vector) {
    return {transform[0] * vector[0] + transform[4] * vector[1] + transform[8] * vector[2],
            transform[1] * vector[0] + transform[5] * vector[1] + transform[9] * vector[2],
            transform[2] * vector[0] + transform[6] * vector[1] + transform[10] * vector[2]};
}

double dot(const std::array<double, 3>& left, const std::array<double, 3>& right) {
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

std::array<double, 3> subtract(const openbus::rendering::ObjPosition& left,
                               const openbus::rendering::ObjPosition& right) {
    return {left.x - right.x, left.y - right.y, left.z - right.z};
}

std::array<double, 3> cross(const std::array<double, 3>& left, const std::array<double, 3>& right) {
    return {left[1] * right[2] - left[2] * right[1], left[2] * right[0] - left[0] * right[2],
            left[0] * right[1] - left[1] * right[0]};
}

bool parseTransformComment(const std::string& line, std::array<double, 16>& transform) {
    constexpr const char* prefix = "# openbus_transform";
    if (line.rfind(prefix, 0) != 0) {
        return false;
    }
    std::istringstream values(line.substr(std::char_traits<char>::length(prefix)));
    std::array<double, 16> source = {};
    for (double& value : source) {
        if (!(values >> value)) {
            return false;
        }
    }
    transform = convertSourceTransform(source);
    return true;
}

} // namespace

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
        if (parseTransformComment(line, result->transform)) {
            result->hasTransform = true;
            continue;
        }
        std::istringstream stream(line);
        std::string type;
        stream >> type;
        if (type == "v") {
            ObjPosition position = {};
            double sourceX = 0.0;
            double sourceY = 0.0;
            double sourceZ = 0.0;
            stream >> sourceX >> sourceY >> sourceZ;
            const std::array<double, 3> converted =
                convertSourcePosition(sourceX, sourceY, sourceZ);
            position.x = converted[0];
            position.y = converted[1];
            position.z = converted[2];
            result->positions.push_back(position);
        } else if (type == "vn") {
            ObjNormal normal = {};
            double sourceX = 0.0;
            double sourceY = 0.0;
            double sourceZ = 0.0;
            stream >> sourceX >> sourceY >> sourceZ;
            const std::array<double, 3> converted =
                convertSourcePosition(sourceX, sourceY, sourceZ);
            normal.x = -converted[0];
            normal.y = -converted[1];
            normal.z = -converted[2];
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
    const bool mirrored = result->hasTransform && transformDeterminant(result->transform) > 0.0;
    std::size_t against = 0;
    std::size_t againstTurned = 0;
    std::size_t counted = 0;
    for (const ObjTriangle& triangle : result->triangles) {
        std::array<std::array<double, 3>, 3> normals = {};
        bool valid = true;
        for (std::size_t index = 0; index < 3; ++index) {
            const ObjIndex& objIndex = triangle.indices[index];
            if (objIndex.position <= 0 ||
                objIndex.position > static_cast<int>(result->positions.size()) ||
                objIndex.normal <= 0 ||
                objIndex.normal > static_cast<int>(result->normals.size())) {
                valid = false;
                break;
            }
            const ObjNormal& normal =
                result->normals[static_cast<std::size_t>(objIndex.normal - 1)];
            normals[index] = {normal.x, normal.y, normal.z};
        }
        if (!valid) {
            continue;
        }
        const std::array<double, 3> face = cross(
            subtract(result->positions[static_cast<std::size_t>(triangle.indices[1].position - 1)],
                     result->positions[static_cast<std::size_t>(triangle.indices[0].position - 1)]),
            subtract(
                result->positions[static_cast<std::size_t>(triangle.indices[2].position - 1)],
                result->positions[static_cast<std::size_t>(triangle.indices[0].position - 1)]));
        const std::array<double, 3> normal = {normals[0][0] + normals[1][0] + normals[2][0],
                                              normals[0][1] + normals[1][1] + normals[2][1],
                                              normals[0][2] + normals[1][2] + normals[2][2]};
        const double faceLength = dot(face, face);
        const double normalLength = dot(normal, normal);
        if (faceLength <= 1.0e-12 || normalLength <= 1.0e-12) {
            continue;
        }
        ++counted;
        if (dot(face, normal) < 0.0) {
            ++against;
        }
        if (dot(face, transformVector(result->transform, normal)) < 0.0) {
            ++againstTurned;
        }
    }
    const bool transformedNormalsExplainWinding = againstTurned * 10 <= counted;
    const bool reverseWinding = mirrored && !transformedNormalsExplainWinding && counted >= 2 &&
                                against * 10 >= counted * 9;
    if (reverseWinding) {
        for (ObjTriangle& triangle : result->triangles) {
            std::swap(triangle.indices[1], triangle.indices[2]);
        }
    }
    result->backFaceCulling = mirrored;
    std::array<double, 3> boundsMin = {std::numeric_limits<double>::max(),
                                       std::numeric_limits<double>::max(),
                                       std::numeric_limits<double>::max()};
    std::array<double, 3> boundsMax = {std::numeric_limits<double>::lowest(),
                                       std::numeric_limits<double>::lowest(),
                                       std::numeric_limits<double>::lowest()};
    for (const ObjPosition& position : result->positions) {
        const std::array<double, 3> converted = {position.y, -position.x, position.z};
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
        const double x = position.y - result->boundsCenter[0];
        const double y = -position.x - result->boundsCenter[1];
        const double z = position.z - result->boundsCenter[2];
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
