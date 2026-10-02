#include "ObjLoader.h"

#include "PerfTrace.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <fstream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <unordered_map>

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

std::string_view nextToken(std::string_view& input) {
    const std::size_t first = input.find_first_not_of(" \t\r\n");
    if (first == std::string_view::npos) {
        input = {};
        return {};
    }
    input.remove_prefix(first);
    const std::size_t length = input.find_first_of(" \t\r\n");
    const std::string_view token = input.substr(0, length);
    input.remove_prefix(length == std::string_view::npos ? input.size() : length);
    return token;
}

template <typename Number> Number parseNumber(std::string_view value, Number fallback) {
    Number result = fallback;
    const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
    return parsed.ec == std::errc() && parsed.ptr == value.data() + value.size() ? result
                                                                                 : fallback;
}

std::vector<ObjIndex> parseFace(std::string_view value, int positionCount, int texCoordCount,
                                int normalCount) {
    std::vector<ObjIndex> result;
    result.reserve(4);
    for (std::string_view token = nextToken(value); !token.empty(); token = nextToken(value)) {
        ObjIndex index;
        const std::size_t firstSlash = token.find('/');
        const std::size_t secondSlash =
            firstSlash == std::string::npos ? std::string::npos : token.find('/', firstSlash + 1);
        index.position = parseNumber(token.substr(0, firstSlash), 0);
        if (firstSlash != std::string::npos) {
            const std::string_view texCoordText = token.substr(
                firstSlash + 1, secondSlash == std::string::npos ? std::string::npos
                                                                 : secondSlash - firstSlash - 1);
            index.texCoord = parseNumber(texCoordText, 0);
        }
        if (secondSlash != std::string::npos) {
            index.normal = parseNumber(token.substr(secondSlash + 1), 0);
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

namespace {

constexpr char BUNDLE_MAGIC[] = "OBUSOBX1";

void writeU32(std::ostream& output, std::uint32_t value) {
    output.put(static_cast<char>(value & 0xff));
    output.put(static_cast<char>((value >> 8) & 0xff));
    output.put(static_cast<char>((value >> 16) & 0xff));
    output.put(static_cast<char>((value >> 24) & 0xff));
}

void writeString(std::ostream& output, const std::string& value) {
    writeU32(output, static_cast<std::uint32_t>(value.size()));
    output.write(value.data(), static_cast<std::streamsize>(value.size()));
}

template <typename T> void writeValue(std::ostream& output, const T& value) {
    output.write(reinterpret_cast<const char*>(&value), sizeof(value));
}

void writeParsed(std::ostream& output, const ParsedObj& object) {
    writeU32(output, static_cast<std::uint32_t>(object.positions.size()));
    for (const ObjPosition& value : object.positions) {
        writeValue(output, value.x);
        writeValue(output, value.y);
        writeValue(output, value.z);
    }
    writeU32(output, static_cast<std::uint32_t>(object.normals.size()));
    for (const ObjNormal& value : object.normals) {
        writeValue(output, value.x);
        writeValue(output, value.y);
        writeValue(output, value.z);
    }
    writeU32(output, static_cast<std::uint32_t>(object.texCoords.size()));
    for (const ObjTexCoord& value : object.texCoords) {
        writeValue(output, value.u);
        writeValue(output, value.v);
    }
    writeU32(output, static_cast<std::uint32_t>(object.triangles.size()));
    for (const ObjTriangle& triangle : object.triangles) {
        for (const ObjIndex& index : triangle.indices) {
            writeValue(output, index.position);
            writeValue(output, index.texCoord);
            writeValue(output, index.normal);
        }
        writeString(output, triangle.material);
    }
    writeU32(output, static_cast<std::uint32_t>(object.materials.size()));
    for (const auto& [name, material] : object.materials) {
        writeString(output, name);
        writeValue(output, material.materialIndex);
        writeString(output, material.texturePath.generic_string());
        writeString(output, material.textureName);
        for (double value : material.color)
            writeValue(output, value);
        for (double value : material.specular)
            writeValue(output, value);
        for (double value : material.emission)
            writeValue(output, value);
        writeValue(output, material.specularPower);
        writeValue(output, material.alpha);
    }
    for (double value : object.transform)
        writeValue(output, value);
    writeValue(output, object.hasTransform);
    writeValue(output, object.backFaceCulling);
    for (double value : object.boundsCenter)
        writeValue(output, value);
    for (double value : object.boundsSize)
        writeValue(output, value);
    writeValue(output, object.boundsRadius);
}

class BundleReader {
  public:
    explicit BundleReader(const std::vector<std::uint8_t>& data) : data_(data) {}

    std::uint32_t u32() {
        ensure(4);
        const std::uint32_t value = data_[offset_] | (data_[offset_ + 1] << 8) |
                                    (data_[offset_ + 2] << 16) | (data_[offset_ + 3] << 24);
        offset_ += 4;
        return value;
    }

    template <typename T> T value() {
        ensure(sizeof(T));
        T result;
        std::memcpy(&result, data_.data() + offset_, sizeof(T));
        offset_ += sizeof(T);
        return result;
    }

    std::string string() {
        const std::uint32_t length = u32();
        ensure(length);
        std::string result(reinterpret_cast<const char*>(data_.data() + offset_), length);
        offset_ += length;
        return result;
    }

  private:
    void ensure(std::size_t length) {
        if (length > data_.size() - offset_) {
            throw std::runtime_error("Invalid object bundle");
        }
    }
    const std::vector<std::uint8_t>& data_;
    std::size_t offset_ = 0;
};

std::shared_ptr<ParsedObj> readParsed(BundleReader& reader) {
    auto result = std::make_shared<ParsedObj>();
    const auto readCount = [](std::uint32_t count) {
        if (count > 100000000)
            throw std::runtime_error("Object bundle count is too large");
        return static_cast<std::size_t>(count);
    };
    result->positions.resize(readCount(reader.u32()));
    for (ObjPosition& value : result->positions) {
        value.x = reader.value<double>();
        value.y = reader.value<double>();
        value.z = reader.value<double>();
    }
    result->normals.resize(readCount(reader.u32()));
    for (ObjNormal& value : result->normals) {
        value.x = reader.value<double>();
        value.y = reader.value<double>();
        value.z = reader.value<double>();
    }
    result->texCoords.resize(readCount(reader.u32()));
    for (ObjTexCoord& value : result->texCoords) {
        value.u = reader.value<double>();
        value.v = reader.value<double>();
    }
    result->triangles.resize(readCount(reader.u32()));
    for (ObjTriangle& triangle : result->triangles) {
        for (ObjIndex& index : triangle.indices) {
            index.position = reader.value<int>();
            index.texCoord = reader.value<int>();
            index.normal = reader.value<int>();
        }
        triangle.material = reader.string();
    }
    const std::size_t materialCount = readCount(reader.u32());
    for (std::size_t index = 0; index < materialCount; ++index) {
        const std::string name = reader.string();
        ObjMaterial material;
        material.materialIndex = reader.value<int>();
        material.texturePath = reader.string();
        material.textureName = reader.string();
        for (double& value : material.color)
            value = reader.value<double>();
        for (double& value : material.specular)
            value = reader.value<double>();
        for (double& value : material.emission)
            value = reader.value<double>();
        material.specularPower = reader.value<double>();
        material.alpha = reader.value<double>();
        result->materials.emplace(name, std::move(material));
    }
    for (double& value : result->transform)
        value = reader.value<double>();
    result->hasTransform = reader.value<bool>();
    result->backFaceCulling = reader.value<bool>();
    for (double& value : result->boundsCenter)
        value = reader.value<double>();
    for (double& value : result->boundsSize)
        value = reader.value<double>();
    result->boundsRadius = reader.value<double>();
    return result;
}

std::shared_ptr<ParsedObj> parseBundle(const std::filesystem::path& path,
                                       const std::string& entryName) {
    static std::mutex cacheMutex;
    static std::unordered_map<std::string, std::shared_ptr<std::vector<std::uint8_t>>> cache;
    const std::string key = std::filesystem::absolute(path).lexically_normal().string();
    std::shared_ptr<std::vector<std::uint8_t>> data;
    {
        std::lock_guard<std::mutex> lock(cacheMutex);
        auto found = cache.find(key);
        if (found != cache.end())
            data = found->second;
    }
    if (!data) {
        std::ifstream input(path, std::ios::binary);
        if (!input)
            return {};
        input.seekg(0, std::ios::end);
        const std::streamoff length = input.tellg();
        input.seekg(0, std::ios::beg);
        if (length < static_cast<std::streamoff>(sizeof(BUNDLE_MAGIC) - 1))
            return {};
        data = std::make_shared<std::vector<std::uint8_t>>(static_cast<std::size_t>(length));
        input.read(reinterpret_cast<char*>(data->data()), length);
        std::lock_guard<std::mutex> lock(cacheMutex);
        cache.emplace(key, data);
    }
    BundleReader reader(*data);
    for (const char expected : std::string(BUNDLE_MAGIC)) {
        if (reader.value<std::uint8_t>() != static_cast<std::uint8_t>(expected))
            return {};
    }
    const std::uint32_t count = reader.u32();
    for (std::uint32_t index = 0; index < count; ++index) {
        const std::string name = reader.string();
        const std::uint32_t size = reader.u32();
        if (name == entryName) {
            std::vector<std::uint8_t> objectData(size);
            for (std::uint8_t& value : objectData)
                value = reader.value<std::uint8_t>();
            BundleReader objectReader(objectData);
            return readParsed(objectReader);
        }
        for (std::uint32_t byte = 0; byte < size; ++byte)
            reader.value<std::uint8_t>();
    }
    return {};
}

} // namespace

std::shared_ptr<ParsedObj> ObjLoader::parse(const std::filesystem::path& path,
                                            const std::string& bundleEntry) {
    if (!bundleEntry.empty()) {
        return parseBundle(path, bundleEntry);
    }
    TraceScope trace("obj", "parseObj");
    auto result = std::make_shared<ParsedObj>();
    std::filesystem::path materialLibrary;
    std::string currentMaterial;
    {
        TraceScope phase("obj", "parseObj.readObjText");
        std::ifstream input(path);
        if (!input) {
            return {};
        }
        std::string line;
        while (std::getline(input, line)) {
            if (parseTransformComment(line, result->transform)) {
                result->hasTransform = true;
                continue;
            }
            std::string_view remaining(line);
            const std::string_view type = nextToken(remaining);
            if (type == "v") {
                ObjPosition position = {};
                const double sourceX = parseNumber(nextToken(remaining), 0.0);
                const double sourceY = parseNumber(nextToken(remaining), 0.0);
                const double sourceZ = parseNumber(nextToken(remaining), 0.0);
                const std::array<double, 3> converted =
                    convertSourcePosition(sourceX, sourceY, sourceZ);
                position.x = converted[0];
                position.y = converted[1];
                position.z = converted[2];
                result->positions.push_back(position);
            } else if (type == "vn") {
                ObjNormal normal = {};
                const double sourceX = parseNumber(nextToken(remaining), 0.0);
                const double sourceY = parseNumber(nextToken(remaining), 0.0);
                const double sourceZ = parseNumber(nextToken(remaining), 0.0);
                const std::array<double, 3> converted =
                    convertSourcePosition(sourceX, sourceY, sourceZ);
                normal.x = -converted[0];
                normal.y = -converted[1];
                normal.z = -converted[2];
                result->normals.push_back(normal);
            } else if (type == "vt") {
                ObjTexCoord texCoord = {};
                texCoord.u = parseNumber(nextToken(remaining), 0.0);
                texCoord.v = parseNumber(nextToken(remaining), 0.0);
                result->texCoords.push_back(texCoord);
            } else if (type == "mtllib") {
                std::string materialPath(nextToken(remaining));
                std::replace(materialPath.begin(), materialPath.end(), '\\', '/');
                materialLibrary = path.parent_path() / materialPath;
            } else if (type == "usemtl") {
                currentMaterial.assign(nextToken(remaining));
            } else if (type == "f") {
                const std::vector<ObjIndex> face =
                    parseFace(remaining, static_cast<int>(result->positions.size()),
                              static_cast<int>(result->texCoords.size()),
                              static_cast<int>(result->normals.size()));
                for (std::size_t index = 2; index < face.size(); ++index) {
                    result->triangles.push_back(
                        {{{face[0], face[index - 1], face[index]}}, currentMaterial});
                }
            }
        }
    }
    if (result->triangles.empty()) {
        return {};
    }
    {
        TraceScope phase("obj", "parseObj.validateWinding");
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
                subtract(
                    result->positions[static_cast<std::size_t>(triangle.indices[1].position - 1)],
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
    }

    {
        TraceScope phase("obj", "parseObj.calculateBounds");
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
    }
    if (!materialLibrary.empty()) {
        TraceScope phase("obj", "parseObj.readMtl");
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

bool ObjLoader::writeBundle(const std::filesystem::path& output, const std::filesystem::path& root,
                            const std::vector<std::filesystem::path>& objects) {
    std::vector<std::pair<std::string, std::shared_ptr<ParsedObj>>> entries;
    entries.reserve(objects.size());
    for (const std::filesystem::path& objectPath : objects) {
        const std::shared_ptr<ParsedObj> parsed = parse(objectPath);
        if (!parsed)
            continue;
        std::string name = objectPath.lexically_relative(root).generic_string();
        std::transform(name.begin(), name.end(), name.begin(), [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
        entries.emplace_back(std::move(name), parsed);
    }
    if (entries.empty())
        return false;
    std::ofstream file(output, std::ios::binary | std::ios::trunc);
    if (!file)
        return false;
    file.write(BUNDLE_MAGIC, sizeof(BUNDLE_MAGIC) - 1);
    writeU32(file, static_cast<std::uint32_t>(entries.size()));
    for (const auto& entry : entries) {
        std::ostringstream payload(std::ios::binary);
        writeParsed(payload, *entry.second);
        const std::string bytes = payload.str();
        writeU32(file, static_cast<std::uint32_t>(entry.first.size()));
        file.write(entry.first.data(), static_cast<std::streamsize>(entry.first.size()));
        writeU32(file, static_cast<std::uint32_t>(bytes.size()));
        file.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    }
    return file.good();
}

} // namespace openbus::rendering
