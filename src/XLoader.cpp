#include "XLoader.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace openbus::rendering {
namespace {

using Matrix = std::array<double, 16>;

constexpr Matrix identityMatrix = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                   0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};

struct MeshData {
    std::vector<ObjPosition> positions;
    std::vector<std::vector<int>> faces;
    std::vector<ObjNormal> normals;
    std::vector<std::vector<int>> normalFaces;
    std::vector<ObjTexCoord> texCoords;
    std::vector<int> faceMaterials;
    std::vector<ObjMaterial> materials;
};

std::vector<std::string> tokenize(std::string_view source) {
    std::vector<std::string> tokens;
    for (std::size_t index = 0; index < source.size();) {
        const unsigned char current = static_cast<unsigned char>(source[index]);
        if (std::isspace(current) != 0) {
            ++index;
            continue;
        }
        if (source[index] == '/' && index + 1 < source.size() && source[index + 1] == '/') {
            index += 2;
            while (index < source.size() && source[index] != '\n') {
                ++index;
            }
            continue;
        }
        if (source[index] == '/' && index + 1 < source.size() && source[index + 1] == '*') {
            index += 2;
            const std::size_t end = source.find("*/", index);
            if (end == std::string_view::npos) {
                throw std::runtime_error("unterminated block comment");
            }
            index = end + 2;
            continue;
        }
        if (source[index] == '"') {
            ++index;
            std::string value;
            bool closed = false;
            while (index < source.size()) {
                const char character = source[index++];
                if (character == '"') {
                    closed = true;
                    break;
                }
                if (character == '\\' && index < source.size()) {
                    value.push_back(source[index++]);
                } else {
                    value.push_back(character);
                }
            }
            if (!closed) {
                throw std::runtime_error("unterminated quoted string");
            }
            tokens.push_back(std::move(value));
            continue;
        }
        if (source[index] == '{' || source[index] == '}' || source[index] == ';' ||
            source[index] == ',' || source[index] == '<' || source[index] == '>') {
            tokens.emplace_back(1, source[index++]);
            continue;
        }
        const std::size_t start = index;
        while (index < source.size() &&
               std::isspace(static_cast<unsigned char>(source[index])) == 0 &&
               source[index] != '{' && source[index] != '}' && source[index] != ';' &&
               source[index] != ',' && source[index] != '<' && source[index] != '>') {
            if (source[index] == '/' && index + 1 < source.size() &&
                (source[index + 1] == '/' || source[index + 1] == '*')) {
                break;
            }
            ++index;
        }
        if (index == start) {
            continue;
        }
        tokens.emplace_back(source.substr(start, index - start));
    }
    return tokens;
}

class Parser {
  public:
    explicit Parser(std::vector<std::string> tokens) : tokens_(std::move(tokens)) {}

    std::shared_ptr<ParsedObj> parse() {
        auto result = std::make_shared<ParsedObj>();
        while (!atEnd()) {
            const std::string token = take();
            if (token == "Frame") {
                parseFrame(identityMatrix, *result);
            } else if (token == "Mesh") {
                parseAndAppendMesh(identityMatrix, *result);
            } else if (token == "template") {
                skipTemplate();
            } else if (token == "}") {
                continue;
            } else {
                skipUnknownBlockIfPresent();
            }
        }
        if (result->positions.empty() || result->triangles.empty()) {
            throw std::runtime_error("file contains no static mesh triangles");
        }
        return result;
    }

  private:
    bool atEnd() const {
        return cursor_ >= tokens_.size();
    }

    const std::string& peek() const {
        if (atEnd()) {
            throw std::runtime_error("unexpected end of file");
        }
        return tokens_[cursor_];
    }

    std::string take() {
        const std::string value = peek();
        ++cursor_;
        return value;
    }

    bool accept(std::string_view expected) {
        if (!atEnd() && tokens_[cursor_] == expected) {
            ++cursor_;
            return true;
        }
        return false;
    }

    void expect(std::string_view expected) {
        if (!accept(expected)) {
            throw std::runtime_error("expected '" + std::string(expected) + "'");
        }
    }

    void skipSeparators() {
        while (!atEnd() && (tokens_[cursor_] == ";" || tokens_[cursor_] == ",")) {
            ++cursor_;
        }
    }

    double readNumber() {
        skipSeparators();
        const std::string text = take();
        char* end = nullptr;
        const double value = std::strtod(text.c_str(), &end);
        if (end == text.c_str() || *end != '\0' || !std::isfinite(value)) {
            throw std::runtime_error("expected a finite number, found '" + text + "'");
        }
        skipSeparators();
        return value;
    }

    std::size_t readCount() {
        const double value = readNumber();
        if (value < 0.0 || value > 10000000.0 || std::floor(value) != value) {
            throw std::runtime_error("invalid array count");
        }
        return static_cast<std::size_t>(value);
    }

    ObjPosition readVector3() {
        return {readNumber(), readNumber(), readNumber()};
    }

    Matrix readMatrix() {
        expect("{");
        Matrix matrix = {};
        for (double& value : matrix) {
            value = readNumber();
        }
        expect("}");
        skipSeparators();
        return matrix;
    }

    static Matrix multiply(const Matrix& left, const Matrix& right) {
        Matrix result = {};
        for (std::size_t row = 0; row < 4; ++row) {
            for (std::size_t column = 0; column < 4; ++column) {
                for (std::size_t inner = 0; inner < 4; ++inner) {
                    result[row * 4 + column] += left[row * 4 + inner] * right[inner * 4 + column];
                }
            }
        }
        return result;
    }

    static ObjPosition transformPosition(const Matrix& matrix, const ObjPosition& position) {
        return {
            position.x * matrix[0] + position.y * matrix[4] + position.z * matrix[8] + matrix[12],
            position.x * matrix[1] + position.y * matrix[5] + position.z * matrix[9] + matrix[13],
            position.x * matrix[2] + position.y * matrix[6] + position.z * matrix[10] + matrix[14]};
    }

    static ObjNormal transformNormal(const Matrix& matrix, const ObjNormal& normal) {
        const double a = matrix[0], b = matrix[1], c = matrix[2];
        const double d = matrix[4], e = matrix[5], f = matrix[6];
        const double g = matrix[8], h = matrix[9], i = matrix[10];
        const double determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
        if (std::abs(determinant) < 1.0e-12) {
            throw std::runtime_error("singular frame transform");
        }
        // For row-vector positions, normals use the inverse transpose.
        const std::array<double, 9> inverse = {
            (e * i - f * h) / determinant, (c * h - b * i) / determinant,
            (b * f - c * e) / determinant, (f * g - d * i) / determinant,
            (a * i - c * g) / determinant, (c * d - a * f) / determinant,
            (d * h - e * g) / determinant, (b * g - a * h) / determinant,
            (a * e - b * d) / determinant};
        ObjNormal transformed = {
            normal.x * inverse[0] + normal.y * inverse[3] + normal.z * inverse[6],
            normal.x * inverse[1] + normal.y * inverse[4] + normal.z * inverse[7],
            normal.x * inverse[2] + normal.y * inverse[5] + normal.z * inverse[8]};
        const double length =
            std::sqrt(transformed.x * transformed.x + transformed.y * transformed.y +
                      transformed.z * transformed.z);
        if (length > 1.0e-12) {
            transformed.x /= length;
            transformed.y /= length;
            transformed.z /= length;
        }
        // Match the established OBJ/O3D convention for the map renderer's
        // reflected map-axis basis and GL_CW front-face convention.
        return {-transformed.x, -transformed.z, -transformed.y};
    }

    void skipBalancedBlock() {
        expect("{");
        std::size_t depth = 1;
        while (depth > 0) {
            const std::string token = take();
            if (token == "{") {
                ++depth;
            } else if (token == "}") {
                --depth;
            }
        }
    }

    void skipUnknownBlockIfPresent() {
        if (!atEnd() && tokens_[cursor_] != "{" && tokens_[cursor_] != "}" &&
            tokens_[cursor_] != ";" && tokens_[cursor_] != ",") {
            ++cursor_;
        }
        if (!atEnd() && tokens_[cursor_] == "{") {
            skipBalancedBlock();
        }
    }

    void skipTemplate() {
        while (!atEnd() && tokens_[cursor_] != "{") {
            ++cursor_;
        }
        if (!atEnd()) {
            skipBalancedBlock();
        }
    }

    void parseFrame(const Matrix& parentTransform, ParsedObj& output) {
        if (!atEnd() && peek() != "{") {
            take(); // Optional frame name.
        }
        expect("{");
        Matrix frameTransform = identityMatrix;
        while (!atEnd() && peek() != "}") {
            const std::string token = take();
            if (token == "FrameTransformMatrix") {
                frameTransform = readMatrix();
            } else if (token == "Frame") {
                parseFrame(multiply(frameTransform, parentTransform), output);
            } else if (token == "Mesh") {
                parseAndAppendMesh(multiply(frameTransform, parentTransform), output);
            } else {
                skipUnknownBlockIfPresent();
            }
        }
        expect("}");
    }

    void parseAndAppendMesh(const Matrix& transform, ParsedObj& output) {
        MeshData mesh;
        if (!atEnd() && peek() != "{") {
            take(); // Optional mesh name.
        }
        expect("{");
        const std::size_t positionCount = readCount();
        mesh.positions.reserve(positionCount);
        for (std::size_t index = 0; index < positionCount; ++index) {
            mesh.positions.push_back(readVector3());
        }
        const std::size_t faceCount = readCount();
        mesh.faces.reserve(faceCount);
        for (std::size_t face = 0; face < faceCount; ++face) {
            const std::size_t cornerCount = readCount();
            if (cornerCount < 3 || cornerCount > positionCount) {
                throw std::runtime_error("invalid polygon corner count");
            }
            std::vector<int> indices;
            indices.reserve(cornerCount);
            for (std::size_t corner = 0; corner < cornerCount; ++corner) {
                const std::size_t index = readCount();
                if (index >= positionCount) {
                    throw std::runtime_error("mesh face index is out of range");
                }
                indices.push_back(static_cast<int>(index));
            }
            mesh.faces.push_back(std::move(indices));
        }
        while (!atEnd() && peek() != "}") {
            const std::string token = take();
            if (token == "MeshNormals") {
                parseNormals(mesh);
            } else if (token == "MeshTextureCoords") {
                parseTextureCoordinates(mesh);
            } else if (token == "MeshMaterialList") {
                parseMaterials(mesh);
            } else {
                skipUnknownBlockIfPresent();
            }
        }
        expect("}");
        appendMesh(mesh, transform, output);
    }

    void parseNormals(MeshData& mesh) {
        expect("{");
        const std::size_t count = readCount();
        mesh.normals.clear();
        mesh.normals.reserve(count);
        for (std::size_t index = 0; index < count; ++index) {
            mesh.normals.push_back(readVector3());
        }
        const std::size_t faceCount = readCount();
        mesh.normalFaces.clear();
        mesh.normalFaces.reserve(faceCount);
        for (std::size_t face = 0; face < faceCount; ++face) {
            const std::size_t cornerCount = readCount();
            if (cornerCount > mesh.positions.size()) {
                throw std::runtime_error("invalid normal polygon corner count");
            }
            std::vector<int> indices;
            indices.reserve(cornerCount);
            for (std::size_t corner = 0; corner < cornerCount; ++corner) {
                const std::size_t index = readCount();
                if (index >= count) {
                    throw std::runtime_error("mesh normal index is out of range");
                }
                indices.push_back(static_cast<int>(index));
            }
            mesh.normalFaces.push_back(std::move(indices));
        }
        expect("}");
    }

    void parseTextureCoordinates(MeshData& mesh) {
        expect("{");
        const std::size_t count = readCount();
        mesh.texCoords.clear();
        mesh.texCoords.reserve(count);
        for (std::size_t index = 0; index < count; ++index) {
            mesh.texCoords.push_back({readNumber(), readNumber()});
        }
        expect("}");
    }

    ObjMaterial parseMaterial() {
        if (!atEnd() && peek() != "{") {
            take(); // Optional material name.
        }
        expect("{");
        ObjMaterial material;
        material.color = {readNumber(), readNumber(), readNumber()};
        material.alpha = std::clamp(readNumber(), 0.0, 1.0);
        material.specularPower = readNumber();
        material.specular = {readNumber(), readNumber(), readNumber()};
        material.emission = {readNumber(), readNumber(), readNumber()};
        while (!atEnd() && peek() != "}") {
            const std::string token = take();
            if (token == "TextureFilename") {
                expect("{");
                material.textureName = take();
                std::replace(material.textureName.begin(), material.textureName.end(), '\\', '/');
                while (!atEnd() && peek() != "}") {
                    take();
                }
                expect("}");
                skipSeparators();
            } else {
                skipUnknownBlockIfPresent();
            }
        }
        expect("}");
        return material;
    }

    void parseMaterials(MeshData& mesh) {
        expect("{");
        const std::size_t materialCount = readCount();
        const std::size_t faceCount = readCount();
        if (faceCount != mesh.faces.size()) {
            throw std::runtime_error("material face count does not match mesh face count");
        }
        mesh.faceMaterials.clear();
        mesh.faceMaterials.reserve(faceCount);
        for (std::size_t face = 0; face < faceCount; ++face) {
            const std::size_t index = readCount();
            if (index >= materialCount && materialCount != 0) {
                throw std::runtime_error("mesh material index is out of range");
            }
            mesh.faceMaterials.push_back(static_cast<int>(index));
        }
        mesh.materials.clear();
        mesh.materials.reserve(std::max<std::size_t>(materialCount, 1));
        for (std::size_t index = 0; index < materialCount; ++index) {
            if (atEnd() || peek() != "Material") {
                throw std::runtime_error("named/external X materials are unsupported");
            }
            take();
            mesh.materials.push_back(parseMaterial());
        }
        if (materialCount == 0) {
            mesh.materials.emplace_back();
        }
        expect("}");
    }

    void appendMesh(const MeshData& mesh, const Matrix& transform, ParsedObj& output) {
        const std::size_t positionBase = output.positions.size();
        const std::size_t texCoordBase = output.texCoords.size();
        const std::size_t normalBase = output.normals.size();
        const int materialBase = static_cast<int>(output.materials.size());
        std::vector<ObjPosition> transformedPositions;
        transformedPositions.reserve(mesh.positions.size());
        for (const ObjPosition& position : mesh.positions) {
            const ObjPosition transformed = transformPosition(transform, position);
            transformedPositions.push_back({transformed.x, transformed.z, transformed.y});
            output.positions.push_back(transformedPositions.back());
        }
        for (const ObjTexCoord& texCoord : mesh.texCoords) {
            output.texCoords.push_back(texCoord);
        }
        for (const ObjNormal& normal : mesh.normals) {
            output.normals.push_back(transformNormal(transform, normal));
        }
        const std::vector<ObjMaterial> materials =
            mesh.materials.empty() ? std::vector<ObjMaterial>{ObjMaterial{}} : mesh.materials;
        for (std::size_t index = 0; index < materials.size(); ++index) {
            ObjMaterial material = materials[index];
            material.materialIndex = materialBase + static_cast<int>(index);
            output.materials.emplace("matl_" + std::to_string(material.materialIndex),
                                     std::move(material));
        }
        for (std::size_t faceIndex = 0; faceIndex < mesh.faces.size(); ++faceIndex) {
            const std::vector<int>& face = mesh.faces[faceIndex];
            const int localMaterial =
                faceIndex < mesh.faceMaterials.size() ? mesh.faceMaterials[faceIndex] : 0;
            const std::string materialName = "matl_" + std::to_string(materialBase + localMaterial);
            for (std::size_t corner = 2; corner < face.size(); ++corner) {
                const std::array<std::size_t, 3> corners = {0, corner - 1, corner};
                ObjTriangle triangle;
                triangle.material = materialName;
                for (std::size_t vertex = 0; vertex < corners.size(); ++vertex) {
                    const std::size_t faceCorner = corners[vertex];
                    const std::size_t positionIndex = static_cast<std::size_t>(face[faceCorner]);
                    int normalIndex = 0;
                    if (faceIndex < mesh.normalFaces.size() &&
                        mesh.normalFaces[faceIndex].size() == face.size()) {
                        normalIndex = static_cast<int>(normalBase) +
                                      mesh.normalFaces[faceIndex][faceCorner] + 1;
                    } else if (!mesh.normals.empty() && positionIndex < mesh.normals.size()) {
                        normalIndex = static_cast<int>(normalBase + positionIndex + 1);
                    }
                    int texCoordIndex = 0;
                    if (positionIndex < mesh.texCoords.size()) {
                        texCoordIndex = static_cast<int>(texCoordBase + positionIndex + 1);
                    }
                    triangle.indices[vertex] = {static_cast<int>(positionBase + positionIndex + 1),
                                                texCoordIndex, normalIndex};
                }
                if (triangle.indices[0].normal == 0 || triangle.indices[1].normal == 0 ||
                    triangle.indices[2].normal == 0) {
                    const ObjPosition& first = transformedPositions[corners[0]];
                    const ObjPosition& second = transformedPositions[corners[1]];
                    const ObjPosition& third = transformedPositions[corners[2]];
                    const double ax = second.x - first.x, ay = second.y - first.y,
                                 az = second.z - first.z;
                    const double bx = third.x - first.x, by = third.y - first.y,
                                 bz = third.z - first.z;
                    ObjNormal generated = {az * by - ay * bz, ax * bz - az * bx, ay * bx - ax * by};
                    const double length =
                        std::sqrt(generated.x * generated.x + generated.y * generated.y +
                                  generated.z * generated.z);
                    if (length > 1.0e-12) {
                        generated.x /= length;
                        generated.y /= length;
                        generated.z /= length;
                    }
                    output.normals.push_back(generated);
                    const int generatedIndex = static_cast<int>(output.normals.size());
                    for (ObjIndex& index : triangle.indices) {
                        if (index.normal == 0) {
                            index.normal = generatedIndex;
                        }
                    }
                }
                output.triangles.push_back(std::move(triangle));
            }
        }
    }

    std::vector<std::string> tokens_;
    std::size_t cursor_ = 0;
};

} // namespace

std::shared_ptr<ParsedObj> XLoader::parse(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return {};
    }
    const std::string contents((std::istreambuf_iterator<char>(input)),
                               std::istreambuf_iterator<char>());
    const std::size_t headerEnd = contents.find_first_of("\r\n");
    const std::string_view header(contents.data(),
                                  headerEnd == std::string::npos ? contents.size() : headerEnd);
    std::string normalizedHeader(header);
    std::transform(normalizedHeader.begin(), normalizedHeader.end(), normalizedHeader.begin(),
                   [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    if (normalizedHeader.rfind("xof ", 0) != 0 ||
        normalizedHeader.find("txt") == std::string::npos) {
        return {};
    }
    try {
        const std::size_t contentStart =
            headerEnd == std::string::npos ? contents.size() : headerEnd + 1;
        return Parser(tokenize(std::string_view(contents).substr(contentStart))).parse();
    } catch (const std::exception&) {
        return {};
    }
}

} // namespace openbus::rendering