#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace {

constexpr uint8_t SECTION_VERTEX_LIST = 0x17;
constexpr uint8_t SECTION_TRIANGLE_LIST = 0x49;
constexpr uint8_t SECTION_MATERIAL_LIST = 0x26;
constexpr uint8_t SECTION_BONE_LIST = 0x54;
constexpr uint8_t SECTION_TRANSFORM = 0x79;

struct ParseError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

struct Vertex {
    std::array<float, 3> pos{};
    std::array<float, 3> normal{};
    std::array<float, 2> uv{};
};

struct Triangle {
    std::array<uint32_t, 3> indices{};
    uint16_t materialId = 0;
};

struct Material {
    std::array<float, 4> diffuse{};
    std::array<float, 3> specular{};
    std::array<float, 3> emission{};
    float specularPower = 1.0f;
    std::string textureName;
};

struct BoneWeight {
    uint32_t vertexIndex = 0;
    float weight = 0.0f;
};

struct Bone {
    std::string name;
    std::vector<BoneWeight> weights;
};

struct Header {
    uint8_t version = 0;
    bool longHeader = false;
    bool longTriangleIndices = false;
    bool altEncryptionSeed = false;
    bool encrypted = false;
    uint32_t encryptionKey = 0xFFFFFFFFu;
};

struct Mesh {
    Header header;
    std::vector<Vertex> vertices;
    std::vector<Triangle> triangles;
    std::vector<Material> materials;
    std::vector<Bone> bones;
    std::array<float, 16> transform{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
};

class ByteReader {
  public:
    explicit ByteReader(std::vector<uint8_t> data) : m_data(std::move(data)) {}

    size_t remaining() const {
        return m_data.size() - m_offset;
    }

    size_t offset() const {
        return m_offset;
    }

    uint8_t readU8() {
        ensure(1);
        return m_data[m_offset++];
    }

    uint16_t readU16() {
        ensure(2);
        uint16_t v = static_cast<uint16_t>(m_data[m_offset]) |
                     (static_cast<uint16_t>(m_data[m_offset + 1]) << 8);
        m_offset += 2;
        return v;
    }

    uint32_t readU32() {
        ensure(4);
        uint32_t v = static_cast<uint32_t>(m_data[m_offset]) |
                     (static_cast<uint32_t>(m_data[m_offset + 1]) << 8) |
                     (static_cast<uint32_t>(m_data[m_offset + 2]) << 16) |
                     (static_cast<uint32_t>(m_data[m_offset + 3]) << 24);
        m_offset += 4;
        return v;
    }

    float readF32() {
        const uint32_t raw = readU32();
        float out = 0.0f;
        static_assert(sizeof(float) == sizeof(uint32_t), "Unexpected float size");
        std::memcpy(&out, &raw, sizeof(float));
        return out;
    }

    std::string readPascalStringCp1252() {
        const uint8_t len = readU8();
        ensure(len);
        std::string s;
        s.reserve(len);
        for (uint8_t i = 0; i < len; ++i) {
            s.push_back(static_cast<char>(m_data[m_offset + i]));
        }
        m_offset += len;
        return s;
    }

  private:
    void ensure(size_t n) const {
        if (m_offset + n > m_data.size()) {
            throw ParseError("Unexpected end-of-file while parsing O3D stream.");
        }
    }

    std::vector<uint8_t> m_data;
    size_t m_offset = 0;
};

uint32_t readCount(ByteReader& r, bool longHeader) {
    return longHeader ? r.readU32() : static_cast<uint32_t>(r.readU16());
}

std::vector<Vertex> parseVertexList(ByteReader& r, bool longHeader) {
    const uint32_t count = readCount(r, longHeader);
    std::vector<Vertex> out;
    out.reserve(count);

    for (uint32_t i = 0; i < count; ++i) {
        Vertex v;
        v.pos[0] = r.readF32();
        v.pos[1] = r.readF32();
        v.pos[2] = r.readF32();
        v.normal[0] = r.readF32();
        v.normal[1] = r.readF32();
        v.normal[2] = r.readF32();
        v.uv[0] = r.readF32();
        v.uv[1] = r.readF32();
        out.push_back(v);
    }

    return out;
}

std::vector<Triangle> parseTriangleList(ByteReader& r, bool longHeader, bool longTriangleIndices) {
    const uint32_t count = readCount(r, longHeader);
    std::vector<Triangle> out;
    out.reserve(count);

    for (uint32_t i = 0; i < count; ++i) {
        Triangle t;
        if (longTriangleIndices) {
            t.indices[0] = r.readU32();
            t.indices[1] = r.readU32();
            t.indices[2] = r.readU32();
            t.materialId = r.readU16();
        } else {
            t.indices[0] = r.readU16();
            t.indices[1] = r.readU16();
            t.indices[2] = r.readU16();
            t.materialId = r.readU16();
        }
        out.push_back(t);
    }

    return out;
}

std::vector<Material> parseMaterialList(ByteReader& r) {
    const uint16_t count = r.readU16();
    std::vector<Material> out;
    out.reserve(count);

    for (uint16_t i = 0; i < count; ++i) {
        Material m;
        m.diffuse[0] = r.readF32();
        m.diffuse[1] = r.readF32();
        m.diffuse[2] = r.readF32();
        m.diffuse[3] = r.readF32();

        m.specular[0] = r.readF32();
        m.specular[1] = r.readF32();
        m.specular[2] = r.readF32();

        m.emission[0] = r.readF32();
        m.emission[1] = r.readF32();
        m.emission[2] = r.readF32();

        m.specularPower = r.readF32();
        m.textureName = r.readPascalStringCp1252();
        out.push_back(std::move(m));
    }

    return out;
}

std::vector<Bone> parseBoneList(ByteReader& r, bool longTriangleIndices) {
    const uint16_t count = r.readU16();
    std::vector<Bone> out;
    out.reserve(count);

    for (uint16_t i = 0; i < count; ++i) {
        Bone b;
        b.name = r.readPascalStringCp1252();
        const uint16_t weightCount = r.readU16();
        b.weights.reserve(weightCount);
        for (uint16_t w = 0; w < weightCount; ++w) {
            BoneWeight bw;
            bw.vertexIndex = longTriangleIndices ? r.readU32() : static_cast<uint32_t>(r.readU16());
            bw.weight = r.readF32();
            b.weights.push_back(bw);
        }
        out.push_back(std::move(b));
    }

    return out;
}

std::array<float, 16> parseTransform(ByteReader& r) {
    std::array<float, 16> m{};
    for (float& v : m) {
        v = r.readF32();
    }
    return m;
}

Mesh parseO3D(const fs::path& input) {
    std::ifstream in(input, std::ios::binary);
    if (!in) {
        throw ParseError("Failed to open input file: " + input.string());
    }

    std::vector<uint8_t> data((std::istreambuf_iterator<char>(in)),
                              std::istreambuf_iterator<char>());
    if (data.size() < 3) {
        throw ParseError("Input too small to be a valid O3D file: " + input.string());
    }

    ByteReader r(std::move(data));

    const uint8_t magic0 = r.readU8();
    const uint8_t magic1 = r.readU8();
    const uint8_t version = r.readU8();

    if (magic0 != 0x84 || magic1 != 0x19) {
        throw ParseError("Invalid O3D magic in file: " + input.string());
    }

    Mesh mesh;
    mesh.header.version = version;
    mesh.header.longHeader = (version > 3);

    if (mesh.header.longHeader) {
        const uint8_t options = r.readU8();
        mesh.header.encryptionKey = r.readU32();
        mesh.header.longTriangleIndices = (options & 0x1u) != 0;
        mesh.header.altEncryptionSeed = (options & 0x2u) != 0;
        mesh.header.encrypted = mesh.header.encryptionKey != 0xFFFFFFFFu;
    }

    if (mesh.header.encrypted) {
        throw ParseError("Encrypted O3D detected. Public converter cannot decrypt "
                         "encrypted files.");
    }

    while (r.remaining() > 0) {
        const uint8_t section = r.readU8();
        switch (section) {
        case SECTION_VERTEX_LIST:
            mesh.vertices = parseVertexList(r, mesh.header.longHeader);
            break;
        case SECTION_TRIANGLE_LIST:
            mesh.triangles =
                parseTriangleList(r, mesh.header.longHeader, mesh.header.longTriangleIndices);
            break;
        case SECTION_MATERIAL_LIST:
            mesh.materials = parseMaterialList(r);
            break;
        case SECTION_BONE_LIST:
            mesh.bones = parseBoneList(r, mesh.header.longTriangleIndices);
            break;
        case SECTION_TRANSFORM:
            mesh.transform = parseTransform(r);
            break;
        default:
            throw ParseError("Unexpected section byte 0x" +
                             std::to_string(static_cast<unsigned int>(section)) + " at offset 0x" +
                             std::to_string(static_cast<unsigned long long>(r.offset() - 1)));
        }
    }

    return mesh;
}

std::string sanitizeMaterialName(const std::string& input, size_t fallbackIndex) {
    std::string base = input;
    if (!base.empty()) {
        const fs::path p(base);
        base = p.stem().string();
    }
    if (base.empty()) {
        base = "mat_" + std::to_string(fallbackIndex);
    }

    for (char& c : base) {
        const bool ok =
            std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' || c == '.';
        if (!ok)
            c = '_';
    }

    return base;
}

std::string trim(std::string s) {
    auto isWs = [](unsigned char c) { return std::isspace(c) != 0; };
    while (!s.empty() && isWs(static_cast<unsigned char>(s.front())))
        s.erase(s.begin());
    while (!s.empty() && isWs(static_cast<unsigned char>(s.back())))
        s.pop_back();
    return s;
}

std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

fs::path normalizeOmsiPath(const std::string& in) {
    std::string s = in;
    std::replace(s.begin(), s.end(), '\\', '/');
    return fs::path(s);
}

std::string normalizeTextureKey(const std::string& in) {
    std::string s = normalizeOmsiPath(trim(in)).generic_string();
    std::replace(s.begin(), s.end(), '\\', '/');
    return toLower(s);
}

std::string textureFileKey(const std::string& in) {
    return toLower(normalizeOmsiPath(trim(in)).filename().generic_string());
}

int findMaterialIdByTextureOccurrence(const Mesh& mesh, const std::string& textureRef,
                                      int occurrence) {
    if (textureRef.empty() || occurrence < 0) {
        return -1;
    }

    const std::string wantedFull = normalizeTextureKey(textureRef);
    const std::string wantedFile = textureFileKey(textureRef);

    std::vector<int> matches;
    matches.reserve(mesh.materials.size());
    for (size_t i = 0; i < mesh.materials.size(); ++i) {
        const std::string matFull = normalizeTextureKey(mesh.materials[i].textureName);
        const std::string matFile = textureFileKey(mesh.materials[i].textureName);
        if (!wantedFull.empty() && matFull == wantedFull) {
            matches.push_back(static_cast<int>(i));
            continue;
        }
        if (!wantedFile.empty() && matFile == wantedFile) {
            matches.push_back(static_cast<int>(i));
            continue;
        }
    }

    if (occurrence >= static_cast<int>(matches.size())) {
        return -1;
    }

    return matches[occurrence];
}

std::vector<int> findAllMaterialIdsByTexture(const Mesh& mesh, const std::string& textureRef) {
    std::vector<int> out;
    if (textureRef.empty()) {
        return out;
    }

    const std::string wantedFull = normalizeTextureKey(textureRef);
    const std::string wantedFile = textureFileKey(textureRef);

    for (size_t i = 0; i < mesh.materials.size(); ++i) {
        const std::string matFull = normalizeTextureKey(mesh.materials[i].textureName);
        const std::string matFile = textureFileKey(mesh.materials[i].textureName);
        if ((!wantedFull.empty() && matFull == wantedFull) ||
            (!wantedFile.empty() && matFile == wantedFile)) {
            out.push_back(static_cast<int>(i));
        }
    }

    return out;
}

std::optional<fs::path> tryResolveTexture(const std::string& textureRef, const fs::path& cfgDir,
                                          const fs::path& meshDir) {
    if (textureRef.empty()) {
        return std::nullopt;
    }

    const fs::path tex = normalizeOmsiPath(textureRef);
    std::vector<fs::path> candidates;

    if (tex.is_absolute()) {
        candidates.push_back(tex);
    }

    candidates.push_back(meshDir / tex);
    candidates.push_back(cfgDir / tex);
    candidates.push_back(cfgDir.parent_path() / tex);

    const fs::path fileOnly = tex.filename();
    candidates.push_back(cfgDir / fileOnly);
    candidates.push_back(cfgDir.parent_path() / fileOnly);
    candidates.push_back(cfgDir.parent_path() / "Texture" / fileOnly);
    candidates.push_back(cfgDir / "Texture" / fileOnly);
    candidates.push_back(meshDir / "Texture" / fileOnly);
    candidates.push_back(meshDir.parent_path() / "Texture" / fileOnly);

    for (const auto& p : candidates) {
        std::error_code ec;
        if (fs::exists(p, ec) && fs::is_regular_file(p, ec)) {
            return p;
        }
    }

    return std::nullopt;
}

void writeObjMtl(const Mesh& mesh, const fs::path& outObj, bool flipWinding,
                 const std::unordered_map<int, std::string>* materialOverrides = nullptr,
                 const std::unordered_map<int, float>* opacityOverrides = nullptr,
                 const std::unordered_set<int>* hiddenMaterialIds = nullptr,
                 const fs::path* cfgDir = nullptr, const fs::path* meshSourceDir = nullptr,
                 bool copyTextures = false, bool convertTextures = false) {
    if (outObj.has_parent_path() && !outObj.parent_path().empty()) {
        fs::create_directories(outObj.parent_path());
    }
    fs::path outMtl = outObj;
    outMtl.replace_extension(".mtl");

    auto textureForMat = [&](size_t index) -> std::string {
        if (materialOverrides) {
            const auto it = materialOverrides->find(static_cast<int>(index));
            if (it != materialOverrides->end() && !it->second.empty()) {
                return it->second;
            }
        }
        if (index < mesh.materials.size()) {
            return mesh.materials[index].textureName;
        }
        return {};
    };

    std::vector<std::string> materialNames;
    materialNames.reserve(mesh.materials.size());
    for (size_t i = 0; i < mesh.materials.size(); ++i) {
        materialNames.push_back(sanitizeMaterialName(textureForMat(i), i));
    }

    uint16_t maxMat = 0;
    for (const auto& t : mesh.triangles) {
        maxMat = std::max(maxMat, t.materialId);
    }
    while (materialNames.size() <= maxMat) {
        materialNames.push_back("mat_" + std::to_string(materialNames.size()));
    }

    {
        std::ofstream mtl(outMtl, std::ios::binary);
        if (!mtl) {
            throw std::runtime_error("Failed to write MTL: " + outMtl.string());
        }

        mtl << "# Generated by uno3d_converter\n";
        for (size_t i = 0; i < materialNames.size(); ++i) {
            mtl << "\nnewmtl " << materialNames[i] << "\n";
            if (i < mesh.materials.size()) {
                const auto& m = mesh.materials[i];
                mtl << "Kd " << m.diffuse[0] << ' ' << m.diffuse[1] << ' ' << m.diffuse[2] << "\n";
                float opacity = m.diffuse[3];
                if (opacityOverrides) {
                    const auto itOpacity = opacityOverrides->find(static_cast<int>(i));
                    if (itOpacity != opacityOverrides->end()) {
                        opacity = itOpacity->second;
                    }
                }
                mtl << "d " << opacity << "\n";
                mtl << "Ks " << m.specular[0] << ' ' << m.specular[1] << ' ' << m.specular[2]
                    << "\n";
                mtl << "Ns " << std::max(1.0f, m.specularPower) << "\n";
                std::string texRef = textureForMat(i);
                if (!texRef.empty()) {
                    std::string mapKdPath = texRef;

                    if (copyTextures && cfgDir && meshSourceDir) {
                        const auto resolved = tryResolveTexture(texRef, *cfgDir, *meshSourceDir);
                        if (resolved.has_value()) {
                            const bool isDds = toLower(resolved->extension().string()) == ".dds";
                            if (!isDds) {
                                const fs::path dstTex = outMtl.parent_path() / resolved->filename();
                                std::error_code ec;
                                fs::create_directories(dstTex.parent_path(), ec);
                                fs::copy_file(*resolved, dstTex,
                                              fs::copy_options::overwrite_existing, ec);
                                mapKdPath = dstTex.filename().generic_string();
                            }
                        }
                    }

                    std::replace(mapKdPath.begin(), mapKdPath.end(), '\\', '/');
                    if (convertTextures) {
                        const fs::path texPath(mapKdPath);
                        mapKdPath =
                            (texPath.parent_path() / texPath.stem()).generic_string() + ".png";
                    }
                    mtl << "map_Kd " << mapKdPath << "\n";
                }
            } else {
                mtl << "Kd 0.8 0.8 0.8\n";
            }
        }
    }

    {
        std::ofstream obj(outObj, std::ios::binary);
        if (!obj) {
            throw std::runtime_error("Failed to write OBJ: " + outObj.string());
        }

        obj << "# Generated by uno3d_converter\n";
        obj << "mtllib " << outMtl.filename().string() << "\n";

        for (const auto& v : mesh.vertices) {
            obj << "v " << v.pos[0] << ' ' << v.pos[1] << ' ' << v.pos[2] << "\n";
        }
        for (const auto& v : mesh.vertices) {
            obj << "vt " << v.uv[0] << ' ' << (1.0f - v.uv[1]) << "\n";
        }
        for (const auto& v : mesh.vertices) {
            obj << "vn " << v.normal[0] << ' ' << v.normal[1] << ' ' << v.normal[2] << "\n";
        }

        int currentMat = -1;
        for (const auto& t : mesh.triangles) {
            if (hiddenMaterialIds && hiddenMaterialIds->count(static_cast<int>(t.materialId)) > 0) {
                continue;
            }

            if (static_cast<int>(t.materialId) != currentMat) {
                currentMat = static_cast<int>(t.materialId);
                const std::string matName = (t.materialId < materialNames.size())
                                                ? materialNames[t.materialId]
                                                : std::string("mat_default");
                obj << "usemtl " << matName << "\n";
            }

            uint32_t i0 = t.indices[0] + 1;
            uint32_t i1 = t.indices[1] + 1;
            uint32_t i2 = t.indices[2] + 1;
            if (flipWinding)
                std::swap(i1, i2);

            obj << "f " << i0 << '/' << i0 << '/' << i0 << ' ' << i1 << '/' << i1 << '/' << i1
                << ' ' << i2 << '/' << i2 << '/' << i2 << "\n";
        }
    }
}

bool isO3DExt(const fs::path& p) {
    const std::string ext = toLower(p.extension().string());
    return ext == ".o3d";
}

bool isCfgExt(const fs::path& p) {
    const std::string ext = toLower(p.extension().string());
    return ext == ".cfg";
}

std::vector<fs::path> collectO3DInDirectory(const fs::path& input, bool recursive) {
    std::vector<fs::path> out;
    if (!fs::is_directory(input)) {
        return out;
    }

    if (recursive) {
        for (const auto& e : fs::recursive_directory_iterator(input)) {
            if (e.is_regular_file() && isO3DExt(e.path())) {
                out.push_back(e.path());
            }
        }
    } else {
        for (const auto& e : fs::directory_iterator(input)) {
            if (e.is_regular_file() && isO3DExt(e.path())) {
                out.push_back(e.path());
            }
        }
    }

    return out;
}

std::vector<fs::path> collectCfgInDirectory(const fs::path& input, bool recursive) {
    std::vector<fs::path> out;
    if (!fs::is_directory(input)) {
        return out;
    }

    if (recursive) {
        for (const auto& e : fs::recursive_directory_iterator(input)) {
            if (e.is_regular_file() && isCfgExt(e.path())) {
                out.push_back(e.path());
            }
        }
    } else {
        for (const auto& e : fs::directory_iterator(input)) {
            if (e.is_regular_file() && isCfgExt(e.path())) {
                out.push_back(e.path());
            }
        }
    }

    return out;
}

struct CfgMeshEntry {
    fs::path meshPath;
    struct MaterialSelector {
        std::string textureRef;
        int occurrence = 0;
        int alphaMode = -1; // OMSI: 0=no transparency, 1=full transparency,
                            // 2=partial transparency
        bool hideFromAlphaScale = false;
        bool noZWrite = false;
    };
    std::vector<MaterialSelector> materialSelectors;
};

struct ParsedCfg {
    std::vector<CfgMeshEntry> meshes;
};

ParsedCfg parseCfg(const fs::path& cfgPath) {
    std::ifstream in(cfgPath);
    if (!in) {
        throw std::runtime_error("Failed to open CFG: " + cfgPath.string());
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        lines.push_back(line);
    }

    if (!lines.empty() && lines[0].size() >= 3 && static_cast<unsigned char>(lines[0][0]) == 0xEF &&
        static_cast<unsigned char>(lines[0][1]) == 0xBB &&
        static_cast<unsigned char>(lines[0][2]) == 0xBF) {
        lines[0].erase(0, 3);
    }

    ParsedCfg out;
    CfgMeshEntry* currentMesh = nullptr;
    CfgMeshEntry::MaterialSelector* currentSelector = nullptr;

    for (size_t i = 0; i < lines.size(); ++i) {
        const std::string t = trim(lines[i]);
        if (t.empty()) {
            continue;
        }
        if (t.size() < 2 || t.front() != '[' || t.back() != ']') {
            continue;
        }

        const std::string cmd = toLower(t);
        if (cmd == "[mesh]") {
            if (i + 1 >= lines.size()) {
                continue;
            }

            const std::string meshRel = trim(lines[++i]);
            if (meshRel.empty()) {
                continue;
            }

            out.meshes.push_back(CfgMeshEntry{});
            currentMesh = &out.meshes.back();
            currentMesh->meshPath = normalizeOmsiPath(meshRel);
            currentSelector = nullptr;
            continue;
        }

        if (!currentMesh) {
            continue;
        }

        if (cmd == "[matl]" || cmd == "[matl_change]") {
            if (i + 2 >= lines.size()) {
                continue;
            }

            const std::string tex = trim(lines[++i]);
            const std::string indexStr = trim(lines[++i]);
            try {
                const int occurrence = std::stoi(indexStr);
                if (!tex.empty() && occurrence >= 0) {
                    currentMesh->materialSelectors.push_back(CfgMeshEntry::MaterialSelector{});
                    currentSelector = &currentMesh->materialSelectors.back();
                    currentSelector->textureRef = tex;
                    currentSelector->occurrence = occurrence;
                }
            } catch (...) {
                // ignore malformed [matl] section
                currentSelector = nullptr;
            }

            if (cmd == "[matl_change]" && i + 1 < lines.size()) {
                ++i; // consume change var name
            }

            continue;
        }

        if (cmd == "[matl_alpha]") {
            if (i + 1 < lines.size()) {
                const std::string modeStr = trim(lines[++i]);
                if (currentSelector) {
                    try {
                        currentSelector->alphaMode = std::stoi(modeStr);
                    } catch (...) {
                        // ignore malformed alpha mode
                    }
                }
            }
            continue;
        }

        if (cmd == "[matl_nozwrite]") {
            if (currentSelector) {
                currentSelector->noZWrite = true;
            }
            continue;
        }

        if (cmd == "[matl_transmap]") {
            if (i + 1 < lines.size()) {
                ++i; // consume transmap texture path
            }
            continue;
        }

        if (cmd == "[matl_envmap]") {
            if (i + 1 < lines.size()) {
                ++i; // consume envmap texture
            }
            if (i + 1 < lines.size()) {
                ++i; // consume envmap strength
            }
            continue;
        }

        if (cmd == "[alphascale]") {
            if (i + 1 < lines.size()) {
                ++i; // consume alpha variable name
            }

            if (currentSelector) {
                currentSelector->hideFromAlphaScale = true;
            }
            continue;
        }
    }

    return out;
}

int convertO3DSingle(const fs::path& inPath, const fs::path& outObj, bool flipWinding,
                     const std::unordered_map<int, std::string>* materialOverrides = nullptr,
                     const std::unordered_map<int, float>* opacityOverrides = nullptr,
                     const std::unordered_set<int>* hiddenMaterialIds = nullptr,
                     const fs::path* cfgDir = nullptr, const fs::path* meshDir = nullptr,
                     bool copyTextures = false, bool convertTextures = false) {
    Mesh mesh = parseO3D(inPath);
    writeObjMtl(mesh, outObj, flipWinding, materialOverrides, opacityOverrides, hiddenMaterialIds,
                cfgDir, meshDir, copyTextures, convertTextures);
    std::cout << "OK  " << inPath << " -> " << outObj << "\n";
    return 0;
}

fs::path resolveCfgMeshPath(const fs::path& cfgDir, const fs::path& meshPath) {
    const std::vector<fs::path> candidates = {
        cfgDir / meshPath,
        cfgDir.parent_path() / meshPath,
    };
    for (const fs::path& candidate : candidates) {
        std::error_code ec;
        if (fs::exists(candidate, ec) && fs::is_regular_file(candidate, ec)) {
            return candidate;
        }
    }
    return cfgDir / meshPath;
}

int convertCfgSingle(const fs::path& cfgPath, const fs::path& outBaseDir, bool flipWinding,
                     bool convertTextures = false) {
    const fs::path cfgDir = cfgPath.parent_path();
    const ParsedCfg cfg = parseCfg(cfgPath);

    int errors = 0;
    std::set<std::string> converted;
    for (const auto& entry : cfg.meshes) {
        try {
            const fs::path srcMesh = resolveCfgMeshPath(cfgDir, entry.meshPath);
            if (!isO3DExt(srcMesh)) {
                std::cerr << "WARN " << srcMesh << ": non-O3D mesh reference skipped\n";
                continue;
            }

            if (!fs::exists(srcMesh)) {
                std::cerr << "WARN " << srcMesh << ": missing mesh referenced by CFG\n";
                continue;
            }
            const fs::path meshDir = srcMesh.parent_path();

            fs::path outObj = outBaseDir / entry.meshPath;
            outObj.replace_extension(".obj");

            // Avoid repeated conversion of identical source->destination pairs
            const std::string key =
                srcMesh.lexically_normal().string() + "|" + outObj.lexically_normal().string();
            if (!converted.insert(key).second) {
                continue;
            }

            Mesh mesh = parseO3D(srcMesh);

            std::unordered_map<int, std::string> resolvedMaterialOverrides;
            std::unordered_map<int, float> resolvedOpacityOverrides;
            std::unordered_set<int> resolvedHiddenMaterialIds;

            for (const auto& sel : entry.materialSelectors) {
                const int matId =
                    findMaterialIdByTextureOccurrence(mesh, sel.textureRef, sel.occurrence);
                if (matId < 0) {
                    std::cerr << "WARN " << srcMesh << ": [matl] target not found for texture '"
                              << sel.textureRef << "' occurrence " << sel.occurrence << "\n";
                    continue;
                }

                resolvedMaterialOverrides[matId] = sel.textureRef;

                if (sel.alphaMode == 0) {
                    resolvedOpacityOverrides[matId] = 1.0f;
                } else if (sel.alphaMode == 1) {
                    resolvedOpacityOverrides[matId] = 0.0f;
                    resolvedHiddenMaterialIds.insert(matId);
                }

                if (sel.hideFromAlphaScale) {
                    resolvedHiddenMaterialIds.insert(matId);

                    // Robust fallback: hide all material slots that use this texture,
                    // because OMSI [matl] occurrences can vary between assets.
                    for (int id : findAllMaterialIdsByTexture(mesh, sel.textureRef)) {
                        resolvedHiddenMaterialIds.insert(id);
                    }
                }
            }

            writeObjMtl(mesh, outObj, flipWinding, &resolvedMaterialOverrides,
                        &resolvedOpacityOverrides, &resolvedHiddenMaterialIds, &cfgDir, &meshDir,
                        true, convertTextures);
            std::cout << "OK  " << srcMesh << " -> " << outObj << "\n";
        } catch (const std::exception& e) {
            ++errors;
            std::cerr << "ERR " << cfgPath << ": " << e.what() << "\n";
        }
    }

    if (cfg.meshes.empty()) {
        std::cerr << "WARN " << cfgPath << ": no [mesh] entries found\n";
    }

    return errors;
}

struct CliOptions {
    fs::path input;
    std::optional<fs::path> out;
    bool recursive = false;
    bool flipWinding = false;
    bool convertTextures = false;
};

void printUsage(const char* exeName) {
    std::cout << "Usage:\n"
              << "  " << exeName << " <input.o3d> [-o output.obj] [--flip-winding]\n"
              << "  " << exeName << " <input.cfg> [-o output_dir] [--flip-winding]\n"
              << "  " << exeName << " <input_dir>  [-r] [-o output_dir] [--flip-winding]\n\n"
              << "Options:\n"
              << "  -o, --out <path>     Output file for .o3d input, or output "
                 "directory for .cfg/directory input\n"
              << "  -r, --recursive      Recursively scan input directory for "
                 ".o3d files\n"
              << "  --flip-winding       Flip triangle winding in generated OBJ\n"
              << "  --convert-textures   Rewrite all texture references in MTL "
                 "to use .png extension if it not a .dds file\n"
              << "  -h, --help           Show this message\n";
}

std::optional<CliOptions> parseArgs(int argc, char** argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return std::nullopt;
    }

    CliOptions opt;
    bool inputSet = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return std::nullopt;
        }
        if (arg == "-r" || arg == "--recursive") {
            opt.recursive = true;
            continue;
        }
        if (arg == "--flip-winding") {
            opt.flipWinding = true;
            continue;
        }
        if (arg == "--convert-textures") {
            opt.convertTextures = true;
            continue;
        }
        if (arg == "-o" || arg == "--out") {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for --out");
            }
            opt.out = fs::path(argv[++i]);
            continue;
        }
        if (!inputSet) {
            opt.input = fs::path(arg);
            inputSet = true;
            continue;
        }

        throw std::runtime_error("Unexpected argument: " + arg);
    }

    if (!inputSet) {
        throw std::runtime_error("Missing input path.");
    }

    return opt;
}

int run(const CliOptions& opt) {
    if (!fs::exists(opt.input)) {
        std::cerr << "Input does not exist: " << opt.input << "\n";
        return 2;
    }
    int errors = 0;

    if (fs::is_regular_file(opt.input) && isO3DExt(opt.input)) {
        try {
            fs::path outObj;
            if (opt.out.has_value()) {
                outObj = opt.out.value();
            } else {
                outObj = opt.input;
            }
            if (!outObj.has_extension() || toLower(outObj.extension().string()) != ".obj") {
                outObj.replace_extension(".obj");
            }

            errors += convertO3DSingle(opt.input, outObj, opt.flipWinding, nullptr, nullptr,
                                       nullptr, nullptr, nullptr, false, opt.convertTextures);
        } catch (const std::exception& e) {
            ++errors;
            std::cerr << "ERR " << opt.input << ": " << e.what() << "\n";
        }
        return errors == 0 ? 0 : 1;
    }

    if (fs::is_regular_file(opt.input) && isCfgExt(opt.input)) {
        const fs::path outDir =
            opt.out.has_value() ? opt.out.value()
                                : (opt.input.parent_path() / (opt.input.stem().string() + "_obj"));
        errors += convertCfgSingle(opt.input, outDir, opt.flipWinding, opt.convertTextures);
        return errors == 0 ? 0 : 1;
    }

    if (fs::is_directory(opt.input)) {
        const fs::path outDir = opt.out.has_value() ? opt.out.value() : opt.input;

        const auto o3dFiles = collectO3DInDirectory(opt.input, opt.recursive);
        for (const auto& inPath : o3dFiles) {
            try {
                fs::path rel = fs::relative(inPath, opt.input);
                fs::path outObj = outDir / rel;
                outObj.replace_extension(".obj");
                errors += convertO3DSingle(inPath, outObj, opt.flipWinding, nullptr, nullptr,
                                           nullptr, nullptr, nullptr, false, opt.convertTextures);
            } catch (const std::exception& e) {
                ++errors;
                std::cerr << "ERR " << inPath << ": " << e.what() << "\n";
            }
        }

        const auto cfgFiles = collectCfgInDirectory(opt.input, opt.recursive);
        for (const auto& cfgPath : cfgFiles) {
            fs::path cfgRelDir = fs::relative(cfgPath.parent_path(), opt.input);
            fs::path cfgOutDir = outDir / cfgRelDir / (cfgPath.stem().string() + "_obj");
            errors += convertCfgSingle(cfgPath, cfgOutDir, opt.flipWinding, opt.convertTextures);
        }

        if (o3dFiles.empty() && cfgFiles.empty()) {
            std::cerr << "No .o3d or .cfg files found.\n";
            return 1;
        }

        return errors == 0 ? 0 : 1;
    }

    std::cerr << "Unsupported input type: " << opt.input << "\n";
    return 2;
}

} // namespace

int main(int argc, char** argv) {
    try {
        const auto parsed = parseArgs(argc, argv);
        if (!parsed.has_value()) {
            return 0;
        }
        return run(parsed.value());
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 2;
    }
}
