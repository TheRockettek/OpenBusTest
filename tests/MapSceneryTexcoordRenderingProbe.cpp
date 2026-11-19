#include "ProbeTempDirectory.h"
#include "core/Variables.h"
#include "render/AssetRequestManager.h"
#include "render/ComputeCulling.h"
#include "render/CoreRenderer.h"
#include "render/MatrixMath.h"
#include "render/OpenGLFunctions.h"
#include "render/map/MapRenderer.h"
#include "render/map/TreeSeason.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

using namespace openbus::rendering;

namespace {

void check(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void writeText(const std::filesystem::path& path, const std::string& text) {
    std::ofstream output(path, std::ios::binary);
    output << text;
    check(static_cast<bool>(output), "Could not write scenery UV fixture");
}

void writeTexture(const std::filesystem::path& path) {
    std::array<std::uint8_t, 30> image = {};
    image[2] = 2;
    image[12] = image[14] = 2;
    image[16] = 24;
    // Four distinct BGR pixels sampled at the centers of the texture quadrants.
    const std::array<std::uint8_t, 12> pixels = {
        0, 0, 255, 0, 255, 0, 255, 0, 0, 0, 255, 255};
    std::copy(pixels.begin(), pixels.end(), image.begin() + 18);
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(image.data()), image.size());
    check(static_cast<bool>(output), "Could not write scenery UV texture");
}

void writeSolidTexture(const std::filesystem::path& path,
                       const std::array<std::uint8_t, 3>& rgb) {
    std::array<std::uint8_t, 21> image = {};
    image[2] = 2;
    image[12] = image[14] = 1;
    image[16] = 24;
    image[18] = rgb[2];
    image[19] = rgb[1];
    image[20] = rgb[0];
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(image.data()), image.size());
    check(static_cast<bool>(output), "Could not write scenery lightmap texture");
}

void writeRgbaTexture(const std::filesystem::path& path, std::uint8_t alpha) {
    std::array<std::uint8_t, 22> image = {};
    image[2] = 2;
    image[12] = image[14] = 1;
    image[16] = 32;
    image[17] = 8;
    image[18] = 0;
    image[19] = 200;
    image[20] = 255;
    image[21] = alpha;
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(image.data()), image.size());
    check(static_cast<bool>(output), "Could not write scenery cutout texture");
}

void writeSolidBmp(const std::filesystem::path& path,
                   const std::array<std::uint8_t, 3>& rgb) {
    std::array<std::uint8_t, 58> image = {};
    image[0] = 'B';
    image[1] = 'M';
    image[2] = 58;
    image[10] = 54;
    image[14] = 40;
    image[18] = image[22] = 1;
    image[26] = 1;
    image[28] = 24;
    image[34] = 4;
    image[54] = rgb[2];
    image[55] = rgb[1];
    image[56] = rgb[0];
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(image.data()), image.size());
    check(static_cast<bool>(output), "Could not write seasonal BMP");
}

void writeNormalMesh(const std::filesystem::path& path, float normalY, float diffuse = 1.0F) {
    std::vector<std::uint8_t> bytes = {0x84, 0x19, 0x01};
    const auto integer = [&](std::uint32_t value, int width) {
        for (int index = 0; index < width; ++index) {
            bytes.push_back(static_cast<std::uint8_t>(value >> (index * 8)));
        }
    };
    const auto number = [&](float value) { integer(std::bit_cast<std::uint32_t>(value), 4); };
    bytes.push_back(0x17);
    integer(3, 2);
    for (const auto& position : std::array<std::array<float, 3>, 3>{
             {{{-2, 0, -2}}, {{2, 0, -2}}, {{0, 0, 2}}}}) {
        for (const float value : position)
            number(value);
        for (const float value : {0.0F, normalY, 0.0F, 0.5F, 0.5F})
            number(value);
    }
    bytes.push_back(0x49);
    integer(2, 2);
    for (const auto& triangle :
         std::array<std::array<std::uint32_t, 3>, 2>{{{{0, 1, 2}}, {{0, 2, 1}}}}) {
        for (const std::uint32_t index : triangle)
            integer(index, 2);
        integer(0, 2);
    }
    bytes.push_back(0x26);
    integer(1, 2);
    for (const float value : {diffuse, diffuse, diffuse, 1.0F, 0.0F, 0.0F,
                              0.0F, 0.0F, 0.0F, 0.0F, 0.0F})
        number(value);
    bytes.push_back(0);
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(bytes.data()),
                 static_cast<std::streamsize>(bytes.size()));
    check(static_cast<bool>(output), "Could not write O3D normal fixture");
}

void writeHumanMesh(const std::filesystem::path& path) {
    std::vector<std::uint8_t> bytes = {0x84, 0x19, 0x01};
    const auto integer = [&](std::uint32_t value, int width) {
        for (int index = 0; index < width; ++index)
            bytes.push_back(static_cast<std::uint8_t>(value >> (index * 8)));
    };
    const auto number = [&](float value) { integer(std::bit_cast<std::uint32_t>(value), 4); };
    bytes.push_back(0x17);
    integer(3, 2);
    for (const auto& position : std::array<std::array<float, 3>, 3>{
             {{{-0.7F, 0, 0}}, {{0.7F, 0, 0}}, {{0, 1.8F, 0}}}}) {
        for (const float value : position)
            number(value);
        for (const float value : {0.0F, 0.0F, -1.0F, 0.0F, 0.0F})
            number(value);
    }
    bytes.push_back(0x49);
    integer(2, 2);
    for (const auto& triangle :
         std::array<std::array<std::uint32_t, 3>, 2>{{{{0, 1, 2}}, {{0, 2, 1}}}}) {
        for (const auto index : triangle)
            integer(index, 2);
        integer(0, 2);
    }
    bytes.push_back(0x26);
    integer(1, 2);
    for (const float value : {1.0F, 1.0F, 1.0F, 1.0F, 0.0F, 0.0F,
                              0.0F, 0.0F, 0.0F, 0.0F, 0.0F})
        number(value);
    bytes.push_back(0);
    bytes.push_back(0x54);
    integer(1, 2);
    bytes.push_back(4);
    for (const char character : std::string_view("body"))
        bytes.push_back(static_cast<std::uint8_t>(character));
    integer(3, 2);
    for (int index = 0; index < 3; ++index) {
        integer(index, 2);
        number(1.0F);
    }
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(bytes.data()),
                 static_cast<std::streamsize>(bytes.size()));
    check(static_cast<bool>(output), "Could not write skinned human mesh");
}

void runHumanSkinningProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeHumanMesh(root / "model" / "human.o3d");
    writeText(root / "model" / "human.obj",
              "v -0.7 0 0\nv 0.7 0 0\nv 0 1.8 0\n"
              "vn 0 0 -1\nf 1//1 2//1 3//1\nf 3//1 2//1 1//1\n");
    for (const auto& [name, bone] :
         std::array<std::pair<std::string_view, int>, 3>{
             {{"chest", -11}, {"leg", -3}, {"fallback", 0}}}) {
        writeText(root / "model" / (std::string(name) + ".cfg"),
                  "[mesh]\nhuman.o3d\n" +
                      (bone == 0 ? "" : "[setbone]\nbody\n" + std::to_string(bone) + "\n"));
        writeText(root / (std::string(name) + ".hum"),
                  "[model]\nmodel\\" + std::string(name) +
                      ".cfg\n[humangeom]\n0.2\n1.8\n[links]\n"
                      "0.1\n0\n0.9\n0.1\n0\n0.4\n0\n1.0\n"
                      "0.2\n0\n1.4\n0.4\n0\n1.4\n0\n1.6\n"
                      "0.6\n0\n1.4\n0.7\n0\n1.4\n"
                      "[walk_param]\n1.4\n0\n1\n0\n0\n");
    }
    writeText(root / "tile.map", "[version]\n14\n[terrain]\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write human test terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 60);
    lookAt(0, -4, 2, 0, 0, 0.9, 0, 0, 1);
    setLightingViewMatrix(modelViewMatrix());
    const openbus::map::MapTrafficPose pose;
    const auto draw = [&](const std::filesystem::path& path, double phase, double weight) {
        glClearColor(0, 0, 0, 0);
        glDepthMask(GL_TRUE);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        resetFrameTriangleCount();
        renderer.drawHumanTrafficActor(path, pose, phase, weight);
        std::array<std::uint8_t, 64 * 64 * 4> image = {};
        glReadPixels(0, 0, 64, 64, GL_RGBA, GL_UNSIGNED_BYTE, image.data());
        return image;
    };
    const auto awaitModel = [&](const std::filesystem::path& path) {
        renderer.loadHumanTrafficModel(path);
        for (int attempt = 0; attempt < 500; ++attempt) {
            draw(path, 0, 0);
            if (frameTriangleCount() != 0)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        check(frameTriangleCount() != 0, "Asynchronous human model did not render");
    };
    for (const std::string name : {"chest", "leg", "fallback"}) {
        const auto path = root / (name + ".hum");
        awaitModel(path);
        const auto standing = draw(path, 0, 0);
        const auto walking = draw(path, std::numbers::pi / 2, 1);
        check(std::any_of(standing.begin(), standing.end(),
                          [](std::uint8_t value) { return value != 0; }),
              "Human model rendered no visible pixels");
        check(name == "chest" ? standing == walking : standing != walking,
              "Human binding did not select the expected skeletal pose");
    }
    if (const char* path = std::getenv("OPENBUS_HUMAN_SKINNING_TEST_PATH")) {
        awaitModel(path);
        const auto standing = draw(path, 0, 0);
        const auto walking = draw(path, std::numbers::pi / 2, 1);
        check(std::any_of(standing.begin(), standing.end(),
                          [](std::uint8_t value) { return value != 0; }) &&
                  std::any_of(walking.begin(), walking.end(),
                              [](std::uint8_t value) { return value != 0; }),
              "Installed human model was not visible in both poses");
    }
}

void runNormalProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeNormalMesh(root / "model" / "up.o3d", 1.0F);
    writeNormalMesh(root / "model" / "down.o3d", -1.0F);
    writeNormalMesh(root / "model" / "dark.o3d", -1.0F, 0.04F);
    writeText(root / "model" / "up.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshNormals { 1; 0;1;0;; 2; 3;0,0,0;, 3;0,0,0;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 0;0;0;; } } }\n");
    writeText(root / "model" / "dark.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshNormals { 1; 0;-1;0;; 2; 3;0,0,0;, 3;0,0,0;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "0.04;0.04;0.04;1;; 0; 0;0;0;; 0;0;0;; } } }\n");
    writeText(root / "up.sco", "[mesh]\nup.o3d\n");
    writeText(root / "down.sco", "[mesh]\ndown.o3d\n");
    writeText(root / "up-x.sco", "[mesh]\nup.x\n");
    writeText(root / "dark.sco", "[mesh]\ndark.o3d\n");
    writeText(root / "dark-x.sco", "[mesh]\ndark.x\n");
    writeText(root / "shadow.sco", "[mesh]\ndark.o3d\n[isshadow]\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\nshadow.sco\n99\n-10\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nup-x.sco\n100\n-6\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nup.sco\n101\n-2\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\ndown.sco\n102\n2\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\ndark-x.sco\n103\n6\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\ndark.sco\n104\n10\n0\n5\n0\n0\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write O3D normal terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 6, "Normal scenery did not load");
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, 0, -16}));
    setLightingViewMatrix(identityMatrix());
    glDepthMask(GL_TRUE);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    renderer.draw();
    std::array<std::uint8_t, 4> shadow = {}, upX = {}, up = {}, down = {}, darkX = {}, dark = {};
    glReadPixels(2, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, shadow.data());
    glReadPixels(14, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, upX.data());
    glReadPixels(26, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, up.data());
    glReadPixels(38, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, down.data());
    glReadPixels(50, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, darkX.data());
    glReadPixels(62, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, dark.data());
    check(up[0] > 215 && down[0] >= 145 && down[0] <= 150 && up[0] > down[0] + 60,
          "O3D surface normal did not produce the expected directional lighting");
    check(upX[0] == up[0], "X and O3D upward normals did not produce matching lighting");
    check(darkX[0] >= 145 && darkX[0] <= 150 && dark[0] == darkX[0],
          "Dark X and O3D scenery diffuse colours incorrectly darkened ambient light");
    check(shadow[0] >= 4 && shadow[0] <= 8,
          "Scenery shadow mesh lost its authored dark tint");
}

void runCutoutProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    writeSolidTexture(root / "base.tga", {0, 0, 0});
    writeRgbaTexture(root / "yellow.tga", 255);
    writeRgbaTexture(root / "clear.tga", 0);
    const auto mesh = [](std::string_view texture) {
        return "xof 0303txt 0032\n"
               "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
               "MeshTextureCoords { 3; 0.5;0.5;, 0.5;0.5;, 0.5;0.5;; }\n"
               "MeshMaterialList { 1; 2; 0,0;; Material { "
               "0.64;0.64;0.64;0;; 0; 0;0;0;; 0;0;0;; TextureFilename { \"" +
               std::string(texture) + "\"; } } } }\n";
    };
    writeText(root / "model" / "yellow.x", mesh("yellow.tga"));
    writeText(root / "model" / "clear.x", mesh("clear.tga"));
    writeText(root / "cutout.sco",
              "[mesh]\nyellow.x\n[matl]\nyellow.tga\n0\n[matl_alpha]\n1\n");
    writeText(root / "clear.sco",
              "[mesh]\nclear.x\n[matl]\nclear.tga\n0\n[matl_alpha]\n1\n");
    writeText(root / "blend.sco",
              "[mesh]\nyellow.x\n[matl]\nyellow.tga\n0\n[matl_alpha]\n2\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\ncutout.sco\n100\n-4\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nclear.sco\n101\n0\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nblend.sco\n102\n4\n0\n5\n0\n0\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write scenery cutout terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 3, "Cutout scenery did not load");
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, 0, -16}));
    setLightingViewMatrix(identityMatrix());
    glDepthMask(GL_TRUE);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    renderer.draw();
    std::array<std::uint8_t, 4> cutout = {}, clear = {}, blend = {};
    glReadPixels(20, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, cutout.data());
    glReadPixels(32, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, clear.data());
    glReadPixels(44, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, blend.data());
    check(cutout[0] > 100 && cutout[1] > 70, "Zero-alpha cutout marking was not rendered");
    check(clear[0] < 5 && clear[1] < 5,
          "Transparent texture was rendered");
    check(blend[0] < 5 && blend[1] < 5,
          "Blended zero-alpha material was rendered");
}

void runTreeSeasonProbe(const std::filesystem::path& root, bool mesh = false) {
    using namespace openbus::rendering::map_detail;
    const auto period = parseTreeSeason({"Spring:", {"1", "79", "172"}});
    check(period.has_value(), "Valid tree season was rejected");
    const std::vector<TreeSeasonPeriod> periods{*period};
    check(treeSeasonIndex(periods, 79, 0.0) == 0 &&
              treeSeasonIndex(periods, 80, 0.0) == 1 &&
              treeSeasonIndex(periods, 172, 0.0) == 1 &&
              treeSeasonIndex(periods, 173, 0.0) == 0 &&
              treeSeasonIndex(periods, 80, treeSnowThreshold * 0.5) == 1 &&
              treeSeasonIndex(periods, 80, treeSnowThreshold) == 4,
          "Tree season boundary or snow threshold is incorrect");
    check(!parseTreeSeason({"Bad:", {"1", "172", "79"}}) &&
              !parseTreeSeason({"Bad:", {"1", "x", "172"}}) &&
              !parseTreeSeason({"Bad:", {"5", "0", "38"}}),
          "Invalid tree season was accepted");
    const auto textureDirectory = mesh ? root / "plants" / "texture" : root / "texture";
    std::filesystem::create_directories(textureDirectory / "Spring");
    std::filesystem::create_directories(textureDirectory / "Summer");
    std::filesystem::create_directories(textureDirectory / "Winter");
    std::filesystem::create_directories(textureDirectory / "WinterSnow");
    writeSolidTexture(root / "base.tga", {0, 0, 0});
    writeSolidTexture(textureDirectory / "tree.tga", {0, 255, 0});
    writeSolidBmp(textureDirectory / "Spring" / "tree.bmp", {255, 0, 0});
    writeSolidTexture(textureDirectory / "Summer" / "tree.tga", {0, 0, 255});
    writeSolidTexture(textureDirectory / "Winter" / "tree.tga", {255, 255, 0});
    writeSolidTexture(textureDirectory / "WinterSnow" / "tree.tga", {255, 0, 255});
    if (mesh) {
        std::filesystem::create_directories(root / "model");
        writeText(root / "model" / "plant.x",
                  "xof 0303txt 0032\n"
                  "Mesh { 3; -5;0;-5;, 5;0;-5;, 0;0;5;; 2; 3;0,1,2;, 3;2,1,0;;\n"
                  "MeshTextureCoords { 3; 0.5;0.5;, 0.5;0.5;, 0.5;0.5;; }\n"
                  "MeshMaterialList { 1; 2; 0,0;; Material { "
                  "1;1;1;1;; 0; 0;0;0;; 0;0;0;; TextureFilename { \"plants/tree.tga\"; } } } }\n");
        writeText(root / "tree.sco", "[mesh]\nplant.x\n");
    } else {
        writeText(root / "tree.sco", "[tree]\ntree.tga\n10\n10\n1\n1\n");
    }
    writeText(root / "tile.map", "[version]\n14\n[terrain]\n"
              "[object]\n0\ntree.sco\n100\n0\n0\n5\n0\n0\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write tree terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.metadata.seasons = {{"Winter snow:", {"4", "0", "38"}},
                            {"Spring:", {"1", "79", "172"}},
                            {"Fall:", {"2", "264", "355"}}};
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    simulation.sharedVariables().set("dayofyear", 200.0F);
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 1, "Tree scenery did not load");
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, -4, -16}));
    setLightingViewMatrix(identityMatrix());
    const auto draw = [&](MapRenderer& scene) {
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        scene.draw();
        scene.finishFrame();
        std::array<std::uint8_t, 64 * 64 * 4> pixels = {};
        glReadPixels(0, 0, 64, 64, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
        std::array<unsigned int, 5> maximum = {};
        for (std::size_t pixel = 0; pixel < pixels.size(); pixel += 4) {
            for (std::size_t channel = 0; channel < 3; ++channel) {
                maximum[channel] = std::max(maximum[channel],
                                            static_cast<unsigned int>(pixels[pixel + channel]));
            }
            const int magenta = std::min(static_cast<int>(pixels[pixel]),
                                         static_cast<int>(pixels[pixel + 2])) -
                                static_cast<int>(pixels[pixel + 1]);
            maximum[3] = std::max(maximum[3], static_cast<unsigned int>(std::max(0, magenta)));
            const int yellow = std::min(static_cast<int>(pixels[pixel]),
                                        static_cast<int>(pixels[pixel + 1])) -
                               static_cast<int>(pixels[pixel + 2]);
            maximum[4] = std::max(maximum[4], static_cast<unsigned int>(std::max(0, yellow)));
        }
        return maximum;
    };
    const auto awaitColor = [&](MapRenderer& scene, const auto& matches) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        auto color = draw(scene);
        while (!matches(color) && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            color = draw(scene);
        }
        return color;
    };
    const auto summer = awaitColor(renderer, [](const auto& color) { return color[2] > 50; });
    simulation.sharedVariables().set("dayofyear", 80.0F);
    const auto spring = awaitColor(renderer, [](const auto& color) { return color[0] > 50; });
    simulation.sharedVariables().set("dayofyear", 265.0F);
    const auto fallFallback = draw(renderer);
    simulation.sharedVariables().set("dayofyear", 1.0F);
    const auto winter =
        awaitColor(renderer, [](const auto& color) { return color[0] > 50 && color[1] > 50; });
    auto weather = simulation.sharedVariables().weather();
    weather.omsiSnow = true;
    simulation.sharedVariables().setWeather(weather);
    simulation.sharedVariables().seedRoadWeatherFromPreset();
    const auto snow = awaitColor(renderer, [](const auto& color) { return color[3] > 50; });
    simulation.sharedVariables().resetRoadWeather();
    const auto restored = draw(renderer);
    check(summer[2] > 50 && summer[0] < 5 && summer[1] < 5, "Summer tree texture missing");
    check(spring[0] > 50 && spring[1] < 5 && spring[2] < 5,
          "Spring tree texture or alternate extension missing");
    check(fallFallback[1] > 50 && fallFallback[0] < 5 && fallFallback[2] < 5,
          "Missing seasonal texture did not fall back");
    check(winter[0] > 50 && winter[1] > 50 && winter[2] < 5,
          "Dry snowy-season tree did not use winter texture");
    check(snow[3] > 50, "Snow tree texture missing");
    check(restored == winter, "Tree texture did not recover after snow cleared");
    check(std::filesystem::remove(textureDirectory / "WinterSnow" / "tree.tga"),
          "Could not remove snow texture for fallback test");
    simulation.sharedVariables().seedRoadWeatherFromPreset();
    MapRenderer noSnowVariant(map, 0, 0, 0, 0, root, simulation, assets);
    const auto winterFallback = awaitColor(noSnowVariant, [](const auto& color) {
        return color[4] > 30;
    });
    check(winterFallback[4] > 30,
          "Missing snow texture did not fall back to winter");
    {
        std::ofstream invalid(textureDirectory / "WinterSnow" / "tree.tga", std::ios::binary);
        invalid << "invalid texture";
        check(static_cast<bool>(invalid), "Could not write invalid snow texture");
    }
    MapRenderer invalidSnowVariant(map, 0, 0, 0, 0, root, simulation, assets);
    const auto invalidSnowFallback =
        awaitColor(invalidSnowVariant, [](const auto& color) { return color[4] > 30; });
    check(invalidSnowFallback[4] > 30, "Invalid snow texture did not fall back to winter");

    simulation.sharedVariables().resetRoadWeather();
    simulation.sharedVariables().set("dayofyear", 200.0F);
    const auto queueSeasonWhileBlocked = [&](MapRenderer& scene, bool finishFrame = true) {
        std::promise<void> releaseWorkers;
        struct ReleaseWorkers {
            std::promise<void>& promise;
            bool released = false;
            void release() {
                promise.set_value();
                released = true;
            }
            ~ReleaseWorkers() {
                if (!released) {
                    promise.set_value();
                }
            }
        } releaseGuard{releaseWorkers};
        const auto release = releaseWorkers.get_future().share();
        std::vector<std::future<void>> blockers;
        const auto workers = assets.queueStats().workerCount;
        for (std::size_t index = 0; index < workers; ++index) {
            blockers.push_back(assets.submit([release] { release.wait(); }));
        }
        const auto readyDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (assets.queueStats().activeWorkers < workers &&
               std::chrono::steady_clock::now() < readyDeadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        check(workers != 0 && assets.queueStats().activeWorkers == workers,
              "Could not block seasonal texture workers");
        if (finishFrame) {
            draw(scene);
        } else {
            scene.draw();
        }
        releaseGuard.release();
        for (auto& blocker : blockers) {
            blocker.get();
        }
        const auto decodeDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while ((assets.queueStats().activeWorkers != 0 || assets.queueStats().queuedTasks != 0) &&
               std::chrono::steady_clock::now() < decodeDeadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        check(assets.queueStats().activeWorkers == 0 && assets.queueStats().queuedTasks == 0,
              "Seasonal texture decode did not finish");
    };
    MapRenderer slowVisible(map, 0, 0, 0, 0, root, simulation, assets);
    queueSeasonWhileBlocked(slowVisible);
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    setModelViewMatrix(translationMatrix({500, -4, -16}));
    slowVisible.draw();
    setModelViewMatrix(translationMatrix({0, -4, -16}));
    const auto slowColor = draw(slowVisible);
    check(slowColor[2] > 50 && slowColor[0] < 5 && slowColor[1] < 5,
          "Visible seasonal texture was discarded at a low frame rate");

    MapRenderer longFrame(map, 0, 0, 0, 0, root, simulation, assets);
    queueSeasonWhileBlocked(longFrame, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    longFrame.finishFrame();
    check(std::filesystem::remove(textureDirectory / "Summer" / "tree.tga"),
          "Could not remove seasonal texture for long-frame test");
    const auto longFrameColor = draw(longFrame);
    check(longFrameColor[2] > 50 && longFrameColor[0] < 5 && longFrameColor[1] < 5,
          "Seasonal texture requested during a long frame was discarded");
    writeSolidTexture(textureDirectory / "Summer" / "tree.tga", {0, 0, 255});

    MapRenderer unusedRequest(map, 0, 0, 0, 0, root, simulation, assets);
    queueSeasonWhileBlocked(unusedRequest);
    simulation.sharedVariables().set("dayofyear", 265.0F);
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    draw(unusedRequest);
    check(std::filesystem::remove(textureDirectory / "Summer" / "tree.tga"),
          "Could not remove unused seasonal texture");
    simulation.sharedVariables().set("dayofyear", 200.0F);
    draw(unusedRequest);
    const auto retryDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while ((assets.queueStats().activeWorkers != 0 || assets.queueStats().queuedTasks != 0) &&
           std::chrono::steady_clock::now() < retryDeadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    check(assets.queueStats().activeWorkers == 0 && assets.queueStats().queuedTasks == 0,
          "Seasonal texture retry did not finish");
    const auto unusedFallback = draw(unusedRequest);
    check(unusedFallback[1] > 50 && unusedFallback[0] < 5 && unusedFallback[2] < 5,
          "Unused seasonal decode was retained instead of released");
}

void runWaterCornerProbe(const std::filesystem::path& root) {
    writeSolidTexture(root / "base.tga", {255, 0, 0});
    writeText(root / "tile.map", "[version]\n14\n[terrain]\n[water]\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write water-corner terrain");
    terrainFile.close();
    const std::array<std::uint32_t, 5> water = {
        1, std::bit_cast<std::uint32_t>(-1.0F), std::bit_cast<std::uint32_t>(-1.0F),
        std::bit_cast<std::uint32_t>(2.0F), std::bit_cast<std::uint32_t>(-1.0F)};
    std::ofstream waterFile(root / "tile.water", std::ios::binary);
    for (const std::uint32_t value : water) {
        const std::array<char, 4> bytes = {
            static_cast<char>(value & 0xffU), static_cast<char>((value >> 8) & 0xffU),
            static_cast<char>((value >> 16) & 0xffU), static_cast<char>((value >> 24) & 0xffU)};
        waterFile.write(bytes.data(), bytes.size());
    }
    check(static_cast<bool>(waterFile), "Could not write water-corner sidecar");
    waterFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.coordinates.tileSize = 10;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.waterPath = root / "tile.water";
    tile.hasTerrainFile = true;
    tile.hasWaterFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 5, 5, 0, root, simulation, assets);
    check(renderer.renderStats().totalWaterBatches == 1, "Water corner fixture did not load");
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    lookAt(5, 5, 10, 5, 5, 0, 0, 1, 0);
    setLightingViewMatrix(modelViewMatrix());
    glDepthMask(GL_TRUE);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    renderer.draw();
    std::array<std::uint8_t, 4> northwest = {}, northeast = {};
    glReadPixels(19, 45, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, northwest.data());
    glReadPixels(45, 45, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, northeast.data());
    check(northwest[2] > 20 && northeast[2] < 5,
          "Raised north-west water appeared on the wrong side of the terrain");
}

void runLightmapProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "script");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeSolidTexture(root / "light.tga", {255, 255, 255});
    writeText(root / "invalid.tga", "invalid image");
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshTextureCoords { 3; 0.5;0.5;, 0.5;0.5;, 0.5;0.5;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 0;0;0;; TextureFilename { \"base.tga\"; } } } }\n");
    const std::string mesh = "[mesh]\nsurface.x\n[matl]\nbase.tga\n0\n[matl_alpha]\n2\n";
    writeText(root / "controlled.sco",
              "[script]\n1\nscript\\lamp.osc\n"
              "[varnamelist]\n1\nscript\\lamp_vars.txt\n"
              "[stringvarnamelist]\n1\nscript\\lamp_strings.txt\n" + mesh +
                  "[alphascale]\nlamp_alpha\n[matl_lightmap]\nlight.tga\nlamp_light\n");
    writeText(root / "uncontrolled.sco", mesh + "[matl_lightmap]\nlight.tga\n");
    writeText(root / "missing.sco", mesh + "[matl_lightmap]\nmissing.tga\n");
    writeText(root / "invalid.sco", mesh + "[matl_lightmap]\ninvalid.tga\n");
    writeText(root / "script" / "lamp_vars.txt", "instance_id\nlamp_alpha\nlamp_light\n");
    writeText(root / "script" / "lamp_strings.txt", "placement_id\n");
    writeText(root / "script" / "lamp.osc",
              "{init}\n(L.$.placement_id) $StrToFloat (S.L.instance_id)\n{end}\n"
              "{frame}\n"
              "(L.S.test_alpha) (L.L.instance_id) * (S.L.lamp_alpha)\n"
              "(L.S.test_light) (L.L.instance_id) * (S.L.lamp_light)\n{end}\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\ncontrolled.sco\n101\n-6\n0\n5\n0\n0\n0\n1\n1\n"
              "[object]\n0\ncontrolled.sco\n102\n-2\n0\n5\n0\n0\n0\n1\n2\n"
              "[object]\n0\nuncontrolled.sco\n103\n2\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nmissing.sco\n104\n6\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\ninvalid.sco\n105\n50\n0\n5\n0\n0\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write scenery lightmap terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 5,
          "Lightmap scenery or invalid-lightmap fallback did not load");
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, 0, -16}));
    using Pixel = std::array<std::uint8_t, 4>;
    const auto draw = [&](float light, float alpha) {
        simulation.sharedVariables().set("test_light", light);
        simulation.sharedVariables().set("test_alpha", alpha);
        renderer.updateScripts();
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer.draw();
        std::array<Pixel, 4> pixels = {};
        const std::array<int, 4> x = {14, 26, 38, 49};
        for (std::size_t index = 0; index < x.size(); ++index) {
            glReadPixels(x[index], 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels[index].data());
        }
        return pixels;
    };
    const auto dark = draw(0, 1);
    const auto half = draw(0.5F, 1);
    const auto bright = draw(1, 1);
    if (!(dark[0][0] > 0 && dark[0][0] == dark[1][0] &&
          bright[0][0] > dark[0][0] + 30 && bright[1][0] == bright[0][0])) {
        for (const auto& pixels : {dark, half, bright}) {
            std::cerr << "Lightmap samples:";
            for (const auto& pixel : pixels) {
                std::cerr << ' ' << static_cast<int>(pixel[0]) << ',' << static_cast<int>(pixel[3]);
            }
            std::cerr << '\n';
        }
    }
    check(dark[0][0] > 0 && dark[0][0] == dark[1][0] &&
              bright[0][0] > dark[0][0] + 30 && bright[1][0] == bright[0][0],
          "Lightmap controller did not change surface illumination");
    check(half[0][0] > dark[0][0] && half[0][0] < bright[0][0] &&
              half[1][0] == bright[1][0],
          "Lightmap strength is not evaluated independently per placement");
    check(bright[2][0] == bright[0][0] && bright[3][0] == dark[0][0],
          "Uncontrolled or missing lightmap did not preserve its expected fallback");
    check(draw(-1, 1)[0] == dark[0] && draw(2, 1)[0] == bright[0],
          "Lightmap strength was not clamped");
    const auto faded = draw(1, 0.5F);
    const auto hidden = draw(1, 0);
    check(faded[0][0] < bright[0][0] && faded[1][0] == bright[1][0] &&
              hidden[0][0] < faded[0][0] && hidden[1][0] < faded[1][0],
          "Alpha scale did not fade and hide placements independently");
    check(draw(1, 2)[0] == bright[0] && draw(1, -1)[0] == hidden[0],
          "Alpha scale was not clamped");
}

void runRoadWeatherProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "Splines" / "Weather" / "texture");
    writeSolidTexture(root / "Splines" / "Weather" / "texture" / "wet.tga", {180, 180, 180});
    writeSolidTexture(root / "dry.tga", {180, 180, 180});
    writeSolidTexture(root / "classified.tga", {180, 180, 180});
    writeSolidTexture(root / "ground.tga", {30, 30, 30});
    writeText(root / "Splines" / "Weather" / "texture" / "wet.tga.cfg",
              "[moisture]\n[surface]\n0\n");
    writeText(root / "classified.tga.cfg", "[surface]\n0\n");
    writeText(root / "Splines" / "Test.sli",
              "[texture]\nWeather\\wet.tga\n[texture]\nclassified.tga\n[texture]\ndry.tga\n"
              "[profile]\n0\n[profilepnt]\n-6\n0\n0\n0\n"
              "[profilepnt]\n-2\n0\n1\n0\n"
              "[profile]\n1\n[profilepnt]\n-2\n0\n0\n0\n"
              "[profilepnt]\n2\n0\n1\n0\n"
              "[profile]\n2\n[profilepnt]\n2\n0\n0\n0\n"
              "[profilepnt]\n6\n0\n1\n0\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[spline]\n0\nSplines\\Test.sli\n1\n0\n0\n5\n1\n5\n0\n10\n"
              "0\n0\n0\n0\n0\n0\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write road weather terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"ground.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 5, 10, 0, root, simulation, assets);
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    lookAt(5, 10, 10, 5, 10, 0, 0, 1, 0);
    const auto draw = [&]() {
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer.draw();
        std::array<std::array<std::uint8_t, 4>, 3> pixels = {};
        glReadPixels(21, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels[0].data());
        glReadPixels(32, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels[1].data());
        glReadPixels(43, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels[2].data());
        return pixels;
    };
    const auto dry = draw();
    auto weather = simulation.sharedVariables().weather();
    weather.omsiGroundWetness = std::array<double, 3>{79.0, 0.0, 0.0};
    simulation.sharedVariables().setWeather(weather);
    simulation.sharedVariables().seedRoadWeatherFromPreset();
    const auto wet = draw();
    weather.omsiGroundWetness.reset();
    weather.omsiSnow = true;
    simulation.sharedVariables().setWeather(weather);
    simulation.sharedVariables().seedRoadWeatherFromPreset();
    const auto groundOnlySnow = draw();
    check(groundOnlySnow == dry, "Ground-only snow changed spline-road shading");
    weather.omsiSnow = false;
    weather.omsiSnowOnRoad = true;
    simulation.sharedVariables().setWeather(weather);
    simulation.sharedVariables().seedRoadWeatherFromPreset();
    const auto snow = draw();
    weather.omsiGroundWetness = std::array<double, 3>{79.0, 0.0, 0.0};
    simulation.sharedVariables().setWeather(weather);
    simulation.sharedVariables().seedRoadWeatherFromPreset();
    const auto wetSnow = draw();
    simulation.sharedVariables().resetRoadWeather();
    weather.omsiGroundWetness.reset();
    weather.omsiSnowOnRoad = false;
    weather.precipitationType = 2;
    weather.precipitationRate = 1.0;
    weather.temperatureCelsius = -5.0;
    simulation.sharedVariables().setWeather(weather);
    simulation.sharedVariables().advanceRoadWeather(830.0);
    const auto partialSnow = draw();
    simulation.sharedVariables().resetRoadWeather();
    const auto restored = draw();
    int wetCount = 0;
    int snowCount = 0;
    int unchangedCount = 0;
    for (std::size_t index = 0; index < dry.size(); ++index) {
        check(dry[index][0] >= 60 && dry[index] == dry[0],
              "Road strips do not share the expected dry baseline");
        if (wet[index][0] + 10 < dry[index][0]) {
            ++wetCount;
            check(snow[index][0] > dry[index][0] + 10,
                  "Moisture-enabled road did not accumulate snow");
            check(wetSnow[index][0] > wet[index][0] &&
                      wetSnow[index][0] <= snow[index][0],
                  "Combined snow and water did not produce bounded road shading");
        } else {
            check(wet[index] == dry[index], "Wetness changed a non-moisture road");
            check(wetSnow[index] == snow[index], "Water changed a non-moisture snowy road");
        }
        if (snow[index][0] > dry[index][0] + 10) {
            ++snowCount;
            check(partialSnow[index][0] > dry[index][0] &&
                      partialSnow[index][0] < snow[index][0],
                  "Snow accumulation did not produce intermediate shading");
        } else {
            check(snow[index] == dry[index], "Snow changed an unclassified road");
            check(partialSnow[index] == dry[index], "Partial snow changed an unclassified road");
            ++unchangedCount;
        }
        check(wet[index][3] == dry[index][3] && snow[index][3] == dry[index][3] &&
                  partialSnow[index][3] == dry[index][3],
              "Road weather changed surface alpha");
    }
    check(wetCount == 1 && snowCount == 2 && unchangedCount == 1 && restored == dry,
          "Road weather did not respect classification or restore the dry baseline");
}

void runTerrainWeatherProbe(const std::filesystem::path& root) {
    std::array<std::uint8_t, 4> baseDry = {};
    std::array<std::uint8_t, 4> baseSnow = {};
    for (const bool layered : {false, true}) {
        const auto directory = root / (layered ? "layered" : "base");
        std::filesystem::create_directories(directory);
        writeSolidTexture(directory / "ground.tga", {80, 80, 80});
        writeSolidTexture(directory / "layer.tga", {160, 160, 160});
        writeText(directory / "tile.map", "[version]\n14\n[terrain]\n");
        const std::array<std::uint8_t, 20> terrain = {1};
        std::ofstream terrainFile(directory / "tile.terrain", std::ios::binary);
        terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
        check(static_cast<bool>(terrainFile), "Could not write terrain weather fixture");
        terrainFile.close();
        if (layered) {
            std::array<std::uint8_t, 22> mask = {};
            mask[2] = 2;
            mask[12] = mask[14] = 1;
            mask[16] = 32;
            mask[17] = 8;
            mask[18] = mask[19] = mask[20] = 255;
            mask[21] = 128;
            std::ofstream maskFile(directory / "tile.map.2.tga", std::ios::binary);
            maskFile.write(reinterpret_cast<const char*>(mask.data()), mask.size());
            check(static_cast<bool>(maskFile), "Could not write terrain layer mask");
        }
        openbus::map::MapDefinition map;
        map.rootPath = directory;
        map.groundTextures.push_back({"ground.tga", "", {}});
        map.groundTextures.push_back({"layer.tga", "", {}});
        openbus::map::MapTileReference tile;
        tile.textPath = directory / "tile.map";
        tile.terrainPath = directory / "tile.terrain";
        tile.hasTerrainFile = true;
        if (layered) {
            tile.groundTextureSidecars.push_back({"2", directory / "tile.map.2.tga"});
        }
        map.tiles.push_back(tile);
        SimulationState simulation;
        AssetRequestManager assets;
        MapRenderer renderer(map, 0, 150, 150, 0, directory, simulation, assets);
        glViewport(0, 0, 64, 64);
        glDisable(GL_DITHER);
        setPerspective(64, 64, 90);
        lookAt(150, 150, 20, 150, 150, 0, 0, 1, 0);
        const auto draw = [&]() {
            glDepthMask(GL_TRUE);
            glClearColor(0, 0, 0, 0);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            renderer.draw();
            std::array<std::uint8_t, 4> pixel = {};
            glReadPixels(32, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
            return pixel;
        };
        const auto dry = draw();
        auto weather = simulation.sharedVariables().weather();
        weather.omsiSnow = true;
        weather.omsiSnowOnRoad = false;
        simulation.sharedVariables().setWeather(weather);
        simulation.sharedVariables().seedRoadWeatherFromPreset();
        const auto snow = draw();
        simulation.sharedVariables().resetRoadWeather();
        weather.omsiSnow = false;
        weather.precipitationType = 2;
        weather.precipitationRate = 1.0;
        weather.temperatureCelsius = -5.0;
        simulation.sharedVariables().setWeather(weather);
        simulation.sharedVariables().advanceRoadWeather(830.0);
        const auto partialSnow = draw();
        simulation.sharedVariables().resetRoadWeather();
        const auto restored = draw();
        check(dry[0] > 10 && snow[0] > dry[0] + 20 && restored == dry,
              "Terrain snow did not brighten and restore the ground");
        check(partialSnow[0] > dry[0] && partialSnow[0] < snow[0],
              "Terrain snow did not gradually accumulate");
        if (layered) {
            check(dry[0] > baseDry[0] + 5, "Terrain weather fixture did not render its layer");
            check(snow[0] >= baseSnow[0] && snow[0] < baseSnow[0] + 12,
                  "Snow did not cover the blended terrain layer");
        } else {
            baseDry = dry;
            baseSnow = snow;
        }
    }
}

void runSplineLabelsProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "script");
    std::filesystem::create_directories(root / "Splines");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshTextureCoords { 3; 0.5;0.5;, 0.5;0.5;, 0.5;0.5;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 0;0;0;; TextureFilename { \"base.tga\"; } } } }\n");
    writeText(root / "sign.sco",
              "[script]\n1\nscript\\sign.osc\n"
              "[varnamelist]\n1\nscript\\vars.txt\n"
              "[stringvarnamelist]\n1\nscript\\strings.txt\n"
              "[mesh]\nsurface.x\n[matl]\nbase.tga\n0\n[matl_alpha]\n2\n"
              "[alphascale]\nlamp_alpha\n");
    writeText(root / "script" / "vars.txt", "instance_id\nlamp_alpha\n");
    writeText(root / "script" / "strings.txt", "placement_id\n");
    writeText(root / "script" / "sign.osc",
              "{init}\n(L.$.placement_id) $StrToFloat (S.L.instance_id)\n{end}\n"
              "{frame}\n(L.S.test_alpha) (L.L.instance_id) * (S.L.lamp_alpha)\n{end}\n");
    writeText(root / "Splines" / "Test.sli",
              "[texture]\nbase.tga\n[profile]\n0\n[profilepnt]\n-2\n0\n0\n0\n"
              "[profilepnt]\n2\n0\n1\n0\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[spline]\n0\nSplines\\Test.sli\n1\n0\n0\n0\n0\n-5\n0\n20\n"
              "0\n0\n0\n0\n0\n0\n0\n0\n"
              "[splineAttachement]\n0\nsign.sco\n101\n0\n-6\n5\n5\n0\n0\n0\n0\n0\n0\n1\n1\n"
              "[splineAttachement]\n0\nsign.sco\n102\n0\n-2\n5\n5\n0\n0\n0\n0\n0\n0\n1\n2\n"
              "[splineAttachement_repeater]\n0\n0\n0\nsign.sco\n103\n0\n2\n5\n5\n"
              "0\n0\n0\n0\n0\n0\n1\n1\n"
              "[splineAttachement_repeater]\n0\n0\n0\nsign.sco\n104\n0\n6\n5\n5\n"
              "0\n0\n0\n0\n0\n0\n1\n2\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write spline label terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 4,
          "Spline label fixture did not load all four attachments");
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, 0, -16}));
    const auto draw = [&](float alpha) {
        simulation.sharedVariables().set("test_alpha", alpha);
        renderer.updateScripts();
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer.draw();
        std::array<std::array<std::uint8_t, 4>, 4> pixels = {};
        const std::array<int, 4> positions = {14, 26, 38, 49};
        for (std::size_t index = 0; index < positions.size(); ++index) {
            glReadPixels(positions[index], 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE,
                         pixels[index].data());
        }
        return pixels;
    };
    const auto hidden = draw(0.0F);
    const auto half = draw(0.5F);
    const auto full = draw(1.0F);
    bool valid = true;
    for (std::size_t index = 0; index < half.size(); ++index) {
        const int baseline = hidden[index][0];
        const int intermediate = half[index][0];
        const int opaque = full[index][0];
        valid = valid && std::abs(opaque - baseline) > 30;
        if (index % 2 == 0) {
            valid = valid && std::abs(2 * intermediate - baseline - opaque) <= 3;
        } else {
            valid = valid && std::abs(intermediate - opaque) <= 1;
        }
    }
    if (!valid) {
        std::cerr << "Spline label samples:";
        for (std::size_t index = 0; index < half.size(); ++index) {
            std::cerr << ' ' << static_cast<int>(hidden[index][0]) << '/'
                      << static_cast<int>(half[index][0]) << '/' << static_cast<int>(full[index][0]);
        }
        std::cerr << '\n';
    }
    check(valid,
          "Spline attachment labels did not initialize independent regular and repeater scripts");
}

void runSplineTextProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "Splines");
    std::filesystem::create_directories(root / "Fonts");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeSolidTexture(root / "Fonts" / "glyph.tga", {255, 255, 255});
    writeText(root / "Fonts" / "spline_label.oft",
              "[newfont]\nSplineLabelFixture\nglyph.tga\nglyph.tga\n1\n0\n"
              "[char]\nX\n0\n1\n0\n");
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshTextureCoords { 3; 0.5;0.5;, 0.5;0.5;, 0.5;0.5;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 0;0;0;; TextureFilename { \"base.tga\"; } } } }\n");
    writeText(root / "sign.sco",
              "[texttexture]\n0\nSplineLabelFixture\n1\n1\n0\n255\n0\n0\n"
              "[mesh]\nsurface.x\n[matl]\nbase.tga\n0\n[matl_alpha]\n2\n"
              "[usetexttexture]\n0\n");
    writeText(root / "free.sco",
              "[mesh]\nsurface.x\n[matl]\nbase.tga\n0\n"
              "[matl_freetex]\nbase.tga\nsign_texture\n");
    writeText(root / "Splines" / "Test.sli",
              "[texture]\nbase.tga\n[profile]\n0\n[profilepnt]\n-2\n0\n0\n0\n"
              "[profilepnt]\n2\n0\n1\n0\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[spline]\n0\nSplines\\Test.sli\n1\n0\n0\n0\n0\n-5\n0\n20\n"
              "0\n0\n0\n0\n0\n0\n0\n0\n"
              "[splineAttachement]\n0\nsign.sco\n101\n0\n-6\n5\n5\n0\n0\n0\n0\n0\n0\n1\nX\n"
              "[splineAttachement]\n0\nsign.sco\n102\n0\n-2\n5\n5\n0\n0\n0\n0\n0\n0\n1\n\n"
              "[splineAttachement_repeater]\n0\n0\n0\nsign.sco\n103\n0\n2\n5\n5\n"
              "0\n0\n0\n0\n0\n0\n1\nX\n"
              "[splineAttachement_repeater]\n0\n0\n0\nsign.sco\n104\n0\n6\n5\n5\n"
              "0\n0\n0\n0\n0\n0\n1\n\n"
              "[splineAttachement]\n0\nfree.sco\n105\n0\n10\n5\n5\n0\n0\n0\n0\n0\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write spline text terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 5,
          "Spline text fixture did not load all attachments");
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, 0, -16}));
    renderer.updateScripts();
    glDepthMask(GL_TRUE);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    renderer.draw();
    std::array<std::array<std::uint8_t, 4>, 4> pixels = {};
    const std::array<int, 4> positions = {14, 26, 38, 49};
    for (std::size_t index = 0; index < positions.size(); ++index) {
        glReadPixels(positions[index], 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels[index].data());
    }
    std::array<std::uint8_t, 4> freeDay = {};
    glReadPixels(61, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, freeDay.data());
    const auto isRed = [](const auto& pixel) {
        return pixel[0] > 80 && pixel[1] < pixel[0] / 3 && pixel[2] < pixel[0] / 3;
    };
    if (!isRed(pixels[0]) || isRed(pixels[1]) || !isRed(pixels[2]) || isRed(pixels[3])) {
        std::cerr << "Spline text samples:";
        for (const auto& pixel : pixels) {
            std::cerr << ' ' << static_cast<int>(pixel[0]) << ',' << static_cast<int>(pixel[1])
                      << ',' << static_cast<int>(pixel[2]) << ',' << static_cast<int>(pixel[3]);
        }
        std::cerr << '\n';
        throw std::runtime_error(
            "Spline text textures did not reflect regular and repeater placement labels");
    }
    setWeatherLighting(0.2);
    glDepthMask(GL_TRUE);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    renderer.draw();
    std::array<std::uint8_t, 4> dark = {};
    glReadPixels(14, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, dark.data());
    check(dark[0] > 0 && dark[0] < pixels[0][0] / 2 && dark[3] == pixels[0][3],
          "Scenery placement text did not darken with weather lighting");
    std::array<std::uint8_t, 4> freeDark = {};
    glReadPixels(61, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, freeDark.data());
    check(freeDay[0] > 100 && freeDark[0] > 0 && freeDark[0] < freeDay[0] / 2 &&
              freeDark[3] == freeDay[3],
          "Scenery free texture did not darken with weather lighting");
    setWeatherLighting(std::nullopt);
}

void runSplineOwnershipProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "script");
    std::filesystem::create_directories(root / "Splines");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeSolidTexture(root / "ground.tga", {0, 0, 0});
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshTextureCoords { 3; 0.5;0.5;, 0.5;0.5;, 0.5;0.5;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 0;0;0;; TextureFilename { \"base.tga\"; } } } }\n");
    writeText(root / "row.sco",
              "[script]\n1\nscript\\row.osc\n"
              "[varnamelist]\n1\nscript\\vars.txt\n"
              "[mesh]\nsurface.x\n[matl]\nbase.tga\n0\n[matl_alpha]\n2\n"
              "[matl_nozwrite]\n[alphascale]\nopacity\n");
    writeText(root / "script" / "vars.txt", "opacity\n");
    writeText(root / "script" / "row.osc", "{init}\n0.5 (S.L.opacity)\n{end}\n");
    writeText(root / "Splines" / "Test.sli",
              "[texture]\nground.tga\n[profile]\n0\n[profilepnt]\n-2\n0\n0\n0\n"
              "[profilepnt]\n2\n0\n1\n0\n");
    writeText(root / "tile0.map",
              "[version]\n14\n[terrain]\n"
              "[spline]\n0\nSplines\\Test.sli\n10\n0\n11\n0\n0\n290\n0\n10\n"
              "0\n0\n0\n0\n0\n0\n0\n0\n"
              "[splineAttachement]\n0\nrow.sco\n101\n0\n0\n5\n5\n0\n0\n0\n"
              "10\n20\n0\n0\n");
    writeText(root / "tile1.map",
              "[version]\n14\n[terrain]\n"
              "[spline]\n0\nSplines\\Test.sli\n11\n10\n0\n0\n0\n0\n0\n20\n"
              "0\n0\n0\n0\n0\n0\n0\n10\n"
              "[splineAttachement_repeater]\n0\n0\n1\nrow.sco\n101\n0\n0\n5\n5\n"
              "0\n0\n0\n10\n20\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    for (const auto& name : {"tile0.terrain", "tile1.terrain"}) {
        std::ofstream terrainFile(root / name, std::ios::binary);
        terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
        check(static_cast<bool>(terrainFile), "Could not write spline ownership terrain");
    }
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"ground.tga", "", {}});
    openbus::map::MapTileReference regularTile;
    regularTile.textPath = root / "tile0.map";
    regularTile.terrainPath = root / "tile0.terrain";
    regularTile.hasTerrainFile = true;
    map.tiles.push_back(regularTile);
    openbus::map::MapTileReference repeaterTile;
    repeaterTile.y = 1;
    repeaterTile.textPath = root / "tile1.map";
    repeaterTile.terrainPath = root / "tile1.terrain";
    repeaterTile.hasTerrainFile = true;
    map.tiles.push_back(repeaterTile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, -600, 0, root, simulation, assets);
    const std::vector<std::pair<int, int>> regularWindow = {{0, 0}};
    const std::vector<std::pair<int, int>> bothWindow = {{0, 0}, {0, 1}};
    const std::vector<std::pair<int, int>> repeaterWindow = {{0, 1}};
    const auto activate = [&](double y, const auto& expected) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        do {
            renderer.updateStreaming(0, y);
            if (renderer.activeTileCoordinates() == expected) {
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } while (std::chrono::steady_clock::now() < deadline);
        throw std::runtime_error("Spline ownership fixture failed to activate requested tiles");
    };
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, -305, -16}));
    const auto draw = [&]() {
        renderer.updateScripts();
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer.draw();
        std::array<std::uint8_t, 4> pixel = {};
        glReadPixels(32, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
        return pixel;
    };
    activate(-600, regularWindow);
    check(renderer.renderStats().activeSceneryObjects == 3,
          "Regular tile should prepare three row placements");
    const auto regular = draw();
    activate(0, bothWindow);
    check(renderer.renderStats().activeSceneryObjects == 5,
          "Both source tiles should select five row placements");
    const auto both = draw();
    activate(900, repeaterWindow);
    check(renderer.renderStats().activeSceneryObjects == 2,
          "Repeater tile should select two row placements");
    const auto repeater = draw();
    activate(0, bothWindow);
    const auto bothAgain = draw();
    activate(-600, regularWindow);
    const auto regularAgain = draw();
    if (!(regular[0] > 30 && both[0] > regular[0] + 20 &&
          std::abs(static_cast<int>(regular[0]) - static_cast<int>(repeater[0])) <= 2 &&
          bothAgain == both && regularAgain == regular)) {
        std::cerr << "Spline ownership samples (regular/both/repeater/both/regular): "
                  << static_cast<int>(regular[0]) << '/' << static_cast<int>(both[0]) << '/'
                  << static_cast<int>(repeater[0]) << '/' << static_cast<int>(bothAgain[0]) << '/'
                  << static_cast<int>(regularAgain[0]) << '\n';
        throw std::runtime_error("Spline source-tile selection did not match duplicate poses");
    }
}

void runSplineDependencyProbe(const std::filesystem::path& root, bool duplicateId) {
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "Splines");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshTextureCoords { 3; 0.5;0.5;, 0.5;0.5;, 0.5;0.5;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 0;0;0;; TextureFilename { \"base.tga\"; } } } }\n");
    const std::string path = "[path]\n0\n0\n0\n0\n0\n3\n0\n0\n0\n3\n0\n0\n";
    writeText(root / "row.sco",
              "[new_attachment]\nmount\nattach_trans\n0\n0\n0\n" + path +
                  "[mesh]\nsurface.x\n");
    writeText(root / "child.sco", path + "[mesh]\nsurface.x\n");
    writeText(root / "control.sco", "[onlyeditor]\n" + path);
    writeText(root / "Splines" / "Test.sli",
              "[texture]\nbase.tga\n[profile]\n0\n[profilepnt]\n-2\n0\n0\n0\n"
              "[profilepnt]\n2\n0\n1\n0\n");
    writeText(root / "tile0.map",
              "[version]\n14\n[terrain]\n"
              "[spline]\n0\nSplines\\Test.sli\n10\n0\n11\n0\n0\n290\n0\n10\n"
              "0\n0\n0\n0\n0\n0\n0\n0\n"
              "[splineAttachement]\n0\nrow.sco\n101\n0\n0\n5\n5\n0\n0\n0\n"
              "10\n20\n0\n0\n"
              "[attachObj]\n0\nchild.sco\n200\n101\n0\n0\n0\n0\n0\n0\n"
              "[object]\n0\ncontrol.sco\n300\n-1000\n0\n5\n0\n0\n0\n0\n0\n");
    writeText(root / "tile1.map",
              "[version]\n14\n[terrain]\n"
              "[spline]\n0\nSplines\\Test.sli\n11\n10\n0\n0\n0\n0\n0\n20\n"
              "0\n0\n0\n0\n0\n0\n0\n10\n"
              "[splineAttachement_repeater]\n0\n0\n1\nrow.sco\n" +
                  std::string(duplicateId ? "101" : "102") +
                  "\n0\n0\n5\n5\n0\n0\n0\n10\n20\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    for (const auto& name : {"tile0.terrain", "tile1.terrain"}) {
        std::ofstream terrainFile(root / name, std::ios::binary);
        terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
        check(static_cast<bool>(terrainFile), "Could not write spline dependency terrain");
    }
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    for (int y = 0; y < 2; ++y) {
        openbus::map::MapTileReference tile;
        tile.y = y;
        const std::string prefix = "tile" + std::to_string(y);
        tile.textPath = root / (prefix + ".map");
        tile.terrainPath = root / (prefix + ".terrain");
        tile.hasTerrainFile = true;
        map.tiles.push_back(tile);
    }
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, -900, 0, root, simulation, assets);
    const auto waitFor = [&](double y, const std::vector<std::pair<int, int>>& window,
                             std::size_t lanes101, std::size_t lanes102) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        do {
            renderer.updateStreaming(0, y);
            if (renderer.activeTileCoordinates() == window &&
                renderer.lanesForTimetableEntry(300, 0).size() == 1 &&
                renderer.lanesForTimetableEntry(101, 0).size() == lanes101 &&
                renderer.lanesForTimetableEntry(102, 0).size() == lanes102 &&
                renderer.lanesForTimetableEntry(200, 0).size() == (duplicateId ? 0U : 1U)) {
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } while (std::chrono::steady_clock::now() < deadline);
        std::string activeWindow;
        for (const auto& [tileX, tileY] : renderer.activeTileCoordinates()) {
            if (!activeWindow.empty()) {
                activeWindow += ',';
            }
            activeWindow += std::to_string(tileX) + ':' + std::to_string(tileY);
        }
        throw std::runtime_error(
            "Spline dependency window or traffic lanes did not settle: 101=" +
            std::to_string(renderer.lanesForTimetableEntry(101, 0).size()) +
            " 102=" + std::to_string(renderer.lanesForTimetableEntry(102, 0).size()) +
            " 200=" + std::to_string(renderer.lanesForTimetableEntry(200, 0).size()) +
            " active=" + activeWindow);
    };
    const auto debugPaths = [&]() {
        const auto& segments = renderer.sceneryTrafficPathSegments();
        return std::count_if(segments.begin(), segments.end(), [](const auto& segment) {
            return segment.mapObjectId == 101 || segment.mapObjectId == 102;
        });
    };
    waitFor(-900, {}, duplicateId ? 0 : 3, duplicateId ? 0 : 2);
    check(debugPaths() == 0, "Unloaded spline rows must not have prepared debug paths");
    waitFor(-600, {{0, 0}}, 3, duplicateId ? 0 : 2);
    check(renderer.renderStats().activeSceneryObjects == (duplicateId ? 3U : 4U),
          "Duplicate spline parent IDs should prevent the child placement");
    check(debugPaths() == 3, "The regular rows should prepare three debug paths");
    const auto& segments = renderer.sceneryTrafficPathSegments();
    const auto child = std::find_if(segments.begin(), segments.end(), [](const auto& segment) {
        return segment.mapObjectId == 200;
    });
    check((child == segments.end()) == duplicateId,
          "Child path must be prepared only for a resolvable spline parent");
    if (!duplicateId) {
        check(!child->points.empty() && std::abs(child->points.front()[1] - 295.0) < 0.001,
              "Child must resolve to the regular attachment's first row");
    }
    waitFor(0, {{0, 0}, {0, 1}}, duplicateId ? 5 : 3, duplicateId ? 0 : 2);
    check(renderer.renderStats().activeSceneryObjects == (duplicateId ? 5U : 6U),
          "Both source tiles should retain their rows and any resolved child");
    check(debugPaths() == 5, "Both source tiles should prepare five debug paths");
    std::array<std::vector<double>, 2> starts;
    for (const auto& segment : renderer.sceneryTrafficPathSegments()) {
        if (segment.mapObjectId == 101 || segment.mapObjectId == 102) {
            check(segment.tileX == 0 && segment.tileY >= 0 && segment.tileY < 2 &&
                      !segment.points.empty(),
                  "Spline row debug path must retain its source tile and geometry");
            starts[static_cast<std::size_t>(segment.tileY)].push_back(segment.points.front()[1]);
        }
    }
    for (auto& values : starts) {
        std::sort(values.begin(), values.end());
    }
    check(starts[0].size() == 3 && starts[1].size() == 2 &&
              std::abs(starts[0][0] - 295.0) < 0.001 &&
              std::abs(starts[0][1] - starts[1][0]) < 0.001 &&
              std::abs(starts[0][2] - starts[1][1]) < 0.001,
          "Traffic debug paths must expose the two overlapping cross-tile rows");
    std::array<std::vector<double>, 2> routeStarts;
    for (const int objectId : {101, 102}) {
        for (const auto& lane : renderer.lanesForTimetableEntry(objectId, 0)) {
            const auto route = renderer.buildDrivingRoute(objectId, lane.laneSelector);
            check(route.has_value() && !route->segments.empty() &&
                      !route->segments.front().points.empty(),
                  "Every indexed row lane must produce a driving route");
            const auto& segment = route->segments.front();
            check(segment.spline.tileX == 0 && segment.spline.tileY >= 0 &&
                      segment.spline.tileY < 2,
                  "Driving route must retain the row's source tile");
            routeStarts[static_cast<std::size_t>(segment.spline.tileY)].push_back(
                segment.points.front()[1]);
        }
    }
    for (auto& values : routeStarts) {
        std::sort(values.begin(), values.end());
    }
    for (std::size_t tile = 0; tile < starts.size(); ++tile) {
        check(routeStarts[tile].size() == starts[tile].size(),
              "Driving routes must retain all prepared row paths");
        for (std::size_t index = 0; index < starts[tile].size(); ++index) {
            check(std::abs(routeStarts[tile][index] - starts[tile][index]) < 0.001,
                  "Driving route and debug path starts must coincide");
        }
    }
    waitFor(900, {{0, 1}}, duplicateId ? 5 : 3, duplicateId ? 0 : 2);
    check(renderer.renderStats().activeSceneryObjects == 2,
          "Only the repeater rows should be active in the repeater-only window");
}

void runSplineHumanDependencyProbe(const std::filesystem::path& root, bool duplicateId,
                                   bool referenced, bool repeaterFirst) {
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "Splines");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 0;0;0;; TextureFilename { \"base.tga\"; } } } }\n");
    writeText(root / "row.sco",
              "[new_attachment]\nmount\nattach_trans\n0\n0\n0\n"
              "[path]\n0\n0\n0\n0\n0\n10\n0\n0\n1\n2\n0\n0\n"
              "[mesh]\nsurface.x\n");
    writeText(root / "child.sco", "[mesh]\nsurface.x\n");
    const std::string profile =
        "[texture]\nbase.tga\n[profile]\n0\n[profilepnt]\n-2\n0\n0\n0\n"
        "[profilepnt]\n2\n0\n1\n0\n";
    writeText(root / "Splines" / "host.sli", profile + "[path]\n1\n0\n0\n0\n0\n");
    writeText(root / "Splines" / "next.sli", profile);
    writeText(root / "tile0.map",
              std::string("[version]\n14\n[terrain]\n"
                          "[spline]\n0\nSplines\\host.sli\n10\n0\n11\n0\n0\n290\n0\n10\n"
                          "0\n0\n0\n0\n0\n0\n0\n0\n"
                          "[splineAttachement]\n0\nrow.sco\n101\n0\n0\n0\n10\n0\n0\n0\n"
                          "0\n0\n0\n0\n") +
                  (referenced ? "[attachObj]\n0\nchild.sco\n200\n101\n0\n0\n0\n0\n0\n0\n" : ""));
    writeText(root / "tile1.map",
              "[version]\n14\n[terrain]\n"
              "[spline]\n0\nSplines\\next.sli\n11\n10\n0\n0\n0\n0\n0\n20\n"
              "0\n0\n0\n0\n0\n0\n0\n10\n"
              "[splineAttachement_repeater]\n0\n0\n0\nrow.sco\n" +
                  std::string(duplicateId ? "101" : "102") +
                  "\n0\n0\n0\n10\n0\n0\n0\n0\n0\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    for (const auto& name : {"tile0.terrain", "tile1.terrain"}) {
        std::ofstream terrainFile(root / name, std::ios::binary);
        terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
        check(static_cast<bool>(terrainFile), "Could not write pedestrian dependency terrain");
    }
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    for (int y = 0; y < 2; ++y) {
        openbus::map::MapTileReference tile;
        tile.y = y;
        const std::string prefix = "tile" + std::to_string(y);
        tile.textPath = root / (prefix + ".map");
        tile.terrainPath = root / (prefix + ".terrain");
        tile.hasTerrainFile = true;
        map.tiles.push_back(tile);
    }
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, -900, 0, root, simulation, assets);
    const auto activate = [&](double y, const std::vector<std::pair<int, int>>& window) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        do {
            renderer.updateStreaming(0, y);
            if (renderer.initialWindowReady() && renderer.activeTileCoordinates() == window &&
                renderer.humanTrafficLaneNetworkReady()) {
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } while (std::chrono::steady_clock::now() < deadline);
        throw std::runtime_error("Pedestrian dependency window did not settle");
    };
    const auto journey = [&]() {
        return renderer.passengerPedestrianJourneyBetween({0, 298, 0}, {0, 308, 0}, 0, 0.25, 50);
    };
    const auto requireJourney = [&](bool expected, int expectedSegmentId,
                                    int expectedSegmentTileY = 0) {
        const auto result = journey();
        check(result.has_value() == expected,
              "Pedestrian journey did not match the available attachment paths");
        if (result) {
            check(result->route && result->route->segments.size() >= 2,
                  "Pedestrian journey must cross from the spline to the attachment path");
            const auto& segment = result->route->segments[1];
            const bool expectedSegment =
                segment.spline.placement.splineId == expectedSegmentId &&
                segment.spline.tileX == 0 && segment.spline.tileY == expectedSegmentTileY &&
                !segment.points.empty() &&
                std::abs(segment.points.front()[1] - 300.0) < 0.001;
            if (!expectedSegment) {
                std::cerr << "Pedestrian segment expected spline=" << expectedSegmentId
                          << " at tile=0:" << expectedSegmentTileY << " y=300; got spline="
                          << segment.spline.placement.splineId << " tile=" << segment.spline.tileX
                          << ':' << segment.spline.tileY << " y="
                          << (segment.points.empty() ? -1.0 : segment.points.front()[1])
                          << " in " << root.filename().string() << " with active tiles";
                for (const auto& [tileX, tileY] : renderer.activeTileCoordinates()) {
                    std::cerr << ' ' << tileX << ':' << tileY;
                }
                std::cerr << '\n';
            }
            check(expectedSegment,
                  "Pedestrian attachment segment has unexpected identity or position");
        }
        check(!renderer.passengerPedestrianJourneyBetween({0, 304, 0}, {0, 308, 0}, 0, 0.25, 50),
              "Attachment-only pedestrian path is not a journey start candidate");
    };
    const auto debugRows = [&]() {
        const auto& segments = renderer.sceneryTrafficPathSegments();
        return std::count_if(segments.begin(), segments.end(), [](const auto& segment) {
            return (segment.mapObjectId == 101 || segment.mapObjectId == 102) &&
                   segment.pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN;
        });
    };
    activate(-900, {});
    requireJourney(!duplicateId, 101);
    check(debugRows() == 0, "Unloaded pedestrian rows must not have prepared debug paths");
    if (repeaterFirst) {
        activate(900, {{0, 1}});
        requireJourney(true, 101, 1);
        check(debugRows() == 1, "Repeater tile must prepare its pedestrian debug path");
        const auto& segments = renderer.sceneryTrafficPathSegments();
        const auto repeater =
            std::find_if(segments.begin(), segments.end(), [](const auto& segment) {
                return segment.mapObjectId == 101 &&
                       segment.pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN;
            });
        check(repeater != segments.end() && repeater->tileX == 0 && repeater->tileY == 1,
              "Repeater debug path must retain its actual source tile");
    } else {
        activate(-600, {{0, 0}});
        requireJourney(true, 101);
        check(debugRows() == 1, "Regular tile must prepare its pedestrian debug path");
    }
    activate(0, {{0, 0}, {0, 1}});
    requireJourney(true, 101, repeaterFirst ? 1 : 0);
    check(debugRows() == 2, "Both source tiles must prepare pedestrian debug paths");
    std::array<bool, 2> sourceTiles = {};
    for (const auto& segment : renderer.sceneryTrafficPathSegments()) {
        if ((segment.mapObjectId == 101 || segment.mapObjectId == 102) &&
            segment.pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN) {
            check(segment.tileX == 0 && segment.tileY >= 0 && segment.tileY < 2 &&
                      !segment.points.empty() &&
                      std::abs(segment.points.front()[1] - 300.0) < 0.001,
                  "Pedestrian debug paths must coincide and retain their source tiles");
            sourceTiles[static_cast<std::size_t>(segment.tileY)] = true;
        }
    }
    check(sourceTiles[0] && sourceTiles[1],
          "Each source tile must contribute a pedestrian debug path");
    activate(900, {{0, 1}});
    requireJourney(true, 101, repeaterFirst ? 1 : 0);
}

void runItemLightmapProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "script");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeSolidTexture(root / "blue.tga", {0, 0, 255});
    writeSolidTexture(root / "red.tga", {255, 0, 0});
    writeSolidTexture(root / "green.tga", {0, 255, 0});
    writeSolidTexture(root / "white.tga", {255, 255, 255});
    writeText(root / "invalid.tga", "invalid image");
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -1.7;0;-2;, 1.7;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshTextureCoords { 3; 0.5;0.5;, 0.5;0.5;, 0.5;0.5;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 0;0;0;; TextureFilename { \"base.tga\"; } } } }\n");
    const std::string script =
        "[script]\n1\nscript\\lamp.osc\n[varnamelist]\n1\nscript\\vars.txt\n"
        "[stringvarnamelist]\n1\nscript\\strings.txt\n";
    const std::string mesh = "[mesh]\nsurface.x\n[matl]\nbase.tga\n0\n";
    const std::string change = "[matl_change]\nbase.tga\n0\nselector\n";
    writeText(root / "items.sco", script + mesh + "[matl_lightmap]\nblue.tga\nbase_power\n" +
              change + "[matl_item]\n[matl_lightmap]\nred.tga\nred_power\n"
                       "[matl_item]\n[matl_lightmap]\ngreen.tga\ngreen_power\n");
    writeText(root / "uncontrolled.sco", script + mesh + change +
              "[matl_item]\n[matl_lightmap]\nwhite.tga\n");
    writeText(root / "missing.sco", script + mesh + "[matl_lightmap]\nblue.tga\nbase_power\n" +
              change + "[matl_item]\n[matl_lightmap]\nmissing.tga\nred_power\n");
    writeText(root / "invalid.sco", script + mesh + "[matl_lightmap]\nblue.tga\nbase_power\n" +
              change + "[matl_item]\n[matl_lightmap]\ninvalid.tga\nred_power\n");
    writeText(root / "reference.sco", mesh);
    writeText(root / "script" / "vars.txt",
              "instance_id\nselector\nbase_power\nred_power\ngreen_power\n");
    writeText(root / "script" / "strings.txt", "placement_id\n");
    writeText(root / "script" / "lamp.osc",
              "{init}\n(L.$.placement_id) $StrToFloat (S.L.instance_id)\n{end}\n"
              "{frame}\n"
              "(L.S.test_selector) (L.L.instance_id) * (S.L.selector)\n"
              "1 (S.L.base_power)\n"
              "(L.S.test_red) (S.L.red_power)\n"
              "(L.S.test_green) (S.L.green_power)\n{end}\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\nitems.sco\n101\n-10\n0\n5\n0\n0\n0\n1\n1\n"
              "[object]\n0\nitems.sco\n102\n-6\n0\n5\n0\n0\n0\n1\n2\n"
              "[object]\n0\nuncontrolled.sco\n103\n-2\n0\n5\n0\n0\n0\n1\n1\n"
              "[object]\n0\nmissing.sco\n104\n2\n0\n5\n0\n0\n0\n1\n1\n"
              "[object]\n0\ninvalid.sco\n105\n6\n0\n5\n0\n0\n0\n1\n1\n"
              "[object]\n0\nreference.sco\n106\n10\n0\n5\n0\n0\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write scenery item-lightmap terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 6,
          "Item-lightmap scenery or optional-map fallback did not load");
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, 0, -16}));
    using Pixel = std::array<std::uint8_t, 4>;
    const auto draw = [&](float selector, float red, float green) {
        simulation.sharedVariables().set("test_selector", selector);
        simulation.sharedVariables().set("test_red", red);
        simulation.sharedVariables().set("test_green", green);
        renderer.updateScripts();
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer.draw();
        std::array<Pixel, 6> pixels = {};
        const std::array<int, 6> x = {3, 14, 26, 38, 49, 61};
        for (std::size_t index = 0; index < x.size(); ++index) {
            glReadPixels(x[index], 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels[index].data());
        }
        return pixels;
    };
    const auto base = draw(0, 1, 1);
    const auto off = draw(1, 0, 0);
    const auto half = draw(1, 0.5F, 0.5F);
    const auto full = draw(1, 1, 1);
    check(base[0] == base[1] && base[0] == base[3] && base[0] == base[4] &&
              base[0][2] > base[0][0] && base[2] == base[5] && base[5][3] > 0,
          "Base and unlit lightmap states did not load");
    check(off[0] == base[5] && off[1] == base[5] &&
              half[0][0] > off[0][0] && half[0][0] < full[0][0] &&
              half[1][1] > off[1][1] && half[1][1] < full[1][1] &&
              full[0][0] > full[0][1] && full[1][1] > full[1][0],
          "Item masks or independent lightmap controllers did not render correctly");
    check(full[2][0] > base[2][0] && full[2][0] == full[2][1] &&
              full[3] == base[5] && full[4] == base[5],
          "Uncontrolled item or unavailable lightmap fallback failed");
    const auto independent = draw(1, 1, 0);
    check(independent[0] == full[0] && independent[1] == off[1],
          "Selected lightmap controllers are not independent");
    check(draw(1, -1, 2)[0] == off[0] && draw(1, -1, 2)[1] == full[1] &&
              draw(0, 1, 1) == base && draw(3, 1, 1)[0] == base[0],
          "Item lightmap clamping or base restoration failed");

    writeText(root / "required.sco", mesh + change +
              "[matl_item]\n[matl_lightmap]\ninvalid.tga\n"
              "[mesh]\nsurface.x\n[matl]\ninvalid.tga\n0\n");
    writeText(root / "required.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\nrequired.sco\n201\n0\n0\n5\n0\n0\n0\n0\n");
    openbus::map::MapDefinition requiredMap = map;
    requiredMap.tiles[0].textPath = root / "required.map";
    bool requiredTextureFailed = false;
    try {
        AssetRequestManager requiredAssets;
        MapRenderer requiredRenderer(requiredMap, 0, 0, 0, 0, root, simulation, requiredAssets);
    } catch (const std::runtime_error& error) {
        requiredTextureFailed = std::string_view(error.what()).find(
                                    "Could not decode scenery texture") != std::string_view::npos;
    }
    check(requiredTextureFailed,
          "An item lightmap failure must not hide a later required texture failure");
}

void runNightmapProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "script");
    writeSolidTexture(root / "base.tga", {100, 100, 100});
    writeSolidTexture(root / "blue.tga", {0, 0, 255});
    writeSolidTexture(root / "red.tga", {255, 0, 0});
    writeSolidTexture(root / "green.tga", {0, 255, 0});
    writeText(root / "invalid.tga", "invalid image");
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -1.7;0;-2;, 1.7;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshTextureCoords { 3; 0.5;0.5;, 0.5;0.5;, 0.5;0.5;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 1;1;1;; TextureFilename { \"base.tga\"; } } } }\n");
    const std::string script =
        "[script]\n1\nscript\\lamp.osc\n[varnamelist]\n1\nscript\\vars.txt\n"
        "[stringvarnamelist]\n1\nscript\\strings.txt\n";
    const std::string mesh = "[mesh]\nsurface.x\n[matl]\nbase.tga\n0\n";
    writeText(root / "items.sco", script + mesh + "[matl_nightmap]\nblue.tga\n"
              "[matl_change]\nbase.tga\n0\nselector\n"
              "[matl_item]\n[matl_nightmap]\nred.tga\n"
              "[matl_item]\n[matl_nightmap]\ngreen.tga\n");
    writeText(root / "base.sco", script + mesh + "[matl_nightmap]\nblue.tga\n");
    writeText(root / "visible.sco", script + mesh +
              "[matl_nightmap]\nblue.tga\n[visible]\nvisible_flag\n1\n");
    writeText(root / "missing.sco", script + mesh + "[matl_nightmap]\nblue.tga\n" +
              "[matl_change]\nbase.tga\n0\nselector\n"
              "[matl_item]\n[matl_nightmap]\nmissing.tga\n");
    writeText(root / "invalid.sco", script + mesh + "[matl_nightmap]\nblue.tga\n" +
              "[matl_change]\nbase.tga\n0\nselector\n"
              "[matl_item]\n[matl_nightmap]\ninvalid.tga\n");
    writeText(root / "script" / "vars.txt", "instance_id\nselector\nvisible_flag\n");
    writeText(root / "script" / "strings.txt", "placement_id\n");
    writeText(root / "script" / "lamp.osc",
              "{init}\n(L.$.placement_id) $StrToFloat (S.L.instance_id)\n{end}\n"
              "{frame}\n"
              "(L.S.test_selector) (L.L.instance_id) * (S.L.selector)\n"
              "(L.S.test_night) (S.L.nightlighta)\n"
              "1 (S.L.visible_flag)\n{end}\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\nitems.sco\n101\n-10\n0\n5\n0\n0\n0\n1\n1\n"
              "[object]\n0\nitems.sco\n102\n-6\n0\n5\n0\n0\n0\n1\n2\n"
              "[object]\n0\nbase.sco\n103\n-2\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nvisible.sco\n104\n2\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nmissing.sco\n105\n6\n0\n5\n0\n0\n0\n1\n1\n"
              "[object]\n0\ninvalid.sco\n106\n10\n0\n5\n0\n0\n0\n1\n1\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write scenery nightmap terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 6,
          "Nightmap scenery or optional-map fallback did not load");
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, 0, -16}));
    using Pixel = std::array<std::uint8_t, 4>;
    const auto draw = [&](float selector, float night) {
        simulation.sharedVariables().set("test_selector", selector);
        simulation.sharedVariables().set("test_night", night);
        renderer.updateScripts();
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer.draw();
        std::array<Pixel, 6> pixels = {};
        const std::array<int, 6> x = {3, 14, 26, 38, 49, 61};
        for (std::size_t index = 0; index < x.size(); ++index) {
            glReadPixels(x[index], 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels[index].data());
        }
        return pixels;
    };
    const auto day = draw(0, 0);
    const auto half = draw(0, 0.5F);
    const auto night = draw(0, 1);
    const auto selected = draw(1, 0);
    const auto selectedNight = draw(1, 1);
    const auto secondItem = draw(2, 0);
    check(day[0] == day[1] && day[0] == day[2] && day[0] == day[4] &&
              day[0] == day[5] && day[0][3] > 0,
          "Nightmap base materials did not match in daylight");
    check(night[0] == night[1] && night[0] == night[2] &&
              night[0][2] > 250 && night[0][0] < 3 &&
              half[2][2] > day[2][2] && half[2][2] < night[2][2],
          "Base nightmap did not follow its per-placement controller");
    check(day[3] == night[3] && day[3][2] > 250,
          "Visible-gated nightmap did not illuminate in daylight");
    check(selected[0][0] > 250 && selected[0][1] < 3 &&
              selected[1][1] > 250 && selected[1][0] < 3 &&
              selected[4] == day[4] && selected[5] == day[5],
          "Item selection, per-placement texture binding, or optional fallback failed");
    check(night[4] == night[2] && night[5] == night[2] &&
              selectedNight[4] == day[4] && selectedNight[5] == day[5] &&
              secondItem[0][1] > 250 && secondItem[1] == day[1],
          "Unavailable or out-of-range items did not clear or restore the base nightmap");
    check(draw(0, 0) == day && draw(0, -1)[2] == day[2] &&
              draw(0, 2)[2] == night[2],
          "Nightmap state was not restored or controller clamping failed");
    auto weather = simulation.sharedVariables().weather();
    weather.brightness = 0.5;
    simulation.sharedVariables().setWeather(weather);
    setWeatherLighting(weather.brightness);
    const auto weatherHalf = draw(0, 0);
    weather.brightness = 0.0;
    simulation.sharedVariables().setWeather(weather);
    setWeatherLighting(weather.brightness);
    const auto weatherNight = draw(0, 0);
    const auto weatherNightHalf = draw(0, 0.5F);
    const auto weatherNightFull = draw(0, 1);
    const auto weatherSelected = draw(1, 0);
    check(weatherHalf[2][2] == weatherHalf[2][0],
          "Partial weather brightness overrode scripted nightmap off");
    check(weatherNight[2][2] == weatherNight[2][0] &&
              weatherNightHalf[2][2] > weatherNight[2][2] &&
              weatherNightHalf[2][2] < weatherNightFull[2][2] &&
              weatherNightFull[2][2] > 250,
          "Dark weather overrode scripted nightmap off, half, or full");
    check(weatherNight[3][2] > 250 && weatherSelected[0][0] > 250 &&
              weatherSelected[1][1] > 250,
          "Dark weather changed visible-gated or selected-item nightmaps");
    weather.brightness.reset();
    simulation.sharedVariables().setWeather(weather);
    setWeatherLighting(std::nullopt);
    check(draw(0, 0) == day,
          "Removing weather brightness did not restore scenery daylight");

    writeText(root / "static.sco", mesh + "[matl_nightmap]\nblue.tga\n");
    writeText(root / "unrelated.sco",
              "[script]\n1\nscript\\unrelated.osc\n" + mesh + "[matl_nightmap]\nblue.tga\n");
    writeText(root / "script" / "unrelated.osc",
              "{frame}\n1 (S.L.unrelated_variable)\n{end}\n");
    writeText(root / "static.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\nstatic.sco\n107\n-2\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nunrelated.sco\n108\n2\n0\n5\n0\n0\n0\n0\n");
    openbus::map::MapDefinition staticMap = map;
    staticMap.tiles[0].textPath = root / "static.map";
    SimulationState staticSimulation;
    MapRenderer staticRenderer(staticMap, 0, 0, 0, 0, root, staticSimulation, assets);
    check(staticRenderer.renderStats().activeSceneryObjects == 2,
          "Uncontrolled nightmap scenery did not load");
    const auto drawStatic = [&]() {
        staticRenderer.updateScripts();
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        staticRenderer.draw();
        std::array<Pixel, 2> result = {};
        glReadPixels(26, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, result[0].data());
        glReadPixels(38, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, result[1].data());
        return result;
    };
    const auto staticDay = drawStatic();
    auto staticWeather = staticSimulation.sharedVariables().weather();
    staticWeather.brightness = 0.0;
    staticSimulation.sharedVariables().setWeather(staticWeather);
    setWeatherLighting(staticWeather.brightness);
    const auto staticNight = drawStatic();
    check(staticDay[0][3] > 0 && staticDay[1][3] > 0 &&
              staticNight[0][2] > 250 && staticNight[0][0] < 3 &&
              staticNight[1][2] > 250 && staticNight[1][0] < 3,
          "Weather brightness did not activate uncontrolled scenery nightmaps");
    setWeatherLighting(std::nullopt);
}

void runBumpmapProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeSolidTexture(root / "bump.tga", {128, 220, 220});
    writeSolidTexture(root / "alternate.tga", {35, 128, 220});
    writeText(root / "invalid.tga", "invalid image");
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -1.2;0;-2;, 1.2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshNormals { 1; 0;1;0;; 2; 3;0,0,0;, 3;0,0,0;; }\n"
              "MeshTextureCoords { 3; 0;0;, 1;0;, 0.5;1;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 0;0;0;; TextureFilename { \"base.tga\"; } } } }\n");
    const std::string mesh = "[mesh]\nsurface.x\n[matl]\nbase.tga\n0\n";
    writeText(root / "base.sco", mesh);
    writeText(root / "zero.sco", mesh + "[matl_bumpmap]\nbump.tga\n0\n");
    writeText(root / "half.sco", mesh + "[matl_bumpmap]\nbump.tga\n0.5\n");
    writeText(root / "full.sco", mesh + "[matl_bumpmap]\nbump.tga\n1\n");
    writeText(root / "high.sco", mesh + "[matl_bumpmap]\nbump.tga\n2\n");
    writeText(root / "negative.sco", mesh + "[matl_bumpmap]\nbump.tga\n-1\n");
    writeText(root / "missing.sco", mesh + "[matl_bumpmap]\nmissing.tga\n1\n");
    writeText(root / "invalid.sco", mesh + "[matl_bumpmap]\ninvalid.tga\n1\n");
    writeText(root / "alternate.sco", mesh + "[matl_bumpmap]\nalternate.tga\n1\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\nbase.sco\n101\n-10\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nzero.sco\n102\n-7.5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nhalf.sco\n103\n-5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nfull.sco\n104\n-2.5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nhigh.sco\n105\n0\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nnegative.sco\n106\n2.5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nmissing.sco\n107\n5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\ninvalid.sco\n108\n7.5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nalternate.sco\n109\n10\n0\n5\n0\n0\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write scenery bumpmap terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 9,
          "Bumpmap scenery or invalid-bumpmap fallback did not load");
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, 0, -16}));
    glDepthMask(GL_TRUE);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    renderer.draw();
    using Pixel = std::array<std::uint8_t, 4>;
    std::array<Pixel, 9> pixels = {};
    const std::array<int, 9> x = {3, 10, 17, 25, 32, 39, 46, 54, 61};
    for (std::size_t index = 0; index < x.size(); ++index) {
        glReadPixels(x[index], 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels[index].data());
    }
    if (!(pixels[0][0] > pixels[2][0] && pixels[2][0] > pixels[3][0]) ||
        pixels[3] == pixels[8]) {
        std::cerr << "Bumpmap samples:";
        for (const auto& pixel : pixels) {
            std::cerr << ' ' << static_cast<int>(pixel[0]) << ',' << static_cast<int>(pixel[3]);
        }
        std::cerr << '\n';
    }
    check(pixels[0][0] > pixels[2][0] && pixels[2][0] > pixels[3][0],
          "Bumpmap strength did not change surface shading");
    check(pixels[0] == pixels[1] && pixels[0] == pixels[5] && pixels[0] == pixels[6] &&
              pixels[0] == pixels[7] && pixels[3] == pixels[4],
          "Bumpmap strength clamping or invalid/missing texture fallback failed");
    check(pixels[3] != pixels[8], "Different bumpmap normals produced the same shading");

    writeText(root / "required.sco",
              mesh + "[matl_bumpmap]\ninvalid.tga\n1\n"
                     "[mesh]\nsurface.x\n[matl]\ninvalid.tga\n0\n");
    writeText(root / "required.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\nrequired.sco\n201\n0\n0\n5\n0\n0\n0\n0\n");
    openbus::map::MapDefinition requiredMap = map;
    requiredMap.tiles[0].textPath = root / "required.map";
    bool requiredTextureFailed = false;
    try {
        AssetRequestManager requiredAssets;
        MapRenderer requiredRenderer(requiredMap, 0, 0, 0, 0, root, simulation, requiredAssets);
    } catch (const std::runtime_error& error) {
        requiredTextureFailed = std::string_view(error.what()).find(
                                    "Could not decode scenery texture") != std::string_view::npos;
    }
    check(requiredTextureFailed,
          "A failed optional map must not hide a later required base texture failure");
}

void runEnvironmentProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    writeSolidTexture(root / "base.tga", {100, 100, 100});
    writeSolidTexture(root / "environment.tga", {255, 0, 0});
    writeText(root / "invalid.tga", "invalid image");
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -1.2;0;-2;, 1.2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshNormals { 1; 0;0;-1;; 2; 3;0,0,0;, 3;0,0,0;; }\n"
              "MeshTextureCoords { 3; 0;0;, 1;0;, 0.5;1;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 1;1;1;; TextureFilename { \"base.tga\"; } } } }\n");
    const std::string mesh = "[mesh]\nsurface.x\n[matl]\nbase.tga\n0\n";
    writeText(root / "base.sco", mesh);
    writeText(root / "zero.sco", mesh + "[matl_envmap]\nenvironment.tga\n0\n");
    writeText(root / "half.sco", mesh + "[matl_envmap]\nenvironment.tga\n0.5\n");
    writeText(root / "one.sco", mesh + "[matl_envmap]\nenvironment.tga\n1\n");
    writeText(root / "full.sco", mesh + "[matl_envmap]\nenvironment.tga\n10\n");
    writeText(root / "high.sco", mesh + "[matl_envmap]\nenvironment.tga\n20\n");
    writeText(root / "negative.sco", mesh + "[matl_envmap]\nenvironment.tga\n-1\n");
    writeText(root / "missing.sco", mesh + "[matl_envmap]\nmissing.tga\n1\n");
    writeText(root / "invalid.sco", mesh + "[matl_envmap]\ninvalid.tga\n1\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\nbase.sco\n101\n-10\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nzero.sco\n102\n-7.5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nhalf.sco\n103\n-5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\none.sco\n104\n-2.5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nfull.sco\n105\n0\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nhigh.sco\n106\n2.5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nnegative.sco\n107\n5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nmissing.sco\n108\n7.5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\ninvalid.sco\n109\n10\n0\n5\n0\n0\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write scenery environment terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 9,
          "Environment scenery or optional-map fallback did not load");
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, 0, -16}));
    glDepthMask(GL_TRUE);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    renderer.draw();
    using Pixel = std::array<std::uint8_t, 4>;
    std::array<Pixel, 9> pixels = {};
    const std::array<int, 9> x = {3, 10, 17, 25, 32, 39, 46, 54, 61};
    for (std::size_t index = 0; index < x.size(); ++index) {
        glReadPixels(x[index], 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels[index].data());
    }
    const auto close = [](int actual, int expected) { return std::abs(actual - expected) <= 2; };
    if (!(close(pixels[2][0], 108) && close(pixels[3][0], 116) &&
          pixels[4][0] > 250 && pixels[4][1] < 3 &&
          pixels[0] == pixels[1] && pixels[0] == pixels[6] &&
          pixels[0] == pixels[7] && pixels[0] == pixels[8] && pixels[4] == pixels[5])) {
        std::cerr << "Environment samples:";
        for (const auto& pixel : pixels) {
            std::cerr << ' ' << static_cast<int>(pixel[0]) << ',' << static_cast<int>(pixel[1])
                      << ',' << static_cast<int>(pixel[3]);
        }
        std::cerr << '\n';
        throw std::runtime_error("Environment strength, clamping, or optional fallback failed");
    }
}

void runProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "script");
    writeTexture(root / "uv.tga");
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshTextureCoords { 3; 0.25;0.25;, 0.25;0.25;, 0.25;0.25;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 1;1;1;; TextureFilename { \"uv.tga\"; } } } }\n");
    writeText(root / "scroll.sco",
              "[script]\n1\nscript\\scroll.osc\n"
              "[varnamelist]\n1\nscript\\scroll_varlist.txt\n"
              "[stringvarnamelist]\n1\nscript\\scroll_strings.txt\n"
              "[mesh]\nsurface.x\n[matl]\nuv.tga\n0\n"
              "[texcoordtransX]\nscroll_x\n[texcoordtransY]\nscroll_y\n");
    writeText(root / "script" / "scroll_varlist.txt", "instance_id\nscroll_x\nscroll_y\n");
    writeText(root / "script" / "scroll_strings.txt", "placement_id\n");
    writeText(root / "script" / "scroll.osc",
              "{init}\n(L.$.placement_id) $StrToFloat (S.L.instance_id)\n{end}\n"
              "{frame}\n"
              "(L.S.uv_test_x) (L.L.instance_id) * (S.L.scroll_x)\n"
              "(L.S.uv_test_y) (L.L.instance_id) * (S.L.scroll_y)\n{end}\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\nscroll.sco\n101\n-3\n0\n5\n0\n0\n0\n1\n1\n"
              "[object]\n0\nscroll.sco\n102\n3\n0\n5\n0\n0\n0\n1\n2\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write scenery UV terrain");
    terrainFile.close();

    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"uv.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 2, "UV scenery did not load");
    glViewport(0, 0, 32, 32);
    glDisable(GL_DITHER);
    setPerspective(32, 32, 90);
    setModelViewMatrix(translationMatrix({0, 0, -12}));

    using Pixel = std::array<std::uint8_t, 4>;
    const auto draw = [&](float x, float y) {
        simulation.sharedVariables().set("uv_test_x", x);
        simulation.sharedVariables().set("uv_test_y", y);
        renderer.updateScripts();
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer.draw();
        std::array<Pixel, 2> result = {};
        glReadPixels(9, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, result[0].data());
        glReadPixels(22, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, result[1].data());
        return result;
    };
    const auto base = draw(0, 0);
    const auto horizontal = draw(0.5F, 0);
    const auto vertical = draw(0, 0.5F);
    if (base[0] != base[1] || base[0][3] == 0) {
        std::cerr << "Base samples: ";
        for (const auto& pixel : base) {
            std::cerr << static_cast<int>(pixel[0]) << ',' << static_cast<int>(pixel[1])
                      << ',' << static_cast<int>(pixel[2]) << ',' << static_cast<int>(pixel[3])
                      << ' ';
        }
        std::cerr << '\n';
    }
    check(base[0] == base[1] && base[0][3] > 0, "Base UV samples do not match");
    check((horizontal[0] == base[0]) != (horizontal[1] == base[1]),
          "X translation did not vary independently per placement");
    check((vertical[0] == base[0]) != (vertical[1] == base[1]),
          "Y translation did not vary independently per placement");
    check((horizontal[0] == base[0]) == (vertical[0] == base[0]),
          "Different placements responded to X and Y translation");
    const Pixel xChanged = horizontal[0] == base[0] ? horizontal[1] : horizontal[0];
    const Pixel yChanged = vertical[0] == base[0] ? vertical[1] : vertical[0];
    check(base[0][0] < 15 && base[0][1] < 15 && base[0][2] > 240 &&
              xChanged[0] > 240 && xChanged[1] > 240 && xChanged[2] < 15 &&
              yChanged[0] > 240 && yChanged[1] < 15 && yChanged[2] < 15,
          "UV translation sampled the wrong texture quadrant");
    check(draw(0, 0) == base, "UV translation did not reset");
}

} // namespace

int main(int argc, char** argv) {
    const bool lightmap = argc == 2 && std::string_view(argv[1]) == "--lightmap";
    const bool bumpmap = argc == 2 && std::string_view(argv[1]) == "--bumpmap";
    const bool environment = argc == 2 && std::string_view(argv[1]) == "--envmap";
    const bool nightmap = argc == 2 && std::string_view(argv[1]) == "--nightmap";
    const bool itemLightmap = argc == 2 && std::string_view(argv[1]) == "--item-lightmap";
    const bool normals = argc == 2 && std::string_view(argv[1]) == "--normals";
    const bool cutout = argc == 2 && std::string_view(argv[1]) == "--cutout";
    const bool treeSeason = argc == 2 && std::string_view(argv[1]) == "--tree-season";
    const bool meshSeason = argc == 2 && std::string_view(argv[1]) == "--mesh-season";
    const bool waterCorners = argc == 2 && std::string_view(argv[1]) == "--water-corners";
    const bool human = argc == 2 && std::string_view(argv[1]) == "--human-skinning";
    const bool splineLabels = argc == 2 && std::string_view(argv[1]) == "--spline-labels";
    const bool roadWeather = argc == 2 && std::string_view(argv[1]) == "--road-weather";
    const bool terrainWeather = argc == 2 && std::string_view(argv[1]) == "--terrain-weather";
    const bool splineText = argc == 2 && std::string_view(argv[1]) == "--spline-text";
    const bool splineOwnership = argc == 2 && std::string_view(argv[1]) == "--spline-ownership";
    const bool splineDependencies =
        argc == 2 && std::string_view(argv[1]) == "--spline-dependencies";
    const bool splineHumanDependencies =
        argc == 2 && std::string_view(argv[1]) == "--spline-human-dependencies";
    if (argc != 1 && !lightmap && !bumpmap && !environment && !nightmap && !itemLightmap &&
        !normals && !cutout && !treeSeason && !meshSeason && !waterCorners && !human && !splineLabels && !roadWeather && !terrainWeather &&
        !splineText && !splineOwnership &&
        !splineDependencies && !splineHumanDependencies) {
        return 2;
    }
    if (!glfwInit()) {
        return 77;
    }
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    const char* setting = std::getenv("OPENBUS_CULLING_BACKEND");
    const bool indirect = setting != nullptr && std::string_view(setting) == "compute-indirect";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, indirect ? 4 : 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(
        lightmap || bumpmap || environment || nightmap || itemLightmap || normals || cutout || treeSeason || meshSeason || waterCorners || human ||
                splineLabels || roadWeather || terrainWeather || splineText || splineOwnership ? 64 : 32,
        lightmap || bumpmap || environment || nightmap || itemLightmap || normals || cutout || treeSeason || meshSeason || waterCorners || human ||
                splineLabels || roadWeather || terrainWeather || splineText || splineOwnership ? 64 : 32,
                                          "Scenery material probe", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        return 77;
    }
    glfwMakeContextCurrent(window);
    const auto root = openbus::probe::makeProbeDirectory("openbus-scenery-uv-rendering", false);
    bool passed = loadOpenGLFunctions() && initializeCoreRenderer();
    const bool unavailable = passed && indirect && !indirectCullingAvailable();
    if (passed && !unavailable) {
        try {
            if (human) {
                runHumanSkinningProbe(root);
            } else if (normals) {
                runNormalProbe(root);
            } else if (cutout) {
                runCutoutProbe(root);
            } else if (treeSeason || meshSeason) {
                runTreeSeasonProbe(root, meshSeason);
            } else if (waterCorners) {
                runWaterCornerProbe(root);
            } else if (lightmap) {
                runLightmapProbe(root);
            } else if (bumpmap) {
                runBumpmapProbe(root);
            } else if (environment) {
                runEnvironmentProbe(root);
            } else if (nightmap) {
                runNightmapProbe(root);
            } else if (itemLightmap) {
                runItemLightmapProbe(root);
            } else if (splineLabels) {
                runSplineLabelsProbe(root);
            } else if (roadWeather) {
                runRoadWeatherProbe(root);
            } else if (terrainWeather) {
                runTerrainWeatherProbe(root);
            } else if (splineText) {
                runSplineTextProbe(root);
            } else if (splineOwnership) {
                runSplineOwnershipProbe(root);
            } else if (splineDependencies) {
                runSplineDependencyProbe(root / "unique", false);
                runSplineDependencyProbe(root / "duplicate", true);
            } else if (splineHumanDependencies) {
                runSplineHumanDependencyProbe(root / "unique", false, true, false);
                runSplineHumanDependencyProbe(root / "unreferenced", false, false, false);
                runSplineHumanDependencyProbe(root / "duplicate", true, true, false);
                runSplineHumanDependencyProbe(root / "repeater-first", true, true, true);
            } else {
                runProbe(root);
            }
        } catch (const std::exception& error) {
            std::cerr << error.what() << '\n';
            passed = false;
        }
    }
    shutdownCoreRenderer();
    glfwDestroyWindow(window);
    glfwTerminate();
    openbus::probe::removeProbeDirectory(root);
    return unavailable ? 77 : passed ? 0 : 1;
}
