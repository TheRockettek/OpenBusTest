#include "core/Variables.h"
#include "map/MapConfigLoader.h"
#include "render/AssetRequestManager.h"
#include "render/ComputeCulling.h"
#include "render/CoreRenderer.h"
#include "render/MatrixMath.h"
#include "render/OpenGLFunctions.h"
#include "render/map/MapRenderer.h"

#include <GLFW/glfw3.h>

#include <array>
#include <bit>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace openbus::rendering;

namespace {

BeginQueryProc beginQuery = nullptr;
GenQueriesProc genQueries = nullptr;
DrawArraysIndirectProc drawArraysIndirect = nullptr;
DrawElementsIndirectProc drawElementsIndirect = nullptr;
BufferDataProc bufferData = nullptr;
std::vector<GLuint> queries;
std::size_t indirectArraysDraws = 0;
std::size_t indirectElementsDraws = 0;
bool failNextProxyUpload = false;
bool failNextQueryAllocation = false;

void APIENTRY recordQuery(GLenum target, GLuint query) {
    queries.push_back(query);
    beginQuery(target, query);
}

void APIENTRY failQueryAllocation(GLsizei count, GLuint* output) {
    if (failNextQueryAllocation) {
        failNextQueryAllocation = false;
        std::fill_n(output, count, 0);
        return;
    }
    genQueries(count, output);
}

void APIENTRY recordIndirectArrays(GLenum mode, const void* offset) {
    ++indirectArraysDraws;
    drawArraysIndirect(mode, offset);
}

void APIENTRY recordIndirectElements(GLenum mode, GLenum type, const void* offset) {
    ++indirectElementsDraws;
    drawElementsIndirect(mode, type, offset);
}

void recordBufferData(GLenum target, std::ptrdiff_t size, const void* data, GLenum usage) {
    if (failNextProxyUpload && target == GL_ARRAY_BUFFER && usage == GL_STATIC_DRAW) {
        failNextProxyUpload = false;
        glEnable(0xffff);
        return;
    }
    bufferData(target, size, data, usage);
}

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void setQueries(bool enabled) {
#ifdef _WIN32
    require(_putenv_s("OPENBUS_OCCLUSION_CULLING", enabled ? "queries" : "") == 0,
            "Could not set query mode");
#else
    require(setenv("OPENBUS_OCCLUSION_CULLING", enabled ? "queries" : "", 1) == 0,
            "Could not set query mode");
#endif
}

void setTransparentQueries(bool enabled) {
#ifdef _WIN32
    require(_putenv_s("OPENBUS_OCCLUSION_TRANSPARENT", enabled ? "1" : "") == 0,
            "Could not set transparent query mode");
#else
    require(setenv("OPENBUS_OCCLUSION_TRANSPARENT", enabled ? "1" : "", 1) == 0,
            "Could not set transparent query mode");
#endif
}

void setGeometryQueries(bool enabled) {
#ifdef _WIN32
    require(_putenv_s("OPENBUS_OCCLUSION_MAP_GEOMETRY", enabled ? "1" : "") == 0,
            "Could not set geometry query mode");
#else
    require(setenv("OPENBUS_OCCLUSION_MAP_GEOMETRY", enabled ? "1" : "", 1) == 0,
            "Could not set geometry query mode");
#endif
}

void setWaterQueries(bool enabled) {
#ifdef _WIN32
    require(_putenv_s("OPENBUS_OCCLUSION_WATER", enabled ? "1" : "") == 0,
            "Could not set water query mode");
#else
    require(setenv("OPENBUS_OCCLUSION_WATER", enabled ? "1" : "", 1) == 0,
            "Could not set water query mode");
#endif
}

void writeText(const std::filesystem::path& path, const std::string& text) {
    std::ofstream output(path, std::ios::binary);
    output << text;
    require(static_cast<bool>(output), "Could not write map fixture");
}

void writeMesh(const std::filesystem::path& path, int size, const char* color,
               const char* alpha = "1") {
    const std::string extent = std::to_string(size);
    writeText(path, "xof 0303txt 0032\nMesh { 3; -" + extent + ";0;-" + extent +
                        ";, " + extent + ";0;-" + extent + ";, 0;0;" + extent +
                        ";; 2; 3;0,1,2;, 3;2,1,0;;\n"
                        "MeshMaterialList { 1; 2; 0,0;; Material { " + color +
                        ";" + alpha + ";; 0; 0;0;0;; 1;1;1;; } } }\n");
}

void runProbe(const std::filesystem::path& root) {
    setGeometryQueries(false);
    setWaterQueries(false);
    std::filesystem::create_directory(root / "model");
    writeMesh(root / "model" / "front.x", 8, "1;0;0");
    writeMesh(root / "model" / "back.x", 1, "0;1;0");
    writeMesh(root / "model" / "glass.x", 2, "0;0;1", "0.5");
    writeMesh(root / "model" / "transmap.x", 1, "1;0;1");
    writeMesh(root / "model" / "overlay.x", 1, "0;1;1");
    writeMesh(root / "model" / "no-write.x", 1, "1;1;0");
    writeText(root / "front.sco", "[mesh]\nfront.x\n");
    writeText(root / "back.sco", "[mesh]\nback.x\n");
    writeText(root / "glass.sco", "[mesh]\nglass.x\n");
    writeText(root / "transmap.sco",
              "[mesh]\ntransmap.x\n[matl]\nground.tga\n0\n[matl_alpha]\n2\n"
              "[matl_transmap]\nground.tga\n");
    writeText(root / "glass-no-write.sco",
              "[mesh]\nglass.x\n[matl]\nground.tga\n0\n[matl_alpha]\n2\n"
              "[matl_nozwrite]\n");
    writeText(root / "overlay.sco",
              "[mesh]\noverlay.x\n[matl]\nground.tga\n0\n[matl_nozcheck]\n");
    writeText(root / "no-write.sco",
              "[mesh]\nno-write.x\n[matl]\nground.tga\n0\n[matl_nozwrite]\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\nback.sco\n101\n0\n0\n0\n0\n0\n0\n0\n"
              "[object]\n0\nfront.sco\n102\n0\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nglass.sco\n103\n0\n0\n7\n0\n0\n0\n0\n"
              "[object]\n0\noverlay.sco\n104\n4\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nno-write.sco\n105\n-4\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nglass.sco\n106\n0\n0\n-5\n0\n0\n0\n0\n"
              "[object]\n0\ntransmap.sco\n107\n3\n0\n8\n0\n0\n0\n0\n"
              "[object]\n0\nglass-no-write.sco\n108\n-3\n0\n8\n0\n0\n0\n0\n"
              "[object]\n0\nglass.sco\n109\n7\n0\n0\n0\n0\n0\n0\n");
    std::array<std::uint8_t, 21> ground = {};
    ground[2] = 2;
    ground[12] = ground[14] = 1;
    ground[16] = 24;
    std::ofstream texture(root / "ground.tga", std::ios::binary);
    texture.write(reinterpret_cast<const char*>(ground.data()), ground.size());
    require(static_cast<bool>(texture), "Could not write ground texture");
    texture.close();
    std::array<std::uint8_t, 20> terrain = {};
    terrain[0] = 1;
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    require(static_cast<bool>(terrainFile), "Could not write terrain fixture");
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
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    require(renderer.renderStats().activeSceneryObjects == 9, "Fixture scenery did not load");

    using GetQueryObject = void(APIENTRY*)(GLuint, GLenum, GLuint*);
    const auto getQueryObject =
        reinterpret_cast<GetQueryObject>(glfwGetProcAddress("glGetQueryObjectuiv"));
    require(getQueryObject != nullptr, "Query result entry point unavailable");
    beginQuery = pglBeginQuery;
    require(beginQuery != nullptr, "Query entry point unavailable");
    pglBeginQuery = recordQuery;
    drawArraysIndirect = pglDrawArraysIndirect;
    require(drawArraysIndirect != nullptr, "Indirect draw entry point unavailable");
    pglDrawArraysIndirect = recordIndirectArrays;
    drawElementsIndirect = pglDrawElementsIndirect;
    require(drawElementsIndirect != nullptr, "Indexed indirect draw entry point unavailable");
    pglDrawElementsIndirect = recordIndirectElements;
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    const auto draw = [&](bool enabled, const Matrix4& view, bool cull, bool masked,
                          double threshold = 0.0) {
        setQueries(enabled);
        queries.clear();
        indirectArraysDraws = 0;
        setModelViewMatrix(view);
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glColorMask(GL_TRUE, masked ? GL_FALSE : GL_TRUE, GL_TRUE, GL_TRUE);
        if (cull) glEnable(GL_CULL_FACE); else glDisable(GL_CULL_FACE);
        {
            IndirectCullingScope modelPhase;
            renderer.draw(false, threshold);
            require(!drawSelectedIndirect(false), "Map draw leaked its indirect selection");
        }
        std::array<GLboolean, 4> colorMask = {};
        glGetBooleanv(GL_COLOR_WRITEMASK, colorMask.data());
        require(colorMask == std::array<GLboolean, 4>{
                                 GL_TRUE, static_cast<GLboolean>(masked ? GL_FALSE : GL_TRUE),
                                 GL_TRUE, GL_TRUE},
                "Color mask was not restored");
        require((glIsEnabled(GL_CULL_FACE) == GL_TRUE) == cull, "Cull state was not restored");
        GLboolean depthMask = GL_FALSE;
        GLint depthFunc = 0;
        glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
        glGetIntegerv(GL_DEPTH_FUNC, &depthFunc);
        require(depthMask == GL_TRUE && depthFunc == GL_LESS &&
                    glIsEnabled(GL_DEPTH_TEST) == GL_TRUE,
                "Depth state was not restored");
        std::vector<std::uint8_t> pixels(64 * 64 * 4);
        glReadPixels(0, 0, 64, 64, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
        require(glGetError() == GL_NO_ERROR, "Map occlusion draw produced a GL error");
        return pixels;
    };

    std::vector<std::vector<std::uint8_t>> viewSequenceBaseline;
    setTransparentQueries(false);
    initializeComputeCulling(false, false);
    const Matrix4 failureView = translationMatrix({0, 0, -30});
    const auto failureBaseline = draw(false, failureView, false, false);
    const auto sizeCpu = draw(false, failureView, false, false, 0.5);
    initializeComputeCulling(false, true);
    resetFrameTriangleCount();
    draw(false, failureView, false, false);
    const auto allCandidates = frameTriangleCount();
    resetFrameTriangleCount();
    const auto sizeGpu = draw(false, failureView, false, false, 0.5);
    require(sizeGpu == sizeCpu, "Map projected-size CPU/indirect pixels differ");
    require(frameTriangleCount() < allCandidates,
            "Map projected-size broad phase did not remove candidate triangles");
    initializeComputeCulling(false, false);
    bufferData = pglBufferData;
    pglBufferData = recordBufferData;
    failNextProxyUpload = true;
    const auto failurePixels = draw(true, failureView, false, false);
    pglBufferData = bufferData;
    require(!failNextProxyUpload, "Proxy upload failure was not injected");
    require(failurePixels == failureBaseline, "Failed proxy upload hid visible scenery");
    for (bool indirect : {false, true}) {
        initializeComputeCulling(false, indirect);
        require(!indirect || indirectCullingAvailable(), "Indirect compute unavailable");
        for (double offsetX : {0.0, -2.0}) {
            for (bool cull : {false, true}) {
                for (bool masked : {false, true}) {
                    const Matrix4 view = translationMatrix({offsetX, 0, -30});
                    const auto baseline = draw(false, view, cull, masked);
                    const auto occluded = draw(true, view, cull, masked);
                    require(baseline == occluded, "Query culling changed map pixels");
                    bool hasScenery = false;
                    for (std::size_t pixel = 0; pixel < baseline.size(); pixel += 4) {
                        hasScenery |= baseline[pixel] != 0 || baseline[pixel + 1] != 0 ||
                                      baseline[pixel + 2] != 0;
                    }
                    require(hasScenery, "Map fixture did not rasterize scenery");
                    require(queries.size() == 2,
                            "Transparent or special-depth scenery entered the query pass");
                    bool visible = false;
                    bool hidden = false;
                    for (GLuint query : queries) {
                        GLuint result = 0;
                        getQueryObject(query, 0x8866 /* GL_QUERY_RESULT */, &result);
                        visible |= result != 0;
                        hidden |= result == 0;
                    }
                    require(visible && hidden,
                            "Fixture did not exercise visible and occluded queries");
                    setTransparentQueries(true);
                    const auto transparentOccluded = draw(true, view, cull, masked);
                    require(baseline == transparentOccluded,
                            "Transparent query culling changed map pixels");
                    require(queries.size() == 6,
                            "Transparent queries included special-depth scenery or missed glass");
                    GLuint hiddenGlass = 1;
                    getQueryObject(queries[2], 0x8866 /* GL_QUERY_RESULT */, &hiddenGlass);
                    GLuint partialGlass = 0;
                    GLuint visibleGlass = 0;
                    GLuint visibleTransmap = 0;
                    getQueryObject(queries[3], 0x8866 /* GL_QUERY_RESULT */, &partialGlass);
                    getQueryObject(queries[4], 0x8866 /* GL_QUERY_RESULT */, &visibleTransmap);
                    getQueryObject(queries[5], 0x8866 /* GL_QUERY_RESULT */, &visibleGlass);
                    require(hiddenGlass == 0 && partialGlass != 0 && visibleGlass != 0 &&
                                visibleTransmap != 0,
                            "Transparent queries did not reject hidden and retain visible scenery");
                    setTransparentQueries(false);
                }
            }
        }
        const auto nearView = translationMatrix({0, 0, -1});
        const auto nearBaseline = draw(false, nearView, false, false);
        const auto nearOccluded = draw(true, nearView, false, false);
        require(nearBaseline == nearOccluded, "Near-plane proxy changed visible map pixels");
        require(queries.empty(), "Near-plane proxy should not be queried");
        setTransparentQueries(true);
        const auto transparentNearBaseline = draw(false, nearView, false, false);
        require(queries.empty(), "Transparent queries ran without query occlusion");
        const auto transparentNearOccluded = draw(true, nearView, false, false);
        require(transparentNearBaseline == transparentNearOccluded,
                "Near-plane transparent proxy changed visible map pixels");
        require(queries.size() == 1, "Near-plane transparent proxies should not be queried");
        setTransparentQueries(false);
        std::size_t viewIndex = 0;
        for (double offsetX : {0.0, -1000.0, 0.0, -28.0, -30.0, -32.0, 0.0}) {
            const auto pixels = draw(false, translationMatrix({offsetX, 0, -30}), false, false);
            if (!indirect) {
                viewSequenceBaseline.push_back(pixels);
            } else {
                require(pixels == viewSequenceBaseline[viewIndex],
                        "Indirect commands leaked across views");
                if (offsetX == 0.0 || offsetX == -1000.0) {
                    require(offsetX == 0.0 ? indirectArraysDraws > 0 : indirectArraysDraws == 0,
                            "A rejected scenery chunk retained a previous view's slots");
                }
            }
            ++viewIndex;
        }
        if (!indirect) {
            require(viewSequenceBaseline[0] != viewSequenceBaseline[1] &&
                        viewSequenceBaseline[0] == viewSequenceBaseline[2],
                    "View sequence did not leave and return to the scenery");
        }
    }
    writeMesh(root / "model" / "lod-red.x", 8, "1;0;0");
    writeMesh(root / "model" / "lod-green.x", 8, "0;1;0");
    writeMesh(root / "model" / "lod-blue.x", 8, "0;0;1");
    writeText(root / "lod.sco",
              "[lod]\n1\n[mesh]\nlod-red.x\n[lod]\n0.5\n[mesh]\nlod-green.x\n"
              "[lod]\n0.1\n[mesh]\nlod-blue.x\n");
    writeText(root / "lod-tile.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\nlod.sco\n201\n0\n0\n0\n0\n0\n0\n0\n");
    auto lodMap = map;
    lodMap.tiles[0].textPath = root / "lod-tile.map";
    MapRenderer lodRenderer(lodMap, 0, 0, 0, 0, root, simulation, assets);
    require(lodRenderer.renderStats().activeSceneryObjects == 1, "LOD fixture did not load");
    setQueries(false);
    std::vector<std::vector<std::uint8_t>> lodBaseline;
    for (bool indirect : {false, true}) {
        initializeComputeCulling(false, indirect);
        std::size_t viewIndex = 0;
        for (double distance : {24.0, 25.0, 26.0, 49.0, 50.0, 51.0, 249.0, 251.0}) {
            setModelViewMatrix(translationMatrix({0, 0, -distance}));
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
            glDepthMask(GL_TRUE);
            glDisable(GL_CULL_FACE);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            indirectArraysDraws = 0;
            lodRenderer.draw();
            std::vector<std::uint8_t> pixels(64 * 64 * 4);
            glReadPixels(0, 0, 64, 64, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
            if (!indirect) {
                const std::size_t channel = distance <= 25.0 ? 0 : distance <= 50.0 ? 1 : 2;
                if (distance <= 51.0) {
                    std::array<std::uint8_t, 3> maximum = {};
                    for (std::size_t pixel = 0; pixel < pixels.size(); pixel += 4) {
                        for (std::size_t component = 0; component < 3; ++component)
                            maximum[component] = std::max(maximum[component], pixels[pixel + component]);
                    }
                    require(maximum[channel] > 0 && maximum[channel] > maximum[(channel + 1) % 3],
                            "LOD fixture did not render the expected level");
                }
                lodBaseline.push_back(pixels);
            } else {
                require(pixels == lodBaseline[viewIndex], "LOD broad phase changed map pixels");
                const std::size_t expectedDraws = distance == 25.0 || distance == 50.0 ? 2 : 1;
                require(indirectArraysDraws == expectedDraws,
                        "LOD broad phase did not retain the expected candidate levels");
            }
            ++viewIndex;
        }
    }
    std::filesystem::create_directory(root / "Splines");
    std::filesystem::create_directory(root / "Texture");
    writeText(root / "Splines" / "Test.sli",
              "[texture]\nroad.tga\n[profile]\n0\n[profilepnt]\n-2\n0.25\n0\n0.2\n"
              "[profilepnt]\n2\n0.25\n1\n0.2\n");
    ground[18] = ground[19] = ground[20] = 255;
    for (const auto& path : {root / "Texture" / "road.tga", root / "layer.tga",
                             root / "geometry.map.2.tga"}) {
        std::ofstream output(path, std::ios::binary);
        output.write(reinterpret_cast<const char*>(ground.data()), ground.size());
        require(static_cast<bool>(output), "Could not write geometry texture");
    }
    writeMesh(root / "model" / "geometry-large.x", 100, "1;0;0");
    writeMesh(root / "model" / "geometry-small.x", 3, "1;0;0");
    writeText(root / "geometry-large.sco", "[rendertype]\npresurface\n[mesh]\ngeometry-large.x\n");
    writeText(root / "geometry-small.sco", "[rendertype]\npresurface\n[mesh]\ngeometry-small.x\n");
    writeText(root / "distant-terrain.map", "[version]\n14\n[terrain]\n");
    std::array<std::uint8_t, 20> distantTerrain = {};
    distantTerrain[0] = 1;
    const std::uint32_t highTerrain = std::bit_cast<std::uint32_t>(1000.0F);
    for (std::size_t vertex = 0; vertex < 4; ++vertex)
        for (std::size_t byte = 0; byte < 4; ++byte)
            distantTerrain[4 + vertex * 4 + byte] =
                static_cast<std::uint8_t>((highTerrain >> (byte * 8)) & 0xffU);
    {
        std::ofstream output(root / "distant-terrain.terrain", std::ios::binary);
        output.write(reinterpret_cast<const char*>(distantTerrain.data()), distantTerrain.size());
        require(static_cast<bool>(output), "Could not write distant terrain fixture");
    }
    const std::string spline =
        "[spline]\n0\nSplines/Test.sli\n1\n"
        "0\n0\n10\n0\n20\n0\n10\n0\n0\n0\n0\n0\n0\n0\n0\n";
    std::array<std::vector<std::uint8_t>, 6> visibleGeometryPixels;
    for (int scenario = 0; scenario < 9; ++scenario) {
        const int coverage = scenario % 3;
        const bool hasTerrain = scenario / 3 != 2;
        const bool hasRoad = scenario / 3 != 1;
        const std::string occluder =
            coverage == 0 ? "" : coverage == 1
                ? "[object]\n0\ngeometry-large.sco\n401\n0\n0\n30\n0\n0\n0\n0\n"
                : hasTerrain
                    ? "[object]\n0\ngeometry-small.sco\n401\n0\n0\n30\n0\n0\n0\n0\n"
                    : "[object]\n0\ngeometry-small.sco\n401\n6\n15\n30\n0\n0\n0\n0\n";
        const auto tilePath = root / ("geometry-" + std::to_string(scenario) + ".map");
        writeText(tilePath, "[version]\n14\n[terrain]\n" +
                                (hasRoad ? spline : "") + occluder);
        auto geometryMap = map;
        geometryMap.coordinates.tileSize = 10;
        geometryMap.tiles[0].textPath = tilePath;
        geometryMap.tiles[0].hasTerrainFile = hasTerrain;
        if (!hasTerrain) {
            openbus::map::MapTileReference distantTile;
            distantTile.x = 2;
            distantTile.y = -2;
            distantTile.textPath = root / "distant-terrain.map";
            distantTile.terrainPath = root / "distant-terrain.terrain";
            distantTile.hasTerrainFile = true;
            geometryMap.tiles.push_back(distantTile);
        }
        geometryMap.groundTextures.push_back({"layer.tga", "", {}});
        geometryMap.tiles[0].groundTextureSidecars.push_back({"2", root / "geometry.map.2.tga"});
        MapRenderer geometryRenderer(geometryMap, 0, 0, 0, 0, root, simulation, assets);
        require(geometryRenderer.renderStats().totalRoadBatches == (hasRoad ? 1U : 0U),
                "Geometry fixture has the wrong road count");
        if (hasTerrain)
            require(geometryRenderer.renderStats().totalTextures >= (hasRoad ? 3U : 2U),
                    "Geometry fixture ground layer did not load");
        std::vector<std::uint8_t> cpuBaseline;
        for (bool indirect : {false, true}) {
            initializeComputeCulling(false, indirect);
            const auto renderGeometry = [&](bool query, bool geometryQuery, double distance,
                                            double offsetX = 0, double offsetY = 0,
                                            bool cull = false, bool masked = false) {
                setQueries(query);
                setGeometryQueries(geometryQuery);
                queries.clear();
                indirectElementsDraws = 0;
                setModelViewMatrix(translationMatrix({offsetX, offsetY, -distance}));
                glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
                glDepthMask(GL_TRUE);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                glColorMask(GL_TRUE, masked ? GL_FALSE : GL_TRUE, GL_TRUE, GL_TRUE);
                if (cull) glEnable(GL_CULL_FACE); else glDisable(GL_CULL_FACE);
                geometryRenderer.draw();
                std::array<GLboolean, 4> colorMask = {};
                glGetBooleanv(GL_COLOR_WRITEMASK, colorMask.data());
                require(colorMask == std::array<GLboolean, 4>{
                                         GL_TRUE, static_cast<GLboolean>(masked ? GL_FALSE : GL_TRUE),
                                         GL_TRUE, GL_TRUE},
                        "Geometry query did not restore color mask");
                require((glIsEnabled(GL_CULL_FACE) == GL_TRUE) == cull,
                        "Geometry query did not restore face culling");
                GLboolean depthMask = GL_FALSE;
                GLint depthFunc = 0;
                glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
                glGetIntegerv(GL_DEPTH_FUNC, &depthFunc);
                require(depthMask == GL_TRUE && depthFunc == GL_LESS &&
                            glIsEnabled(GL_DEPTH_TEST) == GL_TRUE,
                        "Geometry query did not restore depth state");
                std::vector<std::uint8_t> pixels(64 * 64 * 4);
                glReadPixels(0, 0, 64, 64, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
                require(glGetError() == GL_NO_ERROR, "Geometry query draw produced a GL error");
                return pixels;
            };
            const auto baseline = renderGeometry(false, false, 80);
            auto& visiblePixels = visibleGeometryPixels[static_cast<std::size_t>(scenario / 3) * 2 +
                                                        static_cast<std::size_t>(indirect)];
            if (coverage == 0)
                visiblePixels = baseline;
            if (coverage != 1) {
                bool renderedGeometry = false;
                for (std::size_t pixel = 0; pixel < baseline.size(); pixel += 4)
                    renderedGeometry |= baseline[pixel + 1] != 0 || baseline[pixel + 2] != 0;
                require(renderedGeometry, "Visible geometry fixture did not rasterize");
            }
            if (coverage == 2) {
                bool coveredGeometry = false;
                for (std::size_t pixel = 0; pixel < baseline.size(); pixel += 4)
                    coveredGeometry |= visiblePixels[pixel + 1] != 0 &&
                                       baseline[pixel + 1] == 0 && baseline[pixel] != 0;
                require(coveredGeometry, "Partial fixture did not cover any geometry pixels");
            }
            if (!indirect)
                cpuBaseline = baseline;
            else
                require(baseline == cpuBaseline, "Indirect geometry changed map pixels");
            const auto noGeometryQueries = renderGeometry(true, false, 80);
            require(noGeometryQueries == baseline, "Scenery queries changed geometry pixels");
            const std::size_t sceneryQueries = queries.size();
            const auto withGeometryQueries = renderGeometry(true, true, 80);
            require(withGeometryQueries == baseline, "Geometry queries changed map pixels");
            require(queries.size() == sceneryQueries + static_cast<std::size_t>(hasTerrain) +
                                          static_cast<std::size_t>(hasRoad),
                    "Expected geometry queries were not issued");
            if (indirect && hasTerrain && hasRoad)
                require(indirectElementsDraws >= 3,
                        "Indirect terrain, layer and road draws were not submitted");
            for (std::size_t index = sceneryQueries; index < queries.size(); ++index) {
                GLuint result = 0;
                getQueryObject(queries[index], 0x8866 /* GL_QUERY_RESULT */, &result);
                require((coverage == 1 && result == 0) || (coverage != 1 && result != 0),
                        "Geometry query did not classify hidden and visible bounds correctly");
            }
            if (hasTerrain && hasRoad) {
                const auto maskedBaseline = renderGeometry(false, false, 80, 0, 0, true, true);
                const auto maskedQueried = renderGeometry(true, true, 80, 0, 0, true, true);
                require(maskedQueried == maskedBaseline,
                        "Geometry queries changed pixels with masked colors or face culling");
            }
            const auto nearBaseline = renderGeometry(false, false, 1);
            const auto nearQueried = renderGeometry(true, true, 1);
            require(nearQueried == nearBaseline, "Near-plane geometry query changed map pixels");
            require(queries.empty(), "Near-plane geometry should bypass queries");
            if (coverage == 0 && hasRoad) {
                const auto roadNearBaseline = renderGeometry(false, false, 5.25, -10, -25);
                const auto roadNearQueried = renderGeometry(true, true, 5.25, -10, -25);
                require(roadNearQueried == roadNearBaseline,
                        "Near-plane road query changed map pixels");
                require(queries.empty(),
                        "Near-plane road proxy should bypass queries");
                bool renderedRoad = false;
                for (std::size_t pixel = 0; pixel < roadNearBaseline.size(); pixel += 4)
                    renderedRoad |= roadNearBaseline[pixel + 1] != 0;
                require(renderedRoad, "Near-plane road fixture did not rasterize");
            }
            if (coverage == 0) {
                const auto outside = renderGeometry(false, false, 80, -1000, 0);
                if (indirect)
                    require(indirectElementsDraws == 0,
                            "Off-screen geometry retained indirect draw slots");
                const auto outsideQueried = renderGeometry(true, true, 80, -1000, 0);
                require(outside == outsideQueried && queries.empty(),
                        "Off-screen geometry entered the query pass");
                const auto returned = renderGeometry(true, true, 80);
                require(returned == baseline, "Geometry did not restore after changing views");
            }
        }
    }
    writeText(root / "water.map", "[version]\n14\n[terrain]\n[water]\n");
    {
        std::ofstream output(root / "water.map.water", std::ios::binary);
        const std::array<std::uint32_t, 5> values = {
            1U, std::bit_cast<std::uint32_t>(1.0F), std::bit_cast<std::uint32_t>(1.0F),
            std::bit_cast<std::uint32_t>(1.0F), std::bit_cast<std::uint32_t>(1.0F)};
        for (const std::uint32_t value : values)
            for (std::size_t byte = 0; byte < 4; ++byte)
                output.put(static_cast<char>((value >> (byte * 8)) & 0xffU));
        require(static_cast<bool>(output), "Could not write water fixture");
    }
    auto waterMap = map;
    waterMap.coordinates.tileSize = 10;
    waterMap.tiles[0].textPath = root / "water.map";
    waterMap.tiles[0].waterPath = root / "water.map.water";
    waterMap.tiles[0].hasWaterFile = true;
    waterMap.groundTextures.push_back({"layer.tga", "", {}});
    waterMap.tiles[0].groundTextureSidecars.push_back({"2", root / "geometry.map.2.tga"});
    MapRenderer waterRenderer(waterMap, 0, 0, 0, 0, root, simulation, assets);
    require(waterRenderer.renderStats().totalWaterBatches == 1, "Water fixture did not load");
    setQueries(false);
    setGeometryQueries(false);
    std::vector<std::uint8_t> waterCpuPixels;
    for (bool indirect : {false, true}) {
        initializeComputeCulling(false, indirect);
        const auto drawWaterView = [&](double offsetX) {
            setModelViewMatrix(translationMatrix({offsetX, 0, -80}));
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
            glDepthMask(GL_TRUE);
            glDisable(GL_CULL_FACE);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            indirectElementsDraws = 0;
            waterRenderer.draw();
            std::vector<std::uint8_t> pixels(64 * 64 * 4);
            glReadPixels(0, 0, 64, 64, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
            require(glGetError() == GL_NO_ERROR, "Water broad-phase draw produced a GL error");
            return pixels;
        };
        const auto visible = drawWaterView(0);
        require(visible != visibleGeometryPixels[2 + static_cast<std::size_t>(indirect)],
                "Water fixture did not rasterize");
        if (!indirect)
            waterCpuPixels = visible;
        else {
            require(visible == waterCpuPixels, "Indirect water changed map pixels");
            require(indirectElementsDraws >= 3, "Indirect water was not submitted");
        }
        static_cast<void>(drawWaterView(-1000));
        if (indirect)
            require(indirectElementsDraws == 0, "Off-screen water retained an indirect draw slot");
        require(drawWaterView(0) == visible, "Water did not restore after changing views");
    }
    GLuint reflectionFramebuffer = 0;
    std::array<GLuint, 2> reflectionTextures = {};
    glGenTextures(2, reflectionTextures.data());
    glBindTexture(GL_TEXTURE_2D, reflectionTextures[0]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 64, 64, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glBindTexture(GL_TEXTURE_2D, reflectionTextures[1]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, 64, 64, 0, GL_DEPTH_COMPONENT,
                 GL_FLOAT, nullptr);
    pglGenFramebuffers(1, &reflectionFramebuffer);
    pglBindFramebuffer(GL_FRAMEBUFFER, reflectionFramebuffer);
    pglFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                            reflectionTextures[0], 0);
    pglFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D,
                            reflectionTextures[1], 0);
    require(pglCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE,
            "Water reflection framebuffer is incomplete");
    pglBindFramebuffer(GL_FRAMEBUFFER, 0);
    invalidateTextureBindings();

    {
        std::ofstream output(root / "water-query.map.water", std::ios::binary);
        const std::array<std::uint32_t, 9> values = {
            2U, std::bit_cast<std::uint32_t>(1.0F), std::bit_cast<std::uint32_t>(1.0F),
            std::bit_cast<std::uint32_t>(1.0F), std::bit_cast<std::uint32_t>(1.0F),
            std::bit_cast<std::uint32_t>(2.0F), std::bit_cast<std::uint32_t>(2.0F),
            std::bit_cast<std::uint32_t>(2.0F), std::bit_cast<std::uint32_t>(2.0F)};
        for (const std::uint32_t value : values)
            for (std::size_t byte = 0; byte < 4; ++byte)
                output.put(static_cast<char>((value >> (byte * 8)) & 0xffU));
        require(static_cast<bool>(output), "Could not write layered water fixture");
    }
    std::array<std::size_t, 4> uncoveredWaterPixels = {};
    for (int coverage = 0; coverage < 3; ++coverage) {
        const std::string occluder = coverage == 0 ? "" : coverage == 1
            ? "[object]\n0\ngeometry-large.sco\n501\n0\n0\n30\n0\n0\n0\n0\n"
            : "[object]\n0\ngeometry-small.sco\n501\n0\n0\n30\n0\n0\n0\n0\n";
        const auto tilePath = root / ("water-query-" + std::to_string(coverage) + ".map");
        writeText(tilePath, "[version]\n14\n[terrain]\n[water]\n" + occluder);
        auto queryMap = waterMap;
        queryMap.tiles[0].textPath = tilePath;
        queryMap.tiles[0].waterPath = root / "water-query.map.water";
        MapRenderer queryRenderer(queryMap, 0, 0, 0, 0, root, simulation, assets);
        require(queryRenderer.renderStats().totalWaterBatches == 1,
                "Water query fixture did not load");
        auto noWaterMap = queryMap;
        noWaterMap.tiles[0].hasWaterFile = false;
        MapRenderer noWaterRenderer(noWaterMap, 0, 0, 0, 0, root, simulation, assets);
        auto singleWaterMap = queryMap;
        singleWaterMap.tiles[0].waterPath = root / "water.map.water";
        MapRenderer singleWaterRenderer(singleWaterMap, 0, 0, 0, 0, root, simulation, assets);
        for (bool indirect : {false, true}) {
            initializeComputeCulling(false, indirect);
            require(!indirect || indirectCullingAvailable(), "Indirect compute unavailable");
            for (GLuint framebuffer : {0U, reflectionFramebuffer}) {
                pglBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
                for (bool cull : {false, true}) {
                    for (bool masked : {false, true}) {
                        const auto renderWater = [&](bool queriesEnabled, bool waterEnabled,
                                                     double distance, bool drawWater = true,
                                                     bool rotateView = false,
                                                     bool singleLayer = false) {
                            setQueries(queriesEnabled);
                            setWaterQueries(waterEnabled);
                            queries.clear();
                            const auto translation = translationMatrix({0, 0, -distance});
                            setModelViewMatrix(
                                rotateView
                                    ? multiplyMatrix4(translation, rotationMatrix(35, 0, 0, 1))
                                    : translation);
                            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
                            glDepthMask(GL_TRUE);
                            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                            glColorMask(GL_TRUE, masked ? GL_FALSE : GL_TRUE, GL_TRUE, GL_TRUE);
                            if (cull) glEnable(GL_CULL_FACE); else glDisable(GL_CULL_FACE);
                            (drawWater ? (singleLayer ? singleWaterRenderer : queryRenderer)
                                       : noWaterRenderer).draw();
                            std::array<GLboolean, 4> colorMask = {};
                            glGetBooleanv(GL_COLOR_WRITEMASK, colorMask.data());
                            require(colorMask == std::array<GLboolean, 4>{
                                        GL_TRUE, static_cast<GLboolean>(masked ? GL_FALSE : GL_TRUE),
                                        GL_TRUE, GL_TRUE},
                                    "Water query did not restore color mask");
                            require((glIsEnabled(GL_CULL_FACE) == GL_TRUE) == cull,
                                    "Water query did not restore face culling");
                            GLboolean depthMask = GL_FALSE;
                            GLint depthFunc = 0;
                            glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
                            glGetIntegerv(GL_DEPTH_FUNC, &depthFunc);
                            require(depthMask == GL_TRUE && depthFunc == GL_LESS &&
                                        glIsEnabled(GL_DEPTH_TEST) == GL_TRUE,
                                    "Water query did not restore depth state");
                            std::vector<std::uint8_t> pixels(64 * 64 * 4);
                            std::vector<float> depth(64 * 64);
                            glReadPixels(0, 0, 64, 64, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
                            glReadPixels(0, 0, 64, 64, GL_DEPTH_COMPONENT, GL_FLOAT, depth.data());
                            require(glGetError() == GL_NO_ERROR, "Water query produced a GL error");
                            return std::pair{std::move(pixels), std::move(depth)};
                        };
                        const auto baseline = renderWater(false, false, 80);
                        if (!cull && !masked) {
                            const auto withoutWater = renderWater(false, false, 80, false);
                            std::size_t changedPixels = 0;
                            for (std::size_t pixel = 0; pixel < baseline.first.size(); pixel += 4) {
                                if (!std::equal(baseline.first.begin() + pixel,
                                                baseline.first.begin() + pixel + 4,
                                                withoutWater.first.begin() + pixel))
                                    ++changedPixels;
                            }
                            const std::size_t viewIndex = (indirect ? 2 : 0) +
                                                          (framebuffer == 0 ? 0 : 1);
                            if (coverage == 0) {
                                require(changedPixels > 0, "Visible water did not rasterize");
                                require(renderWater(false, false, 80, true, false, true).first !=
                                            baseline.first,
                                        "Second water surface did not affect pixels");
                                uncoveredWaterPixels[viewIndex] = changedPixels;
                            } else if (coverage == 1) {
                                require(changedPixels == 0, "Hidden water affected pixels");
                            } else {
                                require(changedPixels > 0 &&
                                            changedPixels < uncoveredWaterPixels[viewIndex],
                                        "Partially hidden water was not partially visible");
                            }
                        }
                        const auto disabledQueries = renderWater(false, true, 80);
                        require(disabledQueries == baseline && queries.empty(),
                                "Water queries ran without query occlusion");
                        const auto noWaterQuery = renderWater(true, false, 80);
                        require(noWaterQuery == baseline, "Scenery queries changed water pixels");
                        const std::size_t sceneryCount = queries.size();
                        if (coverage == 0 && !indirect && framebuffer == 0 && !cull && !masked) {
                            genQueries = pglGenQueries;
                            pglGenQueries = failQueryAllocation;
                            failNextQueryAllocation = true;
                            const auto allocationFailure = renderWater(true, true, 80);
                            pglGenQueries = genQueries;
                            require(!failNextQueryAllocation && queries.empty(),
                                    "Water query allocation failure was not injected");
                            require(allocationFailure == baseline,
                                    "Water query allocation failure changed color or depth");
                        }
                        const auto queried = renderWater(true, true, 80);
                        require(queried == baseline, "Water queries changed color or depth");
                        require(queries.size() == sceneryCount + 1,
                                "Water query was not issued");
                        GLuint result = 0;
                        getQueryObject(queries.back(), 0x8866 /* GL_QUERY_RESULT */, &result);
                        require((coverage == 1 && result == 0) || (coverage != 1 && result != 0),
                                "Water query misclassified coverage");
                        if (!cull && !masked) {
                            const auto rotatedBaseline = renderWater(false, false, 80, true, true);
                            const auto rotatedQueried = renderWater(true, true, 80, true, true);
                            require(rotatedQueried == rotatedBaseline && !queries.empty(),
                                    "Rotated view changed water color or depth");
                            getQueryObject(queries.back(), 0x8866 /* GL_QUERY_RESULT */, &result);
                            require((coverage == 1 && result == 0) ||
                                        (coverage != 1 && result != 0),
                                    "Rotated-view water query misclassified coverage");
                            const auto returned = renderWater(true, true, 80);
                            require(returned == baseline, "Water query leaked across views");
                        }
                        if (coverage == 0) {
                            const auto nearBaseline = renderWater(false, false, 1.5);
                            const auto nearQueried = renderWater(true, true, 1.5);
                            require(nearQueried == nearBaseline && queries.empty(),
                                    "Near-plane water proxy did not fall back conservatively");
                        }
                    }
                }
            }
        }
    }
    pglBindFramebuffer(GL_FRAMEBUFFER, 0);
    pglDeleteFramebuffers(1, &reflectionFramebuffer);
    glDeleteTextures(2, reflectionTextures.data());
    invalidateTextureBindings();
    setWaterQueries(false);
    setGeometryQueries(false);
    pglBeginQuery = beginQuery;
    if (genQueries) pglGenQueries = genQueries;
    pglDrawArraysIndirect = drawArraysIndirect;
    pglDrawElementsIndirect = drawElementsIndirect;
}

} // namespace

int main() {
    if (!glfwInit()) return 77;
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(64, 64, "Map occlusion probe", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return 77;
    }
    glfwMakeContextCurrent(window);
    const auto root = std::filesystem::temp_directory_path() /
                      ("openbus-map-occlusion-" +
                       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    int result = 0;
    bool created = false;
    try {
        created = std::filesystem::create_directory(root);
        require(created, "Could not create fixture directory");
        require(loadOpenGLFunctions() && initializeCoreRenderer(), "Renderer initialization failed");
        runProbe(root);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        result = 1;
    }
    pglBeginQuery = beginQuery;
    if (genQueries) pglGenQueries = genQueries;
    if (drawArraysIndirect) pglDrawArraysIndirect = drawArraysIndirect;
    if (drawElementsIndirect) pglDrawElementsIndirect = drawElementsIndirect;
    if (bufferData) pglBufferData = bufferData;
    shutdownCoreRenderer();
    glfwDestroyWindow(window);
    glfwTerminate();
    std::error_code cleanupError;
    if (created) std::filesystem::remove_all(root, cleanupError);
    if (cleanupError) {
        std::cerr << "Could not remove map fixture: " << cleanupError.message() << '\n';
        return 1;
    }
    return result;
}
