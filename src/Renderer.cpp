#include "Renderer.h"

#include "BusSimulation.h"
#include "Logger.h"
#include "RoadFeatures.h"

#include <GLFW/glfw3.h>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <wincodec.h>
#include <windows.h>
#include <wrl/client.h>
#endif
#include <GL/gl.h>
#include <gli/gli.hpp>
#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#ifndef GL_ARRAY_BUFFER
#define GL_ARRAY_BUFFER 0x8892
#define GL_STATIC_DRAW 0x88E4
#endif

Logger gameLog = Logger("Game");
Logger textureLog = Logger("Texture");
Logger wheelLog = Logger("Wheel");

namespace {

enum class RenderViewContext {
    PlayerExterior = 1,
    PlayerInterior = 2,
    NonPlayer = 4,
};

constexpr int viewpointMask(RenderViewContext context) {
    return static_cast<int>(context);
}

constexpr double MAN_DL05_MODEL_OFFSET_Z = -1.035;
constexpr int ROAD_BUMP_SEGMENTS = 12;
constexpr int ROAD_RAMP_SEGMENTS = 6;
constexpr int ROAD_INCLINE_SEGMENTS = 16;

using GenBuffersProc = void (*)(GLsizei, GLuint*);
using BindBufferProc = void (*)(GLenum, GLuint);
using BufferDataProc = void (*)(GLenum, std::ptrdiff_t, const void*, GLenum);
using DeleteBuffersProc = void (*)(GLsizei, const GLuint*);
using CompressedTexImage2DProc = void (*)(GLenum, GLint, GLenum, GLsizei, GLsizei, GLint,
                                          GLsizei, const void*);
GenBuffersProc pglGenBuffers = nullptr;
BindBufferProc pglBindBuffer = nullptr;
BufferDataProc pglBufferData = nullptr;
DeleteBuffersProc pglDeleteBuffers = nullptr;
CompressedTexImage2DProc pglCompressedTexImage2D = nullptr;

bool loadBufferFunctions() {
    pglGenBuffers = reinterpret_cast<GenBuffersProc>(glfwGetProcAddress("glGenBuffers"));
    pglBindBuffer = reinterpret_cast<BindBufferProc>(glfwGetProcAddress("glBindBuffer"));
    pglBufferData = reinterpret_cast<BufferDataProc>(glfwGetProcAddress("glBufferData"));
    pglDeleteBuffers = reinterpret_cast<DeleteBuffersProc>(glfwGetProcAddress("glDeleteBuffers"));
    pglCompressedTexImage2D = reinterpret_cast<CompressedTexImage2DProc>(
        glfwGetProcAddress("glCompressedTexImage2D"));
    const bool available = pglGenBuffers && pglBindBuffer && pglBufferData && pglDeleteBuffers &&
                           pglCompressedTexImage2D;
    if (!available) {
        gameLog.Log("Failed to load required OpenGL VBO functions");
    }
    return available;
}

void drawBox(double length, double width, double height, double red, double green, double blue) {
    const double x = length * 0.5;
    const double y = width * 0.5;
    const double z = height * 0.5;
    glColor3d(red, green, blue);
    glBegin(GL_LINES);
    glVertex3d(-x, -y, -z);
    glVertex3d(x, -y, -z);
    glVertex3d(x, -y, -z);
    glVertex3d(x, y, -z);
    glVertex3d(x, y, -z);
    glVertex3d(-x, y, -z);
    glVertex3d(-x, y, -z);
    glVertex3d(-x, -y, -z);
    glVertex3d(-x, -y, z);
    glVertex3d(x, -y, z);
    glVertex3d(x, -y, z);
    glVertex3d(x, y, z);
    glVertex3d(x, y, z);
    glVertex3d(-x, y, z);
    glVertex3d(-x, y, z);
    glVertex3d(-x, -y, z);
    glVertex3d(-x, -y, -z);
    glVertex3d(-x, -y, z);
    glVertex3d(x, -y, -z);
    glVertex3d(x, -y, z);
    glVertex3d(x, y, -z);
    glVertex3d(x, y, z);
    glVertex3d(-x, y, -z);
    glVertex3d(-x, y, z);
    glEnd();
}

void drawRoadBox(double centerX, double centerY, double length, double width, double height,
                 double bottomZ, const std::array<double, 3>& topColor,
                 const std::array<double, 3>& sideColor) {
    const double halfLength = length * 0.5;
    const double halfWidth = width * 0.5;
    glPushMatrix();
    glTranslated(centerX, centerY, bottomZ);
    glColor3d(topColor[0], topColor[1], topColor[2]);
    glBegin(GL_QUADS);
    glVertex3d(-halfLength, -halfWidth, height);
    glVertex3d(halfLength, -halfWidth, height);
    glVertex3d(halfLength, halfWidth, height);
    glVertex3d(-halfLength, halfWidth, height);
    glColor3d(sideColor[0], sideColor[1], sideColor[2]);
    glVertex3d(-halfLength, -halfWidth, 0.0);
    glVertex3d(halfLength, -halfWidth, 0.0);
    glVertex3d(halfLength, -halfWidth, height);
    glVertex3d(-halfLength, -halfWidth, height);
    glVertex3d(halfLength, halfWidth, 0.0);
    glVertex3d(-halfLength, halfWidth, 0.0);
    glVertex3d(-halfLength, halfWidth, height);
    glVertex3d(halfLength, halfWidth, height);
    glEnd();
    glPopMatrix();

    glPushMatrix();
    glTranslated(centerX, centerY, bottomZ + height * 0.5);
    drawBox(length, width, height, 0.0, 0.85, 0.95);
    glPopMatrix();
}

void drawRoadIncline(const RoadBump& bump) {
    const double halfWidth = bump.width * 0.5;
    const double startX = bump.centerX - bump.length * 0.5;
    const double segmentLength = bump.length / ROAD_INCLINE_SEGMENTS;
    const auto heightAt = [&](double x) { return roadFeatureHeightAt(bump, x - bump.centerX); };

    glBegin(GL_QUADS);
    for (int segment = 0; segment < ROAD_INCLINE_SEGMENTS; ++segment) {
        const double minX = startX + segment * segmentLength;
        const double maxX = minX + segmentLength;
        const double minHeight = heightAt(minX);
        const double maxHeight = heightAt(maxX);
        glColor3d(0.42, 0.50, 0.30);
        glVertex3d(minX, bump.centerY - halfWidth, minHeight);
        glVertex3d(maxX, bump.centerY - halfWidth, maxHeight);
        glVertex3d(maxX, bump.centerY + halfWidth, maxHeight);
        glVertex3d(minX, bump.centerY + halfWidth, minHeight);
        glColor3d(0.18, 0.25, 0.12);
        glVertex3d(minX, bump.centerY - halfWidth, 0.0);
        glVertex3d(maxX, bump.centerY - halfWidth, 0.0);
        glVertex3d(maxX, bump.centerY - halfWidth, maxHeight);
        glVertex3d(minX, bump.centerY - halfWidth, minHeight);
        glVertex3d(maxX, bump.centerY + halfWidth, 0.0);
        glVertex3d(minX, bump.centerY + halfWidth, 0.0);
        glVertex3d(minX, bump.centerY + halfWidth, minHeight);
        glVertex3d(maxX, bump.centerY + halfWidth, maxHeight);
    }
    const double endX = startX + bump.length;
    const double endHeight = heightAt(endX);
    glColor3d(0.18, 0.25, 0.12);
    glVertex3d(endX, bump.centerY - halfWidth, 0.0);
    glVertex3d(endX, bump.centerY + halfWidth, 0.0);
    glVertex3d(endX, bump.centerY + halfWidth, endHeight);
    glVertex3d(endX, bump.centerY - halfWidth, endHeight);
    glEnd();

    glColor3d(0.0, 0.85, 0.95);
    glBegin(GL_LINE_STRIP);
    for (int segment = 0; segment <= ROAD_INCLINE_SEGMENTS; ++segment) {
        const double x = startX + segment * segmentLength;
        glVertex3d(x, bump.centerY - halfWidth, heightAt(x) + 0.005);
    }
    glEnd();
    glBegin(GL_LINE_STRIP);
    for (int segment = 0; segment <= ROAD_INCLINE_SEGMENTS; ++segment) {
        const double x = startX + segment * segmentLength;
        glVertex3d(x, bump.centerY + halfWidth, heightAt(x) + 0.005);
    }
    glEnd();
    glBegin(GL_LINES);
    for (int segment = 0; segment <= ROAD_INCLINE_SEGMENTS; segment += 8) {
        const double x = startX + segment * segmentLength;
        const double height = heightAt(x) + 0.005;
        glVertex3d(x, bump.centerY - halfWidth, height);
        glVertex3d(x, bump.centerY + halfWidth, height);
        glVertex3d(x, bump.centerY - halfWidth, 0.0);
        glVertex3d(x, bump.centerY - halfWidth, height);
        glVertex3d(x, bump.centerY + halfWidth, 0.0);
        glVertex3d(x, bump.centerY + halfWidth, height);
    }
    glEnd();
}

void drawGround(const std::vector<RoadBump>& bumps) {
    glColor3d(0.18, 0.22, 0.18);
    glBegin(GL_QUADS);
    glVertex3d(-100.0, -100.0, 0.0);
    glVertex3d(100.0, -100.0, 0.0);
    glVertex3d(100.0, 100.0, 0.0);
    glVertex3d(-100.0, 100.0, 0.0);
    glEnd();

    glLineWidth(1.0f);
    glBegin(GL_LINES);
    for (int line = -100; line <= 100; line += 2) {
        const double coordinate = static_cast<double>(line);
        const bool major = line % 10 == 0;
        if (major) {
            glColor3d(0.38, 0.43, 0.36);
        } else {
            glColor3d(0.25, 0.30, 0.25);
        }
        glVertex3d(coordinate, -100.0, 0.02);
        glVertex3d(coordinate, 100.0, 0.02);
        glVertex3d(-100.0, coordinate, 0.02);
        glVertex3d(100.0, coordinate, 0.02);
    }
    glEnd();

    for (const RoadBump& bump : bumps) {
        if (bump.type == RoadFeatureType::Barrier) {
            drawRoadBox(bump.centerX, bump.centerY, bump.length, bump.width, bump.height, 0.0,
                        {0.85, 0.22, 0.08}, {0.42, 0.08, 0.03});
            continue;
        }
        if (bump.type == RoadFeatureType::Bridge) {
            const double deckLength = std::clamp(bump.flatLength, 0.0, bump.length);
            const double rampLength = (bump.length - deckLength) * 0.5;
            const double segmentLength = rampLength / ROAD_RAMP_SEGMENTS;
            const double startX = bump.centerX - bump.length * 0.5;
            const std::array<double, 3> deckColor = {0.36, 0.42, 0.48};
            const std::array<double, 3> deckSideColor = {0.16, 0.20, 0.24};
            for (int segment = 0; segment < ROAD_RAMP_SEGMENTS; ++segment) {
                const double phase = (static_cast<double>(segment) + 0.5) / ROAD_RAMP_SEGMENTS;
                drawRoadBox(startX + segmentLength * (segment + 0.5), bump.centerY, segmentLength,
                            bump.width, bump.height * phase, 0.0, deckColor, deckSideColor);
            }
            drawRoadBox(bump.centerX, bump.centerY, deckLength, bump.width, bump.height, 0.0,
                        deckColor, deckSideColor);
            const double downStartX = bump.centerX + bump.length * 0.5 - rampLength;
            for (int segment = 0; segment < ROAD_RAMP_SEGMENTS; ++segment) {
                const double phase = (static_cast<double>(segment) + 0.5) / ROAD_RAMP_SEGMENTS;
                drawRoadBox(downStartX + segmentLength * (segment + 0.5), bump.centerY,
                            segmentLength, bump.width, bump.height * (1.0 - phase), 0.0, deckColor,
                            deckSideColor);
            }
            if (bump.railHeight > 0.0 && bump.railWidth > 0.0) {
                const double railY = bump.width * 0.5 - bump.railWidth * 0.5;
                const std::array<double, 3> railColor = {0.72, 0.76, 0.80};
                const std::array<double, 3> railSideColor = {0.30, 0.34, 0.38};
                drawRoadBox(bump.centerX, bump.centerY + railY, bump.length, bump.railWidth,
                            bump.railHeight, bump.height, railColor, railSideColor);
                drawRoadBox(bump.centerX, bump.centerY - railY, bump.length, bump.railWidth,
                            bump.railHeight, bump.height, railColor, railSideColor);
            }
            continue;
        }
        if (bump.type == RoadFeatureType::Incline) {
            drawRoadIncline(bump);
            continue;
        }
        const double halfWidth = bump.width * 0.5;
        const double segmentLength = bump.length / ROAD_BUMP_SEGMENTS;
        for (int segment = 0; segment < ROAD_BUMP_SEGMENTS; ++segment) {
            const double phase = (static_cast<double>(segment) + 0.5) / ROAD_BUMP_SEGMENTS;
            const double height = bump.height * (phase <= 0.5 ? phase * 2.0 : (1.0 - phase) * 2.0);
            const double minX = bump.centerX - bump.length * 0.5 + segment * segmentLength;
            const double maxX = minX + segmentLength;
            const double minY = bump.centerY - halfWidth;
            const double maxY = bump.centerY + halfWidth;
            drawRoadBox((minX + maxX) * 0.5, bump.centerY, maxX - minX, maxY - minY, height, 0.0,
                        {0.72, 0.46, 0.18}, {0.48, 0.28, 0.10});
        }
    }
}

void drawWheel(double radius, double halfWidth, double red = 0.04, double green = 0.04,
               double blue = 0.04) {
    constexpr int segments = 16;
    // Match dCreateCylinder's local Z axis before applying the ODE pose.
    glColor3d(red, green, blue);
    glBegin(GL_LINES);
    for (int segment = 0; segment < segments; ++segment) {
        const double first = 2.0 * 3.141592653589793 * segment / segments;
        const double second = 2.0 * 3.141592653589793 * (segment + 1) / segments;
        for (const double z : {-halfWidth, halfWidth}) {
            glVertex3d(radius * std::cos(first), radius * std::sin(first), z);
            glVertex3d(radius * std::cos(second), radius * std::sin(second), z);
        }
        glVertex3d(radius * std::cos(first), radius * std::sin(first), -halfWidth);
        glVertex3d(radius * std::cos(first), radius * std::sin(first), halfWidth);
    }
    glEnd();
}

void drawCenterOfGravityMarker(double size) {
    glColor3d(0.95, 0.10, 0.10);
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    glVertex3d(-size, 0.0, 0.0);
    glVertex3d(size, 0.0, 0.0);
    glVertex3d(0.0, -size, 0.0);
    glVertex3d(0.0, size, 0.0);
    glVertex3d(0.0, 0.0, -size);
    glVertex3d(0.0, 0.0, size);
    glEnd();
    glLineWidth(1.0f);
}

void setPerspective(double width, double height) {
    const double aspect = width / height;
    const double nearPlane = 0.1;
    const double farPlane = 500.0;
    const double fov = 60.0 * 3.141592653589793 / 180.0;
    const double top = nearPlane * std::tan(fov * 0.5);
    const double right = top * aspect;
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-right, right, -top, top, nearPlane, farPlane);
}

void lookAt(double eyeX, double eyeY, double eyeZ, double targetX, double targetY, double targetZ) {
    double forwardX = targetX - eyeX;
    double forwardY = targetY - eyeY;
    double forwardZ = targetZ - eyeZ;
    const double forwardLength =
        std::sqrt(forwardX * forwardX + forwardY * forwardY + forwardZ * forwardZ);
    forwardX /= forwardLength;
    forwardY /= forwardLength;
    forwardZ /= forwardLength;

    double sideX = forwardY;
    double sideY = -forwardX;
    const double sideLength = std::sqrt(sideX * sideX + sideY * sideY);
    sideX /= sideLength;
    sideY /= sideLength;
    const double upX = sideY * forwardZ;
    const double upY = -sideX * forwardZ;
    const double upZ = sideX * forwardY - sideY * forwardX;

    const double matrix[16] = {sideX, upX, -forwardX, 0.0, sideY, upY, -forwardY, 0.0,
                               0.0,   upZ, -forwardZ, 0.0, 0.0,   0.0, 0.0,       1.0};
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd(matrix);
    glTranslated(-eyeX, -eyeY, -eyeZ);
}

void applyPose(const BodyPose& pose) {
    const std::array<double, 16> matrix = {pose.rotation[0],
                                           pose.rotation[3],
                                           pose.rotation[6],
                                           0.0,
                                           pose.rotation[1],
                                           pose.rotation[4],
                                           pose.rotation[7],
                                           0.0,
                                           pose.rotation[2],
                                           pose.rotation[5],
                                           pose.rotation[8],
                                           0.0,
                                           0.0,
                                           0.0,
                                           0.0,
                                           1.0};
    glTranslated(pose.position[0], pose.position[1], pose.position[2]);
    glMultMatrixd(matrix.data());
}

std::array<double, 3> transformLocalPoint(const BodyPose& pose,
                                          const std::array<double, 3>& local) {
    return {pose.position[0] + pose.rotation[0] * local[0] + pose.rotation[1] * local[1] +
                pose.rotation[2] * local[2],
            pose.position[1] + pose.rotation[3] * local[0] + pose.rotation[4] * local[1] +
                pose.rotation[5] * local[2],
            pose.position[2] + pose.rotation[6] * local[0] + pose.rotation[7] * local[1] +
                pose.rotation[8] * local[2]};
}

void drawCollisionWireframe(const BusSimulation& simulation) {
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_BLEND);
    glDisable(GL_ALPHA_TEST);
    glDepthMask(GL_FALSE);
    glLineWidth(2.0f);

    const BodyPose chassis = simulation.chassisPose();
    const ChassisCollisionBox collision = simulation.chassisCollisionBox();
    glPushMatrix();
    applyPose(chassis);
    glTranslated(0.0, 0.0, collision.offsetZ);
    drawBox(collision.length, collision.width, collision.height, 0.0, 0.85, 0.95);
    glPopMatrix();

    for (std::size_t index = 0; index < simulation.wheelCount(); ++index) {
        glPushMatrix();
        applyPose(simulation.wheelPose(index));
        drawWheel(simulation.wheelRadius(), simulation.wheelHalfWidth(), 0.0, 0.85, 0.95);
        glPopMatrix();
    }

    glLineWidth(1.0f);
    glDepthMask(GL_TRUE);
    glColor4d(1.0, 1.0, 1.0, 1.0);
}

bool saveFramebufferBmp(const std::filesystem::path& path, int width, int height) {
    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(width) * height * 4);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());

    std::array<std::uint8_t, 54> header = {};
    const std::uint32_t imageSize = static_cast<std::uint32_t>(width * height * 3);
    const std::uint32_t fileSize = 54 + imageSize;
    header[0] = 'B';
    header[1] = 'M';
    header[2] = static_cast<std::uint8_t>(fileSize);
    header[3] = static_cast<std::uint8_t>(fileSize >> 8);
    header[4] = static_cast<std::uint8_t>(fileSize >> 16);
    header[5] = static_cast<std::uint8_t>(fileSize >> 24);
    header[10] = 54;
    header[14] = 40;
    header[18] = static_cast<std::uint8_t>(width);
    header[19] = static_cast<std::uint8_t>(width >> 8);
    header[20] = static_cast<std::uint8_t>(width >> 16);
    header[21] = static_cast<std::uint8_t>(width >> 24);
    header[22] = static_cast<std::uint8_t>(height);
    header[23] = static_cast<std::uint8_t>(height >> 8);
    header[24] = static_cast<std::uint8_t>(height >> 16);
    header[25] = static_cast<std::uint8_t>(height >> 24);
    header[26] = 1;
    header[28] = 24;
    header[34] = static_cast<std::uint8_t>(imageSize);
    header[35] = static_cast<std::uint8_t>(imageSize >> 8);
    header[36] = static_cast<std::uint8_t>(imageSize >> 16);
    header[37] = static_cast<std::uint8_t>(imageSize >> 24);

    std::ofstream output(path, std::ios::binary);
    if (!output) {
        return false;
    }
    output.write(reinterpret_cast<const char*>(header.data()), header.size());
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const std::size_t index = (static_cast<std::size_t>(y) * width + x) * 4;
            const std::array<std::uint8_t, 3> pixel = {pixels[index + 2], pixels[index + 1],
                                                        pixels[index]};
            output.write(reinterpret_cast<const char*>(pixel.data()), pixel.size());
        }
    }
    return output.good();
}
} // namespace

struct BusModel {
    ModelLoadingPolicy loadingPolicy;
    struct Image {
        int width = 0;
        int height = 0;
        std::vector<std::uint8_t> rgba;
    };

    struct TextureRequest {
        std::filesystem::path root;
        std::filesystem::path path;
        std::string name;
        std::filesystem::path resolvedPath;
        std::shared_ptr<Image> image;
        std::shared_ptr<gli::texture> compressedTexture;
        bool complete = false;
        std::mutex mutex;
        std::thread worker;
    };

    struct DecodedTexture {
        std::shared_ptr<Image> image;
        std::shared_ptr<gli::texture> compressedTexture;
    };

    struct TextureCacheEntry {
        std::shared_ptr<TextureRequest> request;
        GLuint texture = 0;
        bool uploadAttempted = false;
    };

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
        double boundsRadius = 0.0;
    };

    struct ObjRequest {
        std::future<std::shared_ptr<ParsedObj>> future;
    };

    struct MaterialState {
        std::filesystem::path texturePath;
        std::string textureName;
        std::string environmentTextureName;
        double environmentStrength = 0.0;
        int alphaMode = 0;
        bool noZwrite = false;
    };

    struct WheelAnimation {
        std::string rotationVariable;
        std::string suspensionVariable;
        std::string steeringVariable;
        std::array<double, 3> origin = {};
        bool hasOrigin = false;
    };

    struct Part {
        std::filesystem::path objPath;
        std::filesystem::path texturePath;
        std::string textureName;
        std::array<double, 3> color = {0.65, 0.65, 0.65};
        int viewpoint = 0;
        int renderType = 2;
        std::string meshIdentifier;
        std::string animationParent;
        std::unordered_map<std::string, MaterialState> materialStates;
        int lodIndex = -1;
        WheelAnimation wheelAnimation;
        std::shared_ptr<ObjRequest> objRequest;
    };

    struct Vertex {
        float x, y, z, u, v;
    };

    struct Batch {
        GLuint buffer = 0;
        GLuint texture = 0;
        std::filesystem::path texturePath;
        std::filesystem::path textureRoot;
        std::string textureName;
        std::string environmentTextureName;
        GLuint environmentTexture = 0;
        double environmentStrength = 0.0;
        bool environmentLoadAttempted = false;
        std::array<double, 3> color = {0.65, 0.65, 0.65};
        bool textured = false;
        bool textureLoadAttempted = false;
        bool textureLoadStarted = false;
        std::shared_ptr<TextureRequest> textureRequest;
        std::shared_ptr<TextureCacheEntry> textureCacheEntry;
        int alphaMode = 0;
        bool noZwrite = false;
        std::size_t vertexCount = 0;
    };

    struct DisplayPart {
        std::vector<Batch> batches;
        int viewpoint;
        int renderType = 2;
        bool transparent;
        int lodIndex;
        std::array<double, 3> center;
        double radius;
        std::size_t triangleCount;
    };

    struct WheelModel {
        WheelAnimation animation;
        std::vector<DisplayPart> parts;
    };

    std::vector<DisplayPart> displayLists;
    std::vector<WheelModel> wheelModels;
    std::vector<Part> pendingParts;
    std::vector<GLuint> textures;
    std::unordered_map<std::string, std::shared_ptr<TextureCacheEntry>> textureCache;
    std::unordered_map<std::string, std::shared_ptr<TextureCacheEntry>> textureAliases;
    std::mutex decodedTextureMutex;
    std::unordered_map<std::string, std::shared_ptr<DecodedTexture>> decodedTextureCache;
    mutable std::unordered_set<std::size_t> loggedWheelBindings;
    double modelOffsetZ = MAN_DL05_MODEL_OFFSET_Z;
    double textureScale = 1;
    bool frustumCulling = true;
    std::vector<double> lodThresholds;
    std::size_t opaqueDisplayCount = 0;
    mutable std::size_t lastRenderedTriangles = 0;
    bool loaded = false;
    bool hasLoadedInitialView = false;
    int activeLod = -1;
    std::chrono::steady_clock::time_point textureUploadStart;

    void loadTextureRequest(const std::shared_ptr<TextureRequest>& request);

    void joinTextureWorkers() {
        for (const auto& cache : textureCache) {
            if (cache.second->request && cache.second->request->worker.joinable()) {
                cache.second->request->worker.join();
            }
        }
    }

#ifdef _WIN32
    bool comInitialized = false;
#endif

    explicit BusModel(BusVehicle vehicle, ModelLoadingPolicy policy)
        : loadingPolicy(policy) {
        if (const char* scale = std::getenv("OPENBUS_TEXTURE_SCALE")) {
            try {
                textureScale = std::clamp(std::stod(scale), 0.25, 1.0);
            } catch (const std::exception&) {
                textureScale = 1.0;
            }
        }
        if (const char* culling = std::getenv("OPENBUS_FRUSTUM_CULLING")) {
            frustumCulling = std::string(culling) == "1";
        }
#ifdef _WIN32
        const HRESULT comResult = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        comInitialized = SUCCEEDED(comResult);
#endif
        std::filesystem::path relativeConfig;
        std::filesystem::path relativeModelRoot;
        if (vehicle == BusVehicle::SpE400Mmc) {
            relativeConfig = std::filesystem::path("SP_E400MMC") / "Model" / "Configuration Files" /
                             "E400MMC_ADL_10.9m_Voith_LowHeight.cfg";
            relativeModelRoot = std::filesystem::path("SP_E400MMC") / "Model" / "Converted" /
                                "E400MMC_ADL_10.9m_Voith_LowHeight_obj";
            modelOffsetZ = -1.02;
        } else {
            relativeConfig = std::filesystem::path("MAN_DL05") / "Model" / "DL05.cfg";
            relativeModelRoot = std::filesystem::path("MAN_DL05") / "Model" / "DL05";
        }
        const auto resolveAssetPath = [](const std::filesystem::path& relative) {
            const std::array<std::filesystem::path, 4> candidates = {
                relative, std::filesystem::current_path() / relative,
                std::filesystem::current_path().parent_path() / relative,
                std::filesystem::current_path().parent_path().parent_path() / relative};
            for (const auto& candidate : candidates) {
                if (std::filesystem::exists(candidate)) {
                    return candidate;
                }
            }
            return relative;
        };
        gameLog.Log("Loading bus model with config: " + resolveAssetPath(relativeConfig).string() +
                    " and model root: " + resolveAssetPath(relativeModelRoot).string());
        load(resolveAssetPath(relativeConfig), resolveAssetPath(relativeModelRoot));
    }

    ~BusModel() {
        joinTextureWorkers();
        const auto deleteBuffers = [](const std::vector<DisplayPart>& parts) {
            for (const DisplayPart& part : parts) {
                for (const Batch& batch : part.batches) {
                    if (batch.buffer) {
                        pglDeleteBuffers(1, &batch.buffer);
                    }
                }
            }
        };
        deleteBuffers(displayLists);
        for (const WheelModel& wheel : wheelModels) {
            deleteBuffers(wheel.parts);
        }
        if (!textures.empty()) {
            glDeleteTextures(static_cast<GLsizei>(textures.size()), textures.data());
        }
#ifdef _WIN32
        if (comInitialized) {
            CoUninitialize();
        }
#endif
    }

    void draw(RenderViewContext context) {
        if (!loaded) {
            return;
        }
        textureUploadStart = std::chrono::steady_clock::now();

        glDisable(GL_TEXTURE_2D);
        glDisable(GL_BLEND);
        glDisable(GL_ALPHA_TEST);
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
        glColor4d(1.0, 1.0, 1.0, 1.0);
        GLdouble modelView[16] = {};
        GLdouble projection[16] = {};
        glGetDoublev(GL_MODELVIEW_MATRIX, modelView);
        glGetDoublev(GL_PROJECTION_MATRIX, projection);
        std::array<std::array<double, 4>, 6> frustumPlanes = {};
        std::array<double, 6> frustumPlaneLengths = {};
        if (frustumCulling) {
            const std::array<std::array<double, 4>, 6> planeSigns = {
                {{{1.0, 0.0, 0.0, 1.0}}, {{-1.0, 0.0, 0.0, 1.0}},
                 {{0.0, 1.0, 0.0, 1.0}}, {{0.0, -1.0, 0.0, 1.0}},
                 {{0.0, 0.0, 1.0, 1.0}}, {{0.0, 0.0, -1.0, 1.0}}}};
            for (std::size_t plane = 0; plane < planeSigns.size(); ++plane) {
                const auto& signs = planeSigns[plane];
                double lengthSquared = 0.0;
                for (int column = 0; column < 4; ++column) {
                    double coefficient = 0.0;
                    for (int row = 0; row < 4; ++row) {
                        coefficient += signs[row] * projection[row + column * 4];
                    }
                    frustumPlanes[plane][column] = coefficient;
                    if (column < 3) {
                        lengthSquared += coefficient * coefficient;
                    }
                }
                frustumPlaneLengths[plane] = std::sqrt(lengthSquared);
            }
        }
        const auto visible = [&](const DisplayPart& part) {
            const double local[4] = {part.center[0], part.center[1], part.center[2] + modelOffsetZ,
                                     1.0};
            double eye[4] = {};
            for (int row = 0; row < 4; ++row) {
                for (int column = 0; column < 4; ++column) {
                    eye[row] += modelView[row + column * 4] * local[column];
                }
            }
            const double distance = std::sqrt(eye[0] * eye[0] + eye[1] * eye[1] + eye[2] * eye[2]);
            if (!lodThresholds.empty() && part.lodIndex >= 0) {
                std::size_t selectedLod = lodThresholds.size() - 1;
                for (std::size_t index = 0; index < lodThresholds.size(); ++index) {
                    const double boundary = 25.0 / std::max(lodThresholds[index], 0.001);
                    if (distance <= boundary) {
                        selectedLod = index;
                        break;
                    }
                }
                if (static_cast<std::size_t>(part.lodIndex) != selectedLod) {
                    return false;
                }
            }
            if (!frustumCulling) {
                return true;
            }
            for (std::size_t plane = 0; plane < frustumPlanes.size(); ++plane) {
                const double planeDistance = frustumPlanes[plane][0] * eye[0] +
                                             frustumPlanes[plane][1] * eye[1] +
                                             frustumPlanes[plane][2] * eye[2] +
                                             frustumPlanes[plane][3];
                if (planeDistance < -part.radius * frustumPlaneLengths[plane]) {
                    return false;
                }
            }
            return true;
        };
        const auto viewDepth = [&](const DisplayPart& part) {
            const double local[4] = {part.center[0], part.center[1], part.center[2] + modelOffsetZ,
                                     1.0};
            double eyeZ = 0.0;
            for (int column = 0; column < 4; ++column) {
                eyeZ += modelView[2 + column * 4] * local[column];
            }
            return -eyeZ;
        };
        struct TransparentBatch {
            Batch* batch;
            double depth;
            int renderType;
        };
        std::vector<DisplayPart*> opaqueParts;
        std::vector<TransparentBatch> noDepthOpaqueBatches;
        std::vector<TransparentBatch> transparentBatches;
        for (DisplayPart& part : displayLists) {
            const bool viewpointMatches =
                part.viewpoint == 0 || (part.viewpoint & viewpointMask(context)) != 0;
            if (!viewpointMatches || !visible(part)) {
                continue;
            }
            const bool hasOpaqueBatch =
                std::any_of(part.batches.begin(), part.batches.end(), [](const Batch& batch) {
                    return batch.alphaMode == 0 && !batch.noZwrite;
                });
            for (Batch& batch : part.batches) {
                if (batch.alphaMode == 0 && batch.noZwrite) {
                        ensureTexture(batch);
                    transparentBatches.push_back({&batch, viewDepth(part), part.renderType});
                }
            }
            if (hasOpaqueBatch) {
                opaqueParts.push_back(&part);
            }
        }
        std::sort(opaqueParts.begin(), opaqueParts.end(),
                  [&](const DisplayPart* first, const DisplayPart* second) {
                      return viewDepth(*first) < viewDepth(*second);
                  });
        std::stable_sort(transparentBatches.begin(), transparentBatches.end(),
                         [](const TransparentBatch& first, const TransparentBatch& second) {
                             return first.renderType < second.renderType;
                         });
        std::stable_sort(noDepthOpaqueBatches.begin(), noDepthOpaqueBatches.end(),
                         [](const TransparentBatch& first, const TransparentBatch& second) {
                             return first.renderType < second.renderType;
                         });
        lastRenderedTriangles = 0;
        for (DisplayPart* part : opaqueParts) {
            lastRenderedTriangles += part->triangleCount;
        }
        for (const TransparentBatch& transparent : transparentBatches) {
            lastRenderedTriangles += transparent.batch->vertexCount / 3;
        }
        glPushMatrix();
        glTranslated(0.0, 0.0, modelOffsetZ);
        for (DisplayPart* part : opaqueParts) {
            for (Batch& batch : part->batches) {
                if (batch.alphaMode != 0 || batch.noZwrite) {
                    continue;
                }
                if (batch.alphaMode == 2 || batch.noZwrite) {
                    glEnable(GL_BLEND);
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    glDepthMask(GL_TRUE);
                } else if (batch.alphaMode == 1) {
                    glDisable(GL_BLEND);
                    glEnable(GL_ALPHA_TEST);
                    glAlphaFunc(GL_GREATER, 0.5f);
                    glDepthMask(GL_FALSE);
                } else {
                    glDisable(GL_BLEND);
                    glDisable(GL_ALPHA_TEST);
                    glDepthMask(GL_TRUE);
                }
                ensureTexture(batch);
                if (batch.textured) {
                    glEnable(GL_TEXTURE_2D);
                    glBindTexture(GL_TEXTURE_2D, batch.texture);
                    glColor3d(1.0, 1.0, 1.0);
                } else {
                    glDisable(GL_TEXTURE_2D);
                    glColor3d(batch.color[0], batch.color[1], batch.color[2]);
                }
                pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
                glEnableClientState(GL_VERTEX_ARRAY);
                glVertexPointer(3, GL_FLOAT, sizeof(Vertex), reinterpret_cast<const void*>(0));
                if (batch.textured) {
                    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
                    glTexCoordPointer(2, GL_FLOAT, sizeof(Vertex),
                                      reinterpret_cast<const void*>(3 * sizeof(float)));
                }
                glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(batch.vertexCount));
                glDisableClientState(GL_TEXTURE_COORD_ARRAY);
                glDisableClientState(GL_VERTEX_ARRAY);
                drawEnvironmentMap(batch);
            }
        }
        glDepthMask(GL_FALSE);
        glDisable(GL_BLEND);
        glDisable(GL_ALPHA_TEST);
        for (const TransparentBatch& noDepthOpaque : noDepthOpaqueBatches) {
            Batch& batch = *noDepthOpaque.batch;
            ensureTexture(batch);
            if (batch.textured) {
                glEnable(GL_TEXTURE_2D);
                glBindTexture(GL_TEXTURE_2D, batch.texture);
                glColor3d(1.0, 1.0, 1.0);
            } else {
                glDisable(GL_TEXTURE_2D);
                glColor3d(batch.color[0], batch.color[1], batch.color[2]);
            }
            pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
            glEnableClientState(GL_VERTEX_ARRAY);
            glVertexPointer(3, GL_FLOAT, sizeof(Vertex), nullptr);
            if (batch.textured) {
                glEnableClientState(GL_TEXTURE_COORD_ARRAY);
                glTexCoordPointer(2, GL_FLOAT, sizeof(Vertex),
                                  reinterpret_cast<const void*>(3 * sizeof(float)));
            }
            glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(batch.vertexCount));
            glDisableClientState(GL_TEXTURE_COORD_ARRAY);
            glDisableClientState(GL_VERTEX_ARRAY);
            drawEnvironmentMap(batch);
        }
        glDepthMask(GL_FALSE);
        for (const TransparentBatch& transparent : transparentBatches) {
            Batch& batch = *transparent.batch;
            if (batch.alphaMode == 1) {
                glDisable(GL_BLEND);
                glEnable(GL_ALPHA_TEST);
                glAlphaFunc(GL_GREATER, 0.5f);
            } else if (batch.alphaMode == 2) {
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                glDisable(GL_ALPHA_TEST);
            } else {
                glDisable(GL_BLEND);
                glDisable(GL_ALPHA_TEST);
            }
            ensureTexture(batch);
            if (batch.textured) {
                glEnable(GL_TEXTURE_2D);
                glBindTexture(GL_TEXTURE_2D, batch.texture);
                glColor3d(1.0, 1.0, 1.0);
            } else {
                glDisable(GL_TEXTURE_2D);
                glColor3d(batch.color[0], batch.color[1], batch.color[2]);
            }
            pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
            glEnableClientState(GL_VERTEX_ARRAY);
            glVertexPointer(3, GL_FLOAT, sizeof(Vertex), nullptr);
            if (batch.textured) {
                glEnableClientState(GL_TEXTURE_COORD_ARRAY);
                glTexCoordPointer(2, GL_FLOAT, sizeof(Vertex),
                                  reinterpret_cast<const void*>(3 * sizeof(float)));
            }
            glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(batch.vertexCount));
            glDisableClientState(GL_TEXTURE_COORD_ARRAY);
            glDisableClientState(GL_VERTEX_ARRAY);
            drawEnvironmentMap(batch);
        }
        pglBindBuffer(GL_ARRAY_BUFFER, 0);
        glDepthMask(GL_TRUE);
        glDisable(GL_TEXTURE_2D);
        glDisable(GL_BLEND);
        glDisable(GL_ALPHA_TEST);
        glDepthMask(GL_TRUE);
        glColor4d(1.0, 1.0, 1.0, 1.0);
        glPopMatrix();
    }

    bool hasConfiguredWheels(std::size_t expectedWheelCount) const {
        if (wheelModels.size() < expectedWheelCount) {
            return false;
        }
        return std::all_of(wheelModels.begin(), wheelModels.end(),
                           [](const WheelModel& wheel) { return !wheel.parts.empty(); });
    }

    void drawConfiguredWheels(const BusSimulation& simulation, const BodyPose& chassis,
                              bool outsideView) {
        std::vector<bool> usedWheelIndices(simulation.wheelCount(), false);
        for (std::size_t modelIndex = 0; modelIndex < wheelModels.size(); ++modelIndex) {
            WheelModel& wheel = wheelModels[modelIndex];
            if (wheel.parts.empty() || !wheel.animation.hasOrigin) {
                continue;
            }
            std::array<double, 3> targetLocal = wheel.animation.origin;
            targetLocal[2] += modelOffsetZ;
            const std::array<double, 3> target = transformLocalPoint(chassis, targetLocal);
            std::size_t simulationIndex = 0;
            double closestDistance = std::numeric_limits<double>::max();
            for (std::size_t index = 0; index < simulation.wheelCount(); ++index) {
                if (usedWheelIndices[index]) {
                    continue;
                }
                const BodyPose pose = simulation.wheelPose(index);
                const double deltaX = pose.position[0] - target[0];
                const double deltaY = pose.position[1] - target[1];
                const double deltaZ = pose.position[2] - target[2];
                const double distance = deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ;
                if (distance < closestDistance) {
                    closestDistance = distance;
                    simulationIndex = index;
                }
            }
            if (closestDistance == std::numeric_limits<double>::max()) {
                continue;
            }
            usedWheelIndices[simulationIndex] = true;
            const BodyPose pose = simulation.wheelPose(simulationIndex);
            if (loggedWheelBindings.insert(modelIndex).second) {
                wheelLog.Log("cfgWheel=" + std::to_string(modelIndex) +
                             " odeWheel=" + std::to_string(simulationIndex) + " origin=(" +
                             std::to_string(wheel.animation.origin[0]) + ',' +
                             std::to_string(wheel.animation.origin[1]) + ',' +
                             std::to_string(wheel.animation.origin[2]) + ')' +
                             " rotationVar=" + wheel.animation.rotationVariable +
                             " suspensionVar=" + wheel.animation.suspensionVariable +
                             " steeringVar=" + wheel.animation.steeringVariable + " pose=(" +
                             std::to_string(pose.position[0]) + ',' +
                             std::to_string(pose.position[1]) + ',' +
                             std::to_string(pose.position[2]) + ")");
            }
            for (DisplayPart& part : wheel.parts) {
                const bool visibleOutside = part.viewpoint == 0 || (part.viewpoint & 1) != 0;
                const bool visibleInside = part.viewpoint == 0 || (part.viewpoint & 2) != 0;
                if ((outsideView && !visibleOutside) || (!outsideView && !visibleInside)) {
                    continue;
                }
                glPushMatrix();
                applyPose(pose);
                glRotated(90.0, 1.0, 0.0, 0.0);
                glTranslated(-part.center[0], -part.center[1], -part.center[2]);
                for (Batch& batch : part.batches) {
                    ensureTexture(batch);
                    if (batch.alphaMode == 2 || batch.noZwrite) {
                        glEnable(GL_BLEND);
                        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                        glDepthMask(GL_FALSE);
                    } else if (batch.alphaMode == 1) {
                        glDisable(GL_BLEND);
                        glEnable(GL_ALPHA_TEST);
                        glAlphaFunc(GL_GREATER, 0.5f);
                        glDepthMask(GL_FALSE);
                    } else {
                        glDisable(GL_BLEND);
                        glDisable(GL_ALPHA_TEST);
                        glDepthMask(GL_TRUE);
                    }
                    if (batch.textured) {
                        glEnable(GL_TEXTURE_2D);
                        glBindTexture(GL_TEXTURE_2D, batch.texture);
                        glColor3d(1.0, 1.0, 1.0);
                    } else {
                        glDisable(GL_TEXTURE_2D);
                        glColor3d(batch.color[0], batch.color[1], batch.color[2]);
                    }
                    pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
                    glEnableClientState(GL_VERTEX_ARRAY);
                    glVertexPointer(3, GL_FLOAT, sizeof(Vertex), nullptr);
                    if (batch.textured) {
                        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
                        glTexCoordPointer(2, GL_FLOAT, sizeof(Vertex),
                                          reinterpret_cast<const void*>(3 * sizeof(float)));
                    } else {
                        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
                    }
                    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(batch.vertexCount));
                    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
                    glDisableClientState(GL_VERTEX_ARRAY);
                }
                glPopMatrix();
            }
        }
        pglBindBuffer(GL_ARRAY_BUFFER, 0);
        glDepthMask(GL_TRUE);
        glDisable(GL_TEXTURE_2D);
        glDisable(GL_BLEND);
        glDisable(GL_ALPHA_TEST);
        glColor4d(1.0, 1.0, 1.0, 1.0);
    }

    std::size_t renderedTriangles() const {
        return lastRenderedTriangles;
    }

    void loadForView(RenderViewContext context, double viewDistance) {
        ensureLoaded(context, viewDistance);
    }

  private:
    static std::string trim(const std::string& value) {
        const std::size_t first = value.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) {
            return {};
        }
        const std::size_t last = value.find_last_not_of(" \t\r\n");
        return value.substr(first, last - first + 1);
    }

    static std::string lower(std::string value) {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
        return value;
    }

    static int parseInt(const std::string& value, int fallback) {
        try {
            return std::stoi(trim(value));
        } catch (const std::exception&) {
            return fallback;
        }
    }

    static double parseDouble(const std::string& value, double fallback) {
        try {
            return std::stod(trim(value));
        } catch (const std::exception&) {
            return fallback;
        }
    }

    static bool readNonEmptyLine(std::istream& input, std::string& value) {
        while (std::getline(input, value)) {
            value = trim(value);
            if (!value.empty()) {
                return true;
            }
        }
        return false;
    }

    static bool readImage(const std::filesystem::path& path, Image& image) {
        gameLog.Log("Reading image from path: " + path.string());
        const std::string extension = lower(path.extension().string());
#ifdef _WIN32
        if (extension == ".png" || extension == ".jpg" || extension == ".jpeg" ||
            extension == ".bmp") {
            if (readWicImage(path, image)) {
                return true;
            }
        }
#endif
        if (extension == ".dds") {
            return readDdsImage(path, image);
        }
        if (extension == ".tga") {
            std::ifstream input(path, std::ios::binary);
            if (!input) {
                return false;
            }
            std::array<std::uint8_t, 18> header = {};
            input.read(reinterpret_cast<char*>(header.data()),
                       static_cast<std::streamsize>(header.size()));
            if (!input || (header[2] != 2 && header[2] != 10) ||
                (header[16] != 24 && header[16] != 32)) {
                return false;
            }
            input.seekg(header[0], std::ios::cur);
            image.width = header[12] | (header[13] << 8);
            image.height = header[14] | (header[15] << 8);
            const int channels = header[16] / 8;
            const std::size_t pixelCount = static_cast<std::size_t>(image.width) * image.height;
            std::vector<std::uint8_t> source(pixelCount * channels);
            if (header[2] == 2) {
                input.read(reinterpret_cast<char*>(source.data()),
                           static_cast<std::streamsize>(source.size()));
                if (!input) {
                    return false;
                }
            } else {
                std::size_t pixelIndex = 0;
                while (pixelIndex < pixelCount && input) {
                    std::uint8_t packetHeader = 0;
                    input.read(reinterpret_cast<char*>(&packetHeader), 1);
                    const std::size_t packetCount =
                        static_cast<std::size_t>(packetHeader & 0x7F) + 1;
                    if (pixelIndex + packetCount > pixelCount) {
                        return false;
                    }
                    if ((packetHeader & 0x80) != 0) {
                        std::array<std::uint8_t, 4> pixel = {};
                        input.read(reinterpret_cast<char*>(pixel.data()), channels);
                        if (!input) {
                            return false;
                        }
                        for (std::size_t count = 0; count < packetCount; ++count) {
                            std::copy_n(pixel.data(), channels,
                                        source.data() + (pixelIndex + count) * channels);
                        }
                    } else {
                        input.read(reinterpret_cast<char*>(source.data() + pixelIndex * channels),
                                   static_cast<std::streamsize>(packetCount * channels));
                        if (!input) {
                            return false;
                        }
                    }
                    pixelIndex += packetCount;
                }
                if (pixelIndex != pixelCount) {
                    return false;
                }
            }
            image.rgba.resize(pixelCount * 4);
            const bool topOrigin = (header[17] & 0x20) != 0;
            for (int y = 0; y < image.height; ++y) {
                const int sourceY = topOrigin ? y : image.height - y - 1;
                for (int x = 0; x < image.width; ++x) {
                    const std::size_t sourceIndex =
                        (static_cast<std::size_t>(sourceY) * image.width + x) * channels;
                    const std::size_t targetIndex =
                        (static_cast<std::size_t>(y) * image.width + x) * 4;
                    image.rgba[targetIndex + 0] = source[sourceIndex + 2];
                    image.rgba[targetIndex + 1] = source[sourceIndex + 1];
                    image.rgba[targetIndex + 2] = source[sourceIndex + 0];
                    image.rgba[targetIndex + 3] = channels == 4 ? source[sourceIndex + 3] : 255;
                }
            }
            return true;
        }

        if (extension != ".bmp") {
            return false;
        }

        std::ifstream input(path, std::ios::binary);

        if (!input) {
            return false;
        }
        std::array<std::uint8_t, 54> header = {};
        input.read(reinterpret_cast<char*>(header.data()),
                   static_cast<std::streamsize>(header.size()));
        if (!input || header[0] != 'B' || header[1] != 'M') {
            return false;
        }
        const std::uint32_t pixelOffset =
            header[10] | (header[11] << 8) | (header[12] << 16) | (header[13] << 24);
        const std::int32_t width = static_cast<std::int32_t>(
            header[18] | (header[19] << 8) | (header[20] << 16) | (header[21] << 24));
        const std::int32_t height = static_cast<std::int32_t>(
            header[22] | (header[23] << 8) | (header[24] << 16) | (header[25] << 24));
        const std::uint16_t bitsPerPixel =
            static_cast<std::uint16_t>(header[28] | (header[29] << 8));
        if (width <= 0 || height == 0 || (bitsPerPixel != 24 && bitsPerPixel != 32)) {
            return false;
        }
        const int absoluteHeight = std::abs(height);
        const int channels = bitsPerPixel / 8;
        const std::size_t rowStride = ((static_cast<std::size_t>(width) * channels + 3) / 4) * 4;
        std::vector<std::uint8_t> source(rowStride * absoluteHeight);
        input.seekg(pixelOffset);
        input.read(reinterpret_cast<char*>(source.data()),
                   static_cast<std::streamsize>(source.size()));
        if (!input) {
            return false;
        }
        image.width = width;
        image.height = absoluteHeight;
        image.rgba.resize(static_cast<std::size_t>(width) * absoluteHeight * 4);
        for (int y = 0; y < absoluteHeight; ++y) {
            const int sourceY = height > 0 ? absoluteHeight - y - 1 : y;
            for (int x = 0; x < width; ++x) {
                const std::size_t sourceIndex = static_cast<std::size_t>(sourceY) * rowStride +
                                                static_cast<std::size_t>(x) * channels;
                const std::size_t targetIndex = (static_cast<std::size_t>(y) * width + x) * 4;
                image.rgba[targetIndex + 0] = source[sourceIndex + 2];
                image.rgba[targetIndex + 1] = source[sourceIndex + 1];
                image.rgba[targetIndex + 2] = source[sourceIndex + 0];
                image.rgba[targetIndex + 3] = channels == 4 ? source[sourceIndex + 3] : 255;
            }
        }
        gameLog.Log("Successfully read image with width: " + std::to_string(image.width) +
                    " and height: " + std::to_string(image.height));
        return true;
    }

#ifdef _WIN32
    static bool readWicImage(const std::filesystem::path& path, Image& image) {
        using Microsoft::WRL::ComPtr;
        ComPtr<IWICImagingFactory> factory;
        if (FAILED(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER,
                                    IID_PPV_ARGS(&factory)))) {
            return false;
        }
        ComPtr<IWICBitmapDecoder> decoder;
        if (FAILED(factory->CreateDecoderFromFilename(path.c_str(), nullptr, GENERIC_READ,
                                                      WICDecodeMetadataCacheOnLoad, &decoder))) {
            return false;
        }
        ComPtr<IWICBitmapFrameDecode> frame;
        if (FAILED(decoder->GetFrame(0, &frame))) {
            return false;
        }
        ComPtr<IWICFormatConverter> converter;
        if (FAILED(factory->CreateFormatConverter(&converter)) ||
            FAILED(converter->Initialize(frame.Get(), GUID_WICPixelFormat32bppRGBA,
                                         WICBitmapDitherTypeNone, nullptr, 0.0,
                                         WICBitmapPaletteTypeCustom))) {
            return false;
        }
        UINT width = 0;
        UINT height = 0;
        if (FAILED(converter->GetSize(&width, &height)) || width == 0 || height == 0) {
            return false;
        }
        image.width = static_cast<int>(width);
        image.height = static_cast<int>(height);
        image.rgba.resize(static_cast<std::size_t>(width) * height * 4);
        return SUCCEEDED(converter->CopyPixels(
            nullptr, width * 4, static_cast<UINT>(image.rgba.size()), image.rgba.data()));
    }
#endif

    static std::uint16_t readU16(const std::vector<std::uint8_t>& data, std::size_t offset) {
        return static_cast<std::uint16_t>(data[offset]) |
               (static_cast<std::uint16_t>(data[offset + 1]) << 8);
    }

    static std::uint32_t readU32(const std::vector<std::uint8_t>& data, std::size_t offset) {
        return static_cast<std::uint32_t>(data[offset]) |
               (static_cast<std::uint32_t>(data[offset + 1]) << 8) |
               (static_cast<std::uint32_t>(data[offset + 2]) << 16) |
               (static_cast<std::uint32_t>(data[offset + 3]) << 24);
    }

    static bool isSafeCompressedDds(const std::filesystem::path& path) {
        std::ifstream input(path, std::ios::binary);
        if (!input) {
            return false;
        }
        std::array<std::uint8_t, 128> header = {};
        input.read(reinterpret_cast<char*>(header.data()), static_cast<std::streamsize>(header.size()));
        if (!input || std::string(header.begin(), header.begin() + 4) != "DDS ") {
            return false;
        }

        const auto readHeaderU32 = [&header](std::size_t offset) {
            return static_cast<std::uint32_t>(header[offset]) |
                   (static_cast<std::uint32_t>(header[offset + 1]) << 8) |
                   (static_cast<std::uint32_t>(header[offset + 2]) << 16) |
                   (static_cast<std::uint32_t>(header[offset + 3]) << 24);
        };
        const std::uint32_t width = readHeaderU32(16);
        const std::uint32_t height = readHeaderU32(12);
        const std::string fourCC(header.begin() + 84, header.begin() + 88);
        const std::size_t blockSize = fourCC == "DXT1" ? 8 : (fourCC == "DXT3" || fourCC == "DXT5" ? 16 : 0);
        if (width == 0 || height == 0 || blockSize == 0) {
            return false;
        }

        const std::uint32_t flags = readHeaderU32(8);
        const std::uint32_t declaredLevels = readHeaderU32(28);
        const std::size_t levelCount = (flags & 0x20000U) != 0 ? declaredLevels : 1;
        if (levelCount == 0 || levelCount > 32) {
            return false;
        }

        std::size_t requiredSize = 128;
        std::uint32_t levelWidth = width;
        std::uint32_t levelHeight = height;
        for (std::size_t level = 0; level < levelCount; ++level) {
            const std::size_t blocksX = (static_cast<std::size_t>(levelWidth) + 3) / 4;
            const std::size_t blocksY = (static_cast<std::size_t>(levelHeight) + 3) / 4;
            if (blocksX > (std::numeric_limits<std::size_t>::max() / blocksY) ||
                blocksX * blocksY > std::numeric_limits<std::size_t>::max() / blockSize ||
                requiredSize > std::numeric_limits<std::size_t>::max() - blocksX * blocksY * blockSize) {
                return false;
            }
            requiredSize += blocksX * blocksY * blockSize;
            levelWidth = std::max(1U, levelWidth / 2);
            levelHeight = std::max(1U, levelHeight / 2);
        }

        input.seekg(0, std::ios::end);
        const std::streamoff fileSize = input.tellg();
        return fileSize >= 0 && static_cast<std::uintmax_t>(fileSize) >= requiredSize;
    }

    static void decode565(std::uint16_t value, std::uint8_t* color) {
        color[0] = static_cast<std::uint8_t>(((value >> 11) & 0x1F) * 255 / 31);
        color[1] = static_cast<std::uint8_t>(((value >> 5) & 0x3F) * 255 / 63);
        color[2] = static_cast<std::uint8_t>((value & 0x1F) * 255 / 31);
        color[3] = 255;
    }

    static bool readDdsImage(const std::filesystem::path& path, Image& image) {
        std::ifstream input(path, std::ios::binary);
        if (!input) {
            return false;
        }
        input.seekg(0, std::ios::end);
        const std::streamoff length = input.tellg();
        input.seekg(0, std::ios::beg);
        if (length < 128) {
            return false;
        }
        std::vector<std::uint8_t> data(static_cast<std::size_t>(length));
        input.read(reinterpret_cast<char*>(data.data()), length);
        if (!input || std::string(data.begin(), data.begin() + 4) != "DDS ") {
            return false;
        }
        image.width = static_cast<int>(readU32(data, 16));
        image.height = static_cast<int>(readU32(data, 12));
        const std::uint32_t pixelFormatFlags = readU32(data, 80);
        const std::string fourCC(data.begin() + 84, data.begin() + 88);
        const bool dxt1 = fourCC == "DXT1";
        const bool dxt2 = fourCC == "DXT2";
        const bool dxt3 = fourCC == "DXT3";
        const bool dxt5 = fourCC == "DXT5";
        const std::uint32_t rgbBitCount = readU32(data, 88);
        const std::uint32_t redMask = readU32(data, 92);
        const std::uint32_t greenMask = readU32(data, 96);
        const std::uint32_t blueMask = readU32(data, 100);
        const std::uint32_t alphaMask = readU32(data, 104);
        const bool dxt1HasAlpha = dxt1 && (pixelFormatFlags & 0x1U) != 0;
        if (image.width <= 0 || image.height <= 0) {
            return false;
        }
        if (!dxt1 && !dxt2 && !dxt3 && !dxt5) {
            if ((pixelFormatFlags & 0x40U) == 0 || (rgbBitCount != 24 && rgbBitCount != 32)) {
                return false;
            }
            const std::size_t bytesPerPixel = rgbBitCount / 8;
            const std::size_t minimumRowPitch =
                static_cast<std::size_t>(image.width) * bytesPerPixel;
            const std::size_t rowPitch = std::max<std::size_t>(readU32(data, 20), minimumRowPitch);
            if (rowPitch > (std::numeric_limits<std::size_t>::max() - 128) /
                               static_cast<std::size_t>(image.height)) {
                return false;
            }
            const std::size_t requiredSize =
                128 + rowPitch * static_cast<std::size_t>(image.height);
            if (data.size() < requiredSize) {
                return false;
            }
            const auto decodeChannel = [](std::uint32_t pixel, std::uint32_t mask,
                                          std::uint8_t fallback) {
                if (mask == 0) {
                    return fallback;
                }
                unsigned shift = 0;
                while ((mask & 1U) == 0) {
                    mask >>= 1;
                    ++shift;
                }
                const std::uint32_t component = (pixel >> shift) & mask;
                return static_cast<std::uint8_t>(
                    (static_cast<std::uint64_t>(component) * 255 + mask / 2) / mask);
            };
            image.rgba.resize(static_cast<std::size_t>(image.width) * image.height * 4);
            for (int y = 0; y < image.height; ++y) {
                const std::size_t rowOffset = 128 + static_cast<std::size_t>(y) * rowPitch;
                for (int x = 0; x < image.width; ++x) {
                    const std::size_t sourceOffset =
                        rowOffset + static_cast<std::size_t>(x) * bytesPerPixel;
                    std::uint32_t pixel =
                        data[sourceOffset] |
                        (static_cast<std::uint32_t>(data[sourceOffset + 1]) << 8) |
                        (static_cast<std::uint32_t>(data[sourceOffset + 2]) << 16);
                    if (bytesPerPixel == 4) {
                        pixel |= static_cast<std::uint32_t>(data[sourceOffset + 3]) << 24;
                    }
                    const std::size_t targetOffset =
                        (static_cast<std::size_t>(y) * image.width + x) * 4;
                    image.rgba[targetOffset + 0] = decodeChannel(pixel, redMask, 0);
                    image.rgba[targetOffset + 1] = decodeChannel(pixel, greenMask, 0);
                    image.rgba[targetOffset + 2] = decodeChannel(pixel, blueMask, 0);
                    image.rgba[targetOffset + 3] = decodeChannel(pixel, alphaMask, 255);
                }
            }
            return true;
        }
        const int blocksX = (image.width + 3) / 4;
        const int blocksY = (image.height + 3) / 4;
        const bool explicitAlpha = dxt2 || dxt3;
        const std::size_t blockSize = dxt5 || explicitAlpha ? 16 : 8;
        const std::size_t requiredSize =
            128 + static_cast<std::size_t>(blocksX) * blocksY * blockSize;
        if (data.size() < requiredSize) {
            return false;
        }
        image.rgba.resize(static_cast<std::size_t>(image.width) * image.height * 4);
        std::size_t sourceOffset = 128;
        for (int blockY = 0; blockY < blocksY; ++blockY) {
            for (int blockX = 0; blockX < blocksX; ++blockX) {
                std::array<std::uint8_t, 8> alphaValues = {};
                std::uint64_t alphaIndices = 0;
                if (explicitAlpha) {
                    for (int byte = 0; byte < 8; ++byte) {
                        alphaIndices |= static_cast<std::uint64_t>(data[sourceOffset + byte])
                                        << (8 * byte);
                    }
                } else if (dxt5) {
                    const std::uint8_t alpha0 = data[sourceOffset];
                    const std::uint8_t alpha1 = data[sourceOffset + 1];
                    alphaValues[0] = alpha0;
                    alphaValues[1] = alpha1;
                    if (alpha0 > alpha1) {
                        for (int index = 1; index <= 6; ++index) {
                            alphaValues[index + 1] = static_cast<std::uint8_t>(
                                ((7 - index) * alpha0 + index * alpha1) / 7);
                        }
                    } else {
                        for (int index = 1; index <= 4; ++index) {
                            alphaValues[index + 1] = static_cast<std::uint8_t>(
                                ((5 - index) * alpha0 + index * alpha1) / 5);
                        }
                        alphaValues[6] = 0;
                        alphaValues[7] = 255;
                    }
                    for (int byte = 0; byte < 6; ++byte) {
                        alphaIndices |= static_cast<std::uint64_t>(data[sourceOffset + 2 + byte])
                                        << (8 * byte);
                    }
                }
                const std::size_t colorOffset = sourceOffset + (dxt5 || explicitAlpha ? 8 : 0);
                const std::uint16_t color0 = readU16(data, colorOffset);
                const std::uint16_t color1 = readU16(data, colorOffset + 2);
                const std::uint32_t indices = readU32(data, colorOffset + 4);
                sourceOffset += blockSize;
                std::array<std::array<std::uint8_t, 4>, 4> colors = {};
                decode565(color0, colors[0].data());
                decode565(color1, colors[1].data());
                if (color0 > color1 || dxt5 || explicitAlpha) {
                    for (int channel = 0; channel < 3; ++channel) {
                        colors[2][channel] = static_cast<std::uint8_t>(
                            (2 * colors[0][channel] + colors[1][channel]) / 3);
                        colors[3][channel] = static_cast<std::uint8_t>(
                            (colors[0][channel] + 2 * colors[1][channel]) / 3);
                    }
                    colors[2][3] = colors[3][3] = 255;
                } else {
                    for (int channel = 0; channel < 3; ++channel) {
                        colors[2][channel] = static_cast<std::uint8_t>(
                            (colors[0][channel] + colors[1][channel]) / 2);
                    }
                    colors[2][3] = 255;
                    colors[3] = {0, 0, 0, static_cast<std::uint8_t>(dxt1HasAlpha ? 0 : 255)};
                }
                for (int row = 0; row < 4; ++row) {
                    for (int column = 0; column < 4; ++column) {
                        const int x = blockX * 4 + column;
                        const int y = blockY * 4 + row;
                        if (x >= image.width || y >= image.height) {
                            continue;
                        }
                        const std::size_t colorIndex = (indices >> (2 * (row * 4 + column))) & 0x3;
                        const std::size_t pixelIndex = static_cast<std::size_t>(row * 4 + column);
                        const std::size_t target =
                            (static_cast<std::size_t>(y) * image.width + x) * 4;
                        std::copy(colors[colorIndex].begin(), colors[colorIndex].end(),
                                  image.rgba.begin() + target);
                        if (explicitAlpha) {
                            image.rgba[target + 3] = static_cast<std::uint8_t>(
                                ((alphaIndices >> (4 * pixelIndex)) & 0xF) * 17);
                        } else if (dxt5) {
                            image.rgba[target + 3] =
                                alphaValues[(alphaIndices >> (3 * pixelIndex)) & 0x7];
                        }
                    }
                }
            }
        }
        return true;
    }

    GLuint uploadTexture(const std::filesystem::path& path, Image image) {
        gameLog.Log("Uploading texture from path: " + path.generic_string());
        if (textureScale < 0.999) {
            const int scaledWidth =
                std::max(1, static_cast<int>(std::lround(image.width * textureScale)));
            const int scaledHeight =
                std::max(1, static_cast<int>(std::lround(image.height * textureScale)));
            std::vector<std::uint8_t> scaled(static_cast<std::size_t>(scaledWidth) * scaledHeight *
                                             4);
            for (int y = 0; y < scaledHeight; ++y) {
                const int sourceY = std::min(image.height - 1, static_cast<int>(y / textureScale));
                for (int x = 0; x < scaledWidth; ++x) {
                    const int sourceX =
                        std::min(image.width - 1, static_cast<int>(x / textureScale));
                    const std::size_t source =
                        (static_cast<std::size_t>(sourceY) * image.width + sourceX) * 4;
                    const std::size_t target = (static_cast<std::size_t>(y) * scaledWidth + x) * 4;
                    std::copy_n(image.rgba.data() + source, 4, scaled.data() + target);
                }
            }
            image.width = scaledWidth;
            image.height = scaledHeight;
            image.rgba = std::move(scaled);
        }
        GLuint texture = 0;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.width, image.height, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, image.rgba.data());
        textures.push_back(texture);
        return texture;
    }

    GLuint uploadCompressedTexture(const std::filesystem::path& path,
                                   const gli::texture& image) {
        gli::gl translator(gli::gl::PROFILE_GL33);
        const gli::gl::format format = translator.translate(image.format(), image.swizzles());
        if (format.Internal == 0 || !gli::is_compressed(image.format())) {
            return 0;
        }
        GLuint texture = 0;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, image.levels() > 1
                                                               ? GL_LINEAR_MIPMAP_LINEAR
                                                               : GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        for (std::size_t level = 0; level < image.levels(); ++level) {
            const auto extent = image.extent(level);
            pglCompressedTexImage2D(
                GL_TEXTURE_2D, static_cast<GLint>(level), format.Internal,
                static_cast<GLsizei>(extent.x), static_cast<GLsizei>(extent.y), 0,
                static_cast<GLsizei>(image.size(level)), image.data(0, 0, level));
        }
        textures.push_back(texture);
        gameLog.Log("Uploaded compressed texture from path: " + path.generic_string());
        return texture;
    }

    static std::vector<ObjIndex> parseFace(const std::string& value, int positionCount,
                                           int texCoordCount) {
        std::vector<ObjIndex> result;
        std::istringstream stream(value);
        std::string token;
        while (stream >> token) {
            ObjIndex index;
            const std::size_t firstSlash = token.find('/');
            const std::size_t secondSlash = firstSlash == std::string::npos
                                                ? std::string::npos
                                                : token.find('/', firstSlash + 1);
            const std::string positionText = token.substr(0, firstSlash);
            index.position = parseInt(positionText, 0);
            if (firstSlash != std::string::npos) {
                const std::string texCoordText =
                    token.substr(firstSlash + 1, secondSlash == std::string::npos
                                                     ? std::string::npos
                                                     : secondSlash - firstSlash - 1);
                index.texCoord = parseInt(texCoordText, 0);
            }
            if (index.position < 0) {
                index.position += positionCount + 1;
            }
            if (index.texCoord < 0) {
                index.texCoord += texCoordCount + 1;
            }
            result.push_back(index);
        }
        return result;
    }

    std::filesystem::path findTexture(const std::filesystem::path& root, const std::string& name) {
        const std::string cleaned = trim(name);
        if (cleaned.empty()) {
            return {};
        }
        static std::mutex resolutionMutex;
        static std::unordered_map<std::string, std::filesystem::path> resolutionCache;
        const std::string resolutionKey = lower(std::filesystem::path(cleaned).stem().string());
        std::lock_guard<std::mutex> lock(resolutionMutex);
        const auto cached = resolutionCache.find(resolutionKey);
        if (cached != resolutionCache.end()) {
            return cached->second;
        }
        gameLog.Log("Resolving texture with name: " + name + " in root: " + root.generic_string());
        const auto cacheResult = [&](const std::filesystem::path& path) {
            resolutionCache.emplace(resolutionKey, path);
            return path;
        };
        std::filesystem::path direct = root / cleaned;
        if (std::filesystem::exists(direct)) {
            return cacheResult(direct);
        }
        for (std::filesystem::path ancestor = root; !ancestor.empty();
             ancestor = ancestor.parent_path()) {
            const std::filesystem::path textureRoot = ancestor / "Texture";
            if (std::filesystem::exists(textureRoot)) {
                direct = textureRoot / cleaned;
                if (std::filesystem::exists(direct)) {
                    return cacheResult(direct);
                }
            }
            const std::filesystem::path parent = ancestor.parent_path();
            if (parent == ancestor) {
                break;
            }
        }
        const std::string basename = lower(std::filesystem::path(cleaned).filename().string());
        const std::string requestedStem = lower(std::filesystem::path(cleaned).stem().string());
        const std::array<std::string, 6> imageExtensions = {".tga", ".bmp", ".png",
                                                            ".dds", ".jpg", ".jpeg"};
        for (std::filesystem::path ancestor = root; !ancestor.empty();
             ancestor = ancestor.parent_path()) {
            const std::filesystem::path textureRoot = ancestor / "Texture";
            if (std::filesystem::exists(textureRoot)) {
                for (const auto& entry : std::filesystem::directory_iterator(
                         textureRoot, std::filesystem::directory_options::skip_permission_denied)) {
                    if (entry.is_regular_file() &&
                        lower(entry.path().stem().string()) == requestedStem &&
                        std::find(imageExtensions.begin(), imageExtensions.end(),
                                  lower(entry.path().extension().string())) !=
                            imageExtensions.end()) {
                        return cacheResult(entry.path());
                    }
                }
            }
            const std::filesystem::path parent = ancestor.parent_path();
            if (parent == ancestor) {
                break;
            }
        }
        if (std::filesystem::exists(root)) {
            for (const auto& entry : std::filesystem::directory_iterator(
                     root, std::filesystem::directory_options::skip_permission_denied)) {
                if (entry.is_regular_file() &&
                    lower(entry.path().stem().string()) == requestedStem &&
                    std::find(imageExtensions.begin(), imageExtensions.end(),
                              lower(entry.path().extension().string())) != imageExtensions.end()) {
                    gameLog.Log("Found texture directly at path: " + entry.path().generic_string());
                    return cacheResult(entry.path());
                }
            }
        }
        for (std::filesystem::path ancestor = root; !ancestor.empty();
             ancestor = ancestor.parent_path()) {
            const std::filesystem::path textureRoot = ancestor / "Texture";
            if (std::filesystem::exists(textureRoot)) {
                direct = textureRoot / cleaned;
                if (std::filesystem::exists(direct)) {
                    gameLog.Log("Found texture at direct path: " + direct.generic_string());
                    return cacheResult(direct);
                }
                for (const auto& entry : std::filesystem::recursive_directory_iterator(
                         textureRoot, std::filesystem::directory_options::skip_permission_denied)) {
                    if (entry.is_regular_file() &&
                        lower(entry.path().filename().string()) == basename) {
                        gameLog.Log("Found texture at recursive path: " +
                                    entry.path().generic_string());
                        return cacheResult(entry.path());
                    }
                }
                for (const auto& entry : std::filesystem::recursive_directory_iterator(
                         textureRoot, std::filesystem::directory_options::skip_permission_denied)) {
                    if (!entry.is_regular_file() ||
                        lower(entry.path().stem().string()) != requestedStem) {
                        continue;
                    }
                    const std::string extension = lower(entry.path().extension().string());
                    if (std::find(imageExtensions.begin(), imageExtensions.end(), extension) !=
                        imageExtensions.end()) {
                        gameLog.Log("Found texture at recursive path with extension: " +
                                    entry.path().generic_string());
                        return cacheResult(entry.path());
                    }
                }
            }
            const std::filesystem::path parent = ancestor.parent_path();
            if (parent == ancestor) {
                break;
            }
        }
        textureLog.Log("missing: " + cleaned);
        return cacheResult({});
    }

    std::string textureCacheKey(const Batch& batch) const {
        const std::filesystem::path identity = batch.texturePath.empty()
                                                   ? batch.textureRoot / batch.textureName
                                                   : batch.texturePath;
        return lower(std::filesystem::absolute(identity).lexically_normal().generic_string());
    }

    std::shared_ptr<TextureCacheEntry> textureEntry(Batch& batch) {
        const std::string key = textureCacheKey(batch);
        const auto found = textureCache.find(key);
        if (found != textureCache.end()) {
            return found->second;
        }
        const std::string alias = lower(std::filesystem::path(
                             batch.texturePath.empty() ? batch.textureName
                                           : batch.texturePath.string())
                             .stem()
                             .generic_string());
        if (!alias.empty()) {
            const auto aliasFound = textureAliases.find(alias);
            if (aliasFound != textureAliases.end()) {
                textureCache.emplace(key, aliasFound->second);
                return aliasFound->second;
            }
        }
        auto entry = std::make_shared<TextureCacheEntry>();
        entry->request = std::make_shared<TextureRequest>();
        entry->request->root = batch.textureRoot;
        entry->request->path = batch.texturePath;
        entry->request->name = batch.textureName;
        textureCache.emplace(key, entry);
        if (!alias.empty()) {
            textureAliases.emplace(alias, entry);
        }
        return entry;
    }

    void startTextureRequest(const std::shared_ptr<TextureCacheEntry>& entry) {
        if (!entry->request->worker.joinable()) {
            entry->request->worker = std::thread(&BusModel::loadTextureRequest, this,
                                                 entry->request);
        }
    }

    void ensureTexture(Batch& batch, bool visible = true) {
        if (batch.textureLoadAttempted) {
            return;
        }
        if (!batch.textureCacheEntry) {
            batch.textureCacheEntry = textureEntry(batch);
            batch.textureRequest = batch.textureCacheEntry->request;
            batch.textureLoadStarted = true;
        }
        startTextureRequest(batch.textureCacheEntry);
        bool requestComplete = false;
        {
            std::lock_guard<std::mutex> lock(batch.textureCacheEntry->request->mutex);
            requestComplete = batch.textureCacheEntry->request->complete;
        }
        if (!requestComplete) {
            return;
        }
        if (batch.textureCacheEntry->request->worker.joinable()) {
            batch.textureCacheEntry->request->worker.join();
        }
        std::lock_guard<std::mutex> lock(batch.textureCacheEntry->request->mutex);
        if (!batch.textureCacheEntry->request->complete) {
            return;
        }
        if (!visible) {
            return;
        }
        batch.texturePath = batch.textureCacheEntry->request->resolvedPath;
        if (!batch.textureCacheEntry->uploadAttempted) {
            if (loadingPolicy.textureMode == AssetLoadingMode::Deferred) {
                constexpr auto textureUploadBudget = std::chrono::milliseconds(2);
                if (std::chrono::steady_clock::now() - textureUploadStart >=
                    textureUploadBudget) {
                    return;
                }
            }
            batch.textureCacheEntry->uploadAttempted = true;
            if (!batch.texturePath.empty()) {
                if (batch.textureCacheEntry->request->compressedTexture) {
                    batch.textureCacheEntry->texture = uploadCompressedTexture(
                        batch.texturePath,
                        *batch.textureCacheEntry->request->compressedTexture);
                } else {
                    batch.textureCacheEntry->texture = uploadTexture(
                        batch.texturePath, *batch.textureCacheEntry->request->image);
                }
            }
        }
        batch.texture = batch.textureCacheEntry->texture;
        batch.textureLoadAttempted = true;
        batch.textured = batch.texture != 0;
        if (batch.texturePath.empty()) {
            textureLog.Log("decode-failed: " + batch.textureName);
        }
        if (!batch.environmentLoadAttempted && !batch.environmentTextureName.empty() &&
            batch.environmentStrength > 0.0) {
            batch.environmentLoadAttempted = true;
            Batch environmentBatch;
            environmentBatch.textureRoot = batch.textureRoot;
            environmentBatch.textureName = batch.environmentTextureName;
            ensureTexture(environmentBatch, visible);
            batch.environmentTexture = environmentBatch.texture;
        }
    }

    void preloadTextures() {
        const auto preload = [&](std::vector<DisplayPart>& parts) {
            for (DisplayPart& part : parts) {
                for (Batch& batch : part.batches) {
                    if (!batch.texturePath.empty() || !batch.textureName.empty()) {
                        ensureTexture(batch, loadingPolicy.textureMode == AssetLoadingMode::Eager);
                    }
                }
            }
        };
        preload(displayLists);
        for (WheelModel& wheel : wheelModels) {
            preload(wheel.parts);
        }
    }

    void drawEnvironmentMap(const Batch& batch) const {
        if (!batch.environmentTexture || batch.environmentStrength <= 0.0) {
            return;
        }
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, batch.environmentTexture);
        glColor4d(1.0, 1.0, 1.0, std::clamp(batch.environmentStrength, 0.0, 1.0));
        pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(Vertex), nullptr);
        glTexCoordPointer(2, GL_FLOAT, sizeof(Vertex),
                          reinterpret_cast<const void*>(3 * sizeof(float)));
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(batch.vertexCount));
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisable(GL_TEXTURE_2D);
        glDisable(GL_BLEND);
        glColor4d(1.0, 1.0, 1.0, 1.0);
    }

    static std::shared_ptr<ParsedObj> parseObj(const std::filesystem::path& path) {
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
                              static_cast<int>(result->texCoords.size()));
                for (std::size_t index = 2; index < face.size(); ++index) {
                    result->triangles.push_back(
                        {{{face[0], face[index - 1], face[index]}}, currentMaterial});
                }
            }
        }
        if (result->triangles.empty()) {
            return {};
        }
        result->boundsCenter = {std::numeric_limits<double>::max(),
                                std::numeric_limits<double>::max(),
                                std::numeric_limits<double>::max()};
        std::array<double, 3> boundsMax = {std::numeric_limits<double>::lowest(),
                                           std::numeric_limits<double>::lowest(),
                                           std::numeric_limits<double>::lowest()};
        for (const ObjPosition& position : result->positions) {
            const std::array<double, 3> converted = {position.z, -position.x, position.y};
            for (int axis = 0; axis < 3; ++axis) {
                result->boundsCenter[axis] = std::min(result->boundsCenter[axis], converted[axis]);
                boundsMax[axis] = std::max(boundsMax[axis], converted[axis]);
            }
        }
        for (int axis = 0; axis < 3; ++axis) {
            result->boundsCenter[axis] = (result->boundsCenter[axis] + boundsMax[axis]) * 0.5;
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
                    result->materials[materialName] = {};
                } else if (materialType == "map_Kd" && !materialName.empty()) {
                    std::string textureName;
                    std::getline(materialStream, textureName);
                    result->materials[materialName].textureName = trim(textureName);
                } else if (materialType == "Kd" && !materialName.empty()) {
                    materialStream >> result->materials[materialName].color[0] >>
                        result->materials[materialName].color[1] >>
                        result->materials[materialName].color[2];
                }
            }
        }
        return result;
    }

    void loadObj(const Part& part, const std::shared_ptr<ParsedObj>& parsed) {
        gameLog.Log("Loading OBJ model from path: " + part.objPath.generic_string());
        if (!parsed) {
            return;
        }
        const auto& positions = parsed->positions;
        const auto& texCoords = parsed->texCoords;
        const auto& triangles = parsed->triangles;
        const auto& materials = parsed->materials;
        const auto& boundsCenter = parsed->boundsCenter;
        const double boundsRadius = parsed->boundsRadius;

        WheelAnimation wheelAnimation = part.wheelAnimation;
        if (!wheelAnimation.rotationVariable.empty() && !wheelAnimation.hasOrigin) {
            wheelAnimation.origin = boundsCenter;
            wheelAnimation.hasOrigin = true;
        }
        std::vector<DisplayPart>* destination = &displayLists;
        if (!wheelAnimation.rotationVariable.empty()) {
            auto wheel = std::find_if(
                wheelModels.begin(), wheelModels.end(), [&](const WheelModel& candidate) {
                    const bool sameAnimation =
                        wheelAnimation.rotationVariable == candidate.animation.rotationVariable &&
                        wheelAnimation.suspensionVariable ==
                            candidate.animation.suspensionVariable &&
                        wheelAnimation.steeringVariable == candidate.animation.steeringVariable;
                    if (sameAnimation) {
                        return true;
                    }
                    if (wheelAnimation.rotationVariable.empty() ||
                        candidate.animation.rotationVariable.empty()) {
                        return false;
                    }
                    if (wheelAnimation.hasOrigin && candidate.animation.hasOrigin) {
                        const double dx = wheelAnimation.origin[0] - candidate.animation.origin[0];
                        const double dy = wheelAnimation.origin[1] - candidate.animation.origin[1];
                        const double dz = wheelAnimation.origin[2] - candidate.animation.origin[2];
                        return dx * dx + dy * dy + dz * dz < 0.0001;
                    }
                    return false;
                });
            if (wheel == wheelModels.end()) {
                wheelModels.push_back({wheelAnimation, {}});
                wheel = wheelModels.end() - 1;
            }
            destination = &wheel->parts;
        }
        auto makeBatch = [&](const std::vector<const ObjTriangle*>& source,
                             const std::filesystem::path& texturePath,
                             const std::string& textureName, const std::array<double, 3>& color,
                             const std::string& environmentTextureName, double environmentStrength,
                             int alphaMode, bool noZwrite) {
            std::vector<Vertex> vertices;
            vertices.reserve(source.size() * 3);
            for (const ObjTriangle* triangle : source) {
                for (const ObjIndex& index : triangle->indices) {
                    if (index.position <= 0 ||
                        index.position > static_cast<int>(positions.size())) {
                        continue;
                    }
                    const ObjPosition& position =
                        positions[static_cast<std::size_t>(index.position - 1)];
                    const ObjTexCoord* texCoord = nullptr;
                    if (index.texCoord > 0 &&
                        index.texCoord <= static_cast<int>(texCoords.size())) {
                        texCoord = &texCoords[static_cast<std::size_t>(index.texCoord - 1)];
                    }
                    vertices.push_back({static_cast<float>(position.z),
                                        static_cast<float>(-position.x),
                                        static_cast<float>(position.y),
                                        texCoord ? static_cast<float>(texCoord->u) : 0.0f,
                                        texCoord ? static_cast<float>(1.0 - texCoord->v) : 0.0f});
                }
            }
            if (vertices.empty())
                return;
            Batch batch;
            batch.texturePath = texturePath;
            batch.textureRoot = part.objPath.parent_path();
            batch.textureName = textureName;
            batch.environmentTextureName = environmentTextureName;
            batch.environmentStrength = environmentStrength;
            batch.color = color;
            batch.alphaMode = alphaMode;
            batch.noZwrite = noZwrite;
            batch.vertexCount = vertices.size();
            pglGenBuffers(1, &batch.buffer);
            pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
            pglBufferData(GL_ARRAY_BUFFER,
                          static_cast<std::ptrdiff_t>(vertices.size() * sizeof(Vertex)),
                          vertices.data(), GL_STATIC_DRAW);
            pglBindBuffer(GL_ARRAY_BUFFER, 0);
            destination->back().batches.push_back(std::move(batch));
        };

        std::unordered_map<std::string, std::vector<const ObjTriangle*>> groups;
        std::unordered_map<std::string, std::filesystem::path> groupTextures;
        std::unordered_map<std::string, std::string> groupTextureNames;
        std::unordered_map<std::string, std::string> groupEnvironmentNames;
        std::unordered_map<std::string, double> groupEnvironmentStrengths;
        std::unordered_map<std::string, std::array<double, 3>> groupColors;
        std::unordered_map<std::string, MaterialState> groupStates;
        std::vector<std::string> groupOrder;
        const std::filesystem::path fallbackTexturePath = part.texturePath;
        const std::string fallbackTextureName = part.textureName;
        std::size_t renderedTriangleCount = 0;
        bool hasTransparentMaterial = false;
        for (const auto& triangle : triangles) {
            const std::string materialName = lower(triangle.material);
            ++renderedTriangleCount;
            const auto material = materials.find(triangle.material);
            std::filesystem::path texturePath =
                material != materials.end() && !material->second.texturePath.empty()
                    ? material->second.texturePath
                    : fallbackTexturePath;
            std::string textureName =
                material != materials.end() && !material->second.textureName.empty()
                    ? material->second.textureName
                    : fallbackTextureName;
            std::array<double, 3> color =
                material != materials.end() ? material->second.color : part.color;
            const std::string key = triangle.material.empty() ? "__fallback__" : triangle.material;
            if (groups.find(key) == groups.end()) {
                groupOrder.push_back(key);
            }
            MaterialState state;
            if (!textureName.empty() || !texturePath.empty()) {
                const std::filesystem::path statePath =
                    textureName.empty() ? texturePath : std::filesystem::path(textureName);
                const std::string stateKey = lower(statePath.filename().string());
                const auto found = part.materialStates.find(stateKey);
                if (found != part.materialStates.end()) {
                    state = found->second;
                } else {
                    const std::string stateStem = lower(statePath.stem().string());
                    const auto matchingState = std::find_if(
                        part.materialStates.begin(), part.materialStates.end(),
                        [&](const auto& entry) {
                            return lower(std::filesystem::path(entry.first).stem().string()) ==
                                   stateStem;
                        });
                    if (matchingState != part.materialStates.end()) {
                        state = matchingState->second;
                    } else {
                        const std::string materialStem = lower(triangle.material);
                        const auto matchingMaterial = std::find_if(
                            part.materialStates.begin(), part.materialStates.end(),
                            [&](const auto& entry) {
                                return lower(std::filesystem::path(entry.first).stem().string()) ==
                                       materialStem;
                            });
                        if (matchingMaterial != part.materialStates.end()) {
                            state = matchingMaterial->second;
                        }
                    }
                }
            }
            if (!state.textureName.empty()) {
                textureName = state.textureName;
                color = {1.0, 1.0, 1.0};
            }
            if (state.alphaMode == 0) {
                state.noZwrite = false;
            }
            groups[key].push_back(&triangle);
            groupTextures[key] = texturePath;
            groupTextureNames[key] = textureName;
            groupEnvironmentNames[key] = state.environmentTextureName;
            groupEnvironmentStrengths[key] = state.environmentStrength;
            groupColors[key] = color;
            groupStates[key] = state;
            hasTransparentMaterial =
                hasTransparentMaterial || state.alphaMode != 0 || state.noZwrite;
        }
        destination->push_back({{},
                                part.viewpoint,
                                part.renderType,
                                hasTransparentMaterial,
                                part.lodIndex,
                                boundsCenter,
                                boundsRadius,
                                renderedTriangleCount});
        for (const std::string& key : groupOrder) {
            const MaterialState& state = groupStates[key];
            makeBatch(groups[key], groupTextures[key], groupTextureNames[key], groupColors[key],
                      groupEnvironmentNames[key], groupEnvironmentStrengths[key], state.alphaMode,
                      state.noZwrite);
        }
    }

    int lodForDistance(double distance) const {
        if (lodThresholds.empty()) {
            return -1;
        }
        for (std::size_t index = 0; index < lodThresholds.size(); ++index) {
            const double boundary = 25.0 / std::max(lodThresholds[index], 0.001);
            if (distance <= boundary) {
                return static_cast<int>(index);
            }
        }
        return static_cast<int>(lodThresholds.size() - 1);
    }

    void rebuildDisplayOrder() {
        std::stable_sort(displayLists.begin(), displayLists.end(),
                         [](const DisplayPart& first, const DisplayPart& second) {
                             return first.transparent < second.transparent;
                         });
        opaqueDisplayCount = 0;
        while (opaqueDisplayCount < displayLists.size() &&
               !displayLists[opaqueDisplayCount].transparent) {
            ++opaqueDisplayCount;
        }
    }

    // Load visible OBJ geometry immediately; texture decoding remains asynchronous.
    void ensureLoaded(RenderViewContext context, double viewDistance) {
        const int selectedLod = lodForDistance(viewDistance);
        activeLod = selectedLod;
        bool loadedPart = false;
        const bool immediateLoad = !hasLoadedInitialView ||
                       loadingPolicy.modelMode == AssetLoadingMode::Eager;
        const auto loadStart = std::chrono::steady_clock::now();
        constexpr auto loadBudget = std::chrono::milliseconds(4);
        const auto budgetReached = [&] {
            return !immediateLoad && std::chrono::steady_clock::now() - loadStart >= loadBudget;
        };

        for (Part& part : pendingParts) {
            const bool viewpointMatches =
                part.viewpoint == 0 || (part.viewpoint & viewpointMask(context)) != 0;
            if (!viewpointMatches || (selectedLod >= 0 && part.lodIndex != selectedLod)) {
                if (!part.objRequest) {
                    part.objRequest = std::make_shared<ObjRequest>();
                    part.objRequest->future =
                        std::async(std::launch::async, &BusModel::parseObj, part.objPath);
                }
            }
        }

        auto part = pendingParts.begin();
        while (part != pendingParts.end()) {
            if (budgetReached()) {
                break;
            }
            const bool viewpointMatches =
                part->viewpoint == 0 || (part->viewpoint & viewpointMask(context)) != 0;
            const bool shouldLoad = loadingPolicy.modelMode == AssetLoadingMode::Eager ||
                                    (viewpointMatches &&
                                     (selectedLod < 0 || part->lodIndex == selectedLod));
            if (!shouldLoad) {
                ++part;
                continue;
            }
            const std::shared_ptr<ParsedObj> parsed = part->objRequest
                                                          ? part->objRequest->future.get()
                                                          : parseObj(part->objPath);
            loadObj(*part, parsed);
            part = pendingParts.erase(part);
            loadedPart = true;
        }

        auto backgroundPart = pendingParts.begin();
        while (backgroundPart != pendingParts.end() && !budgetReached()) {
            const bool viewpointMatches =
                backgroundPart->viewpoint == 0 ||
                (backgroundPart->viewpoint & viewpointMask(context)) != 0;
            const bool lodMatches = selectedLod < 0 || backgroundPart->lodIndex == selectedLod;
            if ((viewpointMatches && lodMatches) || !backgroundPart->objRequest ||
                backgroundPart->objRequest->future.wait_for(std::chrono::milliseconds(0)) !=
                    std::future_status::ready) {
                ++backgroundPart;
                continue;
            }
            const std::shared_ptr<ParsedObj> parsed = backgroundPart->objRequest->future.get();
            loadObj(*backgroundPart, parsed);
            backgroundPart = pendingParts.erase(backgroundPart);
            loadedPart = true;
        }
        hasLoadedInitialView = true;
        if (loadedPart || loadingPolicy.textureMode == AssetLoadingMode::Eager) {
            rebuildDisplayOrder();
            preloadTextures();
        }
        loaded = !displayLists.empty() || !pendingParts.empty();
    }

    void load(const std::filesystem::path& configPath, const std::filesystem::path& modelRoot) {
        std::ifstream input(configPath);
        if (!input) {
            return;
        }
        std::vector<Part> parts;
        std::string line;
        std::string lastMaterialKey;
        int currentLodIndex = -1;
        while (std::getline(input, line)) {
            const std::string value = trim(line);
            if (value == "[LOD]") {
                std::string lodValue;
                std::getline(input, lodValue);
                const double threshold = std::stod(trim(lodValue));
                if (threshold > 0.0) {
                    currentLodIndex = static_cast<int>(lodThresholds.size());
                    lodThresholds.push_back(threshold);
                } else {
                    currentLodIndex = -1;
                }
                continue;
            }
            if (currentLodIndex < 0) {
                continue;
            }
            if (value == "[mesh]") {
                std::string mesh;
                if (std::getline(input, mesh)) {
                    std::string normalized = trim(mesh);
                    std::replace(normalized.begin(), normalized.end(), '\\', '/');
                    std::filesystem::path objPath = modelRoot / normalized;
                    objPath.replace_extension(".obj");
                    if (!std::filesystem::exists(objPath)) {
                        objPath = modelRoot / std::filesystem::path(normalized).filename();
                        objPath.replace_extension(".obj");
                    }
                    if (std::filesystem::exists(objPath)) {
                        Part part;
                        part.objPath = objPath;
                        part.lodIndex = currentLodIndex;
                        parts.push_back(std::move(part));
                        lastMaterialKey.clear();
                    }
                }
            } else if (value == "[viewpoint]" && !parts.empty()) {
                std::string viewpoint;
                if (std::getline(input, viewpoint)) {
                    const int viewpointFlag = parseInt(viewpoint, 0);
                    parts.back().viewpoint = viewpointFlag;
                }
            } else if (value == "[rendertype]" && !parts.empty()) {
                std::string renderType;
                if (std::getline(input, renderType)) {
                    parts.back().renderType = parseInt(renderType, 2);
                }
            } else if (value == "[mesh_ident]" && !parts.empty()) {
                std::string identifier;
                if (std::getline(input, identifier)) {
                    parts.back().meshIdentifier = trim(identifier);
                }
            } else if (value == "[animparent]" && !parts.empty()) {
                std::string parent;
                if (std::getline(input, parent)) {
                    parts.back().animationParent = trim(parent);
                }
            } else if (value == "[newanim]" && !parts.empty()) {
                WheelAnimation animation;
                std::string animationVariable;
                std::string animationLine;
                while (readNonEmptyLine(input, animationLine)) {
                    if (animationLine == "origin_trans") {
                        std::array<double, 3> sourceOrigin = {};
                        bool validOrigin = true;
                        for (double& coordinate : sourceOrigin) {
                            std::string coordinateText;
                            if (!readNonEmptyLine(input, coordinateText)) {
                                validOrigin = false;
                                break;
                            }
                            coordinate = parseDouble(coordinateText, 0.0);
                        }
                        if (validOrigin) {
                            animation.origin = {sourceOrigin[1], -sourceOrigin[0], sourceOrigin[2]};
                            animation.hasOrigin = true;
                        }
                    } else if (animationLine == "origin_rot_x" || animationLine == "origin_rot_y" ||
                               animationLine == "origin_rot_z") {
                        std::string angle;
                        if (!readNonEmptyLine(input, angle)) {
                            break;
                        }
                    } else if (animationLine == "anim_rot" || animationLine == "anim_trans") {
                        std::string variable;
                        std::string scale;
                        if (!readNonEmptyLine(input, variable) || !readNonEmptyLine(input, scale)) {
                            break;
                        }
                        animationVariable = variable;
                        break;
                    }
                }
                const std::string variable = lower(animationVariable);
                if (variable.rfind("wheel_rotation_", 0) == 0) {
                    parts.back().wheelAnimation.rotationVariable = animationVariable;
                    if (animation.hasOrigin) {
                        parts.back().wheelAnimation.origin = animation.origin;
                        parts.back().wheelAnimation.hasOrigin = true;
                    }
                } else if (variable.rfind("axle_suspension_", 0) == 0) {
                    parts.back().wheelAnimation.suspensionVariable = animationVariable;
                } else if (variable.rfind("axle_steering_", 0) == 0) {
                    parts.back().wheelAnimation.steeringVariable = animationVariable;
                }
            } else if (value == "[matl]" && !parts.empty()) {
                std::string texture;
                std::string slot;
                if (std::getline(input, texture) && std::getline(input, slot)) {
                    const std::string textureName = trim(texture);
                    lastMaterialKey = lower(std::filesystem::path(textureName).filename().string());
                    parts.back().materialStates[lastMaterialKey] = {{},  textureName, {},
                                                                    0.0, 0,           false};
                    if (parts.back().textureName.empty()) {
                        parts.back().textureName = textureName;
                    }
                }
            } else if (value == "[matl_alpha]" && !parts.empty()) {
                std::string alphaMode;
                if (std::getline(input, alphaMode) && !lastMaterialKey.empty()) {
                    parts.back().materialStates[lastMaterialKey].alphaMode = parseInt(alphaMode, 0);
                }
            } else if (value == "[matl_noZwrite]" && !parts.empty() && !lastMaterialKey.empty()) {
                parts.back().materialStates[lastMaterialKey].noZwrite = true;
            } else if (value == "[matl_envmap]" && !parts.empty() && !lastMaterialKey.empty()) {
                std::string texture;
                std::string strength;
                if (std::getline(input, texture) && std::getline(input, strength)) {
                    parts.back().materialStates[lastMaterialKey].environmentTextureName =
                        trim(texture);
                    parts.back().materialStates[lastMaterialKey].environmentStrength =
                        parseDouble(strength, 0.0);
                }
            } else if (value == "[matl_change]" && !parts.empty()) {
                std::string texture;
                std::string slot;
                if (std::getline(input, texture) && std::getline(input, slot)) {
                    lastMaterialKey.clear();
                }
            }
        }
        for (Part& part : parts) {
            if (part.animationParent.empty()) {
                continue;
            }
            const auto parent =
                std::find_if(parts.begin(), parts.end(), [&](const Part& candidate) {
                    return candidate.meshIdentifier == part.animationParent;
                });
            if (parent != parts.end() && !parent->wheelAnimation.rotationVariable.empty()) {
                part.wheelAnimation = parent->wheelAnimation;
            }
        }
        pendingParts = std::move(parts);
        loaded = !pendingParts.empty();
    }
};

void BusModel::loadTextureRequest(const std::shared_ptr<TextureRequest>& request) {
#ifdef _WIN32
    const HRESULT comResult = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
#endif
    std::filesystem::path resolved = request->path;
    if (resolved.empty()) {
        resolved = findTexture(request->root, request->name);
    }
    const std::string resolvedKey = resolved.empty()
                                        ? std::string()
                                        : lower(std::filesystem::absolute(resolved)
                                                    .lexically_normal()
                                                    .generic_string());
    if (!resolvedKey.empty()) {
        std::shared_ptr<DecodedTexture> cachedTexture;
        {
            std::lock_guard<std::mutex> lock(decodedTextureMutex);
            const auto cached = decodedTextureCache.find(resolvedKey);
            if (cached != decodedTextureCache.end()) {
                cachedTexture = cached->second;
            }
        }
        if (cachedTexture) {
            std::lock_guard<std::mutex> requestLock(request->mutex);
            request->resolvedPath = resolved;
            request->image = cachedTexture->image;
            request->compressedTexture = cachedTexture->compressedTexture;
            request->complete = true;
            return;
        }
    }
    Image image;
    bool textureLoaded = false;
    std::shared_ptr<gli::texture> compressedTexture;
    if (!resolved.empty() && lower(resolved.extension().string()) == ".dds" &&
        isSafeCompressedDds(resolved)) {
        try {
            gli::texture loaded = gli::load(resolved.string());
            if (!loaded.empty() && gli::is_compressed(loaded.format())) {
                compressedTexture = std::make_shared<gli::texture>(std::move(loaded));
                textureLoaded = true;
            }
        } catch (const std::exception& error) {
            gameLog.Log("GLI failed to load compressed texture " + resolved.generic_string() +
                        ": " + error.what());
        }
    }
    if (!textureLoaded) {
        textureLoaded = !resolved.empty() && readImage(resolved, image);
    }
#ifdef _WIN32
    if (SUCCEEDED(comResult)) {
        CoUninitialize();
    }
#endif
    std::lock_guard<std::mutex> lock(request->mutex);
    if (textureLoaded) {
        request->resolvedPath = std::move(resolved);
        request->image = std::make_shared<Image>(std::move(image));
        request->compressedTexture = std::move(compressedTexture);
        if (!resolvedKey.empty()) {
            auto decoded = std::make_shared<DecodedTexture>();
            decoded->image = request->image;
            decoded->compressedTexture = request->compressedTexture;
            std::lock_guard<std::mutex> cacheLock(decodedTextureMutex);
            decodedTextureCache.emplace(resolvedKey, std::move(decoded));
        }
    }
    request->complete = true;
}

void Renderer::scrollCallback(GLFWwindow* window, double, double yOffset) {
    auto* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
    if (renderer == nullptr) {
        return;
    }
    renderer->cameraDistance_ = std::clamp(renderer->cameraDistance_ - yOffset * 2.0, 6.0, 80.0);
}

Renderer::Renderer(int width, int height, const char* title, BusVehicle vehicle,
                   ModelLoadingPolicy loadingPolicy)
    : window_(nullptr) {
    if (!glfwInit()) {
        gameLog.Log("Failed to initialize GLFW");
        throw std::runtime_error("Failed to initialize GLFW");
    }
    window_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!window_) {
        gameLog.Log("Failed to create OpenGL window");
        glfwTerminate();
        throw std::runtime_error("Failed to create OpenGL window");
    }
    glfwMakeContextCurrent(window_);
    glfwSetWindowUserPointer(window_, this);
    glfwSetScrollCallback(window_, &Renderer::scrollCallback);
    const char* vsyncSetting = std::getenv("OPENBUS_VSYNC");
    const bool vsyncEnabled = vsyncSetting == nullptr ||
                              (std::string(vsyncSetting) != "0" &&
                               std::string(vsyncSetting) != "off" &&
                               std::string(vsyncSetting) != "false");
    glfwSwapInterval(vsyncEnabled ? 1 : 0);
    gameLog.Log(std::string("VSync ") + (vsyncEnabled ? "enabled" : "disabled"));
    if (!loadBufferFunctions()) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
        glfwTerminate();
        throw std::runtime_error("OpenGL VBO functions are unavailable");
    }
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.45f, 0.65f, 0.88f, 1.0f);
    busModel_ = std::make_unique<BusModel>(vehicle, loadingPolicy);
    gameLog.Log("Renderer initialized");
}

Renderer::~Renderer() {
    gameLog.Log("Renderer shutting down");
    busModel_.reset();
    if (window_) {
        glfwDestroyWindow(window_);
    }
    glfwTerminate();
}

bool Renderer::shouldClose() const {
    return glfwWindowShouldClose(window_) != 0;
}

void Renderer::beginFrame() {
    glfwPollEvents();
    const std::array<int, 4> keys = {GLFW_KEY_W, GLFW_KEY_A, GLFW_KEY_S, GLFW_KEY_D};
    const std::array<const char*, 4> keyNames = {"W", "A", "S", "D"};
    for (std::size_t index = 0; index < keys.size(); ++index) {
        const bool pressed = glfwGetKey(window_, keys[index]) == GLFW_PRESS;
        if (pressed != previousKeyStates_[index]) {
            keyEvents_.push_back({keyNames[index], pressed, glfwGetTime()});
            gameLog.Log(std::string("Key ") + keyNames[index] +
                        (pressed ? " pressed" : " released"));
            previousKeyStates_[index] = pressed;
        }
    }
    for (std::size_t index = 0; index < previousViewKeyStates_.size(); ++index) {
        const int key = GLFW_KEY_0 + static_cast<int>(index);
        const bool pressed = glfwGetKey(window_, key) == GLFW_PRESS;
        if (pressed && !previousViewKeyStates_[index]) {
            cameraView_ = static_cast<int>(index);
            viewLookYaw_ = 0.0;
            viewLookPitch_ = 0.0;
            gameLog.Log("Changed camera view to " + std::to_string(cameraView_));
        }
        previousViewKeyStates_[index] = pressed;
    }
    const bool captureKeyPressed = glfwGetKey(window_, GLFW_KEY_F12) == GLFW_PRESS;
    if (captureKeyPressed && !previousCaptureKeyState_) {
        captureRequested_ = true;
    }
    previousCaptureKeyState_ = captureKeyPressed;
    int width = 1;
    int height = 1;
    glfwGetFramebufferSize(window_, &width, &height);
    double cursorX = 0.0;
    double cursorY = 0.0;
    glfwGetCursorPos(window_, &cursorX, &cursorY);
    const bool middleMouse = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    if (middleMouse && !draggingCamera_) {
        previousCursorX_ = cursorX;
        previousCursorY_ = cursorY;
    } else if (middleMouse) {
        const double cursorDeltaX = cursorX - previousCursorX_;
        const double cursorDeltaY = cursorY - previousCursorY_;
        if (cameraView_ == 0) {
            cameraYaw_ -= cursorDeltaX * 0.005;
            cameraPitch_ -= cursorDeltaY * 0.005;
            cameraPitch_ = std::clamp(cameraPitch_, -1.35, 1.35);
        } else {
            viewLookYaw_ -= cursorDeltaX * 0.005;
            viewLookPitch_ -= cursorDeltaY * 0.005;
            viewLookPitch_ = std::clamp(viewLookPitch_, -1.35, 1.35);
        }
    }
    draggingCamera_ = middleMouse;
    previousCursorX_ = cursorX;
    previousCursorY_ = cursorY;
    glViewport(0, 0, width, height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    setPerspective(static_cast<double>(width), static_cast<double>(height));
}

void Renderer::draw(const BusSimulation& simulation) {
    const BodyPose chassis = simulation.chassisPose();
    const ChassisCollisionBox collision = simulation.chassisCollisionBox();
    if (cameraView_ == 0) {
        const double targetX = chassis.position[0];
        const double targetY = chassis.position[1];
        const double targetZ = chassis.position[2] + collision.offsetZ;
        const double horizontalDistance = cameraDistance_ * std::cos(cameraPitch_);
        const double orbitYaw = simulation.yaw() + cameraYaw_;
        const double eyeX = targetX + horizontalDistance * std::cos(orbitYaw);
        const double eyeY = targetY + horizontalDistance * std::sin(orbitYaw);
        const double eyeZ = targetZ + cameraDistance_ * std::sin(cameraPitch_);
        lookAt(eyeX, eyeY, eyeZ, targetX, targetY, targetZ);
    } else {
        std::array<double, 3> eyeLocal = {5.65, 0.70, 0.70};
        std::array<double, 3> targetLocal = {8.65, 0.70, 0.72};
        switch (cameraView_) {
        case 2:
            eyeLocal = {4.25, -0.55, 0.78};
            targetLocal = {7.25, -0.55, 0.80};
            break;
        case 3:
            eyeLocal = {-2.75, 0.0, 0.72};
            targetLocal = {0.25, 0.0, 0.74};
            break;
        case 4:
            eyeLocal = {5.35, 1.05, 0.86};
            targetLocal = {5.35, 4.0, 0.82};
            break;
        case 5:
            eyeLocal = {5.35, -1.05, 0.86};
            targetLocal = {5.35, -4.0, 0.82};
            break;
        case 6:
            eyeLocal = {6.85, 0.0, 0.80};
            targetLocal = {3.85, 0.0, 0.80};
            break;
        case 7:
            eyeLocal = {-6.15, 0.0, 0.80};
            targetLocal = {-3.15, 0.0, 0.80};
            break;
        case 8:
            eyeLocal = {0.0, 1.05, 1.30};
            targetLocal = {0.0, 4.0, 1.30};
            break;
        case 9:
            eyeLocal = {0.0, -1.05, 1.30};
            targetLocal = {0.0, -4.0, 1.30};
            break;
        default:
            break;
        }
        const std::array<double, 3> eye = transformLocalPoint(chassis, eyeLocal);
        const double directionX = targetLocal[0] - eyeLocal[0];
        const double directionY = targetLocal[1] - eyeLocal[1];
        const double directionZ = targetLocal[2] - eyeLocal[2];
        const double horizontalLength = std::hypot(directionX, directionY);
        const double distance =
            std::sqrt(directionX * directionX + directionY * directionY + directionZ * directionZ);
        const double baseYaw = std::atan2(directionY, directionX);
        const double basePitch = std::atan2(directionZ, horizontalLength);
        const double lookYaw = baseYaw + viewLookYaw_;
        const double lookPitch = basePitch + viewLookPitch_;
        const std::array<double, 3> target = transformLocalPoint(
            chassis, {eyeLocal[0] + distance * std::cos(lookPitch) * std::cos(lookYaw),
                      eyeLocal[1] + distance * std::cos(lookPitch) * std::sin(lookYaw),
                      eyeLocal[2] + distance * std::sin(lookPitch)});
        lookAt(eye[0], eye[1], eye[2], target[0], target[1], target[2]);
    }

    glPushMatrix();
    drawGround(simulation.roadBumps());
    glPopMatrix();

    const RenderViewContext context =
        cameraView_ == 0 ? RenderViewContext::PlayerExterior : RenderViewContext::PlayerInterior;

        if (busModel_) {
        // The first draw establishes the active viewpoint/LOD. Subsequent camera changes
        // populate only the newly eligible parts.
        const double modelViewDistance = cameraView_ == 0 ? cameraDistance_ : 3.0;
        busModel_->loadForView(context, modelViewDistance);
    }

    if (busModel_ && busModel_->loaded && !busModel_->displayLists.empty()) {
        glPushMatrix();
        applyPose(chassis);
        busModel_->draw(context);
        glPopMatrix();
    } else {
        glPushMatrix();
        applyPose(chassis);
        glTranslated(0.0, 0.0, collision.offsetZ);
        drawBox(collision.length, collision.width, collision.height, 0.85, 0.70, 0.08);
        glPopMatrix();
    }

    if (busModel_ && glfwGetTime() - lastStatsTitleTime_ > 0.25) {
        std::ostringstream title;
        title << "OpenBus - " << busModel_->renderedTriangles() << " triangles";
        glfwSetWindowTitle(window_, title.str().c_str());
        lastStatsTitleTime_ = glfwGetTime();
    }

    // Render center of gravity marker
    const std::array<double, 3> centerOfGravity = simulation.centerOfGravity();
    glPushMatrix();
    glTranslated(centerOfGravity[0], centerOfGravity[1], centerOfGravity[2]);
    drawCenterOfGravityMarker(0.35);
    glPopMatrix();

    // Render axle lines
    glPushMatrix();
    glColor3d(0.20, 0.20, 0.20);
    for (std::size_t axleIndex = 0; axleIndex < simulation.axleCount(); ++axleIndex) {
        const BodyPose leftWheel = simulation.wheelPose(axleIndex * 2);
        const BodyPose rightWheel = simulation.wheelPose(axleIndex * 2 + 1);
        glBegin(GL_LINES);
        glVertex3dv(rightWheel.position.data());
        glVertex3dv(leftWheel.position.data());
        glEnd();
    }
    glPopMatrix();

    if (busModel_ && busModel_->hasConfiguredWheels(simulation.wheelCount())) {
        busModel_->drawConfiguredWheels(simulation, chassis, cameraView_ == 0);
    } else {
        for (std::size_t index = 0; index < simulation.wheelCount(); ++index) {
            const BodyPose wheel = simulation.wheelPose(index);
            glPushMatrix();
            applyPose(wheel);
            drawWheel(simulation.wheelRadius(), simulation.wheelHalfWidth());
            glPopMatrix();
        }
    }
    drawCollisionWireframe(simulation);
}

void Renderer::endFrame() {
    glfwSwapBuffers(window_);
}

void Renderer::captureViews(const BusSimulation& simulation,
                            const std::filesystem::path& directory) {
    std::filesystem::create_directories(directory);
    int width = 1;
    int height = 1;
    glfwGetFramebufferSize(window_, &width, &height);

    const int previousCameraView = cameraView_;
    const double previousCameraYaw = cameraYaw_;
    const double previousCameraPitch = cameraPitch_;
    const double previousCameraDistance = cameraDistance_;
    const double previousViewLookYaw = viewLookYaw_;
    const double previousViewLookPitch = viewLookPitch_;
    captureMode_ = true;
    cameraView_ = 0;
    cameraDistance_ = 24.0;
    viewLookYaw_ = 0.0;
    viewLookPitch_ = 0.0;

    struct CaptureView {
        const char* name;
        double yaw;
        double pitch;
        double distance;
    };
    const std::array<CaptureView, 4> views = {{{"top", 0.0, 1.25, 28.0},
                                                {"front", 0.0, 0.25, 18.0},
                                                {"left", 1.5707963267948966, 0.25, 18.0},
                                                {"right", -1.5707963267948966, 0.25, 18.0}}};
    for (const CaptureView& view : views) {
        cameraYaw_ = view.yaw;
        cameraPitch_ = view.pitch;
        cameraDistance_ = view.distance;
        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        setPerspective(static_cast<double>(width), static_cast<double>(height));
        draw(simulation);
        glFinish();
        const std::filesystem::path path = directory / (std::string(view.name) + ".bmp");
        if (!saveFramebufferBmp(path, width, height)) {
            gameLog.Log("Failed to save screenshot: " + path.string());
        } else {
            gameLog.Log("Saved screenshot: " + path.string());
        }
    }

    cameraView_ = previousCameraView;
    cameraYaw_ = previousCameraYaw;
    cameraPitch_ = previousCameraPitch;
    cameraDistance_ = previousCameraDistance;
    viewLookYaw_ = previousViewLookYaw;
    viewLookPitch_ = previousViewLookPitch;
    captureMode_ = false;
}

bool Renderer::consumeCaptureRequest() {
    const bool requested = captureRequested_;
    captureRequested_ = false;
    return requested;
}

double Renderer::throttle() const {
    return glfwGetKey(window_, GLFW_KEY_W) == GLFW_PRESS ? 1.0 : 0.0;
}

double Renderer::steering() const {
    const bool left = glfwGetKey(window_, GLFW_KEY_A) == GLFW_PRESS;
    const bool right = glfwGetKey(window_, GLFW_KEY_D) == GLFW_PRESS;
    return static_cast<double>(right) - static_cast<double>(left);
}

double Renderer::brake() const {
    return glfwGetKey(window_, GLFW_KEY_S) == GLFW_PRESS ? 1.0 : 0.0;
}

std::vector<KeyEvent> Renderer::consumeKeyEvents() {
    std::vector<KeyEvent> events;
    events.swap(keyEvents_);
    return events;
}
