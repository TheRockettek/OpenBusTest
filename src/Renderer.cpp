#include "Renderer.h"

#include "Logger.h"

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
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
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

constexpr double MAN_DL05_MODEL_OFFSET_Z = -1.035;
constexpr int ROAD_BUMP_SEGMENTS = 12;
constexpr int ROAD_RAMP_SEGMENTS = 6;
constexpr int ROAD_INCLINE_SEGMENTS = 16;

using GenBuffersProc = void (*)(GLsizei, GLuint*);
using BindBufferProc = void (*)(GLenum, GLuint);
using BufferDataProc = void (*)(GLenum, std::ptrdiff_t, const void*, GLenum);
using DeleteBuffersProc = void (*)(GLsizei, const GLuint*);
GenBuffersProc pglGenBuffers = nullptr;
BindBufferProc pglBindBuffer = nullptr;
BufferDataProc pglBufferData = nullptr;
DeleteBuffersProc pglDeleteBuffers = nullptr;

bool loadBufferFunctions() {
    pglGenBuffers = reinterpret_cast<GenBuffersProc>(glfwGetProcAddress("glGenBuffers"));
    pglBindBuffer = reinterpret_cast<BindBufferProc>(glfwGetProcAddress("glBindBuffer"));
    pglBufferData = reinterpret_cast<BufferDataProc>(glfwGetProcAddress("glBufferData"));
    pglDeleteBuffers = reinterpret_cast<DeleteBuffersProc>(glfwGetProcAddress("glDeleteBuffers"));
    const bool available = pglGenBuffers && pglBindBuffer && pglBufferData && pglDeleteBuffers;
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
} // namespace

struct BusModel {
    struct Image {
        int width = 0;
        int height = 0;
        std::vector<std::uint8_t> rgba;
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
        std::array<double, 3> color = {0.65, 0.65, 0.65};
    };

    struct MaterialState {
        std::filesystem::path texturePath;
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
        std::array<double, 3> color = {0.65, 0.65, 0.65};
        int viewpoint = 0;
        std::string meshIdentifier;
        std::string animationParent;
        std::unordered_map<std::string, MaterialState> materialStates;
        int lodIndex = -1;
        WheelAnimation wheelAnimation;
    };

    struct Vertex {
        float x, y, z, u, v;
    };

    struct Batch {
        GLuint buffer = 0;
        GLuint texture = 0;
        std::array<double, 3> color = {0.65, 0.65, 0.65};
        bool textured = false;
        int alphaMode = 0;
        bool noZwrite = false;
        std::size_t vertexCount = 0;
    };

    struct DisplayPart {
        std::vector<Batch> batches;
        int viewpoint;
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
    std::vector<GLuint> textures;
    std::unordered_map<std::string, GLuint> textureCache;
    std::unordered_set<std::string> loggedTextureIssues;
    mutable std::unordered_set<std::size_t> loggedWheelBindings;
    double modelOffsetZ = MAN_DL05_MODEL_OFFSET_Z;
    double textureScale = 1;
    bool frustumCulling = false;
    std::vector<double> lodThresholds;
    std::size_t opaqueDisplayCount = 0;
    mutable std::size_t lastRenderedTriangles = 0;
    bool loaded = false;

#ifdef _WIN32
    bool comInitialized = false;
#endif

    explicit BusModel(BusVehicle vehicle) {
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

    void draw(bool outsideView) const {
        if (!loaded) {
            return;
        }

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
        const auto visible = [&](const DisplayPart& part) {
            const double local[4] = {part.center[0], part.center[1], part.center[2] + modelOffsetZ,
                                     1.0};
            double eye[4] = {};
            double clip[4] = {};
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
            for (int row = 0; row < 4; ++row) {
                for (int column = 0; column < 4; ++column) {
                    clip[row] += projection[row + column * 4] * eye[column];
                }
            }
            if (clip[3] + part.radius <= 0.0) {
                return false;
            }
            if (part.radius > 8.0) {
                return true;
            }
            const double margin =
                part.radius * 2.5 * std::max(std::abs(projection[0]), std::abs(projection[5]));
            return clip[0] >= -clip[3] - margin && clip[0] <= clip[3] + margin &&
                   clip[1] >= -clip[3] - margin && clip[1] <= clip[3] + margin &&
                   clip[2] >= -clip[3] - margin && clip[2] <= clip[3] + margin;
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
        std::vector<const DisplayPart*> opaqueParts;
        std::vector<const DisplayPart*> transparentParts;
        for (const DisplayPart& part : displayLists) {
            const bool visibleOutside = part.viewpoint == 0 || (part.viewpoint & 1) != 0;
            const bool visibleInside = part.viewpoint == 0 || (part.viewpoint & 2) != 0;
            if ((outsideView && !visibleOutside) || (!outsideView && !visibleInside) ||
                !visible(part)) {
                continue;
            }
            const bool hasTransparentBatch =
                std::any_of(part.batches.begin(), part.batches.end(), [](const Batch& batch) {
                    return batch.alphaMode != 0 || batch.noZwrite;
                });
            const bool hasOpaqueBatch =
                std::any_of(part.batches.begin(), part.batches.end(), [](const Batch& batch) {
                    return batch.alphaMode == 0 && !batch.noZwrite;
                });
            if (hasTransparentBatch) {
                transparentParts.push_back(&part);
            }
            if (hasOpaqueBatch) {
                opaqueParts.push_back(&part);
            }
        }
        std::sort(opaqueParts.begin(), opaqueParts.end(),
                  [&](const DisplayPart* first, const DisplayPart* second) {
                      return viewDepth(*first) < viewDepth(*second);
                  });
        std::sort(transparentParts.begin(), transparentParts.end(),
                  [&](const DisplayPart* first, const DisplayPart* second) {
                      return viewDepth(*first) > viewDepth(*second);
                  });
        lastRenderedTriangles = 0;
        for (const DisplayPart* part : opaqueParts) {
            lastRenderedTriangles += part->triangleCount;
        }
        for (const DisplayPart* part : transparentParts) {
            lastRenderedTriangles += part->triangleCount;
        }
        glPushMatrix();
        glTranslated(0.0, 0.0, modelOffsetZ);
        for (const DisplayPart* part : opaqueParts) {
            for (const Batch& batch : part->batches) {
                if (batch.alphaMode != 0 || batch.noZwrite) {
                    continue;
                }
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
                glVertexPointer(3, GL_FLOAT, sizeof(Vertex), reinterpret_cast<const void*>(0));
                if (batch.textured) {
                    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
                    glTexCoordPointer(2, GL_FLOAT, sizeof(Vertex),
                                      reinterpret_cast<const void*>(3 * sizeof(float)));
                }
                glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(batch.vertexCount));
                glDisableClientState(GL_TEXTURE_COORD_ARRAY);
                glDisableClientState(GL_VERTEX_ARRAY);
            }
        }
        glDepthMask(GL_FALSE);
        for (const DisplayPart* part : transparentParts) {
            for (const Batch& batch : part->batches) {
                if (batch.alphaMode == 0 && !batch.noZwrite) {
                    continue;
                }
                if (batch.alphaMode == 1) {
                    glDisable(GL_BLEND);
                    glEnable(GL_ALPHA_TEST);
                    glAlphaFunc(GL_GREATER, 0.5f);
                } else {
                    glEnable(GL_BLEND);
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    glDisable(GL_ALPHA_TEST);
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
                }
                glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(batch.vertexCount));
                glDisableClientState(GL_TEXTURE_COORD_ARRAY);
                glDisableClientState(GL_VERTEX_ARRAY);
            }
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

    bool hasConfiguredWheels() const {
        return !wheelModels.empty();
    }

    void drawConfiguredWheels(const BusSimulation& simulation, const BodyPose& chassis,
                              bool outsideView) const {
        std::vector<bool> usedWheelIndices(simulation.wheelCount(), false);
        for (std::size_t modelIndex = 0; modelIndex < wheelModels.size(); ++modelIndex) {
            const WheelModel& wheel = wheelModels[modelIndex];
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
            for (const DisplayPart& part : wheel.parts) {
                const bool visibleOutside = part.viewpoint == 0 || (part.viewpoint & 1) != 0;
                const bool visibleInside = part.viewpoint == 0 || (part.viewpoint & 2) != 0;
                if ((outsideView && !visibleOutside) || (!outsideView && !visibleInside)) {
                    continue;
                }
                glPushMatrix();
                applyPose(pose);
                glRotated(90.0, 1.0, 0.0, 0.0);
                glTranslated(-part.center[0], -part.center[1], -part.center[2]);
                for (const Batch& batch : part.batches) {
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

    GLuint loadTexture(const std::filesystem::path& path) {
        gameLog.Log("Loading texture from path: " + path.generic_string());
        const std::string cacheKey = path.generic_string();
        const auto cached = textureCache.find(cacheKey);
        if (cached != textureCache.end()) {
            return cached->second;
        }
        Image image;
        if (!readImage(path, image)) {
            textureLog.Log("decode-failed: " + path.generic_string());
            return 0;
        }
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
        textureCache.emplace(cacheKey, texture);
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
        gameLog.Log("Finding texture with name: " + name + " in root: " + root.generic_string());
        const std::string cleaned = trim(name);
        if (cleaned.empty()) {
            return {};
        }
        std::filesystem::path direct = root / cleaned;
        if (std::filesystem::exists(direct)) {
            return direct;
        }
        const std::string requestedStem = lower(std::filesystem::path(cleaned).stem().string());
        const std::array<std::string, 6> imageExtensions = {".tga", ".bmp", ".png",
                                                            ".dds", ".jpg", ".jpeg"};
        if (std::filesystem::exists(root)) {
            for (const auto& entry : std::filesystem::directory_iterator(
                     root, std::filesystem::directory_options::skip_permission_denied)) {
                if (entry.is_regular_file() &&
                    lower(entry.path().stem().string()) == requestedStem &&
                    std::find(imageExtensions.begin(), imageExtensions.end(),
                              lower(entry.path().extension().string())) != imageExtensions.end()) {
                    gameLog.Log("Found texture directly at path: " + entry.path().generic_string());
                    return entry.path();
                }
            }
        }
        const std::string basename = std::filesystem::path(cleaned).filename().string();
        for (std::filesystem::path ancestor = root; !ancestor.empty();
             ancestor = ancestor.parent_path()) {
            const std::filesystem::path textureRoot = ancestor / "Texture";
            if (std::filesystem::exists(textureRoot)) {
                direct = textureRoot / cleaned;
                if (std::filesystem::exists(direct)) {
                    gameLog.Log("Found texture at direct path: " + direct.generic_string());
                    return direct;
                }
                for (const auto& entry : std::filesystem::recursive_directory_iterator(
                         textureRoot, std::filesystem::directory_options::skip_permission_denied)) {
                    if (entry.is_regular_file() &&
                        lower(entry.path().filename().string()) == lower(basename)) {
                        gameLog.Log("Found texture at recursive path: " +
                                    entry.path().generic_string());
                        return entry.path();
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
                        return entry.path();
                    }
                }
            }
            const std::filesystem::path parent = ancestor.parent_path();
            if (parent == ancestor) {
                break;
            }
        }
        textureLog.Log("missing: " + cleaned);
        return {};
    }

    void loadObj(const Part& part) {
        gameLog.Log("Loading OBJ model from path: " + part.objPath.generic_string());
        std::ifstream input(part.objPath);
        if (!input) {
            gameLog.Log("Failed to open OBJ model at path: " + part.objPath.generic_string());
            return;
        }
        std::vector<ObjPosition> positions;
        std::vector<ObjTexCoord> texCoords;
        std::vector<ObjTriangle> triangles;
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
                positions.push_back(position);
            } else if (type == "vt") {
                ObjTexCoord texCoord = {};
                stream >> texCoord.u >> texCoord.v;
                texCoords.push_back(texCoord);
            } else if (type == "mtllib") {
                std::string materialPath;
                stream >> materialPath;
                std::replace(materialPath.begin(), materialPath.end(), '\\', '/');
                materialLibrary = part.objPath.parent_path() / materialPath;
            } else if (type == "usemtl") {
                stream >> currentMaterial;
            } else if (type == "f") {
                const std::size_t typeEnd = line.find(' ');
                if (typeEnd == std::string::npos) {
                    continue;
                }
                const std::vector<ObjIndex> face =
                    parseFace(line.substr(typeEnd + 1), static_cast<int>(positions.size()),
                              static_cast<int>(texCoords.size()));
                for (std::size_t index = 2; index < face.size(); ++index) {
                    triangles.push_back(
                        {{{face[0], face[index - 1], face[index]}}, currentMaterial});
                }
            }
        }
        if (triangles.empty()) {
            return;
        }
        std::array<double, 3> boundsMin = {std::numeric_limits<double>::max(),
                                           std::numeric_limits<double>::max(),
                                           std::numeric_limits<double>::max()};
        std::array<double, 3> boundsMax = {std::numeric_limits<double>::lowest(),
                                           std::numeric_limits<double>::lowest(),
                                           std::numeric_limits<double>::lowest()};
        for (const ObjPosition& position : positions) {
            const std::array<double, 3> converted = {position.z, -position.x, position.y};
            for (int axis = 0; axis < 3; ++axis) {
                boundsMin[axis] = std::min(boundsMin[axis], converted[axis]);
                boundsMax[axis] = std::max(boundsMax[axis], converted[axis]);
            }
        }
        const std::array<double, 3> boundsCenter = {(boundsMin[0] + boundsMax[0]) * 0.5,
                                                    (boundsMin[1] + boundsMax[1]) * 0.5,
                                                    (boundsMin[2] + boundsMax[2]) * 0.5};
        double boundsRadius = 0.0;
        for (const ObjPosition& position : positions) {
            const double x = position.z - boundsCenter[0];
            const double y = -position.x - boundsCenter[1];
            const double z = position.y - boundsCenter[2];
            boundsRadius = std::max(boundsRadius, std::sqrt(x * x + y * y + z * z));
        }
        std::unordered_map<std::string, ObjMaterial> materials;
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
                    materials[materialName] = {};
                } else if (materialType == "map_Kd" && !materialName.empty()) {
                    std::string textureName;
                    std::getline(materialStream, textureName);
                    materials[materialName].texturePath =
                        findTexture(part.objPath.parent_path(), trim(textureName));
                } else if (materialType == "Kd" && !materialName.empty()) {
                    materialStream >> materials[materialName].color[0] >>
                        materials[materialName].color[1] >> materials[materialName].color[2];
                }
            }
        }

        WheelAnimation wheelAnimation = part.wheelAnimation;
        if (!wheelAnimation.rotationVariable.empty() && !wheelAnimation.hasOrigin) {
            wheelAnimation.origin = boundsCenter;
            wheelAnimation.hasOrigin = true;
        }
        std::vector<DisplayPart>* destination = &displayLists;
        if (!wheelAnimation.rotationVariable.empty()) {
            auto wheel = std::find_if(
                wheelModels.begin(), wheelModels.end(), [&](const WheelModel& candidate) {
                    if (wheelAnimation.rotationVariable == candidate.animation.rotationVariable &&
                        wheelAnimation.suspensionVariable ==
                            candidate.animation.suspensionVariable &&
                        wheelAnimation.steeringVariable == candidate.animation.steeringVariable) {
                        return true;
                    }
                    if (wheelAnimation.hasOrigin && candidate.animation.hasOrigin) {
                        const double dx = wheelAnimation.origin[0] - candidate.animation.origin[0];
                        const double dy = wheelAnimation.origin[1] - candidate.animation.origin[1];
                        const double dz = wheelAnimation.origin[2] - candidate.animation.origin[2];
                        return dx * dx + dy * dy + dz * dz < 0.0001;
                    }
                    return wheelAnimation.rotationVariable ==
                               candidate.animation.rotationVariable &&
                           wheelAnimation.suspensionVariable ==
                               candidate.animation.suspensionVariable &&
                           wheelAnimation.steeringVariable == candidate.animation.steeringVariable;
                });
            if (wheel == wheelModels.end()) {
                wheelModels.push_back({wheelAnimation, {}});
                wheel = wheelModels.end() - 1;
            }
            destination = &wheel->parts;
        }
        auto makeBatch = [&](const std::vector<const ObjTriangle*>& source,
                             const std::filesystem::path& texturePath,
                             const std::array<double, 3>& color, int alphaMode, bool noZwrite) {
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
            batch.texture = texturePath.empty() ? 0 : loadTexture(texturePath);
            batch.textured = batch.texture != 0;
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
            destination->back().batches.push_back(batch);
        };

        std::unordered_map<std::string, std::vector<const ObjTriangle*>> groups;
        std::unordered_map<std::string, std::filesystem::path> groupTextures;
        std::unordered_map<std::string, std::array<double, 3>> groupColors;
        std::unordered_map<std::string, MaterialState> groupStates;
        std::size_t renderedTriangleCount = 0;
        bool hasTransparentMaterial = false;
        for (const auto& triangle : triangles) {
            const std::string materialName = lower(triangle.material);
            ++renderedTriangleCount;
            const auto material = materials.find(triangle.material);
            std::filesystem::path texturePath =
                material != materials.end() && !material->second.texturePath.empty()
                    ? material->second.texturePath
                    : part.texturePath;
            const std::array<double, 3> color =
                material != materials.end() ? material->second.color : part.color;
            const std::string key = triangle.material.empty() ? "__fallback__" : triangle.material;
            MaterialState state;
            if (!texturePath.empty()) {
                const auto found = part.materialStates.find(lower(texturePath.filename().string()));
                if (found != part.materialStates.end())
                    state = found->second;
            }
            groups[key].push_back(&triangle);
            groupTextures[key] = texturePath;
            groupColors[key] = color;
            groupStates[key] = state;
            hasTransparentMaterial =
                hasTransparentMaterial || state.alphaMode != 0 || state.noZwrite;
        }
        destination->push_back({{},
                                part.viewpoint,
                                hasTransparentMaterial,
                                part.lodIndex,
                                boundsCenter,
                                boundsRadius,
                                renderedTriangleCount});
        for (const auto& group : groups) {
            const MaterialState& state = groupStates[group.first];
            makeBatch(group.second, groupTextures[group.first], groupColors[group.first],
                      state.alphaMode, state.noZwrite);
        }
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
                    const std::filesystem::path texturePath = findTexture(modelRoot, textureName);
                    lastMaterialKey = lower(std::filesystem::path(textureName).filename().string());
                    parts.back().materialStates[lastMaterialKey] = {texturePath, 0, false};
                    if (parts.back().texturePath.empty()) {
                        parts.back().texturePath = texturePath;
                    }
                }
            } else if (value == "[matl_alpha]" && !parts.empty()) {
                std::string alphaMode;
                if (std::getline(input, alphaMode) && !lastMaterialKey.empty()) {
                    parts.back().materialStates[lastMaterialKey].alphaMode = parseInt(alphaMode, 0);
                }
            } else if (value == "[matl_noZwrite]" && !parts.empty() && !lastMaterialKey.empty()) {
                parts.back().materialStates[lastMaterialKey].noZwrite = true;
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
        for (const Part& part : parts) {
            if (part.objPath.empty()) {
                continue;
            }
            loadObj(part);
        }
        std::stable_sort(displayLists.begin(), displayLists.end(),
                         [](const DisplayPart& first, const DisplayPart& second) {
                             return first.transparent < second.transparent;
                         });
        opaqueDisplayCount = 0;
        while (opaqueDisplayCount < displayLists.size() &&
               !displayLists[opaqueDisplayCount].transparent) {
            ++opaqueDisplayCount;
        }
        loaded = !displayLists.empty();
    }
};

void Renderer::scrollCallback(GLFWwindow* window, double, double yOffset) {
    auto* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
    if (renderer == nullptr) {
        return;
    }
    renderer->cameraDistance_ = std::clamp(renderer->cameraDistance_ - yOffset * 2.0, 6.0, 80.0);
}

Renderer::Renderer(int width, int height, const char* title, BusVehicle vehicle)
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
    glfwSwapInterval(0);
    if (!loadBufferFunctions()) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
        glfwTerminate();
        throw std::runtime_error("OpenGL VBO functions are unavailable");
    }
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.45f, 0.65f, 0.88f, 1.0f);
    busModel_ = std::make_unique<BusModel>(vehicle);
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

    if (busModel_ && busModel_->loaded) {
        glPushMatrix();
        applyPose(chassis);
        busModel_->draw(cameraView_ == 0);
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

    const std::array<double, 3> centerOfGravity = simulation.centerOfGravity();
    glPushMatrix();
    glTranslated(centerOfGravity[0], centerOfGravity[1], centerOfGravity[2]);
    drawCenterOfGravityMarker(0.35);
    glPopMatrix();

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

    if (busModel_ && busModel_->hasConfiguredWheels()) {
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
