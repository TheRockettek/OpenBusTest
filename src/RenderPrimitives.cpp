#include "RenderPrimitives.h"

#include "CameraMath.h"

#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <array>
#include <cmath>

namespace openbus::rendering {
namespace {

constexpr int ROAD_BUMP_SEGMENTS = 12;
constexpr int ROAD_RAMP_SEGMENTS = 6;
constexpr int ROAD_INCLINE_SEGMENTS = 16;

} // namespace

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
    static GLuint groundGridList = 0;
    if (groundGridList == 0) {
        groundGridList = glGenLists(1);
        glNewList(groundGridList, GL_COMPILE);
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
            glColor3d(major ? 0.38 : 0.25, major ? 0.43 : 0.30, major ? 0.36 : 0.25);
            glVertex3d(coordinate, -100.0, 0.02);
            glVertex3d(coordinate, 100.0, 0.02);
            glVertex3d(-100.0, coordinate, 0.02);
            glVertex3d(100.0, coordinate, 0.02);
        }
        glEnd();
        glEndList();
    }
    glCallList(groundGridList);

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

void drawWheel(double radius, double halfWidth, double red, double green, double blue) {
    constexpr int segments = 16;
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
    glTranslated(collision.offsetX, collision.offsetY, collision.offsetZ);
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

} // namespace openbus::rendering
