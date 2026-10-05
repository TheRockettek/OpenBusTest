#include "RenderPrimitives.h"

#include "CameraMath.h"
#include "CoreRenderer.h"
#include "PerfTrace.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace openbus::rendering {
namespace {

constexpr int ROAD_BUMP_SEGMENTS = 12;
constexpr int ROAD_RAMP_SEGMENTS = 6;
constexpr int ROAD_INCLINE_SEGMENTS = 16;
StaticPrimitiveBuffer groundBufferCache;
StaticPrimitiveBuffer gridBufferCache;

PrimitiveVertex vertex(double x, double y, double z, const std::array<double, 3>& color) {
    return {static_cast<float>(x),        static_cast<float>(y),
            static_cast<float>(z),        static_cast<float>(color[0]),
            static_cast<float>(color[1]), static_cast<float>(color[2])};
}

void appendColoredBox(std::vector<PrimitiveVertex>& vertices, double length, double width,
                      double height, double centerX, double centerY, double bottomZ,
                      const std::array<double, 3>& topColor,
                      const std::array<double, 3>& sideColor) {
    const double halfLength = length * 0.5;
    const double halfWidth = width * 0.5;
    const std::array<std::array<double, 3>, 8> corners = {{{-halfLength, -halfWidth, 0.0},
                                                           {halfLength, -halfWidth, 0.0},
                                                           {halfLength, halfWidth, 0.0},
                                                           {-halfLength, halfWidth, 0.0},
                                                           {-halfLength, -halfWidth, height},
                                                           {halfLength, -halfWidth, height},
                                                           {halfLength, halfWidth, height},
                                                           {-halfLength, halfWidth, height}}};
    const std::array<std::array<int, 4>, 6> faces = {
        {{4, 5, 6, 7}, {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}, {0, 3, 2, 1}}};
    for (std::size_t faceIndex = 0; faceIndex < faces.size(); ++faceIndex) {
        const auto& color = faceIndex == 0 ? topColor : sideColor;
        const auto& face = faces[faceIndex];
        for (int index : {face[0], face[1], face[2], face[0], face[2], face[3]}) {
            const auto& point = corners[static_cast<std::size_t>(index)];
            vertices.push_back(
                vertex(point[0] + centerX, point[1] + centerY, point[2] + bottomZ, color));
        }
    }
}

void drawColoredBox(double length, double width, double height, double centerX, double centerY,
                    double bottomZ, const std::array<double, 3>& topColor,
                    const std::array<double, 3>& sideColor) {
    std::vector<PrimitiveVertex> vertices;
    vertices.reserve(36);
    appendColoredBox(vertices, length, width, height, centerX, centerY, bottomZ, topColor,
                     sideColor);
    drawPrimitives(vertices, GL_TRIANGLES);
}

void drawLineList(const std::vector<PrimitiveVertex>& vertices, float lineWidth = 1.0f) {
    drawPrimitives(vertices, GL_LINES, lineWidth);
}

} // namespace

void drawBox(double length, double width, double height, double red, double green, double blue) {
    const double halfLength = length * 0.5;
    const double halfWidth = width * 0.5;
    const double halfHeight = height * 0.5;
    const std::array<double, 3> color = {red, green, blue};
    const std::array<std::array<double, 3>, 8> corners = {{{-halfLength, -halfWidth, -halfHeight},
                                                           {halfLength, -halfWidth, -halfHeight},
                                                           {halfLength, halfWidth, -halfHeight},
                                                           {-halfLength, halfWidth, -halfHeight},
                                                           {-halfLength, -halfWidth, halfHeight},
                                                           {halfLength, -halfWidth, halfHeight},
                                                           {halfLength, halfWidth, halfHeight},
                                                           {-halfLength, halfWidth, halfHeight}}};
    const std::array<std::array<int, 2>, 12> edges = {{{0, 1},
                                                       {1, 2},
                                                       {2, 3},
                                                       {3, 0},
                                                       {4, 5},
                                                       {5, 6},
                                                       {6, 7},
                                                       {7, 4},
                                                       {0, 4},
                                                       {1, 5},
                                                       {2, 6},
                                                       {3, 7}}};
    std::vector<PrimitiveVertex> vertices;
    vertices.reserve(edges.size() * 2);
    for (const auto& edge : edges) {
        for (int index : edge) {
            const auto& point = corners[static_cast<std::size_t>(index)];
            vertices.push_back(vertex(point[0], point[1], point[2], color));
        }
    }
    drawLineList(vertices);
}

void drawRoadBox(double centerX, double centerY, double length, double width, double height,
                 double bottomZ, const std::array<double, 3>& topColor,
                 const std::array<double, 3>& sideColor) {
    drawColoredBox(length, width, height, centerX, centerY, bottomZ, topColor, sideColor);
}

void appendRoadIncline(std::vector<PrimitiveVertex>& surfaces, std::vector<PrimitiveVertex>& lines,
                       const RoadBump& bump) {
    const double halfWidth = bump.width * 0.5;
    const double startX = bump.centerX - bump.length * 0.5;
    const double segmentLength = bump.length / ROAD_INCLINE_SEGMENTS;
    const auto heightAt = [&](double x) { return roadFeatureHeightAt(bump, x - bump.centerX); };
    const std::array<double, 3> topColor = {0.42, 0.50, 0.30};
    const std::array<double, 3> sideColor = {0.18, 0.25, 0.12};
    const std::array<double, 3> railColor = {0.0, 0.85, 0.95};
    for (int segment = 0; segment < ROAD_INCLINE_SEGMENTS; ++segment) {
        const double minX = startX + segment * segmentLength;
        const double maxX = minX + segmentLength;
        const double minHeight = heightAt(minX);
        const double maxHeight = heightAt(maxX);
        const auto addQuad = [&](const std::array<double, 3>& color, double x0, double y0,
                                 double z0, double x1, double y1, double z1, double x2, double y2,
                                 double z2, double x3, double y3, double z3) {
            surfaces.push_back(vertex(x0, y0, z0, color));
            surfaces.push_back(vertex(x1, y1, z1, color));
            surfaces.push_back(vertex(x2, y2, z2, color));
            surfaces.push_back(vertex(x0, y0, z0, color));
            surfaces.push_back(vertex(x2, y2, z2, color));
            surfaces.push_back(vertex(x3, y3, z3, color));
        };
        addQuad(topColor, minX, bump.centerY - halfWidth, minHeight, maxX, bump.centerY - halfWidth,
                maxHeight, maxX, bump.centerY + halfWidth, maxHeight, minX,
                bump.centerY + halfWidth, minHeight);
        addQuad(sideColor, minX, bump.centerY - halfWidth, 0.0, maxX, bump.centerY - halfWidth, 0.0,
                maxX, bump.centerY - halfWidth, maxHeight, minX, bump.centerY - halfWidth,
                minHeight);
        addQuad(sideColor, maxX, bump.centerY + halfWidth, 0.0, minX, bump.centerY + halfWidth, 0.0,
                minX, bump.centerY + halfWidth, minHeight, maxX, bump.centerY + halfWidth,
                maxHeight);
        lines.push_back(vertex(minX, bump.centerY - halfWidth, minHeight + 0.005, railColor));
        lines.push_back(vertex(maxX, bump.centerY - halfWidth, maxHeight + 0.005, railColor));
        lines.push_back(vertex(minX, bump.centerY + halfWidth, minHeight + 0.005, railColor));
        lines.push_back(vertex(maxX, bump.centerY + halfWidth, maxHeight + 0.005, railColor));
    }
    const double endX = startX + bump.length;
    const double endHeight = heightAt(endX);
    lines.push_back(vertex(endX, bump.centerY - halfWidth, 0.0, sideColor));
    lines.push_back(vertex(endX, bump.centerY + halfWidth, 0.0, sideColor));
    lines.push_back(vertex(endX, bump.centerY + halfWidth, endHeight, sideColor));
    lines.push_back(vertex(endX, bump.centerY - halfWidth, endHeight, sideColor));
}

void drawRoadIncline(const RoadBump& bump) {
    std::vector<PrimitiveVertex> surfaces;
    std::vector<PrimitiveVertex> lines;
    appendRoadIncline(surfaces, lines, bump);
    drawPrimitives(surfaces, GL_TRIANGLES);
    drawLineList(lines);
}

void drawGround(const std::vector<RoadBump>& bumps) {
    TraceScope trace("render", "RenderPrimitives::drawGround");
    static std::vector<PrimitiveVertex> ground;
    static std::vector<PrimitiveVertex> gridLines;
    if (ground.empty()) {
        const std::array<double, 3> groundColor = {0.18, 0.22, 0.18};
        const std::array<double, 3> minorColor = {0.25, 0.30, 0.25};
        const std::array<double, 3> majorColor = {0.38, 0.43, 0.36};
        ground = {
            vertex(-100.0, -100.0, 0.0, groundColor), vertex(100.0, -100.0, 0.0, groundColor),
            vertex(100.0, 100.0, 0.0, groundColor),   vertex(-100.0, -100.0, 0.0, groundColor),
            vertex(100.0, 100.0, 0.0, groundColor),   vertex(-100.0, 100.0, 0.0, groundColor)};
        for (int line = -100; line <= 100; line += 2) {
            const double coordinate = static_cast<double>(line);
            const auto& color = line % 10 == 0 ? majorColor : minorColor;
            gridLines.push_back(vertex(coordinate, -100.0, 0.02, color));
            gridLines.push_back(vertex(coordinate, 100.0, 0.02, color));
            gridLines.push_back(vertex(-100.0, coordinate, 0.02, color));
            gridLines.push_back(vertex(100.0, coordinate, 0.02, color));
        }
    }
    drawStaticPrimitives(groundBufferCache, ground, GL_TRIANGLES);
    drawStaticPrimitives(gridBufferCache, gridLines, GL_LINES);

    std::vector<PrimitiveVertex> surfaces;
    std::vector<PrimitiveVertex> lines;
    surfaces.reserve(bumps.size() * 36);
    lines.reserve(bumps.size() * 8);
    const auto addBox = [&](double length, double width, double height, double centerX,
                            double centerY, double bottomZ, const std::array<double, 3>& topColor,
                            const std::array<double, 3>& sideColor) {
        appendColoredBox(surfaces, length, width, height, centerX, centerY, bottomZ, topColor,
                         sideColor);
    };

    for (const RoadBump& bump : bumps) {
        if (bump.type == RoadFeatureType::Barrier) {
            addBox(bump.length, bump.width, bump.height, bump.centerX, bump.centerY, 0.0,
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
                addBox(segmentLength, bump.width, bump.height * phase,
                       startX + segmentLength * (segment + 0.5), bump.centerY, 0.0, deckColor,
                       deckSideColor);
            }
            addBox(deckLength, bump.width, bump.height, bump.centerX, bump.centerY, 0.0, deckColor,
                   deckSideColor);
            const double downStartX = bump.centerX + bump.length * 0.5 - rampLength;
            for (int segment = 0; segment < ROAD_RAMP_SEGMENTS; ++segment) {
                const double phase = (static_cast<double>(segment) + 0.5) / ROAD_RAMP_SEGMENTS;
                addBox(segmentLength, bump.width, bump.height * (1.0 - phase),
                       downStartX + segmentLength * (segment + 0.5), bump.centerY, 0.0, deckColor,
                       deckSideColor);
            }
            if (bump.railHeight > 0.0 && bump.railWidth > 0.0) {
                const double railY = bump.width * 0.5 - bump.railWidth * 0.5;
                const std::array<double, 3> railColor = {0.72, 0.76, 0.80};
                const std::array<double, 3> railSideColor = {0.30, 0.34, 0.38};
                addBox(bump.length, bump.railWidth, bump.railHeight, bump.centerX,
                       bump.centerY + railY, bump.height, railColor, railSideColor);
                addBox(bump.length, bump.railWidth, bump.railHeight, bump.centerX,
                       bump.centerY - railY, bump.height, railColor, railSideColor);
            }
            continue;
        }
        if (bump.type == RoadFeatureType::Incline) {
            appendRoadIncline(surfaces, lines, bump);
            continue;
        }
        const double halfWidth = bump.width * 0.5;
        const double segmentLength = bump.length / ROAD_BUMP_SEGMENTS;
        for (int segment = 0; segment < ROAD_BUMP_SEGMENTS; ++segment) {
            const double minX = bump.centerX - bump.length * 0.5 + segment * segmentLength;
            const double maxX = minX + segmentLength;
            const double height = roadFeatureHeightAt(bump, (minX + maxX) * 0.5 - bump.centerX);
            addBox(maxX - minX, halfWidth * 2.0, height, (minX + maxX) * 0.5, bump.centerY, 0.0,
                   {0.72, 0.46, 0.18}, {0.48, 0.28, 0.10});
        }
    }
    drawPrimitives(surfaces, GL_TRIANGLES);
    drawLineList(lines);
}

} // namespace openbus::rendering
