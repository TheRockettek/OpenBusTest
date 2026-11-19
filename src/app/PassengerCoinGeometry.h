#pragma once

#include "render/PrimitiveVertex.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

namespace openbus::app {

inline void appendPassengerCoinDisc(std::vector<openbus::rendering::PrimitiveVertex>& vertices,
                                    double radius, double thickness,
                                    const std::array<float, 3>& color) {
    constexpr std::size_t segmentCount = 12;
    const auto vertex = [&color](double x, double y, double z) {
        return openbus::rendering::PrimitiveVertex{static_cast<float>(x),
                                                   static_cast<float>(y),
                                                   static_cast<float>(z),
                                                   color[0],
                                                   color[1],
                                                   color[2]};
    };
    const std::array<float, 3> sideColor = {color[0] * 0.68F, color[1] * 0.68F, color[2] * 0.68F};
    const auto sideVertex = [&sideColor](double x, double y, double z) {
        return openbus::rendering::PrimitiveVertex{static_cast<float>(x), static_cast<float>(y),
                                                   static_cast<float>(z), sideColor[0],
                                                   sideColor[1],          sideColor[2]};
    };
    const double halfThickness = thickness * 0.5;
    for (std::size_t segment = 0; segment < segmentCount; ++segment) {
        const double firstAngle = 2.0 * std::numbers::pi * static_cast<double>(segment) /
                                  static_cast<double>(segmentCount);
        const double secondAngle = 2.0 * std::numbers::pi * static_cast<double>(segment + 1) /
                                   static_cast<double>(segmentCount);
        const double firstX = radius * std::cos(firstAngle);
        const double firstY = radius * std::sin(firstAngle);
        const double secondX = radius * std::cos(secondAngle);
        const double secondY = radius * std::sin(secondAngle);
        vertices.push_back(vertex(0.0, 0.0, halfThickness));
        vertices.push_back(vertex(firstX, firstY, halfThickness));
        vertices.push_back(vertex(secondX, secondY, halfThickness));
        vertices.push_back(vertex(0.0, 0.0, -halfThickness));
        vertices.push_back(vertex(secondX, secondY, -halfThickness));
        vertices.push_back(vertex(firstX, firstY, -halfThickness));
        vertices.push_back(sideVertex(firstX, firstY, -halfThickness));
        vertices.push_back(sideVertex(secondX, secondY, -halfThickness));
        vertices.push_back(sideVertex(secondX, secondY, halfThickness));
        vertices.push_back(sideVertex(firstX, firstY, -halfThickness));
        vertices.push_back(sideVertex(secondX, secondY, halfThickness));
        vertices.push_back(sideVertex(firstX, firstY, halfThickness));
    }
}

inline void appendPassengerTicketCard(std::vector<openbus::rendering::PrimitiveVertex>& vertices,
                                      const std::array<double, 3>& position, double headingRadians,
                                      bool ticketVisible, bool actorSubmitted) {
    if (!ticketVisible || !actorSubmitted) {
        return;
    }

    constexpr double halfWidth = 0.027;
    constexpr double halfHeight = 0.018;
    const double sideX = -std::sin(headingRadians);
    const double sideY = std::cos(headingRadians);
    const double centerX = position[0] + sideX * 0.18;
    const double centerY = position[1] + sideY * 0.18;
    const double centerZ = position[2] + 1.02;
    const auto vertex = [&](double sideOffset, double heightOffset,
                            const std::array<float, 3>& color) {
        return openbus::rendering::PrimitiveVertex{static_cast<float>(centerX + sideX * sideOffset),
                                                   static_cast<float>(centerY + sideY * sideOffset),
                                                   static_cast<float>(centerZ + heightOffset),
                                                   color[0],
                                                   color[1],
                                                   color[2]};
    };
    const std::array<float, 3> paper = {0.92F, 0.88F, 0.72F};
    const std::array<float, 3> ink = {0.20F, 0.36F, 0.48F};
    const auto bottomLeft = vertex(-halfWidth, -halfHeight, paper);
    const auto bottomRight = vertex(halfWidth, -halfHeight, paper);
    const auto topRight = vertex(halfWidth, halfHeight, paper);
    const auto topLeft = vertex(-halfWidth, halfHeight, paper);
    vertices.insert(vertices.end(),
                    {bottomLeft, bottomRight, topRight, bottomLeft, topRight, topLeft});

    const double stripeBottom = halfHeight * 0.45;
    const auto stripeLeftBottom = vertex(-halfWidth * 0.72, stripeBottom, ink);
    const auto stripeRightBottom = vertex(halfWidth * 0.72, stripeBottom, ink);
    const auto stripeRightTop = vertex(halfWidth * 0.72, halfHeight * 0.72, ink);
    const auto stripeLeftTop = vertex(-halfWidth * 0.72, halfHeight * 0.72, ink);
    vertices.insert(vertices.end(), {stripeLeftBottom, stripeRightBottom, stripeRightTop,
                                     stripeLeftBottom, stripeRightTop, stripeLeftTop});
}

} // namespace openbus::app
