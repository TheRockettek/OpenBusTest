#pragma once

#include "render/MatrixMath.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <vector>

namespace openbus::app::vehicle {

struct TransparentTriangleDepth {
    std::size_t index = 0;
    double depth = 0.0;
};

inline std::array<double, 3>
transparentTriangleDepthAxis(const openbus::rendering::Matrix4& modelView) {
    return {modelView[2], modelView[6], modelView[10]};
}

template <typename VertexContainer>
void transparentTriangleOrder(const VertexContainer& vertices,
                              const openbus::rendering::Matrix4& modelView,
                              std::vector<TransparentTriangleDepth>& triangles,
                              std::vector<std::size_t>& order) {
    order.clear();
    if (vertices.size() < 6 || vertices.size() % 3 != 0) {
        return;
    }
    triangles.clear();
    triangles.reserve(vertices.size() / 3);
    for (std::size_t index = 0; index < vertices.size(); index += 3) {
        const auto& first = vertices[index];
        const auto& second = vertices[index + 1];
        const auto& third = vertices[index + 2];
        const std::array<double, 4> center = {
            (static_cast<double>(first.x) + second.x + third.x) / 3.0,
            (static_cast<double>(first.y) + second.y + third.y) / 3.0,
            (static_cast<double>(first.z) + second.z + third.z) / 3.0, 1.0};
        const auto viewCenter = openbus::rendering::transformPoint(modelView, center);
        triangles.push_back({index, -viewCenter[2]});
    }
    std::stable_sort(
        triangles.begin(), triangles.end(),
        [](const TransparentTriangleDepth& first, const TransparentTriangleDepth& second) {
            return first.depth > second.depth;
        });
    order.reserve(triangles.size());
    for (const TransparentTriangleDepth& triangle : triangles) {
        order.push_back(triangle.index);
    }
}

template <typename TransparentBatch>
void sortTransparentBatches(std::vector<TransparentBatch>& batches) {
    std::stable_sort(batches.begin(), batches.end(),
                     [](const TransparentBatch& first, const TransparentBatch& second) {
                         if (first.depth != second.depth) {
                             return first.depth > second.depth;
                         }
                         if (first.renderType != second.renderType) {
                             return first.renderType < second.renderType;
                         }
                         return false;
                     });
}

template <typename VertexContainer>
std::vector<std::size_t> transparentTriangleOrder(const VertexContainer& vertices,
                                                  const openbus::rendering::Matrix4& modelView) {
    std::vector<TransparentTriangleDepth> triangles;
    std::vector<std::size_t> order;
    transparentTriangleOrder(vertices, modelView, triangles, order);
    return order;
}

} // namespace openbus::app::vehicle
