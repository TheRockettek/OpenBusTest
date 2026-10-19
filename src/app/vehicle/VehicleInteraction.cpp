// Mouse picking of clickable parts and the clickable debug overlay.

#include "app/vehicle/Vehicle.h"

bool Vehicle::clickablePartVisible(const DisplayPart& part, std::size_t partIndex,
                                   RenderViewContext context) const {
    const bool visible =
        partIndex < variableVisibleParts.size()
            ? variableVisibleParts[partIndex]
            : part.visibleVariable.empty() ||
                  variables.get(part.visibleVariable) == static_cast<double>(part.visibleValue);
    if (!visible || !openbus::rendering::viewpointMatches(part.viewpoint, context)) {
        return false;
    }
    return activeLod < 0 || part.lodIndex < 0 || part.lodIndex == activeLod;
}

const Vehicle::DisplayPart* Vehicle::pickClickable(double cursorX, double cursorY,
                                                   int viewportWidth, int viewportHeight,
                                                   RenderViewContext context,
                                                   const Batch** hitBatch,
                                                   std::size_t* hitTriangle) const {
    TraceScope trace("input", "Vehicle::pickClickable");
    (void)context;
    if (hitBatch != nullptr) {
        *hitBatch = nullptr;
    }
    if (hitTriangle != nullptr) {
        *hitTriangle = 0;
    }
    if (viewportWidth <= 0 || viewportHeight <= 0) {
        return nullptr;
    }
    const auto& modelView = openbus::rendering::modelViewMatrix();
    const auto& projection = openbus::rendering::projectionMatrix();
    const Batch* selectedBatch = nullptr;
    std::size_t selectedTriangle = 0;
    const double normalizedX = cursorX / static_cast<double>(viewportWidth) * 2.0 - 1.0;
    const double normalizedY = 1.0 - cursorY / static_cast<double>(viewportHeight) * 2.0;
    struct ProjectedVertex {
        double x = 0.0;
        double y = 0.0;
        double depth = 0.0;
        bool valid = false;
    };
    const auto projectVertex = [&](const Matrix4& modelViewPart, const Vertex& vertex) {
        const std::array<double, 4> view =
            transformPoint(modelViewPart, {vertex.x, vertex.y, vertex.z, 1.0});
        const std::array<double, 4> clip = transformPoint(projection, view);
        if (clip[3] <= 1.0e-8) {
            return ProjectedVertex{};
        }
        return ProjectedVertex{clip[0] / clip[3], clip[1] / clip[3], -view[2], true};
    };
    const auto cross2d = [](double ax, double ay, double bx, double by) {
        return ax * by - ay * bx;
    };
    const auto triangleDepthAtCursor = [&](const Matrix4& modelViewPart, const DisplayPart& part,
                                           const Batch& batch, std::size_t index, double& depth) {
        const ProjectedVertex first = projectVertex(modelViewPart, batch.vertices[index]);
        const ProjectedVertex second = projectVertex(modelViewPart, batch.vertices[index + 1]);
        const ProjectedVertex third = projectVertex(modelViewPart, batch.vertices[index + 2]);
        if (!first.valid || !second.valid || !third.valid) {
            return false;
        }
        const double denominator =
            cross2d(second.x - first.x, second.y - first.y, third.x - first.x, third.y - first.y);
        if (std::abs(denominator) <= 1.0e-12 || (part.backFaceCulling && denominator >= 0.0)) {
            return false;
        }
        const double firstWeight = cross2d(second.x - normalizedX, second.y - normalizedY,
                                           third.x - normalizedX, third.y - normalizedY) /
                                   denominator;
        const double secondWeight = cross2d(third.x - normalizedX, third.y - normalizedY,
                                            first.x - normalizedX, first.y - normalizedY) /
                                    denominator;
        const double thirdWeight = 1.0 - firstWeight - secondWeight;
        constexpr double triangleTolerance = 1.0e-7;
        if (firstWeight < -triangleTolerance || secondWeight < -triangleTolerance ||
            thirdWeight < -triangleTolerance) {
            return false;
        }
        depth = firstWeight * first.depth + secondWeight * second.depth + thirdWeight * third.depth;
        return depth >= 0.0;
    };
    const auto cursorWithinProjectedBounds =
        [&](const Matrix4& modelViewPart, const DisplayPart& part,
            std::array<double, 4>& viewCenter, std::array<double, 4>& clipCenter) {
            const std::array<double, 3> center = {part.center[0], part.center[1], part.center[2]};
            viewCenter = transformPoint(modelViewPart, {center[0], center[1], center[2], 1.0});
            clipCenter = transformPoint(projection, viewCenter);
            double minimumNdcX = std::numeric_limits<double>::max();
            double maximumNdcX = std::numeric_limits<double>::lowest();
            double minimumNdcY = std::numeric_limits<double>::max();
            double maximumNdcY = std::numeric_limits<double>::lowest();
            const std::array<double, 3> halfSize = {std::max(part.size[0] * 0.5, 0.0),
                                                    std::max(part.size[1] * 0.5, 0.0),
                                                    std::max(part.size[2] * 0.5, 0.0)};
            bool hasProjectedCorner = false;
            for (int corner = 0; corner < 8; ++corner) {
                const std::array<double, 4> local = {
                    center[0] + ((corner & 1) == 0 ? -halfSize[0] : halfSize[0]),
                    center[1] + ((corner & 2) == 0 ? -halfSize[1] : halfSize[1]),
                    center[2] + ((corner & 4) == 0 ? -halfSize[2] : halfSize[2]), 1.0};
                const std::array<double, 4> view = transformPoint(modelViewPart, local);
                const std::array<double, 4> clip = transformPoint(projection, view);
                if (!std::isfinite(clip[0]) || !std::isfinite(clip[1]) || !std::isfinite(clip[3]) ||
                    clip[3] <= 1.0e-8) {
                    // Retain eye-plane and invalid cases for the exact path.
                    return true;
                }
                const double ndcX = clip[0] / clip[3];
                const double ndcY = clip[1] / clip[3];
                if (!std::isfinite(ndcX) || !std::isfinite(ndcY)) {
                    return true;
                }
                minimumNdcX = std::min(minimumNdcX, ndcX);
                maximumNdcX = std::max(maximumNdcX, ndcX);
                minimumNdcY = std::min(minimumNdcY, ndcY);
                maximumNdcY = std::max(maximumNdcY, ndcY);
                hasProjectedCorner = true;
            }
            if (!hasProjectedCorner) {
                return true;
            }
            return normalizedX >= minimumNdcX && normalizedX <= maximumNdcX &&
                   normalizedY >= minimumNdcY && normalizedY <= maximumNdcY;
        };
    double closestOccluderDepth = std::numeric_limits<double>::max();
    for (const std::size_t partIndex : visibleOpaquePickPartIndices) {
        const DisplayPart& part = displayLists[partIndex];
        const Matrix4 modelViewPart =
            multiplyMatrix4(modelView, multiplyMatrix4(translationMatrix({0.0, 0.0, modelOffsetZ}),
                                                       animationTransformForPart(part)));
        std::array<double, 4> viewCenter = {};
        std::array<double, 4> clipCenter = {};
        if (!cursorWithinProjectedBounds(modelViewPart, part, viewCenter, clipCenter)) {
            continue;
        }
        for (const Batch& batch : part.batches) {
            if (batch.alphaMode != 0 || batch.noZwrite || batch.noZcheck) {
                continue;
            }
            for (std::size_t index = 0; index + 2 < batch.vertices.size(); index += 3) {
                double triangleDepth = 0.0;
                if (triangleDepthAtCursor(modelViewPart, part, batch, index, triangleDepth)) {
                    closestOccluderDepth = std::min(closestOccluderDepth, triangleDepth);
                }
            }
        }
    }
    double closestDepth = std::numeric_limits<double>::max();
    const DisplayPart* selected = nullptr;
    for (const std::size_t partIndex : visibleClickablePartIndices) {
        const DisplayPart& part = displayLists[partIndex];
        const Matrix4 modelViewPart =
            multiplyMatrix4(modelView, multiplyMatrix4(translationMatrix({0.0, 0.0, modelOffsetZ}),
                                                       animationTransformForPart(part)));
        std::array<double, 4> viewCenter = {};
        std::array<double, 4> clipCenter = {};
        if (!cursorWithinProjectedBounds(modelViewPart, part, viewCenter, clipCenter)) {
            continue;
        }
        if (std::abs(clipCenter[3]) <= 1.0e-8) {
            continue;
        }
        double partDistance = std::numeric_limits<double>::max();
        const Batch* partBatch = nullptr;
        std::size_t partTriangle = 0;
        for (const Batch& batch : part.batches) {
            for (std::size_t index = 0; index + 2 < batch.vertices.size(); index += 3) {
                double triangleDistance = 0.0;
                if (!triangleDepthAtCursor(modelViewPart, part, batch, index, triangleDistance)) {
                    continue;
                }
                if (triangleDistance < partDistance) {
                    partDistance = triangleDistance;
                    partBatch = &batch;
                    partTriangle = index;
                }
            }
        }
        if (partDistance >= closestDepth) {
            continue;
        }
        if (partDistance == std::numeric_limits<double>::max()) {
            continue;
        }
        if (partDistance > closestOccluderDepth + 1.0e-4 * std::max(1.0, closestOccluderDepth)) {
            continue;
        }
        closestDepth = partDistance;
        selected = &part;
        selectedBatch = partBatch;
        selectedTriangle = partTriangle;
    }
    if (hitBatch != nullptr) {
        *hitBatch = selectedBatch;
    }
    if (hitTriangle != nullptr) {
        *hitTriangle = selectedTriangle;
    }
    return selected;
}

bool Vehicle::hasClickableAt(double cursorX, double cursorY, int viewportWidth, int viewportHeight,
                             RenderViewContext context) const {
    const Batch* selectedBatch = nullptr;
    std::size_t selectedTriangle = 0;
    const DisplayPart* selected = pickClickable(cursorX, cursorY, viewportWidth, viewportHeight,
                                                context, &selectedBatch, &selectedTriangle);
    hoveredClickablePartIndex = selected == nullptr
                                    ? std::numeric_limits<std::size_t>::max()
                                    : static_cast<std::size_t>(selected - displayLists.data());
    hoveredClickableBatchIndex = std::numeric_limits<std::size_t>::max();
    hoveredClickableTriangle = selectedTriangle;
    if (selected != nullptr && selectedBatch != nullptr) {
        hoveredClickableBatchIndex =
            static_cast<std::size_t>(selectedBatch - selected->batches.data());
    }
    return selected != nullptr;
}

bool Vehicle::hasVisibleClickable(RenderViewContext context) const {
    (void)context;
    return !visibleClickablePartIndices.empty();
}

std::uint64_t Vehicle::clickableRevision() const {
    return clickableStateRevision + displayLists.size();
}

std::string Vehicle::clickableDebugDescription(const DisplayPart& part, std::size_t partIndex,
                                               RenderViewContext context) const {
    std::string scriptEvent = lower(part.mouseEvent);
    for (char& character : scriptEvent) {
        const unsigned char byte = static_cast<unsigned char>(character);
        character = std::isalnum(byte) ? static_cast<char>(std::tolower(byte)) : '_';
    }
    const std::string handler = "trigger_" + scriptEvent;
    const bool pressHandlerFound = scripts != nullptr && scripts->hasScriptEntryPoint(handler);
    const bool dragHandlerFound =
        scripts != nullptr && scripts->hasScriptEntryPoint(handler + "_drag");
    const bool releaseHandlerFound =
        scripts != nullptr && scripts->hasScriptEntryPoint(handler + "_off");
    std::ostringstream description;
    description << "Clickable part[" << partIndex << "] mesh=\"" << part.meshName << "\"";
    if (!part.meshIdentifier.empty()) {
        description << " id=\"" << part.meshIdentifier << "\"";
    }
    description << " mouseevent=\"" << part.mouseEvent << "\" maps_to=" << handler
                << " press=" << (pressHandlerFound ? "found" : "missing")
                << " drag=" << (dragHandlerFound ? "found" : "missing")
                << " release=" << (releaseHandlerFound ? "found" : "missing")
                << " visible=" << (clickablePartVisible(part, partIndex, context) ? "yes" : "no")
                << " bounds_size_xyz=(" << part.size[0] << ',' << part.size[1] << ','
                << part.size[2] << ") [model units]";
    return description.str();
}

void Vehicle::logClickableDebugInfo(RenderViewContext context) const {
    std::size_t clickableCount = 0;
    for (const DisplayPart& part : displayLists) {
        clickableCount += !part.mouseEvent.empty() ? 1U : 0U;
    }
    gameLog.Log("Clickable overlay: " + std::to_string(clickableCount) +
                " mesh(es) with [mouseevent]; bounds are in model units");
    for (std::size_t partIndex = 0; partIndex < displayLists.size(); ++partIndex) {
        const DisplayPart& part = displayLists[partIndex];
        if (!part.mouseEvent.empty()) {
            gameLog.Log(clickableDebugDescription(part, partIndex, context));
        }
    }
}

void Vehicle::drawClickableDebug(RenderViewContext context) const {
    TraceScope trace("debug", "Vehicle::drawClickableDebug");
    static constexpr std::array<std::array<int, 2>, 12> boxEdges = {{{0, 1},
                                                                     {1, 3},
                                                                     {3, 2},
                                                                     {2, 0},
                                                                     {4, 5},
                                                                     {5, 7},
                                                                     {7, 6},
                                                                     {6, 4},
                                                                     {0, 4},
                                                                     {1, 5},
                                                                     {2, 6},
                                                                     {3, 7}}};
    clickableDebugBoxVertices.clear();
    clickableDebugWireVertices.clear();
    clickableDebugTriangleVertices.clear();
    clickableDebugBoxVertices.reserve(visibleClickablePartIndices.size() * 24);
    const auto appendVertex = [](std::vector<openbus::rendering::PrimitiveVertex>& vertices,
                                 const std::array<double, 4>& point,
                                 const std::array<float, 3>& color) {
        vertices.push_back({static_cast<float>(point[0]), static_cast<float>(point[1]),
                            static_cast<float>(point[2]), color[0], color[1], color[2]});
    };
    const auto transformVertex = [this](const Matrix4& animation, const Vertex& vertex) {
        std::array<double, 4> point =
            transformPoint(animation, {vertex.x, vertex.y, vertex.z, 1.0});
        point[2] += modelOffsetZ;
        return point;
    };
    const DisplayPart* selected = nullptr;
    const Batch* selectedBatch = nullptr;
    std::size_t selectedTriangle = hoveredClickableTriangle;
    if (hoveredClickablePartIndex < displayLists.size()) {
        const DisplayPart& part = displayLists[hoveredClickablePartIndex];
        if (clickablePartVisible(part, hoveredClickablePartIndex, context)) {
            selected = &part;
            if (hoveredClickableBatchIndex < part.batches.size()) {
                selectedBatch = &part.batches[hoveredClickableBatchIndex];
            }
        }
    }
    for (std::size_t partIndex = 0; partIndex < displayLists.size(); ++partIndex) {
        const DisplayPart& part = displayLists[partIndex];
        if (part.mouseEvent.empty() || !clickablePartVisible(part, partIndex, context)) {
            continue;
        }
        const bool isSelected = &part == selected;
        const std::array<float, 3> boundsColor = isSelected
                                                     ? std::array<float, 3>{1.0f, 0.15f, 0.05f}
                                                     : std::array<float, 3>{0.0f, 0.75f, 0.95f};
        const Matrix4 animation = animationTransformForPart(part);
        const std::array<double, 3> half = {std::max(part.size[0] * 0.5, 0.0),
                                            std::max(part.size[1] * 0.5, 0.0),
                                            std::max(part.size[2] * 0.5, 0.0)};
        std::array<std::array<double, 4>, 8> corners = {};
        for (int corner = 0; corner < 8; ++corner) {
            const std::array<double, 4> local = {
                part.center[0] + ((corner & 1) == 0 ? -half[0] : half[0]),
                part.center[1] + ((corner & 2) == 0 ? -half[1] : half[1]),
                part.center[2] + ((corner & 4) == 0 ? -half[2] : half[2]), 1.0};
            corners[static_cast<std::size_t>(corner)] = transformPoint(animation, local);
            corners[static_cast<std::size_t>(corner)][2] += modelOffsetZ;
        }
        for (const auto& edge : boxEdges) {
            appendVertex(clickableDebugBoxVertices, corners[static_cast<std::size_t>(edge[0])],
                         boundsColor);
            appendVertex(clickableDebugBoxVertices, corners[static_cast<std::size_t>(edge[1])],
                         boundsColor);
        }
        const std::array<float, 3> meshColor = {1.0f, 0.85f, 0.05f};
        for (const Batch& batch : part.batches) {
            for (std::size_t index = 0; index + 2 < batch.vertices.size(); index += 3) {
                const auto first = transformVertex(animation, batch.vertices[index]);
                const auto second = transformVertex(animation, batch.vertices[index + 1]);
                const auto third = transformVertex(animation, batch.vertices[index + 2]);
                appendVertex(clickableDebugWireVertices, first, meshColor);
                appendVertex(clickableDebugWireVertices, second, meshColor);
                appendVertex(clickableDebugWireVertices, second, meshColor);
                appendVertex(clickableDebugWireVertices, third, meshColor);
                appendVertex(clickableDebugWireVertices, third, meshColor);
                appendVertex(clickableDebugWireVertices, first, meshColor);
            }
            if (isSelected && selectedBatch == &batch &&
                selectedTriangle + 2 < batch.vertices.size()) {
                const std::array<float, 3> fillColor = {1.0f, 0.15f, 0.02f};
                const std::array<float, 3> outlineColor = {1.0f, 1.0f, 0.2f};
                const auto first = transformVertex(animation, batch.vertices[selectedTriangle]);
                const auto second =
                    transformVertex(animation, batch.vertices[selectedTriangle + 1]);
                const auto third = transformVertex(animation, batch.vertices[selectedTriangle + 2]);
                appendVertex(clickableDebugTriangleVertices, first, fillColor);
                appendVertex(clickableDebugTriangleVertices, second, fillColor);
                appendVertex(clickableDebugTriangleVertices, third, fillColor);
                appendVertex(clickableDebugWireVertices, first, outlineColor);
                appendVertex(clickableDebugWireVertices, second, outlineColor);
                appendVertex(clickableDebugWireVertices, second, outlineColor);
                appendVertex(clickableDebugWireVertices, third, outlineColor);
                appendVertex(clickableDebugWireVertices, third, outlineColor);
                appendVertex(clickableDebugWireVertices, first, outlineColor);
            }
        }
    }
    if (clickableDebugBoxVertices.empty() && clickableDebugWireVertices.empty()) {
        return;
    }
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    if (!clickableDebugBoxVertices.empty()) {
        openbus::rendering::drawPrimitives(clickableDebugBoxVertices, GL_LINES);
    }
    if (!clickableDebugTriangleVertices.empty()) {
        openbus::rendering::drawPrimitives(clickableDebugTriangleVertices, GL_TRIANGLES);
    }
    if (!clickableDebugWireVertices.empty()) {
        openbus::rendering::drawPrimitives(clickableDebugWireVertices, GL_LINES, 1.5f);
    }
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
}

std::string Vehicle::mouseEventAt(double cursorX, double cursorY, int viewportWidth,
                                  int viewportHeight, RenderViewContext context) const {
    const DisplayPart* selected =
        pickClickable(cursorX, cursorY, viewportWidth, viewportHeight, context);
    return selected == nullptr ? std::string() : selected->mouseEvent;
}

bool Vehicle::handleMouseClick(const std::string& eventName) {
    if (!scripts) {
        return false;
    }
    if (eventName.empty()) {
        return false;
    }
    if (scripts->hasScriptEntryPoint("trigger_" + lower(eventName))) {
        scripts->invokeMouseEvent(eventName);
    }
    return true;
}
