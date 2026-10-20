// Mouse picking of clickable parts and the clickable debug overlay.

#include "app/vehicle/Vehicle.h"

namespace {

constexpr std::size_t kPickChunkTriangles = 16;
constexpr std::size_t kPickChunksPerGroup = 32;

Vehicle::Batch::PickChunkBounds emptyPickBounds() {
    constexpr float infinity = std::numeric_limits<float>::infinity();
    return {{infinity, infinity, infinity}, {-infinity, -infinity, -infinity}};
}

void expandPickBounds(Vehicle::Batch::PickChunkBounds& bounds, const std::array<float, 3>& minimum,
                      const std::array<float, 3>& maximum) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
        bounds.minimum[axis] = std::min(bounds.minimum[axis], minimum[axis]);
        bounds.maximum[axis] = std::max(bounds.maximum[axis], maximum[axis]);
    }
}

// Builds (or reuses) the batch's chunk and chunk-group bounds.
void ensurePickChunkBounds(const Vehicle::Batch& batch) {
    const std::shared_ptr<std::vector<Vehicle::Vertex>>& source = batch.vertices.values;
    const std::size_t vertexCount = batch.vertices.size();
    const bool sameSource =
        !batch.pickChunkSource.owner_before(source) && !source.owner_before(batch.pickChunkSource);
    if (sameSource && batch.pickChunkVertexCount == vertexCount) {
        return;
    }
    const std::size_t triangleCount = vertexCount / 3;
    const Vehicle::Vertex* vertices = batch.vertices.data();
    std::vector<Vehicle::Batch::PickChunkBounds> chunks;
    chunks.reserve((triangleCount + kPickChunkTriangles - 1) / kPickChunkTriangles);
    for (std::size_t first = 0; first < triangleCount; first += kPickChunkTriangles) {
        const std::size_t last = std::min(triangleCount, first + kPickChunkTriangles);
        Vehicle::Batch::PickChunkBounds chunk = emptyPickBounds();
        for (std::size_t index = first * 3; index < last * 3; ++index) {
            const Vehicle::Vertex& vertex = vertices[index];
            const std::array<float, 3> point = {vertex.x, vertex.y, vertex.z};
            expandPickBounds(chunk, point, point);
        }
        chunks.push_back(chunk);
    }
    std::vector<Vehicle::Batch::PickChunkBounds> groups;
    groups.reserve((chunks.size() + kPickChunksPerGroup - 1) / kPickChunksPerGroup);
    for (std::size_t first = 0; first < chunks.size(); first += kPickChunksPerGroup) {
        const std::size_t last = std::min(chunks.size(), first + kPickChunksPerGroup);
        Vehicle::Batch::PickChunkBounds group = emptyPickBounds();
        for (std::size_t index = first; index < last; ++index) {
            expandPickBounds(group, chunks[index].minimum, chunks[index].maximum);
        }
        groups.push_back(group);
    }
    batch.pickChunkBounds = std::move(chunks);
    batch.pickChunkGroupBounds = std::move(groups);
    batch.pickChunkSource = source;
    batch.pickChunkVertexCount = vertexCount;
}

} // namespace

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
    if (viewportWidth <= 0 || viewportHeight <= 0 || visibleClickablePartIndices.empty()) {
        return nullptr;
    }
    const auto& modelView = openbus::rendering::modelViewMatrix();
    const auto& projection = openbus::rendering::projectionMatrix();
    const Batch* selectedBatch = nullptr;
    std::size_t selectedTriangle = 0;
    const double normalizedX = cursorX / static_cast<double>(viewportWidth) * 2.0 - 1.0;
    const double normalizedY = 1.0 - cursorY / static_cast<double>(viewportHeight) * 2.0;
    // Only clip x/y/w and view z are needed per vertex, so each part's local->clip
    // transform is collapsed into four rows instead of two full 4x4 transforms.
    struct PickTransform {
        std::array<double, 4> clipX{};
        std::array<double, 4> clipY{};
        std::array<double, 4> clipW{};
        std::array<double, 4> viewZ{};
    };
    const auto makePickTransform = [&](const Matrix4& modelViewPart) {
        const Matrix4 clip = multiplyMatrix4(projection, modelViewPart);
        const auto row = [](const Matrix4& matrix, int index) {
            return std::array<double, 4>{matrix[index], matrix[index + 4], matrix[index + 8],
                                         matrix[index + 12]};
        };
        return PickTransform{row(clip, 0), row(clip, 1), row(clip, 3), row(modelViewPart, 2)};
    };
    struct ProjectedVertex {
        double x = 0.0;
        double y = 0.0;
        double depth = 0.0;
    };
    const auto projectVertex = [](const PickTransform& transform, const Vertex& vertex,
                                  ProjectedVertex& projected) {
        const double x = vertex.x;
        const double y = vertex.y;
        const double z = vertex.z;
        const auto dot = [&](const std::array<double, 4>& row) {
            return row[0] * x + row[1] * y + row[2] * z + row[3];
        };
        const double w = dot(transform.clipW);
        if (w <= 1.0e-8) {
            return false;
        }
        projected.x = dot(transform.clipX) / w;
        projected.y = dot(transform.clipY) / w;
        projected.depth = -dot(transform.viewZ);
        return true;
    };
    const auto cross2d = [](double ax, double ay, double bx, double by) {
        return ax * by - ay * bx;
    };
    constexpr double triangleTolerance = 1.0e-7;
    // Same slack as the per-triangle bounds test below: a triangle's projection lies
    // inside its chunk's projected box, so a rejected chunk cannot contain a hit.
    const auto cursorWithinSlackBounds = [&](double minimumX, double maximumX, double minimumY,
                                             double maximumY) {
        const double slackX = 4.0 * triangleTolerance * (maximumX - minimumX) + 1.0e-12;
        const double slackY = 4.0 * triangleTolerance * (maximumY - minimumY) + 1.0e-12;
        return !(normalizedX < minimumX - slackX || normalizedX > maximumX + slackX ||
                 normalizedY < minimumY - slackY || normalizedY > maximumY + slackY);
    };
    // Conservative test of a local-space box. Rejects boxes entirely behind the near
    // threshold, boxes whose nearest possible hit depth exceeds maximumDepth, and boxes
    // whose screen projection cannot contain the cursor.
    const auto boundsMayContainCursor = [&](const PickTransform& transform,
                                            const Batch::PickChunkBounds& bounds,
                                            double maximumDepth) {
        // Triangle vertices need w > 1e-8; clipping at half that absorbs rounding.
        constexpr double clipW = 0.5e-8;
        std::array<std::array<double, 3>, 8> corners{};
        double nearestDepth = std::numeric_limits<double>::max();
        double farthestDepth = std::numeric_limits<double>::lowest();
        bool anyInFront = false;
        bool anyBehind = false;
        for (int corner = 0; corner < 8; ++corner) {
            const double x = (corner & 1) == 0 ? bounds.minimum[0] : bounds.maximum[0];
            const double y = (corner & 2) == 0 ? bounds.minimum[1] : bounds.maximum[1];
            const double z = (corner & 4) == 0 ? bounds.minimum[2] : bounds.maximum[2];
            const auto dot = [&](const std::array<double, 4>& row) {
                return row[0] * x + row[1] * y + row[2] * z + row[3];
            };
            const double depth = -dot(transform.viewZ);
            corners[static_cast<std::size_t>(corner)] = {dot(transform.clipX), dot(transform.clipY),
                                                         dot(transform.clipW)};
            const auto& clip = corners[static_cast<std::size_t>(corner)];
            if (!std::isfinite(clip[0]) || !std::isfinite(clip[1]) || !std::isfinite(clip[2]) ||
                !std::isfinite(depth)) {
                return true;
            }
            nearestDepth = std::min(nearestDepth, depth);
            farthestDepth = std::max(farthestDepth, depth);
            (clip[2] > clipW ? anyInFront : anyBehind) = true;
        }
        // w is affine, so no contained vertex can pass the triangle test's w check.
        if (!anyInFront) {
            return false;
        }
        // Tolerated barycentric weights can undershoot the vertex depths by 2t * range.
        const double nearestHitDepth = nearestDepth -
                                       4.0 * triangleTolerance * (farthestDepth - nearestDepth) -
                                       1.0e-9 * std::max(1.0, std::abs(nearestDepth));
        if (nearestHitDepth > maximumDepth) {
            return false;
        }
        // Screen bounds of the box clipped to w >= clipW: projected front corners plus
        // the projected points where box edges cross the clip plane.
        double minimumX = std::numeric_limits<double>::max();
        double maximumX = std::numeric_limits<double>::lowest();
        double minimumY = std::numeric_limits<double>::max();
        double maximumY = std::numeric_limits<double>::lowest();
        const auto include = [&](double x, double y, double w) {
            const double ndcX = x / w;
            const double ndcY = y / w;
            minimumX = std::min(minimumX, ndcX);
            maximumX = std::max(maximumX, ndcX);
            minimumY = std::min(minimumY, ndcY);
            maximumY = std::max(maximumY, ndcY);
        };
        for (const auto& clip : corners) {
            if (clip[2] > clipW) {
                include(clip[0], clip[1], clip[2]);
            }
        }
        if (anyBehind) {
            for (int corner = 0; corner < 8; ++corner) {
                for (int axisBit = 1; axisBit < 8; axisBit <<= 1) {
                    if ((corner & axisBit) != 0) {
                        continue;
                    }
                    const auto& first = corners[static_cast<std::size_t>(corner)];
                    const auto& second = corners[static_cast<std::size_t>(corner | axisBit)];
                    if ((first[2] > clipW) == (second[2] > clipW)) {
                        continue;
                    }
                    const double s = (clipW - first[2]) / (second[2] - first[2]);
                    include(first[0] + s * (second[0] - first[0]),
                            first[1] + s * (second[1] - first[1]), clipW);
                }
            }
        }
        if (!std::isfinite(minimumX) || !std::isfinite(maximumX) || !std::isfinite(minimumY) ||
            !std::isfinite(maximumY)) {
            return true;
        }
        return cursorWithinSlackBounds(minimumX, maximumX, minimumY, maximumY);
    };
    // Visits triangles of a batch whose chunk bounds may contain the cursor at a depth up
    // to maximumDepth (re-read per chunk); the visitor returns true to stop early.
    const auto forEachCandidateTriangle = [&](const PickTransform& transform, const Batch& batch,
                                              const double& maximumDepth, const auto& visit) {
        ensurePickChunkBounds(batch);
        const std::vector<Batch::PickChunkBounds>& groups = batch.pickChunkGroupBounds;
        const std::vector<Batch::PickChunkBounds>& chunks = batch.pickChunkBounds;
        const Vertex* vertices = batch.vertices.data();
        const std::size_t triangleVertices = batch.vertices.size() / 3 * 3;
        for (std::size_t group = 0; group < groups.size(); ++group) {
            if (!boundsMayContainCursor(transform, groups[group], maximumDepth)) {
                continue;
            }
            const std::size_t lastChunk =
                std::min(chunks.size(), (group + 1) * kPickChunksPerGroup);
            for (std::size_t chunk = group * kPickChunksPerGroup; chunk < lastChunk; ++chunk) {
                if (!boundsMayContainCursor(transform, chunks[chunk], maximumDepth)) {
                    continue;
                }
                const std::size_t begin = chunk * kPickChunkTriangles * 3;
                const std::size_t end = std::min(triangleVertices, begin + kPickChunkTriangles * 3);
                for (std::size_t index = begin; index < end; index += 3) {
                    if (visit(vertices + index, index)) {
                        return true;
                    }
                }
            }
        }
        return false;
    };
    const auto triangleDepthAtCursor = [&](const PickTransform& transform, const DisplayPart& part,
                                           const Vertex* vertices, double& depth) {
        ProjectedVertex first;
        ProjectedVertex second;
        ProjectedVertex third;
        if (!projectVertex(transform, vertices[0], first) ||
            !projectVertex(transform, vertices[1], second) ||
            !projectVertex(transform, vertices[2], third)) {
            return false;
        }
        // Cheap screen-space bounds rejection. Barycentric weights >= -t can only reach
        // 2t * extent outside the triangle bounds, so the slack keeps edge hits identical.
        if (!cursorWithinSlackBounds(
                std::min({first.x, second.x, third.x}), std::max({first.x, second.x, third.x}),
                std::min({first.y, second.y, third.y}), std::max({first.y, second.y, third.y}))) {
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
        const PickTransform transform = makePickTransform(modelViewPart);
        double partDistance = std::numeric_limits<double>::max();
        const Batch* partBatch = nullptr;
        std::size_t partTriangle = 0;
        // Deeper triangles can neither beat this part's best hit nor the current selection.
        double depthLimit = closestDepth;
        for (const Batch& batch : part.batches) {
            forEachCandidateTriangle(
                transform, batch, depthLimit, [&](const Vertex* vertices, std::size_t index) {
                    double triangleDistance = 0.0;
                    if (triangleDepthAtCursor(transform, part, vertices, triangleDistance) &&
                        triangleDistance < partDistance) {
                        partDistance = triangleDistance;
                        partBatch = &batch;
                        partTriangle = index;
                        depthLimit = std::min(depthLimit, partDistance);
                    }
                    return false;
                });
        }
        if (partDistance >= closestDepth) {
            continue;
        }
        if (partDistance == std::numeric_limits<double>::max()) {
            continue;
        }
        closestDepth = partDistance;
        selected = &part;
        selectedBatch = partBatch;
        selectedTriangle = partTriangle;
    }
    if (selected == nullptr) {
        return nullptr;
    }
    // If the nearest clickable is blocked, every deeper clickable is blocked too.
    for (const std::size_t partIndex : visibleOpaquePickPartIndices) {
        const DisplayPart& part = displayLists[partIndex];
        // All triangles in the selected part were tested when finding its nearest hit.
        if (&part == selected) {
            continue;
        }
        const Matrix4 modelViewPart =
            multiplyMatrix4(modelView, multiplyMatrix4(translationMatrix({0.0, 0.0, modelOffsetZ}),
                                                       animationTransformForPart(part)));
        std::array<double, 4> viewCenter = {};
        std::array<double, 4> clipCenter = {};
        if (!cursorWithinProjectedBounds(modelViewPart, part, viewCenter, clipCenter)) {
            continue;
        }
        const double depthRadius = std::abs(modelViewPart[2]) * std::max(part.size[0] * 0.5, 0.0) +
                                   std::abs(modelViewPart[6]) * std::max(part.size[1] * 0.5, 0.0) +
                                   std::abs(modelViewPart[10]) * std::max(part.size[2] * 0.5, 0.0);
        // Two barycentric weights can be negative at tolerated triangle edges.
        const double minimumDepth = -viewCenter[2] - depthRadius * (1.0 + 4.0 * triangleTolerance);
        if (std::isfinite(minimumDepth) && closestDepth < minimumDepth) {
            continue;
        }
        const PickTransform transform = makePickTransform(modelViewPart);
        for (const Batch& batch : part.batches) {
            if (batch.alphaMode != 0 || batch.noZwrite || batch.noZcheck) {
                continue;
            }
            const bool blocked = forEachCandidateTriangle(
                transform, batch, closestDepth, [&](const Vertex* vertices, std::size_t) {
                    double triangleDepth = 0.0;
                    return triangleDepthAtCursor(transform, part, vertices, triangleDepth) &&
                           closestDepth > triangleDepth + 1.0e-4 * std::max(1.0, triangleDepth);
                });
            if (blocked) {
                return nullptr;
            }
        }
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
