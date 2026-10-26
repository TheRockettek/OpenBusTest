#include "physics/World.h"

#include "core/Logger.h"
#include "core/PerfTrace.h"
#include "physics/RoadFeatures.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace {
Logger worldLog("World");

constexpr dReal gravity = -9.81;
constexpr double pi = 3.14159265358979323846;
constexpr double poleBendSpeedRadiansPerSecond = 4.0;
constexpr int roadBumpSegments = 12;
constexpr int roadInclineSegments = 128;

int checkedOdeCount(std::size_t count, const char* label) {
    if (!std::in_range<int>(count)) {
        throw std::runtime_error(std::string(label) + " exceeds ODE's integer limit");
    }
    return static_cast<int>(count);
}

std::uint64_t mapTileKey(int tileX, int tileY) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(tileX)) << 32U) |
           static_cast<std::uint32_t>(tileY);
}

class OdeRuntime {
  public:
    OdeRuntime() {
        openbus::rendering::TraceScope trace("physics", "World::OdeRuntime.initialize");
        if (dInitODE2(0) == 0) {
            throw std::runtime_error("Failed to initialize ODE");
        }
        if (dAllocateODEDataForThread(dAllocateFlagCollisionData) == 0) {
            dCloseODE();
            throw std::runtime_error("Failed to allocate ODE collision data for this thread");
        }
        world = dWorldCreate();
        space = dHashSpaceCreate(nullptr);
        contacts = dJointGroupCreate(0);
        if (!world || !space || !contacts) {
            if (contacts) {
                dJointGroupDestroy(contacts);
            }
            if (space) {
                dSpaceDestroy(space);
            }
            if (world) {
                dWorldDestroy(world);
            }
            dCloseODE();
            throw std::runtime_error("Failed to create ODE runtime");
        }
        dWorldSetGravity(world, 0.0, 0.0, gravity);
        dWorldSetERP(world, 0.8);
        dWorldSetCFM(world, 1e-5);
    }

    ~OdeRuntime() {
        dJointGroupDestroy(contacts);
        dSpaceDestroy(space);
        dWorldDestroy(world);
        dCloseODE();
    }

    dWorldID world = nullptr;
    dSpaceID space = nullptr;
    dJointGroupID contacts = nullptr;
};
} // namespace

struct World::Impl {
    struct StreamedTileCollision {
        std::vector<dGeomID> geometries;
        std::vector<dTriMeshDataID> meshData;
        std::vector<dGeomID> poleGeometries;
        std::vector<dBodyID> poleBodies;
        std::vector<std::vector<double>> terrainVertices;
        std::vector<std::vector<int>> terrainIndices;
        std::vector<StaticCollisionMesh> staticMeshes;
    };

    struct PoleRuntime {
        MapPoleCollision definition;
        dBodyID body = nullptr;
        dGeomID geometry = nullptr;
        std::array<double, 3> up = {};
        std::array<double, 3> rotationAxis = {};
        double length = 0.0;
        double angleRadians = 0.0;
        double targetAngleRadians = 0.0;
        bool targetBroken = false;
        bool broken = false;
    };

    struct PoleImpact {
        dBodyID vehicleBody = nullptr;
        double speedMetersPerSecond = 0.0;
        std::array<double, 3> velocity = {};
    };

    OdeRuntime ode;
    dGeomID ground = nullptr;
    std::vector<dGeomID> roadGeometries;
    std::vector<dGeomID> roadMeshGeometries;
    std::vector<dTriMeshDataID> roadMeshData;
    std::vector<std::vector<double>> roadMeshVertices;
    std::vector<std::vector<int>> roadMeshIndices;
    std::vector<dGeomID> terrainGeometries;
    std::vector<dTriMeshDataID> terrainMeshData;
    std::vector<std::vector<double>> terrainVertices;
    std::vector<std::vector<int>> terrainIndices;
    std::unordered_set<std::uint64_t> initialTerrainTileKeys;
    std::vector<dGeomID> staticGeometries;
    std::vector<dTriMeshDataID> staticMeshData;
    std::vector<StaticCollisionMesh> staticMeshes;
    std::unordered_map<std::uint64_t, StreamedTileCollision> streamedTiles;
    std::unordered_map<dGeomID, std::unique_ptr<PoleRuntime>> mapPoles;
    std::unordered_set<dGeomID> poleContactsPrevious;
    std::unordered_set<dGeomID> poleContactsCurrent;
    std::unordered_map<dGeomID, PoleImpact> poleImpactsCurrent;
    std::uint64_t collisionRevision = 0;

    ~Impl() {
        if (ground) {
            dGeomDestroy(ground);
        }
        for (dGeomID geometry : roadGeometries) {
            if (geometry) {
                dGeomDestroy(geometry);
            }
        }
        destroyMeshes(roadMeshGeometries, roadMeshData);
        destroyMeshes(terrainGeometries, terrainMeshData);
        destroyMeshes(staticGeometries, staticMeshData);
        for (auto& [key, tile] : streamedTiles) {
            static_cast<void>(key);
            for (dGeomID geometry : tile.poleGeometries) {
                if (geometry) {
                    dGeomDestroy(geometry);
                }
            }
            for (dBodyID body : tile.poleBodies) {
                if (body) {
                    dBodyDestroy(body);
                }
            }
            destroyMeshes(tile.geometries, tile.meshData);
        }
    }

    static void destroyMeshes(const std::vector<dGeomID>& geometries,
                              const std::vector<dTriMeshDataID>& data) {
        for (dGeomID geometry : geometries) {
            if (geometry) {
                dGeomDestroy(geometry);
            }
        }
        for (dTriMeshDataID mesh : data) {
            if (mesh) {
                dGeomTriMeshDataDestroy(mesh);
            }
        }
    }

    void addTerrainCollision(const TerrainCollisionGrid& terrain,
                             StreamedTileCollision* streamedTile = nullptr) {
        openbus::rendering::TraceScope trace("physics", "World::addTerrainCollision");
        if (terrain.intervals == 0 || terrain.intervals > 1024 ||
            !std::isfinite(terrain.tileSizeMeters) || terrain.tileSizeMeters <= 0.0) {
            throw std::invalid_argument("Map terrain grid has invalid dimensions");
        }
        const std::size_t side = terrain.intervals + 1;
        const std::size_t vertexCount = side * side;
        if (terrain.heights.size() != vertexCount) {
            throw std::invalid_argument("Map terrain height count does not match its grid");
        }
        const std::size_t triangleIndexCount = terrain.intervals * terrain.intervals * 6;
        checkedOdeCount(vertexCount, "map terrain vertex count");
        checkedOdeCount(triangleIndexCount, "map terrain index count");
        std::vector<double> vertices;
        std::vector<int> indices;
        vertices.reserve(vertexCount * 3);
        indices.reserve(triangleIndexCount);
        const double sampleSpacing = terrain.tileSizeMeters / terrain.intervals;
        for (std::size_t row = 0; row < side; ++row) {
            for (std::size_t column = 0; column < side; ++column) {
                const float height = terrain.heights[row * side + column];
                if (!std::isfinite(height)) {
                    throw std::invalid_argument("Map terrain contains a non-finite height");
                }
                vertices.push_back(static_cast<double>(terrain.tileX) * terrain.tileSizeMeters +
                                   static_cast<double>(column) * sampleSpacing);
                vertices.push_back(static_cast<double>(terrain.tileY) * terrain.tileSizeMeters +
                                   static_cast<double>(row) * sampleSpacing);
                vertices.push_back(height);
            }
        }
        for (std::size_t row = 0; row < terrain.intervals; ++row) {
            for (std::size_t column = 0; column < terrain.intervals; ++column) {
                const std::size_t topLeft = row * side + column;
                const std::size_t topRight = topLeft + 1;
                const std::size_t bottomLeft = topLeft + side;
                const std::size_t bottomRight = bottomLeft + 1;
                indices.insert(indices.end(),
                               {static_cast<int>(topLeft), static_cast<int>(topRight),
                                static_cast<int>(bottomRight), static_cast<int>(topLeft),
                                static_cast<int>(bottomRight), static_cast<int>(bottomLeft)});
            }
        }
        if (streamedTile != nullptr) {
            streamedTile->terrainVertices.reserve(streamedTile->terrainVertices.size() + 1);
            streamedTile->terrainIndices.reserve(streamedTile->terrainIndices.size() + 1);
            streamedTile->meshData.reserve(streamedTile->meshData.size() + 1);
            streamedTile->geometries.reserve(streamedTile->geometries.size() + 1);
        } else {
            terrainVertices.reserve(terrainVertices.size() + 1);
            terrainIndices.reserve(terrainIndices.size() + 1);
            terrainMeshData.reserve(terrainMeshData.size() + 1);
            terrainGeometries.reserve(terrainGeometries.size() + 1);
        }
        const dTriMeshDataID meshData = dGeomTriMeshDataCreate();
        if (!meshData) {
            throw std::runtime_error("Failed to create map terrain collision data");
        }
        dGeomTriMeshDataBuildDouble(
            meshData, vertices.data(), 3 * sizeof(double),
            checkedOdeCount(vertexCount, "map terrain vertex count"), indices.data(),
            checkedOdeCount(indices.size(), "map terrain index count"), 3 * sizeof(int));
        const dGeomID geometry = dCreateTriMesh(ode.space, meshData, nullptr, nullptr, nullptr);
        if (!geometry) {
            dGeomTriMeshDataDestroy(meshData);
            throw std::runtime_error("Failed to create map terrain collision geometry");
        }
        if (streamedTile != nullptr) {
            streamedTile->terrainVertices.push_back(std::move(vertices));
            streamedTile->terrainIndices.push_back(std::move(indices));
            streamedTile->meshData.push_back(meshData);
            streamedTile->geometries.push_back(geometry);
        } else {
            terrainVertices.push_back(std::move(vertices));
            terrainIndices.push_back(std::move(indices));
            terrainMeshData.push_back(meshData);
            terrainGeometries.push_back(geometry);
        }
    }

    void addStaticCollision(StaticCollisionMesh mesh,
                            StreamedTileCollision* streamedTile = nullptr) {
        openbus::rendering::TraceScope trace("physics", "World::addStaticCollision");
        if (mesh.vertices.empty() && mesh.indices.empty()) {
            return;
        }
        if (mesh.vertices.empty() || mesh.vertices.size() % 3 != 0 || mesh.indices.empty() ||
            mesh.indices.size() % 3 != 0) {
            throw std::invalid_argument("Map static collision mesh has invalid buffers");
        }
        const std::size_t vertexCount = mesh.vertices.size() / 3;
        checkedOdeCount(vertexCount, "map static collision vertex count");
        checkedOdeCount(mesh.indices.size(), "map static collision index count");
        if (!std::all_of(mesh.vertices.begin(), mesh.vertices.end(),
                         [](double value) { return std::isfinite(value); })) {
            throw std::invalid_argument("Map static collision mesh contains a non-finite vertex");
        }
        for (const int index : mesh.indices) {
            if (index < 0 || static_cast<std::size_t>(index) >= vertexCount) {
                throw std::invalid_argument("Map static collision mesh has an invalid index");
            }
        }
        std::vector<dGeomID>& geometries =
            streamedTile != nullptr ? streamedTile->geometries : staticGeometries;
        std::vector<dTriMeshDataID>& meshDataStorage =
            streamedTile != nullptr ? streamedTile->meshData : staticMeshData;
        std::vector<StaticCollisionMesh>& meshStorage =
            streamedTile != nullptr ? streamedTile->staticMeshes : staticMeshes;
        geometries.reserve(geometries.size() + 1);
        meshDataStorage.reserve(meshDataStorage.size() + 1);
        meshStorage.reserve(meshStorage.size() + 1);
        meshStorage.push_back(std::move(mesh));
        const StaticCollisionMesh& stored = meshStorage.back();
        const dTriMeshDataID meshData = dGeomTriMeshDataCreate();
        if (!meshData) {
            meshStorage.pop_back();
            throw std::runtime_error("Failed to create map static collision data");
        }
        dGeomTriMeshDataBuildDouble(
            meshData, stored.vertices.data(), 3 * sizeof(double),
            checkedOdeCount(vertexCount, "map static collision vertex count"),
            stored.indices.data(),
            checkedOdeCount(stored.indices.size(), "map static collision index count"),
            3 * sizeof(int));
        const dGeomID geometry = dCreateTriMesh(ode.space, meshData, nullptr, nullptr, nullptr);
        if (!geometry) {
            dGeomTriMeshDataDestroy(meshData);
            meshStorage.pop_back();
            throw std::runtime_error("Failed to create map static collision geometry");
        }
        meshDataStorage.push_back(meshData);
        geometries.push_back(geometry);
    }

    static std::array<double, 3> rotatedVector(const std::array<double, 3>& vector,
                                               const std::array<double, 3>& axis, double angle) {
        const double cosine = std::cos(angle);
        const double sine = std::sin(angle);
        const double dot = vector[0] * axis[0] + vector[1] * axis[1] + vector[2] * axis[2];
        const std::array<double, 3> cross = {axis[1] * vector[2] - axis[2] * vector[1],
                                             axis[2] * vector[0] - axis[0] * vector[2],
                                             axis[0] * vector[1] - axis[1] * vector[0]};
        return {vector[0] * cosine + cross[0] * sine + axis[0] * dot * (1.0 - cosine),
                vector[1] * cosine + cross[1] * sine + axis[1] * dot * (1.0 - cosine),
                vector[2] * cosine + cross[2] * sine + axis[2] * dot * (1.0 - cosine)};
    }

    void setPoleTransform(PoleRuntime& pole) {
        const std::array<double, 3> newUp =
            rotatedVector(pole.up, pole.rotationAxis, pole.angleRadians);
        const std::array<double, 3> center = {
            pole.definition.basePosition[0] + newUp[0] * pole.length * 0.5,
            pole.definition.basePosition[1] + newUp[1] * pole.length * 0.5,
            pole.definition.basePosition[2] + newUp[2] * pole.length * 0.5};
        dBodySetPosition(pole.body, center[0], center[1], center[2]);
        dMatrix3 rotation;
        const std::array<double, 3> axis = {-newUp[1], newUp[0], 0.0};
        const double axisLength = std::hypot(axis[0], axis[1]);
        if (axisLength > 1.0e-8) {
            dRFromAxisAndAngle(rotation, axis[0] / axisLength, axis[1] / axisLength, axis[2],
                               std::atan2(axisLength, std::clamp(newUp[2], -1.0, 1.0)));
        } else {
            dRFromAxisAndAngle(rotation, 1.0, 0.0, 0.0, newUp[2] < 0.0 ? pi : 0.0);
        }
        dBodySetRotation(pole.body, rotation);
    }

    void addPoleCollision(const MapPoleCollision& definition, StreamedTileCollision& streamedTile) {
        const std::array<double, 3> axis = {definition.topPosition[0] - definition.basePosition[0],
                                            definition.topPosition[1] - definition.basePosition[1],
                                            definition.topPosition[2] - definition.basePosition[2]};
        const double length = std::hypot(axis[0], axis[1], axis[2]);
        if (!std::isfinite(length) || length <= 0.0 || !std::isfinite(definition.radiusMeters) ||
            definition.radiusMeters <= 0.0) {
            throw std::invalid_argument("Map pole collision has invalid dimensions");
        }
        auto pole = std::make_unique<PoleRuntime>();
        pole->definition = definition;
        pole->length = length;
        pole->up = {axis[0] / length, axis[1] / length, axis[2] / length};
        const double capsuleLength = std::max(0.01, length - 2.0 * definition.radiusMeters);
        pole->body = dBodyCreate(ode.world);
        if (!pole->body) {
            throw std::runtime_error("Failed to create map pole body");
        }
        dBodySetKinematic(pole->body);
        pole->geometry = dCreateCapsule(ode.space, definition.radiusMeters, capsuleLength);
        if (!pole->geometry) {
            dBodyDestroy(pole->body);
            throw std::runtime_error("Failed to create map pole collision geometry");
        }
        dGeomSetBody(pole->geometry, pole->body);
        setPoleTransform(*pole);
        streamedTile.poleGeometries.push_back(pole->geometry);
        streamedTile.poleBodies.push_back(pole->body);
        mapPoles.emplace(pole->geometry, std::move(pole));
    }

    bool isPole(dGeomID geometry) const {
        return mapPoles.contains(geometry);
    }

    void recordPoleImpact(dGeomID geometry, dBodyID vehicleBody, const dContactGeom& contact) {
        const auto pole = mapPoles.find(geometry);
        if (pole == mapPoles.end() || pole->second->broken) {
            return;
        }
        poleContactsCurrent.insert(geometry);
        const dReal* velocity = dBodyGetLinearVel(vehicleBody);
        const double normalSpeed =
            std::abs(velocity[0] * contact.normal[0] + velocity[1] * contact.normal[1] +
                     velocity[2] * contact.normal[2]);
        PoleImpact& impact = poleImpactsCurrent[geometry];
        if (normalSpeed > impact.speedMetersPerSecond) {
            impact.vehicleBody = vehicleBody;
            impact.speedMetersPerSecond = normalSpeed;
            impact.velocity = {velocity[0], velocity[1], velocity[2]};
        }
    }

    void applyPoleImpacts() {
        for (const auto& [geometry, impact] : poleImpactsCurrent) {
            if (poleContactsPrevious.contains(geometry) || impact.speedMetersPerSecond < 0.5) {
                continue;
            }
            const auto found = mapPoles.find(geometry);
            if (found == mapPoles.end()) {
                continue;
            }
            PoleRuntime& pole = *found->second;
            const double horizontalSpeed = std::hypot(impact.velocity[0], impact.velocity[1]);
            if (horizontalSpeed <= 1.0e-8) {
                continue;
            }
            const std::array<double, 3> direction = {impact.velocity[0] / horizontalSpeed,
                                                     impact.velocity[1] / horizontalSpeed, 0.0};
            std::array<double, 3> rotationAxis = {
                pole.up[1] * direction[2] - pole.up[2] * direction[1],
                pole.up[2] * direction[0] - pole.up[0] * direction[2],
                pole.up[0] * direction[1] - pole.up[1] * direction[0]};
            const double axisLength = std::hypot(rotationAxis[0], rotationAxis[1], rotationAxis[2]);
            if (axisLength <= 1.0e-8) {
                continue;
            }
            if (pole.targetAngleRadians <= 1.0e-8) {
                pole.rotationAxis = {rotationAxis[0] / axisLength, rotationAxis[1] / axisLength,
                                     rotationAxis[2] / axisLength};
            }
            dMass mass;
            dBodyGetMass(impact.vehicleBody, &mass);
            const double momentum = std::max(0.0, mass.mass / 1000.0) * impact.speedMetersPerSecond;
            const double addedAngle =
                pole.definition.resistanceRadiansPerTonneMeterPerSecond * momentum;
            const double impactAngle =
                std::min(pole.definition.breakAngleRadians, pole.targetAngleRadians + addedAngle);
            pole.targetAngleRadians = std::max(pole.targetAngleRadians, impactAngle);
            if (impactAngle >= pole.definition.breakAngleRadians) {
                pole.targetBroken = true;
                pole.targetAngleRadians = std::max(pole.targetAngleRadians, pi * 0.5);
            }
        }
    }

    void updatePoleBends(double elapsedSeconds) {
        const double maximumAngleChange = poleBendSpeedRadiansPerSecond * elapsedSeconds;
        for (auto& [geometry, pole] : mapPoles) {
            static_cast<void>(geometry);
            if (pole->broken || pole->angleRadians >= pole->targetAngleRadians) {
                continue;
            }
            pole->angleRadians =
                std::min(pole->targetAngleRadians, pole->angleRadians + maximumAngleChange);
            setPoleTransform(*pole);
            if (pole->targetBroken && pole->angleRadians >= pole->targetAngleRadians) {
                pole->broken = true;
                dGeomDisable(pole->geometry);
            }
        }
    }

    void addRoadBox(dReal centerX, dReal centerY, dReal length, dReal width, dReal height,
                    dReal bottomZ = 0.0) {
        if (length <= 0.0 || width <= 0.0 || height <= 0.0) {
            return;
        }
        const dGeomID geometry = dCreateBox(ode.space, length, width, height);
        if (!geometry) {
            throw std::runtime_error("Failed to create road collision box");
        }
        dGeomSetPosition(geometry, centerX, centerY, bottomZ + height * 0.5);
        roadGeometries.push_back(geometry);
    }

    void addRoadIncline(const RoadBump& feature) {
        constexpr int verticesPerStation = 4;
        const int stationCount = roadInclineSegments + 1;
        std::vector<double> vertices;
        std::vector<int> indices;
        vertices.reserve(static_cast<std::size_t>(stationCount) * verticesPerStation * 3);
        indices.reserve(roadInclineSegments * 8 * 3 + 12);
        const dReal halfWidth = static_cast<dReal>(feature.width) * 0.5;
        const dReal segmentLength = static_cast<dReal>(feature.length) / roadInclineSegments;
        const dReal startX =
            static_cast<dReal>(feature.centerX) - static_cast<dReal>(feature.length) * 0.5;
        for (int station = 0; station < stationCount; ++station) {
            const dReal localX = segmentLength * static_cast<dReal>(station) -
                                 static_cast<dReal>(feature.length) * 0.5;
            const dReal x = startX + segmentLength * station;
            const dReal height =
                static_cast<dReal>(roadFeatureHeightAt(feature, static_cast<double>(localX)));
            const std::array<std::array<dReal, 3>, 4> points = {
                std::array<dReal, 3>{x, static_cast<dReal>(feature.centerY) - halfWidth, height},
                std::array<dReal, 3>{x, static_cast<dReal>(feature.centerY) + halfWidth, height},
                std::array<dReal, 3>{x, static_cast<dReal>(feature.centerY) - halfWidth, 0.0},
                std::array<dReal, 3>{x, static_cast<dReal>(feature.centerY) + halfWidth, 0.0}};
            for (const auto& point : points) {
                vertices.push_back(static_cast<double>(point[0]));
                vertices.push_back(static_cast<double>(point[1]));
                vertices.push_back(static_cast<double>(point[2]));
            }
        }
        const auto triangle = [&](int a, int b, int c) {
            indices.insert(indices.end(), {a, b, c});
        };
        for (int segment = 0; segment < roadInclineSegments; ++segment) {
            const int current = segment * verticesPerStation;
            const int next = (segment + 1) * verticesPerStation;
            triangle(current, next, next + 1);
            triangle(current, next + 1, current + 1);
            triangle(current, current + 2, next + 2);
            triangle(current, next + 2, next);
            triangle(current + 1, next + 1, next + 3);
            triangle(current + 1, next + 3, current + 3);
            triangle(current + 2, current + 3, next + 3);
            triangle(current + 2, next + 3, next + 2);
        }
        triangle(0, 1, 3);
        triangle(0, 3, 2);
        const int finalStation = roadInclineSegments * verticesPerStation;
        triangle(finalStation, finalStation + 2, finalStation + 3);
        triangle(finalStation, finalStation + 3, finalStation + 1);
        addRoadTriangleMesh(std::move(vertices), std::move(indices),
                            "Failed to create incline collision mesh");
    }

    void addRoadBumpMesh(const RoadBump& feature) {
        constexpr int verticesPerStation = 4;
        const int stationCount = roadBumpSegments + 1;
        const dReal halfWidth = static_cast<dReal>(feature.width) * 0.5;
        const dReal segmentLength = static_cast<dReal>(feature.length) / roadBumpSegments;
        const dReal startX =
            static_cast<dReal>(feature.centerX) - static_cast<dReal>(feature.length) * 0.5;
        std::vector<double> vertices;
        std::vector<int> indices;
        vertices.reserve(stationCount * verticesPerStation * 3);
        indices.reserve(roadBumpSegments * 8 * 3 + 12);
        for (int station = 0; station < stationCount; ++station) {
            const dReal localX = segmentLength * station - feature.length * 0.5;
            const dReal height =
                static_cast<dReal>(roadFeatureHeightAt(feature, static_cast<double>(localX)));
            const dReal x = startX + segmentLength * station;
            const dReal yNegative = static_cast<dReal>(feature.centerY) - halfWidth;
            const dReal yPositive = static_cast<dReal>(feature.centerY) + halfWidth;
            for (const std::array<dReal, 3>& point : {std::array<dReal, 3>{x, yNegative, height},
                                                      std::array<dReal, 3>{x, yPositive, height},
                                                      std::array<dReal, 3>{x, yNegative, 0.0},
                                                      std::array<dReal, 3>{x, yPositive, 0.0}}) {
                vertices.push_back(static_cast<double>(point[0]));
                vertices.push_back(static_cast<double>(point[1]));
                vertices.push_back(static_cast<double>(point[2]));
            }
        }
        const auto triangle = [&](int a, int b, int c) {
            indices.insert(indices.end(), {a, b, c});
        };
        for (int segment = 0; segment < roadBumpSegments; ++segment) {
            const int current = segment * verticesPerStation;
            const int next = (segment + 1) * verticesPerStation;
            triangle(current, next, next + 1);
            triangle(current, next + 1, current + 1);
            triangle(current, current + 2, next + 2);
            triangle(current, next + 2, next);
            triangle(current + 1, next + 1, next + 3);
            triangle(current + 1, next + 3, current + 3);
            triangle(current + 2, current + 3, next + 3);
            triangle(current + 2, next + 3, next + 2);
        }
        triangle(0, 1, 3);
        triangle(0, 3, 2);
        const int last = roadBumpSegments * verticesPerStation;
        triangle(last, last + 2, last + 3);
        triangle(last, last + 3, last + 1);
        addRoadTriangleMesh(std::move(vertices), std::move(indices),
                            "Failed to create road bump collision mesh");
    }

    void addRoadTriangleMesh(std::vector<double> vertices, std::vector<int> indices,
                             const char* errorMessage) {
        roadMeshVertices.push_back(std::move(vertices));
        roadMeshIndices.push_back(std::move(indices));
        const auto& storedVertices = roadMeshVertices.back();
        const auto& storedIndices = roadMeshIndices.back();
        const dTriMeshDataID meshData = dGeomTriMeshDataCreate();
        if (!meshData) {
            roadMeshVertices.pop_back();
            roadMeshIndices.pop_back();
            throw std::runtime_error(errorMessage);
        }
        dGeomTriMeshDataBuildDouble(
            meshData, storedVertices.data(), 3 * sizeof(double),
            checkedOdeCount(storedVertices.size() / 3, "road feature vertex count"),
            storedIndices.data(), checkedOdeCount(storedIndices.size(), "road feature index count"),
            3 * sizeof(int));
        const dGeomID geometry = dCreateTriMesh(ode.space, meshData, nullptr, nullptr, nullptr);
        if (!geometry) {
            dGeomTriMeshDataDestroy(meshData);
            roadMeshVertices.pop_back();
            roadMeshIndices.pop_back();
            throw std::runtime_error(errorMessage);
        }
        roadMeshData.push_back(meshData);
        roadMeshGeometries.push_back(geometry);
    }

    void addDefaultRoadFeatures() {
        for (const RoadBump& feature : defaultRoadBumps()) {
            if (feature.type == RoadFeatureType::Barrier) {
                addRoadBox(feature.centerX, feature.centerY, feature.length, feature.width,
                           feature.height);
            } else if (feature.type == RoadFeatureType::Bridge) {
                const dReal deckLength = std::clamp(static_cast<dReal>(feature.flatLength), 0.0,
                                                    static_cast<dReal>(feature.length));
                const int rampSegments = 6;
                const dReal rampLength = (static_cast<dReal>(feature.length) - deckLength) * 0.5;
                const dReal segmentLength = rampLength / rampSegments;
                const dReal startX = feature.centerX - feature.length * 0.5;
                for (int segment = 0; segment < rampSegments; ++segment) {
                    const dReal phase = (static_cast<dReal>(segment) + 0.5) / rampSegments;
                    addRoadBox(startX + segmentLength * (segment + 0.5), feature.centerY,
                               segmentLength, feature.width, feature.height * phase);
                }
                addRoadBox(feature.centerX, feature.centerY, deckLength, feature.width,
                           feature.height);
                const dReal downStartX = feature.centerX + feature.length * 0.5 - rampLength;
                for (int segment = 0; segment < rampSegments; ++segment) {
                    const dReal phase = (static_cast<dReal>(segment) + 0.5) / rampSegments;
                    addRoadBox(downStartX + segmentLength * (segment + 0.5), feature.centerY,
                               segmentLength, feature.width, feature.height * (1.0 - phase));
                }
                if (feature.railHeight > 0.0 && feature.railWidth > 0.0) {
                    const dReal railY = feature.width * 0.5 - feature.railWidth * 0.5;
                    addRoadBox(feature.centerX, feature.centerY + railY, feature.length,
                               feature.railWidth, feature.railHeight, feature.height);
                    addRoadBox(feature.centerX, feature.centerY - railY, feature.length,
                               feature.railWidth, feature.railHeight, feature.height);
                }
            } else if (feature.type == RoadFeatureType::Incline) {
                addRoadIncline(feature);
            } else {
                addRoadBumpMesh(feature);
            }
        }
    }
};

World::World(double groundPlaneZ, std::vector<TerrainCollisionGrid> terrain,
             std::vector<StaticCollisionMesh> staticCollision, bool createDefaultGround)
    : impl_(std::make_unique<Impl>()) {
    if (!std::isfinite(groundPlaneZ)) {
        throw std::invalid_argument("World ground plane must be finite");
    }
    if (terrain.empty() && createDefaultGround) {
        impl_->ground = dCreatePlane(impl_->ode.space, 0.0, 0.0, 1.0, groundPlaneZ);
        if (!impl_->ground) {
            throw std::runtime_error("Failed to create world ground plane");
        }
        impl_->addDefaultRoadFeatures();
    } else {
        for (const TerrainCollisionGrid& tile : terrain) {
            impl_->initialTerrainTileKeys.insert(mapTileKey(tile.tileX, tile.tileY));
            impl_->addTerrainCollision(tile);
        }
    }
    for (StaticCollisionMesh& mesh : staticCollision) {
        impl_->addStaticCollision(std::move(mesh));
    }
}

World::~World() = default;

void World::updateOnFoot(OnFootState& state, double elapsedSeconds, double yawRadians,
                         double forward, double right, bool jump) const {
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0 || !std::isfinite(yawRadians) ||
        !std::isfinite(forward) || !std::isfinite(right) ||
        !std::isfinite(state.verticalVelocity) ||
        !std::all_of(
            state.feetPosition.begin(), state.feetPosition.end(),
            [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("On-foot movement contains non-finite or invalid values");
    }
    constexpr double radius = 0.3;
    constexpr double height = 1.8;
    constexpr double speed = 1.5;
    const double duration = std::min(elapsedSeconds, 0.1);
    if (duration == 0.0) {
        return;
    }
    const double inputLength = std::max(1.0, std::hypot(forward, right));
    const double velocityX =
        speed * (std::cos(yawRadians) * forward + std::sin(yawRadians) * right) / inputLength;
    const double velocityY =
        speed * (std::sin(yawRadians) * forward - std::cos(yawRadians) * right) / inputLength;
    if (jump && state.grounded) {
        state.verticalVelocity = 5.0;
        state.grounded = false;
    }
    const int steps = static_cast<int>(std::ceil(duration * 120.0));
    const double dt = duration / steps;
    const dGeomID capsule = dCreateCapsule(nullptr, radius, height - 2.0 * radius);
    if (capsule == nullptr) {
        throw std::runtime_error("Could not create on-foot collision capsule");
    }
    struct ContactQuery {
        dGeomID capsule;
        dContactGeom deepest = {};
        bool hit = false;
    };
    const auto callback = +[](void* data, dGeomID first, dGeomID second) {
        auto& query = *static_cast<ContactQuery*>(data);
        const dGeomID other = first == query.capsule ? second : first;
        if (dGeomGetBody(other) != nullptr) {
            return;
        }
        dContactGeom contacts[16] = {};
        const int count = dCollide(query.capsule, other, 16, contacts, sizeof(dContactGeom));
        for (int index = 0; index < count; ++index) {
            if (!query.hit || contacts[index].depth > query.deepest.depth) {
                query.deepest = contacts[index];
                query.hit = true;
            }
        }
    };
    try {
        for (int step = 0; step < steps; ++step) {
            state.verticalVelocity += gravity * dt;
            state.feetPosition[0] += velocityX * dt;
            state.feetPosition[1] += velocityY * dt;
            state.feetPosition[2] += state.verticalVelocity * dt;
            state.grounded = false;
            for (int iteration = 0; iteration < 12; ++iteration) {
                dGeomSetPosition(capsule, state.feetPosition[0], state.feetPosition[1],
                                 state.feetPosition[2] + height * 0.5);
                ContactQuery query{capsule};
                dSpaceCollide2(capsule, reinterpret_cast<dGeomID>(impl_->ode.space), &query,
                               callback);
                if (!query.hit) {
                    break;
                }
                for (std::size_t axis = 0; axis < 3; ++axis) {
                    state.feetPosition[axis] +=
                        query.deepest.normal[axis] * (query.deepest.depth + 1.0e-6);
                }
                if (query.deepest.normal[2] > 0.5 && state.verticalVelocity <= 0.0) {
                    state.grounded = true;
                    state.verticalVelocity = 0.0;
                } else if (query.deepest.normal[2] < -0.5 && state.verticalVelocity > 0.0) {
                    state.verticalVelocity = 0.0;
                }
            }
        }
    } catch (...) {
        dGeomDestroy(capsule);
        throw;
    }
    dGeomDestroy(capsule);
}

void World::addMapTileCollision(int tileX, int tileY, std::optional<TerrainCollisionGrid> terrain,
                                std::vector<StaticCollisionMesh> staticCollision,
                                std::vector<MapPoleCollision> poles) {
    const std::uint64_t key = mapTileKey(tileX, tileY);
    const auto [tile, inserted] = impl_->streamedTiles.try_emplace(key);
    if (!inserted) {
        throw std::invalid_argument("Map tile collision is already active");
    }
    try {
        if (terrain.has_value()) {
            if (terrain->tileX != tileX || terrain->tileY != tileY) {
                throw std::invalid_argument("Map terrain collision tile coordinates do not match");
            }
            impl_->addTerrainCollision(*terrain, &tile->second);
        }
        for (StaticCollisionMesh& mesh : staticCollision) {
            impl_->addStaticCollision(std::move(mesh), &tile->second);
        }
        for (const MapPoleCollision& pole : poles) {
            impl_->addPoleCollision(pole, tile->second);
        }
    } catch (...) {
        for (dGeomID geometry : tile->second.poleGeometries) {
            impl_->mapPoles.erase(geometry);
            dGeomDestroy(geometry);
        }
        for (dBodyID body : tile->second.poleBodies) {
            dBodyDestroy(body);
        }
        Impl::destroyMeshes(tile->second.geometries, tile->second.meshData);
        impl_->streamedTiles.erase(tile);
        throw;
    }
    ++impl_->collisionRevision;
}

void World::addMapTileStaticCollision(int tileX, int tileY,
                                      std::vector<StaticCollisionMesh> staticCollision,
                                      std::vector<MapPoleCollision> poles) {
    const auto tile = impl_->streamedTiles.find(mapTileKey(tileX, tileY));
    if (tile == impl_->streamedTiles.end()) {
        throw std::invalid_argument("Map tile collision must be published before static collision");
    }
    const std::size_t initialGeometryCount = tile->second.geometries.size();
    const std::size_t initialMeshCount = tile->second.meshData.size();
    const std::size_t initialStaticCount = tile->second.staticMeshes.size();
    const std::size_t initialPoleCount = tile->second.poleGeometries.size();
    try {
        for (StaticCollisionMesh& mesh : staticCollision) {
            impl_->addStaticCollision(std::move(mesh), &tile->second);
        }
        for (const MapPoleCollision& pole : poles) {
            impl_->addPoleCollision(pole, tile->second);
        }
    } catch (...) {
        while (tile->second.poleGeometries.size() > initialPoleCount) {
            const dGeomID geometry = tile->second.poleGeometries.back();
            tile->second.poleGeometries.pop_back();
            impl_->mapPoles.erase(geometry);
            dGeomDestroy(geometry);
            const dBodyID body = tile->second.poleBodies.back();
            tile->second.poleBodies.pop_back();
            dBodyDestroy(body);
        }
        while (tile->second.geometries.size() > initialGeometryCount) {
            dGeomDestroy(tile->second.geometries.back());
            tile->second.geometries.pop_back();
        }
        while (tile->second.meshData.size() > initialMeshCount) {
            dGeomTriMeshDataDestroy(tile->second.meshData.back());
            tile->second.meshData.pop_back();
        }
        tile->second.staticMeshes.resize(initialStaticCount);
        throw;
    }
    ++impl_->collisionRevision;
}

void World::addMapTileTerrainCollision(int tileX, int tileY, TerrainCollisionGrid terrain) {
    if (terrain.tileX != tileX || terrain.tileY != tileY) {
        throw std::invalid_argument("Map terrain collision tile coordinates do not match");
    }
    auto tile = impl_->streamedTiles.find(mapTileKey(tileX, tileY));
    if (tile == impl_->streamedTiles.end()) {
        addMapTileCollision(tileX, tileY, std::move(terrain), {});
        return;
    }
    impl_->addTerrainCollision(terrain, &tile->second);
    ++impl_->collisionRevision;
}

void World::removeMapTileCollision(int tileX, int tileY) {
    const auto tile = impl_->streamedTiles.find(mapTileKey(tileX, tileY));
    if (tile == impl_->streamedTiles.end()) {
        return;
    }
    for (dGeomID geometry : tile->second.poleGeometries) {
        impl_->mapPoles.erase(geometry);
        impl_->poleContactsPrevious.erase(geometry);
        impl_->poleContactsCurrent.erase(geometry);
        impl_->poleImpactsCurrent.erase(geometry);
        dGeomDestroy(geometry);
    }
    for (dBodyID body : tile->second.poleBodies) {
        dBodyDestroy(body);
    }
    Impl::destroyMeshes(tile->second.geometries, tile->second.meshData);
    impl_->streamedTiles.erase(tile);
    ++impl_->collisionRevision;
}

std::uint64_t World::collisionDebugRevision() const {
    return impl_->collisionRevision;
}

std::size_t World::loadedMapCollisionTileCount() const {
    std::size_t count = impl_->initialTerrainTileKeys.size();
    for (const auto& [key, tile] : impl_->streamedTiles) {
        static_cast<void>(tile);
        if (!impl_->initialTerrainTileKeys.contains(key)) {
            ++count;
        }
    }
    return count;
}

std::vector<StaticCollisionMesh> World::collisionDebugMeshes() const {
    std::vector<StaticCollisionMesh> meshes;
    meshes.reserve(impl_->terrainVertices.size() + impl_->staticMeshes.size() +
                   impl_->streamedTiles.size());
    for (std::size_t index = 0; index < impl_->terrainVertices.size(); ++index) {
        meshes.push_back({impl_->terrainVertices[index], impl_->terrainIndices[index]});
    }
    meshes.insert(meshes.end(), impl_->staticMeshes.begin(), impl_->staticMeshes.end());
    for (const auto& [key, tile] : impl_->streamedTiles) {
        static_cast<void>(key);
        for (std::size_t index = 0; index < tile.terrainVertices.size(); ++index) {
            meshes.push_back({tile.terrainVertices[index], tile.terrainIndices[index]});
        }
        meshes.insert(meshes.end(), tile.staticMeshes.begin(), tile.staticMeshes.end());
    }
    return meshes;
}

std::vector<MapPoleState> World::mapPoleStates() const {
    std::vector<MapPoleState> states;
    states.reserve(impl_->mapPoles.size());
    for (const auto& [geometry, pole] : impl_->mapPoles) {
        static_cast<void>(geometry);
        states.push_back({pole->definition.mapObjectId, pole->definition.tileX,
                          pole->definition.tileY, pole->rotationAxis, pole->angleRadians,
                          pole->broken});
    }
    std::sort(states.begin(), states.end(),
              [](const MapPoleState& left, const MapPoleState& right) {
                  return std::pair{left.tileX, left.mapObjectId} <
                         std::pair{right.tileX, right.mapObjectId};
              });
    return states;
}

dWorldID World::physicsWorld() const {
    return impl_->ode.world;
}
dSpaceID World::collisionSpace() const {
    return impl_->ode.space;
}
dJointGroupID World::contactGroup() const {
    return impl_->ode.contacts;
}

bool World::isMapPoleGeometry(dGeomID geometry) const {
    return impl_->isPole(geometry);
}

void World::recordMapPoleImpact(dGeomID geometry, dBodyID vehicleBody,
                                const dContactGeom& contact) {
    impl_->recordPoleImpact(geometry, vehicleBody, contact);
}

void World::collideAndStep(dReal fixedStep, void* context, dNearCallback callback) {
    impl_->poleContactsCurrent.clear();
    impl_->poleImpactsCurrent.clear();
    dSpaceCollide(impl_->ode.space, context, callback);
    impl_->applyPoleImpacts();
    impl_->updatePoleBends(fixedStep);
    dWorldStep(impl_->ode.world, fixedStep);
    dJointGroupEmpty(impl_->ode.contacts);
    impl_->poleContactsPrevious = impl_->poleContactsCurrent;
}
