#pragma once

#include "core/BusTypes.h"

#include <ode/ode.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

struct OnFootState {
    std::array<double, 3> feetPosition = {};
    double verticalVelocity = 0.0;
    bool grounded = false;
};

class World {
  public:
    explicit World(double groundPlaneZ = 0.0, std::vector<TerrainCollisionGrid> terrain = {},
                   std::vector<StaticCollisionMesh> staticCollision = {},
                   bool createDefaultGround = true);
    ~World();

    World(const World&) = delete;
    World& operator=(const World&) = delete;

    void updateOnFoot(OnFootState& state, double elapsedSeconds, double yawRadians, double forward,
                      double right, bool jump) const;
    std::optional<double> raycastStatic(const std::array<double, 3>& origin,
                                        const std::array<double, 3>& direction,
                                        double maxDistance) const;
    void addMapTileCollision(int tileX, int tileY, std::optional<TerrainCollisionGrid> terrain,
                             std::vector<StaticCollisionMesh> staticCollision,
                             std::vector<MapPoleCollision> poles = {});
    void addMapTileTerrainCollision(int tileX, int tileY, TerrainCollisionGrid terrain);
    void addMapTileStaticCollision(int tileX, int tileY,
                                   std::vector<StaticCollisionMesh> staticCollision,
                                   std::vector<MapPoleCollision> poles = {});
    void removeMapTileCollision(int tileX, int tileY);
    std::uint64_t collisionDebugRevision() const;
    std::size_t loadedMapCollisionTileCount() const;
    std::vector<StaticCollisionMesh> collisionDebugMeshes() const;
    std::vector<MapPoleState> mapPoleStates() const;

  public:
    struct Impl;
    dWorldID physicsWorld() const;
    dSpaceID collisionSpace() const;
    dJointGroupID contactGroup() const;
    bool isMapPoleGeometry(dGeomID geometry) const;
    void recordMapPoleContact(dGeomID geometry);
    void recordMapPoleImpact(dGeomID geometry, dBodyID vehicleBody, const dContactGeom& contact);
    void collideAndStep(dReal fixedStep, void* context, dNearCallback callback);

  private:
    std::unique_ptr<Impl> impl_;
    friend class BusSimulation;
};
