#include "MapCollisionStreamer.h"

#include "BusSimulation.h"
#include "MapCollisionBuilder.h"
#include "MapTileWindow.h"

#include <algorithm>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <exception>
#include <iostream>
#include <mutex>
#include <string>
#include <stdexcept>
#include <thread>
#include <unordered_set>
#include <utility>

namespace openbus::map {
namespace {

std::uint64_t tileKey(int x, int y) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
           static_cast<std::uint32_t>(y);
}

std::string tileLabel(int x, int y) {
    return "tile (" + std::to_string(x) + "," + std::to_string(y) + ")";
}

} // namespace

struct MapCollisionStreamer::WorkerState {
    struct Job {
        std::uint64_t key = 0;
        int tileX = 0;
        int tileY = 0;
    };

    struct Result {
        std::uint64_t key = 0;
        int tileX = 0;
        int tileY = 0;
        std::optional<TerrainCollisionGrid> terrain;
        std::vector<StaticCollisionMesh> meshes;
        std::exception_ptr error;
    };

    WorkerState(const MapDefinition& mapDefinition, std::filesystem::path root)
        : map(mapDefinition), omsiRoot(std::move(root)) {
        thread = std::thread([this] { run(); });
    }

    ~WorkerState() {
        {
            std::lock_guard<std::mutex> lock(mutex);
            stopping = true;
            pending.clear();
        }
        condition.notify_all();
        if (thread.joinable()) {
            thread.join();
        }
    }

    void run() {
        while (true) {
            Job job;
            {
                std::unique_lock<std::mutex> lock(mutex);
                condition.wait(lock, [this] { return stopping || !pending.empty(); });
                if (stopping) {
                    break;
                }
                job = pending.front();
                pending.pop_front();
                if (!desiredKeys.contains(job.key)) {
                    scheduledKeys.erase(job.key);
                    condition.notify_all();
                    continue;
                }
                busy = true;
            }

            Result result;
            result.key = job.key;
            result.tileX = job.tileX;
            result.tileY = job.tileY;
            try {
                const auto tile =
                    std::find_if(map.tiles.begin(), map.tiles.end(), [&](const auto& ref) {
                        return ref.x == job.tileX && ref.y == job.tileY;
                    });
                if (tile == map.tiles.end()) {
                    throw std::runtime_error("Collision request references a missing map tile");
                }
                if (tile->hasTerrainFile) {
                    const TerrainGrid grid = loadTerrainGrid(tile->terrainPath);
                    result.terrain = TerrainCollisionGrid{
                        job.tileX, job.tileY, OMSI_TILE_SIZE_METERS, grid.intervals, grid.heights};
                }
                MapRoadCollisionResult roadCollision = buildMapRoadCollisionTile(
                    map, omsiRoot, job.tileX, job.tileY, &roadConnections());
                result.meshes = std::move(roadCollision.meshes);
            } catch (...) {
                result.error = std::current_exception();
            }

            {
                std::lock_guard<std::mutex> lock(mutex);
                completed.push_back(std::move(result));
                busy = false;
            }
            condition.notify_all();
        }
    }

    const MapRoadConnectionIndex& roadConnections() {
        if (!connections) {
            connections = loadMapRoadConnections(map, omsiRoot);
        }
        return *connections;
    }

    const MapDefinition& map;
    std::filesystem::path omsiRoot;
    // Only accessed by this worker thread; never conditioned on published collision tiles.
    std::optional<MapRoadConnectionIndex> connections;
    std::mutex mutex;
    std::condition_variable condition;
    std::deque<Job> pending;
    std::deque<Result> completed;
    std::unordered_set<std::uint64_t> desiredKeys;
    std::unordered_set<std::uint64_t> scheduledKeys;
    bool busy = false;
    bool stopping = false;
    std::thread thread;
};

MapCollisionStreamer::MapCollisionStreamer(const MapDefinition& map,
                                           const std::filesystem::path& omsiRoot,
                                           BusSimulation& simulation, int loadRadius,
                                           int unloadRadius)
    : map_(map), omsiRoot_(omsiRoot), simulation_(simulation), loadRadius_(loadRadius),
      unloadRadius_(unloadRadius) {
    if (loadRadius_ < 0 || unloadRadius_ < loadRadius_) {
        throw std::invalid_argument("Map collision unload radius must be at least the load radius");
    }
    worker_ = std::make_unique<WorkerState>(map, omsiRoot);
}

MapCollisionStreamer::~MapCollisionStreamer() = default;

void MapCollisionStreamer::update(double worldX, double worldY,
                                  const std::vector<MapTileCoordinate>& loadedGeometryTiles) {
    const int centerX = mapWorldTile(worldX);
    const int centerY = mapWorldTile(worldY);

    std::unordered_set<std::uint64_t> loadedGeometryTileKeys;
    loadedGeometryTileKeys.reserve(loadedGeometryTiles.size());
    for (const auto& [tileX, tileY] : loadedGeometryTiles) {
        loadedGeometryTileKeys.insert(tileKey(tileX, tileY));
    }
    if (lastCenterTile_ != std::pair{centerX, centerY} ||
        lastLoadedGeometryTileKeys_ != loadedGeometryTileKeys) {
        failedTiles_.clear();
    }

    std::unordered_set<std::uint64_t> desiredCollisionKeys;
    desiredCollisionKeys.reserve(loadedGeometryTileKeys.size());
    std::vector<const MapTileReference*> desiredTiles;
    desiredTiles.reserve(loadedGeometryTileKeys.size());
    for (const MapTileReference& tile : map_.tiles) {
        const std::uint64_t key = tileKey(tile.x, tile.y);
        if (loadedGeometryTileKeys.contains(key) &&
            mapTileWithinRadius(tile.x, tile.y, centerX, centerY, loadRadius_)) {
            desiredCollisionKeys.insert(key);
            desiredTiles.push_back(&tile);
        }
    }
    std::sort(desiredTiles.begin(), desiredTiles.end(),
              [centerX, centerY](const auto* left, const auto* right) {
                  const int leftX = std::abs(left->x - centerX);
                  const int leftY = std::abs(left->y - centerY);
                  const int rightX = std::abs(right->x - centerX);
                  const int rightY = std::abs(right->y - centerY);
                  const int leftChebyshev = std::max(leftX, leftY);
                  const int rightChebyshev = std::max(rightX, rightY);
                  if (leftChebyshev != rightChebyshev) {
                      return leftChebyshev < rightChebyshev;
                  }
                  const int leftManhattan = leftX + leftY;
                  const int rightManhattan = rightX + rightY;
                  if (leftManhattan != rightManhattan) {
                      return leftManhattan < rightManhattan;
                  }
                  return std::pair{left->x, left->y} < std::pair{right->x, right->y};
              });

    std::deque<WorkerState::Result> completed;
    {
        std::lock_guard<std::mutex> lock(worker_->mutex);
        worker_->desiredKeys = desiredCollisionKeys;
        for (auto job = worker_->pending.begin(); job != worker_->pending.end();) {
            if (desiredCollisionKeys.contains(job->key)) {
                ++job;
                continue;
            }
            worker_->scheduledKeys.erase(job->key);
            job = worker_->pending.erase(job);
        }
        if (!worker_->completed.empty()) {
            completed.push_back(std::move(worker_->completed.front()));
            worker_->completed.pop_front();
            worker_->scheduledKeys.erase(completed.front().key);
        }
    }

    for (WorkerState::Result& result : completed) {
        if (!desiredCollisionKeys.contains(result.key) ||
            !loadedGeometryTileKeys.contains(result.key)) {
            continue;
        }
        if (result.error != nullptr) {
            try {
                std::rethrow_exception(result.error);
            } catch (const std::exception& error) {
                std::cerr << "Map collision tile build failed at "
                          << tileLabel(result.tileX, result.tileY) << ": " << error.what() << '\n';
            } catch (...) {
                std::cerr << "Map collision tile build failed at "
                          << tileLabel(result.tileX, result.tileY) << " with an unknown error\n";
            }
            failedTiles_.insert(result.key);
            continue;
        }
        try {
            simulation_.addMapTileCollision(result.tileX, result.tileY, std::move(result.terrain),
                                            std::move(result.meshes));
            loadedTiles_.emplace(result.key, std::pair{result.tileX, result.tileY});
        } catch (const std::exception& error) {
            std::cerr << "Map collision tile publication failed at "
                      << tileLabel(result.tileX, result.tileY) << ": " << error.what() << '\n';
            failedTiles_.insert(result.key);
        }
    }

    for (const MapTileReference* tile : desiredTiles) {
        const std::uint64_t key = tileKey(tile->x, tile->y);
        if (loadedTiles_.contains(key) || failedTiles_.contains(key)) {
            continue;
        }
        std::lock_guard<std::mutex> lock(worker_->mutex);
        if (!worker_->scheduledKeys.insert(key).second) {
            continue;
        }
        worker_->pending.push_back({key, tile->x, tile->y});
    }
    worker_->condition.notify_one();

    for (auto tile = loadedTiles_.begin(); tile != loadedTiles_.end();) {
        const auto [tileX, tileY] = tile->second;
        if (loadedGeometryTileKeys.contains(tile->first) &&
            mapTileWithinRadius(tileX, tileY, centerX, centerY, unloadRadius_)) {
            ++tile;
            continue;
        }
        simulation_.removeMapTileCollision(tileX, tileY);
        tile = loadedTiles_.erase(tile);
    }
    lastCenterTile_ = std::pair{centerX, centerY};
    lastLoadedGeometryTileKeys_ = std::move(loadedGeometryTileKeys);
}

void MapCollisionStreamer::waitForPendingBuilds() {
    std::unique_lock<std::mutex> lock(worker_->mutex);
    worker_->condition.wait(lock, [this] { return worker_->pending.empty() && !worker_->busy; });
}

std::size_t MapCollisionStreamer::loadedTileCount() const {
    return loadedTiles_.size();
}

} // namespace openbus::map
