#include "config/SceneryObjectConfigLoader.h"
#include "ProbeTempDirectory.h"

#include "map/MapSplineGeometry.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

namespace {

bool require(bool condition, const char* description) {
    if (!condition) {
        std::cerr << "failed: " << description << '\n';
        return false;
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    if (argc == 2) {
        const SceneryObjectConfig configuration = loadSceneryObjectFile(argv[1]);
        bool valid = require(configuration.trafficPaths.size() == 40,
                             "Bath University A SCO paths parse");
        valid &= require(configuration.onlyEditor,
                         "Bath University A is marked editor-only");
        valid &= require(configuration.absoluteHeight,
                         "Bath University A uses absolute height");
        if (configuration.trafficPaths.size() == 40) {
            const SceneryObjectTrafficPath& curve = configuration.trafficPaths[0];
            const SceneryObjectTrafficPath& continuation = configuration.trafficPaths[3];
            const auto samples = openbus::map::tessellateMapSpline(
                curve.startPosition[0], curve.startPosition[1], curve.headingDegrees,
                curve.length, curve.radius);
            valid &= require(!samples.empty() &&
                                 std::hypot(samples.back().x - continuation.startPosition[0],
                                            samples.back().y - continuation.startPosition[1]) <
                                     0.01,
                             "Bath University A curved path ends at its continuation");
        }
        return valid ? 0 : 1;
    }
    if (argc != 1) {
        return 2;
    }

    const std::filesystem::path directory =
        openbus::probe::makeProbeDirectory("openbus-scenery-object-config-probe");
    const std::filesystem::path path = directory / "paths.sco";
    {
        std::ofstream output(path);
        if (!output) {
            std::cerr << "failed: create scenery object fixture\n";
            openbus::probe::removeProbeDirectory(directory);
            return 1;
        }
        output << "[mesh]\nmesh.x\n"
                  "[path]\n"
                  "1\n2\n3\n4\n5\n6\n7\n8\n0\n2.5\n0\n0\n"
                  "[path]\n"
                  "-1\n-2\n-3\n-4\n-5\n-6\n-7\n-8\n1\n1\n2\n0\n";
    }
    const SceneryObjectConfig configuration = loadSceneryObjectFile(path);
    openbus::probe::removeProbeDirectory(directory);

    bool valid = require(configuration.trafficPaths.size() == 2, "path sections parsed");
    if (configuration.trafficPaths.size() == 2) {
        valid &= require(configuration.trafficPaths[0].startPosition ==
                             std::array<double, 3>{1.0, 2.0, 3.0},
                         "first path start position");
        valid &= require(configuration.trafficPaths[0].headingDegrees == 4.0 &&
                             configuration.trafficPaths[0].radius == 5.0 &&
                             configuration.trafficPaths[0].length == 6.0 &&
                             configuration.trafficPaths[0].gradientStartPercent == 7.0 &&
                             configuration.trafficPaths[0].gradientEndPercent == 8.0 &&
                             configuration.trafficPaths[0].widthMeters == 2.5,
                         "path spline fields are preserved");
        valid &= require(configuration.trafficPaths[0].pathTypeCode == 0.0 &&
                             configuration.trafficPaths[1].pathTypeCode == 1.0,
                         "path categories preserved");
        valid &= require(configuration.diagnostics.entries.empty(),
                         "path sections produce no parser diagnostics");
    }
    return valid ? 0 : 1;
}
