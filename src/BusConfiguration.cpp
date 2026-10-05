#include "BusConfiguration.h"

#include "BusConfigLoader.h"
#include "ConfigurationParser.h"
#include "Environment.h"
#include "ModelConfigLoader.h"
#include "O3DLoader.h"
#include "ObjLoader.h"
#include "PerfTrace.h"
#include "Variables.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

constexpr double DEFAULT_WHEEL_HALF_WIDTH = 0.145;
constexpr const char* DEFAULT_OMSI_ROOT = R"(C:\Program Files (x86)\Steam\steamapps\common\OMSI 2)";
} // namespace

std::filesystem::path omsiRootPath() {
    if (const char* configuredRoot = openbus::getEnvironment("OPENBUS_OMSI_ROOT");
        configuredRoot != nullptr && *configuredRoot != '\0') {
        return configuredRoot;
    }
    return DEFAULT_OMSI_ROOT;
}

namespace {

std::filesystem::path normalizedPath(const std::filesystem::path& path) {
    std::string value = path.string();
    std::replace(value.begin(), value.end(), '\\', '/');
    return std::filesystem::path(value);
}

std::filesystem::path resolveConfiguredBusPath(const std::filesystem::path& configuredPath) {
    const std::filesystem::path relative = normalizedPath(configuredPath);
    if (relative.is_absolute()) {
        return relative;
    }
    const std::filesystem::path root = omsiRootPath();
    const std::array<std::filesystem::path, 5> candidates = {
        root / relative, relative, std::filesystem::current_path() / relative,
        std::filesystem::current_path().parent_path() / relative,
        std::filesystem::current_path().parent_path().parent_path() / relative};
    for (const auto& candidate : candidates) {
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
    }
    return relative;
}

std::filesystem::path resolveConfiguredModelPath(const std::filesystem::path& busPath,
                                                 const std::filesystem::path& configuredPath) {
    const std::filesystem::path relative = normalizedPath(configuredPath);
    if (relative.is_absolute()) {
        return relative;
    }
    const std::filesystem::path root = omsiRootPath();
    const std::array<std::filesystem::path, 6> candidates = {
        busPath.parent_path() / relative,
        root / relative,
        relative,
        std::filesystem::current_path() / relative,
        std::filesystem::current_path().parent_path() / relative,
        std::filesystem::current_path().parent_path().parent_path() / relative};
    for (const auto& candidate : candidates) {
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
    }
    return relative;
}

std::optional<std::size_t> wheelRotationAxleIndex(const ModelPart& part) {
    constexpr std::string_view prefix = "wheel_rotation_";
    for (const ModelAnimation& animation : part.animations) {
        const std::string variable = openbus::config::lower(animation.variable);
        if (variable.rfind(prefix, 0) != 0) {
            continue;
        }
        const std::size_t sideSeparator = variable.find('_', prefix.size());
        if (sideSeparator == std::string::npos || (variable.substr(sideSeparator + 1) != "l" &&
                                                   variable.substr(sideSeparator + 1) != "r")) {
            continue;
        }
        int axleIndex = -1;
        if (openbus::config::parseInt(
                std::string(variable.substr(prefix.size(), sideSeparator - prefix.size())),
                axleIndex) &&
            axleIndex >= 0) {
            return static_cast<std::size_t>(axleIndex);
        }
    }
    return {};
}

std::shared_ptr<openbus::rendering::ParsedObj> parseModelMesh(const ModelPart& part) {
    if (part.objPath.empty()) {
        return {};
    }
    if (openbus::config::lower(part.objPath.extension().string()) == ".o3d") {
        return openbus::rendering::O3DLoader::parse(part.objPath);
    }
    return openbus::rendering::ObjLoader::parse(part.objPath, part.bundleEntry);
}

double visualWheelRadius(const openbus::rendering::ParsedObj& mesh) {
    double radius = 0.0;
    for (const openbus::rendering::ObjPosition& position : mesh.positions) {
        // Wheel rotation is around render Y; the tire envelope is in render XZ.
        const double radialX = position.y - mesh.boundsCenter[0];
        const double radialZ = position.z - mesh.boundsCenter[2];
        const double vertexRadius = std::hypot(radialX, radialZ);
        if (std::isfinite(vertexRadius)) {
            radius = std::max(radius, vertexRadius);
        }
    }
    return radius;
}

void fitWheelCollidersToModel(BusConfiguration& configuration, const ModelConfig& model) {
    for (const ModelPart& part : model.parts) {
        const std::optional<std::size_t> axleIndex = wheelRotationAxleIndex(part);
        if (!axleIndex || *axleIndex >= configuration.axles.size()) {
            continue;
        }
        const auto mesh = parseModelMesh(part);
        if (!mesh || mesh->positions.empty()) {
            continue;
        }
        const double meshRadius = visualWheelRadius(*mesh);
        BusAxle& axle = configuration.axles[*axleIndex];
        if (std::isfinite(meshRadius) && meshRadius > axle.wheelDiameter * 0.5) {
            axle.wheelDiameter = meshRadius * 2.0;
        }
    }
    configuration.wheelRadius = configuration.axles.front().wheelDiameter * 0.5;
}

BusConfiguration configurationFromVehicleConfig(const VehicleConfig& source,
                                                const ModelConfig& model) {
    const std::array<double, 6>& boundingBox =
        model.hasBoundingBox ? model.boundingBox : source.boundingBox;

    BusConfiguration configuration{};
    configuration.articulated = source.articulated;
    configuration.axles = source.axles;
    configuration.bodyHalfHeight = boundingBox[2] * 0.5;
    configuration.bodyHalfLength = boundingBox[1] * 0.5;
    configuration.bodyHalfWidth = boundingBox[0] * 0.5;
    configuration.centerOfGravityHeight = source.centerOfGravityHeight;
    configuration.collisionHeight = boundingBox[2];
    configuration.collisionLength = boundingBox[1];
    configuration.collisionOffsetX = boundingBox[4];
    configuration.collisionOffsetY = -boundingBox[3];
    configuration.collisionOffsetZ = boundingBox[5] - source.centerOfGravityHeight;
    configuration.outsideCameraCenter = {source.outsideCameraCenter[1],
                                         -source.outsideCameraCenter[0],
                                         source.outsideCameraCenter[2]};
    configuration.hasOutsideCameraCenter = source.hasOutsideCameraCenter;
    configuration.collisionWidth = boundingBox[0];
    configuration.inverseMinimumTurnRadius = source.inverseMinimumTurnRadius;
    configuration.length = boundingBox[1];
    configuration.mass = source.massTonnes * 1000.0;
    configuration.momentOfInertia = source.momentOfInertia;
    configuration.wheelHalfWidth = source.wheelHalfWidth;
    configuration.wheelRadius = source.axles.front().wheelDiameter * 0.5;
    configuration.width = boundingBox[0];
    fitWheelCollidersToModel(configuration, model);

    for (const ModelCollisionMesh& collisionMesh : model.collisionMeshes) {
        if (collisionMesh.hasPart) {
            if (collisionMesh.partIndex >= model.parts.size()) {
                throw std::runtime_error("Bus model collision mesh refers to an invalid mesh part");
            }
            const ModelPart& owner = model.parts[collisionMesh.partIndex];
            if (owner.isShadow || owner.noCollision) {
                continue;
            }
        }
        if (collisionMesh.resolvedPath.empty()) {
            throw std::runtime_error("Bus model collision mesh was not found: " +
                                     collisionMesh.sourcePath.string());
        }
        std::shared_ptr<openbus::rendering::ParsedObj> parsed;
        if (openbus::config::lower(collisionMesh.resolvedPath.extension().string()) == ".o3d") {
            parsed = openbus::rendering::O3DLoader::parse(collisionMesh.resolvedPath);
        } else {
            parsed = openbus::rendering::ObjLoader::parse(collisionMesh.resolvedPath,
                                                          collisionMesh.bundleEntry);
        }
        if (!parsed || parsed->triangles.empty()) {
            throw std::runtime_error("Bus model collision mesh has no triangles: " +
                                     collisionMesh.resolvedPath.string());
        }
        const std::size_t initialVertexCount = configuration.collisionMeshVertices.size();
        for (const openbus::rendering::ObjTriangle& triangle : parsed->triangles) {
            std::array<const openbus::rendering::ObjPosition*, 3> positions = {};
            bool validTriangle = true;
            for (std::size_t corner = 0; corner < triangle.indices.size(); ++corner) {
                const int index = triangle.indices[corner].position;
                if (index <= 0 || static_cast<std::size_t>(index) > parsed->positions.size()) {
                    validTriangle = false;
                    break;
                }
                positions[corner] = &parsed->positions[static_cast<std::size_t>(index - 1)];
            }
            if (!validTriangle) {
                continue;
            }
            const std::size_t nextVertex = configuration.collisionMeshVertices.size() / 3;
            if (nextVertex > static_cast<std::size_t>(std::numeric_limits<int>::max()) - 3) {
                throw std::runtime_error("Bus model collision mesh exceeds ODE's vertex limit");
            }
            for (std::size_t corner = 0; corner < positions.size(); ++corner) {
                const auto& position = *positions[corner];
                const std::array<double, 3> converted = {position.y, -position.x, position.z};
                if (!std::all_of(converted.begin(), converted.end(),
                                 [](double value) { return std::isfinite(value); })) {
                    validTriangle = false;
                    break;
                }
                configuration.collisionMeshVertices.insert(
                    configuration.collisionMeshVertices.end(), converted.begin(), converted.end());
                configuration.collisionMeshIndices.push_back(static_cast<int>(nextVertex + corner));
            }
            if (!validTriangle) {
                configuration.collisionMeshVertices.resize(nextVertex * 3);
                configuration.collisionMeshIndices.resize(nextVertex);
            }
        }
        if (configuration.collisionMeshVertices.size() == initialVertexCount) {
            throw std::runtime_error("Bus model collision mesh has no usable triangles: " +
                                     collisionMesh.resolvedPath.string());
        }
    }
    configuration.hasCollisionMesh = !configuration.collisionMeshIndices.empty();

    return configuration;
}

std::filesystem::path resolveModelConfigPath(const std::filesystem::path& busPath,
                                             const std::filesystem::path& modelPath) {
    return busPath.parent_path() / normalizedPath(modelPath);
}

std::optional<std::size_t> steeringAxleIndex(const std::string& variable) {
    const std::string normalized = openbus::config::lower(variable);
    constexpr const char* prefix = "axle_steering_";
    constexpr std::size_t prefixLength = 14;
    if (normalized.rfind(prefix, 0) != 0) {
        return {};
    }
    const std::size_t sideSeparator = normalized.find('_', prefixLength);
    if (sideSeparator == std::string::npos || (normalized.substr(sideSeparator + 1) != "l" &&
                                               normalized.substr(sideSeparator + 1) != "r")) {
        return {};
    }

    int index = -1;
    if (!openbus::config::parseInt(normalized.substr(prefixLength, sideSeparator - prefixLength),
                                   index) ||
        index < 0) {
        return {};
    }

    return static_cast<std::size_t>(index);
}

void applyModelSteering(VehicleConfig& vehicle, const ModelConfig& model) {
    for (const ModelPart& part : model.parts) {
        const std::optional<std::size_t> axleIndex =
            steeringAxleIndex(part.wheelAnimation.steeringVariable);
        if (axleIndex.has_value() && *axleIndex < vehicle.axles.size()) {
            vehicle.axles[*axleIndex].steerable = true;
        }
    }
}

} // namespace

std::filesystem::path modelConfigurationPathForBus(const std::filesystem::path& busConfigPath) {
    openbus::rendering::TraceScope trace("config", "modelConfigurationPathForBus");
    if (const char* configuredPath = openbus::getEnvironment("OPENBUS_MODEL_CONFIG");
        configuredPath != nullptr && *configuredPath != '\0') {
        return modelConfigurationPathForBus(busConfigPath, configuredPath);
    }
    const VehicleConfig source = loadBusConfig(busConfigPath);
    if (source.modelPath.empty()) {
        throw std::runtime_error("Bus configuration has no [model] entry: " +
                                 busConfigPath.string());
    }
    const std::filesystem::path modelConfigPath =
        resolveModelConfigPath(busConfigPath, source.modelPath);
    if (!std::filesystem::exists(modelConfigPath)) {
        throw std::runtime_error("Bus model configuration was not found: " +
                                 modelConfigPath.string());
    }
    return modelConfigPath;
}

std::filesystem::path modelConfigurationPathForBus(const std::filesystem::path& busConfigPath,
                                                   const std::filesystem::path& configuredPath) {
    openbus::rendering::TraceScope trace("config", "modelConfigurationPathForBus.explicit");
    const std::filesystem::path modelConfigPath =
        resolveConfiguredModelPath(busConfigPath, configuredPath);
    if (!std::filesystem::exists(modelConfigPath)) {
        throw std::runtime_error("Configured bus model was not found: " + modelConfigPath.string());
    }
    return modelConfigPath;
}

ModelConfig loadBusModelConfiguration(const std::filesystem::path& busConfigPath) {
    openbus::rendering::TraceScope trace("config", "loadBusModelConfiguration");
    const std::filesystem::path modelConfigPath = modelConfigurationPathForBus(busConfigPath);
    openbus::scripting::Vehicle variables;
    return loadModelConfig(modelConfigPath, modelConfigPath.parent_path(), ModelConfigKind::Bus,
                           variables);
}

BusConfiguration loadBusConfiguration(const std::filesystem::path& configPath) {
    openbus::rendering::TraceScope trace("config", "loadBusConfiguration");
    VehicleConfig source = loadBusConfig(configPath);
    if (source.massTonnes <= 0.0) {
        throw std::runtime_error("Bus configuration has no positive [mass]: " +
                                 configPath.string());
    }
    if (!source.hasBoundingBox || source.boundingBox[0] <= 0.0 || source.boundingBox[1] <= 0.0 ||
        source.boundingBox[2] <= 0.0) {
        throw std::runtime_error("Bus configuration has no valid [boundingbox]: " +
                                 configPath.string());
    }
    if (!source.hasCenterOfGravityHeight || source.centerOfGravityHeight <= 0.0) {
        throw std::runtime_error("Bus configuration has no positive [schwerpunkt]: " +
                                 configPath.string());
    }
    if (!source.hasWheelHalfWidth) {
        source.wheelHalfWidth = DEFAULT_WHEEL_HALF_WIDTH;
    } else if (source.wheelHalfWidth <= 0.0) {
        throw std::runtime_error("Bus configuration has no positive [openbus_wheel_half_width]: " +
                                 configPath.string());
    }
    if (!source.hasArticulated) {
        source.articulated = false;
    }
    if (source.axles.empty()) {
        throw std::runtime_error("Bus configuration contains no valid [newachse] entries: " +
                                 configPath.string());
    }
    const ModelConfig model = loadBusModelConfiguration(configPath);
    if (model.diagnostics.hasErrors()) {
        for (const ConfigurationDiagnostic& diagnostic : model.diagnostics.entries) {
            if (diagnostic.severity == ConfigurationDiagnostic::Severity::Error) {
                throw std::runtime_error("Invalid bus model configuration " +
                                         modelConfigurationPathForBus(configPath).string() +
                                         " at line " + std::to_string(diagnostic.line) + ": " +
                                         diagnostic.message);
            }
        }
    }
    applyModelSteering(source, model);
    return configurationFromVehicleConfig(source, model);
}

void writeBusConfigurationJson(std::ostream& output, const BusConfiguration& configuration) {
    output << std::setprecision(17)
           << "{\n"
              "  \"format_version\": 1,\n"
           << "  \"mass_kg\": " << configuration.mass << ",\n"
           << "  \"dimensions_m\": {\n"
              "    \"length\": "
           << configuration.length << ",\n"
           << "    \"width\": " << configuration.width << ",\n"
           << "    \"body_half_length\": " << configuration.bodyHalfLength << ",\n"
           << "    \"body_half_width\": " << configuration.bodyHalfWidth << ",\n"
           << "    \"body_half_height\": " << configuration.bodyHalfHeight
           << "\n"
              "  },\n"
              "  \"wheels\": {\n"
              "    \"radius_m\": "
           << configuration.wheelRadius << ",\n"
           << "    \"half_width_m\": " << configuration.wheelHalfWidth
           << "\n"
              "  },\n"
              "  \"center_of_gravity_height_m\": "
           << configuration.centerOfGravityHeight << ",\n"
           << "  \"articulated\": " << (configuration.articulated ? "true" : "false")
           << ",\n"
              "  \"collision_box\": {\n"
              "    \"length_m\": "
           << configuration.collisionLength << ",\n"
           << "    \"width_m\": " << configuration.collisionWidth << ",\n"
           << "    \"height_m\": " << configuration.collisionHeight
           << ",\n"
              "    \"offset_m\": {\n"
              "      \"x\": "
           << configuration.collisionOffsetX << ",\n"
           << "      \"y\": " << configuration.collisionOffsetY << ",\n"
           << "      \"z\": " << configuration.collisionOffsetZ
           << "\n"
              "    }\n"
              "  },\n"
              "  \"axles\": [\n";
    for (std::size_t index = 0; index < configuration.axles.size(); ++index) {
        const BusAxle& axle = configuration.axles[index];
        if (index != 0) {
            output << ",\n";
        }
        output << "    {\n"
                  "      \"index\": "
               << index << ",\n"
               << "      \"position_m\": " << axle.position << ",\n"
               << "      \"track_width_m\": " << axle.trackWidth << ",\n"
               << "      \"max_width_m\": " << axle.maxWidth << ",\n"
               << "      \"min_width_m\": " << axle.minWidth << ",\n"
               << "      \"wheel_diameter_m\": " << axle.wheelDiameter << ",\n"
               << "      \"spring_rate_N_per_m\": " << axle.springRate << ",\n"
               << "      \"max_force_N\": " << axle.maxForce << ",\n"
               << "      \"damper_rate_Ns_per_m\": " << axle.damperRate << ",\n"
               << "      \"steerable\": " << (axle.steerable ? "true" : "false") << ",\n"
               << "      \"driven\": " << (axle.driven ? "true" : "false")
               << "\n"
                  "    }";
    }
    output << "\n  ]\n}\n";
}

std::filesystem::path busConfigurationPathFor() {
    if (const char* configuredPath = openbus::getEnvironment("OPENBUS_BUS_CONFIG");
        configuredPath != nullptr && *configuredPath != '\0') {
        return busConfigurationPathFor(configuredPath);
    }
    throw std::runtime_error("OPENBUS_BUS_CONFIG is not set");
}

std::filesystem::path busConfigurationPathFor(const std::filesystem::path& configuredPath) {
    const std::filesystem::path busPath = resolveConfiguredBusPath(configuredPath);
    if (!std::filesystem::exists(busPath)) {
        throw std::runtime_error("Bus configuration was not found: " + busPath.string());
    }
    return busPath;
}

BusConfiguration busConfigurationFor() {
    return loadBusConfiguration(busConfigurationPathFor());
}