#include "BusConfiguration.h"

#include "BusConfigLoader.h"
#include "ConfigurationParser.h"
#include "Environment.h"
#include "ModelConfigLoader.h"
#include "PerfTrace.h"
#include "Variables.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>

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

BusConfiguration configurationFromVehicleConfig(const VehicleConfig& source) {
    const std::array<double, 6>& boundingBox = source.boundingBox;

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
                throw std::runtime_error("Invalid bus model configuration " + configPath.string() +
                                         " at line " + std::to_string(diagnostic.line) + ": " +
                                         diagnostic.message);
            }
        }
    }
    applyModelSteering(source, model);
    return configurationFromVehicleConfig(source);
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