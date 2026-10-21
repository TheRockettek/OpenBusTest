#pragma once

#include "core/BusTypes.h"
#include "core/OmsiRoot.h"

#include <filesystem>
#include <ostream>

struct ModelConfig;

std::filesystem::path busConfigurationPathFor();
std::filesystem::path busConfigurationPathFor(const std::filesystem::path& configuredPath);
BusConfiguration loadBusConfiguration(const std::filesystem::path& configPath);
std::filesystem::path modelConfigurationPathForBus(const std::filesystem::path& busConfigPath);
std::filesystem::path modelConfigurationPathForBus(const std::filesystem::path& busConfigPath,
                                                   const std::filesystem::path& configuredPath);
std::filesystem::path
modelConfigurationPathForVehicle(const std::filesystem::path& vehicleConfigPath);
std::filesystem::path
modelConfigurationPathForVehicle(const std::filesystem::path& vehicleConfigPath,
                                 const std::filesystem::path& configuredPath);
ModelConfig loadBusModelConfiguration(const std::filesystem::path& busConfigPath);
void writeBusConfigurationJson(std::ostream& output, const BusConfiguration& configuration);
BusConfiguration busConfigurationFor();