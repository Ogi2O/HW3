#pragma once

#include <Common/Types.h>
#include <Simulator/CompositionManifest.h>
#include <Simulator/SimulationTypes.h>

#include <filesystem>
#include <vector>

namespace simulator {

// Parses the YAML configuration files into the strongly-typed config structs.
// All numeric fields are converted into mp-units quantities (cm / degrees).
class ConfigLoader {
public:
    [[nodiscard]] static common::types::DroneConfigData loadDrone(const std::filesystem::path& path);
    [[nodiscard]] static common::types::MissionConfigData loadMission(const std::filesystem::path& path);
    [[nodiscard]] static common::types::LidarConfigData loadLidar(const std::filesystem::path& path);
    [[nodiscard]] static types::SimulationConfigData loadSimulation(const std::filesystem::path& path);

    // One SimulationCompositionData per simulation entry (each = that simulation
    // + its missions + all drones + all lidars), preserving the per-simulation
    // cartesian product.
    [[nodiscard]] static std::vector<types::SimulationCompositionData>
    loadComposition(const std::filesystem::path& path);

    // The matching raw paths (relative, as written in the file) for reporting.
    [[nodiscard]] static CompositionManifest loadCompositionManifest(const std::filesystem::path& path);
};

} // namespace simulator
