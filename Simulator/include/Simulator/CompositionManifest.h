#pragma once

#include <filesystem>
#include <vector>

namespace simulator {

// The raw config-file paths from a composition file, in the same order as the
// runs the SimulationManager produces. Used when writing the hierarchical YAML
// report (the typed config structs do not carry their source paths).
struct CompositionManifest {
    struct SimEntry {
        std::filesystem::path simulation_config;
        std::vector<std::filesystem::path> mission_configs;
    };
    std::filesystem::path composition_file;
    std::vector<SimEntry> simulations;
    std::vector<std::filesystem::path> drone_configs;
    std::vector<std::filesystem::path> lidar_configs;
};

} // namespace simulator
