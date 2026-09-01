#include <Simulator/ConfigLoader.h>

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>
#include <utility>

namespace simulator {

using namespace common; // unit symbols, quantity specs, Position3D (NOT `types`)

namespace {

// Returns config[key] if present, otherwise the node itself (tolerates files
// written with or without the leading top-level key).
[[nodiscard]] YAML::Node section(const YAML::Node& root, const std::string& key) {
    return root[key] ? root[key] : root;
}

[[nodiscard]] common::types::MappingBounds parseBounds(const YAML::Node& b) {
    common::types::MappingBounds mb;
    mb.min_x = b["x_boundary"]["min_cm"].as<double>() * x_extent[cm];
    mb.max_x = b["x_boundary"]["max_cm"].as<double>() * x_extent[cm];
    mb.min_y = b["y_boundary"]["min_cm"].as<double>() * y_extent[cm];
    mb.max_y = b["y_boundary"]["max_cm"].as<double>() * y_extent[cm];
    mb.min_height = b["height_boundary"]["min_cm"].as<double>() * z_extent[cm];
    mb.max_height = b["height_boundary"]["max_cm"].as<double>() * z_extent[cm];
    return mb;
}

[[nodiscard]] Position3D parsePositionCm(const YAML::Node& n) {
    return Position3D{
        n["x_cm"].as<double>() * x_extent[cm],
        n["y_cm"].as<double>() * y_extent[cm],
        n["height_cm"].as<double>() * z_extent[cm],
    };
}

[[nodiscard]] Position3D parseOffset(const YAML::Node& n) {
    return Position3D{
        n["x_offset"].as<double>() * x_extent[cm],
        n["y_offset"].as<double>() * y_extent[cm],
        n["height_offset"].as<double>() * z_extent[cm],
    };
}

[[nodiscard]] YAML::Node loadFile(const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) {
        throw std::runtime_error("config file not found: " + path.string());
    }
    return YAML::LoadFile(path.string());
}

} // namespace

common::types::DroneConfigData ConfigLoader::loadDrone(const std::filesystem::path& path) {
    const YAML::Node n = section(loadFile(path), "drone_config");
    common::types::DroneConfigData d;
    d.radius = (n["dimensions_cm"].as<double>() / 2.0) * cm; // diameter -> radius
    d.max_rotate = n["max_rotate_deg"].as<double>() * horizontal_angle[deg];
    d.max_advance = n["max_advance_cm"].as<double>() * cm;
    d.max_elevate = n["max_elevate_cm"].as<double>() * cm;
    return d;
}

common::types::MissionConfigData ConfigLoader::loadMission(const std::filesystem::path& path) {
    const YAML::Node n = section(loadFile(path), "mission_config");
    common::types::MissionConfigData m;
    m.max_steps = n["max_steps"].as<std::size_t>();
    m.gps_resolution = n["gps_resolution_cm"].as<double>() * cm;
    m.output_mapping_resolution_factor =
        n["output_mapping_resolution_factor"] ? n["output_mapping_resolution_factor"].as<double>() : 1.0;
    if (n["boundaries"]) {
        m.mission_bounds = parseBounds(n["boundaries"]);
    }
    return m;
}

common::types::LidarConfigData ConfigLoader::loadLidar(const std::filesystem::path& path) {
    const YAML::Node n = section(loadFile(path), "lidar_config");
    common::types::LidarConfigData l;
    l.z_min = n["z_min_cm"].as<double>() * cm;
    l.z_max = n["z_max_cm"].as<double>() * cm;
    l.d = n["d_cm"].as<double>() * cm;
    l.fov_circles = n["fov_circles"].as<std::size_t>();
    return l;
}

types::SimulationConfigData ConfigLoader::loadSimulation(const std::filesystem::path& path) {
    const YAML::Node n = section(loadFile(path), "simulation_config");
    types::SimulationConfigData s;
    s.map_filename = n["map_filename"].as<std::string>();
    s.map_resolution = n["map_resolution_cm"].as<double>() * cm;
    s.initial_drone_position = parsePositionCm(n["initial_drone_position"]);
    s.initial_angle = n["initial_angle_deg"].as<double>() * horizontal_angle[deg];
    if (n["map_axes_offset"]) {
        s.map_offset = parseOffset(n["map_axes_offset"]);
    }
    // Map filenames are resolved relative to the simulation file's directory,
    // falling back to its parent (the composition directory) - the provided
    // inputs keep the shared maps one level up, under <inputs>/map.
    if (s.map_filename.is_relative()) {
        std::filesystem::path candidate = path.parent_path() / s.map_filename;
        if (!std::filesystem::exists(candidate)) {
            candidate = path.parent_path().parent_path() / s.map_filename;
        }
        s.map_filename = candidate;
    }
    return s;
}

std::vector<types::SimulationCompositionData>
ConfigLoader::loadComposition(const std::filesystem::path& path) {
    const YAML::Node root = loadFile(path);
    const YAML::Node comp = section(root, "simulation_compositions");
    const std::filesystem::path base = path.parent_path();

    std::vector<common::types::DroneConfigData> drones;
    for (const auto& d : comp["drone_configs"]) {
        drones.push_back(loadDrone(base / d.as<std::string>()));
    }
    std::vector<common::types::LidarConfigData> lidars;
    for (const auto& l : comp["lidar_configs"]) {
        lidars.push_back(loadLidar(base / l.as<std::string>()));
    }

    std::vector<types::SimulationCompositionData> result;
    for (const auto& sim : comp["simulations"]) {
        types::SimulationCompositionData c;
        c.composition_file = path;

        types::SimulationConfigData sim_config =
            loadSimulation(base / sim["simulation_config"].as<std::string>());
        std::vector<common::types::MissionConfigData> missions;
        for (const auto& m : sim["mission_configs"]) {
            missions.push_back(loadMission(base / m.as<std::string>()));
        }
        c.simulation_mission_groups.emplace_back(sim_config, missions);

        c.drone_configs = drones;
        c.lidar_configs = lidars;
        result.push_back(std::move(c));
    }
    return result;
}

CompositionManifest ConfigLoader::loadCompositionManifest(const std::filesystem::path& path) {
    const YAML::Node root = loadFile(path);
    const YAML::Node comp = section(root, "simulation_compositions");

    CompositionManifest manifest;
    manifest.composition_file = path;
    for (const auto& d : comp["drone_configs"]) {
        manifest.drone_configs.emplace_back(d.as<std::string>());
    }
    for (const auto& l : comp["lidar_configs"]) {
        manifest.lidar_configs.emplace_back(l.as<std::string>());
    }
    for (const auto& sim : comp["simulations"]) {
        CompositionManifest::SimEntry entry;
        entry.simulation_config = sim["simulation_config"].as<std::string>();
        for (const auto& m : sim["mission_configs"]) {
            entry.mission_configs.emplace_back(m.as<std::string>());
        }
        manifest.simulations.push_back(std::move(entry));
    }
    return manifest;
}

} // namespace simulator
