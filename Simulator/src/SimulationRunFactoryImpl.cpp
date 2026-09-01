#include <Simulator/SimulationRunFactoryImpl.h>

#include <Simulator/Map3DImpl.h>
#include <Simulator/MockGPS.h>
#include <Simulator/MockLidar.h>
#include <Simulator/MockMovement.h>
#include <Simulator/SimulationRunImpl.h>

#include <Common/MappingAlgorithmFactory.h> // MappingAlgorithmDependencies
#include <Common/MissionControlFactory.h>   // MissionControlDependencies

#include <TinyNPY.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <utility>

namespace simulator {

using namespace common; // unit symbols / Position3D / Orientation (NOT `types`)

namespace {

// A fresh owning int8 NPY array, initialised entirely to Unmapped (0xFF == -1).
[[nodiscard]] std::shared_ptr<NpyArray> makeUnmappedArray(std::size_t sx, std::size_t sy, std::size_t sz) {
    auto arr = std::make_shared<NpyArray>(
        NpyArray::shape_t{sx, sy, sz}, static_cast<std::size_t>(1),
        NpyArray::GetTypeChar(typeid(std::int8_t)));
    arr->Allocate();
    std::uint8_t* data = arr->Data<std::uint8_t>();
    std::fill(data, data + arr->NumValue(), static_cast<std::uint8_t>(0xFF));
    return arr;
}

// Number of voxels needed to cover [min_cm, max_cm] at the given resolution.
[[nodiscard]] std::size_t voxelsFor(double min_cm, double max_cm, double res_cm) {
    if (res_cm <= 0.0 || max_cm <= min_cm) {
        return 1;
    }
    return static_cast<std::size_t>(std::llround((max_cm - min_cm) / res_cm) + 1);
}

} // namespace

SimulationRunFactoryImpl::SimulationRunFactoryImpl(MappingAlgorithmFactory algorithm_factory,
                                                   MissionControlFactory mission_control_factory,
                                                   bool verbose)
    : algorithm_factory_(std::move(algorithm_factory)),
      mission_control_factory_(std::move(mission_control_factory)),
      verbose_(verbose) {}

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                 const common::types::MissionConfigData& mission,
                                 const common::types::DroneConfigData& drone,
                                 const common::types::LidarConfigData& lidar,
                                 const std::filesystem::path& output_path) {
    // ---- Load the hidden ground-truth map (full building) ---------------
    auto hidden_arr = std::make_shared<NpyArray>();
    const LPCSTR load_err = hidden_arr->LoadNPY(simulation.map_filename.string());
    if (load_err != nullptr) {
        throw std::runtime_error("failed to load hidden map '" +
                                 simulation.map_filename.string() + "': " + load_err);
    }
    if (hidden_arr->Shape().size() != 3) {
        throw std::runtime_error("hidden map '" + simulation.map_filename.string() +
                                 "' is not a 3D (X, Y, Z) array");
    }

    const double in_res = simulation.map_resolution.numerical_value_in(cm);
    if (in_res <= 0.0) {
        throw std::runtime_error("map_resolution must be positive");
    }

    // The hidden map keeps the simulation offset/resolution but advertises the
    // mission boundaries so scoring iterates that region; atVoxel() still sees
    // the whole building (checked against the NPY shape).
    common::types::MapConfig hidden_cfg;
    hidden_cfg.boundaries = mission.mission_bounds;
    hidden_cfg.offset = simulation.map_offset;
    hidden_cfg.resolution = simulation.map_resolution;

    // Output resolution: factor>=1 scales (coarser); factor<1 is ignored + logged.
    const double factor = mission.output_mapping_resolution_factor;
    const double out_res = (factor >= 1.0) ? in_res * factor : in_res;
    if (factor < 1.0) {
        std::ofstream log(output_path / "error_log.txt", std::ios::app);
        log << "[ERROR] RESOLUTION_FACTOR_TOO_SMALL - output_mapping_resolution_factor="
            << factor << " is < 1; ignored, using the input resolution\n";
    }

    const double mnx = mission.mission_bounds.min_x.numerical_value_in(cm);
    const double mxx = mission.mission_bounds.max_x.numerical_value_in(cm);
    const double mny = mission.mission_bounds.min_y.numerical_value_in(cm);
    const double mxy = mission.mission_bounds.max_y.numerical_value_in(cm);
    const double mnh = mission.mission_bounds.min_height.numerical_value_in(cm);
    const double mxh = mission.mission_bounds.max_height.numerical_value_in(cm);

    // A missing/degenerate boundaries section has no usable mapping region.
    // Reported via invalid_argument so the manager surfaces MISSION_BOUNDARY_INVALID.
    if (!(mnx < mxx && mny < mxy && mnh < mxh)) {
        throw std::invalid_argument(
            "mission boundaries are missing or degenerate (min must be < max on every axis)");
    }

    common::types::MapConfig out_cfg;
    out_cfg.boundaries = mission.mission_bounds;
    out_cfg.offset = Position3D{mnx * x_extent[cm], mny * y_extent[cm], mnh * z_extent[cm]};
    out_cfg.resolution = out_res * cm;
    auto output_arr = makeUnmappedArray(voxelsFor(mnx, mxx, out_res),
                                        voxelsFor(mny, mxy, out_res),
                                        voxelsFor(mnh, mxh, out_res));

    // ---- Build the simulation-owned graph -------------------------------
    auto hidden_map = std::make_unique<Map3DImpl>(hidden_arr, hidden_cfg);
    auto output_map = std::make_unique<Map3DImpl>(output_arr, out_cfg);

    auto gps = std::make_unique<MockGPS>(simulation.initial_drone_position,
                                         Orientation{simulation.initial_angle, AltitudeAngle{}},
                                         mission.gps_resolution);
    auto movement = std::make_unique<MockMovement>(*gps, *hidden_map, drone.radius);
    auto mock_lidar = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);

    // ---- Output file path (unique per run) ------------------------------
    static std::atomic<unsigned long long> run_counter{0};
    const unsigned long long idx = run_counter.fetch_add(1);
    const std::filesystem::path out_dir = output_path / "output_results";
    std::filesystem::create_directories(out_dir);
    const std::filesystem::path output_map_file =
        out_dir / (simulation.map_filename.stem().string() + "_run" + std::to_string(idx) + ".npy");

    // ---- Create the plugin-provided components via the loaded factories --
    auto algorithm = algorithm_factory_(
        MappingAlgorithmDependencies{mission, lidar, drone, *output_map});

    // The mission control builds its own DroneControl from these dependencies.
    auto mission_control = mission_control_factory_(
        MissionControlDependencies{mission, drone, *mock_lidar, *gps, *movement,
                                   *output_map, *algorithm, output_map_file, verbose_});

    // Only the unique_ptrs transfer ownership below; the referenced objects keep
    // their addresses, so the cross-references stay valid.
    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(mock_lidar),
        std::move(algorithm),
        std::move(mission_control),
        simulation,
        mission,
        output_map_file);
}

} // namespace simulator
