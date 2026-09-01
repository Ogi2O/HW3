#include <Simulator/SimulationManager.h>

#include <exception>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace simulator {

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {}

types::SimulationManagerReport
SimulationManager::run(const types::SimulationCompositionData& composition,
                       const std::filesystem::path& output_path) {
    types::SimulationManagerReport report;
    report.composition_file = composition.composition_file;
    report.metric = "output_map_accuracy";
    report.score_range = {0.0, 100.0};
    report.error_score = -1;

    std::filesystem::create_directories(output_path);
    std::ofstream error_log(output_path / "error_log.txt", std::ios::app);

    auto recordFailure = [&](const types::SimulationConfigData& simulation,
                             const common::types::MissionConfigData& mission,
                             const char* code, const std::string& what) {
        error_log << "[ERROR] sim=" << simulation.map_filename.string()
                  << " : " << code << " - " << what << "\n";
        error_log.flush();
        types::SimulationResult failed;
        failed.simulation_config = simulation;
        failed.mission_config = mission;
        failed.mission_score = -1.0;
        failed.mission_results.push_back(common::types::MissionRunResult{
            common::types::MissionRunStatus::Error, 0,
            {common::types::ErrorRef{code, what}}});
        report.runs.push_back(std::move(failed));
    };

    // Iterate simulation-mission groups, then the cartesian product with the
    // drone and lidar configurations.
    for (const auto& [simulation, missions] : composition.simulation_mission_groups) {
        for (const auto& mission : missions) {
            for (const auto& drone : composition.drone_configs) {
                for (const auto& lidar : composition.lidar_configs) {
                    try {
                        auto run = run_factory_->create(simulation, mission, drone, lidar, output_path);
                        if (run) {
                            types::SimulationResult result = run->run();
                            report.runs.push_back(std::move(result));
                        }
                    } catch (const std::invalid_argument& e) {
                        recordFailure(simulation, mission, "MISSION_BOUNDARY_INVALID", e.what());
                    } catch (const std::exception& e) {
                        recordFailure(simulation, mission, "RUN_CREATE_ERROR", e.what());
                    }
                }
            }
        }
    }

    return report;
}

} // namespace simulator
