#include <Simulator/SimulationRunImpl.h>

#include <Simulator/MapsComparison.h>

#include <utility>
#include <vector>

namespace simulator {

using namespace common;

SimulationRunImpl::SimulationRunImpl(std::unique_ptr<const IMap3D> hidden_map,
                                     std::unique_ptr<IMutableMap3D> output_map,
                                     std::unique_ptr<IGPS> gps,
                                     std::unique_ptr<IDroneMovement> movement,
                                     std::unique_ptr<ILidar> lidar,
                                     std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                                     std::unique_ptr<IMissionControl> mission_control,
                                     types::SimulationConfigData simulation_config,
                                     common::types::MissionConfigData mission_config,
                                     std::filesystem::path output_map_file)
    : hidden_map_(std::move(hidden_map)),
      output_map_(std::move(output_map)),
      gps_(std::move(gps)),
      movement_(std::move(movement)),
      lidar_(std::move(lidar)),
      mapping_algorithm_(std::move(mapping_algorithm)),
      mission_control_(std::move(mission_control)),
      simulation_config_(std::move(simulation_config)),
      mission_config_(std::move(mission_config)),
      output_map_file_(std::move(output_map_file)) {}

types::SimulationResult SimulationRunImpl::run() {
    types::SimulationResult result;
    result.simulation_config = simulation_config_;
    result.mission_config = mission_config_;
    result.output_map_file = output_map_file_;

    // Report whether the requested output resolution factor was honoured.
    const double factor = mission_config_.output_mapping_resolution_factor;
    if (factor >= 1.0) {
        result.resolution_request_status = types::ResolutionRequestStatus::Accepted;
    } else if (factor > 0.0) {
        result.resolution_request_status = types::ResolutionRequestStatus::IgnoredTooSmall;
    } else {
        result.resolution_request_status = types::ResolutionRequestStatus::Ignored;
    }

    // Run the (single) mission and record its outcome.
    const common::types::MissionRunResult mission = mission_control_->runMission();
    result.mission_results.push_back(mission);

    if (output_map_) {
        result.output_map_config = output_map_->getMapConfig();
    }

    // Score: compare the hidden ground-truth map against the produced map.
    // A mission error scores -1.
    if (mission.status == common::types::MissionRunStatus::Error || !hidden_map_ || !output_map_) {
        result.mission_score = -1.0;
    } else {
        std::vector<IMap3D*> targets{output_map_.get()};
        const std::vector<double> scores = MapsComparison::compare(*hidden_map_, targets);
        result.mission_score = scores.empty() ? -1.0 : scores.front();
    }

    return result;
}

} // namespace simulator
