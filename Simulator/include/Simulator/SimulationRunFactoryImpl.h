#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>
#include <Simulator/ISimulationRunFactory.h>

namespace simulator {

// Builds one runnable simulation. It constructs the simulation-owned mocks
// (hidden/output maps, GPS, movement, lidar) and then uses the loaded plugin
// factories to create the algorithm and mission control, wiring everything into
// a SimulationRunImpl. The (algorithm, mission-control) pairing is fixed for the
// lifetime of one factory instance - a comparative/competitive run creates one
// factory per cell of its matrix.
class SimulationRunFactoryImpl final : public ISimulationRunFactory {
public:
    SimulationRunFactoryImpl(common::MappingAlgorithmFactory algorithm_factory,
                             common::MissionControlFactory mission_control_factory,
                             bool verbose);

    [[nodiscard]] std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData& simulation_config,
           const common::types::MissionConfigData& mission_config,
           const common::types::DroneConfigData& drone_config,
           const common::types::LidarConfigData& lidar_config,
           const std::filesystem::path& output_path) override;

private:
    common::MappingAlgorithmFactory algorithm_factory_;
    common::MissionControlFactory mission_control_factory_;
    bool verbose_;
};

} // namespace simulator
