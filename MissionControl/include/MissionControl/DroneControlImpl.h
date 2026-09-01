#pragma once

#include <MissionControl/IDroneControl.h>

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMutableMap3D.h>

#include <optional>

namespace mission_control_207637604_325750099 {

// Internal to the MissionControl plugin: runs a single drone step - read state,
// ask the algorithm, perform the clamped movement then the scan, write the scan
// into the output map, advance the step index, and translate the algorithm
// status into a step status.
class DroneControlImpl final : public mission_control::IDroneControl {
public:
    DroneControlImpl(common::types::DroneConfigData drone,
                     common::types::MissionConfigData mission,
                     common::ILidar& lidar,
                     common::IGPS& gps,
                     common::IDroneMovement& movement,
                     common::IMutableMap3D& output_map,
                     common::IMappingAlgorithm& mapping_algorithm,
                     common::types::LidarConfigData lidar_config);

    [[nodiscard]] common::types::DroneStepResult step() override;
    [[nodiscard]] common::types::DroneState state() const override;

private:
    common::types::DroneConfigData drone_;
    common::types::MissionConfigData mission_;
    common::ILidar& lidar_;
    common::IGPS& gps_;
    common::IDroneMovement& movement_;
    common::IMutableMap3D& output_map_;
    common::IMappingAlgorithm& mapping_algorithm_;
    common::types::LidarConfigData lidar_config_;
    std::optional<common::types::LidarScanResult> last_scan_;
    std::size_t step_index_ = 0;
};

} // namespace mission_control_207637604_325750099
