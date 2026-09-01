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
//
// BONUS ERROR HANDLING:
// - Validates movement commands for invalid values (NaN, Inf, negative)
// - Detects and retries NOOP commands (empty movement + no scan)
// - Splits movements larger than max allowed into multiple steps
// - Amends movements that would exceed mission bounds
// - Retries LiDAR scans that return empty results
// - Handles blocked movements with half-step retry
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
    // Check if position would be out of mission bounds
    [[nodiscard]] bool wouldExceedBounds(const common::Position3D& target) const;

    // Compute target position after a movement command
    [[nodiscard]] common::Position3D computeTargetPosition(
        const common::types::MovementCommand& cmd,
        const common::types::DroneState& current) const;

    // Amend movement to stay within mission bounds
    [[nodiscard]] common::types::MovementCommand amendToBounds(
        common::types::MovementCommand cmd,
        const common::types::DroneState& current) const;

    // Execute movement with splitting if larger than max allowed
    void executeMovementWithSplit(const common::types::MovementCommand& cmd);

    // Scan with retry on empty result
    [[nodiscard]] std::optional<common::types::LidarScanResult>
    scanWithRetry(const common::Orientation& orientation);

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
