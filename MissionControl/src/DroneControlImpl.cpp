#include <MissionControl/DroneControlImpl.h>

#include <MissionControl/ScanResultToVoxels.h>

#include <cmath>
#include <exception>
#include <utility>

namespace mission_control_207637604_325750099 {

using namespace common;

namespace {

// Below this distance (cm) a half-step retry is not worth attempting.
constexpr double kMinMoveCm = 0.5;

// Clamp a (non-negative) rotation magnitude to the drone's per-step maximum.
[[nodiscard]] HorizontalAngle clampAngle(HorizontalAngle angle, HorizontalAngle max_angle) {
    double v = angle.numerical_value_in(deg);
    const double mx = max_angle.numerical_value_in(deg);
    if (v < 0.0) v = 0.0;
    if (v > mx) v = mx;
    return v * horizontal_angle[deg];
}

// Clamp a (non-negative) advance distance to the drone's per-step maximum.
[[nodiscard]] PhysicalLength clampAdvance(PhysicalLength distance, PhysicalLength max_distance) {
    double v = distance.numerical_value_in(cm);
    const double mx = max_distance.numerical_value_in(cm);
    if (v < 0.0) v = 0.0;
    if (v > mx) v = mx;
    return v * cm;
}

// Clamp a signed elevation change (can be negative) to +/- the drone's maximum.
[[nodiscard]] PhysicalLength clampElevate(PhysicalLength distance, PhysicalLength max_distance) {
    double v = distance.numerical_value_in(cm);
    const double mx = max_distance.numerical_value_in(cm);
    if (v > mx) v = mx;
    if (v < -mx) v = -mx;
    return v * cm;
}

} // namespace

DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                   types::MissionConfigData mission,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm,
                                   types::LidarConfigData lidar_config)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm),
      lidar_config_(std::move(lidar_config)) {}

types::DroneStepResult DroneControlImpl::step() {
    const types::DroneState current = state();
    const types::LidarScanResult* latest = last_scan_.has_value() ? &last_scan_.value() : nullptr;

    try {
        const types::MappingStepCommand command = mapping_algorithm_.nextStep(current, latest);

        // Movement is performed before scan. A blocked move is retried once at
        // half the distance so the drone can edge toward a wall; a still-blocked
        // move is NOT an error - the algorithm's own stuck-detection reroutes.
        // Only an *exception* from a driver (e.g. a collision the mock signals by
        // throwing) is fatal, and is caught below as a step error.
        if (command.movement.has_value()) {
            const types::MovementCommand& m = *command.movement;
            switch (m.type) {
                case types::MovementCommandType::Rotate:
                    movement_.rotate(m.rotation, clampAngle(m.angle, drone_.max_rotate));
                    break;
                case types::MovementCommandType::Advance: {
                    const PhysicalLength dist = clampAdvance(m.distance, drone_.max_advance);
                    if (!movement_.advance(dist)) {
                        const double half = dist.numerical_value_in(cm) / 2.0;
                        if (half > kMinMoveCm) {
                            movement_.advance(half * cm);
                        }
                    }
                    break;
                }
                case types::MovementCommandType::Elevate: {
                    const PhysicalLength dist = clampElevate(m.distance, drone_.max_elevate);
                    if (!movement_.elevate(dist)) {
                        const double half = dist.numerical_value_in(cm) / 2.0;
                        if (std::abs(half) > kMinMoveCm) {
                            movement_.elevate(half * cm);
                        }
                    }
                    break;
                }
                case types::MovementCommandType::Hover:
                    break;
            }
        }

        // Scan after moving; ScanResultToVoxels writes the whole observation
        // into the output map.
        if (command.scan_orientation.has_value()) {
            types::LidarScanResult scan = lidar_.scan(*command.scan_orientation);
            ScanResultToVoxels::applyToMap(output_map_, gps_.position(), gps_.heading(), scan, lidar_config_);
            last_scan_ = std::move(scan);
        }

        ++step_index_;

        switch (command.status) {
            case types::AlgorithmStatus::Finished:
                return types::DroneStepResult{types::DroneStepStatus::Completed, "mapping complete"};
            case types::AlgorithmStatus::FinishedWithUnmappableVoxels:
                return types::DroneStepResult{types::DroneStepStatus::Completed,
                                              "mapping complete (some voxels unreachable)"};
            case types::AlgorithmStatus::Working:
            default:
                return types::DroneStepResult{types::DroneStepStatus::Continue, ""};
        }
    } catch (const std::exception& e) {
        // A driver or algorithm threw (e.g. a collision): surface it as a step
        // error so the mission ends cleanly instead of the simulator crashing.
        ++step_index_;
        return types::DroneStepResult{types::DroneStepStatus::Error, e.what()};
    }
}

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace mission_control_207637604_325750099
