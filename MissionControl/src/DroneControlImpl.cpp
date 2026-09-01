#include <MissionControl/DroneControlImpl.h>

#include <MissionControl/ScanResultToVoxels.h>

#include <cmath>
#include <exception>
#include <stdexcept>
#include <utility>

namespace mission_control_207637604_325750099 {

using namespace common;

namespace {

// Below this distance (cm) a half-step retry is not worth attempting.
constexpr double kMinMoveCm = 0.5;
constexpr double kMinRotDeg = 0.1;
constexpr int kMaxNoopRetries = 3;
constexpr int kMaxLidarRetries = 3;
constexpr int kMaxMovementRetries = 3;
constexpr int kMaxGPSRetries = 3;
// Maximum reasonable movement per step (cm) - used to detect impossible GPS changes
constexpr double kMaxReasonableMoveCm = 500.0;

// Clamp a (non-negative) rotation magnitude to the drone's per-step maximum.
[[nodiscard, maybe_unused]] HorizontalAngle clampAngle(HorizontalAngle angle, HorizontalAngle max_angle) {
    double v = angle.numerical_value_in(deg);
    const double mx = max_angle.numerical_value_in(deg);
    if (v < 0.0) v = 0.0;
    if (v > mx) v = mx;
    return v * horizontal_angle[deg];
}

// Clamp a (non-negative) advance distance to the drone's per-step maximum.
[[nodiscard, maybe_unused]] PhysicalLength clampAdvance(PhysicalLength distance, PhysicalLength max_distance) {
    double v = distance.numerical_value_in(cm);
    const double mx = max_distance.numerical_value_in(cm);
    if (v < 0.0) v = 0.0;
    if (v > mx) v = mx;
    return v * cm;
}

// Clamp a signed elevation change (can be negative) to +/- the drone's maximum.
[[nodiscard, maybe_unused]] PhysicalLength clampElevate(PhysicalLength distance, PhysicalLength max_distance) {
    double v = distance.numerical_value_in(cm);
    const double mx = max_distance.numerical_value_in(cm);
    if (v > mx) v = mx;
    if (v < -mx) v = -mx;
    return v * cm;
}

// Check if a movement command has invalid values (NaN, Inf, negative where not allowed)
[[nodiscard]] bool hasInvalidValues(const types::MovementCommand& cmd) {
    switch (cmd.type) {
        case types::MovementCommandType::Rotate: {
            const double v = cmd.angle.numerical_value_in(deg);
            return std::isnan(v) || std::isinf(v) || v < 0.0;
        }
        case types::MovementCommandType::Advance: {
            const double v = cmd.distance.numerical_value_in(cm);
            return std::isnan(v) || std::isinf(v) || v < 0.0;
        }
        case types::MovementCommandType::Elevate: {
            const double v = cmd.distance.numerical_value_in(cm);
            return std::isnan(v) || std::isinf(v);
        }
        case types::MovementCommandType::Hover:
            return false;
    }
    return false;
}

// Check if a command is a NOOP (no movement and no scan)
[[nodiscard]] bool isNoop(const types::MappingStepCommand& cmd) {
    const bool no_movement = !cmd.movement.has_value() ||
                             cmd.movement->type == types::MovementCommandType::Hover;
    const bool no_scan = !cmd.scan_orientation.has_value();
    return no_movement && no_scan;
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

// BONUS: Check if position would be out of mission bounds
bool DroneControlImpl::wouldExceedBounds(const Position3D& target) const {
    const double x = target.x.numerical_value_in(cm);
    const double y = target.y.numerical_value_in(cm);
    const double z = target.z.numerical_value_in(cm);

    const auto& b = mission_.mission_bounds;
    return x < b.min_x.numerical_value_in(cm) || x > b.max_x.numerical_value_in(cm) ||
           y < b.min_y.numerical_value_in(cm) || y > b.max_y.numerical_value_in(cm) ||
           z < b.min_height.numerical_value_in(cm) || z > b.max_height.numerical_value_in(cm);
}

// BONUS: Compute target position after a movement command
Position3D DroneControlImpl::computeTargetPosition(const types::MovementCommand& cmd,
                                                    const types::DroneState& current) const {
    const double cx = current.position.x.numerical_value_in(cm);
    const double cy = current.position.y.numerical_value_in(cm);
    const double cz = current.position.z.numerical_value_in(cm);
    const double heading_rad = current.heading.horizontal.numerical_value_in(deg) * (3.14159265358979323846 / 180.0);

    switch (cmd.type) {
        case types::MovementCommandType::Advance: {
            const double d = cmd.distance.numerical_value_in(cm);
            return Position3D{
                (cx + d * std::cos(heading_rad)) * x_extent[cm],
                (cy + d * std::sin(heading_rad)) * y_extent[cm],
                cz * z_extent[cm]
            };
        }
        case types::MovementCommandType::Elevate: {
            const double d = cmd.distance.numerical_value_in(cm);
            return Position3D{
                cx * x_extent[cm],
                cy * y_extent[cm],
                (cz + d) * z_extent[cm]
            };
        }
        default:
            return current.position;
    }
}

// BONUS: Amend movement to stay within bounds
types::MovementCommand DroneControlImpl::amendToBounds(types::MovementCommand cmd,
                                                        const types::DroneState& current) const {
    const double cx = current.position.x.numerical_value_in(cm);
    const double cy = current.position.y.numerical_value_in(cm);
    const double cz = current.position.z.numerical_value_in(cm);
    const auto& b = mission_.mission_bounds;

    if (cmd.type == types::MovementCommandType::Advance) {
        const double heading_rad = current.heading.horizontal.numerical_value_in(deg) * (3.14159265358979323846 / 180.0);
        double d = cmd.distance.numerical_value_in(cm);
        const double cos_h = std::cos(heading_rad);
        const double sin_h = std::sin(heading_rad);

        // Compute max distance to each boundary
        double max_d = d;
        if (std::abs(cos_h) > 1e-9) {
            if (cos_h > 0) {
                max_d = std::min(max_d, (b.max_x.numerical_value_in(cm) - cx) / cos_h);
            } else {
                max_d = std::min(max_d, (b.min_x.numerical_value_in(cm) - cx) / cos_h);
            }
        }
        if (std::abs(sin_h) > 1e-9) {
            if (sin_h > 0) {
                max_d = std::min(max_d, (b.max_y.numerical_value_in(cm) - cy) / sin_h);
            } else {
                max_d = std::min(max_d, (b.min_y.numerical_value_in(cm) - cy) / sin_h);
            }
        }
        if (max_d < d && max_d > kMinMoveCm) {
            cmd.distance = max_d * cm;
        } else if (max_d <= kMinMoveCm) {
            cmd.type = types::MovementCommandType::Hover;
        }
    } else if (cmd.type == types::MovementCommandType::Elevate) {
        double d = cmd.distance.numerical_value_in(cm);
        if (d > 0) {
            const double max_up = b.max_height.numerical_value_in(cm) - cz;
            if (d > max_up) {
                cmd.distance = std::max(0.0, max_up) * cm;
            }
        } else {
            const double max_down = cz - b.min_height.numerical_value_in(cm);
            if (-d > max_down) {
                cmd.distance = -std::max(0.0, max_down) * cm;
            }
        }
    }
    return cmd;
}

// BONUS: Execute movement with splitting if larger than max allowed
void DroneControlImpl::executeMovementWithSplit(const types::MovementCommand& original_cmd) {
    types::MovementCommand cmd = original_cmd;

    switch (cmd.type) {
        case types::MovementCommandType::Rotate: {
            double remaining = cmd.angle.numerical_value_in(deg);
            const double max_rot = drone_.max_rotate.numerical_value_in(deg);
            while (remaining > kMinRotDeg) {
                const double step = std::min(remaining, max_rot);
                movement_.rotate(cmd.rotation, step * horizontal_angle[deg]);
                remaining -= step;
            }
            break;
        }
        case types::MovementCommandType::Advance: {
            double remaining = cmd.distance.numerical_value_in(cm);
            const double max_adv = drone_.max_advance.numerical_value_in(cm);
            while (remaining > kMinMoveCm) {
                const double step = std::min(remaining, max_adv);
                if (!movement_.advance(step * cm)) {
                    // Blocked - try half step once
                    const double half = step / 2.0;
                    if (half > kMinMoveCm) {
                        if (!movement_.advance(half * cm)) {
                            break; // Still blocked, stop
                        }
                        remaining -= half;
                    } else {
                        break;
                    }
                } else {
                    remaining -= step;
                }
            }
            break;
        }
        case types::MovementCommandType::Elevate: {
            double remaining = cmd.distance.numerical_value_in(cm);
            const double max_elev = drone_.max_elevate.numerical_value_in(cm);
            const int sign = (remaining >= 0) ? 1 : -1;
            remaining = std::abs(remaining);
            while (remaining > kMinMoveCm) {
                const double step = std::min(remaining, max_elev);
                if (!movement_.elevate(sign * step * cm)) {
                    const double half = step / 2.0;
                    if (half > kMinMoveCm) {
                        if (!movement_.elevate(sign * half * cm)) {
                            break;
                        }
                        remaining -= half;
                    } else {
                        break;
                    }
                } else {
                    remaining -= step;
                }
            }
            break;
        }
        case types::MovementCommandType::Hover:
            break;
    }
}

// BONUS: Scan with retry on empty result
std::optional<types::LidarScanResult> DroneControlImpl::scanWithRetry(const Orientation& orientation) {
    for (int attempt = 0; attempt < kMaxLidarRetries; ++attempt) {
        types::LidarScanResult scan = lidar_.scan(orientation);
        if (!scan.empty()) {
            return scan;
        }
        // Empty result - retry
    }
    return std::nullopt; // All retries exhausted
}

// BONUS: Check if GPS coordinates changed impossibly (teleportation detection)
bool DroneControlImpl::isImpossibleGPSChange(const Position3D& gps_pos,
                                              const Position3D& expected) const {
    const double dx = gps_pos.x.numerical_value_in(cm) - expected.x.numerical_value_in(cm);
    const double dy = gps_pos.y.numerical_value_in(cm) - expected.y.numerical_value_in(cm);
    const double dz = gps_pos.z.numerical_value_in(cm) - expected.z.numerical_value_in(cm);
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    return dist > kMaxReasonableMoveCm;
}

// BONUS: Get validated GPS position with retry on impossible coordinates
std::optional<Position3D> DroneControlImpl::getValidatedGPSPosition() {
    for (int attempt = 0; attempt < kMaxGPSRetries; ++attempt) {
        Position3D gps_pos = gps_.position();

        // BONUS: Check if GPS returns out-of-bound coordinates
        if (wouldExceedBounds(gps_pos)) {
            // Compare with internal coordinates
            if (internal_position_.has_value() && !wouldExceedBounds(*internal_position_)) {
                // Internal is valid, GPS is OOB - ignore GPS, use internal
                return internal_position_;
            }
            // Both are OOB - this is a real error, throw
            throw std::runtime_error("GPS_OUT_OF_BOUNDS");
        }

        // BONUS: Check for impossible coordinate change after movement
        if (internal_position_.has_value()) {
            if (isImpossibleGPSChange(gps_pos, *internal_position_)) {
                // Impossible change - retry GPS read
                continue;
            }
        }

        // GPS position is valid
        return gps_pos;
    }
    // All retries exhausted - return error
    return std::nullopt;
}

types::DroneStepResult DroneControlImpl::step() {
    // BONUS: Validate GPS position with retry on impossible coordinates
    auto validated_pos = getValidatedGPSPosition();
    if (!validated_pos.has_value()) {
        ++step_index_;
        return types::DroneStepResult{types::DroneStepStatus::Error,
            "GPS returned impossible coordinates after " + std::to_string(kMaxGPSRetries) + " retries"};
    }

    // Initialize internal position tracking on first step
    if (!internal_position_.has_value()) {
        internal_position_ = *validated_pos;
    }

    const types::DroneState current = state();
    const types::LidarScanResult* latest = last_scan_.has_value() ? &last_scan_.value() : nullptr;

    try {
        types::MappingStepCommand command = mapping_algorithm_.nextStep(current, latest);

        // BONUS: Handle NOOP commands (empty movement and scan)
        int noop_retries = 0;
        while (isNoop(command) && command.status == types::AlgorithmStatus::Working) {
            ++noop_retries;
            if (noop_retries >= kMaxNoopRetries) {
                ++step_index_;
                return types::DroneStepResult{types::DroneStepStatus::Error,
                    "algorithm returned NOOP " + std::to_string(kMaxNoopRetries) + " times"};
            }
            command = mapping_algorithm_.nextStep(current, latest);
        }

        // Process movement if present
        if (command.movement.has_value()) {
            types::MovementCommand mv = *command.movement;

            // BONUS: Check for invalid values and ignore if invalid
            if (hasInvalidValues(mv)) {
                // Gracefully ignore invalid command - don't pass to movement
            } else if (mv.type != types::MovementCommandType::Hover) {
                // BONUS: Check and amend if movement would exceed bounds
                Position3D target = computeTargetPosition(mv, current);
                if (wouldExceedBounds(target)) {
                    mv = amendToBounds(mv, current);
                }

                // Update internal position tracking before movement
                Position3D expected_pos = computeTargetPosition(mv, current);

                // BONUS: Execute with splitting if movement exceeds max
                executeMovementWithSplit(mv);

                // Update internal position after successful movement
                internal_position_ = expected_pos;
            }
        }

        // Scan after moving
        if (command.scan_orientation.has_value()) {
            // BONUS: Retry on empty LiDAR result
            auto scan_result = scanWithRetry(*command.scan_orientation);
            if (scan_result.has_value()) {
                ScanResultToVoxels::applyToMap(output_map_, gps_.position(), gps_.heading(),
                                               *scan_result, lidar_config_);
                last_scan_ = std::move(*scan_result);
            }
            // If all retries failed, continue without updating the map
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
        ++step_index_;
        return types::DroneStepResult{types::DroneStepStatus::Error, e.what()};
    }
}

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace mission_control_207637604_325750099