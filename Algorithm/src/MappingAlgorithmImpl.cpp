#include <Algorithm/MappingAlgorithmImpl.h>

#include <Common/IMap3D.h>
#include <Common/MappingAlgorithmRegistration.h>

#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>

namespace algorithm_207637604_325750099 {

// Bring the shared unit symbols / quantity specs / command types into this
// project's namespace so the ported logic reads exactly as before.
using namespace common;

namespace {

constexpr std::size_t kNumDirs = 8;                 // horizontal scan directions
constexpr std::size_t kNumElev = 3;                 // elevation angles per direction
constexpr std::size_t kTotalScans = kNumDirs * kNumElev;
constexpr std::array<double, kNumElev> kElevAngles = {-30.0, 0.0, 30.0};

constexpr int kMaxStuck = 3;
constexpr double kMinMoveCm = 0.5;
constexpr double kMinRotDeg = 1.0;
constexpr double kFullCircleDeg = 360.0;
constexpr double kHalfCircleDeg = 180.0;
constexpr double kDegToRad = std::numbers::pi / 180.0;

constexpr std::array<std::array<int, 3>, 6> kNeighbors = {{
    {{1, 0, 0}}, {{-1, 0, 0}},
    {{0, 1, 0}}, {{0, -1, 0}},
    {{0, 0, 1}}, {{0, 0, -1}},
}};

[[nodiscard]] Orientation scanOrientation(std::size_t index) {
    const std::size_t dir = index / kNumElev;
    const std::size_t elev = index % kNumElev;
    const double h = static_cast<double>(dir) * (kFullCircleDeg / static_cast<double>(kNumDirs));
    const double v = kElevAngles[elev];
    return Orientation{h * horizontal_angle[deg], v * altitude_angle[deg]};
}

} // namespace

MappingAlgorithmImpl::GridKey MappingAlgorithmImpl::worldToGrid(const Position3D& pos) const {
    const types::MapConfig cfg = output_map_.getMapConfig();
    double res = cfg.resolution.numerical_value_in(cm);
    if (res <= 0.0) res = 1.0;
    const double ox = cfg.offset.x.numerical_value_in(cm);
    const double oy = cfg.offset.y.numerical_value_in(cm);
    const double oz = cfg.offset.z.numerical_value_in(cm);
    return GridKey{
        std::llround((pos.x.numerical_value_in(cm) - ox) / res),
        std::llround((pos.y.numerical_value_in(cm) - oy) / res),
        std::llround((pos.z.numerical_value_in(cm) - oz) / res),
    };
}

Position3D MappingAlgorithmImpl::gridToWorld(const GridKey& key) const {
    const types::MapConfig cfg = output_map_.getMapConfig();
    double res = cfg.resolution.numerical_value_in(cm);
    if (res <= 0.0) res = 1.0;
    const double ox = cfg.offset.x.numerical_value_in(cm);
    const double oy = cfg.offset.y.numerical_value_in(cm);
    const double oz = cfg.offset.z.numerical_value_in(cm);
    return Position3D{
        (ox + static_cast<double>(key.x) * res) * x_extent[cm],
        (oy + static_cast<double>(key.y) * res) * y_extent[cm],
        (oz + static_cast<double>(key.z) * res) * z_extent[cm],
    };
}

void MappingAlgorithmImpl::recomputeFrontier(const types::DroneState& /*state*/) {
    frontier_.clear();

    const types::MapConfig cfg = output_map_.getMapConfig();
    double res = cfg.resolution.numerical_value_in(cm);
    if (res <= 0.0) res = 1.0;
    const double ox = cfg.offset.x.numerical_value_in(cm);
    const double oy = cfg.offset.y.numerical_value_in(cm);
    const double oz = cfg.offset.z.numerical_value_in(cm);

    const long long ix0 = std::llround((cfg.boundaries.min_x.numerical_value_in(cm) - ox) / res);
    const long long ix1 = std::llround((cfg.boundaries.max_x.numerical_value_in(cm) - ox) / res);
    const long long iy0 = std::llround((cfg.boundaries.min_y.numerical_value_in(cm) - oy) / res);
    const long long iy1 = std::llround((cfg.boundaries.max_y.numerical_value_in(cm) - oy) / res);
    const long long ih0 = std::llround((cfg.boundaries.min_height.numerical_value_in(cm) - oz) / res);
    const long long ih1 = std::llround((cfg.boundaries.max_height.numerical_value_in(cm) - oz) / res);

    auto inBounds = [&](const GridKey& k) {
        return k.x >= ix0 && k.x <= ix1 && k.y >= iy0 && k.y <= iy1 && k.z >= ih0 && k.z <= ih1;
    };

    for (long long x = ix0; x <= ix1; ++x) {
        for (long long y = iy0; y <= iy1; ++y) {
            for (long long z = ih0; z <= ih1; ++z) {
                const GridKey key{x, y, z};
                const bool is_empty =
                    output_map_.atVoxel(gridToWorld(key)) == types::VoxelOccupancy::Empty ||
                    visited_.count(key) > 0;
                if (!is_empty) {
                    continue;
                }
                for (const auto& n : kNeighbors) {
                    const GridKey nk{x + n[0], y + n[1], z + n[2]};
                    if (!inBounds(nk) || unreachable_.count(nk) > 0) {
                        continue;
                    }
                    if (output_map_.atVoxel(gridToWorld(nk)) == types::VoxelOccupancy::Unmapped) {
                        frontier_.insert(nk);
                    }
                }
            }
        }
    }
}

bool MappingAlgorithmImpl::selectNearestFrontier(const types::DroneState& state) {
    if (frontier_.empty()) {
        have_target_ = false;
        return false;
    }
    const GridKey cur = worldToGrid(state.position);
    GridKey best = *frontier_.begin();
    double best_dist = std::numeric_limits<double>::max();
    for (const GridKey& cell : frontier_) {
        const double dx = static_cast<double>(cell.x - cur.x);
        const double dy = static_cast<double>(cell.y - cur.y);
        const double dz = static_cast<double>(cell.z - cur.z);
        const double dist = dx * dx + dy * dy + dz * dz;
        if (dist < best_dist) {
            best_dist = dist;
            best = cell;
        }
    }
    current_target_ = best;
    have_target_ = true;
    return true;
}

void MappingAlgorithmImpl::planPath(const types::DroneState& state) {
    move_queue_.clear();

    const Position3D tw = gridToWorld(current_target_);
    const double cx = state.position.x.numerical_value_in(cm);
    const double cy = state.position.y.numerical_value_in(cm);
    const double cz = state.position.z.numerical_value_in(cm);
    const double dx = tw.x.numerical_value_in(cm) - cx;
    const double dy = tw.y.numerical_value_in(cm) - cy;
    const double dz = tw.z.numerical_value_in(cm) - cz;
    const double horiz = std::sqrt(dx * dx + dy * dy);

    const double max_elev = drone_config_.max_elevate.numerical_value_in(cm);
    const double max_adv = drone_config_.max_advance.numerical_value_in(cm);
    const double max_rot = drone_config_.max_rotate.numerical_value_in(deg);

    if (std::abs(dz) > kMinMoveCm) {
        double e = std::min(std::abs(dz), max_elev);
        if (dz < 0.0) e = -e;
        move_queue_.push_back(types::MovementCommand{
            types::MovementCommandType::Elevate, types::RotationDirection::Left, {}, e * cm});
    }

    if (horiz > kMinMoveCm) {
        double target_deg = std::atan2(dy, dx) / kDegToRad;
        if (target_deg < 0.0) target_deg += kFullCircleDeg;

        const double cur_deg = state.heading.horizontal.numerical_value_in(deg);
        double diff = target_deg - cur_deg;
        while (diff > kHalfCircleDeg) diff -= kFullCircleDeg;
        while (diff < -kHalfCircleDeg) diff += kFullCircleDeg;

        if (std::abs(diff) > kMinRotDeg) {
            const double rot = std::min(std::abs(diff), max_rot);
            move_queue_.push_back(types::MovementCommand{
                types::MovementCommandType::Rotate,
                (diff > 0.0) ? types::RotationDirection::Left : types::RotationDirection::Right,
                rot * horizontal_angle[deg], {}});
        }

        const double adv = std::min(horiz, max_adv);
        move_queue_.push_back(types::MovementCommand{
            types::MovementCommandType::Advance, types::RotationDirection::Left, {}, adv * cm});
    }
}

types::AlgorithmStatus MappingAlgorithmImpl::finishStatus() const {
    return unreachable_.empty() ? types::AlgorithmStatus::Finished
                                : types::AlgorithmStatus::FinishedWithUnmappableVoxels;
}

// Scanning phase: emit the next scan orientation, or (sweep done) recompute the
// frontier, pick a target and plan a path toward it.
MappingAlgorithmImpl::StepOutcome
MappingAlgorithmImpl::runScanningPhase(const types::DroneState& state) {
    using types::AlgorithmStatus;
    using types::MappingStepCommand;

    if (scan_index_ < kTotalScans) {
        const Orientation ori = scanOrientation(scan_index_);
        ++scan_index_;
        return {StepFlow::Return, MappingStepCommand{std::nullopt, ori, AlgorithmStatus::Working}};
    }

    // Sweep complete: this cell is now scanned; recompute the frontier.
    scan_index_ = 0;
    visited_.insert(worldToGrid(state.position));
    recomputeFrontier(state);

    if (frontier_.empty() || !selectNearestFrontier(state)) {
        finished_ = true;
        return {StepFlow::Return, MappingStepCommand{std::nullopt, std::nullopt, finishStatus()}};
    }

    planPath(state);
    if (move_queue_.empty()) {
        // Can't head toward this target -> drop it and pick another.
        unreachable_.insert(current_target_);
        frontier_.erase(current_target_);
        have_target_ = false;
        return {StepFlow::Continue, {}};
    }

    phase_ = Phase::Moving;
    last_pos_ = worldToGrid(state.position);
    plan_start_pos_ = last_pos_;
    stuck_ = 0;
    return {StepFlow::FallThrough, {}};
}

// Moving phase: emit the next movement command, or give up on the target if the
// drone is stuck / made no progress, returning to scanning.
MappingAlgorithmImpl::StepOutcome
MappingAlgorithmImpl::runMovingPhase(const types::DroneState& state) {
    using types::AlgorithmStatus;
    using types::MappingStepCommand;
    using types::MovementCommand;

    const GridKey cur = worldToGrid(state.position);
    if (cur == last_pos_) {
        ++stuck_;
    } else {
        stuck_ = 0;
    }
    last_pos_ = cur;

    const bool stuck = stuck_ >= kMaxStuck;
    const bool no_progress = move_queue_.empty() && have_target_ && cur == plan_start_pos_;
    if (stuck || no_progress) {
        // Blocked: abandon this target so we don't re-select it forever.
        if (have_target_) {
            unreachable_.insert(current_target_);
            frontier_.erase(current_target_);
        }
        move_queue_.clear();
        have_target_ = false;
        stuck_ = 0;
        phase_ = Phase::Scanning;
        scan_index_ = 0;
        return {StepFlow::Continue, {}};
    }

    if (move_queue_.empty()) {
        // Plan consumed and the drone did move -> rescan from the new spot.
        phase_ = Phase::Scanning;
        scan_index_ = 0;
        return {StepFlow::Continue, {}};
    }

    const MovementCommand mv = move_queue_.front();
    move_queue_.pop_front();
    return {StepFlow::Return, MappingStepCommand{mv, std::nullopt, AlgorithmStatus::Working}};
}

types::MappingStepCommand
MappingAlgorithmImpl::nextStep(const types::DroneState& state,
                              const types::LidarScanResult* latest_scan) {
    (void)latest_scan; // planning reads output_map_, which DroneControl keeps current

    if (finished_) {
        return types::MappingStepCommand{std::nullopt, std::nullopt, finishStatus()};
    }

    // Bound internal phase transitions per call so we always make progress.
    for (int guard = 0; guard < 4; ++guard) {
        if (phase_ == Phase::Scanning) {
            const StepOutcome r = runScanningPhase(state);
            if (r.flow == StepFlow::Return) {
                return r.command;
            }
            if (r.flow == StepFlow::Continue) {
                continue;
            }
            // FallThrough -> drop into the moving phase below.
        }

        const StepOutcome r = runMovingPhase(state);
        if (r.flow == StepFlow::Return) {
            return r.command;
        }
        // Continue (the moving phase never falls through).
    }

    // Churned through transitions without a concrete command -> hover this step.
    return types::MappingStepCommand{types::MovementCommand{types::MovementCommandType::Hover},
                                     std::nullopt, types::AlgorithmStatus::Working};
}

} // namespace algorithm_207637604_325750099

// ---------------------------------------------------------------------------
// Self-registration. The macro instantiates a global object whose constructor
// runs when this .so is dlopen'd, handing our factory to the simulator's
// registrar (defined in the Simulator project). The id-suffixed alias makes the
// generated `register_me_...` symbol unique across all loaded plugins.
// ---------------------------------------------------------------------------
using MappingAlgorithmImpl_207637604_325750099 =
    algorithm_207637604_325750099::MappingAlgorithmImpl;

REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl_207637604_325750099);
