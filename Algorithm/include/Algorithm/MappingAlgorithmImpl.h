#pragma once

#include <Common/IMappingAlgorithm.h>

#include <deque>
#include <set>

namespace algorithm_207637604_325750099 {

// Frontier-based explorer. Reads (never writes) the shared output map: it scans
// from the current pose, marks the cell visited, recomputes the Unmapped
// frontier, drives toward the nearest frontier cell within the drone limits,
// and abandons unreachable targets until nothing is left to map.
class MappingAlgorithmImpl final : public common::IMappingAlgorithm {
public:
    using common::IMappingAlgorithm::IMappingAlgorithm; // inherits the Dependencies ctor

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& state,
        const common::types::LidarScanResult* latest_scan) override;

private:
    struct GridKey {
        long long x, y, z;
        bool operator<(const GridKey& o) const {
            if (x != o.x) return x < o.x;
            if (y != o.y) return y < o.y;
            return z < o.z;
        }
        bool operator==(const GridKey& o) const {
            return x == o.x && y == o.y && z == o.z;
        }
    };

    enum class Phase { Scanning, Moving };
    enum class StepFlow { Return, Continue, FallThrough };
    struct StepOutcome {
        StepFlow flow;
        common::types::MappingStepCommand command;
    };

    [[nodiscard]] GridKey worldToGrid(const common::Position3D& pos) const;
    [[nodiscard]] common::Position3D gridToWorld(const GridKey& key) const;
    void recomputeFrontier(const common::types::DroneState& state);
    bool selectNearestFrontier(const common::types::DroneState& state);
    void planPath(const common::types::DroneState& state);
    [[nodiscard]] common::types::AlgorithmStatus finishStatus() const;
    [[nodiscard]] StepOutcome runScanningPhase(const common::types::DroneState& state);
    [[nodiscard]] StepOutcome runMovingPhase(const common::types::DroneState& state);

    Phase phase_ = Phase::Scanning;
    std::size_t scan_index_ = 0;
    bool finished_ = false;
    std::set<GridKey> visited_;
    std::set<GridKey> frontier_;
    std::set<GridKey> unreachable_;
    bool have_target_ = false;
    GridKey current_target_{};
    std::deque<common::types::MovementCommand> move_queue_;
    GridKey last_pos_{};
    GridKey plan_start_pos_{};
    int stuck_ = 0;
};

} // namespace algorithm_207637604_325750099
