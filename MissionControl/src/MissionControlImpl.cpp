#include <MissionControl/MissionControlImpl.h>

#include <Common/MissionControlRegistration.h>

#include <exception>
#include <fstream>
#include <utility>

namespace mission_control_207637604_325750099 {

using namespace common;

namespace {

// Error/verbose lines for a run are written next to its output map, e.g.
// "out.npy" -> "out.npy.log".
[[nodiscard]] std::filesystem::path deriveLogPath(std::filesystem::path output_map_file) {
    output_map_file += ".log";
    return output_map_file;
}

} // namespace

MissionControlImpl::MissionControlImpl(MissionControlDependencies deps)
    : mission_(deps.mission_config),
      drone_(deps.drone_config),
      output_map_(deps.output_map),
      output_map_file_(std::move(deps.output_map_file)),
      verbose_(deps.verbose),
      error_log_path_(deriveLogPath(output_map_file_)),
      // The MissionControl builds its own DroneControl; the lidar's config comes
      // straight from the lidar rather than a separate parameter.
      drone_control_(deps.drone_config, deps.mission_config,
                     deps.lidar, deps.gps, deps.movement,
                     deps.output_map, deps.mapping_algorithm,
                     deps.lidar.config()) {}

void MissionControlImpl::logImmediately(const types::ErrorRef& error) const {
    if (error_log_path_.empty()) {
        return;
    }
    std::ofstream log(error_log_path_, std::ios::app);
    log << "[ERROR] " << error.code << " - " << error.message << "\n";
}

types::MissionRunResult MissionControlImpl::runMission() {
    types::MissionRunResult result;
    std::size_t steps = 0;

    while (true) {
        // max_steps == 0 is treated as "no explicit limit" (the algorithm is
        // expected to terminate once the frontier is exhausted).
        if (mission_.max_steps != 0 && steps >= mission_.max_steps) {
            result.status = types::MissionRunStatus::MaxSteps;
            break;
        }

        const types::DroneStepResult step = drone_control_.step();
        ++steps;

        if (step.status == types::DroneStepStatus::Error) {
            // BONUS: Log the error and continue (don't break)
            const types::ErrorRef err{"DRONE_STEP_ERROR", step.message};
            logImmediately(err);
            result.errors.push_back(err);
            // Continue running - don't break on error
        }
        if (step.status == types::DroneStepStatus::Completed) {
            result.status = types::MissionRunStatus::Completed;
            break;
        }
        // Continue -> next step.
    }

    result.steps = steps;

    // If there were errors during the run, mark status accordingly
    if (!result.errors.empty() && result.status != types::MissionRunStatus::MaxSteps) {
        result.status = types::MissionRunStatus::Error;
    }

    // Persist the produced map (best effort; a failure is recorded, not thrown).
    try {
        output_map_.save(output_map_file_);
    } catch (const std::exception& e) {
        const types::ErrorRef err{"MAP_SAVE_ERROR", e.what()};
        logImmediately(err);
        result.errors.push_back(err);
        if (result.status != types::MissionRunStatus::Error) {
            result.status = types::MissionRunStatus::Error;
        }
    }

    if (verbose_ && !error_log_path_.empty()) {
        std::ofstream log(error_log_path_, std::ios::app);
        log << "[INFO] mission finished: steps=" << result.steps << "\n";
    }

    return result;
}

} // namespace mission_control_207637604_325750099

// ---------------------------------------------------------------------------
// Self-registration (see the Algorithm plugin for the rationale). The
// id-suffixed alias keeps the generated symbol unique across loaded plugins.
// ---------------------------------------------------------------------------
using MissionControlImpl_207637604_325750099 =
    mission_control_207637604_325750099::MissionControlImpl;

REGISTER_MISSION_CONTROL(MissionControlImpl_207637604_325750099);
