#pragma once

#include <Common/IMissionControl.h>
#include <Common/IMutableMap3D.h>
#include <Common/MissionControlFactory.h> // MissionControlDependencies
#include <MissionControl/DroneControlImpl.h>

#include <filesystem>

namespace mission_control_207637604_325750099 {

// Drives one mission: repeatedly steps its (internally owned) DroneControl until
// Completed / Error / max_steps, then persists the produced map. In ex3 the
// MissionControl builds its own DroneControl from the injected dependencies
// (lidar/gps/movement/output map/algorithm) rather than receiving one.
class MissionControlImpl final : public common::IMissionControl {
public:
    explicit MissionControlImpl(common::MissionControlDependencies dependencies);

    [[nodiscard]] common::types::MissionRunResult runMission() override;

private:
    void logImmediately(const common::types::ErrorRef& error) const;

    common::types::MissionConfigData mission_;
    common::types::DroneConfigData drone_;
    common::IMutableMap3D& output_map_;
    std::filesystem::path output_map_file_;
    bool verbose_;
    std::filesystem::path error_log_path_;
    DroneControlImpl drone_control_; // owned; built from the dependencies
};

} // namespace mission_control_207637604_325750099
