#pragma once

#include <Common/IMutableMap3D.h>
#include <Common/Types.h>

namespace mission_control_207637604_325750099 {

// Applies a LiDAR scan directly to the output map, writing only observation
// states: Empty along the rays, Occupied at hits, and PotentiallyOccupied for
// too-close (distance 0) hits.
class ScanResultToVoxels {
public:
    static void applyToMap(common::IMutableMap3D& output_map,
                           const common::Position3D& scan_origin,
                           const common::Orientation& drone_heading,
                           const common::types::LidarScanResult& scan,
                           const common::types::LidarConfigData& lidar_config);
};

} // namespace mission_control_207637604_325750099
