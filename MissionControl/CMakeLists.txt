# MissionControl plugin: built as a SHARED library the Simulator loads at
# runtime. Output name has no "lib" prefix -> MissionControl_207637604_325750099.so.
# Like the Algorithm plugin it leaves the registration ctor undefined (resolved
# by the Simulator at load time).

add_library(MissionControl_207637604_325750099 SHARED
    src/ScanResultToVoxels.cpp
    src/DroneControlImpl.cpp
    src/MissionControlImpl.cpp
)

set_target_properties(MissionControl_207637604_325750099 PROPERTIES
    PREFIX ""
    CXX_VISIBILITY_PRESET default
)

target_include_directories(MissionControl_207637604_325750099 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_mission_control/include
)

target_link_libraries(MissionControl_207637604_325750099 PRIVATE common::common)

drone_warnings(MissionControl_207637604_325750099)
