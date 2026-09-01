cmake_minimum_required(VERSION 3.15)
project(assignment_3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# The lidar ray-marching is compute-heavy; an unoptimized (-O0) build makes runs
# many times slower. Default to an optimized build when the caller did not pick a
# configuration (an explicit -DCMAKE_BUILD_TYPE=... still wins).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(mp-units CONFIG REQUIRED)

function(drone_warnings target)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -pedantic)
endfunction()

# UserCommon has NO build file of its own (per the assignment). It is a
# header-only INTERFACE target defined here so the three projects can link it
# without duplicating its include path. Headers live in UserCommon/include and
# use the namespace user_common_207637604_325750099.
add_library(user_common INTERFACE)
add_library(user_common::user_common ALIAS user_common)
target_include_directories(user_common INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/UserCommon/include>
)
target_link_libraries(user_common INTERFACE common::common)

add_subdirectory(common)
add_subdirectory(Algorithm)
add_subdirectory(MissionControl)
add_subdirectory(Simulator)
