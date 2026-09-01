# Algorithm plugin: built as a SHARED library that the Simulator loads at
# runtime with dlopen. The output name has no "lib" prefix, so the artifact is
# exactly Algorithm_207637604_325750099.so.
#
# The library intentionally leaves one symbol undefined - the
# common::MappingAlgorithmRegistration constructor - which the Simulator defines
# and exports; it is resolved when the .so is loaded. (A shared library allows
# undefined symbols by default, so no extra link flags are needed here.)

add_library(Algorithm_207637604_325750099 SHARED
    src/MappingAlgorithmImpl.cpp
)

set_target_properties(Algorithm_207637604_325750099 PROPERTIES
    PREFIX ""                       # drop the "lib" prefix
    CXX_VISIBILITY_PRESET default   # keep the registration ctor's init visible
)

target_include_directories(Algorithm_207637604_325750099 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(Algorithm_207637604_325750099 PRIVATE common::common)

drone_warnings(Algorithm_207637604_325750099)
