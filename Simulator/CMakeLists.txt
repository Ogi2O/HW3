# The Simulator executable. It loads the Algorithm / MissionControl plugins at
# runtime with dlopen, so it must EXPORT its symbols (ENABLE_EXPORTS) - that is
# how each plugin's undefined registration-ctor symbol is resolved on load. It
# also defines those ctors (Registration.cpp) and owns the Registrar/loader.

find_package(yaml-cpp CONFIG REQUIRED)
find_package(tinynpy CONFIG REQUIRED)
find_package(Threads REQUIRED)

add_executable(simulator_207637604_325750099
    src/Map3DImpl.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/MapsComparison.cpp
    src/SimulationManager.cpp
    src/SimulationRunImpl.cpp
    src/SimulationRunFactoryImpl.cpp
    src/ConfigLoader.cpp
    src/ReportWriter.cpp
    src/ParallelExecutor.cpp
    src/Registrar.cpp
    src/Registration.cpp
    src/PluginLoader.cpp
    src/main.cpp
)

# Export symbols so dlopen'd plugins can resolve the registration constructors.
set_target_properties(simulator_207637604_325750099 PROPERTIES ENABLE_EXPORTS ON)

target_include_directories(simulator_207637604_325750099 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
)

target_link_libraries(simulator_207637604_325750099 PRIVATE
    common::common
    user_common::user_common
    TinyNPY::TinyNPY
    yaml-cpp::yaml-cpp
    Threads::Threads
    ${CMAKE_DL_LIBS}
)

drone_warnings(simulator_207637604_325750099)
