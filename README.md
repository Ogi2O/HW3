# Assignment 3 - Drone Mapper

Submitters:

- Alon Ben David - 207637604
- Stav Shemesh - 325750099

## Project layout & namespaces

Five folders: three buildable projects (`Simulator`, `MissionControl`,
`Algorithm`) plus two common folders (`common`, `UserCommon`).

| Folder          | Kind                         | Namespace                              |
|-----------------|------------------------------|----------------------------------------|
| `common`        | staff-provided, use as-is    | `common`                               |
| `UserCommon`    | our shared headers           | `user_common_207637604_325750099`      |
| `Algorithm`     | shared library (`.so`)       | `algorithm_207637604_325750099`        |
| `MissionControl`| shared library (`.so`)       | `mission_control_207637604_325750099`  |
| `Simulator`     | executable                   | `simulator`                            |

Produced artifacts:

- `Algorithm_207637604_325750099.so`
- `MissionControl_207637604_325750099.so`
- `simulator_207637604_325750099`

> The `common` folder is used as-is - do not add files to it. Shared code we
> write goes in `UserCommon`.

## Provided file tree

```text
.
|-- .devcontainer/...
|-- Algorithm/
|   |-- CMakeLists.txt
|   |-- include/Algorithm/
|   `-- src/
|-- MissionControl/
|   |-- CMakeLists.txt
|   |-- common_mission_control/include/MissionControl/IDroneControl.h
|   |-- include/MissionControl/
|   `-- src/
|-- Simulator/
|   |-- CMakeLists.txt
|   |-- common_simulator/include/Simulator/
|   |   |-- ISimulation.h
|   |   |-- ISimulationRun.h
|   |   |-- ISimulationRunFactory.h
|   |   `-- SimulationTypes.h
|   |-- include/Simulator/
|   `-- src/
|-- UserCommon/
|   |-- CMakeLists.txt
|   `-- include/UserCommon/UserCommon.h
|-- common/
|   |-- CMakeLists.txt
|   `-- include/Common/
|       |-- types/
|       |   |-- DroneTypes.h
|       |   |-- LidarTypes.h
|       |   |-- MapTypes.h
|       |   `-- MissionTypes.h
|       |-- IDroneMovement.h
|       |-- IGPS.h
|       |-- ILidar.h
|       |-- IMap3D.h
|       |-- IMappingAlgorithm.h
|       |-- IMissionControl.h
|       |-- IMutableMap3D.h
|       |-- MappingAlgorithmFactory.h
|       |-- MappingAlgorithmRegistration.h
|       |-- MissionControlFactory.h
|       |-- MissionControlRegistration.h
|       |-- Types.h
|       `-- Units.h
|-- .gitignore
|-- CMakeLists.txt
|-- CMakePresets.json
|-- README.md
|-- students.txt
|-- vcpkg-configuration.json
`-- vcpkg.json
```

## Building

The project uses CMake + vcpkg (mp-units, yaml-cpp, tinynpy, gtest), as wired in
the provided `.devcontainer` / `CMakePresets.json`:

```sh
cmake --preset default
cmake --build --preset default
```

This produces the two plugins and the executable:

- `Algorithm_207637604_325750099.so`
- `MissionControl_207637604_325750099.so`
- `simulator_207637604_325750099`

## Running

Both modes take all arguments in any order; `=` has no surrounding spaces.
`num_threads` and `-verbose` are optional.

Comparative (one algorithm vs. every mission control in a folder):

```sh
./simulator_207637604_325750099 -comparative \
    simulation=inputs/sim_compose.yaml \
    mission_control_folder=<folder-of-.so> \
    algorithm=Algorithm_207637604_325750099.so \
    [num_threads=<n>] [-verbose]
```

Competitive (one mission control vs. every algorithm in a folder):

```sh
./simulator_207637604_325750099 -competition \
    simulation=inputs/sim_compose.yaml \
    mission_control=MissionControl_207637604_325750099.so \
    algorithms_folder=<folder-of-.so> \
    [num_threads=<n>] [-verbose]
```

Outputs are written under a timestamped folder (`comparative_results_<time>` /
`competition_<time>`): the per-run output maps, per-run error logs, and a summary
YAML (`comparative_results.yaml` / `competition_results.yaml`).

### Threads

`num_threads` is the number of worker threads **in addition to** the main
thread. Missing or `1` runs everything on the main thread; the total thread count
is never exactly `2`, and no worker is spawned without work to do.

## Design notes

- **Plugins & self-registration.** `Algorithm` and `MissionControl` build as
  `.so` files. Each registers its factory at load time via
  `REGISTER_MAPPING_ALGORITHM` / `REGISTER_MISSION_CONTROL`; the registration
  constructors are defined only in the Simulator (`Registration.cpp`) and
  resolved when the plugin is `dlopen`'d (the executable is built with exported
  symbols). The `Registrar` collects the factories and `PluginLoader` hands them
  back. Plugins are `dlclose`'d only after every created instance and factory
  copy is destroyed (the loader is created first in `main`, destroyed last).
- **MissionControl owns its DroneControl.** In ex3 the MissionControl receives
  the hardware mocks + algorithm through `MissionControlDependencies` and builds
  its own `DroneControlImpl` (taking the lidar config from `lidar.config()`).
- **Run graph & lifetimes.** `SimulationRunImpl` owns the maps, mocks, algorithm
  and mission control; member order guarantees the mission control (and its inner
  drone control) is destroyed before the algorithm/mocks it references.
- **Robustness.** A driver/algorithm exception during a step is caught and turned
  into a mission error (the map is still saved); a run that fails to create is
  scored `-1` and the batch continues - the simulator does not crash.

## Tests (development only)

Not part of the graded build. Enable with:

```sh
cmake --preset default -DBUILD_TESTS=ON
cmake --build --preset default
ctest --preset default    # or run ./drone_tests and ./plugin_tests
```

- `drone_tests` - component tests (mocks, maps, comparison, drone/mission
  control, algorithm, manager/run, parallel executor), covering normal paths and
  the edge cases that map to the assignment-2 feedback.
- `plugin_tests` - integration tests that `dlopen` the real built plugins and
  create instances through the registered factories.
