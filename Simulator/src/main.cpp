#include <Simulator/ConfigLoader.h>
#include <Simulator/ParallelExecutor.h>
#include <Simulator/PluginLoader.h>
#include <Simulator/ReportWriter.h>
#include <Simulator/SimulationRunFactoryImpl.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <exception>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

enum class Mode { None, Comparative, Competitive };

struct Options {
    Mode mode = Mode::None;
    fs::path simulation;
    fs::path mission_control_folder; // comparative
    fs::path algorithm;              // comparative
    fs::path mission_control;        // competitive
    fs::path algorithms_folder;      // competitive
    std::optional<int> num_threads;
    bool verbose = false;
};

void printUsage(std::ostream& os) {
    os << "Usage:\n"
       << "  simulator_207637604_325750099 -comparative simulation=<yaml> "
          "mission_control_folder=<folder> algorithm=<so> [num_threads=<n>] [-verbose]\n"
       << "  simulator_207637604_325750099 -competition simulation=<yaml> "
          "mission_control=<so> algorithms_folder=<folder> [num_threads=<n>] [-verbose]\n";
}

[[nodiscard]] std::string timestamp() {
    const std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return std::string{buf};
}

[[nodiscard]] std::vector<fs::path> listSharedObjects(const fs::path& folder) {
    std::vector<fs::path> result;
    if (!fs::is_directory(folder)) {
        return result;
    }
    for (const auto& entry : fs::directory_iterator(folder)) {
        if (entry.is_regular_file() && entry.path().extension() == ".so") {
            result.push_back(entry.path());
        }
    }
    std::sort(result.begin(), result.end());
    return result;
}

[[nodiscard]] bool parseArgs(int argc, char** argv, Options& opts, std::vector<std::string>& errors) {
    std::map<std::string, std::string> kv;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-comparative") {
            opts.mode = Mode::Comparative;
        } else if (arg == "-competition") {
            opts.mode = Mode::Competitive;
        } else if (arg == "-verbose") {
            opts.verbose = true;
        } else {
            const auto eq = arg.find('=');
            if (eq == std::string::npos) {
                errors.push_back("unsupported argument: " + arg);
                continue;
            }
            kv[arg.substr(0, eq)] = arg.substr(eq + 1);
        }
    }

    const std::vector<std::string> comparative_keys = {"simulation", "mission_control_folder",
                                                       "algorithm", "num_threads"};
    const std::vector<std::string> competitive_keys = {"simulation", "mission_control",
                                                       "algorithms_folder", "num_threads"};
    const auto& allowed = (opts.mode == Mode::Competitive) ? competitive_keys : comparative_keys;
    for (const auto& [key, value] : kv) {
        if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
            errors.push_back("unsupported argument: " + key + "=" + value);
        }
    }

    auto get = [&](const std::string& k) -> std::optional<std::string> {
        auto it = kv.find(k);
        return it == kv.end() ? std::nullopt : std::optional<std::string>{it->second};
    };

    if (auto n = get("num_threads")) {
        try {
            opts.num_threads = std::stoi(*n);
        } catch (const std::exception&) {
            errors.push_back("num_threads must be an integer");
        }
    }
    if (auto s = get("simulation")) opts.simulation = *s;
    if (opts.mode == Mode::Comparative) {
        if (auto v = get("mission_control_folder")) opts.mission_control_folder = *v;
        if (auto v = get("algorithm")) opts.algorithm = *v;
    } else if (opts.mode == Mode::Competitive) {
        if (auto v = get("mission_control")) opts.mission_control = *v;
        if (auto v = get("algorithms_folder")) opts.algorithms_folder = *v;
    }
    return errors.empty();
}

[[nodiscard]] bool validate(const Options& opts, std::vector<std::string>& errors) {
    if (opts.mode == Mode::None) {
        errors.push_back("missing mode: -comparative or -competition is required");
        return false;
    }
    auto requireFile = [&](const fs::path& p, const std::string& what) {
        if (p.empty()) errors.push_back("missing argument: " + what);
        else if (!fs::is_regular_file(p)) errors.push_back(what + " is not a readable file: " + p.string());
    };
    auto requireFolderWithSo = [&](const fs::path& p, const std::string& what) {
        if (p.empty()) errors.push_back("missing argument: " + what);
        else if (!fs::is_directory(p)) errors.push_back(what + " is not a folder: " + p.string());
        else if (listSharedObjects(p).empty()) errors.push_back(what + " contains no .so plugins: " + p.string());
    };
    requireFile(opts.simulation, "simulation");
    if (opts.mode == Mode::Comparative) {
        requireFolderWithSo(opts.mission_control_folder, "mission_control_folder");
        requireFile(opts.algorithm, "algorithm");
    } else {
        requireFile(opts.mission_control, "mission_control");
        requireFolderWithSo(opts.algorithms_folder, "algorithms_folder");
    }
    return errors.empty();
}

// A fixed (algorithm, mission-control) pairing with its own run factory.
struct Pairing {
    std::string name;
    std::unique_ptr<simulator::SimulationRunFactoryImpl> factory;
};

[[nodiscard]] simulator::types::SimulationResult
failedResult(const char* code, const std::string& what) {
    simulator::types::SimulationResult r;
    r.mission_score = -1.0;
    r.mission_results.push_back(common::types::MissionRunResult{
        common::types::MissionRunStatus::Error, 0, {common::types::ErrorRef{code, what}}});
    return r;
}

// Create + run one simulation, turning any failure into a scored (-1) result.
[[nodiscard]] simulator::types::SimulationResult
runOne(simulator::ISimulationRunFactory& factory,
       const simulator::types::SimulationConfigData& sim,
       const common::types::MissionConfigData& mission,
       const common::types::DroneConfigData& drone,
       const common::types::LidarConfigData& lidar,
       const fs::path& out) {
    try {
        auto run = factory.create(sim, mission, drone, lidar, out);
        if (run) {
            return run->run();
        }
        return failedResult("RUN_CREATE_ERROR", "factory returned a null run");
    } catch (const std::invalid_argument& e) {
        return failedResult("MISSION_BOUNDARY_INVALID", e.what());
    } catch (const std::exception& e) {
        return failedResult("RUN_CREATE_ERROR", e.what());
    }
}

// One unit of work: run (entry.results[slot]) with the pairing's factory.
struct RunTask {
    simulator::ISimulationRunFactory* factory;
    const simulator::types::SimulationConfigData* sim;
    const common::types::MissionConfigData* mission;
    const common::types::DroneConfigData* drone;
    const common::types::LidarConfigData* lidar;
    std::size_t entry;
    std::size_t slot;
};

// Build the full (known-in-advance) run table and execute it in parallel. Each
// task writes to its own pre-sized slot, so no locking is needed.
[[nodiscard]] std::vector<simulator::ReportWriter::ModeRunResult>
runMatrix(std::vector<Pairing>& pairings,
          const std::vector<simulator::types::SimulationCompositionData>& compositions,
          const fs::path& out_dir,
          std::optional<int> num_threads) {
    std::vector<simulator::ReportWriter::ModeRunResult> entries(pairings.size());
    std::vector<RunTask> tasks;

    for (std::size_t p = 0; p < pairings.size(); ++p) {
        entries[p].plugin_name = pairings[p].name;
        std::size_t slot = 0;
        for (const auto& composition : compositions) {
            for (const auto& group : composition.simulation_mission_groups) {
                const auto& sim = std::get<0>(group);
                const auto& missions = std::get<1>(group);
                for (const auto& mission : missions) {
                    for (const auto& drone : composition.drone_configs) {
                        for (const auto& lidar : composition.lidar_configs) {
                            tasks.push_back(RunTask{pairings[p].factory.get(), &sim, &mission,
                                                    &drone, &lidar, p, slot});
                            ++slot;
                        }
                    }
                }
            }
        }
        entries[p].results.resize(slot);
    }

    simulator::ParallelExecutor::run(tasks.size(), num_threads, [&](std::size_t k) {
        const RunTask& t = tasks[k];
        entries[t.entry].results[t.slot] =
            runOne(*t.factory, *t.sim, *t.mission, *t.drone, *t.lidar, out_dir);
    });

    return entries;
}

int runComparative(const Options& opts, simulator::PluginLoader& loader) {
    auto algorithm = loader.loadAlgorithm(opts.algorithm);
    if (!algorithm) {
        std::cerr << "failed to load algorithm: " << loader.lastError() << "\n";
        return 1;
    }
    const auto compositions = simulator::ConfigLoader::loadComposition(opts.simulation);
    const fs::path out_dir = opts.mission_control_folder / ("comparative_results_" + timestamp());
    fs::create_directories(out_dir);

    std::vector<Pairing> pairings;
    std::vector<std::string> failed_plugins;
    for (const fs::path& mc_so : listSharedObjects(opts.mission_control_folder)) {
        auto mission_control = loader.loadMissionControl(mc_so);
        if (!mission_control) {
            std::cerr << "skipping mission control " << mc_so.filename() << ": "
                      << loader.lastError() << "\n";
            failed_plugins.push_back(mc_so.filename().string());
            continue;
        }
        pairings.push_back(Pairing{mc_so.filename().string(),
                                   std::make_unique<simulator::SimulationRunFactoryImpl>(
                                       *algorithm, *mission_control, opts.verbose)});
    }

    auto entries = runMatrix(pairings, compositions, out_dir, opts.num_threads);
    simulator::ReportWriter::writeComparativeReport(entries, failed_plugins,
                                                    opts.simulation, opts.mission_control_folder, out_dir);
    std::cout << "comparative results written to " << out_dir << "\n";
    return 0;
}

int runCompetitive(const Options& opts, simulator::PluginLoader& loader) {
    auto mission_control = loader.loadMissionControl(opts.mission_control);
    if (!mission_control) {
        std::cerr << "failed to load mission control: " << loader.lastError() << "\n";
        return 1;
    }
    const auto compositions = simulator::ConfigLoader::loadComposition(opts.simulation);
    const fs::path out_dir = opts.algorithms_folder / ("competition_" + timestamp());
    fs::create_directories(out_dir);

    std::vector<Pairing> pairings;
    std::vector<std::string> failed_plugins;
    for (const fs::path& algo_so : listSharedObjects(opts.algorithms_folder)) {
        auto algorithm = loader.loadAlgorithm(algo_so);
        if (!algorithm) {
            std::cerr << "skipping algorithm " << algo_so.filename() << ": "
                      << loader.lastError() << "\n";
            failed_plugins.push_back(algo_so.filename().string());
            continue;
        }
        pairings.push_back(Pairing{algo_so.filename().string(),
                                   std::make_unique<simulator::SimulationRunFactoryImpl>(
                                       *algorithm, *mission_control, opts.verbose)});
    }

    auto entries = runMatrix(pairings, compositions, out_dir, opts.num_threads);
    simulator::ReportWriter::writeCompetitiveReport(entries, failed_plugins,
                                                    opts.simulation, opts.mission_control, out_dir);
    std::cout << "competition results written to " << out_dir << "\n";
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    Options opts;
    std::vector<std::string> errors;
    if (!parseArgs(argc, argv, opts, errors) || !validate(opts, errors)) {
        printUsage(std::cerr);
        for (const auto& e : errors) {
            std::cerr << "error: " << e << "\n";
        }
        return 2;
    }

    // Declared first so it is destroyed LAST: every factory copy and created
    // instance (all local to the mode runners) is gone before its destructor
    // clears the registrar and dlclose's the plugins.
    simulator::PluginLoader loader;
    try {
        return (opts.mode == Mode::Comparative) ? runComparative(opts, loader)
                                                : runCompetitive(opts, loader);
    } catch (const std::exception& e) {
        std::cerr << "fatal: " << e.what() << "\n";
        return 1;
    }
}
