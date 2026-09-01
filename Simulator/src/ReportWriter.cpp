#include <Simulator/ReportWriter.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <ctime>
#include <fstream>
#include <limits>
#include <numeric>
#include <string>
#include <tuple>
#include <vector>

namespace simulator {

using namespace common; // for the `cm` unit symbol (NOT the `types` namespace)

namespace {

[[nodiscard]] std::string nowUtcIso8601() {
    const std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
    return std::string{buf};
}

[[nodiscard]] std::string statusString(const types::SimulationResult& r) {
    if (r.mission_results.empty()) {
        return "error";
    }
    switch (r.mission_results.front().status) {
        case common::types::MissionRunStatus::Completed: return "completed";
        case common::types::MissionRunStatus::MaxSteps:  return "max_steps";
        case common::types::MissionRunStatus::Error:     return "error";
    }
    return "error";
}

[[nodiscard]] std::size_t stepsOf(const types::SimulationResult& r) {
    return r.mission_results.empty() ? 0 : r.mission_results.front().steps;
}

[[nodiscard]] std::string resolutionStatusString(types::ResolutionRequestStatus s) {
    switch (s) {
        case types::ResolutionRequestStatus::Accepted:        return "ACCEPTED";
        case types::ResolutionRequestStatus::Ignored:         return "IGNORED";
        case types::ResolutionRequestStatus::IgnoredTooSmall: return "IGNORED TOO SMALL";
    }
    return "IGNORED";
}

[[nodiscard]] bool isErrorRun(const types::SimulationResult& r) {
    return statusString(r) == "error" || r.mission_score < 0.0;
}

// A stable signature of a plugin's whole result set (score+steps per run), used
// to group mission controls that reached the "same result".
[[nodiscard]] std::string resultsSignature(const ReportWriter::ModeRunResult& e) {
    std::string sig;
    for (const auto& r : e.results) {
        sig += std::to_string(r.mission_score) + ":" + std::to_string(stepsOf(r)) + ";";
    }
    return sig;
}

// Aggregate a plugin's runs into (representative score, total steps) for ranking.
[[nodiscard]] std::pair<double, std::size_t> aggregate(const ReportWriter::ModeRunResult& e) {
    double sum = 0.0;
    std::size_t scored = 0;
    std::size_t steps = 0;
    for (const auto& r : e.results) {
        steps += stepsOf(r);
        if (!isErrorRun(r)) {
            sum += r.mission_score;
            ++scored;
        }
    }
    const double score = (scored > 0) ? sum / static_cast<double>(scored) : -1.0;
    return {score, steps};
}

// The first error (code: message) across a plugin's runs, if any. Surfaced in
// the report so a batch of failing runs explains itself.
[[nodiscard, maybe_unused]] std::string firstError(const ReportWriter::ModeRunResult& e) {
    for (const auto& r : e.results) {
        for (const auto& mr : r.mission_results) {
            for (const auto& err : mr.errors) {
                return err.code + ": " + err.message;
            }
        }
    }
    return "";
}

// How many of a plugin's runs failed (scored -1).
[[nodiscard, maybe_unused]] std::size_t errorCount(const ReportWriter::ModeRunResult& e) {
    std::size_t n = 0;
    for (const auto& r : e.results) {
        if (isErrorRun(r)) {
            ++n;
        }
    }
    return n;
}

} // namespace

void ReportWriter::write(const types::SimulationManagerReport& report,
                         const CompositionManifest& manifest,
                         const std::filesystem::path& output_path) {
    std::filesystem::create_directories(output_path);

    std::size_t scored = 0;
    std::size_t errors = 0;
    double sum = 0.0;
    double min_score = std::numeric_limits<double>::max();
    double max_score = std::numeric_limits<double>::lowest();
    for (const auto& r : report.runs) {
        if (isErrorRun(r)) {
            ++errors;
        } else {
            ++scored;
            sum += r.mission_score;
            min_score = std::min(min_score, r.mission_score);
            max_score = std::max(max_score, r.mission_score);
        }
    }

    YAML::Node root;
    YAML::Node sr = root["score_report"];
    sr["composition_file"] = manifest.composition_file.filename().string();
    sr["generated_at_utc"] = nowUtcIso8601();
    sr["metric"] = report.metric;
    sr["score_range"]["min"] = std::get<0>(report.score_range);
    sr["score_range"]["max"] = std::get<1>(report.score_range);
    sr["error_score"] = report.error_score;
    sr["summary"]["total_runs"] = report.runs.size();
    sr["summary"]["scored_runs"] = scored;
    sr["summary"]["error_runs"] = errors;
    sr["summary"]["average_score"] = (scored > 0) ? (sum / static_cast<double>(scored)) : 0.0;
    sr["summary"]["min_score"] = (scored > 0) ? min_score : 0.0;
    sr["summary"]["max_score"] = (scored > 0) ? max_score : 0.0;

    YAML::Node simulations;
    std::size_t ri = 0;
    for (const auto& sim : manifest.simulations) {
        YAML::Node sim_node;
        sim_node["simulation_config"] = sim.simulation_config.string();

        YAML::Node missions_node;
        for (const auto& mission_path : sim.mission_configs) {
            YAML::Node mission_node;
            mission_node["mission_config"] = mission_path.string();

            YAML::Node runs_node;
            bool mission_meta_set = false;
            for (const auto& drone_path : manifest.drone_configs) {
                for (const auto& lidar_path : manifest.lidar_configs) {
                    if (ri >= report.runs.size()) {
                        break;
                    }
                    const types::SimulationResult& res = report.runs[ri++];

                    if (!mission_meta_set) {
                        mission_node["resolution_cm"] =
                            res.output_map_config.resolution.numerical_value_in(cm);
                        mission_node["resolution_request_status"] =
                            resolutionStatusString(res.resolution_request_status);
                        mission_meta_set = true;
                    }

                    YAML::Node run_node;
                    run_node["drone_config"] = drone_path.string();
                    run_node["lidar_config"] = lidar_path.string();
                    run_node["status"] = statusString(res);
                    run_node["steps"] = stepsOf(res);
                    run_node["score"] = res.mission_score;

                    if (!res.mission_results.empty()) {
                        const std::vector<common::types::ErrorRef>& errs =
                            res.mission_results.front().errors;
                        if (errs.size() == 1) {
                            run_node["error_ref"]["code"] = errs.front().code;
                        } else if (errs.size() > 1) {
                            YAML::Node error_refs;
                            for (const common::types::ErrorRef& err : errs) {
                                YAML::Node error_ref;
                                error_ref["code"] = err.code;
                                error_refs.push_back(error_ref);
                            }
                            run_node["error_refs"] = error_refs;
                        }
                    }
                    runs_node.push_back(run_node);
                }
            }
            mission_node["runs"] = runs_node;
            missions_node.push_back(mission_node);
        }
        sim_node["missions"] = missions_node;
        simulations.push_back(sim_node);
    }
    sr["simulations"] = simulations;

    std::ofstream out(output_path / "simulation_output.yaml", std::ios::trunc);
    out << root;
}

void ReportWriter::writeComparativeReport(const std::vector<ModeRunResult>& entries,
                                          const std::vector<std::string>& failed_plugins,
                                          const std::filesystem::path& composition_file,
                                          const std::filesystem::path& mission_control_folder,
                                          const std::filesystem::path& output_path) {
    std::filesystem::create_directories(output_path);

    // Group mission controls by identical result signatures.
    std::vector<std::pair<std::string, std::vector<const ModeRunResult*>>> groups;
    for (const auto& e : entries) {
        const std::string sig = resultsSignature(e);
        auto it = std::find_if(groups.begin(), groups.end(),
                               [&](const auto& g) { return g.first == sig; });
        if (it == groups.end()) {
            groups.push_back({sig, {&e}});
        } else {
            it->second.push_back(&e);
        }
    }
    // Sort groups by number of agreeing managers, descending.
    std::sort(groups.begin(), groups.end(),
              [](const auto& a, const auto& b) { return a.second.size() > b.second.size(); });

    YAML::Node root;
    root["comparative_report"]["composition_file"] = composition_file.filename().string();
    root["comparative_report"]["mission_control_folder"] = mission_control_folder.string();
    root["comparative_report"]["generated_at_utc"] = nowUtcIso8601();

    YAML::Node results_summary;
    for (const auto& [sig, members] : groups) {
        (void)sig;
        YAML::Node group;
        YAML::Node names;
        for (const ModeRunResult* m : members) {
            names.push_back(m->plugin_name);
        }
        group["same_results"] = names;
        const auto [score, steps] = aggregate(*members.front());
        group["total_score"] = static_cast<int>(score);
        group["total_steps"] = steps;
        results_summary.push_back(group);
    }
    root["comparative_report"]["results_summary"] = results_summary;

    // Add errors array for plugins that could not be loaded/run
    if (!failed_plugins.empty()) {
        YAML::Node errors;
        for (const auto& name : failed_plugins) {
            errors.push_back(name);
        }
        root["comparative_report"]["errors"] = errors;
    }

    std::ofstream out(output_path / "comparative_results.yaml", std::ios::trunc);
    out << root;
}

void ReportWriter::writeCompetitiveReport(const std::vector<ModeRunResult>& entries,
                                          const std::vector<std::string>& failed_plugins,
                                          const std::filesystem::path& composition_file,
                                          const std::filesystem::path& mission_control,
                                          const std::filesystem::path& output_path) {
    std::filesystem::create_directories(output_path);

    struct Ranked {
        std::string name;
        double score;
        std::size_t steps;
    };
    std::vector<Ranked> ranked;
    ranked.reserve(entries.size());
    for (const auto& e : entries) {
        const auto [score, steps] = aggregate(e);
        ranked.push_back({e.plugin_name, score, steps});
    }
    // Score descending, then steps ascending.
    std::sort(ranked.begin(), ranked.end(), [](const Ranked& a, const Ranked& b) {
        if (a.score != b.score) return a.score > b.score;
        return a.steps < b.steps;
    });

    YAML::Node root;
    root["competitive_report"]["composition_file"] = composition_file.filename().string();
    root["competitive_report"]["mission_control"] = mission_control.filename().string();
    root["competitive_report"]["generated_at_utc"] = nowUtcIso8601();

    YAML::Node results_summary;
    for (const Ranked& r : ranked) {
        YAML::Node node;
        node["algorithm"] = r.name;
        node["total_score"] = static_cast<int>(r.score);
        node["total_steps"] = r.steps;
        results_summary.push_back(node);
    }
    root["competitive_report"]["results_summary"] = results_summary;

    // Add errors array for plugins that could not be loaded/run
    if (!failed_plugins.empty()) {
        YAML::Node errors;
        for (const auto& name : failed_plugins) {
            errors.push_back(name);
        }
        root["competitive_report"]["errors"] = errors;
    }

    std::ofstream out(output_path / "competition_results.yaml", std::ios::trunc);
    out << root;
}

} // namespace simulator