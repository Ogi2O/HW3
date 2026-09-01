#pragma once

#include <Simulator/CompositionManifest.h>
#include <Simulator/SimulationTypes.h>

#include <filesystem>
#include <string>
#include <vector>

namespace simulator {

// Writes simulation results to disk.
class ReportWriter {
public:
    // Assignment-2-style hierarchical score report for one composition:
    // output_path/simulation_output.yaml. The flat report.runs are zipped with
    // `manifest` (same cartesian order) to recover per-run config paths.
    static void write(const types::SimulationManagerReport& report,
                      const CompositionManifest& manifest,
                      const std::filesystem::path& output_path);

    // One plugin (a mission control in comparative mode, an algorithm in
    // competitive mode) together with all of its run results.
    struct ModeRunResult {
        std::string plugin_name;
        std::vector<types::SimulationResult> results;
    };

    // Comparative report: groups mission controls that produced identical
    // results under `same_results`, groups sorted by how many agree (desc).
    // Written to output_path/comparative_results.yaml.
    // Also writes per-plugin result files: output_path/simulation_output_<plugin>.yaml
    static void writeComparativeReport(const std::vector<ModeRunResult>& entries,
                                       const std::vector<std::string>& failed_plugins,
                                       const std::filesystem::path& composition_file,
                                       const std::filesystem::path& mission_control_folder,
                                       const std::filesystem::path& output_path);

    // Competitive report: algorithms ranked by score (desc) then steps (asc).
    // Written to output_path/competition_results.yaml.
    // Also writes per-plugin result files: output_path/simulation_output_<plugin>.yaml
    static void writeCompetitiveReport(const std::vector<ModeRunResult>& entries,
                                       const std::vector<std::string>& failed_plugins,
                                       const std::filesystem::path& composition_file,
                                       const std::filesystem::path& mission_control,
                                       const std::filesystem::path& output_path);

private:
    // Write a per-plugin result YAML file (assignment-2 style format).
    static void writePluginReport(const ModeRunResult& entry,
                                  const std::filesystem::path& output_path);
};

} // namespace simulator
