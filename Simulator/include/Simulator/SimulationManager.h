#pragma once

#include <Simulator/ISimulation.h>
#include <Simulator/ISimulationRunFactory.h>

#include <memory>

namespace simulator {

// Expands the per-simulation cartesian product (missions x drones x lidars),
// runs each via the injected factory, scores a failed run as -1 and continues.
// (Threading and the comparative/competitive modes are layered on top of this
// core in later sub-steps.)
class SimulationManager final : public ISimulation {
public:
    explicit SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory);

    [[nodiscard]] types::SimulationManagerReport run(
        const types::SimulationCompositionData& composition,
        const std::filesystem::path& output_path) override;

private:
    std::unique_ptr<ISimulationRunFactory> run_factory_;
};

} // namespace simulator
