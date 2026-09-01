#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <cstddef>
#include <vector>

namespace simulator {

// Process-wide collection point for factories contributed by plugin .so files.
// When a plugin is dlopen'd, its global REGISTER_* object's constructor runs and
// calls the corresponding Registration ctor (defined in Registration.cpp), which
// forwards the factory here. The PluginLoader then reads back what was just
// registered.
//
// The stored factories are std::functions whose call/destroy operations live
// inside the plugin .so, so they MUST be destroyed (clear()) before that .so is
// dlclose'd - the PluginLoader does exactly that in its destructor.
class Registrar {
public:
    static Registrar& instance();

    void addAlgorithm(common::MappingAlgorithmFactory factory);
    void addMissionControl(common::MissionControlFactory factory);

    [[nodiscard]] std::size_t algorithmCount() const { return algorithms_.size(); }
    [[nodiscard]] std::size_t missionControlCount() const { return mission_controls_.size(); }
    [[nodiscard]] const common::MappingAlgorithmFactory& algorithmAt(std::size_t i) const {
        return algorithms_.at(i);
    }
    [[nodiscard]] const common::MissionControlFactory& missionControlAt(std::size_t i) const {
        return mission_controls_.at(i);
    }

    void clear();

private:
    Registrar() = default;

    std::vector<common::MappingAlgorithmFactory> algorithms_;
    std::vector<common::MissionControlFactory> mission_controls_;
};

} // namespace simulator
