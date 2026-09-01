#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace simulator {

// Loads plugin .so files with dlopen and hands back the factory each one
// registered (via the Registrar). Owns every open handle and closes them in its
// destructor.
//
// LIFETIME CONTRACT: a factory (and any instance it creates) contains code that
// lives inside its .so. The PluginLoader must therefore OUTLIVE every factory
// copy and every created algorithm / mission-control / run. In practice: create
// the PluginLoader first in main so it is destroyed last. Its destructor clears
// the Registrar's factories and then dlclose's all handles, in that order.
class PluginLoader {
public:
    PluginLoader() = default;
    ~PluginLoader();

    PluginLoader(const PluginLoader&) = delete;
    PluginLoader& operator=(const PluginLoader&) = delete;

    // Load one plugin and return the factory it registered, or nullopt on
    // failure (bad path, dlopen error, or nothing registered). See lastError().
    [[nodiscard]] std::optional<common::MappingAlgorithmFactory>
    loadAlgorithm(const std::filesystem::path& shared_object);

    [[nodiscard]] std::optional<common::MissionControlFactory>
    loadMissionControl(const std::filesystem::path& shared_object);

    [[nodiscard]] const std::string& lastError() const { return last_error_; }

private:
    [[nodiscard]] void* open(const std::filesystem::path& shared_object);

    std::vector<void*> handles_;
    std::string last_error_;
};

} // namespace simulator
