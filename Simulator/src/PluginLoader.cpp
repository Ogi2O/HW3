#include <Simulator/PluginLoader.h>

#include <Simulator/Registrar.h>

#include <dlfcn.h>

namespace simulator {

void* PluginLoader::open(const std::filesystem::path& shared_object) {
    if (!std::filesystem::exists(shared_object)) {
        last_error_ = "shared object not found: " + shared_object.string();
        return nullptr;
    }
    // RTLD_NOW: resolve every symbol at load time (so a missing symbol - e.g. an
    // unresolved registration ctor - fails here rather than mid-run).
    // RTLD_LOCAL: keep each plugin's own symbols out of the global namespace so
    // several plugins can define identically named internals without clashing.
    void* handle = dlopen(shared_object.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (handle == nullptr) {
        const char* err = dlerror();
        last_error_ = err != nullptr ? err : "unknown dlopen error";
        return nullptr;
    }
    handles_.push_back(handle);
    return handle;
}

std::optional<common::MappingAlgorithmFactory>
PluginLoader::loadAlgorithm(const std::filesystem::path& shared_object) {
    Registrar& registrar = Registrar::instance();
    const std::size_t before = registrar.algorithmCount();

    if (open(shared_object) == nullptr) {
        return std::nullopt;
    }

    const std::size_t after = registrar.algorithmCount();
    if (after == before) {
        last_error_ = "plugin registered no mapping algorithm: " + shared_object.string();
        return std::nullopt;
    }
    // The plugin registers exactly one algorithm; take the one it just added.
    return registrar.algorithmAt(after - 1);
}

std::optional<common::MissionControlFactory>
PluginLoader::loadMissionControl(const std::filesystem::path& shared_object) {
    Registrar& registrar = Registrar::instance();
    const std::size_t before = registrar.missionControlCount();

    if (open(shared_object) == nullptr) {
        return std::nullopt;
    }

    const std::size_t after = registrar.missionControlCount();
    if (after == before) {
        last_error_ = "plugin registered no mission control: " + shared_object.string();
        return std::nullopt;
    }
    return registrar.missionControlAt(after - 1);
}

PluginLoader::~PluginLoader() {
    // Destroy the registrar's factory objects (whose code lives in the plugins)
    // BEFORE unmapping the plugins, then close every handle.
    Registrar::instance().clear();
    for (void* handle : handles_) {
        dlclose(handle);
    }
}

} // namespace simulator
