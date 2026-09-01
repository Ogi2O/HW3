#include <Simulator/Registrar.h>

#include <utility>

namespace simulator {

Registrar& Registrar::instance() {
    static Registrar registrar;
    return registrar;
}

void Registrar::addAlgorithm(common::MappingAlgorithmFactory factory) {
    algorithms_.push_back(std::move(factory));
}

void Registrar::addMissionControl(common::MissionControlFactory factory) {
    mission_controls_.push_back(std::move(factory));
}

void Registrar::clear() {
    algorithms_.clear();
    mission_controls_.clear();
}

} // namespace simulator
