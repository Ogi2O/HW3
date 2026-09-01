// Definitions of the registration-struct constructors declared in the common
// headers. Per the assignment these live ONLY in the Simulator project: each
// plugin .so leaves them undefined and they are resolved (against the exported
// symbols of the simulator executable) when the .so is dlopen'd. They forward
// the plugin's factory into the process-wide Registrar.

#include <Common/MappingAlgorithmRegistration.h>
#include <Common/MissionControlRegistration.h>

#include <Simulator/Registrar.h>

#include <utility>

namespace common {

MappingAlgorithmRegistration::MappingAlgorithmRegistration(MappingAlgorithmFactory factory) {
    simulator::Registrar::instance().addAlgorithm(std::move(factory));
}

MissionControlRegistration::MissionControlRegistration(MissionControlFactory factory) {
    simulator::Registrar::instance().addMissionControl(std::move(factory));
}

} // namespace common
