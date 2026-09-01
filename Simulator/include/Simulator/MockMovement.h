#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IMap3D.h>
#include <Simulator/MockGPS.h>

namespace simulator {

// Moves the drone by updating the injected GPS, but first checks the hidden
// ground-truth map for collisions along the requested path and refuses any move
// that would put the drone's spherical body into an occupied voxel (or outside
// the building). The retry-at-half-distance policy lives in DroneControlImpl.
class MockMovement final : public common::IDroneMovement {
public:
    MockMovement(MockGPS& gps, const common::IMap3D& world, common::PhysicalLength drone_radius);

    common::types::MovementResult rotate(common::types::RotationDirection direction,
                                         common::HorizontalAngle angle) override;
    common::types::MovementResult advance(common::PhysicalLength distance) override;
    common::types::MovementResult elevate(common::PhysicalLength distance) override;

private:
    [[nodiscard]] bool collidesAt(const common::Position3D& center) const;
    [[nodiscard]] bool pathBlocked(const common::Position3D& from, const common::Position3D& to) const;

    MockGPS& gps_;
    const common::IMap3D& world_;
    common::PhysicalLength radius_;
};

} // namespace simulator
