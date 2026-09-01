#include <Simulator/MockMovement.h>

#include <mp-units/systems/si/math.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace simulator {

using namespace common;

namespace {
constexpr double kCollisionStepCm = 0.5;
} // namespace

MockMovement::MockMovement(MockGPS& gps, const IMap3D& world, PhysicalLength drone_radius)
    : gps_(gps), world_(world), radius_(drone_radius) {}

types::MovementResult MockMovement::rotate(types::RotationDirection direction, HorizontalAngle angle) {
    const Orientation current = gps_.heading();
    const HorizontalAngle signed_angle =
        (direction == types::RotationDirection::Left) ? angle : -angle;
    gps_.setHeading(Orientation{current.horizontal + signed_angle, current.altitude});
    return types::MovementResult{true, {}};
}

bool MockMovement::collidesAt(const Position3D& center) const {
    const double r = radius_.numerical_value_in(cm);
    const double cx = center.x.numerical_value_in(cm);
    const double cy = center.y.numerical_value_in(cm);
    const double cz = center.z.numerical_value_in(cm);

    const Position3D samples[] = {
        Position3D{cx * x_extent[cm], cy * y_extent[cm], cz * z_extent[cm]},
        Position3D{(cx + r) * x_extent[cm], cy * y_extent[cm], cz * z_extent[cm]},
        Position3D{(cx - r) * x_extent[cm], cy * y_extent[cm], cz * z_extent[cm]},
        Position3D{cx * x_extent[cm], (cy + r) * y_extent[cm], cz * z_extent[cm]},
        Position3D{cx * x_extent[cm], (cy - r) * y_extent[cm], cz * z_extent[cm]},
        Position3D{cx * x_extent[cm], cy * y_extent[cm], (cz + r) * z_extent[cm]},
        Position3D{cx * x_extent[cm], cy * y_extent[cm], (cz - r) * z_extent[cm]},
    };
    for (const Position3D& s : samples) {
        const types::VoxelOccupancy occ = world_.atVoxel(s);
        if (occ == types::VoxelOccupancy::Occupied ||
            occ == types::VoxelOccupancy::OutOfBounds) {
            return true;
        }
    }
    return false;
}

bool MockMovement::pathBlocked(const Position3D& from, const Position3D& to) const {
    const double fx = from.x.numerical_value_in(cm);
    const double fy = from.y.numerical_value_in(cm);
    const double fz = from.z.numerical_value_in(cm);
    const double tx = to.x.numerical_value_in(cm);
    const double ty = to.y.numerical_value_in(cm);
    const double tz = to.z.numerical_value_in(cm);

    const double dist =
        std::sqrt((tx - fx) * (tx - fx) + (ty - fy) * (ty - fy) + (tz - fz) * (tz - fz));
    const int steps = std::max(1, static_cast<int>(std::ceil(dist / kCollisionStepCm)));
    for (int i = 1; i <= steps; ++i) {
        const double f = static_cast<double>(i) / static_cast<double>(steps);
        const Position3D p{
            (fx + (tx - fx) * f) * x_extent[cm],
            (fy + (ty - fy) * f) * y_extent[cm],
            (fz + (tz - fz) * f) * z_extent[cm],
        };
        if (collidesAt(p)) {
            return true;
        }
    }
    return false;
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const Position3D from = gps_.position();
    const Orientation h = gps_.heading();

    const auto cos_alt = si::cos(h.altitude);
    const auto ux = cos_alt * si::cos(h.horizontal);
    const auto uy = cos_alt * si::sin(h.horizontal);
    const auto uz = si::sin(h.altitude);

    const double d = distance.numerical_value_in(cm);
    const Position3D to{
        from.x + ux.numerical_value_in(mp::one) * d * x_extent[cm],
        from.y + uy.numerical_value_in(mp::one) * d * y_extent[cm],
        from.z + uz.numerical_value_in(mp::one) * d * z_extent[cm],
    };

    if (pathBlocked(from, to)) {
        throw std::runtime_error("DRONE_HITS_OBSTACLE");
    }
    gps_.setPosition(to);
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    const Position3D from = gps_.position();
    const double d = distance.numerical_value_in(cm);
    const Position3D to{from.x, from.y, from.z + d * z_extent[cm]};

    if (pathBlocked(from, to)) {
        throw std::runtime_error("DRONE_HITS_OBSTACLE");
    }
    gps_.setPosition(to);
    return types::MovementResult{true, {}};
}

} // namespace simulator
