#include <Simulator/MapsComparison.h>

#include <algorithm>
#include <cmath>

namespace simulator {

using namespace common;

namespace {

// Per-class Jaccard (IoU) accumulator.
struct ClassIoU {
    long long intersection = 0;
    long long uni = 0;
    void add(bool origin_is, bool target_is) {
        if (origin_is || target_is) {
            ++uni;
            if (origin_is && target_is) {
                ++intersection;
            }
        }
    }
    [[nodiscard]] bool present() const { return uni > 0; }
    [[nodiscard]] double iou() const {
        return uni > 0 ? static_cast<double>(intersection) / static_cast<double>(uni) : 0.0;
    }
};

template <typename Length>
[[nodiscard]] long long stepsAlong(Length min_len, Length max_len, PhysicalLength resolution) {
    const double res_cm = resolution.numerical_value_in(cm);
    const double min_cm = min_len.numerical_value_in(cm);
    const double max_cm = max_len.numerical_value_in(cm);
    if (res_cm <= 0.0 || max_cm <= min_cm) {
        return 0;
    }
    return std::llround((max_cm - min_cm) / res_cm);
}

} // namespace

std::vector<double> MapsComparison::compare(const IMap3D& origin,
                                            const std::vector<IMap3D*> targets) {
    std::vector<double> scores;
    scores.reserve(targets.size());

    const types::MapConfig cfg = origin.getMapConfig();
    double res = cfg.resolution.numerical_value_in(cm);
    if (res <= 0.0) {
        res = 1.0;
    }

    const double min_x = cfg.boundaries.min_x.numerical_value_in(cm);
    const double min_y = cfg.boundaries.min_y.numerical_value_in(cm);
    const double min_h = cfg.boundaries.min_height.numerical_value_in(cm);
    const long long nx = stepsAlong(cfg.boundaries.min_x, cfg.boundaries.max_x, cfg.resolution);
    const long long ny = stepsAlong(cfg.boundaries.min_y, cfg.boundaries.max_y, cfg.resolution);
    const long long nh =
        stepsAlong(cfg.boundaries.min_height, cfg.boundaries.max_height, cfg.resolution);

    for (const IMap3D* target : targets) {
        if (target == nullptr) {
            scores.push_back(-1.0);
            continue;
        }

        ClassIoU occupied;
        ClassIoU empty;

        for (long long ix = 0; ix <= nx; ++ix) {
            for (long long iy = 0; iy <= ny; ++iy) {
                for (long long ih = 0; ih <= nh; ++ih) {
                    const Position3D p{
                        (min_x + static_cast<double>(ix) * res) * x_extent[cm],
                        (min_y + static_cast<double>(iy) * res) * y_extent[cm],
                        (min_h + static_cast<double>(ih) * res) * z_extent[cm],
                    };

                    const types::VoxelOccupancy ov = origin.atVoxel(p);
                    if (ov != types::VoxelOccupancy::Occupied &&
                        ov != types::VoxelOccupancy::Empty) {
                        continue;
                    }
                    const types::VoxelOccupancy tv = target->atVoxel(p);

                    occupied.add(ov == types::VoxelOccupancy::Occupied,
                                 tv == types::VoxelOccupancy::Occupied);
                    empty.add(ov == types::VoxelOccupancy::Empty,
                              tv == types::VoxelOccupancy::Empty);
                }
            }
        }

        double sum = 0.0;
        int parts = 0;
        if (occupied.present()) { sum += occupied.iou(); ++parts; }
        if (empty.present())    { sum += empty.iou();    ++parts; }

        scores.push_back(parts == 0 ? 100.0 : 100.0 * (sum / parts));
    }

    return scores;
}

} // namespace simulator
