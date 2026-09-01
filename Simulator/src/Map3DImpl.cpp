#include <Simulator/Map3DImpl.h>

#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <utility>

namespace simulator {

using namespace common;

namespace {

constexpr std::uint8_t kByteEmpty = 0;
constexpr std::uint8_t kByteOccupied = 1;
constexpr std::uint8_t kByteUnmapped = 0xFF;             // int8 -1 -> Unmapped
constexpr std::uint8_t kBytePotentiallyOccupied = 0xFD; // int8 -3 -> PotentiallyOccupied

struct VoxelIndex {
    std::size_t xi;
    std::size_t yi;
    std::size_t zi;
};

[[nodiscard]] std::optional<VoxelIndex>
worldToIndex(const Position3D& pos, const types::MapConfig& config, const NpyArray::shape_t& shape) {
    const double resolution_cm = config.resolution.numerical_value_in(cm);
    if (resolution_cm <= 0.0) {
        return std::nullopt;
    }

    const double px = pos.x.numerical_value_in(cm) - config.offset.x.numerical_value_in(cm);
    const double py = pos.y.numerical_value_in(cm) - config.offset.y.numerical_value_in(cm);
    const double pz = pos.z.numerical_value_in(cm) - config.offset.z.numerical_value_in(cm);

    const long long xi = std::llround(px / resolution_cm);
    const long long yi = std::llround(py / resolution_cm);
    const long long zi = std::llround(pz / resolution_cm);

    if (xi < 0 || yi < 0 || zi < 0 ||
        static_cast<std::size_t>(xi) >= shape[0] ||
        static_cast<std::size_t>(yi) >= shape[1] ||
        static_cast<std::size_t>(zi) >= shape[2]) {
        return std::nullopt;
    }
    return VoxelIndex{static_cast<std::size_t>(xi),
                      static_cast<std::size_t>(yi),
                      static_cast<std::size_t>(zi)};
}

[[nodiscard]] std::size_t flatOffset(const VoxelIndex& v, const NpyArray::shape_t& shape) {
    return (v.xi * shape[1] + v.yi) * shape[2] + v.zi;
}

} // namespace

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr)
    : Map3DImpl(std::move(map_ptr), types::MapConfig{}) {}

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config)
    : map_(std::move(map_ptr)),
      config_(map_config) {
    if (!map_) {
        throw std::invalid_argument("Map3DImpl requires a valid map pointer.");
    }
    if (map_->Shape().size() != 3) {
        throw std::invalid_argument("Map3DImpl requires a 3D (X, Y, Z) npy array.");
    }
}

types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    const auto& shape = map_->Shape();
    const auto index = worldToIndex(pos, config_, shape);
    if (!index) {
        return types::VoxelOccupancy::OutOfBounds;
    }

    const std::uint8_t* data = map_->Data<std::uint8_t>();
    const std::uint8_t b = data[flatOffset(*index, shape)];
    if (b == kByteUnmapped) {
        return types::VoxelOccupancy::Unmapped;
    }
    if (b == kBytePotentiallyOccupied) {
        return types::VoxelOccupancy::PotentiallyOccupied;
    }
    return (b == kByteEmpty) ? types::VoxelOccupancy::Empty
                             : types::VoxelOccupancy::Occupied;
}

types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

bool Map3DImpl::isInBounds(const Position3D& pos) const {
    return worldToIndex(pos, config_, map_->Shape()).has_value();
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy value) {
    const auto& shape = map_->Shape();
    const auto index = worldToIndex(pos, config_, shape);
    if (!index) {
        return;
    }

    std::uint8_t byte = kByteEmpty;
    switch (value) {
        case types::VoxelOccupancy::Occupied:            byte = kByteOccupied;            break;
        case types::VoxelOccupancy::Empty:               byte = kByteEmpty;               break;
        case types::VoxelOccupancy::Unmapped:            byte = kByteUnmapped;            break;
        case types::VoxelOccupancy::PotentiallyOccupied: byte = kBytePotentiallyOccupied; break;
        case types::VoxelOccupancy::OutOfBounds:         return;
    }

    std::uint8_t* data = map_->Data<std::uint8_t>();
    data[flatOffset(*index, shape)] = byte;
}

void Map3DImpl::save(const std::filesystem::path& output_path) const {
    const LPCSTR err = map_->SaveNPY(output_path.string());
    if (err != nullptr) {
        throw std::runtime_error(std::string{"Failed to save output map: "} + err);
    }
}

} // namespace simulator
