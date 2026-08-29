#include "scout_planner/core/hybrid_map_query.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace scout_planner::core {
namespace {

constexpr double kBoundaryTolerance = 1.0e-12;

struct Grid3d {
  Point3dEnu origin;
  Point3dEnu resolution;
  std::array<std::uint32_t, 3U> count;

  [[nodiscard]] Point3dEnu minimum() const noexcept {
    return {origin.x_m - 0.5 * resolution.x_m,
            origin.y_m - 0.5 * resolution.y_m,
            origin.z_m - 0.5 * resolution.z_m};
  }

  [[nodiscard]] Point3dEnu maximum() const noexcept {
    const auto low = minimum();
    return {low.x_m + resolution.x_m * static_cast<double>(count[0]),
            low.y_m + resolution.y_m * static_cast<double>(count[1]),
            low.z_m + resolution.z_m * static_cast<double>(count[2])};
  }
};

struct SemanticRegion {
  MapSemanticRestriction restriction;
  Aabb3dEnu bounds;
};

[[nodiscard]] std::optional<std::array<double, 2U>> segment_aabb_interval(
    const Point3dEnu start, const Point3dEnu end,
    const Aabb3dEnu& bounds) {
  double entry = 0.0;
  double exit = 1.0;
  const std::array starts{start.x_m, start.y_m, start.z_m};
  const std::array ends{end.x_m, end.y_m, end.z_m};
  const std::array lower{bounds.minimum_m.x_m, bounds.minimum_m.y_m,
                         bounds.minimum_m.z_m};
  const std::array upper{bounds.maximum_m.x_m, bounds.maximum_m.y_m,
                         bounds.maximum_m.z_m};
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    const double delta = ends[axis] - starts[axis];
    if (delta == 0.0) {
      if (starts[axis] < lower[axis] || starts[axis] > upper[axis]) {
        return std::nullopt;
      }
      continue;
    }
    double first = (lower[axis] - starts[axis]) / delta;
    double second = (upper[axis] - starts[axis]) / delta;
    if (first > second) {
      std::swap(first, second);
    }
    entry = std::max(entry, first);
    exit = std::min(exit, second);
    if (entry > exit) {
      return std::nullopt;
    }
  }
  return std::array<double, 2U>{entry, exit};
}

[[nodiscard]] Aabb3dEnu expanded(const Aabb3dEnu& bounds,
                                 const double margin) noexcept {
  return {{bounds.minimum_m.x_m - margin, bounds.minimum_m.y_m - margin,
           bounds.minimum_m.z_m - margin},
          {bounds.maximum_m.x_m + margin, bounds.maximum_m.y_m + margin,
           bounds.maximum_m.z_m + margin}};
}

[[nodiscard]] bool contains(const Aabb3dEnu& bounds,
                            const Point3dEnu point) noexcept {
  return point.x_m >= bounds.minimum_m.x_m &&
         point.y_m >= bounds.minimum_m.y_m &&
         point.z_m >= bounds.minimum_m.z_m &&
         point.x_m <= bounds.maximum_m.x_m &&
         point.y_m <= bounds.maximum_m.y_m &&
         point.z_m <= bounds.maximum_m.z_m;
}

[[nodiscard]] Hash256 to_hash(const std::vector<std::uint8_t>& bytes) {
  Hash256 hash{};
  if (bytes.size() == hash.size()) {
    std::copy(bytes.begin(), bytes.end(), hash.begin());
  }
  return hash;
}

[[nodiscard]] const CoreField& field(const CoreMessage& message,
                                     const std::uint32_t number) {
  const auto found = std::find_if(
      message.fields.begin(), message.fields.end(),
      [number](const CoreField& candidate) { return candidate.number == number; });
  if (found == message.fields.end()) {
    throw std::invalid_argument{"validated map field is missing"};
  }
  return *found;
}

template <typename T>
[[nodiscard]] const T& scalar(const CoreMessage& message,
                              const std::uint32_t number) {
  const auto& values = field(message, number).values;
  if (values.size() != 1U) {
    throw std::invalid_argument{"validated singular map field is malformed"};
  }
  return std::get<T>(values.front());
}

[[nodiscard]] const CoreMessage& child(const CoreMessage& message,
                                       const std::uint32_t number) {
  const auto& value = scalar<CoreMessagePtr>(message, number);
  if (value == nullptr) {
    throw std::invalid_argument{"validated map child is null"};
  }
  return *value;
}

template <typename T>
[[nodiscard]] std::vector<T> repeated(const CoreMessage& message,
                                      const std::uint32_t number) {
  std::vector<T> result;
  const auto& values = field(message, number).values;
  result.reserve(values.size());
  for (const auto& value : values) {
    result.push_back(std::get<T>(value));
  }
  return result;
}

[[nodiscard]] Aabb3dEnu core_aabb(const CoreMessage& region) {
  const auto coordinates = repeated<double>(region, 1U);
  if (coordinates.size() != 6U) {
    throw std::invalid_argument{"validated map AABB is malformed"};
  }
  return {{coordinates[0], coordinates[1], coordinates[2]},
          {coordinates[3], coordinates[4], coordinates[5]}};
}

[[nodiscard]] Grid3d core_grid(const CoreMessage& grid) {
  return {{scalar<double>(grid, 1U), scalar<double>(grid, 2U),
           scalar<double>(grid, 3U)},
          {scalar<double>(grid, 4U), scalar<double>(grid, 5U),
           scalar<double>(grid, 6U)},
          {scalar<std::uint32_t>(grid, 7U),
           scalar<std::uint32_t>(grid, 8U),
           scalar<std::uint32_t>(grid, 9U)}};
}

[[nodiscard]] Aabb3dEnu voxel_bounds(const Grid3d& grid,
                                     const VoxelIndex3d voxel) noexcept {
  const auto low = grid.minimum();
  return {{low.x_m + grid.resolution.x_m * voxel.x,
           low.y_m + grid.resolution.y_m * voxel.y,
           low.z_m + grid.resolution.z_m * voxel.z},
          {low.x_m + grid.resolution.x_m * (voxel.x + 1U),
           low.y_m + grid.resolution.y_m * (voxel.y + 1U),
           low.z_m + grid.resolution.z_m * (voxel.z + 1U)}};
}

[[nodiscard]] bool finite_point(const Point3dEnu point) noexcept {
  return std::isfinite(point.x_m) && std::isfinite(point.y_m) &&
         std::isfinite(point.z_m);
}

[[nodiscard]] bool valid_margins(const SafetyMargins& margins) noexcept {
  const std::array values{margins.body_m, margins.localization_m,
                          margins.tracking_m, margins.map_m,
                          margins.discretization_m};
  return std::all_of(values.begin(), values.end(), [](const double value) {
    return std::isfinite(value) && value >= 0.0;
  }) && std::isfinite(margins.total_m());
}

[[nodiscard]] std::size_t linear_index(const Grid3d& grid,
                                       const VoxelIndex3d index) noexcept {
  return static_cast<std::size_t>(index.x) +
         static_cast<std::size_t>(grid.count[0]) *
             (static_cast<std::size_t>(index.y) +
              static_cast<std::size_t>(grid.count[1]) * index.z);
}

[[nodiscard]] std::optional<VoxelIndex3d> voxel_at(const Grid3d& grid,
                                                   const Point3dEnu point) {
  const auto low = grid.minimum();
  const auto high = grid.maximum();
  if (point.x_m < low.x_m || point.y_m < low.y_m || point.z_m < low.z_m ||
      point.x_m >= high.x_m || point.y_m >= high.y_m ||
      point.z_m >= high.z_m) {
    return std::nullopt;
  }
  return VoxelIndex3d{
      static_cast<std::uint32_t>(
          std::floor((point.x_m - low.x_m) / grid.resolution.x_m)),
      static_cast<std::uint32_t>(
          std::floor((point.y_m - low.y_m) / grid.resolution.y_m)),
      static_cast<std::uint32_t>(
          std::floor((point.z_m - low.z_m) / grid.resolution.z_m)),
  };
}

[[nodiscard]] Point3dEnu voxel_center(const Grid3d& grid,
                                      const VoxelIndex3d index) noexcept {
  return {grid.origin.x_m + grid.resolution.x_m * index.x,
          grid.origin.y_m + grid.resolution.y_m * index.y,
          grid.origin.z_m + grid.resolution.z_m * index.z};
}

[[nodiscard]] VoxelIndex3d voxel_from_linear(const Grid3d& grid,
                                             const std::size_t index) noexcept {
  const auto plane =
      static_cast<std::size_t>(grid.count[0]) * grid.count[1];
  const auto z = static_cast<std::uint32_t>(index / plane);
  const auto within_plane = index % plane;
  const auto y =
      static_cast<std::uint32_t>(within_plane / grid.count[0]);
  const auto x =
      static_cast<std::uint32_t>(within_plane % grid.count[0]);
  return {x, y, z};
}

[[nodiscard]] std::vector<VoxelIndex3d> nearby_voxels(
    const Grid3d& grid, const VoxelIndex3d center, const double margin) {
  const auto radius = [margin](const double resolution,
                               const std::uint32_t count) {
    const double cells = std::ceil(margin / resolution) + 1.0;
    return cells >= static_cast<double>(count)
               ? static_cast<std::int64_t>(count)
               : static_cast<std::int64_t>(cells);
  };
  const std::array radii{
      radius(grid.resolution.x_m, grid.count[0]),
      radius(grid.resolution.y_m, grid.count[1]),
      radius(grid.resolution.z_m, grid.count[2])};
  const std::array coordinates{static_cast<std::int64_t>(center.x),
                               static_cast<std::int64_t>(center.y),
                               static_cast<std::int64_t>(center.z)};
  std::array<std::int64_t, 3U> first{};
  std::array<std::int64_t, 3U> last{};
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    first[axis] = std::max<std::int64_t>(0, coordinates[axis] - radii[axis]);
    last[axis] = std::min<std::int64_t>(
        static_cast<std::int64_t>(grid.count[axis]) - 1,
        coordinates[axis] + radii[axis]);
  }
  std::vector<VoxelIndex3d> result;
  for (auto z = first[2]; z <= last[2]; ++z) {
    for (auto y = first[1]; y <= last[1]; ++y) {
      for (auto x = first[0]; x <= last[0]; ++x) {
        result.push_back({static_cast<std::uint32_t>(x),
                          static_cast<std::uint32_t>(y),
                          static_cast<std::uint32_t>(z)});
      }
    }
  }
  return result;
}

[[nodiscard]] int state_rank(const MapCellState state) noexcept {
  switch (state) {
    case MapCellState::free:
      return 0;
    case MapCellState::occupied:
      return 1;
    case MapCellState::unknown:
      return 2;
    case MapCellState::stale:
      return 3;
    case MapCellState::conflicted:
      return 4;
  }
  return 4;
}

}  // namespace

struct HybridMapQuery::Data {
  Grid3d grid;
  MapGridInfo public_grid;
  Aabb3dEnu map_bounds;
  std::vector<float> seafloor_elevation;
  std::vector<float> seafloor_quality;
  std::vector<MapCellState> occupancy;
  std::vector<float> esdf_distance;
  std::vector<bool> allowed_water;
  std::vector<SemanticRegion> semantic_regions;
  MapSourceVersion source;
};

const MapGridInfo& HybridMapQuery::grid_info() const noexcept {
  static const MapGridInfo empty{};
  if (data_ == nullptr) {
    return empty;
  }
  return data_->public_grid;
}

double SafetyMargins::total_m() const noexcept {
  return body_m + localization_m + tracking_m + map_m + discretization_m;
}

MapQueryResult<HybridMapQuery> HybridMapQuery::create(
    const HybridMapSnapshot& snapshot,
    const MapSnapshotIdentity& expected_identity) {
  try {
    const auto& map = snapshot.document();
    const auto actual_identity =
        to_hash(scalar<BytesValue>(child(map, 4U), 1U).value);
    const auto& map_id = scalar<TextValue>(map, 2U).value;
    const auto map_version = scalar<std::uint64_t>(map, 3U);
    if (map_id != expected_identity.map_id ||
        map_version != expected_identity.map_version ||
        actual_identity != expected_identity.content_identity) {
      return MapQueryResult<HybridMapQuery>::failure(
          {MapQueryErrorCode::invalid_dependency,
           "map id, version, or content identity does not match the dependency"});
    }

    auto data = std::make_shared<Data>();
    const auto& seafloor = child(map, 8U);
    const auto& occupancy = child(map, 9U);
    const auto& esdf = child(map, 10U);
    const auto& allowed_water = child(map, 11U);
    data->grid = core_grid(child(occupancy, 1U));
    data->map_bounds = core_aabb(child(map, 5U));
    data->public_grid = {data->grid.origin, data->grid.resolution,
                         data->grid.count, data->map_bounds};
    data->seafloor_elevation = repeated<float>(seafloor, 2U);
    data->seafloor_quality = repeated<float>(seafloor, 3U);
    data->esdf_distance = repeated<float>(esdf, 3U);
    data->allowed_water = repeated<bool>(allowed_water, 2U);

    const auto& occupancy_values = field(occupancy, 2U).values;
    data->occupancy.reserve(occupancy_values.size());
    for (const auto& atom : occupancy_values) {
      switch (std::get<EnumValue>(atom).number) {
        case 1:
          data->occupancy.push_back(MapCellState::free);
          break;
        case 2:
          data->occupancy.push_back(MapCellState::occupied);
          break;
        case 3:
          data->occupancy.push_back(MapCellState::unknown);
          break;
        case 4:
          data->occupancy.push_back(MapCellState::stale);
          break;
        case 5:
          data->occupancy.push_back(MapCellState::conflicted);
          break;
        default:
          throw std::invalid_argument{
              "validated snapshot contains an unknown occupancy state"};
      }
    }

    const auto semantic = std::find_if(
        map.fields.begin(), map.fields.end(),
        [](const CoreField& candidate) { return candidate.number == 12U; });
    if (semantic != map.fields.end()) {
      for (const auto& atom : semantic->values) {
        const auto& region = *std::get<CoreMessagePtr>(atom);
        SemanticRegion value{};
        switch (scalar<EnumValue>(region, 2U).number) {
          case 1:
            value.restriction = MapSemanticRestriction::no_entry;
            break;
          case 2:
            value.restriction =
                MapSemanticRestriction::communication_shadow;
            break;
          case 3:
            value.restriction = MapSemanticRestriction::special_region;
            break;
          default:
            throw std::invalid_argument{
                "validated snapshot contains an unknown semantic class"};
        }
        value.bounds = core_aabb(child(region, 3U));
        data->semantic_regions.push_back(value);
      }
    }

    const auto& observed_at = child(map, 14U);
    MapTimeSyncStatus sync_status;
    switch (scalar<EnumValue>(observed_at, 2U).number) {
      case 1:
        sync_status = MapTimeSyncStatus::unsynchronized;
        break;
      case 2:
        sync_status = MapTimeSyncStatus::synchronized;
        break;
      case 3:
        sync_status = MapTimeSyncStatus::degraded;
        break;
      default:
        throw std::invalid_argument{
            "validated snapshot contains an unknown time sync status"};
    }
    data->source = {
        map_id,
        map_version,
        actual_identity,
        scalar<TextValue>(map, 1U).value,
        scalar<std::int64_t>(map, 13U),
        scalar<std::int64_t>(observed_at, 1U),
        sync_status,
        scalar<std::uint64_t>(observed_at, 3U),
        data->map_bounds,
        scalar<std::uint64_t>(child(map, 6U), 2U),
        scalar<std::uint64_t>(child(map, 7U), 2U),
    };
    return MapQueryResult<HybridMapQuery>::success(HybridMapQuery(data));
  } catch (const std::exception& error) {
    return MapQueryResult<HybridMapQuery>::failure(
        {MapQueryErrorCode::malformed_snapshot, error.what()});
  }
}

MapQueryResult<MapQuerySample> HybridMapQuery::query_point(
    const Point3dEnu point, const SafetyMargins& margins) const {
  if (!finite_point(point)) {
    return MapQueryResult<MapQuerySample>::failure(
        {MapQueryErrorCode::invalid_position,
         "query position must contain finite SI coordinates"});
  }
  if (!valid_margins(margins)) {
    return MapQueryResult<MapQuerySample>::failure(
        {MapQueryErrorCode::invalid_safety_margin,
         "each safety margin must be finite and non-negative"});
  }

  const auto voxel = voxel_at(data_->grid, point);
  const bool inside_declared_region =
      point.x_m >= data_->map_bounds.minimum_m.x_m &&
      point.y_m >= data_->map_bounds.minimum_m.y_m &&
      point.z_m >= data_->map_bounds.minimum_m.z_m &&
      point.x_m < data_->map_bounds.maximum_m.x_m &&
      point.y_m < data_->map_bounds.maximum_m.y_m &&
      point.z_m < data_->map_bounds.maximum_m.z_m;
  if (!voxel.has_value() || !inside_declared_region) {
    return MapQueryResult<MapQuerySample>::success(
        {MapCellState::unknown,
         0.0,
         -margins.total_m(),
         0.0,
         0.0F,
         false,
         data_->source,
         {},
         {{MapInformationGapKind::outside_snapshot, point}},
         {}});
  }

  const auto index = linear_index(data_->grid, *voxel);
  const auto floor_index = static_cast<std::size_t>(voxel->x) +
                           static_cast<std::size_t>(data_->grid.count[0]) *
                               voxel->y;
  const double distance = data_->esdf_distance.at(index);
  const double floor = data_->seafloor_elevation.at(floor_index);
  MapQuerySample result{data_->occupancy.at(index),
                        distance,
                        std::min(distance, point.z_m - floor) -
                            margins.total_m(),
                        floor,
                        data_->seafloor_quality.at(floor_index),
                        true,
                        data_->source,
                        {},
                        {},
                        {*voxel}};

  switch (result.state) {
    case MapCellState::unknown:
      result.information_gaps.push_back(
          {MapInformationGapKind::unknown_occupancy, point});
      break;
    case MapCellState::stale:
      result.information_gaps.push_back(
          {MapInformationGapKind::stale_occupancy, point});
      break;
    case MapCellState::conflicted:
      result.information_gaps.push_back(
          {MapInformationGapKind::conflicted_occupancy, point});
      break;
    case MapCellState::free:
    case MapCellState::occupied:
      break;
  }

  const auto low = data_->grid.minimum();
  const auto high = data_->grid.maximum();
  const double boundary_distance =
      std::min({point.x_m - low.x_m, point.y_m - low.y_m,
                point.z_m - low.z_m, high.x_m - point.x_m,
                high.y_m - point.y_m, high.z_m - point.z_m,
                point.x_m - data_->map_bounds.minimum_m.x_m,
                point.y_m - data_->map_bounds.minimum_m.y_m,
                point.z_m - data_->map_bounds.minimum_m.z_m,
                data_->map_bounds.maximum_m.x_m - point.x_m,
                data_->map_bounds.maximum_m.y_m - point.y_m,
                data_->map_bounds.maximum_m.z_m - point.z_m});
  result.clearance_margin_m = std::min(
      result.clearance_margin_m, boundary_distance - margins.total_m());
  if (boundary_distance < margins.total_m()) {
    if (state_rank(result.state) < state_rank(MapCellState::unknown)) {
      result.state = MapCellState::unknown;
    }
    result.information_gaps.push_back(
        {MapInformationGapKind::boundary_clearance, point});
  }

  for (const auto candidate :
       nearby_voxels(data_->grid, *voxel, margins.total_m())) {
    if (!data_->allowed_water.at(linear_index(data_->grid, candidate)) &&
        contains(expanded(voxel_bounds(data_->grid, candidate),
                          margins.total_m()),
                 point)) {
      result.allowed_water = false;
      break;
    }
  }
  for (const auto& region : data_->semantic_regions) {
    if (contains(expanded(region.bounds, margins.total_m()), point)) {
      result.semantic_restrictions.push_back(region.restriction);
      if (region.restriction == MapSemanticRestriction::no_entry &&
          state_rank(result.state) < state_rank(MapCellState::occupied)) {
        result.state = MapCellState::occupied;
      }
    }
  }
  if ((!result.allowed_water || result.clearance_margin_m < 0.0) &&
      state_rank(result.state) < state_rank(MapCellState::occupied)) {
    result.state = MapCellState::occupied;
  }
  return MapQueryResult<MapQuerySample>::success(std::move(result));
}

MapQueryResult<MapQuerySample> HybridMapQuery::query_supercover(
    const Point3dEnu start, const Point3dEnu end,
    const SafetyMargins& margins) const {
  const auto start_query = query_point(start, margins);
  if (!start_query.has_value()) {
    return start_query;
  }
  const auto end_query = query_point(end, margins);
  if (!end_query.has_value()) {
    return end_query;
  }
  if (!voxel_at(data_->grid, start).has_value() ||
      !voxel_at(data_->grid, end).has_value()) {
    auto result = start_query.value();
    if (state_rank(end_query.value().state) > state_rank(result.state)) {
      result.state = end_query.value().state;
    }
    result.information_gaps.insert(result.information_gaps.end(),
                                   end_query.value().information_gaps.begin(),
                                   end_query.value().information_gaps.end());
    return MapQueryResult<MapQuerySample>::success(std::move(result));
  }

  std::vector<double> events{0.0, 1.0};
  const auto low = data_->grid.minimum();
  const std::array start_value{start.x_m, start.y_m, start.z_m};
  const std::array end_value{end.x_m, end.y_m, end.z_m};
  const std::array low_value{low.x_m, low.y_m, low.z_m};
  const std::array resolution{data_->grid.resolution.x_m,
                              data_->grid.resolution.y_m,
                              data_->grid.resolution.z_m};
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    const double delta = end_value[axis] - start_value[axis];
    if (delta == 0.0) {
      continue;
    }
    const double start_scaled =
        (start_value[axis] - low_value[axis]) / resolution[axis];
    const double end_scaled =
        (end_value[axis] - low_value[axis]) / resolution[axis];
    const auto first_plane = static_cast<std::uint32_t>(std::max(
        1.0, std::floor(std::min(start_scaled, end_scaled)) + 1.0));
    const auto last_plane = static_cast<std::uint32_t>(std::min(
        static_cast<double>(data_->grid.count[axis] - 1U),
        std::ceil(std::max(start_scaled, end_scaled))));
    for (std::uint32_t plane = first_plane; plane <= last_plane; ++plane) {
      const double coordinate =
          low_value[axis] + resolution[axis] * static_cast<double>(plane);
      const double t = (coordinate - start_value[axis]) / delta;
      if (t > 0.0 && t < 1.0) {
        events.push_back(t);
      }
    }
  }
  std::sort(events.begin(), events.end());
  events.erase(std::unique(events.begin(), events.end(), [](const double left,
                                                            const double right) {
                 return std::abs(left - right) <= kBoundaryTolerance;
               }),
               events.end());

  std::set<std::size_t> visited;
  const auto add_point = [&](const double t) {
    const Point3dEnu point{start.x_m + (end.x_m - start.x_m) * t,
                           start.y_m + (end.y_m - start.y_m) * t,
                           start.z_m + (end.z_m - start.z_m) * t};
    std::array<std::vector<std::uint32_t>, 3U> candidates;
    const std::array coordinate{point.x_m, point.y_m, point.z_m};
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
      const double scaled =
          (coordinate[axis] - low_value[axis]) / resolution[axis];
      const double nearest = std::round(scaled);
      if (std::abs(scaled - nearest) <= kBoundaryTolerance && nearest > 0.0 &&
          nearest < static_cast<double>(data_->grid.count[axis])) {
        const auto upper = static_cast<std::uint32_t>(nearest);
        candidates[axis] = {upper - 1U, upper};
      } else {
        candidates[axis] = {
            static_cast<std::uint32_t>(std::floor(scaled))};
      }
    }
    for (const auto x : candidates[0]) {
      for (const auto y : candidates[1]) {
        for (const auto z : candidates[2]) {
          visited.insert(linear_index(data_->grid, {x, y, z}));
        }
      }
    }
  };
  for (const double event : events) {
    add_point(event);
  }
  for (std::size_t index = 1U; index < events.size(); ++index) {
    add_point(0.5 * (events[index - 1U] + events[index]));
  }

  MapQuerySample aggregate = start_query.value();
  aggregate.information_gaps.clear();
  aggregate.semantic_restrictions.clear();
  aggregate.queried_voxels.clear();
  aggregate.esdf_distance_m = std::numeric_limits<double>::infinity();
  aggregate.clearance_margin_m = std::numeric_limits<double>::infinity();
  aggregate.seafloor_elevation_m = -std::numeric_limits<double>::infinity();
  aggregate.quality = 1.0F;
  aggregate.allowed_water = true;
  aggregate.state = MapCellState::free;

  for (const auto index : visited) {
    const auto voxel = voxel_from_linear(data_->grid, index);
    const auto sample = query_point(voxel_center(data_->grid, voxel),
                                    SafetyMargins{0.0, 0.0, 0.0, 0.0, 0.0});
    const auto& value = sample.value();
    aggregate.queried_voxels.push_back(voxel);
    aggregate.esdf_distance_m =
        std::min(aggregate.esdf_distance_m, value.esdf_distance_m);
    aggregate.clearance_margin_m =
        std::min(aggregate.clearance_margin_m,
                 value.esdf_distance_m - margins.total_m());
    const auto interval = segment_aabb_interval(
        start, end, voxel_bounds(data_->grid, voxel));
    if (interval.has_value()) {
      const double first_z =
          start.z_m + (end.z_m - start.z_m) * (*interval)[0];
      const double second_z =
          start.z_m + (end.z_m - start.z_m) * (*interval)[1];
      aggregate.clearance_margin_m = std::min(
          aggregate.clearance_margin_m,
          std::min(first_z, second_z) - value.seafloor_elevation_m -
              margins.total_m());
    }
    aggregate.seafloor_elevation_m =
        std::max(aggregate.seafloor_elevation_m,
                 value.seafloor_elevation_m);
    aggregate.quality = std::min(aggregate.quality, value.quality);
    aggregate.allowed_water = aggregate.allowed_water && value.allowed_water;
    if (state_rank(value.state) > state_rank(aggregate.state)) {
      aggregate.state = value.state;
    }
    aggregate.information_gaps.insert(aggregate.information_gaps.end(),
                                      value.information_gaps.begin(),
                                      value.information_gaps.end());
    for (const auto restriction : value.semantic_restrictions) {
      if (std::find(aggregate.semantic_restrictions.begin(),
                    aggregate.semantic_restrictions.end(), restriction) ==
          aggregate.semantic_restrictions.end()) {
        aggregate.semantic_restrictions.push_back(restriction);
      }
    }
  }
  if (state_rank(start_query.value().state) > state_rank(aggregate.state)) {
    aggregate.state = start_query.value().state;
  }
  if (state_rank(end_query.value().state) > state_rank(aggregate.state)) {
    aggregate.state = end_query.value().state;
  }
  aggregate.information_gaps.insert(
      aggregate.information_gaps.end(),
      start_query.value().information_gaps.begin(),
      start_query.value().information_gaps.end());
  aggregate.information_gaps.insert(aggregate.information_gaps.end(),
                                    end_query.value().information_gaps.begin(),
                                    end_query.value().information_gaps.end());
  aggregate.clearance_margin_m =
      std::min({aggregate.clearance_margin_m,
                start_query.value().clearance_margin_m,
                end_query.value().clearance_margin_m});
  std::set<std::size_t> allowed_water_candidates;
  for (const auto index : visited) {
    const auto voxel = voxel_from_linear(data_->grid, index);
    for (const auto candidate :
         nearby_voxels(data_->grid, voxel, margins.total_m())) {
      allowed_water_candidates.insert(linear_index(data_->grid, candidate));
    }
  }
  for (const auto index : allowed_water_candidates) {
    if (data_->allowed_water.at(index)) {
      continue;
    }
    const auto bounds = expanded(
        voxel_bounds(data_->grid, voxel_from_linear(data_->grid, index)),
        margins.total_m());
    if (segment_aabb_interval(start, end, bounds).has_value()) {
      aggregate.allowed_water = false;
      break;
    }
  }
  for (const auto& region : data_->semantic_regions) {
    if (!segment_aabb_interval(start, end,
                               expanded(region.bounds, margins.total_m()))
             .has_value()) {
      continue;
    }
    if (std::find(aggregate.semantic_restrictions.begin(),
                  aggregate.semantic_restrictions.end(), region.restriction) ==
        aggregate.semantic_restrictions.end()) {
      aggregate.semantic_restrictions.push_back(region.restriction);
    }
    if (region.restriction == MapSemanticRestriction::no_entry &&
        state_rank(aggregate.state) < state_rank(MapCellState::occupied)) {
      aggregate.state = MapCellState::occupied;
    }
  }
  if ((!aggregate.allowed_water || aggregate.clearance_margin_m < 0.0) &&
      state_rank(aggregate.state) < state_rank(MapCellState::occupied)) {
    aggregate.state = MapCellState::occupied;
  }
  return MapQueryResult<MapQuerySample>::success(std::move(aggregate));
}

}  // namespace scout_planner::core
