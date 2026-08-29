#pragma once

#include "scout_planner/core/protobuf_adapter.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace scout_planner::core {

struct Point3dEnu {
  double x_m;
  double y_m;
  double z_m;
};

struct Aabb3dEnu {
  Point3dEnu minimum_m;
  Point3dEnu maximum_m;
};

struct VoxelIndex3d {
  std::uint32_t x;
  std::uint32_t y;
  std::uint32_t z;

  [[nodiscard]] bool operator==(const VoxelIndex3d& other) const noexcept {
    return x == other.x && y == other.y && z == other.z;
  }
};

enum class MapCellState { free, occupied, unknown, stale, conflicted };

enum class MapInformationGapKind {
  outside_snapshot,
  boundary_clearance,
  unknown_occupancy,
  stale_occupancy,
  conflicted_occupancy,
};

enum class MapSemanticRestriction {
  no_entry,
  communication_shadow,
  special_region,
};

struct MapInformationGap {
  MapInformationGapKind kind;
  Point3dEnu location_m;
};

struct SafetyMargins {
  double body_m;
  double localization_m;
  double tracking_m;
  double map_m;
  double discretization_m;

  [[nodiscard]] double total_m() const noexcept;
};

struct MapSnapshotIdentity {
  std::string map_id;
  std::uint64_t map_version;
  Hash256 content_identity;
};

struct MapGridInfo {
  Point3dEnu origin_m{};
  Point3dEnu resolution_m{};
  std::array<std::uint32_t, 3U> cell_count{};
  Aabb3dEnu bounds{};
};

enum class MapTimeSyncStatus { unsynchronized, synchronized, degraded };

struct MapSourceVersion {
  std::string map_id;
  std::uint64_t map_version;
  Hash256 content_identity;
  std::string source_clock_domain_id;
  std::int64_t generated_at_monotonic_ns;
  std::int64_t observed_at_utc_ns;
  MapTimeSyncStatus observation_sync_status;
  std::uint64_t observation_uncertainty_ns;
  Aabb3dEnu map_region_bbox;
  std::uint64_t sensor_extrinsic_version;
  std::uint64_t mapping_parameter_version;
};

struct MapQuerySample {
  MapCellState state;
  double esdf_distance_m;
  double clearance_margin_m;
  double seafloor_elevation_m;
  float quality;
  bool allowed_water;
  MapSourceVersion source;
  std::vector<MapSemanticRestriction> semantic_restrictions;
  std::vector<MapInformationGap> information_gaps;
  std::vector<VoxelIndex3d> queried_voxels;
};

enum class MapQueryErrorCode {
  invalid_dependency,
  malformed_snapshot,
  invalid_position,
  invalid_safety_margin,
};

struct MapQueryError {
  MapQueryErrorCode code;
  std::string detail;
};

template <typename T>
class MapQueryResult final {
 public:
  [[nodiscard]] static MapQueryResult success(T value) {
    return MapQueryResult(std::move(value));
  }

  [[nodiscard]] static MapQueryResult failure(MapQueryError error) {
    return MapQueryResult(std::move(error));
  }

  [[nodiscard]] bool has_value() const noexcept {
    return std::holds_alternative<T>(storage_);
  }
  [[nodiscard]] const T& value() const { return std::get<T>(storage_); }
  [[nodiscard]] T& value() { return std::get<T>(storage_); }
  [[nodiscard]] const MapQueryError& error() const noexcept {
    static const MapQueryError no_error{MapQueryErrorCode::malformed_snapshot,
                                        {}};
    const auto* error = std::get_if<MapQueryError>(&storage_);
    return error == nullptr ? no_error : *error;
  }

 private:
  explicit MapQueryResult(T value) : storage_(std::move(value)) {}
  explicit MapQueryResult(MapQueryError error)
      : storage_(std::move(error)) {}

  std::variant<T, MapQueryError> storage_;
};

class HybridMapQuery final {
 public:
  [[nodiscard]] static MapQueryResult<HybridMapQuery> create(
      const HybridMapSnapshot& snapshot,
      const MapSnapshotIdentity& expected_identity);

  [[nodiscard]] MapQueryResult<MapQuerySample> query_point(
      Point3dEnu point, const SafetyMargins& margins) const;
  [[nodiscard]] MapQueryResult<MapQuerySample> query_supercover(
      Point3dEnu start, Point3dEnu end, const SafetyMargins& margins) const;
  [[nodiscard]] const MapGridInfo& grid_info() const noexcept;

 private:
  struct Data;
  explicit HybridMapQuery(std::shared_ptr<const Data> data)
      : data_(std::move(data)) {}

  std::shared_ptr<const Data> data_;
};

}  // namespace scout_planner::core
