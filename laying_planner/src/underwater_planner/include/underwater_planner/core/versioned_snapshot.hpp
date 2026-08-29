#pragma once

#include "underwater_planner/core/data_contract.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace underwater_planner::core {

struct Vector2d {
  double x{};
  double y{};
};

struct MapCell {
  double elevation_m{};
  double elevation_variance_m2{};
  double confidence{};
  bool known{};
  MonotonicTime measurement_timestamp;
  bool obstacle{};
  std::optional<Vector2d> obstacle_normal;
  bool cable_forbidden{};
};

struct MapUpdateRegion {
  double min_x_m{};
  double min_y_m{};
  double max_x_m{};
  double max_y_m{};
};

struct MapSnapshot {
  MapVersion version;
  std::size_t width{};
  std::size_t height{};
  double resolution_m{};
  double origin_x_m{};
  double origin_y_m{};
  std::vector<MapCell> cells;
  std::uint64_t derived_configuration_version{};
  std::vector<MapUpdateRegion> update_regions;

  [[nodiscard]] const MapCell& at(std::size_t row, std::size_t column) const;
};

struct ReferencePoint {
  double arc_length_m{};
  double x_m{};
  double y_m{};
  double tangent_x{};
  double tangent_y{};
  double normal_x{};
  double normal_y{};
};

struct ReferenceProjection {
  ReferencePoint point;
  double distance_m{};
};

struct ReferenceLine {
  std::uint32_t version{};
  std::string coordinate_frame;
  std::vector<ReferencePoint> points;

  [[nodiscard]] std::optional<ReferencePoint> query(double arc_length_m) const;
  [[nodiscard]] std::vector<ReferencePoint> local_window(
      double center_arc_length_m, double half_window_m) const;
  [[nodiscard]] std::vector<ReferenceProjection> local_projection_candidates(
      Vector2m position_m, double lower_arc_length_m,
      double upper_arc_length_m) const;
};

[[nodiscard]] ReferenceLine make_reference_line(
    std::uint32_t version, std::string coordinate_frame,
    const std::vector<Vector2m>& positions);

struct Point2d {
  double x_m{};
  double y_m{};
};

struct RobotOperatingArea {
  std::uint32_t version{};
  std::string id;
  std::vector<Point2d> polygon;

  [[nodiscard]] bool contains_footprint(
      const std::vector<Point2d>& footprint_body_m,
      const Pose2d& robot_pose) const;
  [[nodiscard]] bool contains_footprint_with_clearance(
      const std::vector<Point2d>& footprint_body_m, const Pose2d& robot_pose,
      double boundary_clearance_m) const;
};

struct CableCorridor {
  std::uint32_t version{};
  std::string id;
  std::vector<Point2d> polygon;
};

struct VersionedPlanningSnapshot {
  MapSnapshot map;
  ReferenceLine reference_line;
  RobotOperatingArea robot_operating_area;
  CableCorridor cable_corridor;
};

struct SnapshotValidation {
  bool valid{};
  std::vector<std::string> issues;
};

[[nodiscard]] SnapshotValidation validate(const MapSnapshot& map);
[[nodiscard]] SnapshotValidation validate(const ReferenceLine& reference_line);
[[nodiscard]] SnapshotValidation validate(const RobotOperatingArea& area);
[[nodiscard]] SnapshotValidation validate(const CableCorridor& corridor);
[[nodiscard]] SnapshotValidation validate(
    const VersionedPlanningSnapshot& snapshot);

enum class SnapshotUpdateStatus {
  accepted,
  duplicate,
  out_of_order,
  expired,
  version_rollback,
  invalid,
};

struct SnapshotUpdateResult {
  SnapshotUpdateStatus status{SnapshotUpdateStatus::invalid};
  std::vector<std::string> issues;
  [[nodiscard]] bool accepted() const noexcept {
    return status == SnapshotUpdateStatus::accepted;
  }
};

class SnapshotManager {
 public:
  explicit SnapshotManager(Duration maximum_age = Duration{0});

  [[nodiscard]] SnapshotUpdateResult update(
      const VersionedPlanningSnapshot& snapshot, MonotonicTime now);
  [[nodiscard]] std::optional<VersionedPlanningSnapshot> latest() const;
  [[nodiscard]] bool is_current(const VersionedPlanningSnapshot& snapshot) const;

 private:
  Duration maximum_age_;
  std::optional<VersionedPlanningSnapshot> latest_;
};

}  // namespace underwater_planner::core
