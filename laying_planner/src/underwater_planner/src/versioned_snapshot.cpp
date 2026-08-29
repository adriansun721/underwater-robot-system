#include "underwater_planner/core/versioned_snapshot.hpp"

#include "planar_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace underwater_planner::core {
namespace {

bool finite(const double value) { return std::isfinite(value); }
void issue(SnapshotValidation& result, const char* message) {
  result.issues.emplace_back(message);
}
SnapshotValidation valid_result() { return SnapshotValidation{true, {}}; }

SnapshotValidation validate_polygon(const std::vector<Point2d>& polygon,
                                    const char* name) {
  SnapshotValidation result = valid_result();
  if (polygon.size() < 3U) {
    result.valid = false;
    result.issues.emplace_back(std::string{name} + " must be non-empty polygon");
    return result;
  }
  for (const Point2d point : polygon) {
    if (!finite(point.x_m) || !finite(point.y_m)) {
      result.valid = false;
      result.issues.emplace_back(std::string{name} + " contains non-finite point");
    }
  }
  double twice_area = 0.0;
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const Point2d& left = polygon[index];
    const Point2d& right = polygon[(index + 1) % polygon.size()];
    twice_area += left.x_m * right.y_m - right.x_m * left.y_m;
  }
  if (std::abs(twice_area) <= 1.0e-12) {
    result.valid = false;
    result.issues.emplace_back(std::string{name} + " must have non-zero area");
  }
  return result;
}

constexpr double kGeometryTolerance = 1.0e-12;

void append_intersection_parameters(const Point2d& segment_start,
                                    const Point2d& segment_end,
                                    const Point2d& boundary_start,
                                    const Point2d& boundary_end,
                                    std::vector<double>& parameters) {
  const double segment_x = segment_end.x_m - segment_start.x_m;
  const double segment_y = segment_end.y_m - segment_start.y_m;
  const double boundary_x = boundary_end.x_m - boundary_start.x_m;
  const double boundary_y = boundary_end.y_m - boundary_start.y_m;
  const double offset_x = boundary_start.x_m - segment_start.x_m;
  const double offset_y = boundary_start.y_m - segment_start.y_m;
  const double denominator = segment_x * boundary_y - segment_y * boundary_x;
  if (std::abs(denominator) > kGeometryTolerance) {
    const double segment_parameter =
        (offset_x * boundary_y - offset_y * boundary_x) / denominator;
    const double boundary_parameter =
        (offset_x * segment_y - offset_y * segment_x) / denominator;
    if (segment_parameter >= -kGeometryTolerance &&
        segment_parameter <= 1.0 + kGeometryTolerance &&
        boundary_parameter >= -kGeometryTolerance &&
        boundary_parameter <= 1.0 + kGeometryTolerance) {
      parameters.push_back(std::clamp(segment_parameter, 0.0, 1.0));
    }
    return;
  }
  if (std::abs(offset_x * segment_y - offset_y * segment_x) >
      kGeometryTolerance) {
    return;
  }
  const double squared_length =
      segment_x * segment_x + segment_y * segment_y;
  if (squared_length <= kGeometryTolerance) {
    return;
  }
  const auto projection = [&](const Point2d& point) {
    return ((point.x_m - segment_start.x_m) * segment_x +
            (point.y_m - segment_start.y_m) * segment_y) /
           squared_length;
  };
  const double overlap_start =
      std::max(0.0, std::min(projection(boundary_start),
                             projection(boundary_end)));
  const double overlap_end =
      std::min(1.0, std::max(projection(boundary_start),
                             projection(boundary_end)));
  if (overlap_start <= overlap_end + kGeometryTolerance) {
    parameters.push_back(std::clamp(overlap_start, 0.0, 1.0));
    parameters.push_back(std::clamp(overlap_end, 0.0, 1.0));
  }
}

bool polygon_contains_polygon(const std::vector<Point2d>& outer,
                              const std::vector<Point2d>& inner) {
  if (std::any_of(inner.begin(), inner.end(), [&](const Point2d& point) {
        return detail::locate_point(point, outer) ==
               detail::PointLocation::outside;
      })) {
    return false;
  }

  // A concave outer boundary can enter the inner polygon even when every
  // inner vertex is covered. Split each outer edge at all boundary contacts
  // and reject any open subsegment lying in the footprint interior.
  for (std::size_t outer_index = 0U; outer_index < outer.size(); ++outer_index) {
    const Point2d& outer_start = outer[outer_index];
    const Point2d& outer_end = outer[(outer_index + 1U) % outer.size()];
    std::vector<double> parameters{0.0, 1.0};
    for (std::size_t inner_index = 0U; inner_index < inner.size();
         ++inner_index) {
      append_intersection_parameters(
          outer_start, outer_end, inner[inner_index],
          inner[(inner_index + 1U) % inner.size()], parameters);
    }
    std::sort(parameters.begin(), parameters.end());
    parameters.erase(
        std::unique(parameters.begin(), parameters.end(),
                    [](const double left, const double right) {
                      return std::abs(left - right) <= kGeometryTolerance;
                    }),
        parameters.end());
    for (std::size_t index = 1U; index < parameters.size(); ++index) {
      if (parameters[index] - parameters[index - 1U] <=
          kGeometryTolerance) {
        continue;
      }
      const double midpoint_parameter =
          (parameters[index] + parameters[index - 1U]) * 0.5;
      const Point2d midpoint{
          outer_start.x_m +
              midpoint_parameter * (outer_end.x_m - outer_start.x_m),
          outer_start.y_m +
              midpoint_parameter * (outer_end.y_m - outer_start.y_m)};
      if (detail::locate_point(midpoint, inner) ==
          detail::PointLocation::inside) {
        return false;
      }
    }
  }
  return true;
}

bool same_points(const std::vector<ReferencePoint>& left,
                 const std::vector<ReferencePoint>& right) {
  if (left.size() != right.size()) return false;
  for (std::size_t i = 0; i < left.size(); ++i) {
    if (left[i].arc_length_m != right[i].arc_length_m || left[i].x_m != right[i].x_m ||
        left[i].y_m != right[i].y_m || left[i].tangent_x != right[i].tangent_x ||
        left[i].tangent_y != right[i].tangent_y || left[i].normal_x != right[i].normal_x ||
        left[i].normal_y != right[i].normal_y) return false;
  }
  return true;
}
bool same_polygon(const std::vector<Point2d>& left, const std::vector<Point2d>& right) {
  if (left.size() != right.size()) return false;
  for (std::size_t i = 0; i < left.size(); ++i)
    if (left[i].x_m != right[i].x_m || left[i].y_m != right[i].y_m) return false;
  return true;
}
bool same_regions(const std::vector<MapUpdateRegion>& left,
                  const std::vector<MapUpdateRegion>& right) {
  if (left.size() != right.size()) return false;
  for (std::size_t i = 0; i < left.size(); ++i)
    if (left[i].min_x_m != right[i].min_x_m || left[i].min_y_m != right[i].min_y_m ||
        left[i].max_x_m != right[i].max_x_m || left[i].max_y_m != right[i].max_y_m) return false;
  return true;
}
bool same_map_payload(const MapSnapshot& left, const MapSnapshot& right) {
  if (left.version != right.version || left.width != right.width ||
      left.height != right.height || left.resolution_m != right.resolution_m ||
      left.origin_x_m != right.origin_x_m || left.origin_y_m != right.origin_y_m ||
      left.derived_configuration_version != right.derived_configuration_version ||
      left.cells.size() != right.cells.size() ||
      !same_regions(left.update_regions, right.update_regions)) {
    return false;
  }
  for (std::size_t index = 0; index < left.cells.size(); ++index) {
    const MapCell& a = left.cells[index];
    const MapCell& b = right.cells[index];
    if (a.elevation_m != b.elevation_m || a.elevation_variance_m2 != b.elevation_variance_m2 ||
        a.confidence != b.confidence || a.known != b.known ||
        a.obstacle != b.obstacle ||
        a.cable_forbidden != b.cable_forbidden ||
        a.obstacle_normal.has_value() != b.obstacle_normal.has_value() ||
        a.measurement_timestamp.nanoseconds !=
            b.measurement_timestamp.nanoseconds) return false;
    if (a.obstacle_normal.has_value() &&
        (a.obstacle_normal->x != b.obstacle_normal->x ||
         a.obstacle_normal->y != b.obstacle_normal->y)) {
      return false;
    }
  }
  return true;
}

bool same_reference_payload(const ReferenceLine& left,
                            const ReferenceLine& right) {
  return left.version == right.version &&
         left.coordinate_frame == right.coordinate_frame &&
         same_points(left.points, right.points);
}

bool same_area_payload(const RobotOperatingArea& left,
                       const RobotOperatingArea& right) {
  return left.version == right.version && left.id == right.id &&
         same_polygon(left.polygon, right.polygon);
}

bool same_corridor_payload(const CableCorridor& left,
                           const CableCorridor& right) {
  return left.version == right.version && left.id == right.id &&
         same_polygon(left.polygon, right.polygon);
}

bool same_payload(const VersionedPlanningSnapshot& left,
                  const VersionedPlanningSnapshot& right) {
  return same_map_payload(left.map, right.map) &&
         same_reference_payload(left.reference_line, right.reference_line) &&
         same_area_payload(left.robot_operating_area,
                           right.robot_operating_area) &&
         same_corridor_payload(left.cable_corridor, right.cable_corridor);
}

}  // namespace

const MapCell& MapSnapshot::at(const std::size_t row,
                               const std::size_t column) const {
  if (row >= height || column >= width || cells.size() != width * height) {
    throw std::out_of_range("map snapshot cell is outside the map");
  }
  return cells.at(row * width + column);
}

bool RobotOperatingArea::contains_footprint(
    const std::vector<Point2d>& footprint_body_m,
    const Pose2d& robot_pose) const {
  return contains_footprint_with_clearance(footprint_body_m, robot_pose, 0.0);
}

bool RobotOperatingArea::contains_footprint_with_clearance(
    const std::vector<Point2d>& footprint_body_m, const Pose2d& robot_pose,
    const double boundary_clearance_m) const {
  if (!validate(*this).valid ||
      !validate_polygon(footprint_body_m, "robot footprint").valid ||
      !finite(robot_pose.x_m) || !finite(robot_pose.y_m) ||
      !finite(robot_pose.heading_rad) || !finite(boundary_clearance_m) ||
      boundary_clearance_m < 0.0) {
    return false;
  }
  const double cosine = std::cos(robot_pose.heading_rad);
  const double sine = std::sin(robot_pose.heading_rad);
  std::vector<Point2d> footprint_world_m;
  footprint_world_m.reserve(footprint_body_m.size());
  for (const Point2d& point : footprint_body_m) {
    footprint_world_m.push_back(
        {robot_pose.x_m + cosine * point.x_m - sine * point.y_m,
         robot_pose.y_m + sine * point.x_m + cosine * point.y_m});
  }
  if (!polygon_contains_polygon(polygon, footprint_world_m)) {
    return false;
  }
  if (boundary_clearance_m == 0.0) {
    return true;
  }
  for (std::size_t area_index = 0U; area_index < polygon.size(); ++area_index) {
    const Point2d& area_start = polygon[area_index];
    const Point2d& area_end = polygon[(area_index + 1U) % polygon.size()];
    for (std::size_t footprint_index = 0U;
         footprint_index < footprint_world_m.size(); ++footprint_index) {
      const Point2d& footprint_start = footprint_world_m[footprint_index];
      const Point2d& footprint_end =
          footprint_world_m[(footprint_index + 1U) % footprint_world_m.size()];
      if (detail::segment_distance(area_start, area_end, footprint_start,
                                   footprint_end) < boundary_clearance_m) {
        return false;
      }
    }
  }
  return true;
}

std::optional<ReferencePoint> ReferenceLine::query(
    const double arc_length_m) const {
  if (points.empty() || !finite(arc_length_m) ||
      arc_length_m < points.front().arc_length_m ||
      arc_length_m > points.back().arc_length_m) {
    return std::nullopt;
  }
  const auto upper = std::lower_bound(
      points.begin(), points.end(), arc_length_m,
      [](const ReferencePoint& point, const double value) {
        return point.arc_length_m < value;
      });
  if (upper == points.begin() || upper == points.end() ||
      upper->arc_length_m == arc_length_m) {
    return upper == points.end() ? points.back() : *upper;
  }
  const ReferencePoint& left = *(upper - 1);
  const ReferencePoint& right = *upper;
  const double span = right.arc_length_m - left.arc_length_m;
  const double ratio = (arc_length_m - left.arc_length_m) / span;
  ReferencePoint result;
  result.arc_length_m = arc_length_m;
  result.x_m = left.x_m + ratio * (right.x_m - left.x_m);
  result.y_m = left.y_m + ratio * (right.y_m - left.y_m);
  result.tangent_x = left.tangent_x + ratio * (right.tangent_x - left.tangent_x);
  result.tangent_y = left.tangent_y + ratio * (right.tangent_y - left.tangent_y);
  const double tangent_norm = std::hypot(result.tangent_x, result.tangent_y);
  if (tangent_norm > 0.0) {
    result.tangent_x /= tangent_norm;
    result.tangent_y /= tangent_norm;
  }
  result.normal_x = -result.tangent_y;
  result.normal_y = result.tangent_x;
  return result;
}

std::vector<ReferencePoint> ReferenceLine::local_window(
    const double center_arc_length_m, const double half_window_m) const {
  if (!finite(center_arc_length_m) || !finite(half_window_m) ||
      half_window_m < 0.0) {
    return {};
  }
  std::vector<ReferencePoint> result;
  if (points.empty() || center_arc_length_m + half_window_m < points.front().arc_length_m ||
      center_arc_length_m - half_window_m > points.back().arc_length_m) {
    return result;
  }
  const double lower = std::max(center_arc_length_m - half_window_m,
                                points.front().arc_length_m);
  const double upper = std::min(center_arc_length_m + half_window_m,
                                points.back().arc_length_m);
  if (const auto left = query(lower); left.has_value()) result.push_back(*left);
  for (const ReferencePoint& point : points) {
    if (point.arc_length_m > lower && point.arc_length_m < upper) {
      result.push_back(point);
    }
  }
  if (upper > lower) {
    if (const auto right = query(upper); right.has_value() &&
        (result.empty() || result.back().arc_length_m != right->arc_length_m)) {
      result.push_back(*right);
    }
  }
  return result;
}

std::vector<ReferenceProjection> ReferenceLine::local_projection_candidates(
    const Vector2m position_m, const double lower_arc_length_m,
    const double upper_arc_length_m) const {
  std::vector<ReferenceProjection> result;
  if (!finite(position_m.x_m) || !finite(position_m.y_m) ||
      !finite(lower_arc_length_m) || !finite(upper_arc_length_m) ||
      lower_arc_length_m > upper_arc_length_m) {
    return result;
  }
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const ReferencePoint& left = points[index - 1U];
    const ReferencePoint& right = points[index];
    const double segment_lower =
        std::max(lower_arc_length_m, left.arc_length_m);
    const double segment_upper =
        std::min(upper_arc_length_m, right.arc_length_m);
    if (segment_lower > segment_upper) continue;

    const double segment_span = right.arc_length_m - left.arc_length_m;
    const double dx = right.x_m - left.x_m;
    const double dy = right.y_m - left.y_m;
    const double length_squared = dx * dx + dy * dy;
    if (!(segment_span > 0.0) || !(length_squared > 0.0)) continue;
    const double projected_ratio =
        ((position_m.x_m - left.x_m) * dx +
         (position_m.y_m - left.y_m) * dy) /
        length_squared;
    const double projected_arc_length_m = std::clamp(
        left.arc_length_m + projected_ratio * segment_span, segment_lower,
        segment_upper);
    const std::optional<ReferencePoint> projected =
        query(projected_arc_length_m);
    if (!projected.has_value()) continue;
    result.push_back(
        {*projected,
         std::hypot(position_m.x_m - projected->x_m,
                    position_m.y_m - projected->y_m)});
  }
  return result;
}

ReferenceLine make_reference_line(const std::uint32_t version,
                                  std::string coordinate_frame,
                                  const std::vector<Vector2m>& positions) {
  ReferenceLine result{version, std::move(coordinate_frame), {}};
  if (positions.empty()) return result;
  result.points.reserve(positions.size());
  double arc = 0.0;
  for (std::size_t index = 0; index < positions.size(); ++index) {
    if (index > 0) {
      arc += std::hypot(positions[index].x_m - positions[index - 1].x_m,
                        positions[index].y_m - positions[index - 1].y_m);
    }
    const std::size_t next = std::min(index + 1, positions.size() - 1);
    const std::size_t previous = index == 0 ? 0 : index - 1;
    const double dx = positions[next].x_m - positions[previous].x_m;
    const double dy = positions[next].y_m - positions[previous].y_m;
    const double norm = std::hypot(dx, dy);
    result.points.push_back({arc, positions[index].x_m, positions[index].y_m,
                             norm > 0.0 ? dx / norm : 0.0,
                             norm > 0.0 ? dy / norm : 0.0,
                             norm > 0.0 ? -dy / norm : 0.0,
                             norm > 0.0 ? dx / norm : 0.0});
  }
  return result;
}

SnapshotValidation validate(const MapSnapshot& map) {
  SnapshotValidation result = valid_result();
  if (map.version.map_id.empty() || map.version.sequence_number == 0U ||
      map.version.timestamp.nanoseconds < 0 || map.version.coordinate_frame.empty()) {
    result.valid = false;
    issue(result, "map version is incomplete");
  }
  if (map.width == 0U || map.height == 0U || map.cells.size() != map.width * map.height ||
      !finite(map.resolution_m) || map.resolution_m <= 0.0 ||
      !finite(map.origin_x_m) || !finite(map.origin_y_m) ||
      map.derived_configuration_version == 0U) {
    result.valid = false;
    issue(result, "map geometry or derived configuration is invalid");
  }
  for (const MapCell cell : map.cells) {
    if (!finite(cell.elevation_m) || !finite(cell.elevation_variance_m2) ||
        cell.elevation_variance_m2 < 0.0 || !finite(cell.confidence) ||
        cell.confidence < 0.0 || cell.confidence > 1.0) {
      result.valid = false;
      issue(result, "map cell contains invalid value");
      break;
    }
    if (cell.obstacle_normal.has_value() &&
        (!cell.obstacle || !finite(cell.obstacle_normal->x) ||
         !finite(cell.obstacle_normal->y) ||
         std::hypot(cell.obstacle_normal->x, cell.obstacle_normal->y) <=
             1.0e-12)) {
      result.valid = false;
      issue(result, "map obstacle normal is invalid");
      break;
    }
  }
  for (const MapUpdateRegion region : map.update_regions) {
    if (!finite(region.min_x_m) || !finite(region.min_y_m) ||
        !finite(region.max_x_m) || !finite(region.max_y_m) ||
        region.min_x_m > region.max_x_m || region.min_y_m > region.max_y_m) {
      result.valid = false;
      issue(result, "map update region is invalid");
    }
  }
  return result;
}

SnapshotValidation validate(const ReferenceLine& line) {
  SnapshotValidation result = valid_result();
  if (line.version == 0U || line.coordinate_frame.empty() || line.points.size() < 2U) {
    result.valid = false;
    issue(result, "reference line version, frame, or points are invalid");
  }
  double previous = -1.0;
  for (const ReferencePoint point : line.points) {
    if (!finite(point.arc_length_m) || point.arc_length_m <= previous ||
        !finite(point.x_m) || !finite(point.y_m) ||
        !finite(point.tangent_x) || !finite(point.tangent_y) ||
        !finite(point.normal_x) || !finite(point.normal_y)) {
      result.valid = false;
      issue(result, "reference line points must be finite and strictly increasing");
      break;
    }
    previous = point.arc_length_m;
  }
  return result;
}

SnapshotValidation validate(const RobotOperatingArea& area) {
  SnapshotValidation result = validate_polygon(area.polygon, "robot operating area");
  if (area.version == 0U || area.id.empty()) {
    result.valid = false;
    issue(result, "robot operating area version or id is missing");
  }
  return result;
}

SnapshotValidation validate(const CableCorridor& corridor) {
  SnapshotValidation result = validate_polygon(corridor.polygon, "cable corridor");
  if (corridor.version == 0U || corridor.id.empty()) {
    result.valid = false;
    issue(result, "cable corridor version or id is missing");
  }
  return result;
}

SnapshotValidation validate(const VersionedPlanningSnapshot& snapshot) {
  SnapshotValidation result = validate(snapshot.map);
  const auto append = [&result](const SnapshotValidation& child) {
    if (!child.valid) result.valid = false;
    result.issues.insert(result.issues.end(), child.issues.begin(), child.issues.end());
  };
  append(validate(snapshot.reference_line));
  append(validate(snapshot.robot_operating_area));
  append(validate(snapshot.cable_corridor));
  if (snapshot.map.version.coordinate_frame != snapshot.reference_line.coordinate_frame) {
    result.valid = false;
    issue(result, "map and reference line coordinate frames differ");
  }
  return result;
}

SnapshotManager::SnapshotManager(const Duration maximum_age)
    : maximum_age_(maximum_age) {
  if (maximum_age_.nanoseconds < 0) {
    throw std::invalid_argument("snapshot maximum age must not be negative");
  }
}

SnapshotUpdateResult SnapshotManager::update(
    const VersionedPlanningSnapshot& snapshot, const MonotonicTime now) {
  const SnapshotValidation checked = validate(snapshot);
  if (!checked.valid) return {SnapshotUpdateStatus::invalid, checked.issues};
  if (now.nanoseconds < 0) {
    return {SnapshotUpdateStatus::invalid, {"current time is invalid"}};
  }
  if (now.nanoseconds < snapshot.map.version.timestamp.nanoseconds) {
    return {SnapshotUpdateStatus::expired, {"snapshot timestamp is in the future"}};
  }
  if (maximum_age_.nanoseconds > 0 &&
      now.nanoseconds - snapshot.map.version.timestamp.nanoseconds >
          maximum_age_.nanoseconds) {
    return {SnapshotUpdateStatus::expired, {"snapshot exceeds maximum age"}};
  }
  if (!latest_) {
    latest_ = snapshot;
    return {SnapshotUpdateStatus::accepted, {}};
  }
  const auto& old = *latest_;
  if (snapshot.map.version == old.map.version &&
      !same_map_payload(snapshot.map, old.map)) {
    return {SnapshotUpdateStatus::version_rollback,
            {"snapshot payload conflicts with installed map version"}};
  }
  if (snapshot.reference_line.version == old.reference_line.version &&
      !same_reference_payload(snapshot.reference_line, old.reference_line)) {
    return {SnapshotUpdateStatus::version_rollback,
            {"snapshot payload conflicts with installed reference-line version"}};
  }
  if (snapshot.robot_operating_area.version == old.robot_operating_area.version &&
      !same_area_payload(snapshot.robot_operating_area, old.robot_operating_area)) {
    return {SnapshotUpdateStatus::version_rollback,
            {"snapshot payload conflicts with installed operating-area version"}};
  }
  if (snapshot.cable_corridor.version == old.cable_corridor.version &&
      !same_corridor_payload(snapshot.cable_corridor, old.cable_corridor)) {
    return {SnapshotUpdateStatus::version_rollback,
            {"snapshot payload conflicts with installed corridor version"}};
  }
  if (snapshot.map.version == old.map.version &&
      snapshot.reference_line.version == old.reference_line.version &&
      snapshot.robot_operating_area.version == old.robot_operating_area.version &&
      snapshot.cable_corridor.version == old.cable_corridor.version) {
    if (same_payload(snapshot, old)) {
      return {SnapshotUpdateStatus::duplicate, {"snapshot version already installed"}};
    }
    return {SnapshotUpdateStatus::version_rollback,
            {"snapshot payload conflicts with installed version"}};
  }
  if (snapshot.map.version.map_id == old.map.version.map_id &&
      snapshot.map.version.sequence_number == old.map.version.sequence_number) {
    return {SnapshotUpdateStatus::version_rollback,
            {"snapshot payload conflicts with installed map version"}};
  }
  if (snapshot.map.version.map_id == old.map.version.map_id &&
      snapshot.map.version.sequence_number < old.map.version.sequence_number) {
    return {SnapshotUpdateStatus::version_rollback, {"map version rolled back"}};
  }
  if (snapshot.reference_line.version < old.reference_line.version ||
      snapshot.robot_operating_area.version < old.robot_operating_area.version ||
      snapshot.cable_corridor.version < old.cable_corridor.version) {
    return {SnapshotUpdateStatus::out_of_order, {"snapshot version is out of order"}};
  }
  latest_ = snapshot;
  return {SnapshotUpdateStatus::accepted, {}};
}

std::optional<VersionedPlanningSnapshot> SnapshotManager::latest() const { return latest_; }

bool SnapshotManager::is_current(const VersionedPlanningSnapshot& snapshot) const {
  return latest_.has_value() && same_payload(snapshot, *latest_);
}

}  // namespace underwater_planner::core
