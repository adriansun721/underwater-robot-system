#include "underwater_planner/core/traversability_evaluator.hpp"

#include "planar_geometry.hpp"
#include "underwater_planner/core/parameter_config.hpp"
#include "underwater_planner/core/step_traversal_rules.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace underwater_planner::core {
namespace {

constexpr double kMaximumSweepSpacingFraction = 0.5;

bool finite(const double value) { return std::isfinite(value); }

struct CovarianceEigenvalues {
  double minimum_m2{};
  double maximum_m2{};
};

struct CovarianceComponents {
  double xx{};
  double xy{};
  double yx{};
  double yy{};
};

CovarianceComponents components(const Covariance2dM2& covariance) {
  return {covariance.xx_m2, covariance.xy_m2, covariance.yx_m2,
          covariance.yy_m2};
}

CovarianceComponents components(const GradientCovariance& covariance) {
  return {covariance.xx, covariance.xy, covariance.yx, covariance.yy};
}

CovarianceEigenvalues covariance_eigenvalues(
    const CovarianceComponents& covariance) {
  const double symmetric_xy =
      0.5 * (covariance.xy + covariance.yx);
  const double half_trace =
      0.5 * (covariance.xx + covariance.yy);
  const double eigen_radius =
      std::hypot(0.5 * (covariance.xx - covariance.yy),
                 symmetric_xy);
  return {half_trace - eigen_radius, half_trace + eigen_radius};
}

double inverse_standard_normal(const double probability) {
  // Acklam's rational approximation keeps the core independent of a stats
  // runtime while supporting the calibrated one-sided collision epsilon.
  constexpr double a1 = -3.969683028665376e+01;
  constexpr double a2 = 2.209460984245205e+02;
  constexpr double a3 = -2.759285104469687e+02;
  constexpr double a4 = 1.383577518672690e+02;
  constexpr double a5 = -3.066479806614716e+01;
  constexpr double a6 = 2.506628277459239e+00;
  constexpr double b1 = -5.447609879822406e+01;
  constexpr double b2 = 1.615858368580409e+02;
  constexpr double b3 = -1.556989798598866e+02;
  constexpr double b4 = 6.680131188771972e+01;
  constexpr double b5 = -1.328068155288572e+01;
  constexpr double c1 = -7.784894002430293e-03;
  constexpr double c2 = -3.223964580411365e-01;
  constexpr double c3 = -2.400758277161838e+00;
  constexpr double c4 = -2.549732539343734e+00;
  constexpr double c5 = 4.374664141464968e+00;
  constexpr double c6 = 2.938163982698783e+00;
  constexpr double d1 = 7.784695709041462e-03;
  constexpr double d2 = 3.224671290700398e-01;
  constexpr double d3 = 2.445134137142996e+00;
  constexpr double d4 = 3.754408661907416e+00;
  constexpr double lower = 0.02425;
  constexpr double upper = 1.0 - lower;

  if (probability < lower) {
    const double q = std::sqrt(-2.0 * std::log(probability));
    return (((((c1 * q + c2) * q + c3) * q + c4) * q + c5) * q + c6) /
           ((((d1 * q + d2) * q + d3) * q + d4) * q + 1.0);
  }
  if (probability <= upper) {
    const double q = probability - 0.5;
    const double r = q * q;
    return (((((a1 * r + a2) * r + a3) * r + a4) * r + a5) * r + a6) *
           q / (((((b1 * r + b2) * r + b3) * r + b4) * r + b5) * r +
                1.0);
  }
  const double q = std::sqrt(-2.0 * std::log(1.0 - probability));
  return -(((((c1 * q + c2) * q + c3) * q + c4) * q + c5) * q + c6) /
         ((((d1 * q + d2) * q + d3) * q + d4) * q + 1.0);
}

bool valid_covariance(const CovarianceComponents& covariance) {
  if (!finite(covariance.xx) || !finite(covariance.xy) ||
      !finite(covariance.yx) || !finite(covariance.yy)) {
    return false;
  }
  const double symmetry_scale =
      std::max({1.0, std::abs(covariance.xy), std::abs(covariance.yx)});
  const double symmetry_tolerance =
      64.0 * std::numeric_limits<double>::epsilon() * symmetry_scale;
  if (std::abs(covariance.xy - covariance.yx) > symmetry_tolerance ||
      covariance.xx < 0.0 || covariance.yy < 0.0) {
    return false;
  }
  const double covariance_bound =
      std::sqrt(covariance.xx) * std::sqrt(covariance.yy);
  return std::max(std::abs(covariance.xy), std::abs(covariance.yx)) <=
         covariance_bound;
}

bool valid_covariance(const Covariance2dM2& covariance) {
  return valid_covariance(components(covariance));
}

bool valid_covariance(const GradientCovariance& covariance) {
  return valid_covariance(components(covariance));
}

double directional_variance(const CovarianceComponents& covariance,
                            const double x, const double y) {
  return x * x * covariance.xx + x * y * (covariance.xy + covariance.yx) +
         y * y * covariance.yy;
}

double directional_variance(const Covariance2dM2& covariance,
                            const Vector2d& direction) {
  const double norm = std::hypot(direction.x, direction.y);
  return directional_variance(components(covariance), direction.x / norm,
                              direction.y / norm);
}

Point2d cell_center(const MapSnapshot& map, const std::size_t row,
                    const std::size_t column) {
  return {map.origin_x_m + (static_cast<double>(column) + 0.5) * map.resolution_m,
          map.origin_y_m + (static_cast<double>(row) + 0.5) * map.resolution_m};
}

bool polygon_intersects_cell(const std::vector<Point2d>& polygon,
                             const double minimum_x,
                             const double minimum_y,
                             const double resolution_m,
                             const double expansion_m) {
  const double expanded_minimum_x = minimum_x - expansion_m;
  const double expanded_minimum_y = minimum_y - expansion_m;
  const double maximum_x = minimum_x + resolution_m + expansion_m;
  const double maximum_y = minimum_y + resolution_m + expansion_m;
  const std::vector<Point2d> corners{{expanded_minimum_x, expanded_minimum_y},
                                     {maximum_x, expanded_minimum_y},
                                     {maximum_x, maximum_y},
                                     {expanded_minimum_x, maximum_y}};
  for (const Point2d& vertex : polygon) {
    if (vertex.x_m >= expanded_minimum_x && vertex.x_m <= maximum_x &&
        vertex.y_m >= expanded_minimum_y && vertex.y_m <= maximum_y) {
      return true;
    }
  }
  for (const Point2d& corner : corners) {
    if (detail::point_covered_by_polygon(corner, polygon)) return true;
  }
  for (std::size_t polygon_index = 0; polygon_index < polygon.size();
       ++polygon_index) {
    const Point2d& polygon_start = polygon[polygon_index];
    const Point2d& polygon_end =
        polygon[(polygon_index + 1U) % polygon.size()];
    for (std::size_t corner_index = 0; corner_index < corners.size();
         ++corner_index) {
      if (detail::segments_intersect(
              polygon_start, polygon_end, corners[corner_index],
              corners[(corner_index + 1U) % corners.size()])) {
        return true;
      }
    }
  }
  return false;
}

std::vector<Point2d> transformed_footprint(const TrackFootprint& footprint,
                                           const Pose2d& pose) {
  const double cosine = std::cos(pose.heading_rad);
  const double sine = std::sin(pose.heading_rad);
  std::vector<Point2d> transformed;
  transformed.reserve(footprint.polygon.size());
  for (const Point2d& point : footprint.polygon) {
    transformed.push_back({pose.x_m + cosine * point.x_m - sine * point.y_m,
                           pose.y_m + sine * point.x_m + cosine * point.y_m});
  }
  return transformed;
}

struct PolygonBounds {
  double minimum_x{};
  double maximum_x{};
  double minimum_y{};
  double maximum_y{};
};

PolygonBounds polygon_bounds(const std::vector<Point2d>& polygon) {
  PolygonBounds bounds{polygon.front().x_m, polygon.front().x_m,
                       polygon.front().y_m, polygon.front().y_m};
  for (const Point2d& point : polygon) {
    bounds.minimum_x = std::min(bounds.minimum_x, point.x_m);
    bounds.maximum_x = std::max(bounds.maximum_x, point.x_m);
    bounds.minimum_y = std::min(bounds.minimum_y, point.y_m);
    bounds.maximum_y = std::max(bounds.maximum_y, point.y_m);
  }
  return bounds;
}

bool valid_footprint(const TrackFootprint& footprint) {
  if (footprint.polygon.size() < 3U) return false;
  double twice_area = 0.0;
  double coordinate_scale = 1.0;
  for (std::size_t index = 0; index < footprint.polygon.size(); ++index) {
    const Point2d& current = footprint.polygon[index];
    const Point2d& next =
        footprint.polygon[(index + 1U) % footprint.polygon.size()];
    if (!finite(current.x_m) || !finite(current.y_m)) return false;
    coordinate_scale =
        std::max({coordinate_scale, std::abs(current.x_m),
                  std::abs(current.y_m)});
    twice_area += current.x_m * next.y_m - current.y_m * next.x_m;
  }
  const double area_tolerance =
      64.0 * std::numeric_limits<double>::epsilon() * coordinate_scale *
      coordinate_scale;
  if (std::abs(twice_area) <= area_tolerance) return false;

  const std::size_t count = footprint.polygon.size();
  for (std::size_t first = 0; first < count; ++first) {
    const std::size_t first_end = (first + 1U) % count;
    for (std::size_t second = first + 1U; second < count; ++second) {
      const std::size_t second_end = (second + 1U) % count;
      if (first == second || first_end == second || second_end == first) {
        continue;
      }
      if (detail::segments_intersect(
              footprint.polygon[first], footprint.polygon[first_end],
              footprint.polygon[second], footprint.polygon[second_end])) {
        return false;
      }
    }
  }
  return true;
}

bool valid_capability(const RobotCapability& capability) {
  const double half_pi = 0.5 * std::acos(-1.0);
  return finite(capability.maximum_slope_up_rad) &&
         capability.maximum_slope_up_rad > 0.0 &&
         capability.maximum_slope_up_rad < half_pi &&
         finite(capability.maximum_slope_down_rad) &&
         capability.maximum_slope_down_rad > 0.0 &&
         capability.maximum_slope_down_rad < half_pi &&
         finite(capability.maximum_slope_lateral_rad) &&
         capability.maximum_slope_lateral_rad > 0.0 &&
         capability.maximum_slope_lateral_rad < half_pi &&
         finite(capability.maximum_support_roll_rad) &&
         capability.maximum_support_roll_rad > 0.0 &&
         capability.maximum_support_roll_rad < half_pi &&
         finite(capability.maximum_step_climb_m) &&
         capability.maximum_step_climb_m > 0.0 &&
         finite(capability.maximum_step_drop_m) &&
         capability.maximum_step_drop_m > 0.0 &&
         finite(capability.minimum_track_support_ratio) &&
         capability.minimum_track_support_ratio > 0.0 &&
         capability.minimum_track_support_ratio <= 1.0 &&
         finite(capability.effective_track_spacing_m) &&
         capability.effective_track_spacing_m > 0.0 &&
         valid_step_alignment_domain(
             capability.minimum_step_crossing_alignment,
             capability.step_alignment_transition_band) &&
         finite(capability.maximum_roughness_m) &&
         capability.maximum_roughness_m >= 0.0;
}

bool valid_track_geometry(const TrackFootprint& geometry) {
  if (!valid_footprint(geometry) ||
      !valid_footprint(TrackFootprint{geometry.left_support_polygon}) ||
      !valid_footprint(TrackFootprint{geometry.right_support_polygon})) {
    return false;
  }
  const auto contained_by_outer =
      [&geometry](const std::vector<Point2d>& track) {
        return std::all_of(
            track.begin(), track.end(), [&geometry](const Point2d& point) {
              return detail::point_covered_by_polygon(point, geometry.polygon);
            });
      };
  if (!contained_by_outer(geometry.left_support_polygon) ||
      !contained_by_outer(geometry.right_support_polygon)) {
    return false;
  }
  if (detail::point_covered_by_polygon(geometry.left_support_polygon.front(),
                                       geometry.right_support_polygon) ||
      detail::point_covered_by_polygon(geometry.right_support_polygon.front(),
                                       geometry.left_support_polygon)) {
    return false;
  }
  for (std::size_t left = 0; left < geometry.left_support_polygon.size();
       ++left) {
    for (std::size_t right = 0; right < geometry.right_support_polygon.size();
         ++right) {
      if (detail::segments_intersect(
              geometry.left_support_polygon[left],
              geometry.left_support_polygon[
                  (left + 1U) % geometry.left_support_polygon.size()],
              geometry.right_support_polygon[right],
              geometry.right_support_polygon[
                  (right + 1U) % geometry.right_support_polygon.size()])) {
        return false;
      }
    }
  }
  return true;
}

bool polygon_intersects_polyline(const std::vector<Point2d>& polygon,
                                 const Polyline2D& polyline) {
  for (std::size_t edge_index = 0; edge_index + 1U < polyline.size();
       ++edge_index) {
    if (detail::point_covered_by_polygon(polyline[edge_index], polygon) ||
        detail::point_covered_by_polygon(polyline[edge_index + 1U], polygon)) {
      return true;
    }
    for (std::size_t polygon_index = 0; polygon_index < polygon.size();
         ++polygon_index) {
      if (detail::segments_intersect(
              polyline[edge_index], polyline[edge_index + 1U],
              polygon[polygon_index],
              polygon[(polygon_index + 1U) % polygon.size()])) {
        return true;
      }
    }
  }
  return false;
}

struct PosePairSweep {
  double heading_delta_rad{};
  double boundary_displacement_m{};
};

PosePairSweep measure_pose_pair_sweep(const Pose2d& start, const Pose2d& end,
                                      double footprint_radius_m);

double polyline_polygon_distance(const Polyline2D& polyline,
                                 const std::vector<Point2d>& polygon) {
  if (polygon_intersects_polyline(polygon, polyline)) return 0.0;
  double minimum_distance = std::numeric_limits<double>::infinity();
  for (std::size_t edge_index = 0; edge_index + 1U < polyline.size();
       ++edge_index) {
    for (std::size_t polygon_index = 0; polygon_index < polygon.size();
         ++polygon_index) {
      minimum_distance = std::min(
          minimum_distance,
          detail::segment_distance(
              polyline[edge_index], polyline[edge_index + 1U],
              polygon[polygon_index],
              polygon[(polygon_index + 1U) % polygon.size()]));
    }
  }
  return minimum_distance;
}

Pose2d midpoint_pose(const Pose2d& start, const Pose2d& end) {
  const double heading_delta =
      std::atan2(std::sin(end.heading_rad - start.heading_rad),
                 std::cos(end.heading_rad - start.heading_rad));
  return {0.5 * (start.x_m + end.x_m), 0.5 * (start.y_m + end.y_m),
          start.heading_rad + 0.5 * heading_delta,
          MonotonicTime{start.timestamp.nanoseconds / 2 +
                        end.timestamp.nanoseconds / 2 +
                        (start.timestamp.nanoseconds % 2 +
                         end.timestamp.nanoseconds % 2) /
                            2}};
}

std::optional<Pose2d> continuous_step_contact(
    const StepEdge& edge, const Pose2d& start, const Pose2d& end,
    const TrackFootprint& geometry, const double footprint_radius_m,
    const std::size_t depth) {
  const std::vector<Point2d> start_polygon =
      transformed_footprint(geometry, start);
  if (polygon_intersects_polyline(start_polygon, edge.extent)) return start;
  const std::vector<Point2d> end_polygon = transformed_footprint(geometry, end);
  if (polygon_intersects_polyline(end_polygon, edge.extent)) return end;

  const double displacement =
      measure_pose_pair_sweep(start, end, footprint_radius_m)
          .boundary_displacement_m;
  const double endpoint_distance =
      std::min(polyline_polygon_distance(edge.extent, start_polygon),
               polyline_polygon_distance(edge.extent, end_polygon));
  const double coordinate_scale =
      std::max({1.0, std::abs(start.x_m), std::abs(start.y_m),
                std::abs(end.x_m), std::abs(end.y_m)});
  const double contact_tolerance =
      256.0 * std::numeric_limits<double>::epsilon() * coordinate_scale;
  if (endpoint_distance > displacement + contact_tolerance) {
    return std::nullopt;
  }

  const Pose2d midpoint = midpoint_pose(start, end);
  const std::vector<Point2d> midpoint_polygon =
      transformed_footprint(geometry, midpoint);
  if (polygon_intersects_polyline(midpoint_polygon, edge.extent) ||
      (depth >= 60U &&
       polyline_polygon_distance(edge.extent, midpoint_polygon) <=
           contact_tolerance)) {
    return midpoint;
  }
  if (depth >= 60U || displacement <= contact_tolerance) {
    return std::nullopt;
  }
  if (const std::optional<Pose2d> first = continuous_step_contact(
          edge, start, midpoint, geometry, footprint_radius_m, depth + 1U)) {
    return first;
  }
  return continuous_step_contact(edge, midpoint, end, geometry,
                                 footprint_radius_m, depth + 1U);
}

std::optional<Pose2d> find_step_sweep_contact(
    const StepEdge& edge, const std::vector<Pose2d>& sweep,
    const TrackFootprint& geometry, const std::size_t begin_index,
    const std::size_t end_index) {
  if (begin_index == end_index) {
    return polygon_intersects_polyline(
               transformed_footprint(geometry, sweep[begin_index]), edge.extent)
               ? std::optional<Pose2d>{sweep[begin_index]}
               : std::nullopt;
  }
  const double radius = track_footprint_radius(geometry);
  for (std::size_t index = begin_index; index < end_index; ++index) {
    if (const std::optional<Pose2d> contact = continuous_step_contact(
            edge, sweep[index], sweep[index + 1U], geometry, radius, 0U)) {
      return contact;
    }
  }
  return std::nullopt;
}

std::optional<Pose2d> find_step_sweep_contact(
    const StepEdge& edge, const std::vector<Pose2d>& sweep,
    const TrackFootprint& geometry) {
  return find_step_sweep_contact(edge, sweep, geometry, 0U, sweep.size() - 1U);
}

bool step_intersects_sweep(const StepEdge& edge,
                           const std::vector<Pose2d>& sweep,
                           const TrackFootprint& geometry) {
  return find_step_sweep_contact(edge, sweep, geometry).has_value();
}

bool discontinuity_is_explained_by_crossed_step(
    const Point2d& cell_center_point, const TerrainLayers& terrain,
    const std::vector<Pose2d>& sweep, const TrackFootprint& geometry) {
  const double radius =
      0.5 * terrain.surface_fit_window_size_m +
      std::sqrt(0.5) * terrain.surface.resolution_m;
  for (const StepEstimate& estimate : terrain.steps.estimates) {
    if (estimate.status != StepEstimateStatus::valid ||
        !step_intersects_sweep(estimate.edge, sweep, geometry)) {
      continue;
    }
    for (std::size_t index = 0; index + 1U < estimate.edge.extent.size();
         ++index) {
      if (detail::point_to_segment_distance(
              cell_center_point, estimate.edge.extent[index],
              estimate.edge.extent[index + 1U]) <=
          radius) {
        return true;
      }
    }
  }
  return false;
}

enum class StepSide { low, high, spanning };

struct StepContactInterval {
  Pose2d contact_pose;
  std::optional<StepSide> start_side;
  std::optional<StepSide> end_side;
};

void add_limiting_factor(TraversabilityResult& result,
                         TraversabilityLimitingFactor factor);

StepSide footprint_step_side(const std::vector<Point2d>& footprint,
                             const StepEdge& edge) {
  const Point2d& origin = edge.extent.front();
  const double half_transition_width = 0.5 * edge.transition_width_m;
  double minimum_signed_distance = std::numeric_limits<double>::infinity();
  double maximum_signed_distance = -std::numeric_limits<double>::infinity();
  for (const Point2d& point : footprint) {
    const double signed_distance =
        (point.x_m - origin.x_m) * edge.normal_low_to_high.x +
        (point.y_m - origin.y_m) * edge.normal_low_to_high.y;
    minimum_signed_distance = std::min(minimum_signed_distance, signed_distance);
    maximum_signed_distance = std::max(maximum_signed_distance, signed_distance);
  }
  if (maximum_signed_distance < -half_transition_width) return StepSide::low;
  if (minimum_signed_distance > half_transition_width) return StepSide::high;
  return StepSide::spanning;
}

std::vector<StepContactInterval> find_step_contact_intervals(
    const StepEdge& edge, const std::vector<Pose2d>& sweep,
    const TrackFootprint& geometry) {
  std::vector<StepSide> sides;
  sides.reserve(sweep.size());
  std::vector<std::size_t> definite_indices;
  for (std::size_t index = 0; index < sweep.size(); ++index) {
    const StepSide side = footprint_step_side(
        transformed_footprint(geometry, sweep[index]), edge);
    sides.push_back(side);
    if (side != StepSide::spanning) definite_indices.push_back(index);
  }

  std::vector<StepContactInterval> intervals;
  const auto append_interval =
      [&edge, &sweep, &geometry, &intervals](
          const std::size_t begin_index, const std::size_t end_index,
          const std::optional<StepSide> start_side,
          const std::optional<StepSide> end_side) {
        if (const std::optional<Pose2d> contact = find_step_sweep_contact(
                edge, sweep, geometry, begin_index, end_index)) {
          intervals.push_back({*contact, start_side, end_side});
        }
      };

  if (definite_indices.empty()) {
    append_interval(0U, sweep.size() - 1U, std::nullopt, std::nullopt);
    return intervals;
  }

  const std::size_t first_definite = definite_indices.front();
  if (first_definite > 0U) {
    append_interval(0U, first_definite, std::nullopt, sides[first_definite]);
  }
  for (std::size_t index = 0; index + 1U < definite_indices.size(); ++index) {
    const std::size_t begin_index = definite_indices[index];
    const std::size_t end_index = definite_indices[index + 1U];
    append_interval(begin_index, end_index, sides[begin_index],
                    sides[end_index]);
  }
  const std::size_t last_definite = definite_indices.back();
  if (last_definite + 1U < sweep.size()) {
    append_interval(last_definite, sweep.size() - 1U, sides[last_definite],
                    std::nullopt);
  }
  return intervals;
}

int diagnostic_priority(const StepCrossingType type) {
  switch (type) {
    case StepCrossingType::transition:
      return 4;
    case StepCrossingType::edge_riding:
      return 3;
    case StepCrossingType::climb:
      return 2;
    case StepCrossingType::drop:
      return 1;
    case StepCrossingType::none:
      return 0;
  }
  return 0;
}

StepCrossingEvent make_step_crossing_event(
    const StepContactInterval& interval, const StepEdge& edge,
    const RobotCapability& capability) {
  StepCrossingEvent event;
  event.complete_height_m = edge.height_m;
  event.contact_pose = interval.contact_pose;
  if (interval.start_side == StepSide::low &&
      interval.end_side == StepSide::high) {
    event.direction = StepCrossingDirection::low_to_high;
  } else if (interval.start_side == StepSide::high &&
             interval.end_side == StepSide::low) {
    event.direction = StepCrossingDirection::high_to_low;
  }

  const double forward_x = std::cos(interval.contact_pose.heading_rad);
  const double forward_y = std::sin(interval.contact_pose.heading_rad);
  const double absolute_alignment = std::abs(
      forward_x * edge.normal_low_to_high.x +
      forward_y * edge.normal_low_to_high.y);
  const double transition_minimum =
      capability.minimum_step_crossing_alignment -
      capability.step_alignment_transition_band;
  const double transition_maximum =
      capability.minimum_step_crossing_alignment +
      capability.step_alignment_transition_band;
  if (absolute_alignment >= transition_minimum &&
      absolute_alignment <= transition_maximum) {
    event.type = StepCrossingType::transition;
  } else if (absolute_alignment < transition_minimum ||
             event.direction == StepCrossingDirection::none) {
    event.type = StepCrossingType::edge_riding;
  } else if (event.direction == StepCrossingDirection::low_to_high) {
    event.type = StepCrossingType::climb;
  } else {
    event.type = StepCrossingType::drop;
  }
  return event;
}

void record_step_crossing_event(const StepCrossingEvent& event,
                                const RobotCapability& capability,
                                TraversabilityResult& result) {
  result.step_crossing_events.push_back(event);
  if (event.direction == StepCrossingDirection::low_to_high &&
      event.complete_height_m > capability.maximum_step_climb_m) {
    add_limiting_factor(result,
                        TraversabilityLimitingFactor::step_climb_exceeded);
  } else if (event.direction == StepCrossingDirection::high_to_low &&
             event.complete_height_m > capability.maximum_step_drop_m) {
    add_limiting_factor(result,
                        TraversabilityLimitingFactor::step_drop_exceeded);
  } else if (event.direction == StepCrossingDirection::none &&
             event.complete_height_m >
                 std::min(capability.maximum_step_climb_m,
                          capability.maximum_step_drop_m)) {
    add_limiting_factor(
        result,
        TraversabilityLimitingFactor::step_transition_height_exceeded);
  }

  if (event.complete_height_m > result.maximum_complete_step_height_m ||
      (event.complete_height_m == result.maximum_complete_step_height_m &&
       diagnostic_priority(event.type) >
           diagnostic_priority(result.step_crossing_type))) {
    result.maximum_complete_step_height_m = event.complete_height_m;
    result.step_crossing_type = event.type;
  }
}

void evaluate_step_crossings(const std::vector<Pose2d>& sweep,
                             const TrackFootprint& geometry,
                             const RobotCapability& capability,
                             const StepLayer& steps,
                             TraversabilityResult& result) {
  for (const StepEstimate& estimate : steps.estimates) {
    if (estimate.status != StepEstimateStatus::valid) continue;
    const StepEdge& edge = estimate.edge;
    for (const StepContactInterval& interval :
         find_step_contact_intervals(edge, sweep, geometry)) {
      record_step_crossing_event(
          make_step_crossing_event(interval, edge, capability), capability,
          result);
    }
  }
  std::sort(result.step_crossing_events.begin(),
            result.step_crossing_events.end(),
            [](const StepCrossingEvent& left,
               const StepCrossingEvent& right) {
              if (left.contact_pose.timestamp.nanoseconds !=
                  right.contact_pose.timestamp.nanoseconds) {
                return left.contact_pose.timestamp.nanoseconds <
                       right.contact_pose.timestamp.nanoseconds;
              }
              if (left.contact_pose.x_m != right.contact_pose.x_m) {
                return left.contact_pose.x_m < right.contact_pose.x_m;
              }
              if (left.contact_pose.y_m != right.contact_pose.y_m) {
                return left.contact_pose.y_m < right.contact_pose.y_m;
              }
              if (left.contact_pose.heading_rad !=
                  right.contact_pose.heading_rad) {
                return left.contact_pose.heading_rad <
                       right.contact_pose.heading_rad;
              }
              if (left.complete_height_m != right.complete_height_m) {
                return left.complete_height_m > right.complete_height_m;
              }
              if (left.type != right.type) {
                return static_cast<int>(left.type) <
                       static_cast<int>(right.type);
              }
              return static_cast<int>(left.direction) <
                     static_cast<int>(right.direction);
            });
}

struct WeightedElevation {
  double elevation_m{};
  double weight{};
};

struct TrackSupportStatistics {
  double coverage_ratio{};
  double robust_elevation_m{};
  double local_drop_m{};
  bool has_elevation{};
  bool has_outlier{};
};

double weighted_quantile(const std::vector<WeightedElevation>& samples,
                         const double probability,
                         const double total_weight) {
  const double target = probability * total_weight;
  double cumulative_weight = 0.0;
  for (const WeightedElevation& sample : samples) {
    cumulative_weight += sample.weight;
    if (cumulative_weight >= target) return sample.elevation_m;
  }
  return samples.back().elevation_m;
}

TrackSupportStatistics evaluate_track_support(
    const std::vector<Point2d>& support_polygon, const Pose2d& pose,
    const SurfaceLayer& surface) {
  const std::vector<Point2d> transformed = transformed_footprint(
      TrackFootprint{support_polygon}, pose);
  const PolygonBounds bounds = polygon_bounds(transformed);
  const std::size_t minimum_column = static_cast<std::size_t>(std::max(
      0.0, std::floor((bounds.minimum_x - surface.origin_x_m) /
                      surface.resolution_m)));
  const std::size_t maximum_column = std::min(
      surface.width - 1U,
      static_cast<std::size_t>(std::floor(
          (bounds.maximum_x - surface.origin_x_m) / surface.resolution_m)));
  const std::size_t minimum_row = static_cast<std::size_t>(std::max(
      0.0, std::floor((bounds.minimum_y - surface.origin_y_m) /
                      surface.resolution_m)));
  const std::size_t maximum_row = std::min(
      surface.height - 1U,
      static_cast<std::size_t>(std::floor(
          (bounds.maximum_y - surface.origin_y_m) / surface.resolution_m)));

  std::size_t expected_samples = 0U;
  double supported_weight = 0.0;
  std::vector<WeightedElevation> elevations;
  for (std::size_t row = minimum_row; row <= maximum_row; ++row) {
    for (std::size_t column = minimum_column; column <= maximum_column;
         ++column) {
      const double cell_x =
          surface.origin_x_m + static_cast<double>(column) * surface.resolution_m;
      const double cell_y =
          surface.origin_y_m + static_cast<double>(row) * surface.resolution_m;
      if (!polygon_intersects_cell(transformed, cell_x, cell_y,
                                   surface.resolution_m, 0.0)) {
        continue;
      }
      ++expected_samples;
      const SurfaceEstimate& estimate = surface.at(row, column);
      if ((estimate.status != TerrainEstimateStatus::valid &&
           estimate.status != TerrainEstimateStatus::discontinuous) ||
          !finite(estimate.elevation_m) || !finite(estimate.support_ratio) ||
          estimate.support_ratio <= 0.0) {
        continue;
      }
      const double weight = std::min(1.0, estimate.support_ratio);
      supported_weight += weight;
      elevations.push_back({estimate.elevation_m, weight});
    }
  }

  TrackSupportStatistics statistics;
  statistics.coverage_ratio =
      expected_samples == 0U
          ? 0.0
          : supported_weight / static_cast<double>(expected_samples);
  if (elevations.empty()) return statistics;
  std::sort(elevations.begin(), elevations.end(),
            [](const WeightedElevation& left, const WeightedElevation& right) {
              return left.elevation_m < right.elevation_m;
            });
  statistics.robust_elevation_m =
      weighted_quantile(elevations, 0.5, supported_weight);
  const double lower_quartile =
      weighted_quantile(elevations, 0.25, supported_weight);
  const double upper_quartile =
      weighted_quantile(elevations, 0.75, supported_weight);
  const double interquartile_range = upper_quartile - lower_quartile;
  const double lower_fence = lower_quartile - 1.5 * interquartile_range;
  const double upper_fence = upper_quartile + 1.5 * interquartile_range;
  const double outlier_tolerance =
      64.0 * std::numeric_limits<double>::epsilon() *
      std::max({1.0, std::abs(lower_fence), std::abs(upper_fence)});
  statistics.has_outlier = std::any_of(
      elevations.begin(), elevations.end(),
      [lower_fence, upper_fence, outlier_tolerance](
          const WeightedElevation& sample) {
        return sample.elevation_m < lower_fence - outlier_tolerance ||
               sample.elevation_m > upper_fence + outlier_tolerance;
      });
  statistics.local_drop_m =
      elevations.back().elevation_m - elevations.front().elevation_m;
  statistics.has_elevation = true;
  return statistics;
}

void evaluate_track_support_at_pose(const Pose2d& pose,
                                    const SurfaceLayer& surface,
                                    const TrackFootprint& geometry,
                                    const RobotCapability& capability,
                                    TraversabilityResult& result) {
  const TrackSupportStatistics left = evaluate_track_support(
      geometry.left_support_polygon, pose, surface);
  const TrackSupportStatistics right = evaluate_track_support(
      geometry.right_support_polygon, pose, surface);
  result.minimum_left_track_support_ratio =
      std::min(result.minimum_left_track_support_ratio, left.coverage_ratio);
  result.minimum_right_track_support_ratio =
      std::min(result.minimum_right_track_support_ratio, right.coverage_ratio);
  result.maximum_local_track_drop_m =
      std::max({result.maximum_local_track_drop_m, left.local_drop_m,
                right.local_drop_m});
  if (left.coverage_ratio < capability.minimum_track_support_ratio) {
    add_limiting_factor(
        result,
        TraversabilityLimitingFactor::left_track_support_insufficient);
  }
  if (right.coverage_ratio < capability.minimum_track_support_ratio) {
    add_limiting_factor(
        result,
        TraversabilityLimitingFactor::right_track_support_insufficient);
  }
  if (left.local_drop_m > capability.maximum_step_drop_m ||
      right.local_drop_m > capability.maximum_step_drop_m) {
    add_limiting_factor(
        result, TraversabilityLimitingFactor::local_track_drop_exceeded);
  }
  if (left.has_outlier || right.has_outlier) {
    result.track_elevation_outlier_detected = true;
    add_limiting_factor(
        result,
        TraversabilityLimitingFactor::track_elevation_outlier_detected);
  }
  if (left.has_elevation && right.has_elevation) {
    const double roll = std::atan(
        (left.robust_elevation_m - right.robust_elevation_m) /
        capability.effective_track_spacing_m);
    result.maximum_absolute_support_roll_rad =
        std::max(result.maximum_absolute_support_roll_rad, std::abs(roll));
    if (std::abs(roll) > capability.maximum_support_roll_rad) {
      add_limiting_factor(
          result, TraversabilityLimitingFactor::support_roll_exceeded);
    }
  }
}

bool valid_motion_segment(const MotionSegment& segment) {
  if (segment.samples.empty()) return false;
  std::int64_t previous_timestamp = -1;
  for (const Pose2d& pose : segment.samples) {
    if (!finite(pose.x_m) || !finite(pose.y_m) ||
        !finite(pose.heading_rad) || pose.timestamp.nanoseconds < 0 ||
        pose.timestamp.nanoseconds < previous_timestamp) {
      return false;
    }
    previous_timestamp = pose.timestamp.nanoseconds;
  }
  return true;
}

bool valid_surface_layer(const SurfaceLayer& surface) {
  if (surface.width == 0U || surface.height == 0U ||
      surface.width > std::numeric_limits<std::size_t>::max() /
                          surface.height ||
      surface.cells.size() != surface.width * surface.height ||
      !finite(surface.resolution_m) || surface.resolution_m <= 0.0 ||
      !finite(surface.origin_x_m) || !finite(surface.origin_y_m)) {
    return false;
  }
  return finite(surface.origin_x_m +
                static_cast<double>(surface.width) * surface.resolution_m) &&
         finite(surface.origin_y_m +
                static_cast<double>(surface.height) * surface.resolution_m);
}

bool recognized_step_status(const StepEstimateStatus status) {
  switch (status) {
    case StepEstimateStatus::valid:
    case StepEstimateStatus::insufficient_side_support:
    case StepEstimateStatus::noise_not_significant:
    case StepEstimateStatus::below_minimum_height:
    case StepEstimateStatus::unstable_normal:
    case StepEstimateStatus::insufficient_extent:
    case StepEstimateStatus::duplicate_extent_point:
    case StepEstimateStatus::low_confidence:
      return true;
  }
  return false;
}

bool valid_step_layer(const StepLayer& steps) {
  for (const StepEstimate& estimate : steps.estimates) {
    if (!recognized_step_status(estimate.status)) return false;
    if (estimate.status != StepEstimateStatus::valid) continue;
    const StepEdge& edge = estimate.edge;
    const double normal_norm =
        std::hypot(edge.normal_low_to_high.x, edge.normal_low_to_high.y);
    if (edge.extent.size() < 2U || !finite(normal_norm) ||
        std::abs(normal_norm - 1.0) > 1.0e-9 || !finite(edge.height_m) ||
        edge.height_m <= 0.0 || !finite(edge.transition_width_m) ||
        edge.transition_width_m < 0.0 || !finite(edge.confidence) ||
        edge.confidence <= 0.0 || edge.confidence > 1.0) {
      return false;
    }
    for (std::size_t index = 0; index < edge.extent.size(); ++index) {
      const Point2d& point = edge.extent[index];
      if (!finite(point.x_m) || !finite(point.y_m)) return false;
      for (std::size_t previous = 0; previous < index; ++previous) {
        if (point.x_m == edge.extent[previous].x_m &&
            point.y_m == edge.extent[previous].y_m) {
          return false;
        }
      }
    }
  }
  return true;
}

bool valid_map_version(const MapVersion& version) {
  return !version.map_id.empty() && version.sequence_number > 0U &&
         version.timestamp.nanoseconds >= 0 &&
         !version.coordinate_frame.empty();
}

bool recognized_coverage_model(const GradientCoverageModel model) {
  switch (model) {
    case GradientCoverageModel::calibrated_gaussian:
    case GradientCoverageModel::empirical_bounded:
    case GradientCoverageModel::deterministic_bounded:
      return true;
    case GradientCoverageModel::unspecified:
      return false;
  }
  return false;
}

PosePairSweep measure_pose_pair_sweep(const Pose2d& start, const Pose2d& end,
                                      const double footprint_radius_m) {
  const double translation =
      std::hypot(end.x_m - start.x_m, end.y_m - start.y_m);
  const double heading_delta =
      std::atan2(std::sin(end.heading_rad - start.heading_rad),
                 std::cos(end.heading_rad - start.heading_rad));
  return {heading_delta,
          translation + footprint_radius_m * std::abs(heading_delta)};
}

std::vector<Pose2d> adaptive_sweep_poses(const MotionSegment& segment,
                                         const TrackFootprint& footprint,
                                         const double map_resolution_m,
                                         const double maximum_spacing_fraction =
                                             kMaximumSweepSpacingFraction) {
  std::vector<Pose2d> sweep;
  if (segment.samples.size() == 1U) {
    sweep.push_back(segment.samples.front());
    return sweep;
  }
  const double radius = track_footprint_radius(footprint);
  for (std::size_t pair_index = 0;
       pair_index + 1U < segment.samples.size(); ++pair_index) {
    const Pose2d& start = segment.samples[pair_index];
    const Pose2d& end = segment.samples[pair_index + 1U];
    const PosePairSweep pair_sweep =
        measure_pose_pair_sweep(start, end, radius);
    const double raw_intervals =
        std::ceil(pair_sweep.boundary_displacement_m /
                   (maximum_spacing_fraction * map_resolution_m));
    const std::size_t intervals = static_cast<std::size_t>(
        std::max(1.0, raw_intervals));
    const std::size_t first_interval = pair_index == 0U ? 0U : 1U;
    for (std::size_t interval = first_interval; interval <= intervals;
         ++interval) {
      const double fraction = static_cast<double>(interval) /
                              static_cast<double>(intervals);
      const long double timestamp =
          static_cast<long double>(start.timestamp.nanoseconds) +
          static_cast<long double>(end.timestamp.nanoseconds -
                                   start.timestamp.nanoseconds) *
              fraction;
      sweep.push_back(
          {start.x_m + fraction * (end.x_m - start.x_m),
           start.y_m + fraction * (end.y_m - start.y_m),
           start.heading_rad + fraction * pair_sweep.heading_delta_rad,
           MonotonicTime{static_cast<std::int64_t>(std::llround(timestamp))}});
    }
  }
  return sweep;
}

double slope_sweep_margin(const std::vector<Pose2d>& sweep,
                          const TrackFootprint& footprint) {
  if (sweep.size() < 2U) return 0.0;
  const double radius = track_footprint_radius(footprint);
  double maximum_step_displacement = 0.0;
  for (std::size_t index = 0; index + 1U < sweep.size(); ++index) {
    const Pose2d& start = sweep[index];
    const Pose2d& end = sweep[index + 1U];
    const PosePairSweep pair_sweep =
        measure_pose_pair_sweep(start, end, radius);
    maximum_step_displacement =
        std::max(maximum_step_displacement,
                 pair_sweep.boundary_displacement_m);
  }
  return 0.5 * maximum_step_displacement;
}

void add_limiting_factor(TraversabilityResult& result,
                         const TraversabilityLimitingFactor factor) {
  if (std::find(result.limiting_factors.begin(),
                result.limiting_factors.end(),
                factor) == result.limiting_factors.end()) {
    result.limiting_factors.push_back(factor);
  }
  result.traversable = false;
}

}  // namespace

std::optional<RobotCapability> make_robot_capability(
    const RobotParameterConfig& parameters) {
  const auto present_and_finite = [](const std::optional<double>& value) {
    return value.has_value() && std::isfinite(*value);
  };
  if (!present_and_finite(parameters.maximum_slope_up_rad) ||
      !present_and_finite(parameters.maximum_slope_down_rad) ||
      !present_and_finite(parameters.maximum_slope_lateral_rad) ||
      !present_and_finite(parameters.maximum_support_roll_rad) ||
      !present_and_finite(parameters.maximum_step_climb_m) ||
      !present_and_finite(parameters.maximum_step_drop_m) ||
      !present_and_finite(parameters.minimum_track_support_ratio) ||
      !present_and_finite(parameters.effective_track_spacing_m) ||
      !present_and_finite(parameters.minimum_step_crossing_alignment) ||
      !present_and_finite(parameters.step_alignment_transition_band) ||
      !present_and_finite(parameters.maximum_roughness_m)) {
    return std::nullopt;
  }
  const RobotCapability capability{
      *parameters.maximum_slope_up_rad,
      *parameters.maximum_slope_down_rad,
      *parameters.maximum_slope_lateral_rad,
      *parameters.maximum_support_roll_rad,
      *parameters.maximum_step_climb_m,
      *parameters.maximum_step_drop_m,
      *parameters.minimum_track_support_ratio,
      *parameters.effective_track_spacing_m,
      *parameters.minimum_step_crossing_alignment,
      *parameters.step_alignment_transition_band,
      *parameters.maximum_roughness_m};
  return valid_capability(capability) ? std::optional<RobotCapability>{capability}
                                      : std::nullopt;
}

double track_footprint_radius(const TrackFootprint& footprint) noexcept {
  double radius_m = 0.0;
  for (const Point2d& point : footprint.polygon) {
    radius_m = std::max(radius_m, std::hypot(point.x_m, point.y_m));
  }
  return radius_m;
}

TerrainGradientRiskAudit make_terrain_gradient_risk_audit(
    const TerrainLayers& terrain,
    const TerrainGradientRiskPolicy& gradient_risk_policy) {
  return {terrain.source_map_version,
          gradient_risk_policy.version,
          terrain.analysis_config_version,
          gradient_risk_policy.terrain_analysis_config_version,
          gradient_risk_policy.epsilon_local,
          gradient_risk_policy.coverage_multiplier,
          gradient_risk_policy.coverage_model,
          gradient_risk_policy.calibration_dataset_id,
          gradient_risk_policy.operating_domain_id,
          false,
          TerrainGradientRiskSemantics::
              local_pointwise_only_no_path_joint_guarantee};
}

const CollisionCellResult& CollisionLayerResult::at(
    const std::size_t row, const std::size_t column) const {
  if (row >= height || column >= width || cells.size() != width * height) {
    throw std::out_of_range("collision layer cell is outside the map");
  }
  return cells.at(row * width + column);
}

TraversabilityEvaluator::TraversabilityEvaluator(
    const RobotCapability capability, TrackFootprint track_footprint)
    : capability_(capability), track_footprint_(std::move(track_footprint)) {}

TraversabilityResult TraversabilityEvaluator::evaluate(
    const MotionSegment& segment, const TerrainLayers& terrain,
    const TerrainGradientRiskPolicy& gradient_risk_policy) const {
  TraversabilityResult result;
  result.risk_audit =
      make_terrain_gradient_risk_audit(terrain, gradient_risk_policy);
  bool gaussian_multiplier_consistent = true;
  if (gradient_risk_policy.coverage_model ==
          GradientCoverageModel::calibrated_gaussian &&
      finite(gradient_risk_policy.epsilon_local) &&
      gradient_risk_policy.epsilon_local > 0.0 &&
      gradient_risk_policy.epsilon_local < 1.0) {
    const double expected_multiplier =
        std::sqrt(-2.0 * std::log(gradient_risk_policy.epsilon_local));
    const double multiplier_scale =
        std::max({1.0, expected_multiplier,
                  std::abs(gradient_risk_policy.coverage_multiplier)});
    gaussian_multiplier_consistent =
        std::abs(gradient_risk_policy.coverage_multiplier -
                 expected_multiplier) <=
        64.0 * std::numeric_limits<double>::epsilon() * multiplier_scale;
  }
  if (gradient_risk_policy.version == 0U ||
      gradient_risk_policy.terrain_analysis_config_version == 0U ||
      !finite(gradient_risk_policy.epsilon_local) ||
      gradient_risk_policy.epsilon_local <= 0.0 ||
      gradient_risk_policy.epsilon_local >= 1.0 ||
      !finite(gradient_risk_policy.coverage_multiplier) ||
      gradient_risk_policy.coverage_multiplier <= 0.0 ||
      !recognized_coverage_model(gradient_risk_policy.coverage_model) ||
      gradient_risk_policy.calibration_dataset_id.empty() ||
      gradient_risk_policy.operating_domain_id.empty() ||
      !gradient_risk_policy.coverage_calibrated ||
      !gaussian_multiplier_consistent) {
    result.validity = TraversabilityEvaluationValidity::risk_policy_invalid;
    result.issues.emplace_back(
        "terrain gradient risk policy is incomplete or uncalibrated");
    return result;
  }
  if (terrain.analysis_config_version !=
          gradient_risk_policy.terrain_analysis_config_version ||
      terrain.operating_domain_id != gradient_risk_policy.operating_domain_id) {
    result.validity = TraversabilityEvaluationValidity::version_mismatch;
    result.issues.emplace_back(
        "terrain analysis and gradient risk policy dependencies differ");
    return result;
  }
  if (!valid_capability(capability_) || !valid_track_geometry(track_footprint_) ||
      !valid_motion_segment(segment) ||
      !valid_surface_layer(terrain.surface) ||
      !finite(terrain.surface_fit_window_size_m) ||
      terrain.surface_fit_window_size_m < 0.0 ||
      !valid_step_layer(terrain.steps) ||
      !valid_map_version(terrain.source_map_version)) {
    result.validity = TraversabilityEvaluationValidity::input_invalid;
    result.issues.emplace_back(
        "robot capability, footprint, motion segment, or terrain grid is invalid");
    return result;
  }
  result.validity = TraversabilityEvaluationValidity::valid;
  result.traversable = true;
  bool has_valid_surface_sample = false;
  const std::vector<Pose2d> sweep =
      adaptive_sweep_poses(segment, track_footprint_,
                           terrain.surface.resolution_m);
  result.evaluated_sweep_poses = sweep.size();
  result.slope_sweep_discretization_margin_m =
      slope_sweep_margin(sweep, track_footprint_);
  evaluate_step_crossings(sweep, track_footprint_, capability_, terrain.steps,
                          result);
  for (const Pose2d& pose : sweep) {
    const std::vector<Point2d> footprint =
        transformed_footprint(track_footprint_, pose);
    const double terrain_minimum_x = terrain.surface.origin_x_m;
    const double terrain_minimum_y = terrain.surface.origin_y_m;
    const double terrain_maximum_x =
        terrain_minimum_x + static_cast<double>(terrain.surface.width) *
                                terrain.surface.resolution_m;
    const double terrain_maximum_y =
        terrain_minimum_y + static_cast<double>(terrain.surface.height) *
                                terrain.surface.resolution_m;
    if (std::any_of(footprint.begin(), footprint.end(),
                    [terrain_minimum_x, terrain_minimum_y, terrain_maximum_x,
                     terrain_maximum_y](const Point2d& point) {
                      return point.x_m < terrain_minimum_x ||
                             point.x_m > terrain_maximum_x ||
                             point.y_m < terrain_minimum_y ||
                             point.y_m > terrain_maximum_y;
                    })) {
      result.validity = TraversabilityEvaluationValidity::terrain_invalid;
      add_limiting_factor(
          result, TraversabilityLimitingFactor::footprint_outside_terrain);
      result.issues.emplace_back(
          "transformed footprint extends outside the terrain grid");
      return result;
    }
    const double forward_x = std::cos(pose.heading_rad);
    const double forward_y = std::sin(pose.heading_rad);
    const double lateral_x = -forward_y;
    const double lateral_y = forward_x;
    const PolygonBounds footprint_bounds = polygon_bounds(footprint);
    const double sweep_margin =
        result.slope_sweep_discretization_margin_m;
    if (footprint_bounds.minimum_x - sweep_margin < terrain_minimum_x ||
        footprint_bounds.maximum_x + sweep_margin > terrain_maximum_x ||
        footprint_bounds.minimum_y - sweep_margin < terrain_minimum_y ||
        footprint_bounds.maximum_y + sweep_margin > terrain_maximum_y) {
      result.validity = TraversabilityEvaluationValidity::terrain_invalid;
      add_limiting_factor(
          result, TraversabilityLimitingFactor::footprint_outside_terrain);
      result.issues.emplace_back(
          "slope sweep margin extends outside the terrain grid");
      return result;
    }
    const auto minimum_column = static_cast<std::size_t>(std::floor(
        (footprint_bounds.minimum_x - sweep_margin -
         terrain.surface.origin_x_m) /
        terrain.surface.resolution_m));
    const auto maximum_column = std::min(
        terrain.surface.width - 1U,
        static_cast<std::size_t>(std::floor(
            (footprint_bounds.maximum_x + sweep_margin -
             terrain.surface.origin_x_m) /
            terrain.surface.resolution_m)));
    const auto minimum_row = static_cast<std::size_t>(std::floor(
        (footprint_bounds.minimum_y - sweep_margin -
         terrain.surface.origin_y_m) /
        terrain.surface.resolution_m));
    const auto maximum_row = std::min(
        terrain.surface.height - 1U,
        static_cast<std::size_t>(std::floor(
            (footprint_bounds.maximum_y + sweep_margin -
             terrain.surface.origin_y_m) /
            terrain.surface.resolution_m)));
    for (std::size_t row = minimum_row; row <= maximum_row; ++row) {
      for (std::size_t column = minimum_column; column <= maximum_column;
           ++column) {
        const double cell_x = terrain.surface.origin_x_m +
                              static_cast<double>(column) *
                                  terrain.surface.resolution_m;
        const double cell_y = terrain.surface.origin_y_m +
                              static_cast<double>(row) *
                                  terrain.surface.resolution_m;
        if (!polygon_intersects_cell(footprint, cell_x, cell_y,
                                     terrain.surface.resolution_m,
                                     sweep_margin)) {
          continue;
        }
        ++result.evaluated_footprint_samples;
        const SurfaceEstimate& surface = terrain.surface.at(row, column);
        if (surface.status == TerrainEstimateStatus::discontinuous &&
            finite(surface.elevation_m) &&
            discontinuity_is_explained_by_crossed_step(
                {cell_x + 0.5 * terrain.surface.resolution_m,
                 cell_y + 0.5 * terrain.surface.resolution_m},
                terrain, sweep, track_footprint_)) {
          result.worst_terrain_estimate_status =
              TerrainEstimateStatus::discontinuous;
          continue;
        }
        if (surface.status != TerrainEstimateStatus::valid ||
            !finite(surface.gradient_x) || !finite(surface.gradient_y)) {
          result.validity = TraversabilityEvaluationValidity::terrain_invalid;
          result.worst_terrain_estimate_status = surface.status;
          add_limiting_factor(
              result,
              TraversabilityLimitingFactor::terrain_estimate_invalid);
          result.issues.emplace_back(
              "footprint surface estimate is unavailable or non-finite");
          return result;
        }
        if (!finite(surface.detrended_roughness_rms_m) ||
            surface.detrended_roughness_rms_m < 0.0) {
          result.validity = TraversabilityEvaluationValidity::terrain_invalid;
          add_limiting_factor(
              result, TraversabilityLimitingFactor::roughness_invalid);
          result.issues.emplace_back(
              "footprint surface roughness is non-finite or negative");
          return result;
        }
        result.maximum_detrended_roughness_rms_m = std::max(
            result.maximum_detrended_roughness_rms_m,
            surface.detrended_roughness_rms_m);
        if (surface.detrended_roughness_rms_m >
            capability_.maximum_roughness_m) {
          add_limiting_factor(
              result, TraversabilityLimitingFactor::roughness_exceeded);
        }
        if (!valid_covariance(surface.gradient_covariance)) {
          result.validity =
              TraversabilityEvaluationValidity::covariance_invalid;
          add_limiting_factor(
              result,
              TraversabilityLimitingFactor::gradient_covariance_invalid);
          result.issues.emplace_back(
              "footprint surface gradient covariance is invalid");
          return result;
        }
        const double longitudinal_mean =
            surface.gradient_x * forward_x + surface.gradient_y * forward_y;
        const CovarianceComponents gradient_covariance =
            components(surface.gradient_covariance);
        const double longitudinal_variance = directional_variance(
            gradient_covariance, forward_x, forward_y);
        const double lateral_mean =
            surface.gradient_x * lateral_x + surface.gradient_y * lateral_y;
        const double lateral_variance =
            directional_variance(gradient_covariance, lateral_x, lateral_y);
        if (!finite(longitudinal_variance) || longitudinal_variance < 0.0 ||
            !finite(lateral_variance) || lateral_variance < 0.0) {
          result.validity =
              TraversabilityEvaluationValidity::covariance_invalid;
          add_limiting_factor(
              result,
              TraversabilityLimitingFactor::gradient_covariance_invalid);
          result.issues.emplace_back(
              "projected footprint gradient variance is invalid");
          return result;
        }
        const double longitudinal_sigma = std::sqrt(longitudinal_variance);
        const double lateral_sigma = std::sqrt(lateral_variance);
        const double lower_gradient =
            longitudinal_mean -
            gradient_risk_policy.coverage_multiplier * longitudinal_sigma;
        const double upper_gradient =
            longitudinal_mean +
            gradient_risk_policy.coverage_multiplier * longitudinal_sigma;
        const double lateral_upper_gradient =
            std::abs(lateral_mean) +
            gradient_risk_policy.coverage_multiplier * lateral_sigma;
        const double lower_angle = std::atan(lower_gradient);
        const double upper_angle = std::atan(upper_gradient);
        const double mean_angle = std::atan(longitudinal_mean);
        const double lateral_upper_angle = std::atan(lateral_upper_gradient);
        if (!has_valid_surface_sample) {
          result.maximum_longitudinal_mean_gradient = longitudinal_mean;
          result.minimum_longitudinal_mean_gradient = longitudinal_mean;
          result.maximum_longitudinal_mean_angle_rad = mean_angle;
          result.minimum_longitudinal_mean_angle_rad = mean_angle;
          result.maximum_longitudinal_upper_angle_rad = upper_angle;
          result.minimum_longitudinal_lower_angle_rad = lower_angle;
          result.maximum_lateral_absolute_upper_angle_rad =
              lateral_upper_angle;
          has_valid_surface_sample = true;
        } else {
          result.maximum_longitudinal_mean_gradient =
              std::max(result.maximum_longitudinal_mean_gradient,
                       longitudinal_mean);
          result.minimum_longitudinal_mean_gradient =
              std::min(result.minimum_longitudinal_mean_gradient,
                       longitudinal_mean);
          result.maximum_longitudinal_mean_angle_rad =
              std::max(result.maximum_longitudinal_mean_angle_rad, mean_angle);
          result.minimum_longitudinal_mean_angle_rad =
              std::min(result.minimum_longitudinal_mean_angle_rad, mean_angle);
          result.maximum_longitudinal_upper_angle_rad = std::max(
              result.maximum_longitudinal_upper_angle_rad, upper_angle);
          result.minimum_longitudinal_lower_angle_rad = std::min(
              result.minimum_longitudinal_lower_angle_rad, lower_angle);
          result.maximum_lateral_absolute_upper_angle_rad =
              std::max(result.maximum_lateral_absolute_upper_angle_rad,
                       lateral_upper_angle);
        }
        if (upper_angle > capability_.maximum_slope_up_rad) {
          add_limiting_factor(
              result, TraversabilityLimitingFactor::up_slope_exceeded);
        }
        if (lower_angle < -capability_.maximum_slope_down_rad) {
          add_limiting_factor(
              result, TraversabilityLimitingFactor::down_slope_exceeded);
        }
        if (lateral_upper_angle > capability_.maximum_slope_lateral_rad) {
          add_limiting_factor(
              result, TraversabilityLimitingFactor::lateral_slope_exceeded);
        }
      }
    }
    evaluate_track_support_at_pose(pose, terrain.surface, track_footprint_,
                                   capability_, result);
  }
  if (!has_valid_surface_sample) {
    result.validity = TraversabilityEvaluationValidity::terrain_invalid;
    result.traversable = false;
    result.issues.emplace_back("swept footprint covered no terrain samples");
  }
  return result;
}

CollisionLayerResult TraversabilityEvaluator::evaluate_collision_layer(
    const MapSnapshot& map, const TerrainLayers& terrain,
    const Covariance2dM2& robot_relative_obstacle_covariance_m2,
    const RobotCollisionRiskPolicy& policy) const {
  CollisionLayerResult result;
  result.source_map_version = map.version;
  result.terrain_analysis_config_version = terrain.analysis_config_version;
  result.collision_risk_policy_version = policy.version;
  result.epsilon_robot = policy.epsilon_robot;
  result.operating_domain_id = policy.operating_domain_id;
  result.calibration_dataset_id = policy.calibration_dataset_id;
  result.risk_semantics = "robot-relative-obstacle-pointwise-only";
  result.width = map.width;
  result.height = map.height;
  result.cells.assign(map.width * map.height, CollisionCellResult{});

  const SnapshotValidation map_validation = validate(map);
  if (!map_validation.valid) {
    result.validity = CollisionEvaluationValidity::input_invalid;
    result.issues = map_validation.issues;
    return result;
  }
  if (map.version != terrain.source_map_version ||
      terrain.analysis_config_version != map.derived_configuration_version ||
      terrain.operating_domain_id != policy.operating_domain_id) {
    result.validity = CollisionEvaluationValidity::version_mismatch;
    result.issues.emplace_back("map, terrain, and collision policy versions differ");
    return result;
  }
  if (map.cells.size() != map.width * map.height ||
      terrain.surface.width != map.width || terrain.surface.height != map.height ||
      terrain.surface.resolution_m != map.resolution_m ||
      terrain.surface.origin_x_m != map.origin_x_m ||
      terrain.surface.origin_y_m != map.origin_y_m ||
      terrain.surface.cells.size() != map.cells.size() || policy.version == 0U ||
      policy.calibration_dataset_id.empty() || policy.operating_domain_id.empty() ||
      !finite(policy.epsilon_robot) || policy.epsilon_robot <= 0.0 ||
      policy.epsilon_robot >= 0.5 || !finite(policy.minimum_map_confidence) ||
      policy.minimum_map_confidence <= 0.0 ||
      policy.minimum_map_confidence > 1.0 || !finite(policy.safe_distance_m) ||
      policy.safe_distance_m < 0.0) {
    result.validity = CollisionEvaluationValidity::input_invalid;
    result.issues.emplace_back("collision evaluation input is invalid");
    return result;
  }
  if (!valid_covariance(robot_relative_obstacle_covariance_m2)) {
    result.validity = CollisionEvaluationValidity::covariance_invalid;
    result.issues.emplace_back("robot-relative obstacle covariance is invalid");
    return result;
  }

  result.validity = CollisionEvaluationValidity::valid;
  for (CollisionCellResult& cell : result.cells) {
    cell.classification = CollisionCellClassification::traversable;
  }
  for (std::size_t index = 0; index < map.cells.size(); ++index) {
    const MapCell& map_cell = map.cells[index];
    const SurfaceEstimate& surface = terrain.surface.cells[index];
    const std::size_t row = index / map.width;
    const std::size_t column = index % map.width;
    if (!map_cell.known) {
      result.cells[index].classification = CollisionCellClassification::unknown;
      result.information_gaps.push_back(
          {row, column, cell_center(map, row, column),
           InformationGapReason::unknown});
    } else if (map_cell.confidence < policy.minimum_map_confidence) {
      result.cells[index].classification =
          CollisionCellClassification::low_confidence;
      result.information_gaps.push_back(
          {row, column, cell_center(map, row, column),
           InformationGapReason::low_confidence});
    } else if (surface.status == TerrainEstimateStatus::discontinuous) {
      result.cells[index].classification =
          CollisionCellClassification::
              step_discontinuity_requires_validation;
    } else if (surface.status != TerrainEstimateStatus::valid) {
      result.cells[index].classification =
          CollisionCellClassification::invalid_terrain;
      result.information_gaps.push_back(
          {row, column, cell_center(map, row, column),
           InformationGapReason::invalid_terrain});
    }
  }
  const double quantile = inverse_standard_normal(1.0 - policy.epsilon_robot);
  const double isotropic_variance =
      covariance_eigenvalues(
          components(robot_relative_obstacle_covariance_m2)).maximum_m2;
  const double boundary_collision_margin =
      quantile * std::sqrt(std::max(0.0, isotropic_variance));
  const double boundary_inflation_radius =
      policy.safe_distance_m + boundary_collision_margin;
  const double maximum_x =
      map.origin_x_m + static_cast<double>(map.width) * map.resolution_m;
  const double maximum_y =
      map.origin_y_m + static_cast<double>(map.height) * map.resolution_m;
  for (std::size_t index = 0; index < result.cells.size(); ++index) {
    const std::size_t row = index / map.width;
    const std::size_t column = index % map.width;
    const Point2d center = cell_center(map, row, column);
    const double distance_to_boundary =
        std::min({center.x_m - map.origin_x_m, maximum_x - center.x_m,
                  center.y_m - map.origin_y_m, maximum_y - center.y_m});
    if (distance_to_boundary <= boundary_inflation_radius) {
      result.cells[index].classification =
          CollisionCellClassification::map_boundary;
      result.cells[index].collision_margin_m = boundary_collision_margin;
    }
  }

  for (std::size_t obstacle_index = 0; obstacle_index < map.cells.size();
       ++obstacle_index) {
    const MapCell& obstacle = map.cells[obstacle_index];
    if (!obstacle.obstacle) continue;
    double variance = isotropic_variance;
    if (obstacle.obstacle_normal.has_value()) {
      variance = directional_variance(robot_relative_obstacle_covariance_m2,
                                      *obstacle.obstacle_normal);
    }
    const double collision_margin = quantile * std::sqrt(std::max(0.0, variance));
    const double inflation_radius = policy.safe_distance_m + collision_margin;
    const std::size_t obstacle_row = obstacle_index / map.width;
    const std::size_t obstacle_column = obstacle_index % map.width;
    const Point2d obstacle_center =
        cell_center(map, obstacle_row, obstacle_column);
    result.cells[obstacle_index].classification =
        CollisionCellClassification::obstacle;
    result.cells[obstacle_index].collision_margin_m = collision_margin;

    for (std::size_t index = 0; index < result.cells.size(); ++index) {
      const std::size_t row = index / map.width;
      const std::size_t column = index % map.width;
      const Point2d center = cell_center(map, row, column);
      const double distance =
          std::hypot(center.x_m - obstacle_center.x_m,
                     center.y_m - obstacle_center.y_m);
      if (distance <= inflation_radius && index != obstacle_index &&
          !map.cells[index].obstacle) {
        result.cells[index].classification =
            CollisionCellClassification::inflated_obstacle;
        result.cells[index].collision_margin_m =
            std::max(result.cells[index].collision_margin_m, collision_margin);
      }
    }
  }
  return result;
}

CollisionSweepResult TraversabilityEvaluator::evaluate_collision_sweep(
    const MotionSegment& segment, const TerrainLayers& terrain,
    const CollisionLayerResult& collision_layer,
    const double maximum_sweep_spacing_fraction) const {
  CollisionSweepResult result;
  const SurfaceLayer& surface = terrain.surface;
  if (collision_layer.validity != CollisionEvaluationValidity::valid ||
      !valid_track_geometry(track_footprint_) ||
      !valid_motion_segment(segment) || !valid_surface_layer(surface) ||
      collision_layer.width != surface.width ||
      collision_layer.height != surface.height ||
      collision_layer.cells.size() != surface.cells.size() ||
      !finite(maximum_sweep_spacing_fraction) ||
      maximum_sweep_spacing_fraction <= 0.0 ||
      maximum_sweep_spacing_fraction > kMaximumSweepSpacingFraction ||
      collision_layer.source_map_version != terrain.source_map_version ||
      collision_layer.terrain_analysis_config_version !=
          terrain.analysis_config_version) {
    return result;
  }

  result.validity = CollisionEvaluationValidity::valid;
  result.collision_free = true;
  const std::vector<Pose2d> sweep =
      adaptive_sweep_poses(segment, track_footprint_, surface.resolution_m,
                           maximum_sweep_spacing_fraction);
  result.evaluated_sweep_poses = sweep.size();
  const double radius = track_footprint_radius(track_footprint_);
  for (std::size_t index = 0U; index + 1U < sweep.size(); ++index) {
    result.maximum_boundary_displacement_m =
        std::max(result.maximum_boundary_displacement_m,
                 measure_pose_pair_sweep(sweep[index], sweep[index + 1U],
                                         radius)
                     .boundary_displacement_m);
  }
  result.sweep_discretization_margin_m =
      0.5 * maximum_sweep_spacing_fraction * surface.resolution_m;

  const double minimum_x = surface.origin_x_m;
  const double minimum_y = surface.origin_y_m;
  const double maximum_x =
      minimum_x + static_cast<double>(surface.width) * surface.resolution_m;
  const double maximum_y =
      minimum_y + static_cast<double>(surface.height) * surface.resolution_m;
  for (const Pose2d& pose : sweep) {
    const std::vector<Point2d> footprint =
        transformed_footprint(track_footprint_, pose);
    const PolygonBounds bounds = polygon_bounds(footprint);
    const double margin = result.sweep_discretization_margin_m;
    if (bounds.minimum_x - margin < minimum_x ||
        bounds.maximum_x + margin >= maximum_x ||
        bounds.minimum_y - margin < minimum_y ||
        bounds.maximum_y + margin >= maximum_y) {
      result.collision_free = false;
      return result;
    }
    const auto minimum_column = static_cast<std::size_t>(std::floor(
        (bounds.minimum_x - margin - minimum_x) / surface.resolution_m));
    const auto maximum_column = std::min(
        surface.width - 1U, static_cast<std::size_t>(std::floor(
                                (bounds.maximum_x + margin - minimum_x) /
                                surface.resolution_m)));
    const auto minimum_row = static_cast<std::size_t>(std::floor(
        (bounds.minimum_y - margin - minimum_y) / surface.resolution_m));
    const auto maximum_row = std::min(
        surface.height - 1U, static_cast<std::size_t>(std::floor(
                                 (bounds.maximum_y + margin - minimum_y) /
                                 surface.resolution_m)));
    for (std::size_t row = minimum_row; row <= maximum_row; ++row) {
      for (std::size_t column = minimum_column; column <= maximum_column;
           ++column) {
        const double cell_x =
            minimum_x + static_cast<double>(column) * surface.resolution_m;
        const double cell_y =
            minimum_y + static_cast<double>(row) * surface.resolution_m;
        if (!polygon_intersects_cell(footprint, cell_x, cell_y,
                                     surface.resolution_m, margin)) {
          continue;
        }
        ++result.evaluated_footprint_cells;
        if (!collision_layer.at(row, column).collision_candidate()) {
          result.collision_free = false;
          return result;
        }
      }
    }
  }
  return result;
}

}  // namespace underwater_planner::core
