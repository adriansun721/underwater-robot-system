#include "underwater_planner/core/path_candidate_verifier.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace underwater_planner::core {
namespace {

constexpr double kGeometryEpsilon = 1.0e-12;

[[nodiscard]] bool finite(const double value) noexcept {
  return std::isfinite(value);
}

[[nodiscard]] double angle_error(const double left,
                                 const double right) noexcept {
  return std::abs(std::remainder(left - right, 2.0 * std::acos(-1.0)));
}

[[nodiscard]] double cross(const Vector2m& left, const Vector2m& right) noexcept {
  return left.x_m * right.y_m - left.y_m * right.x_m;
}

[[nodiscard]] double norm(const Vector2m& value) noexcept {
  return std::hypot(value.x_m, value.y_m);
}

[[nodiscard]] Vector2m difference(const PathPoint& left,
                                  const PathPoint& right) noexcept {
  return {right.x_m - left.x_m, right.y_m - left.y_m};
}

[[nodiscard]] bool finite_boundary(const PathBoundary& boundary,
                                   const bool require_curvature,
                                   const bool require_provenance) noexcept {
  return finite(boundary.x_m) && finite(boundary.y_m) &&
         finite(boundary.heading_rad) &&
         (!require_provenance ||
          (boundary.pose_timestamp.nanoseconds >= 0 &&
           boundary.source_sequence_number != 0U)) &&
         (!require_curvature ||
          (boundary.curvature_per_m.has_value() &&
           finite(*boundary.curvature_per_m) &&
           (!require_provenance || boundary.curvature_timestamp.nanoseconds >= 0)));
}

[[nodiscard]] bool finite_limits(const SmoothingLimits& limits) noexcept {
  return limits.minimum_segment_length_m > 0.0 &&
         finite(limits.minimum_segment_length_m) &&
         limits.maximum_curvature_per_m > 0.0 &&
         finite(limits.maximum_curvature_per_m) &&
         limits.maximum_curvature_rate_per_m2 > 0.0 &&
         finite(limits.maximum_curvature_rate_per_m2) &&
         limits.maximum_boundary_time_skew.nanoseconds >= 0 &&
         limits.allowed_residuals.maximum_dynamics_residual >= 0.0 &&
         finite(limits.allowed_residuals.maximum_dynamics_residual) &&
         limits.allowed_residuals.maximum_curvature_audit_residual >= 0.0 &&
         finite(limits.allowed_residuals.maximum_curvature_audit_residual) &&
         limits.allowed_residuals.maximum_curvature_rate_residual >= 0.0 &&
         finite(limits.allowed_residuals.maximum_curvature_rate_residual) &&
         limits.allowed_residuals.start_position_residual_m >= 0.0 &&
         finite(limits.allowed_residuals.start_position_residual_m) &&
         limits.allowed_residuals.start_heading_residual_rad >= 0.0 &&
         finite(limits.allowed_residuals.start_heading_residual_rad) &&
         limits.allowed_residuals.start_curvature_residual_per_m >= 0.0 &&
         finite(limits.allowed_residuals.start_curvature_residual_per_m) &&
         limits.allowed_residuals.goal_position_residual_m >= 0.0 &&
         finite(limits.allowed_residuals.goal_position_residual_m) &&
         limits.allowed_residuals.goal_heading_residual_rad >= 0.0 &&
         finite(limits.allowed_residuals.goal_heading_residual_rad) &&
         limits.allowed_residuals.goal_curvature_residual_per_m >= 0.0 &&
         finite(limits.allowed_residuals.goal_curvature_residual_per_m);
}

[[nodiscard]] double geometric_curvature(const PathPoint& previous,
                                          const PathPoint& current,
                                          const PathPoint& next) noexcept {
  const Vector2m first = difference(previous, current);
  const Vector2m second = difference(current, next);
  const Vector2m sum{first.x_m + second.x_m, first.y_m + second.y_m};
  const double denominator = norm(first) * norm(second) * norm(sum);
  if (!(denominator > kGeometryEpsilon) || !finite(denominator)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return 2.0 * cross(first, second) / denominator;
}

[[nodiscard]] double tangent_heading(const PathPoint& previous,
                                     const PathPoint& current,
                                     const PathPoint* next) noexcept {
  Vector2m tangent = difference(previous, current);
  if (next != nullptr) {
    const Vector2m outgoing = difference(current, *next);
    tangent = {tangent.x_m + outgoing.x_m, tangent.y_m + outgoing.y_m};
  }
  if (!(norm(tangent) > kGeometryEpsilon)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::atan2(tangent.y_m, tangent.x_m);
}

[[nodiscard]] PathConstraintResiduals boundary_residuals(
    const GeometricPath& path, const PathBoundary& start,
    const PathBoundary& goal) noexcept {
  PathConstraintResiduals result;
  const PathPoint& first = path.points.front();
  const PathPoint& last = path.points.back();
  result.start_position_residual_m =
      std::hypot(first.x_m - start.x_m, first.y_m - start.y_m);
  result.start_heading_residual_rad = angle_error(first.heading_rad, start.heading_rad);
  result.start_curvature_residual_per_m =
      std::abs(first.curvature_per_m - *start.curvature_per_m);
  result.goal_position_residual_m =
      std::hypot(last.x_m - goal.x_m, last.y_m - goal.y_m);
  result.goal_heading_residual_rad = angle_error(last.heading_rad, goal.heading_rad);
  result.goal_curvature_residual_per_m =
      std::abs(last.curvature_per_m - *goal.curvature_per_m);
  return result;
}

[[nodiscard]] bool boundary_within(const PathConstraintResiduals& residuals,
                                   const PathConstraintResiduals& limits) noexcept {
  return residuals.start_position_residual_m <= limits.start_position_residual_m &&
         residuals.start_heading_residual_rad <= limits.start_heading_residual_rad &&
         residuals.start_curvature_residual_per_m <=
             limits.start_curvature_residual_per_m &&
         residuals.goal_position_residual_m <= limits.goal_position_residual_m &&
         residuals.goal_heading_residual_rad <= limits.goal_heading_residual_rad &&
         residuals.goal_curvature_residual_per_m <=
             limits.goal_curvature_residual_per_m;
}

[[nodiscard]] MotionSegment make_motion_segment(
    const GeometricPath& path, const double maximum_spacing_m) {
  MotionSegment result;
  for (std::size_t index = 0U; index + 1U < path.points.size(); ++index) {
    const PathPoint& left = path.points[index];
    const PathPoint& right = path.points[index + 1U];
    const double distance_m = std::hypot(right.x_m - left.x_m,
                                         right.y_m - left.y_m);
    const std::size_t intervals = static_cast<std::size_t>(std::max(
        1.0, std::ceil(distance_m / maximum_spacing_m)));
    for (std::size_t sample = 0U; sample < intervals; ++sample) {
      const double fraction = static_cast<double>(sample) /
                              static_cast<double>(intervals);
      result.samples.push_back(
          {left.x_m + fraction * (right.x_m - left.x_m),
           left.y_m + fraction * (right.y_m - left.y_m),
           normalize_angle_radians(
               left.heading_rad + fraction *
                   normalize_angle_radians(right.heading_rad - left.heading_rad)),
           MonotonicTime{0}});
    }
  }
  const PathPoint& last = path.points.back();
  result.samples.push_back(
      {last.x_m, last.y_m, last.heading_rad, MonotonicTime{0}});
  return result;
}

}  // namespace

PathGeometryAudit auditPathGeometry(
    const GeometricPath& path, const PathBoundary& start,
    const PathBoundary& goal, const SmoothingLimits& limits,
    const double geometric_curvature_tolerance_per_m,
    const double heading_tolerance_rad,
    const double curvature_rate_tolerance_per_m2) {
  PathGeometryAudit result;
  if (!validate(path).valid || !finite_boundary(start, true, true) ||
      !finite_boundary(goal, true, false) || !finite_limits(limits) ||
      !finite(geometric_curvature_tolerance_per_m) ||
      geometric_curvature_tolerance_per_m < 0.0 ||
      !finite(heading_tolerance_rad) || heading_tolerance_rad < 0.0 ||
      !finite(curvature_rate_tolerance_per_m2) ||
      curvature_rate_tolerance_per_m2 < 0.0) {
    result.reason = "path_geometry_or_boundary_invalid";
    return result;
  }
  if (start.curvature_source !=
          PathBoundarySource::synchronized_actual_state &&
      start.curvature_source !=
          PathBoundarySource::committed_segment_terminal) {
    result.reason = "start_boundary_provenance_invalid";
    return result;
  }

  result.boundary_residuals = boundary_residuals(path, start, goal);
  if (std::llabs(start.pose_timestamp.nanoseconds -
                 start.curvature_timestamp.nanoseconds) >
      limits.maximum_boundary_time_skew.nanoseconds) {
    result.reason = "boundary_timestamp_skew_exceeded";
    return result;
  }
  for (std::size_t index = 0U; index < path.points.size(); ++index) {
    const PathPoint& point = path.points[index];
    result.maximum_absolute_geometric_curvature_per_m = std::max(
        result.maximum_absolute_geometric_curvature_per_m,
        std::abs(point.curvature_per_m));
    const PathPoint& previous = path.points[index == 0U ? 0U : index - 1U];
    const PathPoint* next = index + 1U < path.points.size()
                                ? &path.points[index + 1U]
                                : nullptr;
    const double heading = tangent_heading(previous, point, next);
    if (!finite(heading)) {
      result.reason = "degenerate_path_tangent";
      return result;
    }
    const double heading_residual = angle_error(point.heading_rad, heading);
    if (heading_residual > result.maximum_heading_residual_rad) {
      result.maximum_heading_residual_rad = heading_residual;
      result.worst_heading_index = index;
      result.worst_heading_position_m = {point.x_m, point.y_m};
    }
    if (index == 0U || index + 1U == path.points.size()) {
      continue;
    }
    const double curvature = geometric_curvature(path.points[index - 1U], point,
                                                 path.points[index + 1U]);
    if (!finite(curvature)) {
      result.reason = "degenerate_geometric_curvature";
      return result;
    }
    const double curvature_residual = std::abs(curvature - point.curvature_per_m);
    result.maximum_absolute_geometric_curvature_per_m = std::max(
        result.maximum_absolute_geometric_curvature_per_m, std::abs(curvature));
    if (curvature_residual > result.maximum_geometric_curvature_residual_per_m) {
      result.maximum_geometric_curvature_residual_per_m = curvature_residual;
      result.worst_curvature_index = index;
      result.worst_curvature_position_m = {point.x_m, point.y_m};
    }
  }
  for (std::size_t index = 0U; index + 1U < path.points.size(); ++index) {
    const double segment_length_m = path.points[index + 1U].arc_length_m -
                                    path.points[index].arc_length_m;
    if (!(segment_length_m >= limits.minimum_segment_length_m) ||
        !finite(segment_length_m)) {
      result.reason = "path_segment_too_short";
      return result;
    }
    const double rate = std::abs(path.points[index + 1U].curvature_per_m -
                                 path.points[index].curvature_per_m) /
                        segment_length_m;
    if (!finite(rate)) {
      result.reason = "nonfinite_curvature_rate";
      return result;
    }
    if (rate > result.maximum_curvature_rate_per_m2) {
      result.maximum_curvature_rate_per_m2 = rate;
      result.worst_curvature_rate_index = index;
      result.worst_curvature_rate_position_m = {path.points[index].x_m,
                                                path.points[index].y_m};
    }
  }
  result.valid = result.maximum_heading_residual_rad <= heading_tolerance_rad &&
                 result.maximum_geometric_curvature_residual_per_m <=
                     geometric_curvature_tolerance_per_m &&
                 result.maximum_absolute_geometric_curvature_per_m <=
                     limits.maximum_curvature_per_m +
                         geometric_curvature_tolerance_per_m &&
                 result.maximum_curvature_rate_per_m2 <=
                     limits.maximum_curvature_rate_per_m2 +
                         curvature_rate_tolerance_per_m2;
  if (!result.valid) {
    result.reason = result.maximum_heading_residual_rad > heading_tolerance_rad
                        ? "geometric_heading_residual_exceeded"
                        : result.maximum_geometric_curvature_residual_per_m >
                                  geometric_curvature_tolerance_per_m
                              ? "geometric_curvature_residual_exceeded"
                              : result.maximum_absolute_geometric_curvature_per_m >
                                        limits.maximum_curvature_per_m +
                                            geometric_curvature_tolerance_per_m
                                    ? "curvature_limit_exceeded"
                              : "curvature_rate_exceeded";
  } else {
    result.reason = "valid";
  }
  return result;
}

PathG2MergeResult mergePathsG2(const GeometricPath& committed,
                               const GeometricPath& tail,
                               const PathG2MergeLimits& limits) {
  PathG2MergeResult result;
  if (!validate(committed).valid || !validate(tail).valid ||
      committed.metadata.coordinate_frame != tail.metadata.coordinate_frame ||
      committed.metadata.reference_line_version !=
          tail.metadata.reference_line_version ||
      committed.metadata.interpolation_rule != tail.metadata.interpolation_rule ||
      !finite(limits.position_tolerance_m) || limits.position_tolerance_m < 0.0 ||
      !finite(limits.heading_tolerance_rad) || limits.heading_tolerance_rad < 0.0 ||
      !finite(limits.curvature_tolerance_per_m) ||
      limits.curvature_tolerance_per_m < 0.0) {
    result.reason = "path_or_merge_context_invalid";
    return result;
  }
  const PathPoint& left = committed.points.back();
  const PathPoint& right = tail.points.front();
  result.junction_residuals.start_position_residual_m =
      std::hypot(left.x_m - right.x_m, left.y_m - right.y_m);
  result.junction_residuals.start_heading_residual_rad =
      angle_error(left.heading_rad, right.heading_rad);
  result.junction_residuals.start_curvature_residual_per_m =
      std::abs(left.curvature_per_m - right.curvature_per_m);
  if (result.junction_residuals.start_position_residual_m >
          limits.position_tolerance_m ||
      result.junction_residuals.start_heading_residual_rad >
          limits.heading_tolerance_rad ||
      result.junction_residuals.start_curvature_residual_per_m >
          limits.curvature_tolerance_per_m) {
    result.reason = "g2_junction_residual_exceeded";
    return result;
  }
  result.path = committed;
  const double arc_offset = left.arc_length_m - tail.points.front().arc_length_m;
  for (std::size_t index = 1U; index < tail.points.size(); ++index) {
    PathPoint point = tail.points[index];
    point.arc_length_m += arc_offset;
    result.path.points.push_back(point);
  }
  result.path.metadata.path_version =
      std::max(committed.metadata.path_version, tail.metadata.path_version);
  // A merge creates a new geometric object; neither segment's smoothing audit
  // alone certifies the stitched path.  The caller must run a fresh audit.
  result.path.metadata.smoothing.reset();
  result.valid = validate(result.path).valid;
  result.reason = result.valid ? "valid" : "merged_path_invalid";
  return result;
}

PathCandidateVerificationResult PathCandidateVerifier::verify(
    const GeometricPath& path, const PathBoundary& start,
    const PathBoundary& goal, const SmoothingLimits& smoothing_limits,
    const PathCandidateVerificationContext& context) const {
  PathCandidateVerificationResult result;
  result.geometry = auditPathGeometry(
      path, start, goal, smoothing_limits,
      context.geometric_curvature_tolerance_per_m, context.heading_tolerance_rad,
      context.curvature_rate_tolerance_per_m2);
  if (!result.geometry.valid) {
    result.status = result.geometry.reason == "path_geometry_or_boundary_invalid" ||
                            result.geometry.reason ==
                                "start_boundary_provenance_invalid" ||
                            result.geometry.reason == "boundary_timestamp_skew_exceeded"
                        ? PathCandidateVerificationStatus::input_invalid
                        : result.geometry.reason == "curvature_rate_exceeded"
                        ? PathCandidateVerificationStatus::curvature_rate_limit_exceeded
                        : result.geometry.reason == "curvature_limit_exceeded"
                              ? PathCandidateVerificationStatus::curvature_limit_exceeded
                              : PathCandidateVerificationStatus::geometry_invalid;
    result.issues.push_back(result.geometry.reason);
    return result;
  }
  if (!boundary_within(result.geometry.boundary_residuals,
                       smoothing_limits.allowed_residuals)) {
    result.status = PathCandidateVerificationStatus::boundary_residual_exceeded;
    result.issues.push_back("g2_boundary_residual_exceeded");
    return result;
  }
  if (result.geometry.maximum_geometric_curvature_residual_per_m >
          context.geometric_curvature_tolerance_per_m ||
      result.geometry.maximum_absolute_geometric_curvature_per_m >
          smoothing_limits.maximum_curvature_per_m +
              context.geometric_curvature_tolerance_per_m ||
      result.geometry.maximum_curvature_rate_per_m2 >
          smoothing_limits.maximum_curvature_rate_per_m2 +
              context.curvature_rate_tolerance_per_m2) {
    result.status = PathCandidateVerificationStatus::curvature_limit_exceeded;
    result.issues.push_back("curvature_limit_exceeded");
    return result;
  }
  if (!validate(context.map).valid || !validate(context.robot_operating_area).valid ||
      path.metadata.coordinate_frame != context.map.version.coordinate_frame ||
      context.terrain.source_map_version != context.map.version ||
      context.terrain.analysis_config_version !=
          context.map.derived_configuration_version ||
      !finite(context.maximum_sweep_spacing_fraction) ||
      context.maximum_sweep_spacing_fraction <= 0.0 ||
      context.maximum_sweep_spacing_fraction > 0.5 ||
      !(context.map.resolution_m > 0.0) ||
      !finite(context.operating_area_clearance_m) ||
      context.operating_area_clearance_m < 0.0) {
    result.status = PathCandidateVerificationStatus::input_invalid;
    result.issues.push_back("verification_context_invalid");
    return result;
  }
  const MotionSegment segment = make_motion_segment(
      path, context.maximum_sweep_spacing_fraction * context.map.resolution_m);
  for (std::size_t index = 0U; index < segment.samples.size(); ++index) {
    if (!context.robot_operating_area.contains_footprint_with_clearance(
            context.track_footprint.polygon, segment.samples[index],
            context.operating_area_clearance_m)) {
      result.status = PathCandidateVerificationStatus::operating_area_violation;
      result.failing_sample_index = index;
      result.failing_position_m = {segment.samples[index].x_m,
                                   segment.samples[index].y_m};
      result.issues.push_back("robot_operating_area");
      return result;
    }
  }
  TraversabilityEvaluator evaluator(context.robot_capability,
                                    context.track_footprint);
  const CollisionLayerResult collision_layer = evaluator.evaluate_collision_layer(
      context.map, context.terrain, context.robot_relative_obstacle_covariance_m2,
      context.collision_risk_policy);
  result.collision = evaluator.evaluate_collision_sweep(
      segment, context.terrain, collision_layer,
      context.maximum_sweep_spacing_fraction);
  if (result.collision.validity != CollisionEvaluationValidity::valid ||
      !result.collision.collision_free) {
    result.status = PathCandidateVerificationStatus::collision_violation;
    bool located_failure = false;
    for (std::size_t index = 0U; index < segment.samples.size(); ++index) {
      const CollisionSweepResult focused = evaluator.evaluate_collision_sweep(
          MotionSegment{{segment.samples[index]}}, context.terrain,
          collision_layer, context.maximum_sweep_spacing_fraction);
      if (focused.validity != CollisionEvaluationValidity::valid ||
          !focused.collision_free) {
        result.failing_sample_index = index;
        result.failing_position_m = {segment.samples[index].x_m,
                                     segment.samples[index].y_m};
        located_failure = true;
        break;
      }
    }
    if (!located_failure && !segment.samples.empty()) {
      result.failing_sample_index = segment.samples.size() - 1U;
      result.failing_position_m = {segment.samples.back().x_m,
                                   segment.samples.back().y_m};
    }
    result.issues.push_back("robot_collision_or_map_boundary");
    return result;
  }
  result.traversability = evaluator.evaluate(
      segment, context.terrain, context.terrain_gradient_risk_policy);
  if (result.traversability.validity != TraversabilityEvaluationValidity::valid ||
      !result.traversability.traversable) {
    result.status = PathCandidateVerificationStatus::traversability_violation;
    bool located_failure = false;
    for (std::size_t index = 0U; index < segment.samples.size(); ++index) {
      const TraversabilityResult focused = evaluator.evaluate(
          MotionSegment{{segment.samples[index]}}, context.terrain,
          context.terrain_gradient_risk_policy);
      if (focused.validity != TraversabilityEvaluationValidity::valid ||
          !focused.traversable) {
        result.failing_sample_index = index;
        result.failing_position_m = {segment.samples[index].x_m,
                                     segment.samples[index].y_m};
        located_failure = true;
        break;
      }
    }
    if (!located_failure && !segment.samples.empty()) {
      result.failing_sample_index = segment.samples.size() - 1U;
      result.failing_position_m = {segment.samples.back().x_m,
                                   segment.samples.back().y_m};
    }
    result.issues.push_back("terrain_or_step_traversability");
    return result;
  }
  result.valid = true;
  result.status = PathCandidateVerificationStatus::valid;
  result.issues.push_back("complete_path_recheck_passed");
  return result;
}

}  // namespace underwater_planner::core
