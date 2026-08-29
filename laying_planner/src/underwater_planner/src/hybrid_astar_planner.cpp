#include "underwater_planner/core/hybrid_astar_planner.hpp"

#include "storage_estimation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace underwater_planner::core {
namespace {

constexpr double kComparisonTolerance = 1.0e-12;
constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr std::uint64_t kFnvOffsetBasis = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

bool finite(const double value) { return std::isfinite(value); }

bool valid_parameters(const HybridAStarSearchParameters& parameters) {
  if (parameters.version == 0U || parameters.primitive_set_version == 0U ||
      parameters.path_version == 0U || parameters.maximum_expansions == 0U ||
      parameters.maximum_active_labels == 0U ||
      parameters.motion_primitives.empty()) {
    return false;
  }
  const double positive_values[] = {
      parameters.xy_resolution_m,
      parameters.heading_resolution_rad,
      parameters.cable_lag_resolution_rad,
      parameters.reference_progress_resolution_m,
      parameters.goal_position_tolerance_m,
      parameters.goal_heading_tolerance_rad,
      parameters.goal_lag_tolerance_rad,
      parameters.goal_progress_tolerance_m,
      parameters.goal_touchdown_position_tolerance_m,
      parameters.minimum_turning_radius_m,
      parameters.path_length_cost_weight,
  };
  for (const double value : positive_values) {
    if (!finite(value) || value <= 0.0) return false;
  }
  if (!finite(parameters.cable_sweep_margin_m) ||
      parameters.cable_sweep_margin_m < 0.0 ||
      !finite(parameters.equivalent_label_cost_tolerance_m) ||
      parameters.equivalent_label_cost_tolerance_m < 0.0 ||
      !finite(parameters.maximum_planning_duration_s) ||
      parameters.maximum_planning_duration_s <= 0.0 ||
      !finite(parameters.maximum_sweep_spacing_fraction) ||
      parameters.maximum_sweep_spacing_fraction <= 0.0 ||
      parameters.maximum_sweep_spacing_fraction > 0.5 ||
      !finite(parameters.path_curvature_cost_weight) ||
      parameters.path_curvature_cost_weight < 0.0 ||
      !finite(parameters.touchdown_center_cost_weight) ||
      parameters.touchdown_center_cost_weight < 0.0 ||
      !finite(parameters.touchdown_margin_cost_weight) ||
      parameters.touchdown_margin_cost_weight < 0.0 ||
      !finite(parameters.robot_terrain_cost_weight) ||
      parameters.robot_terrain_cost_weight < 0.0) {
    return false;
  }
  for (const HybridAStarMotionPrimitive& primitive :
       parameters.motion_primitives) {
    if (primitive.version == 0U || !finite(primitive.arc_length_m) ||
        primitive.arc_length_m <= 0.0 ||
        !finite(primitive.curvature_per_m) ||
        std::abs(primitive.curvature_per_m) >
            1.0 / parameters.minimum_turning_radius_m +
                kComparisonTolerance) {
      return false;
    }
  }
  return true;
}

DiagnosticEntry diagnostic(const DiagnosticSeverity severity, std::string code,
                           std::string message,
                           const MonotonicTime timestamp) {
  return {severity, std::move(code), "hybrid_astar_search",
          std::move(message), timestamp};
}

void hash_uint64(std::uint64_t& hash, std::uint64_t value) {
  for (int byte = 0; byte < 8; ++byte) {
    hash ^= value & 0xffU;
    hash *= kFnvPrime;
    value >>= 8U;
  }
}

void hash_double(std::uint64_t& hash, const double value) {
  std::uint64_t bits{};
  static_assert(sizeof(bits) == sizeof(value),
                "fingerprint assumes IEEE-754 binary64 storage");
  std::memcpy(&bits, &value, sizeof(bits));
  hash_uint64(hash, bits);
}

void hash_string(std::uint64_t& hash, const std::string& value) {
  hash_uint64(hash, static_cast<std::uint64_t>(value.size()));
  for (const unsigned char byte : value) {
    hash ^= byte;
    hash *= kFnvPrime;
  }
}

void finalize_fingerprint(HybridAStarPlanningResult& result) {
  std::uint64_t hash = kFnvOffsetBasis;
  hash_uint64(hash, static_cast<std::uint64_t>(result.state));
  const HybridAStarSearchDiagnostics& diagnostics = result.diagnostics;
  hash_uint64(hash, diagnostics.search_parameter_version);
  hash_uint64(hash, diagnostics.primitive_set_version);
  hash_uint64(hash, diagnostics.cable_model_version);
  hash_uint64(hash, diagnostics.uncertainty_envelope_version);
  hash_uint64(hash, diagnostics.uncertainty_envelope_generator_version);
  hash_uint64(hash, diagnostics.execution_operating_envelope_version);
  hash_uint64(hash, diagnostics.corridor_risk_policy_version);
  hash_uint64(hash, diagnostics.terrain_map_sequence);
  hash_uint64(hash, diagnostics.terrain_analysis_config_version);
  hash_uint64(hash, diagnostics.collision_risk_policy_version);
  hash_uint64(hash, diagnostics.terrain_gradient_risk_policy_version);
  hash_uint64(hash, diagnostics.cable_laying_limits_version);
  hash_uint64(hash, diagnostics.robot_operating_area_version);
  hash_uint64(hash, diagnostics.reference_line_version);
  hash_uint64(hash, diagnostics.random_seed);
  hash_double(hash, diagnostics.epsilon_point);
  hash_double(hash, diagnostics.standard_normal_quantile);
  hash_double(hash, diagnostics.maximum_sweep_spacing_fraction);
  hash_double(hash, diagnostics.envelope_discretization_margin_m);
  hash_double(hash, diagnostics.cable_sweep_margin_m);
  hash_double(hash, diagnostics.equivalent_label_cost_tolerance_m);
  hash_double(hash, diagnostics.maximum_planning_duration_s);
  hash_double(hash, diagnostics.goal_touchdown_position_tolerance_m);
  hash_double(hash, diagnostics.minimum_turning_radius_m);
  hash_double(hash, diagnostics.path_length_cost_weight);
  hash_double(hash, diagnostics.path_curvature_cost_weight);
  hash_double(hash, diagnostics.touchdown_center_cost_weight);
  hash_double(hash, diagnostics.touchdown_margin_cost_weight);
  hash_double(hash, diagnostics.robot_terrain_cost_weight);
  hash_double(hash, diagnostics.maximum_envelope_stddev_upper_bound_m);
  hash_uint64(hash,
              static_cast<std::uint64_t>(diagnostics.expanded_state_count));
  hash_uint64(hash,
              static_cast<std::uint64_t>(diagnostics.generated_successor_count));
  hash_uint64(hash,
              static_cast<std::uint64_t>(diagnostics.envelope_query_count));
  hash_uint64(
      hash,
      static_cast<std::uint64_t>(diagnostics.cable_model_rejection_count));
  hash_uint64(hash, static_cast<std::uint64_t>(
                        diagnostics.reference_association_rejection_count));
  hash_uint64(hash, static_cast<std::uint64_t>(
                        diagnostics.envelope_unavailable_rejection_count));
  hash_uint64(hash,
              static_cast<std::uint64_t>(diagnostics.corridor_rejection_count));
  hash_uint64(
      hash,
      static_cast<std::uint64_t>(diagnostics.operating_area_rejection_count));
  hash_uint64(hash,
              static_cast<std::uint64_t>(diagnostics.collision_rejection_count));
  hash_uint64(
      hash,
      static_cast<std::uint64_t>(diagnostics.traversability_rejection_count));
  hash_uint64(
      hash,
      static_cast<std::uint64_t>(diagnostics.cable_laying_rejection_count));
  hash_uint64(
      hash,
      static_cast<std::uint64_t>(diagnostics.maximum_active_label_budget));
  hash_uint64(hash,
              static_cast<std::uint64_t>(diagnostics.fixed_bytes_per_search_label));
  hash_uint64(
      hash, static_cast<std::uint64_t>(
                diagnostics.peak_observed_bytes_per_search_label));
  hash_uint64(hash, static_cast<std::uint64_t>(
                        diagnostics.analytic_expansion_interval));
  hash_uint64(hash,
              static_cast<std::uint64_t>(diagnostics.active_label_count));
  hash_uint64(hash,
              static_cast<std::uint64_t>(diagnostics.peak_active_label_count));
  hash_uint64(
      hash,
      static_cast<std::uint64_t>(diagnostics.maximum_labels_per_base_key));
  hash_uint64(hash,
              static_cast<std::uint64_t>(diagnostics.labels_per_base_key_p50));
  hash_uint64(hash,
              static_cast<std::uint64_t>(diagnostics.labels_per_base_key_p95));
  hash_uint64(hash,
              static_cast<std::uint64_t>(diagnostics.labels_per_base_key_p99));
  hash_uint64(hash, static_cast<std::uint64_t>(
                        diagnostics.equivalent_label_discard_count));
  hash_uint64(hash, static_cast<std::uint64_t>(
                        diagnostics.equivalent_label_replacement_count));
  hash_uint64(hash, static_cast<std::uint64_t>(
                        diagnostics.signature_fallback_comparison_count));
  hash_uint64(hash,
              static_cast<std::uint64_t>(diagnostics.stale_queue_entry_count));
  hash_uint64(hash, static_cast<std::uint64_t>(
                        diagnostics.analytic_expansion_attempt_count));
  hash_uint64(hash, static_cast<std::uint64_t>(
                        diagnostics.analytic_expansion_accepted_count));
  hash_uint64(hash, diagnostics.active_label_budget_exhausted ? 1U : 0U);
  hash_uint64(hash, diagnostics.deadline_exceeded ? 1U : 0U);
  hash_uint64(
      hash,
      static_cast<std::uint64_t>(diagnostics.maximum_robot_sweep_pose_count));
  hash_double(hash, diagnostics.maximum_robot_sweep_spacing_m);
  hash_double(hash, diagnostics.maximum_collision_sweep_margin_m);
  hash_double(hash, diagnostics.maximum_primitive_laying_soft_cost);
  hash_double(hash, diagnostics.initial_heuristic_cost);
  hash_double(hash, diagnostics.solution_cost);
  hash_double(hash, diagnostics.solution_cost_components.robot_length);
  hash_double(hash, diagnostics.solution_cost_components.robot_curvature);
  hash_double(hash, diagnostics.solution_cost_components.touchdown_corridor);
  hash_double(hash, diagnostics.solution_cost_components.cable_suitability);
  hash_double(hash, diagnostics.solution_cost_components.robot_terrain);
  hash_uint64(hash, diagnostics.worst_constraint.recorded ? 1U : 0U);
  hash_string(hash, diagnostics.worst_constraint.reason);
  hash_double(hash, diagnostics.worst_constraint.position_m.x_m);
  hash_double(hash, diagnostics.worst_constraint.position_m.y_m);
  hash_double(hash, diagnostics.worst_constraint.constraint_value);
  hash_double(hash, diagnostics.worst_constraint.hard_limit);
  hash_double(hash, diagnostics.worst_constraint.normalized_utilization);
  hash_uint64(hash, diagnostics.path_dependent_covariance_propagated ? 1U : 0U);
  hash_string(hash, diagnostics.operating_domain_id);
  hash_string(hash, diagnostics.risk_semantics);
  hash_string(hash, diagnostics.queue_rule);
  for (const DiagnosticEntry& entry : diagnostics.entries) {
    hash_uint64(hash, static_cast<std::uint64_t>(entry.severity));
    hash_string(hash, entry.code);
    hash_string(hash, entry.stage);
    hash_string(hash, entry.message);
    hash_uint64(hash, static_cast<std::uint64_t>(entry.timestamp.nanoseconds));
  }
  for (const PathPoint& point : result.robot_path.points) {
    hash_double(hash, point.arc_length_m);
    hash_double(hash, point.x_m);
    hash_double(hash, point.y_m);
    hash_double(hash, point.heading_rad);
    hash_double(hash, point.curvature_per_m);
  }
  for (const PathPoint& point : result.touchdown_path.points) {
    hash_double(hash, point.arc_length_m);
    hash_double(hash, point.x_m);
    hash_double(hash, point.y_m);
    hash_double(hash, point.heading_rad);
    hash_double(hash, point.curvature_per_m);
  }
  for (const HybridAStarStateTraceEntry& entry : result.state_trace) {
    hash_uint64(hash, static_cast<std::uint64_t>(entry.base_key.x_index));
    hash_uint64(hash, static_cast<std::uint64_t>(entry.base_key.y_index));
    hash_uint64(hash, static_cast<std::uint64_t>(entry.base_key.heading_index));
    hash_uint64(hash,
                static_cast<std::uint64_t>(entry.base_key.cable_lag_index));
    hash_uint64(
        hash,
        static_cast<std::uint64_t>(entry.base_key.reference_progress_index));
    hash_double(hash, entry.robot_pose.x_m);
    hash_double(hash, entry.robot_pose.y_m);
    hash_double(hash, entry.robot_pose.heading_rad);
    hash_double(hash, entry.cable_lag_angle_rad);
    hash_double(hash, entry.reference_progress.arc_length_m);
  }
  hash_double(hash, result.terminal_cable_state.lag_angle_rad);
  hash_double(hash, result.terminal_reference_progress.arc_length_m);
  result.diagnostics.deterministic_fingerprint = hash;
}

HybridAStarBaseKey discretize(const Pose2d& pose, const double lag_angle_rad,
                              const ReferenceProgress& progress,
                              const HybridAStarSearchParameters& parameters) {
  return {
      static_cast<std::int64_t>(std::llround(pose.x_m /
                                            parameters.xy_resolution_m)),
      static_cast<std::int64_t>(std::llround(pose.y_m /
                                            parameters.xy_resolution_m)),
      static_cast<std::int64_t>(std::llround(
          normalize_angle_radians(pose.heading_rad) /
          parameters.heading_resolution_rad)),
      static_cast<std::int64_t>(std::llround(
          normalize_angle_radians(lag_angle_rad) /
          parameters.cable_lag_resolution_rad)),
      static_cast<std::int64_t>(std::llround(
          progress.arc_length_m /
          parameters.reference_progress_resolution_m)),
  };
}

PathPoint point_on_primitive(const Pose2d& start,
                             const HybridAStarMotionPrimitive& primitive,
                             const double arc_length_m) {
  const double heading =
      start.heading_rad + primitive.curvature_per_m * arc_length_m;
  double x_m{};
  double y_m{};
  if (std::abs(primitive.curvature_per_m) <= kComparisonTolerance) {
    x_m = start.x_m + arc_length_m * std::cos(start.heading_rad);
    y_m = start.y_m + arc_length_m * std::sin(start.heading_rad);
  } else {
    x_m = start.x_m +
          (std::sin(heading) - std::sin(start.heading_rad)) /
              primitive.curvature_per_m;
    y_m = start.y_m +
          (std::cos(start.heading_rad) - std::cos(heading)) /
              primitive.curvature_per_m;
  }
  return {arc_length_m, x_m, y_m, normalize_angle_radians(heading),
          primitive.curvature_per_m};
}

GeometricPath apply_motion(const Pose2d& start,
                           const HybridAStarMotionPrimitive& primitive,
                           const std::uint64_t path_version,
                           const std::uint32_t reference_line_version,
                           const std::string& coordinate_frame,
                           const double map_resolution_m,
                           const double footprint_radius_m,
                           const double maximum_sweep_spacing_fraction) {
  const double length_m = primitive.arc_length_m;
  const double curvature_per_m = primitive.curvature_per_m;
  const double heading_change = curvature_per_m * length_m;
  const double sweep_distance_m =
      length_m + footprint_radius_m * std::abs(heading_change);
  const auto intervals = static_cast<std::size_t>(std::max(
      1.0, std::ceil(sweep_distance_m /
                     (maximum_sweep_spacing_fraction * map_resolution_m))));
  GeometricPath segment;
  segment.metadata = {path_version, coordinate_frame, reference_line_version,
                      "constant-curvature-exact"};
  segment.points.reserve(intervals + 1U);
  for (std::size_t index = 0U; index <= intervals; ++index) {
    const double arc_length_m =
        length_m * static_cast<double>(index) / static_cast<double>(intervals);
    segment.points.push_back(point_on_primitive(start, primitive, arc_length_m));
  }
  return segment;
}

MotionSegment motion_segment(const GeometricPath& path,
                             const MonotonicTime timestamp) {
  MotionSegment segment;
  segment.samples.reserve(path.points.size());
  for (const PathPoint& point : path.points) {
    segment.samples.push_back(
        {point.x_m, point.y_m, point.heading_rad, timestamp});
  }
  return segment;
}

std::optional<Pose2d> operating_area_violation(
    const MotionSegment& segment, const TrackFootprint& footprint,
    const RobotOperatingArea& operating_area,
    const double boundary_clearance_m) {
  for (const Pose2d& pose : segment.samples) {
    if (!operating_area.contains_footprint_with_clearance(
            footprint.polygon, pose, boundary_clearance_m)) {
      return pose;
    }
  }
  return std::nullopt;
}

double goal_distance(const Pose2d& pose, const MergeGoal& goal) {
  return std::hypot(pose.x_m - goal.robot_pose.x_m,
                    pose.y_m - goal.robot_pose.y_m);
}

bool matches_goal(const Pose2d& pose, const double lag_angle_rad,
                  const ReferenceProgress& progress,
                  const CableMeanSample& predicted_touchdown,
                  const MergeGoal& goal,
                  const HybridAStarSearchParameters& parameters) {
  return goal_distance(pose, goal) <= parameters.goal_position_tolerance_m &&
         std::abs(normalize_angle_radians(pose.heading_rad -
                                          goal.robot_pose.heading_rad)) <=
             parameters.goal_heading_tolerance_rad &&
         std::abs(normalize_angle_radians(lag_angle_rad -
                                          goal.cable_lag_angle_rad)) <=
             parameters.goal_lag_tolerance_rad &&
         std::abs(progress.arc_length_m - goal.reference_progress_m) <=
             parameters.goal_progress_tolerance_m &&
         predicted_touchdown.validity == CableModelValidity::valid &&
         std::hypot(predicted_touchdown.touchdown_position_m.x_m -
                        goal.touchdown_target_m.x_m,
                    predicted_touchdown.touchdown_position_m.y_m -
                        goal.touchdown_target_m.y_m) <=
             parameters.goal_touchdown_position_tolerance_m;
}

double positive_mod_two_pi(const double angle_rad) {
  double result = std::fmod(angle_rad, kTwoPi);
  if (result < 0.0) result += kTwoPi;
  return result;
}

enum class DubinsSegmentKind { left, straight, right };

struct DubinsPath {
  std::array<DubinsSegmentKind, 3U> kinds{
      DubinsSegmentKind::straight, DubinsSegmentKind::straight,
      DubinsSegmentKind::straight};
  std::array<double, 3U> normalized_lengths{};
  double total_normalized_length{std::numeric_limits<double>::infinity()};
};

void retain_dubins_candidate(
    DubinsPath& shortest,
    const std::array<DubinsSegmentKind, 3U>& kinds,
    const double first_turn, const double straight_or_middle_turn,
    const double final_turn) {
  const double length = first_turn + straight_or_middle_turn + final_turn;
  if (finite(length) && length >= 0.0 &&
      length < shortest.total_normalized_length) {
    shortest.kinds = kinds;
    shortest.normalized_lengths = {
        first_turn, straight_or_middle_turn, final_turn};
    shortest.total_normalized_length = length;
  }
}

DubinsPath shortest_dubins_path(const Pose2d& start, const Pose2d& goal,
                                const double minimum_turning_radius_m) {
  const double dx = (goal.x_m - start.x_m) / minimum_turning_radius_m;
  const double dy = (goal.y_m - start.y_m) / minimum_turning_radius_m;
  const double distance = std::hypot(dx, dy);
  if (distance <= kComparisonTolerance &&
      std::abs(normalize_angle_radians(goal.heading_rad - start.heading_rad)) <=
          kComparisonTolerance) {
    DubinsPath zero;
    zero.total_normalized_length = 0.0;
    return zero;
  }
  const double direction = std::atan2(dy, dx);
  const double alpha = positive_mod_two_pi(start.heading_rad - direction);
  const double beta = positive_mod_two_pi(goal.heading_rad - direction);
  const double sin_alpha = std::sin(alpha);
  const double sin_beta = std::sin(beta);
  const double cos_alpha = std::cos(alpha);
  const double cos_beta = std::cos(beta);
  const double cos_alpha_minus_beta = std::cos(alpha - beta);
  DubinsPath shortest;

  double squared = 2.0 + distance * distance -
                   2.0 * cos_alpha_minus_beta +
                   2.0 * distance * (sin_alpha - sin_beta);
  if (squared >= -kComparisonTolerance) {
    const double middle = std::sqrt(std::max(0.0, squared));
    const double direction_offset =
        std::atan2(cos_beta - cos_alpha,
                   distance + sin_alpha - sin_beta);
    retain_dubins_candidate(
        shortest,
        {DubinsSegmentKind::left, DubinsSegmentKind::straight,
         DubinsSegmentKind::left},
        positive_mod_two_pi(-alpha + direction_offset), middle,
        positive_mod_two_pi(beta - direction_offset));
  }

  squared = 2.0 + distance * distance - 2.0 * cos_alpha_minus_beta +
            2.0 * distance * (-sin_alpha + sin_beta);
  if (squared >= -kComparisonTolerance) {
    const double middle = std::sqrt(std::max(0.0, squared));
    const double direction_offset =
        std::atan2(cos_alpha - cos_beta,
                   distance - sin_alpha + sin_beta);
    retain_dubins_candidate(
        shortest,
        {DubinsSegmentKind::right, DubinsSegmentKind::straight,
         DubinsSegmentKind::right},
        positive_mod_two_pi(alpha - direction_offset), middle,
        positive_mod_two_pi(-beta + direction_offset));
  }

  squared = -2.0 + distance * distance + 2.0 * cos_alpha_minus_beta +
            2.0 * distance * (sin_alpha + sin_beta);
  if (squared >= -kComparisonTolerance) {
    const double middle = std::sqrt(std::max(0.0, squared));
    const double direction_offset =
        std::atan2(-cos_alpha - cos_beta,
                   distance + sin_alpha + sin_beta) -
        std::atan2(-2.0, middle);
    retain_dubins_candidate(
        shortest,
        {DubinsSegmentKind::left, DubinsSegmentKind::straight,
         DubinsSegmentKind::right},
        positive_mod_two_pi(-alpha + direction_offset), middle,
        positive_mod_two_pi(-beta + direction_offset));
  }

  squared = distance * distance - 2.0 + 2.0 * cos_alpha_minus_beta -
            2.0 * distance * (sin_alpha + sin_beta);
  if (squared >= -kComparisonTolerance) {
    const double middle = std::sqrt(std::max(0.0, squared));
    const double direction_offset =
        std::atan2(cos_alpha + cos_beta,
                   distance - sin_alpha - sin_beta) -
        std::atan2(2.0, middle);
    retain_dubins_candidate(
        shortest,
        {DubinsSegmentKind::right, DubinsSegmentKind::straight,
         DubinsSegmentKind::left},
        positive_mod_two_pi(alpha - direction_offset), middle,
        positive_mod_two_pi(beta - direction_offset));
  }

  double cosine_middle =
      (6.0 - distance * distance + 2.0 * cos_alpha_minus_beta +
       2.0 * distance * (sin_alpha - sin_beta)) /
      8.0;
  if (std::abs(cosine_middle) <= 1.0 + kComparisonTolerance) {
    cosine_middle = std::clamp(cosine_middle, -1.0, 1.0);
    const double middle =
        positive_mod_two_pi(kTwoPi - std::acos(cosine_middle));
    const double first = positive_mod_two_pi(
        alpha - std::atan2(cos_alpha - cos_beta,
                           distance - sin_alpha + sin_beta) +
        0.5 * middle);
    retain_dubins_candidate(
        shortest,
        {DubinsSegmentKind::right, DubinsSegmentKind::left,
         DubinsSegmentKind::right},
        first, middle,
        positive_mod_two_pi(alpha - beta - first + middle));
  }

  cosine_middle =
      (6.0 - distance * distance + 2.0 * cos_alpha_minus_beta +
       2.0 * distance * (-sin_alpha + sin_beta)) /
      8.0;
  if (std::abs(cosine_middle) <= 1.0 + kComparisonTolerance) {
    cosine_middle = std::clamp(cosine_middle, -1.0, 1.0);
    const double middle =
        positive_mod_two_pi(kTwoPi - std::acos(cosine_middle));
    const double first = positive_mod_two_pi(
        -alpha - std::atan2(cos_alpha - cos_beta,
                            distance + sin_alpha - sin_beta) +
        0.5 * middle);
    retain_dubins_candidate(
        shortest,
        {DubinsSegmentKind::left, DubinsSegmentKind::right,
         DubinsSegmentKind::left},
        first, middle,
        positive_mod_two_pi(beta - alpha - first + middle));
  }
  return shortest;
}

double dubins_distance(const Pose2d& start, const Pose2d& goal,
                       const double minimum_turning_radius_m) {
  return minimum_turning_radius_m *
         shortest_dubins_path(start, goal, minimum_turning_radius_m)
             .total_normalized_length;
}

std::optional<HybridAStarMotionPrimitive> next_dubins_primitive(
    const Pose2d& start, const MergeGoal& goal,
    const std::uint64_t primitive_set_version,
    const double minimum_turning_radius_m) {
  const DubinsPath path = shortest_dubins_path(
      start, goal.robot_pose, minimum_turning_radius_m);
  if (!finite(path.total_normalized_length)) return std::nullopt;
  for (std::size_t index = 0U; index < path.normalized_lengths.size();
       ++index) {
    const double arc_length_m =
        path.normalized_lengths[index] * minimum_turning_radius_m;
    if (arc_length_m <= kComparisonTolerance) continue;
    double curvature_per_m = 0.0;
    if (path.kinds[index] == DubinsSegmentKind::left) {
      curvature_per_m = 1.0 / minimum_turning_radius_m;
    } else if (path.kinds[index] == DubinsSegmentKind::right) {
      curvature_per_m = -1.0 / minimum_turning_radius_m;
    }
    return HybridAStarMotionPrimitive{
        primitive_set_version, arc_length_m, curvature_per_m};
  }
  return std::nullopt;
}

double heuristic(const Pose2d& pose, const std::vector<MergeGoal>& goals,
                 const HybridAStarSearchParameters& parameters) {
  double value = std::numeric_limits<double>::infinity();
  for (const MergeGoal& goal : goals) {
    const double exact_goal_distance_m =
        dubins_distance(pose, goal.robot_pose,
                        parameters.minimum_turning_radius_m);
    const double position_lower_bound_m =
        std::max(0.0, goal_distance(pose, goal) -
                          parameters.goal_position_tolerance_m);
    const double heading_error_rad = std::abs(normalize_angle_radians(
        pose.heading_rad - goal.robot_pose.heading_rad));
    const double heading_lower_bound_m =
        parameters.minimum_turning_radius_m *
        std::max(0.0, heading_error_rad -
                          parameters.goal_heading_tolerance_rad);
    const double goal_region_lower_bound_m =
        std::max(position_lower_bound_m, heading_lower_bound_m);
    value = std::min(
        value, std::min(exact_goal_distance_m, goal_region_lower_bound_m));
  }
  return parameters.path_length_cost_weight * value;
}

struct SearchNode {
  std::size_t id{};
  std::uint64_t label_id{};
  Pose2d robot_pose;
  CableState cable_state;
  ReferenceProgress progress;
  HybridAStarBaseKey base_key;
  double path_cost_m{};
  HybridAStarCostComponents cost_components;
  double heuristic_m{};
  std::size_t parent_id{};
  GeometricPath incoming_robot_segment;
  CablePrediction incoming_cable_prediction;
};

struct OpenEntry {
  double total_cost_m{};
  double heuristic_m{};
  double path_cost_m{};
  std::uint64_t label_id{};
  std::size_t node_id{};
};

struct OpenEntryLater {
  bool operator()(const OpenEntry& left, const OpenEntry& right) const {
    return std::tie(left.total_cost_m, left.heuristic_m, left.path_cost_m,
                    left.label_id, left.node_id) >
           std::tie(right.total_cost_m, right.heuristic_m, right.path_cost_m,
                    right.label_id, right.node_id);
  }
};

struct MechanicalLabel {
  std::size_t active_node_id{};
  double path_cost_m{};
};

std::size_t observed_label_storage_bytes(const SearchNode& node) {
  return sizeof(SearchNode) + sizeof(MechanicalLabel) + sizeof(OpenEntry) +
         detail::dynamic_storage_bytes(node.cable_state) +
         detail::dynamic_storage_bytes(node.incoming_robot_segment) +
         detail::dynamic_storage_bytes(node.incoming_cable_prediction);
}

void observe_label_storage(const SearchNode& node,
                           HybridAStarSearchDiagnostics& diagnostics) {
  diagnostics.peak_observed_bytes_per_search_label =
      std::max(diagnostics.peak_observed_bytes_per_search_label,
               observed_label_storage_bytes(node));
}

struct BaseKeyLabels {
  std::map<std::uint64_t, std::vector<std::uint64_t>> by_signature;
  std::size_t label_count{};
};

std::size_t nearest_rank_percentile(const std::vector<std::size_t>& sorted,
                                    const std::size_t percentile) {
  const std::size_t count = sorted.size();
  const std::size_t rank =
      (count / 100U) * percentile +
      ((count % 100U) * percentile + 99U) / 100U;
  return sorted[rank - 1U];
}

void update_label_distribution(
    const std::map<HybridAStarBaseKey, BaseKeyLabels>& labels_by_key,
    HybridAStarSearchDiagnostics& diagnostics) {
  std::vector<std::size_t> counts;
  counts.reserve(labels_by_key.size());
  for (const auto& [base_key, labels] : labels_by_key) {
    static_cast<void>(base_key);
    counts.push_back(labels.label_count);
  }
  std::sort(counts.begin(), counts.end());
  diagnostics.labels_per_base_key_p50 = nearest_rank_percentile(counts, 50U);
  diagnostics.labels_per_base_key_p95 = nearest_rank_percentile(counts, 95U);
  diagnostics.labels_per_base_key_p99 = nearest_rank_percentile(counts, 99U);
}

enum class RequestValidation {
  valid,
  input_invalid,
  dependency_mismatch,
};

bool valid_robot_terrain_cost_layer(const TerrainLayers& terrain) {
  return std::all_of(
      terrain.surface.cells.begin(), terrain.surface.cells.end(),
      [](const SurfaceEstimate& cell) {
        return finite(cell.detrended_roughness_rms_m) &&
               cell.detrended_roughness_rms_m >= 0.0;
      });
}

RequestValidation validate_request(
    const HybridAStarPlanningRequest& request,
    const CableModelIdentity& model,
    const HybridAStarSearchParameters& parameters,
    const CableCorridorRiskPolicy& policy) {
  if (!validate(request.start_state).valid ||
      !validate(request.initial_cable_state).valid ||
      !validate(request.initial_reference_progress).valid ||
      !validate(request.reference_line).valid || request.goals.empty() ||
      request.planning_timestamp.nanoseconds < 0 || request.random_seed == 0U ||
      request.cable_context.mode != PredictionMode::search ||
      request.locked_uncertainty_envelope.envelope == nullptr ||
      !validate(request.primitive_sweep_context.map).valid ||
      !validate(request.primitive_sweep_context.robot_operating_area).valid ||
      !valid_robot_terrain_cost_layer(
          request.primitive_sweep_context.terrain)) {
    return RequestValidation::input_invalid;
  }
  const CableUncertaintyEnvelope& envelope =
      *request.locked_uncertainty_envelope.envelope;
  if (request.start_state.pose.timestamp.nanoseconds !=
          request.initial_cable_state.timestamp.nanoseconds ||
      request.initial_cable_state.timestamp.nanoseconds !=
          request.cable_context.current_telemetry.timestamp.nanoseconds ||
      request.initial_reference_progress.reference_line_version !=
          request.reference_line.version ||
      request.cable_context.uncertainty_envelope_version !=
          request.locked_uncertainty_envelope.envelope_version ||
      request.cable_context.uncertainty_envelope_generator_version !=
          envelope.dependencies.generator_version ||
      request.cable_context.execution_envelope.version !=
          envelope.dependencies.execution_operating_envelope_version ||
      request.cable_context.execution_envelope.operating_domain_id !=
          envelope.dependencies.operating_domain_id ||
      request.cable_context.sensor_mode != envelope.dependencies.sensor_mode ||
      request.reference_line.version !=
          envelope.dependencies.reference_line_version ||
      model.version != envelope.dependencies.cable_model_version ||
      model.calibration_dataset_id !=
          envelope.dependencies.cable_model_calibration_dataset_id ||
      model.operating_domain_id != envelope.dependencies.operating_domain_id ||
      parameters.primitive_set_version !=
          envelope.dependencies.primitive_set_version ||
      policy.operating_domain_id != envelope.dependencies.operating_domain_id) {
    return RequestValidation::dependency_mismatch;
  }
  const HybridAStarPrimitiveSweepContext& sweep =
      request.primitive_sweep_context;
  if (sweep.map.version.map_id != sweep.terrain.source_map_version.map_id ||
      sweep.map.version.sequence_number !=
          sweep.terrain.source_map_version.sequence_number ||
      sweep.map.version.coordinate_frame !=
          sweep.terrain.source_map_version.coordinate_frame ||
      sweep.map.derived_configuration_version !=
          sweep.terrain.analysis_config_version ||
      sweep.map.version.coordinate_frame != request.reference_line.coordinate_frame ||
      sweep.collision_risk_policy.operating_domain_id !=
          model.operating_domain_id ||
      sweep.terrain_gradient_risk_policy.operating_domain_id !=
          model.operating_domain_id ||
      sweep.cable_laying_limits.operating_domain_id != model.operating_domain_id ||
      sweep.terrain.operating_domain_id != model.operating_domain_id) {
    return RequestValidation::dependency_mismatch;
  }
  for (const MergeGoal& goal : request.goals) {
    if (!finite(goal.robot_pose.x_m) || !finite(goal.robot_pose.y_m) ||
        !finite(goal.robot_pose.heading_rad) ||
        !finite(goal.cable_lag_angle_rad) ||
        !finite(goal.reference_progress_m) ||
        !finite(goal.touchdown_target_m.x_m) ||
        !finite(goal.touchdown_target_m.y_m) ||
        !finite(goal.cable_heading_rad) || !finite(goal.merge_distance_m)) {
      return RequestValidation::input_invalid;
    }
    if (goal.reference_line_version != request.reference_line.version ||
        goal.cable_model_version != model.version ||
        goal.robot_operating_area_version !=
            request.primitive_sweep_context.robot_operating_area.version) {
      return RequestValidation::dependency_mismatch;
    }
  }
  return RequestValidation::valid;
}

void initialize_diagnostics(HybridAStarPlanningResult& result,
                            const HybridAStarPlanningRequest& request,
                            const CableModelIdentity& model,
                            const HybridAStarSearchParameters& parameters,
                            const CableCorridorRiskPolicy& corridor_policy) {
  result.diagnostics.search_parameter_version = parameters.version;
  result.diagnostics.primitive_set_version = parameters.primitive_set_version;
  result.diagnostics.cable_model_version = model.version;
  result.diagnostics.uncertainty_envelope_version =
      request.locked_uncertainty_envelope.envelope_version;
  result.diagnostics.uncertainty_envelope_generator_version =
      request.cable_context.uncertainty_envelope_generator_version;
  result.diagnostics.execution_operating_envelope_version =
      request.cable_context.execution_envelope.version;
  result.diagnostics.corridor_risk_policy_version = corridor_policy.version;
  const HybridAStarPrimitiveSweepContext& sweep =
      request.primitive_sweep_context;
  result.diagnostics.terrain_map_sequence =
      sweep.terrain.source_map_version.sequence_number;
  result.diagnostics.terrain_analysis_config_version =
      sweep.terrain.analysis_config_version;
  result.diagnostics.collision_risk_policy_version =
      sweep.collision_risk_policy.version;
  result.diagnostics.terrain_gradient_risk_policy_version =
      sweep.terrain_gradient_risk_policy.version;
  result.diagnostics.cable_laying_limits_version =
      sweep.cable_laying_limits.version;
  result.diagnostics.robot_operating_area_version =
      sweep.robot_operating_area.version;
  result.diagnostics.reference_line_version = request.reference_line.version;
  result.diagnostics.random_seed = request.random_seed;
  result.diagnostics.epsilon_point = corridor_policy.epsilon_point;
  result.diagnostics.standard_normal_quantile =
      two_sided_standard_normal_quantile(corridor_policy.epsilon_point);
  result.diagnostics.maximum_sweep_spacing_fraction =
      parameters.maximum_sweep_spacing_fraction;
  result.diagnostics.cable_sweep_margin_m =
      parameters.cable_sweep_margin_m;
  result.diagnostics.maximum_active_label_budget =
      parameters.maximum_active_labels;
  result.diagnostics.fixed_bytes_per_search_label =
      sizeof(SearchNode) + sizeof(MechanicalLabel) + sizeof(OpenEntry);
  result.diagnostics.analytic_expansion_interval =
      parameters.analytic_expansion_interval;
  result.diagnostics.equivalent_label_cost_tolerance_m =
      parameters.equivalent_label_cost_tolerance_m;
  result.diagnostics.maximum_planning_duration_s =
      parameters.maximum_planning_duration_s;
  result.diagnostics.goal_touchdown_position_tolerance_m =
      parameters.goal_touchdown_position_tolerance_m;
  result.diagnostics.minimum_turning_radius_m =
      parameters.minimum_turning_radius_m;
  result.diagnostics.path_length_cost_weight =
      parameters.path_length_cost_weight;
  result.diagnostics.path_curvature_cost_weight =
      parameters.path_curvature_cost_weight;
  result.diagnostics.touchdown_center_cost_weight =
      parameters.touchdown_center_cost_weight;
  result.diagnostics.touchdown_margin_cost_weight =
      parameters.touchdown_margin_cost_weight;
  result.diagnostics.robot_terrain_cost_weight =
      parameters.robot_terrain_cost_weight;
  result.diagnostics.path_dependent_covariance_propagated = false;
  result.diagnostics.operating_domain_id = model.operating_domain_id;
  result.diagnostics.risk_semantics = kPointwiseEnvelopeRiskSemantics;
  result.diagnostics.queue_rule =
      "f_cost,heuristic,path_cost,label_id,node_id:ascending";
}

enum class PrimitiveValidation {
  valid,
  reference_association_invalid,
  envelope_unavailable,
  corridor_violation,
};

void record_worst_constraint(HybridAStarSearchDiagnostics& diagnostics,
                             const std::string& reason,
                             const Vector2m position_m,
                             const double constraint_value,
                             const double hard_limit) {
  const double utilization =
      hard_limit > 0.0 ? std::abs(constraint_value) / hard_limit : 1.0;
  if (diagnostics.worst_constraint.recorded &&
      utilization <= diagnostics.worst_constraint.normalized_utilization) {
    return;
  }
  diagnostics.worst_constraint = {true, reason, position_m, constraint_value,
                                  hard_limit, utilization};
}

PrimitiveValidation propagate_progress_and_check_corridor(
    const SearchNode& parent, const CablePrediction& prediction,
    const HybridAStarPlanningRequest& request,
    const ReferenceProgressAssociator& associator,
    const CableCorridorRiskPolicy& corridor_policy,
    const HybridAStarSearchParameters& search_parameters,
    CableUncertaintyEnvelopeManager& envelope_manager,
    std::vector<ReferenceProgress>& progress_profile,
    std::vector<CableCorridorSearchBound>& corridor_profile,
    HybridAStarSearchDiagnostics& diagnostics) {
  if (prediction.touchdown_path.points.size() !=
          prediction.state_profile.size() ||
      prediction.touchdown_path.points.size() !=
          prediction.robot_arc_length_profile_m.size() ||
      prediction.touchdown_path.points.empty()) {
    ++diagnostics.reference_association_rejection_count;
    return PrimitiveValidation::reference_association_invalid;
  }
  ReferenceProgress progress = parent.progress;
  progress_profile.reserve(prediction.touchdown_path.points.size());
  corridor_profile.reserve(prediction.touchdown_path.points.size());
  progress_profile.push_back(progress);
  for (std::size_t index = 0U;
       index < prediction.touchdown_path.points.size(); ++index) {
    const PathPoint& touchdown = prediction.touchdown_path.points[index];
    if (index > 0U) {
      const double increment_m =
          touchdown.arc_length_m -
          prediction.touchdown_path.points[index - 1U].arc_length_m;
      const TouchdownAssociationSample sample{
          touchdown.arc_length_m,
          {touchdown.x_m, touchdown.y_m},
          touchdown.heading_rad,
          request.planning_timestamp};
      const ReferenceAssociationResult associated =
          associator.propagate_candidate(progress, sample, increment_m,
                                         request.reference_line);
      if (associated.status != ReferenceAssociationStatus::tracked ||
          !associated.context.has_value()) {
        ++diagnostics.reference_association_rejection_count;
        return PrimitiveValidation::reference_association_invalid;
      }
      progress = associated.context->progress;
      progress_profile.push_back(progress);
    }
    const EnvelopeQueryResult bound = envelope_manager.query(
        request.locked_uncertainty_envelope, progress.arc_length_m,
        request.planning_timestamp);
    ++diagnostics.envelope_query_count;
    diagnostics.envelope_discretization_margin_m = std::max(
        diagnostics.envelope_discretization_margin_m,
        bound.certified_discretization_margin_m);
    diagnostics.maximum_envelope_stddev_upper_bound_m = std::max(
        diagnostics.maximum_envelope_stddev_upper_bound_m,
        bound.lateral_stddev_upper_bound_m);
    if (bound.status != EnvelopeQueryStatus::valid) {
      ++diagnostics.envelope_unavailable_rejection_count;
      return PrimitiveValidation::envelope_unavailable;
    }
    const std::optional<ReferencePoint> reference =
        request.reference_line.query(progress.arc_length_m);
    if (!reference.has_value()) {
      ++diagnostics.reference_association_rejection_count;
      return PrimitiveValidation::reference_association_invalid;
    }
    const CableCorridorSearchBound corridor_bound =
        evaluate_search_corridor_bound(
            corridor_policy, *reference, {touchdown.x_m, touchdown.y_m},
            bound.lateral_stddev_upper_bound_m,
            search_parameters.cable_sweep_margin_m);
    if (corridor_bound.validity != CorridorEvaluationValidity::valid ||
        !corridor_bound.hard_feasible) {
      ++diagnostics.corridor_rejection_count;
      record_worst_constraint(
          diagnostics, "cable_corridor", {touchdown.x_m, touchdown.y_m},
          corridor_bound.validity == CorridorEvaluationValidity::valid
              ? corridor_bound.upper_bound_m
              : 1.0,
          corridor_bound.validity == CorridorEvaluationValidity::valid
              ? corridor_policy.absolute_half_width_m
              : 1.0);
      return PrimitiveValidation::corridor_violation;
    }
    corridor_profile.push_back(corridor_bound);
  }
  return PrimitiveValidation::valid;
}

double touchdown_corridor_cost(
    const std::vector<double>& robot_arc_length_profile_m,
    const std::vector<CableCorridorSearchBound>& corridor_profile,
    const HybridAStarSearchParameters& parameters,
    const CableCorridorRiskPolicy& policy) {
  double cost = 0.0;
  for (std::size_t index = 1U; index < corridor_profile.size(); ++index) {
    const CableCorridorSearchBound& left = corridor_profile[index - 1U];
    const CableCorridorSearchBound& right = corridor_profile[index];
    const double mean_lateral_error_m =
        0.5 * (left.mean_lateral_error_m + right.mean_lateral_error_m);
    const double upper_bound_m =
        0.5 * (left.upper_bound_m + right.upper_bound_m);
    double density = 0.0;
    if (upper_bound_m < policy.nominal_half_width_m) {
      density = parameters.touchdown_center_cost_weight *
                mean_lateral_error_m * mean_lateral_error_m;
    } else {
      const double normalized_margin =
          (upper_bound_m - policy.nominal_half_width_m) /
          (policy.absolute_half_width_m - policy.nominal_half_width_m);
      density = parameters.touchdown_margin_cost_weight * normalized_margin *
                normalized_margin;
    }
    const double interval_length_m =
        robot_arc_length_profile_m[index] -
        robot_arc_length_profile_m[index - 1U];
    cost += density * interval_length_m;
  }
  return cost;
}

double robot_terrain_cost(const Pose2d& start,
                          const HybridAStarMotionPrimitive& primitive,
                          const TerrainLayers& terrain,
                          const HybridAStarSearchParameters& parameters) {
  const PathPoint midpoint =
      point_on_primitive(start, primitive, 0.5 * primitive.arc_length_m);
  const auto column = static_cast<std::size_t>(std::floor(
      (midpoint.x_m - terrain.surface.origin_x_m) /
      terrain.surface.resolution_m));
  const auto row = static_cast<std::size_t>(std::floor(
      (midpoint.y_m - terrain.surface.origin_y_m) /
      terrain.surface.resolution_m));
  return parameters.robot_terrain_cost_weight *
         terrain.surface.at(row, column).detrended_roughness_rms_m *
         primitive.arc_length_m;
}

void reconstruct_result(const std::vector<SearchNode>& nodes,
                        const std::size_t goal_id,
                        const HybridAStarPlanningRequest& request,
                        const HybridAStarSearchParameters& parameters,
                        const CableModel& cable_model,
                        HybridAStarPlanningResult& result) {
  std::vector<std::size_t> chain;
  for (std::size_t id = goal_id;; id = nodes[id].parent_id) {
    chain.push_back(id);
    if (id == 0U) break;
  }
  std::reverse(chain.begin(), chain.end());

  result.robot_path.metadata = {parameters.path_version,
                                request.reference_line.coordinate_frame,
                                request.reference_line.version,
                                "constant-curvature-exact"};
  result.touchdown_path.metadata = {
      parameters.path_version, request.reference_line.coordinate_frame,
      request.reference_line.version, "cable-mean-spatial-lag"};
  const CableMeanSample initial_touchdown = cable_model.predict_touchdown_mean(
      request.start_state.pose, request.initial_cable_state.lag_angle_rad);
  result.robot_path.points.push_back(
      {0.0, request.start_state.pose.x_m, request.start_state.pose.y_m,
       request.start_state.pose.heading_rad,
       chain.size() > 1U
           ? nodes[chain[1U]].incoming_robot_segment.points.front()
                 .curvature_per_m
           : request.start_state.curvature_per_m});
  result.touchdown_path.points.push_back(
      {0.0, initial_touchdown.touchdown_position_m.x_m,
       initial_touchdown.touchdown_position_m.y_m,
       initial_touchdown.cable_heading_rad, 0.0});
  result.state_trace.push_back(
      {nodes[0].base_key, nodes[0].robot_pose, nodes[0].cable_state.lag_angle_rad,
       nodes[0].progress});

  double accumulated_arc_length_m = 0.0;
  for (std::size_t chain_index = 1U; chain_index < chain.size(); ++chain_index) {
    const SearchNode& node = nodes[chain[chain_index]];
    const GeometricPath& robot = node.incoming_robot_segment;
    const CablePrediction& cable = node.incoming_cable_prediction;
    const double primitive_length_m = robot.points.back().arc_length_m;
    const double primitive_start_arc_length_m = accumulated_arc_length_m;
    accumulated_arc_length_m += primitive_length_m;
    for (std::size_t index = 1U; index < robot.points.size(); ++index) {
      PathPoint robot_sample = robot.points[index];
      robot_sample.arc_length_m += primitive_start_arc_length_m;
      result.robot_path.points.push_back(robot_sample);
    }
    for (std::size_t index = 1U; index < cable.touchdown_path.points.size();
         ++index) {
      PathPoint touchdown = cable.touchdown_path.points[index];
      touchdown.arc_length_m += accumulated_arc_length_m - primitive_length_m;
      result.touchdown_path.points.push_back(touchdown);
    }
    result.state_trace.push_back(
        {node.base_key, node.robot_pose, node.cable_state.lag_angle_rad,
         node.progress});
  }
  result.terminal_cable_state = nodes[goal_id].cable_state;
  result.terminal_reference_progress = nodes[goal_id].progress;
}

}  // namespace

bool operator<(const HybridAStarBaseKey& left,
               const HybridAStarBaseKey& right) noexcept {
  return std::tie(left.x_index, left.y_index, left.heading_index,
                  left.cable_lag_index, left.reference_progress_index) <
         std::tie(right.x_index, right.y_index, right.heading_index,
                  right.cable_lag_index, right.reference_progress_index);
}

HybridAStarPlanner::HybridAStarPlanner(
    CableModelParameters cable_model_parameters,
    ReferenceProgressAssociationParameters progress_parameters,
    HybridAStarSearchParameters search_parameters,
    CableCorridorRiskPolicy corridor_risk_policy,
    CableUncertaintyEnvelopeManager& envelope_manager,
    HybridAStarSteadyClock clock)
    : cable_model_(std::move(cable_model_parameters)),
      progress_associator_(std::move(progress_parameters)),
      search_parameters_(std::move(search_parameters)),
      corridor_risk_policy_(std::move(corridor_risk_policy)),
      envelope_manager_(envelope_manager),
      clock_(std::move(clock)) {
  if (!valid_parameters(search_parameters_) ||
      !clock_ ||
      validate_search_corridor_risk_policy(corridor_risk_policy_) !=
          CorridorEvaluationValidity::valid) {
    throw std::invalid_argument(
        "Hybrid A* requires valid versioned search and corridor parameters");
  }
}

HybridAStarPlanningResult HybridAStarPlanner::plan(
    const HybridAStarPlanningRequest& request) const {
  HybridAStarPlanningResult result;
  const CableModelIdentity model = cable_model_.identity();
  initialize_diagnostics(result, request, model, search_parameters_,
                         corridor_risk_policy_);
  const RequestValidation validation = validate_request(
      request, model, search_parameters_, corridor_risk_policy_);
  if (validation != RequestValidation::valid) {
    const bool dependency_mismatch =
        validation == RequestValidation::dependency_mismatch;
    result.diagnostics.entries.push_back(diagnostic(
        DiagnosticSeverity::error,
        dependency_mismatch ? "HYBRID_ASTAR_DEPENDENCY_MISMATCH"
                            : "HYBRID_ASTAR_INPUT_INVALID",
        dependency_mismatch
            ? "search inputs do not match the locked dependency versions"
            : "search input structure or values are invalid",
        request.planning_timestamp));
    finalize_fingerprint(result);
    return result;
  }
  const EnvelopeQueryResult initial_bound = envelope_manager_.query(
      request.locked_uncertainty_envelope,
      request.initial_reference_progress.arc_length_m,
      request.planning_timestamp);
  ++result.diagnostics.envelope_query_count;
  result.diagnostics.envelope_discretization_margin_m =
      initial_bound.certified_discretization_margin_m;
  result.diagnostics.maximum_envelope_stddev_upper_bound_m =
      initial_bound.lateral_stddev_upper_bound_m;
  if (initial_bound.status != EnvelopeQueryStatus::valid) {
    result.state = PlanningState::no_solution_under_covariance_envelope;
    result.diagnostics.entries.push_back(diagnostic(
        DiagnosticSeverity::error, "SEARCH_ENVELOPE_UNAVAILABLE",
        "the locked statistical envelope is unavailable at search start",
        request.planning_timestamp));
    finalize_fingerprint(result);
    return result;
  }
  const CableMeanSample initial_touchdown = cable_model_.predict_touchdown_mean(
      request.start_state.pose, request.initial_cable_state.lag_angle_rad);
  if (initial_touchdown.validity != CableModelValidity::valid) {
    result.state = PlanningState::input_invalid;
    result.diagnostics.entries.push_back(diagnostic(
        DiagnosticSeverity::error,
        "HYBRID_ASTAR_INITIAL_CABLE_STATE_OUT_OF_RANGE",
        "the initial cable state is outside the calibrated mean-model domain",
        request.planning_timestamp));
    finalize_fingerprint(result);
    return result;
  }
  const std::optional<ReferencePoint> initial_reference =
      request.reference_line.query(
          request.initial_reference_progress.arc_length_m);
  const CableCorridorSearchBound initial_corridor_bound =
      initial_reference.has_value()
          ? evaluate_search_corridor_bound(
                corridor_risk_policy_, *initial_reference,
                initial_touchdown.touchdown_position_m,
                initial_bound.lateral_stddev_upper_bound_m,
                search_parameters_.cable_sweep_margin_m)
          : CableCorridorSearchBound{};
  if (initial_corridor_bound.validity != CorridorEvaluationValidity::valid ||
      !initial_corridor_bound.hard_feasible) {
    result.state = PlanningState::no_solution_under_covariance_envelope;
    ++result.diagnostics.corridor_rejection_count;
    record_worst_constraint(
        result.diagnostics, "cable_corridor",
        initial_touchdown.touchdown_position_m,
        initial_corridor_bound.validity == CorridorEvaluationValidity::valid
            ? initial_corridor_bound.upper_bound_m
            : 1.0,
        initial_corridor_bound.validity == CorridorEvaluationValidity::valid
            ? corridor_risk_policy_.absolute_half_width_m
            : 1.0);
    result.diagnostics.entries.push_back(diagnostic(
        DiagnosticSeverity::error,
        "NO_SOLUTION_UNDER_COVARIANCE_ENVELOPE",
        "the initial touchdown mean violates the locked search corridor",
        request.planning_timestamp));
    finalize_fingerprint(result);
    return result;
  }

  const HybridAStarPrimitiveSweepContext& sweep_context =
      request.primitive_sweep_context;
  const TraversabilityEvaluator traversability_evaluator(
      sweep_context.robot_capability, sweep_context.track_footprint);
  const CollisionLayerResult collision_layer =
      traversability_evaluator.evaluate_collision_layer(
          sweep_context.map, sweep_context.terrain,
          sweep_context.robot_relative_obstacle_covariance_m2,
          sweep_context.collision_risk_policy);
  if (collision_layer.validity != CollisionEvaluationValidity::valid) {
    result.state = PlanningState::input_invalid;
    result.diagnostics.entries.push_back(diagnostic(
        DiagnosticSeverity::error, "HYBRID_ASTAR_COLLISION_LAYER_INVALID",
        "the versioned collision layer could not be constructed",
        request.planning_timestamp));
    finalize_fingerprint(result);
    return result;
  }
  const MotionSegment initial_robot_sweep{{request.start_state.pose}};
  const double robot_sweep_discretization_margin_m =
      0.5 * search_parameters_.maximum_sweep_spacing_fraction *
      sweep_context.map.resolution_m;
  const CollisionSweepResult initial_collision =
      traversability_evaluator.evaluate_collision_sweep(
          initial_robot_sweep, sweep_context.terrain, collision_layer,
          search_parameters_.maximum_sweep_spacing_fraction);
  const TraversabilityResult initial_traversability =
      traversability_evaluator.evaluate(
          initial_robot_sweep, sweep_context.terrain,
          sweep_context.terrain_gradient_risk_policy);
  result.diagnostics.maximum_robot_sweep_pose_count =
      initial_collision.evaluated_sweep_poses;
  result.diagnostics.maximum_robot_sweep_spacing_m =
      initial_collision.maximum_boundary_displacement_m;
  result.diagnostics.maximum_collision_sweep_margin_m =
      initial_collision.sweep_discretization_margin_m;
  if (initial_collision.validity != CollisionEvaluationValidity::valid ||
      initial_traversability.validity !=
          TraversabilityEvaluationValidity::valid) {
    result.state = PlanningState::input_invalid;
    result.diagnostics.entries.push_back(diagnostic(
        DiagnosticSeverity::error, "HYBRID_ASTAR_SWEEP_CONTEXT_INVALID",
        "the primitive sweep geometry, terrain, or risk policy is invalid",
        request.planning_timestamp));
    finalize_fingerprint(result);
    return result;
  }
  const std::optional<Pose2d> initial_operating_area_failure =
      operating_area_violation(
          initial_robot_sweep, sweep_context.track_footprint,
          sweep_context.robot_operating_area,
          robot_sweep_discretization_margin_m);
  if (initial_operating_area_failure.has_value() ||
      !initial_collision.collision_free || !initial_traversability.traversable) {
    result.state = PlanningState::no_solution;
    if (initial_operating_area_failure.has_value()) {
      record_worst_constraint(
          result.diagnostics, "robot_operating_area",
          {initial_operating_area_failure->x_m,
           initial_operating_area_failure->y_m},
          1.0, 1.0);
    } else if (!initial_collision.collision_free) {
      record_worst_constraint(
          result.diagnostics, "robot_collision",
          {request.start_state.pose.x_m, request.start_state.pose.y_m}, 1.0,
          1.0);
    } else {
      record_worst_constraint(
          result.diagnostics, "robot_traversability",
          {request.start_state.pose.x_m, request.start_state.pose.y_m}, 1.0,
          1.0);
    }
    result.diagnostics.entries.push_back(diagnostic(
        DiagnosticSeverity::error, "HYBRID_ASTAR_INITIAL_SWEEP_INFEASIBLE",
        "the initial full footprint violates a hard sweep constraint",
        request.planning_timestamp));
    finalize_fingerprint(result);
    return result;
  }

  const CableLayingEvaluator cable_laying_evaluator;
  const std::optional<CableConstraintMemory> initial_memory =
      cable_laying_evaluator.canonicalize_memory(
          request.initial_cable_state.laying_memory,
          sweep_context.cable_laying_limits,
          sweep_context.cable_history_boundary);
  if (!initial_memory.has_value()) {
    result.state = PlanningState::input_invalid;
    result.diagnostics.entries.push_back(diagnostic(
        DiagnosticSeverity::error,
        "HYBRID_ASTAR_INITIAL_MECHANICAL_HISTORY_INVALID",
        "the initial cable mechanical history cannot be canonicalized",
        request.planning_timestamp));
    finalize_fingerprint(result);
    return result;
  }

  SearchNode start;
  start.id = 0U;
  start.label_id = 0U;
  start.robot_pose = request.start_state.pose;
  start.cable_state = request.initial_cable_state;
  start.cable_state.kind = CableStateKind::search_mean;
  start.cable_state.lag_angle_variance_rad2.reset();
  start.cable_state.laying_memory = *initial_memory;
  start.progress = request.initial_reference_progress;
  start.base_key = discretize(start.robot_pose, start.cable_state.lag_angle_rad,
                              start.progress, search_parameters_);
  start.heuristic_m =
      heuristic(start.robot_pose, request.goals, search_parameters_);
  result.diagnostics.initial_heuristic_cost = start.heuristic_m;

  std::vector<SearchNode> nodes;
  nodes.push_back(start);
  observe_label_storage(nodes.back(), result.diagnostics);
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryLater> open;
  open.push({start.heuristic_m, start.heuristic_m, 0.0, start.label_id,
             start.id});
  std::vector<MechanicalLabel> labels{{start.id, start.path_cost_m}};
  std::map<HybridAStarBaseKey, BaseKeyLabels> labels_by_key;
  BaseKeyLabels& start_labels = labels_by_key[start.base_key];
  start_labels.by_signature[start.cable_state.laying_memory.canonical_signature]
      .push_back(start.label_id);
  start_labels.label_count = 1U;
  result.diagnostics.active_label_count = 1U;
  result.diagnostics.peak_active_label_count = 1U;
  result.diagnostics.maximum_labels_per_base_key = 1U;
  bool rejected_by_envelope = false;
  const std::chrono::steady_clock::time_point search_started_at = clock_();
  const auto deadline_exceeded = [&]() {
    return std::chrono::duration<double>(clock_() - search_started_at).count() >=
           search_parameters_.maximum_planning_duration_s;
  };
  bool reached_deadline = false;

  while (!open.empty() &&
         result.diagnostics.expanded_state_count <
             search_parameters_.maximum_expansions) {
    if (deadline_exceeded()) {
      reached_deadline = true;
      break;
    }
    const OpenEntry entry = open.top();
    open.pop();
    if (entry.label_id >= labels.size() ||
        labels[entry.label_id].active_node_id != entry.node_id) {
      ++result.diagnostics.stale_queue_entry_count;
      continue;
    }
    const SearchNode current = nodes[entry.node_id];
    ++result.diagnostics.expanded_state_count;
    const CableMeanSample current_touchdown =
        cable_model_.predict_touchdown_mean(
            current.robot_pose, current.cable_state.lag_angle_rad);
    for (const MergeGoal& goal : request.goals) {
      if (matches_goal(current.robot_pose, current.cable_state.lag_angle_rad,
                       current.progress, current_touchdown, goal,
                       search_parameters_)) {
        result.state = PlanningState::success;
        result.diagnostics.solution_cost = current.path_cost_m;
        result.diagnostics.solution_cost_components = current.cost_components;
        reconstruct_result(nodes, current.id, request, search_parameters_,
                           cable_model_, result);
        result.diagnostics.entries.push_back(diagnostic(
            DiagnosticSeverity::info, "HYBRID_ASTAR_PATH_FOUND",
            "deterministic augmented-state search reached a merge goal",
            request.planning_timestamp));
        update_label_distribution(labels_by_key, result.diagnostics);
        finalize_fingerprint(result);
        return result;
      }
    }

    struct ExpansionCandidate {
      HybridAStarMotionPrimitive primitive;
      bool analytic{};
    };
    std::vector<ExpansionCandidate> expansion_candidates;
    expansion_candidates.reserve(search_parameters_.motion_primitives.size() +
                                 request.goals.size());
    for (const HybridAStarMotionPrimitive& primitive :
         search_parameters_.motion_primitives) {
      expansion_candidates.push_back({primitive, false});
    }
    if (search_parameters_.analytic_expansion_interval > 0U &&
        result.diagnostics.expanded_state_count %
                search_parameters_.analytic_expansion_interval ==
            0U) {
      for (std::size_t goal_index = 0U; goal_index < request.goals.size();
           ++goal_index) {
        ++result.diagnostics.analytic_expansion_attempt_count;
        const auto analytic = next_dubins_primitive(
            current.robot_pose, request.goals[goal_index],
            search_parameters_.primitive_set_version,
            search_parameters_.minimum_turning_radius_m);
        if (analytic.has_value()) {
          expansion_candidates.push_back({*analytic, true});
        }
      }
    }

    for (const ExpansionCandidate& expansion : expansion_candidates) {
      const HybridAStarMotionPrimitive& primitive = expansion.primitive;
      if (deadline_exceeded()) {
        reached_deadline = true;
        break;
      }
      ++result.diagnostics.generated_successor_count;
      const std::uint64_t path_version =
          search_parameters_.path_version +
          static_cast<std::uint64_t>(result.diagnostics.generated_successor_count);
      const GeometricPath robot_segment = apply_motion(
          current.robot_pose, primitive, path_version,
          request.reference_line.version,
          request.reference_line.coordinate_frame,
          sweep_context.map.resolution_m,
          track_footprint_radius(sweep_context.track_footprint),
          search_parameters_.maximum_sweep_spacing_fraction);
      const MotionSegment robot_sweep =
          motion_segment(robot_segment, request.planning_timestamp);
      const std::optional<Pose2d> operating_area_failure =
          operating_area_violation(
              robot_sweep, sweep_context.track_footprint,
              sweep_context.robot_operating_area,
              robot_sweep_discretization_margin_m);
      if (operating_area_failure.has_value()) {
        ++result.diagnostics.operating_area_rejection_count;
        record_worst_constraint(result.diagnostics, "robot_operating_area",
                                {operating_area_failure->x_m,
                                 operating_area_failure->y_m},
                                1.0, 1.0);
        continue;
      }
      const CollisionSweepResult collision =
          traversability_evaluator.evaluate_collision_sweep(
              robot_sweep, sweep_context.terrain, collision_layer,
              search_parameters_.maximum_sweep_spacing_fraction);
      result.diagnostics.maximum_robot_sweep_pose_count =
          std::max(result.diagnostics.maximum_robot_sweep_pose_count,
                   collision.evaluated_sweep_poses);
      result.diagnostics.maximum_robot_sweep_spacing_m =
          std::max(result.diagnostics.maximum_robot_sweep_spacing_m,
                   collision.maximum_boundary_displacement_m);
      result.diagnostics.maximum_collision_sweep_margin_m =
          std::max(result.diagnostics.maximum_collision_sweep_margin_m,
                   collision.sweep_discretization_margin_m);
      if (collision.validity != CollisionEvaluationValidity::valid ||
          !collision.collision_free) {
        ++result.diagnostics.collision_rejection_count;
        Pose2d position = robot_sweep.samples.back();
        for (const Pose2d& pose : robot_sweep.samples) {
          const CollisionSweepResult focused =
              traversability_evaluator.evaluate_collision_sweep(
                  MotionSegment{{pose}}, sweep_context.terrain,
                  collision_layer,
                  search_parameters_.maximum_sweep_spacing_fraction);
          if (focused.validity != CollisionEvaluationValidity::valid ||
              !focused.collision_free) {
            position = pose;
            break;
          }
        }
        record_worst_constraint(result.diagnostics, "robot_collision",
                                {position.x_m, position.y_m}, 1.0, 1.0);
        continue;
      }
      const TraversabilityResult traversability = traversability_evaluator.evaluate(
          robot_sweep, sweep_context.terrain,
          sweep_context.terrain_gradient_risk_policy);
      if (traversability.validity != TraversabilityEvaluationValidity::valid ||
          !traversability.traversable) {
        ++result.diagnostics.traversability_rejection_count;
        Pose2d position = robot_sweep.samples.back();
        TraversabilityResult failing_sample = traversability;
        for (const Pose2d& pose : robot_sweep.samples) {
          const TraversabilityResult focused = traversability_evaluator.evaluate(
              MotionSegment{{pose}}, sweep_context.terrain,
              sweep_context.terrain_gradient_risk_policy);
          if (focused.validity != TraversabilityEvaluationValidity::valid ||
              !focused.traversable) {
            position = pose;
            failing_sample = focused;
            break;
          }
        }
        const double up_utilization =
            std::abs(failing_sample.maximum_longitudinal_upper_angle_rad) /
            sweep_context.robot_capability.maximum_slope_up_rad;
        const double down_utilization =
            std::abs(failing_sample.minimum_longitudinal_lower_angle_rad) /
            sweep_context.robot_capability.maximum_slope_down_rad;
        const double lateral_utilization =
            failing_sample.maximum_lateral_absolute_upper_angle_rad /
            sweep_context.robot_capability.maximum_slope_lateral_rad;
        double worst_angle_rad = 1.0;
        double angle_limit_rad = 1.0;
        if (up_utilization >= 1.0 && up_utilization >= down_utilization &&
            up_utilization >= lateral_utilization) {
          worst_angle_rad =
              std::abs(failing_sample.maximum_longitudinal_upper_angle_rad);
          angle_limit_rad =
              sweep_context.robot_capability.maximum_slope_up_rad;
        } else if (down_utilization >= 1.0 &&
                   down_utilization >= lateral_utilization) {
          worst_angle_rad =
              std::abs(failing_sample.minimum_longitudinal_lower_angle_rad);
          angle_limit_rad =
              sweep_context.robot_capability.maximum_slope_down_rad;
        } else if (lateral_utilization >= 1.0) {
          worst_angle_rad =
              failing_sample.maximum_lateral_absolute_upper_angle_rad;
          angle_limit_rad =
              sweep_context.robot_capability.maximum_slope_lateral_rad;
        }
        record_worst_constraint(result.diagnostics, "robot_traversability",
                                {position.x_m, position.y_m}, worst_angle_rad,
                                angle_limit_rad);
        continue;
      }
      const CablePrediction cable_prediction = cable_model_.predict_search(
          current.cable_state, robot_segment, request.cable_context);
      if (cable_prediction.validity != CableModelValidity::valid) {
        ++result.diagnostics.cable_model_rejection_count;
        continue;
      }

      std::vector<ReferenceProgress> progress_profile;
      std::vector<CableCorridorSearchBound> corridor_profile;
      const PrimitiveValidation primitive_validation =
          propagate_progress_and_check_corridor(
              current, cable_prediction, request, progress_associator_,
               corridor_risk_policy_, search_parameters_, envelope_manager_,
               progress_profile, corridor_profile, result.diagnostics);
      if (primitive_validation != PrimitiveValidation::valid) {
        rejected_by_envelope =
            rejected_by_envelope ||
            primitive_validation == PrimitiveValidation::envelope_unavailable ||
            primitive_validation == PrimitiveValidation::corridor_violation;
        continue;
      }
      const CableLayingEvaluation laying =
          cable_laying_evaluator.evaluate_segment(
              current.cable_state.laying_memory,
              cable_prediction.touchdown_path, cable_prediction.state_profile,
              sweep_context.terrain, sweep_context.cable_laying_limits,
              sweep_context.cable_history_boundary);
      if (!laying.valid || !laying.hard_feasible) {
        ++result.diagnostics.cable_laying_rejection_count;
        const Vector2m terminal_position{
            cable_prediction.touchdown_path.points.back().x_m,
            cable_prediction.touchdown_path.points.back().y_m};
        const Vector2m fallback_position =
            laying.failure_segments.empty()
                ? terminal_position
                : laying.failure_segments.front().representative_position_m;
        const double curvature_utilization =
            laying.maximum_absolute_curvature_per_m /
            sweep_context.cable_laying_limits.maximum_curvature_per_m;
        const double support_utilization =
            laying.maximum_support_proxy_range_m /
            sweep_context.cable_laying_limits.maximum_support_proxy_range_m;
        if (curvature_utilization >= 1.0 &&
            curvature_utilization >= support_utilization) {
          record_worst_constraint(
              result.diagnostics, "cable_laying",
              laying.maximum_absolute_curvature_position_m.value_or(
                  fallback_position),
              laying.maximum_absolute_curvature_per_m,
              sweep_context.cable_laying_limits.maximum_curvature_per_m);
        } else if (support_utilization >= 1.0) {
          record_worst_constraint(
              result.diagnostics, "cable_laying",
              laying.maximum_support_proxy_position_m.value_or(
                  fallback_position),
              laying.maximum_support_proxy_range_m,
              sweep_context.cable_laying_limits.maximum_support_proxy_range_m);
        } else {
          record_worst_constraint(
              result.diagnostics, "cable_laying", fallback_position, 1.0,
              1.0);
        }
        continue;
      }
      result.diagnostics.maximum_primitive_laying_soft_cost =
          std::max(result.diagnostics.maximum_primitive_laying_soft_cost,
                   laying.soft_cost);
      SearchNode successor;
      successor.id = nodes.size();
      successor.robot_pose = {
          robot_segment.points.back().x_m, robot_segment.points.back().y_m,
          robot_segment.points.back().heading_rad, request.planning_timestamp};
      successor.cable_state = cable_prediction.terminal_state;
      successor.cable_state.laying_memory = laying.terminal_memory;
      successor.progress = progress_profile.back();
      successor.base_key = discretize(
          successor.robot_pose, successor.cable_state.lag_angle_rad,
          successor.progress, search_parameters_);
      successor.cost_components = current.cost_components;
      successor.cost_components.robot_length +=
          search_parameters_.path_length_cost_weight * primitive.arc_length_m;
      successor.cost_components.robot_curvature +=
          search_parameters_.path_curvature_cost_weight *
          std::abs(primitive.curvature_per_m) * primitive.arc_length_m;
      successor.cost_components.touchdown_corridor += touchdown_corridor_cost(
          cable_prediction.robot_arc_length_profile_m, corridor_profile,
          search_parameters_, corridor_risk_policy_);
      successor.cost_components.cable_suitability += laying.soft_cost;
      successor.cost_components.robot_terrain += robot_terrain_cost(
          current.robot_pose, primitive, sweep_context.terrain,
          search_parameters_);
      successor.path_cost_m = successor.cost_components.total();
      successor.heuristic_m =
          heuristic(successor.robot_pose, request.goals, search_parameters_);
      successor.parent_id = current.id;
      successor.incoming_robot_segment = robot_segment;
      successor.incoming_cable_prediction = cable_prediction;
      if (expansion.analytic) {
        ++result.diagnostics.analytic_expansion_accepted_count;
      }

      // The signature narrows candidates; sample equality proves equivalence.
      const std::uint64_t signature =
          successor.cable_state.laying_memory.canonical_signature;
      std::optional<std::uint64_t> equivalent_label_id;
      const auto base_labels_it = labels_by_key.find(successor.base_key);
      if (base_labels_it != labels_by_key.end()) {
        const auto compare_candidates = [&](const auto& candidates) {
          for (const std::uint64_t label_id : candidates) {
            const SearchNode& active = nodes[labels[label_id].active_node_id];
            if (cable_laying_evaluator.future_equivalent(
                    active.cable_state.laying_memory,
                    successor.cable_state.laying_memory)) {
              equivalent_label_id = label_id;
              return;
            }
          }
        };
        const auto signature_it =
            base_labels_it->second.by_signature.find(signature);
        if (signature_it != base_labels_it->second.by_signature.end()) {
          compare_candidates(signature_it->second);
        }
        for (const auto& [candidate_signature, candidates] :
             base_labels_it->second.by_signature) {
          if (equivalent_label_id.has_value()) break;
          if (candidate_signature == signature) continue;
          result.diagnostics.signature_fallback_comparison_count +=
              candidates.size();
          compare_candidates(candidates);
        }
      }
      if (equivalent_label_id.has_value()) {
        MechanicalLabel& equivalent = labels[*equivalent_label_id];
        if (equivalent.path_cost_m <=
            successor.path_cost_m +
                search_parameters_.equivalent_label_cost_tolerance_m) {
          ++result.diagnostics.equivalent_label_discard_count;
          continue;
        }
        successor.label_id = *equivalent_label_id;
        nodes.push_back(std::move(successor));
        const SearchNode& inserted = nodes.back();
        observe_label_storage(inserted, result.diagnostics);
        equivalent.active_node_id = inserted.id;
        equivalent.path_cost_m = inserted.path_cost_m;
        ++result.diagnostics.equivalent_label_replacement_count;
        open.push({inserted.path_cost_m + inserted.heuristic_m,
                   inserted.heuristic_m, inserted.path_cost_m,
                   inserted.label_id, inserted.id});
        continue;
      }
      if (result.diagnostics.active_label_count >=
          search_parameters_.maximum_active_labels) {
        result.state = PlanningState::timeout;
        result.diagnostics.active_label_budget_exhausted = true;
        result.diagnostics.entries.push_back(diagnostic(
            DiagnosticSeverity::error,
            "HYBRID_ASTAR_ACTIVE_LABEL_BUDGET_EXHAUSTED",
            "search could not retain another mechanically distinct label "
            "within the configured global active-label budget",
            request.planning_timestamp));
        update_label_distribution(labels_by_key, result.diagnostics);
        finalize_fingerprint(result);
        return result;
      }
      successor.label_id = static_cast<std::uint64_t>(labels.size());
      nodes.push_back(std::move(successor));
      const SearchNode& inserted = nodes.back();
      observe_label_storage(inserted, result.diagnostics);
      labels.push_back({inserted.id, inserted.path_cost_m});
      BaseKeyLabels& base_labels = labels_by_key[inserted.base_key];
      std::vector<std::uint64_t>& signature_candidates =
          base_labels.by_signature[signature];
      signature_candidates.push_back(inserted.label_id);
      ++base_labels.label_count;
      ++result.diagnostics.active_label_count;
      result.diagnostics.peak_active_label_count = std::max(
          result.diagnostics.peak_active_label_count,
          result.diagnostics.active_label_count);
      result.diagnostics.maximum_labels_per_base_key = std::max(
          result.diagnostics.maximum_labels_per_base_key,
          base_labels.label_count);
      open.push({inserted.path_cost_m + inserted.heuristic_m,
                 inserted.heuristic_m, inserted.path_cost_m,
                 inserted.label_id, inserted.id});
    }
    if (reached_deadline) break;
  }

  if (reached_deadline) {
    result.state = PlanningState::timeout;
    result.diagnostics.deadline_exceeded = true;
    update_label_distribution(labels_by_key, result.diagnostics);
    result.diagnostics.entries.push_back(diagnostic(
        DiagnosticSeverity::error, "HYBRID_ASTAR_DEADLINE_EXCEEDED",
        "search stopped after reaching the configured planning deadline",
        request.planning_timestamp));
    finalize_fingerprint(result);
    return result;
  }

  if (!open.empty() &&
      result.diagnostics.expanded_state_count >=
          search_parameters_.maximum_expansions) {
    result.state = PlanningState::timeout;
    result.diagnostics.entries.push_back(diagnostic(
        DiagnosticSeverity::error,
        "HYBRID_ASTAR_EXPANSION_BUDGET_EXHAUSTED",
        "search stopped with an active frontier after exhausting the "
        "configured expansion budget",
        request.planning_timestamp));
    update_label_distribution(labels_by_key, result.diagnostics);
    finalize_fingerprint(result);
    return result;
  }
  const bool envelope_is_only_recorded_hard_failure =
      rejected_by_envelope &&
      result.diagnostics.cable_model_rejection_count == 0U &&
      result.diagnostics.reference_association_rejection_count == 0U &&
      result.diagnostics.operating_area_rejection_count == 0U &&
      result.diagnostics.collision_rejection_count == 0U &&
      result.diagnostics.traversability_rejection_count == 0U &&
      result.diagnostics.cable_laying_rejection_count == 0U;
  result.state = envelope_is_only_recorded_hard_failure
                     ? PlanningState::no_solution_under_covariance_envelope
                     : PlanningState::no_solution;
  result.diagnostics.entries.push_back(diagnostic(
      DiagnosticSeverity::error,
      envelope_is_only_recorded_hard_failure
          ? "NO_SOLUTION_UNDER_COVARIANCE_ENVELOPE"
          : "HYBRID_ASTAR_NO_SOLUTION",
      "deterministic augmented-state search exhausted its frontier",
      request.planning_timestamp));
  update_label_distribution(labels_by_key, result.diagnostics);
  finalize_fingerprint(result);
  return result;
}

std::string serialize_hybrid_astar_search_parameters(
    const HybridAStarSearchParameters& parameters) {
  std::ostringstream output;
  output << std::setprecision(17) << parameters.version << '\n'
         << parameters.primitive_set_version << '\n'
         << parameters.path_version << '\n' << parameters.xy_resolution_m
         << '\n' << parameters.heading_resolution_rad << '\n'
         << parameters.cable_lag_resolution_rad << '\n'
         << parameters.reference_progress_resolution_m << '\n'
         << parameters.goal_position_tolerance_m << '\n'
         << parameters.goal_heading_tolerance_rad << '\n'
         << parameters.goal_lag_tolerance_rad << '\n'
         << parameters.goal_progress_tolerance_m << '\n'
         << parameters.goal_touchdown_position_tolerance_m << '\n'
         << parameters.minimum_turning_radius_m << '\n'
         << parameters.path_length_cost_weight << '\n'
         << parameters.path_curvature_cost_weight << '\n'
         << parameters.touchdown_center_cost_weight << '\n'
         << parameters.touchdown_margin_cost_weight << '\n'
         << parameters.robot_terrain_cost_weight << '\n'
         << parameters.maximum_sweep_spacing_fraction << '\n'
         << parameters.cable_sweep_margin_m << '\n'
         << parameters.equivalent_label_cost_tolerance_m << '\n'
         << parameters.maximum_planning_duration_s << '\n'
         << parameters.maximum_expansions << '\n'
         << parameters.maximum_active_labels << '\n'
         << parameters.analytic_expansion_interval << '\n'
         << parameters.motion_primitives.size() << '\n';
  for (const HybridAStarMotionPrimitive& primitive :
       parameters.motion_primitives) {
    output << primitive.version << ' ' << primitive.arc_length_m << ' '
           << primitive.curvature_per_m << '\n';
  }
  return output.str();
}

}  // namespace underwater_planner::core
