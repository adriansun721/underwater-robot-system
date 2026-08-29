#include "underwater_planner/core/path_smoother.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

using underwater_planner::core::GeometricPath;
using underwater_planner::core::PathBoundary;
using underwater_planner::core::PathBoundarySource;
using underwater_planner::core::PathPoint;
using underwater_planner::core::PathSmoother;
using underwater_planner::core::PathSmoothingSolver;
using underwater_planner::core::SmoothingSolverResult;
using underwater_planner::core::SmoothingLimits;
using underwater_planner::core::SmoothingStatus;

void require(const bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

GeometricPath straight_path() {
  GeometricPath path;
  path.points = {
      PathPoint{0.0, 0.0, 0.0, 0.0, 0.0},
      PathPoint{1.0, 1.0, 0.0, 0.0, 0.0},
      PathPoint{2.0, 2.0, 0.0, 0.0, 0.0},
  };
  path.metadata.path_version = 26U;
  path.metadata.coordinate_frame = "map";
  path.metadata.reference_line_version = 7U;
  path.metadata.interpolation_rule = "constant-curvature-exact";
  return path;
}

SmoothingLimits limits() {
  SmoothingLimits value;
  value.version = 1U;
  value.output_path_version = 2601U;
  value.spatial_step_m = 0.25;
  value.maximum_curvature_per_m = 0.5;
  value.maximum_curvature_rate_per_m2 = 0.25;
  value.minimum_segment_length_m = 0.01;
  value.topology_tube_radius_m = 0.5;
  value.timeout.nanoseconds = 100'000'000;
  value.maximum_boundary_time_skew.nanoseconds = 50;
  value.allowed_residuals.maximum_dynamics_residual = 1.0e-7;
  value.allowed_residuals.maximum_curvature_rate_residual = 1.0e-9;
  value.allowed_residuals.start_position_residual_m = 1.0e-9;
  value.allowed_residuals.start_heading_residual_rad = 1.0e-9;
  value.allowed_residuals.start_curvature_residual_per_m = 1.0e-9;
  value.allowed_residuals.goal_position_residual_m = 1.0e-5;
  value.allowed_residuals.goal_heading_residual_rad = 1.0e-5;
  value.allowed_residuals.goal_curvature_residual_per_m = 1.0e-5;
  value.objective_weights.deviation = 2.0;
  value.objective_weights.curvature = 3.0;
  value.objective_weights.curvature_rate = 4.0;
  value.objective_weights.length = 1.0;
  return value;
}

PathBoundary goal_boundary() {
  PathBoundary goal;
  goal.x_m = 2.0;
  goal.y_m = 0.0;
  goal.heading_rad = 0.0;
  goal.curvature_per_m = 0.0;
  goal.curvature_source = PathBoundarySource::planned_goal;
  return goal;
}

PathBoundary start_boundary(const double curvature_per_m = 0.0) {
  PathBoundary start;
  start.x_m = 0.0;
  start.y_m = 0.0;
  start.heading_rad = 0.0;
  start.curvature_per_m = curvature_per_m;
  start.curvature_source = PathBoundarySource::synchronized_actual_state;
  start.pose_timestamp.nanoseconds = 1'000;
  start.curvature_timestamp.nanoseconds = 1'000;
  start.source_sequence_number = 41U;
  return start;
}

void require_near(const double actual, const double expected,
                  const double tolerance, const std::string& message) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(message + ": actual=" + std::to_string(actual) +
                             " expected=" + std::to_string(expected));
  }
}

void missing_actual_curvature_is_not_replaced_with_zero() {
  // Design: 18.2.5-10
  PathBoundary start;
  start.x_m = 0.0;
  start.y_m = 0.0;
  start.heading_rad = 0.0;
  start.curvature_source = PathBoundarySource::synchronized_actual_state;

  const auto result =
      PathSmoother().smooth(straight_path(), start, goal_boundary(), limits());

  require(result.status == SmoothingStatus::boundary_state_invalid,
          "missing synchronized start curvature was not rejected");
  require(!result.path.has_value(),
          "a boundary-state failure returned a candidate path");
}

void unaudited_actual_curvature_is_not_treated_as_synchronized() {
  PathBoundary start;
  start.x_m = 0.0;
  start.y_m = 0.0;
  start.heading_rad = 0.0;
  start.curvature_per_m = 0.0;
  start.curvature_source = PathBoundarySource::synchronized_actual_state;

  const auto result =
      PathSmoother().smooth(straight_path(), start, goal_boundary(), limits());

  require(result.status == SmoothingStatus::boundary_state_invalid,
          "actual curvature without time and sequence evidence was accepted");
}

void pose_and_curvature_timestamps_must_be_synchronized() {
  // Design: 18.2.5-10
  PathBoundary start = start_boundary();
  start.curvature_timestamp.nanoseconds = 1'051;

  const auto result =
      PathSmoother().smooth(straight_path(), start, goal_boundary(), limits());

  require(result.status == SmoothingStatus::boundary_state_invalid,
          "a curvature sample outside the pose synchronization tolerance was "
          "accepted");
}

void straight_path_is_parameterized_by_one_clothoid_curve() {
  // Design: 18.2.5-invariant-1
  const auto result = PathSmoother().smooth(
      straight_path(), start_boundary(), goal_boundary(), limits());

  require(result.status == SmoothingStatus::success,
          "a feasible straight path did not smooth successfully");
  require(result.path.has_value(), "successful smoothing omitted its path");
  require(result.path->points.size() == 9U,
          "the configured spatial step did not control output sampling");
  for (std::size_t index = 0; index < result.path->points.size(); ++index) {
    const PathPoint& point = result.path->points[index];
    const double expected_x_m = 0.25 * static_cast<double>(index);
    require_near(point.x_m, expected_x_m, 1.0e-10,
                 "straight clothoid x was inconsistent");
    require_near(point.y_m, 0.0, 1.0e-10,
                 "straight clothoid y was inconsistent");
    require_near(point.heading_rad, 0.0, 1.0e-10,
                 "straight clothoid heading was inconsistent");
    require_near(point.curvature_per_m, 0.0, 1.0e-10,
                 "straight clothoid curvature was inconsistent");
  }
  require_near(result.audit.maximum_absolute_curvature_per_m, 0.0, 1.0e-12,
               "maximum curvature audit was wrong");
  require_near(result.audit.maximum_absolute_curvature_rate_per_m2, 0.0,
               1.0e-12, "maximum curvature-rate audit was wrong");
  require(result.audit.objective.reference_line_proxy == 0.0,
          "smoothing exposed a robot-center reference-line proxy cost");
  require(result.path->metadata.smoothing.has_value(),
          "successful output omitted path smoothing metadata");
  require(result.path->metadata.path_version == 2601U,
          "smoothed geometry reused the raw path version");
  require(result.path->metadata.smoothing->smoother_version ==
                  "path-smoother/clothoid-v1" &&
              result.path->metadata.smoothing->solver_status == "converged" &&
              result.path->metadata.smoothing->limits_version == 1U,
          "path smoothing identity metadata was incomplete");
  require_near(
      result.path->metadata.smoothing->maximum_absolute_curvature_per_m,
      result.audit.maximum_absolute_curvature_per_m, 1.0e-12,
      "path metadata maximum curvature disagreed with the result audit");
}

GeometricPath constant_curvature_path(const double curvature_per_m) {
  GeometricPath path = straight_path();
  path.points.clear();
  for (std::size_t index = 0; index <= 8U; ++index) {
    const double arc_length_m = 0.25 * static_cast<double>(index);
    const double heading_rad = curvature_per_m * arc_length_m;
    path.points.push_back(
        {arc_length_m, std::sin(heading_rad) / curvature_per_m,
         (1.0 - std::cos(heading_rad)) / curvature_per_m, heading_rad,
         curvature_per_m});
  }
  return path;
}

void nonzero_synchronized_curvature_is_preserved_at_both_g2_boundaries() {
  constexpr double kCurvaturePerM = 0.2;
  const GeometricPath raw = constant_curvature_path(kCurvaturePerM);
  PathBoundary goal;
  goal.x_m = raw.points.back().x_m;
  goal.y_m = raw.points.back().y_m;
  goal.heading_rad = raw.points.back().heading_rad;
  goal.curvature_per_m = kCurvaturePerM;
  goal.curvature_source = PathBoundarySource::planned_goal;

  const auto result = PathSmoother().smooth(
      raw, start_boundary(kCurvaturePerM), goal, limits());

  require(result.status == SmoothingStatus::success && result.path.has_value(),
          "a feasible nonzero-curvature G2 path was rejected");
  require_near(result.path->points.front().curvature_per_m, kCurvaturePerM,
               1.0e-12, "actual start curvature was replaced");
  require_near(result.path->points.back().curvature_per_m, kCurvaturePerM,
               1.0e-10, "goal curvature was not met");
  require_near(result.residuals.start_curvature_residual_per_m, 0.0, 1.0e-12,
               "start G2 curvature residual was wrong");
  require_near(result.residuals.goal_position_residual_m, 0.0, 1.0e-8,
               "constant-curvature goal position residual was wrong");
}

class FixedCandidateSolver final : public PathSmoothingSolver {
 public:
  explicit FixedCandidateSolver(GeometricPath candidate)
      : candidate_(std::move(candidate)) {}

  [[nodiscard]] SmoothingSolverResult solve(
      const GeometricPath&, const PathBoundary&, const PathBoundary&,
      const SmoothingLimits&) const override {
    return {SmoothingStatus::success, candidate_, 3U};
  }

 private:
  GeometricPath candidate_;
};

class TimeoutSolver final : public PathSmoothingSolver {
 public:
  [[nodiscard]] SmoothingSolverResult solve(
      const GeometricPath&, const PathBoundary&, const PathBoundary&,
      const SmoothingLimits&) const override {
    return {SmoothingStatus::solver_timeout, {}, 5U};
  }
};

void a_false_solver_convergence_cannot_bypass_residual_checks() {
  // Design: 18.2.5-3
  // Design: 18.2.5-8
  GeometricPath invalid = straight_path();
  invalid.points[1].curvature_per_m = 0.4;
  const PathSmoother smoother(
      std::make_shared<FixedCandidateSolver>(invalid));

  const auto result = smoother.smooth(
      straight_path(), start_boundary(), goal_boundary(), limits());

  require(result.status == SmoothingStatus::constraint_residual_exceeded,
          "a converged solver candidate with invalid dynamics was accepted");
  require(!result.path.has_value(),
          "a residual failure exposed the solver candidate as usable output");
  require(result.residuals.maximum_curvature_rate_residual > 0.0,
          "the independent gate did not report the curvature-rate violation");
}

GeometricPath detour_path() {
  GeometricPath path = straight_path();
  const double half_sine = std::sin(0.5);
  const double half_versine = 1.0 - std::cos(0.5);
  path.points = {
      {0.0, 0.0, 0.0, 0.0, 1.0},
      {0.5, half_sine, half_versine, 0.5, 1.0},
      {1.0, 2.0 * half_sine, 2.0 * half_versine, 0.0, -1.0},
      {1.5, 3.0 * half_sine, half_versine, -0.5, -1.0},
      {2.0, 4.0 * half_sine, 0.0, 0.0, 1.0},
  };
  return path;
}

void a_converged_candidate_cannot_switch_sides_outside_the_topology_tube() {
  GeometricPath raw = detour_path();
  GeometricPath shortcut = straight_path();
  const double shortcut_length_m = raw.points.back().x_m;
  shortcut.points = {
      {0.0, 0.0, 0.0, 0.0, 0.0},
      {0.5 * shortcut_length_m, 0.5 * shortcut_length_m, 0.0, 0.0, 0.0},
      {shortcut_length_m, shortcut_length_m, 0.0, 0.0, 0.0},
  };
  PathBoundary goal = goal_boundary();
  goal.x_m = shortcut_length_m;
  const PathSmoother smoother(
      std::make_shared<FixedCandidateSolver>(shortcut));
  SmoothingLimits constrained = limits();
  constrained.topology_tube_radius_m = 0.1;

  const auto result =
      smoother.smooth(raw, start_boundary(), goal, constrained);

  require(result.status == SmoothingStatus::trackability_validation_failed,
          "a shortcut outside the verified topology tube was accepted");
  require(!result.path.has_value(),
          "a topology failure exposed a candidate path");
}

void declared_raw_curves_define_the_topology_tube() {
  constexpr double kCurvaturePerM = 0.5;
  GeometricPath raw = straight_path();
  raw.points = {
      {0.0, 0.0, 0.0, 0.0, kCurvaturePerM},
      {1.0, 1.917702154416812 / 2.0, 0.489669752438509 / 2.0,
       kCurvaturePerM, kCurvaturePerM},
  };
  PathBoundary start = start_boundary(kCurvaturePerM);
  PathBoundary goal;
  goal.x_m = raw.points.back().x_m;
  goal.y_m = raw.points.back().y_m;
  goal.heading_rad = kCurvaturePerM;
  goal.curvature_per_m = kCurvaturePerM;
  goal.curvature_source = PathBoundarySource::planned_goal;
  SmoothingLimits narrow = limits();
  narrow.maximum_curvature_per_m = 0.6;
  narrow.spatial_step_m = 1.0;
  narrow.topology_tube_radius_m = 0.01;
  const PathSmoother smoother(std::make_shared<FixedCandidateSolver>(raw));

  const auto result = smoother.smooth(raw, start, goal, narrow);

  require(result.status == SmoothingStatus::success && result.path.has_value(),
          "a candidate on the declared raw curve was compared with its chord");
}

void verified_integration_accepts_an_analytic_long_arc() {
  constexpr double kCurvaturePerM = 1.25;
  constexpr double kLengthM = 5.0;
  const double heading_rad = kCurvaturePerM * kLengthM;
  GeometricPath exact_arc = straight_path();
  exact_arc.points = {
      {0.0, 0.0, 0.0, 0.0, kCurvaturePerM},
      {kLengthM, std::sin(heading_rad) / kCurvaturePerM,
       (1.0 - std::cos(heading_rad)) / kCurvaturePerM,
       underwater_planner::core::normalize_angle_radians(heading_rad),
       kCurvaturePerM},
  };
  PathBoundary goal;
  goal.x_m = exact_arc.points.back().x_m;
  goal.y_m = exact_arc.points.back().y_m;
  goal.heading_rad = exact_arc.points.back().heading_rad;
  goal.curvature_per_m = kCurvaturePerM;
  goal.curvature_source = PathBoundarySource::planned_goal;
  SmoothingLimits arc_limits = limits();
  arc_limits.maximum_curvature_per_m = 1.5;
  arc_limits.topology_tube_radius_m = 0.05;
  arc_limits.allowed_residuals.maximum_dynamics_residual = 1.0e-8;
  const PathSmoother smoother(
      std::make_shared<FixedCandidateSolver>(exact_arc));

  const auto result = smoother.smooth(
      exact_arc, start_boundary(kCurvaturePerM), goal, arc_limits);

  require(result.status == SmoothingStatus::success && result.path.has_value(),
          "verified integration exceeded its error bound on an analytic arc");
}

void left_and_right_curvature_profiles_are_mirror_symmetric() {
  // Design: 18.2.5-2
  constexpr double kCurvaturePerM = 0.2;
  const GeometricPath left_raw = constant_curvature_path(kCurvaturePerM);
  const GeometricPath right_raw = constant_curvature_path(-kCurvaturePerM);
  PathBoundary left_goal;
  left_goal.x_m = left_raw.points.back().x_m;
  left_goal.y_m = left_raw.points.back().y_m;
  left_goal.heading_rad = left_raw.points.back().heading_rad;
  left_goal.curvature_per_m = kCurvaturePerM;
  left_goal.curvature_source = PathBoundarySource::planned_goal;
  PathBoundary right_goal = left_goal;
  right_goal.y_m = -left_goal.y_m;
  right_goal.heading_rad = -left_goal.heading_rad;
  right_goal.curvature_per_m = -kCurvaturePerM;

  const auto left = PathSmoother().smooth(
      left_raw, start_boundary(kCurvaturePerM), left_goal, limits());
  const auto right = PathSmoother().smooth(
      right_raw, start_boundary(-kCurvaturePerM), right_goal, limits());

  require(left.status == SmoothingStatus::success && left.path.has_value() &&
              right.status == SmoothingStatus::success &&
              right.path.has_value(),
          "a feasible mirrored curvature profile was rejected");
  require(left.path->points.size() == right.path->points.size(),
          "mirrored profiles used different sample counts");
  for (std::size_t index = 0; index < left.path->points.size(); ++index) {
    require_near(left.path->points[index].x_m, right.path->points[index].x_m,
                 1.0e-10, "mirrored x coordinates diverged");
    require_near(left.path->points[index].y_m,
                 -right.path->points[index].y_m, 1.0e-10,
                 "mirrored y coordinates diverged");
    require_near(left.path->points[index].heading_rad,
                 -right.path->points[index].heading_rad, 1.0e-10,
                 "mirrored headings diverged");
    require_near(left.path->points[index].curvature_per_m,
                 -right.path->points[index].curvature_per_m, 1.0e-10,
                 "mirrored curvatures diverged");
  }

  constexpr double kOverLimitNegativeCurvaturePerM = -0.6;
  const GeometricPath over_limit =
      constant_curvature_path(kOverLimitNegativeCurvaturePerM);
  PathBoundary over_limit_goal;
  over_limit_goal.x_m = over_limit.points.back().x_m;
  over_limit_goal.y_m = over_limit.points.back().y_m;
  over_limit_goal.heading_rad = over_limit.points.back().heading_rad;
  over_limit_goal.curvature_per_m = kOverLimitNegativeCurvaturePerM;
  over_limit_goal.curvature_source = PathBoundarySource::planned_goal;
  const auto rejected = PathSmoother().smooth(
      over_limit, start_boundary(kOverLimitNegativeCurvaturePerM),
      over_limit_goal, limits());
  require(rejected.status == SmoothingStatus::seed_infeasible &&
              !rejected.path.has_value(),
          "negative curvature beyond the absolute limit was accepted");
}

void raw_path_resampling_does_not_change_the_smoothed_curve() {
  constexpr double kCurvaturePerM = 0.2;
  const GeometricPath fine = constant_curvature_path(kCurvaturePerM);
  GeometricPath coarse = fine;
  coarse.points = {fine.points[0], fine.points[4], fine.points[8]};
  PathBoundary goal;
  goal.x_m = fine.points.back().x_m;
  goal.y_m = fine.points.back().y_m;
  goal.heading_rad = fine.points.back().heading_rad;
  goal.curvature_per_m = kCurvaturePerM;
  goal.curvature_source = PathBoundarySource::planned_goal;

  const auto fine_result = PathSmoother().smooth(
      fine, start_boundary(kCurvaturePerM), goal, limits());
  const auto coarse_result = PathSmoother().smooth(
      coarse, start_boundary(kCurvaturePerM), goal, limits());

  require(fine_result.status == SmoothingStatus::success &&
              coarse_result.status == SmoothingStatus::success &&
              fine_result.path.has_value() && coarse_result.path.has_value(),
          "equivalent raw resampling changed feasibility");
  require(fine_result.path->points.size() == coarse_result.path->points.size(),
          "equivalent raw resampling changed output sample count");
  for (std::size_t index = 0; index < fine_result.path->points.size(); ++index) {
    const PathPoint& fine_point = fine_result.path->points[index];
    const PathPoint& coarse_point = coarse_result.path->points[index];
    require_near(fine_point.arc_length_m, coarse_point.arc_length_m, 1.0e-12,
                 "resampling changed output arc length");
    require_near(fine_point.x_m, coarse_point.x_m, 1.0e-12,
                 "resampling changed output x");
    require_near(fine_point.y_m, coarse_point.y_m, 1.0e-12,
                 "resampling changed output y");
    require_near(fine_point.heading_rad, coarse_point.heading_rad, 1.0e-12,
                 "resampling changed output heading");
    require_near(fine_point.curvature_per_m, coarse_point.curvature_per_m,
                 1.0e-12, "resampling changed output curvature");
  }
  require_near(fine_result.audit.maximum_absolute_curvature_per_m,
               coarse_result.audit.maximum_absolute_curvature_per_m, 1.0e-12,
               "resampling changed maximum curvature");
  require_near(fine_result.audit.maximum_absolute_curvature_rate_per_m2,
               coarse_result.audit.maximum_absolute_curvature_rate_per_m2,
               1.0e-12, "resampling changed maximum curvature rate");
}

void output_sampling_interval_does_not_change_g2_or_curvature_extrema() {
  // Design: 18.2.5-9
  constexpr double kCurvaturePerM = 0.2;
  const GeometricPath raw = constant_curvature_path(kCurvaturePerM);
  PathBoundary goal;
  goal.x_m = raw.points.back().x_m;
  goal.y_m = raw.points.back().y_m;
  goal.heading_rad = raw.points.back().heading_rad;
  goal.curvature_per_m = kCurvaturePerM;
  goal.curvature_source = PathBoundarySource::planned_goal;
  SmoothingLimits coarse_limits = limits();
  SmoothingLimits fine_limits = limits();
  fine_limits.spatial_step_m = 0.125;
  fine_limits.output_path_version = 2602U;

  const auto coarse = PathSmoother().smooth(
      raw, start_boundary(kCurvaturePerM), goal, coarse_limits);
  const auto fine = PathSmoother().smooth(
      raw, start_boundary(kCurvaturePerM), goal, fine_limits);

  require(coarse.status == SmoothingStatus::success &&
              fine.status == SmoothingStatus::success &&
              coarse.path.has_value() && fine.path.has_value(),
          "changing output sampling interval changed G2 feasibility");
  require(coarse.path->points.size() != fine.path->points.size(),
          "output sampling interval did not change output density");
  require_near(coarse.audit.maximum_absolute_curvature_per_m,
               fine.audit.maximum_absolute_curvature_per_m, 1.0e-12,
               "output sampling changed maximum curvature");
  require_near(coarse.audit.maximum_absolute_curvature_rate_per_m2,
               fine.audit.maximum_absolute_curvature_rate_per_m2, 1.0e-12,
               "output sampling changed maximum curvature rate");
  require_near(coarse.residuals.goal_position_residual_m,
               fine.residuals.goal_position_residual_m, 1.0e-8,
               "output sampling changed terminal position residual");
}

void a_committed_segment_terminal_is_an_accepted_nonzero_g2_source() {
  // Design: 18.2.5-1
  constexpr double kCurvaturePerM = 0.2;
  const GeometricPath raw = constant_curvature_path(kCurvaturePerM);
  PathBoundary committed = start_boundary(kCurvaturePerM);
  committed.curvature_source = PathBoundarySource::committed_segment_terminal;
  committed.source_sequence_number = 88U;
  PathBoundary goal;
  goal.x_m = raw.points.back().x_m;
  goal.y_m = raw.points.back().y_m;
  goal.heading_rad = raw.points.back().heading_rad;
  goal.curvature_per_m = kCurvaturePerM;
  goal.curvature_source = PathBoundarySource::planned_goal;

  const auto result = PathSmoother().smooth(raw, committed, goal, limits());

  require(result.status == SmoothingStatus::success && result.path.has_value(),
          "a versioned committed-segment G2 boundary was rejected");
  require_near(result.path->points.front().curvature_per_m, kCurvaturePerM,
               1.0e-12, "committed terminal curvature was not preserved");
}

void solver_timeout_does_not_publish_the_unvalidated_raw_path() {
  // Design: 18.2.5-invariant-3
  const PathSmoother smoother(std::make_shared<TimeoutSolver>());
  const auto result = smoother.smooth(
      straight_path(), start_boundary(), goal_boundary(), limits());

  require(result.status == SmoothingStatus::solver_timeout,
          "solver timeout was not preserved as a distinct result");
  require(!result.path.has_value(),
          "solver timeout automatically returned the raw path");
}

void default_solver_timeout_covers_objective_optimization() {
  SmoothingLimits expired = limits();
  expired.timeout.nanoseconds = 1;

  const auto result = PathSmoother().smooth(
      straight_path(), start_boundary(), goal_boundary(), expired);

  require(result.status == SmoothingStatus::solver_timeout,
          "an expired objective-optimization deadline was reported converged");
  require(!result.path.has_value(),
          "an expired objective optimization published a candidate path");
}

void translating_the_problem_does_not_add_a_reference_line_proxy_cost() {
  // Design: 18.2.5-11
  GeometricPath translated = straight_path();
  for (PathPoint& point : translated.points) {
    point.x_m += 10.0;
    point.y_m -= 3.0;
  }
  PathBoundary translated_start = start_boundary();
  translated_start.x_m += 10.0;
  translated_start.y_m -= 3.0;
  PathBoundary translated_goal = goal_boundary();
  translated_goal.x_m += 10.0;
  translated_goal.y_m -= 3.0;

  const auto baseline = PathSmoother().smooth(
      straight_path(), start_boundary(), goal_boundary(), limits());
  const auto shifted = PathSmoother().smooth(
      translated, translated_start, translated_goal, limits());

  require(baseline.status == SmoothingStatus::success &&
              shifted.status == SmoothingStatus::success,
          "translation changed smoothing feasibility");
  require_near(baseline.audit.objective.total,
               shifted.audit.objective.total, 1.0e-12,
               "translation introduced an absolute reference proxy cost");
  require(shifted.audit.objective.reference_line_proxy == 0.0,
          "translated smoothing reported a reference-line proxy cost");
}

void terminal_g2_correction_iterates_from_a_perturbed_curvature_seed() {
  constexpr double kCurvaturePerM = 0.2;
  GeometricPath raw = constant_curvature_path(kCurvaturePerM);
  const double pi = std::acos(-1.0);
  for (std::size_t index = 1U; index + 1U < raw.points.size(); ++index) {
    const double phase = pi * raw.points[index].arc_length_m /
                         raw.points.back().arc_length_m;
    raw.points[index].curvature_per_m += 0.02 * std::sin(phase);
  }
  PathBoundary goal;
  goal.x_m = raw.points.back().x_m;
  goal.y_m = raw.points.back().y_m;
  goal.heading_rad = raw.points.back().heading_rad;
  goal.curvature_per_m = kCurvaturePerM;
  goal.curvature_source = PathBoundarySource::planned_goal;

  const auto result = PathSmoother().smooth(
      raw, start_boundary(kCurvaturePerM), goal, limits());

  require(result.status == SmoothingStatus::success && result.path.has_value(),
          "the terminal G2 correction did not recover a perturbed seed");
  require(result.audit.solver_iterations > 0U,
          "the perturbed seed bypassed the terminal correction iteration");
  require(result.residuals.goal_position_residual_m <=
              limits().allowed_residuals.goal_position_residual_m &&
              result.residuals.goal_heading_residual_rad <=
                  limits().allowed_residuals.goal_heading_residual_rad &&
              result.residuals.goal_curvature_residual_per_m <=
                  limits().allowed_residuals.goal_curvature_residual_per_m,
          "terminal correction reported success outside the G2 tolerances");
}

void objective_weights_change_the_feasible_curvature_profile() {
  constexpr double kCurvaturePerM = 0.2;
  GeometricPath raw = constant_curvature_path(kCurvaturePerM);
  const double pi = std::acos(-1.0);
  for (std::size_t index = 1U; index + 1U < raw.points.size(); ++index) {
    const double phase = pi * raw.points[index].arc_length_m /
                         raw.points.back().arc_length_m;
    raw.points[index].curvature_per_m += 0.02 * std::sin(phase);
  }
  PathBoundary goal;
  goal.x_m = raw.points.back().x_m;
  goal.y_m = raw.points.back().y_m;
  goal.heading_rad = raw.points.back().heading_rad;
  goal.curvature_per_m = kCurvaturePerM;
  goal.curvature_source = PathBoundarySource::planned_goal;
  SmoothingLimits deviation_heavy = limits();
  deviation_heavy.objective_weights = {1000.0, 0.01, 0.01, 0.0};
  SmoothingLimits smoothness_heavy = limits();
  smoothness_heavy.output_path_version = 2603U;
  smoothness_heavy.objective_weights = {0.01, 100.0, 100.0, 0.0};

  const auto close_to_raw = PathSmoother().smooth(
      raw, start_boundary(kCurvaturePerM), goal, deviation_heavy);
  const auto low_curvature = PathSmoother().smooth(
      raw, start_boundary(kCurvaturePerM), goal, smoothness_heavy);

  require(close_to_raw.status == SmoothingStatus::success &&
              low_curvature.status == SmoothingStatus::success &&
              close_to_raw.path.has_value() && low_curvature.path.has_value(),
          "objective-weight comparison did not retain hard feasibility");
  double maximum_profile_difference = 0.0;
  for (std::size_t index = 0U; index < close_to_raw.path->points.size(); ++index) {
    maximum_profile_difference = std::max(
        maximum_profile_difference,
        std::abs(close_to_raw.path->points[index].curvature_per_m -
                 low_curvature.path->points[index].curvature_per_m));
  }
  require(maximum_profile_difference > 1.0e-6,
          "objective weights were reported but did not affect the solution");
}

}  // namespace

int main() {
  constexpr unsigned kSeed = 2626U;
  try {
    missing_actual_curvature_is_not_replaced_with_zero();
    unaudited_actual_curvature_is_not_treated_as_synchronized();
    pose_and_curvature_timestamps_must_be_synchronized();
    straight_path_is_parameterized_by_one_clothoid_curve();
    nonzero_synchronized_curvature_is_preserved_at_both_g2_boundaries();
    a_false_solver_convergence_cannot_bypass_residual_checks();
    a_converged_candidate_cannot_switch_sides_outside_the_topology_tube();
    declared_raw_curves_define_the_topology_tube();
    verified_integration_accepts_an_analytic_long_arc();
    left_and_right_curvature_profiles_are_mirror_symmetric();
    raw_path_resampling_does_not_change_the_smoothed_curve();
    output_sampling_interval_does_not_change_g2_or_curvature_extrema();
    a_committed_segment_terminal_is_an_accepted_nonzero_g2_source();
    solver_timeout_does_not_publish_the_unvalidated_raw_path();
    default_solver_timeout_covers_objective_optimization();
    translating_the_problem_does_not_add_a_reference_line_proxy_cost();
    terminal_g2_correction_iterates_from_a_perturbed_curvature_seed();
    objective_weights_change_the_feasible_curvature_profile();
    std::cout << "path smoother checks passed: 18"
              << " seed=" << kSeed
              << " input_version=t26-path-smoother/v1"
              << " units=SI timestamp=monotonic-ns"
              << " domain=synthetic-topology-tube/v1\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "path smoother failure: " << error.what()
              << " seed=" << kSeed
              << " input_version=t26-path-smoother/v1"
              << " units=SI timestamp=monotonic-ns"
              << " domain=synthetic-topology-tube/v1\n";
    return 1;
  }
}
