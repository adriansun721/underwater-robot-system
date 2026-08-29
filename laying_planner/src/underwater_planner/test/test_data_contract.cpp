#include "underwater_planner/core/data_contract.hpp"

#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using underwater_planner::core::PlanningResult;
using underwater_planner::core::PlanningState;
using underwater_planner::core::CableState;
using underwater_planner::core::CableStateKind;
using underwater_planner::core::GeometricPath;
using underwater_planner::core::ErrorBudget;
using underwater_planner::core::ExecutionSample;
using underwater_planner::core::ExecutionProfile;
using underwater_planner::core::ExecutionProfileVersioner;
using underwater_planner::core::SensorHealthMode;
using underwater_planner::core::TimedPath;
using underwater_planner::core::PathPoint;
using underwater_planner::core::ReferenceProgress;
using underwater_planner::core::RobotState;
using underwater_planner::core::normalize_angle_radians;
using underwater_planner::core::validate;
using underwater_planner::core::to_string;
using underwater_planner::core::deserialize_planning_result;
using underwater_planner::core::serialize_planning_result;
using underwater_planner::core::same_execution_profile_content;
using underwater_planner::core::validate_execution_profile_revision;

void require(const bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

std::uint64_t fnv1a64(const std::string_view value) {
  std::uint64_t hash = 14695981039346656037ULL;
  for (const char character : value) {
    hash ^= static_cast<unsigned char>(character);
    hash *= 1099511628211ULL;
  }
  return hash;
}

void default_constructed_contracts_are_invalid() {
  require(!validate(RobotState{}).valid,
          "a default robot state was accepted as planning input");
  require(!validate(CableState{}).valid,
          "a default cable state was accepted as planning input");
  require(!validate(ReferenceProgress{}).valid,
          "default reference progress was accepted as planning input");
  require(!validate(GeometricPath{}).valid,
          "a default geometric path was accepted as planning input");
  require(!validate(TimedPath{}).valid,
          "a default timed path was accepted as executable");
  require(!validate(ErrorBudget{}).valid,
          "a default error budget was accepted as evidence");
  require(!validate(PlanningResult{}).valid,
          "a default planning result was accepted for publication");
}

RobotState valid_robot_state() {
  RobotState state;
  state.pose = {1.0, -2.0, 0.25, {1000}};
  state.ground_speed_mps = 0.4;
  state.curvature_per_m = -0.1;
  state.curvature_timestamp = {1000};
  state.sequence_number = 7;
  return state;
}

void invalid_numeric_values_are_rejected() {
  RobotState state = valid_robot_state();
  require(validate(state).valid, "a finite robot state was rejected");

  state.ground_speed_mps = std::numeric_limits<double>::infinity();
  require(!validate(state).valid,
          "an infinite ground speed was accepted as planning input");
}

void angles_are_normalized_and_nonfinite_angles_are_rejected() {
  constexpr double kPi = 3.14159265358979323846;
  require(std::abs(normalize_angle_radians(3.0 * kPi) + kPi) < 1.0e-12,
          "positive wrapped angle did not normalize to [-pi, pi)");
  require(std::abs(normalize_angle_radians(-2.5 * kPi) + 0.5 * kPi) <
              1.0e-12,
          "negative wrapped angle did not normalize to [-pi, pi)");

  bool rejected = false;
  try {
    static_cast<void>(normalize_angle_radians(
        std::numeric_limits<double>::quiet_NaN()));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected, "a non-finite angle was silently normalized");
}

void cable_state_and_reference_progress_enforce_context() {
  CableState cable;
  cable.kind = CableStateKind::tracked;
  cable.lag_angle_rad = normalize_angle_radians(0.2);
  cable.lag_angle_variance_rad2 = 0.01;
  cable.timestamp = {1000};
  cable.sequence_number = 3;
  require(validate(cable).valid, "a finite tracked cable state was rejected");

  cable.lag_angle_variance_rad2 = -0.01;
  require(!validate(cable).valid,
          "a negative cable angle variance was accepted");

  ReferenceProgress progress;
  progress.reference_line_version = 9;
  progress.arc_length_m = 12.5;
  progress.timestamp = {1000};
  progress.sequence_number = 4;
  require(validate(progress).valid, "valid reference progress was rejected");
  progress.arc_length_m = -0.1;
  require(!validate(progress).valid,
          "negative reference progress was accepted");
}

GeometricPath valid_geometric_path() {
  GeometricPath path;
  path.metadata.path_version = 12;
  path.metadata.coordinate_frame = "world";
  path.metadata.reference_line_version = 9;
  path.metadata.interpolation_rule = "clothoid-g2/v1";
  underwater_planner::core::PathSmoothingMetadata smoothing;
  smoothing.smoother_version = "path-smoother/clothoid-v1";
  smoothing.solver_status = "converged";
  smoothing.limits_version = 17U;
  smoothing.maximum_constraint_residual = 1.0e-8;
  smoothing.maximum_absolute_curvature_per_m = 0.03;
  smoothing.maximum_absolute_curvature_rate_per_m2 = 0.05;
  smoothing.residuals.maximum_dynamics_residual = 7.0e-9;
  smoothing.residuals.maximum_curvature_audit_residual = 8.0e-9;
  smoothing.residuals.maximum_curvature_rate_residual = 9.0e-9;
  smoothing.residuals.start_position_residual_m = 1.0e-9;
  smoothing.residuals.start_heading_residual_rad = 2.0e-9;
  smoothing.residuals.start_curvature_residual_per_m = 3.0e-9;
  smoothing.residuals.goal_position_residual_m = 4.0e-9;
  smoothing.residuals.goal_heading_residual_rad = 5.0e-9;
  smoothing.residuals.goal_curvature_residual_per_m = 6.0e-9;
  path.metadata.smoothing = smoothing;
  path.points = {
      PathPoint{0.0, 1.0, 2.0, 0.1, 0.02},
      PathPoint{1.0, 2.0, 2.2, 0.2, -0.03},
  };
  return path;
}

void geometric_paths_require_finite_strictly_increasing_arc_length() {
  GeometricPath path = valid_geometric_path();
  require(validate(path).valid, "a valid geometric path was rejected");

  path.points[1].arc_length_m = path.points[0].arc_length_m;
  require(!validate(path).valid,
          "a path with repeated arc length was accepted");

  path = valid_geometric_path();
  path.points[1].heading_rad = 4.0;
  require(!validate(path).valid,
          "a path with a non-normalized heading was accepted");
}

TimedPath valid_timed_path() {
  TimedPath path;
  path.geometry = valid_geometric_path();
  path.execution_profile.version = 21;
  path.execution_profile.operating_envelope_version = 20;
  path.execution_profile.interpolation_rule = "linear-time/v1";
  path.execution_profile.stopping_point_arc_length_m = 1.0;
  path.execution_profile.samples = {
      ExecutionSample{0.0, {0}, 0.2, 0.1, 0.15, 0.02, 100.0},
      ExecutionSample{1.0, {2000000000}, 0.0, -0.1, 0.1, -0.02, 101.0},
  };
  path.execution_profile.approved_tracking_limits.ground_speed = {-0.2, 0.8};
  path.execution_profile.approved_tracking_limits.ground_acceleration = {-0.4,
                                                                          0.3};
  path.execution_profile.approved_tracking_limits
      .maximum_lateral_acceleration_mps2 = 0.25;
  path.execution_profile.approved_tracking_limits.payout_speed = {0.05, 0.5};
  path.execution_profile.approved_tracking_limits.payout_acceleration = {-0.1,
                                                                          0.1};
  path.execution_profile.approved_tracking_limits
      .maximum_payout_tracking_error_mps = 0.03;
  path.execution_profile.approved_tracking_limits.tension = {80.0, 120.0};
  path.execution_profile.approved_tracking_limits.maximum_stopping_distance_m =
      1.5;
  return path;
}

void timed_paths_require_strict_time_and_matching_geometry() {
  // Design: 18.2.6-5
  TimedPath path = valid_timed_path();
  require(validate(path).valid, "a valid timed path was rejected");
  path.execution_profile.samples[1].time_from_start.nanoseconds = 0;
  require(!validate(path).valid,
          "an execution profile with repeated time was accepted");

  path = valid_timed_path();
  path.execution_profile.samples[1].arc_length_m = 1.1;
  require(!validate(path).valid,
          "an execution profile outside its geometry was accepted");

  path = valid_timed_path();
  path.execution_profile.samples[0].time_from_start.nanoseconds = 1;
  require(!validate(path).valid,
          "an execution profile whose relative time did not start at zero was accepted");

  path = valid_timed_path();
  path.geometry.points.insert(path.geometry.points.begin() + 1,
                              PathPoint{0.5, 1.5, 2.1, 0.15, -0.005});
  require(validate(path).valid,
          "an execution profile spanning independently sampled geometry was rejected");

  path = valid_timed_path();
  path.execution_profile.stopping_point_arc_length_m.reset();
  require(!validate(path).valid,
          "an execution profile without an explicit stopping point was accepted");

  path = valid_timed_path();
  path.execution_profile.samples[1].ground_speed_mps = 0.1;
  require(!validate(path).valid,
          "a stopping point with nonzero ground speed was accepted");
}

void timed_paths_reject_samples_outside_approved_limits() {
  using TimedPathMutation = void (*)(TimedPath&);
  const std::vector<std::pair<const char*, TimedPathMutation>> mutations{
      {"ground speed", [](TimedPath& path) {
         path.execution_profile.samples[0].ground_speed_mps = 0.81;
       }},
      {"ground acceleration", [](TimedPath& path) {
         path.execution_profile.samples[0].ground_acceleration_mps2 = 0.31;
       }},
      {"lateral acceleration", [](TimedPath& path) {
         path.execution_profile.approved_tracking_limits.ground_speed.maximum_mps =
             4.0;
         path.execution_profile.samples[0].ground_speed_mps = 4.0;
       }},
      {"payout speed", [](TimedPath& path) {
         path.execution_profile.samples[0].payout_speed_mps = 0.51;
       }},
      {"payout acceleration", [](TimedPath& path) {
         path.execution_profile.samples[0].payout_acceleration_mps2 = 0.11;
       }},
      {"tension", [](TimedPath& path) {
         path.execution_profile.samples[0].tension_setpoint_n = 120.01;
       }},
  };

  for (const auto& [name, mutate] : mutations) {
    TimedPath path = valid_timed_path();
    mutate(path);
    require(!validate(path).valid,
            std::string("an execution sample outside the approved ") + name +
                " limit was accepted");
  }
}

void execution_profile_versions_track_every_semantic_change() {
  // Design: 18.2.6-6
  ExecutionProfile baseline = valid_timed_path().execution_profile;
  ExecutionProfileVersioner stable_versioner{40};
  const ExecutionProfile first = stable_versioner.assign_version(baseline);
  const ExecutionProfile repeated = stable_versioner.assign_version(baseline);
  require(first.version == 41 && repeated.version == first.version,
          "identical execution profiles did not retain their assigned version");

  using ProfileMutation = void (*)(ExecutionProfile&);
  const std::vector<std::pair<const char*, ProfileMutation>> mutations{
      {"operating envelope", [](ExecutionProfile& profile) {
         ++profile.operating_envelope_version;
       }},
      {"interpolation rule", [](ExecutionProfile& profile) {
         profile.interpolation_rule = "monotone-cubic-time/v1";
       }},
      {"stopping point", [](ExecutionProfile& profile) {
         profile.stopping_point_arc_length_m = 0.5;
       }},
      {"sample arc length", [](ExecutionProfile& profile) {
         profile.samples[1].arc_length_m += 0.01;
       }},
      {"sample time", [](ExecutionProfile& profile) {
         ++profile.samples[1].time_from_start.nanoseconds;
       }},
      {"ground speed", [](ExecutionProfile& profile) {
         profile.samples[0].ground_speed_mps += 0.01;
       }},
      {"ground acceleration", [](ExecutionProfile& profile) {
         profile.samples[0].ground_acceleration_mps2 += 0.01;
       }},
      {"payout speed", [](ExecutionProfile& profile) {
         profile.samples[0].payout_speed_mps += 0.01;
       }},
      {"payout acceleration", [](ExecutionProfile& profile) {
         profile.samples[0].payout_acceleration_mps2 += 0.01;
       }},
      {"tension setpoint", [](ExecutionProfile& profile) {
         profile.samples[0].tension_setpoint_n += 0.01;
       }},
      {"ground speed minimum", [](ExecutionProfile& profile) {
         profile.approved_tracking_limits.ground_speed.minimum_mps -= 0.01;
       }},
      {"ground speed maximum", [](ExecutionProfile& profile) {
         profile.approved_tracking_limits.ground_speed.maximum_mps += 0.01;
       }},
      {"ground acceleration minimum", [](ExecutionProfile& profile) {
         profile.approved_tracking_limits.ground_acceleration.minimum_mps2 -=
             0.01;
       }},
      {"ground acceleration maximum", [](ExecutionProfile& profile) {
         profile.approved_tracking_limits.ground_acceleration.maximum_mps2 +=
             0.01;
       }},
      {"lateral acceleration", [](ExecutionProfile& profile) {
         profile.approved_tracking_limits.maximum_lateral_acceleration_mps2 +=
             0.01;
       }},
      {"payout speed minimum", [](ExecutionProfile& profile) {
         profile.approved_tracking_limits.payout_speed.minimum_mps -= 0.01;
       }},
      {"payout speed maximum", [](ExecutionProfile& profile) {
         profile.approved_tracking_limits.payout_speed.maximum_mps += 0.01;
       }},
      {"payout acceleration minimum", [](ExecutionProfile& profile) {
         profile.approved_tracking_limits.payout_acceleration.minimum_mps2 -=
             0.01;
       }},
      {"payout acceleration maximum", [](ExecutionProfile& profile) {
         profile.approved_tracking_limits.payout_acceleration.maximum_mps2 +=
             0.01;
       }},
      {"payout tracking error", [](ExecutionProfile& profile) {
         profile.approved_tracking_limits.maximum_payout_tracking_error_mps +=
             0.01;
       }},
      {"tension minimum", [](ExecutionProfile& profile) {
         profile.approved_tracking_limits.tension.minimum_n -= 0.01;
       }},
      {"tension maximum", [](ExecutionProfile& profile) {
         profile.approved_tracking_limits.tension.maximum_n += 0.01;
       }},
      {"stopping distance", [](ExecutionProfile& profile) {
         profile.approved_tracking_limits.maximum_stopping_distance_m += 0.01;
       }},
  };

  for (const auto& [name, mutate] : mutations) {
    ExecutionProfileVersioner versioner{100};
    const ExecutionProfile original = versioner.assign_version(baseline);
    ExecutionProfile changed = baseline;
    mutate(changed);
    const ExecutionProfile revised = versioner.assign_version(changed);
    require(revised.version == original.version + 1,
            std::string("execution profile version did not change with ") +
                name);
  }

  ExecutionProfile revised = baseline;
  revised.samples[0].payout_speed_mps += 0.01;
  require(!validate_execution_profile_revision(baseline, revised).valid,
          "changed execution content reused its prior version");
  revised.version = baseline.version + 1;
  require(validate_execution_profile_revision(baseline, revised).valid,
          "changed execution content with a newer version was rejected");
  revised.version = baseline.version - 1;
  require(!validate_execution_profile_revision(baseline, revised).valid,
          "changed execution content used a regressed version");
}

ErrorBudget valid_error_budget() {
  ErrorBudget budget;
  budget.robot_position_covariance_m2 = {0.04, 0.0, 0.0, 0.09};
  budget.touchdown_position_covariance_m2 = {
      {0.01, 0.0, 0.0, 0.02},
      {0.02, 0.001, 0.001, 0.03},
  };
  budget.epsilon_robot = 0.01;
  budget.epsilon_terrain_gradient_local = 0.02;
  budget.epsilon_point = 0.05;
  budget.reference_is_deterministic = true;
  budget.path_joint_risk_implemented = false;
  budget.terrain_gradient_path_joint_risk_implemented = false;
  budget.calibration_dataset_id = "corridor-cal/v1";
  budget.terrain_gradient_calibration_dataset_id = "terrain-cal/v1";
  budget.terrain_gradient_policy_version = 31;
  budget.corridor_risk_policy_version = 32;
  budget.cable_model_version = 33;
  budget.uncertainty_envelope_version = 34;
  budget.uncertainty_envelope_generator_version = 35;
  budget.execution_operating_envelope_version = 20;
  budget.operating_domain_id = "pool-a/v1";
  budget.sensor_mode = SensorHealthMode::approved_degraded;
  budget.covariance_envelope_audit_passed = true;
  return budget;
}

void error_budgets_reject_invalid_covariance_and_joint_risk_claims() {
  ErrorBudget budget = valid_error_budget();
  require(validate(budget).valid, "a valid error budget was rejected");
  budget.robot_position_covariance_m2.xy_m2 = 2.0;
  require(!validate(budget).valid,
          "a non-symmetric covariance matrix was accepted");

  budget = valid_error_budget();
  budget.epsilon_path = 0.1;
  require(!validate(budget).valid,
          "a first-version path joint-risk claim was accepted");
}

void planning_states_have_stable_complete_names() {
  const std::vector<std::pair<underwater_planner::core::PlanningState,
                              std::string>> expected{
      {underwater_planner::core::PlanningState::success, "SUCCESS"},
      {underwater_planner::core::PlanningState::path_valid, "PATH_VALID"},
      {underwater_planner::core::PlanningState::waiting_map, "WAITING_MAP"},
      {underwater_planner::core::PlanningState::request_scout, "REQUEST_SCOUT"},
      {underwater_planner::core::PlanningState::no_solution, "NO_SOLUTION"},
      {underwater_planner::core::PlanningState::no_solution_under_covariance_envelope,
       "NO_SOLUTION_UNDER_COVARIANCE_ENVELOPE"},
      {underwater_planner::core::PlanningState::covariance_envelope_breach,
       "COVARIANCE_ENVELOPE_BREACH"},
      {underwater_planner::core::PlanningState::input_invalid, "INPUT_INVALID"},
      {underwater_planner::core::PlanningState::map_expired, "MAP_EXPIRED"},
      {underwater_planner::core::PlanningState::timeout, "TIMEOUT"},
      {underwater_planner::core::PlanningState::communication_degraded,
       "COMM_DEGRADED"},
      {underwater_planner::core::PlanningState::manual_override,
       "MANUAL_OVERRIDE"},
      {underwater_planner::core::PlanningState::init, "INIT"},
      {underwater_planner::core::PlanningState::normal_planning,
       "NORMAL_PLANNING"},
      {underwater_planner::core::PlanningState::planning_with_caution,
       "PLANNING_WITH_CAUTION"},
      {underwater_planner::core::PlanningState::emergency_stop,
       "EMERGENCY_STOP"},
  };
  for (const auto& [state, name] : expected) {
    require(to_string(state) == name, "planning state name is unstable: " + name);
  }
}

PlanningResult valid_planning_result() {
  PlanningResult result;
  result.sequence_number = 44;
  result.timestamp = {5000};
  result.validity_duration = {1000000000};
  result.state = underwater_planner::core::PlanningState::path_valid;
  result.robot_trajectory = valid_timed_path();
  result.cable_path = valid_geometric_path();
  result.cable_path.metadata.path_version = 13;
  result.terminal_cable_state.kind = CableStateKind::tracked;
  result.terminal_cable_state.lag_angle_rad = 0.1;
  result.terminal_cable_state.lag_angle_variance_rad2 = 0.01;
  result.terminal_cable_state.timestamp = {5000};
  result.terminal_cable_state.sequence_number = 8;
  result.terminal_cable_state.laying_memory
      .previous_distinct_touchdown_points_m = {{-1.0, 2.0}, {-0.5, 2.1}};
  result.terminal_cable_state.laying_memory.trailing_support_samples = {
      {4.0, {-1.0, 2.0}}, {5.0, {-0.5, 2.1}}};
  result.terminal_cable_state.laying_memory.retained_arc_length_m = 1.0;
  result.terminal_cable_state.laying_memory.canonical_signature = 77;
  result.cable_model_validity =
      underwater_planner::core::CableModelValidity::valid;
  result.corridor_result.validity =
      underwater_planner::core::CorridorEvaluationValidity::valid;
  result.corridor_result.points = {
      {underwater_planner::core::CableValidationStatus::pass, 0.01, 0.02, 0.05,
       underwater_planner::core::CableCorridorPointBasis::below_nominal_bound,
       0.0, 0.0},
      {underwater_planner::core::CableValidationStatus::marginal, 0.02, 0.03,
       0.08,
       underwater_planner::core::CableCorridorPointBasis::within_absolute_bound,
       1.0, 1.0},
  };
  result.corridor_result.hard_feasible = true;
  result.corridor_result.marginal_count = 1;
  result.corridor_result.total_marginal_length_m = 0.5;
  result.corridor_result.maximum_marginal_length_m = 1.0;
  result.corridor_result.epsilon_point = 0.05;
  result.corridor_result.corridor_risk_policy_version = 32;
  result.corridor_result.reference_line_version = 9;
  result.corridor_result.interval_bound_certificate.version = 3;
  result.corridor_result.interval_bound_certificate.upper_bound_error_m = {
      0.01};
  result.corridor_result.evaluation_timestamp = {5000};
  result.corridor_result.operating_domain_id = "pool-a/v1";
  result.corridor_result.residual_distribution_calibration_dataset_id =
      "corridor-cal/v1";
  result.corridor_result.reference_is_deterministic = true;
  result.corridor_result.covariance_includes_coordinate_transform_error = true;
  result.corridor_result.covariance_envelope_audit_performed = true;
  result.corridor_result.path_joint_risk_implemented = false;
  result.corridor_result.risk_semantics =
      "POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE";
  result.cable_laying_result.valid = true;
  result.cable_laying_result.hard_feasible = true;
  result.cable_laying_result.failure_reasons = {
      underwater_planner::core::CableLayingFailure::none};
  result.cable_laying_result.limits_version = 16;
  result.cable_laying_result.terrain_map_sequence = 4;
  result.cable_laying_result.terrain_analysis_config_version = 6;
  result.cable_laying_result.operating_domain_id = "pool-a/v1";
  result.cable_laying_result.risk_semantics =
      "CONSERVATIVE_SUPPORT_PROXY:NO_FLEXIBLE_CABLE_DYNAMICS_GUARANTEE";
  result.cable_laying_result.maximum_absolute_curvature_per_m = 0.1;
  result.cable_laying_result.maximum_absolute_curvature_position_m =
      underwater_planner::core::Vector2m{1.0, 2.0};
  result.cable_laying_result.maximum_support_proxy_range_m = 0.05;
  result.cable_laying_result.maximum_support_proxy_position_m =
      underwater_planner::core::Vector2m{3.0, 4.0};
  result.cable_laying_result.terminal_support_window_length_m = 2.0;
  result.cable_laying_result.soft_cost = 1.5;
  result.cable_laying_result.terminal_memory =
      result.terminal_cable_state.laying_memory;
  result.error_budget = valid_error_budget();
  result.map_version = {"map-a", 4, {4000}, "world"};
  result.reference_line_version = 9;
  result.robot_operating_area_version = 10;
  result.terrain_gradient_policy_version = 31;
  result.corridor_risk_policy_version = 32;
  result.cable_model_version = 33;
  result.uncertainty_envelope_version = 34;
  result.uncertainty_envelope_generator_version = 35;
  result.execution_operating_envelope_version = 20;
  result.execution_profile_version = 21;
  result.sensor_mode = SensorHealthMode::approved_degraded;
  result.operating_domain_id = "pool-a/v1";
  result.cable_corridor_version = 17;
  result.diagnostics.schema_version = "underwater-planner-diagnostics/v1";
  result.diagnostics.random_seed = 1234;
  result.diagnostics.input_version = "scenario/v2";
  result.diagnostics.unit_system =
      "SI[length=m,angle=rad,time=s,curvature=1/m,speed=m/s,tension=N]";
  result.diagnostics.operating_domain_id = "pool-a/v1";
  result.diagnostics.risk_semantics =
      "POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE";
  result.diagnostics.dependencies.map_version = result.map_version;
  result.diagnostics.dependencies.reference_line_version = 9;
  result.diagnostics.dependencies.robot_operating_area_version = 10;
  result.diagnostics.dependencies.terrain_gradient_policy_version = 31;
  result.diagnostics.dependencies.corridor_risk_policy_version = 32;
  result.diagnostics.dependencies.cable_model_version = 33;
  result.diagnostics.dependencies.uncertainty_envelope_version = 34;
  result.diagnostics.dependencies.uncertainty_envelope_generator_version = 35;
  result.diagnostics.dependencies.execution_operating_envelope_version = 20;
  result.diagnostics.dependencies.execution_profile_version = 21;
  result.diagnostics.dependencies.sensor_mode =
      SensorHealthMode::approved_degraded;
  result.diagnostics.dependencies.operating_domain_id = "pool-a/v1";
  result.diagnostics.dependencies.cable_corridor_version =
      result.cable_corridor_version;
  result.diagnostics.entries.push_back(
      {underwater_planner::core::DiagnosticSeverity::warning, "PLAN_OK",
       "validation", "candidate passed all hard constraints", {5000}});
  return result;
}

void require_path_equal(const GeometricPath& actual,
                        const GeometricPath& expected) {
  require(actual.metadata.path_version == expected.metadata.path_version &&
              actual.metadata.coordinate_frame ==
                  expected.metadata.coordinate_frame &&
              actual.metadata.reference_line_version ==
                  expected.metadata.reference_line_version &&
              actual.metadata.interpolation_rule ==
                  expected.metadata.interpolation_rule &&
              actual.metadata.smoothing.has_value() ==
                  expected.metadata.smoothing.has_value(),
          "path metadata changed during serialization");
  if (actual.metadata.smoothing.has_value()) {
    const auto& left = *actual.metadata.smoothing;
    const auto& right = *expected.metadata.smoothing;
    require(left.smoother_version == right.smoother_version &&
                left.solver_status == right.solver_status &&
                left.limits_version == right.limits_version &&
                left.maximum_constraint_residual ==
                    right.maximum_constraint_residual &&
                left.maximum_absolute_curvature_per_m ==
                    right.maximum_absolute_curvature_per_m &&
                left.maximum_absolute_curvature_rate_per_m2 ==
                    right.maximum_absolute_curvature_rate_per_m2 &&
                left.residuals.maximum_dynamics_residual ==
                    right.residuals.maximum_dynamics_residual &&
                left.residuals.maximum_curvature_audit_residual ==
                    right.residuals.maximum_curvature_audit_residual &&
                left.residuals.maximum_curvature_rate_residual ==
                    right.residuals.maximum_curvature_rate_residual &&
                left.residuals.start_position_residual_m ==
                    right.residuals.start_position_residual_m &&
                left.residuals.start_heading_residual_rad ==
                    right.residuals.start_heading_residual_rad &&
                left.residuals.start_curvature_residual_per_m ==
                    right.residuals.start_curvature_residual_per_m &&
                left.residuals.goal_position_residual_m ==
                    right.residuals.goal_position_residual_m &&
                left.residuals.goal_heading_residual_rad ==
                    right.residuals.goal_heading_residual_rad &&
                left.residuals.goal_curvature_residual_per_m ==
                    right.residuals.goal_curvature_residual_per_m,
            "path smoothing metadata changed during serialization");
  }
  require(actual.points.size() == expected.points.size(),
          "path point count changed during serialization");
  for (std::size_t index = 0; index < actual.points.size(); ++index) {
    const PathPoint& left = actual.points[index];
    const PathPoint& right = expected.points[index];
    require(left.arc_length_m == right.arc_length_m && left.x_m == right.x_m &&
                left.y_m == right.y_m && left.heading_rad == right.heading_rad &&
                left.curvature_per_m == right.curvature_per_m,
            "path point changed during serialization");
  }
}

void require_memory_equal(
    const underwater_planner::core::CableConstraintMemory& actual,
    const underwater_planner::core::CableConstraintMemory& expected) {
  require(actual.retained_arc_length_m == expected.retained_arc_length_m &&
              actual.canonical_signature == expected.canonical_signature &&
              actual.previous_distinct_touchdown_points_m.size() ==
                  expected.previous_distinct_touchdown_points_m.size() &&
              actual.trailing_support_samples.size() ==
                  expected.trailing_support_samples.size(),
          "cable memory summary changed during serialization");
  for (std::size_t index = 0;
       index < actual.previous_distinct_touchdown_points_m.size(); ++index) {
    require(actual.previous_distinct_touchdown_points_m[index].x_m ==
                    expected.previous_distinct_touchdown_points_m[index].x_m &&
                actual.previous_distinct_touchdown_points_m[index].y_m ==
                    expected.previous_distinct_touchdown_points_m[index].y_m,
            "cable memory point changed during serialization");
  }
  for (std::size_t index = 0; index < actual.trailing_support_samples.size();
       ++index) {
    require(actual.trailing_support_samples[index].touchdown_arc_length_m ==
                    expected.trailing_support_samples[index]
                        .touchdown_arc_length_m &&
                actual.trailing_support_samples[index]
                        .touchdown_position_m.x_m ==
                    expected.trailing_support_samples[index]
                        .touchdown_position_m.x_m &&
                actual.trailing_support_samples[index]
                        .touchdown_position_m.y_m ==
                    expected.trailing_support_samples[index]
                        .touchdown_position_m.y_m,
            "cable support history changed during serialization");
  }
}

void require_timed_path_equal(const TimedPath& actual,
                              const TimedPath& expected) {
  require_path_equal(actual.geometry, expected.geometry);
  const auto& left = actual.execution_profile;
  const auto& right = expected.execution_profile;
  require(left.version == right.version &&
              same_execution_profile_content(left, right),
          "execution profile changed during serialization");
}

void require_covariance_equal(
    const underwater_planner::core::Covariance2dM2& actual,
    const underwater_planner::core::Covariance2dM2& expected) {
  require(actual.xx_m2 == expected.xx_m2 && actual.xy_m2 == expected.xy_m2 &&
              actual.yx_m2 == expected.yx_m2 && actual.yy_m2 == expected.yy_m2,
          "covariance changed during serialization");
}

void require_dependencies_equal(
    const underwater_planner::core::PlanningDependencyVersions& actual,
    const underwater_planner::core::PlanningDependencyVersions& expected) {
  require(actual.map_version.map_id == expected.map_version.map_id &&
              actual.map_version.sequence_number ==
                  expected.map_version.sequence_number &&
              actual.map_version.timestamp.nanoseconds ==
                  expected.map_version.timestamp.nanoseconds &&
              actual.map_version.coordinate_frame ==
                  expected.map_version.coordinate_frame &&
              actual.reference_line_version == expected.reference_line_version &&
              actual.robot_operating_area_version ==
                  expected.robot_operating_area_version &&
              actual.terrain_gradient_policy_version ==
                  expected.terrain_gradient_policy_version &&
              actual.corridor_risk_policy_version ==
                  expected.corridor_risk_policy_version &&
              actual.cable_model_version == expected.cable_model_version &&
              actual.uncertainty_envelope_version ==
                  expected.uncertainty_envelope_version &&
              actual.uncertainty_envelope_generator_version ==
                  expected.uncertainty_envelope_generator_version &&
              actual.execution_operating_envelope_version ==
                  expected.execution_operating_envelope_version &&
              actual.execution_profile_version ==
                  expected.execution_profile_version &&
              actual.sensor_mode == expected.sensor_mode &&
              actual.operating_domain_id == expected.operating_domain_id &&
              actual.cable_corridor_version ==
                  expected.cable_corridor_version,
          "dependency versions changed during serialization");
}

void require_planning_result_equal(const PlanningResult& actual,
                                   const PlanningResult& expected) {
  require(actual.sequence_number == expected.sequence_number &&
              actual.timestamp.nanoseconds == expected.timestamp.nanoseconds &&
              actual.validity_duration.nanoseconds ==
                  expected.validity_duration.nanoseconds &&
              actual.state == expected.state,
          "planning result identity changed during serialization");
  require_timed_path_equal(actual.robot_trajectory, expected.robot_trajectory);
  require_path_equal(actual.cable_path, expected.cable_path);
  require(actual.terminal_cable_state.kind ==
                  expected.terminal_cable_state.kind &&
              actual.terminal_cable_state.lag_angle_rad ==
                  expected.terminal_cable_state.lag_angle_rad &&
              actual.terminal_cable_state.lag_angle_variance_rad2 ==
                  expected.terminal_cable_state.lag_angle_variance_rad2 &&
              actual.terminal_cable_state.timestamp.nanoseconds ==
                  expected.terminal_cable_state.timestamp.nanoseconds &&
              actual.terminal_cable_state.sequence_number ==
                  expected.terminal_cable_state.sequence_number &&
              actual.cable_model_validity == expected.cable_model_validity,
          "terminal cable state changed during serialization");
  require_memory_equal(actual.terminal_cable_state.laying_memory,
                       expected.terminal_cable_state.laying_memory);
  require(actual.corridor_result.validity ==
                  expected.corridor_result.validity &&
              actual.corridor_result.hard_feasible ==
                  expected.corridor_result.hard_feasible &&
              actual.corridor_result.points.size() ==
                  expected.corridor_result.points.size() &&
              actual.corridor_result.marginal_count ==
                  expected.corridor_result.marginal_count &&
              actual.corridor_result.violation_count ==
                  expected.corridor_result.violation_count &&
              actual.corridor_result.total_marginal_length_m ==
                  expected.corridor_result.total_marginal_length_m &&
              actual.corridor_result.total_violation_length_m ==
                  expected.corridor_result.total_violation_length_m &&
              actual.corridor_result.maximum_marginal_length_m ==
                  expected.corridor_result.maximum_marginal_length_m &&
              actual.corridor_result.marginal_length_limit_exceeded ==
                  expected.corridor_result.marginal_length_limit_exceeded &&
              actual.corridor_result.epsilon_point ==
                  expected.corridor_result.epsilon_point &&
              actual.corridor_result.corridor_risk_policy_version ==
                  expected.corridor_result.corridor_risk_policy_version &&
              actual.corridor_result.reference_line_version ==
                  expected.corridor_result.reference_line_version &&
              actual.corridor_result.interval_bound_certificate.version ==
                  expected.corridor_result.interval_bound_certificate.version &&
              actual.corridor_result.interval_bound_certificate
                      .upper_bound_error_m ==
                  expected.corridor_result.interval_bound_certificate
                      .upper_bound_error_m &&
              actual.corridor_result.evaluation_timestamp.nanoseconds ==
                  expected.corridor_result.evaluation_timestamp.nanoseconds &&
              actual.corridor_result.operating_domain_id ==
                  expected.corridor_result.operating_domain_id &&
              actual.corridor_result
                      .residual_distribution_calibration_dataset_id ==
                  expected.corridor_result
                      .residual_distribution_calibration_dataset_id &&
              actual.corridor_result.reference_is_deterministic ==
                  expected.corridor_result.reference_is_deterministic &&
              actual.corridor_result
                      .covariance_includes_coordinate_transform_error ==
                  expected.corridor_result
                      .covariance_includes_coordinate_transform_error &&
              actual.corridor_result.covariance_envelope_audit_performed ==
                  expected.corridor_result
                      .covariance_envelope_audit_performed &&
              actual.corridor_result.path_joint_risk_implemented ==
                  expected.corridor_result.path_joint_risk_implemented &&
              actual.corridor_result.risk_semantics ==
                  expected.corridor_result.risk_semantics &&
              actual.corridor_result.issues ==
                  expected.corridor_result.issues,
          "corridor summary changed during serialization");
  for (std::size_t index = 0; index < actual.corridor_result.points.size();
       ++index) {
    const auto& left = actual.corridor_result.points[index];
    const auto& right = expected.corridor_result.points[index];
    require(left.status == right.status &&
                left.mean_lateral_error_m == right.mean_lateral_error_m &&
                left.lateral_stddev_m == right.lateral_stddev_m &&
                left.upper_bound_m == right.upper_bound_m &&
                left.basis == right.basis &&
                left.touchdown_arc_length_m ==
                    right.touchdown_arc_length_m &&
                left.reference_progress_m == right.reference_progress_m,
            "corridor point changed during serialization");
  }
  require(actual.cable_laying_result.valid ==
                  expected.cable_laying_result.valid &&
              actual.cable_laying_result.hard_feasible ==
                  expected.cable_laying_result.hard_feasible &&
              actual.cable_laying_result.failure_reasons ==
                  expected.cable_laying_result.failure_reasons &&
              actual.cable_laying_result.maximum_absolute_curvature_per_m ==
                  expected.cable_laying_result
                      .maximum_absolute_curvature_per_m &&
              actual.cable_laying_result
                      .maximum_absolute_curvature_position_m.has_value() &&
              expected.cable_laying_result
                      .maximum_absolute_curvature_position_m.has_value() &&
              actual.cable_laying_result
                      .maximum_absolute_curvature_position_m->x_m ==
                  expected.cable_laying_result
                      .maximum_absolute_curvature_position_m->x_m &&
              actual.cable_laying_result
                      .maximum_absolute_curvature_position_m->y_m ==
                  expected.cable_laying_result
                      .maximum_absolute_curvature_position_m->y_m &&
          actual.cable_laying_result.maximum_support_proxy_range_m ==
                  expected.cable_laying_result.maximum_support_proxy_range_m &&
              actual.cable_laying_result
                      .maximum_support_proxy_position_m.has_value() &&
              expected.cable_laying_result
                      .maximum_support_proxy_position_m.has_value() &&
              actual.cable_laying_result
                      .maximum_support_proxy_position_m->x_m ==
                  expected.cable_laying_result
                      .maximum_support_proxy_position_m->x_m &&
              actual.cable_laying_result
                      .maximum_support_proxy_position_m->y_m ==
                  expected.cable_laying_result
                      .maximum_support_proxy_position_m->y_m &&
              actual.cable_laying_result.terminal_support_window_length_m ==
                  expected.cable_laying_result
                      .terminal_support_window_length_m &&
              actual.cable_laying_result.soft_cost ==
                  expected.cable_laying_result.soft_cost,
          "cable laying result changed during serialization");
  require(actual.cable_laying_result.failure_segments.size() ==
              expected.cable_laying_result.failure_segments.size(),
          "cable laying failure segments changed during serialization");
  for (std::size_t index = 0;
       index < actual.cable_laying_result.failure_segments.size(); ++index) {
    const auto& left = actual.cable_laying_result.failure_segments[index];
    const auto& right = expected.cable_laying_result.failure_segments[index];
    require(left.reason == right.reason &&
                left.start_arc_length_m == right.start_arc_length_m &&
                left.end_arc_length_m == right.end_arc_length_m &&
                left.representative_position_m.x_m ==
                    right.representative_position_m.x_m &&
                left.representative_position_m.y_m ==
                    right.representative_position_m.y_m,
            "cable laying failure segment changed during serialization");
  }
  require(actual.cable_laying_result.limits_version ==
                  expected.cable_laying_result.limits_version &&
              actual.cable_laying_result.terrain_map_sequence ==
                  expected.cable_laying_result.terrain_map_sequence &&
              actual.cable_laying_result.terrain_analysis_config_version ==
                  expected.cable_laying_result
                      .terrain_analysis_config_version &&
              actual.cable_laying_result.operating_domain_id ==
                  expected.cable_laying_result.operating_domain_id &&
              actual.cable_laying_result.risk_semantics ==
                  expected.cable_laying_result.risk_semantics,
          "cable laying audit metadata changed during serialization");
  require_memory_equal(actual.cable_laying_result.terminal_memory,
                       expected.cable_laying_result.terminal_memory);
  const ErrorBudget& left_budget = actual.error_budget;
  const ErrorBudget& right_budget = expected.error_budget;
  require_covariance_equal(left_budget.robot_position_covariance_m2,
                           right_budget.robot_position_covariance_m2);
  require(left_budget.touchdown_position_covariance_m2.size() ==
              right_budget.touchdown_position_covariance_m2.size(),
          "touchdown covariance count changed during serialization");
  for (std::size_t index = 0;
       index < left_budget.touchdown_position_covariance_m2.size(); ++index) {
    require_covariance_equal(
        left_budget.touchdown_position_covariance_m2[index],
        right_budget.touchdown_position_covariance_m2[index]);
  }
  require(left_budget.epsilon_robot == right_budget.epsilon_robot &&
              left_budget.epsilon_terrain_gradient_local ==
                  right_budget.epsilon_terrain_gradient_local &&
              left_budget.epsilon_point == right_budget.epsilon_point &&
              left_budget.epsilon_path == right_budget.epsilon_path &&
              left_budget.reference_is_deterministic ==
                  right_budget.reference_is_deterministic &&
              left_budget.path_joint_risk_implemented ==
                  right_budget.path_joint_risk_implemented &&
              left_budget.terrain_gradient_path_joint_risk_implemented ==
                  right_budget.terrain_gradient_path_joint_risk_implemented &&
              left_budget.calibration_dataset_id ==
                  right_budget.calibration_dataset_id &&
              left_budget.terrain_gradient_calibration_dataset_id ==
                  right_budget.terrain_gradient_calibration_dataset_id &&
              left_budget.terrain_gradient_policy_version ==
                  right_budget.terrain_gradient_policy_version &&
              left_budget.corridor_risk_policy_version ==
                  right_budget.corridor_risk_policy_version &&
              left_budget.cable_model_version ==
                  right_budget.cable_model_version &&
              left_budget.uncertainty_envelope_version ==
                  right_budget.uncertainty_envelope_version &&
              left_budget.uncertainty_envelope_generator_version ==
                  right_budget.uncertainty_envelope_generator_version &&
              left_budget.execution_operating_envelope_version ==
                  right_budget.execution_operating_envelope_version &&
              left_budget.operating_domain_id ==
                  right_budget.operating_domain_id &&
              left_budget.sensor_mode == right_budget.sensor_mode &&
              left_budget.covariance_envelope_audit_passed ==
                  right_budget.covariance_envelope_audit_passed,
          "error budget changed during serialization");
  require(actual.map_version.map_id == expected.map_version.map_id &&
              actual.map_version.sequence_number ==
                  expected.map_version.sequence_number &&
              actual.map_version.timestamp.nanoseconds ==
                  expected.map_version.timestamp.nanoseconds &&
              actual.map_version.coordinate_frame ==
                  expected.map_version.coordinate_frame &&
              actual.reference_line_version == expected.reference_line_version &&
              actual.robot_operating_area_version ==
                  expected.robot_operating_area_version &&
              actual.terrain_gradient_policy_version ==
                  expected.terrain_gradient_policy_version &&
              actual.corridor_risk_policy_version ==
                  expected.corridor_risk_policy_version &&
              actual.cable_model_version == expected.cable_model_version &&
              actual.uncertainty_envelope_version ==
                  expected.uncertainty_envelope_version &&
              actual.uncertainty_envelope_generator_version ==
                  expected.uncertainty_envelope_generator_version &&
              actual.execution_operating_envelope_version ==
                  expected.execution_operating_envelope_version &&
              actual.execution_profile_version ==
                  expected.execution_profile_version &&
              actual.sensor_mode == expected.sensor_mode &&
              actual.operating_domain_id == expected.operating_domain_id &&
              actual.cable_corridor_version ==
                  expected.cable_corridor_version,
          "planning dependency metadata changed during serialization");
  const auto& left_diagnostics = actual.diagnostics;
  const auto& right_diagnostics = expected.diagnostics;
  require(left_diagnostics.schema_version == right_diagnostics.schema_version &&
              left_diagnostics.random_seed == right_diagnostics.random_seed &&
              left_diagnostics.input_version == right_diagnostics.input_version &&
              left_diagnostics.unit_system == right_diagnostics.unit_system &&
              left_diagnostics.operating_domain_id ==
                  right_diagnostics.operating_domain_id &&
              left_diagnostics.risk_semantics ==
                  right_diagnostics.risk_semantics &&
              left_diagnostics.entries.size() ==
                  right_diagnostics.entries.size(),
          "diagnostic metadata changed during serialization");
  require_dependencies_equal(left_diagnostics.dependencies,
                             right_diagnostics.dependencies);
  for (std::size_t index = 0; index < left_diagnostics.entries.size(); ++index) {
    const auto& left = left_diagnostics.entries[index];
    const auto& right = right_diagnostics.entries[index];
    require(left.severity == right.severity && left.code == right.code &&
                left.stage == right.stage && left.message == right.message &&
                left.timestamp.nanoseconds == right.timestamp.nanoseconds,
            "diagnostic entry changed during serialization");
  }
}

void planning_results_require_complete_consistent_evidence() {
  PlanningResult result = valid_planning_result();
  require(validate(result).valid, "a complete successful planning result was rejected");

  result.execution_profile_version = 22;
  require(!validate(result).valid,
          "a result with a mismatched execution profile version was accepted");

  result = valid_planning_result();
  result.corridor_result.points[1].status =
      underwater_planner::core::CableValidationStatus::violation;
  require(!validate(result).valid,
          "a successful result with a corridor violation was accepted");

  result = valid_planning_result();
  result.corridor_result.covariance_envelope_audit_performed = false;
  require(!validate(result).valid,
          "a pointwise corridor result without envelope audit was published");

  result = valid_planning_result();
  result.error_budget.touchdown_position_covariance_m2.pop_back();
  require(!validate(result).valid,
          "a covariance profile misaligned with the cable path was accepted");

  result = valid_planning_result();
  result.cable_laying_result.terminal_memory.canonical_signature = 99;
  require(!validate(result).valid,
          "contradictory terminal cable memories were accepted");

  result = valid_planning_result();
  result.diagnostics.dependencies.cable_model_version = 99;
  require(!validate(result).valid,
          "diagnostics with mismatched dependency versions were accepted");

  result = valid_planning_result();
  result.diagnostics.dependencies.uncertainty_envelope_generator_version = 99;
  require(!validate(result).valid,
          "diagnostics with a mismatched envelope generator were accepted");

  result = valid_planning_result();
  result.cable_laying_result.valid = false;
  require(!validate(result).valid,
          "an invalid but hard-feasible cable evaluation was accepted");

  result = valid_planning_result();
  result.state = underwater_planner::core::PlanningState::no_solution;
  result.cable_laying_result.hard_feasible = false;
  result.cable_laying_result.failure_reasons = {
      underwater_planner::core::CableLayingFailure::none,
      underwater_planner::core::CableLayingFailure::curvature_exceeded};
  result.cable_laying_result.failure_segments = {
      {underwater_planner::core::CableLayingFailure::curvature_exceeded,
       0.5, 1.0, {0.75, 0.0}}};
  result.cable_laying_result.soft_cost = 0.0;
  require(!validate(result).valid,
          "an infeasible cable result retained the NONE reason");

  result = valid_planning_result();
  result.state = underwater_planner::core::PlanningState::no_solution;
  result.cable_laying_result.hard_feasible = false;
  result.cable_laying_result.failure_reasons = {
      underwater_planner::core::CableLayingFailure::curvature_exceeded,
      underwater_planner::core::CableLayingFailure::support_proxy_exceeded};
  result.cable_laying_result.failure_segments = {
      {underwater_planner::core::CableLayingFailure::curvature_exceeded,
       0.5, 1.0, {0.75, 0.0}}};
  result.cable_laying_result.soft_cost = 0.0;
  require(!validate(result).valid,
          "a cable failure reason without an audit segment was accepted");

  result = valid_planning_result();
  result.state = underwater_planner::core::PlanningState::no_solution;
  result.cable_laying_result.valid = false;
  result.cable_laying_result.hard_feasible = false;
  result.cable_laying_result.failure_reasons = {
      underwater_planner::core::CableLayingFailure::numerically_invalid};
  result.cable_laying_result.limits_version = 0;
  result.cable_laying_result.terrain_map_sequence = 0;
  result.cable_laying_result.terrain_analysis_config_version = 0;
  result.cable_laying_result.operating_domain_id.clear();
  result.cable_laying_result.risk_semantics.clear();
  require(!validate(result).valid,
          "an invalid cable evaluation omitted its audit dependencies");
}

void failure_results_still_reject_nonfinite_contract_fields() {
  PlanningResult result = valid_planning_result();
  result.state = underwater_planner::core::PlanningState::no_solution;
  result.robot_trajectory.execution_profile.samples[0].ground_speed_mps =
      std::numeric_limits<double>::quiet_NaN();
  require(!validate(result).valid,
          "a failure result serialized a non-finite trajectory field");

  result = valid_planning_result();
  result.state = underwater_planner::core::PlanningState::no_solution;
  result.robot_trajectory.geometry.points[1].heading_rad = 4.0;
  require(!validate(result).valid,
          "a failure result serialized a non-normalized path heading");

  result = valid_planning_result();
  result.state = underwater_planner::core::PlanningState::no_solution;
  result.robot_trajectory.execution_profile.samples[1]
      .time_from_start.nanoseconds = -1;
  require(!validate(result).valid,
          "a failure result serialized a negative execution time");

  result = valid_planning_result();
  result.state = underwater_planner::core::PlanningState::no_solution;
  result.error_budget = ErrorBudget{};
  result.uncertainty_envelope_generator_version = 0;
  result.diagnostics.dependencies.uncertainty_envelope_generator_version = 0;
  require(!validate(result).valid,
          "a failure result omitted the envelope generator version");
}

void failure_results_require_nested_evaluation_times() {
  const PlanningState failure_states[] = {
      PlanningState::no_solution,
      PlanningState::input_invalid,
      PlanningState::timeout,
  };
  for (const PlanningState state : failure_states) {
    PlanningResult result = valid_planning_result();
    result.state = state;
    result.terminal_cable_state.timestamp = {};
    require(!validate(result).valid,
            "a failure result with an omitted terminal cable timestamp was accepted");

    result = valid_planning_result();
    result.state = state;
    result.corridor_result.evaluation_timestamp = {};
    require(!validate(result).valid,
            "a failure result with an omitted corridor timestamp was accepted");

    result = valid_planning_result();
    result.state = state;
    result.terminal_cable_state.timestamp = {-1};
    require(!validate(result).valid,
            "a failure result with a negative terminal timestamp was accepted");

    result = valid_planning_result();
    result.state = state;
    result.corridor_result.evaluation_timestamp = {-1};
    require(!validate(result).valid,
            "a failure result with a negative corridor timestamp was accepted");

    result = valid_planning_result();
    result.state = state;
    result.terminal_cable_state.timestamp = result.timestamp;
    result.corridor_result.evaluation_timestamp = result.timestamp;
    require(validate(result).valid,
            "a failure result with assembled nested timestamps was rejected");
    const PlanningResult decoded =
        deserialize_planning_result(serialize_planning_result(result));
    require(decoded.terminal_cable_state.timestamp.nanoseconds ==
                result.timestamp.nanoseconds &&
            decoded.corridor_result.evaluation_timestamp.nanoseconds ==
                result.timestamp.nanoseconds,
            "failure result nested timestamps did not round-trip");
  }
}

void unknown_enum_values_are_rejected_before_serialization() {
  PlanningResult result = valid_planning_result();
  result.state = static_cast<underwater_planner::core::PlanningState>(99);
  require(!validate(result).valid,
          "an unknown planning state was accepted by direct validation");

  result = valid_planning_result();
  result.corridor_result.points[0].status =
      static_cast<underwater_planner::core::CableValidationStatus>(99);
  require(!validate(result).valid,
          "an unknown corridor status was accepted by direct validation");
}

void planning_results_round_trip_through_versioned_serialization() {
  const PlanningResult original = valid_planning_result();
  const std::string encoded = serialize_planning_result(original);
  require(encoded.find("UP_RESULT 8") == 0,
          "planning result serialization is missing its schema version");
  require(encoded.find("POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE") !=
              std::string::npos,
          "serialized result dropped explicit risk semantics");

  const PlanningResult decoded = deserialize_planning_result(encoded);
  require(validate(decoded).valid, "deserialized planning result is invalid");
  require_planning_result_equal(decoded, original);
  require(serialize_planning_result(decoded) == encoded,
          "planning result serialization was not a field-stable round trip");
  const std::uint64_t schema_fingerprint = fnv1a64(encoded);
  require(schema_fingerprint == 6342600014517287561ULL,
          "UP_RESULT v8 golden-schema fingerprint changed: " +
              std::to_string(schema_fingerprint));

  bool rejected = false;
  try {
    static_cast<void>(deserialize_planning_result("UP_RESULT 6"));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected, "the superseded planning-result schema was accepted");
}

void cable_laying_failure_segments_round_trip() {
  PlanningResult original = valid_planning_result();
  original.state = underwater_planner::core::PlanningState::no_solution;
  original.cable_laying_result.hard_feasible = false;
  original.cable_laying_result.failure_reasons = {
      underwater_planner::core::CableLayingFailure::support_proxy_exceeded};
  original.cable_laying_result.failure_segments = {
      {underwater_planner::core::CableLayingFailure::support_proxy_exceeded,
       1.25, 1.75, {1.5, -0.25}}};
  original.cable_laying_result.soft_cost = 0.0;
  require(validate(original).valid,
          "an auditable cable hard-failure result was rejected");

  const PlanningResult decoded =
      deserialize_planning_result(serialize_planning_result(original));
  require_planning_result_equal(decoded, original);
}

}  // namespace

int main() {
  constexpr std::uint64_t kSeed = 0x5EED1234ULL;
  const std::string context =
      "seed=" + std::to_string(kSeed) +
      " input_version=data-contract/v1"
      " units=SI[length=m,angle=rad,time=s,curvature=1/m,speed=m/s,tension=N]"
      " timestamp_ns=1700000000000000000"
      " operating_domain=synthetic-level1/v1"
      " risk_semantics=POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE";
  const std::vector<std::pair<const char*, void (*)()>> tests{
      {"default_constructed_contracts_are_invalid",
       default_constructed_contracts_are_invalid},
      {"invalid_numeric_values_are_rejected", invalid_numeric_values_are_rejected},
      {"angles_are_normalized_and_nonfinite_angles_are_rejected",
       angles_are_normalized_and_nonfinite_angles_are_rejected},
      {"cable_state_and_reference_progress_enforce_context",
       cable_state_and_reference_progress_enforce_context},
      {"geometric_paths_require_finite_strictly_increasing_arc_length",
       geometric_paths_require_finite_strictly_increasing_arc_length},
      {"timed_paths_require_strict_time_and_matching_geometry",
       timed_paths_require_strict_time_and_matching_geometry},
      {"timed_paths_reject_samples_outside_approved_limits",
       timed_paths_reject_samples_outside_approved_limits},
      {"execution_profile_versions_track_every_semantic_change",
       execution_profile_versions_track_every_semantic_change},
      {"error_budgets_reject_invalid_covariance_and_joint_risk_claims",
       error_budgets_reject_invalid_covariance_and_joint_risk_claims},
      {"planning_states_have_stable_complete_names",
       planning_states_have_stable_complete_names},
      {"planning_results_require_complete_consistent_evidence",
       planning_results_require_complete_consistent_evidence},
      {"failure_results_still_reject_nonfinite_contract_fields",
       failure_results_still_reject_nonfinite_contract_fields},
      {"failure_results_require_nested_evaluation_times",
       failure_results_require_nested_evaluation_times},
      {"unknown_enum_values_are_rejected_before_serialization",
       unknown_enum_values_are_rejected_before_serialization},
      {"planning_results_round_trip_through_versioned_serialization",
       planning_results_round_trip_through_versioned_serialization},
      {"cable_laying_failure_segments_round_trip",
       cable_laying_failure_segments_round_trip},
  };

  try {
    for (const auto& [name, test] : tests) {
      test();
      std::cout << "[pass] " << name << '\n';
    }
  } catch (const std::exception& error) {
    std::cerr << "[failure] " << context << " error=" << error.what() << '\n';
    return 1;
  }
  std::cout << "[metrics] tests=" << tests.size() << ' ' << context << '\n';
  return 0;
}
