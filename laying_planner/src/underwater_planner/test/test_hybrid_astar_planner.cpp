#include "underwater_planner/core/algorithm_diagnostics.hpp"
#include "underwater_planner/core/hybrid_astar_planner.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

using namespace underwater_planner::core;

constexpr MonotonicTime kTimestamp{1'000'000'000};
constexpr MonotonicTime kPlanningTime{1'100'000'000};
constexpr MonotonicTime kValidUntil{2'000'000'000};

void require(const bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void require_near(const double actual, const double expected,
                  const double tolerance, const std::string& message) {
  require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
          message);
}

CableModelParameters model_parameters() {
  CableModelParameters parameters;
  parameters.version = 20;
  parameters.calibration_dataset_id = "t21-cable-cal-v1";
  parameters.operating_domain_id = "flat-straight-domain-v1";
  parameters.release_point_offset_m = {0.0, 0.0};
  parameters.touchdown_distance_m = 1.0;
  parameters.direction_response_length_m = 2.0;
  parameters.maximum_lag_angle_rad = 0.5;
  parameters.maximum_payout_tracking_error_mps = 0.1;
  parameters.payout_speed_range = {0.0, 1.0};
  parameters.maximum_payout_acceleration_mps2 = 0.4;
  parameters.maximum_tension_tracking_error_n = 10.0;
  parameters.tension_range = {10.0, 100.0};
  parameters.search_integration_step_m = 0.25;
  parameters.validation_integration_step_m = 0.02;
  parameters.touchdown_distance_variance_m2 = 0.0025;
  parameters.direction_response_length_variance_m2 = 0.04;
  parameters.lag_angle_process_variance_per_m_rad2 = 0.03;
  parameters.touchdown_process_noise_per_m_m2 = {0.001, 0.0, 0.0, 0.002};
  parameters.approved_sensor_modes = {SensorHealthMode::nominal};
  return parameters;
}

ExecutionOperatingEnvelope execution_envelope() {
  ExecutionOperatingEnvelope envelope;
  envelope.version = 7;
  envelope.operating_domain_id = "flat-straight-domain-v1";
  envelope.limits.ground_speed = {0.0, 0.8};
  envelope.limits.ground_acceleration = {-0.4, 0.4};
  envelope.limits.maximum_lateral_acceleration_mps2 = 0.4;
  envelope.limits.payout_speed = {0.0, 0.9};
  envelope.limits.payout_acceleration = {-0.3, 0.3};
  envelope.limits.maximum_payout_tracking_error_mps = 0.08;
  envelope.limits.tension = {20.0, 80.0};
  envelope.limits.maximum_stopping_distance_m = 1.5;
  envelope.maximum_payout_acceleration_tracking_error_mps2 = 0.1;
  envelope.maximum_tension_tracking_error_n = 8.0;
  return envelope;
}

ParameterConfig parameter_config() {
  ParameterConfig config;
  config.profile_id = "t21-search-profile-v1";
  config.operating_domain_id = "flat-straight-domain-v1";
  config.search.reference_progress_backward_tolerance_m = 0.0;
  config.search.reference_progress_maximum_ratio = 1.0;
  config.search.reference_progress_forward_slack_m = 0.01;
  config.search.reference_progress_distance_scale_m = 1.0;
  config.search.reference_progress_heading_scale_rad = 0.5;
  config.search.reference_progress_heading_weight = 1.0;
  config.search.reference_progress_association_score_tolerance = 1.0e-10;
  return config;
}

HybridAStarSearchParameters search_parameters() {
  HybridAStarSearchParameters parameters;
  parameters.version = 21;
  parameters.primitive_set_version = 31;
  parameters.path_version = 2101;
  parameters.xy_resolution_m = 0.5;
  parameters.heading_resolution_rad = 0.1;
  parameters.cable_lag_resolution_rad = 0.1;
  parameters.reference_progress_resolution_m = 0.5;
  parameters.goal_position_tolerance_m = 0.01;
  parameters.goal_heading_tolerance_rad = 0.01;
  parameters.goal_lag_tolerance_rad = 0.01;
  parameters.goal_progress_tolerance_m = 0.01;
  parameters.goal_touchdown_position_tolerance_m = 0.01;
  parameters.minimum_turning_radius_m = 1.0;
  parameters.path_length_cost_weight = 1.0;
  parameters.path_curvature_cost_weight = 1.0;
  parameters.touchdown_center_cost_weight = 1.0;
  parameters.touchdown_margin_cost_weight = 1.0;
  parameters.robot_terrain_cost_weight = 1.0;
  parameters.maximum_sweep_spacing_fraction = 0.5;
  parameters.cable_sweep_margin_m = 0.01;
  parameters.equivalent_label_cost_tolerance_m = 0.0;
  parameters.maximum_planning_duration_s = 5.0;
  parameters.maximum_expansions = 100;
  parameters.maximum_active_labels = 1000;
  parameters.motion_primitives = {{1, 1.0, 0.0}};
  return parameters;
}

HybridAStarSearchParameters detour_search_parameters() {
  HybridAStarSearchParameters parameters = search_parameters();
  parameters.xy_resolution_m = 0.25;
  parameters.heading_resolution_rad = 0.25;
  parameters.cable_lag_resolution_rad = 0.25;
  parameters.reference_progress_resolution_m = 0.25;
  parameters.goal_position_tolerance_m = 0.02;
  parameters.goal_heading_tolerance_rad = 0.02;
  parameters.goal_lag_tolerance_rad = 0.06;
  parameters.goal_progress_tolerance_m = 0.2;
  parameters.goal_touchdown_position_tolerance_m = 0.2;
  parameters.maximum_expansions = 2000U;
  parameters.maximum_active_labels = 20000U;
  parameters.analytic_expansion_interval = 1U;
  parameters.motion_primitives = {
      {1U, 1.0, 0.0}, {2U, 1.0, 0.5}, {3U, 1.0, -0.5}};
  return parameters;
}

CableCorridorRiskPolicy corridor_policy() {
  return {41,
          "t21-corridor-risk-cal-v1",
          "flat-straight-domain-v1",
          0.05,
          0.20,
          0.30,
          0.0,
          0.01,
          true};
}

CableCorridorRiskPolicy narrow_corridor_policy() {
  CableCorridorRiskPolicy policy = corridor_policy();
  policy.nominal_half_width_m = 0.08;
  policy.absolute_half_width_m = 0.09;
  return policy;
}

CableCorridorRiskPolicy wide_corridor_policy() {
  CableCorridorRiskPolicy policy = corridor_policy();
  policy.nominal_half_width_m = 1.5;
  policy.absolute_half_width_m = 2.5;
  return policy;
}

MapSnapshot planning_map() {
  MapSnapshot map;
  map.version = {"t22-planning-map", 22U, kTimestamp, "map"};
  map.width = 140U;
  map.height = 80U;
  map.resolution_m = 0.1;
  map.origin_x_m = -4.0;
  map.origin_y_m = -4.0;
  map.derived_configuration_version = 22U;
  map.cells.assign(map.width * map.height,
                   MapCell{0.0, 0.001, 1.0, true});
  for (MapCell& cell : map.cells) {
    cell.measurement_timestamp = kTimestamp;
  }
  return map;
}

TerrainLayers flat_terrain(const MapSnapshot& map) {
  TerrainLayers terrain;
  terrain.source_map_version = map.version;
  terrain.analysis_config_version = map.derived_configuration_version;
  terrain.operating_domain_id = "flat-straight-domain-v1";
  terrain.surface_fit_window_size_m = 0.3;
  terrain.surface.width = map.width;
  terrain.surface.height = map.height;
  terrain.surface.resolution_m = map.resolution_m;
  terrain.surface.origin_x_m = map.origin_x_m;
  terrain.surface.origin_y_m = map.origin_y_m;
  terrain.surface.cells.resize(map.cells.size());
  terrain.cable_laying.cells.resize(map.cells.size());
  for (std::size_t index = 0U; index < map.cells.size(); ++index) {
    terrain.surface.cells[index].support_ratio = 1.0;
    terrain.surface.cells[index].status = TerrainEstimateStatus::valid;
    terrain.cable_laying.cells[index].known = true;
    terrain.cable_laying.cells[index].confidence = 1.0;
  }
  return terrain;
}

TrackFootprint track_footprint() {
  return {{{-0.05, -0.05}, {0.05, -0.05}, {0.05, 0.05},
           {-0.05, 0.05}},
          {{-0.04, 0.01}, {0.04, 0.01}, {0.04, 0.04}, {-0.04, 0.04}},
          {{-0.04, -0.04}, {0.04, -0.04}, {0.04, -0.01},
           {-0.04, -0.01}}};
}

TrackFootprint tiny_track_footprint() {
  return {{{-0.005, -0.005}, {0.005, -0.005}, {0.005, 0.005},
           {-0.005, 0.005}},
          {{-0.004, 0.001}, {0.004, 0.001}, {0.004, 0.004},
           {-0.004, 0.004}},
          {{-0.004, -0.004}, {0.004, -0.004}, {0.004, -0.001},
           {-0.004, -0.001}}};
}

HybridAStarPrimitiveSweepContext primitive_sweep_context() {
  HybridAStarPrimitiveSweepContext context;
  context.map = planning_map();
  context.terrain = flat_terrain(context.map);
  context.robot_operating_area = {
      22U,
      "t22-operating-area",
      {{-3.5, -3.5}, {9.5, -3.5}, {9.5, 3.5}, {-3.5, 3.5}}};
  context.robot_relative_obstacle_covariance_m2 = {0.0, 0.0, 0.0, 0.0};
  context.collision_risk_policy = {
      22U, "t22-collision-cal-v1", "flat-straight-domain-v1",
      0.05, 0.5, 0.0};
  context.robot_capability = {0.8, 0.8, 0.8, 0.8, 0.3, 0.3,
                              0.5, 0.05, 0.2, 0.1, 0.1};
  context.track_footprint = track_footprint();
  context.terrain_gradient_risk_policy = {
      22U,
      22U,
      0.05,
      1.0,
      GradientCoverageModel::deterministic_bounded,
      "t22-gradient-cal-v1",
      "flat-straight-domain-v1",
      true};
  context.cable_laying_limits = {
      22U, "flat-straight-domain-v1", 0.2, 2.0, 0.2, 0.5,
      0.1, 0.3, 0.5, 1.0e-6, 1.0, 1.0, 1.0};
  context.cable_history_boundary = CableHistoryBoundary::explicit_task_start;
  return context;
}

ReferenceLine reference_line() {
  return make_reference_line(11, "map", {{0.0, 0.0}, {8.0, 0.0}});
}

CableUncertaintyEnvelope envelope() {
  CableUncertaintyEnvelope result;
  result.validity = EnvelopeBuildValidity::valid;
  result.dependencies.generator_version = 10;
  result.dependencies.cable_model_version = 20;
  result.dependencies.execution_operating_envelope_version = 7;
  result.dependencies.reference_line_version = 11;
  result.dependencies.operating_domain_version = 1;
  result.dependencies.primitive_set_version = 31;
  result.dependencies.initial_uncertainty_version = 2;
  result.dependencies.sensor_uncertainty_version = 3;
  result.dependencies.execution_uncertainty_version = 4;
  result.dependencies.margin_certification_version = 5;
  result.dependencies.sensor_mode = SensorHealthMode::nominal;
  result.dependencies.operating_domain_id = "flat-straight-domain-v1";
  result.dependencies.cable_model_calibration_dataset_id = "t21-cable-cal-v1";
  result.dependencies.certification_dataset_id = "t21-envelope-proof-v1";
  result.dependencies.sensor_calibration_dataset_id = "t21-sensor-proof-v1";
  result.dependencies.execution_uncertainty_calibration_dataset_id =
      "t21-execution-proof-v1";
  result.dependencies.margin_calibration_dataset_id = "t21-margin-proof-v1";
  result.margin_budget.certification_version = 5;
  result.margin_budget.calibration_dataset_id = "t21-margin-proof-v1";
  result.margin_budget.state_binning_stddev_m = 0.001;
  result.margin_budget.numerical_integration_stddev_m = 0.002;
  result.margin_budget.reference_normal_sweep_stddev_m = 0.003;
  result.margin_budget.statistical_quantile_stddev_m = 0.004;
  result.segments = {{0.0, 8.0, 0.0025, 0.05}};
  result.generation_timestamp = kTimestamp;
  result.path_joint_risk_implemented = false;
  result.risk_semantics = kPointwiseEnvelopeRiskSemantics;
  return result;
}

EnvelopeCoverageCertification certification() {
  return {6,
          "t21-independent-envelope-audit-v1",
          true,
          kTimestamp,
          kValidUntil,
          91,
          envelope().dependencies};
}

EnvelopeLookupKey envelope_key() {
  return {11, SensorHealthMode::nominal, "flat-straight-domain-v1", 20, 7};
}

LockedCableUncertaintyEnvelope lock_search_envelope(
    CableUncertaintyEnvelopeManager& manager) {
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  return *locked;
}

HybridAStarPlanningRequest request(
    const LockedCableUncertaintyEnvelope& locked) {
  HybridAStarPlanningRequest result;
  result.start_state = {{1.0, 0.0, 0.0, kTimestamp}, 0.5, 0.0,
                        kTimestamp, 1};
  result.initial_cable_state = {CableStateKind::tracked,
                                0.0,
                                0.01,
                                kTimestamp,
                                {},
                                2};
  result.initial_reference_progress = {11, 0.0, kTimestamp, 3};
  result.reference_line = reference_line();
  result.cable_context.current_telemetry =
      {0.5, 0.0, 40.0, kTimestamp, 4};
  result.cable_context.execution_envelope = execution_envelope();
  result.cable_context.mode = PredictionMode::search;
  result.cable_context.sensor_mode = SensorHealthMode::nominal;
  result.cable_context.uncertainty_envelope_version = 91;
  result.cable_context.uncertainty_envelope_generator_version = 10;
  result.cable_context.robot_uncertainty_profile_version = 1;
  result.primitive_sweep_context = primitive_sweep_context();
  result.locked_uncertainty_envelope = locked;
  MergeGoal goal;
  goal.robot_pose = {5.0, 0.0, 0.0, kTimestamp};
  goal.cable_lag_angle_rad = 0.0;
  goal.reference_progress_m = 4.0;
  goal.reference_line_version = 11;
  goal.touchdown_target_m = {4.0, 0.0};
  goal.cable_heading_rad = 0.0;
  goal.merge_distance_m = 4.0;
  goal.generation_parameters_version = 8;
  goal.cable_model_version = 20;
  goal.robot_operating_area_version = 22;
  result.goals = {goal};
  result.planning_timestamp = kPlanningTime;
  result.random_seed = 2121;
  return result;
}

std::size_t map_index(const MapSnapshot& map, const double x_m,
                      const double y_m) {
  const auto column = static_cast<std::size_t>(
      std::floor((x_m - map.origin_x_m) / map.resolution_m));
  const auto row = static_cast<std::size_t>(
      std::floor((y_m - map.origin_y_m) / map.resolution_m));
  return row * map.width + column;
}

void an_obstacle_between_clear_primitive_endpoints_is_rejected() {
  // Design: 18.2.3-6
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  HybridAStarSearchParameters hard_gate = search_parameters();
  hard_gate.path_curvature_cost_weight = 0.0;
  hard_gate.touchdown_center_cost_weight = 0.0;
  hard_gate.touchdown_margin_cost_weight = 0.0;
  hard_gate.robot_terrain_cost_weight = 0.0;
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      hard_gate, corridor_policy(), manager);
  HybridAStarPlanningRequest blocked = request(*locked);
  blocked.goals.front().robot_pose.x_m = 2.0;
  blocked.goals.front().reference_progress_m = 1.0;
  blocked.goals.front().touchdown_target_m = {1.0, 0.0};
  MapSnapshot& map = blocked.primitive_sweep_context.map;
  map.cells[map_index(map, 1.5, 0.0)].obstacle = true;

  const HybridAStarPlanningResult result = planner.plan(blocked);

  require(result.state == PlanningState::no_solution &&
              result.robot_path.points.empty() &&
              result.diagnostics.collision_rejection_count > 0U &&
              result.diagnostics.maximum_robot_sweep_spacing_m <= 0.05 +
                  1.0e-12,
          "a primitive crossed a midpoint obstacle despite clear endpoints");
}

void a_forbidden_cell_between_clear_touchdown_endpoints_is_rejected() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      search_parameters(), corridor_policy(), manager);
  HybridAStarPlanningRequest blocked = request(*locked);
  blocked.goals.front().robot_pose.x_m = 2.0;
  blocked.goals.front().reference_progress_m = 1.0;
  blocked.goals.front().touchdown_target_m = {1.0, 0.0};
  const std::size_t forbidden_index = map_index(
      blocked.primitive_sweep_context.map, 0.5, 0.0);
  blocked.primitive_sweep_context.terrain.cable_laying
      .cells[forbidden_index]
      .forbidden = true;

  const HybridAStarPlanningResult result = planner.plan(blocked);

  require(result.state == PlanningState::no_solution &&
              result.robot_path.points.empty() &&
              result.diagnostics.cable_laying_rejection_count > 0U &&
              result.diagnostics.worst_constraint.recorded &&
              result.diagnostics.worst_constraint.reason == "cable_laying" &&
              std::isfinite(
                  result.diagnostics.worst_constraint.position_m.x_m),
          "a touchdown segment crossed a forbidden midpoint cell");
}

void a_directional_slope_between_clear_primitive_endpoints_is_rejected() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      search_parameters(), corridor_policy(), manager);
  HybridAStarPlanningRequest blocked = request(*locked);
  blocked.goals.front().robot_pose.x_m = 2.0;
  blocked.goals.front().reference_progress_m = 1.0;
  blocked.goals.front().touchdown_target_m = {1.0, 0.0};
  const std::size_t steep_index =
      map_index(blocked.primitive_sweep_context.map, 1.5, 0.0);
  blocked.primitive_sweep_context.terrain.surface.cells[steep_index]
      .gradient_x = std::tan(1.0);

  const HybridAStarPlanningResult result = planner.plan(blocked);

  require(result.state == PlanningState::no_solution &&
              result.diagnostics.collision_rejection_count == 0U &&
              result.diagnostics.traversability_rejection_count > 0U,
          "a primitive crossed an excessive directional slope at its midpoint");
}

void curved_primitive_samples_follow_the_exact_constant_curvature_arc() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  HybridAStarSearchParameters curved_search = search_parameters();
  const double half_pi = 0.5 * std::acos(-1.0);
  curved_search.goal_heading_tolerance_rad = 0.01;
  curved_search.goal_lag_tolerance_rad = 3.0;
  curved_search.goal_progress_tolerance_m = 8.0;
  curved_search.goal_touchdown_position_tolerance_m = 10.0;
  curved_search.motion_primitives = {{1U, half_pi, 1.0}};
  CableCorridorRiskPolicy wide_corridor = corridor_policy();
  wide_corridor.nominal_half_width_m = 4.0;
  wide_corridor.absolute_half_width_m = 5.0;
  ParameterConfig curved_progress_config = parameter_config();
  curved_progress_config.search.reference_progress_backward_tolerance_m = 2.0;
  curved_progress_config.search.reference_progress_maximum_ratio = 5.0;
  curved_progress_config.search.reference_progress_forward_slack_m = 2.0;
  CableModelParameters curved_model = model_parameters();
  curved_model.maximum_lag_angle_rad = 2.0;
  const HybridAStarPlanner planner(
      curved_model,
      make_reference_progress_association_parameters(curved_progress_config),
      curved_search, wide_corridor, manager);
  HybridAStarPlanningRequest curved = request(*locked);
  curved.goals.front().robot_pose = {2.0, 1.0, half_pi, kTimestamp};
  curved.goals.front().reference_progress_m = 0.0;
  curved.goals.front().cable_lag_angle_rad = 0.0;
  curved.primitive_sweep_context.cable_laying_limits
      .maximum_curvature_per_m = 100.0;

  const HybridAStarPlanningResult result = planner.plan(curved);

  require(result.state == PlanningState::success &&
              result.robot_path.points.size() > 3U,
          "the curved primitive did not produce a successful adaptive sweep: "
          "state=" + std::to_string(static_cast<int>(result.state)) +
              " collision=" +
              std::to_string(result.diagnostics.collision_rejection_count) +
              " terrain=" +
              std::to_string(
                  result.diagnostics.traversability_rejection_count) +
              " corridor=" +
              std::to_string(result.diagnostics.corridor_rejection_count) +
              " laying=" +
              std::to_string(result.diagnostics.cable_laying_rejection_count));
  const PathPoint& midpoint =
      result.robot_path.points[result.robot_path.points.size() / 2U];
  require_near(midpoint.arc_length_m, 0.5 * half_pi, 1.0e-12,
               "the exact curved sweep omitted its arc-length midpoint");
  require_near(midpoint.x_m, 1.0 + std::sin(0.5 * half_pi), 1.0e-12,
               "the curved sweep midpoint was placed on a chord");
  require_near(midpoint.y_m, 1.0 - std::cos(0.5 * half_pi), 1.0e-12,
               "the curved sweep midpoint did not follow the analytic arc");
  require_near(result.diagnostics.maximum_collision_sweep_margin_m, 0.025,
               1.0e-12,
               "collision sweep margin shrank below eta times map resolution "
               "over two");
}

void a_turning_footprint_corner_sweep_rejects_an_obstacle() {
  // Design: 18.2.3-7
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");

  const double half_pi = 0.5 * std::acos(-1.0);
  HybridAStarSearchParameters curved_search = search_parameters();
  curved_search.goal_heading_tolerance_rad = 0.01;
  curved_search.goal_lag_tolerance_rad = 3.0;
  curved_search.goal_progress_tolerance_m = 8.0;
  curved_search.goal_touchdown_position_tolerance_m = 10.0;
  curved_search.motion_primitives = {{1U, half_pi, 1.0}};
  ParameterConfig curved_progress = parameter_config();
  curved_progress.search.reference_progress_backward_tolerance_m = 2.0;
  curved_progress.search.reference_progress_maximum_ratio = 5.0;
  curved_progress.search.reference_progress_forward_slack_m = 2.0;
  CableModelParameters curved_model = model_parameters();
  curved_model.maximum_lag_angle_rad = 2.0;
  const HybridAStarPlanner planner(
      curved_model,
      make_reference_progress_association_parameters(curved_progress),
      curved_search, wide_corridor_policy(), manager);
  HybridAStarPlanningRequest blocked = request(*locked);
  blocked.goals.front().robot_pose = {2.0, 1.0, half_pi, kTimestamp};
  blocked.goals.front().reference_progress_m = 0.0;
  blocked.goals.front().cable_lag_angle_rad = 0.0;
  blocked.primitive_sweep_context.cable_laying_limits
      .maximum_curvature_per_m = 100.0;
  blocked.primitive_sweep_context.track_footprint = {
      {{-0.3, -0.2}, {0.3, -0.2}, {0.3, 0.2}, {-0.3, 0.2}},
      {{-0.25, 0.05}, {0.25, 0.05}, {0.25, 0.18}, {-0.25, 0.18}},
      {{-0.25, -0.18}, {0.25, -0.18}, {0.25, -0.05},
       {-0.25, -0.05}}};
  blocked.primitive_sweep_context.map
      .cells[map_index(blocked.primitive_sweep_context.map, 2.05, 0.35)]
      .obstacle = true;

  require(std::abs(std::hypot(2.05 - 1.0, 0.35 - 1.0) - 1.0) > 0.2,
          "corner-sweep obstacle fixture overlaps the robot center arc");

  const HybridAStarPlanningResult result = planner.plan(blocked);

  require(result.state == PlanningState::no_solution &&
              result.robot_path.points.empty() &&
              result.diagnostics.collision_rejection_count > 0U,
          "a turning footprint outer corner swept through an obstacle");
}

void primitive_length_and_touchdown_sampling_density_preserve_cost_and_gates() {
  // Design: 18.2.3-9
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");

  CableModelParameters dense_model = model_parameters();
  dense_model.search_integration_step_m = 0.02;
  HybridAStarSearchParameters long_primitives = search_parameters();
  long_primitives.motion_primitives = {{2U, 2.0, 0.0}};
  const HybridAStarPlanner baseline_planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      search_parameters(), corridor_policy(), manager);
  const HybridAStarPlanner dense_planner(
      dense_model,
      make_reference_progress_association_parameters(parameter_config()),
      search_parameters(), corridor_policy(), manager);
  const HybridAStarPlanner long_primitive_planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      long_primitives, corridor_policy(), manager);
  HybridAStarPlanningRequest common = request(*locked);
  for (CableLayingTerrainCell& cell :
       common.primitive_sweep_context.terrain.cable_laying.cells) {
    cell.roughness_m = 0.05;
  }

  const HybridAStarPlanningResult baseline = baseline_planner.plan(common);
  const HybridAStarPlanningResult dense = dense_planner.plan(common);
  const HybridAStarPlanningResult longer = long_primitive_planner.plan(common);

  require(baseline.state == PlanningState::success &&
              dense.state == PlanningState::success &&
              longer.state == PlanningState::success &&
              baseline.diagnostics.solution_cost > 4.0,
          "sampling or primitive length changed a hard feasibility gate");
  require_near(dense.diagnostics.solution_cost,
               baseline.diagnostics.solution_cost, 1.0e-10,
               "touchdown integration density changed the accumulated cost");
  require_near(longer.diagnostics.solution_cost,
               baseline.diagnostics.solution_cost, 1.0e-10,
               "primitive length changed the accumulated cost");
}

void feasible_solution_cost_is_explicit_and_touchdown_only() {
  // Design: 18.2.3-19
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");

  HybridAStarSearchParameters weighted = search_parameters();
  weighted.path_length_cost_weight = 2.0;
  weighted.path_curvature_cost_weight = 3.0;
  weighted.touchdown_center_cost_weight = 4.0;
  weighted.touchdown_margin_cost_weight = 5.0;
  weighted.robot_terrain_cost_weight = 6.0;

  HybridAStarPlanningRequest centered = request(*locked);
  for (SurfaceEstimate& cell :
       centered.primitive_sweep_context.terrain.surface.cells) {
    cell.detrended_roughness_rms_m = 0.02;
  }
  for (CableLayingTerrainCell& cell :
       centered.primitive_sweep_context.terrain.cable_laying.cells) {
    cell.roughness_m = 0.05;
  }
  const HybridAStarPlanner centered_planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      weighted, corridor_policy(), manager);
  const HybridAStarPlanningResult centered_result =
      centered_planner.plan(centered);

  CableModelParameters offset_model = model_parameters();
  offset_model.release_point_offset_m.y_m = 1.0;
  HybridAStarPlanningRequest offset = centered;
  offset.start_state.pose.y_m = -1.0;
  offset.goals.front().robot_pose.y_m = -1.0;
  const HybridAStarPlanner offset_planner(
      offset_model,
      make_reference_progress_association_parameters(parameter_config()),
      weighted, corridor_policy(), manager);
  const HybridAStarPlanningResult offset_result = offset_planner.plan(offset);

  CableModelParameters shifted_touchdown_model = model_parameters();
  shifted_touchdown_model.release_point_offset_m.y_m = 0.05;
  HybridAStarPlanningRequest shifted_touchdown = centered;
  shifted_touchdown.goals.front().touchdown_target_m.y_m = 0.05;
  const HybridAStarPlanner shifted_touchdown_planner(
      shifted_touchdown_model,
      make_reference_progress_association_parameters(parameter_config()),
      weighted, corridor_policy(), manager);
  const HybridAStarPlanningResult shifted_touchdown_result =
      shifted_touchdown_planner.plan(shifted_touchdown);

  require(centered_result.state == PlanningState::success &&
              offset_result.state == PlanningState::success &&
              shifted_touchdown_result.state == PlanningState::success,
          "explicit-cost comparison paths were not both feasible");
  const HybridAStarCostComponents& costs =
      centered_result.diagnostics.solution_cost_components;
  require_near(costs.robot_length, 8.0, 1.0e-12,
               "robot length cost omitted its configured weight");
  require_near(costs.robot_curvature, 0.0, 1.0e-12,
               "straight primitives accumulated a turning cost");
  require_near(costs.touchdown_corridor, 0.0, 1.0e-12,
               "reference-aligned touchdown accumulated corridor cost");
  require(costs.cable_suitability > 0.0 && costs.robot_terrain > 0.0,
          "feasible cable and robot terrain soft costs were not recorded");
  require_near(costs.total(), centered_result.diagnostics.solution_cost,
               1.0e-12, "solution cost contains a hidden or duplicated term");
  require_near(
      offset_result.diagnostics.solution_cost_components.touchdown_corridor,
      costs.touchdown_corridor, 1.0e-12,
      "robot center displacement changed touchdown corridor cost");
  require_near(offset_result.diagnostics.solution_cost,
               centered_result.diagnostics.solution_cost, 1.0e-12,
               "robot center distance to the cable reference line was charged");
  require_near(
      shifted_touchdown_result.diagnostics.solution_cost_components
          .touchdown_corridor,
      0.04, 1.0e-10,
      "touchdown center deviation was not integrated exactly once");
}

void dubins_heuristic_is_an_admissible_robot_goal_lower_bound() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");

  constexpr double half_pi = 1.5707963267948966;
  HybridAStarSearchParameters curved = search_parameters();
  curved.path_length_cost_weight = 2.0;
  curved.path_curvature_cost_weight = 3.0;
  curved.touchdown_center_cost_weight = 0.0;
  curved.touchdown_margin_cost_weight = 0.0;
  curved.robot_terrain_cost_weight = 0.0;
  curved.goal_lag_tolerance_rad = 3.0;
  curved.goal_progress_tolerance_m = 8.0;
  curved.goal_touchdown_position_tolerance_m = 10.0;
  curved.motion_primitives = {{1U, half_pi, 1.0}};
  CableCorridorRiskPolicy wide = corridor_policy();
  wide.nominal_half_width_m = 4.0;
  wide.absolute_half_width_m = 5.0;
  ParameterConfig curved_progress = parameter_config();
  curved_progress.search.reference_progress_backward_tolerance_m = 2.0;
  curved_progress.search.reference_progress_maximum_ratio = 5.0;
  curved_progress.search.reference_progress_forward_slack_m = 2.0;
  CableModelParameters curved_model = model_parameters();
  curved_model.maximum_lag_angle_rad = 2.0;
  const HybridAStarPlanner planner(
      curved_model,
      make_reference_progress_association_parameters(curved_progress),
      curved, wide, manager);
  HybridAStarPlanningRequest quarter_turn = request(*locked);
  quarter_turn.goals.front().robot_pose = {2.0, 1.0, half_pi, kTimestamp};
  quarter_turn.goals.front().reference_progress_m = 0.0;
  quarter_turn.primitive_sweep_context.cable_laying_limits
      .maximum_curvature_per_m = 100.0;

  const HybridAStarPlanningResult result = planner.plan(quarter_turn);

  require(result.state == PlanningState::success,
          "known Dubins quarter-turn path was not feasible");
  require_near(result.diagnostics.initial_heuristic_cost,
               2.0 * (half_pi - curved.goal_heading_tolerance_rad),
               1.0e-10,
               "initial heuristic was not a goal-region-safe kinematic "
               "lower bound: " +
                   std::to_string(
                       result.diagnostics.initial_heuristic_cost));
  require(result.diagnostics.initial_heuristic_cost > 2.0 * std::sqrt(2.0) &&
              result.diagnostics.initial_heuristic_cost <=
                  result.diagnostics.solution_cost + 1.0e-12,
          "Dubins heuristic was Euclidean-only or overestimated a feasible "
          "solution");
  require_near(result.diagnostics.solution_cost_components.robot_curvature,
               3.0 * half_pi, 1.0e-10,
               "curved primitive omitted its weighted turning cost");
}

void heuristic_is_zero_inside_the_robot_goal_tolerances() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      search_parameters(), corridor_policy(), manager);
  HybridAStarPlanningRequest already_at_goal = request(*locked);
  already_at_goal.goals.front().robot_pose = {
      already_at_goal.start_state.pose.x_m + 0.005,
      already_at_goal.start_state.pose.y_m,
      already_at_goal.start_state.pose.heading_rad + 0.005, kTimestamp};
  already_at_goal.goals.front().cable_lag_angle_rad = 0.0;
  already_at_goal.goals.front().reference_progress_m = 0.0;
  already_at_goal.goals.front().touchdown_target_m = {0.0, 0.0};

  const HybridAStarPlanningResult result = planner.plan(already_at_goal);

  require(result.state == PlanningState::success,
          "a state inside every goal tolerance did not terminate");
  require_near(result.diagnostics.initial_heuristic_cost, 0.0, 1.0e-12,
               "the heuristic overestimated zero remaining goal-region cost");
}

void touchdown_corridor_cost_uses_robot_primitive_arc_length() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");

  constexpr double half_pi = 1.5707963267948966;
  HybridAStarSearchParameters curved = search_parameters();
  curved.path_curvature_cost_weight = 0.0;
  curved.touchdown_center_cost_weight = 1.0;
  curved.touchdown_margin_cost_weight = 0.0;
  curved.robot_terrain_cost_weight = 0.0;
  curved.goal_lag_tolerance_rad = 3.0;
  curved.goal_progress_tolerance_m = 8.0;
  curved.goal_touchdown_position_tolerance_m = 10.0;
  curved.motion_primitives = {{1U, half_pi, 1.0}};
  CableCorridorRiskPolicy wide = corridor_policy();
  wide.nominal_half_width_m = 4.0;
  wide.absolute_half_width_m = 5.0;
  ParameterConfig curved_progress = parameter_config();
  curved_progress.search.reference_progress_backward_tolerance_m = 2.0;
  curved_progress.search.reference_progress_maximum_ratio = 5.0;
  curved_progress.search.reference_progress_forward_slack_m = 2.0;
  CableModelParameters curved_model = model_parameters();
  curved_model.maximum_lag_angle_rad = 2.0;
  const HybridAStarPlanner planner(
      curved_model,
      make_reference_progress_association_parameters(curved_progress), curved,
      wide, manager);
  HybridAStarPlanningRequest request_with_turn = request(*locked);
  request_with_turn.goals.front().robot_pose = {2.0, 1.0, half_pi,
                                                kTimestamp};
  request_with_turn.goals.front().reference_progress_m = 0.0;
  request_with_turn.primitive_sweep_context.cable_laying_limits
      .maximum_curvature_per_m = 100.0;

  const HybridAStarPlanningResult result = planner.plan(request_with_turn);

  require(result.state == PlanningState::success &&
              result.robot_path.points.size() ==
                  result.touchdown_path.points.size(),
          "curved cost test did not retain aligned robot/touchdown samples");
  double expected = 0.0;
  for (std::size_t index = 1U; index < result.robot_path.points.size();
       ++index) {
    const double left_error_m = result.touchdown_path.points[index - 1U].y_m;
    const double right_error_m = result.touchdown_path.points[index].y_m;
    const double midpoint_error_m = 0.5 * (left_error_m + right_error_m);
    const double robot_interval_m =
        result.robot_path.points[index].arc_length_m -
        result.robot_path.points[index - 1U].arc_length_m;
    expected += midpoint_error_m * midpoint_error_m * robot_interval_m;
  }
  require_near(
      result.diagnostics.solution_cost_components.touchdown_corridor, expected,
      1.0e-10,
      "touchdown corridor density was integrated over touchdown geometry "
      "instead of robot primitive length");
}

void rejected_successors_have_complete_reason_counts() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");

  HybridAStarSearchParameters curved = search_parameters();
  curved.motion_primitives = {{1U, 1.0, 1.0}};
  CableModelParameters narrow_model = model_parameters();
  narrow_model.maximum_lag_angle_rad = 0.01;
  const HybridAStarPlanner model_rejecting_planner(
      narrow_model,
      make_reference_progress_association_parameters(parameter_config()),
      curved, corridor_policy(), manager);
  const HybridAStarPlanningResult model_rejected =
      model_rejecting_planner.plan(request(*locked));

  ParameterConfig ambiguous_progress = parameter_config();
  ambiguous_progress.search.reference_progress_maximum_ratio = 5.0;
  ambiguous_progress.search.reference_progress_forward_slack_m = 3.0;
  const HybridAStarPlanner association_rejecting_planner(
      model_parameters(),
      make_reference_progress_association_parameters(ambiguous_progress),
      search_parameters(), corridor_policy(), manager);
  HybridAStarPlanningRequest ambiguous_reference = request(*locked);
  ambiguous_reference.reference_line = make_reference_line(
      11, "map", {{0.0, 0.0}, {1.0, 0.0}, {0.0, 0.0}, {8.0, 0.0}});
  const HybridAStarPlanningResult association_rejected =
      association_rejecting_planner.plan(ambiguous_reference);

  CableUncertaintyEnvelopeManager short_envelope_manager;
  CableUncertaintyEnvelope short_envelope = envelope();
  short_envelope.segments = {{0.0, 0.1, 0.0025, 0.05}};
  require(short_envelope_manager
              .registerValidated(91, short_envelope, certification())
              .accepted(),
          "short test envelope registration failed");
  static_cast<void>(short_envelope_manager.setCurrentContext(
      envelope_key(), 1, kPlanningTime));
  const auto short_locked =
      short_envelope_manager.getValidated(envelope_key(), kPlanningTime);
  require(short_locked.has_value(), "short test envelope did not lock");
  const HybridAStarPlanner envelope_rejecting_planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      search_parameters(), corridor_policy(), short_envelope_manager);
  const HybridAStarPlanningResult envelope_rejected =
      envelope_rejecting_planner.plan(request(*short_locked));

  require(model_rejected.state == PlanningState::no_solution &&
              model_rejected.diagnostics.cable_model_rejection_count > 0U,
          "cable-model successor rejection was not counted");
  require(association_rejected.state == PlanningState::no_solution &&
              association_rejected.diagnostics
                      .reference_association_rejection_count > 0U,
          "reference-association successor rejection was not counted");
  require(envelope_rejected.state ==
                  PlanningState::no_solution_under_covariance_envelope &&
              envelope_rejected.diagnostics
                      .envelope_unavailable_rejection_count > 0U,
          "mid-primitive envelope-unavailable rejection was not counted");
}

void a_corridor_excursion_between_legal_touchdown_endpoints_is_rejected() {
  // Design: 18.2.3-8
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  ParameterConfig local_progress = parameter_config();
  local_progress.search.reference_progress_maximum_ratio = 3.0;
  local_progress.search.reference_progress_forward_slack_m = 0.1;
  HybridAStarSearchParameters one_step = search_parameters();
  one_step.goal_progress_tolerance_m = 8.0;
  one_step.touchdown_center_cost_weight = 1.0e12;
  one_step.touchdown_margin_cost_weight = 1.0e12;
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(local_progress), one_step,
      corridor_policy(), manager);
  HybridAStarPlanningRequest deviating = request(*locked);
  deviating.reference_line = make_reference_line(
      11U, "map", {{0.0, 0.0}, {0.5, 1.0}, {1.0, 0.0}, {8.0, 0.0}});
  deviating.goals.front().robot_pose.x_m = 2.0;
  deviating.goals.front().reference_progress_m = 0.0;
  deviating.goals.front().touchdown_target_m = {1.0, 0.0};

  const HybridAStarPlanningResult result = planner.plan(deviating);

  require(result.state ==
                  PlanningState::no_solution_under_covariance_envelope &&
               result.diagnostics.corridor_rejection_count > 0U &&
               result.diagnostics.cable_sweep_margin_m == 0.01 &&
               result.diagnostics.worst_constraint.recorded &&
               result.diagnostics.worst_constraint.reason ==
                   "cable_corridor" &&
               result.diagnostics.worst_constraint.constraint_value >=
                   result.diagnostics.worst_constraint.hard_limit,
          "a touchdown segment crossed an intermediate absolute corridor "
          "violation despite legal endpoints");
}

void mismatched_map_and_terrain_grid_geometry_fails_before_expansion() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      search_parameters(), corridor_policy(), manager);
  HybridAStarPlanningRequest mismatched = request(*locked);
  mismatched.primitive_sweep_context.terrain.surface.origin_x_m += 0.05;

  const HybridAStarPlanningResult result = planner.plan(mismatched);

  require(result.state == PlanningState::input_invalid &&
              result.diagnostics.expanded_state_count == 0U &&
              !result.diagnostics.entries.empty() &&
              result.diagnostics.entries.back().code ==
                  "HYBRID_ASTAR_COLLISION_LAYER_INVALID",
          "map and terrain grids with different physical geometry were not "
          "rejected before search");
}

void a_narrow_operating_area_notch_between_sweep_samples_is_rejected() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      search_parameters(), corridor_policy(), manager);
  HybridAStarPlanningRequest notched = request(*locked);
  notched.goals.front().robot_pose.x_m = 2.0;
  notched.goals.front().reference_progress_m = 1.0;
  notched.goals.front().touchdown_target_m = {1.0, 0.0};
  notched.primitive_sweep_context.track_footprint = tiny_track_footprint();
  notched.primitive_sweep_context.robot_operating_area.polygon = {
      {-3.5, -3.5}, {9.5, -3.5}, {9.5, 3.5},
      {1.5275, 3.5}, {1.5275, 0.004}, {1.5225, 0.004},
      {1.5225, 3.5}, {-3.5, 3.5}};

  const HybridAStarPlanningResult result = planner.plan(notched);

  require(result.state == PlanningState::no_solution &&
              result.diagnostics.operating_area_rejection_count > 0U &&
              result.diagnostics.collision_rejection_count == 0U &&
              result.diagnostics.worst_constraint.recorded &&
              result.diagnostics.worst_constraint.reason ==
                  "robot_operating_area",
          "a swept footprint crossed a narrow operating-area notch between "
          "sample poses");
}

void climbable_terrain_remains_feasible_and_is_soft_ranked() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  HybridAStarSearchParameters weighted = search_parameters();
  weighted.robot_terrain_cost_weight = 7.0;
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      weighted, corridor_policy(), manager);
  HybridAStarPlanningRequest climbable = request(*locked);
  for (SurfaceEstimate& cell :
       climbable.primitive_sweep_context.terrain.surface.cells) {
    cell.gradient_x = std::tan(0.1);
    cell.slope_angle_rad = 0.1;
    cell.detrended_roughness_rms_m = 0.03;
  }

  const HybridAStarPlanningResult result = planner.plan(climbable);

  require(result.state == PlanningState::success &&
              result.diagnostics.traversability_rejection_count == 0U &&
              result.diagnostics.solution_cost_components.robot_terrain > 0.0,
          "climbable terrain was treated as a hard rejection or omitted from "
          "soft ranking");
}

void nonfinite_robot_terrain_cost_input_fails_before_search() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      search_parameters(), corridor_policy(), manager);
  HybridAStarPlanningRequest invalid = request(*locked);
  invalid.primitive_sweep_context.terrain.surface.cells.front()
      .detrended_roughness_rms_m =
      std::numeric_limits<double>::quiet_NaN();

  const HybridAStarPlanningResult result = planner.plan(invalid);

  require(result.state == PlanningState::input_invalid &&
              result.diagnostics.expanded_state_count == 0U,
          "non-finite robot terrain cost input reached the search queue");
}

void flat_straight_search_returns_a_reference_aligned_touchdown_path() {
  // Design: 18.2.3-1
  // Design: 18.2.4-19
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  require(manager.setCurrentContext(envelope_key(), 1, kPlanningTime)
              .context_changed,
          "test envelope context was not activated");
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");

  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      search_parameters(), corridor_policy(), manager);
  const HybridAStarPlanningResult result = planner.plan(request(*locked));

  require(result.state == PlanningState::success,
          "flat straight search did not find the merge goal");
  require(validate(result.robot_path).valid &&
              validate(result.touchdown_path).valid,
          "the returned robot or touchdown geometry violates its public path "
          "contract");
  require(result.robot_path.points.size() > 5U &&
              result.touchdown_path.points.size() >= 5U &&
              result.state_trace.size() == 5U &&
              result.diagnostics.maximum_robot_sweep_pose_count > 2U,
          "the returned path did not preserve adaptive primitive sweep samples "
          "and primitive-boundary states");
  require_near(result.robot_path.points.front().x_m, 1.0, 1.0e-12,
               "robot path did not start from the supplied state");
  require_near(result.robot_path.points.back().x_m, 5.0, 1.0e-12,
               "robot path did not end at the inverse merge goal");
  require_near(result.touchdown_path.points.front().x_m, 0.0, 1.0e-12,
               "initial touchdown mean was not propagated from the robot state");
  require_near(result.touchdown_path.points.back().x_m, 4.0, 1.0e-12,
               "terminal touchdown mean missed the reference merge point");
  require_near(result.terminal_reference_progress.arc_length_m, 4.0, 1.0e-12,
               "task progress did not advance with the touchdown path");
  require_near(result.terminal_cable_state.lag_angle_rad, 0.0, 1.0e-12,
               "straight primitives changed the zero cable lag mean");
  require(result.terminal_cable_state.kind == CableStateKind::search_mean &&
              !result.terminal_cable_state.lag_angle_variance_rad2.has_value() &&
              !result.terminal_cable_state.laying_memory
                   .trailing_support_samples.empty(),
          "search incorrectly propagated path-dependent cable covariance");
  require(result.state_trace.front().base_key.x_index == 2 &&
              result.state_trace.front().base_key.y_index == 0 &&
              result.state_trace.front().base_key.heading_index == 0 &&
              result.state_trace.front().base_key.cable_lag_index == 0 &&
              result.state_trace.front().base_key.reference_progress_index == 0,
          "the auditable base key did not contain all five discretized states");
  require_near(result.diagnostics.epsilon_point, 0.05, 1.0e-12,
               "diagnostics omitted the corridor pointwise epsilon");
  require_near(result.diagnostics.standard_normal_quantile,
               1.95996398454005, 1.0e-12,
               "diagnostics omitted the two-sided statistical quantile");
  require_near(result.diagnostics.envelope_discretization_margin_m, 0.006,
               1.0e-12,
               "diagnostics merged or omitted the envelope discretization "
               "margin");
  require_near(result.diagnostics.cable_sweep_margin_m, 0.01, 1.0e-12,
               "diagnostics omitted the independent cable sweep margin");
  require_near(result.diagnostics.minimum_turning_radius_m, 1.0, 1.0e-12,
               "diagnostics omitted the Dubins turning radius");
  require_near(result.diagnostics.path_length_cost_weight, 1.0, 1.0e-12,
               "diagnostics omitted the path length weight");
  require_near(result.diagnostics.maximum_envelope_stddev_upper_bound_m, 0.06,
               1.0e-12,
               "diagnostics omitted the queried envelope standard-deviation "
               "bound");
  require(result.diagnostics.envelope_query_count > 0U &&
              result.diagnostics.uncertainty_envelope_version == 91 &&
              result.diagnostics.corridor_risk_policy_version == 41 &&
              result.diagnostics.terrain_map_sequence == 22U &&
              result.diagnostics.terrain_analysis_config_version == 22U &&
              result.diagnostics.collision_risk_policy_version == 22U &&
              result.diagnostics.terrain_gradient_risk_policy_version == 22U &&
              result.diagnostics.cable_laying_limits_version == 22U &&
              result.diagnostics.robot_operating_area_version == 22U &&
              !result.diagnostics.path_dependent_covariance_propagated &&
              result.diagnostics.risk_semantics ==
                  kPointwiseEnvelopeRiskSemantics,
          "search diagnostics omitted locked-envelope or risk semantics");
}

void robot_and_cable_spatial_domains_are_evaluated_independently() {
  // Design: 18.2.3-16
  CableUncertaintyEnvelopeManager manager;
  const LockedCableUncertaintyEnvelope locked = lock_search_envelope(manager);
  CableModelParameters offset_model = model_parameters();
  offset_model.release_point_offset_m.y_m = -0.4;
  const HybridAStarPlanner planner(
      offset_model,
      make_reference_progress_association_parameters(parameter_config()),
      search_parameters(), corridor_policy(), manager);
  HybridAStarPlanningRequest separated = request(locked);
  separated.start_state.pose.y_m = 0.4;
  separated.goals.front().robot_pose.y_m = 0.4;
  separated.goals.front().touchdown_target_m.y_m = 0.0;

  const HybridAStarPlanningResult result = planner.plan(separated);

  require(result.state == PlanningState::success &&
              result.robot_path.points.front().y_m >
                  corridor_policy().absolute_half_width_m &&
              std::abs(result.touchdown_path.points.front().y_m) < 1.0e-12 &&
              result.diagnostics.operating_area_rejection_count == 0U &&
              result.diagnostics.corridor_rejection_count == 0U,
          "robot center was incorrectly constrained to the independent cable corridor");
}

void returned_solution_satisfies_all_segment_hard_invariants() {
  // Design: 18.2.3-invariant-1
  // Design: 18.2.3-invariant-2
  // Design: 18.2.3-invariant-3
  // Design: 18.2.3-invariant-4
  CableUncertaintyEnvelopeManager manager;
  const LockedCableUncertaintyEnvelope locked = lock_search_envelope(manager);
  const HybridAStarSearchParameters parameters = search_parameters();
  const HybridAStarPlanningRequest planning_request = request(locked);
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      parameters, corridor_policy(), manager);
  const HybridAStarPlanningResult result = planner.plan(planning_request);
  require(result.state == PlanningState::success,
          "hard-invariant audit did not receive a successful path");

  MotionSegment robot_motion;
  for (const PathPoint& point : result.robot_path.points) {
    robot_motion.samples.push_back(
        {point.x_m, point.y_m, point.heading_rad, kTimestamp});
    require(std::abs(point.curvature_per_m) <=
                1.0 / parameters.minimum_turning_radius_m + 1.0e-12,
            "returned path exceeds the robot curvature hard limit");
  }
  const auto& sweep = planning_request.primitive_sweep_context;
  const TraversabilityEvaluator traversability(sweep.robot_capability,
                                                sweep.track_footprint);
  const CollisionLayerResult collision_layer =
      traversability.evaluate_collision_layer(
          sweep.map, sweep.terrain,
          sweep.robot_relative_obstacle_covariance_m2,
          sweep.collision_risk_policy);
  const CollisionSweepResult collision = traversability.evaluate_collision_sweep(
      robot_motion, sweep.terrain, collision_layer,
      parameters.maximum_sweep_spacing_fraction);
  const TraversabilityResult terrain = traversability.evaluate(
      robot_motion, sweep.terrain, sweep.terrain_gradient_risk_policy);
  require(collision.validity == CollisionEvaluationValidity::valid &&
              collision.collision_free &&
              terrain.validity == TraversabilityEvaluationValidity::valid &&
              terrain.traversable,
          "returned robot path failed independent swept traversability audit");

  const CablePrediction prediction = CableModel(model_parameters()).predict_search(
      planning_request.initial_cable_state, result.robot_path,
      planning_request.cable_context);
  require(prediction.validity == CableModelValidity::valid,
          "returned path failed independent cable-model replay");
  const CableLayingEvaluation laying = CableLayingEvaluator{}.evaluate(
      planning_request.initial_cable_state.laying_memory,
      prediction.touchdown_path, prediction.state_profile, sweep.terrain,
      sweep.cable_laying_limits, sweep.cable_history_boundary);
  require(laying.valid && laying.hard_feasible,
          "returned path failed independent mechanical-history audit");

  for (const PathPoint& touchdown : prediction.touchdown_path.points) {
    const double progress_m = touchdown.x_m;
    const auto reference = planning_request.reference_line.query(progress_m);
    const auto envelope_bound = manager.query(
        locked, progress_m, planning_request.planning_timestamp);
    require(reference.has_value() &&
                envelope_bound.status == EnvelopeQueryStatus::valid,
            "returned touchdown sample lacks locked reference/envelope evidence");
    const CableCorridorSearchBound corridor = evaluate_search_corridor_bound(
        corridor_policy(), *reference,
        {touchdown.x_m, touchdown.y_m},
        envelope_bound.lateral_stddev_upper_bound_m,
        parameters.cable_sweep_margin_m);
    require(corridor.validity == CorridorEvaluationValidity::valid &&
                corridor.hard_feasible,
            "returned touchdown sample failed independent corridor audit");
  }
}

void validated_analytic_expansion_reaches_an_off_lattice_merge_goal() {
  CableUncertaintyEnvelopeManager manager;
  const LockedCableUncertaintyEnvelope locked = lock_search_envelope(manager);
  HybridAStarSearchParameters analytic = search_parameters();
  analytic.analytic_expansion_interval = 1U;
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      analytic, corridor_policy(), manager);
  HybridAStarPlanningRequest multi_goal = request(locked);
  MergeGoal off_lattice = multi_goal.goals.front();
  off_lattice.robot_pose.x_m = 4.5;
  off_lattice.reference_progress_m = 3.5;
  off_lattice.touchdown_target_m.x_m = 3.5;
  MergeGoal farther = multi_goal.goals.front();
  farther.robot_pose.x_m = 5.5;
  farther.reference_progress_m = 4.5;
  farther.touchdown_target_m.x_m = 4.5;
  multi_goal.goals = {farther, off_lattice};

  const HybridAStarPlanningResult result = planner.plan(multi_goal);

  require(result.state == PlanningState::success &&
              !result.robot_path.points.empty() &&
              result.diagnostics.analytic_expansion_attempt_count >= 2U &&
              result.diagnostics.analytic_expansion_accepted_count >= 1U,
          "a fully feasible analytic connection did not reach an off-lattice "
          "merge goal");
  require_near(result.robot_path.points.back().x_m, 4.5, 1.0e-12,
               "multi-goal analytic search did not select the lower-cost "
               "feasible merge point");
  require_near(result.touchdown_path.points.back().x_m, 3.5, 1.0e-12,
               "analytic expansion did not forward-predict the selected "
               "touchdown target");
  require_near(result.touchdown_path.points.back().y_m, 0.0, 1.0e-12,
               "analytic expansion missed the touchdown target y coordinate");
  require_near(result.terminal_cable_state.lag_angle_rad, 0.0, 1.0e-12,
               "multi-goal search missed the terminal cable lag target");
  require_near(result.terminal_reference_progress.arc_length_m, 3.5, 1.0e-12,
               "multi-goal search missed the terminal progress target");
}

void validated_analytic_expansion_supports_an_exact_curved_connection() {
  CableUncertaintyEnvelopeManager manager;
  const LockedCableUncertaintyEnvelope locked = lock_search_envelope(manager);
  const double half_pi = 0.5 * std::acos(-1.0);
  HybridAStarSearchParameters analytic = search_parameters();
  analytic.analytic_expansion_interval = 1U;
  ParameterConfig progress = parameter_config();
  progress.search.reference_progress_backward_tolerance_m = 2.0;
  progress.search.reference_progress_maximum_ratio = 5.0;
  progress.search.reference_progress_forward_slack_m = 2.0;
  CableModelParameters model = model_parameters();
  model.maximum_lag_angle_rad = 2.0;
  const HybridAStarPlanner planner(
      model, make_reference_progress_association_parameters(progress), analytic,
      wide_corridor_policy(), manager);
  HybridAStarPlanningRequest curved = request(locked);
  curved.goals.front().robot_pose = {2.0, 1.0, half_pi, kTimestamp};
  curved.goals.front().cable_lag_angle_rad = -1.08812;
  curved.goals.front().reference_progress_m = 1.11424;
  curved.goals.front().touchdown_target_m = {1.11424, 0.535852};
  curved.primitive_sweep_context.cable_laying_limits
      .maximum_curvature_per_m = 100.0;

  const HybridAStarPlanningResult result = planner.plan(curved);

  require(result.state == PlanningState::success &&
              result.diagnostics.analytic_expansion_accepted_count > 0U,
          "a feasible exact curved analytic connection was not accepted");
  require_near(result.robot_path.points.back().x_m, 2.0, 1.0e-12,
               "curved analytic expansion missed the goal x coordinate");
  require_near(result.robot_path.points.back().y_m, 1.0, 1.0e-12,
               "curved analytic expansion missed the goal y coordinate");
  require_near(result.robot_path.points.back().heading_rad, half_pi, 1.0e-12,
               "curved analytic expansion missed the goal heading");
  require_near(result.terminal_cable_state.lag_angle_rad,
               curved.goals.front().cable_lag_angle_rad, 1.0e-5,
               "curved analytic expansion missed the cable lag target");
  require_near(result.terminal_reference_progress.arc_length_m,
               curved.goals.front().reference_progress_m, 1.0e-5,
               "curved analytic expansion missed the progress target");
}

void validated_analytic_expansion_follows_a_multi_segment_dubins_connection() {
  CableUncertaintyEnvelopeManager manager;
  const LockedCableUncertaintyEnvelope locked = lock_search_envelope(manager);
  const double half_pi = 0.5 * std::acos(-1.0);
  HybridAStarSearchParameters analytic = search_parameters();
  analytic.analytic_expansion_interval = 1U;
  ParameterConfig progress = parameter_config();
  progress.search.reference_progress_backward_tolerance_m = 2.0;
  progress.search.reference_progress_maximum_ratio = 5.0;
  progress.search.reference_progress_forward_slack_m = 2.0;
  CableModelParameters model = model_parameters();
  model.maximum_lag_angle_rad = 2.0;
  const HybridAStarPlanner planner(
      model, make_reference_progress_association_parameters(progress), analytic,
      wide_corridor_policy(), manager);
  HybridAStarPlanningRequest curved = request(locked);
  curved.goals.front().robot_pose = {4.0, 2.0, half_pi, kTimestamp};
  curved.goals.front().cable_lag_angle_rad = -0.86579;
  curved.goals.front().reference_progress_m = 3.23839;
  curved.goals.front().touchdown_target_m = {3.23839, 1.35196};
  curved.primitive_sweep_context.cable_laying_limits
      .maximum_curvature_per_m = 100.0;

  const HybridAStarPlanningResult result = planner.plan(curved);

  require(result.state == PlanningState::success &&
              result.state_trace.size() >= 3U &&
              result.diagnostics.analytic_expansion_accepted_count >= 2U,
          "a feasible multi-segment Dubins analytic connection was not "
          "validated segment by segment");
  require_near(result.robot_path.points.back().x_m, 4.0, 1.0e-10,
               "multi-segment Dubins expansion missed the goal x coordinate");
  require_near(result.robot_path.points.back().y_m, 2.0, 1.0e-10,
               "multi-segment Dubins expansion missed the goal y coordinate");
  require_near(result.robot_path.points.back().heading_rad, half_pi, 1.0e-10,
               "multi-segment Dubins expansion missed the goal heading");
  require_near(result.terminal_cable_state.lag_angle_rad,
               curved.goals.front().cable_lag_angle_rad, 1.0e-5,
               "multi-segment Dubins expansion missed the cable lag target");
  require_near(result.terminal_reference_progress.arc_length_m,
               curved.goals.front().reference_progress_m, 1.0e-5,
               "multi-segment Dubins expansion missed the progress target");
}

void a_robot_goal_match_cannot_substitute_for_the_touchdown_goal() {
  // Design: 18.2.3-17
  CableUncertaintyEnvelopeManager manager;
  const LockedCableUncertaintyEnvelope locked = lock_search_envelope(manager);
  HybridAStarSearchParameters bounded = search_parameters();
  bounded.maximum_expansions = 1U;
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      bounded, corridor_policy(), manager);
  HybridAStarPlanningRequest proxy_goal = request(locked);
  proxy_goal.goals.front().robot_pose = proxy_goal.start_state.pose;
  proxy_goal.goals.front().cable_lag_angle_rad = 0.0;
  proxy_goal.goals.front().reference_progress_m = 0.0;
  proxy_goal.goals.front().touchdown_target_m = {0.2, 0.0};

  const HybridAStarPlanningResult result = planner.plan(proxy_goal);

  require(result.state != PlanningState::success &&
              result.robot_path.points.empty(),
          "a matching robot state was accepted despite missing the touchdown "
          "target");
}

void analytic_expansion_is_rejected_by_a_mid_connection_obstacle() {
  CableUncertaintyEnvelopeManager manager;
  const LockedCableUncertaintyEnvelope locked = lock_search_envelope(manager);
  HybridAStarSearchParameters analytic = search_parameters();
  analytic.analytic_expansion_interval = 1U;
  analytic.maximum_expansions = 1U;
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      analytic, corridor_policy(), manager);
  HybridAStarPlanningRequest blocked = request(locked);
  const std::size_t obstacle_index =
      map_index(blocked.primitive_sweep_context.map, 3.5, 0.0);
  blocked.primitive_sweep_context.map.cells[obstacle_index].obstacle = true;

  const HybridAStarPlanningResult result = planner.plan(blocked);

  require(result.state == PlanningState::timeout &&
              result.diagnostics.analytic_expansion_attempt_count == 1U &&
              result.diagnostics.analytic_expansion_accepted_count == 0U &&
              result.diagnostics.collision_rejection_count > 0U &&
              result.robot_path.points.empty(),
          "analytic expansion bypassed the complete swept-footprint collision "
          "gate");
}

void mixed_obstacle_and_envelope_rejections_are_ordinary_no_solution() {
  CableUncertaintyEnvelopeManager manager;
  const LockedCableUncertaintyEnvelope locked = lock_search_envelope(manager);
  HybridAStarSearchParameters mixed = search_parameters();
  mixed.motion_primitives = {{1U, 1.0, 0.0}, {2U, 1.0, 0.5}};
  CableCorridorRiskPolicy limited = corridor_policy();
  limited.nominal_half_width_m = 0.14;
  limited.absolute_half_width_m = 0.16;
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      mixed, limited, manager);
  HybridAStarPlanningRequest blocked = request(locked);
  blocked.goals.front().robot_pose.x_m = 2.0;
  blocked.goals.front().reference_progress_m = 1.0;
  blocked.goals.front().touchdown_target_m = {1.0, 0.0};
  MapSnapshot& map = blocked.primitive_sweep_context.map;
  map.cells[map_index(map, 1.95, 0.0)].obstacle = true;

  const HybridAStarPlanningResult result = planner.plan(blocked);

  require(result.state == PlanningState::no_solution &&
              result.diagnostics.collision_rejection_count > 0U &&
              result.diagnostics.corridor_rejection_count > 0U &&
              !result.diagnostics.entries.empty() &&
              result.diagnostics.entries.back().code ==
                  "HYBRID_ASTAR_NO_SOLUTION",
          "a mixed hard-constraint failure was falsely attributed only to "
          "the covariance envelope: state=" +
              std::to_string(static_cast<int>(result.state)) +
              " collision=" +
              std::to_string(result.diagnostics.collision_rejection_count) +
              " corridor=" +
              std::to_string(result.diagnostics.corridor_rejection_count));
}

void a_single_open_side_is_used_before_returning_to_a_merge_goal() {
  // Design: 18.2.3-2
  CableUncertaintyEnvelopeManager manager;
  const LockedCableUncertaintyEnvelope locked = lock_search_envelope(manager);
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      detour_search_parameters(), wide_corridor_policy(), manager);
  HybridAStarPlanningRequest one_side = request(locked);
  const double turn_advance_m = std::sin(0.5) / 0.5;
  one_side.goals.front().robot_pose = {
      1.0 + 4.0 * turn_advance_m + 3.0, 0.0, 0.0, kTimestamp};
  one_side.goals.front().reference_progress_m = 7.0;
  one_side.goals.front().touchdown_target_m = {7.0, 0.0};
  one_side.primitive_sweep_context.cable_laying_limits
      .maximum_curvature_per_m = 100.0;
  MapSnapshot& map = one_side.primitive_sweep_context.map;
  for (double x_m = 2.5; x_m <= 3.5; x_m += 0.1) {
    for (double y_m = 0.0; y_m <= 1.0; y_m += 0.1) {
      map.cells[map_index(map, x_m, y_m)].obstacle = true;
    }
  }

  const HybridAStarPlanningResult result = planner.plan(one_side);

  require(result.state == PlanningState::success &&
              result.diagnostics.collision_rejection_count > 0U &&
              result.diagnostics.analytic_expansion_attempt_count > 0U &&
              result.diagnostics.analytic_expansion_accepted_count > 0U,
          "single-side obstacle search did not find a fully validated detour");
  double minimum_y_m = 0.0;
  for (const PathPoint& point : result.robot_path.points) {
    minimum_y_m = std::min(minimum_y_m, point.y_m);
  }
  require(minimum_y_m < -0.2,
          "the search did not use the only open side of the obstacle");
  require_near(result.robot_path.points.back().y_m, 0.0, 0.02,
               "the detour did not return to the feasible merge pose");
  require(std::abs(result.terminal_cable_state.lag_angle_rad) <= 0.06 &&
              std::abs(result.terminal_reference_progress.arc_length_m - 7.0) <=
                  0.2 &&
              std::hypot(result.touchdown_path.points.back().x_m - 7.0,
                         result.touchdown_path.points.back().y_m) <= 0.2,
          "single-side detour missed the complete augmented merge target");
}

void two_open_sides_choose_the_lower_soft_cost_detour() {
  // Design: 18.2.3-3
  CableUncertaintyEnvelopeManager manager;
  const LockedCableUncertaintyEnvelope locked = lock_search_envelope(manager);
  HybridAStarSearchParameters detour = detour_search_parameters();
  detour.robot_terrain_cost_weight = 20.0;
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      detour, wide_corridor_policy(), manager);
  HybridAStarPlanningRequest both_sides = request(locked);
  const double turn_advance_m = std::sin(0.5) / 0.5;
  both_sides.goals.front().robot_pose = {
      1.0 + 4.0 * turn_advance_m + 3.0, 0.0, 0.0, kTimestamp};
  both_sides.goals.front().reference_progress_m = 7.0;
  both_sides.goals.front().touchdown_target_m = {7.0, 0.0};
  both_sides.primitive_sweep_context.cable_laying_limits
      .maximum_curvature_per_m = 100.0;
  MapSnapshot& map = both_sides.primitive_sweep_context.map;
  for (double x_m = 2.8; x_m <= 3.2; x_m += 0.1) {
    for (double y_m = -0.1; y_m <= 0.1; y_m += 0.1) {
      map.cells[map_index(map, x_m, y_m)].obstacle = true;
    }
  }
  TerrainLayers& terrain = both_sides.primitive_sweep_context.terrain;
  for (double x_m = 1.0; x_m <= 7.5; x_m += 0.1) {
    for (double y_m = 0.1; y_m <= 1.2; y_m += 0.1) {
      terrain.surface.cells[map_index(map, x_m, y_m)]
          .detrended_roughness_rms_m = 0.02;
    }
  }

  const HybridAStarPlanningResult result = planner.plan(both_sides);

  require(result.state == PlanningState::success,
          "two-sided obstacle search did not find a feasible detour");
  double minimum_y_m = 0.0;
  double maximum_y_m = 0.0;
  for (const PathPoint& point : result.robot_path.points) {
    minimum_y_m = std::min(minimum_y_m, point.y_m);
    maximum_y_m = std::max(maximum_y_m, point.y_m);
  }
  require(minimum_y_m < -0.2 && maximum_y_m < 0.2,
          "two-sided search ignored explicit terrain soft cost when choosing "
          "the bypass side");
  require(std::abs(result.terminal_cable_state.lag_angle_rad) <= 0.06 &&
              std::abs(result.terminal_reference_progress.arc_length_m - 7.0) <=
                  0.2 &&
              std::hypot(result.touchdown_path.points.back().x_m - 7.0,
                         result.touchdown_path.points.back().y_m) <= 0.2,
          "two-sided detour missed the complete augmented merge target");
}

void a_near_footprint_width_passage_preserves_the_safe_straight_route() {
  // Design: 18.2.3-4
  CableUncertaintyEnvelopeManager manager;
  const LockedCableUncertaintyEnvelope locked = lock_search_envelope(manager);
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      detour_search_parameters(), wide_corridor_policy(), manager);
  HybridAStarPlanningRequest passage = request(locked);
  passage.goals.front().robot_pose.x_m = 5.5;
  passage.goals.front().reference_progress_m = 4.5;
  passage.goals.front().touchdown_target_m = {4.5, 0.0};
  passage.primitive_sweep_context.cable_laying_limits
      .maximum_curvature_per_m = 100.0;
  MapSnapshot& map = passage.primitive_sweep_context.map;
  for (double x_m = 1.5; x_m <= 5.5; x_m += 0.1) {
    map.cells[map_index(map, x_m, 0.2)].obstacle = true;
    map.cells[map_index(map, x_m, -0.3)].obstacle = true;
  }

  const HybridAStarPlanningResult result = planner.plan(passage);

  require(result.state == PlanningState::success &&
              result.diagnostics.collision_rejection_count > 0U,
          "near-footprint-width passage was not handled conservatively");
  double maximum_absolute_y_m = 0.0;
  for (const PathPoint& point : result.robot_path.points) {
    maximum_absolute_y_m = std::max(maximum_absolute_y_m, std::abs(point.y_m));
  }
  require(maximum_absolute_y_m < 0.06,
          "the selected route left the validated narrow passage");
}

void a_reference_crossing_does_not_merge_distinct_progress_phases() {
  // Design: 18.2.3-14
  // Design: 18.2.4-key-3
  CableUncertaintyEnvelopeManager manager;
  const LockedCableUncertaintyEnvelope locked = lock_search_envelope(manager);
  HybridAStarSearchParameters analytic = search_parameters();
  analytic.analytic_expansion_interval = 1U;
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      analytic, corridor_policy(), manager);
  HybridAStarPlanningRequest crossing = request(locked);
  crossing.reference_line = make_reference_line(
      11, "map", {{0.0, 0.0}, {4.0, 0.0}, {0.0, 0.0}, {0.0, 4.0}});
  MergeGoal correct_phase = crossing.goals.front();
  correct_phase.robot_pose.x_m = 3.5;
  correct_phase.reference_progress_m = 2.5;
  correct_phase.touchdown_target_m = {2.5, 0.0};
  MergeGoal wrong_phase = correct_phase;
  wrong_phase.reference_progress_m = 5.5;
  crossing.goals = {wrong_phase, correct_phase};

  const HybridAStarPlanningResult result = planner.plan(crossing);

  require(result.state == PlanningState::success,
          "reference-crossing search did not reach the local task phase");
  require_near(result.terminal_reference_progress.arc_length_m, 2.5, 1.0e-12,
               "search merged geometrically identical goals from different "
               "reference-progress phases");
  require(result.state_trace.back().base_key.reference_progress_index == 5,
          "reference progress was omitted from the terminal augmented key");
}

void fixed_inputs_reproduce_the_path_and_diagnostics_field_for_field() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      search_parameters(), corridor_policy(), manager);
  const HybridAStarPlanningRequest fixed_request = request(*locked);

  const HybridAStarPlanningResult first = planner.plan(fixed_request);
  const HybridAStarPlanningResult second = planner.plan(fixed_request);
  require(first.state == PlanningState::success &&
              second.state == PlanningState::success,
          "repeated fixed-input searches did not both succeed");
  const ProcessMemorySample memory = sample_process_memory();
  require(memory.peak_resident_bytes > 0U &&
              memory.peak_resident_bytes < 100U * 1024U * 1024U,
          "the target search process exceeds the 100 MB memory budget");
  std::cout << "[metrics] fixed_bytes_per_search_label="
            << first.diagnostics.fixed_bytes_per_search_label
            << " peak_observed_bytes_per_search_label="
            << first.diagnostics.peak_observed_bytes_per_search_label
            << " peak_process_memory_bytes=" << memory.peak_resident_bytes
            << '\n';
  require(first.diagnostics.deterministic_fingerprint != 0U &&
              first.diagnostics.deterministic_fingerprint ==
                  second.diagnostics.deterministic_fingerprint &&
              first.diagnostics.expanded_state_count ==
                  second.diagnostics.expanded_state_count &&
              first.diagnostics.generated_successor_count ==
                  second.diagnostics.generated_successor_count &&
              first.diagnostics.envelope_query_count ==
                  second.diagnostics.envelope_query_count &&
              first.diagnostics.maximum_active_label_budget ==
                  second.diagnostics.maximum_active_label_budget &&
              first.diagnostics.fixed_bytes_per_search_label > 0U &&
              first.diagnostics.fixed_bytes_per_search_label ==
                  second.diagnostics.fixed_bytes_per_search_label &&
              first.diagnostics.peak_observed_bytes_per_search_label >
                  first.diagnostics.fixed_bytes_per_search_label &&
              first.diagnostics.peak_observed_bytes_per_search_label ==
                  second.diagnostics.peak_observed_bytes_per_search_label &&
              first.diagnostics.analytic_expansion_interval ==
                  second.diagnostics.analytic_expansion_interval &&
              first.diagnostics.maximum_planning_duration_s ==
                  second.diagnostics.maximum_planning_duration_s &&
              first.diagnostics.goal_touchdown_position_tolerance_m ==
                  second.diagnostics.goal_touchdown_position_tolerance_m &&
              first.diagnostics.active_label_count ==
                  second.diagnostics.active_label_count &&
              first.diagnostics.peak_active_label_count ==
                  second.diagnostics.peak_active_label_count &&
              first.diagnostics.maximum_labels_per_base_key ==
                  second.diagnostics.maximum_labels_per_base_key &&
              first.diagnostics.labels_per_base_key_p50 ==
                  second.diagnostics.labels_per_base_key_p50 &&
              first.diagnostics.labels_per_base_key_p95 ==
                  second.diagnostics.labels_per_base_key_p95 &&
              first.diagnostics.labels_per_base_key_p99 ==
                  second.diagnostics.labels_per_base_key_p99 &&
              first.diagnostics.equivalent_label_discard_count ==
                  second.diagnostics.equivalent_label_discard_count &&
              first.diagnostics.equivalent_label_replacement_count ==
                  second.diagnostics.equivalent_label_replacement_count &&
              first.diagnostics.signature_fallback_comparison_count ==
                  second.diagnostics.signature_fallback_comparison_count &&
              first.diagnostics.stale_queue_entry_count ==
                  second.diagnostics.stale_queue_entry_count &&
              first.diagnostics.analytic_expansion_attempt_count ==
                  second.diagnostics.analytic_expansion_attempt_count &&
              first.diagnostics.analytic_expansion_accepted_count ==
                  second.diagnostics.analytic_expansion_accepted_count &&
              first.diagnostics.active_label_budget_exhausted ==
                  second.diagnostics.active_label_budget_exhausted &&
              first.diagnostics.deadline_exceeded ==
                  second.diagnostics.deadline_exceeded &&
              first.diagnostics.queue_rule == second.diagnostics.queue_rule,
          "fixed inputs did not reproduce identical search diagnostics");
  require(first.robot_path.points.size() == second.robot_path.points.size() &&
              first.touchdown_path.points.size() ==
                  second.touchdown_path.points.size() &&
              first.state_trace.size() == second.state_trace.size(),
          "fixed inputs changed the reconstructed path shape");
  for (std::size_t index = 0U; index < first.state_trace.size(); ++index) {
    const HybridAStarStateTraceEntry& left = first.state_trace[index];
    const HybridAStarStateTraceEntry& right = second.state_trace[index];
    require(left.base_key.x_index == right.base_key.x_index &&
                left.base_key.y_index == right.base_key.y_index &&
                left.base_key.heading_index == right.base_key.heading_index &&
                left.base_key.cable_lag_index ==
                    right.base_key.cable_lag_index &&
                left.base_key.reference_progress_index ==
                    right.base_key.reference_progress_index &&
                left.robot_pose.x_m == right.robot_pose.x_m &&
                left.robot_pose.y_m == right.robot_pose.y_m &&
                left.robot_pose.heading_rad == right.robot_pose.heading_rad &&
                left.cable_lag_angle_rad == right.cable_lag_angle_rad &&
                left.reference_progress.arc_length_m ==
                    right.reference_progress.arc_length_m,
            "fixed inputs changed an augmented state-trace field");
  }
}

void dependency_version_mismatches_fail_before_search_expansion() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      search_parameters(), corridor_policy(), manager);
  HybridAStarPlanningRequest mismatched = request(*locked);
  mismatched.cable_context.execution_envelope.version = 8;

  const HybridAStarPlanningResult result = planner.plan(mismatched);
  require(result.state == PlanningState::input_invalid &&
              result.diagnostics.expanded_state_count == 0U &&
              result.diagnostics.generated_successor_count == 0U &&
              result.robot_path.points.empty() &&
              !result.diagnostics.entries.empty() &&
              result.diagnostics.entries.front().code ==
                  "HYBRID_ASTAR_DEPENDENCY_MISMATCH",
          "an execution-envelope mismatch was not rejected as an auditable "
          "dependency failure before search");
}

void the_locked_envelope_is_a_hard_search_corridor_gate() {
  // Design: 18.2.3-5
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      search_parameters(), narrow_corridor_policy(), manager);

  const HybridAStarPlanningResult result = planner.plan(request(*locked));
  require(result.state ==
                  PlanningState::no_solution_under_covariance_envelope &&
              result.robot_path.points.empty() &&
              result.touchdown_path.points.empty() &&
              result.diagnostics.corridor_rejection_count > 0U &&
              result.diagnostics.envelope_query_count > 0U &&
              !result.diagnostics.path_dependent_covariance_propagated &&
              !result.diagnostics.entries.empty() &&
              result.diagnostics.entries.back().code ==
                  "NO_SOLUTION_UNDER_COVARIANCE_ENVELOPE",
          "the certified envelope was not enforced as the search corridor "
          "hard gate");
}

void an_initial_goal_cannot_bypass_the_locked_corridor_gate() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      search_parameters(), corridor_policy(), manager);
  HybridAStarPlanningRequest initial_goal = request(*locked);
  initial_goal.start_state.pose.y_m = 0.5;
  initial_goal.goals.front().robot_pose = initial_goal.start_state.pose;
  initial_goal.goals.front().cable_lag_angle_rad = 0.0;
  initial_goal.goals.front().reference_progress_m = 0.0;
  initial_goal.goals.front().touchdown_target_m = {0.0, 0.5};

  const HybridAStarPlanningResult result = planner.plan(initial_goal);
  require(result.state ==
                  PlanningState::no_solution_under_covariance_envelope &&
              result.diagnostics.corridor_rejection_count == 1U &&
              result.diagnostics.worst_constraint.recorded &&
              result.diagnostics.worst_constraint.reason ==
                  "cable_corridor" &&
              result.robot_path.points.empty(),
          "an initial merge goal bypassed the locked statistical corridor "
          "hard gate");
}

void an_initial_footprint_failure_records_its_position() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      search_parameters(), corridor_policy(), manager);
  HybridAStarPlanningRequest outside_area = request(*locked);
  outside_area.start_state.pose.x_m = 9.48;

  const HybridAStarPlanningResult result = planner.plan(outside_area);

  require(result.state == PlanningState::no_solution &&
              result.diagnostics.worst_constraint.recorded &&
              result.diagnostics.worst_constraint.reason ==
                  "robot_operating_area" &&
              std::abs(result.diagnostics.worst_constraint.position_m.x_m -
                       outside_area.start_state.pose.x_m) < 1.0e-12,
          "an initial footprint failure omitted its constraint position");
}

void expansion_budget_exhaustion_is_not_reported_as_no_solution() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  HybridAStarSearchParameters limited = search_parameters();
  limited.maximum_expansions = 1U;
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      limited, corridor_policy(), manager);

  const HybridAStarPlanningResult result = planner.plan(request(*locked));
  require(result.state == PlanningState::timeout &&
              result.diagnostics.expanded_state_count == 1U &&
              !result.diagnostics.entries.empty() &&
              result.diagnostics.entries.back().code ==
                  "HYBRID_ASTAR_EXPANSION_BUDGET_EXHAUSTED",
          "an exhausted expansion budget was falsely reported as no solution");
}

void active_label_budget_exhaustion_is_a_distinct_timeout() {
  // Design: 18.2.3-13
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  HybridAStarSearchParameters limited = search_parameters();
  limited.maximum_active_labels = 1U;
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      limited, corridor_policy(), manager);

  const HybridAStarPlanningResult result = planner.plan(request(*locked));
  require(result.state == PlanningState::timeout &&
              result.diagnostics.maximum_active_label_budget == 1U &&
              result.diagnostics.active_label_count == 1U &&
              result.diagnostics.peak_active_label_count == 1U &&
              result.diagnostics.active_label_budget_exhausted &&
              !result.diagnostics.entries.empty() &&
              result.diagnostics.entries.back().code ==
                  "HYBRID_ASTAR_ACTIVE_LABEL_BUDGET_EXHAUSTED",
          "an exhausted active-label budget did not fail with its distinct "
          "auditable timeout reason");
}

void planning_deadline_exhaustion_is_a_distinct_timeout() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  HybridAStarSearchParameters limited = search_parameters();
  limited.maximum_planning_duration_s = 0.5;
  std::size_t clock_call_count{};
  const HybridAStarSteadyClock clock = [&clock_call_count]() {
    const auto elapsed = clock_call_count++ == 0U
                             ? std::chrono::milliseconds{0}
                             : std::chrono::milliseconds{1000};
    return std::chrono::steady_clock::time_point{elapsed};
  };
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      limited, corridor_policy(), manager, clock);

  const HybridAStarPlanningResult result = planner.plan(request(*locked));
  require(result.state == PlanningState::timeout &&
              result.diagnostics.deadline_exceeded &&
              result.diagnostics.expanded_state_count == 0U &&
              !result.diagnostics.active_label_budget_exhausted &&
              !result.diagnostics.entries.empty() &&
              result.diagnostics.entries.back().code ==
                  "HYBRID_ASTAR_DEADLINE_EXCEEDED",
          "an exhausted planning deadline did not use its distinct timeout "
          "diagnostic");
}

void initial_mechanical_history_is_canonicalized_before_labeling() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      search_parameters(), corridor_policy(), manager);
  HybridAStarPlanningRequest initial_goal = request(*locked);
  CableConstraintMemory& memory =
      initial_goal.initial_cable_state.laying_memory;
  memory.previous_distinct_touchdown_points_m = {{-0.25, 0.0}, {0.0, 0.0}};
  for (std::size_t index = 0U; index <= 8U; ++index) {
    const double arc_length_m = 0.25 * static_cast<double>(index);
    memory.trailing_support_samples.push_back(
        {arc_length_m, {-2.0 + arc_length_m, 0.0}});
  }
  memory.retained_arc_length_m = 2.0;
  memory.canonical_signature = 77U;
  initial_goal.goals.front().robot_pose = initial_goal.start_state.pose;
  initial_goal.goals.front().cable_lag_angle_rad = 0.0;
  initial_goal.goals.front().reference_progress_m = 0.0;
  initial_goal.goals.front().touchdown_target_m = {0.0, 0.0};

  const HybridAStarPlanningResult result = planner.plan(initial_goal);
  const CableConstraintMemory& canonical =
      result.terminal_cable_state.laying_memory;
  require(result.state == PlanningState::success &&
              canonical.trailing_support_samples.size() == 3U &&
              canonical.retained_arc_length_m == 0.5 &&
              canonical.canonical_signature != 0U &&
              canonical.canonical_signature != 77U,
          "the initial search label retained an unbounded or stale mechanical "
          "history");
}

void incomplete_actual_history_cannot_bypass_labeling_at_an_initial_goal() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      search_parameters(), corridor_policy(), manager);
  HybridAStarPlanningRequest initial_goal = request(*locked);
  initial_goal.primitive_sweep_context.cable_history_boundary =
      CableHistoryBoundary::actual_laying_history;
  initial_goal.goals.front().robot_pose = initial_goal.start_state.pose;
  initial_goal.goals.front().cable_lag_angle_rad = 0.0;
  initial_goal.goals.front().reference_progress_m = 0.0;

  const HybridAStarPlanningResult result = planner.plan(initial_goal);
  require(result.state == PlanningState::input_invalid &&
              !result.diagnostics.entries.empty() &&
              result.diagnostics.entries.back().code ==
                  "HYBRID_ASTAR_INITIAL_MECHANICAL_HISTORY_INVALID",
          "an incomplete actual laying history bypassed the initial label "
          "gate when the start pose already matched a goal");
}

void mechanically_distinct_histories_share_a_base_key_without_merging() {
  // Design: 18.2.3-11
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  HybridAStarSearchParameters one_expansion = search_parameters();
  one_expansion.maximum_expansions = 1U;
  one_expansion.motion_primitives = {{1U, 1.0, 0.0},
                                     {2U, 1.0, 0.02}};
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      one_expansion, corridor_policy(), manager);

  const HybridAStarPlanningResult result = planner.plan(request(*locked));
  require(result.state == PlanningState::timeout &&
              result.diagnostics.generated_successor_count == 2U &&
              result.diagnostics.active_label_count == 3U &&
              result.diagnostics.peak_active_label_count == 3U &&
              result.diagnostics.maximum_labels_per_base_key == 2U &&
              result.diagnostics.labels_per_base_key_p50 == 1U &&
              result.diagnostics.labels_per_base_key_p95 == 2U &&
              result.diagnostics.labels_per_base_key_p99 == 2U,
          "mechanically distinct histories at one base key were merged by "
          "current path cost");
}

void a_higher_cost_mechanical_history_can_reach_the_only_matching_goal() {
  // Design: 18.2.4-25
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  HybridAStarSearchParameters branching = search_parameters();
  branching.xy_resolution_m = 1.0;
  branching.heading_resolution_rad = 1.0;
  branching.cable_lag_resolution_rad = 1.0;
  branching.reference_progress_resolution_m = 1.0;
  branching.goal_lag_tolerance_rad = 3.0;
  branching.goal_progress_tolerance_m = 8.0;
  branching.goal_touchdown_position_tolerance_m = 10.0;
  branching.motion_primitives = {{1U, 1.0, 0.0}, {2U, 1.0, 0.3}};
  CableCorridorRiskPolicy wide_corridor = corridor_policy();
  wide_corridor.nominal_half_width_m = 2.0;
  wide_corridor.absolute_half_width_m = 3.0;
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      branching, wide_corridor, manager);
  HybridAStarPlanningRequest only_curved_history_reaches = request(*locked);
  const double turn_angle_rad = 0.3;
  only_curved_history_reaches.goals.front().robot_pose = {
      1.0 + std::sin(turn_angle_rad) / 0.3 +
          std::cos(turn_angle_rad),
      (1.0 - std::cos(turn_angle_rad)) / 0.3 +
          std::sin(turn_angle_rad),
      turn_angle_rad,
      kTimestamp};

  const HybridAStarPlanningResult result =
      planner.plan(only_curved_history_reaches);
  require(result.state == PlanningState::success &&
              result.state_trace.size() == 3U &&
              result.state_trace[1U].robot_pose.y_m > 0.0 &&
              result.diagnostics.solution_cost > 2.0 &&
              result.diagnostics.maximum_labels_per_base_key >= 2U,
          "the higher-cost mechanical history was discarded before reaching "
          "its uniquely matching successor goal: state=" +
              std::to_string(static_cast<int>(result.state)) +
              " expanded=" +
              std::to_string(result.diagnostics.expanded_state_count) +
              " generated=" +
              std::to_string(result.diagnostics.generated_successor_count) +
              " active_labels=" +
              std::to_string(result.diagnostics.active_label_count) +
              " max_per_key=" +
              std::to_string(result.diagnostics.maximum_labels_per_base_key) +
              " equivalent_discards=" +
              std::to_string(
                  result.diagnostics.equivalent_label_discard_count) +
              " collision_rejections=" +
              std::to_string(result.diagnostics.collision_rejection_count) +
              " terrain_rejections=" +
              std::to_string(
                  result.diagnostics.traversability_rejection_count) +
              " corridor_rejections=" +
              std::to_string(result.diagnostics.corridor_rejection_count) +
              " laying_rejections=" +
              std::to_string(
                  result.diagnostics.cable_laying_rejection_count));
  require_near(result.state_trace[2U].robot_pose.heading_rad, turn_angle_rad,
               1.0e-12,
               "the retained curved history reached the goal with the wrong "
               "terminal heading");
}

void future_equivalent_histories_are_dominated_within_one_label() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  HybridAStarSearchParameters duplicate_successors = search_parameters();
  duplicate_successors.motion_primitives = {{1U, 1.0, 0.0},
                                            {2U, 1.0, 0.0}};
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      duplicate_successors, corridor_policy(), manager);

  const HybridAStarPlanningResult result = planner.plan(request(*locked));
  require(result.state == PlanningState::success &&
              result.diagnostics.active_label_count == 5U &&
              result.diagnostics.maximum_labels_per_base_key == 1U &&
              result.diagnostics.equivalent_label_discard_count == 4U &&
              result.diagnostics.equivalent_label_replacement_count == 0U,
          "future-equivalent duplicate successors occupied separate labels");
}

void a_lower_cost_equivalent_history_reopens_and_stales_the_old_queue_entry() {
  // Design: 18.2.3-10
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  HybridAStarSearchParameters reopening = search_parameters();
  const double expensive_length_m = 1.0e-5;
  const double cheaper_length_m = std::nextafter(expensive_length_m, 0.0);
  reopening.motion_primitives = {{1U, expensive_length_m, 0.0},
                                 {2U, cheaper_length_m, 0.0}};
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      reopening, corridor_policy(), manager);
  HybridAStarPlanningRequest bounded = request(*locked);
  bounded.primitive_sweep_context.track_footprint = tiny_track_footprint();
  bounded.primitive_sweep_context.robot_operating_area.polygon =
      {{-3.5, -3.5}, {1.030015, -3.5}, {1.030015, 3.5}, {-3.5, 3.5}};

  const HybridAStarPlanningResult result = planner.plan(bounded);
  require(result.state == PlanningState::no_solution &&
              result.diagnostics.equivalent_label_replacement_count == 1U &&
              result.diagnostics.stale_queue_entry_count == 1U &&
              result.diagnostics.active_label_count == 2U &&
              result.diagnostics.maximum_labels_per_base_key == 2U &&
              result.diagnostics.operating_area_rejection_count == 2U,
          "a lower-cost equivalent history did not reopen its stable label or "
          "retire the old queue entry");
}

void an_initial_cable_state_outside_the_model_domain_is_not_a_corridor_failure() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(91, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(
      manager.setCurrentContext(envelope_key(), 1, kPlanningTime));
  const auto locked = manager.getValidated(envelope_key(), kPlanningTime);
  require(locked.has_value(), "test envelope did not lock");
  const HybridAStarPlanner planner(
      model_parameters(),
      make_reference_progress_association_parameters(parameter_config()),
      search_parameters(), corridor_policy(), manager);
  HybridAStarPlanningRequest outside_model = request(*locked);
  outside_model.initial_cable_state.lag_angle_rad = 0.6;

  const HybridAStarPlanningResult result = planner.plan(outside_model);
  require(result.state == PlanningState::input_invalid &&
              result.diagnostics.corridor_rejection_count == 0U &&
              !result.diagnostics.entries.empty() &&
              result.diagnostics.entries.back().code ==
                  "HYBRID_ASTAR_INITIAL_CABLE_STATE_OUT_OF_RANGE",
          "an out-of-domain initial cable state was misreported as a corridor "
          "failure");
}

void equal_nominal_and_absolute_corridor_widths_are_rejected() {
  CableUncertaintyEnvelopeManager manager;
  CableCorridorRiskPolicy invalid_policy = corridor_policy();
  invalid_policy.absolute_half_width_m = invalid_policy.nominal_half_width_m;

  bool rejected = false;
  try {
    static_cast<void>(HybridAStarPlanner(
        model_parameters(),
        make_reference_progress_association_parameters(parameter_config()),
        search_parameters(), invalid_policy, manager));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected,
          "equal nominal and absolute corridor widths were accepted by the "
          "planner but rejected by corridor evaluation");
}

void negative_equivalent_label_cost_tolerance_is_rejected() {
  CableUncertaintyEnvelopeManager manager;
  HybridAStarSearchParameters invalid = search_parameters();
  invalid.equivalent_label_cost_tolerance_m = -1.0e-12;

  bool rejected = false;
  try {
    static_cast<void>(HybridAStarPlanner(
        model_parameters(),
        make_reference_progress_association_parameters(parameter_config()),
        invalid, corridor_policy(), manager));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected,
          "a negative equivalent-label cost tolerance was accepted");
}

}  // namespace

int main() {
  constexpr std::uint64_t kSeed = 2121;
  try {
    flat_straight_search_returns_a_reference_aligned_touchdown_path();
    robot_and_cable_spatial_domains_are_evaluated_independently();
    returned_solution_satisfies_all_segment_hard_invariants();
    validated_analytic_expansion_reaches_an_off_lattice_merge_goal();
    validated_analytic_expansion_supports_an_exact_curved_connection();
    validated_analytic_expansion_follows_a_multi_segment_dubins_connection();
    a_robot_goal_match_cannot_substitute_for_the_touchdown_goal();
    analytic_expansion_is_rejected_by_a_mid_connection_obstacle();
    mixed_obstacle_and_envelope_rejections_are_ordinary_no_solution();
    a_single_open_side_is_used_before_returning_to_a_merge_goal();
    two_open_sides_choose_the_lower_soft_cost_detour();
    a_near_footprint_width_passage_preserves_the_safe_straight_route();
    a_reference_crossing_does_not_merge_distinct_progress_phases();
    an_obstacle_between_clear_primitive_endpoints_is_rejected();
    a_forbidden_cell_between_clear_touchdown_endpoints_is_rejected();
    a_directional_slope_between_clear_primitive_endpoints_is_rejected();
    curved_primitive_samples_follow_the_exact_constant_curvature_arc();
    a_turning_footprint_corner_sweep_rejects_an_obstacle();
    primitive_length_and_touchdown_sampling_density_preserve_cost_and_gates();
    feasible_solution_cost_is_explicit_and_touchdown_only();
    dubins_heuristic_is_an_admissible_robot_goal_lower_bound();
    heuristic_is_zero_inside_the_robot_goal_tolerances();
    touchdown_corridor_cost_uses_robot_primitive_arc_length();
    rejected_successors_have_complete_reason_counts();
    a_corridor_excursion_between_legal_touchdown_endpoints_is_rejected();
    mismatched_map_and_terrain_grid_geometry_fails_before_expansion();
    a_narrow_operating_area_notch_between_sweep_samples_is_rejected();
    climbable_terrain_remains_feasible_and_is_soft_ranked();
    nonfinite_robot_terrain_cost_input_fails_before_search();
    fixed_inputs_reproduce_the_path_and_diagnostics_field_for_field();
    dependency_version_mismatches_fail_before_search_expansion();
    the_locked_envelope_is_a_hard_search_corridor_gate();
    an_initial_goal_cannot_bypass_the_locked_corridor_gate();
    an_initial_footprint_failure_records_its_position();
    expansion_budget_exhaustion_is_not_reported_as_no_solution();
    active_label_budget_exhaustion_is_a_distinct_timeout();
    planning_deadline_exhaustion_is_a_distinct_timeout();
    initial_mechanical_history_is_canonicalized_before_labeling();
    incomplete_actual_history_cannot_bypass_labeling_at_an_initial_goal();
    mechanically_distinct_histories_share_a_base_key_without_merging();
    a_higher_cost_mechanical_history_can_reach_the_only_matching_goal();
    future_equivalent_histories_are_dominated_within_one_label();
    a_lower_cost_equivalent_history_reopens_and_stales_the_old_queue_entry();
    an_initial_cable_state_outside_the_model_domain_is_not_a_corridor_failure();
    equal_nominal_and_absolute_corridor_widths_are_rejected();
    negative_equivalent_label_cost_tolerance_is_rejected();
    std::cout << "hybrid A* planner checks passed: 46"
              << " seed=" << kSeed
              << " input_version=t25-multi-goal-analytic/v1"
              << " units=SI timestamp=monotonic-ns"
              << " domain=flat-straight-domain-v1"
              << " risk=pointwise-envelope-only\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "hybrid A* planner failure: " << error.what()
              << " seed=" << kSeed
              << " input_version=t25-multi-goal-analytic/v1"
              << " units=SI timestamp=monotonic-ns"
              << " domain=flat-straight-domain-v1"
              << " risk=pointwise-envelope-only\n";
    return 1;
  }
}
