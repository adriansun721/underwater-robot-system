#include "underwater_planner/core/plan_validity_evaluator.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using namespace underwater_planner::core;

constexpr MonotonicTime kDataTime{1'000'000'000};
constexpr MonotonicTime kNow{2'000'000'000};
constexpr const char* kDomain = "competition-seabed-v1";
constexpr const char* kParameterProfile = "reference-progress/v1";

void require(const bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

CableModelParameters model_parameters() {
  CableModelParameters p;
  p.version = 14;
  p.calibration_dataset_id = "cable-mean-cal-v1";
  p.operating_domain_id = kDomain;
  p.release_point_offset_m = {0.5, 0.25};
  p.touchdown_distance_m = 1.0;
  p.direction_response_length_m = 2.0;
  p.maximum_lag_angle_rad = 1.2;
  p.maximum_payout_tracking_error_mps = 0.1;
  p.payout_speed_range = {0.0, 1.0};
  p.maximum_payout_acceleration_mps2 = 0.4;
  p.maximum_tension_tracking_error_n = 10.0;
  p.tension_range = {10.0, 100.0};
  p.search_integration_step_m = 0.5;
  p.validation_integration_step_m = 0.02;
  p.touchdown_distance_variance_m2 = 0.0025;
  p.direction_response_length_variance_m2 = 0.04;
  p.lag_angle_process_variance_per_m_rad2 = 0.03;
  p.touchdown_process_noise_per_m_m2 = {0.001, 0.0, 0.0, 0.002};
  p.approved_sensor_modes = {SensorHealthMode::nominal};
  return p;
}

ExecutionOperatingEnvelope execution_envelope() {
  ExecutionOperatingEnvelope e;
  e.version = 7;
  e.operating_domain_id = kDomain;
  e.limits.ground_speed = {0.0, 0.8};
  e.limits.ground_acceleration = {-0.4, 0.4};
  e.limits.maximum_lateral_acceleration_mps2 = 0.4;
  e.limits.payout_speed = {0.0, 0.9};
  e.limits.payout_acceleration = {-0.3, 0.3};
  e.limits.maximum_payout_tracking_error_mps = 0.08;
  e.limits.tension = {20.0, 80.0};
  e.limits.maximum_stopping_distance_m = 1.5;
  e.maximum_payout_acceleration_tracking_error_mps2 = 0.1;
  e.maximum_tension_tracking_error_n = 8.0;
  return e;
}

TimedPath timed_path() {
  TimedPath path;
  path.geometry.metadata = {31, "map", 4, "constant-curvature"};
  path.geometry.points = {{0.0, 0.0, 0.0, 0.0, 0.0},
                          {1.0, 1.0, 0.0, 0.0, 0.0},
                          {2.0, 2.0, 0.0, 0.0, 0.0},
                          {3.0, 3.0, 0.0, 0.0, 0.0}};
  path.execution_profile.version = 41;
  path.execution_profile.operating_envelope_version = 7;
  path.execution_profile.interpolation_rule = "linear-in-arc-length";
  path.execution_profile.stopping_point_arc_length_m = 3.0;
  path.execution_profile.approved_tracking_limits = execution_envelope().limits;
  path.execution_profile.samples = {
      {0.0, {0}, 0.5, 0.0, 0.5, 0.0, 40.0},
      {1.0, {2'000'000'000}, 0.5, 0.0, 0.5, 0.0, 40.0},
      {2.0, {4'000'000'000}, 0.5, 0.0, 0.5, 0.0, 40.0},
      {3.0, {6'000'000'000}, 0.0, -0.25, 0.0, -0.25, 40.0}};
  return path;
}

ReferenceLine reference_line() {
  return make_reference_line(4, "map", {{0.0, 0.0}, {1.0, 0.0},
                                         {2.0, 0.0}, {3.0, 0.0}});
}

MapSnapshot map() {
  MapSnapshot value;
  value.version = {"map", 1, kDataTime, "map"};
  value.width = 40;
  value.height = 40;
  value.resolution_m = 0.5;
  value.origin_x_m = -5.0;
  value.origin_y_m = -5.0;
  value.derived_configuration_version = 12;
  value.cells.assign(value.width * value.height,
                     MapCell{0.0, 0.01, 1.0, true, kDataTime});
  return value;
}

TerrainLayers terrain() {
  TerrainLayers t;
  t.source_map_version = map().version;
  t.analysis_config_version = 12;
  t.operating_domain_id = kDomain;
  t.surface_fit_window_size_m = 1.0;
  t.surface.width = 40;
  t.surface.height = 40;
  t.surface.resolution_m = 0.5;
  t.surface.origin_x_m = -5.0;
  t.surface.origin_y_m = -5.0;
  t.surface.cells.assign(t.surface.width * t.surface.height, SurfaceEstimate{});
  t.cable_laying.cells.assign(t.surface.cells.size(), CableLayingTerrainCell{});
  for (SurfaceEstimate& cell : t.surface.cells) {
    cell.status = TerrainEstimateStatus::valid;
    cell.support_ratio = 1.0;
  }
  for (CableLayingTerrainCell& cell : t.cable_laying.cells) {
    cell.known = true;
    cell.confidence = 1.0;
  }
  return t;
}

CableLayingLimits laying_limits() {
  return {16, kDomain, 0.2, 0.8, 0.5, 0.2, 0.5, 1.0, 0.5,
          0.01, 1.0, 1.0, 1.0};
}

CableCorridorRiskPolicy corridor_policy() {
  return {17, "corridor-cal-v1", kDomain, 1.0e-3,
          2.0, 3.0, 10.0, 0.01, true};
}

ReferenceProgressAssociationParameters association_parameters() {
  ParameterConfig config;
  config.profile_id = kParameterProfile;
  config.operating_domain_id = kDomain;
  config.search.reference_progress_backward_tolerance_m = 0.2;
  config.search.reference_progress_maximum_ratio = 1.2;
  config.search.reference_progress_forward_slack_m = 0.1;
  config.search.reference_progress_distance_scale_m = 1.0;
  config.search.reference_progress_heading_scale_rad = 0.5;
  config.search.reference_progress_heading_weight = 1.0;
  config.search.reference_progress_association_score_tolerance = 1.0e-10;
  return make_reference_progress_association_parameters(config);
}

CableContext cable_context() {
  CableContext c;
  c.current_telemetry = {0.5, 0.0, 40.0, kDataTime, 11};
  c.execution_envelope = execution_envelope();
  c.mode = PredictionMode::validation;
  c.sensor_mode = SensorHealthMode::nominal;
  c.uncertainty_envelope_version = 9;
  c.uncertainty_envelope_generator_version = 10;
  c.robot_uncertainty_profile_version = 11;
  for (const double s : {0.0, 1.0, 2.0, 3.0}) {
    c.robot_uncertainty_profile.push_back(
        {s, {{0.04, 0.0, 0.0, 0.09}, 0.0, 0.0, 0.01}, 0.002});
  }
  return c;
}

CableUncertaintyEnvelope envelope() {
  CableUncertaintyEnvelope e;
  e.validity = EnvelopeBuildValidity::valid;
  e.dependencies = {10, 14, 7, 4, 2, 3, 4, 5, 6, 7,
                    SensorHealthMode::nominal, kDomain,
                    "cable-mean-cal-v1", "cert-v1", "sensor-v1",
                    "exec-v1", "margin-v1"};
  e.margin_budget = {7, "margin-v1", 0.0, 0.0, 0.0, 0.0};
  e.segments = {{0.0, 3.0, 100.0, 10.0}};
  e.generation_timestamp = kDataTime;
  e.risk_semantics = kPointwiseEnvelopeRiskSemantics;
  return e;
}

LockedCableUncertaintyEnvelope lock_envelope(
    CableUncertaintyEnvelopeManager& manager) {
  const CableUncertaintyEnvelope value = envelope();
  EnvelopeCoverageCertification certification;
  certification.version = 21;
  certification.calibration_dataset_id = "coverage-v1";
  certification.passed = true;
  certification.audited_at = kDataTime;
  certification.valid_until = {9'000'000'000};
  certification.certified_envelope_version = 9;
  certification.certified_dependencies = value.dependencies;
  require(manager.registerValidated(9, value, certification).accepted(),
          "envelope registration failed");
  require(manager.setCurrentContext({4, SensorHealthMode::nominal, kDomain, 14, 7},
                                    1, kDataTime)
              .context_update_status == EnvelopeContextUpdateStatus::accepted,
          "envelope context setup failed");
  const auto locked = manager.getValidated(
      {4, SensorHealthMode::nominal, kDomain, 14, 7}, kNow);
  require(locked.has_value(), "envelope lock failed");
  return *locked;
}

CableState current_cable_state() {
  CableState state;
  state.kind = CableStateKind::tracked;
  state.lag_angle_rad = 0.0;
  state.lag_angle_variance_rad2 = 0.01;
  state.timestamp = kDataTime;
  state.sequence_number = 21;
  state.laying_memory.previous_distinct_touchdown_points_m =
      {{-1.5, 0.25}, {-0.5, 0.25}};
  state.laying_memory.trailing_support_samples =
      {{0.0, {-1.5, 0.25}}, {1.0, {-0.5, 0.25}}};
  state.laying_memory.retained_arc_length_m = 1.0;
  return state;
}

PathCandidateVerificationContext path_context() {
  PathCandidateVerificationContext c;
  c.map = map();
  c.terrain = terrain();
  c.robot_operating_area =
      {2, "area", {{-4.0, -4.0}, {8.0, -4.0}, {8.0, 4.0}, {-4.0, 4.0}}};
  c.collision_risk_policy = {18, "collision-cal-v1", kDomain, 0.01, 0.5, 0.0};
  c.robot_relative_obstacle_covariance_m2 = {0.0, 0.0, 0.0, 0.0};
  c.robot_capability = {1.0, 1.0, 1.0, 1.0, 0.3, 0.3, 0.5, 0.5, 0.2, 0.1, 1.0};
  c.track_footprint = {{{-0.2, -0.2}, {0.2, -0.2}, {0.2, 0.2}, {-0.2, 0.2}},
                       {{-0.15, 0.05}, {0.15, 0.05}, {0.15, 0.15}, {-0.15, 0.15}},
                       {{-0.15, -0.15}, {0.15, -0.15}, {0.15, -0.05}, {-0.15, -0.05}}};
  c.terrain_gradient_risk_policy =
      {13, 12, 0.01, 2.0, GradientCoverageModel::empirical_bounded,
       "terrain-cal-v1", kDomain, true};
  c.maximum_sweep_spacing_fraction = 0.5;
  c.geometric_curvature_tolerance_per_m = 1.0e-9;
  c.heading_tolerance_rad = 1.0e-9;
  c.curvature_rate_tolerance_per_m2 = 1.0e-9;
  return c;
}

SmoothingLimits smoothing_limits() {
  SmoothingLimits limits;
  limits.version = 1;
  limits.output_path_version = 31;
  limits.minimum_segment_length_m = 0.01;
  limits.maximum_curvature_per_m = 0.5;
  limits.maximum_curvature_rate_per_m2 = 0.5;
  limits.maximum_boundary_time_skew = {0};
  limits.allowed_residuals.start_position_residual_m = 1.0e-8;
  limits.allowed_residuals.start_heading_residual_rad = 1.0e-8;
  limits.allowed_residuals.start_curvature_residual_per_m = 1.0e-8;
  limits.allowed_residuals.goal_position_residual_m = 1.0e-8;
  limits.allowed_residuals.goal_heading_residual_rad = 1.0e-8;
  limits.allowed_residuals.goal_curvature_residual_per_m = 1.0e-8;
  return limits;
}

PlanningResult plan(const LockedCableUncertaintyEnvelope& locked,
                    CableUncertaintyEnvelopeManager& manager) {
  CableState initial = current_cable_state();
  initial.laying_memory = {};
  TimedCableCandidateInput input;
  input.initial_cable_state = initial;
  input.robot_path = timed_path();
  input.cable_context = cable_context();
  input.reference_line = reference_line();
  input.corridor_policy = corridor_policy();
  const CablePrediction predicted =
      CableModel(model_parameters()).predict(initial, input.robot_path, input.cable_context);
  input.reference_progress_m = predicted.robot_arc_length_profile_m;
  input.interval_bound_certificate =
      {9, std::vector<double>(input.reference_progress_m.size() - 1U, 0.0)};
  input.terrain = terrain();
  input.laying_limits = laying_limits();
  input.history_boundary = CableHistoryBoundary::explicit_task_start;
  input.reference_is_deterministic = true;
  input.covariance_includes_coordinate_transform_error = true;
  input.evaluation_timestamp = kNow;
  input.locked_envelope = locked;
  const auto checked =
      TimedCableCandidateVerifier(CableModel(model_parameters()), &manager).validate(input);
  require(checked.valid && checked.cable_prediction.has_value() &&
              checked.terminal_cable_state.has_value(),
          "planning fixture cable validation failed");

  PlanningResult result;
  result.sequence_number = 44;
  result.timestamp = kNow;
  result.validity_duration = {5'000'000'000};
  result.state = PlanningState::path_valid;
  result.robot_trajectory = timed_path();
  result.cable_path = checked.cable_prediction->touchdown_path;
  result.terminal_cable_state = *checked.terminal_cable_state;
  result.cable_model_validity = CableModelValidity::valid;
  result.corridor_result = checked.corridor_result;
  result.cable_laying_result = checked.laying_result;
  result.error_budget.robot_position_covariance_m2 = {0.04, 0.0, 0.0, 0.09};
  result.error_budget.touchdown_position_covariance_m2 =
      *checked.cable_prediction->touchdown_covariance_profile_m2;
  result.error_budget.epsilon_robot = 0.01;
  result.error_budget.epsilon_terrain_gradient_local = 0.01;
  result.error_budget.epsilon_point = 1.0e-3;
  result.error_budget.calibration_dataset_id = "corridor-cal-v1";
  result.error_budget.terrain_gradient_calibration_dataset_id = "terrain-cal-v1";
  result.error_budget.terrain_gradient_policy_version = 13;
  result.error_budget.corridor_risk_policy_version = 17;
  result.error_budget.cable_model_version = 14;
  result.error_budget.uncertainty_envelope_version = 9;
  result.error_budget.uncertainty_envelope_generator_version = 10;
  result.error_budget.execution_operating_envelope_version = 7;
  result.error_budget.operating_domain_id = kDomain;
  result.error_budget.covariance_envelope_audit_passed = true;
  result.map_version = map().version;
  result.reference_line_version = 4;
  result.robot_operating_area_version = 2;
  result.terrain_gradient_policy_version = 13;
  result.corridor_risk_policy_version = 17;
  result.cable_model_version = 14;
  result.uncertainty_envelope_version = 9;
  result.uncertainty_envelope_generator_version = 10;
  result.execution_operating_envelope_version = 7;
  result.execution_profile_version = 41;
  result.operating_domain_id = kDomain;
  result.cable_corridor_version = 3;
  result.diagnostics.schema_version = "diagnostics/v1";
  result.diagnostics.input_version = "fixture/v1";
  result.diagnostics.unit_system = "SI";
  result.diagnostics.operating_domain_id = kDomain;
  result.diagnostics.risk_semantics =
      "POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE";
  result.diagnostics.dependencies = {result.map_version, 4, 2, 13, 17, 14,
                                     9, 10, 7, 41,
                                     SensorHealthMode::nominal, kDomain, 3};
  const ValidationResult validation = validate(result);
  if (!validation.valid) {
    throw std::runtime_error("planning fixture is invalid: " +
                             validation.issues.front());
  }
  require(manager.registerDependentPlan(result.sequence_number, locked, kNow),
          "planning fixture could not be bound to the envelope");
  return result;
}

ImmutablePlanningResult publish(const PlanningResult& candidate) {
  PlanningResultPublisher publisher;
  const PlanningResultPublication publication = publisher.publish(candidate);
  require(publication.published(), "planning fixture publication failed");
  return *publication.result;
}

SynchronizedValidationInputs inputs() {
  SynchronizedValidationInputs value;
  value.captured_at = kNow;
  value.source_revision = 50;
  value.robot_state = {{1.0, 0.0, 0.0, kDataTime}, 0.5, 0.0, kDataTime, 31};
  value.cable_state = current_cable_state();
  value.reference_progress = {4, 0.5, kDataTime, 33};
  value.cable_telemetry = {0.5, 0.0, 40.0, kDataTime, 34};
  value.execution_tracking_state = {41, 7, 1.0, kDataTime, 35};
  value.planning_snapshot.map = map();
  value.planning_snapshot.reference_line = reference_line();
  value.planning_snapshot.robot_operating_area = path_context().robot_operating_area;
  value.planning_snapshot.cable_corridor =
      {3, "corridor", {{-4.0, -3.0}, {8.0, -3.0}, {8.0, 3.0}, {-4.0, 3.0}}};
  value.dependencies = {map().version, 4, 2, 13, 17, 14, 9, 10, 7, 41,
                        SensorHealthMode::nominal, kDomain, 3};
  value.tracker_update_receipt = {36, 37, std::nullopt, 34, 21, 33};
  return value;
}

PlanValidityContext validation_context(
    const LockedCableUncertaintyEnvelope& locked,
    CableUncertaintyEnvelopeManager& manager) {
  PlanValidityContext context;
  context.terrain = terrain();
  context.cable_context = cable_context();
  context.corridor_policy = corridor_policy();
  context.corridor_interval_bound = {9, 0.0};
  context.reference_progress_parameters = association_parameters();
  context.laying_limits = laying_limits();
  context.locked_envelope = locked;
  context.envelope_manager = &manager;
  context.path_context = path_context();
  context.smoothing_limits = smoothing_limits();
  context.goal_boundary = {3.0, 0.0, 0.0, 0.0,
                           PathBoundarySource::planned_goal, kDataTime,
                           kDataTime, 0};
  context.reference_is_deterministic = true;
  context.covariance_includes_coordinate_transform_error = true;
  return context;
}

PlanValidityEvaluatorConfig config() {
  PlanValidityEvaluatorConfig value;
  value.version = 31;
  value.parameter_profile_id = kParameterProfile;
  value.operating_domain_id = kDomain;
  value.maximum_reuse_duration = {3'000'000'000};
  value.input_limits =
      {{5'000'000'000}, {5'000'000'000}, {5'000'000'000},
       {5'000'000'000}, {5'000'000'000}, {5'000'000'000}, {100'000'000}};
  value.envelope_validity_margin = {0};
  value.position_tolerance_m = 0.01;
  value.heading_tolerance_rad = 0.01;
  value.curvature_tolerance_per_m = 0.01;
  value.maximum_ground_speed_tracking_error_mps = 0.05;
  value.maximum_ground_acceleration_tracking_error_mps2 = 0.05;
  value.stopping_safety_margin_m = 0.1;
  return value;
}

void successful_recheck_crops_and_issues_a_new_lease() {
  // Design: 18.2.7-14
  CableUncertaintyEnvelopeManager manager;
  const auto locked = lock_envelope(manager);
  const ImmutablePlanningResult immutable_plan = publish(plan(locked, manager));
  PlanValidityEvaluator evaluator(CableModel(model_parameters()), config());
  const auto result = evaluator.validateRemainingPlan(
      immutable_plan, inputs(), validation_context(locked, manager), kNow);
  if (!(result.valid && result.action == PlanValidationAction::reuse &&
        result.status == PlanValidationStatus::valid &&
        result.remaining_path && result.lease.has_value())) {
    throw std::runtime_error(
        "valid remaining plan did not receive a reuse lease: status=" +
        std::to_string(static_cast<int>(result.status)) + " issue=" +
        (result.issues.empty() ? std::string{"none"} : result.issues.front()));
  }
  require(result.remaining_path->geometry.points.front().arc_length_m == 1.0 &&
              result.remaining_path->execution_profile.samples.front()
                      .time_from_start.nanoseconds == 0 &&
              result.remaining_path->execution_profile.version == 41 &&
              result.lease->lease_sequence == 1 &&
              result.lease->plan_sequence_number == 44 &&
              result.lease->evaluator_config_version == 31 &&
              result.lease->parameter_profile_id == kParameterProfile &&
              result.lease->execution_profile_version == 41 &&
              result.lease->expires_at.nanoseconds == 5'000'000'000 &&
              result.lease->robot_path_validation_passed &&
              result.lease->cable_corridor_validation_passed &&
              result.lease->cable_laying_validation_passed &&
              result.diagnostics.dependencies.cable_model_version == 14 &&
              result.diagnostics.operating_domain_id == kDomain &&
              result.evaluator_config_version == 31 &&
              result.parameter_profile_id == kParameterProfile &&
              result.diagnostics.input_version.find(kParameterProfile) !=
                  std::string::npos &&
              !result.diagnostics.risk_semantics.empty(),
          "remaining path or lease omitted required immutable bindings");
  require(immutable_plan->robot_trajectory.execution_profile.samples.front()
              .time_from_start.nanoseconds == 0 &&
              immutable_plan->robot_trajectory.geometry.points.front().arc_length_m == 0.0,
          "remaining-plan validation mutated the published plan");
}

void publication_candidate_may_advance_the_execution_profile_version() {
  // Design: 18.2.7-21
  CableUncertaintyEnvelopeManager manager;
  const auto locked = lock_envelope(manager);
  PlanningResult candidate = plan(locked, manager);
  candidate.sequence_number = 45;
  candidate.robot_trajectory.geometry.points.erase(
      candidate.robot_trajectory.geometry.points.begin());
  candidate.robot_trajectory.execution_profile.samples.erase(
      candidate.robot_trajectory.execution_profile.samples.begin());
  for (PathPoint& point : candidate.robot_trajectory.geometry.points) {
    point.arc_length_m -= 1.0;
  }
  for (ExecutionSample& sample :
       candidate.robot_trajectory.execution_profile.samples) {
    sample.arc_length_m -= 1.0;
    sample.time_from_start.nanoseconds -= 2'000'000'000;
  }
  candidate.robot_trajectory.execution_profile.stopping_point_arc_length_m =
      2.0;
  candidate.execution_profile_version = 42;
  candidate.robot_trajectory.execution_profile.version = 42;
  candidate.diagnostics.dependencies.execution_profile_version = 42;
  require(manager.registerDependentPlan(candidate.sequence_number, locked, kNow),
          "candidate fixture could not be bound to the envelope");
  const ImmutablePlanningResult immutable_candidate = publish(candidate);
  PlanValidityEvaluator evaluator(CableModel(model_parameters()), config());

  const auto candidate_result = evaluator.validatePublicationCandidate(
      immutable_candidate, inputs(), validation_context(locked, manager), kNow);
  if (!(candidate_result.valid && candidate_result.lease.has_value() &&
        candidate_result.lease->execution_profile_version == 42)) {
    throw std::runtime_error(
        "a publication candidate could not advance from the tracked execution profile: status=" +
        std::to_string(static_cast<int>(candidate_result.status)) +
        " issue=" +
        (candidate_result.issues.empty() ? std::string{"none"}
                                         : candidate_result.issues.front()));
  }

  const auto current_result = evaluator.validateRemainingPlan(
      immutable_candidate, inputs(), validation_context(locked, manager), kNow);
  require(!current_result.valid &&
              current_result.status == PlanValidationStatus::state_mismatch &&
              !current_result.lease.has_value(),
          "current-plan validation stopped binding the plan to the tracked profile");
}

void revalidation_ignores_cached_cable_path_and_repredicts() {
  // Design: 18.2.7-4
  // Design: 18.2.7-7
  CableUncertaintyEnvelopeManager manager;
  const auto locked = lock_envelope(manager);
  PlanningResult cached_candidate = plan(locked, manager);
  for (PathPoint& point : cached_candidate.cable_path.points) {
    point.y_m = 100.0;
  }
  const ImmutablePlanningResult immutable_plan = publish(cached_candidate);

  PlanValidityEvaluator baseline_evaluator(CableModel(model_parameters()),
                                            config());
  const auto baseline = baseline_evaluator.validateRemainingPlan(
      immutable_plan, inputs(), validation_context(locked, manager), kNow);
  require(baseline.valid && baseline.cable_prediction &&
              baseline.cable_prediction->touchdown_path.points.front().y_m !=
                  immutable_plan->cable_path.points.front().y_m,
          "remaining-plan validation reused the cached cable path");

  auto changed_inputs = inputs();
  changed_inputs.cable_state.lag_angle_rad = 0.02;
  auto changed_config = config();
  changed_config.last_issued_lease_sequence = 1U;
  PlanValidityEvaluator changed_evaluator(CableModel(model_parameters()),
                                           changed_config);
  const auto changed = changed_evaluator.validateRemainingPlan(
      immutable_plan, changed_inputs, validation_context(locked, manager),
      kNow);
  require(changed.cable_prediction &&
              changed.cable_prediction->touchdown_path.points.front().y_m !=
                  baseline.cable_prediction->touchdown_path.points.front().y_m,
          "current cable state did not drive a fresh cable prediction");

  auto unsafe_inputs = inputs();
  unsafe_inputs.cable_state.lag_angle_rad = 1.3;
  auto unsafe_config = config();
  unsafe_config.last_issued_lease_sequence = 2U;
  PlanValidityEvaluator unsafe_evaluator(CableModel(model_parameters()),
                                          unsafe_config);
  const auto unsafe = unsafe_evaluator.validateRemainingPlan(
      immutable_plan, unsafe_inputs, validation_context(locked, manager), kNow);
  require(!unsafe.valid &&
              unsafe.status == PlanValidationStatus::cable_model_invalid &&
              unsafe.action == PlanValidationAction::replan &&
              !unsafe.lease.has_value(),
          "an out-of-domain current lag angle reused the cached cable path");
}

void failures_never_issue_or_advance_a_lease() {
  CableUncertaintyEnvelopeManager manager;
  const auto locked = lock_envelope(manager);
  const ImmutablePlanningResult immutable_plan = publish(plan(locked, manager));
  PlanValidityEvaluator evaluator(CableModel(model_parameters()), config());
  auto bad_state = inputs();
  bad_state.robot_state.pose.x_m += 0.2;
  const auto state_result = evaluator.validateRemainingPlan(
      immutable_plan, bad_state, validation_context(locked, manager), kNow);
  require(state_result.status == PlanValidationStatus::state_mismatch &&
              state_result.action == PlanValidationAction::replan &&
              !state_result.lease.has_value() && evaluator.next_lease_sequence() == 1,
          "state mismatch issued or consumed a lease sequence");

  auto stale = inputs();
  const auto stale_result = evaluator.validateRemainingPlan(
      immutable_plan, stale, validation_context(locked, manager),
      MonotonicTime{6'000'000'001});
  require(stale_result.status == PlanValidationStatus::input_expired &&
              stale_result.action == PlanValidationAction::stop &&
              !stale_result.lease.has_value(),
          "expired synchronized input did not fail closed");

  auto mismatched = inputs();
  mismatched.dependencies.map_version.sequence_number = 2;
  const auto context_result = evaluator.validateRemainingPlan(
      immutable_plan, mismatched, validation_context(locked, manager), kNow);
  require(context_result.status == PlanValidationStatus::context_mismatch &&
              context_result.action == PlanValidationAction::replan &&
              !context_result.lease.has_value(),
          "dependency mismatch received a lease");

  auto profile_mismatch = inputs();
  profile_mismatch.execution_tracking_state.execution_profile_version = 42;
  const auto profile_version_result = evaluator.validateRemainingPlan(
      immutable_plan, profile_mismatch, validation_context(locked, manager),
      kNow);
  require(profile_version_result.status == PlanValidationStatus::state_mismatch &&
              !profile_version_result.lease.has_value(),
          "profile version mismatch was hidden by generic context failure");

  auto policy_context = validation_context(locked, manager);
  policy_context.corridor_policy.version = 99;
  const auto policy_result = evaluator.validateRemainingPlan(
      immutable_plan, inputs(), policy_context, kNow);
  require(policy_result.status == PlanValidationStatus::context_mismatch &&
              !policy_result.lease.has_value(),
          "a mismatched current policy object received a lease");

  auto discontinuous = inputs();
  discontinuous.cable_telemetry.payout_speed_mps = 0.7;
  const auto profile_result = evaluator.validateRemainingPlan(
      immutable_plan, discontinuous, validation_context(locked, manager), kNow);
  require(profile_result.status ==
              PlanValidationStatus::execution_profile_mismatch &&
              !profile_result.lease.has_value(),
          "execution-profile discontinuity received a lease");

  auto acceleration = inputs();
  acceleration.execution_tracking_state.ground_acceleration_mps2 = 0.2;
  const auto acceleration_result = evaluator.validateRemainingPlan(
      immutable_plan, acceleration, validation_context(locked, manager), kNow);
  require(acceleration_result.status ==
              PlanValidationStatus::execution_profile_mismatch &&
              !acceleration_result.lease.has_value(),
          "ground-acceleration discontinuity received a lease");

  auto invalid_acceleration = inputs();
  invalid_acceleration.execution_tracking_state.ground_acceleration_mps2 =
      std::numeric_limits<double>::quiet_NaN();
  const auto invalid_acceleration_result = evaluator.validateRemainingPlan(
      immutable_plan, invalid_acceleration, validation_context(locked, manager),
      kNow);
  require(invalid_acceleration_result.status ==
              PlanValidationStatus::input_invalid &&
              invalid_acceleration_result.action == PlanValidationAction::stop &&
              !invalid_acceleration_result.lease.has_value(),
          "non-finite ground acceleration received a lease");

  auto mismatched_map_snapshot = inputs();
  mismatched_map_snapshot.planning_snapshot.map.version.sequence_number = 2;
  const auto map_snapshot_result = evaluator.validateRemainingPlan(
      immutable_plan, mismatched_map_snapshot,
      validation_context(locked, manager), kNow);
  require(map_snapshot_result.status == PlanValidationStatus::context_mismatch &&
              !map_snapshot_result.lease.has_value(),
          "a snapshot map outside the frozen dependency tuple received a lease");

  auto mismatched_corridor_snapshot = inputs();
  ++mismatched_corridor_snapshot.planning_snapshot.cable_corridor.version;
  const auto corridor_snapshot_result = evaluator.validateRemainingPlan(
      immutable_plan, mismatched_corridor_snapshot,
      validation_context(locked, manager), kNow);
  require(corridor_snapshot_result.status ==
              PlanValidationStatus::context_mismatch &&
              !corridor_snapshot_result.lease.has_value(),
          "a snapshot cable corridor outside the frozen dependency tuple received a lease");

  auto mismatched_config = config();
  mismatched_config.operating_domain_id = "other-domain";
  PlanValidityEvaluator mismatched_config_evaluator(
      CableModel(model_parameters()), mismatched_config);
  const auto config_result = mismatched_config_evaluator.validateRemainingPlan(
      immutable_plan, inputs(), validation_context(locked, manager), kNow);
  require(config_result.status == PlanValidationStatus::context_mismatch &&
              !config_result.lease.has_value(),
          "an evaluator configuration from another domain received a lease");

  auto unsynchronized = inputs();
  unsynchronized.reference_progress.timestamp = {1'200'000'001};
  const auto synchronization_result = evaluator.validateRemainingPlan(
      immutable_plan, unsynchronized, validation_context(locked, manager),
      kNow);
  require(synchronization_result.status == PlanValidationStatus::input_invalid &&
              synchronization_result.action == PlanValidationAction::stop &&
              !synchronization_result.lease.has_value(),
          "non-atomic synchronized inputs received a lease");

  auto laying_context = validation_context(locked, manager);
  laying_context.laying_limits.maximum_curvature_per_m = -1.0;
  const auto laying_result = evaluator.validateRemainingPlan(
      immutable_plan, inputs(), laying_context, kNow);
  require(laying_result.status == PlanValidationStatus::cable_laying_invalid &&
              !laying_result.lease.has_value(),
          "mechanical validation failure was not reported precisely");
}

void insufficient_stopping_distance_requires_stop() {
  CableUncertaintyEnvelopeManager manager;
  const auto locked = lock_envelope(manager);
  const ImmutablePlanningResult short_plan = publish(plan(locked, manager));
  auto state = inputs();
  state.execution_tracking_state.tracked_arc_length_m = 2.9;
  state.robot_state.pose.x_m = 2.9;
  state.robot_state.ground_speed_mps = 0.05;
  state.execution_tracking_state.ground_acceleration_mps2 = -0.225;
  state.cable_telemetry.payout_speed_mps = 0.05;
  state.cable_telemetry.payout_acceleration_mps2 = -0.225;
  PlanValidityEvaluator evaluator(CableModel(model_parameters()), config());
  const auto result = evaluator.validateRemainingPlan(
      short_plan, state, validation_context(locked, manager), kNow);
  require(result.status == PlanValidationStatus::stopping_distance_insufficient &&
              result.action == PlanValidationAction::stop &&
              !result.lease.has_value(),
          "unsafe remaining stopping distance did not require stop");
}

void curvature_age_bounds_the_lease() {
  CableUncertaintyEnvelopeManager manager;
  const auto locked = lock_envelope(manager);
  const ImmutablePlanningResult immutable_plan = publish(plan(locked, manager));
  auto state = inputs();
  state.robot_state.curvature_timestamp = {950'000'000};
  auto evaluator_config = config();
  evaluator_config.maximum_reuse_duration = {10'000'000'000};
  auto context = validation_context(locked, manager);
  context.smoothing_limits.maximum_boundary_time_skew = {100'000'000};
  PlanValidityEvaluator evaluator(CableModel(model_parameters()), evaluator_config);
  const auto result = evaluator.validateRemainingPlan(
      immutable_plan, state, context, kNow);
  require(result.valid && result.lease.has_value() &&
              result.lease->robot_state_timestamp.nanoseconds == 950'000'000 &&
              result.lease->expires_at.nanoseconds == 5'950'000'000,
          "curvature freshness did not bound the robot-state lease lifetime");
}

}  // namespace

int main() {
  try {
    successful_recheck_crops_and_issues_a_new_lease();
    publication_candidate_may_advance_the_execution_profile_version();
    revalidation_ignores_cached_cable_path_and_repredicts();
    failures_never_issue_or_advance_a_lease();
    insufficient_stopping_distance_requires_stop();
    curvature_age_bounds_the_lease();
  } catch (const std::exception& error) {
    std::cerr << "T31 failure: " << error.what() << '\n';
    return 1;
  }
  std::cout << "T31 plan validity evaluator checks passed\n";
  return 0;
}
