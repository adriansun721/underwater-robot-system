#include "underwater_planner/core/algorithm_diagnostics.hpp"
#include "underwater_planner/core/main_planning_loop.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
using namespace underwater_planner::core;

constexpr const char* kDomain = "competition-seabed-v1";

void require(const bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

PlanningDependencyVersions dependencies() {
  return {{"map", 7, {100}, "world"}, 4, 5, 6, 7, 8, 9, 10, 11, 12,
          SensorHealthMode::nominal, kDomain, 3};
}

ParameterConfig parameters() {
  ParameterConfig config;
  config.profile_id = "main-loop-test-v1";
  config.mode = ParameterProfileMode::non_production_capability_profile;
  config.operating_domain_id = kDomain;
  config.search.maximum_active_labels = 100U;
  config.statistical_risk.maximum_planning_duration_s = 0.5;
  return config;
}

AlgorithmRuntimeParameterSnapshot runtime_parameters() {
  AlgorithmRuntimeParameterSnapshot snapshot;
  snapshot.profile = parameters();
  snapshot.terrain_analysis.config_version = 31U;
  snapshot.search.version = 32U;
  snapshot.search.primitive_set_version = 33U;
  snapshot.search.maximum_expansions = 10'000U;
  snapshot.search.motion_primitives = {{33U, 0.5, -0.2},
                                       {33U, 0.5, 0.2}};
  snapshot.smoothing.version = 34U;
  snapshot.smoothing.timeout = {50'000'000};
  snapshot.parameterization.version = 35U;
  snapshot.parameterization.timeout = {50'000'000};
  return snapshot;
}

template <typename Target>
void apply_dependencies(Target& target,
                        const PlanningDependencyVersions& versions) {
  target.map_version = versions.map_version;
  target.reference_line_version = versions.reference_line_version;
  target.robot_operating_area_version = versions.robot_operating_area_version;
  target.terrain_gradient_policy_version =
      versions.terrain_gradient_policy_version;
  target.corridor_risk_policy_version = versions.corridor_risk_policy_version;
  target.cable_model_version = versions.cable_model_version;
  target.uncertainty_envelope_version = versions.uncertainty_envelope_version;
  target.uncertainty_envelope_generator_version =
      versions.uncertainty_envelope_generator_version;
  target.execution_operating_envelope_version =
      versions.execution_operating_envelope_version;
  target.execution_profile_version = versions.execution_profile_version;
  target.sensor_mode = versions.sensor_mode;
  target.operating_domain_id = versions.operating_domain_id;
  target.cable_corridor_version = versions.cable_corridor_version;
}

GeometricPath path(const double start_x_m = 0.0) {
  GeometricPath value;
  value.metadata = {31, "world", 4, "linear"};
  value.points = {{0.0, start_x_m, 0.0, 0.0, 0.0},
                  {1.0, start_x_m + 1.0, 0.0, 0.0, 0.0}};
  return value;
}

TimedPath timed_path(const GeometricPath& geometry = path()) {
  TimedPath value;
  value.geometry = geometry;
  value.execution_profile.version = 13;
  value.execution_profile.operating_envelope_version = 11;
  value.execution_profile.interpolation_rule = "linear";
  value.execution_profile.stopping_point_arc_length_m = 1.0;
  value.execution_profile.samples = {
      {0.0, {0}, 0.5, 0.0, 0.5, 0.0, 40.0},
      {1.0, {2'000'000'000}, 0.0, -0.25, 0.0, -0.25, 40.0}};
  value.execution_profile.approved_tracking_limits.ground_speed = {0.0, 1.0};
  value.execution_profile.approved_tracking_limits.ground_acceleration =
      {-1.0, 1.0};
  value.execution_profile.approved_tracking_limits.payout_speed = {0.0, 1.0};
  value.execution_profile.approved_tracking_limits.payout_acceleration =
      {-1.0, 1.0};
  value.execution_profile.approved_tracking_limits.tension = {0.0, 100.0};
  value.execution_profile.approved_tracking_limits.maximum_lateral_acceleration_mps2 =
      1.0;
  value.execution_profile.approved_tracking_limits.maximum_payout_tracking_error_mps =
      0.1;
  value.execution_profile.approved_tracking_limits.maximum_stopping_distance_m =
      1.0;
  return value;
}

TimedPath committed_prefix() {
  TimedPath value;
  value.geometry.metadata = {30, "world", 4, "linear"};
  value.geometry.points = {{0.0, 0.0, 0.0, 0.0, 0.0},
                           {0.25, 0.25, 0.0, 0.0, 0.0}};
  value.execution_profile.version = 12;
  value.execution_profile.operating_envelope_version = 11;
  value.execution_profile.interpolation_rule = "linear";
  value.execution_profile.stopping_point_arc_length_m = 1.0;
  value.execution_profile.samples = {
      {0.0, {0}, 0.5, 0.0, 0.5, 0.0, 40.0},
      {0.25, {500'000'000}, 0.5, 0.0, 0.5, 0.0, 40.0}};
  value.execution_profile.approved_tracking_limits =
      timed_path().execution_profile.approved_tracking_limits;
  return value;
}

void apply_prediction_dependencies(CablePrediction& prediction,
                                   const TimedPath& robot_path) {
  prediction.dependencies.robot_path_version =
      robot_path.geometry.metadata.path_version;
  prediction.dependencies.cable_model_version = 8;
  prediction.dependencies.reference_line_version = 4;
  prediction.dependencies.execution_profile_version =
      robot_path.execution_profile.version;
  prediction.dependencies.execution_operating_envelope_version = 11;
  prediction.dependencies.uncertainty_envelope_version = 9;
  prediction.dependencies.uncertainty_envelope_generator_version = 10;
  prediction.dependencies.sensor_mode = SensorHealthMode::nominal;
  prediction.dependencies.operating_domain_id = kDomain;
  prediction.dependencies.execution_operating_domain_id = kDomain;
}

SynchronizedValidationInputs inputs(const std::uint64_t revision) {
  SynchronizedValidationInputs value;
  value.captured_at = {200};
  value.source_revision = revision;
  value.robot_state = {{0.0, 0.0, 0.0, {100}}, 0.5, 0.0, {100}, 20};
  value.cable_state.kind = CableStateKind::tracked;
  value.cable_state.lag_angle_rad = 0.0;
  value.cable_state.lag_angle_variance_rad2 = 0.01;
  value.cable_state.timestamp = {100};
  value.cable_state.sequence_number = 21;
  value.reference_progress = {4, 0.0, {100}, 22};
  value.cable_telemetry = {0.5, 0.0, 40.0, {100}, 23};
  value.execution_tracking_state = {12, 11, 0.0, {100}, 24, 0.0};
  value.planning_snapshot.map.version = dependencies().map_version;
  value.planning_snapshot.reference_line.version = 4;
  value.planning_snapshot.reference_line.coordinate_frame = "world";
  value.planning_snapshot.reference_line.points = {
      {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0},
      {1.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0}};
  value.planning_snapshot.robot_operating_area =
      {5, "area", {{-1.0, -1.0}, {2.0, -1.0}, {2.0, 1.0}, {-1.0, 1.0}}};
  value.planning_snapshot.cable_corridor =
      {3, "corridor", {{-1.0, -1.0}, {2.0, -1.0}, {2.0, 1.0}, {-1.0, 1.0}}};
  value.dependencies = dependencies();
  value.tracker_update_receipt = {25, 26, std::nullopt, 23, 21, 22};
  return value;
}

TerrainLayers terrain() {
  TerrainLayers value;
  value.source_map_version = dependencies().map_version;
  value.analysis_config_version = 6;
  value.operating_domain_id = kDomain;
  return value;
}

PlanningResult candidate() {
  PlanningResult value;
  value.sequence_number = 50;
  value.timestamp = {500};
  value.validity_duration = {5'000'000'000};
  value.state = PlanningState::success;
  value.robot_trajectory = timed_path();
  value.cable_path = path();
  value.cable_path.metadata.path_version = 32;
  value.terminal_cable_state.kind = CableStateKind::tracked;
  value.terminal_cable_state.lag_angle_rad = 0.0;
  value.terminal_cable_state.lag_angle_variance_rad2 = 0.01;
  value.terminal_cable_state.timestamp = {500};
  value.terminal_cable_state.sequence_number = 30;
  value.cable_model_validity = CableModelValidity::valid;
  value.corridor_result.validity = CorridorEvaluationValidity::valid;
  value.corridor_result.hard_feasible = true;
  value.corridor_result.points = {
      {CableValidationStatus::pass, 0.0, 0.01, 0.03,
       CableCorridorPointBasis::below_nominal_bound, 0.0, 0.0},
      {CableValidationStatus::pass, 0.0, 0.01, 0.03,
       CableCorridorPointBasis::below_nominal_bound, 1.0, 1.0}};
  value.corridor_result.epsilon_point = 0.01;
  value.corridor_result.corridor_risk_policy_version = 7;
  value.corridor_result.reference_line_version = 4;
  value.corridor_result.interval_bound_certificate.version = 1;
  value.corridor_result.interval_bound_certificate.upper_bound_error_m = {0.0};
  value.corridor_result.evaluation_timestamp = {500};
  value.corridor_result.operating_domain_id = kDomain;
  value.corridor_result.residual_distribution_calibration_dataset_id =
      "corridor-cal-v1";
  value.corridor_result.covariance_includes_coordinate_transform_error = true;
  value.corridor_result.covariance_envelope_audit_performed = true;
  value.corridor_result.risk_semantics =
      "POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE";
  value.cable_laying_result.valid = true;
  value.cable_laying_result.hard_feasible = true;
  value.cable_laying_result.failure_reasons = {CableLayingFailure::none};
  value.cable_laying_result.limits_version = 1;
  value.cable_laying_result.terrain_map_sequence = 7;
  value.cable_laying_result.terrain_analysis_config_version = 6;
  value.cable_laying_result.operating_domain_id = kDomain;
  value.cable_laying_result.risk_semantics =
      "CONSERVATIVE_SUPPORT_PROXY:NO_FLEXIBLE_CABLE_DYNAMICS_GUARANTEE";
  value.error_budget.touchdown_position_covariance_m2 =
      {{0.01, 0.0, 0.0, 0.01}, {0.01, 0.0, 0.0, 0.01}};
  value.error_budget.epsilon_robot = 0.01;
  value.error_budget.epsilon_terrain_gradient_local = 0.01;
  value.error_budget.epsilon_point = 0.01;
  value.error_budget.calibration_dataset_id = "corridor-cal-v1";
  value.error_budget.terrain_gradient_calibration_dataset_id = "terrain-cal-v1";
  value.error_budget.terrain_gradient_policy_version = 6;
  value.error_budget.corridor_risk_policy_version = 7;
  value.error_budget.cable_model_version = 8;
  value.error_budget.uncertainty_envelope_version = 9;
  value.error_budget.uncertainty_envelope_generator_version = 10;
  value.error_budget.execution_operating_envelope_version = 11;
  value.error_budget.operating_domain_id = kDomain;
  value.error_budget.covariance_envelope_audit_passed = true;
  const auto versions = dependencies();
  apply_dependencies(value, versions);
  value.execution_profile_version = 13;
  value.diagnostics.schema_version = "planning-cycle/v1";
  value.diagnostics.input_version = "fixture/v1";
  value.diagnostics.unit_system = "SI";
  value.diagnostics.operating_domain_id = kDomain;
  value.diagnostics.risk_semantics =
      "POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE";
  value.diagnostics.dependencies = versions;
  value.diagnostics.dependencies.execution_profile_version = 13;
  return value;
}

PlanValidationLease lease(const std::uint64_t execution_profile_version = 13,
                          const std::uint64_t lease_sequence = 70,
                          const std::int64_t expires_at_ns = 10'000,
                          const std::uint64_t plan_sequence_number = 50) {
  PlanValidationLease value;
  value.lease_sequence = lease_sequence;
  value.plan_sequence_number = plan_sequence_number;
  value.evaluator_config_version = 1;
  value.parameter_profile_id = "competition-v1";
  value.validated_at = {600};
  value.expires_at = {expires_at_ns};
  const auto versions = dependencies();
  apply_dependencies(value, versions);
  value.execution_profile_version = execution_profile_version;
  value.robot_path_validation_passed = true;
  value.cable_corridor_validation_passed = true;
  value.cable_laying_validation_passed = true;
  return value;
}

ActiveExecutionContext execution_context() {
  ActiveExecutionContext value;
  apply_dependencies(value, dependencies());
  value.execution_profile_version = 13;
  return value;
}

ExecutionFeedback execution_feedback(const PlanningResult& plan) {
  ExecutionFeedback value;
  value.plan_sequence_number = plan.sequence_number;
  value.execution_profile_version = plan.execution_profile_version;
  value.timestamp = {900};
  value.ground_speed_mps = 0.5;
  value.ground_acceleration_mps2 = 0.0;
  value.payout_speed_mps = 0.5;
  value.payout_acceleration_mps2 = 0.0;
  value.tension_n = 40.0;
  value.tracked_arc_length_m = 0.0;
  value.sequence_number = 1;
  return value;
}

void require_followup_command_rejected(
    const ExecutionLeaseMonitor& monitor, const PlanningResult& plan,
    PlanValidationLease authorization) {
  authorization.max_ground_speed_tracking_error_mps = 0.1;
  authorization.max_payout_speed_tracking_error_mps = 0.1;
  authorization.allowed_ground_acceleration = {-1.0, 1.0};
  authorization.allowed_tension = {0.0, 100.0};
  const ExecutionAuthorization followup = monitor.evaluate(
      plan, plan.robot_trajectory, authorization, execution_context(),
      execution_feedback(plan), MonotonicTime{1'000});
  require(followup.revoked() && !followup.authorized() &&
              followup.request_controlled_stop &&
              followup.reason_code == "LEASE_ALREADY_REVOKED",
          "a safety failure allowed a follow-up path command");
}

AuthorizedPlanningPublication publish_candidate(
    AuthorizedPlanningResultPublisher& publisher,
    const PlanValidationLease& authorization) {
  const PlanningResult plan = candidate();
  return publisher.publish(
      plan, std::make_shared<const TimedPath>(plan.robot_trajectory),
      authorization, 100.0);
}

class SuccessfulStages final : public MainPlanningLoopStages {
 public:
  AlgorithmRuntimeParameterSnapshot capture_runtime_parameters()
      const override {
    return runtime_parameters();
  }

  explicit SuccessfulStages(
      const bool stale_decision_context = false,
      const PathHysteresisConfig hysteresis_config = {})
      : MainPlanningLoopStages(hysteresis_config),
        stale_decision_context_(stale_decision_context) {}

  ValidationInputCaptureResult capture(MonotonicTime) override {
    const bool initial_capture = captures++ == 0;
    order.push_back(initial_capture
                        ? PlanningCycleStage::capture_inputs
                        : PlanningCycleStage::decision_context_capture);
    ValidationInputCaptureResult result;
    result.status = initial_capture ? initial_capture_status
                                    : decision_capture_status;
    if (result.status != ValidationInputCaptureStatus::captured) return result;
    result.inputs = inputs(captures == 1 ? 100 : 101);
    if (captures == 2 && stale_decision_context_) {
      result.inputs->dependencies.map_version.sequence_number = 8;
      result.inputs->planning_snapshot.map.version.sequence_number = 8;
    }
    return result;
  }

  TerrainAnalysisStageResult analyze_terrain(
      const SynchronizedValidationInputs& captured) override {
    observe(PlanningCycleStage::terrain_analysis, captured.source_revision);
    return {true, terrain(), {}, {}};
  }

  CommitmentValidationStageResult validate_commitment(
      const TimedPath& authorized_prefix,
      const CableState& synchronized_actual_cable_state,
      const ReferenceProgress&, const LockedPlanningCycleContext& context) override {
    observe(PlanningCycleStage::commitment_validation,
            context.inputs.source_revision);
    observed_commitment_cable_state = synchronized_actual_cable_state;
    CommitmentValidationStageResult result;
    result.valid = true;
    CablePrediction prediction;
    prediction.validity = CableModelValidity::valid;
    prediction.terminal_state = synchronized_actual_cable_state;
    prediction.terminal_state.sequence_number = 91;
    apply_prediction_dependencies(prediction, authorized_prefix);
    result.terminal_reference_progress = {4, 0.25, {100}, 92};
    result.robot_validation.status = commitment_robot_status;
    result.robot_validation.valid =
        commitment_robot_status == PathCandidateVerificationStatus::valid;
    result.cable_validation.status = commitment_cable_status;
    result.cable_validation.valid =
        commitment_cable_status == TimedCableValidationStatus::valid;
    result.cable_validation.stop_required = commitment_cable_stop_required;
    result.cable_validation.cable_prediction = prediction;
    result.cable_validation.terminal_cable_state = prediction.terminal_state;
    result.cable_validation.corridor_result = candidate().corridor_result;
    result.cable_validation.laying_result = candidate().cable_laying_result;
    result.cable_validation.laying_result.terminal_memory =
        prediction.terminal_state.laying_memory;
    result.observed_safety_event = commitment_safety_event;
    result.obstacle_stopping = commitment_obstacle_stopping;
    return result;
  }

  void request_commitment_safety_stop(
      const CommitmentSafetyCheckResult& safety,
      MonotonicTime) override {
    ++safety_stop_requests;
    observed_safety_action = safety.action;
    lease_was_revoked_before_stop =
        revoked_probe ? revoked_probe() : false;
  }

  void request_controlled_stop(const PlanningFailure& failure,
                               MonotonicTime) override {
    ++controlled_stop_requests;
    controlled_stop_reason = failure.reason_code;
    lease_was_revoked_before_controlled_stop =
        revoked_probe ? revoked_probe() : true;
  }

  HybridAStarPlanningResult search(
      const PlanningCycleStart& start,
      const LockedPlanningCycleContext& context) override {
    observe(PlanningCycleStage::search, context.inputs.source_revision);
    observed_start = start;
    HybridAStarPlanningResult result;
    result.state = search_state;
    result.diagnostics.fixed_bytes_per_search_label = 1U;
    result.diagnostics.peak_observed_bytes_per_search_label = 1U;
    result.diagnostics.deadline_exceeded = search_deadline_exceeded;
    result.diagnostics.active_label_budget_exhausted =
        search_label_budget_exhausted;
    result.diagnostics.solution_cost = candidate_cost;
    if (search_state == PlanningState::success) {
      result.robot_path = path(start.robot_state.pose.x_m);
    }
    return result;
  }

  SmoothingResult smooth(const GeometricPath& raw_path,
                         const PlanningCycleStart&,
                         const LockedPlanningCycleContext& context) override {
    observe(PlanningCycleStage::smoothing, context.inputs.source_revision);
    SmoothingResult result;
    result.status = smoothing_status;
    if (smoothing_status == SmoothingStatus::success) result.path = raw_path;
    return result;
  }

  TrackabilityResult validate_raw_path_trackability(
      const GeometricPath&, const PlanningCycleStart&,
      const LockedPlanningCycleContext& context) override {
    observe(PlanningCycleStage::raw_path_trackability_validation,
            context.inputs.source_revision);
    return {raw_trackability_valid,
            raw_trackability_valid ? "raw_path_trackable"
                                   : "raw_path_not_trackable",
            {}};
  }

  ParameterizationResult parameterize(
      const GeometricPath& geometry, const PlanningCycleStart&,
      const LockedPlanningCycleContext& context) override {
    observe(PlanningCycleStage::parameterization, context.inputs.source_revision);
    ParameterizationResult result;
    result.status = parameterization_status;
    if (parameterization_status == ParameterizationStatus::success) {
      result.trajectory = timed_path(geometry);
    }
    return result;
  }

  TimedPathMergeResult merge_commitment(
      const TimedPath& authorized_prefix, const TimedPath& new_tail,
      const LockedPlanningCycleContext& context) override {
    observe(PlanningCycleStage::commitment_merge,
            context.inputs.source_revision);
    return StabilityManager().merge_timed_paths(
        authorized_prefix, new_tail, {1.0e-9, 1.0e-9, 1.0e-9},
        {1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9},
        [](const TimedPath& complete) { return validate(complete).valid; });
  }

  PathCandidateVerificationResult verify_complete_robot_path(
      const TimedPath& complete_path, const PlanningCycleStart&,
      const LockedPlanningCycleContext& context) override {
    observe(PlanningCycleStage::complete_robot_path_validation,
            context.inputs.source_revision);
    observed_complete_robot_path = complete_path;
    PathCandidateVerificationResult result;
    result.status = complete_robot_path_status;
    result.valid = complete_robot_path_status ==
                   PathCandidateVerificationStatus::valid;
    result.geometry.valid = result.valid;
    return result;
  }

  TimedCableCandidateResult verify_cable(
      const TimedPath& complete_path,
      const CableState& synchronized_actual_cable_state,
      const LockedPlanningCycleContext& context) override {
    observe(PlanningCycleStage::cable_validation,
            context.inputs.source_revision);
    observed_cable_validation_state = synchronized_actual_cable_state;
    const PlanningResult evidence = candidate();
    TimedCableCandidateResult result;
    result.status = cable_validation_status;
    result.valid = cable_validation_status == TimedCableValidationStatus::valid;
    result.stop_required = cable_stop_required;
    CablePrediction prediction;
    prediction.validity = CableModelValidity::valid;
    prediction.touchdown_path = complete_path.geometry;
    prediction.touchdown_path.metadata.path_version += 100;
    apply_prediction_dependencies(prediction, complete_path);
    prediction.terminal_state = evidence.terminal_cable_state;
    result.cable_prediction = prediction;
    result.terminal_cable_state = prediction.terminal_state;
    result.corridor_result = evidence.corridor_result;
    result.corridor_result.points.assign(
        prediction.touchdown_path.points.size(),
        {CableValidationStatus::pass, 0.0, 0.01, 0.03,
         CableCorridorPointBasis::below_nominal_bound, 0.0, 0.0});
    for (std::size_t index = 0; index < result.corridor_result.points.size();
         ++index) {
      result.corridor_result.points[index].touchdown_arc_length_m =
          prediction.touchdown_path.points[index].arc_length_m;
      result.corridor_result.points[index].reference_progress_m =
          prediction.touchdown_path.points[index].arc_length_m;
    }
    result.corridor_result.interval_bound_certificate.upper_bound_error_m.assign(
        prediction.touchdown_path.points.size() - 1U, 0.0);
    result.laying_result = evidence.cable_laying_result;
    return result;
  }

  PlanningCandidateMetadata assemble_candidate_metadata(
      const PlanningCycleRequest&, const PlanningCycleStart&,
      const LockedPlanningCycleContext& context,
      const HybridAStarPlanningResult& search_result, const SmoothingResult&,
      const ParameterizationResult&,
      const TimedCableCandidateResult& cable_result) override {
    observe(PlanningCycleStage::candidate_assembly,
            context.inputs.source_revision);
    const PlanningResult evidence = candidate();
    PlanningCandidateMetadata result;
    result.sequence_number = candidate_sequence_number;
    result.timestamp = evidence.timestamp;
    result.validity_duration = evidence.validity_duration;
    result.path_cost = search_result.diagnostics.solution_cost;
    result.error_budget = evidence.error_budget;
    result.error_budget.touchdown_position_covariance_m2.resize(
        cable_result.cable_prediction->touchdown_path.points.size(),
        {0.01, 0.0, 0.0, 0.01});
    result.diagnostics = evidence.diagnostics;
    return result;
  }

  PlanValidityEvaluation revalidate_plan(
      const PlanningResult& candidate_plan, const PlanValidationTarget target,
      const SynchronizedValidationInputs& latest,
      MonotonicTime) override {
    const bool is_candidate =
        target == PlanValidationTarget::publication_candidate;
    observe(is_candidate ? PlanningCycleStage::candidate_revalidation
                         : PlanningCycleStage::current_plan_revalidation,
            latest.source_revision);
    PlanValidityEvaluation result;
    const bool valid = is_candidate ? revalidation_valid
                                    : current_plan_revalidation_valid;
    const PlanValidationStatus status =
        is_candidate ? revalidation_status
                     : current_plan_revalidation_status;
    const PlanValidationAction action =
        is_candidate ? revalidation_action
                     : current_plan_revalidation_action;
    result.valid = valid;
    result.status = status;
    result.action = action;
    result.evaluator_config_version = 1;
    result.parameter_profile_id = "competition-v1";
    if (valid && status == PlanValidationStatus::valid &&
        action == PlanValidationAction::reuse) {
      result.lease = lease(candidate_plan.execution_profile_version,
                           revalidation_lease_sequence,
                           revalidation_lease_expires_at_ns,
                           candidate_plan.sequence_number);
      if (!is_candidate && current_plan_map_sequence_override.has_value()) {
        result.lease->map_version.sequence_number =
            *current_plan_map_sequence_override;
      }
      result.remaining_path =
          std::make_shared<const TimedPath>(candidate_plan.robot_trajectory);
    }
    return result;
  }

  void observe_candidate_decision_inputs(
      const std::optional<PlanValidityEvaluation>& validated_current,
      const PlanValidityEvaluation& validated_candidate,
      const SynchronizedValidationInputs& latest) override {
    static_cast<void>(validated_candidate);
    observe(PlanningCycleStage::candidate_decision, latest.source_revision);
    decision_saw_current = validated_current.has_value();
    decision_source_revision = latest.source_revision;
  }

  std::vector<PlanningCycleStage> order;
  std::vector<std::uint64_t> locked_revisions;
  PlanningCycleStart observed_start;
  CableState observed_commitment_cable_state;
  CableState observed_cable_validation_state;
  TimedPath observed_complete_robot_path;
  ValidationInputCaptureStatus initial_capture_status{
      ValidationInputCaptureStatus::captured};
  ValidationInputCaptureStatus decision_capture_status{
      ValidationInputCaptureStatus::captured};
  PlanningState search_state{PlanningState::success};
  bool search_deadline_exceeded{};
  bool search_label_budget_exhausted{};
  std::uint64_t revalidation_lease_sequence{70};
  std::int64_t revalidation_lease_expires_at_ns{10'000};
  bool revalidation_valid{true};
  PlanValidationStatus revalidation_status{PlanValidationStatus::valid};
  PlanValidationAction revalidation_action{PlanValidationAction::reuse};
  bool current_plan_revalidation_valid{true};
  PlanValidationStatus current_plan_revalidation_status{
      PlanValidationStatus::valid};
  PlanValidationAction current_plan_revalidation_action{
      PlanValidationAction::reuse};
  std::optional<std::uint64_t> current_plan_map_sequence_override;
  std::uint64_t candidate_sequence_number{50};
  double candidate_cost{80.0};
  PathCandidateVerificationStatus complete_robot_path_status{
      PathCandidateVerificationStatus::valid};
  SmoothingStatus smoothing_status{SmoothingStatus::success};
  bool raw_trackability_valid{true};
  ParameterizationStatus parameterization_status{
      ParameterizationStatus::success};
  TimedCableValidationStatus cable_validation_status{
      TimedCableValidationStatus::valid};
  bool cable_stop_required{};
  CommitmentSafetyEvent commitment_safety_event{CommitmentSafetyEvent::none};
  PathCandidateVerificationStatus commitment_robot_status{
      PathCandidateVerificationStatus::valid};
  TimedCableValidationStatus commitment_cable_status{
      TimedCableValidationStatus::valid};
  bool commitment_cable_stop_required{};
  std::optional<ObstacleStoppingEvidence> commitment_obstacle_stopping;
  std::size_t safety_stop_requests{};
  std::size_t controlled_stop_requests{};
  std::string controlled_stop_reason;
  bool lease_was_revoked_before_controlled_stop{};
  CommitmentSafetyAction observed_safety_action{
      CommitmentSafetyAction::continue_commitment};
  bool lease_was_revoked_before_stop{};
  bool decision_saw_current{};
  std::uint64_t decision_source_revision{};
  std::function<bool()> revoked_probe;
  std::function<void(PlanningCycleStage)> on_stage;

 private:
  void observe(const PlanningCycleStage stage, const std::uint64_t revision) {
    order.push_back(stage);
    locked_revisions.push_back(revision);
    if (on_stage) on_stage(stage);
  }

  std::uint64_t captures{};
  bool stale_decision_context_{};
};

void actual_state_success_path_is_published_with_a_fresh_lease() {
  SuccessfulStages stages;
  AuthorizedPlanningResultPublisher publisher;
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 1'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });

  PlanningCycleRequest request;
  request.cycle_sequence = 1;
  request.random_seed = 42;
  request.triggered_at = {1'000};
  const PlanningCycleResult result = loop.run_cycle(request);

  if (!result.succeeded()) {
    throw std::runtime_error(
        "successful planning chain did not publish: status=" +
        std::to_string(static_cast<int>(result.status)) + " issue=" +
        (result.issues.empty() ? std::string{"none"} : result.issues.front()));
  }
  require(result.start.has_value() &&
              result.start->source == PlanningStartSource::synchronized_actual_state &&
              result.start->robot_state.sequence_number == 20,
          "cycle did not start from synchronized actual state");
  require(result.publication.has_value() &&
              result.publication->lease.lease_sequence == 70 &&
              result.publication->lease.plan_sequence_number == 50 &&
              result.publication->remaining_path &&
              result.publication->path_cost == 80.0,
          "published plan was not paired with the fresh lease");
  require(result.publication->plan.value().sequence_number == 50 &&
              result.publication->plan.value().execution_profile_version == 13 &&
              publisher.current().has_value(),
          "candidate and lease did not cross the atomic publication boundary");
  require(stages.order ==
              std::vector<PlanningCycleStage>(
                  {PlanningCycleStage::capture_inputs,
                   PlanningCycleStage::terrain_analysis,
                   PlanningCycleStage::search, PlanningCycleStage::smoothing,
                   PlanningCycleStage::parameterization,
                   PlanningCycleStage::complete_robot_path_validation,
                   PlanningCycleStage::cable_validation,
                   PlanningCycleStage::candidate_assembly,
                   PlanningCycleStage::decision_context_capture,
                   PlanningCycleStage::candidate_revalidation,
                   PlanningCycleStage::candidate_decision}),
          "planning stages ran out of design order");
  require(stages.locked_revisions ==
              std::vector<std::uint64_t>(
                  {100, 100, 100, 100, 100, 100, 100, 101, 101}),
          "a stage observed a mutable or mixed planning snapshot");
  require(result.diagnostics.initial_source_revision == 100 &&
              result.diagnostics.decision_source_revision == 101 &&
              result.diagnostics.stages.size() == 13,
          "cycle diagnostics omitted revisions or stage timings");
  require(result.initial_inputs &&
              result.initial_inputs->source_revision == 100 &&
              result.initial_inputs->dependencies == dependencies(),
          "cycle result omitted the immutable initial replay input");
  require(loop.experiment_log().records().size() == 1U &&
              result.experiment_recorded &&
              result.experiment_record_valid &&
              result.experiment_recording_issues.empty() &&
              loop.experiment_log().records().front().cycle_sequence == 1U &&
              loop.experiment_log()
                      .records()
                      .front()
                      .parameters.profile.search.maximum_active_labels == 100U &&
              result.replay_input_captures.size() == 2U,
          "main planning loop did not automatically record the cycle");
  SuccessfulStages replay_stages;
  std::int64_t replay_now = 1'000;
  const AlgorithmExperimentReplayResult replay =
      AlgorithmExperimentReplayer::replay(
          loop.experiment_log().records().front(), replay_stages,
          [&replay_now] {
            replay_now += 10;
            return MonotonicTime{replay_now};
          });
  require(replay.status == AlgorithmExperimentReplayStatus::reproduced &&
              replay.differences.empty(),
          "recorded inputs and typed parameters did not reproduce the real loop");
  require(result.artifacts.terrain && result.artifacts.search &&
              result.artifacts.smoothing && result.artifacts.parameterization &&
              result.artifacts.complete_robot_path_validation &&
              result.artifacts.cable_validation &&
              result.artifacts.candidate_metadata &&
              result.artifacts.candidate &&
              result.artifacts.candidate_revalidation &&
              result.artifacts.candidate_decision,
          "cycle result omitted intermediate stage evidence");
  for (const PlanningCycleStageMetric& metric : result.diagnostics.stages) {
    require(metric.duration.nanoseconds == 10,
            "cycle result omitted a deterministic stage duration");
  }
}

void committed_terminal_state_is_the_planning_start() {
  SuccessfulStages stages;
  AuthorizedPlanningResultPublisher publisher;
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 2'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });

  PlanningCycleRequest request;
  request.cycle_sequence = 2;
  request.random_seed = 43;
  request.triggered_at = {2'000};
  CommittedPlanningStart committed;
  committed.source_plan_sequence_number = 49;
  committed.lease_sequence = 69;
  committed.dependencies = dependencies();
  committed.authorized_prefix = committed_prefix();
  committed.terminal_robot_state =
      {{0.25, 0.0, 0.0, {100}}, 0.5, 0.0, {100}, 90};
  request.committed_start = committed;

  const PlanningCycleResult result = loop.run_cycle(request);
  require(result.succeeded(), "committed-start success chain did not publish");
  require(stages.observed_start.source ==
              PlanningStartSource::committed_segment_terminal &&
              stages.observed_start.robot_state.sequence_number == 90 &&
              stages.observed_start.cable_state.sequence_number == 91 &&
              stages.observed_start.reference_progress.sequence_number == 92 &&
              stages.observed_start.source_plan_sequence_number == 49 &&
              stages.observed_start.lease_sequence == 69,
          "search did not consume the authorized commitment terminal state");
  require(result.artifacts.commitment_merge &&
              result.publication->plan.value().robot_trajectory.geometry.points
                      .front()
                      .x_m == 0.0 &&
              result.publication->plan.value().robot_trajectory.geometry.points[1]
                      .x_m == 0.25 &&
              result.publication->plan.value().robot_trajectory.geometry.points
                      .back()
                      .x_m == 1.25 &&
              stages.observed_commitment_cable_state.sequence_number == 21 &&
              stages.observed_cable_validation_state.sequence_number == 21,
          "commitment terminal state or complete path was not derived from actual state");
  require(result.diagnostics.stages.size() == 15 &&
              stages.order[2] == PlanningCycleStage::commitment_validation &&
              stages.order[6] == PlanningCycleStage::commitment_merge &&
              stages.order[7] ==
                  PlanningCycleStage::complete_robot_path_validation &&
              stages.observed_complete_robot_path.geometry.points.size() == 3,
          "committed path omitted merge evidence or timing");
}

void urgent_commitment_safety_event_revokes_before_stop_and_search() {
  // Design: 18.2.7-18
  SuccessfulStages stages;
  stages.commitment_safety_event = CommitmentSafetyEvent::new_obstacle;
  stages.commitment_obstacle_stopping = ObstacleStoppingEvidence{2.0, 1.0};
  AuthorizedPlanningResultPublisher publisher;
  const PlanValidationLease current_lease = lease(13, 69, 10'000, 50);
  require(publish_candidate(publisher, current_lease).published(),
          "test setup did not publish the current plan");
  ExecutionLeaseMonitor lease_monitor;
  stages.revoked_probe = [&lease_monitor] { return lease_monitor.isRevoked(69); };
  std::int64_t now = 2'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });

  PlanningCycleRequest request;
  request.cycle_sequence = 40;
  request.random_seed = 4040;
  request.triggered_at = {2'000};
  CommittedPlanningStart committed;
  committed.source_plan_sequence_number = 50;
  committed.lease_sequence = 69;
  committed.dependencies = dependencies();
  committed.authorized_prefix = committed_prefix();
  committed.terminal_robot_state =
      {{0.25, 0.0, 0.0, {100}}, 0.5, 0.0, {100}, 90};
  request.committed_start = committed;

  const PlanningCycleResult result = loop.run_cycle(request);
  require(result.status == PlanningCycleStatus::commitment_overridden &&
              result.state == PlanningState::normal_planning &&
              result.urgent_replan_required &&
              result.controlled_stop_required && !result.publication &&
              result.revoked_lease_sequence == 69 &&
              !publisher.current().has_value(),
          "urgent commitment event did not revoke and enter stopped replanning");
  require(result.commitment_safety &&
              result.commitment_safety->event ==
                  CommitmentSafetyEvent::new_obstacle &&
              result.commitment_safety->action ==
                  CommitmentSafetyAction::replan_urgent &&
              result.root_cause &&
              result.root_cause->reason_code == "COMMITMENT_NEW_OBSTACLE",
          "urgent commitment transition was not traceable");
  require(stages.safety_stop_requests == 1 &&
              stages.observed_safety_action ==
                  CommitmentSafetyAction::replan_urgent &&
              stages.lease_was_revoked_before_stop,
          "controlled-stop channel ran before durable lease revocation");
  require(stages.order ==
              std::vector<PlanningCycleStage>{
                  PlanningCycleStage::capture_inputs,
                  PlanningCycleStage::terrain_analysis,
                  PlanningCycleStage::commitment_validation},
          "search or hysteresis ran after a commitment safety override");
}

void too_close_obstacle_enters_emergency_stop() {
  SuccessfulStages stages;
  stages.commitment_safety_event = CommitmentSafetyEvent::new_obstacle;
  stages.commitment_obstacle_stopping = ObstacleStoppingEvidence{1.0, 1.0};
  AuthorizedPlanningResultPublisher publisher;
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 2'500;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });

  PlanningCycleRequest request;
  request.cycle_sequence = 41;
  request.random_seed = 4041;
  request.triggered_at = {2'500};
  CommittedPlanningStart committed;
  committed.source_plan_sequence_number = 49;
  committed.lease_sequence = 71;
  committed.dependencies = dependencies();
  committed.authorized_prefix = committed_prefix();
  committed.terminal_robot_state =
      {{0.25, 0.0, 0.0, {100}}, 0.5, 0.0, {100}, 90};
  request.committed_start = committed;

  const PlanningCycleResult result = loop.run_cycle(request);
  require(result.status == PlanningCycleStatus::commitment_overridden &&
              result.state == PlanningState::emergency_stop &&
              !result.urgent_replan_required &&
              result.controlled_stop_required && result.commitment_safety &&
              result.commitment_safety->event ==
                  CommitmentSafetyEvent::new_obstacle &&
              result.commitment_safety->action ==
                  CommitmentSafetyAction::stop &&
              stages.safety_stop_requests == 1,
          "too-close obstacle did not enter traceable emergency stop");
}

void revoked_commitment_cannot_be_reused_on_the_next_cycle() {
  // Design: 18.2.7-18
  SuccessfulStages stages;
  stages.commitment_safety_event = CommitmentSafetyEvent::execution_deviation;
  AuthorizedPlanningResultPublisher publisher;
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 2'700;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });

  PlanningCycleRequest request;
  request.cycle_sequence = 42;
  request.random_seed = 4042;
  request.triggered_at = {2'700};
  CommittedPlanningStart committed;
  committed.source_plan_sequence_number = 49;
  committed.lease_sequence = 72;
  committed.dependencies = dependencies();
  committed.authorized_prefix = committed_prefix();
  committed.terminal_robot_state =
      {{0.25, 0.0, 0.0, {100}}, 0.5, 0.0, {100}, 90};
  request.committed_start = committed;
  const PlanningCycleResult overridden = loop.run_cycle(request);
  require(overridden.status == PlanningCycleStatus::commitment_overridden &&
              lease_monitor.isRevoked(72),
          "test setup did not revoke the commitment lease");

  stages.order.clear();
  stages.locked_revisions.clear();
  stages.commitment_safety_event = CommitmentSafetyEvent::none;
  stages.candidate_sequence_number = 51;
  stages.revalidation_lease_sequence = 73;
  request.cycle_sequence = 43;
  request.random_seed = 4043;
  request.triggered_at = {3'000};
  const std::int64_t replay_start = now;
  const PlanningCycleResult replanned = loop.run_cycle(request);

  require(replanned.succeeded() && replanned.start &&
              replanned.start->source ==
                  PlanningStartSource::synchronized_actual_state &&
              replanned.start->robot_state.sequence_number == 20 &&
              replanned.ignored_revoked_commitment_lease_sequence == 72 &&
              std::find(stages.order.begin(), stages.order.end(),
                        PlanningCycleStage::commitment_validation) ==
                  stages.order.end(),
          "revoked commitment was reused instead of replanning from actual state");
  const AlgorithmExperimentRecord& record =
      loop.experiment_log().records().back();
  require(record.committed_lease_was_revoked,
          "replay record omitted the revoked commitment state");
  SuccessfulStages replay_stages;
  replay_stages.candidate_sequence_number = 51;
  replay_stages.revalidation_lease_sequence = 73;
  std::int64_t replay_now = replay_start;
  const AlgorithmExperimentReplayResult replay =
      AlgorithmExperimentReplayer::replay(record, replay_stages,
                                           [&replay_now] {
                                             replay_now += 10;
                                             return MonotonicTime{replay_now};
                                           });
  require(replay.status == AlgorithmExperimentReplayStatus::reproduced,
          "revoked commitment control state was not restored for replay");
}

void every_commitment_safety_event_has_a_traceable_transition() {
  struct Scenario {
    CommitmentSafetyEvent event;
    CommitmentSafetyAction action;
    PlanningState state;
    const char* reason_code;
  };
  const Scenario scenarios[] = {
      {CommitmentSafetyEvent::terrain_constraint_change,
       CommitmentSafetyAction::replan_urgent, PlanningState::normal_planning,
       "COMMITMENT_TERRAIN_CONSTRAINT_CHANGE"},
      {CommitmentSafetyEvent::localization_jump,
       CommitmentSafetyAction::stop, PlanningState::emergency_stop,
       "COMMITMENT_LOCALIZATION_JUMP"},
      {CommitmentSafetyEvent::cable_state_anomaly,
       CommitmentSafetyAction::stop, PlanningState::emergency_stop,
       "COMMITMENT_CABLE_STATE_ANOMALY"},
      {CommitmentSafetyEvent::execution_deviation,
       CommitmentSafetyAction::replan_urgent, PlanningState::normal_planning,
       "COMMITMENT_EXECUTION_DEVIATION"},
      {CommitmentSafetyEvent::emergency_stop,
       CommitmentSafetyAction::stop, PlanningState::emergency_stop,
       "COMMITMENT_EMERGENCY_STOP"},
  };

  std::uint64_t lease_sequence = 100;
  for (const Scenario& scenario : scenarios) {
    SuccessfulStages stages;
    stages.commitment_safety_event = scenario.event;
    AuthorizedPlanningResultPublisher publisher;
    ExecutionLeaseMonitor lease_monitor;
    std::int64_t now = 3'000;
    MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
      now += 10;
      return MonotonicTime{now};
    });
    PlanningCycleRequest request;
    request.cycle_sequence = lease_sequence;
    request.random_seed = 4040 + lease_sequence;
    request.triggered_at = {3'000};
    CommittedPlanningStart committed;
    committed.source_plan_sequence_number = 49;
    committed.lease_sequence = lease_sequence++;
    committed.dependencies = dependencies();
    committed.authorized_prefix = committed_prefix();
    committed.terminal_robot_state =
        {{0.25, 0.0, 0.0, {100}}, 0.5, 0.0, {100}, 90};
    request.committed_start = committed;

    const PlanningCycleResult result = loop.run_cycle(request);
    require(result.status == PlanningCycleStatus::commitment_overridden &&
                result.state == scenario.state &&
                result.controlled_stop_required &&
                result.urgent_replan_required ==
                    (scenario.action == CommitmentSafetyAction::replan_urgent) &&
                result.root_cause &&
                result.root_cause->reason_code == scenario.reason_code &&
                result.commitment_safety &&
                result.commitment_safety->event == scenario.event &&
                result.commitment_safety->action == scenario.action &&
                stages.safety_stop_requests == 1,
            "fault-injected safety event lacked a traceable transition");
  }
}

void complete_commitment_constraints_can_break_authorization() {
  struct Scenario {
    PathCandidateVerificationStatus robot_status;
    TimedCableValidationStatus cable_status;
    CommitmentSafetyEvent expected_event;
  };
  const Scenario scenarios[] = {
      {PathCandidateVerificationStatus::collision_violation,
       TimedCableValidationStatus::valid,
       CommitmentSafetyEvent::new_obstacle},
      {PathCandidateVerificationStatus::traversability_violation,
       TimedCableValidationStatus::valid,
       CommitmentSafetyEvent::terrain_constraint_change},
      {PathCandidateVerificationStatus::valid,
       TimedCableValidationStatus::corridor_violation,
       CommitmentSafetyEvent::cable_state_anomaly},
  };
  std::uint64_t sequence = 200;
  for (const Scenario& scenario : scenarios) {
    SuccessfulStages stages;
    stages.commitment_robot_status = scenario.robot_status;
    stages.commitment_cable_status = scenario.cable_status;
    if (scenario.robot_status ==
        PathCandidateVerificationStatus::collision_violation) {
      stages.commitment_obstacle_stopping =
          ObstacleStoppingEvidence{2.0, 1.0};
    }
    AuthorizedPlanningResultPublisher publisher;
    ExecutionLeaseMonitor lease_monitor;
    std::int64_t now = 4'000;
    MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
      now += 10;
      return MonotonicTime{now};
    });
    PlanningCycleRequest request;
    request.cycle_sequence = sequence;
    request.random_seed = sequence;
    request.triggered_at = {4'000};
    CommittedPlanningStart committed;
    committed.source_plan_sequence_number = 49;
    committed.lease_sequence = sequence++;
    committed.dependencies = dependencies();
    committed.authorized_prefix = committed_prefix();
    committed.terminal_robot_state =
        {{0.25, 0.0, 0.0, {100}}, 0.5, 0.0, {100}, 90};
    request.committed_start = committed;

    const PlanningCycleResult result = loop.run_cycle(request);
    require(result.commitment_safety &&
                result.commitment_safety->event == scenario.expected_event &&
                result.status == PlanningCycleStatus::commitment_overridden &&
                stages.order.back() ==
                    PlanningCycleStage::commitment_validation,
            "full robot/cable commitment constraint did not override safety");
  }
}

void dependency_change_breaks_commitment_before_analysis_or_search() {
  // Design: 18.2.7-10
  using Mutation = std::function<void(PlanningDependencyVersions&)>;
  const std::vector<Mutation> mutations = {
      [](PlanningDependencyVersions& value) {
        value.map_version.sequence_number = 6;
      },
      [](PlanningDependencyVersions& value) {
        value.reference_line_version = 3;
      },
      [](PlanningDependencyVersions& value) {
        value.robot_operating_area_version = 4;
      },
      [](PlanningDependencyVersions& value) {
        value.terrain_gradient_policy_version = 5;
      },
      [](PlanningDependencyVersions& value) {
        value.corridor_risk_policy_version = 6;
      },
      [](PlanningDependencyVersions& value) { value.cable_model_version = 7; },
      [](PlanningDependencyVersions& value) {
        value.uncertainty_envelope_version = 8;
      },
      [](PlanningDependencyVersions& value) {
        value.uncertainty_envelope_generator_version = 9;
      },
      [](PlanningDependencyVersions& value) {
        value.execution_operating_envelope_version = 10;
      },
      [](PlanningDependencyVersions& value) {
        value.execution_profile_version = 13;
      },
      [](PlanningDependencyVersions& value) {
        value.sensor_mode = SensorHealthMode::approved_degraded;
      },
      [](PlanningDependencyVersions& value) {
        value.operating_domain_id = "other-domain";
      },
  };

  std::uint64_t lease_sequence = 300;
  for (const Mutation& mutate : mutations) {
    SuccessfulStages stages;
    AuthorizedPlanningResultPublisher publisher;
    const PlanValidationLease current_lease =
        lease(13, lease_sequence, 10'000, 50);
    require(publish_candidate(publisher, current_lease).published(),
            "test setup did not publish the current plan");
    ExecutionLeaseMonitor lease_monitor;
    stages.revoked_probe = [&lease_monitor, lease_sequence] {
      return lease_monitor.isRevoked(lease_sequence);
    };
    std::int64_t now = 5'000;
    MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
      now += 10;
      return MonotonicTime{now};
    });

    PlanningCycleRequest request;
    request.cycle_sequence = lease_sequence;
    request.random_seed = 4'000 + lease_sequence;
    request.triggered_at = {5'000};
    CommittedPlanningStart committed;
    committed.source_plan_sequence_number = 50;
    committed.lease_sequence = lease_sequence;
    committed.dependencies = dependencies();
    mutate(committed.dependencies);
    committed.authorized_prefix = committed_prefix();
    committed.terminal_robot_state =
        {{0.25, 0.0, 0.0, {100}}, 0.5, 0.0, {100}, 90};
    request.committed_start = committed;

    const PlanningCycleResult result = loop.run_cycle(request);
    require(result.status == PlanningCycleStatus::commitment_overridden &&
                result.state == PlanningState::normal_planning &&
                result.urgent_replan_required &&
                result.controlled_stop_required &&
                result.commitment_safety &&
                result.commitment_safety->event ==
                    CommitmentSafetyEvent::dependency_version_change &&
                result.revoked_lease_sequence == lease_sequence &&
                stages.safety_stop_requests == 1 &&
                stages.lease_was_revoked_before_stop &&
                stages.order == std::vector<PlanningCycleStage>{
                                    PlanningCycleStage::capture_inputs},
            "dependency change did not override commitment before planning");
    ++lease_sequence;
  }
}

void invalid_lease_cannot_expose_a_plan_without_authorization() {
  AuthorizedPlanningResultPublisher publisher;
  PlanValidationLease mismatched = lease(12);
  const AuthorizedPlanningPublication result =
      publish_candidate(publisher, mismatched);
  require(result.status == AuthorizedPlanningPublishStatus::invalid_lease &&
              !result.value.has_value() && !publisher.current().has_value(),
          "plan became observable without its matching lease");
}

void stale_candidate_is_not_revalidated_leased_or_published() {
  SuccessfulStages stages(true);
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease()).published(),
          "test setup did not publish the old plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 3'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });

  PlanningCycleRequest request;
  request.cycle_sequence = 3;
  request.random_seed = 44;
  request.triggered_at = {3'000};
  const PlanningCycleResult result = loop.run_cycle(request);

  require(result.status == PlanningCycleStatus::candidate_invalidated &&
              !result.publication.has_value() &&
              !publisher.current().has_value(),
          "stale candidate crossed the lease or publication gate");
  require(stages.order.back() ==
                  PlanningCycleStage::decision_context_capture &&
              stages.order.size() == 10 &&
              result.replay_input_captures.size() == 3U,
          "stale candidate was processed after latest-context mismatch");
  SuccessfulStages replay_stages(true);
  std::int64_t replay_now = 3'000;
  const AlgorithmExperimentReplayResult replay =
      AlgorithmExperimentReplayer::replay(
          loop.experiment_log().records().front(), replay_stages,
          [&replay_now] {
            replay_now += 10;
            return MonotonicTime{replay_now};
          });
  require(replay.status == AlgorithmExperimentReplayStatus::reproduced,
          "three-capture fallback cycle was not reproduced offline");
}

void search_deadline_revalidates_old_plan_before_continuing() {
  SuccessfulStages stages;
  stages.search_state = PlanningState::timeout;
  stages.search_deadline_exceeded = true;
  stages.revalidation_lease_sequence = 71;
  AuthorizedPlanningResultPublisher publisher;
  const AuthorizedPlanningPublication initial =
      publish_candidate(publisher, lease());
  require(initial.published(), "test setup did not publish the old plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 4'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });

  PlanningCycleRequest request;
  request.cycle_sequence = 4;
  request.random_seed = 45;
  request.triggered_at = {4'000};
  const PlanningCycleResult result = loop.run_cycle(request);

  require(result.status == PlanningCycleStatus::current_plan_reused &&
              result.state == PlanningState::path_valid &&
              result.publication.has_value() &&
              result.publication->lease.lease_sequence == 71 &&
              result.publication->remaining_path &&
              result.publication->remaining_path->execution_profile.version ==
                  result.publication->lease.execution_profile_version &&
              !result.controlled_stop_required,
          "deadline did not continue only on a freshly leased old plan");
  require(result.root_cause.has_value() &&
              result.root_cause->cause ==
                  PlanningFailureCause::search_deadline_exceeded &&
              result.issues.size() == 1,
          "deadline failure did not expose exactly one root cause");
  require(lease_monitor.isRevoked(70),
          "the superseded lease remained executable after renewal");
}

void total_cycle_deadline_is_enforced_at_the_publication_boundary() {
  constexpr std::int64_t kDeadlineNs = 500'000'000;

  {
    SuccessfulStages stages;
    stages.revalidation_lease_expires_at_ns = 2'000'000'000;
    std::int64_t now = 600;
    stages.on_stage = [&now](const PlanningCycleStage stage) {
      if (stage == PlanningCycleStage::candidate_decision) {
        now = kDeadlineNs;
      }
    };
    AuthorizedPlanningResultPublisher publisher;
    ExecutionLeaseMonitor lease_monitor;
    MainPlanningLoop loop(stages, publisher, lease_monitor,
                          [&now] { return MonotonicTime{now}; });
    const PlanningCycleResult result =
        loop.run_cycle({1U, 101U, MonotonicTime{0}, std::nullopt});
    require(result.succeeded() && publisher.current().has_value(),
            "the exact 500 ms cycle boundary was treated as exceeded");
  }

  {
    SuccessfulStages stages;
    stages.revalidation_lease_expires_at_ns = 2'000'000'000;
    std::int64_t now = 600;
    stages.on_stage = [&now](const PlanningCycleStage stage) {
      if (stage == PlanningCycleStage::candidate_decision) {
        now = kDeadlineNs + 1;
      }
    };
    AuthorizedPlanningResultPublisher publisher;
    ExecutionLeaseMonitor lease_monitor;
    MainPlanningLoop loop(stages, publisher, lease_monitor,
                          [&now] { return MonotonicTime{now}; });
    const PlanningCycleResult result =
        loop.run_cycle({2U, 102U, MonotonicTime{0}, std::nullopt});
    require(result.status == PlanningCycleStatus::cycle_timeout &&
                result.state == PlanningState::timeout &&
                result.controlled_stop_required && !result.publication &&
                !publisher.current() && result.root_cause &&
                result.root_cause->cause ==
                    PlanningFailureCause::planning_cycle_deadline_exceeded,
            "a total-cycle timeout without an old plan did not stop safely");
  }

  {
    SuccessfulStages stages;
    stages.candidate_sequence_number = 51U;
    stages.revalidation_lease_sequence = 71U;
    stages.revalidation_lease_expires_at_ns = 2'000'000'000;
    std::int64_t now = 600;
    stages.on_stage = [&now](const PlanningCycleStage stage) {
      if (stage == PlanningCycleStage::candidate_decision) {
        now = kDeadlineNs + 1;
      }
    };
    AuthorizedPlanningResultPublisher publisher;
    require(publish_candidate(publisher,
                              lease(13U, 70U, 2'000'000'000, 50U))
                .published(),
            "test setup did not publish the old plan");
    ExecutionLeaseMonitor lease_monitor;
    MainPlanningLoop loop(stages, publisher, lease_monitor,
                          [&now] { return MonotonicTime{now}; });
    const PlanningCycleResult result =
        loop.run_cycle({3U, 103U, MonotonicTime{0}, std::nullopt});
    require(result.status == PlanningCycleStatus::cycle_timeout &&
                result.state == PlanningState::timeout && result.publication &&
                result.publication->plan.value().sequence_number == 50U &&
                result.publication->lease.lease_sequence == 71U &&
                result.decision_inputs &&
                result.decision_inputs->source_revision == 101U &&
                lease_monitor.isRevoked(70U) &&
                !result.controlled_stop_required,
            "a total-cycle timeout reused an old plan without fresh authorization");
  }

  {
    SuccessfulStages stages;
    stages.revalidation_lease_expires_at_ns = 2'000'000'000;
    AuthorizedPlanningResultPublisher publisher;
    ExecutionLeaseMonitor lease_monitor;
    MainPlanningLoop loop(stages, publisher, lease_monitor, [&publisher] {
      return MonotonicTime{publisher.current().has_value()
                               ? kDeadlineNs + 1
                               : 600};
    });
    const PlanningCycleResult result =
        loop.run_cycle({4U, 104U, MonotonicTime{0}, std::nullopt});
    require(result.status == PlanningCycleStatus::cycle_timeout &&
                result.state == PlanningState::timeout &&
                result.controlled_stop_required && !result.publication &&
                !publisher.current() && lease_monitor.isRevoked(70U) &&
                stages.controlled_stop_requests == 1U,
            "a publication that crossed the cycle deadline remained executable");
  }
}

void old_plan_reauthorization_reports_a_deadline_crossing() {
  SuccessfulStages stages;
  stages.search_state = PlanningState::timeout;
  stages.search_deadline_exceeded = true;
  stages.revalidation_lease_sequence = 71U;
  stages.revalidation_lease_expires_at_ns = 2'000'000'000;
  std::int64_t now = 600;
  stages.on_stage = [&now](const PlanningCycleStage stage) {
    if (stage == PlanningCycleStage::current_plan_revalidation) {
      now = 500'000'001;
    }
  };
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher,
                            lease(13U, 70U, 2'000'000'000, 50U))
              .published(),
          "test setup did not publish the old plan");
  ExecutionLeaseMonitor lease_monitor;
  MainPlanningLoop loop(stages, publisher, lease_monitor,
                        [&now] { return MonotonicTime{now}; });
  const PlanningCycleResult result =
      loop.run_cycle({5U, 105U, MonotonicTime{0}, std::nullopt});
  require(result.status == PlanningCycleStatus::cycle_timeout &&
              result.state == PlanningState::timeout && result.publication &&
              result.publication->lease.lease_sequence == 71U &&
              lease_monitor.isRevoked(70U) &&
              !result.controlled_stop_required && result.root_cause &&
              result.root_cause->cause ==
                  PlanningFailureCause::planning_cycle_deadline_exceeded,
          "old-plan reauthorization hid a total-cycle deadline crossing");
}

void ambiguous_timeout_diagnostics_fail_as_invalid_input() {
  SuccessfulStages stages;
  stages.search_state = PlanningState::timeout;
  stages.search_deadline_exceeded = true;
  stages.search_label_budget_exhausted = true;
  stages.revalidation_lease_sequence = 71;
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease()).published(),
          "test setup did not publish the old plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 5'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });

  PlanningCycleRequest request;
  request.cycle_sequence = 5;
  request.random_seed = 46;
  request.triggered_at = {5'000};
  const PlanningCycleResult result = loop.run_cycle(request);

  require(result.status == PlanningCycleStatus::search_failed &&
              result.state == PlanningState::input_invalid &&
              result.controlled_stop_required && !result.publication &&
              result.root_cause &&
              result.root_cause->cause == PlanningFailureCause::input_invalid &&
              result.issues.size() == 1 && lease_monitor.isRevoked(70),
          "contradictory timeout evidence was assigned an unsafe root cause");
}

void timeout_revalidation_failure_cannot_reuse_the_old_plan() {
  // Design: 18.2.7-4
  struct Scenario {
    PlanValidationStatus status;
    PlanValidationAction action;
  };
  const Scenario scenarios[] = {
      {PlanValidationStatus::execution_profile_mismatch,
       PlanValidationAction::stop},
      {PlanValidationStatus::state_mismatch, PlanValidationAction::stop},
      {PlanValidationStatus::cable_model_invalid,
       PlanValidationAction::replan},
  };
  for (const Scenario& scenario : scenarios) {
    SuccessfulStages stages;
    stages.search_state = PlanningState::timeout;
    stages.search_deadline_exceeded = true;
    stages.revalidation_valid = false;
    stages.revalidation_status = scenario.status;
    stages.revalidation_action = scenario.action;
    AuthorizedPlanningResultPublisher publisher;
    require(publish_candidate(publisher, lease()).published(),
            "test setup did not publish the old plan");
    ExecutionLeaseMonitor lease_monitor;
    std::int64_t now = 6'000;
    MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
      now += 10;
      return MonotonicTime{now};
    });

    PlanningCycleRequest request;
    request.cycle_sequence = 6;
    request.random_seed = 47;
    request.triggered_at = {6'000};
    const PlanningCycleResult result = loop.run_cycle(request);

    require(result.status == PlanningCycleStatus::search_failed &&
                result.state == PlanningState::timeout &&
                result.controlled_stop_required && !result.publication &&
                result.root_cause && result.issues.size() == 1 &&
                lease_monitor.isRevoked(70) && !publisher.current(),
            "a failed timeout revalidation survived the old-plan reuse gate");
    require_followup_command_rejected(lease_monitor, candidate(), lease());
  }
}

void every_t46_safety_failure_blocks_followup_commands() {
  // Design: T46-safety-invariant
  struct Scenario {
    bool stale_decision_context{};
    std::function<void(SuccessfulStages&)> inject;
  };
  const Scenario scenarios[] = {
      {false,
       [](SuccessfulStages& stages) {
         stages.initial_capture_status =
             ValidationInputCaptureStatus::validation_context_invalid;
       }},
      {false,
       [](SuccessfulStages& stages) {
         stages.smoothing_status = SmoothingStatus::boundary_state_invalid;
         stages.raw_trackability_valid = false;
       }},
      {false,
       [](SuccessfulStages& stages) {
         stages.parameterization_status =
             ParameterizationStatus::execution_envelope_mismatch;
       }},
      {false,
       [](SuccessfulStages& stages) {
         stages.complete_robot_path_status =
             PathCandidateVerificationStatus::input_invalid;
       }},
      {false,
       [](SuccessfulStages& stages) {
         stages.cable_validation_status =
             TimedCableValidationStatus::covariance_envelope_breach;
         stages.cable_stop_required = true;
       }},
      {true, [](SuccessfulStages&) {}},
      {false,
       [](SuccessfulStages& stages) {
         stages.search_state = PlanningState::timeout;
         stages.search_deadline_exceeded = true;
         stages.current_plan_revalidation_status =
             PlanValidationStatus::cable_model_invalid;
         stages.current_plan_revalidation_action = PlanValidationAction::replan;
       }},
  };

  std::uint64_t cycle_sequence = 60;
  for (const Scenario& scenario : scenarios) {
    SuccessfulStages stages(scenario.stale_decision_context);
    stages.current_plan_revalidation_valid = false;
    stages.current_plan_revalidation_status =
        PlanValidationStatus::cable_model_invalid;
    stages.current_plan_revalidation_action = PlanValidationAction::replan;
    scenario.inject(stages);

    PlanningResult old_plan = candidate();
    old_plan.sequence_number = 49;
    const PlanValidationLease old_lease = lease(13, 70, 20'000, 49);
    AuthorizedPlanningResultPublisher publisher;
    require(publisher
                .publish(old_plan,
                         std::make_shared<const TimedPath>(
                             old_plan.robot_trajectory),
                         old_lease, 100.0)
                .published(),
            "test setup did not publish the old authorization");
    ExecutionLeaseMonitor lease_monitor;
    std::int64_t now = 2'000;
    MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
      now += 10;
      return MonotonicTime{now};
    });
    const PlanningCycleResult result = loop.run_cycle(
        {cycle_sequence++, 4'600, MonotonicTime{2'000}, std::nullopt});
    require(result.controlled_stop_required && !result.publication &&
                !publisher.current() && lease_monitor.isRevoked(70),
            "a T46 planning safety failure left the old lease executable");
    require_followup_command_rejected(lease_monitor, old_plan, old_lease);
  }

  {
    PlanningResult old_plan = candidate();
    old_plan.sequence_number = 49;
    PlanValidationLease old_lease = lease(13, 70, 20'000, 49);
    old_lease.max_ground_speed_tracking_error_mps = 0.1;
    old_lease.max_payout_speed_tracking_error_mps = 0.1;
    old_lease.allowed_ground_acceleration = {-1.0, 1.0};
    old_lease.allowed_tension = {0.0, 100.0};
    ExecutionFeedback deviation = execution_feedback(old_plan);
    deviation.ground_speed_mps = 0.9;
    ExecutionLeaseMonitor lease_monitor;
    const ExecutionAuthorization rejected = lease_monitor.evaluate(
        old_plan, old_plan.robot_trajectory, old_lease, execution_context(),
        deviation, MonotonicTime{1'000});
    require(rejected.revoked() && lease_monitor.isRevoked(70),
            "execution tracking failure did not revoke its lease");
    require_followup_command_rejected(lease_monitor, old_plan, old_lease);
  }

  {
    SuccessfulStages stages;
    stages.commitment_safety_event =
        CommitmentSafetyEvent::execution_deviation;
    PlanningResult old_plan = candidate();
    old_plan.sequence_number = 49;
    const PlanValidationLease old_lease = lease(13, 70, 20'000, 49);
    AuthorizedPlanningResultPublisher publisher;
    require(publisher
                .publish(old_plan,
                         std::make_shared<const TimedPath>(
                             old_plan.robot_trajectory),
                         old_lease, 100.0)
                .published(),
            "test setup did not publish the commitment authorization");
    ExecutionLeaseMonitor lease_monitor;
    std::int64_t now = 3'000;
    MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
      now += 10;
      return MonotonicTime{now};
    });
    CommittedPlanningStart committed;
    committed.source_plan_sequence_number = 49;
    committed.lease_sequence = 70;
    committed.dependencies = dependencies();
    committed.authorized_prefix = committed_prefix();
    committed.terminal_robot_state =
        {{0.25, 0.0, 0.0, {100}}, 0.5, 0.0, {100}, 90};
    const PlanningCycleResult result = loop.run_cycle(
        {cycle_sequence, 4'601, MonotonicTime{3'000}, committed});
    require(result.controlled_stop_required && result.urgent_replan_required &&
                !publisher.current() && lease_monitor.isRevoked(70),
            "urgent commitment failure left its lease executable");
    require_followup_command_rejected(lease_monitor, old_plan, old_lease);
  }
}

void trackable_raw_search_path_is_the_only_smoothing_failure_fallback() {
  SuccessfulStages stages;
  stages.smoothing_status = SmoothingStatus::solver_timeout;
  AuthorizedPlanningResultPublisher publisher;
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 7'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });

  PlanningCycleRequest request;
  request.cycle_sequence = 7;
  request.random_seed = 48;
  request.triggered_at = {7'000};
  const PlanningCycleResult result = loop.run_cycle(request);

  require(result.succeeded() && result.used_raw_search_path &&
              result.root_cause &&
              result.root_cause->cause ==
                  PlanningFailureCause::smoothing_deadline_exceeded &&
              result.publication->plan.value()
                      .robot_trajectory.geometry.points.front().x_m == 0.0 &&
              result.publication->plan.value()
                      .robot_trajectory.geometry.points.back().x_m == 1.0,
          "a trackable raw path was not carried through the full success chain");
}

void candidate_covariance_breach_cannot_fall_back_to_a_valid_old_plan() {
  SuccessfulStages stages;
  stages.candidate_sequence_number = 51;
  stages.revalidation_valid = false;
  stages.revalidation_status =
      PlanValidationStatus::covariance_envelope_breach;
  stages.revalidation_action = PlanValidationAction::stop;
  stages.revalidation_lease_sequence = 71;
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease()).published(),
          "test setup did not publish the old plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 18'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });
  PlanningCycleRequest request;
  request.cycle_sequence = 22;
  request.random_seed = 59;
  request.triggered_at = {18'000};
  const PlanningCycleResult result = loop.run_cycle(request);

  require(result.status ==
              PlanningCycleStatus::covariance_envelope_breached &&
              result.state == PlanningState::covariance_envelope_breach &&
              result.controlled_stop_required && !result.publication &&
              result.root_cause &&
              result.root_cause->cause ==
                  PlanningFailureCause::covariance_envelope_breach &&
              lease_monitor.isRevoked(70) && !publisher.current(),
          "candidate covariance breach was downgraded to old-plan reuse");
}

void invalid_complete_robot_path_input_cannot_reuse_old_plan() {
  SuccessfulStages stages;
  stages.complete_robot_path_status =
      PathCandidateVerificationStatus::input_invalid;
  stages.revalidation_lease_sequence = 71;
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease()).published(),
          "test setup did not publish the old plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 19'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });
  PlanningCycleRequest request;
  request.cycle_sequence = 23;
  request.random_seed = 60;
  request.triggered_at = {19'000};
  const PlanningCycleResult result = loop.run_cycle(request);

  require(result.status ==
              PlanningCycleStatus::robot_path_validation_failed &&
              result.state == PlanningState::input_invalid &&
              result.controlled_stop_required && !result.publication &&
              result.root_cause &&
              result.root_cause->cause == PlanningFailureCause::input_invalid &&
              lease_monitor.isRevoked(70) && !publisher.current(),
          "invalid complete-path inputs were treated as reusable no-solution");
}

void replacement_plan_requires_newer_lease_and_revokes_the_old_one() {
  SuccessfulStages stages;
  stages.candidate_sequence_number = 51;
  stages.revalidation_lease_sequence = 71;
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease()).published(),
          "test setup did not publish the old plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 20'000;
  stages.revalidation_lease_expires_at_ns = 30'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });
  PlanningCycleRequest request;
  request.cycle_sequence = 24;
  request.random_seed = 61;
  request.triggered_at = {20'000};
  const PlanningCycleResult result = loop.run_cycle(request);

  require(result.succeeded() && result.publication &&
              result.publication->plan.value().sequence_number == 51 &&
              result.publication->lease.lease_sequence == 71 &&
              result.revoked_lease_sequence == 70 &&
              lease_monitor.isRevoked(70),
          "new plan did not replace and explicitly revoke the old lease");

  PlanningResult later = candidate();
  later.sequence_number = 52;
  PlanValidationLease regressed = lease(13, 70, 30'000, 52);
  const auto rejected = publisher.publish(
      later, std::make_shared<const TimedPath>(later.robot_trajectory),
      regressed, 80.0);
  require(!rejected.published() &&
              publisher.current()->lease.lease_sequence == 71,
          "same or older lease sequence replaced a newer authorization");
}

void slight_improvement_keeps_current_with_latest_context_lease() {
  // Design: 18.2.7-1
  SuccessfulStages stages;
  stages.candidate_sequence_number = 51;
  stages.candidate_cost = 95.0;
  stages.revalidation_lease_sequence = 71;
  stages.revalidation_lease_expires_at_ns = 30'000;
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease(13, 70, 21'000)).published(),
          "test setup did not publish the current plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 20'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });

  const PlanningCycleResult result = loop.run_cycle(
      {25, 62, MonotonicTime{20'000}, std::nullopt});

  require(result.status == PlanningCycleStatus::current_plan_reused &&
              result.state == PlanningState::path_valid &&
              result.publication &&
              result.publication->plan.value().sequence_number == 50 &&
              result.publication->lease.lease_sequence == 71 &&
              result.artifacts.current_plan_revalidation &&
              result.artifacts.candidate_decision &&
              result.artifacts.candidate_decision->should_keep_current() &&
              stages.decision_saw_current &&
              stages.decision_source_revision == 101 &&
              result.revoked_lease_sequence == 70 &&
              lease_monitor.isRevoked(70),
          "a slight improvement bypassed paired revalidation or hysteresis");
}

void configured_hysteresis_reaches_the_nonvirtual_decision_seam() {
  // Design: T65-config-invariant
  PathHysteresisConfig hysteresis;
  hysteresis.relative_cost_threshold = 0.25;
  SuccessfulStages stages(false, hysteresis);
  stages.candidate_sequence_number = 51;
  stages.candidate_cost = 80.0;
  stages.revalidation_lease_sequence = 71;
  stages.revalidation_lease_expires_at_ns = 30'000;
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease(13, 70, 21'000)).published(),
          "test setup did not publish the current plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 20'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });

  const PlanningCycleResult result = loop.run_cycle(
      {32, 69, MonotonicTime{20'000}, std::nullopt});

  require(result.status == PlanningCycleStatus::current_plan_reused &&
              result.artifacts.candidate_decision &&
              result.artifacts.candidate_decision->reason ==
                  "cost_improvement_within_hysteresis",
          "the nonvirtual decision seam ignored its configured hysteresis");
}

void significant_improvement_switches_after_paired_revalidation() {
  // Design: 18.2.7-2
  SuccessfulStages stages;
  stages.candidate_sequence_number = 51;
  stages.candidate_cost = 80.0;
  stages.revalidation_lease_sequence = 71;
  stages.revalidation_lease_expires_at_ns = 30'000;
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease(13, 70, 21'000)).published(),
          "test setup did not publish the current plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 20'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });

  const PlanningCycleResult result = loop.run_cycle(
      {26, 63, MonotonicTime{20'000}, std::nullopt});

  require(result.succeeded() && result.publication &&
              result.publication->plan.value().sequence_number == 51 &&
              result.artifacts.current_plan_revalidation &&
              result.artifacts.candidate_decision &&
              result.artifacts.candidate_decision->should_switch() &&
              stages.decision_saw_current &&
              stages.decision_source_revision == 101,
          "a significant improvement did not switch from paired validations");
}

void expired_during_candidate_planning_is_paired_revalidated() {
  // Design: 18.2.7-9
  SuccessfulStages stages;
  stages.candidate_sequence_number = 51;
  stages.candidate_cost = 95.0;
  stages.revalidation_lease_sequence = 71;
  stages.revalidation_lease_expires_at_ns = 30'000;
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease(13, 70, 20'030)).published(),
          "test setup did not publish the expiring current plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 20'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });

  const PlanningCycleResult result = loop.run_cycle(
      {34, 67, MonotonicTime{20'000}, std::nullopt});

  require(result.status == PlanningCycleStatus::current_plan_reused &&
              result.publication &&
              result.publication->lease.lease_sequence == 71 &&
              result.artifacts.candidate_revalidation &&
              result.artifacts.current_plan_revalidation &&
              result.artifacts.candidate_revalidation->lease &&
              result.artifacts.current_plan_revalidation->lease &&
              result.artifacts.candidate_revalidation->lease->validated_at
                      .nanoseconds ==
                  result.artifacts.current_plan_revalidation->lease
                      ->validated_at.nanoseconds &&
              stages.decision_saw_current && lease_monitor.isRevoked(70),
          "an expired planning lease bypassed paired latest-context validation");
}

void mismatched_validation_context_stops_before_hysteresis() {
  // Design: 18.2.7-20
  SuccessfulStages stages;
  stages.candidate_sequence_number = 51;
  stages.candidate_cost = 80.0;
  stages.current_plan_map_sequence_override = 8;
  stages.revalidation_lease_sequence = 71;
  stages.revalidation_lease_expires_at_ns = 30'000;
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease(13, 70, 21'000)).published(),
          "test setup did not publish the current plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 20'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });

  const PlanningCycleResult result = loop.run_cycle(
      {35, 68, MonotonicTime{20'000}, std::nullopt});

  require(result.status == PlanningCycleStatus::decision_rejected &&
              result.controlled_stop_required && !result.publication &&
              result.artifacts.candidate_decision &&
              result.artifacts.candidate_decision->reason ==
                  "validation_context_mismatch" &&
              lease_monitor.isRevoked(70) && !publisher.current(),
          "mixed validation contexts reached hysteresis or publication");
}

void invalid_current_plan_allows_only_a_finite_candidate_direct_switch() {
  // Design: 18.2.7-6
  SuccessfulStages stages;
  stages.candidate_sequence_number = 51;
  stages.candidate_cost = 95.0;
  stages.current_plan_revalidation_valid = false;
  stages.current_plan_revalidation_status =
      PlanValidationStatus::cable_model_invalid;
  stages.current_plan_revalidation_action = PlanValidationAction::replan;
  stages.revalidation_lease_sequence = 71;
  stages.revalidation_lease_expires_at_ns = 30'000;
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease(13, 70, 21'000)).published(),
          "test setup did not publish the current plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 20'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });

  const PlanningCycleResult result = loop.run_cycle(
      {27, 64, MonotonicTime{20'000}, std::nullopt});

  require(result.succeeded() && result.publication &&
              result.publication->plan.value().sequence_number == 51 &&
              result.artifacts.candidate_decision &&
              result.artifacts.candidate_decision->reason ==
                  "current_plan_not_valid",
          "an invalid current plan blocked a finite validated candidate");
}

void invalid_candidate_is_paired_with_current_before_the_keep_decision() {
  // Design: 18.2.7-22
  SuccessfulStages stages;
  stages.candidate_sequence_number = 51;
  stages.revalidation_valid = false;
  stages.revalidation_status = PlanValidationStatus::cable_model_invalid;
  stages.revalidation_action = PlanValidationAction::replan;
  stages.revalidation_lease_sequence = 71;
  stages.revalidation_lease_expires_at_ns = 30'000;
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease(13, 70, 21'000)).published(),
          "test setup did not publish the current plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 20'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });

  const PlanningCycleResult result = loop.run_cycle(
      {30, 67, MonotonicTime{20'000}, std::nullopt});

  require(result.status == PlanningCycleStatus::current_plan_reused &&
              result.publication &&
              result.publication->plan.value().sequence_number == 50 &&
              result.artifacts.candidate_revalidation &&
              result.artifacts.current_plan_revalidation &&
              result.artifacts.candidate_decision &&
              result.artifacts.candidate_decision->reason ==
                  "candidate_validation_failed" &&
              result.replay_input_captures.size() == 2U,
          "candidate failure bypassed paired latest-context hysteresis");

  SuccessfulStages no_current_stages;
  no_current_stages.candidate_sequence_number = 51;
  no_current_stages.revalidation_valid = false;
  no_current_stages.revalidation_status =
      PlanValidationStatus::cable_model_invalid;
  no_current_stages.revalidation_action = PlanValidationAction::replan;
  no_current_stages.revalidation_lease_expires_at_ns = 30'000;
  AuthorizedPlanningResultPublisher empty_publisher;
  ExecutionLeaseMonitor empty_lease_monitor;
  now = 20'000;
  MainPlanningLoop empty_loop(no_current_stages, empty_publisher,
                              empty_lease_monitor, [&now] {
                                now += 10;
                                return MonotonicTime{now};
                              });
  const PlanningCycleResult stopped = empty_loop.run_cycle(
      {31, 68, MonotonicTime{20'000}, std::nullopt});
  if (!(stopped.status == PlanningCycleStatus::candidate_invalidated &&
        stopped.controlled_stop_required && !stopped.publication &&
        stopped.artifacts.candidate_revalidation &&
        !stopped.artifacts.current_plan_revalidation &&
        stopped.artifacts.candidate_decision &&
        stopped.artifacts.candidate_decision->action ==
            PathSwitchAction::stop &&
        stopped.artifacts.candidate_decision->reason ==
            "candidate_and_current_validation_failed" &&
        stopped.replay_input_captures.size() == 2U)) {
    throw std::runtime_error(
        "candidate failure without a current fallback bypassed paired stop: status=" +
        std::to_string(static_cast<int>(stopped.status)) + " reason=" +
        (stopped.artifacts.candidate_decision
             ? stopped.artifacts.candidate_decision->reason
             : std::string{"none"}) +
        " captures=" + std::to_string(stopped.replay_input_captures.size()));
  }
}

void invalid_candidate_cost_keeps_valid_current_or_stops() {
  // Design: 18.2.7-19
  const double invalid_costs[] = {
      -1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()};
  std::uint64_t cycle_sequence = 28;
  for (const double invalid_cost : invalid_costs) {
    SuccessfulStages stages;
    stages.candidate_sequence_number = 51;
    stages.candidate_cost = invalid_cost;
    stages.revalidation_lease_sequence = 71;
    stages.revalidation_lease_expires_at_ns = 30'000;
    AuthorizedPlanningResultPublisher publisher;
    require(publish_candidate(publisher, lease(13, 70, 21'000)).published(),
            "test setup did not publish the current plan");
    ExecutionLeaseMonitor lease_monitor;
    std::int64_t now = 20'000;
    MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
      now += 10;
      return MonotonicTime{now};
    });

    const PlanningCycleResult kept = loop.run_cycle(
        {cycle_sequence++, 65, MonotonicTime{20'000}, std::nullopt});
    require(kept.status == PlanningCycleStatus::current_plan_reused &&
                kept.publication &&
                kept.publication->plan.value().sequence_number == 50 &&
                kept.artifacts.candidate_decision &&
                kept.artifacts.candidate_decision->reason ==
                    "invalid_path_cost",
            "an invalid candidate cost displaced a valid current plan");

    SuccessfulStages no_current_stages;
    no_current_stages.candidate_cost = invalid_cost;
    no_current_stages.revalidation_lease_expires_at_ns = 30'000;
    AuthorizedPlanningResultPublisher empty_publisher;
    ExecutionLeaseMonitor empty_lease_monitor;
    now = 20'000;
    MainPlanningLoop empty_loop(no_current_stages, empty_publisher,
                                empty_lease_monitor, [&now] {
                                  now += 10;
                                  return MonotonicTime{now};
                                });
    const PlanningCycleResult stopped = empty_loop.run_cycle(
        {cycle_sequence++, 66, MonotonicTime{20'000}, std::nullopt});
    require(stopped.status == PlanningCycleStatus::decision_rejected &&
                stopped.controlled_stop_required && !stopped.publication &&
                stopped.artifacts.candidate_decision &&
                stopped.artifacts.candidate_decision->action ==
                    PathSwitchAction::stop &&
                stopped.artifacts.candidate_decision->reason ==
                    "invalid_candidate_cost",
            "an invalid candidate cost published without a current fallback");
  }
}

void invalid_cost_cannot_cross_atomic_authorization_boundary() {
  // Design: 18.2.7-19
  // Design: T65-cost-invariant
  const double invalid_costs[] = {
      -1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()};
  for (const double invalid_cost : invalid_costs) {
    AuthorizedPlanningResultPublisher publisher;
    const PlanningResult plan = candidate();
    const AuthorizedPlanningPublication rejected = publisher.publish(
        plan, std::make_shared<const TimedPath>(plan.robot_trajectory), lease(),
        invalid_cost);
    require(!rejected.published() && !publisher.current() &&
                !rejected.issues.empty(),
            "an invalid path cost crossed the atomic authorization boundary");
  }
}

void edited_remaining_path_cannot_cross_atomic_authorization_boundary() {
  AuthorizedPlanningResultPublisher publisher;
  PlanningResult plan = candidate();
  TimedPath edited = plan.robot_trajectory;
  edited.geometry.points.front().x_m = 0.1;
  const auto publication = publisher.publish(
      plan, std::make_shared<const TimedPath>(edited), lease(), 100.0);
  require(!publication.published() && !publisher.current(),
          "edited remaining geometry was accepted as a crop of the plan");
}

void revoked_publisher_history_cannot_accept_an_older_lease() {
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease()).published(),
          "test setup did not publish the initial lease");
  publisher.revoke_current();
  PlanningResult later = candidate();
  later.sequence_number = 51;
  const auto regressed = lease(13, 69, 30'000, 51);
  const auto publication = publisher.publish(
      later, std::make_shared<const TimedPath>(later.robot_trajectory),
      regressed, 80.0);
  require(!publication.published() && !publisher.current(),
          "revocation erased the lease sequence high-water mark");
}

void untrackable_raw_path_preserves_the_smoothing_timeout_root_cause() {
  // Design: 18.2.5-6
  SuccessfulStages stages;
  stages.smoothing_status = SmoothingStatus::solver_timeout;
  stages.raw_trackability_valid = false;
  AuthorizedPlanningResultPublisher publisher;
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 8'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });

  PlanningCycleRequest request;
  request.cycle_sequence = 8;
  request.random_seed = 49;
  request.triggered_at = {8'000};
  const PlanningCycleResult result = loop.run_cycle(request);

  require(result.status == PlanningCycleStatus::smoothing_failed &&
              result.state == PlanningState::timeout &&
              result.controlled_stop_required && !result.publication &&
              result.root_cause &&
              result.root_cause->cause ==
                  PlanningFailureCause::smoothing_deadline_exceeded &&
              result.issues.size() == 1,
          "untrackable raw path hid the smoothing timeout root cause");
}

void trackable_raw_path_preserves_non_timeout_smoothing_root_cause() {
  SuccessfulStages stages;
  stages.smoothing_status = SmoothingStatus::solver_failed;
  AuthorizedPlanningResultPublisher publisher;
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 8'500;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });
  PlanningCycleRequest request;
  request.cycle_sequence = 25;
  request.random_seed = 62;
  request.triggered_at = {8'500};
  const PlanningCycleResult result = loop.run_cycle(request);

  require(result.succeeded() && result.used_raw_search_path &&
              result.root_cause &&
              result.root_cause->cause ==
                  PlanningFailureCause::smoothing_infeasible &&
              result.issues.size() == 1,
          "raw fallback hid a non-timeout smoothing failure root cause");
}

void parameterization_infeasibility_revalidates_old_plan_or_stops() {
  SuccessfulStages stages;
  stages.parameterization_status = ParameterizationStatus::dynamics_infeasible;
  stages.revalidation_lease_sequence = 71;
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease()).published(),
          "test setup did not publish the old plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 9'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });

  PlanningCycleRequest request;
  request.cycle_sequence = 9;
  request.random_seed = 50;
  request.triggered_at = {9'000};
  const PlanningCycleResult result = loop.run_cycle(request);

  require(result.status == PlanningCycleStatus::current_plan_reused &&
              result.state == PlanningState::path_valid && result.publication &&
              result.publication->lease.lease_sequence == 71 &&
              result.root_cause &&
              result.root_cause->cause ==
                  PlanningFailureCause::parameterization_infeasible &&
              lease_monitor.isRevoked(70),
          "parameterization infeasibility bypassed old-plan revalidation");
}

void invalid_cycle_request_revokes_authorization_and_stops() {
  SuccessfulStages stages;
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease()).published(),
          "test setup did not publish the old plan");
  ExecutionLeaseMonitor lease_monitor;
  stages.revoked_probe = [&lease_monitor] {
    return lease_monitor.isRevoked(70U);
  };
  std::int64_t now = 10'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });

  PlanningCycleRequest request;
  request.cycle_sequence = 0;
  request.random_seed = 51;
  request.triggered_at = {10'000};
  const PlanningCycleResult result = loop.run_cycle(request);

  require(result.status == PlanningCycleStatus::input_invalid &&
              result.state == PlanningState::input_invalid &&
              result.controlled_stop_required && result.root_cause &&
              result.root_cause->cause == PlanningFailureCause::input_invalid &&
              result.issues.size() == 1 && lease_monitor.isRevoked(70) &&
              !publisher.current() && stages.controlled_stop_requests == 1U &&
              stages.controlled_stop_reason ==
                  "PLANNING_CYCLE_REQUEST_INVALID" &&
              stages.lease_was_revoked_before_controlled_stop,
          "invalid cycle input left an executable authorization behind");
}

void search_failures_are_classified_without_collapsing_root_causes() {
  struct Case {
    PlanningState state;
    bool deadline;
    bool label_budget;
    PlanningFailureCause expected_cause;
    bool expected_reuse;
  };
  const std::vector<Case> cases = {
      {PlanningState::timeout, false, true,
       PlanningFailureCause::search_label_budget_exhausted, true},
      {PlanningState::timeout, false, false,
       PlanningFailureCause::search_budget_exhausted, true},
      {PlanningState::no_solution_under_covariance_envelope, false, false,
       PlanningFailureCause::no_solution_under_covariance_envelope, false},
      {PlanningState::no_solution, false, false,
       PlanningFailureCause::no_solution, true},
      {PlanningState::input_invalid, false, false,
       PlanningFailureCause::input_invalid, false},
  };

  std::uint64_t cycle_sequence = 11;
  for (const Case& value : cases) {
    SuccessfulStages stages;
    stages.search_state = value.state;
    stages.search_deadline_exceeded = value.deadline;
    stages.search_label_budget_exhausted = value.label_budget;
    stages.revalidation_lease_sequence = 71;
    stages.revalidation_lease_expires_at_ns = 20'000;
    AuthorizedPlanningResultPublisher publisher;
    require(publish_candidate(publisher, lease(13, 70, 12'000)).published(),
            "test setup did not publish the old plan");
    ExecutionLeaseMonitor lease_monitor;
    std::int64_t now = 11'000;
    MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
      now += 10;
      return MonotonicTime{now};
    });
    PlanningCycleRequest request;
    request.cycle_sequence = cycle_sequence++;
    request.random_seed = 52;
    request.triggered_at = {11'000};
    const PlanningCycleResult result = loop.run_cycle(request);

    require(result.root_cause &&
                result.root_cause->cause == value.expected_cause &&
                result.issues.size() == 1 &&
                result.controlled_stop_required != value.expected_reuse &&
                result.publication.has_value() == value.expected_reuse &&
                lease_monitor.isRevoked(70),
            "search failure categories collapsed or bypassed their safe action");
  }
}

void expired_during_planning_lease_is_replaced_not_reused() {
  // Design: 18.2.7-15
  SuccessfulStages stages;
  stages.search_state = PlanningState::timeout;
  stages.search_deadline_exceeded = true;
  stages.revalidation_lease_sequence = 71;
  stages.revalidation_lease_expires_at_ns = 20'000;
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease(13, 70, 12'030)).published(),
          "test setup did not publish the expiring old plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 12'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });
  PlanningCycleRequest request;
  request.cycle_sequence = 16;
  request.random_seed = 53;
  request.triggered_at = {12'000};
  const PlanningCycleResult result = loop.run_cycle(request);

  require(result.status == PlanningCycleStatus::current_plan_reused &&
              result.publication &&
              result.publication->lease.lease_sequence == 71 &&
              result.publication->lease.validated_at.nanoseconds == 600 &&
              lease_monitor.isRevoked(70),
          "an expired old lease was extended instead of replaced");
}

void old_lease_sequence_cannot_masquerade_as_a_renewal() {
  SuccessfulStages stages;
  stages.search_state = PlanningState::timeout;
  stages.search_deadline_exceeded = true;
  stages.revalidation_lease_sequence = 70;
  stages.revalidation_lease_expires_at_ns = 20'000;
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease(13, 70, 15'000)).published(),
          "test setup did not publish the old plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 13'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });
  PlanningCycleRequest request;
  request.cycle_sequence = 17;
  request.random_seed = 54;
  request.triggered_at = {13'000};
  const PlanningCycleResult result = loop.run_cycle(request);

  require(result.status == PlanningCycleStatus::search_failed &&
              result.controlled_stop_required && !result.publication &&
              lease_monitor.isRevoked(70) && !publisher.current(),
          "the old lease sequence was silently reused as a renewal");
}

void invalid_parameterization_input_never_falls_back_to_old_execution() {
  SuccessfulStages stages;
  stages.parameterization_status =
      ParameterizationStatus::execution_envelope_mismatch;
  stages.revalidation_lease_sequence = 71;
  stages.revalidation_lease_expires_at_ns = 20'000;
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease(13, 70, 15'000)).published(),
          "test setup did not publish the old plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 14'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });
  PlanningCycleRequest request;
  request.cycle_sequence = 18;
  request.random_seed = 55;
  request.triggered_at = {14'000};
  const PlanningCycleResult result = loop.run_cycle(request);

  require(result.status == PlanningCycleStatus::parameterization_failed &&
              result.state == PlanningState::input_invalid &&
              result.controlled_stop_required && result.root_cause &&
              result.root_cause->cause == PlanningFailureCause::input_invalid &&
              !result.publication && lease_monitor.isRevoked(70),
          "invalid parameterization input reused an unrelated old profile");
}

void same_revision_with_changed_dependencies_cannot_renew_old_plan() {
  SuccessfulStages stages(true);
  stages.revalidation_lease_sequence = 71;
  stages.revalidation_lease_expires_at_ns = 20'000;
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease(13, 70, 16'000)).published(),
          "test setup did not publish the old plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 15'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });
  PlanningCycleRequest request;
  request.cycle_sequence = 19;
  request.random_seed = 56;
  request.triggered_at = {15'000};
  const PlanningCycleResult result = loop.run_cycle(request);

  require(result.status == PlanningCycleStatus::candidate_invalidated &&
              result.controlled_stop_required && !result.publication &&
              lease_monitor.isRevoked(70) && !publisher.current(),
          "one source revision represented two dependency snapshots during renewal");
}

void covariance_envelope_breach_overrides_old_plan_reuse() {
  SuccessfulStages stages;
  stages.cable_validation_status =
      TimedCableValidationStatus::covariance_envelope_breach;
  stages.cable_stop_required = true;
  stages.revalidation_lease_sequence = 71;
  stages.revalidation_lease_expires_at_ns = 20'000;
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease(13, 70, 17'000)).published(),
          "test setup did not publish the old plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 16'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });
  PlanningCycleRequest request;
  request.cycle_sequence = 20;
  request.random_seed = 57;
  request.triggered_at = {16'000};
  const PlanningCycleResult result = loop.run_cycle(request);

  require(result.status ==
              PlanningCycleStatus::covariance_envelope_breached &&
              result.state == PlanningState::covariance_envelope_breach &&
              result.controlled_stop_required && !result.publication &&
              result.root_cause &&
              result.root_cause->cause ==
                  PlanningFailureCause::covariance_envelope_breach &&
              lease_monitor.isRevoked(70) && !publisher.current(),
          "covariance breach was downgraded to an old-plan reuse decision");
}

void parameterization_deadline_revalidates_old_plan() {
  SuccessfulStages stages;
  stages.parameterization_status =
      ParameterizationStatus::deadline_exceeded;
  stages.revalidation_lease_sequence = 71;
  stages.revalidation_lease_expires_at_ns = 20'000;
  AuthorizedPlanningResultPublisher publisher;
  require(publish_candidate(publisher, lease(13, 70, 18'000)).published(),
          "test setup did not publish the old plan");
  ExecutionLeaseMonitor lease_monitor;
  std::int64_t now = 17'000;
  MainPlanningLoop loop(stages, publisher, lease_monitor, [&now] {
    now += 10;
    return MonotonicTime{now};
  });
  PlanningCycleRequest request;
  request.cycle_sequence = 21;
  request.random_seed = 58;
  request.triggered_at = {17'000};
  const PlanningCycleResult result = loop.run_cycle(request);

  require(result.status == PlanningCycleStatus::current_plan_reused &&
              result.state == PlanningState::path_valid && result.root_cause &&
              result.root_cause->cause ==
                  PlanningFailureCause::parameterization_deadline_exceeded &&
              result.publication &&
              result.publication->lease.lease_sequence == 71 &&
              lease_monitor.isRevoked(70),
          "parameterization deadline was collapsed into ordinary infeasibility");
}

}  // namespace

int main() {
  try {
    actual_state_success_path_is_published_with_a_fresh_lease();
    committed_terminal_state_is_the_planning_start();
    urgent_commitment_safety_event_revokes_before_stop_and_search();
    too_close_obstacle_enters_emergency_stop();
    revoked_commitment_cannot_be_reused_on_the_next_cycle();
    every_commitment_safety_event_has_a_traceable_transition();
    complete_commitment_constraints_can_break_authorization();
    dependency_change_breaks_commitment_before_analysis_or_search();
    stale_candidate_is_not_revalidated_leased_or_published();
    invalid_lease_cannot_expose_a_plan_without_authorization();
    search_deadline_revalidates_old_plan_before_continuing();
    total_cycle_deadline_is_enforced_at_the_publication_boundary();
    old_plan_reauthorization_reports_a_deadline_crossing();
    ambiguous_timeout_diagnostics_fail_as_invalid_input();
    timeout_revalidation_failure_cannot_reuse_the_old_plan();
    every_t46_safety_failure_blocks_followup_commands();
    trackable_raw_search_path_is_the_only_smoothing_failure_fallback();
    untrackable_raw_path_preserves_the_smoothing_timeout_root_cause();
    trackable_raw_path_preserves_non_timeout_smoothing_root_cause();
    parameterization_infeasibility_revalidates_old_plan_or_stops();
    invalid_cycle_request_revokes_authorization_and_stops();
    search_failures_are_classified_without_collapsing_root_causes();
    expired_during_planning_lease_is_replaced_not_reused();
    old_lease_sequence_cannot_masquerade_as_a_renewal();
    invalid_parameterization_input_never_falls_back_to_old_execution();
    same_revision_with_changed_dependencies_cannot_renew_old_plan();
    covariance_envelope_breach_overrides_old_plan_reuse();
    parameterization_deadline_revalidates_old_plan();
    candidate_covariance_breach_cannot_fall_back_to_a_valid_old_plan();
    invalid_complete_robot_path_input_cannot_reuse_old_plan();
    replacement_plan_requires_newer_lease_and_revokes_the_old_one();
    slight_improvement_keeps_current_with_latest_context_lease();
    configured_hysteresis_reaches_the_nonvirtual_decision_seam();
    significant_improvement_switches_after_paired_revalidation();
    expired_during_candidate_planning_is_paired_revalidated();
    mismatched_validation_context_stops_before_hysteresis();
    invalid_current_plan_allows_only_a_finite_candidate_direct_switch();
    invalid_candidate_is_paired_with_current_before_the_keep_decision();
    invalid_candidate_cost_keeps_valid_current_or_stops();
    invalid_cost_cannot_cross_atomic_authorization_boundary();
    edited_remaining_path_cannot_cross_atomic_authorization_boundary();
    revoked_publisher_history_cannot_accept_an_older_lease();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
