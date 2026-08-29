#include "underwater_planner/core/timed_cable_candidate_verifier.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

namespace {
using namespace underwater_planner::core;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

CableModelParameters model_parameters() {
  CableModelParameters p;
  p.version = 14;
  p.calibration_dataset_id = "cable-mean-cal-v1";
  p.operating_domain_id = "competition-seabed-v1";
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
  e.operating_domain_id = "competition-seabed-v1";
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
  path.execution_profile.samples = {{0.0, {0}, 0.5, 0.0, 0.50, 0.0, 40.0},
                                     {1.0, {2'000'000'000}, 0.5, 0.0, 0.50, 0.0, 40.0},
                                     {2.0, {4'000'000'000}, 0.5, 0.0, 0.50, 0.0, 40.0},
                                     {3.0, {6'000'000'000}, 0.0, -0.25, 0.0, -0.25, 40.0}};
  return path;
}

CableState actual_state() {
  CableState state;
  state.kind = CableStateKind::tracked;
  state.lag_angle_rad = 0.0;
  state.lag_angle_variance_rad2 = 0.01;
  state.timestamp = {1'000'000'000};
  state.sequence_number = 21;
  return state;
}

CableContext validation_context() {
  CableContext c;
  c.current_telemetry = {0.5, 0.0, 40.0, {1'000'000'000}, 11};
  c.execution_envelope = execution_envelope();
  c.mode = PredictionMode::validation;
  c.sensor_mode = SensorHealthMode::nominal;
  c.uncertainty_envelope_version = 9;
  c.uncertainty_envelope_generator_version = 10;
  c.robot_uncertainty_profile_version = 11;
  for (double s : {0.0, 1.0, 2.0, 3.0}) {
    c.robot_uncertainty_profile.push_back(
        {s, {{0.04, 0.0, 0.0, 0.09}, 0.0, 0.0, 0.01}, 0.002});
  }
  return c;
}

ReferenceLine reference_line() {
  return make_reference_line(4, "map", {{0.0, 0.0}, {1.0, 0.0},
                                         {2.0, 0.0}, {3.0, 0.0}});
}

TerrainLayers terrain() {
  TerrainLayers t;
  t.source_map_version = {"map", 1, {1'000'000'000}, "map"};
  t.analysis_config_version = 12;
  t.operating_domain_id = "competition-seabed-v1";
  t.surface_fit_window_size_m = 1.0;
  t.surface.width = 100;
  t.surface.height = 100;
  t.surface.resolution_m = 1.0;
  t.surface.origin_x_m = -50.0;
  t.surface.origin_y_m = -50.0;
  t.surface.cells.resize(t.surface.width * t.surface.height);
  t.cable_laying.cells.resize(t.surface.cells.size());
  for (auto& cell : t.surface.cells) {
    cell.status = TerrainEstimateStatus::valid;
    cell.support_ratio = 1.0;
  }
  for (auto& cell : t.cable_laying.cells) {
    cell.known = true;
    cell.confidence = 1.0;
  }
  return t;
}

CableLayingLimits laying_limits() {
  return {16, "competition-seabed-v1", 0.2, 0.8, 0.5, 0.2, 0.5,
          1.0, 0.5, 0.01, 1.0, 1.0, 1.0};
}

CableCorridorRiskPolicy corridor_policy() {
  return {17, "corridor-cal-v1", "competition-seabed-v1", 1.0e-3,
          2.0, 3.0, 10.0, 0.01, true};
}

CableUncertaintyEnvelope make_envelope() {
  CableUncertaintyEnvelope e;
  e.validity = EnvelopeBuildValidity::valid;
  e.dependencies = {10, 14, 7, 4, 2, 3, 4, 5, 6, 7,
                    SensorHealthMode::nominal, "competition-seabed-v1",
                    "cable-mean-cal-v1", "cert-v1", "sensor-v1",
                    "exec-v1", "margin-v1"};
  e.margin_budget = {7, "margin-v1", 0.0, 0.0, 0.0, 0.0};
  e.segments = {{0.0, 3.0, 100.0, 10.0}};
  e.generation_timestamp = {1'000'000'000};
  e.path_joint_risk_implemented = false;
  e.risk_semantics = kPointwiseEnvelopeRiskSemantics;
  return e;
}

TimedCableCandidateInput input(const LockedCableUncertaintyEnvelope& locked) {
  TimedCableCandidateInput value;
  value.initial_cable_state = actual_state();
  value.robot_path = timed_path();
  value.cable_context = validation_context();
  value.reference_line = reference_line();
  value.corridor_policy = corridor_policy();
  const CablePrediction predicted = CableModel(model_parameters()).predict(
      value.initial_cable_state, value.robot_path, value.cable_context);
  value.reference_progress_m = predicted.robot_arc_length_profile_m;
  value.interval_bound_certificate = {
      9, std::vector<double>(value.reference_progress_m.size() - 1U, 0.0)};
  value.terrain = terrain();
  value.laying_limits = laying_limits();
  value.history_boundary = CableHistoryBoundary::explicit_task_start;
  value.reference_is_deterministic = true;
  value.covariance_includes_coordinate_transform_error = true;
  value.envelope_audit_tolerance_m = 0.0;
  value.evaluation_timestamp = {2'000'000'000};
  value.locked_envelope = locked;
  return value;
}

LockedCableUncertaintyEnvelope register_envelope(
    CableUncertaintyEnvelopeManager& manager, const bool tight = false) {
  CableUncertaintyEnvelope envelope = make_envelope();
  if (tight) {
    envelope.segments.front().lateral_variance_upper_bound_m2 = 1.0e-8;
    envelope.segments.front().lateral_stddev_upper_bound_m = 0.0001;
  }
  EnvelopeCoverageCertification certification;
  certification.version = 21;
  certification.calibration_dataset_id = "coverage-v1";
  certification.passed = true;
  certification.audited_at = {1'000'000'000};
  certification.valid_until = {9'000'000'000};
  certification.certified_envelope_version = 9;
  certification.certified_dependencies = envelope.dependencies;
  const auto registration = manager.registerValidated(9, envelope, certification);
  require(registration.accepted(),
          "test envelope registration failed status=" +
              std::to_string(static_cast<int>(registration.status)) +
              (registration.diagnostics.empty() ? std::string{} :
               " code=" + registration.diagnostics.front().code +
                   " message=" + registration.diagnostics.front().message));
  require(manager.setCurrentContext(
                     {4, SensorHealthMode::nominal, "competition-seabed-v1",
                      14, 7},
                     1, {1'000'000'001})
              .context_update_status == EnvelopeContextUpdateStatus::accepted,
          "test envelope context setup failed");
  const auto locked = manager.getValidated(
      {4, SensorHealthMode::nominal, "competition-seabed-v1", 14, 7},
      {2'000'000'000});
  require(locked.has_value(), "test envelope could not be locked");
  return *locked;
}

void rejects_missing_locked_envelope() {
  CableUncertaintyEnvelopeManager manager;
  TimedCableCandidateVerifier verifier(CableModel(model_parameters()), &manager);
  TimedCableCandidateInput value;
  value.evaluation_timestamp = {1};
  const auto result = verifier.validate(value);
  require(result.status == TimedCableValidationStatus::input_invalid &&
              !result.valid,
          "missing validation inputs were not rejected");
}

void validates_complete_timed_candidate_and_returns_terminal_state() {
  // Design: 18.2.4-key-6
  CableUncertaintyEnvelopeManager manager;
  const auto locked = register_envelope(manager);
  TimedCableCandidateVerifier verifier(CableModel(model_parameters()), &manager);
  const auto result = verifier.validate(input(locked));
  require(result.valid && result.status == TimedCableValidationStatus::valid,
          "complete timed candidate did not pass all hard gates status=" +
              std::to_string(static_cast<int>(result.status)) +
              " issues=" + (result.issues.empty() ? std::string{} : result.issues.front()) +
              " laying_valid=" + std::to_string(result.laying_result.valid) +
              " laying_hard=" + std::to_string(result.laying_result.hard_feasible) +
              " laying_failures=" + std::to_string(result.laying_result.failure_reasons.size()) +
              " first_failure=" +
              (result.laying_result.failure_reasons.empty()
                   ? std::string{"none"}
                   : std::to_string(static_cast<int>(result.laying_result.failure_reasons.front()))));
  require(result.cable_prediction.has_value() &&
              result.terminal_cable_state.has_value() &&
              result.terminal_cable_state->laying_memory.canonical_signature ==
                  result.laying_result.terminal_memory.canonical_signature &&
              result.laying_result.terminal_memory.trailing_support_samples.size() >=
                  2U &&
              result.corridor_result.hard_feasible,
          "successful validation omitted terminal cable evidence");
}

void map_confidence_is_a_hard_gate_not_a_covariance_scale() {
  // Design: 18.2.4-11
  CableUncertaintyEnvelopeManager manager;
  const auto locked = register_envelope(manager);
  TimedCableCandidateVerifier verifier(CableModel(model_parameters()), &manager);

  auto high_confidence = input(locked);
  const auto high_result = verifier.validate(high_confidence);
  require(high_result.valid && high_result.cable_prediction.has_value(),
          "high-confidence baseline candidate was rejected");

  auto acceptable_confidence = input(locked);
  for (auto& cell : acceptable_confidence.terrain.cable_laying.cells) {
    cell.confidence = 0.6;
  }
  const auto acceptable_result = verifier.validate(acceptable_confidence);
  require(acceptable_result.valid &&
              acceptable_result.maximum_actual_lateral_stddev_m ==
                  high_result.maximum_actual_lateral_stddev_m,
          "map confidence directly scaled predicted cable covariance");

  auto low_confidence = input(locked);
  for (auto& cell : low_confidence.terrain.cable_laying.cells) {
    cell.confidence = 0.49;
  }
  const auto low_result = verifier.validate(low_confidence);
  require(low_result.status == TimedCableValidationStatus::laying_invalid &&
              low_result.cable_prediction.has_value() &&
              low_result.maximum_actual_lateral_stddev_m ==
                  high_result.maximum_actual_lateral_stddev_m,
          "low map confidence was not an independent hard terrain gate");
}

void full_timed_candidate_rechecks_actual_history_boundary() {
  // Design: 18.2.4-24
  // Design: 18.2.4-key-5
  CableUncertaintyEnvelopeManager manager;
  const auto locked = register_envelope(manager);
  TimedCableCandidateVerifier verifier(CableModel(model_parameters()), &manager);
  auto value = input(locked);
  const CablePrediction prediction = CableModel(model_parameters()).predict(
      value.initial_cable_state, value.robot_path, value.cable_context);
  require(prediction.validity == CableModelValidity::valid &&
              prediction.touchdown_path.points.size() >= 2U,
          "history-boundary fixture cable prediction failed");
  const Vector2m first{prediction.touchdown_path.points.front().x_m,
                       prediction.touchdown_path.points.front().y_m};
  CableConstraintMemory history;
  history.previous_distinct_touchdown_points_m = {
      {first.x_m, first.y_m - 1.0}, first};
  history.trailing_support_samples = {
      {0.0, {first.x_m, first.y_m - 2.0}},
      {1.0, {first.x_m, first.y_m - 1.0}},
      {2.0, first},
  };
  history.retained_arc_length_m = 2.0;
  history.canonical_signature = 91;
  value.initial_cable_state.laying_memory = history;
  value.history_boundary = CableHistoryBoundary::actual_laying_history;

  const auto result = verifier.validate(value);

  require(result.status == TimedCableValidationStatus::laying_invalid &&
              !result.valid && result.laying_result.valid &&
              !result.laying_result.hard_feasible &&
              !result.laying_result.failure_segments.empty() &&
              result.laying_result.failure_segments.front().start_arc_length_m ==
                  0.0,
          "full timed validation ignored the actual-history splice curvature");
}

void slower_timed_profile_is_repredicted_under_a_new_version() {
  // Design: 18.2.4-29
  CableUncertaintyEnvelopeManager manager;
  const auto locked = register_envelope(manager);
  TimedCableCandidateVerifier verifier(CableModel(model_parameters()), &manager);
  const auto nominal_input = input(locked);
  const auto nominal = verifier.validate(nominal_input);
  require(nominal.valid && nominal.cable_prediction.has_value(),
          "nominal timed profile failed before slowdown comparison");

  auto slowed_input = nominal_input;
  auto& slowed_profile = slowed_input.robot_path.execution_profile;
  slowed_profile.version = 42;
  slowed_profile.samples[1] =
      {1.0, {4'000'000'000}, 0.25, -0.125, 0.25, -0.125, 40.0};
  slowed_profile.samples[2] =
      {2.0, {8'000'000'000}, 0.25, 0.0, 0.25, 0.0, 40.0};
  slowed_profile.samples[3] =
      {3.0, {12'000'000'000}, 0.0, -0.125, 0.0, -0.125, 40.0};
  const auto slowed = verifier.validate(slowed_input);

  require(slowed.valid && slowed.cable_prediction.has_value() &&
              nominal.cable_prediction->dependencies.execution_profile_version ==
                  41 &&
              slowed.cable_prediction->dependencies.execution_profile_version ==
                  42 &&
              slowed.cable_prediction->state_profile.back()
                      .timestamp.nanoseconds !=
                  nominal.cable_prediction->state_profile.back()
                      .timestamp.nanoseconds &&
              slowed_input.robot_path.geometry.points.size() ==
                  nominal_input.robot_path.geometry.points.size() &&
              slowed_input.robot_path.geometry.points.back().x_m ==
                  nominal_input.robot_path.geometry.points.back().x_m,
          "a valid slowdown profile reused the nominal timed cable prediction");
}

void envelope_breach_requires_stop_and_invalidates_dependencies() {
  CableUncertaintyEnvelopeManager manager;
  auto locked = register_envelope(manager, true);
  TimedCableCandidateVerifier verifier(CableModel(model_parameters()), &manager);
  auto value = input(locked);
  const auto result = verifier.validate(value);
  require(result.status == TimedCableValidationStatus::covariance_envelope_breach &&
              result.stop_required && !result.valid &&
              manager.envelopeStatus(9) == DependentArtifactStatus::invalidated,
          "covariance envelope breach did not stop and invalidate dependents");
}

}  // namespace

int main() {
  try {
    rejects_missing_locked_envelope();
    validates_complete_timed_candidate_and_returns_terminal_state();
    map_confidence_is_a_hard_gate_not_a_covariance_scale();
    full_timed_candidate_rechecks_actual_history_boundary();
    slower_timed_profile_is_repredicted_under_a_new_version();
    envelope_breach_requires_stop_and_invalidates_dependencies();
    std::cout << "timed cable candidate verifier checks passed: 6\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "timed cable candidate verifier failure: " << error.what()
              << "\n";
    return 1;
  }
}
