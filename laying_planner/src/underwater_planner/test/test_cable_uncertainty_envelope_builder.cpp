#include "underwater_planner/core/cable_uncertainty_envelope_builder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using namespace underwater_planner::core;

constexpr MonotonicTime kGenerationTimestamp{5'000'000'000};

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
  parameters.version = 18;
  parameters.calibration_dataset_id = "cable-model-cal-v4";
  parameters.operating_domain_id = "survey-domain-v3";
  parameters.release_point_offset_m = {0.0, 0.0};
  parameters.touchdown_distance_m = 1.0;
  parameters.direction_response_length_m = 2.0;
  parameters.maximum_lag_angle_rad = 1.0;
  parameters.maximum_payout_tracking_error_mps = 0.1;
  parameters.payout_speed_range = {0.0, 1.0};
  parameters.maximum_payout_acceleration_mps2 = 0.5;
  parameters.maximum_tension_tracking_error_n = 10.0;
  parameters.tension_range = {10.0, 100.0};
  parameters.search_integration_step_m = 0.25;
  parameters.validation_integration_step_m = 0.05;
  parameters.touchdown_distance_variance_m2 = 0.0;
  parameters.direction_response_length_variance_m2 = 0.0;
  parameters.lag_angle_process_variance_per_m_rad2 = 0.0;
  parameters.touchdown_process_noise_per_m_m2 = {};
  parameters.approved_sensor_modes = {SensorHealthMode::nominal,
                                      SensorHealthMode::approved_degraded};
  return parameters;
}

ExecutionOperatingEnvelope execution_envelope() {
  ExecutionOperatingEnvelope envelope;
  envelope.version = 12;
  envelope.operating_domain_id = "survey-domain-v3";
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

SpeedPayoutLimits certified_execution_limits(const double maximum_speed_mps) {
  SpeedPayoutLimits limits = execution_envelope().limits;
  limits.ground_speed.maximum_mps = maximum_speed_mps;
  return limits;
}

CertifiedMotionPrimitive primitive(
    const std::uint64_t version, const double arc_length_m,
    const double curvature_per_m, const double minimum_duration_s,
    const double maximum_duration_s,
    const double minimum_reference_progress_advance_m,
    const double maximum_reference_progress_advance_m,
    const double maximum_speed_mps) {
  CertifiedMotionPrimitive result;
  result.version = version;
  result.arc_length_m = arc_length_m;
  result.curvature_per_m = curvature_per_m;
  result.minimum_duration_s = minimum_duration_s;
  result.maximum_duration_s = maximum_duration_s;
  result.minimum_reference_progress_advance_m =
      minimum_reference_progress_advance_m;
  result.maximum_reference_progress_advance_m =
      maximum_reference_progress_advance_m;
  result.certified_execution_limits =
      certified_execution_limits(maximum_speed_mps);
  return result;
}

ReferenceLine reference_line() {
  return make_reference_line(7, "map", {{0.0, 0.0}, {1.0, 0.0},
                                         {2.0, 0.0}, {3.0, 0.0}});
}

OperatingDomain operating_domain() {
  OperatingDomain domain;
  domain.version = 5;
  domain.operating_domain_id = "survey-domain-v3";
  domain.certification_dataset_id = "envelope-proof-v2";
  domain.primitive_set_version = 9;
  domain.initial_uncertainty_version = 4;
  domain.reference_progress_start_m = 0.0;
  domain.reference_progress_end_m = 3.0;
  domain.maximum_planning_length_m = 4.0;
  domain.maximum_planning_time_s = 12.0;
  domain.initial_lag_angle_rad = {-0.2, 0.2};
  domain.initial_uncertainty = {
      0.04, 0.09, 0.01, 0.01, 0.002, 0.003,
  };
  domain.execution_uncertainty = {
      41, 12, "execution-uncertainty-proof-v2", 0.001, 0.002, 0.003};
  domain.approved_sensor_uncertainty = {
      {SensorHealthMode::nominal, 21, "sensor-proof-nominal-v3", 0.0, 0.0,
       0.0},
      {SensorHealthMode::approved_degraded, 22,
       "sensor-proof-degraded-v3", 0.01, 0.002, 0.003},
  };
  domain.allowed_primitives = {
      primitive(101, 1.0, -0.15, 1.25, 2.0, 0.6, 1.0, 0.8),
      primitive(102, 1.0, 0.15, 1.25, 2.0, 0.6, 1.0, 0.8),
  };
  domain.binning = {0.5, 0.2, 0.1, 0.5, 0.5};
  domain.margin_certification = {
      31, "envelope-margin-proof-v2", 0.02, 0.03, 0.04};
  domain.maximum_reachable_sets = 1000;
  return domain;
}

void certified_build_records_dependencies_and_independent_margins() {
  // Design: 18.2.4-33
  const auto result = CableUncertaintyEnvelopeBuilder{}.buildCertifiedUpperBound(
      operating_domain(), reference_line(), SensorHealthMode::nominal,
      model_parameters(), execution_envelope(), 44, kGenerationTimestamp);
  require(result.validity == EnvelopeBuildValidity::valid,
          "a calibrated bounded domain was rejected");
  require(result.segments.size() == 6U,
          "the reference progress range was not segmented");
  require(result.dependencies.generator_version == 44 &&
              result.dependencies.cable_model_version == 18 &&
              result.dependencies.execution_operating_envelope_version == 12 &&
              result.dependencies.reference_line_version == 7 &&
              result.dependencies.operating_domain_version == 5 &&
              result.dependencies.primitive_set_version == 9 &&
              result.dependencies.initial_uncertainty_version == 4 &&
              result.dependencies.sensor_uncertainty_version == 21 &&
              result.dependencies.execution_uncertainty_version == 41 &&
              result.dependencies.margin_certification_version == 31 &&
              result.dependencies.sensor_mode == SensorHealthMode::nominal &&
              result.dependencies.operating_domain_id == "survey-domain-v3" &&
              result.dependencies.cable_model_calibration_dataset_id ==
                  "cable-model-cal-v4" &&
              result.dependencies.certification_dataset_id ==
                  "envelope-proof-v2" &&
              result.dependencies.sensor_calibration_dataset_id ==
                  "sensor-proof-nominal-v3" &&
              result.dependencies.execution_uncertainty_calibration_dataset_id ==
                  "execution-uncertainty-proof-v2" &&
              result.dependencies.margin_calibration_dataset_id ==
                  "envelope-margin-proof-v2",
          "the envelope omitted a generation dependency");
  const auto& margins = result.margin_budget;
  require(margins.certification_version == 31 &&
              margins.calibration_dataset_id ==
                  "envelope-margin-proof-v2" &&
              margins.numerical_integration_stddev_m == 0.02 &&
              margins.reference_normal_sweep_stddev_m == 0.03 &&
              margins.statistical_quantile_stddev_m == 0.04,
          "independent envelope margins were merged or omitted");
  require_near(margins.state_binning_stddev_m, 0.15, 1.0e-12,
               "state-binning margin was not calculated from bin widths");
  for (const auto& segment : result.segments) {
    require_near(segment.lateral_stddev_upper_bound_m,
                 std::sqrt(segment.lateral_variance_upper_bound_m2) + 0.24,
                 1.0e-12,
                 "a certified margin was not included exactly once");
  }
  require(!result.path_joint_risk_implemented &&
              result.generation_timestamp.nanoseconds ==
                  kGenerationTimestamp.nanoseconds &&
              !result.diagnostics.empty() &&
              result.diagnostics.back().timestamp.nanoseconds ==
                  kGenerationTimestamp.nanoseconds &&
              result.risk_semantics ==
                  "POINTWISE_ENVELOPE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE",
          "the envelope overstated its statistical guarantee");
}

void noncomparable_histories_survive_and_inclusion_is_the_only_pruning_rule() {
  OperatingDomain domain = operating_domain();
  domain.allowed_primitives.push_back(domain.allowed_primitives.front());
  domain.allowed_primitives.back().version = 103;
  const auto result = CableUncertaintyEnvelopeBuilder{}.buildCertifiedUpperBound(
      domain, reference_line(), SensorHealthMode::nominal, model_parameters(),
      execution_envelope(), 44, kGenerationTimestamp);
  require(result.validity == EnvelopeBuildValidity::valid,
          "a finite branched reachability problem was rejected");
  require(result.generation_stats.maximum_incomparable_sets_in_bin >= 2U,
          "left and right curvature histories were incorrectly merged");
  require(result.generation_stats.containment_pruned_count > 0U,
          "an exactly contained duplicate reachable set was not pruned");
  require(result.generation_stats.risk_bound_pruned_count == 0U,
          "the generator used an unproved risk-only pruning rule");
  require(result.reachable_set_certificates.size() ==
              result.generation_stats.retained_reachable_set_count,
          "retained reachability evidence is incomplete");
}

void a_worked_uncertainty_bound_is_conservative_and_deterministic() {
  OperatingDomain domain = operating_domain();
  domain.maximum_planning_length_m = 1.0;
  domain.reference_progress_end_m = 1.0;
  domain.initial_lag_angle_rad = {0.0, 0.0};
  domain.initial_uncertainty = {0.04, 0.09, 0.0, 0.0, 0.0, 0.0};
  domain.execution_uncertainty = {
      41, 12, "execution-uncertainty-proof-v2", 0.0, 0.0, 0.0};
  domain.allowed_primitives = {
      primitive(101, 1.0, 0.0, 1.25, 1.25, 1.0, 1.0, 0.8)};
  domain.margin_certification = {
      31, "envelope-margin-proof-v2", 0.0, 0.0, 0.0};
  const auto first = CableUncertaintyEnvelopeBuilder{}.buildCertifiedUpperBound(
      domain, reference_line(), SensorHealthMode::nominal, model_parameters(),
      execution_envelope(), 44, kGenerationTimestamp);
  const auto second = CableUncertaintyEnvelopeBuilder{}.buildCertifiedUpperBound(
      domain, reference_line(), SensorHealthMode::nominal, model_parameters(),
      execution_envelope(), 44, kGenerationTimestamp);
  require(first.validity == EnvelopeBuildValidity::valid &&
              first.segments.size() == 2U,
          "the worked bound did not build");
  require_near(first.segments.front().lateral_variance_upper_bound_m2, 0.25,
               1.0e-12,
               "the arbitrary-correlation position bound is not conservative");
  require(first.segments == second.segments &&
              first.reachable_set_certificates ==
                  second.reachable_set_certificates,
          "identical certified inputs changed the envelope fields");
}

void progress_bins_and_sensor_certificates_control_the_bound() {
  ReferenceLine sparse_reference =
      make_reference_line(7, "map", {{0.0, 0.0}, {3.0, 0.0}});
  const auto nominal = CableUncertaintyEnvelopeBuilder{}.buildCertifiedUpperBound(
      operating_domain(), sparse_reference, SensorHealthMode::nominal,
      model_parameters(), execution_envelope(), 44, kGenerationTimestamp);
  const auto degraded =
      CableUncertaintyEnvelopeBuilder{}.buildCertifiedUpperBound(
          operating_domain(), sparse_reference,
          SensorHealthMode::approved_degraded, model_parameters(),
          execution_envelope(), 44, kGenerationTimestamp);
  require(nominal.validity == EnvelopeBuildValidity::valid &&
              nominal.segments.size() == 6U,
          "progress bins incorrectly depended on reference sampling density");
  require(degraded.validity == EnvelopeBuildValidity::valid &&
              degraded.dependencies.sensor_uncertainty_version == 22 &&
              degraded.dependencies.sensor_calibration_dataset_id ==
                  "sensor-proof-degraded-v3" &&
              degraded.segments.front().lateral_variance_upper_bound_m2 >
                  nominal.segments.front().lateral_variance_upper_bound_m2,
          "the approved degraded sensor certificate did not enlarge the bound");
  for (std::size_t index = 0U; index < nominal.segments.size(); ++index) {
    require_near(nominal.segments[index].reference_progress_start_m,
                 0.5 * static_cast<double>(index), 1.0e-12,
                 "progress-bin start is not deterministic");
    require_near(nominal.segments[index].reference_progress_end_m,
                 0.5 * static_cast<double>(index + 1U), 1.0e-12,
                 "progress-bin end is not deterministic");
  }
}

void transient_primitives_are_checked_by_propagated_lag_not_steady_state() {
  OperatingDomain domain = operating_domain();
  domain.reference_progress_end_m = 0.1;
  domain.maximum_planning_length_m = 0.1;
  domain.allowed_primitives = {
      primitive(201, 0.1, 2.0, 0.25, 0.3, 0.1, 0.1, 0.4)};
  domain.binning.arc_length_bin_m = 0.1;
  domain.binning.reference_progress_bin_m = 0.1;
  const auto result = CableUncertaintyEnvelopeBuilder{}.buildCertifiedUpperBound(
      domain, reference_line(), SensorHealthMode::nominal, model_parameters(),
      execution_envelope(), 44, kGenerationTimestamp);
  require(result.validity == EnvelopeBuildValidity::valid &&
              result.reachable_set_certificates.size() == 2U &&
              std::abs(result.reachable_set_certificates.back()
                           .lag_angle_rad.minimum) < 1.0,
          "a transient-safe primitive was rejected by a steady-state shortcut");
}

void execution_domain_and_partial_lag_intersections_fail_closed() {
  // Design: 18.2.4-30
  OperatingDomain impossible_speed = operating_domain();
  impossible_speed.allowed_primitives.front().minimum_duration_s = 0.5;
  const auto impossible_result =
      CableUncertaintyEnvelopeBuilder{}.buildCertifiedUpperBound(
          impossible_speed, reference_line(), SensorHealthMode::nominal,
          model_parameters(), execution_envelope(), 44,
          kGenerationTimestamp);
  require(impossible_result.validity == EnvelopeBuildValidity::input_invalid &&
              impossible_result.segments.empty(),
          "a primitive outside the certified speed envelope was accepted");

  OperatingDomain acceleration_escape = operating_domain();
  acceleration_escape.allowed_primitives.front()
      .certified_execution_limits.ground_acceleration.maximum_mps2 = 0.5;
  require(CableUncertaintyEnvelopeBuilder{}
              .buildCertifiedUpperBound(
                  acceleration_escape, reference_line(),
                  SensorHealthMode::nominal, model_parameters(),
                  execution_envelope(), 44, kGenerationTimestamp)
              .validity == EnvelopeBuildValidity::input_invalid,
          "a primitive acceleration certificate escaped the execution envelope");

  OperatingDomain certificate_mismatch = operating_domain();
  certificate_mismatch.execution_uncertainty
      .execution_operating_envelope_version = 11;
  require(CableUncertaintyEnvelopeBuilder{}
              .buildCertifiedUpperBound(
                  certificate_mismatch, reference_line(),
                  SensorHealthMode::nominal, model_parameters(),
                  execution_envelope(), 44, kGenerationTimestamp)
              .validity == EnvelopeBuildValidity::input_invalid,
          "execution uncertainty was not bound to the execution envelope version");

  OperatingDomain partial = operating_domain();
  partial.reference_progress_end_m = 0.1;
  partial.maximum_planning_length_m = 0.1;
  partial.initial_lag_angle_rad = {0.9, 1.0};
  partial.allowed_primitives = {
      primitive(301, 0.1, -1.0, 0.2, 0.3, 0.1, 0.1, 0.5)};
  partial.binning.arc_length_bin_m = 0.1;
  partial.binning.reference_progress_bin_m = 0.1;
  const auto partial_result =
      CableUncertaintyEnvelopeBuilder{}.buildCertifiedUpperBound(
          partial, reference_line(), SensorHealthMode::nominal,
          model_parameters(), execution_envelope(), 44,
          kGenerationTimestamp);
  require(partial_result.validity == EnvelopeBuildValidity::valid &&
              partial_result.reachable_set_certificates.size() == 2U,
          "the in-domain portion of a lag interval was discarded");
  const ClosedRange clipped_lag =
      partial_result.reachable_set_certificates.back().lag_angle_rad;
  require(clipped_lag.minimum < 1.0 && clipped_lag.maximum == 1.0,
          "a partial lag interval was not intersected with the model domain");
}

void actual_cable_model_covariance_is_covered_by_the_envelope() {
  // Design: 18.2.4-15
  OperatingDomain domain = operating_domain();
  domain.maximum_planning_length_m = 1.0;
  domain.reference_progress_end_m = 1.0;
  domain.allowed_primitives = {
      primitive(401, 1.0, 0.0, 2.0, 4.0, 1.0, 1.0, 0.5)};
  const auto envelope =
      CableUncertaintyEnvelopeBuilder{}.buildCertifiedUpperBound(
          domain, reference_line(), SensorHealthMode::nominal,
          model_parameters(), execution_envelope(), 44,
          kGenerationTimestamp);
  require(envelope.validity == EnvelopeBuildValidity::valid,
          "the covariance coverage envelope did not build");

  CableState initial;
  initial.kind = CableStateKind::tracked;
  initial.lag_angle_rad = 0.0;
  initial.lag_angle_variance_rad2 = 0.01;
  initial.timestamp = {1'000'000'000};
  initial.sequence_number = 1;

  TimedPath path;
  path.geometry.metadata = {51, "map", 7, "constant-curvature"};
  path.geometry.points = {
      {0.0, 0.0, 0.0, 0.0, 0.0},
      {1.0, 1.0, 0.0, 0.0, 0.0},
  };
  path.execution_profile.version = 52;
  path.execution_profile.operating_envelope_version = 12;
  path.execution_profile.interpolation_rule = "linear-in-arc-length";
  path.execution_profile.stopping_point_arc_length_m = 1.0;
  path.execution_profile.approved_tracking_limits = execution_envelope().limits;
  path.execution_profile.samples = {
      {0.0, {0}, 0.5, 0.0, 0.5, 0.0, 40.0},
      {1.0, {2'000'000'000}, 0.0, -0.25, 0.0, -0.25, 40.0},
  };

  CableContext context;
  context.current_telemetry = {0.5, 0.0, 40.0, initial.timestamp, 2};
  context.execution_envelope = execution_envelope();
  context.mode = PredictionMode::validation;
  context.sensor_mode = SensorHealthMode::nominal;
  context.uncertainty_envelope_version = 1;
  context.uncertainty_envelope_generator_version = 44;
  context.robot_uncertainty_profile_version = 4;
  context.robot_uncertainty_profile = {
      {0.0, {{0.04, 0.0, 0.0, 0.09}, 0.0, 0.0, 0.01}, 0.003},
      {1.0, {{0.04, 0.0, 0.0, 0.09}, 0.0, 0.0, 0.01}, 0.003},
  };
  const CablePrediction prediction =
      CableModel(model_parameters()).predict(initial, path, context);
  require(prediction.validity == CableModelValidity::valid &&
              prediction.touchdown_covariance_profile_m2.has_value(),
          "the certified boundary trajectory did not produce covariance");
  const auto require_prediction_covered = [](const CablePrediction& actual,
                                             const CableUncertaintyEnvelope& bound) {
    for (std::size_t index = 0U;
         index < actual.touchdown_covariance_profile_m2->size(); ++index) {
      const double progress_m =
          std::min(1.0, actual.touchdown_path.points[index].arc_length_m);
      double progress_aligned_upper_m = 0.0;
      for (const auto& segment : bound.segments) {
        if (progress_m >= segment.reference_progress_start_m - 1.0e-12 &&
            progress_m <= segment.reference_progress_end_m + 1.0e-12) {
          progress_aligned_upper_m =
              std::max(progress_aligned_upper_m,
                       segment.lateral_stddev_upper_bound_m);
        }
      }
      require(progress_aligned_upper_m > 0.0 &&
                  std::sqrt((*actual.touchdown_covariance_profile_m2)[index]
                                .yy_m2) <= progress_aligned_upper_m,
              "actual lateral covariance exceeded its progress-aligned envelope");
    }
  };
  require_prediction_covered(prediction, envelope);

  const auto degraded_envelope =
      CableUncertaintyEnvelopeBuilder{}.buildCertifiedUpperBound(
          domain, reference_line(), SensorHealthMode::approved_degraded,
          model_parameters(), execution_envelope(), 44,
          kGenerationTimestamp);
  context.sensor_mode = SensorHealthMode::approved_degraded;
  const CablePrediction degraded_prediction =
      CableModel(model_parameters()).predict(initial, path, context);
  require(degraded_envelope.validity == EnvelopeBuildValidity::valid &&
              degraded_prediction.validity == CableModelValidity::valid,
          "the approved degraded coverage scenario was rejected");
  require_prediction_covered(degraded_prediction, degraded_envelope);
}

void primitive_sweeps_cover_intermediate_progress_bins() {
  OperatingDomain domain = operating_domain();
  domain.maximum_planning_length_m = 1.0;
  domain.reference_progress_end_m = 1.0;
  domain.allowed_primitives = {
      primitive(501, 1.0, 0.0, 2.0, 4.0, 0.6, 1.0, 0.5)};
  const auto envelope =
      CableUncertaintyEnvelopeBuilder{}.buildCertifiedUpperBound(
          domain, reference_line(), SensorHealthMode::nominal,
          model_parameters(), execution_envelope(), 44,
          kGenerationTimestamp);
  require(envelope.validity == EnvelopeBuildValidity::valid &&
              envelope.segments.size() == 2U,
          "the primitive-sweep envelope did not build");
  require_near(envelope.segments.front().lateral_variance_upper_bound_m2,
               envelope.segments.back().lateral_variance_upper_bound_m2,
               1.0e-12,
               "an intermediate progress bin omitted the primitive sweep");
  require(envelope.reachable_set_certificates.back()
                  .swept_reference_progress_m.minimum == 0.0 &&
              envelope.reachable_set_certificates.back()
                  .swept_reference_progress_m.maximum == 1.0,
          "the reachable-set proof omitted swept reference progress");
}

void domain_and_resource_failures_are_closed() {
  OperatingDomain mismatch = operating_domain();
  mismatch.operating_domain_id = "other-domain";
  const auto mismatch_result =
      CableUncertaintyEnvelopeBuilder{}.buildCertifiedUpperBound(
          mismatch, reference_line(), SensorHealthMode::nominal,
          model_parameters(), execution_envelope(), 44, kGenerationTimestamp);
  require(mismatch_result.validity ==
              EnvelopeBuildValidity::dependency_mismatch &&
              mismatch_result.segments.empty(),
          "a mismatched operating domain emitted an envelope");

  OperatingDomain unsupported = operating_domain();
  unsupported.approved_sensor_uncertainty.erase(
      unsupported.approved_sensor_uncertainty.begin());
  require(CableUncertaintyEnvelopeBuilder{}
              .buildCertifiedUpperBound(
                  unsupported, reference_line(), SensorHealthMode::nominal,
                  model_parameters(), execution_envelope(), 44,
                  kGenerationTimestamp)
              .validity == EnvelopeBuildValidity::sensor_mode_unapproved,
          "an uncertified sensor mode was extrapolated");

  OperatingDomain exhausted = operating_domain();
  exhausted.maximum_reachable_sets = 1;
  const auto exhausted_result =
      CableUncertaintyEnvelopeBuilder{}.buildCertifiedUpperBound(
          exhausted, reference_line(), SensorHealthMode::nominal,
          model_parameters(), execution_envelope(), 44, kGenerationTimestamp);
  require(exhausted_result.validity ==
              EnvelopeBuildValidity::resource_limit_exceeded &&
              exhausted_result.segments.empty(),
          "reachability resource exhaustion published a partial envelope");

  OperatingDomain oversegmented = operating_domain();
  oversegmented.binning.reference_progress_bin_m = 0.001;
  const auto oversegmented_result =
      CableUncertaintyEnvelopeBuilder{}.buildCertifiedUpperBound(
          oversegmented, reference_line(), SensorHealthMode::nominal,
          model_parameters(), execution_envelope(), 44, kGenerationTimestamp);
  require(oversegmented_result.validity ==
              EnvelopeBuildValidity::resource_limit_exceeded &&
              oversegmented_result.segments.empty(),
          "an over-budget progress discretization emitted a partial envelope");

  const auto missing_timestamp =
      CableUncertaintyEnvelopeBuilder{}.buildCertifiedUpperBound(
          operating_domain(), reference_line(), SensorHealthMode::nominal,
          model_parameters(), execution_envelope(), 44, MonotonicTime{-1});
  require(missing_timestamp.validity == EnvelopeBuildValidity::input_invalid &&
              !missing_timestamp.diagnostics.empty(),
          "a missing generation timestamp was not diagnosed");
}

}  // namespace

int main() {
  constexpr std::uint64_t kSeed = 1818;
  try {
    certified_build_records_dependencies_and_independent_margins();
    noncomparable_histories_survive_and_inclusion_is_the_only_pruning_rule();
    a_worked_uncertainty_bound_is_conservative_and_deterministic();
    progress_bins_and_sensor_certificates_control_the_bound();
    transient_primitives_are_checked_by_propagated_lag_not_steady_state();
    execution_domain_and_partial_lag_intersections_fail_closed();
    actual_cable_model_covariance_is_covered_by_the_envelope();
    primitive_sweeps_cover_intermediate_progress_bins();
    domain_and_resource_failures_are_closed();
    std::cout << "cable uncertainty envelope builder checks passed: 9"
              << " seed=" << kSeed
              << " input_version=t18-envelope-builder/v1"
              << " units=SI timestamp=monotonic-ns"
              << " domain=survey-domain-v3"
              << " risk=pointwise-envelope-only\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "cable uncertainty envelope builder failure: " << error.what()
              << " seed=" << kSeed
              << " input_version=t18-envelope-builder/v1"
              << " units=SI timestamp=monotonic-ns"
              << " domain=survey-domain-v3"
              << " risk=pointwise-envelope-only\n";
    return 1;
  }
}
