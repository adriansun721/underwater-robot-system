#include "underwater_planner/core/timed_cable_candidate_verifier.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace underwater_planner::core {
namespace {

bool finite(const double value) { return std::isfinite(value); }

void issue(TimedCableCandidateResult& result, const std::string& code,
           const std::string& message, const MonotonicTime timestamp,
           const DiagnosticSeverity severity = DiagnosticSeverity::error) {
  result.issues.push_back(message);
  result.diagnostics.push_back(
      {severity, code, "timed_cable_validation", message, timestamp});
}

bool dependency_match(const TimedCableCandidateInput& input,
                      const CablePrediction& prediction,
                      const LockedCableUncertaintyEnvelope& locked) {
  if (!locked.envelope || locked.envelope_version == 0U) {
    return false;
  }
  const EnvelopeDependencies& dependencies = locked.envelope->dependencies;
  return dependencies.cable_model_version ==
             prediction.dependencies.cable_model_version &&
         dependencies.cable_model_calibration_dataset_id ==
             prediction.dependencies.calibration_dataset_id &&
         dependencies.reference_line_version ==
             input.reference_line.version &&
         dependencies.execution_operating_envelope_version ==
             input.robot_path.execution_profile.operating_envelope_version &&
         dependencies.sensor_mode == input.cable_context.sensor_mode &&
         dependencies.operating_domain_id == prediction.dependencies.operating_domain_id &&
         dependencies.operating_domain_id == input.cable_context
                                                 .execution_envelope
                                                 .operating_domain_id &&
         input.cable_context.uncertainty_envelope_version ==
             locked.envelope_version &&
         input.cable_context.uncertainty_envelope_generator_version ==
             dependencies.generator_version;
}

bool valid_prediction_profile(const CablePrediction& prediction) {
  return prediction.validity == CableModelValidity::valid &&
         prediction.touchdown_covariance_profile_m2.has_value() &&
         prediction.touchdown_path.points.size() ==
             prediction.state_profile.size() &&
         prediction.touchdown_path.points.size() ==
             prediction.touchdown_covariance_profile_m2->size();
}

double lateral_stddev(const ReferencePoint& reference,
                      const Covariance2dM2& covariance) {
  const double variance =
      reference.normal_x *
          (covariance.xx_m2 * reference.normal_x +
           covariance.xy_m2 * reference.normal_y) +
      reference.normal_y *
          (covariance.yx_m2 * reference.normal_x +
           covariance.yy_m2 * reference.normal_y);
  if (!finite(variance) || variance < -1.0e-12) {
    return -1.0;
  }
  return std::sqrt(std::max(0.0, variance));
}

}  // namespace

TimedCableCandidateVerifier::TimedCableCandidateVerifier(
    CableModel model, CableUncertaintyEnvelopeManager* envelope_manager)
    : model_(std::move(model)), envelope_manager_(envelope_manager) {}

TimedCableCandidateResult TimedCableCandidateVerifier::validate(
    const TimedCableCandidateInput& input) const {
  TimedCableCandidateResult result;
  result.risk_semantics = kPointwiseEnvelopeRiskSemantics;
  if (input.evaluation_timestamp.nanoseconds < 0 ||
      !finite(input.envelope_audit_tolerance_m) ||
      input.envelope_audit_tolerance_m < 0.0 ||
      input.cable_context.mode != PredictionMode::validation ||
      !underwater_planner::core::validate(input.initial_cable_state).valid ||
      !underwater_planner::core::validate(input.robot_path).valid ||
      !underwater_planner::core::validate(input.reference_line).valid ||
      input.reference_line.version == 0U ||
      input.robot_path.geometry.metadata.reference_line_version !=
          input.reference_line.version ||
      !input.locked_envelope.has_value() || envelope_manager_ == nullptr) {
    issue(result, "TIMED_CABLE_INPUT_INVALID",
          "timed candidate, validation context, reference progress, and a locked envelope are required",
          input.evaluation_timestamp);
    return result;
  }

  const LockedCableUncertaintyEnvelope& locked = *input.locked_envelope;
  if (!locked.envelope ||
      locked.coverage_certification.certified_envelope_version !=
          locked.envelope_version ||
      !locked.coverage_certification.passed) {
    issue(result, "TIMED_CABLE_ENVELOPE_CONTEXT_INVALID",
          "locked uncertainty envelope is incomplete or does not match the validation context",
          input.evaluation_timestamp);
    result.status = TimedCableValidationStatus::covariance_envelope_unavailable;
    return result;
  }

  // The validation entry point always recomputes the complete prediction.
  // Search means and cached covariance profiles are deliberately ignored.
  CablePrediction prediction =
      model_.predict(input.initial_cable_state, input.robot_path,
                     input.cable_context);
  result.cable_prediction = prediction;
  if (!valid_prediction_profile(prediction)) {
    result.status = TimedCableValidationStatus::cable_model_invalid;
    issue(result, "TIMED_CABLE_MODEL_INVALID",
          "high-precision timed cable prediction failed or omitted covariance",
          input.evaluation_timestamp);
    return result;
  }
  if (!dependency_match(input, prediction, locked)) {
    result.status = TimedCableValidationStatus::covariance_envelope_unavailable;
    issue(result, "TIMED_CABLE_DEPENDENCY_MISMATCH",
          "prediction and uncertainty envelope dependencies do not match",
          input.evaluation_timestamp);
    return result;
  }

  const std::size_t sample_count = prediction.touchdown_path.points.size();
  if (input.reference_progress_m.size() != sample_count ||
      input.interval_bound_certificate.upper_bound_error_m.size() !=
          sample_count - 1U ||
      input.interval_bound_certificate.version == 0U ||
      input.interval_bound_certificate.version != locked.envelope_version) {
    result.status = TimedCableValidationStatus::input_invalid;
    issue(result, "TIMED_CABLE_CORRIDOR_INPUT_INVALID",
          "reference progress and interval-bound certificate must align with the predicted path",
          input.evaluation_timestamp);
    return result;
  }

  for (std::size_t index = 0U; index < sample_count; ++index) {
    const auto reference =
        input.reference_line.query(input.reference_progress_m[index]);
    if (!reference.has_value()) {
      result.status = TimedCableValidationStatus::input_invalid;
      issue(result, "TIMED_CABLE_REFERENCE_PROGRESS_INVALID",
            "predicted touchdown progress is outside the reference line",
            input.evaluation_timestamp);
      return result;
    }
    const double actual = lateral_stddev(
        *reference, (*prediction.touchdown_covariance_profile_m2)[index]);
    if (actual < 0.0) {
      result.status = TimedCableValidationStatus::cable_model_invalid;
      issue(result, "TIMED_CABLE_COVARIANCE_INVALID",
            "predicted touchdown covariance is not finite positive semidefinite",
            input.evaluation_timestamp);
      return result;
    }
    const EnvelopeAuditResult audit = envelope_manager_->auditActualLateralStddev(
        locked, input.reference_progress_m[index], actual,
        input.envelope_audit_tolerance_m, input.evaluation_timestamp);
    result.maximum_actual_lateral_stddev_m =
        std::max(result.maximum_actual_lateral_stddev_m, actual);
    result.maximum_allowed_lateral_stddev_m =
        std::max(result.maximum_allowed_lateral_stddev_m,
                 audit.allowed_lateral_stddev_m);
    if (audit.status == EnvelopeAuditStatus::covariance_envelope_breach) {
      result.status = TimedCableValidationStatus::covariance_envelope_breach;
      result.stop_required = true;
      result.diagnostics.insert(result.diagnostics.end(),
                                audit.diagnostics.begin(),
                                audit.diagnostics.end());
      result.issues.emplace_back("actual lateral covariance breached the certified envelope");
      return result;
    }
    if (audit.status != EnvelopeAuditStatus::pass) {
      result.status = TimedCableValidationStatus::covariance_envelope_unavailable;
      issue(result, "TIMED_CABLE_ENVELOPE_AUDIT_UNAVAILABLE",
            "the locked uncertainty envelope could not be audited",
            input.evaluation_timestamp);
      return result;
    }
  }

  CableCorridorEvaluationInput corridor_input;
  corridor_input.reference_line = input.reference_line;
  corridor_input.touchdown_path = prediction.touchdown_path;
  corridor_input.touchdown_path.metadata.interpolation_rule =
      "piecewise-linear-in-arc-length";
  corridor_input.reference_progress_m = input.reference_progress_m;
  corridor_input.touchdown_covariance_m2 =
      *prediction.touchdown_covariance_profile_m2;
  corridor_input.interval_bound_certificate = input.interval_bound_certificate;
  corridor_input.evaluation_timestamp = input.evaluation_timestamp;
  corridor_input.reference_is_deterministic = input.reference_is_deterministic;
  corridor_input.covariance_includes_coordinate_transform_error =
      input.covariance_includes_coordinate_transform_error;
  result.corridor_result =
      CableCorridorEvaluator(input.corridor_policy)
          .evaluate_pointwise(corridor_input);
  result.corridor_result.covariance_envelope_audit_performed = true;
  if (result.corridor_result.validity != CorridorEvaluationValidity::valid) {
    result.status = TimedCableValidationStatus::corridor_invalid;
    issue(result, "TIMED_CABLE_CORRIDOR_INVALID",
          "full timed touchdown path corridor evaluation failed",
          input.evaluation_timestamp);
    return result;
  }
  if (!result.corridor_result.hard_feasible) {
    result.status = TimedCableValidationStatus::corridor_violation;
    issue(result, "TIMED_CABLE_CORRIDOR_VIOLATION",
          "full timed touchdown path violates the corridor hard gate",
          input.evaluation_timestamp);
    return result;
  }

  result.laying_result = CableLayingEvaluator{}.evaluate(
      input.initial_cable_state.laying_memory, prediction.touchdown_path,
      prediction.state_profile, input.terrain, input.laying_limits,
      input.history_boundary);
  if (!result.laying_result.valid || !result.laying_result.hard_feasible) {
    result.status = TimedCableValidationStatus::laying_invalid;
    issue(result, "TIMED_CABLE_LAYING_INVALID",
          "full timed touchdown path violates a mechanical or terrain hard gate",
          input.evaluation_timestamp);
    return result;
  }

  result.valid = true;
  result.status = TimedCableValidationStatus::valid;
  result.terminal_cable_state = prediction.terminal_state;
  result.terminal_cable_state->laying_memory = result.laying_result.terminal_memory;
  result.diagnostics.push_back(
      {DiagnosticSeverity::info, "TIMED_CABLE_VALIDATION_PASSED",
       "timed_cable_validation", "high-precision cable, envelope, corridor, and laying checks passed",
       input.evaluation_timestamp});
  return result;
}

}  // namespace underwater_planner::core
