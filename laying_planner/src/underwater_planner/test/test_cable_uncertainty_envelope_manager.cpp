#include "underwater_planner/core/cable_uncertainty_envelope_manager.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using underwater_planner::core::CableUncertaintyEnvelope;
using underwater_planner::core::CableUncertaintyEnvelopeManager;
using underwater_planner::core::CableUncertaintyEnvelopeSegment;
using underwater_planner::core::DependentArtifactStatus;
using underwater_planner::core::DiagnosticSeverity;
using underwater_planner::core::EnvelopeAuditStatus;
using underwater_planner::core::EnvelopeBuildValidity;
using underwater_planner::core::EnvelopeCoverageCertification;
using underwater_planner::core::EnvelopeContextUpdateStatus;
using underwater_planner::core::EnvelopeLookupKey;
using underwater_planner::core::EnvelopeQueryStatus;
using underwater_planner::core::EnvelopeRegistrationStatus;
using underwater_planner::core::MonotonicTime;
using underwater_planner::core::SensorHealthMode;

constexpr MonotonicTime kTimestamp{9'000'000'000};
constexpr MonotonicTime kUseTime{9'100'000'000};
constexpr MonotonicTime kValidUntil{10'000'000'000};

void require(const bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void require_near(const double actual, const double expected,
                  const double tolerance, const std::string& message) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(message + ": expected " +
                             std::to_string(expected) + ", got " +
                             std::to_string(actual));
  }
}

EnvelopeLookupKey key() {
  return {7, SensorHealthMode::nominal, "survey-domain-v3", 11, 12};
}

CableUncertaintyEnvelope envelope() {
  CableUncertaintyEnvelope result;
  result.validity = EnvelopeBuildValidity::valid;
  result.dependencies.generator_version = 44;
  result.dependencies.cable_model_version = 11;
  result.dependencies.execution_operating_envelope_version = 12;
  result.dependencies.reference_line_version = 7;
  result.dependencies.operating_domain_version = 13;
  result.dependencies.primitive_set_version = 14;
  result.dependencies.initial_uncertainty_version = 15;
  result.dependencies.sensor_uncertainty_version = 16;
  result.dependencies.execution_uncertainty_version = 17;
  result.dependencies.margin_certification_version = 18;
  result.dependencies.sensor_mode = SensorHealthMode::nominal;
  result.dependencies.operating_domain_id = "survey-domain-v3";
  result.dependencies.cable_model_calibration_dataset_id = "model-cal-v4";
  result.dependencies.certification_dataset_id = "envelope-proof-v2";
  result.dependencies.sensor_calibration_dataset_id = "sensor-proof-v3";
  result.dependencies.execution_uncertainty_calibration_dataset_id =
      "execution-proof-v2";
  result.dependencies.margin_calibration_dataset_id = "margin-proof-v2";
  result.margin_budget.certification_version = 18;
  result.margin_budget.calibration_dataset_id = "margin-proof-v2";
  result.margin_budget.state_binning_stddev_m = 0.01;
  result.margin_budget.numerical_integration_stddev_m = 0.02;
  result.margin_budget.reference_normal_sweep_stddev_m = 0.03;
  result.margin_budget.statistical_quantile_stddev_m = 0.04;
  result.segments = {
      CableUncertaintyEnvelopeSegment{0.0, 1.0, 0.04, 0.30},
      CableUncertaintyEnvelopeSegment{1.0, 2.0, 0.16, 0.50},
      CableUncertaintyEnvelopeSegment{2.0, 3.0, 0.09, 0.40},
  };
  result.generation_timestamp = kTimestamp;
  result.path_joint_risk_implemented = false;
  result.risk_semantics =
      underwater_planner::core::kPointwiseEnvelopeRiskSemantics;
  result.diagnostics.push_back(
      {DiagnosticSeverity::info, "ENVELOPE_CERTIFIED",
       "cable_uncertainty_envelope_builder", "certified envelope",
       kTimestamp});
  return result;
}

EnvelopeCoverageCertification certification(
    const std::uint64_t envelope_version = 101) {
  return {21,
          "independent-envelope-audit-v2",
          true,
          kTimestamp,
          kValidUntil,
          envelope_version,
          envelope().dependencies};
}

void complete_tuple_is_required_before_an_envelope_can_be_locked() {
  // Design: 18.2.4-31
  CableUncertaintyEnvelopeManager manager;
  const auto registered =
      manager.registerValidated(101, envelope(), certification());
  require(registered.status == EnvelopeRegistrationStatus::accepted,
          "a certified envelope with complete dependencies was rejected");
  require(manager.setCurrentContext(key(), 1, kUseTime).context_changed,
          "the first complete context was not activated");

  const auto locked = manager.getValidated(key(), kUseTime);
  require(locked.has_value() && locked->envelope_version == 101 &&
              locked->envelope != nullptr &&
              locked->envelope->dependencies.generator_version == 44 &&
              locked->coverage_certification.valid_until.nanoseconds ==
                  kValidUntil.nanoseconds &&
              locked->coverage_certification.certified_envelope_version ==
                  locked->envelope_version,
          "the exact dependency tuple did not return an immutable envelope");

  EnvelopeLookupKey old_model = key();
  old_model.cable_model_version = 10;
  require(!manager.getValidated(old_model, kUseTime).has_value(),
          "the operating-domain id incorrectly hid a model-version mismatch");
  EnvelopeLookupKey old_execution = key();
  old_execution.execution_operating_envelope_version = 9;
  require(!manager.getValidated(old_execution, kUseTime).has_value(),
          "the operating-domain id incorrectly hid an execution-envelope mismatch");

  CableUncertaintyEnvelope incomplete = envelope();
  incomplete.dependencies.generator_version = 0;
  require(manager.registerValidated(102, incomplete, certification()).status ==
              EnvelopeRegistrationStatus::input_invalid,
          "an envelope with an incomplete generation tuple was published");
  EnvelopeCoverageCertification failed = certification(103);
  failed.passed = false;
  require(manager.registerValidated(103, envelope(), failed).status ==
              EnvelopeRegistrationStatus::coverage_audit_failed,
          "an envelope without passing independent coverage audit was published");
  require(manager.registerValidated(104, envelope(), certification(999)).status ==
              EnvelopeRegistrationStatus::input_invalid,
          "coverage evidence for another envelope version was accepted");
  EnvelopeCoverageCertification wrong_dependencies = certification(105);
  ++wrong_dependencies.certified_dependencies.cable_model_version;
  require(manager
              .registerValidated(105, envelope(), wrong_dependencies)
              .status == EnvelopeRegistrationStatus::input_invalid,
          "coverage evidence for another dependency tuple was accepted");
}

void progress_queries_use_adjacent_upper_bounds_and_certified_margins() {
  // Design: 18.2.4-16
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(101, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(manager.setCurrentContext(key(), 1, kUseTime));
  const auto locked = manager.getValidated(key(), kUseTime);
  require(locked.has_value(), "test envelope lock failed");

  const auto at_first_boundary = manager.query(*locked, 1.0, kUseTime);
  require(at_first_boundary.status == EnvelopeQueryStatus::valid,
          "a covered progress boundary was rejected");
  require_near(at_first_boundary.lateral_stddev_upper_bound_m, 0.50, 1.0e-12,
               "the query did not conservatively cover both adjacent segments");
  require_near(at_first_boundary.certified_discretization_margin_m, 0.06,
               1.0e-12,
               "the independent discretization margins were not auditable");
  require(at_first_boundary.adjacent_segment_count == 2,
          "the boundary query did not report both covering segments");

  const auto interior = manager.query(*locked, 2.5, kUseTime);
  require(interior.status == EnvelopeQueryStatus::valid &&
              interior.adjacent_segment_count == 2,
          "an interior query did not cover both adjacent progress endpoints");
  require_near(interior.lateral_stddev_upper_bound_m, 0.50, 1.0e-12,
               "an interior query underestimated an adjacent endpoint bound");
  require(manager.query(*locked, 3.01, kUseTime).status ==
              EnvelopeQueryStatus::progress_out_of_range,
          "progress outside the certified domain was extrapolated");

  CableUncertaintyEnvelope boundary_guard = envelope();
  boundary_guard.segments.front().lateral_variance_upper_bound_m2 = 0.81;
  boundary_guard.segments.front().lateral_stddev_upper_bound_m = 1.0;
  CableUncertaintyEnvelopeManager boundary_manager;
  require(boundary_manager
              .registerValidated(200, boundary_guard, certification(200))
              .accepted(),
          "the boundary-query envelope was rejected");
  static_cast<void>(
      boundary_manager.setCurrentContext(key(), 1, kUseTime));
  const auto boundary_locked =
      boundary_manager.getValidated(key(), kUseTime);
  require(boundary_locked.has_value(), "the boundary-query envelope did not lock");
  const auto exact_internal_boundary =
      boundary_manager.query(*boundary_locked, 2.0, kUseTime);
  require(exact_internal_boundary.adjacent_segment_count == 2,
          "an internal boundary consumed more than its two adjacent segments");
  require_near(exact_internal_boundary.lateral_stddev_upper_bound_m, 0.50,
               1.0e-12,
               "a non-adjacent segment contaminated an internal boundary");

  CableUncertaintyEnvelope margin_only = envelope();
  margin_only.segments.front().lateral_stddev_upper_bound_m = 0.20;
  CableUncertaintyEnvelopeManager margin_manager;
  require(margin_manager
              .registerValidated(201, margin_only, certification(201))
              .accepted(),
          "a structurally valid envelope was rejected before conservative query");
  static_cast<void>(margin_manager.setCurrentContext(key(), 1, kUseTime));
  const auto margin_locked = margin_manager.getValidated(key(), kUseTime);
  require(margin_locked.has_value(), "the margin-query envelope did not lock");
  require_near(
      margin_manager.query(*margin_locked, 0.5, kUseTime)
          .lateral_stddev_upper_bound_m,
      0.30, 1.0e-12,
      "the manager failed to add all independently certified margins");
}

void context_changes_atomically_invalidate_envelope_plan_and_lease() {
  // Design: 18.2.4-18
  // Design: 18.2.4-32
  std::vector<EnvelopeLookupKey> changed_keys;
  EnvelopeLookupKey changed = key();
  changed.reference_line_version = 8;
  changed_keys.push_back(changed);
  changed = key();
  changed.sensor_mode = SensorHealthMode::approved_degraded;
  changed_keys.push_back(changed);
  changed = key();
  changed.operating_domain_id = "survey-domain-v4";
  changed_keys.push_back(changed);
  changed = key();
  changed.cable_model_version = 19;
  changed_keys.push_back(changed);
  changed = key();
  changed.execution_operating_envelope_version = 20;
  changed_keys.push_back(changed);

  for (std::size_t index = 0; index < changed_keys.size(); ++index) {
    CableUncertaintyEnvelopeManager manager;
    require(
        manager.registerValidated(101, envelope(), certification()).accepted(),
        "test envelope registration failed");
    static_cast<void>(manager.setCurrentContext(key(), 1, kUseTime));
    const auto locked = manager.getValidated(key(), kUseTime);
    require(locked.has_value(), "test envelope lock failed");
    require(manager.registerDependentPlan(701, *locked, kUseTime) &&
                manager.registerDependentLease(801, 701, *locked, kUseTime),
            "valid plan and lease dependencies were rejected");

    const auto invalidation = manager.setCurrentContext(
        changed_keys[index], 2,
        MonotonicTime{9'100'000'000 + static_cast<std::int64_t>(index)});
    require(invalidation.context_changed &&
                invalidation.invalidated_envelope_count == 1 &&
                invalidation.invalidated_plan_count == 1 &&
                invalidation.revoked_lease_count == 1,
            "a context tuple change did not atomically invalidate all dependants");
    require(
        manager.envelopeStatus(101) == DependentArtifactStatus::invalidated &&
            manager.planStatus(701) == DependentArtifactStatus::invalidated &&
            manager.leaseStatus(801) == DependentArtifactStatus::invalidated,
        "dependant status escaped the context-change transaction");
    require(!manager.getValidated(key(), kUseTime).has_value() &&
                !manager.getValidated(changed_keys[index], kUseTime)
                     .has_value(),
            "an old or uncertified new-context envelope remained lockable");
    require(manager.query(*locked, 1.0, kUseTime).status ==
                EnvelopeQueryStatus::envelope_invalidated,
            "an immutable snapshot remained authorized after invalidation");
  }
}

void a_new_envelope_version_revokes_old_dependants_before_activation() {
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(101, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(manager.setCurrentContext(key(), 1, kUseTime));
  const auto old_lock = manager.getValidated(key(), kUseTime);
  require(old_lock.has_value() &&
              manager.registerDependentPlan(703, *old_lock, kUseTime) &&
              manager.registerDependentLease(803, 703, *old_lock, kUseTime),
          "old envelope dependencies were not established");

  EnvelopeCoverageCertification newer_certification = certification(102);
  newer_certification.version = 22;
  newer_certification.audited_at = MonotonicTime{9'400'000'000};
  require(manager
              .registerValidated(102, envelope(), newer_certification)
              .accepted(),
          "a newer audited envelope was rejected");
  const auto new_lock = manager.getValidated(key(),
                                             MonotonicTime{9'500'000'000});
  require(new_lock.has_value() && new_lock->envelope_version == 102,
          "the newest matching envelope was not activated");
  require(manager.envelopeStatus(101) == DependentArtifactStatus::invalidated &&
              manager.planStatus(703) == DependentArtifactStatus::invalidated &&
              manager.leaseStatus(803) == DependentArtifactStatus::invalidated,
          "an envelope-version change left old plan authorization active");
  require(manager.query(*old_lock, 1.0, MonotonicTime{9'500'000'000}).status ==
              EnvelopeQueryStatus::envelope_invalidated,
          "an old envelope lock survived version replacement");
}

void covariance_breach_has_system_failure_semantics() {
  // Design: 18.2.4-17
  CableUncertaintyEnvelopeManager manager;
  require(manager.registerValidated(101, envelope(), certification()).accepted(),
          "test envelope registration failed");
  static_cast<void>(manager.setCurrentContext(key(), 1, kUseTime));
  const auto locked = manager.getValidated(key(), kUseTime);
  require(locked.has_value() &&
              manager.registerDependentPlan(702, *locked, kUseTime) &&
              manager.registerDependentLease(802, 702, *locked, kUseTime),
          "test dependencies were not registered");

  const auto pass = manager.auditActualLateralStddev(
      *locked, 1.0, 0.509, 0.01, MonotonicTime{9'200'000'000});
  require(pass.status == EnvelopeAuditStatus::pass &&
              manager.leaseStatus(802) == DependentArtifactStatus::active,
          "a value within the explicit audit tolerance caused invalidation");
  const auto breach = manager.auditActualLateralStddev(
      *locked, 1.0,
      std::nextafter(0.51, std::numeric_limits<double>::infinity()), 0.01,
      MonotonicTime{9'300'000'000});
  require(breach.status == EnvelopeAuditStatus::covariance_envelope_breach &&
              breach.planning_state ==
                  underwater_planner::core::PlanningState::
                      covariance_envelope_breach &&
              breach.stop_required && breach.invalidated_envelope_count == 1 &&
              breach.invalidated_plan_count == 1 &&
              breach.revoked_lease_count == 1,
          "a covariance breach did not return the required system failure");
  require(manager.envelopeStatus(101) == DependentArtifactStatus::invalidated &&
              manager.planStatus(702) == DependentArtifactStatus::invalidated &&
              manager.leaseStatus(802) == DependentArtifactStatus::invalidated,
          "a covariance breach did not atomically revoke all dependencies");
  require(!breach.diagnostics.empty() &&
              breach.diagnostics.back().code ==
                  "COVARIANCE_ENVELOPE_BREACH" &&
              breach.diagnostics.back().timestamp.nanoseconds ==
                  9'300'000'000,
          "the breach was not auditable at the failure timestamp");
}

void expiry_and_context_regression_fail_closed() {
  CableUncertaintyEnvelopeManager manager;
  EnvelopeCoverageCertification short_lived = certification();
  short_lived.valid_until = MonotonicTime{9'500'000'000};
  require(manager.registerValidated(101, envelope(), short_lived).accepted(),
          "a currently valid short-lived certification was rejected");
  require(manager.setCurrentContext(key(), 10, kUseTime)
              .context_update_status == EnvelopeContextUpdateStatus::accepted,
          "the initial sequenced context was rejected");
  const auto locked = manager.getValidated(key(), kUseTime);
  require(locked.has_value() &&
              manager.registerDependentPlan(704, *locked, kUseTime) &&
              manager.registerDependentLease(804, 704, *locked, kUseTime),
          "expiry test dependencies were not established");

  EnvelopeLookupKey newer_model = key();
  newer_model.cable_model_version = 12;
  const auto stale_sequence = manager.setCurrentContext(
      newer_model, 9, MonotonicTime{9'200'000'000});
  require(stale_sequence.context_update_status ==
              EnvelopeContextUpdateStatus::stale &&
              !stale_sequence.context_changed &&
              manager.envelopeStatus(101) == DependentArtifactStatus::active,
          "an out-of-order context update replaced current state");

  EnvelopeLookupKey regressed_version = key();
  regressed_version.cable_model_version = 10;
  const auto regression = manager.setCurrentContext(
      regressed_version, 11, MonotonicTime{9'300'000'000});
  require(regression.context_update_status ==
              EnvelopeContextUpdateStatus::stale &&
              manager.envelopeStatus(101) == DependentArtifactStatus::active,
          "a newer message carrying a regressed model version was accepted");

  const auto expired = manager.expire(MonotonicTime{9'500'000'000});
  require(expired.invalidated_envelope_count == 1 &&
              expired.invalidated_plan_count == 1 &&
              expired.revoked_lease_count == 1 &&
              !manager
                   .getValidated(key(), MonotonicTime{9'500'000'000})
                   .has_value(),
          "certification expiry did not revoke the envelope and dependants");
}

}  // namespace

int main() {
  constexpr std::uint64_t kSeed = 1919;
  try {
    complete_tuple_is_required_before_an_envelope_can_be_locked();
    progress_queries_use_adjacent_upper_bounds_and_certified_margins();
    context_changes_atomically_invalidate_envelope_plan_and_lease();
    a_new_envelope_version_revokes_old_dependants_before_activation();
    covariance_breach_has_system_failure_semantics();
    expiry_and_context_regression_fail_closed();
    std::cout << "cable uncertainty envelope manager checks passed: 6"
              << " seed=" << kSeed
              << " input_version=t19-envelope-manager/v1"
              << " units=SI timestamp=monotonic-ns"
              << " domain=survey-domain-v3"
              << " risk=pointwise-envelope-only\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "cable uncertainty envelope manager failure: " << error.what()
              << " seed=" << kSeed
              << " input_version=t19-envelope-manager/v1"
              << " units=SI timestamp=monotonic-ns"
              << " domain=survey-domain-v3"
              << " risk=pointwise-envelope-only\n";
    return 1;
  }
}
