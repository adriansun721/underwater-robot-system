#include "underwater_planner/core/cable_uncertainty_envelope_manager.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace underwater_planner::core {
namespace {

constexpr double kProgressToleranceM = 1.0e-12;

[[nodiscard]] bool finite_nonnegative(const double value) noexcept {
  return std::isfinite(value) && value >= 0.0;
}

[[nodiscard]] bool known_sensor_mode(const SensorHealthMode mode) noexcept {
  return mode == SensorHealthMode::nominal ||
         mode == SensorHealthMode::approved_degraded;
}

[[nodiscard]] EnvelopeLookupKey key_from(
    const CableUncertaintyEnvelope& envelope) {
  const EnvelopeDependencies& dependencies = envelope.dependencies;
  return {dependencies.reference_line_version, dependencies.sensor_mode,
          dependencies.operating_domain_id,
          dependencies.cable_model_version,
          dependencies.execution_operating_envelope_version};
}

[[nodiscard]] bool complete_dependencies(
    const EnvelopeDependencies& dependencies) noexcept {
  return dependencies.generator_version != 0U &&
         dependencies.cable_model_version != 0U &&
         dependencies.execution_operating_envelope_version != 0U &&
         dependencies.reference_line_version != 0U &&
         dependencies.operating_domain_version != 0U &&
         dependencies.primitive_set_version != 0U &&
         dependencies.initial_uncertainty_version != 0U &&
         dependencies.sensor_uncertainty_version != 0U &&
         dependencies.execution_uncertainty_version != 0U &&
         dependencies.margin_certification_version != 0U &&
         known_sensor_mode(dependencies.sensor_mode) &&
         !dependencies.operating_domain_id.empty() &&
         !dependencies.cable_model_calibration_dataset_id.empty() &&
         !dependencies.certification_dataset_id.empty() &&
         !dependencies.sensor_calibration_dataset_id.empty() &&
         !dependencies.execution_uncertainty_calibration_dataset_id.empty() &&
         !dependencies.margin_calibration_dataset_id.empty();
}

[[nodiscard]] bool same_dependencies(
    const EnvelopeDependencies& left,
    const EnvelopeDependencies& right) noexcept {
  return left.generator_version == right.generator_version &&
         left.cable_model_version == right.cable_model_version &&
         left.execution_operating_envelope_version ==
             right.execution_operating_envelope_version &&
         left.reference_line_version == right.reference_line_version &&
         left.operating_domain_version == right.operating_domain_version &&
         left.primitive_set_version == right.primitive_set_version &&
         left.initial_uncertainty_version ==
             right.initial_uncertainty_version &&
         left.sensor_uncertainty_version == right.sensor_uncertainty_version &&
         left.execution_uncertainty_version ==
             right.execution_uncertainty_version &&
         left.margin_certification_version ==
             right.margin_certification_version &&
         left.sensor_mode == right.sensor_mode &&
         left.operating_domain_id == right.operating_domain_id &&
         left.cable_model_calibration_dataset_id ==
             right.cable_model_calibration_dataset_id &&
         left.certification_dataset_id == right.certification_dataset_id &&
         left.sensor_calibration_dataset_id ==
             right.sensor_calibration_dataset_id &&
         left.execution_uncertainty_calibration_dataset_id ==
             right.execution_uncertainty_calibration_dataset_id &&
         left.margin_calibration_dataset_id ==
             right.margin_calibration_dataset_id;
}

[[nodiscard]] bool valid_margin_budget(
    const EnvelopeMarginBudget& margin,
    const EnvelopeDependencies& dependencies) noexcept {
  return margin.certification_version != 0U &&
         margin.certification_version ==
             dependencies.margin_certification_version &&
         !margin.calibration_dataset_id.empty() &&
         margin.calibration_dataset_id ==
             dependencies.margin_calibration_dataset_id &&
         finite_nonnegative(margin.state_binning_stddev_m) &&
         finite_nonnegative(margin.numerical_integration_stddev_m) &&
         finite_nonnegative(margin.reference_normal_sweep_stddev_m) &&
         finite_nonnegative(margin.statistical_quantile_stddev_m);
}

[[nodiscard]] bool valid_segments(
    const CableUncertaintyEnvelope& envelope) noexcept {
  if (envelope.segments.empty()) return false;
  double previous_end = envelope.segments.front().reference_progress_start_m;
  for (const CableUncertaintyEnvelopeSegment& segment : envelope.segments) {
    if (!std::isfinite(segment.reference_progress_start_m) ||
        !std::isfinite(segment.reference_progress_end_m) ||
        segment.reference_progress_start_m >= segment.reference_progress_end_m ||
        !finite_nonnegative(segment.lateral_variance_upper_bound_m2) ||
        !finite_nonnegative(segment.lateral_stddev_upper_bound_m) ||
        segment.lateral_stddev_upper_bound_m + kProgressToleranceM <
            std::sqrt(segment.lateral_variance_upper_bound_m2) ||
        std::abs(segment.reference_progress_start_m - previous_end) >
            kProgressToleranceM) {
      return false;
    }
    previous_end = segment.reference_progress_end_m;
  }
  return true;
}

[[nodiscard]] bool valid_envelope(
    const CableUncertaintyEnvelope& envelope) noexcept {
  return envelope.validity == EnvelopeBuildValidity::valid &&
         complete_dependencies(envelope.dependencies) &&
         valid_margin_budget(envelope.margin_budget, envelope.dependencies) &&
         valid_segments(envelope) &&
         envelope.generation_timestamp.nanoseconds >= 0 &&
         !envelope.path_joint_risk_implemented &&
         envelope.risk_semantics == kPointwiseEnvelopeRiskSemantics;
}

[[nodiscard]] DiagnosticEntry diagnostic(
    const DiagnosticSeverity severity, std::string code, std::string message,
    const MonotonicTime timestamp) {
  return {severity, std::move(code), "cable_uncertainty_envelope_manager",
          std::move(message), timestamp};
}

void merge_invalidation(EnvelopeInvalidationResult& destination,
                        const EnvelopeInvalidationResult& source) {
  destination.invalidated_envelope_count +=
      source.invalidated_envelope_count;
  destination.invalidated_plan_count += source.invalidated_plan_count;
  destination.revoked_lease_count += source.revoked_lease_count;
  destination.diagnostics.insert(destination.diagnostics.end(),
                                 source.diagnostics.begin(),
                                 source.diagnostics.end());
}

[[nodiscard]] bool versions_regress(const EnvelopeLookupKey& current,
                                    const EnvelopeLookupKey& next) noexcept {
  return next.reference_line_version < current.reference_line_version ||
         next.cable_model_version < current.cable_model_version ||
         next.execution_operating_envelope_version <
             current.execution_operating_envelope_version;
}

}  // namespace

bool operator==(const EnvelopeLookupKey& left,
                const EnvelopeLookupKey& right) noexcept {
  return left.reference_line_version == right.reference_line_version &&
         left.sensor_mode == right.sensor_mode &&
         left.operating_domain_id == right.operating_domain_id &&
         left.cable_model_version == right.cable_model_version &&
         left.execution_operating_envelope_version ==
             right.execution_operating_envelope_version;
}

bool operator!=(const EnvelopeLookupKey& left,
                const EnvelopeLookupKey& right) noexcept {
  return !(left == right);
}

EnvelopeRegistrationResult
CableUncertaintyEnvelopeManager::registerValidated(
    const std::uint64_t envelope_version, CableUncertaintyEnvelope envelope,
    EnvelopeCoverageCertification coverage_certification) {
  std::lock_guard<std::mutex> lock(mutex_);
  EnvelopeRegistrationResult result;
  const MonotonicTime timestamp = coverage_certification.audited_at;
  if (envelope_version == 0U || !valid_envelope(envelope) ||
      coverage_certification.version == 0U ||
      coverage_certification.calibration_dataset_id.empty() ||
      coverage_certification.certified_envelope_version != envelope_version ||
      !same_dependencies(coverage_certification.certified_dependencies,
                         envelope.dependencies) ||
      coverage_certification.audited_at.nanoseconds <
          envelope.generation_timestamp.nanoseconds ||
      coverage_certification.valid_until.nanoseconds <=
          coverage_certification.audited_at.nanoseconds) {
    result.status = EnvelopeRegistrationStatus::input_invalid;
    result.diagnostics.push_back(diagnostic(
        DiagnosticSeverity::error, "ENVELOPE_REGISTRATION_INVALID",
        "a complete certified envelope and timestamped coverage audit are required",
        timestamp));
    return result;
  }
  if (!coverage_certification.passed) {
    result.status = EnvelopeRegistrationStatus::coverage_audit_failed;
    result.diagnostics.push_back(diagnostic(
        DiagnosticSeverity::error, "ENVELOPE_COVERAGE_AUDIT_FAILED",
        "an envelope that failed independent coverage audit cannot be published",
        timestamp));
    return result;
  }
  if (envelopes_.find(envelope_version) != envelopes_.end()) {
    result.status = EnvelopeRegistrationStatus::duplicate;
    result.diagnostics.push_back(diagnostic(
        DiagnosticSeverity::error, "ENVELOPE_VERSION_DUPLICATE",
        "an envelope version can only be registered once", timestamp));
    return result;
  }
  if (envelope_version <= last_registered_version_) {
    result.status = EnvelopeRegistrationStatus::version_rollback;
    result.diagnostics.push_back(diagnostic(
        DiagnosticSeverity::error, "ENVELOPE_VERSION_ROLLBACK",
        "envelope versions must increase monotonically", timestamp));
    return result;
  }

  const EnvelopeLookupKey envelope_key = key_from(envelope);
  if (current_context_.has_value() && *current_context_ == envelope_key) {
    std::vector<std::uint64_t> replaced_versions;
    for (const auto& entry : envelopes_) {
      if (entry.second.status == DependentArtifactStatus::active &&
          entry.second.key == envelope_key) {
        replaced_versions.push_back(entry.first);
      }
    }
    for (const std::uint64_t replaced_version : replaced_versions) {
      static_cast<void>(invalidateLocked(
          replaced_version, timestamp, "ENVELOPE_VERSION_REPLACED",
          "a newer envelope version replaced the active version tuple"));
    }
  }

  EnvelopeRecord record;
  record.key = envelope_key;
  record.envelope =
      std::make_shared<const CableUncertaintyEnvelope>(std::move(envelope));
  record.coverage_certification = std::move(coverage_certification);
  record.authorization_generation = next_authorization_generation_++;
  envelopes_.emplace(envelope_version, std::move(record));
  last_registered_version_ = envelope_version;
  result.status = EnvelopeRegistrationStatus::accepted;
  result.diagnostics.push_back(diagnostic(
      DiagnosticSeverity::info, "ENVELOPE_REGISTERED",
      "coverage-audited envelope registered as an immutable snapshot",
      timestamp));
  return result;
}

EnvelopeInvalidationResult CableUncertaintyEnvelopeManager::setCurrentContext(
    const EnvelopeLookupKey& context, const std::uint64_t context_sequence,
    const MonotonicTime changed_at) {
  std::lock_guard<std::mutex> lock(mutex_);
  EnvelopeInvalidationResult result;
  if (context.reference_line_version == 0U ||
      context.cable_model_version == 0U ||
      context.execution_operating_envelope_version == 0U ||
      context.operating_domain_id.empty() ||
      !known_sensor_mode(context.sensor_mode) || context_sequence == 0U ||
      changed_at.nanoseconds < 0) {
    result.context_update_status =
        EnvelopeContextUpdateStatus::input_invalid;
    result.diagnostics.push_back(diagnostic(
        DiagnosticSeverity::error, "ENVELOPE_CONTEXT_INVALID",
        "the runtime envelope context requires a complete version tuple",
        changed_at));
    return result;
  }
  merge_invalidation(result, expireLocked(changed_at));
  if (current_context_.has_value()) {
    if (context_sequence < current_context_sequence_ ||
        (context_sequence == current_context_sequence_ &&
         *current_context_ != context) ||
        (context_sequence > current_context_sequence_ &&
         versions_regress(*current_context_, context))) {
      result.context_update_status = EnvelopeContextUpdateStatus::stale;
      result.diagnostics.push_back(diagnostic(
          DiagnosticSeverity::error, "ENVELOPE_CONTEXT_STALE",
          "a stale sequence or regressed dependency version cannot replace the current context",
          changed_at));
      return result;
    }
    if (context_sequence == current_context_sequence_) {
      result.context_update_status = EnvelopeContextUpdateStatus::idempotent;
      return result;
    }
  }

  result.context_update_status = EnvelopeContextUpdateStatus::accepted;
  const bool context_changed =
      !current_context_.has_value() || *current_context_ != context;
  current_context_sequence_ = context_sequence;
  if (!context_changed) return result;

  result.context_changed = true;
  current_context_ = context;
  std::vector<std::uint64_t> invalid_versions;
  for (const auto& entry : envelopes_) {
    if (entry.second.status == DependentArtifactStatus::active &&
        entry.second.key != context) {
      invalid_versions.push_back(entry.first);
    }
  }
  for (const std::uint64_t version : invalid_versions) {
    const EnvelopeInvalidationResult invalidated = invalidateLocked(
        version, changed_at, "ENVELOPE_CONTEXT_CHANGED",
        "a dependency version, sensor mode, or operating domain changed");
    merge_invalidation(result, invalidated);
  }
  return result;
}

std::optional<LockedCableUncertaintyEnvelope>
CableUncertaintyEnvelopeManager::getValidated(
    const EnvelopeLookupKey& key, const MonotonicTime now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!prepareForUseLocked(now)) return std::nullopt;
  if (!current_context_.has_value() || *current_context_ != key) {
    return std::nullopt;
  }
  const EnvelopeRecord* newest = nullptr;
  std::uint64_t newest_version = 0U;
  for (const auto& entry : envelopes_) {
    if (entry.second.status == DependentArtifactStatus::active &&
        entry.second.key == key && entry.first > newest_version) {
      newest_version = entry.first;
      newest = &entry.second;
    }
  }
  if (newest == nullptr) return std::nullopt;
  return LockedCableUncertaintyEnvelope{
      newest_version, newest->authorization_generation, newest->envelope,
      newest->coverage_certification};
}

bool CableUncertaintyEnvelopeManager::isCurrentLocked(
    const LockedCableUncertaintyEnvelope& locked) const {
  const auto found = envelopes_.find(locked.envelope_version);
  return found != envelopes_.end() &&
         found->second.status == DependentArtifactStatus::active &&
         found->second.authorization_generation ==
             locked.authorization_generation &&
         found->second.envelope == locked.envelope &&
         current_context_.has_value() &&
         found->second.key == *current_context_;
}

EnvelopeQueryResult CableUncertaintyEnvelopeManager::queryLocked(
    const LockedCableUncertaintyEnvelope& locked,
    const double reference_progress_m) const {
  EnvelopeQueryResult result;
  if (!isCurrentLocked(locked)) {
    result.status = EnvelopeQueryStatus::envelope_invalidated;
    result.diagnostics.push_back(diagnostic(
        DiagnosticSeverity::error, "ENVELOPE_QUERY_INVALIDATED",
        "the locked envelope is no longer authorized by the current context",
        locked.envelope == nullptr ? MonotonicTime{}
                                   : locked.envelope->generation_timestamp));
    return result;
  }
  if (!std::isfinite(reference_progress_m)) {
    result.status = EnvelopeQueryStatus::input_invalid;
    result.diagnostics.push_back(diagnostic(
        DiagnosticSeverity::error, "ENVELOPE_QUERY_INPUT_INVALID",
        "reference progress must be finite",
        locked.envelope->generation_timestamp));
    return result;
  }

  const CableUncertaintyEnvelope& envelope = *locked.envelope;
  std::vector<bool> selected(envelope.segments.size(), false);
  std::vector<std::size_t> containing_segments;
  for (std::size_t index = 0U; index < envelope.segments.size(); ++index) {
    const CableUncertaintyEnvelopeSegment& segment = envelope.segments[index];
    if (reference_progress_m >=
            segment.reference_progress_start_m - kProgressToleranceM &&
        reference_progress_m <=
            segment.reference_progress_end_m + kProgressToleranceM) {
      containing_segments.push_back(index);
    }
  }
  if (containing_segments.size() >= 2U) {
    selected[containing_segments.front()] = true;
    selected[containing_segments.back()] = true;
  } else if (containing_segments.size() == 1U) {
    const std::size_t index = containing_segments.front();
    selected[index] = true;
    if (index > 0U) selected[index - 1U] = true;
  }
  double upper_bound_m = 0.0;
  for (std::size_t index = 0U; index < envelope.segments.size(); ++index) {
    if (!selected[index]) continue;
    const CableUncertaintyEnvelopeSegment& segment = envelope.segments[index];
    const double reconstructed_upper_bound_m =
        std::sqrt(segment.lateral_variance_upper_bound_m2) +
        total_envelope_margin_stddev_m(envelope.margin_budget);
    upper_bound_m = std::max(
        upper_bound_m,
        std::max(segment.lateral_stddev_upper_bound_m,
                 reconstructed_upper_bound_m));
    ++result.adjacent_segment_count;
  }
  if (result.adjacent_segment_count == 0U) {
    result.status = EnvelopeQueryStatus::progress_out_of_range;
    result.diagnostics.push_back(diagnostic(
        DiagnosticSeverity::error, "ENVELOPE_PROGRESS_OUT_OF_RANGE",
        "reference progress is outside the certified envelope domain",
        envelope.generation_timestamp));
    return result;
  }

  result.status = EnvelopeQueryStatus::valid;
  result.lateral_stddev_upper_bound_m = upper_bound_m;
  result.certified_discretization_margin_m =
      envelope_discretization_margin_stddev_m(envelope.margin_budget);
  return result;
}

EnvelopeQueryResult CableUncertaintyEnvelopeManager::query(
    const LockedCableUncertaintyEnvelope& locked,
    const double reference_progress_m, const MonotonicTime now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (now.nanoseconds < 0) {
    EnvelopeQueryResult result;
    result.status = EnvelopeQueryStatus::input_invalid;
    result.diagnostics.push_back(diagnostic(
        DiagnosticSeverity::error, "ENVELOPE_QUERY_TIME_INVALID",
        "query time must be monotonic", now));
    return result;
  }
  static_cast<void>(prepareForUseLocked(now));
  return queryLocked(locked, reference_progress_m);
}

bool CableUncertaintyEnvelopeManager::registerDependentPlan(
    const std::uint64_t plan_sequence,
    const LockedCableUncertaintyEnvelope& locked, const MonotonicTime now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!prepareForUseLocked(now)) return false;
  if (plan_sequence == 0U || !isCurrentLocked(locked) ||
      plans_.find(plan_sequence) != plans_.end()) {
    return false;
  }
  plans_.emplace(plan_sequence,
                 PlanRecord{locked.envelope_version,
                            locked.authorization_generation,
                            DependentArtifactStatus::active});
  return true;
}

bool CableUncertaintyEnvelopeManager::registerDependentLease(
    const std::uint64_t lease_sequence, const std::uint64_t plan_sequence,
    const LockedCableUncertaintyEnvelope& locked, const MonotonicTime now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!prepareForUseLocked(now)) return false;
  const auto plan = plans_.find(plan_sequence);
  if (lease_sequence == 0U || !isCurrentLocked(locked) ||
      leases_.find(lease_sequence) != leases_.end() || plan == plans_.end() ||
      plan->second.status != DependentArtifactStatus::active ||
      plan->second.envelope_version != locked.envelope_version ||
      plan->second.envelope_generation != locked.authorization_generation) {
    return false;
  }
  leases_.emplace(lease_sequence,
                  LeaseRecord{plan_sequence, locked.envelope_version,
                              locked.authorization_generation,
                              DependentArtifactStatus::active});
  return true;
}

EnvelopeInvalidationResult CableUncertaintyEnvelopeManager::invalidateLocked(
    const std::uint64_t envelope_version,
    const MonotonicTime invalidated_at, std::string diagnostic_code,
    std::string diagnostic_message) {
  EnvelopeInvalidationResult result;
  const auto envelope = envelopes_.find(envelope_version);
  if (envelope == envelopes_.end() ||
      envelope->second.status != DependentArtifactStatus::active) {
    return result;
  }
  envelope->second.status = DependentArtifactStatus::invalidated;
  ++result.invalidated_envelope_count;

  for (auto& plan : plans_) {
    if (plan.second.status == DependentArtifactStatus::active &&
        plan.second.envelope_version == envelope_version &&
        plan.second.envelope_generation ==
            envelope->second.authorization_generation) {
      plan.second.status = DependentArtifactStatus::invalidated;
      ++result.invalidated_plan_count;
    }
  }
  for (auto& lease : leases_) {
    if (lease.second.status == DependentArtifactStatus::active &&
        lease.second.envelope_version == envelope_version &&
        lease.second.envelope_generation ==
            envelope->second.authorization_generation) {
      lease.second.status = DependentArtifactStatus::invalidated;
      ++result.revoked_lease_count;
    }
  }
  result.diagnostics.push_back(
      diagnostic(DiagnosticSeverity::error, std::move(diagnostic_code),
                 std::move(diagnostic_message), invalidated_at));
  return result;
}

EnvelopeInvalidationResult CableUncertaintyEnvelopeManager::invalidate(
    const std::uint64_t envelope_version,
    const MonotonicTime invalidated_at) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (invalidated_at.nanoseconds < 0) {
    EnvelopeInvalidationResult result;
    result.diagnostics.push_back(diagnostic(
        DiagnosticSeverity::error, "ENVELOPE_INVALIDATION_TIME_INVALID",
        "invalidation time must be monotonic", invalidated_at));
    return result;
  }
  return invalidateLocked(envelope_version, invalidated_at,
                          "ENVELOPE_INVALIDATED",
                          "the envelope and every dependent artifact were invalidated");
}

EnvelopeInvalidationResult CableUncertaintyEnvelopeManager::expireLocked(
    const MonotonicTime now) {
  EnvelopeInvalidationResult result;
  std::vector<std::uint64_t> expired_versions;
  for (const auto& entry : envelopes_) {
    if (entry.second.status == DependentArtifactStatus::active &&
        entry.second.coverage_certification.valid_until.nanoseconds <=
            now.nanoseconds) {
      expired_versions.push_back(entry.first);
    }
  }
  for (const std::uint64_t version : expired_versions) {
    merge_invalidation(
        result,
        invalidateLocked(version, now, "ENVELOPE_CERTIFICATION_EXPIRED",
                         "the envelope coverage certification expired"));
  }
  return result;
}

bool CableUncertaintyEnvelopeManager::prepareForUseLocked(
    const MonotonicTime now) {
  if (now.nanoseconds < 0) return false;
  static_cast<void>(expireLocked(now));
  return true;
}

EnvelopeInvalidationResult CableUncertaintyEnvelopeManager::expire(
    const MonotonicTime now) {
  std::lock_guard<std::mutex> lock(mutex_);
  EnvelopeInvalidationResult result;
  if (now.nanoseconds < 0) {
    result.diagnostics.push_back(diagnostic(
        DiagnosticSeverity::error, "ENVELOPE_EXPIRY_TIME_INVALID",
        "expiry check time must be monotonic", now));
    return result;
  }
  return expireLocked(now);
}

EnvelopeAuditResult
CableUncertaintyEnvelopeManager::auditActualLateralStddev(
    const LockedCableUncertaintyEnvelope& locked,
    const double reference_progress_m,
    const double actual_lateral_stddev_m, const double audit_tolerance_m,
    const MonotonicTime audited_at) {
  std::lock_guard<std::mutex> lock(mutex_);
  EnvelopeAuditResult result;
  result.actual_lateral_stddev_m = actual_lateral_stddev_m;
  if (!finite_nonnegative(actual_lateral_stddev_m) ||
      !finite_nonnegative(audit_tolerance_m) || audited_at.nanoseconds < 0) {
    result.status = EnvelopeAuditStatus::input_invalid;
    result.diagnostics.push_back(diagnostic(
        DiagnosticSeverity::error, "ENVELOPE_AUDIT_INPUT_INVALID",
        "actual standard deviation, tolerance, and timestamp must be valid",
        audited_at));
    return result;
  }

  static_cast<void>(prepareForUseLocked(audited_at));

  const EnvelopeQueryResult bound = queryLocked(locked, reference_progress_m);
  if (bound.status != EnvelopeQueryStatus::valid) {
    result.status = EnvelopeAuditStatus::envelope_unavailable;
    result.diagnostics.push_back(diagnostic(
        DiagnosticSeverity::error, "ENVELOPE_AUDIT_UNAVAILABLE",
        "the locked envelope is invalidated or does not cover the requested progress",
        audited_at));
    return result;
  }
  result.allowed_lateral_stddev_m =
      bound.lateral_stddev_upper_bound_m + audit_tolerance_m;
  if (actual_lateral_stddev_m <= result.allowed_lateral_stddev_m) {
    result.status = EnvelopeAuditStatus::pass;
    result.planning_state = PlanningState::path_valid;
    result.diagnostics.push_back(diagnostic(
        DiagnosticSeverity::info, "COVARIANCE_ENVELOPE_AUDIT_PASSED",
        "actual lateral standard deviation is within the certified envelope",
        audited_at));
    return result;
  }

  const EnvelopeInvalidationResult invalidated = invalidateLocked(
      locked.envelope_version, audited_at, "COVARIANCE_ENVELOPE_BREACH",
      "actual lateral standard deviation exceeded the envelope plus audit tolerance");
  result.status = EnvelopeAuditStatus::covariance_envelope_breach;
  result.planning_state = PlanningState::covariance_envelope_breach;
  result.stop_required = true;
  result.invalidated_envelope_count =
      invalidated.invalidated_envelope_count;
  result.invalidated_plan_count = invalidated.invalidated_plan_count;
  result.revoked_lease_count = invalidated.revoked_lease_count;
  result.diagnostics = invalidated.diagnostics;
  return result;
}

DependentArtifactStatus CableUncertaintyEnvelopeManager::envelopeStatus(
    const std::uint64_t envelope_version) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = envelopes_.find(envelope_version);
  return found == envelopes_.end() ? DependentArtifactStatus::unknown
                                   : found->second.status;
}

DependentArtifactStatus CableUncertaintyEnvelopeManager::planStatus(
    const std::uint64_t plan_sequence) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = plans_.find(plan_sequence);
  return found == plans_.end() ? DependentArtifactStatus::unknown
                               : found->second.status;
}

DependentArtifactStatus CableUncertaintyEnvelopeManager::leaseStatus(
    const std::uint64_t lease_sequence) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = leases_.find(lease_sequence);
  return found == leases_.end() ? DependentArtifactStatus::unknown
                                : found->second.status;
}

}  // namespace underwater_planner::core
