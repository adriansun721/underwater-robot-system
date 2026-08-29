#pragma once

#include "underwater_planner/core/cable_uncertainty_envelope_builder.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace underwater_planner::core {

struct EnvelopeLookupKey {
  std::uint32_t reference_line_version{};
  SensorHealthMode sensor_mode{SensorHealthMode::nominal};
  std::string operating_domain_id;
  std::uint64_t cable_model_version{};
  std::uint64_t execution_operating_envelope_version{};
};

[[nodiscard]] bool operator==(const EnvelopeLookupKey& left,
                              const EnvelopeLookupKey& right) noexcept;
[[nodiscard]] bool operator!=(const EnvelopeLookupKey& left,
                              const EnvelopeLookupKey& right) noexcept;

struct EnvelopeCoverageCertification {
  std::uint64_t version{};
  std::string calibration_dataset_id;
  bool passed{};
  MonotonicTime audited_at;
  MonotonicTime valid_until;
  std::uint64_t certified_envelope_version{};
  EnvelopeDependencies certified_dependencies;
};

enum class EnvelopeRegistrationStatus {
  accepted,
  input_invalid,
  coverage_audit_failed,
  duplicate,
  version_rollback,
};

struct EnvelopeRegistrationResult {
  EnvelopeRegistrationStatus status{EnvelopeRegistrationStatus::input_invalid};
  std::vector<DiagnosticEntry> diagnostics;

  [[nodiscard]] bool accepted() const noexcept {
    return status == EnvelopeRegistrationStatus::accepted;
  }
};

enum class DependentArtifactStatus { unknown, active, invalidated };

struct LockedCableUncertaintyEnvelope {
  std::uint64_t envelope_version{};
  std::uint64_t authorization_generation{};
  std::shared_ptr<const CableUncertaintyEnvelope> envelope;
  EnvelopeCoverageCertification coverage_certification;
};

enum class EnvelopeQueryStatus {
  valid,
  input_invalid,
  envelope_invalidated,
  progress_out_of_range,
};

struct EnvelopeQueryResult {
  EnvelopeQueryStatus status{EnvelopeQueryStatus::input_invalid};
  double lateral_stddev_upper_bound_m{};
  double certified_discretization_margin_m{};
  std::size_t adjacent_segment_count{};
  std::vector<DiagnosticEntry> diagnostics;
};

enum class EnvelopeContextUpdateStatus {
  not_applicable,
  accepted,
  idempotent,
  input_invalid,
  stale,
};

struct EnvelopeInvalidationResult {
  EnvelopeContextUpdateStatus context_update_status{
      EnvelopeContextUpdateStatus::not_applicable};
  bool context_changed{};
  std::size_t invalidated_envelope_count{};
  std::size_t invalidated_plan_count{};
  std::size_t revoked_lease_count{};
  std::vector<DiagnosticEntry> diagnostics;
};

enum class EnvelopeAuditStatus {
  pass,
  covariance_envelope_breach,
  input_invalid,
  envelope_unavailable,
};

struct EnvelopeAuditResult {
  EnvelopeAuditStatus status{EnvelopeAuditStatus::input_invalid};
  PlanningState planning_state{PlanningState::input_invalid};
  bool stop_required{};
  double actual_lateral_stddev_m{};
  double allowed_lateral_stddev_m{};
  std::size_t invalidated_envelope_count{};
  std::size_t invalidated_plan_count{};
  std::size_t revoked_lease_count{};
  std::vector<DiagnosticEntry> diagnostics;
};

class CableUncertaintyEnvelopeManager {
 public:
  [[nodiscard]] EnvelopeRegistrationResult registerValidated(
      std::uint64_t envelope_version, CableUncertaintyEnvelope envelope,
      EnvelopeCoverageCertification coverage_certification);

  [[nodiscard]] EnvelopeInvalidationResult setCurrentContext(
      const EnvelopeLookupKey& context, std::uint64_t context_sequence,
      MonotonicTime changed_at);

  [[nodiscard]] std::optional<LockedCableUncertaintyEnvelope> getValidated(
      const EnvelopeLookupKey& key, MonotonicTime now);

  [[nodiscard]] EnvelopeQueryResult query(
      const LockedCableUncertaintyEnvelope& locked,
      double reference_progress_m, MonotonicTime now);

  [[nodiscard]] bool registerDependentPlan(
      std::uint64_t plan_sequence,
      const LockedCableUncertaintyEnvelope& locked, MonotonicTime now);
  [[nodiscard]] bool registerDependentLease(
      std::uint64_t lease_sequence, std::uint64_t plan_sequence,
      const LockedCableUncertaintyEnvelope& locked, MonotonicTime now);

  [[nodiscard]] EnvelopeInvalidationResult invalidate(
      std::uint64_t envelope_version, MonotonicTime invalidated_at);
  [[nodiscard]] EnvelopeInvalidationResult expire(MonotonicTime now);

  [[nodiscard]] EnvelopeAuditResult auditActualLateralStddev(
      const LockedCableUncertaintyEnvelope& locked,
      double reference_progress_m, double actual_lateral_stddev_m,
      double audit_tolerance_m, MonotonicTime audited_at);

  [[nodiscard]] DependentArtifactStatus envelopeStatus(
      std::uint64_t envelope_version) const;
  [[nodiscard]] DependentArtifactStatus planStatus(
      std::uint64_t plan_sequence) const;
  [[nodiscard]] DependentArtifactStatus leaseStatus(
      std::uint64_t lease_sequence) const;

 private:
  struct EnvelopeRecord {
    EnvelopeLookupKey key;
    std::shared_ptr<const CableUncertaintyEnvelope> envelope;
    EnvelopeCoverageCertification coverage_certification;
    DependentArtifactStatus status{DependentArtifactStatus::active};
    std::uint64_t authorization_generation{};
  };

  struct PlanRecord {
    std::uint64_t envelope_version{};
    std::uint64_t envelope_generation{};
    DependentArtifactStatus status{DependentArtifactStatus::active};
  };

  struct LeaseRecord {
    std::uint64_t plan_sequence{};
    std::uint64_t envelope_version{};
    std::uint64_t envelope_generation{};
    DependentArtifactStatus status{DependentArtifactStatus::active};
  };

  [[nodiscard]] EnvelopeQueryResult queryLocked(
      const LockedCableUncertaintyEnvelope& locked,
      double reference_progress_m) const;
  [[nodiscard]] bool isCurrentLocked(
      const LockedCableUncertaintyEnvelope& locked) const;
  EnvelopeInvalidationResult invalidateLocked(
      std::uint64_t envelope_version, MonotonicTime invalidated_at,
      std::string diagnostic_code, std::string diagnostic_message);
  EnvelopeInvalidationResult expireLocked(MonotonicTime now);
  [[nodiscard]] bool prepareForUseLocked(MonotonicTime now);

  mutable std::mutex mutex_;
  std::map<std::uint64_t, EnvelopeRecord> envelopes_;
  std::map<std::uint64_t, PlanRecord> plans_;
  std::map<std::uint64_t, LeaseRecord> leases_;
  std::optional<EnvelopeLookupKey> current_context_;
  std::uint64_t current_context_sequence_{};
  std::uint64_t last_registered_version_{};
  std::uint64_t next_authorization_generation_{1};
};

}  // namespace underwater_planner::core
