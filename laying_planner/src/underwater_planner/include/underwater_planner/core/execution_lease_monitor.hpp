#pragma once

#include "underwater_planner/core/plan_validity_evaluator.hpp"

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace underwater_planner::core {

// The execution endpoint has only one safe response to an invalid lease:
// revoke it and enter the controlled-stop path. Renewal is a request for a
// complete PlanValidityEvaluator pass; it never extends the current lease.
enum class ExecutionAuthorizationStatus {
  authorized,
  renewal_required,
  revoke_and_controlled_stop,
};

struct ActiveExecutionContext {
  MapVersion map_version;
  std::uint32_t reference_line_version{};
  std::uint32_t robot_operating_area_version{};
  std::uint64_t terrain_gradient_policy_version{};
  std::uint64_t corridor_risk_policy_version{};
  std::uint64_t cable_model_version{};
  std::uint64_t uncertainty_envelope_version{};
  std::uint64_t uncertainty_envelope_generator_version{};
  std::uint64_t execution_operating_envelope_version{};
  std::uint64_t execution_profile_version{};
  SensorHealthMode sensor_mode{SensorHealthMode::nominal};
  std::string operating_domain_id;
  std::uint32_t cable_corridor_version{};

  [[nodiscard]] PlanningDependencyVersions dependencies() const {
    return {map_version,
            reference_line_version,
            robot_operating_area_version,
            terrain_gradient_policy_version,
            corridor_risk_policy_version,
            cable_model_version,
            uncertainty_envelope_version,
            uncertainty_envelope_generator_version,
            execution_operating_envelope_version,
            execution_profile_version,
            sensor_mode,
            operating_domain_id,
            cable_corridor_version};
  }
};

struct ExecutionFeedback {
  std::uint64_t plan_sequence_number{};
  std::uint64_t execution_profile_version{};
  MonotonicTime timestamp;
  double ground_speed_mps{};
  double ground_acceleration_mps2{};
  double payout_speed_mps{};
  double payout_acceleration_mps2{};
  double tension_n{};
  double tracked_arc_length_m{};
  std::uint64_t sequence_number{};

  // Unit-suffixed fields above are the canonical contract. These optional
  // spellings mirror the design pseudocode for adapters that do not carry SI
  // suffixes; when present they take precedence during evaluation.
  std::optional<double> ground_speed;
  std::optional<double> ground_acceleration;
  std::optional<double> payout_speed;
  std::optional<double> payout_acceleration;
  std::optional<double> tension;
};

struct ExecutionLeaseMonitorConfig {
  Duration feedback_max_age{2'000'000'000};
  Duration monitor_period{500'000'000};
  Duration renewal_margin{1'000'000'000};
  Duration maximum_lease_duration{5'000'000'000};
  double maximum_ground_acceleration_tracking_error_mps2{0.1};
  double maximum_payout_acceleration_tracking_error_mps2{0.1};
  double maximum_tension_tracking_error_n{1.0};
};

struct ExecutionAuthorization {
  ExecutionAuthorizationStatus status{
      ExecutionAuthorizationStatus::revoke_and_controlled_stop};
  std::uint64_t lease_sequence{};
  bool revoke_lease{};
  bool request_controlled_stop{};
  bool request_replan{};
  std::string reason_code;
  std::string reason;
  MonotonicTime evaluated_at;
  std::vector<DiagnosticEntry> diagnostics;

  [[nodiscard]] bool authorized() const noexcept {
    return status == ExecutionAuthorizationStatus::authorized;
  }
  [[nodiscard]] bool renewalRequired() const noexcept {
    return status == ExecutionAuthorizationStatus::renewal_required;
  }
  [[nodiscard]] bool revoked() const noexcept {
    return status == ExecutionAuthorizationStatus::revoke_and_controlled_stop;
  }
};

class ExecutionLeaseMonitor {
 public:
  explicit ExecutionLeaseMonitor(
      ExecutionLeaseMonitorConfig config = {}) noexcept;

  // This overload follows the design contract. The feedback timestamp is the
  // observation point used for expiry and freshness checks.
  [[nodiscard]] ExecutionAuthorization evaluate(
      const PlanningResult& plan, const TimedPath& authorized_remaining_trajectory,
      const PlanValidationLease& lease,
      const ActiveExecutionContext& active_context,
      const ExecutionFeedback& feedback) const;

  // A caller with an independent monotonic clock should use this overload so
  // an old feedback sample cannot make an expired lease appear current.
  [[nodiscard]] ExecutionAuthorization evaluate(
      const PlanningResult& plan, const TimedPath& authorized_remaining_trajectory,
      const PlanValidationLease& lease,
      const ActiveExecutionContext& active_context,
      const ExecutionFeedback& feedback, MonotonicTime now) const;

  // Idempotent and thread-safe. Once revoked, a lease sequence cannot be
  // accepted again, even if a delayed feedback message arrives.
  void revokeLease(std::uint64_t lease_sequence, std::string reason_code,
                   std::string reason, MonotonicTime at) const;
  [[nodiscard]] bool isRevoked(std::uint64_t lease_sequence) const;
  [[nodiscard]] const ExecutionLeaseMonitorConfig& config() const noexcept {
    return config_;
  }

 private:
  struct LeaseObservation {
    std::size_t profile_fingerprint{};
    std::uint64_t last_feedback_sequence{};
    bool fingerprint_set{};
  };

  [[nodiscard]] ExecutionAuthorization evaluateAt(
      const PlanningResult& plan, const TimedPath& trajectory,
      const PlanValidationLease& lease,
      const ActiveExecutionContext& active_context,
      const ExecutionFeedback& feedback, MonotonicTime now) const;

  ExecutionLeaseMonitorConfig config_;
  mutable std::mutex mutex_;
  mutable std::uint64_t highest_plan_sequence_{};
  mutable std::uint64_t highest_lease_sequence_{};
  mutable std::map<std::uint64_t, LeaseObservation> observations_;
  mutable std::map<std::uint64_t, std::pair<std::string, std::string>> revoked_;
};

}  // namespace underwater_planner::core
