#pragma once

#include "underwater_planner/core/execution_lease_monitor.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <string_view>

namespace underwater_planner::core {

enum class CommitmentSafetyEvent {
  none,
  emergency_stop,
  new_obstacle,
  terrain_constraint_change,
  localization_jump,
  cable_state_anomaly,
  execution_deviation,
  dependency_version_change,
  robot_constraint_violation,
  validation_unavailable,
};

enum class CommitmentSafetyAction {
  continue_commitment,
  replan_urgent,
  stop,
};

struct ObstacleStoppingEvidence {
  double obstacle_distance_m{};
  double certified_stopping_distance_m{};
};

// Validator evidence is optional so an asynchronous event source can revoke
// immediately without waiting for a planning-cycle validation pass. A normal
// commitment check (no observed event) requires both complete validators.
struct CommitmentSafetyObservation {
  std::optional<PathCandidateVerificationResult> robot_validation;
  std::optional<TimedCableCandidateResult> cable_validation;
  CommitmentSafetyEvent observed_event{CommitmentSafetyEvent::none};
  std::optional<ObstacleStoppingEvidence> obstacle_stopping;
};

struct CommitmentSafetyCheckResult {
  bool is_safe{};
  CommitmentSafetyEvent event{CommitmentSafetyEvent::validation_unavailable};
  CommitmentSafetyAction action{CommitmentSafetyAction::stop};
};

[[nodiscard]] std::string_view commitment_safety_reason_code(
    CommitmentSafetyEvent event) noexcept;

class CommitmentSafetyEvaluator {
 public:
  [[nodiscard]] CommitmentSafetyCheckResult evaluate(
      const CommitmentSafetyObservation& observation) const noexcept;
};

using CommitmentSafetyStopChannel =
    std::function<void(const CommitmentSafetyCheckResult&, MonotonicTime)>;

struct CommitmentSafetyEnforcement {
  CommitmentSafetyCheckResult safety;
  std::optional<std::uint64_t> revoked_lease_sequence;
  bool stop_channel_requested{};
};

// Event adapters may call handle_event immediately on map, localization,
// cable, execution, or dependency notifications. Revocation is durable before
// the independent stop channel is invoked, so this path does not wait for the
// periodic planning loop.
class CommitmentSafetySupervisor {
 public:
  CommitmentSafetySupervisor(
      ExecutionLeaseMonitor& lease_monitor,
      CommitmentSafetyStopChannel stop_channel);

  [[nodiscard]] CommitmentSafetyEnforcement handle_event(
      const CommitmentSafetyObservation& observation,
      std::uint64_t active_lease_sequence, MonotonicTime now) const;

  [[nodiscard]] CommitmentSafetyEnforcement enforce(
      const CommitmentSafetyCheckResult& safety,
      std::uint64_t active_lease_sequence, MonotonicTime now) const;

 private:
  ExecutionLeaseMonitor& lease_monitor_;
  CommitmentSafetyStopChannel stop_channel_;
  CommitmentSafetyEvaluator evaluator_;
};

}  // namespace underwater_planner::core
