#pragma once

#include "underwater_planner/core/execution_lease_monitor.hpp"
#include "underwater_planner/core/message_consistency.hpp"
#include "underwater_planner/core/planning_state_machine.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace underwater_planner::core {

enum class CommunicationRecoveryStatus {
  not_recovering,
  awaiting_resynchronization,
  context_mismatch,
  authorization_invalid,
  authorized,
};

struct CommunicationRecoveryDecision {
  CommunicationRecoveryStatus status{
      CommunicationRecoveryStatus::not_recovering};
  std::optional<RecoveryAuthorization> authorization;
  std::vector<MessageStream> missing_streams;
  std::string reason_code;
  std::string reason;
};

class CommunicationRecoveryGate {
 public:
  // Captures the receive watermarks at the recovery boundary. Messages that
  // were already present, or were generated before restored_at, cannot count
  // toward the new synchronized execution context.
  void begin_resynchronization(const MessageConsistencyGate& ingress,
                               std::uint64_t synchronized_source_revision,
                               std::uint64_t active_lease_sequence,
                               MonotonicTime restored_at);

  [[nodiscard]] CommunicationRecoveryDecision evaluate(
      const MessageConsistencyGate& ingress,
      const SynchronizedValidationInputs& inputs, const PlanningResult& plan,
      const PlanValidationLease& lease,
      const ExecutionAuthorization& execution_authorization,
      MonotonicTime now) const;

 private:
  static constexpr std::size_t stream_count =
      static_cast<std::size_t>(MessageStream::count);

  mutable std::mutex mutex_;
  bool recovering_{};
  MonotonicTime restored_at_;
  std::uint64_t baseline_source_revision_{};
  std::uint64_t baseline_lease_sequence_{};
  std::array<std::optional<MessageStreamWatermark>, stream_count> baselines_;
};

}  // namespace underwater_planner::core
