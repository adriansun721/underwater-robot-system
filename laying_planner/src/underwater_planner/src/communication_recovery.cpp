#include "underwater_planner/core/communication_recovery.hpp"

#include <array>
#include <utility>

namespace underwater_planner::core {
namespace {

constexpr std::array<MessageStream, 9> required_recovery_streams{
    MessageStream::map_update,
    MessageStream::reference_line,
    MessageStream::robot_state,
    MessageStream::cable_state,
    MessageStream::reference_progress,
    MessageStream::cable_telemetry,
    MessageStream::execution_tracking,
    MessageStream::planning_result,
    MessageStream::validation_lease,
};

std::size_t index_of(const MessageStream stream) {
  return static_cast<std::size_t>(stream);
}

CommunicationRecoveryDecision decision(
    const CommunicationRecoveryStatus status, std::string code,
    std::string reason) {
  CommunicationRecoveryDecision result;
  result.status = status;
  result.reason_code = std::move(code);
  result.reason = std::move(reason);
  return result;
}

}  // namespace

void CommunicationRecoveryGate::begin_resynchronization(
    const MessageConsistencyGate& ingress,
    const std::uint64_t synchronized_source_revision,
    const std::uint64_t active_lease_sequence,
    const MonotonicTime restored_at) {
  const std::lock_guard<std::mutex> lock(mutex_);
  recovering_ = restored_at.nanoseconds >= 0;
  restored_at_ = restored_at;
  baseline_source_revision_ = synchronized_source_revision;
  baseline_lease_sequence_ = active_lease_sequence;
  for (std::size_t index = 0; index < stream_count; ++index) {
    baselines_[index] =
        ingress.watermark(static_cast<MessageStream>(index));
  }
}

CommunicationRecoveryDecision CommunicationRecoveryGate::evaluate(
    const MessageConsistencyGate& ingress,
    const SynchronizedValidationInputs& inputs, const PlanningResult& plan,
    const PlanValidationLease& lease,
    const ExecutionAuthorization& execution_authorization,
    const MonotonicTime now) const {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!recovering_) {
    return decision(CommunicationRecoveryStatus::not_recovering,
                    "COMMUNICATION_RECOVERY_NOT_ACTIVE",
                    "a valid recovery boundary has not been established");
  }
  if (now.nanoseconds < restored_at_.nanoseconds) {
    return decision(CommunicationRecoveryStatus::context_mismatch,
                    "RECOVERY_TIME_REGRESSION",
                    "recovery evaluation time precedes the restore boundary");
  }

  CommunicationRecoveryDecision result = decision(
      CommunicationRecoveryStatus::awaiting_resynchronization,
      "RECOVERY_CONTEXT_INCOMPLETE",
      "every execution context stream must advance after communication restoration");
  for (const MessageStream stream : required_recovery_streams) {
    const std::optional<MessageStreamWatermark> current =
        ingress.watermark(stream);
    const std::optional<MessageStreamWatermark>& baseline =
        baselines_[index_of(stream)];
    const bool advanced =
        current.has_value() &&
        current->generated_at.nanoseconds >= restored_at_.nanoseconds &&
        (!baseline.has_value() ||
         current->sequence_number > baseline->sequence_number);
    if (!advanced) result.missing_streams.push_back(stream);
  }
  if (!result.missing_streams.empty()) return result;

  const auto map = ingress.watermark(MessageStream::map_update);
  const auto reference = ingress.watermark(MessageStream::reference_line);
  const auto robot = ingress.watermark(MessageStream::robot_state);
  const auto cable = ingress.watermark(MessageStream::cable_state);
  const auto progress = ingress.watermark(MessageStream::reference_progress);
  const auto telemetry = ingress.watermark(MessageStream::cable_telemetry);
  const auto tracking = ingress.watermark(MessageStream::execution_tracking);
  const auto accepted_plan = ingress.watermark(MessageStream::planning_result);
  const auto accepted_lease = ingress.watermark(MessageStream::validation_lease);
  const auto* accepted_map_key =
      std::get_if<MapUpdateKey>(&map->deduplication_identity);

  const bool synchronized_payload_matches =
      inputs.captured_at.nanoseconds >= restored_at_.nanoseconds &&
      inputs.captured_at.nanoseconds <= now.nanoseconds &&
      inputs.source_revision > baseline_source_revision_ &&
      accepted_map_key != nullptr &&
      accepted_map_key->map_id == inputs.dependencies.map_version.map_id &&
      accepted_map_key->coordinate_frame ==
          inputs.dependencies.map_version.coordinate_frame &&
      map->sequence_number == inputs.dependencies.map_version.sequence_number &&
      reference->version == inputs.dependencies.reference_line_version &&
      robot->sequence_number == inputs.robot_state.sequence_number &&
      cable->sequence_number == inputs.cable_state.sequence_number &&
      progress->sequence_number == inputs.reference_progress.sequence_number &&
      telemetry->sequence_number == inputs.cable_telemetry.sequence_number &&
      tracking->sequence_number ==
          inputs.execution_tracking_state.sequence_number &&
      accepted_plan->sequence_number == plan.sequence_number &&
      accepted_lease->sequence_number == lease.lease_sequence &&
      plan.dependencies() == inputs.dependencies &&
      lease.dependencies() == inputs.dependencies &&
      lease.plan_sequence_number == plan.sequence_number;
  if (!synchronized_payload_matches) {
    return decision(CommunicationRecoveryStatus::context_mismatch,
                    "RECOVERY_CONTEXT_MISMATCH",
                    "received watermarks, synchronized inputs, plan, and lease must describe one context");
  }

  const bool authorization_valid =
      lease.lease_sequence > baseline_lease_sequence_ &&
      lease.validated_at.nanoseconds >= restored_at_.nanoseconds &&
      lease.validated_at.nanoseconds <= now.nanoseconds &&
      lease.expires_at.nanoseconds > now.nanoseconds &&
      execution_authorization.authorized() &&
      execution_authorization.lease_sequence == lease.lease_sequence &&
      execution_authorization.evaluated_at.nanoseconds >=
          restored_at_.nanoseconds &&
      execution_authorization.evaluated_at.nanoseconds <= now.nanoseconds;
  if (!authorization_valid) {
    return decision(CommunicationRecoveryStatus::authorization_invalid,
                    "RECOVERY_AUTHORIZATION_INVALID",
                    "recovery requires a new live lease confirmed by the execution monitor");
  }

  result = decision(CommunicationRecoveryStatus::authorized,
                    "COMMUNICATION_RECOVERY_AUTHORIZED",
                    "complete post-recovery context and execution lease are synchronized");
  result.authorization = RecoveryAuthorization{inputs.source_revision,
                                                lease.lease_sequence, true,
                                                true, true};
  return result;
}

}  // namespace underwater_planner::core
