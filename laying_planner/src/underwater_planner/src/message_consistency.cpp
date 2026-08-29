#include "underwater_planner/core/message_consistency.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>

namespace underwater_planner::core {
namespace {

std::size_t index_of(const MessageStream stream) {
  return static_cast<std::size_t>(stream);
}

bool valid_duration(const Duration value) noexcept {
  return value.nanoseconds > 0;
}

MessageGateDecision reject(const MessageDisposition disposition,
                           std::string code, std::string reason) {
  MessageGateDecision result;
  result.disposition = disposition;
  result.reason_code = std::move(code);
  result.reason = std::move(reason);
  return result;
}

struct ValidatedDeduplicationIdentity {
  MessageDeduplicationKey kind{MessageDeduplicationKey::message_id};
  std::string value;
};

struct IdentityValidator {
  const MessageEnvelope& message;

  std::optional<ValidatedDeduplicationIdentity> operator()(
      const MessageIdKey& key) const {
    if (key.message_id.empty()) return std::nullopt;
    return ValidatedDeduplicationIdentity{
        MessageDeduplicationKey::message_id, key.message_id};
  }

  std::optional<ValidatedDeduplicationIdentity> operator()(
      const MapUpdateKey& key) const {
    if (key.map_id.empty() || key.coordinate_frame.empty() ||
        key.map_sequence != message.sequence_number) {
      return std::nullopt;
    }
    return ValidatedDeduplicationIdentity{
        MessageDeduplicationKey::map_id_and_sequence,
        key.map_id + ":" + std::to_string(key.map_sequence)};
  }

  std::optional<ValidatedDeduplicationIdentity> operator()(
      const ScoutResponseKey& key) const {
    if (key.request_sequence == 0U || key.revision != message.version) {
      return std::nullopt;
    }
    return ValidatedDeduplicationIdentity{
        MessageDeduplicationKey::request_sequence_and_revision,
        std::to_string(key.request_sequence) + ":" +
            std::to_string(key.revision)};
  }

  std::optional<ValidatedDeduplicationIdentity> operator()(
      const PlanningResultKey& key) const {
    if (key.plan_sequence != message.sequence_number ||
        key.initial_acceptance_validity.nanoseconds <= 0) {
      return std::nullopt;
    }
    return ValidatedDeduplicationIdentity{
        MessageDeduplicationKey::plan_sequence,
        std::to_string(key.plan_sequence)};
  }

  std::optional<ValidatedDeduplicationIdentity> operator()(
      const ValidationLeaseKey& key) const {
    if (key.lease_sequence != message.sequence_number) {
      return std::nullopt;
    }
    return ValidatedDeduplicationIdentity{
        MessageDeduplicationKey::lease_sequence,
        std::to_string(key.lease_sequence)};
  }
};

std::optional<ValidatedDeduplicationIdentity> validate_identity(
    const MessageEnvelope& message) {
  return std::visit(IdentityValidator{message},
                    message.deduplication_identity);
}

MessageGateAudit audit_for(const MessageConsistencyConfig& config,
                           const MessageEnvelope& message,
                           const MonotonicTime now) {
  return {message,
          now,
          config.version,
          config.parameter_profile_id,
          config.operating_domain_id,
          config.risk_semantics};
}

}  // namespace

std::vector<MessageStreamPolicy> make_message_consistency_policies(
    const MessageConsistencyLimits& limits) {
  return {
      {MessageStream::map_update,
       MessageDeduplicationKey::map_id_and_sequence, limits.map_max_age,
       limits.reorder_sequence_window, true},
      {MessageStream::reference_line,
       MessageDeduplicationKey::message_id, limits.reference_line_max_age,
       limits.immediate_sequence_window, false},
      {MessageStream::robot_state, MessageDeduplicationKey::message_id,
       limits.robot_state_max_age, limits.immediate_sequence_window, false},
      {MessageStream::cable_state, MessageDeduplicationKey::message_id,
       limits.cable_state_max_age, limits.immediate_sequence_window, false},
      {MessageStream::reference_progress,
       MessageDeduplicationKey::message_id,
       limits.reference_progress_max_age, limits.immediate_sequence_window,
       false},
      {MessageStream::cable_telemetry, MessageDeduplicationKey::message_id,
       limits.cable_telemetry_max_age, limits.immediate_sequence_window,
       false},
      {MessageStream::execution_tracking,
       MessageDeduplicationKey::message_id,
       limits.execution_tracking_max_age, limits.immediate_sequence_window,
       false},
      {MessageStream::scout_response,
       MessageDeduplicationKey::request_sequence_and_revision,
       limits.scout_response_max_age, limits.reorder_sequence_window, true},
      {MessageStream::planning_result,
       MessageDeduplicationKey::plan_sequence,
       limits.planning_result_max_age, limits.immediate_sequence_window,
       false},
      {MessageStream::validation_lease,
       MessageDeduplicationKey::lease_sequence,
       limits.validation_lease_max_age, limits.immediate_sequence_window,
       false},
  };
}

MessageConsistencyGate::MessageConsistencyGate(
    const MessageConsistencyConfig& config)
    : MessageConsistencyGate(config,
                             make_message_consistency_policies(config.limits)) {}

MessageConsistencyGate::MessageConsistencyGate(
    MessageConsistencyConfig config,
    std::vector<MessageStreamPolicy> policies)
    : config_(std::move(config)) {
  if (config_.version == 0U || config_.parameter_profile_id.empty() ||
      config_.operating_domain_id.empty() || config_.risk_semantics.empty()) {
    throw std::invalid_argument(
        "message consistency configuration audit metadata is required");
  }
  for (const MessageStreamPolicy& policy : policies) {
    const std::size_t index = index_of(policy.stream);
    if (index >= stream_count || policies_[index].has_value() ||
        !valid_duration(policy.maximum_age) || policy.sequence_window == 0U) {
      throw std::invalid_argument(
          "message policies require one valid entry per message stream");
    }
    policies_[index] = policy;
  }
  if (std::any_of(policies_.begin(), policies_.end(),
                  [](const auto& policy) { return !policy.has_value(); })) {
    throw std::invalid_argument(
        "message policies require one valid entry per message stream");
  }
}

MessageGateDecision MessageConsistencyGate::receive(
    const MessageEnvelope& message, const MonotonicTime now) {
  const std::lock_guard<std::mutex> lock(mutex_);
  const auto reject_message = [&](const MessageDisposition disposition,
                                  std::string code, std::string reason) {
    MessageGateDecision result =
        reject(disposition, std::move(code), std::move(reason));
    result.audit = audit_for(config_, message, now);
    return result;
  };
  const std::size_t index = index_of(message.stream);
  if (index >= stream_count || message.sequence_number == 0U ||
      message.version == 0U ||
      message.generated_at.nanoseconds < 0 || now.nanoseconds < 0) {
    return reject_message(
        MessageDisposition::invalid, "MESSAGE_METADATA_INVALID",
        "message identity, sequence, version, and monotonic times are required");
  }

  const MessageStreamPolicy& policy = *policies_[index];
  StreamState& state = states_[index];
  const std::optional<ValidatedDeduplicationIdentity> identity =
      validate_identity(message);
  if (!identity.has_value() || identity->kind != policy.deduplication_key) {
    return reject_message(
        MessageDisposition::invalid, "MESSAGE_DEDUPLICATION_KEY_INVALID",
        "message deduplication identity does not match its stream protocol");
  }
  const std::string& deduplication_key = identity->value;
  if (message.generated_at.nanoseconds > now.nanoseconds) {
    return reject_message(MessageDisposition::future_timestamp,
                          "MESSAGE_TIMESTAMP_IN_FUTURE",
                          "message generation time is later than receive time");
  }
  if (state.last_receive_time.has_value() &&
      now.nanoseconds < state.last_receive_time->nanoseconds) {
    return reject_message(MessageDisposition::receive_time_regression,
                          "MESSAGE_RECEIVE_TIME_REGRESSION",
                          "stream receive time cannot move backward");
  }
  const std::int64_t age =
      now.nanoseconds - message.generated_at.nanoseconds;
  if (const auto* plan_key =
          std::get_if<PlanningResultKey>(&message.deduplication_identity);
      plan_key != nullptr &&
      age >= plan_key->initial_acceptance_validity.nanoseconds) {
    return reject_message(
        MessageDisposition::too_old,
        "PLANNING_RESULT_INITIAL_ACCEPTANCE_EXPIRED",
        "planning result exceeded its own initial acceptance validity duration");
  }
  if (age > policy.maximum_age.nanoseconds) {
    return reject_message(MessageDisposition::too_old, "MESSAGE_TOO_OLD",
                          "message exceeded its stream maximum age");
  }
  if (state.accepted_ids.find(deduplication_key) !=
          state.accepted_ids.end() ||
      std::any_of(state.buffered.begin(), state.buffered.end(),
                  [&deduplication_key](const auto& item) {
                    const auto buffered_identity =
                        validate_identity(item.second);
                    return buffered_identity.has_value() &&
                           buffered_identity->value == deduplication_key;
                  })) {
    return reject_message(MessageDisposition::duplicate, "MESSAGE_DUPLICATE",
                          "message deduplication key was already received");
  }
  if (state.buffered.find(message.sequence_number) != state.buffered.end()) {
    return reject_message(MessageDisposition::duplicate,
                          "MESSAGE_SEQUENCE_ALREADY_BUFFERED",
                          "another message already claims this stream sequence");
  }
  if (message.sequence_number < state.next_sequence) {
    return reject_message(
        MessageDisposition::sequence_regression, "MESSAGE_SEQUENCE_REGRESSION",
        "message sequence is older than the committed stream sequence");
  }
  if (state.watermark.has_value() &&
      message.version < state.watermark->version) {
    return reject_message(
        MessageDisposition::version_regression, "MESSAGE_VERSION_REGRESSION",
        "message version is older than the committed stream version");
  }
  if (message.sequence_number - state.next_sequence >= policy.sequence_window) {
    return reject_message(
        MessageDisposition::sequence_outside_window,
        "MESSAGE_SEQUENCE_OUTSIDE_WINDOW",
        "message sequence is outside the configured receive window");
  }

  std::optional<MissingSequenceRange> missing_sequence_range;
  if (!policy.allow_reordering &&
      message.sequence_number != state.next_sequence) {
    missing_sequence_range =
        MissingSequenceRange{state.next_sequence,
                             message.sequence_number - 1U};
    state.next_sequence = message.sequence_number;
  }
  state.last_receive_time = now;
  state.buffered.emplace(message.sequence_number, message);
  if (message.sequence_number != state.next_sequence) {
    MessageGateDecision result;
    result.disposition = MessageDisposition::buffered;
    result.audit = audit_for(config_, message, now);
    result.reason_code = "MESSAGE_BUFFERED_FOR_REORDERING";
    result.reason = "message is waiting for preceding sequence numbers";
    return result;
  }

  MessageGateDecision result;
  result.disposition = MessageDisposition::accepted;
  result.audit = audit_for(config_, message, now);
  result.missing_sequence_range = missing_sequence_range;
  result.reason_code = "MESSAGE_ACCEPTED";
  result.reason = "message passed freshness, ordering, and version checks";
  while (true) {
    const auto next = state.buffered.find(state.next_sequence);
    if (next == state.buffered.end()) break;
    const MessageEnvelope ready = next->second;
    if (now.nanoseconds - ready.generated_at.nanoseconds >
        policy.maximum_age.nanoseconds) {
      result.discarded.push_back(
          {ready, MessageDisposition::too_old, "BUFFERED_MESSAGE_TOO_OLD"});
      state.buffered.erase(next);
      ++state.next_sequence;
      continue;
    }
    if (state.watermark.has_value() &&
        ready.version < state.watermark->version) {
      result.discarded.push_back({ready,
                                  MessageDisposition::version_regression,
                                  "BUFFERED_MESSAGE_VERSION_REGRESSION"});
      state.buffered.erase(next);
      ++state.next_sequence;
      continue;
    }
    result.ready.push_back(ready);
    state.watermark = MessageStreamWatermark{
        ready.sequence_number, ready.version, ready.generated_at,
        ready.deduplication_identity};
    const std::string ready_key = validate_identity(ready)->value;
    state.accepted_ids.insert(ready_key);
    state.accepted_id_order.emplace_back(ready.sequence_number,
                                         ready_key);
    state.buffered.erase(next);
    ++state.next_sequence;
  }

  while (!state.accepted_id_order.empty() &&
         state.accepted_id_order.front().first < state.next_sequence &&
         state.next_sequence - state.accepted_id_order.front().first >
             policy.sequence_window) {
    state.accepted_ids.erase(state.accepted_id_order.front().second);
    state.accepted_id_order.pop_front();
  }
  if (!result.discarded.empty()) {
    result.reason_code = "MESSAGE_ACCEPTED_WITH_BUFFER_DISCARDS";
    result.reason =
        "current message was accepted while stale or regressive buffered messages were discarded";
  } else if (result.missing_sequence_range.has_value()) {
    result.reason_code = "MESSAGE_ACCEPTED_WITH_SEQUENCE_GAP";
    result.reason =
        "message was accepted after recording a missing sequence range";
  }
  return result;
}

std::optional<MessageStreamWatermark> MessageConsistencyGate::watermark(
    const MessageStream stream) const {
  const std::lock_guard<std::mutex> lock(mutex_);
  const std::size_t index = index_of(stream);
  if (index >= stream_count) return std::nullopt;
  return states_[index].watermark;
}

}  // namespace underwater_planner::core
