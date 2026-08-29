#pragma once

#include "underwater_planner/core/data_contract.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <variant>
#include <vector>

namespace underwater_planner::core {

enum class MessageStream {
  map_update,
  reference_line,
  robot_state,
  cable_state,
  reference_progress,
  cable_telemetry,
  execution_tracking,
  scout_response,
  planning_result,
  validation_lease,
  count,
};

enum class MessageDeduplicationKey {
  message_id,
  map_id_and_sequence,
  request_sequence_and_revision,
  plan_sequence,
  lease_sequence,
};

struct MessageIdKey {
  std::string message_id;
};

struct MapUpdateKey {
  std::string map_id;
  std::string coordinate_frame;
  std::uint64_t map_sequence{};
};

struct ScoutResponseKey {
  std::uint64_t request_sequence{};
  std::uint64_t revision{};
};

struct PlanningResultKey {
  std::uint64_t plan_sequence{};
  Duration initial_acceptance_validity;
};

struct ValidationLeaseKey {
  std::uint64_t lease_sequence{};
};

using MessageDeduplicationIdentity =
    std::variant<MessageIdKey, MapUpdateKey, ScoutResponseKey,
                 PlanningResultKey, ValidationLeaseKey>;

struct MessageEnvelope {
  MessageStream stream{MessageStream::map_update};
  MessageDeduplicationIdentity deduplication_identity{MessageIdKey{}};
  std::uint64_t sequence_number{};
  std::uint64_t version{};
  MonotonicTime generated_at;
};

struct MessageStreamPolicy {
  MessageStream stream{MessageStream::map_update};
  MessageDeduplicationKey deduplication_key{
      MessageDeduplicationKey::message_id};
  Duration maximum_age;
  std::uint64_t sequence_window{};
  bool allow_reordering{};
};

struct MessageConsistencyLimits {
  Duration map_max_age;
  Duration reference_line_max_age;
  Duration robot_state_max_age;
  Duration cable_state_max_age;
  Duration reference_progress_max_age;
  Duration cable_telemetry_max_age;
  Duration execution_tracking_max_age;
  Duration scout_response_max_age;
  Duration planning_result_max_age;
  Duration validation_lease_max_age;
  std::uint64_t reorder_sequence_window{};
  std::uint64_t immediate_sequence_window{};
};

struct MessageConsistencyConfig {
  std::uint64_t version{};
  std::string parameter_profile_id;
  std::string operating_domain_id;
  std::string risk_semantics;
  MessageConsistencyLimits limits;
};

[[nodiscard]] std::vector<MessageStreamPolicy>
make_message_consistency_policies(const MessageConsistencyLimits& limits);

enum class MessageDisposition {
  accepted,
  buffered,
  duplicate,
  too_old,
  future_timestamp,
  receive_time_regression,
  sequence_regression,
  sequence_outside_window,
  version_regression,
  invalid,
};

struct BufferedMessageDiscard {
  MessageEnvelope message;
  MessageDisposition disposition{MessageDisposition::invalid};
  std::string reason_code;
};

struct MessageGateAudit {
  MessageEnvelope message;
  MonotonicTime evaluated_at;
  std::uint64_t policy_version{};
  std::string parameter_profile_id;
  std::string operating_domain_id;
  std::string risk_semantics;
};

struct MissingSequenceRange {
  std::uint64_t first{};
  std::uint64_t last{};
};

struct MessageGateDecision {
  MessageDisposition disposition{MessageDisposition::invalid};
  std::vector<MessageEnvelope> ready;
  std::vector<BufferedMessageDiscard> discarded;
  MessageGateAudit audit;
  std::optional<MissingSequenceRange> missing_sequence_range;
  std::string reason_code;
  std::string reason;
};

struct MessageStreamWatermark {
  std::uint64_t sequence_number{};
  std::uint64_t version{};
  MonotonicTime generated_at;
  MessageDeduplicationIdentity deduplication_identity{MessageIdKey{}};
};

class MessageConsistencyGate {
 public:
  explicit MessageConsistencyGate(const MessageConsistencyConfig& config);

  [[nodiscard]] MessageGateDecision receive(const MessageEnvelope& message,
                                            MonotonicTime now);
  [[nodiscard]] std::optional<MessageStreamWatermark> watermark(
      MessageStream stream) const;

 private:
  static constexpr std::size_t stream_count =
      static_cast<std::size_t>(MessageStream::count);

  struct StreamState {
    std::uint64_t next_sequence{1U};
    std::optional<MonotonicTime> last_receive_time;
    std::optional<MessageStreamWatermark> watermark;
    std::map<std::uint64_t, MessageEnvelope> buffered;
    std::set<std::string> accepted_ids;
    std::deque<std::pair<std::uint64_t, std::string>> accepted_id_order;
  };

  MessageConsistencyGate(MessageConsistencyConfig config,
                         std::vector<MessageStreamPolicy> policies);

  MessageConsistencyConfig config_;
  std::array<std::optional<MessageStreamPolicy>, stream_count> policies_;
  std::array<StreamState, stream_count> states_;
  mutable std::mutex mutex_;
};

}  // namespace underwater_planner::core
