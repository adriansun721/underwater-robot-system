#include "underwater_planner/core/message_consistency.hpp"
#include "underwater_planner/core/communication_recovery.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace underwater_planner::core;

namespace {

void require(const bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

MessageEnvelope map_message(const std::uint64_t sequence,
                            const std::uint64_t version,
                            const std::int64_t generated_at_ns) {
  return {MessageStream::map_update,
          MapUpdateKey{"map", "world", sequence},
          sequence,
          version,
          {generated_at_ns}};
}

MessageConsistencyConfig config() {
  MessageConsistencyConfig value;
  value.version = 41U;
  value.parameter_profile_id = "message-profile-v1";
  value.operating_domain_id = "domain-v1";
  value.risk_semantics = "per-stream-consistency-only";
  value.limits.map_max_age = {1'000'000'000};
  value.limits.reference_line_max_age = {900'000'000};
  value.limits.robot_state_max_age = {200'000'000};
  value.limits.cable_state_max_age = {300'000'000};
  value.limits.reference_progress_max_age = {300'000'000};
  value.limits.cable_telemetry_max_age = {100'000'000};
  value.limits.execution_tracking_max_age = {150'000'000};
  value.limits.scout_response_max_age = {2'000'000'000};
  value.limits.planning_result_max_age = {500'000'000};
  value.limits.validation_lease_max_age = {250'000'000};
  value.limits.reorder_sequence_window = 8U;
  value.limits.immediate_sequence_window = 32U;
  return value;
}

MessageEnvelope message(const MessageStream stream,
                        const std::uint64_t sequence,
                        const std::uint64_t version,
                        const std::int64_t generated_at_ns,
                        const std::string& id = "message") {
  MessageDeduplicationIdentity identity = MessageIdKey{id};
  if (stream == MessageStream::map_update) {
    identity = MapUpdateKey{id, "world", sequence};
  } else if (stream == MessageStream::scout_response) {
    identity = ScoutResponseKey{sequence, version};
  } else if (stream == MessageStream::planning_result) {
    identity = PlanningResultKey{sequence, {1'000'000'000}};
  } else if (stream == MessageStream::validation_lease) {
    identity = ValidationLeaseKey{sequence};
  }
  return {stream, identity, sequence, version, {generated_at_ns}};
}

void map_messages_are_reordered_deduplicated_and_fail_closed() {
  MessageConsistencyGate gate(config());

  const MessageGateDecision buffered =
      gate.receive(map_message(2, 2, 900), {1'000});
  require(buffered.disposition == MessageDisposition::buffered &&
              buffered.ready.empty(),
          "out-of-order map update was not buffered");

  const MessageGateDecision released =
      gate.receive(map_message(1, 1, 950), {1'000});
  require(released.disposition == MessageDisposition::accepted &&
              released.ready.size() == 2U &&
              released.ready[0].sequence_number == 1U &&
              released.ready[1].sequence_number == 2U,
          "continuous map updates were not released in sequence order");

  require(gate.receive(map_message(2, 2, 900), {1'000}).disposition ==
              MessageDisposition::duplicate,
          "duplicate map update was not rejected");
  require(gate.receive(map_message(3, 3, 1), {1'000'000'002})
              .disposition == MessageDisposition::too_old,
          "over-age map update was not rejected");
  require(gate.receive(map_message(3, 1, 990), {1'000}).disposition ==
              MessageDisposition::version_regression,
          "map version rollback was not rejected");
}

void buffered_messages_are_revalidated_before_release() {
  MessageConsistencyConfig short_config = config();
  short_config.limits.map_max_age = {100};
  MessageConsistencyGate gate(short_config);
  require(gate.receive(map_message(2U, 2U, 100), {101}).disposition ==
              MessageDisposition::buffered,
          "map update was not buffered for the revalidation scenario");
  const MessageGateDecision released =
      gate.receive(map_message(1U, 1U, 250), {251});
  require(released.ready.size() == 1U &&
              released.ready.front().sequence_number == 1U &&
              released.discarded.size() == 1U &&
              released.discarded.front().message.sequence_number == 2U &&
              released.discarded.front().disposition ==
                  MessageDisposition::too_old,
          "stale buffered map update was released after its maximum age");

  MessageConsistencyGate conflict_gate(config());
  require(conflict_gate.receive(map_message(2U, 2U, 300), {301})
              .disposition == MessageDisposition::buffered,
          "first sequence candidate was not buffered");
  require(conflict_gate.receive(
              message(MessageStream::map_update, 2U, 2U, 302,
                      "different-map"),
              {303})
              .disposition == MessageDisposition::duplicate,
          "two payload identities claimed the same stream sequence");

  MessageConsistencyConfig invalid_config = config();
  invalid_config.limits.robot_state_max_age = {0};
  bool rejected_invalid_policy = false;
  try {
    MessageConsistencyGate invalid_gate(invalid_config);
    static_cast<void>(invalid_gate);
  } catch (const std::invalid_argument&) {
    rejected_invalid_policy = true;
  }
  require(rejected_invalid_policy,
          "zero message maximum age was accepted as a production policy");
}

void every_message_class_has_an_explicit_policy_and_monotonic_gate() {
  const std::vector<MessageStreamPolicy> policies =
      make_message_consistency_policies(config().limits);
  require(policies.size() == static_cast<std::size_t>(MessageStream::count),
          "not every T41 message class has a policy");
  require(policies[static_cast<std::size_t>(MessageStream::map_update)]
                  .deduplication_key ==
              MessageDeduplicationKey::map_id_and_sequence &&
              policies[static_cast<std::size_t>(MessageStream::scout_response)]
                      .deduplication_key ==
                  MessageDeduplicationKey::request_sequence_and_revision &&
              policies[static_cast<std::size_t>(MessageStream::planning_result)]
                      .deduplication_key ==
                  MessageDeduplicationKey::plan_sequence &&
              policies[static_cast<std::size_t>(MessageStream::validation_lease)]
                      .deduplication_key ==
                  MessageDeduplicationKey::lease_sequence,
          "protocol-specific deduplication keys were not defined");
  const MessageConsistencyLimits configured_limits = config().limits;
  const std::vector<Duration> expected_ages{
      configured_limits.map_max_age,
      configured_limits.reference_line_max_age,
      configured_limits.robot_state_max_age,
      configured_limits.cable_state_max_age,
      configured_limits.reference_progress_max_age,
      configured_limits.cable_telemetry_max_age,
      configured_limits.execution_tracking_max_age,
      configured_limits.scout_response_max_age,
      configured_limits.planning_result_max_age,
      configured_limits.validation_lease_max_age};
  for (std::size_t index = 0; index < policies.size(); ++index) {
    require(policies[index].maximum_age.nanoseconds ==
                expected_ages[index].nanoseconds,
            "message maximum age was not bound to its explicit policy field");
  }

  const std::vector<MessageStream> immediate_streams{
      MessageStream::reference_line, MessageStream::robot_state,
      MessageStream::cable_state, MessageStream::reference_progress,
      MessageStream::cable_telemetry,
      MessageStream::execution_tracking, MessageStream::planning_result,
      MessageStream::validation_lease};
  for (const MessageStream stream : immediate_streams) {
    MessageConsistencyGate gate(config());
    const MessageEnvelope newest = message(stream, 12U, 12U, 1'000, "new");
    const MessageGateDecision accepted = gate.receive(newest, {1'001});
    require(accepted.disposition == MessageDisposition::accepted &&
                accepted.missing_sequence_range.has_value() &&
                accepted.missing_sequence_range->first == 1U &&
                accepted.missing_sequence_range->last == 11U,
            "new stream message was not accepted");
    require(gate.receive(message(stream, 11U, 11U, 1'001, "old"), {1'002})
                .disposition == MessageDisposition::sequence_regression,
            "old stream message was allowed to roll state backward");
    require(gate.receive(message(stream, 13U, 10U, 1'002, "rollback"), {1'003})
                .disposition == MessageDisposition::version_regression,
            "newer sequence with older version was accepted");
  }

  MessageConsistencyGate plan_gate(config());
  require(plan_gate.receive(
              message(MessageStream::planning_result, 20U, 20U, 2'000,
                      "publisher-a"),
              {2'001})
              .disposition == MessageDisposition::accepted,
          "new plan was not accepted");
  require(plan_gate.receive(
              message(MessageStream::planning_result, 20U, 20U, 2'002,
                      "publisher-b"),
              {2'003})
              .disposition == MessageDisposition::duplicate,
          "plan sequence deduplication depended on the transport message ID");

  MessageConsistencyGate lease_gate(config());
  require(lease_gate.receive(
              message(MessageStream::validation_lease, 7U, 7U, 3'000,
                      "transport-a"),
              {3'001})
              .disposition == MessageDisposition::accepted,
          "new lease was not accepted");
  require(lease_gate.receive(
              message(MessageStream::validation_lease, 7U, 7U, 3'002,
                      "transport-b"),
              {3'003})
              .disposition == MessageDisposition::duplicate,
          "lease sequence deduplication depended on the transport message ID");

  MessageConsistencyGate scout_gate(config());
  require(scout_gate.receive(
              message(MessageStream::scout_response, 2U, 2U, 4'000,
                      "response-b"),
              {4'001})
              .disposition == MessageDisposition::buffered,
          "out-of-order scout response was not buffered");
  const MessageGateDecision scout_release = scout_gate.receive(
      message(MessageStream::scout_response, 1U, 1U, 4'001, "response-a"),
      {4'002});
  require(scout_release.ready.size() == 2U &&
              scout_release.ready[0].sequence_number == 1U &&
              scout_release.ready[1].sequence_number == 2U,
          "scout responses were not released in request/revision order");

  MessageConsistencyGate bounded_gate(config());
  require(bounded_gate.receive(
              message(MessageStream::map_update, 9U, 9U, 5'000, "map"),
              {5'001})
              .disposition == MessageDisposition::sequence_outside_window,
          "message outside the configured sequence window was accepted");
  require(bounded_gate.receive(
              message(MessageStream::robot_state, 1U, 1U, 5'010, "future"),
              {5'009})
              .disposition == MessageDisposition::future_timestamp,
          "future-dated telemetry was accepted");
}

void planning_validity_typed_keys_and_rejections_are_auditable() {
  MessageConsistencyGate gate(config());
  MessageEnvelope expired_plan =
      message(MessageStream::planning_result, 1U, 1U, 100, "plan");
  expired_plan.deduplication_identity =
      PlanningResultKey{1U, Duration{50}};
  const MessageGateDecision expired = gate.receive(expired_plan, {150});
  require(expired.disposition == MessageDisposition::too_old &&
              expired.reason_code ==
                  "PLANNING_RESULT_INITIAL_ACCEPTANCE_EXPIRED",
          "planning result outlived its own initial acceptance duration");
  require(expired.audit.message.sequence_number == 1U &&
              expired.audit.evaluated_at.nanoseconds == 150 &&
              expired.audit.policy_version == 41U &&
              expired.audit.parameter_profile_id == "message-profile-v1" &&
              expired.audit.operating_domain_id == "domain-v1" &&
              expired.audit.risk_semantics ==
                  "per-stream-consistency-only",
          "rejected message lost version, time, domain, or risk audit data");

  MessageEnvelope wrong_key =
      message(MessageStream::map_update, 1U, 1U, 200, "map");
  wrong_key.deduplication_identity = MessageIdKey{"transport-id"};
  require(gate.receive(wrong_key, {201}).disposition ==
              MessageDisposition::invalid,
          "map update accepted a transport ID in place of its typed map key");
}

void receive(MessageConsistencyGate& gate, const MessageStream stream,
             const std::uint64_t sequence, const std::uint64_t version,
             const std::int64_t generated_at_ns) {
  const std::string identity = stream == MessageStream::map_update
                                   ? "map"
                                   : "message-" + std::to_string(sequence);
  const MessageGateDecision decision = gate.receive(
      message(stream, sequence, version, generated_at_ns, identity),
      {generated_at_ns + 1});
  require(decision.disposition == MessageDisposition::accepted,
          "resynchronization message was not accepted by ingress gate");
}

PlanningDependencyVersions dependencies() {
  PlanningDependencyVersions value;
  value.map_version = {"map", 3U, {1'100}, "world"};
  value.reference_line_version = 2U;
  value.robot_operating_area_version = 4U;
  value.terrain_gradient_policy_version = 5U;
  value.corridor_risk_policy_version = 6U;
  value.cable_model_version = 7U;
  value.uncertainty_envelope_version = 8U;
  value.uncertainty_envelope_generator_version = 9U;
  value.execution_operating_envelope_version = 10U;
  value.execution_profile_version = 11U;
  value.sensor_mode = SensorHealthMode::nominal;
  value.operating_domain_id = "domain-v1";
  value.cable_corridor_version = 1U;
  return value;
}

void communication_recovery_requires_a_complete_new_authorized_context() {
  MessageConsistencyGate ingress(config());
  const std::vector<MessageStream> required_streams{
      MessageStream::map_update,          MessageStream::reference_line,
      MessageStream::robot_state,         MessageStream::cable_state,
      MessageStream::reference_progress,  MessageStream::cable_telemetry,
      MessageStream::execution_tracking,  MessageStream::planning_result,
      MessageStream::validation_lease};
  for (const MessageStream stream : required_streams) {
    receive(ingress, stream, 1U, 1U, 900);
  }

  CommunicationRecoveryGate recovery;
  recovery.begin_resynchronization(ingress, 10U, 1U, {1'000});

  SynchronizedValidationInputs inputs;
  inputs.source_revision = 11U;
  inputs.captured_at = {1'150};
  inputs.robot_state.sequence_number = 2U;
  inputs.cable_state.sequence_number = 2U;
  inputs.reference_progress.sequence_number = 2U;
  inputs.cable_telemetry.sequence_number = 2U;
  inputs.execution_tracking_state.sequence_number = 2U;
  inputs.dependencies = dependencies();
  inputs.planning_snapshot.map.version = inputs.dependencies.map_version;
  inputs.planning_snapshot.reference_line.version = 2U;
  inputs.planning_snapshot.robot_operating_area.version = 4U;
  inputs.planning_snapshot.cable_corridor.version =
      inputs.dependencies.cable_corridor_version;

  PlanningResult plan;
  plan.sequence_number = 2U;
  plan.map_version = inputs.dependencies.map_version;
  plan.reference_line_version = inputs.dependencies.reference_line_version;
  plan.robot_operating_area_version =
      inputs.dependencies.robot_operating_area_version;
  plan.terrain_gradient_policy_version =
      inputs.dependencies.terrain_gradient_policy_version;
  plan.corridor_risk_policy_version =
      inputs.dependencies.corridor_risk_policy_version;
  plan.cable_model_version = inputs.dependencies.cable_model_version;
  plan.uncertainty_envelope_version =
      inputs.dependencies.uncertainty_envelope_version;
  plan.uncertainty_envelope_generator_version =
      inputs.dependencies.uncertainty_envelope_generator_version;
  plan.execution_operating_envelope_version =
      inputs.dependencies.execution_operating_envelope_version;
  plan.execution_profile_version =
      inputs.dependencies.execution_profile_version;
  plan.sensor_mode = inputs.dependencies.sensor_mode;
  plan.operating_domain_id = inputs.dependencies.operating_domain_id;
  plan.cable_corridor_version = inputs.dependencies.cable_corridor_version;

  PlanValidationLease lease;
  lease.lease_sequence = 2U;
  lease.plan_sequence_number = plan.sequence_number;
  lease.map_version = plan.map_version;
  lease.reference_line_version = plan.reference_line_version;
  lease.robot_operating_area_version = plan.robot_operating_area_version;
  lease.terrain_gradient_policy_version = plan.terrain_gradient_policy_version;
  lease.corridor_risk_policy_version = plan.corridor_risk_policy_version;
  lease.cable_model_version = plan.cable_model_version;
  lease.uncertainty_envelope_version = plan.uncertainty_envelope_version;
  lease.uncertainty_envelope_generator_version =
      plan.uncertainty_envelope_generator_version;
  lease.execution_operating_envelope_version =
      plan.execution_operating_envelope_version;
  lease.execution_profile_version = plan.execution_profile_version;
  lease.sensor_mode = plan.sensor_mode;
  lease.operating_domain_id = plan.operating_domain_id;
  lease.cable_corridor_version = plan.cable_corridor_version;
  lease.validated_at = {1'150};
  lease.expires_at = {1'300};

  ExecutionAuthorization authorization;
  authorization.status = ExecutionAuthorizationStatus::authorized;
  authorization.lease_sequence = lease.lease_sequence;
  authorization.evaluated_at = {1'200};

  require(recovery.evaluate(ingress, inputs, plan, lease, authorization, {1'200})
              .status == CommunicationRecoveryStatus::awaiting_resynchronization,
          "automatic execution recovered before messages were resynchronized");

  PlanningStateMachineConfig state_config;
  state_config.version = 1U;
  state_config.parameter_profile_id = "profile-v1";
  state_config.operating_domain_id = "domain-v1";
  state_config.planning_period = {100};
  state_config.maximum_consecutive_failures = 3U;
  state_config.short_communication_outage_limit = {5'000};
  state_config.medium_communication_outage_limit = {15'000};
  PlanningDecisionContext degraded_context;
  degraded_context.communication_outage = {1'000};
  degraded_context.current_lease_live = true;
  degraded_context.degraded_sensor_mode_approved = true;
  PlanningStateMachine blocked_machine(state_config);
  require(blocked_machine.dispatch(
              {PlanningEventType::communication_lost, 1U, {1'000}, "lost"},
              degraded_context)
              .action == PlanningAction::continue_authorized_path,
          "approved short degradation did not preserve the committed lease");
  PlanningDecisionContext incomplete_recovery_context;
  incomplete_recovery_context.safe_stop =
      SafeStopContext{0.1, 1.0, 1.0, 1.0, 0.05, {100}, true};
  const PlanningDecision blocked_recovery = blocked_machine.dispatch(
      {PlanningEventType::communication_restored, 2U, {1'200}, "restored"},
      incomplete_recovery_context);
  require(blocked_recovery.action == PlanningAction::controlled_stop &&
              blocked_recovery.reason_code == "RECOVERY_EVIDENCE_NOT_NEW",
          "state machine continued without a complete recovery authorization");

  receive(ingress, MessageStream::map_update, 2U, 2U, 950);
  receive(ingress, MessageStream::map_update, 3U, 3U, 1'100);
  receive(ingress, MessageStream::reference_line, 2U, 2U, 1'100);
  receive(ingress, MessageStream::robot_state, 2U, 2U, 1'100);
  receive(ingress, MessageStream::cable_state, 2U, 2U, 1'100);
  receive(ingress, MessageStream::reference_progress, 2U, 2U, 1'100);
  receive(ingress, MessageStream::cable_telemetry, 2U, 2U, 1'100);
  receive(ingress, MessageStream::execution_tracking, 2U, 2U, 1'100);
  receive(ingress, MessageStream::planning_result, 2U, 2U, 1'100);

  require(recovery.evaluate(ingress, inputs, plan, lease, authorization, {1'200})
              .status == CommunicationRecoveryStatus::awaiting_resynchronization,
          "automatic execution recovered without a post-recovery lease");
  receive(ingress, MessageStream::validation_lease, 2U, 2U, 1'100);

  const CommunicationRecoveryDecision recovered =
      recovery.evaluate(ingress, inputs, plan, lease, authorization, {1'200});
  require(recovered.status == CommunicationRecoveryStatus::authorized &&
              recovered.authorization.has_value() &&
              recovered.authorization->synchronized_source_revision == 11U &&
              recovered.authorization->lease_sequence == 2U,
          "complete synchronized context did not authorize recovery");

  PlanningStateMachine authorized_machine(state_config);
  static_cast<void>(authorized_machine.dispatch(
      {PlanningEventType::communication_lost, 1U, {1'000}, "lost"},
      degraded_context));
  PlanningDecisionContext authorized_context;
  authorized_context.recovery = recovered.authorization;
  const PlanningDecision authorized_recovery = authorized_machine.dispatch(
      {PlanningEventType::communication_restored, 2U, {1'200}, "restored"},
      authorized_context);
  require(authorized_recovery.action ==
              PlanningAction::continue_authorized_path &&
              authorized_recovery.state == PlanningState::path_valid,
          "state machine rejected the complete post-recovery authorization");

  SynchronizedValidationInputs mixed_inputs = inputs;
  mixed_inputs.dependencies.map_version.map_id = "other-map";
  mixed_inputs.dependencies.map_version.coordinate_frame = "other-frame";
  PlanningResult mixed_plan = plan;
  mixed_plan.map_version = mixed_inputs.dependencies.map_version;
  PlanValidationLease mixed_lease = lease;
  mixed_lease.map_version = mixed_inputs.dependencies.map_version;
  require(recovery.evaluate(ingress, mixed_inputs, mixed_plan, mixed_lease,
                            authorization, {1'200})
              .status == CommunicationRecoveryStatus::context_mismatch,
          "map ID/frame from another snapshot satisfied recovery by sequence alone");

  ExecutionAuthorization future_authorization = authorization;
  future_authorization.evaluated_at = {1'201};
  require(recovery.evaluate(ingress, inputs, plan, lease,
                            future_authorization, {1'200})
              .status == CommunicationRecoveryStatus::authorization_invalid,
          "future-dated execution authorization recovered automatic execution");

  PlanningResult stale_plan = plan;
  stale_plan.sequence_number = 1U;
  require(recovery.evaluate(ingress, inputs, stale_plan, lease, authorization,
                            {1'200})
              .status == CommunicationRecoveryStatus::context_mismatch,
          "old plan was allowed to replace the recovered plan");
}

}  // namespace

int main() {
  try {
    map_messages_are_reordered_deduplicated_and_fail_closed();
    buffered_messages_are_revalidated_before_release();
    every_message_class_has_an_explicit_policy_and_monotonic_gate();
    planning_validity_typed_keys_and_rejections_are_auditable();
    communication_recovery_requires_a_complete_new_authorized_context();
    std::cout << "message consistency tests passed\n";
    return EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::cerr << "message consistency tests failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
