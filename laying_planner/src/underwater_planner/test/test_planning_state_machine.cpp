#include "underwater_planner/core/planning_state_machine.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>

using namespace underwater_planner::core;

namespace {

void require(const bool condition, const char* message) {
  if (!condition) {
    std::cerr << "T37 failure: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

PlanningEvent event(const PlanningEventType type, const std::uint64_t sequence,
                    const std::int64_t time_ns) {
  PlanningEvent result;
  result.type = type;
  result.sequence_number = sequence;
  result.observed_at = MonotonicTime{time_ns};
  return result;
}

SafeStopContext safe_stop(const double remaining_distance_m) {
  SafeStopContext result;
  result.current_ground_speed_mps = 2.0;
  result.maximum_braking_deceleration_mps2 = 2.0;
  result.terrain_limited_braking_deceleration_mps2 = 1.0;
  result.remaining_safe_distance_m = remaining_distance_m;
  result.safety_margin_m = 1.0;
  result.control_reaction_time = Duration{500'000'000};
  result.terrain_braking_model_certified = true;
  return result;
}

RecoveryAuthorization recovery(const std::uint64_t source_revision,
                               const std::uint64_t lease_sequence) {
  RecoveryAuthorization result;
  result.synchronized_source_revision = source_revision;
  result.lease_sequence = lease_sequence;
  result.synchronized_snapshot_valid = true;
  result.lease_live = true;
  result.dependencies_match = true;
  return result;
}

PlanningStateMachineConfig configuration() {
  PlanningStateMachineConfig result;
  result.version = 37U;
  result.parameter_profile_id = "state-machine-test-v37";
  result.operating_domain_id = "synthetic-level1/v1";
  result.planning_period = Duration{1'000'000'000};
  result.maximum_consecutive_failures = 3U;
  result.short_communication_outage_limit = Duration{5'000'000'000};
  result.medium_communication_outage_limit = Duration{15'000'000'000};
  return result;
}

void periodic_trigger_enters_planning_only_when_due() {
  const PlanningStateMachineConfig config = configuration();
  PlanningStateMachine machine(config);

  const PlanningDecision first =
      machine.dispatch(event(PlanningEventType::periodic_tick, 1U, 0));
  require(first.accepted && first.previous_state == PlanningState::init &&
              first.state == PlanningState::normal_planning,
          "the initial periodic tick must enter normal planning");
  require(first.action == PlanningAction::begin_planning &&
              first.directives.size() == 1U &&
              first.directives.front() == PlanningDirective::start_planning,
          "the first periodic tick must emit one planning directive");

  const PlanningDecision early = machine.dispatch(
      event(PlanningEventType::periodic_tick, 2U, 999'999'999));
  require(early.accepted && early.state == PlanningState::normal_planning &&
              early.action == PlanningAction::none && early.directives.empty(),
          "a periodic tick before the configured period must not replan");

  const PlanningDecision due = machine.dispatch(
      event(PlanningEventType::periodic_tick, 3U, 1'000'000'000));
  require(due.accepted && due.action == PlanningAction::begin_planning &&
              due.directives.size() == 1U &&
              due.directives.front() == PlanningDirective::start_planning,
          "a periodic tick at the configured period must replan");
}

void only_a_new_map_sequence_triggers_replanning() {
  const PlanningStateMachineConfig config = configuration();
  PlanningStateMachine machine(config);
  (void)machine.dispatch(event(PlanningEventType::periodic_tick, 1U, 0));

  PlanningEvent map = event(PlanningEventType::new_map, 2U, 100);
  PlanningDecisionContext map_context;
  map_context.map_sequence = 8U;
  map_context.safe_stop = safe_stop(10.0);
  const PlanningDecision first_map = machine.dispatch(map, map_context);
  require(first_map.accepted &&
              first_map.action == PlanningAction::controlled_stop &&
              first_map.directives ==
                  std::vector<PlanningDirective>{
                      PlanningDirective::revoke_current_lease,
                      PlanningDirective::request_controlled_stop,
                      PlanningDirective::start_planning} &&
              first_map.reason_code == "NEW_MAP",
          "a newer map must revoke the old lease before replanning");

  map.sequence_number = 3U;
  map.observed_at = MonotonicTime{200};
  const PlanningDecision duplicate = machine.dispatch(map, map_context);
  require(duplicate.accepted && duplicate.action == PlanningAction::none &&
              duplicate.directives.empty() &&
              duplicate.reason_code == "MAP_NOT_NEWER",
          "a repeated map sequence must not retrigger planning");
}

void invalidated_path_revokes_before_stopping_and_replanning() {
  const PlanningStateMachineConfig config = configuration();
  PlanningStateMachine machine(config);
  (void)machine.dispatch(event(PlanningEventType::periodic_tick, 1U, 0));

  PlanningEvent invalidated =
      event(PlanningEventType::path_invalidated, 2U, 100);
  PlanningDecisionContext invalidation_context;
  invalidation_context.safe_stop = safe_stop(4.0);
  const PlanningDecision controlled =
      machine.dispatch(invalidated, invalidation_context);
  require(controlled.accepted &&
              controlled.action == PlanningAction::controlled_stop &&
              controlled.state == PlanningState::normal_planning,
          "a path invalidation with enough distance must request a controlled stop");
  require(controlled.safe_stop_assessment.has_value() &&
              controlled.safe_stop_assessment->feasible() &&
              controlled.safe_stop_assessment->required_stopping_distance_m ==
                  4.0,
          "safe stopping must use speed, terrain-limited braking and margin");
  require(controlled.directives ==
              std::vector<PlanningDirective>{
                  PlanningDirective::revoke_current_lease,
                  PlanningDirective::request_controlled_stop,
                  PlanningDirective::start_planning},
          "path invalidation must revoke before stop and replan");

  invalidated.sequence_number = 3U;
  invalidated.observed_at = MonotonicTime{200};
  invalidation_context.safe_stop = safe_stop(3.99);
  const PlanningDecision emergency =
      machine.dispatch(invalidated, invalidation_context);
  require(emergency.accepted &&
              emergency.action == PlanningAction::emergency_stop &&
              emergency.state == PlanningState::emergency_stop &&
              emergency.safe_stop_assessment.has_value() &&
              emergency.safe_stop_assessment->status ==
                  SafeStopStatus::insufficient_distance,
          "insufficient remaining distance must escalate to emergency stop");
  require(emergency.directives ==
              std::vector<PlanningDirective>{
                  PlanningDirective::revoke_current_lease,
                  PlanningDirective::request_emergency_stop,
                  PlanningDirective::start_planning},
          "emergency path invalidation must still revoke before any other action");
}

void communication_outage_applies_the_validated_degradation_policy() {
  const PlanningStateMachineConfig config = configuration();

  PlanningStateMachine short_machine(config);
  PlanningEvent short_loss =
      event(PlanningEventType::communication_lost, 1U, 0);
  PlanningDecisionContext short_context;
  short_context.communication_outage = Duration{4'999'999'999};
  short_context.current_lease_live = true;
  short_context.degraded_sensor_mode_approved = true;
  const PlanningDecision short_decision =
      short_machine.dispatch(short_loss, short_context);
  require(short_decision.accepted &&
              short_decision.state == PlanningState::communication_degraded &&
              short_decision.action == PlanningAction::continue_authorized_path &&
              short_decision.directives ==
                  std::vector<PlanningDirective>{
                      PlanningDirective::continue_authorized_path},
          "a short outage may continue only with an approved mode and live lease");

  PlanningStateMachine medium_machine(config);
  PlanningEvent medium_loss =
      event(PlanningEventType::communication_lost, 1U, 0);
  PlanningDecisionContext medium_context;
  medium_context.communication_outage = Duration{5'000'000'000};
  medium_context.degraded_profile_lease_live = true;
  const PlanningDecision medium_decision =
      medium_machine.dispatch(medium_loss, medium_context);
  require(medium_decision.accepted &&
              medium_decision.action == PlanningAction::reduce_speed &&
              medium_decision.directives ==
                  std::vector<PlanningDirective>{
                      PlanningDirective::switch_to_validated_cautious_profile},
          "a medium outage requires a fully validated degraded profile lease");

  PlanningStateMachine long_machine(config);
  PlanningEvent long_loss =
      event(PlanningEventType::communication_lost, 1U, 0);
  PlanningDecisionContext long_context;
  long_context.communication_outage = Duration{15'000'000'001};
  long_context.safe_stop = safe_stop(10.0);
  const PlanningDecision long_decision =
      long_machine.dispatch(long_loss, long_context);
  require(long_decision.accepted &&
              long_decision.action == PlanningAction::controlled_stop &&
              long_decision.directives ==
                  std::vector<PlanningDirective>{
                      PlanningDirective::revoke_current_lease,
                      PlanningDirective::request_controlled_stop},
          "a long outage must revoke and stop even when a lease was live");
}

void recovery_requires_a_new_synchronized_snapshot_and_new_lease() {
  const PlanningStateMachineConfig config = configuration();
  PlanningStateMachine machine(config);

  PlanningEvent success =
      event(PlanningEventType::planning_succeeded, 1U, 0);
  PlanningDecisionContext success_context;
  success_context.recovery = recovery(10U, 20U);
  const PlanningDecision authorized =
      machine.dispatch(success, success_context);
  require(authorized.accepted && authorized.state == PlanningState::success &&
              authorized.action == PlanningAction::continue_authorized_path &&
              authorized.state_machine_config_version == 37U &&
              authorized.parameter_profile_id == "state-machine-test-v37" &&
              authorized.operating_domain_id == "synthetic-level1/v1" &&
              authorized.synchronized_source_revision == 10U &&
              authorized.lease_sequence == 20U,
          "a successful plan requires synchronized authorization evidence");

  PlanningEvent outage =
      event(PlanningEventType::communication_lost, 2U, 1);
  PlanningDecisionContext outage_context;
  outage_context.communication_outage = Duration{16'000'000'000};
  outage_context.safe_stop = safe_stop(10.0);
  (void)machine.dispatch(outage, outage_context);

  PlanningEvent stale =
      event(PlanningEventType::communication_restored, 3U, 2);
  PlanningDecisionContext stale_context;
  stale_context.recovery = recovery(10U, 20U);
  stale_context.safe_stop = safe_stop(10.0);
  const PlanningDecision rejected = machine.dispatch(stale, stale_context);
  require(rejected.accepted &&
              rejected.state == PlanningState::communication_degraded &&
              rejected.action == PlanningAction::controlled_stop &&
              rejected.reason_code == "RECOVERY_EVIDENCE_NOT_NEW",
          "recovery must not reuse the snapshot or lease from before degradation");

  PlanningEvent fresh =
      event(PlanningEventType::communication_restored, 4U, 3);
  PlanningDecisionContext fresh_context;
  fresh_context.recovery = recovery(11U, 21U);
  const PlanningDecision restored = machine.dispatch(fresh, fresh_context);
  require(restored.accepted && restored.state == PlanningState::path_valid &&
              restored.action == PlanningAction::continue_authorized_path &&
              restored.directives ==
                  std::vector<PlanningDirective>{
                      PlanningDirective::continue_authorized_path,
                      PlanningDirective::start_planning},
          "fresh recovery evidence must authorize execution and trigger replanning");
}

void planning_outcomes_cover_caution_wait_timeout_and_failure_states() {
  const PlanningStateMachineConfig config = configuration();

  PlanningStateMachine cautious_machine(config);
  PlanningEvent cautious =
      event(PlanningEventType::planning_with_caution, 1U, 0);
  PlanningDecisionContext cautious_context;
  cautious_context.recovery = recovery(1U, 1U);
  const PlanningDecision cautious_decision =
      cautious_machine.dispatch(cautious, cautious_context);
  require(cautious_decision.accepted &&
              cautious_decision.state == PlanningState::planning_with_caution &&
              cautious_decision.action == PlanningAction::reduce_speed &&
              cautious_decision.directives ==
                  std::vector<PlanningDirective>{
                      PlanningDirective::switch_to_validated_cautious_profile},
          "cautious planning must use a separately validated reduced profile");

  PlanningStateMachine waiting_machine(config);
  PlanningEvent waiting =
      event(PlanningEventType::waiting_for_map, 1U, 0);
  PlanningDecisionContext waiting_context;
  waiting_context.safe_stop = safe_stop(10.0);
  const PlanningDecision waiting_decision =
      waiting_machine.dispatch(waiting, waiting_context);
  require(waiting_decision.accepted &&
              waiting_decision.state == PlanningState::waiting_map &&
              waiting_decision.action == PlanningAction::controlled_stop &&
              waiting_decision.directives ==
                  std::vector<PlanningDirective>{
                      PlanningDirective::revoke_current_lease,
                      PlanningDirective::request_controlled_stop,
                      PlanningDirective::request_scout},
          "a blocking map gap must stop and request scout coverage");

  PlanningStateMachine cautious_waiting_machine(config);
  PlanningEvent cautious_waiting =
      event(PlanningEventType::waiting_for_map, 1U, 0);
  PlanningDecisionContext cautious_waiting_context;
  cautious_waiting_context.degraded_profile_lease_live = true;
  const PlanningDecision cautious_waiting_decision =
      cautious_waiting_machine.dispatch(cautious_waiting,
                                        cautious_waiting_context);
  require(cautious_waiting_decision.accepted &&
              cautious_waiting_decision.state == PlanningState::waiting_map &&
              cautious_waiting_decision.action == PlanningAction::reduce_speed &&
              cautious_waiting_decision.directives ==
                  std::vector<PlanningDirective>{
                      PlanningDirective::request_scout,
                      PlanningDirective::switch_to_validated_cautious_profile},
          "WAITING_MAP may slow only on a separately validated cautious lease");

  PlanningStateMachine timeout_reuse_machine(config);
  PlanningEvent reusable_timeout =
      event(PlanningEventType::planning_timed_out, 1U, 0);
  PlanningDecisionContext reusable_timeout_context;
  reusable_timeout_context.recovery = recovery(1U, 1U);
  const PlanningDecision timeout_reuse =
      timeout_reuse_machine.dispatch(reusable_timeout,
                                     reusable_timeout_context);
  require(timeout_reuse.accepted &&
              timeout_reuse.state == PlanningState::timeout &&
              timeout_reuse.action == PlanningAction::continue_authorized_path &&
              timeout_reuse.directives ==
                  std::vector<PlanningDirective>{
                      PlanningDirective::continue_authorized_path},
          "timeout reuse requires a fresh complete revalidation lease");

  PlanningStateMachine timeout_stop_machine(config);
  PlanningEvent unsafe_timeout =
      event(PlanningEventType::planning_timed_out, 1U, 0);
  PlanningDecisionContext unsafe_timeout_context;
  unsafe_timeout_context.safe_stop = safe_stop(10.0);
  const PlanningDecision timeout_stop =
      timeout_stop_machine.dispatch(unsafe_timeout, unsafe_timeout_context);
  require(timeout_stop.accepted &&
              timeout_stop.state == PlanningState::timeout &&
              timeout_stop.action == PlanningAction::controlled_stop &&
              timeout_stop.directives.front() ==
                  PlanningDirective::revoke_current_lease,
          "timeout without a new lease must revoke and stop");

  struct FailureCase {
    PlanningEventType event_type;
    PlanningState expected_state;
  };
  const std::vector<FailureCase> failures{
      {PlanningEventType::planning_failed, PlanningState::no_solution},
      {PlanningEventType::covariance_solution_unavailable,
       PlanningState::no_solution_under_covariance_envelope},
      {PlanningEventType::covariance_envelope_breached,
       PlanningState::covariance_envelope_breach},
      {PlanningEventType::input_invalid, PlanningState::input_invalid},
      {PlanningEventType::map_expired, PlanningState::map_expired},
  };
  for (const FailureCase failure : failures) {
    PlanningStateMachine machine(config);
    PlanningEvent failure_event = event(failure.event_type, 1U, 0);
    PlanningDecisionContext failure_context;
    failure_context.safe_stop = safe_stop(10.0);
    const PlanningDecision decision =
        machine.dispatch(failure_event, failure_context);
    require(decision.accepted && decision.state == failure.expected_state &&
                decision.action == PlanningAction::controlled_stop &&
                decision.directives.front() ==
                    PlanningDirective::revoke_current_lease,
            "a terminal planning state must revoke authorization and stop");
  }
}

void safety_lease_scout_and_manual_events_have_explicit_transitions() {
  const PlanningStateMachineConfig config = configuration();

  PlanningStateMachine localization_machine(config);
  PlanningEvent localization =
      event(PlanningEventType::localization_invalid, 1U, 0);
  PlanningDecisionContext localization_context;
  localization_context.safe_stop = safe_stop(10.0);
  const PlanningDecision localization_stop =
      localization_machine.dispatch(localization, localization_context);
  require(localization_stop.accepted &&
              localization_stop.state == PlanningState::input_invalid &&
              localization_stop.action == PlanningAction::controlled_stop &&
              localization_stop.directives ==
                  std::vector<PlanningDirective>{
                      PlanningDirective::revoke_current_lease,
                      PlanningDirective::request_controlled_stop},
          "invalid localization must revoke and stop pending fresh evidence");

  PlanningStateMachine renewal_machine(config);
  PlanningEvent renewal =
      event(PlanningEventType::lease_renewal_required, 1U, 0);
  PlanningDecisionContext renewal_context;
  renewal_context.current_lease_live = true;
  const PlanningDecision renew =
      renewal_machine.dispatch(renewal, renewal_context);
  require(renew.accepted && renew.state == PlanningState::normal_planning &&
              renew.action == PlanningAction::continue_authorized_path &&
              renew.directives ==
                  std::vector<PlanningDirective>{
                      PlanningDirective::continue_authorized_path,
                      PlanningDirective::start_planning},
          "a live lease may continue only while complete renewal starts");

  PlanningStateMachine revoked_machine(config);
  PlanningEvent revoked =
      event(PlanningEventType::lease_invalidated, 1U, 0);
  PlanningDecisionContext revoked_context;
  revoked_context.safe_stop = safe_stop(10.0);
  const PlanningDecision revoked_stop =
      revoked_machine.dispatch(revoked, revoked_context);
  require(revoked_stop.accepted &&
              revoked_stop.state == PlanningState::normal_planning &&
              revoked_stop.directives ==
                  std::vector<PlanningDirective>{
                      PlanningDirective::revoke_current_lease,
                      PlanningDirective::request_controlled_stop,
                      PlanningDirective::start_planning},
          "an invalid lease must stop the old path before renewal planning");

  PlanningStateMachine scout_machine(config);
  PlanningEvent scout =
      event(PlanningEventType::scout_requested, 1U, 0);
  PlanningDecisionContext scout_context;
  scout_context.degraded_profile_lease_live = true;
  const PlanningDecision scout_decision =
      scout_machine.dispatch(scout, scout_context);
  require(scout_decision.accepted &&
              scout_decision.state == PlanningState::request_scout &&
              scout_decision.action == PlanningAction::reduce_speed &&
              scout_decision.directives ==
                  std::vector<PlanningDirective>{
                      PlanningDirective::request_scout,
                      PlanningDirective::switch_to_validated_cautious_profile},
          "a scout request may reduce speed only on a validated profile lease");

  PlanningStateMachine manual_machine(config);
  const PlanningDecision manual = manual_machine.dispatch(
      event(PlanningEventType::manual_override_requested, 1U, 0));
  require(manual.accepted && manual.state == PlanningState::manual_override &&
              manual.action == PlanningAction::manual_takeover &&
              manual.directives ==
                  std::vector<PlanningDirective>{
                      PlanningDirective::revoke_current_lease,
                      PlanningDirective::stop_automatic_planning,
                      PlanningDirective::request_manual_takeover},
          "manual override must revoke authorization and stop automatic planning");

  for (const PlanningEventType emergency_type :
       {PlanningEventType::emergency_stop_requested,
        PlanningEventType::critical_system_failure}) {
    PlanningStateMachine emergency_machine(config);
    const PlanningDecision emergency =
        emergency_machine.dispatch(event(emergency_type, 1U, 0));
    require(emergency.accepted &&
                emergency.state == PlanningState::emergency_stop &&
                emergency.action == PlanningAction::emergency_stop &&
                emergency.directives ==
                    std::vector<PlanningDirective>{
                        PlanningDirective::revoke_current_lease,
                        PlanningDirective::request_emergency_stop},
            "critical events must revoke before emergency stop");
  }
}

void consecutive_failures_request_manual_takeover_at_the_configured_limit() {
  const PlanningStateMachineConfig config = configuration();
  PlanningStateMachine machine(config);

  for (std::uint64_t sequence = 1U; sequence <= 2U; ++sequence) {
    PlanningEvent failure =
        event(PlanningEventType::planning_failed, sequence,
              static_cast<std::int64_t>(sequence));
    PlanningDecisionContext failure_context;
    failure_context.safe_stop = safe_stop(10.0);
    const PlanningDecision decision =
        machine.dispatch(failure, failure_context);
    require(decision.state == PlanningState::no_solution,
            "failures below the configured limit must remain NO_SOLUTION");
  }
  PlanningEvent third = event(PlanningEventType::planning_failed, 3U, 3);
  PlanningDecisionContext third_context;
  third_context.safe_stop = safe_stop(10.0);
  const PlanningDecision takeover = machine.dispatch(third, third_context);
  require(takeover.accepted &&
              takeover.state == PlanningState::manual_override &&
              takeover.action == PlanningAction::manual_takeover &&
              takeover.directives ==
                  std::vector<PlanningDirective>{
                      PlanningDirective::revoke_current_lease,
                      PlanningDirective::request_controlled_stop,
                      PlanningDirective::request_manual_takeover},
          "the configured consecutive-failure limit must request manual takeover");
}

void invalid_stop_evidence_fails_closed_and_events_cannot_move_backward() {
  const PlanningStateMachineConfig config = configuration();
  PlanningStateMachine machine(config);

  PlanningEvent uncertified =
      event(PlanningEventType::path_invalidated, 1U, 10);
  PlanningDecisionContext uncertified_context;
  uncertified_context.safe_stop = safe_stop(100.0);
  uncertified_context.safe_stop->terrain_braking_model_certified = false;
  const PlanningDecision rejected_stop =
      machine.dispatch(uncertified, uncertified_context);
  require(rejected_stop.accepted &&
              rejected_stop.state == PlanningState::emergency_stop &&
              rejected_stop.safe_stop_assessment.has_value() &&
              rejected_stop.safe_stop_assessment->status ==
                  SafeStopStatus::terrain_model_uncertified,
          "uncertified terrain braking must not authorize controlled stopping");

  PlanningEvent nonfinite =
      event(PlanningEventType::path_invalidated, 2U, 11);
  PlanningDecisionContext nonfinite_context;
  nonfinite_context.safe_stop = safe_stop(100.0);
  nonfinite_context.safe_stop->current_ground_speed_mps =
      std::numeric_limits<double>::quiet_NaN();
  PlanningStateMachine nonfinite_machine(config);
  const PlanningDecision invalid_stop =
      nonfinite_machine.dispatch(nonfinite, nonfinite_context);
  require(invalid_stop.safe_stop_assessment.has_value() &&
              invalid_stop.safe_stop_assessment->status ==
                  SafeStopStatus::invalid_input &&
              invalid_stop.action == PlanningAction::emergency_stop,
          "non-finite stopping evidence must fail closed");

  const PlanningState before_stale = nonfinite_machine.state();
  const PlanningDecision duplicate = nonfinite_machine.dispatch(
      event(PlanningEventType::periodic_tick, 2U, 12));
  require(!duplicate.accepted && duplicate.state == before_stale &&
              duplicate.action == PlanningAction::none &&
              duplicate.reason_code == "EVENT_SEQUENCE_NOT_NEW",
          "duplicate event sequences must not cause a state rollback");

  const PlanningDecision time_regression = nonfinite_machine.dispatch(
      event(PlanningEventType::periodic_tick, 3U, 10));
  require(!time_regression.accepted && time_regression.state == before_stale &&
              time_regression.reason_code == "EVENT_TIME_REGRESSION",
          "an older event timestamp must not move state backward");
}

void identical_event_streams_produce_identical_decisions() {
  const PlanningStateMachineConfig config = configuration();
  PlanningStateMachine first(config);
  PlanningStateMachine second(config);

  std::vector<PlanningEvent> events;
  std::vector<PlanningDecisionContext> contexts;
  events.push_back(event(PlanningEventType::periodic_tick, 1U, 0));
  contexts.emplace_back();
  PlanningEvent map = event(PlanningEventType::new_map, 2U, 1);
  events.push_back(map);
  PlanningDecisionContext map_context;
  map_context.map_sequence = 9U;
  contexts.push_back(map_context);
  PlanningEvent invalidated =
      event(PlanningEventType::path_invalidated, 3U, 2);
  events.push_back(invalidated);
  PlanningDecisionContext invalidation_context;
  invalidation_context.safe_stop = safe_stop(10.0);
  contexts.push_back(invalidation_context);
  PlanningEvent recovered =
      event(PlanningEventType::planning_succeeded, 4U, 3);
  events.push_back(recovered);
  PlanningDecisionContext recovery_context;
  recovery_context.recovery = recovery(5U, 6U);
  contexts.push_back(recovery_context);

  for (std::size_t index = 0U; index < events.size(); ++index) {
    const PlanningDecision left = first.dispatch(events[index], contexts[index]);
    const PlanningDecision right =
        second.dispatch(events[index], contexts[index]);
    require(left.accepted == right.accepted &&
                left.previous_state == right.previous_state &&
                left.state == right.state && left.action == right.action &&
                left.directives == right.directives &&
                left.reason_code == right.reason_code &&
                left.safe_stop_assessment.has_value() ==
                    right.safe_stop_assessment.has_value() &&
                (!left.safe_stop_assessment.has_value() ||
                 left.safe_stop_assessment->required_stopping_distance_m ==
                     right.safe_stop_assessment->required_stopping_distance_m),
            "identical event streams must produce identical decisions");
  }
}

void invalid_configuration_and_missing_map_sequence_fail_closed() {
  PlanningStateMachineConfig invalid = configuration();
  invalid.short_communication_outage_limit = Duration{-1};
  PlanningStateMachine invalid_machine(invalid);
  const PlanningDecision config_failure = invalid_machine.dispatch(
      event(PlanningEventType::periodic_tick, 1U, 0));
  require(config_failure.accepted &&
              config_failure.state == PlanningState::input_invalid &&
              config_failure.action == PlanningAction::emergency_stop &&
              config_failure.directives ==
                  std::vector<PlanningDirective>{
                      PlanningDirective::revoke_current_lease,
                      PlanningDirective::request_emergency_stop} &&
              config_failure.reason_code == "INVALID_STATE_MACHINE_CONFIG",
          "an incomplete degradation policy must fail closed");

  PlanningStateMachine map_machine(configuration());
  const PlanningDecision missing_map_sequence = map_machine.dispatch(
      event(PlanningEventType::new_map, 1U, 0));
  require(!missing_map_sequence.accepted &&
              missing_map_sequence.state == PlanningState::init &&
              missing_map_sequence.action == PlanningAction::none &&
              missing_map_sequence.reason_code == "MAP_SEQUENCE_INVALID",
          "a map event without a source sequence must not trigger planning");
}

void stale_safety_events_override_ordering_watermarks() {
  const PlanningStateMachineConfig config = configuration();
  PlanningStateMachine emergency_machine(config);
  (void)emergency_machine.dispatch(
      event(PlanningEventType::periodic_tick, 10U, 10));
  const PlanningDecision stale_emergency = emergency_machine.dispatch(
      event(PlanningEventType::emergency_stop_requested, 1U, 1));
  require(stale_emergency.accepted &&
              stale_emergency.state == PlanningState::emergency_stop &&
              stale_emergency.action == PlanningAction::emergency_stop &&
              stale_emergency.directives.front() ==
                  PlanningDirective::revoke_current_lease,
          "a delayed emergency must override event ordering protection");

  PlanningStateMachine path_machine(config);
  (void)path_machine.dispatch(
      event(PlanningEventType::periodic_tick, 10U, 10));
  PlanningDecisionContext stale_path_context;
  stale_path_context.safe_stop = safe_stop(10.0);
  const PlanningDecision stale_path = path_machine.dispatch(
      event(PlanningEventType::path_invalidated, 9U, 9), stale_path_context);
  require(stale_path.accepted &&
              stale_path.action == PlanningAction::controlled_stop &&
              stale_path.directives ==
                  std::vector<PlanningDirective>{
                      PlanningDirective::revoke_current_lease,
                      PlanningDirective::request_controlled_stop,
                      PlanningDirective::start_planning},
          "a delayed path safety event must still revoke before stopping");

  PlanningStateMachine communication_machine(config);
  (void)communication_machine.dispatch(
      event(PlanningEventType::periodic_tick, 10U, 10));
  PlanningDecisionContext communication_context;
  communication_context.communication_outage = Duration{16'000'000'000};
  communication_context.safe_stop = safe_stop(10.0);
  const PlanningDecision stale_communication =
      communication_machine.dispatch(
          event(PlanningEventType::communication_lost, 9U, 9),
          communication_context);
  require(stale_communication.accepted &&
              stale_communication.action == PlanningAction::controlled_stop &&
              stale_communication.directives.front() ==
                  PlanningDirective::revoke_current_lease,
          "a delayed long communication outage must still stop execution");
}

void locked_states_require_explicit_fresh_reauthorization() {
  const PlanningStateMachineConfig config = configuration();
  PlanningStateMachine manual_machine(config);
  (void)manual_machine.dispatch(
      event(PlanningEventType::manual_override_requested, 1U, 1));
  const PlanningDecision periodic = manual_machine.dispatch(
      event(PlanningEventType::periodic_tick, 2U, 2));
  require(periodic.state == PlanningState::manual_override &&
              periodic.action == PlanningAction::manual_takeover &&
              periodic.directives ==
                  std::vector<PlanningDirective>{
                      PlanningDirective::stop_automatic_planning,
                      PlanningDirective::request_manual_takeover},
          "a periodic trigger must not clear manual override");

  PlanningDecisionContext proposed_context;
  proposed_context.recovery = recovery(1U, 1U);
  const PlanningDecision communication = manual_machine.dispatch(
      event(PlanningEventType::communication_restored, 3U, 3),
      proposed_context);
  require(communication.state == PlanningState::manual_override &&
              communication.reason_code == "MANUAL_OVERRIDE_ACTIVE",
          "communication recovery must not clear manual override");
  const PlanningDecision proposed = manual_machine.dispatch(
      event(PlanningEventType::planning_succeeded, 4U, 4), proposed_context);
  require(proposed.state == PlanningState::manual_override &&
              proposed.reason_code == "MANUAL_RELEASE_REQUIRED",
          "a successful plan alone must not clear manual override");
  const PlanningDecision released = manual_machine.dispatch(
      event(PlanningEventType::manual_control_released, 5U, 5),
      proposed_context);
  require(released.state == PlanningState::path_valid &&
              released.action == PlanningAction::continue_authorized_path &&
              released.synchronized_source_revision == 1U &&
              released.lease_sequence == 1U,
          "manual release requires a fresh synchronized snapshot and lease");

  PlanningStateMachine emergency_machine(config);
  (void)emergency_machine.dispatch(
      event(PlanningEventType::emergency_stop_requested, 1U, 1));
  PlanningEvent map = event(PlanningEventType::new_map, 2U, 2);
  PlanningDecisionContext map_context;
  map_context.map_sequence = 1U;
  const PlanningDecision blocked_map =
      emergency_machine.dispatch(map, map_context);
  require(blocked_map.state == PlanningState::emergency_stop &&
              blocked_map.action == PlanningAction::emergency_stop,
          "a new map must not clear emergency stop");
  PlanningDecisionContext cleared_context;
  cleared_context.recovery = recovery(2U, 2U);
  const PlanningDecision cleared = emergency_machine.dispatch(
      event(PlanningEventType::emergency_stop_cleared, 3U, 3),
      cleared_context);
  require(cleared.state == PlanningState::path_valid &&
              cleared.action == PlanningAction::continue_authorized_path,
          "emergency clear requires fresh synchronized authorization");
}

void all_design_replanning_change_triggers_are_explicit() {
  const PlanningStateMachineConfig config = configuration();
  const std::vector<PlanningEventType> trigger_types{
      PlanningEventType::reference_line_changed,
      PlanningEventType::robot_operating_area_changed,
      PlanningEventType::robot_state_changed,
  };
  std::uint64_t sequence = 1U;
  PlanningStateMachine machine(config);
  for (const PlanningEventType trigger : trigger_types) {
    PlanningDecisionContext trigger_context;
    trigger_context.safe_stop = safe_stop(10.0);
    const PlanningDecision decision =
        machine.dispatch(
            event(trigger, sequence, static_cast<std::int64_t>(sequence)),
            trigger_context);
    require(decision.accepted &&
                decision.state == PlanningState::normal_planning &&
                decision.action == PlanningAction::controlled_stop &&
                decision.directives ==
                    std::vector<PlanningDirective>{
                        PlanningDirective::revoke_current_lease,
                        PlanningDirective::request_controlled_stop,
                        PlanningDirective::start_planning},
            "dependency changes must revoke and stop before replanning");
    ++sequence;
  }
}

}  // namespace

int main() {
  periodic_trigger_enters_planning_only_when_due();
  only_a_new_map_sequence_triggers_replanning();
  invalidated_path_revokes_before_stopping_and_replanning();
  communication_outage_applies_the_validated_degradation_policy();
  recovery_requires_a_new_synchronized_snapshot_and_new_lease();
  planning_outcomes_cover_caution_wait_timeout_and_failure_states();
  safety_lease_scout_and_manual_events_have_explicit_transitions();
  consecutive_failures_request_manual_takeover_at_the_configured_limit();
  invalid_stop_evidence_fails_closed_and_events_cannot_move_backward();
  identical_event_streams_produce_identical_decisions();
  invalid_configuration_and_missing_map_sequence_fail_closed();
  stale_safety_events_override_ordering_watermarks();
  locked_states_require_explicit_fresh_reauthorization();
  all_design_replanning_change_triggers_are_explicit();
  std::cout << "planning state machine tests passed\n";
  return EXIT_SUCCESS;
}
