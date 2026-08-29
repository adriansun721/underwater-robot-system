#include "underwater_planner/core/planning_state_machine.hpp"

#include <algorithm>
#include <cmath>

namespace underwater_planner::core {
namespace {

SafeStopAssessment assess_safe_stop(const SafeStopContext& context) {
  SafeStopAssessment result;
  result.remaining_safe_distance_m = context.remaining_safe_distance_m;
  if (!std::isfinite(context.current_ground_speed_mps) ||
      !std::isfinite(context.maximum_braking_deceleration_mps2) ||
      !std::isfinite(context.terrain_limited_braking_deceleration_mps2) ||
      !std::isfinite(context.remaining_safe_distance_m) ||
      !std::isfinite(context.safety_margin_m) ||
      context.current_ground_speed_mps < 0.0 ||
      context.maximum_braking_deceleration_mps2 <= 0.0 ||
      context.terrain_limited_braking_deceleration_mps2 <= 0.0 ||
      context.remaining_safe_distance_m < 0.0 || context.safety_margin_m < 0.0 ||
      context.control_reaction_time.nanoseconds < 0) {
    return result;
  }
  if (!context.terrain_braking_model_certified) {
    result.status = SafeStopStatus::terrain_model_uncertified;
    return result;
  }

  result.effective_braking_deceleration_mps2 =
      std::min(context.maximum_braking_deceleration_mps2,
               context.terrain_limited_braking_deceleration_mps2);
  const double reaction_time_s =
      static_cast<double>(context.control_reaction_time.nanoseconds) / 1.0e9;
  const double speed = context.current_ground_speed_mps;
  result.required_stopping_distance_m =
      speed * reaction_time_s +
      speed * speed / (2.0 * result.effective_braking_deceleration_mps2) +
      context.safety_margin_m;
  result.status = result.required_stopping_distance_m <=
                          context.remaining_safe_distance_m
                      ? SafeStopStatus::feasible
                      : SafeStopStatus::insufficient_distance;
  return result;
}

void append_urgent_stop(PlanningDecision& decision,
                        const std::optional<SafeStopContext>& context,
                        const bool replan) {
  decision.directives.push_back(PlanningDirective::revoke_current_lease);
  if (context.has_value()) {
    decision.safe_stop_assessment = assess_safe_stop(*context);
  } else {
    decision.safe_stop_assessment = SafeStopAssessment{};
  }

  if (decision.safe_stop_assessment->feasible()) {
    decision.action = PlanningAction::controlled_stop;
    decision.directives.push_back(PlanningDirective::request_controlled_stop);
  } else {
    decision.action = PlanningAction::emergency_stop;
    decision.state = PlanningState::emergency_stop;
    decision.directives.push_back(PlanningDirective::request_emergency_stop);
  }
  if (replan) {
    decision.directives.push_back(PlanningDirective::start_planning);
  }
}

bool has_fresh_authorization(
    const std::optional<RecoveryAuthorization>& authorization,
    const std::uint64_t highest_source_revision,
    const std::uint64_t highest_lease_sequence) {
  return authorization.has_value() &&
         authorization->synchronized_snapshot_valid &&
         authorization->lease_live && authorization->dependencies_match &&
         authorization->synchronized_source_revision >
             highest_source_revision &&
         authorization->lease_sequence > highest_lease_sequence;
}

void record_authorization(const RecoveryAuthorization& authorization,
                          std::uint64_t& highest_source_revision,
                          std::uint64_t& highest_lease_sequence,
                          PlanningDecision& decision) {
  highest_source_revision = authorization.synchronized_source_revision;
  highest_lease_sequence = authorization.lease_sequence;
  decision.synchronized_source_revision = highest_source_revision;
  decision.lease_sequence = highest_lease_sequence;
}

bool safety_event_overrides_ordering(const PlanningEventType type) {
  switch (type) {
    case PlanningEventType::path_invalidated:
    case PlanningEventType::communication_lost:
    case PlanningEventType::localization_invalid:
    case PlanningEventType::lease_invalidated:
    case PlanningEventType::covariance_envelope_breached:
    case PlanningEventType::input_invalid:
    case PlanningEventType::map_expired:
    case PlanningEventType::emergency_stop_requested:
    case PlanningEventType::critical_system_failure:
      return true;
    default:
      return false;
  }
}

void hold_locked_state(PlanningDecision& decision, const PlanningState state,
                       const PlanningEventType event_type) {
  decision.state = state;
  if (state == PlanningState::manual_override) {
    decision.action = PlanningAction::manual_takeover;
    decision.directives = {PlanningDirective::stop_automatic_planning,
                           PlanningDirective::request_manual_takeover};
    decision.reason_code = event_type == PlanningEventType::planning_succeeded
                               ? "MANUAL_RELEASE_REQUIRED"
                               : "MANUAL_OVERRIDE_ACTIVE";
    return;
  }
  decision.action = PlanningAction::emergency_stop;
  decision.directives = {PlanningDirective::revoke_current_lease,
                         PlanningDirective::request_emergency_stop};
  decision.reason_code = event_type == PlanningEventType::planning_succeeded
                             ? "EMERGENCY_CLEAR_REQUIRED"
                             : "EMERGENCY_STOP_ACTIVE";
}

}  // namespace

PlanningStateMachine::PlanningStateMachine(
    const PlanningStateMachineConfig config) noexcept
    : config_(config) {}

PlanningDecision PlanningStateMachine::dispatch(
    const PlanningEvent& event, const PlanningDecisionContext& context) {
  PlanningDecision result;
  result.accepted = true;
  result.previous_state = state_;
  result.state = state_;
  result.event_sequence = event.sequence_number;
  result.decided_at = event.observed_at;
  result.state_machine_config_version = config_.version;
  result.parameter_profile_id = config_.parameter_profile_id;
  result.operating_domain_id = config_.operating_domain_id;
  result.synchronized_source_revision = highest_source_revision_;
  result.lease_sequence = highest_lease_sequence_;
  result.reason = event.reason;

  const bool config_valid =
      config_.version > 0U && !config_.parameter_profile_id.empty() &&
      !config_.operating_domain_id.empty() &&
      config_.planning_period.nanoseconds > 0 &&
      config_.maximum_consecutive_failures > 0U &&
      config_.short_communication_outage_limit.nanoseconds > 0 &&
      config_.medium_communication_outage_limit.nanoseconds >
          config_.short_communication_outage_limit.nanoseconds;
  if (!config_valid) {
    result.accepted = true;
    state_ = PlanningState::input_invalid;
    result.state = state_;
    result.action = PlanningAction::emergency_stop;
    result.directives = {PlanningDirective::revoke_current_lease,
                         PlanningDirective::request_emergency_stop};
    result.reason_code = "INVALID_STATE_MACHINE_CONFIG";
    result.reason = "state machine configuration is incomplete or invalid";
    return result;
  }

  const bool safety_override = safety_event_overrides_ordering(event.type);
  if (!safety_override && event.sequence_number == 0U) {
    result.accepted = false;
    result.reason_code = "EVENT_SEQUENCE_INVALID";
    result.reason = "event sequence must be nonzero";
    return result;
  }
  if (!safety_override && event.sequence_number <= highest_event_sequence_) {
    result.accepted = false;
    result.reason_code = "EVENT_SEQUENCE_NOT_NEW";
    result.reason = "event sequence must increase strictly";
    return result;
  }
  if (!safety_override && event.observed_at.nanoseconds < 0) {
    result.accepted = false;
    result.reason_code = "EVENT_TIME_INVALID";
    result.reason = "event timestamp must be monotonic time";
    return result;
  }
  if (!safety_override && last_event_at_.has_value() &&
      event.observed_at.nanoseconds < last_event_at_->nanoseconds) {
    result.accepted = false;
    result.reason_code = "EVENT_TIME_REGRESSION";
    result.reason = "event timestamp cannot move backward";
    return result;
  }
  if (event.type == PlanningEventType::new_map && context.map_sequence == 0U) {
    result.accepted = false;
    result.reason_code = "MAP_SEQUENCE_INVALID";
    result.reason = "new-map events require a nonzero map sequence";
    return result;
  }
  if (!safety_override ||
      (event.sequence_number > highest_event_sequence_ &&
       event.observed_at.nanoseconds >= 0 &&
       (!last_event_at_.has_value() ||
        event.observed_at.nanoseconds >= last_event_at_->nanoseconds))) {
    highest_event_sequence_ = event.sequence_number;
    last_event_at_ = event.observed_at;
  }

  if (state_ == PlanningState::emergency_stop &&
      event.type != PlanningEventType::emergency_stop_cleared) {
    hold_locked_state(result, state_, event.type);
    return result;
  }
  if (state_ == PlanningState::manual_override &&
      event.type != PlanningEventType::manual_control_released) {
    if (event.type == PlanningEventType::emergency_stop_requested ||
        event.type == PlanningEventType::critical_system_failure) {
      // The higher-priority emergency transition is handled below.
    } else if (safety_override) {
      result.state = state_;
      append_urgent_stop(result, context.safe_stop, false);
      state_ = result.state;
      result.reason_code = "SAFETY_EVENT_DURING_MANUAL_OVERRIDE";
      return result;
    } else {
      hold_locked_state(result, state_, event.type);
      return result;
    }
  }

  if (event.type == PlanningEventType::periodic_tick) {
    const bool initial_tick = state_ == PlanningState::init;
    const bool period_due =
        last_plan_trigger_at_.has_value() &&
        event.observed_at.nanoseconds - last_plan_trigger_at_->nanoseconds >=
            config_.planning_period.nanoseconds;
    if (initial_tick || period_due) {
      state_ = PlanningState::normal_planning;
      result.state = state_;
      result.action = PlanningAction::begin_planning;
      result.directives.push_back(PlanningDirective::start_planning);
      result.reason_code =
          initial_tick ? "INITIAL_PLANNING" : "PERIODIC_REPLAN";
      last_plan_trigger_at_ = event.observed_at;
    }
    return result;
  }

  if (event.type == PlanningEventType::new_map) {
    if (context.map_sequence > highest_map_sequence_) {
      highest_map_sequence_ = context.map_sequence;
      state_ = PlanningState::normal_planning;
      result.state = state_;
      append_urgent_stop(result, context.safe_stop, true);
      state_ = result.state;
      result.reason_code = "NEW_MAP";
    } else {
      result.reason_code = "MAP_NOT_NEWER";
    }
    return result;
  }

  if (event.type == PlanningEventType::reference_line_changed ||
      event.type == PlanningEventType::robot_operating_area_changed ||
      event.type == PlanningEventType::robot_state_changed) {
    state_ = PlanningState::normal_planning;
    result.state = state_;
    append_urgent_stop(result, context.safe_stop, true);
    state_ = result.state;
    switch (event.type) {
      case PlanningEventType::reference_line_changed:
        result.reason_code = "REFERENCE_LINE_CHANGED";
        break;
      case PlanningEventType::robot_operating_area_changed:
        result.reason_code = "ROBOT_OPERATING_AREA_CHANGED";
        break;
      case PlanningEventType::robot_state_changed:
        result.reason_code = "ROBOT_STATE_CHANGED";
        break;
      default:
        break;
    }
    return result;
  }

  if (event.type == PlanningEventType::path_invalidated) {
    state_ = PlanningState::normal_planning;
    result.state = state_;
    append_urgent_stop(result, context.safe_stop, true);
    state_ = result.state;
    result.reason_code = result.safe_stop_assessment->feasible()
                             ? "PATH_INVALIDATED_CONTROLLED_STOP"
                             : "PATH_INVALIDATED_EMERGENCY_STOP";
    return result;
  }

  if (event.type == PlanningEventType::communication_lost) {
    state_ = PlanningState::communication_degraded;
    result.state = state_;
    const bool outage_valid = context.communication_outage.nanoseconds >= 0;
    if (outage_valid &&
        context.communication_outage.nanoseconds <
            config_.short_communication_outage_limit.nanoseconds &&
        context.current_lease_live && context.degraded_sensor_mode_approved) {
      result.action = PlanningAction::continue_authorized_path;
      result.directives.push_back(
          PlanningDirective::continue_authorized_path);
      result.reason_code = "SHORT_COMMUNICATION_DEGRADATION";
      return result;
    }
    if (outage_valid &&
        context.communication_outage.nanoseconds >=
            config_.short_communication_outage_limit.nanoseconds &&
        context.communication_outage.nanoseconds <=
            config_.medium_communication_outage_limit.nanoseconds &&
        context.degraded_profile_lease_live) {
      result.action = PlanningAction::reduce_speed;
      result.directives.push_back(
          PlanningDirective::switch_to_validated_cautious_profile);
      result.reason_code = "MEDIUM_COMMUNICATION_DEGRADATION";
      return result;
    }

    append_urgent_stop(result, context.safe_stop, false);
    state_ = result.state;
    result.reason_code = outage_valid ? "COMMUNICATION_STOP_REQUIRED"
                                      : "COMMUNICATION_OUTAGE_INVALID";
    return result;
  }

  if (event.type == PlanningEventType::planning_succeeded) {
    const bool fresh_authorization = has_fresh_authorization(
        context.recovery, highest_source_revision_, highest_lease_sequence_);
    if (!fresh_authorization) {
      state_ = PlanningState::input_invalid;
      result.state = state_;
      append_urgent_stop(result, context.safe_stop, false);
      state_ = result.state;
      result.reason_code = "AUTHORIZATION_EVIDENCE_INVALID";
      return result;
    }
    record_authorization(*context.recovery, highest_source_revision_,
                         highest_lease_sequence_, result);
    consecutive_failures_ = 0U;
    state_ = PlanningState::success;
    result.state = state_;
    result.action = PlanningAction::continue_authorized_path;
    result.directives.push_back(PlanningDirective::continue_authorized_path);
    result.reason_code = "PLANNING_SUCCEEDED";
    return result;
  }

  if (event.type == PlanningEventType::communication_restored) {
    const bool fresh_authorization = has_fresh_authorization(
        context.recovery, highest_source_revision_, highest_lease_sequence_);
    if (!fresh_authorization) {
      state_ = PlanningState::communication_degraded;
      result.state = state_;
      append_urgent_stop(result, context.safe_stop, false);
      state_ = result.state;
      result.reason_code = "RECOVERY_EVIDENCE_NOT_NEW";
      return result;
    }
    record_authorization(*context.recovery, highest_source_revision_,
                         highest_lease_sequence_, result);
    state_ = PlanningState::path_valid;
    result.state = state_;
    result.action = PlanningAction::continue_authorized_path;
    result.directives.push_back(PlanningDirective::continue_authorized_path);
    result.directives.push_back(PlanningDirective::start_planning);
    result.reason_code = "COMMUNICATION_RECOVERED";
    return result;
  }

  if (event.type == PlanningEventType::planning_with_caution) {
    const bool fresh_authorization = has_fresh_authorization(
        context.recovery, highest_source_revision_, highest_lease_sequence_);
    if (!fresh_authorization) {
      state_ = PlanningState::input_invalid;
      result.state = state_;
      append_urgent_stop(result, context.safe_stop, false);
      state_ = result.state;
      result.reason_code = "CAUTIOUS_PROFILE_NOT_AUTHORIZED";
      return result;
    }
    record_authorization(*context.recovery, highest_source_revision_,
                         highest_lease_sequence_, result);
    consecutive_failures_ = 0U;
    state_ = PlanningState::planning_with_caution;
    result.state = state_;
    result.action = PlanningAction::reduce_speed;
    result.directives.push_back(
        PlanningDirective::switch_to_validated_cautious_profile);
    result.reason_code = "PLANNING_WITH_CAUTION";
    return result;
  }

  if (event.type == PlanningEventType::waiting_for_map) {
    state_ = PlanningState::waiting_map;
    result.state = state_;
    if (context.degraded_profile_lease_live) {
      result.action = PlanningAction::reduce_speed;
      result.directives = {
          PlanningDirective::request_scout,
          PlanningDirective::switch_to_validated_cautious_profile};
      result.reason_code = "WAITING_FOR_MAP_WITH_CAUTION";
      return result;
    }
    append_urgent_stop(result, context.safe_stop, false);
    state_ = result.state;
    result.directives.push_back(PlanningDirective::request_scout);
    result.reason_code = "WAITING_FOR_MAP";
    return result;
  }

  if (event.type == PlanningEventType::planning_timed_out) {
    state_ = PlanningState::timeout;
    result.state = state_;
    const bool fresh_authorization = has_fresh_authorization(
        context.recovery, highest_source_revision_, highest_lease_sequence_);
    if (fresh_authorization) {
      record_authorization(*context.recovery, highest_source_revision_,
                           highest_lease_sequence_, result);
      result.action = PlanningAction::continue_authorized_path;
      result.directives.push_back(
          PlanningDirective::continue_authorized_path);
      result.reason_code = "TIMEOUT_REVALIDATED_PATH";
      return result;
    }
    append_urgent_stop(result, context.safe_stop, false);
    state_ = result.state;
    result.reason_code = "TIMEOUT_STOP_REQUIRED";
    return result;
  }

  if (event.type == PlanningEventType::localization_invalid) {
    state_ = PlanningState::input_invalid;
    result.state = state_;
    append_urgent_stop(result, context.safe_stop, false);
    state_ = result.state;
    result.reason_code = "LOCALIZATION_INVALID";
    return result;
  }

  if (event.type == PlanningEventType::lease_renewal_required) {
    state_ = PlanningState::normal_planning;
    result.state = state_;
    if (context.current_lease_live) {
      result.action = PlanningAction::continue_authorized_path;
      result.directives.push_back(
          PlanningDirective::continue_authorized_path);
      result.directives.push_back(PlanningDirective::start_planning);
      result.reason_code = "LEASE_RENEWAL_REQUIRED";
      return result;
    }
    append_urgent_stop(result, context.safe_stop, true);
    state_ = result.state;
    result.reason_code = "LEASE_RENEWAL_WITHOUT_LIVE_LEASE";
    return result;
  }

  if (event.type == PlanningEventType::lease_invalidated) {
    state_ = PlanningState::normal_planning;
    result.state = state_;
    append_urgent_stop(result, context.safe_stop, true);
    state_ = result.state;
    result.reason_code = "LEASE_INVALIDATED";
    return result;
  }

  if (event.type == PlanningEventType::scout_requested) {
    state_ = PlanningState::request_scout;
    result.state = state_;
    result.directives.push_back(PlanningDirective::request_scout);
    if (context.degraded_profile_lease_live) {
      result.action = PlanningAction::reduce_speed;
      result.directives.push_back(
          PlanningDirective::switch_to_validated_cautious_profile);
      result.reason_code = "SCOUT_REQUEST_WITH_CAUTION";
      return result;
    }
    result.directives.clear();
    append_urgent_stop(result, context.safe_stop, false);
    state_ = result.state;
    result.directives.push_back(PlanningDirective::request_scout);
    result.reason_code = "SCOUT_REQUEST_STOP_REQUIRED";
    return result;
  }

  if (event.type == PlanningEventType::manual_override_requested) {
    state_ = PlanningState::manual_override;
    result.state = state_;
    result.action = PlanningAction::manual_takeover;
    result.directives = {PlanningDirective::revoke_current_lease,
                         PlanningDirective::stop_automatic_planning,
                         PlanningDirective::request_manual_takeover};
    result.reason_code = "MANUAL_OVERRIDE";
    return result;
  }

  if (event.type == PlanningEventType::manual_control_released ||
      event.type == PlanningEventType::emergency_stop_cleared) {
    const PlanningState required_state =
        event.type == PlanningEventType::manual_control_released
            ? PlanningState::manual_override
            : PlanningState::emergency_stop;
    if (state_ != required_state) {
      result.accepted = false;
      result.reason_code = "RELEASE_STATE_MISMATCH";
      result.reason = "release event does not match the active locked state";
      return result;
    }
    if (!has_fresh_authorization(context.recovery, highest_source_revision_,
                                 highest_lease_sequence_)) {
      hold_locked_state(result, state_, event.type);
      result.reason_code = "RELEASE_AUTHORIZATION_NOT_NEW";
      return result;
    }
    record_authorization(*context.recovery, highest_source_revision_,
                         highest_lease_sequence_, result);
    state_ = PlanningState::path_valid;
    result.state = state_;
    result.action = PlanningAction::continue_authorized_path;
    result.directives = {PlanningDirective::continue_authorized_path};
    result.reason_code = event.type == PlanningEventType::manual_control_released
                             ? "MANUAL_CONTROL_RELEASED"
                             : "EMERGENCY_STOP_CLEARED";
    return result;
  }

  if (event.type == PlanningEventType::emergency_stop_requested ||
      event.type == PlanningEventType::critical_system_failure) {
    state_ = PlanningState::emergency_stop;
    result.state = state_;
    result.action = PlanningAction::emergency_stop;
    result.directives = {PlanningDirective::revoke_current_lease,
                         PlanningDirective::request_emergency_stop};
    result.reason_code = event.type == PlanningEventType::emergency_stop_requested
                             ? "EMERGENCY_STOP_REQUESTED"
                             : "CRITICAL_SYSTEM_FAILURE";
    return result;
  }

  PlanningState failure_state = PlanningState::input_invalid;
  bool is_terminal_failure = true;
  switch (event.type) {
    case PlanningEventType::planning_failed:
      failure_state = PlanningState::no_solution;
      ++consecutive_failures_;
      break;
    case PlanningEventType::covariance_solution_unavailable:
      failure_state = PlanningState::no_solution_under_covariance_envelope;
      ++consecutive_failures_;
      break;
    case PlanningEventType::covariance_envelope_breached:
      failure_state = PlanningState::covariance_envelope_breach;
      break;
    case PlanningEventType::input_invalid:
      failure_state = PlanningState::input_invalid;
      break;
    case PlanningEventType::map_expired:
      failure_state = PlanningState::map_expired;
      break;
    default:
      is_terminal_failure = false;
      break;
  }
  if (is_terminal_failure) {
    state_ = failure_state;
    result.state = state_;
    append_urgent_stop(result, context.safe_stop, false);
    state_ = result.state;
    if ((event.type == PlanningEventType::planning_failed ||
         event.type == PlanningEventType::covariance_solution_unavailable) &&
        config_.maximum_consecutive_failures > 0U &&
        consecutive_failures_ >= config_.maximum_consecutive_failures) {
      state_ = PlanningState::manual_override;
      result.state = state_;
      result.action = PlanningAction::manual_takeover;
      result.directives.push_back(PlanningDirective::request_manual_takeover);
      result.reason_code = "CONSECUTIVE_FAILURE_LIMIT";
      return result;
    }
    result.reason_code = "TERMINAL_PLANNING_STATE";
    return result;
  }

  result.accepted = false;
  result.reason_code = "EVENT_NOT_IMPLEMENTED";
  result.reason = "the event is not implemented by this state machine slice";
  return result;
}

}  // namespace underwater_planner::core
