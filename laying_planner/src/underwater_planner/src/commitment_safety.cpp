#include "underwater_planner/core/commitment_safety.hpp"

#include <cstddef>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace underwater_planner::core {
namespace {

struct EventPolicy {
  CommitmentSafetyAction action;
  int priority;
  std::string_view reason_code;
};

constexpr EventPolicy kEventPolicies[] = {
    {CommitmentSafetyAction::continue_commitment, 0, "COMMITMENT_SAFE"},
    {CommitmentSafetyAction::stop, 9, "COMMITMENT_EMERGENCY_STOP"},
    {CommitmentSafetyAction::replan_urgent, 4, "COMMITMENT_NEW_OBSTACLE"},
    {CommitmentSafetyAction::replan_urgent, 3,
     "COMMITMENT_TERRAIN_CONSTRAINT_CHANGE"},
    {CommitmentSafetyAction::stop, 8, "COMMITMENT_LOCALIZATION_JUMP"},
    {CommitmentSafetyAction::stop, 7, "COMMITMENT_CABLE_STATE_ANOMALY"},
    {CommitmentSafetyAction::replan_urgent, 2,
     "COMMITMENT_EXECUTION_DEVIATION"},
    {CommitmentSafetyAction::replan_urgent, 1,
     "COMMITMENT_DEPENDENCY_VERSION_CHANGE"},
    {CommitmentSafetyAction::stop, 5,
     "COMMITMENT_ROBOT_CONSTRAINT_VIOLATION"},
    {CommitmentSafetyAction::stop, 6,
     "COMMITMENT_VALIDATION_UNAVAILABLE"},
};

static_assert(
    sizeof(kEventPolicies) / sizeof(kEventPolicies[0]) ==
        static_cast<std::size_t>(
            CommitmentSafetyEvent::validation_unavailable) +
            1U,
    "Every commitment safety event must have an auditable policy");

const EventPolicy& event_policy(const CommitmentSafetyEvent event) noexcept {
  const auto index = static_cast<std::size_t>(event);
  constexpr std::size_t policy_count =
      sizeof(kEventPolicies) / sizeof(kEventPolicies[0]);
  return index < policy_count
             ? kEventPolicies[index]
             : kEventPolicies[static_cast<std::size_t>(
                   CommitmentSafetyEvent::validation_unavailable)];
}

int action_priority(const CommitmentSafetyAction action) noexcept {
  switch (action) {
    case CommitmentSafetyAction::continue_commitment:
      return 0;
    case CommitmentSafetyAction::replan_urgent:
      return 1;
    case CommitmentSafetyAction::stop:
      return 2;
  }
  return 2;
}

CommitmentSafetyAction obstacle_action(
    const std::optional<ObstacleStoppingEvidence>& evidence) noexcept {
  if (!evidence.has_value() ||
      !std::isfinite(evidence->obstacle_distance_m) ||
      !std::isfinite(evidence->certified_stopping_distance_m) ||
      evidence->obstacle_distance_m < 0.0 ||
      evidence->certified_stopping_distance_m < 0.0) {
    return CommitmentSafetyAction::stop;
  }
  return evidence->certified_stopping_distance_m <
                 evidence->obstacle_distance_m
             ? CommitmentSafetyAction::replan_urgent
             : CommitmentSafetyAction::stop;
}

CommitmentSafetyAction action_for_event(
    const CommitmentSafetyEvent event,
    const std::optional<ObstacleStoppingEvidence>& obstacle_stopping) noexcept {
  return event == CommitmentSafetyEvent::new_obstacle
             ? obstacle_action(obstacle_stopping)
             : event_policy(event).action;
}

void consider(CommitmentSafetyCheckResult& result,
              const CommitmentSafetyEvent event,
              const std::optional<ObstacleStoppingEvidence>& stopping) noexcept {
  if (event == CommitmentSafetyEvent::none) return;
  const CommitmentSafetyAction action = action_for_event(event, stopping);
  const int candidate_action_priority = action_priority(action);
  const int current_action_priority = action_priority(result.action);
  if (result.is_safe || candidate_action_priority > current_action_priority ||
      (candidate_action_priority == current_action_priority &&
       event_policy(event).priority > event_policy(result.event).priority)) {
    result.is_safe = false;
    result.event = event;
    result.action = action;
  }
}

}  // namespace

std::string_view commitment_safety_reason_code(
    const CommitmentSafetyEvent event) noexcept {
  return event_policy(event).reason_code;
}

CommitmentSafetyCheckResult CommitmentSafetyEvaluator::evaluate(
    const CommitmentSafetyObservation& observation) const noexcept {
  CommitmentSafetyCheckResult result;
  result.is_safe = true;
  result.event = CommitmentSafetyEvent::none;
  result.action = CommitmentSafetyAction::continue_commitment;

  consider(result, observation.observed_event,
           observation.obstacle_stopping);

  if (observation.robot_validation.has_value()) {
    const PathCandidateVerificationResult& robot =
        *observation.robot_validation;
    if (!robot.valid ||
        robot.status != PathCandidateVerificationStatus::valid) {
      CommitmentSafetyEvent event =
          CommitmentSafetyEvent::robot_constraint_violation;
      if (robot.status == PathCandidateVerificationStatus::collision_violation) {
        event = CommitmentSafetyEvent::new_obstacle;
      } else if (robot.status ==
                 PathCandidateVerificationStatus::traversability_violation) {
        event = CommitmentSafetyEvent::terrain_constraint_change;
      }
      consider(result, event, observation.obstacle_stopping);
    }
  }

  if (observation.cable_validation.has_value()) {
    const TimedCableCandidateResult& cable =
        *observation.cable_validation;
    if (!cable.valid || cable.status != TimedCableValidationStatus::valid ||
        cable.stop_required) {
      consider(result, CommitmentSafetyEvent::cable_state_anomaly,
               observation.obstacle_stopping);
    }
  }

  const bool complete_validation =
      observation.robot_validation.has_value() &&
      observation.cable_validation.has_value();
  if (!complete_validation &&
      observation.observed_event == CommitmentSafetyEvent::none) {
    consider(result, CommitmentSafetyEvent::validation_unavailable,
             observation.obstacle_stopping);
  }
  return result;
}

CommitmentSafetySupervisor::CommitmentSafetySupervisor(
    ExecutionLeaseMonitor& lease_monitor,
    CommitmentSafetyStopChannel stop_channel)
    : lease_monitor_(lease_monitor), stop_channel_(std::move(stop_channel)) {
  if (!stop_channel_) {
    throw std::invalid_argument(
        "commitment safety stop channel must be configured");
  }
}

CommitmentSafetyEnforcement CommitmentSafetySupervisor::handle_event(
    const CommitmentSafetyObservation& observation,
    const std::uint64_t active_lease_sequence,
    const MonotonicTime now) const {
  return enforce(evaluator_.evaluate(observation), active_lease_sequence, now);
}

CommitmentSafetyEnforcement CommitmentSafetySupervisor::enforce(
    const CommitmentSafetyCheckResult& safety,
    const std::uint64_t active_lease_sequence,
    const MonotonicTime now) const {
  CommitmentSafetyEnforcement result;
  result.safety = safety;
  if (safety.is_safe) return result;

  const std::string reason_code{commitment_safety_reason_code(safety.event)};
  if (active_lease_sequence != 0U) {
    lease_monitor_.revokeLease(
        active_lease_sequence, reason_code,
        "commitment safety event overrode authorization", now);
    result.revoked_lease_sequence = active_lease_sequence;
  }
  if (stop_channel_) {
    stop_channel_(safety, now);
    result.stop_channel_requested = true;
  }
  return result;
}

}  // namespace underwater_planner::core
