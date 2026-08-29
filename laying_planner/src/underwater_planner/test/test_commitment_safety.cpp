#include "underwater_planner/core/commitment_safety.hpp"

#include <iostream>
#include <stdexcept>

namespace {
using namespace underwater_planner::core;

void require(const bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

PathCandidateVerificationResult valid_robot() {
  PathCandidateVerificationResult result;
  result.status = PathCandidateVerificationStatus::valid;
  result.valid = true;
  return result;
}

TimedCableCandidateResult valid_cable() {
  TimedCableCandidateResult result;
  result.status = TimedCableValidationStatus::valid;
  result.valid = true;
  return result;
}

void full_validation_and_event_actions_are_deterministic() {
  CommitmentSafetyEvaluator evaluator;
  CommitmentSafetyObservation observation;
  observation.robot_validation = valid_robot();
  observation.cable_validation = valid_cable();
  const auto safe = evaluator.evaluate(observation);
  require(safe.is_safe &&
              safe.action == CommitmentSafetyAction::continue_commitment &&
              safe.event == CommitmentSafetyEvent::none,
          "fully validated commitment was not allowed to continue");
  observation.cable_validation.reset();
  const auto incomplete = evaluator.evaluate(observation);
  require(incomplete.event == CommitmentSafetyEvent::validation_unavailable &&
              incomplete.action == CommitmentSafetyAction::stop,
          "incomplete full-constraint evidence did not fail closed");
  observation.cable_validation = valid_cable();

  struct Scenario {
    CommitmentSafetyEvent event;
    CommitmentSafetyAction expected_action;
    const char* expected_code;
  };
  const Scenario scenarios[] = {
      {CommitmentSafetyEvent::terrain_constraint_change,
       CommitmentSafetyAction::replan_urgent,
       "COMMITMENT_TERRAIN_CONSTRAINT_CHANGE"},
      {CommitmentSafetyEvent::execution_deviation,
       CommitmentSafetyAction::replan_urgent,
       "COMMITMENT_EXECUTION_DEVIATION"},
      {CommitmentSafetyEvent::dependency_version_change,
       CommitmentSafetyAction::replan_urgent,
       "COMMITMENT_DEPENDENCY_VERSION_CHANGE"},
      {CommitmentSafetyEvent::localization_jump,
       CommitmentSafetyAction::stop, "COMMITMENT_LOCALIZATION_JUMP"},
      {CommitmentSafetyEvent::cable_state_anomaly,
       CommitmentSafetyAction::stop, "COMMITMENT_CABLE_STATE_ANOMALY"},
      {CommitmentSafetyEvent::emergency_stop,
       CommitmentSafetyAction::stop, "COMMITMENT_EMERGENCY_STOP"},
  };
  for (const Scenario& scenario : scenarios) {
    observation.observed_event = scenario.event;
    const auto result = evaluator.evaluate(observation);
    require(!result.is_safe && result.event == scenario.event &&
                result.action == scenario.expected_action &&
                commitment_safety_reason_code(result.event) ==
                    scenario.expected_code,
            "safety event action or typed reason mapping was incorrect");
  }
}

void strongest_safety_action_wins() {
  CommitmentSafetyObservation observation;
  observation.robot_validation = valid_robot();
  TimedCableCandidateResult cable = valid_cable();
  cable.stop_required = true;
  observation.cable_validation = cable;
  observation.observed_event = CommitmentSafetyEvent::execution_deviation;

  const auto result = CommitmentSafetyEvaluator().evaluate(observation);
  require(result.event == CommitmentSafetyEvent::cable_state_anomaly &&
              result.action == CommitmentSafetyAction::stop,
          "urgent replan event masked a STOP-grade cable failure");
}

void obstacle_action_uses_certified_stopping_evidence() {
  CommitmentSafetyObservation observation;
  PathCandidateVerificationResult robot = valid_robot();
  robot.status = PathCandidateVerificationStatus::collision_violation;
  robot.valid = false;
  observation.robot_validation = robot;
  observation.cable_validation = valid_cable();

  observation.obstacle_stopping = ObstacleStoppingEvidence{2.0, 1.0};
  require(CommitmentSafetyEvaluator().evaluate(observation).action ==
              CommitmentSafetyAction::replan_urgent,
          "stoppable obstacle did not request urgent replanning");
  observation.obstacle_stopping = ObstacleStoppingEvidence{1.0, 1.0};
  require(CommitmentSafetyEvaluator().evaluate(observation).action ==
              CommitmentSafetyAction::stop,
          "too-close obstacle did not request STOP");
  observation.obstacle_stopping.reset();
  require(CommitmentSafetyEvaluator().evaluate(observation).action ==
              CommitmentSafetyAction::stop,
          "missing stopping evidence did not fail closed");
}

void asynchronous_supervisor_revokes_before_stop_channel() {
  // Design: 18.2.7-18
  ExecutionLeaseMonitor lease_monitor;
  bool stop_requested = false;
  bool revoked_before_stop = false;
  CommitmentSafetySupervisor supervisor(
      lease_monitor,
      [&](const CommitmentSafetyCheckResult&, MonotonicTime) {
        stop_requested = true;
        revoked_before_stop = lease_monitor.isRevoked(55);
      });
  CommitmentSafetyObservation event;
  event.observed_event = CommitmentSafetyEvent::execution_deviation;
  const CommitmentSafetyEnforcement result =
      supervisor.handle_event(event, 55, {1'000});

  require(result.safety.action == CommitmentSafetyAction::replan_urgent &&
              result.revoked_lease_sequence == 55 &&
              result.stop_channel_requested && stop_requested &&
              revoked_before_stop && lease_monitor.isRevoked(55),
          "asynchronous event did not revoke before requesting safe stop");
}

void supervisor_rejects_a_missing_stop_channel() {
  ExecutionLeaseMonitor lease_monitor;
  bool rejected = false;
  try {
    CommitmentSafetySupervisor supervisor(lease_monitor, {});
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected,
          "commitment safety supervisor accepted a missing stop channel");
}

}  // namespace

int main() {
  try {
    full_validation_and_event_actions_are_deterministic();
    strongest_safety_action_wins();
    obstacle_action_uses_certified_stopping_evidence();
    asynchronous_supervisor_revokes_before_stop_channel();
    supervisor_rejects_a_missing_stop_channel();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
