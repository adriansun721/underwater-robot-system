#include "underwater_planner/core/execution_lease_monitor.hpp"

#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using namespace underwater_planner::core;

constexpr MonotonicTime kNow{2'000'000'000};

void require(const bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

TimedPath trajectory() {
  TimedPath path;
  path.geometry.metadata = {12, "map", 4, "linear"};
  path.geometry.points = {{0.0, 0.0, 0.0, 0.0, 0.0},
                          {2.0, 2.0, 0.0, 0.0, 0.0}};
  path.execution_profile.version = 41;
  path.execution_profile.operating_envelope_version = 7;
  path.execution_profile.interpolation_rule = "linear-in-arc-length";
  path.execution_profile.approved_tracking_limits.ground_speed = {0.0, 1.0};
  path.execution_profile.approved_tracking_limits.ground_acceleration = {-0.5, 0.5};
  path.execution_profile.approved_tracking_limits.payout_speed = {0.0, 1.0};
  path.execution_profile.approved_tracking_limits.payout_acceleration = {-0.5, 0.5};
  path.execution_profile.approved_tracking_limits.maximum_payout_tracking_error_mps =
      0.1;
  path.execution_profile.approved_tracking_limits.tension = {20.0, 80.0};
  path.execution_profile.samples = {
      {0.0, {1'000'000'000}, 0.5, 0.0, 0.5, 0.0, 40.0},
      {2.0, {3'000'000'000}, 0.7, 0.2, 0.7, 0.2, 50.0}};
  return path;
}

PlanningResult plan_for(const TimedPath& path) {
  PlanningResult plan;
  plan.sequence_number = 12;
  plan.execution_profile_version = path.execution_profile.version;
  plan.execution_operating_envelope_version =
      path.execution_profile.operating_envelope_version;
  plan.robot_trajectory = path;
  plan.map_version = {"map", 3, {1'000'000'000}, "map"};
  plan.reference_line_version = 4;
  plan.robot_operating_area_version = 5;
  plan.terrain_gradient_policy_version = 6;
  plan.corridor_risk_policy_version = 8;
  plan.cable_model_version = 9;
  plan.uncertainty_envelope_version = 10;
  plan.uncertainty_envelope_generator_version = 11;
  plan.sensor_mode = SensorHealthMode::nominal;
  plan.operating_domain_id = "domain-v1";
  plan.cable_corridor_version = 12;
  return plan;
}

ActiveExecutionContext context_for(const PlanningResult& plan) {
  ActiveExecutionContext context;
  context.map_version = plan.map_version;
  context.reference_line_version = plan.reference_line_version;
  context.robot_operating_area_version = plan.robot_operating_area_version;
  context.terrain_gradient_policy_version = plan.terrain_gradient_policy_version;
  context.corridor_risk_policy_version = plan.corridor_risk_policy_version;
  context.cable_model_version = plan.cable_model_version;
  context.uncertainty_envelope_version = plan.uncertainty_envelope_version;
  context.uncertainty_envelope_generator_version =
      plan.uncertainty_envelope_generator_version;
  context.execution_operating_envelope_version =
      plan.execution_operating_envelope_version;
  context.execution_profile_version = plan.execution_profile_version;
  context.sensor_mode = plan.sensor_mode;
  context.operating_domain_id = plan.operating_domain_id;
  context.cable_corridor_version = plan.cable_corridor_version;
  return context;
}

PlanValidationLease lease_for(const PlanningResult& plan) {
  PlanValidationLease lease;
  lease.lease_sequence = 1;
  lease.plan_sequence_number = plan.sequence_number;
  lease.validated_at = {1'000'000'000};
  lease.expires_at = {5'000'000'000};
  lease.remaining_path_start_arc_length_m = 0.0;
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
  lease.operating_domain_id = plan.operating_domain_id;
  lease.cable_corridor_version = plan.cable_corridor_version;
  lease.max_ground_speed_tracking_error_mps = 0.1;
  lease.max_payout_speed_tracking_error_mps = 0.1;
  lease.allowed_ground_acceleration = {-0.5, 0.5};
  lease.allowed_tension = {20.0, 80.0};
  return lease;
}

ExecutionFeedback feedback(const double ground_speed = 0.6) {
  ExecutionFeedback value;
  value.plan_sequence_number = 12;
  value.execution_profile_version = 41;
  value.timestamp = {1'500'000'000};
  value.ground_speed_mps = ground_speed;
  value.ground_acceleration_mps2 = 0.1;
  value.payout_speed_mps = 0.6;
  value.payout_acceleration_mps2 = 0.1;
  value.tension_n = 45.0;
  value.tracked_arc_length_m = 1.0;
  value.sequence_number = 1;
  return value;
}

void authorized_and_interpolated_profile() {
  const TimedPath path = trajectory();
  const PlanningResult plan = plan_for(path);
  ExecutionLeaseMonitor monitor;
  const ExecutionAuthorization result = monitor.evaluate(
      plan, path, lease_for(plan), context_for(plan), feedback(), kNow);
  require(result.authorized(), "in-range execution feedback was rejected");
  require(result.reason_code == "LEASE_AUTHORIZED",
          "authorized result lost its audit reason");
}

void deviation_revokes_and_stale_lease_is_rejected() {
  // Design: 18.2.7-11
  const TimedPath path = trajectory();
  const PlanningResult plan = plan_for(path);
  struct Scenario {
    std::function<void(ExecutionFeedback&)> inject;
    const char* reason_code;
  };
  const Scenario scenarios[] = {
      {[](ExecutionFeedback& value) { value.ground_speed_mps = 0.9; },
       "GROUND_SPEED_TRACKING_BREACH"},
      {[](ExecutionFeedback& value) { value.payout_speed_mps = 0.9; },
       "PAYOUT_SPEED_TRACKING_BREACH"},
      {[](ExecutionFeedback& value) {
         value.ground_acceleration_mps2 = 0.4;
       },
       "GROUND_ACCELERATION_TRACKING_BREACH"},
      {[](ExecutionFeedback& value) {
         value.payout_acceleration_mps2 = 0.4;
       },
       "PAYOUT_ACCELERATION_TRACKING_BREACH"},
      {[](ExecutionFeedback& value) { value.tension_n = 48.0; },
       "TENSION_TRACKING_BREACH"},
  };
  for (const Scenario& scenario : scenarios) {
    ExecutionLeaseMonitor monitor;
    const PlanValidationLease lease = lease_for(plan);
    ExecutionFeedback injected = feedback();
    scenario.inject(injected);
    const ExecutionAuthorization breach = monitor.evaluate(
        plan, path, lease, context_for(plan), injected, kNow);
    require(breach.revoked() && breach.request_controlled_stop &&
                breach.request_replan &&
                breach.reason_code == scenario.reason_code &&
                monitor.isRevoked(lease.lease_sequence),
            "tracking breach did not revoke and request controlled stop");
    const ExecutionAuthorization delayed = monitor.evaluate(
        plan, path, lease, context_for(plan), feedback(), kNow);
    require(delayed.reason_code == "LEASE_ALREADY_REVOKED",
            "revoked lease was accepted after delayed feedback");
  }
}

void versions_order_and_renewal_are_enforced() {
  const TimedPath path = trajectory();
  const PlanningResult plan = plan_for(path);
  ExecutionLeaseMonitor monitor;
  PlanValidationLease newer = lease_for(plan);
  newer.lease_sequence = 2;
  const ExecutionAuthorization accepted = monitor.evaluate(
      plan, path, newer, context_for(plan), feedback(), kNow);
  require(accepted.authorized(), "newer lease was rejected");
  const PlanValidationLease older = lease_for(plan);
  const ExecutionAuthorization out_of_order = monitor.evaluate(
      plan, path, older, context_for(plan), feedback(), kNow);
  require(out_of_order.reason_code == "LEASE_SEQUENCE_OUT_OF_ORDER",
          "older lease was not rejected");
  const MonotonicTime renewal_time{4'500'000'000};
  ExecutionFeedback renewal_feedback = feedback();
  renewal_feedback.timestamp = {4'400'000'000};
  const ExecutionAuthorization renewal = monitor.evaluate(
      plan, path, newer, context_for(plan), renewal_feedback, renewal_time);
  require(renewal.renewalRequired(), "renewal margin did not request revalidation");
}

void context_changes_revoke_and_block_subsequent_commands() {
  // Design: 18.2.7-5
  // Design: 18.2.7-10
  const TimedPath path = trajectory();
  const PlanningResult plan = plan_for(path);
  using Mutation = std::function<void(ActiveExecutionContext&)>;
  const std::vector<Mutation> mutations{
      [](ActiveExecutionContext& value) {
        ++value.map_version.sequence_number;
      },
      [](ActiveExecutionContext& value) { ++value.reference_line_version; },
      [](ActiveExecutionContext& value) {
        ++value.robot_operating_area_version;
      },
      [](ActiveExecutionContext& value) { ++value.cable_corridor_version; },
      [](ActiveExecutionContext& value) {
        ++value.terrain_gradient_policy_version;
      },
      [](ActiveExecutionContext& value) {
        ++value.corridor_risk_policy_version;
      },
      [](ActiveExecutionContext& value) { ++value.cable_model_version; },
      [](ActiveExecutionContext& value) {
        ++value.uncertainty_envelope_version;
      },
      [](ActiveExecutionContext& value) {
        ++value.execution_operating_envelope_version;
      },
      [](ActiveExecutionContext& value) {
        value.sensor_mode = SensorHealthMode::approved_degraded;
      },
  };

  for (const Mutation& mutate : mutations) {
    ExecutionLeaseMonitor monitor;
    const PlanValidationLease lease = lease_for(plan);
    ActiveExecutionContext changed = context_for(plan);
    mutate(changed);
    const ExecutionAuthorization rejected =
        monitor.evaluate(plan, path, lease, changed, feedback(), kNow);
    require(rejected.revoked() && rejected.request_controlled_stop &&
                rejected.request_replan &&
                monitor.isRevoked(lease.lease_sequence),
            "an execution-context change did not revoke authorization");
    const ExecutionAuthorization followup = monitor.evaluate(
        plan, path, lease, context_for(plan), feedback(), kNow);
    require(followup.revoked() &&
                followup.reason_code == "LEASE_ALREADY_REVOKED",
            "a context safety failure allowed a subsequent path command");
  }
}

void plan_profile_and_lease_pairing_fail_closed() {
  // Design: 18.2.7-8
  // Design: 18.2.7-12
  // Design: 18.2.7-13
  const TimedPath path = trajectory();
  const PlanningResult plan = plan_for(path);

  const auto require_pairing_rejection =
      [&plan, &path](const TimedPath& candidate_path,
              const PlanValidationLease& candidate_lease,
              const ActiveExecutionContext& candidate_context,
              ExecutionFeedback candidate_feedback) {
        ExecutionLeaseMonitor monitor;
        const ExecutionAuthorization rejected =
            monitor.evaluate(plan, candidate_path, candidate_lease,
                             candidate_context, candidate_feedback, kNow);
        require(rejected.revoked() && rejected.request_controlled_stop &&
                    monitor.isRevoked(candidate_lease.lease_sequence),
                "a plan/profile/lease pairing mismatch remained authorized");
        const ExecutionAuthorization followup = monitor.evaluate(
            plan, path, candidate_lease, context_for(plan), feedback(), kNow);
        require(followup.revoked() && !followup.authorized() &&
                    monitor.isRevoked(candidate_lease.lease_sequence),
                "a pairing safety failure allowed a subsequent path command");
      };

  PlanValidationLease mismatched_plan = lease_for(plan);
  ++mismatched_plan.plan_sequence_number;
  require_pairing_rejection(path, mismatched_plan, context_for(plan),
                            feedback());

  TimedPath mismatched_path = path;
  ++mismatched_path.execution_profile.version;
  require_pairing_rejection(mismatched_path, lease_for(plan), context_for(plan),
                            feedback());

  ActiveExecutionContext mismatched_context = context_for(plan);
  ++mismatched_context.execution_profile_version;
  require_pairing_rejection(path, lease_for(plan), mismatched_context,
                            feedback());

  PlanValidationLease mismatched_lease = lease_for(plan);
  ++mismatched_lease.execution_profile_version;
  require_pairing_rejection(path, mismatched_lease, context_for(plan),
                            feedback());

  ExecutionLeaseMonitor changed_profile_monitor;
  const PlanValidationLease unchanged_lease = lease_for(plan);
  require(changed_profile_monitor
              .evaluate(plan, path, unchanged_lease, context_for(plan),
                        feedback(), kNow)
              .authorized(),
          "test setup did not authorize the original execution profile");
  TimedPath reduced_speed = path;
  reduced_speed.execution_profile.samples.front().ground_speed_mps *= 0.5;
  ExecutionFeedback next_feedback = feedback(0.3);
  next_feedback.sequence_number = 2U;
  const ExecutionAuthorization changed_profile = changed_profile_monitor.evaluate(
      plan, reduced_speed, unchanged_lease, context_for(plan), next_feedback,
      kNow);
  require(changed_profile.revoked() &&
              changed_profile_monitor.isRevoked(unchanged_lease.lease_sequence),
          "a reduced-speed profile reused authorization for old profile data");
}

void newer_lease_cannot_reauthorize_an_older_plan() {
  const TimedPath path = trajectory();
  const PlanningResult newest_plan = plan_for(path);
  ExecutionLeaseMonitor monitor;
  PlanValidationLease newest_lease = lease_for(newest_plan);
  newest_lease.lease_sequence = 2U;
  require(monitor.evaluate(newest_plan, path, newest_lease,
                           context_for(newest_plan), feedback(), kNow)
              .authorized(),
          "newest plan and lease were not accepted");

  PlanningResult delayed_plan = newest_plan;
  delayed_plan.sequence_number = newest_plan.sequence_number - 1U;
  PlanValidationLease delayed_plan_lease = lease_for(delayed_plan);
  delayed_plan_lease.lease_sequence = 3U;
  ExecutionFeedback delayed_feedback = feedback();
  delayed_feedback.plan_sequence_number = delayed_plan.sequence_number;
  const ExecutionAuthorization rejected = monitor.evaluate(
      delayed_plan, path, delayed_plan_lease, context_for(delayed_plan),
      delayed_feedback, kNow);
  require(rejected.reason_code == "PLAN_SEQUENCE_OUT_OF_ORDER" &&
              rejected.revoked(),
          "a newer lease sequence reauthorized an older plan");
}

void rejected_high_sequences_do_not_poison_valid_state() {
  const TimedPath path = trajectory();
  const PlanningResult valid_plan = plan_for(path);
  ExecutionLeaseMonitor monitor;

  PlanningResult malformed_plan = valid_plan;
  malformed_plan.sequence_number = 100U;
  PlanValidationLease malformed_lease = lease_for(malformed_plan);
  malformed_lease.lease_sequence = 100U;
  malformed_lease.expires_at = {500'000'000};
  ExecutionFeedback malformed_feedback = feedback();
  malformed_feedback.plan_sequence_number = malformed_plan.sequence_number;
  require(monitor.evaluate(malformed_plan, path, malformed_lease,
                           context_for(malformed_plan), malformed_feedback,
                           kNow)
              .reason_code == "LEASE_EXPIRED",
          "malformed high-sequence lease did not fail its validity check");

  require(monitor.evaluate(valid_plan, path, lease_for(valid_plan),
                           context_for(valid_plan), feedback(), kNow)
              .authorized(),
          "rejected high sequence poisoned later valid plan/lease watermarks");

  malformed_plan.sequence_number = 101U;
  malformed_lease = lease_for(malformed_plan);
  malformed_lease.lease_sequence = 101U;
  malformed_lease.max_ground_speed_tracking_error_mps =
      std::numeric_limits<double>::quiet_NaN();
  malformed_feedback.plan_sequence_number = malformed_plan.sequence_number;
  require(monitor.evaluate(malformed_plan, path, malformed_lease,
                           context_for(malformed_plan), malformed_feedback,
                           kNow)
              .reason_code == "LEASE_BOUNDS_INVALID",
          "invalid high-sequence lease bounds were not rejected");
  require(monitor.evaluate(valid_plan, path, lease_for(valid_plan),
                           context_for(valid_plan), feedback(), kNow)
              .authorized(),
          "invalid high-sequence lease bounds poisoned valid watermarks");

  malformed_plan.sequence_number = 102U;
  malformed_lease = lease_for(malformed_plan);
  malformed_lease.lease_sequence = 102U;
  malformed_feedback.plan_sequence_number = malformed_plan.sequence_number;
  malformed_feedback.tension_n =
      std::numeric_limits<double>::quiet_NaN();
  require(monitor.evaluate(malformed_plan, path, malformed_lease,
                           context_for(malformed_plan), malformed_feedback,
                           kNow)
              .reason_code == "FEEDBACK_NONFINITE",
          "non-finite high-sequence feedback was not rejected");
  require(monitor.evaluate(valid_plan, path, lease_for(valid_plan),
                           context_for(valid_plan), feedback(), kNow)
              .authorized(),
          "non-finite high-sequence feedback poisoned valid watermarks");
}

void malformed_contracts_fail_closed() {
  const TimedPath path = trajectory();
  const PlanningResult plan = plan_for(path);
  const ActiveExecutionContext context = context_for(plan);
  PlanValidationLease lease = lease_for(plan);
  ExecutionLeaseMonitor monitor;
  TimedPath mismatched = path;
  mismatched.geometry.points.back().arc_length_m = 3.0;
  require(monitor.evaluate(plan, mismatched, lease, context, feedback(), kNow)
                  .reason_code == "EXECUTION_PROFILE_INVALID",
          "geometry/profile arc mismatch was accepted");
  lease = lease_for(plan);
  lease.max_ground_speed_tracking_error_mps =
      std::numeric_limits<double>::quiet_NaN();
  ExecutionLeaseMonitor threshold_monitor;
  require(threshold_monitor.evaluate(plan, path, lease, context, feedback(), kNow)
                  .reason_code == "LEASE_BOUNDS_INVALID",
          "non-finite tracking threshold was accepted");
  lease = lease_for(plan);
  lease.expires_at = {10'000'000'000};
  ExecutionLeaseMonitor duration_monitor;
  require(duration_monitor.evaluate(plan, path, lease, context, feedback(), kNow)
                  .reason_code == "LEASE_EXPIRED",
          "overlong lease was accepted");
  ExecutionLeaseMonitorConfig invalid_config;
  invalid_config.renewal_margin = {0};
  ExecutionLeaseMonitor invalid_monitor(invalid_config);
  require(invalid_monitor.evaluate(plan, path, lease_for(plan), context,
                                   feedback(), kNow)
                  .reason_code == "INVALID_MONITOR_CONFIGURATION",
          "zero renewal margin was accepted");
}

}  // namespace

int main() {
  try {
    authorized_and_interpolated_profile();
    deviation_revokes_and_stale_lease_is_rejected();
    versions_order_and_renewal_are_enforced();
    context_changes_revoke_and_block_subsequent_commands();
    plan_profile_and_lease_pairing_fail_closed();
    newer_lease_cannot_reauthorize_an_older_plan();
    rejected_high_sequences_do_not_poison_valid_state();
    malformed_contracts_fail_closed();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
