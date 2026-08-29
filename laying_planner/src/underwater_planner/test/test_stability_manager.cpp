#include "underwater_planner/core/stability_manager.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

namespace {
using namespace underwater_planner::core;

void require(const bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

GeometricPath path(const double y = 0.0) {
  GeometricPath result;
  result.points = {{0.0, 0.0, y, 0.0, 0.0},
                   {1.0, 1.0, y, 0.0, 0.0},
                   {2.0, 2.0, y, 0.0, 0.0}};
  return result;
}

TimedPath timed_path() {
  TimedPath result;
  result.geometry.metadata.path_version = 100;
  result.geometry.metadata.coordinate_frame = "world";
  result.geometry.metadata.reference_line_version = 4;
  result.geometry.metadata.interpolation_rule = "linear/v1";
  result.geometry.points = {{0.0, 0.0, 0.0, 0.0, 0.0},
                            {1.0, 1.0, 0.0, 0.0, 0.0},
                            {2.0, 2.0, 0.0, 0.0, 0.0},
                            {3.0, 3.0, 0.0, 0.0, 0.0},
                            {4.0, 4.0, 0.0, 0.0, 0.0}};
  result.execution_profile.version = 12;
  result.execution_profile.operating_envelope_version = 11;
  result.execution_profile.interpolation_rule = "piecewise-linear-execution/v1";
  result.execution_profile.stopping_point_arc_length_m = 4.0;
  result.execution_profile.samples = {
      {0.0, {0}, 1.0, 0.0, 1.0, 0.0, 100.0},
      {1.0, {1'000'000'000}, 1.0, 0.0, 1.0, 0.0, 100.0},
      {2.0, {2'000'000'000}, 1.0, 0.0, 1.0, 0.0, 100.0},
      {3.0, {3'000'000'000}, 1.0, 0.0, 1.0, 0.0, 100.0},
      {4.0, {4'000'000'000}, 0.0, -1.0, 0.0, -1.0, 100.0}};
  result.execution_profile.approved_tracking_limits.ground_speed = {0.0, 1.0};
  result.execution_profile.approved_tracking_limits.ground_acceleration = {-1.0, 1.0};
  result.execution_profile.approved_tracking_limits.maximum_lateral_acceleration_mps2 = 1.0;
  result.execution_profile.approved_tracking_limits.payout_speed = {0.0, 1.0};
  result.execution_profile.approved_tracking_limits.payout_acceleration = {-1.0, 1.0};
  result.execution_profile.approved_tracking_limits.maximum_payout_tracking_error_mps = 1.0;
  result.execution_profile.approved_tracking_limits.tension = {0.0, 200.0};
  result.execution_profile.approved_tracking_limits.maximum_stopping_distance_m = 4.0;
  return result;
}

PlanValidationLease lease(const std::uint64_t plan_sequence,
                          const std::uint64_t lease_sequence,
                          const std::int64_t expiry = 10'000'000'000) {
  PlanValidationLease result;
  result.plan_sequence_number = plan_sequence;
  result.lease_sequence = lease_sequence;
  result.evaluator_config_version = 3;
  result.parameter_profile_id = "production-v1";
  result.map_version = {"map", 7, {1'000'000'000}, "world"};
  result.reference_line_version = 4;
  result.robot_operating_area_version = 5;
  result.terrain_gradient_policy_version = 6;
  result.corridor_risk_policy_version = 7;
  result.cable_model_version = 8;
  result.uncertainty_envelope_version = 9;
  result.uncertainty_envelope_generator_version = 10;
  result.execution_operating_envelope_version = 11;
  result.execution_profile_version = 12;
  result.operating_domain_id = "domain-v1";
  result.cable_corridor_version = 13;
  result.validated_at = {2'000'000'000};
  result.expires_at = {expiry};
  result.robot_path_validation_passed = true;
  result.cable_corridor_validation_passed = true;
  result.cable_laying_validation_passed = true;
  return result;
}

PlanValidityEvaluation evaluation(const GeometricPath& geometry,
                                  const PlanValidationLease& validation_lease,
                                  const bool valid = true) {
  PlanValidityEvaluation result;
  result.valid = valid;
  result.status = valid ? PlanValidationStatus::valid
                        : PlanValidationStatus::robot_constraint_violation;
  auto remaining = std::make_shared<TimedPath>();
  remaining->geometry = geometry;
  remaining->geometry.metadata.reference_line_version = validation_lease.reference_line_version;
  remaining->execution_profile.version = validation_lease.execution_profile_version;
  remaining->execution_profile.operating_envelope_version =
      validation_lease.execution_operating_envelope_version;
  result.remaining_path = std::move(remaining);
  result.lease = validation_lease;
  result.evaluator_config_version = validation_lease.evaluator_config_version;
  result.parameter_profile_id = validation_lease.parameter_profile_id;
  return result;
}

void relative_threshold_and_boundary() {
  // Design: 18.2.7-1
  // Design: 18.2.7-2
  StabilityManager manager;
  require(!manager.should_switch_path(path(), path(0.1), 100.0, 90.0),
          "threshold boundary switched unexpectedly");
  require(!manager.should_switch_path(path(), path(0.1), 100.0, 95.0),
          "small improvement bypassed hysteresis");
  require(manager.should_switch_path(path(), path(0.1), 100.0, 89.9),
          "significant improvement did not switch");
}

void topology_change_can_override_cost_hysteresis() {
  PathHysteresisConfig config;
  config.topology_distance_threshold_m = 1.0;
  StabilityManager manager(config);
  require(manager.should_switch_path(path(), path(2.0), 100.0, 99.0),
          "large topology change did not trigger a switch");
}

void invalid_inputs_fail_closed() {
  StabilityManager manager;
  GeometricPath malformed = path();
  malformed.points[1].arc_length_m = malformed.points[0].arc_length_m;
  require(!manager.should_switch_path(malformed, path(), 1.0, 0.1),
          "non-monotonic path was accepted");
  require(!manager.should_switch_path(path(), path(), -1.0, 0.1),
          "negative cost was accepted");
}

void repeated_small_perturbations_do_not_oscillate() {
  // Design: 18.2.7-3
  StabilityManager manager;
  const GeometricPath current = path();
  for (int index = 0; index < 20; ++index) {
    const double perturbation = (index % 2 == 0) ? 0.01 : -0.01;
    require(!manager.should_switch_path(current, path(perturbation), 100.0,
                                        95.0),
            "small alternating perturbation caused a path switch");
  }
}

void safety_precedes_hysteresis_and_pairs_lease() {
  // Design: 18.2.7-6
  StabilityManager manager;
  const MonotonicTime now{3'000'000'000};
  const auto current = evaluation(path(), lease(11, 20));
  const auto candidate = evaluation(path(0.2), lease(12, 21));
  const PathSwitchDecision switched =
      manager.decide_path_switch(current, candidate, 100.0, 80.0, now);
  require(switched.should_switch() && switched.lease.has_value() &&
              switched.lease->lease_sequence == 21,
          "valid candidate was not selected with its matching lease");

  const auto invalid_current = evaluation(path(), lease(11, 20), false);
  const PathSwitchDecision direct =
      manager.decide_path_switch(invalid_current, candidate, 1.0, 1.0, now);
  require(direct.should_switch(), "invalid current plan retained hysteresis");

  const double invalid_candidate_costs[] = {
      -1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()};
  for (const double invalid_cost : invalid_candidate_costs) {
    const PathSwitchDecision rejected = manager.decide_path_switch(
        invalid_current, candidate, 1.0, invalid_cost, now);
    require(rejected.action == PathSwitchAction::stop &&
                rejected.reason == "invalid_candidate_cost",
            "an invalid candidate cost bypassed the direct-switch gate");
  }
  const PathSwitchDecision missing_current_rejected =
      manager.decide_path_switch(std::nullopt, candidate, 0.0,
                                 std::numeric_limits<double>::quiet_NaN(), now);
  require(missing_current_rejected.action == PathSwitchAction::stop &&
              missing_current_rejected.reason == "invalid_candidate_cost",
          "a candidate without a current fallback bypassed the cost gate");

  const auto invalid_candidate = evaluation(path(0.2), lease(12, 21), false);
  const PathSwitchDecision kept =
      manager.decide_path_switch(current, invalid_candidate, 100.0, 1.0, now);
  require(kept.should_keep_current() && kept.lease->lease_sequence == 20,
          "invalid candidate displaced a valid current plan");
}

void expiry_and_context_mismatch_fail_closed() {
  StabilityManager manager;
  const MonotonicTime now{3'000'000'000};
  const auto expired = evaluation(path(), lease(11, 20, 2'500'000'000));
  const auto candidate = evaluation(path(0.1), lease(12, 21));
  require(manager.decide_path_switch(expired, candidate, 1.0, 1.0, now)
              .should_switch(),
          "expired current lease was retained");

  auto mismatched_lease = lease(12, 21);
  mismatched_lease.map_version.sequence_number = 8;
  const PathSwitchDecision mismatch = manager.decide_path_switch(
      evaluation(path(), lease(11, 20)), evaluation(path(0.1), mismatched_lease),
      100.0, 1.0, now);
  require(mismatch.action == PathSwitchAction::stop &&
              mismatch.reason == "validation_context_mismatch",
          "mixed validation contexts were compared by hysteresis");

  auto mismatched_corridor = lease(12, 21);
  ++mismatched_corridor.cable_corridor_version;
  const PathSwitchDecision corridor_mismatch = manager.decide_path_switch(
      evaluation(path(), lease(11, 20)),
      evaluation(path(0.1), mismatched_corridor), 100.0, 1.0, now);
  require(corridor_mismatch.action == PathSwitchAction::stop &&
              corridor_mismatch.reason == "validation_context_mismatch",
          "different cable corridor versions were compared by hysteresis");

  auto new_profile = lease(12, 21);
  new_profile.execution_profile_version = 99;
  const PathSwitchDecision profile_change = manager.decide_path_switch(
      evaluation(path(), lease(11, 20)), evaluation(path(0.1), new_profile),
      100.0, 80.0, now);
  require(profile_change.should_switch(),
          "plan-specific execution profile blocked a same-context switch");

  auto newer_state = lease(12, 21);
  newer_state.robot_state_timestamp = {2'100'000'000};
  const PathSwitchDecision state_mismatch = manager.decide_path_switch(
      evaluation(path(), lease(11, 20)), evaluation(path(0.1), newer_state),
      100.0, 80.0, now);
  require(state_mismatch.action == PathSwitchAction::stop,
          "different synchronized state timestamps were compared");

  const PathSwitchDecision malformed_cost = manager.decide_path_switch(
      evaluation(path(), lease(11, 20)), candidate, 100.0,
      std::numeric_limits<double>::quiet_NaN(), now);
  require(malformed_cost.should_keep_current() &&
              malformed_cost.reason == "invalid_path_cost",
          "non-finite candidate cost was not rejected safely");

  auto future = lease(12, 21);
  future.validated_at = {4'000'000'000};
  const PathSwitchDecision future_decision = manager.decide_path_switch(
      evaluation(path(), lease(11, 20)), evaluation(path(0.1), future),
      100.0, 80.0, now);
  require(future_decision.should_keep_current(),
          "future-dated validation lease was accepted");
}

void expiry_requires_a_fresh_revalidation_callback() {
  // Design: 18.2.7-9
  StabilityManager manager;
  const MonotonicTime now{3'000'000'000};
  const auto stale = evaluation(path(), lease(11, 20, 2'500'000'000));
  const auto candidate = evaluation(path(0.1), lease(12, 21));
  bool recaptured = false;
  const auto decision = manager.decide_path_switch(
      stale, candidate, 100.0, 80.0, now,
      [&recaptured]() {
        recaptured = true;
        return StabilityManager::RevalidationResult{
            evaluation(path(), lease(11, 22)), evaluation(path(0.1), lease(12, 23))};
      });
  require(recaptured && decision.should_switch() && decision.lease->lease_sequence == 23,
          "expired lease did not force a fresh paired revalidation");
  const auto stopped = manager.decide_path_switch(stale, candidate, 100.0, 80.0,
                                                   now, {});
  require(stopped.action == PathSwitchAction::stop &&
              stopped.reason == "lease_expired_without_recapture",
          "expired lease was decided without recapture callback");
}

void topology_distance_is_symmetric_and_deterministic() {
  const double forward = StabilityManager::topology_distance_m(path(), path(2.0));
  const double reverse = StabilityManager::topology_distance_m(path(2.0), path());
  require(std::abs(forward - 2.0) < 1.0e-12,
          "Hausdorff topology distance was incorrect");
  require(std::abs(forward - reverse) < 1.0e-12,
          "topology distance was not symmetric");
}

void commitment_length_is_bounded_by_speed_stop_and_authorization() {
  StabilityManager manager;
  RobotState robot;
  robot.pose.x_m = 0.0;
  robot.pose.y_m = 0.0;
  robot.ground_speed_mps = 1.0;
  const auto extracted = manager.extract_commitment_segment(
      timed_path(), robot, CommitmentSegmentConfig{0.5, 0.2, 0.5});
  require(extracted.status == CommitmentExtractionStatus::valid &&
              extracted.segment.has_value(),
          extracted.reason.c_str());
  require(std::abs(extracted.required_length_m - 0.7) < 1.0e-12,
          "commitment length did not include the stopping margin");
  require(extracted.segment->geometry.points.front().arc_length_m == 0.0 &&
              extracted.segment->geometry.points.back().arc_length_m == 0.7,
          "commitment geometry was not sliced at the computed boundary");
  require(extracted.segment->execution_profile.samples.front().time_from_start.nanoseconds ==
              0 &&
              extracted.segment->execution_profile.samples.back().time_from_start.nanoseconds ==
              700'000'000,
          "commitment execution time was not rebased");
  robot.pose.x_m = 1.0;
  const auto too_long = manager.extract_commitment_segment(
      timed_path(), robot, CommitmentSegmentConfig{4.0, 0.0, 0.5});
  require(too_long.status == CommitmentExtractionStatus::authorization_range_insufficient,
          "commitment extraction exceeded the authorized remaining path");
}

void commitment_prefix_is_immutable_and_timed_merge_checks_continuity() {
  // Design: 18.2.6-7
  StabilityManager manager;
  RobotState robot;
  robot.ground_speed_mps = 1.0;
  const TimedPath original = timed_path();
  const auto extracted = manager.extract_commitment_segment(
      original, robot, CommitmentSegmentConfig{1.0, 0.0, 0.5});
  require(extracted.segment.has_value(), "commitment extraction failed");
  TimedPath tail = timed_path();
  tail.geometry.points.front().arc_length_m = 0.0;
  tail.geometry.points.front().x_m = 1.0;
  tail.geometry.points[1].arc_length_m = 1.0;
  tail.geometry.points[1].x_m = 2.0;
  tail.geometry.points[2].arc_length_m = 2.0;
  tail.geometry.points[2].x_m = 3.0;
  tail.geometry.points[3].arc_length_m = 3.0;
  tail.geometry.points[3].x_m = 4.0;
  tail.geometry.points[4].arc_length_m = 4.0;
  tail.geometry.points[4].x_m = 5.0;
  tail.execution_profile.samples.front().ground_speed_mps = 1.0;
  const auto merged = manager.merge_timed_paths(
      *extracted.segment, tail, PathG2MergeLimits{1.0e-9, 1.0e-9, 1.0e-9},
      ExecutionJoinTolerances{1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9},
      [](const TimedPath& candidate) { return validate(candidate).valid; });
  require(merged.valid && merged.trajectory.has_value(),
          merged.reason.c_str());
  require(merged.trajectory->execution_profile.version >
              extracted.segment->execution_profile.version,
          "timed merge did not issue a new execution profile version");
  require(merged.trajectory->execution_profile.samples.front().ground_speed_mps ==
              extracted.segment->execution_profile.samples.front().ground_speed_mps &&
              merged.trajectory->geometry.points.front().x_m ==
              extracted.segment->geometry.points.front().x_m,
          "merged path changed the committed prefix");
  const auto missing_final_gate = manager.merge_timed_paths(
      *extracted.segment, tail, PathG2MergeLimits{1.0e-9, 1.0e-9, 1.0e-9},
      ExecutionJoinTolerances{1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9});
  require(!missing_final_gate.valid &&
              missing_final_gate.reason == "full_path_final_validation_failed",
          "timed merge published without the required full-path verifier");
  tail.execution_profile.samples.front().tension_setpoint_n = 101.0;
  const auto discontinuous = manager.merge_timed_paths(
      *extracted.segment, tail, PathG2MergeLimits{1.0e-9, 1.0e-9, 1.0e-9},
      ExecutionJoinTolerances{1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9},
      [](const TimedPath& candidate) { return validate(candidate).valid; });
  require(!discontinuous.valid && discontinuous.reason ==
              "execution_junction_residual_exceeded",
          "execution discontinuity bypassed the timed merge gate");
}

}  // namespace

int main() {
  try {
    relative_threshold_and_boundary();
    topology_change_can_override_cost_hysteresis();
    invalid_inputs_fail_closed();
    repeated_small_perturbations_do_not_oscillate();
    safety_precedes_hysteresis_and_pairs_lease();
    expiry_and_context_mismatch_fail_closed();
    expiry_requires_a_fresh_revalidation_callback();
    topology_distance_is_symmetric_and_deterministic();
    commitment_length_is_bounded_by_speed_stop_and_authorization();
    commitment_prefix_is_immutable_and_timed_merge_checks_continuity();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
