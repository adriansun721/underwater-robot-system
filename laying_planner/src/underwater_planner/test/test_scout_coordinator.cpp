#include "underwater_planner/core/scout_coordinator.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace underwater_planner::core;

namespace {

void require(const bool condition, const char* message) {
  if (!condition) {
    std::cerr << "T35 failure: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

bool same_target(const ScoutTarget& left, const ScoutTarget& right) {
  return left.target_pose.x_m == right.target_pose.x_m &&
         left.target_pose.y_m == right.target_pose.y_m &&
         left.target_pose.heading_rad == right.target_pose.heading_rad &&
         left.target_pose.timestamp.nanoseconds ==
             right.target_pose.timestamp.nanoseconds &&
         left.gap_start_progress_m == right.gap_start_progress_m &&
         left.gap_end_progress_m == right.gap_end_progress_m &&
         left.coverage_fraction == right.coverage_fraction &&
         left.information_value == right.information_value &&
         left.forward_progress_m == right.forward_progress_m &&
         left.estimated_arrival_cost_m == right.estimated_arrival_cost_m &&
         left.priority == right.priority &&
         left.scout_corridor_half_width_m ==
             right.scout_corridor_half_width_m &&
         left.urgency == right.urgency &&
         left.source_map_version == right.source_map_version &&
         left.reference_line_version == right.reference_line_version &&
         left.policy_version == right.policy_version &&
         left.parameter_profile_id == right.parameter_profile_id &&
         left.operating_domain_id == right.operating_domain_id;
}

bool same_request(const ScoutRequest& left, const ScoutRequest& right) {
  return left.schema_version == right.schema_version &&
         left.request_sequence == right.request_sequence &&
         left.revision == right.revision &&
         left.policy_version == right.policy_version &&
         left.status == right.status && same_target(left.target, right.target) &&
         left.requested_at.nanoseconds == right.requested_at.nanoseconds &&
         left.expires_at.nanoseconds == right.expires_at.nanoseconds &&
         left.last_associated_map_version ==
             right.last_associated_map_version &&
         left.completed_map_version == right.completed_map_version &&
         left.recommended_main_action == right.recommended_main_action &&
         left.planning_directive == right.planning_directive;
}

bool same_optional_request(const std::optional<ScoutRequest>& left,
                           const std::optional<ScoutRequest>& right) {
  return left.has_value() == right.has_value() &&
         (!left.has_value() || same_request(*left, *right));
}

bool same_issue_result(const ScoutRequestIssueResult& left,
                       const ScoutRequestIssueResult& right) {
  return left.disposition == right.disposition &&
         same_optional_request(left.request, right.request) &&
         left.issues == right.issues;
}

bool same_map_update_result(const ScoutMapUpdateResult& left,
                            const ScoutMapUpdateResult& right) {
  return left.disposition == right.disposition &&
         same_optional_request(left.request, right.request) &&
         left.invalidate_old_plan == right.invalidate_old_plan &&
         left.trigger_replanning == right.trigger_replanning &&
         left.issues == right.issues;
}

ScoutCoordinationParameters parameters() {
  ScoutCoordinationParameters result;
  result.parameter_profile_id = "scout-test-v36";
  result.operating_domain_id = "synthetic-level1/v1";
  result.minimum_map_confidence = 0.5;
  result.sample_interval_m = 1.0;
  result.merge_distance_m = 0.1;
  result.minimum_safe_distance_m = 2.0;
  result.planning_lead_time_s = 5.0;
  result.average_velocity_mps = 1.0;
  result.hysteresis_distance_m = 0.25;
  result.hysteresis_time_s = 0.5;
  result.policy_version = 36U;
  result.sensor_coverage_radius_m = 1.5;
  result.scout_corridor_half_width_m = 3.0;
  result.communication_max_distance_m = 50.0;
  result.desired_scout_distance_m = 20.0;
  result.continue_scout_distance_m = 40.0;
  result.stop_scout_distance_m = 45.0;
  result.blocking_priority_weight = 100.0;
  result.information_value_weight = 10.0;
  result.forward_progress_weight = 1.0;
  result.arrival_cost_weight = 1.0;
  result.request_timeout = Duration{30'000'000'000};
  return result;
}

MapSnapshot make_map() {
  MapSnapshot map;
  map.version = {"gap-map", 7U, MonotonicTime{7'000'000'000}, "map"};
  map.width = 12U;
  map.height = 3U;
  map.resolution_m = 1.0;
  map.origin_x_m = 0.0;
  map.origin_y_m = -1.5;
  map.derived_configuration_version = 4U;
  map.cells.assign(map.width * map.height, MapCell{0.0, 0.01, 1.0, true});
  for (MapCell& cell : map.cells) cell.measurement_timestamp = map.version.timestamp;
  map.cells.at(1U * map.width + 3U).known = false;
  map.cells.at(1U * map.width + 4U).confidence = 0.2;
  map.cells.at(1U * map.width + 5U).known = false;
  return map;
}

TimedPath approved_path() {
  TimedPath path;
  path.geometry.metadata.path_version = 10U;
  path.geometry.metadata.coordinate_frame = "map";
  path.geometry.metadata.reference_line_version = 9U;
  path.geometry.metadata.interpolation_rule = "linear";
  path.geometry.points = {{0.0, 0.0, 0.0, 0.0, 0.0},
                          {10.0, 10.0, 0.0, 0.0, 0.0}};
  path.execution_profile.version = 11U;
  path.execution_profile.operating_envelope_version = 12U;
  path.execution_profile.interpolation_rule = "linear";
  path.execution_profile.stopping_point_arc_length_m = 10.0;
  path.execution_profile.samples = {
      {0.0, Duration{0}, 1.0, 0.0, 1.0, 0.0, 40.0},
      {10.0, Duration{10'000'000'000}, 0.0, -0.1, 0.0, 0.0, 40.0}};
  path.execution_profile.approved_tracking_limits.ground_speed = {0.0, 2.0};
  path.execution_profile.approved_tracking_limits.ground_acceleration = {-1.0, 1.0};
  path.execution_profile.approved_tracking_limits.maximum_lateral_acceleration_mps2 = 1.0;
  path.execution_profile.approved_tracking_limits.payout_speed = {0.0, 2.0};
  path.execution_profile.approved_tracking_limits.payout_acceleration = {-1.0, 1.0};
  path.execution_profile.approved_tracking_limits.maximum_payout_tracking_error_mps = 1.0;
  path.execution_profile.approved_tracking_limits.tension = {0.0, 100.0};
  path.execution_profile.approved_tracking_limits.maximum_stopping_distance_m = 20.0;
  return path;
}

void identifies_and_merges_gaps_with_snapshot_metadata() {
  const ReferenceLine reference = make_reference_line(
      9U, "map", {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0},
                   {4.0, 0.0}, {5.0, 0.0}, {6.0, 0.0}, {7.0, 0.0}});
  const ScoutCoordinator coordinator(parameters());
  const InformationGapScanResult result =
      coordinator.identify_gaps_result(reference, make_map(), 7.0);
  require(result.validity == ScoutGapScanValidity::valid, "valid scan rejected");
  require(result.gaps.size() == 1U, "adjacent information gaps were not merged");
  require(result.gaps.front().start_progress_m == 3.0 &&
              result.gaps.front().end_progress_m == 5.0,
          "merged gap lost reference progress interval");
  require(result.gaps.front().source_map_version.sequence_number == 7U &&
              result.gaps.front().reference_line_version == 9U,
          "merged gap lost immutable snapshot versions");
  require(result.gaps.front().reason == InformationGapReason::unknown,
          "unknown gap was made less conservative by low confidence");
}

void urgency_uses_approved_profile_and_explicit_fallback() {
  const ReferenceLine reference = make_reference_line(
      9U, "map", {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0},
                   {4.0, 0.0}, {5.0, 0.0}, {6.0, 0.0}, {7.0, 0.0}});
  const InformationGap gap =
      ScoutCoordinator(parameters()).identify_gaps(reference, make_map(), 7.0).front();
  RobotState robot;
  robot.pose = {0.0, 0.0, 0.0, MonotonicTime{0}};
  robot.ground_speed_mps = 1.0;
  robot.curvature_timestamp = MonotonicTime{0};
  robot.sequence_number = 1U;
  const ScoutCoordinator coordinator(parameters());
  const GapUrgencyAssessment urgent = coordinator.assess_gap_urgency(
      gap, robot, 0.0, approved_path(), 7.0);
  require(urgent.urgency == GapUrgency::urgent && urgent.time_to_gap_s == 3.0,
          "approved execution profile was not used for time-to-gap");
  require(urgent.recommended_action ==
              "REQUEST_VALIDATED_REDUCED_SPEED_PROFILE",
          "urgent action directly changed speed instead of requesting a profile");

  const GapUrgencyAssessment fallback =
      coordinator.assess_gap_urgency(gap, robot, 0.0, std::nullopt, 7.0);
  require(fallback.used_conservative_fallback &&
              fallback.urgency == GapUrgency::blocking &&
              std::isinf(fallback.time_to_gap_s),
          "missing approved path did not fail conservatively");
}

void invalid_inputs_and_hysteresis_are_deterministic() {
  const ReferenceLine reference = make_reference_line(
      9U, "map", {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
  ScoutCoordinationParameters bad = parameters();
  bad.minimum_map_confidence = std::numeric_limits<double>::quiet_NaN();
  const InformationGapScanResult invalid =
      ScoutCoordinator(bad).identify_gaps_result(reference, make_map(), 2.0);
  require(invalid.validity == ScoutGapScanValidity::input_invalid &&
              !invalid.issues.empty(),
          "invalid scout parameters were accepted");

  const ScoutCoordinator coordinator(parameters());
  InformationGap gap;
  gap.start_progress_m = 6.0;
  gap.end_progress_m = 6.0;
  gap.source_map_version = make_map().version;
  gap.reference_line_version = 9U;
  RobotState robot;
  robot.pose = {0.0, 0.0, 0.0, MonotonicTime{0}};
  robot.curvature_timestamp = MonotonicTime{0};
  robot.sequence_number = 1U;
  const GapUrgencyAssessment first =
      coordinator.assess_gap_urgency(gap, robot, 0.0, approved_path(), 7.0);
  require(first.urgency == GapUrgency::scheduled, "scheduled boundary was misclassified");
  const GapUrgencyAssessment repeat =
      coordinator.assess_gap_urgency(gap, robot, 0.0, approved_path(), 7.0);
  require(repeat.urgency == first.urgency &&
              repeat.recommended_action == first.recommended_action,
          "repeated urgency assessment was not deterministic");

  InformationGap out_of_range = gap;
  out_of_range.start_progress_m = 1.0e13;
  out_of_range.end_progress_m = 1.0e13;
  const GapUrgencyAssessment unquantizable = coordinator.assess_gap_urgency(
      out_of_range, robot, 0.0, approved_path(), 7.0);
  require(unquantizable.used_conservative_fallback &&
              unquantizable.urgency == GapUrgency::blocking,
          "unrepresentable progress did not fail closed");
}

void generates_auditable_targets_and_ranks_value_against_arrival_cost() {
  const ReferenceLine reference = make_reference_line(
      9U, "map", {{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0},
                    {30.0, 0.0}, {40.0, 0.0}});
  InformationGap near_gap;
  near_gap.center = {10.0, 0.0};
  near_gap.start_progress_m = 9.0;
  near_gap.end_progress_m = 11.0;
  near_gap.minimum_confidence = 0.4;
  near_gap.source_map_version = make_map().version;
  near_gap.reference_line_version = reference.version;
  InformationGap valuable_gap = near_gap;
  valuable_gap.center = {25.0, 0.0};
  valuable_gap.start_progress_m = 22.0;
  valuable_gap.end_progress_m = 28.0;
  valuable_gap.minimum_confidence = 0.0;

  GapUrgencyAssessment near_assessment;
  near_assessment.urgency = GapUrgency::scheduled;
  near_assessment.distance_to_gap_m = 9.0;
  GapUrgencyAssessment valuable_assessment = near_assessment;
  valuable_assessment.distance_to_gap_m = 22.0;
  const ScoutCoordinator coordinator(parameters());
  const std::vector<ScoutTarget> targets = coordinator.generate_scout_targets(
      {{near_gap, near_assessment}, {valuable_gap, valuable_assessment}},
      reference, Pose2d{0.0, 0.0, 0.0, MonotonicTime{1}},
      Pose2d{2.0, 0.0, 0.0, MonotonicTime{1}}, 0.0);

  require(targets.size() == 2U, "valid gaps did not produce scout targets");
  require(targets.front().gap_start_progress_m == 22.0,
          "target priority ignored coverage and information value");
  require(targets.front().coverage_fraction == 0.5,
          "sensor coverage was not included in the target audit");
  require(targets.front().target_pose.x_m == 25.0 &&
              targets.front().target_pose.y_m == 0.0 &&
              targets.front().target_pose.heading_rad == 0.0,
          "target did not use the gap center and reference tangent");
  require(targets.front().source_map_version.sequence_number == 7U &&
              targets.front().reference_line_version == 9U &&
              targets.front().policy_version == 36U &&
              targets.front().parameter_profile_id == "scout-test-v36" &&
              targets.front().operating_domain_id == "synthetic-level1/v1",
          "target omitted immutable source and policy versions");
  require(targets.front().scout_corridor_half_width_m == 3.0 &&
              targets.front().estimated_arrival_cost_m == 23.0,
          "target audit omitted corridor or scout arrival cost");

  InformationGap unreachable = near_gap;
  unreachable.center = {60.0, 0.0};
  unreachable.start_progress_m = 39.0;
  unreachable.end_progress_m = 40.0;
  const ScoutTargetGenerationResult rejected = coordinator.generate_scout_target(
      unreachable, near_assessment, reference,
      Pose2d{0.0, 0.0, 0.0, MonotonicTime{1}},
      Pose2d{2.0, 0.0, 0.0, MonotonicTime{1}}, 0.0);
  require(rejected.validity == ScoutTargetValidity::distance_constraint_violated &&
              !rejected.issues.empty(),
          "target beyond the communication hard limit was accepted");

  ScoutCoordinationParameters proximity_parameters = parameters();
  proximity_parameters.blocking_priority_weight = 0.0;
  proximity_parameters.information_value_weight = 0.0;
  proximity_parameters.arrival_cost_weight = 0.0;
  InformationGap far_same_value = near_gap;
  far_same_value.center = {30.0, 0.0};
  far_same_value.start_progress_m = 29.0;
  far_same_value.end_progress_m = 31.0;
  const std::vector<ScoutTarget> proximity_order =
      ScoutCoordinator(proximity_parameters).generate_scout_targets(
          {{far_same_value, near_assessment}, {near_gap, near_assessment}},
          reference, Pose2d{0.0, 0.0, 0.0, MonotonicTime{1}},
          Pose2d{0.0, 0.0, 0.0, MonotonicTime{1}}, 0.0);
  require(proximity_order.front().gap_start_progress_m == 9.0,
          "farther forward gap outranked an otherwise equal imminent gap");

  GapUrgencyAssessment blocking_assessment = near_assessment;
  blocking_assessment.urgency = GapUrgency::blocking;
  const std::vector<ScoutTarget> blocking_order =
      ScoutCoordinator(proximity_parameters).generate_scout_targets(
          {{near_gap, near_assessment}, {far_same_value, blocking_assessment}},
          reference, Pose2d{0.0, 0.0, 0.0, MonotonicTime{1}},
          Pose2d{0.0, 0.0, 0.0, MonotonicTime{1}}, 0.0);
  require(blocking_order.front().urgency == GapUrgency::blocking,
          "configurable weights overrode the blocking-gap priority rule");

  InformationGap wrong_reference = near_gap;
  wrong_reference.reference_line_version = 8U;
  require(coordinator.generate_scout_target(
              wrong_reference, near_assessment, reference,
              Pose2d{0.0, 0.0, 0.0, MonotonicTime{1}},
              Pose2d{2.0, 0.0, 0.0, MonotonicTime{1}}, 0.0)
              .validity == ScoutTargetValidity::input_invalid,
          "target accepted a gap from another reference version");
  require(coordinator.generate_scout_target(
              near_gap, near_assessment, reference,
              Pose2d{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0,
                     MonotonicTime{1}},
              Pose2d{2.0, 0.0, 0.0, MonotonicTime{1}}, 0.0)
              .validity == ScoutTargetValidity::input_invalid,
          "target accepted a non-finite main robot pose");
  require(coordinator.generate_scout_target(
              near_gap, near_assessment, reference,
              Pose2d{0.0, 0.0, 0.0, MonotonicTime{1}},
              Pose2d{2.0, 0.0, 0.0, MonotonicTime{1}}, 12.0)
              .validity == ScoutTargetValidity::input_invalid,
          "target accepted a gap wholly behind current task progress");

  const std::vector<ScoutTarget> repeated = coordinator.generate_scout_targets(
      {{near_gap, near_assessment}, {valuable_gap, valuable_assessment}},
      reference, Pose2d{0.0, 0.0, 0.0, MonotonicTime{1}},
      Pose2d{2.0, 0.0, 0.0, MonotonicTime{1}}, 0.0);
  require(repeated.size() == targets.size() &&
              repeated.front().gap_start_progress_m ==
                  targets.front().gap_start_progress_m &&
              repeated.front().priority == targets.front().priority,
          "target ranking was not deterministic for identical inputs");
}

void distance_policy_uses_independent_thresholds_hysteresis_and_degradation() {
  const ScoutCoordinator coordinator(parameters());
  const Pose2d main_pose{0.0, 0.0, 0.0, MonotonicTime{1}};

  const ScoutDistanceAssessment desired = coordinator.assess_distance_constraint(
      main_pose, Pose2d{20.0, 0.0, 0.0, MonotonicTime{1}});
  require(desired.directive == ScoutDistanceDirective::maintain_desired_spacing &&
              desired.communication_satisfied && !desired.main_robot_degraded,
          "desired scout spacing did not produce a stable maintain directive");

  const ScoutDistanceAssessment stop = coordinator.assess_distance_constraint(
      main_pose, Pose2d{46.0, 0.0, 0.0, MonotonicTime{2}});
  require(stop.directive == ScoutDistanceDirective::hold_position &&
              stop.hysteresis_hold_active,
          "stop threshold did not hold the scout robot");
  const ScoutDistanceAssessment inside_band =
      coordinator.assess_distance_constraint(
          main_pose, Pose2d{42.0, 0.0, 0.0, MonotonicTime{3}});
  require(inside_band.directive == ScoutDistanceDirective::hold_position &&
              inside_band.hysteresis_hold_active,
          "distance hysteresis released inside the continue/stop band");
  const ScoutDistanceAssessment resumed = coordinator.assess_distance_constraint(
      main_pose, Pose2d{39.0, 0.0, 0.0, MonotonicTime{4}});
  require(resumed.directive == ScoutDistanceDirective::advance &&
              !resumed.hysteresis_hold_active,
          "continue threshold did not release the scout hold");

  const ScoutDistanceAssessment breached = coordinator.assess_distance_constraint(
      main_pose, Pose2d{51.0, 0.0, 0.0, MonotonicTime{5}});
  require(breached.directive == ScoutDistanceDirective::recover_communication &&
              !breached.communication_satisfied &&
              breached.main_robot_degraded &&
              breached.recommended_main_action ==
                  ScoutMainAction::stop_and_recover_communication,
          "communication hard-limit breach did not trigger safe degradation");
}

void request_lifecycle_deduplicates_times_out_and_closes_the_replan_loop() {
  const ReferenceLine reference = make_reference_line(
      9U, "map", {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0},
                    {4.0, 0.0}, {5.0, 0.0}, {6.0, 0.0}, {7.0, 0.0}});
  ScoutCoordinator coordinator(parameters());
  const InformationGap gap =
      coordinator.identify_gaps(reference, make_map(), 7.0).front();
  GapUrgencyAssessment assessment;
  assessment.urgency = GapUrgency::scheduled;
  assessment.distance_to_gap_m = gap.start_progress_m;
  const ScoutTargetGenerationResult generated = coordinator.generate_scout_target(
      gap, assessment, reference,
      Pose2d{0.0, 0.0, 0.0, MonotonicTime{10'000'000'000}},
      Pose2d{1.0, 0.0, 0.0, MonotonicTime{10'000'000'000}}, 0.0);
  require(generated.target.has_value(), "request fixture target was not generated");

  const ScoutRequestIssueResult issued = coordinator.issue_scout_request(
      *generated.target, MonotonicTime{10'000'000'000});
  require(issued.disposition == ScoutRequestIssueDisposition::issued &&
              issued.request.has_value() &&
              issued.request->request_sequence == 1U &&
              issued.request->revision == 1U &&
              issued.request->policy_version == 36U &&
              issued.request->status == ScoutRequestStatus::awaiting_map_update &&
              issued.request->expires_at.nanoseconds == 40'000'000'000,
          "issued request lost its versioned lifecycle metadata");
  const ScoutRequestIssueResult duplicate = coordinator.issue_scout_request(
      *generated.target, MonotonicTime{11'000'000'000});
  require(duplicate.disposition == ScoutRequestIssueDisposition::deduplicated &&
              duplicate.request->request_sequence == 1U &&
              duplicate.request->revision == 1U,
          "active request was not deterministically deduplicated");

  MapSnapshot unresolved = make_map();
  unresolved.version.sequence_number = 8U;
  unresolved.version.timestamp = MonotonicTime{20'000'000'000};
  for (MapCell& cell : unresolved.cells) {
    cell.measurement_timestamp = unresolved.version.timestamp;
  }
  const ScoutMapUpdateResult associated = coordinator.correlate_map_update(
      1U, reference, unresolved, MonotonicTime{20'000'000'000});
  require(associated.disposition == ScoutMapUpdateDisposition::associated_unresolved &&
              associated.request->revision == 2U &&
              !associated.trigger_replanning,
          "unresolved map update incorrectly completed the request");
  const ScoutMapUpdateResult stale = coordinator.correlate_map_update(
      1U, reference, unresolved, MonotonicTime{20'000'000'001});
  require(stale.disposition == ScoutMapUpdateDisposition::rejected &&
              stale.request->revision == 2U && !stale.issues.empty(),
          "duplicate or out-of-order map update advanced the request");

  MapSnapshot resolved = unresolved;
  resolved.version.sequence_number = 9U;
  resolved.version.timestamp = MonotonicTime{21'000'000'000};
  for (MapCell& cell : resolved.cells) {
    cell.known = true;
    cell.confidence = 1.0;
    cell.measurement_timestamp = resolved.version.timestamp;
  }
  const ScoutMapUpdateResult completed = coordinator.correlate_map_update(
      1U, reference, resolved, MonotonicTime{21'000'000'000});
  require(completed.disposition == ScoutMapUpdateDisposition::completed &&
              completed.request->revision == 3U &&
              completed.request->status == ScoutRequestStatus::completed &&
              completed.request->planning_directive ==
                  ScoutPlanningDirective::replan_with_new_map &&
              completed.invalidate_old_plan && completed.trigger_replanning &&
              completed.request->completed_map_version->sequence_number == 9U,
          "resolved gap did not close the map-update-to-replan interface");
  const ScoutMapUpdateResult terminal = coordinator.correlate_map_update(
      1U, reference, resolved, MonotonicTime{21'000'000'001});
  require(terminal.disposition == ScoutMapUpdateDisposition::request_terminal &&
              terminal.request->revision == 3U,
          "terminal request accepted another map update");

  ScoutCoordinator replay_coordinator(parameters());
  const ScoutRequestIssueResult replay_request =
      replay_coordinator.issue_scout_request(
          *generated.target, MonotonicTime{10'000'000'000});
  const ScoutMapUpdateResult replay_associated =
      replay_coordinator.correlate_map_update(
          replay_request.request->request_sequence, reference, unresolved,
          MonotonicTime{20'000'000'000});
  const ScoutMapUpdateResult replay_completed =
      replay_coordinator.correlate_map_update(
          replay_request.request->request_sequence, reference, resolved,
          MonotonicTime{21'000'000'000});
  require(same_issue_result(replay_request, issued) &&
              same_map_update_result(replay_associated, associated) &&
              same_map_update_result(replay_completed, completed),
          "fresh coordinator did not reproduce the request lifecycle fields");

  ScoutCoordinator timeout_coordinator(parameters());
  const ScoutRequestIssueResult timeout_request =
      timeout_coordinator.issue_scout_request(
          *generated.target, MonotonicTime{10'000'000'000});
  require(timeout_request.request.has_value(), "timeout request was not issued");
  const ScoutRequestExpiryResult expired =
      timeout_coordinator.expire_scout_requests(MonotonicTime{40'000'000'000});
  require(expired.valid && expired.expired.size() == 1U &&
              expired.expired.front().status == ScoutRequestStatus::timed_out &&
              expired.expired.front().revision == 2U &&
              expired.expired.front().planning_directive ==
                  ScoutPlanningDirective::waiting_map &&
              expired.expired.front().recommended_main_action ==
                  ScoutMainAction::stop_and_wait_for_map,
          "request timeout boundary did not fail safe");
  const ScoutRequestExpiryResult invalid_expiry =
      timeout_coordinator.expire_scout_requests(MonotonicTime{-1});
  require(!invalid_expiry.valid && !invalid_expiry.issues.empty(),
          "invalid timeout observation failed without diagnostics");

  ScoutCoordinator correlation_timeout(parameters());
  const ScoutRequestIssueResult correlation_request =
      correlation_timeout.issue_scout_request(
          *generated.target, MonotonicTime{10'000'000'000});
  const ScoutMapUpdateResult timed_out = correlation_timeout.correlate_map_update(
      correlation_request.request->request_sequence, reference, resolved,
      MonotonicTime{40'000'000'000});
  require(timed_out.disposition == ScoutMapUpdateDisposition::timed_out &&
              timed_out.request->status == ScoutRequestStatus::timed_out &&
              timed_out.request->planning_directive ==
                  ScoutPlanningDirective::waiting_map &&
              timed_out.invalidate_old_plan,
          "map correlation at timeout did not atomically terminate the request");

  ScoutTarget informational = *generated.target;
  informational.urgency = GapUrgency::informational;
  require(coordinator.issue_scout_request(
              informational, MonotonicTime{22'000'000'000})
              .disposition == ScoutRequestIssueDisposition::rejected,
          "record-only informational gap was issued as a scout request");
  ScoutTarget wrong_policy = *generated.target;
  wrong_policy.policy_version = 35U;
  require(coordinator.issue_scout_request(
              wrong_policy, MonotonicTime{22'000'000'000})
              .disposition == ScoutRequestIssueDisposition::rejected,
          "request accepted a target from another scout policy version");
  require(coordinator.issue_scout_request(
              *generated.target,
              MonotonicTime{std::numeric_limits<std::int64_t>::max() - 1})
              .disposition == ScoutRequestIssueDisposition::rejected,
          "nonrepresentable request expiry was accepted");
}

void off_reference_gap_requires_its_actual_target_cell_to_be_updated() {
  const ReferenceLine reference = make_reference_line(
      9U, "map", {{0.0, 0.0}, {4.0, 0.0}, {8.0, 0.0}});
  MapSnapshot source = make_map();
  source.height = 7U;
  source.origin_y_m = -3.5;
  source.cells.assign(source.width * source.height,
                      MapCell{0.0, 0.01, 1.0, true});
  for (MapCell& cell : source.cells) {
    cell.measurement_timestamp = source.version.timestamp;
  }
  InformationGap detour_gap;
  detour_gap.row = 5U;
  detour_gap.column = 4U;
  detour_gap.center = {4.0, 2.0};
  detour_gap.start_progress_m = 4.0;
  detour_gap.end_progress_m = 4.0;
  detour_gap.minimum_confidence = 0.0;
  detour_gap.source_map_version = source.version;
  detour_gap.reference_line_version = reference.version;
  GapUrgencyAssessment assessment;
  assessment.urgency = GapUrgency::scheduled;
  ScoutCoordinator coordinator(parameters());
  const ScoutTargetGenerationResult target = coordinator.generate_scout_target(
      detour_gap, assessment, reference,
      Pose2d{0.0, 0.0, 0.0, MonotonicTime{10'000'000'000}},
      Pose2d{1.0, 0.0, 0.0, MonotonicTime{10'000'000'000}}, 0.0);
  require(target.target.has_value(), "off-reference scout target was rejected");
  const ScoutRequestIssueResult request = coordinator.issue_scout_request(
      *target.target, MonotonicTime{10'000'000'000});
  InformationGap mirrored_gap = detour_gap;
  mirrored_gap.center.y_m = -2.0;
  mirrored_gap.row = 1U;
  const ScoutTargetGenerationResult mirrored_target =
      coordinator.generate_scout_target(
          mirrored_gap, assessment, reference,
          Pose2d{0.0, 0.0, 0.0, MonotonicTime{10'000'000'000}},
          Pose2d{1.0, 0.0, 0.0, MonotonicTime{10'000'000'000}}, 0.0);
  const ScoutRequestIssueResult mirrored_request =
      coordinator.issue_scout_request(
          *mirrored_target.target, MonotonicTime{10'000'000'000});
  require(mirrored_request.disposition ==
              ScoutRequestIssueDisposition::issued &&
              mirrored_request.request->request_sequence !=
                  request.request->request_sequence,
          "spatially distinct gaps at the same progress were deduplicated");

  MapSnapshot update = source;
  update.version.sequence_number = 8U;
  update.version.timestamp = MonotonicTime{20'000'000'000};
  for (MapCell& cell : update.cells) {
    cell.measurement_timestamp = update.version.timestamp;
  }
  update.cells.at(detour_gap.row * update.width + detour_gap.column).known = false;
  const ScoutMapUpdateResult unresolved = coordinator.correlate_map_update(
      request.request->request_sequence, reference, update,
      MonotonicTime{20'000'000'000});
  require(unresolved.disposition ==
              ScoutMapUpdateDisposition::associated_unresolved &&
              !unresolved.trigger_replanning,
          "reference-line coverage falsely completed an off-reference gap");
}

}  // namespace

int main() {
  identifies_and_merges_gaps_with_snapshot_metadata();
  urgency_uses_approved_profile_and_explicit_fallback();
  invalid_inputs_and_hysteresis_are_deterministic();
  generates_auditable_targets_and_ranks_value_against_arrival_cost();
  distance_policy_uses_independent_thresholds_hysteresis_and_degradation();
  request_lifecycle_deduplicates_times_out_and_closes_the_replan_loop();
  off_reference_gap_requires_its_actual_target_cell_to_be_updated();
  std::cout << "T35/T36 scout coordinator checks passed\n";
}
