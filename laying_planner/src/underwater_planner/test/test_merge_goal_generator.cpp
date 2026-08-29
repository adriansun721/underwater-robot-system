#include "underwater_planner/core/merge_goal_generator.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using underwater_planner::core::CableMeanSample;
using underwater_planner::core::CableModel;
using underwater_planner::core::CableModelParameters;
using underwater_planner::core::MergeGoalGenerationParameters;
using underwater_planner::core::MergeGoalGenerator;
using underwater_planner::core::MergeGoalRejectionReason;
using underwater_planner::core::MonotonicTime;
using underwater_planner::core::Point2d;
using underwater_planner::core::ReferenceProgress;
using underwater_planner::core::RobotCapability;
using underwater_planner::core::RobotOperatingArea;
using underwater_planner::core::SensorHealthMode;
using underwater_planner::core::SurfaceEstimate;
using underwater_planner::core::TerrainEstimateStatus;
using underwater_planner::core::TerrainGradientRiskPolicy;
using underwater_planner::core::TerrainLayers;
using underwater_planner::core::TrackFootprint;
using underwater_planner::core::Vector2m;
using underwater_planner::core::make_reference_line;

constexpr double kTolerance = 1.0e-10;

void require(const bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void require_near(const double actual, const double expected,
                  const double tolerance, const std::string& message) {
  require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
          message);
}

CableModelParameters model_parameters() {
  CableModelParameters parameters;
  parameters.version = 20;
  parameters.calibration_dataset_id = "merge-goal-cable-cal-v1";
  parameters.operating_domain_id = "competition-seabed-v1";
  parameters.release_point_offset_m = {0.5, 0.25};
  parameters.touchdown_distance_m = 1.0;
  parameters.direction_response_length_m = 2.0;
  parameters.maximum_lag_angle_rad = 0.4;
  parameters.maximum_payout_tracking_error_mps = 0.1;
  parameters.payout_speed_range = {0.0, 1.0};
  parameters.maximum_payout_acceleration_mps2 = 0.4;
  parameters.maximum_tension_tracking_error_n = 10.0;
  parameters.tension_range = {10.0, 100.0};
  parameters.search_integration_step_m = 0.5;
  parameters.validation_integration_step_m = 0.02;
  parameters.touchdown_distance_variance_m2 = 0.0025;
  parameters.direction_response_length_variance_m2 = 0.04;
  parameters.lag_angle_process_variance_per_m_rad2 = 0.03;
  parameters.touchdown_process_noise_per_m_m2 = {0.001, 0.0, 0.0, 0.002};
  parameters.approved_sensor_modes = {SensorHealthMode::nominal};
  return parameters;
}

MergeGoalGenerationParameters generation_parameters() {
  MergeGoalGenerationParameters parameters;
  parameters.version = 3;
  parameters.merge_distances_m = {2.0, 4.0};
  parameters.terminal_heading_offsets_rad = {-0.4, 0.0, 0.4};
  parameters.terminal_lag_angles_rad = {-0.4, 0.0, 0.4};
  parameters.maximum_goal_count = 10;
  parameters.forward_position_tolerance_m = 1.0e-9;
  parameters.forward_heading_tolerance_rad = 1.0e-9;
  parameters.merge_distance_cost_weight = 1.0;
  parameters.absolute_lag_cost_weight = 10.0;
  return parameters;
}

std::vector<Point2d> footprint() {
  return {{-0.5, -0.4}, {0.5, -0.4}, {0.5, 0.4}, {-0.5, 0.4}};
}

TrackFootprint track_footprint() {
  return {footprint(),
          {{-0.4, 0.1}, {0.4, 0.1}, {0.4, 0.35}, {-0.4, 0.35}},
          {{-0.4, -0.35}, {0.4, -0.35}, {0.4, -0.1}, {-0.4, -0.1}}};
}

RobotCapability robot_capability() {
  RobotCapability capability;
  capability.maximum_slope_up_rad = 0.8;
  capability.maximum_slope_down_rad = 0.8;
  capability.maximum_slope_lateral_rad = 0.8;
  capability.maximum_support_roll_rad = 0.8;
  capability.maximum_step_climb_m = 0.3;
  capability.maximum_step_drop_m = 0.3;
  capability.minimum_track_support_ratio = 0.5;
  capability.effective_track_spacing_m = 0.5;
  capability.minimum_step_crossing_alignment = 0.2;
  capability.step_alignment_transition_band = 0.1;
  capability.maximum_roughness_m = 1.0;
  return capability;
}

TerrainLayers flat_terrain() {
  TerrainLayers terrain;
  terrain.source_map_version =
      {"merge-goal-map", 13U, MonotonicTime{1'000'000'000}, "map"};
  terrain.analysis_config_version = 17U;
  terrain.operating_domain_id = "terrain-domain-v1";
  terrain.surface_fit_window_size_m = 1.0;
  terrain.surface.width = 50U;
  terrain.surface.height = 20U;
  terrain.surface.resolution_m = 1.0;
  terrain.surface.origin_x_m = -10.0;
  terrain.surface.origin_y_m = -10.0;
  terrain.surface.cells.assign(terrain.surface.width * terrain.surface.height,
                               SurfaceEstimate{});
  for (SurfaceEstimate& estimate : terrain.surface.cells) {
    estimate.status = TerrainEstimateStatus::valid;
    estimate.support_ratio = 1.0;
  }
  return terrain;
}

TerrainGradientRiskPolicy terrain_policy() {
  return {19U,
          17U,
          0.05,
          2.0,
          underwater_planner::core::GradientCoverageModel::empirical_bounded,
          "merge-goal-gradient-cal-v1",
          "terrain-domain-v1",
          true};
}

RobotOperatingArea operating_area(const double maximum_x_m = 30.0) {
  return {7,
          "robot-area-v7",
          {{-5.0, -5.0},
           {maximum_x_m, -5.0},
           {maximum_x_m, 5.0},
           {-5.0, 5.0}}};
}

underwater_planner::core::ReferenceLine reference_line() {
  return make_reference_line(11, "map", {{0.0, 0.0}, {20.0, 0.0}});
}

ReferenceProgress current_progress() {
  return {11, 2.0, MonotonicTime{1'000'000'000}, 9};
}

void inverse_targets_close_through_the_forward_cable_model() {
  // Design: 18.2.3-18
  const CableModel forward_model(model_parameters());
  const MergeGoalGenerator generator(model_parameters(),
                                     generation_parameters(), robot_capability(),
                                     track_footprint());
  const auto result = generator.generate(current_progress(), reference_line(),
                                         operating_area(), flat_terrain(),
                                         terrain_policy());

  require(result.valid_input, "valid merge-goal inputs were rejected");
  require(result.goals.size() == 6U,
          "heading/lag combinations not aligned with the reference survived");
  bool saw_nonzero_lag_boundary = false;
  for (const auto& goal : result.goals) {
    require(std::hypot(goal.robot_pose.x_m - goal.touchdown_target_m.x_m,
                       goal.robot_pose.y_m - goal.touchdown_target_m.y_m) >
                0.1,
            "robot target was incorrectly collapsed onto touchdown target");
    const CableMeanSample forward = forward_model.predict_touchdown_mean(
        goal.robot_pose, goal.cable_lag_angle_rad);
    require(forward.validity ==
                underwater_planner::core::CableModelValidity::valid,
            "accepted goal could not be evaluated by the forward cable model");
    require_near(forward.touchdown_position_m.x_m,
                 goal.touchdown_target_m.x_m, kTolerance,
                 "inverse goal missed touchdown x under forward prediction");
    require_near(forward.touchdown_position_m.y_m,
                 goal.touchdown_target_m.y_m, kTolerance,
                 "inverse goal missed touchdown y under forward prediction");
    require_near(forward.cable_heading_rad, goal.cable_heading_rad, kTolerance,
                 "inverse goal missed the reference tangent heading");
    require_near(goal.reference_progress_m,
                 current_progress().arc_length_m + goal.merge_distance_m,
                 kTolerance,
                 "future goal progress was not stored as a candidate value");
    saw_nonzero_lag_boundary =
        saw_nonzero_lag_boundary ||
        std::abs(std::abs(goal.cable_lag_angle_rad) - 0.4) <= kTolerance;
  }
  require(saw_nonzero_lag_boundary,
          "calibrated lag-angle boundary candidates were not retained");
}

void candidates_span_merge_distances_and_use_only_explicit_soft_costs() {
  const MergeGoalGenerator generator(model_parameters(),
                                     generation_parameters(), robot_capability(),
                                     track_footprint());
  const auto result = generator.generate(current_progress(), reference_line(),
                                         operating_area(), flat_terrain(),
                                         terrain_policy());

  bool saw_near = false;
  bool saw_far = false;
  double previous_total = -1.0;
  for (const auto& goal : result.goals) {
    saw_near = saw_near || std::abs(goal.merge_distance_m - 2.0) <= kTolerance;
    saw_far = saw_far || std::abs(goal.merge_distance_m - 4.0) <= kTolerance;
    require_near(goal.soft_cost.merge_distance_component,
                 goal.merge_distance_m, kTolerance,
                 "merge-distance soft cost was not explicit");
    require_near(goal.soft_cost.absolute_lag_component,
                 10.0 * std::abs(goal.cable_lag_angle_rad), kTolerance,
                 "lag soft cost was not explicit");
    require_near(goal.soft_cost.total,
                 goal.soft_cost.merge_distance_component +
                     goal.soft_cost.absolute_lag_component,
                 kTolerance, "candidate total contains a hidden cost");
    require(goal.soft_cost.total >= previous_total,
            "feasible candidates were not stably sorted by explicit soft cost");
    previous_total = goal.soft_cost.total;
  }
  require(saw_near && saw_far,
          "candidate goals did not cover the configured merge distances");
}

void forward_misses_and_out_of_domain_lags_are_rejected() {
  auto policy = generation_parameters();
  policy.terminal_heading_offsets_rad = {0.0};
  policy.terminal_lag_angles_rad = {0.0, 0.2, 0.5};
  const MergeGoalGenerator generator(model_parameters(), policy,
                                     robot_capability(), track_footprint());
  const auto result = generator.generate(current_progress(), reference_line(),
                                         operating_area(), flat_terrain(),
                                         terrain_policy());

  require(result.goals.size() == 2U,
          "only aligned zero-lag candidates at both distances should survive");
  bool saw_forward_miss = false;
  bool saw_lag_out_of_range = false;
  for (const auto& diagnostic : result.diagnostics) {
    saw_forward_miss =
        saw_forward_miss ||
        diagnostic.reason == MergeGoalRejectionReason::forward_model_mismatch;
    saw_lag_out_of_range =
        saw_lag_out_of_range ||
        diagnostic.reason == MergeGoalRejectionReason::lag_angle_out_of_range;
  }
  require(saw_forward_miss,
          "a forward prediction that missed the touchdown target was not audited");
  require(saw_lag_out_of_range,
          "a lag outside the calibrated cable-model domain was not rejected");
}

void the_complete_terminal_footprint_must_remain_in_the_robot_area() {
  auto policy = generation_parameters();
  policy.merge_distances_m = {2.0};
  policy.terminal_heading_offsets_rad = {0.0};
  policy.terminal_lag_angles_rad = {0.0};
  const MergeGoalGenerator generator(model_parameters(), policy,
                                     robot_capability(), track_footprint());
  const auto result = generator.generate(current_progress(), reference_line(),
                                         operating_area(4.75), flat_terrain(),
                                         terrain_policy());

  require(result.goals.empty(),
          "a terminal footprint crossing the robot-area boundary was accepted");
  require(result.diagnostics.size() == 1U &&
              result.diagnostics.front().reason ==
                  MergeGoalRejectionReason::robot_footprint_outside_area,
          "robot-area rejection did not identify the hard feasibility gate");
}

void a_concave_robot_area_notch_cannot_hide_inside_the_terminal_footprint() {
  auto policy = generation_parameters();
  policy.merge_distances_m = {2.0};
  policy.terminal_heading_offsets_rad = {0.0};
  policy.terminal_lag_angles_rad = {0.0};
  const MergeGoalGenerator generator(model_parameters(), policy,
                                     robot_capability(), track_footprint());
  const RobotOperatingArea notched_area{
      8,
      "notched-robot-area-v8",
      {{-5.0, -5.0},
       {30.0, -5.0},
       {30.0, 0.15},
       {4.3, 0.15},
       {4.2, -0.2},
       {4.1, 0.15},
       {-5.0, 0.15}}};
  const auto result = generator.generate(current_progress(), reference_line(),
                                         notched_area, flat_terrain(),
                                         terrain_policy());

  require(result.goals.empty(),
          "an operating-area notch hidden inside the footprint was accepted");
  require(!result.diagnostics.empty() &&
              result.diagnostics.front().reason ==
                  MergeGoalRejectionReason::robot_footprint_outside_area,
          "the concave-area rejection did not report the hard workspace gate");
}

void terminal_terrain_must_be_traversable() {
  auto policy = generation_parameters();
  policy.merge_distances_m = {2.0};
  policy.terminal_heading_offsets_rad = {0.0};
  policy.terminal_lag_angles_rad = {0.0};
  const MergeGoalGenerator generator(model_parameters(), policy,
                                     robot_capability(), track_footprint());
  TerrainLayers terrain = flat_terrain();
  for (SurfaceEstimate& estimate : terrain.surface.cells) {
    estimate.gradient_x = 2.0;
  }
  const auto result = generator.generate(current_progress(), reference_line(),
                                         operating_area(), terrain,
                                         terrain_policy());

  require(result.goals.empty(),
          "a merge goal on non-traversable terrain was accepted");
  require(!result.diagnostics.empty() &&
              result.diagnostics.front().reason ==
                  MergeGoalRejectionReason::terminal_terrain_not_traversable,
          "terrain rejection did not identify the terminal hard gate");
}

void versions_invalid_progress_and_goal_limit_are_fail_closed_and_deterministic() {
  auto policy = generation_parameters();
  policy.maximum_goal_count = 2;
  const MergeGoalGenerator generator(model_parameters(), policy,
                                     robot_capability(), track_footprint());
  const auto first = generator.generate(current_progress(), reference_line(),
                                        operating_area(), flat_terrain(),
                                        terrain_policy());
  const auto second = generator.generate(current_progress(), reference_line(),
                                         operating_area(), flat_terrain(),
                                         terrain_policy());
  require(first.goals.size() == 2U && second.goals.size() == 2U,
          "goal-count limit was not enforced");
  require(first.cable_calibration_dataset_id == "merge-goal-cable-cal-v1" &&
              first.operating_domain_id == "competition-seabed-v1" &&
              first.reference_coordinate_frame == "map" &&
              first.robot_operating_area_id == "robot-area-v7" &&
              first.terrain_risk_audit.source_map_version.map_id ==
                  "merge-goal-map" &&
              first.terrain_risk_audit.gradient_risk_policy_version == 19U &&
              first.terrain_risk_audit.source_terrain_analysis_config_version ==
                  17U &&
              first.terrain_risk_audit.policy_terrain_analysis_config_version ==
                  17U &&
              first.terrain_risk_audit.calibration_dataset_id ==
                  "merge-goal-gradient-cal-v1" &&
              !first.terrain_risk_audit.path_joint_risk_implemented,
          "goal generation omitted locked calibration or spatial-domain identity");
  bool saw_goal_limit = false;
  for (const auto& diagnostic : first.diagnostics) {
    saw_goal_limit =
        saw_goal_limit ||
        diagnostic.reason == MergeGoalRejectionReason::goal_limit_reached;
  }
  require(saw_goal_limit,
          "feasible candidates removed by the goal limit were not audited");
  for (std::size_t index = 0; index < first.goals.size(); ++index) {
    require(first.goals[index].robot_pose.x_m == second.goals[index].robot_pose.x_m &&
                first.goals[index].robot_pose.y_m == second.goals[index].robot_pose.y_m &&
                first.goals[index].cable_lag_angle_rad ==
                    second.goals[index].cable_lag_angle_rad &&
                first.goals[index].soft_cost.total ==
                    second.goals[index].soft_cost.total,
            "fixed inputs did not produce deterministic merge goals");
  }

  ReferenceProgress mismatched = current_progress();
  mismatched.reference_line_version = 12;
  const auto mismatch = generator.generate(mismatched, reference_line(),
                                           operating_area(), flat_terrain(),
                                           terrain_policy());
  require(!mismatch.valid_input && mismatch.goals.empty(),
          "reference-version mismatch did not fail closed");

  ReferenceProgress beyond_end = current_progress();
  beyond_end.arc_length_m = 19.0;
  const auto outside = generator.generate(beyond_end, reference_line(),
                                          operating_area(), flat_terrain(),
                                          terrain_policy());
  require(outside.valid_input && outside.goals.empty() &&
              !outside.diagnostics.empty(),
          "merge progress beyond the reference line was not diagnosed");
}

}  // namespace

int main() {
  try {
    inverse_targets_close_through_the_forward_cable_model();
    candidates_span_merge_distances_and_use_only_explicit_soft_costs();
    forward_misses_and_out_of_domain_lags_are_rejected();
    the_complete_terminal_footprint_must_remain_in_the_robot_area();
    a_concave_robot_area_notch_cannot_hide_inside_the_terminal_footprint();
    terminal_terrain_must_be_traversable();
    versions_invalid_progress_and_goal_limit_are_fail_closed_and_deterministic();
    std::cout << "merge goal generator tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "merge goal generator test failure: " << error.what() << '\n';
    return 1;
  }
}
