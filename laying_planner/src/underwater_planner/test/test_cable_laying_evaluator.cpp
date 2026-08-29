#include "underwater_planner/core/cable_laying_evaluator.hpp"
#include "underwater_planner/core/cable_state_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using underwater_planner::core::CableConstraintMemory;
using underwater_planner::core::CableHistoryBoundary;
using underwater_planner::core::CableLayingEvaluator;
using underwater_planner::core::CableLayingFailure;
using underwater_planner::core::CableLayingLimits;
using underwater_planner::core::CableState;
using underwater_planner::core::CableStateKind;
using underwater_planner::core::GeometricPath;
using underwater_planner::core::PathPoint;
using underwater_planner::core::TerrainLayers;

void require(const bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

bool has_failure(
    const underwater_planner::core::CableLayingEvaluation& result,
    const CableLayingFailure failure) {
  return std::find(result.failure_reasons.begin(), result.failure_reasons.end(),
                   failure) != result.failure_reasons.end();
}

CableLayingLimits limits() {
  CableLayingLimits value;
  value.version = 16;
  value.operating_domain_id = "competition-seabed-v1";
  value.preferred_curvature_per_m = 0.2;
  value.maximum_curvature_per_m = 0.4;
  value.curvature_evaluation_spacing_m = 0.5;
  value.support_evaluation_length_m = 2.0;
  value.medium_support_proxy_range_m = 0.1;
  value.maximum_support_proxy_range_m = 0.3;
  value.minimum_terrain_confidence = 0.5;
  value.minimum_distinct_touchdown_distance_m = 1.0e-6;
  value.bend_weight = 1.0;
  value.terrain_risk_weight = 1.0;
  value.roughness_weight = 1.0;
  return value;
}

TerrainLayers flat_terrain() {
  TerrainLayers terrain;
  terrain.source_map_version = {"map-t16", 16, {16'000}, "world"};
  terrain.analysis_config_version = 6;
  terrain.operating_domain_id = "competition-seabed-v1";
  terrain.surface.width = 80;
  terrain.surface.height = 80;
  terrain.surface.resolution_m = 0.1;
  terrain.surface.origin_x_m = -4.0;
  terrain.surface.origin_y_m = -4.0;
  terrain.surface.cells.resize(terrain.surface.width * terrain.surface.height);
  terrain.cable_laying.cells.resize(terrain.surface.cells.size());
  for (std::size_t index = 0; index < terrain.surface.cells.size(); ++index) {
    auto& surface = terrain.surface.cells[index];
    surface.status = underwater_planner::core::TerrainEstimateStatus::valid;
    surface.support_ratio = 1.0;
    terrain.cable_laying.cells[index].known = true;
    terrain.cable_laying.cells[index].confidence = 1.0;
  }
  return terrain;
}

std::size_t terrain_index(const TerrainLayers& terrain, const double x_m,
                          const double y_m) {
  const auto column = static_cast<std::size_t>(
      std::floor((x_m - terrain.surface.origin_x_m) /
                 terrain.surface.resolution_m));
  const auto row = static_cast<std::size_t>(
      std::floor((y_m - terrain.surface.origin_y_m) /
                 terrain.surface.resolution_m));
  return row * terrain.surface.width + column;
}

GeometricPath straight_path() {
  GeometricPath path;
  path.metadata = {161, "world", 4, "linear"};
  path.points = {
      PathPoint{0.0, 0.0, 0.0, 0.0, 0.0},
      PathPoint{3.0, 3.0, 0.0, 0.0, 0.0},
  };
  return path;
}

GeometricPath sampled_straight_path(const double interval_m) {
  GeometricPath path;
  path.metadata = {163, "world", 4, "linear"};
  for (double arc_length_m = 0.0; arc_length_m < 3.0;
       arc_length_m += interval_m) {
    path.points.push_back(
        {arc_length_m, arc_length_m, 0.0, 0.0, 0.0});
  }
  path.points.push_back({3.0, 3.0, 0.0, 0.0, 0.0});
  return path;
}

CableConstraintMemory complete_history() {
  CableConstraintMemory memory;
  memory.previous_distinct_touchdown_points_m = {{-1.0, 0.0}, {0.0, 0.0}};
  memory.trailing_support_samples = {
      {0.0, {-2.0, 0.0}},
      {1.0, {-1.0, 0.0}},
      {2.0, {0.0, 0.0}},
  };
  memory.retained_arc_length_m = 2.0;
  memory.canonical_signature = 44;
  return memory;
}

GeometricPath arc_path(const double curvature_per_m) {
  GeometricPath path;
  path.metadata = {160, "world", 4, "cable-mean-spatial-lag"};
  const double radius_m = 1.0 / std::abs(curvature_per_m);
  double touchdown_arc_length_m = 0.0;
  for (const double arc_length_m : {0.0, 0.5, 1.0, 1.5}) {
    const double signed_angle = curvature_per_m * arc_length_m;
    PathPoint sample{0.0, radius_m * std::sin(std::abs(signed_angle)),
                     std::copysign(
                         radius_m * (1.0 - std::cos(signed_angle)),
                         curvature_per_m),
                     signed_angle, curvature_per_m};
    if (!path.points.empty()) {
      touchdown_arc_length_m +=
          std::hypot(sample.x_m - path.points.back().x_m,
                     sample.y_m - path.points.back().y_m);
    }
    sample.arc_length_m = touchdown_arc_length_m;
    path.points.push_back(sample);
  }
  return path;
}

std::vector<CableState> state_profile(const GeometricPath& path) {
  std::vector<CableState> profile(path.points.size());
  for (CableState& state : profile) {
    state.kind = CableStateKind::search_mean;
    state.lag_angle_rad = 0.0;
    state.timestamp = {16'000};
    state.sequence_number = 1;
  }
  return profile;
}

void left_and_right_curvature_share_the_hard_limit() {
  // Design: 18.2.4-20
  const CableLayingEvaluator evaluator;
  const TerrainLayers terrain = flat_terrain();
  CableLayingLimits test_limits = limits();
  test_limits.maximum_curvature_per_m = 0.35;
  test_limits.bend_weight = 0.0;

  const GeometricPath left = arc_path(0.5);
  const GeometricPath right = arc_path(-0.5);
  const auto left_result = evaluator.evaluate(
      CableConstraintMemory{}, left, state_profile(left), terrain, test_limits,
      CableHistoryBoundary::explicit_task_start);
  const auto right_result = evaluator.evaluate(
      CableConstraintMemory{}, right, state_profile(right), terrain, test_limits,
      CableHistoryBoundary::explicit_task_start);

  require(left_result.valid && right_result.valid,
          "finite circular paths were not evaluated");
  require(!left_result.hard_feasible && !right_result.hard_feasible,
          "a zero bend weight bypassed the mechanical hard limit");
  require(has_failure(left_result, CableLayingFailure::curvature_exceeded) &&
              has_failure(right_result,
                          CableLayingFailure::curvature_exceeded),
          "left/right curvature did not report the same hard failure");
  require(std::abs(left_result.maximum_absolute_curvature_per_m -
                   right_result.maximum_absolute_curvature_per_m) < 1.0e-12,
          "left/right curvature magnitudes were asymmetric");
  require(left_result.maximum_absolute_curvature_position_m.has_value() &&
              right_result.maximum_absolute_curvature_position_m.has_value(),
          "maximum curvature omitted its corresponding position");
  require(!left_result.failure_segments.empty() &&
              left_result.failure_segments.front().start_arc_length_m >= 0.0,
          "curvature failure omitted its auditable path segment");
}

void swept_forbidden_unknown_and_low_confidence_cells_fail_hard() {
  // Design: 18.2.4-22
  const CableLayingEvaluator evaluator;
  const GeometricPath path = straight_path();
  const auto profile = state_profile(path);

  TerrainLayers forbidden = flat_terrain();
  forbidden.cable_laying.cells[terrain_index(forbidden, 1.5, 0.0)]
      .forbidden = true;
  const auto forbidden_result = evaluator.evaluate(
      CableConstraintMemory{}, path, profile, forbidden, limits(),
      CableHistoryBoundary::explicit_task_start);
  require(forbidden_result.valid && !forbidden_result.hard_feasible &&
              has_failure(
                  forbidden_result,
                  CableLayingFailure::forbidden_area_intersection),
          "a forbidden cell between path samples was not a hard failure");

  TerrainLayers obstacle = flat_terrain();
  obstacle.cable_laying.cells[terrain_index(obstacle, 1.5, 0.0)].obstacle =
      true;
  const auto obstacle_result = evaluator.evaluate(
      CableConstraintMemory{}, path, profile, obstacle, limits(),
      CableHistoryBoundary::explicit_task_start);
  require(obstacle_result.valid && !obstacle_result.hard_feasible &&
              has_failure(
                  obstacle_result,
                  CableLayingFailure::forbidden_area_intersection),
          "an obstacle occupancy between path samples was not a hard failure");

  TerrainLayers unknown = flat_terrain();
  unknown.cable_laying.cells[terrain_index(unknown, 1.5, 0.0)].known = false;
  const auto unknown_result = evaluator.evaluate(
      CableConstraintMemory{}, path, profile, unknown, limits(),
      CableHistoryBoundary::explicit_task_start);
  require(unknown_result.valid && !unknown_result.hard_feasible &&
              has_failure(unknown_result,
                          CableLayingFailure::terrain_data_invalid),
          "unknown terrain between path samples was not a hard failure");

  TerrainLayers low_confidence = flat_terrain();
  low_confidence
      .cable_laying.cells[terrain_index(low_confidence, 1.5, 0.0)]
      .confidence = 0.49;
  const auto low_confidence_result = evaluator.evaluate(
      CableConstraintMemory{}, path, profile, low_confidence, limits(),
      CableHistoryBoundary::explicit_task_start);
  require(low_confidence_result.valid &&
              !low_confidence_result.hard_feasible &&
              has_failure(low_confidence_result,
                          CableLayingFailure::terrain_data_invalid),
          "low-confidence terrain was not a hard failure");
}

void coordinate_frame_and_arc_length_mismatches_fail_closed() {
  const CableLayingEvaluator evaluator;
  GeometricPath wrong_frame = straight_path();
  wrong_frame.metadata.coordinate_frame = "robot";
  const auto frame_result = evaluator.evaluate(
      CableConstraintMemory{}, wrong_frame, state_profile(wrong_frame),
      flat_terrain(), limits(), CableHistoryBoundary::explicit_task_start);
  require(!frame_result.valid && !frame_result.hard_feasible &&
              has_failure(frame_result,
                          CableLayingFailure::numerically_invalid) &&
              frame_result.limits_version == limits().version &&
              frame_result.terrain_map_sequence ==
                  flat_terrain().source_map_version.sequence_number &&
              frame_result.terrain_analysis_config_version ==
                  flat_terrain().analysis_config_version &&
              frame_result.operating_domain_id ==
                  flat_terrain().operating_domain_id &&
              !frame_result.risk_semantics.empty(),
          "a touchdown path in a different map frame was evaluated");

  GeometricPath forged_arc = straight_path();
  forged_arc.points.back().arc_length_m = 0.1;
  const auto arc_result = evaluator.evaluate(
      CableConstraintMemory{}, forged_arc, state_profile(forged_arc),
      flat_terrain(), limits(), CableHistoryBoundary::explicit_task_start);
  require(!arc_result.valid && !arc_result.hard_feasible &&
              has_failure(arc_result,
                          CableLayingFailure::numerically_invalid),
          "an arc length shorter than its geometric chord diluted the sweep");

  forged_arc = straight_path();
  forged_arc.points.back().arc_length_m = 10.0;
  const auto oversized_result = evaluator.evaluate(
      CableConstraintMemory{}, forged_arc, state_profile(forged_arc),
      flat_terrain(), limits(), CableHistoryBoundary::explicit_task_start);
  require(!oversized_result.valid && !oversized_result.hard_feasible,
          "an arc length larger than its linear geometry truncated the window");

  GeometricPath unsupported = straight_path();
  unsupported.metadata.interpolation_rule = "circular-arc";
  const auto interpolation_result = evaluator.evaluate(
      CableConstraintMemory{}, unsupported, state_profile(unsupported),
      flat_terrain(), limits(), CableHistoryBoundary::explicit_task_start);
  require(!interpolation_result.valid &&
              !interpolation_result.hard_feasible,
          "an uninterpreted nonlinear touchdown rule was swept as a chord");
}

void categorical_terrain_uses_conservative_grid_supercover() {
  const CableLayingEvaluator evaluator;
  GeometricPath path;
  path.metadata = {168, "world", 4, "linear"};
  const double end_y_m = 0.09542;
  const double length_m = std::hypot(0.22, end_y_m - 0.12);
  path.points = {
      {0.0, 0.02, 0.12, 0.0, 0.0},
      {length_m, 0.24, end_y_m, 0.0, 0.0},
  };
  TerrainLayers terrain = flat_terrain();
  terrain.cable_laying.cells[terrain_index(terrain, 0.15, 0.05)]
      .forbidden = true;
  const auto result = evaluator.evaluate(
      CableConstraintMemory{}, path, state_profile(path), terrain, limits(),
      CableHistoryBoundary::explicit_task_start);
  require(result.valid && !result.hard_feasible &&
              has_failure(result,
                          CableLayingFailure::forbidden_area_intersection),
          "a short diagonal crossing skipped a forbidden grid cell");
}

void repeated_touchdown_points_are_explicitly_rejected() {
  const CableLayingEvaluator evaluator;
  GeometricPath path;
  path.metadata = {162, "world", 4, "linear"};
  path.points = {
      PathPoint{0.0, 0.0, 0.0, 0.0, 0.0},
      PathPoint{1.0, 1.0, 0.0, 0.0, 0.0},
      PathPoint{2.0, 1.0, 0.0, 0.0, 0.0},
  };
  const auto result = evaluator.evaluate(
      CableConstraintMemory{}, path, state_profile(path), flat_terrain(),
      limits(), CableHistoryBoundary::explicit_task_start);
  require(!result.valid && !result.hard_feasible &&
              has_failure(result,
                          CableLayingFailure::duplicate_touchdown_point),
          "a repeated touchdown point was treated as zero curvature");
}

void support_proxy_is_physical_window_and_sampling_invariant() {
  // Design: 18.2.4-23
  const CableLayingEvaluator evaluator;
  TerrainLayers terrain = flat_terrain();
  for (double x_m = 1.0; x_m < 1.5; x_m += terrain.surface.resolution_m) {
    terrain.cable_laying.cells[terrain_index(terrain, x_m, 0.0)]
        .elevation_m = 0.4;
  }
  CableLayingLimits test_limits = limits();
  test_limits.terrain_risk_weight = 0.0;

  const GeometricPath coarse = sampled_straight_path(1.0);
  const GeometricPath fine = sampled_straight_path(0.2);
  const auto coarse_result = evaluator.evaluate(
      CableConstraintMemory{}, coarse, state_profile(coarse), terrain,
      test_limits, CableHistoryBoundary::explicit_task_start);
  const auto fine_result = evaluator.evaluate(
      CableConstraintMemory{}, fine, state_profile(fine), terrain, test_limits,
      CableHistoryBoundary::explicit_task_start);

  require(coarse_result.valid && fine_result.valid &&
              !coarse_result.hard_feasible && !fine_result.hard_feasible &&
              has_failure(coarse_result,
                          CableLayingFailure::support_proxy_exceeded) &&
              has_failure(fine_result,
                          CableLayingFailure::support_proxy_exceeded),
          "support proxy depended on touchdown input sampling");
  require(std::abs(coarse_result.maximum_support_proxy_range_m -
                   fine_result.maximum_support_proxy_range_m) < 1.0e-12,
          "fixed physical support window changed with input sampling");
  require(coarse_result.maximum_support_proxy_position_m.has_value() &&
              fine_result.maximum_support_proxy_position_m.has_value(),
          "maximum support proxy omitted its corresponding position");
}

void curvature_is_invariant_to_collinear_input_resampling() {
  const CableLayingEvaluator evaluator;
  GeometricPath coarse;
  coarse.metadata = {169, "world", 4, "linear"};
  coarse.points = {
      {0.0, 0.0, 0.0, 0.0, 0.0},
      {1.0, 1.0, 0.0, 0.0, 0.0},
      {1.0 + std::sqrt(2.0), 2.0, 1.0, 0.0, 0.0},
  };
  GeometricPath fine;
  fine.metadata = coarse.metadata;
  fine.points = {
      {0.0, 0.0, 0.0, 0.0, 0.0},
      {0.9, 0.9, 0.0, 0.0, 0.0},
      {1.0, 1.0, 0.0, 0.0, 0.0},
      {1.0 + std::sqrt(0.02), 1.1, 0.1, 0.0, 0.0},
      {1.0 + std::sqrt(2.0), 2.0, 1.0, 0.0, 0.0},
  };
  CableLayingLimits test_limits = limits();
  test_limits.maximum_curvature_per_m = 2.0;
  const auto coarse_result = evaluator.evaluate(
      CableConstraintMemory{}, coarse, state_profile(coarse), flat_terrain(),
      test_limits, CableHistoryBoundary::explicit_task_start);
  const auto fine_result = evaluator.evaluate(
      CableConstraintMemory{}, fine, state_profile(fine), flat_terrain(),
      test_limits, CableHistoryBoundary::explicit_task_start);
  require(coarse_result.valid && fine_result.valid &&
              coarse_result.hard_feasible && fine_result.hard_feasible &&
              coarse_result.soft_cost > 0.0 && fine_result.soft_cost > 0.0 &&
              std::abs(coarse_result.maximum_absolute_curvature_per_m -
                       fine_result.maximum_absolute_curvature_per_m) <
                  1.0e-12 &&
              std::abs(coarse_result.soft_cost - fine_result.soft_cost) <
                  1.0e-12,
          "collinear input resampling changed curvature evaluation");

  test_limits.maximum_curvature_per_m = 1.0;
  const auto coarse_failure = evaluator.evaluate(
      CableConstraintMemory{}, coarse, state_profile(coarse), flat_terrain(),
      test_limits, CableHistoryBoundary::explicit_task_start);
  const auto fine_failure = evaluator.evaluate(
      CableConstraintMemory{}, fine, state_profile(fine), flat_terrain(),
      test_limits, CableHistoryBoundary::explicit_task_start);
  require(!coarse_failure.hard_feasible && !fine_failure.hard_feasible &&
              coarse_failure.failure_segments.size() == 1U &&
              fine_failure.failure_segments.size() == 1U &&
              std::abs(coarse_failure.failure_segments.front()
                           .start_arc_length_m -
                       fine_failure.failure_segments.front()
                           .start_arc_length_m) < 1.0e-12 &&
              std::abs(coarse_failure.failure_segments.front()
                           .end_arc_length_m -
                       fine_failure.failure_segments.front().end_arc_length_m) <
                  1.0e-12,
          "collinear input resampling changed the curvature failure segment");
}

void curvature_normalization_remains_conservative_at_corners() {
  GeometricPath path;
  path.metadata = {171, "world", 4, "linear"};
  path.points = {
      {0.0, 0.0, 0.0, 0.0, 0.0},
      {1.0, 1.0, 0.0, 0.0, 0.0},
      {2.0, 2.0, 0.0, 0.0, 0.0},
      {2.0 + std::sqrt(2.0), 3.0, 1.0, 0.0, 0.0},
  };
  CableLayingLimits test_limits = limits();
  test_limits.maximum_curvature_per_m = 0.5;
  const auto result = CableLayingEvaluator{}.evaluate(
      CableConstraintMemory{}, path, state_profile(path), flat_terrain(),
      test_limits, CableHistoryBoundary::explicit_task_start);
  require(result.valid && !result.hard_feasible &&
              has_failure(result, CableLayingFailure::curvature_exceeded),
          "collinear normalization diluted a sharp corner below the limit");
  require(result.failure_segments.size() == 1U &&
              std::abs(result.failure_segments.front()
                           .representative_position_m.x_m -
                       2.0) < 1.0e-12 &&
              std::abs(result.failure_segments.front()
                           .representative_position_m.y_m) < 1.0e-12,
          "curvature failure was not located at its evaluation center");
}

void actual_history_is_required_and_requeried_on_the_current_map() {
  // Design: 18.2.4-27
  const CableLayingEvaluator evaluator;
  GeometricPath path;
  path.metadata = {164, "world", 4, "linear"};
  path.points = {
      {0.0, 0.0, 0.0, 0.0, 0.0},
      {1.0, 1.0, 0.0, 0.0, 0.0},
  };
  TerrainLayers changed_map = flat_terrain();
  changed_map.cable_laying.cells[terrain_index(changed_map, -1.0, 0.0)]
      .elevation_m = 0.4;
  const auto changed_result = evaluator.evaluate(
      complete_history(), path, state_profile(path), changed_map, limits(),
      CableHistoryBoundary::actual_laying_history);
  require(changed_result.valid && !changed_result.hard_feasible &&
              has_failure(changed_result,
                          CableLayingFailure::support_proxy_exceeded),
          "updated terrain beneath actual history was not re-evaluated");

  CableConstraintMemory incomplete;
  incomplete.previous_distinct_touchdown_points_m = {{-1.0, 0.0},
                                                      {0.0, 0.0}};
  incomplete.trailing_support_samples = {{0.0, {-1.0, 0.0}},
                                         {1.0, {0.0, 0.0}}};
  incomplete.retained_arc_length_m = 1.0;
  const auto incomplete_result = evaluator.evaluate(
      incomplete, path, state_profile(path), flat_terrain(), limits(),
      CableHistoryBoundary::actual_laying_history);
  require(!incomplete_result.valid && !incomplete_result.hard_feasible &&
              has_failure(
                  incomplete_result,
                  CableLayingFailure::mechanical_history_incomplete),
          "insufficient actual support history was silently treated as task start");

  underwater_planner::core::CableStateTrackerConfig tracker_config;
  tracker_config.cable_model_version = 16;
  tracker_config.calibration_dataset_id = "t16-tracker-integration/v1";
  tracker_config.operating_domain_id = "competition-seabed-v1";
  tracker_config.touchdown_distance_m = 1.0;
  tracker_config.direction_response_length_m = 2.0;
  tracker_config.curvature_evaluation_spacing_m = 0.5;
  tracker_config.support_evaluation_length_m = 0.2;
  tracker_config.minimum_distinct_touchdown_distance_m = 1.0e-6;
  tracker_config.initial_lag_variance_rad2 = 0.25;
  tracker_config.process_variance_per_m_rad2 = 0.01;
  tracker_config.maximum_touchdown_observation_residual_m = 0.1;
  tracker_config.maximum_observation_gap = {2'000'000'000};
  tracker_config.synchronization_tolerance = {50'000'000};
  tracker_config.maximum_payout_speed_error_mps = 0.1;
  tracker_config.minimum_tension_n = 20.0;
  tracker_config.maximum_tension_n = 200.0;
  underwater_planner::core::CableStateTracker tracker(tracker_config);
  static_cast<void>(tracker.begin_new_task({1'000'000'000}));
  underwater_planner::core::ExecutedRobotSegment executed;
  executed.sequence_number = 1;
  for (std::size_t index = 0; index <= 4U; ++index) {
    const double arc_length_m = 0.25 * static_cast<double>(index);
    executed.samples.push_back(
        {arc_length_m,
         {1.0 + arc_length_m, 0.0, 0.0,
          {1'000'000'000 + static_cast<std::int64_t>(index) * 250'000'000}},
         0.5});
  }
  underwater_planner::core::TouchdownObservation observation;
  observation.touchdown_position_m = {1.0, 0.0};
  observation.position_covariance_m2 = {0.01, 0.0, 0.0, 0.01};
  observation.timestamp = {2'000'000'000};
  observation.sequence_number = 1;
  const auto tracked = tracker.update(
      executed, {0.5, 0.0, 80.0, {2'000'000'000}, 1}, observation);
  require(tracked.usable_for_planning() && tracked.state.has_value() &&
              tracked.state->laying_memory.retained_arc_length_m >= 1.0,
          "tracker did not retain its complete curvature history");
  CableLayingLimits short_support_limits = limits();
  short_support_limits.support_evaluation_length_m = 0.2;
  GeometricPath tracked_continuation;
  tracked_continuation.metadata = path.metadata;
  tracked_continuation.points = {
      {0.0, 1.0, 0.0, 0.0, 0.0},
      {1.0, 2.0, 0.0, 0.0, 0.0},
  };
  const auto integrated_result = evaluator.evaluate(
      tracked.state->laying_memory, tracked_continuation,
      state_profile(tracked_continuation), flat_terrain(), short_support_limits,
      CableHistoryBoundary::actual_laying_history);
  require(integrated_result.valid && integrated_result.hard_feasible,
          "tracker memory did not satisfy the evaluator curvature window");
}

void terrain_before_the_fixed_history_window_is_not_rejected() {
  CableConstraintMemory memory;
  memory.previous_distinct_touchdown_points_m = {{-1.5, 0.0}, {0.0, 0.0}};
  memory.trailing_support_samples = {
      {0.0, {-2.5, 0.0}},
      {1.0, {-1.5, 0.0}},
      {2.5, {0.0, 0.0}},
  };
  memory.retained_arc_length_m = 2.5;
  GeometricPath path;
  path.metadata = {170, "world", 4, "linear"};
  path.points = {
      {0.0, 0.0, 0.0, 0.0, 0.0},
      {1.0, 1.0, 0.0, 0.0, 0.0},
  };
  TerrainLayers terrain = flat_terrain();
  terrain.cable_laying.cells[terrain_index(terrain, -2.4, 0.0)].known =
      false;
  const auto result = CableLayingEvaluator{}.evaluate(
      memory, path, state_profile(path), terrain, limits(),
      CableHistoryBoundary::actual_laying_history);
  require(result.valid && result.hard_feasible,
          "terrain before the fixed support window rejected a candidate");
}

void explicit_task_start_records_effective_window_and_advances_memory() {
  const CableLayingEvaluator evaluator;
  GeometricPath path;
  path.metadata = {165, "world", 4, "linear"};
  path.points = {
      {0.0, 0.0, 0.0, 0.0, 0.0},
      {1.0, 1.0, 0.0, 0.0, 0.0},
  };
  const auto result = evaluator.evaluate(
      CableConstraintMemory{}, path, state_profile(path), flat_terrain(),
      limits(), CableHistoryBoundary::explicit_task_start);
  require(result.valid && result.hard_feasible,
          "an explicit flat task-start segment was rejected");
  require(std::abs(result.terminal_support_window_length_m - 1.0) < 1.0e-12,
          "task-start effective support window length was not recorded");
  require(result.terminal_memory.previous_distinct_touchdown_points_m.size() ==
              2U &&
              result.terminal_memory.trailing_support_samples.size() >= 2U &&
              result.terminal_memory.canonical_signature != 0U,
          "feasible evaluation did not advance normalized mechanical memory");

  GeometricPath continuation;
  continuation.metadata = {167, "world", 4, "linear"};
  continuation.points = {
      {0.0, 1.0, 0.0, 0.0, 0.0},
      {1.0, 2.0, 0.0, 0.0, 0.0},
  };
  const auto continued = evaluator.evaluate(
      result.terminal_memory, continuation, state_profile(continuation),
      flat_terrain(), limits(), CableHistoryBoundary::explicit_task_start);
  require(continued.valid && continued.hard_feasible &&
              std::abs(continued.terminal_support_window_length_m - 2.0) <
                  1.0e-12,
          "incremental task-start evaluation forgot its partial window");
}

void terminal_memory_uses_touchdown_arc_length_on_curves() {
  const GeometricPath coarse = arc_path(0.3);
  const auto coarse_result = CableLayingEvaluator{}.evaluate(
      CableConstraintMemory{}, coarse, state_profile(coarse), flat_terrain(),
      limits(), CableHistoryBoundary::explicit_task_start);
  require(coarse_result.valid && coarse_result.hard_feasible &&
              std::abs(coarse_result.terminal_memory.retained_arc_length_m -
                       coarse.points.back().arc_length_m) <
                  1.0e-12,
          "terminal memory replaced curved touchdown arc length with chords");
}

void actual_history_participates_in_first_candidate_curvature() {
  // Design: 18.2.4-26
  const CableLayingEvaluator evaluator;
  GeometricPath path;
  path.metadata = {166, "world", 4, "linear"};
  path.points = {
      {0.0, 0.0, 0.0, 0.0, 0.0},
      {std::sqrt(0.5), 0.5, 0.5, 0.7853981633974483, 0.0},
  };
  const auto result = evaluator.evaluate(
      complete_history(), path, state_profile(path), flat_terrain(), limits(),
      CableHistoryBoundary::actual_laying_history);
  require(result.valid && !result.hard_feasible &&
              has_failure(result, CableLayingFailure::curvature_exceeded),
          "candidate curvature ignored the actual laid-cable boundary");
  require(result.failure_segments.size() == 1U &&
              result.failure_segments.front().start_arc_length_m == 0.0 &&
              result.failure_segments.front().end_arc_length_m > 0.0,
          "boundary curvature was not assigned to the first candidate span");

  CableLayingLimits preferred_only = limits();
  preferred_only.maximum_curvature_per_m = 2.0;
  const auto soft_result = evaluator.evaluate(
      complete_history(), path, state_profile(path), flat_terrain(),
      preferred_only, CableHistoryBoundary::actual_laying_history);
  require(soft_result.valid && soft_result.hard_feasible &&
              soft_result.soft_cost > 0.0,
          "preferred boundary curvature did not contribute a soft cost");

  GeometricPath whole;
  whole.metadata = {172, "world", 4, "linear"};
  whole.points = {
      {0.0, 0.0, 0.0, 0.0, 0.0},
      {0.25, 0.25, 0.0, 0.0, 0.0},
      {0.5, 0.5, 0.0, 0.0, 0.0},
      {0.75, 0.75, 0.0, 0.0, 0.0},
      {1.0, 1.0, 0.0, 0.0, 0.0},
      {1.1, 1.0, 0.1, 0.0, 0.0},
      {2.0, 1.0, 1.0, 0.0, 0.0},
  };
  GeometricPath first_segment;
  first_segment.metadata = whole.metadata;
  first_segment.points.assign(whole.points.begin(), whole.points.end() - 1);
  GeometricPath second_segment;
  second_segment.metadata = whole.metadata;
  second_segment.points = {
      {0.0, 1.0, 0.1, 0.0, 0.0},
      {0.9, 1.0, 1.0, 0.0, 0.0},
  };
  CableLayingLimits additive_limits = limits();
  additive_limits.maximum_curvature_per_m = 3.5;
  additive_limits.support_evaluation_length_m = 0.2;
  const auto whole_result = evaluator.evaluate(
      CableConstraintMemory{}, whole, state_profile(whole), flat_terrain(),
      additive_limits, CableHistoryBoundary::explicit_task_start);
  const auto first_result = evaluator.evaluate_segment(
      CableConstraintMemory{}, first_segment, state_profile(first_segment),
      flat_terrain(), additive_limits,
      CableHistoryBoundary::explicit_task_start);
  const auto second_result = evaluator.evaluate_segment(
      first_result.terminal_memory, second_segment,
      state_profile(second_segment), flat_terrain(), additive_limits,
      CableHistoryBoundary::explicit_task_start);
  require(whole_result.valid && whole_result.hard_feasible &&
              whole_result.soft_cost > 0.0 &&
              first_result.valid && first_result.hard_feasible &&
              second_result.valid && second_result.hard_feasible &&
              std::abs(whole_result.soft_cost - first_result.soft_cost -
                       second_result.soft_cost) < 1.0e-12,
          "curvature soft cost changed across an incremental boundary: whole=" +
              std::to_string(whole_result.soft_cost) +
              " first=" + std::to_string(first_result.soft_cost) +
              " second=" + std::to_string(second_result.soft_cost));
}

void soft_cost_only_ranks_hard_feasible_paths() {
  const CableLayingEvaluator evaluator;
  const GeometricPath bend = arc_path(0.3);
  const TerrainLayers terrain = flat_terrain();
  const auto weighted = evaluator.evaluate(
      CableConstraintMemory{}, bend, state_profile(bend), terrain, limits(),
      CableHistoryBoundary::explicit_task_start);
  require(weighted.valid && weighted.hard_feasible && weighted.soft_cost > 0.0,
          "preferred-curvature excess did not contribute a soft cost");

  CableLayingLimits zero_bend = limits();
  zero_bend.bend_weight = 0.0;
  const auto unweighted = evaluator.evaluate(
      CableConstraintMemory{}, bend, state_profile(bend), terrain, zero_bend,
      CableHistoryBoundary::explicit_task_start);
  require(unweighted.valid && unweighted.hard_feasible &&
              unweighted.soft_cost == 0.0,
          "zero bend weight changed feasibility or retained a bend cost");

  TerrainLayers medium = flat_terrain();
  for (double x_m = 1.0; x_m < 1.5;
       x_m += medium.surface.resolution_m) {
    medium.cable_laying.cells[terrain_index(medium, x_m, 0.0)]
        .elevation_m = 0.2;
  }
  const GeometricPath straight = straight_path();
  const auto medium_result = evaluator.evaluate(
      CableConstraintMemory{}, straight, state_profile(straight), medium,
      limits(), CableHistoryBoundary::explicit_task_start);
  require(medium_result.valid && medium_result.hard_feasible &&
              medium_result.soft_cost > 0.0,
          "medium support-proxy risk was not a soft ranking cost");

  TerrainLayers rough = flat_terrain();
  for (auto& cable_cell : rough.cable_laying.cells) {
    cable_cell.roughness_m = 0.1;
  }
  const auto rough_result = evaluator.evaluate(
      CableConstraintMemory{}, straight, state_profile(straight), rough,
      limits(), CableHistoryBoundary::explicit_task_start);
  require(rough_result.valid && rough_result.hard_feasible &&
              rough_result.soft_cost > 0.0,
          "terrain roughness did not contribute a soft ranking cost");
}

void zero_soft_weights_cannot_bypass_mechanical_hard_limits() {
  // Design: 18.2.4-21
  CableLayingLimits zero_weights = limits();
  zero_weights.bend_weight = 0.0;
  zero_weights.terrain_risk_weight = 0.0;
  zero_weights.roughness_weight = 0.0;

  const GeometricPath excessive_bend = arc_path(1.0);
  const auto bend_result = CableLayingEvaluator{}.evaluate(
      CableConstraintMemory{}, excessive_bend, state_profile(excessive_bend),
      flat_terrain(), zero_weights,
      CableHistoryBoundary::explicit_task_start);
  require(bend_result.valid && !bend_result.hard_feasible &&
              has_failure(bend_result,
                          CableLayingFailure::curvature_exceeded),
          "zero bend weight bypassed the maximum cable curvature");

  TerrainLayers unsupported = flat_terrain();
  for (double x_m = 1.0; x_m < 1.5;
       x_m += unsupported.surface.resolution_m) {
    unsupported.cable_laying.cells[terrain_index(unsupported, x_m, 0.0)]
        .elevation_m = 2.0;
  }
  const GeometricPath straight = straight_path();
  const auto support_result = CableLayingEvaluator{}.evaluate(
      CableConstraintMemory{}, straight, state_profile(straight), unsupported,
      zero_weights, CableHistoryBoundary::explicit_task_start);
  require(support_result.valid && !support_result.hard_feasible &&
              has_failure(support_result,
                          CableLayingFailure::support_proxy_exceeded),
          "zero terrain-risk weight bypassed the support proxy hard limit");
}

void future_equivalence_compares_memory_not_only_hashes() {
  // Design: 18.2.3-12
  const CableLayingEvaluator evaluator;
  CableConstraintMemory left = complete_history();
  CableConstraintMemory right = left;
  right.canonical_signature = left.canonical_signature + 1U;
  require(evaluator.future_equivalent(left, right),
          "identical normalized memories depended on their cached hash");

  right = left;
  right.trailing_support_samples[1].touchdown_position_m.y_m = 0.01;
  right.canonical_signature = left.canonical_signature;
  require(!evaluator.future_equivalent(left, right),
          "a matching hash merged mechanically different histories");
}

void canonical_memory_recomputes_signatures_and_signed_zero() {
  const CableLayingEvaluator evaluator;
  CableConstraintMemory positive = complete_history();
  positive.canonical_signature = 1U;
  CableConstraintMemory negative = positive;
  negative.previous_distinct_touchdown_points_m.back().y_m = -0.0;
  negative.trailing_support_samples.back().touchdown_position_m.y_m = -0.0;
  negative.canonical_signature = 2U;

  const auto canonical_positive =
      evaluator.canonicalize_memory(
          positive, limits(), CableHistoryBoundary::explicit_task_start);
  const auto canonical_negative =
      evaluator.canonicalize_memory(
          negative, limits(), CableHistoryBoundary::explicit_task_start);
  require(canonical_positive.has_value() && canonical_negative.has_value() &&
              canonical_positive->canonical_signature ==
                  canonical_negative->canonical_signature &&
              evaluator.future_equivalent(*canonical_positive,
                                          *canonical_negative),
          "canonical memory retained stale hashes or signed-zero differences");
}

void canonical_memory_clips_a_sparse_boundary_segment_to_the_fixed_window() {
  const CableLayingEvaluator evaluator;
  CableConstraintMemory sparse;
  sparse.trailing_support_samples = {
      {0.0, {0.0, 0.0}}, {100.0, {100.0, 0.0}}};
  sparse.retained_arc_length_m = 100.0;

  const auto canonical = evaluator.canonicalize_memory(
      sparse, limits(), CableHistoryBoundary::explicit_task_start);
  require(canonical.has_value() &&
              canonical->trailing_support_samples.size() == 2U &&
              canonical->retained_arc_length_m == 2.0 &&
              canonical->trailing_support_samples.front()
                      .touchdown_arc_length_m == 98.0 &&
              canonical->trailing_support_samples.front()
                      .touchdown_position_m.x_m == 98.0 &&
              canonical->trailing_support_samples.front()
                      .touchdown_position_m.y_m == 0.0,
          "canonical memory retained an unbounded sparse prefix instead of "
          "an exactly interpolated window boundary");
}

void evaluation_is_field_deterministic() {
  const CableLayingEvaluator evaluator;
  const GeometricPath path = sampled_straight_path(0.2);
  const auto profile = state_profile(path);
  const TerrainLayers terrain = flat_terrain();
  const auto first = evaluator.evaluate(
      CableConstraintMemory{}, path, profile, terrain, limits(),
      CableHistoryBoundary::explicit_task_start);
  const auto second = evaluator.evaluate(
      CableConstraintMemory{}, path, profile, terrain, limits(),
      CableHistoryBoundary::explicit_task_start);
  require(first.valid == second.valid &&
              first.hard_feasible == second.hard_feasible &&
              first.failure_reasons == second.failure_reasons &&
              first.maximum_absolute_curvature_per_m ==
                  second.maximum_absolute_curvature_per_m &&
              first.maximum_support_proxy_range_m ==
                  second.maximum_support_proxy_range_m &&
              first.terminal_support_window_length_m ==
                  second.terminal_support_window_length_m &&
              first.soft_cost == second.soft_cost &&
              first.limits_version == second.limits_version &&
              first.terrain_map_sequence == second.terrain_map_sequence &&
              first.terrain_analysis_config_version ==
                  second.terrain_analysis_config_version &&
              first.operating_domain_id == second.operating_domain_id &&
              first.risk_semantics == second.risk_semantics &&
              evaluator.future_equivalent(first.terminal_memory,
                                          second.terminal_memory),
          "identical cable laying inputs produced different results");
}

}  // namespace

int main() {
  constexpr std::uint64_t kSeed = 1616;
  try {
    left_and_right_curvature_share_the_hard_limit();
    swept_forbidden_unknown_and_low_confidence_cells_fail_hard();
    coordinate_frame_and_arc_length_mismatches_fail_closed();
    categorical_terrain_uses_conservative_grid_supercover();
    repeated_touchdown_points_are_explicitly_rejected();
    support_proxy_is_physical_window_and_sampling_invariant();
    curvature_is_invariant_to_collinear_input_resampling();
    curvature_normalization_remains_conservative_at_corners();
    actual_history_is_required_and_requeried_on_the_current_map();
    terrain_before_the_fixed_history_window_is_not_rejected();
    explicit_task_start_records_effective_window_and_advances_memory();
    terminal_memory_uses_touchdown_arc_length_on_curves();
    actual_history_participates_in_first_candidate_curvature();
    soft_cost_only_ranks_hard_feasible_paths();
    zero_soft_weights_cannot_bypass_mechanical_hard_limits();
    future_equivalence_compares_memory_not_only_hashes();
    canonical_memory_recomputes_signatures_and_signed_zero();
    canonical_memory_clips_a_sparse_boundary_segment_to_the_fixed_window();
    evaluation_is_field_deterministic();
    std::cout << "cable laying evaluator checks passed: 19"
              << " seed=" << kSeed
              << " input_version=t16-cable-laying/v1"
              << " units=SI timestamp=monotonic-ns"
              << " domain=competition-seabed-v1"
              << " risk=conservative-support-proxy-not-flexible-dynamics\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "cable laying evaluator failure: " << error.what()
              << " seed=" << kSeed
              << " input_version=t16-cable-laying/v1"
              << " units=SI timestamp=monotonic-ns"
              << " domain=competition-seabed-v1"
              << " risk=conservative-support-proxy-not-flexible-dynamics\n";
    return 1;
  }
}
