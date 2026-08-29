#include "underwater_planner/core/traversability_evaluator.hpp"

#include "terrain_test_config.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void require(const bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "T10 failure: " << message << '\n';
    std::exit(1);
  }
}

underwater_planner::core::TerrainLayers make_step_terrain() {
  using namespace underwater_planner::core;
  TerrainLayers terrain;
  terrain.source_map_version =
      {"step-traversability-map", 10U, MonotonicTime{10'000'000'000}, "map"};
  terrain.analysis_config_version = 17U;
  terrain.operating_domain_id = "terrain-domain";
  terrain.surface.width = 16U;
  terrain.surface.height = 16U;
  terrain.surface.resolution_m = 0.25;
  terrain.surface.origin_x_m = 0.0;
  terrain.surface.origin_y_m = 0.0;
  terrain.surface.cells.assign(256U, SurfaceEstimate{});
  for (std::size_t row = 0; row < terrain.surface.height; ++row) {
    for (std::size_t column = 0; column < terrain.surface.width; ++column) {
      SurfaceEstimate& estimate =
          terrain.surface.cells.at(row * terrain.surface.width + column);
      estimate.elevation_m = column >= 6U ? 0.2 : 0.0;
      estimate.support_ratio = 1.0;
      estimate.status = TerrainEstimateStatus::valid;
    }
  }
  terrain.steps.estimates.push_back(
      {{{{1.5, 0.5}, {1.5, 3.5}}, {1.0, 0.0}, 0.2, 0.1, 0.99},
       StepEstimateStatus::valid});
  return terrain;
}

underwater_planner::core::MapSnapshot make_analyzed_step_map() {
  using namespace underwater_planner::core;
  MapSnapshot map;
  map.version =
      {"analyzed-step-map", 10U, MonotonicTime{10'000'000'000}, "map"};
  map.width = 41U;
  map.height = 41U;
  map.resolution_m = 0.1;
  map.origin_x_m = -2.0;
  map.origin_y_m = -2.0;
  map.derived_configuration_version = 17U;
  for (std::size_t row = 0; row < map.height; ++row) {
    for (std::size_t column = 0; column < map.width; ++column) {
      const double x_m =
          map.origin_x_m + static_cast<double>(column) * map.resolution_m;
      map.cells.push_back(
          {x_m >= 0.0 ? 0.2 : 0.0, 1.0e-4, 1.0, true,
           MonotonicTime{9'000'000'000}});
    }
  }
  return map;
}

underwater_planner::core::TerrainAnalysisConfig make_analysis_config() {
  return underwater_planner::testing::make_terrain_analysis_config(
      17U, "terrain-domain", 0.7, 0.7);
}

underwater_planner::core::TerrainGradientRiskPolicy make_policy() {
  using namespace underwater_planner::core;
  return {5U, 17U, 0.05, 2.0, GradientCoverageModel::empirical_bounded,
          "terrain-gradient-independent-v5", "terrain-domain", true};
}

underwater_planner::core::TrackFootprint make_track_geometry() {
  using namespace underwater_planner::core;
  TrackFootprint geometry;
  geometry.polygon =
      {{-0.4, -0.4}, {0.4, -0.4}, {0.4, 0.4}, {-0.4, 0.4}};
  geometry.left_support_polygon =
      {{-0.35, 0.12}, {0.35, 0.12}, {0.35, 0.35}, {-0.35, 0.35}};
  geometry.right_support_polygon =
      {{-0.35, -0.35}, {0.35, -0.35}, {0.35, -0.12}, {-0.35, -0.12}};
  return geometry;
}

underwater_planner::core::RobotCapability make_capability() {
  using namespace underwater_planner::core;
  RobotCapability capability;
  capability.maximum_slope_up_rad = 1.0;
  capability.maximum_slope_down_rad = 1.0;
  capability.maximum_slope_lateral_rad = 1.0;
  capability.maximum_support_roll_rad = 1.0;
  capability.maximum_step_climb_m = 0.19;
  capability.maximum_step_drop_m = 0.19;
  capability.minimum_track_support_ratio = 0.75;
  capability.effective_track_spacing_m = 0.5;
  capability.minimum_step_crossing_alignment = 0.2;
  capability.step_alignment_transition_band = 0.1;
  capability.maximum_roughness_m = 1.0;
  return capability;
}

void complete_step_height_is_not_scaled_by_crossing_angle_or_direction() {
  // Design: 18.2.2-6
  // Design: 18.2.2-invariant-4
  using namespace underwater_planner::core;
  const TerrainLayers terrain = make_step_terrain();
  const TrackFootprint geometry = make_track_geometry();
  const RobotCapability capability = make_capability();
  const TraversabilityEvaluator evaluator(capability, geometry);

  const MotionSegment orthogonal{
      {Pose2d{0.75, 2.0, 0.0, MonotonicTime{10'000'000'000}},
       Pose2d{2.25, 2.0, 0.0, MonotonicTime{11'000'000'000}}}};
  const TraversabilityResult climb =
      evaluator.evaluate(orthogonal, terrain, make_policy());
  require(climb.validity == TraversabilityEvaluationValidity::valid &&
              !climb.traversable &&
              climb.step_crossing_type == StepCrossingType::climb &&
              climb.maximum_complete_step_height_m == 0.2 &&
              climb.limiting_factors.front() ==
                  TraversabilityLimitingFactor::step_climb_exceeded,
          "orthogonal climb did not use the complete height and climb limit");

  const double quarter_pi = 0.25 * std::acos(-1.0);
  const MotionSegment diagonal{
      {Pose2d{0.75, 1.0, quarter_pi, MonotonicTime{10'000'000'000}},
       Pose2d{2.25, 2.5, quarter_pi, MonotonicTime{11'000'000'000}}}};
  const TraversabilityResult diagonal_climb =
      evaluator.evaluate(diagonal, terrain, make_policy());
  require(diagonal_climb.validity == TraversabilityEvaluationValidity::valid &&
              !diagonal_climb.traversable &&
              diagonal_climb.step_crossing_type == StepCrossingType::climb &&
              diagonal_climb.maximum_complete_step_height_m == 0.2 &&
              diagonal_climb.limiting_factors.front() ==
                  TraversabilityLimitingFactor::step_climb_exceeded,
          "45 degree climb scaled or misclassified the complete step height");

  const MotionSegment reverse{
      {Pose2d{2.25, 2.0, std::acos(-1.0), MonotonicTime{10'000'000'000}},
       Pose2d{0.75, 2.0, std::acos(-1.0), MonotonicTime{11'000'000'000}}}};
  const TraversabilityResult drop =
      evaluator.evaluate(reverse, terrain, make_policy());
  require(drop.validity == TraversabilityEvaluationValidity::valid &&
              !drop.traversable &&
              drop.step_crossing_type == StepCrossingType::drop &&
              drop.maximum_complete_step_height_m == 0.2 &&
              drop.limiting_factors.front() ==
                  TraversabilityLimitingFactor::step_drop_exceeded,
          "reverse crossing did not use the independent drop limit");
}

bool has_factor(
    const underwater_planner::core::TraversabilityResult& result,
    const underwater_planner::core::TraversabilityLimitingFactor factor) {
  for (const auto actual : result.limiting_factors) {
    if (actual == factor) return true;
  }
  return false;
}

bool same_step_crossing_events(
    const std::vector<underwater_planner::core::StepCrossingEvent>& left,
    const std::vector<underwater_planner::core::StepCrossingEvent>& right) {
  using underwater_planner::core::StepCrossingEvent;
  return left.size() == right.size() &&
         std::equal(
             left.begin(), left.end(), right.begin(),
             [](const StepCrossingEvent& left_event,
                const StepCrossingEvent& right_event) {
               return left_event.type == right_event.type &&
                      left_event.direction == right_event.direction &&
                      left_event.complete_height_m ==
                          right_event.complete_height_m &&
                      left_event.contact_pose.x_m ==
                          right_event.contact_pose.x_m &&
                      left_event.contact_pose.y_m ==
                          right_event.contact_pose.y_m &&
                      left_event.contact_pose.heading_rad ==
                          right_event.contact_pose.heading_rad &&
                      left_event.contact_pose.timestamp.nanoseconds ==
                          right_event.contact_pose.timestamp.nanoseconds;
             });
}

void independent_track_support_reports_roll_coverage_and_local_drop() {
  // Design: 18.2.2-8
  // Design: 18.2.2-9
  // Design: 18.2.2-invariant-6
  using namespace underwater_planner::core;
  TerrainLayers terrain = make_step_terrain();
  RobotCapability capability = make_capability();
  capability.maximum_step_climb_m = 0.3;
  capability.maximum_step_drop_m = 0.3;
  capability.maximum_support_roll_rad = 0.3;
  const TraversabilityEvaluator evaluator(capability, make_track_geometry());
  const double half_pi = 0.5 * std::acos(-1.0);
  const MotionSegment riding{{Pose2d{
      1.5, 2.0, half_pi, MonotonicTime{10'000'000'000}}}};

  const TraversabilityResult roll =
      evaluator.evaluate(riding, terrain, make_policy());
  require(roll.validity == TraversabilityEvaluationValidity::valid &&
              !roll.traversable &&
              roll.step_crossing_type == StepCrossingType::edge_riding &&
              std::abs(roll.maximum_absolute_support_roll_rad -
                       std::atan(0.2 / 0.5)) < 1.0e-12 &&
              has_factor(roll,
                         TraversabilityLimitingFactor::support_roll_exceeded),
          "independent track elevations did not expose the riding roll limit");

  capability.maximum_support_roll_rad = 1.0;
  capability.maximum_step_drop_m = 0.15;
  const MotionSegment straddling{{Pose2d{
      1.5, 2.0, 0.0, MonotonicTime{10'000'000'000}}}};
  const TraversabilityResult local_drop =
      TraversabilityEvaluator(capability, make_track_geometry())
          .evaluate(straddling, terrain, make_policy());
  require(!local_drop.traversable &&
              local_drop.maximum_local_track_drop_m == 0.2 &&
              has_factor(
                  local_drop,
                  TraversabilityLimitingFactor::local_track_drop_exceeded),
          "a local track drop hidden by robust location was accepted");

  terrain = make_step_terrain();
  for (std::size_t row = 6U; row <= 9U; ++row) {
    for (std::size_t column = 6U; column <= 8U; ++column) {
      terrain.surface.cells.at(row * terrain.surface.width + column)
          .support_ratio = 0.1;
    }
  }
  capability.maximum_step_drop_m = 0.3;
  const TraversabilityResult missing =
      TraversabilityEvaluator(capability, make_track_geometry())
          .evaluate(riding, terrain, make_policy());
  require(!missing.traversable &&
              (missing.minimum_left_track_support_ratio <
                   capability.minimum_track_support_ratio ||
               missing.minimum_right_track_support_ratio <
                   capability.minimum_track_support_ratio) &&
              (has_factor(
                   missing,
                   TraversabilityLimitingFactor::left_track_support_insufficient) ||
               has_factor(
                   missing,
                   TraversabilityLimitingFactor::right_track_support_insufficient)),
          "single-track support loss was hidden by the combined footprint");
}

void riding_transition_nearby_and_invalid_geometry_are_distinct() {
  // Design: 18.2.2-10
  using namespace underwater_planner::core;
  const TerrainLayers terrain = make_step_terrain();
  RobotCapability capability = make_capability();
  capability.maximum_support_roll_rad = 1.0;
  const TraversabilityEvaluator evaluator(capability, make_track_geometry());

  const double half_pi = 0.5 * std::acos(-1.0);
  const MotionSegment riding{{Pose2d{
      1.5, 2.0, half_pi, MonotonicTime{10'000'000'000}}}};
  const TraversabilityResult edge =
      evaluator.evaluate(riding, terrain, make_policy());
  require(edge.validity == TraversabilityEvaluationValidity::valid &&
              !edge.traversable &&
              edge.step_crossing_type == StepCrossingType::edge_riding &&
              edge.maximum_complete_step_height_m == 0.2 &&
              has_factor(
                  edge,
                  TraversabilityLimitingFactor::step_transition_height_exceeded),
          "edge riding did not combine complete-height and support checks");

  const double transition_heading = std::acos(0.2);
  const MotionSegment transition{
      {Pose2d{0.75, 1.0, transition_heading,
              MonotonicTime{10'000'000'000}},
       Pose2d{2.25, 2.5, transition_heading,
              MonotonicTime{11'000'000'000}}}};
  const TraversabilityResult boundary =
      evaluator.evaluate(transition, terrain, make_policy());
  require(boundary.validity == TraversabilityEvaluationValidity::valid &&
              !boundary.traversable &&
              boundary.step_crossing_type == StepCrossingType::transition &&
              boundary.maximum_complete_step_height_m == 0.2 &&
              has_factor(
                  boundary,
                  TraversabilityLimitingFactor::step_climb_exceeded),
          "the approach-alignment transition band did not apply both checks");

  capability.maximum_step_climb_m = 0.3;
  capability.maximum_step_drop_m = 0.1;
  const TraversabilityResult asymmetric_transition =
      TraversabilityEvaluator(capability, make_track_geometry())
          .evaluate(transition, terrain, make_policy());
  require(asymmetric_transition.step_crossing_type ==
                  StepCrossingType::transition &&
              !has_factor(
                  asymmetric_transition,
                  TraversabilityLimitingFactor::step_climb_exceeded) &&
              !has_factor(
                  asymmetric_transition,
                  TraversabilityLimitingFactor::step_drop_exceeded) &&
              !has_factor(
                  asymmetric_transition,
                  TraversabilityLimitingFactor::step_transition_height_exceeded),
          "a transition-band climb incorrectly used the independent drop limit");

  capability.maximum_step_climb_m = 0.3;
  capability.maximum_step_drop_m = 0.3;
  const MotionSegment nearby{
      {Pose2d{0.75, 1.0, half_pi, MonotonicTime{10'000'000'000}},
       Pose2d{0.75, 3.0, half_pi, MonotonicTime{11'000'000'000}}}};
  const TraversabilityResult not_intersected =
      TraversabilityEvaluator(capability, make_track_geometry())
          .evaluate(nearby, terrain, make_policy());
  require(not_intersected.validity == TraversabilityEvaluationValidity::valid &&
              not_intersected.traversable &&
              not_intersected.step_crossing_type == StepCrossingType::none &&
              not_intersected.maximum_complete_step_height_m == 0.0,
          "a nearby non-intersected step was classified as a crossing");

  TerrainLayers malformed = terrain;
  malformed.steps.estimates.front().edge.extent.clear();
  const TraversabilityResult invalid =
      evaluator.evaluate(riding, malformed, make_policy());
  require(invalid.validity == TraversabilityEvaluationValidity::input_invalid &&
              !invalid.traversable && invalid.evaluated_sweep_poses == 0U,
          "malformed valid step geometry entered swept evaluation");

  const TraversabilityResult repeated =
      evaluator.evaluate(transition, terrain, make_policy());
  require(repeated.validity == boundary.validity &&
              repeated.traversable == boundary.traversable &&
              repeated.limiting_factors == boundary.limiting_factors &&
              repeated.step_crossing_type == boundary.step_crossing_type &&
              same_step_crossing_events(repeated.step_crossing_events,
                                        boundary.step_crossing_events) &&
              repeated.maximum_complete_step_height_m ==
                  boundary.maximum_complete_step_height_m &&
              repeated.maximum_absolute_support_roll_rad ==
                  boundary.maximum_absolute_support_roll_rad &&
              repeated.minimum_left_track_support_ratio ==
                  boundary.minimum_left_track_support_ratio &&
              repeated.minimum_right_track_support_ratio ==
                  boundary.minimum_right_track_support_ratio &&
              repeated.maximum_local_track_drop_m ==
                  boundary.maximum_local_track_drop_m &&
              repeated.track_elevation_outlier_detected ==
                  boundary.track_elevation_outlier_detected,
          "repeated step evaluation was not field-for-field deterministic");
}

void outliers_multi_step_diagnostics_and_between_pose_sweep_are_auditable() {
  // Design: 18.2.2-invariant-5
  using namespace underwater_planner::core;
  TerrainLayers outlier_terrain = make_step_terrain();
  outlier_terrain.steps.estimates.clear();
  for (SurfaceEstimate& estimate : outlier_terrain.surface.cells) {
    estimate.elevation_m = 0.0;
  }
  outlier_terrain.surface.cells.at(8U * outlier_terrain.surface.width + 8U)
      .elevation_m = 0.1;
  RobotCapability capability = make_capability();
  capability.maximum_step_climb_m = 0.3;
  capability.maximum_step_drop_m = 0.3;
  const MotionSegment pose{{Pose2d{
      2.0, 2.0, 0.0, MonotonicTime{10'000'000'000}}}};
  const TraversabilityResult outlier =
      TraversabilityEvaluator(capability, make_track_geometry())
          .evaluate(pose, outlier_terrain, make_policy());
  require(!outlier.traversable && outlier.track_elevation_outlier_detected &&
              has_factor(
                  outlier,
                  TraversabilityLimitingFactor::track_elevation_outlier_detected) &&
              !has_factor(
                  outlier,
                  TraversabilityLimitingFactor::local_track_drop_exceeded),
          "an isolated track elevation outlier below the drop limit was accepted");

  TerrainLayers multi_step = make_step_terrain();
  multi_step.steps.estimates.push_back(
      {{{{2.0, 0.5}, {2.0, 3.5}}, {-1.0, 0.0}, 0.25, 0.1, 0.99},
       StepEstimateStatus::valid});
  capability.maximum_step_climb_m = 0.4;
  capability.maximum_step_drop_m = 0.4;
  const MotionSegment crossing{
      {Pose2d{0.75, 2.0, 0.0, MonotonicTime{10'000'000'000}},
       Pose2d{2.75, 2.0, 0.0, MonotonicTime{11'000'000'000}}}};
  const TraversabilityResult ordered =
      TraversabilityEvaluator(capability, make_track_geometry())
          .evaluate(crossing, multi_step, make_policy());
  std::reverse(multi_step.steps.estimates.begin(),
               multi_step.steps.estimates.end());
  const TraversabilityResult reversed =
      TraversabilityEvaluator(capability, make_track_geometry())
          .evaluate(crossing, multi_step, make_policy());
  require(ordered.maximum_complete_step_height_m == 0.25 &&
              ordered.step_crossing_type == StepCrossingType::drop &&
              reversed.maximum_complete_step_height_m ==
                  ordered.maximum_complete_step_height_m &&
              reversed.step_crossing_type == ordered.step_crossing_type &&
              same_step_crossing_events(reversed.step_crossing_events,
                                        ordered.step_crossing_events),
          "multi-step diagnostics changed with the step estimate order");

  TerrainLayers corner_sweep = make_step_terrain();
  const double half_sample_heading = 0.03125 * std::acos(-1.0);
  const double corner_x =
      2.0 + 0.4 * std::cos(half_sample_heading) -
      0.4 * std::sin(half_sample_heading);
  const double corner_y =
      2.0 + 0.4 * std::sin(half_sample_heading) +
      0.4 * std::cos(half_sample_heading);
  const double normal_x = std::cos(0.25 * std::acos(-1.0) +
                                   half_sample_heading);
  const double normal_y = std::sin(0.25 * std::acos(-1.0) +
                                   half_sample_heading);
  const double tangent_x = -normal_y;
  const double tangent_y = normal_x;
  corner_sweep.steps.estimates = {
      {{{{corner_x - 0.02 * tangent_x, corner_y - 0.02 * tangent_y},
         {corner_x + 0.02 * tangent_x, corner_y + 0.02 * tangent_y}},
        {normal_x, normal_y}, 0.1, 0.01, 0.99},
       StepEstimateStatus::valid}};
  const MotionSegment rotation{
      {Pose2d{2.0, 2.0, 0.0, MonotonicTime{10'000'000'000}},
       Pose2d{2.0, 2.0, 0.5 * std::acos(-1.0),
              MonotonicTime{11'000'000'000}}}};
  const TraversabilityResult swept =
      TraversabilityEvaluator(capability, make_track_geometry())
          .evaluate(rotation, corner_sweep, make_policy());
  require(swept.validity == TraversabilityEvaluationValidity::valid &&
              swept.step_crossing_type != StepCrossingType::none &&
              swept.maximum_complete_step_height_m == 0.1,
          "a footprint corner crossing between sweep poses was missed");

  corner_sweep.steps.estimates.front().edge.extent =
      {{corner_x + 0.01 * normal_x - 0.02 * tangent_x,
        corner_y + 0.01 * normal_y - 0.02 * tangent_y},
       {corner_x + 0.01 * normal_x + 0.02 * tangent_x,
        corner_y + 0.01 * normal_y + 0.02 * tangent_y}};
  const TraversabilityResult near_miss =
      TraversabilityEvaluator(capability, make_track_geometry())
          .evaluate(rotation, corner_sweep, make_policy());
  require(near_miss.validity == TraversabilityEvaluationValidity::valid &&
              near_miss.traversable &&
              near_miss.step_crossing_type == StepCrossingType::none,
          "a nearby non-intersecting edge was mistaken for swept contact");

  TerrainLayers turning_terrain = make_step_terrain();
  const MotionSegment turning_crossing{
      {Pose2d{0.75, 2.0, 0.5 * std::acos(-1.0),
              MonotonicTime{10'000'000'000}},
       Pose2d{2.25, 2.0, 0.0, MonotonicTime{11'000'000'000}}}};
  const TraversabilityResult turning =
      TraversabilityEvaluator(capability, make_track_geometry())
          .evaluate(turning_crossing, turning_terrain, make_policy());
  require(turning.validity == TraversabilityEvaluationValidity::valid &&
              turning.step_crossing_type == StepCrossingType::climb,
          "turning contact was classified with the primitive's initial heading");
}

void repeated_crossings_apply_each_directional_height_limit() {
  // Design: 18.2.2-7
  using namespace underwater_planner::core;
  const TerrainLayers terrain = make_step_terrain();
  RobotCapability capability = make_capability();
  capability.maximum_step_climb_m = 0.15;
  capability.maximum_step_drop_m = 0.25;
  const MotionSegment out_and_back{
      {Pose2d{0.75, 2.0, 0.0, MonotonicTime{10'000'000'000}},
       Pose2d{2.25, 2.0, 0.0, MonotonicTime{11'000'000'000}},
       Pose2d{0.75, 2.0, 0.0, MonotonicTime{12'000'000'000}}}};

  const TraversabilityResult climb_limited =
      TraversabilityEvaluator(capability, make_track_geometry())
          .evaluate(out_and_back, terrain, make_policy());
  require(climb_limited.step_crossing_events.size() == 2U &&
              climb_limited.step_crossing_events[0].direction ==
                  StepCrossingDirection::low_to_high &&
              climb_limited.step_crossing_events[1].direction ==
                  StepCrossingDirection::high_to_low &&
              has_factor(climb_limited,
                         TraversabilityLimitingFactor::step_climb_exceeded) &&
              !has_factor(climb_limited,
                          TraversabilityLimitingFactor::step_drop_exceeded),
          "an out-and-back primitive did not audit its climb and drop separately");

  capability.maximum_step_climb_m = 0.25;
  capability.maximum_step_drop_m = 0.15;
  const TraversabilityResult drop_limited =
      TraversabilityEvaluator(capability, make_track_geometry())
          .evaluate(out_and_back, terrain, make_policy());
  require(drop_limited.step_crossing_events.size() == 2U &&
              !has_factor(drop_limited,
                          TraversabilityLimitingFactor::step_climb_exceeded) &&
              has_factor(drop_limited,
                         TraversabilityLimitingFactor::step_drop_exceeded),
          "an out-and-back primitive did not apply the independent drop limit");
}

void analyzed_step_height_is_invariant_to_robot_test_heading() {
  // Design: 18.2.1-5
  using namespace underwater_planner::core;
  const TerrainLayers terrain =
      TerrainAnalyzer{}.analyze(make_analyzed_step_map(),
                                make_analysis_config());
  require(terrain.steps.estimates.size() == 1U &&
              terrain.steps.estimates.front().status ==
                  StepEstimateStatus::valid,
          "the analyzed step fixture did not produce one valid edge");
  const double analyzed_height_m =
      terrain.steps.estimates.front().edge.height_m;
  const TraversabilityEvaluator evaluator(make_capability(),
                                           make_track_geometry());
  const TraversabilityResult orthogonal = evaluator.evaluate(
      MotionSegment{
          {Pose2d{-0.75, -0.75, 0.0, MonotonicTime{10'000'000'000}},
           Pose2d{0.75, -0.75, 0.0, MonotonicTime{11'000'000'000}}}},
      terrain, make_policy());
  const double diagonal_heading = 0.25 * std::acos(-1.0);
  const TraversabilityResult diagonal = evaluator.evaluate(
      MotionSegment{{
          Pose2d{-0.75, -0.75, diagonal_heading,
                 MonotonicTime{10'000'000'000}},
          Pose2d{0.75, 0.75, diagonal_heading,
                 MonotonicTime{11'000'000'000}}}},
      terrain, make_policy());

  require(std::abs(analyzed_height_m - 0.2) < 1.0e-12 &&
              orthogonal.validity ==
                  TraversabilityEvaluationValidity::valid &&
              diagonal.validity == TraversabilityEvaluationValidity::valid &&
              orthogonal.step_crossing_type == StepCrossingType::climb &&
              diagonal.step_crossing_type == StepCrossingType::climb &&
              orthogonal.maximum_complete_step_height_m ==
                  analyzed_height_m &&
              diagonal.maximum_complete_step_height_m == analyzed_height_m,
          "robot test heading changed the TerrainAnalyzer complete step height");
}

void analyzer_discontinuity_band_is_handled_by_the_crossed_step() {
  using namespace underwater_planner::core;
  const TerrainLayers terrain =
      TerrainAnalyzer{}.analyze(make_analyzed_step_map(),
                                make_analysis_config());
  const MotionSegment crossing{
      {Pose2d{-0.75, 0.0, 0.0, MonotonicTime{10'000'000'000}},
       Pose2d{0.75, 0.0, 0.0, MonotonicTime{11'000'000'000}}}};
  const TraversabilityResult result =
      TraversabilityEvaluator(make_capability(), make_track_geometry())
          .evaluate(crossing, terrain, make_policy());
  require(result.validity == TraversabilityEvaluationValidity::valid &&
              !result.traversable &&
              result.step_crossing_type == StepCrossingType::climb &&
              has_factor(result,
                         TraversabilityLimitingFactor::step_climb_exceeded) &&
              result.worst_terrain_estimate_status ==
                  TerrainEstimateStatus::discontinuous &&
              result.maximum_local_track_drop_m >= 0.2 - 1.0e-12 &&
              !has_factor(
                  result,
                  TraversabilityLimitingFactor::terrain_estimate_invalid),
          "an analyzed step did not preserve side support through its discontinuity band");
}

void unexplained_discontinuity_fails_the_complete_traversability_gate() {
  // Design: 18.2.2-6, 18.2.2-invariant-4
  using namespace underwater_planner::core;
  TerrainLayers terrain =
      TerrainAnalyzer{}.analyze(make_analyzed_step_map(),
                                make_analysis_config());
  terrain.steps.estimates.clear();
  const MotionSegment crossing{
      {Pose2d{-0.75, 0.0, 0.0, MonotonicTime{10'000'000'000}},
       Pose2d{0.75, 0.0, 0.0, MonotonicTime{11'000'000'000}}}};
  const TraversabilityResult result =
      TraversabilityEvaluator(make_capability(), make_track_geometry())
          .evaluate(crossing, terrain, make_policy());
  require(result.validity ==
              TraversabilityEvaluationValidity::terrain_invalid &&
              !result.traversable &&
              has_factor(
                  result,
                  TraversabilityLimitingFactor::terrain_estimate_invalid),
          "an unexplained terrain discontinuity bypassed the complete gate");
}

}  // namespace

int main() {
  complete_step_height_is_not_scaled_by_crossing_angle_or_direction();
  independent_track_support_reports_roll_coverage_and_local_drop();
  riding_transition_nearby_and_invalid_geometry_are_distinct();
  outliers_multi_step_diagnostics_and_between_pose_sweep_are_auditable();
  repeated_crossings_apply_each_directional_height_limit();
  analyzed_step_height_is_invariant_to_robot_test_heading();
  analyzer_discontinuity_band_is_handled_by_the_crossed_step();
  unexplained_discontinuity_fails_the_complete_traversability_gate();
  std::cout << "T10 step traversability tests passed\n";
  return 0;
}
