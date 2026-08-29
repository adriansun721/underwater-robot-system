#include "underwater_planner/core/traversability_evaluator.hpp"
#include "underwater_planner/core/parameter_config.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace {

void require(const bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "T09 failure: " << message << '\n';
    std::exit(1);
  }
}

underwater_planner::core::TerrainLayers make_terrain() {
  using namespace underwater_planner::core;
  TerrainLayers terrain;
  terrain.source_map_version =
      {"slope-map", 9U, MonotonicTime{9'000'000'000}, "map"};
  terrain.analysis_config_version = 17U;
  terrain.operating_domain_id = "terrain-domain";
  terrain.surface.width = 5U;
  terrain.surface.height = 5U;
  terrain.surface.resolution_m = 1.0;
  terrain.surface.origin_x_m = 0.0;
  terrain.surface.origin_y_m = 0.0;
  terrain.surface.cells.assign(25U, SurfaceEstimate{});
  for (SurfaceEstimate& estimate : terrain.surface.cells) {
    estimate.gradient_x = 1.0;
    estimate.gradient_y = 1.0;
    estimate.status = TerrainEstimateStatus::valid;
    estimate.support_ratio = 1.0;
  }
  return terrain;
}

underwater_planner::core::TerrainGradientRiskPolicy make_policy() {
  using namespace underwater_planner::core;
  return {5U, 17U, 0.05, 2.0, GradientCoverageModel::empirical_bounded,
          "terrain-gradient-independent-v5", "terrain-domain", true};
}

underwater_planner::core::TerrainLayers make_anisotropic_terrain() {
  using namespace underwater_planner::core;
  TerrainLayers terrain = make_terrain();
  for (SurfaceEstimate& estimate : terrain.surface.cells) {
    estimate.gradient_x = 0.1;
    estimate.gradient_y = 0.0;
    estimate.gradient_covariance = {0.09, 0.0, 0.0, 0.0001};
  }
  return terrain;
}

underwater_planner::core::TrackFootprint make_footprint() {
  using namespace underwater_planner::core;
  return {{{-0.4, -0.4}, {0.4, -0.4}, {0.4, 0.4}, {-0.4, 0.4}},
          {{-0.35, 0.1}, {0.35, 0.1}, {0.35, 0.35}, {-0.35, 0.35}},
          {{-0.35, -0.35}, {0.35, -0.35}, {0.35, -0.1}, {-0.35, -0.1}}};
}

underwater_planner::core::RobotCapability make_capability(
    const double maximum_up_rad, const double maximum_down_rad,
    const double maximum_lateral_rad) {
  using namespace underwater_planner::core;
  RobotCapability capability;
  capability.maximum_slope_up_rad = maximum_up_rad;
  capability.maximum_slope_down_rad = maximum_down_rad;
  capability.maximum_slope_lateral_rad = maximum_lateral_rad;
  capability.maximum_support_roll_rad = 1.0;
  capability.maximum_step_climb_m = 0.3;
  capability.maximum_step_drop_m = 0.3;
  capability.minimum_track_support_ratio = 0.5;
  capability.effective_track_spacing_m = 0.5;
  capability.minimum_step_crossing_alignment = 0.2;
  capability.step_alignment_transition_band = 0.1;
  capability.maximum_roughness_m = 1.0;
  return capability;
}

void gradient_is_projected_before_atan_with_signed_longitudinal_limits() {
  // Design: 18.2.2-5
  using namespace underwater_planner::core;
  const TerrainLayers terrain = make_terrain();
  const TrackFootprint footprint = make_footprint();

  const TraversabilityEvaluator uphill_evaluator(
      make_capability(0.75, 1.0, 1.0), footprint);
  const TraversabilityResult uphill = uphill_evaluator.evaluate(
      MotionSegment{{Pose2d{2.5, 2.5, 0.0,
                           MonotonicTime{9'000'000'000}}}},
      terrain, make_policy());
  require(uphill.validity == TraversabilityEvaluationValidity::valid &&
              !uphill.traversable &&
              uphill.limiting_factors.size() == 1U &&
              uphill.limiting_factors.front() ==
                  TraversabilityLimitingFactor::up_slope_exceeded,
          "the forward component did not use its independent uphill limit");
  require(std::abs(uphill.maximum_longitudinal_mean_gradient - 1.0) <
                  1.0e-12 &&
              std::abs(uphill.maximum_longitudinal_upper_angle_rad -
                       std::atan(1.0)) < 1.0e-12,
          "the evaluator did not project the gradient before atan");

  const TraversabilityEvaluator downhill_evaluator(
      make_capability(1.0, 0.75, 1.0), footprint);
  const TraversabilityResult downhill = downhill_evaluator.evaluate(
      MotionSegment{{Pose2d{2.5, 2.5, std::acos(-1.0),
                           MonotonicTime{9'000'000'000}}}},
      terrain, make_policy());
  require(downhill.validity == TraversabilityEvaluationValidity::valid &&
              !downhill.traversable &&
              downhill.limiting_factors.size() == 1U &&
              downhill.limiting_factors.front() ==
                  TraversabilityLimitingFactor::down_slope_exceeded,
          "the signed reverse component did not use the downhill limit");
  require(std::abs(downhill.minimum_longitudinal_mean_gradient + 1.0) <
                  1.0e-12 &&
              std::abs(downhill.minimum_longitudinal_lower_angle_rad +
                       std::atan(1.0)) < 1.0e-12,
          "the downhill result lost the longitudinal sign");
}

void complete_footprint_rejects_a_lateral_limit_at_its_edge() {
  using namespace underwater_planner::core;
  TerrainLayers terrain = make_terrain();
  for (SurfaceEstimate& estimate : terrain.surface.cells) {
    estimate.gradient_x = 0.0;
    estimate.gradient_y = 0.0;
  }
  terrain.surface.cells.at(2U * terrain.surface.width + 3U).gradient_y = 1.0;
  const TrackFootprint wide_footprint{
      {{-0.6, -0.6}, {0.6, -0.6}, {0.6, 0.6}, {-0.6, 0.6}},
      {{-0.5, 0.1}, {0.5, 0.1}, {0.5, 0.5}, {-0.5, 0.5}},
      {{-0.5, -0.5}, {0.5, -0.5}, {0.5, -0.1}, {-0.5, -0.1}}};
  const TraversabilityEvaluator evaluator(
      make_capability(1.0, 1.0, 0.75), wide_footprint);

  const TraversabilityResult result = evaluator.evaluate(
      MotionSegment{{Pose2d{2.5, 2.5, 0.0,
                           MonotonicTime{9'000'000'000}}}},
      terrain, make_policy());

  require(result.validity == TraversabilityEvaluationValidity::valid &&
              !result.traversable &&
              result.limiting_factors.size() == 1U &&
              result.limiting_factors.front() ==
                  TraversabilityLimitingFactor::lateral_slope_exceeded,
          "a lateral limit at the footprint edge did not reject the pose");
  require(result.evaluated_footprint_samples > 1U &&
              std::abs(result.maximum_lateral_absolute_upper_angle_rad -
                       std::atan(1.0)) < 1.0e-12,
          "the result did not report the complete-footprint lateral bound");
}

void risk_policy_is_bound_to_analysis_calibration_domain_and_local_epsilon() {
  // Design: 18.2.2-14
  // Design: 18.2.2-15
  // Design: 18.2.2-16
  using namespace underwater_planner::core;
  const TerrainLayers terrain = make_terrain();
  const TraversabilityEvaluator evaluator(
      make_capability(1.2, 1.2, 1.2), make_footprint());
  const MotionSegment segment{{Pose2d{2.5, 2.5, 0.0,
                                     MonotonicTime{9'000'000'000}}}};

  const TraversabilityResult valid =
      evaluator.evaluate(segment, terrain, make_policy());
  require(valid.validity == TraversabilityEvaluationValidity::valid &&
              valid.traversable &&
              valid.risk_audit.gradient_risk_policy_version == 5U &&
              valid.risk_audit.source_terrain_analysis_config_version == 17U &&
              valid.risk_audit.policy_terrain_analysis_config_version == 17U &&
              valid.risk_audit.epsilon_local == 0.05 &&
              valid.risk_audit.coverage_model ==
                  GradientCoverageModel::empirical_bounded &&
              valid.risk_audit.calibration_dataset_id ==
                  "terrain-gradient-independent-v5" &&
              valid.risk_audit.operating_domain_id == "terrain-domain" &&
              valid.risk_audit.source_map_version == terrain.source_map_version,
          "a valid result omitted its gradient risk policy dependencies");
  require(!valid.risk_audit.path_joint_risk_implemented &&
              valid.risk_audit.risk_semantics ==
                  TerrainGradientRiskSemantics::
                      local_pointwise_only_no_path_joint_guarantee,
          "the result overstated the local gradient risk guarantee");

  TerrainGradientRiskPolicy mismatched = make_policy();
  mismatched.terrain_analysis_config_version = 18U;
  const TraversabilityResult version_mismatch =
      evaluator.evaluate(segment, terrain, mismatched);
  require(version_mismatch.validity ==
                  TraversabilityEvaluationValidity::version_mismatch &&
              !version_mismatch.traversable &&
              version_mismatch.evaluated_footprint_samples == 0U &&
              version_mismatch.risk_audit
                      .source_terrain_analysis_config_version == 17U &&
              version_mismatch.risk_audit
                      .policy_terrain_analysis_config_version == 18U,
          "an analysis-policy version mismatch entered terrain evaluation");

  TerrainGradientRiskPolicy domain_mismatch = make_policy();
  domain_mismatch.operating_domain_id = "other-terrain-domain";
  const TraversabilityResult wrong_domain =
      evaluator.evaluate(segment, terrain, domain_mismatch);
  require(wrong_domain.validity ==
                  TraversabilityEvaluationValidity::version_mismatch &&
              !wrong_domain.traversable &&
              wrong_domain.evaluated_footprint_samples == 0U,
          "an operating-domain mismatch entered terrain evaluation");

  TerrainGradientRiskPolicy uncalibrated = make_policy();
  uncalibrated.coverage_calibrated = false;
  uncalibrated.calibration_dataset_id.clear();
  const TraversabilityResult policy_invalid =
      evaluator.evaluate(segment, terrain, uncalibrated);
  require(policy_invalid.validity ==
                  TraversabilityEvaluationValidity::risk_policy_invalid &&
              !policy_invalid.traversable &&
              policy_invalid.evaluated_footprint_samples == 0U,
          "an uncalibrated risk policy entered terrain evaluation");

  TerrainGradientRiskPolicy unversioned = make_policy();
  unversioned.version = 0U;
  const TraversabilityResult version_missing =
      evaluator.evaluate(segment, terrain, unversioned);
  require(version_missing.validity ==
                  TraversabilityEvaluationValidity::risk_policy_invalid &&
              !version_missing.traversable,
          "an unversioned gradient risk policy entered terrain evaluation");

  TerrainGradientRiskPolicy dataset_missing = make_policy();
  dataset_missing.calibration_dataset_id.clear();
  const TraversabilityResult calibration_missing =
      evaluator.evaluate(segment, terrain, dataset_missing);
  require(calibration_missing.validity ==
                  TraversabilityEvaluationValidity::risk_policy_invalid &&
              !calibration_missing.traversable,
          "a policy without its calibration dataset entered terrain evaluation");

  TerrainGradientRiskPolicy inconsistent_gaussian = make_policy();
  inconsistent_gaussian.coverage_model =
      GradientCoverageModel::calibrated_gaussian;
  const TraversabilityResult gaussian_invalid =
      evaluator.evaluate(segment, terrain, inconsistent_gaussian);
  require(gaussian_invalid.validity ==
                  TraversabilityEvaluationValidity::risk_policy_invalid &&
              !gaussian_invalid.traversable,
          "a Gaussian policy accepted a multiplier inconsistent with epsilon");

  TerrainGradientRiskPolicy unknown_model = make_policy();
  unknown_model.coverage_model = static_cast<GradientCoverageModel>(99);
  const TraversabilityResult unknown_model_invalid =
      evaluator.evaluate(segment, terrain, unknown_model);
  require(unknown_model_invalid.validity ==
                  TraversabilityEvaluationValidity::risk_policy_invalid &&
              !unknown_model_invalid.traversable,
          "an unknown gradient coverage model was accepted");
}

void mean_safe_but_conservative_upper_bound_rejects_segment() {
  // Design: 18.2.2-11
  using namespace underwater_planner::core;
  const TerrainLayers terrain = make_anisotropic_terrain();
  const TraversabilityEvaluator evaluator(
      make_capability(0.5, 1.0, 1.0), make_footprint());
  const TraversabilityResult result = evaluator.evaluate(
      MotionSegment{{Pose2d{2.5, 2.5, 0.0,
                           MonotonicTime{9'000'000'000}}}},
      terrain, make_policy());

  require(result.validity == TraversabilityEvaluationValidity::valid &&
              !result.traversable &&
              result.maximum_longitudinal_mean_angle_rad < 0.5 &&
              result.maximum_longitudinal_upper_angle_rad > 0.5 &&
              result.limiting_factors.size() == 1U &&
              result.limiting_factors.front() ==
                  TraversabilityLimitingFactor::up_slope_exceeded,
          "a mean-safe slope was accepted after its conservative bound exceeded capability");
}

void anisotropic_covariance_rotates_between_directional_bounds() {
  // Design: 18.2.2-12
  using namespace underwater_planner::core;
  const TerrainLayers terrain = make_anisotropic_terrain();
  const TraversabilityEvaluator evaluator(
      make_capability(0.5, 1.0, 0.5), make_footprint());
  const TraversabilityResult heading_x = evaluator.evaluate(
      MotionSegment{{Pose2d{2.5, 2.5, 0.0,
                           MonotonicTime{9'000'000'000}}}},
      terrain, make_policy());
  const TraversabilityResult heading_y = evaluator.evaluate(
      MotionSegment{{Pose2d{2.5, 2.5, 0.5 * std::acos(-1.0),
                           MonotonicTime{9'000'000'000}}}},
      terrain, make_policy());

  require(heading_x.validity == TraversabilityEvaluationValidity::valid &&
              heading_y.validity == TraversabilityEvaluationValidity::valid &&
              heading_x.maximum_longitudinal_upper_angle_rad > 0.5 &&
              heading_x.maximum_lateral_absolute_upper_angle_rad < 0.5 &&
              heading_y.maximum_longitudinal_upper_angle_rad < 0.5 &&
              heading_y.maximum_lateral_absolute_upper_angle_rad > 0.5,
          "rotating the heading did not move anisotropic covariance between longitudinal and lateral bounds");
  require(heading_x.limiting_factors.size() == 1U &&
              heading_x.limiting_factors.front() ==
                  TraversabilityLimitingFactor::up_slope_exceeded &&
              heading_y.limiting_factors.size() == 1U &&
              heading_y.limiting_factors.front() ==
                  TraversabilityLimitingFactor::lateral_slope_exceeded,
          "directional limits did not consume their corresponding covariance projection");
}

void asymmetric_up_down_bounds_include_the_same_nonzero_variance() {
  // Design: 18.2.2-13
  using namespace underwater_planner::core;
  const TerrainLayers terrain = make_anisotropic_terrain();
  const MotionSegment uphill{{Pose2d{
      2.5, 2.5, 0.0, MonotonicTime{9'000'000'000}}}};
  const MotionSegment downhill{{Pose2d{
      2.5, 2.5, std::acos(-1.0), MonotonicTime{9'000'000'000}}}};

  const TraversabilityEvaluator uphill_limited(
      make_capability(0.55, 0.65, 1.0), make_footprint());
  const TraversabilityResult uphill_rejected =
      uphill_limited.evaluate(uphill, terrain, make_policy());
  const TraversabilityResult downhill_allowed =
      uphill_limited.evaluate(downhill, terrain, make_policy());
  require(!uphill_rejected.traversable && downhill_allowed.traversable &&
              uphill_rejected.limiting_factors.front() ==
                  TraversabilityLimitingFactor::up_slope_exceeded,
          "the uphill risk bound did not use its independent capability");

  const TraversabilityEvaluator downhill_limited(
      make_capability(0.65, 0.55, 1.0), make_footprint());
  const TraversabilityResult uphill_allowed =
      downhill_limited.evaluate(uphill, terrain, make_policy());
  const TraversabilityResult downhill_rejected =
      downhill_limited.evaluate(downhill, terrain, make_policy());
  require(uphill_allowed.traversable && !downhill_rejected.traversable &&
              downhill_rejected.limiting_factors.front() ==
                  TraversabilityLimitingFactor::down_slope_exceeded,
          "the downhill risk bound did not use its independent capability");

  require(std::abs(uphill_rejected.maximum_longitudinal_mean_gradient - 0.1) <
                  1.0e-12 &&
              std::abs(uphill_rejected.maximum_longitudinal_upper_angle_rad -
                       std::atan(0.7)) < 1.0e-12 &&
              std::abs(downhill_rejected.minimum_longitudinal_mean_gradient +
                       0.1) < 1.0e-12 &&
              std::abs(downhill_rejected.minimum_longitudinal_lower_angle_rad +
                       std::atan(0.7)) < 1.0e-12,
          "equal nonzero variance did not enter both signed longitudinal bounds");
}

void invalid_gradient_covariance_rejects_the_whole_motion_segment() {
  // Design: 18.2.1-8
  using namespace underwater_planner::core;
  TerrainLayers terrain = make_terrain();
  for (SurfaceEstimate& estimate : terrain.surface.cells) {
    estimate.gradient_x = 0.0;
    estimate.gradient_y = 0.0;
  }
  SurfaceEstimate& invalid = terrain.surface.cells.at(3U * 5U + 3U);
  invalid.gradient_covariance = {1.0, 2.0, 2.0, 1.0};
  const TraversabilityEvaluator evaluator(
      make_capability(1.0, 1.0, 1.0), make_footprint());
  const MotionSegment segment{
      {Pose2d{1.5, 1.5, 0.0, MonotonicTime{9'000'000'000}},
       Pose2d{3.5, 3.5, 0.0, MonotonicTime{9'100'000'000}}}};

  const TraversabilityResult result =
      evaluator.evaluate(segment, terrain, make_policy());

  require(result.validity ==
                  TraversabilityEvaluationValidity::covariance_invalid &&
              !result.traversable &&
              result.limiting_factors.size() == 1U &&
              result.limiting_factors.front() ==
                  TraversabilityLimitingFactor::gradient_covariance_invalid,
          "an invalid covariance in a later footprint sample was not fail-closed");

  terrain = make_terrain();
  for (SurfaceEstimate& estimate : terrain.surface.cells) {
    estimate.gradient_x = 0.0;
    estimate.gradient_y = 0.0;
  }
  SurfaceEstimate& scaled_invalid = terrain.surface.cells.at(2U * 5U + 2U);
  scaled_invalid.gradient_covariance =
      {5'000'000'000'000'000.0, 5'000'000'000'000'001.0,
       5'000'000'000'000'001.0, 5'000'000'000'000'000.0};
  const TraversabilityResult scaled = evaluator.evaluate(
      MotionSegment{{Pose2d{2.5, 2.5, 0.25 * std::acos(-1.0),
                           MonotonicTime{9'000'000'000}}}},
      terrain, make_policy());
  require(scaled.validity ==
                  TraversabilityEvaluationValidity::covariance_invalid &&
              !scaled.traversable,
          "a negative covariance eigenvalue was hidden by scale tolerance");
}

void unavailable_or_out_of_map_footprint_surface_is_rejected() {
  using namespace underwater_planner::core;
  TerrainLayers terrain = make_terrain();
  terrain.surface.cells.at(2U * 5U + 2U).status =
      TerrainEstimateStatus::insufficient_support;
  const TraversabilityEvaluator evaluator(
      make_capability(1.0, 1.0, 1.0), make_footprint());
  const TraversabilityResult unavailable = evaluator.evaluate(
      MotionSegment{{Pose2d{2.5, 2.5, 0.0,
                           MonotonicTime{9'000'000'000}}}},
      terrain, make_policy());
  require(unavailable.validity ==
                  TraversabilityEvaluationValidity::terrain_invalid &&
              !unavailable.traversable &&
              unavailable.worst_terrain_estimate_status ==
                  TerrainEstimateStatus::insufficient_support &&
              unavailable.limiting_factors.front() ==
                  TraversabilityLimitingFactor::terrain_estimate_invalid,
          "an unavailable surface inside the footprint was accepted");
  require(std::isfinite(unavailable.maximum_longitudinal_mean_gradient) &&
              std::isfinite(unavailable.minimum_longitudinal_mean_gradient) &&
              std::isfinite(unavailable.maximum_longitudinal_mean_angle_rad) &&
              std::isfinite(unavailable.minimum_longitudinal_mean_angle_rad) &&
              std::isfinite(
                  unavailable.maximum_longitudinal_upper_angle_rad) &&
              std::isfinite(
                  unavailable.minimum_longitudinal_lower_angle_rad) &&
              std::isfinite(
                  unavailable.maximum_lateral_absolute_upper_angle_rad),
          "a fail-closed terrain result exposed non-finite extrema");

  terrain = make_terrain();
  const TraversabilityResult outside = evaluator.evaluate(
      MotionSegment{{Pose2d{0.2, 2.5, 0.0,
                           MonotonicTime{9'000'000'000}}}},
      terrain, make_policy());
  require(outside.validity ==
                  TraversabilityEvaluationValidity::terrain_invalid &&
              !outside.traversable &&
              outside.limiting_factors.front() ==
                  TraversabilityLimitingFactor::footprint_outside_terrain,
          "a footprint extending beyond the terrain grid was accepted");
}

void adaptive_sweep_rejects_a_hazard_between_segment_endpoints() {
  // Design: 18.2.2-invariant-7
  using namespace underwater_planner::core;
  TerrainLayers terrain = make_terrain();
  for (SurfaceEstimate& estimate : terrain.surface.cells) {
    estimate.gradient_x = 0.0;
    estimate.gradient_y = 0.0;
  }
  terrain.surface.cells.at(2U * 5U + 2U).gradient_y = 1.0;
  const TraversabilityEvaluator evaluator(
      make_capability(1.0, 1.0, 0.75), make_footprint());
  const MotionSegment endpoints{
      {Pose2d{1.5, 2.5, 0.0, MonotonicTime{9'000'000'000}},
       Pose2d{3.5, 2.5, 0.0, MonotonicTime{10'000'000'000}}}};

  const TerrainGradientRiskPolicy policy = make_policy();
  const TraversabilityResult result =
      evaluator.evaluate(endpoints, terrain, policy);

  require(result.validity == TraversabilityEvaluationValidity::valid &&
              !result.traversable && result.evaluated_sweep_poses >= 5U &&
              result.risk_audit.gradient_risk_policy_version ==
                  policy.version &&
              result.risk_audit.source_terrain_analysis_config_version ==
                  terrain.analysis_config_version &&
              result.risk_audit.policy_terrain_analysis_config_version ==
                  policy.terrain_analysis_config_version &&
              result.limiting_factors.front() ==
                  TraversabilityLimitingFactor::lateral_slope_exceeded,
          "adaptive sweep skipped a hazard or lost its locked policy version");

  terrain = make_terrain();
  for (SurfaceEstimate& estimate : terrain.surface.cells) {
    estimate.gradient_x = 0.0;
    estimate.gradient_y = 0.0;
  }
  terrain.surface.cells.at(1U * 5U + 2U).gradient_y = 1.0;
  const TrackFootprint point_like_footprint{
      {{-0.01, -0.01}, {0.01, -0.01}, {0.01, 0.01}, {-0.01, 0.01}},
      {{-0.009, 0.002}, {0.009, 0.002}, {0.009, 0.009}, {-0.009, 0.009}},
      {{-0.009, -0.009}, {0.009, -0.009}, {0.009, -0.002},
       {-0.009, -0.002}}};
  const TraversabilityEvaluator point_like_evaluator(
      make_capability(1.0, 1.0, 0.75), point_like_footprint);
  const MotionSegment corner_crossing{
      {Pose2d{1.05, 1.05, 0.0, MonotonicTime{9'000'000'000}},
       Pose2d{2.85, 2.85, 0.0, MonotonicTime{10'000'000'000}}}};
  const TraversabilityResult gap_margin =
      point_like_evaluator.evaluate(corner_crossing, terrain, make_policy());
  require(gap_margin.validity == TraversabilityEvaluationValidity::valid &&
              !gap_margin.traversable &&
              gap_margin.slope_sweep_discretization_margin_m > 0.0 &&
              gap_margin.slope_sweep_discretization_margin_m <= 0.25 &&
              gap_margin.limiting_factors.front() ==
                  TraversabilityLimitingFactor::lateral_slope_exceeded,
          "the slope-specific sweep margin missed a between-pose corner cell");

  terrain = make_terrain();
  for (SurfaceEstimate& estimate : terrain.surface.cells) {
    estimate.gradient_x = 0.0;
    estimate.gradient_y = 0.0;
  }
  const TraversabilityResult static_near_boundary = evaluator.evaluate(
      MotionSegment{{Pose2d{0.41, 2.5, 0.0,
                           MonotonicTime{9'000'000'000}}}},
      terrain, make_policy());
  require(static_near_boundary.validity ==
                  TraversabilityEvaluationValidity::valid &&
              static_near_boundary.traversable &&
              static_near_boundary.slope_sweep_discretization_margin_m == 0.0,
          "a pose-only query received a nonzero inter-pose sweep margin");
}

void invalid_public_inputs_are_rejected_with_finite_outputs() {
  using namespace underwater_planner::core;
  const TerrainLayers terrain = make_terrain();
  const TraversabilityEvaluator evaluator(
      make_capability(1.0, 1.0, 1.0), make_footprint());
  const TraversabilityResult empty =
      evaluator.evaluate(MotionSegment{}, terrain, make_policy());
  require(empty.validity == TraversabilityEvaluationValidity::input_invalid &&
              !empty.traversable &&
              std::isfinite(empty.maximum_longitudinal_mean_gradient) &&
              std::isfinite(empty.minimum_longitudinal_mean_gradient) &&
              std::isfinite(empty.maximum_longitudinal_upper_angle_rad) &&
              std::isfinite(empty.minimum_longitudinal_lower_angle_rad) &&
              std::isfinite(empty.maximum_lateral_absolute_upper_angle_rad),
          "an empty segment returned a valid or non-finite result");

  const MotionSegment valid_segment{{Pose2d{
      2.5, 2.5, 0.0, MonotonicTime{9'000'000'000}}}};
  RobotCapability invalid_robot_capability = make_capability(1.0, 1.0, 1.0);
  invalid_robot_capability.maximum_slope_up_rad = 0.0;
  const TraversabilityResult invalid_capability =
      TraversabilityEvaluator(invalid_robot_capability, make_footprint())
          .evaluate(valid_segment, terrain, make_policy());
  require(invalid_capability.validity ==
                  TraversabilityEvaluationValidity::input_invalid &&
              !invalid_capability.traversable,
          "an invalid robot capability was accepted");

  TrackFootprint invalid_geometry = make_footprint();
  invalid_geometry.polygon = {{0.0, 0.0}, {1.0, 0.0}};
  const TraversabilityResult invalid_footprint =
      TraversabilityEvaluator(make_capability(1.0, 1.0, 1.0),
                              invalid_geometry)
          .evaluate(valid_segment, terrain, make_policy());
  require(invalid_footprint.validity ==
                  TraversabilityEvaluationValidity::input_invalid &&
              !invalid_footprint.traversable,
          "a degenerate track footprint was accepted");

  MotionSegment non_finite = valid_segment;
  non_finite.samples.front().heading_rad =
      std::numeric_limits<double>::quiet_NaN();
  const TraversabilityResult invalid_pose =
      evaluator.evaluate(non_finite, terrain, make_policy());
  require(invalid_pose.validity ==
                  TraversabilityEvaluationValidity::input_invalid &&
              !invalid_pose.traversable,
          "a non-finite motion sample was accepted");

  TerrainLayers unversioned_terrain = terrain;
  unversioned_terrain.source_map_version.sequence_number = 0U;
  const TraversabilityResult invalid_map_version =
      evaluator.evaluate(valid_segment, unversioned_terrain, make_policy());
  require(invalid_map_version.validity ==
                  TraversabilityEvaluationValidity::input_invalid &&
              !invalid_map_version.traversable,
          "an unversioned terrain snapshot was accepted");
}

void covariance_projection_and_repeated_results_are_auditable() {
  using namespace underwater_planner::core;
  TerrainLayers terrain = make_terrain();
  for (SurfaceEstimate& estimate : terrain.surface.cells) {
    estimate.gradient_x = 0.1;
    estimate.gradient_y = -0.2;
    estimate.gradient_covariance = {0.04, 0.01, 0.01, 0.09};
  }
  const TraversabilityEvaluator evaluator(
      make_capability(1.0, 1.0, 1.0), make_footprint());
  const MotionSegment segment{{Pose2d{2.5, 2.5, 0.0,
                                     MonotonicTime{9'000'000'000}}}};
  const TraversabilityResult first =
      evaluator.evaluate(segment, terrain, make_policy());
  const TraversabilityResult second =
      evaluator.evaluate(segment, terrain, make_policy());

  require(first.traversable &&
              std::abs(first.maximum_longitudinal_mean_angle_rad -
                       0.09966865249116204) < 1.0e-15 &&
              std::abs(first.maximum_longitudinal_upper_angle_rad -
                       0.4636476090008061) < 1.0e-15 &&
              std::abs(first.minimum_longitudinal_lower_angle_rad +
                       0.2914567944778671) < 1.0e-15 &&
              std::abs(first.maximum_lateral_absolute_upper_angle_rad -
                       0.6747409422235527) < 1.0e-15,
          "the known covariance projection did not match the worked bounds");
  require(first.validity == second.validity &&
              first.traversable == second.traversable &&
              first.limiting_factors == second.limiting_factors &&
              first.maximum_longitudinal_mean_gradient ==
                  second.maximum_longitudinal_mean_gradient &&
              first.minimum_longitudinal_mean_gradient ==
                  second.minimum_longitudinal_mean_gradient &&
              first.maximum_longitudinal_mean_angle_rad ==
                  second.maximum_longitudinal_mean_angle_rad &&
              first.minimum_longitudinal_mean_angle_rad ==
                  second.minimum_longitudinal_mean_angle_rad &&
              first.maximum_longitudinal_upper_angle_rad ==
                  second.maximum_longitudinal_upper_angle_rad &&
              first.minimum_longitudinal_lower_angle_rad ==
                  second.minimum_longitudinal_lower_angle_rad &&
              first.maximum_lateral_absolute_upper_angle_rad ==
                  second.maximum_lateral_absolute_upper_angle_rad &&
              first.evaluated_footprint_samples ==
                  second.evaluated_footprint_samples &&
              first.evaluated_sweep_poses == second.evaluated_sweep_poses &&
              first.slope_sweep_discretization_margin_m ==
                  second.slope_sweep_discretization_margin_m &&
              first.risk_audit.gradient_risk_policy_version ==
                  second.risk_audit.gradient_risk_policy_version &&
              first.risk_audit.source_terrain_analysis_config_version ==
                  second.risk_audit.source_terrain_analysis_config_version &&
              first.risk_audit.policy_terrain_analysis_config_version ==
                  second.risk_audit.policy_terrain_analysis_config_version &&
              first.risk_audit.epsilon_local ==
                  second.risk_audit.epsilon_local &&
              first.risk_audit.coverage_model ==
                  second.risk_audit.coverage_model &&
              first.risk_audit.calibration_dataset_id ==
                  second.risk_audit.calibration_dataset_id &&
              first.risk_audit.operating_domain_id ==
                  second.risk_audit.operating_domain_id &&
              first.risk_audit.source_map_version ==
                  second.risk_audit.source_map_version &&
              first.risk_audit.risk_semantics ==
                  second.risk_audit.risk_semantics &&
              first.issues == second.issues &&
              first.worst_terrain_estimate_status ==
                  second.worst_terrain_estimate_status,
          "repeated directional slope evaluation changed a result field");
}

void roughness_hard_gate_covers_edges_and_fails_closed() {
  // Design: 18.2.2-17
  // Design: 18.2.2-invariant-8
  using namespace underwater_planner::core;
  TrackFootprint footprint = make_footprint();
  footprint.polygon = {
      {-0.6, -0.4}, {0.6, -0.4}, {0.6, 0.4}, {-0.6, 0.4}};
  const MotionSegment segment{{Pose2d{2.5, 2.5, 0.0,
                                     MonotonicTime{9'000'000'000}}}};
  RobotCapability capability = make_capability(1.0, 1.0, 1.0);
  capability.maximum_roughness_m = 0.1;
  TraversabilityEvaluator evaluator(capability, footprint);

  TerrainLayers edge_roughness = make_terrain();
  // The centre is smooth, while this footprint-edge cell is not.
  edge_roughness.surface.cells.at(2U * edge_roughness.surface.width + 3U)
      .detrended_roughness_rms_m = 0.100001;
  const TraversabilityResult rejected =
      evaluator.evaluate(segment, edge_roughness, make_policy());
  const TraversabilityResult rejected_repeat =
      evaluator.evaluate(segment, edge_roughness, make_policy());
  require(!rejected.traversable &&
              rejected.validity == TraversabilityEvaluationValidity::valid &&
              rejected.maximum_detrended_roughness_rms_m == 0.100001 &&
              std::find(rejected.limiting_factors.begin(),
                        rejected.limiting_factors.end(),
                        TraversabilityLimitingFactor::roughness_exceeded) !=
                  rejected.limiting_factors.end(),
          "roughness at the complete footprint edge was not a hard rejection");
  require(rejected_repeat.validity == rejected.validity &&
              rejected_repeat.traversable == rejected.traversable &&
              rejected_repeat.maximum_detrended_roughness_rms_m ==
                  rejected.maximum_detrended_roughness_rms_m &&
              rejected_repeat.evaluated_footprint_samples ==
                  rejected.evaluated_footprint_samples &&
              rejected_repeat.limiting_factors == rejected.limiting_factors,
          "repeated roughness evaluation changed an auditable result field");

  TerrainLayers boundary = make_terrain();
  boundary.surface.cells.at(2U * boundary.surface.width + 3U)
      .detrended_roughness_rms_m = 0.1;
  const TraversabilityResult accepted =
      evaluator.evaluate(segment, boundary, make_policy());
  require(accepted.traversable &&
              accepted.maximum_detrended_roughness_rms_m == 0.1,
          "roughness exactly at the capability boundary was rejected");

  TerrainLayers invalid = make_terrain();
  invalid.surface.cells.at(2U * invalid.surface.width + 3U)
      .detrended_roughness_rms_m =
      std::numeric_limits<double>::quiet_NaN();
  const TraversabilityResult invalid_result =
      evaluator.evaluate(segment, invalid, make_policy());
  require(!invalid_result.traversable &&
              invalid_result.validity ==
                  TraversabilityEvaluationValidity::terrain_invalid &&
              std::find(invalid_result.limiting_factors.begin(),
                        invalid_result.limiting_factors.end(),
                        TraversabilityLimitingFactor::roughness_invalid) !=
                  invalid_result.limiting_factors.end(),
          "non-finite roughness did not fail closed");

  invalid.surface.cells.at(2U * invalid.surface.width + 3U)
      .detrended_roughness_rms_m = -0.001;
  const TraversabilityResult negative_result =
      evaluator.evaluate(segment, invalid, make_policy());
  require(!negative_result.traversable &&
              negative_result.validity ==
                  TraversabilityEvaluationValidity::terrain_invalid &&
              std::find(negative_result.limiting_factors.begin(),
                        negative_result.limiting_factors.end(),
                        TraversabilityLimitingFactor::roughness_invalid) !=
                  negative_result.limiting_factors.end(),
          "negative roughness did not fail closed");
}

void production_robot_capability_assembly_carries_roughness() {
  // Design: 5.3, 14.2, 20.2
  using namespace underwater_planner::core;
  RobotParameterConfig parameters;
  parameters.maximum_slope_up_rad = 0.8;
  parameters.maximum_slope_down_rad = 0.8;
  parameters.maximum_slope_lateral_rad = 0.8;
  parameters.maximum_support_roll_rad = 0.8;
  parameters.maximum_step_climb_m = 0.3;
  parameters.maximum_step_drop_m = 0.3;
  parameters.minimum_track_support_ratio = 0.5;
  parameters.effective_track_spacing_m = 0.5;
  parameters.minimum_step_crossing_alignment = 0.2;
  parameters.step_alignment_transition_band = 0.1;
  parameters.maximum_roughness_m = 0.07;
  const auto capability = make_robot_capability(parameters);
  require(capability.has_value() &&
              capability->maximum_roughness_m == 0.07,
          "production robot capability assembly dropped maximum roughness");
  parameters.maximum_roughness_m.reset();
  require(!make_robot_capability(parameters).has_value(),
          "missing maximum roughness was silently defaulted during assembly");
  parameters.maximum_roughness_m = -0.01;
  require(!make_robot_capability(parameters).has_value(),
          "negative maximum roughness was accepted during assembly");
}

}  // namespace

int main() {
  gradient_is_projected_before_atan_with_signed_longitudinal_limits();
  complete_footprint_rejects_a_lateral_limit_at_its_edge();
  risk_policy_is_bound_to_analysis_calibration_domain_and_local_epsilon();
  mean_safe_but_conservative_upper_bound_rejects_segment();
  anisotropic_covariance_rotates_between_directional_bounds();
  asymmetric_up_down_bounds_include_the_same_nonzero_variance();
  invalid_gradient_covariance_rejects_the_whole_motion_segment();
  unavailable_or_out_of_map_footprint_surface_is_rejected();
  adaptive_sweep_rejects_a_hazard_between_segment_endpoints();
  invalid_public_inputs_are_rejected_with_finite_outputs();
  covariance_projection_and_repeated_results_are_auditable();
  roughness_hard_gate_covers_edges_and_fails_closed();
  production_robot_capability_assembly_carries_roughness();
  std::cout << "T09 directional slope checks passed\n";
}
