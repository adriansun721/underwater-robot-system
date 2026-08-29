#include "underwater_planner/core/traversability_evaluator.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace {

void require(const bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "T08 failure: " << message << '\n';
    std::exit(1);
  }
}

underwater_planner::core::MapSnapshot make_map() {
  using namespace underwater_planner::core;
  MapSnapshot map;
  map.version = {"collision-map", 8U, MonotonicTime{8'000'000'000}, "map"};
  map.width = 9U;
  map.height = 9U;
  map.resolution_m = 1.0;
  map.origin_x_m = 0.0;
  map.origin_y_m = 0.0;
  map.derived_configuration_version = 17U;
  map.cells.assign(map.width * map.height,
                   MapCell{0.0, 0.01, 1.0, true});
  for (MapCell& cell : map.cells) {
    cell.measurement_timestamp = map.version.timestamp;
  }
  return map;
}

underwater_planner::core::TerrainLayers make_terrain(
    const underwater_planner::core::MapSnapshot& map) {
  using namespace underwater_planner::core;
  TerrainLayers terrain;
  terrain.source_map_version = map.version;
  terrain.analysis_config_version = map.derived_configuration_version;
  terrain.operating_domain_id = "collision-domain";
  terrain.surface.width = map.width;
  terrain.surface.height = map.height;
  terrain.surface.resolution_m = map.resolution_m;
  terrain.surface.origin_x_m = map.origin_x_m;
  terrain.surface.origin_y_m = map.origin_y_m;
  terrain.surface.cells.assign(map.width * map.height, SurfaceEstimate{});
  for (SurfaceEstimate& estimate : terrain.surface.cells) {
    estimate.status = TerrainEstimateStatus::valid;
    estimate.support_ratio = 1.0;
  }
  return terrain;
}

underwater_planner::core::RobotCollisionRiskPolicy make_policy() {
  using namespace underwater_planner::core;
  return RobotCollisionRiskPolicy{3U, "collision-calibration-v3",
                                  "collision-domain", 0.15865525393145707,
                                  0.5, 0.0};
}

void local_normal_uses_directional_margin_and_missing_normal_uses_upper_bound() {
  // Design: 18.2.2-1
  // Design: 18.2.2-4
  using namespace underwater_planner::core;
  MapSnapshot directional_map = make_map();
  MapCell& directional_obstacle = directional_map.cells.at(4U * 9U + 4U);
  directional_obstacle.obstacle = true;
  directional_obstacle.obstacle_normal = Vector2d{0.0, 1.0};

  const Covariance2dM2 robot_relative_covariance{4.0, 0.0, 0.0, 1.0};
  const CollisionLayerResult directional =
      TraversabilityEvaluator{}.evaluate_collision_layer(
          directional_map, make_terrain(directional_map),
          robot_relative_covariance, make_policy());
  require(directional.validity == CollisionEvaluationValidity::valid,
          "directional collision evaluation was rejected");
  require(directional.operating_domain_id == "collision-domain" &&
              directional.calibration_dataset_id ==
                  "collision-calibration-v3" &&
              directional.risk_semantics ==
                  "robot-relative-obstacle-pointwise-only",
          "collision result omitted domain, calibration, or risk semantics");
  require(std::abs(directional.cells.at(4U * 9U + 4U).collision_margin_m -
                   1.0) < 1.0e-8,
          "local obstacle normal did not project robot covariance");
  require(directional.at(4U, 6U).classification ==
              CollisionCellClassification::traversable,
          "directional margin used the larger isotropic radius");

  MapSnapshot isotropic_map = make_map();
  isotropic_map.cells.at(4U * 9U + 4U).obstacle = true;
  const CollisionLayerResult isotropic =
      TraversabilityEvaluator{}.evaluate_collision_layer(
          isotropic_map, make_terrain(isotropic_map), robot_relative_covariance,
          make_policy());
  require(std::abs(isotropic.cells.at(4U * 9U + 4U).collision_margin_m -
                   2.0) < 1.0e-8,
          "missing normal did not use maximum-eigenvalue upper bound");
  require(isotropic.at(4U, 6U).classification ==
              CollisionCellClassification::inflated_obstacle,
          "isotropic upper bound did not conservatively inflate obstacle");
}

void complete_complex_footprint_rejects_an_edge_collision() {
  // Design: 18.2.2-2
  // Design: 18.2.2-invariant-2
  using namespace underwater_planner::core;
  MapSnapshot map = make_map();
  map.cells.at(4U * map.width + 5U).obstacle = true;
  const TerrainLayers terrain = make_terrain(map);
  const CollisionLayerResult collision_layer =
      TraversabilityEvaluator{}.evaluate_collision_layer(
          map, terrain, Covariance2dM2{}, make_policy());
  require(collision_layer.validity == CollisionEvaluationValidity::valid &&
              collision_layer.at(4U, 4U).traversable() &&
              !collision_layer.at(4U, 5U).traversable(),
          "the fixture did not isolate an obstacle outside the robot center cell");

  const TrackFootprint footprint{
      {{-0.9, -0.5}, {1.4, -0.5}, {1.6, 0.0},
       {1.4, 0.5}, {-0.9, 0.5}, {-1.1, 0.0}},
      {{-0.7, 0.1}, {0.7, 0.1}, {0.7, 0.4}, {-0.7, 0.4}},
      {{-0.7, -0.4}, {0.7, -0.4}, {0.7, -0.1}, {-0.7, -0.1}}};
  const MotionSegment centered_away_from_obstacle{{
      Pose2d{4.0, 4.0, 0.0, MonotonicTime{8'000'000'000}}}};
  const CollisionSweepResult result =
      TraversabilityEvaluator(RobotCapability{}, footprint)
          .evaluate_collision_sweep(centered_away_from_obstacle, terrain,
                                    collision_layer, 0.25);

  require(result.validity == CollisionEvaluationValidity::valid &&
              !result.collision_free && result.evaluated_sweep_poses == 1U &&
              result.evaluated_footprint_cells > 1U,
          "a collision at the complex footprint edge was hidden by the free center cell");
}

void collision_sweep_requires_the_complete_map_version() {
  // Design: 18.2.2-18
  using namespace underwater_planner::core;
  const MapSnapshot map = make_map();
  const TerrainLayers terrain = make_terrain(map);
  const CollisionLayerResult collision_layer =
      TraversabilityEvaluator{}.evaluate_collision_layer(
          map, terrain, Covariance2dM2{}, make_policy());
  const TrackFootprint footprint{
      {{-0.4, -0.4}, {0.4, -0.4}, {0.4, 0.4}, {-0.4, 0.4}},
      {{-0.3, 0.1}, {0.3, 0.1}, {0.3, 0.3}, {-0.3, 0.3}},
      {{-0.3, -0.3}, {0.3, -0.3}, {0.3, -0.1}, {-0.3, -0.1}}};
  const TraversabilityEvaluator evaluator{RobotCapability{}, footprint};
  const MotionSegment segment{{
      Pose2d{4.0, 4.0, 0.0, MonotonicTime{8'000'000'000}}}};

  const CollisionSweepResult matching = evaluator.evaluate_collision_sweep(
      segment, terrain, collision_layer, 0.25);
  require(matching.validity == CollisionEvaluationValidity::valid &&
              matching.collision_free && matching.evaluated_sweep_poses > 0U &&
              matching.evaluated_footprint_cells > 0U,
          "a completely matching map version did not execute the collision sweep");

  const auto require_version_rejected =
      [&](const CollisionLayerResult& mismatched,
          const std::string& field_name) {
        const CollisionSweepResult result = evaluator.evaluate_collision_sweep(
            segment, terrain, mismatched, 0.25);
        require(result.validity == CollisionEvaluationValidity::input_invalid &&
                    !result.collision_free &&
                    result.evaluated_sweep_poses == 0U &&
                    result.evaluated_footprint_cells == 0U &&
                    std::isfinite(result.maximum_boundary_displacement_m) &&
                    std::isfinite(result.sweep_discretization_margin_m),
                "collision sweep did not fail closed before evaluation for mismatched " +
                    field_name);
      };

  CollisionLayerResult mismatched = collision_layer;
  ++mismatched.source_map_version.timestamp.nanoseconds;
  require_version_rejected(mismatched, "map timestamp");
  mismatched = collision_layer;
  mismatched.source_map_version.map_id += "-other";
  require_version_rejected(mismatched, "map id");
  mismatched = collision_layer;
  ++mismatched.source_map_version.sequence_number;
  require_version_rejected(mismatched, "map sequence");
  mismatched = collision_layer;
  mismatched.source_map_version.coordinate_frame += "-other";
  require_version_rejected(mismatched, "map coordinate frame");
  mismatched = collision_layer;
  ++mismatched.terrain_analysis_config_version;
  require_version_rejected(mismatched, "terrain analysis configuration");
}

void unavailable_terrain_is_blocked_and_preserved_as_information_gaps() {
  // Design: 18.2.2-3
  // Design: 18.2.2-invariant-3
  using namespace underwater_planner::core;
  MapSnapshot map = make_map();
  map.cells.at(4U * 9U + 3U).known = false;
  map.cells.at(4U * 9U + 4U).confidence = 0.49;
  TerrainLayers terrain = make_terrain(map);
  terrain.surface.cells.at(4U * 9U + 5U).status =
      TerrainEstimateStatus::ill_conditioned;

  RobotCollisionRiskPolicy policy = make_policy();
  policy.minimum_map_confidence = 0.5;
  const CollisionLayerResult result =
      TraversabilityEvaluator{}.evaluate_collision_layer(
          map, terrain, Covariance2dM2{0.0, 0.0, 0.0, 0.0}, policy);

  require(result.validity == CollisionEvaluationValidity::valid,
          "unavailable terrain made the complete layer invalid");
  require(result.at(4U, 3U).classification ==
              CollisionCellClassification::unknown &&
              result.at(4U, 4U).classification ==
                  CollisionCellClassification::low_confidence &&
              result.at(4U, 5U).classification ==
                  CollisionCellClassification::invalid_terrain,
          "unknown, low-confidence, and invalid terrain were not distinct");
  require(result.information_gaps.size() == 3U,
          "information gap positions were not retained");
  require(result.information_gaps.at(0).reason == InformationGapReason::unknown &&
              result.information_gaps.at(0).row == 4U &&
              result.information_gaps.at(0).column == 3U &&
              result.information_gaps.at(1).reason ==
                  InformationGapReason::low_confidence &&
              result.information_gaps.at(2).reason ==
                  InformationGapReason::invalid_terrain,
          "information gaps lost deterministic locations or reasons");
}

void discontinuity_is_deferred_but_not_reported_as_traversable() {
  // Design: 18.2.2-6, 18.2.2-invariant-4
  using namespace underwater_planner::core;
  MapSnapshot map = make_map();
  TerrainLayers terrain = make_terrain(map);
  terrain.surface.cells.at(4U * map.width + 4U).status =
      TerrainEstimateStatus::discontinuous;

  const CollisionLayerResult result =
      TraversabilityEvaluator{}.evaluate_collision_layer(
          map, terrain, Covariance2dM2{}, make_policy());
  const CollisionCellResult& discontinuity = result.at(4U, 4U);
  require(result.validity == CollisionEvaluationValidity::valid &&
              discontinuity.classification ==
                  CollisionCellClassification::
                      step_discontinuity_requires_validation &&
              !discontinuity.traversable() &&
              discontinuity.collision_candidate(),
          "a step discontinuity bypassed the mandatory direction-aware gate");
}

void obstacles_boundaries_and_invalid_covariance_fail_safely() {
  // Design: 18.2.2-invariant-1
  using namespace underwater_planner::core;
  MapSnapshot obstacle_map = make_map();
  obstacle_map.cells.at(4U * 9U + 4U).obstacle = true;
  obstacle_map.cells.at(4U * 9U + 5U).obstacle = true;
  RobotCollisionRiskPolicy obstacle_policy = make_policy();
  obstacle_policy.safe_distance_m = 1.1;
  const CollisionLayerResult obstacle_result =
      TraversabilityEvaluator{}.evaluate_collision_layer(
          obstacle_map, make_terrain(obstacle_map),
          Covariance2dM2{0.0, 0.0, 0.0, 0.0}, obstacle_policy);
  require(obstacle_result.at(4U, 4U).classification ==
              CollisionCellClassification::obstacle &&
              obstacle_result.at(4U, 5U).classification ==
                  CollisionCellClassification::obstacle,
          "an original obstacle was overwritten by another inflation region");

  MapSnapshot boundary_map = make_map();
  RobotCollisionRiskPolicy boundary_policy = make_policy();
  boundary_policy.safe_distance_m = 0.6;
  const CollisionLayerResult boundary_result =
      TraversabilityEvaluator{}.evaluate_collision_layer(
          boundary_map, make_terrain(boundary_map),
          Covariance2dM2{0.0, 0.0, 0.0, 0.0}, boundary_policy);
  require(boundary_result.at(0U, 4U).classification ==
              CollisionCellClassification::map_boundary &&
              boundary_result.at(1U, 4U).classification ==
                  CollisionCellClassification::traversable,
          "map exterior was not conservatively inflated as an obstacle");

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const CollisionLayerResult invalid =
      TraversabilityEvaluator{}.evaluate_collision_layer(
          boundary_map, make_terrain(boundary_map),
          Covariance2dM2{nan, 0.0, 0.0, 1.0}, make_policy());
  require(invalid.validity == CollisionEvaluationValidity::covariance_invalid,
          "non-finite robot-relative covariance was accepted");
  for (const CollisionCellResult& cell : invalid.cells) {
    require(!cell.traversable(),
            "invalid covariance left a traversable collision cell");
  }

  const CollisionLayerResult indefinite =
      TraversabilityEvaluator{}.evaluate_collision_layer(
          boundary_map, make_terrain(boundary_map),
          Covariance2dM2{1.0, 2.0, 2.0, 1.0}, make_policy());
  require(indefinite.validity ==
              CollisionEvaluationValidity::covariance_invalid,
          "non-positive-semidefinite covariance was accepted");
  const CollisionLayerResult small_indefinite =
      TraversabilityEvaluator{}.evaluate_collision_layer(
          boundary_map, make_terrain(boundary_map),
          Covariance2dM2{0.0, 1.0e-7, 1.0e-7, 0.0}, make_policy());
  require(small_indefinite.validity ==
              CollisionEvaluationValidity::covariance_invalid,
          "small indefinite covariance was hidden by an absolute tolerance");

  RobotCollisionRiskPolicy zero_confidence_policy = make_policy();
  zero_confidence_policy.minimum_map_confidence = 0.0;
  const CollisionLayerResult zero_confidence =
      TraversabilityEvaluator{}.evaluate_collision_layer(
          boundary_map, make_terrain(boundary_map),
          Covariance2dM2{0.0, 0.0, 0.0, 0.0}, zero_confidence_policy);
  require(zero_confidence.validity == CollisionEvaluationValidity::input_invalid,
          "zero minimum confidence disabled conservative unknown handling");

  MapSnapshot invalid_normal_map = make_map();
  invalid_normal_map.cells.at(4U * 9U + 4U).obstacle = true;
  invalid_normal_map.cells.at(4U * 9U + 4U).obstacle_normal =
      Vector2d{0.0, 0.0};
  const CollisionLayerResult invalid_normal =
      TraversabilityEvaluator{}.evaluate_collision_layer(
          invalid_normal_map, make_terrain(invalid_normal_map),
          Covariance2dM2{1.0, 0.0, 0.0, 1.0}, make_policy());
  require(invalid_normal.validity == CollisionEvaluationValidity::input_invalid,
          "invalid local obstacle normal entered covariance projection");
  for (const CollisionCellResult& cell : invalid_normal.cells) {
    require(!cell.traversable(),
            "invalid obstacle normal left a traversable collision cell");
  }
}

void repeated_collision_evaluation_is_fieldwise_deterministic() {
  using namespace underwater_planner::core;
  MapSnapshot map = make_map();
  map.cells.at(4U * 9U + 4U).obstacle = true;
  map.cells.at(4U * 9U + 4U).obstacle_normal = Vector2d{1.0, 1.0};
  map.cells.at(3U * 9U + 3U).known = false;
  const TerrainLayers terrain = make_terrain(map);
  const Covariance2dM2 covariance{2.0, 0.25, 0.25, 1.0};
  const CollisionLayerResult first =
      TraversabilityEvaluator{}.evaluate_collision_layer(
          map, terrain, covariance, make_policy());
  const CollisionLayerResult second =
      TraversabilityEvaluator{}.evaluate_collision_layer(
          map, terrain, covariance, make_policy());

  require(first.validity == second.validity &&
              first.source_map_version.map_id ==
                  second.source_map_version.map_id &&
              first.source_map_version.sequence_number ==
                  second.source_map_version.sequence_number &&
              first.terrain_analysis_config_version ==
                  second.terrain_analysis_config_version &&
              first.collision_risk_policy_version ==
                  second.collision_risk_policy_version &&
              first.epsilon_robot == second.epsilon_robot &&
              first.operating_domain_id == second.operating_domain_id &&
              first.calibration_dataset_id == second.calibration_dataset_id &&
              first.risk_semantics == second.risk_semantics &&
              first.cells.size() == second.cells.size() &&
              first.information_gaps.size() == second.information_gaps.size() &&
              first.issues == second.issues,
          "repeated evaluation changed collision layer metadata");
  for (std::size_t index = 0; index < first.cells.size(); ++index) {
    require(first.cells[index].classification ==
                    second.cells[index].classification &&
                first.cells[index].collision_margin_m ==
                    second.cells[index].collision_margin_m,
            "repeated evaluation changed a collision cell");
  }
  for (std::size_t index = 0; index < first.information_gaps.size(); ++index) {
    const InformationGap& left = first.information_gaps[index];
    const InformationGap& right = second.information_gaps[index];
    require(left.row == right.row && left.column == right.column &&
                left.center.x_m == right.center.x_m &&
                left.center.y_m == right.center.y_m &&
                left.reason == right.reason,
            "repeated evaluation changed an information gap");
  }
}

}  // namespace

int main() {
  local_normal_uses_directional_margin_and_missing_normal_uses_upper_bound();
  complete_complex_footprint_rejects_an_edge_collision();
  collision_sweep_requires_the_complete_map_version();
  unavailable_terrain_is_blocked_and_preserved_as_information_gaps();
  discontinuity_is_deferred_but_not_reported_as_traversable();
  obstacles_boundaries_and_invalid_covariance_fail_safely();
  repeated_collision_evaluation_is_fieldwise_deterministic();
  std::cout << "T08 traversability evaluator checks passed\n";
}
