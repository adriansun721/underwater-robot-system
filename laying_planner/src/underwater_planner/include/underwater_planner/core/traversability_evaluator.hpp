#pragma once

#include "underwater_planner/core/data_contract.hpp"
#include "underwater_planner/core/terrain_analyzer.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace underwater_planner::core {

struct RobotParameterConfig;

struct RobotCollisionRiskPolicy {
  std::uint64_t version{};
  std::string calibration_dataset_id;
  std::string operating_domain_id;
  double epsilon_robot{};
  double minimum_map_confidence{};
  double safe_distance_m{};
};

enum class CollisionEvaluationValidity {
  valid,
  input_invalid,
  covariance_invalid,
  version_mismatch,
};

enum class CollisionCellClassification {
  traversable,
  step_discontinuity_requires_validation,
  obstacle,
  inflated_obstacle,
  unknown,
  low_confidence,
  invalid_terrain,
  map_boundary,
  input_invalid,
};

enum class InformationGapReason {
  unknown,
  low_confidence,
  invalid_terrain,
};

struct InformationGap {
  std::size_t row{};
  std::size_t column{};
  Point2d center;
  InformationGapReason reason{InformationGapReason::unknown};
  // T35 metadata: a gap is tied to the immutable map/reference snapshot and
  // to the interval on the task reference line that produced it.
  double start_progress_m{};
  double end_progress_m{};
  double minimum_confidence{};
  MapVersion source_map_version;
  std::uint32_t reference_line_version{};
};

struct CollisionCellResult {
  CollisionCellClassification classification{
      CollisionCellClassification::input_invalid};
  double collision_margin_m{};

  [[nodiscard]] bool traversable() const noexcept {
    return classification == CollisionCellClassification::traversable;
  }

  // A recognized terrain discontinuity is not independently traversable.
  // Collision sweep may defer it only to the direction-aware step gate.
  [[nodiscard]] bool collision_candidate() const noexcept {
    return traversable() ||
           classification ==
               CollisionCellClassification::
                   step_discontinuity_requires_validation;
  }
};

struct CollisionLayerResult {
  CollisionEvaluationValidity validity{
      CollisionEvaluationValidity::input_invalid};
  MapVersion source_map_version;
  std::uint64_t terrain_analysis_config_version{};
  std::uint64_t collision_risk_policy_version{};
  double epsilon_robot{};
  std::string operating_domain_id;
  std::string calibration_dataset_id;
  std::string risk_semantics;
  std::size_t width{};
  std::size_t height{};
  std::vector<CollisionCellResult> cells;
  std::vector<InformationGap> information_gaps;
  std::vector<std::string> issues;

  [[nodiscard]] const CollisionCellResult& at(std::size_t row,
                                               std::size_t column) const;
};

struct CollisionSweepResult {
  CollisionEvaluationValidity validity{
      CollisionEvaluationValidity::input_invalid};
  bool collision_free{};
  std::size_t evaluated_sweep_poses{};
  std::size_t evaluated_footprint_cells{};
  double maximum_boundary_displacement_m{};
  double sweep_discretization_margin_m{};
};

struct RobotCapability {
  double maximum_slope_up_rad{};
  double maximum_slope_down_rad{};
  double maximum_slope_lateral_rad{};
  double maximum_support_roll_rad{};
  double maximum_step_climb_m{};
  double maximum_step_drop_m{};
  double minimum_track_support_ratio{};
  double effective_track_spacing_m{};
  double minimum_step_crossing_alignment{};
  double step_alignment_transition_band{};
  // Maximum detrended surface roughness accepted under the complete robot
  // footprint, in metres. This is a hard robot capability, not a soft cost.
  double maximum_roughness_m{};
};

// Assemble the runtime capability used by TraversabilityEvaluator from the
// versioned robot parameter group. Missing or invalid capability fields fail
// closed instead of being filled with example/default values.
[[nodiscard]] std::optional<RobotCapability> make_robot_capability(
    const RobotParameterConfig& parameters);

struct TrackFootprint {
  std::vector<Point2d> polygon;
  std::vector<Point2d> left_support_polygon;
  std::vector<Point2d> right_support_polygon;
};

[[nodiscard]] double track_footprint_radius(
    const TrackFootprint& footprint) noexcept;

struct MotionSegment {
  std::vector<Pose2d> samples;
};

enum class GradientCoverageModel {
  unspecified,
  calibrated_gaussian,
  empirical_bounded,
  deterministic_bounded,
};

enum class TerrainGradientRiskSemantics {
  local_pointwise_only_no_path_joint_guarantee,
};

struct TerrainGradientRiskPolicy {
  std::uint64_t version{};
  std::uint64_t terrain_analysis_config_version{};
  double epsilon_local{};
  double coverage_multiplier{};
  GradientCoverageModel coverage_model{GradientCoverageModel::unspecified};
  std::string calibration_dataset_id;
  std::string operating_domain_id;
  bool coverage_calibrated{};
};

struct TerrainGradientRiskAudit {
  MapVersion source_map_version;
  std::uint64_t gradient_risk_policy_version{};
  std::uint64_t source_terrain_analysis_config_version{};
  std::uint64_t policy_terrain_analysis_config_version{};
  double epsilon_local{};
  double coverage_multiplier{};
  GradientCoverageModel coverage_model{GradientCoverageModel::unspecified};
  std::string calibration_dataset_id;
  std::string operating_domain_id;
  bool path_joint_risk_implemented{};
  TerrainGradientRiskSemantics risk_semantics{
      TerrainGradientRiskSemantics::
          local_pointwise_only_no_path_joint_guarantee};
};

[[nodiscard]] TerrainGradientRiskAudit make_terrain_gradient_risk_audit(
    const TerrainLayers& terrain,
    const TerrainGradientRiskPolicy& gradient_risk_policy);

enum class TraversabilityEvaluationValidity {
  valid,
  input_invalid,
  risk_policy_invalid,
  version_mismatch,
  covariance_invalid,
  terrain_invalid,
};

enum class TraversabilityLimitingFactor {
  up_slope_exceeded,
  down_slope_exceeded,
  lateral_slope_exceeded,
  gradient_covariance_invalid,
  terrain_estimate_invalid,
  footprint_outside_terrain,
  step_climb_exceeded,
  step_drop_exceeded,
  step_transition_height_exceeded,
  roughness_exceeded,
  roughness_invalid,
  left_track_support_insufficient,
  right_track_support_insufficient,
  support_roll_exceeded,
  local_track_drop_exceeded,
  track_elevation_outlier_detected,
};

enum class StepCrossingType {
  none,
  climb,
  drop,
  edge_riding,
  transition,
};

enum class StepCrossingDirection {
  none,
  low_to_high,
  high_to_low,
};

struct StepCrossingEvent {
  StepCrossingType type{StepCrossingType::none};
  StepCrossingDirection direction{StepCrossingDirection::none};
  double complete_height_m{};
  Pose2d contact_pose;
};

struct TraversabilityResult {
  TraversabilityEvaluationValidity validity{
      TraversabilityEvaluationValidity::input_invalid};
  bool traversable{};
  std::vector<TraversabilityLimitingFactor> limiting_factors;
  double maximum_longitudinal_mean_gradient{};
  double minimum_longitudinal_mean_gradient{};
  double maximum_longitudinal_mean_angle_rad{};
  double minimum_longitudinal_mean_angle_rad{};
  double maximum_longitudinal_upper_angle_rad{};
  double minimum_longitudinal_lower_angle_rad{};
  double maximum_lateral_absolute_upper_angle_rad{};
  double maximum_detrended_roughness_rms_m{};
  std::size_t evaluated_footprint_samples{};
  std::size_t evaluated_sweep_poses{};
  double slope_sweep_discretization_margin_m{};
  StepCrossingType step_crossing_type{StepCrossingType::none};
  std::vector<StepCrossingEvent> step_crossing_events;
  double maximum_complete_step_height_m{};
  double maximum_absolute_support_roll_rad{};
  double minimum_left_track_support_ratio{1.0};
  double minimum_right_track_support_ratio{1.0};
  double maximum_local_track_drop_m{};
  bool track_elevation_outlier_detected{};
  TerrainGradientRiskAudit risk_audit;
  std::vector<std::string> issues;
  TerrainEstimateStatus worst_terrain_estimate_status{
      TerrainEstimateStatus::valid};
};

class TraversabilityEvaluator {
 public:
  TraversabilityEvaluator() = default;
  TraversabilityEvaluator(RobotCapability capability,
                          TrackFootprint track_footprint);

  [[nodiscard]] CollisionLayerResult evaluate_collision_layer(
      const MapSnapshot& map, const TerrainLayers& terrain,
      const Covariance2dM2& robot_relative_obstacle_covariance_m2,
      const RobotCollisionRiskPolicy& policy) const;

  [[nodiscard]] CollisionSweepResult evaluate_collision_sweep(
      const MotionSegment& segment, const TerrainLayers& terrain,
      const CollisionLayerResult& collision_layer,
      double maximum_sweep_spacing_fraction) const;

  // This direction-aware gate is mandatory after collision sweep whenever
  // the layer contains step_discontinuity_requires_validation cells.
  [[nodiscard]] TraversabilityResult evaluate(
      const MotionSegment& segment, const TerrainLayers& terrain,
      const TerrainGradientRiskPolicy& gradient_risk_policy) const;

 private:
  RobotCapability capability_;
  TrackFootprint track_footprint_;
};

}  // namespace underwater_planner::core
