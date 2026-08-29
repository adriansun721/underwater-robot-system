#pragma once

#include "underwater_planner/core/cable_model.hpp"
#include "underwater_planner/core/traversability_evaluator.hpp"
#include "underwater_planner/core/versioned_snapshot.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace underwater_planner::core {

struct MergeGoalGenerationParameters {
  std::uint64_t version{};
  std::vector<double> merge_distances_m;
  std::vector<double> terminal_heading_offsets_rad;
  std::vector<double> terminal_lag_angles_rad;
  std::size_t maximum_goal_count{};
  double forward_position_tolerance_m{};
  double forward_heading_tolerance_rad{};
  double merge_distance_cost_weight{};
  double absolute_lag_cost_weight{};
};

struct MergeGoalSoftCost {
  double merge_distance_component{};
  double absolute_lag_component{};
  double total{};
};

struct MergeGoal {
  Pose2d robot_pose;
  double cable_lag_angle_rad{};
  double reference_progress_m{};
  std::uint32_t reference_line_version{};
  Vector2m touchdown_target_m;
  double cable_heading_rad{};
  double merge_distance_m{};
  MergeGoalSoftCost soft_cost;
  std::uint64_t generation_parameters_version{};
  std::uint64_t cable_model_version{};
  std::uint32_t robot_operating_area_version{};
};

struct MergeGoalCandidateAttempt {
  double merge_distance_m{};
  double terminal_heading_offset_rad{};
  double terminal_lag_angle_rad{};
};

enum class MergeGoalRejectionReason {
  invalid_input,
  reference_version_mismatch,
  merge_progress_outside_reference,
  lag_angle_out_of_range,
  forward_model_mismatch,
  robot_footprint_outside_area,
  terrain_evaluation_invalid,
  terminal_terrain_not_traversable,
  goal_limit_reached,
};

struct MergeGoalDiagnostic {
  MergeGoalRejectionReason reason{MergeGoalRejectionReason::invalid_input};
  MergeGoalCandidateAttempt candidate;
  std::string message;
};

struct MergeGoalGenerationResult {
  bool valid_input{};
  std::vector<MergeGoal> goals;
  std::vector<MergeGoalDiagnostic> diagnostics;
  std::uint64_t generation_parameters_version{};
  std::uint64_t cable_model_version{};
  std::uint32_t reference_line_version{};
  std::uint32_t robot_operating_area_version{};
  std::string cable_calibration_dataset_id;
  std::string operating_domain_id;
  std::string reference_coordinate_frame;
  std::string robot_operating_area_id;
  TerrainGradientRiskAudit terrain_risk_audit;
};

class MergeGoalGenerator {
 public:
  MergeGoalGenerator(CableModelParameters cable_model_parameters,
                     MergeGoalGenerationParameters generation_parameters,
                     RobotCapability robot_capability,
                     TrackFootprint track_footprint);

  [[nodiscard]] MergeGoalGenerationResult generate(
      const ReferenceProgress& current_progress,
      const ReferenceLine& reference_line,
      const RobotOperatingArea& robot_operating_area,
      const TerrainLayers& terrain,
      const TerrainGradientRiskPolicy& terrain_gradient_risk_policy) const;

 private:
  MergeGoalGenerationParameters generation_parameters_;
  std::vector<Point2d> robot_footprint_body_m_;
  CableModel cable_model_;
  TraversabilityEvaluator traversability_evaluator_;
};

}  // namespace underwater_planner::core
