#include "underwater_planner/core/merge_goal_generator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace underwater_planner::core {
namespace {

[[nodiscard]] bool finite(const double value) noexcept {
  return std::isfinite(value);
}

[[nodiscard]] bool valid_generation_parameters(
    const MergeGoalGenerationParameters& parameters) noexcept {
  const auto finite_values = [](const std::vector<double>& values) {
    return !values.empty() &&
           std::all_of(values.begin(), values.end(), finite);
  };
  return parameters.version != 0U &&
         finite_values(parameters.merge_distances_m) &&
         std::all_of(parameters.merge_distances_m.begin(),
                     parameters.merge_distances_m.end(),
                     [](const double distance) { return distance > 0.0; }) &&
         finite_values(parameters.terminal_heading_offsets_rad) &&
         finite_values(parameters.terminal_lag_angles_rad) &&
         parameters.maximum_goal_count > 0U &&
         finite(parameters.forward_position_tolerance_m) &&
         parameters.forward_position_tolerance_m >= 0.0 &&
         finite(parameters.forward_heading_tolerance_rad) &&
         parameters.forward_heading_tolerance_rad >= 0.0 &&
         finite(parameters.merge_distance_cost_weight) &&
         parameters.merge_distance_cost_weight >= 0.0 &&
         finite(parameters.absolute_lag_cost_weight) &&
         parameters.absolute_lag_cost_weight >= 0.0 &&
         (parameters.merge_distance_cost_weight > 0.0 ||
          parameters.absolute_lag_cost_weight > 0.0);
}

void reject(MergeGoalGenerationResult& result,
            const MergeGoalRejectionReason reason,
            const MergeGoalCandidateAttempt candidate,
            std::string message) {
  result.diagnostics.push_back({reason, candidate, std::move(message)});
}

}  // namespace

MergeGoalGenerator::MergeGoalGenerator(
    CableModelParameters cable_model_parameters,
    MergeGoalGenerationParameters generation_parameters,
    const RobotCapability robot_capability, TrackFootprint track_footprint)
    : generation_parameters_(std::move(generation_parameters)),
      robot_footprint_body_m_(track_footprint.polygon),
      cable_model_(std::move(cable_model_parameters)),
      traversability_evaluator_(robot_capability, std::move(track_footprint)) {
  if (!valid_generation_parameters(generation_parameters_)) {
    throw std::invalid_argument("merge goal generation parameters are invalid");
  }
}

MergeGoalGenerationResult MergeGoalGenerator::generate(
    const ReferenceProgress& current_progress,
    const ReferenceLine& reference_line,
    const RobotOperatingArea& robot_operating_area,
    const TerrainLayers& terrain,
    const TerrainGradientRiskPolicy& terrain_gradient_risk_policy) const {
  MergeGoalGenerationResult result;
  const CableModelIdentity cable_identity = cable_model_.identity();
  result.generation_parameters_version = generation_parameters_.version;
  result.cable_model_version = cable_identity.version;
  result.reference_line_version = reference_line.version;
  result.robot_operating_area_version = robot_operating_area.version;
  result.cable_calibration_dataset_id = cable_identity.calibration_dataset_id;
  result.operating_domain_id = cable_identity.operating_domain_id;
  result.reference_coordinate_frame = reference_line.coordinate_frame;
  result.robot_operating_area_id = robot_operating_area.id;
  result.terrain_risk_audit =
      make_terrain_gradient_risk_audit(terrain,
                                       terrain_gradient_risk_policy);

  if (!validate(current_progress).valid || !validate(reference_line).valid ||
      !validate(robot_operating_area).valid) {
    reject(result, MergeGoalRejectionReason::invalid_input, {},
           "reference progress, reference line, or robot area is invalid");
    return result;
  }
  if (current_progress.reference_line_version != reference_line.version) {
    reject(result, MergeGoalRejectionReason::reference_version_mismatch, {},
           "reference progress does not match the locked reference line");
    return result;
  }
  result.valid_input = true;

  for (const double merge_distance_m :
       generation_parameters_.merge_distances_m) {
    const double goal_progress_m =
        current_progress.arc_length_m + merge_distance_m;
    const std::optional<ReferencePoint> reference_goal =
        reference_line.query(goal_progress_m);
    if (!reference_goal.has_value()) {
      reject(result, MergeGoalRejectionReason::merge_progress_outside_reference,
             {merge_distance_m, 0.0, 0.0},
             "merge progress is outside the locked reference line");
      continue;
    }
    const double reference_heading_rad =
        std::atan2(reference_goal->tangent_y, reference_goal->tangent_x);
    const Vector2m touchdown_target{reference_goal->x_m,
                                    reference_goal->y_m};
    for (const double heading_offset_rad :
         generation_parameters_.terminal_heading_offsets_rad) {
      const double robot_heading_rad = normalize_angle_radians(
          reference_heading_rad + heading_offset_rad);
      for (const double lag_angle_rad :
           generation_parameters_.terminal_lag_angles_rad) {
        const MergeGoalCandidateAttempt candidate{
            merge_distance_m, heading_offset_rad, lag_angle_rad};
        const CableInverseMeanSample inverse = cable_model_.inverse_touchdown_mean(
            touchdown_target, reference_heading_rad, robot_heading_rad,
            lag_angle_rad, current_progress.timestamp);
        const CableMeanSample& forward = inverse.forward_prediction;
        if (forward.validity == CableModelValidity::lag_angle_out_of_range) {
          reject(result, MergeGoalRejectionReason::lag_angle_out_of_range,
                 candidate,
                 "terminal lag angle exceeds the calibrated cable-model range");
          continue;
        }
        const double position_error_m =
            forward.validity == CableModelValidity::valid
                ? std::hypot(forward.touchdown_position_m.x_m -
                                 touchdown_target.x_m,
                             forward.touchdown_position_m.y_m -
                                 touchdown_target.y_m)
                : 0.0;
        const double heading_error_rad =
            forward.validity == CableModelValidity::valid
                ? std::abs(normalize_angle_radians(
                      forward.cable_heading_rad - reference_heading_rad))
                : 0.0;
        if (forward.validity != CableModelValidity::valid ||
            position_error_m >
                generation_parameters_.forward_position_tolerance_m ||
            heading_error_rad >
                generation_parameters_.forward_heading_tolerance_rad) {
          reject(result, MergeGoalRejectionReason::forward_model_mismatch,
                 candidate,
                 "forward cable prediction does not hit the touchdown target");
          continue;
        }

        if (!robot_operating_area.contains_footprint(
                robot_footprint_body_m_, inverse.robot_pose)) {
          reject(result,
                 MergeGoalRejectionReason::robot_footprint_outside_area,
                 candidate,
                 "complete terminal footprint is outside the robot operating area");
          continue;
        }

        const TraversabilityResult traversability =
            traversability_evaluator_.evaluate(
                MotionSegment{{inverse.robot_pose}}, terrain,
                terrain_gradient_risk_policy);
        if (traversability.validity !=
            TraversabilityEvaluationValidity::valid) {
          result.valid_input = false;
          result.goals.clear();
          reject(result, MergeGoalRejectionReason::terrain_evaluation_invalid,
                 candidate,
                 "terminal terrain evaluation inputs or dependencies are invalid");
          return result;
        }
        if (!traversability.traversable) {
          reject(result,
                 MergeGoalRejectionReason::terminal_terrain_not_traversable,
                 candidate, "terminal robot pose is not terrain traversable");
          continue;
        }

        MergeGoalSoftCost soft_cost;
        soft_cost.merge_distance_component =
            generation_parameters_.merge_distance_cost_weight *
            merge_distance_m;
        soft_cost.absolute_lag_component =
            generation_parameters_.absolute_lag_cost_weight *
            std::abs(lag_angle_rad);
        soft_cost.total = soft_cost.merge_distance_component +
                          soft_cost.absolute_lag_component;
        result.goals.push_back(
            {inverse.robot_pose,
             lag_angle_rad,
             goal_progress_m,
             reference_line.version,
             touchdown_target,
             reference_heading_rad,
             merge_distance_m,
             soft_cost,
             generation_parameters_.version,
             cable_identity.version,
             robot_operating_area.version});
      }
    }
  }

  std::stable_sort(result.goals.begin(), result.goals.end(),
                   [](const MergeGoal& left, const MergeGoal& right) {
                     return left.soft_cost.total < right.soft_cost.total;
                   });
  if (result.goals.size() > generation_parameters_.maximum_goal_count) {
    const std::size_t removed_goal_count =
        result.goals.size() - generation_parameters_.maximum_goal_count;
    reject(result, MergeGoalRejectionReason::goal_limit_reached, {},
           std::to_string(removed_goal_count) +
               " feasible merge goals were removed by the configured limit");
    result.goals.resize(generation_parameters_.maximum_goal_count);
  }
  return result;
}

}  // namespace underwater_planner::core
