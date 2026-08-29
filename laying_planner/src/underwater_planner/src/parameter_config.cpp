#include "underwater_planner/core/parameter_config.hpp"
#include "underwater_planner/core/step_traversal_rules.hpp"
#include "underwater_planner/core/versioned_snapshot.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <charconv>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace underwater_planner::core {
namespace {

void issue(ParameterValidationResult& result, std::string code,
           std::string path, std::string message) {
  result.issues.push_back(
      ParameterIssue{std::move(code), std::move(path), std::move(message)});
}

void required(ParameterValidationResult& result, const std::string& path,
              const std::string& value) {
  if (value.empty()) {
    issue(result, "MISSING_REQUIRED_PARAMETER", path,
          "required version, identifier, or provenance is missing");
  }
}

void required(ParameterValidationResult& result, const std::string& path,
              const std::optional<double>& value) {
  if (!value.has_value()) {
    issue(result, "MISSING_REQUIRED_PARAMETER", path,
          "required calibrated physical parameter is missing");
  }
}

template <typename T>
void finite_positive(ParameterValidationResult& result, const std::string& path,
                     const std::optional<T>& value, bool allow_zero = false) {
  if (!value.has_value()) {
    return;
  }
  const double numeric = static_cast<double>(*value);
  if (!std::isfinite(numeric) || (allow_zero ? numeric < 0.0 : numeric <= 0.0)) {
    issue(result, "INVALID_NUMERIC_PARAMETER", path,
          allow_zero ? "value must be finite and non-negative"
                     : "value must be finite and positive");
  }
}

void required_positive(ParameterValidationResult& result, const std::string& path,
                       const std::optional<double>& value) {
  required(result, path, value);
  finite_positive(result, path, value);
}

void required_positive_uint(ParameterValidationResult& result,
                            const std::string& path,
                            const std::optional<std::uint64_t>& value) {
  if (!value.has_value() || *value == 0U) {
    issue(result, "MISSING_REQUIRED_PARAMETER", path,
          "required version or resource budget must be positive");
  }
}

void required_nonnegative(ParameterValidationResult& result,
                          const std::string& path,
                          const std::optional<double>& value) {
  required(result, path, value);
  finite_positive(result, path, value, true);
}

void required_finite(ParameterValidationResult& result, const std::string& path,
                     const std::optional<double>& value) {
  required(result, path, value);
  if (value.has_value() && !std::isfinite(*value)) {
    issue(result, "INVALID_NUMERIC_PARAMETER", path,
          "value must be finite");
  }
}

void finite_any(ParameterValidationResult& result, const char* path,
                const std::optional<double>& value) {
  if (value.has_value() && !std::isfinite(*value)) {
    issue(result, "INVALID_NUMERIC_PARAMETER", path, "value must be finite");
  }
}

void validate_common_finite(ParameterValidationResult& result,
                            const ParameterConfig& config) {
  const auto& r = config.robot;
  const std::pair<const char*, std::optional<double>> values[] = {
      {"robot.length_m", r.length_m},
      {"robot.width_m", r.width_m},
      {"robot.height_m", r.height_m},
      {"robot.minimum_turning_radius_m", r.minimum_turning_radius_m},
      {"robot.maximum_curvature_per_m", r.maximum_curvature_per_m},
      {"robot.maximum_curvature_rate_per_m2", r.maximum_curvature_rate_per_m2},
      {"robot.curvature_state_max_age_s", r.curvature_state_max_age_s},
      {"robot.minimum_ground_speed_mps", r.minimum_ground_speed_mps},
      {"robot.maximum_ground_speed_mps", r.maximum_ground_speed_mps},
      {"robot.maximum_acceleration_mps2", r.maximum_acceleration_mps2},
      {"robot.maximum_deceleration_mps2", r.maximum_deceleration_mps2},
      {"robot.maximum_lateral_acceleration_mps2", r.maximum_lateral_acceleration_mps2},
      {"robot.maximum_slope_up_rad", r.maximum_slope_up_rad},
      {"robot.maximum_slope_down_rad", r.maximum_slope_down_rad},
      {"robot.maximum_slope_lateral_rad", r.maximum_slope_lateral_rad},
      {"robot.maximum_support_roll_rad", r.maximum_support_roll_rad},
      {"robot.maximum_step_climb_m", r.maximum_step_climb_m},
      {"robot.maximum_step_drop_m", r.maximum_step_drop_m},
      {"robot.minimum_track_support_ratio", r.minimum_track_support_ratio},
      {"robot.maximum_roughness_m", r.maximum_roughness_m},
      {"robot.safe_obstacle_distance_m", r.safe_obstacle_distance_m},
      {"robot.effective_track_spacing_m", r.effective_track_spacing_m},
      {"robot.minimum_step_crossing_alignment",
       r.minimum_step_crossing_alignment},
      {"robot.step_alignment_transition_band",
       r.step_alignment_transition_band}};
  for (const auto& [path, value] : values) {
    finite_positive(result, path, value, true);
  }

  // Every optional numeric field is checked in every profile.  Production
  // requiredness is applied below; this pass prevents a non-production
  // capability profile from carrying NaN, infinity, or negative values.
  const auto nonnegative = [&](const char* path,
                               const std::optional<double>& value) {
    finite_positive(result, path, value, true);
  };
  const auto any_finite = [&](const char* path,
                              const std::optional<double>& value) {
    finite_any(result, path, value);
  };
  finite_positive(result, "terrain_gradient_risk.epsilon_local",
                  config.terrain_gradient_risk.epsilon_local);
  finite_positive(result, "terrain_gradient_risk.coverage_multiplier",
                  config.terrain_gradient_risk.coverage_multiplier);
  if (config.terrain_gradient_risk.epsilon_local.has_value() &&
      *config.terrain_gradient_risk.epsilon_local >= 1.0) {
    issue(result, "INVALID_RISK_PARAMETER", "terrain_gradient_risk.epsilon_local",
          "epsilon must be strictly between zero and one");
  }

  const auto& execution = config.execution;
  const std::pair<const char*, std::optional<double>> execution_values[] = {
      {"execution.minimum_ground_speed_mps", execution.minimum_ground_speed_mps},
      {"execution.maximum_ground_speed_mps", execution.maximum_ground_speed_mps},
      {"execution.maximum_acceleration_mps2", execution.maximum_acceleration_mps2},
      {"execution.maximum_deceleration_mps2", execution.maximum_deceleration_mps2},
      {"execution.minimum_payout_speed_mps", execution.minimum_payout_speed_mps},
      {"execution.maximum_payout_speed_mps", execution.maximum_payout_speed_mps},
      {"execution.maximum_payout_acceleration_mps2", execution.maximum_payout_acceleration_mps2},
      {"execution.minimum_tension_n", execution.minimum_tension_n},
      {"execution.maximum_tension_n", execution.maximum_tension_n},
      {"execution.sample_period_s", execution.sample_period_s},
      {"execution.terminal_speed_mps", execution.terminal_speed_mps},
      {"execution.stopping_distance_margin_m", execution.stopping_distance_margin_m},
      {"execution.ground_speed_tracking_error_mps", execution.ground_speed_tracking_error_mps},
      {"execution.ground_acceleration_tolerance_mps2", execution.ground_acceleration_tolerance_mps2},
      {"execution.payout_speed_tracking_error_mps", execution.payout_speed_tracking_error_mps},
      {"execution.tension_tracking_error_n", execution.tension_tracking_error_n},
      {"execution.position_join_tolerance_m", execution.position_join_tolerance_m},
      {"execution.heading_join_tolerance_rad", execution.heading_join_tolerance_rad},
      {"execution.curvature_join_tolerance_per_m", execution.curvature_join_tolerance_per_m}};
  for (const auto& [path, value] : execution_values) nonnegative(path, value);

  const auto& cable = config.cable;
  any_finite("cable.release_point_x_m", cable.release_point_x_m);
  any_finite("cable.release_point_y_m", cable.release_point_y_m);
  const std::pair<const char*, std::optional<double>> cable_values[] = {
      {"cable.touchdown_distance_m", cable.touchdown_distance_m},
      {"cable.direction_response_length_m", cable.direction_response_length_m},
      {"cable.maximum_lag_angle_rad", cable.maximum_lag_angle_rad},
      {"cable.maximum_payout_speed_tracking_error_mps", cable.maximum_payout_speed_tracking_error_mps},
      {"cable.minimum_payout_speed_mps", cable.minimum_payout_speed_mps},
      {"cable.maximum_payout_speed_mps", cable.maximum_payout_speed_mps},
      {"cable.maximum_payout_acceleration_mps2", cable.maximum_payout_acceleration_mps2},
      {"cable.maximum_tension_tracking_error_n", cable.maximum_tension_tracking_error_n},
      {"cable.minimum_tension_n", cable.minimum_tension_n},
      {"cable.maximum_tension_n", cable.maximum_tension_n},
      {"cable.touchdown_distance_variance_m2", cable.touchdown_distance_variance_m2},
      {"cable.direction_response_length_variance_m2", cable.direction_response_length_variance_m2},
      {"cable.lag_angle_process_variance_per_m_rad2", cable.lag_angle_process_variance_per_m_rad2},
      {"cable.manufacturer_minimum_bend_radius_m", cable.manufacturer_minimum_bend_radius_m},
      {"cable.preferred_curvature_per_m", cable.preferred_curvature_per_m},
      {"cable.maximum_curvature_per_m", cable.maximum_curvature_per_m},
      {"cable.curvature_evaluation_spacing_m", cable.curvature_evaluation_spacing_m},
      {"cable.support_evaluation_length_m", cable.support_evaluation_length_m},
      {"cable.medium_support_proxy_range_m", cable.medium_support_proxy_range_m},
      {"cable.maximum_support_proxy_range_m", cable.maximum_support_proxy_range_m},
      {"cable.minimum_terrain_confidence", cable.minimum_terrain_confidence}};
  for (const auto& [path, value] : cable_values) nonnegative(path, value);
  if (cable.minimum_terrain_confidence.has_value() &&
      (*cable.minimum_terrain_confidence <= 0.0 ||
       *cable.minimum_terrain_confidence > 1.0)) {
    issue(result, "INVALID_CABLE_PARAMETER",
          "cable.minimum_terrain_confidence",
          "minimum terrain confidence must be in (0,1]");
  }
  any_finite("cable.touchdown_process_noise_xy_m2_per_m",
             cable.touchdown_process_noise_per_m_m2.xy_m2);
  nonnegative("cable.touchdown_process_noise_xx_m2_per_m",
              cable.touchdown_process_noise_per_m_m2.xx_m2);
  nonnegative("cable.touchdown_process_noise_yy_m2_per_m",
              cable.touchdown_process_noise_per_m_m2.yy_m2);

  const auto& risk = config.statistical_risk;
  const std::pair<const char*, std::optional<double>> risk_values[] = {
      {"statistical_risk.epsilon_point", risk.epsilon_point},
      {"statistical_risk.nominal_corridor_width_m", risk.nominal_corridor_width_m},
      {"statistical_risk.absolute_corridor_width_m", risk.absolute_corridor_width_m},
      {"statistical_risk.maximum_marginal_length_m", risk.maximum_marginal_length_m},
      {"statistical_risk.maximum_candidate_length_m", risk.maximum_candidate_length_m},
      {"statistical_risk.maximum_planning_duration_s", risk.maximum_planning_duration_s},
      {"statistical_risk.progress_resolution_m", risk.progress_resolution_m},
      {"statistical_risk.envelope_discretization_margin_m", risk.envelope_discretization_margin_m},
      {"statistical_risk.envelope_audit_tolerance_m", risk.envelope_audit_tolerance_m}};
  for (const auto& [path, value] : risk_values) nonnegative(path, value);
  finite_positive(result, "statistical_risk.epsilon_point", risk.epsilon_point);
  if (risk.epsilon_point.has_value() && *risk.epsilon_point >= 1.0) {
    issue(result, "INVALID_RISK_PARAMETER", "statistical_risk.epsilon_point",
          "epsilon must be strictly between zero and one");
  }

  const auto& reuse = config.path_reuse;
  const std::pair<const char*, std::optional<double>> reuse_values[] = {
      {"path_reuse.reuse_max_s", reuse.reuse_max_s},
      {"path_reuse.robot_state_max_age_s", reuse.robot_state_max_age_s},
      {"path_reuse.cable_state_max_age_s", reuse.cable_state_max_age_s},
      {"path_reuse.cable_telemetry_max_age_s", reuse.cable_telemetry_max_age_s},
      {"path_reuse.execution_tracking_max_age_s", reuse.execution_tracking_max_age_s},
      {"path_reuse.lease_monitor_period_s", reuse.lease_monitor_period_s},
      {"path_reuse.lease_renewal_margin_s", reuse.lease_renewal_margin_s}};
  for (const auto& [path, value] : reuse_values) nonnegative(path, value);

  const auto& search = config.search;
  const std::pair<const char*, std::optional<double>> search_values[] = {
      {"search.equivalent_label_cost_tolerance_m", search.equivalent_label_cost_tolerance_m},
      {"search.xy_resolution_m", search.xy_resolution_m},
      {"search.heading_resolution_rad", search.heading_resolution_rad},
      {"search.cable_lag_resolution_rad", search.cable_lag_resolution_rad},
      {"search.reference_progress_resolution_m", search.reference_progress_resolution_m},
      {"search.reference_progress_backward_tolerance_m", search.reference_progress_backward_tolerance_m},
      {"search.reference_progress_maximum_ratio", search.reference_progress_maximum_ratio},
      {"search.reference_progress_forward_slack_m", search.reference_progress_forward_slack_m},
      {"search.reference_progress_distance_scale_m", search.reference_progress_distance_scale_m},
      {"search.reference_progress_heading_scale_rad", search.reference_progress_heading_scale_rad},
      {"search.reference_progress_heading_weight", search.reference_progress_heading_weight},
      {"search.reference_progress_association_score_tolerance", search.reference_progress_association_score_tolerance},
      {"search.collision_sweep_margin_m", search.collision_sweep_margin_m},
      {"search.cable_sweep_margin_m", search.cable_sweep_margin_m},
      {"search.path_length_cost_weight", search.path_length_cost_weight},
      {"search.path_curvature_cost_weight", search.path_curvature_cost_weight},
      {"search.touchdown_center_cost_weight", search.touchdown_center_cost_weight},
      {"search.touchdown_margin_cost_weight", search.touchdown_margin_cost_weight},
      {"search.robot_terrain_cost_weight", search.robot_terrain_cost_weight}};
  for (const auto& [path, value] : search_values) nonnegative(path, value);

  nonnegative("task.laying_success_ratio_target", config.task.laying_success_ratio_target);
  if (config.task.laying_success_ratio_target.has_value() &&
      *config.task.laying_success_ratio_target > 1.0) {
    issue(result, "INVALID_TASK_PARAMETER", "task.laying_success_ratio_target",
          "laying success ratio target must be in [0,1]");
  }
  if (r.minimum_track_support_ratio.has_value() &&
      *r.minimum_track_support_ratio > 1.0) {
    issue(result, "INVALID_CAPABILITY_PARAMETER", "robot.minimum_track_support_ratio",
          "support ratio must be in [0,1]");
  }
  if (r.minimum_step_crossing_alignment.has_value() &&
      r.step_alignment_transition_band.has_value() &&
      *r.minimum_step_crossing_alignment > 0.0 &&
      *r.step_alignment_transition_band > 0.0 &&
      !valid_step_alignment_domain(*r.minimum_step_crossing_alignment,
                                   *r.step_alignment_transition_band)) {
    issue(result, "INCONSISTENT_CAPABILITY_PARAMETER",
          "robot.step_alignment_transition_band",
          "transition band must stay within the normalized alignment domain");
  }
  if (r.minimum_ground_speed_mps.has_value() &&
      r.maximum_ground_speed_mps.has_value() &&
      *r.minimum_ground_speed_mps > *r.maximum_ground_speed_mps) {
    issue(result, "INCONSISTENT_CAPABILITY_PARAMETER", "robot.ground_speed",
          "minimum speed exceeds maximum speed");
  }
  if (r.maximum_curvature_per_m.has_value() &&
      r.minimum_turning_radius_m.has_value() &&
      *r.maximum_curvature_per_m > 1.0 / *r.minimum_turning_radius_m + 1e-12) {
    issue(result, "INCONSISTENT_CAPABILITY_PARAMETER", "robot.maximum_curvature_per_m",
          "maximum curvature exceeds the calibrated turning-radius bound");
  }
  finite_positive(result, "robot_collision_risk.epsilon_robot",
                  config.robot_collision_risk.epsilon_robot);
  finite_positive(result, "robot_collision_risk.minimum_map_confidence",
                  config.robot_collision_risk.minimum_map_confidence);
  if (config.robot_collision_risk.epsilon_robot.has_value() &&
      *config.robot_collision_risk.epsilon_robot >= 0.5) {
    issue(result, "INVALID_RISK_PARAMETER", "robot_collision_risk.epsilon_robot",
          "one-sided collision epsilon must be strictly between zero and 0.5");
  }
  if (config.robot_collision_risk.minimum_map_confidence.has_value() &&
      *config.robot_collision_risk.minimum_map_confidence > 1.0) {
    issue(result, "INVALID_RISK_PARAMETER",
          "robot_collision_risk.minimum_map_confidence",
          "minimum map confidence must be in (0,1]");
  }
  const TaskParameterConfig& task = config.task;
  const std::pair<const char*, std::optional<double>> scout_positive[] = {
      {"task.scout_minimum_map_confidence",
       task.scout_minimum_map_confidence},
      {"task.scout_sample_interval_m", task.scout_sample_interval_m},
      {"task.scout_minimum_safe_distance_m",
       task.scout_minimum_safe_distance_m},
      {"task.scout_planning_lead_time_s", task.scout_planning_lead_time_s},
      {"task.scout_average_velocity_mps", task.scout_average_velocity_mps},
      {"task.scout_sensor_coverage_radius_m",
       task.scout_sensor_coverage_radius_m},
      {"task.scout_corridor_half_width_m",
       task.scout_corridor_half_width_m},
      {"task.communication_max_m", task.communication_max_m},
      {"task.scout_desired_distance_m", task.scout_desired_distance_m},
      {"task.scout_continue_distance_m", task.scout_continue_distance_m},
      {"task.scout_stop_distance_m", task.scout_stop_distance_m},
      {"task.scout_request_timeout_s", task.scout_request_timeout_s}};
  for (const auto& [path, value] : scout_positive) {
    finite_positive(result, path, value);
  }
  const std::pair<const char*, std::optional<double>> scout_nonnegative[] = {
      {"task.scout_merge_distance_m", task.scout_merge_distance_m},
      {"task.scout_urgency_hysteresis_distance_m",
       task.scout_urgency_hysteresis_distance_m},
      {"task.scout_urgency_hysteresis_time_s",
       task.scout_urgency_hysteresis_time_s},
      {"task.scout_blocking_priority_weight",
       task.scout_blocking_priority_weight},
      {"task.scout_information_value_weight",
       task.scout_information_value_weight},
      {"task.scout_forward_progress_weight",
       task.scout_forward_progress_weight},
      {"task.scout_arrival_cost_weight", task.scout_arrival_cost_weight}};
  for (const auto& [path, value] : scout_nonnegative) {
    finite_positive(result, path, value, true);
  }
  if (task.scout_minimum_map_confidence.has_value() &&
      *task.scout_minimum_map_confidence > 1.0) {
    issue(result, "INVALID_SCOUT_PARAMETER",
          "task.scout_minimum_map_confidence",
          "scout map confidence must be in (0,1]");
  }
  if (task.scout_request_timeout_s.has_value()) {
    const long double nanoseconds =
        static_cast<long double>(*task.scout_request_timeout_s) * 1.0e9L;
    constexpr std::int64_t kDurationConversionMarginNanoseconds =
        1'000'000'000;
    const long double maximum_safe_nanoseconds = static_cast<long double>(
        std::numeric_limits<std::int64_t>::max() -
        kDurationConversionMarginNanoseconds);
    if (nanoseconds < 1.0L ||
        nanoseconds > maximum_safe_nanoseconds) {
      issue(result, "INVALID_SCOUT_PARAMETER", "task.scout_request_timeout_s",
            "scout request timeout is not representable in nanoseconds");
    }
  }
}

void validate_production(ParameterValidationResult& result,
                         const ParameterConfig& config) {
  const auto& r = config.robot;
  required(result, "robot.calibration_version", r.calibration_version);
  required(result, "robot.calibration_dataset_id", r.calibration_dataset_id);
  const std::pair<const char*, std::optional<double>> robot_required[] = {
      {"robot.length_m", r.length_m},
      {"robot.width_m", r.width_m},
      {"robot.height_m", r.height_m},
      {"robot.minimum_turning_radius_m", r.minimum_turning_radius_m},
      {"robot.maximum_slope_up_rad", r.maximum_slope_up_rad},
      {"robot.maximum_slope_down_rad", r.maximum_slope_down_rad},
      {"robot.maximum_slope_lateral_rad", r.maximum_slope_lateral_rad},
      {"robot.maximum_support_roll_rad", r.maximum_support_roll_rad},
      {"robot.maximum_step_climb_m", r.maximum_step_climb_m},
      {"robot.maximum_step_drop_m", r.maximum_step_drop_m},
      {"robot.minimum_track_support_ratio", r.minimum_track_support_ratio},
      {"robot.effective_track_spacing_m", r.effective_track_spacing_m},
      {"robot.minimum_step_crossing_alignment",
       r.minimum_step_crossing_alignment},
      {"robot.step_alignment_transition_band",
       r.step_alignment_transition_band},
      {"robot.maximum_curvature_per_m", r.maximum_curvature_per_m},
      {"robot.maximum_curvature_rate_per_m2", r.maximum_curvature_rate_per_m2},
      {"robot.curvature_state_max_age_s", r.curvature_state_max_age_s},
      {"robot.minimum_ground_speed_mps", r.minimum_ground_speed_mps},
      {"robot.maximum_ground_speed_mps", r.maximum_ground_speed_mps},
      {"robot.maximum_acceleration_mps2", r.maximum_acceleration_mps2},
      {"robot.maximum_deceleration_mps2", r.maximum_deceleration_mps2},
      {"robot.maximum_lateral_acceleration_mps2", r.maximum_lateral_acceleration_mps2},
      {"robot.safe_obstacle_distance_m", r.safe_obstacle_distance_m},
      {"execution.stopping_distance_margin_m", config.execution.stopping_distance_margin_m},
      {"execution.ground_speed_tracking_error_mps", config.execution.ground_speed_tracking_error_mps},
      {"execution.ground_acceleration_tolerance_mps2", config.execution.ground_acceleration_tolerance_mps2},
      {"execution.payout_speed_tracking_error_mps", config.execution.payout_speed_tracking_error_mps},
      {"execution.tension_tracking_error_n", config.execution.tension_tracking_error_n}};
  for (const auto& [path, value] : robot_required) {
    required_positive(result, path, value);
  }
  // Roughness is a robot hard capability and may legitimately be zero; it is
  // therefore required separately from the strictly-positive capabilities.
  required_nonnegative(result, "robot.maximum_roughness_m",
                       r.maximum_roughness_m);
  if (!r.left_track_support_defined || !r.right_track_support_defined) {
    issue(result, "MISSING_ROBOT_GEOMETRY", "robot.track_support",
          "left and right effective support polygons are independently required");
  }
  if (!r.localization_covariance_defined || !r.control_tracking_covariance_defined) {
    issue(result, "MISSING_ROBOT_UNCERTAINTY", "robot.uncertainty",
          "localization and control tracking covariance are required");
  }

  const auto& terrain = config.terrain_gradient_risk;
  required(result, "terrain_gradient_risk.policy_version", terrain.policy_version);
  required(result, "terrain_gradient_risk.terrain_analysis_config_version",
           terrain.terrain_analysis_config_version);
  required(result, "terrain_gradient_risk.calibration_dataset_id",
           terrain.calibration_dataset_id);
  required(result, "terrain_gradient_risk.operating_domain_id",
           terrain.operating_domain_id);
  required_positive(result, "terrain_gradient_risk.epsilon_local", terrain.epsilon_local);
  required_positive(result, "terrain_gradient_risk.coverage_multiplier",
                    terrain.coverage_multiplier);
  if (terrain.operating_domain_id != config.operating_domain_id) {
    issue(result, "VERSION_DOMAIN_MISMATCH",
          "terrain_gradient_risk.operating_domain_id",
          "terrain gradient risk domain does not match the configuration domain");
  }
  if (terrain.coverage_model == "gaussian_pending_calibration" ||
      terrain.coverage_model == "empirical_pending_calibration" ||
      terrain.coverage_model == "deterministic_pending_calibration" ||
      terrain.coverage_model.empty()) {
    issue(result, "RISK_MODEL_NOT_CALIBRATED", "terrain_gradient_risk.coverage_model",
          "production requires a calibrated terrain coverage model");
  } else if (terrain.coverage_model != "calibrated_gaussian" &&
             terrain.coverage_model != "empirical_bounded" &&
             terrain.coverage_model != "deterministic_bounded") {
    issue(result, "UNSUPPORTED_RISK_MODEL", "terrain_gradient_risk.coverage_model",
          "coverage model is not a supported calibrated model");
  }
  const auto version_tag = [](const std::string& value) -> std::string_view {
    const std::size_t marker = value.rfind("-v");
    return marker == std::string::npos ? std::string_view{} :
                                         std::string_view(value).substr(marker);
  };
  const std::string_view policy_tag = version_tag(terrain.policy_version);
  const std::string_view analysis_tag =
      version_tag(terrain.terrain_analysis_config_version);
  const std::string_view dataset_tag = version_tag(terrain.calibration_dataset_id);
  if (!policy_tag.empty() && !analysis_tag.empty() && !dataset_tag.empty() &&
      (policy_tag != analysis_tag || policy_tag != dataset_tag)) {
    issue(result, "VERSION_DEPENDENCY_MISMATCH",
          "terrain_gradient_risk.calibration_dataset_id",
          "coverage policy, terrain analysis config, and calibration dataset versions must match");
  }
  if (terrain.epsilon_local.has_value() && *terrain.epsilon_local >= 1.0) {
    issue(result, "INVALID_RISK_PARAMETER", "terrain_gradient_risk.epsilon_local",
          "epsilon must be strictly between zero and one");
  }

  const auto& collision = config.robot_collision_risk;
  required(result, "robot_collision_risk.policy_version",
           collision.policy_version);
  required(result, "robot_collision_risk.calibration_dataset_id",
           collision.calibration_dataset_id);
  required(result, "robot_collision_risk.operating_domain_id",
           collision.operating_domain_id);
  required_positive(result, "robot_collision_risk.epsilon_robot",
                    collision.epsilon_robot);
  required_positive(result, "robot_collision_risk.minimum_map_confidence",
                    collision.minimum_map_confidence);
  if (collision.epsilon_robot.has_value() &&
      *collision.epsilon_robot >= 0.5) {
    issue(result, "INVALID_RISK_PARAMETER",
          "robot_collision_risk.epsilon_robot",
          "one-sided collision epsilon must be strictly between zero and 0.5");
  }
  if (collision.minimum_map_confidence.has_value() &&
      *collision.minimum_map_confidence > 1.0) {
    issue(result, "INVALID_RISK_PARAMETER",
          "robot_collision_risk.minimum_map_confidence",
          "minimum map confidence must be in (0,1]");
  }
  if (collision.operating_domain_id != config.operating_domain_id) {
    issue(result, "VERSION_DOMAIN_MISMATCH",
          "robot_collision_risk.operating_domain_id",
          "collision risk policy domain does not match the configuration domain");
  }

  const auto& domains = config.spatial_domains;
  required(result, "spatial_domains.robot_operating_area_id",
           domains.robot_operating_area_id);
  required(result, "spatial_domains.robot_operating_area_version",
           domains.robot_operating_area_version);
  required(result, "spatial_domains.cable_corridor_id", domains.cable_corridor_id);
  required(result, "spatial_domains.cable_corridor_version",
           domains.cable_corridor_version);
  if (!domains.robot_operating_area_non_empty || !domains.cable_corridor_non_empty) {
    issue(result, "EMPTY_SPATIAL_DOMAIN", "spatial_domains",
          "robot operating area and cable corridor must both be non-empty");
  }
  if (domains.robot_operating_area_id == domains.cable_corridor_id) {
    issue(result, "SPATIAL_DOMAIN_SEMANTICS_SHARED", "spatial_domains",
          "robot and cable domains must not share one implicit field");
  }

  const auto& execution = config.execution;
  required(result, "execution.operating_envelope_version",
           execution.operating_envelope_version);
  required(result, "execution.operating_domain_id", execution.operating_domain_id);
  required_positive(result, "execution.minimum_ground_speed_mps", execution.minimum_ground_speed_mps);
  required_positive(result, "execution.maximum_ground_speed_mps", execution.maximum_ground_speed_mps);
  required_positive(result, "execution.maximum_acceleration_mps2", execution.maximum_acceleration_mps2);
  required_positive(result, "execution.maximum_deceleration_mps2", execution.maximum_deceleration_mps2);
  required_nonnegative(result, "execution.minimum_payout_speed_mps", execution.minimum_payout_speed_mps);
  required_positive(result, "execution.maximum_payout_speed_mps", execution.maximum_payout_speed_mps);
  required_positive(result, "execution.maximum_payout_acceleration_mps2", execution.maximum_payout_acceleration_mps2);
  required_positive(result, "execution.minimum_tension_n", execution.minimum_tension_n);
  required_positive(result, "execution.maximum_tension_n", execution.maximum_tension_n);
  required_positive(result, "execution.sample_period_s", execution.sample_period_s);
  required_nonnegative(result, "execution.terminal_speed_mps",
                       execution.terminal_speed_mps);
  required_positive(result, "execution.position_join_tolerance_m", execution.position_join_tolerance_m);
  required_positive(result, "execution.heading_join_tolerance_rad", execution.heading_join_tolerance_rad);
  required_positive(result, "execution.curvature_join_tolerance_per_m", execution.curvature_join_tolerance_per_m);
  if (execution.operating_domain_id != config.operating_domain_id) {
    issue(result, "VERSION_DOMAIN_MISMATCH", "execution.operating_domain_id",
          "execution operating domain does not match the configuration domain");
  }

  const auto& cable = config.cable;
  required(result, "cable.model_version", cable.model_version);
  required(result, "cable.calibration_dataset_id", cable.calibration_dataset_id);
  const std::pair<const char*, std::optional<double>> cable_required[] = {
      {"cable.touchdown_distance_m", cable.touchdown_distance_m},
      {"cable.direction_response_length_m", cable.direction_response_length_m},
      {"cable.maximum_lag_angle_rad", cable.maximum_lag_angle_rad},
      {"cable.maximum_payout_speed_tracking_error_mps", cable.maximum_payout_speed_tracking_error_mps},
      {"cable.minimum_payout_speed_mps", cable.minimum_payout_speed_mps},
      {"cable.maximum_payout_speed_mps", cable.maximum_payout_speed_mps},
      {"cable.maximum_payout_acceleration_mps2", cable.maximum_payout_acceleration_mps2},
      {"cable.maximum_tension_tracking_error_n", cable.maximum_tension_tracking_error_n},
      {"cable.minimum_tension_n", cable.minimum_tension_n},
      {"cable.maximum_tension_n", cable.maximum_tension_n},
      {"cable.manufacturer_minimum_bend_radius_m", cable.manufacturer_minimum_bend_radius_m},
      {"cable.preferred_curvature_per_m", cable.preferred_curvature_per_m},
      {"cable.maximum_curvature_per_m", cable.maximum_curvature_per_m},
      {"cable.curvature_evaluation_spacing_m",
       cable.curvature_evaluation_spacing_m},
      {"cable.support_evaluation_length_m", cable.support_evaluation_length_m},
      {"cable.medium_support_proxy_range_m",
       cable.medium_support_proxy_range_m},
      {"cable.maximum_support_proxy_range_m",
       cable.maximum_support_proxy_range_m},
      {"cable.minimum_terrain_confidence",
       cable.minimum_terrain_confidence}};
  required_finite(result, "cable.release_point_x_m", cable.release_point_x_m);
  required_finite(result, "cable.release_point_y_m", cable.release_point_y_m);
  for (const auto& [path, value] : cable_required) {
    if (std::string_view(path) == "cable.minimum_payout_speed_mps") {
      required_nonnegative(result, path, value);
    } else {
      required_positive(result, path, value);
    }
  }
  required_positive(result, "cable.touchdown_distance_variance_m2",
                    cable.touchdown_distance_variance_m2);
  required_positive(result, "cable.direction_response_length_variance_m2",
                    cable.direction_response_length_variance_m2);
  required_positive(result,
                    "cable.lag_angle_process_variance_per_m_rad2",
                    cable.lag_angle_process_variance_per_m_rad2);
  required_nonnegative(result,
                       "cable.touchdown_process_noise_xx_m2_per_m",
                       cable.touchdown_process_noise_per_m_m2.xx_m2);
  required_finite(result, "cable.touchdown_process_noise_xy_m2_per_m",
                  cable.touchdown_process_noise_per_m_m2.xy_m2);
  required_nonnegative(result,
                       "cable.touchdown_process_noise_yy_m2_per_m",
                       cable.touchdown_process_noise_per_m_m2.yy_m2);
  const auto& process_noise = cable.touchdown_process_noise_per_m_m2;
  if (process_noise.xx_m2.has_value() && process_noise.xy_m2.has_value() &&
      process_noise.yy_m2.has_value()) {
    const double xx = *process_noise.xx_m2;
    const double xy = *process_noise.xy_m2;
    const double yy = *process_noise.yy_m2;
    if (std::isfinite(xx) && std::isfinite(xy) && std::isfinite(yy) &&
        xx >= 0.0 && yy >= 0.0 &&
        (xx + yy <= 0.0 ||
         std::abs(xy) > std::sqrt(xx) * std::sqrt(yy))) {
      issue(result, "INVALID_COVARIANCE_PARAMETER",
            "cable.touchdown_process_noise_m2_per_m",
            "touchdown process noise must be nonzero and positive semidefinite");
    }
  }
  required(result, "cable.forbidden_area_layer", cable.forbidden_area_layer);
  if (cable.preferred_curvature_per_m.has_value() && cable.maximum_curvature_per_m.has_value() &&
      *cable.preferred_curvature_per_m >= *cable.maximum_curvature_per_m) {
    issue(result, "INCONSISTENT_CABLE_PARAMETER", "cable.preferred_curvature_per_m",
          "preferred curvature must be lower than the hard maximum");
  }
  if (cable.maximum_curvature_per_m.has_value() && cable.manufacturer_minimum_bend_radius_m.has_value() &&
      *cable.maximum_curvature_per_m > 1.0 / *cable.manufacturer_minimum_bend_radius_m + 1e-12) {
    issue(result, "INCONSISTENT_CABLE_PARAMETER", "cable.maximum_curvature_per_m",
          "hard curvature exceeds manufacturer bend-radius bound");
  }
  if (cable.medium_support_proxy_range_m.has_value() &&
      cable.maximum_support_proxy_range_m.has_value() &&
      *cable.medium_support_proxy_range_m >=
          *cable.maximum_support_proxy_range_m) {
    issue(result, "INCONSISTENT_CABLE_PARAMETER",
          "cable.medium_support_proxy_range_m",
          "medium support threshold must be below the hard maximum");
  }
  if (cable.minimum_terrain_confidence.has_value() &&
      *cable.minimum_terrain_confidence > 1.0) {
    issue(result, "INVALID_CABLE_PARAMETER",
          "cable.minimum_terrain_confidence",
          "minimum terrain confidence must be in (0,1]");
  }

  const auto& risk = config.statistical_risk;
  required(result, "statistical_risk.policy_version", risk.policy_version);
  required(result, "statistical_risk.calibration_dataset_id", risk.calibration_dataset_id);
  required(result, "statistical_risk.uncertainty_envelope_version",
           risk.uncertainty_envelope_version);
  required(result, "statistical_risk.envelope_generator_version",
           risk.envelope_generator_version);
  required(result, "statistical_risk.envelope_operating_domain_id",
           risk.envelope_operating_domain_id);
  required(result, "statistical_risk.envelope_execution_version",
           risk.envelope_execution_version);
  required_positive(result, "statistical_risk.epsilon_point", risk.epsilon_point);
  required_positive(result, "statistical_risk.nominal_corridor_width_m", risk.nominal_corridor_width_m);
  required_positive(result, "statistical_risk.absolute_corridor_width_m", risk.absolute_corridor_width_m);
  required_positive(result, "statistical_risk.maximum_marginal_length_m", risk.maximum_marginal_length_m);
  required_positive(result, "statistical_risk.maximum_candidate_length_m", risk.maximum_candidate_length_m);
  required_positive(result, "statistical_risk.maximum_planning_duration_s", risk.maximum_planning_duration_s);
  required_positive(result, "statistical_risk.progress_resolution_m", risk.progress_resolution_m);
  required_positive(result, "statistical_risk.envelope_discretization_margin_m", risk.envelope_discretization_margin_m);
  required_positive(result, "statistical_risk.envelope_audit_tolerance_m", risk.envelope_audit_tolerance_m);
  if (risk.sensor_health_modes.empty()) {
    issue(result, "MISSING_SENSOR_HEALTH_MODE",
          "statistical_risk.sensor_health_mode",
          "production requires at least one supported sensor health mode");
  }
  const std::unordered_set<std::string> supported_sensor_modes = {
      "imu", "depth", "odometry", "localization"};
  std::unordered_set<std::string> seen_sensor_modes;
  for (const auto& sensor_mode : risk.sensor_health_modes) {
    if (supported_sensor_modes.find(sensor_mode) == supported_sensor_modes.end()) {
      issue(result, "UNSUPPORTED_SENSOR_HEALTH_MODE",
            "statistical_risk.sensor_health_mode",
            "sensor health mode is not recognized by this build");
    }
    if (!seen_sensor_modes.insert(sensor_mode).second) {
      issue(result, "DUPLICATE_SENSOR_HEALTH_MODE",
            "statistical_risk.sensor_health_mode",
            "sensor health modes must be unique");
    }
  }
  if (!risk.distribution_calibrated) {
    issue(result, "RISK_DISTRIBUTION_NOT_CALIBRATED", "statistical_risk.distribution_calibrated",
          "pointwise corridor risk cannot be declared valid without independent coverage calibration");
  }
  if (risk.epsilon_point.has_value() && *risk.epsilon_point >= 1.0) {
    issue(result, "INVALID_RISK_PARAMETER", "statistical_risk.epsilon_point",
          "epsilon must be strictly between zero and one");
  }
  if (risk.nominal_corridor_width_m.has_value() && risk.absolute_corridor_width_m.has_value() &&
      *risk.nominal_corridor_width_m > *risk.absolute_corridor_width_m) {
    issue(result, "INCONSISTENT_RISK_PARAMETER", "statistical_risk.corridor_width",
          "nominal corridor width exceeds absolute hard width");
  }
  if (risk.envelope_operating_domain_id != config.operating_domain_id) {
    issue(result, "VERSION_DOMAIN_MISMATCH", "statistical_risk.envelope_operating_domain_id",
          "uncertainty envelope domain does not match the configuration domain");
  }
  if (risk.envelope_execution_version != execution.operating_envelope_version) {
    issue(result, "VERSION_DEPENDENCY_MISMATCH", "statistical_risk.envelope_execution_version",
          "uncertainty envelope is bound to a different execution envelope");
  }

  const auto& reuse = config.path_reuse;
  required_positive(result, "path_reuse.reuse_max_s", reuse.reuse_max_s);
  required_positive(result, "path_reuse.robot_state_max_age_s", reuse.robot_state_max_age_s);
  required_positive(result, "path_reuse.cable_state_max_age_s", reuse.cable_state_max_age_s);
  required_positive(result, "path_reuse.cable_telemetry_max_age_s", reuse.cable_telemetry_max_age_s);
  required_positive(result, "path_reuse.execution_tracking_max_age_s", reuse.execution_tracking_max_age_s);
  required_positive(result, "path_reuse.lease_monitor_period_s", reuse.lease_monitor_period_s);
  required_positive(result, "path_reuse.lease_renewal_margin_s", reuse.lease_renewal_margin_s);
  if (reuse.lease_monitor_period_s.has_value() && reuse.reuse_max_s.has_value() &&
      *reuse.lease_monitor_period_s >= *reuse.reuse_max_s) {
    issue(result, "INVALID_REUSE_PARAMETER", "path_reuse.lease_monitor_period_s",
          "monitor period must be shorter than reuse lease");
  }
  if (reuse.lease_renewal_margin_s.has_value() && reuse.reuse_max_s.has_value() &&
      *reuse.lease_renewal_margin_s >= *reuse.reuse_max_s) {
    issue(result, "INVALID_REUSE_PARAMETER", "path_reuse.lease_renewal_margin_s",
          "renewal margin must be shorter than reuse lease");
  }

  required_positive_uint(result, "search.maximum_active_labels", config.search.maximum_active_labels);
  required_nonnegative(result, "search.equivalent_label_cost_tolerance_m",
                       config.search.equivalent_label_cost_tolerance_m);
  required_positive(result, "search.xy_resolution_m", config.search.xy_resolution_m);
  required_positive(result, "search.heading_resolution_rad", config.search.heading_resolution_rad);
  required_positive(result, "search.cable_lag_resolution_rad", config.search.cable_lag_resolution_rad);
  required_positive(result, "search.reference_progress_resolution_m", config.search.reference_progress_resolution_m);
  required_nonnegative(result, "search.reference_progress_backward_tolerance_m",
                       config.search.reference_progress_backward_tolerance_m);
  required_positive(result, "search.reference_progress_maximum_ratio",
                    config.search.reference_progress_maximum_ratio);
  required_nonnegative(result, "search.reference_progress_forward_slack_m",
                       config.search.reference_progress_forward_slack_m);
  required_positive(result, "search.reference_progress_distance_scale_m",
                    config.search.reference_progress_distance_scale_m);
  required_positive(result, "search.reference_progress_heading_scale_rad",
                    config.search.reference_progress_heading_scale_rad);
  required_positive(result, "search.reference_progress_heading_weight",
                    config.search.reference_progress_heading_weight);
  required_nonnegative(
      result, "search.reference_progress_association_score_tolerance",
      config.search.reference_progress_association_score_tolerance);
  required_positive(result, "search.collision_sweep_margin_m", config.search.collision_sweep_margin_m);
  required_positive(result, "search.cable_sweep_margin_m", config.search.cable_sweep_margin_m);
  required_positive(result, "search.path_length_cost_weight",
                    config.search.path_length_cost_weight);
  required_nonnegative(result, "search.path_curvature_cost_weight",
                       config.search.path_curvature_cost_weight);
  required_nonnegative(result, "search.touchdown_center_cost_weight",
                       config.search.touchdown_center_cost_weight);
  required_nonnegative(result, "search.touchdown_margin_cost_weight",
                       config.search.touchdown_margin_cost_weight);
  required_nonnegative(result, "search.robot_terrain_cost_weight",
                       config.search.robot_terrain_cost_weight);

  const TaskParameterConfig& task = config.task;
  required(result, "task.laying_success_ratio_target",
           task.laying_success_ratio_target);
  if (task.laying_success_ratio_target.has_value() &&
      (*task.laying_success_ratio_target < 0.0 ||
       *task.laying_success_ratio_target > 1.0)) {
    issue(result, "INVALID_TASK_PARAMETER", "task.laying_success_ratio_target",
          "laying success ratio target must be in [0,1]");
  }
  if (task.laying_success_ratio_target.has_value() &&
      *task.laying_success_ratio_target < 0.8) {
    issue(result, "INVALID_TASK_PARAMETER", "task.laying_success_ratio_target",
          "production laying success ratio target must be at least 0.8");
  }
  required_positive_uint(result, "task.scout_policy_version",
                         task.scout_policy_version);
  const std::pair<const char*, std::optional<double>> scout_required_positive[] = {
      {"task.scout_minimum_map_confidence",
       task.scout_minimum_map_confidence},
      {"task.scout_sample_interval_m", task.scout_sample_interval_m},
      {"task.scout_minimum_safe_distance_m",
       task.scout_minimum_safe_distance_m},
      {"task.scout_planning_lead_time_s", task.scout_planning_lead_time_s},
      {"task.scout_average_velocity_mps", task.scout_average_velocity_mps},
      {"task.scout_sensor_coverage_radius_m",
       task.scout_sensor_coverage_radius_m},
      {"task.scout_corridor_half_width_m",
       task.scout_corridor_half_width_m},
      {"task.communication_max_m", task.communication_max_m},
      {"task.scout_desired_distance_m", task.scout_desired_distance_m},
      {"task.scout_continue_distance_m", task.scout_continue_distance_m},
      {"task.scout_stop_distance_m", task.scout_stop_distance_m},
      {"task.scout_request_timeout_s", task.scout_request_timeout_s}};
  for (const auto& [path, value] : scout_required_positive) {
    required_positive(result, path, value);
  }
  const std::pair<const char*, std::optional<double>> scout_required_nonnegative[] = {
      {"task.scout_merge_distance_m", task.scout_merge_distance_m},
      {"task.scout_urgency_hysteresis_distance_m",
       task.scout_urgency_hysteresis_distance_m},
      {"task.scout_urgency_hysteresis_time_s",
       task.scout_urgency_hysteresis_time_s},
      {"task.scout_blocking_priority_weight",
       task.scout_blocking_priority_weight},
      {"task.scout_information_value_weight",
       task.scout_information_value_weight},
      {"task.scout_forward_progress_weight",
       task.scout_forward_progress_weight},
      {"task.scout_arrival_cost_weight", task.scout_arrival_cost_weight}};
  for (const auto& [path, value] : scout_required_nonnegative) {
    required_nonnegative(result, path, value);
  }
  if (task.scout_urgency_hysteresis_distance_m.has_value() &&
      task.scout_minimum_safe_distance_m.has_value() &&
      *task.scout_urgency_hysteresis_distance_m >=
          *task.scout_minimum_safe_distance_m) {
    issue(result, "INCONSISTENT_SCOUT_URGENCY_POLICY",
          "task.scout_urgency_hysteresis_distance_m",
          "urgency distance hysteresis must be below the safe-distance threshold");
  }
  if (task.scout_urgency_hysteresis_time_s.has_value() &&
      task.scout_planning_lead_time_s.has_value() &&
      *task.scout_urgency_hysteresis_time_s >=
          *task.scout_planning_lead_time_s) {
    issue(result, "INCONSISTENT_SCOUT_URGENCY_POLICY",
          "task.scout_urgency_hysteresis_time_s",
          "urgency time hysteresis must be below the planning lead time");
  }
  if (task.scout_desired_distance_m.has_value() &&
      task.scout_continue_distance_m.has_value() &&
      task.scout_stop_distance_m.has_value() &&
      task.communication_max_m.has_value() &&
      !(*task.scout_desired_distance_m < *task.scout_continue_distance_m &&
        *task.scout_continue_distance_m < *task.scout_stop_distance_m &&
        *task.scout_stop_distance_m < *task.communication_max_m)) {
    issue(result, "INCONSISTENT_SCOUT_DISTANCE_POLICY",
          "task.scout_distance_thresholds",
          "expected, continue, stop, and communication distances must be "
          "strictly increasing");
  }
}

std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  value = value.substr(first, last - first + 1U);
  if (value.size() >= 2U && ((value.front() == '"' && value.back() == '"') ||
                             (value.front() == '\'' && value.back() == '\''))) {
    value = value.substr(1U, value.size() - 2U);
  }
  return value;
}

std::optional<double> parse_double(const std::string& value) {
  if (value.empty() || value == "null" || value == "~") {
    return std::nullopt;
  }
  std::size_t consumed = 0U;
  const double parsed = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(parsed)) {
    throw std::invalid_argument("invalid numeric parameter value: " + value);
  }
  return parsed;
}

std::optional<std::uint64_t> parse_unsigned(const std::string& value,
                                            const char* field_name) {
  if (value.empty() || value == "null" || value == "~") {
    return std::nullopt;
  }
  std::uint64_t parsed{};
  const auto result = std::from_chars(value.data(), value.data() + value.size(),
                                      parsed, 10);
  if (result.ec != std::errc{} || result.ptr != value.data() + value.size()) {
    throw std::invalid_argument(std::string("invalid unsigned ") + field_name +
                                ": " + value);
  }
  return parsed;
}

bool parse_boolean(const std::string& value, const char* field_name) {
  if (value == "true" || value == "1") {
    return true;
  }
  if (value == "false" || value == "0") {
    return false;
  }
  throw std::invalid_argument(std::string("invalid boolean ") + field_name +
                              ": " + value);
}

ParameterProfileMode parse_profile_mode(const std::string& value) {
  if (value == "production") {
    return ParameterProfileMode::production;
  }
  if (value == "non_production_capability_profile") {
    return ParameterProfileMode::non_production_capability_profile;
  }
  throw std::invalid_argument("invalid profile mode: " + value);
}

bool assign_search_scalar(SearchParameterConfig& search,
                          const std::string& key,
                          const std::string& value) {
  const auto number = [&]() { return parse_double(value); };
  if (key == "search.maximum_active_labels") {
    search.maximum_active_labels =
        parse_unsigned(value, "search.maximum_active_labels");
    return true;
  }
  if (key == "search.equivalent_label_cost_tolerance_m") search.equivalent_label_cost_tolerance_m = number();
  else if (key == "search.xy_resolution_m") search.xy_resolution_m = number();
  else if (key == "search.heading_resolution_rad") search.heading_resolution_rad = number();
  else if (key == "search.cable_lag_resolution_rad") search.cable_lag_resolution_rad = number();
  else if (key == "search.reference_progress_resolution_m") search.reference_progress_resolution_m = number();
  else if (key == "search.reference_progress_backward_tolerance_m") search.reference_progress_backward_tolerance_m = number();
  else if (key == "search.reference_progress_maximum_ratio") search.reference_progress_maximum_ratio = number();
  else if (key == "search.reference_progress_forward_slack_m") search.reference_progress_forward_slack_m = number();
  else if (key == "search.reference_progress_distance_scale_m") search.reference_progress_distance_scale_m = number();
  else if (key == "search.reference_progress_heading_scale_rad") search.reference_progress_heading_scale_rad = number();
  else if (key == "search.reference_progress_heading_weight") search.reference_progress_heading_weight = number();
  else if (key == "search.reference_progress_association_score_tolerance") search.reference_progress_association_score_tolerance = number();
  else if (key == "search.collision_sweep_margin_m") search.collision_sweep_margin_m = number();
  else if (key == "search.cable_sweep_margin_m") search.cable_sweep_margin_m = number();
  else if (key == "search.path_length_cost_weight") search.path_length_cost_weight = number();
  else if (key == "search.path_curvature_cost_weight") search.path_curvature_cost_weight = number();
  else if (key == "search.touchdown_center_cost_weight") search.touchdown_center_cost_weight = number();
  else if (key == "search.touchdown_margin_cost_weight") search.touchdown_margin_cost_weight = number();
  else if (key == "search.robot_terrain_cost_weight") search.robot_terrain_cost_weight = number();
  else return false;
  return true;
}

bool assign_task_scalar(TaskParameterConfig& task, const std::string& key,
                        const std::string& value) {
  const auto number = [&]() { return parse_double(value); };
  if (key == "task.scout_policy_version") {
    task.scout_policy_version = parse_unsigned(value, "task.scout_policy_version");
    return true;
  }
  if (key == "task.laying_success_ratio_target") task.laying_success_ratio_target = number();
  else if (key == "task.communication_max_m") task.communication_max_m = number();
  else if (key == "task.scout_desired_distance_m") task.scout_desired_distance_m = number();
  else if (key == "task.scout_minimum_map_confidence") task.scout_minimum_map_confidence = number();
  else if (key == "task.scout_sample_interval_m") task.scout_sample_interval_m = number();
  else if (key == "task.scout_merge_distance_m") task.scout_merge_distance_m = number();
  else if (key == "task.scout_minimum_safe_distance_m") task.scout_minimum_safe_distance_m = number();
  else if (key == "task.scout_planning_lead_time_s") task.scout_planning_lead_time_s = number();
  else if (key == "task.scout_average_velocity_mps") task.scout_average_velocity_mps = number();
  else if (key == "task.scout_urgency_hysteresis_distance_m") task.scout_urgency_hysteresis_distance_m = number();
  else if (key == "task.scout_urgency_hysteresis_time_s") task.scout_urgency_hysteresis_time_s = number();
  else if (key == "task.scout_sensor_coverage_radius_m") task.scout_sensor_coverage_radius_m = number();
  else if (key == "task.scout_corridor_half_width_m") task.scout_corridor_half_width_m = number();
  else if (key == "task.scout_continue_distance_m") task.scout_continue_distance_m = number();
  else if (key == "task.scout_stop_distance_m") task.scout_stop_distance_m = number();
  else if (key == "task.scout_blocking_priority_weight") task.scout_blocking_priority_weight = number();
  else if (key == "task.scout_information_value_weight") task.scout_information_value_weight = number();
  else if (key == "task.scout_forward_progress_weight") task.scout_forward_progress_weight = number();
  else if (key == "task.scout_arrival_cost_weight") task.scout_arrival_cost_weight = number();
  else if (key == "task.scout_request_timeout_s") task.scout_request_timeout_s = number();
  else return false;
  return true;
}

bool assign_execution_scalar(ExecutionParameterConfig& execution,
                             const std::string& key,
                             const std::string& value) {
  const auto number = [&]() { return parse_double(value); };
  if (key == "execution.operating_envelope_version")
    execution.operating_envelope_version = value;
  else if (key == "execution.operating_domain_id")
    execution.operating_domain_id = value;
  else if (key == "execution.minimum_ground_speed_mps")
    execution.minimum_ground_speed_mps = number();
  else if (key == "execution.maximum_ground_speed_mps")
    execution.maximum_ground_speed_mps = number();
  else if (key == "execution.maximum_acceleration_mps2")
    execution.maximum_acceleration_mps2 = number();
  else if (key == "execution.maximum_deceleration_mps2")
    execution.maximum_deceleration_mps2 = number();
  else if (key == "execution.minimum_payout_speed_mps")
    execution.minimum_payout_speed_mps = number();
  else if (key == "execution.maximum_payout_speed_mps")
    execution.maximum_payout_speed_mps = number();
  else if (key == "execution.maximum_payout_acceleration_mps2")
    execution.maximum_payout_acceleration_mps2 = number();
  else if (key == "execution.minimum_tension_n")
    execution.minimum_tension_n = number();
  else if (key == "execution.maximum_tension_n")
    execution.maximum_tension_n = number();
  else if (key == "execution.sample_period_s")
    execution.sample_period_s = number();
  else if (key == "execution.terminal_speed_mps")
    execution.terminal_speed_mps = number();
  else if (key == "execution.stopping_distance_margin_m")
    execution.stopping_distance_margin_m = number();
  else if (key == "execution.ground_speed_tracking_error_mps")
    execution.ground_speed_tracking_error_mps = number();
  else if (key == "execution.ground_acceleration_tolerance_mps2")
    execution.ground_acceleration_tolerance_mps2 = number();
  else if (key == "execution.payout_speed_tracking_error_mps")
    execution.payout_speed_tracking_error_mps = number();
  else if (key == "execution.tension_tracking_error_n")
    execution.tension_tracking_error_n = number();
  else if (key == "execution.position_join_tolerance_m")
    execution.position_join_tolerance_m = number();
  else if (key == "execution.heading_join_tolerance_rad")
    execution.heading_join_tolerance_rad = number();
  else if (key == "execution.curvature_join_tolerance_per_m")
    execution.curvature_join_tolerance_per_m = number();
  else
    return false;
  return true;
}

bool assign_cable_scalar(CableMechanicalParameterConfig& cable,
                         const std::string& key,
                         const std::string& value) {
  const auto number = [&]() { return parse_double(value); };
  if (key == "cable.model_version") cable.model_version = value;
  else if (key == "cable.calibration_dataset_id")
    cable.calibration_dataset_id = value;
  else if (key == "cable.release_point_x_m") cable.release_point_x_m = number();
  else if (key == "cable.release_point_y_m") cable.release_point_y_m = number();
  else if (key == "cable.touchdown_distance_m")
    cable.touchdown_distance_m = number();
  else if (key == "cable.direction_response_length_m")
    cable.direction_response_length_m = number();
  else if (key == "cable.maximum_lag_angle_rad")
    cable.maximum_lag_angle_rad = number();
  else if (key == "cable.maximum_payout_speed_tracking_error_mps")
    cable.maximum_payout_speed_tracking_error_mps = number();
  else if (key == "cable.minimum_payout_speed_mps")
    cable.minimum_payout_speed_mps = number();
  else if (key == "cable.maximum_payout_speed_mps")
    cable.maximum_payout_speed_mps = number();
  else if (key == "cable.maximum_payout_acceleration_mps2")
    cable.maximum_payout_acceleration_mps2 = number();
  else if (key == "cable.maximum_tension_tracking_error_n")
    cable.maximum_tension_tracking_error_n = number();
  else if (key == "cable.minimum_tension_n") cable.minimum_tension_n = number();
  else if (key == "cable.maximum_tension_n") cable.maximum_tension_n = number();
  else if (key == "cable.touchdown_distance_variance_m2")
    cable.touchdown_distance_variance_m2 = number();
  else if (key == "cable.direction_response_length_variance_m2")
    cable.direction_response_length_variance_m2 = number();
  else if (key == "cable.lag_angle_process_variance_per_m_rad2")
    cable.lag_angle_process_variance_per_m_rad2 = number();
  else if (key == "cable.touchdown_process_noise_xx_m2_per_m")
    cable.touchdown_process_noise_per_m_m2.xx_m2 = number();
  else if (key == "cable.touchdown_process_noise_xy_m2_per_m")
    cable.touchdown_process_noise_per_m_m2.xy_m2 = number();
  else if (key == "cable.touchdown_process_noise_yy_m2_per_m")
    cable.touchdown_process_noise_per_m_m2.yy_m2 = number();
  else if (key == "cable.manufacturer_minimum_bend_radius_m")
    cable.manufacturer_minimum_bend_radius_m = number();
  else if (key == "cable.preferred_curvature_per_m")
    cable.preferred_curvature_per_m = number();
  else if (key == "cable.maximum_curvature_per_m")
    cable.maximum_curvature_per_m = number();
  else if (key == "cable.curvature_evaluation_spacing_m")
    cable.curvature_evaluation_spacing_m = number();
  else if (key == "cable.support_evaluation_length_m")
    cable.support_evaluation_length_m = number();
  else if (key == "cable.medium_support_proxy_range_m")
    cable.medium_support_proxy_range_m = number();
  else if (key == "cable.maximum_support_proxy_range_m")
    cable.maximum_support_proxy_range_m = number();
  else if (key == "cable.minimum_terrain_confidence")
    cable.minimum_terrain_confidence = number();
  else if (key == "cable.forbidden_area_layer")
    cable.forbidden_area_layer = value;
  else
    return false;
  return true;
}

bool assign_statistical_risk_scalar(StatisticalRiskParameterConfig& risk,
                                    const std::string& key,
                                    const std::string& value) {
  const auto number = [&]() { return parse_double(value); };
  if (key == "statistical_risk.policy_version") risk.policy_version = value;
  else if (key == "statistical_risk.calibration_dataset_id")
    risk.calibration_dataset_id = value;
  else if (key == "statistical_risk.uncertainty_envelope_version")
    risk.uncertainty_envelope_version = value;
  else if (key == "statistical_risk.envelope_generator_version")
    risk.envelope_generator_version = value;
  else if (key == "statistical_risk.envelope_operating_domain_id")
    risk.envelope_operating_domain_id = value;
  else if (key == "statistical_risk.envelope_execution_version")
    risk.envelope_execution_version = value;
  else if (key == "statistical_risk.epsilon_point")
    risk.epsilon_point = number();
  else if (key == "statistical_risk.nominal_corridor_width_m")
    risk.nominal_corridor_width_m = number();
  else if (key == "statistical_risk.absolute_corridor_width_m")
    risk.absolute_corridor_width_m = number();
  else if (key == "statistical_risk.maximum_marginal_length_m")
    risk.maximum_marginal_length_m = number();
  else if (key == "statistical_risk.maximum_candidate_length_m")
    risk.maximum_candidate_length_m = number();
  else if (key == "statistical_risk.maximum_planning_duration_s")
    risk.maximum_planning_duration_s = number();
  else if (key == "statistical_risk.progress_resolution_m")
    risk.progress_resolution_m = number();
  else if (key == "statistical_risk.envelope_discretization_margin_m")
    risk.envelope_discretization_margin_m = number();
  else if (key == "statistical_risk.envelope_audit_tolerance_m")
    risk.envelope_audit_tolerance_m = number();
  else if (key == "statistical_risk.sensor_health_mode")
    risk.sensor_health_modes.push_back(value);
  else if (key == "statistical_risk.distribution_calibrated")
    risk.distribution_calibrated =
        parse_boolean(value, "statistical_risk.distribution_calibrated");
  else
    return false;
  return true;
}

void assign_scalar(ParameterConfig& config, const std::string& key,
                   const std::string& raw) {
  const std::string value = trim(raw);
  const auto number = [&]() { return parse_double(value); };
  const auto boolean = [&]() { return parse_boolean(value, key.c_str()); };
  if (key == "schema_version") config.schema_version = value;
  else if (key == "profile_id") config.profile_id = value;
  else if (key == "mode") config.mode = parse_profile_mode(value);
  else if (key == "operating_domain_id") config.operating_domain_id = value;
  else if (key == "robot.calibration_version") config.robot.calibration_version = value;
  else if (key == "robot.calibration_dataset_id") config.robot.calibration_dataset_id = value;
  else if (key == "robot.length_m") config.robot.length_m = number();
  else if (key == "robot.width_m") config.robot.width_m = number();
  else if (key == "robot.height_m") config.robot.height_m = number();
  else if (key == "robot.minimum_turning_radius_m") config.robot.minimum_turning_radius_m = number();
  else if (key == "robot.maximum_curvature_per_m") config.robot.maximum_curvature_per_m = number();
  else if (key == "robot.maximum_curvature_rate_per_m2") config.robot.maximum_curvature_rate_per_m2 = number();
  else if (key == "robot.curvature_state_max_age_s") config.robot.curvature_state_max_age_s = number();
  else if (key == "robot.minimum_ground_speed_mps") config.robot.minimum_ground_speed_mps = number();
  else if (key == "robot.maximum_ground_speed_mps") config.robot.maximum_ground_speed_mps = number();
  else if (key == "robot.maximum_acceleration_mps2") config.robot.maximum_acceleration_mps2 = number();
  else if (key == "robot.maximum_deceleration_mps2") config.robot.maximum_deceleration_mps2 = number();
  else if (key == "robot.maximum_lateral_acceleration_mps2") config.robot.maximum_lateral_acceleration_mps2 = number();
  else if (key == "robot.maximum_slope_up_rad") config.robot.maximum_slope_up_rad = number();
  else if (key == "robot.maximum_slope_down_rad") config.robot.maximum_slope_down_rad = number();
  else if (key == "robot.maximum_slope_lateral_rad") config.robot.maximum_slope_lateral_rad = number();
  else if (key == "robot.maximum_support_roll_rad") config.robot.maximum_support_roll_rad = number();
  else if (key == "robot.maximum_step_climb_m") config.robot.maximum_step_climb_m = number();
  else if (key == "robot.maximum_step_drop_m") config.robot.maximum_step_drop_m = number();
  else if (key == "robot.minimum_track_support_ratio") config.robot.minimum_track_support_ratio = number();
  else if (key == "robot.maximum_roughness_m") config.robot.maximum_roughness_m = number();
  else if (key == "robot.safe_obstacle_distance_m") config.robot.safe_obstacle_distance_m = number();
  else if (key == "robot.effective_track_spacing_m") config.robot.effective_track_spacing_m = number();
  else if (key == "robot.minimum_step_crossing_alignment") config.robot.minimum_step_crossing_alignment = number();
  else if (key == "robot.step_alignment_transition_band") config.robot.step_alignment_transition_band = number();
  else if (key == "robot.left_track_support_defined") config.robot.left_track_support_defined = boolean();
  else if (key == "robot.right_track_support_defined") config.robot.right_track_support_defined = boolean();
  else if (key == "robot.localization_covariance_defined") config.robot.localization_covariance_defined = boolean();
  else if (key == "robot.control_tracking_covariance_defined") config.robot.control_tracking_covariance_defined = boolean();
  else if (key == "terrain_gradient_risk.policy_version") config.terrain_gradient_risk.policy_version = value;
  else if (key == "terrain_gradient_risk.terrain_analysis_config_version") config.terrain_gradient_risk.terrain_analysis_config_version = value;
  else if (key == "terrain_gradient_risk.calibration_dataset_id") config.terrain_gradient_risk.calibration_dataset_id = value;
  else if (key == "terrain_gradient_risk.operating_domain_id") config.terrain_gradient_risk.operating_domain_id = value;
  else if (key == "terrain_gradient_risk.epsilon_local") config.terrain_gradient_risk.epsilon_local = number();
  else if (key == "terrain_gradient_risk.coverage_multiplier") config.terrain_gradient_risk.coverage_multiplier = number();
  else if (key == "terrain_gradient_risk.coverage_model") config.terrain_gradient_risk.coverage_model = value;
  else if (key == "robot_collision_risk.policy_version") config.robot_collision_risk.policy_version = value;
  else if (key == "robot_collision_risk.calibration_dataset_id") config.robot_collision_risk.calibration_dataset_id = value;
  else if (key == "robot_collision_risk.operating_domain_id") config.robot_collision_risk.operating_domain_id = value;
  else if (key == "robot_collision_risk.epsilon_robot") config.robot_collision_risk.epsilon_robot = number();
  else if (key == "robot_collision_risk.minimum_map_confidence") config.robot_collision_risk.minimum_map_confidence = number();
  else if (key == "spatial_domains.robot_operating_area_id") config.spatial_domains.robot_operating_area_id = value;
  else if (key == "spatial_domains.robot_operating_area_version") config.spatial_domains.robot_operating_area_version = value;
  else if (key == "spatial_domains.cable_corridor_id") config.spatial_domains.cable_corridor_id = value;
  else if (key == "spatial_domains.cable_corridor_version") config.spatial_domains.cable_corridor_version = value;
  else if (key == "spatial_domains.robot_operating_area_non_empty") config.spatial_domains.robot_operating_area_non_empty = boolean();
  else if (key == "spatial_domains.cable_corridor_non_empty") config.spatial_domains.cable_corridor_non_empty = boolean();
  else if (key.rfind("execution.", 0U) == 0U) {
    if (!assign_execution_scalar(config.execution, key, value)) {
      throw std::invalid_argument("unknown parameter key: " + key);
    }
  }
  else if (key.rfind("cable.", 0U) == 0U) {
    if (!assign_cable_scalar(config.cable, key, value)) {
      throw std::invalid_argument("unknown parameter key: " + key);
    }
  }
  else if (key.rfind("statistical_risk.", 0U) == 0U) {
    if (!assign_statistical_risk_scalar(config.statistical_risk, key, value)) {
      throw std::invalid_argument("unknown parameter key: " + key);
    }
  }
  else if (key == "path_reuse.reuse_max_s") config.path_reuse.reuse_max_s = number();
  else if (key == "path_reuse.robot_state_max_age_s") config.path_reuse.robot_state_max_age_s = number();
  else if (key == "path_reuse.cable_state_max_age_s") config.path_reuse.cable_state_max_age_s = number();
  else if (key == "path_reuse.cable_telemetry_max_age_s") config.path_reuse.cable_telemetry_max_age_s = number();
  else if (key == "path_reuse.execution_tracking_max_age_s") config.path_reuse.execution_tracking_max_age_s = number();
  else if (key == "path_reuse.lease_monitor_period_s") config.path_reuse.lease_monitor_period_s = number();
  else if (key == "path_reuse.lease_renewal_margin_s") config.path_reuse.lease_renewal_margin_s = number();
  else if (key.rfind("search.", 0U) == 0U) {
    if (!assign_search_scalar(config.search, key, value)) {
      throw std::invalid_argument("unknown parameter key: " + key);
    }
  }
  else if (key.rfind("task.", 0U) == 0U) {
    if (!assign_task_scalar(config.task, key, value)) {
      throw std::invalid_argument("unknown parameter key: " + key);
    }
  }
  else {
    throw std::invalid_argument("unknown parameter key: " + key);
  }
}

}  // namespace

ParameterValidationResult validate_parameters(const ParameterConfig& config,
                                              ParameterProfileMode mode) {
  ParameterValidationResult result;
  result.mode = mode;
  result.non_production = mode == ParameterProfileMode::non_production_capability_profile;
  if (config.schema_version != "parameter-config/v1") {
    issue(result, "UNSUPPORTED_SCHEMA_VERSION", "schema_version",
          "only parameter-config/v1 is supported");
  }
  if (config.profile_id.empty()) {
    issue(result, "MISSING_PROFILE_ID", "profile_id",
          "every loaded parameter profile must have an auditable identifier");
  }
  if (config.operating_domain_id.empty()) {
    issue(result, "MISSING_OPERATING_DOMAIN", "operating_domain_id",
          "configuration operating domain is required");
  }
  if (config.mode != mode) {
    issue(result, "PROFILE_MODE_MISMATCH", "mode",
          "requested validation mode does not match the profile mode");
  }
  validate_common_finite(result, config);
  if (mode == ParameterProfileMode::production) {
    validate_production(result, config);
  }
  result.valid = result.issues.empty();
  return result;
}

ParameterValidationResult validate(const ParameterConfig& config,
                                   ParameterProfileMode mode) {
  return validate_parameters(config, mode);
}

bool production_ready(const ParameterConfig& config) {
  return validate_parameters(config, ParameterProfileMode::production).valid;
}

ParameterValidationResult validate_spatial_domain_snapshot(
    const ParameterConfig& config,
    const VersionedPlanningSnapshot& snapshot) {
  ParameterValidationResult result;
  result.mode = config.mode;
  result.non_production =
      config.mode == ParameterProfileMode::non_production_capability_profile;
  const auto parse_version = [&](const std::string& text,
                                 const char* path) -> std::optional<std::uint32_t> {
    std::uint32_t value{};
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (text.empty() || parsed.ec != std::errc{} ||
        parsed.ptr != text.data() + text.size() || value == 0U ||
        std::to_string(value) != text) {
      issue(result, "SPATIAL_DOMAIN_VERSION_INVALID", path,
            "snapshot version must be canonical nonzero unsigned decimal text");
      return std::nullopt;
    }
    return value;
  };
  const auto area_version = parse_version(
      config.spatial_domains.robot_operating_area_version,
      "spatial_domains.robot_operating_area_version");
  const auto corridor_version = parse_version(
      config.spatial_domains.cable_corridor_version,
      "spatial_domains.cable_corridor_version");
  if (config.spatial_domains.robot_operating_area_id !=
      snapshot.robot_operating_area.id) {
    issue(result, "SPATIAL_DOMAIN_ID_MISMATCH",
          "spatial_domains.robot_operating_area_id",
          "parameter id does not match the immutable planning snapshot");
  }
  if (config.spatial_domains.cable_corridor_id != snapshot.cable_corridor.id) {
    issue(result, "SPATIAL_DOMAIN_ID_MISMATCH",
          "spatial_domains.cable_corridor_id",
          "parameter id does not match the immutable planning snapshot");
  }
  if (area_version.has_value() &&
      *area_version != snapshot.robot_operating_area.version) {
    issue(result, "SPATIAL_DOMAIN_VERSION_MISMATCH",
          "spatial_domains.robot_operating_area_version",
          "parameter version does not match the immutable planning snapshot");
  }
  if (corridor_version.has_value() &&
      *corridor_version != snapshot.cable_corridor.version) {
    issue(result, "SPATIAL_DOMAIN_VERSION_MISMATCH",
          "spatial_domains.cable_corridor_version",
          "parameter version does not match the immutable planning snapshot");
  }
  result.valid = result.issues.empty();
  return result;
}

ParameterConfig load_parameter_config(std::string_view text) {
  ParameterConfig config;
  std::istringstream stream{std::string(text)};
  std::string line;
  std::size_t line_number = 0U;
  std::vector<std::pair<std::size_t, std::string>> sections;
  std::unordered_set<std::string> scalar_keys;
  while (std::getline(stream, line)) {
    ++line_number;
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line.erase(comment);
    }
    const std::size_t indentation = line.find_first_not_of(" \t");
    if (indentation == std::string::npos) {
      continue;
    }
    line = trim(line);
    if (line == "UP_CONFIG 1") {
      continue;
    }
    const auto separator = line.find(':');
    if (separator == std::string::npos) {
      throw std::invalid_argument("parameter config line " + std::to_string(line_number) +
                                  " is missing ':'");
    }
    const std::string key = trim(line.substr(0U, separator));
    const std::string value = trim(line.substr(separator + 1U));
    while (!sections.empty() && indentation <= sections.back().first) {
      sections.pop_back();
    }
    std::string dotted_key;
    for (const auto& section : sections) {
      if (!dotted_key.empty()) {
        dotted_key += '.';
      }
      dotted_key += section.second;
    }
    if (!dotted_key.empty() && key.find('.') == std::string::npos) {
      dotted_key += '.';
    }
    dotted_key += key;
    try {
      if (value.empty()) {
        sections.emplace_back(indentation, key);
      } else {
        constexpr std::string_view kRepeatableSensorHealthMode =
            "statistical_risk.sensor_health_mode";
        if (dotted_key != kRepeatableSensorHealthMode &&
            !scalar_keys.insert(dotted_key).second) {
          throw std::invalid_argument("duplicate scalar parameter key: " +
                                      dotted_key);
        }
        assign_scalar(config, dotted_key, value);
      }
    } catch (const std::exception& error) {
      throw std::invalid_argument("parameter config line " + std::to_string(line_number) +
                                  ": " + error.what());
    }
  }
  return config;
}

std::string serialize_parameter_config(const ParameterConfig& config) {
  std::ostringstream output;
  output << std::setprecision(17);
  const auto optional = [&output](const char* path, const auto& value) {
    if (value.has_value()) output << path << ':' << *value << '\n';
  };
  output << "UP_CONFIG 1\n";
  output << "schema_version:" << config.schema_version << '\n';
  output << "profile_id:" << config.profile_id << '\n';
  output << "mode:" << (config.mode == ParameterProfileMode::production
                             ? "production"
                             : "non_production_capability_profile") << '\n';
  output << "operating_domain_id:" << config.operating_domain_id << '\n';
  output << "robot.calibration_version:" << config.robot.calibration_version << '\n';
  output << "robot.calibration_dataset_id:" << config.robot.calibration_dataset_id << '\n';
  output << "terrain_gradient_risk.policy_version:" << config.terrain_gradient_risk.policy_version << '\n';
  output << "terrain_gradient_risk.terrain_analysis_config_version:" << config.terrain_gradient_risk.terrain_analysis_config_version << '\n';
  output << "terrain_gradient_risk.calibration_dataset_id:" << config.terrain_gradient_risk.calibration_dataset_id << '\n';
  output << "terrain_gradient_risk.operating_domain_id:" << config.terrain_gradient_risk.operating_domain_id << '\n';
  output << "robot_collision_risk.policy_version:" << config.robot_collision_risk.policy_version << '\n';
  output << "robot_collision_risk.calibration_dataset_id:" << config.robot_collision_risk.calibration_dataset_id << '\n';
  output << "robot_collision_risk.operating_domain_id:" << config.robot_collision_risk.operating_domain_id << '\n';
  if (config.robot_collision_risk.epsilon_robot.has_value()) {
    output << "robot_collision_risk.epsilon_robot:"
           << *config.robot_collision_risk.epsilon_robot << '\n';
  }
  if (config.robot_collision_risk.minimum_map_confidence.has_value()) {
    output << "robot_collision_risk.minimum_map_confidence:"
           << *config.robot_collision_risk.minimum_map_confidence << '\n';
  }
  output << "spatial_domains.robot_operating_area_version:" << config.spatial_domains.robot_operating_area_version << '\n';
  output << "spatial_domains.cable_corridor_version:" << config.spatial_domains.cable_corridor_version << '\n';
  output << "execution.operating_envelope_version:" << config.execution.operating_envelope_version << '\n';
  output << "execution.operating_domain_id:" << config.execution.operating_domain_id << '\n';
  output << "cable.model_version:" << config.cable.model_version << '\n';
  if (config.cable.touchdown_distance_variance_m2.has_value()) {
    output << "cable.touchdown_distance_variance_m2:"
           << *config.cable.touchdown_distance_variance_m2 << '\n';
  }
  if (config.cable.direction_response_length_variance_m2.has_value()) {
    output << "cable.direction_response_length_variance_m2:"
           << *config.cable.direction_response_length_variance_m2 << '\n';
  }
  if (config.cable.lag_angle_process_variance_per_m_rad2.has_value()) {
    output << "cable.lag_angle_process_variance_per_m_rad2:"
           << *config.cable.lag_angle_process_variance_per_m_rad2 << '\n';
  }
  const auto& process_noise = config.cable.touchdown_process_noise_per_m_m2;
  if (process_noise.xx_m2.has_value()) {
    output << "cable.touchdown_process_noise_xx_m2_per_m:"
           << *process_noise.xx_m2 << '\n';
  }
  if (process_noise.xy_m2.has_value()) {
    output << "cable.touchdown_process_noise_xy_m2_per_m:"
           << *process_noise.xy_m2 << '\n';
  }
  if (process_noise.yy_m2.has_value()) {
    output << "cable.touchdown_process_noise_yy_m2_per_m:"
           << *process_noise.yy_m2 << '\n';
  }
  if (config.cable.medium_support_proxy_range_m.has_value()) {
    output << "cable.medium_support_proxy_range_m:"
           << *config.cable.medium_support_proxy_range_m << '\n';
  }
  if (config.cable.curvature_evaluation_spacing_m.has_value()) {
    output << "cable.curvature_evaluation_spacing_m:"
           << *config.cable.curvature_evaluation_spacing_m << '\n';
  }
  if (config.cable.minimum_terrain_confidence.has_value()) {
    output << "cable.minimum_terrain_confidence:"
           << *config.cable.minimum_terrain_confidence << '\n';
  }
  output << "statistical_risk.policy_version:" << config.statistical_risk.policy_version << '\n';
  output << "statistical_risk.uncertainty_envelope_version:" << config.statistical_risk.uncertainty_envelope_version << '\n';
  output << "statistical_risk.envelope_generator_version:" << config.statistical_risk.envelope_generator_version << '\n';
  output << "statistical_risk.envelope_operating_domain_id:" << config.statistical_risk.envelope_operating_domain_id << '\n';
  output << "statistical_risk.envelope_execution_version:" << config.statistical_risk.envelope_execution_version << '\n';
  const TaskParameterConfig& task = config.task;
  if (task.scout_policy_version.has_value()) {
    output << "task.scout_policy_version:" << *task.scout_policy_version << '\n';
  }
  const std::pair<const char*, std::optional<double>> scout_values[] = {
      {"task.scout_minimum_map_confidence", task.scout_minimum_map_confidence},
      {"task.scout_sample_interval_m", task.scout_sample_interval_m},
      {"task.scout_merge_distance_m", task.scout_merge_distance_m},
      {"task.scout_minimum_safe_distance_m", task.scout_minimum_safe_distance_m},
      {"task.scout_planning_lead_time_s", task.scout_planning_lead_time_s},
      {"task.scout_average_velocity_mps", task.scout_average_velocity_mps},
      {"task.scout_urgency_hysteresis_distance_m",
       task.scout_urgency_hysteresis_distance_m},
      {"task.scout_urgency_hysteresis_time_s",
       task.scout_urgency_hysteresis_time_s},
      {"task.scout_sensor_coverage_radius_m",
       task.scout_sensor_coverage_radius_m},
      {"task.scout_corridor_half_width_m", task.scout_corridor_half_width_m},
      {"task.communication_max_m", task.communication_max_m},
      {"task.scout_desired_distance_m", task.scout_desired_distance_m},
      {"task.scout_continue_distance_m", task.scout_continue_distance_m},
      {"task.scout_stop_distance_m", task.scout_stop_distance_m},
      {"task.scout_blocking_priority_weight",
       task.scout_blocking_priority_weight},
      {"task.scout_information_value_weight",
       task.scout_information_value_weight},
      {"task.scout_forward_progress_weight",
       task.scout_forward_progress_weight},
      {"task.scout_arrival_cost_weight", task.scout_arrival_cost_weight},
      {"task.scout_request_timeout_s", task.scout_request_timeout_s}};
  for (const auto& [path, value] : scout_values) {
    if (value.has_value()) output << path << ':' << *value << '\n';
  }
  const RobotParameterConfig& robot = config.robot;
  const std::pair<const char*, std::optional<double>> robot_values[] = {
      {"robot.length_m", robot.length_m},
      {"robot.width_m", robot.width_m},
      {"robot.height_m", robot.height_m},
      {"robot.minimum_turning_radius_m", robot.minimum_turning_radius_m},
      {"robot.maximum_curvature_per_m", robot.maximum_curvature_per_m},
      {"robot.maximum_curvature_rate_per_m2",
       robot.maximum_curvature_rate_per_m2},
      {"robot.curvature_state_max_age_s", robot.curvature_state_max_age_s},
      {"robot.minimum_ground_speed_mps", robot.minimum_ground_speed_mps},
      {"robot.maximum_ground_speed_mps", robot.maximum_ground_speed_mps},
      {"robot.maximum_acceleration_mps2", robot.maximum_acceleration_mps2},
      {"robot.maximum_deceleration_mps2", robot.maximum_deceleration_mps2},
      {"robot.maximum_lateral_acceleration_mps2",
       robot.maximum_lateral_acceleration_mps2},
      {"robot.maximum_slope_up_rad", robot.maximum_slope_up_rad},
      {"robot.maximum_slope_down_rad", robot.maximum_slope_down_rad},
      {"robot.maximum_slope_lateral_rad", robot.maximum_slope_lateral_rad},
      {"robot.maximum_support_roll_rad", robot.maximum_support_roll_rad},
      {"robot.maximum_step_climb_m", robot.maximum_step_climb_m},
      {"robot.maximum_step_drop_m", robot.maximum_step_drop_m},
      {"robot.minimum_track_support_ratio",
       robot.minimum_track_support_ratio},
      {"robot.maximum_roughness_m", robot.maximum_roughness_m},
      {"robot.safe_obstacle_distance_m", robot.safe_obstacle_distance_m},
      {"robot.effective_track_spacing_m",
       robot.effective_track_spacing_m},
      {"robot.minimum_step_crossing_alignment",
       robot.minimum_step_crossing_alignment},
      {"robot.step_alignment_transition_band",
       robot.step_alignment_transition_band}};
  for (const auto& [path, value] : robot_values) optional(path, value);
  output << "robot.left_track_support_defined:"
         << robot.left_track_support_defined << '\n'
         << "robot.right_track_support_defined:"
         << robot.right_track_support_defined << '\n'
         << "robot.localization_covariance_defined:"
         << robot.localization_covariance_defined << '\n'
         << "robot.control_tracking_covariance_defined:"
         << robot.control_tracking_covariance_defined << '\n';
  optional("terrain_gradient_risk.epsilon_local",
           config.terrain_gradient_risk.epsilon_local);
  optional("terrain_gradient_risk.coverage_multiplier",
           config.terrain_gradient_risk.coverage_multiplier);
  output << "terrain_gradient_risk.coverage_model:"
         << config.terrain_gradient_risk.coverage_model << '\n'
         << "spatial_domains.robot_operating_area_id:"
         << config.spatial_domains.robot_operating_area_id << '\n'
         << "spatial_domains.cable_corridor_id:"
         << config.spatial_domains.cable_corridor_id << '\n'
         << "spatial_domains.robot_operating_area_non_empty:"
         << config.spatial_domains.robot_operating_area_non_empty << '\n'
         << "spatial_domains.cable_corridor_non_empty:"
         << config.spatial_domains.cable_corridor_non_empty << '\n';
  const ExecutionParameterConfig& execution = config.execution;
  const std::pair<const char*, std::optional<double>> execution_values[] = {
      {"execution.minimum_ground_speed_mps",
       execution.minimum_ground_speed_mps},
      {"execution.maximum_ground_speed_mps",
       execution.maximum_ground_speed_mps},
      {"execution.maximum_acceleration_mps2",
       execution.maximum_acceleration_mps2},
      {"execution.maximum_deceleration_mps2",
       execution.maximum_deceleration_mps2},
      {"execution.minimum_payout_speed_mps",
       execution.minimum_payout_speed_mps},
      {"execution.maximum_payout_speed_mps",
       execution.maximum_payout_speed_mps},
      {"execution.maximum_payout_acceleration_mps2",
       execution.maximum_payout_acceleration_mps2},
      {"execution.minimum_tension_n", execution.minimum_tension_n},
      {"execution.maximum_tension_n", execution.maximum_tension_n},
      {"execution.sample_period_s", execution.sample_period_s},
      {"execution.terminal_speed_mps", execution.terminal_speed_mps},
      {"execution.stopping_distance_margin_m",
       execution.stopping_distance_margin_m},
      {"execution.ground_speed_tracking_error_mps",
       execution.ground_speed_tracking_error_mps},
      {"execution.ground_acceleration_tolerance_mps2",
       execution.ground_acceleration_tolerance_mps2},
      {"execution.payout_speed_tracking_error_mps",
       execution.payout_speed_tracking_error_mps},
      {"execution.tension_tracking_error_n",
       execution.tension_tracking_error_n},
      {"execution.position_join_tolerance_m",
       execution.position_join_tolerance_m},
      {"execution.heading_join_tolerance_rad",
       execution.heading_join_tolerance_rad},
      {"execution.curvature_join_tolerance_per_m",
       execution.curvature_join_tolerance_per_m}};
  for (const auto& [path, value] : execution_values) optional(path, value);
  const CableMechanicalParameterConfig& cable = config.cable;
  const std::pair<const char*, std::optional<double>> cable_values[] = {
      {"cable.release_point_x_m", cable.release_point_x_m},
      {"cable.release_point_y_m", cable.release_point_y_m},
      {"cable.touchdown_distance_m", cable.touchdown_distance_m},
      {"cable.direction_response_length_m",
       cable.direction_response_length_m},
      {"cable.maximum_lag_angle_rad", cable.maximum_lag_angle_rad},
      {"cable.maximum_payout_speed_tracking_error_mps",
       cable.maximum_payout_speed_tracking_error_mps},
      {"cable.minimum_payout_speed_mps", cable.minimum_payout_speed_mps},
      {"cable.maximum_payout_speed_mps", cable.maximum_payout_speed_mps},
      {"cable.maximum_payout_acceleration_mps2",
       cable.maximum_payout_acceleration_mps2},
      {"cable.maximum_tension_tracking_error_n",
       cable.maximum_tension_tracking_error_n},
      {"cable.minimum_tension_n", cable.minimum_tension_n},
      {"cable.maximum_tension_n", cable.maximum_tension_n},
      {"cable.manufacturer_minimum_bend_radius_m",
       cable.manufacturer_minimum_bend_radius_m},
      {"cable.preferred_curvature_per_m", cable.preferred_curvature_per_m},
      {"cable.maximum_curvature_per_m", cable.maximum_curvature_per_m},
      {"cable.support_evaluation_length_m",
       cable.support_evaluation_length_m},
      {"cable.maximum_support_proxy_range_m",
       cable.maximum_support_proxy_range_m}};
  for (const auto& [path, value] : cable_values) optional(path, value);
  output << "cable.calibration_dataset_id:" << cable.calibration_dataset_id
         << '\n' << "cable.forbidden_area_layer:"
         << cable.forbidden_area_layer << '\n';
  const StatisticalRiskParameterConfig& risk = config.statistical_risk;
  const std::pair<const char*, std::optional<double>> risk_values[] = {
      {"statistical_risk.epsilon_point", risk.epsilon_point},
      {"statistical_risk.nominal_corridor_width_m",
       risk.nominal_corridor_width_m},
      {"statistical_risk.absolute_corridor_width_m",
       risk.absolute_corridor_width_m},
      {"statistical_risk.maximum_marginal_length_m",
       risk.maximum_marginal_length_m},
      {"statistical_risk.maximum_candidate_length_m",
       risk.maximum_candidate_length_m},
      {"statistical_risk.maximum_planning_duration_s",
       risk.maximum_planning_duration_s},
      {"statistical_risk.progress_resolution_m", risk.progress_resolution_m},
      {"statistical_risk.envelope_discretization_margin_m",
       risk.envelope_discretization_margin_m},
      {"statistical_risk.envelope_audit_tolerance_m",
       risk.envelope_audit_tolerance_m}};
  for (const auto& [path, value] : risk_values) optional(path, value);
  output << "statistical_risk.calibration_dataset_id:"
         << risk.calibration_dataset_id << '\n';
  for (const std::string& mode : risk.sensor_health_modes) {
    output << "statistical_risk.sensor_health_mode:" << mode << '\n';
  }
  output << "statistical_risk.distribution_calibrated:"
         << risk.distribution_calibrated << '\n';
  const PathReuseParameterConfig& reuse = config.path_reuse;
  const std::pair<const char*, std::optional<double>> reuse_values[] = {
      {"path_reuse.reuse_max_s", reuse.reuse_max_s},
      {"path_reuse.robot_state_max_age_s", reuse.robot_state_max_age_s},
      {"path_reuse.cable_state_max_age_s", reuse.cable_state_max_age_s},
      {"path_reuse.cable_telemetry_max_age_s",
       reuse.cable_telemetry_max_age_s},
      {"path_reuse.execution_tracking_max_age_s",
       reuse.execution_tracking_max_age_s},
      {"path_reuse.lease_monitor_period_s", reuse.lease_monitor_period_s},
      {"path_reuse.lease_renewal_margin_s",
       reuse.lease_renewal_margin_s}};
  for (const auto& [path, value] : reuse_values) optional(path, value);
  const SearchParameterConfig& search = config.search;
  if (search.maximum_active_labels.has_value()) {
    output << "search.maximum_active_labels:"
           << *search.maximum_active_labels << '\n';
  }
  const std::pair<const char*, std::optional<double>> search_values[] = {
      {"search.equivalent_label_cost_tolerance_m",
       search.equivalent_label_cost_tolerance_m},
      {"search.xy_resolution_m", search.xy_resolution_m},
      {"search.heading_resolution_rad", search.heading_resolution_rad},
      {"search.cable_lag_resolution_rad", search.cable_lag_resolution_rad},
      {"search.reference_progress_resolution_m",
       search.reference_progress_resolution_m},
      {"search.reference_progress_backward_tolerance_m",
       search.reference_progress_backward_tolerance_m},
      {"search.reference_progress_maximum_ratio",
       search.reference_progress_maximum_ratio},
      {"search.reference_progress_forward_slack_m",
       search.reference_progress_forward_slack_m},
      {"search.reference_progress_distance_scale_m",
       search.reference_progress_distance_scale_m},
      {"search.reference_progress_heading_scale_rad",
       search.reference_progress_heading_scale_rad},
      {"search.reference_progress_heading_weight",
       search.reference_progress_heading_weight},
      {"search.reference_progress_association_score_tolerance",
       search.reference_progress_association_score_tolerance},
      {"search.collision_sweep_margin_m", search.collision_sweep_margin_m},
      {"search.cable_sweep_margin_m", search.cable_sweep_margin_m},
      {"search.path_length_cost_weight", search.path_length_cost_weight},
      {"search.path_curvature_cost_weight",
       search.path_curvature_cost_weight},
      {"search.touchdown_center_cost_weight",
       search.touchdown_center_cost_weight},
      {"search.touchdown_margin_cost_weight",
       search.touchdown_margin_cost_weight},
      {"search.robot_terrain_cost_weight", search.robot_terrain_cost_weight}};
  for (const auto& [path, value] : search_values) optional(path, value);
  optional("task.laying_success_ratio_target",
           config.task.laying_success_ratio_target);
  return output.str();
}

}  // namespace underwater_planner::core
