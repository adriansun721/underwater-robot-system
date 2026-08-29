#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace underwater_planner::core {

struct VersionedPlanningSnapshot;

enum class ParameterProfileMode { production, non_production_capability_profile };

struct ParameterIssue {
  std::string code;
  std::string path;
  std::string message;
};

struct ParameterValidationResult {
  bool valid{};
  ParameterProfileMode mode{ParameterProfileMode::production};
  bool non_production{};
  std::vector<ParameterIssue> issues;
  [[nodiscard]] explicit operator bool() const noexcept { return valid; }
};

struct RobotParameterConfig {
  std::string calibration_version;
  std::string calibration_dataset_id;
  std::optional<double> length_m;
  std::optional<double> width_m;
  std::optional<double> height_m;
  std::optional<double> minimum_turning_radius_m;
  std::optional<double> maximum_curvature_per_m;
  std::optional<double> maximum_curvature_rate_per_m2;
  std::optional<double> curvature_state_max_age_s;
  std::optional<double> minimum_ground_speed_mps;
  std::optional<double> maximum_ground_speed_mps;
  std::optional<double> maximum_acceleration_mps2;
  std::optional<double> maximum_deceleration_mps2;
  std::optional<double> maximum_lateral_acceleration_mps2;
  std::optional<double> maximum_slope_up_rad;
  std::optional<double> maximum_slope_down_rad;
  std::optional<double> maximum_slope_lateral_rad;
  std::optional<double> maximum_support_roll_rad;
  std::optional<double> maximum_step_climb_m;
  std::optional<double> maximum_step_drop_m;
  std::optional<double> minimum_track_support_ratio;
  std::optional<double> maximum_roughness_m;
  std::optional<double> safe_obstacle_distance_m;
  std::optional<double> effective_track_spacing_m;
  std::optional<double> minimum_step_crossing_alignment;
  std::optional<double> step_alignment_transition_band;
  bool left_track_support_defined{};
  bool right_track_support_defined{};
  bool localization_covariance_defined{};
  bool control_tracking_covariance_defined{};
};

struct TerrainGradientRiskConfig {
  std::string policy_version;
  std::string terrain_analysis_config_version;
  std::string calibration_dataset_id;
  std::string operating_domain_id;
  std::optional<double> epsilon_local;
  std::optional<double> coverage_multiplier;
  std::string coverage_model{"gaussian_pending_calibration"};
};

struct RobotCollisionRiskConfig {
  std::string policy_version;
  std::string calibration_dataset_id;
  std::string operating_domain_id;
  std::optional<double> epsilon_robot;
  std::optional<double> minimum_map_confidence;
};

struct SpatialDomainConfig {
  std::string robot_operating_area_id;
  std::string robot_operating_area_version;
  std::string cable_corridor_id;
  std::string cable_corridor_version;
  bool robot_operating_area_non_empty{};
  bool cable_corridor_non_empty{};
};

struct ExecutionParameterConfig {
  std::string operating_envelope_version;
  std::string operating_domain_id;
  std::optional<double> minimum_ground_speed_mps;
  std::optional<double> maximum_ground_speed_mps;
  std::optional<double> maximum_acceleration_mps2;
  std::optional<double> maximum_deceleration_mps2;
  std::optional<double> minimum_payout_speed_mps;
  std::optional<double> maximum_payout_speed_mps;
  std::optional<double> maximum_payout_acceleration_mps2;
  std::optional<double> minimum_tension_n;
  std::optional<double> maximum_tension_n;
  std::optional<double> sample_period_s;
  std::optional<double> terminal_speed_mps;
  std::optional<double> stopping_distance_margin_m;
  std::optional<double> ground_speed_tracking_error_mps;
  std::optional<double> ground_acceleration_tolerance_mps2;
  std::optional<double> payout_speed_tracking_error_mps;
  std::optional<double> tension_tracking_error_n;
  std::optional<double> position_join_tolerance_m;
  std::optional<double> heading_join_tolerance_rad;
  std::optional<double> curvature_join_tolerance_per_m;
};

struct SymmetricCovariance2dParameterConfig {
  std::optional<double> xx_m2;
  std::optional<double> xy_m2;
  std::optional<double> yy_m2;
};

struct CableMechanicalParameterConfig {
  std::string model_version;
  std::string calibration_dataset_id;
  std::optional<double> release_point_x_m;
  std::optional<double> release_point_y_m;
  std::optional<double> touchdown_distance_m;
  std::optional<double> direction_response_length_m;
  std::optional<double> maximum_lag_angle_rad;
  std::optional<double> maximum_payout_speed_tracking_error_mps;
  std::optional<double> minimum_payout_speed_mps;
  std::optional<double> maximum_payout_speed_mps;
  std::optional<double> maximum_payout_acceleration_mps2;
  std::optional<double> maximum_tension_tracking_error_n;
  std::optional<double> minimum_tension_n;
  std::optional<double> maximum_tension_n;
  std::optional<double> touchdown_distance_variance_m2;
  std::optional<double> direction_response_length_variance_m2;
  std::optional<double> lag_angle_process_variance_per_m_rad2;
  SymmetricCovariance2dParameterConfig touchdown_process_noise_per_m_m2;
  std::optional<double> manufacturer_minimum_bend_radius_m;
  std::optional<double> preferred_curvature_per_m;
  std::optional<double> maximum_curvature_per_m;
  std::optional<double> curvature_evaluation_spacing_m;
  std::optional<double> support_evaluation_length_m;
  std::optional<double> medium_support_proxy_range_m;
  std::optional<double> maximum_support_proxy_range_m;
  std::optional<double> minimum_terrain_confidence;
  std::string forbidden_area_layer;
};

struct StatisticalRiskParameterConfig {
  std::string policy_version;
  std::string calibration_dataset_id;
  std::string uncertainty_envelope_version;
  std::string envelope_generator_version;
  std::string envelope_operating_domain_id;
  std::string envelope_execution_version;
  std::optional<double> epsilon_point;
  std::optional<double> nominal_corridor_width_m;
  std::optional<double> absolute_corridor_width_m;
  std::optional<double> maximum_marginal_length_m;
  std::optional<double> maximum_candidate_length_m;
  std::optional<double> maximum_planning_duration_s;
  std::optional<double> progress_resolution_m;
  std::optional<double> envelope_discretization_margin_m;
  std::optional<double> envelope_audit_tolerance_m;
  std::vector<std::string> sensor_health_modes;
  bool distribution_calibrated{};
};

struct PathReuseParameterConfig {
  std::optional<double> reuse_max_s;
  std::optional<double> robot_state_max_age_s;
  std::optional<double> cable_state_max_age_s;
  std::optional<double> cable_telemetry_max_age_s;
  std::optional<double> execution_tracking_max_age_s;
  std::optional<double> lease_monitor_period_s;
  std::optional<double> lease_renewal_margin_s;
};

struct SearchParameterConfig {
  std::optional<std::uint64_t> maximum_active_labels;
  std::optional<double> equivalent_label_cost_tolerance_m;
  std::optional<double> xy_resolution_m;
  std::optional<double> heading_resolution_rad;
  std::optional<double> cable_lag_resolution_rad;
  std::optional<double> reference_progress_resolution_m;
  std::optional<double> reference_progress_backward_tolerance_m;
  std::optional<double> reference_progress_maximum_ratio;
  std::optional<double> reference_progress_forward_slack_m;
  std::optional<double> reference_progress_distance_scale_m;
  std::optional<double> reference_progress_heading_scale_rad;
  std::optional<double> reference_progress_heading_weight;
  std::optional<double> reference_progress_association_score_tolerance;
  std::optional<double> collision_sweep_margin_m;
  std::optional<double> cable_sweep_margin_m;
  std::optional<double> path_length_cost_weight;
  std::optional<double> path_curvature_cost_weight;
  std::optional<double> touchdown_center_cost_weight;
  std::optional<double> touchdown_margin_cost_weight;
  std::optional<double> robot_terrain_cost_weight;
};

struct TaskParameterConfig {
  std::optional<double> laying_success_ratio_target;
  std::optional<double> communication_max_m;
  std::optional<double> scout_desired_distance_m;
  std::optional<std::uint64_t> scout_policy_version;
  std::optional<double> scout_minimum_map_confidence;
  std::optional<double> scout_sample_interval_m;
  std::optional<double> scout_merge_distance_m;
  std::optional<double> scout_minimum_safe_distance_m;
  std::optional<double> scout_planning_lead_time_s;
  std::optional<double> scout_average_velocity_mps;
  std::optional<double> scout_urgency_hysteresis_distance_m;
  std::optional<double> scout_urgency_hysteresis_time_s;
  std::optional<double> scout_sensor_coverage_radius_m;
  std::optional<double> scout_corridor_half_width_m;
  std::optional<double> scout_continue_distance_m;
  std::optional<double> scout_stop_distance_m;
  std::optional<double> scout_blocking_priority_weight;
  std::optional<double> scout_information_value_weight;
  std::optional<double> scout_forward_progress_weight;
  std::optional<double> scout_arrival_cost_weight;
  std::optional<double> scout_request_timeout_s;
};

struct ParameterConfig {
  std::string schema_version{"parameter-config/v1"};
  std::string profile_id;
  ParameterProfileMode mode{ParameterProfileMode::production};
  std::string operating_domain_id;
  RobotParameterConfig robot;
  TerrainGradientRiskConfig terrain_gradient_risk;
  RobotCollisionRiskConfig robot_collision_risk;
  SpatialDomainConfig spatial_domains;
  ExecutionParameterConfig execution;
  CableMechanicalParameterConfig cable;
  StatisticalRiskParameterConfig statistical_risk;
  PathReuseParameterConfig path_reuse;
  SearchParameterConfig search;
  TaskParameterConfig task;
};

[[nodiscard]] ParameterValidationResult validate_parameters(
    const ParameterConfig& config,
    ParameterProfileMode mode = ParameterProfileMode::production);

[[nodiscard]] ParameterValidationResult validate(
    const ParameterConfig& config,
    ParameterProfileMode mode = ParameterProfileMode::production);

[[nodiscard]] bool production_ready(const ParameterConfig& config);

// Spatial-domain versions use canonical unsigned decimal text in parameter
// files and are compared to the numeric immutable snapshot versions here.
[[nodiscard]] ParameterValidationResult validate_spatial_domain_snapshot(
    const ParameterConfig& config,
    const VersionedPlanningSnapshot& snapshot);

// Parses a deliberately small, deterministic YAML-like subset: one
// dotted-key/value per line (comments and blank lines are ignored). This keeps
// the core independent of a YAML runtime while retaining a versioned loader.
[[nodiscard]] ParameterConfig load_parameter_config(std::string_view text);

[[nodiscard]] std::string serialize_parameter_config(
    const ParameterConfig& config);

}  // namespace underwater_planner::core
