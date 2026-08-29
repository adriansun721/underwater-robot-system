#include "underwater_planner/core/parameter_config.hpp"
#include "underwater_planner/core/scout_coordinator.hpp"
#include "underwater_planner/core/versioned_snapshot.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

using namespace underwater_planner::core;

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

ParameterConfig valid_production_config() {
  ParameterConfig config;
  config.profile_id = "prod-test-v1";
  config.operating_domain_id = "tank-v1";
  config.robot.calibration_version = "robot-cal-v1";
  config.robot.calibration_dataset_id = "robot-dataset-v1";
  config.robot.length_m = 2.5;
  config.robot.width_m = 1.2;
  config.robot.height_m = 0.8;
  config.robot.minimum_turning_radius_m = 3.0;
  config.robot.maximum_curvature_per_m = 1.0 / 3.0;
  config.robot.maximum_curvature_rate_per_m2 = 0.1;
  config.robot.curvature_state_max_age_s = 0.1;
  config.robot.minimum_ground_speed_mps = 0.05;
  config.robot.maximum_ground_speed_mps = 0.5;
  config.robot.maximum_acceleration_mps2 = 0.2;
  config.robot.maximum_deceleration_mps2 = 0.3;
  config.robot.maximum_lateral_acceleration_mps2 = 0.2;
  config.robot.maximum_slope_up_rad = 0.4;
  config.robot.maximum_slope_down_rad = 0.35;
  config.robot.maximum_slope_lateral_rad = 0.25;
  config.robot.maximum_support_roll_rad = 0.2;
  config.robot.maximum_step_climb_m = 0.2;
  config.robot.maximum_step_drop_m = 0.2;
  config.robot.minimum_track_support_ratio = 0.8;
  config.robot.maximum_roughness_m = 0.05;
  config.robot.safe_obstacle_distance_m = 0.2;
  config.robot.effective_track_spacing_m = 0.8;
  config.robot.minimum_step_crossing_alignment = 0.2;
  config.robot.step_alignment_transition_band = 0.05;
  config.robot.left_track_support_defined = true;
  config.robot.right_track_support_defined = true;
  config.robot.localization_covariance_defined = true;
  config.robot.control_tracking_covariance_defined = true;

  config.terrain_gradient_risk.policy_version = "gradient-policy-v1";
  config.terrain_gradient_risk.terrain_analysis_config_version = "terrain-analysis-v1";
  config.terrain_gradient_risk.calibration_dataset_id = "terrain-dataset-v1";
  config.terrain_gradient_risk.operating_domain_id = "tank-v1";
  config.terrain_gradient_risk.epsilon_local = 0.02;
  config.terrain_gradient_risk.coverage_multiplier = 2.5;
  config.terrain_gradient_risk.coverage_model = "calibrated_gaussian";

  config.robot_collision_risk.policy_version = "collision-policy-v1";
  config.robot_collision_risk.calibration_dataset_id = "collision-dataset-v1";
  config.robot_collision_risk.operating_domain_id = "tank-v1";
  config.robot_collision_risk.epsilon_robot = 0.01;
  config.robot_collision_risk.minimum_map_confidence = 0.75;

  config.spatial_domains.robot_operating_area_id = "robot-area";
  config.spatial_domains.robot_operating_area_version = "1";
  config.spatial_domains.cable_corridor_id = "cable-corridor";
  config.spatial_domains.cable_corridor_version = "2";
  config.spatial_domains.robot_operating_area_non_empty = true;
  config.spatial_domains.cable_corridor_non_empty = true;

  config.execution.operating_envelope_version = "exec-envelope-v1";
  config.execution.operating_domain_id = "tank-v1";
  config.execution.minimum_ground_speed_mps = 0.05;
  config.execution.maximum_ground_speed_mps = 0.5;
  config.execution.maximum_acceleration_mps2 = 0.2;
  config.execution.maximum_deceleration_mps2 = 0.3;
  config.execution.minimum_payout_speed_mps = 0.0;
  config.execution.maximum_payout_speed_mps = 0.6;
  config.execution.maximum_payout_acceleration_mps2 = 0.2;
  config.execution.minimum_tension_n = 10.0;
  config.execution.maximum_tension_n = 100.0;
  config.execution.sample_period_s = 0.1;
  config.execution.terminal_speed_mps = 0.0;
  config.execution.stopping_distance_margin_m = 1.0;
  config.execution.ground_speed_tracking_error_mps = 0.05;
  config.execution.ground_acceleration_tolerance_mps2 = 0.05;
  config.execution.payout_speed_tracking_error_mps = 0.05;
  config.execution.tension_tracking_error_n = 5.0;
  config.execution.position_join_tolerance_m = 0.01;
  config.execution.heading_join_tolerance_rad = 0.02;
  config.execution.curvature_join_tolerance_per_m = 0.01;

  config.cable.model_version = "cable-model-v1";
  config.cable.calibration_dataset_id = "cable-dataset-v1";
  config.cable.release_point_x_m = -1.0;
  config.cable.release_point_y_m = 0.0;
  config.cable.touchdown_distance_m = 2.0;
  config.cable.direction_response_length_m = 1.5;
  config.cable.maximum_lag_angle_rad = 0.52;
  config.cable.maximum_payout_speed_tracking_error_mps = 0.05;
  config.cable.minimum_payout_speed_mps = 0.0;
  config.cable.maximum_payout_speed_mps = 0.6;
  config.cable.maximum_payout_acceleration_mps2 = 0.2;
  config.cable.maximum_tension_tracking_error_n = 5.0;
  config.cable.minimum_tension_n = 10.0;
  config.cable.maximum_tension_n = 100.0;
  config.cable.touchdown_distance_variance_m2 = 0.01;
  config.cable.direction_response_length_variance_m2 = 0.02;
  config.cable.lag_angle_process_variance_per_m_rad2 = 0.001;
  config.cable.touchdown_process_noise_per_m_m2 = {0.002, 0.0005, 0.003};
  config.cable.manufacturer_minimum_bend_radius_m = 2.0;
  config.cable.preferred_curvature_per_m = 0.2;
  config.cable.maximum_curvature_per_m = 0.4;
  config.cable.curvature_evaluation_spacing_m = 0.5;
  config.cable.support_evaluation_length_m = 3.0;
  config.cable.medium_support_proxy_range_m = 0.1;
  config.cable.maximum_support_proxy_range_m = 0.2;
  config.cable.minimum_terrain_confidence = 0.75;
  config.cable.forbidden_area_layer = "cable-forbidden-v1";

  config.statistical_risk.policy_version = "corridor-policy-v1";
  config.statistical_risk.calibration_dataset_id = "corridor-dataset-v1";
  config.statistical_risk.uncertainty_envelope_version = "envelope-v1";
  config.statistical_risk.envelope_generator_version = "envelope-generator-v1";
  config.statistical_risk.envelope_operating_domain_id = "tank-v1";
  config.statistical_risk.envelope_execution_version = "exec-envelope-v1";
  config.statistical_risk.epsilon_point = 0.05;
  config.statistical_risk.nominal_corridor_width_m = 10.0;
  config.statistical_risk.absolute_corridor_width_m = 15.0;
  config.statistical_risk.maximum_marginal_length_m = 2.0;
  config.statistical_risk.maximum_candidate_length_m = 50.0;
  config.statistical_risk.maximum_planning_duration_s = 0.5;
  config.statistical_risk.progress_resolution_m = 0.5;
  config.statistical_risk.envelope_discretization_margin_m = 0.1;
  config.statistical_risk.envelope_audit_tolerance_m = 0.05;
  config.statistical_risk.sensor_health_modes = {"imu"};
  config.statistical_risk.distribution_calibrated = true;

  config.path_reuse.reuse_max_s = 5.0;
  config.path_reuse.robot_state_max_age_s = 0.2;
  config.path_reuse.cable_state_max_age_s = 0.2;
  config.path_reuse.cable_telemetry_max_age_s = 0.2;
  config.path_reuse.execution_tracking_max_age_s = 0.2;
  config.path_reuse.lease_monitor_period_s = 0.5;
  config.path_reuse.lease_renewal_margin_s = 1.0;

  config.search.maximum_active_labels = 1000;
  config.search.equivalent_label_cost_tolerance_m = 1.0e-9;
  config.search.xy_resolution_m = 0.5;
  config.search.heading_resolution_rad = 0.087;
  config.search.cable_lag_resolution_rad = 0.087;
  config.search.reference_progress_resolution_m = 0.5;
  config.search.reference_progress_backward_tolerance_m = 0.2;
  config.search.reference_progress_maximum_ratio = 1.2;
  config.search.reference_progress_forward_slack_m = 0.1;
  config.search.reference_progress_distance_scale_m = 1.0;
  config.search.reference_progress_heading_scale_rad = 0.5;
  config.search.reference_progress_heading_weight = 1.0;
  config.search.reference_progress_association_score_tolerance = 1.0e-10;
  config.search.collision_sweep_margin_m = 0.1;
  config.search.cable_sweep_margin_m = 0.1;
  config.search.path_length_cost_weight = 1.0;
  config.search.path_curvature_cost_weight = 1.0;
  config.search.touchdown_center_cost_weight = 1.0;
  config.search.touchdown_margin_cost_weight = 1.0;
  config.search.robot_terrain_cost_weight = 1.0;
  config.task.scout_policy_version = 36U;
  config.task.scout_minimum_map_confidence = 0.5;
  config.task.scout_sample_interval_m = 1.0;
  config.task.scout_merge_distance_m = 0.1;
  config.task.scout_minimum_safe_distance_m = 2.0;
  config.task.scout_planning_lead_time_s = 5.0;
  config.task.scout_average_velocity_mps = 1.0;
  config.task.scout_urgency_hysteresis_distance_m = 0.25;
  config.task.scout_urgency_hysteresis_time_s = 0.5;
  config.task.scout_sensor_coverage_radius_m = 1.5;
  config.task.scout_corridor_half_width_m = 3.0;
  config.task.communication_max_m = 50.0;
  config.task.scout_desired_distance_m = 20.0;
  config.task.scout_continue_distance_m = 40.0;
  config.task.scout_stop_distance_m = 45.0;
  config.task.scout_blocking_priority_weight = 100.0;
  config.task.scout_information_value_weight = 10.0;
  config.task.scout_forward_progress_weight = 1.0;
  config.task.scout_arrival_cost_weight = 1.0;
  config.task.scout_request_timeout_s = 30.0;
  config.task.laying_success_ratio_target = 0.8;
  return config;
}

void valid_production_profile_is_accepted() {
  const auto result = validate_parameters(valid_production_config());
  std::string details;
  for (const auto& item : result.issues) {
    details += " [" + item.code + ":" + item.path + "]";
  }
  require(result.valid && !result.non_production && result.issues.empty(),
          "complete production profile was rejected" + details);
}

void production_requires_robot_roughness_capability() {
  ParameterConfig config = valid_production_config();
  config.robot.maximum_roughness_m.reset();
  const ParameterValidationResult missing_result = validate_parameters(config);
  require(std::any_of(missing_result.issues.begin(), missing_result.issues.end(),
                      [](const ParameterIssue& issue) {
                        return issue.path == "robot.maximum_roughness_m";
                      }),
          "production accepted a missing robot roughness capability");

  config = valid_production_config();
  config.robot.maximum_roughness_m = -0.01;
  const ParameterValidationResult negative_result = validate_parameters(config);
  require(std::any_of(negative_result.issues.begin(), negative_result.issues.end(),
                      [](const ParameterIssue& issue) {
                        return issue.path == "robot.maximum_roughness_m";
                      }),
          "production accepted a negative robot roughness capability");
}

void missing_calibration_is_structured_failure() {
  auto config = valid_production_config();
  config.cable.calibration_dataset_id.clear();
  config.statistical_risk.distribution_calibrated = false;
  const auto result = validate_parameters(config);
  require(!result.valid, "uncalibrated production profile was accepted");
  const bool missing_dataset = std::any_of(
      result.issues.begin(), result.issues.end(), [](const ParameterIssue& issue) {
        return issue.code == "MISSING_REQUIRED_PARAMETER" &&
               issue.path == "cable.calibration_dataset_id";
      });
  const bool missing_coverage = std::any_of(
      result.issues.begin(), result.issues.end(), [](const ParameterIssue& issue) {
        return issue.code == "RISK_DISTRIBUTION_NOT_CALIBRATED";
      });
  require(missing_dataset && missing_coverage,
          "production failure did not expose structured calibration reasons");
}

void non_production_profile_is_explicitly_marked() {
  ParameterConfig config;
  config.profile_id = "simulation-example";
  config.mode = ParameterProfileMode::non_production_capability_profile;
  config.operating_domain_id = "synthetic-level1/v1";
  const auto result = validate_parameters(
      config, ParameterProfileMode::non_production_capability_profile);
  require(result.valid, "explicit non-production profile was rejected");
  require(result.non_production &&
              result.mode == ParameterProfileMode::non_production_capability_profile,
          "non-production validation lost its explicit marker");
  const std::string encoded = serialize_parameter_config(config);
  require(encoded.find("mode:non_production_capability_profile") != std::string::npos,
          "serialized profile omitted non-production marker");
}

void domains_and_risks_are_not_silently_shared() {
  auto config = valid_production_config();
  config.spatial_domains.cable_corridor_id =
      config.spatial_domains.robot_operating_area_id;
  const auto result = validate_parameters(config);
  const bool found = std::any_of(
      result.issues.begin(), result.issues.end(), [](const ParameterIssue& issue) {
        return issue.code == "SPATIAL_DOMAIN_SEMANTICS_SHARED";
      });
  require(found, "shared robot/cable domain semantics were accepted");
}

void robot_collision_risk_has_an_independent_production_gate() {
  auto config = valid_production_config();
  config.robot_collision_risk.epsilon_robot.reset();
  config.terrain_gradient_risk.epsilon_local = 0.01;
  config.statistical_risk.epsilon_point = 0.01;
  config.task.laying_success_ratio_target = 0.99;

  const auto result = validate_parameters(config);
  const bool missing_collision_epsilon = std::any_of(
      result.issues.begin(), result.issues.end(), [](const ParameterIssue& issue) {
        return issue.code == "MISSING_REQUIRED_PARAMETER" &&
               issue.path == "robot_collision_risk.epsilon_robot";
      });
  require(!result.valid && missing_collision_epsilon,
          "other risk parameters substituted for robot collision epsilon");
}

void step_classification_parameters_are_production_gated() {
  auto missing = valid_production_config();
  missing.robot.minimum_step_crossing_alignment.reset();
  const auto missing_result = validate_parameters(missing);
  const bool missing_alignment = std::any_of(
      missing_result.issues.begin(), missing_result.issues.end(),
      [](const ParameterIssue& issue) {
        return issue.code == "MISSING_REQUIRED_PARAMETER" &&
               issue.path == "robot.minimum_step_crossing_alignment";
      });
  require(!missing_result.valid && missing_alignment,
          "production accepted a missing step direction threshold");

  auto inconsistent = valid_production_config();
  inconsistent.robot.step_alignment_transition_band = 0.25;
  const auto inconsistent_result = validate_parameters(inconsistent);
  const bool invalid_band = std::any_of(
      inconsistent_result.issues.begin(), inconsistent_result.issues.end(),
      [](const ParameterIssue& issue) {
        return issue.code == "INCONSISTENT_CAPABILITY_PARAMETER" &&
               issue.path == "robot.step_alignment_transition_band";
      });
  require(!inconsistent_result.valid && invalid_band,
          "an invalid step direction transition band was accepted");
}

void reference_progress_parameters_are_production_gated() {
  auto missing = valid_production_config();
  missing.search.reference_progress_backward_tolerance_m.reset();
  const auto missing_result = validate_parameters(missing);
  const bool missing_tolerance = std::any_of(
      missing_result.issues.begin(), missing_result.issues.end(),
      [](const ParameterIssue& issue) {
        return issue.code == "MISSING_REQUIRED_PARAMETER" &&
               issue.path ==
                   "search.reference_progress_backward_tolerance_m";
      });
  require(!missing_result.valid && missing_tolerance,
          "production accepted missing reference progress association bounds");

  auto invalid = valid_production_config();
  invalid.search.reference_progress_heading_weight = 0.0;
  const auto invalid_result = validate_parameters(invalid);
  const bool invalid_heading_weight = std::any_of(
      invalid_result.issues.begin(), invalid_result.issues.end(),
      [](const ParameterIssue& issue) {
        return issue.code == "INVALID_NUMERIC_PARAMETER" &&
               issue.path == "search.reference_progress_heading_weight";
      });
  require(!invalid_result.valid && invalid_heading_weight,
          "production accepted a progress association that ignores heading");
}

void mechanical_label_dominance_parameters_are_production_gated() {
  auto missing = valid_production_config();
  missing.search.equivalent_label_cost_tolerance_m.reset();
  const auto missing_result = validate_parameters(missing);
  const bool missing_tolerance = std::any_of(
      missing_result.issues.begin(), missing_result.issues.end(),
      [](const ParameterIssue& issue) {
        return issue.code == "MISSING_REQUIRED_PARAMETER" &&
               issue.path ==
                   "search.equivalent_label_cost_tolerance_m";
      });
  require(!missing_result.valid && missing_tolerance,
          "production accepted a missing mechanical-label cost tolerance");

  auto invalid = valid_production_config();
  invalid.search.equivalent_label_cost_tolerance_m = -1.0e-12;
  const auto invalid_result = validate_parameters(invalid);
  const bool invalid_tolerance = std::any_of(
      invalid_result.issues.begin(), invalid_result.issues.end(),
      [](const ParameterIssue& issue) {
        return issue.code == "INVALID_NUMERIC_PARAMETER" &&
               issue.path ==
                   "search.equivalent_label_cost_tolerance_m";
      });
  require(!invalid_result.valid && invalid_tolerance,
          "production accepted a negative mechanical-label cost tolerance");
}

void search_cost_weights_are_production_gated() {
  auto missing = valid_production_config();
  missing.search.path_length_cost_weight.reset();
  const auto missing_result = validate_parameters(missing);
  require(
      !missing_result.valid &&
          std::any_of(missing_result.issues.begin(),
                      missing_result.issues.end(),
                      [](const ParameterIssue& issue) {
                        return issue.code == "MISSING_REQUIRED_PARAMETER" &&
                               issue.path ==
                                   "search.path_length_cost_weight";
                      }),
      "production accepted a missing path-length cost weight");

  auto invalid = valid_production_config();
  invalid.search.touchdown_margin_cost_weight = -1.0e-12;
  const auto invalid_result = validate_parameters(invalid);
  require(
      !invalid_result.valid &&
          std::any_of(invalid_result.issues.begin(),
                      invalid_result.issues.end(),
                      [](const ParameterIssue& issue) {
                        return issue.code == "INVALID_NUMERIC_PARAMETER" &&
                               issue.path ==
                                   "search.touchdown_margin_cost_weight";
                      }),
      "production accepted a negative touchdown margin cost weight");
}

void cable_uncertainty_parameters_are_production_gated() {
  auto missing = valid_production_config();
  missing.cable.lag_angle_process_variance_per_m_rad2.reset();
  const auto missing_result = validate_parameters(missing);
  const bool missing_process_noise = std::any_of(
      missing_result.issues.begin(), missing_result.issues.end(),
      [](const ParameterIssue& issue) {
        return issue.code == "MISSING_REQUIRED_PARAMETER" &&
               issue.path ==
                   "cable.lag_angle_process_variance_per_m_rad2";
      });
  require(!missing_result.valid && missing_process_noise,
          "production accepted missing cable process noise");

  auto non_psd = valid_production_config();
  non_psd.cable.touchdown_process_noise_per_m_m2.xy_m2 = 0.01;
  const auto non_psd_result = validate_parameters(non_psd);
  const bool covariance_rejected = std::any_of(
      non_psd_result.issues.begin(), non_psd_result.issues.end(),
      [](const ParameterIssue& issue) {
        return issue.code == "INVALID_COVARIANCE_PARAMETER" &&
               issue.path == "cable.touchdown_process_noise_m2_per_m";
      });
  require(!non_psd_result.valid && covariance_rejected,
          "production accepted non-PSD cable process noise");

  auto subtly_non_psd = valid_production_config();
  subtly_non_psd.cable.touchdown_process_noise_per_m_m2.xx_m2 = 1.0e-9;
  subtly_non_psd.cable.touchdown_process_noise_per_m_m2.xy_m2 = 1.0e-6;
  subtly_non_psd.cable.touchdown_process_noise_per_m_m2.yy_m2 = 1.0e-9;
  const auto subtle_result = validate_parameters(subtly_non_psd);
  require(!subtle_result.valid &&
              std::any_of(subtle_result.issues.begin(),
                          subtle_result.issues.end(),
                          [](const ParameterIssue& issue) {
                            return issue.code ==
                                   "INVALID_COVARIANCE_PARAMETER";
                          }),
          "production accepted small-scale non-PSD cable process noise");
}

void scout_coordination_parameters_are_loaded_gated_and_versioned() {
  const ParameterConfig config = valid_production_config();
  const ScoutCoordinationParameters scout =
      make_scout_coordination_parameters(config);
  require(valid(scout) && scout.policy_version == 36U &&
              scout.parameter_profile_id == "prod-test-v1" &&
              scout.operating_domain_id == "tank-v1" &&
              scout.communication_max_distance_m == 50.0 &&
              scout.continue_scout_distance_m == 40.0 &&
              scout.stop_scout_distance_m == 45.0 &&
              scout.request_timeout.nanoseconds == 30'000'000'000,
          "scout coordinator was not built from the versioned profile");

  const ParameterConfig round_trip =
      load_parameter_config(serialize_parameter_config(config));
  require(round_trip.task.scout_policy_version ==
              config.task.scout_policy_version &&
              round_trip.task.scout_corridor_half_width_m ==
                  config.task.scout_corridor_half_width_m &&
              round_trip.task.scout_arrival_cost_weight ==
                  config.task.scout_arrival_cost_weight &&
              round_trip.robot.length_m == config.robot.length_m &&
              round_trip.execution.maximum_ground_speed_mps ==
                  config.execution.maximum_ground_speed_mps &&
              round_trip.cable.release_point_x_m ==
                  config.cable.release_point_x_m &&
              round_trip.search.maximum_active_labels ==
                  config.search.maximum_active_labels &&
              round_trip.statistical_risk.sensor_health_modes ==
                  config.statistical_risk.sensor_health_modes,
          "complete canonical parameter profile did not round-trip");

  ParameterConfig missing = config;
  missing.task.scout_stop_distance_m.reset();
  const ParameterValidationResult missing_result = validate_parameters(missing);
  require(!missing_result.valid &&
              std::any_of(missing_result.issues.begin(),
                          missing_result.issues.end(),
                          [](const ParameterIssue& issue) {
                            return issue.code == "MISSING_REQUIRED_PARAMETER" &&
                                   issue.path ==
                                       "task.scout_stop_distance_m";
                          }),
          "production accepted a missing scout hysteresis threshold");

  ParameterConfig inconsistent = config;
  inconsistent.task.scout_stop_distance_m = 55.0;
  const ParameterValidationResult inconsistent_result =
      validate_parameters(inconsistent);
  require(!inconsistent_result.valid &&
              std::any_of(inconsistent_result.issues.begin(),
                          inconsistent_result.issues.end(),
                          [](const ParameterIssue& issue) {
                            return issue.code ==
                                       "INCONSISTENT_SCOUT_DISTANCE_POLICY";
                          }),
          "production accepted scout stop distance beyond communication max");

  ParameterConfig unrepresentable = config;
  unrepresentable.task.scout_request_timeout_s =
      static_cast<double>(std::numeric_limits<std::int64_t>::max()) * 1.0e-9;
  const ParameterValidationResult unrepresentable_result =
      validate_parameters(unrepresentable);
  require(!unrepresentable_result.valid &&
              std::any_of(unrepresentable_result.issues.begin(),
                          unrepresentable_result.issues.end(),
                          [](const ParameterIssue& issue) {
                            return issue.code == "INVALID_SCOUT_PARAMETER" &&
                                   issue.path ==
                                       "task.scout_request_timeout_s";
                          }),
          "rounded int64 timeout boundary was accepted");

  ParameterConfig representable = config;
  representable.task.scout_request_timeout_s = 1.0e9;
  require(validate_parameters(representable).valid,
          "large but representable scout timeout was rejected");
}

void loader_is_deterministic_and_rejects_unknown_keys() {
  cable_uncertainty_parameters_are_production_gated();
  const std::string input =
      "schema_version: parameter-config/v1\n"
      "profile_id: parser-test\n"
      "mode: non_production_capability_profile\n"
      "operating_domain_id: synthetic/v1\n"
      "robot.maximum_ground_speed_mps: 0.5 # SI m/s\n"
      "search.equivalent_label_cost_tolerance_m: 1e-9\n"
      "search.reference_progress_backward_tolerance_m: 0.2\n"
      "search.reference_progress_maximum_ratio: 1.2\n"
      "search.reference_progress_forward_slack_m: 0.1\n"
      "search.reference_progress_distance_scale_m: 1.0\n"
      "search.reference_progress_heading_scale_rad: 0.5\n"
      "search.reference_progress_heading_weight: 1.0\n"
      "search.reference_progress_association_score_tolerance: 1e-10\n"
      "search.path_length_cost_weight: 2.0\n"
      "search.path_curvature_cost_weight: 3.0\n"
      "search.touchdown_center_cost_weight: 4.0\n"
      "search.touchdown_margin_cost_weight: 5.0\n"
      "search.robot_terrain_cost_weight: 6.0\n"
      "cable.touchdown_distance_variance_m2: 0.01\n"
      "cable.direction_response_length_variance_m2: 0.02\n"
      "cable.lag_angle_process_variance_per_m_rad2: 0.001\n"
      "cable.touchdown_process_noise_xx_m2_per_m: 0.002\n"
      "cable.touchdown_process_noise_xy_m2_per_m: 0.0005\n"
      "cable.touchdown_process_noise_yy_m2_per_m: 0.003\n"
      "cable.curvature_evaluation_spacing_m: 0.5\n"
      "cable.medium_support_proxy_range_m: 0.1\n"
      "cable.minimum_terrain_confidence: 0.75\n";
  const auto config = load_parameter_config(input);
  require(config.profile_id == "parser-test" &&
              config.mode == ParameterProfileMode::non_production_capability_profile &&
               config.robot.maximum_ground_speed_mps.value() == 0.5 &&
               config.search.equivalent_label_cost_tolerance_m == 1.0e-9 &&
              config.search.reference_progress_backward_tolerance_m == 0.2 &&
              config.search.reference_progress_maximum_ratio == 1.2 &&
              config.search.reference_progress_forward_slack_m == 0.1 &&
              config.search.reference_progress_distance_scale_m == 1.0 &&
              config.search.reference_progress_heading_scale_rad == 0.5 &&
              config.search.reference_progress_heading_weight == 1.0 &&
               config.search.reference_progress_association_score_tolerance ==
                   1.0e-10 &&
               config.search.path_length_cost_weight == 2.0 &&
               config.search.path_curvature_cost_weight == 3.0 &&
               config.search.touchdown_center_cost_weight == 4.0 &&
               config.search.touchdown_margin_cost_weight == 5.0 &&
               config.search.robot_terrain_cost_weight == 6.0 &&
              config.cable.touchdown_distance_variance_m2 == 0.01 &&
              config.cable.direction_response_length_variance_m2 == 0.02 &&
              config.cable.lag_angle_process_variance_per_m_rad2 == 0.001 &&
              config.cable.touchdown_process_noise_per_m_m2.xx_m2 == 0.002 &&
              config.cable.touchdown_process_noise_per_m_m2.xy_m2 == 0.0005 &&
              config.cable.touchdown_process_noise_per_m_m2.yy_m2 == 0.003 &&
              config.cable.curvature_evaluation_spacing_m == 0.5 &&
              config.cable.medium_support_proxy_range_m == 0.1 &&
              config.cable.minimum_terrain_confidence == 0.75,
          "parameter loader changed parsed values");
  require(serialize_parameter_config(config) == serialize_parameter_config(config),
          "parameter serialization was not deterministic");
  bool rejected = false;
  try {
    static_cast<void>(load_parameter_config("unknown.key: 1\n"));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected, "unknown parameter key was accepted");
}

void loader_rejects_ambiguous_scalar_values_and_duplicates() {
  const auto rejects = [](const std::string& input) {
    try {
      static_cast<void>(load_parameter_config(input));
    } catch (const std::invalid_argument&) {
      return true;
    }
    return false;
  };

  require(rejects("mode: typo-production\n"),
          "unknown profile mode was silently mapped to production");
  require(rejects("robot.left_track_support_defined: maybe\n"),
          "ambiguous boolean was silently mapped to false");
  require(rejects("search.maximum_active_labels: -1\n"),
          "negative unsigned budget was accepted");
  require(rejects("search.maximum_active_labels: 12labels\n"),
          "trailing unsigned characters were accepted");
  require(rejects("task.scout_policy_version: +7\n"),
          "signed unsigned value was accepted");
  require(rejects("task.scout_policy_version: 18446744073709551616\n"),
          "overflowing unsigned value was accepted");
  require(rejects("profile_id: first\nprofile_id: second\n"),
          "duplicate scalar key used last-write-wins");
  require(rejects("robot:\n  width_m: 1\nrobot.width_m: 2\n"),
          "indented and dotted duplicate scalar keys were not canonicalized");

  const auto repeated = load_parameter_config(
      "statistical_risk.sensor_health_mode: imu\n"
      "statistical_risk.sensor_health_mode: depth\n");
  require(repeated.statistical_risk.sensor_health_modes.size() == 2U &&
              repeated.statistical_risk.sensor_health_modes[0] == "imu" &&
              repeated.statistical_risk.sensor_health_modes[1] == "depth",
          "repeatable sensor health modes did not preserve append order");
}

void cable_laying_support_thresholds_are_production_gated() {
  auto missing = valid_production_config();
  missing.cable.medium_support_proxy_range_m.reset();
  missing.cable.minimum_terrain_confidence.reset();
  missing.cable.curvature_evaluation_spacing_m.reset();
  const auto missing_result = validate_parameters(missing);
  require(!missing_result.valid &&
              std::any_of(
                  missing_result.issues.begin(), missing_result.issues.end(),
                  [](const ParameterIssue& issue) {
                    return issue.code == "MISSING_REQUIRED_PARAMETER" &&
                           issue.path ==
                               "cable.medium_support_proxy_range_m";
                  }),
          "production accepted a missing medium support-proxy threshold");
  require(std::any_of(
              missing_result.issues.begin(), missing_result.issues.end(),
              [](const ParameterIssue& issue) {
                return issue.code == "MISSING_REQUIRED_PARAMETER" &&
                       issue.path == "cable.minimum_terrain_confidence";
              }),
          "production accepted a missing cable terrain-confidence threshold");
  require(std::any_of(
              missing_result.issues.begin(), missing_result.issues.end(),
              [](const ParameterIssue& issue) {
                return issue.code == "MISSING_REQUIRED_PARAMETER" &&
                       issue.path ==
                           "cable.curvature_evaluation_spacing_m";
              }),
          "production accepted a missing curvature evaluation spacing");

  auto inconsistent = valid_production_config();
  inconsistent.cable.medium_support_proxy_range_m = 0.2;
  const auto inconsistent_result = validate_parameters(inconsistent);
  require(!inconsistent_result.valid &&
              std::any_of(
                  inconsistent_result.issues.begin(),
                  inconsistent_result.issues.end(),
                  [](const ParameterIssue& issue) {
                    return issue.code == "INCONSISTENT_CABLE_PARAMETER" &&
                           issue.path ==
                               "cable.medium_support_proxy_range_m";
                  }),
          "medium support threshold was not kept below the hard maximum");
}

void laying_success_ratio_does_not_derive_pointwise_epsilon() {
  // Design: 18.2.4-13
  auto independent = valid_production_config();
  independent.task.laying_success_ratio_target = 0.8;
  independent.statistical_risk.epsilon_point = 0.05;
  const auto accepted = validate_parameters(independent);
  require(accepted.valid &&
              independent.task.laying_success_ratio_target == 0.8 &&
              independent.statistical_risk.epsilon_point == 0.05,
          "task success ratio was converted into a pointwise risk budget");

  independent.statistical_risk.epsilon_point.reset();
  const auto missing_point_risk = validate_parameters(independent);
  require(!missing_point_risk.valid &&
              std::any_of(
                  missing_point_risk.issues.begin(),
                  missing_point_risk.issues.end(),
                  [](const ParameterIssue& issue) {
                    return issue.code == "MISSING_REQUIRED_PARAMETER" &&
                           issue.path == "statistical_risk.epsilon_point";
                  }),
          "laying success ratio silently supplied a missing epsilon_point");
}

void t60_checks_all_optional_numbers_in_non_production() {
  const auto rejects = [](auto mutate, const std::string& expected_path) {
    auto config = valid_production_config();
    config.mode = ParameterProfileMode::non_production_capability_profile;
    mutate(config);
    const auto result = validate_parameters(
        config, ParameterProfileMode::non_production_capability_profile);
    require(!result.valid && result.non_production &&
                std::any_of(result.issues.begin(), result.issues.end(),
                            [&](const ParameterIssue& issue) {
                              return issue.code == "INVALID_NUMERIC_PARAMETER" &&
                                     issue.path == expected_path;
                            }),
            "non-production numeric finite/range gate missed " + expected_path);
  };
  rejects([](ParameterConfig& config) {
            config.execution.terminal_speed_mps =
                std::numeric_limits<double>::quiet_NaN();
          },
          "execution.terminal_speed_mps");
  rejects([](ParameterConfig& config) {
            config.cable.touchdown_distance_m = -1.0;
          },
          "cable.touchdown_distance_m");
  rejects([](ParameterConfig& config) {
            config.statistical_risk.maximum_candidate_length_m =
                std::numeric_limits<double>::infinity();
          },
          "statistical_risk.maximum_candidate_length_m");
  rejects([](ParameterConfig& config) {
            config.search.path_curvature_cost_weight = -0.1;
          },
          "search.path_curvature_cost_weight");
  rejects([](ParameterConfig& config) {
            config.robot.length_m = std::numeric_limits<double>::infinity();
          },
          "robot.length_m");
  rejects([](ParameterConfig& config) {
            config.terrain_gradient_risk.epsilon_local =
                std::numeric_limits<double>::quiet_NaN();
          },
          "terrain_gradient_risk.epsilon_local");
  rejects([](ParameterConfig& config) {
            config.robot_collision_risk.epsilon_robot =
                std::numeric_limits<double>::infinity();
          },
          "robot_collision_risk.epsilon_robot");
  rejects([](ParameterConfig& config) {
            config.path_reuse.reuse_max_s = -1.0;
          },
          "path_reuse.reuse_max_s");
  rejects([](ParameterConfig& config) {
            config.task.laying_success_ratio_target = -1.0;
          },
          "task.laying_success_ratio_target");
}

void t60_production_gates_geometry_models_domains_and_sensor_modes() {
  auto missing = valid_production_config();
  missing.robot.length_m.reset();
  missing.execution.terminal_speed_mps.reset();
  missing.task.laying_success_ratio_target.reset();
  const auto missing_result = validate_parameters(missing);
  require(!missing_result.valid &&
              std::any_of(missing_result.issues.begin(), missing_result.issues.end(),
                          [](const ParameterIssue& issue) {
                            return issue.code == "MISSING_REQUIRED_PARAMETER" &&
                                   issue.path == "robot.length_m";
                          }) &&
              std::any_of(missing_result.issues.begin(), missing_result.issues.end(),
                          [](const ParameterIssue& issue) {
                            return issue.path == "execution.terminal_speed_mps";
                          }) &&
              std::any_of(missing_result.issues.begin(), missing_result.issues.end(),
                          [](const ParameterIssue& issue) {
                            return issue.path == "task.laying_success_ratio_target";
                          }),
          "production did not require T60 capability and task fields");

  auto wrong_domain = valid_production_config();
  wrong_domain.terrain_gradient_risk.operating_domain_id = "other-domain";
  const auto wrong_domain_result = validate_parameters(wrong_domain);
  require(std::any_of(wrong_domain_result.issues.begin(),
                      wrong_domain_result.issues.end(),
                      [](const ParameterIssue& issue) {
                        return issue.code == "VERSION_DOMAIN_MISMATCH" &&
                               issue.path == "terrain_gradient_risk.operating_domain_id";
                      }),
          "terrain gradient risk domain mismatch was accepted");

  auto pending = valid_production_config();
  pending.terrain_gradient_risk.coverage_model = "gaussian_pending_calibration";
  const auto pending_result = validate_parameters(pending);
  require(std::any_of(pending_result.issues.begin(), pending_result.issues.end(),
                      [](const ParameterIssue& issue) {
                        return issue.code == "RISK_MODEL_NOT_CALIBRATED";
                      }),
          "pending terrain coverage model was accepted");

  auto sensors = valid_production_config();
  sensors.statistical_risk.sensor_health_modes.clear();
  const auto empty_sensor_result = validate_parameters(sensors);
  require(std::any_of(empty_sensor_result.issues.begin(),
                      empty_sensor_result.issues.end(),
                      [](const ParameterIssue& issue) {
                        return issue.code == "MISSING_SENSOR_HEALTH_MODE";
                      }),
          "empty sensor health mode set was accepted");

  sensors.statistical_risk.sensor_health_modes = {"imu", "imu"};
  const auto duplicate_sensor_result = validate_parameters(sensors);
  require(std::any_of(duplicate_sensor_result.issues.begin(),
                      duplicate_sensor_result.issues.end(),
                      [](const ParameterIssue& issue) {
                        return issue.code == "DUPLICATE_SENSOR_HEALTH_MODE";
                      }),
          "duplicate sensor health mode was accepted");
  sensors.statistical_risk.sensor_health_modes = {"unknown"};
  const auto unknown_sensor_result = validate_parameters(sensors);
  require(std::any_of(unknown_sensor_result.issues.begin(),
                      unknown_sensor_result.issues.end(),
                      [](const ParameterIssue& issue) {
                        return issue.code == "UNSUPPORTED_SENSOR_HEALTH_MODE";
                      }),
          "unknown sensor health mode was accepted");

  auto low_target = valid_production_config();
  low_target.task.laying_success_ratio_target = 0.79;
  const auto low_target_result = validate_parameters(low_target);
  require(std::any_of(low_target_result.issues.begin(), low_target_result.issues.end(),
                      [](const ParameterIssue& issue) {
                        return issue.code == "INVALID_TASK_PARAMETER" &&
                               issue.path == "task.laying_success_ratio_target";
                      }),
          "production accepted a laying success target below 0.8");

  auto mismatched_versions = valid_production_config();
  mismatched_versions.terrain_gradient_risk.calibration_dataset_id =
      "terrain-dataset-v2";
  const auto mismatched_versions_result = validate_parameters(mismatched_versions);
  require(std::any_of(mismatched_versions_result.issues.begin(),
                      mismatched_versions_result.issues.end(),
                      [](const ParameterIssue& issue) {
                        return issue.code == "VERSION_DEPENDENCY_MISMATCH" &&
                               issue.path ==
                                   "terrain_gradient_risk.calibration_dataset_id";
                      }),
          "mismatched terrain model provenance versions were accepted");

  auto unknown_model = valid_production_config();
  unknown_model.terrain_gradient_risk.coverage_model = "future_model";
  const auto unknown_model_result = validate_parameters(unknown_model);
  require(std::any_of(unknown_model_result.issues.begin(),
                      unknown_model_result.issues.end(),
                      [](const ParameterIssue& issue) {
                        return issue.code == "UNSUPPORTED_RISK_MODEL";
                      }),
          "unknown terrain coverage model was accepted");
}

void spatial_domain_parameters_bind_to_runtime_snapshot() {
  ParameterConfig config = valid_production_config();
  VersionedPlanningSnapshot snapshot;
  snapshot.robot_operating_area =
      {1U, "robot-area", {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}}};
  snapshot.cable_corridor =
      {2U, "cable-corridor", {{0.0, 0.0}, {2.0, 0.0}, {0.0, 2.0}}};
  require(validate_spatial_domain_snapshot(config, snapshot).valid,
          "matching spatial-domain parameter versions were rejected");

  config.spatial_domains.cable_corridor_version = "cable-corridor-v2";
  require(!validate_spatial_domain_snapshot(config, snapshot).valid,
          "opaque non-comparable spatial-domain version was accepted");
  config.spatial_domains.cable_corridor_version = "3";
  require(!validate_spatial_domain_snapshot(config, snapshot).valid,
          "mismatched cable-corridor snapshot version was accepted");
  config.spatial_domains.cable_corridor_version = "2";
  config.spatial_domains.robot_operating_area_id = "another-area";
  require(!validate_spatial_domain_snapshot(config, snapshot).valid,
          "mismatched robot operating-area id was accepted");
}

}  // namespace

int main() {
  try {
    valid_production_profile_is_accepted();
    production_requires_robot_roughness_capability();
    missing_calibration_is_structured_failure();
    non_production_profile_is_explicitly_marked();
    domains_and_risks_are_not_silently_shared();
    robot_collision_risk_has_an_independent_production_gate();
    step_classification_parameters_are_production_gated();
    reference_progress_parameters_are_production_gated();
    mechanical_label_dominance_parameters_are_production_gated();
    search_cost_weights_are_production_gated();
    scout_coordination_parameters_are_loaded_gated_and_versioned();
    cable_laying_support_thresholds_are_production_gated();
    laying_success_ratio_does_not_derive_pointwise_epsilon();
    t60_checks_all_optional_numbers_in_non_production();
    t60_production_gates_geometry_models_domains_and_sensor_modes();
    spatial_domain_parameters_bind_to_runtime_snapshot();
    loader_is_deterministic_and_rejects_unknown_keys();
    loader_rejects_ambiguous_scalar_values_and_duplicates();
  } catch (const std::exception& error) {
    std::cerr << "[failure] seed=0x503 parameter-config/v1 units=SI error="
              << error.what() << '\n';
    return 1;
  }
  std::cout << "[pass] parameter configuration gates\n";
  return 0;
}
