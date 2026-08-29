#pragma once

#include "underwater_planner/core/data_contract.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace underwater_planner::core {

struct PoseTrackingCovariance2d {
  Covariance2dM2 position_covariance_m2;
  double x_heading_covariance_m_rad{};
  double y_heading_covariance_m_rad{};
  double heading_variance_rad2{};
};

struct CableUncertaintyCrossCovariance {
  double robot_x_initial_lag_m_rad{};
  double robot_y_initial_lag_m_rad{};
  double robot_heading_initial_lag_rad2{};
  double robot_x_touchdown_distance_m2{};
  double robot_y_touchdown_distance_m2{};
  double robot_heading_touchdown_distance_m_rad{};
  double robot_x_direction_response_length_m2{};
  double robot_y_direction_response_length_m2{};
  double robot_heading_direction_response_length_m_rad{};
  double initial_lag_touchdown_distance_m_rad{};
  double initial_lag_direction_response_length_m_rad{};
  double touchdown_distance_direction_response_length_m2{};
};

struct RobotUncertaintySample {
  double arc_length_m{};
  PoseTrackingCovariance2d pose_tracking_covariance;
  double heading_tracking_process_variance_per_m_rad2{};
  CableUncertaintyCrossCovariance cross_covariance;
};

struct CableModelParameters {
  std::uint64_t version{};
  std::string calibration_dataset_id;
  std::string operating_domain_id;
  Vector2m release_point_offset_m;
  double touchdown_distance_m{};
  double direction_response_length_m{};
  double maximum_lag_angle_rad{};
  double maximum_payout_tracking_error_mps{};
  RangeMps payout_speed_range;
  double maximum_payout_acceleration_mps2{};
  double maximum_tension_tracking_error_n{};
  RangeN tension_range;
  double search_integration_step_m{};
  double validation_integration_step_m{};
  double touchdown_distance_variance_m2{};
  double direction_response_length_variance_m2{};
  double lag_angle_process_variance_per_m_rad2{};
  Covariance2dM2 touchdown_process_noise_per_m_m2;
  std::vector<SensorHealthMode> approved_sensor_modes;
};

struct ExecutionOperatingEnvelope {
  std::uint64_t version{};
  std::string operating_domain_id;
  SpeedPayoutLimits limits;
  double maximum_payout_acceleration_tracking_error_mps2{};
  double maximum_tension_tracking_error_n{};
};

struct CableContext {
  CableTelemetry current_telemetry;
  ExecutionOperatingEnvelope execution_envelope;
  PredictionMode mode{PredictionMode::validation};
  SensorHealthMode sensor_mode{SensorHealthMode::nominal};
  std::uint64_t uncertainty_envelope_version{};
  std::uint64_t uncertainty_envelope_generator_version{};
  std::uint64_t robot_uncertainty_profile_version{};
  std::vector<RobotUncertaintySample> robot_uncertainty_profile;
};

struct CableModelDependencyVersions {
  std::uint64_t cable_model_version{};
  std::string calibration_dataset_id;
  std::uint64_t robot_path_version{};
  std::uint32_t reference_line_version{};
  std::uint64_t execution_profile_version{};
  std::uint64_t execution_operating_envelope_version{};
  std::uint64_t uncertainty_envelope_version{};
  std::uint64_t uncertainty_envelope_generator_version{};
  std::uint64_t robot_uncertainty_profile_version{};
  SensorHealthMode sensor_mode{SensorHealthMode::nominal};
  std::string operating_domain_id;
  std::string execution_operating_domain_id;
};

struct CablePrediction {
  CableState terminal_state;
  GeometricPath touchdown_path;
  std::vector<double> robot_arc_length_profile_m;
  std::vector<CableState> state_profile;
  std::optional<std::vector<Covariance2dM2>>
      touchdown_covariance_profile_m2;
  CableModelValidity validity{CableModelValidity::input_invalid};
  CableModelDependencyVersions dependencies;
  std::vector<std::string> issues;
};

struct CableMeanSample {
  Vector2m touchdown_position_m;
  double cable_heading_rad{};
  CableModelValidity validity{CableModelValidity::input_invalid};
  std::uint64_t cable_model_version{};
  std::vector<std::string> issues;
};

struct CableInverseMeanSample {
  Pose2d robot_pose;
  CableMeanSample forward_prediction;
};

struct CableModelIdentity {
  std::uint64_t version{};
  std::string calibration_dataset_id;
  std::string operating_domain_id;
};

class CableModel {
 public:
  explicit CableModel(CableModelParameters parameters);

  [[nodiscard]] CableMeanSample predict_touchdown_mean(
      const Pose2d& robot_pose, double lag_angle_rad) const;

  [[nodiscard]] CableInverseMeanSample inverse_touchdown_mean(
      Vector2m touchdown_target_m, double touchdown_heading_rad,
      double robot_heading_rad, double lag_angle_rad,
      MonotonicTime timestamp) const;

  [[nodiscard]] CableModelIdentity identity() const;

  [[nodiscard]] CablePrediction predict_search(
      const CableState& initial_state, const GeometricPath& robot_segment,
      const CableContext& context) const;

  [[nodiscard]] CablePrediction predict(const CableState& initial_state,
                                         const TimedPath& robot_path,
                                         const CableContext& context) const;

  void set_parameters(CableModelParameters parameters);
  [[nodiscard]] std::uint64_t version() const noexcept;

 private:
  CableModelParameters parameters_;
};

}  // namespace underwater_planner::core
