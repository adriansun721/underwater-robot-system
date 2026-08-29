#include "underwater_planner/core/cable_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace underwater_planner::core {
namespace {

constexpr double kPi = 3.14159265358979323846;

[[nodiscard]] double wrap_angle(double angle_rad) noexcept {
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

[[nodiscard]] bool finite(const double value) noexcept {
  return std::isfinite(value);
}

[[nodiscard]] bool valid_range(const RangeMps& range) noexcept {
  return finite(range.minimum_mps) && finite(range.maximum_mps) &&
         range.minimum_mps <= range.maximum_mps;
}

[[nodiscard]] bool valid_range(const RangeMps2& range) noexcept {
  return finite(range.minimum_mps2) && finite(range.maximum_mps2) &&
         range.minimum_mps2 <= range.maximum_mps2;
}

[[nodiscard]] bool valid_range(const RangeN& range) noexcept {
  return finite(range.minimum_n) && finite(range.maximum_n) &&
         range.minimum_n <= range.maximum_n;
}

template <std::size_t Dimension>
using CovarianceMatrix =
    std::array<std::array<double, Dimension>, Dimension>;

template <std::size_t Dimension>
[[nodiscard]] bool finite_symmetric_psd(
    CovarianceMatrix<Dimension> matrix) noexcept {
  double scale = 0.0;
  for (std::size_t row = 0; row < Dimension; ++row) {
    for (std::size_t column = 0; column < Dimension; ++column) {
      if (!finite(matrix[row][column])) {
        return false;
      }
      scale = std::max(scale, std::abs(matrix[row][column]));
    }
  }
  const double tolerance =
      scale * std::numeric_limits<double>::epsilon() *
      128.0 * static_cast<double>(Dimension);
  for (std::size_t row = 0; row < Dimension; ++row) {
    for (std::size_t column = row + 1U; column < Dimension; ++column) {
      if (std::abs(matrix[row][column] - matrix[column][row]) > tolerance) {
        return false;
      }
      const double symmetric =
          0.5 * (matrix[row][column] + matrix[column][row]);
      matrix[row][column] = symmetric;
      matrix[column][row] = symmetric;
    }
  }

  for (std::size_t pivot_column = 0; pivot_column < Dimension;
       ++pivot_column) {
    std::size_t pivot = pivot_column;
    for (std::size_t candidate = pivot_column + 1U;
         candidate < Dimension; ++candidate) {
      if (matrix[candidate][candidate] > matrix[pivot][pivot]) {
        pivot = candidate;
      }
    }
    if (pivot != pivot_column) {
      std::swap(matrix[pivot], matrix[pivot_column]);
      for (std::size_t row = 0; row < Dimension; ++row) {
        std::swap(matrix[row][pivot], matrix[row][pivot_column]);
      }
    }
    const double diagonal = matrix[pivot_column][pivot_column];
    if (diagonal < -tolerance) {
      return false;
    }
    if (diagonal <= tolerance) {
      for (std::size_t row = pivot_column + 1U; row < Dimension; ++row) {
        if (std::abs(matrix[row][pivot_column]) > tolerance) {
          return false;
        }
      }
      continue;
    }
    for (std::size_t row = pivot_column + 1U; row < Dimension; ++row) {
      for (std::size_t column = row; column < Dimension; ++column) {
        matrix[column][row] -=
            matrix[row][pivot_column] * matrix[column][pivot_column] /
            diagonal;
        matrix[row][column] = matrix[column][row];
      }
    }
  }
  return true;
}

[[nodiscard]] bool valid_covariance(
    const Covariance2dM2& covariance) noexcept {
  return finite_symmetric_psd<2U>(
      {{{covariance.xx_m2, covariance.xy_m2},
        {covariance.yx_m2, covariance.yy_m2}}});
}

[[nodiscard]] bool valid_pose_tracking_covariance(
    const PoseTrackingCovariance2d& covariance) noexcept {
  if (!valid_covariance(covariance.position_covariance_m2)) {
    return false;
  }
  const auto& position = covariance.position_covariance_m2;
  return finite_symmetric_psd<3U>(
      {{{position.xx_m2, position.xy_m2,
         covariance.x_heading_covariance_m_rad},
        {position.yx_m2, position.yy_m2,
         covariance.y_heading_covariance_m_rad},
        {covariance.x_heading_covariance_m_rad,
         covariance.y_heading_covariance_m_rad,
         covariance.heading_variance_rad2}}});
}

[[nodiscard]] bool parameters_valid(
    const CableModelParameters& parameters) noexcept {
  return parameters.version != 0U &&
         !parameters.calibration_dataset_id.empty() &&
         !parameters.operating_domain_id.empty() &&
         finite(parameters.release_point_offset_m.x_m) &&
         finite(parameters.release_point_offset_m.y_m) &&
         finite(parameters.touchdown_distance_m) &&
         parameters.touchdown_distance_m > 0.0 &&
         finite(parameters.direction_response_length_m) &&
         parameters.direction_response_length_m > 0.0 &&
         finite(parameters.maximum_lag_angle_rad) &&
         parameters.maximum_lag_angle_rad > 0.0 &&
         parameters.maximum_lag_angle_rad <= kPi &&
         finite(parameters.maximum_payout_tracking_error_mps) &&
         parameters.maximum_payout_tracking_error_mps >= 0.0 &&
         valid_range(parameters.payout_speed_range) &&
         parameters.payout_speed_range.minimum_mps >= 0.0 &&
         finite(parameters.maximum_payout_acceleration_mps2) &&
         parameters.maximum_payout_acceleration_mps2 >= 0.0 &&
         finite(parameters.maximum_tension_tracking_error_n) &&
         parameters.maximum_tension_tracking_error_n >= 0.0 &&
         valid_range(parameters.tension_range) &&
         parameters.tension_range.minimum_n >= 0.0 &&
         finite(parameters.search_integration_step_m) &&
         parameters.search_integration_step_m > 0.0 &&
         finite(parameters.validation_integration_step_m) &&
         parameters.validation_integration_step_m > 0.0 &&
         parameters.validation_integration_step_m <=
             parameters.search_integration_step_m &&
         finite(parameters.touchdown_distance_variance_m2) &&
         parameters.touchdown_distance_variance_m2 >= 0.0 &&
         finite(parameters.direction_response_length_variance_m2) &&
         parameters.direction_response_length_variance_m2 >= 0.0 &&
         finite(parameters.lag_angle_process_variance_per_m_rad2) &&
         parameters.lag_angle_process_variance_per_m_rad2 >= 0.0 &&
         valid_covariance(parameters.touchdown_process_noise_per_m_m2) &&
         !parameters.approved_sensor_modes.empty() &&
         std::all_of(parameters.approved_sensor_modes.begin(),
                     parameters.approved_sensor_modes.end(),
                     [](const SensorHealthMode mode) {
                       return mode == SensorHealthMode::nominal ||
                              mode == SensorHealthMode::approved_degraded;
                     });
}

[[nodiscard]] bool envelope_valid(
    const ExecutionOperatingEnvelope& envelope,
    const CableModelParameters& parameters) noexcept {
  const auto& limits = envelope.limits;
  return envelope.version != 0U &&
         envelope.operating_domain_id == parameters.operating_domain_id &&
         valid_range(limits.ground_speed) &&
         limits.ground_speed.minimum_mps >= 0.0 &&
         valid_range(limits.ground_acceleration) &&
         finite(limits.maximum_lateral_acceleration_mps2) &&
         limits.maximum_lateral_acceleration_mps2 >= 0.0 &&
         valid_range(limits.payout_speed) &&
         limits.payout_speed.minimum_mps >=
             parameters.payout_speed_range.minimum_mps &&
         limits.payout_speed.maximum_mps <=
             parameters.payout_speed_range.maximum_mps &&
         valid_range(limits.payout_acceleration) &&
         std::abs(limits.payout_acceleration.minimum_mps2) <=
             parameters.maximum_payout_acceleration_mps2 &&
         std::abs(limits.payout_acceleration.maximum_mps2) <=
             parameters.maximum_payout_acceleration_mps2 &&
         finite(limits.maximum_payout_tracking_error_mps) &&
         limits.maximum_payout_tracking_error_mps >= 0.0 &&
         limits.maximum_payout_tracking_error_mps <=
             parameters.maximum_payout_tracking_error_mps &&
         valid_range(limits.tension) &&
         limits.tension.minimum_n -
                 envelope.maximum_tension_tracking_error_n >=
             parameters.tension_range.minimum_n &&
         limits.tension.maximum_n +
                 envelope.maximum_tension_tracking_error_n <=
             parameters.tension_range.maximum_n &&
         finite(limits.maximum_stopping_distance_m) &&
         limits.maximum_stopping_distance_m >= 0.0 &&
         finite(envelope.maximum_payout_acceleration_tracking_error_mps2) &&
         envelope.maximum_payout_acceleration_tracking_error_mps2 >= 0.0 &&
         envelope.maximum_payout_acceleration_tracking_error_mps2 <=
             parameters.maximum_payout_acceleration_mps2 &&
         finite(envelope.maximum_tension_tracking_error_n) &&
         envelope.maximum_tension_tracking_error_n >= 0.0 &&
         envelope.maximum_tension_tracking_error_n <=
             parameters.maximum_tension_tracking_error_n;
}

[[nodiscard]] CablePrediction make_prediction_shell(
    const CableModelParameters& parameters,
    const CableContext& context) {
  CablePrediction prediction;
  prediction.dependencies.cable_model_version = parameters.version;
  prediction.dependencies.execution_operating_envelope_version =
      context.execution_envelope.version;
  prediction.dependencies.calibration_dataset_id =
      parameters.calibration_dataset_id;
  prediction.dependencies.uncertainty_envelope_version =
      context.uncertainty_envelope_version;
  prediction.dependencies.uncertainty_envelope_generator_version =
      context.uncertainty_envelope_generator_version;
  prediction.dependencies.robot_uncertainty_profile_version =
      context.robot_uncertainty_profile_version;
  prediction.dependencies.sensor_mode = context.sensor_mode;
  prediction.dependencies.operating_domain_id =
      parameters.operating_domain_id;
  prediction.dependencies.execution_operating_domain_id =
      context.execution_envelope.operating_domain_id;
  return prediction;
}

void fail(CablePrediction& prediction, const CableModelValidity validity,
          std::string issue) {
  prediction.validity = validity;
  prediction.issues.push_back(std::move(issue));
}

[[nodiscard]] bool calibrated_context_valid(
    const CableContext& context, const CableModelParameters& parameters,
    CablePrediction& prediction) {
  if (std::find(parameters.approved_sensor_modes.begin(),
                parameters.approved_sensor_modes.end(),
                context.sensor_mode) ==
      parameters.approved_sensor_modes.end()) {
    fail(prediction, CableModelValidity::sensor_mode_unapproved,
         "sensor health mode is outside the calibrated model domain");
    return false;
  }
  if (context.execution_envelope.operating_domain_id !=
      parameters.operating_domain_id) {
    fail(prediction, CableModelValidity::operating_domain_mismatch,
         "execution envelope and cable model operating domains differ");
    return false;
  }
  if (!envelope_valid(context.execution_envelope, parameters)) {
    fail(prediction, CableModelValidity::input_invalid,
         "execution envelope is outside the calibrated model domain");
    return false;
  }
  return true;
}

[[nodiscard]] double interpolate(const double left, const double right,
                                 const double ratio) noexcept {
  return left + ratio * (right - left);
}

[[nodiscard]] double interpolate_heading(const double left,
                                         const double right,
                                         const double ratio) noexcept {
  return wrap_angle(left + ratio * wrap_angle(right - left));
}

[[nodiscard]] PathPoint interpolate_geometry(const GeometricPath& path,
                                             const double arc_length_m) {
  const auto upper = std::lower_bound(
      path.points.begin(), path.points.end(), arc_length_m,
      [](const PathPoint& point, const double value) {
        return point.arc_length_m < value;
      });
  if (upper == path.points.begin()) {
    return path.points.front();
  }
  if (upper == path.points.end()) {
    return path.points.back();
  }
  const auto& right = *upper;
  const auto& left = *(upper - 1);
  const double ratio = (arc_length_m - left.arc_length_m) /
                       (right.arc_length_m - left.arc_length_m);
  return {arc_length_m,
          interpolate(left.x_m, right.x_m, ratio),
          interpolate(left.y_m, right.y_m, ratio),
          interpolate_heading(left.heading_rad, right.heading_rad, ratio),
          interpolate(left.curvature_per_m, right.curvature_per_m, ratio)};
}

[[nodiscard]] Vector2m touchdown_position(
    const PathPoint& robot, const double lag_angle_rad,
    const CableModelParameters& parameters) noexcept {
  const double cos_heading = std::cos(robot.heading_rad);
  const double sin_heading = std::sin(robot.heading_rad);
  const double release_x_m =
      robot.x_m + cos_heading * parameters.release_point_offset_m.x_m -
      sin_heading * parameters.release_point_offset_m.y_m;
  const double release_y_m =
      robot.y_m + sin_heading * parameters.release_point_offset_m.x_m +
      cos_heading * parameters.release_point_offset_m.y_m;
  const double cable_heading_rad =
      wrap_angle(robot.heading_rad + lag_angle_rad);
  return {release_x_m -
              parameters.touchdown_distance_m * std::cos(cable_heading_rad),
          release_y_m -
              parameters.touchdown_distance_m * std::sin(cable_heading_rad)};
}

struct CovariancePropagationState {
  double initial_lag_angle_variance_rad2{};
  double lag_angle_process_variance_rad2{};
  double lag_initial_sensitivity{1.0};
  double lag_response_length_sensitivity_rad_per_m{};
  Covariance2dM2 accumulated_touchdown_process_noise_m2;
};

constexpr std::size_t kRobotX = 0U;
constexpr std::size_t kRobotY = 1U;
constexpr std::size_t kRobotHeading = 2U;
constexpr std::size_t kInitialLag = 3U;
constexpr std::size_t kTouchdownDistance = 4U;
constexpr std::size_t kDirectionResponseLength = 5U;
constexpr std::size_t kJointUncertaintyDimension = 6U;
using JointInputCovariance =
    CovarianceMatrix<kJointUncertaintyDimension>;

void set_symmetric(JointInputCovariance& covariance,
                   const std::size_t row, const std::size_t column,
                   const double value) noexcept {
  covariance[row][column] = value;
  covariance[column][row] = value;
}

[[nodiscard]] JointInputCovariance joint_input_covariance(
    const RobotUncertaintySample& uncertainty,
    const CovariancePropagationState& propagation,
    const CableModelParameters& parameters) noexcept {
  JointInputCovariance covariance{};
  const auto& pose = uncertainty.pose_tracking_covariance;
  covariance[kRobotX][kRobotX] = pose.position_covariance_m2.xx_m2;
  set_symmetric(covariance, kRobotX, kRobotY,
                pose.position_covariance_m2.xy_m2);
  covariance[kRobotY][kRobotY] = pose.position_covariance_m2.yy_m2;
  set_symmetric(covariance, kRobotX, kRobotHeading,
                pose.x_heading_covariance_m_rad);
  set_symmetric(covariance, kRobotY, kRobotHeading,
                pose.y_heading_covariance_m_rad);
  covariance[kRobotHeading][kRobotHeading] = pose.heading_variance_rad2;
  covariance[kInitialLag][kInitialLag] =
      propagation.initial_lag_angle_variance_rad2;
  covariance[kTouchdownDistance][kTouchdownDistance] =
      parameters.touchdown_distance_variance_m2;
  covariance[kDirectionResponseLength][kDirectionResponseLength] =
      parameters.direction_response_length_variance_m2;

  const auto& cross = uncertainty.cross_covariance;
  set_symmetric(covariance, kRobotX, kInitialLag,
                cross.robot_x_initial_lag_m_rad);
  set_symmetric(covariance, kRobotY, kInitialLag,
                cross.robot_y_initial_lag_m_rad);
  set_symmetric(covariance, kRobotHeading, kInitialLag,
                cross.robot_heading_initial_lag_rad2);
  set_symmetric(covariance, kRobotX, kTouchdownDistance,
                cross.robot_x_touchdown_distance_m2);
  set_symmetric(covariance, kRobotY, kTouchdownDistance,
                cross.robot_y_touchdown_distance_m2);
  set_symmetric(covariance, kRobotHeading, kTouchdownDistance,
                cross.robot_heading_touchdown_distance_m_rad);
  set_symmetric(covariance, kRobotX, kDirectionResponseLength,
                cross.robot_x_direction_response_length_m2);
  set_symmetric(covariance, kRobotY, kDirectionResponseLength,
                cross.robot_y_direction_response_length_m2);
  set_symmetric(covariance, kRobotHeading, kDirectionResponseLength,
                cross.robot_heading_direction_response_length_m_rad);
  set_symmetric(covariance, kInitialLag, kTouchdownDistance,
                cross.initial_lag_touchdown_distance_m_rad);
  set_symmetric(covariance, kInitialLag, kDirectionResponseLength,
                cross.initial_lag_direction_response_length_m_rad);
  set_symmetric(covariance, kTouchdownDistance, kDirectionResponseLength,
                cross.touchdown_distance_direction_response_length_m2);
  return covariance;
}

[[nodiscard]] double projected_covariance(
    const std::array<double, kJointUncertaintyDimension>& left,
    const JointInputCovariance& covariance,
    const std::array<double, kJointUncertaintyDimension>& right) noexcept {
  double result = 0.0;
  for (std::size_t row = 0; row < kJointUncertaintyDimension; ++row) {
    for (std::size_t column = 0; column < kJointUncertaintyDimension;
         ++column) {
      result += left[row] * covariance[row][column] * right[column];
    }
  }
  return result;
}

[[nodiscard]] RobotUncertaintySample interpolate_uncertainty(
    const std::vector<RobotUncertaintySample>& profile,
    const double arc_length_m) {
  const auto upper = std::lower_bound(
      profile.begin(), profile.end(), arc_length_m,
      [](const RobotUncertaintySample& sample, const double value) {
        return sample.arc_length_m < value;
      });
  if (upper == profile.begin()) {
    return profile.front();
  }
  if (upper == profile.end()) {
    return profile.back();
  }
  const auto& right = *upper;
  const auto& left = *(upper - 1);
  const double ratio = (arc_length_m - left.arc_length_m) /
                       (right.arc_length_m - left.arc_length_m);
  const auto blend = [ratio](const double a, const double b) {
    return interpolate(a, b, ratio);
  };
  const auto& a = left.pose_tracking_covariance;
  const auto& b = right.pose_tracking_covariance;
  const auto& ac = left.cross_covariance;
  const auto& bc = right.cross_covariance;
  RobotUncertaintySample result;
  result.arc_length_m = arc_length_m;
  result.pose_tracking_covariance =
      {{blend(a.position_covariance_m2.xx_m2,
              b.position_covariance_m2.xx_m2),
        blend(a.position_covariance_m2.xy_m2,
              b.position_covariance_m2.xy_m2),
        blend(a.position_covariance_m2.yx_m2,
              b.position_covariance_m2.yx_m2),
        blend(a.position_covariance_m2.yy_m2,
              b.position_covariance_m2.yy_m2)},
       blend(a.x_heading_covariance_m_rad,
             b.x_heading_covariance_m_rad),
       blend(a.y_heading_covariance_m_rad,
             b.y_heading_covariance_m_rad),
       blend(a.heading_variance_rad2, b.heading_variance_rad2)};
  result.heading_tracking_process_variance_per_m_rad2 =
      blend(left.heading_tracking_process_variance_per_m_rad2,
            right.heading_tracking_process_variance_per_m_rad2);
  result.cross_covariance = {
      blend(ac.robot_x_initial_lag_m_rad,
            bc.robot_x_initial_lag_m_rad),
      blend(ac.robot_y_initial_lag_m_rad,
            bc.robot_y_initial_lag_m_rad),
      blend(ac.robot_heading_initial_lag_rad2,
            bc.robot_heading_initial_lag_rad2),
      blend(ac.robot_x_touchdown_distance_m2,
            bc.robot_x_touchdown_distance_m2),
      blend(ac.robot_y_touchdown_distance_m2,
            bc.robot_y_touchdown_distance_m2),
      blend(ac.robot_heading_touchdown_distance_m_rad,
            bc.robot_heading_touchdown_distance_m_rad),
      blend(ac.robot_x_direction_response_length_m2,
            bc.robot_x_direction_response_length_m2),
      blend(ac.robot_y_direction_response_length_m2,
            bc.robot_y_direction_response_length_m2),
      blend(ac.robot_heading_direction_response_length_m_rad,
            bc.robot_heading_direction_response_length_m_rad),
      blend(ac.initial_lag_touchdown_distance_m_rad,
            bc.initial_lag_touchdown_distance_m_rad),
      blend(ac.initial_lag_direction_response_length_m_rad,
            bc.initial_lag_direction_response_length_m_rad),
      blend(ac.touchdown_distance_direction_response_length_m2,
            bc.touchdown_distance_direction_response_length_m2)};
  return result;
}

[[nodiscard]] double interpolate_tracking_process_variance(
    const std::vector<RobotUncertaintySample>& profile,
    const double arc_length_m) {
  const auto upper = std::lower_bound(
      profile.begin(), profile.end(), arc_length_m,
      [](const RobotUncertaintySample& sample, const double value) {
        return sample.arc_length_m < value;
      });
  if (upper == profile.begin()) {
    return profile.front().heading_tracking_process_variance_per_m_rad2;
  }
  if (upper == profile.end()) {
    return profile.back().heading_tracking_process_variance_per_m_rad2;
  }
  const auto& right = *upper;
  const auto& left = *(upper - 1);
  const double ratio = (arc_length_m - left.arc_length_m) /
                       (right.arc_length_m - left.arc_length_m);
  return interpolate(
      left.heading_tracking_process_variance_per_m_rad2,
      right.heading_tracking_process_variance_per_m_rad2, ratio);
}

[[nodiscard]] Covariance2dM2 touchdown_covariance(
    const PathPoint& robot, const double lag_angle_rad,
    const RobotUncertaintySample& uncertainty,
    const CovariancePropagationState& propagation,
    const CableModelParameters& parameters) noexcept {
  const double heading = robot.heading_rad;
  const double cable_heading = wrap_angle(heading + lag_angle_rad);
  const double heading_jacobian_x =
      -std::sin(heading) * parameters.release_point_offset_m.x_m -
      std::cos(heading) * parameters.release_point_offset_m.y_m +
      parameters.touchdown_distance_m * std::sin(cable_heading);
  const double heading_jacobian_y =
      std::cos(heading) * parameters.release_point_offset_m.x_m -
      std::sin(heading) * parameters.release_point_offset_m.y_m -
      parameters.touchdown_distance_m * std::cos(cable_heading);
  const double lag_jacobian_x =
      parameters.touchdown_distance_m * std::sin(cable_heading);
  const double lag_jacobian_y =
      -parameters.touchdown_distance_m * std::cos(cable_heading);
  const double distance_jacobian_x = -std::cos(cable_heading);
  const double distance_jacobian_y = -std::sin(cable_heading);
  const std::array<double, kJointUncertaintyDimension> jacobian_x = {
      1.0,
      0.0,
      heading_jacobian_x,
      lag_jacobian_x * propagation.lag_initial_sensitivity,
      distance_jacobian_x,
      lag_jacobian_x *
          propagation.lag_response_length_sensitivity_rad_per_m};
  const std::array<double, kJointUncertaintyDimension> jacobian_y = {
      0.0,
      1.0,
      heading_jacobian_y,
      lag_jacobian_y * propagation.lag_initial_sensitivity,
      distance_jacobian_y,
      lag_jacobian_y *
          propagation.lag_response_length_sensitivity_rad_per_m};
  const JointInputCovariance joint =
      joint_input_covariance(uncertainty, propagation, parameters);
  const double process_xx =
      lag_jacobian_x * lag_jacobian_x *
          propagation.lag_angle_process_variance_rad2 +
      propagation.accumulated_touchdown_process_noise_m2.xx_m2;
  const double process_xy =
      lag_jacobian_x * lag_jacobian_y *
          propagation.lag_angle_process_variance_rad2 +
      propagation.accumulated_touchdown_process_noise_m2.xy_m2;
  const double process_yy =
      lag_jacobian_y * lag_jacobian_y *
          propagation.lag_angle_process_variance_rad2 +
      propagation.accumulated_touchdown_process_noise_m2.yy_m2;
  const double xy =
      projected_covariance(jacobian_x, joint, jacobian_y) + process_xy;
  return {projected_covariance(jacobian_x, joint, jacobian_x) + process_xx,
          xy, xy,
          projected_covariance(jacobian_y, joint, jacobian_y) + process_yy};
}

[[nodiscard]] double lag_angle_variance(
    const RobotUncertaintySample& uncertainty,
    const CovariancePropagationState& propagation,
    const CableModelParameters& parameters) noexcept {
  const JointInputCovariance joint =
      joint_input_covariance(uncertainty, propagation, parameters);
  std::array<double, kJointUncertaintyDimension> jacobian{};
  jacobian[kInitialLag] = propagation.lag_initial_sensitivity;
  jacobian[kDirectionResponseLength] =
      propagation.lag_response_length_sensitivity_rad_per_m;
  return projected_covariance(jacobian, joint, jacobian) +
         propagation.lag_angle_process_variance_rad2;
}

void append_sample(CablePrediction& prediction, const PathPoint& robot,
                   const double lag_angle_rad, const CableState& initial_state,
                   const CableStateKind output_kind,
                   const MonotonicTime timestamp,
                   const CableModelParameters& parameters,
                   const CovariancePropagationState* covariance_state = nullptr,
                   const RobotUncertaintySample* uncertainty = nullptr) {
  const Vector2m touchdown =
      touchdown_position(robot, lag_angle_rad, parameters);
  double touchdown_arc_length_m = 0.0;
  if (!prediction.touchdown_path.points.empty()) {
    const auto& previous = prediction.touchdown_path.points.back();
    touchdown_arc_length_m = previous.arc_length_m +
                             std::hypot(touchdown.x_m - previous.x_m,
                                        touchdown.y_m - previous.y_m);
  }
  prediction.touchdown_path.points.push_back(
      {touchdown_arc_length_m, touchdown.x_m, touchdown.y_m,
       wrap_angle(robot.heading_rad + lag_angle_rad), 0.0});
  prediction.robot_arc_length_profile_m.push_back(robot.arc_length_m);

  CableState state = initial_state;
  state.kind = output_kind;
  state.lag_angle_rad = lag_angle_rad;
  state.timestamp = timestamp;
  if (output_kind == CableStateKind::search_mean) {
    state.lag_angle_variance_rad2.reset();
  } else if (covariance_state != nullptr && uncertainty != nullptr) {
    state.lag_angle_variance_rad2 =
        lag_angle_variance(*uncertainty, *covariance_state, parameters);
  }
  prediction.state_profile.push_back(std::move(state));
  if (covariance_state != nullptr && uncertainty != nullptr) {
    prediction.touchdown_covariance_profile_m2->push_back(
        touchdown_covariance(robot, lag_angle_rad, *uncertainty,
                             *covariance_state, parameters));
  }
}

[[nodiscard]] bool propagate_mean_interval(
    CablePrediction& prediction, const GeometricPath& geometry,
    const double start_arc_length_m, const double end_arc_length_m,
    double& lag_angle_rad, const double maximum_step_m,
    const CableState& initial_state, const CableStateKind output_kind,
    const MonotonicTime start_time, const MonotonicTime end_time,
    const CableModelParameters& parameters,
    CovariancePropagationState* covariance_state = nullptr,
    const std::vector<RobotUncertaintySample>* robot_uncertainty_profile =
        nullptr) {
  const double distance_m = end_arc_length_m - start_arc_length_m;
  if (!finite(distance_m) || distance_m <= 0.0 ||
      end_time.nanoseconds < start_time.nanoseconds) {
    fail(prediction, CableModelValidity::input_invalid,
         "mean propagation interval is invalid");
    return false;
  }
  const auto step_count = static_cast<std::size_t>(
      std::max(1.0, std::ceil(distance_m / maximum_step_m)));
  const double step_m = distance_m / static_cast<double>(step_count);
  const std::int64_t duration_ns =
      end_time.nanoseconds - start_time.nanoseconds;
  for (std::size_t step = 0; step < step_count; ++step) {
    const double midpoint_arc_length_m =
        start_arc_length_m + (static_cast<double>(step) + 0.5) * step_m;
    const double curvature_per_m =
        interpolate_geometry(geometry, midpoint_arc_length_m).curvature_per_m;
    const double decay =
        std::exp(-step_m / parameters.direction_response_length_m);
    const double previous_lag_angle_rad = lag_angle_rad;
    if (covariance_state != nullptr) {
      const double decay_derivative =
          decay * step_m /
          (parameters.direction_response_length_m *
           parameters.direction_response_length_m);
      covariance_state->lag_response_length_sensitivity_rad_per_m =
          decay *
              covariance_state->lag_response_length_sensitivity_rad_per_m +
          previous_lag_angle_rad * decay_derivative -
          curvature_per_m *
              (1.0 - decay -
               parameters.direction_response_length_m * decay_derivative);
      covariance_state->lag_initial_sensitivity *= decay;
      covariance_state->lag_angle_process_variance_rad2 =
          decay * decay * covariance_state->lag_angle_process_variance_rad2 +
          parameters.lag_angle_process_variance_per_m_rad2 * step_m;
      if (robot_uncertainty_profile != nullptr) {
        covariance_state->lag_angle_process_variance_rad2 +=
            interpolate_tracking_process_variance(
                *robot_uncertainty_profile, midpoint_arc_length_m) *
            step_m;
      }
      auto& process =
          covariance_state->accumulated_touchdown_process_noise_m2;
      process.xx_m2 +=
          parameters.touchdown_process_noise_per_m_m2.xx_m2 * step_m;
      process.xy_m2 +=
          parameters.touchdown_process_noise_per_m_m2.xy_m2 * step_m;
      process.yx_m2 +=
          parameters.touchdown_process_noise_per_m_m2.yx_m2 * step_m;
      process.yy_m2 +=
          parameters.touchdown_process_noise_per_m_m2.yy_m2 * step_m;
    }
    lag_angle_rad = wrap_angle(
        lag_angle_rad * decay -
        curvature_per_m * parameters.direction_response_length_m *
            (1.0 - decay));
    if (std::abs(lag_angle_rad) > parameters.maximum_lag_angle_rad) {
      fail(prediction, CableModelValidity::lag_angle_out_of_range,
           "predicted lag angle exceeds the calibrated range");
      return false;
    }
    const double ratio = static_cast<double>(step + 1U) /
                         static_cast<double>(step_count);
    const double robot_arc_length_m =
        start_arc_length_m + static_cast<double>(step + 1U) * step_m;
    const MonotonicTime timestamp{
        start_time.nanoseconds +
        static_cast<std::int64_t>(std::llround(
            static_cast<long double>(duration_ns) * ratio))};
    const PathPoint robot =
        interpolate_geometry(geometry, robot_arc_length_m);
    if (covariance_state != nullptr && robot_uncertainty_profile != nullptr) {
      const RobotUncertaintySample uncertainty =
          interpolate_uncertainty(*robot_uncertainty_profile,
                                  robot_arc_length_m);
      append_sample(prediction, robot, lag_angle_rad, initial_state,
                    output_kind, timestamp, parameters, covariance_state,
                    &uncertainty);
    } else {
      append_sample(prediction, robot, lag_angle_rad, initial_state,
                    output_kind, timestamp, parameters);
    }
  }
  return true;
}

[[nodiscard]] bool finalize_touchdown_curvature(
    GeometricPath& touchdown_path) {
  if (touchdown_path.points.size() < 2U) {
    return false;
  }
  for (std::size_t index = 0; index < touchdown_path.points.size(); ++index) {
    const std::size_t left_index = index == 0U ? 0U : index - 1U;
    const std::size_t right_index =
        index + 1U < touchdown_path.points.size() ? index + 1U : index;
    const auto& left = touchdown_path.points[left_index];
    const auto& right = touchdown_path.points[right_index];
    const double arc_span_m = right.arc_length_m - left.arc_length_m;
    if (!finite(arc_span_m) || arc_span_m <= 0.0) {
      return false;
    }
    touchdown_path.points[index].curvature_per_m =
        wrap_angle(right.heading_rad - left.heading_rad) / arc_span_m;
  }
  return true;
}

void complete_prediction(CablePrediction& prediction) {
  if (prediction.robot_arc_length_profile_m.size() !=
          prediction.touchdown_path.points.size() ||
      !finalize_touchdown_curvature(prediction.touchdown_path)) {
    fail(prediction, CableModelValidity::input_invalid,
         "touchdown mean path contains a zero-length segment");
    return;
  }
  if (prediction.touchdown_covariance_profile_m2.has_value()) {
    if (prediction.touchdown_covariance_profile_m2->size() !=
            prediction.touchdown_path.points.size() ||
        !std::all_of(prediction.touchdown_covariance_profile_m2->begin(),
                     prediction.touchdown_covariance_profile_m2->end(),
                     valid_covariance)) {
      fail(prediction, CableModelValidity::covariance_invalid,
           "touchdown covariance propagation produced an invalid profile");
      return;
    }
  }
  prediction.terminal_state = prediction.state_profile.back();
  prediction.validity = CableModelValidity::valid;
}

[[nodiscard]] bool telemetry_valid(const CableTelemetry& telemetry) noexcept {
  return finite(telemetry.payout_speed_mps) &&
         finite(telemetry.payout_acceleration_mps2) &&
         finite(telemetry.tension_n) && telemetry.timestamp.nanoseconds >= 0 &&
         telemetry.sequence_number != 0U;
}

[[nodiscard]] bool absolute_time(const MonotonicTime start,
                                 const Duration offset,
                                 MonotonicTime& result) noexcept {
  if (start.nanoseconds < 0 || offset.nanoseconds < 0 ||
      offset.nanoseconds >
          std::numeric_limits<std::int64_t>::max() - start.nanoseconds) {
    return false;
  }
  result.nanoseconds = start.nanoseconds + offset.nanoseconds;
  return true;
}

[[nodiscard]] bool within(const double value, const RangeMps& range) noexcept {
  return value >= range.minimum_mps && value <= range.maximum_mps;
}

[[nodiscard]] bool within(const double value, const RangeMps2& range) noexcept {
  return value >= range.minimum_mps2 && value <= range.maximum_mps2;
}

[[nodiscard]] bool within(const double value, const RangeN& range) noexcept {
  return value >= range.minimum_n && value <= range.maximum_n;
}

[[nodiscard]] double lateral_acceleration_mps2(
    const double ground_speed_mps,
    const double curvature_per_m) noexcept {
  return ground_speed_mps * ground_speed_mps *
         std::abs(curvature_per_m);
}

[[nodiscard]] bool classify_execution_sample(
    const ExecutionSample& sample, const CableModelParameters& parameters,
    const ExecutionOperatingEnvelope& envelope, const double curvature_per_m,
    CablePrediction& prediction) {
  if (!finite(sample.ground_speed_mps) || sample.ground_speed_mps < 0.0 ||
      !within(sample.ground_speed_mps, envelope.limits.ground_speed) ||
      !finite(sample.ground_acceleration_mps2) ||
      !within(sample.ground_acceleration_mps2,
              envelope.limits.ground_acceleration) ||
      !finite(curvature_per_m) ||
      lateral_acceleration_mps2(sample.ground_speed_mps,
                                curvature_per_m) >
          envelope.limits.maximum_lateral_acceleration_mps2) {
    fail(prediction, CableModelValidity::motion_mode_out_of_range,
         "execution sample is outside the calibrated forward motion mode");
    return false;
  }
  if (!finite(sample.payout_speed_mps) ||
      !within(sample.payout_speed_mps, envelope.limits.payout_speed) ||
      sample.payout_speed_mps < parameters.payout_speed_range.minimum_mps ||
      sample.payout_speed_mps > parameters.payout_speed_range.maximum_mps ||
      !finite(sample.payout_acceleration_mps2) ||
      !within(sample.payout_acceleration_mps2,
              envelope.limits.payout_acceleration) ||
      std::abs(sample.payout_acceleration_mps2) >
          parameters.maximum_payout_acceleration_mps2 ||
      std::abs(sample.payout_speed_mps - sample.ground_speed_mps) >
          envelope.limits.maximum_payout_tracking_error_mps) {
    fail(prediction, CableModelValidity::payout_tracking_out_of_range,
         "execution sample payout is outside the calibrated tracking range");
    return false;
  }
  if (!finite(sample.tension_setpoint_n) ||
      !within(sample.tension_setpoint_n, envelope.limits.tension) ||
      sample.tension_setpoint_n -
                  envelope.maximum_tension_tracking_error_n <
              parameters.tension_range.minimum_n ||
      sample.tension_setpoint_n +
                  envelope.maximum_tension_tracking_error_n >
              parameters.tension_range.maximum_n) {
    fail(prediction, CableModelValidity::tension_out_of_range,
         "execution sample tension is outside the calibrated range");
    return false;
  }
  return true;
}

[[nodiscard]] bool lateral_acceleration_interval_within_limit(
    const GeometricPath& geometry, const ExecutionSample& left,
    const ExecutionSample& right, const double maximum_mps2) {
  const double execution_span_m =
      right.arc_length_m - left.arc_length_m;
  if (!finite(execution_span_m) || execution_span_m <= 0.0 ||
      !finite(maximum_mps2) || maximum_mps2 < 0.0) {
    return false;
  }
  const auto speed_at = [&](const double arc_length_m) {
    const double ratio =
        (arc_length_m - left.arc_length_m) / execution_span_m;
    return interpolate(left.ground_speed_mps, right.ground_speed_mps,
                       ratio);
  };
  const auto subinterval_within_limit =
      [&](const double start_arc_length_m,
          const double end_arc_length_m) {
        const double start_speed_mps = speed_at(start_arc_length_m);
        const double speed_delta_mps =
            speed_at(end_arc_length_m) - start_speed_mps;
        const double start_curvature_per_m =
            interpolate_geometry(geometry, start_arc_length_m)
                .curvature_per_m;
        const double curvature_delta_per_m =
            interpolate_geometry(geometry, end_arc_length_m)
                .curvature_per_m -
            start_curvature_per_m;
        const auto acceleration_at =
            [&](const double ratio) {
              const double speed_mps =
                  start_speed_mps + speed_delta_mps * ratio;
              const double curvature_per_m =
                  start_curvature_per_m +
                  curvature_delta_per_m * ratio;
              return lateral_acceleration_mps2(speed_mps,
                                               curvature_per_m);
            };
        if (acceleration_at(0.0) > maximum_mps2 ||
            acceleration_at(1.0) > maximum_mps2) {
          return false;
        }
        const double stationary_denominator =
            3.0 * speed_delta_mps * curvature_delta_per_m;
        if (stationary_denominator != 0.0) {
          const double stationary_ratio =
              -(2.0 * speed_delta_mps * start_curvature_per_m +
                start_speed_mps * curvature_delta_per_m) /
              stationary_denominator;
          if (stationary_ratio > 0.0 && stationary_ratio < 1.0 &&
              acceleration_at(stationary_ratio) > maximum_mps2) {
            return false;
          }
        }
        return true;
      };

  double subinterval_start_m = left.arc_length_m;
  for (const auto& point : geometry.points) {
    if (point.arc_length_m > subinterval_start_m &&
        point.arc_length_m < right.arc_length_m) {
      if (!subinterval_within_limit(subinterval_start_m,
                                    point.arc_length_m)) {
        return false;
      }
      subinterval_start_m = point.arc_length_m;
    }
  }
  return subinterval_within_limit(subinterval_start_m,
                                  right.arc_length_m);
}

[[nodiscard]] bool uncertainty_profile_valid(
    const std::vector<RobotUncertaintySample>& profile,
    const ExecutionProfile& execution_profile, const CableState& initial_state,
    const CableModelParameters& parameters) noexcept {
  if (profile.size() != execution_profile.samples.size() || profile.empty()) {
    return false;
  }
  CovariancePropagationState propagation;
  propagation.initial_lag_angle_variance_rad2 =
      *initial_state.lag_angle_variance_rad2;
  for (std::size_t index = 0; index < profile.size(); ++index) {
    if (!finite(profile[index].arc_length_m) ||
        profile[index].arc_length_m !=
            execution_profile.samples[index].arc_length_m ||
        !finite(profile[index]
                    .heading_tracking_process_variance_per_m_rad2) ||
        profile[index].heading_tracking_process_variance_per_m_rad2 < 0.0 ||
        !valid_pose_tracking_covariance(
            profile[index].pose_tracking_covariance) ||
        !finite_symmetric_psd<kJointUncertaintyDimension>(
            joint_input_covariance(profile[index], propagation,
                                   parameters))) {
      return false;
    }
  }
  return true;
}

}  // namespace

CableModel::CableModel(CableModelParameters parameters)
    : parameters_(std::move(parameters)) {
  if (!parameters_valid(parameters_)) {
    throw std::invalid_argument("cable model parameters are invalid");
  }
}

CableMeanSample CableModel::predict_touchdown_mean(
    const Pose2d& robot_pose, const double lag_angle_rad) const {
  CableMeanSample sample;
  sample.cable_model_version = parameters_.version;
  if (!finite(robot_pose.x_m) || !finite(robot_pose.y_m) ||
      !finite(robot_pose.heading_rad) || !finite(lag_angle_rad)) {
    sample.issues.emplace_back("robot pose or lag angle is non-finite");
    return sample;
  }
  if (std::abs(lag_angle_rad) > parameters_.maximum_lag_angle_rad) {
    sample.validity = CableModelValidity::lag_angle_out_of_range;
    sample.issues.emplace_back("lag angle exceeds the calibrated range");
    return sample;
  }
  const PathPoint robot{0.0, robot_pose.x_m, robot_pose.y_m,
                        robot_pose.heading_rad, 0.0};
  sample.touchdown_position_m =
      touchdown_position(robot, lag_angle_rad, parameters_);
  sample.cable_heading_rad =
      wrap_angle(robot_pose.heading_rad + lag_angle_rad);
  sample.validity = CableModelValidity::valid;
  return sample;
}

CableInverseMeanSample CableModel::inverse_touchdown_mean(
    const Vector2m touchdown_target_m, const double touchdown_heading_rad,
    const double robot_heading_rad, const double lag_angle_rad,
    const MonotonicTime timestamp) const {
  CableInverseMeanSample inverse;
  if (!finite(touchdown_target_m.x_m) || !finite(touchdown_target_m.y_m) ||
      !finite(touchdown_heading_rad) || !finite(robot_heading_rad) ||
      !finite(lag_angle_rad) || timestamp.nanoseconds < 0) {
    inverse.forward_prediction.cable_model_version = parameters_.version;
    inverse.forward_prediction.issues.emplace_back(
        "inverse touchdown target or terminal state is invalid");
    return inverse;
  }
  const double release_x_m =
      touchdown_target_m.x_m +
      parameters_.touchdown_distance_m * std::cos(touchdown_heading_rad);
  const double release_y_m =
      touchdown_target_m.y_m +
      parameters_.touchdown_distance_m * std::sin(touchdown_heading_rad);
  const double normalized_robot_heading = wrap_angle(robot_heading_rad);
  const double cosine = std::cos(normalized_robot_heading);
  const double sine = std::sin(normalized_robot_heading);
  inverse.robot_pose.x_m =
      release_x_m -
      (cosine * parameters_.release_point_offset_m.x_m -
       sine * parameters_.release_point_offset_m.y_m);
  inverse.robot_pose.y_m =
      release_y_m -
      (sine * parameters_.release_point_offset_m.x_m +
       cosine * parameters_.release_point_offset_m.y_m);
  inverse.robot_pose.heading_rad = normalized_robot_heading;
  inverse.robot_pose.timestamp = timestamp;
  inverse.forward_prediction =
      predict_touchdown_mean(inverse.robot_pose, lag_angle_rad);
  return inverse;
}

CableModelIdentity CableModel::identity() const {
  return {parameters_.version, parameters_.calibration_dataset_id,
          parameters_.operating_domain_id};
}

CablePrediction CableModel::predict_search(
    const CableState& initial_state, const GeometricPath& robot_segment,
    const CableContext& context) const {
  CablePrediction prediction = make_prediction_shell(parameters_, context);
  prediction.dependencies.robot_path_version =
      robot_segment.metadata.path_version;
  prediction.dependencies.reference_line_version =
      robot_segment.metadata.reference_line_version;
  if (context.mode != PredictionMode::search) {
    fail(prediction, CableModelValidity::input_invalid,
         "search entry requires SEARCH context mode");
    return prediction;
  }
  if (!validate(initial_state).valid || !validate(robot_segment).valid ||
      !telemetry_valid(context.current_telemetry) ||
      context.current_telemetry.timestamp.nanoseconds !=
          initial_state.timestamp.nanoseconds ||
      context.uncertainty_envelope_version == 0U ||
      context.uncertainty_envelope_generator_version == 0U) {
    fail(prediction, CableModelValidity::input_invalid,
         "initial state, geometric path, or cable context is invalid");
    return prediction;
  }
  if (!calibrated_context_valid(context, parameters_, prediction)) {
    return prediction;
  }
  if (std::abs(initial_state.lag_angle_rad) >
      parameters_.maximum_lag_angle_rad) {
    fail(prediction, CableModelValidity::lag_angle_out_of_range,
         "initial lag angle exceeds the calibrated range");
    return prediction;
  }

  prediction.touchdown_path.metadata = robot_segment.metadata;
  prediction.touchdown_path.metadata.interpolation_rule =
      "cable-mean-spatial-lag";
  double lag_angle_rad = initial_state.lag_angle_rad;
  append_sample(prediction, robot_segment.points.front(), lag_angle_rad,
                initial_state, CableStateKind::search_mean,
                initial_state.timestamp, parameters_);
  for (std::size_t index = 1; index < robot_segment.points.size(); ++index) {
    if (!propagate_mean_interval(
            prediction, robot_segment,
            robot_segment.points[index - 1].arc_length_m,
            robot_segment.points[index].arc_length_m, lag_angle_rad,
            parameters_.search_integration_step_m, initial_state,
            CableStateKind::search_mean, initial_state.timestamp,
            initial_state.timestamp, parameters_)) {
      return prediction;
    }
  }
  complete_prediction(prediction);
  return prediction;
}

CablePrediction CableModel::predict(const CableState& initial_state,
                                    const TimedPath& robot_path,
                                    const CableContext& context) const {
  CablePrediction prediction = make_prediction_shell(parameters_, context);
  prediction.dependencies.robot_path_version =
      robot_path.geometry.metadata.path_version;
  prediction.dependencies.reference_line_version =
      robot_path.geometry.metadata.reference_line_version;
  prediction.dependencies.execution_profile_version =
      robot_path.execution_profile.version;
  if (context.mode != PredictionMode::validation) {
    fail(prediction, CableModelValidity::input_invalid,
         "validation entry requires VALIDATION context mode");
    return prediction;
  }
  if (!validate(initial_state).valid ||
      initial_state.kind != CableStateKind::tracked ||
      !initial_state.lag_angle_variance_rad2.has_value()) {
    fail(prediction, CableModelValidity::initial_state_uncertain,
         "validation requires a tracked actual cable state snapshot");
    return prediction;
  }
  if (!calibrated_context_valid(context, parameters_, prediction)) {
    return prediction;
  }
  if (robot_path.execution_profile.operating_envelope_version !=
      context.execution_envelope.version) {
    fail(prediction,
         CableModelValidity::execution_envelope_version_mismatch,
         "timed path and cable context execution envelope versions differ");
    return prediction;
  }
  const auto& execution_samples = robot_path.execution_profile.samples;
  const bool invalid_execution_profile_structure =
      execution_samples.empty() ||
      std::adjacent_find(
          execution_samples.begin(), execution_samples.end(),
          [](const ExecutionSample& left, const ExecutionSample& right) {
            return right.arc_length_m <= left.arc_length_m ||
                   right.time_from_start.nanoseconds <=
                       left.time_from_start.nanoseconds;
          }) != execution_samples.end();
  if (invalid_execution_profile_structure) {
    fail(prediction, CableModelValidity::input_invalid,
         "execution profile must be nonempty with strictly increasing arc length and time");
    return prediction;
  }
  if (!uncertainty_profile_valid(context.robot_uncertainty_profile,
                                 robot_path.execution_profile, initial_state,
                                 parameters_)) {
    fail(prediction, CableModelValidity::covariance_invalid,
         "robot pose or tracking covariance profile is invalid");
    return prediction;
  }
  if (!telemetry_valid(context.current_telemetry) ||
      context.current_telemetry.timestamp.nanoseconds !=
          initial_state.timestamp.nanoseconds ||
      context.uncertainty_envelope_version == 0U ||
      context.uncertainty_envelope_generator_version == 0U ||
      context.robot_uncertainty_profile_version == 0U) {
    fail(prediction, CableModelValidity::input_invalid,
         "cable context or execution envelope binding is invalid");
    return prediction;
  }
  if (std::abs(initial_state.lag_angle_rad) >
      parameters_.maximum_lag_angle_rad) {
    fail(prediction, CableModelValidity::lag_angle_out_of_range,
         "initial lag angle exceeds the calibrated range");
    return prediction;
  }

  if (!validate(robot_path.geometry).valid) {
    fail(prediction, CableModelValidity::input_invalid,
         "timed path geometry is invalid");
    return prediction;
  }
  for (const auto& sample : robot_path.execution_profile.samples) {
    const double curvature_per_m =
        interpolate_geometry(robot_path.geometry, sample.arc_length_m)
            .curvature_per_m;
    if (!classify_execution_sample(sample, parameters_,
                                   context.execution_envelope, curvature_per_m,
                                   prediction)) {
      return prediction;
    }
  }
  if (!validate(robot_path).valid) {
    fail(prediction, CableModelValidity::input_invalid,
         "timed path contract is invalid");
    return prediction;
  }
  for (std::size_t index = 1U;
       index < robot_path.execution_profile.samples.size(); ++index) {
    if (!lateral_acceleration_interval_within_limit(
            robot_path.geometry,
            robot_path.execution_profile.samples[index - 1U],
            robot_path.execution_profile.samples[index],
            context.execution_envelope.limits
                .maximum_lateral_acceleration_mps2)) {
      fail(prediction, CableModelValidity::motion_mode_out_of_range,
           "execution interval exceeds the certified lateral acceleration");
      return prediction;
    }
  }
  const auto& first_sample = robot_path.execution_profile.samples.front();
  if (std::abs(first_sample.payout_speed_mps -
               context.current_telemetry.payout_speed_mps) >
      std::min(robot_path.execution_profile.approved_tracking_limits
                   .maximum_payout_tracking_error_mps,
               context.execution_envelope.limits
                   .maximum_payout_tracking_error_mps) ||
      std::abs(first_sample.payout_acceleration_mps2 -
               context.current_telemetry.payout_acceleration_mps2) >
          context.execution_envelope
              .maximum_payout_acceleration_tracking_error_mps2) {
    fail(prediction, CableModelValidity::payout_tracking_out_of_range,
         "first planned payout sample is discontinuous with current telemetry");
    return prediction;
  }
  if (std::abs(first_sample.tension_setpoint_n -
               context.current_telemetry.tension_n) >
      context.execution_envelope.maximum_tension_tracking_error_n) {
    fail(prediction, CableModelValidity::tension_out_of_range,
         "first planned tension sample is discontinuous with current telemetry");
    return prediction;
  }

  prediction.touchdown_path.metadata = robot_path.geometry.metadata;
  prediction.touchdown_path.metadata.interpolation_rule =
      "cable-mean-spatial-lag";
  prediction.touchdown_covariance_profile_m2.emplace();
  prediction.touchdown_covariance_profile_m2->reserve(
      robot_path.execution_profile.samples.size());
  CovariancePropagationState covariance_state;
  covariance_state.initial_lag_angle_variance_rad2 =
      *initial_state.lag_angle_variance_rad2;
  double lag_angle_rad = initial_state.lag_angle_rad;
  PathPoint robot = interpolate_geometry(
      robot_path.geometry, first_sample.arc_length_m);
  const RobotUncertaintySample initial_uncertainty =
      interpolate_uncertainty(context.robot_uncertainty_profile,
                              first_sample.arc_length_m);
  append_sample(prediction, robot, lag_angle_rad, initial_state,
                CableStateKind::tracked, initial_state.timestamp, parameters_,
                &covariance_state, &initial_uncertainty);
  for (std::size_t index = 1;
       index < robot_path.execution_profile.samples.size(); ++index) {
    const auto& previous = robot_path.execution_profile.samples[index - 1];
    const auto& sample = robot_path.execution_profile.samples[index];
    MonotonicTime previous_time;
    MonotonicTime sample_time;
    if (!absolute_time(initial_state.timestamp, previous.time_from_start,
                       previous_time) ||
        !absolute_time(initial_state.timestamp, sample.time_from_start,
                       sample_time)) {
      fail(prediction, CableModelValidity::input_invalid,
           "execution profile timestamp overflows monotonic time");
      return prediction;
    }
    if (!propagate_mean_interval(
            prediction, robot_path.geometry, previous.arc_length_m,
            sample.arc_length_m, lag_angle_rad,
            parameters_.validation_integration_step_m, initial_state,
            CableStateKind::tracked, previous_time, sample_time, parameters_,
            &covariance_state, &context.robot_uncertainty_profile)) {
      return prediction;
    }
  }
  complete_prediction(prediction);
  return prediction;
}

void CableModel::set_parameters(CableModelParameters parameters) {
  if (!parameters_valid(parameters)) {
    throw std::invalid_argument("cable model parameters are invalid");
  }
  parameters_ = std::move(parameters);
}

std::uint64_t CableModel::version() const noexcept {
  return parameters_.version;
}

}  // namespace underwater_planner::core
