#include "underwater_planner/core/cable_model.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace {

using underwater_planner::core::CableContext;
using underwater_planner::core::CableModel;
using underwater_planner::core::CableModelParameters;
using underwater_planner::core::CableModelValidity;
using underwater_planner::core::CablePrediction;
using underwater_planner::core::CableState;
using underwater_planner::core::CableStateKind;
using underwater_planner::core::ExecutionOperatingEnvelope;
using underwater_planner::core::GeometricPath;
using underwater_planner::core::MonotonicTime;
using underwater_planner::core::PathPoint;
using underwater_planner::core::PredictionMode;
using underwater_planner::core::SensorHealthMode;
using underwater_planner::core::TimedPath;
using underwater_planner::core::validate;

static_assert(
    !std::is_invocable_v<decltype(&CableModel::predict), const CableModel&,
                         const CableState&, const GeometricPath&,
                         const CableContext&>,
    "final cable validation must reject geometry without an execution profile");

void require(const bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void require_near(const double actual, const double expected,
                  const double tolerance, const std::string& message) {
  require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
          message);
}

bool same_points(const GeometricPath& left, const GeometricPath& right) {
  if (left.points.size() != right.points.size()) {
    return false;
  }
  for (std::size_t index = 0; index < left.points.size(); ++index) {
    const auto& a = left.points[index];
    const auto& b = right.points[index];
    if (a.arc_length_m != b.arc_length_m || a.x_m != b.x_m ||
        a.y_m != b.y_m || a.heading_rad != b.heading_rad ||
        a.curvature_per_m != b.curvature_per_m) {
      return false;
    }
  }
  return true;
}

bool same_covariances(
    const std::vector<underwater_planner::core::Covariance2dM2>& left,
    const std::vector<underwater_planner::core::Covariance2dM2>& right) {
  if (left.size() != right.size()) {
    return false;
  }
  for (std::size_t index = 0; index < left.size(); ++index) {
    if (left[index].xx_m2 != right[index].xx_m2 ||
        left[index].xy_m2 != right[index].xy_m2 ||
        left[index].yx_m2 != right[index].yx_m2 ||
        left[index].yy_m2 != right[index].yy_m2) {
      return false;
    }
  }
  return true;
}

CableModelParameters model_parameters() {
  CableModelParameters parameters;
  parameters.version = 14;
  parameters.calibration_dataset_id = "cable-mean-cal-v1";
  parameters.operating_domain_id = "competition-seabed-v1";
  parameters.release_point_offset_m = {0.5, 0.25};
  parameters.touchdown_distance_m = 1.0;
  parameters.direction_response_length_m = 2.0;
  parameters.maximum_lag_angle_rad = 1.2;
  parameters.maximum_payout_tracking_error_mps = 0.1;
  parameters.payout_speed_range = {0.0, 1.0};
  parameters.maximum_payout_acceleration_mps2 = 0.4;
  parameters.maximum_tension_tracking_error_n = 10.0;
  parameters.tension_range = {10.0, 100.0};
  parameters.search_integration_step_m = 0.5;
  parameters.validation_integration_step_m = 0.02;
  parameters.touchdown_distance_variance_m2 = 0.0025;
  parameters.direction_response_length_variance_m2 = 0.04;
  parameters.lag_angle_process_variance_per_m_rad2 = 0.03;
  parameters.touchdown_process_noise_per_m_m2 =
      {0.001, 0.0, 0.0, 0.002};
  parameters.approved_sensor_modes = {SensorHealthMode::nominal};
  return parameters;
}

ExecutionOperatingEnvelope execution_envelope() {
  ExecutionOperatingEnvelope envelope;
  envelope.version = 7;
  envelope.operating_domain_id = "competition-seabed-v1";
  envelope.limits.ground_speed = {0.0, 0.8};
  envelope.limits.ground_acceleration = {-0.4, 0.4};
  envelope.limits.maximum_lateral_acceleration_mps2 = 0.4;
  envelope.limits.payout_speed = {0.0, 0.9};
  envelope.limits.payout_acceleration = {-0.3, 0.3};
  envelope.limits.maximum_payout_tracking_error_mps = 0.08;
  envelope.limits.tension = {20.0, 80.0};
  envelope.limits.maximum_stopping_distance_m = 1.5;
  envelope.maximum_payout_acceleration_tracking_error_mps2 = 0.1;
  envelope.maximum_tension_tracking_error_n = 8.0;
  return envelope;
}

CableContext search_context() {
  CableContext context;
  context.current_telemetry = {0.5, 0.0, 40.0, {1'000'000'000}, 11};
  context.execution_envelope = execution_envelope();
  context.mode = PredictionMode::search;
  context.sensor_mode = SensorHealthMode::nominal;
  context.uncertainty_envelope_version = 9;
  context.uncertainty_envelope_generator_version = 10;
  context.robot_uncertainty_profile_version = 11;
  return context;
}

CableContext validation_context() {
  CableContext context = search_context();
  context.mode = PredictionMode::validation;
  context.robot_uncertainty_profile = {
      {0.0, {{0.04, 0.0, 0.0, 0.09}, 0.0, 0.0, 0.01}, 0.002},
      {1.0, {{0.04, 0.0, 0.0, 0.09}, 0.0, 0.0, 0.01}, 0.002},
      {2.0, {{0.04, 0.0, 0.0, 0.09}, 0.0, 0.0, 0.01}, 0.002},
      {3.0, {{0.04, 0.0, 0.0, 0.09}, 0.0, 0.0, 0.01}, 0.002},
  };
  return context;
}

CableState actual_state() {
  CableState state;
  state.kind = CableStateKind::tracked;
  state.lag_angle_rad = 0.0;
  state.lag_angle_variance_rad2 = 0.01;
  state.timestamp = MonotonicTime{1'000'000'000};
  state.laying_memory.canonical_signature = 77;
  state.sequence_number = 21;
  return state;
}

GeometricPath straight_path() {
  GeometricPath path;
  path.metadata = {31, "map", 4, "constant-curvature"};
  path.points = {
      PathPoint{0.0, 0.0, 0.0, 0.0, 0.0},
      PathPoint{1.0, 1.0, 0.0, 0.0, 0.0},
      PathPoint{2.0, 2.0, 0.0, 0.0, 0.0},
  };
  return path;
}

GeometricPath constant_curvature_path(const double curvature_per_m) {
  GeometricPath path;
  path.metadata = {32, "map", 4, "constant-curvature"};
  for (const double arc_length_m : {0.0, 1.0, 2.0, 3.0}) {
    const double heading_rad = curvature_per_m * arc_length_m;
    path.points.push_back(
        {arc_length_m, std::sin(heading_rad) / curvature_per_m,
         (1.0 - std::cos(heading_rad)) / curvature_per_m, heading_rad,
         curvature_per_m});
  }
  return path;
}

TimedPath timed_constant_curvature_path(const double curvature_per_m) {
  TimedPath path;
  path.geometry = constant_curvature_path(curvature_per_m);
  path.execution_profile.version = 41;
  path.execution_profile.operating_envelope_version = 7;
  path.execution_profile.interpolation_rule = "linear-in-arc-length";
  path.execution_profile.stopping_point_arc_length_m = 3.0;
  path.execution_profile.approved_tracking_limits = execution_envelope().limits;
  path.execution_profile.samples = {
      {0.0, {0}, 0.5, 0.0, 0.50, 0.0, 40.0},
      {1.0, {2'000'000'000}, 0.5, 0.0, 0.52, 0.02, 45.0},
      {2.0, {4'000'000'000}, 0.5, 0.0, 0.48, -0.02, 35.0},
      {3.0, {6'000'000'000}, 0.0, -0.25, 0.0, -0.25, 50.0},
  };
  return path;
}

GeometricPath varying_curvature_path() {
  GeometricPath path;
  path.metadata = {33, "map", 4, "curvature-linear-in-arc-length"};
  path.points = {
      {0.0, 0.0, 0.0, 0.0, 0.00},
      {1.0, 1.0, 0.025, 0.05, 0.10},
      {2.0, 1.99, 0.15, 0.20, 0.20},
      {3.0, 2.92, 0.50, 0.45, 0.30},
  };
  return path;
}

TimedPath timed_varying_curvature_path() {
  TimedPath path = timed_constant_curvature_path(0.2);
  path.geometry = varying_curvature_path();
  return path;
}

void straight_prediction_uses_release_offset_and_touchdown_distance() {
  // Design: 18.2.4-1
  // Design: 18.2.4-7
  const CableModel model(model_parameters());
  const auto prediction =
      model.predict_search(actual_state(), straight_path(), search_context());

  require(prediction.validity == CableModelValidity::valid,
          "valid straight search prediction was rejected");
  require(prediction.touchdown_path.points.size() >
              straight_path().points.size(),
          "search integration samples were not emitted");
  const auto& first = prediction.touchdown_path.points.front();
  const auto& last = prediction.touchdown_path.points.back();
  require_near(first.x_m, -0.5, 1.0e-12,
               "touchdown x ignored release offset or touchdown distance");
  require_near(first.y_m, 0.25, 1.0e-12,
               "touchdown y ignored lateral release offset");
  require_near(last.x_m, 1.5, 1.0e-12,
               "straight touchdown translation is incorrect");
  require_near(last.y_m, 0.25, 1.0e-12,
               "straight touchdown path drifted laterally");
  require(prediction.terminal_state.kind == CableStateKind::search_mean &&
              !prediction.terminal_state.lag_angle_variance_rad2.has_value(),
          "search output retained path-dependent covariance state");

  GeometricPath settling_path;
  settling_path.metadata = {35, "map", 4, "constant-curvature"};
  settling_path.points = {
      {0.0, 0.0, 0.0, 0.0, 0.0},
      {10.0, 10.0, 0.0, 0.0, 0.0},
      {20.0, 20.0, 0.0, 0.0, 0.0},
  };
  CableState unsettled = actual_state();
  unsettled.lag_angle_rad = 0.4;
  const CablePrediction settled =
      model.predict_search(unsettled, settling_path, search_context());
  require(settled.validity == CableModelValidity::valid &&
              std::abs(settled.terminal_state.lag_angle_rad) < 2.0e-5,
          "straight laying did not converge a nonzero lag state toward zero");
  const PathPoint& settled_touchdown = settled.touchdown_path.points.back();
  require_near(std::hypot(settled_touchdown.x_m - 20.5,
                          settled_touchdown.y_m - 0.25),
               model_parameters().touchdown_distance_m, 1.0e-12,
               "steady straight touchdown distance did not converge to L_td");
}

void validation_propagates_aligned_finite_psd_covariance() {
  const CableModel model(model_parameters());
  const auto prediction = model.predict(
      actual_state(), timed_constant_curvature_path(0.2),
      validation_context());

  require(prediction.validity == CableModelValidity::valid &&
              prediction.touchdown_covariance_profile_m2.has_value(),
          "validation did not publish actual touchdown covariance");
  const auto& covariance = *prediction.touchdown_covariance_profile_m2;
  require(covariance.size() == prediction.touchdown_path.points.size(),
          "touchdown covariance is not aligned with the mean path");
  require_near(covariance.front().xx_m2, 0.043125, 1.0e-12,
               "initial position, heading, lag, or model variance was lost");
  require_near(covariance.front().xy_m2, 0.00125, 1.0e-12,
               "initial covariance cross term is incorrect");
  require_near(covariance.front().yx_m2, 0.00125, 1.0e-12,
               "initial covariance is not symmetric");
  require_near(covariance.front().yy_m2, 0.1025, 1.0e-12,
               "initial lateral covariance is incorrect");
  for (const auto& sample : covariance) {
    const double determinant =
        sample.xx_m2 * sample.yy_m2 - sample.xy_m2 * sample.yx_m2;
    require(std::isfinite(sample.xx_m2) && std::isfinite(sample.xy_m2) &&
                std::isfinite(sample.yx_m2) && std::isfinite(sample.yy_m2) &&
                sample.xx_m2 >= 0.0 && sample.yy_m2 >= 0.0 &&
                sample.xy_m2 == sample.yx_m2 && determinant >= -1.0e-12,
            "touchdown covariance is non-finite, asymmetric, or non-PSD");
  }
  require(prediction.terminal_state.lag_angle_variance_rad2.has_value() &&
              *prediction.terminal_state.lag_angle_variance_rad2 >
                  *actual_state().lag_angle_variance_rad2,
          "initial lag variance and path process noise were not propagated");
}

void covariance_retains_robot_tracking_history() {
  // Design: 18.2.4-14
  const CableModel model(model_parameters());
  CableContext nominal_history = validation_context();
  CableContext disturbed_history = validation_context();
  disturbed_history.robot_uncertainty_profile[1]
      .heading_tracking_process_variance_per_m_rad2 = 0.05;
  disturbed_history.robot_uncertainty_profile[2]
      .heading_tracking_process_variance_per_m_rad2 = 0.05;
  const TimedPath path = timed_constant_curvature_path(0.2);

  const auto nominal = model.predict(actual_state(), path, nominal_history);
  const auto disturbed =
      model.predict(actual_state(), path, disturbed_history);
  require(nominal.validity == CableModelValidity::valid &&
              disturbed.validity == CableModelValidity::valid,
          "valid tracking uncertainty histories were rejected");
  require(nominal.terminal_state.lag_angle_rad ==
              disturbed.terminal_state.lag_angle_rad &&
              nominal.touchdown_path.points.back().x_m ==
                  disturbed.touchdown_path.points.back().x_m &&
              nominal.touchdown_path.points.back().y_m ==
                  disturbed.touchdown_path.points.back().y_m,
          "tracking uncertainty changed the terminal mean base state");
  require(disturbed.touchdown_covariance_profile_m2->back().yy_m2 >
              nominal.touchdown_covariance_profile_m2->back().yy_m2,
          "terminal covariance forgot the distinct tracking history");
}

void covariance_propagates_joint_input_cross_terms() {
  const CableModel model(model_parameters());
  CableContext independent = validation_context();
  CableContext correlated = validation_context();
  correlated.robot_uncertainty_profile.front()
      .cross_covariance.initial_lag_touchdown_distance_m_rad = 0.002;
  const TimedPath path = timed_constant_curvature_path(0.2);

  const auto independent_prediction =
      model.predict(actual_state(), path, independent);
  const auto correlated_prediction =
      model.predict(actual_state(), path, correlated);
  require(independent_prediction.validity == CableModelValidity::valid &&
              correlated_prediction.validity == CableModelValidity::valid,
          "a valid joint input covariance was rejected");
  require_near(correlated_prediction.touchdown_covariance_profile_m2
                   ->front()
                   .xy_m2,
               independent_prediction.touchdown_covariance_profile_m2
                       ->front()
                       .xy_m2 +
                   0.002,
               1.0e-12,
               "lag/distance cross covariance was not propagated by J Sigma J^T");
}

void validity_is_detailed_and_dependencies_are_complete() {
  const CableModel model(model_parameters());
  const TimedPath path = timed_constant_curvature_path(0.2);
  const auto valid = model.predict(actual_state(), path, validation_context());
  const auto& dependencies = valid.dependencies;
  require(valid.validity == CableModelValidity::valid &&
              dependencies.cable_model_version == 14 &&
              dependencies.calibration_dataset_id == "cable-mean-cal-v1" &&
              dependencies.robot_path_version == 32 &&
              dependencies.reference_line_version == 4 &&
              dependencies.execution_profile_version == 41 &&
              dependencies.execution_operating_envelope_version == 7 &&
              dependencies.uncertainty_envelope_version == 9 &&
              dependencies.uncertainty_envelope_generator_version == 10 &&
              dependencies.robot_uncertainty_profile_version == 11 &&
              dependencies.sensor_mode == SensorHealthMode::nominal &&
              dependencies.operating_domain_id == "competition-seabed-v1" &&
              dependencies.execution_operating_domain_id ==
                  "competition-seabed-v1",
          "cable prediction omitted or changed a dependency version");

  CableContext sensor_mismatch = validation_context();
  sensor_mismatch.sensor_mode = SensorHealthMode::approved_degraded;
  require(model.predict(actual_state(), path, sensor_mismatch).validity ==
              CableModelValidity::sensor_mode_unapproved,
          "an unapproved sensor mode was not distinguished");
  CableContext search_sensor_mismatch = search_context();
  search_sensor_mismatch.sensor_mode = SensorHealthMode::approved_degraded;
  require(model.predict_search(actual_state(), path.geometry,
                               search_sensor_mismatch)
              .validity == CableModelValidity::sensor_mode_unapproved,
          "search ignored an unapproved sensor mode");

  CableContext domain_mismatch = validation_context();
  domain_mismatch.execution_envelope.operating_domain_id = "other-domain";
  const auto domain_result =
      model.predict(actual_state(), path, domain_mismatch);
  require(domain_result.validity ==
              CableModelValidity::operating_domain_mismatch &&
              domain_result.dependencies.operating_domain_id ==
                  "competition-seabed-v1" &&
              domain_result.dependencies.execution_operating_domain_id ==
                  "other-domain",
          "an operating-domain mismatch was not distinguished");

  TimedPath envelope_mismatch = path;
  envelope_mismatch.execution_profile.operating_envelope_version = 8;
  require(model.predict(actual_state(), envelope_mismatch,
                        validation_context())
              .validity ==
              CableModelValidity::execution_envelope_version_mismatch,
          "an execution-envelope mismatch was not distinguished");

  CableContext invalid_covariance = validation_context();
  invalid_covariance.robot_uncertainty_profile[1]
      .pose_tracking_covariance.position_covariance_m2.xy_m2 = 1.0;
  require(model.predict(actual_state(), path, invalid_covariance).validity ==
              CableModelValidity::covariance_invalid,
          "a non-PSD robot pose/tracking covariance was not distinguished");

  CableContext subtly_invalid_covariance = validation_context();
  auto& subtle = subtly_invalid_covariance.robot_uncertainty_profile[1]
                     .pose_tracking_covariance.position_covariance_m2;
  subtle = {1.0e-9, 1.0e-6, 1.0e-6, 1.0e-9};
  require(model.predict(actual_state(), path, subtly_invalid_covariance)
                  .validity == CableModelValidity::covariance_invalid,
          "a small-scale covariance with a negative eigenvalue was accepted");
}

void certified_lateral_acceleration_fails_closed() {
  const CableModel model(model_parameters());
  const TimedPath path = timed_constant_curvature_path(0.2);
  CableContext narrowed = validation_context();
  narrowed.execution_envelope.limits.maximum_lateral_acceleration_mps2 =
      0.04;

  require(model.predict(actual_state(), path, narrowed).validity ==
              CableModelValidity::motion_mode_out_of_range,
          "validation used widened path limits instead of the certified envelope");

  TimedPath interior_peak = timed_varying_curvature_path();
  interior_peak.execution_profile.samples[0].ground_speed_mps = 0.8;
  interior_peak.execution_profile.samples[0].payout_speed_mps = 0.8;
  interior_peak.execution_profile.samples[1].ground_speed_mps = 0.4;
  interior_peak.execution_profile.samples[1].payout_speed_mps = 0.4;
  interior_peak.execution_profile.samples[2].ground_speed_mps = 0.25;
  interior_peak.execution_profile.samples[2].payout_speed_mps = 0.25;
  CableContext interior_context = validation_context();
  interior_context.current_telemetry.payout_speed_mps = 0.8;
  interior_context.execution_envelope.limits
      .maximum_lateral_acceleration_mps2 = 0.018;
  require(model.predict(actual_state(), interior_peak, interior_context)
                  .validity == CableModelValidity::motion_mode_out_of_range,
          "validation missed an interior lateral-acceleration peak");
}

void constant_curvature_turns_are_left_right_symmetric() {
  // Design: 18.2.4-2
  // Design: 18.2.4-3
  auto parameters = model_parameters();
  parameters.release_point_offset_m.y_m = 0.0;
  const CableModel model(parameters);
  const auto left = model.predict_search(
      actual_state(), constant_curvature_path(0.2), search_context());
  const auto right = model.predict_search(
      actual_state(), constant_curvature_path(-0.2), search_context());

  require(left.validity == CableModelValidity::valid &&
              right.validity == CableModelValidity::valid,
          "calibrated constant-curvature turns were rejected");
  require(left.touchdown_path.points.size() ==
              right.touchdown_path.points.size(),
          "mirrored turns produced different sample counts");
  for (std::size_t index = 0; index < left.touchdown_path.points.size();
       ++index) {
    require_near(left.touchdown_path.points[index].x_m,
                 right.touchdown_path.points[index].x_m, 1.0e-12,
                 "left/right touchdown x positions are not symmetric");
    require_near(left.touchdown_path.points[index].y_m,
                 -right.touchdown_path.points[index].y_m, 1.0e-12,
                 "left/right touchdown y positions are not mirrored");
    require_near(left.state_profile[index].lag_angle_rad,
                 -right.state_profile[index].lag_angle_rad, 1.0e-12,
                 "left/right lag states are not antisymmetric");
  }
  const double expected_lag_rad =
      -0.2 * parameters.direction_response_length_m *
      (1.0 - std::exp(-3.0 / parameters.direction_response_length_m));
  require_near(left.terminal_state.lag_angle_rad, expected_lag_rad, 1.0e-12,
               "constant-curvature lag does not match the spatial model");
}

void timed_validation_reads_the_complete_execution_profile() {
  // Design: 18.2.4-6
  // Design: 18.2.4-key-4
  const CableModel model(model_parameters());
  const TimedPath path = timed_constant_curvature_path(0.2);
  const auto valid =
      model.predict(actual_state(), path, validation_context());
  const auto search = model.predict_search(actual_state(), path.geometry,
                                           search_context());

  require(valid.validity == CableModelValidity::valid &&
              search.validity == CableModelValidity::valid,
          "valid timed cable prediction was rejected");
  require(valid.touchdown_path.points.size() >
              path.execution_profile.samples.size() &&
              valid.state_profile.size() == valid.touchdown_path.points.size(),
          "validation integration samples were not emitted and aligned");
  require(valid.state_profile.back().timestamp.nanoseconds == 7'000'000'000,
          "validation state timestamps ignored the execution profile");
  require(valid.terminal_state.kind == CableStateKind::tracked &&
              valid.terminal_state.lag_angle_variance_rad2.has_value() &&
              valid.terminal_state.laying_memory.canonical_signature == 77,
          "validation corrupted the tracked state kind or laying memory");
  require_near(search.terminal_state.lag_angle_rad,
               valid.terminal_state.lag_angle_rad, 1.0e-12,
               "search and validation changed the shared mean state equation");

  TimedPath bad_payout = path;
  bad_payout.execution_profile.samples[2].payout_speed_mps = 0.75;
  const auto payout_result =
      model.predict(actual_state(), bad_payout, validation_context());
  require(payout_result.validity ==
              CableModelValidity::payout_tracking_out_of_range,
          "a future payout mismatch was hidden by current telemetry");

  TimedPath bad_tension = path;
  bad_tension.execution_profile.samples[2].tension_setpoint_n = 90.0;
  const auto tension_result =
      model.predict(actual_state(), bad_tension, validation_context());
  require(tension_result.validity == CableModelValidity::tension_out_of_range,
          "a future tension violation was hidden by current telemetry");
}

void finer_search_integration_converges_to_validation() {
  // Design: 18.2.4-5
  auto coarse_parameters = model_parameters();
  coarse_parameters.search_integration_step_m = 1.0;
  coarse_parameters.validation_integration_step_m = 0.01;
  auto fine_parameters = coarse_parameters;
  fine_parameters.search_integration_step_m = 0.1;
  const CableModel coarse_model(coarse_parameters);
  const CableModel fine_model(fine_parameters);
  const GeometricPath geometry = varying_curvature_path();

  const auto coarse = coarse_model.predict_search(
      actual_state(), geometry, search_context());
  const auto fine =
      fine_model.predict_search(actual_state(), geometry, search_context());
  const auto validation = fine_model.predict(
      actual_state(), timed_varying_curvature_path(), validation_context());
  require(coarse.validity == CableModelValidity::valid &&
              fine.validity == CableModelValidity::valid &&
              validation.validity == CableModelValidity::valid,
          "convergence comparison rejected a calibrated prediction");

  const double coarse_error =
      std::abs(coarse.terminal_state.lag_angle_rad -
               validation.terminal_state.lag_angle_rad);
  const double fine_error =
      std::abs(fine.terminal_state.lag_angle_rad -
               validation.terminal_state.lag_angle_rad);
  require(fine_error < coarse_error && fine_error < 1.0e-4,
          "reducing search integration step did not converge to validation");
}

void invalid_inputs_fail_closed_and_predictions_are_deterministic() {
  const CableModel model(model_parameters());

  CableContext wrong_mode = search_context();
  wrong_mode.mode = PredictionMode::validation;
  require(model.predict_search(actual_state(), straight_path(), wrong_mode)
              .validity == CableModelValidity::input_invalid,
          "search accepted a validation-only context");

  GeometricPath nonmonotonic = straight_path();
  nonmonotonic.points[1].arc_length_m = 0.0;
  require(model.predict_search(actual_state(), nonmonotonic, search_context())
              .validity == CableModelValidity::input_invalid,
          "search accepted nonmonotonic geometry");

  CableState uncertain = actual_state();
  uncertain.kind = CableStateKind::search_mean;
  uncertain.lag_angle_variance_rad2.reset();
  require(model.predict(uncertain, timed_constant_curvature_path(0.2),
                        validation_context())
              .validity == CableModelValidity::initial_state_uncertain,
          "validation accepted a candidate mean as the actual state snapshot");

  TimedPath mismatched = timed_constant_curvature_path(0.2);
  mismatched.execution_profile.operating_envelope_version = 8;
  require(model.predict(actual_state(), mismatched, validation_context())
              .validity ==
          CableModelValidity::execution_envelope_version_mismatch,
          "validation accepted an execution envelope version mismatch");

  const auto first = model.predict_search(
      actual_state(), constant_curvature_path(0.2), search_context());
  const auto second = model.predict_search(
      actual_state(), constant_curvature_path(0.2), search_context());
  const auto validation_first = model.predict(
      actual_state(), timed_constant_curvature_path(0.2),
      validation_context());
  const auto validation_second = model.predict(
      actual_state(), timed_constant_curvature_path(0.2),
      validation_context());
  require(first.validity == CableModelValidity::valid &&
              validate(first.touchdown_path).valid &&
              same_points(first.touchdown_path, second.touchdown_path) &&
              first.state_profile.size() == second.state_profile.size() &&
              first.terminal_state.lag_angle_rad ==
                  second.terminal_state.lag_angle_rad &&
              first.dependencies.cable_model_version == 14 &&
              first.dependencies.execution_operating_envelope_version == 7 &&
              first.dependencies.calibration_dataset_id ==
                  "cable-mean-cal-v1" &&
              first.dependencies.operating_domain_id ==
                  "competition-seabed-v1",
          "identical model inputs did not reproduce an auditable result");
  require(validation_first.touchdown_covariance_profile_m2.has_value() &&
              validation_second.touchdown_covariance_profile_m2.has_value() &&
              same_covariances(
                  *validation_first.touchdown_covariance_profile_m2,
                  *validation_second.touchdown_covariance_profile_m2),
          "identical validation inputs changed actual covariance fields");
  require(std::abs(first.touchdown_path.points[1].curvature_per_m) > 0.01,
          "curved touchdown geometry published zero curvature");
}

void certified_tracking_and_snapshot_continuity_fail_closed() {
  CableModel model(model_parameters());
  TimedPath payout_mismatch = timed_constant_curvature_path(0.2);
  payout_mismatch.execution_profile.samples[1].payout_speed_mps = 0.59;
  require(model.predict(actual_state(), payout_mismatch,
                        validation_context())
              .validity ==
              CableModelValidity::payout_tracking_out_of_range,
          "planned payout mismatch exceeded the certified envelope");

  TimedPath acceleration_jump = timed_constant_curvature_path(0.2);
  acceleration_jump.execution_profile.samples.front()
      .payout_acceleration_mps2 = 0.2;
  require(model.predict(actual_state(), acceleration_jump,
                        validation_context())
              .validity ==
              CableModelValidity::payout_tracking_out_of_range,
          "first payout acceleration ignored snapshot continuity");

  CableContext stale_context = validation_context();
  stale_context.current_telemetry.timestamp.nanoseconds -= 1;
  require(model.predict(actual_state(), timed_constant_curvature_path(0.2),
                        stale_context)
              .validity == CableModelValidity::input_invalid,
          "validation accepted telemetry from a different snapshot time");

  CableContext excessive_tension_error = search_context();
  excessive_tension_error.execution_envelope
      .maximum_tension_tracking_error_n = 11.0;
  require(model.predict_search(actual_state(), straight_path(),
                               excessive_tension_error)
              .validity == CableModelValidity::input_invalid,
          "search accepted a tension tracking error beyond calibration");

  auto updated_parameters = model_parameters();
  updated_parameters.version = 15;
  model.set_parameters(updated_parameters);
  require(model.version() == 15,
          "set_parameters did not publish the new model version");
}

void intermediate_lag_range_exits_are_rejected() {
  auto parameters = model_parameters();
  parameters.maximum_lag_angle_rad = 0.12;
  parameters.search_integration_step_m = 0.05;
  parameters.validation_integration_step_m = 0.01;
  const CableModel model(parameters);
  GeometricPath path;
  path.metadata = {34, "map", 4, "curvature-linear-in-arc-length"};
  path.points = {
      {0.0, 0.0, 0.0, 0.0, 1.0},
      {2.0, 1.98, 0.28, 0.282, -0.718},
  };

  require(model.predict_search(actual_state(), path, search_context())
              .validity == CableModelValidity::lag_angle_out_of_range,
          "lag left and re-entered calibration between outer samples");
}

void untimed_and_invalid_execution_profiles_fail_closed() {
  // Design: 18.2.4-28
  constexpr bool geometry_only_validation_is_rejected =
      !std::is_invocable_v<decltype(&CableModel::predict), const CableModel&,
                           const CableState&, const GeometricPath&,
                           const CableContext&>;
  require(geometry_only_validation_is_rejected,
          "final prediction accepted geometry without a timed profile");

  const CableModel model(model_parameters());
  TimedPath empty = timed_constant_curvature_path(0.2);
  empty.execution_profile.samples.clear();
  require(model.predict(actual_state(), empty, validation_context()).validity ==
              CableModelValidity::input_invalid,
          "final prediction accepted an empty execution profile");

  TimedPath nonmonotonic = timed_constant_curvature_path(0.2);
  nonmonotonic.execution_profile.samples[2].time_from_start =
      nonmonotonic.execution_profile.samples[1].time_from_start;
  require(model.predict(actual_state(), nonmonotonic, validation_context())
                  .validity == CableModelValidity::input_invalid,
          "final prediction accepted nonmonotonic execution time");
}

void initial_lag_changes_touchdown_without_state_aliasing() {
  // Design: 18.2.4-key-2
  const CableModel model(model_parameters());
  CableState left = actual_state();
  CableState right = actual_state();
  left.lag_angle_rad = 0.2;
  right.lag_angle_rad = -0.2;

  const CablePrediction left_prediction =
      model.predict_search(left, straight_path(), search_context());
  const CablePrediction right_prediction =
      model.predict_search(right, straight_path(), search_context());

  require(left_prediction.validity == CableModelValidity::valid &&
              right_prediction.validity == CableModelValidity::valid &&
              left_prediction.touchdown_path.points.front().y_m !=
                  right_prediction.touchdown_path.points.front().y_m &&
              left_prediction.state_profile.front().lag_angle_rad == 0.2 &&
              right_prediction.state_profile.front().lag_angle_rad == -0.2,
          "distinct initial lag states aliased to the same touchdown state");
}

}  // namespace

int main() {
  constexpr std::uint64_t kSeed = 1414;
  try {
    straight_prediction_uses_release_offset_and_touchdown_distance();
    validation_propagates_aligned_finite_psd_covariance();
    covariance_retains_robot_tracking_history();
    covariance_propagates_joint_input_cross_terms();
    validity_is_detailed_and_dependencies_are_complete();
    certified_lateral_acceleration_fails_closed();
    constant_curvature_turns_are_left_right_symmetric();
    timed_validation_reads_the_complete_execution_profile();
    finer_search_integration_converges_to_validation();
    invalid_inputs_fail_closed_and_predictions_are_deterministic();
    certified_tracking_and_snapshot_continuity_fail_closed();
    intermediate_lag_range_exits_are_rejected();
    untimed_and_invalid_execution_profiles_fail_closed();
    initial_lag_changes_touchdown_without_state_aliasing();
    std::cout << "cable model checks passed: 14"
              << " seed=" << kSeed
              << " input_version=t15-cable-covariance/v1"
              << " units=SI timestamp=monotonic-ns"
              << " domain=competition-seabed-v1"
              << " risk=actual-path-pointwise-only\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "cable model failure: " << error.what()
              << " seed=" << kSeed
              << " input_version=t15-cable-covariance/v1"
              << " units=SI timestamp=monotonic-ns"
              << " domain=competition-seabed-v1"
              << " risk=actual-path-pointwise-only\n";
    return 1;
  }
}
