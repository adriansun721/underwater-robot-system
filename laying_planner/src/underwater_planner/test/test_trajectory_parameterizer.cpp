#include "underwater_planner/core/trajectory_parameterizer.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace underwater_planner::core;

namespace {
void require(const bool condition, const char* message) {
  if (!condition) {
    std::cerr << "T28 failure: " << message << "\n";
    std::exit(EXIT_FAILURE);
  }
}

GeometricPath straight_path(const double length_m) {
  GeometricPath path;
  path.metadata.path_version = 41;
  path.metadata.reference_line_version = 7;
  path.metadata.coordinate_frame = "map";
  path.metadata.interpolation_rule = "clothoid-linear-curvature/v1";
  path.points = {{0.0, 0.0, 0.0, 0.0, 0.0},
                {length_m * 0.5, length_m * 0.5, 0.0, 0.0, 0.0},
                {length_m, length_m, 0.0, 0.0, 0.0}};
  return path;
}

ExecutionOperatingEnvelope envelope() {
  ExecutionOperatingEnvelope value;
  value.version = 17;
  value.operating_domain_id = "tank-domain";
  value.limits.ground_speed = {0.0, 2.0};
  value.limits.ground_acceleration = {-1.0, 0.5};
  value.limits.maximum_lateral_acceleration_mps2 = 0.5;
  value.limits.payout_speed = {0.0, 2.0};
  value.limits.payout_acceleration = {-1.0, 1.0};
  value.limits.maximum_payout_tracking_error_mps = 0.05;
  value.limits.tension = {20.0, 80.0};
  value.limits.maximum_stopping_distance_m = 10.0;
  return value;
}

TrajectoryInitialState initial() { return {1.0, 1.0, 0.0, 40.0}; }

TrajectoryParameterizationLimits limits() {
  TrajectoryParameterizationLimits value;
  value.version = 9;
  value.sample_period_s = 0.1;
  value.terminal_speed_mps = 0.0;
  value.stopping_distance_margin_m = 0.2;
  value.timeout = {1'000'000'000};
  value.execution_profile_version = 33;
  return value;
}

void nominal_profile_is_complete_and_geometry_is_immutable() {
  // Design: 18.2.4-30
  // Design: 18.2.6-1
  // Design: 18.2.6-2
  // Design: 18.2.6-3
  const GeometricPath geometry = straight_path(4.0);
  const ExecutionOperatingEnvelope certified = envelope();
  const auto result = TrajectoryParameterizer{}.parameterize(
      geometry, initial(), certified, limits());
  require(result.status == ParameterizationStatus::success,
          "nominal path was not parameterized");
  require(result.trajectory.has_value(), "success omitted timed path");
  require(result.trajectory->execution_profile.version == 33,
          "profile version was not assigned");
  require(result.trajectory->execution_profile.operating_envelope_version == 17,
          "envelope version was not carried");
  require(result.trajectory->execution_profile.samples.size() == geometry.points.size(),
          "profile did not cover every geometry sample");
  require(result.trajectory->execution_profile.samples.back().ground_speed_mps == 0.0,
          "terminal stop sample is not zero speed");
  require(result.trajectory->execution_profile.samples.front().time_from_start.nanoseconds == 0,
          "profile did not start at zero time");
  require(result.diagnostics.geometry_unchanged, "parameterization changed geometry");
  const GeometricPath& actual_geometry = result.trajectory->geometry;
  require(actual_geometry.metadata.path_version ==
                  geometry.metadata.path_version &&
              actual_geometry.metadata.coordinate_frame ==
                  geometry.metadata.coordinate_frame &&
              actual_geometry.metadata.reference_line_version ==
                  geometry.metadata.reference_line_version &&
              actual_geometry.metadata.interpolation_rule ==
                  geometry.metadata.interpolation_rule &&
              actual_geometry.points.size() == geometry.points.size(),
          "parameterization did not preserve every geometric path field");
  for (std::size_t index = 0U; index < geometry.points.size(); ++index) {
    const PathPoint& actual = actual_geometry.points[index];
    const PathPoint& expected = geometry.points[index];
    require(actual.arc_length_m == expected.arc_length_m &&
                actual.x_m == expected.x_m && actual.y_m == expected.y_m &&
                actual.heading_rad == expected.heading_rad &&
                actual.curvature_per_m == expected.curvature_per_m,
            "parameterization changed a geometric path sample");
  }
  require(result.diagnostics.maximum_lateral_acceleration_mps2 <= 0.5 + 1.0e-9,
          "lateral acceleration exceeded envelope");
  for (const ExecutionSample& sample :
       result.trajectory->execution_profile.samples) {
    require(sample.ground_speed_mps >=
                    certified.limits.ground_speed.minimum_mps &&
                sample.ground_speed_mps <=
                    certified.limits.ground_speed.maximum_mps &&
                sample.ground_acceleration_mps2 >=
                    certified.limits.ground_acceleration.minimum_mps2 &&
                sample.ground_acceleration_mps2 <=
                    certified.limits.ground_acceleration.maximum_mps2 &&
                sample.payout_speed_mps >=
                    certified.limits.payout_speed.minimum_mps &&
                sample.payout_speed_mps <=
                    certified.limits.payout_speed.maximum_mps &&
                sample.payout_acceleration_mps2 >=
                    certified.limits.payout_acceleration.minimum_mps2 &&
                sample.payout_acceleration_mps2 <=
                    certified.limits.payout_acceleration.maximum_mps2 &&
                std::abs(sample.payout_speed_mps -
                         sample.ground_speed_mps) <=
                    certified.limits.maximum_payout_tracking_error_mps &&
                sample.tension_setpoint_n >=
                    certified.limits.tension.minimum_n &&
                sample.tension_setpoint_n <=
                    certified.limits.tension.maximum_n,
            "approved final profile escaped its certified execution envelope");
  }
}

void stopping_distance_is_hard_constraint() {
  // Design: 18.2.6-4
  auto short_limits = limits();
  short_limits.stopping_distance_margin_m = 3.0;
  const auto result = TrajectoryParameterizer{}.parameterize(
      straight_path(1.0), initial(), envelope(), short_limits);
  require(result.status == ParameterizationStatus::stopping_constraint_infeasible,
          "unavailable stopping distance was accepted");
  require(!result.trajectory.has_value(), "failed parameterization returned trajectory");
}

void invalid_envelope_and_tracking_are_rejected() {
  auto bad_envelope = envelope();
  bad_envelope.limits.maximum_payout_tracking_error_mps = 0.0;
  bad_envelope.limits.payout_speed.minimum_mps = 0.5;
  const auto tracking = TrajectoryParameterizer{}.parameterize(
      straight_path(4.0), initial(), bad_envelope, limits());
  require(tracking.status == ParameterizationStatus::payout_infeasible,
          "payout tracking limit was ignored");

  bad_envelope = envelope();
  bad_envelope.version = 0;
  const auto missing_version = TrajectoryParameterizer{}.parameterize(
      straight_path(4.0), initial(), bad_envelope, limits());
  require(missing_version.status == ParameterizationStatus::execution_envelope_mismatch,
          "unversioned envelope was accepted");
}

void curved_path_obeys_lateral_acceleration() {
  // Design: 18.2.6-2
  GeometricPath geometry = straight_path(4.0);
  geometry.points[1].curvature_per_m = 0.5;
  const auto result = TrajectoryParameterizer{}.parameterize(
      geometry, initial(), envelope(), limits());
  require(result.status == ParameterizationStatus::success,
          "curved path was rejected despite lateral cap");
  require(result.diagnostics.maximum_lateral_acceleration_mps2 <= 0.5 + 1.0e-9,
          "curved path exceeded lateral cap");
}

void monotonic_deadline_stops_parameterization_without_partial_output() {
  std::int64_t now_ns = 0;
  TrajectoryParameterizer parameterizer([&now_ns] {
    now_ns += 10;
    return MonotonicTime{now_ns};
  });
  auto short_deadline = limits();
  short_deadline.timeout = {15};
  const auto result = parameterizer.parameterize(
      straight_path(4.0), initial(), envelope(), short_deadline);
  require(result.status == ParameterizationStatus::deadline_exceeded,
          "expired monotonic deadline was not reported distinctly");
  require(!result.trajectory.has_value(),
          "deadline returned a partially parameterized trajectory");
}

void completion_boundary_rechecks_deadline_before_success() {
  std::size_t clock_calls = 0U;
  TrajectoryParameterizer parameterizer([&clock_calls] {
    ++clock_calls;
    return MonotonicTime{clock_calls >= 13U ? 20 : 0};
  });
  auto completion_deadline = limits();
  completion_deadline.timeout = {10};
  const auto result = parameterizer.parameterize(
      straight_path(4.0), initial(), envelope(), completion_deadline);
  require(result.status == ParameterizationStatus::deadline_exceeded,
          "completion after the deadline was reported as success");
  require(!result.trajectory.has_value(),
          "completion-boundary deadline exposed a trajectory");
}
}  // namespace

int main() {
  nominal_profile_is_complete_and_geometry_is_immutable();
  stopping_distance_is_hard_constraint();
  invalid_envelope_and_tracking_are_rejected();
  curved_path_obeys_lateral_acceleration();
  monotonic_deadline_stops_parameterization_without_partial_output();
  completion_boundary_rechecks_deadline_before_success();
  std::cout << "T28 trajectory parameterizer tests passed\n";
}
