#include "underwater_planner/core/trajectory_parameterizer.hpp"

#include "underwater_planner/core/data_contract.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace underwater_planner::core {
namespace {

constexpr double kEpsilon = 1.0e-9;

bool finite(const double value) { return std::isfinite(value); }

bool in_range(const double value, const RangeMps range) {
  return finite(value) && finite(range.minimum_mps) &&
         finite(range.maximum_mps) && range.minimum_mps <= range.maximum_mps &&
         value >= range.minimum_mps - kEpsilon &&
         value <= range.maximum_mps + kEpsilon;
}

bool in_range(const double value, const RangeMps2 range) {
  return finite(value) && finite(range.minimum_mps2) &&
         finite(range.maximum_mps2) && range.minimum_mps2 <= range.maximum_mps2 &&
         value >= range.minimum_mps2 - kEpsilon &&
         value <= range.maximum_mps2 + kEpsilon;
}

bool in_range(const double value, const RangeN range) {
  return finite(value) && finite(range.minimum_n) && finite(range.maximum_n) &&
         range.minimum_n <= range.maximum_n &&
         value >= range.minimum_n - kEpsilon && value <= range.maximum_n + kEpsilon;
}

void fail(ParameterizationResult& result, const ParameterizationStatus status,
          std::string issue) {
  result.status = status;
  result.diagnostics.issues.push_back(std::move(issue));
}

}  // namespace

TrajectoryParameterizer::TrajectoryParameterizer()
    : TrajectoryParameterizer([] {
        return MonotonicTime{std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 std::chrono::steady_clock::now()
                                     .time_since_epoch())
                                 .count()};
      }) {}

TrajectoryParameterizer::TrajectoryParameterizer(Clock clock)
    : clock_(std::move(clock)) {}

ParameterizationResult TrajectoryParameterizer::parameterize(
    const GeometricPath& geometry, const TrajectoryInitialState& initial_state,
    const ExecutionOperatingEnvelope& certified_envelope,
    const TrajectoryParameterizationLimits& limits) const {
  ParameterizationResult result;
  result.diagnostics.limits_version = limits.version;
  result.diagnostics.envelope_version = certified_envelope.version;

  if (!clock_ || limits.timeout.nanoseconds <= 0) {
    fail(result, ParameterizationStatus::numerically_invalid,
         "parameterization requires a positive monotonic timeout");
    return result;
  }
  const MonotonicTime started_at = clock_();
  if (started_at.nanoseconds < 0 ||
      limits.timeout.nanoseconds >
          std::numeric_limits<std::int64_t>::max() - started_at.nanoseconds) {
    fail(result, ParameterizationStatus::numerically_invalid,
         "parameterization deadline is not representable");
    return result;
  }
  const std::int64_t deadline_ns =
      started_at.nanoseconds + limits.timeout.nanoseconds;
  const auto deadline_exceeded = [this, deadline_ns] {
    return clock_().nanoseconds >= deadline_ns;
  };
  const auto stop_on_deadline = [&result, &deadline_exceeded] {
    if (!deadline_exceeded()) return false;
    fail(result, ParameterizationStatus::deadline_exceeded,
         "trajectory parameterization exceeded its monotonic deadline");
    return true;
  };

  if (certified_envelope.version == 0U ||
      certified_envelope.operating_domain_id.empty()) {
    fail(result, ParameterizationStatus::execution_envelope_mismatch,
         "certified execution envelope requires version and operating domain");
    return result;
  }
  const SpeedPayoutLimits& approved = certified_envelope.limits;
  if (!finite(limits.sample_period_s) || limits.sample_period_s <= 0.0 ||
      !finite(limits.terminal_speed_mps) || limits.terminal_speed_mps < 0.0 ||
      !finite(limits.stopping_distance_margin_m) ||
      limits.stopping_distance_margin_m < 0.0 || limits.version == 0U ||
      (limits.execution_profile_version == 0U && geometry.metadata.path_version == 0U)) {
    fail(result, ParameterizationStatus::numerically_invalid,
         "parameterization limits are incomplete or non-finite");
    return result;
  }
  if (geometry.points.size() < 2U) {
    fail(result, ParameterizationStatus::numerically_invalid,
         "geometry must contain at least two points");
    return result;
  }
  for (std::size_t i = 0; i < geometry.points.size(); ++i) {
    if (stop_on_deadline()) return result;
    const PathPoint& point = geometry.points[i];
    if (!finite(point.arc_length_m) || !finite(point.x_m) || !finite(point.y_m) ||
        !finite(point.heading_rad) || !finite(point.curvature_per_m) ||
        (i > 0U && point.arc_length_m <= geometry.points[i - 1U].arc_length_m)) {
      fail(result, ParameterizationStatus::numerically_invalid,
           "geometry contains non-finite or non-monotonic samples");
      return result;
    }
  }
  if (!finite(initial_state.ground_speed_mps) ||
      !finite(initial_state.payout_speed_mps) ||
      !finite(initial_state.payout_acceleration_mps2) ||
      !finite(initial_state.tension_n)) {
    fail(result, ParameterizationStatus::initial_state_invalid,
         "initial execution state must be finite");
    return result;
  }
  if (!in_range(initial_state.ground_speed_mps, approved.ground_speed) ||
      !in_range(initial_state.payout_speed_mps, approved.payout_speed) ||
      !in_range(initial_state.payout_acceleration_mps2, approved.payout_acceleration) ||
      !in_range(initial_state.tension_n, approved.tension)) {
    fail(result, ParameterizationStatus::initial_state_invalid,
         "initial execution state is outside the certified envelope");
    return result;
  }
  if (!in_range(approved.ground_speed.minimum_mps, approved.ground_speed) ||
      !in_range(approved.ground_speed.maximum_mps, approved.ground_speed) ||
      approved.maximum_lateral_acceleration_mps2 <= 0.0 ||
      approved.maximum_stopping_distance_m < 0.0 ||
      approved.maximum_payout_tracking_error_mps < 0.0 ||
      !in_range(approved.tension.minimum_n, approved.tension) ||
      !in_range(approved.tension.maximum_n, approved.tension)) {
    fail(result, ParameterizationStatus::execution_envelope_mismatch,
         "certified execution envelope limits are invalid");
    return result;
  }

  const double total_length = geometry.points.back().arc_length_m -
                              geometry.points.front().arc_length_m;
  const double brake = std::max(0.0, -approved.ground_acceleration.minimum_mps2);
  if (!finite(total_length) || total_length < 0.0 || brake <= kEpsilon) {
    fail(result, ParameterizationStatus::stopping_constraint_infeasible,
         "a positive certified braking capability is required");
    return result;
  }
  const double required_stop =
      initial_state.ground_speed_mps * initial_state.ground_speed_mps /
          (2.0 * brake) +
      limits.stopping_distance_margin_m;
  result.diagnostics.required_stopping_distance_m = required_stop;
  result.diagnostics.available_stopping_distance_m = total_length;
  if (limits.require_terminal_stop &&
      (required_stop > total_length + kEpsilon ||
       required_stop > approved.maximum_stopping_distance_m + kEpsilon)) {
    fail(result, ParameterizationStatus::stopping_constraint_infeasible,
         "available path distance cannot satisfy terminal stopping distance");
    return result;
  }

  std::vector<double> speeds(geometry.points.size(), approved.ground_speed.maximum_mps);
  speeds.front() = initial_state.ground_speed_mps;
  for (std::size_t i = 0; i < speeds.size(); ++i) {
    if (stop_on_deadline()) return result;
    const double remaining = geometry.points.back().arc_length_m -
                             geometry.points[i].arc_length_m;
    const double stop_cap = limits.require_terminal_stop
                                ? std::sqrt(std::max(0.0, 2.0 * brake * remaining))
                                : approved.ground_speed.maximum_mps;
    const double lateral_cap =
        std::abs(geometry.points[i].curvature_per_m) > kEpsilon
            ? std::sqrt(approved.maximum_lateral_acceleration_mps2 /
                        std::abs(geometry.points[i].curvature_per_m))
            : approved.ground_speed.maximum_mps;
    speeds[i] = std::min({speeds[i], approved.ground_speed.maximum_mps,
                           std::max(0.0, stop_cap), lateral_cap});
    if (i + 1U < speeds.size()) {
      const double ds = geometry.points[i + 1U].arc_length_m -
                        geometry.points[i].arc_length_m;
      const double next_cap =
          std::sqrt(std::max(0.0, speeds[i] * speeds[i] +
                                      2.0 * approved.ground_acceleration.maximum_mps2 * ds));
      speeds[i + 1U] = std::min(speeds[i + 1U], next_cap);
    }
  }
  if (limits.require_terminal_stop) {
    speeds.back() = 0.0;
    for (std::size_t i = speeds.size() - 1U; i > 0U; --i) {
      if (stop_on_deadline()) return result;
      const double ds = geometry.points[i].arc_length_m -
                        geometry.points[i - 1U].arc_length_m;
      const double previous_cap = std::sqrt(std::max(
          0.0, speeds[i] * speeds[i] + 2.0 * brake * ds));
      speeds[i - 1U] = std::min(speeds[i - 1U], previous_cap);
    }
    if (speeds.front() + kEpsilon < initial_state.ground_speed_mps) {
      fail(result, ParameterizationStatus::stopping_constraint_infeasible,
           "backward braking propagation conflicts with initial speed");
      return result;
    }
  }

  ExecutionProfile profile;
  profile.operating_envelope_version = certified_envelope.version;
  profile.interpolation_rule = "piecewise-linear-execution/v1";
  profile.stopping_point_arc_length_m =
      geometry.points.back().arc_length_m;
  profile.approved_tracking_limits = approved;
  profile.samples.reserve(geometry.points.size());
  std::int64_t elapsed_ns = 0;
  double maximum_lateral = 0.0;
  double maximum_speed = 0.0;
  double minimum_speed = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < geometry.points.size(); ++i) {
    if (stop_on_deadline()) return result;
    const double speed = speeds[i];
    if (!finite(speed) || speed < -kEpsilon ||
        speed * speed * std::abs(geometry.points[i].curvature_per_m) >
            approved.maximum_lateral_acceleration_mps2 + 1.0e-8) {
      fail(result, ParameterizationStatus::dynamics_infeasible,
           "speed or lateral acceleration constraint is infeasible");
      return result;
    }
    double acceleration = 0.0;
    double dt_s = limits.sample_period_s;
    if (i > 0U) {
      const double ds = geometry.points[i].arc_length_m -
                        geometry.points[i - 1U].arc_length_m;
      const double average_speed = (speeds[i] + speeds[i - 1U]) * 0.5;
      dt_s = average_speed > kEpsilon ? ds / average_speed : limits.sample_period_s;
      if (!finite(dt_s) || dt_s <= 0.0) dt_s = limits.sample_period_s;
      acceleration = (speeds[i] - speeds[i - 1U]) / dt_s;
    }
    if (!in_range(acceleration, approved.ground_acceleration)) {
      fail(result, ParameterizationStatus::dynamics_infeasible,
           "ground acceleration exceeds certified limits");
      return result;
    }
    const double payout_speed =
        i == 0U
            ? initial_state.payout_speed_mps
            : std::clamp(speed, approved.payout_speed.minimum_mps,
                         approved.payout_speed.maximum_mps);
    const double payout_acceleration = i == 0U
                                           ? initial_state.payout_acceleration_mps2
                                           : (payout_speed - profile.samples.back().payout_speed_mps) /
                                                 dt_s;
    if (!in_range(payout_speed, approved.payout_speed) ||
        std::abs(payout_speed - speed) >
            approved.maximum_payout_tracking_error_mps + kEpsilon ||
        !in_range(payout_acceleration, approved.payout_acceleration)) {
      fail(result, ParameterizationStatus::payout_infeasible,
           "payout speed, tracking error, or acceleration exceeds limits");
      return result;
    }
    const double tension = std::clamp(initial_state.tension_n,
                                      approved.tension.minimum_n,
                                      approved.tension.maximum_n);
    if (!in_range(tension, approved.tension)) {
      fail(result, ParameterizationStatus::payout_infeasible,
           "tension setpoint exceeds certified limits");
      return result;
    }
    if (i > 0U) {
      const auto increment = static_cast<std::int64_t>(
          std::llround(dt_s * 1.0e9));
      if (increment <= 0 || elapsed_ns > std::numeric_limits<std::int64_t>::max() - increment) {
        fail(result, ParameterizationStatus::numerically_invalid,
             "execution time is not representable");
        return result;
      }
      elapsed_ns += increment;
    }
    profile.samples.push_back(ExecutionSample{
        geometry.points[i].arc_length_m, Duration{elapsed_ns}, speed, acceleration,
        payout_speed, payout_acceleration, tension});
    maximum_lateral = std::max(maximum_lateral,
                               speed * speed * std::abs(geometry.points[i].curvature_per_m));
    maximum_speed = std::max(maximum_speed, speed);
    minimum_speed = std::min(minimum_speed, speed);
  }
  if (profile.samples.back().ground_speed_mps > limits.terminal_speed_mps + kEpsilon) {
    fail(result, ParameterizationStatus::stopping_constraint_infeasible,
         "terminal speed exceeds requested stopping speed");
    return result;
  }
  profile.version = limits.execution_profile_version;
  if (profile.version == 0U) profile.version = limits.version;
  TimedPath trajectory{geometry, std::move(profile)};
  const ValidationResult validation = validate(trajectory);
  if (!validation.valid) {
    result.diagnostics.issues = validation.issues;
    result.status = ParameterizationStatus::numerically_invalid;
    return result;
  }
  result.diagnostics.maximum_lateral_acceleration_mps2 = maximum_lateral;
  result.diagnostics.maximum_speed_mps = maximum_speed;
  result.diagnostics.minimum_speed_mps = minimum_speed;
  result.diagnostics.geometry_unchanged =
      trajectory.geometry.metadata.path_version == geometry.metadata.path_version &&
      trajectory.geometry.points.size() == geometry.points.size();
  if (result.diagnostics.geometry_unchanged) {
    for (std::size_t i = 0; i < geometry.points.size(); ++i) {
      const PathPoint& left = trajectory.geometry.points[i];
      const PathPoint& right = geometry.points[i];
      if (left.arc_length_m != right.arc_length_m || left.x_m != right.x_m ||
          left.y_m != right.y_m || left.heading_rad != right.heading_rad ||
          left.curvature_per_m != right.curvature_per_m) {
        result.diagnostics.geometry_unchanged = false;
        break;
      }
    }
  }
  if (stop_on_deadline()) return result;
  result.status = ParameterizationStatus::success;
  result.trajectory = std::move(trajectory);
  return result;
}

std::string serialize_trajectory_parameterization_limits(
    const TrajectoryParameterizationLimits& limits) {
  std::ostringstream output;
  output << std::setprecision(17) << limits.version << '\n'
         << limits.sample_period_s << '\n' << limits.terminal_speed_mps << '\n'
         << limits.stopping_distance_margin_m << '\n'
         << limits.require_terminal_stop << '\n' << limits.timeout.nanoseconds
         << '\n' << limits.execution_profile_version << '\n';
  return output.str();
}

}  // namespace underwater_planner::core
