#include "underwater_planner/core/path_smoother.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <optional>
#include <stdexcept>
#include <sstream>
#include <string_view>
#include <utility>
#include <vector>

namespace underwater_planner::core {
namespace {

constexpr std::string_view kSmootherVersion = "path-smoother/clothoid-v1";
constexpr std::size_t kMaximumSegments = 512U;
constexpr std::size_t kMaximumIterations = 60U;
constexpr std::size_t kMaximumObjectiveIterations = 60U;
constexpr std::size_t kMaximumTopologySamples = 100'000U;
constexpr std::size_t kMaximumIntegrationIntervals = 16'384U;
constexpr double kClothoidIntegrationErrorBoundM = 1.0e-10;

struct ClothoidState {
  double x_m{};
  double y_m{};
  double heading_rad{};
  double curvature_per_m{};
};

struct SolverRun {
  SmoothingStatus status{SmoothingStatus::solver_failed};
  GeometricPath path;
  std::size_t iterations{};
};

enum class RawPathInterpolation {
  constant_curvature_exact,
};

[[nodiscard]] bool finite(const ClothoidState& state) noexcept {
  return std::isfinite(state.x_m) && std::isfinite(state.y_m) &&
         std::isfinite(state.heading_rad) &&
         std::isfinite(state.curvature_per_m);
}

[[nodiscard]] bool finite(const double value) noexcept {
  return std::isfinite(value);
}

[[nodiscard]] double angle_error(const double left,
                                 const double right) noexcept {
  return std::remainder(left - right, 2.0 * std::acos(-1.0));
}

[[nodiscard]] ClothoidState advance_clothoid(const ClothoidState& start,
                                              const double curvature_rate_per_m2,
                                              const double length_m) {
  if (!(length_m >= 0.0) || !finite(length_m) ||
      !finite(start.heading_rad) || !finite(start.curvature_per_m) ||
      !finite(curvature_rate_per_m2)) {
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    return {invalid, invalid, invalid, invalid};
  }
  if (length_m == 0.0) {
    return start;
  }

  const double maximum_heading_derivative =
      std::abs(start.curvature_per_m) +
      std::abs(curvature_rate_per_m2) * length_m;
  const double rate_magnitude = std::abs(curvature_rate_per_m2);
  // For f=sin(theta) or cos(theta), theta'=kappa and theta''=u,
  // |f''''| <= |kappa|^4 + 6|kappa|^2|u| + 3|u|^2. Composite
  // Simpson error is at most length*h^4*max|f''''|/180 per coordinate.
  const double fourth_derivative_bound =
      std::pow(maximum_heading_derivative, 4.0) +
      6.0 * maximum_heading_derivative * maximum_heading_derivative *
          rate_magnitude +
      3.0 * rate_magnitude * rate_magnitude;
  const double required_intervals_value = std::ceil(std::pow(
      std::pow(length_m, 5.0) * fourth_derivative_bound /
          (180.0 * kClothoidIntegrationErrorBoundM),
      0.25));
  if (!finite(required_intervals_value) ||
      required_intervals_value >
          static_cast<double>(kMaximumIntegrationIntervals)) {
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    return {invalid, invalid, invalid, invalid};
  }
  std::size_t integration_intervals = std::max(
      std::size_t{2U}, static_cast<std::size_t>(required_intervals_value));
  if (integration_intervals % 2U != 0U) {
    ++integration_intervals;
  }
  const double interval_m =
      length_m / static_cast<double>(integration_intervals);
  double cosine_sum = 0.0;
  double sine_sum = 0.0;
  for (std::size_t index = 0; index <= integration_intervals; ++index) {
    const double local_s_m = interval_m * static_cast<double>(index);
    const double heading_rad =
        start.heading_rad + start.curvature_per_m * local_s_m +
        0.5 * curvature_rate_per_m2 * local_s_m * local_s_m;
    const double coefficient =
        index == 0U || index == integration_intervals
            ? 1.0
            : (index % 2U == 0U ? 2.0 : 4.0);
    cosine_sum += coefficient * std::cos(heading_rad);
    sine_sum += coefficient * std::sin(heading_rad);
  }
  return {
      start.x_m + interval_m * cosine_sum / 3.0,
      start.y_m + interval_m * sine_sum / 3.0,
      normalize_angle_radians(
          start.heading_rad + start.curvature_per_m * length_m +
          0.5 * curvature_rate_per_m2 * length_m * length_m),
      start.curvature_per_m + curvature_rate_per_m2 * length_m,
  };
}

[[nodiscard]] double point_segment_distance(const double x_m, const double y_m,
                                             const PathPoint& start,
                                             const PathPoint& end) noexcept {
  const double dx_m = end.x_m - start.x_m;
  const double dy_m = end.y_m - start.y_m;
  const double length_squared_m2 = dx_m * dx_m + dy_m * dy_m;
  if (!(length_squared_m2 > 0.0)) {
    return std::hypot(x_m - start.x_m, y_m - start.y_m);
  }
  const double projection =
      std::clamp(((x_m - start.x_m) * dx_m + (y_m - start.y_m) * dy_m) /
                     length_squared_m2,
                 0.0, 1.0);
  return std::hypot(x_m - (start.x_m + projection * dx_m),
                    y_m - (start.y_m + projection * dy_m));
}

[[nodiscard]] std::optional<RawPathInterpolation> parse_raw_interpolation(
    const GeometricPath& path) noexcept {
  if (path.metadata.interpolation_rule == "constant-curvature-exact") {
    return RawPathInterpolation::constant_curvature_exact;
  }
  return std::nullopt;
}

[[nodiscard]] ClothoidState advance_constant_curvature(
    const PathPoint& start, const double curvature_per_m,
    const double length_m) noexcept {
  const double heading_rad = start.heading_rad + curvature_per_m * length_m;
  double x_m{};
  double y_m{};
  if (std::abs(curvature_per_m) <= 1.0e-12) {
    x_m = start.x_m + length_m * std::cos(start.heading_rad);
    y_m = start.y_m + length_m * std::sin(start.heading_rad);
  } else {
    x_m = start.x_m +
          (std::sin(heading_rad) - std::sin(start.heading_rad)) /
              curvature_per_m;
    y_m = start.y_m +
          (std::cos(start.heading_rad) - std::cos(heading_rad)) /
              curvature_per_m;
  }
  return {x_m, y_m, normalize_angle_radians(heading_rad), curvature_per_m};
}

[[nodiscard]] double point_constant_curvature_arc_distance(
    const double x_m, const double y_m, const PathPoint& start,
    const PathPoint& end) noexcept {
  const double length_m = end.arc_length_m - start.arc_length_m;
  const double curvature_per_m = end.curvature_per_m;
  if (std::abs(curvature_per_m) <= 1.0e-12) {
    const ClothoidState exact_end =
        advance_constant_curvature(start, curvature_per_m, length_m);
    const PathPoint endpoint{end.arc_length_m, exact_end.x_m, exact_end.y_m,
                             exact_end.heading_rad, curvature_per_m};
    return point_segment_distance(x_m, y_m, start, endpoint);
  }

  const double center_x_m =
      start.x_m - std::sin(start.heading_rad) / curvature_per_m;
  const double center_y_m =
      start.y_m + std::cos(start.heading_rad) / curvature_per_m;
  const double radius_m = 1.0 / std::abs(curvature_per_m);
  const double start_angle_rad =
      std::atan2(start.y_m - center_y_m, start.x_m - center_x_m);
  const double query_angle_rad = std::atan2(y_m - center_y_m, x_m - center_x_m);
  const double directed_angle_rad =
      curvature_per_m > 0.0
          ? normalize_angle_radians(query_angle_rad - start_angle_rad)
          : normalize_angle_radians(start_angle_rad - query_angle_rad);
  const double nonnegative_angle_rad =
      directed_angle_rad < 0.0 ? directed_angle_rad + 2.0 * std::acos(-1.0)
                               : directed_angle_rad;
  const double arc_angle_rad = std::abs(curvature_per_m * length_m);
  if (arc_angle_rad >= 2.0 * std::acos(-1.0) ||
      nonnegative_angle_rad <= arc_angle_rad) {
    return std::abs(std::hypot(x_m - center_x_m, y_m - center_y_m) - radius_m);
  }
  const ClothoidState exact_end =
      advance_constant_curvature(start, curvature_per_m, length_m);
  return std::min(std::hypot(x_m - start.x_m, y_m - start.y_m),
                  std::hypot(x_m - exact_end.x_m, y_m - exact_end.y_m));
}

[[nodiscard]] double distance_to_raw_path(const PathPoint& point,
                                          const GeometricPath& raw_path) {
  double minimum_distance_m = std::numeric_limits<double>::infinity();
  for (std::size_t index = 1U; index < raw_path.points.size(); ++index) {
    const PathPoint& start = raw_path.points[index - 1U];
    const PathPoint& end = raw_path.points[index];
    const double distance_m = point_constant_curvature_arc_distance(
        point.x_m, point.y_m, start, end);
    minimum_distance_m = std::min(minimum_distance_m, distance_m);
  }
  return minimum_distance_m;
}

[[nodiscard]] bool path_within_topology_tube(
    const GeometricPath& path, const GeometricPath& raw_path,
    const SmoothingLimits& limits) {
  std::size_t total_samples = 0U;
  for (std::size_t index = 1U; index < path.points.size(); ++index) {
    const PathPoint& left = path.points[index - 1U];
    const PathPoint& right = path.points[index];
    const double segment_length_m = right.arc_length_m - left.arc_length_m;
    const double maximum_sample_spacing_m =
        limits.topology_tube_radius_m * 0.5;
    const double interval_count_value =
        std::ceil(segment_length_m / maximum_sample_spacing_m);
    if (!finite(interval_count_value) || interval_count_value < 1.0 ||
        interval_count_value >
            static_cast<double>(kMaximumTopologySamples - total_samples)) {
      return false;
    }
    const std::size_t interval_count =
        static_cast<std::size_t>(interval_count_value);
    total_samples += interval_count;
    const double interval_length_m =
        segment_length_m / static_cast<double>(interval_count);
    const double rate_per_m2 =
        (right.curvature_per_m - left.curvature_per_m) / segment_length_m;
    const ClothoidState segment_start{left.x_m, left.y_m, left.heading_rad,
                                      left.curvature_per_m};
    for (std::size_t sample = 0U; sample <= interval_count; ++sample) {
      const double local_length_m =
          interval_length_m * static_cast<double>(sample);
      const ClothoidState state =
          advance_clothoid(segment_start, rate_per_m2, local_length_m);
      const PathPoint point{left.arc_length_m + local_length_m,
                            state.x_m,
                            state.y_m,
                            state.heading_rad,
                            state.curvature_per_m};
      const double unsampled_arc_margin_m = 0.5 * interval_length_m;
      if (distance_to_raw_path(point, raw_path) + unsampled_arc_margin_m >
          limits.topology_tube_radius_m) {
        return false;
      }
    }
  }
  return true;
}

[[nodiscard]] double interpolate_raw_curvature(const GeometricPath& raw_path,
                                               const double arc_length_m) {
  if (arc_length_m <= raw_path.points.front().arc_length_m) {
    return raw_path.points.front().curvature_per_m;
  }
  for (std::size_t index = 1U; index < raw_path.points.size(); ++index) {
    const PathPoint& right = raw_path.points[index];
    if (arc_length_m <= right.arc_length_m) {
      const PathPoint& left = raw_path.points[index - 1U];
      const double ratio = (arc_length_m - left.arc_length_m) /
                           (right.arc_length_m - left.arc_length_m);
      return left.curvature_per_m +
             ratio * (right.curvature_per_m - left.curvature_per_m);
    }
  }
  return raw_path.points.back().curvature_per_m;
}

[[nodiscard]] Vector2m interpolate_raw_position(
    const GeometricPath& raw_path, const double arc_length_m) {
  if (arc_length_m <= raw_path.points.front().arc_length_m) {
    return {raw_path.points.front().x_m, raw_path.points.front().y_m};
  }
  for (std::size_t index = 1U; index < raw_path.points.size(); ++index) {
    const PathPoint& right = raw_path.points[index];
    if (arc_length_m <= right.arc_length_m) {
      const PathPoint& left = raw_path.points[index - 1U];
      const double local_length_m = arc_length_m - left.arc_length_m;
      const ClothoidState state = advance_constant_curvature(
          left, right.curvature_per_m, local_length_m);
      return {state.x_m, state.y_m};
    }
  }
  return {raw_path.points.back().x_m, raw_path.points.back().y_m};
}

[[nodiscard]] bool valid_boundary(const PathBoundary& boundary) noexcept {
  return finite(boundary.x_m) && finite(boundary.y_m) &&
         finite(boundary.heading_rad) &&
         boundary.heading_rad >= -std::acos(-1.0) &&
         boundary.heading_rad < std::acos(-1.0) &&
         boundary.curvature_per_m.has_value() &&
         finite(*boundary.curvature_per_m);
}

[[nodiscard]] bool valid_start_boundary(
    const PathBoundary& boundary,
    const Duration maximum_time_skew) noexcept {
  const long double time_skew_ns = std::abs(
      static_cast<long double>(boundary.pose_timestamp.nanoseconds) -
      static_cast<long double>(boundary.curvature_timestamp.nanoseconds));
  if (!valid_boundary(boundary) ||
      boundary.curvature_source == PathBoundarySource::planned_goal ||
      boundary.pose_timestamp.nanoseconds < 0 ||
      boundary.curvature_timestamp.nanoseconds < 0 ||
      boundary.source_sequence_number == 0U ||
      maximum_time_skew.nanoseconds < 0 ||
      time_skew_ns >
          static_cast<long double>(maximum_time_skew.nanoseconds)) {
    return false;
  }
  return true;
}

[[nodiscard]] bool valid_allowed_residuals(
    const ConstraintResiduals& residuals) noexcept {
  const std::array<double, 9U> values{
      residuals.maximum_dynamics_residual,
      residuals.maximum_curvature_audit_residual,
      residuals.maximum_curvature_rate_residual,
      residuals.start_position_residual_m,
      residuals.start_heading_residual_rad,
      residuals.start_curvature_residual_per_m,
      residuals.goal_position_residual_m,
      residuals.goal_heading_residual_rad,
      residuals.goal_curvature_residual_per_m,
  };
  return std::all_of(values.begin(), values.end(),
                     [](const double value) {
                       return finite(value) && value >= 0.0;
                     });
}

[[nodiscard]] bool valid_limits(const SmoothingLimits& limits) noexcept {
  const std::array<double, 4U> objective_weights{
      limits.objective_weights.deviation,
      limits.objective_weights.curvature,
      limits.objective_weights.curvature_rate,
      limits.objective_weights.length,
  };
  return limits.version != 0U && limits.output_path_version != 0U &&
         finite(limits.spatial_step_m) &&
         limits.spatial_step_m > 0.0 &&
         finite(limits.maximum_curvature_per_m) &&
         limits.maximum_curvature_per_m > 0.0 &&
         finite(limits.maximum_curvature_rate_per_m2) &&
         limits.maximum_curvature_rate_per_m2 > 0.0 &&
         finite(limits.minimum_segment_length_m) &&
         limits.minimum_segment_length_m > 0.0 &&
         limits.minimum_segment_length_m <= limits.spatial_step_m &&
         finite(limits.topology_tube_radius_m) &&
         limits.topology_tube_radius_m > 0.0 &&
         limits.timeout.nanoseconds > 0 &&
         limits.maximum_boundary_time_skew.nanoseconds >= 0 &&
         valid_allowed_residuals(limits.allowed_residuals) &&
         std::all_of(objective_weights.begin(), objective_weights.end(),
                     [](const double weight) {
                       return finite(weight) && weight >= 0.0;
                     }) &&
         std::any_of(objective_weights.begin(), objective_weights.end(),
                     [](const double weight) { return weight > 0.0; });
}

[[nodiscard]] std::vector<double> project_rates(
    std::vector<double> rates, const double start_curvature_per_m,
    const double segment_length_m, const SmoothingLimits& limits) {
  double curvature_per_m = start_curvature_per_m;
  for (double& rate_per_m2 : rates) {
    rate_per_m2 = std::clamp(rate_per_m2,
                             -limits.maximum_curvature_rate_per_m2,
                             limits.maximum_curvature_rate_per_m2);
    const double next_curvature_per_m =
        curvature_per_m + rate_per_m2 * segment_length_m;
    if (next_curvature_per_m > limits.maximum_curvature_per_m) {
      rate_per_m2 =
          (limits.maximum_curvature_per_m - curvature_per_m) /
          segment_length_m;
    } else if (next_curvature_per_m < -limits.maximum_curvature_per_m) {
      rate_per_m2 =
          (-limits.maximum_curvature_per_m - curvature_per_m) /
          segment_length_m;
    }
    curvature_per_m += rate_per_m2 * segment_length_m;
  }
  return rates;
}

[[nodiscard]] GeometricPath integrate_path(
    const GeometricPath& raw_path, const PathBoundary& start,
    const double segment_length_m, const std::vector<double>& rates_per_m2) {
  GeometricPath path;
  path.metadata = raw_path.metadata;
  path.metadata.interpolation_rule = "clothoid-linear-curvature/v1";
  ClothoidState state{start.x_m, start.y_m, start.heading_rad,
                      *start.curvature_per_m};
  double arc_length_m = raw_path.points.front().arc_length_m;
  path.points.push_back({arc_length_m, state.x_m, state.y_m,
                         state.heading_rad, state.curvature_per_m});
  for (const double rate_per_m2 : rates_per_m2) {
    state = advance_clothoid(state, rate_per_m2, segment_length_m);
    arc_length_m += segment_length_m;
    path.points.push_back({arc_length_m, state.x_m, state.y_m,
                           state.heading_rad, state.curvature_per_m});
  }
  return path;
}

using TerminalError = std::array<double, 4U>;
using Objective = SmoothingResult::Audit::Objective;

[[nodiscard]] Objective evaluate_objective(
    const GeometricPath& raw_path, const GeometricPath& path,
    const SmoothingLimits& limits) {
  Objective objective;
  for (std::size_t index = 1U; index < path.points.size(); ++index) {
    const PathPoint& left = path.points[index - 1U];
    const PathPoint& right = path.points[index];
    const double length_m = right.arc_length_m - left.arc_length_m;
    objective.curvature +=
        0.5 * (left.curvature_per_m * left.curvature_per_m +
               right.curvature_per_m * right.curvature_per_m) *
        length_m;
    objective.length += length_m;
    const Vector2m raw_position =
        interpolate_raw_position(raw_path, right.arc_length_m);
    const double deviation_m =
        std::hypot(right.x_m - raw_position.x_m,
                   right.y_m - raw_position.y_m);
    objective.deviation += deviation_m * deviation_m * length_m;
    const double rate_per_m2 =
        (right.curvature_per_m - left.curvature_per_m) / length_m;
    objective.curvature_rate += rate_per_m2 * rate_per_m2 * length_m;
  }
  objective.deviation *= limits.objective_weights.deviation;
  objective.curvature *= limits.objective_weights.curvature;
  objective.curvature_rate *= limits.objective_weights.curvature_rate;
  objective.length *= limits.objective_weights.length;
  objective.total = objective.deviation + objective.curvature +
                    objective.curvature_rate + objective.length;
  return objective;
}

[[nodiscard]] TerminalError terminal_error(const GeometricPath& path,
                                           const PathBoundary& goal) {
  const PathPoint& terminal = path.points.back();
  return {terminal.x_m - goal.x_m, terminal.y_m - goal.y_m,
          angle_error(terminal.heading_rad, goal.heading_rad),
          terminal.curvature_per_m - *goal.curvature_per_m};
}

[[nodiscard]] double normalized_error_squared(
    const TerminalError& error, const ConstraintResiduals& allowed) noexcept {
  const double position_scale =
      std::max(allowed.goal_position_residual_m, 1.0e-12);
  const double heading_scale =
      std::max(allowed.goal_heading_residual_rad, 1.0e-12);
  const double curvature_scale =
      std::max(allowed.goal_curvature_residual_per_m, 1.0e-12);
  return (error[0] * error[0] + error[1] * error[1]) /
             (position_scale * position_scale) +
         error[2] * error[2] / (heading_scale * heading_scale) +
         error[3] * error[3] / (curvature_scale * curvature_scale);
}

[[nodiscard]] bool terminal_within_tolerance(
    const TerminalError& error, const ConstraintResiduals& allowed) noexcept {
  return std::hypot(error[0], error[1]) <=
             allowed.goal_position_residual_m &&
         std::abs(error[2]) <= allowed.goal_heading_residual_rad &&
         std::abs(error[3]) <= allowed.goal_curvature_residual_per_m;
}

[[nodiscard]] bool solve_four_by_four(
    std::array<std::array<double, 4U>, 4U> matrix,
    std::array<double, 4U> right_hand_side,
    std::array<double, 4U>& solution) noexcept {
  for (std::size_t pivot = 0U; pivot < 4U; ++pivot) {
    std::size_t best = pivot;
    for (std::size_t row = pivot + 1U; row < 4U; ++row) {
      if (std::abs(matrix.at(row).at(pivot)) >
          std::abs(matrix.at(best).at(pivot))) {
        best = row;
      }
    }
    if (std::abs(matrix.at(best).at(pivot)) < 1.0e-14) {
      return false;
    }
    std::swap(matrix.at(pivot), matrix.at(best));
    std::swap(right_hand_side.at(pivot), right_hand_side.at(best));
    const double divisor = matrix.at(pivot).at(pivot);
    for (std::size_t column = pivot; column < 4U; ++column) {
      matrix.at(pivot).at(column) /= divisor;
    }
    right_hand_side.at(pivot) /= divisor;
    for (std::size_t row = 0U; row < 4U; ++row) {
      if (row == pivot) {
        continue;
      }
      const double factor = matrix.at(row).at(pivot);
      for (std::size_t column = pivot; column < 4U; ++column) {
        matrix.at(row).at(column) -= factor * matrix.at(pivot).at(column);
      }
      right_hand_side.at(row) -= factor * right_hand_side.at(pivot);
    }
  }
  solution = right_hand_side;
  return true;
}

[[nodiscard]] SmoothingStatus improve_objective(
    const GeometricPath& raw_path, const PathBoundary& start,
    const PathBoundary& goal, const SmoothingLimits& limits,
    const double segment_length_m,
    const std::chrono::steady_clock::time_point deadline,
    std::vector<double>& rates, GeometricPath& path,
    std::size_t& iteration_count) {
  TerminalError error = terminal_error(path, goal);
  double objective = evaluate_objective(raw_path, path, limits).total;
  const std::array<double, 4U> scales{
      std::max(limits.allowed_residuals.goal_position_residual_m, 1.0e-12),
      std::max(limits.allowed_residuals.goal_position_residual_m, 1.0e-12),
      std::max(limits.allowed_residuals.goal_heading_residual_rad, 1.0e-12),
      std::max(limits.allowed_residuals.goal_curvature_residual_per_m, 1.0e-12),
  };
  const double perturbation =
      std::max(1.0e-7,
               limits.maximum_curvature_rate_per_m2 * 1.0e-5);

  for (std::size_t iteration = 0U;
       iteration < kMaximumObjectiveIterations; ++iteration) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return SmoothingStatus::solver_timeout;
    }
    std::vector<std::array<double, 4U>> jacobian(rates.size());
    std::vector<double> gradient(rates.size(), 0.0);
    for (std::size_t column = 0U; column < rates.size(); ++column) {
      if (std::chrono::steady_clock::now() >= deadline) {
        return SmoothingStatus::solver_timeout;
      }
      std::vector<double> perturbed = rates;
      perturbed[column] += perturbation;
      perturbed = project_rates(std::move(perturbed), *start.curvature_per_m,
                                segment_length_m, limits);
      const double actual_perturbation = perturbed[column] - rates[column];
      if (std::abs(actual_perturbation) <= 1.0e-15) {
        continue;
      }
      const GeometricPath perturbed_path =
          integrate_path(raw_path, start, segment_length_m, perturbed);
      if (std::chrono::steady_clock::now() >= deadline) {
        return SmoothingStatus::solver_timeout;
      }
      const TerminalError perturbed_error = terminal_error(perturbed_path, goal);
      for (std::size_t row = 0U; row < 4U; ++row) {
        double difference = perturbed_error[row] - error[row];
        if (row == 2U) {
          difference = angle_error(perturbed_error[row], error[row]);
        }
        jacobian[column][row] =
            difference / actual_perturbation / scales[row];
      }
      gradient[column] =
          (evaluate_objective(raw_path, perturbed_path, limits).total -
           objective) /
          actual_perturbation;
    }

    std::array<std::array<double, 4U>, 4U> normal{};
    std::array<double, 4U> projected_gradient_on_constraints{};
    for (std::size_t row = 0U; row < 4U; ++row) {
      for (std::size_t column = 0U; column < 4U; ++column) {
        for (const auto& derivative : jacobian) {
          normal[row][column] += derivative[row] * derivative[column];
        }
      }
      for (std::size_t column = 0U; column < rates.size(); ++column) {
        projected_gradient_on_constraints[row] +=
            jacobian[column][row] * gradient[column];
      }
      normal[row][row] += 1.0e-8;
    }
    std::array<double, 4U> multiplier{};
    if (std::chrono::steady_clock::now() >= deadline) {
      return SmoothingStatus::solver_timeout;
    }
    if (!solve_four_by_four(normal, projected_gradient_on_constraints,
                            multiplier)) {
      return SmoothingStatus::solver_failed;
    }
    std::vector<double> direction(rates.size(), 0.0);
    double maximum_direction = 0.0;
    for (std::size_t column = 0U; column < rates.size(); ++column) {
      double nullspace_gradient = gradient[column];
      for (std::size_t row = 0U; row < 4U; ++row) {
        nullspace_gradient -= jacobian[column][row] * multiplier[row];
      }
      direction[column] = -nullspace_gradient;
      maximum_direction =
          std::max(maximum_direction, std::abs(direction[column]));
    }
    if (!finite(maximum_direction)) {
      return SmoothingStatus::solver_failed;
    }
    if (!(maximum_direction > 1.0e-12)) {
      return SmoothingStatus::success;
    }
    const double direction_scale =
        limits.maximum_curvature_rate_per_m2 * 0.01 / maximum_direction;
    for (double& value : direction) {
      value *= direction_scale;
    }

    bool improved = false;
    for (double step = 1.0; step >= 1.0 / 4096.0; step *= 0.5) {
      if (std::chrono::steady_clock::now() >= deadline) {
        return SmoothingStatus::solver_timeout;
      }
      std::vector<double> candidate_rates = rates;
      for (std::size_t index = 0U; index < rates.size(); ++index) {
        candidate_rates[index] += step * direction[index];
      }
      candidate_rates = project_rates(std::move(candidate_rates),
                                      *start.curvature_per_m,
                                      segment_length_m, limits);
      GeometricPath candidate_path =
          integrate_path(raw_path, start, segment_length_m, candidate_rates);
      if (std::chrono::steady_clock::now() >= deadline) {
        return SmoothingStatus::solver_timeout;
      }
      const TerminalError candidate_error = terminal_error(candidate_path, goal);
      if (!terminal_within_tolerance(candidate_error,
                                     limits.allowed_residuals) ||
          !path_within_topology_tube(candidate_path, raw_path, limits)) {
        continue;
      }
      const double candidate_objective =
          evaluate_objective(raw_path, candidate_path, limits).total;
      const double minimum_improvement =
          1.0e-12 * std::max(1.0, std::abs(objective));
      if (candidate_objective < objective - minimum_improvement) {
        rates = std::move(candidate_rates);
        path = std::move(candidate_path);
        error = candidate_error;
        objective = candidate_objective;
        improved = true;
        ++iteration_count;
        break;
      }
    }
    if (!improved) {
      if (std::chrono::steady_clock::now() >= deadline) {
        return SmoothingStatus::solver_timeout;
      }
      return SmoothingStatus::success;
    }
  }
  return SmoothingStatus::solver_failed;
}

[[nodiscard]] SolverRun solve_default(const GeometricPath& raw_path,
                                      const PathBoundary& start,
                                      const PathBoundary& goal,
                                      const SmoothingLimits& limits) {
  SolverRun run;
  const double total_length_m = raw_path.points.back().arc_length_m -
                                raw_path.points.front().arc_length_m;
  const double segment_count_value =
      std::ceil(total_length_m / limits.spatial_step_m);
  if (!finite(total_length_m) || !finite(segment_count_value) ||
      segment_count_value < 1.0 ||
      segment_count_value > static_cast<double>(kMaximumSegments)) {
    run.status = SmoothingStatus::seed_infeasible;
    return run;
  }
  const std::size_t segment_count =
      static_cast<std::size_t>(segment_count_value);
  const double segment_length_m =
      total_length_m / static_cast<double>(segment_count);
  if (segment_length_m < limits.minimum_segment_length_m) {
    run.status = SmoothingStatus::seed_infeasible;
    return run;
  }

  std::vector<double> rates(segment_count, 0.0);
  double curvature_per_m = *start.curvature_per_m;
  for (std::size_t index = 0U; index < segment_count; ++index) {
    const double target_arc_length_m =
        raw_path.points.front().arc_length_m +
        segment_length_m * static_cast<double>(index + 1U);
    const double target_curvature_per_m =
        interpolate_raw_curvature(raw_path, target_arc_length_m);
    rates[index] = (target_curvature_per_m - curvature_per_m) /
                   segment_length_m;
    curvature_per_m = target_curvature_per_m;
  }
  rates = project_rates(std::move(rates), *start.curvature_per_m,
                        segment_length_m, limits);

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::nanoseconds(limits.timeout.nanoseconds);
  GeometricPath path = integrate_path(raw_path, start, segment_length_m, rates);
  TerminalError error = terminal_error(path, goal);
  for (std::size_t iteration = 0U;
       !terminal_within_tolerance(error, limits.allowed_residuals) &&
       iteration < kMaximumIterations;
       ++iteration) {
    if (std::chrono::steady_clock::now() >= deadline) {
      run.status = SmoothingStatus::solver_timeout;
      run.iterations = iteration;
      return run;
    }
    const std::array<double, 4U> scales{
        std::max(limits.allowed_residuals.goal_position_residual_m, 1.0e-12),
        std::max(limits.allowed_residuals.goal_position_residual_m, 1.0e-12),
        std::max(limits.allowed_residuals.goal_heading_residual_rad, 1.0e-12),
        std::max(limits.allowed_residuals.goal_curvature_residual_per_m, 1.0e-12),
    };
    std::vector<std::array<double, 4U>> jacobian(segment_count);
    const double perturbation =
        std::max(1.0e-7,
                 limits.maximum_curvature_rate_per_m2 * 1.0e-5);
    for (std::size_t column = 0U; column < segment_count; ++column) {
      if (std::chrono::steady_clock::now() >= deadline) {
        run.status = SmoothingStatus::solver_timeout;
        run.iterations = iteration;
        return run;
      }
      std::vector<double> perturbed = rates;
      perturbed[column] += perturbation;
      perturbed = project_rates(std::move(perturbed), *start.curvature_per_m,
                                segment_length_m, limits);
      const TerminalError perturbed_error = terminal_error(
          integrate_path(raw_path, start, segment_length_m, perturbed), goal);
      if (std::chrono::steady_clock::now() >= deadline) {
        run.status = SmoothingStatus::solver_timeout;
        run.iterations = iteration;
        return run;
      }
      const double actual_perturbation = perturbed[column] - rates[column];
      if (std::abs(actual_perturbation) <= 1.0e-15) {
        jacobian[column] = {};
        continue;
      }
      for (std::size_t row = 0U; row < 4U; ++row) {
        double difference = perturbed_error[row] - error[row];
        if (row == 2U) {
          difference = angle_error(perturbed_error[row], error[row]);
        }
        jacobian[column][row] =
            difference / actual_perturbation / scales[row];
      }
    }

    std::array<std::array<double, 4U>, 4U> normal{};
    for (std::size_t row = 0U; row < 4U; ++row) {
      for (std::size_t column = 0U; column < 4U; ++column) {
        for (const auto& derivative : jacobian) {
          normal[row][column] += derivative[row] * derivative[column];
        }
      }
      normal[row][row] += 1.0e-8;
    }
    std::array<double, 4U> normalized_error{};
    for (std::size_t row = 0U; row < 4U; ++row) {
      normalized_error[row] = error[row] / scales[row];
    }
    std::array<double, 4U> multiplier{};
    if (std::chrono::steady_clock::now() >= deadline) {
      run.status = SmoothingStatus::solver_timeout;
      run.iterations = iteration;
      return run;
    }
    if (!solve_four_by_four(normal, normalized_error, multiplier)) {
      run.status = SmoothingStatus::solver_failed;
      run.iterations = iteration;
      return run;
    }
    std::vector<double> delta(segment_count, 0.0);
    for (std::size_t column = 0U; column < segment_count; ++column) {
      for (std::size_t row = 0U; row < 4U; ++row) {
        delta[column] -= jacobian[column][row] * multiplier[row];
      }
    }

    const double previous_error =
        normalized_error_squared(error, limits.allowed_residuals);
    bool improved = false;
    for (double step = 1.0; step >= 1.0 / 128.0; step *= 0.5) {
      if (std::chrono::steady_clock::now() >= deadline) {
        run.status = SmoothingStatus::solver_timeout;
        run.iterations = iteration;
        return run;
      }
      std::vector<double> candidate_rates = rates;
      for (std::size_t index = 0U; index < segment_count; ++index) {
        candidate_rates[index] += step * delta[index];
      }
      candidate_rates = project_rates(std::move(candidate_rates),
                                      *start.curvature_per_m,
                                      segment_length_m, limits);
      GeometricPath candidate_path =
          integrate_path(raw_path, start, segment_length_m, candidate_rates);
      if (std::chrono::steady_clock::now() >= deadline) {
        run.status = SmoothingStatus::solver_timeout;
        run.iterations = iteration;
        return run;
      }
      const TerminalError candidate_error = terminal_error(candidate_path, goal);
      if (normalized_error_squared(candidate_error, limits.allowed_residuals) <
          previous_error) {
        rates = std::move(candidate_rates);
        path = std::move(candidate_path);
        error = candidate_error;
        improved = true;
        break;
      }
    }
    run.iterations = iteration + 1U;
    if (!improved) {
      run.status = SmoothingStatus::solver_failed;
      return run;
    }
  }
  if (!terminal_within_tolerance(error, limits.allowed_residuals)) {
    run.status = SmoothingStatus::solver_failed;
    return run;
  }
  const SmoothingStatus objective_status = improve_objective(
      raw_path, start, goal, limits, segment_length_m, deadline, rates, path,
      run.iterations);
  if (objective_status != SmoothingStatus::success) {
    run.status = objective_status;
    return run;
  }
  run.status = SmoothingStatus::success;
  run.path = std::move(path);
  return run;
}

class DefaultPathSmoothingSolver final : public PathSmoothingSolver {
 public:
  [[nodiscard]] SmoothingSolverResult solve(
      const GeometricPath& raw_path, const PathBoundary& start,
      const PathBoundary& goal,
      const SmoothingLimits& limits) const override {
    SolverRun run = solve_default(raw_path, start, goal, limits);
    return {run.status, std::move(run.path), run.iterations};
  }
};

[[nodiscard]] std::string_view solver_status_name(
    const SmoothingStatus status) noexcept {
  switch (status) {
    case SmoothingStatus::success:
      return "converged";
    case SmoothingStatus::seed_infeasible:
      return "seed_infeasible";
    case SmoothingStatus::solver_timeout:
      return "timeout";
    case SmoothingStatus::solver_failed:
      return "failed";
    default:
      return "invalid_solver_status";
  }
}

[[nodiscard]] ConstraintResiduals calculate_residuals(
    const GeometricPath& path, const PathBoundary& start,
    const PathBoundary& goal, const SmoothingLimits& limits) {
  ConstraintResiduals residuals;
  const PathPoint& first = path.points.front();
  const PathPoint& last = path.points.back();
  residuals.start_position_residual_m =
      std::hypot(first.x_m - start.x_m, first.y_m - start.y_m);
  residuals.start_heading_residual_rad =
      std::abs(angle_error(first.heading_rad, start.heading_rad));
  residuals.start_curvature_residual_per_m =
      std::abs(first.curvature_per_m - *start.curvature_per_m);
  residuals.goal_position_residual_m =
      std::hypot(last.x_m - goal.x_m, last.y_m - goal.y_m);
  residuals.goal_heading_residual_rad =
      std::abs(angle_error(last.heading_rad, goal.heading_rad));
  residuals.goal_curvature_residual_per_m =
      std::abs(last.curvature_per_m - *goal.curvature_per_m);

  for (std::size_t index = 1U; index < path.points.size(); ++index) {
    const PathPoint& left = path.points[index - 1U];
    const PathPoint& right = path.points[index];
    const double length_m = right.arc_length_m - left.arc_length_m;
    if (!(length_m >= limits.minimum_segment_length_m)) {
      residuals.maximum_dynamics_residual =
          std::numeric_limits<double>::infinity();
      continue;
    }
    const double rate_per_m2 =
        (right.curvature_per_m - left.curvature_per_m) / length_m;
    const ClothoidState expected = advance_clothoid(
        {left.x_m, left.y_m, left.heading_rad, left.curvature_per_m},
        rate_per_m2, length_m);
    if (!finite(expected)) {
      residuals.maximum_dynamics_residual =
          std::numeric_limits<double>::infinity();
      continue;
    }
    residuals.maximum_dynamics_residual = std::max(
        residuals.maximum_dynamics_residual,
        std::max({std::hypot(expected.x_m - right.x_m,
                            expected.y_m - right.y_m),
                  std::abs(angle_error(expected.heading_rad,
                                       right.heading_rad)),
                  std::abs(expected.curvature_per_m -
                           right.curvature_per_m)}));
    residuals.maximum_curvature_rate_residual = std::max(
        residuals.maximum_curvature_rate_residual,
        std::max(0.0, std::abs(rate_per_m2) -
                          limits.maximum_curvature_rate_per_m2));
  }
  for (const PathPoint& point : path.points) {
    residuals.maximum_curvature_audit_residual = std::max(
        residuals.maximum_curvature_audit_residual,
        std::max(0.0, std::abs(point.curvature_per_m) -
                          limits.maximum_curvature_per_m));
  }
  return residuals;
}

[[nodiscard]] bool residuals_within_limits(
    const ConstraintResiduals& residuals,
    const ConstraintResiduals& allowed) noexcept {
  return residuals.maximum_dynamics_residual <=
             allowed.maximum_dynamics_residual &&
         residuals.maximum_curvature_audit_residual <=
             allowed.maximum_curvature_audit_residual &&
         residuals.maximum_curvature_rate_residual <=
             allowed.maximum_curvature_rate_residual &&
         residuals.start_position_residual_m <=
             allowed.start_position_residual_m &&
         residuals.start_heading_residual_rad <=
             allowed.start_heading_residual_rad &&
         residuals.start_curvature_residual_per_m <=
             allowed.start_curvature_residual_per_m &&
         residuals.goal_position_residual_m <=
             allowed.goal_position_residual_m &&
         residuals.goal_heading_residual_rad <=
             allowed.goal_heading_residual_rad &&
         residuals.goal_curvature_residual_per_m <=
             allowed.goal_curvature_residual_per_m;
}

[[nodiscard]] double maximum_constraint_residual(
    const ConstraintResiduals& residuals) noexcept {
  return std::max({residuals.maximum_dynamics_residual,
                   residuals.maximum_curvature_audit_residual,
                   residuals.maximum_curvature_rate_residual,
                   residuals.start_position_residual_m,
                   residuals.start_heading_residual_rad,
                   residuals.start_curvature_residual_per_m,
                   residuals.goal_position_residual_m,
                   residuals.goal_heading_residual_rad,
                   residuals.goal_curvature_residual_per_m});
}

void calculate_audit(const GeometricPath& raw_path,
                     const GeometricPath& path,
                     const SmoothingLimits& limits,
                     SmoothingResult::Audit& audit) {
  audit.smoother_version = std::string(kSmootherVersion);
  audit.objective = evaluate_objective(raw_path, path, limits);
  for (std::size_t index = 0U; index < path.points.size(); ++index) {
    const PathPoint& point = path.points[index];
    audit.maximum_absolute_curvature_per_m =
        std::max(audit.maximum_absolute_curvature_per_m,
                 std::abs(point.curvature_per_m));
  }
  for (std::size_t index = 0U; index + 1U < path.points.size(); ++index) {
    const double rate_per_m2 =
        (path.points[index + 1U].curvature_per_m -
         path.points[index].curvature_per_m) /
        (path.points[index + 1U].arc_length_m -
         path.points[index].arc_length_m);
    audit.maximum_absolute_curvature_rate_per_m2 =
        std::max(audit.maximum_absolute_curvature_rate_per_m2,
                 std::abs(rate_per_m2));
  }
}

}  // namespace

PathSmoother::PathSmoother()
    : solver_(std::make_shared<DefaultPathSmoothingSolver>()) {}

PathSmoother::PathSmoother(std::shared_ptr<const PathSmoothingSolver> solver)
    : solver_(std::move(solver)) {
  if (!solver_) {
    throw std::invalid_argument("path smoothing solver must be provided");
  }
}

TrackabilityResult PathSmoother::validateTrackability(
    const GeometricPath& path, const GeometricPath& raw_path,
    const PathBoundary& start, const PathBoundary& goal,
    const SmoothingLimits& limits) const {
  TrackabilityResult result;
  if (!validate(path).valid || !validate(raw_path).valid ||
      !parse_raw_interpolation(raw_path).has_value() ||
      !valid_start_boundary(start, limits.maximum_boundary_time_skew) ||
      !valid_boundary(goal) ||
      !valid_limits(limits)) {
    result.reason = "path_geometry_or_validation_context_invalid";
    return result;
  }
  result.residuals = calculate_residuals(path, start, goal, limits);
  if (!residuals_within_limits(result.residuals,
                               limits.allowed_residuals)) {
    result.reason = "constraint_residual_exceeded";
    return result;
  }
  if (!path_within_topology_tube(path, raw_path, limits)) {
    result.reason = "topology_tube_exceeded";
    return result;
  }
  result.valid = true;
  result.reason = "valid";
  return result;
}

SmoothingResult PathSmoother::smooth(const GeometricPath& raw_path,
                                     const PathBoundary& start,
                                     const PathBoundary& goal,
                                     const SmoothingLimits& limits) const {
  SmoothingResult result;
  result.audit.smoother_version = std::string(kSmootherVersion);
  result.audit.limits_version = limits.version;
  if (!valid_start_boundary(start, limits.maximum_boundary_time_skew) ||
      !valid_boundary(goal)) {
    result.status = SmoothingStatus::boundary_state_invalid;
    result.audit.solver_status = "not_run_boundary_state_invalid";
    return result;
  }
  if (!validate(raw_path).valid ||
      !parse_raw_interpolation(raw_path).has_value() ||
      !valid_limits(limits) ||
      std::abs(*start.curvature_per_m) > limits.maximum_curvature_per_m ||
      std::abs(*goal.curvature_per_m) > limits.maximum_curvature_per_m) {
    result.status = SmoothingStatus::seed_infeasible;
    result.audit.solver_status = "not_run_seed_infeasible";
    return result;
  }

  const SmoothingSolverResult solver_result =
      solver_->solve(raw_path, start, goal, limits);
  result.audit.solver_iterations = solver_result.iterations;
  if (solver_result.status != SmoothingStatus::success) {
    const bool valid_solver_failure =
        solver_result.status == SmoothingStatus::seed_infeasible ||
        solver_result.status == SmoothingStatus::solver_timeout ||
        solver_result.status == SmoothingStatus::solver_failed;
    result.status = valid_solver_failure ? solver_result.status
                                         : SmoothingStatus::solver_failed;
    result.audit.solver_status = std::string(solver_status_name(result.status));
    return result;
  }
  result.audit.solver_status =
      std::string(solver_status_name(solver_result.status));
  GeometricPath candidate = solver_result.candidate;
  candidate.metadata = raw_path.metadata;
  candidate.metadata.path_version = limits.output_path_version;
  candidate.metadata.interpolation_rule = "clothoid-linear-curvature/v1";
  candidate.metadata.smoothing.reset();
  const TrackabilityResult trackability = validateTrackability(
      candidate, raw_path, start, goal, limits);
  result.residuals = trackability.residuals;
  if (!trackability.valid) {
    result.status = trackability.reason == "constraint_residual_exceeded"
                        ? SmoothingStatus::constraint_residual_exceeded
                        : SmoothingStatus::trackability_validation_failed;
    return result;
  }
  calculate_audit(raw_path, candidate, limits, result.audit);
  GeometricPath output = std::move(candidate);
  PathSmoothingMetadata smoothing;
  smoothing.smoother_version = result.audit.smoother_version;
  smoothing.solver_status = result.audit.solver_status;
  smoothing.limits_version = result.audit.limits_version;
  smoothing.maximum_constraint_residual =
      maximum_constraint_residual(result.residuals);
  smoothing.maximum_absolute_curvature_per_m =
      result.audit.maximum_absolute_curvature_per_m;
  smoothing.maximum_absolute_curvature_rate_per_m2 =
      result.audit.maximum_absolute_curvature_rate_per_m2;
  smoothing.residuals = result.residuals;
  output.metadata.smoothing = std::move(smoothing);
  result.status = SmoothingStatus::success;
  result.path = std::move(output);
  return result;
}

std::string serialize_smoothing_limits(const SmoothingLimits& limits) {
  const ConstraintResiduals& residuals = limits.allowed_residuals;
  const SmoothingObjectiveWeights& weights = limits.objective_weights;
  std::ostringstream output;
  output << std::setprecision(17) << limits.version << '\n'
         << limits.output_path_version << '\n' << limits.spatial_step_m << '\n'
         << limits.maximum_curvature_per_m << '\n'
         << limits.maximum_curvature_rate_per_m2 << '\n'
         << limits.minimum_segment_length_m << '\n'
         << limits.topology_tube_radius_m << '\n'
         << limits.timeout.nanoseconds << '\n'
         << limits.maximum_boundary_time_skew.nanoseconds << '\n'
         << residuals.maximum_dynamics_residual << '\n'
         << residuals.maximum_curvature_audit_residual << '\n'
         << residuals.maximum_curvature_rate_residual << '\n'
         << residuals.start_position_residual_m << '\n'
         << residuals.start_heading_residual_rad << '\n'
         << residuals.start_curvature_residual_per_m << '\n'
         << residuals.goal_position_residual_m << '\n'
         << residuals.goal_heading_residual_rad << '\n'
         << residuals.goal_curvature_residual_per_m << '\n'
         << weights.deviation << '\n' << weights.curvature << '\n'
         << weights.curvature_rate << '\n' << weights.length << '\n';
  return output.str();
}

}  // namespace underwater_planner::core
