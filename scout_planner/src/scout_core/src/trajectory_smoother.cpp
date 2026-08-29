#include "scout_planner/core/trajectory_smoother.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace scout_planner::core {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kEpsilon = 1.0e-9;

bool finite(const double value) noexcept { return std::isfinite(value); }
bool finite(const Point3dEnu& p) noexcept {
  return finite(p.x_m) && finite(p.y_m) && finite(p.z_m);
}
Point3dEnu add(const Point3dEnu a, const Point3dEnu b) noexcept {
  return {a.x_m + b.x_m, a.y_m + b.y_m, a.z_m + b.z_m};
}
Point3dEnu sub(const Point3dEnu a, const Point3dEnu b) noexcept {
  return {a.x_m - b.x_m, a.y_m - b.y_m, a.z_m - b.z_m};
}
Point3dEnu scale(const Point3dEnu p, const double s) noexcept {
  return {p.x_m * s, p.y_m * s, p.z_m * s};
}
double norm(const Point3dEnu p) noexcept {
  return std::sqrt(p.x_m * p.x_m + p.y_m * p.y_m + p.z_m * p.z_m);
}
double distance(const Point3dEnu a, const Point3dEnu b) noexcept {
  return norm(sub(a, b));
}
double normalize_yaw(double yaw) noexcept {
  yaw = std::fmod(yaw + kPi, 2.0 * kPi);
  if (yaw < 0.0) yaw += 2.0 * kPi;
  return yaw - kPi;
}
double unwrap_delta(const double delta) noexcept {
  return normalize_yaw(delta);
}

struct Candidate {
  std::vector<QuinticBezierSegment4d> segments;
  double initial_yaw{};
};

bool valid_config(const TrajectorySmootherConfig& c) noexcept {
  const std::array values{c.tube_offset_scale, c.minimum_segment_duration_s,
                          c.maximum_speed_mps, c.maximum_acceleration_mps2,
                          c.maximum_yaw_rate_rps,
                          c.maximum_yaw_acceleration_rps2};
  const auto valid_boundary = [&](const std::optional<TrajectoryBoundaryState>& state) {
    if (!state.has_value()) return true;
    return finite(state->velocity_mps) && finite(state->acceleration_mps2) &&
           finite(state->yaw_rate_rps) && finite(state->yaw_acceleration_rps2);
  };
  return c.tube_offset_scale > 0.0 && c.tube_offset_scale < 1.0 &&
         c.minimum_segment_duration_s > 0.0 &&
         std::all_of(values.begin(), values.end(), [](const double v) {
           return std::isfinite(v);
         }) &&
         c.maximum_speed_mps > 0.0 && c.maximum_acceleration_mps2 > 0.0 &&
         c.maximum_yaw_rate_rps > 0.0 &&
         c.maximum_yaw_acceleration_rps2 > 0.0 && c.maximum_segments > 0U &&
         c.maximum_attempts > 0U && valid_boundary(c.start_state) &&
         valid_boundary(c.goal_state);
}

bool expired(const TrajectorySmootherConfig& c) {
  return c.deadline_monotonic_ns != 0U && c.monotonic_now_ns &&
         c.monotonic_now_ns() >= c.deadline_monotonic_ns;
}

std::vector<Point3dEnu> densify(const std::vector<Point3dEnu>& points) {
  std::vector<Point3dEnu> output;
  output.reserve(points.size() * 2U - 1U);
  for (std::size_t i = 0U; i + 1U < points.size(); ++i) {
    output.push_back(points[i]);
    output.push_back(scale(add(points[i], points[i + 1U]), 0.5));
  }
  output.push_back(points.back());
  return output;
}

bool within(const Point3dEnu p, const Point3dEnu center, const double radius) {
  return distance(p, center) <= radius + kEpsilon;
}

bool within_tube(const Candidate& candidate, const FeasibleTube& tube) {
  for (std::size_t segment_index = 0U; segment_index < candidate.segments.size();
       ++segment_index) {
    const auto& raw = candidate.segments[segment_index];
    const auto segment = QuinticBezierSegment::create(raw);
    if (!segment.has_value()) return false;
    for (std::size_t sample = 0U; sample <= 32U; ++sample) {
      const auto value = segment.value().evaluate_normalized(
          static_cast<double>(sample) / 32.0);
      if (!value.has_value()) return false;
      const bool covered = within(value.value().position,
                                  tube.samples[segment_index].center_m,
                                  tube.samples[segment_index].radius_m) ||
                           within(value.value().position,
                                  tube.samples[segment_index + 1U].center_m,
                                  tube.samples[segment_index + 1U].radius_m);
      if (!covered) return false;
    }
  }
  return true;
}

BezierResult<BezierTrajectory4d> trajectory_for(const Candidate& candidate,
                                                const std::string& frame_id) {
  return BezierTrajectory4d::create(frame_id, candidate.initial_yaw,
                                    candidate.segments);
}

std::optional<Candidate> make_candidate(
    const std::vector<Point3dEnu>& points, const std::optional<StateLatticePath>& seed,
    const FeasibleTube& tube, const TrajectorySmootherConfig& config,
    const double duration_scale) {
  if (points.size() < 2U || points.size() > config.maximum_segments + 1U ||
      tube.samples.size() != points.size()) return std::nullopt;

  std::vector<double> durations(points.size() - 1U, 0.0);
  if (seed.has_value() && seed->states.size() == points.size()) {
    for (std::size_t i = 0U; i + 1U < points.size(); ++i) {
      const auto start = seed->states[i].arrival_time_offset_ns;
      const auto end = seed->states[i + 1U].arrival_time_offset_ns;
      if (end <= start) return std::nullopt;
      durations[i] = static_cast<double>(end - start) / 1.0e9;
    }
  } else {
    for (std::size_t i = 0U; i + 1U < points.size(); ++i) {
      const double length = distance(points[i], points[i + 1U]);
      if (!(length > kEpsilon)) return std::nullopt;
      durations[i] = std::max(config.minimum_segment_duration_s,
                              length / config.maximum_speed_mps);
    }
  }
  for (double& duration : durations) {
    duration = std::max(config.minimum_segment_duration_s,
                        duration * duration_scale);
    if (!finite(duration) || duration <= 0.0 ||
        duration > static_cast<double>(std::numeric_limits<std::uint64_t>::max()) /
                       1.0e9) return std::nullopt;
  }

  std::vector<Point3dEnu> velocities(points.size());
  for (std::size_t i = 1U; i + 1U < points.size(); ++i) {
    const double before = durations[i - 1U];
    const double after = durations[i];
    const auto incoming = scale(sub(points[i], points[i - 1U]), 1.0 / before);
    const auto outgoing = scale(sub(points[i + 1U], points[i]), 1.0 / after);
    velocities[i] = scale(add(incoming, outgoing), 0.5);
    const double speed = norm(velocities[i]);
    if (speed > config.maximum_speed_mps) {
      velocities[i] = scale(velocities[i], config.maximum_speed_mps / speed);
    }
  }
  velocities.front() = scale(sub(points[1U], points.front()),
                              1.0 / durations.front());
  velocities.back() = scale(sub(points.back(), points[points.size() - 2U]),
                             1.0 / durations.back());
  if (config.start_state.has_value()) velocities.front() = config.start_state->velocity_mps;
  if (config.goal_state.has_value()) velocities.back() = config.goal_state->velocity_mps;
  if (!std::all_of(velocities.begin(), velocities.end(), [](const auto p) {
        return finite(p) && norm(p) >= 0.0;
      }) || norm(velocities.front()) > config.maximum_speed_mps + kEpsilon ||
      norm(velocities.back()) > config.maximum_speed_mps + kEpsilon) return std::nullopt;

  std::vector<double> yaw(points.size(), 0.0);
  yaw[0] = std::atan2(points[1].y_m - points[0].y_m,
                      points[1].x_m - points[0].x_m);
  for (std::size_t i = 1U; i < points.size(); ++i) {
    const double target = std::atan2(points[i].y_m - points[i - 1U].y_m,
                                     points[i].x_m - points[i - 1U].x_m);
    yaw[i] = yaw[i - 1U] + unwrap_delta(target - yaw[i - 1U]);
  }
  std::vector<double> yaw_rates(points.size(), 0.0);
  for (std::size_t i = 1U; i + 1U < points.size(); ++i) {
    yaw_rates[i] = unwrap_delta(yaw[i + 1U] - yaw[i - 1U]) /
                   (durations[i - 1U] + durations[i]);
    yaw_rates[i] = std::clamp(yaw_rates[i], -config.maximum_yaw_rate_rps,
                              config.maximum_yaw_rate_rps);
  }
  yaw_rates.front() = unwrap_delta(yaw[1] - yaw[0]) / durations.front();
  yaw_rates.back() = unwrap_delta(yaw.back() - yaw[yaw.size() - 2U]) /
                     durations.back();
  if (config.start_state.has_value()) yaw_rates.front() = config.start_state->yaw_rate_rps;
  if (config.goal_state.has_value()) yaw_rates.back() = config.goal_state->yaw_rate_rps;
  if (std::abs(yaw_rates.front()) > config.maximum_yaw_rate_rps + kEpsilon ||
      std::abs(yaw_rates.back()) > config.maximum_yaw_rate_rps + kEpsilon) return std::nullopt;

  std::vector<QuinticBezierSegment4d> segments;
  segments.reserve(points.size() - 1U);
  std::uint64_t start_ns = 0U;
  for (std::size_t i = 0U; i + 1U < points.size(); ++i) {
    const double duration = durations[i];
    const auto start_acceleration =
        (i == 0U && config.start_state.has_value())
            ? config.start_state->acceleration_mps2
            : Point3dEnu{0.0, 0.0, 0.0};
    const auto end_acceleration =
        (i + 1U == points.size() - 1U && config.goal_state.has_value())
            ? config.goal_state->acceleration_mps2
            : Point3dEnu{0.0, 0.0, 0.0};
    const double t2 = duration * duration;
    QuinticBezierSegment4d segment{};
    segment.start_time_offset_ns = start_ns;
    segment.duration_ns = static_cast<std::uint64_t>(std::llround(duration * 1.0e9));
    segment.position_control_points[0] = points[i];
    segment.position_control_points[1] = add(points[i], scale(velocities[i], duration / 5.0));
    segment.position_control_points[2] = add(
        sub(scale(segment.position_control_points[1], 2.0), points[i]),
        scale(start_acceleration, t2 / 20.0));
    segment.position_control_points[5] = points[i + 1U];
    segment.position_control_points[4] = sub(points[i + 1U], scale(velocities[i + 1U], duration / 5.0));
    segment.position_control_points[3] = add(sub(scale(end_acceleration, t2 / 20.0), points[i + 1U]), scale(segment.position_control_points[4], 2.0));
    segment.yaw_offset_control_points_rad[0] = yaw[i] - yaw[0];
    segment.yaw_offset_control_points_rad[1] = segment.yaw_offset_control_points_rad[0] + yaw_rates[i] * duration / 5.0;
    segment.yaw_offset_control_points_rad[2] =
        2.0 * segment.yaw_offset_control_points_rad[1] -
        segment.yaw_offset_control_points_rad[0] +
        ((i == 0U && config.start_state.has_value())
             ? config.start_state->yaw_acceleration_rps2 * t2 / 20.0
             : 0.0);
    segment.yaw_offset_control_points_rad[5] = yaw[i + 1U] - yaw[0];
    segment.yaw_offset_control_points_rad[4] = segment.yaw_offset_control_points_rad[5] - yaw_rates[i + 1U] * duration / 5.0;
    segment.yaw_offset_control_points_rad[3] =
        2.0 * segment.yaw_offset_control_points_rad[4] -
        segment.yaw_offset_control_points_rad[5] +
        ((i + 1U == points.size() - 1U && config.goal_state.has_value())
             ? config.goal_state->yaw_acceleration_rps2 * t2 / 20.0
             : 0.0);
    if (!within(segment.position_control_points[1], points[i], tube.samples[i].radius_m) ||
        !within(segment.position_control_points[2], points[i], tube.samples[i].radius_m) ||
        !within(segment.position_control_points[3], points[i + 1U], tube.samples[i + 1U].radius_m) ||
        !within(segment.position_control_points[4], points[i + 1U], tube.samples[i + 1U].radius_m)) return std::nullopt;
    auto checked = QuinticBezierSegment::create(segment);
    if (!checked.has_value()) return std::nullopt;
    const auto bounds = checked.value().derivative_bounds();
    if (bounds.maximum_speed_mps > config.maximum_speed_mps + kEpsilon ||
        bounds.maximum_acceleration_mps2 > config.maximum_acceleration_mps2 + kEpsilon ||
        bounds.maximum_yaw_rate_rps > config.maximum_yaw_rate_rps + kEpsilon ||
        bounds.maximum_yaw_acceleration_rps2 > config.maximum_yaw_acceleration_rps2 + kEpsilon) return std::nullopt;
    segments.push_back(std::move(segment));
    if (i + 1U < points.size() - 1U) {
      const auto duration_ns = segments.back().duration_ns;
      if (start_ns > std::numeric_limits<std::uint64_t>::max() - duration_ns) return std::nullopt;
      start_ns += duration_ns;
    }
  }
  return Candidate{std::move(segments), yaw[0]};
}

}  // namespace

MapQueryResult<FeasibleTube> TrajectorySmoother::build_feasible_tube(
    const HybridMapQuery& map, const std::vector<Point3dEnu>& points,
    const TrajectorySmootherConfig& config) {
  if (!valid_config(config) || points.size() < 2U ||
      points.size() > config.maximum_segments + 1U ||
      !std::all_of(points.begin(), points.end(), [](const Point3dEnu point) {
        return finite(point);
      })) {
    return MapQueryResult<FeasibleTube>::failure(
        {MapQueryErrorCode::invalid_position, "smoothing input or configuration is invalid"});
  }
  FeasibleTube tube;
  tube.offset_scale = config.tube_offset_scale;
  tube.samples.reserve(points.size());
  for (const auto point : points) {
    const auto query = map.query_point(point, config.safety_margins);
    if (!query.has_value()) return MapQueryResult<FeasibleTube>::failure(query.error());
    const auto& sample = query.value();
    if (sample.state != MapCellState::free || !sample.allowed_water ||
        !finite(sample.clearance_margin_m) || sample.clearance_margin_m <= 0.0) {
      return MapQueryResult<FeasibleTube>::failure(
          {MapQueryErrorCode::invalid_position, "path point has no positive free clearance"});
    }
    tube.samples.push_back({point, sample.clearance_margin_m * config.tube_offset_scale,
                            sample.clearance_margin_m});
  }
  for (std::size_t i = 0U; i + 1U < tube.samples.size(); ++i) {
    if (distance(tube.samples[i].center_m, tube.samples[i + 1U].center_m) >
        tube.samples[i].radius_m + tube.samples[i + 1U].radius_m + kEpsilon) {
      return MapQueryResult<FeasibleTube>::failure(
          {MapQueryErrorCode::invalid_position, "adjacent feasible tube balls do not overlap"});
    }
  }
  return MapQueryResult<FeasibleTube>::success(std::move(tube));
}

TrajectorySmoothingResult TrajectorySmoother::smooth(
    const HybridMapQuery& map, const StateLatticePath& path,
    const TrajectorySmootherConfig& config, const std::string& frame_id) {
  if ((path.status != StateLatticeSearchStatus::found_path &&
       path.status != StateLatticeSearchStatus::found_survey_path) ||
      path.points_m.size() < 2U || frame_id != "mission_enu") {
    return {TrajectorySmoothingStatus::invalid_input, std::nullopt, std::nullopt,
            0U, TrajectorySmoothingFailureStage::none, "path/frame is invalid"};
  }
  std::optional<StateLatticePath> seed = path;
  auto result = smooth(map, path.points_m, config, frame_id);
  if (result.has_value() && seed->states.size() == path.points_m.size()) {
    // Rebuild with the searcher's physical arrival times when they are available.
    const auto tube = build_feasible_tube(map, path.points_m, config);
    if (tube.has_value()) {
      const auto candidate = make_candidate(path.points_m, seed, tube.value(), config, 1.0);
      if (candidate.has_value() && within_tube(*candidate, tube.value())) {
        auto trajectory = trajectory_for(*candidate, frame_id);
        if (trajectory.has_value()) {
          result.trajectory = trajectory.value();
        } else {
          result.status = TrajectorySmoothingStatus::smoothing_failed;
          result.trajectory.reset();
          result.detail = "search arrival timing cannot satisfy trajectory constraints";
        }
      } else {
        result.status = TrajectorySmoothingStatus::smoothing_failed;
        result.trajectory.reset();
        result.detail = "search arrival timing cannot satisfy trajectory constraints";
      }
    } else {
      result.status = TrajectorySmoothingStatus::tube_infeasible;
      result.trajectory.reset();
      result.detail = tube.error().detail;
    }
  }
  return result;
}

TrajectorySmoothingResult TrajectorySmoother::smooth(
    const HybridMapQuery& map, const std::vector<Point3dEnu>& input_points,
    const TrajectorySmootherConfig& config, const std::string& frame_id) {
  TrajectorySmoothingResult result{};
  if (!valid_config(config) || frame_id != "mission_enu" || input_points.size() < 2U ||
      input_points.size() > config.maximum_segments + 1U ||
      !std::all_of(input_points.begin(), input_points.end(), [](const Point3dEnu point) {
        return finite(point);
      })) {
    result.status = TrajectorySmoothingStatus::invalid_input;
    result.detail = "path, frame, or smoothing configuration is invalid";
    return result;
  }
  std::vector<Point3dEnu> points = input_points;
  bool alternative_attempted = false;
  for (std::size_t attempt = 0U; attempt < config.maximum_attempts; ++attempt) {
    result.attempts = attempt + 1U;
    if (expired(config)) {
      result.status = TrajectorySmoothingStatus::timeout;
      result.last_failure_stage = TrajectorySmoothingFailureStage::constraints;
      result.detail = "smoothing deadline expired";
      return result;
    }
    TrajectorySmootherConfig attempt_config = config;
    if (attempt == 1U) {
      attempt_config.tube_offset_scale = config.tube_offset_scale * 0.75;
      result.last_failure_stage = TrajectorySmoothingFailureStage::shrink_offset;
    } else if (attempt == 2U) {
      if (points.size() * 2U - 1U <= config.maximum_segments + 1U) points = densify(input_points);
      result.last_failure_stage = TrajectorySmoothingFailureStage::add_control_points;
    } else if (attempt == 3U) {
      attempt_config.tube_offset_scale = config.tube_offset_scale * 0.75;
      result.last_failure_stage = TrajectorySmoothingFailureStage::extend_duration;
    } else if (attempt >= 4U) {
      if (!alternative_attempted && config.alternative_path_provider) {
        alternative_attempted = true;
        const auto alternative = config.alternative_path_provider();
        if (!alternative.empty()) points = alternative;
      }
      result.last_failure_stage = TrajectorySmoothingFailureStage::constraints;
    }
    const auto tube = build_feasible_tube(map, points, attempt_config);
    if (!tube.has_value()) {
      result.last_failure_stage = TrajectorySmoothingFailureStage::build_tube;
      result.detail = tube.error().detail;
      continue;
    }
    result.feasible_tube = tube.value();
    const auto candidate = make_candidate(points, std::nullopt, tube.value(), attempt_config,
                                           attempt == 3U ? 4.0 : 1.0);
    if (!candidate.has_value() || !within_tube(*candidate, tube.value())) {
      result.last_failure_stage = attempt >= 3U ? TrajectorySmoothingFailureStage::extend_duration
                                                : TrajectorySmoothingFailureStage::constraints;
      result.detail = "quintic controls violate tube or dynamic constraints";
      continue;
    }
    auto trajectory = trajectory_for(*candidate, frame_id);
    if (!trajectory.has_value()) {
      result.last_failure_stage = TrajectorySmoothingFailureStage::constraints;
      result.detail = trajectory.error().detail;
      continue;
    }
    result.status = TrajectorySmoothingStatus::success;
    result.trajectory = trajectory.value();
    result.detail.clear();
    return result;
  }
  result.status = result.last_failure_stage == TrajectorySmoothingFailureStage::build_tube
                      ? TrajectorySmoothingStatus::tube_infeasible
                      : TrajectorySmoothingStatus::smoothing_failed;
  if (result.detail.empty()) result.detail = "all deterministic smoothing attempts failed";
  return result;
}

}  // namespace scout_planner::core
