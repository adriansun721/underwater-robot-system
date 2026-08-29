#include "underwater_planner/core/cable_laying_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>
#include <set>
#include <utility>

namespace underwater_planner::core {
namespace {

constexpr double kGeometryToleranceM = 1.0e-12;
constexpr double kArcLengthToleranceM = 1.0e-9;
constexpr std::size_t kMaximumSweepIntervalsPerSegment = 1'000'000U;

bool finite(const double value) { return std::isfinite(value); }

bool finite(const Vector2m value) {
  return finite(value.x_m) && finite(value.y_m);
}

double distance(const Vector2m left, const Vector2m right) {
  return std::hypot(left.x_m - right.x_m, left.y_m - right.y_m);
}

Vector2m point(const PathPoint& value) { return {value.x_m, value.y_m}; }

bool valid_limits(const CableLayingLimits& limits) {
  return limits.version != 0U && !limits.operating_domain_id.empty() &&
         finite(limits.preferred_curvature_per_m) &&
         limits.preferred_curvature_per_m > 0.0 &&
         finite(limits.maximum_curvature_per_m) &&
         limits.maximum_curvature_per_m >
             limits.preferred_curvature_per_m &&
         finite(limits.curvature_evaluation_spacing_m) &&
         limits.curvature_evaluation_spacing_m > 0.0 &&
         finite(limits.support_evaluation_length_m) &&
         limits.support_evaluation_length_m > 0.0 &&
         finite(limits.medium_support_proxy_range_m) &&
         limits.medium_support_proxy_range_m >= 0.0 &&
         finite(limits.maximum_support_proxy_range_m) &&
         limits.maximum_support_proxy_range_m >
             limits.medium_support_proxy_range_m &&
         finite(limits.minimum_terrain_confidence) &&
         limits.minimum_terrain_confidence > 0.0 &&
         limits.minimum_terrain_confidence <= 1.0 &&
         finite(limits.minimum_distinct_touchdown_distance_m) &&
         limits.minimum_distinct_touchdown_distance_m > 0.0 &&
         limits.curvature_evaluation_spacing_m >
             limits.minimum_distinct_touchdown_distance_m &&
         finite(limits.bend_weight) && limits.bend_weight >= 0.0 &&
         finite(limits.terrain_risk_weight) &&
         limits.terrain_risk_weight >= 0.0 && finite(limits.roughness_weight) &&
         limits.roughness_weight >= 0.0;
}

bool valid_memory(const CableConstraintMemory& memory,
                  const double minimum_distance_m) {
  if (!finite(memory.retained_arc_length_m) ||
      memory.retained_arc_length_m < 0.0 ||
      memory.previous_distinct_touchdown_points_m.size() > 2U) {
    return false;
  }
  for (std::size_t index = 0;
       index < memory.previous_distinct_touchdown_points_m.size(); ++index) {
    if (!finite(memory.previous_distinct_touchdown_points_m[index]) ||
        (index > 0U &&
         distance(memory.previous_distinct_touchdown_points_m[index - 1U],
                  memory.previous_distinct_touchdown_points_m[index]) <
             minimum_distance_m)) {
      return false;
    }
  }
  for (std::size_t index = 0; index < memory.trailing_support_samples.size();
       ++index) {
    const CableHistorySample& sample = memory.trailing_support_samples[index];
    if (!finite(sample.touchdown_arc_length_m) ||
        !finite(sample.touchdown_position_m) ||
        (index > 0U &&
         sample.touchdown_arc_length_m <=
             memory.trailing_support_samples[index - 1U]
                 .touchdown_arc_length_m) ||
        (index > 0U &&
         std::abs(sample.touchdown_arc_length_m -
                      memory.trailing_support_samples[index - 1U]
                          .touchdown_arc_length_m -
                  distance(memory.trailing_support_samples[index - 1U]
                               .touchdown_position_m,
                           sample.touchdown_position_m)) >
             kArcLengthToleranceM)) {
      return false;
    }
  }
  if (memory.trailing_support_samples.empty()) {
    return memory.retained_arc_length_m == 0.0;
  }
  const double retained_m =
      memory.trailing_support_samples.back().touchdown_arc_length_m -
      memory.trailing_support_samples.front().touchdown_arc_length_m;
  return std::abs(retained_m - memory.retained_arc_length_m) <= 1.0e-9;
}

bool complete_for_boundary(const CableConstraintMemory& memory,
                           const CableLayingLimits& limits,
                           const CableHistoryBoundary history_boundary) {
  return history_boundary == CableHistoryBoundary::explicit_task_start ||
         (memory.previous_distinct_touchdown_points_m.size() >= 2U &&
          memory.trailing_support_samples.size() >= 2U &&
          memory.retained_arc_length_m + kGeometryToleranceM >=
              std::max(limits.support_evaluation_length_m,
                       2.0 * limits.curvature_evaluation_spacing_m));
}

bool has_supported_linear_interpolation(const GeometricPath& path) {
  if (path.metadata.interpolation_rule != "linear" &&
      path.metadata.interpolation_rule != "cable-mean-spatial-lag") {
    return false;
  }
  for (std::size_t index = 1U; index < path.points.size(); ++index) {
    const double arc_span_m = path.points[index].arc_length_m -
                              path.points[index - 1U].arc_length_m;
    const double chord_m =
        distance(point(path.points[index - 1U]), point(path.points[index]));
    if (chord_m > kGeometryToleranceM &&
        std::abs(arc_span_m - chord_m) > kArcLengthToleranceM) {
      return false;
    }
  }
  return true;
}

void add_failure(CableLayingEvaluation& result,
                 const CableLayingFailure reason,
                 const double start_arc_length_m,
                 const double end_arc_length_m,
                 const Vector2m representative_position_m) {
  if (std::find(result.failure_reasons.begin(), result.failure_reasons.end(),
                reason) == result.failure_reasons.end()) {
    result.failure_reasons.push_back(reason);
  }
  const bool duplicate_segment =
      !result.failure_segments.empty() &&
      result.failure_segments.back().reason == reason &&
      result.failure_segments.back().start_arc_length_m == start_arc_length_m &&
      result.failure_segments.back().end_arc_length_m == end_arc_length_m;
  if (!duplicate_segment) {
    result.failure_segments.push_back(
        {reason, start_arc_length_m, end_arc_length_m,
         representative_position_m});
  }
  result.hard_feasible = false;
}

double signed_curvature(const Vector2m first, const Vector2m middle,
                        const Vector2m last) {
  const double first_middle = distance(first, middle);
  const double middle_last = distance(middle, last);
  const double first_last = distance(first, last);
  if (first_middle <= kGeometryToleranceM ||
      middle_last <= kGeometryToleranceM ||
      first_last <= kGeometryToleranceM) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double cross =
      (middle.x_m - first.x_m) * (last.y_m - first.y_m) -
      (middle.y_m - first.y_m) * (last.x_m - first.x_m);
  return 2.0 * cross / (first_middle * middle_last * first_last);
}

struct CurvaturePoint {
  Vector2m position_m;
  double arc_length_m{};
};

bool is_redundant_collinear_point(const CurvaturePoint& first,
                                  const CurvaturePoint& middle,
                                  const CurvaturePoint& last) {
  const double first_middle = distance(first.position_m, middle.position_m);
  const double middle_last = distance(middle.position_m, last.position_m);
  const double first_last = distance(first.position_m, last.position_m);
  return std::abs(first_middle + middle_last - first_last) <=
         kArcLengthToleranceM * std::max(1.0, first_middle + middle_last);
}

void remove_redundant_collinear_points(std::vector<CurvaturePoint>& points) {
  std::vector<CurvaturePoint> canonical;
  canonical.reserve(points.size());
  for (const CurvaturePoint& value : points) {
    canonical.push_back(value);
    while (canonical.size() >= 3U &&
           is_redundant_collinear_point(
               canonical[canonical.size() - 3U],
               canonical[canonical.size() - 2U], canonical.back())) {
      canonical.erase(canonical.end() - 2);
    }
  }
  points = std::move(canonical);
}

Vector2m interpolate_curvature_position(
    const std::vector<CurvaturePoint>& points, const double arc_length_m) {
  const auto right = std::lower_bound(
      points.begin(), points.end(), arc_length_m,
      [](const CurvaturePoint& value, const double target) {
        return value.arc_length_m < target;
      });
  if (right == points.begin()) return points.front().position_m;
  if (right == points.end()) return points.back().position_m;
  const CurvaturePoint& first = *(right - 1);
  const double ratio =
      (arc_length_m - first.arc_length_m) /
      (right->arc_length_m - first.arc_length_m);
  return {first.position_m.x_m +
              ratio * (right->position_m.x_m - first.position_m.x_m),
          first.position_m.y_m +
              ratio * (right->position_m.y_m - first.position_m.y_m)};
}

double evaluate_curvature(CableLayingEvaluation& result,
                          const CableConstraintMemory& initial_memory,
                          const GeometricPath& path,
                          const CableLayingLimits& limits,
                          const bool incremental_segment) {
  for (std::size_t index = 1U; index < path.points.size(); ++index) {
    if (distance(point(path.points[index - 1U]), point(path.points[index])) <
        limits.minimum_distinct_touchdown_distance_m) {
      add_failure(result, CableLayingFailure::duplicate_touchdown_point,
                  path.points[index - 1U].arc_length_m,
                  path.points[index].arc_length_m, point(path.points[index]));
      result.valid = false;
    }
  }
  if (!result.valid) return 0.0;

  std::vector<CurvaturePoint> points;
  points.reserve(initial_memory.trailing_support_samples.size() +
                 path.points.size());
  double global_arc_length_m = 0.0;
  for (const CableHistorySample& history_sample :
       initial_memory.trailing_support_samples) {
    const Vector2m history_point = history_sample.touchdown_position_m;
    if (!points.empty()) {
      global_arc_length_m += distance(points.back().position_m, history_point);
    }
    points.push_back({history_point, global_arc_length_m});
  }
  double candidate_start_global_arc_length_m{};
  for (std::size_t path_index = 0; path_index < path.points.size();
       ++path_index) {
    const Vector2m position_m = point(path.points[path_index]);
    const bool shared_history_boundary =
        path_index == 0U && !points.empty() &&
        distance(points.back().position_m, position_m) <
            limits.minimum_distinct_touchdown_distance_m;
    if (shared_history_boundary) {
      candidate_start_global_arc_length_m = points.back().arc_length_m;
      continue;
    }
    if (!points.empty()) {
      global_arc_length_m +=
          path_index == 0U
              ? distance(points.back().position_m, position_m)
              : path.points[path_index].arc_length_m -
                    path.points[path_index - 1U].arc_length_m;
    }
    if (path_index == 0U) {
      candidate_start_global_arc_length_m = global_arc_length_m;
    }
    points.push_back({position_m, global_arc_length_m});
  }
  remove_redundant_collinear_points(points);

  double bend_cost = 0.0;
  for (std::size_t index = 1U; index + 1U < points.size(); ++index) {
    const double available_right_span_m =
        points.back().arc_length_m - points[index].arc_length_m;
    if (points[index].arc_length_m + limits.curvature_evaluation_spacing_m <=
        candidate_start_global_arc_length_m + kGeometryToleranceM) {
      continue;
    }
    if (incremental_segment &&
        available_right_span_m + kGeometryToleranceM <
            limits.curvature_evaluation_spacing_m) {
      continue;
    }
    const double left_arc_length_m =
        std::max(points.front().arc_length_m,
                 points[index].arc_length_m -
                     limits.curvature_evaluation_spacing_m);
    const double right_arc_length_m =
        std::min(points.back().arc_length_m,
                 points[index].arc_length_m +
                     limits.curvature_evaluation_spacing_m);
    const Vector2m left =
        interpolate_curvature_position(points, left_arc_length_m);
    const Vector2m right =
        interpolate_curvature_position(points, right_arc_length_m);
    const double curvature_per_m =
        signed_curvature(left, points[index].position_m, right);
    const double center_arc_length_m = std::clamp(
        path.points.front().arc_length_m + points[index].arc_length_m -
            candidate_start_global_arc_length_m,
        path.points.front().arc_length_m, path.points.back().arc_length_m);
    const double start_arc_length_m =
        std::max(path.points.front().arc_length_m,
                 center_arc_length_m - limits.curvature_evaluation_spacing_m);
    const double end_arc_length_m =
        std::min(path.points.back().arc_length_m,
                 center_arc_length_m + limits.curvature_evaluation_spacing_m);
    if (!finite(curvature_per_m)) {
      add_failure(result, CableLayingFailure::duplicate_touchdown_point,
                  start_arc_length_m, end_arc_length_m,
                  points[index].position_m);
      result.valid = false;
      continue;
    }
    const double absolute_curvature_per_m = std::abs(curvature_per_m);
    if (!result.maximum_absolute_curvature_position_m.has_value() ||
        absolute_curvature_per_m >
            result.maximum_absolute_curvature_per_m) {
      result.maximum_absolute_curvature_per_m = absolute_curvature_per_m;
      result.maximum_absolute_curvature_position_m =
          points[index].position_m;
    }
    if (absolute_curvature_per_m > limits.maximum_curvature_per_m) {
      add_failure(result, CableLayingFailure::curvature_exceeded,
                  start_arc_length_m, end_arc_length_m,
                  points[index].position_m);
    }
    if (absolute_curvature_per_m > limits.preferred_curvature_per_m) {
      const double excess =
          absolute_curvature_per_m - limits.preferred_curvature_per_m;
      const double curvature_coverage_m =
          right_arc_length_m - left_arc_length_m;
      bend_cost += limits.bend_weight * excess * excess * curvature_coverage_m;
    }
  }
  return bend_cost;
}

struct TerrainQuery {
  bool inside{};
  const SurfaceEstimate* surface{};
  const CableLayingTerrainCell* cable{};
};

TerrainQuery query_terrain(const TerrainLayers& terrain,
                           const Vector2m position_m) {
  const double column_value =
      (position_m.x_m - terrain.surface.origin_x_m) /
      terrain.surface.resolution_m;
  const double row_value =
      (position_m.y_m - terrain.surface.origin_y_m) /
      terrain.surface.resolution_m;
  if (!finite(column_value) || !finite(row_value) || column_value < 0.0 ||
      row_value < 0.0 ||
      column_value >= static_cast<double>(terrain.surface.width) ||
      row_value >= static_cast<double>(terrain.surface.height)) {
    return {};
  }
  const auto column = static_cast<std::size_t>(std::floor(column_value));
  const auto row = static_cast<std::size_t>(std::floor(row_value));
  return {true, &terrain.surface.at(row, column),
          &terrain.cable_laying.at(row, column, terrain.surface)};
}

struct GridCellIndex {
  std::size_t row{};
  std::size_t column{};
};

bool near_integer(const double value) {
  return std::abs(value - std::round(value)) <= kGeometryToleranceM;
}

std::vector<GridCellIndex> grid_supercover(const TerrainLayers& terrain,
                                           const Vector2m first,
                                           const Vector2m last) {
  const double x0 =
      (first.x_m - terrain.surface.origin_x_m) / terrain.surface.resolution_m;
  const double y0 =
      (first.y_m - terrain.surface.origin_y_m) / terrain.surface.resolution_m;
  const double x1 =
      (last.x_m - terrain.surface.origin_x_m) / terrain.surface.resolution_m;
  const double y1 =
      (last.y_m - terrain.surface.origin_y_m) / terrain.surface.resolution_m;
  const auto inside = [&terrain](const double x, const double y) {
    return x >= 0.0 && y >= 0.0 &&
           x <= static_cast<double>(terrain.surface.width) &&
           y <= static_cast<double>(terrain.surface.height);
  };
  if (!finite(x0) || !finite(y0) || !finite(x1) || !finite(y1) ||
      !inside(x0, y0) || !inside(x1, y1)) {
    return {};
  }

  std::vector<GridCellIndex> cells;
  std::set<std::pair<std::int64_t, std::int64_t>> visited;
  const bool vertical_boundary = x0 == x1 && near_integer(x0);
  const bool horizontal_boundary = y0 == y1 && near_integer(y0);
  const auto add = [&](const std::int64_t row, const std::int64_t column) {
    const auto width = static_cast<std::int64_t>(terrain.surface.width);
    const auto height = static_cast<std::int64_t>(terrain.surface.height);
    if (row < 0 || column < 0 || row >= height || column >= width) return;
    if (visited.emplace(row, column).second) {
      cells.push_back({static_cast<std::size_t>(row),
                       static_cast<std::size_t>(column)});
    }
  };
  const auto add_current = [&](const std::int64_t row,
                               const std::int64_t column) {
    add(row, column);
    if (vertical_boundary) add(row, column - 1);
    if (horizontal_boundary) add(row - 1, column);
    if (vertical_boundary && horizontal_boundary) add(row - 1, column - 1);
  };
  const auto add_touching_point = [&](const double x, const double y) {
    const auto column = static_cast<std::int64_t>(std::floor(x));
    const auto row = static_cast<std::int64_t>(std::floor(y));
    add(row, column);
    if (near_integer(x)) add(row, column - 1);
    if (near_integer(y)) add(row - 1, column);
    if (near_integer(x) && near_integer(y)) add(row - 1, column - 1);
  };

  std::int64_t column = static_cast<std::int64_t>(std::floor(x0));
  std::int64_t row = static_cast<std::int64_t>(std::floor(y0));
  add_touching_point(x0, y0);
  add_current(row, column);
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const int step_x = dx > 0.0 ? 1 : (dx < 0.0 ? -1 : 0);
  const int step_y = dy > 0.0 ? 1 : (dy < 0.0 ? -1 : 0);
  const double infinity = std::numeric_limits<double>::infinity();
  const double next_x = step_x > 0 ? std::floor(x0) + 1.0
                                   : std::ceil(x0) - 1.0;
  const double next_y = step_y > 0 ? std::floor(y0) + 1.0
                                   : std::ceil(y0) - 1.0;
  double t_max_x = step_x == 0 ? infinity : (next_x - x0) / dx;
  double t_max_y = step_y == 0 ? infinity : (next_y - y0) / dy;
  const double t_delta_x = step_x == 0 ? infinity : 1.0 / std::abs(dx);
  const double t_delta_y = step_y == 0 ? infinity : 1.0 / std::abs(dy);
  const std::size_t maximum_steps =
      terrain.surface.width + terrain.surface.height + 4U;
  for (std::size_t step = 0U; step < maximum_steps; ++step) {
    const double next_t = std::min(t_max_x, t_max_y);
    if (next_t > 1.0 + kGeometryToleranceM) break;
    if (std::abs(t_max_x - t_max_y) <= kGeometryToleranceM) {
      add(row, column + step_x);
      add(row + step_y, column);
      column += step_x;
      row += step_y;
      t_max_x += t_delta_x;
      t_max_y += t_delta_y;
    } else if (t_max_x < t_max_y) {
      column += step_x;
      t_max_x += t_delta_x;
    } else {
      row += step_y;
      t_max_y += t_delta_y;
    }
    add_current(row, column);
  }
  add_touching_point(x1, y1);
  return cells;
}

struct SweepControlPoint {
  double global_arc_length_m{};
  double local_arc_length_m{};
  Vector2m position_m;
  bool candidate{};
};

struct SweepSample : SweepControlPoint {
  double segment_start_arc_length_m{};
  double segment_end_arc_length_m{};
  double elevation_m{};
  double roughness_m{};
  bool terrain_valid{};
};

std::vector<SweepControlPoint> make_control_points(
    const CableConstraintMemory& memory, const GeometricPath& path,
    const CableLayingLimits& limits) {
  std::vector<SweepControlPoint> controls;
  controls.reserve(memory.trailing_support_samples.size() + path.points.size());
  for (const CableHistorySample& sample : memory.trailing_support_samples) {
    controls.push_back({sample.touchdown_arc_length_m, 0.0,
                        sample.touchdown_position_m, false});
  }

  double global_arc_length_m = controls.empty()
                                   ? 0.0
                                   : controls.back().global_arc_length_m;
  const bool shared_boundary =
      !controls.empty() &&
      distance(controls.back().position_m, point(path.points.front())) <
          limits.minimum_distinct_touchdown_distance_m;
  if (!controls.empty() && !shared_boundary) {
    global_arc_length_m +=
        distance(controls.back().position_m, point(path.points.front()));
  }
  controls.push_back({global_arc_length_m, path.points.front().arc_length_m,
                      point(path.points.front()), true});
  for (std::size_t index = 1U; index < path.points.size(); ++index) {
    global_arc_length_m += path.points[index].arc_length_m -
                           path.points[index - 1U].arc_length_m;
    controls.push_back({global_arc_length_m, path.points[index].arc_length_m,
                        point(path.points[index]), true});
  }
  return controls;
}

bool valid_cable_terrain_cell(const SurfaceEstimate& surface,
                              const CableLayingTerrainCell& cable,
                              const CableLayingLimits& limits) {
  return surface.status == TerrainEstimateStatus::valid && cable.known &&
         finite(cable.confidence) &&
         cable.confidence >= limits.minimum_terrain_confidence &&
         finite(cable.elevation_m) && finite(cable.roughness_m) &&
         cable.roughness_m >= 0.0;
}

void evaluate_categorical_sweep(
    CableLayingEvaluation& result,
    const std::vector<SweepControlPoint>& controls,
    const TerrainLayers& terrain, const CableLayingLimits& limits,
    const double minimum_global_arc_length_m) {
  for (std::size_t index = 1U; index < controls.size(); ++index) {
    const SweepControlPoint& first = controls[index - 1U];
    const SweepControlPoint& last = controls[index];
    const double span_m = last.global_arc_length_m - first.global_arc_length_m;
    if (!(span_m > 0.0) ||
        last.global_arc_length_m + kGeometryToleranceM <
            minimum_global_arc_length_m) {
      continue;
    }
    const double clipped_ratio =
        std::clamp((minimum_global_arc_length_m - first.global_arc_length_m) /
                       span_m,
                   0.0, 1.0);
    const Vector2m clipped_first{
        first.position_m.x_m +
            clipped_ratio * (last.position_m.x_m - first.position_m.x_m),
        first.position_m.y_m +
            clipped_ratio * (last.position_m.y_m - first.position_m.y_m)};
    const std::vector<GridCellIndex> cells =
        grid_supercover(terrain, clipped_first, last.position_m);
    for (const GridCellIndex cell : cells) {
      const SurfaceEstimate& surface = terrain.surface.at(cell.row, cell.column);
      const CableLayingTerrainCell& cable =
          terrain.cable_laying.at(cell.row, cell.column, terrain.surface);
      const Vector2m representative{
          terrain.surface.origin_x_m +
              (static_cast<double>(cell.column) + 0.5) *
                  terrain.surface.resolution_m,
          terrain.surface.origin_y_m +
              (static_cast<double>(cell.row) + 0.5) *
                  terrain.surface.resolution_m};
      if (!valid_cable_terrain_cell(surface, cable, limits)) {
        add_failure(result, CableLayingFailure::terrain_data_invalid,
                    last.candidate ? first.local_arc_length_m : 0.0,
                    last.candidate ? last.local_arc_length_m : 0.0,
                    representative);
      } else if (cable.forbidden || cable.obstacle) {
        add_failure(result,
                    CableLayingFailure::forbidden_area_intersection,
                    last.candidate ? first.local_arc_length_m : 0.0,
                    last.candidate ? last.local_arc_length_m : 0.0,
                    representative);
      }
    }
  }
}

std::vector<SweepSample> sample_terrain(
    CableLayingEvaluation& result,
    const std::vector<SweepControlPoint>& controls,
    const TerrainLayers& terrain, const CableLayingLimits& limits,
    const double minimum_global_arc_length_m) {
  std::vector<SweepSample> samples;
  const double maximum_step_m = terrain.surface.resolution_m * 0.5;
  for (std::size_t segment_index = 1U; segment_index < controls.size();
       ++segment_index) {
    const SweepControlPoint& first = controls[segment_index - 1U];
    const SweepControlPoint& last = controls[segment_index];
    const double arc_span_m =
        last.global_arc_length_m - first.global_arc_length_m;
    if (last.global_arc_length_m + kGeometryToleranceM <
        minimum_global_arc_length_m) {
      continue;
    }
    if (!(arc_span_m > 0.0)) {
      if (first.candidate != last.candidate &&
          distance(first.position_m, last.position_m) <
              limits.minimum_distinct_touchdown_distance_m) {
        if (!samples.empty()) {
          samples.back().candidate = last.candidate;
          samples.back().local_arc_length_m = last.local_arc_length_m;
        }
        continue;
      }
      result.valid = false;
      add_failure(result, CableLayingFailure::numerically_invalid,
                  last.local_arc_length_m, last.local_arc_length_m,
                  last.position_m);
      continue;
    }
    const double interval_count_value =
        std::max(1.0, std::ceil(arc_span_m / maximum_step_m));
    if (!finite(interval_count_value) ||
        interval_count_value >
            static_cast<double>(kMaximumSweepIntervalsPerSegment)) {
      result.valid = false;
      add_failure(result, CableLayingFailure::numerically_invalid,
                  last.local_arc_length_m, last.local_arc_length_m,
                  last.position_m);
      continue;
    }
    const auto interval_count =
        static_cast<std::size_t>(interval_count_value);
    const double clipped_ratio =
        std::clamp((minimum_global_arc_length_m - first.global_arc_length_m) /
                       arc_span_m,
                   0.0, 1.0);
    std::vector<double> ratios{clipped_ratio};
    const auto first_interval = static_cast<std::size_t>(std::ceil(
        clipped_ratio * static_cast<double>(interval_count) -
        kGeometryToleranceM));
    ratios.reserve(interval_count - first_interval + 2U);
    for (std::size_t interval = first_interval; interval <= interval_count;
         ++interval) {
      const double ratio = static_cast<double>(interval) /
                           static_cast<double>(interval_count);
      if (std::abs(ratio - clipped_ratio) > kGeometryToleranceM)
        ratios.push_back(ratio);
    }
    for (const double ratio : ratios) {
      SweepSample sample;
      sample.global_arc_length_m =
          first.global_arc_length_m + ratio * arc_span_m;
      sample.local_arc_length_m =
          first.local_arc_length_m +
          ratio * (last.local_arc_length_m - first.local_arc_length_m);
      sample.position_m =
          {first.position_m.x_m +
               ratio * (last.position_m.x_m - first.position_m.x_m),
           first.position_m.y_m +
               ratio * (last.position_m.y_m - first.position_m.y_m)};
      sample.candidate =
          last.candidate &&
          (first.candidate || ratio >= 1.0 - kGeometryToleranceM);
      sample.segment_start_arc_length_m =
          last.candidate ? first.local_arc_length_m : 0.0;
      sample.segment_end_arc_length_m =
          last.candidate ? last.local_arc_length_m : 0.0;
      const TerrainQuery query = query_terrain(terrain, sample.position_m);
      sample.terrain_valid =
          query.inside &&
          valid_cable_terrain_cell(*query.surface, *query.cable, limits);
      if (!sample.terrain_valid) {
        add_failure(result, CableLayingFailure::terrain_data_invalid,
                    sample.segment_start_arc_length_m,
                    sample.segment_end_arc_length_m, sample.position_m);
      } else {
        sample.elevation_m = query.cable->elevation_m;
        sample.roughness_m = query.cable->roughness_m;
        if (query.cable->forbidden || query.cable->obstacle) {
          add_failure(result,
                      CableLayingFailure::forbidden_area_intersection,
                      sample.segment_start_arc_length_m,
                      sample.segment_end_arc_length_m, sample.position_m);
        }
      }
      samples.push_back(sample);
    }
  }
  return samples;
}

double evaluate_support_proxy(CableLayingEvaluation& result,
                              const std::vector<SweepSample>& samples,
                              const CableLayingLimits& limits) {
  double soft_cost = 0.0;
  std::optional<double> previous_candidate_arc_length_m;
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const SweepSample& current = samples[index];
    if (!current.candidate) continue;
    const double cutoff_m =
        current.global_arc_length_m - limits.support_evaluation_length_m;
    double minimum_elevation_m = std::numeric_limits<double>::infinity();
    double maximum_elevation_m = -std::numeric_limits<double>::infinity();
    for (std::size_t window_index = 0U; window_index <= index;
         ++window_index) {
      const SweepSample& window_sample = samples[window_index];
      if (window_sample.global_arc_length_m + kGeometryToleranceM < cutoff_m ||
          !window_sample.terrain_valid) {
        continue;
      }
      minimum_elevation_m =
          std::min(minimum_elevation_m, window_sample.elevation_m);
      maximum_elevation_m =
          std::max(maximum_elevation_m, window_sample.elevation_m);
    }
    if (!finite(minimum_elevation_m) || !finite(maximum_elevation_m)) {
      continue;
    }
    const double support_range_m =
        maximum_elevation_m - minimum_elevation_m;
    if (!result.maximum_support_proxy_position_m.has_value() ||
        support_range_m > result.maximum_support_proxy_range_m) {
      result.maximum_support_proxy_range_m = support_range_m;
      result.maximum_support_proxy_position_m = current.position_m;
    }
    if (support_range_m > limits.maximum_support_proxy_range_m) {
      add_failure(result, CableLayingFailure::support_proxy_exceeded,
                  current.segment_start_arc_length_m,
                  current.segment_end_arc_length_m, current.position_m);
    }
    if (previous_candidate_arc_length_m.has_value()) {
      const double integration_length_m =
          current.global_arc_length_m - *previous_candidate_arc_length_m;
      if (support_range_m > limits.medium_support_proxy_range_m &&
          support_range_m <= limits.maximum_support_proxy_range_m) {
        soft_cost += limits.terrain_risk_weight * 1.5 * integration_length_m;
      }
      soft_cost += limits.roughness_weight * current.roughness_m *
                   integration_length_m;
    }
    previous_candidate_arc_length_m = current.global_arc_length_m;
  }
  return soft_cost;
}

std::uint64_t mix(std::uint64_t hash, const std::uint64_t value) {
  for (unsigned int shift = 0; shift < 64U; shift += 8U) {
    hash ^= (value >> shift) & 0xffU;
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::uint64_t bits(const double value) {
  std::uint64_t result{};
  static_assert(sizeof(result) == sizeof(value), "double must be 64 bits");
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

void update_signature(CableConstraintMemory& memory) {
  std::uint64_t signature = 14695981039346656037ULL;
  signature = mix(signature,
                  memory.previous_distinct_touchdown_points_m.size());
  for (const Vector2m history_point :
       memory.previous_distinct_touchdown_points_m) {
    signature = mix(signature, bits(history_point.x_m));
    signature = mix(signature, bits(history_point.y_m));
  }
  signature = mix(signature, memory.trailing_support_samples.size());
  for (const CableHistorySample& sample : memory.trailing_support_samples) {
    signature = mix(signature, bits(sample.touchdown_arc_length_m));
    signature = mix(signature, bits(sample.touchdown_position_m.x_m));
    signature = mix(signature, bits(sample.touchdown_position_m.y_m));
  }
  memory.canonical_signature =
      mix(signature, bits(memory.retained_arc_length_m));
}

void canonicalize_signed_zeros(CableConstraintMemory& memory) {
  const auto canonicalize = [](double& value) {
    if (value == 0.0) value = 0.0;
  };
  for (Vector2m& point : memory.previous_distinct_touchdown_points_m) {
    canonicalize(point.x_m);
    canonicalize(point.y_m);
  }
  for (CableHistorySample& sample : memory.trailing_support_samples) {
    canonicalize(sample.touchdown_arc_length_m);
    canonicalize(sample.touchdown_position_m.x_m);
    canonicalize(sample.touchdown_position_m.y_m);
  }
  canonicalize(memory.retained_arc_length_m);
}

void normalize_memory(CableConstraintMemory& memory,
                      const CableLayingLimits& limits) {
  if (memory.trailing_support_samples.empty()) {
    memory.retained_arc_length_m = 0.0;
    memory.canonical_signature = 0U;
    return;
  }
  const double retained_history_length_m =
      std::max(limits.support_evaluation_length_m,
               2.0 * limits.curvature_evaluation_spacing_m);
  const double cutoff_m =
      memory.trailing_support_samples.back().touchdown_arc_length_m -
      retained_history_length_m;
  while (memory.trailing_support_samples.size() > 1U &&
         memory.trailing_support_samples[1U].touchdown_arc_length_m <=
             cutoff_m) {
    memory.trailing_support_samples.erase(
        memory.trailing_support_samples.begin());
  }
  if (memory.trailing_support_samples.size() > 1U &&
      memory.trailing_support_samples.front().touchdown_arc_length_m <
          cutoff_m) {
    CableHistorySample& first = memory.trailing_support_samples.front();
    const CableHistorySample& second = memory.trailing_support_samples[1U];
    const double ratio =
        (cutoff_m - first.touchdown_arc_length_m) /
        (second.touchdown_arc_length_m - first.touchdown_arc_length_m);
    first.touchdown_position_m = {
        first.touchdown_position_m.x_m +
            ratio * (second.touchdown_position_m.x_m -
                     first.touchdown_position_m.x_m),
        first.touchdown_position_m.y_m +
            ratio * (second.touchdown_position_m.y_m -
                     first.touchdown_position_m.y_m)};
    first.touchdown_arc_length_m = cutoff_m;
  }
  memory.retained_arc_length_m =
      memory.trailing_support_samples.back().touchdown_arc_length_m -
      memory.trailing_support_samples.front().touchdown_arc_length_m;
  canonicalize_signed_zeros(memory);
  update_signature(memory);
}

CableConstraintMemory advance_memory(
    CableConstraintMemory memory, const GeometricPath& path,
    const CableLayingLimits& limits) {
  for (std::size_t index = 0U; index < path.points.size(); ++index) {
    const PathPoint& path_point = path.points[index];
    const Vector2m position_m = point(path_point);
    if (memory.trailing_support_samples.empty()) {
      memory.trailing_support_samples.push_back({0.0, position_m});
    } else {
      const Vector2m previous =
          memory.trailing_support_samples.back().touchdown_position_m;
      const double step_m =
          index == 0U
              ? distance(previous, position_m)
              : path.points[index].arc_length_m -
                    path.points[index - 1U].arc_length_m;
      if (step_m >= limits.minimum_distinct_touchdown_distance_m) {
        memory.trailing_support_samples.push_back(
            {memory.trailing_support_samples.back().touchdown_arc_length_m +
                 step_m,
             position_m});
      }
    }
    if (memory.previous_distinct_touchdown_points_m.empty() ||
        distance(memory.previous_distinct_touchdown_points_m.back(),
                 position_m) >=
            limits.minimum_distinct_touchdown_distance_m) {
      memory.previous_distinct_touchdown_points_m.push_back(position_m);
      if (memory.previous_distinct_touchdown_points_m.size() > 2U) {
        memory.previous_distinct_touchdown_points_m.erase(
            memory.previous_distinct_touchdown_points_m.begin());
      }
    }
  }
  normalize_memory(memory, limits);
  return memory;
}

bool same_memory(const CableConstraintMemory& left,
                 const CableConstraintMemory& right) {
  if (left.retained_arc_length_m != right.retained_arc_length_m ||
      left.previous_distinct_touchdown_points_m.size() !=
          right.previous_distinct_touchdown_points_m.size() ||
      left.trailing_support_samples.size() !=
          right.trailing_support_samples.size()) {
    return false;
  }
  for (std::size_t index = 0;
       index < left.previous_distinct_touchdown_points_m.size(); ++index) {
    const Vector2m a = left.previous_distinct_touchdown_points_m[index];
    const Vector2m b = right.previous_distinct_touchdown_points_m[index];
    if (a.x_m != b.x_m || a.y_m != b.y_m) return false;
  }
  for (std::size_t index = 0; index < left.trailing_support_samples.size();
       ++index) {
    const CableHistorySample& a = left.trailing_support_samples[index];
    const CableHistorySample& b = right.trailing_support_samples[index];
    if (a.touchdown_arc_length_m != b.touchdown_arc_length_m ||
        a.touchdown_position_m.x_m != b.touchdown_position_m.x_m ||
        a.touchdown_position_m.y_m != b.touchdown_position_m.y_m) {
      return false;
    }
  }
  return true;
}

CableLayingEvaluation evaluate_impl(
    const CableConstraintMemory& initial_memory,
    const GeometricPath& touchdown_path,
    const std::vector<CableState>& state_profile,
    const TerrainLayers& terrain,
    const CableLayingLimits& limits,
    const CableHistoryBoundary history_boundary,
    const bool incremental_segment) {
  CableLayingEvaluation result;
  result.terminal_memory = initial_memory;
  result.limits_version = limits.version;
  result.terrain_map_sequence = terrain.source_map_version.sequence_number;
  result.terrain_analysis_config_version = terrain.analysis_config_version;
  result.operating_domain_id = terrain.operating_domain_id;
  result.risk_semantics =
      "CONSERVATIVE_SUPPORT_PROXY:NO_FLEXIBLE_CABLE_DYNAMICS_GUARANTEE";
  if (!valid_limits(limits) || !validate(touchdown_path).valid ||
      !has_supported_linear_interpolation(touchdown_path) ||
      state_profile.size() != touchdown_path.points.size() ||
      terrain.source_map_version.sequence_number == 0U ||
      terrain.analysis_config_version == 0U ||
      terrain.source_map_version.timestamp.nanoseconds < 0 ||
      terrain.operating_domain_id.empty() || terrain.surface.width == 0U ||
      terrain.surface.height == 0U || !finite(terrain.surface.resolution_m) ||
      terrain.surface.resolution_m <= 0.0 ||
      terrain.surface.cells.size() !=
          terrain.surface.width * terrain.surface.height ||
      terrain.cable_laying.cells.size() != terrain.surface.cells.size() ||
      touchdown_path.metadata.coordinate_frame !=
          terrain.source_map_version.coordinate_frame ||
      terrain.operating_domain_id != limits.operating_domain_id ||
      !valid_memory(initial_memory,
                    limits.minimum_distinct_touchdown_distance_m)) {
    result.failure_reasons = {CableLayingFailure::numerically_invalid};
    return result;
  }
  for (const CableState& state : state_profile) {
    if (!validate(state).valid) {
      result.failure_reasons = {CableLayingFailure::numerically_invalid};
      return result;
    }
  }
  if (!complete_for_boundary(initial_memory, limits, history_boundary)) {
    result.failure_reasons = {
        CableLayingFailure::mechanical_history_incomplete};
    return result;
  }

  result.valid = true;
  result.hard_feasible = true;
  double soft_cost =
      evaluate_curvature(result, initial_memory, touchdown_path, limits,
                         incremental_segment);
  if (!result.valid) return result;

  const std::vector<SweepControlPoint> controls =
      make_control_points(initial_memory, touchdown_path, limits);
  const auto first_candidate =
      std::find_if(controls.begin(), controls.end(),
                   [](const SweepControlPoint& value) {
                     return value.candidate;
                   });
  const double minimum_global_arc_length_m =
      std::max(controls.front().global_arc_length_m,
               first_candidate->global_arc_length_m -
                   limits.support_evaluation_length_m);
  evaluate_categorical_sweep(result, controls, terrain, limits,
                             minimum_global_arc_length_m);
  const std::vector<SweepSample> samples =
      sample_terrain(result, controls, terrain, limits,
                     minimum_global_arc_length_m);
  soft_cost += evaluate_support_proxy(result, samples, limits);
  result.terminal_support_window_length_m =
      history_boundary == CableHistoryBoundary::actual_laying_history
          ? limits.support_evaluation_length_m
          : std::min(limits.support_evaluation_length_m,
                     controls.back().global_arc_length_m -
                         controls.front().global_arc_length_m);

  if (result.valid && result.hard_feasible) {
    result.failure_reasons = {CableLayingFailure::none};
    result.soft_cost = soft_cost;
    result.terminal_memory =
        advance_memory(initial_memory, touchdown_path, limits);
  } else {
    result.soft_cost = 0.0;
  }
  return result;
}

}  // namespace

std::optional<CableConstraintMemory> CableLayingEvaluator::canonicalize_memory(
    const CableConstraintMemory& memory,
    const CableLayingLimits& limits,
    const CableHistoryBoundary history_boundary) const {
  if (!valid_limits(limits) ||
      !valid_memory(memory, limits.minimum_distinct_touchdown_distance_m) ||
      !complete_for_boundary(memory, limits, history_boundary)) {
    return std::nullopt;
  }
  CableConstraintMemory canonical = memory;
  normalize_memory(canonical, limits);
  return canonical;
}

CableLayingEvaluation CableLayingEvaluator::evaluate_segment(
    const CableConstraintMemory& initial_memory,
    const GeometricPath& touchdown_segment,
    const std::vector<CableState>& state_profile,
    const TerrainLayers& terrain,
    const CableLayingLimits& limits,
    const CableHistoryBoundary history_boundary) const {
  return evaluate_impl(initial_memory, touchdown_segment, state_profile,
                       terrain, limits, history_boundary, true);
}

CableLayingEvaluation CableLayingEvaluator::evaluate(
    const CableConstraintMemory& initial_memory,
    const GeometricPath& touchdown_path,
    const std::vector<CableState>& state_profile,
    const TerrainLayers& terrain,
    const CableLayingLimits& limits,
    const CableHistoryBoundary history_boundary) const {
  return evaluate_impl(initial_memory, touchdown_path, state_profile, terrain,
                       limits, history_boundary, false);
}

bool CableLayingEvaluator::future_equivalent(
    const CableConstraintMemory& left,
    const CableConstraintMemory& right) const noexcept {
  return same_memory(left, right);
}

}  // namespace underwater_planner::core
