#include "step_geometry.hpp"

#include "terrain_analysis_primitives.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace underwater_planner::core::detail {
namespace {

enum class CandidateKind {
  observed_jump,
  unknown_boundary,
};

struct CandidatePoint {
  Point2d point;
  Vector2d low_to_high;
  double crossing_width_m{};
  CandidateKind kind{CandidateKind::observed_jump};
};

struct GridOffset {
  std::size_t row{};
  std::size_t column{};
};

struct PlaneSample {
  double dx_m{};
  double dy_m{};
  double elevation_m{};
  double weight{};
  double confidence{};
};

struct PlaneFit {
  std::array<double, 3> coefficients{};
  double intercept_variance_m2{};
  double mean_confidence{};
  bool valid{};
};

PlaneFit fit_plane(const std::vector<PlaneSample>& samples) {
  PlaneFit fit;
  if (samples.size() < 3) return fit;
  Matrix3 normal{};
  Vector3 right_hand_side{};
  double confidence_sum = 0.0;
  for (const PlaneSample& sample : samples) {
    const Vector3 design{sample.dx_m, sample.dy_m, 1.0};
    for (std::size_t row = 0; row < 3; ++row) {
      right_hand_side[row] +=
          sample.weight * design[row] * sample.elevation_m;
      for (std::size_t column = 0; column < 3; ++column) {
        normal[row][column] +=
            sample.weight * design[row] * design[column];
      }
    }
    confidence_sum += sample.confidence;
  }
  fit.valid =
      solve_3x3_linear_system(normal, right_hand_side, fit.coefficients);
  if (!fit.valid) return fit;
  Vector3 intercept_inverse_column{};
  if (!solve_3x3_linear_system(normal, {0.0, 0.0, 1.0},
                               intercept_inverse_column) ||
      !finite(intercept_inverse_column[2]) ||
      intercept_inverse_column[2] < 0.0) {
    fit.valid = false;
    return fit;
  }
  const double sample_count = static_cast<double>(samples.size());
  fit.intercept_variance_m2 = intercept_inverse_column[2];
  fit.mean_confidence = confidence_sum / sample_count;
  return fit;
}

double median_filtered_elevation(const MapSnapshot& map, const std::size_t row,
                                 const std::size_t column) {
  const MapCell& center = map.at(row, column);
  if (!usable(center, map.version.timestamp)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::vector<double> neighborhood;
  for (std::int64_t row_offset = -1; row_offset <= 1; ++row_offset) {
    for (std::int64_t column_offset = -1; column_offset <= 1;
         ++column_offset) {
      const std::int64_t sample_row =
          static_cast<std::int64_t>(row) + row_offset;
      const std::int64_t sample_column =
          static_cast<std::int64_t>(column) + column_offset;
      if (sample_row < 0 || sample_column < 0 ||
          sample_row >= static_cast<std::int64_t>(map.height) ||
          sample_column >= static_cast<std::int64_t>(map.width)) {
        continue;
      }
      const MapCell& sample =
          map.at(static_cast<std::size_t>(sample_row),
                 static_cast<std::size_t>(sample_column));
      if (usable(sample, map.version.timestamp)) {
        neighborhood.push_back(sample.elevation_m);
      }
    }
  }
  std::sort(neighborhood.begin(), neighborhood.end());
  return neighborhood[neighborhood.size() / 2];
}

std::vector<double> median_filtered_elevations(const MapSnapshot& map) {
  std::vector<double> filtered(map.cells.size());
  for (std::size_t row = 0; row < map.height; ++row) {
    for (std::size_t column = 0; column < map.width; ++column) {
      filtered[row * map.width + column] =
          median_filtered_elevation(map, row, column);
    }
  }
  return filtered;
}

bool point_in_regions(const Point2d point,
                      const std::vector<MapUpdateRegion>& regions,
                      const double margin_m) {
  return std::any_of(
      regions.begin(), regions.end(), [point, margin_m](const auto& region) {
        return point.x_m >= region.min_x_m - margin_m &&
               point.x_m <= region.max_x_m + margin_m &&
               point.y_m >= region.min_y_m - margin_m &&
               point.y_m <= region.max_y_m + margin_m;
      });
}

std::vector<CandidatePoint> find_candidates(const MapSnapshot& map,
                                             const TerrainAnalysisConfig& config,
                                             const std::vector<MapUpdateRegion>*
                                                 regions = nullptr) {
  const std::optional<std::vector<double>> elevation =
      regions == nullptr
          ? std::optional<std::vector<double>>{
                median_filtered_elevations(map)}
          : std::nullopt;
  const auto filtered_elevation = [&](const std::size_t row,
                                      const std::size_t column) {
    return elevation.has_value()
               ? elevation->at(row * map.width + column)
               : median_filtered_elevation(map, row, column);
  };
  std::vector<CandidatePoint> candidates;
  const double candidate_threshold_m = config.minimum_step_height_m * 0.5;
  const std::array<GridOffset, 2> neighbors{{{0, 1}, {1, 0}}};
  for (std::size_t row = 0; row < map.height; ++row) {
    for (std::size_t column = 0; column < map.width; ++column) {
      for (const auto& neighbor : neighbors) {
        const std::size_t next_row = row + neighbor.row;
        const std::size_t next_column = column + neighbor.column;
        if (next_row >= map.height || next_column >= map.width) continue;
        const Point2d midpoint{
            map.origin_x_m +
                (static_cast<double>(column + next_column) * 0.5) *
                    map.resolution_m,
            map.origin_y_m +
                (static_cast<double>(row + next_row) * 0.5) *
                    map.resolution_m};
        if (regions != nullptr &&
            !point_in_regions(midpoint, *regions, 2.0 * map.resolution_m)) {
          continue;
        }
        const double first = filtered_elevation(row, column);
        const double second = filtered_elevation(next_row, next_column);
        const bool first_known = finite(first);
        const bool second_known = finite(second);
        if (!first_known && !second_known) {
          continue;
        }
        Vector2d direction{static_cast<double>(neighbor.column),
                           static_cast<double>(neighbor.row)};
        if (first_known != second_known) {
          if (!first_known) {
            direction.x = -direction.x;
            direction.y = -direction.y;
          }
          candidates.push_back({midpoint, direction, map.resolution_m,
                                CandidateKind::unknown_boundary});
          continue;
        }
        if (std::abs(second - first) < candidate_threshold_m) continue;
        if (second < first) {
          direction.x = -direction.x;
          direction.y = -direction.y;
        }
        candidates.push_back({midpoint, direction, map.resolution_m,
                              CandidateKind::observed_jump});
      }
    }
  }
  return candidates;
}

bool same_candidates(const std::vector<CandidatePoint>& left,
                     const std::vector<CandidatePoint>& right) {
  if (left.size() != right.size()) return false;
  for (std::size_t index = 0; index < left.size(); ++index) {
    const CandidatePoint& left_candidate = left[index];
    const CandidatePoint& right_candidate = right[index];
    if (left_candidate.point.x_m != right_candidate.point.x_m ||
        left_candidate.point.y_m != right_candidate.point.y_m ||
        left_candidate.low_to_high.x != right_candidate.low_to_high.x ||
        left_candidate.low_to_high.y != right_candidate.low_to_high.y ||
        left_candidate.crossing_width_m !=
            right_candidate.crossing_width_m ||
        left_candidate.kind != right_candidate.kind) {
      return false;
    }
  }
  return true;
}

std::vector<std::vector<std::size_t>> connected_components(
    const std::vector<CandidatePoint>& candidates, const double resolution_m) {
  std::vector<std::vector<std::size_t>> components;
  std::vector<bool> assigned(candidates.size(), false);
  const double connection_distance_m = 1.6 * resolution_m;
  for (std::size_t seed = 0; seed < candidates.size(); ++seed) {
    if (assigned[seed]) continue;
    assigned[seed] = true;
    components.push_back({seed});
    for (std::size_t cursor = 0; cursor < components.back().size(); ++cursor) {
      const std::size_t current = components.back()[cursor];
      for (std::size_t candidate = 0; candidate < candidates.size();
           ++candidate) {
        if (assigned[candidate] ||
            candidates[candidate].kind != candidates[current].kind) {
          continue;
        }
        const double distance_m = std::hypot(
            candidates[current].point.x_m - candidates[candidate].point.x_m,
            candidates[current].point.y_m - candidates[candidate].point.y_m);
        if (distance_m <= connection_distance_m + 1.0e-12) {
          assigned[candidate] = true;
          components.back().push_back(candidate);
        }
      }
    }
  }
  return components;
}

double projection(const Point2d& point, const Point2d& origin,
                  const Vector2d& axis) {
  return (point.x_m - origin.x_m) * axis.x +
         (point.y_m - origin.y_m) * axis.y;
}

StepEstimate estimate_component(const MapSnapshot& map,
                                const TerrainAnalysisConfig& config,
                                const std::vector<CandidatePoint>& candidates,
                                const std::vector<std::size_t>& component) {
  StepEstimate estimate;
  Point2d center{};
  Vector2d normal_sum{};
  double transition_width_sum_m = 0.0;
  for (const std::size_t index : component) {
    center.x_m += candidates[index].point.x_m;
    center.y_m += candidates[index].point.y_m;
    normal_sum.x += candidates[index].low_to_high.x;
    normal_sum.y += candidates[index].low_to_high.y;
  }
  const double count = static_cast<double>(component.size());
  center.x_m /= count;
  center.y_m /= count;
  const double normal_sum_length = std::hypot(normal_sum.x, normal_sum.y);
  const double normal_consistency = normal_sum_length / count;
  if (normal_sum_length <= 0.0 ||
      normal_consistency < config.minimum_step_normal_consistency) {
    estimate.status = StepEstimateStatus::unstable_normal;
    return estimate;
  }
  const Vector2d normal{normal_sum.x / normal_sum_length,
                        normal_sum.y / normal_sum_length};
  const Vector2d tangent{-normal.y, normal.x};
  double minimum_normal = std::numeric_limits<double>::infinity();
  double maximum_normal = -std::numeric_limits<double>::infinity();
  for (const std::size_t index : component) {
    const Vector2d& crossing = candidates[index].low_to_high;
    transition_width_sum_m +=
        candidates[index].crossing_width_m *
        std::abs(crossing.x * normal.x + crossing.y * normal.y);
    const double normal_coordinate =
        projection(candidates[index].point, center, normal);
    minimum_normal = std::min(minimum_normal, normal_coordinate);
    maximum_normal = std::max(maximum_normal, normal_coordinate);
  }
  double minimum_tangent = std::numeric_limits<double>::infinity();
  double maximum_tangent = -std::numeric_limits<double>::infinity();
  for (const std::size_t index : component) {
    const double coordinate =
        projection(candidates[index].point, center, tangent);
    minimum_tangent = std::min(minimum_tangent, coordinate);
    maximum_tangent = std::max(maximum_tangent, coordinate);
  }
  if (component.size() < 2 || maximum_tangent == minimum_tangent) {
    estimate.status = StepEstimateStatus::duplicate_extent_point;
    return estimate;
  }
  const double extent_m = maximum_tangent - minimum_tangent;
  estimate.edge.extent = {
      {center.x_m + tangent.x * minimum_tangent,
       center.y_m + tangent.y * minimum_tangent},
      {center.x_m + tangent.x * maximum_tangent,
       center.y_m + tangent.y * maximum_tangent}};
  estimate.edge.normal_low_to_high = normal;
  estimate.edge.transition_width_m =
      maximum_normal - minimum_normal + transition_width_sum_m / count;
  if (extent_m < config.minimum_step_extent_m) {
    estimate.status = StepEstimateStatus::insufficient_extent;
    return estimate;
  }
  if (candidates[component.front()].kind == CandidateKind::unknown_boundary) {
    estimate.edge.normal_low_to_high = {};
    estimate.status = StepEstimateStatus::insufficient_side_support;
    return estimate;
  }

  std::vector<PlaneSample> low_samples;
  std::vector<PlaneSample> high_samples;
  std::size_t possible_low = 0;
  std::size_t possible_high = 0;
  for (std::size_t row = 0; row < map.height; ++row) {
    for (std::size_t column = 0; column < map.width; ++column) {
      const Point2d point{
          map.origin_x_m + static_cast<double>(column) * map.resolution_m,
          map.origin_y_m + static_cast<double>(row) * map.resolution_m};
      const double normal_coordinate = projection(point, center, normal);
      const double tangent_coordinate = projection(point, center, tangent);
      if (std::abs(normal_coordinate) > config.step_support_band_width_m ||
          tangent_coordinate < minimum_tangent - map.resolution_m ||
          tangent_coordinate > maximum_tangent + map.resolution_m) {
        continue;
      }
      if (normal_coordinate >= minimum_normal &&
          normal_coordinate <= maximum_normal) {
        continue;
      }
      const bool high_side = normal_coordinate > maximum_normal;
      if (high_side) {
        ++possible_high;
      } else {
        ++possible_low;
      }
      const MapCell& cell = map.at(row, column);
      if (!usable(cell, map.version.timestamp)) continue;
      const double variance = std::max(cell.elevation_variance_m2,
                                       config.minimum_elevation_variance_m2);
      PlaneSample sample{point.x_m - center.x_m, point.y_m - center.y_m,
                         cell.elevation_m, cell.confidence / variance,
                         cell.confidence};
      (high_side ? high_samples : low_samples).push_back(sample);
    }
  }
  const double low_support_ratio =
      possible_low == 0 ? 0.0
                        : static_cast<double>(low_samples.size()) /
                              static_cast<double>(possible_low);
  const double high_support_ratio =
      possible_high == 0 ? 0.0
                         : static_cast<double>(high_samples.size()) /
                               static_cast<double>(possible_high);
  if (low_support_ratio < config.minimum_step_side_support_ratio ||
      high_support_ratio < config.minimum_step_side_support_ratio) {
    estimate.status = StepEstimateStatus::insufficient_side_support;
    return estimate;
  }
  const PlaneFit low = fit_plane(low_samples);
  const PlaneFit high = fit_plane(high_samples);
  if (!low.valid || !high.valid) {
    estimate.status = StepEstimateStatus::insufficient_side_support;
    return estimate;
  }
  const double signed_height_m =
      high.coefficients[2] - low.coefficients[2];
  if (signed_height_m < 0.0) {
    estimate.edge.normal_low_to_high.x = -normal.x;
    estimate.edge.normal_low_to_high.y = -normal.y;
  }
  estimate.edge.height_m = std::abs(signed_height_m);
  if (estimate.edge.height_m < config.minimum_step_height_m) {
    estimate.status = StepEstimateStatus::below_minimum_height;
    return estimate;
  }
  const double noise_m =
      std::sqrt(low.intercept_variance_m2 + high.intercept_variance_m2);
  const double required_signal_m = config.step_noise_sigma_multiplier * noise_m;
  if (!finite(required_signal_m) ||
      estimate.edge.height_m < required_signal_m) {
    estimate.status = StepEstimateStatus::noise_not_significant;
    return estimate;
  }
  const double signal_confidence =
      required_signal_m > 0.0
          ? std::min(1.0, estimate.edge.height_m / required_signal_m)
          : 1.0;
  const double grid_normal_confidence =
      std::min(1.0, std::sqrt(2.0) * normal_consistency);
  estimate.edge.confidence =
      std::min({low.mean_confidence, high.mean_confidence,
                low_support_ratio, high_support_ratio, signal_confidence,
                grid_normal_confidence});
  if (estimate.edge.confidence < config.minimum_step_confidence) {
    estimate.status = StepEstimateStatus::low_confidence;
    return estimate;
  }
  estimate.status = StepEstimateStatus::valid;
  return estimate;
}

double distance_to_segment(const Point2d& point, const Point2d& start,
                           const Point2d& end) {
  const double dx = end.x_m - start.x_m;
  const double dy = end.y_m - start.y_m;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= 0.0) {
    return std::hypot(point.x_m - start.x_m, point.y_m - start.y_m);
  }
  const double parameter = std::clamp(
      ((point.x_m - start.x_m) * dx + (point.y_m - start.y_m) * dy) /
          length_squared,
      0.0, 1.0);
  return std::hypot(point.x_m - (start.x_m + parameter * dx),
                    point.y_m - (start.y_m + parameter * dy));
}

}  // namespace

StepLayer extract_step_geometry(const MapSnapshot& map,
                                const TerrainAnalysisConfig& config) {
  StepLayer layer;
  const std::vector<CandidatePoint> candidates = find_candidates(map, config);
  for (const std::vector<std::size_t>& component :
       connected_components(candidates, map.resolution_m)) {
    layer.estimates.push_back(
        estimate_component(map, config, candidates, component));
  }
  return layer;
}

bool step_geometry_changed_in_regions(
    const MapSnapshot& previous, const MapSnapshot& current,
    const std::vector<MapUpdateRegion>& regions,
    const TerrainAnalysisConfig& config) {
  return !same_candidates(find_candidates(previous, config, &regions),
                          find_candidates(current, config, &regions));
}

void mark_step_discontinuities(const StepLayer& steps,
                               const TerrainAnalysisConfig& config,
                               const MapSnapshot& map,
                               SurfaceLayer& surface) {
  std::vector<std::size_t> all_cell_indices(map.cells.size());
  for (std::size_t index = 0; index < all_cell_indices.size(); ++index) {
    all_cell_indices[index] = index;
  }
  mark_step_discontinuities(steps, config, map, surface, all_cell_indices);
}

void mark_step_discontinuities(
    const StepLayer& steps, const TerrainAnalysisConfig& config,
    const MapSnapshot& map, SurfaceLayer& surface,
    const std::vector<std::size_t>& affected_cell_indices) {
  const double radius_m = config.surface_window_size_m * 0.5;
  for (const StepEstimate& estimate : steps.estimates) {
    if (estimate.status != StepEstimateStatus::valid ||
        estimate.edge.extent.size() != 2) {
      continue;
    }
    for (const std::size_t index : affected_cell_indices) {
      const std::size_t row = index / map.width;
      const std::size_t column = index % map.width;
      const Point2d point{
          map.origin_x_m + static_cast<double>(column) * map.resolution_m,
          map.origin_y_m + static_cast<double>(row) * map.resolution_m};
      if (distance_to_segment(point, estimate.edge.extent.front(),
                              estimate.edge.extent.back()) <=
          radius_m + 1.0e-12) {
        SurfaceEstimate& surface_estimate = surface.cells.at(index);
        const MapCell& map_cell = map.at(row, column);
        if (usable(map_cell, map.version.timestamp)) {
          surface_estimate.elevation_m = map_cell.elevation_m;
          surface_estimate.status = TerrainEstimateStatus::discontinuous;
        }
      }
    }
  }
}

}  // namespace underwater_planner::core::detail
