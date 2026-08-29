#include "underwater_planner/core/terrain_analyzer.hpp"

#include "step_geometry.hpp"
#include "terrain_analysis_primitives.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace underwater_planner::core {
namespace {

using detail::finite;
using detail::solve_3x3_linear_system;
using detail::usable;

bool same_cell(const MapCell& left, const MapCell& right) {
  const bool same_normal =
      left.obstacle_normal.has_value() == right.obstacle_normal.has_value() &&
      (!left.obstacle_normal.has_value() ||
       (left.obstacle_normal->x == right.obstacle_normal->x &&
        left.obstacle_normal->y == right.obstacle_normal->y));
  return left.elevation_m == right.elevation_m &&
         left.elevation_variance_m2 == right.elevation_variance_m2 &&
         left.confidence == right.confidence && left.known == right.known &&
         left.measurement_timestamp.nanoseconds ==
             right.measurement_timestamp.nanoseconds &&
         left.obstacle == right.obstacle && same_normal &&
         left.cable_forbidden == right.cable_forbidden;
}

bool same_region(const MapUpdateRegion& left, const MapUpdateRegion& right) {
  return left.min_x_m == right.min_x_m && left.min_y_m == right.min_y_m &&
         left.max_x_m == right.max_x_m && left.max_y_m == right.max_y_m;
}

bool same_map_payload(const MapSnapshot& left, const MapSnapshot& right) {
  if (left.version != right.version || left.width != right.width ||
      left.height != right.height || left.resolution_m != right.resolution_m ||
      left.origin_x_m != right.origin_x_m ||
      left.origin_y_m != right.origin_y_m ||
      left.derived_configuration_version !=
          right.derived_configuration_version ||
      left.cells.size() != right.cells.size() ||
      left.update_regions.size() != right.update_regions.size()) {
    return false;
  }
  for (std::size_t index = 0; index < left.cells.size(); ++index) {
    if (!same_cell(left.cells[index], right.cells[index])) return false;
  }
  for (std::size_t index = 0; index < left.update_regions.size(); ++index) {
    if (!same_region(left.update_regions[index], right.update_regions[index])) {
      return false;
    }
  }
  return true;
}

bool same_grid(const MapSnapshot& left, const MapSnapshot& right) {
  return left.version.map_id == right.version.map_id &&
         left.version.coordinate_frame == right.version.coordinate_frame &&
         left.width == right.width && left.height == right.height &&
         left.resolution_m == right.resolution_m &&
         left.origin_x_m == right.origin_x_m &&
         left.origin_y_m == right.origin_y_m;
}

bool same_step_availability(const MapCell& current,
                            const MonotonicTime current_time,
                            const MapCell& previous,
                            const MonotonicTime previous_time) {
  const bool current_usable = usable(current, current_time);
  const bool previous_usable = usable(previous, previous_time);
  return current_usable == previous_usable;
}

bool region_contains(const MapUpdateRegion& region, const double x_m,
                     const double y_m) {
  constexpr double tolerance = 1.0e-12;
  return x_m >= region.min_x_m - tolerance &&
         x_m <= region.max_x_m + tolerance &&
         y_m >= region.min_y_m - tolerance &&
         y_m <= region.max_y_m + tolerance;
}

bool step_support_intersects_regions(
    const StepLayer& steps, const std::vector<MapUpdateRegion>& regions,
    const double support_margin_m) {
  for (const StepEstimate& estimate : steps.estimates) {
    if (estimate.edge.extent.size() != 2U) continue;
    const Point2d& first = estimate.edge.extent.front();
    const Point2d& last = estimate.edge.extent.back();
    const double step_min_x = std::min(first.x_m, last.x_m) - support_margin_m;
    const double step_max_x = std::max(first.x_m, last.x_m) + support_margin_m;
    const double step_min_y = std::min(first.y_m, last.y_m) - support_margin_m;
    const double step_max_y = std::max(first.y_m, last.y_m) + support_margin_m;
    for (const MapUpdateRegion& region : regions) {
      if (region.max_x_m >= step_min_x && region.min_x_m <= step_max_x &&
          region.max_y_m >= step_min_y && region.min_y_m <= step_max_y) {
        return true;
      }
    }
  }
  return false;
}

struct Sample {
  double dx_m{};
  double dy_m{};
  double elevation_m{};
  double weight{};
  MonotonicTime measurement_timestamp;
};

using Matrix3 = std::array<std::array<double, 3>, 3>;
using Vector3 = std::array<double, 3>;

struct NormalEquations {
  Matrix3 matrix;
  Vector3 right_hand_side;
};

double huber_multiplier(const Sample& sample, const Vector3& coefficients,
                        const double huber_delta_m) {
  const double residual =
      sample.elevation_m -
      (coefficients[0] * sample.dx_m + coefficients[1] * sample.dy_m +
       coefficients[2]);
  const double absolute_residual = std::abs(residual);
  return absolute_residual > huber_delta_m
             ? huber_delta_m / absolute_residual
             : 1.0;
}

NormalEquations build_normal_equations(
    const std::vector<Sample>& samples, const Vector3* coefficients,
    const double huber_delta_m) {
  NormalEquations equations{};
  for (const Sample& sample : samples) {
    const Vector3 design{sample.dx_m, sample.dy_m, 1.0};
    const double robust_weight =
        coefficients == nullptr
            ? 1.0
            : huber_multiplier(sample, *coefficients, huber_delta_m);
    const double weight = sample.weight * robust_weight;
    for (std::size_t row = 0; row < 3; ++row) {
      equations.right_hand_side[row] +=
          weight * design[row] * sample.elevation_m;
      for (std::size_t column = 0; column < 3; ++column) {
        equations.matrix[row][column] +=
            weight * design[row] * design[column];
      }
    }
  }
  return equations;
}

SurfaceEstimate fit_surface(const MapSnapshot& map,
                            const TerrainAnalysisConfig& config,
                            const std::size_t center_row,
                            const std::size_t center_column) {
  SurfaceEstimate estimate;
  const double radius_m = config.surface_window_size_m * 0.5;
  const auto cell_radius = static_cast<std::size_t>(
      std::ceil(radius_m / map.resolution_m));
  std::vector<Sample> samples;
  std::size_t expected_support = 0;

  for (std::int64_t row_offset = -static_cast<std::int64_t>(cell_radius);
       row_offset <= static_cast<std::int64_t>(cell_radius); ++row_offset) {
    for (std::int64_t column_offset = -static_cast<std::int64_t>(cell_radius);
         column_offset <= static_cast<std::int64_t>(cell_radius);
         ++column_offset) {
      const double dx_m = static_cast<double>(column_offset) * map.resolution_m;
      const double dy_m = static_cast<double>(row_offset) * map.resolution_m;
      const double distance_m = std::hypot(dx_m, dy_m);
      if (distance_m > radius_m + 1.0e-12) continue;
      ++expected_support;
      const std::int64_t row = static_cast<std::int64_t>(center_row) + row_offset;
      const std::int64_t column =
          static_cast<std::int64_t>(center_column) + column_offset;
      if (row < 0 || column < 0 || row >= static_cast<std::int64_t>(map.height) ||
          column >= static_cast<std::int64_t>(map.width)) {
        continue;
      }
      const MapCell& cell = map.at(static_cast<std::size_t>(row),
                                   static_cast<std::size_t>(column));
      if (!usable(cell, map.version.timestamp)) {
        continue;
      }
      const double distance_weight =
          std::exp(-0.5 * distance_m * distance_m / (radius_m * radius_m));
      const double variance = std::max(cell.elevation_variance_m2,
                                       config.minimum_elevation_variance_m2);
      samples.push_back(
          {dx_m, dy_m, cell.elevation_m,
           cell.confidence * distance_weight / variance,
           cell.measurement_timestamp});
    }
  }

  const auto newest_sample = std::max_element(
      samples.begin(), samples.end(), [](const Sample& left,
                                         const Sample& right) {
        return left.measurement_timestamp.nanoseconds <
               right.measurement_timestamp.nanoseconds;
      });
  if (newest_sample != samples.end()) {
    const std::int64_t newest_timestamp_ns =
        newest_sample->measurement_timestamp.nanoseconds;
    for (Sample& sample : samples) {
      const double age_s = static_cast<double>(
                               newest_timestamp_ns -
                               sample.measurement_timestamp.nanoseconds) /
                           1.0e9;
      sample.weight *= std::exp(-std::log(2.0) * age_s /
                                config.temporal_weight_half_life_s);
    }
  }

  estimate.support_ratio = expected_support == 0
                               ? 0.0
                               : static_cast<double>(samples.size()) /
                                     static_cast<double>(expected_support);
  if (!usable(map.at(center_row, center_column), map.version.timestamp) ||
      samples.size() < 3 ||
      estimate.support_ratio < config.minimum_fit_support_ratio) {
    return estimate;
  }

  Vector3 coefficients{};
  Matrix3 final_normal{};
  bool converged = false;
  for (std::size_t iteration = 0;
       iteration < config.maximum_irls_iterations; ++iteration) {
    const NormalEquations equations = build_normal_equations(
        samples, iteration == 0 ? nullptr : &coefficients,
        config.huber_delta_m);
    Vector3 next{};
    if (!solve_3x3_linear_system(equations.matrix,
                                 equations.right_hand_side, next)) {
      estimate.status = TerrainEstimateStatus::ill_conditioned;
      return estimate;
    }
    const double maximum_change =
        std::max({std::abs(next[0] - coefficients[0]),
                  std::abs(next[1] - coefficients[1]),
                  std::abs(next[2] - coefficients[2])});
    coefficients = next;
    if (iteration > 0 && maximum_change <= 1.0e-12) {
      converged = true;
      break;
    }
  }
  if (!converged) {
    estimate.status = TerrainEstimateStatus::invalid_covariance;
    return estimate;
  }

  final_normal =
      build_normal_equations(samples, &coefficients, config.huber_delta_m)
          .matrix;

  Vector3 inverse_column_x{};
  Vector3 inverse_column_y{};
  if (!solve_3x3_linear_system(final_normal, {1.0, 0.0, 0.0},
                               inverse_column_x) ||
      !solve_3x3_linear_system(final_normal, {0.0, 1.0, 0.0},
                               inverse_column_y)) {
    estimate.status = TerrainEstimateStatus::invalid_covariance;
    return estimate;
  }
  const double covariance_xy =
      0.5 * (inverse_column_x[1] + inverse_column_y[0]);
  estimate.gradient_covariance = {inverse_column_x[0], covariance_xy,
                                  covariance_xy, inverse_column_y[1]};
  const GradientCovariance& covariance = estimate.gradient_covariance;
  const double determinant =
      covariance.xx * covariance.yy - covariance.xy * covariance.yx;
  const double determinant_scale =
      std::max({std::abs(covariance.xx * covariance.yy),
                std::abs(covariance.xy * covariance.yx),
                std::numeric_limits<double>::min()});
  const double determinant_tolerance =
      64.0 * std::numeric_limits<double>::epsilon() * determinant_scale;
  if (!finite(covariance.xx) || !finite(covariance.xy) ||
      !finite(covariance.yx) || !finite(covariance.yy) ||
      covariance.xx < 0.0 || covariance.yy < 0.0 ||
      determinant < -determinant_tolerance) {
    estimate.gradient_covariance = {};
    estimate.status = TerrainEstimateStatus::invalid_covariance;
    return estimate;
  }

  double weighted_squared_residual = 0.0;
  double weight_sum = 0.0;
  std::vector<double> absolute_residuals;
  absolute_residuals.reserve(samples.size());
  for (const Sample& sample : samples) {
    const double residual =
        sample.elevation_m -
        (coefficients[0] * sample.dx_m + coefficients[1] * sample.dy_m +
         coefficients[2]);
    weighted_squared_residual += sample.weight * residual * residual;
    weight_sum += sample.weight;
    absolute_residuals.push_back(std::abs(residual));
  }
  std::sort(absolute_residuals.begin(), absolute_residuals.end());
  const std::size_t p95_index = static_cast<std::size_t>(
      std::ceil(0.95 * static_cast<double>(absolute_residuals.size())) - 1.0);
  estimate.gradient_x = coefficients[0];
  estimate.gradient_y = coefficients[1];
  estimate.elevation_m = coefficients[2];
  estimate.slope_angle_rad =
      std::atan(std::hypot(coefficients[0], coefficients[1]));
  estimate.detrended_roughness_rms_m =
      std::sqrt(weighted_squared_residual / weight_sum);
  estimate.residual_p95_m = absolute_residuals.at(p95_index);
  estimate.status = TerrainEstimateStatus::valid;
  return estimate;
}

void validate_config(const TerrainAnalysisConfig& config) {
  if (config.config_version == 0 || config.operating_domain_id.empty() ||
      !finite(config.surface_window_size_m) ||
      config.surface_window_size_m <= 0.0 ||
      !finite(config.minimum_fit_support_ratio) ||
      config.minimum_fit_support_ratio <= 0.0 ||
      config.minimum_fit_support_ratio > 1.0 || !finite(config.huber_delta_m) ||
      config.huber_delta_m <= 0.0 ||
      !finite(config.minimum_elevation_variance_m2) ||
      config.minimum_elevation_variance_m2 <= 0.0 ||
      !finite(config.temporal_weight_half_life_s) ||
      config.temporal_weight_half_life_s <= 0.0 ||
      config.maximum_irls_iterations < 2 ||
      !finite(config.minimum_step_height_m) ||
      config.minimum_step_height_m <= 0.0 ||
      !finite(config.step_support_band_width_m) ||
      config.step_support_band_width_m <= 0.0 ||
      !finite(config.minimum_step_side_support_ratio) ||
      config.minimum_step_side_support_ratio <= 0.0 ||
      config.minimum_step_side_support_ratio > 1.0 ||
      !finite(config.minimum_step_extent_m) ||
      config.minimum_step_extent_m <= 0.0 ||
      !finite(config.step_noise_sigma_multiplier) ||
      config.step_noise_sigma_multiplier <= 0.0 ||
      !finite(config.minimum_step_confidence) ||
      config.minimum_step_confidence <= 0.0 ||
      config.minimum_step_confidence > 1.0 ||
      !finite(config.minimum_step_normal_consistency) ||
      config.minimum_step_normal_consistency <= 0.0 ||
      config.minimum_step_normal_consistency > 1.0) {
    throw std::invalid_argument("terrain analysis configuration is invalid");
  }
}

void validate_analysis_request(const MapSnapshot& height_map,
                               const TerrainAnalysisConfig& config) {
  validate_config(config);
  if (!validate(height_map).valid ||
      height_map.version.timestamp.nanoseconds < 0) {
    throw std::invalid_argument("terrain height map is invalid");
  }
  if (height_map.derived_configuration_version != config.config_version) {
    throw std::invalid_argument(
        "terrain analysis configuration does not match the map-derived version");
  }
}

}  // namespace

const SurfaceEstimate& SurfaceLayer::at(const std::size_t row,
                                        const std::size_t column) const {
  if (row >= height || column >= width || cells.size() != width * height) {
    throw std::out_of_range("terrain surface cell is outside the layer");
  }
  return cells.at(row * width + column);
}

const CableLayingTerrainCell& CableLayingTerrainLayer::at(
    const std::size_t row, const std::size_t column,
    const SurfaceLayer& surface) const {
  if (row >= surface.height || column >= surface.width ||
      cells.size() != surface.width * surface.height) {
    throw std::out_of_range("cable laying cell is outside the layer");
  }
  return cells.at(row * surface.width + column);
}

TerrainLayers TerrainAnalyzer::analyze(
    const MapSnapshot& height_map,
    const TerrainAnalysisConfig& config) const {
  validate_analysis_request(height_map, config);
  TerrainLayers layers;
  layers.source_map_version = height_map.version;
  layers.analysis_config_version = config.config_version;
  layers.operating_domain_id = config.operating_domain_id;
  layers.surface_fit_window_size_m = config.surface_window_size_m;
  layers.surface.width = height_map.width;
  layers.surface.height = height_map.height;
  layers.surface.resolution_m = height_map.resolution_m;
  layers.surface.origin_x_m = height_map.origin_x_m;
  layers.surface.origin_y_m = height_map.origin_y_m;
  layers.surface.cells.reserve(height_map.width * height_map.height);
  layers.cable_laying.cells.reserve(height_map.width * height_map.height);
  for (std::size_t row = 0; row < height_map.height; ++row) {
    for (std::size_t column = 0; column < height_map.width; ++column) {
      layers.surface.cells.push_back(
          fit_surface(height_map, config, row, column));
      const MapCell& map_cell = height_map.at(row, column);
      const SurfaceEstimate& surface = layers.surface.cells.back();
      layers.cable_laying.cells.push_back(
          {map_cell.elevation_m, surface.detrended_roughness_rms_m,
           map_cell.known, map_cell.confidence, map_cell.obstacle,
           map_cell.cable_forbidden});
    }
  }
  layers.steps = detail::extract_step_geometry(height_map, config);
  detail::mark_step_discontinuities(layers.steps, config, height_map,
                                    layers.surface);
  return layers;
}

IncrementalTerrainAnalysisResult IncrementalTerrainAnalyzer::analyze(
    const MapSnapshot& height_map, const TerrainAnalysisConfig& config) {
  validate_analysis_request(height_map, config);

  const std::string serialized_config =
      serialize_terrain_analysis_config(config);
  const std::size_t cell_count = height_map.width * height_map.height;
  const auto rebuild = [&](const bool invalidated) {
    auto layers = std::make_shared<TerrainLayers>(
        TerrainAnalyzer{}.analyze(height_map, config));
    cached_map_ = height_map;
    cached_config_ = serialized_config;
    cached_layers_ = layers;
    return IncrementalTerrainAnalysisResult{
        std::move(layers),
        {TerrainAnalysisUpdateMode::full_rebuild, cell_count, 0U,
         invalidated, {}}};
  };

  if (!cached_map_.has_value() || cached_layers_ == nullptr) {
    return rebuild(false);
  }
  const MapSnapshot& previous = *cached_map_;
  if (height_map.version == previous.version) {
    if (serialized_config != cached_config_ ||
        !same_map_payload(height_map, previous)) {
      throw std::invalid_argument(
          "terrain map payload or analysis configuration changed without a new version");
    }
    return {cached_layers_,
            {TerrainAnalysisUpdateMode::cache_hit, 0U, cell_count, false, {}}};
  }
  if (height_map.version.map_id == previous.version.map_id &&
      (height_map.version.sequence_number <=
           previous.version.sequence_number ||
       height_map.version.timestamp.nanoseconds <
           previous.version.timestamp.nanoseconds)) {
    throw std::invalid_argument("terrain map version rolled back");
  }

  const bool cache_compatible =
      same_grid(height_map, previous) &&
      serialized_config == cached_config_ && !height_map.update_regions.empty();
  if (!cache_compatible) return rebuild(true);

  bool step_availability_changed = false;
  bool step_fit_values_changed = false;
  for (std::size_t index = 0; index < cell_count; ++index) {
    const MapCell& current_cell = height_map.cells[index];
    const MapCell& previous_cell = previous.cells[index];
    if (!same_step_availability(current_cell, height_map.version.timestamp,
                                previous_cell, previous.version.timestamp)) {
      step_availability_changed = true;
    }
    if (same_cell(current_cell, previous_cell)) continue;
    const std::size_t row = index / height_map.width;
    const std::size_t column = index % height_map.width;
    const double x_m = height_map.origin_x_m +
                       static_cast<double>(column) * height_map.resolution_m;
    const double y_m = height_map.origin_y_m +
                       static_cast<double>(row) * height_map.resolution_m;
    const bool declared = std::any_of(
        height_map.update_regions.begin(), height_map.update_regions.end(),
        [x_m, y_m](const MapUpdateRegion& region) {
          return region_contains(region, x_m, y_m);
        });
    if (!declared) {
      throw std::invalid_argument(
          "terrain map changed outside its declared update regions");
    }
    if ((usable(current_cell, height_map.version.timestamp) ||
         usable(previous_cell, previous.version.timestamp)) &&
        (current_cell.elevation_m != previous_cell.elevation_m ||
         current_cell.elevation_variance_m2 !=
             previous_cell.elevation_variance_m2 ||
         current_cell.confidence != previous_cell.confidence)) {
      step_fit_values_changed = true;
    }
  }
  const bool local_step_candidates_changed =
      detail::step_geometry_changed_in_regions(
          previous, height_map, height_map.update_regions, config);
  const double step_support_margin_m =
      config.step_support_band_width_m + height_map.resolution_m;
  if (step_availability_changed || local_step_candidates_changed ||
      (step_fit_values_changed &&
       step_support_intersects_regions(cached_layers_->steps,
                                       height_map.update_regions,
                                       step_support_margin_m))) {
    return rebuild(true);
  }

  const double radius_m = config.surface_window_size_m * 0.5;
  const double map_max_x_m =
      height_map.origin_x_m +
      static_cast<double>(height_map.width - 1U) * height_map.resolution_m;
  const double map_max_y_m =
      height_map.origin_y_m +
      static_cast<double>(height_map.height - 1U) * height_map.resolution_m;
  std::vector<bool> recompute(cell_count, false);
  std::vector<MapUpdateRegion> expanded_regions;
  expanded_regions.reserve(height_map.update_regions.size());
  for (const MapUpdateRegion& region : height_map.update_regions) {
    const MapUpdateRegion expanded{
        std::max(height_map.origin_x_m, region.min_x_m - radius_m),
        std::max(height_map.origin_y_m, region.min_y_m - radius_m),
        std::min(map_max_x_m, region.max_x_m + radius_m),
        std::min(map_max_y_m, region.max_y_m + radius_m)};
    if (expanded.min_x_m > expanded.max_x_m ||
        expanded.min_y_m > expanded.max_y_m) {
      throw std::invalid_argument(
          "terrain update region does not intersect the map grid");
    }
    expanded_regions.push_back(expanded);
    for (std::size_t row = 0; row < height_map.height; ++row) {
      for (std::size_t column = 0; column < height_map.width; ++column) {
        const double x_m = height_map.origin_x_m +
                           static_cast<double>(column) * height_map.resolution_m;
        const double y_m = height_map.origin_y_m +
                           static_cast<double>(row) * height_map.resolution_m;
        if (region_contains(expanded, x_m, y_m)) {
          recompute[row * height_map.width + column] = true;
        }
      }
    }
  }

  auto layers = std::make_shared<TerrainLayers>(*cached_layers_);
  layers->source_map_version = height_map.version;
  std::size_t recomputed_cell_count = 0U;
  std::vector<std::size_t> recomputed_cell_indices;
  recomputed_cell_indices.reserve(cell_count);
  for (std::size_t row = 0; row < height_map.height; ++row) {
    for (std::size_t column = 0; column < height_map.width; ++column) {
      const std::size_t index = row * height_map.width + column;
      if (!recompute[index]) continue;
      ++recomputed_cell_count;
      recomputed_cell_indices.push_back(index);
      layers->surface.cells[index] = fit_surface(height_map, config, row, column);
      const MapCell& map_cell = height_map.cells[index];
      const SurfaceEstimate& surface = layers->surface.cells[index];
      layers->cable_laying.cells[index] =
          {map_cell.elevation_m, surface.detrended_roughness_rms_m,
           map_cell.known, map_cell.confidence, map_cell.obstacle,
           map_cell.cable_forbidden};
    }
  }

  detail::mark_step_discontinuities(layers->steps, config, height_map,
                                    layers->surface,
                                    recomputed_cell_indices);

  cached_map_ = height_map;
  cached_config_ = serialized_config;
  cached_layers_ = layers;
  return {std::move(layers),
          {TerrainAnalysisUpdateMode::incremental_update,
           recomputed_cell_count, cell_count - recomputed_cell_count, true,
           std::move(expanded_regions)}};
}

std::string serialize_terrain_analysis_config(
    const TerrainAnalysisConfig& config) {
  std::ostringstream output;
  output << std::setprecision(17) << config.config_version << '\n'
         << config.operating_domain_id.size() << ':'
         << config.operating_domain_id << '\n' << config.surface_window_size_m
         << '\n' << config.minimum_fit_support_ratio << '\n'
         << config.huber_delta_m << '\n'
         << config.minimum_elevation_variance_m2 << '\n'
         << config.temporal_weight_half_life_s << '\n'
         << config.maximum_irls_iterations << '\n'
         << config.minimum_step_height_m << '\n'
         << config.step_support_band_width_m << '\n'
         << config.minimum_step_side_support_ratio << '\n'
         << config.minimum_step_extent_m << '\n'
         << config.step_noise_sigma_multiplier << '\n'
         << config.minimum_step_confidence << '\n'
         << config.minimum_step_normal_consistency << '\n';
  return output.str();
}

}  // namespace underwater_planner::core
