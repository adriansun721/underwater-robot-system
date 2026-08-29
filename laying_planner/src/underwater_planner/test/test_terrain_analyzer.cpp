#include "underwater_planner/core/terrain_analyzer.hpp"

#include "terrain_test_config.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

namespace {

void require(const bool condition, const char* message) {
  if (!condition) {
    std::cerr << "T06 failure: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

underwater_planner::core::MapSnapshot make_plane_map(const double gradient_x,
                                                      const double gradient_y) {
  using namespace underwater_planner::core;
  MapSnapshot map;
  map.version = {"terrain-map", 1, {1'000'000'000}, "map-frame"};
  map.width = 9;
  map.height = 9;
  map.resolution_m = 0.25;
  map.origin_x_m = -1.0;
  map.origin_y_m = -1.0;
  map.derived_configuration_version = 6;
  for (std::size_t row = 0; row < map.height; ++row) {
    for (std::size_t column = 0; column < map.width; ++column) {
      const double x_m = map.origin_x_m +
                         static_cast<double>(column) * map.resolution_m;
      const double y_m = map.origin_y_m +
                         static_cast<double>(row) * map.resolution_m;
      map.cells.push_back({gradient_x * x_m + gradient_y * y_m,
                           1.0e-4, 1.0, true,
                           MonotonicTime{900'000'000}});
    }
  }
  return map;
}

underwater_planner::core::TerrainAnalysisConfig make_config() {
  return underwater_planner::testing::make_terrain_analysis_config(
      6, "synthetic-terrain/v1", 1.5, 0.75);
}

void add_checkerboard_roughness(underwater_planner::core::MapSnapshot& map,
                                const double amplitude_m) {
  for (std::size_t row = 0; row < map.height; ++row) {
    for (std::size_t column = 0; column < map.width; ++column) {
      const double sign = (row + column) % 2 == 0 ? 1.0 : -1.0;
      map.cells.at(row * map.width + column).elevation_m += sign * amplitude_m;
    }
  }
}

void multi_direction_plane_has_direction_independent_gradient() {
  // Design: 18.2.1-2
  using namespace underwater_planner::core;
  const TerrainLayers layers =
      TerrainAnalyzer{}.analyze(make_plane_map(0.3, -0.2), make_config());
  const SurfaceEstimate& center = layers.surface.at(4, 4);

  require(layers.source_map_version.map_id == "terrain-map" &&
              layers.source_map_version.sequence_number == 1 &&
              layers.source_map_version.timestamp.nanoseconds ==
                  1'000'000'000 &&
              layers.analysis_config_version == 6 &&
              layers.operating_domain_id == "synthetic-terrain/v1",
          "derived terrain layers lost their source versions or domain");
  require(center.status == TerrainEstimateStatus::valid,
          "an ideal plane did not produce a valid estimate");
  require(std::abs(center.elevation_m) < 1.0e-12 &&
              layers.surface_fit_window_size_m == 1.5,
          "the fitted support elevation or its physical window was omitted");
  require(std::abs(center.gradient_x - 0.3) < 1.0e-9,
          "x gradient changed with plane orientation");
  require(std::abs(center.gradient_y + 0.2) < 1.0e-9,
          "y gradient changed with plane orientation");
  require(std::abs(center.slope_angle_rad -
                   std::atan(std::hypot(0.3, -0.2))) < 1.0e-12,
          "direction-independent slope angle was not reported");
  require(center.detrended_roughness_rms_m < 1.0e-12,
          "an ideal plane was reported as rough");
  require(center.support_ratio == 1.0,
          "a complete physical window did not report full support");
  require(std::isfinite(center.gradient_covariance.xx) &&
              std::isfinite(center.gradient_covariance.xy) &&
              center.gradient_covariance.xy ==
                  center.gradient_covariance.yx &&
              center.gradient_covariance.xx >= 0.0 &&
              center.gradient_covariance.yy >= 0.0 &&
              center.gradient_covariance.xx * center.gradient_covariance.yy -
                      center.gradient_covariance.xy *
                          center.gradient_covariance.yx >=
                  0.0,
          "an ideal multi-direction plane produced invalid covariance");

  const std::array<std::array<double, 2>, 3> other_directions{{
      {{-0.3, 0.2}}, {{0.0, 0.4}}, {{-0.25, 0.0}}}};
  for (const auto& expected : other_directions) {
    const SurfaceEstimate estimate =
        TerrainAnalyzer{}
            .analyze(make_plane_map(expected[0], expected[1]), make_config())
            .surface.at(4, 4);
    require(std::abs(estimate.gradient_x - expected[0]) < 1.0e-9 &&
                std::abs(estimate.gradient_y - expected[1]) < 1.0e-9,
            "a multi-direction plane changed gradient magnitude or direction");
    require(estimate.gradient_covariance.xx ==
                    center.gradient_covariance.xx &&
                estimate.gradient_covariance.xy ==
                    center.gradient_covariance.xy &&
                estimate.gradient_covariance.yy ==
                    center.gradient_covariance.yy,
            "ideal-plane covariance changed with gradient direction");
  }
}

void isolated_outlier_does_not_drag_the_surface_gradient() {
  // Design: 18.2.1-6
  using namespace underwater_planner::core;
  MapSnapshot map = make_plane_map(0.3, -0.2);
  map.cells.at(4 * map.width + 6).elevation_m += 2.0;

  const SurfaceEstimate center =
      TerrainAnalyzer{}.analyze(map, make_config()).surface.at(4, 4);

  require(center.status == TerrainEstimateStatus::valid,
          "an isolated outlier invalidated an otherwise supported fit");
  require(std::abs(center.gradient_x - 0.3) < 0.005 &&
              std::abs(center.gradient_y + 0.2) < 0.005,
          "an isolated outlier dragged the robust plane gradient");
}

void unknown_center_is_not_silently_interpolated() {
  using namespace underwater_planner::core;
  MapSnapshot map = make_plane_map(0.3, -0.2);
  map.cells.at(4 * map.width + 4).known = false;

  const SurfaceEstimate center =
      TerrainAnalyzer{}.analyze(map, make_config()).surface.at(4, 4);

  require(center.status == TerrainEstimateStatus::insufficient_support,
          "an unknown center was silently replaced by an interpolated slope");

  map = make_plane_map(0.3, -0.2);
  map.cells.at(4 * map.width + 4).measurement_timestamp = MonotonicTime{-1};
  const SurfaceEstimate untimed_center =
      TerrainAnalyzer{}.analyze(map, make_config()).surface.at(4, 4);
  require(untimed_center.status == TerrainEstimateStatus::insufficient_support,
          "an untimed center was silently replaced by an interpolated slope");
}

void gradient_covariance_tracks_measurement_uncertainty() {
  using namespace underwater_planner::core;
  const MapSnapshot low_variance_map = make_plane_map(0.3, -0.2);
  MapSnapshot high_variance_map = low_variance_map;
  for (MapCell& cell : high_variance_map.cells) {
    cell.elevation_variance_m2 = 4.0e-4;
  }

  const GradientCovariance low =
      TerrainAnalyzer{}
          .analyze(low_variance_map, make_config())
          .surface.at(4, 4)
          .gradient_covariance;
  const GradientCovariance high =
      TerrainAnalyzer{}
          .analyze(high_variance_map, make_config())
          .surface.at(4, 4)
          .gradient_covariance;

  require(std::isfinite(low.xx) && std::isfinite(low.xy) &&
              std::isfinite(low.yx) && std::isfinite(low.yy),
          "gradient covariance contains a non-finite value");
  require(low.xx > 0.0 && low.yy > 0.0 && low.xy == low.yx &&
              low.xx * low.yy - low.xy * low.yx >= 0.0,
          "gradient covariance is not symmetric positive semidefinite");
  require(high.xx > 3.9 * low.xx && high.yy > 3.9 * low.yy,
          "gradient covariance ignored measurement variance weights");

  MapSnapshot uniformly_uncertain = low_variance_map;
  for (MapCell& cell : uniformly_uncertain.cells) {
    cell.elevation_variance_m2 = 1.0e14;
  }
  const SurfaceEstimate uncertain =
      TerrainAnalyzer{}
          .analyze(uniformly_uncertain, make_config())
          .surface.at(4, 4);
  require(uncertain.status == TerrainEstimateStatus::valid &&
              std::isfinite(uncertain.gradient_covariance.xx) &&
              uncertain.gradient_covariance.xx > high.xx,
          "uniform weight scaling was mistaken for ill-conditioned geometry");
}

void configuration_cannot_disable_robust_reweighting() {
  using namespace underwater_planner::core;
  TerrainAnalysisConfig config = make_config();
  config.maximum_irls_iterations = 1;
  bool rejected = false;
  try {
    static_cast<void>(TerrainAnalyzer{}.analyze(make_plane_map(0.0, 0.0),
                                                config));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected,
          "a one-pass configuration silently disabled robust reweighting");
}

void map_and_analysis_configuration_versions_must_match() {
  using namespace underwater_planner::core;
  TerrainAnalysisConfig config = make_config();
  config.config_version = 7;
  bool rejected = false;
  try {
    static_cast<void>(TerrainAnalyzer{}.analyze(make_plane_map(0.0, 0.0),
                                                config));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected,
          "terrain analysis accepted a mismatched map-derived configuration");
}

void detrended_roughness_is_independent_of_plane_slope() {
  // Design: 18.2.1-1
  // Design: 18.2.1-3
  using namespace underwater_planner::core;
  MapSnapshot flat = make_plane_map(0.0, 0.0);
  MapSnapshot slope = make_plane_map(-0.25, 0.35);
  add_checkerboard_roughness(flat, 0.01);
  add_checkerboard_roughness(slope, 0.01);

  const SurfaceEstimate flat_estimate =
      TerrainAnalyzer{}.analyze(flat, make_config()).surface.at(4, 4);
  const SurfaceEstimate slope_estimate =
      TerrainAnalyzer{}.analyze(slope, make_config()).surface.at(4, 4);

  require(std::abs(flat_estimate.detrended_roughness_rms_m -
                   slope_estimate.detrended_roughness_rms_m) < 1.0e-12,
          "roughness changed when only the plane slope changed");
  require(std::abs(flat_estimate.gradient_x) < 1.0e-12 &&
              std::abs(flat_estimate.gradient_y) < 1.0e-12,
          "ordinary zero-mean elevation noise created a false slope");
  require(flat_estimate.detrended_roughness_rms_m > 0.009 &&
              flat_estimate.detrended_roughness_rms_m < 0.011 &&
              flat_estimate.residual_p95_m > 0.009,
          "RMS or P95 residual did not preserve known surface roughness");
}

void larger_physical_window_does_not_create_false_roughness() {
  using namespace underwater_planner::core;
  const MapSnapshot smooth_slope = make_plane_map(-0.25, 0.35);
  TerrainAnalysisConfig small_window = make_config();
  small_window.surface_window_size_m = 1.0;
  TerrainAnalysisConfig large_window = make_config();
  large_window.surface_window_size_m = 2.0;

  const SurfaceEstimate small =
      TerrainAnalyzer{}
          .analyze(smooth_slope, small_window)
          .surface.at(4, 4);
  const SurfaceEstimate large =
      TerrainAnalyzer{}
          .analyze(smooth_slope, large_window)
          .surface.at(4, 4);
  require(small.status == TerrainEstimateStatus::valid &&
              large.status == TerrainEstimateStatus::valid &&
              small.detrended_roughness_rms_m < 1.0e-12 &&
              large.detrended_roughness_rms_m < 1.0e-12,
          "a larger physical window made a smooth slope appear rough");
}

void unrepresentable_gradient_covariance_is_rejected() {
  // Design: 18.2.1-8
  using namespace underwater_planner::core;
  MapSnapshot map = make_plane_map(0.3, -0.2);
  for (MapCell& cell : map.cells) {
    cell.elevation_variance_m2 = std::numeric_limits<double>::max();
    cell.confidence = 0.1;
  }
  const SurfaceEstimate estimate =
      TerrainAnalyzer{}.analyze(map, make_config()).surface.at(4, 4);
  require(estimate.status == TerrainEstimateStatus::invalid_covariance,
          "an unrepresentable gradient covariance was not rejected");

  map = make_plane_map(0.3, -0.2);
  map.cells.at(4 * map.width + 6).elevation_m += 2.0;
  TerrainAnalysisConfig truncated_irls = make_config();
  truncated_irls.maximum_irls_iterations = 2;
  const SurfaceEstimate unconverged =
      TerrainAnalyzer{}
          .analyze(map, truncated_irls)
          .surface.at(4, 4);
  require(unconverged.status == TerrainEstimateStatus::invalid_covariance,
          "an unconverged robust fit published mismatched covariance");
}

void support_failure_modes_are_distinct() {
  // Design: 18.2.1-7
  using namespace underwater_planner::core;
  TerrainAnalysisConfig config = make_config();
  config.minimum_fit_support_ratio = 0.2;
  MapSnapshot collinear = make_plane_map(0.0, 0.0);
  for (std::size_t row = 0; row < collinear.height; ++row) {
    if (row == 4) continue;
    for (std::size_t column = 0; column < collinear.width; ++column) {
      collinear.cells.at(row * collinear.width + column).known = false;
    }
  }
  const SurfaceEstimate ill_conditioned =
      TerrainAnalyzer{}.analyze(collinear, config).surface.at(4, 4);
  require(ill_conditioned.status == TerrainEstimateStatus::ill_conditioned,
          "collinear support was not classified as ill-conditioned");

  MapSnapshot sparse = collinear;
  for (std::size_t column = 0; column < sparse.width; ++column) {
    sparse.cells.at(4 * sparse.width + column).known = column == 4;
  }
  const SurfaceEstimate insufficient =
      TerrainAnalyzer{}.analyze(sparse, config).surface.at(4, 4);
  require(insufficient.status == TerrainEstimateStatus::insufficient_support &&
              insufficient.support_ratio < config.minimum_fit_support_ratio,
          "sparse support was not classified as insufficient");
}

void stale_outlier_has_less_influence_than_fresh_outlier() {
  using namespace underwater_planner::core;
  TerrainAnalysisConfig config = make_config();
  config.huber_delta_m = 100.0;
  MapSnapshot fresh = make_plane_map(0.3, -0.2);
  fresh.version.timestamp.nanoseconds = 200'000'000'000;
  for (MapCell& cell : fresh.cells) {
    cell.measurement_timestamp.nanoseconds = 199'900'000'000;
  }
  fresh.cells.at(4 * fresh.width + 6).elevation_m += 1.0;
  fresh.cells.at(4 * fresh.width + 6).measurement_timestamp =
      fresh.version.timestamp;
  MapSnapshot stale = fresh;
  stale.cells.at(4 * stale.width + 6).measurement_timestamp.nanoseconds =
      100'000'000'000;

  const SurfaceEstimate fresh_estimate =
      TerrainAnalyzer{}.analyze(fresh, config).surface.at(4, 4);
  const SurfaceEstimate stale_estimate =
      TerrainAnalyzer{}.analyze(stale, config).surface.at(4, 4);
  const double fresh_error = std::abs(fresh_estimate.gradient_x - 0.3);
  const double stale_error = std::abs(stale_estimate.gradient_x - 0.3);
  require(stale_error < 0.01 * fresh_error,
          "measurement age did not reduce stale-sample influence");
}

void repeated_analysis_is_fieldwise_deterministic() {
  using namespace underwater_planner::core;
  MapSnapshot map = make_plane_map(0.3, -0.2);
  add_checkerboard_roughness(map, 0.01);
  const TerrainLayers first = TerrainAnalyzer{}.analyze(map, make_config());
  const TerrainLayers second = TerrainAnalyzer{}.analyze(map, make_config());
  require(first.surface.cells.size() == second.surface.cells.size(),
          "repeated analysis changed layer dimensions");
  for (std::size_t index = 0; index < first.surface.cells.size(); ++index) {
    const SurfaceEstimate& left = first.surface.cells[index];
    const SurfaceEstimate& right = second.surface.cells[index];
    require(left.elevation_m == right.elevation_m &&
                left.gradient_x == right.gradient_x &&
                left.gradient_y == right.gradient_y &&
                left.slope_angle_rad == right.slope_angle_rad &&
                left.gradient_covariance.xx == right.gradient_covariance.xx &&
                left.gradient_covariance.xy == right.gradient_covariance.xy &&
                left.gradient_covariance.yx == right.gradient_covariance.yx &&
                left.gradient_covariance.yy == right.gradient_covariance.yy &&
                left.detrended_roughness_rms_m ==
                    right.detrended_roughness_rms_m &&
                left.residual_p95_m == right.residual_p95_m &&
                left.support_ratio == right.support_ratio &&
                left.status == right.status,
            "repeated analysis changed a surface estimate field");
    const auto& left_cable = first.cable_laying.cells[index];
    const auto& right_cable = second.cable_laying.cells[index];
    require(left_cable.elevation_m == right_cable.elevation_m &&
                left_cable.roughness_m == right_cable.roughness_m &&
                left_cable.known == right_cable.known &&
                left_cable.confidence == right_cable.confidence &&
                left_cable.obstacle == right_cable.obstacle &&
                left_cable.forbidden == right_cable.forbidden,
            "repeated analysis changed cable laying terrain semantics");
  }
}

void cable_forbidden_and_obstacle_cells_reach_the_derived_layer() {
  using namespace underwater_planner::core;
  MapSnapshot map = make_plane_map(0.0, 0.0);
  const std::size_t forbidden_index = 4U * map.width + 4U;
  const std::size_t obstacle_index = 4U * map.width + 5U;
  map.cells[forbidden_index].cable_forbidden = true;
  map.cells[obstacle_index].obstacle = true;
  const TerrainLayers layers = TerrainAnalyzer{}.analyze(map, make_config());
  require(layers.cable_laying.cells[forbidden_index].forbidden &&
              layers.cable_laying.cells[obstacle_index].obstacle,
          "map cable restrictions were dropped from derived terrain");
}

void incremental_updates_expand_the_physical_window_and_share_snapshots() {
  using namespace underwater_planner::core;
  IncrementalTerrainAnalyzer analyzer;
  const MapSnapshot initial_map = make_plane_map(0.3, -0.2);
  const IncrementalTerrainAnalysisResult initial =
      analyzer.analyze(initial_map, make_config());
  require(initial.layers != nullptr &&
              initial.diagnostics.mode ==
                  TerrainAnalysisUpdateMode::full_rebuild &&
              initial.diagnostics.recomputed_cell_count == 81U &&
              initial.diagnostics.reused_cell_count == 0U,
          "the first terrain snapshot was not built and audited in full");

  const IncrementalTerrainAnalysisResult shared =
      analyzer.analyze(initial_map, make_config());
  require(shared.layers == initial.layers &&
              shared.diagnostics.mode ==
                  TerrainAnalysisUpdateMode::cache_hit &&
              shared.diagnostics.recomputed_cell_count == 0U &&
              shared.diagnostics.reused_cell_count == 81U,
          "an identical map version did not share its immutable terrain snapshot");

  MapSnapshot updated = initial_map;
  updated.version.sequence_number = 2U;
  updated.version.timestamp = {2'000'000'000};
  updated.cells.at(4U * updated.width + 4U).obstacle = true;
  updated.update_regions = {{0.0, 0.0, 0.0, 0.0}};
  const IncrementalTerrainAnalysisResult incremental =
      analyzer.analyze(updated, make_config());
  const TerrainLayers expected = TerrainAnalyzer{}.analyze(updated, make_config());
  require(incremental.layers != nullptr &&
              incremental.layers != initial.layers &&
              incremental.diagnostics.mode ==
                  TerrainAnalysisUpdateMode::incremental_update &&
              incremental.diagnostics.recomputed_cell_count == 49U &&
              incremental.diagnostics.reused_cell_count == 32U &&
              incremental.diagnostics.expanded_update_regions.size() == 1U,
          "a one-cell update did not expand by the physical fitting radius");
  for (std::size_t index = 0; index < expected.surface.cells.size(); ++index) {
    const SurfaceEstimate& actual = incremental.layers->surface.cells[index];
    const SurfaceEstimate& full = expected.surface.cells[index];
    require(actual.elevation_m == full.elevation_m &&
                actual.gradient_x == full.gradient_x &&
                actual.gradient_y == full.gradient_y &&
                actual.gradient_covariance.xx == full.gradient_covariance.xx &&
                actual.gradient_covariance.xy == full.gradient_covariance.xy &&
                actual.detrended_roughness_rms_m ==
                    full.detrended_roughness_rms_m &&
                actual.residual_p95_m == full.residual_p95_m &&
                actual.support_ratio == full.support_ratio &&
                actual.status == full.status,
            "incremental terrain differs from a full recomputation");
    const CableLayingTerrainCell& actual_cable =
        incremental.layers->cable_laying.cells[index];
    const CableLayingTerrainCell& full_cable = expected.cable_laying.cells[index];
    require(actual_cable.elevation_m == full_cable.elevation_m &&
                actual_cable.roughness_m == full_cable.roughness_m &&
                actual_cable.known == full_cable.known &&
                actual_cable.confidence == full_cable.confidence &&
                actual_cable.obstacle == full_cable.obstacle &&
                actual_cable.forbidden == full_cable.forbidden,
            "incremental cable terrain differs from a full recomputation");
  }
  require(initial.layers->source_map_version.sequence_number == 1U &&
              incremental.layers->source_map_version.sequence_number == 2U,
          "publishing a new terrain version mutated an existing shared snapshot");
  std::cout << "[metrics] terrain_cells_recomputed="
            << incremental.diagnostics.recomputed_cell_count
            << " terrain_cells_reused="
            << incremental.diagnostics.reused_cell_count << '\n';

  IncrementalTerrainAnalyzer elevation_analyzer;
  const MapSnapshot flat = make_plane_map(0.0, 0.0);
  static_cast<void>(elevation_analyzer.analyze(flat, make_config()));
  MapSnapshot local_elevation = flat;
  local_elevation.version.sequence_number = 2U;
  local_elevation.version.timestamp = {2'000'000'000};
  local_elevation.cells.at(4U * local_elevation.width + 4U).elevation_m = 0.01;
  local_elevation.cells.at(4U * local_elevation.width + 4U)
      .measurement_timestamp = {1'500'000'000};
  local_elevation.update_regions = {{0.0, 0.0, 0.0, 0.0}};
  const IncrementalTerrainAnalysisResult elevation_update =
      elevation_analyzer.analyze(local_elevation, make_config());
  const TerrainLayers full_elevation =
      TerrainAnalyzer{}.analyze(local_elevation, make_config());
  require(elevation_update.diagnostics.mode ==
              TerrainAnalysisUpdateMode::incremental_update &&
              elevation_update.diagnostics.recomputed_cell_count == 49U &&
              elevation_update.diagnostics.reused_cell_count == 32U &&
              elevation_update.layers->surface.at(4U, 4U).elevation_m ==
                  full_elevation.surface.at(4U, 4U).elevation_m,
          "a local elevation update did not stay within its physical window");
}

void incremental_cache_fails_closed_on_bad_update_provenance() {
  using namespace underwater_planner::core;
  IncrementalTerrainAnalyzer analyzer;
  const MapSnapshot initial_map = make_plane_map(0.0, 0.0);
  static_cast<void>(analyzer.analyze(initial_map, make_config()));

  MapSnapshot undeclared_change = initial_map;
  undeclared_change.version.sequence_number = 2U;
  undeclared_change.cells.front().elevation_m = 1.0;
  undeclared_change.update_regions = {{0.0, 0.0, 0.0, 0.0}};
  bool rejected = false;
  try {
    static_cast<void>(analyzer.analyze(undeclared_change, make_config()));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected,
          "a map mutation outside its declared update region reused stale terrain");

  MapSnapshot replacement = initial_map;
  replacement.version = {"replacement-map", 1U, {2'000'000'000}, "map-frame"};
  replacement.update_regions.clear();
  const IncrementalTerrainAnalysisResult rebuilt =
      analyzer.analyze(replacement, make_config());
  require(rebuilt.diagnostics.mode == TerrainAnalysisUpdateMode::full_rebuild &&
              rebuilt.diagnostics.source_version_invalidated &&
              rebuilt.diagnostics.recomputed_cell_count == 81U,
          "a new map identity did not invalidate all cached derived terrain");

  IncrementalTerrainAnalyzer timestamp_analyzer;
  MapSnapshot before_measurement = make_plane_map(0.0, 0.0);
  before_measurement.cells.at(4U * before_measurement.width + 4U)
      .measurement_timestamp = {3'000'000'000};
  static_cast<void>(timestamp_analyzer.analyze(before_measurement, make_config()));
  MapSnapshot after_measurement = before_measurement;
  after_measurement.version.sequence_number = 2U;
  after_measurement.version.timestamp = {4'000'000'000};
  after_measurement.update_regions = {{0.0, 0.0, 0.0, 0.0}};
  const IncrementalTerrainAnalysisResult availability_changed =
      timestamp_analyzer.analyze(after_measurement, make_config());
  require(availability_changed.diagnostics.mode ==
                  TerrainAnalysisUpdateMode::full_rebuild &&
              availability_changed.diagnostics.source_version_invalidated,
          "timestamp advancement reused terrain after cell availability changed");
}

}  // namespace

int main() {
  multi_direction_plane_has_direction_independent_gradient();
  isolated_outlier_does_not_drag_the_surface_gradient();
  unknown_center_is_not_silently_interpolated();
  gradient_covariance_tracks_measurement_uncertainty();
  configuration_cannot_disable_robust_reweighting();
  map_and_analysis_configuration_versions_must_match();
  detrended_roughness_is_independent_of_plane_slope();
  larger_physical_window_does_not_create_false_roughness();
  unrepresentable_gradient_covariance_is_rejected();
  support_failure_modes_are_distinct();
  stale_outlier_has_less_influence_than_fresh_outlier();
  repeated_analysis_is_fieldwise_deterministic();
  cable_forbidden_and_obstacle_cells_reach_the_derived_layer();
  incremental_updates_expand_the_physical_window_and_share_snapshots();
  incremental_cache_fails_closed_on_bad_update_provenance();
  std::cout << "T06/T43 terrain analyzer checks passed\n";
}
