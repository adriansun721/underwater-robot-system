#include "underwater_planner/core/terrain_analyzer.hpp"

#include "terrain_test_config.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {

void require(const bool condition, const char* message) {
  if (!condition) {
    std::cerr << "T07 failure: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

underwater_planner::core::MapSnapshot make_step_map(
    const double normal_x = 1.0, const double normal_y = 0.0,
    const double step_height_m = 0.2) {
  using namespace underwater_planner::core;
  MapSnapshot map;
  map.version = {"step-map", 1, {1'000'000'000}, "map-frame"};
  map.width = 21;
  map.height = 21;
  map.resolution_m = 0.1;
  map.origin_x_m = -1.0;
  map.origin_y_m = -1.0;
  map.derived_configuration_version = 7;
  for (std::size_t row = 0; row < map.height; ++row) {
    for (std::size_t column = 0; column < map.width; ++column) {
      const double x_m = map.origin_x_m +
                         static_cast<double>(column) * map.resolution_m;
      const double y_m = map.origin_y_m +
                         static_cast<double>(row) * map.resolution_m;
      const double base_elevation_m = 0.03 * x_m - 0.02 * y_m;
      const double signed_distance_m = normal_x * x_m + normal_y * y_m;
      map.cells.push_back({base_elevation_m +
                               (signed_distance_m >= 0.0 ? step_height_m : 0.0),
                           1.0e-4, 1.0, true,
                           MonotonicTime{900'000'000}});
    }
  }
  return map;
}

underwater_planner::core::TerrainAnalysisConfig make_config() {
  return underwater_planner::testing::make_terrain_analysis_config(
      7, "synthetic-step/v1", 0.7, 0.7);
}

underwater_planner::core::MapSnapshot make_ramped_step_map() {
  using namespace underwater_planner::core;
  MapSnapshot map = make_step_map(1.0, 0.0, 0.0);
  for (std::size_t row = 0; row < map.height; ++row) {
    for (std::size_t column = 0; column < map.width; ++column) {
      const double x_m = map.origin_x_m +
                         static_cast<double>(column) * map.resolution_m;
      const double step_offset_m =
          x_m <= -0.2 ? 0.0
                      : (x_m >= 0.2 ? 0.2 : 0.5 * (x_m + 0.2));
      map.cells.at(row * map.width + column).elevation_m += step_offset_m;
    }
  }
  return map;
}

void complete_step_geometry_is_extracted_from_two_support_surfaces() {
  // Design: 18.2.1-4
  using namespace underwater_planner::core;
  const MapSnapshot map = make_step_map();
  const TerrainLayers layers = TerrainAnalyzer{}.analyze(map, make_config());

  require(layers.steps.estimates.size() == 1,
          "a single straight step did not produce one estimate");
  const StepEstimate& estimate = layers.steps.estimates.front();
  require(estimate.status == StepEstimateStatus::valid,
          "a fully supported step was not valid");
  require(std::abs(estimate.edge.height_m - 0.2) < 1.0e-9,
          "step height was not the full fitted surface separation");
  require(std::abs(estimate.edge.normal_low_to_high.x - 1.0) < 1.0e-9 &&
              std::abs(estimate.edge.normal_low_to_high.y) < 1.0e-9,
          "step normal did not point from the low side to the high side");
  require(estimate.edge.extent.size() == 2 &&
              estimate.edge.extent.front().y_m < -0.9 &&
              estimate.edge.extent.back().y_m > 0.9,
          "step extent did not cover the observed edge");
  require(estimate.edge.transition_width_m > 0.0 &&
              estimate.edge.confidence >= 0.75,
          "step width or confidence was not reported");
  require(layers.surface.at(10, 9).status ==
                  TerrainEstimateStatus::discontinuous &&
              layers.surface.at(10, 10).status ==
                  TerrainEstimateStatus::discontinuous,
          "surface windows crossing a high-confidence step stayed continuous");
  require(layers.surface.at(10, 9).elevation_m ==
                  map.at(10, 9).elevation_m &&
              layers.surface.at(10, 10).elevation_m ==
                  map.at(10, 10).elevation_m,
          "a cross-step plane was reused as discontinuous support elevation");
}

bool contains_status(const underwater_planner::core::StepLayer& layer,
                     const underwater_planner::core::StepEstimateStatus status) {
  for (const underwater_planner::core::StepEstimate& estimate :
       layer.estimates) {
    if (estimate.status == status) return true;
  }
  return false;
}

void oblique_observation_preserves_complete_height_and_world_normal() {
  using namespace underwater_planner::core;
  const double inverse_sqrt_two = 1.0 / std::sqrt(2.0);
  const TerrainLayers layers = TerrainAnalyzer{}.analyze(
      make_step_map(inverse_sqrt_two, inverse_sqrt_two), make_config());

  if (layers.steps.estimates.size() != 1 ||
      layers.steps.estimates.front().status != StepEstimateStatus::valid) {
    std::cerr << "oblique diagnostics: count=" << layers.steps.estimates.size();
    for (const StepEstimate& estimate : layers.steps.estimates) {
      std::cerr << " status=" << static_cast<int>(estimate.status);
    }
    std::cerr << '\n';
  }
  require(layers.steps.estimates.size() == 1 &&
              layers.steps.estimates.front().status ==
                  StepEstimateStatus::valid,
          "an oblique step was not extracted as one stable edge");
  const StepEdge& edge = layers.steps.estimates.front().edge;
  require(std::abs(edge.height_m - 0.2) < 1.0e-9,
          "oblique observation scaled the complete step height");
  require(std::abs(edge.normal_low_to_high.x - inverse_sqrt_two) < 0.02 &&
              std::abs(edge.normal_low_to_high.y - inverse_sqrt_two) < 0.02,
          "oblique edge normal was not expressed in the map frame");
}

void transition_width_tracks_a_multi_cell_elevation_band() {
  using namespace underwater_planner::core;
  const TerrainLayers sharp_layers =
      TerrainAnalyzer{}.analyze(make_step_map(), make_config());
  const StepEstimate& sharp = sharp_layers.steps.estimates.front();
  const TerrainLayers ramp_layers =
      TerrainAnalyzer{}.analyze(make_ramped_step_map(), make_config());
  require(ramp_layers.steps.estimates.size() == 1 &&
              ramp_layers.steps.estimates.front().status ==
                  StepEstimateStatus::valid,
          "a supported multi-cell transition was not a valid step");
  const StepEdge& ramp = ramp_layers.steps.estimates.front().edge;
  require(std::abs(ramp.height_m - 0.2) < 0.01,
          "transition samples contaminated the two support-surface fits");
  if (!(ramp.transition_width_m > 2.0 * sharp.edge.transition_width_m &&
        ramp.transition_width_m >= 0.3)) {
    std::cerr << "transition diagnostics: sharp="
              << sharp.edge.transition_width_m
              << " ramp=" << ramp.transition_width_m << '\n';
  }
  require(ramp.transition_width_m > 2.0 * sharp.edge.transition_width_m &&
              ramp.transition_width_m >= 0.3,
          "transition width stayed fixed at one grid cell");
}

void invalid_step_candidates_report_their_rejection_reason() {
  using namespace underwater_planner::core;

  MapSnapshot unknown_low_side = make_step_map();
  for (std::size_t row = 0; row < unknown_low_side.height; ++row) {
    for (std::size_t column = 0; column < unknown_low_side.width / 2;
         ++column) {
      unknown_low_side.cells.at(row * unknown_low_side.width + column).known =
          false;
    }
  }
  require(contains_status(
              TerrainAnalyzer{}.analyze(unknown_low_side, make_config()).steps,
              StepEstimateStatus::insufficient_side_support),
          "an unknown step side did not report insufficient side support");

  MapSnapshot low_confidence = make_step_map();
  for (MapCell& cell : low_confidence.cells) cell.confidence = 0.5;
  require(contains_status(
              TerrainAnalyzer{}.analyze(low_confidence, make_config()).steps,
              StepEstimateStatus::low_confidence),
          "a low-confidence edge did not report low confidence");

  MapSnapshot noisy = make_step_map();
  for (MapCell& cell : noisy.cells) cell.elevation_variance_m2 = 0.08;
  require(contains_status(TerrainAnalyzer{}.analyze(noisy, make_config()).steps,
                          StepEstimateStatus::noise_not_significant),
          "one-sided fit uncertainty was reduced to a sample-count average");

  const TerrainLayers subthreshold = TerrainAnalyzer{}.analyze(
      make_step_map(1.0, 0.0, 0.075), make_config());
  require(contains_status(subthreshold.steps,
                          StepEstimateStatus::below_minimum_height),
          "a geometrically detected sub-threshold edge lacked a clear status");
}

void degenerate_and_short_extents_fail_explicitly() {
  using namespace underwater_planner::core;
  MapSnapshot duplicate_extent = make_step_map();
  for (std::size_t row = 0; row < duplicate_extent.height; ++row) {
    if (row == duplicate_extent.height / 2) continue;
    for (std::size_t column = 0; column < duplicate_extent.width; ++column) {
      duplicate_extent.cells.at(row * duplicate_extent.width + column).known =
          false;
    }
  }
  require(contains_status(
              TerrainAnalyzer{}.analyze(duplicate_extent, make_config()).steps,
              StepEstimateStatus::duplicate_extent_point),
          "a one-point extent did not report a duplicate endpoint");

  MapSnapshot short_extent = make_step_map();
  for (std::size_t row = 0; row < short_extent.height; ++row) {
    if (row >= 9 && row <= 11) continue;
    for (std::size_t column = 0; column < short_extent.width; ++column) {
      short_extent.cells.at(row * short_extent.width + column).known = false;
    }
  }
  require(contains_status(
              TerrainAnalyzer{}.analyze(short_extent, make_config()).steps,
              StepEstimateStatus::insufficient_extent),
          "a short edge did not report insufficient extent");
}

void unstable_closed_edge_and_isolated_spike_are_not_valid_steps() {
  using namespace underwater_planner::core;
  MapSnapshot plateau = make_step_map(1.0, 0.0, 0.0);
  for (std::size_t row = 0; row < plateau.height; ++row) {
    for (std::size_t column = 0; column < plateau.width; ++column) {
      const double x_m = plateau.origin_x_m +
                         static_cast<double>(column) * plateau.resolution_m;
      const double y_m = plateau.origin_y_m +
                         static_cast<double>(row) * plateau.resolution_m;
      if (std::abs(x_m) <= 0.4 && std::abs(y_m) <= 0.4) {
        plateau.cells.at(row * plateau.width + column).elevation_m += 0.2;
      }
    }
  }
  require(contains_status(TerrainAnalyzer{}.analyze(plateau, make_config()).steps,
                          StepEstimateStatus::unstable_normal),
          "a closed edge with cancelling normals was accepted as stable");

  MapSnapshot isolated_spike = make_step_map(1.0, 0.0, 0.0);
  isolated_spike.cells.at(10 * isolated_spike.width + 10).elevation_m += 0.5;
  require(TerrainAnalyzer{}
              .analyze(isolated_spike, make_config())
              .steps.estimates.empty(),
          "edge-preserving denoising promoted an isolated spike to a step");
}

void repeated_step_analysis_is_fieldwise_deterministic() {
  using namespace underwater_planner::core;
  const MapSnapshot map = make_step_map();
  const TerrainLayers first = TerrainAnalyzer{}.analyze(map, make_config());
  const TerrainLayers second = TerrainAnalyzer{}.analyze(map, make_config());
  require(first.steps.estimates.size() == second.steps.estimates.size(),
          "repeated analysis changed the step count");
  for (std::size_t index = 0; index < first.steps.estimates.size(); ++index) {
    const StepEstimate& left = first.steps.estimates[index];
    const StepEstimate& right = second.steps.estimates[index];
    require(left.status == right.status &&
                left.edge.height_m == right.edge.height_m &&
                left.edge.normal_low_to_high.x ==
                    right.edge.normal_low_to_high.x &&
                left.edge.normal_low_to_high.y ==
                    right.edge.normal_low_to_high.y &&
                left.edge.transition_width_m ==
                    right.edge.transition_width_m &&
                left.edge.confidence == right.edge.confidence &&
                left.edge.extent.size() == right.edge.extent.size(),
            "repeated analysis changed a step field");
    for (std::size_t point = 0; point < left.edge.extent.size(); ++point) {
      require(left.edge.extent[point].x_m == right.edge.extent[point].x_m &&
                  left.edge.extent[point].y_m == right.edge.extent[point].y_m,
              "repeated analysis changed an extent point");
    }
  }
}

}  // namespace

int main() {
  complete_step_geometry_is_extracted_from_two_support_surfaces();
  oblique_observation_preserves_complete_height_and_world_normal();
  transition_width_tracks_a_multi_cell_elevation_band();
  invalid_step_candidates_report_their_rejection_reason();
  degenerate_and_short_extents_fail_explicitly();
  unstable_closed_edge_and_isolated_spike_are_not_valid_steps();
  repeated_step_analysis_is_fieldwise_deterministic();
  std::cout << "T07 step geometry checks passed\n";
}
