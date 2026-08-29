#pragma once

#include "underwater_planner/core/versioned_snapshot.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace underwater_planner::core {

enum class TerrainEstimateStatus {
  valid,
  insufficient_support,
  ill_conditioned,
  invalid_covariance,
  discontinuous,
};

using Polyline2D = std::vector<Point2d>;

struct StepEdge {
  Polyline2D extent;
  Vector2d normal_low_to_high;
  double height_m{};
  double transition_width_m{};
  double confidence{};
};

enum class StepEstimateStatus {
  valid,
  insufficient_side_support,
  noise_not_significant,
  below_minimum_height,
  unstable_normal,
  insufficient_extent,
  duplicate_extent_point,
  low_confidence,
};

struct StepEstimate {
  StepEdge edge;
  StepEstimateStatus status{StepEstimateStatus::insufficient_side_support};
};

struct StepLayer {
  std::vector<StepEstimate> estimates;
};

struct GradientCovariance {
  // Gradient components are dimensionless (m/m), so covariance is (m/m)^2.
  double xx{};
  double xy{};
  double yx{};
  double yy{};
};

struct SurfaceEstimate {
  double elevation_m{};
  double gradient_x{};
  double gradient_y{};
  double slope_angle_rad{};
  GradientCovariance gradient_covariance;
  double detrended_roughness_rms_m{};
  double residual_p95_m{};
  double support_ratio{};
  TerrainEstimateStatus status{TerrainEstimateStatus::insufficient_support};
};

struct SurfaceLayer {
  std::size_t width{};
  std::size_t height{};
  double resolution_m{};
  double origin_x_m{};
  double origin_y_m{};
  std::vector<SurfaceEstimate> cells;

  [[nodiscard]] const SurfaceEstimate& at(std::size_t row,
                                          std::size_t column) const;
};

struct CableLayingTerrainCell {
  double elevation_m{};
  double roughness_m{};
  bool known{};
  double confidence{};
  bool obstacle{};
  bool forbidden{};
};

struct CableLayingTerrainLayer {
  std::vector<CableLayingTerrainCell> cells;

  [[nodiscard]] const CableLayingTerrainCell& at(
      std::size_t row, std::size_t column,
      const SurfaceLayer& surface) const;
};

struct TerrainLayers {
  MapVersion source_map_version;
  std::uint64_t analysis_config_version{};
  std::string operating_domain_id;
  double surface_fit_window_size_m{};
  SurfaceLayer surface;
  CableLayingTerrainLayer cable_laying;
  StepLayer steps;
};

struct TerrainAnalysisConfig {
  std::uint64_t config_version{};
  std::string operating_domain_id;
  // Full diameter of the circular physical fitting window.
  double surface_window_size_m{};
  double minimum_fit_support_ratio{};
  double huber_delta_m{};
  double minimum_elevation_variance_m2{};
  double temporal_weight_half_life_s{};
  // Includes the initial weighted least-squares pass.
  std::size_t maximum_irls_iterations{};
  double minimum_step_height_m{};
  double step_support_band_width_m{};
  double minimum_step_side_support_ratio{};
  double minimum_step_extent_m{};
  double step_noise_sigma_multiplier{};
  double minimum_step_confidence{};
  double minimum_step_normal_consistency{};
};

[[nodiscard]] std::string serialize_terrain_analysis_config(
    const TerrainAnalysisConfig& config);

class TerrainAnalyzer {
 public:
  [[nodiscard]] TerrainLayers analyze(
      const MapSnapshot& height_map,
      const TerrainAnalysisConfig& config) const;
};

enum class TerrainAnalysisUpdateMode {
  full_rebuild,
  incremental_update,
  cache_hit,
};

struct IncrementalTerrainAnalysisDiagnostics {
  TerrainAnalysisUpdateMode mode{TerrainAnalysisUpdateMode::full_rebuild};
  std::size_t recomputed_cell_count{};
  std::size_t reused_cell_count{};
  bool source_version_invalidated{};
  std::vector<MapUpdateRegion> expanded_update_regions;
};

struct IncrementalTerrainAnalysisResult {
  std::shared_ptr<const TerrainLayers> layers;
  IncrementalTerrainAnalysisDiagnostics diagnostics;
};

class IncrementalTerrainAnalyzer {
 public:
  [[nodiscard]] IncrementalTerrainAnalysisResult analyze(
      const MapSnapshot& height_map, const TerrainAnalysisConfig& config);

 private:
  std::optional<MapSnapshot> cached_map_;
  std::string cached_config_;
  std::shared_ptr<const TerrainLayers> cached_layers_;
};

}  // namespace underwater_planner::core
