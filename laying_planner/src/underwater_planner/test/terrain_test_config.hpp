#pragma once

#include "underwater_planner/core/terrain_analyzer.hpp"

#include <cstdint>
#include <string>
#include <utility>

namespace underwater_planner::testing {

inline core::TerrainAnalysisConfig make_terrain_analysis_config(
    const std::uint64_t config_version, std::string operating_domain_id,
    const double surface_window_size_m,
    const double minimum_fit_support_ratio) {
  core::TerrainAnalysisConfig config;
  config.config_version = config_version;
  config.operating_domain_id = std::move(operating_domain_id);
  config.surface_window_size_m = surface_window_size_m;
  config.minimum_fit_support_ratio = minimum_fit_support_ratio;
  config.huber_delta_m = 0.02;
  config.minimum_elevation_variance_m2 = 1.0e-6;
  config.temporal_weight_half_life_s = 10.0;
  config.maximum_irls_iterations = 12;
  config.minimum_step_height_m = 0.1;
  config.step_support_band_width_m = 0.4;
  config.minimum_step_side_support_ratio = 0.7;
  config.minimum_step_extent_m = 0.8;
  config.step_noise_sigma_multiplier = 3.0;
  config.minimum_step_confidence = 0.75;
  config.minimum_step_normal_consistency = 0.7;
  return config;
}

}  // namespace underwater_planner::testing
