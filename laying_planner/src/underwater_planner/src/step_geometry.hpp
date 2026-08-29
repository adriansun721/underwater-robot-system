#pragma once

#include "underwater_planner/core/terrain_analyzer.hpp"

namespace underwater_planner::core::detail {

[[nodiscard]] StepLayer extract_step_geometry(
    const MapSnapshot& map, const TerrainAnalysisConfig& config);

[[nodiscard]] bool step_geometry_changed_in_regions(
    const MapSnapshot& previous, const MapSnapshot& current,
    const std::vector<MapUpdateRegion>& regions,
    const TerrainAnalysisConfig& config);

void mark_step_discontinuities(const StepLayer& steps,
                               const TerrainAnalysisConfig& config,
                               const MapSnapshot& map,
                               SurfaceLayer& surface);

void mark_step_discontinuities(
    const StepLayer& steps, const TerrainAnalysisConfig& config,
    const MapSnapshot& map, SurfaceLayer& surface,
    const std::vector<std::size_t>& affected_cell_indices);

}  // namespace underwater_planner::core::detail
