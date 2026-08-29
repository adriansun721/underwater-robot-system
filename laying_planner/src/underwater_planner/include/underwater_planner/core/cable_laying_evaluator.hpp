#pragma once

#include "underwater_planner/core/data_contract.hpp"
#include "underwater_planner/core/terrain_analyzer.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace underwater_planner::core {

enum class CableHistoryBoundary {
  actual_laying_history,
  explicit_task_start,
};

struct CableLayingLimits {
  std::uint64_t version{};
  std::string operating_domain_id;
  double preferred_curvature_per_m{};
  double maximum_curvature_per_m{};
  double curvature_evaluation_spacing_m{};
  double support_evaluation_length_m{};
  double medium_support_proxy_range_m{};
  double maximum_support_proxy_range_m{};
  double minimum_terrain_confidence{};
  double minimum_distinct_touchdown_distance_m{};
  double bend_weight{};
  double terrain_risk_weight{};
  double roughness_weight{};
};

class CableLayingEvaluator {
 public:
  [[nodiscard]] std::optional<CableConstraintMemory> canonicalize_memory(
      const CableConstraintMemory& memory,
      const CableLayingLimits& limits,
      CableHistoryBoundary history_boundary) const;

  [[nodiscard]] CableLayingEvaluation evaluate_segment(
      const CableConstraintMemory& initial_memory,
      const GeometricPath& touchdown_segment,
      const std::vector<CableState>& state_profile,
      const TerrainLayers& terrain,
      const CableLayingLimits& limits,
      CableHistoryBoundary history_boundary) const;

  [[nodiscard]] CableLayingEvaluation evaluate(
      const CableConstraintMemory& initial_memory,
      const GeometricPath& touchdown_path,
      const std::vector<CableState>& state_profile,
      const TerrainLayers& terrain,
      const CableLayingLimits& limits,
      CableHistoryBoundary history_boundary) const;

  [[nodiscard]] bool future_equivalent(
      const CableConstraintMemory& left,
      const CableConstraintMemory& right) const noexcept;
};

}  // namespace underwater_planner::core
