#pragma once

#include <cmath>

namespace underwater_planner::core {

[[nodiscard]] inline bool valid_step_alignment_domain(
    const double minimum_crossing_alignment,
    const double transition_band) noexcept {
  return std::isfinite(minimum_crossing_alignment) &&
         minimum_crossing_alignment > 0.0 &&
         minimum_crossing_alignment < 1.0 && std::isfinite(transition_band) &&
         transition_band > 0.0 &&
         transition_band < minimum_crossing_alignment &&
         minimum_crossing_alignment + transition_band < 1.0;
}

}  // namespace underwater_planner::core
