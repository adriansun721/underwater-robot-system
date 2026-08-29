#pragma once

#include "underwater_planner/core/cable_model.hpp"

#include <cstddef>

namespace underwater_planner::core::detail {

[[nodiscard]] std::size_t dynamic_storage_bytes(
    const CableConstraintMemory& memory);
[[nodiscard]] std::size_t dynamic_storage_bytes(const CableState& state);
[[nodiscard]] std::size_t dynamic_storage_bytes(const GeometricPath& path);
[[nodiscard]] std::size_t dynamic_storage_bytes(
    const CablePrediction& prediction);

}  // namespace underwater_planner::core::detail
