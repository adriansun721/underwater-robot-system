#pragma once

#include "underwater_planner/core/versioned_snapshot.hpp"

#include <array>

namespace underwater_planner::core::detail {

using Matrix3 = std::array<std::array<double, 3>, 3>;
using Vector3 = std::array<double, 3>;

[[nodiscard]] bool finite(double value);
[[nodiscard]] bool usable(const MapCell& cell, MonotonicTime map_timestamp);
[[nodiscard]] bool solve_3x3_linear_system(Matrix3 matrix,
                                           Vector3 right_hand_side,
                                           Vector3& solution);

}  // namespace underwater_planner::core::detail
