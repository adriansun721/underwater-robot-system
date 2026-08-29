#include "terrain_analysis_primitives.hpp"

#include <algorithm>
#include <cmath>

namespace underwater_planner::core::detail {

bool finite(const double value) { return std::isfinite(value); }

bool usable(const MapCell& cell, const MonotonicTime map_timestamp) {
  return cell.known && finite(cell.elevation_m) &&
         finite(cell.elevation_variance_m2) &&
         cell.elevation_variance_m2 >= 0.0 && finite(cell.confidence) &&
         cell.confidence > 0.0 && cell.confidence <= 1.0 &&
         cell.measurement_timestamp.nanoseconds >= 0 &&
         cell.measurement_timestamp.nanoseconds <= map_timestamp.nanoseconds;
}

bool solve_3x3_linear_system(Matrix3 matrix, Vector3 right_hand_side,
                             Vector3& solution) {
  for (std::size_t pivot = 0; pivot < 3; ++pivot) {
    std::size_t best = pivot;
    for (std::size_t row = pivot + 1; row < 3; ++row) {
      if (std::abs(matrix.at(row).at(pivot)) >
          std::abs(matrix.at(best).at(pivot))) {
        best = row;
      }
    }
    const double scale = std::max(
        {std::abs(matrix[0][0]), std::abs(matrix[1][1]),
         std::abs(matrix[2][2])});
    if (!finite(scale) || scale <= 0.0 ||
        std::abs(matrix.at(best).at(pivot)) <= scale * 1.0e-12) {
      return false;
    }
    std::swap(matrix.at(pivot), matrix.at(best));
    std::swap(right_hand_side.at(pivot), right_hand_side.at(best));
    for (std::size_t row = pivot + 1; row < 3; ++row) {
      const double factor =
          matrix.at(row).at(pivot) / matrix.at(pivot).at(pivot);
      for (std::size_t column = pivot; column < 3; ++column) {
        matrix.at(row).at(column) -=
            factor * matrix.at(pivot).at(column);
      }
      right_hand_side.at(row) -= factor * right_hand_side.at(pivot);
    }
  }
  for (std::size_t offset = 0; offset < 3; ++offset) {
    const std::size_t row = 2 - offset;
    double value = right_hand_side.at(row);
    for (std::size_t column = row + 1; column < 3; ++column) {
      value -= matrix.at(row).at(column) * solution.at(column);
    }
    solution.at(row) = value / matrix.at(row).at(row);
  }
  return std::all_of(solution.begin(), solution.end(), finite);
}

}  // namespace underwater_planner::core::detail
