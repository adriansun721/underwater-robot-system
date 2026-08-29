#pragma once

#include "underwater_planner/core/versioned_snapshot.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace underwater_planner::core::detail {

enum class PointLocation { outside, boundary, inside };

inline double cross_product(const Point2d& origin, const Point2d& first,
                            const Point2d& second) noexcept {
  return (first.x_m - origin.x_m) * (second.y_m - origin.y_m) -
         (first.y_m - origin.y_m) * (second.x_m - origin.x_m);
}

inline double geometry_tolerance(const Point2d& first,
                                 const Point2d& second) noexcept {
  const double scale =
      std::max({1.0, std::abs(first.x_m), std::abs(first.y_m),
                std::abs(second.x_m), std::abs(second.y_m)});
  return 64.0 * std::numeric_limits<double>::epsilon() * scale * scale;
}

inline bool point_on_segment(const Point2d& point, const Point2d& start,
                             const Point2d& end) noexcept {
  const double tolerance = geometry_tolerance(start, end);
  return std::abs(cross_product(start, end, point)) <= tolerance &&
         point.x_m >= std::min(start.x_m, end.x_m) - tolerance &&
         point.x_m <= std::max(start.x_m, end.x_m) + tolerance &&
         point.y_m >= std::min(start.y_m, end.y_m) - tolerance &&
         point.y_m <= std::max(start.y_m, end.y_m) + tolerance;
}

inline PointLocation locate_point(
    const Point2d& point, const std::vector<Point2d>& polygon) noexcept {
  bool inside = false;
  for (std::size_t index = 0U, previous = polygon.size() - 1U;
       index < polygon.size(); previous = index++) {
    const Point2d& start = polygon[previous];
    const Point2d& end = polygon[index];
    if (point_on_segment(point, start, end)) {
      return PointLocation::boundary;
    }
    const bool crosses_y = (start.y_m > point.y_m) != (end.y_m > point.y_m);
    if (crosses_y) {
      const double crossing_x =
          start.x_m + (point.y_m - start.y_m) *
                          (end.x_m - start.x_m) / (end.y_m - start.y_m);
      if (crossing_x >= point.x_m) {
        inside = !inside;
      }
    }
  }
  return inside ? PointLocation::inside : PointLocation::outside;
}

inline bool point_covered_by_polygon(
    const Point2d& point, const std::vector<Point2d>& polygon) noexcept {
  return locate_point(point, polygon) != PointLocation::outside;
}

inline double point_to_segment_distance(const Point2d& point,
                                        const Point2d& start,
                                        const Point2d& end) noexcept {
  const double x = end.x_m - start.x_m;
  const double y = end.y_m - start.y_m;
  const double squared_length = x * x + y * y;
  if (squared_length <= geometry_tolerance(start, end)) {
    return std::hypot(point.x_m - start.x_m, point.y_m - start.y_m);
  }
  const double projection = std::clamp(
      ((point.x_m - start.x_m) * x + (point.y_m - start.y_m) * y) /
          squared_length,
      0.0, 1.0);
  return std::hypot(point.x_m - (start.x_m + projection * x),
                    point.y_m - (start.y_m + projection * y));
}

inline bool segments_intersect(const Point2d& first_start,
                               const Point2d& first_end,
                               const Point2d& second_start,
                               const Point2d& second_end) noexcept {
  const double first_a = cross_product(first_start, first_end, second_start);
  const double first_b = cross_product(first_start, first_end, second_end);
  const double second_a = cross_product(second_start, second_end, first_start);
  const double second_b = cross_product(second_start, second_end, first_end);
  const double tolerance = std::max(geometry_tolerance(first_start, first_end),
                                    geometry_tolerance(second_start, second_end));
  if (((first_a > tolerance && first_b < -tolerance) ||
       (first_a < -tolerance && first_b > tolerance)) &&
      ((second_a > tolerance && second_b < -tolerance) ||
       (second_a < -tolerance && second_b > tolerance))) {
    return true;
  }
  return point_on_segment(first_start, second_start, second_end) ||
         point_on_segment(first_end, second_start, second_end) ||
         point_on_segment(second_start, first_start, first_end) ||
         point_on_segment(second_end, first_start, first_end);
}

inline double segment_distance(const Point2d& first_start,
                               const Point2d& first_end,
                               const Point2d& second_start,
                               const Point2d& second_end) noexcept {
  if (segments_intersect(first_start, first_end, second_start, second_end)) {
    return 0.0;
  }
  return std::min(
      {point_to_segment_distance(first_start, second_start, second_end),
       point_to_segment_distance(first_end, second_start, second_end),
       point_to_segment_distance(second_start, first_start, first_end),
       point_to_segment_distance(second_end, first_start, first_end)});
}

}  // namespace underwater_planner::core::detail
