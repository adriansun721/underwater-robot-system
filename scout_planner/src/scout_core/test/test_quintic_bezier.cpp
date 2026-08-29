#include "scout_planner/core/quintic_bezier.hpp"
#include "test_support.hpp"

#include <cmath>
#include <exception>
#include <iostream>

namespace core = scout_planner::core;
using scout_planner::test_support::require;

namespace {

core::QuinticBezierSegment4d line_segment(std::uint64_t start = 0U,
                                           std::uint64_t duration = 2'000'000'000ULL) {
  core::QuinticBezierSegment4d segment{};
  segment.start_time_offset_ns = start;
  segment.duration_ns = duration;
  for (std::size_t i = 0U; i < 6U; ++i) {
    segment.position_control_points[i] = {static_cast<double>(i), 0.0, 1.0};
    segment.yaw_offset_control_points_rad[i] = 0.1 * static_cast<double>(i);
  }
  return segment;
}

void analytic_values_and_bounds() {
  const auto segment = core::QuinticBezierSegment::create(line_segment());
  require(segment.has_value(), segment.error().detail);
  const auto sample = segment.value().evaluate_normalized(0.5);
  require(sample.has_value(), sample.error().detail);
  require(std::abs(sample.value().position.x_m - 2.5) < 1.0e-12,
          "linear control polygon did not remain linear");
  require(std::abs(sample.value().velocity_mps.x_m - 2.5) < 1.0e-12,
          "time-scaled velocity is incorrect");
  require(std::abs(sample.value().acceleration_mps2.x_m) < 1.0e-12,
          "linear trajectory has non-zero acceleration");
  require(std::abs(sample.value().jerk_mps3.x_m) < 1.0e-12,
          "linear trajectory has non-zero jerk");
  require(std::abs(sample.value().yaw_rate_rps - 0.25) < 1.0e-12,
          "yaw rate is incorrect");
  const auto bounds = segment.value().derivative_bounds();
  require(std::abs(bounds.maximum_speed_mps - 2.5) < 1.0e-12,
          "convex-hull velocity bound is incorrect");
  require(bounds.maximum_acceleration_mps2 < 1.0e-12 &&
              bounds.maximum_jerk_mps3 < 1.0e-12,
          "convex-hull higher derivative bounds are incorrect");
}

void nonzero_boundary_derivatives_are_analytic() {
  auto segment = line_segment(0U, 1'000'000'000ULL);
  segment.position_control_points[1] = {0.2, 0.0, 1.0};
  segment.position_control_points[2] = {0.5, 0.0, 1.0};
  segment.yaw_offset_control_points_rad[0] = 0.0;
  segment.yaw_offset_control_points_rad[1] = 0.2;
  const auto created = core::QuinticBezierSegment::create(segment);
  require(created.has_value(), created.error().detail);
  const auto start = created.value().evaluate_normalized(0.0);
  require(start.has_value(), start.error().detail);
  require(std::abs(start.value().velocity_mps.x_m - 1.0) < 1.0e-12,
          "non-zero boundary velocity is incorrect");
  require(std::abs(start.value().yaw_rate_rps - 1.0) < 1.0e-12,
          "non-zero boundary yaw rate is incorrect");
}

void trajectory_continuity_identity_and_time() {
  auto first = line_segment();
  auto second = line_segment(2'000'000'000ULL);
  for (std::size_t i = 0U; i < 6U; ++i) {
    second.position_control_points[i].x_m += 5.0;
    second.yaw_offset_control_points_rad[i] += 0.5;
  }
  second.position_control_points[0] = first.position_control_points[5];
  second.position_control_points[1] = {6.0, 0.0, 1.0};
  second.position_control_points[2] = {7.0, 0.0, 1.0};
  second.yaw_offset_control_points_rad[0] = first.yaw_offset_control_points_rad[5];
  second.yaw_offset_control_points_rad[1] = 0.6;
  second.yaw_offset_control_points_rad[2] = 0.7;
  const auto trajectory = core::BezierTrajectory4d::create(
      "mission_enu", 3.0 * std::acos(-1.0), {first, second});
  require(trajectory.has_value(), trajectory.error().detail);
  require(std::abs(trajectory.value().initial_yaw_rad() + std::acos(-1.0)) < 1.0e-12,
          "initial yaw was not normalized");
  require(trajectory.value().validate_c2_continuity().has_value(),
          "valid adjacent segments failed C2 validation");
  const auto at_boundary = trajectory.value().evaluate_time(2'000'000'000ULL);
  require(at_boundary.has_value(), at_boundary.error().detail);
  require(std::abs(at_boundary.value().position.x_m - 5.0) < 1.0e-12,
          "boundary selected the wrong segment");
  const auto hash = trajectory.value().content_hash();
  const auto repeat = core::BezierTrajectory4d::create(
      "mission_enu", 3.0 * std::acos(-1.0), {first, second});
  require(repeat.has_value() && hash == repeat.value().content_hash(),
          "trajectory identity is not deterministic");
}

void rejects_invalid_and_discontinuous_inputs() {
  auto bad = line_segment(); bad.duration_ns = 0U;
  require(!core::QuinticBezierSegment::create(bad).has_value(),
          "zero-duration segment was accepted");
  auto discontinuous = line_segment(1U);
  const auto result = core::BezierTrajectory4d::create(
      "mission_enu", 0.0, {line_segment(), discontinuous});
  require(!result.has_value() && result.error().code == core::BezierErrorCode::invalid_time,
          "non-contiguous segments were accepted");
  require(!core::BezierTrajectory4d::create("map", 0.0, {line_segment()}).has_value(),
          "non-mission frame was accepted");
  auto bad_yaw = line_segment();
  bad_yaw.yaw_offset_control_points_rad[0] = 0.1;
  require(!core::BezierTrajectory4d::create("mission_enu", 0.0, {bad_yaw}).has_value(),
          "non-zero initial yaw offset was accepted");
  auto bad_seam = line_segment(2'000'000'000ULL);
  bad_seam.position_control_points[0].x_m += 0.1;
  require(!core::BezierTrajectory4d::create("mission_enu", 0.0,
                                             {line_segment(), bad_seam}).has_value(),
          "C2-discontinuous seam was accepted");
}

void preserves_unwrapped_heading_in_a_short_segment() {
  auto segment = line_segment(0U, 1U);
  for (std::size_t i = 0U; i < 6U; ++i)
    segment.yaw_offset_control_points_rad[i] = 4.0 * static_cast<double>(i) / 5.0;
  const auto trajectory = core::BezierTrajectory4d::create(
      "mission_enu", std::acos(-1.0) - 0.1, {segment});
  require(trajectory.has_value(), trajectory.error().detail);
  const auto sample = trajectory.value().evaluate_time(1U);
  require(sample.has_value(), sample.error().detail);
  require(sample.value().yaw_rad > 2.0 * std::acos(-1.0),
          "yaw offset was incorrectly wrapped at pi");
  require(!trajectory.value().evaluate_time(2U).has_value(),
          "time outside a short segment was accepted");
}

}  // namespace

int main() {
  try {
    analytic_values_and_bounds();
    nonzero_boundary_derivatives_are_analytic();
    trajectory_continuity_identity_and_time();
    rejects_invalid_and_discontinuous_inputs();
    preserves_unwrapped_heading_in_a_short_segment();
  } catch (const std::exception& error) {
    std::cerr << "[failure] " << error.what() << '\n';
    return 1;
  }
  std::cout << "[pass] quintic bezier seam\n";
  return 0;
}
