#pragma once

#include "scout_planner/core/hybrid_map_query.hpp"
#include "scout_planner/core/quintic_bezier.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace scout_planner::core {

enum class ContinuousGeometryValidationStatus : std::uint8_t {
  safe,
  unsafe,
  inconclusive,
  invalid_input,
};

enum class ContinuousGeometryValidationOutcome : std::uint8_t {
  success,
  validation_rejected,
  validation_inconclusive,
  input_invalid,
  numerically_invalid,
};

struct ContinuousGeometryValidationConfig {
  SafetyMargins safety_margins{};
  // A leaf interval this short cannot be proven by the chord bound.
  double minimum_interval_s{1.0e-3};
  std::uint32_t maximum_refinement_depth{20U};
  // A zero limit disables that optional derivative gate.
  double maximum_speed_mps{0.0};
  double maximum_acceleration_mps2{0.0};
  double maximum_yaw_rate_rps{0.0};
  double maximum_yaw_acceleration_rps2{0.0};
};

struct ContinuousGeometryValidationReport {
  ContinuousGeometryValidationStatus status{
      ContinuousGeometryValidationStatus::invalid_input};
  ContinuousGeometryValidationOutcome primary_outcome{
      ContinuousGeometryValidationOutcome::input_invalid};
  std::optional<std::uint64_t> earliest_failure_time_offset_ns;
  std::optional<double> minimum_collision_margin_m;
  std::uint32_t refinement_depth{};
  std::uint64_t checked_interval_count{};
  std::string validated_map_id;
  std::uint64_t validated_map_version{};
  Hash256 validated_map_content_identity{};
  double discrete_margin_m{};
  Hash256 validated_trajectory_content_identity{};
  std::vector<std::string> diagnostics;

  [[nodiscard]] bool has_value() const noexcept {
    return status == ContinuousGeometryValidationStatus::safe;
  }
};

class ContinuousGeometryValidator final {
 public:
  [[nodiscard]] static ContinuousGeometryValidationReport validate(
      const HybridMapQuery& map, const BezierTrajectory4d& trajectory,
      const ContinuousGeometryValidationConfig& config = {});
};

}  // namespace scout_planner::core
