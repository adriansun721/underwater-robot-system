#pragma once

#include "scout_planner/core/hybrid_map_query.hpp"
#include "scout_planner/core/planning_context.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

namespace scout_planner::core {

struct MotionSample {
  double duration_s{};
  Point3dEnu position_m{};
  Point3dEnu velocity_mps{};
  Point3dEnu acceleration_mps2{};
  double yaw_rad{};
  double yaw_rate_radps{};
  double yaw_acceleration_radps2{};
  double roll_rad{};
  double pitch_rad{};
};

struct CapabilityEnergyGateConfig {
  // Required calibration bounds used when the current contract has no spatial
  // gradient. All values are SI and must be finite and non-negative.
  double conservative_acceleration_margin_mps2;
  double yaw_rate_error_radps;
  double yaw_acceleration_error_radps2;
};

enum class CapabilityEnergyFailure {
  invalid_trajectory,
  current_invalid,
  current_outside_region,
  capability_infeasible,
  health_profile_mismatch,
  non_production_profile,
  energy_insufficient,
};

struct CapabilityEnergyError {
  CapabilityEnergyFailure code;
  std::size_t sample_index{};
  std::string detail;
};

struct CapabilityEnergyReport {
  double minimum_capability_margin{};
  double minimum_water_speed_margin_mps{};
  double minimum_acceleration_margin_mps2{};
  double minimum_vertical_speed_margin_mps{};
  double minimum_yaw_rate_margin_radps{};
  double minimum_yaw_acceleration_margin_radps2{};
  double estimated_plan_energy_j{};
  double required_energy_j{};
  double energy_margin_j{};
  std::uint64_t capability_profile_version{};
  std::uint64_t energy_model_version{};
  std::uint64_t energy_state_version{};
};

class CapabilityEnergyResult final {
 public:
  static CapabilityEnergyResult success(CapabilityEnergyReport report);
  static CapabilityEnergyResult failure(CapabilityEnergyError error);

  [[nodiscard]] bool has_value() const noexcept;
  [[nodiscard]] const CapabilityEnergyReport& value() const;
  [[nodiscard]] const CapabilityEnergyError& error() const;

 private:
  explicit CapabilityEnergyResult(CapabilityEnergyReport report);
  explicit CapabilityEnergyResult(CapabilityEnergyError error);
  std::variant<CapabilityEnergyReport, CapabilityEnergyError> storage_;
};

class CapabilityEnergyGate final {
 public:
  [[nodiscard]] static CapabilityEnergyResult evaluate(
      const ScoutPlanningContext& context,
      const std::vector<MotionSample>& trajectory,
      const CapabilityEnergyGateConfig& configuration);
};

}  // namespace scout_planner::core
