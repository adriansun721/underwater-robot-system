#pragma once

#include "underwater_planner/core/cable_model.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace underwater_planner::core {

enum class ParameterizationStatus {
  success,
  deadline_exceeded,
  initial_state_invalid,
  execution_envelope_mismatch,
  dynamics_infeasible,
  payout_infeasible,
  stopping_constraint_infeasible,
  numerically_invalid,
};

struct TrajectoryInitialState {
  double ground_speed_mps{};
  double payout_speed_mps{};
  double payout_acceleration_mps2{};
  double tension_n{};
};

struct TrajectoryParameterizationLimits {
  std::uint64_t version{};
  double sample_period_s{};
  double terminal_speed_mps{};
  double stopping_distance_margin_m{};
  bool require_terminal_stop{true};
  Duration timeout;
  std::uint64_t execution_profile_version{};
};

[[nodiscard]] std::string serialize_trajectory_parameterization_limits(
    const TrajectoryParameterizationLimits& limits);

struct ParameterizationDiagnostics {
  std::vector<std::string> issues;
  double maximum_lateral_acceleration_mps2{};
  double required_stopping_distance_m{};
  double available_stopping_distance_m{};
  double minimum_speed_mps{};
  double maximum_speed_mps{};
  std::uint64_t limits_version{};
  std::uint64_t envelope_version{};
  bool geometry_unchanged{};
};

struct ParameterizationResult {
  ParameterizationStatus status{ParameterizationStatus::numerically_invalid};
  std::optional<TimedPath> trajectory;
  ParameterizationDiagnostics diagnostics;
};

class TrajectoryParameterizer {
 public:
  using Clock = std::function<MonotonicTime()>;

  TrajectoryParameterizer();
  explicit TrajectoryParameterizer(Clock clock);

  [[nodiscard]] ParameterizationResult parameterize(
      const GeometricPath& geometry, const TrajectoryInitialState& initial_state,
      const ExecutionOperatingEnvelope& certified_envelope,
      const TrajectoryParameterizationLimits& limits) const;

 private:
  Clock clock_;
};

}  // namespace underwater_planner::core
