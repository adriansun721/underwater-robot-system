#pragma once

#include "scout_planner/core/quintic_bezier.hpp"
#include "scout_planner/core/state_lattice_astar.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace scout_planner::core {

struct FeasibleTubeSample {
  Point3dEnu center_m{};
  double radius_m{};
  double clearance_margin_m{};
};

struct FeasibleTube {
  std::vector<FeasibleTubeSample> samples;
  double offset_scale{};
};

struct TrajectoryBoundaryState {
  Point3dEnu velocity_mps{};
  Point3dEnu acceleration_mps2{};
  double yaw_rate_rps{};
  double yaw_acceleration_rps2{};
};

struct TrajectorySmootherConfig {
  SafetyMargins safety_margins{0.0, 0.0, 0.0, 0.0, 0.0};
  double tube_offset_scale{0.5};
  double minimum_segment_duration_s{0.1};
  double maximum_speed_mps{1.0};
  double maximum_acceleration_mps2{1.0};
  double maximum_yaw_rate_rps{3.14159265358979323846};
  double maximum_yaw_acceleration_rps2{3.14159265358979323846};
  std::size_t maximum_segments{128U};
  std::size_t maximum_attempts{5U};
  std::uint64_t deadline_monotonic_ns{};
  std::function<std::uint64_t()> monotonic_now_ns;
  std::optional<TrajectoryBoundaryState> start_state;
  std::optional<TrajectoryBoundaryState> goal_state;
  std::function<std::vector<Point3dEnu>()> alternative_path_provider;
};

enum class TrajectorySmoothingStatus : std::uint8_t {
  success,
  invalid_input,
  tube_infeasible,
  smoothing_failed,
  timeout,
};

enum class TrajectorySmoothingFailureStage : std::uint8_t {
  none,
  build_tube,
  shrink_offset,
  add_control_points,
  extend_duration,
  constraints,
};

struct TrajectorySmoothingResult {
  TrajectorySmoothingStatus status{TrajectorySmoothingStatus::smoothing_failed};
  std::optional<BezierTrajectory4d> trajectory;
  std::optional<FeasibleTube> feasible_tube;
  std::size_t attempts{};
  TrajectorySmoothingFailureStage last_failure_stage{
      TrajectorySmoothingFailureStage::none};
  std::string detail;

  [[nodiscard]] bool has_value() const noexcept {
    return status == TrajectorySmoothingStatus::success && trajectory.has_value();
  }
};

class TrajectorySmoother final {
 public:
  [[nodiscard]] static TrajectorySmoothingResult smooth(
      const HybridMapQuery& map, const StateLatticePath& path,
      const TrajectorySmootherConfig& config = {},
      const std::string& frame_id = "mission_enu");

  [[nodiscard]] static TrajectorySmoothingResult smooth(
      const HybridMapQuery& map, const std::vector<Point3dEnu>& points,
      const TrajectorySmootherConfig& config = {},
      const std::string& frame_id = "mission_enu");

  [[nodiscard]] static MapQueryResult<FeasibleTube> build_feasible_tube(
      const HybridMapQuery& map, const std::vector<Point3dEnu>& points,
      const TrajectorySmootherConfig& config = {});
};

}  // namespace scout_planner::core
