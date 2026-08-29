#pragma once

#include "scout_planner/core/hybrid_map_query.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace scout_planner::core {

enum class StateLatticeActionMode : std::uint8_t { cruise, observe, wait };

struct StateLatticeState {
  VoxelIndex3d voxel{};
  std::uint8_t heading_bin{};
  StateLatticeActionMode mode{StateLatticeActionMode::cruise};
  // Quantized Scout-local arrival time in nanoseconds. Zero is the initial label.
  std::uint64_t arrival_time_offset_ns{};

  [[nodiscard]] bool operator==(const StateLatticeState& other) const noexcept {
    return voxel == other.voxel && heading_bin == other.heading_bin &&
           mode == other.mode && arrival_time_offset_ns == other.arrival_time_offset_ns;
  }
};

struct CooperativeSearchInterval {
  std::uint64_t start_offset_ns{};
  std::uint64_t end_offset_ns{};
  Point3dEnu start_center_m{};
  Point3dEnu end_center_m{};
  double conservative_radius_m{};
};

struct CooperativeSearchConstraint {
  std::vector<CooperativeSearchInterval> occupied_intervals;
  double minimum_separation_m{};
  double maximum_communication_distance_m{};
};

struct SearchEnergyBudget {
  double available_energy_j{};
  double reserve_energy_j{};
  double return_energy_j{};
  // Conservative lower-bound power for every traversed/waiting edge.
  double lower_bound_power_w{};
};

struct StateLatticeSearchRequest {
  Point3dEnu start_m{};
  Aabb3dEnu goal_region{};
  std::uint8_t initial_heading_bin{};
  StateLatticeActionMode initial_mode{StateLatticeActionMode::cruise};
  std::optional<Aabb3dEnu> exit_region;
};

struct StateLatticeSearchConfig {
  std::uint8_t heading_bins{8U};
  double maximum_speed_mps{1.0};
  double maximum_yaw_rate_rps{3.14159265358979323846};
  double observe_duration_s{1.0};
  double wait_duration_s{1.0};
  StateLatticeActionMode goal_mode{StateLatticeActionMode::cruise};
  SafetyMargins safety_margins{0.0, 0.0, 0.0, 0.0, 0.0};
  std::size_t maximum_expanded_nodes{10000U};
  std::size_t maximum_open_queue{20000U};
  std::size_t maximum_memory_bytes{16U * 1024U * 1024U};
  std::size_t deadline_expansions{0U};
  std::uint64_t deadline_monotonic_ns{};
  std::function<std::uint64_t()> monotonic_now_ns;
  std::function<bool()> cancellation_requested;
  std::uint64_t time_quantum_ns{100000000U};
  std::size_t maximum_time_labels{1U};
  std::optional<CooperativeSearchConstraint> cooperative_constraint;
  std::optional<SearchEnergyBudget> energy_budget;
  bool allow_wait_transitions{false};
  double distance_cost_weight{1.0};
  double time_cost_weight{1.0};
  double energy_cost_weight{0.0};
};

struct StateLatticeActionSeed {
  std::vector<Point3dEnu> approach;
  std::vector<Point3dEnu> observe;
  std::vector<Point3dEnu> exit;
  double observe_duration_s{};
};

enum class StateLatticeSearchStatus : std::uint8_t {
  found_path,
  found_survey_path,
  no_path,
  timeout,
  cancelled,
  dependency_invalid,
  budget_exhausted,
};

struct StateLatticeSearchError {
  StateLatticeSearchStatus status{StateLatticeSearchStatus::no_path};
  std::size_t expanded_nodes{};
  std::string detail;
};

struct StateLatticePath {
  StateLatticeSearchStatus status{StateLatticeSearchStatus::found_path};
  std::vector<StateLatticeState> states;
  std::vector<Point3dEnu> points_m;
  double cost{};
  double duration_s{};
  std::size_t expanded_nodes{};
  std::optional<StateLatticeActionSeed> action_seed;
};

class StateLatticeSearchResult final {
 public:
  static StateLatticeSearchResult success(StateLatticePath path);
  static StateLatticeSearchResult failure(StateLatticeSearchError error);
  [[nodiscard]] bool has_value() const noexcept;
  [[nodiscard]] const StateLatticePath& value() const;
  [[nodiscard]] const StateLatticeSearchError& error() const;

 private:
  explicit StateLatticeSearchResult(StateLatticePath path);
  explicit StateLatticeSearchResult(StateLatticeSearchError error);
  std::variant<StateLatticePath, StateLatticeSearchError> storage_;
};

class TimeAwareStateLatticeAStar3d final {
 public:
  [[nodiscard]] static StateLatticeSearchResult plan(
      const HybridMapQuery& map, const StateLatticeSearchRequest& request,
      const StateLatticeSearchConfig& config = {});
};

}  // namespace scout_planner::core
