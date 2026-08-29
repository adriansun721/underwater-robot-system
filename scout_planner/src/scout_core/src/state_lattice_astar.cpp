#include "scout_planner/core/state_lattice_astar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace scout_planner::core {
namespace {

constexpr double kPi = 3.14159265358979323846;

bool finite_point(const Point3dEnu p) {
  return std::isfinite(p.x_m) && std::isfinite(p.y_m) &&
         std::isfinite(p.z_m);
}

bool valid_region(const Aabb3dEnu& region) {
  return finite_point(region.minimum_m) && finite_point(region.maximum_m) &&
         region.minimum_m.x_m <= region.maximum_m.x_m &&
         region.minimum_m.y_m <= region.maximum_m.y_m &&
         region.minimum_m.z_m <= region.maximum_m.z_m;
}

bool valid_margins(const SafetyMargins& margins) {
  const std::array values{margins.body_m, margins.localization_m,
                           margins.tracking_m, margins.map_m,
                           margins.discretization_m};
  return std::all_of(values.begin(), values.end(), [](const double value) {
    return std::isfinite(value) && value >= 0.0;
  });
}

bool valid_mode(const StateLatticeActionMode mode) {
  return mode == StateLatticeActionMode::cruise ||
         mode == StateLatticeActionMode::observe ||
         mode == StateLatticeActionMode::wait;
}

int mode_rank(const StateLatticeActionMode mode) {
  return static_cast<int>(mode);
}

std::uint64_t key_for(const StateLatticeState& state,
                      const std::array<std::uint32_t, 3U>& counts,
                      const std::uint8_t heading_bins,
                      const std::size_t time_labels,
                      const std::uint64_t time_quantum_ns) {
  const auto cell = static_cast<std::uint64_t>(state.voxel.x) +
                    static_cast<std::uint64_t>(counts[0]) *
                        (static_cast<std::uint64_t>(state.voxel.y) +
                         static_cast<std::uint64_t>(counts[1]) *
                             state.voxel.z);
  const auto label = time_quantum_ns == 0U
                         ? 0U
                         : std::min<std::uint64_t>(
                               static_cast<std::uint64_t>(time_labels - 1U),
                               state.arrival_time_offset_ns / time_quantum_ns);
  return ((((cell * heading_bins) + state.heading_bin) * 3U) +
          static_cast<std::uint8_t>(state.mode)) * time_labels +
         label;
}

bool valid_cooperative(const CooperativeSearchConstraint& constraint) {
  if (!std::isfinite(constraint.minimum_separation_m) ||
      !std::isfinite(constraint.maximum_communication_distance_m) ||
      constraint.minimum_separation_m < 0.0 ||
      constraint.maximum_communication_distance_m < constraint.minimum_separation_m ||
      constraint.occupied_intervals.empty()) {
    return false;
  }
  std::uint64_t previous_end = 0U;
  for (std::size_t index = 0U; index < constraint.occupied_intervals.size(); ++index) {
    const auto& interval = constraint.occupied_intervals[index];
    if (interval.end_offset_ns < interval.start_offset_ns ||
        (index == 0U && interval.start_offset_ns != 0U) ||
        (index != 0U && interval.start_offset_ns != previous_end) ||
        !finite_point(interval.start_center_m) || !finite_point(interval.end_center_m) ||
        !std::isfinite(interval.conservative_radius_m) ||
        interval.conservative_radius_m < 0.0) {
      return false;
    }
    if (index != 0U) {
      const auto& previous = constraint.occupied_intervals[index - 1U];
      const auto close = [](const double left, const double right) {
        return std::abs(left - right) <= 1e-9;
      };
      if (!close(previous.end_center_m.x_m, interval.start_center_m.x_m) ||
          !close(previous.end_center_m.y_m, interval.start_center_m.y_m) ||
          !close(previous.end_center_m.z_m, interval.start_center_m.z_m)) {
        return false;
      }
    }
    previous_end = interval.end_offset_ns;
  }
  return true;
}

bool cooperative_edge_valid(const CooperativeSearchConstraint& constraint,
                            const Point3dEnu start, const Point3dEnu end,
                            const std::uint64_t start_ns,
                            const std::uint64_t end_ns) {
  if (end_ns < start_ns) return false;
  const std::array<double, 9U> fractions{0.0, 0.125, 0.25, 0.375, 0.5,
                                         0.625, 0.75, 0.875, 1.0};
  for (const double fraction : fractions) {
    const auto tick = start_ns + static_cast<std::uint64_t>(
        static_cast<long double>(end_ns - start_ns) * fraction);
    const auto scout = Point3dEnu{
        start.x_m + (end.x_m - start.x_m) * fraction,
        start.y_m + (end.y_m - start.y_m) * fraction,
        start.z_m + (end.z_m - start.z_m) * fraction};
    const auto found = std::find_if(
        constraint.occupied_intervals.begin(), constraint.occupied_intervals.end(),
        [tick](const CooperativeSearchInterval& interval) {
          return tick >= interval.start_offset_ns && tick <= interval.end_offset_ns;
        });
    if (found == constraint.occupied_intervals.end()) return false;
    const double alpha = found->end_offset_ns == found->start_offset_ns
                             ? 0.0
                             : static_cast<double>(tick - found->start_offset_ns) /
                                   static_cast<double>(found->end_offset_ns - found->start_offset_ns);
    const Point3dEnu main{
        found->start_center_m.x_m + (found->end_center_m.x_m - found->start_center_m.x_m) * alpha,
        found->start_center_m.y_m + (found->end_center_m.y_m - found->start_center_m.y_m) * alpha,
        found->start_center_m.z_m + (found->end_center_m.z_m - found->start_center_m.z_m) * alpha};
    const double dx = scout.x_m - main.x_m;
    const double dy = scout.y_m - main.y_m;
    const double dz = scout.z_m - main.z_m;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (distance - found->conservative_radius_m < constraint.minimum_separation_m ||
        distance > constraint.maximum_communication_distance_m) return false;
  }
  return true;
}

Point3dEnu center(const MapGridInfo& grid, const VoxelIndex3d voxel) {
  return {grid.origin_m.x_m + grid.resolution_m.x_m * voxel.x,
          grid.origin_m.y_m + grid.resolution_m.y_m * voxel.y,
          grid.origin_m.z_m + grid.resolution_m.z_m * voxel.z};
}

std::optional<VoxelIndex3d> voxel_for(const MapGridInfo& grid,
                                      const Point3dEnu point) {
  const auto to_index = [](const double value, const double origin,
                           const double resolution) -> std::optional<std::uint32_t> {
    const auto index = std::llround((value - origin) / resolution);
    if (index < 0 || index > std::numeric_limits<std::uint32_t>::max()) {
      return std::nullopt;
    }
    return static_cast<std::uint32_t>(index);
  };
  const auto x = to_index(point.x_m, grid.origin_m.x_m, grid.resolution_m.x_m);
  const auto y = to_index(point.y_m, grid.origin_m.y_m, grid.resolution_m.y_m);
  const auto z = to_index(point.z_m, grid.origin_m.z_m, grid.resolution_m.z_m);
  if (!x.has_value() || !y.has_value() || !z.has_value() ||
      *x >= grid.cell_count[0] || *y >= grid.cell_count[1] ||
      *z >= grid.cell_count[2]) {
    return std::nullopt;
  }
  return VoxelIndex3d{*x, *y, *z};
}

bool in_goal(const Point3dEnu point, const Aabb3dEnu& goal) {
  return point.x_m >= goal.minimum_m.x_m && point.x_m <= goal.maximum_m.x_m &&
         point.y_m >= goal.minimum_m.y_m && point.y_m <= goal.maximum_m.y_m &&
         point.z_m >= goal.minimum_m.z_m && point.z_m <= goal.maximum_m.z_m;
}

double distance_to_goal(const Point3dEnu point, const Aabb3dEnu& goal) {
  const auto axis = [](const double value, const double low,
                       const double high) {
    return value < low ? low - value : (value > high ? value - high : 0.0);
  };
  const double dx = axis(point.x_m, goal.minimum_m.x_m, goal.maximum_m.x_m);
  const double dy = axis(point.y_m, goal.minimum_m.y_m, goal.maximum_m.y_m);
  const double dz = axis(point.z_m, goal.minimum_m.z_m, goal.maximum_m.z_m);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::uint8_t heading_for(const int dx, const int dy,
                         const std::uint8_t bins,
                         const std::uint8_t fallback) {
  if (dx == 0 && dy == 0) return fallback;
  double angle = std::atan2(static_cast<double>(dy), static_cast<double>(dx));
  if (angle < 0.0) angle += 2.0 * kPi;
  auto value = static_cast<int>(std::floor(angle * bins / (2.0 * kPi) + 0.5));
  value %= bins;
  return static_cast<std::uint8_t>(value);
}

double heading_delta(const std::uint8_t from, const std::uint8_t to,
                     const std::uint8_t bins) {
  const int difference = std::abs(static_cast<int>(to) -
                                  static_cast<int>(from));
  const int wrapped = std::min(difference, static_cast<int>(bins) - difference);
  return 2.0 * kPi * static_cast<double>(wrapped) / bins;
}

}  // namespace

StateLatticeSearchResult::StateLatticeSearchResult(StateLatticePath path)
    : storage_(std::move(path)) {}

StateLatticeSearchResult::StateLatticeSearchResult(
    StateLatticeSearchError error)
    : storage_(std::move(error)) {}

StateLatticeSearchResult StateLatticeSearchResult::success(
    StateLatticePath path) {
  return StateLatticeSearchResult(std::move(path));
}

StateLatticeSearchResult StateLatticeSearchResult::failure(
    StateLatticeSearchError error) {
  return StateLatticeSearchResult(std::move(error));
}

bool StateLatticeSearchResult::has_value() const noexcept {
  return std::holds_alternative<StateLatticePath>(storage_);
}

const StateLatticePath& StateLatticeSearchResult::value() const {
  return std::get<StateLatticePath>(storage_);
}

const StateLatticeSearchError& StateLatticeSearchResult::error() const {
  static const StateLatticeSearchError empty{};
  const auto* error = std::get_if<StateLatticeSearchError>(&storage_);
  return error == nullptr ? empty : *error;
}

StateLatticeSearchResult TimeAwareStateLatticeAStar3d::plan(
    const HybridMapQuery& map, const StateLatticeSearchRequest& request,
    const StateLatticeSearchConfig& config) {
  const auto fail = [](const StateLatticeSearchStatus status,
                       const std::string& detail, const std::size_t expanded = 0U) {
    return StateLatticeSearchResult::failure({status, expanded, detail});
  };
  if (!finite_point(request.start_m) || !valid_region(request.goal_region) ||
      !valid_margins(config.safety_margins) || config.heading_bins == 0U ||
      config.heading_bins > 64U || !std::isfinite(config.maximum_speed_mps) ||
      config.maximum_speed_mps <= 0.0 ||
      !std::isfinite(config.maximum_yaw_rate_rps) ||
      config.maximum_yaw_rate_rps <= 0.0 ||
      !std::isfinite(config.observe_duration_s) ||
      config.observe_duration_s <= 0.0 || !std::isfinite(config.wait_duration_s) ||
      config.wait_duration_s <= 0.0 || config.maximum_expanded_nodes == 0U ||
      config.maximum_open_queue == 0U || config.maximum_memory_bytes == 0U) {
    return fail(StateLatticeSearchStatus::dependency_invalid,
                "state-lattice request or limits are invalid");
  }
  if (request.exit_region.has_value() && !valid_region(request.exit_region.value())) {
    return fail(StateLatticeSearchStatus::dependency_invalid,
                "exit region is invalid");
  }
  if (!std::isfinite(config.distance_cost_weight) ||
      !std::isfinite(config.time_cost_weight) ||
      !std::isfinite(config.energy_cost_weight) ||
      config.distance_cost_weight < 0.0 || config.time_cost_weight < 0.0 ||
      config.energy_cost_weight < 0.0 ||
      (config.distance_cost_weight == 0.0 && config.time_cost_weight == 0.0 &&
       config.energy_cost_weight == 0.0)) {
    return fail(StateLatticeSearchStatus::dependency_invalid,
                "search cost weights are invalid");
  }
  if (!valid_mode(request.initial_mode) || !valid_mode(config.goal_mode)) {
    return fail(StateLatticeSearchStatus::dependency_invalid,
                "state-lattice action mode is invalid");
  }
  if (mode_rank(config.goal_mode) < mode_rank(request.initial_mode)) {
    return fail(StateLatticeSearchStatus::dependency_invalid,
                "action mode cannot transition backwards");
  }
  if (config.maximum_time_labels == 0U ||
      (config.maximum_time_labels > 1U && config.time_quantum_ns == 0U) ||
      (config.cooperative_constraint.has_value() &&
       !valid_cooperative(config.cooperative_constraint.value()))) {
    return fail(StateLatticeSearchStatus::dependency_invalid,
                "time labels or cooperative constraints are invalid");
  }
  if (config.energy_budget.has_value()) {
    const auto& budget = config.energy_budget.value();
    if (!std::isfinite(budget.available_energy_j) ||
        !std::isfinite(budget.reserve_energy_j) ||
        !std::isfinite(budget.return_energy_j) ||
        !std::isfinite(budget.lower_bound_power_w) ||
        budget.available_energy_j < 0.0 || budget.reserve_energy_j < 0.0 ||
        budget.return_energy_j < 0.0 || budget.lower_bound_power_w < 0.0 ||
        budget.available_energy_j < budget.reserve_energy_j + budget.return_energy_j) {
      return fail(StateLatticeSearchStatus::dependency_invalid,
                  "energy budget is invalid");
    }
  }
  if (config.deadline_monotonic_ns != 0U && !config.monotonic_now_ns) {
    return fail(StateLatticeSearchStatus::dependency_invalid,
                "a monotonic clock is required for a time deadline");
  }
  const auto& grid = map.grid_info();
  const auto start_voxel = voxel_for(grid, request.start_m);
  if (!start_voxel.has_value() ||
      request.initial_heading_bin >= config.heading_bins) {
    return fail(StateLatticeSearchStatus::dependency_invalid,
                "start is outside the map grid or heading is invalid");
  }
  const auto start_sample = map.query_point(request.start_m, config.safety_margins);
  if (!start_sample.has_value() || start_sample.value().state != MapCellState::free ||
      !start_sample.value().allowed_water ||
      start_sample.value().clearance_margin_m < 0.0) {
    return fail(StateLatticeSearchStatus::no_path,
                "start cell is not known free and traversable");
  }
  if (config.cooperative_constraint.has_value() &&
      !cooperative_edge_valid(config.cooperative_constraint.value(), request.start_m,
                              request.start_m, 0U, 0U)) {
    return fail(StateLatticeSearchStatus::no_path,
                "start violates cooperative separation or communication bounds");
  }

  struct Record {
    StateLatticeState state;
    Point3dEnu point;
    double g{};
    double duration{};
    double energy_used_j{};
    std::size_t parent{std::numeric_limits<std::size_t>::max()};
  };
  struct QueueEntry {
    double f;
    double g;
    std::uint64_t serial;
    std::size_t record;
  };
  struct QueueCompare {
    bool operator()(const QueueEntry& left, const QueueEntry& right) const {
      if (left.f != right.f) return left.f > right.f;
      if (left.g != right.g) return left.g > right.g;
      return left.serial > right.serial;
    }
  };

  std::uint64_t total_states_u64 = 1U;
  for (const auto factor : {static_cast<std::uint64_t>(grid.cell_count[0]),
                            static_cast<std::uint64_t>(grid.cell_count[1]),
                            static_cast<std::uint64_t>(grid.cell_count[2]),
                            static_cast<std::uint64_t>(config.heading_bins),
                            std::uint64_t{3U},
                            static_cast<std::uint64_t>(config.maximum_time_labels)}) {
    if (factor != 0U &&
        total_states_u64 > std::numeric_limits<std::uint64_t>::max() / factor) {
      return fail(StateLatticeSearchStatus::budget_exhausted,
                  "state index exceeds addressable memory");
    }
    total_states_u64 *= factor;
  }
  if (total_states_u64 > std::numeric_limits<std::size_t>::max()) {
    return fail(StateLatticeSearchStatus::budget_exhausted,
                "state index exceeds addressable memory");
  }
  const auto total_states = static_cast<std::size_t>(total_states_u64);
  const auto record_capacity =
      std::min(config.maximum_expanded_nodes, total_states);
  const auto open_capacity = config.maximum_open_queue;
  const long double required_memory =
      static_cast<long double>(total_states) * sizeof(std::size_t) +
      static_cast<long double>(record_capacity) * sizeof(Record) +
      static_cast<long double>(open_capacity) * sizeof(QueueEntry);
  if (required_memory >
      static_cast<long double>(config.maximum_memory_bytes)) {
    return fail(StateLatticeSearchStatus::budget_exhausted,
                "configured state, node, and queue caps exceed memory budget");
  }
  std::vector<Record> records;
  records.reserve(record_capacity);
  const auto no_record = std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> best(total_states, no_record);
  std::vector<QueueEntry> open_storage;
  open_storage.reserve(open_capacity);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> open(
      QueueCompare{}, std::move(open_storage));
  const StateLatticeState initial{*start_voxel, request.initial_heading_bin,
                                  request.initial_mode, 0U};
  records.push_back({initial, request.start_m, 0.0, 0.0, 0.0});
  best.at(static_cast<std::size_t>(
      key_for(initial, grid.cell_count, config.heading_bins,
              config.maximum_time_labels, config.time_quantum_ns))) = 0U;
  open.push({distance_to_goal(request.start_m, request.goal_region) /
                 config.maximum_speed_mps,
             0.0, 0U, 0U});
  std::uint64_t serial = 1U;
  std::size_t expanded = 0U;
  while (!open.empty()) {
    if (config.cancellation_requested && config.cancellation_requested()) {
      return fail(StateLatticeSearchStatus::cancelled, "search cancelled", expanded);
    }
    if (config.deadline_expansions != 0U &&
        expanded >= config.deadline_expansions) {
      return fail(StateLatticeSearchStatus::timeout, "search deadline reached", expanded);
    }
    if (config.deadline_monotonic_ns != 0U &&
        config.monotonic_now_ns() >= config.deadline_monotonic_ns) {
      return fail(StateLatticeSearchStatus::timeout, "search deadline reached", expanded);
    }
    if (expanded >= config.maximum_expanded_nodes) {
      return fail(StateLatticeSearchStatus::budget_exhausted,
                  "search node or memory budget exhausted", expanded);
    }
    const auto current_entry = open.top();
    open.pop();
    const auto current_key = static_cast<std::size_t>(key_for(
        records[current_entry.record].state, grid.cell_count,
        config.heading_bins, config.maximum_time_labels, config.time_quantum_ns));
    if (best.at(current_key) != current_entry.record ||
        records[current_entry.record].g != current_entry.g) {
      continue;
    }
    const auto& current = records[current_entry.record];
    const auto current_point = current.point;
    ++expanded;
    const bool at_goal = in_goal(current_point, request.goal_region);
    const bool exit_required = request.exit_region.has_value();
    const auto terminal_mode = exit_required ? StateLatticeActionMode::wait
                                             : config.goal_mode;
    const bool at_terminal = current.state.mode == terminal_mode &&
                             (!exit_required ||
                              in_goal(current_point, request.exit_region.value()));
    if ((exit_required ? in_goal(current_point, request.exit_region.value()) : at_goal) &&
        at_terminal) {
      StateLatticePath path;
      path.status = current.state.mode == StateLatticeActionMode::cruise
                        ? StateLatticeSearchStatus::found_path
                        : StateLatticeSearchStatus::found_survey_path;
      path.cost = current.g;
      path.duration_s = current.duration;
      path.expanded_nodes = expanded;
      for (auto index = current_entry.record;
           index != std::numeric_limits<std::size_t>::max();
           index = records[index].parent) {
        path.states.push_back(records[index].state);
        path.points_m.push_back(records[index].point);
      }
      std::reverse(path.states.begin(), path.states.end());
      std::reverse(path.points_m.begin(), path.points_m.end());
      if (path.status == StateLatticeSearchStatus::found_survey_path) {
        StateLatticeActionSeed seed;
        bool observing = false;
        for (std::size_t index = 0U; index < path.points_m.size(); ++index) {
          const auto mode = path.states[index].mode;
          if (mode == StateLatticeActionMode::observe) {
            observing = true;
            seed.observe.push_back(path.points_m[index]);
          } else if (!observing) {
            seed.approach.push_back(path.points_m[index]);
          } else {
            seed.exit.push_back(path.points_m[index]);
          }
        }
        if (seed.approach.empty()) seed.approach.push_back(path.points_m.front());
        if (seed.observe.empty()) seed.observe.push_back(path.points_m.back());
        if (seed.exit.empty()) seed.exit.push_back(path.points_m.back());
        seed.observe_duration_s = config.observe_duration_s;
        const auto has_motion = [](const std::vector<Point3dEnu>& points) {
          for (std::size_t index = 1U; index < points.size(); ++index) {
            const auto& left = points[index - 1U];
            const auto& right = points[index];
            if (left.x_m != right.x_m || left.y_m != right.y_m ||
                left.z_m != right.z_m) return true;
          }
          return false;
        };
        if (!has_motion(seed.approach) ||
            (request.exit_region.has_value() && !has_motion(seed.exit))) {
          return fail(StateLatticeSearchStatus::no_path,
                      "survey action requires distinct approach and exit motion", expanded);
        }
        if (request.exit_region.has_value() && seed.exit.size() < 2U) {
          return fail(StateLatticeSearchStatus::no_path,
                      "survey action has no distinct exit segment", expanded);
        }
        path.action_seed = std::move(seed);
      }
      return StateLatticeSearchResult::success(std::move(path));
    }

    // Satisfy a requested terminal action mode before exploring unrelated edges.
    if (at_goal && (current.state.mode != config.goal_mode || exit_required) &&
        current.state.mode != StateLatticeActionMode::wait) {
      const auto next_mode = current.state.mode == StateLatticeActionMode::cruise
                                 ? StateLatticeActionMode::observe
                                 : StateLatticeActionMode::wait;
      const double mode_duration = next_mode == StateLatticeActionMode::observe
                                       ? config.observe_duration_s
                                       : config.wait_duration_s;
      StateLatticeState next{current.state.voxel,
                             current.state.heading_bin, next_mode};
      const double candidate_g = current.g + mode_duration;
      const double candidate_energy = current.energy_used_j +
                                      (config.energy_budget.has_value()
                                           ? config.energy_budget->lower_bound_power_w * mode_duration
                                           : 0.0);
      if (config.cooperative_constraint.has_value() &&
          !cooperative_edge_valid(config.cooperative_constraint.value(), current_point,
                                  current_point,
                                  static_cast<std::uint64_t>(current.duration * 1e9),
                                  static_cast<std::uint64_t>((current.duration + mode_duration) * 1e9))) {
        continue;
      }
      if (config.energy_budget.has_value() &&
          candidate_energy + config.energy_budget->return_energy_j +
                  config.energy_budget->reserve_energy_j >
              config.energy_budget->available_energy_j) {
        continue;
      }
      const auto end_ns = static_cast<std::uint64_t>((current.duration + mode_duration) * 1e9);
      next.arrival_time_offset_ns = end_ns;
      const auto key = static_cast<std::size_t>(key_for(
          next, grid.cell_count, config.heading_bins, config.maximum_time_labels,
          config.time_quantum_ns));
      const auto found = best.at(key);
      if (found == no_record || records[found].g > candidate_g) {
        if (open.size() >= config.maximum_open_queue ||
            records.size() >= record_capacity) {
          return fail(StateLatticeSearchStatus::budget_exhausted,
                      "mode transition exceeded search budget", expanded);
        }
        records.push_back({next, current_point, candidate_g,
                           current.duration + mode_duration, candidate_energy,
                           current_entry.record});
        const auto index = records.size() - 1U;
        best.at(key) = index;
        open.push({candidate_g, candidate_g, serial++, index});
      }
      continue;
    }

    if (config.allow_wait_transitions &&
        current.state.mode != StateLatticeActionMode::observe) {
      const double mode_duration = config.wait_duration_s;
      const auto start_ns = static_cast<std::uint64_t>(current.duration * 1e9);
      const auto end_ns = static_cast<std::uint64_t>((current.duration + mode_duration) * 1e9);
      if (!config.cooperative_constraint.has_value() ||
          cooperative_edge_valid(config.cooperative_constraint.value(), current_point,
                                 current_point, start_ns, end_ns)) {
        const double candidate_energy = current.energy_used_j +
                                        (config.energy_budget.has_value()
                                             ? config.energy_budget->lower_bound_power_w * mode_duration
                                             : 0.0);
        if (!config.energy_budget.has_value() ||
            candidate_energy + config.energy_budget->return_energy_j +
                    config.energy_budget->reserve_energy_j <=
                config.energy_budget->available_energy_j) {
          StateLatticeState waited{current.state.voxel, current.state.heading_bin,
                                   StateLatticeActionMode::wait, 0U};
          waited.arrival_time_offset_ns = end_ns;
          const auto wait_key = static_cast<std::size_t>(key_for(
              waited, grid.cell_count, config.heading_bins, config.maximum_time_labels,
              config.time_quantum_ns));
          const auto found = best.at(wait_key);
          if ((found == no_record || records[found].g > current.g + mode_duration) &&
              open.size() < config.maximum_open_queue && records.size() < record_capacity) {
            records.push_back({waited, current_point, current.g + mode_duration,
                               current.duration + mode_duration, candidate_energy,
                               current_entry.record});
            const auto index = records.size() - 1U;
            best.at(wait_key) = index;
            open.push({current.g + mode_duration +
                           distance_to_goal(current_point, request.goal_region) /
                               config.maximum_speed_mps,
                       current.g + mode_duration, serial++, index});
          }
        }
      }
    }

    for (int dz = -1; dz <= 1; ++dz) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0 && dz == 0) continue;
          const auto next_x = static_cast<int>(current.state.voxel.x) + dx;
          const auto next_y = static_cast<int>(current.state.voxel.y) + dy;
          const auto next_z = static_cast<int>(current.state.voxel.z) + dz;
          if (next_x < 0 || next_y < 0 || next_z < 0 ||
              next_x >= static_cast<int>(grid.cell_count[0]) ||
              next_y >= static_cast<int>(grid.cell_count[1]) ||
              next_z >= static_cast<int>(grid.cell_count[2])) continue;
          const auto next_voxel = VoxelIndex3d{
              static_cast<std::uint32_t>(next_x), static_cast<std::uint32_t>(next_y),
              static_cast<std::uint32_t>(next_z)};
          const auto next_point = center(grid, next_voxel);
          const double distance = std::sqrt(
              std::pow(dx * grid.resolution_m.x_m, 2.0) +
              std::pow(dy * grid.resolution_m.y_m, 2.0) +
              std::pow(dz * grid.resolution_m.z_m, 2.0));
          const double duration = distance / config.maximum_speed_mps;
          const auto next_heading = heading_for(dx, dy, config.heading_bins,
                                                current.state.heading_bin);
          if (heading_delta(current.state.heading_bin, next_heading,
                            config.heading_bins) /
                  duration >
              config.maximum_yaw_rate_rps) continue;
          const auto edge = map.query_supercover(current_point, next_point,
                                                 config.safety_margins);
          if (!edge.has_value() || edge.value().state != MapCellState::free ||
              !edge.value().allowed_water || edge.value().clearance_margin_m < 0.0) {
            continue;
          }
          const auto start_ns = static_cast<std::uint64_t>(current.duration * 1e9);
          const auto end_ns = static_cast<std::uint64_t>((current.duration + duration) * 1e9);
          if (config.cooperative_constraint.has_value() &&
              !cooperative_edge_valid(config.cooperative_constraint.value(), current_point,
                                      next_point, start_ns, end_ns)) {
            continue;
          }
          const double edge_energy = config.energy_budget.has_value()
                                         ? config.energy_budget->lower_bound_power_w * duration
                                         : 0.0;
          const double candidate_energy = current.energy_used_j + edge_energy;
          if (config.energy_budget.has_value() &&
              candidate_energy + config.energy_budget->return_energy_j +
                      config.energy_budget->reserve_energy_j >
                  config.energy_budget->available_energy_j) {
            continue;
          }
          const StateLatticeState next{next_voxel, next_heading,
                                       current.state.mode == StateLatticeActionMode::wait &&
                                               !request.exit_region.has_value()
                                           ? StateLatticeActionMode::cruise
                                           : current.state.mode,
                                       end_ns};
          const auto key = key_for(next, grid.cell_count, config.heading_bins,
                                   config.maximum_time_labels, config.time_quantum_ns);
          const double candidate_g = current.g +
                                     config.distance_cost_weight * distance +
                                     config.time_cost_weight * duration +
                                     config.energy_cost_weight * edge_energy;
          const auto key_index = static_cast<std::size_t>(key);
          const auto found = best.at(key_index);
          if (found != no_record && records[found].g <= candidate_g) {
            continue;
          }
          if (open.size() >= config.maximum_open_queue ||
              records.size() >= config.maximum_expanded_nodes) {
            return fail(StateLatticeSearchStatus::budget_exhausted,
                        "open queue budget exhausted", expanded);
          }
          records.push_back({next, next_point, candidate_g,
                             current.duration + duration, candidate_energy,
                             current_entry.record});
          const auto index = records.size() - 1U;
          best.at(key_index) = index;
          open.push({candidate_g + distance_to_goal(next_point, request.goal_region) /
                                  config.maximum_speed_mps,
                     candidate_g, serial++, index});
        }
      }
    }
  }
  return fail(StateLatticeSearchStatus::no_path, "no traversable lattice path",
              expanded);
}

}  // namespace scout_planner::core
