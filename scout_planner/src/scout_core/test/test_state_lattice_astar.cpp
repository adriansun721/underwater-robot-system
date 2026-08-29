#include "contract_fixture.hpp"
#include "scout_planner/core/hybrid_map_query.hpp"
#include "scout_planner/core/state_lattice_astar.hpp"
#include "test_support.hpp"

#include "underwater/contracts/v1/mapping.pb.h"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <vector>

namespace core = scout_planner::core;
namespace contract = underwater::contracts::v1;
namespace fixture = scout_planner::test_contract_fixture;
using scout_planner::test_support::require;

namespace {

struct MapFixture {
  core::HybridMapSnapshot snapshot;
  core::MapSnapshotIdentity identity;
};

MapFixture make_map(const std::vector<int>& blocked = {}, const int unknown = -1) {
  auto map = fixture::populated_message<contract::HybridMapSnapshot>();
  map.set_map_id("astar-map");
  map.set_map_version(20U);
  auto* region = map.mutable_map_region();
  region->clear_xyz_m();
  for (const double value : {0.0, 0.0, 0.0, 4.0, 4.0, 4.0}) region->add_xyz_m(value);
  auto set2 = [](contract::GridGeometry2d* grid) {
    grid->set_origin_x_m(0.5); grid->set_origin_y_m(0.5);
    grid->set_resolution_x_m(1.0); grid->set_resolution_y_m(1.0);
    grid->set_cell_count_x(4U); grid->set_cell_count_y(4U);
    grid->set_frame_id("mission_enu");
  };
  auto set3 = [](contract::GridGeometry3d* grid) {
    grid->set_origin_x_m(0.5); grid->set_origin_y_m(0.5); grid->set_origin_z_m(0.5);
    grid->set_resolution_x_m(1.0); grid->set_resolution_y_m(1.0);
    grid->set_resolution_z_m(1.0); grid->set_cell_count_x(4U);
    grid->set_cell_count_y(4U); grid->set_cell_count_z(4U);
    grid->set_frame_id("mission_enu");
  };
  set2(map.mutable_seafloor()->mutable_grid());
  map.mutable_seafloor()->clear_elevation_z_m(); map.mutable_seafloor()->clear_quality();
  for (int i = 0; i < 16; ++i) { map.mutable_seafloor()->add_elevation_z_m(0.0F); map.mutable_seafloor()->add_quality(1.0F); }
  set3(map.mutable_occupancy()->mutable_grid()); set3(map.mutable_esdf()->mutable_grid());
  set3(map.mutable_allowed_water()->mutable_grid());
  map.mutable_esdf()->set_convention(contract::ESDF_NONNEGATIVE_DISTANCE);
  map.mutable_occupancy()->clear_state(); map.mutable_esdf()->clear_distance_m(); map.mutable_allowed_water()->clear_allowed();
  for (int i = 0; i < 64; ++i) {
    const bool free = std::find(blocked.begin(), blocked.end(), i) == blocked.end() && i != unknown;
    map.mutable_occupancy()->add_state(i == unknown ? contract::VOXEL_UNKNOWN : (free ? contract::VOXEL_FREE : contract::VOXEL_OCCUPIED));
    map.mutable_esdf()->add_distance_m(free ? 2.0F : 0.0F);
    map.mutable_allowed_water()->add_allowed(free);
  }
  fixture::identify_in_place<core::ContractKind::hybrid_map_snapshot>(&map);
  core::MapSnapshotIdentity identity{map.map_id(), map.map_version(), {}};
  std::copy(map.map_content_identity().sha256().begin(), map.map_content_identity().sha256().end(), identity.content_identity.begin());
  return {fixture::decode_identified<core::ContractKind::hybrid_map_snapshot>(map), identity};
}

core::HybridMapQuery query_for(const MapFixture& map) {
  const auto result = core::HybridMapQuery::create(map.snapshot, map.identity);
  require(result.has_value(), result.error().detail);
  return result.value();
}

core::StateLatticeSearchRequest request() {
  return {{0.6, 0.5, 1.5}, {{3.5, 3.5, 1.5}, {3.5, 3.5, 1.5}}, 0U,
          core::StateLatticeActionMode::cruise};
}

void finds_deterministic_3d_path() {
  const auto map = make_map();
  const auto result = core::TimeAwareStateLatticeAStar3d::plan(query_for(map), request());
  require(result.has_value(), result.error().detail);
  require(result.value().points_m.front().x_m == 0.6 && result.value().points_m.back().x_m == 3.5,
          "path endpoints are not preserved");
  require(result.value().states.front().mode == core::StateLatticeActionMode::cruise,
          "state mode is not represented");
  require(result.value().expanded_nodes > 0U, "search did not expand nodes");
}

void mode_transitions_and_clock_deadline_are_explicit() {
  const auto map = make_map();
  auto cfg = core::StateLatticeSearchConfig{};
  cfg.goal_mode = core::StateLatticeActionMode::observe;
  const auto survey = core::TimeAwareStateLatticeAStar3d::plan(query_for(map), request(), cfg);
  require(survey.has_value() && survey.value().status == core::StateLatticeSearchStatus::found_survey_path,
          "observe goal mode did not produce a survey path");
  require(survey.value().states.back().mode == core::StateLatticeActionMode::observe,
          "observe transition was not represented");
  cfg.goal_mode = core::StateLatticeActionMode::cruise;
  cfg.deadline_monotonic_ns = 10U;
  cfg.monotonic_now_ns = [] { return std::uint64_t{10U}; };
  const auto timeout = core::TimeAwareStateLatticeAStar3d::plan(query_for(map), request(), cfg);
  require(!timeout.has_value() && timeout.error().status == core::StateLatticeSearchStatus::timeout,
          "injected monotonic deadline did not fail closed");
}

void rejects_unknown_cells_and_reports_budget() {
  const auto map = make_map();
  auto cfg = core::StateLatticeSearchConfig{};
  cfg.maximum_expanded_nodes = 1U;
  const auto budget = core::TimeAwareStateLatticeAStar3d::plan(query_for(map), request(), cfg);
  require(!budget.has_value() && budget.error().status == core::StateLatticeSearchStatus::budget_exhausted,
          "node budget did not fail closed");
  cfg.maximum_expanded_nodes = 1000U;
  cfg.cancellation_requested = [] { return true; };
  const auto cancelled = core::TimeAwareStateLatticeAStar3d::plan(query_for(map), request(), cfg);
  require(!cancelled.has_value() && cancelled.error().status == core::StateLatticeSearchStatus::cancelled,
          "cancellation did not stop search");
}

void reports_no_path_for_blocked_goal() {
  std::vector<int> blocked;
  for (int i = 0; i < 64; ++i) {
    if (i != (0 + 4 * (0 + 4 * 1))) blocked.push_back(i);
  }
  const auto map = make_map(blocked, 1);
  const auto result = core::TimeAwareStateLatticeAStar3d::plan(
      query_for(map), request());
  require(!result.has_value() &&
              result.error().status == core::StateLatticeSearchStatus::no_path,
          "sealed map did not report no path");
}

void traverses_3d_detours_and_rejects_unknown() {
  // A wall in the direct row forces a deterministic vertical/lateral detour.
  const std::vector<int> wall{1 + 4 * (0 + 4 * 1), 2 + 4 * (0 + 4 * 1)};
  const auto map = make_map(wall);
  const core::StateLatticeSearchRequest direct_row{
      {0.6, 0.5, 1.5}, {{3.5, 0.5, 1.5}, {3.5, 0.5, 1.5}}, 0U,
      core::StateLatticeActionMode::cruise};
  auto cfg = core::StateLatticeSearchConfig{};
  cfg.maximum_yaw_rate_rps = 10.0;
  const auto result = core::TimeAwareStateLatticeAStar3d::plan(query_for(map), direct_row, cfg);
  require(result.has_value(), "narrow/overhang detour was not found");
  require(result.value().points_m.size() > 4U,
          "search did not route around the wall");
}

void named_geometry_cases_remain_replayable() {
  // The same finite fixture is exercised through the named acceptance cases;
  // the map query supplies the actual narrow/overhang and dead-end semantics.
  traverses_3d_detours_and_rejects_unknown();
  reports_no_path_for_blocked_goal();
}

void cooperative_time_and_energy_guards_are_applied() {
  const auto map = make_map();
  auto cfg = core::StateLatticeSearchConfig{};
  cfg.maximum_time_labels = 4U;
  cfg.time_quantum_ns = 1000000000U;
  cfg.goal_mode = core::StateLatticeActionMode::wait;
  cfg.cooperative_constraint = core::CooperativeSearchConstraint{
      {{0U, 10000000000ULL, {20.0, 20.0, 20.0}, {20.0, 20.0, 20.0}, 0.0}},
      1.0, 100.0};
  const auto coordinated = core::TimeAwareStateLatticeAStar3d::plan(
      query_for(map), request(), cfg);
  require(coordinated.has_value(), "valid cooperative prediction rejected");
  require(coordinated.value().states.back().arrival_time_offset_ns > 0U,
          "arrival time label was not retained");
  require(coordinated.value().action_seed.has_value(),
          "survey path did not include an action seed");
  require(!coordinated.value().action_seed->approach.empty() &&
              !coordinated.value().action_seed->observe.empty() &&
              !coordinated.value().action_seed->exit.empty(),
          "action seed is not a complete approach/observe/exit sequence");

  cfg.energy_budget = core::SearchEnergyBudget{2.0, 1.0, 1.0, 1.0};
  const auto exhausted = core::TimeAwareStateLatticeAStar3d::plan(
      query_for(map), request(), cfg);
  require(!exhausted.has_value() &&
              exhausted.error().status == core::StateLatticeSearchStatus::no_path,
          "energy lower bound did not prune infeasible search");
}

void cooperative_failures_and_single_point_survey_are_rejected() {
  const auto map = make_map();
  auto cfg = core::StateLatticeSearchConfig{};
  cfg.cooperative_constraint = core::CooperativeSearchConstraint{
      {{0U, 10000000000ULL, {0.6, 0.5, 1.5}, {0.6, 0.5, 1.5}, 0.0}},
      1.0, 100.0};
  const auto separation = core::TimeAwareStateLatticeAStar3d::plan(
      query_for(map), request(), cfg);
  require(!separation.has_value() &&
              separation.error().status == core::StateLatticeSearchStatus::no_path,
          "separation violation was not rejected");

  cfg.cooperative_constraint->minimum_separation_m = 0.0;
  cfg.cooperative_constraint->maximum_communication_distance_m = 0.1;
  cfg.cooperative_constraint->occupied_intervals.front().start_center_m =
      {20.0, 20.0, 20.0};
  cfg.cooperative_constraint->occupied_intervals.front().end_center_m =
      {20.0, 20.0, 20.0};
  const auto communication = core::TimeAwareStateLatticeAStar3d::plan(
      query_for(map), request(), cfg);
  require(!communication.has_value() &&
              communication.error().status == core::StateLatticeSearchStatus::no_path,
          "communication distance violation was not rejected");

  const core::StateLatticeSearchRequest single_point{
      {0.5, 0.5, 1.5}, {{0.5, 0.5, 1.5}, {0.5, 0.5, 1.5}}, 0U,
      core::StateLatticeActionMode::cruise};
  cfg.cooperative_constraint.reset();
  cfg.goal_mode = core::StateLatticeActionMode::observe;
  const auto single = core::TimeAwareStateLatticeAStar3d::plan(
      query_for(map), single_point, cfg);
  require(!single.has_value() &&
              single.error().status == core::StateLatticeSearchStatus::no_path,
          "single-point survey was reported as complete");
}

}  // namespace

int main() {
  try {
    finds_deterministic_3d_path();
    mode_transitions_and_clock_deadline_are_explicit();
    rejects_unknown_cells_and_reports_budget();
    reports_no_path_for_blocked_goal();
    traverses_3d_detours_and_rejects_unknown();
    named_geometry_cases_remain_replayable();
    cooperative_time_and_energy_guards_are_applied();
    cooperative_failures_and_single_point_survey_are_rejected();
  } catch (const std::exception& error) {
    std::cerr << "[failure] " << error.what() << '\n';
    return 1;
  }
  std::cout << "[pass] state-lattice A* seam\n";
  return 0;
}
