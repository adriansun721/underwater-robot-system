#include "contract_fixture.hpp"
#include "scout_planner/core/trajectory_smoother.hpp"
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

MapFixture make_map(const int unknown = -1) {
  auto map = fixture::populated_message<contract::HybridMapSnapshot>();
  map.set_map_id("smoother-map"); map.set_map_version(1U);
  auto* region = map.mutable_map_region(); region->clear_xyz_m();
  for (const double value : {0.0, 0.0, 0.0, 4.0, 4.0, 4.0}) region->add_xyz_m(value);
  auto set2 = [](contract::GridGeometry2d* grid) {
    grid->set_origin_x_m(0.5); grid->set_origin_y_m(0.5);
    grid->set_resolution_x_m(1.0); grid->set_resolution_y_m(1.0);
    grid->set_cell_count_x(4U); grid->set_cell_count_y(4U); grid->set_frame_id("mission_enu");
  };
  auto set3 = [](contract::GridGeometry3d* grid) {
    grid->set_origin_x_m(0.5); grid->set_origin_y_m(0.5); grid->set_origin_z_m(0.5);
    grid->set_resolution_x_m(1.0); grid->set_resolution_y_m(1.0); grid->set_resolution_z_m(1.0);
    grid->set_cell_count_x(4U); grid->set_cell_count_y(4U); grid->set_cell_count_z(4U);
    grid->set_frame_id("mission_enu");
  };
  set2(map.mutable_seafloor()->mutable_grid()); map.mutable_seafloor()->clear_elevation_z_m(); map.mutable_seafloor()->clear_quality();
  for (int i = 0; i < 16; ++i) { map.mutable_seafloor()->add_elevation_z_m(0.0F); map.mutable_seafloor()->add_quality(1.0F); }
  set3(map.mutable_occupancy()->mutable_grid()); set3(map.mutable_esdf()->mutable_grid()); set3(map.mutable_allowed_water()->mutable_grid());
  map.mutable_esdf()->set_convention(contract::ESDF_NONNEGATIVE_DISTANCE); map.mutable_occupancy()->clear_state(); map.mutable_esdf()->clear_distance_m(); map.mutable_allowed_water()->clear_allowed();
  for (int i = 0; i < 64; ++i) { map.mutable_occupancy()->add_state(i == unknown ? contract::VOXEL_UNKNOWN : contract::VOXEL_FREE); map.mutable_esdf()->add_distance_m(2.0F); map.mutable_allowed_water()->add_allowed(true); }
  fixture::identify_in_place<core::ContractKind::hybrid_map_snapshot>(&map);
  core::MapSnapshotIdentity identity{map.map_id(), map.map_version(), {}};
  std::copy(map.map_content_identity().sha256().begin(), map.map_content_identity().sha256().end(), identity.content_identity.begin());
  return {fixture::decode_identified<core::ContractKind::hybrid_map_snapshot>(map), identity};
}

core::HybridMapQuery query_for(const MapFixture& map) {
  const auto result = core::HybridMapQuery::create(map.snapshot, map.identity);
  require(result.has_value(), result.error().detail); return result.value();
}

void straight_path_and_c2() {
  const auto map = make_map();
  const std::vector<core::Point3dEnu> points{{1.2, 1.2, 1.2}, {1.8, 1.8, 1.5}, {2.4, 2.4, 1.8}};
  auto config = core::TrajectorySmootherConfig{}; config.maximum_speed_mps = 10.0; config.maximum_acceleration_mps2 = 1000.0;
  const auto result = core::TrajectorySmoother::smooth(query_for(map), points, config);
  require(result.has_value(), result.detail); require(result.trajectory->validate_c2_continuity().has_value(), "trajectory is not C2");
  require(result.feasible_tube.has_value() && result.feasible_tube->samples.size() == points.size(), "tube was not retained");
}

void nonzero_boundary_state_and_failure_are_explicit() {
  const auto map = make_map();
  auto config = core::TrajectorySmootherConfig{}; config.maximum_speed_mps = 10.0; config.maximum_acceleration_mps2 = 1000.0;
  config.start_state = core::TrajectoryBoundaryState{{0.1, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0, 0.0};
  config.goal_state = core::TrajectoryBoundaryState{{0.1, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0, 0.2};
  const auto result = core::TrajectorySmoother::smooth(query_for(map), std::vector<core::Point3dEnu>{{1.2, 1.2, 1.5}, {2.4, 1.2, 1.5}}, config);
  require(result.has_value(), result.detail);
  require(result.trajectory->evaluate_time(0U).value().velocity_mps.x_m > 0.09, "start velocity was not preserved");
  require(result.trajectory->evaluate_time(result.trajectory->duration_ns()).value().yaw_acceleration_rps2 > 0.19,
          "goal yaw acceleration was not preserved");

  auto bad = config; bad.maximum_speed_mps = 0.01; bad.maximum_attempts = 1U;
  const auto failed = core::TrajectorySmoother::smooth(query_for(map), std::vector<core::Point3dEnu>{{1.2, 1.2, 1.5}, {1.8, 1.2, 1.5}}, bad);
  require(!failed.has_value() && failed.status == core::TrajectorySmoothingStatus::smoothing_failed, "dynamic failure was not explicit");
}

void rejects_unknown_and_invalid_inputs() {
  const auto map = make_map(21);
  const auto result = core::TrajectorySmoother::smooth(query_for(map), std::vector<core::Point3dEnu>{{1.2, 1.2, 1.5}, {1.8, 1.2, 1.5}});
  require(!result.has_value(), "unknown path point was accepted");
  const auto invalid = core::TrajectorySmoother::smooth(query_for(make_map()), std::vector<core::Point3dEnu>{{0.0, 0.0, 0.0}});
  require(invalid.status == core::TrajectorySmoothingStatus::invalid_input, "short path was not rejected");
}

void rejects_failed_search_and_uses_alternative_path() {
  const auto map = make_map();
  core::StateLatticePath failed_path;
  failed_path.status = core::StateLatticeSearchStatus::timeout;
  failed_path.points_m = {{1.2, 1.2, 1.5}, {1.8, 1.2, 1.5}};
  const auto rejected = core::TrajectorySmoother::smooth(query_for(map), failed_path);
  require(rejected.status == core::TrajectorySmoothingStatus::invalid_input,
          "failed search state was accepted");

  auto config = core::TrajectorySmootherConfig{};
  config.maximum_speed_mps = 10.0; config.maximum_acceleration_mps2 = 100.0;
  bool called = false;
  config.alternative_path_provider = [&called] {
    called = true;
    return std::vector<core::Point3dEnu>{{1.2, 1.2, 1.5}, {1.8, 1.2, 1.5}};
  };
  const auto recovered = core::TrajectorySmoother::smooth(
      query_for(map), std::vector<core::Point3dEnu>{{-1.0, -1.0, -1.0}, {-2.0, -2.0, -2.0}}, config);
  require(called && recovered.has_value(), "alternative path fallback was not attempted");
}

}  // namespace

int main() {
  try {
    straight_path_and_c2();
    nonzero_boundary_state_and_failure_are_explicit();
    rejects_unknown_and_invalid_inputs();
    rejects_failed_search_and_uses_alternative_path();
    std::cout << "[pass] trajectory smoother\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "[fail] " << error.what() << '\n'; return 1;
  }
}
