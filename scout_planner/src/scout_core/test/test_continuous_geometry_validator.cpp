#include "contract_fixture.hpp"
#include "scout_planner/core/continuous_geometry_validator.hpp"
#include "test_support.hpp"

#include "underwater/contracts/v1/mapping.pb.h"

#include <algorithm>
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

MapFixture make_map(const int occupied = -1, const int unknown = -1) {
  auto map = fixture::populated_message<contract::HybridMapSnapshot>();
  map.set_map_id("validator-map");
  map.set_map_version(1U);
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
  map.mutable_seafloor()->clear_elevation_z_m();
  map.mutable_seafloor()->clear_quality();
  for (int i = 0; i < 16; ++i) {
    map.mutable_seafloor()->add_elevation_z_m(0.0F);
    map.mutable_seafloor()->add_quality(1.0F);
  }
  set3(map.mutable_occupancy()->mutable_grid());
  set3(map.mutable_esdf()->mutable_grid());
  set3(map.mutable_allowed_water()->mutable_grid());
  map.mutable_esdf()->set_convention(contract::ESDF_NONNEGATIVE_DISTANCE);
  map.mutable_occupancy()->clear_state();
  map.mutable_esdf()->clear_distance_m();
  map.mutable_allowed_water()->clear_allowed();
  for (int i = 0; i < 64; ++i) {
    map.mutable_occupancy()->add_state(
        i == occupied ? contract::VOXEL_OCCUPIED
                       : (i == unknown ? contract::VOXEL_UNKNOWN : contract::VOXEL_FREE));
    map.mutable_esdf()->add_distance_m(2.0F);
    map.mutable_allowed_water()->add_allowed(true);
  }
  fixture::identify_in_place<core::ContractKind::hybrid_map_snapshot>(&map);
  core::MapSnapshotIdentity identity{map.map_id(), map.map_version(), {}};
  std::copy(map.map_content_identity().sha256().begin(),
            map.map_content_identity().sha256().end(),
            identity.content_identity.begin());
  return {fixture::decode_identified<core::ContractKind::hybrid_map_snapshot>(map),
          identity};
}

core::HybridMapQuery query_for(const MapFixture& map) {
  const auto result = core::HybridMapQuery::create(map.snapshot, map.identity);
  require(result.has_value(), result.error().detail);
  return result.value();
}

core::BezierTrajectory4d line(const double y = 1.5) {
  core::QuinticBezierSegment4d segment{};
  segment.duration_ns = 2'000'000'000ULL;
  for (std::size_t i = 0U; i < segment.position_control_points.size(); ++i) {
    const double x = 0.8 + static_cast<double>(i) * 0.4;
    segment.position_control_points[i] = {x, y, 1.5};
    segment.yaw_offset_control_points_rad[i] = 0.0;
  }
  return core::BezierTrajectory4d::create("mission_enu", 0.0, {segment}).value();
}

void accepts_clear_continuous_segment() {
  const auto report = core::ContinuousGeometryValidator::validate(
      query_for(make_map()), line());
  require(report.status == core::ContinuousGeometryValidationStatus::safe,
          "clear segment was not validated safe");
  require(report.primary_outcome == core::ContinuousGeometryValidationOutcome::success,
          "safe segment has wrong outcome");
  require(report.minimum_collision_margin_m.has_value() &&
              report.minimum_collision_margin_m.value() > 0.0,
          "safe report omitted clearance");
  require(report.validated_trajectory_content_identity == line().content_hash(),
          "trajectory identity was not retained");
}

void rejects_derivative_limit() {
  auto config = core::ContinuousGeometryValidationConfig{};
  config.maximum_speed_mps = 0.01;
  const auto report = core::ContinuousGeometryValidator::validate(
      query_for(make_map()), line(), config);
  require(report.status == core::ContinuousGeometryValidationStatus::unsafe,
          "derivative limit was not rejected");
  require(!report.diagnostics.empty(), "derivative rejection lacks diagnostics");
}

void reports_unknown_as_inconclusive() {
  const auto report = core::ContinuousGeometryValidator::validate(
      query_for(make_map(-1, 21)), line());
  require(report.status == core::ContinuousGeometryValidationStatus::inconclusive,
          "unknown map was treated as safe");
  require(report.primary_outcome ==
              core::ContinuousGeometryValidationOutcome::validation_inconclusive,
          "unknown map has wrong outcome");
}

void rejects_occupied_geometry() {
  const auto map = query_for(make_map(21));
  const auto trajectory = line();
  // Independent dense oracle: the endpoints are free, but an interior sample
  // intersects the occupied voxel that a sparse endpoint check could miss.
  bool oracle_detected = false;
  for (std::size_t sample = 0U; sample <= 2048U; ++sample) {
    const auto time = static_cast<std::uint64_t>(
        (static_cast<long double>(sample) * trajectory.duration_ns()) / 2048.0L);
    const auto value = trajectory.evaluate_time(time);
    require(value.has_value(), "dense oracle trajectory evaluation failed");
    const auto query = map.query_point(value.value().position, {});
    require(query.has_value(), "dense oracle map query failed");
    if (query.value().state == core::MapCellState::occupied ||
        query.value().clearance_margin_m < 0.0) {
      oracle_detected = true;
      break;
    }
  }
  require(oracle_detected, "dense oracle failed to detect interior obstacle");
  const auto report = core::ContinuousGeometryValidator::validate(map, trajectory);
  require(report.status == core::ContinuousGeometryValidationStatus::unsafe,
          "occupied geometry was not rejected");
  require(report.earliest_failure_time_offset_ns.has_value(),
          "occupied rejection lacks earliest failure time");
}

}  // namespace

int main() {
  try {
    accepts_clear_continuous_segment();
    rejects_derivative_limit();
    reports_unknown_as_inconclusive();
    rejects_occupied_geometry();
    std::cout << "[pass] continuous geometry validator\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "[fail] " << error.what() << '\n';
    return 1;
  }
}
