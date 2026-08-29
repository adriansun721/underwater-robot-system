#include "contract_fixture.hpp"
#include "scout_planner/core/hybrid_map_query.hpp"
#include "test_support.hpp"

#include "underwater/contracts/v1/mapping.pb.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
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

contract::HybridMapSnapshot base_map() {
  auto map = fixture::populated_message<contract::HybridMapSnapshot>();
  map.set_map_id("query-map");
  map.set_map_version(16U);
  auto* region = map.mutable_map_region();
  region->clear_xyz_m();
  for (const double value : {0.0, 0.0, 0.0, 4.0, 4.0, 4.0}) {
    region->add_xyz_m(value);
  }

  auto set_grid_3d = [](contract::GridGeometry3d* grid) {
    grid->set_origin_x_m(0.5);
    grid->set_origin_y_m(0.5);
    grid->set_origin_z_m(0.5);
    grid->set_resolution_x_m(1.0);
    grid->set_resolution_y_m(1.0);
    grid->set_resolution_z_m(1.0);
    grid->set_cell_count_x(4U);
    grid->set_cell_count_y(4U);
    grid->set_cell_count_z(4U);
    grid->set_frame_id("mission_enu");
  };
  auto set_grid_2d = [](contract::GridGeometry2d* grid) {
    grid->set_origin_x_m(0.5);
    grid->set_origin_y_m(0.5);
    grid->set_resolution_x_m(1.0);
    grid->set_resolution_y_m(1.0);
    grid->set_cell_count_x(4U);
    grid->set_cell_count_y(4U);
    grid->set_frame_id("mission_enu");
  };

  set_grid_2d(map.mutable_seafloor()->mutable_grid());
  map.mutable_seafloor()->clear_elevation_z_m();
  map.mutable_seafloor()->clear_quality();
  for (std::size_t index = 0U; index < 16U; ++index) {
    map.mutable_seafloor()->add_elevation_z_m(0.0F);
    map.mutable_seafloor()->add_quality(0.9F);
  }
  set_grid_3d(map.mutable_occupancy()->mutable_grid());
  map.mutable_occupancy()->clear_state();
  set_grid_3d(map.mutable_esdf()->mutable_grid());
  map.mutable_esdf()->set_convention(contract::ESDF_NONNEGATIVE_DISTANCE);
  map.mutable_esdf()->clear_distance_m();
  set_grid_3d(map.mutable_allowed_water()->mutable_grid());
  map.mutable_allowed_water()->clear_allowed();
  for (std::size_t index = 0U; index < 64U; ++index) {
    map.mutable_occupancy()->add_state(contract::VOXEL_FREE);
    map.mutable_esdf()->add_distance_m(2.0F);
    map.mutable_allowed_water()->add_allowed(true);
  }
  map.clear_semantic_regions();

  return map;
}

MapFixture identify_map(contract::HybridMapSnapshot map) {
  fixture::identify_in_place<core::ContractKind::hybrid_map_snapshot>(&map);
  core::MapSnapshotIdentity identity{
      map.map_id(), map.map_version(), {}};
  std::copy(map.map_content_identity().sha256().begin(),
            map.map_content_identity().sha256().end(),
            identity.content_identity.begin());
  return {fixture::decode_identified<
              core::ContractKind::hybrid_map_snapshot>(map),
          identity};
}

MapFixture make_map() { return identify_map(base_map()); }

std::size_t voxel_index(const std::uint32_t x, const std::uint32_t y,
                        const std::uint32_t z) {
  return x + 4U * (y + 4U * z);
}

void free_point_reports_all_layers_and_source() {
  const auto fixture_map = make_map();
  const auto built =
      core::HybridMapQuery::create(fixture_map.snapshot, fixture_map.identity);
  require(built.has_value(), built.error().detail);

  const core::SafetyMargins margins{0.2, 0.1, 0.1, 0.1, 0.1};
  const auto result = built.value().query_point({1.5, 1.5, 1.5}, margins);
  require(result.has_value(), result.error().detail);
  require(result.value().state == core::MapCellState::free,
          "known free point was not traversable");
  require(result.value().esdf_distance_m == 2.0,
          "ESDF distance was not returned");
  require(std::abs(result.value().clearance_margin_m - 0.9) < 1.0e-12,
          "separated safety margins were not applied");
  require(result.value().seafloor_elevation_m == 0.0 &&
              result.value().quality == 0.9F,
          "seafloor elevation/quality were not returned");
  require(result.value().source.map_version == 16U &&
              result.value().source.map_id == "query-map",
          "map source identity was not returned");
  require(result.value().source.generated_at_monotonic_ns == 300 &&
              result.value().source.observed_at_utc_ns == 100 &&
              result.value().source.observation_uncertainty_ns == 1U &&
              result.value().source.map_region_bbox.minimum_m.x_m == 0.0 &&
              result.value().source.map_region_bbox.maximum_m.z_m == 4.0,
          "map timestamp or region provenance was not returned");
  require(result.value().information_gaps.empty(),
          "known free point reported an information gap");
}

void uncertainty_states_and_overhang_fail_closed() {
  struct Case {
    contract::VoxelState wire_state;
    core::MapCellState state;
    core::MapInformationGapKind gap;
  };
  for (const auto& test_case : {
           Case{contract::VOXEL_UNKNOWN, core::MapCellState::unknown,
                core::MapInformationGapKind::unknown_occupancy},
           Case{contract::VOXEL_STALE, core::MapCellState::stale,
                core::MapInformationGapKind::stale_occupancy},
           Case{contract::VOXEL_CONFLICTED, core::MapCellState::conflicted,
                core::MapInformationGapKind::conflicted_occupancy},
       }) {
    auto map = base_map();
    map.mutable_occupancy()->set_state(
        static_cast<int>(voxel_index(2U, 2U, 2U)), test_case.wire_state);
    const auto fixture_map = identify_map(std::move(map));
    const auto built =
        core::HybridMapQuery::create(fixture_map.snapshot, fixture_map.identity);
    const auto result = built.value().query_point(
        {2.5, 2.5, 2.5}, {0.0, 0.0, 0.0, 0.0, 0.0});
    require(result.value().state == test_case.state,
            "information state was not preserved");
    require(result.value().information_gaps.size() == 1U &&
                result.value().information_gaps.front().kind == test_case.gap,
            "information-gap location was not reported");
  }

  auto overhang = base_map();
  overhang.mutable_occupancy()->set_state(
      static_cast<int>(voxel_index(1U, 1U, 2U)),
      contract::VOXEL_OCCUPIED);
  const auto overhang_fixture = identify_map(std::move(overhang));
  const auto query = core::HybridMapQuery::create(overhang_fixture.snapshot,
                                                   overhang_fixture.identity);
  const auto result = query.value().query_point(
      {1.5, 1.5, 2.5}, {0.0, 0.0, 0.0, 0.0, 0.0});
  require(result.value().state == core::MapCellState::occupied,
          "2.5D seafloor incorrectly erased an overhanging obstacle");
}

void allowed_water_semantics_clearance_and_boundaries_are_hard_limits() {
  auto map = base_map();
  map.mutable_allowed_water()->set_allowed(
      static_cast<int>(voxel_index(1U, 1U, 1U)), false);
  map.mutable_allowed_water()->set_allowed(
      static_cast<int>(voxel_index(2U, 2U, 1U)), false);
  auto* no_entry = map.add_semantic_regions();
  no_entry->set_region_id("a-no-entry");
  no_entry->set_semantic_class(contract::SEMANTIC_NO_ENTRY);
  for (const double value : {2.0, 2.0, 2.0, 3.0, 3.0, 3.0}) {
    no_entry->mutable_region()->add_xyz_m(value);
  }
  no_entry->mutable_region()->set_frame_id("mission_enu");
  auto* shadow = map.add_semantic_regions();
  shadow->set_region_id("b-shadow");
  shadow->set_semantic_class(contract::SEMANTIC_COMMUNICATION_SHADOW);
  for (const double value : {3.0, 3.0, 3.0, 4.0, 4.0, 4.0}) {
    shadow->mutable_region()->add_xyz_m(value);
  }
  shadow->mutable_region()->set_frame_id("mission_enu");
  map.mutable_esdf()->set_distance_m(
      static_cast<int>(voxel_index(1U, 2U, 1U)), 0.25F);
  const auto fixture_map = identify_map(std::move(map));
  const auto query =
      core::HybridMapQuery::create(fixture_map.snapshot, fixture_map.identity)
          .value();

  const core::SafetyMargins margins{0.1, 0.1, 0.1, 0.1, 0.1};
  require(query.query_point({1.5, 1.5, 1.5}, margins).value().state ==
              core::MapCellState::occupied,
          "disallowed water was traversable");
  require(query
              .query_point({1.5, 2.5, 1.5},
                           {0.6, 0.0, 0.0, 0.0, 0.0})
              .value()
              .state == core::MapCellState::occupied,
          "safety margin overlapping disallowed water was traversable");
  const auto semantic = query.query_point({2.5, 2.5, 2.5}, margins).value();
  require(semantic.state == core::MapCellState::occupied &&
              semantic.semantic_restrictions.front() ==
                  core::MapSemanticRestriction::no_entry,
          "NO_ENTRY semantic region was not enforced");
  const auto inflated_semantic =
      query
          .query_point({1.5, 2.5, 3.0}, {0.6, 0.0, 0.0, 0.0, 0.0})
          .value();
  require(inflated_semantic.state == core::MapCellState::occupied &&
              std::find(inflated_semantic.semantic_restrictions.begin(),
                        inflated_semantic.semantic_restrictions.end(),
                        core::MapSemanticRestriction::no_entry) !=
                  inflated_semantic.semantic_restrictions.end(),
          "safety margin overlapping NO_ENTRY was traversable");
  const auto communication =
      query.query_point({3.5, 3.5, 3.5}, {0.0, 0.0, 0.0, 0.0, 0.0}).value();
  require(communication.state == core::MapCellState::free &&
              communication.semantic_restrictions.front() ==
                  core::MapSemanticRestriction::communication_shadow,
          "non-prohibitive semantic restriction was lost");
  require(query.query_point({1.5, 2.5, 1.5}, margins).value().state ==
              core::MapCellState::occupied,
          "insufficient ESDF clearance was traversable");

  const auto boundary = query.query_point({0.25, 2.0, 2.0}, margins).value();
  require(boundary.state == core::MapCellState::unknown &&
              boundary.information_gaps.front().kind ==
                  core::MapInformationGapKind::boundary_clearance &&
              std::abs(boundary.clearance_margin_m + 0.25) < 1.0e-12,
          "map boundary did not fail closed");
  const auto outside = query.query_point({-0.01, 2.0, 2.0}, margins).value();
  require(outside.state == core::MapCellState::unknown &&
              outside.information_gaps.front().kind ==
                  core::MapInformationGapKind::outside_snapshot,
          "out-of-map query did not locate its information gap");
}

void dependency_and_numeric_errors_are_rejected() {
  const auto fixture_map = make_map();
  auto wrong = fixture_map.identity;
  ++wrong.map_version;
  const auto mismatched = core::HybridMapQuery::create(fixture_map.snapshot, wrong);
  require(!mismatched.has_value() &&
              mismatched.error().code ==
                  core::MapQueryErrorCode::invalid_dependency,
          "version mismatch was accepted");
  wrong = fixture_map.identity;
  wrong.content_identity[0] ^= 0xFFU;
  require(!core::HybridMapQuery::create(fixture_map.snapshot, wrong).has_value(),
          "content-identity mismatch was accepted");

  const auto query =
      core::HybridMapQuery::create(fixture_map.snapshot, fixture_map.identity)
          .value();
  const auto invalid_margin = query.query_point(
      {1.5, 1.5, 1.5}, {0.0, 0.0, -0.1, 0.0, 0.0});
  require(!invalid_margin.has_value() &&
              invalid_margin.error().code ==
                  core::MapQueryErrorCode::invalid_safety_margin,
          "negative separated margin was accepted");
  const auto invalid_position = query.query_point(
      {std::numeric_limits<double>::infinity(), 1.5, 1.5},
      {0.0, 0.0, 0.0, 0.0, 0.0});
  require(!invalid_position.has_value(), "non-finite query was accepted");
}

void invalid_dense_layers_and_unknown_semantics_never_reach_query() {
  auto missing = base_map();
  missing.mutable_allowed_water()->mutable_allowed()->RemoveLast();
  const auto missing_result =
      core::ProtobufAdapter::canonicalize_and_identify<
          core::ContractKind::hybrid_map_snapshot>(missing.SerializeAsString());
  require(!missing_result.has_value(), "missing dense map cell was accepted");

  auto non_finite = base_map();
  non_finite.mutable_esdf()->set_distance_m(
      0, std::numeric_limits<float>::quiet_NaN());
  const auto finite_result =
      core::ProtobufAdapter::canonicalize_and_identify<
          core::ContractKind::hybrid_map_snapshot>(
          non_finite.SerializeAsString());
  require(!finite_result.has_value(), "non-finite ESDF reached the query seam");

  auto unknown_semantic = base_map();
  auto* region = unknown_semantic.add_semantic_regions();
  region->set_region_id("unknown-class");
  region->set_semantic_class(static_cast<contract::SemanticClass>(99));
  for (const double value : {1.0, 1.0, 1.0, 2.0, 2.0, 2.0}) {
    region->mutable_region()->add_xyz_m(value);
  }
  region->mutable_region()->set_frame_id("mission_enu");
  const auto semantic_result =
      core::ProtobufAdapter::canonicalize_and_identify<
          core::ContractKind::hybrid_map_snapshot>(
          unknown_semantic.SerializeAsString());
  require(!semantic_result.has_value(),
          "unknown semantic class reached the query seam");
}

void deterministic_supercover_catches_thin_and_corner_touching_obstacles() {
  auto thin = base_map();
  thin.mutable_occupancy()->set_state(
      static_cast<int>(voxel_index(1U, 1U, 1U)),
      contract::VOXEL_OCCUPIED);
  auto* narrow_no_entry = thin.add_semantic_regions();
  narrow_no_entry->set_region_id("thin-no-entry");
  narrow_no_entry->set_semantic_class(contract::SEMANTIC_NO_ENTRY);
  for (const double value : {2.20, 0.10, 2.20, 2.30, 0.20, 2.30}) {
    narrow_no_entry->mutable_region()->add_xyz_m(value);
  }
  narrow_no_entry->mutable_region()->set_frame_id("mission_enu");
  const auto fixture_map = identify_map(std::move(thin));
  const auto query =
      core::HybridMapQuery::create(fixture_map.snapshot, fixture_map.identity)
          .value();
  const core::SafetyMargins zero{0.0, 0.0, 0.0, 0.0, 0.0};

  const auto axis =
      query.query_supercover({0.5, 1.5, 1.5}, {3.5, 1.5, 1.5}, zero)
          .value();
  require(axis.state == core::MapCellState::occupied,
          "one-voxel-thick obstacle was skipped");
  require(axis.queried_voxels ==
              std::vector<core::VoxelIndex3d>{{0U, 1U, 1U}, {1U, 1U, 1U},
                                              {2U, 1U, 1U}, {3U, 1U, 1U}},
          "axis supercover was not canonical X-fastest order");

  const auto diagonal_first =
      query.query_supercover({0.5, 0.5, 0.5}, {3.5, 3.5, 3.5}, zero)
          .value();
  const auto diagonal_second =
      query.query_supercover({0.5, 0.5, 0.5}, {3.5, 3.5, 3.5}, zero)
          .value();
  require(diagonal_first.queried_voxels.size() == 22U,
          "corner-touching supercover omitted voxels");
  require(diagonal_first.queried_voxels == diagonal_second.queried_voxels,
          "supercover order was nondeterministic");
  const auto diagonal_reverse =
      query.query_supercover({3.5, 3.5, 3.5}, {0.5, 0.5, 0.5}, zero)
          .value();
  require(diagonal_first.queried_voxels == diagonal_reverse.queried_voxels,
          "supercover changed canonical order when direction reversed");

  const auto semantic_crossing =
      query.query_supercover({2.0, 0.15, 2.25}, {2.5, 0.15, 2.25}, zero)
          .value();
  require(semantic_crossing.state == core::MapCellState::occupied &&
              std::find(semantic_crossing.semantic_restrictions.begin(),
                        semantic_crossing.semantic_restrictions.end(),
                        core::MapSemanticRestriction::no_entry) !=
                  semantic_crossing.semantic_restrictions.end(),
          "continuous segment skipped a thin semantic NO_ENTRY region");
  const auto semantic_margin_crossing =
      query
          .query_supercover({2.0, 0.75, 2.25}, {2.5, 0.75, 2.25},
                            {0.6, 0.0, 0.0, 0.0, 0.0})
          .value();
  require(semantic_margin_crossing.state == core::MapCellState::occupied,
          "swept safety margin skipped a nearby NO_ENTRY region");
}

}  // namespace

int main() {
  try {
    free_point_reports_all_layers_and_source();
    uncertainty_states_and_overhang_fail_closed();
    allowed_water_semantics_clearance_and_boundaries_are_hard_limits();
    dependency_and_numeric_errors_are_rejected();
    invalid_dense_layers_and_unknown_semantics_never_reach_query();
    deterministic_supercover_catches_thin_and_corner_touching_obstacles();
    std::cout << "hybrid map query tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "hybrid map query test failed: " << error.what() << '\n';
    return 1;
  }
}
