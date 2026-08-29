#include "scout_planner/testing/deterministic_fixture.hpp"
#include "test_support.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

#ifdef _WIN32
#include <Windows.h>
#include <Psapi.h>
#else
#include <sys/resource.h>
#endif

namespace {

using scout_planner::testing::DeterministicFixture;
using scout_planner::testing::Scenario;
using scout_planner::testing::VoxelState;

constexpr std::uint64_t kSeed = 0x5C0A7B13ULL;
std::string active_failure_context =
    scout_planner::test_support::default_failure_context(kSeed);

using scout_planner::test_support::require;

DeterministicFixture tracked_fixture(const Scenario scenario,
                                     const std::uint64_t seed = kSeed) {
  active_failure_context = scout_planner::testing::format_failure_context(
      scout_planner::testing::make_fixture_metadata(seed));
  auto fixture =
      scout_planner::testing::make_deterministic_fixture(scenario, seed);
  active_failure_context =
      scout_planner::testing::format_failure_context(fixture.metadata);
  return fixture;
}

void test_metadata_is_auditable() {
  const auto fixture = tracked_fixture(Scenario::flat_seabed);
  const auto context =
      scout_planner::testing::format_failure_context(fixture.metadata);

  require(context.find("seed=1544190739") != std::string::npos,
          "failure context omits seed");
  require(context.find("units=SI[length=m,time=s]") != std::string::npos,
          "failure context omits units");
  require(context.find("frame=mission_enu") != std::string::npos,
          "failure context omits frame");
  require(context.find("clock_domain=scout_nuc_fixture_boot/v1") !=
              std::string::npos,
          "failure context omits clock domain");
  require(context.find("input_versions={map:13,navigation:7,prediction:5}") !=
              std::string::npos,
          "failure context omits ordered input versions");
  require(context.find("non_production=true") != std::string::npos,
          "failure context omits non-production marker");
}

void test_all_required_scenarios_are_field_deterministic() {
  for (const auto scenario : scout_planner::testing::required_scenarios()) {
    const auto first = tracked_fixture(scenario);
    const auto second = tracked_fixture(scenario);
    active_failure_context =
        scout_planner::testing::format_failure_context(first.metadata) +
        " scenario=" + std::string{scout_planner::testing::scenario_name(scenario)};
    require(scout_planner::testing::serialize_fixture(first) ==
                scout_planner::testing::serialize_fixture(second),
            "same input produced different fixture fields");
  }
}

void test_seed_changes_seeded_fields() {
  const auto first = tracked_fixture(Scenario::flat_seabed, kSeed);
  const auto second = tracked_fixture(Scenario::flat_seabed, kSeed + 1U);
  require(first.scout_start_m.x != second.scout_start_m.x,
          "seed does not affect seeded start position");
}

void test_flat_seabed_has_solid_floor_and_free_water() {
  const auto fixture = tracked_fixture(Scenario::flat_seabed);
  require(fixture.map.at(4, 4, 0) == VoxelState::occupied,
          "flat seabed floor is not occupied");
  require(fixture.map.at(4, 4, 1) == VoxelState::free,
          "flat seabed water column is not free");
}

void test_overhang_preserves_free_space_below_obstacle() {
  const auto fixture = tracked_fixture(Scenario::overhang_obstacle);
  require(fixture.map.at(4, 4, 4) == VoxelState::occupied,
          "overhang obstacle is missing");
  require(fixture.map.at(4, 4, 3) == VoxelState::free,
          "overhang fixture incorrectly fills the water below it");
}

void test_narrow_passage_has_two_walls_and_free_corridor() {
  const auto fixture = tracked_fixture(Scenario::narrow_passage);
  require(fixture.map.at(2, 4, 2) == VoxelState::occupied,
          "left passage wall is missing");
  require(fixture.map.at(5, 4, 2) == VoxelState::occupied,
          "right passage wall is missing");
  require(fixture.map.at(3, 4, 2) == VoxelState::free,
          "narrow passage center is not free");
}

void test_information_states_remain_distinct() {
  const auto unknown = tracked_fixture(Scenario::unknown_region);
  const auto stale = tracked_fixture(Scenario::stale_region);
  const auto conflicted = tracked_fixture(Scenario::conflicted_region);
  require(unknown.map.at(4, 4, 2) == VoxelState::unknown,
          "UNKNOWN fixture lost its state");
  require(stale.map.at(4, 4, 2) == VoxelState::stale,
          "STALE fixture lost its state");
  require(conflicted.map.at(4, 4, 2) == VoxelState::conflicted,
          "CONFLICTED fixture lost its state");
}

void test_moving_main_robot_prediction_is_time_ordered() {
  const auto fixture = tracked_fixture(Scenario::moving_main_robot);
  const auto& samples = fixture.main_robot_prediction.samples;
  require(samples.size() == 4U, "moving main robot needs four samples");
  for (std::size_t index = 1; index < samples.size(); ++index) {
    require(samples[index - 1U].time_from_epoch_ns <
                samples[index].time_from_epoch_ns,
            "prediction times are not strictly increasing");
    require(samples[index - 1U].position_m.x < samples[index].position_m.x,
            "main robot does not move forward");
  }

  const auto& intervals = fixture.main_robot_prediction.occupied_intervals;
  require(intervals.size() == 3U,
          "moving main robot needs three swept occupancy intervals");
  require(intervals.front().start_offset_ns == 0U,
          "moving occupancy does not start at the alignment epoch");
  require(intervals.back().end_offset_ns == 3'000'000'000ULL,
          "moving occupancy does not cover the prediction horizon");
  for (std::size_t index = 0U; index < intervals.size(); ++index) {
    const auto& interval = intervals[index];
    require(interval.start_offset_ns < interval.end_offset_ns,
            "moving occupancy interval has no duration");
    require(interval.swept_volume.conservative_occupied_radius_m >=
                interval.swept_volume.physical_radius_m +
                    interval.swept_volume.position_uncertainty_radius_m,
            "moving occupancy radius is not conservative");
    if (index != 0U) {
      const auto& previous = intervals[index - 1U];
      require(previous.end_offset_ns == interval.start_offset_ns,
              "moving occupancy contains a temporal gap");
      require(previous.swept_volume.end_center_m.x ==
                  interval.swept_volume.start_center_m.x &&
                  previous.swept_volume.end_center_m.y ==
                      interval.swept_volume.start_center_m.y &&
                  previous.swept_volume.end_center_m.z ==
                      interval.swept_volume.start_center_m.z,
              "moving occupancy is spatially discontinuous");
    }
  }
}

void test_grid_bounds_and_unknown_scenarios_are_rejected() {
  const auto fixture = tracked_fixture(Scenario::flat_seabed);
  bool bounds_rejected = false;
  try {
    static_cast<void>(fixture.map.at(fixture.map.size_x, 0U, 0U));
  } catch (const std::out_of_range&) {
    bounds_rejected = true;
  }
  require(bounds_rejected, "out-of-bounds voxel access was accepted");

  bool scenario_rejected = false;
  try {
    static_cast<void>(tracked_fixture(static_cast<Scenario>(255U)));
  } catch (const std::invalid_argument&) {
    scenario_rejected = true;
  }
  require(scenario_rejected, "unknown scenario was silently accepted");
}

std::size_t peak_rss_kib() {
#ifdef _WIN32
  PROCESS_MEMORY_COUNTERS counters{};
  if (GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters)) ==
      0) {
    return 0U;
  }
  return counters.PeakWorkingSetSize / 1024U;
#else
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0) {
    return 0U;
  }
#ifdef __APPLE__
  return static_cast<std::size_t>(usage.ru_maxrss / 1024L);
#else
  return static_cast<std::size_t>(usage.ru_maxrss);
#endif
#endif
}

}  // namespace

int main() {
  const auto started_at = std::chrono::steady_clock::now();
  try {
    test_metadata_is_auditable();
    test_all_required_scenarios_are_field_deterministic();
    test_seed_changes_seeded_fields();
    test_flat_seabed_has_solid_floor_and_free_water();
    test_overhang_preserves_free_space_below_obstacle();
    test_narrow_passage_has_two_walls_and_free_corridor();
    test_information_states_remain_distinct();
    test_moving_main_robot_prediction_is_time_ordered();
    test_grid_bounds_and_unknown_scenarios_are_rejected();
  } catch (const std::exception& error) {
    std::cerr << "[failure] " << active_failure_context
              << " error=" << error.what() << '\n';
    return 1;
  }

  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started_at);
  std::cout << "[pass] deterministic Scout fixture seam\n"
            << "[metrics] scenarios="
            << scout_planner::testing::required_scenarios().size()
            << " elapsed_ms=" << elapsed.count()
            << " peak_rss_kib=" << peak_rss_kib() << ' '
            << active_failure_context << '\n';

  return 0;
}
