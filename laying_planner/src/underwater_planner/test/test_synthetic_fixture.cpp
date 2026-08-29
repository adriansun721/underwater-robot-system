#include "underwater_planner/testing/synthetic_fixture.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifdef _WIN32
#include <Windows.h>
#include <Psapi.h>
#else
#include <sys/resource.h>
#endif

namespace {

using underwater_planner::testing::Scenario;
using underwater_planner::testing::SyntheticFixture;
using underwater_planner::testing::format_failure_context;
using underwater_planner::testing::serialize_fixture;

constexpr std::uint64_t kSeed = 0x5EED1234ULL;
std::string active_failure_context;

SyntheticFixture make_tracked_fixture(const Scenario scenario,
                                      const std::uint64_t seed) {
  SyntheticFixture fixture =
      underwater_planner::testing::make_synthetic_fixture(scenario, seed);
  active_failure_context = format_failure_context(fixture.metadata);
  return fixture;
}

void track_fixture_pair(const SyntheticFixture& first,
                        const SyntheticFixture& second) {
  active_failure_context = format_failure_context(first.metadata) +
                           " compared_to={" +
                           format_failure_context(second.metadata) + '}';
}

void require(const bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void require_close(const double actual, const double expected,
                   const double tolerance, const std::string& message) {
  require(std::abs(actual - expected) <= tolerance, message);
}

std::size_t peak_rss_kib() {
#ifdef _WIN32
  PROCESS_MEMORY_COUNTERS counters{};
  if (GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters)) ==
      0) {
    return 0;
  }
  return counters.PeakWorkingSetSize / 1024U;
#else
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0) {
    return 0;
  }
#ifdef __APPLE__
  return static_cast<std::size_t>(usage.ru_maxrss / 1024L);
#else
  return static_cast<std::size_t>(usage.ru_maxrss);
#endif
#endif
}

void test_metadata_is_auditable() {
  const SyntheticFixture fixture = make_tracked_fixture(Scenario::flat, kSeed);
  require(fixture.metadata.seed == kSeed, "fixture seed was not preserved");
  require(fixture.metadata.input_version == "synthetic-fixture/v1",
          "fixture input version is missing");
  require(fixture.metadata.units ==
              "SI[length=m,angle=rad,time=s,curvature=1/m,speed=m/s,tension=N]",
          "fixture units are missing or ambiguous");
  require(fixture.metadata.timestamp_ns == 1700000000592594996ULL,
          "fixture timestamp is missing or nondeterministic");
  require(fixture.metadata.operating_domain == "synthetic-level1/v1",
          "fixture operating domain is missing");
  require(fixture.metadata.risk_semantics ==
              "NOT_APPLICABLE:fixture_only:no_safety_claim",
          "fixture risk semantics overstate a safety guarantee");

  const std::string context = format_failure_context(fixture.metadata);
  require(context.find("seed=1592594996") != std::string::npos,
          "failure context does not contain the random seed");
  require(context.find("input_version=synthetic-fixture/v1") != std::string::npos,
          "failure context does not contain the input version");
  require(context.find("units=SI[") != std::string::npos,
          "failure context does not contain unit information");
  require(context.find("timestamp_ns=1700000000592594996") != std::string::npos,
          "failure context does not contain the input timestamp");
  require(context.find("operating_domain=synthetic-level1/v1") !=
              std::string::npos,
          "failure context does not contain the operating domain");
  require(context.find("risk_semantics=NOT_APPLICABLE:fixture_only:no_safety_claim") !=
              std::string::npos,
          "failure context does not contain explicit risk semantics");
}

void test_generation_is_field_deterministic() {
  const std::vector<Scenario> scenarios{
      Scenario::flat,          Scenario::slope,   Scenario::step,
      Scenario::obstacle,      Scenario::unknown, Scenario::crossing_reference,
  };

  for (const Scenario scenario : scenarios) {
    const SyntheticFixture first = make_tracked_fixture(scenario, kSeed);
    const SyntheticFixture second = make_tracked_fixture(scenario, kSeed);
    track_fixture_pair(first, second);
    require(serialize_fixture(first) == serialize_fixture(second),
            "same scenario and seed produced different fields");
  }
}

void test_seed_changes_generated_state() {
  const SyntheticFixture first = make_tracked_fixture(Scenario::flat, kSeed);
  const SyntheticFixture second =
      make_tracked_fixture(Scenario::flat, kSeed + 1U);
  track_fixture_pair(first, second);
  require(first.robot.x_m != second.robot.x_m,
          "seed does not affect the generated robot state");
  require(first.cable.lag_angle_rad != second.cable.lag_angle_rad,
          "seed does not affect the generated cable state");
}

void test_flat_map() {
  const SyntheticFixture fixture = make_tracked_fixture(Scenario::flat, kSeed);
  for (const auto& cell : fixture.map.cells) {
    require_close(cell.elevation_m, 0.0, 0.0, "flat map contains relief");
    require(cell.known, "flat map contains an unknown cell");
    require(!cell.obstacle, "flat map contains an obstacle");
  }
}

void test_slope_map() {
  const SyntheticFixture fixture = make_tracked_fixture(Scenario::slope, kSeed);
  require_close(fixture.map.at(0, 0).elevation_m, -0.3, 1.0e-12,
                "slope origin has the wrong elevation");
  require_close(fixture.map.at(8, 8).elevation_m, 0.3, 1.0e-12,
                "slope corner has the wrong elevation");
}

void test_step_map() {
  const SyntheticFixture fixture = make_tracked_fixture(Scenario::step, kSeed);
  require_close(fixture.map.at(4, 3).elevation_m, 0.0, 0.0,
                "step low side has the wrong elevation");
  require_close(fixture.map.at(4, 4).elevation_m, 0.3, 0.0,
                "step high side has the wrong elevation");
}

void test_obstacle_map() {
  const SyntheticFixture fixture =
      make_tracked_fixture(Scenario::obstacle, kSeed);
  require(fixture.map.at(4, 4).obstacle, "obstacle fixture has no obstacle");
  require_close(fixture.map.at(4, 4).elevation_m, 0.5, 0.0,
                "obstacle height is incorrect");
}

void test_unknown_region_map() {
  const SyntheticFixture fixture = make_tracked_fixture(Scenario::unknown, kSeed);
  require(!fixture.map.at(4, 4).known, "unknown fixture center is marked known");
  require_close(fixture.map.at(4, 4).confidence, 0.0, 0.0,
                "unknown fixture center has nonzero confidence");
  require(fixture.map.at(0, 0).known,
          "unknown fixture erased cells outside its information gap");
}

void test_crossing_reference_line() {
  const SyntheticFixture fixture =
      make_tracked_fixture(Scenario::crossing_reference, kSeed);
  std::size_t origin_visits = 0;
  for (const auto& sample : fixture.reference_line.samples) {
    if (sample.x_m == 0.0 && sample.y_m == 0.0) {
      ++origin_visits;
    }
  }
  require(origin_visits == 2,
          "crossing reference fixture does not revisit the intersection");
}

void test_grid_bounds_are_rejected() {
  const SyntheticFixture fixture = make_tracked_fixture(Scenario::flat, kSeed);
  bool rejected = false;
  try {
    static_cast<void>(fixture.map.at(fixture.map.height, 0));
  } catch (const std::out_of_range&) {
    rejected = true;
  }
  require(rejected, "out-of-bounds grid access was not rejected");
}

void test_unknown_scenario_is_rejected() {
  bool rejected = false;
  try {
    static_cast<void>(
        make_tracked_fixture(static_cast<Scenario>(255), kSeed));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected, "unknown synthetic scenario was silently accepted");
}

}  // namespace

int main() {
  const auto started_at = std::chrono::steady_clock::now();
  const std::vector<std::pair<const char*, void (*)()>> tests{
      {"metadata_is_auditable", test_metadata_is_auditable},
      {"generation_is_field_deterministic", test_generation_is_field_deterministic},
      {"seed_changes_generated_state", test_seed_changes_generated_state},
      {"flat_map", test_flat_map},
      {"slope_map", test_slope_map},
      {"step_map", test_step_map},
      {"obstacle_map", test_obstacle_map},
      {"unknown_region_map", test_unknown_region_map},
      {"crossing_reference_line", test_crossing_reference_line},
      {"grid_bounds_are_rejected", test_grid_bounds_are_rejected},
      {"unknown_scenario_is_rejected", test_unknown_scenario_is_rejected},
  };

  try {
    for (const auto& [name, test] : tests) {
      test();
      std::cout << "[pass] " << name << '\n';
    }
  } catch (const std::exception& error) {
    std::cerr << "[failure] " << active_failure_context
              << " error=" << error.what() << '\n';
    return 1;
  }

  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started_at);
  std::cout << "[metrics] tests=" << tests.size()
            << " elapsed_ms=" << elapsed.count()
            << " peak_rss_kib=" << peak_rss_kib() << ' '
            << format_failure_context(
                   make_tracked_fixture(Scenario::flat, kSeed).metadata)
            << '\n';
  return 0;
}
