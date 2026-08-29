#include "scout_planner/testing/deterministic_fixture.hpp"

#include <array>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace scout_planner::testing {
namespace {

constexpr std::size_t kSizeX = 8U;
constexpr std::size_t kSizeY = 8U;
constexpr std::size_t kSizeZ = 6U;
constexpr std::array kRequiredScenarios{
    Scenario::flat_seabed,       Scenario::overhang_obstacle,
    Scenario::narrow_passage,    Scenario::unknown_region,
    Scenario::stale_region,      Scenario::conflicted_region,
    Scenario::moving_main_robot,
};
constexpr std::array<std::string_view, kRequiredScenarios.size()>
    kScenarioNames{
        "flat_seabed",      "overhang_obstacle", "narrow_passage",
        "unknown_region",   "stale_region",      "conflicted_region",
        "moving_main_robot",
    };

class DeterministicGenerator {
 public:
  explicit DeterministicGenerator(const std::uint64_t seed)
      : state_{seed == 0U ? 0x9E3779B97F4A7C15ULL : seed} {}

  [[nodiscard]] double next_symmetric_unit() {
    state_ ^= state_ >> 12U;
    state_ ^= state_ << 25U;
    state_ ^= state_ >> 27U;
    const std::uint64_t value = state_ * 0x2545F4914F6CDD1DULL;
    constexpr double denominator =
        static_cast<double>(std::numeric_limits<std::uint64_t>::max());
    return 2.0 * (static_cast<double>(value) / denominator) - 1.0;
  }

 private:
  std::uint64_t state_;
};

VoxelGrid3d make_flat_seabed() {
  VoxelGrid3d map{
      kSizeX,
      kSizeY,
      kSizeZ,
      0.5,
      {-2.0, -2.0, -0.5},
      std::vector<VoxelState>(kSizeX * kSizeY * kSizeZ, VoxelState::free),
  };

  for (std::size_t y = 0U; y < map.size_y; ++y) {
    for (std::size_t x = 0U; x < map.size_x; ++x) {
      map.at(x, y, 0U) = VoxelState::occupied;
    }
  }
  return map;
}

void set_information_region(VoxelGrid3d& map, const VoxelState state) {
  for (std::size_t z = 2U; z <= 3U; ++z) {
    for (std::size_t y = 3U; y <= 4U; ++y) {
      for (std::size_t x = 3U; x <= 4U; ++x) {
        map.at(x, y, z) = state;
      }
    }
  }
}

std::vector<MainRobotOccupiedInterval> make_occupied_intervals(
    const std::vector<TimedPosition>& samples) {
  constexpr double physical_radius_m = 0.7;
  constexpr double position_uncertainty_radius_m = 0.15;
  constexpr double conservative_occupied_radius_m = 0.9;
  std::vector<MainRobotOccupiedInterval> intervals;
  if (samples.size() > 1U) {
    intervals.reserve(samples.size() - 1U);
  }
  for (std::size_t index = 1U; index < samples.size(); ++index) {
    const auto& start = samples[index - 1U];
    const auto& end = samples[index];
    intervals.push_back({
        start.time_from_epoch_ns,
        end.time_from_epoch_ns,
        {
            start.position_m,
            end.position_m,
            physical_radius_m,
            position_uncertainty_radius_m,
            conservative_occupied_radius_m,
        },
    });
  }
  return intervals;
}

MainRobotPrediction stationary_main_robot_prediction() {
  std::vector<TimedPosition> samples{
      {0U, {-1.5, 1.5, 0.5}},
      {3'000'000'000ULL, {-1.5, 1.5, 0.5}},
  };
  return {5U, samples, make_occupied_intervals(samples)};
}

void append_metadata(std::ostringstream& output,
                     const FixtureMetadata& metadata) {
  output << "seed=" << metadata.seed << " units=" << metadata.units
         << " frame=" << metadata.frame_id
         << " clock_domain=" << metadata.clock_domain << " input_versions={";
  for (std::size_t index = 0U; index < metadata.input_versions.size(); ++index) {
    if (index != 0U) {
      output << ',';
    }
    const auto& [name, version] = metadata.input_versions[index];
    output << name << ':' << version;
  }
  output << "} non_production="
         << (metadata.non_production ? "true" : "false");
}

std::size_t linear_index(const VoxelGrid3d& map, const std::size_t x,
                         const std::size_t y, const std::size_t z) {
  if (x >= map.size_x || y >= map.size_y || z >= map.size_z) {
    throw std::out_of_range{"voxel coordinate is outside the fixture map"};
  }
  return (z * map.size_y + y) * map.size_x + x;
}

}  // namespace

VoxelState& VoxelGrid3d::at(const std::size_t x, const std::size_t y,
                            const std::size_t z) {
  return cells.at(linear_index(*this, x, y, z));
}

const VoxelState& VoxelGrid3d::at(const std::size_t x, const std::size_t y,
                                  const std::size_t z) const {
  return cells.at(linear_index(*this, x, y, z));
}

std::string_view scenario_name(const Scenario scenario) {
  const auto index = static_cast<std::size_t>(scenario);
  if (index >= kScenarioNames.size()) {
    throw std::invalid_argument{"unknown deterministic fixture scenario"};
  }
  return kScenarioNames[index];
}

const std::array<Scenario, 7U>& required_scenarios() noexcept {
  return kRequiredScenarios;
}

FixtureMetadata make_fixture_metadata(const std::uint64_t seed) {
  return {
      seed,
      "SI[length=m,time=s]",
      "mission_enu",
      "scout_nuc_fixture_boot/v1",
      {{"map", 13U}, {"navigation", 7U}, {"prediction", 5U}},
      true,
  };
}

DeterministicFixture make_deterministic_fixture(const Scenario scenario,
                                                const std::uint64_t seed) {
  static_cast<void>(scenario_name(scenario));
  DeterministicGenerator generator{seed};
  DeterministicFixture fixture{
      scenario,
      make_fixture_metadata(seed),
      make_flat_seabed(),
      {-1.75 + 0.01 * generator.next_symmetric_unit(), -1.5, 0.5},
      stationary_main_robot_prediction(),
  };

  switch (scenario) {
    case Scenario::flat_seabed:
      break;
    case Scenario::overhang_obstacle:
      for (std::size_t y = 2U; y <= 5U; ++y) {
        for (std::size_t x = 2U; x <= 5U; ++x) {
          fixture.map.at(x, y, 4U) = VoxelState::occupied;
        }
      }
      break;
    case Scenario::narrow_passage:
      for (std::size_t z = 1U; z <= 4U; ++z) {
        for (std::size_t y = 0U; y < fixture.map.size_y; ++y) {
          fixture.map.at(2U, y, z) = VoxelState::occupied;
          fixture.map.at(5U, y, z) = VoxelState::occupied;
        }
      }
      break;
    case Scenario::unknown_region:
      set_information_region(fixture.map, VoxelState::unknown);
      break;
    case Scenario::stale_region:
      set_information_region(fixture.map, VoxelState::stale);
      break;
    case Scenario::conflicted_region:
      set_information_region(fixture.map, VoxelState::conflicted);
      break;
    case Scenario::moving_main_robot:
      fixture.main_robot_prediction.samples = {
          {0U, {-1.5, 1.5, 0.5}},
          {1'000'000'000ULL, {-0.5, 1.5, 0.5}},
          {2'000'000'000ULL, {0.5, 1.5, 0.5}},
          {3'000'000'000ULL, {1.5, 1.5, 0.5}},
      };
      fixture.main_robot_prediction.occupied_intervals =
          make_occupied_intervals(fixture.main_robot_prediction.samples);
      break;
  }

  return fixture;
}

std::string serialize_fixture(const DeterministicFixture& fixture) {
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << std::setprecision(std::numeric_limits<double>::max_digits10);
  output << "scenario=" << scenario_name(fixture.scenario) << ' ';
  append_metadata(output, fixture.metadata);
  output << " map=" << fixture.map.size_x << ',' << fixture.map.size_y << ','
         << fixture.map.size_z << ',' << fixture.map.resolution_m << ','
         << fixture.map.origin_m.x << ',' << fixture.map.origin_m.y << ','
         << fixture.map.origin_m.z;
  for (const auto state : fixture.map.cells) {
    output << ',' << static_cast<unsigned int>(state);
  }
  output << " scout_start=" << fixture.scout_start_m.x << ','
         << fixture.scout_start_m.y << ',' << fixture.scout_start_m.z;
  output << " prediction=" << fixture.main_robot_prediction.version;
  for (const auto& sample : fixture.main_robot_prediction.samples) {
    output << ';' << sample.time_from_epoch_ns << ',' << sample.position_m.x
           << ',' << sample.position_m.y << ',' << sample.position_m.z;
  }
  output << " occupied_intervals=";
  for (const auto& interval :
       fixture.main_robot_prediction.occupied_intervals) {
    const auto& volume = interval.swept_volume;
    output << ';' << interval.start_offset_ns << ',' << interval.end_offset_ns
           << ',' << volume.start_center_m.x << ',' << volume.start_center_m.y
           << ',' << volume.start_center_m.z << ',' << volume.end_center_m.x
           << ',' << volume.end_center_m.y << ',' << volume.end_center_m.z << ','
           << volume.physical_radius_m << ','
           << volume.position_uncertainty_radius_m << ','
           << volume.conservative_occupied_radius_m;
  }
  return output.str();
}

std::string format_failure_context(const FixtureMetadata& metadata) {
  std::ostringstream output;
  output.imbue(std::locale::classic());
  append_metadata(output, metadata);
  return output.str();
}

}  // namespace scout_planner::testing
