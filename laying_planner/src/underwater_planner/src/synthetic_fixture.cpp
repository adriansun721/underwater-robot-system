#include "underwater_planner/testing/synthetic_fixture.hpp"

#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace underwater_planner::testing {
namespace {

constexpr std::size_t kMapExtent = 9;
constexpr double kResolutionM = 0.5;
constexpr double kOriginM = -2.0;
constexpr std::string_view kInputVersion = "synthetic-fixture/v1";
constexpr std::string_view kUnits =
    "SI[length=m,angle=rad,time=s,curvature=1/m,speed=m/s,tension=N]";
constexpr std::uint64_t kFixtureEpochNs = 1700000000000000000ULL;
constexpr std::string_view kOperatingDomain = "synthetic-level1/v1";
constexpr std::string_view kRiskSemantics =
    "NOT_APPLICABLE:fixture_only:no_safety_claim";

class SplitMix64 {
 public:
  explicit SplitMix64(const std::uint64_t seed) : state_(seed) {}

  double next_symmetric_unit() {
    constexpr double kInverse53Bits = 1.0 / 9007199254740992.0;
    return 2.0 * static_cast<double>(next() >> 11U) * kInverse53Bits - 1.0;
  }

 private:
  std::uint64_t next() {
    std::uint64_t value = (state_ += 0x9E3779B97F4A7C15ULL);
    value = (value ^ (value >> 30U)) * 0xBF58476D1CE4E5B9ULL;
    value = (value ^ (value >> 27U)) * 0x94D049BB133111EBULL;
    return value ^ (value >> 31U);
  }

  std::uint64_t state_;
};

GridMap make_flat_map() {
  GridMap map;
  map.width = kMapExtent;
  map.height = kMapExtent;
  map.resolution_m = kResolutionM;
  map.origin_x_m = kOriginM;
  map.origin_y_m = kOriginM;
  map.version = "synthetic-map/v1";
  map.cells.assign(map.width * map.height,
                   GridCell{0.0, 1.0e-4, 1.0, true, false});
  return map;
}

ReferenceLine make_straight_reference() {
  ReferenceLine line;
  line.version = "synthetic-reference/v1";
  for (std::size_t column = 0; column < kMapExtent; ++column) {
    line.samples.push_back(
        {kOriginM + static_cast<double>(column) * kResolutionM, 0.0});
  }
  return line;
}

ReferenceLine make_crossing_reference() {
  ReferenceLine line;
  line.version = "synthetic-reference-crossing/v1";
  line.samples = {
      {-2.0, -2.0}, {0.0, 0.0}, {2.0, 2.0}, {0.0, 2.0},
      {0.0, 0.0},   {2.0, -2.0},
  };
  return line;
}

std::string_view scenario_name(const Scenario scenario) {
  switch (scenario) {
    case Scenario::flat:
      return "flat";
    case Scenario::slope:
      return "slope";
    case Scenario::step:
      return "step";
    case Scenario::obstacle:
      return "obstacle";
    case Scenario::unknown:
      return "unknown";
    case Scenario::crossing_reference:
      return "crossing_reference";
  }
  throw std::invalid_argument("unrecognized synthetic scenario");
}

void append_metadata(std::ostringstream& output, const FixtureMetadata& metadata,
                     const std::string_view separator) {
  output << "seed=" << metadata.seed << separator
         << "input_version=" << metadata.input_version << separator
         << "units=" << metadata.units << separator
         << "timestamp_ns=" << metadata.timestamp_ns << separator
         << "operating_domain=" << metadata.operating_domain << separator
         << "risk_semantics=" << metadata.risk_semantics;
}

}  // namespace

const GridCell& GridMap::at(const std::size_t row,
                            const std::size_t column) const {
  if (row >= height || column >= width) {
    throw std::out_of_range("synthetic grid cell is outside the map");
  }
  return cells.at(row * width + column);
}

SyntheticFixture make_synthetic_fixture(const Scenario scenario,
                                         const std::uint64_t seed) {
  static_cast<void>(scenario_name(scenario));

  SplitMix64 generator(seed);
  SyntheticFixture fixture;
  fixture.scenario = scenario;
  fixture.metadata = {
      seed,
      std::string{kInputVersion},
      std::string{kUnits},
      kFixtureEpochNs + seed % 1000000000ULL,
      std::string{kOperatingDomain},
      std::string{kRiskSemantics},
  };
  fixture.map = make_flat_map();
  fixture.reference_line = make_straight_reference();
  fixture.robot = {0.01 * generator.next_symmetric_unit(),
                   -1.5,
                   0.0,
                   0.2,
                   0.0,
                   "synthetic-robot-state/v1"};
  fixture.cable = {0.005 * generator.next_symmetric_unit(),
                   1.0e-4,
                   0.2,
                   100.0,
                   "synthetic-cable-state/v1"};

  switch (scenario) {
    case Scenario::flat:
      break;
    case Scenario::slope:
      for (std::size_t row = 0; row < fixture.map.height; ++row) {
        for (std::size_t column = 0; column < fixture.map.width; ++column) {
          const double x_m = fixture.map.origin_x_m +
                             static_cast<double>(column) * fixture.map.resolution_m;
          const double y_m = fixture.map.origin_y_m +
                             static_cast<double>(row) * fixture.map.resolution_m;
          fixture.map.cells.at(row * fixture.map.width + column).elevation_m =
              0.1 * x_m + 0.05 * y_m;
        }
      }
      break;
    case Scenario::step:
      for (std::size_t row = 0; row < fixture.map.height; ++row) {
        for (std::size_t column = 4; column < fixture.map.width; ++column) {
          fixture.map.cells.at(row * fixture.map.width + column).elevation_m = 0.3;
        }
      }
      break;
    case Scenario::obstacle: {
      GridCell& center = fixture.map.cells.at(4 * fixture.map.width + 4);
      center.elevation_m = 0.5;
      center.obstacle = true;
      break;
    }
    case Scenario::unknown:
      for (std::size_t row = 3; row <= 5; ++row) {
        for (std::size_t column = 3; column <= 5; ++column) {
          GridCell& cell = fixture.map.cells.at(row * fixture.map.width + column);
          cell.known = false;
          cell.confidence = 0.0;
        }
      }
      break;
    case Scenario::crossing_reference:
      fixture.reference_line = make_crossing_reference();
      break;
  }

  return fixture;
}

std::string serialize_fixture(const SyntheticFixture& fixture) {
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << std::setprecision(std::numeric_limits<double>::max_digits10);
  output << "scenario=" << scenario_name(fixture.scenario) << ';';
  append_metadata(output, fixture.metadata, ";");
  output << ";map=" << fixture.map.version << ',' << fixture.map.width << ','
         << fixture.map.height << ',' << fixture.map.resolution_m << ','
         << fixture.map.origin_x_m << ',' << fixture.map.origin_y_m;
  for (const GridCell& cell : fixture.map.cells) {
    output << ";cell=" << cell.elevation_m << ',' << cell.elevation_variance_m2
           << ',' << cell.confidence << ',' << cell.known << ',' << cell.obstacle;
  }
  output << ";reference=" << fixture.reference_line.version;
  for (const ReferenceSample& sample : fixture.reference_line.samples) {
    output << ";reference_sample=" << sample.x_m << ',' << sample.y_m;
  }
  output << ";robot=" << fixture.robot.version << ',' << fixture.robot.x_m << ','
         << fixture.robot.y_m << ',' << fixture.robot.heading_rad << ','
         << fixture.robot.speed_mps << ',' << fixture.robot.curvature_per_m
         << ";cable=" << fixture.cable.version << ','
         << fixture.cable.lag_angle_rad << ',' << fixture.cable.lag_variance_rad2
         << ',' << fixture.cable.payout_speed_mps << ',' << fixture.cable.tension_n;
  return output.str();
}

std::string format_failure_context(const FixtureMetadata& metadata) {
  std::ostringstream output;
  output.imbue(std::locale::classic());
  append_metadata(output, metadata, " ");
  return output.str();
}

}  // namespace underwater_planner::testing
