#include "underwater_planner/core/versioned_snapshot.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace {

void require(const bool condition, const char* message) {
  if (!condition) {
    std::cerr << "T04 failure: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

underwater_planner::core::VersionedPlanningSnapshot make_snapshot(
    const std::uint64_t sequence) {
  using namespace underwater_planner::core;
  VersionedPlanningSnapshot snapshot;
  snapshot.map.version = {"map", sequence, {1000 + static_cast<std::int64_t>(sequence),}, "map-frame"};
  snapshot.map.width = 2;
  snapshot.map.height = 2;
  snapshot.map.resolution_m = 1.0;
  snapshot.map.derived_configuration_version = 3;
  snapshot.map.cells.assign(4, MapCell{0.0, 0.01, 1.0, true});
  snapshot.reference_line = make_reference_line(
      static_cast<std::uint32_t>(sequence), "map-frame",
      {{0.0, 0.0}, {1.0, 0.0}, {2.0, 1.0}});
  snapshot.robot_operating_area = {static_cast<std::uint32_t>(sequence), "robot", {{-1, -1}, {3, -1}, {3, 2}, {-1, 2}}};
  snapshot.cable_corridor = {static_cast<std::uint32_t>(sequence), "cable", {{-1, -0.5}, {3, -0.5}, {3, 0.5}, {-1, 0.5}}};
  return snapshot;
}

}  // namespace

int main() {
  using namespace underwater_planner::core;
  const auto snapshot = make_snapshot(1);
  require(validate(snapshot).valid, "valid snapshot was rejected");
  require(snapshot.map.at(1, 1).confidence == 1.0, "map cell lookup failed");

  const auto& line = snapshot.reference_line;
  const auto middle = line.query(0.5);
  require(middle.has_value() && std::abs(middle->x_m - 0.5) < 1e-12,
          "continuous arc-length query failed");
  require(line.local_window(1.0, 0.5).size() == 3,
          "local reference window failed");
  require(!line.query(-0.1).has_value(), "out-of-range reference query was accepted");

  SnapshotManager manager(Duration{100});
  require(manager.update(snapshot, MonotonicTime{1202}).status ==
              SnapshotUpdateStatus::expired,
          "expired snapshot was accepted");
  require(manager.update(snapshot, MonotonicTime{1050}).status ==
              SnapshotUpdateStatus::accepted,
          "initial snapshot was not accepted");
  require(manager.update(snapshot, MonotonicTime{1050}).status ==
              SnapshotUpdateStatus::duplicate,
          "duplicate snapshot was accepted");
  auto conflicting = snapshot;
  conflicting.map.cells[0].elevation_m = 1.0;
  require(manager.update(conflicting, MonotonicTime{1050}).status ==
              SnapshotUpdateStatus::version_rollback,
          "changed payload reused an immutable version");
  conflicting = snapshot;
  conflicting.map.cells[0].obstacle = true;
  conflicting.map.cells[0].obstacle_normal = Vector2d{1.0, 0.0};
  require(manager.update(conflicting, MonotonicTime{1050}).status ==
              SnapshotUpdateStatus::version_rollback,
          "changed obstacle evidence reused an immutable map version");
  const auto require_member_conflict = [&](auto mutate, const char* message) {
    auto changed = snapshot;
    changed.map.version.sequence_number = 2;
    changed.map.version.timestamp = {1002};
    mutate(changed);
    require(manager.update(changed, MonotonicTime{1050}).status ==
                SnapshotUpdateStatus::version_rollback,
            message);
  };
  require_member_conflict(
      [](VersionedPlanningSnapshot& value) {
        value.map.version = {"map", 1, {1001}, "map-frame"};
        value.map.cells[0].elevation_m = 2.0;
        ++value.reference_line.version;
      },
      "a map version was reused with new payload while another member advanced");
  require_member_conflict(
      [](VersionedPlanningSnapshot& value) {
        value.reference_line.points.back().y_m += 0.25;
      },
      "a reference-line version was reused with new payload");
  require_member_conflict(
      [](VersionedPlanningSnapshot& value) {
        value.robot_operating_area.polygon.back().y_m += 0.25;
      },
      "an operating-area version was reused with new payload");
  require_member_conflict(
      [](VersionedPlanningSnapshot& value) {
        value.cable_corridor.polygon.back().y_m += 0.25;
      },
      "a cable-corridor version was reused with new payload");

  SnapshotManager default_manager;
  require(default_manager.update(snapshot, MonotonicTime{1000}).status ==
              SnapshotUpdateStatus::expired,
          "default manager accepted a future-dated map");
  auto next = make_snapshot(2);
  require(manager.update(next, MonotonicTime{1050}).accepted(),
          "new snapshot was not accepted");
  require(manager.is_current(next), "manager did not lock latest snapshot");
  require(manager.update(snapshot, MonotonicTime{1050}).status ==
              SnapshotUpdateStatus::version_rollback,
          "map version rollback was accepted");
  auto corridor_rollback = next;
  corridor_rollback.map.version.sequence_number = 3;
  corridor_rollback.map.version.timestamp = {1003};
  corridor_rollback.cable_corridor = snapshot.cable_corridor;
  require(manager.update(corridor_rollback, MonotonicTime{1050}).status ==
              SnapshotUpdateStatus::out_of_order,
          "cable corridor version rollback was accepted");
  auto corridor_advance = next;
  corridor_advance.map.version.sequence_number = 3;
  corridor_advance.map.version.timestamp = {1003};
  ++corridor_advance.cable_corridor.version;
  require(manager.update(corridor_advance, MonotonicTime{1050}).accepted(),
          "cable corridor version advance was rejected");

  auto invalid = next;
  invalid.cable_corridor.polygon.clear();
  require(!validate(invalid).valid, "empty cable corridor was accepted");
  invalid = next;
  invalid.map.cells[0].elevation_variance_m2 = -1.0;
  require(!validate(invalid).valid, "negative map variance was accepted");
  invalid = next;
  invalid.map.cells[0].obstacle = true;
  invalid.map.cells[0].obstacle_normal = Vector2d{0.0, 0.0};
  require(!validate(invalid).valid, "zero obstacle normal was accepted");
  invalid = next;
  invalid.reference_line.coordinate_frame = "odom";
  require(!validate(invalid).valid, "mixed coordinate frames were accepted");
  std::cout << "T04 versioned snapshot checks passed\n";
}
