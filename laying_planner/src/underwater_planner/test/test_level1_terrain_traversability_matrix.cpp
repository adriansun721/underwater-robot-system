#include <array>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#ifndef UNDERWATER_PLANNER_TEST_SOURCE_DIR
#error "UNDERWATER_PLANNER_TEST_SOURCE_DIR must identify the test source directory"
#endif

namespace {

enum class TestSource : std::size_t {
  terrain_analyzer,
  step_geometry,
  collision,
  directional_slope,
  step_traversability,
  count,
};

struct DesignTestLink {
  const char* design_id;
  TestSource source;
  const char* test_anchor;
};

constexpr std::array<DesignTestLink, 34> kMatrix{{
    {"18.2.1-1", TestSource::terrain_analyzer,
     "detrended_roughness_is_independent_of_plane_slope"},
    {"18.2.1-2", TestSource::terrain_analyzer,
     "multi_direction_plane_has_direction_independent_gradient"},
    {"18.2.1-3", TestSource::terrain_analyzer,
     "detrended_roughness_is_independent_of_plane_slope"},
    {"18.2.1-4", TestSource::step_geometry,
     "complete_step_geometry_is_extracted_from_two_support_surfaces"},
    {"18.2.1-5", TestSource::step_traversability,
     "analyzed_step_height_is_invariant_to_robot_test_heading"},
    {"18.2.1-6", TestSource::terrain_analyzer,
     "isolated_outlier_does_not_drag_the_surface_gradient"},
    {"18.2.1-7", TestSource::terrain_analyzer,
     "support_failure_modes_are_distinct"},
    {"18.2.1-8", TestSource::terrain_analyzer,
     "unrepresentable_gradient_covariance_is_rejected"},
    {"18.2.2-1", TestSource::collision,
     "local_normal_uses_directional_margin_and_missing_normal_uses_upper_bound"},
    {"18.2.2-2", TestSource::collision,
     "complete_complex_footprint_rejects_an_edge_collision"},
    {"18.2.2-3", TestSource::collision,
     "unavailable_terrain_is_blocked_and_preserved_as_information_gaps"},
    {"18.2.2-4", TestSource::collision,
     "local_normal_uses_directional_margin_and_missing_normal_uses_upper_bound"},
    {"18.2.2-5", TestSource::directional_slope,
     "gradient_is_projected_before_atan_with_signed_longitudinal_limits"},
    {"18.2.2-6", TestSource::step_traversability,
     "complete_step_height_is_not_scaled_by_crossing_angle_or_direction"},
    {"18.2.2-7", TestSource::step_traversability,
     "repeated_crossings_apply_each_directional_height_limit"},
    {"18.2.2-8", TestSource::step_traversability,
     "independent_track_support_reports_roll_coverage_and_local_drop"},
    {"18.2.2-9", TestSource::step_traversability,
     "independent_track_support_reports_roll_coverage_and_local_drop"},
    {"18.2.2-10", TestSource::step_traversability,
     "riding_transition_nearby_and_invalid_geometry_are_distinct"},
    {"18.2.2-11", TestSource::directional_slope,
     "mean_safe_but_conservative_upper_bound_rejects_segment"},
    {"18.2.2-12", TestSource::directional_slope,
     "anisotropic_covariance_rotates_between_directional_bounds"},
    {"18.2.2-13", TestSource::directional_slope,
     "asymmetric_up_down_bounds_include_the_same_nonzero_variance"},
    {"18.2.2-14", TestSource::directional_slope,
     "risk_policy_is_bound_to_analysis_calibration_domain_and_local_epsilon"},
    {"18.2.2-15", TestSource::directional_slope,
     "risk_policy_is_bound_to_analysis_calibration_domain_and_local_epsilon"},
    {"18.2.2-16", TestSource::directional_slope,
     "risk_policy_is_bound_to_analysis_calibration_domain_and_local_epsilon"},
    {"18.2.2-17", TestSource::directional_slope,
     "roughness_hard_gate_covers_edges_and_fails_closed"},
    {"18.2.2-18", TestSource::collision,
     "collision_sweep_requires_the_complete_map_version"},
    {"18.2.2-invariant-1", TestSource::collision,
     "obstacles_boundaries_and_invalid_covariance_fail_safely"},
    {"18.2.2-invariant-2", TestSource::collision,
     "complete_complex_footprint_rejects_an_edge_collision"},
    {"18.2.2-invariant-3", TestSource::collision,
     "unavailable_terrain_is_blocked_and_preserved_as_information_gaps"},
    {"18.2.2-invariant-4", TestSource::step_traversability,
     "complete_step_height_is_not_scaled_by_crossing_angle_or_direction"},
    {"18.2.2-invariant-5", TestSource::step_traversability,
     "outliers_multi_step_diagnostics_and_between_pose_sweep_are_auditable"},
    {"18.2.2-invariant-6", TestSource::step_traversability,
     "independent_track_support_reports_roll_coverage_and_local_drop"},
    {"18.2.2-invariant-7", TestSource::directional_slope,
     "adaptive_sweep_rejects_a_hazard_between_segment_endpoints"},
    {"18.2.2-invariant-8", TestSource::directional_slope,
     "roughness_hard_gate_covers_edges_and_fails_closed"},
}};

constexpr std::array<DesignTestLink, 1> kSupportingLinks{{
    {"18.2.1-8", TestSource::directional_slope,
     "invalid_gradient_covariance_rejects_the_whole_motion_segment"},
}};

constexpr std::array<const char*, static_cast<std::size_t>(TestSource::count)>
    kSourceNames{{
    "test_terrain_analyzer.cpp",
    "test_step_geometry.cpp",
    "test_traversability_evaluator.cpp",
    "test_directional_slope.cpp",
    "test_step_traversability.cpp",
}};

std::vector<std::string> expected_design_ids() {
  std::vector<std::string> expected;
  for (int item = 1; item <= 8; ++item) {
    expected.push_back("18.2.1-" + std::to_string(item));
  }
  for (int item = 1; item <= 18; ++item) {
    expected.push_back("18.2.2-" + std::to_string(item));
  }
  for (int item = 1; item <= 8; ++item) {
    expected.push_back("18.2.2-invariant-" + std::to_string(item));
  }
  return expected;
}

[[noreturn]] void fail(const std::string& message) {
  std::cerr << "T44 failure: " << message << '\n';
  std::exit(EXIT_FAILURE);
}

std::string read_source(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) fail("cannot read mapped test source: " + path);
  std::ostringstream contents;
  contents << input.rdbuf();
  return contents.str();
}

std::string quote_command_argument(const std::string& argument) {
  if (argument.find('"') != std::string::npos) {
    fail("test executable path contains an unsupported quote");
  }
  return '"' + argument + '"';
}

std::size_t count_occurrences(const std::string& contents,
                              const std::string& needle) {
  std::size_t count = 0U;
  std::size_t position = 0U;
  while ((position = contents.find(needle, position)) != std::string::npos) {
    ++count;
    position += needle.size();
  }
  return count;
}

bool function_carries_design_marker(const std::string& contents,
                                    const DesignTestLink& entry) {
  const std::string declaration = "void " + std::string{entry.test_anchor} +
                                  "(";
  const std::size_t function_start = contents.find(declaration);
  if (function_start == std::string::npos) return false;
  const std::size_t next_function = contents.find("\nvoid ", function_start + 1U);
  const std::string marker = "// Design: " + std::string{entry.design_id};
  const std::size_t marker_position = contents.find(marker, function_start);
  return marker_position != std::string::npos &&
         (next_function == std::string::npos ||
          marker_position < next_function);
}

}  // namespace

int main(const int argc, const char* const argv[]) {
  if (argc != 6) {
    fail("expected the five T06-T10 test executable paths");
  }

  std::array<std::string, kSourceNames.size()> sources;
  for (std::size_t index = 0U; index < kSourceNames.size(); ++index) {
    sources[index] = read_source(std::string{UNDERWATER_PLANNER_TEST_SOURCE_DIR} +
                                 "/" + kSourceNames[index]);
  }

  const std::vector<std::string> expected = expected_design_ids();
  if (expected.size() != kMatrix.size()) {
    fail("matrix entry count does not match the design structure");
  }
  for (std::size_t index = 0U; index < kMatrix.size(); ++index) {
    const DesignTestLink& entry = kMatrix[index];
    if (entry.design_id != expected[index]) {
      fail("matrix is missing or reorders design item " +
           expected[index]);
    }
    const std::size_t source_index = static_cast<std::size_t>(entry.source);
    if (source_index >= sources.size() ||
        count_occurrences(sources[source_index], entry.test_anchor) < 2U ||
        !function_carries_design_marker(sources[source_index], entry)) {
      fail("mapped test must be marked, defined, and invoked for " +
           std::string{entry.design_id} + ": " + entry.test_anchor);
    }
  }
  for (const DesignTestLink& link : kSupportingLinks) {
    const std::size_t source_index = static_cast<std::size_t>(link.source);
    if (source_index >= sources.size() ||
        count_occurrences(sources[source_index], link.test_anchor) < 2U ||
        !function_carries_design_marker(sources[source_index], link)) {
      fail("supporting test must be marked, defined, and invoked for " +
           std::string{link.design_id} + ": " + link.test_anchor);
    }
  }

  for (int executable = 1; executable < argc; ++executable) {
    const int exit_code =
        std::system(quote_command_argument(argv[executable]).c_str());
    if (exit_code != 0) {
      fail("mapped T06-T10 test executable failed: " +
           std::string{argv[executable]});
    }
  }

  for (const DesignTestLink& entry : kMatrix) {
    std::cout << "[PASS] " << entry.design_id << " -> " << entry.test_anchor
              << '\n';
  }
  for (const DesignTestLink& link : kSupportingLinks) {
    std::cout << "[SUPPORT] " << link.design_id << " -> " << link.test_anchor
              << '\n';
  }
  std::cout << "T44 Level 1 terrain/traversability matrix passed: "
            << kMatrix.size() << '/' << expected.size() << '\n';
  return EXIT_SUCCESS;
}
