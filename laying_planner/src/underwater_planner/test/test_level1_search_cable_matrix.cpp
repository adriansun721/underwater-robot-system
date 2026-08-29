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
  hybrid_astar,
  reference_progress,
  merge_goal,
  parameter_config,
  cable_state_tracker,
  cable_model,
  cable_corridor,
  cable_laying,
  envelope_builder,
  envelope_manager,
  timed_candidate,
  trajectory_parameterizer,
  count,
};

struct DesignTestLink {
  const char* design_id;
  TestSource source;
  const char* test_anchor;
};

constexpr std::array<DesignTestLink, 64> kMatrix{{
    {"18.2.3-1", TestSource::hybrid_astar,
     "flat_straight_search_returns_a_reference_aligned_touchdown_path"},
    {"18.2.3-2", TestSource::hybrid_astar,
     "a_single_open_side_is_used_before_returning_to_a_merge_goal"},
    {"18.2.3-3", TestSource::hybrid_astar,
     "two_open_sides_choose_the_lower_soft_cost_detour"},
    {"18.2.3-4", TestSource::hybrid_astar,
     "a_near_footprint_width_passage_preserves_the_safe_straight_route"},
    {"18.2.3-5", TestSource::hybrid_astar,
     "the_locked_envelope_is_a_hard_search_corridor_gate"},
    {"18.2.3-6", TestSource::hybrid_astar,
     "an_obstacle_between_clear_primitive_endpoints_is_rejected"},
    {"18.2.3-7", TestSource::hybrid_astar,
     "a_turning_footprint_corner_sweep_rejects_an_obstacle"},
    {"18.2.3-8", TestSource::hybrid_astar,
     "a_corridor_excursion_between_legal_touchdown_endpoints_is_rejected"},
    {"18.2.3-9", TestSource::hybrid_astar,
     "primitive_length_and_touchdown_sampling_density_preserve_cost_and_gates"},
    {"18.2.3-10", TestSource::hybrid_astar,
     "a_lower_cost_equivalent_history_reopens_and_stales_the_old_queue_entry"},
    {"18.2.3-11", TestSource::hybrid_astar,
     "mechanically_distinct_histories_share_a_base_key_without_merging"},
    {"18.2.3-12", TestSource::cable_laying,
     "future_equivalence_compares_memory_not_only_hashes"},
    {"18.2.3-13", TestSource::hybrid_astar,
     "active_label_budget_exhaustion_is_a_distinct_timeout"},
    {"18.2.3-14", TestSource::hybrid_astar,
     "a_reference_crossing_does_not_merge_distinct_progress_phases"},
    {"18.2.3-15", TestSource::reference_progress,
     "short_primitive_does_not_jump_to_a_nearby_competing_branch"},
    {"18.2.3-16", TestSource::hybrid_astar,
     "robot_and_cable_spatial_domains_are_evaluated_independently"},
    {"18.2.3-17", TestSource::hybrid_astar,
     "a_robot_goal_match_cannot_substitute_for_the_touchdown_goal"},
    {"18.2.3-18", TestSource::merge_goal,
     "inverse_targets_close_through_the_forward_cable_model"},
    {"18.2.3-19", TestSource::hybrid_astar,
     "feasible_solution_cost_is_explicit_and_touchdown_only"},
    {"18.2.3-invariant-1", TestSource::hybrid_astar,
     "returned_solution_satisfies_all_segment_hard_invariants"},
    {"18.2.3-invariant-2", TestSource::hybrid_astar,
     "returned_solution_satisfies_all_segment_hard_invariants"},
    {"18.2.3-invariant-3", TestSource::hybrid_astar,
     "returned_solution_satisfies_all_segment_hard_invariants"},
    {"18.2.3-invariant-4", TestSource::hybrid_astar,
     "returned_solution_satisfies_all_segment_hard_invariants"},

    {"18.2.4-1", TestSource::cable_model,
     "straight_prediction_uses_release_offset_and_touchdown_distance"},
    {"18.2.4-2", TestSource::cable_model,
     "constant_curvature_turns_are_left_right_symmetric"},
    {"18.2.4-3", TestSource::cable_model,
     "constant_curvature_turns_are_left_right_symmetric"},
    {"18.2.4-4", TestSource::cable_state_tracker,
     "rolling_window_updates_are_continuous_and_fail_closed_on_a_gap"},
    {"18.2.4-5", TestSource::cable_model,
     "finer_search_integration_converges_to_validation"},
    {"18.2.4-6", TestSource::cable_model,
     "timed_validation_reads_the_complete_execution_profile"},
    {"18.2.4-7", TestSource::cable_model,
     "straight_prediction_uses_release_offset_and_touchdown_distance"},
    {"18.2.4-8", TestSource::cable_corridor,
     "anisotropic_covariance_projection_rotates_with_reference_normal"},
    {"18.2.4-9", TestSource::cable_corridor,
     "evaluates_touchdown_lateral_risk_and_boundary_classes"},
    {"18.2.4-10", TestSource::cable_corridor,
     "evaluates_touchdown_lateral_risk_and_boundary_classes"},
    {"18.2.4-11", TestSource::timed_candidate,
     "map_confidence_is_a_hard_gate_not_a_covariance_scale"},
    {"18.2.4-12", TestSource::cable_corridor,
     "missing_risk_evidence_fails_closed"},
    {"18.2.4-13", TestSource::parameter_config,
     "laying_success_ratio_does_not_derive_pointwise_epsilon"},
    {"18.2.4-14", TestSource::cable_model,
     "covariance_retains_robot_tracking_history"},
    {"18.2.4-15", TestSource::envelope_builder,
     "actual_cable_model_covariance_is_covered_by_the_envelope"},
    {"18.2.4-16", TestSource::envelope_manager,
     "progress_queries_use_adjacent_upper_bounds_and_certified_margins"},
    {"18.2.4-17", TestSource::envelope_manager,
     "covariance_breach_has_system_failure_semantics"},
    {"18.2.4-18", TestSource::envelope_manager,
     "context_changes_atomically_invalidate_envelope_plan_and_lease"},
    {"18.2.4-19", TestSource::hybrid_astar,
     "flat_straight_search_returns_a_reference_aligned_touchdown_path"},
    {"18.2.4-20", TestSource::cable_laying,
     "left_and_right_curvature_share_the_hard_limit"},
    {"18.2.4-21", TestSource::cable_laying,
     "zero_soft_weights_cannot_bypass_mechanical_hard_limits"},
    {"18.2.4-22", TestSource::cable_laying,
     "swept_forbidden_unknown_and_low_confidence_cells_fail_hard"},
    {"18.2.4-23", TestSource::cable_laying,
     "support_proxy_is_physical_window_and_sampling_invariant"},
    {"18.2.4-24", TestSource::timed_candidate,
     "full_timed_candidate_rechecks_actual_history_boundary"},
    {"18.2.4-25", TestSource::hybrid_astar,
     "a_higher_cost_mechanical_history_can_reach_the_only_matching_goal"},
    {"18.2.4-26", TestSource::cable_laying,
     "actual_history_participates_in_first_candidate_curvature"},
    {"18.2.4-27", TestSource::cable_laying,
     "actual_history_is_required_and_requeried_on_the_current_map"},
    {"18.2.4-28", TestSource::cable_model,
     "untimed_and_invalid_execution_profiles_fail_closed"},
    {"18.2.4-29", TestSource::timed_candidate,
     "slower_timed_profile_is_repredicted_under_a_new_version"},
    {"18.2.4-30", TestSource::trajectory_parameterizer,
     "nominal_profile_is_complete_and_geometry_is_immutable"},
    {"18.2.4-31", TestSource::envelope_manager,
     "complete_tuple_is_required_before_an_envelope_can_be_locked"},
    {"18.2.4-32", TestSource::envelope_manager,
     "context_changes_atomically_invalidate_envelope_plan_and_lease"},
    {"18.2.4-33", TestSource::envelope_builder,
     "certified_build_records_dependencies_and_independent_margins"},
    {"18.2.4-34", TestSource::cable_corridor,
     "marginal_length_uses_interval_intersection_and_a_hard_limit"},

    {"18.2.4-key-1", TestSource::cable_corridor,
     "evaluates_touchdown_lateral_risk_and_boundary_classes"},
    {"18.2.4-key-2", TestSource::cable_model,
     "initial_lag_changes_touchdown_without_state_aliasing"},
    {"18.2.4-key-3", TestSource::hybrid_astar,
     "a_reference_crossing_does_not_merge_distinct_progress_phases"},
    {"18.2.4-key-4", TestSource::cable_model,
     "timed_validation_reads_the_complete_execution_profile"},
    {"18.2.4-key-5", TestSource::timed_candidate,
     "full_timed_candidate_rechecks_actual_history_boundary"},
    {"18.2.4-key-6", TestSource::timed_candidate,
     "validates_complete_timed_candidate_and_returns_terminal_state"},
    {"18.2.4-key-7", TestSource::cable_corridor,
     "marginal_length_uses_interval_intersection_and_a_hard_limit"},
}};

constexpr std::array<DesignTestLink, 1> kSupportingLinks{{
    {"18.2.4-30", TestSource::envelope_builder,
     "execution_domain_and_partial_lag_intersections_fail_closed"},
}};

constexpr std::array<const char*, static_cast<std::size_t>(TestSource::count)>
    kSourceNames{{
        "test_hybrid_astar_planner.cpp",
        "test_reference_progress_tracker.cpp",
        "test_merge_goal_generator.cpp",
        "test_parameter_config.cpp",
        "test_cable_state_tracker.cpp",
        "test_cable_model.cpp",
        "test_cable_corridor_evaluator.cpp",
        "test_cable_laying_evaluator.cpp",
        "test_cable_uncertainty_envelope_builder.cpp",
        "test_cable_uncertainty_envelope_manager.cpp",
        "test_timed_cable_candidate_verifier.cpp",
        "test_trajectory_parameterizer.cpp",
    }};

std::vector<std::string> expected_design_ids() {
  std::vector<std::string> expected;
  for (int item = 1; item <= 19; ++item) {
    expected.push_back("18.2.3-" + std::to_string(item));
  }
  for (int item = 1; item <= 4; ++item) {
    expected.push_back("18.2.3-invariant-" + std::to_string(item));
  }
  for (int item = 1; item <= 34; ++item) {
    expected.push_back("18.2.4-" + std::to_string(item));
  }
  for (int item = 1; item <= 7; ++item) {
    expected.push_back("18.2.4-key-" + std::to_string(item));
  }
  return expected;
}

[[noreturn]] void fail(const std::string& message) {
  std::cerr << "T45 failure: " << message << '\n';
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
  const std::string declaration = "void " + std::string{entry.test_anchor} + "(";
  const std::size_t function_start = contents.find(declaration);
  if (function_start == std::string::npos) return false;
  const std::size_t next_function = contents.find("\nvoid ", function_start + 1U);
  const std::string marker = "// Design: " + std::string{entry.design_id};
  const std::size_t marker_position = contents.find(marker, function_start);
  return marker_position != std::string::npos &&
         (next_function == std::string::npos || marker_position < next_function);
}

}  // namespace

int main(const int argc, const char* const argv[]) {
  constexpr int kExpectedExecutableCount =
      static_cast<int>(TestSource::count);
  if (argc != kExpectedExecutableCount + 1) {
    fail("expected all mapped T03 and T11-T29 test executable paths");
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
      fail("matrix is missing or reorders design item " + expected[index]);
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
      fail("mapped search/cable test executable failed: " +
           std::string{argv[executable]});
    }
  }

  for (const DesignTestLink& entry : kMatrix) {
    std::cout << "[PASS] " << entry.design_id << " -> " << entry.test_anchor
              << '\n';
  }
  for (const DesignTestLink& link : kSupportingLinks) {
    std::cout << "[SUPPORT] " << link.design_id << " -> "
              << link.test_anchor << '\n';
  }
  std::cout << "T45 Level 1 search/cable matrix passed: " << kMatrix.size()
            << '/' << expected.size() << '\n';
  return EXIT_SUCCESS;
}
