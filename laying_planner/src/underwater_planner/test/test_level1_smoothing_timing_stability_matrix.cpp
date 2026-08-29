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
  path_smoother,
  path_candidate_verifier,
  trajectory_parameterizer,
  data_contract,
  stability_manager,
  plan_validity_evaluator,
  execution_lease_monitor,
  synchronized_inputs,
  commitment_safety,
  main_planning_loop,
  count,
};

struct DesignTestLink {
  const char* design_id;
  TestSource source;
  const char* test_anchor;
};

constexpr std::array<DesignTestLink, 44> kMatrix{{
    {"18.2.5-1", TestSource::path_smoother,
     "a_committed_segment_terminal_is_an_accepted_nonzero_g2_source"},
    {"18.2.5-2", TestSource::path_smoother,
     "left_and_right_curvature_profiles_are_mirror_symmetric"},
    {"18.2.5-3", TestSource::path_smoother,
     "a_false_solver_convergence_cannot_bypass_residual_checks"},
    {"18.2.5-4", TestSource::path_candidate_verifier,
     "g2_merge_rejects_junction_mismatch"},
    {"18.2.5-5", TestSource::path_candidate_verifier,
     "metadata_tampering_is_detected"},
    {"18.2.5-6", TestSource::main_planning_loop,
     "untrackable_raw_path_preserves_the_smoothing_timeout_root_cause"},
    {"18.2.5-7", TestSource::path_candidate_verifier,
     "merged_path_sweep_rechecks_the_committed_prefix_on_the_current_map"},
    {"18.2.5-8", TestSource::path_smoother,
     "a_false_solver_convergence_cannot_bypass_residual_checks"},
    {"18.2.5-9", TestSource::path_smoother,
     "output_sampling_interval_does_not_change_g2_or_curvature_extrema"},
    {"18.2.5-10", TestSource::path_smoother,
     "missing_actual_curvature_is_not_replaced_with_zero"},
    {"18.2.5-11", TestSource::path_smoother,
     "translating_the_problem_does_not_add_a_reference_line_proxy_cost"},
    {"18.2.5-12", TestSource::path_candidate_verifier,
     "independent_audit_enforces_start_curvature_provenance"},
    {"18.2.5-invariant-1", TestSource::path_smoother,
     "straight_path_is_parameterized_by_one_clothoid_curve"},
    {"18.2.5-invariant-2", TestSource::path_candidate_verifier,
     "g2_merge_rejects_junction_mismatch"},
    {"18.2.5-invariant-3", TestSource::path_smoother,
     "solver_timeout_does_not_publish_the_unvalidated_raw_path"},

    {"18.2.6-1", TestSource::trajectory_parameterizer,
     "nominal_profile_is_complete_and_geometry_is_immutable"},
    {"18.2.6-2", TestSource::trajectory_parameterizer,
     "nominal_profile_is_complete_and_geometry_is_immutable"},
    {"18.2.6-3", TestSource::trajectory_parameterizer,
     "nominal_profile_is_complete_and_geometry_is_immutable"},
    {"18.2.6-4", TestSource::trajectory_parameterizer,
     "stopping_distance_is_hard_constraint"},
    {"18.2.6-5", TestSource::data_contract,
     "timed_paths_require_strict_time_and_matching_geometry"},
    {"18.2.6-6", TestSource::data_contract,
     "execution_profile_versions_track_every_semantic_change"},
    {"18.2.6-7", TestSource::stability_manager,
     "commitment_prefix_is_immutable_and_timed_merge_checks_continuity"},

    {"18.2.7-1", TestSource::main_planning_loop,
     "slight_improvement_keeps_current_with_latest_context_lease"},
    {"18.2.7-2", TestSource::main_planning_loop,
     "significant_improvement_switches_after_paired_revalidation"},
    {"18.2.7-3", TestSource::stability_manager,
     "repeated_small_perturbations_do_not_oscillate"},
    {"18.2.7-4", TestSource::plan_validity_evaluator,
     "revalidation_ignores_cached_cable_path_and_repredicts"},
    {"18.2.7-5", TestSource::execution_lease_monitor,
     "context_changes_revoke_and_block_subsequent_commands"},
    {"18.2.7-6", TestSource::main_planning_loop,
     "invalid_current_plan_allows_only_a_finite_candidate_direct_switch"},
    {"18.2.7-7", TestSource::plan_validity_evaluator,
     "revalidation_ignores_cached_cable_path_and_repredicts"},
    {"18.2.7-8", TestSource::execution_lease_monitor,
     "plan_profile_and_lease_pairing_fail_closed"},
    {"18.2.7-9", TestSource::main_planning_loop,
     "expired_during_candidate_planning_is_paired_revalidated"},
    {"18.2.7-10", TestSource::execution_lease_monitor,
     "context_changes_revoke_and_block_subsequent_commands"},
    {"18.2.7-11", TestSource::execution_lease_monitor,
     "deviation_revokes_and_stale_lease_is_rejected"},
    {"18.2.7-12", TestSource::execution_lease_monitor,
     "plan_profile_and_lease_pairing_fail_closed"},
    {"18.2.7-13", TestSource::execution_lease_monitor,
     "plan_profile_and_lease_pairing_fail_closed"},
    {"18.2.7-14", TestSource::plan_validity_evaluator,
     "successful_recheck_crops_and_issues_a_new_lease"},
    {"18.2.7-15", TestSource::main_planning_loop,
     "expired_during_planning_lease_is_replaced_not_reused"},
    {"18.2.7-16", TestSource::synchronized_inputs,
     "a_mid_capture_change_rejects_the_whole_snapshot"},
    {"18.2.7-17", TestSource::synchronized_inputs,
     "tracking_state_is_required_and_profile_bound"},
    {"18.2.7-18", TestSource::commitment_safety,
     "asynchronous_supervisor_revokes_before_stop_channel"},
    {"18.2.7-19", TestSource::main_planning_loop,
     "invalid_candidate_cost_keeps_valid_current_or_stops"},
    {"18.2.7-20", TestSource::main_planning_loop,
     "mismatched_validation_context_stops_before_hysteresis"},
    {"18.2.7-21", TestSource::plan_validity_evaluator,
     "publication_candidate_may_advance_the_execution_profile_version"},
    {"18.2.7-22", TestSource::main_planning_loop,
     "invalid_candidate_is_paired_with_current_before_the_keep_decision"},
}};

constexpr std::array<DesignTestLink, 10> kSupportingLinks{{
    {"18.2.5-10", TestSource::path_smoother,
     "pose_and_curvature_timestamps_must_be_synchronized"},
    {"18.2.6-2", TestSource::trajectory_parameterizer,
     "curved_path_obeys_lateral_acceleration"},
    {"18.2.7-10", TestSource::main_planning_loop,
     "dependency_change_breaks_commitment_before_analysis_or_search"},
    {"18.2.7-4", TestSource::main_planning_loop,
     "timeout_revalidation_failure_cannot_reuse_the_old_plan"},
    {"18.2.7-17", TestSource::synchronized_inputs,
     "stale_or_future_inputs_are_rejected_by_class"},
    {"18.2.7-18", TestSource::main_planning_loop,
     "urgent_commitment_safety_event_revokes_before_stop_and_search"},
    {"18.2.7-18", TestSource::main_planning_loop,
     "revoked_commitment_cannot_be_reused_on_the_next_cycle"},
    {"T46-safety-invariant", TestSource::main_planning_loop,
     "every_t46_safety_failure_blocks_followup_commands"},
    {"T65-cost-invariant", TestSource::main_planning_loop,
     "invalid_cost_cannot_cross_atomic_authorization_boundary"},
    {"T65-config-invariant", TestSource::main_planning_loop,
     "configured_hysteresis_reaches_the_nonvirtual_decision_seam"},
}};

constexpr std::array<const char*, static_cast<std::size_t>(TestSource::count)>
    kSourceNames{{
        "test_path_smoother.cpp",
        "test_path_candidate_verifier.cpp",
        "test_trajectory_parameterizer.cpp",
        "test_data_contract.cpp",
        "test_stability_manager.cpp",
        "test_plan_validity_evaluator.cpp",
        "test_execution_lease_monitor.cpp",
        "test_synchronized_validation_inputs.cpp",
        "test_commitment_safety.cpp",
        "test_main_planning_loop.cpp",
    }};

std::vector<std::string> expected_design_ids() {
  std::vector<std::string> expected;
  for (int item = 1; item <= 12; ++item) {
    expected.push_back("18.2.5-" + std::to_string(item));
  }
  for (int item = 1; item <= 3; ++item) {
    expected.push_back("18.2.5-invariant-" + std::to_string(item));
  }
  for (int item = 1; item <= 7; ++item) {
    expected.push_back("18.2.6-" + std::to_string(item));
  }
  for (int item = 1; item <= 22; ++item) {
    expected.push_back("18.2.7-" + std::to_string(item));
  }
  return expected;
}

[[noreturn]] void fail(const std::string& message) {
  std::cerr << "T46 failure: " << message << '\n';
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
  const std::string declaration =
      "void " + std::string{entry.test_anchor} + "(";
  const std::size_t function_start = contents.find(declaration);
  if (function_start == std::string::npos) return false;
  const std::size_t next_function =
      contents.find("\nvoid ", function_start + 1U);
  const std::string marker = "// Design: " + std::string{entry.design_id};
  const std::size_t marker_position = contents.find(marker, function_start);
  return marker_position != std::string::npos &&
         (next_function == std::string::npos || marker_position < next_function);
}

void validate_link(const DesignTestLink& link,
                   const std::array<std::string, kSourceNames.size()>& sources,
                   const char* kind) {
  const std::size_t source_index = static_cast<std::size_t>(link.source);
  if (source_index >= sources.size() ||
      count_occurrences(sources[source_index], link.test_anchor) < 2U ||
      !function_carries_design_marker(sources[source_index], link)) {
    fail(std::string{kind} + " test must be marked, defined, and invoked for " +
         link.design_id + ": " + link.test_anchor);
  }
}

}  // namespace

int main(const int argc, const char* const argv[]) {
  constexpr int kExpectedExecutableCount =
      static_cast<int>(TestSource::count);
  if (argc != kExpectedExecutableCount + 1) {
    fail("expected the ten T05/T26-T28/T31-T34/T38-T40 test executables");
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
    validate_link(entry, sources, "mapped");
  }
  for (const DesignTestLink& link : kSupportingLinks) {
    validate_link(link, sources, "supporting");
  }

  for (int executable = 1; executable < argc; ++executable) {
    const int exit_code =
        std::system(quote_command_argument(argv[executable]).c_str());
    if (exit_code != 0) {
      fail("mapped T05/T26-T28/T31-T34/T38-T40 test executable failed: " +
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
  std::cout << "T46 Level 1 smoothing/timing/stability matrix passed: "
            << kMatrix.size() << '/' << expected.size() << '\n';
  return EXIT_SUCCESS;
}
