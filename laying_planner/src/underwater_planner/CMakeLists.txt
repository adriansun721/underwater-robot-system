function(underwater_planner_enable_strict_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /WX /permissive-)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
endfunction()

add_library(underwater_planner_core
  src/algorithm_diagnostics.cpp
  src/communication_recovery.cpp
  src/cable_corridor_evaluator.cpp
  src/cable_laying_evaluator.cpp
  src/cable_model.cpp
  src/cable_state_tracker.cpp
  src/cable_uncertainty_envelope_builder.cpp
  src/cable_uncertainty_envelope_manager.cpp
  src/commitment_safety.cpp
  src/core.cpp
  src/data_contract.cpp
  src/execution_lease_monitor.cpp
  src/hybrid_astar_planner.cpp
  src/main_planning_loop.cpp
  src/message_consistency.cpp
  src/merge_goal_generator.cpp
  src/path_candidate_verifier.cpp
  src/planning_state_machine.cpp
  src/planning_result.cpp
  src/plan_validity_evaluator.cpp
  src/path_smoother.cpp
  src/trajectory_parameterizer.cpp
  src/timed_cable_candidate_verifier.cpp
  src/parameter_config.cpp
  src/scout_coordinator.cpp
  src/reference_progress_tracker.cpp
  src/stability_manager.cpp
  src/storage_estimation.cpp
  src/synchronized_validation_inputs.cpp
  src/terrain_analysis_primitives.cpp
  src/step_geometry.cpp
  src/terrain_analyzer.cpp
  src/traversability_evaluator.cpp
  src/versioned_snapshot.cpp
)
add_library(underwater_planner::core ALIAS underwater_planner_core)
target_compile_features(underwater_planner_core PUBLIC cxx_std_17)
target_include_directories(underwater_planner_core PUBLIC include)
underwater_planner_enable_strict_warnings(underwater_planner_core)
if(WIN32)
  target_link_libraries(underwater_planner_core PRIVATE Psapi)
endif()

add_library(underwater_planner_test_support
  src/deterministic_closed_loop.cpp
  src/deterministic_closed_loop_report.cpp
  src/synthetic_fixture.cpp
)
add_library(underwater_planner::test_support ALIAS underwater_planner_test_support)
target_compile_features(underwater_planner_test_support PUBLIC cxx_std_17)
target_include_directories(underwater_planner_test_support PUBLIC include)
target_link_libraries(underwater_planner_test_support PUBLIC
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(underwater_planner_test_support)

add_library(underwater_planner_static_analysis OBJECT EXCLUDE_FROM_ALL
  src/algorithm_diagnostics.cpp
  src/communication_recovery.cpp
  src/cable_corridor_evaluator.cpp
  src/cable_laying_evaluator.cpp
  src/cable_model.cpp
  src/cable_state_tracker.cpp
  src/cable_uncertainty_envelope_builder.cpp
  src/cable_uncertainty_envelope_manager.cpp
  src/commitment_safety.cpp
  src/core.cpp
  src/data_contract.cpp
  src/deterministic_closed_loop.cpp
  src/deterministic_closed_loop_report.cpp
  src/execution_lease_monitor.cpp
  src/hybrid_astar_planner.cpp
  src/main_planning_loop.cpp
  src/message_consistency.cpp
  src/merge_goal_generator.cpp
  src/path_candidate_verifier.cpp
  src/planning_state_machine.cpp
  src/planning_result.cpp
  src/plan_validity_evaluator.cpp
  src/path_smoother.cpp
  src/trajectory_parameterizer.cpp
  src/timed_cable_candidate_verifier.cpp
  src/parameter_config.cpp
  src/scout_coordinator.cpp
  src/reference_progress_tracker.cpp
  src/stability_manager.cpp
  src/storage_estimation.cpp
  src/synthetic_fixture.cpp
  src/synchronized_validation_inputs.cpp
  src/terrain_analysis_primitives.cpp
  src/step_geometry.cpp
  src/terrain_analyzer.cpp
  src/traversability_evaluator.cpp
  src/versioned_snapshot.cpp
)
target_compile_features(underwater_planner_static_analysis PRIVATE cxx_std_17)
target_include_directories(underwater_planner_static_analysis PRIVATE include)
underwater_planner_enable_strict_warnings(underwater_planner_static_analysis)
if(MSVC)
  target_compile_options(underwater_planner_static_analysis PRIVATE /analyze)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_compile_options(underwater_planner_static_analysis PRIVATE -fanalyzer)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  find_program(UNDERWATER_PLANNER_CLANG_TIDY NAMES clang-tidy REQUIRED)
  set_target_properties(underwater_planner_static_analysis PROPERTIES
    CXX_CLANG_TIDY "${UNDERWATER_PLANNER_CLANG_TIDY};--warnings-as-errors=*"
  )
else()
  message(FATAL_ERROR
    "No static analyzer is configured for ${CMAKE_CXX_COMPILER_ID}"
  )
endif()
add_custom_target(static-check DEPENDS underwater_planner_static_analysis)

add_executable(underwater_planner_fixture_tests test/test_synthetic_fixture.cpp)
target_compile_features(underwater_planner_fixture_tests PRIVATE cxx_std_17)
target_link_libraries(underwater_planner_fixture_tests PRIVATE
  underwater_planner::core
  underwater_planner::test_support
)
underwater_planner_enable_strict_warnings(underwater_planner_fixture_tests)
if(WIN32)
  target_link_libraries(underwater_planner_fixture_tests PRIVATE Psapi)
endif()

add_test(NAME synthetic_fixtures COMMAND underwater_planner_fixture_tests)
set_tests_properties(synthetic_fixtures PROPERTIES LABELS "unit;deterministic")

add_executable(underwater_planner_contract_tests test/test_data_contract.cpp)
target_compile_features(underwater_planner_contract_tests PRIVATE cxx_std_17)
target_link_libraries(underwater_planner_contract_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(underwater_planner_contract_tests)

add_test(NAME data_contract COMMAND underwater_planner_contract_tests)
set_tests_properties(data_contract PROPERTIES LABELS "unit;contract")

add_executable(underwater_planner_parameter_tests test/test_parameter_config.cpp)
target_compile_features(underwater_planner_parameter_tests PRIVATE cxx_std_17)
target_link_libraries(underwater_planner_parameter_tests PRIVATE underwater_planner::core)
underwater_planner_enable_strict_warnings(underwater_planner_parameter_tests)

add_test(NAME parameter_config COMMAND underwater_planner_parameter_tests)
set_tests_properties(parameter_config PROPERTIES LABELS "unit;parameters;deterministic")

add_executable(underwater_planner_snapshot_tests test/test_versioned_snapshot.cpp)
target_compile_features(underwater_planner_snapshot_tests PRIVATE cxx_std_17)
target_link_libraries(underwater_planner_snapshot_tests PRIVATE underwater_planner::core)
underwater_planner_enable_strict_warnings(underwater_planner_snapshot_tests)
add_test(NAME versioned_snapshots COMMAND underwater_planner_snapshot_tests)
set_tests_properties(versioned_snapshots PROPERTIES LABELS "unit;contract;deterministic")

add_executable(underwater_planner_synchronized_input_tests
  test/test_synchronized_validation_inputs.cpp
)
target_compile_features(underwater_planner_synchronized_input_tests PRIVATE cxx_std_17)
target_link_libraries(underwater_planner_synchronized_input_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(underwater_planner_synchronized_input_tests)
add_test(NAME synchronized_validation_inputs
  COMMAND underwater_planner_synchronized_input_tests
)
set_tests_properties(synchronized_validation_inputs PROPERTIES
  LABELS "unit;contract;deterministic;concurrency"
)

add_executable(underwater_planner_terrain_analyzer_tests
  test/test_terrain_analyzer.cpp
)
target_compile_features(underwater_planner_terrain_analyzer_tests PRIVATE cxx_std_17)
target_link_libraries(underwater_planner_terrain_analyzer_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(underwater_planner_terrain_analyzer_tests)
add_test(NAME terrain_analyzer COMMAND underwater_planner_terrain_analyzer_tests)
set_tests_properties(terrain_analyzer PROPERTIES
  LABELS "unit;terrain;deterministic"
)

add_executable(underwater_planner_step_geometry_tests
  test/test_step_geometry.cpp
)
target_compile_features(underwater_planner_step_geometry_tests PRIVATE cxx_std_17)
target_link_libraries(underwater_planner_step_geometry_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(underwater_planner_step_geometry_tests)
add_test(NAME step_geometry COMMAND underwater_planner_step_geometry_tests)
set_tests_properties(step_geometry PROPERTIES
  LABELS "unit;terrain;deterministic"
)

add_executable(underwater_planner_traversability_tests
  test/test_traversability_evaluator.cpp
)
target_compile_features(underwater_planner_traversability_tests PRIVATE cxx_std_17)
target_link_libraries(underwater_planner_traversability_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(underwater_planner_traversability_tests)
add_test(NAME traversability_evaluator
  COMMAND underwater_planner_traversability_tests
)
set_tests_properties(traversability_evaluator PROPERTIES
  LABELS "unit;terrain;collision;deterministic"
)

add_executable(underwater_planner_directional_slope_tests
  test/test_directional_slope.cpp
)
target_compile_features(underwater_planner_directional_slope_tests PRIVATE cxx_std_17)
target_link_libraries(underwater_planner_directional_slope_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(underwater_planner_directional_slope_tests)
add_test(NAME directional_slope
  COMMAND underwater_planner_directional_slope_tests
)
set_tests_properties(directional_slope PROPERTIES
  LABELS "unit;terrain;traversability;deterministic"
)

add_executable(underwater_planner_step_traversability_tests
  test/test_step_traversability.cpp
)
target_compile_features(underwater_planner_step_traversability_tests PRIVATE cxx_std_17)
target_link_libraries(underwater_planner_step_traversability_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(underwater_planner_step_traversability_tests)
add_test(NAME step_traversability
  COMMAND underwater_planner_step_traversability_tests
)
set_tests_properties(step_traversability PROPERTIES
  LABELS "unit;terrain;traversability;deterministic"
)

add_executable(underwater_planner_level1_terrain_traversability_tests
  test/test_level1_terrain_traversability_matrix.cpp
)
target_compile_features(
  underwater_planner_level1_terrain_traversability_tests PRIVATE cxx_std_17
)
target_compile_definitions(
  underwater_planner_level1_terrain_traversability_tests PRIVATE
  UNDERWATER_PLANNER_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test"
)
underwater_planner_enable_strict_warnings(
  underwater_planner_level1_terrain_traversability_tests
)
add_dependencies(
  underwater_planner_level1_terrain_traversability_tests
  underwater_planner_terrain_analyzer_tests
  underwater_planner_step_geometry_tests
  underwater_planner_traversability_tests
  underwater_planner_directional_slope_tests
  underwater_planner_step_traversability_tests
)
add_test(NAME level1_terrain_traversability_matrix
  COMMAND underwater_planner_level1_terrain_traversability_tests
    $<TARGET_FILE:underwater_planner_terrain_analyzer_tests>
    $<TARGET_FILE:underwater_planner_step_geometry_tests>
    $<TARGET_FILE:underwater_planner_traversability_tests>
    $<TARGET_FILE:underwater_planner_directional_slope_tests>
    $<TARGET_FILE:underwater_planner_step_traversability_tests>
)
set_tests_properties(level1_terrain_traversability_matrix PROPERTIES
  LABELS "acceptance;level1;terrain;traversability;deterministic"
)

add_executable(underwater_planner_cable_state_tracker_tests
  test/test_cable_state_tracker.cpp
)
target_compile_features(underwater_planner_cable_state_tracker_tests PRIVATE
  cxx_std_17
)
target_link_libraries(underwater_planner_cable_state_tracker_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(
  underwater_planner_cable_state_tracker_tests
)
add_test(NAME cable_state_tracker
  COMMAND underwater_planner_cable_state_tracker_tests
)
set_tests_properties(cable_state_tracker PROPERTIES
  LABELS "unit;cable;tracker;deterministic"
)

add_executable(underwater_planner_cable_model_tests
  test/test_cable_model.cpp
)
target_compile_features(underwater_planner_cable_model_tests PRIVATE
  cxx_std_17
)
target_link_libraries(underwater_planner_cable_model_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(
  underwater_planner_cable_model_tests
)
add_test(NAME cable_model COMMAND underwater_planner_cable_model_tests)
set_tests_properties(cable_model PROPERTIES
  LABELS "unit;cable;model;deterministic"
)

add_executable(underwater_planner_cable_laying_evaluator_tests
  test/test_cable_laying_evaluator.cpp
)
target_compile_features(underwater_planner_cable_laying_evaluator_tests PRIVATE
  cxx_std_17
)
target_link_libraries(underwater_planner_cable_laying_evaluator_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(
  underwater_planner_cable_laying_evaluator_tests
)
add_test(NAME cable_laying_evaluator
  COMMAND underwater_planner_cable_laying_evaluator_tests
)
set_tests_properties(cable_laying_evaluator PROPERTIES
  LABELS "unit;cable;mechanical;terrain;deterministic"
)

add_executable(underwater_planner_cable_corridor_evaluator_tests
  test/test_cable_corridor_evaluator.cpp
)
target_compile_features(underwater_planner_cable_corridor_evaluator_tests
  PRIVATE cxx_std_17
)
target_link_libraries(underwater_planner_cable_corridor_evaluator_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(
  underwater_planner_cable_corridor_evaluator_tests
)
add_test(NAME cable_corridor_evaluator
  COMMAND underwater_planner_cable_corridor_evaluator_tests
)
set_tests_properties(cable_corridor_evaluator PROPERTIES
  LABELS "unit;cable;corridor;risk;deterministic"
)

add_executable(underwater_planner_cable_uncertainty_envelope_builder_tests
  test/test_cable_uncertainty_envelope_builder.cpp
)
target_compile_features(
  underwater_planner_cable_uncertainty_envelope_builder_tests
  PRIVATE cxx_std_17
)
target_link_libraries(
  underwater_planner_cable_uncertainty_envelope_builder_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(
  underwater_planner_cable_uncertainty_envelope_builder_tests
)
add_test(NAME cable_uncertainty_envelope_builder
  COMMAND underwater_planner_cable_uncertainty_envelope_builder_tests
)
set_tests_properties(cable_uncertainty_envelope_builder PROPERTIES
  LABELS "unit;cable;envelope;risk;deterministic"
)

add_executable(underwater_planner_cable_uncertainty_envelope_manager_tests
  test/test_cable_uncertainty_envelope_manager.cpp
)
target_compile_features(
  underwater_planner_cable_uncertainty_envelope_manager_tests
  PRIVATE cxx_std_17
)
target_link_libraries(
  underwater_planner_cable_uncertainty_envelope_manager_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(
  underwater_planner_cable_uncertainty_envelope_manager_tests
)
add_test(NAME cable_uncertainty_envelope_manager
  COMMAND underwater_planner_cable_uncertainty_envelope_manager_tests
)
set_tests_properties(cable_uncertainty_envelope_manager PROPERTIES
  LABELS "unit;cable;envelope;risk;deterministic"
)

add_executable(underwater_planner_reference_progress_tracker_tests
  test/test_reference_progress_tracker.cpp
)
target_compile_features(underwater_planner_reference_progress_tracker_tests
  PRIVATE cxx_std_17
)
target_link_libraries(underwater_planner_reference_progress_tracker_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(
  underwater_planner_reference_progress_tracker_tests
)
add_test(NAME reference_progress_tracker
  COMMAND underwater_planner_reference_progress_tracker_tests
)
set_tests_properties(reference_progress_tracker PROPERTIES
  LABELS "unit;reference;tracker;deterministic"
)

add_executable(underwater_planner_merge_goal_generator_tests
  test/test_merge_goal_generator.cpp
)
target_compile_features(underwater_planner_merge_goal_generator_tests PRIVATE
  cxx_std_17
)
target_link_libraries(underwater_planner_merge_goal_generator_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(
  underwater_planner_merge_goal_generator_tests
)
add_test(NAME merge_goal_generator
  COMMAND underwater_planner_merge_goal_generator_tests
)
set_tests_properties(merge_goal_generator PROPERTIES
  LABELS "unit;search;cable;reference;deterministic"
)

add_executable(underwater_planner_hybrid_astar_tests
  test/test_hybrid_astar_planner.cpp
)
target_compile_features(underwater_planner_hybrid_astar_tests PRIVATE
  cxx_std_17
)
target_link_libraries(underwater_planner_hybrid_astar_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(
  underwater_planner_hybrid_astar_tests
)
add_test(NAME hybrid_astar_planner
  COMMAND underwater_planner_hybrid_astar_tests
)
set_tests_properties(hybrid_astar_planner PROPERTIES
  LABELS "unit;search;cable;reference;deterministic"
)

add_executable(underwater_planner_path_smoother_tests
  test/test_path_smoother.cpp
)
target_compile_features(underwater_planner_path_smoother_tests PRIVATE
  cxx_std_17
)
target_link_libraries(underwater_planner_path_smoother_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(underwater_planner_path_smoother_tests)
add_test(NAME path_smoother COMMAND underwater_planner_path_smoother_tests)
set_tests_properties(path_smoother PROPERTIES
  LABELS "unit;smoothing;deterministic"
)

add_executable(underwater_planner_path_candidate_verifier_tests
  test/test_path_candidate_verifier.cpp
)
target_compile_features(underwater_planner_path_candidate_verifier_tests PRIVATE
  cxx_std_17
)
target_link_libraries(underwater_planner_path_candidate_verifier_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(underwater_planner_path_candidate_verifier_tests)
add_test(NAME path_candidate_verifier
  COMMAND underwater_planner_path_candidate_verifier_tests
)
set_tests_properties(path_candidate_verifier PROPERTIES
  LABELS "unit;smoothing;geometry;traversability;deterministic"
)

add_executable(underwater_planner_trajectory_parameterizer_tests
  test/test_trajectory_parameterizer.cpp
)
target_compile_features(underwater_planner_trajectory_parameterizer_tests PRIVATE cxx_std_17)
target_link_libraries(underwater_planner_trajectory_parameterizer_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(underwater_planner_trajectory_parameterizer_tests)
add_test(NAME trajectory_parameterizer
  COMMAND underwater_planner_trajectory_parameterizer_tests
)
set_tests_properties(trajectory_parameterizer PROPERTIES
  LABELS "unit;execution;deterministic"
)

add_executable(underwater_planner_timed_cable_candidate_verifier_tests
  test/test_timed_cable_candidate_verifier.cpp
)
target_compile_features(underwater_planner_timed_cable_candidate_verifier_tests
  PRIVATE cxx_std_17
)
target_link_libraries(underwater_planner_timed_cable_candidate_verifier_tests
  PRIVATE underwater_planner::core
)
underwater_planner_enable_strict_warnings(
  underwater_planner_timed_cable_candidate_verifier_tests
)
add_test(NAME timed_cable_candidate_verifier
  COMMAND underwater_planner_timed_cable_candidate_verifier_tests
)
set_tests_properties(timed_cable_candidate_verifier PROPERTIES
  LABELS "unit;cable;validation;deterministic"
)

add_executable(underwater_planner_level1_search_cable_tests
  test/test_level1_search_cable_matrix.cpp
)
target_compile_features(
  underwater_planner_level1_search_cable_tests PRIVATE cxx_std_17
)
target_compile_definitions(
  underwater_planner_level1_search_cable_tests PRIVATE
  UNDERWATER_PLANNER_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test"
)
underwater_planner_enable_strict_warnings(
  underwater_planner_level1_search_cable_tests
)
add_dependencies(
  underwater_planner_level1_search_cable_tests
  underwater_planner_hybrid_astar_tests
  underwater_planner_reference_progress_tracker_tests
  underwater_planner_merge_goal_generator_tests
  underwater_planner_parameter_tests
  underwater_planner_cable_state_tracker_tests
  underwater_planner_cable_model_tests
  underwater_planner_cable_corridor_evaluator_tests
  underwater_planner_cable_laying_evaluator_tests
  underwater_planner_cable_uncertainty_envelope_builder_tests
  underwater_planner_cable_uncertainty_envelope_manager_tests
  underwater_planner_timed_cable_candidate_verifier_tests
  underwater_planner_trajectory_parameterizer_tests
)
add_test(NAME level1_search_cable_matrix
  COMMAND underwater_planner_level1_search_cable_tests
    $<TARGET_FILE:underwater_planner_hybrid_astar_tests>
    $<TARGET_FILE:underwater_planner_reference_progress_tracker_tests>
    $<TARGET_FILE:underwater_planner_merge_goal_generator_tests>
    $<TARGET_FILE:underwater_planner_parameter_tests>
    $<TARGET_FILE:underwater_planner_cable_state_tracker_tests>
    $<TARGET_FILE:underwater_planner_cable_model_tests>
    $<TARGET_FILE:underwater_planner_cable_corridor_evaluator_tests>
    $<TARGET_FILE:underwater_planner_cable_laying_evaluator_tests>
    $<TARGET_FILE:underwater_planner_cable_uncertainty_envelope_builder_tests>
    $<TARGET_FILE:underwater_planner_cable_uncertainty_envelope_manager_tests>
    $<TARGET_FILE:underwater_planner_timed_cable_candidate_verifier_tests>
    $<TARGET_FILE:underwater_planner_trajectory_parameterizer_tests>
)
set_tests_properties(level1_search_cable_matrix PROPERTIES
  LABELS "acceptance;level1;search;cable;deterministic"
)

add_executable(underwater_planner_planning_result_tests
  test/test_planning_result.cpp
)
target_compile_features(underwater_planner_planning_result_tests PRIVATE cxx_std_17)
target_link_libraries(underwater_planner_planning_result_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(underwater_planner_planning_result_tests)
add_test(NAME planning_result COMMAND underwater_planner_planning_result_tests)
set_tests_properties(planning_result PROPERTIES LABELS "unit;contract;deterministic")

add_executable(underwater_planner_plan_validity_evaluator_tests
  test/test_plan_validity_evaluator.cpp
)
target_compile_features(underwater_planner_plan_validity_evaluator_tests PRIVATE cxx_std_17)
target_link_libraries(underwater_planner_plan_validity_evaluator_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(
  underwater_planner_plan_validity_evaluator_tests
)
add_test(NAME plan_validity_evaluator
  COMMAND underwater_planner_plan_validity_evaluator_tests
)
set_tests_properties(plan_validity_evaluator PROPERTIES
  LABELS "unit;execution;validation;deterministic"
)

add_executable(underwater_planner_execution_lease_monitor_tests
  test/test_execution_lease_monitor.cpp
)
target_compile_features(underwater_planner_execution_lease_monitor_tests PRIVATE cxx_std_17)
target_link_libraries(underwater_planner_execution_lease_monitor_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(
  underwater_planner_execution_lease_monitor_tests
)
add_test(NAME execution_lease_monitor
  COMMAND underwater_planner_execution_lease_monitor_tests
)
set_tests_properties(execution_lease_monitor PROPERTIES
  LABELS "unit;execution;validation;deterministic;safety"
)

add_executable(underwater_planner_stability_manager_tests
  test/test_stability_manager.cpp
)
target_compile_features(underwater_planner_stability_manager_tests PRIVATE cxx_std_17)
target_link_libraries(underwater_planner_stability_manager_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(underwater_planner_stability_manager_tests)
add_test(NAME stability_manager COMMAND underwater_planner_stability_manager_tests)
set_tests_properties(stability_manager PROPERTIES
  LABELS "unit;execution;stability;deterministic;safety"
)

add_executable(underwater_planner_commitment_safety_tests
  test/test_commitment_safety.cpp
)
target_compile_features(underwater_planner_commitment_safety_tests PRIVATE
  cxx_std_17
)
target_link_libraries(underwater_planner_commitment_safety_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(
  underwater_planner_commitment_safety_tests
)
add_test(NAME commitment_safety
  COMMAND underwater_planner_commitment_safety_tests
)
set_tests_properties(commitment_safety PROPERTIES
  LABELS "unit;execution;stability;deterministic;safety"
)

add_executable(underwater_planner_scout_coordinator_tests
  test/test_scout_coordinator.cpp
)
target_compile_features(underwater_planner_scout_coordinator_tests PRIVATE cxx_std_17)
target_link_libraries(underwater_planner_scout_coordinator_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(underwater_planner_scout_coordinator_tests)
add_test(NAME scout_coordinator COMMAND underwater_planner_scout_coordinator_tests)
set_tests_properties(scout_coordinator PROPERTIES
  LABELS "unit;scout;deterministic;safety"
)

add_executable(underwater_planner_planning_state_machine_tests
  test/test_planning_state_machine.cpp
)
target_compile_features(underwater_planner_planning_state_machine_tests PRIVATE cxx_std_17)
target_link_libraries(underwater_planner_planning_state_machine_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(
  underwater_planner_planning_state_machine_tests
)
add_test(NAME planning_state_machine
  COMMAND underwater_planner_planning_state_machine_tests
)
set_tests_properties(planning_state_machine PROPERTIES
  LABELS "unit;planning;state-machine;deterministic;safety"
)

add_executable(underwater_planner_main_planning_loop_tests
  test/test_main_planning_loop.cpp
)
target_compile_features(underwater_planner_main_planning_loop_tests PRIVATE
  cxx_std_17
)
target_link_libraries(underwater_planner_main_planning_loop_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(
  underwater_planner_main_planning_loop_tests
)
add_test(NAME main_planning_loop COMMAND underwater_planner_main_planning_loop_tests)
set_tests_properties(main_planning_loop PROPERTIES
  LABELS "integration;planning;deterministic;safety"
)

add_executable(underwater_planner_level1_smoothing_timing_stability_tests
  test/test_level1_smoothing_timing_stability_matrix.cpp
)
target_compile_features(
  underwater_planner_level1_smoothing_timing_stability_tests PRIVATE cxx_std_17
)
target_compile_definitions(
  underwater_planner_level1_smoothing_timing_stability_tests PRIVATE
  UNDERWATER_PLANNER_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test"
)
underwater_planner_enable_strict_warnings(
  underwater_planner_level1_smoothing_timing_stability_tests
)
add_dependencies(
  underwater_planner_level1_smoothing_timing_stability_tests
  underwater_planner_path_smoother_tests
  underwater_planner_path_candidate_verifier_tests
  underwater_planner_trajectory_parameterizer_tests
  underwater_planner_contract_tests
  underwater_planner_stability_manager_tests
  underwater_planner_plan_validity_evaluator_tests
  underwater_planner_execution_lease_monitor_tests
  underwater_planner_synchronized_input_tests
  underwater_planner_commitment_safety_tests
  underwater_planner_main_planning_loop_tests
)
add_test(NAME level1_smoothing_timing_stability_matrix
  COMMAND underwater_planner_level1_smoothing_timing_stability_tests
    $<TARGET_FILE:underwater_planner_path_smoother_tests>
    $<TARGET_FILE:underwater_planner_path_candidate_verifier_tests>
    $<TARGET_FILE:underwater_planner_trajectory_parameterizer_tests>
    $<TARGET_FILE:underwater_planner_contract_tests>
    $<TARGET_FILE:underwater_planner_stability_manager_tests>
    $<TARGET_FILE:underwater_planner_plan_validity_evaluator_tests>
    $<TARGET_FILE:underwater_planner_execution_lease_monitor_tests>
    $<TARGET_FILE:underwater_planner_synchronized_input_tests>
    $<TARGET_FILE:underwater_planner_commitment_safety_tests>
    $<TARGET_FILE:underwater_planner_main_planning_loop_tests>
)
set_tests_properties(level1_smoothing_timing_stability_matrix PROPERTIES
  LABELS "acceptance;level1;smoothing;timing;stability;deterministic;safety"
)

add_executable(underwater_planner_message_consistency_tests
  test/test_message_consistency.cpp
)
target_compile_features(underwater_planner_message_consistency_tests PRIVATE
  cxx_std_17
)
target_link_libraries(underwater_planner_message_consistency_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(
  underwater_planner_message_consistency_tests
)
add_test(NAME message_consistency
  COMMAND underwater_planner_message_consistency_tests
)
set_tests_properties(message_consistency PROPERTIES
  LABELS "unit;messages;deterministic;safety"
)

add_executable(underwater_planner_algorithm_diagnostics_tests
  test/test_algorithm_diagnostics.cpp
)
target_compile_features(underwater_planner_algorithm_diagnostics_tests PRIVATE
  cxx_std_17
)
target_link_libraries(underwater_planner_algorithm_diagnostics_tests PRIVATE
  underwater_planner::core
)
underwater_planner_enable_strict_warnings(
  underwater_planner_algorithm_diagnostics_tests
)
add_test(NAME algorithm_diagnostics
  COMMAND underwater_planner_algorithm_diagnostics_tests
)
set_tests_properties(algorithm_diagnostics PROPERTIES
  LABELS "unit;diagnostics;replay;deterministic"
)

add_executable(underwater_planner_level1_closed_loop_tests
  test/test_level1_closed_loop_scenarios.cpp
)
target_compile_features(underwater_planner_level1_closed_loop_tests PRIVATE
  cxx_std_17
)
target_link_libraries(underwater_planner_level1_closed_loop_tests PRIVATE
  underwater_planner::core
  underwater_planner::test_support
)
underwater_planner_enable_strict_warnings(
  underwater_planner_level1_closed_loop_tests
)
add_test(NAME level1_closed_loop_scenarios
  COMMAND underwater_planner_level1_closed_loop_tests
    "${CMAKE_BINARY_DIR}/t47_closed_loop_report.json"
)
set_tests_properties(level1_closed_loop_scenarios PROPERTIES
  LABELS "acceptance;level1;closed-loop;deterministic;safety"
  FIXTURES_SETUP t47_closed_loop_report
)

add_test(NAME level1_closed_loop_report_json
  COMMAND ${CMAKE_COMMAND}
    -DREPORT_PATH=${CMAKE_BINARY_DIR}/t47_closed_loop_report.json
    -P ${CMAKE_CURRENT_SOURCE_DIR}/test/validate_closed_loop_report.cmake.in
)
set_tests_properties(level1_closed_loop_report_json PROPERTIES
  LABELS "acceptance;level1;closed-loop;deterministic;report"
  FIXTURES_REQUIRED t47_closed_loop_report
)
