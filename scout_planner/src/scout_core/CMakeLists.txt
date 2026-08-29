function(scout_enable_strict_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /WX /permissive-)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
endfunction()

find_program(SCOUT_PROTOC_EXECUTABLE NAMES protoc REQUIRED)
get_filename_component(SCOUT_PROTOBUF_BIN_DIR
  "${SCOUT_PROTOC_EXECUTABLE}" DIRECTORY
)
get_filename_component(SCOUT_PROTOBUF_PREFIX
  "${SCOUT_PROTOBUF_BIN_DIR}" DIRECTORY
)
list(PREPEND CMAKE_PREFIX_PATH "${SCOUT_PROTOBUF_PREFIX}")
find_package(Protobuf CONFIG REQUIRED)
find_package(ICU REQUIRED COMPONENTS uc)
find_package(Threads REQUIRED)

set(SCOUT_CONTRACT_PROTO_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../interfaces/proto"
)
set(SCOUT_GENERATED_PROTO_ROOT "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(SCOUT_CONTRACT_PROTO_FILES
  underwater/contracts/v1/capability.proto
  underwater/contracts/v1/codes.proto
  underwater/contracts/v1/common.proto
  underwater/contracts/v1/cooperation.proto
  underwater/contracts/v1/diagnostics.proto
  underwater/contracts/v1/execution.proto
  underwater/contracts/v1/mapping.proto
  underwater/contracts/v1/planning.proto
  underwater/contracts/v1/profiles.proto
  underwater/contracts/v1/sensing.proto
  underwater/contracts/v1/state.proto
)

set(SCOUT_CONTRACT_PROTO_SOURCES)
set(SCOUT_CONTRACT_PROTO_HEADERS)
foreach(proto_file IN LISTS SCOUT_CONTRACT_PROTO_FILES)
  string(REPLACE ".proto" ".pb.cc" generated_source "${proto_file}")
  string(REPLACE ".proto" ".pb.h" generated_header "${proto_file}")
  list(APPEND SCOUT_CONTRACT_PROTO_SOURCES
    "${SCOUT_GENERATED_PROTO_ROOT}/${generated_source}"
  )
  list(APPEND SCOUT_CONTRACT_PROTO_HEADERS
    "${SCOUT_GENERATED_PROTO_ROOT}/${generated_header}"
  )
  add_custom_command(
    OUTPUT
      "${SCOUT_GENERATED_PROTO_ROOT}/${generated_source}"
      "${SCOUT_GENERATED_PROTO_ROOT}/${generated_header}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory
      "${SCOUT_GENERATED_PROTO_ROOT}"
    COMMAND protobuf::protoc
      "--cpp_out=${SCOUT_GENERATED_PROTO_ROOT}"
      "--proto_path=${SCOUT_CONTRACT_PROTO_ROOT}"
      "${SCOUT_CONTRACT_PROTO_ROOT}/${proto_file}"
    DEPENDS "${SCOUT_CONTRACT_PROTO_ROOT}/${proto_file}"
    VERBATIM
  )
endforeach()

set(SCOUT_CONTRACT_DESCRIPTOR
  "${SCOUT_GENERATED_PROTO_ROOT}/contracts.pb"
)
set(SCOUT_CONTRACT_PROTO_ABSOLUTE_FILES)
foreach(proto_file IN LISTS SCOUT_CONTRACT_PROTO_FILES)
  list(APPEND SCOUT_CONTRACT_PROTO_ABSOLUTE_FILES
    "${SCOUT_CONTRACT_PROTO_ROOT}/${proto_file}"
  )
endforeach()
add_custom_command(
  OUTPUT "${SCOUT_CONTRACT_DESCRIPTOR}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory
    "${SCOUT_GENERATED_PROTO_ROOT}"
  COMMAND protobuf::protoc
    "--proto_path=${SCOUT_CONTRACT_PROTO_ROOT}"
    --include_imports
    "--descriptor_set_out=${SCOUT_CONTRACT_DESCRIPTOR}"
    ${SCOUT_CONTRACT_PROTO_ABSOLUTE_FILES}
  DEPENDS ${SCOUT_CONTRACT_PROTO_ABSOLUTE_FILES}
  VERBATIM
)
add_custom_target(golden-schema-check
  COMMAND "${CMAKE_COMMAND}"
    "-DDESCRIPTOR_FILE=${SCOUT_CONTRACT_DESCRIPTOR}"
    "-DEXPECTED_SHA256=44ded468a534e8c75722d98125d3eb49c2aebf3a7ab0f2ee8cecc251907cf7f1"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/verify_sha256.cmake"
  DEPENDS "${SCOUT_CONTRACT_DESCRIPTOR}"
  VERBATIM
)

add_library(scout_contract_proto
  ${SCOUT_CONTRACT_PROTO_SOURCES}
  ${SCOUT_CONTRACT_PROTO_HEADERS}
)
target_compile_features(scout_contract_proto PUBLIC cxx_std_17)
target_include_directories(scout_contract_proto SYSTEM PUBLIC
  "${SCOUT_GENERATED_PROTO_ROOT}"
)
target_link_libraries(scout_contract_proto PUBLIC protobuf::libprotobuf)
scout_enable_strict_warnings(scout_contract_proto)
if(MSVC)
  # protoc 29 emits intentionally unused arena/visibility parameters.
  target_compile_options(scout_contract_proto PRIVATE /wd4100 /wd4267)
endif()

add_library(scout_core
  src/core.cpp
  src/capability_energy_gate.cpp
  src/coordination_evaluator.cpp
  src/continuous_geometry_validator.cpp
  src/hybrid_map_query.cpp
  src/planning_context.cpp
  src/protobuf_adapter.cpp
  src/quintic_bezier.cpp
  src/trajectory_smoother.cpp
  src/state_lattice_astar.cpp
  src/survey_action.cpp
  src/survey_plan_evidence.cpp
)
add_library(scout::core ALIAS scout_core)
target_compile_features(scout_core PUBLIC cxx_std_17)
target_include_directories(scout_core PUBLIC include)
target_link_libraries(scout_core PUBLIC scout_contract_proto PRIVATE
  ICU::uc
  Threads::Threads
)
scout_enable_strict_warnings(scout_core)

add_library(scout_test_support src/deterministic_fixture.cpp)
add_library(scout::test_support ALIAS scout_test_support)
target_compile_features(scout_test_support PUBLIC cxx_std_17)
target_include_directories(scout_test_support PUBLIC include)
target_link_libraries(scout_test_support PUBLIC scout::core)
scout_enable_strict_warnings(scout_test_support)

add_library(scout_core_static_analysis OBJECT EXCLUDE_FROM_ALL
  src/core.cpp
  src/capability_energy_gate.cpp
  src/coordination_evaluator.cpp
  src/continuous_geometry_validator.cpp
  src/hybrid_map_query.cpp
  src/planning_context.cpp
  src/protobuf_adapter.cpp
  src/quintic_bezier.cpp
  src/trajectory_smoother.cpp
  src/state_lattice_astar.cpp
  src/survey_action.cpp
  src/survey_plan_evidence.cpp
  src/deterministic_fixture.cpp
)
target_compile_features(scout_core_static_analysis PRIVATE cxx_std_17)
target_include_directories(scout_core_static_analysis PRIVATE include)
target_link_libraries(scout_core_static_analysis PRIVATE
  scout_contract_proto
  ICU::uc
  Threads::Threads
)
scout_enable_strict_warnings(scout_core_static_analysis)
if(MSVC)
  target_compile_options(scout_core_static_analysis PRIVATE
    /analyze
    /analyze:external-
  )
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_compile_options(scout_core_static_analysis PRIVATE -fanalyzer)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  find_program(SCOUT_CLANG_TIDY NAMES clang-tidy REQUIRED)
  set_target_properties(scout_core_static_analysis PROPERTIES
    CXX_CLANG_TIDY "${SCOUT_CLANG_TIDY};--warnings-as-errors=*"
  )
else()
  message(FATAL_ERROR "No static analyzer is configured for ${CMAKE_CXX_COMPILER_ID}")
endif()
add_custom_target(static-check DEPENDS scout_core_static_analysis)

add_executable(scout_core_tests test/test_core.cpp)
target_compile_features(scout_core_tests PRIVATE cxx_std_17)
target_link_libraries(scout_core_tests PRIVATE scout::core scout::test_support)
scout_enable_strict_warnings(scout_core_tests)

add_test(NAME scout_core_build_boundary COMMAND scout_core_tests)
set_tests_properties(scout_core_build_boundary PROPERTIES
  LABELS "unit;bootstrap;deterministic"
)

add_executable(scout_protobuf_adapter_tests test/test_protobuf_adapter.cpp)
target_compile_features(scout_protobuf_adapter_tests PRIVATE cxx_std_17)
target_link_libraries(scout_protobuf_adapter_tests PRIVATE
  scout::core
  scout::test_support
)
scout_enable_strict_warnings(scout_protobuf_adapter_tests)

add_test(NAME scout_protobuf_adapter COMMAND scout_protobuf_adapter_tests)
set_tests_properties(scout_protobuf_adapter PROPERTIES
  LABELS "unit;contract;protobuf;deterministic"
)

add_executable(scout_planning_context_tests test/test_planning_context.cpp)
target_compile_features(scout_planning_context_tests PRIVATE cxx_std_17)
target_link_libraries(scout_planning_context_tests PRIVATE
  scout::core
  scout::test_support
)
scout_enable_strict_warnings(scout_planning_context_tests)

add_test(NAME scout_planning_context COMMAND scout_planning_context_tests)
set_tests_properties(scout_planning_context PROPERTIES
  LABELS "unit;context;deterministic"
)

add_executable(scout_hybrid_map_query_tests test/test_hybrid_map_query.cpp)
target_compile_features(scout_hybrid_map_query_tests PRIVATE cxx_std_17)
target_link_libraries(scout_hybrid_map_query_tests PRIVATE
  scout::core
  scout::test_support
)
scout_enable_strict_warnings(scout_hybrid_map_query_tests)

add_test(NAME scout_hybrid_map_query COMMAND scout_hybrid_map_query_tests)
set_tests_properties(scout_hybrid_map_query PROPERTIES
  LABELS "unit;map;deterministic"
)

add_executable(scout_coordination_evaluator_tests test/test_coordination_evaluator.cpp)
target_compile_features(scout_coordination_evaluator_tests PRIVATE cxx_std_17)
target_link_libraries(scout_coordination_evaluator_tests PRIVATE
  scout::core
  scout::test_support
)
scout_enable_strict_warnings(scout_coordination_evaluator_tests)

add_test(NAME scout_coordination_evaluator COMMAND scout_coordination_evaluator_tests)
set_tests_properties(scout_coordination_evaluator PROPERTIES
  LABELS "unit;coordination;deterministic"
)

add_executable(scout_survey_action_tests test/test_survey_action.cpp)
target_compile_features(scout_survey_action_tests PRIVATE cxx_std_17)
target_link_libraries(scout_survey_action_tests PRIVATE scout::core)
scout_enable_strict_warnings(scout_survey_action_tests)

add_test(NAME scout_survey_action COMMAND scout_survey_action_tests)
set_tests_properties(scout_survey_action PROPERTIES
  LABELS "unit;survey;deterministic"
)

add_executable(scout_survey_plan_evidence_tests test/test_survey_plan_evidence.cpp)
target_compile_features(scout_survey_plan_evidence_tests PRIVATE cxx_std_17)
target_link_libraries(scout_survey_plan_evidence_tests PRIVATE scout::core)
scout_enable_strict_warnings(scout_survey_plan_evidence_tests)

add_test(NAME scout_survey_plan_evidence COMMAND scout_survey_plan_evidence_tests)
set_tests_properties(scout_survey_plan_evidence PROPERTIES
  LABELS "unit;survey;evidence;deterministic"
)

add_executable(scout_state_lattice_astar_tests test/test_state_lattice_astar.cpp)
target_compile_features(scout_state_lattice_astar_tests PRIVATE cxx_std_17)
target_link_libraries(scout_state_lattice_astar_tests PRIVATE scout::core)
scout_enable_strict_warnings(scout_state_lattice_astar_tests)

add_test(NAME scout_state_lattice_astar COMMAND scout_state_lattice_astar_tests)
set_tests_properties(scout_state_lattice_astar PROPERTIES
  LABELS "unit;search;deterministic"
)

add_executable(scout_quintic_bezier_tests test/test_quintic_bezier.cpp)
target_compile_features(scout_quintic_bezier_tests PRIVATE cxx_std_17)
target_link_libraries(scout_quintic_bezier_tests PRIVATE scout::core)
scout_enable_strict_warnings(scout_quintic_bezier_tests)

add_test(NAME scout_quintic_bezier COMMAND scout_quintic_bezier_tests)
set_tests_properties(scout_quintic_bezier PROPERTIES
  LABELS "unit;trajectory;deterministic"
)

add_executable(scout_trajectory_smoother_tests test/test_trajectory_smoother.cpp)
target_compile_features(scout_trajectory_smoother_tests PRIVATE cxx_std_17)
target_link_libraries(scout_trajectory_smoother_tests PRIVATE scout::core scout::test_support)
scout_enable_strict_warnings(scout_trajectory_smoother_tests)

add_test(NAME scout_trajectory_smoother COMMAND scout_trajectory_smoother_tests)
set_tests_properties(scout_trajectory_smoother PROPERTIES
  LABELS "unit;trajectory;smoothing;deterministic"
)

add_executable(scout_continuous_geometry_validator_tests
  test/test_continuous_geometry_validator.cpp)
target_compile_features(scout_continuous_geometry_validator_tests PRIVATE cxx_std_17)
target_link_libraries(scout_continuous_geometry_validator_tests PRIVATE scout::core)
scout_enable_strict_warnings(scout_continuous_geometry_validator_tests)

add_test(NAME scout_continuous_geometry_validator
  COMMAND scout_continuous_geometry_validator_tests)
set_tests_properties(scout_continuous_geometry_validator PROPERTIES
  LABELS "unit;trajectory;geometry;deterministic"
)

add_executable(scout_fixture_tests test/test_deterministic_fixture.cpp)
target_compile_features(scout_fixture_tests PRIVATE cxx_std_17)
target_link_libraries(scout_fixture_tests PRIVATE scout::test_support)
scout_enable_strict_warnings(scout_fixture_tests)
if(WIN32)
  target_link_libraries(scout_fixture_tests PRIVATE Psapi)
endif()

add_test(NAME scout_deterministic_fixtures COMMAND scout_fixture_tests)
set_tests_properties(scout_deterministic_fixtures PROPERTIES
  LABELS "unit;fixture;deterministic"
)
