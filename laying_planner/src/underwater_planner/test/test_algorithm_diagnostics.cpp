#include "underwater_planner/core/algorithm_diagnostics.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

using namespace underwater_planner::core;

void require(const bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

PlanningDependencyVersions dependencies() {
  return {{"map", 42U, {1'000}, "world"},
          3U,
          4U,
          5U,
          6U,
          7U,
          8U,
          9U,
          10U,
          11U,
          SensorHealthMode::nominal,
          "competition-seabed-v1",
          12U};
}

ParameterConfig parameters() {
  ParameterConfig config;
  config.profile_id = "diagnostics-test-v1";
  config.mode = ParameterProfileMode::non_production_capability_profile;
  config.operating_domain_id = "competition-seabed-v1";
  config.search.maximum_active_labels = 100U;
  config.statistical_risk.maximum_planning_duration_s = 0.5;
  return config;
}

AlgorithmRuntimeParameterSnapshot runtime_parameters() {
  AlgorithmRuntimeParameterSnapshot snapshot;
  snapshot.profile = parameters();
  snapshot.terrain_analysis.config_version = 31U;
  snapshot.search.version = 32U;
  snapshot.search.primitive_set_version = 33U;
  snapshot.search.maximum_expansions = 10'000U;
  snapshot.search.motion_primitives = {{33U, 0.5, -0.2},
                                       {33U, 0.5, 0.2}};
  snapshot.smoothing.version = 34U;
  snapshot.smoothing.timeout = {50'000'000};
  snapshot.parameterization.version = 35U;
  snapshot.parameterization.timeout = {50'000'000};
  return snapshot;
}

SynchronizedValidationInputs inputs(const std::uint64_t revision) {
  SynchronizedValidationInputs value;
  value.source_revision = revision;
  value.captured_at = {1'100};
  value.dependencies = dependencies();
  value.planning_snapshot.map.version = value.dependencies.map_version;
  value.planning_snapshot.reference_line.version =
      value.dependencies.reference_line_version;
  value.planning_snapshot.reference_line.coordinate_frame = "world";
  value.robot_state.sequence_number = 20U;
  value.cable_state.sequence_number = 21U;
  value.reference_progress.sequence_number = 22U;
  value.cable_telemetry.sequence_number = 23U;
  value.execution_tracking_state.sequence_number = 24U;
  return value;
}

GeometricPath path(const double lateral_offset_m = 0.0) {
  GeometricPath value;
  value.metadata = {12U, "world", 3U, "linear"};
  value.points = {{0.0, 0.0, lateral_offset_m, 0.0, 0.0},
                  {1.0, 1.0, lateral_offset_m, 0.0, 0.0}};
  return value;
}

PlanningCycleResult cycle(const std::uint64_t sequence,
                          const std::int64_t duration_ns = 100) {
  PlanningCycleResult result;
  result.status = PlanningCycleStatus::success;
  result.state = PlanningState::success;
  result.initial_inputs =
      std::make_shared<const SynchronizedValidationInputs>(inputs(sequence));
  result.decision_inputs =
      std::make_shared<const SynchronizedValidationInputs>(inputs(sequence + 1U));
  result.replay_input_captures = {
      {ValidationInputCaptureStatus::captured, inputs(sequence), {}},
      {ValidationInputCaptureStatus::captured, inputs(sequence + 1U), {}}};
  result.diagnostics.cycle_sequence = sequence;
  result.diagnostics.random_seed = 99U;
  result.diagnostics.initial_source_revision = sequence;
  result.diagnostics.decision_source_revision = sequence + 1U;
  result.diagnostics.initial_dependencies = dependencies();
  result.diagnostics.decision_dependencies = dependencies();
  result.diagnostics.parameters = runtime_parameters();
  result.diagnostics.cycle_started_at = {1'900};
  result.diagnostics.maximum_cycle_duration = {500'000'000};
  result.diagnostics.stages = {
      {PlanningCycleStage::search, {2'000}, {duration_ns}, sequence,
       dependencies(), true},
      {PlanningCycleStage::smoothing, {2'100}, {duration_ns + 10}, sequence,
       dependencies(), true},
      {PlanningCycleStage::parameterization, {2'200}, {duration_ns + 20},
       sequence, dependencies(), true},
      {PlanningCycleStage::candidate_revalidation, {2'300},
       {duration_ns + 30}, sequence + 1U, dependencies(), true}};

  auto search = std::make_shared<HybridAStarPlanningResult>();
  search->state = PlanningState::success;
  search->robot_path = path();
  search->diagnostics.search_parameter_version = 14U;
  search->diagnostics.primitive_set_version = 15U;
  search->diagnostics.peak_active_label_count = 80U;
  search->diagnostics.maximum_active_label_budget = 100U;
  search->diagnostics.fixed_bytes_per_search_label = 512U;
  search->diagnostics.peak_observed_bytes_per_search_label = 768U;
  search->diagnostics.equivalent_label_discard_count = 7U;
  search->diagnostics.equivalent_label_replacement_count = 3U;
  search->diagnostics.signature_fallback_comparison_count = 2U;
  search->diagnostics.active_label_budget_exhausted = sequence == 2U;
  search->diagnostics.labels_per_base_key_p50 = 2U;
  search->diagnostics.labels_per_base_key_p95 = 5U;
  search->diagnostics.labels_per_base_key_p99 = 8U;
  result.artifacts.search = std::move(search);

  auto smoothing = std::make_shared<SmoothingResult>();
  smoothing->status = SmoothingStatus::success;
  smoothing->path = path();
  smoothing->audit.solver_iterations = 17U;
  smoothing->residuals.maximum_dynamics_residual = 0.02;
  smoothing->residuals.maximum_curvature_audit_residual = 0.04;
  smoothing->residuals.goal_heading_residual_rad = 0.03;
  result.artifacts.smoothing = std::move(smoothing);

  auto parameterization = std::make_shared<ParameterizationResult>();
  parameterization->status = ParameterizationStatus::success;
  parameterization->diagnostics.maximum_lateral_acceleration_mps2 = 0.3;
  parameterization->diagnostics.required_stopping_distance_m = 0.7;
  parameterization->diagnostics.available_stopping_distance_m = 1.0;
  parameterization->diagnostics.minimum_speed_mps = 0.1;
  parameterization->diagnostics.maximum_speed_mps = 0.8;
  parameterization->diagnostics.limits_version = 16U;
  TimedPath trajectory;
  trajectory.geometry = path();
  trajectory.execution_profile.version = 11U;
  trajectory.execution_profile.operating_envelope_version = 10U;
  trajectory.execution_profile.approved_tracking_limits.ground_speed =
      {0.0, 1.0};
  trajectory.execution_profile.approved_tracking_limits.ground_acceleration =
      {-0.5, 0.5};
  trajectory.execution_profile.approved_tracking_limits.payout_speed =
      {0.0, 1.2};
  trajectory.execution_profile.approved_tracking_limits.payout_acceleration =
      {-0.4, 0.4};
  trajectory.execution_profile.approved_tracking_limits.tension = {20.0, 60.0};
  trajectory.execution_profile.approved_tracking_limits
      .maximum_lateral_acceleration_mps2 = 0.5;
  trajectory.execution_profile.samples = {
      {0.0, {0}, 0.1, 0.2, 0.2, 0.1, 30.0},
      {1.0, {1'000}, 0.8, -0.3, 0.9, -0.2, 50.0}};
  parameterization->trajectory = trajectory;
  result.artifacts.parameterization = std::move(parameterization);

  auto candidate = std::make_shared<PlanningResult>();
  candidate->sequence_number = sequence;
  candidate->state = PlanningState::success;
  candidate->robot_trajectory = trajectory;
  candidate->diagnostics.schema_version = "diagnostics/v1";
  candidate->diagnostics.random_seed = 99U;
  candidate->diagnostics.input_version = "input/v42";
  candidate->diagnostics.unit_system = "SI";
  candidate->diagnostics.operating_domain_id =
      dependencies().operating_domain_id;
  candidate->diagnostics.risk_semantics =
      "POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE";
  candidate->diagnostics.dependencies = dependencies();
  candidate->error_budget.epsilon_robot = 0.01;
  candidate->error_budget.epsilon_terrain_gradient_local = 0.02;
  candidate->error_budget.epsilon_point = 0.03;
  result.artifacts.candidate = std::move(candidate);
  return result;
}

PlanningCycleRequest request(const std::uint64_t sequence) {
  PlanningCycleRequest value;
  value.cycle_sequence = sequence;
  value.random_seed = 99U;
  value.triggered_at = {1'000};
  return value;
}

void unified_record_preserves_auditable_raw_evidence() {
  const AlgorithmExperimentRecord record =
      AlgorithmDiagnosticsRecorder::capture(request(1U), cycle(1U));
  const ExperimentRecordValidation validation = validate(record);
  require(validation.valid, "complete experiment record was rejected");
  require(record.schema_version == "algorithm-experiment/v3" &&
              record.cycle_sequence == 1U && record.random_seed == 99U &&
              record.initial_source_revision == 1U &&
              record.decision_source_revision == 2U &&
              record.dependencies.map_version.sequence_number == 42U,
          "input versions or deterministic seed were not recorded");
  require(record.process_memory.resident_bytes > 0U &&
              record.process_memory.peak_resident_bytes >=
                  record.process_memory.resident_bytes &&
              record.total_cycle_duration.nanoseconds == 530,
          "capture did not attach process memory or trigger-to-finish duration");
  require(record.parameters.profile.search.maximum_active_labels == 100U &&
              record.parameters.profile.statistical_risk
                      .maximum_planning_duration_s ==
                  0.5 &&
              record.parameters.search.maximum_expansions == 10'000U &&
              record.parameters.search.motion_primitives.size() == 2U &&
              !record.canonical_parameters.empty() &&
              record.parameter_fingerprint != 0U,
          "typed parameter values were not canonically recorded");
  require(record.search.peak_active_labels == 80U &&
              record.search.maximum_active_labels == 100U &&
              record.search.equivalent_discard_count == 7U &&
              record.search.equivalent_replacement_count == 3U &&
              record.search.signature_fallback_comparison_count == 2U &&
              !record.search.label_budget_exhausted,
          "search labels, equivalence comparisons, or budget sample missing");
  require(record.smoothing.iterations == 17U &&
              record.smoothing.maximum_constraint_residual == 0.04,
          "smoothing iteration or maximum residual missing");
  require(record.parameterization.constraint_margins.size() == 12U &&
              record.parameterization.minimum_constraint_margin == 0.1,
          "parameterized constraint margins are incomplete");
  if (record.lease_revalidation_duration.nanoseconds != 130) {
    throw std::runtime_error(
        "lease revalidation timing sample is missing: " +
        std::to_string(record.lease_revalidation_duration.nanoseconds));
  }
  require(record.stage_duration_samples.size() == 4U,
          "stage raw timing samples are missing");
  require(record.risk.planning_succeeded &&
              record.risk.epsilon_robot_pointwise == 0.01 &&
              record.risk.epsilon_terrain_gradient_local == 0.02 &&
              record.risk.epsilon_cable_pointwise == 0.03 &&
              !record.risk.epsilon_path.has_value() &&
              !record.risk.path_joint_risk_implemented &&
              record.risk.semantics ==
                  "POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE",
          "task outcome, local epsilon, and unimplemented joint risk were mixed");
  AlgorithmExperimentRecord repeated =
      AlgorithmDiagnosticsRecorder::capture(request(1U), cycle(1U));
  AlgorithmExperimentRecord normalized = record;
  normalized.process_memory = {};
  repeated.process_memory = {};
  require(serialize_algorithm_experiment_record(normalized) ==
              serialize_algorithm_experiment_record(repeated),
          "identical experiment records did not serialize deterministically");

  AlgorithmExperimentRecord overstated = record;
  overstated.risk.epsilon_path = 0.001;
  require(!validate(overstated).valid,
          "unimplemented joint path risk was accepted by the record gate");

  AlgorithmExperimentRecord missing_decision = record;
  missing_decision.decision_inputs.reset();
  require(!validate(missing_decision).valid,
          "record with a decision revision but no decision input was accepted");

  AlgorithmExperimentRecord tampered_parameters = record;
  tampered_parameters.parameters.search.maximum_expansions = 10'001U;
  require(!validate(tampered_parameters).valid,
          "expanded stage parameter changes escaped derived evidence");
  tampered_parameters = record;
  tampered_parameters.parameters.terrain_analysis.huber_delta_m = 0.25;
  require(!validate(tampered_parameters).valid,
          "terrain analysis parameter changes escaped derived evidence");
  tampered_parameters = record;
  tampered_parameters.parameters.smoothing.maximum_curvature_per_m = 0.4;
  require(!validate(tampered_parameters).valid,
          "smoothing limit changes escaped derived evidence");
  tampered_parameters = record;
  tampered_parameters.parameters.parameterization.sample_period_s = 0.1;
  require(!validate(tampered_parameters).valid,
          "parameterization limit changes escaped derived evidence");
}

void experiment_log_retains_raw_samples_and_reports_quantiles() {
  AlgorithmExperimentLog log;
  require(log.append(AlgorithmDiagnosticsRecorder::capture(request(1U),
                                                            cycle(1U)))
              .valid,
          "first record was not appended");
  require(log.append(AlgorithmDiagnosticsRecorder::capture(request(2U),
                                                            cycle(2U, 300)))
              .valid,
          "second record was not appended");
  const AlgorithmExperimentSummary summary = log.summarize();
  require(summary.sample_count == 2U &&
              summary.search_sample_count == 2U &&
              summary.peak_active_label_samples ==
                  std::vector<std::size_t>({80U, 80U}) &&
              summary.search_duration_samples_ns ==
                  std::vector<std::int64_t>({100, 300}) &&
              summary.label_budget_exhaustion_rate == 0.5 &&
              summary.search_duration_ns.p50 == 100 &&
              summary.search_duration_ns.p95 == 300 &&
              summary.search_duration_ns.p99 == 300,
          "raw samples, budget exhaustion rate, or nearest-rank quantiles are wrong");

  PlanningCycleResult before_search = cycle(3U, 500);
  before_search.artifacts.search.reset();
  before_search.diagnostics.stages.erase(
      before_search.diagnostics.stages.begin());
  require(log.append(AlgorithmDiagnosticsRecorder::capture(
                         request(3U), before_search))
              .valid,
          "pre-search failure record was not appended");
  const AlgorithmExperimentSummary unbiased = log.summarize();
  require(unbiased.sample_count == 3U && unbiased.search_sample_count == 2U &&
              unbiased.label_budget_exhaustion_rate == 0.5 &&
              unbiased.search_duration_samples_ns.size() == 2U,
          "cycles that never searched biased search statistics");

  const ExperimentRecordValidation duplicate = log.append(
      AlgorithmDiagnosticsRecorder::capture(request(3U), before_search));
  require(!duplicate.valid && log.records().size() == 4U,
          "invalid cycle identity was silently dropped from the audit log");
  const AlgorithmExperimentSummary filtered = log.summarize();
  require(filtered.sample_count == 3U && filtered.rejected_sample_count == 1U,
          "an invalid audit record contaminated performance statistics");
}

void performance_budget_uses_measured_memory_and_complete_stage_quantiles() {
  using namespace underwater_planner::core;
  AlgorithmExperimentLog log;
  AlgorithmExperimentRecord first =
      AlgorithmDiagnosticsRecorder::capture(request(1U), cycle(1U, 100));
  AlgorithmExperimentRecord second =
      AlgorithmDiagnosticsRecorder::capture(request(2U), cycle(2U, 300));
  first.process_memory = {20U * 1024U * 1024U, 24U * 1024U * 1024U};
  second.process_memory = {22U * 1024U * 1024U, 30U * 1024U * 1024U};
  first.total_cycle_duration = {180'000'000};
  second.total_cycle_duration = {190'000'000};
  require(first.search.fixed_bytes_per_search_label > 0U &&
              first.search.peak_observed_bytes_per_search_label >
                  first.search.fixed_bytes_per_search_label,
          "the concrete search label storage size was not measured");
  require(log.append(first).valid && log.append(second).valid,
          "valid performance records were not appended");

  const AlgorithmExperimentSummary summary = log.summarize();
  require(summary.process_peak_memory_samples_bytes ==
              std::vector<std::size_t>({24U * 1024U * 1024U,
                                        30U * 1024U * 1024U}) &&
              summary.total_cycle_duration_samples_ns ==
                  std::vector<std::int64_t>({180'000'000, 190'000'000}) &&
              summary.fixed_bytes_per_search_label.p50 ==
                  first.search.fixed_bytes_per_search_label &&
              summary.peak_observed_bytes_per_search_label.p50 ==
                  first.search.peak_observed_bytes_per_search_label &&
              summary.process_peak_memory_bytes.p99 ==
                  30U * 1024U * 1024U &&
              summary.total_cycle_duration_ns.p95 == 190'000'000 &&
              summary.stage_duration_ns.count(PlanningCycleStage::search) ==
                  1U &&
              summary.stage_duration_ns.at(PlanningCycleStage::search).p99 ==
                  300 &&
              summary.timeout_rate == 0.0,
          "memory, total-cycle, stage quantiles, or timeout rate are incomplete");

  const AlgorithmPerformanceBudget budget{
      100U * 1024U * 1024U, Duration{500'000'000}, 5.0, 2U};
  const PerformanceBudgetAssessment verified =
      assess_performance_budget(summary, budget);
  require(verified.status == PerformanceBudgetStatus::verified &&
              verified.target_frequency_verified &&
              verified.maximum_observed_memory_bytes ==
                  30U * 1024U * 1024U &&
              verified.maximum_observed_cycle_duration.nanoseconds ==
                  190'000'000 &&
              verified.diagnostics.empty(),
          "measured sub-200 ms cycles did not verify the 5 Hz target");

  AlgorithmPerformanceBudget tight_memory = budget;
  tight_memory.maximum_total_memory_bytes = 25U * 1024U * 1024U;
  const PerformanceBudgetAssessment memory_failure =
      assess_performance_budget(summary, tight_memory);
  require(memory_failure.status ==
              PerformanceBudgetStatus::memory_budget_exceeded &&
              !memory_failure.target_frequency_verified &&
              memory_failure.requires_safe_failure &&
              memory_failure.diagnostics.front() ==
                  "MEMORY_BUDGET_EXCEEDED",
          "memory excess did not produce a fail-closed diagnostic");

  AlgorithmExperimentLog timeout_log;
  second.root_cause = PlanningFailure{
      PlanningFailureCause::smoothing_deadline_exceeded,
      PlanningCycleStage::smoothing, "SMOOTHING_DEADLINE_EXCEEDED",
      "smoothing deadline exceeded before old-plan reauthorization"};
  second.total_cycle_duration = {500'000'001};
  require(timeout_log.append(second).valid,
          "timeout evidence could not be retained");
  AlgorithmPerformanceBudget one_sample_budget = budget;
  one_sample_budget.minimum_sample_count = 1U;
  const PerformanceBudgetAssessment timeout_failure =
      assess_performance_budget(timeout_log.summarize(), one_sample_budget);
  require(timeout_failure.status ==
              PerformanceBudgetStatus::cycle_timeout_exceeded &&
              timeout_failure.timeout_rate == 1.0 &&
              timeout_failure.requires_safe_failure &&
              timeout_failure.diagnostics.front() ==
                  "CYCLE_TIMEOUT_EXCEEDED",
          "the 500 ms timeout path was presented as frequency-capable");
}

void target_process_memory_is_below_the_design_budget() {
  using namespace underwater_planner::core;
  const ProcessMemorySample memory = sample_process_memory();
  require(memory.resident_bytes > 0U &&
              memory.peak_resident_bytes >= memory.resident_bytes,
          "target process memory could not be measured");
  require(memory.peak_resident_bytes < 100U * 1024U * 1024U,
          "target process exceeds the design's 100 MB total memory budget");
}

void constraint_failures_are_preserved_with_the_final_decision() {
  PlanningCycleResult failed = cycle(1U);
  failed.status = PlanningCycleStatus::search_failed;
  failed.state = PlanningState::timeout;
  failed.root_cause = PlanningFailure{
      PlanningFailureCause::search_label_budget_exhausted,
      PlanningCycleStage::search,
      "LABEL_BUDGET_EXHAUSTED",
      "global active-label budget exhausted"};
  auto search = std::make_shared<HybridAStarPlanningResult>(
      *failed.artifacts.search);
  search->diagnostics.worst_constraint =
      {true, "CABLE_CURVATURE_LIMIT", {0.5, 0.25}, 0.7, 0.5, 1.4};
  failed.artifacts.search = std::move(search);
  auto candidate =
      std::make_shared<PlanningResult>(*failed.artifacts.candidate);
  candidate->diagnostics.entries.push_back(
      {DiagnosticSeverity::error, "CABLE_VALIDATION_FAILED",
       "cable_validation", "touchdown path violated a hard constraint",
       {2'400}});
  failed.artifacts.candidate = std::move(candidate);

  const AlgorithmExperimentRecord record =
      AlgorithmDiagnosticsRecorder::capture(request(1U), failed);
  require(record.final_status == PlanningCycleStatus::search_failed &&
              record.final_state == PlanningState::timeout &&
              record.constraint_failures.size() == 3U &&
              record.constraint_failures[0].code ==
                  "CABLE_CURVATURE_LIMIT" &&
              record.constraint_failures[0].constraint_value == 0.7 &&
              record.constraint_failures[0].hard_limit == 0.5 &&
              record.constraint_failures[1].code ==
                  "LABEL_BUDGET_EXHAUSTED" &&
              record.constraint_failures[2].code ==
                  "CABLE_VALIDATION_FAILED",
          "constraint failures or final decision were not preserved");
}

}  // namespace

int main() {
  try {
    unified_record_preserves_auditable_raw_evidence();
    experiment_log_retains_raw_samples_and_reports_quantiles();
    performance_budget_uses_measured_memory_and_complete_stage_quantiles();
    target_process_memory_is_below_the_design_budget();
    constraint_failures_are_preserved_with_the_final_decision();
    std::cout << "algorithm diagnostics tests passed\n";
    return EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::cerr << "algorithm diagnostics tests failed: " << error.what()
              << '\n';
    return EXIT_FAILURE;
  }
}
