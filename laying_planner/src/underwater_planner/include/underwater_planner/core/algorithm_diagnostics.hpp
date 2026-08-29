#pragma once

#include "underwater_planner/core/main_planning_loop.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace underwater_planner::core {

struct SearchExperimentSample {
  bool executed{};
  std::size_t fixed_bytes_per_search_label{};
  std::size_t peak_observed_bytes_per_search_label{};
  std::size_t peak_active_labels{};
  std::size_t maximum_active_labels{};
  std::size_t labels_per_base_key_p50{};
  std::size_t labels_per_base_key_p95{};
  std::size_t labels_per_base_key_p99{};
  std::size_t equivalent_discard_count{};
  std::size_t equivalent_replacement_count{};
  std::size_t signature_fallback_comparison_count{};
  bool label_budget_exhausted{};
  bool deadline_exceeded{};
};

struct ProcessMemorySample {
  std::size_t resident_bytes{};
  std::size_t peak_resident_bytes{};
};

[[nodiscard]] ProcessMemorySample sample_process_memory();

struct SmoothingExperimentSample {
  std::size_t iterations{};
  double maximum_constraint_residual{};
};

struct ConstraintMarginSample {
  std::string constraint;
  double margin{};
  std::string unit;
};

struct ParameterizationExperimentSample {
  std::uint64_t limits_version{};
  std::uint64_t execution_profile_version{};
  std::uint64_t execution_operating_envelope_version{};
  std::vector<ConstraintMarginSample> constraint_margins;
  double minimum_constraint_margin{};
};

struct StageDurationSample {
  PlanningCycleStage stage{PlanningCycleStage::capture_inputs};
  Duration duration;
  bool succeeded{};
};

struct ConstraintFailureSample {
  std::string code;
  std::optional<PlanningCycleStage> planning_stage;
  std::string external_stage;
  std::string message;
  bool has_numeric_evidence{};
  double constraint_value{};
  double hard_limit{};
  std::optional<Vector2m> position_m;
};

// A planning outcome is not the external, physical laying success rate. Each
// epsilon remains explicitly scoped to its local risk domain.
struct AlgorithmRiskRecord {
  bool planning_succeeded{};
  std::optional<double> epsilon_robot_pointwise;
  std::optional<double> epsilon_terrain_gradient_local;
  std::optional<double> epsilon_cable_pointwise;
  std::optional<double> epsilon_path;
  bool path_joint_risk_implemented{};
  bool terrain_gradient_path_joint_risk_implemented{};
  std::string semantics{"POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE"};
};

struct AlgorithmParameterVersions {
  std::uint64_t search_parameters{};
  std::uint64_t motion_primitives{};
  std::uint64_t smoothing_limits{};
  std::uint64_t parameterization_limits{};
  std::uint64_t execution_profile{};
};

struct AlgorithmExperimentRecord {
  std::string schema_version{"algorithm-experiment/v3"};
  PlanningCycleRequest request;
  std::uint64_t cycle_sequence{};
  std::uint64_t random_seed{};
  std::uint64_t initial_source_revision{};
  std::uint64_t decision_source_revision{};
  PlanningDependencyVersions dependencies;
  PlanningDependencyVersions decision_dependencies;
  AlgorithmRuntimeParameterSnapshot parameters;
  std::string canonical_parameters;
  std::uint64_t parameter_fingerprint{};
  std::optional<SynchronizedValidationInputs> initial_inputs;
  std::optional<SynchronizedValidationInputs> decision_inputs;
  std::optional<AuthorizedPlanningResult> initial_authorization;
  std::vector<ValidationInputCaptureResult> input_captures;
  bool committed_lease_was_revoked{};
  AlgorithmParameterVersions parameter_versions;
  SearchExperimentSample search;
  SmoothingExperimentSample smoothing;
  ParameterizationExperimentSample parameterization;
  std::vector<ConstraintFailureSample> constraint_failures;
  std::vector<StageDurationSample> stage_duration_samples;
  std::vector<Duration> lease_revalidation_duration_samples;
  Duration lease_revalidation_duration{0};
  Duration total_cycle_duration{0};
  ProcessMemorySample process_memory;
  PlanningCycleStatus final_status{PlanningCycleStatus::input_invalid};
  PlanningState final_state{PlanningState::input_invalid};
  std::optional<GeometricPath> final_path;
  Diagnostics final_diagnostics;
  std::optional<PlanningFailure> root_cause;
  AlgorithmRiskRecord risk;
};

struct ExperimentRecordValidation {
  bool valid{};
  std::vector<std::string> issues;
};

[[nodiscard]] ExperimentRecordValidation validate(
    const AlgorithmExperimentRecord& record);
[[nodiscard]] std::string serialize_algorithm_experiment_record(
    const AlgorithmExperimentRecord& record);

class AlgorithmDiagnosticsRecorder {
 public:
  [[nodiscard]] static AlgorithmExperimentRecord capture(
      const PlanningCycleRequest& request, const PlanningCycleResult& result);
};

template <typename Value>
struct ExperimentQuantiles {
  Value p50{};
  Value p95{};
  Value p99{};
};

struct AlgorithmExperimentSummary {
  std::size_t sample_count{};
  std::size_t rejected_sample_count{};
  std::size_t search_sample_count{};
  std::vector<std::size_t> peak_active_label_samples;
  std::vector<std::size_t> equivalence_comparison_samples;
  std::vector<std::int64_t> search_duration_samples_ns;
  std::vector<std::int64_t> smoothing_duration_samples_ns;
  std::vector<std::int64_t> lease_revalidation_duration_samples_ns;
  std::vector<std::size_t> fixed_bytes_per_search_label_samples;
  std::vector<std::size_t> peak_observed_bytes_per_search_label_samples;
  std::vector<std::size_t> process_peak_memory_samples_bytes;
  std::vector<std::int64_t> total_cycle_duration_samples_ns;
  double label_budget_exhaustion_rate{};
  double timeout_rate{};
  ExperimentQuantiles<std::size_t> peak_active_labels;
  ExperimentQuantiles<std::int64_t> search_duration_ns;
  ExperimentQuantiles<std::int64_t> smoothing_duration_ns;
  ExperimentQuantiles<std::int64_t> lease_revalidation_duration_ns;
  ExperimentQuantiles<std::size_t> fixed_bytes_per_search_label;
  ExperimentQuantiles<std::size_t> peak_observed_bytes_per_search_label;
  ExperimentQuantiles<std::size_t> process_peak_memory_bytes;
  ExperimentQuantiles<std::int64_t> total_cycle_duration_ns;
  std::map<PlanningCycleStage, ExperimentQuantiles<std::int64_t>>
      stage_duration_ns;
};

struct AlgorithmPerformanceBudget {
  std::size_t maximum_total_memory_bytes{100U * 1024U * 1024U};
  Duration maximum_cycle_duration{500'000'000};
  double target_frequency_hz{5.0};
  std::size_t minimum_sample_count{1U};
};

enum class PerformanceBudgetStatus {
  verified,
  insufficient_evidence,
  memory_budget_exceeded,
  cycle_timeout_exceeded,
  target_frequency_unverified,
};

struct PerformanceBudgetAssessment {
  PerformanceBudgetStatus status{PerformanceBudgetStatus::insufficient_evidence};
  bool target_frequency_verified{};
  bool requires_safe_failure{};
  std::size_t maximum_observed_memory_bytes{};
  Duration maximum_observed_cycle_duration;
  double timeout_rate{};
  std::vector<std::string> diagnostics;
};

[[nodiscard]] PerformanceBudgetAssessment assess_performance_budget(
    const AlgorithmExperimentSummary& summary,
    const AlgorithmPerformanceBudget& budget);

class AlgorithmExperimentLog {
 public:
  [[nodiscard]] ExperimentRecordValidation append(
      AlgorithmExperimentRecord record);
  [[nodiscard]] AlgorithmExperimentSummary summarize() const;
  [[nodiscard]] const std::vector<AlgorithmExperimentRecord>& records() const
      noexcept {
    return records_;
  }

 private:
  std::vector<AlgorithmExperimentRecord> records_;
  std::vector<bool> summary_eligible_;
};

enum class AlgorithmExperimentReplayStatus {
  reproduced,
  invalid_record,
  replay_failed,
  mismatch,
};

struct AlgorithmExperimentReplayResult {
  AlgorithmExperimentReplayStatus status{
      AlgorithmExperimentReplayStatus::invalid_record};
  std::vector<std::string> differences;
};

class AlgorithmExperimentReplayer {
 public:
  [[nodiscard]] static AlgorithmExperimentReplayResult replay(
      const AlgorithmExperimentRecord& record,
      MainPlanningLoopStages& stages, MainPlanningLoopClock clock);
};

}  // namespace underwater_planner::core
