#pragma once

#include "underwater_planner/core/commitment_safety.hpp"
#include "underwater_planner/core/hybrid_astar_planner.hpp"
#include "underwater_planner/core/execution_lease_monitor.hpp"
#include "underwater_planner/core/path_smoother.hpp"
#include "underwater_planner/core/parameter_config.hpp"
#include "underwater_planner/core/plan_validity_evaluator.hpp"
#include "underwater_planner/core/planning_result.hpp"
#include "underwater_planner/core/stability_manager.hpp"
#include "underwater_planner/core/terrain_analyzer.hpp"
#include "underwater_planner/core/timed_cable_candidate_verifier.hpp"
#include "underwater_planner/core/trajectory_parameterizer.hpp"

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace underwater_planner::core {

class AlgorithmExperimentLog;
struct PlanningFailure;

enum class PlanningStartSource {
  synchronized_actual_state,
  committed_segment_terminal,
};

struct CommittedPlanningStart {
  std::uint64_t source_plan_sequence_number{};
  std::uint64_t lease_sequence{};
  PlanningDependencyVersions dependencies;
  TimedPath authorized_prefix;
  RobotState terminal_robot_state;
};

struct PlanningCycleRequest {
  std::uint64_t cycle_sequence{};
  std::uint64_t random_seed{};
  MonotonicTime triggered_at;
  std::optional<CommittedPlanningStart> committed_start;
};

struct PlanningCycleStart {
  PlanningStartSource source{PlanningStartSource::synchronized_actual_state};
  RobotState robot_state;
  CableState cable_state;
  ReferenceProgress reference_progress;
  std::uint64_t source_plan_sequence_number{};
  std::uint64_t lease_sequence{};
};

// The object is assembled once after terrain analysis. Every subsequent stage
// receives this same const object, so it has no API for consulting mutable
// global state or replacing one member of the synchronized snapshot.
struct LockedPlanningCycleContext {
  const SynchronizedValidationInputs& inputs;
  const TerrainLayers& terrain;
};

struct TerrainAnalysisStageResult {
  bool valid{};
  TerrainLayers terrain;
  Diagnostics diagnostics;
  std::vector<std::string> issues;
};

struct PlanningCandidateMetadata {
  std::uint64_t sequence_number{};
  MonotonicTime timestamp;
  Duration validity_duration;
  double path_cost{std::numeric_limits<double>::quiet_NaN()};
  ErrorBudget error_budget;
  Diagnostics diagnostics;
};

struct CommitmentValidationStageResult {
  bool valid{};
  PathCandidateVerificationResult robot_validation;
  TimedCableCandidateResult cable_validation;
  CommitmentSafetyEvent observed_safety_event{CommitmentSafetyEvent::none};
  std::optional<ObstacleStoppingEvidence> obstacle_stopping;
  ReferenceProgress terminal_reference_progress;
  Diagnostics diagnostics;
  std::vector<std::string> issues;
};

// Captures the expanded, typed values actually consumed by the planning
// stages. The profile alone is insufficient because stage construction also
// supplies solver limits, search budgets, and the concrete primitive set.
struct AlgorithmRuntimeParameterSnapshot {
  std::string schema_version{"algorithm-runtime-parameters/v1"};
  ParameterConfig profile;
  TerrainAnalysisConfig terrain_analysis;
  HybridAStarSearchParameters search;
  SmoothingLimits smoothing;
  TrajectoryParameterizationLimits parameterization;
};

class MainPlanningLoopInputSource {
 public:
  virtual ~MainPlanningLoopInputSource() = default;

  [[nodiscard]] virtual AlgorithmRuntimeParameterSnapshot
  capture_runtime_parameters() const = 0;
  [[nodiscard]] virtual ValidationInputCaptureResult capture(
      MonotonicTime now) = 0;
};

class MainPlanningLoopStages : public MainPlanningLoopInputSource {
 public:
  explicit MainPlanningLoopStages(
      PathHysteresisConfig hysteresis_config = {}) noexcept
      : stability_manager_(hysteresis_config) {}
  ~MainPlanningLoopStages() override = default;

  [[nodiscard]] virtual TerrainAnalysisStageResult analyze_terrain(
      const SynchronizedValidationInputs& captured) = 0;
  [[nodiscard]] virtual CommitmentValidationStageResult validate_commitment(
      const TimedPath& authorized_prefix,
      const CableState& synchronized_actual_cable_state,
      const ReferenceProgress& synchronized_reference_progress,
      const LockedPlanningCycleContext& context) = 0;
  // Independent safety channel. The main loop revokes the old lease before
  // invoking this boundary and does not enter search in the same cycle.
  virtual void request_commitment_safety_stop(
      const CommitmentSafetyCheckResult& safety, MonotonicTime at) = 0;
  // Ordinary planning failures use the same revoke-before-stop ordering as
  // commitment overrides. This boundary must not issue motion commands.
  virtual void request_controlled_stop(const PlanningFailure& failure,
                                       MonotonicTime at) = 0;
  [[nodiscard]] virtual HybridAStarPlanningResult search(
      const PlanningCycleStart& start,
      const LockedPlanningCycleContext& context) = 0;
  [[nodiscard]] virtual SmoothingResult smooth(
      const GeometricPath& raw_path, const PlanningCycleStart& start,
      const LockedPlanningCycleContext& context) = 0;
  [[nodiscard]] virtual TrackabilityResult validate_raw_path_trackability(
      const GeometricPath& raw_path, const PlanningCycleStart& start,
      const LockedPlanningCycleContext& context) = 0;
  [[nodiscard]] virtual ParameterizationResult parameterize(
      const GeometricPath& path, const PlanningCycleStart& start,
      const LockedPlanningCycleContext& context) = 0;
  [[nodiscard]] virtual TimedPathMergeResult merge_commitment(
      const TimedPath& authorized_prefix, const TimedPath& new_tail,
      const LockedPlanningCycleContext& context) = 0;
  [[nodiscard]] virtual PathCandidateVerificationResult
  verify_complete_robot_path(
      const TimedPath& complete_path, const PlanningCycleStart& start,
      const LockedPlanningCycleContext& context) = 0;
  [[nodiscard]] virtual TimedCableCandidateResult verify_cable(
      const TimedPath& complete_path,
      const CableState& synchronized_actual_cable_state,
      const LockedPlanningCycleContext& context) = 0;
  [[nodiscard]] virtual PlanningCandidateMetadata assemble_candidate_metadata(
      const PlanningCycleRequest& request, const PlanningCycleStart& start,
      const LockedPlanningCycleContext& context,
      const HybridAStarPlanningResult& search_result,
      const SmoothingResult& smoothing_result,
      const ParameterizationResult& parameterization_result,
      const TimedCableCandidateResult& cable_result) = 0;
  [[nodiscard]] virtual PlanValidityEvaluation revalidate_plan(
      const PlanningResult& plan, PlanValidationTarget target,
      const SynchronizedValidationInputs& latest_inputs,
      MonotonicTime now) = 0;
  [[nodiscard]] PathSwitchDecision decide_candidate(
      const std::optional<PlanValidityEvaluation>& validated_current,
      const PlanValidityEvaluation& validated_candidate,
      double current_cost, double candidate_cost,
      const SynchronizedValidationInputs& latest_inputs,
      MonotonicTime now);

 protected:
  virtual void observe_candidate_decision_inputs(
      const std::optional<PlanValidityEvaluation>&,
      const PlanValidityEvaluation&,
      const SynchronizedValidationInputs&) {}

 private:
  StabilityManager stability_manager_;
};

enum class PlanningCycleStage {
  capture_inputs,
  terrain_analysis,
  commitment_validation,
  search,
  smoothing,
  raw_path_trackability_validation,
  parameterization,
  commitment_merge,
  complete_robot_path_validation,
  cable_validation,
  candidate_assembly,
  decision_context_capture,
  candidate_revalidation,
  candidate_decision,
  lease_acquisition,
  publication,
  current_plan_revalidation,
  current_plan_reauthorization,
};

struct PlanningCycleStageMetric {
  PlanningCycleStage stage{PlanningCycleStage::capture_inputs};
  MonotonicTime started_at;
  Duration duration;
  std::uint64_t source_revision{};
  PlanningDependencyVersions dependencies;
  bool succeeded{};
};

struct PlanningCycleDiagnostics {
  std::uint64_t cycle_sequence{};
  std::uint64_t random_seed{};
  std::uint64_t initial_source_revision{};
  std::uint64_t decision_source_revision{};
  PlanningDependencyVersions initial_dependencies;
  PlanningDependencyVersions decision_dependencies;
  AlgorithmRuntimeParameterSnapshot parameters;
  MonotonicTime cycle_started_at;
  Duration maximum_cycle_duration;
  std::vector<PlanningCycleStageMetric> stages;
};

struct PlanningCycleArtifacts {
  std::shared_ptr<const TerrainAnalysisStageResult> terrain;
  std::shared_ptr<const CommitmentValidationStageResult> commitment_validation;
  std::shared_ptr<const HybridAStarPlanningResult> search;
  std::shared_ptr<const SmoothingResult> smoothing;
  std::shared_ptr<const ParameterizationResult> parameterization;
  std::shared_ptr<const TimedPathMergeResult> commitment_merge;
  std::shared_ptr<const PathCandidateVerificationResult>
      complete_robot_path_validation;
  std::shared_ptr<const TimedCableCandidateResult> cable_validation;
  std::shared_ptr<const PlanningCandidateMetadata> candidate_metadata;
  std::shared_ptr<const PlanningResult> candidate;
  std::shared_ptr<const PlanValidityEvaluation> candidate_revalidation;
  std::shared_ptr<const PlanValidityEvaluation> current_plan_revalidation;
  std::shared_ptr<const PathSwitchDecision> candidate_decision;
};

enum class PlanningCycleStatus {
  success,
  current_plan_reused,
  commitment_overridden,
  covariance_envelope_breached,
  input_invalid,
  commitment_invalid,
  terrain_analysis_failed,
  robot_path_validation_failed,
  search_failed,
  smoothing_failed,
  parameterization_failed,
  cable_validation_failed,
  candidate_invalid,
  candidate_invalidated,
  decision_rejected,
  lease_invalid,
  publication_failed,
  cycle_timeout,
};

enum class PlanningFailureCause {
  commitment_safety_event,
  search_deadline_exceeded,
  search_label_budget_exhausted,
  search_budget_exhausted,
  smoothing_deadline_exceeded,
  smoothing_infeasible,
  parameterization_deadline_exceeded,
  parameterization_infeasible,
  planning_cycle_deadline_exceeded,
  covariance_envelope_breach,
  no_solution_under_covariance_envelope,
  no_solution,
  input_invalid,
};

struct PlanningFailure {
  PlanningFailureCause cause{PlanningFailureCause::input_invalid};
  PlanningCycleStage stage{PlanningCycleStage::capture_inputs};
  std::string reason_code;
  std::string message;
};

struct AuthorizedPlanningResult {
  ImmutablePlanningResult plan;
  std::shared_ptr<const TimedPath> remaining_path;
  PlanValidationLease lease;
  double path_cost{};
};

struct PlanningCycleResult {
  PlanningCycleStatus status{PlanningCycleStatus::input_invalid};
  PlanningState state{PlanningState::input_invalid};
  std::optional<PlanningCycleStart> start;
  std::optional<AuthorizedPlanningResult> replay_initial_authorization;
  std::vector<ValidationInputCaptureResult> replay_input_captures;
  bool replay_committed_lease_revoked{};
  std::shared_ptr<const SynchronizedValidationInputs> initial_inputs;
  PlanningCycleArtifacts artifacts;
  std::shared_ptr<const SynchronizedValidationInputs> decision_inputs;
  std::optional<AuthorizedPlanningResult> publication;
  std::optional<PlanningFailure> root_cause;
  std::shared_ptr<const CommitmentSafetyCheckResult> commitment_safety;
  bool controlled_stop_required{};
  bool urgent_replan_required{};
  bool used_raw_search_path{};
  bool experiment_recorded{};
  bool experiment_record_valid{};
  std::vector<std::string> experiment_recording_issues;
  std::optional<std::uint64_t> revoked_lease_sequence;
  std::optional<std::uint64_t> ignored_revoked_commitment_lease_sequence;
  PlanningCycleDiagnostics diagnostics;
  std::vector<std::string> issues;

  [[nodiscard]] bool succeeded() const noexcept {
    return status == PlanningCycleStatus::success &&
           state == PlanningState::success && publication.has_value();
  }
};

using MainPlanningLoopClock = std::function<MonotonicTime()>;

enum class AuthorizedPlanningPublishStatus {
  published,
  invalid_lease,
  plan_rejected,
};

struct AuthorizedPlanningPublication {
  AuthorizedPlanningPublishStatus status{
      AuthorizedPlanningPublishStatus::plan_rejected};
  std::optional<AuthorizedPlanningResult> value;
  std::vector<std::string> issues;

  [[nodiscard]] bool published() const noexcept {
    return status == AuthorizedPlanningPublishStatus::published &&
           value.has_value();
  }
};

// The underlying plan publisher is private, so no consumer can observe the
// immutable plan before the matching execution lease is installed.
class AuthorizedPlanningResultPublisher {
 public:
  explicit AuthorizedPlanningResultPublisher(
      std::uint64_t last_plan_sequence = 0,
      std::uint64_t last_lease_sequence = 0) noexcept
      : plan_publisher_(last_plan_sequence),
        last_lease_sequence_(last_lease_sequence) {}

  [[nodiscard]] AuthorizedPlanningPublication publish(
      const PlanningResult& candidate,
      std::shared_ptr<const TimedPath> remaining_path,
      const PlanValidationLease& lease, double path_cost);
  [[nodiscard]] AuthorizedPlanningPublication reauthorize_current(
      std::shared_ptr<const TimedPath> remaining_path,
      const PlanValidationLease& lease);
  void revoke_current() noexcept { current_.reset(); }
  [[nodiscard]] const std::optional<AuthorizedPlanningResult>& current()
      const noexcept {
    return current_;
  }

 private:
  PlanningResultPublisher plan_publisher_;
  std::optional<AuthorizedPlanningResult> current_;
  std::uint64_t last_lease_sequence_{};
};

// Executes one design section 16 planning cycle. Commitment safety overrides
// are enforced before search; all ordinary failure and old-plan reuse paths
// are resolved before this call returns.
class MainPlanningLoop {
 public:
  MainPlanningLoop(MainPlanningLoopStages& stages,
                   AuthorizedPlanningResultPublisher& publisher,
                   ExecutionLeaseMonitor& lease_monitor,
                   MainPlanningLoopClock clock);
  MainPlanningLoop(MainPlanningLoopStages& stages,
                   AuthorizedPlanningResultPublisher& publisher,
                   ExecutionLeaseMonitor& lease_monitor,
                   MainPlanningLoopClock clock,
                   MainPlanningLoopInputSource& input_source);
  ~MainPlanningLoop();

  MainPlanningLoop(const MainPlanningLoop&) = delete;
  MainPlanningLoop& operator=(const MainPlanningLoop&) = delete;

  [[nodiscard]] PlanningCycleResult run_cycle(
      const PlanningCycleRequest& request);
  [[nodiscard]] const AlgorithmExperimentLog& experiment_log() const noexcept;

 private:
  [[nodiscard]] PlanningCycleResult run_cycle_impl(
      const PlanningCycleRequest& request);
  [[nodiscard]] PlanningCycleResult finish_failure(
      PlanningCycleResult result, PlanningCycleStatus status,
      PlanningState state, PlanningFailure failure,
      bool current_plan_reuse_allowed);
  [[nodiscard]] PlanningCycleResult finish_commitment_override(
      PlanningCycleResult result, const CommitmentSafetyCheckResult& safety,
      std::uint64_t lease_sequence, std::string message);

  MainPlanningLoopStages& stages_;
  MainPlanningLoopInputSource& input_source_;
  AuthorizedPlanningResultPublisher& publisher_;
  ExecutionLeaseMonitor& lease_monitor_;
  MainPlanningLoopClock clock_;
  std::unique_ptr<AlgorithmExperimentLog> experiment_log_;
};

}  // namespace underwater_planner::core
