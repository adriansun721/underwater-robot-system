#include "underwater_planner/core/main_planning_loop.hpp"

#include "underwater_planner/core/algorithm_diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>

namespace underwater_planner::core {
namespace {

// Execution-profile identity is an output of parameterization. It is not a
// mutable planning input and may advance while every locked map/model/policy
// dependency remains unchanged.
bool same_locked_dependencies(const PlanningDependencyVersions& left,
                              const PlanningDependencyVersions& right) {
  PlanningDependencyVersions normalized_left = left;
  PlanningDependencyVersions normalized_right = right;
  normalized_left.execution_profile_version = 1U;
  normalized_right.execution_profile_version = 1U;
  return normalized_left == normalized_right;
}

bool valid_capture(const ValidationInputCaptureResult& capture) {
  return capture.status == ValidationInputCaptureStatus::captured &&
         capture.inputs.has_value() && capture.inputs->source_revision != 0U;
}

std::optional<Duration> planning_cycle_limit(
    const AlgorithmRuntimeParameterSnapshot& parameters) {
  const std::optional<double> seconds =
      parameters.profile.statistical_risk.maximum_planning_duration_s;
  if (!seconds.has_value() || !std::isfinite(*seconds) || *seconds <= 0.0) {
    return std::nullopt;
  }
  const long double nanoseconds = static_cast<long double>(*seconds) * 1.0e9L;
  if (nanoseconds < 1.0L ||
      nanoseconds >
          static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
    return std::nullopt;
  }
  return Duration{static_cast<std::int64_t>(nanoseconds)};
}

bool cycle_deadline_exceeded(const PlanningCycleDiagnostics& diagnostics,
                             const MonotonicTime observed_at) {
  return diagnostics.cycle_started_at.nanoseconds >= 0 &&
         diagnostics.maximum_cycle_duration.nanoseconds > 0 &&
         observed_at.nanoseconds > diagnostics.cycle_started_at.nanoseconds &&
         observed_at.nanoseconds - diagnostics.cycle_started_at.nanoseconds >
             diagnostics.maximum_cycle_duration.nanoseconds;
}

bool valid_start(const PlanningCycleStart& start) {
  return validate(start.robot_state).valid && validate(start.cable_state).valid &&
         validate(start.reference_progress).valid;
}

bool same_path_point(const PathPoint& left, const PathPoint& right) {
  return left.arc_length_m == right.arc_length_m && left.x_m == right.x_m &&
         left.y_m == right.y_m && left.heading_rad == right.heading_rad &&
         left.curvature_per_m == right.curvature_per_m;
}

bool same_execution_sample(const ExecutionSample& left,
                           const ExecutionSample& right) {
  return left.arc_length_m == right.arc_length_m &&
         left.time_from_start.nanoseconds == right.time_from_start.nanoseconds &&
         left.ground_speed_mps == right.ground_speed_mps &&
         left.ground_acceleration_mps2 == right.ground_acceleration_mps2 &&
         left.payout_speed_mps == right.payout_speed_mps &&
         left.payout_acceleration_mps2 == right.payout_acceleration_mps2 &&
         left.tension_setpoint_n == right.tension_setpoint_n;
}

bool preserves_authorized_prefix(const TimedPath& prefix,
                                 const TimedPath& merged) {
  if (merged.geometry.points.size() < prefix.geometry.points.size() ||
      merged.execution_profile.samples.size() <
          prefix.execution_profile.samples.size()) {
    return false;
  }
  for (std::size_t index = 0; index < prefix.geometry.points.size(); ++index) {
    if (!same_path_point(prefix.geometry.points[index],
                         merged.geometry.points[index])) {
      return false;
    }
  }
  for (std::size_t index = 0; index < prefix.execution_profile.samples.size();
       ++index) {
    if (!same_execution_sample(prefix.execution_profile.samples[index],
                               merged.execution_profile.samples[index])) {
      return false;
    }
  }
  return true;
}

bool commitment_matches_terminal(const CommittedPlanningStart& committed) {
  if (!validate_authorized_prefix(committed.authorized_prefix).valid) {
    return false;
  }
  const PathPoint& point = committed.authorized_prefix.geometry.points.back();
  const ExecutionSample& sample =
      committed.authorized_prefix.execution_profile.samples.back();
  constexpr double tolerance = 1.0e-9;
  return std::abs(point.x_m - committed.terminal_robot_state.pose.x_m) <=
             tolerance &&
         std::abs(point.y_m - committed.terminal_robot_state.pose.y_m) <=
             tolerance &&
         std::abs(point.heading_rad -
                  committed.terminal_robot_state.pose.heading_rad) <= tolerance &&
         std::abs(point.curvature_per_m -
                  committed.terminal_robot_state.curvature_per_m) <= tolerance &&
         std::abs(sample.ground_speed_mps -
                  committed.terminal_robot_state.ground_speed_mps) <= tolerance &&
         committed.authorized_prefix.execution_profile.version ==
             committed.dependencies.execution_profile_version &&
         committed.authorized_prefix.execution_profile.operating_envelope_version ==
             committed.dependencies.execution_operating_envelope_version;
}

bool valid_terrain(const TerrainAnalysisStageResult& result,
                   const SynchronizedValidationInputs& inputs) {
  const MapSnapshot& map = inputs.planning_snapshot.map;
  return result.valid && result.issues.empty() &&
         result.terrain.source_map_version == inputs.dependencies.map_version &&
         result.terrain.source_map_version == map.version &&
         result.terrain.analysis_config_version != 0U &&
         (map.derived_configuration_version == 0U ||
          result.terrain.analysis_config_version ==
              map.derived_configuration_version) &&
         result.terrain.operating_domain_id ==
             inputs.dependencies.operating_domain_id;
}

bool prediction_matches_locked_context(
    const CablePrediction& prediction, const TimedPath& robot_path,
    const SynchronizedValidationInputs& inputs) {
  return prediction.dependencies.robot_path_version ==
             robot_path.geometry.metadata.path_version &&
         prediction.dependencies.execution_profile_version ==
             robot_path.execution_profile.version &&
         prediction.dependencies.cable_model_version ==
             inputs.dependencies.cable_model_version &&
         prediction.dependencies.reference_line_version ==
             inputs.dependencies.reference_line_version &&
         prediction.dependencies.execution_operating_envelope_version ==
             inputs.dependencies.execution_operating_envelope_version &&
         prediction.dependencies.uncertainty_envelope_version ==
             inputs.dependencies.uncertainty_envelope_version &&
         prediction.dependencies.uncertainty_envelope_generator_version ==
             inputs.dependencies.uncertainty_envelope_generator_version &&
         prediction.dependencies.sensor_mode == inputs.dependencies.sensor_mode &&
         prediction.dependencies.operating_domain_id ==
             inputs.dependencies.operating_domain_id &&
         prediction.dependencies.execution_operating_domain_id ==
             inputs.dependencies.operating_domain_id;
}

bool valid_lease(const PlanValidationLease& lease,
                 const PlanningResult& candidate,
                 const SynchronizedValidationInputs& latest,
                 const MonotonicTime now) {
  return lease.lease_sequence != 0U &&
         lease.plan_sequence_number == candidate.sequence_number &&
         lease.validated_at.nanoseconds >= latest.captured_at.nanoseconds &&
         lease.validated_at.nanoseconds <= now.nanoseconds &&
         lease.expires_at.nanoseconds > now.nanoseconds &&
         lease.robot_path_validation_passed &&
         lease.cable_corridor_validation_passed &&
         lease.cable_laying_validation_passed &&
         lease.dependencies() == candidate.dependencies() &&
         same_locked_dependencies(lease.dependencies(), latest.dependencies);
}

bool remaining_path_matches_authorization(
    const PlanningResult& plan, const std::shared_ptr<const TimedPath>& remaining,
    const PlanValidationLease& lease) {
  constexpr double tolerance = 1.0e-9;
  if (!remaining || !validate(*remaining).valid ||
      remaining->geometry.metadata.path_version !=
          plan.robot_trajectory.geometry.metadata.path_version ||
      remaining->geometry.metadata.coordinate_frame !=
          plan.robot_trajectory.geometry.metadata.coordinate_frame ||
      remaining->geometry.metadata.reference_line_version !=
          plan.robot_trajectory.geometry.metadata.reference_line_version ||
      remaining->geometry.metadata.interpolation_rule !=
          plan.robot_trajectory.geometry.metadata.interpolation_rule ||
      remaining->execution_profile.version != plan.execution_profile_version ||
      remaining->execution_profile.version != lease.execution_profile_version ||
      remaining->execution_profile.operating_envelope_version !=
          plan.execution_operating_envelope_version) {
    return false;
  }

  const double start_arc = lease.remaining_path_start_arc_length_m;
  const TimedPath& original = plan.robot_trajectory;
  if (std::abs(remaining->geometry.points.front().arc_length_m - start_arc) >
          tolerance ||
      std::abs(remaining->execution_profile.samples.front().arc_length_m -
               start_arc) > tolerance ||
      start_arc < original.geometry.points.front().arc_length_m - tolerance ||
      start_arc > original.geometry.points.back().arc_length_m + tolerance) {
    return false;
  }

  const auto interpolate_geometry = [start_arc](const GeometricPath& path)
      -> std::optional<PathPoint> {
    std::size_t right = 1U;
    while (right < path.points.size() &&
           path.points[right].arc_length_m < start_arc) {
      ++right;
    }
    if (right >= path.points.size()) return std::nullopt;
    const PathPoint& left = path.points[right - 1U];
    const PathPoint& upper = path.points[right];
    if (start_arc == left.arc_length_m) return left;
    if (start_arc == upper.arc_length_m) return upper;
    const double span = upper.arc_length_m - left.arc_length_m;
    if (!(span > 0.0)) return std::nullopt;
    const double ratio =
        std::clamp((start_arc - left.arc_length_m) / span, 0.0, 1.0);
    return PathPoint{
        start_arc,
        left.x_m + ratio * (upper.x_m - left.x_m),
        left.y_m + ratio * (upper.y_m - left.y_m),
        normalize_angle_radians(
            left.heading_rad +
            ratio * normalize_angle_radians(upper.heading_rad -
                                            left.heading_rad)),
        left.curvature_per_m +
            ratio * (upper.curvature_per_m - left.curvature_per_m)};
  };
  const auto interpolate_execution =
      [start_arc](const ExecutionProfile& profile)
      -> std::optional<ExecutionSample> {
    std::size_t right = 1U;
    while (right < profile.samples.size() &&
           profile.samples[right].arc_length_m < start_arc) {
      ++right;
    }
    if (right >= profile.samples.size()) return std::nullopt;
    const ExecutionSample& left = profile.samples[right - 1U];
    const ExecutionSample& upper = profile.samples[right];
    if (start_arc == left.arc_length_m) return left;
    if (start_arc == upper.arc_length_m) return upper;
    const double span = upper.arc_length_m - left.arc_length_m;
    if (!(span > 0.0)) return std::nullopt;
    const double ratio =
        std::clamp((start_arc - left.arc_length_m) / span, 0.0, 1.0);
    const auto blend = [ratio](const double a, const double b) {
      return a + ratio * (b - a);
    };
    return ExecutionSample{
        start_arc,
        Duration{static_cast<std::int64_t>(std::llround(
            left.time_from_start.nanoseconds +
            ratio * static_cast<double>(upper.time_from_start.nanoseconds -
                                        left.time_from_start.nanoseconds)))},
        blend(left.ground_speed_mps, upper.ground_speed_mps),
        blend(left.ground_acceleration_mps2, upper.ground_acceleration_mps2),
        blend(left.payout_speed_mps, upper.payout_speed_mps),
        blend(left.payout_acceleration_mps2,
              upper.payout_acceleration_mps2),
        blend(left.tension_setpoint_n, upper.tension_setpoint_n)};
  };

  const auto first_geometry = interpolate_geometry(original.geometry);
  const auto first_execution =
      interpolate_execution(original.execution_profile);
  if (!first_geometry || !first_execution) return false;
  TimedPath expected = original;
  expected.geometry.points.assign(1U, *first_geometry);
  expected.execution_profile.samples.assign(1U, *first_execution);
  for (const PathPoint& point : original.geometry.points) {
    if (point.arc_length_m > start_arc + tolerance) {
      expected.geometry.points.push_back(point);
    }
  }
  for (const ExecutionSample& sample : original.execution_profile.samples) {
    if (sample.arc_length_m > start_arc + tolerance) {
      expected.execution_profile.samples.push_back(sample);
    }
  }
  const std::int64_t time_origin =
      expected.execution_profile.samples.front().time_from_start.nanoseconds;
  for (ExecutionSample& sample : expected.execution_profile.samples) {
    sample.time_from_start.nanoseconds -= time_origin;
  }
  if (remaining->geometry.points.size() != expected.geometry.points.size() ||
      remaining->execution_profile.version != expected.execution_profile.version ||
      !same_execution_profile_content(remaining->execution_profile,
                                      expected.execution_profile)) {
    return false;
  }
  for (std::size_t index = 0U; index < expected.geometry.points.size();
       ++index) {
    if (!same_path_point(remaining->geometry.points[index],
                         expected.geometry.points[index])) {
      return false;
    }
  }
  return true;
}

bool lease_authorizes_plan(
    const PlanValidationLease& lease, const PlanningResult& plan,
    const std::shared_ptr<const TimedPath>& remaining_path) {
  return lease.lease_sequence != 0U &&
         lease.plan_sequence_number == plan.sequence_number &&
         lease.robot_path_validation_passed &&
         lease.cable_corridor_validation_passed &&
         lease.cable_laying_validation_passed &&
         lease.dependencies() == plan.dependencies() &&
         remaining_path_matches_authorization(plan, remaining_path, lease);
}

PlanningCycleStageMetric start_metric(
    const PlanningCycleStage stage, const MonotonicTime now,
    const std::uint64_t revision = 0U,
    const PlanningDependencyVersions& dependencies = {}) {
  PlanningCycleStageMetric metric;
  metric.stage = stage;
  metric.started_at = now;
  metric.source_revision = revision;
  metric.dependencies = dependencies;
  return metric;
}

void finish_metric(PlanningCycleStageMetric& metric, const MonotonicTime now,
                   const bool succeeded) {
  metric.duration.nanoseconds =
      std::max<std::int64_t>(0, now.nanoseconds - metric.started_at.nanoseconds);
  metric.succeeded = succeeded;
}

PlanningFailure planning_failure(const PlanningFailureCause cause,
                                 const PlanningCycleStage stage,
                                 std::string reason_code,
                                 std::string message) {
  return {cause, stage, std::move(reason_code), std::move(message)};
}

PlanningFailure cycle_timeout_failure(const PlanningCycleStage stage) {
  return planning_failure(
      PlanningFailureCause::planning_cycle_deadline_exceeded, stage,
      "PLANNING_CYCLE_DEADLINE_EXCEEDED",
      "planning cycle exceeded its total wall-clock deadline");
}

}  // namespace

AuthorizedPlanningPublication AuthorizedPlanningResultPublisher::publish(
    const PlanningResult& candidate,
    std::shared_ptr<const TimedPath> remaining_path,
    const PlanValidationLease& lease, const double path_cost) {
  AuthorizedPlanningPublication result;
  if (!std::isfinite(path_cost) || path_cost < 0.0) {
    result.status = AuthorizedPlanningPublishStatus::plan_rejected;
    result.issues.emplace_back(
        "candidate path cost must be finite and non-negative");
    return result;
  }
  if (lease.lease_sequence <= last_lease_sequence_ ||
      !lease_authorizes_plan(lease, candidate, remaining_path)) {
    result.status = AuthorizedPlanningPublishStatus::invalid_lease;
    result.issues.emplace_back(
        "execution lease does not authorize the candidate plan");
    return result;
  }

  PlanningResultPublication plan = plan_publisher_.publish(candidate);
  if (!plan.published()) {
    result.status = AuthorizedPlanningPublishStatus::plan_rejected;
    result.issues = std::move(plan.issues);
    return result;
  }
  current_ = AuthorizedPlanningResult{*plan.result, std::move(remaining_path),
                                      lease, path_cost};
  last_lease_sequence_ = lease.lease_sequence;
  result.status = AuthorizedPlanningPublishStatus::published;
  result.value = current_;
  return result;
}

AuthorizedPlanningPublication
AuthorizedPlanningResultPublisher::reauthorize_current(
    std::shared_ptr<const TimedPath> remaining_path,
    const PlanValidationLease& lease) {
  AuthorizedPlanningPublication result;
  if (!current_.has_value()) {
    result.status = AuthorizedPlanningPublishStatus::plan_rejected;
    result.issues.emplace_back("there is no current immutable plan to renew");
    return result;
  }

  const PlanningResult& plan = current_->plan.value();
  const PlanValidationLease& previous = current_->lease;
  if (lease.lease_sequence <= last_lease_sequence_ ||
      lease.validated_at.nanoseconds < previous.validated_at.nanoseconds ||
      lease.expires_at.nanoseconds <= lease.validated_at.nanoseconds ||
      !lease_authorizes_plan(lease, plan, remaining_path)) {
    result.status = AuthorizedPlanningPublishStatus::invalid_lease;
    result.issues.emplace_back(
        "fresh execution lease does not authorize the current immutable plan");
    return result;
  }

  current_ = AuthorizedPlanningResult{current_->plan,
                                      std::move(remaining_path), lease,
                                      current_->path_cost};
  last_lease_sequence_ = lease.lease_sequence;
  result.status = AuthorizedPlanningPublishStatus::published;
  result.value = current_;
  return result;
}

MainPlanningLoop::MainPlanningLoop(MainPlanningLoopStages& stages,
                                   AuthorizedPlanningResultPublisher& publisher,
                                   ExecutionLeaseMonitor& lease_monitor,
                                   MainPlanningLoopClock clock)
    : MainPlanningLoop(stages, publisher, lease_monitor, std::move(clock),
                       stages) {}

MainPlanningLoop::MainPlanningLoop(
    MainPlanningLoopStages& stages,
    AuthorizedPlanningResultPublisher& publisher,
    ExecutionLeaseMonitor& lease_monitor, MainPlanningLoopClock clock,
    MainPlanningLoopInputSource& input_source)
    : stages_(stages),
      input_source_(input_source),
      publisher_(publisher),
      lease_monitor_(lease_monitor),
      clock_(std::move(clock)),
      experiment_log_(std::make_unique<AlgorithmExperimentLog>()) {}

PathSwitchDecision MainPlanningLoopStages::decide_candidate(
    const std::optional<PlanValidityEvaluation>& validated_current,
    const PlanValidityEvaluation& validated_candidate,
    const double current_cost, const double candidate_cost,
    const SynchronizedValidationInputs& latest_inputs,
    const MonotonicTime now) {
  observe_candidate_decision_inputs(validated_current, validated_candidate,
                                    latest_inputs);
  return stability_manager_.decide_path_switch(
      validated_current, validated_candidate, current_cost, candidate_cost,
      now);
}

MainPlanningLoop::~MainPlanningLoop() = default;

const AlgorithmExperimentLog& MainPlanningLoop::experiment_log() const
    noexcept {
  return *experiment_log_;
}

PlanningCycleResult MainPlanningLoop::finish_failure(
    PlanningCycleResult result, PlanningCycleStatus status,
    PlanningState state, PlanningFailure failure,
    const bool current_plan_reuse_allowed) {
  if (clock_) {
    const MonotonicTime observed_at = clock_();
    if (cycle_deadline_exceeded(result.diagnostics, observed_at)) {
      status = PlanningCycleStatus::cycle_timeout;
      state = PlanningState::timeout;
      failure = cycle_timeout_failure(failure.stage);
    }
  }
  result.status = status;
  result.state = state;
  result.root_cause = std::move(failure);
  const bool total_cycle_timeout =
      result.root_cause->cause ==
      PlanningFailureCause::planning_cycle_deadline_exceeded;
  result.issues.clear();
  result.issues.push_back(result.root_cause->message);

  const std::optional<AuthorizedPlanningResult> current = publisher_.current();
  if (current_plan_reuse_allowed && current.has_value()) {
    PlanningCycleStageMetric capture_metric =
        start_metric(PlanningCycleStage::decision_context_capture, clock_());
    ValidationInputCaptureResult recaptured =
        input_source_.capture(capture_metric.started_at);
    result.replay_input_captures.push_back(recaptured);
    bool capture_ok = valid_capture(recaptured);
    if (capture_ok) {
      const std::uint64_t minimum_revision = std::max(
          result.diagnostics.initial_source_revision,
          result.diagnostics.decision_source_revision);
      capture_ok = recaptured.inputs->source_revision >= minimum_revision;
      if (capture_ok && result.decision_inputs &&
          recaptured.inputs->source_revision ==
              result.diagnostics.decision_source_revision) {
        capture_ok = recaptured.inputs->dependencies ==
                     result.diagnostics.decision_dependencies;
      } else if (capture_ok && !result.decision_inputs &&
                 recaptured.inputs->source_revision ==
                     result.diagnostics.initial_source_revision) {
        capture_ok = recaptured.inputs->dependencies ==
                     result.diagnostics.initial_dependencies;
      }
    }
    if (capture_ok) {
      capture_metric.source_revision = recaptured.inputs->source_revision;
      capture_metric.dependencies = recaptured.inputs->dependencies;
    }
    finish_metric(capture_metric, clock_(), capture_ok);
    result.diagnostics.stages.push_back(capture_metric);

    if (capture_ok) {
      auto latest_inputs = std::make_shared<SynchronizedValidationInputs>(
          std::move(*recaptured.inputs));
      result.decision_inputs = latest_inputs;
      result.diagnostics.decision_source_revision =
          latest_inputs->source_revision;
      result.diagnostics.decision_dependencies = latest_inputs->dependencies;

      PlanningCycleStageMetric validation_metric = start_metric(
          PlanningCycleStage::current_plan_revalidation, clock_(),
          latest_inputs->source_revision, latest_inputs->dependencies);
      const PlanValidityEvaluation validation = stages_.revalidate_plan(
          current->plan.value(), PlanValidationTarget::authorized_current,
          *latest_inputs, validation_metric.started_at);
      const MonotonicTime lease_checked_at = clock_();
      if (validation.status ==
          PlanValidationStatus::covariance_envelope_breach) {
        result.status = PlanningCycleStatus::covariance_envelope_breached;
        result.state = PlanningState::covariance_envelope_breach;
        result.root_cause = planning_failure(
            PlanningFailureCause::covariance_envelope_breach,
            PlanningCycleStage::current_plan_revalidation,
            "CURRENT_PLAN_COVARIANCE_ENVELOPE_BREACH",
            "current plan revalidation breached the covariance envelope");
        result.issues.assign(1U, result.root_cause->message);
      }
      const bool validation_ok =
          validation.valid && validation.status == PlanValidationStatus::valid &&
          validation.action == PlanValidationAction::reuse &&
          validation.lease.has_value() && validation.remaining_path &&
          validation.lease->lease_sequence != current->lease.lease_sequence &&
          valid_lease(*validation.lease, current->plan.value(), *latest_inputs,
                      lease_checked_at);
      finish_metric(validation_metric, lease_checked_at, validation_ok);
      result.diagnostics.stages.push_back(validation_metric);

      if (validation_ok) {
        lease_monitor_.revokeLease(
            current->lease.lease_sequence, result.root_cause->reason_code,
            result.root_cause->message, lease_checked_at);
        result.revoked_lease_sequence = current->lease.lease_sequence;
        PlanningCycleStageMetric authorization_metric = start_metric(
            PlanningCycleStage::current_plan_reauthorization, clock_(),
            latest_inputs->source_revision, latest_inputs->dependencies);
        AuthorizedPlanningPublication publication =
            publisher_.reauthorize_current(validation.remaining_path,
                                           *validation.lease);
        const MonotonicTime reauthorized_at = clock_();
        finish_metric(authorization_metric, reauthorized_at,
                      publication.published());
        result.diagnostics.stages.push_back(authorization_metric);
        if (publication.published()) {
          if (!total_cycle_timeout &&
              cycle_deadline_exceeded(result.diagnostics, reauthorized_at)) {
            result.status = PlanningCycleStatus::cycle_timeout;
            result.state = PlanningState::timeout;
            result.root_cause = cycle_timeout_failure(
                PlanningCycleStage::current_plan_reauthorization);
            result.issues = {result.root_cause->message};
          } else if (!total_cycle_timeout) {
            result.status = PlanningCycleStatus::current_plan_reused;
            result.state = PlanningState::path_valid;
          }
          result.publication = publication.value;
          return result;
        }
      }
    }
  }

  if (current.has_value()) {
    if (!result.revoked_lease_sequence.has_value()) {
      lease_monitor_.revokeLease(
          current->lease.lease_sequence, result.root_cause->reason_code,
          result.root_cause->message,
          clock_ ? clock_() : MonotonicTime{0});
      result.revoked_lease_sequence = current->lease.lease_sequence;
    }
    publisher_.revoke_current();
  }
  stages_.request_controlled_stop(*result.root_cause,
                                  clock_ ? clock_() : MonotonicTime{0});
  result.controlled_stop_required = true;
  return result;
}

PlanningCycleResult MainPlanningLoop::finish_commitment_override(
    PlanningCycleResult result, const CommitmentSafetyCheckResult& safety,
    const std::uint64_t lease_sequence, std::string message) {
  result.commitment_safety =
      std::make_shared<CommitmentSafetyCheckResult>(safety);
  CommitmentSafetySupervisor supervisor(
      lease_monitor_, [this](const CommitmentSafetyCheckResult& assessment,
                             const MonotonicTime at) {
        stages_.request_commitment_safety_stop(assessment, at);
      });
  const CommitmentSafetyEnforcement enforcement =
      supervisor.enforce(safety, lease_sequence, clock_());
  result.revoked_lease_sequence = enforcement.revoked_lease_sequence;

  const std::optional<AuthorizedPlanningResult> current = publisher_.current();
  if (current.has_value() &&
      current->lease.lease_sequence == lease_sequence) {
    publisher_.revoke_current();
  }

  const std::string reason_code{
      commitment_safety_reason_code(safety.event)};
  result.status = PlanningCycleStatus::commitment_overridden;
  result.state = safety.action == CommitmentSafetyAction::stop
                     ? PlanningState::emergency_stop
                     : PlanningState::normal_planning;
  result.root_cause = planning_failure(
      PlanningFailureCause::commitment_safety_event,
      PlanningCycleStage::commitment_validation, reason_code,
      std::move(message));
  result.issues = {result.root_cause->message};
  result.controlled_stop_required = enforcement.stop_channel_requested;
  result.urgent_replan_required =
      safety.action == CommitmentSafetyAction::replan_urgent;
  return result;
}

PlanningCycleResult MainPlanningLoop::run_cycle(
    const PlanningCycleRequest& request) {
  const std::optional<AuthorizedPlanningResult> initial_authorization =
      publisher_.current();
  PlanningCycleResult result = run_cycle_impl(request);
  result.replay_initial_authorization = initial_authorization;
  const ExperimentRecordValidation recording = experiment_log_->append(
      AlgorithmDiagnosticsRecorder::capture(request, result));
  result.experiment_recorded = true;
  result.experiment_record_valid = recording.valid;
  result.experiment_recording_issues = recording.issues;
  return result;
}

PlanningCycleResult MainPlanningLoop::run_cycle_impl(
    const PlanningCycleRequest& request) {
  PlanningCycleResult result;
  result.diagnostics.cycle_sequence = request.cycle_sequence;
  result.diagnostics.random_seed = request.random_seed;
  result.diagnostics.parameters = input_source_.capture_runtime_parameters();
  result.diagnostics.cycle_started_at = request.triggered_at;
  const std::optional<Duration> cycle_limit =
      planning_cycle_limit(result.diagnostics.parameters);
  if (cycle_limit.has_value()) {
    result.diagnostics.maximum_cycle_duration = *cycle_limit;
  }
  if (request.cycle_sequence == 0U || request.triggered_at.nanoseconds < 0 ||
      !clock_ || !cycle_limit.has_value()) {
    return finish_failure(
        std::move(result), PlanningCycleStatus::input_invalid,
        PlanningState::input_invalid,
        planning_failure(PlanningFailureCause::input_invalid,
                         PlanningCycleStage::capture_inputs,
                         "PLANNING_CYCLE_REQUEST_INVALID",
                         "invalid planning cycle request"),
        false);
  }

  auto metric = start_metric(PlanningCycleStage::capture_inputs, clock_());
  const auto captured = std::make_unique<ValidationInputCaptureResult>(
      input_source_.capture(metric.started_at));
  result.replay_input_captures.push_back(*captured);
  finish_metric(metric, clock_(), valid_capture(*captured));
  result.diagnostics.stages.push_back(metric);
  if (!valid_capture(*captured)) {
    return finish_failure(
        std::move(result), PlanningCycleStatus::input_invalid,
        PlanningState::input_invalid,
        planning_failure(PlanningFailureCause::input_invalid,
                         PlanningCycleStage::capture_inputs,
                         "SYNCHRONIZED_INPUT_CAPTURE_FAILED",
                         "initial synchronized input capture failed"),
        false);
  }

  result.initial_inputs = std::make_shared<const SynchronizedValidationInputs>(
      *captured->inputs);
  const SynchronizedValidationInputs& initial_inputs = *result.initial_inputs;
  result.diagnostics.initial_source_revision = initial_inputs.source_revision;
  result.diagnostics.initial_dependencies = initial_inputs.dependencies;
  result.diagnostics.stages.back().source_revision = initial_inputs.source_revision;
  result.diagnostics.stages.back().dependencies = initial_inputs.dependencies;

  if (result.diagnostics.parameters.profile.mode ==
      ParameterProfileMode::production) {
    const ParameterValidationResult spatial_binding =
        validate_spatial_domain_snapshot(result.diagnostics.parameters.profile,
                                         initial_inputs.planning_snapshot);
    if (!spatial_binding.valid) {
      return finish_failure(
          std::move(result), PlanningCycleStatus::input_invalid,
          PlanningState::input_invalid,
          planning_failure(PlanningFailureCause::input_invalid,
                           PlanningCycleStage::capture_inputs,
                           "SPATIAL_DOMAIN_SNAPSHOT_MISMATCH",
                           spatial_binding.issues.front().message),
          false);
    }
  }

  const CommittedPlanningStart* committed_start = nullptr;
  if (request.committed_start.has_value()) {
    result.replay_committed_lease_revoked =
        lease_monitor_.isRevoked(request.committed_start->lease_sequence);
    if (result.replay_committed_lease_revoked) {
      result.ignored_revoked_commitment_lease_sequence =
          request.committed_start->lease_sequence;
    } else {
      committed_start = &*request.committed_start;
    }
  }

  PlanningCycleStart start;
  if (committed_start != nullptr) {
    const CommittedPlanningStart& committed = *committed_start;
    start.source = PlanningStartSource::committed_segment_terminal;
    start.robot_state = committed.terminal_robot_state;
    start.source_plan_sequence_number = committed.source_plan_sequence_number;
    start.lease_sequence = committed.lease_sequence;
    if (committed.source_plan_sequence_number == 0U ||
        committed.lease_sequence == 0U ||
        !validate(start.robot_state).valid) {
      result.start = start;
      return finish_failure(
          std::move(result), PlanningCycleStatus::commitment_invalid,
          PlanningState::input_invalid,
          planning_failure(
              PlanningFailureCause::input_invalid,
              PlanningCycleStage::commitment_validation,
              "COMMITTED_START_INVALID",
              "committed terminal state is invalid or version-mismatched"),
          false);
    }
    if (committed.dependencies != initial_inputs.dependencies) {
      result.start = start;
      CommitmentSafetyObservation observation;
      observation.observed_event =
          CommitmentSafetyEvent::dependency_version_change;
      const CommitmentSafetyCheckResult safety =
          CommitmentSafetyEvaluator().evaluate(observation);
      return finish_commitment_override(
          std::move(result), safety, committed.lease_sequence,
          "dependency version change overrode committed authorization");
    }
    if (!commitment_matches_terminal(committed)) {
      result.start = start;
      return finish_failure(
          std::move(result), PlanningCycleStatus::commitment_invalid,
          PlanningState::input_invalid,
          planning_failure(
              PlanningFailureCause::input_invalid,
              PlanningCycleStage::commitment_validation,
              "COMMITTED_START_INVALID",
              "committed terminal state is invalid or version-mismatched"),
          false);
    }
  } else {
    start.source = PlanningStartSource::synchronized_actual_state;
    start.robot_state = initial_inputs.robot_state;
    start.cable_state = initial_inputs.cable_state;
    start.reference_progress = initial_inputs.reference_progress;
    if (!valid_start(start)) {
      result.start = start;
      return finish_failure(
          std::move(result), PlanningCycleStatus::input_invalid,
          PlanningState::input_invalid,
          planning_failure(PlanningFailureCause::input_invalid,
                           PlanningCycleStage::capture_inputs,
                           "ACTUAL_PLANNING_START_INVALID",
                           "synchronized actual planning start is invalid"),
          false);
    }
  }
  if (committed_start == nullptr) result.start = start;

  metric = start_metric(PlanningCycleStage::terrain_analysis, clock_(),
                        initial_inputs.source_revision,
                        initial_inputs.dependencies);
  const auto terrain_result = std::make_shared<TerrainAnalysisStageResult>(
      stages_.analyze_terrain(initial_inputs));
  const bool terrain_ok = valid_terrain(*terrain_result, initial_inputs);
  finish_metric(metric, clock_(), terrain_ok);
  result.diagnostics.stages.push_back(metric);
  result.artifacts.terrain = terrain_result;
  if (!terrain_ok) {
    return finish_failure(
        std::move(result), PlanningCycleStatus::terrain_analysis_failed,
        PlanningState::input_invalid,
        planning_failure(
            PlanningFailureCause::input_invalid,
            PlanningCycleStage::terrain_analysis,
            "TERRAIN_ANALYSIS_CONTEXT_MISMATCH",
            "terrain analysis did not match the locked map snapshot"),
        false);
  }

  const LockedPlanningCycleContext context{initial_inputs,
                                           terrain_result->terrain};

  if (committed_start != nullptr) {
    metric = start_metric(PlanningCycleStage::commitment_validation, clock_(),
                          initial_inputs.source_revision,
                          initial_inputs.dependencies);
    const auto commitment =
        std::make_shared<CommitmentValidationStageResult>(
            stages_.validate_commitment(
                committed_start->authorized_prefix,
                initial_inputs.cable_state, initial_inputs.reference_progress,
                context));
    CommitmentSafetyObservation observation;
    observation.robot_validation = commitment->robot_validation;
    observation.cable_validation = commitment->cable_validation;
    observation.observed_event = commitment->observed_safety_event;
    observation.obstacle_stopping = commitment->obstacle_stopping;
    const CommitmentSafetyCheckResult safety =
        CommitmentSafetyEvaluator().evaluate(observation);
    result.commitment_safety =
        std::make_shared<CommitmentSafetyCheckResult>(safety);
    CableState derived_terminal_cable;
    if (commitment->cable_validation.terminal_cable_state.has_value()) {
      derived_terminal_cable =
          *commitment->cable_validation.terminal_cable_state;
    }
    derived_terminal_cable.laying_memory =
        commitment->cable_validation.laying_result.terminal_memory;
    const bool commitment_ok =
        safety.is_safe && commitment->valid && commitment->issues.empty() &&
        commitment->cable_validation.cable_prediction.has_value() &&
        commitment->cable_validation.terminal_cable_state.has_value() &&
        commitment->cable_validation.cable_prediction->validity ==
            CableModelValidity::valid &&
        commitment->cable_validation.corridor_result.validity ==
            CorridorEvaluationValidity::valid &&
        commitment->cable_validation.corridor_result.hard_feasible &&
        commitment->cable_validation.laying_result.valid &&
        commitment->cable_validation.laying_result.hard_feasible &&
        prediction_matches_locked_context(
            *commitment->cable_validation.cable_prediction,
            committed_start->authorized_prefix, initial_inputs) &&
        validate(derived_terminal_cable).valid &&
        validate(commitment->terminal_reference_progress).valid &&
        commitment->terminal_reference_progress.reference_line_version ==
            initial_inputs.dependencies.reference_line_version;
    finish_metric(metric, clock_(), commitment_ok);
    result.diagnostics.stages.push_back(metric);
    result.artifacts.commitment_validation = commitment;
    if (!safety.is_safe) {
      return finish_commitment_override(
          std::move(result), safety,
          committed_start->lease_sequence,
          "commitment safety event overrode authorization");
    }
    if (!commitment_ok) {
      return finish_failure(
          std::move(result), PlanningCycleStatus::commitment_invalid,
          PlanningState::input_invalid,
          planning_failure(
              PlanningFailureCause::input_invalid,
              PlanningCycleStage::commitment_validation,
              "COMMITMENT_VALIDATION_FAILED",
              "commitment terminal cable or reference state validation failed"),
          false);
    }
    start.cable_state = std::move(derived_terminal_cable);
    start.reference_progress = commitment->terminal_reference_progress;
    if (!valid_start(start)) {
      return finish_failure(
          std::move(result), PlanningCycleStatus::commitment_invalid,
          PlanningState::input_invalid,
          planning_failure(PlanningFailureCause::input_invalid,
                           PlanningCycleStage::commitment_validation,
                           "COMMITMENT_TERMINAL_STATE_INVALID",
                           "derived commitment terminal state is invalid"),
          false);
    }
    result.start = start;
  }

  metric = start_metric(PlanningCycleStage::search, clock_(),
                        initial_inputs.source_revision,
                        initial_inputs.dependencies);
  const auto search_result = std::make_shared<HybridAStarPlanningResult>(
      stages_.search(start, context));
  const bool search_ok =
      search_result->state == PlanningState::success &&
      validate(search_result->robot_path).valid;
  finish_metric(metric, clock_(), search_ok);
  result.diagnostics.stages.push_back(metric);
  result.artifacts.search = search_result;
  if (!search_ok) {
    PlanningFailure failure;
    failure.stage = PlanningCycleStage::search;
    bool reuse_allowed = false;
    PlanningState failure_state = search_result->state;
    if (search_result->state == PlanningState::timeout &&
        search_result->diagnostics.deadline_exceeded &&
        search_result->diagnostics.active_label_budget_exhausted) {
      failure.cause = PlanningFailureCause::input_invalid;
      failure.reason_code = "SEARCH_TIMEOUT_DIAGNOSTICS_CONTRADICTORY";
      failure.message =
          "search reported mutually exclusive timeout root causes";
      failure_state = PlanningState::input_invalid;
    } else if (search_result->state == PlanningState::timeout &&
        search_result->diagnostics.deadline_exceeded) {
      failure.cause = PlanningFailureCause::search_deadline_exceeded;
      failure.reason_code = "SEARCH_DEADLINE_EXCEEDED";
      failure.message = "search exceeded its wall-clock deadline";
      reuse_allowed = true;
    } else if (search_result->state == PlanningState::timeout &&
               search_result->diagnostics.active_label_budget_exhausted) {
      failure.cause = PlanningFailureCause::search_label_budget_exhausted;
      failure.reason_code = "SEARCH_LABEL_BUDGET_EXHAUSTED";
      failure.message = "search exhausted its active-label budget";
      reuse_allowed = true;
    } else if (search_result->state == PlanningState::timeout) {
      failure.cause = PlanningFailureCause::search_budget_exhausted;
      failure.reason_code = "SEARCH_BUDGET_EXHAUSTED";
      failure.message = "search exhausted a bounded planning resource";
      reuse_allowed = true;
    } else if (search_result->state ==
               PlanningState::no_solution_under_covariance_envelope) {
      failure.cause =
          PlanningFailureCause::no_solution_under_covariance_envelope;
      failure.reason_code = "NO_SOLUTION_UNDER_COVARIANCE_ENVELOPE";
      failure.message =
          "search found no solution under the locked covariance envelope";
    } else if (search_result->state == PlanningState::no_solution) {
      failure.cause = PlanningFailureCause::no_solution;
      failure.reason_code = "SEARCH_NO_SOLUTION";
      failure.message = "search found no feasible path";
      reuse_allowed = true;
    } else {
      failure.cause = PlanningFailureCause::input_invalid;
      failure.reason_code = "SEARCH_INPUT_INVALID";
      failure.message = "search rejected its locked planning inputs";
      failure_state = PlanningState::input_invalid;
    }
    return finish_failure(std::move(result),
                          PlanningCycleStatus::search_failed, failure_state,
                          std::move(failure), reuse_allowed);
  }

  metric = start_metric(PlanningCycleStage::smoothing, clock_(),
                        initial_inputs.source_revision,
                        initial_inputs.dependencies);
  const auto smoothing_result = std::make_shared<SmoothingResult>(
      stages_.smooth(search_result->robot_path, start, context));
  const bool smoothing_ok =
      smoothing_result->status == SmoothingStatus::success &&
      smoothing_result->path.has_value() &&
      validate(*smoothing_result->path).valid;
  finish_metric(metric, clock_(), smoothing_ok);
  result.diagnostics.stages.push_back(metric);
  result.artifacts.smoothing = smoothing_result;
  const GeometricPath* candidate_geometry = nullptr;
  if (smoothing_ok) {
    candidate_geometry = &*smoothing_result->path;
  } else {
    metric = start_metric(PlanningCycleStage::raw_path_trackability_validation,
                          clock_(), initial_inputs.source_revision,
                          initial_inputs.dependencies);
    const TrackabilityResult raw_trackability =
        stages_.validate_raw_path_trackability(search_result->robot_path, start,
                                               context);
    const bool raw_path_ok = raw_trackability.valid;
    finish_metric(metric, clock_(), raw_path_ok);
    result.diagnostics.stages.push_back(metric);
    if (!raw_path_ok) {
      PlanningFailure failure;
      failure.stage = PlanningCycleStage::smoothing;
      bool reuse_allowed = true;
      PlanningState failure_state = PlanningState::no_solution;
      if (smoothing_result->status == SmoothingStatus::solver_timeout) {
        failure.cause = PlanningFailureCause::smoothing_deadline_exceeded;
        failure.reason_code = "SMOOTHING_DEADLINE_EXCEEDED";
        failure.message =
            "smoothing timed out and the raw path failed trackability";
        failure_state = PlanningState::timeout;
      } else if (smoothing_result->status ==
                 SmoothingStatus::boundary_state_invalid) {
        failure.cause = PlanningFailureCause::input_invalid;
        failure.reason_code = "SMOOTHING_INPUT_INVALID";
        failure.message =
            "smoothing boundary input and raw-path trackability were invalid";
        failure_state = PlanningState::input_invalid;
        reuse_allowed = false;
      } else {
        failure.cause = PlanningFailureCause::smoothing_infeasible;
        failure.reason_code = "SMOOTHING_NO_TRACKABLE_PATH";
        failure.message =
            "neither smoothing nor the raw search path was trackable";
      }
      return finish_failure(std::move(result),
                            PlanningCycleStatus::smoothing_failed,
                            failure_state, std::move(failure), reuse_allowed);
    }
    if (smoothing_result->status == SmoothingStatus::success ||
        smoothing_result->status ==
            SmoothingStatus::boundary_state_invalid) {
      return finish_failure(
          std::move(result), PlanningCycleStatus::smoothing_failed,
          PlanningState::input_invalid,
          planning_failure(
              PlanningFailureCause::input_invalid,
              PlanningCycleStage::smoothing,
              "SMOOTHING_STAGE_OUTPUT_INVALID",
              "smoothing returned a contradictory or invalid boundary result"),
          false);
    }
    result.used_raw_search_path = true;
    if (smoothing_result->status == SmoothingStatus::solver_timeout) {
      result.root_cause = planning_failure(
          PlanningFailureCause::smoothing_deadline_exceeded,
          PlanningCycleStage::smoothing, "SMOOTHING_DEADLINE_EXCEEDED",
          "smoothing timed out; the raw path passed the same trackability contract");
      result.issues.assign(1U, result.root_cause->message);
    } else {
      result.root_cause = planning_failure(
          PlanningFailureCause::smoothing_infeasible,
          PlanningCycleStage::smoothing,
          "SMOOTHING_INFEASIBLE_RAW_PATH_FALLBACK",
          "smoothing was infeasible; the raw path passed the same trackability contract");
      result.issues.assign(1U, result.root_cause->message);
    }
    candidate_geometry = &search_result->robot_path;
  }

  metric = start_metric(PlanningCycleStage::parameterization, clock_(),
                        initial_inputs.source_revision,
                        initial_inputs.dependencies);
  const auto parameterization_result = std::make_shared<ParameterizationResult>(
      stages_.parameterize(*candidate_geometry, start, context));
  const bool parameterization_ok =
      parameterization_result->status == ParameterizationStatus::success &&
      parameterization_result->trajectory.has_value() &&
      validate(*parameterization_result->trajectory).valid;
  finish_metric(metric, clock_(), parameterization_ok);
  result.diagnostics.stages.push_back(metric);
  result.artifacts.parameterization = parameterization_result;
  if (!parameterization_ok) {
    PlanningFailure failure;
    failure.stage = PlanningCycleStage::parameterization;
    const bool deadline_exceeded =
        parameterization_result->status ==
        ParameterizationStatus::deadline_exceeded;
    const bool input_invalid =
        parameterization_result->status == ParameterizationStatus::success ||
        parameterization_result->status ==
            ParameterizationStatus::initial_state_invalid ||
        parameterization_result->status ==
            ParameterizationStatus::execution_envelope_mismatch ||
        parameterization_result->status ==
            ParameterizationStatus::numerically_invalid;
    if (deadline_exceeded) {
      failure.cause =
          PlanningFailureCause::parameterization_deadline_exceeded;
      failure.reason_code = "PARAMETERIZATION_DEADLINE_EXCEEDED";
      failure.message = "trajectory parameterization exceeded its deadline";
    } else if (input_invalid) {
      failure.cause = PlanningFailureCause::input_invalid;
      failure.reason_code = "PARAMETERIZATION_INPUT_INVALID";
      failure.message = "trajectory parameterization inputs were invalid";
    } else {
      failure.cause = PlanningFailureCause::parameterization_infeasible;
      failure.reason_code = "PARAMETERIZATION_INFEASIBLE";
      failure.message =
          "trajectory parameterization found no feasible execution profile";
    }
    return finish_failure(
        std::move(result), PlanningCycleStatus::parameterization_failed,
        deadline_exceeded
            ? PlanningState::timeout
            : (input_invalid ? PlanningState::input_invalid
                             : PlanningState::no_solution),
        std::move(failure), deadline_exceeded || !input_invalid);
  }

  std::shared_ptr<const TimedPath> complete_trajectory =
      std::make_shared<const TimedPath>(*parameterization_result->trajectory);
  if (committed_start != nullptr) {
    metric = start_metric(PlanningCycleStage::commitment_merge, clock_(),
                          initial_inputs.source_revision,
                          initial_inputs.dependencies);
    const auto merge_result = std::make_shared<TimedPathMergeResult>(
        stages_.merge_commitment(committed_start->authorized_prefix,
                                 *parameterization_result->trajectory, context));
    const bool merge_ok =
        merge_result->valid && merge_result->trajectory.has_value() &&
        validate(*merge_result->trajectory).valid &&
        preserves_authorized_prefix(committed_start->authorized_prefix,
                                    *merge_result->trajectory);
    finish_metric(metric, clock_(), merge_ok);
    result.diagnostics.stages.push_back(metric);
    result.artifacts.commitment_merge = merge_result;
    if (!merge_ok) {
      return finish_failure(
          std::move(result), PlanningCycleStatus::commitment_invalid,
          PlanningState::no_solution,
          planning_failure(
              PlanningFailureCause::no_solution,
              PlanningCycleStage::commitment_merge,
              "COMMITMENT_MERGE_FAILED",
              "authorized commitment prefix was not preserved by the merge"),
          false);
    }
    complete_trajectory =
        std::make_shared<const TimedPath>(*merge_result->trajectory);
  }

  metric = start_metric(
      PlanningCycleStage::complete_robot_path_validation, clock_(),
      initial_inputs.source_revision, initial_inputs.dependencies);
  const auto complete_robot_validation =
      std::make_shared<PathCandidateVerificationResult>(
          stages_.verify_complete_robot_path(*complete_trajectory, start,
                                             context));
  const bool complete_robot_ok =
      complete_robot_validation->valid &&
      complete_robot_validation->status ==
          PathCandidateVerificationStatus::valid;
  finish_metric(metric, clock_(), complete_robot_ok);
  result.diagnostics.stages.push_back(metric);
  result.artifacts.complete_robot_path_validation =
      complete_robot_validation;
  if (!complete_robot_ok) {
    const bool input_invalid =
        complete_robot_validation->status ==
            PathCandidateVerificationStatus::valid ||
        complete_robot_validation->status ==
        PathCandidateVerificationStatus::input_invalid;
    return finish_failure(
        std::move(result), PlanningCycleStatus::robot_path_validation_failed,
        input_invalid ? PlanningState::input_invalid
                      : PlanningState::no_solution,
        planning_failure(
            input_invalid ? PlanningFailureCause::input_invalid
                          : PlanningFailureCause::no_solution,
            PlanningCycleStage::complete_robot_path_validation,
            input_invalid ? "COMPLETE_ROBOT_PATH_INPUT_INVALID"
                          : "COMPLETE_ROBOT_PATH_INVALID",
            "complete robot trajectory failed the independent path audit"),
        !input_invalid);
  }

  metric = start_metric(PlanningCycleStage::cable_validation, clock_(),
                        initial_inputs.source_revision,
                        initial_inputs.dependencies);
  const auto cable_result = std::make_shared<TimedCableCandidateResult>(
      stages_.verify_cable(*complete_trajectory, initial_inputs.cable_state,
                           context));
  CableState derived_terminal_cable =
      cable_result->cable_prediction.has_value()
          ? cable_result->cable_prediction->terminal_state
          : CableState{};
  derived_terminal_cable.laying_memory =
      cable_result->laying_result.terminal_memory;
  const bool cable_ok =
      cable_result->valid &&
      cable_result->status == TimedCableValidationStatus::valid &&
      !cable_result->stop_required &&
      cable_result->cable_prediction.has_value() &&
      cable_result->cable_prediction->validity == CableModelValidity::valid &&
      cable_result->corridor_result.validity ==
          CorridorEvaluationValidity::valid &&
      cable_result->corridor_result.hard_feasible &&
      cable_result->laying_result.valid &&
      cable_result->laying_result.hard_feasible &&
      prediction_matches_locked_context(*cable_result->cable_prediction,
                                        *complete_trajectory, initial_inputs) &&
      validate(derived_terminal_cable).valid;
  finish_metric(metric, clock_(), cable_ok);
  result.diagnostics.stages.push_back(metric);
  result.artifacts.cable_validation = cable_result;
  if (!cable_ok) {
    if (cable_result->status ==
        TimedCableValidationStatus::covariance_envelope_breach) {
      return finish_failure(
          std::move(result),
          PlanningCycleStatus::covariance_envelope_breached,
          PlanningState::covariance_envelope_breach,
          planning_failure(
              PlanningFailureCause::covariance_envelope_breach,
              PlanningCycleStage::cable_validation,
              "CANDIDATE_COVARIANCE_ENVELOPE_BREACH",
              "timed cable validation breached the covariance envelope"),
          false);
    }
    if (cable_result->status ==
        TimedCableValidationStatus::covariance_envelope_unavailable) {
      return finish_failure(
          std::move(result), PlanningCycleStatus::cable_validation_failed,
          PlanningState::no_solution_under_covariance_envelope,
          planning_failure(
              PlanningFailureCause::no_solution_under_covariance_envelope,
              PlanningCycleStage::cable_validation,
              "CANDIDATE_COVARIANCE_ENVELOPE_UNAVAILABLE",
              "timed cable validation could not use the locked covariance envelope"),
          false);
    }
    const bool input_invalid =
        cable_result->status == TimedCableValidationStatus::valid ||
        cable_result->status == TimedCableValidationStatus::input_invalid ||
        cable_result->status ==
            TimedCableValidationStatus::cable_model_invalid ||
        cable_result->status == TimedCableValidationStatus::corridor_invalid;
    return finish_failure(
        std::move(result), PlanningCycleStatus::cable_validation_failed,
        input_invalid ? PlanningState::input_invalid
                      : PlanningState::no_solution,
        planning_failure(input_invalid ? PlanningFailureCause::input_invalid
                                       : PlanningFailureCause::no_solution,
                         PlanningCycleStage::cable_validation,
                         "TIMED_CABLE_CANDIDATE_INVALID",
                         "timed cable candidate validation failed"),
        !input_invalid);
  }

  metric = start_metric(PlanningCycleStage::candidate_assembly, clock_(),
                        initial_inputs.source_revision,
                        initial_inputs.dependencies);
  const auto candidate_metadata =
      std::make_shared<PlanningCandidateMetadata>(
          stages_.assemble_candidate_metadata(
              request, start, context, *search_result, *smoothing_result,
              *parameterization_result, *cable_result));
  result.artifacts.candidate_metadata = candidate_metadata;
  auto mutable_candidate = std::make_shared<PlanningResult>();
  mutable_candidate->sequence_number = candidate_metadata->sequence_number;
  mutable_candidate->timestamp = candidate_metadata->timestamp;
  mutable_candidate->validity_duration = candidate_metadata->validity_duration;
  mutable_candidate->state = PlanningState::success;
  mutable_candidate->robot_trajectory = *complete_trajectory;
  mutable_candidate->cable_path =
      cable_result->cable_prediction->touchdown_path;
  mutable_candidate->terminal_cable_state =
      derived_terminal_cable;
  mutable_candidate->cable_model_validity =
      cable_result->cable_prediction->validity;
  mutable_candidate->corridor_result = cable_result->corridor_result;
  mutable_candidate->cable_laying_result = cable_result->laying_result;
  mutable_candidate->error_budget = candidate_metadata->error_budget;
  mutable_candidate->map_version = initial_inputs.dependencies.map_version;
  mutable_candidate->reference_line_version =
      initial_inputs.dependencies.reference_line_version;
  mutable_candidate->robot_operating_area_version =
      initial_inputs.dependencies.robot_operating_area_version;
  mutable_candidate->terrain_gradient_policy_version =
      initial_inputs.dependencies.terrain_gradient_policy_version;
  mutable_candidate->corridor_risk_policy_version =
      initial_inputs.dependencies.corridor_risk_policy_version;
  mutable_candidate->cable_model_version =
      initial_inputs.dependencies.cable_model_version;
  mutable_candidate->uncertainty_envelope_version =
      initial_inputs.dependencies.uncertainty_envelope_version;
  mutable_candidate->uncertainty_envelope_generator_version =
      initial_inputs.dependencies.uncertainty_envelope_generator_version;
  mutable_candidate->execution_operating_envelope_version =
      initial_inputs.dependencies.execution_operating_envelope_version;
  mutable_candidate->execution_profile_version =
      complete_trajectory->execution_profile.version;
  mutable_candidate->sensor_mode = initial_inputs.dependencies.sensor_mode;
  mutable_candidate->operating_domain_id =
      initial_inputs.dependencies.operating_domain_id;
  mutable_candidate->cable_corridor_version =
      initial_inputs.dependencies.cable_corridor_version;
  mutable_candidate->diagnostics = candidate_metadata->diagnostics;
  mutable_candidate->diagnostics.random_seed = request.random_seed;
  mutable_candidate->diagnostics.dependencies =
      mutable_candidate->dependencies();
  const std::shared_ptr<const PlanningResult> candidate = mutable_candidate;
  const ValidationResult candidate_validation = validate(*candidate);
  const bool candidate_ok =
      candidate->state == PlanningState::success && candidate_validation.valid &&
      same_locked_dependencies(candidate->dependencies(),
                               initial_inputs.dependencies) &&
      candidate->diagnostics.dependencies == candidate->dependencies() &&
      candidate->execution_profile_version >=
          initial_inputs.dependencies.execution_profile_version &&
      candidate->robot_trajectory.execution_profile.version ==
          complete_trajectory->execution_profile.version;
  finish_metric(metric, clock_(), candidate_ok);
  result.diagnostics.stages.push_back(metric);
  result.artifacts.candidate = candidate;
  if (!candidate_ok) {
    const std::string message =
        candidate_validation.issues.empty()
            ? "candidate versions do not match the locked cycle"
            : candidate_validation.issues.front();
    return finish_failure(
        std::move(result), PlanningCycleStatus::candidate_invalid,
        PlanningState::input_invalid,
        planning_failure(PlanningFailureCause::input_invalid,
                         PlanningCycleStage::candidate_assembly,
                         "CANDIDATE_ASSEMBLY_INVALID", message),
        false);
  }

  metric = start_metric(PlanningCycleStage::decision_context_capture, clock_());
  const auto recaptured = std::make_unique<ValidationInputCaptureResult>(
      input_source_.capture(metric.started_at));
  result.replay_input_captures.push_back(*recaptured);
  bool recapture_ok = valid_capture(*recaptured);
  if (recapture_ok) {
    recapture_ok =
        recaptured->inputs->source_revision >= initial_inputs.source_revision;
    metric.source_revision = recaptured->inputs->source_revision;
    metric.dependencies = recaptured->inputs->dependencies;
  }
  finish_metric(metric, clock_(), recapture_ok);
  result.diagnostics.stages.push_back(metric);
  if (!recapture_ok) {
    return finish_failure(
        std::move(result), PlanningCycleStatus::candidate_invalidated,
        PlanningState::input_invalid,
        planning_failure(PlanningFailureCause::input_invalid,
                         PlanningCycleStage::decision_context_capture,
                         "DECISION_CONTEXT_CAPTURE_FAILED",
                         "decision context recapture failed or regressed"),
        false);
  }

  const auto latest_inputs = std::make_shared<SynchronizedValidationInputs>(
      std::move(*recaptured->inputs));
  result.decision_inputs = latest_inputs;
  result.diagnostics.decision_source_revision = latest_inputs->source_revision;
  result.diagnostics.decision_dependencies = latest_inputs->dependencies;
  if (!same_locked_dependencies(candidate->dependencies(),
                                latest_inputs->dependencies)) {
    return finish_failure(
        std::move(result), PlanningCycleStatus::candidate_invalidated,
        PlanningState::no_solution,
        planning_failure(PlanningFailureCause::no_solution,
                         PlanningCycleStage::decision_context_capture,
                         "CANDIDATE_CONTEXT_CHANGED",
                         "candidate became stale before the publication decision"),
        true);
  }

  metric = start_metric(PlanningCycleStage::candidate_revalidation, clock_(),
                        latest_inputs->source_revision,
                        latest_inputs->dependencies);
  const MonotonicTime decision_validation_at = metric.started_at;
  const auto candidate_revalidation =
      std::make_shared<PlanValidityEvaluation>(stages_.revalidate_plan(
          *candidate, PlanValidationTarget::publication_candidate,
          *latest_inputs, decision_validation_at));
  const bool candidate_revalidation_ok =
      candidate_revalidation->valid &&
      candidate_revalidation->status == PlanValidationStatus::valid &&
      candidate_revalidation->action == PlanValidationAction::reuse &&
      candidate_revalidation->lease.has_value() &&
      candidate_revalidation->remaining_path;
  finish_metric(metric, clock_(), candidate_revalidation_ok);
  result.diagnostics.stages.push_back(metric);
  result.artifacts.candidate_revalidation = candidate_revalidation;

  const std::optional<AuthorizedPlanningResult> current = publisher_.current();
  std::optional<PlanValidityEvaluation> current_revalidation;
  if (current.has_value()) {
    metric = start_metric(PlanningCycleStage::current_plan_revalidation,
                          clock_(), latest_inputs->source_revision,
                          latest_inputs->dependencies);
    current_revalidation = stages_.revalidate_plan(
        current->plan.value(), PlanValidationTarget::authorized_current,
        *latest_inputs, decision_validation_at);
    const bool current_revalidation_ok =
        current_revalidation->valid &&
        current_revalidation->status == PlanValidationStatus::valid &&
        current_revalidation->action == PlanValidationAction::reuse &&
        current_revalidation->lease.has_value() &&
        current_revalidation->remaining_path;
    finish_metric(metric, clock_(), current_revalidation_ok);
    result.diagnostics.stages.push_back(metric);
    result.artifacts.current_plan_revalidation =
        std::make_shared<PlanValidityEvaluation>(*current_revalidation);
  }

  metric = start_metric(PlanningCycleStage::candidate_decision, clock_(),
                        latest_inputs->source_revision,
                        latest_inputs->dependencies);
  const auto decision = std::make_shared<PathSwitchDecision>(
      stages_.decide_candidate(
          current_revalidation, *candidate_revalidation,
          current.has_value() ? current->path_cost : 0.0,
          candidate_metadata->path_cost, *latest_inputs, metric.started_at));
  const bool switching = decision->should_switch();
  const bool keeping = decision->should_keep_current();
  const PlanValidityEvaluation* selected_validation = nullptr;
  const PlanningResult* selected_plan = nullptr;
  if (switching) {
    selected_validation = candidate_revalidation.get();
    selected_plan = candidate.get();
  } else if (keeping && current.has_value() &&
             current_revalidation.has_value()) {
    selected_validation = &*current_revalidation;
    selected_plan = &current->plan.value();
  }
  const bool decision_ok =
      selected_validation != nullptr && selected_plan != nullptr &&
      decision->lease.has_value() && decision->remaining_path &&
      selected_validation->lease.has_value() &&
      decision->lease->lease_sequence ==
          selected_validation->lease->lease_sequence &&
      decision->lease->plan_sequence_number == selected_plan->sequence_number;
  finish_metric(metric, clock_(), decision_ok);
  result.diagnostics.stages.push_back(metric);
  result.artifacts.candidate_decision = decision;
  const bool candidate_covariance_breach =
      candidate_revalidation->status ==
      PlanValidationStatus::covariance_envelope_breach;
  const bool current_covariance_breach =
      current_revalidation.has_value() &&
      current_revalidation->status ==
          PlanValidationStatus::covariance_envelope_breach;
  if (candidate_covariance_breach || current_covariance_breach) {
    const bool candidate_breached = candidate_covariance_breach;
    return finish_failure(
        std::move(result), PlanningCycleStatus::covariance_envelope_breached,
        PlanningState::covariance_envelope_breach,
        planning_failure(
            PlanningFailureCause::covariance_envelope_breach,
            candidate_breached ? PlanningCycleStage::candidate_revalidation
                               : PlanningCycleStage::current_plan_revalidation,
            candidate_breached
                ? "CANDIDATE_REVALIDATION_COVARIANCE_ENVELOPE_BREACH"
                : "CURRENT_PLAN_COVARIANCE_ENVELOPE_BREACH",
            candidate_breached
                ? "candidate revalidation breached the covariance envelope"
                : "current plan revalidation breached the covariance envelope"),
        false);
  }
  const bool validation_inputs_invalid =
      candidate_revalidation->status == PlanValidationStatus::input_invalid ||
      (current_revalidation.has_value() &&
       current_revalidation->status == PlanValidationStatus::input_invalid);
  if (validation_inputs_invalid) {
    const bool candidate_inputs_invalid =
        candidate_revalidation->status == PlanValidationStatus::input_invalid;
    return finish_failure(
        std::move(result), PlanningCycleStatus::candidate_invalidated,
        PlanningState::input_invalid,
        planning_failure(
            PlanningFailureCause::input_invalid,
            candidate_inputs_invalid
                ? PlanningCycleStage::candidate_revalidation
                : PlanningCycleStage::current_plan_revalidation,
            candidate_inputs_invalid
                ? "CANDIDATE_REVALIDATION_INPUT_INVALID"
                : "CURRENT_PLAN_REVALIDATION_INPUT_INVALID",
            candidate_inputs_invalid
                ? "candidate revalidation inputs were invalid"
                : "current plan revalidation inputs were invalid"),
        false);
  }
  if (!decision_ok) {
    if (!candidate_revalidation_ok) {
      return finish_failure(
          std::move(result), PlanningCycleStatus::candidate_invalidated,
          PlanningState::no_solution,
          planning_failure(PlanningFailureCause::no_solution,
                           PlanningCycleStage::candidate_revalidation,
                           "CANDIDATE_REVALIDATION_FAILED",
                           "candidate failed latest-context revalidation"),
          false);
    }
    return finish_failure(
        std::move(result), PlanningCycleStatus::decision_rejected,
        PlanningState::no_solution,
        planning_failure(
            PlanningFailureCause::no_solution,
            PlanningCycleStage::candidate_decision,
            "CANDIDATE_DECISION_REJECTED",
            "latest-context decision did not select a validated plan and lease"),
        false);
  }

  metric = start_metric(PlanningCycleStage::lease_acquisition, clock_(),
                        latest_inputs->source_revision,
                        latest_inputs->dependencies);
  const MonotonicTime lease_checked_at = clock_();
  const bool lease_ok = valid_lease(*decision->lease, *selected_plan,
                                    *latest_inputs, lease_checked_at);
  finish_metric(metric, lease_checked_at, lease_ok);
  result.diagnostics.stages.push_back(metric);
  if (!lease_ok) {
    return finish_failure(
        std::move(result), PlanningCycleStatus::lease_invalid,
        PlanningState::input_invalid,
        planning_failure(
            PlanningFailureCause::input_invalid,
            PlanningCycleStage::lease_acquisition,
            "CANDIDATE_LEASE_INVALID",
            "path decision returned an invalid or mismatched lease"),
        false);
  }
  if (cycle_deadline_exceeded(result.diagnostics, lease_checked_at)) {
    return finish_failure(
        std::move(result), PlanningCycleStatus::cycle_timeout,
        PlanningState::timeout,
        cycle_timeout_failure(PlanningCycleStage::publication), true);
  }
  metric = start_metric(PlanningCycleStage::publication, clock_(),
                        latest_inputs->source_revision,
                        latest_inputs->dependencies);
  const std::optional<AuthorizedPlanningResult> previous = publisher_.current();
  if (previous) {
    lease_monitor_.revokeLease(previous->lease.lease_sequence,
                               switching ? "PLAN_REPLACED"
                                         : "PLAN_REAUTHORIZED",
                               switching
                                   ? "a newer plan and lease are being published"
                                   : "the current plan is receiving a fresh lease",
                               metric.started_at);
    result.revoked_lease_sequence = previous->lease.lease_sequence;
  }
  AuthorizedPlanningPublication publication = switching
      ? publisher_.publish(*candidate, decision->remaining_path,
                           *decision->lease, candidate_metadata->path_cost)
      : publisher_.reauthorize_current(decision->remaining_path,
                                       *decision->lease);
  const MonotonicTime published_at = clock_();
  finish_metric(metric, published_at, publication.published());
  result.diagnostics.stages.push_back(metric);
  if (!publication.published()) {
    const std::string message = publication.issues.empty()
                                    ? "immutable publication failed"
                                    : publication.issues.front();
    return finish_failure(
        std::move(result), PlanningCycleStatus::publication_failed,
        PlanningState::input_invalid,
        planning_failure(PlanningFailureCause::input_invalid,
                         PlanningCycleStage::publication,
                         "CANDIDATE_PUBLICATION_FAILED", message),
        false);
  }
  if (cycle_deadline_exceeded(result.diagnostics, published_at)) {
    lease_monitor_.revokeLease(
        publication.value->lease.lease_sequence,
        "PLANNING_CYCLE_DEADLINE_EXCEEDED",
        "planning cycle exceeded its total wall-clock deadline",
        published_at);
    result.revoked_lease_sequence = publication.value->lease.lease_sequence;
    publisher_.revoke_current();
    result.status = PlanningCycleStatus::cycle_timeout;
    result.state = PlanningState::timeout;
    result.root_cause = cycle_timeout_failure(PlanningCycleStage::publication);
    result.issues = {result.root_cause->message};
    stages_.request_controlled_stop(*result.root_cause, published_at);
    result.controlled_stop_required = true;
    return result;
  }

  result.status = switching ? PlanningCycleStatus::success
                            : PlanningCycleStatus::current_plan_reused;
  result.state = switching ? PlanningState::success
                           : PlanningState::path_valid;
  result.publication = publication.value;
  return result;
}

}  // namespace underwater_planner::core
