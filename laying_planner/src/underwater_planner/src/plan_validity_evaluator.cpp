#include "underwater_planner/core/plan_validity_evaluator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace underwater_planner::core {
namespace {

constexpr double kEpsilon = 1.0e-9;

void issue(PlanValidityEvaluation& result, const char* code,
           const char* message, const MonotonicTime now) {
  result.issues.emplace_back(message);
  result.diagnostics.entries.push_back(
      {DiagnosticSeverity::error, code, "plan_validity", message, now});
}

bool finite_nonnegative(const double value) {
  return std::isfinite(value) && value >= 0.0;
}

bool age_valid(const MonotonicTime timestamp, const Duration maximum,
               const MonotonicTime now) {
  return timestamp.nanoseconds >= 0 && now.nanoseconds >= timestamp.nanoseconds &&
         maximum.nanoseconds >= 0 &&
         now.nanoseconds - timestamp.nanoseconds <= maximum.nanoseconds;
}

bool synchronized_provenance_valid(
    const SynchronizedValidationInputs& inputs,
    const Duration synchronization_tolerance, const MonotonicTime now) {
  const TrackerUpdateReceipt& receipt = inputs.tracker_update_receipt;
  if (inputs.captured_at.nanoseconds < 0 ||
      inputs.captured_at.nanoseconds > now.nanoseconds ||
      inputs.source_revision == 0U ||
      inputs.cable_context_mode != PredictionMode::validation ||
      receipt.evidence_batch_sequence == 0U ||
      receipt.executed_motion_sequence == 0U ||
      receipt.cable_telemetry_sequence != inputs.cable_telemetry.sequence_number ||
      receipt.resulting_cable_state_sequence != inputs.cable_state.sequence_number ||
      receipt.resulting_reference_progress_sequence !=
          inputs.reference_progress.sequence_number ||
      (receipt.touchdown_observation_sequence.has_value() &&
       *receipt.touchdown_observation_sequence == 0U)) {
    return false;
  }
  const std::array<std::int64_t, 6> timestamps{
      inputs.robot_state.pose.timestamp.nanoseconds,
      inputs.robot_state.curvature_timestamp.nanoseconds,
      inputs.cable_state.timestamp.nanoseconds,
      inputs.reference_progress.timestamp.nanoseconds,
      inputs.cable_telemetry.timestamp.nanoseconds,
      inputs.execution_tracking_state.timestamp.nanoseconds};
  const auto bounds = std::minmax_element(timestamps.begin(), timestamps.end());
  return *bounds.first >= 0 &&
         *bounds.second - *bounds.first <= synchronization_tolerance.nanoseconds;
}

bool same_versions(const PlanningResult& plan,
                   const SynchronizedValidationInputs& inputs,
                   const PlanValidationTarget target) {
  const PlanningDependencyVersions& current = inputs.dependencies;
  return plan.map_version == current.map_version &&
         plan.reference_line_version == current.reference_line_version &&
         plan.robot_operating_area_version == current.robot_operating_area_version &&
         plan.terrain_gradient_policy_version ==
             current.terrain_gradient_policy_version &&
         plan.corridor_risk_policy_version == current.corridor_risk_policy_version &&
         plan.cable_model_version == current.cable_model_version &&
         plan.uncertainty_envelope_version == current.uncertainty_envelope_version &&
         plan.uncertainty_envelope_generator_version ==
             current.uncertainty_envelope_generator_version &&
         plan.execution_operating_envelope_version ==
             current.execution_operating_envelope_version &&
         (target == PlanValidationTarget::publication_candidate
              ? plan.execution_profile_version >=
                    current.execution_profile_version
              : plan.execution_profile_version ==
                    current.execution_profile_version) &&
         plan.sensor_mode == current.sensor_mode &&
         plan.operating_domain_id == current.operating_domain_id &&
         plan.cable_corridor_version == current.cable_corridor_version;
}

bool context_versions_match(const PlanValidityContext& context,
                            const SynchronizedValidationInputs& inputs,
                            const CableModelIdentity& cable_model,
                            const PlanningResult& plan) {
  if (!context.locked_envelope.has_value() ||
      !context.locked_envelope->envelope) {
    return false;
  }
  const PlanningDependencyVersions& expected = inputs.dependencies;
  const EnvelopeDependencies& envelope =
      context.locked_envelope->envelope->dependencies;
  return context.corridor_policy.version ==
             expected.corridor_risk_policy_version &&
         context.corridor_policy.operating_domain_id ==
             expected.operating_domain_id &&
         !context.reference_progress_parameters.parameter_profile_id().empty() &&
         context.reference_progress_parameters.operating_domain_id() ==
             expected.operating_domain_id &&
         plan.error_budget.epsilon_point.has_value() &&
         context.corridor_policy.epsilon_point ==
             *plan.error_budget.epsilon_point &&
         context.path_context.terrain_gradient_risk_policy.version ==
             expected.terrain_gradient_policy_version &&
         context.path_context.terrain_gradient_risk_policy.operating_domain_id ==
             expected.operating_domain_id &&
         plan.error_budget.epsilon_terrain_gradient_local.has_value() &&
         context.path_context.terrain_gradient_risk_policy.epsilon_local ==
             *plan.error_budget.epsilon_terrain_gradient_local &&
         plan.error_budget.epsilon_robot.has_value() &&
         context.path_context.collision_risk_policy.epsilon_robot ==
             *plan.error_budget.epsilon_robot &&
         context.terrain.source_map_version == expected.map_version &&
         context.terrain.analysis_config_version ==
             inputs.planning_snapshot.map.derived_configuration_version &&
         context.terrain.operating_domain_id == expected.operating_domain_id &&
         cable_model.version == expected.cable_model_version &&
         cable_model.operating_domain_id == expected.operating_domain_id &&
         context.cable_context.execution_envelope.version ==
             expected.execution_operating_envelope_version &&
         context.cable_context.execution_envelope.operating_domain_id ==
             expected.operating_domain_id &&
         context.cable_context.sensor_mode == expected.sensor_mode &&
         context.cable_context.mode == PredictionMode::validation &&
         context.cable_context.uncertainty_envelope_version ==
             expected.uncertainty_envelope_version &&
         context.cable_context.uncertainty_envelope_generator_version ==
             expected.uncertainty_envelope_generator_version &&
         context.locked_envelope->envelope_version ==
             expected.uncertainty_envelope_version &&
         envelope.generator_version ==
             expected.uncertainty_envelope_generator_version &&
         envelope.cable_model_version == expected.cable_model_version &&
         envelope.execution_operating_envelope_version ==
             expected.execution_operating_envelope_version &&
         envelope.reference_line_version == expected.reference_line_version &&
         envelope.sensor_mode == expected.sensor_mode &&
         envelope.operating_domain_id == expected.operating_domain_id;
}

double wrap_delta(const double angle) {
  return normalize_angle_radians(angle);
}

struct InterpolatedPathSample {
  PathPoint geometry;
  ExecutionSample execution;
};

std::optional<InterpolatedPathSample> interpolate_at(const TimedPath& path,
                                                     const double arc) {
  const auto& points = path.geometry.points;
  const auto& samples = path.execution_profile.samples;
  if (points.size() < 2U || samples.size() < 2U ||
      arc < points.front().arc_length_m - kEpsilon ||
      arc > points.back().arc_length_m + kEpsilon) {
    return std::nullopt;
  }
  const auto find_interval = [arc](const auto& values) {
    std::size_t right = 1U;
    while (right < values.size() && values[right].arc_length_m < arc) ++right;
    return right;
  };
  const std::size_t pi = find_interval(points);
  const std::size_t ei = find_interval(samples);
  if (pi >= points.size() || ei >= samples.size()) return std::nullopt;
  const PathPoint& p0 = points[pi - 1U];
  const PathPoint& p1 = points[pi];
  const ExecutionSample& e0 = samples[ei - 1U];
  const ExecutionSample& e1 = samples[ei];
  const double ps = p1.arc_length_m - p0.arc_length_m;
  const double es = e1.arc_length_m - e0.arc_length_m;
  if (!(ps > 0.0) || !(es > 0.0)) return std::nullopt;
  const double pr = std::clamp((arc - p0.arc_length_m) / ps, 0.0, 1.0);
  const double er = std::clamp((arc - e0.arc_length_m) / es, 0.0, 1.0);
  InterpolatedPathSample out;
  out.geometry = {arc,
                  p0.x_m + pr * (p1.x_m - p0.x_m),
                  p0.y_m + pr * (p1.y_m - p0.y_m),
                  normalize_angle_radians(p0.heading_rad +
                                          pr * wrap_delta(p1.heading_rad - p0.heading_rad)),
                  p0.curvature_per_m + pr *
                      (p1.curvature_per_m - p0.curvature_per_m)};
  out.execution = {arc,
                   Duration{static_cast<std::int64_t>(
                       std::llround(e0.time_from_start.nanoseconds +
                                    er * static_cast<double>(e1.time_from_start.nanoseconds -
                                                                e0.time_from_start.nanoseconds)))},
                   e0.ground_speed_mps + er * (e1.ground_speed_mps - e0.ground_speed_mps),
                   e0.ground_acceleration_mps2 +
                       er * (e1.ground_acceleration_mps2 - e0.ground_acceleration_mps2),
                   e0.payout_speed_mps + er * (e1.payout_speed_mps - e0.payout_speed_mps),
                   e0.payout_acceleration_mps2 +
                       er * (e1.payout_acceleration_mps2 - e0.payout_acceleration_mps2),
                   e0.tension_setpoint_n +
                       er * (e1.tension_setpoint_n - e0.tension_setpoint_n)};
  return out;
}

std::optional<TimedPath> crop_path(const TimedPath& original, const double start_s) {
  const auto first = interpolate_at(original, start_s);
  if (!first.has_value()) return std::nullopt;
  TimedPath cropped = original;
  cropped.geometry.points.clear();
  cropped.execution_profile.samples.clear();
  cropped.geometry.points.push_back(first->geometry);
  cropped.execution_profile.samples.push_back(first->execution);
  for (const PathPoint& point : original.geometry.points) {
    if (point.arc_length_m > start_s + kEpsilon) cropped.geometry.points.push_back(point);
  }
  for (const ExecutionSample& sample : original.execution_profile.samples) {
    if (sample.arc_length_m > start_s + kEpsilon) cropped.execution_profile.samples.push_back(sample);
  }
  if (cropped.geometry.points.size() < 2U || cropped.execution_profile.samples.size() < 2U)
    return std::nullopt;
  const std::int64_t origin = cropped.execution_profile.samples.front().time_from_start.nanoseconds;
  for (ExecutionSample& sample : cropped.execution_profile.samples) {
    sample.time_from_start.nanoseconds -= origin;
  }
  cropped.execution_profile.stopping_point_arc_length_m =
      original.execution_profile.stopping_point_arc_length_m;
  cropped.geometry.metadata.path_version = original.geometry.metadata.path_version;
  return cropped;
}

std::optional<std::vector<RobotUncertaintySample>> crop_uncertainty(
    const std::vector<RobotUncertaintySample>& original, const double start_s,
    const double end_s) {
  if (original.size() < 2U || start_s < original.front().arc_length_m - kEpsilon ||
      end_s > original.back().arc_length_m + kEpsilon) {
    return std::nullopt;
  }
  std::size_t right = 1U;
  while (right < original.size() && original[right].arc_length_m < start_s) ++right;
  if (right >= original.size()) return std::nullopt;
  const RobotUncertaintySample& left = original[right - 1U];
  const RobotUncertaintySample& upper = original[right];
  const double span = upper.arc_length_m - left.arc_length_m;
  if (!(span > 0.0)) return std::nullopt;
  const double ratio = std::clamp((start_s - left.arc_length_m) / span, 0.0, 1.0);
  const auto lerp = [ratio](const double a, const double b) {
    return a + ratio * (b - a);
  };
  RobotUncertaintySample first = left;
  first.arc_length_m = start_s;
  auto& p = first.pose_tracking_covariance;
  const auto& lp = left.pose_tracking_covariance;
  const auto& up = upper.pose_tracking_covariance;
  p.position_covariance_m2 = {
      lerp(lp.position_covariance_m2.xx_m2, up.position_covariance_m2.xx_m2),
      lerp(lp.position_covariance_m2.xy_m2, up.position_covariance_m2.xy_m2),
      lerp(lp.position_covariance_m2.yx_m2, up.position_covariance_m2.yx_m2),
      lerp(lp.position_covariance_m2.yy_m2, up.position_covariance_m2.yy_m2)};
  p.x_heading_covariance_m_rad =
      lerp(lp.x_heading_covariance_m_rad, up.x_heading_covariance_m_rad);
  p.y_heading_covariance_m_rad =
      lerp(lp.y_heading_covariance_m_rad, up.y_heading_covariance_m_rad);
  p.heading_variance_rad2 =
      lerp(lp.heading_variance_rad2, up.heading_variance_rad2);
  first.heading_tracking_process_variance_per_m_rad2 = lerp(
      left.heading_tracking_process_variance_per_m_rad2,
      upper.heading_tracking_process_variance_per_m_rad2);
  auto& c = first.cross_covariance;
  const auto& lc = left.cross_covariance;
  const auto& uc = upper.cross_covariance;
  c.robot_x_initial_lag_m_rad = lerp(lc.robot_x_initial_lag_m_rad, uc.robot_x_initial_lag_m_rad);
  c.robot_y_initial_lag_m_rad = lerp(lc.robot_y_initial_lag_m_rad, uc.robot_y_initial_lag_m_rad);
  c.robot_heading_initial_lag_rad2 = lerp(lc.robot_heading_initial_lag_rad2, uc.robot_heading_initial_lag_rad2);
  c.robot_x_touchdown_distance_m2 = lerp(lc.robot_x_touchdown_distance_m2, uc.robot_x_touchdown_distance_m2);
  c.robot_y_touchdown_distance_m2 = lerp(lc.robot_y_touchdown_distance_m2, uc.robot_y_touchdown_distance_m2);
  c.robot_heading_touchdown_distance_m_rad = lerp(lc.robot_heading_touchdown_distance_m_rad, uc.robot_heading_touchdown_distance_m_rad);
  c.robot_x_direction_response_length_m2 = lerp(lc.robot_x_direction_response_length_m2, uc.robot_x_direction_response_length_m2);
  c.robot_y_direction_response_length_m2 = lerp(lc.robot_y_direction_response_length_m2, uc.robot_y_direction_response_length_m2);
  c.robot_heading_direction_response_length_m_rad = lerp(lc.robot_heading_direction_response_length_m_rad, uc.robot_heading_direction_response_length_m_rad);
  c.initial_lag_touchdown_distance_m_rad = lerp(lc.initial_lag_touchdown_distance_m_rad, uc.initial_lag_touchdown_distance_m_rad);
  c.initial_lag_direction_response_length_m_rad = lerp(lc.initial_lag_direction_response_length_m_rad, uc.initial_lag_direction_response_length_m_rad);
  c.touchdown_distance_direction_response_length_m2 = lerp(lc.touchdown_distance_direction_response_length_m2, uc.touchdown_distance_direction_response_length_m2);
  std::vector<RobotUncertaintySample> cropped{first};
  for (const RobotUncertaintySample& sample : original) {
    if (sample.arc_length_m > start_s + kEpsilon &&
        sample.arc_length_m <= end_s + kEpsilon) {
      cropped.push_back(sample);
    }
  }
  return cropped.size() >= 2U
             ? std::optional<std::vector<RobotUncertaintySample>>{cropped}
             : std::nullopt;
}

std::optional<std::vector<double>> associate_reference_progress(
    const CablePrediction& prediction, const ReferenceProgress& initial_progress,
    const ReferenceLine& reference,
    const ReferenceProgressAssociationParameters& parameters,
    const MonotonicTime now) {
  if (prediction.touchdown_path.points.size() < 2U ||
      prediction.touchdown_path.points.size() !=
          prediction.robot_arc_length_profile_m.size()) {
    return std::nullopt;
  }
  ReferenceProgressAssociator associator(parameters);
  ReferenceProgress progress = initial_progress;
  std::vector<double> associated{progress.arc_length_m};
  for (std::size_t index = 1U;
       index < prediction.touchdown_path.points.size(); ++index) {
    const PathPoint& touchdown = prediction.touchdown_path.points[index];
    const double primitive_length_m =
        prediction.robot_arc_length_profile_m[index] -
        prediction.robot_arc_length_profile_m[index - 1U];
    const TouchdownAssociationSample sample{
        touchdown.arc_length_m, {touchdown.x_m, touchdown.y_m},
        touchdown.heading_rad, now};
    const ReferenceAssociationResult result = associator.propagate_candidate(
        progress, sample, primitive_length_m, reference);
    if (result.status != ReferenceAssociationStatus::tracked ||
        !result.context.has_value()) {
      return std::nullopt;
    }
    progress = result.context->progress;
    associated.push_back(progress.arc_length_m);
  }
  return associated;
}

bool state_matches(const PlanningResult& plan, const SynchronizedValidationInputs& inputs,
                   const PlanValidityEvaluatorConfig& config, const double start_s) {
  const auto expected = interpolate_at(plan.robot_trajectory, start_s);
  if (!expected.has_value()) return false;
  const RobotState& state = inputs.robot_state;
  return std::hypot(expected->geometry.x_m - state.pose.x_m,
                   expected->geometry.y_m - state.pose.y_m) <= config.position_tolerance_m &&
         std::abs(wrap_delta(expected->geometry.heading_rad - state.pose.heading_rad)) <=
             config.heading_tolerance_rad &&
         std::abs(expected->geometry.curvature_per_m - state.curvature_per_m) <=
             config.curvature_tolerance_per_m;
}

MonotonicTime minimum_time(const MonotonicTime left, const MonotonicTime right) {
  return left.nanoseconds < right.nanoseconds ? left : right;
}

}  // namespace

PlanValidityEvaluator::PlanValidityEvaluator(CableModel cable_model,
                                             PlanValidityEvaluatorConfig config)
    : cable_model_(std::move(cable_model)), config_(config) {
  if (config_.version == 0U || config_.parameter_profile_id.empty() ||
      config_.operating_domain_id.empty() ||
      config_.maximum_reuse_duration.nanoseconds <= 0 ||
      config_.input_limits.robot_state_max_age.nanoseconds < 0 ||
      config_.input_limits.cable_state_max_age.nanoseconds < 0 ||
      config_.input_limits.reference_progress_max_age.nanoseconds < 0 ||
      config_.input_limits.cable_telemetry_max_age.nanoseconds < 0 ||
      config_.input_limits.execution_tracking_max_age.nanoseconds < 0 ||
      config_.input_limits.map_max_age.nanoseconds < 0 ||
      config_.input_limits.synchronization_tolerance.nanoseconds < 0 ||
      config_.envelope_validity_margin.nanoseconds < 0 ||
      !finite_nonnegative(config_.position_tolerance_m) ||
      !finite_nonnegative(config_.heading_tolerance_rad) ||
      !finite_nonnegative(config_.curvature_tolerance_per_m) ||
      !finite_nonnegative(config_.maximum_ground_speed_tracking_error_mps) ||
      !finite_nonnegative(
          config_.maximum_ground_acceleration_tracking_error_mps2) ||
      config_.last_issued_lease_sequence >=
          std::numeric_limits<std::uint64_t>::max() - 1U ||
      !finite_nonnegative(config_.stopping_safety_margin_m)) {
    throw std::invalid_argument("plan validity configuration is invalid");
  }
  next_lease_sequence_ = config_.last_issued_lease_sequence + 1U;
}

PlanValidityEvaluation PlanValidityEvaluator::validateRemainingPlan(
    const ImmutablePlanningResult& immutable_plan,
    const SynchronizedValidationInputs& inputs,
    const PlanValidityContext& context, const MonotonicTime now) {
  return validatePlan(immutable_plan, inputs, context, now,
                      PlanValidationTarget::authorized_current);
}

PlanValidityEvaluation PlanValidityEvaluator::validatePublicationCandidate(
    const ImmutablePlanningResult& immutable_plan,
    const SynchronizedValidationInputs& inputs,
    const PlanValidityContext& context, const MonotonicTime now) {
  return validatePlan(immutable_plan, inputs, context, now,
                      PlanValidationTarget::publication_candidate);
}

PlanValidityEvaluation PlanValidityEvaluator::validatePlan(
    const ImmutablePlanningResult& immutable_plan,
    const SynchronizedValidationInputs& inputs,
    const PlanValidityContext& context, const MonotonicTime now,
    const PlanValidationTarget target) {
  const PlanningResult& plan = immutable_plan.value();
  PlanValidityEvaluation result;
  result.evaluator_config_version = config_.version;
  result.parameter_profile_id = config_.parameter_profile_id;
  result.diagnostics.schema_version = "plan-validity-diagnostics/v1";
  result.diagnostics.input_version =
      "synchronized-validation-inputs/v1;parameter-profile=" +
      config_.parameter_profile_id + ";evaluator-config=" +
      std::to_string(config_.version);
  result.diagnostics.unit_system =
      "SI[length=m,angle=rad,time=s,curvature=1/m,speed=m/s,tension=N]";
  result.diagnostics.operating_domain_id = inputs.dependencies.operating_domain_id;
  result.diagnostics.risk_semantics =
      "POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE";
  result.diagnostics.dependencies = inputs.dependencies;
  result.action = PlanValidationAction::stop;
  if (now.nanoseconds < 0 || plan.sequence_number == 0U ||
      !validate(plan).valid || !validate(inputs.robot_state).valid ||
      !validate(inputs.cable_state).valid || !validate(inputs.reference_progress).valid ||
      !validate(plan.robot_trajectory).valid ||
      inputs.execution_tracking_state.execution_profile_version == 0U ||
      !std::isfinite(
          inputs.execution_tracking_state.ground_acceleration_mps2)) {
    issue(result, "PLAN_VALIDITY_INPUT_INVALID", "validation inputs or plan are invalid", now);
    return result;
  }
  if (!synchronized_provenance_valid(
          inputs, config_.input_limits.synchronization_tolerance, now)) {
    result.status = PlanValidationStatus::input_invalid;
    result.action = PlanValidationAction::stop;
    issue(result, "PLAN_VALIDITY_SYNCHRONIZATION_INVALID",
          "validation inputs lack one atomic synchronized capture provenance",
          now);
    return result;
  }
  if (!age_valid(inputs.robot_state.pose.timestamp,
                 config_.input_limits.robot_state_max_age, now) ||
      !age_valid(inputs.robot_state.curvature_timestamp,
                 config_.input_limits.robot_state_max_age, now) ||
      !age_valid(inputs.cable_state.timestamp,
                 config_.input_limits.cable_state_max_age, now) ||
      !age_valid(inputs.reference_progress.timestamp,
                 config_.input_limits.reference_progress_max_age, now) ||
      !age_valid(inputs.cable_telemetry.timestamp,
                 config_.input_limits.cable_telemetry_max_age, now) ||
      !age_valid(inputs.execution_tracking_state.timestamp,
                 config_.input_limits.execution_tracking_max_age, now) ||
      !age_valid(inputs.planning_snapshot.map.version.timestamp,
                 config_.input_limits.map_max_age, now)) {
    result.status = PlanValidationStatus::input_expired;
    result.action = PlanValidationAction::stop;
    issue(result, "PLAN_VALIDITY_INPUT_EXPIRED", "a synchronized validation input is too old", now);
    return result;
  }
  const double start_s =
      target == PlanValidationTarget::publication_candidate
          ? plan.robot_trajectory.geometry.points.front().arc_length_m
          : inputs.execution_tracking_state.tracked_arc_length_m;
  if (!finite_nonnegative(start_s) || start_s + kEpsilon < plan.robot_trajectory.geometry.points.front().arc_length_m ||
      start_s > plan.robot_trajectory.geometry.points.back().arc_length_m + kEpsilon ||
      !state_matches(plan, inputs, config_, start_s)) {
    result.status = PlanValidationStatus::state_mismatch;
    result.action = PlanValidationAction::replan;
    issue(result, "PLAN_VALIDITY_STATE_MISMATCH", "current robot state is not the confirmed plan prefix", now);
    return result;
  }
  if (inputs.execution_tracking_state.execution_profile_version !=
          inputs.dependencies.execution_profile_version ||
      inputs.execution_tracking_state.execution_operating_envelope_version !=
          inputs.dependencies.execution_operating_envelope_version ||
      inputs.execution_tracking_state.execution_operating_envelope_version !=
          plan.execution_operating_envelope_version ||
      (target == PlanValidationTarget::authorized_current &&
       inputs.execution_tracking_state.execution_profile_version !=
           plan.execution_profile_version)) {
    result.status = PlanValidationStatus::state_mismatch;
    result.action = PlanValidationAction::replan;
    issue(result, "PLAN_VALIDITY_EXECUTION_PROFILE_MISMATCH",
          "tracked execution profile is not the plan profile", now);
    return result;
  }
  if (!same_versions(plan, inputs, target) ||
      config_.operating_domain_id != inputs.dependencies.operating_domain_id ||
      config_.parameter_profile_id !=
          context.reference_progress_parameters.parameter_profile_id() ||
      inputs.planning_snapshot.map.version != inputs.dependencies.map_version ||
      inputs.planning_snapshot.reference_line.version != plan.reference_line_version ||
      inputs.planning_snapshot.robot_operating_area.version != plan.robot_operating_area_version ||
      inputs.planning_snapshot.cable_corridor.version !=
          plan.cable_corridor_version ||
      inputs.reference_progress.reference_line_version != plan.reference_line_version ||
      !context_versions_match(context, inputs, cable_model_.identity(), plan)) {
    result.status = PlanValidationStatus::context_mismatch;
    result.action = PlanValidationAction::replan;
    issue(result, "PLAN_VALIDITY_CONTEXT_MISMATCH", "plan dependencies do not match the current synchronized context", now);
    return result;
  }
  const auto remaining = crop_path(plan.robot_trajectory, start_s);
  if (!remaining.has_value()) {
    result.status = PlanValidationStatus::execution_profile_invalid;
    result.action = PlanValidationAction::stop;
    issue(result, "PLAN_VALIDITY_REMAINING_PROFILE_INVALID", "the remaining profile cannot be cropped without changing its version", now);
    return result;
  }
  const auto current_sample = interpolate_at(plan.robot_trajectory, start_s);
  if (!current_sample.has_value() ||
      !finite_nonnegative(config_.maximum_ground_speed_tracking_error_mps) ||
      std::abs(inputs.robot_state.ground_speed_mps -
               current_sample->execution.ground_speed_mps) >
          config_.maximum_ground_speed_tracking_error_mps + kEpsilon ||
      std::abs(inputs.execution_tracking_state.ground_acceleration_mps2 -
               current_sample->execution.ground_acceleration_mps2) >
          config_.maximum_ground_acceleration_tracking_error_mps2 + kEpsilon ||
      std::abs(inputs.cable_telemetry.payout_speed_mps -
               current_sample->execution.payout_speed_mps) >
          remaining->execution_profile.approved_tracking_limits
                  .maximum_payout_tracking_error_mps +
              kEpsilon ||
      std::abs(inputs.cable_telemetry.payout_acceleration_mps2 -
               current_sample->execution.payout_acceleration_mps2) >
          context.cable_context.execution_envelope
                  .maximum_payout_acceleration_tracking_error_mps2 +
              kEpsilon ||
      std::abs(inputs.cable_telemetry.tension_n -
               current_sample->execution.tension_setpoint_n) >
          context.cable_context.execution_envelope
                  .maximum_tension_tracking_error_n +
              kEpsilon) {
    result.status = PlanValidationStatus::execution_profile_mismatch;
    result.action = PlanValidationAction::replan;
    issue(result, "PLAN_VALIDITY_EXECUTION_PROFILE_DISCONTINUITY",
          "current execution state is outside the approved profile tracking errors",
          now);
    return result;
  }
  const ValidationResult profile_validation = validate(*remaining);
  if (!profile_validation.valid) {
    result.status = PlanValidationStatus::execution_profile_invalid;
    result.action = PlanValidationAction::replan;
    result.issues = profile_validation.issues;
    issue(result, "PLAN_VALIDITY_PROFILE_INVALID",
          "cropped execution profile violates its approved limits", now);
    return result;
  }
  const double stopping_distance = remaining->execution_profile
                                       .approved_tracking_limits
                                       .maximum_stopping_distance_m;
  const double distance_to_approved_stop =
      *remaining->execution_profile.stopping_point_arc_length_m - start_s;
  if (!finite_nonnegative(stopping_distance) ||
      distance_to_approved_stop + kEpsilon <
          stopping_distance + config_.stopping_safety_margin_m) {
    result.status = PlanValidationStatus::stopping_distance_insufficient;
    result.action = PlanValidationAction::stop;
    issue(result, "PLAN_VALIDITY_STOPPING_DISTANCE",
          "remaining path does not cover the worst-case stopping distance", now);
    return result;
  }
  PathCandidateVerifier verifier;
  const PathBoundary start_boundary{inputs.robot_state.pose.x_m,
                                    inputs.robot_state.pose.y_m,
                                    inputs.robot_state.pose.heading_rad,
                                    inputs.robot_state.curvature_per_m,
                                    PathBoundarySource::synchronized_actual_state,
                                    inputs.robot_state.pose.timestamp,
                                    inputs.robot_state.curvature_timestamp,
                                    inputs.robot_state.sequence_number};
  PathCandidateVerificationContext current_path_context = context.path_context;
  current_path_context.map = inputs.planning_snapshot.map;
  current_path_context.terrain = context.terrain;
  current_path_context.robot_operating_area =
      inputs.planning_snapshot.robot_operating_area;
  const auto path_result = verifier.verify(
      remaining->geometry, start_boundary, context.goal_boundary,
      context.smoothing_limits, current_path_context);
  if (!path_result.valid) {
    result.status = PlanValidationStatus::robot_constraint_violation;
    result.action = PlanValidationAction::replan;
    issue(result, "PLAN_VALIDITY_ROBOT_CONSTRAINT", "remaining geometry failed the complete robot hard-constraint audit", now);
    return result;
  }
  if (!context.locked_envelope.has_value() || context.envelope_manager == nullptr) {
    result.status = PlanValidationStatus::covariance_envelope_unavailable;
    result.action = PlanValidationAction::replan;
    issue(result, "PLAN_VALIDITY_CABLE_CONTEXT_INVALID", "a current locked uncertainty envelope is required for reuse", now);
    return result;
  }
  CableContext cable_context = context.cable_context;
  cable_context.current_telemetry = inputs.cable_telemetry;
  cable_context.mode = PredictionMode::validation;
  const auto remaining_uncertainty = crop_uncertainty(
      cable_context.robot_uncertainty_profile, start_s,
      remaining->geometry.points.back().arc_length_m);
  if (!remaining_uncertainty.has_value()) {
    result.status = PlanValidationStatus::cable_model_invalid;
    result.action = PlanValidationAction::replan;
    issue(result, "PLAN_VALIDITY_UNCERTAINTY_PROFILE",
          "robot uncertainty profile does not cover the remaining trajectory", now);
    return result;
  }
  cable_context.robot_uncertainty_profile = *remaining_uncertainty;
  const CablePrediction seed_prediction = cable_model_.predict(
      inputs.cable_state, *remaining, cable_context);
  if (seed_prediction.validity != CableModelValidity::valid) {
    result.status = PlanValidationStatus::cable_model_invalid;
    result.action = PlanValidationAction::replan;
    issue(result, "PLAN_VALIDITY_CABLE_PREDICTION", "current cable state cannot produce a valid remaining prediction", now);
    result.issues.insert(result.issues.end(), seed_prediction.issues.begin(),
                         seed_prediction.issues.end());
    return result;
  }
  TimedCableCandidateInput cable_input;
  cable_input.initial_cable_state = inputs.cable_state;
  cable_input.robot_path = *remaining;
  cable_input.cable_context = cable_context;
  cable_input.reference_line = inputs.planning_snapshot.reference_line;
  cable_input.corridor_policy = context.corridor_policy;
  if (context.corridor_interval_bound.certificate_version == 0U ||
      !finite_nonnegative(context.corridor_interval_bound.upper_bound_error_m) ||
      seed_prediction.robot_arc_length_profile_m.size() < 2U) {
    result.status = PlanValidationStatus::cable_corridor_invalid;
    result.action = PlanValidationAction::replan;
    issue(result, "PLAN_VALIDITY_CORRIDOR_CERTIFICATE",
          "current corridor interval-bound certification is invalid", now);
    return result;
  }
  cable_input.interval_bound_certificate = {
      context.corridor_interval_bound.certificate_version,
      std::vector<double>(seed_prediction.robot_arc_length_profile_m.size() - 1U,
                          context.corridor_interval_bound.upper_bound_error_m)};
  const auto associated_progress = associate_reference_progress(
      seed_prediction, inputs.reference_progress,
      inputs.planning_snapshot.reference_line,
      context.reference_progress_parameters, now);
  if (!associated_progress.has_value()) {
    result.status = PlanValidationStatus::cable_corridor_invalid;
    result.action = PlanValidationAction::replan;
    issue(result, "PLAN_VALIDITY_REFERENCE_ASSOCIATION",
          "remaining touchdown path cannot be associated with current task progress",
          now);
    return result;
  }
  cable_input.reference_progress_m = *associated_progress;
  cable_input.terrain = context.terrain;
  cable_input.laying_limits = context.laying_limits;
  cable_input.history_boundary = context.history_boundary;
  cable_input.reference_is_deterministic = context.reference_is_deterministic;
  cable_input.covariance_includes_coordinate_transform_error =
      context.covariance_includes_coordinate_transform_error;
  cable_input.envelope_audit_tolerance_m = context.envelope_audit_tolerance_m;
  cable_input.evaluation_timestamp = now;
  cable_input.locked_envelope = context.locked_envelope;
  TimedCableCandidateVerifier cable_verifier(cable_model_, context.envelope_manager);
  const TimedCableCandidateResult cable_result = cable_verifier.validate(cable_input);
  if (!cable_result.valid) {
    switch (cable_result.status) {
      case TimedCableValidationStatus::cable_model_invalid:
        result.status = PlanValidationStatus::cable_model_invalid;
        break;
      case TimedCableValidationStatus::covariance_envelope_unavailable:
        result.status = PlanValidationStatus::covariance_envelope_unavailable;
        break;
      case TimedCableValidationStatus::covariance_envelope_breach:
        result.status = PlanValidationStatus::covariance_envelope_breach;
        break;
      case TimedCableValidationStatus::corridor_invalid:
      case TimedCableValidationStatus::input_invalid:
        result.status = PlanValidationStatus::cable_corridor_invalid;
        break;
      case TimedCableValidationStatus::corridor_violation:
        result.status = PlanValidationStatus::cable_corridor_violation;
        break;
      case TimedCableValidationStatus::laying_invalid:
        result.status = PlanValidationStatus::cable_laying_invalid;
        break;
      case TimedCableValidationStatus::valid:
        result.status = PlanValidationStatus::input_invalid;
        break;
    }
    result.action = cable_result.stop_required ? PlanValidationAction::stop
                                                : PlanValidationAction::replan;
    result.cable_prediction = cable_result.cable_prediction;
    result.issues = cable_result.issues;
    result.diagnostics.entries.insert(result.diagnostics.entries.end(),
                                      cable_result.diagnostics.begin(),
                                      cable_result.diagnostics.end());
    issue(result, "PLAN_VALIDITY_CABLE_CONSTRAINT", "remaining cable corridor or mechanical hard constraints failed", now);
    return result;
  }
  result.cable_prediction = cable_result.cable_prediction;
  result.diagnostics.entries.insert(result.diagnostics.entries.end(),
                                    cable_result.diagnostics.begin(),
                                    cable_result.diagnostics.end());
  result.status = PlanValidationStatus::valid;
  result.action = PlanValidationAction::reuse;
  result.valid = true;
  const auto add_duration = [](const MonotonicTime time,
                               const Duration duration) {
    if (time.nanoseconds >
        std::numeric_limits<std::int64_t>::max() - duration.nanoseconds) {
      return MonotonicTime{std::numeric_limits<std::int64_t>::max()};
    }
    return MonotonicTime{time.nanoseconds + duration.nanoseconds};
  };
  const MonotonicTime reuse_expiry =
      add_duration(now, config_.maximum_reuse_duration);
  const MonotonicTime map_expiry = add_duration(
      inputs.planning_snapshot.map.version.timestamp,
      config_.input_limits.map_max_age);
  const MonotonicTime robot_state_timestamp = minimum_time(
      inputs.robot_state.pose.timestamp, inputs.robot_state.curvature_timestamp);
  const MonotonicTime state_expiry = add_duration(
      robot_state_timestamp, config_.input_limits.robot_state_max_age);
  const MonotonicTime cable_expiry =
      add_duration(inputs.cable_state.timestamp,
                   config_.input_limits.cable_state_max_age);
  const MonotonicTime progress_expiry = add_duration(
      inputs.reference_progress.timestamp,
      config_.input_limits.reference_progress_max_age);
  const MonotonicTime telemetry_expiry = add_duration(
      inputs.cable_telemetry.timestamp,
      config_.input_limits.cable_telemetry_max_age);
  const MonotonicTime tracking_expiry = add_duration(
      inputs.execution_tracking_state.timestamp,
      config_.input_limits.execution_tracking_max_age);
  MonotonicTime expires = reuse_expiry;
  for (const MonotonicTime candidate :
       {map_expiry, state_expiry, cable_expiry, progress_expiry,
        telemetry_expiry, tracking_expiry}) {
    expires = minimum_time(expires, candidate);
  }
  MonotonicTime envelope_expiry =
      context.locked_envelope->coverage_certification.valid_until;
  envelope_expiry.nanoseconds =
      envelope_expiry.nanoseconds >= config_.envelope_validity_margin.nanoseconds
          ? envelope_expiry.nanoseconds -
                config_.envelope_validity_margin.nanoseconds
          : 0;
  expires = minimum_time(expires, envelope_expiry);
  if (expires.nanoseconds <= now.nanoseconds) {
    result.valid = false;
    result.status = PlanValidationStatus::input_expired;
    result.action = PlanValidationAction::stop;
    result.cable_prediction.reset();
    issue(result, "PLAN_VALIDITY_LEASE_WINDOW_EMPTY",
          "no positive validity interval remains for a new lease", now);
    return result;
  }
  PlanValidationLease lease;
  lease.lease_sequence = next_lease_sequence_;
  lease.plan_sequence_number = plan.sequence_number;
  lease.evaluator_config_version = config_.version;
  lease.parameter_profile_id = config_.parameter_profile_id;
  lease.validated_at = now;
  lease.expires_at = expires;
  lease.remaining_path_start_arc_length_m = start_s;
  lease.map_version = plan.map_version;
  lease.reference_line_version = plan.reference_line_version;
  lease.robot_operating_area_version = plan.robot_operating_area_version;
  lease.terrain_gradient_policy_version = plan.terrain_gradient_policy_version;
  lease.corridor_risk_policy_version = plan.corridor_risk_policy_version;
  lease.cable_model_version = plan.cable_model_version;
  lease.uncertainty_envelope_version = plan.uncertainty_envelope_version;
  lease.uncertainty_envelope_generator_version =
      plan.uncertainty_envelope_generator_version;
  lease.execution_operating_envelope_version = plan.execution_operating_envelope_version;
  lease.execution_profile_version = plan.execution_profile_version;
  lease.sensor_mode = plan.sensor_mode;
  lease.operating_domain_id = plan.operating_domain_id;
  lease.cable_corridor_version = plan.cable_corridor_version;
  lease.robot_state_timestamp = robot_state_timestamp;
  lease.cable_state_timestamp = inputs.cable_state.timestamp;
  lease.cable_telemetry_timestamp = inputs.cable_telemetry.timestamp;
  lease.execution_tracking_timestamp = inputs.execution_tracking_state.timestamp;
  lease.max_ground_speed_tracking_error_mps =
      config_.maximum_ground_speed_tracking_error_mps;
  lease.max_payout_speed_tracking_error_mps =
      remaining->execution_profile.approved_tracking_limits.maximum_payout_tracking_error_mps;
  lease.allowed_tension = remaining->execution_profile.approved_tracking_limits.tension;
  lease.allowed_ground_acceleration =
      remaining->execution_profile.approved_tracking_limits.ground_acceleration;
  lease.robot_path_validation_passed = true;
  lease.cable_corridor_validation_passed = cable_result.corridor_result.hard_feasible;
  lease.cable_laying_validation_passed = cable_result.laying_result.hard_feasible;
  if (!context.envelope_manager->registerDependentLease(
          lease.lease_sequence, plan.sequence_number, *context.locked_envelope,
          now)) {
    result.valid = false;
    result.status = PlanValidationStatus::context_mismatch;
    result.action = PlanValidationAction::replan;
    result.cable_prediction.reset();
    issue(result, "PLAN_VALIDITY_LEASE_REGISTRATION",
          "lease could not be bound to the active plan and uncertainty envelope",
          now);
    return result;
  }
  ++next_lease_sequence_;
  result.remaining_path = std::make_shared<const TimedPath>(*remaining);
  result.lease = lease;
  return result;
}

}  // namespace underwater_planner::core
