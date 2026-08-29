#include "underwater_planner/core/stability_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace underwater_planner::core {
namespace {

constexpr double kEpsilon = 1.0e-12;
constexpr double kPi = 3.141592653589793238462643383279502884;

bool finite(const double value) { return std::isfinite(value); }

double point_distance(const PathPoint& left, const PathPoint& right) {
  return std::hypot(left.x_m - right.x_m, left.y_m - right.y_m);
}

double directed_distance(const GeometricPath& source,
                         const GeometricPath& target) {
  double maximum = 0.0;
  for (const PathPoint& point : source.points) {
    double nearest = std::numeric_limits<double>::infinity();
    for (const PathPoint& other : target.points) {
      nearest = std::min(nearest, point_distance(point, other));
    }
    maximum = std::max(maximum, nearest);
  }
  return maximum;
}

bool finite_nonnegative(const double value) {
  return finite(value) && value >= 0.0;
}

std::optional<PathPoint> interpolate_geometry(const GeometricPath& path,
                                              const double arc) {
  if (path.points.size() < 2U || !finite(arc) ||
      arc < path.points.front().arc_length_m ||
      arc > path.points.back().arc_length_m) {
    return std::nullopt;
  }
  std::size_t right_index = 1U;
  while (right_index < path.points.size() &&
         path.points[right_index].arc_length_m < arc) {
    ++right_index;
  }
  if (right_index == path.points.size()) return path.points.back();
  const PathPoint& left = path.points[right_index - 1U];
  const PathPoint& right = path.points[right_index];
  const double span = right.arc_length_m - left.arc_length_m;
  if (!finite(span) || span <= 0.0) return std::nullopt;
  const double ratio = std::clamp((arc - left.arc_length_m) / span, 0.0, 1.0);
  auto lerp = [ratio](const double a, const double b) {
    return a + ratio * (b - a);
  };
  const double heading_delta =
      std::remainder(right.heading_rad - left.heading_rad, 2.0 * kPi);
  return PathPoint{arc, lerp(left.x_m, right.x_m), lerp(left.y_m, right.y_m),
                   normalize_angle_radians(left.heading_rad + ratio * heading_delta),
                   lerp(left.curvature_per_m, right.curvature_per_m)};
}

std::optional<ExecutionSample> interpolate_execution(
    const ExecutionProfile& profile, const double arc) {
  if (profile.samples.size() < 2U || !finite(arc) ||
      arc < profile.samples.front().arc_length_m ||
      arc > profile.samples.back().arc_length_m) {
    return std::nullopt;
  }
  std::size_t right_index = 1U;
  while (right_index < profile.samples.size() &&
         profile.samples[right_index].arc_length_m < arc) {
    ++right_index;
  }
  if (right_index == profile.samples.size()) return profile.samples.back();
  const ExecutionSample& left = profile.samples[right_index - 1U];
  const ExecutionSample& right = profile.samples[right_index];
  const double span = right.arc_length_m - left.arc_length_m;
  if (!finite(span) || span <= 0.0) return std::nullopt;
  const double ratio = std::clamp((arc - left.arc_length_m) / span, 0.0, 1.0);
  const auto lerp = [ratio](const double a, const double b) {
    return a + ratio * (b - a);
  };
  const double left_time = static_cast<double>(left.time_from_start.nanoseconds);
  const double right_time = static_cast<double>(right.time_from_start.nanoseconds);
  const double time = lerp(left_time, right_time);
  if (!finite(time) || time < 0.0 ||
      time > static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
    return std::nullopt;
  }
  return ExecutionSample{
      arc,
      Duration{static_cast<std::int64_t>(std::llround(time))},
      lerp(left.ground_speed_mps, right.ground_speed_mps),
      lerp(left.ground_acceleration_mps2, right.ground_acceleration_mps2),
      lerp(left.payout_speed_mps, right.payout_speed_mps),
      lerp(left.payout_acceleration_mps2, right.payout_acceleration_mps2),
      lerp(left.tension_setpoint_n, right.tension_setpoint_n)};
}

double projection_arc(const GeometricPath& path, const RobotState& state) {
  double best_distance = std::numeric_limits<double>::infinity();
  double best_arc = path.points.front().arc_length_m;
  for (std::size_t index = 1U; index < path.points.size(); ++index) {
    const PathPoint& left = path.points[index - 1U];
    const PathPoint& right = path.points[index];
    const double dx = right.x_m - left.x_m;
    const double dy = right.y_m - left.y_m;
    const double length_squared = dx * dx + dy * dy;
    const double ratio = length_squared > kEpsilon
                             ? std::clamp(((state.pose.x_m - left.x_m) * dx +
                                           (state.pose.y_m - left.y_m) * dy) /
                                              length_squared,
                                          0.0, 1.0)
                             : 0.0;
    const double x = left.x_m + ratio * dx;
    const double y = left.y_m + ratio * dy;
    const double distance = std::hypot(state.pose.x_m - x, state.pose.y_m - y);
    if (distance < best_distance) {
      best_distance = distance;
      best_arc = left.arc_length_m +
                 ratio * (right.arc_length_m - left.arc_length_m);
    }
  }
  return best_arc;
}

std::vector<PathPoint> slice_geometry(const GeometricPath& path,
                                      const double start, const double end) {
  std::vector<PathPoint> points;
  points.push_back(*interpolate_geometry(path, start));
  for (const PathPoint& point : path.points) {
    if (point.arc_length_m > start + kEpsilon &&
        point.arc_length_m < end - kEpsilon) {
      points.push_back(point);
    }
  }
  points.push_back(*interpolate_geometry(path, end));
  return points;
}

std::vector<ExecutionSample> slice_execution(const ExecutionProfile& profile,
                                              const double start,
                                              const double end) {
  std::vector<ExecutionSample> samples;
  const ExecutionSample first = *interpolate_execution(profile, start);
  const std::int64_t offset = first.time_from_start.nanoseconds;
  auto shifted = [offset](ExecutionSample sample) {
    sample.time_from_start.nanoseconds -= offset;
    return sample;
  };
  samples.push_back(shifted(first));
  for (const ExecutionSample& sample : profile.samples) {
    if (sample.arc_length_m > start + kEpsilon &&
        sample.arc_length_m < end - kEpsilon) {
      samples.push_back(shifted(sample));
    }
  }
  samples.push_back(shifted(*interpolate_execution(profile, end)));
  return samples;
}

bool close_enough(const double left, const double right, const double tolerance) {
  return finite(left) && finite(right) && finite(tolerance) && tolerance >= 0.0 &&
         std::abs(left - right) <= tolerance;
}

bool same_execution_limits(const SpeedPayoutLimits& left,
                           const SpeedPayoutLimits& right) {
  return left.ground_speed.minimum_mps == right.ground_speed.minimum_mps &&
         left.ground_speed.maximum_mps == right.ground_speed.maximum_mps &&
         left.ground_acceleration.minimum_mps2 ==
             right.ground_acceleration.minimum_mps2 &&
         left.ground_acceleration.maximum_mps2 ==
             right.ground_acceleration.maximum_mps2 &&
         left.maximum_lateral_acceleration_mps2 ==
             right.maximum_lateral_acceleration_mps2 &&
         left.payout_speed.minimum_mps == right.payout_speed.minimum_mps &&
         left.payout_speed.maximum_mps == right.payout_speed.maximum_mps &&
         left.payout_acceleration.minimum_mps2 ==
             right.payout_acceleration.minimum_mps2 &&
         left.payout_acceleration.maximum_mps2 ==
             right.payout_acceleration.maximum_mps2 &&
         left.maximum_payout_tracking_error_mps ==
             right.maximum_payout_tracking_error_mps &&
         left.tension.minimum_n == right.tension.minimum_n &&
         left.tension.maximum_n == right.tension.maximum_n &&
         left.maximum_stopping_distance_m ==
             right.maximum_stopping_distance_m;
}

}  // namespace

StabilityManager::StabilityManager(PathHysteresisConfig config) noexcept
    : config_(config) {}

bool StabilityManager::valid_path(const GeometricPath& path) noexcept {
  if (path.points.size() < 2U) return false;
  double previous_arc = -std::numeric_limits<double>::infinity();
  for (const PathPoint& point : path.points) {
    if (!finite(point.arc_length_m) || !finite(point.x_m) ||
        !finite(point.y_m) || !finite(point.heading_rad) ||
        !finite(point.curvature_per_m) ||
        point.arc_length_m <= previous_arc + kEpsilon) {
      return false;
    }
    previous_arc = point.arc_length_m;
  }
  return true;
}

bool StabilityManager::should_switch_path(const GeometricPath& current_path,
                                           const GeometricPath& new_path,
                                           const double current_cost,
                                           const double new_cost) const noexcept {
  if (!valid_path(current_path) || !valid_path(new_path) ||
      !finite(current_cost) || !finite(new_cost) || current_cost < 0.0 ||
      new_cost < 0.0 || !finite(config_.relative_cost_threshold) ||
      config_.relative_cost_threshold < 0.0 ||
      config_.relative_cost_threshold >= 1.0 ||
      (config_.topology_distance_threshold_m.has_value() &&
       (!finite(*config_.topology_distance_threshold_m) ||
        *config_.topology_distance_threshold_m <= 0.0))) {
    return false;
  }
  const double threshold = current_cost * config_.relative_cost_threshold;
  if (new_cost < current_cost - threshold) return true;
  return config_.topology_distance_threshold_m.has_value() &&
         topology_distance_m(current_path, new_path) >
             *config_.topology_distance_threshold_m;
}

bool StabilityManager::valid_evaluation(
    const PlanValidityEvaluation& evaluation, const MonotonicTime now) noexcept {
  if (!evaluation.valid || evaluation.status != PlanValidationStatus::valid ||
      !evaluation.lease.has_value() || !evaluation.remaining_path ||
      !valid_path(evaluation.remaining_path->geometry)) {
    return false;
  }
  const PlanValidationLease& lease = *evaluation.lease;
  return lease.lease_sequence != 0U && lease.plan_sequence_number != 0U &&
         lease.validated_at.nanoseconds >= 0 &&
         lease.validated_at.nanoseconds <= now.nanoseconds &&
         lease.expires_at.nanoseconds > now.nanoseconds &&
         lease.expires_at.nanoseconds > lease.validated_at.nanoseconds &&
         evaluation.evaluator_config_version == lease.evaluator_config_version &&
         evaluation.parameter_profile_id == lease.parameter_profile_id &&
         lease.robot_path_validation_passed &&
         lease.cable_corridor_validation_passed &&
         lease.cable_laying_validation_passed &&
         evaluation.remaining_path->geometry.metadata.reference_line_version ==
             lease.reference_line_version &&
         evaluation.remaining_path->execution_profile.version ==
             lease.execution_profile_version &&
         evaluation.remaining_path->execution_profile.operating_envelope_version ==
             lease.execution_operating_envelope_version;
}

bool StabilityManager::lease_requires_recapture(
    const std::optional<PlanValidityEvaluation>& evaluation,
    const MonotonicTime now) noexcept {
  if (!evaluation.has_value() || !evaluation->lease.has_value()) return false;
  return evaluation->lease->expires_at.nanoseconds <= now.nanoseconds;
}

bool StabilityManager::same_validation_context(
    const PlanValidationLease& left,
    const PlanValidationLease& right) noexcept {
  return left.evaluator_config_version == right.evaluator_config_version &&
         left.parameter_profile_id == right.parameter_profile_id &&
         left.map_version == right.map_version &&
         left.reference_line_version == right.reference_line_version &&
         left.robot_operating_area_version == right.robot_operating_area_version &&
         left.cable_corridor_version == right.cable_corridor_version &&
         left.terrain_gradient_policy_version ==
             right.terrain_gradient_policy_version &&
         left.corridor_risk_policy_version == right.corridor_risk_policy_version &&
         left.cable_model_version == right.cable_model_version &&
         left.uncertainty_envelope_version == right.uncertainty_envelope_version &&
         left.uncertainty_envelope_generator_version ==
             right.uncertainty_envelope_generator_version &&
         left.execution_operating_envelope_version ==
             right.execution_operating_envelope_version &&
         left.sensor_mode == right.sensor_mode &&
         left.operating_domain_id == right.operating_domain_id &&
         left.validated_at.nanoseconds == right.validated_at.nanoseconds &&
         left.robot_state_timestamp.nanoseconds ==
             right.robot_state_timestamp.nanoseconds &&
         left.cable_state_timestamp.nanoseconds ==
             right.cable_state_timestamp.nanoseconds &&
         left.cable_telemetry_timestamp.nanoseconds ==
             right.cable_telemetry_timestamp.nanoseconds &&
         left.execution_tracking_timestamp.nanoseconds ==
             right.execution_tracking_timestamp.nanoseconds;
}

PathSwitchDecision StabilityManager::decide_path_switch(
    const std::optional<PlanValidityEvaluation>& current,
    const PlanValidityEvaluation& candidate, const double current_cost,
    const double candidate_cost, const MonotonicTime now) const {
  const bool candidate_valid = valid_evaluation(candidate, now);
  const bool current_valid = current.has_value() && valid_evaluation(*current, now);

  if (!candidate_valid) {
    if (current_valid) {
      return {PathSwitchAction::keep_current, current->lease,
              current->remaining_path, "candidate_validation_failed"};
    }
    return {PathSwitchAction::stop, std::nullopt, nullptr,
            "candidate_and_current_validation_failed"};
  }

  if (!finite(candidate_cost) || candidate_cost < 0.0) {
    if (current_valid) {
      return {PathSwitchAction::keep_current, current->lease,
              current->remaining_path, "invalid_path_cost"};
    }
    return {PathSwitchAction::stop, std::nullopt, nullptr,
            "invalid_candidate_cost"};
  }

  if (!current.has_value() || !current_valid) {
    return {PathSwitchAction::switch_to_candidate, candidate.lease,
            candidate.remaining_path, "current_plan_not_valid"};
  }

  if (!same_validation_context(*current->lease, *candidate.lease)) {
    return {PathSwitchAction::stop, std::nullopt, nullptr,
            "validation_context_mismatch"};
  }

  // A malformed soft-cost report must never make a candidate publishable.  A
  // separately validated current plan remains the safe fallback.
  if (!finite(current_cost) || current_cost < 0.0) {
    return {PathSwitchAction::keep_current, current->lease,
            current->remaining_path, "invalid_path_cost"};
  }

  if (should_switch_path(current->remaining_path->geometry,
                         candidate.remaining_path->geometry, current_cost,
                         candidate_cost)) {
    return {PathSwitchAction::switch_to_candidate, candidate.lease,
            candidate.remaining_path, "candidate_cost_significantly_lower"};
  }
  return {PathSwitchAction::keep_current, current->lease,
          current->remaining_path, "cost_improvement_within_hysteresis"};
}

PathSwitchDecision StabilityManager::decide_path_switch(
    const std::optional<PlanValidityEvaluation>& current,
    const PlanValidityEvaluation& candidate, const double current_cost,
    const double candidate_cost, const MonotonicTime now,
    const RevalidationCallback& recapture_and_revalidate) const {
  if (lease_requires_recapture(current, now) ||
      lease_requires_recapture(candidate, now)) {
    if (!recapture_and_revalidate) {
      return {PathSwitchAction::stop, std::nullopt, nullptr,
              "lease_expired_without_recapture"};
    }
    const RevalidationResult refreshed = recapture_and_revalidate();
    return decide_path_switch(refreshed.first, refreshed.second, current_cost,
                              candidate_cost, now);
  }
  return decide_path_switch(current, candidate, current_cost, candidate_cost,
                            now);
}

double StabilityManager::topology_distance_m(const GeometricPath& left,
                                              const GeometricPath& right) noexcept {
  if (!valid_path(left) || !valid_path(right)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(directed_distance(left, right), directed_distance(right, left));
}

CommitmentExtractionResult StabilityManager::extract_commitment_segment(
    const TimedPath& current_trajectory, const RobotState& robot_state,
    const CommitmentSegmentConfig& config) const {
  CommitmentExtractionResult result;
  const ValidationResult validation = validate(current_trajectory);
  if (!validation.valid || !finite(config.commitment_time_s) ||
      config.commitment_time_s <= 0.0 || !finite(config.safety_margin_m) ||
      config.safety_margin_m < 0.0 || !finite(robot_state.pose.x_m) ||
      !finite(robot_state.pose.y_m) || !finite(robot_state.ground_speed_mps) ||
      robot_state.ground_speed_mps < 0.0) {
    result.reason = "commitment_input_invalid";
    return result;
  }
  const SpeedPayoutLimits& limits =
      current_trajectory.execution_profile.approved_tracking_limits;
  if (!config.certified_worst_case_stopping_distance_m.has_value() ||
      !finite(*config.certified_worst_case_stopping_distance_m) ||
      *config.certified_worst_case_stopping_distance_m < 0.0) {
    result.status = CommitmentExtractionStatus::stopping_distance_unavailable;
    result.reason = "certified_braking_distance_unavailable";
    return result;
  }
  const double stopping_distance =
      *config.certified_worst_case_stopping_distance_m;
  result.required_length_m = std::max(
      robot_state.ground_speed_mps * config.commitment_time_s,
      stopping_distance + config.safety_margin_m);
  if (!finite(result.required_length_m)) {
    result.status = CommitmentExtractionStatus::stopping_distance_unavailable;
    result.reason = "required_commitment_exceeds_certified_stop_limit";
    return result;
  }
  const double start = projection_arc(current_trajectory.geometry, robot_state);
  const auto expected_initial =
      interpolate_execution(current_trajectory.execution_profile, start);
  if (!expected_initial.has_value() ||
      robot_state.ground_speed_mps >
          limits.ground_speed.maximum_mps + kEpsilon ||
      std::abs(robot_state.ground_speed_mps -
               expected_initial->ground_speed_mps) > 1.0e-6) {
    result.status = CommitmentExtractionStatus::input_invalid;
    result.reason = "robot_speed_not_synchronized_to_authorized_profile";
    return result;
  }
  const double end = current_trajectory.geometry.points.back().arc_length_m;
  result.available_length_m = end - start;
  if (!finite(result.available_length_m) ||
      result.available_length_m + kEpsilon < result.required_length_m) {
    result.status = CommitmentExtractionStatus::authorization_range_insufficient;
    result.reason = "authorized_trajectory_does_not_cover_commitment";
    return result;
  }
  const double commitment_end = start + result.required_length_m;
  if (commitment_end <= start + kEpsilon ||
      !interpolate_geometry(current_trajectory.geometry, commitment_end).has_value() ||
      !interpolate_execution(current_trajectory.execution_profile, commitment_end)
           .has_value()) {
    result.reason = "commitment_boundary_cannot_be_interpolated";
    return result;
  }
  TimedPath segment;
  segment.geometry = current_trajectory.geometry;
  segment.geometry.points =
      slice_geometry(current_trajectory.geometry, start, commitment_end);
  segment.geometry.metadata.smoothing.reset();
  segment.execution_profile = current_trajectory.execution_profile;
  segment.execution_profile.samples = slice_execution(
      current_trajectory.execution_profile, start, commitment_end);
  // Preserve the original authorized stop location.  The extracted segment
  // is a moving prefix, so its stop point intentionally remains beyond the
  // prefix geometry.
  segment.execution_profile.stopping_point_arc_length_m =
      current_trajectory.execution_profile.stopping_point_arc_length_m;
  if (!validate_authorized_prefix(segment).valid) {
    result.reason = "commitment_slice_invalid";
    return result;
  }
  result.status = CommitmentExtractionStatus::valid;
  result.reason = "valid";
  result.segment = std::move(segment);
  return result;
}

TimedPathMergeResult StabilityManager::merge_timed_paths(
    const TimedPath& commitment, const TimedPath& new_tail,
    const PathG2MergeLimits& geometric_tolerances,
    const ExecutionJoinTolerances& execution_tolerances,
    const TimedPathFinalVerifier& final_verifier) const {
  TimedPathMergeResult result;
  if (!validate_authorized_prefix(commitment).valid ||
      !validate(new_tail).valid ||
      commitment.execution_profile.operating_envelope_version !=
          new_tail.execution_profile.operating_envelope_version ||
      commitment.execution_profile.interpolation_rule !=
          new_tail.execution_profile.interpolation_rule ||
      !same_execution_limits(commitment.execution_profile.approved_tracking_limits,
                             new_tail.execution_profile.approved_tracking_limits) ||
      !finite_nonnegative(execution_tolerances.ground_speed_mps) ||
      !finite_nonnegative(execution_tolerances.ground_acceleration_mps2) ||
      !finite_nonnegative(execution_tolerances.payout_speed_mps) ||
      !finite_nonnegative(execution_tolerances.payout_acceleration_mps2) ||
      !finite_nonnegative(execution_tolerances.tension_n)) {
    result.reason = "timed_path_or_execution_tolerances_invalid";
    return result;
  }
  const PathG2MergeResult geometry = mergePathsG2(
      commitment.geometry, new_tail.geometry, geometric_tolerances);
  if (!geometry.valid) {
    result.reason = "g2_junction_residual_exceeded";
    return result;
  }
  const ExecutionSample& left = commitment.execution_profile.samples.back();
  const ExecutionSample& right = new_tail.execution_profile.samples.front();
  if (!close_enough(left.ground_speed_mps, right.ground_speed_mps,
                    execution_tolerances.ground_speed_mps) ||
      !close_enough(left.ground_acceleration_mps2, right.ground_acceleration_mps2,
                    execution_tolerances.ground_acceleration_mps2) ||
      !close_enough(left.payout_speed_mps, right.payout_speed_mps,
                    execution_tolerances.payout_speed_mps) ||
      !close_enough(left.payout_acceleration_mps2,
                    right.payout_acceleration_mps2,
                    execution_tolerances.payout_acceleration_mps2) ||
      !close_enough(left.tension_setpoint_n, right.tension_setpoint_n,
                    execution_tolerances.tension_n)) {
    result.reason = "execution_junction_residual_exceeded";
    return result;
  }
  if (commitment.execution_profile.samples.back().time_from_start.nanoseconds < 0 ||
      new_tail.execution_profile.samples.front().time_from_start.nanoseconds < 0) {
    result.reason = "execution_time_invalid";
    return result;
  }
  if (new_tail.execution_profile.samples.back().time_from_start.nanoseconds >
      std::numeric_limits<std::int64_t>::max() -
          commitment.execution_profile.samples.back().time_from_start.nanoseconds) {
    result.reason = "merged_execution_time_overflow";
    return result;
  }
  TimedPath merged;
  merged.geometry = geometry.path;
  merged.execution_profile = commitment.execution_profile;
  merged.execution_profile.samples = commitment.execution_profile.samples;
  const double arc_offset = commitment.geometry.points.back().arc_length_m -
                            new_tail.geometry.points.front().arc_length_m;
  const std::int64_t time_offset =
      commitment.execution_profile.samples.back().time_from_start.nanoseconds;
  for (std::size_t index = 1U;
       index < new_tail.execution_profile.samples.size(); ++index) {
    ExecutionSample sample = new_tail.execution_profile.samples[index];
    sample.arc_length_m += arc_offset;
    sample.time_from_start.nanoseconds += time_offset;
    merged.execution_profile.samples.push_back(sample);
  }
  if (merged.execution_profile.version == std::numeric_limits<std::uint64_t>::max() ||
      new_tail.execution_profile.version == std::numeric_limits<std::uint64_t>::max()) {
    result.reason = "execution_profile_version_exhausted";
    return result;
  }
  merged.execution_profile.version = std::max(
      commitment.execution_profile.version, new_tail.execution_profile.version) + 1U;
  if (new_tail.execution_profile.stopping_point_arc_length_m.has_value()) {
    merged.execution_profile.stopping_point_arc_length_m =
        *new_tail.execution_profile.stopping_point_arc_length_m + arc_offset;
  } else {
    merged.execution_profile.stopping_point_arc_length_m =
        merged.execution_profile.samples.back().arc_length_m;
  }
  if (!validate(merged).valid) {
    result.reason = "merged_timed_path_invalid";
    return result;
  }
  if (!final_verifier || !final_verifier(merged)) {
    result.reason = "full_path_final_validation_failed";
    return result;
  }
  result.valid = true;
  result.reason = "valid";
  result.trajectory = std::move(merged);
  return result;
}

}  // namespace underwater_planner::core
