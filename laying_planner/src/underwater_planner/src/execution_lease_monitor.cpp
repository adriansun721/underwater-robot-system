#include "underwater_planner/core/execution_lease_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace underwater_planner::core {
namespace {

bool finite(const double value) { return std::isfinite(value); }

bool finite_range(const RangeMps& range) {
  return finite(range.minimum_mps) && finite(range.maximum_mps) &&
         range.minimum_mps <= range.maximum_mps;
}

bool finite_range(const RangeMps2& range) {
  return finite(range.minimum_mps2) && finite(range.maximum_mps2) &&
         range.minimum_mps2 <= range.maximum_mps2;
}

bool finite_range(const RangeN& range) {
  return finite(range.minimum_n) && finite(range.maximum_n) &&
         range.minimum_n <= range.maximum_n;
}

bool in_range(const double value, const RangeMps& range) {
  return value >= range.minimum_mps && value <= range.maximum_mps;
}

bool in_range(const double value, const RangeMps2& range) {
  return value >= range.minimum_mps2 && value <= range.maximum_mps2;
}

bool in_range(const double value, const RangeN& range) {
  return value >= range.minimum_n && value <= range.maximum_n;
}

bool time_valid(const MonotonicTime time) { return time.nanoseconds >= 0; }

bool time_not_after(const MonotonicTime left, const MonotonicTime right) {
  return time_valid(left) && time_valid(right) &&
         left.nanoseconds <= right.nanoseconds;
}

bool age_within(const MonotonicTime timestamp, const MonotonicTime now,
                const Duration maximum) {
  return time_not_after(timestamp, now) && maximum.nanoseconds > 0 &&
         now.nanoseconds - timestamp.nanoseconds <= maximum.nanoseconds;
}

void add_diagnostic(ExecutionAuthorization& result, const char* code,
                    const std::string& message, const MonotonicTime at) {
  result.reason_code = code;
  result.reason = message;
  result.diagnostics.push_back(
      {DiagnosticSeverity::error, code, "execution_lease_monitor", message, at});
}

std::size_t hash_double(std::size_t seed, const double value) {
  std::uint64_t bits{};
  static_assert(sizeof(bits) == sizeof(value), "unexpected double size");
  std::memcpy(&bits, &value, sizeof(bits));
  seed ^= static_cast<std::size_t>(bits) +
          static_cast<std::size_t>(0x9e3779b97f4a7c15ULL) + (seed << 6U) +
          (seed >> 2U);
  return seed;
}

std::size_t profile_fingerprint(const ExecutionProfile& profile) {
  std::size_t hash = static_cast<std::size_t>(profile.version);
  hash ^= static_cast<std::size_t>(profile.operating_envelope_version) +
          (hash << 6U) + (hash >> 2U);
  for (const ExecutionSample& sample : profile.samples) {
    hash = hash_double(hash, sample.arc_length_m);
    hash = hash_double(hash, static_cast<double>(sample.time_from_start.nanoseconds));
    hash = hash_double(hash, sample.ground_speed_mps);
    hash = hash_double(hash, sample.ground_acceleration_mps2);
    hash = hash_double(hash, sample.payout_speed_mps);
    hash = hash_double(hash, sample.payout_acceleration_mps2);
    hash = hash_double(hash, sample.tension_setpoint_n);
  }
  return hash;
}

bool profile_valid(const TimedPath& path) {
  const ExecutionProfile& profile = path.execution_profile;
  if (profile.version == 0U || profile.operating_envelope_version == 0U ||
      path.geometry.points.empty() || profile.interpolation_rule.empty() ||
      profile.samples.empty() || !finite_range(profile.approved_tracking_limits.ground_speed) ||
      !finite_range(profile.approved_tracking_limits.ground_acceleration) ||
      !finite_range(profile.approved_tracking_limits.payout_speed) ||
      !finite_range(profile.approved_tracking_limits.payout_acceleration) ||
      !finite_range(profile.approved_tracking_limits.tension) ||
      !finite(profile.approved_tracking_limits.maximum_payout_tracking_error_mps)) {
    return false;
  }
  double previous_arc = -std::numeric_limits<double>::infinity();
  std::int64_t previous_time = -1;
  for (const ExecutionSample& sample : profile.samples) {
    if (!finite(sample.arc_length_m) || sample.arc_length_m <= previous_arc ||
        !time_valid(MonotonicTime{sample.time_from_start.nanoseconds}) ||
        sample.time_from_start.nanoseconds <= previous_time ||
        !finite(sample.ground_speed_mps) || !finite(sample.ground_acceleration_mps2) ||
        !finite(sample.payout_speed_mps) || !finite(sample.payout_acceleration_mps2) ||
        !finite(sample.tension_setpoint_n)) {
      return false;
    }
    previous_arc = sample.arc_length_m;
    previous_time = sample.time_from_start.nanoseconds;
  }
  double previous_geometry_arc = -std::numeric_limits<double>::infinity();
  for (const PathPoint& point : path.geometry.points) {
    if (!finite(point.arc_length_m) || point.arc_length_m <= previous_geometry_arc ||
        !finite(point.x_m) || !finite(point.y_m) || !finite(point.heading_rad) ||
        !finite(point.curvature_per_m)) {
      return false;
    }
    previous_geometry_arc = point.arc_length_m;
  }
  const double tolerance = 1.0e-6;
  if (std::abs(profile.samples.front().arc_length_m -
               path.geometry.points.front().arc_length_m) > tolerance ||
      std::abs(profile.samples.back().arc_length_m -
               path.geometry.points.back().arc_length_m) > tolerance) {
    return false;
  }
  if (profile.stopping_point_arc_length_m.has_value() &&
      (!finite(*profile.stopping_point_arc_length_m) ||
       *profile.stopping_point_arc_length_m < profile.samples.front().arc_length_m ||
       *profile.stopping_point_arc_length_m > profile.samples.back().arc_length_m)) {
    return false;
  }
  return true;
}

struct ExpectedValues {
  double ground_speed_mps{};
  double ground_acceleration_mps2{};
  double payout_speed_mps{};
  double payout_acceleration_mps2{};
  double tension_n{};
};

ExpectedValues expected_at(const ExecutionProfile& profile, double arc_length_m) {
  const std::vector<ExecutionSample>& samples = profile.samples;
  if (arc_length_m <= samples.front().arc_length_m) {
    const ExecutionSample& sample = samples.front();
    return {sample.ground_speed_mps, sample.ground_acceleration_mps2,
            sample.payout_speed_mps, sample.payout_acceleration_mps2,
            sample.tension_setpoint_n};
  }
  if (arc_length_m >= samples.back().arc_length_m) {
    const ExecutionSample& sample = samples.back();
    return {sample.ground_speed_mps, sample.ground_acceleration_mps2,
            sample.payout_speed_mps, sample.payout_acceleration_mps2,
            sample.tension_setpoint_n};
  }
  for (std::size_t index = 1U; index < samples.size(); ++index) {
    if (arc_length_m <= samples[index].arc_length_m) {
      const ExecutionSample& lower = samples[index - 1U];
      const ExecutionSample& upper = samples[index];
      const double span = upper.arc_length_m - lower.arc_length_m;
      const double ratio = (arc_length_m - lower.arc_length_m) / span;
      const auto interpolate = [ratio](const double first, const double second) {
        return first + ratio * (second - first);
      };
      return {interpolate(lower.ground_speed_mps, upper.ground_speed_mps),
              interpolate(lower.ground_acceleration_mps2,
                          upper.ground_acceleration_mps2),
              interpolate(lower.payout_speed_mps, upper.payout_speed_mps),
              interpolate(lower.payout_acceleration_mps2,
                          upper.payout_acceleration_mps2),
              interpolate(lower.tension_setpoint_n, upper.tension_setpoint_n)};
    }
  }
  return {};
}

ExecutionAuthorization rejected(const PlanValidationLease& lease,
                                const MonotonicTime now,
                                const char* code, const std::string& reason) {
  ExecutionAuthorization result;
  result.lease_sequence = lease.lease_sequence;
  result.evaluated_at = now;
  result.revoke_lease = true;
  result.request_controlled_stop = true;
  result.request_replan = true;
  add_diagnostic(result, code, reason, now);
  return result;
}

}  // namespace

ExecutionLeaseMonitor::ExecutionLeaseMonitor(
    ExecutionLeaseMonitorConfig config) noexcept
    : config_(config) {}

ExecutionAuthorization ExecutionLeaseMonitor::evaluate(
    const PlanningResult& plan, const TimedPath& trajectory,
    const PlanValidationLease& lease,
    const ActiveExecutionContext& active_context,
    const ExecutionFeedback& feedback) const {
  return evaluateAt(plan, trajectory, lease, active_context, feedback,
                    feedback.timestamp);
}

ExecutionAuthorization ExecutionLeaseMonitor::evaluate(
    const PlanningResult& plan, const TimedPath& trajectory,
    const PlanValidationLease& lease,
    const ActiveExecutionContext& active_context,
    const ExecutionFeedback& feedback, const MonotonicTime now) const {
  return evaluateAt(plan, trajectory, lease, active_context, feedback, now);
}

ExecutionAuthorization ExecutionLeaseMonitor::evaluateAt(
    const PlanningResult& plan, const TimedPath& trajectory,
    const PlanValidationLease& lease,
    const ActiveExecutionContext& active_context,
      const ExecutionFeedback& feedback, const MonotonicTime now) const {
  const auto reject = [&](const char* code, const std::string& reason) {
    ExecutionAuthorization result = rejected(lease, now, code, reason);
    revokeLease(lease.lease_sequence, code, reason, now);
    return result;
  };
  if (config_.feedback_max_age.nanoseconds <= 0 ||
      config_.monitor_period.nanoseconds <= 0 ||
      config_.renewal_margin.nanoseconds <= 0 ||
      config_.maximum_lease_duration.nanoseconds <= 0 ||
      config_.monitor_period.nanoseconds >=
          config_.maximum_lease_duration.nanoseconds ||
      config_.renewal_margin.nanoseconds >=
          config_.maximum_lease_duration.nanoseconds ||
      !finite(config_.maximum_ground_acceleration_tracking_error_mps2) ||
      config_.maximum_ground_acceleration_tracking_error_mps2 < 0.0 ||
      !finite(config_.maximum_payout_acceleration_tracking_error_mps2) ||
      config_.maximum_payout_acceleration_tracking_error_mps2 < 0.0 ||
      !finite(config_.maximum_tension_tracking_error_n) ||
      config_.maximum_tension_tracking_error_n < 0.0) {
    return reject("INVALID_MONITOR_CONFIGURATION",
                     "execution lease monitor configuration is invalid");
  }
  if (lease.lease_sequence == 0U || plan.sequence_number == 0U ||
      lease.plan_sequence_number != plan.sequence_number ||
      lease.execution_profile_version != trajectory.execution_profile.version ||
      plan.execution_profile_version != trajectory.execution_profile.version ||
      lease.execution_profile_version != plan.execution_profile_version) {
    return reject("PLAN_LEASE_MISMATCH",
                     "plan, remaining trajectory and lease are not paired");
  }
  if (isRevoked(lease.lease_sequence)) {
    return reject("LEASE_ALREADY_REVOKED",
                     "lease sequence was previously revoked");
  }
  if (!time_valid(now) || !time_valid(lease.validated_at) ||
      !time_valid(lease.expires_at) || !time_not_after(lease.validated_at, now) ||
      !time_not_after(now, lease.expires_at) || now.nanoseconds >= lease.expires_at.nanoseconds ||
      lease.expires_at.nanoseconds - lease.validated_at.nanoseconds >
          config_.maximum_lease_duration.nanoseconds) {
    return reject("LEASE_EXPIRED",
                     "lease is expired or has an invalid validity interval");
  }
  if (!time_valid(feedback.timestamp) || !age_within(feedback.timestamp, now,
                                                      config_.feedback_max_age)) {
    return reject("FEEDBACK_EXPIRED",
                     "execution feedback is stale, future-dated or invalid");
  }
  if (feedback.plan_sequence_number != plan.sequence_number ||
      feedback.execution_profile_version != trajectory.execution_profile.version) {
    return reject("FEEDBACK_PROFILE_MISMATCH",
                     "execution feedback does not identify the authorized plan/profile");
  }
  if (!profile_valid(trajectory)) {
    return reject("EXECUTION_PROFILE_INVALID",
                     "authorized execution profile is incomplete or non-finite");
  }
  const PlanningDependencyVersions current = active_context.dependencies();
  if (current.cable_corridor_version == 0U ||
      current != plan.dependencies() || current != lease.dependencies()) {
    return reject("DEPENDENCY_VERSION_MISMATCH",
                     "active context, plan and lease dependency versions differ");
  }
  if (!same_execution_profile_content(plan.robot_trajectory.execution_profile,
                                      trajectory.execution_profile)) {
    return reject("EXECUTION_PROFILE_CHANGED",
                     "execution profile content changed without a new validated version");
  }
  const double ground_speed =
      feedback.ground_speed.value_or(feedback.ground_speed_mps);
  const double ground_acceleration =
      feedback.ground_acceleration.value_or(feedback.ground_acceleration_mps2);
  const double payout_speed =
      feedback.payout_speed.value_or(feedback.payout_speed_mps);
  const double payout_acceleration = feedback.payout_acceleration.value_or(
      feedback.payout_acceleration_mps2);
  const double tension = feedback.tension.value_or(feedback.tension_n);
  if (!finite(ground_speed) || !finite(ground_acceleration) ||
      !finite(payout_speed) || !finite(payout_acceleration) || !finite(tension)) {
    return reject("FEEDBACK_NONFINITE",
                  "execution feedback contains a non-finite measurement");
  }
  const double arc_length = finite(feedback.tracked_arc_length_m)
                                ? feedback.tracked_arc_length_m
                                : lease.remaining_path_start_arc_length_m;
  if (!finite(arc_length)) {
    return reject("FEEDBACK_ARC_INVALID",
                  "execution feedback has no finite path progress");
  }
  const ExpectedValues expected =
      expected_at(trajectory.execution_profile, arc_length);
  const SpeedPayoutLimits& limits =
      trajectory.execution_profile.approved_tracking_limits;
  if (!finite(lease.max_ground_speed_tracking_error_mps) ||
      lease.max_ground_speed_tracking_error_mps < 0.0 ||
      !finite(lease.max_payout_speed_tracking_error_mps) ||
      lease.max_payout_speed_tracking_error_mps < 0.0 ||
      !finite_range(lease.allowed_ground_acceleration) ||
      !finite_range(lease.allowed_tension)) {
    return reject("LEASE_BOUNDS_INVALID",
                  "lease tracking limits are missing, non-finite or negative");
  }
  const std::size_t fingerprint = profile_fingerprint(trajectory.execution_profile);
  bool plan_sequence_out_of_order = false;
  bool lease_sequence_out_of_order = false;
  bool profile_changed = false;
  bool feedback_sequence_out_of_order = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    plan_sequence_out_of_order =
        plan.sequence_number < highest_plan_sequence_;
    lease_sequence_out_of_order =
        !plan_sequence_out_of_order &&
        lease.lease_sequence < highest_lease_sequence_;
    if (!plan_sequence_out_of_order && !lease_sequence_out_of_order) {
      const auto existing = observations_.find(lease.lease_sequence);
      if (existing != observations_.end() &&
          existing->second.fingerprint_set &&
          existing->second.profile_fingerprint != fingerprint) {
        profile_changed = true;
      }
      if (!profile_changed && existing != observations_.end() &&
          feedback.sequence_number != 0U &&
          feedback.sequence_number <
              existing->second.last_feedback_sequence) {
        feedback_sequence_out_of_order = true;
      }
      if (!profile_changed && !feedback_sequence_out_of_order) {
        highest_plan_sequence_ =
            std::max(highest_plan_sequence_, plan.sequence_number);
        highest_lease_sequence_ =
            std::max(highest_lease_sequence_, lease.lease_sequence);
        LeaseObservation& observation = observations_[lease.lease_sequence];
        observation.profile_fingerprint = fingerprint;
        observation.fingerprint_set = true;
        if (feedback.sequence_number > 0U) {
          observation.last_feedback_sequence = std::max(
              observation.last_feedback_sequence, feedback.sequence_number);
        }
      }
    }
  }
  if (plan_sequence_out_of_order) {
    return reject("PLAN_SEQUENCE_OUT_OF_ORDER",
                  "an older plan cannot supersede the newest accepted plan");
  }
  if (lease_sequence_out_of_order) {
    return reject("LEASE_SEQUENCE_OUT_OF_ORDER",
                  "an older lease cannot supersede the newest accepted lease");
  }
  if (profile_changed) {
    return reject("EXECUTION_PROFILE_CHANGED",
                  "control/profile content changed while the lease was active");
  }
  if (feedback_sequence_out_of_order) {
    return reject("FEEDBACK_SEQUENCE_OUT_OF_ORDER",
                  "an older execution feedback sample was received");
  }
  const bool ground_speed_bad =
      !in_range(ground_speed, limits.ground_speed) ||
      std::abs(ground_speed - expected.ground_speed_mps) >
          lease.max_ground_speed_tracking_error_mps;
  const bool payout_speed_bad =
      !in_range(payout_speed, limits.payout_speed) ||
      std::abs(payout_speed - expected.payout_speed_mps) >
          lease.max_payout_speed_tracking_error_mps;
  const bool ground_acceleration_bad =
      !in_range(ground_acceleration, lease.allowed_ground_acceleration) ||
      std::abs(ground_acceleration - expected.ground_acceleration_mps2) >
          config_.maximum_ground_acceleration_tracking_error_mps2;
  const bool payout_acceleration_bad =
      !in_range(payout_acceleration, limits.payout_acceleration) ||
      std::abs(payout_acceleration - expected.payout_acceleration_mps2) >
          config_.maximum_payout_acceleration_tracking_error_mps2;
  const bool tension_bad =
      !in_range(tension, lease.allowed_tension) ||
      std::abs(tension - expected.tension_n) >
          config_.maximum_tension_tracking_error_n;
  if (ground_speed_bad || payout_speed_bad || ground_acceleration_bad ||
      payout_acceleration_bad || tension_bad) {
    const char* code = ground_speed_bad
                           ? "GROUND_SPEED_TRACKING_BREACH"
                           : payout_speed_bad
                                 ? "PAYOUT_SPEED_TRACKING_BREACH"
                                 : ground_acceleration_bad
                                       ? "GROUND_ACCELERATION_TRACKING_BREACH"
                                       : payout_acceleration_bad
                                             ? "PAYOUT_ACCELERATION_TRACKING_BREACH"
                                             : "TENSION_TRACKING_BREACH";
    return reject(code,
                     "measured execution deviation exceeded the lease boundary");
  }
  const std::int64_t remaining_ns = lease.expires_at.nanoseconds - now.nanoseconds;
  ExecutionAuthorization result;
  result.status = remaining_ns <= config_.renewal_margin.nanoseconds
                      ? ExecutionAuthorizationStatus::renewal_required
                      : ExecutionAuthorizationStatus::authorized;
  result.lease_sequence = lease.lease_sequence;
  result.evaluated_at = now;
  result.reason_code = result.renewalRequired() ? "LEASE_RENEWAL_REQUIRED"
                                                : "LEASE_AUTHORIZED";
  result.reason = result.renewalRequired()
                      ? "lease is within renewal margin; complete a fresh validation"
                      : "execution feedback is within the current lease bounds";
  result.diagnostics.push_back(
      {DiagnosticSeverity::info, result.reason_code, "execution_lease_monitor",
       result.reason, now});
  return result;
}

void ExecutionLeaseMonitor::revokeLease(const std::uint64_t lease_sequence,
                                        std::string reason_code,
                                        std::string reason,
                                        const MonotonicTime /*at*/) const {
  if (lease_sequence == 0U) return;
  std::lock_guard<std::mutex> lock(mutex_);
  revoked_.emplace(lease_sequence,
                   std::make_pair(std::move(reason_code), std::move(reason)));
}

bool ExecutionLeaseMonitor::isRevoked(const std::uint64_t lease_sequence) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return revoked_.find(lease_sequence) != revoked_.end();
}

}  // namespace underwater_planner::core
