#include "underwater_planner/core/cable_state_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace underwater_planner::core {
namespace {

bool finite_vector(const Vector2m value) {
  return std::isfinite(value.x_m) && std::isfinite(value.y_m);
}

void validate_config(const CableStateTrackerConfig& config) {
  const bool positive_values =
      std::isfinite(config.touchdown_distance_m) &&
      config.touchdown_distance_m > 0.0 &&
      std::isfinite(config.direction_response_length_m) &&
      config.direction_response_length_m > 0.0 &&
      std::isfinite(config.curvature_evaluation_spacing_m) &&
      config.curvature_evaluation_spacing_m >
          config.minimum_distinct_touchdown_distance_m &&
      std::isfinite(config.support_evaluation_length_m) &&
      config.support_evaluation_length_m > 0.0 &&
      std::isfinite(config.minimum_distinct_touchdown_distance_m) &&
      config.minimum_distinct_touchdown_distance_m > 0.0 &&
      std::isfinite(config.initial_lag_variance_rad2) &&
      config.initial_lag_variance_rad2 > 0.0 &&
      std::isfinite(config.process_variance_per_m_rad2) &&
      config.process_variance_per_m_rad2 >= 0.0 &&
      std::isfinite(config.maximum_touchdown_observation_residual_m) &&
      config.maximum_touchdown_observation_residual_m > 0.0 &&
      std::isfinite(config.maximum_payout_speed_error_mps) &&
      config.maximum_payout_speed_error_mps >= 0.0 &&
      std::isfinite(config.minimum_tension_n) &&
      std::isfinite(config.maximum_tension_n) &&
      config.minimum_tension_n >= 0.0 &&
      config.maximum_tension_n >= config.minimum_tension_n;
  if (config.cable_model_version == 0U ||
      config.calibration_dataset_id.empty() ||
      config.operating_domain_id.empty() ||
      !finite_vector(config.release_point_offset_m) || !positive_values ||
      config.maximum_observation_gap.nanoseconds < 0 ||
      config.synchronization_tolerance.nanoseconds < 0) {
    throw std::invalid_argument(
        "cable tracker configuration must be finite and physically valid");
  }
}

Vector2m release_position(const ExecutedRobotSample& sample,
                          const Vector2m offset) {
  const double cosine = std::cos(sample.pose.heading_rad);
  const double sine = std::sin(sample.pose.heading_rad);
  return {sample.pose.x_m + cosine * offset.x_m - sine * offset.y_m,
          sample.pose.y_m + sine * offset.x_m + cosine * offset.y_m};
}

Vector2m touchdown_position(const ExecutedRobotSample& sample,
                            const Vector2m release_offset,
                            const double touchdown_distance_m,
                            const double lag_angle_rad) {
  const Vector2m release = release_position(sample, release_offset);
  const double cable_heading = sample.pose.heading_rad + lag_angle_rad;
  return {release.x_m - touchdown_distance_m * std::cos(cable_heading),
          release.y_m - touchdown_distance_m * std::sin(cable_heading)};
}

double squared_distance(const Vector2m left, const Vector2m right) {
  const double dx = left.x_m - right.x_m;
  const double dy = left.y_m - right.y_m;
  return dx * dx + dy * dy;
}

void append_distinct_point(CableConstraintMemory& memory, const Vector2m point,
                           const double minimum_distance_m) {
  if (!memory.previous_distinct_touchdown_points_m.empty() &&
      squared_distance(memory.previous_distinct_touchdown_points_m.back(),
                       point) < minimum_distance_m * minimum_distance_m) {
    return;
  }
  memory.previous_distinct_touchdown_points_m.push_back(point);
  if (memory.previous_distinct_touchdown_points_m.size() > 2U) {
    memory.previous_distinct_touchdown_points_m.erase(
        memory.previous_distinct_touchdown_points_m.begin());
  }
}

void append_touchdown_sample(CableConstraintMemory& memory,
                             const Vector2m point,
                             const double minimum_distinct_distance_m) {
  if (memory.trailing_support_samples.empty()) {
    memory.trailing_support_samples.push_back({0.0, point});
    append_distinct_point(memory, point, minimum_distinct_distance_m);
    return;
  }
  const double distance_m = std::sqrt(squared_distance(
      memory.trailing_support_samples.back().touchdown_position_m, point));
  if (distance_m <= 1.0e-12) return;
  memory.trailing_support_samples.push_back(
      {memory.trailing_support_samples.back().touchdown_arc_length_m +
           distance_m,
       point});
  append_distinct_point(memory, point, minimum_distinct_distance_m);
}

void initialize_memory_from_executed_history(
    CableState& state, const ExecutedRobotSegment& segment,
    const TouchdownObservation& observation,
    const CableStateTrackerConfig& config) {
  std::vector<double> lag_angles_rad(segment.samples.size());
  lag_angles_rad.back() = state.lag_angle_rad;
  for (std::size_t index = segment.samples.size() - 1U; index > 0U; --index) {
    const ExecutedRobotSample& previous = segment.samples[index - 1U];
    const ExecutedRobotSample& sample = segment.samples[index];
    const double distance_m =
        sample.robot_arc_length_m - previous.robot_arc_length_m;
    const double heading_change_rad = normalize_angle_radians(
        sample.pose.heading_rad - previous.pose.heading_rad);
    const double curvature_per_m = heading_change_rad / distance_m;
    const double decay =
        std::exp(-distance_m / config.direction_response_length_m);
    lag_angles_rad[index - 1U] = normalize_angle_radians(
        (lag_angles_rad[index] +
         curvature_per_m * config.direction_response_length_m *
             (1.0 - decay)) /
        decay);
  }

  for (std::size_t index = 0; index < segment.samples.size(); ++index) {
    const Vector2m touchdown =
        index + 1U == segment.samples.size()
            ? observation.touchdown_position_m
            : touchdown_position(segment.samples[index],
                                 config.release_point_offset_m,
                                 config.touchdown_distance_m,
                                 lag_angles_rad[index]);
    append_touchdown_sample(state.laying_memory, touchdown,
                            config.minimum_distinct_touchdown_distance_m);
  }
}

std::uint64_t fnv_mix(std::uint64_t hash, const std::uint64_t value) {
  for (unsigned int shift = 0; shift < 64U; shift += 8U) {
    hash ^= (value >> shift) & 0xffU;
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::uint64_t double_bits(const double value) {
  std::uint64_t bits{};
  static_assert(sizeof(bits) == sizeof(value), "double must be 64 bits");
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

void normalize_memory(CableConstraintMemory& memory,
                      const double retained_history_length_m) {
  if (memory.trailing_support_samples.empty()) {
    memory.retained_arc_length_m = 0.0;
    memory.canonical_signature = 0U;
    return;
  }
  const double cutoff_m =
      memory.trailing_support_samples.back().touchdown_arc_length_m -
      retained_history_length_m;
  while (memory.trailing_support_samples.size() > 1U &&
         memory.trailing_support_samples[1].touchdown_arc_length_m <= cutoff_m) {
    memory.trailing_support_samples.erase(
        memory.trailing_support_samples.begin());
  }
  memory.retained_arc_length_m =
      memory.trailing_support_samples.back().touchdown_arc_length_m -
      memory.trailing_support_samples.front().touchdown_arc_length_m;

  std::uint64_t signature = 14695981039346656037ULL;
  signature = fnv_mix(signature,
                      memory.previous_distinct_touchdown_points_m.size());
  for (const Vector2m point : memory.previous_distinct_touchdown_points_m) {
    signature = fnv_mix(signature, double_bits(point.x_m));
    signature = fnv_mix(signature, double_bits(point.y_m));
  }
  signature = fnv_mix(signature, memory.trailing_support_samples.size());
  for (const CableHistorySample& sample : memory.trailing_support_samples) {
    signature = fnv_mix(signature,
                        double_bits(sample.touchdown_arc_length_m));
    signature = fnv_mix(signature,
                        double_bits(sample.touchdown_position_m.x_m));
    signature = fnv_mix(signature,
                        double_bits(sample.touchdown_position_m.y_m));
  }
  memory.canonical_signature = fnv_mix(
      signature, double_bits(memory.retained_arc_length_m));
}

bool valid_sample(const ExecutedRobotSample& sample) {
  return std::isfinite(sample.robot_arc_length_m) &&
         sample.robot_arc_length_m >= 0.0 && std::isfinite(sample.pose.x_m) &&
         std::isfinite(sample.pose.y_m) &&
         std::isfinite(sample.pose.heading_rad) &&
         sample.pose.heading_rad >= -3.14159265358979323846 &&
         sample.pose.heading_rad < 3.14159265358979323846 &&
         sample.pose.timestamp.nanoseconds >= 0 &&
         std::isfinite(sample.ground_speed_mps) &&
         sample.ground_speed_mps >= 0.0;
}

bool segment_is_valid(const ExecutedRobotSegment& segment) {
  if (segment.sequence_number == 0U || segment.samples.empty()) return false;
  for (std::size_t index = 0; index < segment.samples.size(); ++index) {
    if (!valid_sample(segment.samples[index])) return false;
    if (index > 0U &&
        (segment.samples[index].robot_arc_length_m <=
             segment.samples[index - 1U].robot_arc_length_m ||
         segment.samples[index].pose.timestamp.nanoseconds <=
             segment.samples[index - 1U].pose.timestamp.nanoseconds)) {
      return false;
    }
  }
  return true;
}

bool same_boundary(const ExecutedRobotSample& left,
                   const ExecutedRobotSample& right) {
  constexpr double kPositionToleranceM = 1.0e-9;
  constexpr double kArcLengthToleranceM = 1.0e-9;
  constexpr double kHeadingToleranceRad = 1.0e-12;
  return std::abs(left.robot_arc_length_m - right.robot_arc_length_m) <=
             kArcLengthToleranceM &&
         std::hypot(left.pose.x_m - right.pose.x_m,
                    left.pose.y_m - right.pose.y_m) <= kPositionToleranceM &&
         std::abs(normalize_angle_radians(left.pose.heading_rad -
                                          right.pose.heading_rad)) <=
             kHeadingToleranceRad &&
         left.pose.timestamp.nanoseconds == right.pose.timestamp.nanoseconds;
}

bool observation_is_finite(const TouchdownObservation& observation) {
  const Covariance2dM2& covariance = observation.position_covariance_m2;
  const double determinant =
      covariance.xx_m2 * covariance.yy_m2 -
      covariance.xy_m2 * covariance.yx_m2;
  return observation.sequence_number != 0U &&
         finite_vector(observation.touchdown_position_m) &&
         observation.timestamp.nanoseconds >= 0 &&
         std::isfinite(covariance.xx_m2) &&
         std::isfinite(covariance.xy_m2) &&
         std::isfinite(covariance.yx_m2) &&
         std::isfinite(covariance.yy_m2) &&
         std::abs(covariance.xy_m2 - covariance.yx_m2) <= 1.0e-12 &&
         covariance.xx_m2 >= 0.0 && covariance.yy_m2 >= 0.0 &&
         determinant >= -1.0e-18;
}

void correct_from_observation(CableState& state,
                              const ExecutedRobotSample& terminal,
                              const TouchdownObservation& observation,
                              const CableStateTrackerConfig& config) {
  const Vector2m release =
      release_position(terminal, config.release_point_offset_m);
  const double dx = release.x_m - observation.touchdown_position_m.x_m;
  const double dy = release.y_m - observation.touchdown_position_m.y_m;
  const double distance_squared_m2 = dx * dx + dy * dy;
  const double distance_m = std::sqrt(distance_squared_m2);
  const double observed_heading_rad = std::atan2(dy, dx);
  const double observed_lag_rad = normalize_angle_radians(
      observed_heading_rad - terminal.pose.heading_rad);
  const double tangent_x = -dy / distance_m;
  const double tangent_y = dx / distance_m;
  const Covariance2dM2& covariance = observation.position_covariance_m2;
  const double tangential_variance_m2 =
      tangent_x * tangent_x * covariance.xx_m2 +
      tangent_x * tangent_y * (covariance.xy_m2 + covariance.yx_m2) +
      tangent_y * tangent_y * covariance.yy_m2;
  const double observation_variance_rad2 =
      std::max(0.0, tangential_variance_m2 / distance_squared_m2);
  const double prior_variance_rad2 = *state.lag_angle_variance_rad2;
  const double innovation_variance_rad2 =
      prior_variance_rad2 + observation_variance_rad2;
  const double gain = innovation_variance_rad2 > 0.0
                          ? prior_variance_rad2 / innovation_variance_rad2
                          : 0.0;
  const double residual_rad = normalize_angle_radians(
      observed_lag_rad - state.lag_angle_rad);
  state.lag_angle_rad =
      normalize_angle_radians(state.lag_angle_rad + gain * residual_rad);
  *state.lag_angle_variance_rad2 =
      (1.0 - gain) * prior_variance_rad2;

  CableConstraintMemory& memory = state.laying_memory;
  if (!memory.trailing_support_samples.empty()) {
    const Vector2m predicted_terminal =
        memory.trailing_support_samples.back().touchdown_position_m;
    if (memory.trailing_support_samples.size() > 1U) {
      const CableHistorySample& previous =
          memory.trailing_support_samples[memory.trailing_support_samples.size() -
                                          2U];
      const double corrected_distance_m = std::sqrt(squared_distance(
          previous.touchdown_position_m, observation.touchdown_position_m));
      if (corrected_distance_m <= 1.0e-12) {
        memory.trailing_support_samples.pop_back();
      } else {
        CableHistorySample& corrected = memory.trailing_support_samples.back();
        corrected.touchdown_arc_length_m =
            previous.touchdown_arc_length_m + corrected_distance_m;
        corrected.touchdown_position_m = observation.touchdown_position_m;
      }
    } else {
      memory.trailing_support_samples.back().touchdown_position_m =
          observation.touchdown_position_m;
    }
    if (!memory.previous_distinct_touchdown_points_m.empty() &&
        squared_distance(memory.previous_distinct_touchdown_points_m.back(),
                         predicted_terminal) < 1.0e-24) {
      memory.previous_distinct_touchdown_points_m.pop_back();
    }
  }
  append_distinct_point(memory, observation.touchdown_position_m,
                        config.minimum_distinct_touchdown_distance_m);
}

}  // namespace

CableStateTracker::CableStateTracker(CableStateTrackerConfig config)
    : config_(std::move(config)) {
  validate_config(config_);
  current_.cable_model_version = config_.cable_model_version;
  current_.calibration_dataset_id = config_.calibration_dataset_id;
  current_.operating_domain_id = config_.operating_domain_id;
  current_.risk_semantics = "state-estimate-only-no-path-risk-guarantee";
  current_.diagnostics.push_back(
      {CableTrackerDiagnosticCode::initial_state_uncertain,
       "no executed touchdown evidence has initialized the cable state",
       {-1}, 0, 0, std::nullopt});
}

std::string_view to_string(const CableTrackerDiagnosticCode code) {
  switch (code) {
    case CableTrackerDiagnosticCode::initial_state_uncertain:
      return "INITIAL_STATE_UNCERTAIN";
    case CableTrackerDiagnosticCode::task_start_uninitialized:
      return "TASK_START_UNINITIALIZED";
    case CableTrackerDiagnosticCode::mechanical_history_incomplete:
      return "MECHANICAL_HISTORY_INCOMPLETE";
    case CableTrackerDiagnosticCode::tracking_initialized:
      return "TRACKING_INITIALIZED";
    case CableTrackerDiagnosticCode::tracking_propagated:
      return "TRACKING_PROPAGATED";
    case CableTrackerDiagnosticCode::tracking_corrected:
      return "TRACKING_CORRECTED";
    case CableTrackerDiagnosticCode::observation_interrupted:
      return "OBSERVATION_INTERRUPTED";
    case CableTrackerDiagnosticCode::state_lost:
      return "STATE_LOST";
    case CableTrackerDiagnosticCode::tracker_input_invalid:
      return "TRACKER_INPUT_INVALID";
    case CableTrackerDiagnosticCode::executed_segment_discontinuity:
      return "EXECUTED_SEGMENT_DISCONTINUITY";
    case CableTrackerDiagnosticCode::touchdown_observation_discontinuity:
      return "TOUCHDOWN_OBSERVATION_DISCONTINUITY";
    case CableTrackerDiagnosticCode::touchdown_observation_unreliable:
      return "TOUCHDOWN_OBSERVATION_UNRELIABLE";
  }
  return "UNKNOWN_CABLE_TRACKER_DIAGNOSTIC";
}

CableTrackerSnapshot CableStateTracker::update(
    const ExecutedRobotSegment& executed_segment,
    const CableTelemetry& telemetry,
    const std::optional<TouchdownObservation>& observation) {
  const ExecutedRobotSample* terminal = executed_segment.samples.empty()
                                            ? nullptr
                                            : &executed_segment.samples.back();
  const bool telemetry_invalid =
      telemetry.sequence_number == 0U ||
      !std::isfinite(telemetry.payout_speed_mps) ||
      !std::isfinite(telemetry.payout_acceleration_mps2) ||
      !std::isfinite(telemetry.tension_n) ||
      telemetry.timestamp.nanoseconds < 0 ||
      (terminal != nullptr &&
       (std::abs(telemetry.payout_speed_mps - terminal->ground_speed_mps) >
            config_.maximum_payout_speed_error_mps ||
        telemetry.tension_n < config_.minimum_tension_n ||
        telemetry.tension_n > config_.maximum_tension_n ||
        std::abs(telemetry.timestamp.nanoseconds -
                 terminal->pose.timestamp.nanoseconds) >
            config_.synchronization_tolerance.nanoseconds));
  const bool observation_invalid =
      observation.has_value() &&
      (!observation_is_finite(*observation) || terminal == nullptr ||
       (terminal != nullptr &&
        squared_distance(
            release_position(*terminal, config_.release_point_offset_m),
            observation->touchdown_position_m) <= 1.0e-18) ||
       std::abs(observation->timestamp.nanoseconds -
                terminal->pose.timestamp.nanoseconds) >
           config_.synchronization_tolerance.nanoseconds);
  const bool sequence_regressed =
      executed_segment.sequence_number <= last_executed_segment_sequence_ ||
      telemetry.sequence_number <= last_telemetry_sequence_ ||
      (observation.has_value() &&
       observation->sequence_number <= last_observation_sequence_);
  if (!segment_is_valid(executed_segment) || telemetry_invalid ||
      observation_invalid || sequence_regressed) {
    current_.status = CableTrackerStatus::input_invalid;
    current_.diagnostics = {
        {CableTrackerDiagnosticCode::tracker_input_invalid,
         "executed evidence is malformed",
         telemetry.timestamp, executed_segment.sequence_number,
         telemetry.sequence_number,
         observation.has_value()
             ? std::optional<std::uint64_t>{observation->sequence_number}
             : std::nullopt}};
    return current_;
  }
  if (last_executed_sample_.has_value() &&
      !same_boundary(*last_executed_sample_, executed_segment.samples.front())) {
    return transition_to_state_lost(
        {CableTrackerDiagnosticCode::executed_segment_discontinuity,
         "executed segment does not continue from the prior accepted boundary",
         telemetry.timestamp, executed_segment.sequence_number,
         telemetry.sequence_number,
         observation.has_value()
             ? std::optional<std::uint64_t>{observation->sequence_number}
             : std::nullopt});
  }
  if (!current_.state.has_value() && !observation.has_value()) {
    current_.status = CableTrackerStatus::initial_state_uncertain;
    current_.diagnostics = {
        {CableTrackerDiagnosticCode::initial_state_uncertain,
         "a confirmed touchdown observation is required to initialize tracking",
         telemetry.timestamp, executed_segment.sequence_number,
         telemetry.sequence_number, std::nullopt}};
    return current_;
  }

  const ExecutedRobotSample& terminal_sample = executed_segment.samples.back();
  if (!current_.state.has_value()) {
    const Vector2m release =
        release_position(terminal_sample, config_.release_point_offset_m);
    const double observed_distance_m =
        std::sqrt(squared_distance(release,
                                   observation->touchdown_position_m));
    if (std::abs(observed_distance_m - config_.touchdown_distance_m) >
        config_.maximum_touchdown_observation_residual_m) {
      current_.status = CableTrackerStatus::initial_state_uncertain;
      current_.state.reset();
      current_.diagnostics = {
          {CableTrackerDiagnosticCode::touchdown_observation_unreliable,
           "initial touchdown distance is outside the calibrated model residual",
           observation->timestamp, executed_segment.sequence_number,
           telemetry.sequence_number, observation->sequence_number}};
      return current_;
    }
  }
  CableState state;
  if (!current_.state.has_value()) {
    const Vector2m release =
        release_position(terminal_sample, config_.release_point_offset_m);
    const double cable_heading =
        std::atan2(release.y_m - observation->touchdown_position_m.y_m,
                   release.x_m - observation->touchdown_position_m.x_m);
    state.kind = CableStateKind::tracked;
    state.lag_angle_rad =
        normalize_angle_radians(cable_heading - terminal_sample.pose.heading_rad);
    state.lag_angle_variance_rad2 = config_.initial_lag_variance_rad2;
    initialize_memory_from_executed_history(state, executed_segment,
                                            *observation, config_);
  } else {
    state = *current_.state;
    for (std::size_t index = 1U; index < executed_segment.samples.size();
         ++index) {
      const ExecutedRobotSample& previous = executed_segment.samples[index - 1U];
      const ExecutedRobotSample& sample = executed_segment.samples[index];
      const double distance_m =
          sample.robot_arc_length_m - previous.robot_arc_length_m;
      const double heading_change_rad = normalize_angle_radians(
          sample.pose.heading_rad - previous.pose.heading_rad);
      const double curvature_per_m = heading_change_rad / distance_m;
      const double decay =
          std::exp(-distance_m / config_.direction_response_length_m);
      state.lag_angle_rad = normalize_angle_radians(
          state.lag_angle_rad * decay -
          curvature_per_m * config_.direction_response_length_m *
              (1.0 - decay));
      *state.lag_angle_variance_rad2 =
          decay * decay * *state.lag_angle_variance_rad2 +
          config_.process_variance_per_m_rad2 * distance_m;
      const Vector2m touchdown =
          touchdown_position(sample, config_.release_point_offset_m,
                             config_.touchdown_distance_m,
                             state.lag_angle_rad);
      append_touchdown_sample(state.laying_memory, touchdown,
                              config_.minimum_distinct_touchdown_distance_m);
    }
    if (observation.has_value()) {
      const Vector2m predicted_touchdown =
          state.laying_memory.trailing_support_samples.back()
              .touchdown_position_m;
      if (squared_distance(predicted_touchdown,
                           observation->touchdown_position_m) >
          config_.maximum_touchdown_observation_residual_m *
              config_.maximum_touchdown_observation_residual_m) {
        return transition_to_state_lost(
            {CableTrackerDiagnosticCode::touchdown_observation_discontinuity,
             "confirmed touchdown is discontinuous with the propagated state",
             observation->timestamp, executed_segment.sequence_number,
             telemetry.sequence_number, observation->sequence_number});
      }
      correct_from_observation(state, terminal_sample, *observation, config_);
    }
  }
  const double required_history_length_m =
      std::max(config_.support_evaluation_length_m,
               2.0 * config_.curvature_evaluation_spacing_m);
  normalize_memory(state.laying_memory, required_history_length_m);
  state.timestamp = terminal_sample.pose.timestamp;
  state.sequence_number = next_state_sequence_++;

  const bool observation_expired =
      !observation.has_value() && last_observation_timestamp_.has_value() &&
      terminal_sample.pose.timestamp.nanoseconds -
              last_observation_timestamp_->nanoseconds >
          config_.maximum_observation_gap.nanoseconds;
  const bool mechanical_history_complete =
      state.laying_memory.previous_distinct_touchdown_points_m.size() >= 2U &&
      state.laying_memory.retained_arc_length_m + 1.0e-12 >=
          required_history_length_m;
  if (mechanical_history_complete) {
    explicit_task_start_boundary_ = false;
  }
  current_.status =
      observation_expired
          ? CableTrackerStatus::observation_interrupted
          : (explicit_task_start_boundary_ || mechanical_history_complete
                 ? CableTrackerStatus::tracked
                 : CableTrackerStatus::initial_state_uncertain);
  current_.state = std::move(state);
  current_.diagnostics = {
      {observation_expired
           ? CableTrackerDiagnosticCode::observation_interrupted
           : (current_.status == CableTrackerStatus::initial_state_uncertain
                  ? CableTrackerDiagnosticCode::mechanical_history_incomplete
                  : (observation.has_value() &&
                             last_executed_sample_.has_value()
                         ? CableTrackerDiagnosticCode::tracking_corrected
                         : (last_executed_sample_.has_value()
                                ? CableTrackerDiagnosticCode::tracking_propagated
                                : CableTrackerDiagnosticCode::tracking_initialized))),
       observation_expired
           ? "touchdown observation age exceeds the configured maximum"
           : (current_.status == CableTrackerStatus::initial_state_uncertain
                  ? "mechanical history is incomplete outside a task start boundary"
                  : (last_executed_sample_.has_value()
                         ? "tracking propagated from executed motion and synchronized telemetry"
                         : "tracking initialized from executed evidence")),
       terminal_sample.pose.timestamp, executed_segment.sequence_number,
       telemetry.sequence_number,
       observation.has_value()
           ? std::optional<std::uint64_t>{observation->sequence_number}
           : std::nullopt}};
  last_executed_sample_ = terminal_sample;
  last_executed_segment_sequence_ = executed_segment.sequence_number;
  last_telemetry_sequence_ = telemetry.sequence_number;
  if (observation.has_value()) {
    last_observation_timestamp_ = observation->timestamp;
    last_observation_sequence_ = observation->sequence_number;
  }
  return current_;
}

CableTrackerSnapshot CableStateTracker::snapshot() const { return current_; }

CableTrackerSnapshot CableStateTracker::begin_new_task(
    const MonotonicTime timestamp) {
  if (timestamp.nanoseconds < 0) {
    throw std::invalid_argument("task start timestamp must be monotonic");
  }
  current_.status = CableTrackerStatus::initial_state_uncertain;
  current_.state.reset();
  current_.diagnostics = {
      {CableTrackerDiagnosticCode::task_start_uninitialized,
       "explicit task start awaits executed touchdown evidence", timestamp,
       last_executed_segment_sequence_, last_telemetry_sequence_,
       last_observation_sequence_ == 0U
           ? std::nullopt
           : std::optional<std::uint64_t>{last_observation_sequence_}}};
  last_executed_sample_.reset();
  last_observation_timestamp_.reset();
  explicit_task_start_boundary_ = true;
  return current_;
}

CableTrackerSnapshot CableStateTracker::mark_state_lost(
    const MonotonicTime timestamp, std::string reason) {
  if (timestamp.nanoseconds < 0 || reason.empty()) {
    throw std::invalid_argument(
        "state loss requires a monotonic timestamp and nonempty reason");
  }
  return transition_to_state_lost(
      {CableTrackerDiagnosticCode::state_lost,
       "cable tracking state lost: " + std::move(reason), timestamp,
       last_executed_segment_sequence_, last_telemetry_sequence_,
       last_observation_sequence_ == 0U
           ? std::nullopt
           : std::optional<std::uint64_t>{last_observation_sequence_}});
}

CableTrackerSnapshot CableStateTracker::transition_to_state_lost(
    CableTrackerDiagnostic diagnostic) {
  current_.status = CableTrackerStatus::state_lost;
  current_.state.reset();
  current_.diagnostics = {std::move(diagnostic)};
  last_executed_sample_.reset();
  last_observation_timestamp_.reset();
  explicit_task_start_boundary_ = false;
  return current_;
}

}  // namespace underwater_planner::core
