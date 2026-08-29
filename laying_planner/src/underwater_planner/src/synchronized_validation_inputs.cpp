#include "underwater_planner/core/synchronized_validation_inputs.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace underwater_planner::core {
namespace {

void add_issue(ValidationInputCaptureResult& result, const CaptureIssueCode code,
               std::string field, std::string message) {
  result.issues.push_back({code, std::move(field), std::move(message)});
}

bool known_sensor_mode(const SensorHealthMode mode) {
  switch (mode) {
    case SensorHealthMode::nominal:
    case SensorHealthMode::approved_degraded:
      return true;
  }
  return false;
}

void check_age(ValidationInputCaptureResult& result, const char* field,
               const MonotonicTime timestamp, const Duration maximum_age,
               const MonotonicTime now) {
  if (timestamp.nanoseconds < 0 || now.nanoseconds < timestamp.nanoseconds ||
      now.nanoseconds - timestamp.nanoseconds > maximum_age.nanoseconds) {
    add_issue(result, CaptureIssueCode::input_age_invalid, field,
              "timestamp is in the future or exceeds its maximum age");
  }
}

void add_validation_issues(ValidationInputCaptureResult& result,
                           const char* field,
                           const ValidationResult& validation) {
  for (const std::string& message : validation.issues) {
    add_issue(result, CaptureIssueCode::invalid_input, field, message);
  }
}

void add_snapshot_validation_issues(ValidationInputCaptureResult& result,
                                    const SnapshotValidation& validation) {
  for (const std::string& message : validation.issues) {
    add_issue(result, CaptureIssueCode::invalid_input, "planning_snapshot",
              message);
  }
}

bool missing_required_inputs(const ValidationInputFrame& frame,
                             ValidationInputCaptureResult& result) {
  const auto missing = [&result](const bool absent, const char* field) {
    if (absent) {
      add_issue(result, CaptureIssueCode::missing_input, field,
                "required synchronized input is missing");
    }
  };
  missing(!frame.robot_state.has_value(), "robot_state");
  missing(!frame.cable_state.has_value(), "cable_state");
  missing(!frame.reference_progress.has_value(), "reference_progress");
  missing(!frame.cable_telemetry.has_value(), "cable_telemetry");
  missing(!frame.execution_tracking_state.has_value(),
          "execution_tracking_state");
  missing(!frame.planning_snapshot.has_value(), "planning_snapshot");
  return !result.issues.empty();
}

void validate_sequences(const ValidationInputFrame& frame,
                        ValidationInputCaptureResult& result) {
  const auto require_sequence = [&result](const std::uint64_t sequence,
                                          const char* field) {
    if (sequence == 0U) {
      add_issue(result, CaptureIssueCode::invalid_sequence_number, field,
                "sequence number must be nonzero");
    }
  };
  require_sequence(frame.robot_state->sequence_number, "robot_state");
  require_sequence(frame.cable_state->sequence_number, "cable_state");
  require_sequence(frame.reference_progress->sequence_number,
                   "reference_progress");
  require_sequence(frame.cable_telemetry->sequence_number, "cable_telemetry");
  require_sequence(frame.execution_tracking_state->sequence_number,
                   "execution_tracking_state");
  require_sequence(frame.planning_snapshot->map.version.sequence_number,
                   "map");
}

void validate_payloads(const ValidationInputFrame& frame,
                       ValidationInputCaptureResult& result) {
  add_validation_issues(result, "robot_state", validate(*frame.robot_state));
  add_validation_issues(result, "cable_state", validate(*frame.cable_state));
  add_validation_issues(result, "reference_progress",
                        validate(*frame.reference_progress));
  add_snapshot_validation_issues(result, validate(*frame.planning_snapshot));

  const CableTelemetry& telemetry = *frame.cable_telemetry;
  if (!std::isfinite(telemetry.payout_speed_mps) ||
      !std::isfinite(telemetry.payout_acceleration_mps2) ||
      !std::isfinite(telemetry.tension_n) || telemetry.tension_n < 0.0) {
    add_issue(result, CaptureIssueCode::invalid_input, "cable_telemetry",
              "telemetry values must be finite and tension nonnegative");
  }
  if (telemetry.timestamp.nanoseconds < 0) {
    add_issue(result, CaptureIssueCode::invalid_input, "cable_telemetry",
              "timestamp must be monotonic");
  }

  const ExecutionTrackingState& tracking = *frame.execution_tracking_state;
  if (tracking.execution_profile_version == 0U ||
      tracking.execution_operating_envelope_version == 0U ||
      !std::isfinite(tracking.tracked_arc_length_m) ||
      tracking.tracked_arc_length_m < 0.0 ||
      !std::isfinite(tracking.ground_acceleration_mps2) ||
      tracking.timestamp.nanoseconds < 0) {
    add_issue(result, CaptureIssueCode::invalid_input,
              "execution_tracking_state",
              "tracking state requires finite progress, timestamp, and versions");
  }
}

void validate_ages_and_synchronization(
    const ValidationInputFrame& frame, const ValidationInputCaptureLimits& limits,
    const MonotonicTime now, ValidationInputCaptureResult& result) {
  check_age(result, "robot_state", frame.robot_state->pose.timestamp,
            limits.robot_state_max_age, now);
  check_age(result, "robot_curvature", frame.robot_state->curvature_timestamp,
            limits.robot_state_max_age, now);
  check_age(result, "cable_state", frame.cable_state->timestamp,
            limits.cable_state_max_age, now);
  check_age(result, "reference_progress", frame.reference_progress->timestamp,
            limits.reference_progress_max_age, now);
  check_age(result, "cable_telemetry", frame.cable_telemetry->timestamp,
            limits.cable_telemetry_max_age, now);
  check_age(result, "execution_tracking_state",
            frame.execution_tracking_state->timestamp,
            limits.execution_tracking_max_age, now);
  check_age(result, "map", frame.planning_snapshot->map.version.timestamp,
            limits.map_max_age, now);

  const std::array<std::int64_t, 6> state_timestamps{
      frame.robot_state->pose.timestamp.nanoseconds,
      frame.robot_state->curvature_timestamp.nanoseconds,
      frame.cable_state->timestamp.nanoseconds,
      frame.reference_progress->timestamp.nanoseconds,
      frame.cable_telemetry->timestamp.nanoseconds,
      frame.execution_tracking_state->timestamp.nanoseconds};
  const auto bounds = std::minmax_element(state_timestamps.begin(),
                                          state_timestamps.end());
  if (*bounds.first >= 0 &&
      static_cast<std::uint64_t>(*bounds.second) -
              static_cast<std::uint64_t>(*bounds.first) >
          static_cast<std::uint64_t>(limits.synchronization_tolerance.nanoseconds)) {
    add_issue(result, CaptureIssueCode::inputs_not_synchronized, "timestamps",
              "state inputs exceed the synchronization tolerance");
  }
}

void validate_tracker_receipt(const TrackerUpdateReceipt& receipt,
                              const ValidationInputFrame& frame,
                              ValidationInputCaptureResult& result) {
  if (receipt.evidence_batch_sequence == 0U ||
      receipt.executed_motion_sequence == 0U ||
      (receipt.touchdown_observation_sequence.has_value() &&
       *receipt.touchdown_observation_sequence == 0U)) {
    add_issue(result, CaptureIssueCode::tracker_evidence_mismatch,
              "tracker_update_receipt",
              "tracker evidence sequences must be nonzero when present");
  }
  if (receipt.cable_telemetry_sequence !=
          frame.cable_telemetry->sequence_number ||
      receipt.resulting_cable_state_sequence !=
          frame.cable_state->sequence_number ||
      receipt.resulting_reference_progress_sequence !=
          frame.reference_progress->sequence_number) {
    add_issue(result, CaptureIssueCode::tracker_evidence_mismatch,
              "tracker_update_receipt",
              "telemetry and both tracker outputs must match one evidence batch receipt");
  }
}

void validate_dependencies(const ValidationInputFrame& frame,
                           ValidationInputCaptureResult& result) {
  const PlanningDependencyVersions& versions = frame.dependencies;
  if (versions.map_version.map_id.empty() ||
      versions.map_version.sequence_number == 0U ||
      versions.map_version.timestamp.nanoseconds < 0 ||
      versions.map_version.coordinate_frame.empty() ||
      versions.reference_line_version == 0U ||
      versions.robot_operating_area_version == 0U ||
      versions.cable_corridor_version == 0U ||
      versions.terrain_gradient_policy_version == 0U ||
      versions.corridor_risk_policy_version == 0U ||
      versions.cable_model_version == 0U ||
      versions.uncertainty_envelope_version == 0U ||
      versions.uncertainty_envelope_generator_version == 0U ||
      versions.execution_operating_envelope_version == 0U ||
      versions.execution_profile_version == 0U ||
      versions.operating_domain_id.empty() ||
      !known_sensor_mode(versions.sensor_mode)) {
    add_issue(result, CaptureIssueCode::incomplete_dependencies, "dependencies",
              "the complete version, sensor mode, and operating-domain tuple is required");
  }

  const VersionedPlanningSnapshot& snapshot = *frame.planning_snapshot;
  if (versions.map_version != snapshot.map.version ||
      versions.reference_line_version != snapshot.reference_line.version ||
      versions.robot_operating_area_version !=
          snapshot.robot_operating_area.version ||
      versions.cable_corridor_version != snapshot.cable_corridor.version) {
    add_issue(result, CaptureIssueCode::version_mismatch, "dependencies",
              "map, reference line, robot operating area, or cable corridor version mismatches the snapshot");
  }
  if (frame.reference_progress->reference_line_version !=
      versions.reference_line_version) {
    add_issue(result, CaptureIssueCode::version_mismatch, "reference_progress",
              "reference progress is bound to another reference line version");
  }
  const ExecutionTrackingState& tracking = *frame.execution_tracking_state;
  if (tracking.execution_profile_version != versions.execution_profile_version ||
      tracking.execution_operating_envelope_version !=
          versions.execution_operating_envelope_version) {
    add_issue(result, CaptureIssueCode::version_mismatch,
              "execution_tracking_state",
              "tracking state is bound to another profile or execution envelope");
  }
}

}  // namespace

bool operator==(const CaptureIssue& left, const CaptureIssue& right) noexcept {
  return left.code == right.code && left.field == right.field &&
         left.message == right.message;
}

SynchronizedValidationInputCapturer::SynchronizedValidationInputCapturer(
    ValidationInputCaptureLimits limits)
    : limits_(limits) {
  const std::array<Duration, 7> values{
      limits_.robot_state_max_age,
      limits_.cable_state_max_age,
      limits_.reference_progress_max_age,
      limits_.cable_telemetry_max_age,
      limits_.execution_tracking_max_age,
      limits_.map_max_age,
      limits_.synchronization_tolerance};
  if (std::any_of(values.begin(), values.end(),
                  [](const Duration value) { return value.nanoseconds < 0; })) {
    throw std::invalid_argument("capture age and synchronization limits must be nonnegative");
  }
}

ValidationInputCaptureResult SynchronizedValidationInputCapturer::capture(
    ValidationInputSource& source, const MonotonicTime now) {
  const std::lock_guard<std::mutex> capture_lock(capture_mutex_);
  ValidationInputCaptureResult result;
  if (now.nanoseconds < 0) {
    add_issue(result, CaptureIssueCode::input_age_invalid, "captured_at",
              "capture time must be monotonic");
    return result;
  }

  const std::optional<TrackerSynchronizedFrame> captured_frame =
      source.advance_trackers_and_capture_frame();
  if (!captured_frame.has_value()) {
    add_issue(result, CaptureIssueCode::tracker_update_failed,
              "tracker_update_receipt",
              "the source could not atomically advance trackers and freeze one frame");
    return result;
  }

  const std::uint64_t revision_before = captured_frame->source_revision;
  const ValidationInputFrame& frame = captured_frame->frame;
  const std::uint64_t revision_after = source.revision();
  if (revision_before == 0U || revision_before != revision_after) {
    add_issue(result, CaptureIssueCode::source_changed_during_capture,
              "source_revision",
              "input source changed while the validation frame was being copied");
    return result;
  }
  if (missing_required_inputs(frame, result)) return result;

  validate_sequences(frame, result);
  validate_payloads(frame, result);
  validate_ages_and_synchronization(frame, limits_, now, result);
  validate_tracker_receipt(captured_frame->tracker_update_receipt, frame, result);
  validate_dependencies(frame, result);
  SequenceWatermarks current_sequences{
      revision_before,
      frame.robot_state->sequence_number,
      frame.cable_state->sequence_number,
      frame.reference_progress->sequence_number,
      frame.cable_telemetry->sequence_number,
      frame.execution_tracking_state->sequence_number,
      frame.planning_snapshot->map.version.sequence_number,
      captured_frame->tracker_update_receipt.evidence_batch_sequence,
      captured_frame->tracker_update_receipt.executed_motion_sequence,
      captured_frame->tracker_update_receipt.touchdown_observation_sequence};
  if (last_successful_sequences_.has_value()) {
    const SequenceWatermarks& previous = *last_successful_sequences_;
    if (current_sequences.source_revision < previous.source_revision ||
        current_sequences.robot_state < previous.robot_state ||
        current_sequences.cable_state < previous.cable_state ||
        current_sequences.reference_progress < previous.reference_progress ||
        current_sequences.cable_telemetry < previous.cable_telemetry ||
        current_sequences.execution_tracking < previous.execution_tracking ||
        current_sequences.map < previous.map ||
        current_sequences.evidence_batch < previous.evidence_batch ||
        current_sequences.executed_motion < previous.executed_motion ||
        (current_sequences.touchdown_observation.has_value() &&
         previous.touchdown_observation.has_value() &&
         *current_sequences.touchdown_observation <
             *previous.touchdown_observation)) {
      add_issue(result, CaptureIssueCode::sequence_regression, "sequences",
                "an input sequence regressed below the last successful capture");
    }
    if (!current_sequences.touchdown_observation.has_value()) {
      current_sequences.touchdown_observation =
          previous.touchdown_observation;
    }
  }
  if (!result.issues.empty()) return result;

  last_successful_sequences_ = current_sequences;
  result.status = ValidationInputCaptureStatus::captured;
  result.inputs = SynchronizedValidationInputs{
      now,
      revision_before,
      *frame.robot_state,
      *frame.cable_state,
      *frame.reference_progress,
      *frame.cable_telemetry,
      *frame.execution_tracking_state,
      *frame.planning_snapshot,
      frame.dependencies,
      captured_frame->tracker_update_receipt,
      PredictionMode::validation};
  return result;
}

}  // namespace underwater_planner::core
