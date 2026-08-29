#include "underwater_planner/core/synchronized_validation_inputs.hpp"

#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace {

using underwater_planner::core::CableState;
using underwater_planner::core::CableStateKind;
using underwater_planner::core::CableTelemetry;
using underwater_planner::core::CaptureIssueCode;
using underwater_planner::core::ExecutionTrackingState;
using underwater_planner::core::MapCell;
using underwater_planner::core::MonotonicTime;
using underwater_planner::core::PredictionMode;
using underwater_planner::core::ReferenceProgress;
using underwater_planner::core::RobotState;
using underwater_planner::core::SensorHealthMode;
using underwater_planner::core::SynchronizedValidationInputCapturer;
using underwater_planner::core::TrackerSynchronizedFrame;
using underwater_planner::core::TrackerUpdateReceipt;
using underwater_planner::core::ValidationInputCaptureLimits;
using underwater_planner::core::ValidationInputCaptureResult;
using underwater_planner::core::ValidationInputCaptureStatus;
using underwater_planner::core::ValidationInputFrame;
using underwater_planner::core::ValidationInputSource;
using underwater_planner::core::Vector2m;
using underwater_planner::core::VersionedPlanningSnapshot;

void require(const bool condition, const char* message) {
  if (!condition) {
    std::cerr << "T05 failure: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

bool has_issue(const ValidationInputCaptureResult& result,
               const CaptureIssueCode code) {
  for (const auto& issue : result.issues) {
    if (issue.code == code) return true;
  }
  return false;
}

VersionedPlanningSnapshot make_planning_snapshot() {
  using namespace underwater_planner::core;
  VersionedPlanningSnapshot snapshot;
  snapshot.map.version = {"map-v1", 7, {970}, "map-frame"};
  snapshot.map.width = 2;
  snapshot.map.height = 2;
  snapshot.map.resolution_m = 1.0;
  snapshot.map.derived_configuration_version = 5;
  snapshot.map.cells.assign(4, MapCell{0.0, 0.01, 1.0, true});
  snapshot.reference_line = make_reference_line(
      11, "map-frame", std::vector<Vector2m>{{0.0, 0.0}, {1.0, 0.0}});
  snapshot.robot_operating_area =
      {13, "robot-area-v13", {{-1, -1}, {2, -1}, {2, 1}, {-1, 1}}};
  snapshot.cable_corridor =
      {17, "cable-corridor-v17", {{-1, -0.5}, {2, -0.5}, {2, 0.5}, {-1, 0.5}}};
  return snapshot;
}

ValidationInputFrame make_frame() {
  ValidationInputFrame frame;
  frame.robot_state = RobotState{{0.0, 0.0, 0.0, {995}}, 0.2, 0.01, {994}, 21};
  frame.cable_state =
      CableState{CableStateKind::tracked, 0.02, 0.001, {993}, {}, 22};
  frame.reference_progress = ReferenceProgress{11, 0.4, {992}, 23};
  frame.cable_telemetry = CableTelemetry{0.21, 0.0, 30.0, {991}, 24};
  frame.execution_tracking_state =
      ExecutionTrackingState{31, 29, 0.4, {990}, 25};
  frame.planning_snapshot = make_planning_snapshot();
  frame.dependencies.map_version = frame.planning_snapshot->map.version;
  frame.dependencies.reference_line_version = 11;
  frame.dependencies.robot_operating_area_version = 13;
  frame.dependencies.terrain_gradient_policy_version = 19;
  frame.dependencies.corridor_risk_policy_version = 20;
  frame.dependencies.cable_model_version = 23;
  frame.dependencies.uncertainty_envelope_version = 27;
  frame.dependencies.uncertainty_envelope_generator_version = 28;
  frame.dependencies.execution_operating_envelope_version = 29;
  frame.dependencies.execution_profile_version = 31;
  frame.dependencies.sensor_mode = SensorHealthMode::nominal;
  frame.dependencies.operating_domain_id = "tank-domain-v1";
  frame.dependencies.cable_corridor_version =
      frame.planning_snapshot->cable_corridor.version;
  return frame;
}

ValidationInputCaptureLimits make_limits() {
  using underwater_planner::core::Duration;
  return {{100}, {100}, {100}, {100}, {100}, {100}, {10}};
}

class InMemoryInputSource final : public ValidationInputSource {
 public:
  explicit InMemoryInputSource(ValidationInputFrame initial)
      : frame(std::move(initial)), tracker_receipt(TrackerUpdateReceipt{
                                       61,
                                       60,
                                       59,
                                       frame.cable_telemetry->sequence_number,
                                       frame.cable_state->sequence_number,
                                       frame.reference_progress->sequence_number}) {}

  std::uint64_t revision() const noexcept override { return revision_number; }

  std::optional<TrackerSynchronizedFrame>
  advance_trackers_and_capture_frame() override {
    if (!tracker_receipt.has_value()) return std::nullopt;
    const TrackerSynchronizedFrame result{
        revision_number, *tracker_receipt, frame};
    if (change_during_read) {
      ++revision_number;
      if (change_during_read_mutation) {
        change_during_read_mutation(frame);
      } else if (frame.robot_state.has_value()) {
        ++frame.robot_state->sequence_number;
      }
    }
    return result;
  }

  mutable ValidationInputFrame frame;
  std::optional<TrackerUpdateReceipt> tracker_receipt;
  mutable std::uint64_t revision_number{41};
  bool change_during_read{};
  std::function<void(ValidationInputFrame&)> change_during_read_mutation;
};

ValidationInputCaptureResult capture(
    InMemoryInputSource& source,
    SynchronizedValidationInputCapturer& capturer) {
  return capturer.capture(source, MonotonicTime{1000});
}

ValidationInputCaptureResult capture(InMemoryInputSource& source) {
  SynchronizedValidationInputCapturer capturer(make_limits());
  return capture(source, capturer);
}

struct InvalidFrameCase {
  const char* name;
  void (*change)(ValidationInputFrame&);
};

void require_invalid_frame_cases(const std::vector<InvalidFrameCase>& cases,
                                 const CaptureIssueCode expected_issue) {
  for (const auto& item : cases) {
    InMemoryInputSource source(make_frame());
    item.change(source.frame);
    const auto result = capture(source);
    require(result.status ==
                ValidationInputCaptureStatus::validation_context_invalid,
            item.name);
    require(has_issue(result, expected_issue),
            "invalid frame omitted its structured reason");
  }
}

void valid_frame_is_frozen_for_validation() {
  InMemoryInputSource source(make_frame());
  const auto result = capture(source);
  require(result.status == ValidationInputCaptureStatus::captured,
          "valid synchronized inputs were rejected");
  require(result.inputs.has_value(), "successful capture omitted its snapshot");
  require(result.inputs->source_revision == 41,
          "capture did not bind the source revision");
  require(result.inputs->cable_context_mode == PredictionMode::validation,
          "captured cable context was not validation-only");
  require(result.inputs->tracker_update_receipt.evidence_batch_sequence == 61,
          "capture omitted same-batch tracker provenance");
  source.frame.robot_state->sequence_number = 99;
  require(result.inputs->robot_state.sequence_number == 21,
          "captured inputs changed with their asynchronous source");
}

void a_mid_capture_change_rejects_the_whole_snapshot() {
  // Design: 18.2.7-16
  using Mutation = std::function<void(ValidationInputFrame&)>;
  const std::vector<Mutation> mutations{
      [](ValidationInputFrame& frame) {
        ++frame.robot_state->sequence_number;
      },
      [](ValidationInputFrame& frame) {
        ++frame.cable_telemetry->sequence_number;
      },
      [](ValidationInputFrame& frame) {
        ++frame.execution_tracking_state->sequence_number;
      },
      [](ValidationInputFrame& frame) {
        ++frame.dependencies.map_version.sequence_number;
      },
      [](ValidationInputFrame& frame) {
        ++frame.dependencies.reference_line_version;
      },
      [](ValidationInputFrame& frame) {
        ++frame.dependencies.robot_operating_area_version;
      },
      [](ValidationInputFrame& frame) {
        ++frame.dependencies.cable_corridor_version;
      },
      [](ValidationInputFrame& frame) {
        ++frame.dependencies.terrain_gradient_policy_version;
      },
      [](ValidationInputFrame& frame) {
        ++frame.dependencies.corridor_risk_policy_version;
      },
      [](ValidationInputFrame& frame) {
        ++frame.dependencies.cable_model_version;
      },
      [](ValidationInputFrame& frame) {
        ++frame.dependencies.uncertainty_envelope_version;
      },
      [](ValidationInputFrame& frame) {
        ++frame.dependencies.uncertainty_envelope_generator_version;
      },
      [](ValidationInputFrame& frame) {
        ++frame.dependencies.execution_operating_envelope_version;
      },
      [](ValidationInputFrame& frame) {
        ++frame.dependencies.execution_profile_version;
      },
      [](ValidationInputFrame& frame) {
        frame.dependencies.sensor_mode = SensorHealthMode::approved_degraded;
      },
      [](ValidationInputFrame& frame) {
        frame.dependencies.operating_domain_id = "changed-domain";
      },
  };

  for (const Mutation& mutate : mutations) {
    InMemoryInputSource first_source(make_frame());
    first_source.change_during_read = true;
    first_source.change_during_read_mutation = mutate;
    InMemoryInputSource second_source(make_frame());
    second_source.change_during_read = true;
    second_source.change_during_read_mutation = mutate;
    const auto first = capture(first_source);
    const auto second = capture(second_source);
    require(first.status ==
                ValidationInputCaptureStatus::validation_context_invalid,
            "mid-capture change was accepted");
    require(!first.inputs.has_value(),
            "invalid capture returned stitched inputs");
    require(has_issue(first, CaptureIssueCode::source_changed_during_capture),
            "mid-capture change omitted its structured reason");
    require(second.status == first.status && second.issues == first.issues,
            "capture race did not reproduce deterministically");
  }
}

void stale_or_future_inputs_are_rejected_by_class() {
  // Design: 18.2.7-17
  const std::vector<InvalidFrameCase> cases{
      {"robot state", [](ValidationInputFrame& f) { f.robot_state->pose.timestamp = {899}; }},
      {"robot curvature", [](ValidationInputFrame& f) { f.robot_state->curvature_timestamp = {899}; }},
      {"cable state", [](ValidationInputFrame& f) { f.cable_state->timestamp = {899}; }},
      {"reference progress", [](ValidationInputFrame& f) { f.reference_progress->timestamp = {899}; }},
      {"cable telemetry", [](ValidationInputFrame& f) { f.cable_telemetry->timestamp = {899}; }},
      {"execution tracking", [](ValidationInputFrame& f) { f.execution_tracking_state->timestamp = {899}; }},
      {"map", [](ValidationInputFrame& f) { f.planning_snapshot->map.version.timestamp = {869};
                                              f.dependencies.map_version.timestamp = {869}; }},
      {"future robot state", [](ValidationInputFrame& f) { f.robot_state->pose.timestamp = {1001}; }},
  };
  require_invalid_frame_cases(cases, CaptureIssueCode::input_age_invalid);
}

void timestamps_must_be_synchronized() {
  InMemoryInputSource source(make_frame());
  source.frame.execution_tracking_state->timestamp = {981};
  const auto result = capture(source);
  require(result.status == ValidationInputCaptureStatus::validation_context_invalid,
          "inputs beyond synchronization tolerance were accepted");
  require(has_issue(result, CaptureIssueCode::inputs_not_synchronized),
          "synchronization failure omitted its structured reason");
}

void every_stream_requires_a_sequence_number() {
  const std::vector<InvalidFrameCase> cases{
      {"robot state", [](ValidationInputFrame& f) { f.robot_state->sequence_number = 0; }},
      {"cable state", [](ValidationInputFrame& f) { f.cable_state->sequence_number = 0; }},
      {"reference progress", [](ValidationInputFrame& f) { f.reference_progress->sequence_number = 0; }},
      {"cable telemetry", [](ValidationInputFrame& f) { f.cable_telemetry->sequence_number = 0; }},
      {"execution tracking", [](ValidationInputFrame& f) { f.execution_tracking_state->sequence_number = 0; }},
      {"map", [](ValidationInputFrame& f) { f.planning_snapshot->map.version.sequence_number = 0;
                                             f.dependencies.map_version.sequence_number = 0; }},
  };
  require_invalid_frame_cases(cases, CaptureIssueCode::invalid_sequence_number);
}

void trackers_must_advance_from_one_evidence_batch() {
  InMemoryInputSource missing(make_frame());
  missing.tracker_receipt.reset();
  const auto missing_result = capture(missing);
  require(missing_result.status ==
              ValidationInputCaptureStatus::validation_context_invalid,
          "capture accepted states without a tracker update receipt");
  require(has_issue(missing_result, CaptureIssueCode::tracker_update_failed),
          "missing tracker update omitted its structured reason");

  InMemoryInputSource mismatched(make_frame());
  mismatched.tracker_receipt->resulting_reference_progress_sequence = 99;
  const auto mismatch_result = capture(mismatched);
  require(mismatch_result.status ==
              ValidationInputCaptureStatus::validation_context_invalid,
          "capture accepted tracker states from different evidence batches");
  require(has_issue(mismatch_result,
                    CaptureIssueCode::tracker_evidence_mismatch),
          "tracker evidence mismatch omitted its structured reason");
}

void successful_sequence_watermarks_reject_regression() {
  SynchronizedValidationInputCapturer capturer(make_limits());
  InMemoryInputSource source(make_frame());
  require(capture(source, capturer).status ==
              ValidationInputCaptureStatus::captured,
          "initial sequence watermark capture failed");
  require(capture(source, capturer).status ==
              ValidationInputCaptureStatus::captured,
          "an unchanged but fresh input frame was treated as a replay");

  ++source.revision_number;
  source.frame.robot_state->sequence_number = 20;
  const auto regressed = capture(source, capturer);
  require(regressed.status ==
              ValidationInputCaptureStatus::validation_context_invalid,
          "regressed robot sequence was accepted");
  require(has_issue(regressed, CaptureIssueCode::sequence_regression),
          "sequence regression omitted its structured reason");

  ++source.revision_number;
  source.frame.robot_state->sequence_number = 21;
  source.tracker_receipt->evidence_batch_sequence = 58;
  const auto evidence_regressed = capture(source, capturer);
  require(evidence_regressed.status ==
              ValidationInputCaptureStatus::validation_context_invalid,
          "regressed executed-evidence batch was accepted");
  require(has_issue(evidence_regressed, CaptureIssueCode::sequence_regression),
          "evidence regression omitted its structured reason");

  ++source.revision_number;
  source.tracker_receipt->evidence_batch_sequence = 61;
  source.tracker_receipt->touchdown_observation_sequence = 58;
  const auto observation_regressed = capture(source, capturer);
  require(observation_regressed.status ==
              ValidationInputCaptureStatus::validation_context_invalid,
          "regressed touchdown-observation sequence was accepted");
  require(has_issue(observation_regressed, CaptureIssueCode::sequence_regression),
          "touchdown observation regression omitted its structured reason");
}

void tracking_state_is_required_and_profile_bound() {
  // Design: 18.2.7-17
  InMemoryInputSource missing(make_frame());
  missing.frame.execution_tracking_state.reset();
  const auto missing_result = capture(missing);
  require(missing_result.status ==
              ValidationInputCaptureStatus::validation_context_invalid,
          "missing execution tracking state was accepted");
  require(has_issue(missing_result, CaptureIssueCode::missing_input),
          "missing tracking state omitted its structured reason");

  InMemoryInputSource mismatched(make_frame());
  mismatched.frame.execution_tracking_state->execution_profile_version = 30;
  const auto mismatch_result = capture(mismatched);
  require(mismatch_result.status ==
              ValidationInputCaptureStatus::validation_context_invalid,
          "tracking state from another profile was accepted");
  require(has_issue(mismatch_result, CaptureIssueCode::version_mismatch),
          "tracking mismatch omitted its structured reason");

  InMemoryInputSource nonfinite(make_frame());
  nonfinite.frame.execution_tracking_state->ground_acceleration_mps2 =
      std::numeric_limits<double>::quiet_NaN();
  const auto nonfinite_result = capture(nonfinite);
  require(nonfinite_result.status ==
              ValidationInputCaptureStatus::validation_context_invalid &&
              has_issue(nonfinite_result, CaptureIssueCode::invalid_input),
          "nonfinite tracked ground acceleration was accepted");
}

void dependencies_must_be_complete_and_match_the_snapshot() {
  InMemoryInputSource incomplete(make_frame());
  incomplete.frame.dependencies.cable_model_version = 0;
  const auto incomplete_result = capture(incomplete);
  require(incomplete_result.status ==
              ValidationInputCaptureStatus::validation_context_invalid,
          "incomplete model version tuple was accepted");
  require(has_issue(incomplete_result, CaptureIssueCode::incomplete_dependencies),
          "incomplete tuple omitted its structured reason");

  InMemoryInputSource mismatched(make_frame());
  mismatched.frame.dependencies.reference_line_version = 12;
  const auto mismatch_result = capture(mismatched);
  require(mismatch_result.status ==
              ValidationInputCaptureStatus::validation_context_invalid,
          "mismatched reference version was accepted");
  require(has_issue(mismatch_result, CaptureIssueCode::version_mismatch),
          "version mismatch omitted its structured reason");

  InMemoryInputSource corridor_mismatched(make_frame());
  ++corridor_mismatched.frame.dependencies.cable_corridor_version;
  const auto corridor_result = capture(corridor_mismatched);
  require(corridor_result.status ==
              ValidationInputCaptureStatus::validation_context_invalid &&
              has_issue(corridor_result, CaptureIssueCode::version_mismatch),
          "mismatched cable corridor version was accepted");
}

}  // namespace

int main() {
  valid_frame_is_frozen_for_validation();
  a_mid_capture_change_rejects_the_whole_snapshot();
  stale_or_future_inputs_are_rejected_by_class();
  timestamps_must_be_synchronized();
  every_stream_requires_a_sequence_number();
  trackers_must_advance_from_one_evidence_batch();
  successful_sequence_watermarks_reject_regression();
  tracking_state_is_required_and_profile_bound();
  dependencies_must_be_complete_and_match_the_snapshot();
  std::cout << "T05 synchronized validation input checks passed\n";
}
