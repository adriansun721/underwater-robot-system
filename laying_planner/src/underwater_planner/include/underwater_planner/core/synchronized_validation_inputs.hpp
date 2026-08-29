#pragma once

#include "underwater_planner/core/data_contract.hpp"
#include "underwater_planner/core/versioned_snapshot.hpp"

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace underwater_planner::core {

struct ExecutionTrackingState {
  std::uint64_t execution_profile_version{};
  std::uint64_t execution_operating_envelope_version{};
  double tracked_arc_length_m{};
  MonotonicTime timestamp;
  std::uint64_t sequence_number{};
  double ground_acceleration_mps2{};
};

struct TrackerUpdateReceipt {
  std::uint64_t evidence_batch_sequence{};
  std::uint64_t executed_motion_sequence{};
  std::optional<std::uint64_t> touchdown_observation_sequence;
  std::uint64_t cable_telemetry_sequence{};
  std::uint64_t resulting_cable_state_sequence{};
  std::uint64_t resulting_reference_progress_sequence{};
};

struct ValidationInputFrame {
  std::optional<RobotState> robot_state;
  std::optional<CableState> cable_state;
  std::optional<ReferenceProgress> reference_progress;
  std::optional<CableTelemetry> cable_telemetry;
  std::optional<ExecutionTrackingState> execution_tracking_state;
  std::optional<VersionedPlanningSnapshot> planning_snapshot;
  PlanningDependencyVersions dependencies;
};

struct TrackerSynchronizedFrame {
  std::uint64_t source_revision{};
  TrackerUpdateReceipt tracker_update_receipt;
  ValidationInputFrame frame;
};

class ValidationInputSource {
 public:
  virtual ~ValidationInputSource() = default;

  // Under one source-side lock, advances both trackers from one executed-
  // evidence batch and freezes every state and dependency field. The returned
  // revision is the linearization point for the complete frame.
  [[nodiscard]] virtual std::optional<TrackerSynchronizedFrame>
  advance_trackers_and_capture_frame() = 0;

  // The source must increment this revision for every state or dependency
  // update that could affect a validation decision.
  [[nodiscard]] virtual std::uint64_t revision() const noexcept = 0;
};

struct ValidationInputCaptureLimits {
  Duration robot_state_max_age;
  Duration cable_state_max_age;
  Duration reference_progress_max_age;
  Duration cable_telemetry_max_age;
  Duration execution_tracking_max_age;
  Duration map_max_age;
  Duration synchronization_tolerance;
};

enum class CaptureIssueCode {
  source_changed_during_capture,
  missing_input,
  invalid_input,
  invalid_sequence_number,
  input_age_invalid,
  inputs_not_synchronized,
  tracker_update_failed,
  tracker_evidence_mismatch,
  sequence_regression,
  incomplete_dependencies,
  version_mismatch,
};

struct CaptureIssue {
  CaptureIssueCode code{CaptureIssueCode::invalid_input};
  std::string field;
  std::string message;
};

[[nodiscard]] bool operator==(const CaptureIssue& left,
                              const CaptureIssue& right) noexcept;

struct SynchronizedValidationInputs {
  MonotonicTime captured_at;
  std::uint64_t source_revision{};
  RobotState robot_state;
  CableState cable_state;
  ReferenceProgress reference_progress;
  CableTelemetry cable_telemetry;
  ExecutionTrackingState execution_tracking_state;
  VersionedPlanningSnapshot planning_snapshot;
  PlanningDependencyVersions dependencies;
  TrackerUpdateReceipt tracker_update_receipt;
  PredictionMode cable_context_mode{PredictionMode::validation};
};

enum class ValidationInputCaptureStatus {
  captured,
  validation_context_invalid,
};

struct ValidationInputCaptureResult {
  ValidationInputCaptureStatus status{
      ValidationInputCaptureStatus::validation_context_invalid};
  std::optional<SynchronizedValidationInputs> inputs;
  std::vector<CaptureIssue> issues;
};

class SynchronizedValidationInputCapturer {
 public:
  explicit SynchronizedValidationInputCapturer(
      ValidationInputCaptureLimits limits);

  [[nodiscard]] ValidationInputCaptureResult capture(
      ValidationInputSource& source, MonotonicTime now);

 private:
  struct SequenceWatermarks {
    std::uint64_t source_revision{};
    std::uint64_t robot_state{};
    std::uint64_t cable_state{};
    std::uint64_t reference_progress{};
    std::uint64_t cable_telemetry{};
    std::uint64_t execution_tracking{};
    std::uint64_t map{};
    std::uint64_t evidence_batch{};
    std::uint64_t executed_motion{};
    std::optional<std::uint64_t> touchdown_observation;
  };

  ValidationInputCaptureLimits limits_;
  std::optional<SequenceWatermarks> last_successful_sequences_;
  std::mutex capture_mutex_;
};

}  // namespace underwater_planner::core
