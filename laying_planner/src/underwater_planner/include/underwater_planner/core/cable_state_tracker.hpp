#pragma once

#include "underwater_planner/core/data_contract.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace underwater_planner::core {

struct CableStateTrackerConfig {
  std::uint64_t cable_model_version{};
  std::string calibration_dataset_id;
  std::string operating_domain_id;
  Vector2m release_point_offset_m;
  double touchdown_distance_m{};
  double direction_response_length_m{};
  double curvature_evaluation_spacing_m{};
  double support_evaluation_length_m{};
  double minimum_distinct_touchdown_distance_m{};
  double initial_lag_variance_rad2{};
  double process_variance_per_m_rad2{};
  double maximum_touchdown_observation_residual_m{};
  Duration maximum_observation_gap;
  Duration synchronization_tolerance;
  double maximum_payout_speed_error_mps{};
  double minimum_tension_n{};
  double maximum_tension_n{};
};

struct ExecutedRobotSample {
  double robot_arc_length_m{};
  Pose2d pose;
  double ground_speed_mps{};
};

struct ExecutedRobotSegment {
  std::uint64_t sequence_number{};
  std::vector<ExecutedRobotSample> samples;
};

struct TouchdownObservation {
  Vector2m touchdown_position_m;
  Covariance2dM2 position_covariance_m2;
  MonotonicTime timestamp;
  std::uint64_t sequence_number{};
};

enum class CableTrackerStatus {
  tracked,
  initial_state_uncertain,
  observation_interrupted,
  state_lost,
  input_invalid,
};

enum class CableTrackerDiagnosticCode {
  initial_state_uncertain,
  task_start_uninitialized,
  mechanical_history_incomplete,
  tracking_initialized,
  tracking_propagated,
  tracking_corrected,
  observation_interrupted,
  state_lost,
  tracker_input_invalid,
  executed_segment_discontinuity,
  touchdown_observation_discontinuity,
  touchdown_observation_unreliable,
};

[[nodiscard]] std::string_view to_string(CableTrackerDiagnosticCode code);

struct CableTrackerDiagnostic {
  CableTrackerDiagnosticCode code{
      CableTrackerDiagnosticCode::initial_state_uncertain};
  std::string message;
  MonotonicTime timestamp;
  std::uint64_t executed_segment_sequence{};
  std::uint64_t telemetry_sequence{};
  std::optional<std::uint64_t> touchdown_observation_sequence;
};

struct CableTrackerSnapshot {
  CableTrackerStatus status{CableTrackerStatus::initial_state_uncertain};
  std::optional<CableState> state;
  std::uint64_t cable_model_version{};
  std::string calibration_dataset_id;
  std::string operating_domain_id;
  std::string risk_semantics;
  std::vector<CableTrackerDiagnostic> diagnostics;

  [[nodiscard]] bool usable_for_planning() const noexcept {
    return status == CableTrackerStatus::tracked && state.has_value();
  }
};

class CableStateTracker {
 public:
  explicit CableStateTracker(CableStateTrackerConfig config);

  [[nodiscard]] CableTrackerSnapshot update(
      const ExecutedRobotSegment& executed_segment,
      const CableTelemetry& telemetry,
      const std::optional<TouchdownObservation>& observation);

  [[nodiscard]] CableTrackerSnapshot snapshot() const;

  [[nodiscard]] CableTrackerSnapshot begin_new_task(MonotonicTime timestamp);

  [[nodiscard]] CableTrackerSnapshot mark_state_lost(
      MonotonicTime timestamp, std::string reason);

 private:
  [[nodiscard]] CableTrackerSnapshot transition_to_state_lost(
      CableTrackerDiagnostic diagnostic);

  CableStateTrackerConfig config_;
  CableTrackerSnapshot current_;
  std::uint64_t next_state_sequence_{1};
  std::optional<ExecutedRobotSample> last_executed_sample_;
  std::optional<MonotonicTime> last_observation_timestamp_;
  std::uint64_t last_executed_segment_sequence_{};
  std::uint64_t last_telemetry_sequence_{};
  std::uint64_t last_observation_sequence_{};
  bool explicit_task_start_boundary_{};
};

}  // namespace underwater_planner::core
