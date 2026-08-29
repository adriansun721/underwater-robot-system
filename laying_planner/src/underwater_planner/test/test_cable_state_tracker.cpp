#include "underwater_planner/core/cable_state_tracker.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using underwater_planner::core::CableStateTracker;
using underwater_planner::core::CableStateTrackerConfig;
using underwater_planner::core::CableTelemetry;
using underwater_planner::core::CableTrackerDiagnosticCode;
using underwater_planner::core::CableTrackerStatus;
using underwater_planner::core::ExecutedRobotSample;
using underwater_planner::core::ExecutedRobotSegment;
using underwater_planner::core::TouchdownObservation;

void require(const bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

CableStateTrackerConfig tracker_config() {
  CableStateTrackerConfig config;
  config.cable_model_version = 17;
  config.calibration_dataset_id = "cable-tracker-cal/v3";
  config.operating_domain_id = "pool-a/v2";
  config.release_point_offset_m = {0.0, 0.0};
  config.touchdown_distance_m = 1.0;
  config.direction_response_length_m = 2.0;
  config.curvature_evaluation_spacing_m = 0.5;
  config.support_evaluation_length_m = 2.0;
  config.minimum_distinct_touchdown_distance_m = 0.05;
  config.initial_lag_variance_rad2 = 0.25;
  config.process_variance_per_m_rad2 = 0.01;
  config.maximum_touchdown_observation_residual_m = 2.0;
  config.maximum_observation_gap = {2'000'000'000};
  config.synchronization_tolerance = {50'000'000};
  config.maximum_payout_speed_error_mps = 0.1;
  config.minimum_tension_n = 20.0;
  config.maximum_tension_n = 200.0;
  return config;
}

ExecutedRobotSegment initial_segment() {
  ExecutedRobotSegment segment;
  segment.sequence_number = 1;
  segment.samples = {
      ExecutedRobotSample{0.0, {0.0, 0.0, 0.0, {1'000'000'000}}, 0.5},
      ExecutedRobotSample{1.0, {1.0, 0.0, 0.0, {2'000'000'000}}, 0.5},
  };
  return segment;
}

CableTelemetry telemetry(const std::uint64_t sequence,
                         const std::int64_t timestamp_ns) {
  return CableTelemetry{0.5, 0.0, 80.0, {timestamp_ns}, sequence};
}

TouchdownObservation initial_observation() {
  TouchdownObservation observation;
  observation.touchdown_position_m = {0.0, 0.0};
  observation.position_covariance_m2 = {0.01, 0.0, 0.0, 0.01};
  observation.timestamp = {2'000'000'000};
  observation.sequence_number = 1;
  return observation;
}

void begin_task(CableStateTracker& tracker) {
  const auto started = tracker.begin_new_task({1'000'000'000});
  require(started.status == CableTrackerStatus::initial_state_uncertain &&
              !started.state.has_value(),
          "explicit task start fabricated initialized cable state");
}

void snapshots_bind_the_model_calibration_domain_and_risk_semantics() {
  CableStateTracker tracker(tracker_config());
  const auto startup = tracker.snapshot();
  require(startup.cable_model_version == 17U &&
              startup.calibration_dataset_id == "cable-tracker-cal/v3" &&
              startup.operating_domain_id == "pool-a/v2" &&
              startup.risk_semantics ==
                  "state-estimate-only-no-path-risk-guarantee",
          "startup tracker diagnostics omitted their audit context");

  begin_task(tracker);
  const auto tracked = tracker.update(initial_segment(),
                                      telemetry(1, 2'000'000'000),
                                      initial_observation());
  require(tracked.cable_model_version == startup.cable_model_version &&
              tracked.calibration_dataset_id == startup.calibration_dataset_id &&
              tracked.operating_domain_id == startup.operating_domain_id &&
              tracked.risk_semantics == startup.risk_semantics,
          "tracker update silently changed its audit context");
}

void startup_is_uncertain_until_actual_touchdown_evidence_arrives() {
  CableStateTracker tracker(tracker_config());

  const auto startup = tracker.snapshot();
  require(startup.status == CableTrackerStatus::initial_state_uncertain,
          "startup did not report INITIAL_STATE_UNCERTAIN");
  require(!startup.state.has_value(),
          "startup fabricated a tracked cable state without evidence");
  require(!startup.diagnostics.empty() &&
              startup.diagnostics.front().code ==
                  CableTrackerDiagnosticCode::initial_state_uncertain,
          "startup uncertainty was not auditable");

  begin_task(tracker);
  const auto initialized = tracker.update(initial_segment(), telemetry(1, 2'000'000'000),
                                          initial_observation());
  require(initialized.status == CableTrackerStatus::tracked &&
              initialized.state.has_value(),
          "actual executed evidence did not initialize the tracker");
  require(std::abs(initialized.state->lag_angle_rad) < 1.0e-12,
          "touchdown observation initialized the wrong lag angle");
  require(initialized.state->lag_angle_variance_rad2.has_value() &&
              *initialized.state->lag_angle_variance_rad2 > 0.0,
          "tracked initialization omitted lag-angle uncertainty");

  auto detached_snapshot = initialized;
  detached_snapshot.state->lag_angle_rad = 1.2;
  require(std::abs(tracker.snapshot().state->lag_angle_rad) < 1.0e-12,
          "a detached candidate/snapshot mutated the actual tracker state");
}

void unreliable_initial_touchdown_observation_remains_uncertain() {
  CableStateTracker tracker(tracker_config());
  begin_task(tracker);
  TouchdownObservation unreliable = initial_observation();
  unreliable.touchdown_position_m = {20.0, 20.0};

  const auto rejected = tracker.update(initial_segment(),
                                       telemetry(1, 2'000'000'000),
                                       unreliable);
  require(rejected.status == CableTrackerStatus::initial_state_uncertain &&
              !rejected.state.has_value() && !rejected.usable_for_planning() &&
              rejected.diagnostics.front().code ==
                  CableTrackerDiagnosticCode::touchdown_observation_unreliable,
          "an initial touchdown inconsistent with L_td initialized tracking");
}

void complete_executed_history_initializes_without_task_start_exemption() {
  CableStateTracker tracker(tracker_config());
  ExecutedRobotSegment history;
  history.sequence_number = 1;
  history.samples = {
      ExecutedRobotSample{0.0, {0.0, 0.0, 0.0, {1'000'000'000}}, 0.5},
      ExecutedRobotSample{1.0, {1.0, 0.0, 0.0, {2'000'000'000}}, 0.5},
      ExecutedRobotSample{2.0, {2.0, 0.0, 0.0, {3'000'000'000}}, 0.5},
  };
  TouchdownObservation observation = initial_observation();
  observation.touchdown_position_m = {1.0, 0.0};
  observation.timestamp = {3'000'000'000};

  const auto initialized = tracker.update(
      history, telemetry(1, 3'000'000'000), observation);
  require(initialized.status == CableTrackerStatus::tracked &&
              initialized.usable_for_planning(),
          "complete executed touchdown history did not initialize tracking");
  require(initialized.state->laying_memory.trailing_support_samples.size() ==
                  3U &&
              initialized.state->laying_memory.retained_arc_length_m == 2.0 &&
              initialized.state->laying_memory
                      .previous_distinct_touchdown_points_m.size() == 2U,
          "initialization discarded the executed physical support window");
}

void executed_motion_propagates_uncertainty_and_normalizes_memory() {
  CableStateTracker tracker(tracker_config());
  begin_task(tracker);
  static_cast<void>(tracker.update(initial_segment(), telemetry(1, 2'000'000'000),
                                   initial_observation()));

  ExecutedRobotSegment continuation;
  continuation.sequence_number = 2;
  continuation.samples = {
      ExecutedRobotSample{1.0, {1.0, 0.0, 0.0, {2'000'000'000}}, 0.5},
      ExecutedRobotSample{2.0, {2.0, 0.0, 0.0, {2'500'000'000}}, 0.5},
      ExecutedRobotSample{3.0, {3.0, 0.0, 0.0, {3'000'000'000}}, 0.5},
      ExecutedRobotSample{4.0, {4.0, 0.0, 0.0, {3'500'000'000}}, 0.5},
  };

  const auto propagated =
      tracker.update(continuation, telemetry(2, 3'500'000'000), std::nullopt);
  require(propagated.status == CableTrackerStatus::tracked &&
              propagated.state.has_value(),
          "actual executed motion without a fresh observation was not propagated");
  require(std::abs(propagated.state->lag_angle_rad) < 1.0e-12,
          "straight executed motion changed a zero lag angle");
  require(std::abs(*propagated.state->lag_angle_variance_rad2 - 0.02747896) <
              1.0e-7,
          "lag variance did not follow the spatial process model");

  const auto& memory = propagated.state->laying_memory;
  require(memory.trailing_support_samples.size() == 3U &&
              memory.trailing_support_samples.front().touchdown_arc_length_m ==
                  2.0 &&
              memory.trailing_support_samples.back().touchdown_arc_length_m ==
                  4.0 &&
              memory.retained_arc_length_m == 2.0,
          "support memory was not normalized to its physical trailing window");
  require(memory.previous_distinct_touchdown_points_m.size() == 2U &&
              memory.previous_distinct_touchdown_points_m[0].x_m == 2.0 &&
              memory.previous_distinct_touchdown_points_m[1].x_m == 3.0,
          "memory did not retain the final two distinct touchdown points");
  require(memory.canonical_signature != 0U,
          "normalized mechanical memory omitted its deterministic signature");
  require(underwater_planner::core::validate(*propagated.state).valid,
          "tracker emitted a cable state that violates the public contract");
}

void support_memory_uses_touchdown_geometry_arc_length() {
  CableStateTracker tracker(tracker_config());
  begin_task(tracker);
  static_cast<void>(tracker.update(initial_segment(), telemetry(1, 2'000'000'000),
                                   initial_observation()));

  ExecutedRobotSegment turn;
  turn.sequence_number = 2;
  turn.samples = {
      ExecutedRobotSample{1.0, {1.0, 0.0, 0.0, {2'000'000'000}}, 0.5},
      ExecutedRobotSample{2.0,
                          {1.0, 1.0, 1.5707963267948966,
                           {2'500'000'000}},
                          0.5},
  };
  const auto tracked =
      tracker.update(turn, telemetry(2, 2'500'000'000), std::nullopt);
  const auto& samples = tracked.state->laying_memory.trailing_support_samples;
  require(samples.size() >= 2U,
          "turning execution did not append a touchdown history sample");
  const std::size_t previous_index = samples.size() - 2U;
  const std::size_t terminal_index = samples.size() - 1U;
  const double dx = samples[terminal_index].touchdown_position_m.x_m -
                    samples[previous_index].touchdown_position_m.x_m;
  const double dy = samples[terminal_index].touchdown_position_m.y_m -
                    samples[previous_index].touchdown_position_m.y_m;
  const double touchdown_distance_m = std::hypot(dx, dy);
  require(std::abs((samples[terminal_index].touchdown_arc_length_m -
                    samples[previous_index].touchdown_arc_length_m) -
                   touchdown_distance_m) < 1.0e-12,
          "support memory copied robot arc length instead of touchdown arc length");
}

void observation_interruption_and_state_loss_are_auditable() {
  CableStateTracker tracker(tracker_config());
  begin_task(tracker);
  static_cast<void>(tracker.update(initial_segment(), telemetry(1, 2'000'000'000),
                                   initial_observation()));

  ExecutedRobotSegment stale_observation_segment;
  stale_observation_segment.sequence_number = 2;
  stale_observation_segment.samples = {
      ExecutedRobotSample{1.0, {1.0, 0.0, 0.0, {2'000'000'000}}, 0.5},
      ExecutedRobotSample{2.0, {2.0, 0.0, 0.0, {5'000'000'001}}, 0.5},
  };
  const auto interrupted = tracker.update(
      stale_observation_segment, telemetry(2, 5'000'000'001), std::nullopt);
  require(interrupted.status == CableTrackerStatus::observation_interrupted &&
              interrupted.state.has_value() &&
              !interrupted.usable_for_planning(),
          "an expired touchdown observation remained planning-usable");
  require(interrupted.diagnostics.front().code ==
              CableTrackerDiagnosticCode::observation_interrupted,
          "observation interruption did not emit a structured audit code");

  const auto lost =
      tracker.mark_state_lost({5'100'000'000}, "localization-reset");
  require(lost.status == CableTrackerStatus::state_lost &&
              !lost.state.has_value() && !lost.usable_for_planning(),
          "explicit cable state loss retained a planning-usable estimate");
  require(lost.diagnostics.front().code ==
              CableTrackerDiagnosticCode::state_lost &&
              lost.diagnostics.front().message.find("localization-reset") !=
                  std::string::npos,
          "state loss reason was not preserved in diagnostics");

  ExecutedRobotSegment recovery_segment;
  recovery_segment.sequence_number = 3;
  recovery_segment.samples = {
      ExecutedRobotSample{2.0, {2.0, 0.0, 0.0, {5'000'000'001}}, 0.5},
      ExecutedRobotSample{3.0, {3.0, 0.0, 0.0, {6'000'000'000}}, 0.5},
  };
  TouchdownObservation recovery_observation = initial_observation();
  recovery_observation.touchdown_position_m = {2.0, 0.0};
  recovery_observation.timestamp = {6'000'000'000};
  recovery_observation.sequence_number = 2;
  const auto recovered = tracker.update(
      recovery_segment, telemetry(3, 6'000'000'000), recovery_observation);
  require(recovered.status == CableTrackerStatus::initial_state_uncertain &&
              recovered.state.has_value() && !recovered.usable_for_planning(),
          "post-loss single-point evidence bypassed the mechanical-history gate");

  ExecutedRobotSegment history_segment;
  history_segment.sequence_number = 4;
  history_segment.samples = {
      ExecutedRobotSample{3.0, {3.0, 0.0, 0.0, {6'000'000'000}}, 0.5},
      ExecutedRobotSample{4.0, {4.0, 0.0, 0.0, {6'500'000'000}}, 0.5},
      ExecutedRobotSample{5.0, {5.0, 0.0, 0.0, {7'000'000'000}}, 0.5},
  };
  const auto history_recovered = tracker.update(
      history_segment, telemetry(4, 7'000'000'000), std::nullopt);
  require(history_recovered.status == CableTrackerStatus::tracked &&
              history_recovered.usable_for_planning(),
          "complete post-loss mechanical history did not recover tracking");
}

void rolling_window_updates_are_continuous_and_fail_closed_on_a_gap() {
  // Design: 18.2.4-4
  CableStateTracker split_tracker(tracker_config());
  CableStateTracker single_tracker(tracker_config());
  begin_task(split_tracker);
  begin_task(single_tracker);
  static_cast<void>(split_tracker.update(initial_segment(),
                                         telemetry(1, 2'000'000'000),
                                         initial_observation()));
  static_cast<void>(single_tracker.update(initial_segment(),
                                          telemetry(1, 2'000'000'000),
                                          initial_observation()));

  ExecutedRobotSegment first_half;
  first_half.sequence_number = 2;
  first_half.samples = {
      ExecutedRobotSample{1.0, {1.0, 0.0, 0.0, {2'000'000'000}}, 0.5},
      ExecutedRobotSample{2.0, {2.0, 0.0, 0.0, {2'500'000'000}}, 0.5},
  };
  ExecutedRobotSegment second_half;
  second_half.sequence_number = 3;
  second_half.samples = {
      ExecutedRobotSample{2.0, {2.0, 0.0, 0.0, {2'500'000'000}}, 0.5},
      ExecutedRobotSample{3.0, {3.0, 0.0, 0.0, {3'000'000'000}}, 0.5},
  };
  static_cast<void>(split_tracker.update(first_half,
                                         telemetry(2, 2'500'000'000),
                                         std::nullopt));
  const auto split = split_tracker.update(
      second_half, telemetry(3, 3'000'000'000), std::nullopt);

  ExecutedRobotSegment whole;
  whole.sequence_number = 2;
  whole.samples = {first_half.samples[0], first_half.samples[1],
                   second_half.samples[1]};
  const auto single = single_tracker.update(
      whole, telemetry(2, 3'000'000'000), std::nullopt);
  require(split.status == CableTrackerStatus::tracked &&
              single.status == CableTrackerStatus::tracked,
          "equivalent rolling-window updates did not remain tracked");
  require(split.state->lag_angle_rad == single.state->lag_angle_rad &&
              split.state->lag_angle_variance_rad2 ==
                  single.state->lag_angle_variance_rad2 &&
              split.state->laying_memory.canonical_signature ==
                  single.state->laying_memory.canonical_signature,
          "splitting an executed path changed its terminal cable state");

  ExecutedRobotSegment discontinuous;
  discontinuous.sequence_number = 4;
  discontinuous.samples = {
      ExecutedRobotSample{3.0, {30.0, 0.0, 0.0, {3'000'000'000}}, 0.5},
      ExecutedRobotSample{4.0, {31.0, 0.0, 0.0, {3'500'000'000}}, 0.5},
  };
  const auto lost = split_tracker.update(
      discontinuous, telemetry(4, 3'500'000'000), std::nullopt);
  require(lost.status == CableTrackerStatus::state_lost &&
              !lost.state.has_value() &&
              lost.diagnostics.front().code ==
                  CableTrackerDiagnosticCode::executed_segment_discontinuity,
          "a discontinuous rolling-window boundary did not lose state safely");
}

void confirmed_touchdown_observations_correct_state_and_actual_memory() {
  CableStateTracker tracker(tracker_config());
  begin_task(tracker);
  static_cast<void>(tracker.update(initial_segment(), telemetry(1, 2'000'000'000),
                                   initial_observation()));

  ExecutedRobotSegment executed;
  executed.sequence_number = 2;
  executed.samples = {
      ExecutedRobotSample{1.0, {1.0, 0.0, 0.0, {2'000'000'000}}, 0.5},
      ExecutedRobotSample{2.0, {2.0, 0.0, 0.0, {2'500'000'000}}, 0.5},
  };
  TouchdownObservation observation;
  observation.touchdown_position_m = {2.0, -1.0};
  observation.position_covariance_m2 = {0.01, 0.0, 0.0, 0.01};
  observation.timestamp = {2'500'000'000};
  observation.sequence_number = 2;

  const auto corrected = tracker.update(
      executed, telemetry(2, 2'500'000'000), observation);
  require(corrected.status == CableTrackerStatus::tracked &&
              corrected.state->lag_angle_rad > 1.3 &&
              corrected.state->lag_angle_rad < 1.5,
          "confirmed touchdown geometry did not correct the lag estimate");
  require(*corrected.state->lag_angle_variance_rad2 < 0.01,
          "confirmed touchdown evidence did not reduce lag uncertainty");
  const auto& terminal =
      corrected.state->laying_memory.trailing_support_samples.back();
  require(terminal.touchdown_position_m.x_m == 2.0 &&
              terminal.touchdown_position_m.y_m == -1.0,
          "actual memory retained a prediction instead of confirmed touchdown");
}

void discontinuous_touchdown_observations_lose_state_instead_of_forcing_it() {
  CableStateTracker tracker(tracker_config());
  begin_task(tracker);
  static_cast<void>(tracker.update(initial_segment(), telemetry(1, 2'000'000'000),
                                   initial_observation()));

  ExecutedRobotSegment executed;
  executed.sequence_number = 2;
  executed.samples = {
      ExecutedRobotSample{1.0, {1.0, 0.0, 0.0, {2'000'000'000}}, 0.5},
      ExecutedRobotSample{2.0, {2.0, 0.0, 0.0, {2'500'000'000}}, 0.5},
  };
  TouchdownObservation discontinuous = initial_observation();
  discontinuous.touchdown_position_m = {20.0, 20.0};
  discontinuous.timestamp = {2'500'000'000};
  discontinuous.sequence_number = 2;

  const auto lost = tracker.update(
      executed, telemetry(2, 2'500'000'000), discontinuous);
  require(lost.status == CableTrackerStatus::state_lost &&
              !lost.state.has_value() &&
              lost.diagnostics.front().code ==
                  CableTrackerDiagnosticCode::touchdown_observation_discontinuity,
          "a discontinuous touchdown observation was forced into the estimate");
}

}  // namespace

int main() {
  constexpr std::uint64_t kSeed = 1111;
  try {
    snapshots_bind_the_model_calibration_domain_and_risk_semantics();
    startup_is_uncertain_until_actual_touchdown_evidence_arrives();
    unreliable_initial_touchdown_observation_remains_uncertain();
    complete_executed_history_initializes_without_task_start_exemption();
    executed_motion_propagates_uncertainty_and_normalizes_memory();
    support_memory_uses_touchdown_geometry_arc_length();
    observation_interruption_and_state_loss_are_auditable();
    rolling_window_updates_are_continuous_and_fail_closed_on_a_gap();
    confirmed_touchdown_observations_correct_state_and_actual_memory();
    discontinuous_touchdown_observations_lose_state_instead_of_forcing_it();
    std::cout << "cable state tracker checks passed: 10"
              << " seed=" << kSeed
              << " input_version=t11-cable-state-tracker/v1"
              << " units=SI timestamp=monotonic-ns"
              << " domain=ros-agnostic-core risk=initial-state-audited\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "cable state tracker failure: " << error.what()
              << " seed=" << kSeed
              << " input_version=t11-cable-state-tracker/v1"
              << " units=SI timestamp=monotonic-ns"
              << " domain=ros-agnostic-core risk=initial-state-audited\n";
    return 1;
  }
}
