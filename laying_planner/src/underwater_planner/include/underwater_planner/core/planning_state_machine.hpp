#pragma once

#include "underwater_planner/core/data_contract.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace underwater_planner::core {

enum class PlanningEventType {
  periodic_tick,
  new_map,
  reference_line_changed,
  robot_operating_area_changed,
  robot_state_changed,
  path_invalidated,
  communication_lost,
  communication_restored,
  localization_invalid,
  lease_renewal_required,
  lease_invalidated,
  planning_succeeded,
  planning_with_caution,
  waiting_for_map,
  scout_requested,
  planning_timed_out,
  planning_failed,
  covariance_solution_unavailable,
  covariance_envelope_breached,
  input_invalid,
  map_expired,
  manual_override_requested,
  manual_control_released,
  emergency_stop_requested,
  emergency_stop_cleared,
  critical_system_failure,
};

enum class PlanningAction {
  none,
  continue_authorized_path,
  begin_planning,
  reduce_speed,
  controlled_stop,
  emergency_stop,
  manual_takeover,
};

// Directives are ordered. Safety consumers must execute them in sequence, so
// lease revocation cannot accidentally occur after a stop or replan request.
enum class PlanningDirective {
  revoke_current_lease,
  request_controlled_stop,
  request_emergency_stop,
  start_planning,
  continue_authorized_path,
  switch_to_validated_cautious_profile,
  request_scout,
  request_manual_takeover,
  stop_automatic_planning,
};

struct SafeStopContext {
  double current_ground_speed_mps{};
  double maximum_braking_deceleration_mps2{};
  double terrain_limited_braking_deceleration_mps2{};
  double remaining_safe_distance_m{};
  double safety_margin_m{};
  Duration control_reaction_time;
  bool terrain_braking_model_certified{};
};

enum class SafeStopStatus {
  feasible,
  insufficient_distance,
  invalid_input,
  terrain_model_uncertified,
};

struct SafeStopAssessment {
  SafeStopStatus status{SafeStopStatus::invalid_input};
  double effective_braking_deceleration_mps2{};
  double required_stopping_distance_m{};
  double remaining_safe_distance_m{};

  [[nodiscard]] bool feasible() const noexcept {
    return status == SafeStopStatus::feasible;
  }
};

struct RecoveryAuthorization {
  std::uint64_t synchronized_source_revision{};
  std::uint64_t lease_sequence{};
  bool synchronized_snapshot_valid{};
  bool lease_live{};
  bool dependencies_match{};
};

struct PlanningEvent {
  PlanningEventType type{PlanningEventType::input_invalid};
  std::uint64_t sequence_number{};
  MonotonicTime observed_at;
  std::string reason;
};

// A single immutable view captured for the event decision. Adapters populate
// it from the synchronized validation frame and current lease monitor output;
// the state machine never reads mutable global state during dispatch.
struct PlanningDecisionContext {
  std::uint64_t map_sequence{};
  bool current_lease_live{};
  bool degraded_sensor_mode_approved{};
  bool degraded_profile_lease_live{};
  Duration communication_outage;
  std::optional<SafeStopContext> safe_stop;
  std::optional<RecoveryAuthorization> recovery;
};

struct PlanningStateMachineConfig {
  std::uint64_t version{};
  std::string parameter_profile_id;
  std::string operating_domain_id;
  Duration planning_period;
  std::uint32_t maximum_consecutive_failures{};
  Duration short_communication_outage_limit;
  Duration medium_communication_outage_limit;
};

struct PlanningDecision {
  bool accepted{};
  PlanningState previous_state{PlanningState::init};
  PlanningState state{PlanningState::init};
  PlanningAction action{PlanningAction::none};
  std::vector<PlanningDirective> directives;
  std::optional<SafeStopAssessment> safe_stop_assessment;
  std::uint64_t event_sequence{};
  MonotonicTime decided_at;
  std::uint64_t state_machine_config_version{};
  std::string parameter_profile_id;
  std::string operating_domain_id;
  std::uint64_t synchronized_source_revision{};
  std::uint64_t lease_sequence{};
  std::string reason_code;
  std::string reason;
};

class PlanningStateMachine {
 public:
  explicit PlanningStateMachine(PlanningStateMachineConfig config) noexcept;

  [[nodiscard]] PlanningDecision dispatch(
      const PlanningEvent& event,
      const PlanningDecisionContext& context = PlanningDecisionContext{});
  [[nodiscard]] PlanningState state() const noexcept { return state_; }

 private:
  PlanningStateMachineConfig config_;
  PlanningState state_{PlanningState::init};
  std::optional<MonotonicTime> last_plan_trigger_at_;
  std::optional<MonotonicTime> last_event_at_;
  std::uint64_t highest_event_sequence_{};
  std::uint64_t highest_source_revision_{};
  std::uint64_t highest_lease_sequence_{};
  std::uint64_t highest_map_sequence_{};
  std::uint32_t consecutive_failures_{};
};

}  // namespace underwater_planner::core
