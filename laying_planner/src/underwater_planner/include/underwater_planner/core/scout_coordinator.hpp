#pragma once

#include "underwater_planner/core/data_contract.hpp"
#include "underwater_planner/core/parameter_config.hpp"
#include "underwater_planner/core/traversability_evaluator.hpp"
#include "underwater_planner/core/versioned_snapshot.hpp"

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace underwater_planner::core {

enum class GapUrgency { blocking, urgent, scheduled, informational };

[[nodiscard]] std::string_view to_string(GapUrgency urgency) noexcept;

enum class ScoutGapScanValidity { valid, input_invalid };

struct ScoutCoordinationParameters {
  std::string parameter_profile_id;
  std::string operating_domain_id;
  double minimum_map_confidence{};
  double sample_interval_m{};
  double merge_distance_m{};
  double minimum_safe_distance_m{};
  double planning_lead_time_s{};
  double average_velocity_mps{};
  double hysteresis_distance_m{};
  double hysteresis_time_s{};
  std::uint64_t policy_version{};
  double sensor_coverage_radius_m{};
  double scout_corridor_half_width_m{};
  double communication_max_distance_m{};
  double desired_scout_distance_m{};
  double continue_scout_distance_m{};
  double stop_scout_distance_m{};
  double blocking_priority_weight{};
  double information_value_weight{};
  double forward_progress_weight{};
  double arrival_cost_weight{};
  Duration request_timeout;
};

[[nodiscard]] bool valid(const ScoutCoordinationParameters& parameters) noexcept;
[[nodiscard]] ScoutCoordinationParameters make_scout_coordination_parameters(
    const ParameterConfig& config);

struct InformationGapScanResult {
  ScoutGapScanValidity validity{ScoutGapScanValidity::input_invalid};
  MapVersion source_map_version;
  std::uint32_t reference_line_version{};
  double planning_horizon_m{};
  std::vector<InformationGap> gaps;
  std::vector<std::string> issues;
};

struct GapUrgencyAssessment {
  GapUrgency urgency{GapUrgency::informational};
  double distance_to_gap_m{};
  double time_to_gap_s{};
  double safe_path_remaining_m{};
  bool blocks_planning_window{};
  bool used_conservative_fallback{};
  // URGENT deliberately requests a newly validated profile. It does not
  // authorize a direct speed multiplier.
  std::string recommended_action;
};

struct PrioritizedGapAssessment {
  InformationGap gap;
  GapUrgencyAssessment assessment;
};

enum class ScoutTargetValidity {
  valid,
  input_invalid,
  distance_constraint_violated,
};

struct ScoutTarget {
  Pose2d target_pose;
  double gap_start_progress_m{};
  double gap_end_progress_m{};
  double coverage_fraction{};
  double information_value{};
  double forward_progress_m{};
  double estimated_arrival_cost_m{};
  double priority{};
  double scout_corridor_half_width_m{};
  GapUrgency urgency{GapUrgency::informational};
  MapVersion source_map_version;
  std::uint32_t reference_line_version{};
  std::uint64_t policy_version{};
  std::string parameter_profile_id;
  std::string operating_domain_id;
};

struct ScoutTargetGenerationResult {
  ScoutTargetValidity validity{ScoutTargetValidity::input_invalid};
  std::optional<ScoutTarget> target;
  std::vector<std::string> issues;
};

enum class ScoutDistanceDirective {
  advance,
  maintain_desired_spacing,
  hold_position,
  recover_communication,
  input_invalid,
};

enum class ScoutMainAction {
  continue_approved_plan,
  request_validated_reduced_speed_profile,
  stop_and_wait_for_map,
  stop_and_recover_communication,
  trigger_replan_with_new_map,
};

enum class ScoutPlanningDirective { none, waiting_map, replan_with_new_map };

struct ScoutDistanceAssessment {
  ScoutDistanceDirective directive{ScoutDistanceDirective::input_invalid};
  double separation_m{};
  double desired_distance_m{};
  double communication_max_distance_m{};
  bool communication_satisfied{};
  bool hysteresis_hold_active{};
  bool main_robot_degraded{true};
  ScoutMainAction recommended_main_action{
      ScoutMainAction::stop_and_recover_communication};
  std::vector<std::string> issues;
};

enum class ScoutRequestStatus {
  awaiting_map_update,
  completed,
  timed_out,
};

enum class ScoutRequestIssueDisposition { issued, deduplicated, rejected };

struct ScoutRequest {
  std::string schema_version{"scout-request/v1"};
  std::uint64_t request_sequence{};
  std::uint64_t revision{};
  std::uint64_t policy_version{};
  ScoutRequestStatus status{ScoutRequestStatus::awaiting_map_update};
  ScoutTarget target;
  MonotonicTime requested_at;
  MonotonicTime expires_at;
  std::optional<MapVersion> last_associated_map_version;
  std::optional<MapVersion> completed_map_version;
  ScoutMainAction recommended_main_action{
      ScoutMainAction::stop_and_wait_for_map};
  ScoutPlanningDirective planning_directive{ScoutPlanningDirective::none};
};

struct ScoutRequestIssueResult {
  ScoutRequestIssueDisposition disposition{
      ScoutRequestIssueDisposition::rejected};
  std::optional<ScoutRequest> request;
  std::vector<std::string> issues;
};

enum class ScoutMapUpdateDisposition {
  associated_unresolved,
  completed,
  rejected,
  request_not_found,
  request_terminal,
  timed_out,
};

struct ScoutMapUpdateResult {
  ScoutMapUpdateDisposition disposition{
      ScoutMapUpdateDisposition::rejected};
  std::optional<ScoutRequest> request;
  bool invalidate_old_plan{};
  bool trigger_replanning{};
  std::vector<std::string> issues;
};

struct ScoutRequestExpiryResult {
  bool valid{};
  std::vector<ScoutRequest> expired;
  std::vector<std::string> issues;
};

class ScoutCoordinator {
 public:
  explicit ScoutCoordinator(ScoutCoordinationParameters parameters);

  [[nodiscard]] const ScoutCoordinationParameters& parameters() const noexcept {
    return parameters_;
  }

  [[nodiscard]] InformationGapScanResult identify_gaps_result(
      const ReferenceLine& reference_line, const MapSnapshot& map,
      double planning_horizon_m) const;

  [[nodiscard]] InformationGapScanResult identify_gaps_result(
      const ReferenceLine& reference_line, const MapSnapshot& map,
      double planning_horizon_m,
      const std::optional<GeometricPath>& candidate_detour) const;

  [[nodiscard]] std::vector<InformationGap> identify_gaps(
      const ReferenceLine& reference_line, const MapSnapshot& map,
      double planning_horizon_m) const;

  [[nodiscard]] std::vector<InformationGap> identify_gaps(
      const ReferenceLine& reference_line, const MapSnapshot& map,
      double planning_horizon_m,
      const std::optional<GeometricPath>& candidate_detour) const;

  // Camel-case wrappers mirror the design document while the snake-case
  // methods follow the repository API convention.
  [[nodiscard]] std::vector<InformationGap> identifyGaps(
      const ReferenceLine& reference_line, const MapSnapshot& map,
      double planning_horizon_m) const {
    return identify_gaps(reference_line, map, planning_horizon_m);
  }

  [[nodiscard]] GapUrgencyAssessment assess_gap_urgency(
      const InformationGap& gap, const RobotState& robot_state,
      double current_reference_progress_m,
      const std::optional<TimedPath>& approved_remaining_path,
      double planning_horizon_m) const;

  [[nodiscard]] GapUrgencyAssessment assessGapUrgency(
      const InformationGap& gap, const RobotState& robot_state,
      double current_reference_progress_m,
      const std::optional<TimedPath>& approved_remaining_path,
      double planning_horizon_m) const {
    return assess_gap_urgency(gap, robot_state, current_reference_progress_m,
                              approved_remaining_path, planning_horizon_m);
  }

  [[nodiscard]] std::vector<PrioritizedGapAssessment> assess_gaps(
      const std::vector<InformationGap>& gaps, const RobotState& robot_state,
      double current_reference_progress_m,
      const std::optional<TimedPath>& approved_remaining_path,
      double planning_horizon_m) const;

  [[nodiscard]] ScoutTargetGenerationResult generate_scout_target(
      const InformationGap& gap, const GapUrgencyAssessment& assessment,
      const ReferenceLine& reference_line, const Pose2d& main_robot_pose,
      const Pose2d& scout_robot_pose,
      double current_reference_progress_m) const;

  [[nodiscard]] std::vector<ScoutTarget> generate_scout_targets(
      const std::vector<PrioritizedGapAssessment>& gaps,
      const ReferenceLine& reference_line, const Pose2d& main_robot_pose,
      const Pose2d& scout_robot_pose,
      double current_reference_progress_m) const;

  [[nodiscard]] ScoutDistanceAssessment assess_distance_constraint(
      const Pose2d& main_robot_pose, const Pose2d& scout_robot_pose) const;

  [[nodiscard]] ScoutRequestIssueResult issue_scout_request(
      const ScoutTarget& target, MonotonicTime now);

  [[nodiscard]] ScoutMapUpdateResult correlate_map_update(
      std::uint64_t request_sequence, const ReferenceLine& reference_line,
      const MapSnapshot& updated_map, MonotonicTime now);

  [[nodiscard]] ScoutRequestExpiryResult expire_scout_requests(
      MonotonicTime now);

  [[nodiscard]] std::optional<ScoutRequest> scout_request(
      std::uint64_t request_sequence) const;

  [[nodiscard]] bool isDistanceConstraintSatisfied(
      const Pose2d& main_robot_pose, const Pose2d& scout_robot_pose) const {
    return assess_distance_constraint(main_robot_pose, scout_robot_pose)
        .communication_satisfied;
  }

  [[nodiscard]] ScoutTargetGenerationResult generateScoutTarget(
      const InformationGap& gap, const GapUrgencyAssessment& assessment,
      const ReferenceLine& reference_line, const Pose2d& main_robot_pose,
      const Pose2d& scout_robot_pose,
      double current_reference_progress_m) const {
    return generate_scout_target(gap, assessment, reference_line,
                                 main_robot_pose, scout_robot_pose,
                                 current_reference_progress_m);
  }

  void clear_hysteresis() const;

 public:
  struct GapKey {
    std::string map_id;
    std::uint64_t map_sequence{};
    std::uint32_t reference_version{};
    std::int64_t start_micrometers{};
    std::int64_t end_micrometers{};
    std::int64_t target_x_micrometers{};
    std::int64_t target_y_micrometers{};

    friend bool operator<(const GapKey& left, const GapKey& right) noexcept;
  };

 private:

  [[nodiscard]] GapUrgency apply_hysteresis(
      const GapKey& key, GapUrgency candidate, double time_to_gap_s,
      double safe_path_remaining_m, bool blocks_window,
      double planning_horizon_m) const;

  void mark_request_timed_out(ScoutRequest& request);

  ScoutCoordinationParameters parameters_;
  mutable std::map<GapKey, GapUrgency> previous_urgencies_;
  mutable bool distance_hold_active_{};
  std::uint64_t next_request_sequence_{1U};
  std::map<std::uint64_t, ScoutRequest> requests_;
  std::map<GapKey, std::uint64_t> active_request_by_gap_;
};

}  // namespace underwater_planner::core
