#pragma once

#include "underwater_planner/core/cable_corridor_evaluator.hpp"
#include "underwater_planner/core/cable_laying_evaluator.hpp"
#include "underwater_planner/core/cable_model.hpp"
#include "underwater_planner/core/cable_uncertainty_envelope_manager.hpp"
#include "underwater_planner/core/merge_goal_generator.hpp"
#include "underwater_planner/core/reference_progress_tracker.hpp"
#include "underwater_planner/core/traversability_evaluator.hpp"

#include <cstddef>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace underwater_planner::core {

struct HybridAStarMotionPrimitive {
  std::uint64_t version{};
  double arc_length_m{};
  double curvature_per_m{};
};

struct HybridAStarSearchParameters {
  std::uint64_t version{};
  std::uint64_t primitive_set_version{};
  std::uint64_t path_version{};
  double xy_resolution_m{};
  double heading_resolution_rad{};
  double cable_lag_resolution_rad{};
  double reference_progress_resolution_m{};
  double goal_position_tolerance_m{};
  double goal_heading_tolerance_rad{};
  double goal_lag_tolerance_rad{};
  double goal_progress_tolerance_m{};
  double goal_touchdown_position_tolerance_m{};
  double minimum_turning_radius_m{};
  double path_length_cost_weight{};
  double path_curvature_cost_weight{};
  double touchdown_center_cost_weight{};
  double touchdown_margin_cost_weight{};
  double robot_terrain_cost_weight{};
  double maximum_sweep_spacing_fraction{};
  double cable_sweep_margin_m{};
  double equivalent_label_cost_tolerance_m{};
  double maximum_planning_duration_s{};
  std::size_t maximum_expansions{};
  std::size_t maximum_active_labels{};
  std::size_t analytic_expansion_interval{};
  std::vector<HybridAStarMotionPrimitive> motion_primitives;
};

[[nodiscard]] std::string serialize_hybrid_astar_search_parameters(
    const HybridAStarSearchParameters& parameters);

struct HybridAStarBaseKey {
  std::int64_t x_index{};
  std::int64_t y_index{};
  std::int64_t heading_index{};
  std::int64_t cable_lag_index{};
  std::int64_t reference_progress_index{};
};

struct HybridAStarPrimitiveSweepContext {
  MapSnapshot map;
  TerrainLayers terrain;
  RobotOperatingArea robot_operating_area;
  Covariance2dM2 robot_relative_obstacle_covariance_m2;
  RobotCollisionRiskPolicy collision_risk_policy;
  RobotCapability robot_capability;
  TrackFootprint track_footprint;
  TerrainGradientRiskPolicy terrain_gradient_risk_policy;
  CableLayingLimits cable_laying_limits;
  CableHistoryBoundary cable_history_boundary{
      CableHistoryBoundary::actual_laying_history};
};

[[nodiscard]] bool operator<(const HybridAStarBaseKey& left,
                             const HybridAStarBaseKey& right) noexcept;

struct HybridAStarStateTraceEntry {
  HybridAStarBaseKey base_key;
  Pose2d robot_pose;
  double cable_lag_angle_rad{};
  ReferenceProgress reference_progress;
};

struct HybridAStarCostComponents {
  double robot_length{};
  double robot_curvature{};
  double touchdown_corridor{};
  double cable_suitability{};
  double robot_terrain{};

  [[nodiscard]] double total() const noexcept {
    return robot_length + robot_curvature + touchdown_corridor +
           cable_suitability + robot_terrain;
  }
};

struct HybridAStarWorstConstraint {
  bool recorded{};
  std::string reason;
  Vector2m position_m;
  double constraint_value{};
  double hard_limit{};
  double normalized_utilization{};
};

struct HybridAStarPlanningRequest {
  RobotState start_state;
  CableState initial_cable_state;
  CableContext cable_context;
  ReferenceProgress initial_reference_progress;
  ReferenceLine reference_line;
  std::vector<MergeGoal> goals;
  HybridAStarPrimitiveSweepContext primitive_sweep_context;
  LockedCableUncertaintyEnvelope locked_uncertainty_envelope;
  MonotonicTime planning_timestamp;
  std::uint64_t random_seed{};
};

struct HybridAStarSearchDiagnostics {
  std::uint64_t search_parameter_version{};
  std::uint64_t primitive_set_version{};
  std::uint64_t cable_model_version{};
  std::uint64_t uncertainty_envelope_version{};
  std::uint64_t uncertainty_envelope_generator_version{};
  std::uint64_t execution_operating_envelope_version{};
  std::uint64_t corridor_risk_policy_version{};
  std::uint64_t terrain_map_sequence{};
  std::uint64_t terrain_analysis_config_version{};
  std::uint64_t collision_risk_policy_version{};
  std::uint64_t terrain_gradient_risk_policy_version{};
  std::uint64_t cable_laying_limits_version{};
  std::uint32_t robot_operating_area_version{};
  std::uint32_t reference_line_version{};
  std::uint64_t random_seed{};
  std::uint64_t deterministic_fingerprint{};
  double epsilon_point{};
  double standard_normal_quantile{};
  double maximum_sweep_spacing_fraction{};
  double envelope_discretization_margin_m{};
  double cable_sweep_margin_m{};
  double equivalent_label_cost_tolerance_m{};
  double maximum_planning_duration_s{};
  double goal_touchdown_position_tolerance_m{};
  double minimum_turning_radius_m{};
  double path_length_cost_weight{};
  double path_curvature_cost_weight{};
  double touchdown_center_cost_weight{};
  double touchdown_margin_cost_weight{};
  double robot_terrain_cost_weight{};
  double maximum_envelope_stddev_upper_bound_m{};
  std::size_t expanded_state_count{};
  std::size_t generated_successor_count{};
  std::size_t envelope_query_count{};
  std::size_t cable_model_rejection_count{};
  std::size_t reference_association_rejection_count{};
  std::size_t envelope_unavailable_rejection_count{};
  std::size_t corridor_rejection_count{};
  std::size_t operating_area_rejection_count{};
  std::size_t collision_rejection_count{};
  std::size_t traversability_rejection_count{};
  std::size_t cable_laying_rejection_count{};
  std::size_t maximum_active_label_budget{};
  std::size_t fixed_bytes_per_search_label{};
  std::size_t peak_observed_bytes_per_search_label{};
  std::size_t analytic_expansion_interval{};
  std::size_t active_label_count{};
  std::size_t peak_active_label_count{};
  std::size_t maximum_labels_per_base_key{};
  std::size_t labels_per_base_key_p50{};
  std::size_t labels_per_base_key_p95{};
  std::size_t labels_per_base_key_p99{};
  std::size_t equivalent_label_discard_count{};
  std::size_t equivalent_label_replacement_count{};
  std::size_t signature_fallback_comparison_count{};
  std::size_t stale_queue_entry_count{};
  std::size_t analytic_expansion_attempt_count{};
  std::size_t analytic_expansion_accepted_count{};
  bool active_label_budget_exhausted{};
  bool deadline_exceeded{};
  std::size_t maximum_robot_sweep_pose_count{};
  double maximum_robot_sweep_spacing_m{};
  double maximum_collision_sweep_margin_m{};
  double maximum_primitive_laying_soft_cost{};
  double initial_heuristic_cost{};
  double solution_cost{};
  HybridAStarCostComponents solution_cost_components;
  HybridAStarWorstConstraint worst_constraint;
  bool path_dependent_covariance_propagated{};
  std::string operating_domain_id;
  std::string risk_semantics;
  std::string queue_rule;
  std::vector<DiagnosticEntry> entries;
};

struct HybridAStarPlanningResult {
  PlanningState state{PlanningState::input_invalid};
  GeometricPath robot_path;
  GeometricPath touchdown_path;
  CableState terminal_cable_state;
  ReferenceProgress terminal_reference_progress;
  std::vector<HybridAStarStateTraceEntry> state_trace;
  HybridAStarSearchDiagnostics diagnostics;
};

using HybridAStarSteadyClock =
    std::function<std::chrono::steady_clock::time_point()>;

class HybridAStarPlanner {
 public:
  HybridAStarPlanner(
      CableModelParameters cable_model_parameters,
      ReferenceProgressAssociationParameters progress_parameters,
      HybridAStarSearchParameters search_parameters,
      CableCorridorRiskPolicy corridor_risk_policy,
      CableUncertaintyEnvelopeManager& envelope_manager,
      HybridAStarSteadyClock clock = [] {
        return std::chrono::steady_clock::now();
      });

  [[nodiscard]] HybridAStarPlanningResult plan(
      const HybridAStarPlanningRequest& request) const;

 private:
  CableModel cable_model_;
  ReferenceProgressAssociator progress_associator_;
  HybridAStarSearchParameters search_parameters_;
  CableCorridorRiskPolicy corridor_risk_policy_;
  CableUncertaintyEnvelopeManager& envelope_manager_;
  HybridAStarSteadyClock clock_;
};

}  // namespace underwater_planner::core
