#pragma once

#include "underwater_planner/core/path_candidate_verifier.hpp"
#include "underwater_planner/core/planning_result.hpp"
#include "underwater_planner/core/reference_progress_tracker.hpp"
#include "underwater_planner/core/synchronized_validation_inputs.hpp"
#include "underwater_planner/core/timed_cable_candidate_verifier.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace underwater_planner::core {

enum class PlanValidationAction { reuse, replan, stop };

enum class PlanValidationStatus {
  valid,
  state_mismatch,
  context_mismatch,
  input_expired,
  execution_profile_mismatch,
  execution_profile_invalid,
  stopping_distance_insufficient,
  robot_constraint_violation,
  cable_model_invalid,
  covariance_envelope_unavailable,
  covariance_envelope_breach,
  cable_corridor_invalid,
  cable_corridor_violation,
  cable_laying_invalid,
  input_invalid,
};

struct PlanValidationLease {
  std::uint64_t lease_sequence{};
  std::uint64_t plan_sequence_number{};
  std::uint64_t evaluator_config_version{};
  std::string parameter_profile_id;
  MonotonicTime validated_at;
  MonotonicTime expires_at;
  double remaining_path_start_arc_length_m{};
  MapVersion map_version;
  std::uint32_t reference_line_version{};
  std::uint32_t robot_operating_area_version{};
  std::uint64_t terrain_gradient_policy_version{};
  std::uint64_t corridor_risk_policy_version{};
  std::uint64_t cable_model_version{};
  std::uint64_t uncertainty_envelope_version{};
  std::uint64_t uncertainty_envelope_generator_version{};
  std::uint64_t execution_operating_envelope_version{};
  std::uint64_t execution_profile_version{};
  SensorHealthMode sensor_mode{SensorHealthMode::nominal};
  std::string operating_domain_id;
  std::uint32_t cable_corridor_version{};
  MonotonicTime robot_state_timestamp;
  MonotonicTime cable_state_timestamp;
  MonotonicTime cable_telemetry_timestamp;
  MonotonicTime execution_tracking_timestamp;
  double max_ground_speed_tracking_error_mps{};
  double max_payout_speed_tracking_error_mps{};
  RangeN allowed_tension;
  RangeMps2 allowed_ground_acceleration;
  bool robot_path_validation_passed{};
  bool cable_corridor_validation_passed{};
  bool cable_laying_validation_passed{};

  [[nodiscard]] PlanningDependencyVersions dependencies() const {
    return {map_version,
            reference_line_version,
            robot_operating_area_version,
            terrain_gradient_policy_version,
            corridor_risk_policy_version,
            cable_model_version,
            uncertainty_envelope_version,
            uncertainty_envelope_generator_version,
            execution_operating_envelope_version,
            execution_profile_version,
            sensor_mode,
            operating_domain_id,
            cable_corridor_version};
  }
};

struct PlanValidityEvaluatorConfig {
  std::uint64_t version{};
  std::string parameter_profile_id;
  std::string operating_domain_id;
  Duration maximum_reuse_duration;
  ValidationInputCaptureLimits input_limits;
  Duration envelope_validity_margin;
  double position_tolerance_m{0.05};
  double heading_tolerance_rad{0.05};
  double curvature_tolerance_per_m{0.02};
  double maximum_ground_speed_tracking_error_mps{};
  double maximum_ground_acceleration_tracking_error_mps2{};
  double stopping_safety_margin_m{};
  std::uint64_t last_issued_lease_sequence{};
};

struct PlanValidationCorridorIntervalBound {
  std::uint64_t certificate_version{};
  double upper_bound_error_m{};
};

struct PlanValidityContext {
  TerrainLayers terrain;
  CableContext cable_context;
  CableCorridorRiskPolicy corridor_policy;
  PlanValidationCorridorIntervalBound corridor_interval_bound;
  ReferenceProgressAssociationParameters reference_progress_parameters;
  CableLayingLimits laying_limits;
  CableHistoryBoundary history_boundary{
      CableHistoryBoundary::actual_laying_history};
  std::optional<LockedCableUncertaintyEnvelope> locked_envelope;
  CableUncertaintyEnvelopeManager* envelope_manager{};
  PathCandidateVerificationContext path_context;
  SmoothingLimits smoothing_limits;
  PathBoundary goal_boundary;
  double envelope_audit_tolerance_m{};
  bool reference_is_deterministic{true};
  bool covariance_includes_coordinate_transform_error{};
};

struct PlanValidityEvaluation {
  PlanValidationAction action{PlanValidationAction::stop};
  PlanValidationStatus status{PlanValidationStatus::input_invalid};
  bool valid{};
  std::uint64_t evaluator_config_version{};
  std::string parameter_profile_id;
  std::shared_ptr<const TimedPath> remaining_path;
  std::optional<PlanValidationLease> lease;
  std::optional<CablePrediction> cable_prediction;
  Diagnostics diagnostics;
  std::vector<std::string> issues;
};

enum class PlanValidationTarget {
  authorized_current,
  publication_candidate,
};

class PlanValidityEvaluator {
 public:
  PlanValidityEvaluator(CableModel cable_model,
                        PlanValidityEvaluatorConfig config);

  [[nodiscard]] PlanValidityEvaluation validateRemainingPlan(
      const ImmutablePlanningResult& plan,
      const SynchronizedValidationInputs& inputs,
      const PlanValidityContext& context,
      MonotonicTime now);

  [[nodiscard]] PlanValidityEvaluation validatePublicationCandidate(
      const ImmutablePlanningResult& plan,
      const SynchronizedValidationInputs& inputs,
      const PlanValidityContext& context,
      MonotonicTime now);

  [[nodiscard]] std::uint64_t next_lease_sequence() const noexcept {
    return next_lease_sequence_;
  }

 private:
  [[nodiscard]] PlanValidityEvaluation validatePlan(
      const ImmutablePlanningResult& plan,
      const SynchronizedValidationInputs& inputs,
      const PlanValidityContext& context, MonotonicTime now,
      PlanValidationTarget target);

  CableModel cable_model_;
  PlanValidityEvaluatorConfig config_;
  std::uint64_t next_lease_sequence_{1};
};

}  // namespace underwater_planner::core
