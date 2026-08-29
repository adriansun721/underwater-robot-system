#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace underwater_planner::core {

// All contract values use SI units. Time points are nanoseconds from the
// application's monotonic clock and are deliberately not wall-clock times.
struct MonotonicTime {
  std::int64_t nanoseconds{-1};
};

struct Duration {
  std::int64_t nanoseconds{-1};
};

struct Pose2d {
  double x_m{};
  double y_m{};
  double heading_rad{};
  MonotonicTime timestamp;
};

struct RobotState {
  Pose2d pose;
  double ground_speed_mps{};
  double curvature_per_m{};
  MonotonicTime curvature_timestamp;
  std::uint64_t sequence_number{};
};

struct Vector2m {
  double x_m{};
  double y_m{};
};

struct CableHistorySample {
  double touchdown_arc_length_m{};
  Vector2m touchdown_position_m;
};

struct CableConstraintMemory {
  std::vector<Vector2m> previous_distinct_touchdown_points_m;
  std::vector<CableHistorySample> trailing_support_samples;
  double retained_arc_length_m{};
  std::uint64_t canonical_signature{};
};

enum class CableStateKind { tracked, search_mean };

struct CableState {
  CableStateKind kind{CableStateKind::tracked};
  double lag_angle_rad{};
  std::optional<double> lag_angle_variance_rad2;
  MonotonicTime timestamp;
  CableConstraintMemory laying_memory;
  std::uint64_t sequence_number{};
};

struct CableTelemetry {
  double payout_speed_mps{};
  double payout_acceleration_mps2{};
  double tension_n{};
  MonotonicTime timestamp;
  std::uint64_t sequence_number{};
};

struct ReferenceProgress {
  std::uint32_t reference_line_version{};
  double arc_length_m{};
  MonotonicTime timestamp;
  std::uint64_t sequence_number{};
};

struct PathPoint {
  double arc_length_m{};
  double x_m{};
  double y_m{};
  double heading_rad{};
  double curvature_per_m{};
};

struct PathConstraintResiduals {
  double maximum_dynamics_residual{};
  double maximum_curvature_audit_residual{};
  double maximum_curvature_rate_residual{};
  double start_position_residual_m{};
  double start_heading_residual_rad{};
  double start_curvature_residual_per_m{};
  double goal_position_residual_m{};
  double goal_heading_residual_rad{};
  double goal_curvature_residual_per_m{};
};

struct PathSmoothingMetadata {
  std::string smoother_version;
  std::string solver_status;
  std::uint64_t limits_version{};
  double maximum_constraint_residual{};
  double maximum_absolute_curvature_per_m{};
  double maximum_absolute_curvature_rate_per_m2{};
  PathConstraintResiduals residuals;
};

struct PathMetadata {
  std::uint64_t path_version{};
  std::string coordinate_frame;
  std::uint32_t reference_line_version{};
  std::string interpolation_rule;
  std::optional<PathSmoothingMetadata> smoothing;
};

struct GeometricPath {
  std::vector<PathPoint> points;
  PathMetadata metadata;
};

struct RangeMps {
  double minimum_mps{};
  double maximum_mps{};
};

struct RangeMps2 {
  double minimum_mps2{};
  double maximum_mps2{};
};

struct RangeN {
  double minimum_n{};
  double maximum_n{};
};

struct SpeedPayoutLimits {
  RangeMps ground_speed;
  RangeMps2 ground_acceleration;
  double maximum_lateral_acceleration_mps2{};
  RangeMps payout_speed;
  RangeMps2 payout_acceleration;
  double maximum_payout_tracking_error_mps{};
  RangeN tension;
  double maximum_stopping_distance_m{};
};

struct ExecutionSample {
  double arc_length_m{};
  Duration time_from_start;
  double ground_speed_mps{};
  double ground_acceleration_mps2{};
  double payout_speed_mps{};
  double payout_acceleration_mps2{};
  double tension_setpoint_n{};
};

struct ExecutionProfile {
  std::uint64_t version{};
  std::uint64_t operating_envelope_version{};
  std::string interpolation_rule;
  std::optional<double> stopping_point_arc_length_m;
  std::vector<ExecutionSample> samples;
  SpeedPayoutLimits approved_tracking_limits;
};

[[nodiscard]] bool same_execution_profile_content(
    const ExecutionProfile& left, const ExecutionProfile& right) noexcept;

class ExecutionProfileVersioner {
 public:
  explicit ExecutionProfileVersioner(
      std::uint64_t last_issued_version) noexcept;

  [[nodiscard]] ExecutionProfile assign_version(ExecutionProfile profile);

 private:
  std::uint64_t last_issued_version_{};
  std::optional<ExecutionProfile> last_profile_;
};

struct TimedPath {
  GeometricPath geometry;
  ExecutionProfile execution_profile;
};

struct Covariance2dM2 {
  double xx_m2{};
  double xy_m2{};
  double yx_m2{};
  double yy_m2{};
};

enum class SensorHealthMode { nominal, approved_degraded };
enum class PredictionMode { search, validation };

struct ErrorBudget {
  Covariance2dM2 robot_position_covariance_m2;
  std::vector<Covariance2dM2> touchdown_position_covariance_m2;
  std::optional<double> epsilon_robot;
  std::optional<double> epsilon_terrain_gradient_local;
  std::optional<double> epsilon_point;
  std::optional<double> epsilon_path;
  bool reference_is_deterministic{true};
  bool path_joint_risk_implemented{};
  bool terrain_gradient_path_joint_risk_implemented{};
  std::string calibration_dataset_id;
  std::string terrain_gradient_calibration_dataset_id;
  std::uint64_t terrain_gradient_policy_version{};
  std::uint64_t corridor_risk_policy_version{};
  std::uint64_t cable_model_version{};
  std::uint64_t uncertainty_envelope_version{};
  std::uint64_t uncertainty_envelope_generator_version{};
  std::uint64_t execution_operating_envelope_version{};
  std::string operating_domain_id;
  SensorHealthMode sensor_mode{SensorHealthMode::nominal};
  bool covariance_envelope_audit_passed{};
};

enum class PlanningState {
  success,
  path_valid,
  waiting_map,
  request_scout,
  no_solution,
  no_solution_under_covariance_envelope,
  covariance_envelope_breach,
  input_invalid,
  map_expired,
  timeout,
  communication_degraded,
  manual_override,
  init,
  normal_planning,
  planning_with_caution,
  emergency_stop,
};

enum class CableModelValidity {
  valid,
  initial_state_uncertain,
  payout_tracking_out_of_range,
  tension_out_of_range,
  lag_angle_out_of_range,
  motion_mode_out_of_range,
  input_invalid,
  sensor_mode_unapproved,
  operating_domain_mismatch,
  execution_envelope_version_mismatch,
  covariance_invalid,
};

[[nodiscard]] std::string_view to_string(CableModelValidity validity) noexcept;

enum class CableValidationStatus { pass, marginal, violation };

enum class CableCorridorPointBasis {
  below_nominal_bound,
  within_absolute_bound,
  at_or_above_absolute_bound,
};

enum class CorridorEvaluationValidity {
  valid,
  risk_policy_missing,
  input_invalid,
  reference_version_mismatch,
  coordinate_transform_error_missing,
  covariance_invalid,
  distribution_not_calibrated,
  envelope_missing,
  envelope_version_mismatch,
  covariance_envelope_breach,
};

struct CableCorridorPointResult {
  CableValidationStatus status{CableValidationStatus::violation};
  double mean_lateral_error_m{};
  double lateral_stddev_m{};
  double upper_bound_m{};
  CableCorridorPointBasis basis{
      CableCorridorPointBasis::at_or_above_absolute_bound};
  double touchdown_arc_length_m{};
  double reference_progress_m{};
};

struct CableCorridorIntervalBoundCertificate {
  // Certifies d_upper(s) <= lerp(endpoint d_upper) + upper_bound_error_m[i].
  std::uint64_t version{};
  std::vector<double> upper_bound_error_m;
};

struct CableCorridorResult {
  CorridorEvaluationValidity validity{
      CorridorEvaluationValidity::risk_policy_missing};
  bool hard_feasible{};
  std::vector<CableCorridorPointResult> points;
  std::uint64_t marginal_count{};
  std::uint64_t violation_count{};
  double total_marginal_length_m{};
  double total_violation_length_m{};
  double maximum_marginal_length_m{};
  bool marginal_length_limit_exceeded{};
  double epsilon_point{};
  std::uint64_t corridor_risk_policy_version{};
  std::uint32_t reference_line_version{};
  CableCorridorIntervalBoundCertificate interval_bound_certificate;
  MonotonicTime evaluation_timestamp;
  std::string operating_domain_id;
  std::string residual_distribution_calibration_dataset_id;
  bool reference_is_deterministic{true};
  bool covariance_includes_coordinate_transform_error{};
  bool covariance_envelope_audit_performed{};
  bool path_joint_risk_implemented{};
  std::string risk_semantics;
  std::vector<std::string> issues;
};

enum class CableLayingFailure {
  none,
  curvature_exceeded,
  support_proxy_exceeded,
  forbidden_area_intersection,
  terrain_data_invalid,
  numerically_invalid,
  duplicate_touchdown_point,
  mechanical_history_incomplete,
};

struct CableLayingFailureSegment {
  CableLayingFailure reason{CableLayingFailure::none};
  double start_arc_length_m{};
  double end_arc_length_m{};
  Vector2m representative_position_m;
};

struct CableLayingEvaluation {
  // valid reports whether evaluation completed. hard_feasible is meaningful
  // only when valid is true; the validator rejects the contradictory pair.
  bool valid{};
  bool hard_feasible{};
  std::vector<CableLayingFailure> failure_reasons;
  std::vector<CableLayingFailureSegment> failure_segments;
  std::uint64_t limits_version{};
  std::uint64_t terrain_map_sequence{};
  std::uint64_t terrain_analysis_config_version{};
  std::string operating_domain_id;
  std::string risk_semantics;
  double maximum_absolute_curvature_per_m{};
  std::optional<Vector2m> maximum_absolute_curvature_position_m;
  double maximum_support_proxy_range_m{};
  std::optional<Vector2m> maximum_support_proxy_position_m;
  double terminal_support_window_length_m{};
  double soft_cost{};
  CableConstraintMemory terminal_memory;
};

struct MapVersion {
  std::string map_id;
  std::uint64_t sequence_number{};
  MonotonicTime timestamp;
  std::string coordinate_frame;
};

[[nodiscard]] bool operator==(const MapVersion& left,
                              const MapVersion& right) noexcept;
[[nodiscard]] bool operator!=(const MapVersion& left,
                              const MapVersion& right) noexcept;

struct PlanningDependencyVersions {
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
  // Version of the immutable cable-construction corridor snapshot.
  std::uint32_t cable_corridor_version{};
};

[[nodiscard]] bool operator==(const PlanningDependencyVersions& left,
                              const PlanningDependencyVersions& right) noexcept;
[[nodiscard]] bool operator!=(const PlanningDependencyVersions& left,
                              const PlanningDependencyVersions& right) noexcept;

enum class DiagnosticSeverity { info, warning, error };

struct DiagnosticEntry {
  DiagnosticSeverity severity{DiagnosticSeverity::error};
  std::string code;
  std::string stage;
  std::string message;
  MonotonicTime timestamp;
};

struct Diagnostics {
  std::string schema_version;
  std::uint64_t random_seed{};
  std::string input_version;
  std::string unit_system;
  std::string operating_domain_id;
  std::string risk_semantics;
  PlanningDependencyVersions dependencies;
  std::vector<DiagnosticEntry> entries;
};

struct PlanningResult {
  std::uint64_t sequence_number{};
  MonotonicTime timestamp;
  Duration validity_duration;
  PlanningState state{PlanningState::input_invalid};
  TimedPath robot_trajectory;
  GeometricPath cable_path;
  CableState terminal_cable_state;
  CableModelValidity cable_model_validity{
      CableModelValidity::initial_state_uncertain};
  CableCorridorResult corridor_result;
  CableLayingEvaluation cable_laying_result;
  ErrorBudget error_budget;
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
  Diagnostics diagnostics;

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

struct ValidationResult {
  bool valid{};
  std::vector<std::string> issues;
};

// Returns the unique representative in [-pi, pi). Rejects NaN and infinity.
[[nodiscard]] double normalize_angle_radians(double angle_rad);
[[nodiscard]] std::string_view to_string(PlanningState state);
[[nodiscard]] ValidationResult validate(const RobotState& state);
[[nodiscard]] ValidationResult validate(const CableState& state);
[[nodiscard]] ValidationResult validate(const ReferenceProgress& progress);
[[nodiscard]] ValidationResult validate(const GeometricPath& path);
[[nodiscard]] ValidationResult validate(const TimedPath& path);
// Validates an explicitly derived authorized prefix.  Unlike a complete
// trajectory, the prefix may end while moving and retain a stop point beyond
// its geometry; callers must only use this for a prefix cut from an approved
// TimedPath.
[[nodiscard]] ValidationResult validate_authorized_prefix(const TimedPath& path);
[[nodiscard]] ValidationResult validate_execution_profile_revision(
    const ExecutionProfile& previous, const ExecutionProfile& revised);
[[nodiscard]] ValidationResult validate(const ErrorBudget& budget);
[[nodiscard]] ValidationResult validate(const Covariance2dM2& covariance);
[[nodiscard]] ValidationResult validate(const PlanningResult& result);
[[nodiscard]] std::string serialize_planning_result(const PlanningResult& result);
[[nodiscard]] PlanningResult deserialize_planning_result(
    const std::string& serialized);

}  // namespace underwater_planner::core
