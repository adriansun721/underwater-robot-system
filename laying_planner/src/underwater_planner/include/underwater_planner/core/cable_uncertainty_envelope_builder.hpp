#pragma once

#include "underwater_planner/core/cable_model.hpp"
#include "underwater_planner/core/versioned_snapshot.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace underwater_planner::core {

inline constexpr char kPointwiseEnvelopeRiskSemantics[] =
    "POINTWISE_ENVELOPE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE";

struct ClosedRange {
  double minimum{};
  double maximum{};
};

struct InitialUncertaintyBounds {
  double robot_x_variance_m2{};
  double robot_y_variance_m2{};
  double robot_heading_variance_rad2{};
  double initial_lag_variance_rad2{};
  double robot_position_process_variance_per_m_m2{};
  double robot_heading_process_variance_per_m_rad2{};
};

struct SensorModeUncertaintyBounds {
  SensorHealthMode mode{SensorHealthMode::nominal};
  std::uint64_t version{};
  std::string calibration_dataset_id;
  double additional_position_variance_per_axis_m2{};
  double additional_heading_variance_rad2{};
  double additional_heading_process_variance_per_m_rad2{};
};

struct CertifiedMotionPrimitive {
  std::uint64_t version{};
  double arc_length_m{};
  double curvature_per_m{};
  double minimum_duration_s{};
  double maximum_duration_s{};
  double minimum_reference_progress_advance_m{};
  double maximum_reference_progress_advance_m{};
  SpeedPayoutLimits certified_execution_limits;
};

struct ReachabilityBinning {
  double arc_length_bin_m{};
  double heading_bin_rad{};
  double lag_angle_bin_rad{};
  double reference_progress_bin_m{};
  double time_bin_s{};
};

struct EnvelopeMarginCertification {
  std::uint64_t version{};
  std::string calibration_dataset_id;
  double numerical_integration_stddev_m{};
  double reference_normal_sweep_stddev_m{};
  double statistical_quantile_stddev_m{};
};

struct EnvelopeMarginBudget {
  std::uint64_t certification_version{};
  std::string calibration_dataset_id;
  double state_binning_stddev_m{};
  double numerical_integration_stddev_m{};
  double reference_normal_sweep_stddev_m{};
  double statistical_quantile_stddev_m{};
};

[[nodiscard]] double envelope_discretization_margin_stddev_m(
    const EnvelopeMarginBudget& margin) noexcept;
[[nodiscard]] double total_envelope_margin_stddev_m(
    const EnvelopeMarginBudget& margin) noexcept;

struct ExecutionUncertaintyCertification {
  std::uint64_t version{};
  std::uint64_t execution_operating_envelope_version{};
  std::string calibration_dataset_id;
  double payout_tracking_variance_upper_bound_m2{};
  double payout_acceleration_variance_upper_bound_m2{};
  double tension_variance_upper_bound_m2{};
};

struct OperatingDomain {
  std::uint64_t version{};
  std::string operating_domain_id;
  std::string certification_dataset_id;
  std::uint64_t primitive_set_version{};
  std::uint64_t initial_uncertainty_version{};
  double reference_progress_start_m{};
  double reference_progress_end_m{};
  double maximum_planning_length_m{};
  double maximum_planning_time_s{};
  ClosedRange initial_lag_angle_rad;
  InitialUncertaintyBounds initial_uncertainty;
  ExecutionUncertaintyCertification execution_uncertainty;
  std::vector<SensorModeUncertaintyBounds> approved_sensor_uncertainty;
  std::vector<CertifiedMotionPrimitive> allowed_primitives;
  ReachabilityBinning binning;
  EnvelopeMarginCertification margin_certification;
  std::size_t maximum_reachable_sets{};
};

enum class EnvelopeBuildValidity {
  valid,
  input_invalid,
  dependency_mismatch,
  sensor_mode_unapproved,
  resource_limit_exceeded,
};

struct EnvelopeDependencies {
  std::uint64_t generator_version{};
  std::uint64_t cable_model_version{};
  std::uint64_t execution_operating_envelope_version{};
  std::uint32_t reference_line_version{};
  std::uint64_t operating_domain_version{};
  std::uint64_t primitive_set_version{};
  std::uint64_t initial_uncertainty_version{};
  std::uint64_t sensor_uncertainty_version{};
  std::uint64_t execution_uncertainty_version{};
  std::uint64_t margin_certification_version{};
  SensorHealthMode sensor_mode{SensorHealthMode::nominal};
  std::string operating_domain_id;
  std::string cable_model_calibration_dataset_id;
  std::string certification_dataset_id;
  std::string sensor_calibration_dataset_id;
  std::string execution_uncertainty_calibration_dataset_id;
  std::string margin_calibration_dataset_id;
};

struct CableUncertaintyEnvelopeSegment {
  double reference_progress_start_m{};
  double reference_progress_end_m{};
  double lateral_variance_upper_bound_m2{};
  double lateral_stddev_upper_bound_m{};
};

[[nodiscard]] bool operator==(
    const CableUncertaintyEnvelopeSegment& left,
    const CableUncertaintyEnvelopeSegment& right) noexcept;

struct ReachableSetCertificate {
  std::uint64_t id{};
  std::uint64_t parent_id{};
  std::uint64_t primitive_version{};
  double path_length_m{};
  ClosedRange elapsed_time_s;
  ClosedRange heading_rad;
  ClosedRange lag_angle_rad;
  ClosedRange reference_progress_m;
  ClosedRange swept_reference_progress_m;
  double lateral_variance_upper_bound_m2{};
};

[[nodiscard]] bool operator==(const ReachableSetCertificate& left,
                              const ReachableSetCertificate& right) noexcept;

struct EnvelopeGenerationStats {
  std::size_t retained_reachable_set_count{};
  std::size_t containment_pruned_count{};
  std::size_t risk_bound_pruned_count{};
  std::size_t maximum_incomparable_sets_in_bin{};
};

struct CableUncertaintyEnvelope {
  EnvelopeBuildValidity validity{EnvelopeBuildValidity::input_invalid};
  EnvelopeDependencies dependencies;
  EnvelopeMarginBudget margin_budget;
  std::vector<CableUncertaintyEnvelopeSegment> segments;
  std::vector<ReachableSetCertificate> reachable_set_certificates;
  EnvelopeGenerationStats generation_stats;
  MonotonicTime generation_timestamp;
  bool path_joint_risk_implemented{};
  std::string risk_semantics;
  std::vector<DiagnosticEntry> diagnostics;
};

class CableUncertaintyEnvelopeBuilder {
 public:
  [[nodiscard]] CableUncertaintyEnvelope buildCertifiedUpperBound(
      const OperatingDomain& gamma_h, const ReferenceLine& reference,
      SensorHealthMode sensor_mode, const CableModelParameters& model,
      const ExecutionOperatingEnvelope& execution_envelope,
      std::uint64_t generator_version,
      MonotonicTime generation_timestamp) const;
};

}  // namespace underwater_planner::core
