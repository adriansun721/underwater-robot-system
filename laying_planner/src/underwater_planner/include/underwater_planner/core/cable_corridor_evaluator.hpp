#pragma once

#include "underwater_planner/core/data_contract.hpp"
#include "underwater_planner/core/versioned_snapshot.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace underwater_planner::core {

struct CableCorridorRiskPolicy {
  std::uint64_t version{};
  std::string residual_distribution_calibration_dataset_id;
  std::string operating_domain_id;
  double epsilon_point{};
  double nominal_half_width_m{};
  double absolute_half_width_m{};
  double maximum_marginal_length_m{};
  double marginal_boundary_margin_m{};
  bool residual_distribution_calibrated{};
};

struct CableCorridorEvaluationInput {
  ReferenceLine reference_line;
  GeometricPath touchdown_path;
  std::vector<double> reference_progress_m;
  std::vector<Covariance2dM2> touchdown_covariance_m2;
  CableCorridorIntervalBoundCertificate interval_bound_certificate;
  MonotonicTime evaluation_timestamp;
  bool reference_is_deterministic{true};
  bool covariance_includes_coordinate_transform_error{};
};

struct CableCorridorSearchBound {
  CorridorEvaluationValidity validity{CorridorEvaluationValidity::input_invalid};
  CableValidationStatus status{CableValidationStatus::violation};
  CableCorridorPointBasis basis{
      CableCorridorPointBasis::at_or_above_absolute_bound};
  double mean_lateral_error_m{};
  double lateral_stddev_upper_bound_m{};
  double upper_bound_m{};
  bool hard_feasible{};
};

[[nodiscard]] double two_sided_standard_normal_quantile(
    double epsilon_point) noexcept;
[[nodiscard]] CorridorEvaluationValidity validate_search_corridor_risk_policy(
    const CableCorridorRiskPolicy& policy) noexcept;
[[nodiscard]] CableCorridorSearchBound evaluate_search_corridor_bound(
    const CableCorridorRiskPolicy& policy,
    const ReferencePoint& reference_point,
    Vector2m touchdown_position_m,
    double lateral_stddev_upper_bound_m,
    double cable_sweep_margin_m) noexcept;

class CableCorridorEvaluator {
 public:
  explicit CableCorridorEvaluator(CableCorridorRiskPolicy policy);

  // T17 performs pointwise classification. T19/T29 must audit the statistical
  // envelope before a result can be published as a validated plan.
  [[nodiscard]] CableCorridorResult evaluate_pointwise(
      const CableCorridorEvaluationInput& input) const;

 private:
  CableCorridorRiskPolicy policy_;
};

}  // namespace underwater_planner::core
