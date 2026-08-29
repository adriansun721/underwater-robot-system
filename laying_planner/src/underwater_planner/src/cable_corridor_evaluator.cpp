#include "underwater_planner/core/cable_corridor_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace underwater_planner::core {
namespace {

constexpr const char* kPointwiseRiskSemantics =
    "POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE";
constexpr double kReferenceFrameTolerance = 1.0e-9;

bool finite(const double value) { return std::isfinite(value); }

double standard_normal_cdf(const double value) {
  return 0.5 * std::erfc(-value / std::sqrt(2.0));
}

double inverse_standard_normal(const double probability) {
  double lower = -10.0;
  double upper = 10.0;
  for (int iteration = 0; iteration < 80; ++iteration) {
    const double middle = 0.5 * (lower + upper);
    if (standard_normal_cdf(middle) < probability) {
      lower = middle;
    } else {
      upper = middle;
    }
  }
  return 0.5 * (lower + upper);
}

bool reference_frames_are_orthonormal(const ReferenceLine& line) {
  return std::all_of(
      line.points.begin(), line.points.end(),
      [](const ReferencePoint& point) {
        return std::abs(std::hypot(point.tangent_x, point.tangent_y) - 1.0) <=
                   kReferenceFrameTolerance &&
               std::abs(std::hypot(point.normal_x, point.normal_y) - 1.0) <=
                   kReferenceFrameTolerance &&
               std::abs(point.normal_x + point.tangent_y) <=
                   kReferenceFrameTolerance &&
               std::abs(point.normal_y - point.tangent_x) <=
                   kReferenceFrameTolerance;
      });
}

struct ArcInterval {
  double start_arc_length_m{};
  double end_arc_length_m{};
};

CableValidationStatus classify_bound(const double upper_bound_m,
                                     const CableCorridorRiskPolicy& policy) {
  if (upper_bound_m < policy.nominal_half_width_m) {
    return CableValidationStatus::pass;
  }
  if (upper_bound_m < policy.absolute_half_width_m) {
    return CableValidationStatus::marginal;
  }
  return CableValidationStatus::violation;
}

void append_interval(std::vector<ArcInterval>& intervals,
                     const double start_arc_length_m,
                     const double end_arc_length_m) {
  if (!(end_arc_length_m > start_arc_length_m)) return;
  constexpr double kMergeToleranceM = 1.0e-12;
  if (!intervals.empty() &&
      start_arc_length_m <=
          intervals.back().end_arc_length_m + kMergeToleranceM) {
    intervals.back().end_arc_length_m =
        std::max(intervals.back().end_arc_length_m, end_arc_length_m);
    return;
  }
  intervals.push_back({start_arc_length_m, end_arc_length_m});
}

double interval_length(const std::vector<ArcInterval>& intervals) {
  double total = 0.0;
  for (const ArcInterval& interval : intervals) {
    total += interval.end_arc_length_m - interval.start_arc_length_m;
  }
  return total;
}

void add_threshold_crossing(std::vector<double>& fractions,
                            const double left_bound_m,
                            const double right_bound_m,
                            const double threshold_m) {
  if (left_bound_m == right_bound_m) return;
  const double fraction =
      (threshold_m - left_bound_m) / (right_bound_m - left_bound_m);
  if (fraction > 0.0 && fraction < 1.0) fractions.push_back(fraction);
}

void accumulate_classified_intervals(
    const std::vector<CableCorridorPointResult>& points,
    const CableCorridorRiskPolicy& policy,
    const std::vector<double>& interval_upper_bound_error_m,
    std::vector<ArcInterval>& marginal_intervals,
    std::vector<ArcInterval>& violation_intervals) {
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const CableCorridorPointResult& left = points[index - 1U];
    const CableCorridorPointResult& right = points[index];
    const double span_m =
        right.touchdown_arc_length_m - left.touchdown_arc_length_m;
    const double interval_error_m =
        interval_upper_bound_error_m[index - 1U];
    const double conservative_left_bound_m =
        left.upper_bound_m + interval_error_m;
    const double conservative_right_bound_m =
        right.upper_bound_m + interval_error_m;
    std::vector<double> fractions{0.0, 1.0};
    add_threshold_crossing(fractions, conservative_left_bound_m,
                           conservative_right_bound_m,
                           policy.nominal_half_width_m);
    add_threshold_crossing(fractions, conservative_left_bound_m,
                           conservative_right_bound_m,
                           policy.absolute_half_width_m);
    std::sort(fractions.begin(), fractions.end());
    fractions.erase(std::unique(fractions.begin(), fractions.end()),
                    fractions.end());
    for (std::size_t part = 1U; part < fractions.size(); ++part) {
      const double lower_fraction = fractions[part - 1U];
      const double upper_fraction = fractions[part];
      const double midpoint_fraction =
          0.5 * (lower_fraction + upper_fraction);
      const double midpoint_bound =
          conservative_left_bound_m +
          midpoint_fraction *
              (conservative_right_bound_m - conservative_left_bound_m);
      const double start_m =
          left.touchdown_arc_length_m + lower_fraction * span_m;
      const double end_m =
          left.touchdown_arc_length_m + upper_fraction * span_m;
      switch (classify_bound(midpoint_bound, policy)) {
        case CableValidationStatus::pass:
          break;
        case CableValidationStatus::marginal:
          append_interval(marginal_intervals, start_m, end_m);
          break;
        case CableValidationStatus::violation:
          append_interval(violation_intervals, start_m, end_m);
          break;
      }
    }
  }
}

std::vector<ArcInterval> expanded_intervals(
    const std::vector<ArcInterval>& intervals, const double margin_m,
    const double path_start_m, const double path_end_m) {
  std::vector<ArcInterval> result;
  for (const ArcInterval& interval : intervals) {
    append_interval(result, std::max(path_start_m,
                                     interval.start_arc_length_m - margin_m),
                    std::min(path_end_m,
                             interval.end_arc_length_m + margin_m));
  }
  return result;
}

CableCorridorResult result_shell(const CableCorridorRiskPolicy& policy,
                                 const CableCorridorEvaluationInput& input) {
  CableCorridorResult result;
  result.maximum_marginal_length_m = policy.maximum_marginal_length_m;
  result.epsilon_point = policy.epsilon_point;
  result.corridor_risk_policy_version = policy.version;
  result.reference_line_version = input.reference_line.version;
  result.interval_bound_certificate.version =
      input.interval_bound_certificate.version;
  result.evaluation_timestamp = input.evaluation_timestamp;
  result.operating_domain_id = policy.operating_domain_id;
  result.residual_distribution_calibration_dataset_id =
      policy.residual_distribution_calibration_dataset_id;
  result.reference_is_deterministic = input.reference_is_deterministic;
  result.covariance_includes_coordinate_transform_error =
      input.covariance_includes_coordinate_transform_error;
  result.covariance_envelope_audit_performed = false;
  result.path_joint_risk_implemented = false;
  result.risk_semantics = kPointwiseRiskSemantics;
  return result;
}

}  // namespace

double two_sided_standard_normal_quantile(
    const double epsilon_point) noexcept {
  if (!(epsilon_point > 0.0 && epsilon_point < 1.0)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return inverse_standard_normal(1.0 - epsilon_point / 2.0);
}

CorridorEvaluationValidity validate_search_corridor_risk_policy(
    const CableCorridorRiskPolicy& policy) noexcept {
  const double quantile_probability = 1.0 - policy.epsilon_point / 2.0;
  if (policy.version == 0U ||
      policy.residual_distribution_calibration_dataset_id.empty() ||
      policy.operating_domain_id.empty() ||
      !(policy.epsilon_point > 0.0 && policy.epsilon_point < 1.0) ||
      !(quantile_probability > 0.5 && quantile_probability < 1.0)) {
    return CorridorEvaluationValidity::risk_policy_missing;
  }
  if (!policy.residual_distribution_calibrated) {
    return CorridorEvaluationValidity::distribution_not_calibrated;
  }
  if (!finite(policy.nominal_half_width_m) ||
      policy.nominal_half_width_m <= 0.0 ||
      !finite(policy.absolute_half_width_m) ||
      policy.absolute_half_width_m <= policy.nominal_half_width_m) {
    return CorridorEvaluationValidity::input_invalid;
  }
  return CorridorEvaluationValidity::valid;
}

CableCorridorSearchBound evaluate_search_corridor_bound(
    const CableCorridorRiskPolicy& policy,
    const ReferencePoint& reference_point,
    const Vector2m touchdown_position_m,
    const double lateral_stddev_upper_bound_m,
    const double cable_sweep_margin_m) noexcept {
  CableCorridorSearchBound result;
  result.validity = validate_search_corridor_risk_policy(policy);
  if (result.validity != CorridorEvaluationValidity::valid) {
    return result;
  }
  if (!finite(reference_point.x_m) || !finite(reference_point.y_m) ||
      !finite(reference_point.normal_x) ||
      !finite(reference_point.normal_y) ||
      !finite(touchdown_position_m.x_m) ||
      !finite(touchdown_position_m.y_m) ||
      !finite(lateral_stddev_upper_bound_m) ||
      lateral_stddev_upper_bound_m < 0.0 ||
      !finite(cable_sweep_margin_m) || cable_sweep_margin_m < 0.0) {
    result.validity = CorridorEvaluationValidity::input_invalid;
    return result;
  }

  result.mean_lateral_error_m =
      reference_point.normal_x *
          (touchdown_position_m.x_m - reference_point.x_m) +
      reference_point.normal_y *
          (touchdown_position_m.y_m - reference_point.y_m);
  result.lateral_stddev_upper_bound_m = lateral_stddev_upper_bound_m;
  result.upper_bound_m =
      std::abs(result.mean_lateral_error_m) +
      two_sided_standard_normal_quantile(policy.epsilon_point) *
          lateral_stddev_upper_bound_m +
      cable_sweep_margin_m;
  if (!finite(result.mean_lateral_error_m) || !finite(result.upper_bound_m)) {
    result.validity = CorridorEvaluationValidity::input_invalid;
    return result;
  }
  result.status = classify_bound(result.upper_bound_m, policy);
  switch (result.status) {
    case CableValidationStatus::pass:
      result.basis = CableCorridorPointBasis::below_nominal_bound;
      break;
    case CableValidationStatus::marginal:
      result.basis = CableCorridorPointBasis::within_absolute_bound;
      break;
    case CableValidationStatus::violation:
      result.basis = CableCorridorPointBasis::at_or_above_absolute_bound;
      break;
  }
  result.hard_feasible = result.status != CableValidationStatus::violation;
  result.validity = CorridorEvaluationValidity::valid;
  return result;
}

CableCorridorEvaluator::CableCorridorEvaluator(CableCorridorRiskPolicy policy)
    : policy_(std::move(policy)) {}

CableCorridorResult CableCorridorEvaluator::evaluate_pointwise(
    const CableCorridorEvaluationInput& input) const {
  CableCorridorResult result = result_shell(policy_, input);
  const double quantile_probability = 1.0 - policy_.epsilon_point / 2.0;
  if (policy_.version == 0U ||
      policy_.residual_distribution_calibration_dataset_id.empty() ||
      policy_.operating_domain_id.empty() ||
      !(policy_.epsilon_point > 0.0 && policy_.epsilon_point < 1.0) ||
      !(quantile_probability > 0.5 && quantile_probability < 1.0)) {
    result.validity = CorridorEvaluationValidity::risk_policy_missing;
    result.issues.emplace_back(
        "epsilon_point, risk policy version, and calibration dataset are required");
    return result;
  }
  if (!policy_.residual_distribution_calibrated) {
    result.validity =
        CorridorEvaluationValidity::distribution_not_calibrated;
    result.issues.emplace_back(
        "touchdown lateral residual distribution is not calibrated");
    return result;
  }
  if (!input.reference_is_deterministic) {
    result.validity = CorridorEvaluationValidity::input_invalid;
    result.issues.emplace_back(
        "the first-version evaluator requires a deterministic reference line");
    return result;
  }
  if (!input.covariance_includes_coordinate_transform_error) {
    result.validity =
        CorridorEvaluationValidity::coordinate_transform_error_missing;
    result.issues.emplace_back(
        "touchdown covariance omits reference-frame transform uncertainty");
    return result;
  }
  if (input.reference_line.version == 0U ||
      input.touchdown_path.metadata.reference_line_version !=
          input.reference_line.version) {
    result.validity =
        CorridorEvaluationValidity::reference_version_mismatch;
    result.issues.emplace_back(
        "touchdown path and reference line versions do not match");
    return result;
  }
  const SnapshotValidation reference_validation =
      validate(input.reference_line);
  const ValidationResult path_validation = validate(input.touchdown_path);
  if (!reference_validation.valid || !path_validation.valid ||
      !reference_frames_are_orthonormal(input.reference_line)) {
    result.validity = CorridorEvaluationValidity::input_invalid;
    result.issues.emplace_back(
        "reference line or touchdown path contract is invalid");
    return result;
  }
  const std::size_t count = input.touchdown_path.points.size();
  if (count == 0U || input.reference_progress_m.size() != count ||
      input.touchdown_covariance_m2.size() != count ||
      input.interval_bound_certificate.version == 0U ||
      input.interval_bound_certificate.upper_bound_error_m.size() !=
          count - 1U ||
      input.touchdown_path.metadata.coordinate_frame !=
          input.reference_line.coordinate_frame ||
      input.touchdown_path.metadata.interpolation_rule !=
          "piecewise-linear-in-arc-length" ||
      input.evaluation_timestamp.nanoseconds < 0 ||
      !finite(policy_.nominal_half_width_m) ||
      !(policy_.nominal_half_width_m > 0.0) ||
      !finite(policy_.absolute_half_width_m) ||
      !(policy_.absolute_half_width_m > policy_.nominal_half_width_m) ||
      !finite(policy_.maximum_marginal_length_m) ||
      policy_.maximum_marginal_length_m < 0.0 ||
      !finite(policy_.marginal_boundary_margin_m) ||
      !(policy_.marginal_boundary_margin_m > 0.0)) {
    result.validity = CorridorEvaluationValidity::input_invalid;
    result.issues.emplace_back(
        "corridor path, reference progress, covariance, or policy is invalid");
    return result;
  }
  if (!std::all_of(input.interval_bound_certificate.upper_bound_error_m.begin(),
                   input.interval_bound_certificate.upper_bound_error_m.end(),
                   [](const double error_m) {
                     return finite(error_m) && error_m >= 0.0;
                   })) {
    result.validity = CorridorEvaluationValidity::input_invalid;
    result.issues.emplace_back(
        "interval upper-bound errors must be finite and nonnegative");
    return result;
  }
  result.interval_bound_certificate.upper_bound_error_m =
      input.interval_bound_certificate.upper_bound_error_m;

  result.points.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    const PathPoint& touchdown = input.touchdown_path.points[index];
    const std::optional<ReferencePoint> reference =
        input.reference_line.query(input.reference_progress_m[index]);
    const Covariance2dM2& covariance =
        input.touchdown_covariance_m2[index];
    if (!reference.has_value() || !finite(touchdown.arc_length_m) ||
        !finite(touchdown.x_m) || !finite(touchdown.y_m) ||
        (index > 0U &&
         touchdown.arc_length_m <=
             input.touchdown_path.points[index - 1U].arc_length_m) ||
        !validate(covariance).valid) {
      result.validity = reference.has_value()
                            ? CorridorEvaluationValidity::covariance_invalid
                            : CorridorEvaluationValidity::input_invalid;
      result.issues.emplace_back(
          "corridor sample has invalid reference progress or covariance");
      result.points.clear();
      return result;
    }

    const double variance =
        reference->normal_x *
            (covariance.xx_m2 * reference->normal_x +
             covariance.xy_m2 * reference->normal_y) +
        reference->normal_y *
            (covariance.yx_m2 * reference->normal_x +
             covariance.yy_m2 * reference->normal_y);
    if (!finite(variance) || variance < -1.0e-12) {
      result.validity = CorridorEvaluationValidity::covariance_invalid;
      result.issues.emplace_back(
          "projected touchdown covariance is invalid");
      result.points.clear();
      return result;
    }

    const double standard_deviation = std::sqrt(std::max(0.0, variance));
    const CableCorridorSearchBound bound = evaluate_search_corridor_bound(
        policy_, *reference, {touchdown.x_m, touchdown.y_m},
        standard_deviation, 0.0);
    if (bound.validity != CorridorEvaluationValidity::valid) {
      result.validity = bound.validity;
      result.issues.emplace_back("corridor point bound is invalid");
      result.points.clear();
      return result;
    }
    CableCorridorPointResult point;
    point.touchdown_arc_length_m = touchdown.arc_length_m;
    point.reference_progress_m = input.reference_progress_m[index];
    point.mean_lateral_error_m = bound.mean_lateral_error_m;
    point.lateral_stddev_m = standard_deviation;
    point.upper_bound_m = bound.upper_bound_m;
    point.status = bound.status;
    point.basis = bound.basis;
    if (point.status == CableValidationStatus::marginal) {
      ++result.marginal_count;
    } else if (point.status == CableValidationStatus::violation) {
      ++result.violation_count;
    }
    result.points.push_back(point);
  }

  std::vector<ArcInterval> marginal_intervals;
  std::vector<ArcInterval> violation_intervals;
  accumulate_classified_intervals(
      result.points, policy_,
      result.interval_bound_certificate.upper_bound_error_m,
      marginal_intervals, violation_intervals);
  const std::vector<ArcInterval> conservative_marginal_intervals =
      expanded_intervals(marginal_intervals,
                         policy_.marginal_boundary_margin_m,
                         result.points.front().touchdown_arc_length_m,
                         result.points.back().touchdown_arc_length_m);
  result.total_marginal_length_m =
      interval_length(conservative_marginal_intervals);
  result.total_violation_length_m = interval_length(violation_intervals);
  result.marginal_length_limit_exceeded =
      result.total_marginal_length_m > policy_.maximum_marginal_length_m;
  result.validity = CorridorEvaluationValidity::valid;
  result.hard_feasible = result.violation_count == 0U &&
                         result.total_violation_length_m == 0.0 &&
                         !result.marginal_length_limit_exceeded;
  return result;
}

}  // namespace underwater_planner::core
