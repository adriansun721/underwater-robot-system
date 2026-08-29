#include "underwater_planner/core/cable_corridor_evaluator.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using underwater_planner::core::CableCorridorEvaluationInput;
using underwater_planner::core::CableCorridorEvaluator;
using underwater_planner::core::CableCorridorPointBasis;
using underwater_planner::core::CableCorridorRiskPolicy;
using underwater_planner::core::CableValidationStatus;
using underwater_planner::core::CorridorEvaluationValidity;
using underwater_planner::core::Covariance2dM2;
using underwater_planner::core::GeometricPath;
using underwater_planner::core::PathMetadata;
using underwater_planner::core::PathPoint;
using underwater_planner::core::ReferenceLine;
using underwater_planner::core::Vector2m;
using underwater_planner::core::make_reference_line;

void require(const bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void require_near(const double actual, const double expected,
                  const double tolerance, const std::string& message) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(message + ": actual=" + std::to_string(actual) +
                             " expected=" + std::to_string(expected));
  }
}

CableCorridorRiskPolicy policy() {
  CableCorridorRiskPolicy result;
  result.version = 17;
  result.residual_distribution_calibration_dataset_id =
      "corridor-residuals-v3";
  result.operating_domain_id = "competition-seabed-v1";
  result.epsilon_point = 0.04550026389635842;  // z_(1-epsilon/2) = 2.
  result.nominal_half_width_m = 0.5;
  result.absolute_half_width_m = 1.0;
  result.maximum_marginal_length_m = 6.0;
  result.marginal_boundary_margin_m = 0.05;
  result.residual_distribution_calibrated = true;
  return result;
}

ReferenceLine reference_line() {
  return make_reference_line(4, "map", {{0.0, 0.0}, {20.0, 0.0}});
}

GeometricPath touchdown_path(const std::vector<double>& arc_lengths_m,
                             const std::vector<double>& lateral_offsets_m) {
  GeometricPath result;
  result.metadata = PathMetadata{71, "map", 4,
                                 "piecewise-linear-in-arc-length"};
  for (std::size_t index = 0; index < arc_lengths_m.size(); ++index) {
    result.points.push_back(PathPoint{arc_lengths_m[index],
                                      arc_lengths_m[index],
                                      lateral_offsets_m[index], 0.0, 0.0});
  }
  return result;
}

CableCorridorEvaluationInput input(
    const std::vector<double>& arc_lengths_m,
    const std::vector<double>& lateral_offsets_m) {
  CableCorridorEvaluationInput result;
  result.reference_line = reference_line();
  result.touchdown_path =
      touchdown_path(arc_lengths_m, lateral_offsets_m);
  result.reference_progress_m = arc_lengths_m;
  result.touchdown_covariance_m2.assign(
      arc_lengths_m.size(), Covariance2dM2{0.04, 0.0, 0.0, 0.01});
  result.interval_bound_certificate.version = 3;
  if (!arc_lengths_m.empty()) {
    result.interval_bound_certificate.upper_bound_error_m.assign(
        arc_lengths_m.size() - 1U, 0.0);
  }
  result.evaluation_timestamp = {17'000};
  result.reference_is_deterministic = true;
  result.covariance_includes_coordinate_transform_error = true;
  return result;
}

void evaluates_touchdown_lateral_risk_and_boundary_classes() {
  // Design: 18.2.4-9
  // Design: 18.2.4-10
  // Design: 18.2.4-key-1
  const CableCorridorEvaluator evaluator(policy());
  const auto result =
      evaluator.evaluate_pointwise(input({0.0, 1.0, 2.0},
                                         {0.0, 0.3, 0.8}));

  require(result.validity == CorridorEvaluationValidity::valid,
          "a calibrated corridor evaluation was invalid");
  require(!result.hard_feasible && result.points.size() == 3,
          "a pointwise violation did not reject the candidate");
  require(result.points[0].status == CableValidationStatus::pass &&
              result.points[1].status == CableValidationStatus::marginal &&
              result.points[2].status == CableValidationStatus::violation,
          "PASS/MARGINAL/VIOLATION boundaries were not applied");
  require(result.points[0].basis ==
                  CableCorridorPointBasis::below_nominal_bound &&
              result.points[1].basis ==
                  CableCorridorPointBasis::within_absolute_bound &&
              result.points[2].basis ==
                  CableCorridorPointBasis::at_or_above_absolute_bound,
          "point classification omitted its auditable basis");
  require_near(result.points[1].mean_lateral_error_m, 0.3, 1.0e-12,
               "touchdown lateral mean was not projected on the reference normal");
  require_near(result.points[1].lateral_stddev_m, 0.1, 1.0e-12,
               "touchdown covariance was not projected on the reference normal");
  require_near(result.points[1].upper_bound_m, 0.5, 1.0e-10,
               "the conservative two-sided chance bound is wrong");
  require(result.corridor_risk_policy_version == 17 &&
              result.reference_line_version == 4 &&
              result.evaluation_timestamp.nanoseconds == 17'000 &&
              result.operating_domain_id == "competition-seabed-v1" &&
              result.interval_bound_certificate.version == 3 &&
              result.interval_bound_certificate.upper_bound_error_m ==
                  std::vector<double>({0.0, 0.0}) &&
              result.residual_distribution_calibration_dataset_id ==
                  "corridor-residuals-v3" &&
              result.reference_is_deterministic &&
              result.covariance_includes_coordinate_transform_error &&
              !result.covariance_envelope_audit_performed &&
              !result.path_joint_risk_implemented,
          "result omitted risk provenance or overstated joint-path risk");
}

void anisotropic_covariance_projection_rotates_with_reference_normal() {
  // Design: 18.2.4-8
  const auto horizontal_result = CableCorridorEvaluator(policy())
                                     .evaluate_pointwise(
                                         input({0.0, 1.0}, {0.0, 0.0}));
  auto vertical = input({0.0, 1.0}, {0.0, 0.0});
  vertical.reference_line =
      make_reference_line(4, "map", {{0.0, 0.0}, {0.0, 20.0}});
  vertical.touchdown_path.points[0].x_m = 0.0;
  vertical.touchdown_path.points[0].y_m = 0.0;
  vertical.touchdown_path.points[1].x_m = 0.0;
  vertical.touchdown_path.points[1].y_m = 1.0;
  vertical.touchdown_path.points[0].heading_rad = 0.5 * std::acos(-1.0);
  vertical.touchdown_path.points[1].heading_rad = 0.5 * std::acos(-1.0);
  const auto vertical_result =
      CableCorridorEvaluator(policy()).evaluate_pointwise(vertical);

  require(horizontal_result.validity == CorridorEvaluationValidity::valid &&
              vertical_result.validity == CorridorEvaluationValidity::valid,
          "valid rotated anisotropic covariance inputs were rejected");
  require_near(horizontal_result.points[1].lateral_stddev_m, 0.1, 1.0e-12,
               "horizontal reference did not project the y variance");
  require_near(vertical_result.points[1].lateral_stddev_m, 0.2, 1.0e-12,
               "vertical reference did not project the x variance");
}

void missing_risk_evidence_fails_closed() {
  // Design: 18.2.4-12
  auto missing_policy = policy();
  missing_policy.version = 0;
  require(CableCorridorEvaluator(missing_policy)
                  .evaluate_pointwise(input({0.0, 1.0}, {0.0, 0.0}))
                  .validity == CorridorEvaluationValidity::risk_policy_missing,
          "a missing corridor risk policy version was accepted");

  auto unrepresentable_tail = policy();
  unrepresentable_tail.epsilon_point =
      std::numeric_limits<double>::denorm_min();
  require(CableCorridorEvaluator(unrepresentable_tail)
                  .evaluate_pointwise(input({0.0, 1.0}, {0.0, 0.0}))
                  .validity == CorridorEvaluationValidity::risk_policy_missing,
          "an unrepresentable Gaussian tail probability was accepted");

  auto uncalibrated = policy();
  uncalibrated.residual_distribution_calibrated = false;
  require(CableCorridorEvaluator(uncalibrated)
                  .evaluate_pointwise(input({0.0, 1.0}, {0.0, 0.0}))
                  .validity ==
              CorridorEvaluationValidity::distribution_not_calibrated,
          "an uncalibrated residual distribution was accepted");

  auto missing_transform = input({0.0, 1.0}, {0.0, 0.0});
  missing_transform.covariance_includes_coordinate_transform_error = false;
  require(CableCorridorEvaluator(policy())
                  .evaluate_pointwise(missing_transform)
                  .validity ==
              CorridorEvaluationValidity::coordinate_transform_error_missing,
          "reference-frame transform uncertainty was silently omitted");

  auto nonfinite_width = policy();
  nonfinite_width.absolute_half_width_m =
      std::numeric_limits<double>::infinity();
  require(CableCorridorEvaluator(nonfinite_width)
                  .evaluate_pointwise(input({0.0, 1.0}, {0.0, 0.0}))
                  .validity == CorridorEvaluationValidity::input_invalid,
          "a non-finite absolute corridor hard bound was accepted");

  auto missing_discretization_margin = policy();
  missing_discretization_margin.marginal_boundary_margin_m = 0.0;
  require(CableCorridorEvaluator(missing_discretization_margin)
                  .evaluate_pointwise(input({0.0, 1.0}, {0.0, 0.0}))
                  .validity == CorridorEvaluationValidity::input_invalid,
          "a missing interval discretization margin was accepted");

  auto missing_sampling_contract = input({0.0, 1.0}, {0.0, 0.0});
  missing_sampling_contract.interval_bound_certificate.version = 0;
  require(CableCorridorEvaluator(policy())
                  .evaluate_pointwise(missing_sampling_contract)
                  .validity == CorridorEvaluationValidity::input_invalid,
          "an unversioned interval sampling contract was accepted");
}

void certified_interval_error_prevents_hidden_interior_violations() {
  auto coarse = input({0.0, 4.0}, {0.0, 0.0});
  coarse.reference_line =
      make_reference_line(4, "map", {{0.0, 0.0}, {2.0, -2.0}, {4.0, 0.0}});
  coarse.reference_progress_m = {
      coarse.reference_line.points.front().arc_length_m,
      coarse.reference_line.points.back().arc_length_m};
  coarse.interval_bound_certificate.upper_bound_error_m = {2.0};

  const auto coarse_result =
      CableCorridorEvaluator(policy()).evaluate_pointwise(coarse);
  require(coarse_result.validity == CorridorEvaluationValidity::valid &&
              !coarse_result.hard_feasible &&
              coarse_result.violation_count == 0 &&
              coarse_result.total_violation_length_m == 4.0,
          "certified interval error missed a violation between PASS endpoints");

  auto fine = input({0.0, 2.0, 4.0}, {0.0, 0.0, 0.0});
  fine.reference_line = coarse.reference_line;
  fine.reference_progress_m = {
      fine.reference_line.points.front().arc_length_m,
      fine.reference_line.points[1].arc_length_m,
      fine.reference_line.points.back().arc_length_m};
  const auto fine_result =
      CableCorridorEvaluator(policy()).evaluate_pointwise(fine);
  require(!fine_result.hard_feasible && fine_result.violation_count == 1,
          "explicit interior sampling did not expose the reference-line peak");
}

void marginal_length_uses_interval_intersection_and_a_hard_limit() {
  // Design: 18.2.4-34
  // Design: 18.2.4-key-7
  auto exact_policy = policy();
  exact_policy.maximum_marginal_length_m = 5.5;
  const auto samples = input({0.0, 2.0, 5.0, 9.0},
                             {0.0, 0.5, 0.5, 0.0});
  const auto exact =
      CableCorridorEvaluator(exact_policy).evaluate_pointwise(samples);
  require(exact.validity == CorridorEvaluationValidity::valid &&
              exact.hard_feasible,
          "an exact marginal interval below the hard limit was rejected");
  require_near(exact.total_marginal_length_m, 5.5, 1.0e-9,
               "marginal length was approximated from sample count");

  auto below_limit_policy = exact_policy;
  below_limit_policy.maximum_marginal_length_m = 5.6;
  const auto below = CableCorridorEvaluator(below_limit_policy)
                         .evaluate_pointwise(samples);
  require(below.hard_feasible &&
              !below.marginal_length_limit_exceeded,
          "marginal length below the hard limit was rejected");

  const auto resampled = input(
      {0.0, 1.0, 2.0, 3.5, 5.0, 7.0, 9.0},
      {0.0, 0.25, 0.5, 0.5, 0.5, 0.25, 0.0});
  const auto resampled_result =
      CableCorridorEvaluator(exact_policy).evaluate_pointwise(resampled);
  require(resampled_result.hard_feasible &&
              !resampled_result.path_joint_risk_implemented &&
              resampled_result.risk_semantics == exact.risk_semantics,
          "resampling changed feasibility or overstated joint-path risk");
  require_near(resampled_result.total_marginal_length_m,
               exact.total_marginal_length_m, 1.0e-9,
               "marginal arc length changed under linear resampling");

  auto conservative_policy = exact_policy;
  conservative_policy.marginal_boundary_margin_m = 0.1;
  const auto conservative =
      CableCorridorEvaluator(conservative_policy).evaluate_pointwise(samples);
  require(conservative.validity == CorridorEvaluationValidity::valid &&
              !conservative.hard_feasible &&
              conservative.marginal_length_limit_exceeded,
          "the mandatory marginal-length gate was bypassed");
  require_near(conservative.total_marginal_length_m, 5.6, 1.0e-9,
               "conservative interval-boundary margin was not accumulated");
}

void invalid_covariance_and_alignment_fail_closed_deterministically() {
  auto invalid_covariance = input({0.0, 1.0}, {0.0, 0.0});
  invalid_covariance.touchdown_covariance_m2[1] =
      Covariance2dM2{0.01, 0.02, 0.02, 0.01};
  require(CableCorridorEvaluator(policy())
                  .evaluate_pointwise(invalid_covariance)
                  .validity == CorridorEvaluationValidity::covariance_invalid,
          "a non-PSD covariance was accepted");

  auto subtly_invalid_covariance = input({0.0, 1.0}, {0.0, 0.0});
  subtly_invalid_covariance.touchdown_covariance_m2[1] =
      Covariance2dM2{1.0e-9, 1.0e-6, 1.0e-6, 1.0e-9};
  require(CableCorridorEvaluator(policy())
                  .evaluate_pointwise(subtly_invalid_covariance)
                  .validity == CorridorEvaluationValidity::covariance_invalid,
          "a small-scale covariance with a negative eigenvalue was accepted");

  auto large_scale_negative_diagonal = input({0.0, 1.0}, {0.0, 0.0});
  large_scale_negative_diagonal.touchdown_covariance_m2[1] =
      Covariance2dM2{-1.0, 0.0, 0.0, 1.0e12};
  require(CableCorridorEvaluator(policy())
                  .evaluate_pointwise(large_scale_negative_diagonal)
                  .validity == CorridorEvaluationValidity::covariance_invalid,
          "a large-scale covariance hid a negative eigenvalue in tolerance");

  auto misaligned = input({0.0, 1.0}, {0.0, 0.0});
  misaligned.reference_progress_m.pop_back();
  require(CableCorridorEvaluator(policy())
                  .evaluate_pointwise(misaligned)
                  .validity ==
              CorridorEvaluationValidity::input_invalid,
          "misaligned path/progress/covariance samples were accepted");

  auto malformed_reference = input({0.0, 1.0}, {0.0, 0.0});
  malformed_reference.reference_line.points.front().normal_x = 2.0;
  require(CableCorridorEvaluator(policy())
                  .evaluate_pointwise(malformed_reference)
                  .validity == CorridorEvaluationValidity::input_invalid,
          "a reference line with a non-unit frame was accepted");

  auto version_mismatch = input({0.0, 1.0}, {0.0, 0.0});
  version_mismatch.touchdown_path.metadata.reference_line_version = 5;
  require(CableCorridorEvaluator(policy())
                  .evaluate_pointwise(version_mismatch)
                  .validity ==
              CorridorEvaluationValidity::reference_version_mismatch,
          "a reference-line version mismatch lacked a distinct diagnosis");

  const auto stable_input = input({0.0, 1.0, 2.0}, {0.0, 0.3, 0.0});
  const auto first =
      CableCorridorEvaluator(policy()).evaluate_pointwise(stable_input);
  const auto second =
      CableCorridorEvaluator(policy()).evaluate_pointwise(stable_input);
  require(first.points.size() == second.points.size() &&
              first.total_marginal_length_m ==
                  second.total_marginal_length_m &&
              first.risk_semantics == second.risk_semantics &&
              first.issues == second.issues,
          "identical corridor inputs did not reproduce an identical result");
}

}  // namespace

int main() {
  constexpr std::uint64_t kSeed = 1717;
  try {
    evaluates_touchdown_lateral_risk_and_boundary_classes();
    anisotropic_covariance_projection_rotates_with_reference_normal();
    missing_risk_evidence_fails_closed();
    marginal_length_uses_interval_intersection_and_a_hard_limit();
    certified_interval_error_prevents_hidden_interior_violations();
    invalid_covariance_and_alignment_fail_closed_deterministically();
    std::cout << "cable corridor checks passed: 6"
              << " seed=" << kSeed
              << " input_version=t17-cable-corridor/v1"
              << " units=SI reference=deterministic"
              << " risk=pointwise-only-no-epsilon-path\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "cable corridor failure: " << error.what()
              << " seed=" << kSeed
              << " input_version=t17-cable-corridor/v1"
              << " units=SI reference=deterministic"
              << " risk=pointwise-only-no-epsilon-path\n";
    return 1;
  }
}
