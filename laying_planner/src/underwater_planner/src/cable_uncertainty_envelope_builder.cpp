#include "underwater_planner/core/cable_uncertainty_envelope_builder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace underwater_planner::core {
namespace {

constexpr double kTolerance = 1.0e-12;

[[nodiscard]] bool finite(const double value) noexcept {
  return std::isfinite(value);
}

[[nodiscard]] bool valid_nonnegative(const double value) noexcept {
  return finite(value) && value >= 0.0;
}

[[nodiscard]] bool valid_positive(const double value) noexcept {
  return finite(value) && value > 0.0;
}

[[nodiscard]] bool valid_range(const ClosedRange& range) noexcept {
  return finite(range.minimum) && finite(range.maximum) &&
         range.minimum <= range.maximum;
}

[[nodiscard]] bool valid_range(const RangeMps& range) noexcept {
  return finite(range.minimum_mps) && finite(range.maximum_mps) &&
         range.minimum_mps <= range.maximum_mps;
}

[[nodiscard]] bool valid_range(const RangeMps2& range) noexcept {
  return finite(range.minimum_mps2) && finite(range.maximum_mps2) &&
         range.minimum_mps2 <= range.maximum_mps2;
}

[[nodiscard]] bool valid_range(const RangeN& range) noexcept {
  return finite(range.minimum_n) && finite(range.maximum_n) &&
         range.minimum_n <= range.maximum_n;
}

[[nodiscard]] bool margin_certification_valid(
    const EnvelopeMarginCertification& margins) noexcept {
  return margins.version != 0U &&
         !margins.calibration_dataset_id.empty() &&
         valid_nonnegative(margins.numerical_integration_stddev_m) &&
         valid_nonnegative(margins.reference_normal_sweep_stddev_m) &&
         valid_nonnegative(margins.statistical_quantile_stddev_m);
}

[[nodiscard]] bool execution_uncertainty_valid(
    const ExecutionUncertaintyCertification& certification) noexcept {
  return certification.version != 0U &&
         certification.execution_operating_envelope_version != 0U &&
         !certification.calibration_dataset_id.empty() &&
         valid_nonnegative(
             certification.payout_tracking_variance_upper_bound_m2) &&
         valid_nonnegative(
             certification.payout_acceleration_variance_upper_bound_m2) &&
         valid_nonnegative(certification.tension_variance_upper_bound_m2);
}

[[nodiscard]] bool subset(const RangeMps& inner,
                          const RangeMps& outer) noexcept {
  return valid_range(inner) && inner.minimum_mps >= outer.minimum_mps &&
         inner.maximum_mps <= outer.maximum_mps;
}

[[nodiscard]] bool subset(const RangeMps2& inner,
                          const RangeMps2& outer) noexcept {
  return valid_range(inner) && inner.minimum_mps2 >= outer.minimum_mps2 &&
         inner.maximum_mps2 <= outer.maximum_mps2;
}

[[nodiscard]] bool subset(const RangeN& inner,
                          const RangeN& outer) noexcept {
  return valid_range(inner) && inner.minimum_n >= outer.minimum_n &&
         inner.maximum_n <= outer.maximum_n;
}

[[nodiscard]] bool uncertainty_valid(
    const InitialUncertaintyBounds& uncertainty) noexcept {
  return valid_nonnegative(uncertainty.robot_x_variance_m2) &&
         valid_nonnegative(uncertainty.robot_y_variance_m2) &&
         valid_nonnegative(uncertainty.robot_heading_variance_rad2) &&
         valid_nonnegative(uncertainty.initial_lag_variance_rad2) &&
         valid_nonnegative(
             uncertainty.robot_position_process_variance_per_m_m2) &&
         valid_nonnegative(
             uncertainty.robot_heading_process_variance_per_m_rad2);
}

[[nodiscard]] bool sensor_bounds_valid(
    const SensorModeUncertaintyBounds& bounds) noexcept {
  return bounds.version != 0U && !bounds.calibration_dataset_id.empty() &&
         valid_nonnegative(bounds.additional_position_variance_per_axis_m2) &&
         valid_nonnegative(bounds.additional_heading_variance_rad2) &&
         valid_nonnegative(
             bounds.additional_heading_process_variance_per_m_rad2);
}

[[nodiscard]] bool sensor_certificates_valid(
    const std::vector<SensorModeUncertaintyBounds>& certificates) {
  std::set<SensorHealthMode> modes;
  return !certificates.empty() &&
         std::all_of(certificates.begin(), certificates.end(),
                     [&modes](const SensorModeUncertaintyBounds& bounds) {
                       return sensor_bounds_valid(bounds) &&
                              modes.insert(bounds.mode).second;
                     });
}

[[nodiscard]] bool primitive_valid(
    const CertifiedMotionPrimitive& primitive) noexcept {
  return primitive.version != 0U && valid_positive(primitive.arc_length_m) &&
         finite(primitive.curvature_per_m) &&
         valid_nonnegative(primitive.minimum_duration_s) &&
         valid_nonnegative(primitive.maximum_duration_s) &&
         primitive.minimum_duration_s <= primitive.maximum_duration_s &&
         valid_nonnegative(primitive.minimum_reference_progress_advance_m) &&
         valid_nonnegative(primitive.maximum_reference_progress_advance_m) &&
         primitive.minimum_reference_progress_advance_m <=
             primitive.maximum_reference_progress_advance_m;
}

[[nodiscard]] bool execution_envelope_valid(
    const ExecutionOperatingEnvelope& envelope,
    const CableModelParameters& model) noexcept {
  const SpeedPayoutLimits& limits = envelope.limits;
  return envelope.version != 0U &&
         envelope.operating_domain_id == model.operating_domain_id &&
         valid_range(limits.ground_speed) &&
         limits.ground_speed.minimum_mps >= 0.0 &&
         valid_range(limits.ground_acceleration) &&
         valid_nonnegative(limits.maximum_lateral_acceleration_mps2) &&
         valid_range(limits.payout_speed) &&
         limits.payout_speed.minimum_mps >= model.payout_speed_range.minimum_mps &&
         limits.payout_speed.maximum_mps <= model.payout_speed_range.maximum_mps &&
         valid_range(limits.payout_acceleration) &&
         std::abs(limits.payout_acceleration.minimum_mps2) <=
             model.maximum_payout_acceleration_mps2 &&
         std::abs(limits.payout_acceleration.maximum_mps2) <=
             model.maximum_payout_acceleration_mps2 &&
         valid_nonnegative(limits.maximum_payout_tracking_error_mps) &&
         limits.maximum_payout_tracking_error_mps <=
             model.maximum_payout_tracking_error_mps &&
         valid_range(limits.tension) &&
         limits.tension.minimum_n -
                 envelope.maximum_tension_tracking_error_n >=
             model.tension_range.minimum_n &&
         limits.tension.maximum_n +
                 envelope.maximum_tension_tracking_error_n <=
             model.tension_range.maximum_n &&
         valid_nonnegative(limits.maximum_stopping_distance_m) &&
         valid_nonnegative(
             envelope.maximum_payout_acceleration_tracking_error_mps2) &&
         envelope.maximum_payout_acceleration_tracking_error_mps2 <=
             model.maximum_payout_acceleration_mps2 &&
         valid_nonnegative(envelope.maximum_tension_tracking_error_n) &&
         envelope.maximum_tension_tracking_error_n <=
             model.maximum_tension_tracking_error_n;
}

[[nodiscard]] bool primitive_within_execution_envelope(
    const CertifiedMotionPrimitive& primitive,
    const ExecutionOperatingEnvelope& envelope) noexcept {
  if (!valid_positive(primitive.minimum_duration_s) ||
      !valid_positive(primitive.maximum_duration_s)) {
    return false;
  }
  const double minimum_average_speed_mps =
      primitive.arc_length_m / primitive.maximum_duration_s;
  const double maximum_average_speed_mps =
      primitive.arc_length_m / primitive.minimum_duration_s;
  const SpeedPayoutLimits& certified = primitive.certified_execution_limits;
  return minimum_average_speed_mps + kTolerance >=
             certified.ground_speed.minimum_mps &&
         maximum_average_speed_mps <=
             certified.ground_speed.maximum_mps + kTolerance &&
         subset(certified.ground_speed, envelope.limits.ground_speed) &&
         subset(certified.ground_acceleration,
                envelope.limits.ground_acceleration) &&
         valid_nonnegative(certified.maximum_lateral_acceleration_mps2) &&
         certified.maximum_lateral_acceleration_mps2 <=
             envelope.limits.maximum_lateral_acceleration_mps2 &&
         certified.ground_speed.maximum_mps *
                 certified.ground_speed.maximum_mps *
                 std::abs(primitive.curvature_per_m) <=
             certified.maximum_lateral_acceleration_mps2 + kTolerance &&
         subset(certified.payout_speed, envelope.limits.payout_speed) &&
         subset(certified.payout_acceleration,
                envelope.limits.payout_acceleration) &&
         valid_nonnegative(certified.maximum_payout_tracking_error_mps) &&
         certified.maximum_payout_tracking_error_mps <=
             envelope.limits.maximum_payout_tracking_error_mps &&
         subset(certified.tension, envelope.limits.tension) &&
         valid_nonnegative(certified.maximum_stopping_distance_m) &&
         certified.maximum_stopping_distance_m <=
             envelope.limits.maximum_stopping_distance_m;
}

[[nodiscard]] bool reference_frames_valid(const ReferenceLine& reference) {
  return std::all_of(reference.points.begin(), reference.points.end(),
                     [](const ReferencePoint& point) {
                       return std::abs(std::hypot(point.tangent_x,
                                                  point.tangent_y) -
                                       1.0) <= kTolerance &&
                              std::abs(std::hypot(point.normal_x,
                                                  point.normal_y) -
                                       1.0) <= kTolerance &&
                              std::abs(point.normal_x + point.tangent_y) <=
                                  kTolerance &&
                              std::abs(point.normal_y - point.tangent_x) <=
                                  kTolerance;
                     });
}

[[nodiscard]] bool model_parameters_valid(
    const CableModelParameters& parameters) noexcept {
  try {
    const CableModel checked(parameters);
    return checked.version() == parameters.version;
  } catch (const std::invalid_argument&) {
    return false;
  }
}

[[nodiscard]] const SensorModeUncertaintyBounds* find_sensor_bounds(
    const OperatingDomain& domain, const SensorHealthMode mode) noexcept {
  const auto found = std::find_if(
      domain.approved_sensor_uncertainty.begin(),
      domain.approved_sensor_uncertainty.end(),
      [mode](const SensorModeUncertaintyBounds& bounds) {
        return bounds.mode == mode;
      });
  return found == domain.approved_sensor_uncertainty.end() ? nullptr
                                                           : &*found;
}

[[nodiscard]] bool model_approves_sensor_mode(
    const CableModelParameters& model, const SensorHealthMode mode) {
  return std::find(model.approved_sensor_modes.begin(),
                   model.approved_sensor_modes.end(), mode) !=
         model.approved_sensor_modes.end();
}

[[nodiscard]] double variance_upper_bound(
    const double path_length_m, const OperatingDomain& domain,
    const SensorModeUncertaintyBounds& sensor,
    const CableModelParameters& model) noexcept {
  const InitialUncertaintyBounds& initial = domain.initial_uncertainty;
  const double position_process_variance_m2 =
      path_length_m * initial.robot_position_process_variance_per_m_m2;
  const double sigma_x_m = std::sqrt(
      initial.robot_x_variance_m2 +
      sensor.additional_position_variance_per_axis_m2 +
      position_process_variance_m2);
  const double sigma_y_m = std::sqrt(
      initial.robot_y_variance_m2 +
      sensor.additional_position_variance_per_axis_m2 +
      position_process_variance_m2);
  const double heading_variance_rad2 =
      initial.robot_heading_variance_rad2 +
      sensor.additional_heading_variance_rad2 +
      path_length_m *
          (initial.robot_heading_process_variance_per_m_rad2 +
           sensor.additional_heading_process_variance_per_m_rad2);
  const double release_lever_m =
      std::hypot(model.release_point_offset_m.x_m,
                 model.release_point_offset_m.y_m) +
      model.touchdown_distance_m;
  const double maximum_response_sensitivity_rad_per_m =
      path_length_m * model.maximum_lag_angle_rad /
      (model.direction_response_length_m *
       model.direction_response_length_m);
  const double process_noise_trace_m2 =
      path_length_m *
      (model.touchdown_process_noise_per_m_m2.xx_m2 +
       model.touchdown_process_noise_per_m_m2.yy_m2);
  // Summing Jacobian-weighted standard deviations is the Cauchy-Schwarz
  // upper bound for every PSD cross-covariance allowed by the domain.
  const double sigma_sum_m =
      sigma_x_m + sigma_y_m +
      release_lever_m * std::sqrt(heading_variance_rad2) +
      model.touchdown_distance_m *
          std::sqrt(initial.initial_lag_variance_rad2 +
                    path_length_m *
                        model.lag_angle_process_variance_per_m_rad2) +
      std::sqrt(model.touchdown_distance_variance_m2) +
      model.touchdown_distance_m * maximum_response_sensitivity_rad_per_m *
          std::sqrt(model.direction_response_length_variance_m2) +
      std::sqrt(std::max(0.0, process_noise_trace_m2)) +
      std::sqrt(domain.execution_uncertainty
                    .payout_tracking_variance_upper_bound_m2) +
      std::sqrt(domain.execution_uncertainty
                    .payout_acceleration_variance_upper_bound_m2) +
      std::sqrt(
          domain.execution_uncertainty.tension_variance_upper_bound_m2);
  return sigma_sum_m * sigma_sum_m;
}

using BinKey = std::tuple<std::int64_t, std::int64_t, std::int64_t,
                          std::int64_t, std::int64_t>;

[[nodiscard]] std::int64_t bin_index(const double value,
                                     const double width) noexcept {
  return static_cast<std::int64_t>(std::floor(value / width));
}

[[nodiscard]] BinKey bin_key(const ReachableSetCertificate& set,
                             const ReachabilityBinning& binning) noexcept {
  const double heading_center =
      0.5 * (set.heading_rad.minimum + set.heading_rad.maximum);
  const double lag_center =
      0.5 * (set.lag_angle_rad.minimum + set.lag_angle_rad.maximum);
  const double progress_center = 0.5 *
      (set.reference_progress_m.minimum + set.reference_progress_m.maximum);
  const double time_center =
      0.5 * (set.elapsed_time_s.minimum + set.elapsed_time_s.maximum);
  return {bin_index(set.path_length_m, binning.arc_length_bin_m),
          bin_index(heading_center, binning.heading_bin_rad),
          bin_index(lag_center, binning.lag_angle_bin_rad),
          bin_index(progress_center, binning.reference_progress_bin_m),
          bin_index(time_center, binning.time_bin_s)};
}

[[nodiscard]] bool contains(const ClosedRange& outer,
                            const ClosedRange& inner) noexcept {
  return outer.minimum <= inner.minimum + kTolerance &&
         outer.maximum + kTolerance >= inner.maximum;
}

[[nodiscard]] bool set_contains(const ReachableSetCertificate& outer,
                                const ReachableSetCertificate& inner) noexcept {
  return std::abs(outer.path_length_m - inner.path_length_m) <= kTolerance &&
         contains(outer.elapsed_time_s, inner.elapsed_time_s) &&
         contains(outer.heading_rad, inner.heading_rad) &&
         contains(outer.lag_angle_rad, inner.lag_angle_rad) &&
         contains(outer.reference_progress_m, inner.reference_progress_m) &&
         contains(outer.swept_reference_progress_m,
                  inner.swept_reference_progress_m) &&
         outer.lateral_variance_upper_bound_m2 + kTolerance >=
             inner.lateral_variance_upper_bound_m2;
}

[[nodiscard]] ReachableSetCertificate extend(
    const ReachableSetCertificate& parent,
    const CertifiedMotionPrimitive& primitive, const std::uint64_t id,
    const OperatingDomain& domain,
    const SensorModeUncertaintyBounds& sensor,
    const CableModelParameters& model) noexcept {
  ReachableSetCertificate result;
  result.id = id;
  result.parent_id = parent.id;
  result.primitive_version = primitive.version;
  result.path_length_m = parent.path_length_m + primitive.arc_length_m;
  result.elapsed_time_s = {
      parent.elapsed_time_s.minimum + primitive.minimum_duration_s,
      std::min(domain.maximum_planning_time_s,
               parent.elapsed_time_s.maximum + primitive.maximum_duration_s)};
  const double heading_delta =
      primitive.arc_length_m * primitive.curvature_per_m;
  result.heading_rad = {parent.heading_rad.minimum + heading_delta,
                        parent.heading_rad.maximum + heading_delta};
  const double decay =
      std::exp(-primitive.arc_length_m /
               model.direction_response_length_m);
  const double forced_lag =
      -primitive.curvature_per_m * model.direction_response_length_m *
      (1.0 - decay);
  result.lag_angle_rad = {
      decay * parent.lag_angle_rad.minimum + forced_lag,
      decay * parent.lag_angle_rad.maximum + forced_lag};
  result.reference_progress_m = {
      parent.reference_progress_m.minimum +
          primitive.minimum_reference_progress_advance_m,
      std::min(domain.reference_progress_end_m,
               parent.reference_progress_m.maximum +
                   primitive.maximum_reference_progress_advance_m)};
  result.swept_reference_progress_m = {
      parent.reference_progress_m.minimum,
      result.reference_progress_m.maximum};
  result.lateral_variance_upper_bound_m2 = variance_upper_bound(
      result.path_length_m, domain, sensor, model);
  return result;
}

void fail(CableUncertaintyEnvelope& envelope,
          const EnvelopeBuildValidity validity, std::string code,
          std::string message) {
  envelope.validity = validity;
  envelope.segments.clear();
  envelope.reachable_set_certificates.clear();
  envelope.generation_stats.retained_reachable_set_count = 0U;
  envelope.diagnostics.push_back(
      {DiagnosticSeverity::error, std::move(code),
       "cable_uncertainty_envelope_builder", std::move(message),
       envelope.generation_timestamp});
}

[[nodiscard]] EnvelopeMarginBudget derive_margin_budget(
    const OperatingDomain& domain,
    const CableModelParameters& model) noexcept {
  const double release_lever_m =
      std::hypot(model.release_point_offset_m.x_m,
                 model.release_point_offset_m.y_m) +
      model.touchdown_distance_m;
  EnvelopeMarginBudget result;
  result.certification_version = domain.margin_certification.version;
  result.calibration_dataset_id =
      domain.margin_certification.calibration_dataset_id;
  // Reachability keeps exact intervals; only heading/lag bin membership can
  // move the touchdown mean, bounded by their maximum geometric levers.
  result.state_binning_stddev_m =
      0.5 * release_lever_m * domain.binning.heading_bin_rad +
      0.5 * model.touchdown_distance_m *
          domain.binning.lag_angle_bin_rad;
  result.numerical_integration_stddev_m =
      domain.margin_certification.numerical_integration_stddev_m;
  result.reference_normal_sweep_stddev_m =
      domain.margin_certification.reference_normal_sweep_stddev_m;
  result.statistical_quantile_stddev_m =
      domain.margin_certification.statistical_quantile_stddev_m;
  return result;
}

}  // namespace

double envelope_discretization_margin_stddev_m(
    const EnvelopeMarginBudget& margin) noexcept {
  return margin.state_binning_stddev_m +
         margin.numerical_integration_stddev_m +
         margin.reference_normal_sweep_stddev_m;
}

double total_envelope_margin_stddev_m(
    const EnvelopeMarginBudget& margin) noexcept {
  return envelope_discretization_margin_stddev_m(margin) +
         margin.statistical_quantile_stddev_m;
}

bool operator==(const CableUncertaintyEnvelopeSegment& left,
                const CableUncertaintyEnvelopeSegment& right) noexcept {
  return left.reference_progress_start_m == right.reference_progress_start_m &&
         left.reference_progress_end_m == right.reference_progress_end_m &&
         left.lateral_variance_upper_bound_m2 ==
             right.lateral_variance_upper_bound_m2 &&
         left.lateral_stddev_upper_bound_m ==
             right.lateral_stddev_upper_bound_m;
}

bool operator==(const ReachableSetCertificate& left,
                const ReachableSetCertificate& right) noexcept {
  return left.id == right.id && left.parent_id == right.parent_id &&
         left.primitive_version == right.primitive_version &&
         left.path_length_m == right.path_length_m &&
         left.elapsed_time_s.minimum == right.elapsed_time_s.minimum &&
         left.elapsed_time_s.maximum == right.elapsed_time_s.maximum &&
         left.heading_rad.minimum == right.heading_rad.minimum &&
         left.heading_rad.maximum == right.heading_rad.maximum &&
         left.lag_angle_rad.minimum == right.lag_angle_rad.minimum &&
         left.lag_angle_rad.maximum == right.lag_angle_rad.maximum &&
         left.reference_progress_m.minimum ==
             right.reference_progress_m.minimum &&
         left.reference_progress_m.maximum ==
             right.reference_progress_m.maximum &&
         left.swept_reference_progress_m.minimum ==
             right.swept_reference_progress_m.minimum &&
         left.swept_reference_progress_m.maximum ==
             right.swept_reference_progress_m.maximum &&
         left.lateral_variance_upper_bound_m2 ==
             right.lateral_variance_upper_bound_m2;
}

CableUncertaintyEnvelope
CableUncertaintyEnvelopeBuilder::buildCertifiedUpperBound(
    const OperatingDomain& domain, const ReferenceLine& reference,
    const SensorHealthMode sensor_mode, const CableModelParameters& model,
    const ExecutionOperatingEnvelope& execution_envelope,
    const std::uint64_t generator_version,
    const MonotonicTime generation_timestamp) const {
  CableUncertaintyEnvelope envelope;
  envelope.generation_timestamp = generation_timestamp;
  envelope.dependencies.generator_version = generator_version;
  envelope.dependencies.cable_model_version = model.version;
  envelope.dependencies.execution_operating_envelope_version =
      execution_envelope.version;
  envelope.dependencies.reference_line_version = reference.version;
  envelope.dependencies.operating_domain_version = domain.version;
  envelope.dependencies.primitive_set_version = domain.primitive_set_version;
  envelope.dependencies.initial_uncertainty_version =
      domain.initial_uncertainty_version;
  envelope.dependencies.execution_uncertainty_version =
      domain.execution_uncertainty.version;
  envelope.dependencies.margin_certification_version =
      domain.margin_certification.version;
  envelope.dependencies.sensor_mode = sensor_mode;
  envelope.dependencies.operating_domain_id = domain.operating_domain_id;
  envelope.dependencies.cable_model_calibration_dataset_id =
      model.calibration_dataset_id;
  envelope.dependencies.certification_dataset_id =
      domain.certification_dataset_id;
  envelope.dependencies.execution_uncertainty_calibration_dataset_id =
      domain.execution_uncertainty.calibration_dataset_id;
  envelope.dependencies.margin_calibration_dataset_id =
      domain.margin_certification.calibration_dataset_id;
  envelope.margin_budget = derive_margin_budget(domain, model);
  envelope.path_joint_risk_implemented = false;
  envelope.risk_semantics = kPointwiseEnvelopeRiskSemantics;

  const SensorModeUncertaintyBounds* sensor =
      find_sensor_bounds(domain, sensor_mode);
  if (sensor != nullptr) {
    envelope.dependencies.sensor_uncertainty_version = sensor->version;
    envelope.dependencies.sensor_calibration_dataset_id =
        sensor->calibration_dataset_id;
  }

  if (domain.operating_domain_id != model.operating_domain_id ||
      domain.operating_domain_id != execution_envelope.operating_domain_id) {
    fail(envelope, EnvelopeBuildValidity::dependency_mismatch,
         "ENVELOPE_DEPENDENCY_MISMATCH",
         "operating domain identifiers do not match");
    return envelope;
  }
  if (sensor == nullptr || !model_approves_sensor_mode(model, sensor_mode)) {
    fail(envelope, EnvelopeBuildValidity::sensor_mode_unapproved,
         "ENVELOPE_SENSOR_MODE_UNAPPROVED",
         "sensor mode lacks a model and domain uncertainty certificate");
    return envelope;
  }

  const SnapshotValidation reference_validation = validate(reference);
  const bool base_valid =
      generator_version != 0U && generation_timestamp.nanoseconds >= 0 &&
      domain.version != 0U &&
      domain.primitive_set_version != 0U &&
      domain.initial_uncertainty_version != 0U &&
      !domain.operating_domain_id.empty() &&
      !domain.certification_dataset_id.empty() &&
      model_parameters_valid(model) &&
      execution_envelope_valid(execution_envelope, model) &&
      reference_validation.valid && reference_frames_valid(reference) &&
      sensor_certificates_valid(domain.approved_sensor_uncertainty) &&
      valid_nonnegative(domain.reference_progress_start_m) &&
      valid_positive(domain.reference_progress_end_m) &&
      domain.reference_progress_start_m < domain.reference_progress_end_m &&
      domain.reference_progress_start_m >=
          reference.points.front().arc_length_m &&
      domain.reference_progress_end_m <=
          reference.points.back().arc_length_m &&
      valid_positive(domain.maximum_planning_length_m) &&
      valid_positive(domain.maximum_planning_time_s) &&
      valid_range(domain.initial_lag_angle_rad) &&
      std::max(std::abs(domain.initial_lag_angle_rad.minimum),
               std::abs(domain.initial_lag_angle_rad.maximum)) <=
          model.maximum_lag_angle_rad &&
      uncertainty_valid(domain.initial_uncertainty) &&
      execution_uncertainty_valid(domain.execution_uncertainty) &&
      domain.execution_uncertainty.execution_operating_envelope_version ==
          execution_envelope.version &&
      !domain.allowed_primitives.empty() &&
      std::all_of(domain.allowed_primitives.begin(),
                  domain.allowed_primitives.end(), primitive_valid) &&
      valid_positive(domain.binning.arc_length_bin_m) &&
      valid_positive(domain.binning.heading_bin_rad) &&
      valid_positive(domain.binning.lag_angle_bin_rad) &&
      valid_positive(domain.binning.reference_progress_bin_m) &&
      valid_positive(domain.binning.time_bin_s) &&
      margin_certification_valid(domain.margin_certification) &&
      domain.maximum_reachable_sets > 0U;
  if (!base_valid) {
    fail(envelope, EnvelopeBuildValidity::input_invalid,
         "ENVELOPE_INPUT_INVALID",
         "operating domain, reference, model, execution envelope, or margins are invalid");
    return envelope;
  }

  std::set<std::uint64_t> primitive_versions;
  for (const CertifiedMotionPrimitive& primitive : domain.allowed_primitives) {
    if (!primitive_versions.insert(primitive.version).second ||
        !primitive_within_execution_envelope(primitive,
                                             execution_envelope)) {
      fail(envelope, EnvelopeBuildValidity::input_invalid,
           "ENVELOPE_PRIMITIVE_OUTSIDE_EXECUTION_DOMAIN",
           "motion primitives must have unique versions and remain inside the certified execution envelope");
      return envelope;
    }
  }

  std::vector<ReachableSetCertificate> retained;
  retained.reserve(domain.maximum_reachable_sets);
  ReachableSetCertificate initial;
  initial.id = 1U;
  initial.path_length_m = 0.0;
  initial.elapsed_time_s = {0.0, 0.0};
  initial.heading_rad = {0.0, 0.0};
  initial.lag_angle_rad = domain.initial_lag_angle_rad;
  initial.reference_progress_m = {
      domain.reference_progress_start_m, domain.reference_progress_start_m};
  initial.swept_reference_progress_m = initial.reference_progress_m;
  initial.lateral_variance_upper_bound_m2 =
      variance_upper_bound(0.0, domain, *sensor, model);
  retained.push_back(initial);

  std::map<BinKey, std::vector<std::size_t>> bins;
  bins[bin_key(initial, domain.binning)].push_back(0U);
  envelope.generation_stats.maximum_incomparable_sets_in_bin = 1U;
  std::uint64_t next_id = 2U;
  for (std::size_t parent_index = 0U; parent_index < retained.size();
       ++parent_index) {
    const ReachableSetCertificate parent = retained[parent_index];
    for (const CertifiedMotionPrimitive& primitive : domain.allowed_primitives) {
      if (parent.path_length_m + primitive.arc_length_m >
              domain.maximum_planning_length_m + kTolerance ||
          parent.elapsed_time_s.minimum + primitive.minimum_duration_s >
              domain.maximum_planning_time_s + kTolerance ||
          parent.reference_progress_m.minimum >=
              domain.reference_progress_end_m - kTolerance) {
        continue;
      }
      ReachableSetCertificate candidate =
          extend(parent, primitive, next_id, domain, *sensor, model);
      if (candidate.lag_angle_rad.maximum <
              -model.maximum_lag_angle_rad - kTolerance ||
          candidate.lag_angle_rad.minimum >
              model.maximum_lag_angle_rad + kTolerance) {
        continue;
      }
      candidate.lag_angle_rad.minimum =
          std::max(candidate.lag_angle_rad.minimum,
                   -model.maximum_lag_angle_rad);
      candidate.lag_angle_rad.maximum =
          std::min(candidate.lag_angle_rad.maximum,
                   model.maximum_lag_angle_rad);
      candidate.reference_progress_m.minimum =
          std::min(candidate.reference_progress_m.minimum,
                   domain.reference_progress_end_m);
      const BinKey key = bin_key(candidate, domain.binning);
      bool pruned = false;
      const auto existing_bin = bins.find(key);
      if (existing_bin != bins.end()) {
        for (const std::size_t existing_index : existing_bin->second) {
          if (set_contains(retained[existing_index], candidate)) {
            pruned = true;
            ++envelope.generation_stats.containment_pruned_count;
            break;
          }
        }
      }
      if (pruned) continue;
      if (retained.size() >= domain.maximum_reachable_sets) {
        fail(envelope, EnvelopeBuildValidity::resource_limit_exceeded,
             "ENVELOPE_REACHABILITY_BUDGET_EXHAUSTED",
             "reachable-set budget exhausted before certified closure");
        return envelope;
      }
      const std::size_t inserted_index = retained.size();
      retained.push_back(candidate);
      bins[key].push_back(inserted_index);
      envelope.generation_stats.maximum_incomparable_sets_in_bin = std::max(
          envelope.generation_stats.maximum_incomparable_sets_in_bin,
          bins[key].size());
      ++next_id;
    }
  }

  envelope.reachable_set_certificates.reserve(retained.size());
  envelope.reachable_set_certificates = retained;
  envelope.generation_stats.retained_reachable_set_count = retained.size();
  envelope.generation_stats.risk_bound_pruned_count = 0U;

  const double progress_span_m = domain.reference_progress_end_m -
                                 domain.reference_progress_start_m;
  const double segment_count_value =
      std::ceil(progress_span_m /
                domain.binning.reference_progress_bin_m);
  if (!finite(segment_count_value) ||
      segment_count_value >
          static_cast<double>(domain.maximum_reachable_sets)) {
    fail(envelope, EnvelopeBuildValidity::resource_limit_exceeded,
         "ENVELOPE_PROGRESS_BIN_BUDGET_EXHAUSTED",
         "reference-progress bin budget exhausted before envelope generation");
    return envelope;
  }
  const std::size_t segment_count =
      static_cast<std::size_t>(segment_count_value);
  for (std::size_t index = 0U; index < segment_count; ++index) {
    const double segment_start =
        domain.reference_progress_start_m +
        static_cast<double>(index) *
            domain.binning.reference_progress_bin_m;
    const double segment_end =
        std::min(domain.reference_progress_end_m,
                 domain.reference_progress_start_m +
                     static_cast<double>(index + 1U) *
                         domain.binning.reference_progress_bin_m);
    double variance_bound_m2 = 0.0;
    for (const ReachableSetCertificate& set : retained) {
      if (set.swept_reference_progress_m.maximum + kTolerance >=
              segment_start &&
          set.swept_reference_progress_m.minimum <=
              segment_end + kTolerance) {
        variance_bound_m2 =
            std::max(variance_bound_m2,
                     set.lateral_variance_upper_bound_m2);
      }
    }
    if (variance_bound_m2 == 0.0) {
      variance_bound_m2 = variance_upper_bound(
          domain.maximum_planning_length_m, domain, *sensor, model);
    }
    envelope.segments.push_back(
        {segment_start, segment_end, variance_bound_m2,
         std::sqrt(variance_bound_m2) +
             total_envelope_margin_stddev_m(envelope.margin_budget)});
  }
  if (envelope.segments.empty()) {
    fail(envelope, EnvelopeBuildValidity::input_invalid,
         "ENVELOPE_PROGRESS_RANGE_EMPTY",
         "reference progress range produced no envelope segments");
    return envelope;
  }
  envelope.validity = EnvelopeBuildValidity::valid;
  envelope.diagnostics.push_back(
      {DiagnosticSeverity::info, "ENVELOPE_CERTIFIED",
       "cable_uncertainty_envelope_builder",
       "certified pointwise lateral uncertainty envelope generated",
       generation_timestamp});
  return envelope;
}

}  // namespace underwater_planner::core
