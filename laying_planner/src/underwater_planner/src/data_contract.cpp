#include "underwater_planner/core/data_contract.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace underwater_planner::core {

bool operator==(const MapVersion& left, const MapVersion& right) noexcept {
  return left.map_id == right.map_id &&
         left.sequence_number == right.sequence_number &&
         left.timestamp.nanoseconds == right.timestamp.nanoseconds &&
         left.coordinate_frame == right.coordinate_frame;
}

bool operator!=(const MapVersion& left, const MapVersion& right) noexcept {
  return !(left == right);
}

bool operator==(const PlanningDependencyVersions& left,
                const PlanningDependencyVersions& right) noexcept {
  return left.map_version == right.map_version &&
         left.reference_line_version == right.reference_line_version &&
         left.robot_operating_area_version ==
             right.robot_operating_area_version &&
         left.terrain_gradient_policy_version ==
             right.terrain_gradient_policy_version &&
         left.corridor_risk_policy_version ==
             right.corridor_risk_policy_version &&
         left.cable_model_version == right.cable_model_version &&
         left.uncertainty_envelope_version ==
             right.uncertainty_envelope_version &&
         left.uncertainty_envelope_generator_version ==
             right.uncertainty_envelope_generator_version &&
         left.execution_operating_envelope_version ==
             right.execution_operating_envelope_version &&
         left.execution_profile_version == right.execution_profile_version &&
         left.sensor_mode == right.sensor_mode &&
         left.operating_domain_id == right.operating_domain_id &&
         left.cable_corridor_version == right.cable_corridor_version;
}

bool operator!=(const PlanningDependencyVersions& left,
                const PlanningDependencyVersions& right) noexcept {
  return !(left == right);
}

namespace {

[[nodiscard]] std::optional<std::string_view> cable_model_validity_name(
    const CableModelValidity validity) noexcept {
  switch (validity) {
    case CableModelValidity::valid:
      return "VALID";
    case CableModelValidity::input_invalid:
      return "INPUT_INVALID";
    case CableModelValidity::initial_state_uncertain:
      return "INITIAL_STATE_UNCERTAIN";
    case CableModelValidity::payout_tracking_out_of_range:
      return "PAYOUT_TRACKING_OUT_OF_RANGE";
    case CableModelValidity::tension_out_of_range:
      return "TENSION_OUT_OF_RANGE";
    case CableModelValidity::lag_angle_out_of_range:
      return "LAG_ANGLE_OUT_OF_RANGE";
    case CableModelValidity::motion_mode_out_of_range:
      return "MOTION_MODE_OUT_OF_RANGE";
    case CableModelValidity::sensor_mode_unapproved:
      return "SENSOR_MODE_UNAPPROVED";
    case CableModelValidity::operating_domain_mismatch:
      return "OPERATING_DOMAIN_MISMATCH";
    case CableModelValidity::execution_envelope_version_mismatch:
      return "EXECUTION_ENVELOPE_VERSION_MISMATCH";
    case CableModelValidity::covariance_invalid:
      return "COVARIANCE_INVALID";
  }
  return std::nullopt;
}

void require_finite(const double value, const char* field,
                    ValidationResult& result) {
  if (!std::isfinite(value)) {
    result.issues.emplace_back(field);
  }
}

void require_nonnegative_finite(const double value, const char* field,
                                ValidationResult& result) {
  if (!std::isfinite(value) || value < 0.0) {
    result.issues.emplace_back(field);
  }
}

bool is_normalized_angle(const double angle_rad) {
  constexpr double kPi = 3.14159265358979323846;
  return std::isfinite(angle_rad) && angle_rad >= -kPi && angle_rad < kPi;
}

void append_issues(const ValidationResult& source, ValidationResult& target) {
  target.issues.insert(target.issues.end(), source.issues.begin(),
                       source.issues.end());
}

void validate_range(const double minimum, const double maximum,
                    const char* field, ValidationResult& result) {
  if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum > maximum) {
    result.issues.emplace_back(field);
  }
}

void validate_probability(const std::optional<double>& probability,
                          const char* field, ValidationResult& result) {
  if (!probability.has_value() || !std::isfinite(*probability) ||
      *probability <= 0.0 || *probability >= 1.0) {
    result.issues.emplace_back(field);
  }
}

void validate_covariance(const Covariance2dM2& covariance,
                         ValidationResult& result) {
  require_finite(covariance.xx_m2, "covariance xx_m2 must be finite", result);
  require_finite(covariance.xy_m2, "covariance xy_m2 must be finite", result);
  require_finite(covariance.yx_m2, "covariance yx_m2 must be finite", result);
  require_finite(covariance.yy_m2, "covariance yy_m2 must be finite", result);
  if (covariance.xy_m2 != covariance.yx_m2) {
    result.issues.emplace_back("covariance must be symmetric");
  }
  const double scale = std::max(
      {std::abs(covariance.xx_m2), std::abs(covariance.xy_m2),
       std::abs(covariance.yx_m2), std::abs(covariance.yy_m2)});
  bool positive_semidefinite = covariance.xx_m2 >= 0.0 &&
                               covariance.yy_m2 >= 0.0;
  if (positive_semidefinite && scale > 0.0 && std::isfinite(scale)) {
    const double normalized_xx = covariance.xx_m2 / scale;
    const double normalized_xy = covariance.xy_m2 / scale;
    const double normalized_yy = covariance.yy_m2 / scale;
    positive_semidefinite =
        normalized_xx * normalized_yy - normalized_xy * normalized_xy >= 0.0;
  }
  if (!positive_semidefinite) {
    result.issues.emplace_back("covariance must be positive semidefinite");
  }
}

void validate_memory(const CableConstraintMemory& memory,
                     ValidationResult& result) {
  require_nonnegative_finite(memory.retained_arc_length_m,
                             "retained cable arc length must be nonnegative",
                             result);
  if (memory.previous_distinct_touchdown_points_m.size() > 2) {
    result.issues.emplace_back(
        "cable memory may retain at most two distinct touchdown points");
  }
  for (const Vector2m& point : memory.previous_distinct_touchdown_points_m) {
    require_finite(point.x_m, "touchdown history x_m must be finite", result);
    require_finite(point.y_m, "touchdown history y_m must be finite", result);
  }
  double previous_arc_length_m = -1.0;
  for (const CableHistorySample& sample : memory.trailing_support_samples) {
    require_nonnegative_finite(sample.touchdown_arc_length_m,
                               "touchdown history arc length must be nonnegative",
                               result);
    if (sample.touchdown_arc_length_m <= previous_arc_length_m) {
      result.issues.emplace_back(
          "touchdown support history arc length must be strictly increasing");
    }
    previous_arc_length_m = sample.touchdown_arc_length_m;
    require_finite(sample.touchdown_position_m.x_m,
                   "touchdown support x_m must be finite", result);
    require_finite(sample.touchdown_position_m.y_m,
                   "touchdown support y_m must be finite", result);
  }
}

bool memories_equal(const CableConstraintMemory& left,
                    const CableConstraintMemory& right) {
  if (left.retained_arc_length_m != right.retained_arc_length_m ||
      left.canonical_signature != right.canonical_signature ||
      left.previous_distinct_touchdown_points_m.size() !=
          right.previous_distinct_touchdown_points_m.size() ||
      left.trailing_support_samples.size() !=
          right.trailing_support_samples.size()) {
    return false;
  }
  for (std::size_t index = 0;
       index < left.previous_distinct_touchdown_points_m.size(); ++index) {
    const Vector2m& left_point =
        left.previous_distinct_touchdown_points_m[index];
    const Vector2m& right_point =
        right.previous_distinct_touchdown_points_m[index];
    if (left_point.x_m != right_point.x_m || left_point.y_m != right_point.y_m) {
      return false;
    }
  }
  for (std::size_t index = 0; index < left.trailing_support_samples.size();
       ++index) {
    const CableHistorySample& left_sample = left.trailing_support_samples[index];
    const CableHistorySample& right_sample =
        right.trailing_support_samples[index];
    if (left_sample.touchdown_arc_length_m !=
            right_sample.touchdown_arc_length_m ||
        left_sample.touchdown_position_m.x_m !=
            right_sample.touchdown_position_m.x_m ||
        left_sample.touchdown_position_m.y_m !=
            right_sample.touchdown_position_m.y_m) {
      return false;
    }
  }
  return true;
}

bool is_known_enum(const CableStateKind value) {
  switch (value) {
    case CableStateKind::tracked:
    case CableStateKind::search_mean:
      return true;
  }
  return false;
}

bool is_known_enum(const SensorHealthMode value) {
  switch (value) {
    case SensorHealthMode::nominal:
    case SensorHealthMode::approved_degraded:
      return true;
  }
  return false;
}

bool is_known_enum(const CorridorEvaluationValidity value) {
  switch (value) {
    case CorridorEvaluationValidity::valid:
    case CorridorEvaluationValidity::risk_policy_missing:
    case CorridorEvaluationValidity::input_invalid:
    case CorridorEvaluationValidity::reference_version_mismatch:
    case CorridorEvaluationValidity::coordinate_transform_error_missing:
    case CorridorEvaluationValidity::covariance_invalid:
    case CorridorEvaluationValidity::distribution_not_calibrated:
    case CorridorEvaluationValidity::envelope_missing:
    case CorridorEvaluationValidity::envelope_version_mismatch:
    case CorridorEvaluationValidity::covariance_envelope_breach:
      return true;
  }
  return false;
}

bool is_known_enum(const CableCorridorPointBasis value) {
  switch (value) {
    case CableCorridorPointBasis::below_nominal_bound:
    case CableCorridorPointBasis::within_absolute_bound:
    case CableCorridorPointBasis::at_or_above_absolute_bound:
      return true;
  }
  return false;
}

bool is_known_enum(const CableValidationStatus value) {
  switch (value) {
    case CableValidationStatus::pass:
    case CableValidationStatus::marginal:
    case CableValidationStatus::violation:
      return true;
  }
  return false;
}

bool is_known_enum(const CableLayingFailure value) {
  switch (value) {
    case CableLayingFailure::none:
    case CableLayingFailure::curvature_exceeded:
    case CableLayingFailure::support_proxy_exceeded:
    case CableLayingFailure::forbidden_area_intersection:
    case CableLayingFailure::terrain_data_invalid:
    case CableLayingFailure::numerically_invalid:
    case CableLayingFailure::duplicate_touchdown_point:
    case CableLayingFailure::mechanical_history_incomplete:
      return true;
  }
  return false;
}

bool is_known_enum(const DiagnosticSeverity value) {
  switch (value) {
    case DiagnosticSeverity::info:
    case DiagnosticSeverity::warning:
    case DiagnosticSeverity::error:
      return true;
  }
  return false;
}

bool is_known_enum(const PlanningState value) {
  switch (value) {
    case PlanningState::success:
    case PlanningState::path_valid:
    case PlanningState::waiting_map:
    case PlanningState::request_scout:
    case PlanningState::no_solution:
    case PlanningState::no_solution_under_covariance_envelope:
    case PlanningState::covariance_envelope_breach:
    case PlanningState::input_invalid:
    case PlanningState::map_expired:
    case PlanningState::timeout:
    case PlanningState::communication_degraded:
    case PlanningState::manual_override:
    case PlanningState::init:
    case PlanningState::normal_planning:
    case PlanningState::planning_with_caution:
    case PlanningState::emergency_stop:
      return true;
  }
  return false;
}

bool is_known_enum(const CableModelValidity value) {
  return cable_model_validity_name(value).has_value();
}

class ContractWriter {
 public:
  ContractWriter() {
    output_.imbue(std::locale::classic());
    output_ << std::setprecision(std::numeric_limits<double>::max_digits10);
  }

  template <typename Value>
  void scalar(const Value value) {
    output_ << value << ' ';
  }

  void boolean(const bool value) { scalar(value ? 1 : 0); }

  void text(const std::string& value) { output_ << std::quoted(value) << ' '; }

  [[nodiscard]] std::string finish() const { return output_.str(); }

 private:
  std::ostringstream output_;
};

class ContractReader {
 public:
  explicit ContractReader(const std::string& serialized) : input_(serialized) {
    input_.imbue(std::locale::classic());
  }

  template <typename Value>
  [[nodiscard]] Value scalar() {
    Value value{};
    if (!(input_ >> value)) {
      throw std::invalid_argument("planning result contains a malformed value");
    }
    return value;
  }

  [[nodiscard]] bool boolean() {
    const int value = scalar<int>();
    if (value != 0 && value != 1) {
      throw std::invalid_argument("planning result contains a malformed boolean");
    }
    return value == 1;
  }

  [[nodiscard]] std::string text() {
    std::string value;
    if (!(input_ >> std::quoted(value))) {
      throw std::invalid_argument("planning result contains malformed text");
    }
    return value;
  }

  [[nodiscard]] std::size_t count() {
    constexpr std::uint64_t kMaximumSerializedElements = 1000000;
    const std::uint64_t value = scalar<std::uint64_t>();
    if (value > kMaximumSerializedElements) {
      throw std::invalid_argument("planning result collection is too large");
    }
    return static_cast<std::size_t>(value);
  }

  template <typename Enum>
  [[nodiscard]] Enum enumeration() {
    const int value = scalar<int>();
    const Enum decoded = static_cast<Enum>(value);
    if (!is_known_enum(decoded)) {
      throw std::invalid_argument("planning result contains an unknown enum value");
    }
    return decoded;
  }

  void require_end() {
    input_ >> std::ws;
    if (!input_.eof()) {
      throw std::invalid_argument("planning result contains trailing fields");
    }
  }

 private:
  std::istringstream input_;
};

template <typename Enum>
void write_enum(ContractWriter& writer, const Enum value) {
  writer.scalar(static_cast<int>(value));
}

void write_optional_double(ContractWriter& writer,
                           const std::optional<double>& value);
std::optional<double> read_optional_double(ContractReader& reader);

void write_path(ContractWriter& writer, const GeometricPath& path) {
  writer.scalar(path.metadata.path_version);
  writer.text(path.metadata.coordinate_frame);
  writer.scalar(path.metadata.reference_line_version);
  writer.text(path.metadata.interpolation_rule);
  writer.boolean(path.metadata.smoothing.has_value());
  if (path.metadata.smoothing.has_value()) {
    const PathSmoothingMetadata& smoothing = *path.metadata.smoothing;
    writer.text(smoothing.smoother_version);
    writer.text(smoothing.solver_status);
    writer.scalar(smoothing.limits_version);
    writer.scalar(smoothing.maximum_constraint_residual);
    writer.scalar(smoothing.maximum_absolute_curvature_per_m);
    writer.scalar(smoothing.maximum_absolute_curvature_rate_per_m2);
    writer.scalar(smoothing.residuals.maximum_dynamics_residual);
    writer.scalar(smoothing.residuals.maximum_curvature_audit_residual);
    writer.scalar(smoothing.residuals.maximum_curvature_rate_residual);
    writer.scalar(smoothing.residuals.start_position_residual_m);
    writer.scalar(smoothing.residuals.start_heading_residual_rad);
    writer.scalar(smoothing.residuals.start_curvature_residual_per_m);
    writer.scalar(smoothing.residuals.goal_position_residual_m);
    writer.scalar(smoothing.residuals.goal_heading_residual_rad);
    writer.scalar(smoothing.residuals.goal_curvature_residual_per_m);
  }
  writer.scalar(path.points.size());
  for (const PathPoint& point : path.points) {
    writer.scalar(point.arc_length_m);
    writer.scalar(point.x_m);
    writer.scalar(point.y_m);
    writer.scalar(point.heading_rad);
    writer.scalar(point.curvature_per_m);
  }
}

GeometricPath read_path(ContractReader& reader) {
  GeometricPath path;
  path.metadata.path_version = reader.scalar<std::uint64_t>();
  path.metadata.coordinate_frame = reader.text();
  path.metadata.reference_line_version = reader.scalar<std::uint32_t>();
  path.metadata.interpolation_rule = reader.text();
  if (reader.boolean()) {
    PathSmoothingMetadata smoothing;
    smoothing.smoother_version = reader.text();
    smoothing.solver_status = reader.text();
    smoothing.limits_version = reader.scalar<std::uint64_t>();
    smoothing.maximum_constraint_residual = reader.scalar<double>();
    smoothing.maximum_absolute_curvature_per_m = reader.scalar<double>();
    smoothing.maximum_absolute_curvature_rate_per_m2 = reader.scalar<double>();
    smoothing.residuals.maximum_dynamics_residual = reader.scalar<double>();
    smoothing.residuals.maximum_curvature_audit_residual =
        reader.scalar<double>();
    smoothing.residuals.maximum_curvature_rate_residual =
        reader.scalar<double>();
    smoothing.residuals.start_position_residual_m = reader.scalar<double>();
    smoothing.residuals.start_heading_residual_rad = reader.scalar<double>();
    smoothing.residuals.start_curvature_residual_per_m = reader.scalar<double>();
    smoothing.residuals.goal_position_residual_m = reader.scalar<double>();
    smoothing.residuals.goal_heading_residual_rad = reader.scalar<double>();
    smoothing.residuals.goal_curvature_residual_per_m = reader.scalar<double>();
    path.metadata.smoothing = std::move(smoothing);
  }
  const std::size_t count = reader.count();
  path.points.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    path.points.push_back({reader.scalar<double>(), reader.scalar<double>(),
                           reader.scalar<double>(), reader.scalar<double>(),
                           reader.scalar<double>()});
  }
  return path;
}

void write_limits(ContractWriter& writer, const SpeedPayoutLimits& limits) {
  writer.scalar(limits.ground_speed.minimum_mps);
  writer.scalar(limits.ground_speed.maximum_mps);
  writer.scalar(limits.ground_acceleration.minimum_mps2);
  writer.scalar(limits.ground_acceleration.maximum_mps2);
  writer.scalar(limits.maximum_lateral_acceleration_mps2);
  writer.scalar(limits.payout_speed.minimum_mps);
  writer.scalar(limits.payout_speed.maximum_mps);
  writer.scalar(limits.payout_acceleration.minimum_mps2);
  writer.scalar(limits.payout_acceleration.maximum_mps2);
  writer.scalar(limits.maximum_payout_tracking_error_mps);
  writer.scalar(limits.tension.minimum_n);
  writer.scalar(limits.tension.maximum_n);
  writer.scalar(limits.maximum_stopping_distance_m);
}

SpeedPayoutLimits read_limits(ContractReader& reader) {
  SpeedPayoutLimits limits;
  limits.ground_speed = {reader.scalar<double>(), reader.scalar<double>()};
  limits.ground_acceleration = {reader.scalar<double>(), reader.scalar<double>()};
  limits.maximum_lateral_acceleration_mps2 = reader.scalar<double>();
  limits.payout_speed = {reader.scalar<double>(), reader.scalar<double>()};
  limits.payout_acceleration = {reader.scalar<double>(), reader.scalar<double>()};
  limits.maximum_payout_tracking_error_mps = reader.scalar<double>();
  limits.tension = {reader.scalar<double>(), reader.scalar<double>()};
  limits.maximum_stopping_distance_m = reader.scalar<double>();
  return limits;
}

void write_timed_path(ContractWriter& writer, const TimedPath& path) {
  write_path(writer, path.geometry);
  const ExecutionProfile& profile = path.execution_profile;
  writer.scalar(profile.version);
  writer.scalar(profile.operating_envelope_version);
  writer.text(profile.interpolation_rule);
  write_optional_double(writer, profile.stopping_point_arc_length_m);
  writer.scalar(profile.samples.size());
  for (const ExecutionSample& sample : profile.samples) {
    writer.scalar(sample.arc_length_m);
    writer.scalar(sample.time_from_start.nanoseconds);
    writer.scalar(sample.ground_speed_mps);
    writer.scalar(sample.ground_acceleration_mps2);
    writer.scalar(sample.payout_speed_mps);
    writer.scalar(sample.payout_acceleration_mps2);
    writer.scalar(sample.tension_setpoint_n);
  }
  write_limits(writer, profile.approved_tracking_limits);
}

TimedPath read_timed_path(ContractReader& reader) {
  TimedPath path;
  path.geometry = read_path(reader);
  ExecutionProfile& profile = path.execution_profile;
  profile.version = reader.scalar<std::uint64_t>();
  profile.operating_envelope_version = reader.scalar<std::uint64_t>();
  profile.interpolation_rule = reader.text();
  profile.stopping_point_arc_length_m = read_optional_double(reader);
  const std::size_t count = reader.count();
  profile.samples.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    profile.samples.push_back(
        {reader.scalar<double>(), {reader.scalar<std::int64_t>()},
         reader.scalar<double>(), reader.scalar<double>(),
         reader.scalar<double>(), reader.scalar<double>(),
         reader.scalar<double>()});
  }
  profile.approved_tracking_limits = read_limits(reader);
  return path;
}

void write_memory(ContractWriter& writer, const CableConstraintMemory& memory) {
  writer.scalar(memory.previous_distinct_touchdown_points_m.size());
  for (const Vector2m& point : memory.previous_distinct_touchdown_points_m) {
    writer.scalar(point.x_m);
    writer.scalar(point.y_m);
  }
  writer.scalar(memory.trailing_support_samples.size());
  for (const CableHistorySample& sample : memory.trailing_support_samples) {
    writer.scalar(sample.touchdown_arc_length_m);
    writer.scalar(sample.touchdown_position_m.x_m);
    writer.scalar(sample.touchdown_position_m.y_m);
  }
  writer.scalar(memory.retained_arc_length_m);
  writer.scalar(memory.canonical_signature);
}

CableConstraintMemory read_memory(ContractReader& reader) {
  CableConstraintMemory memory;
  const std::size_t point_count = reader.count();
  memory.previous_distinct_touchdown_points_m.reserve(point_count);
  for (std::size_t index = 0; index < point_count; ++index) {
    memory.previous_distinct_touchdown_points_m.push_back(
        {reader.scalar<double>(), reader.scalar<double>()});
  }
  const std::size_t history_count = reader.count();
  memory.trailing_support_samples.reserve(history_count);
  for (std::size_t index = 0; index < history_count; ++index) {
    memory.trailing_support_samples.push_back(
        {reader.scalar<double>(),
         {reader.scalar<double>(), reader.scalar<double>()}});
  }
  memory.retained_arc_length_m = reader.scalar<double>();
  memory.canonical_signature = reader.scalar<std::uint64_t>();
  return memory;
}

void write_cable_state(ContractWriter& writer, const CableState& state) {
  write_enum(writer, state.kind);
  writer.scalar(state.lag_angle_rad);
  writer.boolean(state.lag_angle_variance_rad2.has_value());
  if (state.lag_angle_variance_rad2.has_value()) {
    writer.scalar(*state.lag_angle_variance_rad2);
  }
  writer.scalar(state.timestamp.nanoseconds);
  write_memory(writer, state.laying_memory);
  writer.scalar(state.sequence_number);
}

CableState read_cable_state(ContractReader& reader) {
  CableState state;
  state.kind = reader.enumeration<CableStateKind>();
  state.lag_angle_rad = reader.scalar<double>();
  if (reader.boolean()) {
    state.lag_angle_variance_rad2 = reader.scalar<double>();
  }
  state.timestamp.nanoseconds = reader.scalar<std::int64_t>();
  state.laying_memory = read_memory(reader);
  state.sequence_number = reader.scalar<std::uint64_t>();
  return state;
}

void write_covariance(ContractWriter& writer,
                      const Covariance2dM2& covariance) {
  writer.scalar(covariance.xx_m2);
  writer.scalar(covariance.xy_m2);
  writer.scalar(covariance.yx_m2);
  writer.scalar(covariance.yy_m2);
}

Covariance2dM2 read_covariance(ContractReader& reader) {
  return {reader.scalar<double>(), reader.scalar<double>(),
          reader.scalar<double>(), reader.scalar<double>()};
}

void write_optional_double(ContractWriter& writer,
                           const std::optional<double>& value) {
  writer.boolean(value.has_value());
  if (value.has_value()) {
    writer.scalar(*value);
  }
}

std::optional<double> read_optional_double(ContractReader& reader) {
  if (!reader.boolean()) {
    return std::nullopt;
  }
  return reader.scalar<double>();
}

void write_error_budget(ContractWriter& writer, const ErrorBudget& budget) {
  write_covariance(writer, budget.robot_position_covariance_m2);
  writer.scalar(budget.touchdown_position_covariance_m2.size());
  for (const Covariance2dM2& covariance :
       budget.touchdown_position_covariance_m2) {
    write_covariance(writer, covariance);
  }
  write_optional_double(writer, budget.epsilon_robot);
  write_optional_double(writer, budget.epsilon_terrain_gradient_local);
  write_optional_double(writer, budget.epsilon_point);
  write_optional_double(writer, budget.epsilon_path);
  writer.boolean(budget.reference_is_deterministic);
  writer.boolean(budget.path_joint_risk_implemented);
  writer.boolean(budget.terrain_gradient_path_joint_risk_implemented);
  writer.text(budget.calibration_dataset_id);
  writer.text(budget.terrain_gradient_calibration_dataset_id);
  writer.scalar(budget.terrain_gradient_policy_version);
  writer.scalar(budget.corridor_risk_policy_version);
  writer.scalar(budget.cable_model_version);
  writer.scalar(budget.uncertainty_envelope_version);
  writer.scalar(budget.uncertainty_envelope_generator_version);
  writer.scalar(budget.execution_operating_envelope_version);
  writer.text(budget.operating_domain_id);
  write_enum(writer, budget.sensor_mode);
  writer.boolean(budget.covariance_envelope_audit_passed);
}

ErrorBudget read_error_budget(ContractReader& reader) {
  ErrorBudget budget;
  budget.robot_position_covariance_m2 = read_covariance(reader);
  const std::size_t count = reader.count();
  budget.touchdown_position_covariance_m2.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    budget.touchdown_position_covariance_m2.push_back(read_covariance(reader));
  }
  budget.epsilon_robot = read_optional_double(reader);
  budget.epsilon_terrain_gradient_local = read_optional_double(reader);
  budget.epsilon_point = read_optional_double(reader);
  budget.epsilon_path = read_optional_double(reader);
  budget.reference_is_deterministic = reader.boolean();
  budget.path_joint_risk_implemented = reader.boolean();
  budget.terrain_gradient_path_joint_risk_implemented = reader.boolean();
  budget.calibration_dataset_id = reader.text();
  budget.terrain_gradient_calibration_dataset_id = reader.text();
  budget.terrain_gradient_policy_version = reader.scalar<std::uint64_t>();
  budget.corridor_risk_policy_version = reader.scalar<std::uint64_t>();
  budget.cable_model_version = reader.scalar<std::uint64_t>();
  budget.uncertainty_envelope_version = reader.scalar<std::uint64_t>();
  budget.uncertainty_envelope_generator_version = reader.scalar<std::uint64_t>();
  budget.execution_operating_envelope_version = reader.scalar<std::uint64_t>();
  budget.operating_domain_id = reader.text();
  budget.sensor_mode = reader.enumeration<SensorHealthMode>();
  budget.covariance_envelope_audit_passed = reader.boolean();
  return budget;
}

void write_corridor(ContractWriter& writer,
                    const CableCorridorResult& corridor) {
  write_enum(writer, corridor.validity);
  writer.boolean(corridor.hard_feasible);
  writer.scalar(corridor.points.size());
  for (const CableCorridorPointResult& point : corridor.points) {
    write_enum(writer, point.status);
    write_enum(writer, point.basis);
    writer.scalar(point.touchdown_arc_length_m);
    writer.scalar(point.reference_progress_m);
    writer.scalar(point.mean_lateral_error_m);
    writer.scalar(point.lateral_stddev_m);
    writer.scalar(point.upper_bound_m);
  }
  writer.scalar(corridor.marginal_count);
  writer.scalar(corridor.violation_count);
  writer.scalar(corridor.total_marginal_length_m);
  writer.scalar(corridor.total_violation_length_m);
  writer.scalar(corridor.maximum_marginal_length_m);
  writer.boolean(corridor.marginal_length_limit_exceeded);
  writer.scalar(corridor.epsilon_point);
  writer.scalar(corridor.corridor_risk_policy_version);
  writer.scalar(corridor.reference_line_version);
  writer.scalar(corridor.interval_bound_certificate.version);
  writer.scalar(
      corridor.interval_bound_certificate.upper_bound_error_m.size());
  for (const double error_m :
       corridor.interval_bound_certificate.upper_bound_error_m) {
    writer.scalar(error_m);
  }
  writer.scalar(corridor.evaluation_timestamp.nanoseconds);
  writer.text(corridor.operating_domain_id);
  writer.text(corridor.residual_distribution_calibration_dataset_id);
  writer.boolean(corridor.reference_is_deterministic);
  writer.boolean(corridor.covariance_includes_coordinate_transform_error);
  writer.boolean(corridor.covariance_envelope_audit_performed);
  writer.boolean(corridor.path_joint_risk_implemented);
  writer.text(corridor.risk_semantics);
  writer.scalar(corridor.issues.size());
  for (const std::string& issue : corridor.issues) writer.text(issue);
}

CableCorridorResult read_corridor(ContractReader& reader) {
  CableCorridorResult corridor;
  corridor.validity = reader.enumeration<CorridorEvaluationValidity>();
  corridor.hard_feasible = reader.boolean();
  const std::size_t count = reader.count();
  corridor.points.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    CableCorridorPointResult point;
    point.status = reader.enumeration<CableValidationStatus>();
    point.basis = reader.enumeration<CableCorridorPointBasis>();
    point.touchdown_arc_length_m = reader.scalar<double>();
    point.reference_progress_m = reader.scalar<double>();
    point.mean_lateral_error_m = reader.scalar<double>();
    point.lateral_stddev_m = reader.scalar<double>();
    point.upper_bound_m = reader.scalar<double>();
    corridor.points.push_back(point);
  }
  corridor.marginal_count = reader.scalar<std::uint64_t>();
  corridor.violation_count = reader.scalar<std::uint64_t>();
  corridor.total_marginal_length_m = reader.scalar<double>();
  corridor.total_violation_length_m = reader.scalar<double>();
  corridor.maximum_marginal_length_m = reader.scalar<double>();
  corridor.marginal_length_limit_exceeded = reader.boolean();
  corridor.epsilon_point = reader.scalar<double>();
  corridor.corridor_risk_policy_version = reader.scalar<std::uint64_t>();
  corridor.reference_line_version = reader.scalar<std::uint32_t>();
  corridor.interval_bound_certificate.version =
      reader.scalar<std::uint64_t>();
  const std::size_t interval_error_count = reader.count();
  corridor.interval_bound_certificate.upper_bound_error_m.reserve(
      interval_error_count);
  for (std::size_t index = 0; index < interval_error_count; ++index) {
    corridor.interval_bound_certificate.upper_bound_error_m.push_back(
        reader.scalar<double>());
  }
  corridor.evaluation_timestamp.nanoseconds = reader.scalar<std::int64_t>();
  corridor.operating_domain_id = reader.text();
  corridor.residual_distribution_calibration_dataset_id = reader.text();
  corridor.reference_is_deterministic = reader.boolean();
  corridor.covariance_includes_coordinate_transform_error = reader.boolean();
  corridor.covariance_envelope_audit_performed = reader.boolean();
  corridor.path_joint_risk_implemented = reader.boolean();
  corridor.risk_semantics = reader.text();
  const std::size_t issue_count = reader.count();
  corridor.issues.reserve(issue_count);
  for (std::size_t index = 0; index < issue_count; ++index) {
    corridor.issues.push_back(reader.text());
  }
  return corridor;
}

void write_laying(ContractWriter& writer,
                  const CableLayingEvaluation& laying) {
  writer.boolean(laying.valid);
  writer.boolean(laying.hard_feasible);
  writer.scalar(laying.failure_reasons.size());
  for (const CableLayingFailure failure : laying.failure_reasons) {
    write_enum(writer, failure);
  }
  writer.scalar(laying.failure_segments.size());
  for (const CableLayingFailureSegment& segment : laying.failure_segments) {
    write_enum(writer, segment.reason);
    writer.scalar(segment.start_arc_length_m);
    writer.scalar(segment.end_arc_length_m);
    writer.scalar(segment.representative_position_m.x_m);
    writer.scalar(segment.representative_position_m.y_m);
  }
  writer.scalar(laying.limits_version);
  writer.scalar(laying.terrain_map_sequence);
  writer.scalar(laying.terrain_analysis_config_version);
  writer.text(laying.operating_domain_id);
  writer.text(laying.risk_semantics);
  writer.scalar(laying.maximum_absolute_curvature_per_m);
  writer.boolean(laying.maximum_absolute_curvature_position_m.has_value());
  if (laying.maximum_absolute_curvature_position_m.has_value()) {
    writer.scalar(laying.maximum_absolute_curvature_position_m->x_m);
    writer.scalar(laying.maximum_absolute_curvature_position_m->y_m);
  }
  writer.scalar(laying.maximum_support_proxy_range_m);
  writer.boolean(laying.maximum_support_proxy_position_m.has_value());
  if (laying.maximum_support_proxy_position_m.has_value()) {
    writer.scalar(laying.maximum_support_proxy_position_m->x_m);
    writer.scalar(laying.maximum_support_proxy_position_m->y_m);
  }
  writer.scalar(laying.terminal_support_window_length_m);
  writer.scalar(laying.soft_cost);
  write_memory(writer, laying.terminal_memory);
}

CableLayingEvaluation read_laying(ContractReader& reader) {
  CableLayingEvaluation laying;
  laying.valid = reader.boolean();
  laying.hard_feasible = reader.boolean();
  const std::size_t count = reader.count();
  laying.failure_reasons.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    laying.failure_reasons.push_back(
        reader.enumeration<CableLayingFailure>());
  }
  const std::size_t segment_count = reader.count();
  laying.failure_segments.reserve(segment_count);
  for (std::size_t index = 0; index < segment_count; ++index) {
    laying.failure_segments.push_back(
        {reader.enumeration<CableLayingFailure>(), reader.scalar<double>(),
         reader.scalar<double>(),
         {reader.scalar<double>(), reader.scalar<double>()}});
  }
  laying.limits_version = reader.scalar<std::uint64_t>();
  laying.terrain_map_sequence = reader.scalar<std::uint64_t>();
  laying.terrain_analysis_config_version = reader.scalar<std::uint64_t>();
  laying.operating_domain_id = reader.text();
  laying.risk_semantics = reader.text();
  laying.maximum_absolute_curvature_per_m = reader.scalar<double>();
  if (reader.boolean()) {
    laying.maximum_absolute_curvature_position_m =
        Vector2m{reader.scalar<double>(), reader.scalar<double>()};
  }
  laying.maximum_support_proxy_range_m = reader.scalar<double>();
  if (reader.boolean()) {
    laying.maximum_support_proxy_position_m =
        Vector2m{reader.scalar<double>(), reader.scalar<double>()};
  }
  laying.terminal_support_window_length_m = reader.scalar<double>();
  laying.soft_cost = reader.scalar<double>();
  laying.terminal_memory = read_memory(reader);
  return laying;
}

void write_diagnostics(ContractWriter& writer,
                       const Diagnostics& diagnostics) {
  writer.text(diagnostics.schema_version);
  writer.scalar(diagnostics.random_seed);
  writer.text(diagnostics.input_version);
  writer.text(diagnostics.unit_system);
  writer.text(diagnostics.operating_domain_id);
  writer.text(diagnostics.risk_semantics);
  const PlanningDependencyVersions& dependencies = diagnostics.dependencies;
  writer.text(dependencies.map_version.map_id);
  writer.scalar(dependencies.map_version.sequence_number);
  writer.scalar(dependencies.map_version.timestamp.nanoseconds);
  writer.text(dependencies.map_version.coordinate_frame);
  writer.scalar(dependencies.reference_line_version);
  writer.scalar(dependencies.robot_operating_area_version);
  writer.scalar(dependencies.terrain_gradient_policy_version);
  writer.scalar(dependencies.corridor_risk_policy_version);
  writer.scalar(dependencies.cable_model_version);
  writer.scalar(dependencies.uncertainty_envelope_version);
  writer.scalar(dependencies.uncertainty_envelope_generator_version);
  writer.scalar(dependencies.execution_operating_envelope_version);
  writer.scalar(dependencies.execution_profile_version);
  write_enum(writer, dependencies.sensor_mode);
  writer.text(dependencies.operating_domain_id);
  writer.scalar(dependencies.cable_corridor_version);
  writer.scalar(diagnostics.entries.size());
  for (const DiagnosticEntry& entry : diagnostics.entries) {
    write_enum(writer, entry.severity);
    writer.text(entry.code);
    writer.text(entry.stage);
    writer.text(entry.message);
    writer.scalar(entry.timestamp.nanoseconds);
  }
}

Diagnostics read_diagnostics(ContractReader& reader) {
  Diagnostics diagnostics;
  diagnostics.schema_version = reader.text();
  diagnostics.random_seed = reader.scalar<std::uint64_t>();
  diagnostics.input_version = reader.text();
  diagnostics.unit_system = reader.text();
  diagnostics.operating_domain_id = reader.text();
  diagnostics.risk_semantics = reader.text();
  PlanningDependencyVersions& dependencies = diagnostics.dependencies;
  dependencies.map_version.map_id = reader.text();
  dependencies.map_version.sequence_number = reader.scalar<std::uint64_t>();
  dependencies.map_version.timestamp.nanoseconds = reader.scalar<std::int64_t>();
  dependencies.map_version.coordinate_frame = reader.text();
  dependencies.reference_line_version = reader.scalar<std::uint32_t>();
  dependencies.robot_operating_area_version = reader.scalar<std::uint32_t>();
  dependencies.terrain_gradient_policy_version = reader.scalar<std::uint64_t>();
  dependencies.corridor_risk_policy_version = reader.scalar<std::uint64_t>();
  dependencies.cable_model_version = reader.scalar<std::uint64_t>();
  dependencies.uncertainty_envelope_version = reader.scalar<std::uint64_t>();
  dependencies.uncertainty_envelope_generator_version =
      reader.scalar<std::uint64_t>();
  dependencies.execution_operating_envelope_version =
      reader.scalar<std::uint64_t>();
  dependencies.execution_profile_version = reader.scalar<std::uint64_t>();
  dependencies.sensor_mode = reader.enumeration<SensorHealthMode>();
  dependencies.operating_domain_id = reader.text();
  dependencies.cable_corridor_version = reader.scalar<std::uint32_t>();
  const std::size_t count = reader.count();
  diagnostics.entries.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    diagnostics.entries.push_back(
        {reader.enumeration<DiagnosticSeverity>(), reader.text(), reader.text(),
         reader.text(), {reader.scalar<std::int64_t>()}});
  }
  return diagnostics;
}

}  // namespace

std::string_view to_string(const CableModelValidity validity) noexcept {
  return cable_model_validity_name(validity).value_or("UNKNOWN");
}

double normalize_angle_radians(const double angle_rad) {
  if (!std::isfinite(angle_rad)) {
    throw std::invalid_argument("angle_rad must be finite");
  }
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;
  double normalized = std::fmod(angle_rad + kPi, kTwoPi);
  if (normalized < 0.0) {
    normalized += kTwoPi;
  }
  return normalized - kPi;
}

std::string_view to_string(const PlanningState state) {
  switch (state) {
    case PlanningState::success:
      return "SUCCESS";
    case PlanningState::path_valid:
      return "PATH_VALID";
    case PlanningState::waiting_map:
      return "WAITING_MAP";
    case PlanningState::request_scout:
      return "REQUEST_SCOUT";
    case PlanningState::no_solution:
      return "NO_SOLUTION";
    case PlanningState::no_solution_under_covariance_envelope:
      return "NO_SOLUTION_UNDER_COVARIANCE_ENVELOPE";
    case PlanningState::covariance_envelope_breach:
      return "COVARIANCE_ENVELOPE_BREACH";
    case PlanningState::input_invalid:
      return "INPUT_INVALID";
    case PlanningState::map_expired:
      return "MAP_EXPIRED";
    case PlanningState::timeout:
      return "TIMEOUT";
    case PlanningState::communication_degraded:
      return "COMM_DEGRADED";
    case PlanningState::manual_override:
      return "MANUAL_OVERRIDE";
    case PlanningState::init:
      return "INIT";
    case PlanningState::normal_planning:
      return "NORMAL_PLANNING";
    case PlanningState::planning_with_caution:
      return "PLANNING_WITH_CAUTION";
    case PlanningState::emergency_stop:
      return "EMERGENCY_STOP";
  }
  throw std::invalid_argument("unknown PlanningState");
}

ValidationResult validate(const RobotState& state) {
  ValidationResult result{true, {}};
  require_finite(state.pose.x_m, "pose.x_m must be finite", result);
  require_finite(state.pose.y_m, "pose.y_m must be finite", result);
  require_finite(state.pose.heading_rad, "pose.heading_rad must be finite", result);
  require_finite(state.ground_speed_mps, "ground_speed_mps must be finite", result);
  require_finite(state.curvature_per_m, "curvature_per_m must be finite", result);
  if (state.pose.timestamp.nanoseconds < 0) {
    result.issues.emplace_back("pose timestamp must be monotonic");
  }
  if (state.curvature_timestamp.nanoseconds < 0) {
    result.issues.emplace_back("curvature timestamp must be monotonic");
  }
  if (state.sequence_number == 0) {
    result.issues.emplace_back("robot state sequence_number must be nonzero");
  }
  if (!is_normalized_angle(state.pose.heading_rad)) {
    result.issues.emplace_back("pose.heading_rad must be normalized");
  }
  result.valid = result.issues.empty();
  return result;
}

ValidationResult validate(const CableState& state) {
  ValidationResult result{true, {}};
  if (!is_known_enum(state.kind)) {
    result.issues.emplace_back("cable state kind is unknown");
  }
  if (!is_normalized_angle(state.lag_angle_rad)) {
    result.issues.emplace_back("lag_angle_rad must be finite and normalized");
  }
  if (state.kind == CableStateKind::tracked) {
    if (!state.lag_angle_variance_rad2.has_value()) {
      result.issues.emplace_back("tracked cable state requires angle variance");
    } else {
      require_nonnegative_finite(*state.lag_angle_variance_rad2,
                                 "lag_angle_variance_rad2 must be nonnegative",
                                 result);
    }
  } else if (state.lag_angle_variance_rad2.has_value()) {
    result.issues.emplace_back("search mean cable state cannot carry variance");
  }
  if (state.timestamp.nanoseconds < 0) {
    result.issues.emplace_back("cable state timestamp must be monotonic");
  }
  if (state.sequence_number == 0) {
    result.issues.emplace_back("cable state sequence_number must be nonzero");
  }
  validate_memory(state.laying_memory, result);
  result.valid = result.issues.empty();
  return result;
}

ValidationResult validate(const ReferenceProgress& progress) {
  ValidationResult result{true, {}};
  if (progress.reference_line_version == 0) {
    result.issues.emplace_back("reference line version must be nonzero");
  }
  require_nonnegative_finite(progress.arc_length_m,
                             "reference progress must be nonnegative", result);
  if (progress.timestamp.nanoseconds < 0) {
    result.issues.emplace_back("reference progress timestamp must be monotonic");
  }
  if (progress.sequence_number == 0) {
    result.issues.emplace_back("reference progress sequence_number must be nonzero");
  }
  result.valid = result.issues.empty();
  return result;
}

ValidationResult validate(const GeometricPath& path) {
  ValidationResult result{true, {}};
  if (path.metadata.path_version == 0) {
    result.issues.emplace_back("path version must be nonzero");
  }
  if (path.metadata.coordinate_frame.empty()) {
    result.issues.emplace_back("path coordinate frame must be specified");
  }
  if (path.metadata.reference_line_version == 0) {
    result.issues.emplace_back("path reference line version must be nonzero");
  }
  if (path.metadata.interpolation_rule.empty()) {
    result.issues.emplace_back("path interpolation rule must be specified");
  }
  if (path.metadata.smoothing.has_value()) {
    const PathSmoothingMetadata& smoothing = *path.metadata.smoothing;
    if (smoothing.smoother_version.empty() || smoothing.solver_status.empty() ||
        smoothing.limits_version == 0U) {
      result.issues.emplace_back("path smoothing identity is incomplete");
    }
    const PathConstraintResiduals& residuals = smoothing.residuals;
    const std::array<double, 12U> residual_and_limit_values{
        smoothing.maximum_constraint_residual,
        smoothing.maximum_absolute_curvature_per_m,
        smoothing.maximum_absolute_curvature_rate_per_m2,
        residuals.maximum_dynamics_residual,
        residuals.maximum_curvature_audit_residual,
        residuals.maximum_curvature_rate_residual,
        residuals.start_position_residual_m,
        residuals.start_heading_residual_rad,
        residuals.start_curvature_residual_per_m,
        residuals.goal_position_residual_m,
        residuals.goal_heading_residual_rad,
        residuals.goal_curvature_residual_per_m,
    };
    for (const double value : residual_and_limit_values) {
      require_nonnegative_finite(
          value, "path smoothing metadata must be finite and nonnegative",
          result);
    }
  }
  if (path.points.size() < 2) {
    result.issues.emplace_back("path must contain at least two points");
  }
  double previous_arc_length_m = -1.0;
  for (const PathPoint& point : path.points) {
    require_nonnegative_finite(point.arc_length_m,
                               "path arc_length_m must be nonnegative", result);
    require_finite(point.x_m, "path x_m must be finite", result);
    require_finite(point.y_m, "path y_m must be finite", result);
    require_finite(point.curvature_per_m,
                   "path curvature_per_m must be finite", result);
    if (!is_normalized_angle(point.heading_rad)) {
      result.issues.emplace_back("path heading_rad must be finite and normalized");
    }
    if (point.arc_length_m <= previous_arc_length_m) {
      result.issues.emplace_back("path arc length must be strictly increasing");
    }
    previous_arc_length_m = point.arc_length_m;
  }
  result.valid = result.issues.empty();
  return result;
}

ValidationResult validate_timed_path(const TimedPath& path,
                                     const bool allow_authorized_prefix) {
  ValidationResult result{true, {}};
  append_issues(validate(path.geometry), result);
  const ExecutionProfile& profile = path.execution_profile;
  if (profile.version == 0 || profile.operating_envelope_version == 0) {
    result.issues.emplace_back("execution profile versions must be nonzero");
  }
  if (profile.interpolation_rule.empty()) {
    result.issues.emplace_back("execution interpolation rule must be specified");
  }
  if (profile.samples.size() < 2) {
    result.issues.emplace_back("execution profile must contain at least two samples");
  }
  double previous_arc_length_m = -1.0;
  std::int64_t previous_time_ns = -1;
  for (const ExecutionSample& sample : profile.samples) {
    require_nonnegative_finite(sample.arc_length_m,
                               "execution arc_length_m must be nonnegative", result);
    require_finite(sample.ground_speed_mps,
                   "execution ground_speed_mps must be finite", result);
    require_finite(sample.ground_acceleration_mps2,
                   "execution ground_acceleration_mps2 must be finite", result);
    require_finite(sample.payout_speed_mps,
                   "execution payout_speed_mps must be finite", result);
    require_finite(sample.payout_acceleration_mps2,
                   "execution payout_acceleration_mps2 must be finite", result);
    require_finite(sample.tension_setpoint_n,
                   "execution tension_setpoint_n must be finite", result);
    if (sample.arc_length_m <= previous_arc_length_m) {
      result.issues.emplace_back("execution arc length must be strictly increasing");
    }
    if (sample.time_from_start.nanoseconds <= previous_time_ns) {
      result.issues.emplace_back("execution time must be strictly increasing");
    }
    previous_arc_length_m = sample.arc_length_m;
    previous_time_ns = sample.time_from_start.nanoseconds;
  }
  if (!profile.samples.empty() &&
      profile.samples.front().time_from_start.nanoseconds != 0) {
    result.issues.emplace_back("execution profile time must start at zero");
  }
  if (!profile.samples.empty() && !path.geometry.points.empty()) {
    constexpr double kArcLengthToleranceM = 1.0e-9;
    if (std::abs(profile.samples.front().arc_length_m -
                 path.geometry.points.front().arc_length_m) >
            kArcLengthToleranceM ||
        std::abs(profile.samples.back().arc_length_m -
                 path.geometry.points.back().arc_length_m) >
            kArcLengthToleranceM) {
      result.issues.emplace_back("execution profile must span its geometry");
    }
  }
  if (!profile.stopping_point_arc_length_m.has_value()) {
    result.issues.emplace_back("execution profile must define a stopping point");
  } else {
    require_nonnegative_finite(*profile.stopping_point_arc_length_m,
                               "stopping point must be a finite arc length", result);
    constexpr double kStoppingPointToleranceM = 1.0e-9;
    constexpr double kStoppedSpeedToleranceMps = 1.0e-9;
    // A TimedPath may be an authorized prefix (for example, the immutable
    // commitment segment).  In that case its explicit stopping point is
    // beyond the prefix geometry and the prefix is intentionally still in
    // motion.  Complete paths retain the strict zero-speed stopping gate.
    const bool is_nonterminal_prefix = allow_authorized_prefix &&
        !path.geometry.points.empty() &&
        *profile.stopping_point_arc_length_m >
            path.geometry.points.back().arc_length_m + kStoppingPointToleranceM;
    if (!is_nonterminal_prefix) {
      bool found_stopped_sample = false;
      for (const ExecutionSample& sample : profile.samples) {
        if (std::abs(sample.arc_length_m -
                     *profile.stopping_point_arc_length_m) <=
                kStoppingPointToleranceM &&
            std::abs(sample.ground_speed_mps) <= kStoppedSpeedToleranceMps) {
          found_stopped_sample = true;
          break;
        }
      }
      if (!found_stopped_sample) {
        result.issues.emplace_back(
            "stopping point must identify a zero-ground-speed sample");
      }
    }
  }
  const SpeedPayoutLimits& limits = profile.approved_tracking_limits;
  validate_range(limits.ground_speed.minimum_mps,
                 limits.ground_speed.maximum_mps, "invalid ground speed range",
                 result);
  validate_range(limits.ground_acceleration.minimum_mps2,
                 limits.ground_acceleration.maximum_mps2,
                 "invalid ground acceleration range", result);
  validate_range(limits.payout_speed.minimum_mps,
                 limits.payout_speed.maximum_mps, "invalid payout speed range",
                 result);
  validate_range(limits.payout_acceleration.minimum_mps2,
                 limits.payout_acceleration.maximum_mps2,
                 "invalid payout acceleration range", result);
  validate_range(limits.tension.minimum_n, limits.tension.maximum_n,
                 "invalid tension range", result);
  require_nonnegative_finite(limits.maximum_lateral_acceleration_mps2,
                             "maximum lateral acceleration must be nonnegative",
                             result);
  require_nonnegative_finite(limits.maximum_payout_tracking_error_mps,
                             "maximum payout error must be nonnegative", result);
  require_nonnegative_finite(limits.maximum_stopping_distance_m,
                             "maximum stopping distance must be nonnegative",
                             result);
  const auto curvature_at = [&path](const double arc_length_m) {
    if (!std::isfinite(arc_length_m) || path.geometry.points.size() < 2U ||
        arc_length_m < path.geometry.points.front().arc_length_m ||
        arc_length_m > path.geometry.points.back().arc_length_m) {
      return std::optional<double>{};
    }
    std::size_t upper_index = 1U;
    while (upper_index < path.geometry.points.size() &&
           path.geometry.points[upper_index].arc_length_m < arc_length_m) {
      ++upper_index;
    }
    if (upper_index == path.geometry.points.size()) {
      return std::optional<double>{};
    }
    const PathPoint& left = path.geometry.points[upper_index - 1U];
    const PathPoint& right = path.geometry.points[upper_index];
    const double span_m = right.arc_length_m - left.arc_length_m;
    if (!std::isfinite(span_m) || span_m <= 0.0) {
      return std::optional<double>{};
    }
    const double ratio = (arc_length_m - left.arc_length_m) / span_m;
    return std::optional<double>{
        left.curvature_per_m +
        ratio * (right.curvature_per_m - left.curvature_per_m)};
  };
  const auto outside = [](const double value, const double minimum,
                          const double maximum) {
    return std::isfinite(value) && std::isfinite(minimum) &&
           std::isfinite(maximum) && (value < minimum || value > maximum);
  };
  for (const ExecutionSample& sample : profile.samples) {
    if (outside(sample.ground_speed_mps, limits.ground_speed.minimum_mps,
                limits.ground_speed.maximum_mps)) {
      result.issues.emplace_back(
          "execution ground speed exceeds approved limits");
    }
    if (outside(sample.ground_acceleration_mps2,
                limits.ground_acceleration.minimum_mps2,
                limits.ground_acceleration.maximum_mps2)) {
      result.issues.emplace_back(
          "execution ground acceleration exceeds approved limits");
    }
    if (outside(sample.payout_speed_mps, limits.payout_speed.minimum_mps,
                limits.payout_speed.maximum_mps)) {
      result.issues.emplace_back(
          "execution payout speed exceeds approved limits");
    }
    if (outside(sample.payout_acceleration_mps2,
                limits.payout_acceleration.minimum_mps2,
                limits.payout_acceleration.maximum_mps2)) {
      result.issues.emplace_back(
          "execution payout acceleration exceeds approved limits");
    }
    if (outside(sample.tension_setpoint_n, limits.tension.minimum_n,
                limits.tension.maximum_n)) {
      result.issues.emplace_back(
          "execution tension setpoint exceeds approved limits");
    }
    const std::optional<double> curvature_per_m =
        curvature_at(sample.arc_length_m);
    if (curvature_per_m.has_value() &&
        std::isfinite(sample.ground_speed_mps) &&
        std::isfinite(limits.maximum_lateral_acceleration_mps2) &&
        sample.ground_speed_mps * sample.ground_speed_mps *
                std::abs(*curvature_per_m) >
            limits.maximum_lateral_acceleration_mps2) {
      result.issues.emplace_back(
          "execution lateral acceleration exceeds approved limits");
    }
  }
  result.valid = result.issues.empty();
  return result;
}

ValidationResult validate(const TimedPath& path) {
  return validate_timed_path(path, false);
}

ValidationResult validate_authorized_prefix(const TimedPath& path) {
  return validate_timed_path(path, true);
}

ExecutionProfileVersioner::ExecutionProfileVersioner(
    const std::uint64_t last_issued_version) noexcept
    : last_issued_version_(last_issued_version) {}

bool same_execution_profile_content(const ExecutionProfile& left,
                                    const ExecutionProfile& right) noexcept {
  const auto sample_equal = [](const ExecutionSample& left_sample,
                               const ExecutionSample& right_sample) {
    return left_sample.arc_length_m == right_sample.arc_length_m &&
           left_sample.time_from_start.nanoseconds ==
               right_sample.time_from_start.nanoseconds &&
           left_sample.ground_speed_mps == right_sample.ground_speed_mps &&
           left_sample.ground_acceleration_mps2 ==
               right_sample.ground_acceleration_mps2 &&
           left_sample.payout_speed_mps == right_sample.payout_speed_mps &&
           left_sample.payout_acceleration_mps2 ==
               right_sample.payout_acceleration_mps2 &&
           left_sample.tension_setpoint_n == right_sample.tension_setpoint_n;
  };
  const SpeedPayoutLimits& left_limits = left.approved_tracking_limits;
  const SpeedPayoutLimits& right_limits = right.approved_tracking_limits;
  if (left.operating_envelope_version != right.operating_envelope_version ||
      left.interpolation_rule != right.interpolation_rule ||
      left.stopping_point_arc_length_m !=
          right.stopping_point_arc_length_m ||
      left.samples.size() != right.samples.size() ||
      left_limits.ground_speed.minimum_mps !=
          right_limits.ground_speed.minimum_mps ||
      left_limits.ground_speed.maximum_mps !=
          right_limits.ground_speed.maximum_mps ||
      left_limits.ground_acceleration.minimum_mps2 !=
          right_limits.ground_acceleration.minimum_mps2 ||
      left_limits.ground_acceleration.maximum_mps2 !=
          right_limits.ground_acceleration.maximum_mps2 ||
      left_limits.maximum_lateral_acceleration_mps2 !=
          right_limits.maximum_lateral_acceleration_mps2 ||
      left_limits.payout_speed.minimum_mps !=
          right_limits.payout_speed.minimum_mps ||
      left_limits.payout_speed.maximum_mps !=
          right_limits.payout_speed.maximum_mps ||
      left_limits.payout_acceleration.minimum_mps2 !=
          right_limits.payout_acceleration.minimum_mps2 ||
      left_limits.payout_acceleration.maximum_mps2 !=
          right_limits.payout_acceleration.maximum_mps2 ||
      left_limits.maximum_payout_tracking_error_mps !=
          right_limits.maximum_payout_tracking_error_mps ||
      left_limits.tension.minimum_n != right_limits.tension.minimum_n ||
      left_limits.tension.maximum_n != right_limits.tension.maximum_n ||
      left_limits.maximum_stopping_distance_m !=
          right_limits.maximum_stopping_distance_m) {
    return false;
  }
  for (std::size_t index = 0; index < left.samples.size(); ++index) {
    if (!sample_equal(left.samples[index], right.samples[index])) {
      return false;
    }
  }
  return true;
}

ValidationResult validate_execution_profile_revision(
    const ExecutionProfile& previous, const ExecutionProfile& revised) {
  ValidationResult result{true, {}};
  if (previous.version == 0U || revised.version == 0U) {
    result.issues.emplace_back(
        "execution profile revision requires nonzero versions");
  } else if (revised.version < previous.version) {
    result.issues.emplace_back(
        "execution profile revision must not regress its version");
  } else if (!same_execution_profile_content(previous, revised) &&
             revised.version == previous.version) {
    result.issues.emplace_back(
        "changed execution profile content requires a newer version");
  }
  result.valid = result.issues.empty();
  return result;
}

ExecutionProfile ExecutionProfileVersioner::assign_version(
    ExecutionProfile profile) {
  if (!last_profile_.has_value() ||
      !same_execution_profile_content(profile, *last_profile_)) {
    if (last_issued_version_ == std::numeric_limits<std::uint64_t>::max()) {
      throw std::overflow_error("execution profile version space exhausted");
    }
    profile.version = ++last_issued_version_;
    last_profile_ = std::move(profile);
  }
  return *last_profile_;
}

ValidationResult validate(const Covariance2dM2& covariance) {
  ValidationResult result{true, {}};
  validate_covariance(covariance, result);
  result.valid = result.issues.empty();
  return result;
}

ValidationResult validate(const ErrorBudget& budget) {
  ValidationResult result{true, {}};
  if (!is_known_enum(budget.sensor_mode)) {
    result.issues.emplace_back("sensor health mode is unknown");
  }
  validate_covariance(budget.robot_position_covariance_m2, result);
  if (budget.touchdown_position_covariance_m2.empty()) {
    result.issues.emplace_back("touchdown covariance profile must not be empty");
  }
  for (const Covariance2dM2& covariance :
       budget.touchdown_position_covariance_m2) {
    validate_covariance(covariance, result);
  }
  validate_probability(budget.epsilon_robot, "epsilon_robot must be in (0,1)",
                       result);
  validate_probability(budget.epsilon_terrain_gradient_local,
                       "epsilon_terrain_gradient_local must be in (0,1)", result);
  validate_probability(budget.epsilon_point, "epsilon_point must be in (0,1)",
                       result);
  if (budget.epsilon_path.has_value() || budget.path_joint_risk_implemented ||
      budget.terrain_gradient_path_joint_risk_implemented) {
    result.issues.emplace_back(
        "first-version contract cannot claim path joint risk");
  }
  if (!budget.reference_is_deterministic) {
    result.issues.emplace_back("first-version reference must be deterministic");
  }
  if (budget.calibration_dataset_id.empty() ||
      budget.terrain_gradient_calibration_dataset_id.empty() ||
      budget.operating_domain_id.empty()) {
    result.issues.emplace_back("risk datasets and operating domain are required");
  }
  if (budget.terrain_gradient_policy_version == 0 ||
      budget.corridor_risk_policy_version == 0 ||
      budget.cable_model_version == 0 ||
      budget.uncertainty_envelope_version == 0 ||
      budget.uncertainty_envelope_generator_version == 0 ||
      budget.execution_operating_envelope_version == 0) {
    result.issues.emplace_back("risk and model versions must be nonzero");
  }
  if (!budget.covariance_envelope_audit_passed) {
    result.issues.emplace_back("covariance envelope audit must pass");
  }
  result.valid = result.issues.empty();
  return result;
}

void validate_finite_path_fields(const GeometricPath& path,
                                 ValidationResult& result) {
  for (const PathPoint& point : path.points) {
    require_finite(point.arc_length_m, "path arc_length_m must be finite", result);
    require_finite(point.x_m, "path x_m must be finite", result);
    require_finite(point.y_m, "path y_m must be finite", result);
    require_finite(point.heading_rad, "path heading_rad must be finite", result);
    require_finite(point.curvature_per_m, "path curvature must be finite", result);
  }
}

void validate_finite_timed_path_fields(const TimedPath& path,
                                       ValidationResult& result) {
  validate_finite_path_fields(path.geometry, result);
  const ExecutionProfile& profile = path.execution_profile;
  if (profile.stopping_point_arc_length_m.has_value()) {
    require_finite(*profile.stopping_point_arc_length_m,
                   "stopping point must be finite", result);
  }
  for (const ExecutionSample& sample : profile.samples) {
    require_finite(sample.arc_length_m, "execution arc length must be finite",
                   result);
    require_finite(sample.ground_speed_mps, "ground speed must be finite", result);
    require_finite(sample.ground_acceleration_mps2,
                   "ground acceleration must be finite", result);
    require_finite(sample.payout_speed_mps, "payout speed must be finite", result);
    require_finite(sample.payout_acceleration_mps2,
                   "payout acceleration must be finite", result);
    require_finite(sample.tension_setpoint_n, "tension must be finite", result);
  }
  const SpeedPayoutLimits& limits = profile.approved_tracking_limits;
  validate_range(limits.ground_speed.minimum_mps,
                 limits.ground_speed.maximum_mps, "invalid ground speed range",
                 result);
  validate_range(limits.ground_acceleration.minimum_mps2,
                 limits.ground_acceleration.maximum_mps2,
                 "invalid ground acceleration range", result);
  validate_range(limits.payout_speed.minimum_mps,
                 limits.payout_speed.maximum_mps, "invalid payout speed range",
                 result);
  validate_range(limits.payout_acceleration.minimum_mps2,
                 limits.payout_acceleration.maximum_mps2,
                 "invalid payout acceleration range", result);
  validate_range(limits.tension.minimum_n, limits.tension.maximum_n,
                 "invalid tension range", result);
  require_finite(limits.maximum_lateral_acceleration_mps2,
                 "lateral acceleration limit must be finite", result);
  require_finite(limits.maximum_payout_tracking_error_mps,
                 "payout tracking error must be finite", result);
  require_finite(limits.maximum_stopping_distance_m,
                 "stopping distance must be finite", result);
}

void validate_finite_error_budget_fields(const ErrorBudget& budget,
                                         ValidationResult& result) {
  validate_covariance(budget.robot_position_covariance_m2, result);
  for (const Covariance2dM2& covariance :
       budget.touchdown_position_covariance_m2) {
    validate_covariance(covariance, result);
  }
  const std::optional<double> probabilities[]{
      budget.epsilon_robot, budget.epsilon_terrain_gradient_local,
      budget.epsilon_point, budget.epsilon_path};
  for (const std::optional<double>& probability : probabilities) {
    if (probability.has_value() && !std::isfinite(*probability)) {
      result.issues.emplace_back("risk probability must be finite");
    }
  }
}

void validate_all_serialized_numeric_fields(const PlanningResult& value,
                                            ValidationResult& result) {
  validate_finite_timed_path_fields(value.robot_trajectory, result);
  validate_finite_path_fields(value.cable_path, result);
  require_finite(value.terminal_cable_state.lag_angle_rad,
                 "terminal cable lag angle must be finite", result);
  if (value.terminal_cable_state.lag_angle_variance_rad2.has_value()) {
    require_finite(*value.terminal_cable_state.lag_angle_variance_rad2,
                   "terminal cable variance must be finite", result);
  }
  validate_memory(value.terminal_cable_state.laying_memory, result);
  for (const CableCorridorPointResult& point : value.corridor_result.points) {
    require_finite(point.touchdown_arc_length_m,
                   "corridor touchdown arc length must be finite", result);
    require_finite(point.reference_progress_m,
                   "corridor reference progress must be finite", result);
    require_finite(point.mean_lateral_error_m,
                   "corridor mean lateral error must be finite", result);
    require_finite(point.lateral_stddev_m,
                   "corridor standard deviation must be finite", result);
    require_finite(point.upper_bound_m, "corridor upper bound must be finite",
                   result);
  }
  require_finite(value.corridor_result.total_marginal_length_m,
                 "corridor marginal length must be finite", result);
  require_finite(value.corridor_result.total_violation_length_m,
                 "corridor violation length must be finite", result);
  require_finite(value.corridor_result.maximum_marginal_length_m,
                 "corridor marginal limit must be finite", result);
  require_finite(value.corridor_result.epsilon_point,
                 "corridor epsilon_point must be finite", result);
  for (const double error_m :
       value.corridor_result.interval_bound_certificate.upper_bound_error_m) {
    require_finite(error_m,
                   "corridor interval upper-bound error must be finite",
                   result);
  }
  require_finite(value.cable_laying_result.maximum_absolute_curvature_per_m,
                 "cable maximum curvature must be finite", result);
  if (value.cable_laying_result.maximum_absolute_curvature_position_m
          .has_value()) {
    require_finite(
        value.cable_laying_result.maximum_absolute_curvature_position_m->x_m,
        "cable maximum curvature position x must be finite", result);
    require_finite(
        value.cable_laying_result.maximum_absolute_curvature_position_m->y_m,
        "cable maximum curvature position y must be finite", result);
  }
  require_finite(value.cable_laying_result.maximum_support_proxy_range_m,
                 "support proxy range must be finite", result);
  if (value.cable_laying_result.maximum_support_proxy_position_m.has_value()) {
    require_finite(
        value.cable_laying_result.maximum_support_proxy_position_m->x_m,
        "support proxy maximum position x must be finite", result);
    require_finite(
        value.cable_laying_result.maximum_support_proxy_position_m->y_m,
        "support proxy maximum position y must be finite", result);
  }
  require_finite(value.cable_laying_result.terminal_support_window_length_m,
                 "support window length must be finite", result);
  require_finite(value.cable_laying_result.soft_cost,
                 "cable laying soft cost must be finite", result);
  for (const CableLayingFailureSegment& segment :
       value.cable_laying_result.failure_segments) {
    require_finite(segment.start_arc_length_m,
                   "cable failure segment start must be finite", result);
    require_finite(segment.end_arc_length_m,
                   "cable failure segment end must be finite", result);
    require_finite(segment.representative_position_m.x_m,
                   "cable failure position x must be finite", result);
    require_finite(segment.representative_position_m.y_m,
                   "cable failure position y must be finite", result);
  }
  validate_memory(value.cable_laying_result.terminal_memory, result);
  validate_finite_error_budget_fields(value.error_budget, result);
}

void validate_all_enum_fields(const PlanningResult& value,
                              ValidationResult& result) {
  if (!is_known_enum(value.state)) {
    result.issues.emplace_back("planning state is unknown");
  }
  if (!is_known_enum(value.terminal_cable_state.kind)) {
    result.issues.emplace_back("terminal cable state kind is unknown");
  }
  if (!is_known_enum(value.cable_model_validity)) {
    result.issues.emplace_back("cable model validity is unknown");
  }
  if (!is_known_enum(value.corridor_result.validity)) {
    result.issues.emplace_back("corridor validity is unknown");
  }
  for (const CableCorridorPointResult& point : value.corridor_result.points) {
    if (!is_known_enum(point.status)) {
      result.issues.emplace_back("corridor point status is unknown");
    }
    if (!is_known_enum(point.basis)) {
      result.issues.emplace_back("corridor point basis is unknown");
    }
  }
  for (const CableLayingFailure failure :
       value.cable_laying_result.failure_reasons) {
    if (!is_known_enum(failure)) {
      result.issues.emplace_back("cable laying failure is unknown");
    }
  }
  for (const CableLayingFailureSegment& segment :
       value.cable_laying_result.failure_segments) {
    if (!is_known_enum(segment.reason)) {
      result.issues.emplace_back("cable laying failure segment is unknown");
    }
  }
  if (!is_known_enum(value.error_budget.sensor_mode) ||
      !is_known_enum(value.sensor_mode) ||
      !is_known_enum(value.diagnostics.dependencies.sensor_mode)) {
    result.issues.emplace_back("planning result sensor mode is unknown");
  }
  for (const DiagnosticEntry& entry : value.diagnostics.entries) {
    if (!is_known_enum(entry.severity)) {
      result.issues.emplace_back("diagnostic severity is unknown");
    }
  }
}

bool dependency_versions_match(const PlanningResult& result,
                               const PlanningDependencyVersions& versions) {
  return versions.map_version.map_id == result.map_version.map_id &&
         versions.map_version.sequence_number ==
             result.map_version.sequence_number &&
         versions.map_version.timestamp.nanoseconds ==
             result.map_version.timestamp.nanoseconds &&
         versions.map_version.coordinate_frame ==
             result.map_version.coordinate_frame &&
         versions.reference_line_version == result.reference_line_version &&
         versions.robot_operating_area_version ==
             result.robot_operating_area_version &&
         versions.terrain_gradient_policy_version ==
             result.terrain_gradient_policy_version &&
         versions.corridor_risk_policy_version ==
             result.corridor_risk_policy_version &&
         versions.cable_model_version == result.cable_model_version &&
         versions.uncertainty_envelope_version ==
             result.uncertainty_envelope_version &&
         versions.uncertainty_envelope_generator_version ==
             result.uncertainty_envelope_generator_version &&
         versions.execution_operating_envelope_version ==
             result.execution_operating_envelope_version &&
         versions.execution_profile_version == result.execution_profile_version &&
         versions.sensor_mode == result.sensor_mode &&
         versions.operating_domain_id == result.operating_domain_id &&
         versions.cable_corridor_version == result.cable_corridor_version;
}

bool has_path_payload(const GeometricPath& path) {
  return !path.points.empty() || path.metadata.path_version != 0 ||
         !path.metadata.coordinate_frame.empty() ||
         path.metadata.reference_line_version != 0 ||
         !path.metadata.interpolation_rule.empty();
}

bool has_timed_path_payload(const TimedPath& path) {
  const SpeedPayoutLimits& limits =
      path.execution_profile.approved_tracking_limits;
  return has_path_payload(path.geometry) ||
         path.execution_profile.version != 0 ||
         path.execution_profile.operating_envelope_version != 0 ||
         !path.execution_profile.interpolation_rule.empty() ||
         path.execution_profile.stopping_point_arc_length_m.has_value() ||
         !path.execution_profile.samples.empty() ||
         limits.ground_speed.minimum_mps != 0.0 ||
         limits.ground_speed.maximum_mps != 0.0 ||
         limits.ground_acceleration.minimum_mps2 != 0.0 ||
         limits.ground_acceleration.maximum_mps2 != 0.0 ||
         limits.maximum_lateral_acceleration_mps2 != 0.0 ||
         limits.payout_speed.minimum_mps != 0.0 ||
         limits.payout_speed.maximum_mps != 0.0 ||
         limits.payout_acceleration.minimum_mps2 != 0.0 ||
         limits.payout_acceleration.maximum_mps2 != 0.0 ||
         limits.maximum_payout_tracking_error_mps != 0.0 ||
         limits.tension.minimum_n != 0.0 || limits.tension.maximum_n != 0.0 ||
         limits.maximum_stopping_distance_m != 0.0;
}

bool has_cable_state_payload(const CableState& state) {
  return state.sequence_number != 0 || state.lag_angle_rad != 0.0 ||
         state.lag_angle_variance_rad2.has_value() ||
         state.laying_memory.retained_arc_length_m != 0.0 ||
         state.laying_memory.canonical_signature != 0 ||
         !state.laying_memory.previous_distinct_touchdown_points_m.empty() ||
         !state.laying_memory.trailing_support_samples.empty();
}

bool has_error_budget_payload(const ErrorBudget& budget) {
  const Covariance2dM2& robot = budget.robot_position_covariance_m2;
  return robot.xx_m2 != 0.0 || robot.xy_m2 != 0.0 || robot.yx_m2 != 0.0 ||
         robot.yy_m2 != 0.0 ||
         !budget.touchdown_position_covariance_m2.empty() ||
         budget.epsilon_robot.has_value() ||
         budget.epsilon_terrain_gradient_local.has_value() ||
         budget.epsilon_point.has_value() || budget.epsilon_path.has_value() ||
         !budget.reference_is_deterministic ||
         budget.path_joint_risk_implemented ||
         budget.terrain_gradient_path_joint_risk_implemented ||
         !budget.calibration_dataset_id.empty() ||
         !budget.terrain_gradient_calibration_dataset_id.empty() ||
         budget.terrain_gradient_policy_version != 0 ||
         budget.corridor_risk_policy_version != 0 ||
         budget.cable_model_version != 0 ||
         budget.uncertainty_envelope_version != 0 ||
         budget.uncertainty_envelope_generator_version != 0 ||
         budget.execution_operating_envelope_version != 0 ||
         !budget.operating_domain_id.empty() ||
         budget.sensor_mode != SensorHealthMode::nominal ||
         budget.covariance_envelope_audit_passed;
}

ValidationResult validate(const PlanningResult& result_value) {
  ValidationResult result{true, {}};
  validate_all_serialized_numeric_fields(result_value, result);
  validate_all_enum_fields(result_value, result);
  if (result_value.sequence_number == 0) {
    result.issues.emplace_back("planning result sequence_number must be nonzero");
  }
  if (result_value.timestamp.nanoseconds < 0) {
    result.issues.emplace_back("planning result timestamp must be monotonic");
  }
  if (result_value.cable_corridor_version == 0U) {
    result.issues.emplace_back("planning result cable corridor version must be nonzero");
  }
  if (result_value.validity_duration.nanoseconds <= 0) {
    result.issues.emplace_back("planning result validity duration must be positive");
  }
  // These members are serialized for every result, including failures. A
  // default -1 therefore cannot mean "not applicable".
  if (result_value.terminal_cable_state.timestamp.nanoseconds < 0) {
    result.issues.emplace_back(
        "terminal cable state timestamp must be monotonic for every planning result");
  }
  if (result_value.corridor_result.evaluation_timestamp.nanoseconds < 0) {
    result.issues.emplace_back(
        "corridor evaluation timestamp must be monotonic for every planning result");
  }

  const bool publishes_trajectory =
      result_value.state == PlanningState::success ||
      result_value.state == PlanningState::path_valid;
  if (publishes_trajectory || has_timed_path_payload(result_value.robot_trajectory)) {
    append_issues(validate(result_value.robot_trajectory), result);
  }
  if (publishes_trajectory || has_path_payload(result_value.cable_path)) {
    append_issues(validate(result_value.cable_path), result);
  }
  if (publishes_trajectory ||
      has_cable_state_payload(result_value.terminal_cable_state)) {
    append_issues(validate(result_value.terminal_cable_state), result);
  }
  if (publishes_trajectory || has_error_budget_payload(result_value.error_budget)) {
    append_issues(validate(result_value.error_budget), result);
  }
  if (publishes_trajectory) {
    if (result_value.cable_model_validity != CableModelValidity::valid) {
      result.issues.emplace_back("published trajectory requires a valid cable model");
    }
    if (result_value.corridor_result.validity !=
        CorridorEvaluationValidity::valid) {
      result.issues.emplace_back("published trajectory requires valid corridor risk");
    }
    if (!result_value.corridor_result.hard_feasible) {
      result.issues.emplace_back(
          "published trajectory requires corridor hard feasibility");
    }
    if (!result_value.cable_laying_result.valid ||
        !result_value.cable_laying_result.hard_feasible) {
      result.issues.emplace_back("published trajectory requires cable laying feasibility");
    }
    if (result_value.error_budget.touchdown_position_covariance_m2.size() !=
        result_value.cable_path.points.size()) {
      result.issues.emplace_back(
          "touchdown covariance must align pointwise with cable path");
    }
    if (!memories_equal(result_value.terminal_cable_state.laying_memory,
                        result_value.cable_laying_result.terminal_memory)) {
      result.issues.emplace_back(
          "terminal cable memory must match cable laying evaluation");
    }
  }

  std::uint64_t marginal_count = 0;
  std::uint64_t violation_count = 0;
  for (const CableCorridorPointResult& point :
       result_value.corridor_result.points) {
    require_finite(point.mean_lateral_error_m,
                   "corridor mean lateral error must be finite", result);
    require_nonnegative_finite(point.lateral_stddev_m,
                               "corridor standard deviation must be nonnegative",
                               result);
    require_nonnegative_finite(point.upper_bound_m,
                               "corridor upper bound must be nonnegative", result);
    if (point.upper_bound_m < std::abs(point.mean_lateral_error_m)) {
      result.issues.emplace_back("corridor upper bound is below the mean error");
    }
    if ((point.status == CableValidationStatus::pass &&
         point.basis != CableCorridorPointBasis::below_nominal_bound) ||
        (point.status == CableValidationStatus::marginal &&
         point.basis != CableCorridorPointBasis::within_absolute_bound) ||
        (point.status == CableValidationStatus::violation &&
         point.basis !=
             CableCorridorPointBasis::at_or_above_absolute_bound)) {
      result.issues.emplace_back(
          "corridor point basis does not match its classification");
    }
    if (point.status == CableValidationStatus::marginal) {
      ++marginal_count;
    } else if (point.status == CableValidationStatus::violation) {
      ++violation_count;
    }
  }
  if (marginal_count != result_value.corridor_result.marginal_count ||
      violation_count != result_value.corridor_result.violation_count) {
    result.issues.emplace_back("corridor summary counts do not match point statuses");
  }
  require_nonnegative_finite(
      result_value.corridor_result.total_marginal_length_m,
      "corridor marginal length must be nonnegative", result);
  require_nonnegative_finite(
      result_value.corridor_result.total_violation_length_m,
      "corridor violation length must be nonnegative", result);
  require_nonnegative_finite(
      result_value.corridor_result.maximum_marginal_length_m,
      "corridor marginal limit must be nonnegative", result);
  const bool marginal_limit_exceeded =
      result_value.corridor_result.total_marginal_length_m >
      result_value.corridor_result.maximum_marginal_length_m;
  if (marginal_limit_exceeded !=
      result_value.corridor_result.marginal_length_limit_exceeded) {
    result.issues.emplace_back(
        "corridor marginal-limit conclusion is inconsistent");
  }
  if (result_value.corridor_result.validity ==
      CorridorEvaluationValidity::valid) {
    const bool expected_feasible =
        violation_count == 0U &&
        result_value.corridor_result.total_violation_length_m == 0.0 &&
        !marginal_limit_exceeded;
    if (result_value.corridor_result.hard_feasible != expected_feasible) {
      result.issues.emplace_back(
          "corridor hard-feasibility conclusion is inconsistent");
    }
    if (!(result_value.corridor_result.epsilon_point > 0.0 &&
          result_value.corridor_result.epsilon_point < 1.0) ||
        result_value.corridor_result.corridor_risk_policy_version == 0U ||
        result_value.corridor_result.reference_line_version == 0U ||
        result_value.corridor_result.interval_bound_certificate.version == 0U ||
        result_value.corridor_result.interval_bound_certificate
                .upper_bound_error_m.size() !=
            (result_value.corridor_result.points.empty()
                 ? 0U
                 : result_value.corridor_result.points.size() - 1U) ||
        std::any_of(
            result_value.corridor_result.interval_bound_certificate
                .upper_bound_error_m.begin(),
            result_value.corridor_result.interval_bound_certificate
                .upper_bound_error_m.end(),
            [](const double error_m) { return error_m < 0.0; }) ||
        result_value.corridor_result.evaluation_timestamp.nanoseconds < 0 ||
        result_value.corridor_result.operating_domain_id.empty() ||
        result_value.corridor_result
            .residual_distribution_calibration_dataset_id.empty() ||
        !result_value.corridor_result.reference_is_deterministic ||
        !result_value.corridor_result
             .covariance_includes_coordinate_transform_error ||
        result_value.corridor_result.path_joint_risk_implemented ||
        result_value.corridor_result.risk_semantics !=
            "POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE") {
      result.issues.emplace_back(
          "valid corridor result has incomplete or overstated risk evidence");
    }
  } else if (result_value.corridor_result.hard_feasible) {
    result.issues.emplace_back(
        "invalid corridor evaluation cannot be hard feasible");
  }
  if (publishes_trajectory &&
      (violation_count != 0 ||
       result_value.corridor_result.total_violation_length_m > 0.0)) {
    result.issues.emplace_back("published trajectory cannot contain corridor violations");
  }

  require_nonnegative_finite(
      result_value.cable_laying_result.maximum_absolute_curvature_per_m,
      "cable maximum curvature must be nonnegative", result);
  require_nonnegative_finite(
      result_value.cable_laying_result.maximum_support_proxy_range_m,
      "cable support proxy range must be nonnegative", result);
  require_nonnegative_finite(
      result_value.cable_laying_result.terminal_support_window_length_m,
      "cable support window length must be nonnegative", result);
  require_nonnegative_finite(result_value.cable_laying_result.soft_cost,
                             "cable laying soft cost must be nonnegative", result);
  if (result_value.cable_laying_result.limits_version == 0U ||
      result_value.cable_laying_result.terrain_map_sequence == 0U ||
      result_value.cable_laying_result.terrain_analysis_config_version == 0U ||
      result_value.cable_laying_result.operating_domain_id.empty() ||
      result_value.cable_laying_result.risk_semantics !=
          "CONSERVATIVE_SUPPORT_PROXY:NO_FLEXIBLE_CABLE_DYNAMICS_GUARANTEE") {
    result.issues.emplace_back(
        "valid cable laying result lacks auditable dependencies or risk semantics");
  }
  for (const CableLayingFailureSegment& segment :
       result_value.cable_laying_result.failure_segments) {
    require_nonnegative_finite(segment.start_arc_length_m,
                               "cable failure start must be nonnegative",
                               result);
    require_nonnegative_finite(segment.end_arc_length_m,
                               "cable failure end must be nonnegative",
                               result);
    if (segment.start_arc_length_m > segment.end_arc_length_m) {
      result.issues.emplace_back(
          "cable failure segment start exceeds its end");
    }
    if (segment.reason == CableLayingFailure::none ||
        std::find(result_value.cable_laying_result.failure_reasons.begin(),
                  result_value.cable_laying_result.failure_reasons.end(),
                  segment.reason) ==
            result_value.cable_laying_result.failure_reasons.end()) {
      result.issues.emplace_back(
          "cable failure segment does not match a failure reason");
    }
  }
  if (result_value.cable_laying_result.valid &&
      result_value.cable_laying_result.hard_feasible) {
    if (result_value.cable_laying_result.failure_reasons.size() != 1 ||
        result_value.cable_laying_result.failure_reasons.front() !=
            CableLayingFailure::none) {
      result.issues.emplace_back("feasible cable laying result must have failure NONE");
    }
    if (!result_value.cable_laying_result.failure_segments.empty()) {
      result.issues.emplace_back(
          "feasible cable laying result cannot have failure segments");
    }
  } else if (result_value.cable_laying_result.valid &&
             result_value.cable_laying_result.failure_segments.empty()) {
    result.issues.emplace_back(
        "infeasible cable laying result must identify a failure segment");
  }
  if (result_value.cable_laying_result.valid &&
      !result_value.cable_laying_result.hard_feasible) {
    for (const CableLayingFailure failure :
         result_value.cable_laying_result.failure_reasons) {
      if (failure == CableLayingFailure::none) {
        result.issues.emplace_back(
            "infeasible cable laying result cannot contain failure NONE");
        continue;
      }
      const bool has_segment = std::any_of(
          result_value.cable_laying_result.failure_segments.begin(),
          result_value.cable_laying_result.failure_segments.end(),
          [failure](const CableLayingFailureSegment& segment) {
            return segment.reason == failure;
          });
      if (!has_segment) {
        result.issues.emplace_back(
            "cable laying failure reason lacks an audit segment");
      }
    }
  }
  if (!result_value.cable_laying_result.valid &&
      result_value.cable_laying_result.hard_feasible) {
    result.issues.emplace_back(
        "invalid cable laying evaluation cannot be hard feasible");
  }

  if (result_value.map_version.map_id.empty() ||
      result_value.map_version.sequence_number == 0 ||
      result_value.map_version.timestamp.nanoseconds < 0 ||
      result_value.map_version.coordinate_frame.empty()) {
    result.issues.emplace_back("map version metadata is incomplete");
  }
  if (result_value.cable_laying_result.valid &&
      (result_value.cable_laying_result.terrain_map_sequence !=
           result_value.map_version.sequence_number ||
       result_value.cable_laying_result.operating_domain_id !=
           result_value.operating_domain_id)) {
    result.issues.emplace_back(
        "cable laying evidence does not match the planning context");
  }
  if (result_value.reference_line_version == 0 ||
      result_value.robot_operating_area_version == 0 ||
      result_value.terrain_gradient_policy_version == 0 ||
      result_value.corridor_risk_policy_version == 0 ||
      result_value.cable_model_version == 0 ||
      result_value.uncertainty_envelope_version == 0 ||
      result_value.uncertainty_envelope_generator_version == 0 ||
      result_value.execution_operating_envelope_version == 0 ||
      result_value.execution_profile_version == 0) {
    result.issues.emplace_back("planning result dependency versions must be nonzero");
  }
  if (publishes_trajectory &&
      (result_value.robot_trajectory.geometry.metadata.reference_line_version !=
           result_value.reference_line_version ||
       result_value.cable_path.metadata.reference_line_version !=
           result_value.reference_line_version ||
       result_value.robot_trajectory.execution_profile.version !=
           result_value.execution_profile_version ||
       result_value.robot_trajectory.execution_profile.operating_envelope_version !=
           result_value.execution_operating_envelope_version ||
       result_value.error_budget.terrain_gradient_policy_version !=
           result_value.terrain_gradient_policy_version ||
       result_value.error_budget.corridor_risk_policy_version !=
           result_value.corridor_risk_policy_version ||
       result_value.error_budget.cable_model_version !=
           result_value.cable_model_version ||
       result_value.error_budget.uncertainty_envelope_version !=
           result_value.uncertainty_envelope_version ||
       result_value.error_budget.uncertainty_envelope_generator_version !=
           result_value.uncertainty_envelope_generator_version ||
       result_value.error_budget.execution_operating_envelope_version !=
           result_value.execution_operating_envelope_version ||
       result_value.error_budget.sensor_mode != result_value.sensor_mode ||
       result_value.error_budget.operating_domain_id !=
           result_value.operating_domain_id)) {
    result.issues.emplace_back("planning result evidence versions do not match");
  }
  if (publishes_trajectory &&
      (result_value.corridor_result.points.size() !=
           result_value.cable_path.points.size() ||
       result_value.corridor_result.reference_line_version !=
           result_value.reference_line_version ||
       result_value.corridor_result.corridor_risk_policy_version !=
           result_value.corridor_risk_policy_version ||
       result_value.corridor_result.operating_domain_id !=
           result_value.operating_domain_id ||
       !result_value.corridor_result.covariance_envelope_audit_performed ||
       !result_value.error_budget.epsilon_point.has_value() ||
       result_value.corridor_result.epsilon_point !=
           *result_value.error_budget.epsilon_point ||
       result_value.corridor_result
               .residual_distribution_calibration_dataset_id !=
           result_value.error_budget.calibration_dataset_id ||
       result_value.corridor_result.reference_is_deterministic !=
           result_value.error_budget.reference_is_deterministic ||
       result_value.corridor_result.path_joint_risk_implemented !=
           result_value.error_budget.path_joint_risk_implemented ||
       result_value.corridor_result.covariance_envelope_audit_performed !=
           result_value.error_budget.covariance_envelope_audit_passed)) {
    result.issues.emplace_back(
        "corridor result evidence does not match planning context");
  }

  const Diagnostics& diagnostics = result_value.diagnostics;
  if (diagnostics.schema_version.empty() || diagnostics.input_version.empty() ||
      diagnostics.unit_system.empty() ||
      diagnostics.operating_domain_id.empty() || diagnostics.risk_semantics.empty()) {
    result.issues.emplace_back("planning diagnostics metadata is incomplete");
  }
  if (diagnostics.operating_domain_id != result_value.operating_domain_id) {
    result.issues.emplace_back("diagnostics operating domain does not match result");
  }
  if (!dependency_versions_match(result_value, diagnostics.dependencies)) {
    result.issues.emplace_back(
        "diagnostics dependency versions do not match planning result");
  }
  if (diagnostics.risk_semantics !=
      "POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE") {
    result.issues.emplace_back("diagnostics overstate first-version risk semantics");
  }
  for (const DiagnosticEntry& entry : diagnostics.entries) {
    if (entry.code.empty() || entry.stage.empty() || entry.message.empty() ||
        entry.timestamp.nanoseconds < 0) {
      result.issues.emplace_back("diagnostic entry is incomplete");
    }
  }
  result.valid = result.issues.empty();
  return result;
}

std::string serialize_planning_result(const PlanningResult& result) {
  const ValidationResult validation = validate(result);
  if (!validation.valid) {
    throw std::invalid_argument("cannot serialize an invalid planning result: " +
                                validation.issues.front());
  }

  ContractWriter writer;
  writer.scalar("UP_RESULT");
  writer.scalar(8);
  writer.scalar(result.sequence_number);
  writer.scalar(result.timestamp.nanoseconds);
  writer.scalar(result.validity_duration.nanoseconds);
  write_enum(writer, result.state);
  write_timed_path(writer, result.robot_trajectory);
  write_path(writer, result.cable_path);
  write_cable_state(writer, result.terminal_cable_state);
  write_enum(writer, result.cable_model_validity);
  write_corridor(writer, result.corridor_result);
  write_laying(writer, result.cable_laying_result);
  write_error_budget(writer, result.error_budget);
  writer.text(result.map_version.map_id);
  writer.scalar(result.map_version.sequence_number);
  writer.scalar(result.map_version.timestamp.nanoseconds);
  writer.text(result.map_version.coordinate_frame);
  writer.scalar(result.reference_line_version);
  writer.scalar(result.robot_operating_area_version);
  writer.scalar(result.terrain_gradient_policy_version);
  writer.scalar(result.corridor_risk_policy_version);
  writer.scalar(result.cable_model_version);
  writer.scalar(result.uncertainty_envelope_version);
  writer.scalar(result.uncertainty_envelope_generator_version);
  writer.scalar(result.execution_operating_envelope_version);
  writer.scalar(result.execution_profile_version);
  write_enum(writer, result.sensor_mode);
  writer.text(result.operating_domain_id);
  writer.scalar(result.cable_corridor_version);
  write_diagnostics(writer, result.diagnostics);
  return writer.finish();
}

PlanningResult deserialize_planning_result(const std::string& serialized) {
  ContractReader reader(serialized);
  if (reader.scalar<std::string>() != "UP_RESULT") {
    throw std::invalid_argument("unknown planning result schema");
  }
  const int schema_version = reader.scalar<int>();
  if (schema_version != 8) {
    throw std::invalid_argument("unknown planning result schema");
  }

  PlanningResult result;
  result.sequence_number = reader.scalar<std::uint64_t>();
  result.timestamp.nanoseconds = reader.scalar<std::int64_t>();
  result.validity_duration.nanoseconds = reader.scalar<std::int64_t>();
  result.state = reader.enumeration<PlanningState>();
  result.robot_trajectory = read_timed_path(reader);
  result.cable_path = read_path(reader);
  result.terminal_cable_state = read_cable_state(reader);
  result.cable_model_validity = reader.enumeration<CableModelValidity>();
  result.corridor_result = read_corridor(reader);
  result.cable_laying_result = read_laying(reader);
  result.error_budget = read_error_budget(reader);
  result.map_version.map_id = reader.text();
  result.map_version.sequence_number = reader.scalar<std::uint64_t>();
  result.map_version.timestamp.nanoseconds = reader.scalar<std::int64_t>();
  result.map_version.coordinate_frame = reader.text();
  result.reference_line_version = reader.scalar<std::uint32_t>();
  result.robot_operating_area_version = reader.scalar<std::uint32_t>();
  result.terrain_gradient_policy_version = reader.scalar<std::uint64_t>();
  result.corridor_risk_policy_version = reader.scalar<std::uint64_t>();
  result.cable_model_version = reader.scalar<std::uint64_t>();
  result.uncertainty_envelope_version = reader.scalar<std::uint64_t>();
  result.uncertainty_envelope_generator_version =
      reader.scalar<std::uint64_t>();
  result.execution_operating_envelope_version = reader.scalar<std::uint64_t>();
  result.execution_profile_version = reader.scalar<std::uint64_t>();
  result.sensor_mode = reader.enumeration<SensorHealthMode>();
  result.operating_domain_id = reader.text();
  result.cable_corridor_version = reader.scalar<std::uint32_t>();
  result.diagnostics = read_diagnostics(reader);
  reader.require_end();

  const ValidationResult validation = validate(result);
  if (!validation.valid) {
    throw std::invalid_argument("deserialized planning result is invalid: " +
                                validation.issues.front());
  }
  return result;
}

}  // namespace underwater_planner::core
