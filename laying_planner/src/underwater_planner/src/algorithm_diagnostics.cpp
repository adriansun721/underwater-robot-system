#include "underwater_planner/core/algorithm_diagnostics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <Psapi.h>
#else
#include <sys/resource.h>
#include <unistd.h>
#endif

namespace underwater_planner::core {
namespace {

constexpr const char* kRiskSemantics =
    "POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE";

bool finite(const double value) { return std::isfinite(value); }

std::uint64_t fingerprint(const std::string& value) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char byte : value) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::string serialize_runtime_parameters(
    const AlgorithmRuntimeParameterSnapshot& parameters) {
  const std::array<std::string, 5> components{
      serialize_parameter_config(parameters.profile),
      serialize_terrain_analysis_config(parameters.terrain_analysis),
      serialize_hybrid_astar_search_parameters(parameters.search),
      serialize_smoothing_limits(parameters.smoothing),
      serialize_trajectory_parameterization_limits(
          parameters.parameterization)};
  std::ostringstream stream;
  stream << parameters.schema_version.size() << ':'
         << parameters.schema_version << '\n';
  for (const std::string& component : components) {
    stream << component.size() << ':' << component << '\n';
  }
  return stream.str();
}
double maximum_residual(const PathConstraintResiduals& residuals) {
  const std::array<double, 9> values{
      residuals.maximum_dynamics_residual,
      residuals.maximum_curvature_audit_residual,
      residuals.maximum_curvature_rate_residual,
      residuals.start_position_residual_m,
      residuals.start_heading_residual_rad,
      residuals.start_curvature_residual_per_m,
      residuals.goal_position_residual_m,
      residuals.goal_heading_residual_rad,
      residuals.goal_curvature_residual_per_m};
  double maximum = 0.0;
  for (const double value : values) maximum = std::max(maximum, std::abs(value));
  return maximum;
}

template <typename Projection>
std::pair<double, double> sample_range(const ExecutionProfile& profile,
                                       Projection projection) {
  double minimum = std::numeric_limits<double>::infinity();
  double maximum = -std::numeric_limits<double>::infinity();
  for (const ExecutionSample& sample : profile.samples) {
    const double value = projection(sample);
    minimum = std::min(minimum, value);
    maximum = std::max(maximum, value);
  }
  return {minimum, maximum};
}

void add_margin(std::vector<ConstraintMarginSample>& margins,
                const char* constraint, const double margin,
                const char* unit) {
  margins.push_back({constraint, margin, unit});
}

ParameterizationExperimentSample parameterization_sample(
    const ParameterizationResult& result) {
  ParameterizationExperimentSample sample;
  sample.limits_version = result.diagnostics.limits_version;
  sample.minimum_constraint_margin =
      std::numeric_limits<double>::infinity();
  if (!result.trajectory.has_value()) {
    sample.minimum_constraint_margin = 0.0;
    return sample;
  }

  const ExecutionProfile& profile = result.trajectory->execution_profile;
  sample.execution_profile_version = profile.version;
  sample.execution_operating_envelope_version =
      profile.operating_envelope_version;
  if (profile.samples.empty()) {
    sample.minimum_constraint_margin = 0.0;
    return sample;
  }

  const SpeedPayoutLimits& limits = profile.approved_tracking_limits;
  const auto ground_speed = sample_range(
      profile, [](const ExecutionSample& value) { return value.ground_speed_mps; });
  const auto ground_acceleration = sample_range(
      profile,
      [](const ExecutionSample& value) { return value.ground_acceleration_mps2; });
  const auto payout_speed = sample_range(
      profile, [](const ExecutionSample& value) { return value.payout_speed_mps; });
  const auto payout_acceleration = sample_range(
      profile,
      [](const ExecutionSample& value) { return value.payout_acceleration_mps2; });
  const auto tension = sample_range(
      profile,
      [](const ExecutionSample& value) { return value.tension_setpoint_n; });

  add_margin(sample.constraint_margins, "ground_speed_minimum",
             ground_speed.first - limits.ground_speed.minimum_mps, "m/s");
  add_margin(sample.constraint_margins, "ground_speed_maximum",
             limits.ground_speed.maximum_mps - ground_speed.second, "m/s");
  add_margin(sample.constraint_margins, "ground_acceleration_minimum",
             ground_acceleration.first -
                 limits.ground_acceleration.minimum_mps2,
             "m/s^2");
  add_margin(sample.constraint_margins, "ground_acceleration_maximum",
             limits.ground_acceleration.maximum_mps2 -
                 ground_acceleration.second,
             "m/s^2");
  add_margin(sample.constraint_margins, "payout_speed_minimum",
             payout_speed.first - limits.payout_speed.minimum_mps, "m/s");
  add_margin(sample.constraint_margins, "payout_speed_maximum",
             limits.payout_speed.maximum_mps - payout_speed.second, "m/s");
  add_margin(sample.constraint_margins, "payout_acceleration_minimum",
             payout_acceleration.first -
                 limits.payout_acceleration.minimum_mps2,
             "m/s^2");
  add_margin(sample.constraint_margins, "payout_acceleration_maximum",
             limits.payout_acceleration.maximum_mps2 -
                 payout_acceleration.second,
             "m/s^2");
  add_margin(sample.constraint_margins, "tension_minimum",
             tension.first - limits.tension.minimum_n, "N");
  add_margin(sample.constraint_margins, "tension_maximum",
             limits.tension.maximum_n - tension.second, "N");
  add_margin(sample.constraint_margins, "lateral_acceleration_maximum",
             limits.maximum_lateral_acceleration_mps2 -
                 result.diagnostics.maximum_lateral_acceleration_mps2,
             "m/s^2");
  add_margin(sample.constraint_margins, "stopping_distance",
             result.diagnostics.available_stopping_distance_m -
                 result.diagnostics.required_stopping_distance_m,
             "m");
  for (const ConstraintMarginSample& margin : sample.constraint_margins) {
    sample.minimum_constraint_margin =
        std::min(sample.minimum_constraint_margin, margin.margin);
  }
  return sample;
}

struct FinalPlanningEvidence {
  std::optional<GeometricPath> path;
  Diagnostics diagnostics;
  std::optional<ErrorBudget> error_budget;
};

FinalPlanningEvidence select_final_evidence(
    const PlanningCycleResult& result) {
  FinalPlanningEvidence evidence;
  if (result.publication.has_value()) {
    const PlanningResult& plan = result.publication->plan.value();
    evidence.path = plan.robot_trajectory.geometry;
    evidence.diagnostics = plan.diagnostics;
    evidence.error_budget = plan.error_budget;
    return evidence;
  }
  if (result.artifacts.candidate) {
    evidence.path = result.artifacts.candidate->robot_trajectory.geometry;
    evidence.diagnostics = result.artifacts.candidate->diagnostics;
    evidence.error_budget = result.artifacts.candidate->error_budget;
    return evidence;
  }
  if (result.artifacts.candidate_metadata) {
    evidence.diagnostics = result.artifacts.candidate_metadata->diagnostics;
    evidence.error_budget = result.artifacts.candidate_metadata->error_budget;
  }
  if (result.artifacts.smoothing && result.artifacts.smoothing->path) {
    evidence.path = result.artifacts.smoothing->path;
  } else if (result.artifacts.search &&
      !result.artifacts.search->robot_path.points.empty()) {
    evidence.path = result.artifacts.search->robot_path;
  }
  return evidence;
}

bool same_path(const std::optional<GeometricPath>& left,
               const std::optional<GeometricPath>& right) {
  if (left.has_value() != right.has_value()) return false;
  if (!left) return true;
  if (left->metadata.path_version != right->metadata.path_version ||
      left->metadata.coordinate_frame != right->metadata.coordinate_frame ||
      left->metadata.reference_line_version !=
          right->metadata.reference_line_version ||
      left->metadata.interpolation_rule != right->metadata.interpolation_rule ||
      left->points.size() != right->points.size()) {
    return false;
  }
  for (std::size_t index = 0; index < left->points.size(); ++index) {
    const PathPoint& a = left->points[index];
    const PathPoint& b = right->points[index];
    if (a.arc_length_m != b.arc_length_m || a.x_m != b.x_m ||
        a.y_m != b.y_m || a.heading_rad != b.heading_rad ||
        a.curvature_per_m != b.curvature_per_m) {
      return false;
    }
  }
  return true;
}

bool same_diagnostics(const Diagnostics& left, const Diagnostics& right) {
  if (left.schema_version != right.schema_version ||
      left.random_seed != right.random_seed ||
      left.input_version != right.input_version ||
      left.unit_system != right.unit_system ||
      left.operating_domain_id != right.operating_domain_id ||
      left.risk_semantics != right.risk_semantics ||
      left.dependencies != right.dependencies ||
      left.entries.size() != right.entries.size()) {
    return false;
  }
  for (std::size_t index = 0; index < left.entries.size(); ++index) {
    const DiagnosticEntry& a = left.entries[index];
    const DiagnosticEntry& b = right.entries[index];
    if (a.severity != b.severity || a.code != b.code || a.stage != b.stage ||
        a.message != b.message ||
        a.timestamp.nanoseconds != b.timestamp.nanoseconds) {
      return false;
    }
  }
  return true;
}

std::string serialize_replay_input(
    const std::optional<SynchronizedValidationInputs>& optional_input) {
  if (!optional_input) return "missing";
  const SynchronizedValidationInputs& input = *optional_input;
  std::ostringstream stream;
  stream << std::setprecision(17) << input.captured_at.nanoseconds << ' '
         << input.source_revision << ' ' << input.robot_state.pose.x_m << ' '
         << input.robot_state.pose.y_m << ' '
         << input.robot_state.pose.heading_rad << ' '
         << input.robot_state.pose.timestamp.nanoseconds << ' '
         << input.robot_state.ground_speed_mps << ' '
         << input.robot_state.curvature_per_m << ' '
         << input.robot_state.curvature_timestamp.nanoseconds << ' '
         << input.robot_state.sequence_number << '\n'
         << static_cast<int>(input.cable_state.kind) << ' '
         << input.cable_state.lag_angle_rad << ' '
         << input.cable_state.lag_angle_variance_rad2.value_or(-1.0) << ' '
         << input.cable_state.timestamp.nanoseconds << ' '
         << input.cable_state.sequence_number << ' '
         << input.cable_state.laying_memory.retained_arc_length_m << ' '
         << input.cable_state.laying_memory.canonical_signature << '\n';
  for (const Vector2m& point :
       input.cable_state.laying_memory.previous_distinct_touchdown_points_m) {
    stream << point.x_m << ',' << point.y_m << ';';
  }
  stream << '\n';
  for (const CableHistorySample& sample :
       input.cable_state.laying_memory.trailing_support_samples) {
    stream << sample.touchdown_arc_length_m << ','
           << sample.touchdown_position_m.x_m << ','
           << sample.touchdown_position_m.y_m << ';';
  }
  stream << '\n' << input.reference_progress.reference_line_version << ' '
         << input.reference_progress.arc_length_m << ' '
         << input.reference_progress.timestamp.nanoseconds << ' '
         << input.reference_progress.sequence_number << '\n'
         << input.cable_telemetry.payout_speed_mps << ' '
         << input.cable_telemetry.payout_acceleration_mps2 << ' '
         << input.cable_telemetry.tension_n << ' '
         << input.cable_telemetry.timestamp.nanoseconds << ' '
         << input.cable_telemetry.sequence_number << '\n'
         << input.execution_tracking_state.execution_profile_version << ' '
         << input.execution_tracking_state.execution_operating_envelope_version
         << ' ' << input.execution_tracking_state.tracked_arc_length_m << ' '
         << input.execution_tracking_state.timestamp.nanoseconds << ' '
         << input.execution_tracking_state.sequence_number << ' '
         << input.execution_tracking_state.ground_acceleration_mps2 << '\n';

  const MapSnapshot& map = input.planning_snapshot.map;
  stream << map.version.map_id << ' ' << map.version.sequence_number << ' '
         << map.version.timestamp.nanoseconds << ' '
         << map.version.coordinate_frame << ' ' << map.width << ' '
         << map.height << ' ' << map.resolution_m << ' ' << map.origin_x_m << ' '
         << map.origin_y_m << ' ' << map.derived_configuration_version << '\n';
  for (const MapCell& cell : map.cells) {
    stream << cell.elevation_m << ',' << cell.elevation_variance_m2 << ','
           << cell.confidence << ',' << cell.known << ','
           << cell.measurement_timestamp.nanoseconds << ',' << cell.obstacle
           << ',' << cell.obstacle_normal.has_value();
    if (cell.obstacle_normal) {
      stream << ',' << cell.obstacle_normal->x << ','
             << cell.obstacle_normal->y;
    }
    stream << ',' << cell.cable_forbidden << ';';
  }
  stream << '\n';
  for (const MapUpdateRegion& region : map.update_regions) {
    stream << region.min_x_m << ',' << region.min_y_m << ',' << region.max_x_m
           << ',' << region.max_y_m << ';';
  }
  const ReferenceLine& reference = input.planning_snapshot.reference_line;
  stream << '\n' << reference.version << ' ' << reference.coordinate_frame
         << '\n';
  for (const ReferencePoint& point : reference.points) {
    stream << point.arc_length_m << ',' << point.x_m << ',' << point.y_m << ','
           << point.tangent_x << ',' << point.tangent_y << ',' << point.normal_x
           << ',' << point.normal_y << ';';
  }
  const RobotOperatingArea& area =
      input.planning_snapshot.robot_operating_area;
  stream << '\n' << area.version << ' ' << area.id << '\n';
  for (const Point2d& point : area.polygon) {
    stream << point.x_m << ',' << point.y_m << ';';
  }
  const CableCorridor& corridor = input.planning_snapshot.cable_corridor;
  stream << '\n' << corridor.version << ' ' << corridor.id << '\n';
  for (const Point2d& point : corridor.polygon) {
    stream << point.x_m << ',' << point.y_m << ';';
  }
  stream << '\n' << input.dependencies.map_version.map_id << ' '
         << input.dependencies.map_version.sequence_number << ' '
         << input.dependencies.reference_line_version << ' '
         << input.dependencies.robot_operating_area_version << ' '
         << input.dependencies.cable_corridor_version << ' '
         << input.dependencies.terrain_gradient_policy_version << ' '
         << input.dependencies.corridor_risk_policy_version << ' '
         << input.dependencies.cable_model_version << ' '
         << input.dependencies.uncertainty_envelope_version << ' '
         << input.dependencies.uncertainty_envelope_generator_version << ' '
         << input.dependencies.execution_operating_envelope_version << ' '
         << input.dependencies.execution_profile_version << ' '
         << static_cast<int>(input.dependencies.sensor_mode) << ' '
         << input.dependencies.operating_domain_id;
  const TrackerUpdateReceipt& receipt = input.tracker_update_receipt;
  stream << '\n' << receipt.evidence_batch_sequence << ' '
         << receipt.executed_motion_sequence << ' '
         << receipt.touchdown_observation_sequence.value_or(0U) << ' '
         << receipt.touchdown_observation_sequence.has_value() << ' '
         << receipt.cable_telemetry_sequence << ' '
         << receipt.resulting_cable_state_sequence << ' '
         << receipt.resulting_reference_progress_sequence << ' '
         << static_cast<int>(input.cable_context_mode) << '\n';
  return stream.str();
}

std::string serialize_input_capture(
    const ValidationInputCaptureResult& capture) {
  std::ostringstream stream;
  stream << static_cast<int>(capture.status) << '\n';
  const std::string input = serialize_replay_input(capture.inputs);
  stream << input.size() << ':' << input << '\n' << capture.issues.size()
         << '\n';
  for (const CaptureIssue& issue : capture.issues) {
    stream << static_cast<int>(issue.code) << ':' << issue.field.size() << ':'
           << issue.field << ':' << issue.message.size() << ':'
           << issue.message << '\n';
  }
  return stream.str();
}

std::string serialize_input_captures(
    const std::vector<ValidationInputCaptureResult>& captures) {
  std::ostringstream stream;
  stream << captures.size() << '\n';
  for (const ValidationInputCaptureResult& capture : captures) {
    const std::string serialized = serialize_input_capture(capture);
    stream << serialized.size() << ':' << serialized << '\n';
  }
  return stream.str();
}

bool same_experiment_diagnostics(const AlgorithmExperimentRecord& left,
                                 const AlgorithmExperimentRecord& right) {
  const auto same_input = [](const auto& a, const auto& b) {
    return serialize_replay_input(a) == serialize_replay_input(b);
  };
  const auto same_search = [](const SearchExperimentSample& a,
                               const SearchExperimentSample& b) {
    return a.executed == b.executed &&
           a.fixed_bytes_per_search_label ==
               b.fixed_bytes_per_search_label &&
           a.peak_observed_bytes_per_search_label ==
               b.peak_observed_bytes_per_search_label &&
           a.peak_active_labels == b.peak_active_labels &&
           a.maximum_active_labels == b.maximum_active_labels &&
           a.labels_per_base_key_p50 == b.labels_per_base_key_p50 &&
           a.labels_per_base_key_p95 == b.labels_per_base_key_p95 &&
           a.labels_per_base_key_p99 == b.labels_per_base_key_p99 &&
           a.equivalent_discard_count == b.equivalent_discard_count &&
           a.equivalent_replacement_count ==
               b.equivalent_replacement_count &&
           a.signature_fallback_comparison_count ==
               b.signature_fallback_comparison_count &&
           a.label_budget_exhausted == b.label_budget_exhausted &&
           a.deadline_exceeded == b.deadline_exceeded;
  };
  const auto same_margins = [](const auto& a, const auto& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t index = 0; index < a.size(); ++index) {
      if (a[index].constraint != b[index].constraint ||
          a[index].margin != b[index].margin ||
          a[index].unit != b[index].unit) {
        return false;
      }
    }
    return true;
  };
  const auto same_stages = [](const auto& a, const auto& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t index = 0; index < a.size(); ++index) {
      if (a[index].stage != b[index].stage ||
          a[index].duration.nanoseconds != b[index].duration.nanoseconds ||
          a[index].succeeded != b[index].succeeded) {
        return false;
      }
    }
    return true;
  };
  const auto same_failures = [](const auto& a, const auto& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t index = 0; index < a.size(); ++index) {
      if (a[index].code != b[index].code ||
          a[index].planning_stage != b[index].planning_stage ||
          a[index].external_stage != b[index].external_stage ||
          a[index].message != b[index].message ||
          a[index].has_numeric_evidence != b[index].has_numeric_evidence ||
          a[index].constraint_value != b[index].constraint_value ||
          a[index].hard_limit != b[index].hard_limit ||
          a[index].position_m.has_value() != b[index].position_m.has_value()) {
        return false;
      }
      if (a[index].position_m &&
          (a[index].position_m->x_m != b[index].position_m->x_m ||
           a[index].position_m->y_m != b[index].position_m->y_m)) {
        return false;
      }
    }
    return true;
  };
  const auto same_durations = [](const auto& a, const auto& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t index = 0; index < a.size(); ++index) {
      if (a[index].nanoseconds != b[index].nanoseconds) return false;
    }
    return true;
  };
  const auto same_authorization = [](const auto& a, const auto& b) {
    if (a.has_value() != b.has_value()) return false;
    if (!a) return true;
    return serialize_planning_result(a->plan.value()) ==
               serialize_planning_result(b->plan.value()) &&
           a->lease.lease_sequence == b->lease.lease_sequence &&
           a->lease.plan_sequence_number == b->lease.plan_sequence_number &&
           a->lease.validated_at.nanoseconds ==
               b->lease.validated_at.nanoseconds &&
           a->lease.expires_at.nanoseconds == b->lease.expires_at.nanoseconds &&
           a->path_cost == b->path_cost &&
           a->remaining_path && b->remaining_path &&
           same_path(a->remaining_path->geometry,
                     b->remaining_path->geometry);
  };
  const AlgorithmRiskRecord& a_risk = left.risk;
  const AlgorithmRiskRecord& b_risk = right.risk;
  return left.cycle_sequence == right.cycle_sequence &&
         left.random_seed == right.random_seed &&
         left.initial_source_revision == right.initial_source_revision &&
         left.decision_source_revision == right.decision_source_revision &&
         left.dependencies == right.dependencies &&
         left.decision_dependencies == right.decision_dependencies &&
         same_input(left.initial_inputs, right.initial_inputs) &&
         same_input(left.decision_inputs, right.decision_inputs) &&
         serialize_input_captures(left.input_captures) ==
             serialize_input_captures(right.input_captures) &&
         left.committed_lease_was_revoked ==
             right.committed_lease_was_revoked &&
         same_authorization(left.initial_authorization,
                            right.initial_authorization) &&
         left.canonical_parameters == right.canonical_parameters &&
         left.parameter_fingerprint == right.parameter_fingerprint &&
         left.parameter_versions.search_parameters ==
             right.parameter_versions.search_parameters &&
         left.parameter_versions.motion_primitives ==
             right.parameter_versions.motion_primitives &&
         left.parameter_versions.smoothing_limits ==
             right.parameter_versions.smoothing_limits &&
         left.parameter_versions.parameterization_limits ==
             right.parameter_versions.parameterization_limits &&
         left.parameter_versions.execution_profile ==
             right.parameter_versions.execution_profile &&
         same_search(left.search, right.search) &&
         left.smoothing.iterations == right.smoothing.iterations &&
         left.smoothing.maximum_constraint_residual ==
             right.smoothing.maximum_constraint_residual &&
         left.parameterization.limits_version ==
             right.parameterization.limits_version &&
         left.parameterization.execution_profile_version ==
             right.parameterization.execution_profile_version &&
         left.parameterization.execution_operating_envelope_version ==
             right.parameterization.execution_operating_envelope_version &&
         left.parameterization.minimum_constraint_margin ==
             right.parameterization.minimum_constraint_margin &&
         same_margins(left.parameterization.constraint_margins,
                      right.parameterization.constraint_margins) &&
         same_stages(left.stage_duration_samples,
                     right.stage_duration_samples) &&
          same_durations(left.lease_revalidation_duration_samples,
                         right.lease_revalidation_duration_samples) &&
          left.lease_revalidation_duration.nanoseconds ==
              right.lease_revalidation_duration.nanoseconds &&
          left.total_cycle_duration.nanoseconds ==
              right.total_cycle_duration.nanoseconds &&
         same_failures(left.constraint_failures, right.constraint_failures) &&
         a_risk.epsilon_robot_pointwise == b_risk.epsilon_robot_pointwise &&
         a_risk.epsilon_terrain_gradient_local ==
             b_risk.epsilon_terrain_gradient_local &&
         a_risk.epsilon_cable_pointwise == b_risk.epsilon_cable_pointwise &&
         a_risk.epsilon_path == b_risk.epsilon_path &&
         a_risk.path_joint_risk_implemented ==
             b_risk.path_joint_risk_implemented &&
         a_risk.terrain_gradient_path_joint_risk_implemented ==
             b_risk.terrain_gradient_path_joint_risk_implemented &&
         a_risk.semantics == b_risk.semantics;
}

template <typename Value>
ExperimentQuantiles<Value> nearest_rank_quantiles(std::vector<Value> samples) {
  ExperimentQuantiles<Value> result;
  if (samples.empty()) return result;
  std::sort(samples.begin(), samples.end());
  const auto at = [&samples](const double percentile) {
    const std::size_t rank = static_cast<std::size_t>(
        std::ceil(percentile * static_cast<double>(samples.size())));
    return samples.at(std::max<std::size_t>(1U, rank) - 1U);
  };
  result.p50 = at(0.50);
  result.p95 = at(0.95);
  result.p99 = at(0.99);
  return result;
}

std::optional<std::int64_t> duration_for(
    const AlgorithmExperimentRecord& record, const PlanningCycleStage stage) {
  for (const StageDurationSample& sample : record.stage_duration_samples) {
    if (sample.stage == stage) return sample.duration.nanoseconds;
  }
  return std::nullopt;
}

bool timed_out(const AlgorithmExperimentRecord& record) {
  if (record.final_state == PlanningState::timeout ||
      record.search.deadline_exceeded) {
    return true;
  }
  if (!record.root_cause.has_value()) return false;
  const PlanningFailureCause cause = record.root_cause->cause;
  return cause == PlanningFailureCause::search_deadline_exceeded ||
         cause == PlanningFailureCause::smoothing_deadline_exceeded ||
         cause == PlanningFailureCause::parameterization_deadline_exceeded ||
         cause == PlanningFailureCause::planning_cycle_deadline_exceeded;
}

class RecordedReplayInputs final : public MainPlanningLoopInputSource {
 public:
  explicit RecordedReplayInputs(const AlgorithmExperimentRecord& record)
      : record_(record) {}

  AlgorithmRuntimeParameterSnapshot capture_runtime_parameters()
      const override {
    return record_.parameters;
  }

  ValidationInputCaptureResult capture(MonotonicTime) override {
    if (capture_index_ >= record_.input_captures.size()) {
      return {ValidationInputCaptureStatus::validation_context_invalid,
              std::nullopt,
              {}};
    }
    return record_.input_captures[capture_index_++];
  }

 private:
  const AlgorithmExperimentRecord& record_;
  std::size_t capture_index_{};
};

}  // namespace

ProcessMemorySample sample_process_memory() {
#ifdef _WIN32
  PROCESS_MEMORY_COUNTERS_EX counters{};
  if (GetProcessMemoryInfo(
          GetCurrentProcess(),
          reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&counters),
          sizeof(counters)) == 0) {
    return {};
  }
  return {static_cast<std::size_t>(counters.WorkingSetSize),
          static_cast<std::size_t>(counters.PeakWorkingSetSize)};
#else
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0) return {};
#ifdef __APPLE__
  const std::size_t peak_bytes = static_cast<std::size_t>(usage.ru_maxrss);
#else
  const std::size_t peak_bytes =
      static_cast<std::size_t>(usage.ru_maxrss) * 1024U;
#endif
  long resident_pages = 0;
  long page_size = 0;
#ifdef __linux__
  std::ifstream statm("/proc/self/statm");
  long total_pages = 0;
  statm >> total_pages >> resident_pages;
  static_cast<void>(total_pages);
  page_size = sysconf(_SC_PAGESIZE);
#endif
  const std::size_t resident_bytes =
      resident_pages > 0 && page_size > 0
          ? static_cast<std::size_t>(resident_pages) *
                static_cast<std::size_t>(page_size)
          : peak_bytes;
  return {resident_bytes, peak_bytes};
#endif
}

AlgorithmExperimentRecord AlgorithmDiagnosticsRecorder::capture(
    const PlanningCycleRequest& request, const PlanningCycleResult& result) {
  AlgorithmExperimentRecord record;
  record.request = request;
  record.cycle_sequence = result.diagnostics.cycle_sequence;
  record.random_seed = result.diagnostics.random_seed;
  record.initial_source_revision = result.diagnostics.initial_source_revision;
  record.decision_source_revision = result.diagnostics.decision_source_revision;
  record.dependencies = result.diagnostics.initial_dependencies;
  record.decision_dependencies = result.diagnostics.decision_dependencies;
  record.parameters = result.diagnostics.parameters;
  record.canonical_parameters =
      serialize_runtime_parameters(record.parameters);
  record.parameter_fingerprint = fingerprint(record.canonical_parameters);
  if (result.initial_inputs) record.initial_inputs = *result.initial_inputs;
  if (result.decision_inputs) record.decision_inputs = *result.decision_inputs;
  record.initial_authorization = result.replay_initial_authorization;
  record.input_captures = result.replay_input_captures;
  record.committed_lease_was_revoked =
      result.replay_committed_lease_revoked;
  record.final_status = result.status;
  record.final_state = result.state;
  const FinalPlanningEvidence final_evidence = select_final_evidence(result);
  record.final_path = final_evidence.path;
  record.final_diagnostics = final_evidence.diagnostics;
  record.root_cause = result.root_cause;
  record.process_memory = sample_process_memory();

  for (const PlanningCycleStageMetric& metric : result.diagnostics.stages) {
    record.stage_duration_samples.push_back(
        {metric.stage, metric.duration, metric.succeeded});
    if (metric.stage == PlanningCycleStage::candidate_revalidation ||
        metric.stage == PlanningCycleStage::current_plan_revalidation) {
      record.lease_revalidation_duration_samples.push_back(metric.duration);
      record.lease_revalidation_duration.nanoseconds +=
          metric.duration.nanoseconds;
    }
  }
  if (!result.diagnostics.stages.empty()) {
    std::int64_t earliest =
        result.diagnostics.cycle_started_at.nanoseconds >= 0
            ? result.diagnostics.cycle_started_at.nanoseconds
            : std::numeric_limits<std::int64_t>::max();
    std::int64_t latest = std::numeric_limits<std::int64_t>::min();
    bool representable = true;
    for (const PlanningCycleStageMetric& metric : result.diagnostics.stages) {
      if (metric.started_at.nanoseconds < 0 || metric.duration.nanoseconds < 0 ||
          metric.started_at.nanoseconds >
              std::numeric_limits<std::int64_t>::max() -
                  metric.duration.nanoseconds) {
        representable = false;
        break;
      }
      earliest = std::min(earliest, metric.started_at.nanoseconds);
      latest = std::max(latest, metric.started_at.nanoseconds +
                                    metric.duration.nanoseconds);
    }
    if (representable) {
      record.total_cycle_duration = {latest - earliest};
    }
  }

  if (result.artifacts.search) {
    record.search.executed = true;
    const HybridAStarSearchDiagnostics& diagnostics =
        result.artifacts.search->diagnostics;
    record.parameter_versions.search_parameters =
        diagnostics.search_parameter_version;
    record.parameter_versions.motion_primitives = diagnostics.primitive_set_version;
    record.search.peak_active_labels = diagnostics.peak_active_label_count;
    record.search.fixed_bytes_per_search_label =
        diagnostics.fixed_bytes_per_search_label;
    record.search.peak_observed_bytes_per_search_label =
        diagnostics.peak_observed_bytes_per_search_label;
    record.search.maximum_active_labels =
        diagnostics.maximum_active_label_budget;
    record.search.labels_per_base_key_p50 = diagnostics.labels_per_base_key_p50;
    record.search.labels_per_base_key_p95 = diagnostics.labels_per_base_key_p95;
    record.search.labels_per_base_key_p99 = diagnostics.labels_per_base_key_p99;
    record.search.equivalent_discard_count =
        diagnostics.equivalent_label_discard_count;
    record.search.equivalent_replacement_count =
        diagnostics.equivalent_label_replacement_count;
    record.search.signature_fallback_comparison_count =
        diagnostics.signature_fallback_comparison_count;
    record.search.label_budget_exhausted =
        diagnostics.active_label_budget_exhausted;
    record.search.deadline_exceeded = diagnostics.deadline_exceeded;
    if (diagnostics.worst_constraint.recorded) {
      record.constraint_failures.push_back(
          {diagnostics.worst_constraint.reason,
           PlanningCycleStage::search,
           {},
           "worst hard-constraint utilization observed during search",
           true,
           diagnostics.worst_constraint.constraint_value,
           diagnostics.worst_constraint.hard_limit,
           diagnostics.worst_constraint.position_m});
    }
  }
  if (result.artifacts.smoothing) {
    record.smoothing.iterations =
        result.artifacts.smoothing->audit.solver_iterations;
    record.smoothing.maximum_constraint_residual =
        maximum_residual(result.artifacts.smoothing->residuals);
    record.parameter_versions.smoothing_limits =
        result.artifacts.smoothing->audit.limits_version;
  }
  if (result.artifacts.parameterization) {
    record.parameterization =
        parameterization_sample(*result.artifacts.parameterization);
    record.parameter_versions.parameterization_limits =
        record.parameterization.limits_version;
    record.parameter_versions.execution_profile =
        record.parameterization.execution_profile_version;
  }

  record.risk.planning_succeeded =
      result.status == PlanningCycleStatus::success &&
      result.state == PlanningState::success;
  record.risk.semantics = record.final_diagnostics.risk_semantics.empty()
                              ? kRiskSemantics
                              : record.final_diagnostics.risk_semantics;
  if (final_evidence.error_budget.has_value()) {
    const ErrorBudget& budget = *final_evidence.error_budget;
    record.risk.epsilon_robot_pointwise = budget.epsilon_robot;
    record.risk.epsilon_terrain_gradient_local =
        budget.epsilon_terrain_gradient_local;
    record.risk.epsilon_cable_pointwise = budget.epsilon_point;
    record.risk.epsilon_path = budget.epsilon_path;
    record.risk.path_joint_risk_implemented =
        budget.path_joint_risk_implemented;
    record.risk.terrain_gradient_path_joint_risk_implemented =
        budget.terrain_gradient_path_joint_risk_implemented;
  }
  if (record.root_cause.has_value()) {
    record.constraint_failures.push_back(
        {record.root_cause->reason_code,
         record.root_cause->stage,
         {},
         record.root_cause->message,
         false,
         0.0,
         0.0,
         std::nullopt});
  }
  for (const DiagnosticEntry& entry : record.final_diagnostics.entries) {
    if (entry.severity == DiagnosticSeverity::error) {
      record.constraint_failures.push_back(
          {entry.code, std::nullopt, entry.stage, entry.message, false, 0.0, 0.0,
           std::nullopt});
    }
  }
  return record;
}

ExperimentRecordValidation validate(const AlgorithmExperimentRecord& record) {
  ExperimentRecordValidation result;
  if (record.schema_version != "algorithm-experiment/v3") {
    result.issues.emplace_back("unsupported experiment record schema");
  }
  if (record.cycle_sequence == 0U ||
      record.request.cycle_sequence != record.cycle_sequence ||
      record.request.random_seed != record.random_seed) {
    result.issues.emplace_back("cycle identity or random seed is inconsistent");
  }
  if (record.initial_inputs.has_value()) {
    if (record.initial_inputs->source_revision !=
            record.initial_source_revision ||
        record.initial_inputs->dependencies != record.dependencies) {
      result.issues.emplace_back("initial replay input does not match its version");
    }
  } else if (record.initial_source_revision != 0U) {
    result.issues.emplace_back("initial replay input is missing");
  }
  if (record.decision_inputs.has_value() &&
      (record.decision_inputs->source_revision !=
           record.decision_source_revision ||
       record.decision_inputs->dependencies != record.decision_dependencies)) {
    result.issues.emplace_back("decision replay input does not match its version");
  } else if (!record.decision_inputs.has_value() &&
             record.decision_source_revision != 0U) {
    result.issues.emplace_back("decision replay input is missing");
  }
  const ParameterValidationResult parameter_validation =
      validate_parameters(record.parameters.profile,
                          record.parameters.profile.mode);
  const std::string canonical_parameters =
      serialize_runtime_parameters(record.parameters);
  if (!parameter_validation.valid || canonical_parameters.empty() ||
      record.canonical_parameters != canonical_parameters ||
      record.parameter_fingerprint != fingerprint(canonical_parameters)) {
    result.issues.emplace_back(
        "typed algorithm parameters or derived fingerprint are invalid");
  }
  if (record.input_captures.empty()) {
    result.issues.emplace_back("planning input capture sequence is missing");
  } else if (record.initial_inputs.has_value() &&
             serialize_replay_input(record.input_captures.front().inputs) !=
                 serialize_replay_input(record.initial_inputs)) {
    result.issues.emplace_back(
        "initial input does not match the recorded capture sequence");
  }
  if (record.decision_inputs.has_value()) {
    const std::string decision =
        serialize_replay_input(record.decision_inputs);
    const bool found = std::any_of(
        record.input_captures.begin(), record.input_captures.end(),
        [&decision](const ValidationInputCaptureResult& capture) {
          return serialize_replay_input(capture.inputs) == decision;
        });
    if (!found) {
      result.issues.emplace_back(
          "decision input is absent from the recorded capture sequence");
    }
  }
  if (record.committed_lease_was_revoked &&
      !record.request.committed_start.has_value()) {
    result.issues.emplace_back(
        "revoked commitment state has no committed request");
  }
  if (record.initial_authorization.has_value() &&
      (!finite(record.initial_authorization->path_cost) ||
       record.initial_authorization->path_cost < 0.0)) {
    result.issues.emplace_back(
        "initial authorization path cost is non-finite or negative");
  }
  for (const StageDurationSample& sample : record.stage_duration_samples) {
    if (sample.duration.nanoseconds < 0) {
      result.issues.emplace_back("stage duration is negative");
      break;
    }
  }
  if (record.total_cycle_duration.nanoseconds < 0) {
    result.issues.emplace_back("total planning-cycle duration is negative");
  }
  if ((record.process_memory.resident_bytes == 0U) !=
          (record.process_memory.peak_resident_bytes == 0U) ||
      record.process_memory.resident_bytes >
          record.process_memory.peak_resident_bytes) {
    result.issues.emplace_back("process memory sample is invalid");
  }
  if (record.search.executed &&
      (record.search.fixed_bytes_per_search_label == 0U ||
       record.search.peak_observed_bytes_per_search_label <
           record.search.fixed_bytes_per_search_label)) {
    result.issues.emplace_back("search label storage size is missing");
  }
  if (!finite(record.smoothing.maximum_constraint_residual) ||
      record.smoothing.maximum_constraint_residual < 0.0) {
    result.issues.emplace_back("smoothing residual is non-finite or negative");
  }
  for (const ConstraintMarginSample& margin :
       record.parameterization.constraint_margins) {
    if (margin.constraint.empty() || margin.unit.empty() ||
        !finite(margin.margin)) {
      result.issues.emplace_back("parameterization margin is incomplete");
      break;
    }
  }
  for (const ConstraintFailureSample& failure : record.constraint_failures) {
    if (failure.code.empty() ||
        (!failure.planning_stage.has_value() &&
         failure.external_stage.empty()) ||
        failure.message.empty() ||
        (failure.has_numeric_evidence &&
         (!finite(failure.constraint_value) || !finite(failure.hard_limit)))) {
      result.issues.emplace_back("constraint failure evidence is incomplete");
      break;
    }
  }
  const auto valid_epsilon = [](const std::optional<double>& epsilon) {
    return !epsilon.has_value() ||
           (finite(*epsilon) && *epsilon > 0.0 && *epsilon < 1.0);
  };
  if (!valid_epsilon(record.risk.epsilon_robot_pointwise) ||
      !valid_epsilon(record.risk.epsilon_terrain_gradient_local) ||
      !valid_epsilon(record.risk.epsilon_cable_pointwise)) {
    result.issues.emplace_back("local statistical risk epsilon is invalid");
  }
  if (record.risk.epsilon_path.has_value() ||
      record.risk.path_joint_risk_implemented ||
      record.risk.terrain_gradient_path_joint_risk_implemented ||
      record.risk.semantics != kRiskSemantics) {
    result.issues.emplace_back("experiment record overstates joint risk semantics");
  }
  result.valid = result.issues.empty();
  return result;
}

std::string serialize_algorithm_experiment_record(
    const AlgorithmExperimentRecord& record) {
  std::ostringstream stream;
  stream << std::setprecision(17) << record.schema_version << '\n'
         << record.cycle_sequence << ' ' << record.random_seed << ' '
         << record.initial_source_revision << ' '
         << record.decision_source_revision << '\n'
         << record.dependencies.map_version.map_id << ' '
         << record.dependencies.map_version.sequence_number << ' '
         << record.dependencies.reference_line_version << ' '
         << record.dependencies.robot_operating_area_version << ' '
         << record.dependencies.cable_corridor_version << ' '
         << record.dependencies.terrain_gradient_policy_version << ' '
         << record.dependencies.corridor_risk_policy_version << ' '
         << record.dependencies.cable_model_version << ' '
         << record.dependencies.uncertainty_envelope_version << ' '
         << record.dependencies.uncertainty_envelope_generator_version << ' '
         << record.dependencies.execution_operating_envelope_version << ' '
         << record.dependencies.execution_profile_version << '\n'
         << record.decision_dependencies.map_version.sequence_number << ' '
         << record.decision_dependencies.reference_line_version << ' '
         << record.decision_dependencies.robot_operating_area_version << ' '
         << record.decision_dependencies.cable_corridor_version << ' '
         << record.decision_dependencies.terrain_gradient_policy_version << ' '
         << record.decision_dependencies.corridor_risk_policy_version << ' '
         << record.decision_dependencies.cable_model_version << ' '
         << record.decision_dependencies.uncertainty_envelope_version << ' '
         << record.decision_dependencies.uncertainty_envelope_generator_version
         << ' '
         << record.decision_dependencies.execution_operating_envelope_version
         << ' ' << record.decision_dependencies.execution_profile_version
         << '\n'
         << record.canonical_parameters.size() << ':'
         << record.canonical_parameters << '\n'
         << record.parameter_fingerprint << '\n'
         << record.parameter_versions.search_parameters << ' '
         << record.parameter_versions.motion_primitives << ' '
         << record.parameter_versions.smoothing_limits << ' '
         << record.parameter_versions.parameterization_limits << ' '
         << record.parameter_versions.execution_profile << '\n'
         << static_cast<int>(record.final_status) << ' '
          << static_cast<int>(record.final_state) << '\n'
          << record.search.executed << ' '
          << record.search.fixed_bytes_per_search_label << ' '
          << record.search.peak_observed_bytes_per_search_label << ' '
          << record.search.peak_active_labels << ' '
         << record.search.maximum_active_labels << ' '
         << record.search.labels_per_base_key_p50 << ' '
         << record.search.labels_per_base_key_p95 << ' '
         << record.search.labels_per_base_key_p99 << ' '
         << record.search.equivalent_discard_count << ' '
         << record.search.equivalent_replacement_count << ' '
         << record.search.signature_fallback_comparison_count << ' '
         << record.search.label_budget_exhausted << ' '
         << record.search.deadline_exceeded << '\n'
         << record.smoothing.iterations << ' '
         << record.smoothing.maximum_constraint_residual << '\n'
         << record.parameterization.limits_version << ' '
         << record.parameterization.execution_profile_version << ' '
         << record.parameterization.execution_operating_envelope_version << ' '
         << record.parameterization.minimum_constraint_margin << '\n';
  const std::string initial_input =
      serialize_replay_input(record.initial_inputs);
  const std::string decision_input =
      serialize_replay_input(record.decision_inputs);
  const std::string input_captures =
      serialize_input_captures(record.input_captures);
  stream << initial_input.size() << ':' << initial_input << '\n'
         << decision_input.size() << ':' << decision_input << '\n'
         << input_captures.size() << ':' << input_captures << '\n'
         << record.committed_lease_was_revoked << '\n';
  if (record.initial_authorization) {
    const std::string initial_plan =
        serialize_planning_result(record.initial_authorization->plan.value());
    stream << initial_plan.size() << ':' << initial_plan << '\n'
           << record.initial_authorization->lease.lease_sequence << ' '
           << record.initial_authorization->lease.plan_sequence_number << ' '
           << record.initial_authorization->lease.validated_at.nanoseconds << ' '
           << record.initial_authorization->lease.expires_at.nanoseconds << ' '
           << record.initial_authorization->path_cost << '\n';
  } else {
    stream << "0:\n";
  }
  for (const ConstraintMarginSample& margin :
       record.parameterization.constraint_margins) {
    stream << margin.constraint << ':' << margin.margin << ':' << margin.unit
           << ';';
  }
  stream << '\n';
  for (const StageDurationSample& sample : record.stage_duration_samples) {
    stream << static_cast<int>(sample.stage) << ':'
           << sample.duration.nanoseconds << ':' << sample.succeeded << ' ';
  }
  stream << '\n';
  for (const Duration duration :
       record.lease_revalidation_duration_samples) {
    stream << duration.nanoseconds << ' ';
  }
  stream << '\n' << record.lease_revalidation_duration.nanoseconds << '\n'
         << record.total_cycle_duration.nanoseconds << ' '
         << record.process_memory.resident_bytes << ' '
         << record.process_memory.peak_resident_bytes << '\n';
  for (const ConstraintFailureSample& failure : record.constraint_failures) {
    stream << failure.code << ':'
           << (failure.planning_stage.has_value()
                   ? static_cast<int>(*failure.planning_stage)
                   : -1)
           << ':' << failure.external_stage << ':'
           << failure.has_numeric_evidence << ':' << failure.constraint_value
           << ':' << failure.hard_limit << ';';
  }
  stream << '\n';
  if (record.final_path) {
    for (const PathPoint& point : record.final_path->points) {
      stream << point.arc_length_m << ',' << point.x_m << ',' << point.y_m << ','
             << point.heading_rad << ',' << point.curvature_per_m << ';';
    }
  }
  stream << '\n' << record.final_diagnostics.schema_version << '\n'
         << record.final_diagnostics.input_version << '\n'
         << record.final_diagnostics.risk_semantics << '\n';
  for (const DiagnosticEntry& entry : record.final_diagnostics.entries) {
    stream << static_cast<int>(entry.severity) << ':' << entry.code << ':'
           << entry.stage << ':' << entry.message << ':'
           << entry.timestamp.nanoseconds << ';';
  }
  stream << '\n' << record.risk.planning_succeeded << ' '
         << record.risk.epsilon_robot_pointwise.value_or(-1.0) << ' '
         << record.risk.epsilon_terrain_gradient_local.value_or(-1.0) << ' '
         << record.risk.epsilon_cable_pointwise.value_or(-1.0) << ' '
         << record.risk.epsilon_path.value_or(-1.0) << ' '
         << record.risk.path_joint_risk_implemented << ' '
         << record.risk.terrain_gradient_path_joint_risk_implemented << '\n'
         << record.risk.semantics << '\n';
  return stream.str();
}

ExperimentRecordValidation AlgorithmExperimentLog::append(
    AlgorithmExperimentRecord record) {
  ExperimentRecordValidation validation = validate(record);
  if (!records_.empty() &&
      record.cycle_sequence <= records_.back().cycle_sequence) {
    validation.valid = false;
    validation.issues.emplace_back("experiment cycle sequence is not monotonic");
  }
  records_.push_back(std::move(record));
  summary_eligible_.push_back(validation.valid);
  return validation;
}

AlgorithmExperimentSummary AlgorithmExperimentLog::summarize() const {
  AlgorithmExperimentSummary summary;
  std::size_t exhausted = 0U;
  std::size_t timeout_count = 0U;
  std::map<PlanningCycleStage, std::vector<std::int64_t>> stage_samples;
  for (std::size_t record_index = 0; record_index < records_.size();
       ++record_index) {
    if (record_index >= summary_eligible_.size() ||
        !summary_eligible_[record_index]) {
      ++summary.rejected_sample_count;
      continue;
    }
    ++summary.sample_count;
    const AlgorithmExperimentRecord& record = records_[record_index];
    if (record.total_cycle_duration.nanoseconds > 0) {
      summary.total_cycle_duration_samples_ns.push_back(
          record.total_cycle_duration.nanoseconds);
    }
    if (record.process_memory.peak_resident_bytes > 0U) {
      summary.process_peak_memory_samples_bytes.push_back(
          record.process_memory.peak_resident_bytes);
    }
    timeout_count += timed_out(record) ? 1U : 0U;
    for (const StageDurationSample& sample : record.stage_duration_samples) {
      stage_samples[sample.stage].push_back(sample.duration.nanoseconds);
    }
    if (record.search.executed) {
      ++summary.search_sample_count;
      summary.fixed_bytes_per_search_label_samples.push_back(
          record.search.fixed_bytes_per_search_label);
      summary.peak_observed_bytes_per_search_label_samples.push_back(
          record.search.peak_observed_bytes_per_search_label);
      summary.peak_active_label_samples.push_back(
          record.search.peak_active_labels);
      summary.equivalence_comparison_samples.push_back(
          record.search.equivalent_discard_count +
          record.search.equivalent_replacement_count +
          record.search.signature_fallback_comparison_count);
      exhausted += record.search.label_budget_exhausted ? 1U : 0U;
      if (const auto duration =
              duration_for(record, PlanningCycleStage::search)) {
        summary.search_duration_samples_ns.push_back(*duration);
      }
    }
    if (const auto duration =
            duration_for(record, PlanningCycleStage::smoothing)) {
      summary.smoothing_duration_samples_ns.push_back(*duration);
    }
    for (const Duration duration : record.lease_revalidation_duration_samples) {
      summary.lease_revalidation_duration_samples_ns.push_back(
          duration.nanoseconds);
    }
  }
  if (summary.search_sample_count != 0U) {
    summary.label_budget_exhaustion_rate =
        static_cast<double>(exhausted) /
        static_cast<double>(summary.search_sample_count);
  }
  if (summary.sample_count != 0U) {
    summary.timeout_rate = static_cast<double>(timeout_count) /
                           static_cast<double>(summary.sample_count);
  }
  summary.peak_active_labels =
      nearest_rank_quantiles(summary.peak_active_label_samples);
  summary.search_duration_ns =
      nearest_rank_quantiles(summary.search_duration_samples_ns);
  summary.smoothing_duration_ns =
      nearest_rank_quantiles(summary.smoothing_duration_samples_ns);
  summary.lease_revalidation_duration_ns = nearest_rank_quantiles(
      summary.lease_revalidation_duration_samples_ns);
  summary.fixed_bytes_per_search_label =
      nearest_rank_quantiles(summary.fixed_bytes_per_search_label_samples);
  summary.peak_observed_bytes_per_search_label = nearest_rank_quantiles(
      summary.peak_observed_bytes_per_search_label_samples);
  summary.process_peak_memory_bytes =
      nearest_rank_quantiles(summary.process_peak_memory_samples_bytes);
  summary.total_cycle_duration_ns =
      nearest_rank_quantiles(summary.total_cycle_duration_samples_ns);
  for (auto& [stage, samples] : stage_samples) {
    summary.stage_duration_ns.emplace(
        stage, nearest_rank_quantiles(std::move(samples)));
  }
  return summary;
}

PerformanceBudgetAssessment assess_performance_budget(
    const AlgorithmExperimentSummary& summary,
    const AlgorithmPerformanceBudget& budget) {
  PerformanceBudgetAssessment assessment;
  assessment.timeout_rate = summary.timeout_rate;
  if (!summary.process_peak_memory_samples_bytes.empty()) {
    assessment.maximum_observed_memory_bytes = *std::max_element(
        summary.process_peak_memory_samples_bytes.begin(),
        summary.process_peak_memory_samples_bytes.end());
  }
  if (!summary.total_cycle_duration_samples_ns.empty()) {
    assessment.maximum_observed_cycle_duration = {*std::max_element(
        summary.total_cycle_duration_samples_ns.begin(),
        summary.total_cycle_duration_samples_ns.end())};
  }

  if (budget.maximum_total_memory_bytes == 0U ||
      budget.maximum_cycle_duration.nanoseconds <= 0 ||
      !finite(budget.target_frequency_hz) ||
      budget.target_frequency_hz < 2.0 || budget.target_frequency_hz > 5.0 ||
      budget.minimum_sample_count == 0U) {
    assessment.status = PerformanceBudgetStatus::insufficient_evidence;
    assessment.diagnostics.emplace_back("INVALID_PERFORMANCE_BUDGET");
    return assessment;
  }
  if (summary.sample_count < budget.minimum_sample_count ||
      summary.process_peak_memory_samples_bytes.size() != summary.sample_count ||
      summary.total_cycle_duration_samples_ns.size() != summary.sample_count ||
      summary.search_sample_count == 0U ||
      summary.fixed_bytes_per_search_label_samples.size() !=
          summary.search_sample_count ||
      summary.peak_observed_bytes_per_search_label_samples.size() !=
          summary.search_sample_count) {
    assessment.status = PerformanceBudgetStatus::insufficient_evidence;
    assessment.diagnostics.emplace_back("INSUFFICIENT_PERFORMANCE_EVIDENCE");
    return assessment;
  }
  if (assessment.maximum_observed_memory_bytes >=
      budget.maximum_total_memory_bytes) {
    assessment.status = PerformanceBudgetStatus::memory_budget_exceeded;
    assessment.requires_safe_failure = true;
    assessment.diagnostics.emplace_back("MEMORY_BUDGET_EXCEEDED");
    return assessment;
  }
  if (assessment.timeout_rate > 0.0 ||
      assessment.maximum_observed_cycle_duration.nanoseconds >
          budget.maximum_cycle_duration.nanoseconds) {
    assessment.status = PerformanceBudgetStatus::cycle_timeout_exceeded;
    assessment.requires_safe_failure = true;
    assessment.diagnostics.emplace_back("CYCLE_TIMEOUT_EXCEEDED");
    return assessment;
  }
  const double target_period_ns = 1.0e9 / budget.target_frequency_hz;
  if (static_cast<double>(
          assessment.maximum_observed_cycle_duration.nanoseconds) >=
      target_period_ns) {
    assessment.status = PerformanceBudgetStatus::target_frequency_unverified;
    assessment.diagnostics.emplace_back("TARGET_FREQUENCY_UNVERIFIED");
    return assessment;
  }
  assessment.status = PerformanceBudgetStatus::verified;
  assessment.target_frequency_verified = true;
  return assessment;
}

AlgorithmExperimentReplayResult AlgorithmExperimentReplayer::replay(
    const AlgorithmExperimentRecord& record,
    MainPlanningLoopStages& stages, MainPlanningLoopClock clock) {
  if (!validate(record).valid) {
    return {AlgorithmExperimentReplayStatus::invalid_record,
            {"invalid_record"}};
  }
  if (!clock ||
      serialize_runtime_parameters(stages.capture_runtime_parameters()) !=
          record.canonical_parameters) {
    return {AlgorithmExperimentReplayStatus::replay_failed,
            {"replay_environment_mismatch"}};
  }
  auto replayed = std::make_unique<AlgorithmExperimentRecord>();
  try {
    RecordedReplayInputs replay_inputs(record);
    AuthorizedPlanningResultPublisher publisher;
    ExecutionLeaseMonitor lease_monitor;
    if (record.committed_lease_was_revoked) {
      lease_monitor.revokeLease(
          record.request.committed_start->lease_sequence,
          "REPLAY_INITIAL_REVOCATION",
          "commitment lease was revoked before the recorded cycle",
          record.request.triggered_at);
    }
    if (record.initial_authorization.has_value()) {
      const AuthorizedPlanningResult& initial = *record.initial_authorization;
      const AuthorizedPlanningPublication seeded = publisher.publish(
          initial.plan.value(), initial.remaining_path, initial.lease,
          initial.path_cost);
      if (!seeded.published()) {
        return {AlgorithmExperimentReplayStatus::replay_failed,
                {"initial_authorization_invalid"}};
      }
    }
    MainPlanningLoop loop(stages, publisher, lease_monitor, std::move(clock),
                          replay_inputs);
    const PlanningCycleResult replay_result = loop.run_cycle(record.request);
    *replayed = AlgorithmDiagnosticsRecorder::capture(record.request,
                                                      replay_result);
  } catch (...) {
    return {AlgorithmExperimentReplayStatus::replay_failed,
            {"replay_runner_failed"}};
  }

  AlgorithmExperimentReplayResult result;
  if (record.final_state != replayed->final_state ||
      record.final_status != replayed->final_status) {
    result.differences.emplace_back("final_state");
  }
  if (!same_path(record.final_path, replayed->final_path)) {
    result.differences.emplace_back("path");
  }
  if (!same_diagnostics(record.final_diagnostics,
                        replayed->final_diagnostics) ||
      !same_experiment_diagnostics(record, *replayed)) {
    result.differences.emplace_back("diagnostics");
  }
  result.status = result.differences.empty()
                      ? AlgorithmExperimentReplayStatus::reproduced
                      : AlgorithmExperimentReplayStatus::mismatch;
  return result;
}

}  // namespace underwater_planner::core
