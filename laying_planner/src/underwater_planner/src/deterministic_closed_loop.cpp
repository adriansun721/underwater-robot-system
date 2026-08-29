#include "underwater_planner/testing/deterministic_closed_loop.hpp"

#include "underwater_planner/core/execution_lease_monitor.hpp"
#include "underwater_planner/core/path_candidate_verifier.hpp"
#include "underwater_planner/core/main_planning_loop.hpp"
#include "underwater_planner/core/planning_state_machine.hpp"
#include "underwater_planner/core/scout_coordinator.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace underwater_planner::testing {
namespace {
using namespace underwater_planner::core;

constexpr const char* kDomain = "competition-level1/v1";
constexpr const char* kRiskSemantics =
    "POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE";

CableModelParameters cable_model_parameters(
    const std::uint64_t version = 8U) {
  CableModelParameters parameters;
  parameters.version = version;
  parameters.calibration_dataset_id = "closed-loop-cable-cal-v1";
  parameters.operating_domain_id = kDomain;
  parameters.release_point_offset_m = {0.0, 0.0};
  parameters.touchdown_distance_m = 1.0;
  parameters.direction_response_length_m = 2.0;
  parameters.maximum_lag_angle_rad = 0.5;
  parameters.maximum_payout_tracking_error_mps = 0.1;
  parameters.payout_speed_range = {0.0, 1.0};
  parameters.maximum_payout_acceleration_mps2 = 10.0;
  parameters.maximum_tension_tracking_error_n = 10.0;
  parameters.tension_range = {10.0, 100.0};
  parameters.search_integration_step_m = 0.25;
  parameters.validation_integration_step_m = 0.02;
  parameters.touchdown_distance_variance_m2 = 0.0025;
  parameters.direction_response_length_variance_m2 = 0.04;
  parameters.lag_angle_process_variance_per_m_rad2 = 0.03;
  parameters.touchdown_process_noise_per_m_m2 = {0.001, 0.0, 0.0, 0.002};
  parameters.approved_sensor_modes = {SensorHealthMode::nominal};
  return parameters;
}

ExecutionOperatingEnvelope execution_envelope() {
  ExecutionOperatingEnvelope envelope;
  envelope.version = 11U;
  envelope.operating_domain_id = kDomain;
  envelope.limits.ground_speed = {0.0, 0.8};
  envelope.limits.ground_acceleration = {-10.0, 10.0};
  envelope.limits.maximum_lateral_acceleration_mps2 = 1.0;
  envelope.limits.payout_speed = {0.0, 0.9};
  envelope.limits.payout_acceleration = {-10.0, 10.0};
  envelope.limits.maximum_payout_tracking_error_mps = 0.08;
  envelope.limits.tension = {20.0, 80.0};
  envelope.limits.maximum_stopping_distance_m = 1.0;
  envelope.maximum_payout_acceleration_tracking_error_mps2 = 0.1;
  envelope.maximum_tension_tracking_error_n = 8.0;
  return envelope;
}

ParameterConfig progress_parameter_config() {
  ParameterConfig config;
  config.profile_id = "closed-loop-level1-v1";
  config.operating_domain_id = kDomain;
  config.search.reference_progress_backward_tolerance_m = 0.0;
  config.search.reference_progress_maximum_ratio = 1.0;
  config.search.reference_progress_forward_slack_m = 0.01;
  config.search.reference_progress_distance_scale_m = 1.0;
  config.search.reference_progress_heading_scale_rad = 0.5;
  config.search.reference_progress_heading_weight = 1.0;
  config.search.reference_progress_association_score_tolerance = 1.0e-10;
  return config;
}

HybridAStarSearchParameters search_parameters() {
  HybridAStarSearchParameters parameters;
  parameters.version = 32U;
  parameters.primitive_set_version = 33U;
  parameters.path_version = 31U;
  parameters.xy_resolution_m = 0.25;
  parameters.heading_resolution_rad = 0.25;
  parameters.cable_lag_resolution_rad = 0.25;
  parameters.reference_progress_resolution_m = 0.25;
  parameters.goal_position_tolerance_m = 0.02;
  parameters.goal_heading_tolerance_rad = 0.02;
  parameters.goal_lag_tolerance_rad = 0.06;
  parameters.goal_progress_tolerance_m = 0.2;
  parameters.goal_touchdown_position_tolerance_m = 0.2;
  parameters.minimum_turning_radius_m = 2.0;
  parameters.path_length_cost_weight = 1.0;
  parameters.path_curvature_cost_weight = 1.0;
  parameters.touchdown_center_cost_weight = 1.0;
  parameters.touchdown_margin_cost_weight = 1.0;
  parameters.robot_terrain_cost_weight = 20.0;
  parameters.maximum_sweep_spacing_fraction = 0.5;
  parameters.cable_sweep_margin_m = 0.01;
  parameters.equivalent_label_cost_tolerance_m = 0.0;
  parameters.maximum_planning_duration_s = 5.0;
  parameters.maximum_expansions = 2'000U;
  parameters.maximum_active_labels = 20'000U;
  parameters.analytic_expansion_interval = 1U;
  parameters.motion_primitives = {
      {33U, 1.0, 0.0}, {33U, 1.0, 0.5}, {33U, 1.0, -0.5}};
  return parameters;
}

CableCorridorRiskPolicy corridor_policy() {
  return {7U, "closed-loop-corridor-cal-v1", kDomain, 0.05,
          1.5, 2.5, 0.0, 0.01, true};
}

RobotCollisionRiskPolicy collision_policy() {
  return {12U, "closed-loop-collision-cal-v1", kDomain, 0.05, 0.5, 0.0};
}

RobotCapability robot_capability() {
  RobotParameterConfig parameters;
  parameters.maximum_slope_up_rad = 0.8;
  parameters.maximum_slope_down_rad = 0.8;
  parameters.maximum_slope_lateral_rad = 0.8;
  parameters.maximum_support_roll_rad = 0.8;
  parameters.maximum_step_climb_m = 0.3;
  parameters.maximum_step_drop_m = 0.3;
  parameters.minimum_track_support_ratio = 0.5;
  parameters.effective_track_spacing_m = 0.05;
  parameters.minimum_step_crossing_alignment = 0.2;
  parameters.step_alignment_transition_band = 0.1;
  parameters.maximum_roughness_m = 0.1;
  const std::optional<RobotCapability> capability =
      make_robot_capability(parameters);
  if (!capability.has_value()) {
    throw std::logic_error("closed-loop robot capability is invalid");
  }
  return *capability;
}

TrackFootprint track_footprint() {
  return {{{-0.05, -0.05}, {0.05, -0.05}, {0.05, 0.05},
           {-0.05, 0.05}},
          {{-0.04, 0.01}, {0.04, 0.01}, {0.04, 0.04}, {-0.04, 0.04}},
          {{-0.04, -0.04}, {0.04, -0.04}, {0.04, -0.01},
           {-0.04, -0.01}}};
}

TerrainGradientRiskPolicy gradient_policy() {
  return {6U, 6U, 0.05, 2.0, GradientCoverageModel::deterministic_bounded,
          "closed-loop-gradient-cal-v1", kDomain, true};
}

CableLayingLimits laying_limits() {
  return {14U, kDomain, 0.2, 100.0, 0.2, 0.5, 0.1,
          0.3, 0.5, 1.0e-6, 1.0, 1.0, 1.0};
}

SmoothingLimits smoothing_limits() {
  SmoothingLimits limits;
  limits.version = 34U;
  limits.output_path_version = 31U;
  limits.spatial_step_m = 0.1;
  limits.maximum_curvature_per_m = 1.0;
  limits.maximum_curvature_rate_per_m2 = 100.0;
  limits.minimum_segment_length_m = 0.001;
  limits.topology_tube_radius_m = 2.0;
  limits.timeout = {2'000'000'000};
  limits.maximum_boundary_time_skew = {2'000'000'000};
  limits.allowed_residuals.maximum_dynamics_residual = 1.0e-7;
  limits.allowed_residuals.maximum_curvature_audit_residual = 1.0e-7;
  limits.allowed_residuals.maximum_curvature_rate_residual = 1.0e-7;
  limits.allowed_residuals.start_position_residual_m = 1.0e-9;
  limits.allowed_residuals.start_heading_residual_rad = 1.0e-9;
  limits.allowed_residuals.start_curvature_residual_per_m = 1.0e-9;
  limits.allowed_residuals.goal_position_residual_m = 1.0e-5;
  limits.allowed_residuals.goal_heading_residual_rad = 1.0e-5;
  limits.allowed_residuals.goal_curvature_residual_per_m = 1.0e-5;
  limits.objective_weights = {1.0, 1.0, 1.0, 1.0};
  return limits;
}

TrajectoryParameterizationLimits parameterization_limits() {
  TrajectoryParameterizationLimits limits;
  limits.version = 35U;
  limits.sample_period_s = 0.1;
  limits.terminal_speed_mps = 0.0;
  limits.stopping_distance_margin_m = 0.1;
  limits.timeout = {50'000'000};
  limits.execution_profile_version = 13U;
  return limits;
}

PlanningDependencyVersions dependencies(const std::uint64_t map_sequence = 7U,
                                        const std::uint64_t cable_model = 8U) {
  return {{"closed-loop-map", map_sequence, {900}, "world"},
          4U,
          5U,
          6U,
          7U,
          cable_model,
          9U,
          10U,
          11U,
          13U,
          SensorHealthMode::nominal,
          kDomain,
          3U};
}

TerrainAnalysisConfig terrain_config();

template <typename Target>
void apply_dependencies(Target& target,
                        const PlanningDependencyVersions& versions) {
  target.map_version = versions.map_version;
  target.reference_line_version = versions.reference_line_version;
  target.robot_operating_area_version = versions.robot_operating_area_version;
  target.terrain_gradient_policy_version =
      versions.terrain_gradient_policy_version;
  target.corridor_risk_policy_version = versions.corridor_risk_policy_version;
  target.cable_model_version = versions.cable_model_version;
  target.uncertainty_envelope_version = versions.uncertainty_envelope_version;
  target.uncertainty_envelope_generator_version =
      versions.uncertainty_envelope_generator_version;
  target.execution_operating_envelope_version =
      versions.execution_operating_envelope_version;
  target.execution_profile_version = versions.execution_profile_version;
  target.sensor_mode = versions.sensor_mode;
  target.operating_domain_id = versions.operating_domain_id;
  target.cable_corridor_version = versions.cable_corridor_version;
}

AlgorithmRuntimeParameterSnapshot runtime_parameters() {
  AlgorithmRuntimeParameterSnapshot snapshot;
  snapshot.profile = progress_parameter_config();
  snapshot.profile.mode =
      ParameterProfileMode::non_production_capability_profile;
  snapshot.profile.search.maximum_active_labels = 20'000U;
  snapshot.profile.statistical_risk.maximum_planning_duration_s = 0.5;
  snapshot.terrain_analysis = terrain_config();
  snapshot.search = search_parameters();
  snapshot.smoothing = smoothing_limits();
  snapshot.parameterization = parameterization_limits();
  return snapshot;
}

TerrainAnalysisConfig terrain_config() {
  TerrainAnalysisConfig config;
  config.config_version = 6U;
  config.operating_domain_id = kDomain;
  config.surface_window_size_m = 0.75;
  config.minimum_fit_support_ratio = 0.6;
  config.huber_delta_m = 0.02;
  config.minimum_elevation_variance_m2 = 1.0e-6;
  config.temporal_weight_half_life_s = 10.0;
  config.maximum_irls_iterations = 12U;
  config.minimum_step_height_m = 0.1;
  config.step_support_band_width_m = 0.4;
  config.minimum_step_side_support_ratio = 0.6;
  config.minimum_step_extent_m = 0.5;
  config.step_noise_sigma_multiplier = 3.0;
  config.minimum_step_confidence = 0.75;
  config.minimum_step_normal_consistency = 0.7;
  return config;
}

MapSnapshot scenario_map(const ClosedLoopScenario scenario,
                         const PlanningDependencyVersions& versions) {
  MapSnapshot map;
  map.version = versions.map_version;
  map.width = 140U;
  map.height = 80U;
  map.resolution_m = 0.1;
  map.origin_x_m = -4.0;
  map.origin_y_m = -4.0;
  map.derived_configuration_version = 6U;
  for (std::size_t row = 0; row < map.height; ++row) {
    for (std::size_t column = 0; column < map.width; ++column) {
      const double x_m = map.origin_x_m +
                         static_cast<double>(column) * map.resolution_m;
      double elevation_m = 0.0;
      if (scenario == ClosedLoopScenario::traversable_slope) {
        elevation_m = 0.08 * x_m;
      } else if (scenario == ClosedLoopScenario::traversable_step &&
                 x_m >= 4.8) {
        elevation_m = 0.12;
      }
      map.cells.push_back(
          {elevation_m, 1.0e-4, 1.0, true, {800}, false, std::nullopt, false});
    }
  }
  if (scenario == ClosedLoopScenario::single_side_detour) {
    for (std::size_t column = 65U; column <= 75U; ++column) {
      for (std::size_t row = 40U; row <= 50U; ++row) {
        map.cells.at(row * map.width + column).obstacle = true;
      }
    }
  } else if (scenario == ClosedLoopScenario::double_side_detour) {
    for (std::size_t column = 68U; column <= 72U; ++column) {
      for (std::size_t row = 39U; row <= 41U; ++row) {
        map.cells.at(row * map.width + column).obstacle = true;
      }
    }
  } else if (scenario == ClosedLoopScenario::unknown_gap) {
    for (std::size_t column = 68U; column <= 72U; ++column) {
      for (std::size_t row = 0U; row < map.height; ++row) {
        map.cells.at(row * map.width + column).known = false;
      }
    }
  }
  return map;
}

SynchronizedValidationInputs inputs(
    const std::uint64_t revision,
    const PlanningDependencyVersions& versions,
    const ClosedLoopScenario scenario) {
  SynchronizedValidationInputs value;
  value.captured_at = {1'000};
  value.source_revision = revision;
  value.robot_state = {{1.0, 0.0, 0.0, {900}}, 0.5, 0.0, {900}, 20U};
  value.cable_state.kind = CableStateKind::tracked;
  value.cable_state.lag_angle_rad = 0.0;
  value.cable_state.lag_angle_variance_rad2 = 0.01;
  value.cable_state.timestamp = {900};
  value.cable_state.sequence_number = 21U;
  value.reference_progress = {4U, 0.0, {900}, 22U};
  value.cable_telemetry = {0.5, 0.0, 40.0, {900}, 23U};
  value.execution_tracking_state = {13U, 11U, 0.0, {900}, 24U, 0.0};
  value.planning_snapshot.map = scenario_map(scenario, versions);
  value.planning_snapshot.reference_line =
      scenario == ClosedLoopScenario::route_deviation_recovery
          ? make_reference_line(4U, "world",
                                {{0.0, -0.5}, {4.0, 0.0}, {8.0, 0.0}})
          : make_reference_line(4U, "world", {{0.0, 0.0}, {8.0, 0.0}});
  value.planning_snapshot.robot_operating_area =
      {5U, "robot-area", {{-3.5, -3.5}, {9.5, -3.5},
                           {9.5, 3.5}, {-3.5, 3.5}}};
  value.planning_snapshot.cable_corridor =
      {3U, "cable-corridor", {{-3.5, -3.0}, {9.5, -3.0},
                               {9.5, 3.0}, {-3.5, 3.0}}};
  value.dependencies = versions;
  value.tracker_update_receipt = {25U, 26U, std::nullopt, 23U, 21U, 22U};
  return value;
}

CableContext cable_context(const SynchronizedValidationInputs& inputs,
                           const bool covariance_breach = false) {
  CableContext context;
  context.current_telemetry = inputs.cable_telemetry;
  context.execution_envelope = execution_envelope();
  context.mode = PredictionMode::validation;
  context.sensor_mode = inputs.dependencies.sensor_mode;
  context.uncertainty_envelope_version =
      inputs.dependencies.uncertainty_envelope_version;
  context.uncertainty_envelope_generator_version =
      inputs.dependencies.uncertainty_envelope_generator_version;
  context.robot_uncertainty_profile_version = 15U;
  const double variance = covariance_breach ? 100.0 : 0.0025;
  for (const double progress_m : {0.0, 2.0, 4.0, 6.0, 8.0}) {
    context.robot_uncertainty_profile.push_back(
        {progress_m, {{variance, 0.0, 0.0, variance}, 0.0, 0.0, 0.01},
         0.002});
  }
  return context;
}

CableUncertaintyEnvelope uncertainty_envelope(
    const PlanningDependencyVersions& versions) {
  CableUncertaintyEnvelope envelope;
  envelope.validity = EnvelopeBuildValidity::valid;
  envelope.dependencies.generator_version =
      versions.uncertainty_envelope_generator_version;
  envelope.dependencies.cable_model_version = versions.cable_model_version;
  envelope.dependencies.execution_operating_envelope_version =
      versions.execution_operating_envelope_version;
  envelope.dependencies.reference_line_version =
      versions.reference_line_version;
  envelope.dependencies.operating_domain_version = 1U;
  envelope.dependencies.primitive_set_version = 33U;
  envelope.dependencies.initial_uncertainty_version = 2U;
  envelope.dependencies.sensor_uncertainty_version = 3U;
  envelope.dependencies.execution_uncertainty_version = 4U;
  envelope.dependencies.margin_certification_version = 5U;
  envelope.dependencies.sensor_mode = versions.sensor_mode;
  envelope.dependencies.operating_domain_id = versions.operating_domain_id;
  envelope.dependencies.cable_model_calibration_dataset_id =
      "closed-loop-cable-cal-v1";
  envelope.dependencies.certification_dataset_id =
      "closed-loop-envelope-proof-v1";
  envelope.dependencies.sensor_calibration_dataset_id =
      "closed-loop-sensor-proof-v1";
  envelope.dependencies.execution_uncertainty_calibration_dataset_id =
      "closed-loop-execution-proof-v1";
  envelope.dependencies.margin_calibration_dataset_id =
      "closed-loop-margin-proof-v1";
  envelope.margin_budget = {5U, "closed-loop-margin-proof-v1",
                            0.001, 0.002, 0.003, 0.004};
  envelope.segments = {{0.0, 10.0, 1.0, 1.0}};
  envelope.generation_timestamp = {800};
  envelope.path_joint_risk_implemented = false;
  envelope.risk_semantics = kPointwiseEnvelopeRiskSemantics;
  return envelope;
}

LockedCableUncertaintyEnvelope lock_envelope(
    CableUncertaintyEnvelopeManager& manager,
    const PlanningDependencyVersions& versions) {
  const CableUncertaintyEnvelope envelope = uncertainty_envelope(versions);
  EnvelopeCoverageCertification certification;
  certification.version = 16U;
  certification.calibration_dataset_id = "closed-loop-envelope-audit-v1";
  certification.passed = true;
  certification.audited_at = {800};
  certification.valid_until = {9'000'000'000};
  certification.certified_envelope_version =
      versions.uncertainty_envelope_version;
  certification.certified_dependencies = envelope.dependencies;
  if (!manager
           .registerValidated(versions.uncertainty_envelope_version, envelope,
                              certification)
           .accepted()) {
    throw std::runtime_error("closed-loop envelope registration failed");
  }
  const EnvelopeLookupKey key{versions.reference_line_version,
                              versions.sensor_mode,
                              versions.operating_domain_id,
                              versions.cable_model_version,
                              versions.execution_operating_envelope_version};
  if (manager.setCurrentContext(key, 1U, {850}).context_update_status !=
      EnvelopeContextUpdateStatus::accepted) {
    throw std::runtime_error("closed-loop envelope context setup failed");
  }
  const auto locked = manager.getValidated(key, {1'000});
  if (!locked.has_value()) {
    throw std::runtime_error("closed-loop envelope lock failed");
  }
  return *locked;
}

PathBoundary start_boundary(const PlanningCycleStart& start) {
  return {start.robot_state.pose.x_m,
          start.robot_state.pose.y_m,
          start.robot_state.pose.heading_rad,
          start.robot_state.curvature_per_m,
          PathBoundarySource::synchronized_actual_state,
          start.robot_state.pose.timestamp,
          start.robot_state.curvature_timestamp,
          start.robot_state.sequence_number};
}

PathBoundary end_boundary(const GeometricPath& path) {
  const PathPoint& point = path.points.back();
  return {point.x_m, point.y_m, point.heading_rad, point.curvature_per_m,
          PathBoundarySource::planned_goal, {}, {}, 0U};
}

PathCandidateVerificationContext path_verification_context(
    const LockedPlanningCycleContext& context) {
  PathCandidateVerificationContext result;
  result.map = context.inputs.planning_snapshot.map;
  result.terrain = context.terrain;
  result.robot_operating_area =
      context.inputs.planning_snapshot.robot_operating_area;
  result.robot_relative_obstacle_covariance_m2 = {0.0, 0.0, 0.0, 0.0};
  result.collision_risk_policy = collision_policy();
  result.robot_capability = robot_capability();
  result.track_footprint = track_footprint();
  result.terrain_gradient_risk_policy = gradient_policy();
  result.maximum_sweep_spacing_fraction = 0.5;
  result.operating_area_clearance_m = 0.0;
  result.geometric_curvature_tolerance_per_m = 0.2;
  result.heading_tolerance_rad = 0.01;
  result.curvature_rate_tolerance_per_m2 = 0.1;
  return result;
}

ValidationInputCaptureLimits closed_loop_capture_limits() {
  return {{10'000'000'000}, {10'000'000'000}, {10'000'000'000},
          {10'000'000'000}, {10'000'000'000}, {10'000'000'000},
          {100'000'000}};
}

class OneShotValidationInputSource final : public ValidationInputSource {
 public:
  explicit OneShotValidationInputSource(SynchronizedValidationInputs inputs)
      : revision_(inputs.source_revision) {
    frame_.source_revision = inputs.source_revision;
    frame_.tracker_update_receipt = inputs.tracker_update_receipt;
    frame_.frame.robot_state = std::move(inputs.robot_state);
    frame_.frame.cable_state = std::move(inputs.cable_state);
    frame_.frame.reference_progress = std::move(inputs.reference_progress);
    frame_.frame.cable_telemetry = std::move(inputs.cable_telemetry);
    frame_.frame.execution_tracking_state =
        std::move(inputs.execution_tracking_state);
    frame_.frame.planning_snapshot = std::move(inputs.planning_snapshot);
    frame_.frame.dependencies = std::move(inputs.dependencies);
  }

  std::optional<TrackerSynchronizedFrame>
  advance_trackers_and_capture_frame() override {
    if (consumed_) return std::nullopt;
    consumed_ = true;
    return frame_;
  }

  std::uint64_t revision() const noexcept override { return revision_; }

 private:
  TrackerSynchronizedFrame frame_;
  std::uint64_t revision_{};
  bool consumed_{};
};

ErrorBudget error_budget(const std::size_t point_count,
                         const PlanningDependencyVersions& versions) {
  ErrorBudget result;
  result.touchdown_position_covariance_m2.assign(
      point_count, {0.01, 0.0, 0.0, 0.01});
  result.epsilon_robot = 0.01;
  result.epsilon_terrain_gradient_local = 0.01;
  result.epsilon_point = 0.01;
  result.calibration_dataset_id = "corridor-cal-v1";
  result.terrain_gradient_calibration_dataset_id = "terrain-cal-v1";
  result.terrain_gradient_policy_version =
      versions.terrain_gradient_policy_version;
  result.corridor_risk_policy_version = versions.corridor_risk_policy_version;
  result.cable_model_version = versions.cable_model_version;
  result.uncertainty_envelope_version = versions.uncertainty_envelope_version;
  result.uncertainty_envelope_generator_version =
      versions.uncertainty_envelope_generator_version;
  result.execution_operating_envelope_version =
      versions.execution_operating_envelope_version;
  result.operating_domain_id = versions.operating_domain_id;
  result.covariance_envelope_audit_passed = true;
  return result;
}

Diagnostics diagnostics(const std::uint64_t seed,
                        const PlanningDependencyVersions& versions) {
  Diagnostics result;
  result.schema_version = "planning-cycle/v1";
  result.random_seed = seed;
  result.input_version = "closed-loop-fixture/v1";
  result.unit_system = "SI";
  result.operating_domain_id = kDomain;
  result.risk_semantics = kRiskSemantics;
  result.dependencies = versions;
  return result;
}

class ClosedLoopStages final : public MainPlanningLoopStages {
 public:
  ClosedLoopStages(const ClosedLoopScenario scenario, const std::uint64_t seed,
                   PlanningDependencyVersions versions,
                   const std::uint64_t plan_sequence = 50U,
                   const std::uint64_t lease_sequence = 70U,
                   const std::optional<ClosedLoopInjection> injection =
                       std::nullopt,
                   std::optional<MapSnapshot> map_override = std::nullopt)
      : scenario_(scenario),
        seed_(seed),
        versions_(std::move(versions)),
        plan_sequence_(plan_sequence),
        lease_sequence_(lease_sequence),
        injection_(injection),
        map_override_(std::move(map_override)),
        input_capturer_(closed_loop_capture_limits()),
        locked_envelope_(lock_envelope(envelope_manager_, versions_)) {}

  AlgorithmRuntimeParameterSnapshot capture_runtime_parameters()
      const override {
    return runtime_parameters();
  }

  ValidationInputCaptureResult capture(const MonotonicTime now) override {
    PlanningDependencyVersions captured_versions = versions_;
    if (injection_ == ClosedLoopInjection::cable_model_version_change &&
        captures_ >= 1U) {
      ++captured_versions.cable_model_version;
    }
    SynchronizedValidationInputs captured =
        inputs(100U + captures_, captured_versions, scenario_);
    if (map_override_) {
      captured.planning_snapshot.map = *map_override_;
    }
    captured.captured_at = now;
    captured.robot_state.sequence_number += captures_;
    captured.cable_state.sequence_number += captures_;
    captured.reference_progress.sequence_number += captures_;
    captured.cable_telemetry.sequence_number += captures_;
    captured.execution_tracking_state.sequence_number += captures_;
    captured.tracker_update_receipt.evidence_batch_sequence += captures_;
    captured.tracker_update_receipt.executed_motion_sequence += captures_;
    captured.tracker_update_receipt.cable_telemetry_sequence =
        captured.cable_telemetry.sequence_number;
    captured.tracker_update_receipt.resulting_cable_state_sequence =
        captured.cable_state.sequence_number;
    captured.tracker_update_receipt.resulting_reference_progress_sequence =
        captured.reference_progress.sequence_number;
    if (injection_ == ClosedLoopInjection::out_of_order_message &&
        captures_ == 1U) {
      captured.robot_state.sequence_number -= 2U;
    }
    if (injection_ == ClosedLoopInjection::telemetry_deviation) {
      captured.cable_telemetry.payout_speed_mps = 2.0;
    }
    ++captures_;
    OneShotValidationInputSource source(std::move(captured));
    return input_capturer_.capture(source, now);
  }

  TerrainAnalysisStageResult analyze_terrain(
      const SynchronizedValidationInputs& captured) override {
    TerrainAnalysisStageResult result;
    result.terrain =
        TerrainAnalyzer{}.analyze(captured.planning_snapshot.map,
                                  terrain_config());
    if (scenario_ == ClosedLoopScenario::double_side_detour) {
      for (std::size_t column = 50U; column <= 115U; ++column) {
        for (std::size_t row = 41U; row <= 52U; ++row) {
          result.terrain.surface.cells.at(
              row * captured.planning_snapshot.map.width + column)
              .detrended_roughness_rms_m = 0.02;
        }
      }
    }
    result.valid = result.terrain.source_map_version == versions_.map_version &&
                   result.terrain.analysis_config_version == 6U &&
                   result.terrain.operating_domain_id == kDomain;
    if (!result.valid) result.issues.push_back("TERRAIN_CONTEXT_MISMATCH");
    return result;
  }

  CommitmentValidationStageResult validate_commitment(
      const TimedPath&, const CableState&, const ReferenceProgress&,
      const LockedPlanningCycleContext&) override {
    return {};
  }

  void request_commitment_safety_stop(const CommitmentSafetyCheckResult&,
                                      MonotonicTime) override {}

  void request_controlled_stop(const PlanningFailure& failure,
                               MonotonicTime) override {
    controlled_stop_requested_ = true;
    controlled_stop_reason_ = failure.reason_code;
  }

  HybridAStarPlanningResult search(
      const PlanningCycleStart& start,
      const LockedPlanningCycleContext& context) override {
    HybridAStarPlanningRequest request;
    request.start_state = start.robot_state;
    request.initial_cable_state = start.cable_state;
    request.initial_reference_progress = start.reference_progress;
    request.reference_line = context.inputs.planning_snapshot.reference_line;
    request.cable_context = cable_context(context.inputs);
    request.cable_context.mode = PredictionMode::search;
    request.primitive_sweep_context.map =
        context.inputs.planning_snapshot.map;
    request.primitive_sweep_context.terrain = context.terrain;
    request.primitive_sweep_context.robot_operating_area =
        context.inputs.planning_snapshot.robot_operating_area;
    request.primitive_sweep_context.robot_relative_obstacle_covariance_m2 =
        {0.0, 0.0, 0.0, 0.0};
    request.primitive_sweep_context.collision_risk_policy = collision_policy();
    request.primitive_sweep_context.robot_capability = robot_capability();
    request.primitive_sweep_context.track_footprint = track_footprint();
    request.primitive_sweep_context.terrain_gradient_risk_policy =
        gradient_policy();
    request.primitive_sweep_context.cable_laying_limits = laying_limits();
    request.primitive_sweep_context.cable_history_boundary =
        CableHistoryBoundary::explicit_task_start;
    request.locked_uncertainty_envelope = locked_envelope_;
    const double turn_advance_m = std::sin(0.5) / 0.5;
    const bool detour = scenario_ == ClosedLoopScenario::single_side_detour ||
                        scenario_ == ClosedLoopScenario::double_side_detour;
    const double goal_robot_x_m =
        detour ? 1.0 + 4.0 * turn_advance_m + 3.0 : 5.0;
    const double goal_reference_progress_m = detour ? 7.0 : 4.0;
    MergeGoal goal;
    goal.robot_pose = {goal_robot_x_m, 0.0, 0.0,
                       context.inputs.captured_at};
    goal.cable_lag_angle_rad = 0.0;
    goal.reference_progress_m = goal_reference_progress_m;
    goal.reference_line_version = versions_.reference_line_version;
    goal.touchdown_target_m = {goal_reference_progress_m, 0.0};
    goal.cable_heading_rad = 0.0;
    goal.merge_distance_m = goal_reference_progress_m;
    goal.generation_parameters_version = 17U;
    goal.cable_model_version = versions_.cable_model_version;
    goal.robot_operating_area_version =
        versions_.robot_operating_area_version;
    request.goals = {goal};
    request.planning_timestamp = context.inputs.captured_at;
    request.random_seed = seed_;
    const auto fixed_clock = [] {
      return std::chrono::steady_clock::time_point{};
    };
    return HybridAStarPlanner(
               cable_model_parameters(versions_.cable_model_version),
               make_reference_progress_association_parameters(
                   progress_parameter_config()),
               search_parameters(), corridor_policy(), envelope_manager_,
               fixed_clock)
        .plan(request);
  }

  SmoothingResult smooth(const GeometricPath& raw_path,
                         const PlanningCycleStart& start,
                         const LockedPlanningCycleContext&) override {
    return PathSmoother().smooth(raw_path, start_boundary(start),
                                 end_boundary(raw_path), smoothing_limits());
  }

  TrackabilityResult validate_raw_path_trackability(
      const GeometricPath& raw_path, const PlanningCycleStart& start,
      const LockedPlanningCycleContext&) override {
    return PathSmoother().validateTrackability(
        raw_path, raw_path, start_boundary(start), end_boundary(raw_path),
        smoothing_limits());
  }

  ParameterizationResult parameterize(
      const GeometricPath& geometry, const PlanningCycleStart& start,
      const LockedPlanningCycleContext&) override {
    const TrajectoryInitialState initial{
        start.robot_state.ground_speed_mps, 0.5, 0.0, 40.0};
    return TrajectoryParameterizer([] { return MonotonicTime{1'000}; })
        .parameterize(geometry, initial, execution_envelope(),
                      parameterization_limits());
  }

  TimedPathMergeResult merge_commitment(
      const TimedPath& authorized_prefix, const TimedPath& new_tail,
      const LockedPlanningCycleContext&) override {
    return StabilityManager().merge_timed_paths(
        authorized_prefix, new_tail, {1.0e-9, 1.0e-9, 1.0e-9},
        {1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9},
        [](const TimedPath& complete) { return validate(complete).valid; });
  }

  PathCandidateVerificationResult verify_complete_robot_path(
      const TimedPath& complete_path, const PlanningCycleStart& start,
      const LockedPlanningCycleContext& context) override {
    return PathCandidateVerifier().verify(
        complete_path.geometry, start_boundary(start),
        end_boundary(complete_path.geometry), smoothing_limits(),
        path_verification_context(context));
  }

  TimedCableCandidateResult verify_cable(
      const TimedPath& complete_path,
      const CableState& synchronized_actual_cable_state,
      const LockedPlanningCycleContext& context) override {
    TimedCableCandidateInput input;
    input.initial_cable_state = synchronized_actual_cable_state;
    input.robot_path = complete_path;
    input.cable_context = cable_context(
        context.inputs,
        scenario_ == ClosedLoopScenario::covariance_envelope_breach);
    input.cable_context.robot_uncertainty_profile.clear();
    const double pose_variance =
        scenario_ == ClosedLoopScenario::covariance_envelope_breach
            ? 100.0
            : 0.0025;
    for (const ExecutionSample& sample :
         complete_path.execution_profile.samples) {
      input.cable_context.robot_uncertainty_profile.push_back(
          {sample.arc_length_m,
           {{pose_variance, 0.0, 0.0, pose_variance}, 0.0, 0.0, 0.01},
           0.002});
    }
    input.reference_line = context.inputs.planning_snapshot.reference_line;
    input.corridor_policy = corridor_policy();
    const CablePrediction prediction =
        CableModel(cable_model_parameters(versions_.cable_model_version))
            .predict(input.initial_cable_state, complete_path,
                     input.cable_context);
    input.reference_progress_m = prediction.robot_arc_length_profile_m;
    input.interval_bound_certificate = {
        versions_.uncertainty_envelope_version, std::vector<double>(
                 input.reference_progress_m.empty()
                     ? 0U
                     : input.reference_progress_m.size() - 1U,
                 0.0)};
    input.terrain = context.terrain;
    input.laying_limits = laying_limits();
    input.history_boundary = CableHistoryBoundary::explicit_task_start;
    input.reference_is_deterministic = true;
    input.covariance_includes_coordinate_transform_error = true;
    input.envelope_audit_tolerance_m = 0.0;
    input.evaluation_timestamp = context.inputs.captured_at;
    input.locked_envelope = locked_envelope_;
    return TimedCableCandidateVerifier(
               CableModel(cable_model_parameters(
                   versions_.cable_model_version)),
               &envelope_manager_)
        .validate(input);
  }

  PlanningCandidateMetadata assemble_candidate_metadata(
      const PlanningCycleRequest&, const PlanningCycleStart&,
      const LockedPlanningCycleContext&,
      const HybridAStarPlanningResult& search_result, const SmoothingResult&,
      const ParameterizationResult&,
      const TimedCableCandidateResult& cable_result) override {
    PlanningCandidateMetadata result;
    result.sequence_number = plan_sequence_;
    result.timestamp = {1'000};
    result.validity_duration = {1'000'000'000};
    result.path_cost = search_result.diagnostics.solution_cost;
    result.error_budget = error_budget(
        cable_result.cable_prediction->touchdown_path.points.size(), versions_);
    result.error_budget.epsilon_point = cable_result.corridor_result.epsilon_point;
    result.error_budget.epsilon_robot = collision_policy().epsilon_robot;
    result.error_budget.epsilon_terrain_gradient_local =
        gradient_policy().epsilon_local;
    result.error_budget.calibration_dataset_id =
        cable_result.corridor_result
            .residual_distribution_calibration_dataset_id;
    result.error_budget.reference_is_deterministic =
        cable_result.corridor_result.reference_is_deterministic;
    result.error_budget.path_joint_risk_implemented =
        cable_result.corridor_result.path_joint_risk_implemented;
    if (cable_result.cable_prediction->touchdown_covariance_profile_m2) {
      result.error_budget.touchdown_position_covariance_m2 =
          *cable_result.cable_prediction->touchdown_covariance_profile_m2;
    }
    result.diagnostics = diagnostics(seed_, versions_);
    return result;
  }

  PlanValidityEvaluation revalidate_plan(
      const PlanningResult& plan, const PlanValidationTarget target,
      const SynchronizedValidationInputs& latest,
      const MonotonicTime now) override {
    PlanningResultPublisher local_publisher;
    const PlanningResultPublication publication = local_publisher.publish(plan);
    if (!publication.published()) return {};
    static_cast<void>(envelope_manager_.registerDependentPlan(
        plan.sequence_number, locked_envelope_, now));
    PlanValidityEvaluatorConfig config;
    config.version = 19U;
    config.parameter_profile_id = "closed-loop-level1-v1";
    config.operating_domain_id = kDomain;
    config.maximum_reuse_duration = {3'000'000'000};
    config.input_limits.robot_state_max_age = {5'000'000'000};
    config.input_limits.cable_state_max_age = {5'000'000'000};
    config.input_limits.reference_progress_max_age = {5'000'000'000};
    config.input_limits.cable_telemetry_max_age = {5'000'000'000};
    config.input_limits.execution_tracking_max_age = {5'000'000'000};
    config.input_limits.map_max_age = {5'000'000'000};
    config.input_limits.synchronization_tolerance = {100'000'000};
    config.envelope_validity_margin = {0};
    config.position_tolerance_m = 0.05;
    config.heading_tolerance_rad = 0.05;
    config.curvature_tolerance_per_m = 0.05;
    config.maximum_ground_speed_tracking_error_mps = 0.1;
    config.maximum_ground_acceleration_tracking_error_mps2 = 0.1;
    config.stopping_safety_margin_m = 0.1;
    config.last_issued_lease_sequence = lease_sequence_ - 1U;
    PlanValidityContext validation;
    validation.terrain = TerrainAnalyzer{}.analyze(
        latest.planning_snapshot.map, terrain_config());
    validation.cable_context = cable_context(latest);
    validation.cable_context.robot_uncertainty_profile.clear();
    for (const ExecutionSample& sample :
         plan.robot_trajectory.execution_profile.samples) {
      validation.cable_context.robot_uncertainty_profile.push_back(
          {sample.arc_length_m,
           {{0.0025, 0.0, 0.0, 0.0025}, 0.0, 0.0, 0.01}, 0.002});
    }
    validation.corridor_policy = corridor_policy();
    validation.corridor_interval_bound = {
        versions_.uncertainty_envelope_version, 0.0};
    validation.reference_progress_parameters =
        make_reference_progress_association_parameters(
            progress_parameter_config());
    validation.laying_limits = laying_limits();
    validation.history_boundary = CableHistoryBoundary::explicit_task_start;
    validation.locked_envelope = locked_envelope_;
    validation.envelope_manager = &envelope_manager_;
    const LockedPlanningCycleContext locked_context{latest,
                                                     validation.terrain};
    validation.path_context = path_verification_context(locked_context);
    validation.smoothing_limits = smoothing_limits();
    validation.goal_boundary = end_boundary(plan.robot_trajectory.geometry);
    validation.reference_is_deterministic = true;
    validation.covariance_includes_coordinate_transform_error = true;
    PlanValidityEvaluator evaluator(
        CableModel(cable_model_parameters(versions_.cable_model_version)),
        config);
    if (target == PlanValidationTarget::publication_candidate) {
      return evaluator.validatePublicationCandidate(
          *publication.result, latest, validation, now);
    }
    return evaluator.validateRemainingPlan(*publication.result, latest,
                                           validation, now);
  }

  [[nodiscard]] bool controlled_stop_requested() const noexcept {
    return controlled_stop_requested_;
  }

  [[nodiscard]] const std::string& controlled_stop_reason() const noexcept {
    return controlled_stop_reason_;
  }

 private:
  ClosedLoopScenario scenario_{ClosedLoopScenario::flat_straight};
  std::uint64_t seed_{};
  PlanningDependencyVersions versions_;
  std::uint64_t plan_sequence_{};
  std::uint64_t lease_sequence_{};
  std::optional<ClosedLoopInjection> injection_;
  std::optional<MapSnapshot> map_override_;
  std::uint64_t captures_{};
  SynchronizedValidationInputCapturer input_capturer_;
  CableUncertaintyEnvelopeManager envelope_manager_;
  LockedCableUncertaintyEnvelope locked_envelope_;
  bool controlled_stop_requested_{};
  std::string controlled_stop_reason_;
};

ActiveExecutionContext active_context(
    const PlanningDependencyVersions& versions) {
  ActiveExecutionContext context;
  apply_dependencies(context, versions);
  return context;
}

ExecutionFeedback feedback_for(const PlanningResult& plan) {
  ExecutionFeedback feedback;
  feedback.plan_sequence_number = plan.sequence_number;
  feedback.execution_profile_version = plan.execution_profile_version;
  feedback.timestamp = {1'100};
  feedback.ground_speed_mps = 0.5;
  feedback.ground_acceleration_mps2 = 0.0;
  feedback.payout_speed_mps = 0.5;
  feedback.payout_acceleration_mps2 = 0.0;
  feedback.tension_n = 40.0;
  feedback.tracked_arc_length_m = 0.0;
  feedback.sequence_number = 1U;
  return feedback;
}

bool path_inside_operating_area(const GeometricPath& path,
                                const RobotOperatingArea& area) {
  const TrackFootprint footprint = track_footprint();
  return std::all_of(path.points.begin(), path.points.end(),
                     [&area, &footprint](const PathPoint& point) {
                       return area.contains_footprint_with_clearance(
                           footprint.polygon,
                           {point.x_m, point.y_m, point.heading_rad, {}}, 0.0);
                     });
}

double endpoint_lateral_offset(const PathPoint& point,
                               const ReferenceLine& reference) {
  const std::vector<ReferenceProjection> projections =
      reference.local_projection_candidates(
          {point.x_m, point.y_m}, reference.points.front().arc_length_m,
          reference.points.back().arc_length_m);
  if (projections.empty()) return std::numeric_limits<double>::infinity();
  return std::min_element(
             projections.begin(), projections.end(),
             [](const ReferenceProjection& left,
                const ReferenceProjection& right) {
               return left.distance_m < right.distance_m;
             })
      ->distance_m;
}

double maximum_lateral_offset(const GeometricPath& path,
                              const ReferenceLine& reference) {
  double maximum = 0.0;
  for (const PathPoint& point : path.points) {
    maximum =
        std::max(maximum, endpoint_lateral_offset(point, reference));
  }
  return maximum;
}

std::string status_name(const PlanningCycleStatus status) {
  switch (status) {
    case PlanningCycleStatus::success:
      return "success";
    case PlanningCycleStatus::current_plan_reused:
      return "current_plan_reused";
    case PlanningCycleStatus::commitment_overridden:
      return "commitment_overridden";
    case PlanningCycleStatus::covariance_envelope_breached:
      return "covariance_envelope_breached";
    case PlanningCycleStatus::input_invalid:
      return "input_invalid";
    case PlanningCycleStatus::commitment_invalid:
      return "commitment_invalid";
    case PlanningCycleStatus::terrain_analysis_failed:
      return "terrain_analysis_failed";
    case PlanningCycleStatus::robot_path_validation_failed:
      return "robot_path_validation_failed";
    case PlanningCycleStatus::search_failed:
      return "search_failed";
    case PlanningCycleStatus::smoothing_failed:
      return "smoothing_failed";
    case PlanningCycleStatus::parameterization_failed:
      return "parameterization_failed";
    case PlanningCycleStatus::cable_validation_failed:
      return "cable_validation_failed";
    case PlanningCycleStatus::candidate_invalid:
      return "candidate_invalid";
    case PlanningCycleStatus::candidate_invalidated:
      return "candidate_invalidated";
    case PlanningCycleStatus::decision_rejected:
      return "decision_rejected";
    case PlanningCycleStatus::lease_invalid:
      return "lease_invalid";
    case PlanningCycleStatus::publication_failed:
      return "publication_failed";
    case PlanningCycleStatus::cycle_timeout:
      return "cycle_timeout";
  }
  return "unknown";
}

ScoutCoordinationParameters scout_parameters() {
  ScoutCoordinationParameters parameters;
  parameters.parameter_profile_id = "closed-loop-level1-v1";
  parameters.operating_domain_id = kDomain;
  parameters.minimum_map_confidence = 0.5;
  parameters.sample_interval_m = 1.0;
  parameters.merge_distance_m = 0.1;
  parameters.minimum_safe_distance_m = 2.0;
  parameters.planning_lead_time_s = 5.0;
  parameters.average_velocity_mps = 1.0;
  parameters.hysteresis_distance_m = 0.25;
  parameters.hysteresis_time_s = 0.5;
  parameters.policy_version = 36U;
  parameters.sensor_coverage_radius_m = 1.5;
  parameters.scout_corridor_half_width_m = 3.0;
  parameters.communication_max_distance_m = 50.0;
  parameters.desired_scout_distance_m = 20.0;
  parameters.continue_scout_distance_m = 40.0;
  parameters.stop_scout_distance_m = 45.0;
  parameters.blocking_priority_weight = 100.0;
  parameters.information_value_weight = 10.0;
  parameters.forward_progress_weight = 1.0;
  parameters.arrival_cost_weight = 1.0;
  parameters.request_timeout = {30'000'000'000};
  return parameters;
}

PlanningStateMachineConfig closed_loop_state_machine_config() {
  PlanningStateMachineConfig result;
  result.version = 37U;
  result.parameter_profile_id = "closed-loop-level1-v1";
  result.operating_domain_id = kDomain;
  result.planning_period = {1'000'000'000};
  result.maximum_consecutive_failures = 3U;
  result.short_communication_outage_limit = {5'000'000'000};
  result.medium_communication_outage_limit = {15'000'000'000};
  return result;
}

SafeStopContext closed_loop_safe_stop() {
  SafeStopContext result;
  result.current_ground_speed_mps = 0.5;
  result.maximum_braking_deceleration_mps2 = 2.0;
  result.terrain_limited_braking_deceleration_mps2 = 1.0;
  result.remaining_safe_distance_m = 3.0;
  result.safety_margin_m = 0.1;
  result.control_reaction_time = {100'000'000};
  result.terrain_braking_model_certified = true;
  return result;
}

bool has_directive(const PlanningDecision& decision,
                   const PlanningDirective directive) {
  return std::find(decision.directives.begin(), decision.directives.end(),
                   directive) != decision.directives.end();
}

ReferenceLine scout_reference() {
  return make_reference_line(4U, "world", {{0.0, 0.0}, {8.0, 0.0}});
}

MapSnapshot scout_map(const std::uint64_t sequence, const bool resolved) {
  return scenario_map(resolved ? ClosedLoopScenario::flat_straight
                               : ClosedLoopScenario::unknown_gap,
                      dependencies(sequence));
}

struct PlanningCycleExecution {
  ClosedLoopCycleReport report;
  bool experiment_record_valid{};
  bool controlled_stop_channel_requested{};
  std::optional<std::uint64_t> revoked_lease_sequence;
  std::optional<PlanningResult> plan;
  std::optional<PlanValidationLease> lease;
  std::optional<TimedPath> remaining_path;
};

PlanningCycleExecution execute_planning_cycle(
    const ClosedLoopScenario scenario, const std::uint64_t seed,
    const PlanningDependencyVersions& versions,
    const std::uint64_t cycle_sequence, const std::int64_t time_ns,
    const std::string& event, const std::uint64_t plan_sequence,
    const std::uint64_t lease_sequence,
    const std::optional<ClosedLoopInjection> injection = std::nullopt,
    ExecutionLeaseMonitor* shared_lease_monitor = nullptr,
    AuthorizedPlanningResultPublisher* shared_publisher = nullptr,
    const MapSnapshot* map_override = nullptr) {
  ClosedLoopStages stages(scenario, seed, versions, plan_sequence,
                          lease_sequence, injection,
                          map_override ? std::optional<MapSnapshot>(*map_override)
                                       : std::nullopt);
  AuthorizedPlanningResultPublisher local_publisher;
  AuthorizedPlanningResultPublisher& publisher =
      shared_publisher ? *shared_publisher : local_publisher;
  ExecutionLeaseMonitor local_lease_monitor;
  ExecutionLeaseMonitor& lease_monitor = shared_lease_monitor
                                             ? *shared_lease_monitor
                                             : local_lease_monitor;
  MonotonicTime now{time_ns};
  MainPlanningLoop loop(stages, publisher, lease_monitor,
                        [&now] { return now; });
  PlanningCycleRequest request;
  request.cycle_sequence = cycle_sequence;
  request.random_seed = seed;
  request.triggered_at = now;
  const PlanningCycleResult result = loop.run_cycle(request);

  ClosedLoopCycleReport cycle;
  cycle.cycle_sequence = request.cycle_sequence;
  cycle.time_ns = now.nanoseconds;
  cycle.executed_stages = result.diagnostics.stages.size();
  cycle.planning_status = status_name(result.status);
  cycle.planning_state =
      std::string(underwater_planner::core::to_string(result.state));
  cycle.event = event;
  switch (scenario) {
    case ClosedLoopScenario::single_side_detour:
    case ClosedLoopScenario::double_side_detour:
      cycle.terrain_condition = "obstacle";
      break;
    case ClosedLoopScenario::traversable_slope:
      cycle.terrain_condition = "slope";
      break;
    case ClosedLoopScenario::traversable_step:
      cycle.terrain_condition = "step";
      break;
    case ClosedLoopScenario::flat_straight:
    case ClosedLoopScenario::route_deviation_recovery:
    case ClosedLoopScenario::unknown_gap:
    case ClosedLoopScenario::covariance_envelope_breach:
      cycle.terrain_condition = "flat";
      break;
  }
  cycle.planning_succeeded = result.succeeded();
  cycle.controlled_stop_required = result.controlled_stop_required;
  cycle.replan_required = result.urgent_replan_required;
  cycle.real_search_executed =
      result.artifacts.search &&
      result.artifacts.search->diagnostics.search_parameter_version == 32U &&
      result.artifacts.search->diagnostics.expanded_state_count > 0U;
  cycle.independent_robot_path_validation_executed =
      result.artifacts.complete_robot_path_validation &&
      result.artifacts.complete_robot_path_validation->collision
              .evaluated_sweep_poses > 0U;
  cycle.cable_prediction_executed =
      result.artifacts.cable_validation &&
      result.artifacts.cable_validation->cable_prediction &&
      !result.artifacts.cable_validation->cable_prediction->state_profile.empty();
  cycle.plan_revalidation_executed =
      result.artifacts.candidate_revalidation &&
      result.artifacts.candidate_revalidation->evaluator_config_version == 19U;
  if (result.initial_inputs) {
    cycle.source_revision = result.initial_inputs->source_revision;
    cycle.map_sequence =
        result.initial_inputs->dependencies.map_version.sequence_number;
    cycle.cable_model_version =
        result.initial_inputs->dependencies.cable_model_version;
  }

  std::optional<PlanningResult> published_plan;
  std::optional<PlanValidationLease> published_lease;
  std::optional<TimedPath> published_remaining;
  if (result.publication.has_value()) {
    const PlanningResult& plan = result.publication->plan.value();
    published_plan = plan;
    published_lease = result.publication->lease;
    published_remaining = *result.publication->remaining_path;
    cycle.plan_sequence = plan.sequence_number;
    cycle.lease_sequence = result.publication->lease.lease_sequence;
    const ReferenceLine& reference =
        result.initial_inputs->planning_snapshot.reference_line;
    cycle.maximum_robot_lateral_offset_m =
        maximum_lateral_offset(plan.robot_trajectory.geometry, reference);
    cycle.maximum_touchdown_lateral_offset_m =
        maximum_lateral_offset(plan.cable_path, reference);
    cycle.route_deviation_recovered =
        scenario == ClosedLoopScenario::route_deviation_recovery &&
        endpoint_lateral_offset(
            plan.robot_trajectory.geometry.points.front(), reference) >= 0.3 &&
        endpoint_lateral_offset(
            plan.robot_trajectory.geometry.points.back(), reference) <= 0.05;
    cycle.invariants.robot_operating_area =
        result.initial_inputs &&
        path_inside_operating_area(
            plan.robot_trajectory.geometry,
            result.initial_inputs->planning_snapshot.robot_operating_area) &&
        result.artifacts.complete_robot_path_validation &&
        result.artifacts.complete_robot_path_validation->valid;
    cycle.invariants.robot_operating_area_evidence =
        "captured_operating_area+path_candidate_verifier";
    cycle.invariants.robot_operating_area_disposition =
        cycle.invariants.robot_operating_area
            ? ClosedLoopInvariantDisposition::satisfied
            : ClosedLoopInvariantDisposition::failed;
    cycle.invariants.cable_corridor =
        plan.corridor_result.validity == CorridorEvaluationValidity::valid &&
        plan.corridor_result.hard_feasible;
    cycle.invariants.cable_corridor_evidence =
        "timed_cable_candidate_verifier.pointwise_corridor";
    cycle.invariants.cable_corridor_disposition =
        cycle.invariants.cable_corridor
            ? ClosedLoopInvariantDisposition::satisfied
            : ClosedLoopInvariantDisposition::failed;
    cycle.invariants.cable_mechanical_constraints =
        plan.cable_laying_result.valid &&
        plan.cable_laying_result.hard_feasible;
    cycle.invariants.cable_mechanical_constraints_evidence =
        "timed_cable_candidate_verifier.cable_laying";
    cycle.invariants.cable_mechanical_constraints_disposition =
        cycle.invariants.cable_mechanical_constraints
            ? ClosedLoopInvariantDisposition::satisfied
            : ClosedLoopInvariantDisposition::failed;
    cycle.invariants.dependency_versions =
        plan.dependencies() == result.publication->lease.dependencies() &&
        result.decision_inputs &&
        plan.dependencies() == result.decision_inputs->dependencies;
    cycle.invariants.dependency_versions_evidence =
        "candidate+decision_snapshot+lease_dependency_equality";
    cycle.invariants.dependency_versions_disposition =
        cycle.invariants.dependency_versions
            ? ClosedLoopInvariantDisposition::satisfied
            : ClosedLoopInvariantDisposition::failed;

    const ExecutionAuthorization authorization = lease_monitor.evaluate(
        plan, *result.publication->remaining_path, result.publication->lease,
        active_context(plan.dependencies()), feedback_for(plan),
        MonotonicTime{time_ns + 100});
    cycle.command_authorized = authorization.authorized();
    cycle.invariants.execution_lease = cycle.command_authorized;
    cycle.invariants.execution_lease_evidence =
        authorization.authorized() ? "execution_lease_monitor.authorized"
                                   : authorization.reason_code;
    cycle.invariants.execution_lease_disposition =
        cycle.invariants.execution_lease
            ? ClosedLoopInvariantDisposition::satisfied
            : ClosedLoopInvariantDisposition::failed;
    if (!authorization.authorized()) {
      cycle.diagnostics.push_back(authorization.reason_code);
    }
  } else if (result.controlled_stop_required) {
    const bool reached_parameterization =
        result.artifacts.parameterization &&
        result.artifacts.parameterization->trajectory.has_value();
    const bool stopped_without_publication =
        stages.controlled_stop_requested() && !result.publication.has_value();
    if (result.initial_inputs) {
      cycle.invariants.robot_operating_area =
          reached_parameterization
              ? path_inside_operating_area(
                    result.artifacts.parameterization->trajectory->geometry,
                    result.initial_inputs->planning_snapshot.robot_operating_area)
              : result.initial_inputs->planning_snapshot.robot_operating_area
                    .contains_footprint_with_clearance(
                        track_footprint().polygon,
                        result.initial_inputs->robot_state.pose, 0.0);
      cycle.invariants.robot_operating_area_disposition =
          cycle.invariants.robot_operating_area
              ? ClosedLoopInvariantDisposition::satisfied
              : ClosedLoopInvariantDisposition::failed;
      cycle.invariants.robot_operating_area_evidence =
          reached_parameterization
              ? "current_cycle_operating_area+parameterized_path"
              : "current_cycle_operating_area+captured_actual_footprint";
    } else {
      cycle.invariants.robot_operating_area = stopped_without_publication;
      cycle.invariants.robot_operating_area_disposition =
          ClosedLoopInvariantDisposition::safe_not_applicable;
      cycle.invariants.robot_operating_area_evidence =
          "capture_rejected_before_area_use+controlled_stop_channel";
    }

    const bool cable_constraints_evaluated =
        result.artifacts.cable_validation &&
        result.artifacts.cable_validation->cable_prediction &&
        !result.artifacts.cable_validation->cable_prediction->state_profile
             .empty();
    if (cable_constraints_evaluated) {
      const bool corridor_feasible =
          result.artifacts.cable_validation->corridor_result.validity ==
              CorridorEvaluationValidity::valid &&
          result.artifacts.cable_validation->corridor_result.hard_feasible;
      cycle.invariants.cable_corridor =
          corridor_feasible ||
          (stopped_without_publication &&
           !result.artifacts.cable_validation->valid);
      cycle.invariants.cable_corridor_disposition =
          cycle.invariants.cable_corridor
              ? ClosedLoopInvariantDisposition::satisfied
              : ClosedLoopInvariantDisposition::failed;
      cycle.invariants.cable_corridor_evidence = corridor_feasible
          ? "timed_cable_candidate_verifier.pointwise_corridor"
          : "timed_cable_candidate_verifier.rejected+controlled_stop_channel";

      const bool mechanics_feasible =
          result.artifacts.cable_validation->laying_result.valid &&
          result.artifacts.cable_validation->laying_result.hard_feasible;
      cycle.invariants.cable_mechanical_constraints =
          mechanics_feasible ||
          (stopped_without_publication &&
           !result.artifacts.cable_validation->valid);
      cycle.invariants.cable_mechanical_constraints_disposition =
          cycle.invariants.cable_mechanical_constraints
              ? ClosedLoopInvariantDisposition::satisfied
              : ClosedLoopInvariantDisposition::failed;
      cycle.invariants.cable_mechanical_constraints_evidence =
          mechanics_feasible
              ? "timed_cable_candidate_verifier.cable_laying"
              : "timed_cable_candidate_verifier.rejected+controlled_stop_channel";
    } else {
      cycle.invariants.cable_corridor = stopped_without_publication;
      cycle.invariants.cable_corridor_disposition =
          ClosedLoopInvariantDisposition::safe_not_applicable;
      cycle.invariants.cable_corridor_evidence =
          result.artifacts.cable_validation
              ? "cable_prediction_rejected_before_corridor+controlled_stop_channel"
              : "no_cable_candidate+controlled_stop_channel";
      cycle.invariants.cable_mechanical_constraints =
          stopped_without_publication;
      cycle.invariants.cable_mechanical_constraints_disposition =
          ClosedLoopInvariantDisposition::safe_not_applicable;
      cycle.invariants.cable_mechanical_constraints_evidence =
          result.artifacts.cable_validation
              ? "cable_prediction_rejected_before_mechanics+controlled_stop_channel"
              : "no_cable_candidate+controlled_stop_channel";
    }

    if (result.initial_inputs) {
      cycle.invariants.dependency_versions =
          result.initial_inputs->dependencies == versions;
      cycle.invariants.dependency_versions_disposition =
          cycle.invariants.dependency_versions
              ? ClosedLoopInvariantDisposition::satisfied
              : ClosedLoopInvariantDisposition::failed;
      cycle.invariants.dependency_versions_evidence =
          "current_cycle_snapshot_matches_stage_dependencies";
    } else {
      cycle.invariants.dependency_versions = stopped_without_publication;
      cycle.invariants.dependency_versions_disposition =
          ClosedLoopInvariantDisposition::safe_not_applicable;
      cycle.invariants.dependency_versions_evidence =
          "snapshot_capture_rejected_before_dependency_use";
    }

    if (result.replay_initial_authorization) {
      cycle.invariants.execution_lease =
          result.revoked_lease_sequence ==
              result.replay_initial_authorization->lease.lease_sequence &&
          lease_monitor.isRevoked(
              result.replay_initial_authorization->lease.lease_sequence) &&
          stopped_without_publication;
      cycle.invariants.execution_lease_disposition =
          cycle.invariants.execution_lease
              ? ClosedLoopInvariantDisposition::satisfied
              : ClosedLoopInvariantDisposition::failed;
      cycle.invariants.execution_lease_evidence =
          "main_planning_loop.revoked_active_lease_before_controlled_stop";
    } else {
      cycle.invariants.execution_lease = stopped_without_publication;
      cycle.invariants.execution_lease_disposition =
          ClosedLoopInvariantDisposition::safe_not_applicable;
      cycle.invariants.execution_lease_evidence =
          "main_planning_loop.no_publication+controlled_stop_channel";
    }
  }
  cycle.diagnostics.insert(cycle.diagnostics.end(), result.issues.begin(),
                           result.issues.end());
  if (result.artifacts.parameterization) {
    cycle.diagnostics.insert(
        cycle.diagnostics.end(),
        result.artifacts.parameterization->diagnostics.issues.begin(),
        result.artifacts.parameterization->diagnostics.issues.end());
  }
  if (result.artifacts.smoothing &&
      result.artifacts.smoothing->status != SmoothingStatus::success) {
    cycle.diagnostics.push_back(
        "smoothing_status=" +
        std::to_string(static_cast<int>(result.artifacts.smoothing->status)) +
        ";solver=" + result.artifacts.smoothing->audit.solver_status);
    if (result.artifacts.search && result.start) {
      const TrackabilityResult raw = PathSmoother().validateTrackability(
          result.artifacts.search->robot_path,
          result.artifacts.search->robot_path, start_boundary(*result.start),
          end_boundary(result.artifacts.search->robot_path),
          smoothing_limits());
      cycle.diagnostics.push_back("raw_trackability=" + raw.reason);
      std::ostringstream residuals;
      residuals << "raw_residuals="
                << raw.residuals.maximum_dynamics_residual << ','
                << raw.residuals.maximum_curvature_audit_residual << ','
                << raw.residuals.maximum_curvature_rate_residual << ','
                << raw.residuals.start_position_residual_m << ','
                << raw.residuals.start_heading_residual_rad << ','
                << raw.residuals.start_curvature_residual_per_m << ','
                << raw.residuals.goal_position_residual_m << ','
                << raw.residuals.goal_heading_residual_rad << ','
                << raw.residuals.goal_curvature_residual_per_m;
      cycle.diagnostics.push_back(residuals.str());
    }
  }
  if (result.artifacts.search &&
      result.artifacts.search->state != PlanningState::success) {
    const HybridAStarSearchDiagnostics& search =
        result.artifacts.search->diagnostics;
    cycle.diagnostics.push_back(
        "search_rejections=collision:" +
        std::to_string(search.collision_rejection_count) +
        ",traversability:" +
        std::to_string(search.traversability_rejection_count) +
        ",laying:" + std::to_string(search.cable_laying_rejection_count));
    if (search.worst_constraint.recorded) {
      cycle.diagnostics.push_back(
          "search_worst=" + search.worst_constraint.reason + ':' +
          std::to_string(search.worst_constraint.constraint_value) + '/' +
          std::to_string(search.worst_constraint.hard_limit));
    }
  }
  if (result.artifacts.cable_validation) {
    cycle.diagnostics.insert(
        cycle.diagnostics.end(),
        result.artifacts.cable_validation->issues.begin(),
        result.artifacts.cable_validation->issues.end());
    for (const DiagnosticEntry& entry :
         result.artifacts.cable_validation->diagnostics) {
      cycle.diagnostics.push_back(entry.code);
    }
    if (result.artifacts.cable_validation->cable_prediction) {
      cycle.diagnostics.insert(
          cycle.diagnostics.end(),
          result.artifacts.cable_validation->cable_prediction->issues.begin(),
          result.artifacts.cable_validation->cable_prediction->issues.end());
    }
  }
  if (result.artifacts.candidate_revalidation) {
    cycle.diagnostics.insert(
        cycle.diagnostics.end(),
        result.artifacts.candidate_revalidation->issues.begin(),
        result.artifacts.candidate_revalidation->issues.end());
    for (const DiagnosticEntry& entry :
         result.artifacts.candidate_revalidation->diagnostics.entries) {
      cycle.diagnostics.push_back(entry.code);
    }
  }
  if (result.artifacts.complete_robot_path_validation &&
      !result.artifacts.complete_robot_path_validation->valid) {
    cycle.diagnostics.insert(
        cycle.diagnostics.end(),
        result.artifacts.complete_robot_path_validation->issues.begin(),
        result.artifacts.complete_robot_path_validation->issues.end());
    cycle.diagnostics.push_back(
        "robot_path_status=" + std::to_string(static_cast<int>(
            result.artifacts.complete_robot_path_validation->status)));
    cycle.diagnostics.push_back(
        "robot_path_heading_residual=" + std::to_string(
            result.artifacts.complete_robot_path_validation->geometry
                .maximum_heading_residual_rad));
    cycle.diagnostics.push_back(
        "robot_path_curvature_residual=" + std::to_string(
            result.artifacts.complete_robot_path_validation->geometry
                .maximum_geometric_curvature_residual_per_m));
  }
  if (result.artifacts.candidate) {
    const ValidationResult candidate_validation =
        validate(*result.artifacts.candidate);
    cycle.diagnostics.insert(cycle.diagnostics.end(),
                             candidate_validation.issues.begin(),
                             candidate_validation.issues.end());
    const ValidationResult trajectory_validation =
        validate(result.artifacts.candidate->robot_trajectory);
    cycle.diagnostics.insert(cycle.diagnostics.end(),
                             trajectory_validation.issues.begin(),
                             trajectory_validation.issues.end());
  }
  if (!result.experiment_record_valid) {
    cycle.diagnostics.insert(cycle.diagnostics.end(),
                             result.experiment_recording_issues.begin(),
                             result.experiment_recording_issues.end());
  }
  return {std::move(cycle), result.experiment_record_valid,
          stages.controlled_stop_requested(), result.revoked_lease_sequence,
          std::move(published_plan), std::move(published_lease),
          std::move(published_remaining)};
}

}  // namespace

std::string_view to_string(const ClosedLoopScenario scenario) noexcept {
  switch (scenario) {
    case ClosedLoopScenario::flat_straight:
      return "flat_straight";
    case ClosedLoopScenario::route_deviation_recovery:
      return "route_deviation_recovery";
    case ClosedLoopScenario::single_side_detour:
      return "single_side_detour";
    case ClosedLoopScenario::double_side_detour:
      return "double_side_detour";
    case ClosedLoopScenario::traversable_slope:
      return "traversable_slope";
    case ClosedLoopScenario::traversable_step:
      return "traversable_step";
    case ClosedLoopScenario::unknown_gap:
      return "unknown_gap";
    case ClosedLoopScenario::covariance_envelope_breach:
      return "covariance_envelope_breach";
  }
  return "unknown";
}

std::string_view to_string(
    const ClosedLoopInvariantDisposition disposition) noexcept {
  switch (disposition) {
    case ClosedLoopInvariantDisposition::not_checked:
      return "not_checked";
    case ClosedLoopInvariantDisposition::satisfied:
      return "satisfied";
    case ClosedLoopInvariantDisposition::safe_not_applicable:
      return "safe_not_applicable";
    case ClosedLoopInvariantDisposition::failed:
      return "failed";
  }
  return "not_checked";
}

std::string_view to_string(const ClosedLoopInjection injection) noexcept {
  switch (injection) {
    case ClosedLoopInjection::advance_time:
      return "advance_time";
    case ClosedLoopInjection::out_of_order_message:
      return "out_of_order_message";
    case ClosedLoopInjection::telemetry_deviation:
      return "telemetry_deviation";
    case ClosedLoopInjection::cable_model_version_change:
      return "cable_model_version_change";
  }
  return "unknown";
}

namespace {

ClosedLoopScenarioReport run_injections(
    const ClosedLoopScenario scenario, const std::uint64_t seed,
    const std::vector<ClosedLoopInjection>& injections) {
  ClosedLoopScenarioReport report;
  report.scenario = scenario;
  report.seed = seed;
  ExecutionLeaseMonitor lease_monitor;
  AuthorizedPlanningResultPublisher publisher;
  PlanningCycleExecution baseline = execute_planning_cycle(
      scenario, seed, dependencies(), 1U, 1'000, "periodic_tick", 50U, 70U,
      std::nullopt, &lease_monitor, &publisher);
  if (!baseline.report.planning_succeeded ||
      !baseline.report.command_authorized || !baseline.plan.has_value() ||
      !baseline.lease.has_value() || !baseline.remaining_path.has_value()) {
    report.issues.push_back("INJECTION_BASELINE_AUTHORIZATION_FAILED");
    return report;
  }
  report.cycles.push_back(baseline.report);
  bool all_injections_safe = baseline.experiment_record_valid;
  std::uint64_t cycle_sequence = 2U;
  for (const ClosedLoopInjection injection : injections) {
    const std::int64_t event_time_ns =
        injection == ClosedLoopInjection::advance_time ? 6'000'000'001
                                                       : 2'000;
    PlanningCycleExecution rejected = execute_planning_cycle(
        scenario, seed, dependencies(), cycle_sequence++, event_time_ns,
        std::string(to_string(injection)), 51U, 71U, injection,
        &lease_monitor, &publisher);
    ExecutionFeedback stale_feedback = feedback_for(*baseline.plan);
    stale_feedback.timestamp = {event_time_ns + 1};
    ++stale_feedback.sequence_number;
    const ExecutionAuthorization stale_rejection = lease_monitor.evaluate(
        *baseline.plan, *baseline.remaining_path, *baseline.lease,
        active_context(baseline.plan->dependencies()), stale_feedback,
        {event_time_ns + 1});
    rejected.report.plan_sequence = baseline.plan->sequence_number;
    rejected.report.lease_sequence = baseline.lease->lease_sequence;
    rejected.report.old_lease_reuse_rejected =
        !stale_rejection.authorized() &&
        stale_rejection.reason_code == "LEASE_ALREADY_REVOKED";
    rejected.report.invariants.execution_lease =
        rejected.report.invariants.execution_lease &&
        rejected.report.old_lease_reuse_rejected &&
        rejected.revoked_lease_sequence == baseline.lease->lease_sequence;
    rejected.report.invariants.execution_lease_disposition =
        rejected.report.invariants.execution_lease
            ? ClosedLoopInvariantDisposition::satisfied
            : ClosedLoopInvariantDisposition::failed;
    rejected.report.invariants.execution_lease_evidence =
        "main_planning_loop.revoked_active_lease+" +
        stale_rejection.reason_code;
    rejected.report.diagnostics.push_back(stale_rejection.reason_code);
    const bool event_observed_by_loop =
        rejected.report.executed_stages > 0U &&
        !rejected.report.planning_succeeded;
    const bool safely_rejected =
        event_observed_by_loop && rejected.report.controlled_stop_required &&
        rejected.controlled_stop_channel_requested &&
        !rejected.report.command_authorized &&
        rejected.report.old_lease_reuse_rejected &&
        rejected.report.invariants.all_passed() &&
        rejected.report.invariants.all_checked();
    all_injections_safe = all_injections_safe && safely_rejected &&
                          rejected.experiment_record_valid;
    report.cycles.push_back(std::move(rejected.report));

    if (injection == ClosedLoopInjection::advance_time) {
      continue;
    }
    if (injection == ClosedLoopInjection::cable_model_version_change) {
      PlanningCycleExecution replanned = execute_planning_cycle(
          scenario, seed, dependencies(7U, 9U), cycle_sequence++, 3'000,
          "model_version_replan", 51U, 71U, std::nullopt, &lease_monitor,
          &publisher);
      replanned.report.replan_required = true;
      all_injections_safe =
          all_injections_safe && replanned.report.planning_succeeded &&
          replanned.report.command_authorized &&
          replanned.report.invariants.all_passed() &&
          replanned.experiment_record_valid;
      report.cycles.push_back(std::move(replanned.report));
    }
  }
  report.passed = all_injections_safe;
  return report;
}

ClosedLoopScenarioReport run_unknown_gap(const std::uint64_t seed) {
  ClosedLoopScenarioReport report;
  report.scenario = ClosedLoopScenario::unknown_gap;
  report.seed = seed;
  ExecutionLeaseMonitor lease_monitor;
  AuthorizedPlanningResultPublisher publisher;
  auto baseline = std::make_unique<PlanningCycleExecution>(
      execute_planning_cycle(ClosedLoopScenario::flat_straight, seed,
                             dependencies(7U), 1U, 1'000,
                             "pre_gap_authorization", 50U, 70U, std::nullopt,
                             &lease_monitor, &publisher));
  if (!baseline->report.planning_succeeded || !baseline->plan ||
      !baseline->lease || !baseline->remaining_path) {
    report.issues.push_back("UNKNOWN_GAP_BASELINE_AUTHORIZATION_FAILED");
    return report;
  }
  auto coordinator =
      std::make_unique<ScoutCoordinator>(scout_parameters());
  const ReferenceLine reference = scout_reference();
  const auto unknown =
      std::make_unique<const MapSnapshot>(scout_map(8U, false));
  const InformationGapScanResult scan =
      coordinator->identify_gaps_result(reference, *unknown, 7.0);
  if (scan.validity != ScoutGapScanValidity::valid || scan.gaps.empty()) {
    report.issues.push_back("INFORMATION_GAP_SCAN_FAILED");
    return report;
  }
  GapUrgencyAssessment urgency;
  urgency.urgency = GapUrgency::blocking;
  urgency.distance_to_gap_m = scan.gaps.front().start_progress_m;
  urgency.blocks_planning_window = true;
  urgency.recommended_action = "STOP_AND_WAIT_FOR_MAP";
  const ScoutTargetGenerationResult generated =
      coordinator->generate_scout_target(
          scan.gaps.front(), urgency, reference, {0.0, 0.0, 0.0, {2'200}},
          {1.0, 0.0, 0.0, {2'200}}, 0.0);
  if (!generated.target.has_value()) {
    report.issues.push_back("SCOUT_TARGET_GENERATION_FAILED");
    return report;
  }
  const ScoutRequestIssueResult issued =
      coordinator->issue_scout_request(*generated.target, {2'200});
  if (issued.disposition != ScoutRequestIssueDisposition::issued ||
      !issued.request.has_value()) {
    report.issues.push_back("SCOUT_REQUEST_FAILED");
    return report;
  }

  PlanningStateMachine state_machine(closed_loop_state_machine_config());
  PlanningDecisionContext waiting_context;
  waiting_context.current_lease_live = true;
  waiting_context.safe_stop = closed_loop_safe_stop();
  const PlanningDecision waiting_decision = state_machine.dispatch(
      {PlanningEventType::waiting_for_map, 1U, {2'200},
       "blocking information gap requires scout map update"},
      waiting_context);
  const bool waiting_state_valid =
      waiting_decision.accepted &&
      waiting_decision.state == PlanningState::waiting_map &&
      waiting_decision.action == PlanningAction::controlled_stop &&
      has_directive(waiting_decision,
                    PlanningDirective::revoke_current_lease) &&
      has_directive(waiting_decision,
                    PlanningDirective::request_controlled_stop) &&
      has_directive(waiting_decision, PlanningDirective::request_scout);
  if (!waiting_state_valid) {
    report.issues.push_back("WAITING_MAP_STATE_TRANSITION_FAILED");
    report.issues.push_back(waiting_decision.reason_code);
    return report;
  }

  PlanningCycleExecution waiting = execute_planning_cycle(
      ClosedLoopScenario::unknown_gap, seed, dependencies(8U), 2U, 2'200,
      "scout_requested", 51U, 71U, std::nullopt, &lease_monitor,
      &publisher, unknown.get());
  ExecutionFeedback stale_feedback = feedback_for(*baseline->plan);
  stale_feedback.timestamp = {2'201};
  ++stale_feedback.sequence_number;
  const ExecutionAuthorization stale_rejection = lease_monitor.evaluate(
      *baseline->plan, *baseline->remaining_path, *baseline->lease,
      active_context(baseline->plan->dependencies()), stale_feedback, {2'201});

  waiting.report.planning_status = "waiting_for_map";
  waiting.report.planning_state =
      std::string(underwater_planner::core::to_string(waiting_decision.state));
  waiting.report.scout_requested = has_directive(
      waiting_decision, PlanningDirective::request_scout);
  waiting.report.terrain_condition = "unknown";
  waiting.report.plan_sequence = baseline->plan->sequence_number;
  waiting.report.lease_sequence = baseline->lease->lease_sequence;
  waiting.report.old_lease_reuse_rejected =
      !stale_rejection.authorized() &&
      stale_rejection.reason_code == "LEASE_ALREADY_REVOKED";
  waiting.report.invariants.execution_lease =
      waiting.report.invariants.execution_lease &&
      waiting.report.old_lease_reuse_rejected &&
      waiting.revoked_lease_sequence == baseline->lease->lease_sequence;
  waiting.report.invariants.execution_lease_disposition =
      waiting.report.invariants.execution_lease
          ? ClosedLoopInvariantDisposition::satisfied
          : ClosedLoopInvariantDisposition::failed;
  waiting.report.invariants.execution_lease_evidence =
      "main_planning_loop.revoked_active_lease+" +
      stale_rejection.reason_code;
  waiting.report.diagnostics.push_back(
      "SCOUT_REQUEST_SEQUENCE=" +
      std::to_string(issued.request->request_sequence));
  waiting.report.diagnostics.push_back(waiting_decision.reason_code);
  waiting.report.diagnostics.push_back(stale_rejection.reason_code);
  if (!waiting.controlled_stop_channel_requested ||
      !waiting.report.controlled_stop_required ||
      waiting.report.command_authorized ||
      !waiting.report.invariants.all_passed() ||
      !waiting.report.invariants.all_checked()) {
    report.issues.push_back("UNKNOWN_GAP_DID_NOT_FAIL_CLOSED");
    return report;
  }
  report.cycles.push_back(std::move(waiting.report));

  auto resolved = std::make_unique<MapSnapshot>(scout_map(9U, true));
  resolved->version.timestamp = {3'000};
  for (MapCell& cell : resolved->cells) {
    cell.measurement_timestamp = resolved->version.timestamp;
  }
  PlanningDependencyVersions resolved_versions = dependencies(9U);
  resolved_versions.map_version = resolved->version;
  const ScoutMapUpdateResult update = coordinator->correlate_map_update(
      issued.request->request_sequence, reference, *resolved, {3'000});
  if (update.disposition != ScoutMapUpdateDisposition::completed ||
      !update.invalidate_old_plan || !update.trigger_replanning) {
    report.issues.push_back("SCOUT_MAP_REPLAN_CORRELATION_FAILED");
    report.issues.insert(report.issues.end(), update.issues.begin(),
                         update.issues.end());
    return report;
  }

  PlanningDecisionContext replan_context;
  replan_context.map_sequence = resolved->version.sequence_number;
  replan_context.safe_stop = closed_loop_safe_stop();
  const PlanningDecision replan_decision = state_machine.dispatch(
      {PlanningEventType::new_map, 2U, {3'000},
       "scout map update resolved blocking information gap"},
      replan_context);
  const bool replan_state_valid =
      replan_decision.accepted &&
      replan_decision.state == PlanningState::normal_planning &&
      replan_decision.action == PlanningAction::controlled_stop &&
      has_directive(replan_decision,
                    PlanningDirective::revoke_current_lease) &&
      has_directive(replan_decision,
                    PlanningDirective::request_controlled_stop) &&
      has_directive(replan_decision, PlanningDirective::start_planning);
  if (!replan_state_valid) {
    report.issues.push_back("NEW_MAP_STATE_TRANSITION_FAILED");
    report.issues.push_back(replan_decision.reason_code);
    return report;
  }

  PlanningCycleExecution replanned = execute_planning_cycle(
      ClosedLoopScenario::flat_straight, seed, resolved_versions, 3U, 3'000,
      "map_update_replan", 51U, 71U, std::nullopt, &lease_monitor,
      &publisher, resolved.get());
  replanned.report.map_updated = true;
  replanned.report.replan_required = has_directive(
      replan_decision, PlanningDirective::start_planning);
  replanned.report.diagnostics.push_back(replan_decision.reason_code);
  report.cycles.push_back(std::move(replanned.report));
  report.passed = report.cycles.front().scout_requested &&
                  report.cycles.front().controlled_stop_required &&
                  !report.cycles.front().command_authorized &&
                  report.cycles.front().invariants.all_passed() &&
                  report.cycles.back().planning_succeeded &&
                  report.cycles.back().command_authorized &&
                  report.cycles.back().invariants.all_passed() &&
                  replanned.experiment_record_valid;
  return report;
}

ClosedLoopScenarioReport run_standard(const ClosedLoopScenario scenario,
                                      const std::uint64_t seed) {
  ClosedLoopScenarioReport report;
  report.scenario = scenario;
  report.seed = seed;
  PlanningCycleExecution execution = execute_planning_cycle(
      scenario, seed, dependencies(), 1U, 1'000, "periodic_tick", 50U,
      70U);
  report.cycles.push_back(std::move(execution.report));
  if (scenario == ClosedLoopScenario::covariance_envelope_breach) {
    report.passed =
        report.cycles.front().planning_status ==
            "covariance_envelope_breached" &&
        report.cycles.front().controlled_stop_required &&
        !report.cycles.front().command_authorized &&
        report.cycles.front().lease_sequence == 0U &&
        report.cycles.front().invariants.all_passed() &&
        execution.experiment_record_valid;
  } else {
    report.passed = report.cycles.front().planning_succeeded &&
                    report.cycles.front().command_authorized &&
                    report.cycles.front().invariants.all_passed() &&
                    execution.experiment_record_valid;
  }
  return report;
}

}  // namespace

ClosedLoopScenarioReport DeterministicClosedLoopDriver::run(
    const ClosedLoopScenario scenario, const std::uint64_t seed,
    const std::vector<ClosedLoopInjection>& injections) const {
  if (!injections.empty()) return run_injections(scenario, seed, injections);
  if (scenario == ClosedLoopScenario::unknown_gap) return run_unknown_gap(seed);
  return run_standard(scenario, seed);
}

ClosedLoopRegressionReport DeterministicClosedLoopDriver::run_all(
    const std::uint64_t seed) const {
  ClosedLoopRegressionReport report;
  report.seed = seed;
  const ClosedLoopScenario scenarios[] = {
      ClosedLoopScenario::flat_straight,
      ClosedLoopScenario::route_deviation_recovery,
      ClosedLoopScenario::single_side_detour,
      ClosedLoopScenario::double_side_detour,
      ClosedLoopScenario::traversable_slope,
      ClosedLoopScenario::traversable_step,
      ClosedLoopScenario::unknown_gap,
      ClosedLoopScenario::covariance_envelope_breach,
  };
  report.passed = true;
  std::uint64_t scenario_seed = seed;
  for (const ClosedLoopScenario scenario : scenarios) {
    ClosedLoopScenarioReport scenario_report = run(scenario, scenario_seed++);
    report.passed = report.passed && scenario_report.passed;
    report.scenarios.push_back(std::move(scenario_report));
  }
  const ClosedLoopInjection injections[] = {
      ClosedLoopInjection::advance_time,
      ClosedLoopInjection::out_of_order_message,
      ClosedLoopInjection::telemetry_deviation,
      ClosedLoopInjection::cable_model_version_change,
  };
  for (const ClosedLoopInjection injection : injections) {
    ClosedLoopScenarioReport injection_report =
        run(ClosedLoopScenario::flat_straight, scenario_seed++, {injection});
    report.passed = report.passed && injection_report.passed;
    report.injection_runs.push_back(std::move(injection_report));
  }
  return report;
}

}  // namespace underwater_planner::testing
