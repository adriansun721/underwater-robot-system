#include "underwater_planner/core/planning_result.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using underwater_planner::core::ImmutablePlanningResult;
using underwater_planner::core::PlanningResult;
using underwater_planner::core::PlanningResultPublishStatus;
using underwater_planner::core::PlanningResultPublisher;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

PlanningResult valid_result(std::uint64_t sequence) {
  PlanningResult result;
  result.sequence_number = sequence;
  result.timestamp = {1000};
  result.validity_duration = {1000000};
  result.state = underwater_planner::core::PlanningState::no_solution;
  result.map_version = {"map", 1, {900}, "world"};
  result.reference_line_version = 1;
  result.robot_operating_area_version = 1;
  result.terrain_gradient_policy_version = 1;
  result.corridor_risk_policy_version = 1;
  result.cable_model_version = 1;
  result.uncertainty_envelope_version = 1;
  result.uncertainty_envelope_generator_version = 1;
  result.execution_operating_envelope_version = 1;
  result.execution_profile_version = 1;
  result.operating_domain_id = "test/v1";
  result.cable_corridor_version = 1;
  result.diagnostics.schema_version = "diagnostics/v1";
  result.diagnostics.input_version = "input/v1";
  result.diagnostics.unit_system = "SI";
  result.diagnostics.operating_domain_id = result.operating_domain_id;
  result.diagnostics.risk_semantics =
      "POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE";
  result.diagnostics.dependencies.map_version = result.map_version;
  result.diagnostics.dependencies.reference_line_version = 1;
  result.diagnostics.dependencies.robot_operating_area_version = 1;
  result.diagnostics.dependencies.terrain_gradient_policy_version = 1;
  result.diagnostics.dependencies.corridor_risk_policy_version = 1;
  result.diagnostics.dependencies.cable_model_version = 1;
  result.diagnostics.dependencies.uncertainty_envelope_version = 1;
  result.diagnostics.dependencies.uncertainty_envelope_generator_version = 1;
  result.diagnostics.dependencies.execution_operating_envelope_version = 1;
  result.diagnostics.dependencies.execution_profile_version = 1;
  result.diagnostics.dependencies.operating_domain_id = result.operating_domain_id;
  result.diagnostics.dependencies.cable_corridor_version =
      result.cable_corridor_version;
  result.corridor_result.validity =
      underwater_planner::core::CorridorEvaluationValidity::valid;
  result.corridor_result.hard_feasible = true;
  result.corridor_result.epsilon_point = 0.01;
  result.corridor_result.corridor_risk_policy_version = 1;
  result.corridor_result.reference_line_version = 1;
  result.corridor_result.interval_bound_certificate.version = 1;
  result.corridor_result.evaluation_timestamp = {1000};
  result.terminal_cable_state.timestamp = {1000};
  result.corridor_result.operating_domain_id = result.operating_domain_id;
  result.corridor_result.residual_distribution_calibration_dataset_id =
      "corridor/v1";
  result.corridor_result.covariance_includes_coordinate_transform_error = true;
  result.corridor_result.covariance_envelope_audit_performed = true;
  result.corridor_result.risk_semantics =
      "POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE";
  result.cable_laying_result.valid = true;
  result.cable_laying_result.hard_feasible = true;
  result.cable_laying_result.failure_reasons = {
      underwater_planner::core::CableLayingFailure::none};
  result.cable_laying_result.limits_version = 1;
  result.cable_laying_result.terrain_map_sequence = 1;
  result.cable_laying_result.terrain_analysis_config_version = 1;
  result.cable_laying_result.operating_domain_id = result.operating_domain_id;
  result.cable_laying_result.risk_semantics =
      "CONSERVATIVE_SUPPORT_PROXY:NO_FLEXIBLE_CABLE_DYNAMICS_GUARANTEE";
  result.error_budget.touchdown_position_covariance_m2 = {{0.0, 0.0, 0.0, 0.0}};
  result.error_budget.epsilon_robot = 0.01;
  result.error_budget.epsilon_terrain_gradient_local = 0.01;
  result.error_budget.epsilon_point = 0.01;
  result.error_budget.calibration_dataset_id = "corridor/v1";
  result.error_budget.terrain_gradient_calibration_dataset_id = "terrain/v1";
  result.error_budget.terrain_gradient_policy_version = 1;
  result.error_budget.corridor_risk_policy_version = 1;
  result.error_budget.cable_model_version = 1;
  result.error_budget.uncertainty_envelope_version = 1;
  result.error_budget.uncertainty_envelope_generator_version = 1;
  result.error_budget.execution_operating_envelope_version = 1;
  result.error_budget.operating_domain_id = result.operating_domain_id;
  result.error_budget.covariance_envelope_audit_passed = true;
  return result;
}

void valid_candidate_is_published_as_read_only_copy() {
  PlanningResultPublisher publisher;
  PlanningResult candidate = valid_result(1);
  const auto publication = publisher.publish(candidate);
  require(publication.status == PlanningResultPublishStatus::published,
          "valid candidate was not published");
  require(publication.result.has_value() && publication.result->value().sequence_number == 1,
          "published result is missing or has the wrong sequence");
  candidate.sequence_number = 99;
  require(publication.result->value().sequence_number == 1,
          "published result aliases mutable assembly storage");
  const PlanningResult& published = publication.result->value();
  require(published.map_version == candidate.map_version &&
              published.reference_line_version == 1 &&
              published.robot_operating_area_version == 1 &&
              published.terrain_gradient_policy_version == 1 &&
              published.corridor_risk_policy_version == 1 &&
              published.cable_model_version == 1 &&
              published.uncertainty_envelope_version == 1 &&
              published.uncertainty_envelope_generator_version == 1 &&
              published.execution_operating_envelope_version == 1 &&
              published.execution_profile_version == 1 &&
              published.operating_domain_id == "test/v1",
          "published result dropped dependency version metadata");
  require(published.error_budget.epsilon_robot.has_value() &&
              *published.error_budget.epsilon_robot == 0.01 &&
              published.error_budget.epsilon_terrain_gradient_local.has_value() &&
              *published.error_budget.epsilon_terrain_gradient_local == 0.01 &&
              published.error_budget.epsilon_point.has_value() &&
              *published.error_budget.epsilon_point == 0.01 &&
              published.error_budget.calibration_dataset_id == "corridor/v1" &&
              published.error_budget.terrain_gradient_calibration_dataset_id ==
                  "terrain/v1" &&
              published.error_budget.covariance_envelope_audit_passed,
          "published result dropped error-budget evidence");
  require(publisher.current().has_value() && publisher.last_sequence() == 1,
          "publisher did not retain the immutable current result");
}

void invalid_or_non_monotonic_candidates_are_rejected() {
  PlanningResultPublisher publisher;
  PlanningResult invalid = valid_result(1);
  invalid.map_version.sequence_number = 0;
  const auto rejected = publisher.publish(invalid);
  require(rejected.status == PlanningResultPublishStatus::invalid &&
              !rejected.result.has_value(),
          "incomplete dependency versions were published");

  require(publisher.publish(valid_result(2)).published(),
          "valid second result was not published");
  const auto duplicate = publisher.publish(valid_result(2));
  require(duplicate.status == PlanningResultPublishStatus::sequence_not_monotonic,
          "duplicate sequence was accepted");
  const auto rollback = publisher.publish(valid_result(1));
  require(rollback.status == PlanningResultPublishStatus::sequence_not_monotonic,
          "rolled-back sequence was accepted");
  require(publisher.last_sequence() == 2,
          "rejected publication changed the sequence watermark");
}

void publisher_rejects_failure_results_with_unassembled_nested_times() {
  const auto failure_states = {
      underwater_planner::core::PlanningState::no_solution,
      underwater_planner::core::PlanningState::input_invalid,
      underwater_planner::core::PlanningState::timeout,
  };
  std::uint64_t sequence = 1;
  for (const auto state : failure_states) {
    PlanningResult candidate = valid_result(sequence++);
    candidate.state = state;
    candidate.terminal_cable_state.timestamp = {};
    candidate.corridor_result.evaluation_timestamp = {};
    PlanningResultPublisher publisher;
    const auto publication = publisher.publish(candidate);
    require(publication.status == PlanningResultPublishStatus::invalid &&
                !publication.result.has_value(),
            "publisher accepted a failure result with default nested timestamps");
    require(publisher.last_sequence() == 0,
            "rejected failure result advanced publisher sequence");
  }
}

}  // namespace

int main() {
  try {
    valid_candidate_is_published_as_read_only_copy();
    invalid_or_non_monotonic_candidates_are_rejected();
    publisher_rejects_failure_results_with_unassembled_nested_times();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
