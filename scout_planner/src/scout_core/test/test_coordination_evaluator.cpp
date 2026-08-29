#include "contract_fixture.hpp"
#include "scout_planner/core/coordination_evaluator.hpp"
#include "test_support.hpp"

#include "underwater/contracts/v1/cooperation.pb.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace core = scout_planner::core;
namespace contract = underwater::contracts::v1;
namespace fixture = scout_planner::test_contract_fixture;
using scout_planner::test_support::require;

namespace {

struct Inputs {
  core::MainRobotPrediction prediction;
  core::CoordinationConstraint constraint;
};

Inputs make_inputs() {
  auto prediction = fixture::populated_message<contract::MainRobotPrediction>();
  prediction.set_mission_id(7U);
  prediction.set_mission_version(3U);
  prediction.set_prediction_id("prediction-1");
  prediction.set_prediction_version(4U);
  prediction.mutable_alignment_epoch()->set_status(contract::TIME_SYNC_SYNCHRONIZED);
  prediction.mutable_alignment_epoch()->set_uncertainty_ns(1U);
  auto* interval = prediction.mutable_occupied_intervals(0);
  interval->set_start_offset_ns(0U);
  interval->set_end_offset_ns(10U);
  auto* swept = interval->mutable_swept_volume();
  swept->mutable_start_center()->set_x_m(5.0);
  swept->mutable_start_center()->set_y_m(0.0);
  swept->mutable_start_center()->set_z_m(0.0);
  swept->mutable_end_center()->set_x_m(5.0);
  swept->mutable_end_center()->set_y_m(0.0);
  swept->mutable_end_center()->set_z_m(0.0);
  swept->set_physical_radius_m(0.25);
  swept->set_position_uncertainty_radius_m(0.25);
  swept->set_conservative_occupied_radius_m(0.5);
  fixture::identify_in_place<core::ContractKind::main_robot_prediction>(&prediction);

  auto constraint = fixture::populated_message<contract::ScoutCoordinationConstraint>();
  constraint.set_mission_id(prediction.mission_id());
  constraint.set_mission_version(prediction.mission_version());
  constraint.mutable_mission_content_identity()->CopyFrom(
      prediction.mission_content_identity());
  constraint.set_prediction_id(prediction.prediction_id());
  constraint.set_prediction_version(prediction.prediction_version());
  constraint.mutable_prediction_content_identity()->CopyFrom(
      prediction.prediction_content_identity());
  constraint.set_channel_id(contract::CHANNEL_MAIN_SCOUT_COOP);
  constraint.set_minimum_separation_m(1.0);
  constraint.set_maximum_communication_distance_m(20.0);
  constraint.set_link_assurance_basis(contract::LINK_ASSURANCE_GEOMETRIC_DISTANCE_ONLY);
  fixture::identify_in_place<core::ContractKind::scout_coordination_constraint>(&constraint);
  return {fixture::decode_identified<core::ContractKind::main_robot_prediction>(prediction),
          fixture::decode_identified<core::ContractKind::scout_coordination_constraint>(constraint)};
}

core::CoordinationEvaluationConfig config() {
  core::CoordinationEvaluationConfig value;
  value.now_monotonic_ns = 1'000;
  value.prediction_received_at_monotonic_ns = 1'000;
  value.constraint_received_at_monotonic_ns = 1'000;
  value.prediction_reject_ns = 100;
  value.constraint_reject_ns = 100;
  value.maximum_sync_uncertainty_ns = 10;
  return value;
}

void crossing_is_rejected_continuously() {
  const auto inputs = make_inputs();
  const std::vector<core::CoordinationMotionSample> motion{{0U, {5.0, -5.0, 0.0}},
                                                            {10U, {5.0, 5.0, 0.0}}};
  const auto result = core::DualRobotCoordinationEvaluator::evaluate(
      inputs.prediction, inputs.constraint, motion, config());
  require(!result.has_value() &&
              result.error().code == core::CoordinationFailure::separation_violation,
          "continuous crossing was not rejected");
  require(result.error().earliest_failure_time_offset_ns == 3U,
          "crossing failure time was not deterministic");
}

void safe_motion_reports_margins_and_geometry_only_link() {
  const auto inputs = make_inputs();
  const std::vector<core::CoordinationMotionSample> motion{{0U, {0.0, 5.0, 0.0}},
                                                            {10U, {10.0, 5.0, 0.0}}};
  const auto result = core::DualRobotCoordinationEvaluator::evaluate(
      inputs.prediction, inputs.constraint, motion, config());
  require(result.has_value(), result.error().detail);
  require(result.value().separation_passed && result.value().communication_distance_passed,
          "safe coordinated motion did not pass");
  require(!result.value().calibrated_link_quality_asserted,
          "geometric distance was reported as link quality");
  require(std::abs(result.value().minimum_separation_margin_m - 3.5) < 1.0e-12,
          "minimum separation margin was not conservative");
}

void horizon_stale_and_clock_fail_closed() {
  const auto inputs = make_inputs();
  const std::vector<core::CoordinationMotionSample> too_long{{0U, {0.0, 5.0, 0.0}},
                                                              {11U, {10.0, 5.0, 0.0}}};
  auto stale = config();
  stale.now_monotonic_ns = 2'000;
  auto result = core::DualRobotCoordinationEvaluator::evaluate(
      inputs.prediction, inputs.constraint, too_long, config());
  require(!result.has_value() &&
              result.error().code == core::CoordinationFailure::prediction_horizon_exceeded,
          "prediction horizon was silently truncated");
  result = core::DualRobotCoordinationEvaluator::evaluate(
      inputs.prediction, inputs.constraint,
      std::vector<core::CoordinationMotionSample>{{0U, {0.0, 5.0, 0.0}},
                                                   {10U, {10.0, 5.0, 0.0}}},
      stale);
  require(!result.has_value() && result.error().code == core::CoordinationFailure::dependency_stale,
          "stale coordination input was accepted");
}

void repeated_evaluation_is_byte_deterministic() {
  const auto inputs = make_inputs();
  const std::vector<core::CoordinationMotionSample> motion{{0U, {0.0, 5.0, 0.0}},
                                                            {10U, {10.0, 5.0, 0.0}}};
  const auto first = core::DualRobotCoordinationEvaluator::evaluate(
      inputs.prediction, inputs.constraint, motion, config());
  const auto second = core::DualRobotCoordinationEvaluator::evaluate(
      inputs.prediction, inputs.constraint, motion, config());
  require(first.has_value() && second.has_value(), "deterministic case failed");
  require(first.value().minimum_separation_margin_m ==
              second.value().minimum_separation_margin_m &&
              first.value().minimum_communication_margin_m ==
              second.value().minimum_communication_margin_m,
          "repeated evaluation was not deterministic");
}

}  // namespace

int main() {
  try {
    crossing_is_rejected_continuously();
    safe_motion_reports_margins_and_geometry_only_link();
    horizon_stale_and_clock_fail_closed();
    repeated_evaluation_is_byte_deterministic();
    std::cout << "[pass] coordination evaluator\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "[fail] " << error.what() << '\n';
    return 1;
  }
}
