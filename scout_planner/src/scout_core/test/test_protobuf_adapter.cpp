#include "scout_planner/core/protobuf_adapter.hpp"
#include "contract_fixture.hpp"
#include "test_support.hpp"

#include "underwater/contracts/v1/cooperation.pb.h"
#include "underwater/contracts/v1/planning.pb.h"
#include "underwater/contracts/v1/state.pb.h"

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include <algorithm>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

namespace proto = underwater::contracts::v1;
namespace pb = google::protobuf;
using scout_planner::core::ContractKind;
using scout_planner::core::ProtobufAdapter;
using scout_planner::core::AdapterErrorCode;

constexpr std::uint64_t kSeed = 0x5C0A7B14ULL;
const std::string failure_context =
    scout_planner::test_support::default_failure_context(kSeed);

void fill_identity(proto::ContentIdentity* identity, const char fill) {
  identity->set_sha256(std::string(32, fill));
}

void fill_header(proto::MessageHeader* header) {
  header->set_schema_major(1);
  header->set_schema_minor(0);
  header->set_producer_id("main-nuc");
  header->set_producer_session_id(std::string(16, '\x11'));
  header->set_stream_id(proto::STREAM_SCOUT_MISSION);
  header->set_sequence(7);
  header->set_source_clock_domain_id("main-nuc/boot-7");
  header->set_generated_at_monotonic_ns(1'000);
  header->set_event_id(std::string(16, '\x22'));
  header->set_correlation_id(std::string(16, '\x33'));
  header->mutable_manifest()->set_schema_major(1);
  header->mutable_manifest()->set_schema_minor(0);
  fill_identity(header->mutable_manifest()->mutable_manifest_identity(), '\x44');
}

std::string hash_bytes(const scout_planner::core::Hash256& hash) {
  return std::string(hash.begin(), hash.end());
}

proto::ScoutMission valid_mission() {
  proto::ScoutMission mission;
  fill_header(mission.mutable_header());
  mission.set_mission_id(42);
  mission.mutable_required_region()->add_xyz_m(-4.0);
  mission.mutable_required_region()->add_xyz_m(-3.0);
  mission.mutable_required_region()->add_xyz_m(-20.0);
  mission.mutable_required_region()->add_xyz_m(4.0);
  mission.mutable_required_region()->add_xyz_m(3.0);
  mission.mutable_required_region()->add_xyz_m(-10.0);
  mission.mutable_required_region()->set_frame_id("mission_enu");
  mission.mutable_allowed_scout_region()->CopyFrom(mission.required_region());
  mission.set_required_coverage_ratio(0.95);
  mission.set_required_resolution_m(0.1);
  mission.set_maximum_evidence_age_ns(2'000'000'000ULL);
  mission.set_business_deadline_monotonic_ns(9'000);
  mission.set_urgency(proto::SURVEY_URGENCY_BLOCKING);
  mission.set_minimum_separation_m(2.0);
  mission.set_maximum_communication_distance_m(25.0);
  mission.set_coordination_version(3);
  mission.set_mission_version(5);
  fill_identity(mission.mutable_mission_content_identity(), '\x55');
  const auto identity =
      ProtobufAdapter::canonical_content_identity<ContractKind::scout_mission>(
          mission.SerializeAsString());
  scout_planner::test_support::require(identity.has_value(),
                                       identity.error().message);
  mission.mutable_mission_content_identity()->set_sha256(
      hash_bytes(identity.value()));
  return mission;
}

using scout_planner::test_contract_fixture::refresh_fixture_identities;
using scout_planner::test_contract_fixture::valid_contract;
template <ContractKind Kind>
void require_round_trip(const std::string& schema_name) {
  auto message = valid_contract(schema_name);
  refresh_fixture_identities<Kind>(message.get());
  const auto wire = message->SerializeAsString();
  const auto decoded = ProtobufAdapter::decode<Kind>(wire);
  scout_planner::test_support::require(
      decoded.has_value(), schema_name + ": " + decoded.error().message);
  scout_planner::test_support::require(
      decoded.value().document().schema_name == schema_name,
      schema_name + ": core schema identity changed");
  const auto encoded = ProtobufAdapter::encode(decoded.value());
  scout_planner::test_support::require(
      encoded.has_value(), schema_name + ": " + encoded.error().message);
  scout_planner::test_support::require(encoded.value() == wire,
                                       schema_name + ": round-trip changed bytes");
}

void test_scout_mission_round_trips_without_semantic_loss() {
  const auto mission = valid_mission();
  const std::string wire = mission.SerializeAsString();

  const auto decoded =
      ProtobufAdapter::decode<ContractKind::scout_mission>(wire);
  scout_planner::test_support::require(decoded.has_value(),
                                       decoded.error().message);
  scout_planner::test_support::require(
      decoded.value().document().schema_name == mission.GetTypeName(),
      "core value lost its golden schema identity");

  const auto encoded = ProtobufAdapter::encode(decoded.value());
  scout_planner::test_support::require(encoded.has_value(),
                                       encoded.error().message);

  proto::ScoutMission reparsed;
  scout_planner::test_support::require(reparsed.ParseFromString(encoded.value()),
                                       "adapter emitted invalid protobuf");
  scout_planner::test_support::require(
      reparsed.SerializeAsString() == wire,
      "protobuf -> core -> protobuf changed public fields");
}

void test_all_ticket_14_core_contracts_use_the_golden_schema() {
  require_round_trip<ContractKind::scout_mission>(
      "underwater.contracts.v1.ScoutMission");
  require_round_trip<ContractKind::hybrid_map_snapshot>(
      "underwater.contracts.v1.HybridMapSnapshot");
  require_round_trip<ContractKind::scout_navigation_state>(
      "underwater.contracts.v1.ScoutNavigationState");
  require_round_trip<ContractKind::scout_sensor_geometry>(
      "underwater.contracts.v1.ScoutSensorGeometry");
  require_round_trip<ContractKind::scout_sensor_health_state>(
      "underwater.contracts.v1.ScoutSensorHealthState");
  require_round_trip<ContractKind::scout_current_estimate>(
      "underwater.contracts.v1.ScoutCurrentEstimate");
  require_round_trip<ContractKind::scout_capability_profile>(
      "underwater.contracts.v1.ScoutCapabilityProfile");
  require_round_trip<ContractKind::scout_thruster_health_state>(
      "underwater.contracts.v1.ScoutThrusterHealthState");
  require_round_trip<ContractKind::scout_energy_model_profile>(
      "underwater.contracts.v1.ScoutEnergyModelProfile");
  require_round_trip<ContractKind::scout_energy_state>(
      "underwater.contracts.v1.ScoutEnergyState");
  require_round_trip<ContractKind::main_robot_prediction>(
      "underwater.contracts.v1.MainRobotPrediction");
  require_round_trip<ContractKind::scout_coordination_constraint>(
      "underwater.contracts.v1.ScoutCoordinationConstraint");
  require_round_trip<ContractKind::scout_trajectory_4d>(
      "underwater.contracts.v1.ScoutTrajectory4d");
  require_round_trip<ContractKind::scout_plan_validation_report>(
      "underwater.contracts.v1.ScoutPlanValidationReport");
  require_round_trip<ContractKind::scout_planning_dependencies>(
      "underwater.contracts.v1.ScoutPlanningDependencies");
  require_round_trip<ContractKind::scout_plan>(
      "underwater.contracts.v1.ScoutPlan");
  require_round_trip<ContractKind::scout_planning_result>(
      "underwater.contracts.v1.ScoutPlanningResult");
  require_round_trip<ContractKind::scout_execution_lease>(
      "underwater.contracts.v1.ScoutExecutionLease");
  require_round_trip<ContractKind::scout_authorized_execution_bundle>(
      "underwater.contracts.v1.ScoutAuthorizedExecutionBundle");
}

void test_ticket_15_admission_decision_uses_the_golden_schema() {
  require_round_trip<ContractKind::scout_mission_decision>(
      "underwater.contracts.v1.ScoutMissionDecision");
}

template <typename T>
void require_error(const scout_planner::core::AdapterResult<T>& result,
                   const AdapterErrorCode expected, const std::string& context) {
  scout_planner::test_support::require(!result.has_value(),
                                       context + ": invalid value was accepted");
  scout_planner::test_support::require(
      result.error().code == expected,
      context + ": unexpected error: " + result.error().message);
}

void test_adapter_rejects_missing_unknown_and_non_finite_safety_fields() {
  auto missing = valid_mission();
  missing.clear_required_coverage_ratio();
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_mission>(
          missing.SerializeAsString()),
      AdapterErrorCode::missing_safety_field, "missing coverage ratio");

  auto unknown_enum = valid_mission();
  const auto* urgency = unknown_enum.GetDescriptor()->FindFieldByName("urgency");
  unknown_enum.GetReflection()->SetEnumValue(&unknown_enum, urgency, 99);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_mission>(
          unknown_enum.SerializeAsString()),
      AdapterErrorCode::unknown_enum, "unknown urgency");

  auto non_finite = valid_mission();
  non_finite.set_required_resolution_m(
      std::numeric_limits<double>::quiet_NaN());
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_mission>(
          non_finite.SerializeAsString()),
      AdapterErrorCode::non_finite_number, "non-finite resolution");

  std::string unknown_field = valid_mission().SerializeAsString();
  unknown_field.append("\xa0\x06\x01", 3);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_mission>(unknown_field),
      AdapterErrorCode::unknown_field, "unknown wire field");
}

void test_adapter_centralizes_frame_identity_version_and_boundary_checks() {
  auto wrong_frame = valid_mission();
  wrong_frame.mutable_required_region()->set_frame_id("map");
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_mission>(
          wrong_frame.SerializeAsString()),
      AdapterErrorCode::invalid_frame, "non-canonical world frame");

  auto short_identity = valid_mission();
  short_identity.mutable_mission_content_identity()->set_sha256("short");
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_mission>(
          short_identity.SerializeAsString()),
      AdapterErrorCode::invalid_content_identity, "short content identity");

  scout_planner::core::VersionFloor floors{
      {"underwater.contracts.v1.ScoutMission.mission_version", 6}};
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_mission>(
          valid_mission().SerializeAsString(), {&floors}),
      AdapterErrorCode::version_rollback, "mission version rollback");

  auto reversed_bounds = valid_mission();
  reversed_bounds.mutable_required_region()->set_xyz_m(3, -4.0);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_mission>(
          reversed_bounds.SerializeAsString()),
      AdapterErrorCode::invalid_numeric_boundary, "reversed region bounds");

  auto invalid_ratio = valid_mission();
  invalid_ratio.set_required_coverage_ratio(1.0001);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_mission>(
          invalid_ratio.SerializeAsString()),
      AdapterErrorCode::invalid_numeric_boundary, "coverage ratio above one");

  auto missing_deadline = valid_mission();
  missing_deadline.clear_business_deadline_monotonic_ns();
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_mission>(
          missing_deadline.SerializeAsString()),
      AdapterErrorCode::missing_safety_field, "missing business deadline");

  require_error(
      ProtobufAdapter::decode<ContractKind::scout_mission>(
          valid_mission().SerializeAsString(), {nullptr, 8U}),
      AdapterErrorCode::resource_limit_exceeded,
      "activated message-size limit");
}

void test_navigation_covariance_must_be_symmetric_positive_semidefinite() {
  auto navigation =
      valid_contract("underwater.contracts.v1.ScoutNavigationState");
  const auto* position_field =
      navigation->GetDescriptor()->FindFieldByName("position_covariance_m2");
  auto* covariance =
      navigation->GetReflection()->MutableMessage(navigation.get(), position_field);
  const auto* values = covariance->GetDescriptor()->FindFieldByName("row_major");
  covariance->GetReflection()->SetRepeatedDouble(covariance, values, 1, 0.2);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_navigation_state>(
          navigation->SerializeAsString()),
      AdapterErrorCode::invalid_covariance, "asymmetric covariance");

  navigation = valid_contract("underwater.contracts.v1.ScoutNavigationState");
  covariance =
      navigation->GetReflection()->MutableMessage(navigation.get(), position_field);
  covariance->GetReflection()->SetRepeatedDouble(covariance, values, 0, -0.1);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_navigation_state>(
          navigation->SerializeAsString()),
      AdapterErrorCode::invalid_covariance, "indefinite covariance");

  navigation = valid_contract("underwater.contracts.v1.ScoutNavigationState");
  const auto* attitude_field = navigation->GetDescriptor()->FindFieldByName(
      "attitude_covariance_rad2");
  covariance = navigation->GetReflection()->MutableMessage(navigation.get(),
                                                            attitude_field);
  const auto* frame = covariance->GetDescriptor()->FindFieldByName("frame_id");
  covariance->GetReflection()->SetString(covariance, frame, "mission_enu");
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_navigation_state>(
          navigation->SerializeAsString()),
      AdapterErrorCode::invalid_frame, "attitude covariance world frame");

  navigation = valid_contract("underwater.contracts.v1.ScoutNavigationState");
  covariance = navigation->GetReflection()->MutableMessage(navigation.get(),
                                                            position_field);
  covariance->GetReflection()->SetRepeatedDouble(covariance, values, 1, 5e-10);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_navigation_state>(
          navigation->SerializeAsString()),
      AdapterErrorCode::invalid_covariance,
      "covariance above the normative scaled symmetry tolerance");
}

void test_core_originated_document_is_validated_before_encoding() {
  static_assert(!std::is_default_constructible_v<scout_planner::core::SurveyTask>,
                "default construction would create pseudo-valid safety data");
  const auto decoded = ProtobufAdapter::decode<ContractKind::scout_mission>(
      valid_mission().SerializeAsString());
  scout_planner::test_support::require(decoded.has_value(),
                                       decoded.error().message);
  auto document = decoded.value().document();
  const auto unchanged =
      ProtobufAdapter::create<ContractKind::scout_mission>(document);
  scout_planner::test_support::require(unchanged.has_value(),
                                       unchanged.error().message);
  const auto mission_id =
      std::find_if(document.fields.begin(), document.fields.end(),
                   [](const auto& field) { return field.name == "mission_id"; });
  scout_planner::test_support::require(mission_id != document.fields.end(),
                                       "core mission_id is missing");
  mission_id->values.front() = std::uint64_t{43};

  const auto created =
      ProtobufAdapter::create<ContractKind::scout_mission>(std::move(document));
  require_error(created, AdapterErrorCode::invalid_content_identity,
                "stale identity on core-originated document");
}

std::string hex_hash(const scout_planner::core::Hash256& hash) {
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (const auto byte : hash) {
    output << std::setw(2) << static_cast<unsigned>(byte);
  }
  return output.str();
}

void test_canonical_wire_hash_matches_independent_golden_value() {
  const auto decoded = ProtobufAdapter::decode<ContractKind::scout_mission>(
      valid_mission().SerializeAsString());
  scout_planner::test_support::require(decoded.has_value(),
                                       decoded.error().message);
  const auto actual = hex_hash(decoded.value().canonical_wire_sha256());
  scout_planner::test_support::require(
      actual ==
          "cc06134c30eb06f3f7880b6c01bafe5b71d6f3f15ec01364f4982e155176811d",
      "canonical SHA-256 mismatch; actual=" + actual);
}

void test_failure_results_cannot_carry_candidates_or_illegal_times() {
  auto result =
      valid_contract("underwater.contracts.v1.ScoutPlanningResult");
  const auto* outcome = result->GetDescriptor()->FindFieldByName("outcome");
  const auto* no_solution = outcome->enum_type()->FindValueByNumber(21);
  result->GetReflection()->SetEnum(result.get(), outcome, no_solution);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_planning_result>(
          result->SerializeAsString()),
      AdapterErrorCode::invalid_structure, "failure result with candidate");

  const auto* candidate = result->GetDescriptor()->FindFieldByName("candidate");
  result->GetReflection()->ClearField(result.get(), candidate);
  refresh_fixture_identities<ContractKind::scout_planning_result>(result.get());
  const auto valid_failure =
      ProtobufAdapter::decode<ContractKind::scout_planning_result>(
          result->SerializeAsString());
  scout_planner::test_support::require(valid_failure.has_value(),
                                       valid_failure.error().message);

  const auto* diagnostics =
      result->GetDescriptor()->FindFieldByName("diagnostics");
  result->GetReflection()->ClearField(result.get(), diagnostics);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_planning_result>(
          result->SerializeAsString()),
      AdapterErrorCode::invalid_structure,
      "failure result without a diagnostic");

  const auto* evaluated =
      result->GetDescriptor()->FindFieldByName("evaluated_at_monotonic_ns");
  result->GetReflection()->SetInt64(result.get(), evaluated, -1);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_planning_result>(
          result->SerializeAsString()),
      AdapterErrorCode::invalid_time, "negative failure evaluation time");

  auto noncanonical_outcome =
      valid_contract("underwater.contracts.v1.ScoutPlanningResult");
  noncanonical_outcome->GetReflection()->SetEnumValue(
      noncanonical_outcome.get(), outcome, 2);
  noncanonical_outcome->GetReflection()->ClearField(noncanonical_outcome.get(),
                                                     candidate);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_planning_result>(
          noncanonical_outcome->SerializeAsString()),
      AdapterErrorCode::invalid_structure,
      "known non-planner outcome on ScoutPlanningResult");

  auto early_failure =
      valid_contract("underwater.contracts.v1.ScoutPlanningResult");
  early_failure->GetReflection()->SetEnumValue(early_failure.get(), outcome, 21);
  early_failure->GetReflection()->ClearField(early_failure.get(), candidate);
  const auto* dependencies =
      early_failure->GetDescriptor()->FindFieldByName("dependencies");
  auto* partial = early_failure->GetReflection()->MutableMessage(
      early_failure.get(), dependencies);
  const auto* partial_descriptor = partial->GetDescriptor();
  for (int index = 0; index < partial_descriptor->field_count(); ++index) {
    const auto* field = partial_descriptor->field(index);
    if (field->name() != "mission_id" && field->name() != "mission_version" &&
        field->name() != "mission_content_identity" &&
        field->name() != "dependencies_content_identity") {
      partial->GetReflection()->ClearField(partial, field);
    }
  }
  const auto* root_identity = early_failure->GetDescriptor()->FindFieldByName(
      "result_content_identity");
  early_failure->GetReflection()->ClearField(early_failure.get(), root_identity);
  const auto identified = ProtobufAdapter::canonicalize_and_identify<
      ContractKind::scout_planning_result>(early_failure->SerializeAsString());
  scout_planner::test_support::require(identified.has_value(),
                                       identified.error().message);
  const auto decoded_early_failure =
      ProtobufAdapter::decode<ContractKind::scout_planning_result>(
          identified.value());
  scout_planner::test_support::require(
      decoded_early_failure.has_value(),
      decoded_early_failure.error().message);
}

void test_time_intervals_and_trajectory_segments_are_ordered() {
  auto current = valid_contract("underwater.contracts.v1.ScoutCurrentEstimate");
  const auto* from =
      current->GetDescriptor()->FindFieldByName("valid_from_monotonic_ns");
  current->GetReflection()->SetInt64(current.get(), from, 300);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_current_estimate>(
          current->SerializeAsString()),
      AdapterErrorCode::invalid_time, "reversed current validity interval");

  auto trajectory = valid_contract("underwater.contracts.v1.ScoutTrajectory4d");
  const auto* segments =
      trajectory->GetDescriptor()->FindFieldByName("segments");
  const pb::Message& first =
      trajectory->GetReflection()->GetRepeatedMessage(*trajectory, segments, 0);
  auto* second = trajectory->GetReflection()->AddMessage(trajectory.get(), segments);
  second->CopyFrom(first);
  const auto* start =
      second->GetDescriptor()->FindFieldByName("start_time_offset_ns");
  second->GetReflection()->SetUInt64(second, start, 999);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_trajectory_4d>(
          trajectory->SerializeAsString()),
      AdapterErrorCode::invalid_structure, "non-contiguous trajectory segments");
}

void test_trajectory_requires_zero_yaw_origin_and_c2_continuity() {
  auto trajectory = valid_contract("underwater.contracts.v1.ScoutTrajectory4d");
  const auto* segments =
      trajectory->GetDescriptor()->FindFieldByName("segments");
  auto* first = trajectory->GetReflection()->MutableRepeatedMessage(
      trajectory.get(), segments, 0);
  const auto* yaw = first->GetDescriptor()->FindFieldByName(
      "yaw_offset_control_points_rad");
  first->GetReflection()->SetRepeatedDouble(first, yaw, 0, 0.25);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_trajectory_4d>(
          trajectory->SerializeAsString()),
      AdapterErrorCode::invalid_structure, "nonzero first yaw offset");

  trajectory = valid_contract("underwater.contracts.v1.ScoutTrajectory4d");
  first = trajectory->GetReflection()->MutableRepeatedMessage(
      trajectory.get(), segments, 0);
  auto* second =
      trajectory->GetReflection()->AddMessage(trajectory.get(), segments);
  second->CopyFrom(*first);
  const auto* start =
      second->GetDescriptor()->FindFieldByName("start_time_offset_ns");
  second->GetReflection()->SetUInt64(second, start, 1U);
  const auto* points =
      second->GetDescriptor()->FindFieldByName("position_control_points");
  auto* first_point =
      second->GetReflection()->MutableRepeatedMessage(second, points, 0);
  const auto* x = first_point->GetDescriptor()->FindFieldByName("x_m");
  first_point->GetReflection()->SetDouble(first_point, x, 2.0);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_trajectory_4d>(
          trajectory->SerializeAsString()),
      AdapterErrorCode::invalid_structure, "C2-discontinuous trajectory");
}

void test_nested_identities_and_result_dependencies_are_fail_closed() {
  auto result = valid_contract("underwater.contracts.v1.ScoutPlanningResult");
  refresh_fixture_identities<ContractKind::scout_planning_result>(result.get());
  const auto* candidate = result->GetDescriptor()->FindFieldByName("candidate");
  auto* plan = result->GetReflection()->MutableMessage(result.get(), candidate);
  const auto* trajectory_field =
      plan->GetDescriptor()->FindFieldByName("trajectory");
  auto* trajectory =
      plan->GetReflection()->MutableMessage(plan, trajectory_field);
  const auto* segments =
      trajectory->GetDescriptor()->FindFieldByName("segments");
  auto* segment = trajectory->GetReflection()->MutableRepeatedMessage(
      trajectory, segments, 0);
  const auto* points =
      segment->GetDescriptor()->FindFieldByName("position_control_points");
  auto* point = segment->GetReflection()->MutableRepeatedMessage(
      segment, points, 5);
  const auto* x = point->GetDescriptor()->FindFieldByName("x_m");
  point->GetReflection()->SetDouble(point, x, 0.75);

  require_error(
      ProtobufAdapter::canonical_content_identity<
          ContractKind::scout_planning_result>(result->SerializeAsString()),
      AdapterErrorCode::invalid_content_identity,
      "canonical hash with a stale nested identity");
  const auto nested_stale =
      ProtobufAdapter::decode<ContractKind::scout_planning_result>(
          result->SerializeAsString());
  require_error(nested_stale, AdapterErrorCode::invalid_content_identity,
                "stale nested trajectory identity");
  scout_planner::test_support::require(
      nested_stale.error().path ==
          "underwater.contracts.v1.ScoutTrajectory4d",
      "nested identity failure was masked by the stale top-level identity");

  result = valid_contract("underwater.contracts.v1.ScoutPlanningResult");
  refresh_fixture_identities<ContractKind::scout_planning_result>(result.get());
  const auto* dependencies =
      result->GetDescriptor()->FindFieldByName("dependencies");
  auto* result_dependencies =
      result->GetReflection()->MutableMessage(result.get(), dependencies);
  const auto* map_version =
      result_dependencies->GetDescriptor()->FindFieldByName("map_version");
  result_dependencies->GetReflection()->SetUInt64(result_dependencies,
                                                   map_version, 9U);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_planning_result>(
          result->SerializeAsString()),
      AdapterErrorCode::invalid_structure,
      "candidate/result dependency mismatch");

  result = valid_contract("underwater.contracts.v1.ScoutPlanningResult");
  const auto* candidate_field =
      result->GetDescriptor()->FindFieldByName("candidate");
  plan = result->GetReflection()->MutableMessage(result.get(), candidate_field);
  const auto* report_field =
      plan->GetDescriptor()->FindFieldByName("validation_report");
  auto* report = plan->GetReflection()->MutableMessage(plan, report_field);
  const auto* validated_trajectory = report->GetDescriptor()->FindFieldByName(
      "validated_trajectory_content_identity");
  auto* wrong_identity = report->GetReflection()->MutableMessage(
      report, validated_trajectory);
  const auto* wrong_sha =
      wrong_identity->GetDescriptor()->FindFieldByName("sha256");
  wrong_identity->GetReflection()->SetString(wrong_identity, wrong_sha,
                                              std::string(32, '\x6b'));
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_planning_result>(
          result->SerializeAsString()),
      AdapterErrorCode::invalid_structure,
      "SAFE report with a mismatched trajectory binding");
}

void test_activated_collection_limits_and_canonical_ordering_are_enforced() {
  auto trajectory = valid_contract("underwater.contracts.v1.ScoutTrajectory4d");
  const auto* segments =
      trajectory->GetDescriptor()->FindFieldByName("segments");
  const pb::Message& first =
      trajectory->GetReflection()->GetRepeatedMessage(*trajectory, segments, 0);
  auto* second = trajectory->GetReflection()->AddMessage(trajectory.get(), segments);
  second->CopyFrom(first);
  const auto* start =
      second->GetDescriptor()->FindFieldByName("start_time_offset_ns");
  second->GetReflection()->SetUInt64(second, start, 1U);
  scout_planner::core::InterfaceLimits limits;
  limits.maximum_scout_plan_segments = 1U;
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_trajectory_4d>(
          trajectory->SerializeAsString(), {nullptr, 0U, &limits}),
      AdapterErrorCode::resource_limit_exceeded,
      "activated trajectory segment limit");
  require_error(
      ProtobufAdapter::canonicalize_and_identify<
          ContractKind::scout_trajectory_4d>(
          trajectory->SerializeAsString(), {nullptr, 0U, &limits}),
      AdapterErrorCode::resource_limit_exceeded,
      "producer hash seam ignored the activated segment limit");

  auto map = valid_contract("underwater.contracts.v1.HybridMapSnapshot");
  const auto* regions =
      map->GetDescriptor()->FindFieldByName("semantic_regions");
  const pb::Message& first_region =
      map->GetReflection()->GetRepeatedMessage(*map, regions, 0);
  map->GetReflection()->AddMessage(map.get(), regions)->CopyFrom(first_region);
  require_error(
      ProtobufAdapter::decode<ContractKind::hybrid_map_snapshot>(
          map->SerializeAsString()),
      AdapterErrorCode::invalid_structure,
      "duplicate unordered semantic region IDs");
}

void test_prediction_intervals_and_validation_report_modes_are_structural() {
  auto prediction =
      valid_contract("underwater.contracts.v1.MainRobotPrediction");
  const auto* intervals =
      prediction->GetDescriptor()->FindFieldByName("occupied_intervals");
  auto* interval = prediction->GetReflection()->MutableRepeatedMessage(
      prediction.get(), intervals, 0);
  const auto* start =
      interval->GetDescriptor()->FindFieldByName("start_offset_ns");
  interval->GetReflection()->SetUInt64(interval, start, 1U);
  require_error(
      ProtobufAdapter::decode<ContractKind::main_robot_prediction>(
          prediction->SerializeAsString()),
      AdapterErrorCode::invalid_time,
      "prediction whose first occupied interval starts after zero");

  prediction = valid_contract("underwater.contracts.v1.MainRobotPrediction");
  interval = prediction->GetReflection()->MutableRepeatedMessage(
      prediction.get(), intervals, 0);
  const auto* volume_field =
      interval->GetDescriptor()->FindFieldByName("swept_volume");
  auto* volume =
      interval->GetReflection()->MutableMessage(interval, volume_field);
  for (const char* radius_name :
       {"physical_radius_m", "position_uncertainty_radius_m",
        "conservative_occupied_radius_m"}) {
    volume->GetReflection()->SetDouble(
        volume, volume->GetDescriptor()->FindFieldByName(radius_name), -1.0);
  }
  require_error(
      ProtobufAdapter::decode<ContractKind::main_robot_prediction>(
          prediction->SerializeAsString()),
      AdapterErrorCode::invalid_numeric_boundary,
      "negative conservative occupancy radii");

  auto report = valid_contract(
      "underwater.contracts.v1.ScoutPlanValidationReport");
  const auto* status = report->GetDescriptor()->FindFieldByName("status");
  report->GetReflection()->SetEnumValue(report.get(), status, 2);
  const auto* unsafe_outcome =
      report->GetDescriptor()->FindFieldByName("primary_outcome");
  report->GetReflection()->SetEnumValue(report.get(), unsafe_outcome, 2);
  for (const char* field_name :
       {"minimum_collision_margin_m", "minimum_separation_margin_m",
        "minimum_energy_margin_j", "minimum_capability_margin",
        "survey_coverage_ratio"}) {
    report->GetReflection()->ClearField(
        report.get(), report->GetDescriptor()->FindFieldByName(field_name));
  }
  refresh_fixture_identities<ContractKind::scout_plan_validation_report>(
      report.get());
  const auto unsafe =
      ProtobufAdapter::decode<ContractKind::scout_plan_validation_report>(
          report->SerializeAsString());
  scout_planner::test_support::require(unsafe.has_value(),
                                       unsafe.error().message);

  report = valid_contract(
      "underwater.contracts.v1.ScoutPlanValidationReport");
  const auto* outcome =
      report->GetDescriptor()->FindFieldByName("primary_outcome");
  report->GetReflection()->SetEnumValue(report.get(), outcome, 2);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_plan_validation_report>(
          report->SerializeAsString()),
      AdapterErrorCode::invalid_structure,
      "SAFE report with a non-success primary outcome");
}

void test_plan_bindings_times_and_placeholder_free_identity_construction() {
  auto result = valid_contract("underwater.contracts.v1.ScoutPlanningResult");
  const auto* evaluated =
      result->GetDescriptor()->FindFieldByName("evaluated_at_monotonic_ns");
  result->GetReflection()->SetInt64(result.get(), evaluated, 400);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_planning_result>(
          result->SerializeAsString()),
      AdapterErrorCode::invalid_time,
      "planning evaluation after message generation");

  result = valid_contract("underwater.contracts.v1.ScoutPlanningResult");
  const auto* candidate = result->GetDescriptor()->FindFieldByName("candidate");
  auto* candidate_plan =
      result->GetReflection()->MutableMessage(result.get(), candidate);
  const auto* created =
      candidate_plan->GetDescriptor()->FindFieldByName("created_at_monotonic_ns");
  candidate_plan->GetReflection()->SetInt64(candidate_plan, created, 250);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_planning_result>(
          result->SerializeAsString()),
      AdapterErrorCode::invalid_time,
      "candidate creation after result evaluation");

  auto plan = valid_contract("underwater.contracts.v1.ScoutPlan");
  const auto clear_identity = [](pb::Message* owner,
                                 const char* field_name) {
    owner->GetReflection()->ClearField(
        owner, owner->GetDescriptor()->FindFieldByName(field_name));
  };
  auto* trajectory = plan->GetReflection()->MutableMessage(
      plan.get(), plan->GetDescriptor()->FindFieldByName("trajectory"));
  auto* dependencies = plan->GetReflection()->MutableMessage(
      plan.get(), plan->GetDescriptor()->FindFieldByName("dependencies"));
  auto* evidence = plan->GetReflection()->MutableMessage(
      plan.get(), plan->GetDescriptor()->FindFieldByName("survey_evidence"));
  auto* report = plan->GetReflection()->MutableMessage(
      plan.get(), plan->GetDescriptor()->FindFieldByName("validation_report"));
  clear_identity(trajectory, "trajectory_content_identity");
  clear_identity(dependencies, "dependencies_content_identity");
  clear_identity(evidence, "evidence_content_identity");
  clear_identity(report, "validation_report_content_identity");
  clear_identity(report, "validated_dependencies_content_identity");
  clear_identity(report, "validated_trajectory_content_identity");
  clear_identity(report, "validated_survey_evidence_content_identity");
  clear_identity(plan.get(), "plan_content_identity");
  const auto identified =
      ProtobufAdapter::canonicalize_and_identify<ContractKind::scout_plan>(
          plan->SerializeAsString());
  scout_planner::test_support::require(identified.has_value(),
                                       identified.error().message);
  const auto decoded =
      ProtobufAdapter::decode<ContractKind::scout_plan>(identified.value());
  scout_planner::test_support::require(decoded.has_value(),
                                       decoded.error().message);

  plan = valid_contract("underwater.contracts.v1.ScoutPlan");
  evidence = plan->GetReflection()->MutableMessage(
      plan.get(), plan->GetDescriptor()->FindFieldByName("survey_evidence"));
  const auto* mission_identity =
      evidence->GetDescriptor()->FindFieldByName("mission_content_identity");
  auto* identity =
      evidence->GetReflection()->MutableMessage(evidence, mission_identity);
  identity->GetReflection()->SetString(
      identity, identity->GetDescriptor()->FindFieldByName("sha256"),
      std::string(32, '\x71'));
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_plan>(
          plan->SerializeAsString()),
      AdapterErrorCode::invalid_structure,
      "survey evidence mission identity mismatch");
}

void test_map_layer_sizes_and_sensor_quaternions_are_structural_invariants() {
  auto map = valid_contract("underwater.contracts.v1.HybridMapSnapshot");
  const auto* occupancy_field =
      map->GetDescriptor()->FindFieldByName("occupancy");
  auto* occupancy =
      map->GetReflection()->MutableMessage(map.get(), occupancy_field);
  const auto* state = occupancy->GetDescriptor()->FindFieldByName("state");
  occupancy->GetReflection()->AddEnumValue(occupancy, state, 1);
  require_error(
      ProtobufAdapter::decode<ContractKind::hybrid_map_snapshot>(
          map->SerializeAsString()),
      AdapterErrorCode::invalid_structure, "map layer size mismatch");

  auto geometry =
      valid_contract("underwater.contracts.v1.ScoutSensorGeometry");
  const auto* extrinsics_field =
      geometry->GetDescriptor()->FindFieldByName("extrinsics");
  auto* extrinsics = geometry->GetReflection()->MutableMessage(
      geometry.get(), extrinsics_field);
  const auto* q_w = extrinsics->GetDescriptor()->FindFieldByName("q_w");
  extrinsics->GetReflection()->SetDouble(extrinsics, q_w, 2.0);
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_sensor_geometry>(
          geometry->SerializeAsString()),
      AdapterErrorCode::invalid_numeric_boundary,
      "non-unit sensor extrinsics quaternion");
}

void test_business_identity_and_numeric_canonicalization_follow_hashing_rules() {
  auto mission = valid_mission();
  const auto first_identity =
      ProtobufAdapter::canonical_content_identity<ContractKind::scout_mission>(
          mission.SerializeAsString());
  mission.mutable_header()->set_sequence(mission.header().sequence() + 1U);
  const auto second_identity =
      ProtobufAdapter::canonical_content_identity<ContractKind::scout_mission>(
          mission.SerializeAsString());
  scout_planner::test_support::require(
      first_identity.has_value() && second_identity.has_value() &&
          first_identity.value() == second_identity.value(),
      "ScoutMission delivery header changed its business identity");

  mission.clear_mission_content_identity();
  const auto identified =
      ProtobufAdapter::canonicalize_and_identify<ContractKind::scout_mission>(
          mission.SerializeAsString());
  scout_planner::test_support::require(identified.has_value(),
                                       identified.error().message);
  const auto identified_mission =
      ProtobufAdapter::decode<ContractKind::scout_mission>(identified.value());
  scout_planner::test_support::require(
      identified_mission.has_value(), identified_mission.error().message);

  mission = valid_mission();
  mission.mutable_header()->set_producer_id("cafe\xcc\x81");
  const auto normalized_mission =
      ProtobufAdapter::canonicalize_and_identify<ContractKind::scout_mission>(
          mission.SerializeAsString());
  scout_planner::test_support::require(normalized_mission.has_value(),
                                       normalized_mission.error().message);
  proto::ScoutMission reparsed_mission;
  scout_planner::test_support::require(
      reparsed_mission.ParseFromString(normalized_mission.value()) &&
          reparsed_mission.header().producer_id() == "caf\xc3\xa9",
      "producer seam did not normalize text to NFC before hashing");

  auto normalized_trajectory =
      valid_contract("underwater.contracts.v1.ScoutTrajectory4d");
  const auto* initial_yaw = normalized_trajectory->GetDescriptor()->FindFieldByName(
      "initial_yaw_rad");
  normalized_trajectory->GetReflection()->SetDouble(
      normalized_trajectory.get(), initial_yaw, std::acos(-1.0));
  const auto identified_trajectory =
      ProtobufAdapter::canonicalize_and_identify<
          ContractKind::scout_trajectory_4d>(
          normalized_trajectory->SerializeAsString());
  scout_planner::test_support::require(identified_trajectory.has_value(),
                                       identified_trajectory.error().message);
  proto::ScoutTrajectory4d reparsed_trajectory;
  scout_planner::test_support::require(
      reparsed_trajectory.ParseFromString(identified_trajectory.value()) &&
          reparsed_trajectory.initial_yaw_rad() == -std::acos(-1.0),
      "producer seam did not normalize yaw before hashing");

  auto navigation =
      valid_contract("underwater.contracts.v1.ScoutNavigationState");
  const auto* pose_field = navigation->GetDescriptor()->FindFieldByName("pose");
  auto* pose =
      navigation->GetReflection()->MutableMessage(navigation.get(), pose_field);
  const auto* x = pose->GetDescriptor()->FindFieldByName("x_m");
  pose->GetReflection()->SetDouble(pose, x, -0.0);
  refresh_fixture_identities<ContractKind::scout_navigation_state>(
      navigation.get());
  const auto decoded =
      ProtobufAdapter::decode<ContractKind::scout_navigation_state>(
          navigation->SerializeAsString());
  scout_planner::test_support::require(decoded.has_value(),
                                       decoded.error().message);
  const auto encoded = ProtobufAdapter::encode(decoded.value());
  scout_planner::test_support::require(encoded.has_value(),
                                       encoded.error().message);
  proto::ScoutNavigationState reparsed;
  scout_planner::test_support::require(reparsed.ParseFromString(encoded.value()),
                                       "canonical navigation wire is invalid");
  scout_planner::test_support::require(!std::signbit(reparsed.pose().x_m()),
                                       "negative zero reached the core output");
}

void test_non_nfc_text_and_noncanonical_instantaneous_yaw_are_rejected() {
  auto current = valid_contract("underwater.contracts.v1.ScoutCurrentEstimate");
  const auto* source =
      current->GetDescriptor()->FindFieldByName("model_source_id");
  current->GetReflection()->SetString(current.get(), source, "cafe\xcc\x81");
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_current_estimate>(
          current->SerializeAsString()),
      AdapterErrorCode::invalid_text, "non-NFC current source ID");

  auto trajectory = valid_contract("underwater.contracts.v1.ScoutTrajectory4d");
  const auto* yaw =
      trajectory->GetDescriptor()->FindFieldByName("initial_yaw_rad");
  trajectory->GetReflection()->SetDouble(trajectory.get(), yaw, std::acos(-1.0));
  require_error(
      ProtobufAdapter::decode<ContractKind::scout_trajectory_4d>(
          trajectory->SerializeAsString()),
      AdapterErrorCode::invalid_numeric_boundary,
      "instantaneous yaw at positive pi");
}

}  // namespace

int main() {
  try {
    test_scout_mission_round_trips_without_semantic_loss();
    std::cout << "[pass] scout_mission_round_trips_without_semantic_loss\n";
    test_all_ticket_14_core_contracts_use_the_golden_schema();
    std::cout << "[pass] all_ticket_14_core_contracts_use_the_golden_schema\n";
    test_ticket_15_admission_decision_uses_the_golden_schema();
    std::cout << "[pass] ticket_15_admission_decision_uses_golden_schema\n";
    test_adapter_rejects_missing_unknown_and_non_finite_safety_fields();
    std::cout << "[pass] adapter_rejects_missing_unknown_and_non_finite_safety_fields\n";
    test_adapter_centralizes_frame_identity_version_and_boundary_checks();
    std::cout << "[pass] adapter_centralizes_frame_identity_version_and_boundary_checks\n";
    test_navigation_covariance_must_be_symmetric_positive_semidefinite();
    std::cout << "[pass] navigation_covariance_is_psd\n";
    test_core_originated_document_is_validated_before_encoding();
    std::cout << "[pass] core_originated_document_is_validated_before_encoding\n";
    test_canonical_wire_hash_matches_independent_golden_value();
    std::cout << "[pass] canonical_wire_hash_matches_golden_value\n";
    test_failure_results_cannot_carry_candidates_or_illegal_times();
    std::cout << "[pass] failure_results_are_fail_closed\n";
    test_time_intervals_and_trajectory_segments_are_ordered();
    std::cout << "[pass] time_intervals_and_trajectory_segments_are_ordered\n";
    test_trajectory_requires_zero_yaw_origin_and_c2_continuity();
    std::cout << "[pass] trajectory_zero_origin_and_c2_continuity\n";
    test_nested_identities_and_result_dependencies_are_fail_closed();
    std::cout << "[pass] nested_identities_and_dependency_binding\n";
    test_activated_collection_limits_and_canonical_ordering_are_enforced();
    std::cout << "[pass] activated_limits_and_canonical_ordering\n";
    test_prediction_intervals_and_validation_report_modes_are_structural();
    std::cout << "[pass] prediction_intervals_and_validation_modes\n";
    test_plan_bindings_times_and_placeholder_free_identity_construction();
    std::cout << "[pass] plan_bindings_times_and_identity_construction\n";
    test_map_layer_sizes_and_sensor_quaternions_are_structural_invariants();
    std::cout << "[pass] map_sizes_and_sensor_quaternions_are_validated\n";
    test_business_identity_and_numeric_canonicalization_follow_hashing_rules();
    std::cout << "[pass] business_identity_and_numeric_canonicalization\n";
    test_non_nfc_text_and_noncanonical_instantaneous_yaw_are_rejected();
    std::cout << "[pass] nfc_and_yaw_canonicalization_are_enforced\n";
  } catch (const std::exception& error) {
    std::cerr << "[failure] " << failure_context
              << " test=scout_mission_round_trips_without_semantic_loss error="
              << error.what() << '\n';
    return 1;
  }
  return 0;
}
