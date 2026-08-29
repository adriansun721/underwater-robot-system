#include "scout_planner/core/planning_context.hpp"
#include "scout_planner/core/capability_energy_gate.hpp"
#include "scout_planner/core/survey_plan_evidence.hpp"
#include "scout_planner/core/quintic_bezier.hpp"
#include "contract_fixture.hpp"
#include "test_support.hpp"

#include "underwater/contracts/v1/capability.pb.h"
#include "underwater/contracts/v1/cooperation.pb.h"
#include "underwater/contracts/v1/mapping.pb.h"
#include "underwater/contracts/v1/planning.pb.h"
#include "underwater/contracts/v1/sensing.pb.h"
#include "underwater/contracts/v1/state.pb.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <utility>

namespace {

using scout_planner::core::ActivatedPlanningConfiguration;
using scout_planner::core::AdmittedSurveyTask;
using scout_planner::core::ContextErrorCode;
using scout_planner::core::PlanningContextBuilder;
using scout_planner::core::PlanningContextInputs;
using scout_planner::core::PlanningContextSource;
using scout_planner::core::PlanningInputKind;
using scout_planner::core::ReceivedInput;
using scout_planner::core::InputReceipt;
using scout_planner::core::ContractKind;
namespace proto = underwater::contracts::v1;
namespace fixture = scout_planner::test_contract_fixture;

constexpr std::uint64_t kSeed = 0x5C0A7B15ULL;
const std::string failure_context =
    scout_planner::test_support::default_failure_context(kSeed);

class EmptySource final : public PlanningContextSource {
 public:
  [[nodiscard]] std::uint64_t capture_generation() const noexcept override {
    return 1U;
  }

  [[nodiscard]] std::optional<PlanningContextInputs> read_inputs()
      const override {
    return std::nullopt;
  }
};

class FixedSource final : public PlanningContextSource {
 public:
  explicit FixedSource(PlanningContextInputs inputs)
      : inputs_(std::move(inputs)) {}

  [[nodiscard]] std::uint64_t capture_generation() const noexcept override {
    return generation_;
  }

  [[nodiscard]] std::optional<PlanningContextInputs> read_inputs()
      const override {
    return inputs_;
  }

  PlanningContextInputs inputs_;
  std::uint64_t generation_{1U};
};

class RacingSource final : public PlanningContextSource {
 public:
  explicit RacingSource(PlanningContextInputs inputs)
      : inputs_(std::move(inputs)) {}

  [[nodiscard]] std::uint64_t capture_generation() const noexcept override {
    return generation_reads_++ == 0U ? 1U : 2U;
  }

  [[nodiscard]] std::optional<PlanningContextInputs> read_inputs()
      const override {
    return inputs_;
  }

 private:
  PlanningContextInputs inputs_;
  mutable std::uint64_t generation_reads_{0U};
};

void set_synchronized_time(proto::SynchronizedObservationTime* time,
                           const std::int64_t utc_time_ns = 10'000) {
  time->set_utc_time_ns(utc_time_ns);
  time->set_status(proto::TIME_SYNC_SYNCHRONIZED);
  time->set_uncertainty_ns(5U);
}

void set_header(proto::MessageHeader* header, const proto::StreamId stream,
                const std::string& source_clock_domain_id) {
  header->set_stream_id(stream);
  header->set_sequence(10U);
  header->set_source_clock_domain_id(source_clock_domain_id);
  header->set_generated_at_monotonic_ns(900);
  set_synchronized_time(header->mutable_observed_at());
}

template <ContractKind Kind, typename Message, typename Mutation>
scout_planner::core::CoreContract<Kind> mutate_contract(
    const scout_planner::core::CoreContract<Kind>& value, Mutation mutation) {
  const auto encoded = scout_planner::core::ProtobufAdapter::encode(value);
  scout_planner::test_support::require(encoded.has_value(),
                                       encoded.error().message);
  Message message;
  scout_planner::test_support::require(
      message.ParseFromString(encoded.value()), "fixture mutation parse failed");
  mutation(&message);
  fixture::identify_in_place<Kind>(&message);
  return fixture::decode_identified<Kind>(message);
}

template <ContractKind Kind, typename Message>
Message parse_contract(
    const scout_planner::core::CoreContract<Kind>& value) {
  const auto encoded = scout_planner::core::ProtobufAdapter::encode(value);
  scout_planner::test_support::require(encoded.has_value(),
                                       encoded.error().message);
  Message message;
  scout_planner::test_support::require(
      message.ParseFromString(encoded.value()), "fixture parse failed");
  return message;
}

void copy_identity(proto::ContentIdentity* destination,
                   const proto::ContentIdentity& source) {
  destination->set_sha256(source.sha256());
}

void set_profile(proto::ProfileRef* profile, const std::string& id,
                 const std::uint64_t version,
                 const proto::ContentIdentity& identity) {
  profile->set_profile_id(id);
  profile->set_version(version);
  copy_identity(profile->mutable_content_identity(), identity);
}

template <typename Value>
ReceivedInput<Value> received(Value value,
                             const std::int64_t received_at = 900) {
  return ReceivedInput<Value>{std::move(value), InputReceipt{received_at}};
}

AdmittedSurveyTask admitted(scout_planner::core::SurveyTask mission) {
  const auto message = parse_contract<ContractKind::scout_mission,
                                      proto::ScoutMission>(mission);
  auto decision = fixture::populated_message<proto::ScoutMissionDecision>();
  set_header(decision.mutable_header(), proto::STREAM_SCOUT_MISSION_DECISION,
             "scout-nuc/boot-1");
  decision.set_mission_id(message.mission_id());
  decision.set_mission_version(message.mission_version());
  copy_identity(decision.mutable_mission_content_identity(),
                message.mission_content_identity());
  decision.set_disposition(proto::SCOUT_MISSION_DECISION_ACCEPTED);
  decision.set_received_at_monotonic_ns(900);
  decision.set_admission_valid_until_monotonic_ns(2'000);
  decision.set_accepted_coordination_version(message.coordination_version());
  fixture::identify_in_place<ContractKind::scout_mission_decision>(&decision);
  return AdmittedSurveyTask{
      std::move(mission), InputReceipt{900},
      fixture::decode_identified<ContractKind::scout_mission_decision>(
          decision),
      InputReceipt{900}};
}

PlanningContextInputs valid_inputs() {
  auto mission_message = fixture::populated_message<proto::ScoutMission>();
  auto navigation_message =
      fixture::populated_message<proto::ScoutNavigationState>();
  auto map_message = fixture::populated_message<proto::HybridMapSnapshot>();
  auto current_message =
      fixture::populated_message<proto::ScoutCurrentEstimate>();
  auto geometry_message =
      fixture::populated_message<proto::ScoutSensorGeometry>();
  auto health_message =
      fixture::populated_message<proto::ScoutSensorHealthState>();
  auto capability_message =
      fixture::populated_message<proto::ScoutCapabilityProfile>();
  auto thruster_message =
      fixture::populated_message<proto::ScoutThrusterHealthState>();
  auto energy_model_message =
      fixture::populated_message<proto::ScoutEnergyModelProfile>();
  auto energy_state_message =
      fixture::populated_message<proto::ScoutEnergyState>();
  auto prediction_message =
      fixture::populated_message<proto::MainRobotPrediction>();
  auto coordination_message =
      fixture::populated_message<proto::ScoutCoordinationConstraint>();
  auto dependencies_message =
      fixture::populated_message<proto::ScoutPlanningDependencies>();

  set_header(mission_message.mutable_header(), proto::STREAM_SCOUT_MISSION,
             "main-nuc/boot-1");
  set_header(navigation_message.mutable_header(),
             proto::STREAM_SCOUT_NAVIGATION_STATE, "scout-nuc/boot-1");
  navigation_message.set_navigation_version(5U);
  navigation_message.set_observed_at_monotonic_ns(900);
  map_message.set_source_clock_domain_id("scout-nuc/boot-1");
  map_message.set_generated_at_monotonic_ns(900);
  set_synchronized_time(map_message.mutable_observed_at());
  set_header(current_message.mutable_header(),
             proto::STREAM_SCOUT_CURRENT_ESTIMATE, "scout-nuc/boot-1");
  current_message.set_observed_at_monotonic_ns(900);
  current_message.set_valid_from_monotonic_ns(800);
  current_message.set_valid_until_monotonic_ns(2'000);
  set_header(geometry_message.mutable_header(),
             proto::STREAM_SCOUT_SENSOR_GEOMETRY, "scout-nuc/boot-1");
  set_header(health_message.mutable_header(),
             proto::STREAM_SCOUT_SENSOR_HEALTH, "scout-nuc/boot-1");
  health_message.set_observed_at_monotonic_ns(900);
  health_message.set_valid_until_monotonic_ns(2'000);
  set_header(capability_message.mutable_header(),
             proto::STREAM_SCOUT_CAPABILITY_PROFILE, "scout-nuc/boot-1");
  set_header(thruster_message.mutable_header(),
             proto::STREAM_SCOUT_THRUSTER_HEALTH, "scout-nuc/boot-1");
  thruster_message.set_observed_at_monotonic_ns(900);
  thruster_message.set_valid_until_monotonic_ns(2'000);
  set_header(energy_model_message.mutable_header(),
             proto::STREAM_SCOUT_ENERGY_MODEL, "scout-nuc/boot-1");
  set_header(energy_state_message.mutable_header(),
             proto::STREAM_SCOUT_ENERGY_STATE, "scout-nuc/boot-1");
  energy_state_message.set_observed_at_monotonic_ns(900);
  energy_state_message.set_valid_until_monotonic_ns(2'000);
  set_header(prediction_message.mutable_header(),
             proto::STREAM_MAIN_ROBOT_PREDICTION, "main-nuc/boot-1");
  set_synchronized_time(prediction_message.mutable_alignment_epoch());
  set_header(coordination_message.mutable_header(),
             proto::STREAM_SCOUT_COORDINATION_CONSTRAINT, "main-nuc/boot-1");

  fixture::identify_in_place<ContractKind::scout_mission>(&mission_message);
  fixture::identify_in_place<ContractKind::scout_navigation_state>(
      &navigation_message);
  fixture::identify_in_place<ContractKind::hybrid_map_snapshot>(&map_message);
  fixture::identify_in_place<ContractKind::scout_current_estimate>(
      &current_message);
  fixture::identify_in_place<ContractKind::scout_sensor_geometry>(
      &geometry_message);
  fixture::identify_in_place<ContractKind::scout_sensor_health_state>(
      &health_message);
  fixture::identify_in_place<ContractKind::scout_capability_profile>(
      &capability_message);

  set_profile(thruster_message.mutable_active_capability_profile(),
              capability_message.capability_profile_id(),
              capability_message.capability_profile_version(),
              capability_message.capability_content_identity());
  fixture::identify_in_place<ContractKind::scout_thruster_health_state>(
      &thruster_message);

  set_profile(energy_model_message.mutable_capability_profile(),
              capability_message.capability_profile_id(),
              capability_message.capability_profile_version(),
              capability_message.capability_content_identity());
  fixture::identify_in_place<ContractKind::scout_energy_model_profile>(
      &energy_model_message);

  set_profile(energy_state_message.mutable_energy_model(),
              energy_model_message.energy_model_id(),
              energy_model_message.energy_model_version(),
              energy_model_message.energy_model_content_identity());
  fixture::identify_in_place<ContractKind::scout_energy_state>(
      &energy_state_message);

  prediction_message.set_mission_id(mission_message.mission_id());
  prediction_message.set_mission_version(mission_message.mission_version());
  copy_identity(prediction_message.mutable_mission_content_identity(),
                mission_message.mission_content_identity());
  fixture::identify_in_place<ContractKind::main_robot_prediction>(
      &prediction_message);

  coordination_message.set_mission_id(mission_message.mission_id());
  coordination_message.set_mission_version(mission_message.mission_version());
  copy_identity(coordination_message.mutable_mission_content_identity(),
                mission_message.mission_content_identity());
  coordination_message.set_coordination_version(
      mission_message.coordination_version());
  coordination_message.set_prediction_id(prediction_message.prediction_id());
  coordination_message.set_prediction_version(
      prediction_message.prediction_version());
  copy_identity(coordination_message.mutable_prediction_content_identity(),
                prediction_message.prediction_content_identity());
  fixture::identify_in_place<ContractKind::scout_coordination_constraint>(
      &coordination_message);

  dependencies_message.set_mission_id(mission_message.mission_id());
  dependencies_message.set_mission_version(mission_message.mission_version());
  copy_identity(dependencies_message.mutable_mission_content_identity(),
                mission_message.mission_content_identity());
  dependencies_message.set_map_id(map_message.map_id());
  dependencies_message.set_map_version(map_message.map_version());
  copy_identity(dependencies_message.mutable_map_content_identity(),
                map_message.map_content_identity());
  dependencies_message.set_navigation_version(
      navigation_message.navigation_version());
  copy_identity(dependencies_message.mutable_navigation_content_identity(),
                navigation_message.navigation_content_identity());
  dependencies_message.clear_sensors();
  auto* sensor = dependencies_message.add_sensors();
  sensor->set_sensor_id(geometry_message.sensor_id());
  sensor->set_geometry_version(geometry_message.geometry_version());
  copy_identity(sensor->mutable_geometry_content_identity(),
                geometry_message.geometry_content_identity());
  sensor->set_health_version(health_message.health_version());
  copy_identity(sensor->mutable_health_content_identity(),
                health_message.health_content_identity());
  dependencies_message.set_current_model_id(current_message.current_model_id());
  dependencies_message.set_current_model_version(
      current_message.current_model_version());
  copy_identity(dependencies_message.mutable_current_content_identity(),
                current_message.current_content_identity());
  set_profile(dependencies_message.mutable_capability_profile(),
              capability_message.capability_profile_id(),
              capability_message.capability_profile_version(),
              capability_message.capability_content_identity());
  dependencies_message.set_thruster_health_version(
      thruster_message.health_version());
  copy_identity(
      dependencies_message.mutable_thruster_health_content_identity(),
      thruster_message.health_content_identity());
  set_profile(dependencies_message.mutable_energy_model(),
              energy_model_message.energy_model_id(),
              energy_model_message.energy_model_version(),
              energy_model_message.energy_model_content_identity());
  dependencies_message.set_energy_store_id(
      energy_state_message.energy_store_id());
  dependencies_message.set_energy_state_version(
      energy_state_message.energy_state_version());
  copy_identity(dependencies_message.mutable_energy_state_content_identity(),
                energy_state_message.energy_state_content_identity());
  dependencies_message.set_prediction_id(prediction_message.prediction_id());
  dependencies_message.set_prediction_version(
      prediction_message.prediction_version());
  copy_identity(dependencies_message.mutable_prediction_content_identity(),
                prediction_message.prediction_content_identity());
  dependencies_message.set_coordination_version(
      coordination_message.coordination_version());
  copy_identity(dependencies_message.mutable_coordination_content_identity(),
                coordination_message.coordination_content_identity());
  fixture::identify_in_place<ContractKind::scout_planning_dependencies>(
      &dependencies_message);

  const auto mission =
      fixture::decode_identified<ContractKind::scout_mission>(mission_message);
  const auto navigation = fixture::decode_identified<
      ContractKind::scout_navigation_state>(navigation_message);
  const auto map = fixture::decode_identified<
      ContractKind::hybrid_map_snapshot>(map_message);
  const auto current = fixture::decode_identified<
      ContractKind::scout_current_estimate>(current_message);
  const auto geometry = fixture::decode_identified<
      ContractKind::scout_sensor_geometry>(geometry_message);
  const auto health = fixture::decode_identified<
      ContractKind::scout_sensor_health_state>(health_message);
  const auto capability = fixture::decode_identified<
      ContractKind::scout_capability_profile>(capability_message);
  const auto thruster = fixture::decode_identified<
      ContractKind::scout_thruster_health_state>(thruster_message);
  const auto energy_model = fixture::decode_identified<
      ContractKind::scout_energy_model_profile>(energy_model_message);
  const auto energy_state = fixture::decode_identified<
      ContractKind::scout_energy_state>(energy_state_message);
  const auto prediction = fixture::decode_identified<
      ContractKind::main_robot_prediction>(prediction_message);
  const auto coordination = fixture::decode_identified<
      ContractKind::scout_coordination_constraint>(coordination_message);
  const auto dependencies = fixture::decode_identified<
      ContractKind::scout_planning_dependencies>(dependencies_message);

  return PlanningContextInputs{
      admitted(mission),
      received(navigation),
      received(map),
      received(current),
      {{received(geometry), received(health)}},
      received(capability),
      received(thruster),
      received(energy_model),
      received(energy_state),
      received(prediction),
      received(coordination),
      received(dependencies),
  };
}

ActivatedPlanningConfiguration valid_configuration() {
  ActivatedPlanningConfiguration configuration;
  configuration.scout_clock_domain_id = "scout-nuc/boot-1";
  configuration.operating_domain_id = "fixture-id";
  configuration.activation_generation = 1U;
  configuration.planner_configuration =
      {"fixture-id", 1U, {}};
  configuration.timing_profile = {"fixture-id", 1U, {}};
  configuration.interface_limits = {"fixture-id", 1U, {}};
  configuration.planner_configuration.content_identity.fill(0x5aU);
  configuration.timing_profile.content_identity.fill(0x5aU);
  configuration.interface_limits.content_identity.fill(0x5aU);
  configuration.maximum_input_age_ns.fill(1'000U);
  configuration.maximum_synchronization_delta_ns = 100U;
  configuration.maximum_synchronization_uncertainty_ns = 10U;
  return configuration;
}

void test_missing_input_set_is_rejected_as_a_whole() {
  PlanningContextBuilder builder;
  EmptySource source;

  const auto result = builder.capture(source, valid_configuration(), 1'000);

  scout_planner::test_support::require(
      !result.has_value(), "missing input set unexpectedly produced a context");
  scout_planner::test_support::require(
      result.error().code == ContextErrorCode::missing_input,
      "missing input set did not return the structured missing-input reason");
}

void test_local_receipt_age_boundaries_fail_closed() {
  PlanningContextBuilder builder;
  auto configuration = valid_configuration();
  configuration.maximum_input_age_ns.fill(100U);

  FixedSource boundary_source(valid_inputs());
  const auto boundary = builder.capture(boundary_source, configuration, 1'000);
  scout_planner::test_support::require(
      boundary.has_value(), "input at the exact maximum age was rejected");

  FixedSource stale_source(valid_inputs());
  stale_source.inputs_.navigation.receipt.local_received_at_monotonic_ns = 899;
  const auto stale = builder.capture(stale_source, configuration, 1'000);
  scout_planner::test_support::require(
      !stale.has_value() && stale.error().code == ContextErrorCode::stale_input &&
          stale.error().input == PlanningInputKind::navigation,
      "input one nanosecond beyond the maximum age did not fail closed");

  FixedSource future_source(valid_inputs());
  future_source.inputs_.navigation.receipt.local_received_at_monotonic_ns =
      1'001;
  const auto future = builder.capture(future_source, configuration, 1'000);
  scout_planner::test_support::require(
      !future.has_value() &&
          future.error().code == ContextErrorCode::future_receipt &&
          future.error().input == PlanningInputKind::navigation,
      "future local receipt time did not fail closed");
}

void test_missing_sensor_and_capture_race_reject_the_whole_snapshot() {
  PlanningContextBuilder builder;
  const auto configuration = valid_configuration();

  FixedSource missing_sensor(valid_inputs());
  missing_sensor.inputs_.sensors.clear();
  const auto missing = builder.capture(missing_sensor, configuration, 1'000);
  scout_planner::test_support::require(
      !missing.has_value() &&
          missing.error().code == ContextErrorCode::missing_input &&
          missing.error().input == PlanningInputKind::sensor_geometry,
      "a context without a paired sensor was published");

  FixedSource expired_admission(valid_inputs());
  expired_admission.inputs_.mission.decision = mutate_contract<
      ContractKind::scout_mission_decision, proto::ScoutMissionDecision>(
      expired_admission.inputs_.mission.decision, [](auto* decision) {
        decision->set_admission_valid_until_monotonic_ns(999);
      });
  const auto expired =
      builder.capture(expired_admission, configuration, 1'000);
  scout_planner::test_support::require(
      !expired.has_value() && expired.error().code == ContextErrorCode::stale_input &&
          expired.error().input == PlanningInputKind::mission,
      "an expired mission admission entered the planning context");

  FixedSource wrong_admission(valid_inputs());
  wrong_admission.inputs_.mission.decision = mutate_contract<
      ContractKind::scout_mission_decision, proto::ScoutMissionDecision>(
      wrong_admission.inputs_.mission.decision, [](auto* decision) {
        decision->set_mission_version(decision->mission_version() + 1U);
      });
  const auto wrong = builder.capture(wrong_admission, configuration, 1'000);
  scout_planner::test_support::require(
      !wrong.has_value() && wrong.error().code == ContextErrorCode::mission_mismatch &&
          wrong.error().input == PlanningInputKind::mission,
      "an admission decision for another mission entered the context");

  FixedSource rejected_admission(valid_inputs());
  rejected_admission.inputs_.mission.decision = mutate_contract<
      ContractKind::scout_mission_decision, proto::ScoutMissionDecision>(
      rejected_admission.inputs_.mission.decision, [](auto* decision) {
        decision->set_disposition(proto::SCOUT_MISSION_DECISION_REJECTED);
      });
  const auto rejected =
      builder.capture(rejected_admission, configuration, 1'000);
  scout_planner::test_support::require(
      !rejected.has_value() &&
          rejected.error().code == ContextErrorCode::admission_rejected &&
          rejected.error().input == PlanningInputKind::mission,
      "a rejected mission decision was treated as admission");

  FixedSource mismatched_receive_time(valid_inputs());
  mismatched_receive_time.inputs_.mission.mission_receipt
      .local_received_at_monotonic_ns = 899;
  const auto mismatched_receive =
      builder.capture(mismatched_receive_time, configuration, 1'000);
  scout_planner::test_support::require(
      !mismatched_receive.has_value() &&
          mismatched_receive.error().code ==
              ContextErrorCode::mission_mismatch &&
          mismatched_receive.error().input == PlanningInputKind::mission,
      "an admission decision was spliced onto another mission receipt");

  RacingSource racing(valid_inputs());
  const auto raced = builder.capture(racing, configuration, 1'000);
  scout_planner::test_support::require(
      !raced.has_value() &&
          raced.error().code == ContextErrorCode::capture_race,
      "a generation change during capture did not reject the whole snapshot");
}

void test_clock_and_dependency_mismatches_fail_closed() {
  PlanningContextBuilder builder;
  const auto configuration = valid_configuration();

  FixedSource wrong_clock(valid_inputs());
  wrong_clock.inputs_.navigation.value = mutate_contract<
      ContractKind::scout_navigation_state, proto::ScoutNavigationState>(
      wrong_clock.inputs_.navigation.value, [](auto* navigation) {
        navigation->mutable_header()->set_source_clock_domain_id(
            "scout-nuc/another-boot");
      });
  const auto clock_result =
      builder.capture(wrong_clock, configuration, 1'000);
  scout_planner::test_support::require(
      !clock_result.has_value() &&
          clock_result.error().code ==
              ContextErrorCode::clock_domain_mismatch &&
          clock_result.error().input == PlanningInputKind::navigation,
      "a wrong Scout-local clock domain entered the planning context");

  FixedSource wrong_dependency(valid_inputs());
  wrong_dependency.inputs_.dependencies.value = mutate_contract<
      ContractKind::scout_planning_dependencies,
      proto::ScoutPlanningDependencies>(
      wrong_dependency.inputs_.dependencies.value, [](auto* dependencies) {
        dependencies->set_navigation_version(
            dependencies->navigation_version() + 1U);
      });
  const auto dependency_result =
      builder.capture(wrong_dependency, configuration, 1'000);
  scout_planner::test_support::require(
      !dependency_result.has_value() &&
          dependency_result.error().code ==
              ContextErrorCode::dependency_mismatch &&
          dependency_result.error().input == PlanningInputKind::navigation,
      "a mixed navigation/dependency version entered the planning context");
}

void test_pairing_operating_domain_and_synchronization_fail_closed() {
  PlanningContextBuilder builder;
  const auto configuration = valid_configuration();

  FixedSource wrong_mission(valid_inputs());
  wrong_mission.inputs_.main_robot_prediction.value = mutate_contract<
      ContractKind::main_robot_prediction, proto::MainRobotPrediction>(
      wrong_mission.inputs_.main_robot_prediction.value, [](auto* prediction) {
        prediction->set_mission_id(prediction->mission_id() + 1U);
      });
  const auto mission_result =
      builder.capture(wrong_mission, configuration, 1'000);
  scout_planner::test_support::require(
      !mission_result.has_value() &&
          mission_result.error().code == ContextErrorCode::mission_mismatch &&
          mission_result.error().input ==
              PlanningInputKind::main_robot_prediction,
      "a prediction for another mission entered the planning context");

  FixedSource wrong_prediction(valid_inputs());
  wrong_prediction.inputs_.coordination.value = mutate_contract<
      ContractKind::scout_coordination_constraint,
      proto::ScoutCoordinationConstraint>(
      wrong_prediction.inputs_.coordination.value, [](auto* coordination) {
        coordination->set_prediction_version(
            coordination->prediction_version() + 1U);
      });
  const auto prediction_result =
      builder.capture(wrong_prediction, configuration, 1'000);
  scout_planner::test_support::require(
      !prediction_result.has_value() &&
          prediction_result.error().code ==
              ContextErrorCode::prediction_mismatch &&
          prediction_result.error().input == PlanningInputKind::coordination,
      "a coordination constraint for another prediction entered the context");

  FixedSource wrong_domain(valid_inputs());
  wrong_domain.inputs_.energy_state.value = mutate_contract<
      ContractKind::scout_energy_state, proto::ScoutEnergyState>(
      wrong_domain.inputs_.energy_state.value, [](auto* energy) {
        energy->set_operating_domain_id("another-operating-domain");
      });
  const auto domain_result =
      builder.capture(wrong_domain, configuration, 1'000);
  scout_planner::test_support::require(
      !domain_result.has_value() &&
          domain_result.error().code ==
              ContextErrorCode::operating_domain_mismatch &&
          domain_result.error().input == PlanningInputKind::energy_state,
      "an energy state from another operating domain entered the context");

  FixedSource unsynchronized(valid_inputs());
  unsynchronized.inputs_.main_robot_prediction.value = mutate_contract<
      ContractKind::main_robot_prediction, proto::MainRobotPrediction>(
      unsynchronized.inputs_.main_robot_prediction.value, [](auto* prediction) {
        prediction->mutable_alignment_epoch()->set_status(
            proto::TIME_SYNC_UNSYNCHRONIZED);
      });
  const auto synchronization_result =
      builder.capture(unsynchronized, configuration, 1'000);
  scout_planner::test_support::require(
      !synchronization_result.has_value() &&
          synchronization_result.error().code ==
              ContextErrorCode::synchronization_invalid,
      "an unsynchronized cross-device prediction entered the context");

  FixedSource skewed(valid_inputs());
  skewed.inputs_.map.value = mutate_contract<
      ContractKind::hybrid_map_snapshot, proto::HybridMapSnapshot>(
      skewed.inputs_.map.value, [](auto* map) {
        map->mutable_observed_at()->set_utc_time_ns(10'101);
      });
  const auto skew_result = builder.capture(skewed, configuration, 1'000);
  scout_planner::test_support::require(
      !skew_result.has_value() &&
          skew_result.error().code ==
              ContextErrorCode::synchronization_tolerance_exceeded,
      "critical observations beyond synchronization tolerance were mixed");
}

void bind_navigation_dependency(PlanningContextInputs* inputs) {
  const auto navigation =
      parse_contract<ContractKind::scout_navigation_state,
                     proto::ScoutNavigationState>(inputs->navigation.value);
  inputs->dependencies.value = mutate_contract<
      ContractKind::scout_planning_dependencies,
      proto::ScoutPlanningDependencies>(
      inputs->dependencies.value, [&](auto* dependencies) {
        dependencies->set_navigation_version(navigation.navigation_version());
        copy_identity(dependencies->mutable_navigation_content_identity(),
                      navigation.navigation_content_identity());
      });
}

void bind_current_dependency(PlanningContextInputs* inputs) {
  const auto current =
      parse_contract<ContractKind::scout_current_estimate,
                     proto::ScoutCurrentEstimate>(inputs->current.value);
  inputs->dependencies.value = mutate_contract<
      ContractKind::scout_planning_dependencies,
      proto::ScoutPlanningDependencies>(
      inputs->dependencies.value, [&](auto* dependencies) {
        dependencies->set_current_model_id(current.current_model_id());
        dependencies->set_current_model_version(current.current_model_version());
        copy_identity(dependencies->mutable_current_content_identity(),
                      current.current_content_identity());
      });
}

void bind_map_dependency(PlanningContextInputs* inputs) {
  const auto map = parse_contract<ContractKind::hybrid_map_snapshot,
                                  proto::HybridMapSnapshot>(inputs->map.value);
  inputs->dependencies.value = mutate_contract<
      ContractKind::scout_planning_dependencies,
      proto::ScoutPlanningDependencies>(
      inputs->dependencies.value, [&](auto* dependencies) {
        dependencies->set_map_id(map.map_id());
        dependencies->set_map_version(map.map_version());
        copy_identity(dependencies->mutable_map_content_identity(),
                      map.map_content_identity());
      });
}

void bind_energy_dependency(PlanningContextInputs* inputs) {
  const auto energy =
      parse_contract<ContractKind::scout_energy_state,
                     proto::ScoutEnergyState>(inputs->energy_state.value);
  inputs->dependencies.value = mutate_contract<
      ContractKind::scout_planning_dependencies,
      proto::ScoutPlanningDependencies>(
      inputs->dependencies.value, [&](auto* dependencies) {
        dependencies->set_energy_store_id(energy.energy_store_id());
        dependencies->set_energy_state_version(energy.energy_state_version());
        copy_identity(dependencies->mutable_energy_state_content_identity(),
                      energy.energy_state_content_identity());
      });
}

void bind_sensor_dependencies(PlanningContextInputs* inputs) {
  inputs->dependencies.value = mutate_contract<
      ContractKind::scout_planning_dependencies,
      proto::ScoutPlanningDependencies>(
      inputs->dependencies.value, [&](auto* dependencies) {
        dependencies->clear_sensors();
        for (const auto& pair : inputs->sensors) {
          const auto geometry =
              parse_contract<ContractKind::scout_sensor_geometry,
                             proto::ScoutSensorGeometry>(pair.geometry.value);
          const auto health =
              parse_contract<ContractKind::scout_sensor_health_state,
                             proto::ScoutSensorHealthState>(pair.health.value);
          auto* sensor = dependencies->add_sensors();
          sensor->set_sensor_id(geometry.sensor_id());
          sensor->set_geometry_version(geometry.geometry_version());
          copy_identity(sensor->mutable_geometry_content_identity(),
                        geometry.geometry_content_identity());
          sensor->set_health_version(health.health_version());
          copy_identity(sensor->mutable_health_content_identity(),
                        health.health_content_identity());
        }
      });
}

void add_second_sensor(PlanningContextInputs* inputs) {
  auto geometry = mutate_contract<ContractKind::scout_sensor_geometry,
                                  proto::ScoutSensorGeometry>(
      inputs->sensors.front().geometry.value, [](auto* value) {
        value->set_sensor_id("sensor-z");
        value->set_geometry_version(value->geometry_version() + 1U);
        value->mutable_header()->set_sequence(11U);
      });
  auto health = mutate_contract<ContractKind::scout_sensor_health_state,
                                proto::ScoutSensorHealthState>(
      inputs->sensors.front().health.value, [](auto* value) {
        value->set_sensor_id("sensor-z");
        value->set_health_version(value->health_version() + 1U);
        value->mutable_header()->set_sequence(11U);
      });
  inputs->sensors.push_back(
      {{std::move(geometry), InputReceipt{900}},
       {std::move(health), InputReceipt{900}}});
  bind_sensor_dependencies(inputs);
}

void test_session_sequence_and_version_watermarks_fail_closed() {
  PlanningContextBuilder builder;
  const auto configuration = valid_configuration();

  FixedSource initial(valid_inputs());
  const auto first = builder.capture(initial, configuration, 1'000);
  scout_planner::test_support::require(first.has_value(),
                                       "initial watermark capture failed");

  FixedSource reordered(valid_inputs());
  reordered.inputs_.navigation.value = mutate_contract<
      ContractKind::scout_navigation_state, proto::ScoutNavigationState>(
      reordered.inputs_.navigation.value, [](auto* navigation) {
        navigation->mutable_header()->set_sequence(9U);
      });
  const auto sequence_result =
      builder.capture(reordered, configuration, 1'000);
  scout_planner::test_support::require(
      !sequence_result.has_value() &&
          sequence_result.error().code ==
              ContextErrorCode::sequence_rollback &&
          sequence_result.error().input == PlanningInputKind::navigation,
      "same-session sequence rollback entered the planning context");

  FixedSource changed_logical_id(valid_inputs());
  changed_logical_id.inputs_.current.value = mutate_contract<
      ContractKind::scout_current_estimate, proto::ScoutCurrentEstimate>(
      changed_logical_id.inputs_.current.value, [](auto* current) {
        current->mutable_header()->set_sequence(9U);
        current->set_current_model_id("replacement-current-model");
      });
  bind_current_dependency(&changed_logical_id.inputs_);
  const auto logical_id_result =
      builder.capture(changed_logical_id, configuration, 1'000);
  scout_planner::test_support::require(
      !logical_id_result.has_value() &&
          logical_id_result.error().code ==
              ContextErrorCode::sequence_rollback &&
          logical_id_result.error().input == PlanningInputKind::current,
      "changing a logical ID bypassed the producer/session/stream watermark");

  FixedSource lower_version(valid_inputs());
  lower_version.inputs_.navigation.value = mutate_contract<
      ContractKind::scout_navigation_state, proto::ScoutNavigationState>(
      lower_version.inputs_.navigation.value, [](auto* navigation) {
        navigation->mutable_header()->set_sequence(11U);
        navigation->set_navigation_version(4U);
      });
  bind_navigation_dependency(&lower_version.inputs_);
  const auto version_result =
      builder.capture(lower_version, configuration, 1'000);
  scout_planner::test_support::require(
      !version_result.has_value() &&
          version_result.error().code == ContextErrorCode::version_rollback &&
          version_result.error().input == PlanningInputKind::navigation,
      "business-version rollback entered the planning context");

  FixedSource new_session(valid_inputs());
  new_session.inputs_.navigation.value = mutate_contract<
      ContractKind::scout_navigation_state, proto::ScoutNavigationState>(
      new_session.inputs_.navigation.value, [](auto* navigation) {
        navigation->mutable_header()->set_producer_session_id(
            std::string(16U, '\x6b'));
        navigation->mutable_header()->set_sequence(1U);
      });
  const auto new_session_result =
      builder.capture(new_session, configuration, 1'000);
  scout_planner::test_support::require(
      new_session_result.has_value(),
      "a fresh producer session with unchanged business content was rejected");

  const auto replay_result = builder.capture(initial, configuration, 1'000);
  scout_planner::test_support::require(
      !replay_result.has_value() &&
          replay_result.error().code == ContextErrorCode::session_rollback &&
          replay_result.error().input == PlanningInputKind::navigation,
      "a retired producer session was resurrected");
}

void test_multi_sensor_streams_are_stable_and_publishers_are_canonical() {
  PlanningContextBuilder builder;
  const auto configuration = valid_configuration();
  FixedSource source(valid_inputs());
  add_second_sensor(&source.inputs_);

  scout_planner::test_support::require(
      builder.capture(source, configuration, 1'000).has_value(),
      "initial multi-sensor capture failed");
  scout_planner::test_support::require(
      builder.capture(source, configuration, 1'000).has_value(),
      "an unchanged multi-sensor snapshot self-triggered sequence rollback");

  PlanningContextBuilder unordered_builder;
  FixedSource unordered(valid_inputs());
  add_second_sensor(&unordered.inputs_);
  unordered.inputs_.sensors.front().geometry.value = mutate_contract<
      ContractKind::scout_sensor_geometry, proto::ScoutSensorGeometry>(
      unordered.inputs_.sensors.front().geometry.value, [](auto* geometry) {
        geometry->mutable_header()->set_sequence(11U);
      });
  unordered.inputs_.sensors.front().health.value = mutate_contract<
      ContractKind::scout_sensor_health_state, proto::ScoutSensorHealthState>(
      unordered.inputs_.sensors.front().health.value, [](auto* health) {
        health->mutable_header()->set_sequence(11U);
      });
  unordered.inputs_.sensors.back().geometry.value = mutate_contract<
      ContractKind::scout_sensor_geometry, proto::ScoutSensorGeometry>(
      unordered.inputs_.sensors.back().geometry.value, [](auto* geometry) {
        geometry->mutable_header()->set_sequence(10U);
      });
  unordered.inputs_.sensors.back().health.value = mutate_contract<
      ContractKind::scout_sensor_health_state, proto::ScoutSensorHealthState>(
      unordered.inputs_.sensors.back().health.value, [](auto* health) {
        health->mutable_header()->set_sequence(10U);
      });
  scout_planner::test_support::require(
      unordered_builder.capture(unordered, configuration, 1'000).has_value(),
      "sensor ID order was incorrectly treated as delivery sequence order");

  PlanningContextBuilder behind_stream_builder;
  FixedSource stream_high(valid_inputs());
  add_second_sensor(&stream_high.inputs_);
  stream_high.inputs_.sensors.back().geometry.value = mutate_contract<
      ContractKind::scout_sensor_geometry, proto::ScoutSensorGeometry>(
      stream_high.inputs_.sensors.back().geometry.value, [](auto* geometry) {
        geometry->mutable_header()->set_sequence(20U);
      });
  stream_high.inputs_.sensors.back().health.value = mutate_contract<
      ContractKind::scout_sensor_health_state, proto::ScoutSensorHealthState>(
      stream_high.inputs_.sensors.back().health.value, [](auto* health) {
        health->mutable_header()->set_sequence(20U);
      });
  scout_planner::test_support::require(
      behind_stream_builder.capture(stream_high, configuration, 1'000)
          .has_value(),
      "initial high-watermark sensor capture failed");
  FixedSource behind_stream(stream_high.inputs_);
  behind_stream.inputs_.sensors.front().geometry.value = mutate_contract<
      ContractKind::scout_sensor_geometry, proto::ScoutSensorGeometry>(
      behind_stream.inputs_.sensors.front().geometry.value, [](auto* geometry) {
        geometry->mutable_header()->set_sequence(15U);
        geometry->set_geometry_version(geometry->geometry_version() + 1U);
      });
  bind_sensor_dependencies(&behind_stream.inputs_);
  const auto behind =
      behind_stream_builder.capture(behind_stream, configuration, 1'000);
  scout_planner::test_support::require(
      !behind.has_value() &&
          behind.error().code == ContextErrorCode::sequence_rollback &&
          behind.error().input == PlanningInputKind::sensor_geometry,
      "a new object delivery was accepted behind its stream watermark");

  PlanningContextBuilder mixed_session_builder;
  FixedSource mixed_session(valid_inputs());
  add_second_sensor(&mixed_session.inputs_);
  mixed_session.inputs_.sensors.back().geometry.value = mutate_contract<
      ContractKind::scout_sensor_geometry, proto::ScoutSensorGeometry>(
      mixed_session.inputs_.sensors.back().geometry.value, [](auto* geometry) {
        geometry->mutable_header()->set_producer_session_id(
            std::string(16U, '\x73'));
        geometry->mutable_header()->set_sequence(1U);
      });
  const auto mixed =
      mixed_session_builder.capture(mixed_session, configuration, 1'000);
  scout_planner::test_support::require(
      !mixed.has_value() &&
          mixed.error().code == ContextErrorCode::session_rollback &&
          mixed.error().input == PlanningInputKind::sensor_geometry,
      "one capture mixed producer sessions inside a sensor stream");

  FixedSource competing_publisher(valid_inputs());
  competing_publisher.inputs_.navigation.value = mutate_contract<
      ContractKind::scout_navigation_state, proto::ScoutNavigationState>(
      competing_publisher.inputs_.navigation.value, [](auto* navigation) {
        navigation->mutable_header()->set_producer_id("competing-planner");
        navigation->mutable_header()->set_sequence(11U);
        navigation->set_navigation_version(6U);
      });
  bind_navigation_dependency(&competing_publisher.inputs_);
  const auto competitor =
      builder.capture(competing_publisher, configuration, 1'000);
  scout_planner::test_support::require(
      !competitor.has_value() &&
          competitor.error().code == ContextErrorCode::producer_mismatch &&
          competitor.error().input == PlanningInputKind::navigation,
      "a competing canonical publisher entered an established stream");
}

void test_map_source_clock_and_coordination_sessions_use_correct_domains() {
  PlanningContextBuilder builder;
  const auto configuration = valid_configuration();
  FixedSource initial(valid_inputs());
  scout_planner::test_support::require(
      builder.capture(initial, configuration, 1'000).has_value(),
      "initial domain capture failed");

  FixedSource remote_map(valid_inputs());
  remote_map.inputs_.map.value = mutate_contract<
      ContractKind::hybrid_map_snapshot, proto::HybridMapSnapshot>(
      remote_map.inputs_.map.value, [](auto* map) {
        map->set_source_clock_domain_id("mapping-nuc/boot-9");
        map->set_generated_at_monotonic_ns(9'000'000);
        map->set_map_version(2U);
      });
  bind_map_dependency(&remote_map.inputs_);
  scout_planner::test_support::require(
      builder.capture(remote_map, configuration, 1'000).has_value(),
      "a fresh synchronized map was compared against the wrong monotonic clock");

  FixedSource same_domain_future_map(valid_inputs());
  same_domain_future_map.inputs_.map.value = mutate_contract<
      ContractKind::hybrid_map_snapshot, proto::HybridMapSnapshot>(
      same_domain_future_map.inputs_.map.value, [](auto* map) {
        map->set_generated_at_monotonic_ns(1'001);
        map->set_map_version(3U);
      });
  bind_map_dependency(&same_domain_future_map.inputs_);
  const auto future_map =
      builder.capture(same_domain_future_map, configuration, 1'000);
  scout_planner::test_support::require(
      !future_map.has_value() &&
          future_map.error().code == ContextErrorCode::future_timestamp &&
          future_map.error().input == PlanningInputKind::map,
      "a future map tick in the Scout clock domain entered the context");

  PlanningContextBuilder paired_builder;
  FixedSource paired_initial(valid_inputs());
  scout_planner::test_support::require(
      paired_builder.capture(paired_initial, configuration, 1'000).has_value(),
      "initial paired-session capture failed");

  FixedSource prediction_only(valid_inputs());
  prediction_only.inputs_.main_robot_prediction.value = mutate_contract<
      ContractKind::main_robot_prediction, proto::MainRobotPrediction>(
      prediction_only.inputs_.main_robot_prediction.value, [](auto* prediction) {
        prediction->mutable_header()->set_producer_session_id(
            std::string(16U, '\x71'));
        prediction->mutable_header()->set_sequence(1U);
      });
  const auto partial =
      paired_builder.capture(prediction_only, configuration, 1'000);
  scout_planner::test_support::require(
      !partial.has_value() && partial.error().code == ContextErrorCode::session_rollback &&
          partial.error().input == PlanningInputKind::coordination,
      "prediction-only session refresh spliced old coordination into context");

  FixedSource both_refreshed(valid_inputs());
  both_refreshed.inputs_.main_robot_prediction.value = mutate_contract<
      ContractKind::main_robot_prediction, proto::MainRobotPrediction>(
      both_refreshed.inputs_.main_robot_prediction.value, [](auto* prediction) {
        prediction->mutable_header()->set_producer_session_id(
            std::string(16U, '\x71'));
        prediction->mutable_header()->set_sequence(1U);
      });
  both_refreshed.inputs_.coordination.value = mutate_contract<
      ContractKind::scout_coordination_constraint,
      proto::ScoutCoordinationConstraint>(
      both_refreshed.inputs_.coordination.value, [](auto* coordination) {
        coordination->mutable_header()->set_producer_session_id(
            std::string(16U, '\x72'));
        coordination->mutable_header()->set_sequence(1U);
      });
  scout_planner::test_support::require(
      paired_builder.capture(both_refreshed, configuration, 1'000).has_value(),
      "an atomic prediction/coordination session refresh was rejected");
}

void test_published_context_is_immutable_and_rejects_future_source_ticks() {
  PlanningContextBuilder builder;
  const auto configuration = valid_configuration();
  FixedSource source(valid_inputs());

  const auto captured = builder.capture(source, configuration, 1'000);
  scout_planner::test_support::require(captured.has_value(),
                                       "valid immutable context was rejected");
  const auto captured_navigation_identity =
      captured.value().inputs().navigation.value.canonical_wire_sha256();
  source.inputs_.navigation.value = mutate_contract<
      ContractKind::scout_navigation_state, proto::ScoutNavigationState>(
      source.inputs_.navigation.value, [](auto* navigation) {
        navigation->mutable_header()->set_sequence(11U);
        navigation->set_navigation_version(6U);
      });
  scout_planner::test_support::require(
      captured.value().inputs().navigation.value.canonical_wire_sha256() ==
          captured_navigation_identity,
      "published context changed after the latest-input cache advanced");
  scout_planner::test_support::require(
      captured.value().configuration().activation_generation == 1U &&
          captured.value().captured_at_monotonic_ns() == 1'000 &&
          captured.value().source_generation() == 1U,
      "published context did not retain its configuration and capture identity");

  PlanningContextBuilder future_builder;
  FixedSource future_source(valid_inputs());
  future_source.inputs_.navigation.value = mutate_contract<
      ContractKind::scout_navigation_state, proto::ScoutNavigationState>(
      future_source.inputs_.navigation.value, [](auto* navigation) {
        navigation->set_observed_at_monotonic_ns(1'001);
      });
  bind_navigation_dependency(&future_source.inputs_);
  const auto future =
      future_builder.capture(future_source, configuration, 1'000);
  scout_planner::test_support::require(
      !future.has_value() &&
          future.error().code == ContextErrorCode::future_timestamp &&
          future.error().input == PlanningInputKind::navigation,
      "a future source observation tick entered the context");

  PlanningContextBuilder not_yet_valid_builder;
  FixedSource not_yet_valid(valid_inputs());
  not_yet_valid.inputs_.current.value = mutate_contract<
      ContractKind::scout_current_estimate, proto::ScoutCurrentEstimate>(
      not_yet_valid.inputs_.current.value, [](auto* current) {
        current->set_valid_from_monotonic_ns(1'001);
      });
  bind_current_dependency(&not_yet_valid.inputs_);
  const auto validity = not_yet_valid_builder.capture(
      not_yet_valid, configuration, 1'000);
  scout_planner::test_support::require(
      !validity.has_value() &&
          validity.error().code == ContextErrorCode::future_timestamp &&
          validity.error().input == PlanningInputKind::current,
      "a current model used before its local validity window entered context");
}

void test_capability_energy_gate_is_hard_and_current_aware() {
  PlanningContextBuilder builder;
  FixedSource source(valid_inputs());
  source.inputs_.energy_state.value = mutate_contract<
      ContractKind::scout_energy_state, proto::ScoutEnergyState>(
      source.inputs_.energy_state.value,
      [](auto* energy) { energy->set_available_energy_j(10.0); });
  bind_energy_dependency(&source.inputs_);
  source.inputs_.current.value = mutate_contract<
      ContractKind::scout_current_estimate, proto::ScoutCurrentEstimate>(
      source.inputs_.current.value, [](auto* current) {
        current->mutable_component_error_bound_mps()->set_x_mps(0.0);
        current->mutable_component_error_bound_mps()->set_y_mps(0.0);
        current->mutable_component_error_bound_mps()->set_z_mps(0.0);
        current->set_speed_error_bound_mps(0.0);
      });
  bind_current_dependency(&source.inputs_);
  const auto captured = builder.capture(source, valid_configuration(), 1'000);
  scout_planner::test_support::require(
      captured.has_value(),
      captured.has_value() ? "gate fixture context was rejected"
                           : captured.error().detail);

  const auto feasible = scout_planner::core::CapabilityEnergyGate::evaluate(
      captured.value(),
      {{0.1, {0.5, 0.5, 0.5}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0,
        0.0, 0.0, 0.0, 0.0}},
      {0.0, 0.0, 0.0});
  scout_planner::test_support::require(
      feasible.has_value(),
      feasible.has_value() ? "a bounded fixture trajectory was rejected"
                           : feasible.error().detail);
  scout_planner::test_support::require(
      feasible.value().minimum_capability_margin >= 0.0 &&
          feasible.value().energy_margin_j >= 0.0,
      "successful gate did not expose conservative margins");

  const auto capability_failure =
      scout_planner::core::CapabilityEnergyGate::evaluate(
          captured.value(),
          {{0.1, {0.5, 0.5, 0.5}, {3.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0,
            0.0, 0.0, 0.0, 0.0}},
          {0.0, 0.0, 0.0});
  scout_planner::test_support::require(
      !capability_failure.has_value() &&
          capability_failure.error().code ==
              scout_planner::core::CapabilityEnergyFailure::capability_infeasible,
      "capability overflow was treated as a soft cost");

  const auto energy_failure =
      scout_planner::core::CapabilityEnergyGate::evaluate(
          captured.value(),
          {{10.0, {0.5, 0.5, 0.5}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0,
            0.0, 0.0, 0.0, 0.0}},
          {0.0, 0.0, 0.0});
  scout_planner::test_support::require(
      !energy_failure.has_value() &&
          energy_failure.error().code ==
              scout_planner::core::CapabilityEnergyFailure::energy_insufficient,
      "energy reserve was not a hard gate");

  const auto outside_current =
      scout_planner::core::CapabilityEnergyGate::evaluate(
          captured.value(),
          {{0.1, {10.0, 0.5, 0.5}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0,
            0.0, 0.0, 0.0, 0.0}},
          {0.0, 0.0, 0.0});
  scout_planner::test_support::require(
      !outside_current.has_value() &&
          outside_current.error().code ==
              scout_planner::core::CapabilityEnergyFailure::current_outside_region,
      "a trajectory outside the current model region was accepted");

  const auto invalid_margin =
      scout_planner::core::CapabilityEnergyGate::evaluate(
          captured.value(),
          {{0.1, {0.5, 0.5, 0.5}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0,
            0.0, 0.0, 0.0, 0.0}},
          {-1.0, 0.0, 0.0});
  scout_planner::test_support::require(
      !invalid_margin.has_value() &&
          invalid_margin.error().code ==
              scout_planner::core::CapabilityEnergyFailure::invalid_trajectory,
      "an uncalibrated negative safety margin was accepted");

  FixedSource degraded_source(source.inputs_);
  degraded_source.inputs_.current.value = mutate_contract<
      ContractKind::scout_current_estimate, proto::ScoutCurrentEstimate>(
      degraded_source.inputs_.current.value, [](auto* current) {
        current->set_validity(proto::CURRENT_ESTIMATE_DEGRADED);
      });
  bind_current_dependency(&degraded_source.inputs_);
  PlanningContextBuilder degraded_builder;
  const auto degraded_context =
      degraded_builder.capture(degraded_source, valid_configuration(), 1'000);
  scout_planner::test_support::require(
      degraded_context.has_value(),
      "degraded current fixture context was rejected before gate evaluation");
  const auto degraded_gate = scout_planner::core::CapabilityEnergyGate::evaluate(
      degraded_context.value(),
      {{0.1, {0.5, 0.5, 0.5}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0,
        0.0, 0.0, 0.0, 0.0}},
      {0.0, 0.0, 0.0});
  scout_planner::test_support::require(
      !degraded_gate.has_value() &&
          degraded_gate.error().code ==
              scout_planner::core::CapabilityEnergyFailure::current_invalid,
      "degraded current incorrectly authorized new exploration");
}

void test_plan_evidence_rejects_invalid_sampling_configuration() {
  FixedSource source(valid_inputs());
  PlanningContextBuilder builder;
  const auto captured = builder.capture(source, valid_configuration(), 1'000);
  scout_planner::test_support::require(captured.has_value(),
                                      "evidence fixture context was rejected");
  scout_planner::core::QuinticBezierSegment4d segment;
  segment.duration_ns = 1'000'000'000U;
  segment.position_control_points.fill({0.5, 0.5, 0.5});
  segment.yaw_offset_control_points_rad.fill(0.0);
  const auto trajectory = scout_planner::core::BezierTrajectory4d::create(
      "mission_enu", 0.0, {segment});
  scout_planner::test_support::require(trajectory.has_value(),
                                      "evidence fixture trajectory failed");
  auto evidence_config = scout_planner::core::SurveyPlanEvidenceConfig{};
  evidence_config.sample_period_s = 0.0;
  const auto result =
      scout_planner::core::SurveyPlanEvidenceEvaluator::evaluate(
          captured.value(), trajectory.value(), evidence_config);
  scout_planner::test_support::require(
      !result.has_value() &&
          result.error().code ==
              scout_planner::core::SurveyPlanEvidenceFailure::invalid_input,
      "plan evidence accepted an invalid sampling configuration");
}

}  // namespace

int main() {
  try {
    test_missing_input_set_is_rejected_as_a_whole();
    test_local_receipt_age_boundaries_fail_closed();
    test_missing_sensor_and_capture_race_reject_the_whole_snapshot();
    test_clock_and_dependency_mismatches_fail_closed();
    test_pairing_operating_domain_and_synchronization_fail_closed();
    test_session_sequence_and_version_watermarks_fail_closed();
    test_multi_sensor_streams_are_stable_and_publishers_are_canonical();
    test_map_source_clock_and_coordination_sessions_use_correct_domains();
    test_published_context_is_immutable_and_rejects_future_source_ticks();
    test_capability_energy_gate_is_hard_and_current_aware();
    test_plan_evidence_rejects_invalid_sampling_configuration();
    std::cout << "planning_context: deterministic capture gates passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << failure_context << "; error=" << error.what() << '\n';
    return 1;
  }
}
