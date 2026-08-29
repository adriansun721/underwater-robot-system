#include "scout_planner/core/planning_context.hpp"

#include "underwater/contracts/v1/capability.pb.h"
#include "underwater/contracts/v1/cooperation.pb.h"
#include "underwater/contracts/v1/mapping.pb.h"
#include "underwater/contracts/v1/planning.pb.h"
#include "underwater/contracts/v1/sensing.pb.h"
#include "underwater/contracts/v1/state.pb.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

namespace scout_planner::core {

namespace {

namespace proto = underwater::contracts::v1;

[[nodiscard]] ContextError make_error(const ContextErrorCode code,
                                      const PlanningInputKind input,
                                      std::string detail) {
  return ContextError{code, input, std::move(detail)};
}

[[nodiscard]] std::optional<ContextError> validate_receipt(
    const PlanningInputKind kind, const InputReceipt receipt,
    const ActivatedPlanningConfiguration& configuration,
    const std::int64_t captured_at_monotonic_ns) {
  if (receipt.local_received_at_monotonic_ns > captured_at_monotonic_ns) {
    return make_error(ContextErrorCode::future_receipt, kind,
                      "the local receipt time is later than capture time");
  }
  if (receipt.local_received_at_monotonic_ns < 0) {
    return make_error(ContextErrorCode::future_timestamp, kind,
                      "the local receipt time is negative");
  }
  const auto age = static_cast<std::uint64_t>(captured_at_monotonic_ns) -
                   static_cast<std::uint64_t>(
                       receipt.local_received_at_monotonic_ns);
  const auto maximum_age = configuration.maximum_input_age_ns.at(
      static_cast<std::size_t>(kind));
  if (age > maximum_age) {
    return make_error(ContextErrorCode::stale_input, kind,
                      "the local receive age exceeds its configured maximum");
  }
  return std::nullopt;
}

template <ContractKind Kind, typename Message>
[[nodiscard]] std::optional<Message> to_protobuf(
    const CoreContract<Kind>& value) {
  const auto encoded = ProtobufAdapter::encode(value);
  if (!encoded.has_value()) {
    return std::nullopt;
  }
  Message message;
  if (!message.ParseFromString(encoded.value())) {
    return std::nullopt;
  }
  return message;
}

[[nodiscard]] bool same_identity(const proto::ContentIdentity& left,
                                 const proto::ContentIdentity& right) {
  return left.sha256() == right.sha256();
}

[[nodiscard]] std::string hash_bytes(const Hash256& hash) {
  return std::string(reinterpret_cast<const char*>(hash.data()), hash.size());
}

[[nodiscard]] bool matches_binding(const proto::ProfileRef& profile,
                                   const ProfileBinding& binding) {
  return profile.profile_id() == binding.id &&
         profile.version() == binding.version &&
         profile.content_identity().sha256() ==
             hash_bytes(binding.content_identity);
}

[[nodiscard]] bool same_profile(const proto::ProfileRef& left,
                                const proto::ProfileRef& right) {
  return left.profile_id() == right.profile_id() &&
         left.version() == right.version() &&
         same_identity(left.content_identity(), right.content_identity());
}

[[nodiscard]] std::optional<ContextError> validate_synchronized_time(
    const proto::SynchronizedObservationTime& observed,
    const PlanningInputKind kind,
    const ActivatedPlanningConfiguration& configuration,
    std::vector<std::int64_t>* synchronized_times) {
  if (observed.status() != proto::TIME_SYNC_SYNCHRONIZED ||
      !observed.has_utc_time_ns() || !observed.has_uncertainty_ns() ||
      observed.uncertainty_ns() >
          configuration.maximum_synchronization_uncertainty_ns) {
    return make_error(
        ContextErrorCode::synchronization_invalid, kind,
        "a critical observation lacks valid bounded synchronization");
  }
  synchronized_times->push_back(observed.utc_time_ns());
  return std::nullopt;
}

[[nodiscard]] std::optional<ContextError> validate_local_window(
    const std::int64_t observed_at, const std::int64_t valid_until,
    const std::int64_t captured_at, const PlanningInputKind kind) {
  if (observed_at > captured_at) {
    return make_error(ContextErrorCode::future_timestamp, kind,
                      "an input observation tick is later than capture time");
  }
  if (captured_at > valid_until) {
    return make_error(ContextErrorCode::stale_input, kind,
                      "an input local validity window expired before capture");
  }
  return std::nullopt;
}

struct WatermarkCandidate {
  std::string delivery_key;
  std::string business_key;
  PlanningInputKind input;
  std::string producer_id;
  std::string producer_session_id;
  std::uint64_t sequence;
  std::uint64_t business_version;
  std::string business_identity;
  Hash256 wire_identity;
};

template <ContractKind Kind>
[[nodiscard]] WatermarkCandidate make_watermark_candidate(
    const PlanningInputKind input, std::string logical_id,
    const proto::MessageHeader& header, const std::uint64_t business_version,
    const proto::ContentIdentity& business_identity,
    const CoreContract<Kind>& value) {
  return WatermarkCandidate{
      std::to_string(static_cast<int>(header.stream_id())),
      std::to_string(static_cast<std::size_t>(input)) + ":" + logical_id,
      input,
      header.producer_id(),
      header.producer_session_id(),
      header.sequence(),
      business_version,
      business_identity.sha256(),
      value.canonical_wire_sha256(),
  };
}

}  // namespace

PlanningContextResult PlanningContextBuilder::capture(
    const PlanningContextSource& source,
    const ActivatedPlanningConfiguration& configuration,
    const std::int64_t captured_at_monotonic_ns) {
  const auto generation_before = source.capture_generation();
  auto inputs = source.read_inputs();
  if (!inputs.has_value()) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::missing_input, PlanningInputKind::mission,
        "the planning source did not provide a complete input set"));
  }

  const auto generation_after = source.capture_generation();
  if (generation_before != generation_after) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::capture_race, PlanningInputKind::dependencies,
        "the planning input generation changed during capture"));
  }

  if (configuration.scout_clock_domain_id.empty() ||
      configuration.operating_domain_id.empty() ||
      configuration.activation_generation == 0U ||
      captured_at_monotonic_ns < 0 ||
      std::any_of(configuration.maximum_input_age_ns.begin(),
                  configuration.maximum_input_age_ns.end(),
                  [](const auto maximum_age) { return maximum_age == 0U; })) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::invalid_configuration,
        PlanningInputKind::dependencies,
        "the activated planning configuration is incomplete"));
  }
  if (inputs->sensors.empty()) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::missing_input, PlanningInputKind::sensor_geometry,
        "the planning context has no paired sensor geometry and health"));
  }

  const std::array<std::pair<PlanningInputKind, InputReceipt>, 12U> receipts{{
      {PlanningInputKind::mission, inputs->mission.mission_receipt},
      {PlanningInputKind::mission, inputs->mission.decision_receipt},
      {PlanningInputKind::navigation, inputs->navigation.receipt},
      {PlanningInputKind::map, inputs->map.receipt},
      {PlanningInputKind::current, inputs->current.receipt},
      {PlanningInputKind::capability, inputs->capability.receipt},
      {PlanningInputKind::thruster_health, inputs->thruster_health.receipt},
      {PlanningInputKind::energy_model, inputs->energy_model.receipt},
      {PlanningInputKind::energy_state, inputs->energy_state.receipt},
      {PlanningInputKind::main_robot_prediction,
       inputs->main_robot_prediction.receipt},
      {PlanningInputKind::coordination, inputs->coordination.receipt},
      {PlanningInputKind::dependencies, inputs->dependencies.receipt},
  }};
  for (const auto& [kind, receipt] : receipts) {
    if (const auto error = validate_receipt(kind, receipt, configuration,
                                            captured_at_monotonic_ns)) {
      return PlanningContextResult::failure(error.value());
    }
  }
  for (const auto& sensor : inputs->sensors) {
    if (const auto error =
            validate_receipt(PlanningInputKind::sensor_geometry,
                             sensor.geometry.receipt, configuration,
                             captured_at_monotonic_ns)) {
      return PlanningContextResult::failure(error.value());
    }
    if (const auto error =
            validate_receipt(PlanningInputKind::sensor_health,
                             sensor.health.receipt, configuration,
                             captured_at_monotonic_ns)) {
      return PlanningContextResult::failure(error.value());
    }
  }

  const auto mission = to_protobuf<ContractKind::scout_mission,
                                   proto::ScoutMission>(inputs->mission.mission);
  const auto admission = to_protobuf<ContractKind::scout_mission_decision,
                                     proto::ScoutMissionDecision>(
      inputs->mission.decision);
  const auto navigation = to_protobuf<ContractKind::scout_navigation_state,
                                      proto::ScoutNavigationState>(
      inputs->navigation.value);
  const auto map = to_protobuf<ContractKind::hybrid_map_snapshot,
                               proto::HybridMapSnapshot>(inputs->map.value);
  const auto current = to_protobuf<ContractKind::scout_current_estimate,
                                   proto::ScoutCurrentEstimate>(
      inputs->current.value);
  const auto capability = to_protobuf<ContractKind::scout_capability_profile,
                                      proto::ScoutCapabilityProfile>(
      inputs->capability.value);
  const auto thruster = to_protobuf<ContractKind::scout_thruster_health_state,
                                    proto::ScoutThrusterHealthState>(
      inputs->thruster_health.value);
  const auto energy_model =
      to_protobuf<ContractKind::scout_energy_model_profile,
                  proto::ScoutEnergyModelProfile>(inputs->energy_model.value);
  const auto energy_state = to_protobuf<ContractKind::scout_energy_state,
                                        proto::ScoutEnergyState>(
      inputs->energy_state.value);
  const auto prediction = to_protobuf<ContractKind::main_robot_prediction,
                                      proto::MainRobotPrediction>(
      inputs->main_robot_prediction.value);
  const auto coordination =
      to_protobuf<ContractKind::scout_coordination_constraint,
                  proto::ScoutCoordinationConstraint>(
          inputs->coordination.value);
  const auto dependencies = to_protobuf<
      ContractKind::scout_planning_dependencies,
      proto::ScoutPlanningDependencies>(inputs->dependencies.value);
  if (!mission || !admission || !navigation || !map || !current ||
      !capability || !thruster || !energy_model || !energy_state ||
      !prediction || !coordination || !dependencies) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::dependency_mismatch, PlanningInputKind::dependencies,
        "validated core input could not be reconstructed for context capture"));
  }

  std::vector<std::pair<proto::ScoutSensorGeometry,
                        proto::ScoutSensorHealthState>>
      sensors;
  sensors.reserve(inputs->sensors.size());
  for (const auto& input_sensor : inputs->sensors) {
    auto geometry =
        to_protobuf<ContractKind::scout_sensor_geometry,
                    proto::ScoutSensorGeometry>(input_sensor.geometry.value);
    auto health =
        to_protobuf<ContractKind::scout_sensor_health_state,
                    proto::ScoutSensorHealthState>(input_sensor.health.value);
    if (!geometry || !health) {
      return PlanningContextResult::failure(make_error(
          ContextErrorCode::dependency_mismatch,
          PlanningInputKind::sensor_geometry,
          "a sensor pair could not be reconstructed for context capture"));
    }
    sensors.emplace_back(std::move(geometry.value()), std::move(health.value()));
  }

  const auto require_scout_clock = [&](const proto::MessageHeader& header,
                                       const PlanningInputKind kind)
      -> std::optional<ContextError> {
    if (header.source_clock_domain_id() !=
        configuration.scout_clock_domain_id) {
      return make_error(ContextErrorCode::clock_domain_mismatch, kind,
                        "input does not belong to the activated Scout clock domain");
    }
    if (!header.has_generated_at_monotonic_ns() ||
        header.generated_at_monotonic_ns() > captured_at_monotonic_ns) {
      return make_error(ContextErrorCode::future_timestamp, kind,
                        "input generation time is absent or later than capture");
    }
    return std::nullopt;
  };
  const std::array<std::pair<const proto::MessageHeader*, PlanningInputKind>, 8U>
      local_headers{{
          {&admission->header(), PlanningInputKind::mission},
          {&navigation->header(), PlanningInputKind::navigation},
          {&current->header(), PlanningInputKind::current},
          {&capability->header(), PlanningInputKind::capability},
          {&thruster->header(), PlanningInputKind::thruster_health},
          {&energy_model->header(), PlanningInputKind::energy_model},
          {&energy_state->header(), PlanningInputKind::energy_state},
          {&sensors.front().first.header(), PlanningInputKind::sensor_geometry},
      }};
  for (const auto& [header, kind] : local_headers) {
    if (const auto error = require_scout_clock(*header, kind)) {
      return PlanningContextResult::failure(error.value());
    }
  }
  if (map->source_clock_domain_id() == configuration.scout_clock_domain_id &&
      (!map->has_generated_at_monotonic_ns() ||
       map->generated_at_monotonic_ns() > captured_at_monotonic_ns)) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::future_timestamp, PlanningInputKind::map,
        "map generation time in the Scout clock domain is later than capture"));
  }
  for (const auto& sensor : sensors) {
    if (const auto error = require_scout_clock(
            sensor.first.header(), PlanningInputKind::sensor_geometry)) {
      return PlanningContextResult::failure(error.value());
    }
    if (const auto error = require_scout_clock(
            sensor.second.header(), PlanningInputKind::sensor_health)) {
      return PlanningContextResult::failure(error.value());
    }
  }
  if (prediction->header().source_clock_domain_id() !=
      coordination->header().source_clock_domain_id()) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::clock_domain_mismatch,
        PlanningInputKind::coordination,
        "prediction and coordination source clock domains differ"));
  }

  const std::array<std::tuple<std::int64_t, std::int64_t, PlanningInputKind>, 4U>
      windows{{
          {current->observed_at_monotonic_ns(),
           current->valid_until_monotonic_ns(), PlanningInputKind::current},
          {thruster->observed_at_monotonic_ns(),
           thruster->valid_until_monotonic_ns(),
           PlanningInputKind::thruster_health},
          {energy_state->observed_at_monotonic_ns(),
           energy_state->valid_until_monotonic_ns(),
           PlanningInputKind::energy_state},
          {navigation->observed_at_monotonic_ns(),
           std::numeric_limits<std::int64_t>::max(),
           PlanningInputKind::navigation},
      }};
  for (const auto& [observed_at, valid_until, kind] : windows) {
    if (const auto error = validate_local_window(
            observed_at, valid_until, captured_at_monotonic_ns, kind)) {
      return PlanningContextResult::failure(error.value());
    }
  }
  for (const auto& sensor : sensors) {
    if (const auto error = validate_local_window(
            sensor.second.observed_at_monotonic_ns(),
            sensor.second.valid_until_monotonic_ns(), captured_at_monotonic_ns,
            PlanningInputKind::sensor_health)) {
      return PlanningContextResult::failure(error.value());
    }
  }

  std::vector<std::int64_t> synchronized_times;
  synchronized_times.reserve(5U);
  const std::array<std::pair<const proto::SynchronizedObservationTime*,
                             PlanningInputKind>,
                   5U>
      observations{{
          {&navigation->header().observed_at(), PlanningInputKind::navigation},
          {&map->observed_at(), PlanningInputKind::map},
          {&current->header().observed_at(), PlanningInputKind::current},
          {&prediction->alignment_epoch(),
           PlanningInputKind::main_robot_prediction},
          {&coordination->header().observed_at(),
           PlanningInputKind::coordination},
      }};
  for (const auto& [observation, kind] : observations) {
    if (const auto error = validate_synchronized_time(
            *observation, kind, configuration, &synchronized_times)) {
      return PlanningContextResult::failure(error.value());
    }
  }
  const auto [minimum_time, maximum_time] = std::minmax_element(
      synchronized_times.begin(), synchronized_times.end());
  const auto synchronization_delta = static_cast<std::uint64_t>(
      *maximum_time) - static_cast<std::uint64_t>(*minimum_time);
  if (synchronization_delta >
      configuration.maximum_synchronization_delta_ns) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::synchronization_tolerance_exceeded,
        PlanningInputKind::dependencies,
        "critical observation times exceed synchronization tolerance"));
  }

  const std::array<std::pair<std::string_view, PlanningInputKind>, 5U>
      operating_domains{{
          {current->operating_domain_id(), PlanningInputKind::current},
          {capability->operating_domain_id(), PlanningInputKind::capability},
          {energy_model->operating_domain_id(), PlanningInputKind::energy_model},
          {energy_state->operating_domain_id(), PlanningInputKind::energy_state},
          {sensors.front().first.operating_domain_id(),
           PlanningInputKind::sensor_geometry},
      }};
  for (const auto& [domain, kind] : operating_domains) {
    if (domain != configuration.operating_domain_id) {
      return PlanningContextResult::failure(make_error(
          ContextErrorCode::operating_domain_mismatch, kind,
          "input operating domain differs from activated configuration"));
    }
  }
  for (const auto& sensor : sensors) {
    if (sensor.first.operating_domain_id() !=
        configuration.operating_domain_id) {
      return PlanningContextResult::failure(make_error(
          ContextErrorCode::operating_domain_mismatch,
          PlanningInputKind::sensor_geometry,
          "sensor operating domain differs from activated configuration"));
    }
  }

  const auto mission_matches = [&](const std::uint64_t mission_id,
                                   const std::uint64_t mission_version,
                                   const proto::ContentIdentity& identity) {
    return mission_id == mission->mission_id() &&
           mission_version == mission->mission_version() &&
           same_identity(identity, mission->mission_content_identity());
  };
  if (!mission_matches(prediction->mission_id(), prediction->mission_version(),
                       prediction->mission_content_identity())) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::mission_mismatch,
        PlanningInputKind::main_robot_prediction,
        "main-robot prediction belongs to another mission"));
  }
  if (!mission_matches(coordination->mission_id(),
                       coordination->mission_version(),
                       coordination->mission_content_identity())) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::mission_mismatch, PlanningInputKind::coordination,
        "coordination constraint belongs to another mission"));
  }
  if (admission->disposition() !=
      proto::SCOUT_MISSION_DECISION_ACCEPTED) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::admission_rejected, PlanningInputKind::mission,
        "mission decision did not accept the mission"));
  }
  if (!admission->has_received_at_monotonic_ns() ||
      admission->received_at_monotonic_ns() > captured_at_monotonic_ns) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::future_timestamp, PlanningInputKind::mission,
        "mission admission receive time is absent or later than capture"));
  }
  if (admission->received_at_monotonic_ns() !=
      inputs->mission.mission_receipt.local_received_at_monotonic_ns) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::mission_mismatch, PlanningInputKind::mission,
        "mission admission does not bind the captured local receive time"));
  }
  if (!admission->has_admission_valid_until_monotonic_ns() ||
      admission->admission_valid_until_monotonic_ns() <
          captured_at_monotonic_ns) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::stale_input, PlanningInputKind::mission,
        "mission admission has expired"));
  }
  if (admission->accepted_coordination_version() !=
          mission->coordination_version() ||
      admission->accepted_coordination_version() !=
          coordination->coordination_version()) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::mission_mismatch, PlanningInputKind::mission,
        "mission admission does not bind the accepted coordination version"));
  }
  if (!mission_matches(admission->mission_id(), admission->mission_version(),
                       admission->mission_content_identity())) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::mission_mismatch, PlanningInputKind::mission,
        "mission admission binds another mission identity"));
  }
  if (coordination->prediction_id() != prediction->prediction_id() ||
      coordination->prediction_version() != prediction->prediction_version() ||
      !same_identity(coordination->prediction_content_identity(),
                     prediction->prediction_content_identity())) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::prediction_mismatch,
        PlanningInputKind::coordination,
        "coordination constraint binds another prediction"));
  }
  if (current->valid_from_monotonic_ns() > captured_at_monotonic_ns) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::future_timestamp, PlanningInputKind::current,
        "current model is not yet valid at capture time"));
  }

  if (!mission_matches(dependencies->mission_id(),
                       dependencies->mission_version(),
                       dependencies->mission_content_identity())) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::dependency_mismatch, PlanningInputKind::mission,
        "mission differs from planning dependencies"));
  }
  if (dependencies->map_id() != map->map_id() ||
      dependencies->map_version() != map->map_version() ||
      !same_identity(dependencies->map_content_identity(),
                     map->map_content_identity())) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::dependency_mismatch, PlanningInputKind::map,
        "map differs from planning dependencies"));
  }
  if (dependencies->navigation_version() != navigation->navigation_version() ||
      !same_identity(dependencies->navigation_content_identity(),
                     navigation->navigation_content_identity())) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::dependency_mismatch,
        PlanningInputKind::navigation,
        "navigation version or content identity differs from dependencies"));
  }
  if (dependencies->sensors_size() != static_cast<int>(sensors.size())) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::dependency_mismatch,
        PlanningInputKind::sensor_geometry,
        "sensor pair count differs from planning dependencies"));
  }
  for (std::size_t index = 0; index < sensors.size(); ++index) {
    const auto& [geometry, health] = sensors[index];
    const auto& dependency = dependencies->sensors(static_cast<int>(index));
    if (geometry.sensor_id() != health.sensor_id() ||
        dependency.sensor_id() != geometry.sensor_id() ||
        dependency.geometry_version() != geometry.geometry_version() ||
        !same_identity(dependency.geometry_content_identity(),
                       geometry.geometry_content_identity()) ||
        dependency.health_version() != health.health_version() ||
        !same_identity(dependency.health_content_identity(),
                       health.health_content_identity())) {
      return PlanningContextResult::failure(make_error(
          ContextErrorCode::dependency_mismatch,
          PlanningInputKind::sensor_geometry,
          "sensor pair differs from planning dependencies"));
    }
  }
  if (dependencies->current_model_id() != current->current_model_id() ||
      dependencies->current_model_version() !=
          current->current_model_version() ||
      !same_identity(dependencies->current_content_identity(),
                     current->current_content_identity())) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::dependency_mismatch, PlanningInputKind::current,
        "current estimate differs from planning dependencies"));
  }
  if (dependencies->capability_profile().profile_id() !=
          capability->capability_profile_id() ||
      dependencies->capability_profile().version() !=
          capability->capability_profile_version() ||
      !same_identity(dependencies->capability_profile().content_identity(),
                     capability->capability_content_identity()) ||
      dependencies->thruster_health_version() != thruster->health_version() ||
      !same_identity(dependencies->thruster_health_content_identity(),
                     thruster->health_content_identity()) ||
      !same_profile(thruster->active_capability_profile(),
                    dependencies->capability_profile())) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::dependency_mismatch,
        PlanningInputKind::capability,
        "capability or thruster health differs from planning dependencies"));
  }
  if (dependencies->energy_model().profile_id() !=
          energy_model->energy_model_id() ||
      dependencies->energy_model().version() !=
          energy_model->energy_model_version() ||
      !same_identity(dependencies->energy_model().content_identity(),
                     energy_model->energy_model_content_identity()) ||
      dependencies->energy_store_id() != energy_state->energy_store_id() ||
      dependencies->energy_state_version() !=
          energy_state->energy_state_version() ||
      !same_identity(dependencies->energy_state_content_identity(),
                     energy_state->energy_state_content_identity()) ||
      !same_profile(energy_state->energy_model(),
                    dependencies->energy_model())) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::dependency_mismatch,
        PlanningInputKind::energy_model,
        "energy model or state differs from planning dependencies"));
  }
  if (dependencies->prediction_id() != prediction->prediction_id() ||
      dependencies->prediction_version() != prediction->prediction_version() ||
      !same_identity(dependencies->prediction_content_identity(),
                     prediction->prediction_content_identity()) ||
      dependencies->coordination_version() !=
          coordination->coordination_version() ||
      !same_identity(dependencies->coordination_content_identity(),
                     coordination->coordination_content_identity())) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::dependency_mismatch,
        PlanningInputKind::main_robot_prediction,
        "prediction or coordination differs from planning dependencies"));
  }
  if (!matches_binding(dependencies->planner_configuration(),
                       configuration.planner_configuration) ||
      !matches_binding(dependencies->timing_profile(),
                       configuration.timing_profile) ||
      !matches_binding(dependencies->interface_limits(),
                       configuration.interface_limits)) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::dependency_mismatch,
        PlanningInputKind::dependencies,
        "planning dependencies do not bind the activated configuration"));
  }
  if (!matches_binding(navigation->timing_profile(),
                       configuration.timing_profile)) {
    return PlanningContextResult::failure(make_error(
        ContextErrorCode::dependency_mismatch,
        PlanningInputKind::navigation,
        "navigation does not bind the activated timing profile"));
  }

  std::vector<WatermarkCandidate> watermark_candidates;
  watermark_candidates.reserve(12U + sensors.size() * 2U);
  watermark_candidates.push_back(make_watermark_candidate(
      PlanningInputKind::mission,
      "mission:" + std::to_string(mission->mission_id()),
      mission->header(), mission->mission_version(),
      mission->mission_content_identity(), inputs->mission.mission));
  watermark_candidates.push_back(make_watermark_candidate(
      PlanningInputKind::mission,
      "admission:" + std::to_string(admission->mission_id()),
      admission->header(), admission->mission_version(),
      admission->decision_content_identity(), inputs->mission.decision));
  watermark_candidates.push_back(make_watermark_candidate(
      PlanningInputKind::navigation, "navigation", navigation->header(),
      navigation->navigation_version(),
      navigation->navigation_content_identity(), inputs->navigation.value));
  watermark_candidates.push_back(WatermarkCandidate{
      {},
      std::to_string(static_cast<std::size_t>(PlanningInputKind::map)) + ":" +
          map->map_id(),
      PlanningInputKind::map,
      {},
      {},
      0U,
      map->map_version(),
      map->map_content_identity().sha256(),
      inputs->map.value.canonical_wire_sha256(),
  });
  watermark_candidates.push_back(make_watermark_candidate(
      PlanningInputKind::current, current->current_model_id(),
      current->header(), current->current_model_version(),
      current->current_content_identity(), inputs->current.value));
  for (std::size_t index = 0; index < sensors.size(); ++index) {
    const auto& [geometry, health] = sensors[index];
    watermark_candidates.push_back(make_watermark_candidate(
        PlanningInputKind::sensor_geometry, geometry.sensor_id(),
        geometry.header(), geometry.geometry_version(),
        geometry.geometry_content_identity(),
        inputs->sensors[index].geometry.value));
    watermark_candidates.push_back(make_watermark_candidate(
        PlanningInputKind::sensor_health, health.sensor_id(), health.header(),
        health.health_version(), health.health_content_identity(),
        inputs->sensors[index].health.value));
  }
  watermark_candidates.push_back(make_watermark_candidate(
      PlanningInputKind::capability, capability->capability_profile_id(),
      capability->header(), capability->capability_profile_version(),
      capability->capability_content_identity(), inputs->capability.value));
  watermark_candidates.push_back(make_watermark_candidate(
      PlanningInputKind::thruster_health, thruster->vehicle_id(),
      thruster->header(), thruster->health_version(),
      thruster->health_content_identity(), inputs->thruster_health.value));
  watermark_candidates.push_back(make_watermark_candidate(
      PlanningInputKind::energy_model, energy_model->energy_model_id(),
      energy_model->header(), energy_model->energy_model_version(),
      energy_model->energy_model_content_identity(), inputs->energy_model.value));
  watermark_candidates.push_back(make_watermark_candidate(
      PlanningInputKind::energy_state, energy_state->energy_store_id(),
      energy_state->header(), energy_state->energy_state_version(),
      energy_state->energy_state_content_identity(), inputs->energy_state.value));
  watermark_candidates.push_back(make_watermark_candidate(
      PlanningInputKind::main_robot_prediction, prediction->prediction_id(),
      prediction->header(), prediction->prediction_version(),
      prediction->prediction_content_identity(),
      inputs->main_robot_prediction.value));
  watermark_candidates.push_back(make_watermark_candidate(
      PlanningInputKind::coordination,
      std::to_string(coordination->mission_id()), coordination->header(),
      coordination->coordination_version(),
      coordination->coordination_content_identity(),
      inputs->coordination.value));

  std::stable_sort(
      watermark_candidates.begin(), watermark_candidates.end(),
      [](const auto& left, const auto& right) {
        return std::tie(left.delivery_key, left.sequence, left.business_key) <
               std::tie(right.delivery_key, right.sequence,
                        right.business_key);
      });
  std::map<std::string, std::pair<std::string, std::string>> capture_streams;
  for (const auto& candidate : watermark_candidates) {
    if (candidate.delivery_key.empty()) {
      continue;
    }
    const auto [stream, inserted] = capture_streams.emplace(
        candidate.delivery_key,
        std::make_pair(candidate.producer_id,
                       candidate.producer_session_id));
    if (!inserted && stream->second.first != candidate.producer_id) {
      return PlanningContextResult::failure(make_error(
          ContextErrorCode::producer_mismatch, candidate.input,
          "one capture mixed producers inside a canonical stream"));
    }
    if (!inserted &&
        stream->second.second != candidate.producer_session_id) {
      return PlanningContextResult::failure(make_error(
          ContextErrorCode::session_rollback, candidate.input,
          "one capture mixed producer sessions inside a stream"));
    }
  }

  std::lock_guard<std::mutex> watermark_lock(watermark_mutex_);
  auto staged_delivery_watermarks = delivery_watermarks_;
  auto staged_business_watermarks = business_watermarks_;
  auto staged_retired_sessions = retired_delivery_sessions_;
  auto staged_coordination_sessions = coordination_sessions_;
  const CoordinationSessions current_coordination_sessions{
      prediction->header().producer_session_id(),
      coordination->header().producer_session_id()};
  if (staged_coordination_sessions.has_value()) {
    const bool prediction_changed =
        staged_coordination_sessions->prediction_session_id !=
        current_coordination_sessions.prediction_session_id;
    const bool coordination_changed =
        staged_coordination_sessions->coordination_session_id !=
        current_coordination_sessions.coordination_session_id;
    if (prediction_changed != coordination_changed) {
      return PlanningContextResult::failure(make_error(
          ContextErrorCode::session_rollback, PlanningInputKind::coordination,
          "prediction and coordination sessions did not refresh atomically"));
    }
  }
  staged_coordination_sessions = current_coordination_sessions;

  for (const auto& candidate : watermark_candidates) {
    bool same_session_new_delivery = false;
    bool exact_duplicate = false;
    if (!candidate.delivery_key.empty()) {
      const auto existing =
          staged_delivery_watermarks.find(candidate.delivery_key);
      if (existing == staged_delivery_watermarks.end()) {
        std::map<std::string, DeliveryWatermark::Member> members;
        members.emplace(candidate.business_key,
                        DeliveryWatermark::Member{candidate.sequence,
                                                  candidate.wire_identity});
        staged_delivery_watermarks.emplace(
            candidate.delivery_key,
            DeliveryWatermark{candidate.producer_id,
                              candidate.producer_session_id,
                              candidate.sequence,
                              candidate.wire_identity,
                              std::move(members)});
      } else {
        auto& record = existing->second;
        if (candidate.producer_id != record.producer_id) {
          return PlanningContextResult::failure(make_error(
              ContextErrorCode::producer_mismatch, candidate.input,
              "an established canonical stream changed producer"));
        }
        if (candidate.producer_session_id == record.producer_session_id) {
          const auto member = record.members.find(candidate.business_key);
          if (member != record.members.end()) {
            if (candidate.sequence < member->second.sequence) {
              return PlanningContextResult::failure(make_error(
                  ContextErrorCode::sequence_rollback, candidate.input,
                  "same-session object delivery sequence moved backward"));
            }
            if (candidate.sequence == member->second.sequence) {
              if (candidate.wire_identity != member->second.wire_identity) {
                return PlanningContextResult::failure(make_error(
                    ContextErrorCode::identity_conflict, candidate.input,
                    "same-session duplicate sequence changed complete bytes"));
              }
              exact_duplicate = true;
            } else {
              if (candidate.sequence < record.maximum_sequence) {
                return PlanningContextResult::failure(make_error(
                    ContextErrorCode::sequence_rollback, candidate.input,
                    "a new object delivery arrived behind its stream watermark"));
              }
              if (candidate.sequence == record.maximum_sequence) {
                return PlanningContextResult::failure(make_error(
                    ContextErrorCode::identity_conflict, candidate.input,
                    candidate.wire_identity ==
                            record.maximum_sequence_wire_identity
                        ? "one stream delivery was assigned to multiple objects"
                        : "one stream sequence identified multiple deliveries"));
              }
              same_session_new_delivery = true;
              member->second = DeliveryWatermark::Member{
                  candidate.sequence, candidate.wire_identity};
            }
          } else {
            if (candidate.sequence < record.maximum_sequence) {
              return PlanningContextResult::failure(make_error(
                  ContextErrorCode::sequence_rollback, candidate.input,
                  "a new object arrived behind the stream watermark"));
            }
            if (candidate.sequence == record.maximum_sequence) {
              return PlanningContextResult::failure(make_error(
                  ContextErrorCode::identity_conflict, candidate.input,
                  "one stream sequence identified multiple deliveries"));
            }
            same_session_new_delivery = true;
            record.members.emplace(
                candidate.business_key,
                DeliveryWatermark::Member{candidate.sequence,
                                          candidate.wire_identity});
          }
          if (candidate.sequence > record.maximum_sequence) {
            record.maximum_sequence = candidate.sequence;
            record.maximum_sequence_wire_identity = candidate.wire_identity;
          }
        } else {
          const auto retired =
              staged_retired_sessions.find(candidate.delivery_key);
          if (retired != staged_retired_sessions.end() &&
              retired->second.count(candidate.producer_session_id) != 0U) {
            return PlanningContextResult::failure(make_error(
                ContextErrorCode::session_rollback, candidate.input,
                "a retired producer session attempted to return"));
          }
          staged_retired_sessions[candidate.delivery_key].insert(
              record.producer_session_id);
          std::map<std::string, DeliveryWatermark::Member> members;
          members.emplace(candidate.business_key,
                          DeliveryWatermark::Member{candidate.sequence,
                                                    candidate.wire_identity});
          record = DeliveryWatermark{candidate.producer_id,
                                     candidate.producer_session_id,
                                     candidate.sequence,
                                     candidate.wire_identity,
                                     std::move(members)};
        }
      }
    }

    const auto business =
        staged_business_watermarks.find(candidate.business_key);
    if (business == staged_business_watermarks.end()) {
      staged_business_watermarks.emplace(
          candidate.business_key,
          BusinessWatermark{candidate.business_version,
                            candidate.business_identity});
      continue;
    }
    auto& business_record = business->second;
    if (candidate.business_version < business_record.version) {
      return PlanningContextResult::failure(make_error(
          ContextErrorCode::version_rollback, candidate.input,
          "business version moved backward"));
    }
    if (candidate.business_version == business_record.version) {
      if (candidate.business_identity != business_record.content_identity) {
        return PlanningContextResult::failure(make_error(
            ContextErrorCode::identity_conflict, candidate.input,
            "same business version changed content identity"));
      }
      if (same_session_new_delivery && !exact_duplicate) {
        return PlanningContextResult::failure(make_error(
            ContextErrorCode::identity_conflict, candidate.input,
            "new same-session delivery reused a business version"));
      }
      continue;
    }
    business_record = BusinessWatermark{candidate.business_version,
                                        candidate.business_identity};
  }
  delivery_watermarks_ = std::move(staged_delivery_watermarks);
  business_watermarks_ = std::move(staged_business_watermarks);
  retired_delivery_sessions_ = std::move(staged_retired_sessions);
  coordination_sessions_ = std::move(staged_coordination_sessions);

  return PlanningContextResult::success(ScoutPlanningContext(
      std::move(inputs.value()), configuration, captured_at_monotonic_ns,
      generation_after));
}

}  // namespace scout_planner::core
