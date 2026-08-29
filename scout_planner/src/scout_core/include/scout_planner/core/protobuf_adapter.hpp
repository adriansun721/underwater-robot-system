#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace scout_planner::core {

enum class ContractKind {
  scout_mission,
  scout_mission_decision,
  hybrid_map_snapshot,
  scout_navigation_state,
  scout_sensor_geometry,
  scout_sensor_health_state,
  scout_current_estimate,
  scout_capability_profile,
  scout_thruster_health_state,
  scout_energy_model_profile,
  scout_energy_state,
  main_robot_prediction,
  scout_coordination_constraint,
  scout_trajectory_4d,
  scout_plan_validation_report,
  scout_planning_dependencies,
  scout_plan,
  scout_planning_result,
  scout_execution_lease,
  scout_authorized_execution_bundle,
  count,
};

enum class AdapterErrorCode {
  none,
  parse_failed,
  schema_mismatch,
  unknown_field,
  unknown_enum,
  missing_safety_field,
  non_finite_number,
  invalid_frame,
  invalid_time,
  invalid_version,
  version_rollback,
  invalid_covariance,
  invalid_content_identity,
  invalid_text,
  resource_limit_exceeded,
  invalid_numeric_boundary,
  invalid_structure,
  type_mismatch,
  serialization_failed,
};

struct AdapterError {
  AdapterErrorCode code{AdapterErrorCode::none};
  std::string path;
  std::string message;
};

struct TextValue {
  std::string value;
};

struct BytesValue {
  std::vector<std::uint8_t> value;
};

struct EnumValue {
  std::string type_name;
  std::int32_t number{};
};

struct CoreMessage;
using CoreMessagePtr = std::shared_ptr<const CoreMessage>;
using CoreAtom =
    std::variant<std::int32_t, std::int64_t, std::uint32_t, std::uint64_t,
                 float, double, bool, TextValue, BytesValue, EnumValue,
                 CoreMessagePtr>;

enum class FieldCardinality { singular, repeated };

struct CoreField {
  std::uint32_t number{};
  std::string name;
  FieldCardinality cardinality{FieldCardinality::singular};
  std::vector<CoreAtom> values;
};

struct CoreMessage {
  std::string schema_name;
  std::vector<CoreField> fields;
};

using Hash256 = std::array<std::uint8_t, 32>;
using VersionFloor = std::map<std::string, std::uint64_t, std::less<>>;

// ROS-free projection of the activated public InterfaceLimits fields consumed
// at this boundary. Zero selects the adapter's absolute non-production ceiling.
struct InterfaceLimits {
  std::size_t maximum_repeated_items{0};
  std::size_t maximum_string_bytes{0};
  std::size_t maximum_diagnostics{0};
  std::size_t maximum_map_cells_per_layer{0};
  std::size_t maximum_map_semantic_regions{0};
  std::size_t maximum_sensors_per_context{0};
  std::size_t maximum_thrusters_per_vehicle{0};
  std::size_t maximum_prediction_intervals{0};
  std::size_t maximum_scout_plan_segments{0};
};

struct AdapterValidationOptions {
  const VersionFloor* version_floor{nullptr};
  // Zero selects the contract's absolute safety ceiling. Production callers
  // should pass the smaller limit from their activated InterfaceLimits.
  std::size_t maximum_message_bytes{0};
  const InterfaceLimits* interface_limits{nullptr};
};

template <typename T>
class AdapterResult {
 public:
  static AdapterResult success(T value) {
    return AdapterResult(std::move(value));
  }

  static AdapterResult failure(AdapterError error) {
    return AdapterResult(std::move(error));
  }

  [[nodiscard]] bool has_value() const noexcept {
    return std::holds_alternative<T>(storage_);
  }

  [[nodiscard]] const T& value() const { return std::get<T>(storage_); }
  [[nodiscard]] T& value() { return std::get<T>(storage_); }

  [[nodiscard]] const AdapterError& error() const noexcept {
    static const AdapterError no_error{};
    const auto* error = std::get_if<AdapterError>(&storage_);
    return error == nullptr ? no_error : *error;
  }

 private:
  explicit AdapterResult(T value) : storage_(std::move(value)) {}
  explicit AdapterResult(AdapterError error) : storage_(std::move(error)) {}

  std::variant<T, AdapterError> storage_;
};

template <ContractKind Kind>
class CoreContract final {
 public:
  CoreContract() = delete;

  [[nodiscard]] const CoreMessage& document() const noexcept {
    return document_;
  }

  [[nodiscard]] const Hash256& canonical_wire_sha256() const noexcept {
    return canonical_wire_sha256_;
  }

 private:
  friend class ProtobufAdapter;

  CoreContract(CoreMessage document, Hash256 canonical_wire_sha256)
      : document_(std::move(document)),
        canonical_wire_sha256_(canonical_wire_sha256) {}

  CoreMessage document_;
  Hash256 canonical_wire_sha256_;
};

using SurveyTask = CoreContract<ContractKind::scout_mission>;
using ScoutMissionDecision =
    CoreContract<ContractKind::scout_mission_decision>;
using HybridMapSnapshot = CoreContract<ContractKind::hybrid_map_snapshot>;
using ScoutNavigationState =
    CoreContract<ContractKind::scout_navigation_state>;
using ScoutSensorGeometry =
    CoreContract<ContractKind::scout_sensor_geometry>;
using ScoutSensorHealthState =
    CoreContract<ContractKind::scout_sensor_health_state>;
using ScoutCurrentEstimate =
    CoreContract<ContractKind::scout_current_estimate>;
using ScoutCapabilityProfile =
    CoreContract<ContractKind::scout_capability_profile>;
using ScoutThrusterHealthState =
    CoreContract<ContractKind::scout_thruster_health_state>;
using ScoutEnergyModelProfile =
    CoreContract<ContractKind::scout_energy_model_profile>;
using ScoutEnergyState = CoreContract<ContractKind::scout_energy_state>;
using MainRobotPrediction =
    CoreContract<ContractKind::main_robot_prediction>;
using CoordinationConstraint =
    CoreContract<ContractKind::scout_coordination_constraint>;
using CanonicalTrajectory4d =
    CoreContract<ContractKind::scout_trajectory_4d>;
using ScoutPlanValidationReport =
    CoreContract<ContractKind::scout_plan_validation_report>;
using ScoutPlanningDependencies =
    CoreContract<ContractKind::scout_planning_dependencies>;
using ScoutPlan = CoreContract<ContractKind::scout_plan>;
using ScoutPlanningResult =
    CoreContract<ContractKind::scout_planning_result>;
using ScoutExecutionLease =
    CoreContract<ContractKind::scout_execution_lease>;
using ScoutAuthorizedExecutionBundle =
    CoreContract<ContractKind::scout_authorized_execution_bundle>;

class ProtobufAdapter final {
 public:
  template <ContractKind Kind>
  [[nodiscard]] static AdapterResult<CoreContract<Kind>> decode(
      const std::string_view wire,
      const AdapterValidationOptions& options = {}) {
    auto decoded = decode_document(Kind, wire, options);
    if (!decoded.has_value()) {
      return AdapterResult<CoreContract<Kind>>::failure(decoded.error());
    }
    auto value = std::move(decoded.value());
    return AdapterResult<CoreContract<Kind>>::success(
        CoreContract<Kind>(std::move(value.document), value.sha256));
  }

  template <ContractKind Kind>
  [[nodiscard]] static AdapterResult<std::string> encode(
      const CoreContract<Kind>& value) {
    return encode_document(Kind, value.document());
  }

  template <ContractKind Kind>
  [[nodiscard]] static AdapterResult<CoreContract<Kind>> create(
      CoreMessage document) {
    auto encoded = encode_document(Kind, document);
    if (!encoded.has_value()) {
      return AdapterResult<CoreContract<Kind>>::failure(encoded.error());
    }
    return decode<Kind>(encoded.value());
  }

  template <ContractKind Kind>
  [[nodiscard]] static AdapterResult<Hash256> canonical_content_identity(
      const std::string_view wire,
      const AdapterValidationOptions& options = {}) {
    return canonical_content_identity_for(Kind, wire, options);
  }

  // Producer-side canonicalization: validate the document, recompute every
  // nested self identity from the leaves upward, and emit deterministic wire.
  template <ContractKind Kind>
  [[nodiscard]] static AdapterResult<std::string> canonicalize_and_identify(
      const std::string_view wire,
      const AdapterValidationOptions& options = {}) {
    return canonicalize_and_identify_document(Kind, wire, options);
  }

 private:
  struct DecodedDocument {
    CoreMessage document;
    Hash256 sha256;
  };

  [[nodiscard]] static AdapterResult<DecodedDocument> decode_document(
      ContractKind kind, std::string_view wire,
      const AdapterValidationOptions& options);
  [[nodiscard]] static AdapterResult<std::string> encode_document(
      ContractKind kind, const CoreMessage& document);
  [[nodiscard]] static AdapterResult<Hash256> canonical_content_identity_for(
      ContractKind kind, std::string_view wire,
      const AdapterValidationOptions& options);
  [[nodiscard]] static AdapterResult<std::string>
  canonicalize_and_identify_document(ContractKind kind, std::string_view wire,
                                     const AdapterValidationOptions& options);
};

}  // namespace scout_planner::core
