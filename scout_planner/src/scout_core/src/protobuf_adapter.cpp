#include "scout_planner/core/protobuf_adapter.hpp"

#include <google/protobuf/descriptor.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>
#include <google/protobuf/message.h>
#include <google/protobuf/unknown_field_set.h>
#include <google/protobuf/util/message_differencer.h>

#include <unicode/unorm2.h>
#include <unicode/ustring.h>

#include "underwater/contracts/v1/capability.pb.h"
#include "underwater/contracts/v1/cooperation.pb.h"
#include "underwater/contracts/v1/execution.pb.h"
#include "underwater/contracts/v1/mapping.pb.h"
#include "underwater/contracts/v1/planning.pb.h"
#include "underwater/contracts/v1/sensing.pb.h"
#include "underwater/contracts/v1/state.pb.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace scout_planner::core {
namespace {

namespace pb = google::protobuf;
namespace contract = underwater::contracts::v1;

constexpr std::string_view kPackage = "underwater.contracts.v1.";

[[nodiscard]] AdapterError make_error(const AdapterErrorCode code,
                                      std::string path,
                                      std::string message) {
  return AdapterError{code, std::move(path), std::move(message)};
}

struct ContractMetadata {
  const pb::Descriptor* descriptor;
  const char* identity_field_name;
  bool clear_header_for_identity;
  std::size_t maximum_wire_bytes;
};

struct IdentityMetadata {
  const pb::Descriptor* descriptor;
  const char* identity_field_name;
  bool clear_header_for_identity;
};

constexpr std::size_t kSmallContractLimit = 65'536U;
constexpr std::size_t kPlanningContractLimit = 262'144U;

const std::array<ContractMetadata,
                 static_cast<std::size_t>(ContractKind::count)>
    kContractMetadata{{
        {contract::ScoutMission::descriptor(), "mission_content_identity", true,
         kSmallContractLimit},
        {contract::ScoutMissionDecision::descriptor(),
         "decision_content_identity", true, kSmallContractLimit},
        {contract::HybridMapSnapshot::descriptor(), "map_content_identity",
         false, 67'108'864U},
        {contract::ScoutNavigationState::descriptor(),
         "navigation_content_identity", true, kSmallContractLimit},
        {contract::ScoutSensorGeometry::descriptor(),
         "geometry_content_identity", true, kSmallContractLimit},
        {contract::ScoutSensorHealthState::descriptor(),
         "health_content_identity", true, kSmallContractLimit},
        {contract::ScoutCurrentEstimate::descriptor(),
         "current_content_identity", true, kSmallContractLimit},
        {contract::ScoutCapabilityProfile::descriptor(),
         "capability_content_identity", true, kSmallContractLimit},
        {contract::ScoutThrusterHealthState::descriptor(),
         "health_content_identity", true, kSmallContractLimit},
        {contract::ScoutEnergyModelProfile::descriptor(),
         "energy_model_content_identity", true, kSmallContractLimit},
        {contract::ScoutEnergyState::descriptor(),
         "energy_state_content_identity", true, kSmallContractLimit},
        {contract::MainRobotPrediction::descriptor(),
         "prediction_content_identity", true, kPlanningContractLimit},
        {contract::ScoutCoordinationConstraint::descriptor(),
         "coordination_content_identity", true, kSmallContractLimit},
        {contract::ScoutTrajectory4d::descriptor(),
         "trajectory_content_identity", false, kPlanningContractLimit},
        {contract::ScoutPlanValidationReport::descriptor(),
         "validation_report_content_identity", false, kPlanningContractLimit},
        {contract::ScoutPlanningDependencies::descriptor(),
         "dependencies_content_identity", false, kPlanningContractLimit},
        {contract::ScoutPlan::descriptor(), "plan_content_identity", false,
         kPlanningContractLimit},
        {contract::ScoutPlanningResult::descriptor(), "result_content_identity",
         true, kPlanningContractLimit},
        {contract::ScoutExecutionLease::descriptor(), "content_identity", false,
         kSmallContractLimit},
        {contract::ScoutAuthorizedExecutionBundle::descriptor(),
         "bundle_content_identity", false, 524'288U},
    }};

[[nodiscard]] const ContractMetadata& contract_metadata(
    const ContractKind kind) {
  static const ContractMetadata invalid{nullptr, nullptr, false, 0};
  const auto index = static_cast<std::size_t>(kind);
  return index < kContractMetadata.size() ? kContractMetadata[index] : invalid;
}

[[nodiscard]] std::optional<IdentityMetadata> identity_metadata(
    const pb::Descriptor& descriptor) {
  if (&descriptor == contract::SurveyPlanEvidence::descriptor()) {
    return IdentityMetadata{&descriptor, "evidence_content_identity", false};
  }
  for (const auto& metadata : kContractMetadata) {
    if (metadata.descriptor == &descriptor) {
      return IdentityMetadata{metadata.descriptor,
                              metadata.identity_field_name,
                              metadata.clear_header_for_identity};
    }
  }
  return std::nullopt;
}

[[nodiscard]] std::string_view schema_name(const ContractKind kind) {
  const auto metadata = contract_metadata(kind);
  return metadata.descriptor == nullptr ? std::string_view{}
                                        : metadata.descriptor->full_name();
}

[[nodiscard]] bool ends_with(const std::string_view value,
                             const std::string_view suffix) {
  return value.size() >= suffix.size() &&
         value.substr(value.size() - suffix.size()) == suffix;
}

[[nodiscard]] AdapterResult<std::string> normalize_nfc(
    const std::string_view input) {
  if (input.size() > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max())) {
    return AdapterResult<std::string>::failure(make_error(
        AdapterErrorCode::invalid_text, {}, "UTF-8 value exceeds ICU limits"));
  }
  UErrorCode status = U_ZERO_ERROR;
  std::int32_t utf16_length = 0;
  u_strFromUTF8(nullptr, 0, &utf16_length, input.data(),
                static_cast<std::int32_t>(input.size()), &status);
  if (status != U_BUFFER_OVERFLOW_ERROR && U_FAILURE(status)) {
    return AdapterResult<std::string>::failure(make_error(
        AdapterErrorCode::invalid_text, {}, "contract text is not valid UTF-8"));
  }
  status = U_ZERO_ERROR;
  std::vector<UChar> utf16(static_cast<std::size_t>(utf16_length) + 1U);
  u_strFromUTF8(utf16.data(), static_cast<std::int32_t>(utf16.size()),
                &utf16_length, input.data(),
                static_cast<std::int32_t>(input.size()), &status);
  if (U_FAILURE(status)) {
    return AdapterResult<std::string>::failure(make_error(
        AdapterErrorCode::invalid_text, {}, "UTF-8 conversion failed"));
  }

  const UNormalizer2* normalizer = unorm2_getNFCInstance(&status);
  std::int32_t normalized_length =
      unorm2_normalize(normalizer, utf16.data(), utf16_length, nullptr, 0,
                       &status);
  if (status != U_BUFFER_OVERFLOW_ERROR && U_FAILURE(status)) {
    return AdapterResult<std::string>::failure(make_error(
        AdapterErrorCode::invalid_text, {}, "Unicode NFC preflight failed"));
  }
  status = U_ZERO_ERROR;
  std::vector<UChar> normalized(
      static_cast<std::size_t>(normalized_length) + 1U);
  normalized_length = unorm2_normalize(
      normalizer, utf16.data(), utf16_length, normalized.data(),
      static_cast<std::int32_t>(normalized.size()), &status);
  if (U_FAILURE(status)) {
    return AdapterResult<std::string>::failure(make_error(
        AdapterErrorCode::invalid_text, {}, "Unicode NFC normalization failed"));
  }

  std::int32_t utf8_length = 0;
  status = U_ZERO_ERROR;
  u_strToUTF8(nullptr, 0, &utf8_length, normalized.data(), normalized_length,
              &status);
  if (status != U_BUFFER_OVERFLOW_ERROR && U_FAILURE(status)) {
    return AdapterResult<std::string>::failure(make_error(
        AdapterErrorCode::invalid_text, {}, "normalized UTF-8 preflight failed"));
  }
  status = U_ZERO_ERROR;
  std::string result(static_cast<std::size_t>(utf8_length), '\0');
  u_strToUTF8(result.data(), utf8_length, &utf8_length, normalized.data(),
              normalized_length, &status);
  if (U_FAILURE(status)) {
    return AdapterResult<std::string>::failure(make_error(
        AdapterErrorCode::invalid_text, {}, "normalized UTF-8 conversion failed"));
  }
  return AdapterResult<std::string>::success(std::move(result));
}

[[nodiscard]] bool may_be_absent(const pb::FieldDescriptor& field) {
  const std::string_view full_name = field.full_name();
  return full_name == std::string(kPackage) + "MessageHeader.observed_at" ||
         full_name == std::string(kPackage) +
                          "MessageHeader.caused_by_event_id" ||
         full_name == std::string(kPackage) +
                          "ScoutCurrentEstimate.spatial_gradient" ||
         full_name == std::string(kPackage) +
                          "ScoutCoordinationConstraint.calibrated_link_model" ||
         full_name == std::string(kPackage) +
                          "ScoutPlanningResult.candidate" ||
         full_name == std::string(kPackage) +
                          "ScoutPlanValidationReport."
                          "earliest_failure_time_offset_ns" ||
         full_name == std::string(kPackage) +
                          "ScoutPlanValidationReport.minimum_collision_margin_m" ||
         full_name == std::string(kPackage) +
                          "ScoutPlanValidationReport."
                          "minimum_separation_margin_m" ||
         full_name == std::string(kPackage) +
                          "ScoutPlanValidationReport.minimum_energy_margin_j" ||
         full_name == std::string(kPackage) +
                          "ScoutPlanValidationReport.minimum_capability_margin" ||
         full_name == std::string(kPackage) +
                          "ScoutPlanValidationReport.survey_coverage_ratio";
}

[[nodiscard]] bool is_nonempty_identifier(const pb::FieldDescriptor& field) {
  const std::string_view name = field.name();
  return ends_with(name, "_id") || ends_with(name, "_domain_id") ||
         name == "producer_id" || name == "registry_id";
}

struct ValidationContext {
  const VersionFloor* version_floor;
  const InterfaceLimits* interface_limits;
  enum class Mode {
    consumer,
    producer_before_normalization,
    producer_canonical,
  } mode;
  bool allow_unavailable_dependency_facts;
};

[[nodiscard]] bool allows_missing_self_identities(
    const ValidationContext& context) {
  return context.mode != ValidationContext::Mode::consumer;
}

[[nodiscard]] bool allows_noncanonical_values(
    const ValidationContext& context) {
  return context.mode ==
         ValidationContext::Mode::producer_before_normalization;
}

[[nodiscard]] std::size_t bounded_limit(const std::size_t activated,
                                        const std::size_t absolute) {
  return activated == 0 ? absolute : std::min(activated, absolute);
}

[[nodiscard]] std::size_t interface_limit(
    const InterfaceLimits* limits,
    const std::size_t InterfaceLimits::* member,
    const std::size_t absolute) {
  return bounded_limit(limits == nullptr ? 0 : limits->*member, absolute);
}

[[nodiscard]] std::optional<AdapterError> validate_covariance(
    const pb::Message& message, const std::string& path) {
  if (message.GetDescriptor()->full_name() !=
      std::string(kPackage) + "Covariance3d") {
    return std::nullopt;
  }
  const auto* descriptor = message.GetDescriptor();
  const auto* reflection = message.GetReflection();
  const auto* values_field = descriptor->FindFieldByName("row_major");
  if (values_field == nullptr ||
      reflection->FieldSize(message, values_field) != 9) {
    return make_error(AdapterErrorCode::invalid_covariance, path,
                      "Covariance3d must contain exactly nine values");
  }
  std::array<double, 9> values{};
  for (int index = 0; index < 9; ++index) {
    values[static_cast<std::size_t>(index)] =
        reflection->GetRepeatedDouble(message, values_field, index);
  }
  double scale = 1.0;
  for (const double value : values) {
    scale = std::max(scale, std::abs(value));
  }
  const double element_tolerance = 1e-12 * scale;
  const double minor_tolerance = 1e-12 * scale * scale;
  const double determinant_tolerance =
      1e-12 * scale * scale * scale;
  const auto close = [element_tolerance](const double left,
                                         const double right) {
    return std::abs(left - right) <= element_tolerance;
  };
  if (!close(values[1], values[3]) || !close(values[2], values[6]) ||
      !close(values[5], values[7])) {
    return make_error(AdapterErrorCode::invalid_covariance, path,
                      "Covariance3d must be symmetric");
  }
  const double minor_xy = values[0] * values[4] - values[1] * values[3];
  const double minor_xz = values[0] * values[8] - values[2] * values[6];
  const double minor_yz = values[4] * values[8] - values[5] * values[7];
  const double determinant =
      values[0] * (values[4] * values[8] - values[5] * values[7]) -
      values[1] * (values[3] * values[8] - values[5] * values[6]) +
      values[2] * (values[3] * values[7] - values[4] * values[6]);
  if (values[0] < -element_tolerance ||
      values[4] < -element_tolerance ||
      values[8] < -element_tolerance ||
      minor_xy < -minor_tolerance || minor_xz < -minor_tolerance ||
      minor_yz < -minor_tolerance ||
      determinant < -determinant_tolerance) {
    return make_error(AdapterErrorCode::invalid_covariance, path,
                      "Covariance3d must be positive semidefinite");
  }
  return std::nullopt;
}

[[nodiscard]] std::optional<AdapterError> validate_special_structure(
    const pb::Message& message, const ValidationContext& context,
    const std::string& path) {
  const auto* descriptor = message.GetDescriptor();
  const auto* reflection = message.GetReflection();
  const auto* interface_limits = context.interface_limits;
  const std::string& name = descriptor->full_name();
  const auto enforce_repeated_limit = [&](const char* field_name,
                                           const std::size_t limit)
      -> std::optional<AdapterError> {
    const auto* field = descriptor->FindFieldByName(field_name);
    if (field != nullptr &&
        static_cast<std::size_t>(reflection->FieldSize(message, field)) >
            limit) {
      return make_error(AdapterErrorCode::resource_limit_exceeded,
                        path + "." + field_name,
                        "collection exceeds the activated interface limit");
    }
    return std::nullopt;
  };
  const auto validate_ascending_message_key = [&](const char* field_name,
                                                   const char* key_name)
      -> std::optional<AdapterError> {
    const auto* field = descriptor->FindFieldByName(field_name);
    if (field == nullptr) {
      return std::nullopt;
    }
    std::string previous;
    for (int index = 0; index < reflection->FieldSize(message, field); ++index) {
      const pb::Message& item =
          reflection->GetRepeatedMessage(message, field, index);
      const auto* key = item.GetDescriptor()->FindFieldByName(key_name);
      const std::string value = item.GetReflection()->GetString(item, key);
      if (index > 0 && !(previous < value)) {
        return make_error(AdapterErrorCode::invalid_structure,
                          path + "." + field_name,
                          "items must have unique strictly ascending IDs");
      }
      previous = value;
    }
    return std::nullopt;
  };
  const auto* fault_codes = descriptor->FindFieldByName("active_fault_codes");
  if (fault_codes != nullptr) {
    std::uint32_t previous = 0;
    for (int index = 0; index < reflection->FieldSize(message, fault_codes);
         ++index) {
      const auto value =
          reflection->GetRepeatedUInt32(message, fault_codes, index);
      if ((index > 0 && value <= previous) || value == 0) {
        return make_error(AdapterErrorCode::invalid_structure,
                          path + ".active_fault_codes",
                          "fault codes must be known, unique, and ascending");
      }
      previous = value;
    }
  }
  if (name == std::string(kPackage) + "ScoutPlanningDependencies") {
    if (auto error = enforce_repeated_limit(
            "sensors",
            interface_limit(interface_limits,
                            &InterfaceLimits::maximum_sensors_per_context,
                            16U))) {
      return error;
    }
    if (auto error =
            validate_ascending_message_key("sensors", "sensor_id")) {
      return error;
    }
  }
  if (name == std::string(kPackage) + "HybridMapSnapshot") {
    if (auto error = enforce_repeated_limit(
            "semantic_regions",
            interface_limit(interface_limits,
                            &InterfaceLimits::maximum_map_semantic_regions,
                            4'096U))) {
      return error;
    }
    if (auto error =
            validate_ascending_message_key("semantic_regions", "region_id")) {
      return error;
    }
  }
  if (name == std::string(kPackage) + "ScoutCapabilityProfile") {
    if (auto error = enforce_repeated_limit(
            "calibrated_thruster_states",
            interface_limit(interface_limits,
                            &InterfaceLimits::maximum_thrusters_per_vehicle,
                            16U))) {
      return error;
    }
    if (auto error = validate_ascending_message_key(
            "calibrated_thruster_states", "thruster_id")) {
      return error;
    }
  }
  if (name == std::string(kPackage) + "ScoutThrusterHealthState") {
    if (auto error = enforce_repeated_limit(
            "thrusters",
            interface_limit(interface_limits,
                            &InterfaceLimits::maximum_thrusters_per_vehicle,
                            16U))) {
      return error;
    }
    if (auto error =
            validate_ascending_message_key("thrusters", "thruster_id")) {
      return error;
    }
  }
  if (name == std::string(kPackage) + "MainRobotPrediction") {
    if (auto error = enforce_repeated_limit(
            "occupied_intervals",
            interface_limit(interface_limits,
                            &InterfaceLimits::maximum_prediction_intervals,
                            1'024U))) {
      return error;
    }
    const auto* intervals = descriptor->FindFieldByName("occupied_intervals");
    const int count = reflection->FieldSize(message, intervals);
    if (count == 0) {
      return make_error(AdapterErrorCode::invalid_structure, path,
                        "prediction must contain occupied intervals");
    }
    std::uint64_t previous_end = 0;
    const pb::Message* previous_volume = nullptr;
    for (int index = 0; index < count; ++index) {
      const pb::Message& interval =
          reflection->GetRepeatedMessage(message, intervals, index);
      const auto* interval_descriptor = interval.GetDescriptor();
      const auto* interval_reflection = interval.GetReflection();
      const auto start = interval_reflection->GetUInt64(
          interval, interval_descriptor->FindFieldByName("start_offset_ns"));
      const auto end = interval_reflection->GetUInt64(
          interval, interval_descriptor->FindFieldByName("end_offset_ns"));
      if (start != previous_end || end <= start) {
        return make_error(AdapterErrorCode::invalid_time, path,
                          "prediction intervals must start at zero and be contiguous");
      }
      const pb::Message& volume = interval_reflection->GetMessage(
          interval, interval_descriptor->FindFieldByName("swept_volume"));
      const auto* volume_descriptor = volume.GetDescriptor();
      const auto* volume_reflection = volume.GetReflection();
      const auto radius = [&](const char* field_name) {
        return volume_reflection->GetDouble(
            volume, volume_descriptor->FindFieldByName(field_name));
      };
      const double physical = radius("physical_radius_m");
      const double uncertainty = radius("position_uncertainty_radius_m");
      const double conservative = radius("conservative_occupied_radius_m");
      if (physical <= 0.0 || uncertainty < 0.0 || conservative < 0.0 ||
          conservative < physical + uncertainty) {
        return make_error(AdapterErrorCode::invalid_numeric_boundary, path,
                          "conservative occupancy radius is too small");
      }
      if (previous_volume != nullptr) {
        const pb::Message& previous_center =
            previous_volume->GetReflection()->GetMessage(
                *previous_volume,
                previous_volume->GetDescriptor()->FindFieldByName("end_center"));
        const pb::Message& next_center = volume_reflection->GetMessage(
            volume, volume_descriptor->FindFieldByName("start_center"));
        for (const char* coordinate : {"x_m", "y_m", "z_m"}) {
          const auto* previous_field =
              previous_center.GetDescriptor()->FindFieldByName(coordinate);
          const auto* next_field =
              next_center.GetDescriptor()->FindFieldByName(coordinate);
          if (previous_center.GetReflection()->GetDouble(previous_center,
                                                          previous_field) !=
              next_center.GetReflection()->GetDouble(next_center, next_field)) {
            return make_error(AdapterErrorCode::invalid_structure, path,
                              "prediction interval centers are discontinuous");
          }
        }
      }
      previous_end = end;
      previous_volume = &volume;
    }
    const auto valid_from = reflection->GetInt64(
        message,
        descriptor->FindFieldByName("source_valid_from_monotonic_ns"));
    const auto valid_until = reflection->GetInt64(
        message,
        descriptor->FindFieldByName("source_valid_until_monotonic_ns"));
    if (static_cast<std::uint64_t>(valid_until - valid_from) < previous_end) {
      return make_error(AdapterErrorCode::invalid_time, path,
                        "prediction horizon exceeds its validity window");
    }
  }
  if (name == std::string(kPackage) + "ContentIdentity") {
    const auto* field = descriptor->FindFieldByName("sha256");
    if (field == nullptr || reflection->GetString(message, field).size() != 32) {
      return make_error(AdapterErrorCode::invalid_content_identity, path,
                        "ContentIdentity.sha256 must contain exactly 32 bytes");
    }
  }
  if (name == std::string(kPackage) + "ScoutMission") {
    const auto get_double = [&](const char* field_name) {
      return reflection->GetDouble(message,
                                   descriptor->FindFieldByName(field_name));
    };
    const double coverage = get_double("required_coverage_ratio");
    const double resolution = get_double("required_resolution_m");
    const double separation = get_double("minimum_separation_m");
    const double communication =
        get_double("maximum_communication_distance_m");
    if (!(coverage > 0.0 && coverage <= 1.0) || resolution <= 0.0 ||
        separation < 0.0 || communication < separation) {
      return make_error(AdapterErrorCode::invalid_numeric_boundary, path,
                        "mission coverage, resolution, separation, or "
                        "communication boundary is invalid");
    }
    const auto* age = descriptor->FindFieldByName("maximum_evidence_age_ns");
    if (reflection->GetUInt64(message, age) == 0) {
      return make_error(AdapterErrorCode::invalid_time, path,
                        "maximum evidence age must be positive");
    }
  }
  if (name == std::string(kPackage) + "ScoutCoordinationConstraint") {
    const auto* basis = descriptor->FindFieldByName("link_assurance_basis");
    const auto* model = descriptor->FindFieldByName("calibrated_link_model");
    const int basis_value = reflection->GetEnumValue(message, basis);
    const bool has_model = reflection->HasField(message, model);
    if ((basis_value == contract::LINK_ASSURANCE_GEOMETRIC_DISTANCE_ONLY &&
         has_model) ||
        (basis_value == contract::LINK_ASSURANCE_CALIBRATED_LINK_MODEL &&
         !has_model)) {
      return make_error(
          AdapterErrorCode::invalid_structure, path,
          "link-assurance basis and calibrated model presence disagree");
    }
    const auto* channel = descriptor->FindFieldByName("channel_id");
    const double minimum = reflection->GetDouble(
        message, descriptor->FindFieldByName("minimum_separation_m"));
    const double maximum = reflection->GetDouble(
        message,
        descriptor->FindFieldByName("maximum_communication_distance_m"));
    if (reflection->GetEnumValue(message, channel) !=
            contract::CHANNEL_MAIN_SCOUT_COOP ||
        minimum <= 0.0 ||
        maximum <= minimum) {
      return make_error(
          AdapterErrorCode::invalid_numeric_boundary, path,
          "coordination channel or separation/communication bounds are invalid");
    }
  }
  if (name == std::string(kPackage) + "Region3dEnu" ||
      name == std::string(kPackage) + "CurrentRegion3dEnu") {
    const auto* field = descriptor->FindFieldByName("xyz_m");
    if (field == nullptr || reflection->FieldSize(message, field) != 6) {
      return make_error(AdapterErrorCode::invalid_structure, path,
                        "Region3dEnu must contain six bounds");
    }
    for (int axis = 0; axis < 3; ++axis) {
      if (!(reflection->GetRepeatedDouble(message, field, axis) <
            reflection->GetRepeatedDouble(message, field, axis + 3))) {
        return make_error(AdapterErrorCode::invalid_numeric_boundary, path,
                          "Region3dEnu minimum must be below maximum");
      }
    }
  }
  if (name == std::string(kPackage) + "CurrentGradient3d") {
    const auto* field = descriptor->FindFieldByName("row_major_per_s");
    if (field == nullptr || reflection->FieldSize(message, field) != 9) {
      return make_error(AdapterErrorCode::invalid_structure, path,
                        "CurrentGradient3d must contain nine row-major values");
    }
  }
  if (name == std::string(kPackage) + "Pose3dEnuFlu" ||
      name == std::string(kPackage) + "SensorExtrinsicsFlu") {
    const auto get = [&](const char* field_name) {
      return reflection->GetDouble(message,
                                   descriptor->FindFieldByName(field_name));
    };
    const double norm = std::sqrt(get("q_x") * get("q_x") +
                                  get("q_y") * get("q_y") +
                                  get("q_z") * get("q_z") +
                                  get("q_w") * get("q_w"));
    if (std::abs(norm - 1.0) > 1e-6) {
      return make_error(AdapterErrorCode::invalid_numeric_boundary, path,
                        "pose quaternion must have unit norm");
    }
  }
  if (name == std::string(kPackage) + "GridGeometry2d" ||
      name == std::string(kPackage) + "GridGeometry3d") {
    const auto cell_count = [&](const char* axis) {
      return reflection->GetUInt32(
          message, descriptor->FindFieldByName(std::string("cell_count_") + axis));
    };
    const auto resolution = [&](const char* axis) {
      return reflection->GetDouble(
          message, descriptor->FindFieldByName(std::string("resolution_") + axis +
                                               "_m"));
    };
    if (cell_count("x") == 0 || cell_count("y") == 0 ||
        resolution("x") <= 0.0 || resolution("y") <= 0.0 ||
        (descriptor->name() == "GridGeometry3d" &&
         (cell_count("z") == 0 || resolution("z") <= 0.0))) {
      return make_error(AdapterErrorCode::invalid_numeric_boundary, path,
                        "grid dimensions and resolutions must be positive");
    }
  }
  const auto validate_map_layer_size = [&](const char* payload_name,
                                           const bool is_three_dimensional)
      -> std::optional<AdapterError> {
    const auto* grid_field = descriptor->FindFieldByName("grid");
    const auto* payload_field = descriptor->FindFieldByName(payload_name);
    if (grid_field == nullptr || payload_field == nullptr) {
      return std::nullopt;
    }
    const pb::Message& grid = reflection->GetMessage(message, grid_field);
    const auto* grid_descriptor = grid.GetDescriptor();
    const auto* grid_reflection = grid.GetReflection();
    const auto count_x = grid_reflection->GetUInt32(
        grid, grid_descriptor->FindFieldByName("cell_count_x"));
    const auto count_y = grid_reflection->GetUInt32(
        grid, grid_descriptor->FindFieldByName("cell_count_y"));
    std::uint64_t expected = static_cast<std::uint64_t>(count_x) * count_y;
    if (is_three_dimensional) {
      const auto count_z = grid_reflection->GetUInt32(
          grid, grid_descriptor->FindFieldByName("cell_count_z"));
      if (count_z != 0 &&
          expected > std::numeric_limits<std::uint64_t>::max() / count_z) {
        return make_error(AdapterErrorCode::invalid_structure, path,
                          "map grid dimensions overflow the payload size");
      }
      expected *= count_z;
    }
    if (expected > interface_limit(
                       interface_limits,
                       &InterfaceLimits::maximum_map_cells_per_layer,
                       8'000'000U)) {
      return make_error(AdapterErrorCode::resource_limit_exceeded, path,
                        "map layer exceeds the activated cell limit");
    }
    if (expected > static_cast<std::uint64_t>(std::numeric_limits<int>::max()) ||
        reflection->FieldSize(message, payload_field) !=
            static_cast<int>(expected)) {
      return make_error(AdapterErrorCode::invalid_structure, path,
                        "map payload size does not match grid dimensions");
    }
    return std::nullopt;
  };
  if (name == std::string(kPackage) + "SeafloorElevationLayer") {
    if (auto error = validate_map_layer_size("elevation_z_m", false)) {
      return error;
    }
    if (auto error = validate_map_layer_size("quality", false)) {
      return error;
    }
  }
  if (name == std::string(kPackage) + "VoxelOccupancyLayer") {
    if (auto error = validate_map_layer_size("state", true)) {
      return error;
    }
  }
  if (name == std::string(kPackage) + "EsdfLayer") {
    if (auto error = validate_map_layer_size("distance_m", true)) {
      return error;
    }
  }
  if (name == std::string(kPackage) + "AllowedWaterLayer") {
    if (auto error = validate_map_layer_size("allowed", true)) {
      return error;
    }
  }
  if (name == std::string(kPackage) + "ScoutBezierSegment4d") {
    const auto* duration = descriptor->FindFieldByName("duration_ns");
    const auto* position =
        descriptor->FindFieldByName("position_control_points");
    const auto* yaw =
        descriptor->FindFieldByName("yaw_offset_control_points_rad");
    if (reflection->GetUInt64(message, duration) == 0 ||
        reflection->FieldSize(message, position) != 6 ||
        reflection->FieldSize(message, yaw) != 6) {
      return make_error(AdapterErrorCode::invalid_structure, path,
                        "quintic segment requires positive duration and six "
                        "position/yaw control points");
    }
  }
  if (name == std::string(kPackage) + "ScoutTrajectory4d") {
    const auto* segments = descriptor->FindFieldByName("segments");
    const int count = reflection->FieldSize(message, segments);
    if (static_cast<std::size_t>(count) >
        interface_limit(interface_limits,
                        &InterfaceLimits::maximum_scout_plan_segments, 256U)) {
      return make_error(AdapterErrorCode::resource_limit_exceeded, path,
                        "trajectory exceeds the activated segment limit");
    }
    if (count == 0) {
      return make_error(AdapterErrorCode::invalid_structure, path,
                        "trajectory must contain at least one segment");
    }
    std::uint64_t expected_start = 0;
    const auto close = [](const long double left, const long double right) {
      constexpr long double tolerance = 1e-9L;
      return std::abs(left - right) <=
             tolerance * std::max({1.0L, std::abs(left), std::abs(right)});
    };
    for (int index = 0; index < count; ++index) {
      const pb::Message& segment =
          reflection->GetRepeatedMessage(message, segments, index);
      const auto* segment_descriptor = segment.GetDescriptor();
      const auto* segment_reflection = segment.GetReflection();
      const auto* start =
          segment_descriptor->FindFieldByName("start_time_offset_ns");
      const auto* duration = segment_descriptor->FindFieldByName("duration_ns");
      const auto actual_start = segment_reflection->GetUInt64(segment, start);
      const auto actual_duration = segment_reflection->GetUInt64(segment, duration);
      if (actual_start != expected_start || actual_duration == 0 ||
          actual_duration >
              std::numeric_limits<std::uint64_t>::max() - actual_start) {
        return make_error(AdapterErrorCode::invalid_structure, path,
                          "trajectory segments must be positive and contiguous");
      }
      const auto* yaw = segment_descriptor->FindFieldByName(
          "yaw_offset_control_points_rad");
      if (index == 0 &&
          segment_reflection->GetRepeatedDouble(segment, yaw, 0) != 0.0) {
        return make_error(AdapterErrorCode::invalid_structure, path,
                          "the first trajectory yaw offset must be zero");
      }
      if (index > 0) {
        const pb::Message& previous =
            reflection->GetRepeatedMessage(message, segments, index - 1);
        const auto* previous_descriptor = previous.GetDescriptor();
        const auto* previous_reflection = previous.GetReflection();
        const auto* previous_duration_field =
            previous_descriptor->FindFieldByName("duration_ns");
        constexpr long double nanoseconds_per_second = 1'000'000'000.0L;
        const long double previous_duration =
            static_cast<long double>(previous_reflection->GetUInt64(
                previous, previous_duration_field)) /
            nanoseconds_per_second;
        const long double next_duration =
            static_cast<long double>(actual_duration) /
            nanoseconds_per_second;
        const auto* position = segment_descriptor->FindFieldByName(
            "position_control_points");
        const auto* previous_position = previous_descriptor->FindFieldByName(
            "position_control_points");
        for (const char* coordinate : {"x_m", "y_m", "z_m"}) {
          const auto coordinate_at = [&](const pb::Message& owner,
                                         const pb::FieldDescriptor* points,
                                         const int point) {
            const pb::Message& control = owner.GetReflection()->GetRepeatedMessage(
                owner, points, point);
            const auto* field =
                control.GetDescriptor()->FindFieldByName(coordinate);
            return static_cast<long double>(
                control.GetReflection()->GetDouble(control, field));
          };
          const long double p3 = coordinate_at(previous, previous_position, 3);
          const long double p4 = coordinate_at(previous, previous_position, 4);
          const long double p5 = coordinate_at(previous, previous_position, 5);
          const long double q0 = coordinate_at(segment, position, 0);
          const long double q1 = coordinate_at(segment, position, 1);
          const long double q2 = coordinate_at(segment, position, 2);
          if (!close(p5, q0) ||
              !close((p5 - p4) / previous_duration,
                     (q1 - q0) / next_duration) ||
              !close((p5 - 2.0L * p4 + p3) /
                         (previous_duration * previous_duration),
                     (q2 - 2.0L * q1 + q0) /
                         (next_duration * next_duration))) {
            return make_error(AdapterErrorCode::invalid_structure, path,
                              "trajectory position must be C2 continuous");
          }
        }
        const auto* previous_yaw = previous_descriptor->FindFieldByName(
            "yaw_offset_control_points_rad");
        const auto yaw_at = [](const pb::Message& owner,
                               const pb::FieldDescriptor* field,
                               const int point) {
          return static_cast<long double>(
              owner.GetReflection()->GetRepeatedDouble(owner, field, point));
        };
        const long double p3 = yaw_at(previous, previous_yaw, 3);
        const long double p4 = yaw_at(previous, previous_yaw, 4);
        const long double p5 = yaw_at(previous, previous_yaw, 5);
        const long double q0 = yaw_at(segment, yaw, 0);
        const long double q1 = yaw_at(segment, yaw, 1);
        const long double q2 = yaw_at(segment, yaw, 2);
        if (!close(p5, q0) ||
            !close((p5 - p4) / previous_duration,
                   (q1 - q0) / next_duration) ||
            !close((p5 - 2.0L * p4 + p3) /
                       (previous_duration * previous_duration),
                   (q2 - 2.0L * q1 + q0) /
                       (next_duration * next_duration))) {
          return make_error(AdapterErrorCode::invalid_structure, path,
                            "trajectory yaw must be C2 continuous");
        }
      }
      expected_start = actual_start + actual_duration;
    }
  }
  if (name == std::string(kPackage) + "ScoutPlanValidationReport") {
    const auto* status = descriptor->FindFieldByName("status");
    if (reflection->GetEnumValue(message, status) ==
        contract::SCOUT_PLAN_VALIDATION_SAFE) {
      const auto* primary = descriptor->FindFieldByName("primary_outcome");
      const auto* failure =
          descriptor->FindFieldByName("earliest_failure_time_offset_ns");
      if (reflection->GetEnumValue(message, primary) !=
              contract::OUTCOME_SUCCESS ||
          reflection->HasField(message, failure)) {
        return make_error(AdapterErrorCode::invalid_structure, path,
                          "SAFE validation requires success and no failure offset");
      }
      for (const char* margin_name :
           {"minimum_collision_margin_m", "minimum_separation_margin_m",
            "minimum_energy_margin_j", "minimum_capability_margin"}) {
        const auto* margin = descriptor->FindFieldByName(margin_name);
        if (!reflection->HasField(message, margin) ||
            reflection->GetDouble(message, margin) < 0.0) {
          return make_error(
              AdapterErrorCode::invalid_numeric_boundary,
              path + "." + margin_name,
              "SAFE report margins must be present and nonnegative");
        }
      }
      const auto* coverage =
          descriptor->FindFieldByName("survey_coverage_ratio");
      const double coverage_value = reflection->GetDouble(message, coverage);
      if (!reflection->HasField(message, coverage) || coverage_value < 0.0 ||
          coverage_value > 1.0) {
        return make_error(AdapterErrorCode::invalid_numeric_boundary,
                          path + ".survey_coverage_ratio",
                          "SAFE survey coverage must be present in [0, 1]");
      }
    }
  }
  if (name == std::string(kPackage) + "ScoutPlan") {
    const auto child = [&](const char* field_name) -> const pb::Message& {
      const auto* field = descriptor->FindFieldByName(field_name);
      return reflection->GetMessage(message, field);
    };
    const pb::Message& trajectory = child("trajectory");
    const pb::Message& dependencies = child("dependencies");
    const pb::Message& evidence = child("survey_evidence");
    const pb::Message& report = child("validation_report");
    const auto* report_descriptor = report.GetDescriptor();
    const auto* report_reflection = report.GetReflection();
    const auto enum_value = [&](const char* field_name) {
      return report_reflection->GetEnumValue(
          report, report_descriptor->FindFieldByName(field_name));
    };
    if (enum_value("status") != contract::SCOUT_PLAN_VALIDATION_SAFE ||
        enum_value("primary_outcome") != contract::OUTCOME_SUCCESS) {
      return make_error(AdapterErrorCode::invalid_structure, path,
                        "a candidate plan requires an independent SAFE report");
    }
    const auto require_bound_identity = [&](const char* report_field_name,
                                             const pb::Message& source,
                                             const char* source_field_name)
        -> std::optional<AdapterError> {
      const auto* report_field =
          report_descriptor->FindFieldByName(report_field_name);
      if (allows_missing_self_identities(context) &&
          !report_reflection->HasField(report, report_field)) {
        return std::nullopt;
      }
      const pb::Message& validated = report_reflection->GetMessage(
          report, report_field);
      const pb::Message& actual = source.GetReflection()->GetMessage(
          source, source.GetDescriptor()->FindFieldByName(source_field_name));
      if (!pb::util::MessageDifferencer::Equals(validated, actual)) {
        return make_error(AdapterErrorCode::invalid_structure,
                          path + ".validation_report." + report_field_name,
                          "validation report identity does not bind its input");
      }
      return std::nullopt;
    };
    if (auto error = require_bound_identity(
            "validated_dependencies_content_identity", dependencies,
            "dependencies_content_identity")) {
      return error;
    }
    if (auto error = require_bound_identity(
            "validated_trajectory_content_identity", trajectory,
            "trajectory_content_identity")) {
      return error;
    }
    if (auto error = require_bound_identity(
            "validated_survey_evidence_content_identity", evidence,
            "evidence_content_identity")) {
      return error;
    }
    const auto require_equal_scalar = [&](const pb::Message& left,
                                           const char* left_name,
                                           const pb::Message& right,
                                           const char* right_name) {
      const auto* left_field = left.GetDescriptor()->FindFieldByName(left_name);
      const auto* right_field =
          right.GetDescriptor()->FindFieldByName(right_name);
      if (left_field->cpp_type() == pb::FieldDescriptor::CPPTYPE_STRING) {
        return left.GetReflection()->GetString(left, left_field) ==
               right.GetReflection()->GetString(right, right_field);
      }
      return left.GetReflection()->GetUInt64(left, left_field) ==
             right.GetReflection()->GetUInt64(right, right_field);
    };
    const auto equal_identity = [](const pb::Message& left,
                                    const char* left_name,
                                    const pb::Message& right,
                                    const char* right_name) {
      return pb::util::MessageDifferencer::Equals(
          left.GetReflection()->GetMessage(
              left, left.GetDescriptor()->FindFieldByName(left_name)),
          right.GetReflection()->GetMessage(
              right, right.GetDescriptor()->FindFieldByName(right_name)));
    };
    if (!require_equal_scalar(evidence, "mission_id", dependencies,
                              "mission_id") ||
        !require_equal_scalar(evidence, "mission_version", dependencies,
                              "mission_version") ||
        !require_equal_scalar(evidence, "baseline_map_id", dependencies,
                              "map_id") ||
        !require_equal_scalar(evidence, "baseline_map_version", dependencies,
                              "map_version") ||
        !equal_identity(evidence, "mission_content_identity", dependencies,
                        "mission_content_identity") ||
        !equal_identity(evidence, "baseline_map_content_identity",
                        dependencies, "map_content_identity")) {
      return make_error(AdapterErrorCode::invalid_structure, path,
                        "survey evidence does not bind plan dependencies");
    }
  }
  if (name == std::string(kPackage) + "ScoutPlanningResult") {
    const auto* outcome = descriptor->FindFieldByName("outcome");
    const auto* candidate = descriptor->FindFieldByName("candidate");
    const int outcome_value = reflection->GetEnumValue(message, outcome);
    const bool allowed_outcome = [outcome_value] {
      switch (static_cast<contract::OutcomeCode>(outcome_value)) {
        case contract::OUTCOME_SUCCESS:
        case contract::OUTCOME_INPUT_INVALID:
        case contract::OUTCOME_DEPENDENCY_STALE:
        case contract::OUTCOME_TIMEOUT:
        case contract::OUTCOME_NO_SOLUTION:
        case contract::OUTCOME_CANCELLED:
        case contract::OUTCOME_SMOOTHING_FAILED:
        case contract::OUTCOME_CAPABILITY_INFEASIBLE:
        case contract::OUTCOME_ENERGY_INSUFFICIENT:
        case contract::OUTCOME_COORDINATION_INFEASIBLE:
        case contract::OUTCOME_SURVEY_INFEASIBLE:
        case contract::OUTCOME_VALIDATION_REJECTED:
        case contract::OUTCOME_VALIDATION_INCONCLUSIVE:
        case contract::OUTCOME_NUMERICALLY_INVALID:
          return true;
        default:
          return false;
      }
    }();
    if (!allowed_outcome) {
      return make_error(AdapterErrorCode::invalid_structure, path,
                        "outcome is not a Scout planner terminal outcome");
    }
    const bool success = outcome_value == contract::OUTCOME_SUCCESS;
    const bool has_candidate = reflection->HasField(message, candidate);
    if (success != has_candidate) {
      return make_error(AdapterErrorCode::invalid_structure, path,
                        "only successful planning results carry a candidate");
    }
    const pb::Message& header = reflection->GetMessage(
        message, descriptor->FindFieldByName("header"));
    const auto generated = header.GetReflection()->GetInt64(
        header,
        header.GetDescriptor()->FindFieldByName("generated_at_monotonic_ns"));
    const auto evaluated = reflection->GetInt64(
        message,
        descriptor->FindFieldByName("evaluated_at_monotonic_ns"));
    if (evaluated > generated) {
      return make_error(AdapterErrorCode::invalid_time, path,
                        "planning evaluation cannot follow message generation");
    }
    if (has_candidate) {
      const pb::Message& plan = reflection->GetMessage(message, candidate);
      const auto created = plan.GetReflection()->GetInt64(
          plan, plan.GetDescriptor()->FindFieldByName(
                    "created_at_monotonic_ns"));
      if (created > evaluated) {
        return make_error(AdapterErrorCode::invalid_time, path,
                          "candidate creation cannot follow result evaluation");
      }
    }
    const auto* diagnostics = descriptor->FindFieldByName("diagnostics");
    if (!success && reflection->FieldSize(message, diagnostics) == 0) {
      return make_error(AdapterErrorCode::invalid_structure, path,
                        "failed planning results require a diagnostic");
    }
    if (has_candidate) {
      const pb::Message& plan = reflection->GetMessage(message, candidate);
      const auto* plan_dependencies =
          plan.GetDescriptor()->FindFieldByName("dependencies");
      const auto* result_dependencies =
          descriptor->FindFieldByName("dependencies");
      if (!pb::util::MessageDifferencer::Equals(
              plan.GetReflection()->GetMessage(plan, plan_dependencies),
              reflection->GetMessage(message, result_dependencies))) {
        return make_error(
            AdapterErrorCode::invalid_structure, path,
            "candidate dependencies must exactly match result dependencies");
      }
    }
  }

  const auto validate_time_pair = [&](const char* start_name,
                                      const char* end_name,
                                      const bool strict)
      -> std::optional<AdapterError> {
    const auto* start = descriptor->FindFieldByName(start_name);
    const auto* end = descriptor->FindFieldByName(end_name);
    if (start == nullptr || end == nullptr ||
        (start->has_presence() && !reflection->HasField(message, start)) ||
        (end->has_presence() && !reflection->HasField(message, end))) {
      return std::nullopt;
    }
    const auto start_value = reflection->GetInt64(message, start);
    const auto end_value = reflection->GetInt64(message, end);
    if (strict ? start_value >= end_value : start_value > end_value) {
      return make_error(AdapterErrorCode::invalid_time, path,
                        "monotonic validity interval is reversed or empty");
    }
    return std::nullopt;
  };
  for (const auto& pair :
       {std::array<const char*, 2>{"observed_at_monotonic_ns",
                                   "valid_until_monotonic_ns"},
        std::array<const char*, 2>{"valid_from_monotonic_ns",
                                   "valid_until_monotonic_ns"},
        std::array<const char*, 2>{"source_valid_from_monotonic_ns",
                                   "source_valid_until_monotonic_ns"},
        std::array<const char*, 2>{"received_at_monotonic_ns",
                                   "admission_valid_until_monotonic_ns"},
        std::array<const char*, 2>{"validated_at_monotonic_ns",
                                   "expires_at_monotonic_ns"}}) {
    if (auto error = validate_time_pair(pair[0], pair[1], true)) {
      return error;
    }
  }
  return validate_covariance(message, path);
}

[[nodiscard]] std::optional<AdapterError> validate_message(
    const pb::Message& message, const ValidationContext& context,
    const std::string& path) {
  const auto* descriptor = message.GetDescriptor();
  const auto* reflection = message.GetReflection();
  const auto& unknown = reflection->GetUnknownFields(message);
  if (unknown.field_count() != 0) {
    return make_error(AdapterErrorCode::unknown_field, path,
                      "unknown protobuf fields are rejected");
  }

  for (int index = 0; index < descriptor->field_count(); ++index) {
    const auto* field = descriptor->field(index);
    const std::string field_path = path + "." + field->name();
    const int count = field->is_repeated()
                          ? reflection->FieldSize(message, field)
                          : (field->has_presence() &&
                                     !reflection->HasField(message, field)
                                 ? 0
                                 : 1);
    if (field->is_repeated() &&
        static_cast<std::size_t>(count) >
            interface_limit(context.interface_limits,
                            &InterfaceLimits::maximum_repeated_items,
                            20'000U)) {
      return make_error(AdapterErrorCode::resource_limit_exceeded, field_path,
                        "repeated field exceeds the activated interface limit");
    }
    if (field->is_repeated() && field->name() == "diagnostics" &&
        static_cast<std::size_t>(count) >
            interface_limit(context.interface_limits,
                            &InterfaceLimits::maximum_diagnostics, 128U)) {
      return make_error(AdapterErrorCode::resource_limit_exceeded, field_path,
                        "diagnostics exceed the activated interface limit");
    }
    const auto self_identity = identity_metadata(*descriptor);
    const bool unavailable_dependency_field =
        context.allow_unavailable_dependency_facts &&
        descriptor == contract::ScoutPlanningDependencies::descriptor() &&
        field->name() != "dependencies_content_identity";
    const bool missing_derived_identity =
        allows_missing_self_identities(context) && self_identity.has_value() &&
        field->name() == self_identity->identity_field_name;
    const bool missing_validation_binding =
        allows_missing_self_identities(context) &&
        descriptor == contract::ScoutPlanValidationReport::descriptor() &&
        (field->name() == "validated_dependencies_content_identity" ||
         field->name() == "validated_trajectory_content_identity" ||
         field->name() == "validated_survey_evidence_content_identity");
    if (!field->is_repeated() && field->has_presence() && count == 0 &&
        !may_be_absent(*field) && !missing_derived_identity &&
        !missing_validation_binding && !unavailable_dependency_field) {
      return make_error(AdapterErrorCode::missing_safety_field, field_path,
                        "required safety field is absent");
    }

    for (int item = 0; item < count; ++item) {
      const auto repeated = field->is_repeated();
      switch (field->cpp_type()) {
        case pb::FieldDescriptor::CPPTYPE_DOUBLE: {
          const double value = repeated
                                   ? reflection->GetRepeatedDouble(message,
                                                                   field, item)
                                   : reflection->GetDouble(message, field);
          if (!std::isfinite(value)) {
            return make_error(AdapterErrorCode::non_finite_number, field_path,
                              "floating-point contract values must be finite");
          }
          if ((field->name() == "yaw_rad" ||
               field->name() == "initial_yaw_rad") &&
              !allows_noncanonical_values(context) &&
              !(value >= -std::acos(-1.0) && value < std::acos(-1.0))) {
            return make_error(AdapterErrorCode::invalid_numeric_boundary,
                              field_path,
                              "instantaneous ENU yaw must be in [-pi, pi)");
          }
          break;
        }
        case pb::FieldDescriptor::CPPTYPE_FLOAT: {
          const float value = repeated
                                  ? reflection->GetRepeatedFloat(message, field,
                                                                 item)
                                  : reflection->GetFloat(message, field);
          if (!std::isfinite(value)) {
            return make_error(AdapterErrorCode::non_finite_number, field_path,
                              "floating-point contract values must be finite");
          }
          break;
        }
        case pb::FieldDescriptor::CPPTYPE_ENUM: {
          const int value = repeated
                                ? reflection->GetRepeatedEnumValue(message,
                                                                   field, item)
                                : reflection->GetEnumValue(message, field);
          if (field->enum_type()->FindValueByNumber(value) == nullptr ||
              value == 0) {
            return make_error(AdapterErrorCode::unknown_enum, field_path,
                              "unknown or unspecified safety enum is rejected");
          }
          break;
        }
        case pb::FieldDescriptor::CPPTYPE_INT64: {
          const std::int64_t value =
              repeated ? reflection->GetRepeatedInt64(message, field, item)
                       : reflection->GetInt64(message, field);
          if ((ends_with(field->name(), "_monotonic_ns") ||
               ends_with(field->name(), "_utc_ns")) &&
              value < 0) {
            return make_error(AdapterErrorCode::invalid_time, field_path,
                              "time values cannot be negative");
          }
          break;
        }
        case pb::FieldDescriptor::CPPTYPE_UINT64: {
          const std::uint64_t value =
              repeated ? reflection->GetRepeatedUInt64(message, field, item)
                       : reflection->GetUInt64(message, field);
          const bool is_version = field->name() == "version" ||
                                  ends_with(field->name(), "_version");
          if (is_version && value == 0 && !unavailable_dependency_field) {
            return make_error(AdapterErrorCode::invalid_version, field_path,
                              "business and configuration versions start at one");
          }
          if (is_version && context.version_floor != nullptr) {
            const auto floor = context.version_floor->find(field->full_name());
            if (floor != context.version_floor->end() && value < floor->second) {
              return make_error(AdapterErrorCode::version_rollback, field_path,
                                "version is below the accepted watermark");
            }
          }
          break;
        }
        case pb::FieldDescriptor::CPPTYPE_STRING: {
          const std::string value =
              repeated ? reflection->GetRepeatedString(message, field, item)
                       : reflection->GetString(message, field);
          if (field->type() == pb::FieldDescriptor::TYPE_STRING &&
              is_nonempty_identifier(*field) && value.empty() &&
              !unavailable_dependency_field) {
            return make_error(AdapterErrorCode::invalid_structure, field_path,
                              "contract identifiers cannot be empty");
          }
          if (field->type() == pb::FieldDescriptor::TYPE_STRING) {
            if (value.size() >
                interface_limit(context.interface_limits,
                                &InterfaceLimits::maximum_string_bytes, 512U)) {
              return make_error(
                  AdapterErrorCode::resource_limit_exceeded, field_path,
                  "string exceeds the activated interface limit");
            }
            auto normalized = normalize_nfc(value);
            if (!normalized.has_value()) {
              auto error = normalized.error();
              error.path = field_path;
              return error;
            }
            if (!allows_noncanonical_values(context) &&
                normalized.value() != value) {
              return make_error(AdapterErrorCode::invalid_text, field_path,
                                "contract text must already be Unicode NFC");
            }
          }
          if (field->type() == pb::FieldDescriptor::TYPE_STRING &&
              (field->name() == "frame_id" ||
               field->name() == "world_frame_id" ||
               field->name() == "input_frame_id" ||
               field->name() == "output_frame_id")) {
            const bool body_twist = descriptor->name() == "BodyTwist3dFlu";
            const bool body_covariance =
                descriptor->name() == "Covariance3d" &&
                path.find(".attitude_covariance_rad2") != std::string::npos;
            const std::string_view expected =
                body_twist || body_covariance ? "base_link" : "mission_enu";
            if (value != expected) {
              return make_error(AdapterErrorCode::invalid_frame, field_path,
                                "frame does not match the canonical v1 frame");
            }
          }
          if (field->type() == pb::FieldDescriptor::TYPE_STRING &&
              field->name() == "body_frame_id" && value != "base_link") {
            return make_error(AdapterErrorCode::invalid_frame, field_path,
                              "body frame must be base_link/FLU");
          }
          break;
        }
        case pb::FieldDescriptor::CPPTYPE_MESSAGE: {
          const pb::Message& child =
              repeated ? reflection->GetRepeatedMessage(message, field, item)
                       : reflection->GetMessage(message, field);
          auto child_context = context;
          if (descriptor == contract::ScoutPlanningResult::descriptor() &&
              field->name() == "dependencies") {
            const auto* outcome = descriptor->FindFieldByName("outcome");
            child_context.allow_unavailable_dependency_facts =
                reflection->GetEnumValue(message, outcome) !=
                contract::OUTCOME_SUCCESS;
          }
          if (auto error = validate_message(child, child_context, field_path)) {
            return error;
          }
          break;
        }
        case pb::FieldDescriptor::CPPTYPE_INT32:
        case pb::FieldDescriptor::CPPTYPE_UINT32:
        case pb::FieldDescriptor::CPPTYPE_BOOL:
          break;
      }
    }
  }
  return validate_special_structure(message, context, path);
}

[[nodiscard]] CoreMessage message_to_core(const pb::Message& message);

[[nodiscard]] CoreAtom atom_from_message(const pb::Message& message,
                                         const pb::FieldDescriptor& field,
                                         const int index) {
  const auto* reflection = message.GetReflection();
  const bool repeated = field.is_repeated();
  switch (field.cpp_type()) {
    case pb::FieldDescriptor::CPPTYPE_INT32:
      return repeated ? reflection->GetRepeatedInt32(message, &field, index)
                      : reflection->GetInt32(message, &field);
    case pb::FieldDescriptor::CPPTYPE_INT64:
      return repeated ? reflection->GetRepeatedInt64(message, &field, index)
                      : reflection->GetInt64(message, &field);
    case pb::FieldDescriptor::CPPTYPE_UINT32:
      return repeated ? reflection->GetRepeatedUInt32(message, &field, index)
                      : reflection->GetUInt32(message, &field);
    case pb::FieldDescriptor::CPPTYPE_UINT64:
      return repeated ? reflection->GetRepeatedUInt64(message, &field, index)
                      : reflection->GetUInt64(message, &field);
    case pb::FieldDescriptor::CPPTYPE_DOUBLE:
      return repeated ? reflection->GetRepeatedDouble(message, &field, index)
                      : reflection->GetDouble(message, &field);
    case pb::FieldDescriptor::CPPTYPE_FLOAT:
      return repeated ? reflection->GetRepeatedFloat(message, &field, index)
                      : reflection->GetFloat(message, &field);
    case pb::FieldDescriptor::CPPTYPE_BOOL:
      return repeated ? reflection->GetRepeatedBool(message, &field, index)
                      : reflection->GetBool(message, &field);
    case pb::FieldDescriptor::CPPTYPE_ENUM: {
      const int number = repeated
                             ? reflection->GetRepeatedEnumValue(message, &field,
                                                                index)
                             : reflection->GetEnumValue(message, &field);
      return EnumValue{field.enum_type()->full_name(), number};
    }
    case pb::FieldDescriptor::CPPTYPE_STRING: {
      const std::string value =
          repeated ? reflection->GetRepeatedString(message, &field, index)
                   : reflection->GetString(message, &field);
      if (field.type() == pb::FieldDescriptor::TYPE_BYTES) {
        return BytesValue{std::vector<std::uint8_t>(value.begin(), value.end())};
      }
      return TextValue{value};
    }
    case pb::FieldDescriptor::CPPTYPE_MESSAGE: {
      const pb::Message& child =
          repeated ? reflection->GetRepeatedMessage(message, &field, index)
                   : reflection->GetMessage(message, &field);
      return std::make_shared<const CoreMessage>(message_to_core(child));
    }
  }
  return std::int32_t{};
}

[[nodiscard]] CoreMessage message_to_core(const pb::Message& message) {
  std::vector<const pb::FieldDescriptor*> populated;
  message.GetReflection()->ListFields(message, &populated);
  std::sort(populated.begin(), populated.end(),
            [](const auto* left, const auto* right) {
              return left->number() < right->number();
            });
  CoreMessage result{message.GetDescriptor()->full_name(), {}};
  for (const auto* field : populated) {
    CoreField core_field{static_cast<std::uint32_t>(field->number()),
                         field->name(),
                         field->is_repeated()
                             ? FieldCardinality::repeated
                             : FieldCardinality::singular,
                         {}};
    const int count = field->is_repeated()
                          ? message.GetReflection()->FieldSize(message, field)
                          : 1;
    for (int index = 0; index < count; ++index) {
      core_field.values.push_back(atom_from_message(message, *field, index));
    }
    result.fields.push_back(std::move(core_field));
  }
  return result;
}

[[nodiscard]] std::optional<AdapterError> set_atom(
    pb::Message* message, const pb::FieldDescriptor& field,
    const CoreAtom& atom, const int index, const std::string& path);

[[nodiscard]] std::optional<AdapterError> core_to_message(
    const CoreMessage& core, pb::Message* message, const std::string& path) {
  if (core.schema_name != message->GetDescriptor()->full_name()) {
    return make_error(AdapterErrorCode::schema_mismatch, path,
                      "core message schema does not match adapter kind");
  }
  std::vector<bool> seen(
      static_cast<std::size_t>(message->GetDescriptor()->field_count()), false);
  for (const CoreField& core_field : core.fields) {
    const auto* field =
        message->GetDescriptor()->FindFieldByNumber(core_field.number);
    const std::string field_path = path + "." + core_field.name;
    if (field == nullptr || field->name() != core_field.name) {
      return make_error(AdapterErrorCode::schema_mismatch, field_path,
                        "core field is not present in the golden schema");
    }
    const auto descriptor_index = static_cast<std::size_t>(field->index());
    if (seen[descriptor_index]) {
      return make_error(AdapterErrorCode::invalid_structure, field_path,
                        "core field occurs more than once");
    }
    seen[descriptor_index] = true;
    const bool core_is_repeated =
        core_field.cardinality == FieldCardinality::repeated;
    if (core_is_repeated != field->is_repeated() ||
        (!field->is_repeated() && core_field.values.size() != 1)) {
      return make_error(AdapterErrorCode::invalid_structure, field_path,
                        "core field cardinality does not match schema");
    }
    for (std::size_t index = 0; index < core_field.values.size(); ++index) {
      if (auto error = set_atom(message, *field, core_field.values[index],
                                static_cast<int>(index), field_path)) {
        return error;
      }
    }
  }
  return std::nullopt;
}

template <typename T>
[[nodiscard]] const T* atom_as(const CoreAtom& atom) {
  return std::get_if<T>(&atom);
}

[[nodiscard]] std::optional<AdapterError> set_atom(
    pb::Message* message, const pb::FieldDescriptor& field,
    const CoreAtom& atom, const int index, const std::string& path) {
  auto* reflection = message->GetReflection();
  const bool repeated = field.is_repeated();
#define SCOUT_SET_SCALAR(Type, AddMethod, SetMethod)                            \
  do {                                                                         \
    const auto* value = atom_as<Type>(atom);                                   \
    if (value == nullptr) {                                                     \
      return make_error(AdapterErrorCode::type_mismatch, path,                 \
                        "core scalar type does not match schema");            \
    }                                                                          \
    if (repeated) {                                                            \
      reflection->AddMethod(message, &field, *value);                          \
    } else {                                                                   \
      reflection->SetMethod(message, &field, *value);                          \
    }                                                                          \
  } while (false)

  switch (field.cpp_type()) {
    case pb::FieldDescriptor::CPPTYPE_INT32:
      SCOUT_SET_SCALAR(std::int32_t, AddInt32, SetInt32);
      break;
    case pb::FieldDescriptor::CPPTYPE_INT64:
      SCOUT_SET_SCALAR(std::int64_t, AddInt64, SetInt64);
      break;
    case pb::FieldDescriptor::CPPTYPE_UINT32:
      SCOUT_SET_SCALAR(std::uint32_t, AddUInt32, SetUInt32);
      break;
    case pb::FieldDescriptor::CPPTYPE_UINT64:
      SCOUT_SET_SCALAR(std::uint64_t, AddUInt64, SetUInt64);
      break;
    case pb::FieldDescriptor::CPPTYPE_FLOAT:
      SCOUT_SET_SCALAR(float, AddFloat, SetFloat);
      break;
    case pb::FieldDescriptor::CPPTYPE_DOUBLE:
      SCOUT_SET_SCALAR(double, AddDouble, SetDouble);
      break;
    case pb::FieldDescriptor::CPPTYPE_BOOL:
      SCOUT_SET_SCALAR(bool, AddBool, SetBool);
      break;
    case pb::FieldDescriptor::CPPTYPE_ENUM: {
      const auto* value = atom_as<EnumValue>(atom);
      if (value == nullptr || value->type_name != field.enum_type()->full_name()) {
        return make_error(AdapterErrorCode::type_mismatch, path,
                          "core enum type does not match schema");
      }
      if (repeated) {
        reflection->AddEnumValue(message, &field, value->number);
      } else {
        reflection->SetEnumValue(message, &field, value->number);
      }
      break;
    }
    case pb::FieldDescriptor::CPPTYPE_STRING: {
      std::string value;
      if (field.type() == pb::FieldDescriptor::TYPE_BYTES) {
        const auto* bytes = atom_as<BytesValue>(atom);
        if (bytes == nullptr) {
          return make_error(AdapterErrorCode::type_mismatch, path,
                            "core bytes type does not match schema");
        }
        value.assign(bytes->value.begin(), bytes->value.end());
      } else {
        const auto* text = atom_as<TextValue>(atom);
        if (text == nullptr) {
          return make_error(AdapterErrorCode::type_mismatch, path,
                            "core text type does not match schema");
        }
        value = text->value;
      }
      if (repeated) {
        reflection->AddString(message, &field, value);
      } else {
        reflection->SetString(message, &field, value);
      }
      break;
    }
    case pb::FieldDescriptor::CPPTYPE_MESSAGE: {
      const auto* value = atom_as<CoreMessagePtr>(atom);
      if (value == nullptr || !*value) {
        return make_error(AdapterErrorCode::type_mismatch, path,
                          "core message type does not match schema");
      }
      pb::Message* child = repeated
                               ? reflection->AddMessage(message, &field)
                               : reflection->MutableMessage(message, &field);
      if (auto error = core_to_message(**value, child,
                                       path + "[" + std::to_string(index) + "]")) {
        return error;
      }
      break;
    }
  }
#undef SCOUT_SET_SCALAR
  return std::nullopt;
}

[[nodiscard]] AdapterResult<std::string> deterministic_serialize(
    const pb::Message& message) {
  std::string output;
  output.reserve(message.ByteSizeLong());
  pb::io::StringOutputStream raw_output(&output);
  pb::io::CodedOutputStream coded_output(&raw_output);
  coded_output.SetSerializationDeterministic(true);
  if (!message.SerializeToCodedStream(&coded_output) || coded_output.HadError()) {
    return AdapterResult<std::string>::failure(make_error(
        AdapterErrorCode::serialization_failed, message.GetTypeName(),
        "protobuf deterministic serialization failed"));
  }
  coded_output.Trim();
  return AdapterResult<std::string>::success(std::move(output));
}

constexpr std::array<std::uint32_t, 64> kSha256RoundConstants{
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU,
    0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U, 0xd807aa98U, 0x12835b01U,
    0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U,
    0xc19bf174U, 0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
    0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU, 0x983e5152U,
    0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U,
    0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU,
    0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U,
    0xd6990624U, 0xf40e3585U, 0x106aa070U, 0x19a4c116U, 0x1e376c08U,
    0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU,
    0x682e6ff3U, 0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

[[nodiscard]] constexpr std::uint32_t rotate_right(const std::uint32_t value,
                                                   const unsigned shift) {
  return (value >> shift) | (value << (32U - shift));
}

[[nodiscard]] Hash256 sha256(const std::string_view input) {
  std::vector<std::uint8_t> bytes(input.begin(), input.end());
  bytes.push_back(0x80U);
  while ((bytes.size() % 64U) != 56U) {
    bytes.push_back(0U);
  }
  const auto bit_length = static_cast<std::uint64_t>(input.size()) * 8U;
  for (int shift = 56; shift >= 0; shift -= 8) {
    bytes.push_back(static_cast<std::uint8_t>(bit_length >> shift));
  }

  std::array<std::uint32_t, 8> hash{
      0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
      0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U};
  for (std::size_t offset = 0; offset < bytes.size(); offset += 64U) {
    std::array<std::uint32_t, 64> words{};
    for (std::size_t index = 0; index < 16U; ++index) {
      const std::size_t byte = offset + index * 4U;
      words[index] = (static_cast<std::uint32_t>(bytes[byte]) << 24U) |
                     (static_cast<std::uint32_t>(bytes[byte + 1U]) << 16U) |
                     (static_cast<std::uint32_t>(bytes[byte + 2U]) << 8U) |
                     static_cast<std::uint32_t>(bytes[byte + 3U]);
    }
    for (std::size_t index = 16U; index < 64U; ++index) {
      const auto s0 = rotate_right(words[index - 15U], 7U) ^
                      rotate_right(words[index - 15U], 18U) ^
                      (words[index - 15U] >> 3U);
      const auto s1 = rotate_right(words[index - 2U], 17U) ^
                      rotate_right(words[index - 2U], 19U) ^
                      (words[index - 2U] >> 10U);
      words[index] = words[index - 16U] + s0 + words[index - 7U] + s1;
    }
    std::array<std::uint32_t, 8> state = hash;
    for (std::size_t index = 0; index < 64U; ++index) {
      const auto sum1 = rotate_right(state[4], 6U) ^
                        rotate_right(state[4], 11U) ^
                        rotate_right(state[4], 25U);
      const auto choose = (state[4] & state[5]) ^ (~state[4] & state[6]);
      const auto temporary1 =
          state[7] + sum1 + choose + kSha256RoundConstants[index] + words[index];
      const auto sum0 = rotate_right(state[0], 2U) ^
                        rotate_right(state[0], 13U) ^
                        rotate_right(state[0], 22U);
      const auto majority = (state[0] & state[1]) ^ (state[0] & state[2]) ^
                            (state[1] & state[2]);
      const auto temporary2 = sum0 + majority;
      state = {temporary1 + temporary2, state[0], state[1], state[2],
               state[3] + temporary1, state[4], state[5], state[6]};
    }
    for (std::size_t index = 0; index < hash.size(); ++index) {
      hash[index] += state[index];
    }
  }
  Hash256 result{};
  for (std::size_t index = 0; index < hash.size(); ++index) {
    result[index * 4U] = static_cast<std::uint8_t>(hash[index] >> 24U);
    result[index * 4U + 1U] = static_cast<std::uint8_t>(hash[index] >> 16U);
    result[index * 4U + 2U] = static_cast<std::uint8_t>(hash[index] >> 8U);
    result[index * 4U + 3U] = static_cast<std::uint8_t>(hash[index]);
  }
  return result;
}

[[nodiscard]] std::optional<AdapterError> normalize_canonical_message(
    pb::Message* message, const std::string& path) {
  const auto* descriptor = message->GetDescriptor();
  auto* reflection = message->GetReflection();
  for (int field_index = 0; field_index < descriptor->field_count();
       ++field_index) {
    const auto* field = descriptor->field(field_index);
    const int count = field->is_repeated()
                          ? reflection->FieldSize(*message, field)
                          : (field->has_presence() &&
                                     !reflection->HasField(*message, field)
                                 ? 0
                                 : 1);
    for (int index = 0; index < count; ++index) {
      const std::string field_path = path + "." + field->name();
      const bool repeated = field->is_repeated();
      switch (field->cpp_type()) {
        case pb::FieldDescriptor::CPPTYPE_DOUBLE: {
          double value = repeated
                             ? reflection->GetRepeatedDouble(*message, field,
                                                             index)
                             : reflection->GetDouble(*message, field);
          if (field->name() == "yaw_rad" ||
              field->name() == "initial_yaw_rad") {
            constexpr double two = 2.0;
            const double pi = std::acos(-1.0);
            value = std::fmod(value + pi, two * pi);
            if (value < 0.0) {
              value += two * pi;
            }
            value -= pi;
          }
          if (value == 0.0) {
            value = 0.0;
          }
          repeated ? reflection->SetRepeatedDouble(message, field, index, value)
                   : reflection->SetDouble(message, field, value);
          break;
        }
        case pb::FieldDescriptor::CPPTYPE_FLOAT: {
          float value = repeated
                            ? reflection->GetRepeatedFloat(*message, field, index)
                            : reflection->GetFloat(*message, field);
          if (value == 0.0F) {
            value = 0.0F;
          }
          repeated ? reflection->SetRepeatedFloat(message, field, index, value)
                   : reflection->SetFloat(message, field, value);
          break;
        }
        case pb::FieldDescriptor::CPPTYPE_STRING:
          if (field->type() == pb::FieldDescriptor::TYPE_STRING) {
            const std::string value =
                repeated ? reflection->GetRepeatedString(*message, field, index)
                         : reflection->GetString(*message, field);
            auto normalized = normalize_nfc(value);
            if (!normalized.has_value()) {
              auto error = normalized.error();
              error.path = field_path;
              return error;
            }
            repeated ? reflection->SetRepeatedString(
                           message, field, index, normalized.value())
                     : reflection->SetString(message, field,
                                             normalized.value());
          }
          break;
        case pb::FieldDescriptor::CPPTYPE_MESSAGE: {
          pb::Message* child =
              repeated ? reflection->MutableRepeatedMessage(message, field, index)
                       : reflection->MutableMessage(message, field);
          if (auto error = normalize_canonical_message(child, field_path)) {
            return error;
          }
          break;
        }
        case pb::FieldDescriptor::CPPTYPE_INT32:
        case pb::FieldDescriptor::CPPTYPE_INT64:
        case pb::FieldDescriptor::CPPTYPE_UINT32:
        case pb::FieldDescriptor::CPPTYPE_UINT64:
        case pb::FieldDescriptor::CPPTYPE_BOOL:
        case pb::FieldDescriptor::CPPTYPE_ENUM:
          break;
      }
    }
  }
  return std::nullopt;
}

[[nodiscard]] AdapterResult<Hash256> expected_content_identity(
    const pb::Message& message, const IdentityMetadata& metadata) {
  const auto* identity_field = message.GetDescriptor()->FindFieldByName(
      metadata.identity_field_name);
  if (identity_field == nullptr) {
    return AdapterResult<Hash256>::failure(make_error(
        AdapterErrorCode::schema_mismatch, message.GetTypeName(),
        "schema does not declare a canonical self identity"));
  }
  std::unique_ptr<pb::Message> canonical(message.New());
  canonical->CopyFrom(message);
  auto* reflection = canonical->GetReflection();
  reflection->ClearField(canonical.get(), identity_field);
  if (metadata.clear_header_for_identity) {
    const auto* header = canonical->GetDescriptor()->FindFieldByName("header");
    if (header != nullptr) {
      reflection->ClearField(canonical.get(), header);
    }
  }
  if (auto error =
          normalize_canonical_message(canonical.get(), canonical->GetTypeName())) {
    return AdapterResult<Hash256>::failure(std::move(*error));
  }
  auto serialized = deterministic_serialize(*canonical);
  if (!serialized.has_value()) {
    return AdapterResult<Hash256>::failure(serialized.error());
  }
  return AdapterResult<Hash256>::success(sha256(serialized.value()));
}

[[nodiscard]] std::optional<AdapterError> verify_content_identity(
    const pb::Message& message, const IdentityMetadata& metadata) {
  auto expected = expected_content_identity(message, metadata);
  if (!expected.has_value()) {
    return expected.error();
  }
  const auto* identity_field = message.GetDescriptor()->FindFieldByName(
      metadata.identity_field_name);
  const pb::Message& identity =
      message.GetReflection()->GetMessage(message, identity_field);
  const auto* sha_field = identity.GetDescriptor()->FindFieldByName("sha256");
  const std::string actual = identity.GetReflection()->GetString(identity,
                                                                 sha_field);
  if (actual.size() != expected.value().size()) {
    return make_error(AdapterErrorCode::invalid_content_identity,
                      message.GetTypeName(),
                      "content identity has the wrong length");
  }
  std::uint8_t difference = 0;
  for (std::size_t index = 0; index < expected.value().size(); ++index) {
    difference = static_cast<std::uint8_t>(
        difference |
        (static_cast<std::uint8_t>(actual[index]) ^ expected.value()[index]));
  }
  if (difference != 0) {
    return make_error(AdapterErrorCode::invalid_content_identity,
                      message.GetTypeName(),
                      "content identity does not match canonical bytes");
  }
  return std::nullopt;
}

[[nodiscard]] std::optional<AdapterError> verify_content_identity_tree(
    const pb::Message& message, const bool include_root = true) {
  const auto* descriptor = message.GetDescriptor();
  const auto* reflection = message.GetReflection();
  for (int field_index = 0; field_index < descriptor->field_count();
       ++field_index) {
    const auto* field = descriptor->field(field_index);
    if (field->cpp_type() != pb::FieldDescriptor::CPPTYPE_MESSAGE) {
      continue;
    }
    const int count = field->is_repeated()
                          ? reflection->FieldSize(message, field)
                          : (field->has_presence() &&
                                     !reflection->HasField(message, field)
                                 ? 0
                                 : 1);
    for (int index = 0; index < count; ++index) {
      const pb::Message& child =
          field->is_repeated()
              ? reflection->GetRepeatedMessage(message, field, index)
              : reflection->GetMessage(message, field);
      if (auto error = verify_content_identity_tree(child)) {
        return error;
      }
    }
  }
  if (!include_root) {
    return std::nullopt;
  }
  const auto metadata = identity_metadata(*descriptor);
  return metadata.has_value() ? verify_content_identity(message, *metadata)
                              : std::nullopt;
}

[[nodiscard]] std::optional<AdapterError> verify_nested_content_identities(
    const pb::Message& message) {
  return verify_content_identity_tree(message, false);
}

[[nodiscard]] std::optional<AdapterError> install_content_identities(
    pb::Message* message) {
  const auto* descriptor = message->GetDescriptor();
  auto* reflection = message->GetReflection();
  for (int field_index = 0; field_index < descriptor->field_count();
       ++field_index) {
    const auto* field = descriptor->field(field_index);
    if (field->cpp_type() != pb::FieldDescriptor::CPPTYPE_MESSAGE) {
      continue;
    }
    const int count = field->is_repeated()
                          ? reflection->FieldSize(*message, field)
                          : (field->has_presence() &&
                                     !reflection->HasField(*message, field)
                                 ? 0
                                 : 1);
    for (int index = 0; index < count; ++index) {
      pb::Message* child =
          field->is_repeated()
              ? reflection->MutableRepeatedMessage(message, field, index)
              : reflection->MutableMessage(message, field);
      if (auto error = install_content_identities(child)) {
        return error;
      }
    }
  }
  if (descriptor == contract::ScoutPlan::descriptor()) {
    const auto copy_bound_identity = [&](const char* source_message_name,
                                          const char* source_identity_name,
                                          const char* report_identity_name) {
      pb::Message* source = reflection->MutableMessage(
          message, descriptor->FindFieldByName(source_message_name));
      const pb::Message& identity = source->GetReflection()->GetMessage(
          *source,
          source->GetDescriptor()->FindFieldByName(source_identity_name));
      pb::Message* report = reflection->MutableMessage(
          message, descriptor->FindFieldByName("validation_report"));
      report->GetReflection()->MutableMessage(
          report, report->GetDescriptor()->FindFieldByName(report_identity_name))
          ->CopyFrom(identity);
    };
    copy_bound_identity("trajectory", "trajectory_content_identity",
                        "validated_trajectory_content_identity");
    copy_bound_identity("dependencies", "dependencies_content_identity",
                        "validated_dependencies_content_identity");
    copy_bound_identity("survey_evidence", "evidence_content_identity",
                        "validated_survey_evidence_content_identity");
    pb::Message* report = reflection->MutableMessage(
        message, descriptor->FindFieldByName("validation_report"));
    if (auto error = install_content_identities(report)) {
      return error;
    }
  }
  const auto metadata = identity_metadata(*descriptor);
  if (!metadata.has_value()) {
    return std::nullopt;
  }
  auto expected = expected_content_identity(*message, *metadata);
  if (!expected.has_value()) {
    return expected.error();
  }
  const auto* identity_field =
      descriptor->FindFieldByName(metadata->identity_field_name);
  pb::Message* identity = reflection->MutableMessage(message, identity_field);
  const auto* sha = identity->GetDescriptor()->FindFieldByName("sha256");
  identity->GetReflection()->SetString(
      identity, sha,
      std::string(expected.value().begin(), expected.value().end()));
  return std::nullopt;
}

[[nodiscard]] std::optional<AdapterError> enforce_wire_limit(
    const ContractKind kind, const std::size_t wire_bytes,
    const std::size_t requested_limit = 0) {
  const std::size_t absolute_limit = contract_metadata(kind).maximum_wire_bytes;
  const std::size_t effective_limit =
      requested_limit == 0 ? absolute_limit
                           : std::min(requested_limit, absolute_limit);
  if (wire_bytes > effective_limit) {
    return make_error(AdapterErrorCode::resource_limit_exceeded,
                      std::string(schema_name(kind)),
                      "wire payload exceeds the activated interface limit");
  }
  return std::nullopt;
}

[[nodiscard]] std::unique_ptr<pb::Message> new_message(
    const ContractKind kind) {
  const auto* descriptor = contract_metadata(kind).descriptor;
  if (descriptor == nullptr) {
    return nullptr;
  }
  const auto* prototype =
      pb::MessageFactory::generated_factory()->GetPrototype(descriptor);
  return prototype == nullptr ? nullptr
                              : std::unique_ptr<pb::Message>(prototype->New());
}

}  // namespace

AdapterResult<ProtobufAdapter::DecodedDocument>
ProtobufAdapter::decode_document(const ContractKind kind,
                                 const std::string_view wire,
                                 const AdapterValidationOptions& options) {
  if (auto error =
          enforce_wire_limit(kind, wire.size(), options.maximum_message_bytes)) {
    return AdapterResult<DecodedDocument>::failure(std::move(*error));
  }
  auto message = new_message(kind);
  if (message == nullptr) {
    return AdapterResult<DecodedDocument>::failure(make_error(
        AdapterErrorCode::schema_mismatch, std::string(schema_name(kind)),
        "golden protobuf schema is not linked"));
  }
  if (wire.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      !message->ParseFromArray(wire.data(), static_cast<int>(wire.size()))) {
    return AdapterResult<DecodedDocument>::failure(make_error(
        AdapterErrorCode::parse_failed, std::string(schema_name(kind)),
        "wire payload is not a complete message of the expected schema"));
  }
  if (auto error = validate_message(
          *message,
          {options.version_floor, options.interface_limits,
           ValidationContext::Mode::consumer, false},
          message->GetTypeName())) {
    return AdapterResult<DecodedDocument>::failure(std::move(*error));
  }
  if (auto error = verify_content_identity_tree(*message)) {
    return AdapterResult<DecodedDocument>::failure(std::move(*error));
  }
  if (auto error =
          normalize_canonical_message(message.get(), message->GetTypeName())) {
    return AdapterResult<DecodedDocument>::failure(std::move(*error));
  }
  auto canonical = deterministic_serialize(*message);
  if (!canonical.has_value()) {
    return AdapterResult<DecodedDocument>::failure(canonical.error());
  }
  DecodedDocument decoded{message_to_core(*message), sha256(canonical.value())};
  return AdapterResult<DecodedDocument>::success(std::move(decoded));
}

AdapterResult<std::string> ProtobufAdapter::encode_document(
    const ContractKind kind, const CoreMessage& document) {
  auto message = new_message(kind);
  if (message == nullptr) {
    return AdapterResult<std::string>::failure(make_error(
        AdapterErrorCode::schema_mismatch, std::string(schema_name(kind)),
        "golden protobuf schema is not linked"));
  }
  if (auto error = core_to_message(document, message.get(),
                                   std::string(schema_name(kind)))) {
    return AdapterResult<std::string>::failure(std::move(*error));
  }
  if (auto error =
          validate_message(*message,
                           {nullptr, nullptr,
                            ValidationContext::Mode::consumer, false},
                           message->GetTypeName())) {
    return AdapterResult<std::string>::failure(std::move(*error));
  }
  if (auto error = verify_content_identity_tree(*message)) {
    return AdapterResult<std::string>::failure(std::move(*error));
  }
  if (auto error =
          normalize_canonical_message(message.get(), message->GetTypeName())) {
    return AdapterResult<std::string>::failure(std::move(*error));
  }
  auto serialized = deterministic_serialize(*message);
  if (!serialized.has_value()) {
    return serialized;
  }
  if (auto error = enforce_wire_limit(kind, serialized.value().size())) {
    return AdapterResult<std::string>::failure(std::move(*error));
  }
  return serialized;
}

AdapterResult<Hash256> ProtobufAdapter::canonical_content_identity_for(
    const ContractKind kind, const std::string_view wire,
    const AdapterValidationOptions& options) {
  if (auto error =
          enforce_wire_limit(kind, wire.size(), options.maximum_message_bytes)) {
    return AdapterResult<Hash256>::failure(std::move(*error));
  }
  auto message = new_message(kind);
  if (message == nullptr ||
      wire.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      !message->ParseFromArray(wire.data(), static_cast<int>(wire.size()))) {
    return AdapterResult<Hash256>::failure(make_error(
        AdapterErrorCode::parse_failed, std::string(schema_name(kind)),
        "wire payload cannot be canonicalized as the requested schema"));
  }
  if (auto error =
          validate_message(*message,
                           {options.version_floor, options.interface_limits,
                            ValidationContext::Mode::consumer, false},
                           message->GetTypeName())) {
    return AdapterResult<Hash256>::failure(std::move(*error));
  }
  if (auto error = verify_nested_content_identities(*message)) {
    return AdapterResult<Hash256>::failure(std::move(*error));
  }
  const auto metadata = identity_metadata(*message->GetDescriptor());
  if (!metadata.has_value()) {
    return AdapterResult<Hash256>::failure(make_error(
        AdapterErrorCode::schema_mismatch, message->GetTypeName(),
        "schema does not declare a canonical self identity"));
  }
  return expected_content_identity(*message, *metadata);
}

AdapterResult<std::string>
ProtobufAdapter::canonicalize_and_identify_document(
    const ContractKind kind, const std::string_view wire,
    const AdapterValidationOptions& options) {
  if (auto error =
          enforce_wire_limit(kind, wire.size(), options.maximum_message_bytes)) {
    return AdapterResult<std::string>::failure(std::move(*error));
  }
  auto message = new_message(kind);
  if (message == nullptr ||
      wire.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      !message->ParseFromArray(wire.data(), static_cast<int>(wire.size()))) {
    return AdapterResult<std::string>::failure(make_error(
        AdapterErrorCode::parse_failed, std::string(schema_name(kind)),
        "wire payload cannot be identified as the requested schema"));
  }
  if (auto error =
          validate_message(*message,
                           {options.version_floor, options.interface_limits,
                            ValidationContext::Mode::producer_before_normalization,
                            false},
                           message->GetTypeName())) {
    return AdapterResult<std::string>::failure(std::move(*error));
  }
  if (auto error =
          normalize_canonical_message(message.get(), message->GetTypeName())) {
    return AdapterResult<std::string>::failure(std::move(*error));
  }
  if (auto error =
          validate_message(*message,
                           {options.version_floor, options.interface_limits,
                            ValidationContext::Mode::producer_canonical, false},
                           message->GetTypeName())) {
    return AdapterResult<std::string>::failure(std::move(*error));
  }
  if (auto error = install_content_identities(message.get())) {
    return AdapterResult<std::string>::failure(std::move(*error));
  }
  if (auto error =
          validate_message(*message,
                           {options.version_floor, options.interface_limits,
                            ValidationContext::Mode::consumer, false},
                           message->GetTypeName())) {
    return AdapterResult<std::string>::failure(std::move(*error));
  }
  auto serialized = deterministic_serialize(*message);
  if (!serialized.has_value()) {
    return serialized;
  }
  if (auto error = enforce_wire_limit(kind, serialized.value().size(),
                                      options.maximum_message_bytes)) {
    return AdapterResult<std::string>::failure(std::move(*error));
  }
  return serialized;
}

}  // namespace scout_planner::core
