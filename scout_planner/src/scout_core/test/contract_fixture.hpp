#pragma once

#include "scout_planner/core/protobuf_adapter.hpp"
#include "test_support.hpp"

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include <cstdint>
#include <memory>
#include <string>

namespace scout_planner::test_contract_fixture {

namespace pb = google::protobuf;

inline std::string expected_frame(const pb::Descriptor& descriptor,
                                  const pb::FieldDescriptor& field) {
  if (field.name() == "body_frame_id" ||
      descriptor.name() == "BodyTwist3dFlu") {
    return "base_link";
  }
  if (field.name() == "sensor_frame_id") {
    return "sensor_optical";
  }
  return "mission_enu";
}

inline int repeated_fixture_count(const pb::Descriptor& descriptor,
                                  const pb::FieldDescriptor& field) {
  if ((descriptor.name() == "Region3dEnu" ||
       descriptor.name() == "CurrentRegion3dEnu") &&
      field.name() == "xyz_m") {
    return 6;
  }
  if ((descriptor.name() == "Covariance3d" &&
       field.name() == "row_major") ||
      (descriptor.name() == "CurrentGradient3d" &&
       field.name() == "row_major_per_s")) {
    return 9;
  }
  if (descriptor.name() == "ScoutBezierSegment4d" &&
      (field.name() == "position_control_points" ||
       field.name() == "yaw_offset_control_points_rad")) {
    return 6;
  }
  return 1;
}

inline double fixture_double(const pb::Descriptor& descriptor,
                             const pb::FieldDescriptor& field,
                             const int index) {
  if ((descriptor.name() == "Region3dEnu" ||
       descriptor.name() == "CurrentRegion3dEnu") &&
      field.name() == "xyz_m") {
    return index < 3 ? -1.0 : 1.0;
  }
  if (descriptor.name() == "Covariance3d" &&
      field.name() == "row_major") {
    return index == 0 || index == 4 || index == 8 ? 0.25 : 0.0;
  }
  if (field.name() == "q_w") {
    return 1.0;
  }
  if (field.name() == "conservative_occupied_radius_m") {
    return 1.0;
  }
  if (field.name() == "q_x" || field.name() == "q_y" ||
      field.name() == "q_z" || field.name() == "initial_yaw_rad" ||
      field.name() == "yaw_offset_control_points_rad") {
    return 0.0;
  }
  if (field.name().find("minimum_") == 0U) {
    return 0.1;
  }
  if (field.name().find("maximum_") == 0U) {
    return 1.0;
  }
  return 0.5;
}

inline std::uint64_t fixture_uint64(const pb::FieldDescriptor& field) {
  const auto& name = field.name();
  if (name.find("until") != std::string::npos ||
      name.find("expires") != std::string::npos ||
      name.find("end_time") != std::string::npos) {
    return 200U;
  }
  if (name == "start_time_offset_ns" || name == "start_offset_ns") {
    return 0U;
  }
  if (name == "end_offset_ns") {
    return 1U;
  }
  return 1U;
}

inline std::int64_t fixture_int64(const pb::FieldDescriptor& field) {
  const auto& name = field.name();
  if (name.find("generated") != std::string::npos) {
    return 300;
  }
  if (name.find("until") != std::string::npos ||
      name.find("expires") != std::string::npos ||
      name.find("deadline") != std::string::npos ||
      name.find("evaluated") != std::string::npos) {
    return 200;
  }
  return 100;
}

inline void populate_fixture_message(pb::Message* message);

inline void populate_fixture_field(pb::Message* message,
                                   const pb::FieldDescriptor& field,
                                   const int repeated_index) {
  const auto* descriptor = message->GetDescriptor();
  auto* reflection = message->GetReflection();
  const bool repeated = field.is_repeated();
  switch (field.cpp_type()) {
    case pb::FieldDescriptor::CPPTYPE_INT32:
      repeated ? reflection->AddInt32(message, &field, 1)
               : reflection->SetInt32(message, &field, 1);
      break;
    case pb::FieldDescriptor::CPPTYPE_INT64: {
      const auto value = fixture_int64(field);
      repeated ? reflection->AddInt64(message, &field, value)
               : reflection->SetInt64(message, &field, value);
      break;
    }
    case pb::FieldDescriptor::CPPTYPE_UINT32:
      repeated ? reflection->AddUInt32(message, &field, 1U)
               : reflection->SetUInt32(message, &field, 1U);
      break;
    case pb::FieldDescriptor::CPPTYPE_UINT64: {
      const auto value = fixture_uint64(field);
      repeated ? reflection->AddUInt64(message, &field, value)
               : reflection->SetUInt64(message, &field, value);
      break;
    }
    case pb::FieldDescriptor::CPPTYPE_FLOAT:
      repeated ? reflection->AddFloat(message, &field, 0.5F)
               : reflection->SetFloat(message, &field, 0.5F);
      break;
    case pb::FieldDescriptor::CPPTYPE_DOUBLE: {
      const auto value = fixture_double(*descriptor, field, repeated_index);
      repeated ? reflection->AddDouble(message, &field, value)
               : reflection->SetDouble(message, &field, value);
      break;
    }
    case pb::FieldDescriptor::CPPTYPE_BOOL:
      repeated ? reflection->AddBool(message, &field, true)
               : reflection->SetBool(message, &field, true);
      break;
    case pb::FieldDescriptor::CPPTYPE_ENUM: {
      const auto* value = field.name() == "channel_id"
                              ? field.enum_type()->FindValueByNumber(3)
                              : field.enum_type()->value(1);
      repeated ? reflection->AddEnum(message, &field, value)
               : reflection->SetEnum(message, &field, value);
      break;
    }
    case pb::FieldDescriptor::CPPTYPE_STRING: {
      std::string value;
      if (field.type() == pb::FieldDescriptor::TYPE_BYTES) {
        value.assign(field.name().find("identity") != std::string::npos ||
                             field.name() == "sha256"
                         ? 32U
                         : 16U,
                     '\x5a');
      } else if (field.name().find("frame_id") != std::string::npos) {
        value = expected_frame(*descriptor, field);
      } else if (field.name().find("version") != std::string::npos) {
        value = "v1";
      } else {
        value = "fixture-id";
      }
      repeated ? reflection->AddString(message, &field, value)
               : reflection->SetString(message, &field, value);
      break;
    }
    case pb::FieldDescriptor::CPPTYPE_MESSAGE: {
      auto* child = repeated ? reflection->AddMessage(message, &field)
                             : reflection->MutableMessage(message, &field);
      populate_fixture_message(child);
      if (field.name() == "attitude_covariance_rad2") {
        const auto* frame = child->GetDescriptor()->FindFieldByName("frame_id");
        child->GetReflection()->SetString(child, frame, "base_link");
      }
      break;
    }
  }
}

inline void populate_fixture_message(pb::Message* message) {
  const auto* descriptor = message->GetDescriptor();
  for (int field_index = 0; field_index < descriptor->field_count();
       ++field_index) {
    const auto* field = descriptor->field(field_index);
    if (descriptor->name() == "ScoutCoordinationConstraint" &&
        field->name() == "calibrated_link_model") {
      continue;
    }
    if (descriptor->name() == "ScoutPlanValidationReport" &&
        field->name() == "earliest_failure_time_offset_ns") {
      continue;
    }
    const int count = field->is_repeated()
                          ? repeated_fixture_count(*descriptor, *field)
                          : 1;
    for (int index = 0; index < count; ++index) {
      populate_fixture_field(message, *field, index);
    }
  }
}

inline std::unique_ptr<pb::Message> valid_contract(
    const std::string& schema_name) {
  const auto* descriptor =
      pb::DescriptorPool::generated_pool()->FindMessageTypeByName(schema_name);
  scout_planner::test_support::require(descriptor != nullptr,
                                       "golden schema is not linked");
  const auto* prototype =
      pb::MessageFactory::generated_factory()->GetPrototype(descriptor);
  scout_planner::test_support::require(prototype != nullptr,
                                       "generated prototype is unavailable");
  std::unique_ptr<pb::Message> result(prototype->New());
  populate_fixture_message(result.get());
  return result;
}

template <scout_planner::core::ContractKind Kind>
void refresh_fixture_identities(pb::Message* message) {
  const auto identified =
      scout_planner::core::ProtobufAdapter::canonicalize_and_identify<Kind>(
          message->SerializeAsString());
  scout_planner::test_support::require(identified.has_value(),
                                       identified.error().message);
  scout_planner::test_support::require(
      message->ParseFromString(identified.value()),
      "identified fixture is not valid protobuf");
}

template <typename Message>
Message populated_message() {
  Message message;
  populate_fixture_message(&message);
  return message;
}

template <scout_planner::core::ContractKind Kind, typename Message>
void identify_in_place(Message* message) {
  const auto identified =
      scout_planner::core::ProtobufAdapter::canonicalize_and_identify<Kind>(
          message->SerializeAsString());
  scout_planner::test_support::require(identified.has_value(),
                                       identified.error().message);
  scout_planner::test_support::require(
      message->ParseFromString(identified.value()),
      "identified fixture is not valid protobuf");
}

template <scout_planner::core::ContractKind Kind, typename Message>
scout_planner::core::CoreContract<Kind> decode_identified(
    const Message& message) {
  const auto decoded = scout_planner::core::ProtobufAdapter::decode<Kind>(
      message.SerializeAsString());
  scout_planner::test_support::require(decoded.has_value(),
                                       decoded.error().message);
  return decoded.value();
}

template <scout_planner::core::ContractKind Kind, typename Message>
scout_planner::core::CoreContract<Kind> identify_and_decode(Message message) {
  identify_in_place<Kind>(&message);
  return decode_identified<Kind>(message);
}

}  // namespace scout_planner::test_contract_fixture
