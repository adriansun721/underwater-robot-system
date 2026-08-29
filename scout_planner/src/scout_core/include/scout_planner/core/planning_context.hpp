#pragma once

#include "scout_planner/core/protobuf_adapter.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace scout_planner::core {

enum class PlanningInputKind : std::size_t {
  mission,
  navigation,
  map,
  current,
  sensor_geometry,
  sensor_health,
  capability,
  thruster_health,
  energy_model,
  energy_state,
  main_robot_prediction,
  coordination,
  dependencies,
  count,
};

enum class ContextErrorCode {
  missing_input,
  capture_race,
  future_receipt,
  future_timestamp,
  stale_input,
  clock_domain_mismatch,
  synchronization_invalid,
  synchronization_tolerance_exceeded,
  session_rollback,
  sequence_rollback,
  version_rollback,
  identity_conflict,
  mission_mismatch,
  prediction_mismatch,
  dependency_mismatch,
  operating_domain_mismatch,
  admission_rejected,
  producer_mismatch,
  invalid_configuration,
};

struct ContextError {
  ContextErrorCode code;
  PlanningInputKind input;
  std::string detail;
};

struct ProfileBinding {
  std::string id;
  std::uint64_t version{};
  Hash256 content_identity{};
};

struct ActivatedPlanningConfiguration {
  std::string scout_clock_domain_id;
  std::string operating_domain_id;
  ProfileBinding planner_configuration;
  ProfileBinding timing_profile;
  ProfileBinding interface_limits;
  std::uint64_t activation_generation{};
  std::array<std::uint64_t,
             static_cast<std::size_t>(PlanningInputKind::count)>
      maximum_input_age_ns{};
  std::uint64_t maximum_synchronization_delta_ns{};
  std::uint64_t maximum_synchronization_uncertainty_ns{};
};

struct InputReceipt {
  std::int64_t local_received_at_monotonic_ns;
};

struct AdmittedSurveyTask {
  SurveyTask mission;
  InputReceipt mission_receipt;
  ScoutMissionDecision decision;
  InputReceipt decision_receipt;
};

template <typename Value>
struct ReceivedInput {
  Value value;
  InputReceipt receipt;
};

struct PairedSensorInput {
  ReceivedInput<ScoutSensorGeometry> geometry;
  ReceivedInput<ScoutSensorHealthState> health;
};

struct PlanningContextInputs {
  AdmittedSurveyTask mission;
  ReceivedInput<ScoutNavigationState> navigation;
  ReceivedInput<HybridMapSnapshot> map;
  ReceivedInput<ScoutCurrentEstimate> current;
  std::vector<PairedSensorInput> sensors;
  ReceivedInput<ScoutCapabilityProfile> capability;
  ReceivedInput<ScoutThrusterHealthState> thruster_health;
  ReceivedInput<ScoutEnergyModelProfile> energy_model;
  ReceivedInput<ScoutEnergyState> energy_state;
  ReceivedInput<MainRobotPrediction> main_robot_prediction;
  ReceivedInput<CoordinationConstraint> coordination;
  ReceivedInput<ScoutPlanningDependencies> dependencies;
};

class PlanningContextSource {
 public:
  virtual ~PlanningContextSource() = default;

  [[nodiscard]] virtual std::uint64_t capture_generation() const noexcept = 0;
  [[nodiscard]] virtual std::optional<PlanningContextInputs> read_inputs()
      const = 0;
};

class ScoutPlanningContext final {
 public:
  ScoutPlanningContext() = delete;

  [[nodiscard]] const PlanningContextInputs& inputs() const noexcept {
    return inputs_;
  }

  [[nodiscard]] const ActivatedPlanningConfiguration& configuration()
      const noexcept {
    return configuration_;
  }

  [[nodiscard]] std::int64_t captured_at_monotonic_ns() const noexcept {
    return captured_at_monotonic_ns_;
  }

  [[nodiscard]] std::uint64_t source_generation() const noexcept {
    return source_generation_;
  }

 private:
  friend class PlanningContextBuilder;

  ScoutPlanningContext(PlanningContextInputs inputs,
                       ActivatedPlanningConfiguration configuration,
                       const std::int64_t captured_at_monotonic_ns,
                       const std::uint64_t source_generation)
      : inputs_(std::move(inputs)),
        configuration_(std::move(configuration)),
        captured_at_monotonic_ns_(captured_at_monotonic_ns),
        source_generation_(source_generation) {}

  PlanningContextInputs inputs_;
  ActivatedPlanningConfiguration configuration_;
  std::int64_t captured_at_monotonic_ns_;
  std::uint64_t source_generation_;
};

class PlanningContextResult final {
 public:
  [[nodiscard]] static PlanningContextResult success(
      ScoutPlanningContext value) {
    return PlanningContextResult(std::move(value));
  }

  [[nodiscard]] static PlanningContextResult failure(ContextError error) {
    return PlanningContextResult(std::move(error));
  }

  [[nodiscard]] bool has_value() const noexcept { return value_.has_value(); }
  [[nodiscard]] const ScoutPlanningContext& value() const { return value_.value(); }
  [[nodiscard]] const ContextError& error() const { return error_.value(); }

 private:
  explicit PlanningContextResult(ScoutPlanningContext value)
      : value_(std::move(value)) {}
  explicit PlanningContextResult(ContextError error)
      : error_(std::move(error)) {}

  std::optional<ScoutPlanningContext> value_;
  std::optional<ContextError> error_;
};

class PlanningContextBuilder final {
 public:
  [[nodiscard]] PlanningContextResult capture(
      const PlanningContextSource& source,
      const ActivatedPlanningConfiguration& configuration,
      std::int64_t captured_at_monotonic_ns);

 private:
  struct DeliveryWatermark {
    struct Member {
      std::uint64_t sequence;
      Hash256 wire_identity;
    };

    std::string producer_id;
    std::string producer_session_id;
    std::uint64_t maximum_sequence;
    Hash256 maximum_sequence_wire_identity;
    std::map<std::string, Member> members;
  };

  struct BusinessWatermark {
    std::uint64_t version;
    std::string content_identity;
  };

  struct CoordinationSessions {
    std::string prediction_session_id;
    std::string coordination_session_id;
  };

  std::mutex watermark_mutex_;
  std::map<std::string, DeliveryWatermark> delivery_watermarks_;
  std::map<std::string, BusinessWatermark> business_watermarks_;
  std::map<std::string, std::set<std::string>> retired_delivery_sessions_;
  std::optional<CoordinationSessions> coordination_sessions_;
};

}  // namespace scout_planner::core
