#pragma once

#include "scout_planner/core/hybrid_map_query.hpp"
#include "scout_planner/core/protobuf_adapter.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace scout_planner::core {

struct CoordinationMotionSample {
  std::uint64_t time_offset_ns{};
  Point3dEnu position_m{};
};

struct CoordinationEvaluationConfig {
  // These are Scout-local receipt times. Sender-domain validity bounds are
  // intentionally not compared with this clock.
  std::int64_t now_monotonic_ns{};
  std::int64_t prediction_received_at_monotonic_ns{};
  std::int64_t constraint_received_at_monotonic_ns{};
  std::uint64_t prediction_reject_ns{1};
  std::uint64_t constraint_reject_ns{1};
  std::uint64_t maximum_sync_uncertainty_ns{0};
  std::string scout_frame_id{"mission_enu"};
};

enum class CoordinationFailure {
  invalid_input,
  mission_mismatch,
  prediction_mismatch,
  clock_domain_mismatch,
  dependency_stale,
  synchronization_invalid,
  prediction_horizon_exceeded,
  prediction_gap,
  coordination_infeasible,
  separation_violation,
  communication_distance_violation,
};

struct CoordinationError {
  CoordinationFailure code{CoordinationFailure::invalid_input};
  std::uint64_t earliest_failure_time_offset_ns{};
  double minimum_margin_m{};
  std::string detail;
};

struct CoordinationReport {
  bool separation_passed{true};
  bool communication_distance_passed{true};
  // False for geometric-distance-only inputs. No link quality is inferred.
  bool calibrated_link_quality_asserted{false};
  double minimum_separation_margin_m{};
  double minimum_communication_margin_m{};
  std::optional<std::uint64_t> earliest_failure_time_offset_ns;
  std::uint64_t evaluated_horizon_ns{};
};

class CoordinationEvaluationResult final {
 public:
  static CoordinationEvaluationResult success(CoordinationReport report);
  static CoordinationEvaluationResult failure(CoordinationError error);

  [[nodiscard]] bool has_value() const noexcept;
  [[nodiscard]] const CoordinationReport& value() const;
  [[nodiscard]] const CoordinationError& error() const;

 private:
  explicit CoordinationEvaluationResult(CoordinationReport report);
  explicit CoordinationEvaluationResult(CoordinationError error);
  std::variant<CoordinationReport, CoordinationError> storage_;
};

class DualRobotCoordinationEvaluator final {
 public:
  [[nodiscard]] static CoordinationEvaluationResult evaluate(
      const MainRobotPrediction& prediction,
      const CoordinationConstraint& constraint,
      const std::vector<CoordinationMotionSample>& scout_motion,
      const CoordinationEvaluationConfig& configuration);
};

}  // namespace scout_planner::core
