#pragma once

#include "scout_planner/core/planning_context.hpp"
#include "scout_planner/core/quintic_bezier.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>

namespace scout_planner::core {

struct SurveyPlanEvidence {
  std::uint64_t mission_id{};
  std::uint64_t mission_version{};
  Hash256 mission_content_identity{};
  std::string baseline_map_id;
  std::uint64_t baseline_map_version{};
  Hash256 baseline_map_content_identity{};
  Hash256 trajectory_content_identity{};
  std::string planner_configuration_id;
  std::uint64_t planner_configuration_version{};
  Hash256 planner_configuration_content_identity{};
  std::uint64_t computation_config_version{};
  Hash256 computation_config_content_identity{};
  std::string sensor_id;
  std::uint64_t geometry_version{};
  std::uint64_t health_version{};
  Hash256 geometry_content_identity{};
  Hash256 health_content_identity{};
  Aabb3dEnu predicted_covered_region{};
  double conservative_predicted_coverage_ratio{};
  double predicted_resolution_m{};
  double mandatory_coverage_ratio{};
  std::uint64_t trajectory_duration_ns{};
  std::uint64_t evaluated_sample_count{};
  std::uint64_t covered_sample_count{};
  bool approach_valid{};
  bool observe_valid{};
  bool exit_valid{};
  bool completion_evidence{false};
};

enum class SurveyPlanEvidenceFailure {
  invalid_input,
  trajectory_invalid,
  sensor_invalid,
  approach_invalid,
  observe_invalid,
  exit_invalid,
  insufficient_coverage,
  mandatory_coverage_missing,
};

struct SurveyPlanEvidenceError {
  SurveyPlanEvidenceFailure code{SurveyPlanEvidenceFailure::invalid_input};
  std::size_t sample_index{};
  std::string detail;
};

class SurveyPlanEvidenceResult final {
 public:
  static SurveyPlanEvidenceResult success(SurveyPlanEvidence value);
  static SurveyPlanEvidenceResult failure(SurveyPlanEvidenceError error);
  [[nodiscard]] bool has_value() const noexcept;
  [[nodiscard]] const SurveyPlanEvidence& value() const;
  [[nodiscard]] const SurveyPlanEvidenceError& error() const;

 private:
  explicit SurveyPlanEvidenceResult(SurveyPlanEvidence value);
  explicit SurveyPlanEvidenceResult(SurveyPlanEvidenceError error);
  std::variant<SurveyPlanEvidence, SurveyPlanEvidenceError> storage_;
};

struct SurveyPlanEvidenceConfig {
  double minimum_observe_duration_s{0.0};
  double maximum_observe_speed_mps{0.0};
  double sample_period_s{0.1};
  double pose_position_error_m{0.0};
  double pose_range_error_m{0.0};
  double minimum_coverage_ratio{0.0};
  std::uint64_t computation_config_version{1U};
  Hash256 computation_config_content_identity{};
  std::optional<Aabb3dEnu> mandatory_region;
};

class SurveyPlanEvidenceEvaluator final {
 public:
  [[nodiscard]] static SurveyPlanEvidenceResult evaluate(
      const ScoutPlanningContext& context, const BezierTrajectory4d& trajectory,
      const SurveyPlanEvidenceConfig& config = {});
};

}  // namespace scout_planner::core
