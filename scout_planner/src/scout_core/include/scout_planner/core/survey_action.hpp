#pragma once

#include "scout_planner/core/hybrid_map_query.hpp"
#include "scout_planner/core/planning_context.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace scout_planner::core {

enum class SurveyActionPhase { approach, observe, exit };

struct SurveyActionSegment {
  SurveyActionPhase phase{SurveyActionPhase::approach};
  Point3dEnu start_m{};
  Point3dEnu end_m{};
  double duration_s{};
  double yaw_rad{};
  double minimum_dwell_s{};
};

struct ObservationPose {
  Point3dEnu position_m{};
  double yaw_rad{};
  double pitch_rad{};
  double roll_rad{};
  double dwell_s{};
};

enum class SurveyActionFailure {
  invalid_input,
  mission_region_invalid,
  sensor_geometry_invalid,
  sensor_health_invalid,
  sensor_not_nominal,
  no_observation_pose,
  approach_infeasible,
  exit_infeasible,
  insufficient_coverage,
  mandatory_coverage_missing,
};

struct SurveyActionError {
  SurveyActionFailure code{SurveyActionFailure::invalid_input};
  std::size_t sample_index{};
  std::string detail;
};

struct SurveyActionReport {
  std::vector<SurveyActionSegment> segments;
  std::vector<ObservationPose> observation_poses;
  double conservative_coverage_ratio{};
  double mandatory_coverage_ratio{};
  std::size_t required_sample_count{};
  std::size_t covered_sample_count{};
  std::size_t mandatory_sample_count{};
  std::size_t mandatory_covered_sample_count{};
  std::string sensor_id;
  std::uint64_t geometry_version{};
  std::uint64_t health_version{};
};

class SurveyActionResult final {
 public:
  static SurveyActionResult success(SurveyActionReport report);
  static SurveyActionResult failure(SurveyActionError error);
  [[nodiscard]] bool has_value() const noexcept;
  [[nodiscard]] const SurveyActionReport& value() const;
  [[nodiscard]] const SurveyActionError& error() const;

 private:
  explicit SurveyActionResult(SurveyActionReport report);
  explicit SurveyActionResult(SurveyActionError error);
  std::variant<SurveyActionReport, SurveyActionError> storage_;
};

struct SurveyActionConfig {
  double observation_dwell_s{1.0};
  double transit_speed_mps{0.5};
  double pose_position_error_m{0.0};
  double pose_range_error_m{0.0};
  double sample_resolution_m{0.0};
  double minimum_coverage_ratio{0.0};
  std::optional<Aabb3dEnu> mandatory_region;
};

class SurveyActionPlanner final {
 public:
  [[nodiscard]] static SurveyActionResult plan(
      const ScoutPlanningContext& context, const SurveyActionConfig& config);
};

}  // namespace scout_planner::core
