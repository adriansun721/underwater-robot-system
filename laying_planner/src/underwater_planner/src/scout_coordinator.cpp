#include "underwater_planner/core/scout_coordinator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace underwater_planner::core {
namespace {

constexpr double kEpsilon = 1.0e-9;

bool finite(const double value) noexcept { return std::isfinite(value); }

bool finite_pose(const Pose2d& pose) noexcept {
  return finite(pose.x_m) && finite(pose.y_m) && finite(pose.heading_rad) &&
         pose.timestamp.nanoseconds >= 0;
}

int urgency_rank(const GapUrgency urgency) noexcept {
  switch (urgency) {
    case GapUrgency::blocking:
      return 0;
    case GapUrgency::urgent:
      return 1;
    case GapUrgency::scheduled:
      return 2;
    case GapUrgency::informational:
      return 3;
  }
  return 3;
}

InformationGapReason more_conservative_reason(const InformationGapReason left,
                                              const InformationGapReason right) {
  const auto rank = [](const InformationGapReason reason) {
    switch (reason) {
      case InformationGapReason::unknown:
        return 3;
      case InformationGapReason::invalid_terrain:
        return 2;
      case InformationGapReason::low_confidence:
        return 1;
    }
    return 0;
  };
  return rank(left) >= rank(right) ? left : right;
}

std::optional<std::pair<std::size_t, std::size_t>> map_cell_for(
    const MapSnapshot& map, const ReferencePoint& point) {
  const double column_value =
      (point.x_m - map.origin_x_m) / map.resolution_m;
  const double row_value = (point.y_m - map.origin_y_m) / map.resolution_m;
  if (!finite(column_value) || !finite(row_value) || column_value < 0.0 ||
      row_value < 0.0 || column_value >= static_cast<double>(map.width) ||
      row_value >= static_cast<double>(map.height)) {
    return std::nullopt;
  }
  return std::make_pair(static_cast<std::size_t>(std::floor(row_value)),
                        static_cast<std::size_t>(std::floor(column_value)));
}

struct PathPosition {
  double nearest_arc_m{};
  double remaining_m{};
};

PathPosition path_position_from_pose(const TimedPath& path, const Pose2d& pose) {
  if (path.geometry.points.empty()) return {};
  double nearest_arc = path.geometry.points.front().arc_length_m;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (const PathPoint& point : path.geometry.points) {
    const double distance = std::hypot(point.x_m - pose.x_m, point.y_m - pose.y_m);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_arc = point.arc_length_m;
    }
  }
  return {nearest_arc,
          std::max(0.0, path.geometry.points.back().arc_length_m - nearest_arc)};
}

std::optional<double> time_at_arc(const ExecutionProfile& profile,
                                  const double target_arc) {
  if (profile.samples.empty() || !finite(target_arc)) return std::nullopt;
  if (target_arc < profile.samples.front().arc_length_m - kEpsilon ||
      target_arc > profile.samples.back().arc_length_m + kEpsilon) {
    return std::nullopt;
  }
  for (std::size_t index = 0U; index < profile.samples.size(); ++index) {
    const ExecutionSample& sample = profile.samples[index];
    const double sample_time =
        static_cast<double>(sample.time_from_start.nanoseconds) * 1.0e-9;
    if (target_arc <= sample.arc_length_m + kEpsilon) {
      if (index == 0U) return sample_time;
      const ExecutionSample& previous = profile.samples[index - 1U];
      const double span = sample.arc_length_m - previous.arc_length_m;
      if (!(span > 0.0)) return std::nullopt;
      const double ratio = (target_arc - previous.arc_length_m) / span;
      const double previous_time = static_cast<double>(
          previous.time_from_start.nanoseconds) * 1.0e-9;
      return previous_time + ratio * (sample_time - previous_time);
    }
  }
  return std::nullopt;
}

std::optional<std::int64_t> quantized_progress(const double progress_m) {
  constexpr double kScale = 1.0e6;
  const double scaled = progress_m * kScale;
  // llround is undefined outside the representable integer range.  Leave a
  // half-unit margin so values that round to the endpoint remain fail-closed.
  constexpr double kRoundingMargin = 0.5;
  const double minimum =
      static_cast<double>(std::numeric_limits<std::int64_t>::min()) +
      kRoundingMargin;
  const double maximum =
      static_cast<double>(std::numeric_limits<std::int64_t>::max()) -
      kRoundingMargin;
  if (!finite(scaled) || scaled < minimum || scaled > maximum) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(std::llround(scaled));
}

std::optional<ScoutCoordinator::GapKey> gap_key_for(
    const ScoutTarget& target) {
  const std::optional<std::int64_t> start =
      quantized_progress(target.gap_start_progress_m);
  const std::optional<std::int64_t> end =
      quantized_progress(target.gap_end_progress_m);
  const std::optional<std::int64_t> target_x =
      quantized_progress(target.target_pose.x_m);
  const std::optional<std::int64_t> target_y =
      quantized_progress(target.target_pose.y_m);
  if (!start.has_value() || !end.has_value() || !target_x.has_value() ||
      !target_y.has_value()) {
    return std::nullopt;
  }
  return ScoutCoordinator::GapKey{
      target.source_map_version.map_id,
      target.source_map_version.sequence_number,
      target.reference_line_version,
      *start,
      *end,
      *target_x,
      *target_y};
}

}  // namespace

std::string_view to_string(const GapUrgency urgency) noexcept {
  switch (urgency) {
    case GapUrgency::blocking:
      return "BLOCKING";
    case GapUrgency::urgent:
      return "URGENT";
    case GapUrgency::scheduled:
      return "SCHEDULED";
    case GapUrgency::informational:
      return "INFORMATIONAL";
  }
  return "INFORMATIONAL";
}

bool valid(const ScoutCoordinationParameters& parameters) noexcept {
  return !parameters.parameter_profile_id.empty() &&
         !parameters.operating_domain_id.empty() &&
         finite(parameters.minimum_map_confidence) &&
         parameters.minimum_map_confidence > 0.0 &&
         parameters.minimum_map_confidence <= 1.0 &&
         finite(parameters.sample_interval_m) &&
         parameters.sample_interval_m > 0.0 &&
         finite(parameters.merge_distance_m) && parameters.merge_distance_m >= 0.0 &&
         finite(parameters.minimum_safe_distance_m) &&
         parameters.minimum_safe_distance_m >= 0.0 &&
         finite(parameters.planning_lead_time_s) &&
         parameters.planning_lead_time_s > 0.0 &&
         finite(parameters.average_velocity_mps) &&
         parameters.average_velocity_mps > 0.0 &&
         finite(parameters.hysteresis_distance_m) &&
         parameters.hysteresis_distance_m >= 0.0 &&
         (parameters.hysteresis_distance_m == 0.0 ||
          parameters.hysteresis_distance_m < parameters.minimum_safe_distance_m) &&
         finite(parameters.hysteresis_time_s) && parameters.hysteresis_time_s >= 0.0 &&
         (parameters.hysteresis_time_s == 0.0 ||
          parameters.hysteresis_time_s < parameters.planning_lead_time_s) &&
         parameters.policy_version != 0U &&
         finite(parameters.sensor_coverage_radius_m) &&
         parameters.sensor_coverage_radius_m > 0.0 &&
         finite(parameters.scout_corridor_half_width_m) &&
         parameters.scout_corridor_half_width_m > 0.0 &&
         finite(parameters.communication_max_distance_m) &&
         parameters.communication_max_distance_m > 0.0 &&
         finite(parameters.desired_scout_distance_m) &&
         parameters.desired_scout_distance_m > 0.0 &&
         finite(parameters.continue_scout_distance_m) &&
         finite(parameters.stop_scout_distance_m) &&
         parameters.desired_scout_distance_m <
             parameters.continue_scout_distance_m &&
         parameters.continue_scout_distance_m <
             parameters.stop_scout_distance_m &&
         parameters.stop_scout_distance_m <
             parameters.communication_max_distance_m &&
         finite(parameters.blocking_priority_weight) &&
         parameters.blocking_priority_weight >= 0.0 &&
         finite(parameters.information_value_weight) &&
         parameters.information_value_weight >= 0.0 &&
         finite(parameters.forward_progress_weight) &&
         parameters.forward_progress_weight >= 0.0 &&
         finite(parameters.arrival_cost_weight) &&
         parameters.arrival_cost_weight >= 0.0 &&
         parameters.request_timeout.nanoseconds > 0;
}

ScoutCoordinationParameters make_scout_coordination_parameters(
    const ParameterConfig& config) {
  if (!production_ready(config)) {
    throw std::invalid_argument(
        "scout coordination parameters require a production-ready "
        "versioned parameter profile");
  }
  const TaskParameterConfig& task = config.task;
  ScoutCoordinationParameters parameters;
  parameters.parameter_profile_id = config.profile_id;
  parameters.operating_domain_id = config.operating_domain_id;
  parameters.minimum_map_confidence = *task.scout_minimum_map_confidence;
  parameters.sample_interval_m = *task.scout_sample_interval_m;
  parameters.merge_distance_m = *task.scout_merge_distance_m;
  parameters.minimum_safe_distance_m = *task.scout_minimum_safe_distance_m;
  parameters.planning_lead_time_s = *task.scout_planning_lead_time_s;
  parameters.average_velocity_mps = *task.scout_average_velocity_mps;
  parameters.hysteresis_distance_m =
      *task.scout_urgency_hysteresis_distance_m;
  parameters.hysteresis_time_s = *task.scout_urgency_hysteresis_time_s;
  parameters.policy_version = *task.scout_policy_version;
  parameters.sensor_coverage_radius_m =
      *task.scout_sensor_coverage_radius_m;
  parameters.scout_corridor_half_width_m =
      *task.scout_corridor_half_width_m;
  parameters.communication_max_distance_m = *task.communication_max_m;
  parameters.desired_scout_distance_m = *task.scout_desired_distance_m;
  parameters.continue_scout_distance_m = *task.scout_continue_distance_m;
  parameters.stop_scout_distance_m = *task.scout_stop_distance_m;
  parameters.blocking_priority_weight =
      *task.scout_blocking_priority_weight;
  parameters.information_value_weight =
      *task.scout_information_value_weight;
  parameters.forward_progress_weight =
      *task.scout_forward_progress_weight;
  parameters.arrival_cost_weight = *task.scout_arrival_cost_weight;
  const long double request_timeout_nanoseconds =
      static_cast<long double>(*task.scout_request_timeout_s) * 1.0e9L;
  constexpr std::int64_t kDurationConversionMarginNanoseconds =
      1'000'000'000;
  const long double maximum_safe_nanoseconds = static_cast<long double>(
      std::numeric_limits<std::int64_t>::max() -
      kDurationConversionMarginNanoseconds);
  if (request_timeout_nanoseconds < 1.0L ||
      request_timeout_nanoseconds > maximum_safe_nanoseconds) {
    throw std::invalid_argument(
        "scout request timeout is not representable in nanoseconds");
  }
  parameters.request_timeout = Duration{
      static_cast<std::int64_t>(request_timeout_nanoseconds)};
  if (!valid(parameters)) {
    throw std::invalid_argument(
        "scout coordination profile violates the coordinator contract");
  }
  return parameters;
}

ScoutCoordinator::ScoutCoordinator(ScoutCoordinationParameters parameters)
    : parameters_(parameters) {}

InformationGapScanResult ScoutCoordinator::identify_gaps_result(
    const ReferenceLine& reference_line, const MapSnapshot& map,
    const double planning_horizon_m) const {
  return identify_gaps_result(reference_line, map, planning_horizon_m,
                              std::nullopt);
}

InformationGapScanResult ScoutCoordinator::identify_gaps_result(
    const ReferenceLine& reference_line, const MapSnapshot& map,
    const double planning_horizon_m,
    const std::optional<GeometricPath>& candidate_detour) const {
  InformationGapScanResult result;
  result.source_map_version = map.version;
  result.reference_line_version = reference_line.version;
  result.planning_horizon_m = planning_horizon_m;
  if (!valid(parameters_)) result.issues.emplace_back("scout parameters are invalid");
  const SnapshotValidation map_validation = validate(map);
  if (!map_validation.valid) {
    result.issues.insert(result.issues.end(), map_validation.issues.begin(),
                         map_validation.issues.end());
  }
  const SnapshotValidation reference_validation = validate(reference_line);
  if (!reference_validation.valid) {
    result.issues.insert(result.issues.end(), reference_validation.issues.begin(),
                         reference_validation.issues.end());
  }
  if (!finite(planning_horizon_m) || planning_horizon_m <= 0.0) {
    result.issues.emplace_back("planning horizon must be finite and positive");
  }
  if (!result.issues.empty()) return result;

  if (candidate_detour.has_value()) {
    const ValidationResult candidate_validation = validate(*candidate_detour);
    if (!candidate_validation.valid) {
      result.issues.insert(result.issues.end(), candidate_validation.issues.begin(),
                           candidate_validation.issues.end());
      return result;
    }
    if (!candidate_detour->metadata.coordinate_frame.empty() &&
        candidate_detour->metadata.coordinate_frame != reference_line.coordinate_frame) {
      result.issues.emplace_back("candidate detour coordinate frame mismatches reference line");
      return result;
    }
  }

  result.validity = ScoutGapScanValidity::valid;
  const double first_arc = reference_line.points.front().arc_length_m;
  const double last_arc = std::min(
      reference_line.points.back().arc_length_m, first_arc + planning_horizon_m);
  std::vector<ReferencePoint> samples;
  for (double arc = first_arc; arc < last_arc; arc += parameters_.sample_interval_m) {
    const auto point = reference_line.query(arc);
    if (point.has_value()) samples.push_back(*point);
  }
  if (samples.empty() || samples.back().arc_length_m < last_arc - kEpsilon) {
    const auto point = reference_line.query(last_arc);
    if (point.has_value()) samples.push_back(*point);
  }

  std::optional<InformationGap> current;
  for (const ReferencePoint& point : samples) {
    const auto cell_index = map_cell_for(map, point);
    InformationGap sample_gap;
    bool gap = false;
    if (!cell_index.has_value()) {
      sample_gap.reason = InformationGapReason::unknown;
      sample_gap.row = std::numeric_limits<std::size_t>::max();
      sample_gap.column = std::numeric_limits<std::size_t>::max();
      sample_gap.minimum_confidence = 0.0;
      gap = true;
    } else {
      const auto [row, column] = *cell_index;
      const MapCell& cell = map.at(row, column);
      if (!cell.known) {
        sample_gap.reason = InformationGapReason::unknown;
        gap = true;
      } else if (cell.confidence < parameters_.minimum_map_confidence) {
        sample_gap.reason = InformationGapReason::low_confidence;
        gap = true;
      }
      sample_gap.row = row;
      sample_gap.column = column;
      sample_gap.minimum_confidence = cell.confidence;
    }
    if (!gap) {
      if (current.has_value()) result.gaps.push_back(*current);
      current.reset();
      continue;
    }
    sample_gap.center = {point.x_m, point.y_m};
    sample_gap.start_progress_m = point.arc_length_m;
    sample_gap.end_progress_m = point.arc_length_m;
    sample_gap.source_map_version = map.version;
    sample_gap.reference_line_version = reference_line.version;
    if (!current.has_value()) {
      current = sample_gap;
    } else if (point.arc_length_m - current->end_progress_m <=
                   parameters_.merge_distance_m + parameters_.sample_interval_m + kEpsilon &&
               std::hypot(point.x_m - current->center.x_m,
                          point.y_m - current->center.y_m) <=
                          parameters_.merge_distance_m +
                              2.0 * parameters_.sample_interval_m + kEpsilon) {
      current->end_progress_m = point.arc_length_m;
      current->center.x_m = (current->center.x_m + point.x_m) * 0.5;
      current->center.y_m = (current->center.y_m + point.y_m) * 0.5;
      current->minimum_confidence =
          std::min(current->minimum_confidence, sample_gap.minimum_confidence);
      current->reason = more_conservative_reason(current->reason, sample_gap.reason);
    } else {
      result.gaps.push_back(*current);
      current = sample_gap;
    }
  }
  if (current.has_value()) result.gaps.push_back(*current);

  if (candidate_detour.has_value()) {
    const double reference_end = reference_line.points.back().arc_length_m;
    for (const PathPoint& path_point : candidate_detour->points) {
      const std::vector<ReferenceProjection> projections =
          reference_line.local_projection_candidates(
              Vector2m{path_point.x_m, path_point.y_m}, first_arc,
              reference_end);
      if (projections.empty()) continue;
      const auto nearest = std::min_element(
          projections.begin(), projections.end(),
          [](const ReferenceProjection& left, const ReferenceProjection& right) {
            return left.distance_m < right.distance_m;
          });
      const auto cell_index = map_cell_for(
          map, ReferencePoint{nearest->point.arc_length_m, path_point.x_m,
                              path_point.y_m, 0.0, 0.0, 0.0, 0.0});
      if (!cell_index.has_value()) {
        InformationGap gap;
        gap.row = std::numeric_limits<std::size_t>::max();
        gap.column = std::numeric_limits<std::size_t>::max();
        gap.center = {path_point.x_m, path_point.y_m};
        gap.reason = InformationGapReason::unknown;
        gap.minimum_confidence = 0.0;
        gap.start_progress_m = nearest->point.arc_length_m;
        gap.end_progress_m = nearest->point.arc_length_m;
        gap.source_map_version = map.version;
        gap.reference_line_version = reference_line.version;
        result.gaps.push_back(gap);
        continue;
      }
      const auto [row, column] = *cell_index;
      const MapCell& cell = map.at(row, column);
      if (cell.known && cell.confidence >= parameters_.minimum_map_confidence) {
        continue;
      }
      InformationGap gap;
      gap.row = row;
      gap.column = column;
      gap.center = {path_point.x_m, path_point.y_m};
      gap.reason = cell.known ? InformationGapReason::low_confidence
                              : InformationGapReason::unknown;
      gap.minimum_confidence = cell.confidence;
      gap.start_progress_m = nearest->point.arc_length_m;
      gap.end_progress_m = nearest->point.arc_length_m;
      gap.source_map_version = map.version;
      gap.reference_line_version = reference_line.version;
      result.gaps.push_back(gap);
    }
  }
  std::stable_sort(result.gaps.begin(), result.gaps.end(),
                   [](const InformationGap& left, const InformationGap& right) {
                     if (left.start_progress_m != right.start_progress_m)
                       return left.start_progress_m < right.start_progress_m;
                     if (left.end_progress_m != right.end_progress_m)
                       return left.end_progress_m < right.end_progress_m;
                     return static_cast<int>(left.reason) < static_cast<int>(right.reason);
                   });
  std::vector<InformationGap> merged;
  for (const InformationGap& gap : result.gaps) {
    if (!merged.empty() &&
        gap.start_progress_m - merged.back().end_progress_m <=
            parameters_.merge_distance_m + parameters_.sample_interval_m + kEpsilon &&
        std::hypot(gap.center.x_m - merged.back().center.x_m,
                   gap.center.y_m - merged.back().center.y_m) <=
            parameters_.merge_distance_m +
                2.0 * parameters_.sample_interval_m + kEpsilon) {
      InformationGap& previous = merged.back();
      previous.end_progress_m = std::max(previous.end_progress_m, gap.end_progress_m);
      previous.center.x_m = (previous.center.x_m + gap.center.x_m) * 0.5;
      previous.center.y_m = (previous.center.y_m + gap.center.y_m) * 0.5;
      previous.minimum_confidence =
          std::min(previous.minimum_confidence, gap.minimum_confidence);
      previous.reason = more_conservative_reason(previous.reason, gap.reason);
    } else {
      merged.push_back(gap);
    }
  }
  result.gaps = std::move(merged);
  return result;
}

std::vector<InformationGap> ScoutCoordinator::identify_gaps(
    const ReferenceLine& reference_line, const MapSnapshot& map,
    const double planning_horizon_m) const {
  return identify_gaps_result(reference_line, map, planning_horizon_m).gaps;
}

std::vector<InformationGap> ScoutCoordinator::identify_gaps(
    const ReferenceLine& reference_line, const MapSnapshot& map,
    const double planning_horizon_m,
    const std::optional<GeometricPath>& candidate_detour) const {
  return identify_gaps_result(reference_line, map, planning_horizon_m,
                              candidate_detour)
      .gaps;
}

GapUrgencyAssessment ScoutCoordinator::assess_gap_urgency(
    const InformationGap& gap, const RobotState& robot_state,
    const double current_reference_progress_m,
    const std::optional<TimedPath>& approved_remaining_path,
    const double planning_horizon_m) const {
  GapUrgencyAssessment result;
  result.distance_to_gap_m =
      std::max(0.0, gap.start_progress_m - current_reference_progress_m);
  result.blocks_planning_window =
      gap.start_progress_m <= current_reference_progress_m + planning_horizon_m + kEpsilon &&
      gap.end_progress_m + kEpsilon >= current_reference_progress_m;
  if (!valid(parameters_) || !finite(current_reference_progress_m) ||
      !finite(planning_horizon_m) || planning_horizon_m <= 0.0 ||
      !finite(gap.start_progress_m) || !finite(gap.end_progress_m) ||
      gap.end_progress_m < gap.start_progress_m ||
      validate(robot_state).valid == false) {
    result.urgency = GapUrgency::blocking;
    result.recommended_action = "STOP_AND_WAIT";
    result.used_conservative_fallback = true;
    return result;
  }

  double path_remaining = 0.0;
  std::optional<double> path_time;
  if (approved_remaining_path.has_value() &&
      validate(*approved_remaining_path).valid) {
    const PathPosition path_position =
        path_position_from_pose(*approved_remaining_path, robot_state.pose);
    path_remaining = path_position.remaining_m;
    const double path_distance = std::min(result.distance_to_gap_m, path_remaining);
    const double target_arc =
        path_position.nearest_arc_m + path_distance;
    const std::optional<double> current_time = time_at_arc(
        approved_remaining_path->execution_profile, path_position.nearest_arc_m);
    const std::optional<double> target_time = time_at_arc(
        approved_remaining_path->execution_profile, target_arc);
    if (current_time.has_value() && target_time.has_value()) {
      path_time = std::max(0.0, *target_time - *current_time);
    }
  } else {
    result.used_conservative_fallback = true;
  }
  result.safe_path_remaining_m = std::min(result.distance_to_gap_m, path_remaining);
  if (result.used_conservative_fallback) {
    result.urgency = GapUrgency::blocking;
    result.time_to_gap_s = std::numeric_limits<double>::infinity();
    result.recommended_action = "STOP_AND_WAIT";
    return result;
  }
  if (!path_time.has_value()) {
    result.time_to_gap_s = std::numeric_limits<double>::infinity();
  } else {
    result.time_to_gap_s = std::max(0.0, *path_time);
  }

  GapUrgency candidate = GapUrgency::informational;
  if (result.blocks_planning_window &&
      result.safe_path_remaining_m < parameters_.minimum_safe_distance_m) {
    candidate = GapUrgency::blocking;
  } else if (result.time_to_gap_s < parameters_.planning_lead_time_s) {
    candidate = GapUrgency::urgent;
  } else if (result.time_to_gap_s < planning_horizon_m / parameters_.average_velocity_mps) {
    candidate = GapUrgency::scheduled;
  }
  const GapKey key{
      gap.source_map_version.map_id,
      gap.source_map_version.sequence_number,
      gap.reference_line_version,
      quantized_progress(gap.start_progress_m).value_or(0),
      quantized_progress(gap.end_progress_m).value_or(0),
      quantized_progress(gap.center.x_m).value_or(0),
      quantized_progress(gap.center.y_m).value_or(0)};
  if (!quantized_progress(gap.start_progress_m).has_value() ||
      !quantized_progress(gap.end_progress_m).has_value() ||
      !quantized_progress(gap.center.x_m).has_value() ||
      !quantized_progress(gap.center.y_m).has_value()) {
    result.urgency = GapUrgency::blocking;
    result.recommended_action = "STOP_AND_WAIT";
    result.used_conservative_fallback = true;
    return result;
  }
  result.urgency = apply_hysteresis(key, candidate, result.time_to_gap_s,
                                    result.safe_path_remaining_m,
                                    result.blocks_planning_window,
                                    planning_horizon_m);
  switch (result.urgency) {
    case GapUrgency::blocking:
      result.recommended_action = "STOP_AND_WAIT";
      break;
    case GapUrgency::urgent:
      result.recommended_action = "REQUEST_VALIDATED_REDUCED_SPEED_PROFILE";
      break;
    case GapUrgency::scheduled:
      result.recommended_action = "REQUEST_SCOUT_BACKGROUND";
      break;
    case GapUrgency::informational:
      result.recommended_action = "RECORD_ONLY";
      break;
  }
  return result;
}

std::vector<PrioritizedGapAssessment> ScoutCoordinator::assess_gaps(
    const std::vector<InformationGap>& gaps, const RobotState& robot_state,
    const double current_reference_progress_m,
    const std::optional<TimedPath>& approved_remaining_path,
    const double planning_horizon_m) const {
  std::vector<PrioritizedGapAssessment> result;
  result.reserve(gaps.size());
  for (const InformationGap& gap : gaps) {
    result.push_back({gap, assess_gap_urgency(
                              gap, robot_state, current_reference_progress_m,
                              approved_remaining_path, planning_horizon_m)});
  }
  std::stable_sort(result.begin(), result.end(),
                   [](const PrioritizedGapAssessment& left,
                      const PrioritizedGapAssessment& right) {
                     const int left_rank = urgency_rank(left.assessment.urgency);
                     const int right_rank = urgency_rank(right.assessment.urgency);
                     if (left_rank != right_rank) return left_rank < right_rank;
                     if (left.assessment.distance_to_gap_m !=
                         right.assessment.distance_to_gap_m) {
                       return left.assessment.distance_to_gap_m <
                              right.assessment.distance_to_gap_m;
                     }
                     return left.gap.start_progress_m < right.gap.start_progress_m;
                   });
  return result;
}

ScoutTargetGenerationResult ScoutCoordinator::generate_scout_target(
    const InformationGap& gap, const GapUrgencyAssessment& assessment,
    const ReferenceLine& reference_line, const Pose2d& main_robot_pose,
    const Pose2d& scout_robot_pose,
    const double current_reference_progress_m) const {
  ScoutTargetGenerationResult result;
  if (!valid(parameters_)) result.issues.emplace_back("scout parameters are invalid");
  const SnapshotValidation reference_validation = validate(reference_line);
  if (!reference_validation.valid) {
    result.issues.insert(result.issues.end(), reference_validation.issues.begin(),
                         reference_validation.issues.end());
  }
  if (!finite_pose(main_robot_pose) || !finite_pose(scout_robot_pose)) {
    result.issues.emplace_back("main and scout poses must be finite and timestamped");
  }
  if (!finite(current_reference_progress_m)) {
    result.issues.emplace_back("current reference progress must be finite");
  }
  if (!finite(gap.center.x_m) || !finite(gap.center.y_m) ||
      !finite(gap.start_progress_m) || !finite(gap.end_progress_m) ||
      gap.end_progress_m < gap.start_progress_m ||
      gap.end_progress_m + kEpsilon < current_reference_progress_m ||
      !finite(gap.minimum_confidence) || gap.minimum_confidence < 0.0 ||
      gap.minimum_confidence > 1.0 || gap.source_map_version.map_id.empty() ||
      gap.source_map_version.sequence_number == 0U ||
      gap.source_map_version.coordinate_frame != reference_line.coordinate_frame ||
      gap.reference_line_version != reference_line.version) {
    result.issues.emplace_back(
        "gap geometry, confidence, map version, or reference version is invalid");
  }
  if (!result.issues.empty()) return result;

  const double target_progress_m =
      gap.start_progress_m + (gap.end_progress_m - gap.start_progress_m) * 0.5;
  const std::optional<ReferencePoint> reference_point =
      reference_line.query(target_progress_m);
  if (!reference_point.has_value()) {
    result.issues.emplace_back("gap progress is outside the reference line");
    return result;
  }
  const double main_distance_m =
      std::hypot(gap.center.x_m - main_robot_pose.x_m,
                 gap.center.y_m - main_robot_pose.y_m);
  if (main_distance_m > parameters_.communication_max_distance_m + kEpsilon) {
    result.validity = ScoutTargetValidity::distance_constraint_violated;
    result.issues.emplace_back(
        "scout target exceeds the main-to-scout communication hard limit");
    return result;
  }
  const double reference_offset_m =
      std::hypot(gap.center.x_m - reference_point->x_m,
                 gap.center.y_m - reference_point->y_m);
  if (reference_offset_m > parameters_.scout_corridor_half_width_m + kEpsilon) {
    result.issues.emplace_back("scout target is outside the configured scout corridor");
    return result;
  }

  ScoutTarget target;
  const MonotonicTime target_timestamp{
      std::max(main_robot_pose.timestamp.nanoseconds,
               scout_robot_pose.timestamp.nanoseconds)};
  target.target_pose = {
      gap.center.x_m, gap.center.y_m,
      std::atan2(reference_point->tangent_y, reference_point->tangent_x),
      target_timestamp};
  target.gap_start_progress_m = gap.start_progress_m;
  target.gap_end_progress_m = gap.end_progress_m;
  const double gap_length_m =
      std::max(parameters_.sample_interval_m,
               gap.end_progress_m - gap.start_progress_m);
  target.coverage_fraction =
      std::min(1.0, 2.0 * parameters_.sensor_coverage_radius_m / gap_length_m);
  target.information_value =
      (1.0 - gap.minimum_confidence) * gap_length_m * target.coverage_fraction;
  target.forward_progress_m =
      std::max(0.0, target_progress_m - current_reference_progress_m);
  target.estimated_arrival_cost_m =
      std::hypot(gap.center.x_m - scout_robot_pose.x_m,
                 gap.center.y_m - scout_robot_pose.y_m);
  const double blocking_value =
      assessment.urgency == GapUrgency::blocking ? 1.0 : 0.0;
  target.priority =
      parameters_.blocking_priority_weight * blocking_value +
      parameters_.information_value_weight * target.information_value +
      parameters_.forward_progress_weight /
          (1.0 + target.forward_progress_m) -
      parameters_.arrival_cost_weight * target.estimated_arrival_cost_m;
  target.scout_corridor_half_width_m = parameters_.scout_corridor_half_width_m;
  target.urgency = assessment.urgency;
  target.source_map_version = gap.source_map_version;
  target.reference_line_version = gap.reference_line_version;
  target.policy_version = parameters_.policy_version;
  target.parameter_profile_id = parameters_.parameter_profile_id;
  target.operating_domain_id = parameters_.operating_domain_id;
  result.validity = ScoutTargetValidity::valid;
  result.target = target;
  return result;
}

std::vector<ScoutTarget> ScoutCoordinator::generate_scout_targets(
    const std::vector<PrioritizedGapAssessment>& gaps,
    const ReferenceLine& reference_line, const Pose2d& main_robot_pose,
    const Pose2d& scout_robot_pose,
    const double current_reference_progress_m) const {
  std::vector<ScoutTarget> result;
  result.reserve(gaps.size());
  for (const PrioritizedGapAssessment& gap : gaps) {
    ScoutTargetGenerationResult generated = generate_scout_target(
        gap.gap, gap.assessment, reference_line, main_robot_pose,
        scout_robot_pose, current_reference_progress_m);
    if (generated.validity == ScoutTargetValidity::valid &&
        generated.target.has_value()) {
      result.push_back(*generated.target);
    }
  }
  std::stable_sort(result.begin(), result.end(),
                   [](const ScoutTarget& left, const ScoutTarget& right) {
                     const int left_urgency = urgency_rank(left.urgency);
                     const int right_urgency = urgency_rank(right.urgency);
                     if (left_urgency != right_urgency) {
                       return left_urgency < right_urgency;
                     }
                     if (left.priority != right.priority) {
                       return left.priority > right.priority;
                     }
                     if (left.gap_start_progress_m != right.gap_start_progress_m) {
                       return left.gap_start_progress_m < right.gap_start_progress_m;
                     }
                     return left.source_map_version.sequence_number <
                            right.source_map_version.sequence_number;
                   });
  return result;
}

ScoutDistanceAssessment ScoutCoordinator::assess_distance_constraint(
    const Pose2d& main_robot_pose, const Pose2d& scout_robot_pose) const {
  ScoutDistanceAssessment result;
  result.desired_distance_m = parameters_.desired_scout_distance_m;
  result.communication_max_distance_m =
      parameters_.communication_max_distance_m;
  if (!valid(parameters_) || !finite_pose(main_robot_pose) ||
      !finite_pose(scout_robot_pose)) {
    result.issues.emplace_back(
        "scout distance policy parameters and both poses must be valid");
    distance_hold_active_ = true;
    result.hysteresis_hold_active = true;
    return result;
  }

  result.separation_m =
      std::hypot(scout_robot_pose.x_m - main_robot_pose.x_m,
                 scout_robot_pose.y_m - main_robot_pose.y_m);
  if (result.separation_m >
      parameters_.communication_max_distance_m + kEpsilon) {
    distance_hold_active_ = true;
    result.directive = ScoutDistanceDirective::recover_communication;
    result.hysteresis_hold_active = true;
    result.recommended_main_action =
        ScoutMainAction::stop_and_recover_communication;
    result.issues.emplace_back("main-to-scout communication hard limit breached");
    return result;
  }

  result.communication_satisfied = true;
  result.main_robot_degraded = false;
  result.recommended_main_action = ScoutMainAction::continue_approved_plan;
  if (distance_hold_active_) {
    if (result.separation_m <
        parameters_.continue_scout_distance_m - kEpsilon) {
      distance_hold_active_ = false;
      result.directive = ScoutDistanceDirective::advance;
    } else {
      result.directive = ScoutDistanceDirective::hold_position;
    }
  } else if (result.separation_m >
             parameters_.stop_scout_distance_m + kEpsilon) {
    distance_hold_active_ = true;
    result.directive = ScoutDistanceDirective::hold_position;
  } else if (result.separation_m + kEpsilon <
             parameters_.desired_scout_distance_m) {
    result.directive = ScoutDistanceDirective::advance;
  } else {
    result.directive = ScoutDistanceDirective::maintain_desired_spacing;
  }
  result.hysteresis_hold_active = distance_hold_active_;
  return result;
}

ScoutRequestIssueResult ScoutCoordinator::issue_scout_request(
    const ScoutTarget& target, const MonotonicTime now) {
  ScoutRequestIssueResult result;
  const std::optional<GapKey> key = gap_key_for(target);
  if (!valid(parameters_)) result.issues.emplace_back("scout parameters are invalid");
  if (!key.has_value() || !finite_pose(target.target_pose) ||
      !finite(target.coverage_fraction) || target.coverage_fraction <= 0.0 ||
      target.coverage_fraction > 1.0 || !finite(target.priority) ||
      target.policy_version != parameters_.policy_version ||
      target.parameter_profile_id != parameters_.parameter_profile_id ||
      target.operating_domain_id != parameters_.operating_domain_id ||
      target.source_map_version.map_id.empty() ||
      target.source_map_version.sequence_number == 0U ||
      target.reference_line_version == 0U || now.nanoseconds < 0 ||
      target.source_map_version.timestamp.nanoseconds > now.nanoseconds ||
      target.urgency == GapUrgency::informational) {
    result.issues.emplace_back(
        "scout target, source versions, policy version, or request time is invalid");
  }
  if (now.nanoseconds >
      std::numeric_limits<std::int64_t>::max() -
          parameters_.request_timeout.nanoseconds) {
    result.issues.emplace_back("scout request expiry is not representable");
  }
  if (!result.issues.empty()) return result;

  const ScoutRequestExpiryResult expiry = expire_scout_requests(now);
  if (!expiry.valid) {
    result.issues.insert(result.issues.end(), expiry.issues.begin(),
                         expiry.issues.end());
    return result;
  }

  const auto active = active_request_by_gap_.find(*key);
  if (active != active_request_by_gap_.end()) {
    const auto existing = requests_.find(active->second);
    if (existing != requests_.end() &&
        existing->second.status == ScoutRequestStatus::awaiting_map_update) {
      result.disposition = ScoutRequestIssueDisposition::deduplicated;
      result.request = existing->second;
      return result;
    }
    active_request_by_gap_.erase(active);
  }

  ScoutRequest request;
  request.request_sequence = next_request_sequence_++;
  request.revision = 1U;
  request.policy_version = parameters_.policy_version;
  request.status = ScoutRequestStatus::awaiting_map_update;
  request.target = target;
  request.requested_at = now;
  request.expires_at = {
      now.nanoseconds + parameters_.request_timeout.nanoseconds};
  if (target.urgency == GapUrgency::blocking) {
    request.recommended_main_action = ScoutMainAction::stop_and_wait_for_map;
    request.planning_directive = ScoutPlanningDirective::waiting_map;
  } else if (target.urgency == GapUrgency::urgent) {
    request.recommended_main_action =
        ScoutMainAction::request_validated_reduced_speed_profile;
  } else {
    request.recommended_main_action = ScoutMainAction::continue_approved_plan;
  }
  requests_.emplace(request.request_sequence, request);
  active_request_by_gap_.emplace(*key, request.request_sequence);
  result.disposition = ScoutRequestIssueDisposition::issued;
  result.request = request;
  return result;
}

ScoutMapUpdateResult ScoutCoordinator::correlate_map_update(
    const std::uint64_t request_sequence,
    const ReferenceLine& reference_line, const MapSnapshot& updated_map,
    const MonotonicTime now) {
  ScoutMapUpdateResult result;
  const auto found = requests_.find(request_sequence);
  if (found == requests_.end()) {
    result.disposition = ScoutMapUpdateDisposition::request_not_found;
    result.issues.emplace_back("scout request sequence was not found");
    return result;
  }
  ScoutRequest& request = found->second;
  result.request = request;
  if (request.status != ScoutRequestStatus::awaiting_map_update) {
    result.disposition = ScoutMapUpdateDisposition::request_terminal;
    result.issues.emplace_back("terminal scout request cannot accept a map update");
    return result;
  }
  if (now.nanoseconds < 0 || now.nanoseconds >= request.expires_at.nanoseconds) {
    mark_request_timed_out(request);
    result.disposition = ScoutMapUpdateDisposition::timed_out;
    result.request = request;
    result.invalidate_old_plan = true;
    result.issues.emplace_back(
        "map update arrived at or after the scout request timeout");
    return result;
  }
  const SnapshotValidation map_validation = validate(updated_map);
  const SnapshotValidation reference_validation = validate(reference_line);
  if (!map_validation.valid) {
    result.issues.insert(result.issues.end(), map_validation.issues.begin(),
                         map_validation.issues.end());
  }
  if (!reference_validation.valid) {
    result.issues.insert(result.issues.end(), reference_validation.issues.begin(),
                         reference_validation.issues.end());
  }
  const MapVersion& source = request.target.source_map_version;
  const std::uint64_t last_sequence =
      request.last_associated_map_version.has_value()
          ? request.last_associated_map_version->sequence_number
          : source.sequence_number;
  if (updated_map.version.map_id != source.map_id ||
      updated_map.version.coordinate_frame != source.coordinate_frame ||
      updated_map.version.sequence_number <= last_sequence ||
      updated_map.version.timestamp.nanoseconds < request.requested_at.nanoseconds ||
      updated_map.version.timestamp.nanoseconds > now.nanoseconds ||
      reference_line.version != request.target.reference_line_version ||
      reference_line.coordinate_frame != source.coordinate_frame) {
    result.issues.emplace_back(
        "map update is stale or mismatches the request map/reference versions");
  }
  if (!result.issues.empty()) return result;

  bool resolved = true;
  const auto sample_is_resolved = [&](const double progress_m) {
    const std::optional<ReferencePoint> point = reference_line.query(progress_m);
    if (!point.has_value()) return false;
    const std::optional<std::pair<std::size_t, std::size_t>> cell_index =
        map_cell_for(updated_map, *point);
    if (!cell_index.has_value()) return false;
    const auto [row, column] = *cell_index;
    const MapCell& cell = updated_map.at(row, column);
    return cell.known && cell.confidence >= parameters_.minimum_map_confidence;
  };
  for (double progress = request.target.gap_start_progress_m;
       progress < request.target.gap_end_progress_m - kEpsilon;
       progress += parameters_.sample_interval_m) {
    if (!sample_is_resolved(progress)) {
      resolved = false;
      break;
    }
  }
  if (resolved &&
      !sample_is_resolved(request.target.gap_end_progress_m)) {
    resolved = false;
  }
  if (resolved) {
    const ReferencePoint target_point{
        request.target.gap_start_progress_m,
        request.target.target_pose.x_m,
        request.target.target_pose.y_m,
        0.0,
        0.0,
        0.0,
        0.0};
    const auto target_cell = map_cell_for(updated_map, target_point);
    if (!target_cell.has_value()) {
      resolved = false;
    } else {
      const auto [row, column] = *target_cell;
      const MapCell& cell = updated_map.at(row, column);
      resolved = cell.known &&
                 cell.confidence >= parameters_.minimum_map_confidence;
    }
  }

  request.last_associated_map_version = updated_map.version;
  ++request.revision;
  if (!resolved) {
    result.disposition = ScoutMapUpdateDisposition::associated_unresolved;
    result.request = request;
    return result;
  }

  request.status = ScoutRequestStatus::completed;
  request.completed_map_version = updated_map.version;
  request.recommended_main_action =
      ScoutMainAction::trigger_replan_with_new_map;
  request.planning_directive = ScoutPlanningDirective::replan_with_new_map;
  if (const std::optional<GapKey> key = gap_key_for(request.target);
      key.has_value()) {
    active_request_by_gap_.erase(*key);
  }
  result.disposition = ScoutMapUpdateDisposition::completed;
  result.request = request;
  result.invalidate_old_plan = true;
  result.trigger_replanning = true;
  return result;
}

ScoutRequestExpiryResult ScoutCoordinator::expire_scout_requests(
    const MonotonicTime now) {
  ScoutRequestExpiryResult result;
  if (now.nanoseconds < 0) {
    result.issues.emplace_back("expiry observation time must be monotonic");
    return result;
  }
  result.valid = true;
  for (auto& [sequence, request] : requests_) {
    static_cast<void>(sequence);
    if (request.status != ScoutRequestStatus::awaiting_map_update ||
        now.nanoseconds < request.expires_at.nanoseconds) {
      continue;
    }
    mark_request_timed_out(request);
    result.expired.push_back(request);
  }
  return result;
}

void ScoutCoordinator::mark_request_timed_out(ScoutRequest& request) {
  request.status = ScoutRequestStatus::timed_out;
  ++request.revision;
  request.recommended_main_action = ScoutMainAction::stop_and_wait_for_map;
  request.planning_directive = ScoutPlanningDirective::waiting_map;
  if (const std::optional<GapKey> key = gap_key_for(request.target);
      key.has_value()) {
    active_request_by_gap_.erase(*key);
  }
}

std::optional<ScoutRequest> ScoutCoordinator::scout_request(
    const std::uint64_t request_sequence) const {
  const auto found = requests_.find(request_sequence);
  if (found == requests_.end()) return std::nullopt;
  return found->second;
}

void ScoutCoordinator::clear_hysteresis() const {
  previous_urgencies_.clear();
  distance_hold_active_ = false;
}

bool operator<(const ScoutCoordinator::GapKey& left,
               const ScoutCoordinator::GapKey& right) noexcept {
  return std::tie(left.map_id, left.map_sequence, left.reference_version,
                  left.start_micrometers, left.end_micrometers,
                  left.target_x_micrometers, left.target_y_micrometers) <
         std::tie(right.map_id, right.map_sequence, right.reference_version,
                  right.start_micrometers, right.end_micrometers,
                  right.target_x_micrometers, right.target_y_micrometers);
}

GapUrgency ScoutCoordinator::apply_hysteresis(
    const GapKey& key, const GapUrgency candidate, const double time_to_gap_s,
    const double safe_path_remaining_m, const bool blocks_window,
    const double planning_horizon_m) const {
  const auto previous = previous_urgencies_.find(key);
  if (previous == previous_urgencies_.end()) {
    previous_urgencies_.emplace(key, candidate);
    return candidate;
  }
  GapUrgency result = previous->second;
  if (candidate != result) {
    const int candidate_rank = urgency_rank(candidate);
    const int previous_rank = urgency_rank(result);
    bool crossed_hysteresis = true;
    if (candidate_rank < previous_rank) {
      if (candidate == GapUrgency::blocking) {
        crossed_hysteresis = blocks_window &&
                             safe_path_remaining_m <
                                 parameters_.minimum_safe_distance_m -
                                     parameters_.hysteresis_distance_m;
      } else if (candidate == GapUrgency::urgent) {
        crossed_hysteresis = time_to_gap_s <
                             parameters_.planning_lead_time_s -
                                 parameters_.hysteresis_time_s;
      } else if (candidate == GapUrgency::scheduled) {
        crossed_hysteresis = time_to_gap_s <
                             planning_horizon_m /
                                     parameters_.average_velocity_mps -
                                 parameters_.hysteresis_time_s;
      }
    } else {
      if (result == GapUrgency::blocking) {
        crossed_hysteresis = !blocks_window ||
                             safe_path_remaining_m >
                                 parameters_.minimum_safe_distance_m +
                                     parameters_.hysteresis_distance_m;
      } else if (result == GapUrgency::urgent) {
        crossed_hysteresis = time_to_gap_s >
                             parameters_.planning_lead_time_s +
                                 parameters_.hysteresis_time_s;
      } else if (result == GapUrgency::scheduled) {
        crossed_hysteresis = time_to_gap_s >
                             planning_horizon_m /
                                     parameters_.average_velocity_mps +
                                 parameters_.hysteresis_time_s;
      }
    }
    if (crossed_hysteresis) result = candidate;
  }
  previous->second = result;
  return result;
}

}  // namespace underwater_planner::core
