#include "scout_planner/core/continuous_geometry_validator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace scout_planner::core {
namespace {

constexpr double kEpsilon = 1.0e-12;
constexpr std::uint64_t kMaximumCheckedIntervals = 1'000'000U;

bool finite(const Point3dEnu point) noexcept {
  return std::isfinite(point.x_m) && std::isfinite(point.y_m) &&
         std::isfinite(point.z_m);
}

bool finite(const double value) noexcept { return std::isfinite(value); }

bool valid_config(const ContinuousGeometryValidationConfig& config) noexcept {
  const std::array margins{config.safety_margins.body_m,
                           config.safety_margins.localization_m,
                           config.safety_margins.tracking_m,
                           config.safety_margins.map_m,
                           config.safety_margins.discretization_m};
  if (!std::all_of(margins.begin(), margins.end(),
                   [](const double value) { return finite(value) && value >= 0.0; }) ||
      !finite(config.safety_margins.total_m()) ||
      !finite(config.minimum_interval_s) || config.minimum_interval_s <= 0.0 ||
      config.maximum_refinement_depth > 100U) {
    return false;
  }
  const std::array limits{config.maximum_speed_mps,
                          config.maximum_acceleration_mps2,
                          config.maximum_yaw_rate_rps,
                          config.maximum_yaw_acceleration_rps2};
  return std::all_of(limits.begin(), limits.end(), [](const double value) {
    return finite(value) && value >= 0.0;
  });
}

void record_margin(ContinuousGeometryValidationReport& report,
                   const double margin) {
  if (!finite(margin)) return;
  if (!report.minimum_collision_margin_m.has_value() ||
      margin < report.minimum_collision_margin_m.value()) {
    report.minimum_collision_margin_m = margin;
  }
}

void record_failure(ContinuousGeometryValidationReport& report,
                    const std::uint64_t time_ns, const std::string& detail) {
  if (!report.earliest_failure_time_offset_ns.has_value() ||
      time_ns < report.earliest_failure_time_offset_ns.value()) {
    report.earliest_failure_time_offset_ns = time_ns;
  }
  report.diagnostics.push_back(detail);
}

struct Checker final {
  const HybridMapQuery& map;
  const QuinticBezierSegment& segment;
  const ContinuousGeometryValidationConfig& config;
  ContinuousGeometryValidationReport& report;
  double acceleration_bound{};
  bool inconclusive{};
  bool rejected{};

  void check(const double first, const double last, const std::uint32_t depth) {
    if (rejected) return;
    if (report.checked_interval_count >= kMaximumCheckedIntervals) {
      inconclusive = true;
      report.diagnostics.push_back("geometry refinement interval budget exhausted");
      return;
    }
    report.refinement_depth = std::max(report.refinement_depth, depth);
    ++report.checked_interval_count;
    const auto interval_failure_time = [&] (const double normalized) {
      const auto scaled = normalized * static_cast<long double>(segment.value().duration_ns);
      return segment.value().start_time_offset_ns +
             static_cast<std::uint64_t>(std::max<long double>(0.0L, scaled));
    };

    const auto left = segment.evaluate_normalized(first);
    const auto right = segment.evaluate_normalized(last);
    if (!left.has_value() || !right.has_value()) {
      rejected = true;
      record_failure(report, interval_failure_time(first),
                     "trajectory evaluation failed during geometry validation");
      return;
    }
    const auto left_query = map.query_point(left.value().position,
                                            config.safety_margins);
    const auto right_query = map.query_point(right.value().position,
                                             config.safety_margins);
    if (!left_query.has_value() || !right_query.has_value()) {
      rejected = true;
      record_failure(report, interval_failure_time(first),
                     "map query failed during geometry validation");
      return;
    }
    const auto endpoint_bad = [](const MapQuerySample& value) {
      return value.state == MapCellState::occupied || !value.allowed_water ||
             !finite(value.clearance_margin_m) ||
             !value.semantic_restrictions.empty();
    };
    if (endpoint_bad(left_query.value()) || endpoint_bad(right_query.value()) ||
        left_query.value().clearance_margin_m < 0.0 ||
        right_query.value().clearance_margin_m < 0.0) {
      rejected = true;
      record_failure(report,
                     endpoint_bad(left_query.value())
                         ? interval_failure_time(first)
                         : interval_failure_time(last),
                     "trajectory endpoint is occupied or outside allowed water");
      return;
    }
    record_margin(report, left_query.value().clearance_margin_m);
    record_margin(report, right_query.value().clearance_margin_m);

    const auto chord = map.query_supercover(left.value().position,
                                            right.value().position,
                                            config.safety_margins);
    if (!chord.has_value()) {
      rejected = true;
      record_failure(report, interval_failure_time(first),
                     "map supercover query failed during geometry validation");
      return;
    }
    const auto& sample = chord.value();
    record_margin(report, sample.clearance_margin_m);
    const auto duration_s = static_cast<double>(segment.value().duration_ns) /
                            1.0e9 * (last - first);
    const auto curve_deviation = duration_s * duration_s / 8.0 * acceleration_bound;
    const bool chord_safe = sample.state == MapCellState::free &&
                            sample.allowed_water &&
                            sample.semantic_restrictions.empty() &&
                            finite(sample.clearance_margin_m) &&
                            sample.clearance_margin_m >= 0.0;
    if (chord_safe && sample.clearance_margin_m > curve_deviation + kEpsilon) {
      return;
    }

    const auto leaf = duration_s <= config.minimum_interval_s + kEpsilon ||
                      depth >= config.maximum_refinement_depth;
    if (leaf) {
      const auto midpoint = (first + last) * 0.5;
      const auto value = segment.evaluate_normalized(midpoint);
      if (!value.has_value()) {
        rejected = true;
      record_failure(report, interval_failure_time(first),
                       "trajectory midpoint evaluation failed");
        return;
      }
      const auto point = map.query_point(value.value().position,
                                         config.safety_margins);
      if (!point.has_value()) {
        rejected = true;
        record_failure(report, interval_failure_time(midpoint),
                       "map midpoint query failed");
        return;
      }
      record_margin(report, point.value().clearance_margin_m);
      const auto failure_time = interval_failure_time(midpoint);
      if (point.value().state == MapCellState::occupied ||
          !point.value().allowed_water || point.value().clearance_margin_m < 0.0) {
        rejected = true;
        record_failure(report, failure_time,
                       "continuous trajectory intersects occupied or restricted water");
      } else {
        inconclusive = true;
        report.diagnostics.push_back(
            "minimum refinement interval cannot prove continuous clearance");
      }
      return;
    }
    check(first, (first + last) * 0.5, depth + 1U);
    check((first + last) * 0.5, last, depth + 1U);
  }
};

}  // namespace

ContinuousGeometryValidationReport ContinuousGeometryValidator::validate(
    const HybridMapQuery& map, const BezierTrajectory4d& trajectory,
    const ContinuousGeometryValidationConfig& config) {
  ContinuousGeometryValidationReport report{};
  report.validated_trajectory_content_identity = trajectory.content_hash();
  report.discrete_margin_m = config.safety_margins.discretization_m;
  if (!valid_config(config) || trajectory.frame_id() != "mission_enu" ||
      trajectory.segments().empty() || trajectory.duration_ns() == 0U) {
    report.status = ContinuousGeometryValidationStatus::invalid_input;
    report.primary_outcome = ContinuousGeometryValidationOutcome::input_invalid;
    report.diagnostics.push_back("trajectory or validator configuration is invalid");
    return report;
  }

  bool map_identity_set = false;
  for (std::size_t index = 0U; index < trajectory.segments().size(); ++index) {
    const auto segment_result = QuinticBezierSegment::create(trajectory.segments()[index]);
    if (!segment_result.has_value()) {
      report.status = ContinuousGeometryValidationStatus::invalid_input;
      report.primary_outcome = ContinuousGeometryValidationOutcome::input_invalid;
      report.diagnostics.push_back(segment_result.error().detail);
      return report;
    }
    const auto& segment = segment_result.value();
    const auto bounds = segment.derivative_bounds();
    const std::array checks{std::pair{bounds.maximum_speed_mps,
                                      config.maximum_speed_mps},
                            std::pair{bounds.maximum_acceleration_mps2,
                                      config.maximum_acceleration_mps2},
                            std::pair{bounds.maximum_yaw_rate_rps,
                                      config.maximum_yaw_rate_rps},
                            std::pair{bounds.maximum_yaw_acceleration_rps2,
                                      config.maximum_yaw_acceleration_rps2}};
    const std::array names{"speed", "acceleration", "yaw rate", "yaw acceleration"};
    for (std::size_t limit_index = 0U; limit_index < checks.size(); ++limit_index) {
      if (!finite(checks[limit_index].first) ||
          (checks[limit_index].second > 0.0 &&
           checks[limit_index].first > checks[limit_index].second + kEpsilon)) {
        report.status = ContinuousGeometryValidationStatus::unsafe;
        report.primary_outcome =
            ContinuousGeometryValidationOutcome::validation_rejected;
        record_failure(report, segment.value().start_time_offset_ns,
                       std::string("analytic ") + names[limit_index] +
                           " derivative bound exceeds configured limit");
        return report;
      }
    }
    // Fast path: verify the control polygon's consecutive edges and require
    // enough clearance for the full-segment acceleration deviation bound.
    bool control_polygon_safe = true;
    double control_polygon_margin = std::numeric_limits<double>::infinity();
    for (std::size_t control = 0U; control < 6U; ++control) {
      const auto point = map.query_point(segment.value().position_control_points[control],
                                         config.safety_margins);
      if (!point.has_value() || point.value().state != MapCellState::free ||
          !point.value().allowed_water || !finite(point.value().clearance_margin_m)) {
        control_polygon_safe = false;
        break;
      }
      control_polygon_margin = std::min(control_polygon_margin,
                                        point.value().clearance_margin_m);
      record_margin(report, point.value().clearance_margin_m);
      if (control + 1U < 6U) {
        const auto edge = map.query_supercover(
            segment.value().position_control_points[control],
            segment.value().position_control_points[control + 1U],
            config.safety_margins);
        if (!edge.has_value() || edge.value().state != MapCellState::free ||
            !edge.value().allowed_water || edge.value().clearance_margin_m < 0.0 ||
            !edge.value().semantic_restrictions.empty()) {
          control_polygon_safe = false;
          break;
        }
        control_polygon_margin = std::min(control_polygon_margin,
                                          edge.value().clearance_margin_m);
      }
    }
    const auto full_deviation = bounds.maximum_acceleration_mps2 *
        std::pow(static_cast<double>(segment.value().duration_ns) / 1.0e9, 2.0) / 8.0;
    if (!control_polygon_safe || !(control_polygon_margin > full_deviation + kEpsilon)) {
      Checker checker{map, segment, config, report, bounds.maximum_acceleration_mps2};
      checker.check(0.0, 1.0, 0U);
      if (checker.rejected) {
        report.status = ContinuousGeometryValidationStatus::unsafe;
        report.primary_outcome =
            ContinuousGeometryValidationOutcome::validation_rejected;
        return report;
      }
      if (checker.inconclusive) report.status = ContinuousGeometryValidationStatus::inconclusive;
    }
    if (!map_identity_set) {
      const auto sample = map.query_point(segment.evaluate_normalized(0.0).value().position,
                                          config.safety_margins);
      if (sample.has_value()) {
        report.validated_map_id = sample.value().source.map_id;
        report.validated_map_version = sample.value().source.map_version;
        report.validated_map_content_identity = sample.value().source.content_identity;
        map_identity_set = true;
      }
    }
  }
  if (report.status == ContinuousGeometryValidationStatus::inconclusive) {
    report.primary_outcome =
        ContinuousGeometryValidationOutcome::validation_inconclusive;
  } else {
    report.status = ContinuousGeometryValidationStatus::safe;
    report.primary_outcome = ContinuousGeometryValidationOutcome::success;
  }
  return report;
}

}  // namespace scout_planner::core
