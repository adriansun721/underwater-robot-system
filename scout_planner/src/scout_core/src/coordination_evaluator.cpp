#include "scout_planner/core/coordination_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace scout_planner::core {
namespace {

const CoreField* find_field(const CoreMessage& message, std::uint32_t number) {
  const auto found = std::find_if(
      message.fields.begin(), message.fields.end(),
      [number](const CoreField& field) { return field.number == number; });
  return found == message.fields.end() ? nullptr : &*found;
}

template <typename T>
const T& required(const CoreMessage& message, std::uint32_t number) {
  const auto* field = find_field(message, number);
  if (field == nullptr || field->values.size() != 1U) {
    throw std::invalid_argument{"required coordination field is malformed: " +
                               std::to_string(number) + " in " + message.schema_name};
  }
  return std::get<T>(field->values.front());
}

template <typename T>
std::optional<T> optional(const CoreMessage& message, std::uint32_t number) {
  const auto* field = find_field(message, number);
  if (field == nullptr) return std::nullopt;
  if (field->values.size() != 1U) {
    throw std::invalid_argument{"optional coordination field is malformed"};
  }
  return std::get<T>(field->values.front());
}

const CoreMessage& child(const CoreMessage& message, std::uint32_t number) {
  const auto* field = find_field(message, number);
  if (field == nullptr || field->values.size() != 1U)
    throw std::invalid_argument{"required coordination field is malformed: " +
                               std::to_string(number) + " in " + message.schema_name};
  const auto& value = std::get<CoreMessagePtr>(field->values.front());
  if (value == nullptr) throw std::invalid_argument{"null coordination child"};
  return *value;
}

std::vector<const CoreMessage*> repeated_children(const CoreMessage& message,
                                                  std::uint32_t number) {
  const auto* field = find_field(message, number);
  if (field == nullptr) return {};
  std::vector<const CoreMessage*> result;
  result.reserve(field->values.size());
  for (const auto& atom : field->values) {
    const auto& value = std::get<CoreMessagePtr>(atom);
    if (value == nullptr) throw std::invalid_argument{"null repeated child"};
    result.push_back(value.get());
  }
  return result;
}

std::string text(const CoreMessage& message, std::uint32_t number) {
  return required<TextValue>(message, number).value;
}

std::uint64_t u64(const CoreMessage& message, std::uint32_t number) {
  return required<std::uint64_t>(message, number);
}

double real(const CoreMessage& message, std::uint32_t number) {
  return required<double>(message, number);
}

Point3dEnu point(const CoreMessage& message) {
  return {real(message, 1U), real(message, 2U), real(message, 3U)};
}

bool same_document(const CoreMessage& left, const CoreMessage& right) {
  if (left.schema_name != right.schema_name || left.fields.size() != right.fields.size())
    return false;
  for (std::size_t i = 0; i < left.fields.size(); ++i) {
    const auto& a = left.fields[i];
    const auto& b = right.fields[i];
    if (a.number != b.number || a.cardinality != b.cardinality ||
        a.values.size() != b.values.size()) return false;
    for (std::size_t j = 0; j < a.values.size(); ++j) {
      const auto& x = a.values[j];
      const auto& y = b.values[j];
      if (x.index() != y.index()) return false;
      if (const auto* xa = std::get_if<CoreMessagePtr>(&x)) {
        if (*xa == nullptr || std::get<CoreMessagePtr>(y) == nullptr ||
            !same_document(**xa, *std::get<CoreMessagePtr>(y))) return false;
      } else {
        if (const auto* int32_value = std::get_if<std::int32_t>(&x)) {
          if (*int32_value != std::get<std::int32_t>(y)) return false;
        } else if (const auto* int64_value = std::get_if<std::int64_t>(&x)) {
          if (*int64_value != std::get<std::int64_t>(y)) return false;
        } else if (const auto* uint32_value = std::get_if<std::uint32_t>(&x)) {
          if (*uint32_value != std::get<std::uint32_t>(y)) return false;
        } else if (const auto* uint64_value = std::get_if<std::uint64_t>(&x)) {
          if (*uint64_value != std::get<std::uint64_t>(y)) return false;
        } else if (const auto* float_value = std::get_if<float>(&x)) {
          if (*float_value != std::get<float>(y)) return false;
        } else if (const auto* double_value = std::get_if<double>(&x)) {
          if (*double_value != std::get<double>(y)) return false;
        } else if (const auto* bool_value = std::get_if<bool>(&x)) {
          if (*bool_value != std::get<bool>(y)) return false;
        } else if (const auto* text_value = std::get_if<TextValue>(&x)) {
          if (text_value->value != std::get<TextValue>(y).value) return false;
        } else if (const auto* bytes_value = std::get_if<BytesValue>(&x)) {
          if (bytes_value->value != std::get<BytesValue>(y).value) return false;
        } else if (const auto* enum_value = std::get_if<EnumValue>(&x)) {
          const auto& other = std::get<EnumValue>(y);
          if (enum_value->type_name != other.type_name || enum_value->number != other.number)
            return false;
        }
      }
    }
  }
  return true;
}

struct MainSegment {
  std::uint64_t start{};
  std::uint64_t end{};
  Point3dEnu start_center{};
  Point3dEnu end_center{};
  double radius{};
};

double squared_norm(Point3dEnu value) {
  return value.x_m * value.x_m + value.y_m * value.y_m + value.z_m * value.z_m;
}

Point3dEnu subtract(Point3dEnu a, Point3dEnu b) {
  return {a.x_m - b.x_m, a.y_m - b.y_m, a.z_m - b.z_m};
}

Point3dEnu add_scaled(Point3dEnu a, Point3dEnu b, double scale) {
  return {a.x_m + b.x_m * scale, a.y_m + b.y_m * scale,
          a.z_m + b.z_m * scale};
}

Point3dEnu interpolate(Point3dEnu a, Point3dEnu b, double fraction) {
  return add_scaled(a, subtract(b, a), fraction);
}

double distance_at(const Point3dEnu scout_start, const Point3dEnu scout_end,
                   const Point3dEnu main_start, const Point3dEnu main_end,
                   double fraction) {
  return std::sqrt(squared_norm(subtract(
      interpolate(scout_start, scout_end, fraction),
      interpolate(main_start, main_end, fraction))));
}

// Returns the minimum center distance and its earliest normalized location.
std::pair<double, double> minimum_distance(const Point3dEnu scout_start,
                                           const Point3dEnu scout_end,
                                           const Point3dEnu main_start,
                                           const Point3dEnu main_end) {
  const Point3dEnu relative_start = subtract(scout_start, main_start);
  const Point3dEnu relative_delta = subtract(subtract(scout_end, scout_start),
                                              subtract(main_end, main_start));
  const double denominator = squared_norm(relative_delta);
  double fraction = 0.0;
  if (denominator > 0.0) {
    fraction = std::clamp(-((relative_start.x_m * relative_delta.x_m) +
                            (relative_start.y_m * relative_delta.y_m) +
                            (relative_start.z_m * relative_delta.z_m)) /
                              denominator,
                          0.0, 1.0);
  }
  const double at_fraction = std::sqrt(squared_norm(
      add_scaled(relative_start, relative_delta, fraction)));
  const double at_start = std::sqrt(squared_norm(relative_start));
  if (at_start < at_fraction) return {at_start, 0.0};
  return {at_fraction, fraction};
}

std::optional<double> first_distance_below(const Point3dEnu start,
                                           const Point3dEnu delta,
                                           const double threshold) {
  const double c = squared_norm(start) - threshold * threshold;
  const double b = 2.0 * (start.x_m * delta.x_m + start.y_m * delta.y_m +
                          start.z_m * delta.z_m);
  const double a = squared_norm(delta);
  if (c < 0.0) return 0.0;
  if (a == 0.0) return std::nullopt;
  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant <= 0.0) return std::nullopt;
  const double first = (-b - std::sqrt(discriminant)) / (2.0 * a);
  const double second = (-b + std::sqrt(discriminant)) / (2.0 * a);
  if (second < 0.0 || first > 1.0) return std::nullopt;
  return std::clamp(first, 0.0, 1.0);
}

std::optional<double> first_distance_above(const Point3dEnu start,
                                           const Point3dEnu delta,
                                           const double threshold) {
  const double c = squared_norm(start) - threshold * threshold;
  if (c > 0.0) return 0.0;
  const double a = squared_norm(delta);
  const double b = 2.0 * (start.x_m * delta.x_m + start.y_m * delta.y_m +
                          start.z_m * delta.z_m);
  if (a == 0.0) return std::nullopt;
  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant < 0.0) return std::nullopt;
  const double first = (-b - std::sqrt(discriminant)) / (2.0 * a);
  const double second = (-b + std::sqrt(discriminant)) / (2.0 * a);
  if (first >= 0.0 && first <= 1.0 && second > first) return first;
  if (second >= 0.0 && second <= 1.0 && first < 0.0) return second;
  return std::nullopt;
}

CoordinationEvaluationResult fail(CoordinationFailure code, std::string detail,
                                  std::uint64_t time = 0,
                                  double margin = 0.0) {
  return CoordinationEvaluationResult::failure(
      {code, time, margin, std::move(detail)});
}

}  // namespace

CoordinationEvaluationResult::CoordinationEvaluationResult(
    CoordinationReport report)
    : storage_(std::move(report)) {}
CoordinationEvaluationResult::CoordinationEvaluationResult(CoordinationError error)
    : storage_(std::move(error)) {}
CoordinationEvaluationResult CoordinationEvaluationResult::success(
    CoordinationReport report) {
  return CoordinationEvaluationResult(std::move(report));
}
CoordinationEvaluationResult CoordinationEvaluationResult::failure(
    CoordinationError error) {
  return CoordinationEvaluationResult(std::move(error));
}
bool CoordinationEvaluationResult::has_value() const noexcept {
  return std::holds_alternative<CoordinationReport>(storage_);
}
const CoordinationReport& CoordinationEvaluationResult::value() const {
  return std::get<CoordinationReport>(storage_);
}
const CoordinationError& CoordinationEvaluationResult::error() const {
  static const CoordinationError no_error{};
  const auto* error = std::get_if<CoordinationError>(&storage_);
  return error == nullptr ? no_error : *error;
}

CoordinationEvaluationResult DualRobotCoordinationEvaluator::evaluate(
    const MainRobotPrediction& prediction, const CoordinationConstraint& constraint,
    const std::vector<CoordinationMotionSample>& scout_motion,
    const CoordinationEvaluationConfig& configuration) {
  try {
    if (scout_motion.size() < 2U || configuration.scout_frame_id != "mission_enu" ||
        scout_motion.front().time_offset_ns != 0U)
      return fail(CoordinationFailure::invalid_input,
                  "Scout motion must be non-empty and use mission_enu");
    for (std::size_t i = 0; i < scout_motion.size(); ++i) {
      if ((i != 0U && scout_motion[i].time_offset_ns <= scout_motion[i - 1U].time_offset_ns) ||
          !std::isfinite(scout_motion[i].position_m.x_m) ||
          !std::isfinite(scout_motion[i].position_m.y_m) ||
          !std::isfinite(scout_motion[i].position_m.z_m))
        return fail(CoordinationFailure::invalid_input,
                    "Scout motion times must increase and positions be finite");
    }

    const auto& p = prediction.document();
    const auto& c = constraint.document();
    const auto& p_header = child(p, 1U);
    const auto& c_header = child(c, 1U);
    if (text(p_header, 7U) != text(c_header, 7U))
      return fail(CoordinationFailure::clock_domain_mismatch,
                  "prediction and coordination clock domains differ");
    if (u64(p, 2U) != u64(c, 2U) || u64(p, 3U) != u64(c, 3U) ||
        !same_document(child(p, 4U), child(c, 4U)))
      return fail(CoordinationFailure::mission_mismatch,
                  "prediction and coordination mission identities differ");
    if (text(p, 5U) != text(c, 6U) || u64(p, 7U) != u64(c, 7U) ||
        !same_document(child(p, 12U), child(c, 8U)))
      return fail(CoordinationFailure::prediction_mismatch,
                  "coordination does not bind this prediction");
    if (const auto channel = required<EnumValue>(c, 9U); channel.number != 3)
      return fail(CoordinationFailure::invalid_input,
                  "coordination channel is not MAIN_SCOUT_COOP");

    const auto min_separation = optional<double>(c, 10U);
    const auto max_communication = optional<double>(c, 11U);
    if (!min_separation || !max_communication || !std::isfinite(*min_separation) ||
        !std::isfinite(*max_communication) || *min_separation <= 0.0 ||
        *max_communication <= *min_separation)
      return fail(CoordinationFailure::coordination_infeasible,
                  "separation and communication bounds are infeasible");

    const auto& epoch = child(p, 8U);
    if (required<EnumValue>(epoch, 2U).number != 2)
      return fail(CoordinationFailure::synchronization_invalid,
                  "prediction alignment is not synchronized");
    if (const auto uncertainty = optional<std::uint64_t>(epoch, 3U);
        !uncertainty || *uncertainty > configuration.maximum_sync_uncertainty_ns)
      return fail(CoordinationFailure::synchronization_invalid,
                  "prediction synchronization uncertainty exceeds bound");

    const auto check_age = [&](std::int64_t received, std::uint64_t reject,
                               const char* label) -> std::optional<CoordinationEvaluationResult> {
      if (received < 0 || configuration.now_monotonic_ns < received)
        return fail(CoordinationFailure::clock_domain_mismatch,
                    std::string(label) + " receipt is outside Scout clock");
      if (static_cast<std::uint64_t>(configuration.now_monotonic_ns - received) > reject)
        return fail(CoordinationFailure::dependency_stale,
                    std::string(label) + " is stale");
      return std::nullopt;
    };
    if (auto error = check_age(configuration.prediction_received_at_monotonic_ns,
                               configuration.prediction_reject_ns, "prediction"))
      return std::move(*error);
    if (auto error = check_age(configuration.constraint_received_at_monotonic_ns,
                               configuration.constraint_reject_ns, "constraint"))
      return std::move(*error);

    const auto intervals = repeated_children(p, 11U);
    if (intervals.empty())
      return fail(CoordinationFailure::prediction_gap, "prediction has no intervals");
    std::vector<MainSegment> main;
    main.reserve(intervals.size());
    std::uint64_t previous_end = 0;
    Point3dEnu previous_center{};
    for (std::size_t i = 0; i < intervals.size(); ++i) {
      const auto& interval = *intervals[i];
      const auto start = optional<std::uint64_t>(interval, 1U).value_or(0U);
      const auto end = u64(interval, 2U);
      const auto& swept = child(interval, 3U);
      const auto start_center = point(child(swept, 1U));
      const auto end_center = point(child(swept, 2U));
      const auto radius = optional<double>(swept, 5U);
      if (!radius || !std::isfinite(*radius) || *radius < 0.0 || end <= start ||
          (i == 0U ? start != 0U : start != previous_end) ||
          (i != 0U && (start_center.x_m != previous_center.x_m ||
                       start_center.y_m != previous_center.y_m ||
                       start_center.z_m != previous_center.z_m)))
        return fail(CoordinationFailure::prediction_gap,
                    "prediction intervals are not contiguous and continuous");
      main.push_back({start, end, start_center, end_center, *radius});
      previous_end = end;
      previous_center = end_center;
    }

    const auto scout_horizon = scout_motion.back().time_offset_ns;
    if (scout_horizon > previous_end)
      return fail(CoordinationFailure::prediction_horizon_exceeded,
                  "Scout motion extends beyond prediction horizon", previous_end);

    CoordinationReport report;
    report.evaluated_horizon_ns = scout_horizon;
    report.minimum_separation_margin_m = std::numeric_limits<double>::infinity();
    report.minimum_communication_margin_m = std::numeric_limits<double>::infinity();
    const auto basis = required<EnumValue>(c, 12U).number;
    if (basis == 2) {
      if (find_field(c, 13U) == nullptr)
        return fail(CoordinationFailure::invalid_input,
                    "calibrated link assurance lacks its profile");
      report.calibrated_link_quality_asserted = true;
    } else if (basis != 1) {
      return fail(CoordinationFailure::invalid_input,
                  "unknown link assurance basis");
    }

    for (std::size_t si = 0; si + 1U < scout_motion.size(); ++si) {
      const auto st0 = scout_motion[si].time_offset_ns;
      const auto st1 = scout_motion[si + 1U].time_offset_ns;
      for (const auto& segment : main) {
        const auto overlap_start = std::max(st0, segment.start);
        const auto overlap_end = std::min(st1, segment.end);
        if (overlap_start > overlap_end) continue;
        const double scout_fraction0 = static_cast<double>(overlap_start - st0) /
                                       static_cast<double>(st1 - st0);
        const double scout_fraction1 = static_cast<double>(overlap_end - st0) /
                                       static_cast<double>(st1 - st0);
        const double main_fraction0 = segment.end == segment.start
                                          ? 0.0
                                          : static_cast<double>(overlap_start - segment.start) /
                                                static_cast<double>(segment.end - segment.start);
        const double main_fraction1 = segment.end == segment.start
                                          ? 0.0
                                          : static_cast<double>(overlap_end - segment.start) /
                                                static_cast<double>(segment.end - segment.start);
        const auto scout0 = interpolate(scout_motion[si].position_m,
                                        scout_motion[si + 1U].position_m, scout_fraction0);
        const auto scout1 = interpolate(scout_motion[si].position_m,
                                        scout_motion[si + 1U].position_m, scout_fraction1);
        const auto main0 = interpolate(segment.start_center, segment.end_center, main_fraction0);
        const auto main1 = interpolate(segment.start_center, segment.end_center, main_fraction1);
        const auto minimum = minimum_distance(scout0, scout1, main0, main1);
        const double separation_margin = minimum.first - segment.radius - *min_separation;
        report.minimum_separation_margin_m = std::min(report.minimum_separation_margin_m,
                                                       separation_margin);
        const double communication_margin = *max_communication -
                                            std::max(distance_at(scout0, scout1, main0, main1, 0.0),
                                                     distance_at(scout0, scout1, main0, main1, 1.0));
        report.minimum_communication_margin_m =
            std::min(report.minimum_communication_margin_m, communication_margin);
        const auto relative_start = subtract(scout0, main0);
        const auto relative_delta = subtract(subtract(scout1, scout0),
                                             subtract(main1, main0));
        if (separation_margin < 0.0) {
          report.separation_passed = false;
          const auto fraction = first_distance_below(
              relative_start, relative_delta, *min_separation + segment.radius)
                                    .value_or(minimum.second);
          const auto time = overlap_start + static_cast<std::uint64_t>(
                                             fraction * static_cast<double>(overlap_end - overlap_start));
          if (!report.earliest_failure_time_offset_ns || time < *report.earliest_failure_time_offset_ns)
            report.earliest_failure_time_offset_ns = time;
        }
        if (communication_margin < 0.0) {
          report.communication_distance_passed = false;
          const auto fraction = first_distance_above(
              relative_start, relative_delta, *max_communication)
                                    .value_or(1.0);
          const auto time = overlap_start + static_cast<std::uint64_t>(
                                             fraction * static_cast<double>(overlap_end - overlap_start));
          if (!report.earliest_failure_time_offset_ns || time < *report.earliest_failure_time_offset_ns)
            report.earliest_failure_time_offset_ns = time;
        }
      }
    }
    if (!report.separation_passed)
      return fail(CoordinationFailure::separation_violation,
                  "minimum separation margin is negative",
                  *report.earliest_failure_time_offset_ns,
                  report.minimum_separation_margin_m);
    if (!report.communication_distance_passed)
      return fail(CoordinationFailure::communication_distance_violation,
                  "maximum communication distance margin is negative",
                  *report.earliest_failure_time_offset_ns,
                  report.minimum_communication_margin_m);
    return CoordinationEvaluationResult::success(std::move(report));
  } catch (const std::exception& exception) {
    return fail(CoordinationFailure::invalid_input, exception.what());
  }
}

}  // namespace scout_planner::core
