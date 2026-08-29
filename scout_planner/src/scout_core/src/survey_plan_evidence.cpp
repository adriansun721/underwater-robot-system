#include "scout_planner/core/survey_plan_evidence.hpp"

#include "scout_planner/core/hybrid_map_query.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace scout_planner::core {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct Geometry {
  std::string id;
  std::uint64_t version{};
  Point3dEnu translation{};
  std::array<double, 4U> sensor_to_body_quaternion{0.0, 0.0, 0.0, 1.0};
  double horizontal_fov{};
  double vertical_fov{};
  double minimum_range{};
  double maximum_range{};
  bool production_approved{};
  std::uint64_t health_version{};
};

struct Quaternion {
  double x{};
  double y{};
  double z{};
  double w{1.0};
};

Quaternion multiply(const Quaternion a, const Quaternion b) {
  return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
          a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

Point3dEnu rotate(const Quaternion q, const Point3dEnu value) {
  const Quaternion p{value.x_m, value.y_m, value.z_m, 0.0};
  const Quaternion inverse{-q.x, -q.y, -q.z, q.w};
  const auto result = multiply(multiply(q, p), inverse);
  return {result.x, result.y, result.z};
}

const CoreField* field(const CoreMessage& message, std::uint32_t number) {
  const auto found = std::find_if(message.fields.begin(), message.fields.end(),
                                  [number](const CoreField& value) {
                                    return value.number == number;
                                  });
  return found == message.fields.end() ? nullptr : &*found;
}

template <typename T>
const T& required(const CoreMessage& message, std::uint32_t number) {
  const auto* value = field(message, number);
  if (value == nullptr || value->values.size() != 1U) {
    throw std::invalid_argument{"survey evidence field is malformed"};
  }
  return std::get<T>(value->values.front());
}

double real(const CoreMessage& message, std::uint32_t number) {
  return required<double>(message, number);
}

std::optional<double> optional_real(const CoreMessage& message,
                                    std::uint32_t number) {
  const auto* value = field(message, number);
  if (value == nullptr) return std::nullopt;
  if (value->values.size() != 1U) {
    throw std::invalid_argument{"optional survey evidence value is malformed"};
  }
  return std::get<double>(value->values.front());
}

std::string text(const CoreMessage& message, std::uint32_t number) {
  return required<TextValue>(message, number).value;
}

const CoreMessage& child(const CoreMessage& message, std::uint32_t number) {
  const auto& pointer = required<CoreMessagePtr>(message, number);
  if (!pointer) throw std::invalid_argument{"null survey evidence child"};
  return *pointer;
}

Point3dEnu point(const CoreMessage& message) {
  return {real(message, 1U), real(message, 2U), real(message, 3U)};
}

Aabb3dEnu region(const CoreMessage& message) {
  const auto* value = field(message, 1U);
  if (value == nullptr || value->values.size() != 6U) {
    throw std::invalid_argument{"survey region is malformed"};
  }
  std::array<double, 6U> bounds{};
  for (std::size_t index = 0U; index < bounds.size(); ++index) {
    bounds[index] = std::get<double>(value->values[index]);
  }
  return {{bounds[0], bounds[1], bounds[2]},
          {bounds[3], bounds[4], bounds[5]}};
}

bool valid_region(const Aabb3dEnu& value) {
  const std::array<double, 6U> bounds{value.minimum_m.x_m, value.minimum_m.y_m,
                                      value.minimum_m.z_m, value.maximum_m.x_m,
                                      value.maximum_m.y_m, value.maximum_m.z_m};
  return std::all_of(bounds.begin(), bounds.end(),
                     [](double item) { return std::isfinite(item); }) &&
         value.minimum_m.x_m < value.maximum_m.x_m &&
         value.minimum_m.y_m < value.maximum_m.y_m &&
         value.minimum_m.z_m < value.maximum_m.z_m;
}

Geometry read_geometry(const ScoutPlanningContext& context) {
  const auto& sensor = context.inputs().sensors.front();
  const auto& document = sensor.geometry.value.document();
  const auto& extrinsics = child(document, 5U);
  const auto& fov = child(document, 6U);
  const auto& health = sensor.health.value.document();
  const auto health_mode = required<EnumValue>(health, 6U).number;
  const auto* faults = field(health, 7U);
  const Quaternion quaternion{real(extrinsics, 4U), real(extrinsics, 5U),
                              real(extrinsics, 6U), real(extrinsics, 7U)};
  const auto qnorm = std::sqrt(quaternion.x * quaternion.x +
                               quaternion.y * quaternion.y +
                               quaternion.z * quaternion.z +
                               quaternion.w * quaternion.w);
  Geometry result{text(document, 2U), required<std::uint64_t>(document, 3U),
                 point(extrinsics),
                 {quaternion.x, quaternion.y, quaternion.z, quaternion.w},
                 real(fov, 1U), real(fov, 2U),
                 real(fov, 3U), real(fov, 4U),
                 required<bool>(document, 13U), required<std::uint64_t>(health, 3U)};
  if (result.id.empty() || result.version == 0U || !result.production_approved ||
      health_mode != 1 || (faults != nullptr && !faults->values.empty()) ||
      !std::isfinite(qnorm) || std::abs(qnorm - 1.0) > 1.0e-6 ||
      !std::isfinite(result.horizontal_fov) ||
      !std::isfinite(result.vertical_fov) || result.horizontal_fov <= 0.0 ||
      result.horizontal_fov > 2.0 * kPi || result.vertical_fov <= 0.0 ||
      result.vertical_fov > kPi || !std::isfinite(result.minimum_range) ||
      !std::isfinite(result.maximum_range) || result.minimum_range < 0.0 ||
      result.maximum_range <= result.minimum_range) {
    throw std::invalid_argument{"sensor geometry or health is invalid"};
  }
  return result;
}

Point3dEnu subtract(const Point3dEnu a, const Point3dEnu b) {
  return {a.x_m - b.x_m, a.y_m - b.y_m, a.z_m - b.z_m};
}

double norm(const Point3dEnu value) {
  return std::sqrt(value.x_m * value.x_m + value.y_m * value.y_m +
                   value.z_m * value.z_m);
}

bool inside(const Aabb3dEnu& box, const Point3dEnu value) {
  return value.x_m >= box.minimum_m.x_m && value.x_m <= box.maximum_m.x_m &&
         value.y_m >= box.minimum_m.y_m && value.y_m <= box.maximum_m.y_m &&
         value.z_m >= box.minimum_m.z_m && value.z_m <= box.maximum_m.z_m;
}

std::vector<Point3dEnu> samples(const Aabb3dEnu& box, const double spacing) {
  const auto count = [spacing](double low, double high) {
    return std::max<std::size_t>(1U, static_cast<std::size_t>(
        std::ceil((high - low) / spacing)));
  };
  const auto nx = count(box.minimum_m.x_m, box.maximum_m.x_m);
  const auto ny = count(box.minimum_m.y_m, box.maximum_m.y_m);
  const auto nz = count(box.minimum_m.z_m, box.maximum_m.z_m);
  std::vector<Point3dEnu> result;
  result.reserve(nx * ny * nz);
  for (std::size_t z = 0U; z < nz; ++z) {
    for (std::size_t y = 0U; y < ny; ++y) {
      for (std::size_t x = 0U; x < nx; ++x) {
        result.push_back({box.minimum_m.x_m + (static_cast<double>(x) + 0.5) *
                                              (box.maximum_m.x_m - box.minimum_m.x_m) / nx,
                          box.minimum_m.y_m + (static_cast<double>(y) + 0.5) *
                                              (box.maximum_m.y_m - box.minimum_m.y_m) / ny,
                          box.minimum_m.z_m + (static_cast<double>(z) + 0.5) *
                                              (box.maximum_m.z_m - box.minimum_m.z_m) / nz});
      }
    }
  }
  return result;
}

bool visible(const Geometry& geometry, const BezierSample4d& pose,
             const Point3dEnu target, const HybridMapQuery& map,
             const SurveyPlanEvidenceConfig& config) {
  const Quaternion vehicle_q{0.0, 0.0, std::sin(pose.yaw_rad * 0.5),
                             std::cos(pose.yaw_rad * 0.5)};
  const Quaternion sensor_q{geometry.sensor_to_body_quaternion[0],
                            geometry.sensor_to_body_quaternion[1],
                            geometry.sensor_to_body_quaternion[2],
                            geometry.sensor_to_body_quaternion[3]};
  const auto origin_offset = rotate(vehicle_q, geometry.translation);
  const Point3dEnu origin{pose.position.x_m + origin_offset.x_m,
                          pose.position.y_m + origin_offset.y_m,
                          pose.position.z_m + origin_offset.z_m};
  const auto delta = subtract(target, origin);
  const double distance = norm(delta);
  if (distance < geometry.minimum_range + config.pose_range_error_m ||
      distance > geometry.maximum_range - config.pose_position_error_m -
                     config.pose_range_error_m) {
    return false;
  }
  const auto body_delta = rotate(
      Quaternion{0.0, 0.0, -vehicle_q.z, vehicle_q.w}, delta);
  const auto sensor_delta = rotate(
      Quaternion{-sensor_q.x, -sensor_q.y, -sensor_q.z, sensor_q.w},
      body_delta);
  const double horizontal = std::atan2(sensor_delta.y_m, sensor_delta.x_m);
  const double vertical = std::atan2(sensor_delta.z_m,
                                     std::hypot(sensor_delta.x_m, sensor_delta.y_m));
  if (std::abs(horizontal) > geometry.horizontal_fov * 0.5 ||
      std::abs(vertical) > geometry.vertical_fov * 0.5) return false;
  const auto ray = map.query_supercover(origin, target, {0.0, 0.0, 0.0, 0.0, 0.0});
  return ray.has_value() && ray.value().state == MapCellState::free &&
         ray.value().information_gaps.empty();
}

}  // namespace

SurveyPlanEvidenceResult::SurveyPlanEvidenceResult(SurveyPlanEvidence value)
    : storage_(std::move(value)) {}
SurveyPlanEvidenceResult::SurveyPlanEvidenceResult(SurveyPlanEvidenceError error)
    : storage_(std::move(error)) {}
SurveyPlanEvidenceResult SurveyPlanEvidenceResult::success(SurveyPlanEvidence value) {
  return SurveyPlanEvidenceResult(std::move(value));
}
SurveyPlanEvidenceResult SurveyPlanEvidenceResult::failure(SurveyPlanEvidenceError error) {
  return SurveyPlanEvidenceResult(std::move(error));
}
bool SurveyPlanEvidenceResult::has_value() const noexcept {
  return std::holds_alternative<SurveyPlanEvidence>(storage_);
}
const SurveyPlanEvidence& SurveyPlanEvidenceResult::value() const {
  return std::get<SurveyPlanEvidence>(storage_);
}
const SurveyPlanEvidenceError& SurveyPlanEvidenceResult::error() const {
  return std::get<SurveyPlanEvidenceError>(storage_);
}

SurveyPlanEvidenceResult SurveyPlanEvidenceEvaluator::evaluate(
    const ScoutPlanningContext& context, const BezierTrajectory4d& trajectory,
    const SurveyPlanEvidenceConfig& config) {
  try {
    if (context.inputs().sensors.empty() || trajectory.segments().empty() ||
        trajectory.frame_id() != "mission_enu" ||
        !std::isfinite(config.minimum_observe_duration_s) ||
        config.minimum_observe_duration_s < 0.0 ||
        !std::isfinite(config.maximum_observe_speed_mps) ||
        config.maximum_observe_speed_mps < 0.0 ||
        !std::isfinite(config.sample_period_s) || config.sample_period_s <= 0.0 ||
        config.computation_config_version == 0U ||
        !std::isfinite(config.pose_position_error_m) ||
        config.pose_position_error_m < 0.0 ||
        !std::isfinite(config.pose_range_error_m) || config.pose_range_error_m < 0.0 ||
        !std::isfinite(config.minimum_coverage_ratio) ||
        config.minimum_coverage_ratio < 0.0 || config.minimum_coverage_ratio > 1.0 ||
        (config.mandatory_region.has_value() && !valid_region(*config.mandatory_region))) {
      return SurveyPlanEvidenceResult::failure({SurveyPlanEvidenceFailure::invalid_input, 0U,
                                                 "invalid plan evidence input"});
    }
    const auto& mission = context.inputs().mission.mission.document();
    const auto required_region = region(child(mission, 3U));
    const auto allowed_region = region(child(mission, 4U));
    if (!valid_region(required_region) || !valid_region(allowed_region)) {
      return SurveyPlanEvidenceResult::failure({SurveyPlanEvidenceFailure::invalid_input, 0U,
                                                 "mission region is invalid"});
    }
    const auto geometry = read_geometry(context);
    const auto map_document = context.inputs().map.value.document();
    MapSnapshotIdentity map_identity{text(map_document, 2U),
                                    required<std::uint64_t>(map_document, 3U), {}};
    const auto& identity = child(map_document, 4U);
    const auto& bytes = required<BytesValue>(identity, 1U).value;
    if (bytes.size() != map_identity.content_identity.size()) {
      return SurveyPlanEvidenceResult::failure({SurveyPlanEvidenceFailure::invalid_input, 0U,
                                                 "map identity is malformed"});
    }
    std::copy(bytes.begin(), bytes.end(), map_identity.content_identity.begin());
    const auto map = HybridMapQuery::create(context.inputs().map.value, map_identity);
    if (!map.has_value()) return SurveyPlanEvidenceResult::failure(
        {SurveyPlanEvidenceFailure::invalid_input, 0U, map.error().detail});
    if (trajectory.segments().size() < 3U) {
      return SurveyPlanEvidenceResult::failure({SurveyPlanEvidenceFailure::observe_invalid, 0U,
                                                 "final trajectory must expose approach, observe and exit segments"});
    }
    const auto observe_begin = trajectory.segments()[0].start_time_offset_ns +
                               trajectory.segments()[0].duration_ns;
    const auto observe_end = trajectory.segments()[trajectory.segments().size() - 1U].start_time_offset_ns;
    const double observe_duration_s = static_cast<double>(observe_end - observe_begin) / 1.0e9;
    if (observe_duration_s < config.minimum_observe_duration_s) {
      return SurveyPlanEvidenceResult::failure({SurveyPlanEvidenceFailure::observe_invalid, 0U,
                                                 "trajectory is shorter than observe dwell"});
    }
    const auto start = trajectory.evaluate_time(0U);
    const auto end = trajectory.evaluate_time(trajectory.duration_ns());
    if (!start.has_value() || !end.has_value() || !inside(allowed_region, start.value().position) ||
        !inside(allowed_region, end.value().position)) {
      return SurveyPlanEvidenceResult::failure({SurveyPlanEvidenceFailure::approach_invalid, 0U,
                                                 "trajectory endpoints leave allowed region"});
    }
    const auto motion_samples = std::max<std::uint64_t>(1U,
        trajectory.duration_ns() / 100'000'000U);
    auto previous_pose = trajectory.evaluate_time(0U);
    for (std::uint64_t index = 0U; index <= motion_samples; ++index) {
      const auto pose = trajectory.evaluate_time(
          index * trajectory.duration_ns() / motion_samples);
      const auto path_check = previous_pose.has_value() && pose.has_value()
                                  ? map.value().query_supercover(
                                        previous_pose.value().position,
                                        pose.value().position,
                                        {0.0, 0.0, 0.0, 0.0, 0.0})
                                  : std::optional<MapQueryResult<MapQuerySample>>{};
      const bool path_is_known_free =
          !previous_pose.has_value() ||
          (path_check.has_value() && path_check.value().value().state == MapCellState::free &&
           path_check.value().value().information_gaps.empty());
      if (!pose.has_value() || !inside(allowed_region, pose.value().position) ||
          (config.maximum_observe_speed_mps > 0.0 &&
           norm(pose.value().velocity_mps) > config.maximum_observe_speed_mps + 1.0e-9) ||
          !path_is_known_free) {
        return SurveyPlanEvidenceResult::failure(
            {SurveyPlanEvidenceFailure::observe_invalid,
             static_cast<std::size_t>(index),
             "trajectory leaves allowed region or exceeds observe speed"});
      }
      previous_pose = pose;
    }
    const double spacing = optional_real(mission, 6U).value_or(0.5);
    if (!std::isfinite(spacing) || spacing <= 0.0) {
      return SurveyPlanEvidenceResult::failure({SurveyPlanEvidenceFailure::invalid_input, 0U,
                                                 "mission required resolution is invalid"});
    }
    const auto required_samples = samples(required_region, std::max(0.01, spacing));
    std::size_t covered_count = 0U;
    std::size_t mandatory_count = 0U;
    std::size_t mandatory_covered = 0U;
    Aabb3dEnu covered_region{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    bool have_covered = false;
    for (const auto& target : required_samples) {
      bool covered = false;
      const bool mandatory = config.mandatory_region.has_value() &&
                             inside(*config.mandatory_region, target);
      mandatory_count += mandatory ? 1U : 0U;
      const auto sample_count = std::max<std::uint64_t>(1U,
          (trajectory.duration_ns() + static_cast<std::uint64_t>(config.sample_period_s * 1.0e9) - 1U) /
          static_cast<std::uint64_t>(config.sample_period_s * 1.0e9));
      for (std::uint64_t index = 0U; index <= sample_count && !covered; ++index) {
        const auto time = std::min(trajectory.duration_ns(),
          index * trajectory.duration_ns() / sample_count);
        const auto pose = trajectory.evaluate_time(time);
        covered = pose.has_value() && visible(geometry, pose.value(), target, map.value(), config);
      }
      if (covered) {
        ++covered_count;
        if (mandatory) ++mandatory_covered;
        if (!have_covered) {
          covered_region.minimum_m = covered_region.maximum_m = target;
          have_covered = true;
        } else {
          covered_region.minimum_m.x_m = std::min(covered_region.minimum_m.x_m, target.x_m);
          covered_region.minimum_m.y_m = std::min(covered_region.minimum_m.y_m, target.y_m);
          covered_region.minimum_m.z_m = std::min(covered_region.minimum_m.z_m, target.z_m);
          covered_region.maximum_m.x_m = std::max(covered_region.maximum_m.x_m, target.x_m);
          covered_region.maximum_m.y_m = std::max(covered_region.maximum_m.y_m, target.y_m);
          covered_region.maximum_m.z_m = std::max(covered_region.maximum_m.z_m, target.z_m);
        }
      }
    }
    const double ratio = required_samples.empty() ? 0.0 :
        static_cast<double>(covered_count) / required_samples.size();
    const double mandatory_ratio = mandatory_count == 0U ? 1.0 :
        static_cast<double>(mandatory_covered) / mandatory_count;
    const double required_ratio = std::max(config.minimum_coverage_ratio,
        [&mission] {
          const auto* value = field(mission, 5U);
          return value == nullptr ? 0.0 : std::get<double>(value->values.front());
        }());
    if (ratio + 1.0e-12 < required_ratio) {
      return SurveyPlanEvidenceResult::failure({SurveyPlanEvidenceFailure::insufficient_coverage, 0U,
                                                 "predicted coverage is insufficient"});
    }
    if (mandatory_count != mandatory_covered) {
      return SurveyPlanEvidenceResult::failure({SurveyPlanEvidenceFailure::mandatory_coverage_missing, 0U,
                                                 "mandatory region is not fully covered"});
    }
    SurveyPlanEvidence result{};
    result.mission_id = required<std::uint64_t>(mission, 1U);
    result.mission_version = required<std::uint64_t>(mission, 2U);
    result.mission_content_identity = context.inputs().mission.mission.canonical_wire_sha256();
    result.baseline_map_id = map_identity.map_id;
    result.baseline_map_version = map_identity.map_version;
    result.baseline_map_content_identity = map_identity.content_identity;
    result.trajectory_content_identity = trajectory.content_hash();
    result.planner_configuration_id =
        context.configuration().planner_configuration.id;
    result.planner_configuration_version =
        context.configuration().planner_configuration.version;
    result.planner_configuration_content_identity =
        context.configuration().planner_configuration.content_identity;
    result.computation_config_version = config.computation_config_version;
    result.computation_config_content_identity =
        config.computation_config_content_identity;
    result.sensor_id = geometry.id;
    result.geometry_version = geometry.version;
    result.health_version = geometry.health_version;
    result.geometry_content_identity =
        context.inputs().sensors.front().geometry.value.canonical_wire_sha256();
    result.health_content_identity =
        context.inputs().sensors.front().health.value.canonical_wire_sha256();
    result.predicted_covered_region = covered_region;
    result.conservative_predicted_coverage_ratio = ratio;
    result.predicted_resolution_m = spacing;
    result.mandatory_coverage_ratio = mandatory_ratio;
    result.trajectory_duration_ns = trajectory.duration_ns();
    result.evaluated_sample_count = required_samples.size();
    result.covered_sample_count = covered_count;
    result.approach_valid = true;
    result.observe_valid = true;
    result.exit_valid = true;
    result.completion_evidence = false;
    return SurveyPlanEvidenceResult::success(std::move(result));
  } catch (const std::exception& error) {
    return SurveyPlanEvidenceResult::failure({SurveyPlanEvidenceFailure::invalid_input, 0U,
                                               error.what()});
  }
}

}  // namespace scout_planner::core
