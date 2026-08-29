#include "scout_planner/core/survey_action.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace scout_planner::core {
namespace {

constexpr double kPi = 3.14159265358979323846;

const CoreField* find_field(const CoreMessage& message, std::uint32_t number) {
  const auto found = std::find_if(message.fields.begin(), message.fields.end(),
                                  [number](const CoreField& field) {
                                    return field.number == number;
                                  });
  return found == message.fields.end() ? nullptr : &*found;
}

template <typename T>
const T& required(const CoreMessage& message, std::uint32_t number) {
  const auto* value = find_field(message, number);
  if (value == nullptr || value->values.size() != 1U) {
    throw std::invalid_argument{"required survey field is malformed"};
  }
  return std::get<T>(value->values.front());
}

template <typename T>
std::optional<T> optional(const CoreMessage& message, std::uint32_t number) {
  const auto* value = find_field(message, number);
  if (value == nullptr) return std::nullopt;
  if (value->values.size() != 1U) {
    throw std::invalid_argument{"optional survey field is malformed"};
  }
  return std::get<T>(value->values.front());
}

const CoreMessage& child(const CoreMessage& message, std::uint32_t number) {
  const auto& value = required<CoreMessagePtr>(message, number);
  if (value == nullptr) throw std::invalid_argument{"null survey child"};
  return *value;
}

double real(const CoreMessage& message, std::uint32_t number) {
  return required<double>(message, number);
}

std::string text(const CoreMessage& message, std::uint32_t number) {
  return required<TextValue>(message, number).value;
}

Point3dEnu point(const CoreMessage& message) {
  return {real(message, 1U), real(message, 2U), real(message, 3U)};
}

Aabb3dEnu region(const CoreMessage& message) {
  const auto* field = find_field(message, 1U);
  if (field == nullptr || field->values.size() != 6U) {
    throw std::invalid_argument{"survey region must contain six bounds"};
  }
  std::array<double, 6U> values{};
  for (std::size_t i = 0; i < values.size(); ++i) {
    values[i] = std::get<double>(field->values[i]);
  }
  return {{values[0], values[1], values[2]},
          {values[3], values[4], values[5]}};
}

bool valid_region(const Aabb3dEnu& value) {
  const std::array values{value.minimum_m.x_m, value.minimum_m.y_m,
                          value.minimum_m.z_m, value.maximum_m.x_m,
                          value.maximum_m.y_m, value.maximum_m.z_m};
  return std::all_of(values.begin(), values.end(),
                     [](double v) { return std::isfinite(v); }) &&
         value.minimum_m.x_m < value.maximum_m.x_m &&
         value.minimum_m.y_m < value.maximum_m.y_m &&
         value.minimum_m.z_m < value.maximum_m.z_m;
}

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

Point3dEnu rotate(const Quaternion q, const Point3dEnu v) {
  const Quaternion p{v.x_m, v.y_m, v.z_m, 0.0};
  const Quaternion qi{-q.x, -q.y, -q.z, q.w};
  const auto r = multiply(multiply(q, p), qi);
  return {r.x, r.y, r.z};
}

double norm(const Point3dEnu p) {
  return std::sqrt(p.x_m * p.x_m + p.y_m * p.y_m + p.z_m * p.z_m);
}

Point3dEnu add(const Point3dEnu a, const Point3dEnu b) {
  return {a.x_m + b.x_m, a.y_m + b.y_m, a.z_m + b.z_m};
}

Point3dEnu subtract(const Point3dEnu a, const Point3dEnu b) {
  return {a.x_m - b.x_m, a.y_m - b.y_m, a.z_m - b.z_m};
}

Point3dEnu clamp_point(const Point3dEnu p, const Aabb3dEnu& bounds) {
  return {std::clamp(p.x_m, bounds.minimum_m.x_m, bounds.maximum_m.x_m),
          std::clamp(p.y_m, bounds.minimum_m.y_m, bounds.maximum_m.y_m),
          std::clamp(p.z_m, bounds.minimum_m.z_m, bounds.maximum_m.z_m)};
}

double yaw_from_quaternion(const Quaternion q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double normalize_yaw(double yaw) {
  while (yaw >= kPi) yaw -= 2.0 * kPi;
  while (yaw < -kPi) yaw += 2.0 * kPi;
  return yaw;
}

struct Geometry {
  std::string id;
  std::uint64_t version{};
  Point3dEnu translation{};
  Quaternion sensor_to_body{};
  double horizontal_fov{};
  double vertical_fov{};
  double minimum_range{};
  double maximum_range{};
  double range_resolution{};
  bool production_approved{};
  int health{};
  std::uint64_t health_version{};
};

Geometry read_geometry(const ScoutPlanningContext& context) {
  const auto& sensor = context.inputs().sensors.front();
  const auto& geometry = sensor.geometry.value.document();
  const auto& extrinsics = child(geometry, 5U);
  const auto& fov = child(geometry, 6U);
  const auto health = sensor.health.value.document();
  const auto health_enum = required<EnumValue>(health, 6U).number;
  const auto* faults = find_field(health, 7U);
  if (health_enum == 1 && faults != nullptr && !faults->values.empty()) {
    throw std::invalid_argument{"nominal sensor has active faults"};
  }
  const auto q = Quaternion{real(extrinsics, 4U), real(extrinsics, 5U),
                            real(extrinsics, 6U), real(extrinsics, 7U)};
  const auto qnorm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  Geometry result{text(geometry, 2U), required<std::uint64_t>(geometry, 3U),
                 {real(extrinsics, 1U), real(extrinsics, 2U),
                  real(extrinsics, 3U)},
                 q,
                 real(fov, 1U),
                 real(fov, 2U),
                 real(fov, 3U),
                 real(fov, 4U),
                 real(fov, 5U),
                 optional<bool>(geometry, 13U).value_or(false),
                 health_enum,
                 required<std::uint64_t>(health, 3U)};
  if (result.id.empty() || result.version == 0U ||
      !std::isfinite(qnorm) || std::abs(qnorm - 1.0) > 1.0e-6 ||
      !std::isfinite(result.horizontal_fov) ||
      !std::isfinite(result.vertical_fov) || result.horizontal_fov <= 0.0 ||
      result.horizontal_fov > 2.0 * kPi || result.vertical_fov <= 0.0 ||
      result.vertical_fov > kPi || !std::isfinite(result.minimum_range) ||
      !std::isfinite(result.maximum_range) ||
      result.minimum_range < 0.0 || result.maximum_range <= result.minimum_range ||
      !std::isfinite(result.range_resolution) || result.range_resolution <= 0.0) {
    throw std::invalid_argument{"sensor geometry is invalid"};
  }
  return result;
}

bool visible(const Geometry& geometry, const Quaternion vehicle_q,
             const ObservationPose& pose,
             const Point3dEnu target, const HybridMapQuery& map,
             const SurveyActionConfig& config) {
  const auto body_q = Quaternion{0.0, 0.0, std::sin(pose.yaw_rad * 0.5),
                                 std::cos(pose.yaw_rad * 0.5)};
  const auto world_q = multiply(vehicle_q, multiply(body_q, geometry.sensor_to_body));
  const auto sensor_origin = add(pose.position_m, rotate(vehicle_q, geometry.translation));
  const auto local = rotate({-world_q.x, -world_q.y, -world_q.z, world_q.w},
                            subtract(target, sensor_origin));
  const double distance = norm(local);
  const double range_min = geometry.minimum_range + config.pose_range_error_m;
  const double range_max = geometry.maximum_range - config.pose_position_error_m -
                           config.pose_range_error_m;
  if (!(distance >= range_min && distance <= range_max)) return false;
  const double horizontal = std::atan2(local.y_m, local.x_m);
  const double vertical = std::atan2(local.z_m,
                                     std::hypot(local.x_m, local.y_m));
  if (std::abs(horizontal) > geometry.horizontal_fov * 0.5 ||
      std::abs(vertical) > geometry.vertical_fov * 0.5) return false;
  const auto ray = map.query_supercover(sensor_origin, target,
                                        {0.0, 0.0, 0.0, 0.0, 0.0});
  return ray.has_value() && ray.value().state == MapCellState::free &&
         ray.value().information_gaps.empty();
}

std::vector<Point3dEnu> sample_region(const Aabb3dEnu& bounds, double spacing) {
  std::vector<Point3dEnu> result;
  const auto count = [spacing](double low, double high) {
    return std::max<std::size_t>(1U, static_cast<std::size_t>(
        std::ceil((high - low) / spacing)));
  };
  const auto nx = count(bounds.minimum_m.x_m, bounds.maximum_m.x_m);
  const auto ny = count(bounds.minimum_m.y_m, bounds.maximum_m.y_m);
  const auto nz = count(bounds.minimum_m.z_m, bounds.maximum_m.z_m);
  result.reserve(nx * ny * nz);
  for (std::size_t z = 0; z < nz; ++z) for (std::size_t y = 0; y < ny; ++y)
    for (std::size_t x = 0; x < nx; ++x) {
      result.push_back({bounds.minimum_m.x_m + (static_cast<double>(x) + 0.5) *
                                           (bounds.maximum_m.x_m - bounds.minimum_m.x_m) / nx,
                        bounds.minimum_m.y_m + (static_cast<double>(y) + 0.5) *
                                           (bounds.maximum_m.y_m - bounds.minimum_m.y_m) / ny,
                        bounds.minimum_m.z_m + (static_cast<double>(z) + 0.5) *
                                           (bounds.maximum_m.z_m - bounds.minimum_m.z_m) / nz});
    }
  return result;
}

bool inside(const Aabb3dEnu& box, const Point3dEnu p) {
  return p.x_m >= box.minimum_m.x_m && p.x_m <= box.maximum_m.x_m &&
         p.y_m >= box.minimum_m.y_m && p.y_m <= box.maximum_m.y_m &&
         p.z_m >= box.minimum_m.z_m && p.z_m <= box.maximum_m.z_m;
}

}  // namespace

SurveyActionResult::SurveyActionResult(SurveyActionReport report)
    : storage_(std::move(report)) {}
SurveyActionResult::SurveyActionResult(SurveyActionError error)
    : storage_(std::move(error)) {}
SurveyActionResult SurveyActionResult::success(SurveyActionReport report) {
  return SurveyActionResult(std::move(report));
}
SurveyActionResult SurveyActionResult::failure(SurveyActionError error) {
  return SurveyActionResult(std::move(error));
}
bool SurveyActionResult::has_value() const noexcept {
  return std::holds_alternative<SurveyActionReport>(storage_);
}
const SurveyActionReport& SurveyActionResult::value() const {
  return std::get<SurveyActionReport>(storage_);
}
const SurveyActionError& SurveyActionResult::error() const {
  return std::get<SurveyActionError>(storage_);
}

SurveyActionResult SurveyActionPlanner::plan(
    const ScoutPlanningContext& context, const SurveyActionConfig& config) {
  try {
    if (!std::isfinite(config.observation_dwell_s) || config.observation_dwell_s <= 0.0 ||
        !std::isfinite(config.transit_speed_mps) || config.transit_speed_mps <= 0.0 ||
        !std::isfinite(config.pose_position_error_m) || config.pose_position_error_m < 0.0 ||
        !std::isfinite(config.pose_range_error_m) || config.pose_range_error_m < 0.0 ||
        !std::isfinite(config.minimum_coverage_ratio) || config.minimum_coverage_ratio < 0.0 ||
        config.minimum_coverage_ratio > 1.0) {
      return SurveyActionResult::failure({SurveyActionFailure::invalid_input, 0U, "invalid survey action configuration"});
    }
    const auto& mission = context.inputs().mission.mission.document();
  const auto required_region = region(child(mission, 3U));
    const auto allowed_region = region(child(mission, 4U));
    if (!valid_region(required_region) || !valid_region(allowed_region) ||
        (config.mandatory_region.has_value() &&
         !valid_region(*config.mandatory_region))) {
      return SurveyActionResult::failure({SurveyActionFailure::mission_region_invalid, 0U, "mission region is invalid"});
    }
    const auto geometry = read_geometry(context);
    if (!geometry.production_approved) {
      return SurveyActionResult::failure({SurveyActionFailure::sensor_geometry_invalid, 0U, "sensor geometry is not production approved"});
    }
    if (geometry.health != 1) {
      return SurveyActionResult::failure({SurveyActionFailure::sensor_not_nominal, 0U, "sensor health is not NOMINAL"});
    }
    const auto map_document = context.inputs().map.value.document();
    MapSnapshotIdentity identity{text(map_document, 2U),
                                 required<std::uint64_t>(map_document, 3U), {}};
    const auto& identity_message = child(map_document, 4U);
    const auto& hash = required<BytesValue>(identity_message, 1U).value;
    if (hash.size() == identity.content_identity.size()) {
      std::copy(hash.begin(), hash.end(), identity.content_identity.begin());
    }
    const auto map = HybridMapQuery::create(context.inputs().map.value, identity);
    if (!map.has_value()) return SurveyActionResult::failure({SurveyActionFailure::invalid_input, 0U, map.error().detail});

    const auto& navigation = context.inputs().navigation.value.document();
    if (required<EnumValue>(navigation, 8U).number != 1) {
      return SurveyActionResult::failure({SurveyActionFailure::no_observation_pose, 0U,
                                          "navigation solution is not valid"});
    }
    const auto& pose = child(navigation, 4U);
    const Point3dEnu start{real(pose, 1U), real(pose, 2U), real(pose, 3U)};
    const Quaternion vehicle_q{real(pose, 4U), real(pose, 5U), real(pose, 6U), real(pose, 7U)};
    const double vehicle_q_norm = std::sqrt(vehicle_q.x * vehicle_q.x +
                                            vehicle_q.y * vehicle_q.y +
                                            vehicle_q.z * vehicle_q.z +
                                            vehicle_q.w * vehicle_q.w);
    if (!std::isfinite(vehicle_q_norm) || std::abs(vehicle_q_norm - 1.0) > 1.0e-6) {
      return SurveyActionResult::failure({SurveyActionFailure::no_observation_pose, 0U,
                                          "navigation pose quaternion is invalid"});
    }
    const auto target = clamp_point({(required_region.minimum_m.x_m + required_region.maximum_m.x_m) * 0.5,
                                     (required_region.minimum_m.y_m + required_region.maximum_m.y_m) * 0.5,
                                     (required_region.minimum_m.z_m + required_region.maximum_m.z_m) * 0.5}, allowed_region);
    const auto approach_check = map.value().query_supercover(start, target, {0, 0, 0, 0, 0});
    if (!approach_check.has_value() || approach_check.value().state != MapCellState::free ||
        !approach_check.value().information_gaps.empty()) {
      return SurveyActionResult::failure({SurveyActionFailure::approach_infeasible, 0U, "approach path is not known free"});
    }
    const auto exit_target = clamp_point(start, allowed_region);
    const auto exit_check = map.value().query_supercover(target, exit_target, {0, 0, 0, 0, 0});
    if (!exit_check.has_value() || exit_check.value().state != MapCellState::free ||
        !exit_check.value().information_gaps.empty()) {
      return SurveyActionResult::failure({SurveyActionFailure::exit_infeasible, 0U, "exit path is not known free"});
    }
    const double observation_yaw = normalize_yaw(
        std::atan2(target.y_m - start.y_m, target.x_m - start.x_m) -
        yaw_from_quaternion(vehicle_q));
    const double spacing = config.sample_resolution_m > 0.0 ? config.sample_resolution_m :
                           std::max(0.01, optional<double>(mission, 6U).value_or(0.5));
    const auto samples = sample_region(required_region, spacing);
    const ObservationPose observation{target, observation_yaw, 0.0, 0.0, config.observation_dwell_s};
    std::size_t covered = 0U;
    std::size_t mandatory = 0U;
    std::size_t mandatory_covered = 0U;
    for (const auto sample : samples) {
      const bool is_mandatory = config.mandatory_region.has_value() && inside(*config.mandatory_region, sample);
      mandatory += is_mandatory ? 1U : 0U;
      const bool is_covered = visible(geometry, vehicle_q, observation, sample, map.value(), config);
      covered += is_covered ? 1U : 0U;
      mandatory_covered += is_mandatory && is_covered ? 1U : 0U;
    }
    const double coverage = samples.empty() ? 0.0 : static_cast<double>(covered) / samples.size();
    const double mandatory_ratio = mandatory == 0U ? 1.0 : static_cast<double>(mandatory_covered) / mandatory;
    const double required_coverage = optional<double>(mission, 5U).value_or(config.minimum_coverage_ratio);
    if (coverage + 1.0e-12 < required_coverage) {
      return SurveyActionResult::failure({SurveyActionFailure::insufficient_coverage, 0U, "required region coverage is insufficient"});
    }
    if (mandatory > 0U && mandatory_covered != mandatory) {
      return SurveyActionResult::failure({SurveyActionFailure::mandatory_coverage_missing, 0U, "mandatory sub-volume is not fully covered"});
    }
    const double approach_duration = norm(subtract(target, start)) / config.transit_speed_mps;
    const double exit_duration = norm(subtract(exit_target, target)) / config.transit_speed_mps;
    SurveyActionReport report;
    report.segments = {{SurveyActionPhase::approach, start, target, approach_duration, observation_yaw, 0.0},
                       {SurveyActionPhase::observe, target, target, config.observation_dwell_s, observation_yaw, config.observation_dwell_s},
                       {SurveyActionPhase::exit, target, exit_target, exit_duration, observation_yaw, 0.0}};
    report.observation_poses = {observation};
    report.conservative_coverage_ratio = coverage;
    report.mandatory_coverage_ratio = mandatory_ratio;
    report.required_sample_count = samples.size();
    report.covered_sample_count = covered;
    report.mandatory_sample_count = mandatory;
    report.mandatory_covered_sample_count = mandatory_covered;
    report.sensor_id = geometry.id;
    report.geometry_version = geometry.version;
    report.health_version = geometry.health_version;
    return SurveyActionResult::success(std::move(report));
  } catch (const std::exception& error) {
    return SurveyActionResult::failure({SurveyActionFailure::invalid_input, 0U, error.what()});
  }
}

}  // namespace scout_planner::core
