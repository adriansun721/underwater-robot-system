#include "scout_planner/core/capability_energy_gate.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace scout_planner::core {
namespace {

const CoreField& field(const CoreMessage& message, const std::uint32_t number) {
  const auto found = std::find_if(
      message.fields.begin(), message.fields.end(),
      [number](const CoreField& value) { return value.number == number; });
  if (found == message.fields.end()) {
    throw std::invalid_argument{"validated capability field is missing"};
  }
  return *found;
}

template <typename T>
const T& scalar(const CoreMessage& message, const std::uint32_t number) {
  const auto& values = field(message, number).values;
  if (values.size() != 1U) {
    throw std::invalid_argument{"validated singular field is malformed"};
  }
  return std::get<T>(values.front());
}

const CoreMessage& child(const CoreMessage& message, const std::uint32_t number) {
  const auto& value = scalar<CoreMessagePtr>(message, number);
  if (value == nullptr) {
    throw std::invalid_argument{"validated child is null"};
  }
  return *value;
}

double optional_double(const CoreMessage& message, const std::uint32_t number,
                       const double fallback = 0.0) {
  const auto found = std::find_if(
      message.fields.begin(), message.fields.end(),
      [number](const CoreField& value) { return value.number == number; });
  if (found == message.fields.end()) {
    return fallback;
  }
  if (found->values.size() != 1U) {
    throw std::invalid_argument{"validated optional field is malformed"};
  }
  return std::get<double>(found->values.front());
}

bool optional_bool(const CoreMessage& message, const std::uint32_t number,
                   const bool fallback = false) {
  const auto found = std::find_if(
      message.fields.begin(), message.fields.end(),
      [number](const CoreField& value) { return value.number == number; });
  if (found == message.fields.end()) {
    return fallback;
  }
  if (found->values.size() != 1U) {
    throw std::invalid_argument{"validated optional boolean is malformed"};
  }
  return std::get<bool>(found->values.front());
}

std::string text(const CoreMessage& message, const std::uint32_t number) {
  return scalar<TextValue>(message, number).value;
}

Point3dEnu vector3(const CoreMessage& message) {
  return {optional_double(message, 1U), optional_double(message, 2U),
          optional_double(message, 3U)};
}

bool finite(const MotionSample& sample) {
  const std::array values{sample.duration_s, sample.position_m.x_m,
                          sample.position_m.y_m, sample.position_m.z_m,
                          sample.velocity_mps.x_m, sample.velocity_mps.y_m,
                          sample.velocity_mps.z_m, sample.acceleration_mps2.x_m,
                          sample.acceleration_mps2.y_m,
                          sample.acceleration_mps2.z_m, sample.yaw_rad,
                          sample.yaw_rate_radps, sample.yaw_acceleration_radps2,
                          sample.roll_rad, sample.pitch_rad};
  return std::all_of(values.begin(), values.end(),
                     [](const double value) { return std::isfinite(value); });
}

double norm(const Point3dEnu value) {
  return std::sqrt(value.x_m * value.x_m + value.y_m * value.y_m +
                   value.z_m * value.z_m);
}

bool inside(const Point3dEnu point, const Point3dEnu minimum,
            const Point3dEnu maximum) {
  return point.x_m >= minimum.x_m && point.y_m >= minimum.y_m &&
         point.z_m >= minimum.z_m && point.x_m <= maximum.x_m &&
         point.y_m <= maximum.y_m && point.z_m <= maximum.z_m;
}

struct Inputs {
  double max_water_speed;
  double max_acceleration;
  double max_vertical_ascent;
  double max_vertical_descent;
  double max_yaw_rate;
  double max_yaw_acceleration;
  double max_roll;
  double max_pitch;
  double max_vertical_acceleration;
  double speed_error;
  double acceleration_error;
  double yaw_rate_error;
  double yaw_acceleration_error;
  double hotel_power;
  double speed_power;
  double cubic_power;
  double acceleration_power;
  double yaw_power;
  double model_error_power;
  double available_energy;
  double reserve_energy;
  double return_energy;
  double risk_energy;
  std::int32_t contingency;
  std::int32_t health;
  std::uint64_t capability_version;
  std::uint64_t energy_model_version;
  std::uint64_t energy_state_version;
  Point3dEnu current_reference;
  Point3dEnu current_velocity;
  Point3dEnu current_error;
  Point3dEnu current_minimum;
  Point3dEnu current_maximum;
  std::string operating_domain_id;
  std::int32_t current_validity;
  std::array<double, 9U> current_gradient;
  bool has_current_gradient;
};

Inputs read_inputs(const ScoutPlanningContext& context) {
  const auto& values = context.inputs();
  const auto& capability = values.capability.value.document();
  const auto& motion = child(capability, 5U);
  const auto& energy_model = values.energy_model.value.document();
  const auto& power = child(energy_model, 6U);
  const auto& energy_state = values.energy_state.value.document();
  const auto& current = values.current.value.document();
  const auto& region = child(current, 7U);
  const auto& reference = child(current, 8U);
  const auto& current_velocity = child(current, 9U);
  const auto& current_error = child(current, 10U);
  const auto& capability_health = scalar<EnumValue>(capability, 4U);
  const auto& health = scalar<EnumValue>(values.thruster_health.value.document(),
                                         6U);

  if (capability_health.number != health.number) {
    throw std::runtime_error{"capability and thruster health profiles differ"};
  }
  if (!optional_bool(capability, 14U) || !optional_bool(energy_model, 13U)) {
    throw std::runtime_error{"capability or energy model is not production approved"};
  }
  const auto current_validity = scalar<EnumValue>(current, 15U).number;
  if (current_validity != 1) {
    throw std::runtime_error{"current estimate is not valid for new exploration"};
  }
  const auto region_values = field(region, 1U).values;
  if (region_values.size() != 6U) {
    throw std::runtime_error{"current region is malformed"};
  }
  const std::array<double, 6U> bounds = [&] {
    std::array<double, 6U> result{};
    for (std::size_t i = 0U; i < result.size(); ++i) {
      result[i] = std::get<double>(region_values[i]);
    }
    return result;
  }();
  std::array<double, 9U> gradient{};
  bool has_gradient = false;
  const auto gradient_field = std::find_if(
      current.fields.begin(), current.fields.end(),
      [](const CoreField& value) { return value.number == 12U; });
  if (gradient_field != current.fields.end()) {
    const auto& gradient_message = child(current, 12U);
    const auto& gradient_values = field(gradient_message, 1U).values;
    if (gradient_values.size() != gradient.size()) {
      throw std::runtime_error{"current spatial gradient is malformed"};
    }
    for (std::size_t index = 0U; index < gradient.size(); ++index) {
      gradient[index] = std::get<double>(gradient_values[index]);
    }
    has_gradient = true;
  }
  Inputs result{
      optional_double(motion, 1U), optional_double(motion, 2U),
      optional_double(motion, 3U), optional_double(motion, 4U),
      optional_double(motion, 5U), optional_double(motion, 6U),
      optional_double(motion, 7U), optional_double(motion, 8U),
      optional_double(motion, 12U),
      std::max(optional_double(current, 11U), norm(vector3(current_error))),
      0.0, 0.0, 0.0,
      optional_double(power, 1U), optional_double(power, 2U),
      optional_double(power, 3U), optional_double(power, 4U),
      optional_double(power, 5U), optional_double(power, 6U),
      optional_double(energy_state, 8U), optional_double(energy_state, 9U),
      optional_double(energy_state, 10U), optional_double(energy_state, 11U),
      scalar<EnumValue>(energy_state, 12U).number,
      capability_health.number,
      scalar<std::uint64_t>(capability, 3U), scalar<std::uint64_t>(energy_model, 3U),
      scalar<std::uint64_t>(energy_state, 4U), vector3(reference),
      vector3(current_velocity), vector3(current_error),
      {bounds[0], bounds[1], bounds[2]}, {bounds[3], bounds[4], bounds[5]},
      text(capability, 8U), current_validity, gradient, has_gradient};
  return result;
}

}  // namespace

CapabilityEnergyResult::CapabilityEnergyResult(CapabilityEnergyReport report)
    : storage_(std::move(report)) {}

CapabilityEnergyResult::CapabilityEnergyResult(CapabilityEnergyError error)
    : storage_(std::move(error)) {}

CapabilityEnergyResult CapabilityEnergyResult::success(
    CapabilityEnergyReport report) {
  return CapabilityEnergyResult(std::move(report));
}

CapabilityEnergyResult CapabilityEnergyResult::failure(CapabilityEnergyError error) {
  return CapabilityEnergyResult(std::move(error));
}

bool CapabilityEnergyResult::has_value() const noexcept {
  return std::holds_alternative<CapabilityEnergyReport>(storage_);
}

const CapabilityEnergyReport& CapabilityEnergyResult::value() const {
  return std::get<CapabilityEnergyReport>(storage_);
}

const CapabilityEnergyError& CapabilityEnergyResult::error() const {
  return std::get<CapabilityEnergyError>(storage_);
}

CapabilityEnergyResult CapabilityEnergyGate::evaluate(
    const ScoutPlanningContext& context,
    const std::vector<MotionSample>& trajectory,
    const CapabilityEnergyGateConfig& configuration) {
  const std::array configured_margins{
      configuration.conservative_acceleration_margin_mps2,
      configuration.yaw_rate_error_radps,
      configuration.yaw_acceleration_error_radps2};
  if (!std::all_of(configured_margins.begin(), configured_margins.end(),
                  [](const double value) {
                    return std::isfinite(value) && value >= 0.0;
                  })) {
    return CapabilityEnergyResult::failure(
        {CapabilityEnergyFailure::invalid_trajectory, 0U,
         "gate calibration margins are non-finite or negative"});
  }
  if (trajectory.empty()) {
    return CapabilityEnergyResult::failure(
        {CapabilityEnergyFailure::invalid_trajectory, 0U,
         "trajectory fragment is empty"});
  }

  Inputs inputs{};
  try {
    inputs = read_inputs(context);
  } catch (const std::runtime_error& error) {
    const auto code = std::string(error.what()).find("production") !=
                              std::string::npos
                          ? CapabilityEnergyFailure::non_production_profile
                          : std::string(error.what()).find("health") !=
                                    std::string::npos
                                ? CapabilityEnergyFailure::health_profile_mismatch
                                : CapabilityEnergyFailure::current_invalid;
    return CapabilityEnergyResult::failure({code, 0U, error.what()});
  } catch (const std::exception& error) {
    return CapabilityEnergyResult::failure(
        {CapabilityEnergyFailure::current_invalid, 0U, error.what()});
  }

  const auto& capability_document = context.inputs().capability.value.document();
  const auto& energy_model_document = context.inputs().energy_model.value.document();
  const auto& current_document = context.inputs().current.value.document();
  const auto& energy_state_document = context.inputs().energy_state.value.document();
  const auto& capability_profile = child(energy_model_document, 5U);
  const auto& capability_document_profile_id = text(capability_document, 2U);
  const auto& capability_profile_id = text(capability_profile, 1U);
  if (capability_profile_id != capability_document_profile_id ||
      scalar<std::uint64_t>(capability_profile, 2U) !=
          inputs.capability_version) {
    return CapabilityEnergyResult::failure(
        {CapabilityEnergyFailure::health_profile_mismatch, 0U,
         "energy model is not bound to the active capability profile"});
  }
  const std::array energy_values{
      inputs.hotel_power, inputs.speed_power, inputs.cubic_power,
      inputs.acceleration_power, inputs.yaw_power, inputs.model_error_power,
      inputs.available_energy, inputs.reserve_energy, inputs.return_energy,
      inputs.risk_energy};
  if (!std::all_of(energy_values.begin(), energy_values.end(),
                  [](const double value) {
                    return std::isfinite(value) && value >= 0.0;
                  })) {
    return CapabilityEnergyResult::failure(
        {CapabilityEnergyFailure::current_invalid, 0U,
         "energy model or requirements contain a negative/non-finite value"});
  }
  if (text(energy_model_document, 7U) != inputs.operating_domain_id ||
      text(current_document, 13U) != inputs.operating_domain_id ||
      text(energy_state_document, 13U) != inputs.operating_domain_id ||
      inputs.operating_domain_id != context.configuration().operating_domain_id ||
      text(capability_document, 8U) != context.configuration().operating_domain_id) {
    return CapabilityEnergyResult::failure(
        {CapabilityEnergyFailure::current_invalid, 0U,
         "capability, current, energy, and configuration operating domains differ"});
  }

  CapabilityEnergyReport report{
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(), 0.0, 0.0, 0.0,
      inputs.capability_version, inputs.energy_model_version,
      inputs.energy_state_version};
  for (std::size_t index = 0U; index < trajectory.size(); ++index) {
    const auto& sample = trajectory[index];
    if (!finite(sample) || sample.duration_s <= 0.0) {
      return CapabilityEnergyResult::failure(
          {CapabilityEnergyFailure::invalid_trajectory, index,
           "trajectory sample is non-finite or has non-positive duration"});
    }
    if (!inside(sample.position_m, inputs.current_minimum,
                inputs.current_maximum)) {
      return CapabilityEnergyResult::failure(
          {CapabilityEnergyFailure::current_outside_region, index,
           "trajectory sample is outside current applicability region"});
    }
    const auto delta = Point3dEnu{sample.position_m.x_m - inputs.current_reference.x_m,
                                  sample.position_m.y_m - inputs.current_reference.y_m,
                                  sample.position_m.z_m - inputs.current_reference.z_m};
    Point3dEnu current = inputs.current_velocity;
    if (inputs.has_current_gradient) {
      current = {inputs.current_velocity.x_m +
                     inputs.current_gradient[0U] * delta.x_m +
                     inputs.current_gradient[1U] * delta.y_m +
                     inputs.current_gradient[2U] * delta.z_m,
                 inputs.current_velocity.y_m +
                     inputs.current_gradient[3U] * delta.x_m +
                     inputs.current_gradient[4U] * delta.y_m +
                     inputs.current_gradient[5U] * delta.z_m,
                 inputs.current_velocity.z_m +
                     inputs.current_gradient[6U] * delta.x_m +
                     inputs.current_gradient[7U] * delta.y_m +
                     inputs.current_gradient[8U] * delta.z_m};
    }
    const auto relative_velocity = Point3dEnu{
        sample.velocity_mps.x_m - current.x_m,
        sample.velocity_mps.y_m - current.y_m,
        sample.velocity_mps.z_m - current.z_m};
    const double cosine = std::cos(sample.yaw_rad);
    const double sine = std::sin(sample.yaw_rad);
    const auto water_velocity = Point3dEnu{
        cosine * relative_velocity.x_m + sine * relative_velocity.y_m,
        -sine * relative_velocity.x_m + cosine * relative_velocity.y_m,
        relative_velocity.z_m};
    const double water_speed_margin = inputs.max_water_speed - norm(water_velocity) -
                                      inputs.speed_error;
    Point3dEnu water_acceleration = sample.acceleration_mps2;
    if (inputs.has_current_gradient) {
      water_acceleration = {
          sample.acceleration_mps2.x_m -
              inputs.current_gradient[0U] * sample.velocity_mps.x_m -
              inputs.current_gradient[1U] * sample.velocity_mps.y_m -
              inputs.current_gradient[2U] * sample.velocity_mps.z_m,
          sample.acceleration_mps2.y_m -
              inputs.current_gradient[3U] * sample.velocity_mps.x_m -
              inputs.current_gradient[4U] * sample.velocity_mps.y_m -
              inputs.current_gradient[5U] * sample.velocity_mps.z_m,
          sample.acceleration_mps2.z_m -
              inputs.current_gradient[6U] * sample.velocity_mps.x_m -
              inputs.current_gradient[7U] * sample.velocity_mps.y_m -
              inputs.current_gradient[8U] * sample.velocity_mps.z_m};
      const auto rotated_acceleration = Point3dEnu{
          cosine * water_acceleration.x_m + sine * water_acceleration.y_m,
          -sine * water_acceleration.x_m + cosine * water_acceleration.y_m,
          water_acceleration.z_m};
      water_acceleration = {rotated_acceleration.x_m +
                                sample.yaw_rate_radps * water_velocity.y_m,
                            rotated_acceleration.y_m -
                                sample.yaw_rate_radps * water_velocity.x_m,
                            rotated_acceleration.z_m};
    } else {
      const auto rotated_acceleration = Point3dEnu{
          cosine * water_acceleration.x_m + sine * water_acceleration.y_m,
          -sine * water_acceleration.x_m + cosine * water_acceleration.y_m,
          water_acceleration.z_m};
      water_acceleration = rotated_acceleration;
    }
    const double acceleration_margin =
        inputs.max_acceleration - norm(water_acceleration) -
        (inputs.has_current_gradient
             ? 0.0
             : configuration.conservative_acceleration_margin_mps2);
    const double vertical_limit = sample.velocity_mps.z_m >= 0.0
                                      ? inputs.max_vertical_ascent
                                      : inputs.max_vertical_descent;
    const double vertical_margin = vertical_limit - std::abs(sample.velocity_mps.z_m);
    const double vertical_acceleration_margin =
        inputs.max_vertical_acceleration -
        std::abs(sample.acceleration_mps2.z_m);
    const double yaw_rate_margin =
        inputs.max_yaw_rate - std::abs(sample.yaw_rate_radps) -
        configuration.yaw_rate_error_radps;
    const double yaw_acceleration_margin =
        inputs.max_yaw_acceleration - std::abs(sample.yaw_acceleration_radps2) -
        configuration.yaw_acceleration_error_radps2;
    const double roll_margin = inputs.max_roll - std::abs(sample.roll_rad);
    const double pitch_margin = inputs.max_pitch - std::abs(sample.pitch_rad);
    const double capability_margin = std::min(
        {water_speed_margin, acceleration_margin, vertical_margin,
         vertical_acceleration_margin,
         yaw_rate_margin, yaw_acceleration_margin, roll_margin, pitch_margin});
    report.minimum_capability_margin =
        std::min(report.minimum_capability_margin, capability_margin);
    report.minimum_water_speed_margin_mps =
        std::min(report.minimum_water_speed_margin_mps, water_speed_margin);
    report.minimum_acceleration_margin_mps2 =
        std::min(report.minimum_acceleration_margin_mps2, acceleration_margin);
    report.minimum_vertical_speed_margin_mps =
        std::min(report.minimum_vertical_speed_margin_mps, vertical_margin);
    report.minimum_yaw_rate_margin_radps =
        std::min(report.minimum_yaw_rate_margin_radps, yaw_rate_margin);
    report.minimum_yaw_acceleration_margin_radps2 = std::min(
        report.minimum_yaw_acceleration_margin_radps2, yaw_acceleration_margin);
    if (capability_margin < 0.0) {
      return CapabilityEnergyResult::failure(
          {CapabilityEnergyFailure::capability_infeasible, index,
           "trajectory exceeds the calibrated capability envelope"});
    }
    if (!std::isfinite(capability_margin)) {
      return CapabilityEnergyResult::failure(
          {CapabilityEnergyFailure::capability_infeasible, index,
           "capability envelope produced a non-finite margin"});
    }
    const double speed = norm(water_velocity);
    const double acceleration = norm(water_acceleration);
    const double power = inputs.hotel_power + inputs.speed_power * speed +
                         inputs.cubic_power * speed * speed * speed +
                         inputs.acceleration_power * acceleration +
                         inputs.yaw_power * std::abs(sample.yaw_rate_radps) +
                         inputs.model_error_power;
    if (!std::isfinite(power) || power < 0.0) {
      return CapabilityEnergyResult::failure(
          {CapabilityEnergyFailure::current_invalid, index,
           "energy model produced a non-finite or negative power"});
    }
    report.estimated_plan_energy_j += power * sample.duration_s;
  }
  const double contingency = inputs.contingency == 1
                                 ? inputs.return_energy
                                 : inputs.contingency == 2 ? inputs.risk_energy : 0.0;
  report.required_energy_j = report.estimated_plan_energy_j + contingency +
                             inputs.reserve_energy;
  report.energy_margin_j = inputs.available_energy - report.required_energy_j;
  if (!std::isfinite(report.energy_margin_j) || report.energy_margin_j < 0.0) {
    return CapabilityEnergyResult::failure(
        {CapabilityEnergyFailure::energy_insufficient, trajectory.size() - 1U,
         "available energy does not cover plan, contingency, and reserve"});
  }
  return CapabilityEnergyResult::success(std::move(report));
}

}  // namespace scout_planner::core
