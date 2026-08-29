#include "scout_planner/core/quintic_bezier.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

namespace scout_planner::core {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr long double kTolerance = 1.0e-9L;

bool finite(const Point3dEnu& point) noexcept {
  return std::isfinite(point.x_m) && std::isfinite(point.y_m) &&
         std::isfinite(point.z_m);
}

Point3dEnu add(const Point3dEnu& a, const Point3dEnu& b) noexcept {
  return {a.x_m + b.x_m, a.y_m + b.y_m, a.z_m + b.z_m};
}
Point3dEnu subtract(const Point3dEnu& a, const Point3dEnu& b) noexcept {
  return {a.x_m - b.x_m, a.y_m - b.y_m, a.z_m - b.z_m};
}
Point3dEnu scale(const Point3dEnu& a, const double factor) noexcept {
  return {a.x_m * factor, a.y_m * factor, a.z_m * factor};
}
double norm(const Point3dEnu& a) noexcept {
  return std::sqrt(a.x_m * a.x_m + a.y_m * a.y_m + a.z_m * a.z_m);
}

double close(const double a, const double b) noexcept {
  return std::abs(static_cast<long double>(a) - static_cast<long double>(b)) <=
         kTolerance * std::max({1.0L, std::abs(static_cast<long double>(a)),
                                std::abs(static_cast<long double>(b))});
}

double normalize_yaw(double yaw) noexcept {
  yaw = std::fmod(yaw + kPi, 2.0 * kPi);
  if (yaw < 0.0) yaw += 2.0 * kPi;
  return yaw - kPi;
}

template <std::size_t N>
double scalar_bezier(const std::array<double, N>& points, const double s) noexcept {
  std::array<double, N> work = points;
  for (std::size_t order = 1U; order < N; ++order) {
    for (std::size_t i = 0U; i + order < N; ++i) {
      work[i] = (1.0 - s) * work[i] + s * work[i + 1U];
    }
  }
  return work[0];
}

template <std::size_t N>
Point3dEnu point_bezier(const std::array<Point3dEnu, N>& points,
                        const double s) noexcept {
  std::array<Point3dEnu, N> work = points;
  for (std::size_t order = 1U; order < N; ++order) {
    for (std::size_t i = 0U; i + order < N; ++i) {
      work[i] = add(scale(work[i], 1.0 - s), scale(work[i + 1U], s));
    }
  }
  return work[0];
}

// Small self-contained SHA-256 implementation keeps the identity independent
// of platform libraries and protobuf serialization details.
class Sha256 final {
 public:
  Sha256() : state_{0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
                    0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U} {}
  void update(const std::uint8_t* data, std::size_t size) {
    for (std::size_t i = 0U; i < size; ++i) append_byte(data[i]);
  }
  Hash256 finish() {
    const auto original_bits = bit_length_ + length_ * 8U;
    append_byte(0x80U);
    while (length_ != 56U) append_byte(0U);
    for (int shift = 56; shift >= 0; shift -= 8)
      append_byte(static_cast<std::uint8_t>(original_bits >> shift));
    Hash256 output{};
    for (std::size_t i = 0U; i < state_.size(); ++i) {
      output[i * 4U] = static_cast<std::uint8_t>(state_[i] >> 24U);
      output[i * 4U + 1U] = static_cast<std::uint8_t>(state_[i] >> 16U);
      output[i * 4U + 2U] = static_cast<std::uint8_t>(state_[i] >> 8U);
      output[i * 4U + 3U] = static_cast<std::uint8_t>(state_[i]);
    }
    return output;
  }

 private:
  void append_byte(std::uint8_t value) {
    buffer_[length_] = value;
    ++length_;
    if (length_ == 64U) {
      transform();
      bit_length_ += 512U;
      length_ = 0U;
    }
  }

  static constexpr std::array<std::uint32_t, 64U> k = {
      0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU,
      0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U, 0xd807aa98U, 0x12835b01U,
      0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U,
      0xc19bf174U, 0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
      0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU, 0x983e5152U,
      0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U,
      0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU,
      0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
      0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U,
      0xd6990624U, 0xf40e3585U, 0x106aa070U, 0x19a4c116U, 0x1e376c08U,
      0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU,
      0x682e6ff3U, 0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
      0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};
  static std::uint32_t rotr(std::uint32_t x, std::uint32_t n) noexcept {
    return (x >> n) | (x << (32U - n));
  }
  void transform() noexcept {
    std::array<std::uint32_t, 64U> w{};
    for (std::size_t i = 0U; i < 16U; ++i) {
      w[i] = (static_cast<std::uint32_t>(buffer_[i * 4U]) << 24U) |
             (static_cast<std::uint32_t>(buffer_[i * 4U + 1U]) << 16U) |
             (static_cast<std::uint32_t>(buffer_[i * 4U + 2U]) << 8U) |
             static_cast<std::uint32_t>(buffer_[i * 4U + 3U]);
    }
    for (std::size_t i = 16U; i < 64U; ++i) {
      const auto s0 = rotr(w[i - 15U], 7U) ^ rotr(w[i - 15U], 18U) ^
                      (w[i - 15U] >> 3U);
      const auto s1 = rotr(w[i - 2U], 17U) ^ rotr(w[i - 2U], 19U) ^
                      (w[i - 2U] >> 10U);
      w[i] = w[i - 16U] + s0 + w[i - 7U] + s1;
    }
    auto a = state_[0]; auto b = state_[1]; auto c = state_[2]; auto d = state_[3];
    auto e = state_[4]; auto f = state_[5]; auto g = state_[6]; auto h = state_[7];
    for (std::size_t i = 0U; i < 64U; ++i) {
      const auto S1 = rotr(e, 6U) ^ rotr(e, 11U) ^ rotr(e, 25U);
      const auto ch = (e & f) ^ ((~e) & g);
      const auto temp1 = h + S1 + ch + k[i] + w[i];
      const auto S0 = rotr(a, 2U) ^ rotr(a, 13U) ^ rotr(a, 22U);
      const auto maj = (a & b) ^ (a & c) ^ (b & c);
      const auto temp2 = S0 + maj;
      h = g; g = f; f = e; e = d + temp1; d = c; c = b; b = a; a = temp1 + temp2;
    }
    state_[0] += a; state_[1] += b; state_[2] += c; state_[3] += d;
    state_[4] += e; state_[5] += f; state_[6] += g; state_[7] += h;
  }
  std::array<std::uint32_t, 8U> state_{};
  std::array<std::uint8_t, 64U> buffer_{};
  std::size_t length_{};
  std::uint64_t bit_length_{};
};

void append_u64(std::vector<std::uint8_t>& bytes, std::uint64_t value) {
  for (unsigned shift = 0U; shift < 64U; shift += 8U)
    bytes.push_back(static_cast<std::uint8_t>(value >> shift));
}
void append_double(std::vector<std::uint8_t>& bytes, double value) {
  if (value == 0.0) value = 0.0;
  std::uint64_t bits{};
  static_assert(sizeof(bits) == sizeof(value), "unexpected double width");
  std::memcpy(&bits, &value, sizeof(bits));
  append_u64(bytes, bits);
}

Hash256 identity_for(const std::string& frame, double yaw,
                     const std::vector<QuinticBezierSegment4d>& segments) {
  std::vector<std::uint8_t> bytes;
  append_u64(bytes, frame.size());
  bytes.insert(bytes.end(), frame.begin(), frame.end());
  append_double(bytes, yaw);
  append_u64(bytes, segments.size());
  for (const auto& segment : segments) {
    append_u64(bytes, segment.start_time_offset_ns);
    append_u64(bytes, segment.duration_ns);
    for (const auto& point : segment.position_control_points) {
      append_double(bytes, point.x_m); append_double(bytes, point.y_m); append_double(bytes, point.z_m);
    }
    for (const auto value : segment.yaw_offset_control_points_rad) append_double(bytes, value);
  }
  Sha256 sha;
  sha.update(bytes.data(), bytes.size());
  return sha.finish();
}

BezierError error(BezierErrorCode code, std::size_t index, std::string detail) {
  return {code, index, std::move(detail)};
}

}  // namespace

BezierResult<QuinticBezierSegment> QuinticBezierSegment::create(
    QuinticBezierSegment4d segment) {
  if (segment.duration_ns == 0U) return BezierResult<QuinticBezierSegment>::failure(
      error(BezierErrorCode::invalid_time, 0U, "segment duration must be positive"));
  for (const auto& point : segment.position_control_points) {
    if (!finite(point)) return BezierResult<QuinticBezierSegment>::failure(
        error(BezierErrorCode::invalid_numeric, 0U, "position control point is non-finite"));
  }
  for (const auto value : segment.yaw_offset_control_points_rad) {
    if (!std::isfinite(value)) return BezierResult<QuinticBezierSegment>::failure(
        error(BezierErrorCode::invalid_numeric, 0U, "yaw control point is non-finite"));
  }
  return BezierResult<QuinticBezierSegment>::success(
      QuinticBezierSegment(std::move(segment)));
}

BezierResult<BezierSample4d> QuinticBezierSegment::evaluate_normalized(
    double s) const {
  if (!std::isfinite(s) || s < 0.0 || s > 1.0)
    return BezierResult<BezierSample4d>::failure(error(
        BezierErrorCode::invalid_time, 0U, "normalized time must be in [0, 1]"));
  const auto derivatives = derivative_control_points();
  BezierSample4d result{};
  result.position = point_bezier(segment_.position_control_points, s);
  result.velocity_mps = point_bezier(derivatives.position_first, s);
  result.acceleration_mps2 = point_bezier(derivatives.position_second, s);
  result.jerk_mps3 = point_bezier(derivatives.position_third, s);
  result.yaw_rad = scalar_bezier(segment_.yaw_offset_control_points_rad, s);
  result.yaw_rate_rps = scalar_bezier(derivatives.yaw_first, s);
  result.yaw_acceleration_rps2 = scalar_bezier(derivatives.yaw_second, s);
  return BezierResult<BezierSample4d>::success(result);
}

BezierResult<BezierSample4d> QuinticBezierSegment::evaluate_time(
    std::uint64_t time_offset_ns) const {
  if (time_offset_ns > segment_.duration_ns)
    return BezierResult<BezierSample4d>::failure(error(
        BezierErrorCode::invalid_time, 0U, "time is outside segment interval"));
  const double s = static_cast<double>(time_offset_ns) /
                   static_cast<double>(segment_.duration_ns);
  return evaluate_normalized(s);
}

BezierDerivativeControlPoints4d QuinticBezierSegment::derivative_control_points() const
    noexcept {
  const double duration_s = static_cast<double>(segment_.duration_ns) / 1.0e9;
  const double first = 5.0 / duration_s;
  const double second = 20.0 / (duration_s * duration_s);
  const double third = 60.0 / (duration_s * duration_s * duration_s);
  BezierDerivativeControlPoints4d result{};
  for (std::size_t i = 0U; i < 5U; ++i)
    result.position_first[i] = scale(subtract(segment_.position_control_points[i + 1U], segment_.position_control_points[i]), first);
  for (std::size_t i = 0U; i < 4U; ++i)
    result.position_second[i] = scale(add(subtract(segment_.position_control_points[i + 2U], scale(segment_.position_control_points[i + 1U], 2.0)), segment_.position_control_points[i]), second);
  for (std::size_t i = 0U; i < 3U; ++i)
    result.position_third[i] = scale(add(subtract(segment_.position_control_points[i + 3U], scale(segment_.position_control_points[i + 2U], 3.0)), add(scale(segment_.position_control_points[i + 1U], 3.0), scale(segment_.position_control_points[i], -1.0))), third);
  for (std::size_t i = 0U; i < 5U; ++i) result.yaw_first[i] = first * (segment_.yaw_offset_control_points_rad[i + 1U] - segment_.yaw_offset_control_points_rad[i]);
  for (std::size_t i = 0U; i < 4U; ++i) result.yaw_second[i] = second * (segment_.yaw_offset_control_points_rad[i + 2U] - 2.0 * segment_.yaw_offset_control_points_rad[i + 1U] + segment_.yaw_offset_control_points_rad[i]);
  for (std::size_t i = 0U; i < 3U; ++i) result.yaw_third[i] = third * (segment_.yaw_offset_control_points_rad[i + 3U] - 3.0 * segment_.yaw_offset_control_points_rad[i + 2U] + 3.0 * segment_.yaw_offset_control_points_rad[i + 1U] - segment_.yaw_offset_control_points_rad[i]);
  return result;
}

BezierDerivativeBounds4d QuinticBezierSegment::derivative_bounds() const noexcept {
  const auto derivatives = derivative_control_points();
  BezierDerivativeBounds4d result{};
  for (const auto& point : derivatives.position_first) result.maximum_speed_mps = std::max(result.maximum_speed_mps, norm(point));
  for (const auto& point : derivatives.position_second) result.maximum_acceleration_mps2 = std::max(result.maximum_acceleration_mps2, norm(point));
  for (const auto& point : derivatives.position_third) result.maximum_jerk_mps3 = std::max(result.maximum_jerk_mps3, norm(point));
  for (const auto value : derivatives.yaw_first) result.maximum_yaw_rate_rps = std::max(result.maximum_yaw_rate_rps, std::abs(value));
  for (const auto value : derivatives.yaw_second) result.maximum_yaw_acceleration_rps2 = std::max(result.maximum_yaw_acceleration_rps2, std::abs(value));
  return result;
}

BezierResult<BezierTrajectory4d> BezierTrajectory4d::create(
    std::string frame_id, double initial_yaw_rad,
    std::vector<QuinticBezierSegment4d> segments) {
  if (frame_id != "mission_enu") return BezierResult<BezierTrajectory4d>::failure(
      error(BezierErrorCode::invalid_frame, 0U, "trajectory frame must be mission_enu"));
  if (!std::isfinite(initial_yaw_rad)) return BezierResult<BezierTrajectory4d>::failure(
      error(BezierErrorCode::invalid_numeric, 0U, "initial yaw is non-finite"));
  if (segments.empty()) return BezierResult<BezierTrajectory4d>::failure(
      error(BezierErrorCode::invalid_structure, 0U, "trajectory must contain a segment"));
  if (!close(segments.front().yaw_offset_control_points_rad[0], 0.0))
    return BezierResult<BezierTrajectory4d>::failure(error(
        BezierErrorCode::invalid_structure, 0U,
        "the first trajectory yaw offset must be zero"));
  std::uint64_t expected_start = 0U;
  for (std::size_t i = 0U; i < segments.size(); ++i) {
    auto checked = QuinticBezierSegment::create(segments[i]);
    if (!checked.has_value()) {
      auto issue = checked.error(); issue.segment_index = i;
      return BezierResult<BezierTrajectory4d>::failure(std::move(issue));
    }
    if (segments[i].start_time_offset_ns != expected_start ||
        segments[i].duration_ns > std::numeric_limits<std::uint64_t>::max() - expected_start)
      return BezierResult<BezierTrajectory4d>::failure(error(
          BezierErrorCode::invalid_time, i, "segments must be contiguous and non-overflowing"));
    expected_start += segments[i].duration_ns;
  }
  BezierTrajectory4d trajectory(std::move(frame_id), normalize_yaw(initial_yaw_rad),
                               std::move(segments), {}, expected_start);
  auto continuity = trajectory.validate_c2_continuity();
  if (!continuity.has_value()) return BezierResult<BezierTrajectory4d>::failure(continuity.error());
  trajectory.content_hash_ = identity_for(trajectory.frame_id_, trajectory.initial_yaw_rad_, trajectory.segments_);
  return BezierResult<BezierTrajectory4d>::success(std::move(trajectory));
}

BezierResult<BezierSample4d> BezierTrajectory4d::evaluate_time(
    std::uint64_t time_offset_ns) const {
  if (time_offset_ns > duration_ns_) return BezierResult<BezierSample4d>::failure(error(
      BezierErrorCode::invalid_time, segments_.size() - 1U, "time is outside trajectory interval"));
  std::size_t selected = segments_.size() - 1U;
  for (std::size_t i = 0U; i + 1U < segments_.size(); ++i) {
    if (time_offset_ns < segments_[i + 1U].start_time_offset_ns) { selected = i; break; }
  }
  const auto& segment = segments_[selected];
  const auto local = time_offset_ns - segment.start_time_offset_ns;
  auto result = QuinticBezierSegment::create(segment).value().evaluate_time(local);
  if (result.has_value()) result.value().yaw_rad += initial_yaw_rad_;
  return result;
}

BezierResult<bool> BezierTrajectory4d::validate_c2_continuity() const {
  for (std::size_t i = 1U; i < segments_.size(); ++i) {
    const auto previous = QuinticBezierSegment::create(segments_[i - 1U]).value();
    const auto current = QuinticBezierSegment::create(segments_[i]).value();
    const auto left = previous.evaluate_normalized(1.0).value();
    const auto right = current.evaluate_normalized(0.0).value();
    if (!close(left.position.x_m, right.position.x_m) || !close(left.position.y_m, right.position.y_m) || !close(left.position.z_m, right.position.z_m) ||
        !close(left.velocity_mps.x_m, right.velocity_mps.x_m) || !close(left.velocity_mps.y_m, right.velocity_mps.y_m) || !close(left.velocity_mps.z_m, right.velocity_mps.z_m) ||
        !close(left.acceleration_mps2.x_m, right.acceleration_mps2.x_m) || !close(left.acceleration_mps2.y_m, right.acceleration_mps2.y_m) || !close(left.acceleration_mps2.z_m, right.acceleration_mps2.z_m) ||
        !close(left.yaw_rad, right.yaw_rad) || !close(left.yaw_rate_rps, right.yaw_rate_rps) || !close(left.yaw_acceleration_rps2, right.yaw_acceleration_rps2))
      return BezierResult<bool>::failure(error(BezierErrorCode::discontinuous, i, "adjacent segments are not C2 continuous"));
  }
  return BezierResult<bool>::success(true);
}

}  // namespace scout_planner::core
