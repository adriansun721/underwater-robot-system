#pragma once

#include "scout_planner/core/hybrid_map_query.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace scout_planner::core {

using BezierPositionControlPoints = std::array<Point3dEnu, 6U>;
using BezierYawControlPoints = std::array<double, 6U>;

struct QuinticBezierSegment4d {
  std::uint64_t start_time_offset_ns{};
  std::uint64_t duration_ns{};
  BezierPositionControlPoints position_control_points{};
  BezierYawControlPoints yaw_offset_control_points_rad{};
};

struct BezierDerivativeControlPoints4d {
  std::array<Point3dEnu, 5U> position_first{};
  std::array<Point3dEnu, 4U> position_second{};
  std::array<Point3dEnu, 3U> position_third{};
  std::array<double, 5U> yaw_first{};
  std::array<double, 4U> yaw_second{};
  std::array<double, 3U> yaw_third{};
};

struct BezierDerivativeBounds4d {
  double maximum_speed_mps{};
  double maximum_acceleration_mps2{};
  double maximum_jerk_mps3{};
  double maximum_yaw_rate_rps{};
  double maximum_yaw_acceleration_rps2{};
};

struct BezierSample4d {
  Point3dEnu position{};
  Point3dEnu velocity_mps{};
  Point3dEnu acceleration_mps2{};
  Point3dEnu jerk_mps3{};
  double yaw_rad{};
  double yaw_rate_rps{};
  double yaw_acceleration_rps2{};
};

enum class BezierErrorCode : std::uint8_t {
  invalid_frame,
  invalid_numeric,
  invalid_time,
  invalid_structure,
  discontinuous,
};

struct BezierError {
  BezierErrorCode code{BezierErrorCode::invalid_structure};
  std::size_t segment_index{};
  std::string detail;
};

template <typename T>
class BezierResult final {
 public:
  static BezierResult success(T value) { return BezierResult(std::move(value)); }
  static BezierResult failure(BezierError error) {
    return BezierResult(std::move(error));
  }
  [[nodiscard]] bool has_value() const noexcept {
    return std::holds_alternative<T>(storage_);
  }
  [[nodiscard]] const T& value() const { return std::get<T>(storage_); }
  [[nodiscard]] T& value() { return std::get<T>(storage_); }
  [[nodiscard]] const BezierError& error() const noexcept {
    static const BezierError empty{};
    const auto* error = std::get_if<BezierError>(&storage_);
    return error == nullptr ? empty : *error;
  }

 private:
  explicit BezierResult(T value) : storage_(std::move(value)) {}
  explicit BezierResult(BezierError error) : storage_(std::move(error)) {}
  std::variant<T, BezierError> storage_;
};

class QuinticBezierSegment final {
 public:
  [[nodiscard]] static BezierResult<QuinticBezierSegment> create(
      QuinticBezierSegment4d segment);

  [[nodiscard]] BezierResult<BezierSample4d> evaluate_time(
      std::uint64_t time_offset_ns) const;
  [[nodiscard]] BezierResult<BezierSample4d> evaluate_normalized(
      double normalized_time) const;
  [[nodiscard]] BezierResult<BezierSample4d> evaluate(double normalized_time) const {
    return evaluate_normalized(normalized_time);
  }
  [[nodiscard]] BezierDerivativeControlPoints4d derivative_control_points() const
      noexcept;
  [[nodiscard]] BezierDerivativeBounds4d derivative_bounds() const noexcept;
  [[nodiscard]] const QuinticBezierSegment4d& value() const noexcept {
    return segment_;
  }

 private:
  explicit QuinticBezierSegment(QuinticBezierSegment4d segment)
      : segment_(std::move(segment)) {}
  QuinticBezierSegment4d segment_;
};

class BezierTrajectory4d final {
 public:
  [[nodiscard]] static BezierResult<BezierTrajectory4d> create(
      std::string frame_id, double initial_yaw_rad,
      std::vector<QuinticBezierSegment4d> segments);

  [[nodiscard]] BezierResult<BezierSample4d> evaluate_time(
      std::uint64_t time_offset_ns) const;
  [[nodiscard]] BezierResult<BezierSample4d> evaluate(
      std::uint64_t time_offset_ns) const {
    return evaluate_time(time_offset_ns);
  }
  [[nodiscard]] BezierResult<bool> validate_c2_continuity() const;
  [[nodiscard]] const std::string& frame_id() const noexcept { return frame_id_; }
  [[nodiscard]] double initial_yaw_rad() const noexcept { return initial_yaw_rad_; }
  [[nodiscard]] const std::vector<QuinticBezierSegment4d>& segments() const noexcept {
    return segments_;
  }
  [[nodiscard]] const Hash256& content_hash() const noexcept { return content_hash_; }
  [[nodiscard]] std::uint64_t duration_ns() const noexcept { return duration_ns_; }

 private:
  BezierTrajectory4d(std::string frame_id, double initial_yaw_rad,
                     std::vector<QuinticBezierSegment4d> segments,
                     Hash256 content_hash, std::uint64_t duration_ns)
      : frame_id_(std::move(frame_id)), initial_yaw_rad_(initial_yaw_rad),
        segments_(std::move(segments)), content_hash_(content_hash),
        duration_ns_(duration_ns) {}
  std::string frame_id_;
  double initial_yaw_rad_{};
  std::vector<QuinticBezierSegment4d> segments_;
  Hash256 content_hash_{};
  std::uint64_t duration_ns_{};
};

}  // namespace scout_planner::core
