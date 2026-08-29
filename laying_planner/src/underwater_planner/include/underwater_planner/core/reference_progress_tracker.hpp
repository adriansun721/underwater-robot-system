#pragma once

#include "underwater_planner/core/data_contract.hpp"
#include "underwater_planner/core/parameter_config.hpp"
#include "underwater_planner/core/versioned_snapshot.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace underwater_planner::core {

class ReferenceProgressAssociationParameters {
 public:
  [[nodiscard]] double backward_tolerance_m() const noexcept;
  [[nodiscard]] double maximum_progress_per_laying_m() const noexcept;
  [[nodiscard]] double forward_slack_m() const noexcept;
  [[nodiscard]] double distance_scale_m() const noexcept;
  [[nodiscard]] double heading_scale_rad() const noexcept;
  [[nodiscard]] double heading_weight() const noexcept;
  [[nodiscard]] double association_score_tolerance() const noexcept;
  [[nodiscard]] const std::string& parameter_profile_id() const noexcept;
  [[nodiscard]] const std::string& operating_domain_id() const noexcept;

 private:
  double backward_tolerance_m_{};
  double maximum_progress_per_laying_m_{};
  double forward_slack_m_{};
  double distance_scale_m_{};
  double heading_scale_rad_{};
  double heading_weight_{};
  double association_score_tolerance_{};
  std::string parameter_profile_id_;
  std::string operating_domain_id_;

  friend ReferenceProgressAssociationParameters
  make_reference_progress_association_parameters(const ParameterConfig& config);
};

[[nodiscard]] ReferenceProgressAssociationParameters
make_reference_progress_association_parameters(const ParameterConfig& config);

struct TouchdownAssociationSample {
  double laying_arc_length_m{};
  Vector2m touchdown_position_m;
  double cable_heading_rad{};
  MonotonicTime timestamp;
};

struct ExecutedTouchdownSegment {
  std::uint64_t sequence_number{};
  std::vector<TouchdownAssociationSample> samples;
};

enum class ReferenceAssociationStatus {
  tracked,
  uninitialized,
  reference_version_changed,
  association_ambiguous,
  regression_requested,
  no_local_association,
  input_invalid,
  executed_segment_discontinuity,
};

[[nodiscard]] std::string_view to_string(ReferenceAssociationStatus status);

struct ReferenceProgressDiagnostic {
  ReferenceAssociationStatus status{ReferenceAssociationStatus::uninitialized};
  std::string message;
  std::uint32_t tracked_reference_line_version{};
  std::uint32_t supplied_reference_line_version{};
  std::uint64_t executed_segment_sequence{};
};

struct ReferenceProgressContext {
  std::uint32_t reference_line_version{};
  std::string parameter_profile_id;
  std::string operating_domain_id;
  ReferenceProgress progress;
  ReferencePoint reference_point;
  double signed_lateral_distance_m{};
  std::vector<ReferencePoint> local_corridor_centerline;
};

struct ReferenceAssociationResult {
  ReferenceAssociationStatus status{ReferenceAssociationStatus::uninitialized};
  std::optional<ReferenceProgressContext> context;
  std::vector<ReferenceProgressDiagnostic> diagnostics;
};

struct ReferenceProgressSnapshot {
  ReferenceAssociationStatus status{ReferenceAssociationStatus::uninitialized};
  std::optional<ReferenceProgress> progress;
  std::string parameter_profile_id;
  std::string operating_domain_id;
  std::vector<ReferenceProgressDiagnostic> diagnostics;

  [[nodiscard]] bool usable_for_planning() const noexcept {
    return status == ReferenceAssociationStatus::tracked && progress.has_value();
  }
};

class ReferenceProgressAssociator {
 public:
  explicit ReferenceProgressAssociator(
      ReferenceProgressAssociationParameters parameters);

  [[nodiscard]] ReferenceAssociationResult propagate_candidate(
      const ReferenceProgress& parent_progress,
      const TouchdownAssociationSample& terminal_touchdown,
      double primitive_length_m, const ReferenceLine& reference) const;

  [[nodiscard]] ReferenceAssociationResult query_local_context(
      const ReferenceProgress& progress, Vector2m touchdown_position_m,
      double corridor_half_window_m, const ReferenceLine& reference) const;

 private:
  ReferenceProgressAssociationParameters parameters_;
};

class ReferenceProgressTracker {
 public:
  explicit ReferenceProgressTracker(
      ReferenceProgressAssociationParameters parameters);

  [[nodiscard]] ReferenceProgressSnapshot reset_for_new_task(
      const ReferenceLine& reference, double initial_progress_m,
      MonotonicTime timestamp);

  [[nodiscard]] ReferenceProgressSnapshot update_from_executed_laying(
      const ExecutedTouchdownSegment& executed_touchdown,
      const ReferenceLine& reference);

  [[nodiscard]] ReferenceProgressSnapshot snapshot() const;

 private:
  ReferenceProgressAssociator associator_;
  ReferenceProgressSnapshot current_;
  std::optional<TouchdownAssociationSample> last_executed_sample_;
  std::uint64_t last_executed_segment_sequence_{};
};

}  // namespace underwater_planner::core
