#include "underwater_planner/core/reference_progress_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace underwater_planner::core {
namespace {

constexpr double kPositionToleranceM = 1.0e-9;
constexpr double kArcLengthToleranceM = 1.0e-9;
constexpr double kAngleToleranceRad = 1.0e-12;

bool finite_vector(const Vector2m point) {
  return std::isfinite(point.x_m) && std::isfinite(point.y_m);
}

bool valid_sample(const TouchdownAssociationSample& sample) {
  return std::isfinite(sample.laying_arc_length_m) &&
         sample.laying_arc_length_m >= 0.0 &&
         finite_vector(sample.touchdown_position_m) &&
         std::isfinite(sample.cable_heading_rad) &&
         sample.cable_heading_rad >= -3.14159265358979323846 &&
         sample.cable_heading_rad < 3.14159265358979323846 &&
         sample.timestamp.nanoseconds >= 0;
}

bool valid_segment(const ExecutedTouchdownSegment& segment) {
  if (segment.sequence_number == 0U || segment.samples.size() < 2U) {
    return false;
  }
  for (std::size_t index = 0; index < segment.samples.size(); ++index) {
    if (!valid_sample(segment.samples[index])) return false;
    if (index > 0U &&
        (segment.samples[index].laying_arc_length_m <=
             segment.samples[index - 1U].laying_arc_length_m ||
         segment.samples[index].timestamp.nanoseconds <=
             segment.samples[index - 1U].timestamp.nanoseconds)) {
      return false;
    }
  }
  return true;
}

bool same_boundary(const TouchdownAssociationSample& left,
                   const TouchdownAssociationSample& right) {
  return std::abs(left.laying_arc_length_m - right.laying_arc_length_m) <=
             kArcLengthToleranceM &&
         std::hypot(left.touchdown_position_m.x_m -
                        right.touchdown_position_m.x_m,
                    left.touchdown_position_m.y_m -
                        right.touchdown_position_m.y_m) <=
             kPositionToleranceM &&
         std::abs(normalize_angle_radians(left.cable_heading_rad -
                                          right.cable_heading_rad)) <=
             kAngleToleranceRad &&
         left.timestamp.nanoseconds == right.timestamp.nanoseconds;
}

ReferenceProgressDiagnostic diagnostic(
    const ReferenceAssociationStatus status, std::string message,
    const std::uint32_t tracked_version, const std::uint32_t supplied_version,
    const std::uint64_t segment_sequence = 0U) {
  return {status, std::move(message), tracked_version, supplied_version,
          segment_sequence};
}

struct ProjectionCandidate {
  double progress_m{};
  double score{};
  double lateral_distance_m{};
};

double association_score(const ReferencePoint& point,
                         const TouchdownAssociationSample& touchdown,
                         const ReferenceProgressAssociationParameters& parameters,
                         double& lateral_distance_m) {
  const double dx = touchdown.touchdown_position_m.x_m - point.x_m;
  const double dy = touchdown.touchdown_position_m.y_m - point.y_m;
  lateral_distance_m = std::hypot(dx, dy);
  const double reference_heading = std::atan2(point.tangent_y, point.tangent_x);
  const double heading_error = normalize_angle_radians(
      touchdown.cable_heading_rad - reference_heading);
  const double scaled_distance =
      lateral_distance_m / parameters.distance_scale_m();
  const double scaled_heading =
      heading_error / parameters.heading_scale_rad();
  return scaled_distance * scaled_distance +
         parameters.heading_weight() * scaled_heading * scaled_heading;
}

std::vector<ProjectionCandidate> projection_candidates(
    const ReferenceLine& reference,
    const TouchdownAssociationSample& touchdown, const double lower_progress_m,
    const double upper_progress_m,
    const ReferenceProgressAssociationParameters& parameters) {
  std::vector<ProjectionCandidate> candidates;
  const std::vector<ReferenceProjection> projections =
      reference.local_projection_candidates(touchdown.touchdown_position_m,
                                            lower_progress_m,
                                            upper_progress_m);
  candidates.reserve(projections.size());
  for (const ReferenceProjection& projection : projections) {
    double lateral_distance_m{};
    const double score = association_score(projection.point, touchdown,
                                           parameters, lateral_distance_m);
    candidates.push_back(
        {projection.point.arc_length_m, score, lateral_distance_m});
  }
  return candidates;
}

ReferenceAssociationResult associate_in_window(
    const ReferenceProgressAssociationParameters& parameters,
    const ReferenceProgress& parent_progress,
    const TouchdownAssociationSample& touchdown, const double primitive_length_m,
    const ReferenceLine& reference) {
  if (!validate(parent_progress).valid || !valid_sample(touchdown) ||
      !std::isfinite(primitive_length_m) || primitive_length_m < 0.0 ||
      !validate(reference).valid) {
    return {ReferenceAssociationStatus::input_invalid, std::nullopt,
            {diagnostic(ReferenceAssociationStatus::input_invalid,
                        "reference association input is invalid",
                        parent_progress.reference_line_version,
                        reference.version)}};
  }
  if (parent_progress.reference_line_version != reference.version) {
    return {ReferenceAssociationStatus::reference_version_changed,
            std::nullopt,
            {diagnostic(
                ReferenceAssociationStatus::reference_version_changed,
                "progress and reference line versions do not match",
                parent_progress.reference_line_version, reference.version)}};
  }

  const double lower =
      std::max(reference.points.front().arc_length_m,
               parent_progress.arc_length_m -
                   parameters.backward_tolerance_m());
  const double upper =
      std::min(reference.points.back().arc_length_m,
               parent_progress.arc_length_m +
                   parameters.maximum_progress_per_laying_m() *
                       primitive_length_m +
                   parameters.forward_slack_m());
  std::vector<ProjectionCandidate> candidates = projection_candidates(
      reference, touchdown, lower, upper, parameters);
  if (candidates.empty()) {
    return {ReferenceAssociationStatus::no_local_association, std::nullopt,
            {diagnostic(ReferenceAssociationStatus::no_local_association,
                        "no reference association exists in the local progress window",
                        parent_progress.reference_line_version,
                        reference.version)}};
  }
  std::stable_sort(candidates.begin(), candidates.end(),
                   [](const ProjectionCandidate& left,
                      const ProjectionCandidate& right) {
                     if (left.score != right.score) {
                       return left.score < right.score;
                     }
                     return left.progress_m < right.progress_m;
                   });
  const ProjectionCandidate& best = candidates.front();
  for (std::size_t index = 1U; index < candidates.size(); ++index) {
    if (candidates[index].score - best.score >
        parameters.association_score_tolerance()) {
      break;
    }
    if (std::abs(candidates[index].progress_m - best.progress_m) >
        kArcLengthToleranceM) {
      return {ReferenceAssociationStatus::association_ambiguous, std::nullopt,
              {diagnostic(ReferenceAssociationStatus::association_ambiguous,
                          "multiple task stages have equivalent local association scores",
                          parent_progress.reference_line_version,
                          reference.version)}};
    }
  }
  if (best.progress_m + kArcLengthToleranceM <
      parent_progress.arc_length_m) {
    return {ReferenceAssociationStatus::regression_requested, std::nullopt,
            {diagnostic(ReferenceAssociationStatus::regression_requested,
                        "local association requested a task-progress regression",
                        parent_progress.reference_line_version,
                        reference.version)}};
  }

  ReferenceProgress associated{reference.version, best.progress_m,
                               touchdown.timestamp,
                               parent_progress.sequence_number + 1U};
  const std::optional<ReferencePoint> point = reference.query(best.progress_m);
  if (!point.has_value()) {
    return {ReferenceAssociationStatus::no_local_association, std::nullopt,
            {diagnostic(ReferenceAssociationStatus::no_local_association,
                        "associated progress cannot be queried from the reference line",
                        parent_progress.reference_line_version,
                        reference.version)}};
  }
  const double signed_lateral =
      (touchdown.touchdown_position_m.x_m - point->x_m) * point->normal_x +
      (touchdown.touchdown_position_m.y_m - point->y_m) * point->normal_y;
  const double half_window = std::max(
      parameters.backward_tolerance_m(),
      parameters.maximum_progress_per_laying_m() * primitive_length_m +
          parameters.forward_slack_m());
  ReferenceProgressContext context{
      reference.version, parameters.parameter_profile_id(),
      parameters.operating_domain_id(), associated, *point, signed_lateral,
      reference.local_window(best.progress_m, half_window)};
  return {ReferenceAssociationStatus::tracked, std::move(context), {}};
}

}  // namespace

double ReferenceProgressAssociationParameters::backward_tolerance_m() const
    noexcept {
  return backward_tolerance_m_;
}

double ReferenceProgressAssociationParameters::maximum_progress_per_laying_m()
    const noexcept {
  return maximum_progress_per_laying_m_;
}

double ReferenceProgressAssociationParameters::forward_slack_m() const
    noexcept {
  return forward_slack_m_;
}

double ReferenceProgressAssociationParameters::distance_scale_m() const
    noexcept {
  return distance_scale_m_;
}

double ReferenceProgressAssociationParameters::heading_scale_rad() const
    noexcept {
  return heading_scale_rad_;
}

double ReferenceProgressAssociationParameters::heading_weight() const
    noexcept {
  return heading_weight_;
}

double ReferenceProgressAssociationParameters::association_score_tolerance()
    const noexcept {
  return association_score_tolerance_;
}

const std::string&
ReferenceProgressAssociationParameters::parameter_profile_id() const noexcept {
  return parameter_profile_id_;
}

const std::string&
ReferenceProgressAssociationParameters::operating_domain_id() const noexcept {
  return operating_domain_id_;
}

ReferenceProgressAssociationParameters
make_reference_progress_association_parameters(const ParameterConfig& config) {
  const SearchParameterConfig& search = config.search;
  const bool valid =
      config.schema_version == "parameter-config/v1" &&
      !config.profile_id.empty() && !config.operating_domain_id.empty() &&
      search.reference_progress_backward_tolerance_m.has_value() &&
      std::isfinite(*search.reference_progress_backward_tolerance_m) &&
      *search.reference_progress_backward_tolerance_m >= 0.0 &&
      search.reference_progress_maximum_ratio.has_value() &&
      std::isfinite(*search.reference_progress_maximum_ratio) &&
      *search.reference_progress_maximum_ratio > 0.0 &&
      search.reference_progress_forward_slack_m.has_value() &&
      std::isfinite(*search.reference_progress_forward_slack_m) &&
      *search.reference_progress_forward_slack_m >= 0.0 &&
      search.reference_progress_distance_scale_m.has_value() &&
      std::isfinite(*search.reference_progress_distance_scale_m) &&
      *search.reference_progress_distance_scale_m > 0.0 &&
      search.reference_progress_heading_scale_rad.has_value() &&
      std::isfinite(*search.reference_progress_heading_scale_rad) &&
      *search.reference_progress_heading_scale_rad > 0.0 &&
      search.reference_progress_heading_weight.has_value() &&
      std::isfinite(*search.reference_progress_heading_weight) &&
      *search.reference_progress_heading_weight > 0.0 &&
      search.reference_progress_association_score_tolerance.has_value() &&
      std::isfinite(
          *search.reference_progress_association_score_tolerance) &&
      *search.reference_progress_association_score_tolerance >= 0.0;
  if (!valid) {
    throw std::invalid_argument(
        "reference progress association parameters must come from a valid "
        "versioned parameter profile");
  }

  ReferenceProgressAssociationParameters parameters;
  parameters.backward_tolerance_m_ =
      *search.reference_progress_backward_tolerance_m;
  parameters.maximum_progress_per_laying_m_ =
      *search.reference_progress_maximum_ratio;
  parameters.forward_slack_m_ =
      *search.reference_progress_forward_slack_m;
  parameters.distance_scale_m_ =
      *search.reference_progress_distance_scale_m;
  parameters.heading_scale_rad_ =
      *search.reference_progress_heading_scale_rad;
  parameters.heading_weight_ = *search.reference_progress_heading_weight;
  parameters.association_score_tolerance_ =
      *search.reference_progress_association_score_tolerance;
  parameters.parameter_profile_id_ = config.profile_id;
  parameters.operating_domain_id_ = config.operating_domain_id;
  return parameters;
}

std::string_view to_string(const ReferenceAssociationStatus status) {
  switch (status) {
    case ReferenceAssociationStatus::tracked:
      return "TRACKED";
    case ReferenceAssociationStatus::uninitialized:
      return "UNINITIALIZED";
    case ReferenceAssociationStatus::reference_version_changed:
      return "REFERENCE_VERSION_CHANGED";
    case ReferenceAssociationStatus::association_ambiguous:
      return "ASSOCIATION_AMBIGUOUS";
    case ReferenceAssociationStatus::regression_requested:
      return "REGRESSION_REQUESTED";
    case ReferenceAssociationStatus::no_local_association:
      return "NO_LOCAL_ASSOCIATION";
    case ReferenceAssociationStatus::input_invalid:
      return "INPUT_INVALID";
    case ReferenceAssociationStatus::executed_segment_discontinuity:
      return "EXECUTED_SEGMENT_DISCONTINUITY";
  }
  return "UNKNOWN";
}

ReferenceProgressAssociator::ReferenceProgressAssociator(
    ReferenceProgressAssociationParameters parameters)
    : parameters_(std::move(parameters)) {}

ReferenceProgressTracker::ReferenceProgressTracker(
    ReferenceProgressAssociationParameters parameters)
    : associator_(parameters) {
  current_.parameter_profile_id = parameters.parameter_profile_id();
  current_.operating_domain_id = parameters.operating_domain_id();
  current_.diagnostics = {diagnostic(
      ReferenceAssociationStatus::uninitialized,
      "reference progress requires an explicit task start", 0U, 0U)};
}

ReferenceProgressSnapshot ReferenceProgressTracker::reset_for_new_task(
    const ReferenceLine& reference, const double initial_progress_m,
    const MonotonicTime timestamp) {
  if (!validate(reference).valid || !std::isfinite(initial_progress_m) ||
      initial_progress_m < reference.points.front().arc_length_m ||
      initial_progress_m > reference.points.back().arc_length_m ||
      timestamp.nanoseconds < 0) {
    current_.status = ReferenceAssociationStatus::input_invalid;
    current_.diagnostics = {diagnostic(
        ReferenceAssociationStatus::input_invalid,
        "task reset requires a valid reference, progress, and timestamp",
        current_.progress.has_value()
            ? current_.progress->reference_line_version
            : 0U,
        reference.version)};
    return current_;
  }
  current_.status = ReferenceAssociationStatus::tracked;
  current_.progress =
      ReferenceProgress{reference.version, initial_progress_m, timestamp, 1U};
  current_.diagnostics.clear();
  last_executed_sample_.reset();
  last_executed_segment_sequence_ = 0U;
  return current_;
}

ReferenceProgressSnapshot ReferenceProgressTracker::update_from_executed_laying(
    const ExecutedTouchdownSegment& segment,
    const ReferenceLine& reference) {
  if (!current_.progress.has_value()) {
    current_.status = ReferenceAssociationStatus::uninitialized;
    current_.diagnostics = {diagnostic(
        ReferenceAssociationStatus::uninitialized,
        "executed laying cannot advance an uninitialized task", 0U,
        reference.version, segment.sequence_number)};
    return current_;
  }
  if (current_.progress->reference_line_version != reference.version) {
    current_.status = ReferenceAssociationStatus::reference_version_changed;
    current_.diagnostics = {diagnostic(
        ReferenceAssociationStatus::reference_version_changed,
        "reference version changed without explicit reset or migration",
        current_.progress->reference_line_version, reference.version,
        segment.sequence_number)};
    return current_;
  }
  if (!validate(reference).valid || !valid_segment(segment) ||
      segment.sequence_number <= last_executed_segment_sequence_ ||
      segment.samples.front().timestamp.nanoseconds <
          current_.progress->timestamp.nanoseconds) {
    current_.status = ReferenceAssociationStatus::input_invalid;
    current_.diagnostics = {diagnostic(
        ReferenceAssociationStatus::input_invalid,
        "executed touchdown segment is invalid, duplicate, or out of order",
        current_.progress->reference_line_version, reference.version,
        segment.sequence_number)};
    return current_;
  }
  if (last_executed_sample_.has_value() &&
      !same_boundary(*last_executed_sample_, segment.samples.front())) {
    current_.status =
        ReferenceAssociationStatus::executed_segment_discontinuity;
    current_.diagnostics = {diagnostic(
        ReferenceAssociationStatus::executed_segment_discontinuity,
        "executed touchdown segment does not continue the accepted boundary",
        current_.progress->reference_line_version, reference.version,
        segment.sequence_number)};
    return current_;
  }

  ReferenceProgress working = *current_.progress;
  for (std::size_t index = 1U; index < segment.samples.size(); ++index) {
    const double laying_increment_m =
        segment.samples[index].laying_arc_length_m -
        segment.samples[index - 1U].laying_arc_length_m;
    const ReferenceAssociationResult association = associator_.propagate_candidate(
        working, segment.samples[index], laying_increment_m, reference);
    if (association.status != ReferenceAssociationStatus::tracked ||
        !association.context.has_value()) {
      current_.status = association.status;
      current_.diagnostics = association.diagnostics;
      if (!current_.diagnostics.empty()) {
        current_.diagnostics.front().executed_segment_sequence =
            segment.sequence_number;
      }
      return current_;
    }
    if (association.context->progress.arc_length_m + kArcLengthToleranceM <
        working.arc_length_m) {
      current_.status = ReferenceAssociationStatus::regression_requested;
      current_.diagnostics = {diagnostic(
          ReferenceAssociationStatus::regression_requested,
          "executed laying requested a task-progress regression",
          working.reference_line_version, reference.version,
          segment.sequence_number)};
      return current_;
    }
    working.arc_length_m =
        std::max(working.arc_length_m,
                 association.context->progress.arc_length_m);
  }

  working.timestamp = segment.samples.back().timestamp;
  working.sequence_number = current_.progress->sequence_number + 1U;
  current_.status = ReferenceAssociationStatus::tracked;
  current_.progress = working;
  current_.diagnostics.clear();
  last_executed_sample_ = segment.samples.back();
  last_executed_segment_sequence_ = segment.sequence_number;
  return current_;
}

ReferenceAssociationResult ReferenceProgressAssociator::propagate_candidate(
    const ReferenceProgress& parent_progress,
    const TouchdownAssociationSample& terminal_touchdown,
    const double primitive_length_m, const ReferenceLine& reference) const {
  return associate_in_window(parameters_, parent_progress, terminal_touchdown,
                             primitive_length_m, reference);
}

ReferenceAssociationResult ReferenceProgressAssociator::query_local_context(
    const ReferenceProgress& progress, const Vector2m touchdown_position_m,
    const double corridor_half_window_m,
    const ReferenceLine& reference) const {
  if (!validate(progress).valid || !validate(reference).valid ||
      !finite_vector(touchdown_position_m) ||
      !std::isfinite(corridor_half_window_m) ||
      corridor_half_window_m < 0.0) {
    return {ReferenceAssociationStatus::input_invalid, std::nullopt,
            {diagnostic(ReferenceAssociationStatus::input_invalid,
                        "local reference context query input is invalid",
                        progress.reference_line_version, reference.version)}};
  }
  if (progress.reference_line_version != reference.version) {
    return {ReferenceAssociationStatus::reference_version_changed,
            std::nullopt,
            {diagnostic(
                ReferenceAssociationStatus::reference_version_changed,
                "local context progress and reference versions do not match",
                progress.reference_line_version, reference.version)}};
  }
  const std::optional<ReferencePoint> point =
      reference.query(progress.arc_length_m);
  if (!point.has_value()) {
    return {ReferenceAssociationStatus::input_invalid, std::nullopt,
            {diagnostic(ReferenceAssociationStatus::input_invalid,
                        "progress is outside the supplied reference line",
                        progress.reference_line_version, reference.version)}};
  }
  const double signed_lateral =
      (touchdown_position_m.x_m - point->x_m) * point->normal_x +
      (touchdown_position_m.y_m - point->y_m) * point->normal_y;
  ReferenceProgressContext context{
      reference.version,
      parameters_.parameter_profile_id(),
      parameters_.operating_domain_id(),
      progress,
      *point,
      signed_lateral,
      reference.local_window(progress.arc_length_m, corridor_half_window_m)};
  return {ReferenceAssociationStatus::tracked, std::move(context), {}};
}

ReferenceProgressSnapshot ReferenceProgressTracker::snapshot() const {
  return current_;
}

}  // namespace underwater_planner::core
