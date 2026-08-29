#include "underwater_planner/core/reference_progress_tracker.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using underwater_planner::core::ExecutedTouchdownSegment;
using underwater_planner::core::MonotonicTime;
using underwater_planner::core::ParameterConfig;
using underwater_planner::core::ReferenceAssociationStatus;
using underwater_planner::core::ReferenceLine;
using underwater_planner::core::ReferenceProgress;
using underwater_planner::core::ReferenceProgressAssociationParameters;
using underwater_planner::core::ReferenceProgressAssociator;
using underwater_planner::core::ReferenceProgressTracker;
using underwater_planner::core::TouchdownAssociationSample;
using underwater_planner::core::Vector2m;
using underwater_planner::core::make_reference_progress_association_parameters;
using underwater_planner::core::make_reference_line;

constexpr double kPi = 3.14159265358979323846;

void require(const bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

ReferenceProgressAssociationParameters association_parameters(
    const double backward_tolerance_m = 0.2,
    const double maximum_progress_ratio = 1.2) {
  ParameterConfig config;
  config.profile_id = "reference-progress/v1";
  config.operating_domain_id = "synthetic-level1/v1";
  config.search.reference_progress_backward_tolerance_m =
      backward_tolerance_m;
  config.search.reference_progress_maximum_ratio = maximum_progress_ratio;
  config.search.reference_progress_forward_slack_m = 0.1;
  config.search.reference_progress_distance_scale_m = 1.0;
  config.search.reference_progress_heading_scale_rad = 0.5;
  config.search.reference_progress_heading_weight = 1.0;
  config.search.reference_progress_association_score_tolerance = 1.0e-10;
  return make_reference_progress_association_parameters(config);
}

ReferenceLine straight_reference(const std::uint32_t version = 7U) {
  return make_reference_line(version, "map", {{0.0, 0.0}, {100.0, 0.0}});
}

ReferenceLine repeated_crossing_reference() {
  return make_reference_line(7U, "map",
                             {{-5.0, 0.0},
                              {5.0, 0.0},
                              {5.0, 5.0},
                              {-5.0, 5.0},
                              {-5.0, 0.0},
                              {5.0, 0.0},
                              {10.0, 0.0}});
}

ReferenceProgress progress(const double arc_length_m,
                           const std::uint64_t sequence = 1U) {
  return {7U, arc_length_m, {1'000'000'000}, sequence};
}

TouchdownAssociationSample sample(const double laying_arc_length_m,
                                  const double x_m, const double y_m,
                                  const double heading_rad,
                                  const std::int64_t timestamp_ns) {
  return {laying_arc_length_m, {x_m, y_m}, heading_rad, {timestamp_ns}};
}

void candidate_progress_is_local_bounded_and_side_effect_free() {
  const auto parameters = association_parameters();
  ReferenceProgressTracker tracker(parameters);
  ReferenceProgressAssociator associator(parameters);
  const ReferenceLine reference = straight_reference();
  const auto reset = tracker.reset_for_new_task(reference, 10.0,
                                                 MonotonicTime{1'000'000'000});
  require(reset.status == ReferenceAssociationStatus::tracked,
          "valid task reset did not initialize reference progress");

  const auto propagated = associator.propagate_candidate(
      progress(10.0), sample(0.5, 80.0, 0.0, 0.0, 1'100'000'000), 0.5,
      reference);
  require(propagated.status == ReferenceAssociationStatus::tracked &&
              propagated.context.has_value(),
          "local candidate association failed");
  require(propagated.context->progress.arc_length_m <= 10.7 + 1.0e-12,
          "short primitive jumped beyond alpha_s * L_p + epsilon_s");
  require(tracker.snapshot().progress->arc_length_m == 10.0,
          "candidate propagation mutated actual executed task progress");
}

void the_same_crossing_pose_preserves_distinct_task_stages() {
  ReferenceProgressAssociator associator(association_parameters());
  const ReferenceLine reference = repeated_crossing_reference();
  const auto crossing = sample(1.0, 0.0, 0.0, 0.0, 2'000'000'000);

  const auto first = associator.propagate_candidate(progress(4.0), crossing,
                                                     1.0, reference);
  const auto second = associator.propagate_candidate(progress(34.0), crossing,
                                                      1.0, reference);
  require(first.context.has_value() && second.context.has_value(),
          "crossing associations did not produce progress contexts");
  require(std::abs(first.context->progress.arc_length_m - 5.0) < 1.0e-12 &&
              std::abs(second.context->progress.arc_length_m - 35.0) <
                  1.0e-12,
          "global nearest projection collapsed distinct crossing stages");
}

void equal_local_branches_report_association_ambiguity() {
  ReferenceProgressAssociator associator(association_parameters(20.0, 20.0));
  const auto result = associator.propagate_candidate(
      progress(20.0), sample(1.0, 0.0, 0.0, 0.0, 2'000'000'000), 1.0,
      repeated_crossing_reference());

  require(result.status == ReferenceAssociationStatus::association_ambiguous &&
              !result.context.has_value(),
          "equal reference branches were silently resolved by container order");
}

void candidate_regression_requests_are_explicit() {
  ReferenceProgressAssociator associator(association_parameters());
  const auto result = associator.propagate_candidate(
      progress(10.0), sample(0.5, 9.9, 0.0, 0.0, 2'000'000'000), 0.5,
      straight_reference());
  require(result.status == ReferenceAssociationStatus::regression_requested &&
              !result.context.has_value(),
          "candidate association silently moved task progress backward");
}

void actual_executed_laying_advances_monotonically() {
  ReferenceProgressTracker tracker(association_parameters());
  const ReferenceLine reference = straight_reference();
  static_cast<void>(tracker.reset_for_new_task(
      reference, 10.0, MonotonicTime{1'000'000'000}));
  ExecutedTouchdownSegment segment;
  segment.sequence_number = 1U;
  segment.samples = {
      sample(0.0, 10.0, 0.0, 0.0, 1'000'000'000),
      sample(1.0, 11.0, 0.0, 0.0, 1'500'000'000),
      sample(2.0, 12.0, 0.0, 0.0, 2'000'000'000),
  };

  const auto updated = tracker.update_from_executed_laying(segment, reference);
  require(updated.status == ReferenceAssociationStatus::tracked &&
              updated.progress.has_value() &&
              updated.parameter_profile_id == "reference-progress/v1" &&
              updated.operating_domain_id == "synthetic-level1/v1" &&
              std::abs(updated.progress->arc_length_m - 12.0) < 1.0e-12 &&
              updated.progress->timestamp.nanoseconds == 2'000'000'000 &&
              updated.progress->sequence_number == 2U,
          "executed laying did not advance a versioned monotonic snapshot");
}

void backward_execution_requests_and_version_changes_are_explicit() {
  ReferenceProgressTracker tracker(association_parameters());
  const ReferenceLine reference = straight_reference();
  static_cast<void>(tracker.reset_for_new_task(
      reference, 10.0, MonotonicTime{1'000'000'000}));

  ExecutedTouchdownSegment backward;
  backward.sequence_number = 1U;
  backward.samples = {
      sample(0.0, 10.0, 0.0, -kPi, 1'000'000'000),
      sample(5.0, 5.0, 0.0, -kPi, 2'000'000'000),
  };
  const auto rejected = tracker.update_from_executed_laying(backward, reference);
  require(rejected.status == ReferenceAssociationStatus::regression_requested &&
              rejected.progress->arc_length_m == 10.0,
          "task-progress regression was not rejected with an explicit state");

  const auto version_changed = tracker.update_from_executed_laying(
      backward, straight_reference(8U));
  require(version_changed.status ==
                  ReferenceAssociationStatus::reference_version_changed &&
              version_changed.progress->reference_line_version == 7U,
          "reference version change silently migrated task progress");
}

void local_geometry_and_corridor_queries_bind_one_reference_version() {
  ReferenceProgressAssociator associator(association_parameters());
  const ReferenceLine reference = straight_reference();
  const auto context = associator.query_local_context(
      progress(12.0), Vector2m{12.0, 2.0}, 1.0, reference);
  require(context.status == ReferenceAssociationStatus::tracked &&
              context.context.has_value(),
          "valid local reference context query failed");
  require(context.context->reference_line_version == 7U &&
              context.context->parameter_profile_id ==
                  "reference-progress/v1" &&
              context.context->operating_domain_id ==
                  "synthetic-level1/v1" &&
              context.context->progress.reference_line_version == 7U &&
              context.context->reference_point.arc_length_m == 12.0 &&
              context.context->reference_point.tangent_x == 1.0 &&
              context.context->reference_point.tangent_y == 0.0 &&
              context.context->reference_point.normal_x == 0.0 &&
              context.context->reference_point.normal_y == 1.0 &&
              context.context->signed_lateral_distance_m == 2.0 &&
              !context.context->local_corridor_centerline.empty(),
          "progress/tangent/normal/corridor query mixed reference versions");

  ReferenceProgress wrong_version = progress(12.0);
  wrong_version.reference_line_version = 6U;
  const auto mismatch = associator.query_local_context(
      wrong_version, Vector2m{12.0, 2.0}, 1.0, reference);
  require(mismatch.status ==
                  ReferenceAssociationStatus::reference_version_changed &&
              !mismatch.context.has_value(),
          "local query accepted progress from another reference version");
}

void repeated_association_is_field_deterministic() {
  ReferenceProgressAssociator associator(association_parameters());
  const ReferenceLine reference = repeated_crossing_reference();
  const auto touchdown = sample(1.0, 0.0, 0.0, 0.0, 2'000'000'000);
  const auto expected =
      associator.propagate_candidate(progress(4.0), touchdown, 1.0, reference);
  require(expected.context.has_value(),
          "determinism fixture did not produce a reference context");

  for (int repetition = 0; repetition < 20; ++repetition) {
    const auto actual =
        associator.propagate_candidate(progress(4.0), touchdown, 1.0,
                                       reference);
    require(actual.status == expected.status && actual.context.has_value() &&
                actual.context->reference_line_version ==
                    expected.context->reference_line_version &&
                actual.context->progress.arc_length_m ==
                    expected.context->progress.arc_length_m &&
                actual.context->progress.sequence_number ==
                    expected.context->progress.sequence_number &&
                actual.context->reference_point.tangent_x ==
                    expected.context->reference_point.tangent_x &&
                actual.context->reference_point.tangent_y ==
                    expected.context->reference_point.tangent_y &&
                actual.context->reference_point.normal_x ==
                    expected.context->reference_point.normal_x &&
                actual.context->reference_point.normal_y ==
                    expected.context->reference_point.normal_y &&
                actual.context->signed_lateral_distance_m ==
                    expected.context->signed_lateral_distance_m &&
                actual.context->local_corridor_centerline.size() ==
                    expected.context->local_corridor_centerline.size(),
            "identical association input changed a result field");
    for (std::size_t index = 0;
         index < actual.context->local_corridor_centerline.size(); ++index) {
      require(actual.context->local_corridor_centerline[index].arc_length_m ==
                      expected.context->local_corridor_centerline[index]
                          .arc_length_m &&
                  actual.context->local_corridor_centerline[index].x_m ==
                      expected.context->local_corridor_centerline[index].x_m &&
                  actual.context->local_corridor_centerline[index].y_m ==
                      expected.context->local_corridor_centerline[index].y_m,
              "identical association input changed its corridor window");
    }
  }
}

void invalid_and_discontinuous_inputs_fail_without_state_mutation() {
  ReferenceProgressTracker tracker(association_parameters());
  const ReferenceLine reference = straight_reference();
  static_cast<void>(tracker.reset_for_new_task(
      reference, 10.0, MonotonicTime{1'000'000'000}));
  ExecutedTouchdownSegment invalid;
  invalid.sequence_number = 0U;
  invalid.samples = {sample(0.0, 10.0, 0.0, 0.0, 1'000'000'000)};
  const auto rejected = tracker.update_from_executed_laying(invalid, reference);
  require(rejected.status == ReferenceAssociationStatus::input_invalid &&
              rejected.progress->arc_length_m == 10.0,
          "invalid executed input changed tracked progress");

  ExecutedTouchdownSegment first;
  first.sequence_number = 1U;
  first.samples = {
      sample(0.0, 10.0, 0.0, 0.0, 1'000'000'000),
      sample(1.0, 11.0, 0.0, 0.0, 2'000'000'000),
  };
  static_cast<void>(tracker.update_from_executed_laying(first, reference));
  ExecutedTouchdownSegment discontinuous;
  discontinuous.sequence_number = 2U;
  discontinuous.samples = {
      sample(1.0, 11.5, 0.0, 0.0, 2'000'000'000),
      sample(2.0, 12.5, 0.0, 0.0, 3'000'000'000),
  };
  const auto discontinuity =
      tracker.update_from_executed_laying(discontinuous, reference);
  require(discontinuity.status ==
                  ReferenceAssociationStatus::executed_segment_discontinuity &&
              discontinuity.progress->arc_length_m == 11.0,
          "discontinuous execution evidence was spliced into task progress");
}

void stale_execution_time_cannot_be_spliced() {
  ReferenceProgressTracker tracker(association_parameters());
  const ReferenceLine reference = straight_reference();
  static_cast<void>(tracker.reset_for_new_task(
      reference, 10.0, MonotonicTime{1'000'000'000}));

  ExecutedTouchdownSegment stale;
  stale.sequence_number = 1U;
  stale.samples = {
      sample(0.0, 10.0, 0.0, 0.0, 500'000'000),
      sample(1.0, 11.0, 0.0, 0.0, 600'000'000),
  };
  const auto time_regression =
      tracker.update_from_executed_laying(stale, reference);
  require(time_regression.status == ReferenceAssociationStatus::input_invalid &&
              time_regression.progress->timestamp.nanoseconds == 1'000'000'000,
          "stale execution evidence moved the progress timestamp backward");
}

void short_primitive_does_not_jump_to_a_nearby_competing_branch() {
  // Design: 18.2.3-15
  const ReferenceLine reference = make_reference_line(
      7U, "map", {{0.0, 0.0},
                   {10.0, 0.0},
                   {10.0, 5.0},
                   {0.0, 5.0},
                   {0.0, 0.1},
                   {10.0, 0.1}});
  ReferenceProgressAssociator associator(association_parameters());
  const auto result = associator.propagate_candidate(
      progress(4.0), sample(0.5, 4.5, 0.1, 0.0, 2'000'000'000), 0.5,
      reference);
  require(result.status == ReferenceAssociationStatus::tracked &&
              result.context.has_value() &&
              result.context->progress.arc_length_m <= 4.7 + 1.0e-12 &&
              result.context->progress.arc_length_m < 10.0,
          "short primitive jumped to a nearby later reference branch");
}

}  // namespace

int main() {
  try {
    candidate_progress_is_local_bounded_and_side_effect_free();
    the_same_crossing_pose_preserves_distinct_task_stages();
    equal_local_branches_report_association_ambiguity();
    candidate_regression_requests_are_explicit();
    actual_executed_laying_advances_monotonically();
    backward_execution_requests_and_version_changes_are_explicit();
    local_geometry_and_corridor_queries_bind_one_reference_version();
    repeated_association_is_field_deterministic();
    invalid_and_discontinuous_inputs_fail_without_state_mutation();
    stale_execution_time_cannot_be_spliced();
    short_primitive_does_not_jump_to_a_nearby_competing_branch();
    std::cout << "reference progress tracker tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "reference progress tracker test failure: " << error.what()
              << '\n';
    return 1;
  }
}
