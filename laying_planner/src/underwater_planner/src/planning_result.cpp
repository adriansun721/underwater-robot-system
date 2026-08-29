#include "underwater_planner/core/planning_result.hpp"

#include <utility>

namespace underwater_planner::core {

PlanningResultPublication PlanningResultPublisher::publish(
    const PlanningResult& candidate) {
  const ValidationResult validation = validate(candidate);
  if (!validation.valid) {
    return {PlanningResultPublishStatus::invalid, std::nullopt,
            validation.issues};
  }
  if (candidate.sequence_number <= last_sequence_) {
    return {PlanningResultPublishStatus::sequence_not_monotonic, std::nullopt,
            {"planning result sequence_number must strictly increase"}};
  }

  auto immutable = std::make_shared<const PlanningResult>(candidate);
  last_sequence_ = candidate.sequence_number;
  current_ = ImmutablePlanningResult(std::move(immutable));
  return {PlanningResultPublishStatus::published, current_, {}};
}

}  // namespace underwater_planner::core
