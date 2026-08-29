#include "scout_planner/core/survey_plan_evidence.hpp"

#include <iostream>
#include <stdexcept>

namespace core = scout_planner::core;

int main() {
  try {
    core::SurveyPlanEvidence value;
    value.completion_evidence = false;
    value.planner_configuration_id = "planner-v1";
    value.conservative_predicted_coverage_ratio = 0.75;
    const auto success = core::SurveyPlanEvidenceResult::success(value);
    if (!success.has_value() || success.value().completion_evidence ||
        success.value().conservative_predicted_coverage_ratio != 0.75) {
      throw std::runtime_error{"plan evidence value contract failed"};
    }
    const auto failure = core::SurveyPlanEvidenceResult::failure(
        {core::SurveyPlanEvidenceFailure::insufficient_coverage, 3U,
         "coverage"});
    if (failure.has_value() ||
        failure.error().code != core::SurveyPlanEvidenceFailure::insufficient_coverage ||
        failure.error().sample_index != 3U) {
      throw std::runtime_error{"plan evidence failure contract failed"};
    }
    std::cout << "[pass] survey plan evidence value contract\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "[fail] " << error.what() << '\n';
    return 1;
  }
}
