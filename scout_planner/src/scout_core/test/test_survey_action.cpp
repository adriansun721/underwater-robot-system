#include "scout_planner/core/survey_action.hpp"

#include <iostream>
#include <stdexcept>

namespace core = scout_planner::core;

int main() {
  try {
    core::SurveyActionReport report;
    report.segments = {
        {core::SurveyActionPhase::approach, {}, {}, 1.0, 0.0, 0.0},
        {core::SurveyActionPhase::observe, {}, {}, 2.0, 0.0, 2.0},
        {core::SurveyActionPhase::exit, {}, {}, 1.0, 0.0, 0.0},
    };
    const auto success = core::SurveyActionResult::success(report);
    if (!success.has_value() || success.value().segments.size() != 3U ||
        success.value().segments[0].phase != core::SurveyActionPhase::approach ||
        success.value().segments[1].phase != core::SurveyActionPhase::observe ||
        success.value().segments[2].phase != core::SurveyActionPhase::exit) {
      throw std::runtime_error{"survey action phase contract failed"};
    }
    const auto failure = core::SurveyActionResult::failure(
        {core::SurveyActionFailure::mandatory_coverage_missing, 4U, "missing"});
    if (failure.has_value() ||
        failure.error().code != core::SurveyActionFailure::mandatory_coverage_missing ||
        failure.error().sample_index != 4U) {
      throw std::runtime_error{"survey action failure contract failed"};
    }
    std::cout << "[pass] survey action value contract\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "[fail] " << error.what() << '\n';
    return 1;
  }
}
