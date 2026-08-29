#include "underwater_planner/testing/deterministic_closed_loop.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
using namespace underwater_planner::testing;

void require(const bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void flat_straight_scenario_runs_the_complete_authorization_loop() {
  DeterministicClosedLoopDriver driver;
  const ClosedLoopScenarioReport report =
      driver.run(ClosedLoopScenario::flat_straight, 47001U);

  if (!report.passed) {
    throw std::runtime_error("flat straight scenario did not pass: " +
                             serialize_closed_loop_report(report));
  }
  require(report.cycles.size() == 1U,
          "flat straight scenario did not complete in one cycle");
  const ClosedLoopCycleReport& cycle = report.cycles.front();
  require(cycle.planning_succeeded && cycle.command_authorized &&
              cycle.invariants.all_passed() && cycle.invariants.all_checked(),
          "flat straight cycle was not fully validated and authorized");
  require(cycle.real_search_executed &&
              cycle.independent_robot_path_validation_executed &&
              cycle.cable_prediction_executed &&
              cycle.plan_revalidation_executed,
          "flat straight cycle used a scripted planning-stage adapter");
  require(cycle.executed_stages == 13U,
          "flat straight cycle skipped a complete planning stage");

  const std::string json = serialize_closed_loop_report(report);
  require(json.find("\"schema_version\":\"closed-loop-scenario/v1\"") !=
              std::string::npos &&
              json.find("\"scenario\":\"flat_straight\"") !=
                  std::string::npos &&
              json.find("\"seed\":47001") != std::string::npos,
          "flat straight report omitted stable comparison metadata");

  ClosedLoopScenarioReport control_character_report;
  std::string controls;
  controls.push_back('\b');
  controls.push_back('\f');
  controls.push_back(static_cast<char>(0x01));
  control_character_report.issues.push_back(controls);
  const std::string escaped =
      serialize_closed_loop_report(control_character_report);
  require(escaped.find("\\u0008\\u000c\\u0001") != std::string::npos,
          "report serializer emitted an unescaped JSON control byte");
}

void detour_slope_and_step_scenarios_remain_fully_authorized() {
  DeterministicClosedLoopDriver driver;
  const ClosedLoopScenario scenarios[] = {
      ClosedLoopScenario::single_side_detour,
      ClosedLoopScenario::double_side_detour,
      ClosedLoopScenario::traversable_slope,
      ClosedLoopScenario::traversable_step,
  };
  for (const ClosedLoopScenario scenario : scenarios) {
    const ClosedLoopScenarioReport report = driver.run(scenario, 47002U);
    if (!report.passed || report.cycles.size() != 1U) {
      throw std::runtime_error("authorized geometry scenario did not pass: " +
                               serialize_closed_loop_report(report));
    }
    const ClosedLoopCycleReport& cycle = report.cycles.front();
    require(cycle.planning_succeeded && cycle.command_authorized &&
                cycle.invariants.all_passed() &&
                cycle.invariants.all_checked() &&
                cycle.real_search_executed &&
                cycle.independent_robot_path_validation_executed &&
                cycle.cable_prediction_executed &&
                cycle.plan_revalidation_executed,
            "geometry scenario escaped a per-cycle hard invariant");
    if (scenario == ClosedLoopScenario::traversable_slope) {
      require(cycle.terrain_condition == "slope",
              "slope scenario did not retain analyzed terrain evidence");
    } else if (scenario == ClosedLoopScenario::traversable_step) {
      require(cycle.terrain_condition == "step",
              "step scenario did not retain analyzed terrain evidence");
    } else {
      require(cycle.terrain_condition == "obstacle",
              "detour scenario did not retain obstacle evidence");
    }
  }

  const ClosedLoopScenarioReport single_report =
      driver.run(ClosedLoopScenario::single_side_detour, 47002U);
  const ClosedLoopScenarioReport dual_report =
      driver.run(ClosedLoopScenario::double_side_detour, 47002U);
  const ClosedLoopCycleReport& single = single_report.cycles.front();
  const ClosedLoopCycleReport& dual = dual_report.cycles.front();
  if (!(single.maximum_robot_lateral_offset_m > 0.4 &&
        dual.maximum_robot_lateral_offset_m > 0.4)) {
    throw std::runtime_error(
        "detour scenarios did not bypass their map obstacles: " +
        serialize_closed_loop_report(single_report) + " " +
        serialize_closed_loop_report(dual_report));
  }
}

void off_route_state_is_recovered_independently_of_obstacle_detours() {
  DeterministicClosedLoopDriver driver;
  const ClosedLoopScenarioReport report =
      driver.run(ClosedLoopScenario::route_deviation_recovery, 47003U);
  if (!report.passed || report.cycles.size() != 1U) {
    throw std::runtime_error("route deviation scenario did not pass: " +
                             serialize_closed_loop_report(report));
  }
  const ClosedLoopCycleReport& cycle = report.cycles.front();
  require(cycle.route_deviation_recovered && cycle.planning_succeeded &&
              cycle.command_authorized &&
              cycle.maximum_robot_lateral_offset_m >= 0.3 &&
              cycle.invariants.all_passed() && cycle.invariants.all_checked(),
          "off-route actual state did not rejoin the reference line safely");
}

void unknown_gap_closes_the_scout_map_replan_loop() {
  DeterministicClosedLoopDriver driver;
  const ClosedLoopScenarioReport report =
      driver.run(ClosedLoopScenario::unknown_gap, 47003U);
  if (!report.passed || report.cycles.size() != 2U) {
    throw std::runtime_error(
        "unknown gap scenario did not complete its two-cycle loop: " +
        serialize_closed_loop_report(report));
  }
  const ClosedLoopCycleReport& waiting = report.cycles.front();
  const ClosedLoopCycleReport& replanned = report.cycles.back();
  require(waiting.scout_requested && waiting.controlled_stop_required &&
              !waiting.command_authorized && waiting.invariants.all_passed() &&
              waiting.invariants.all_checked() &&
              waiting.old_lease_reuse_rejected,
          "unknown gap did not stop without stale execution authorization");
  require(replanned.map_updated && replanned.replan_required &&
              replanned.planning_succeeded && replanned.command_authorized &&
              replanned.map_sequence > waiting.map_sequence &&
              replanned.invariants.all_passed() &&
              replanned.invariants.all_checked() &&
              replanned.real_search_executed,
          "resolved scout update did not trigger a versioned authorized replan");
}

void covariance_envelope_breach_fails_closed_without_a_lease() {
  DeterministicClosedLoopDriver driver;
  const ClosedLoopScenarioReport report =
      driver.run(ClosedLoopScenario::covariance_envelope_breach, 47004U);
  if (!report.passed || report.cycles.size() != 1U) {
    throw std::runtime_error(
        "covariance envelope breach scenario did not reach its safe outcome: " +
        serialize_closed_loop_report(report));
  }
  const ClosedLoopCycleReport& cycle = report.cycles.front();
  require(cycle.planning_status == "covariance_envelope_breached" &&
              cycle.controlled_stop_required && !cycle.command_authorized &&
              cycle.lease_sequence == 0U && cycle.invariants.all_passed() &&
              cycle.invariants.all_checked(),
          "covariance envelope breach crossed the authorization boundary");
}

void injected_time_order_telemetry_and_model_events_fail_closed() {
  DeterministicClosedLoopDriver driver;
  const ClosedLoopInjection injections[] = {
      ClosedLoopInjection::advance_time,
      ClosedLoopInjection::out_of_order_message,
      ClosedLoopInjection::telemetry_deviation,
      ClosedLoopInjection::cable_model_version_change,
  };
  for (const ClosedLoopInjection injection : injections) {
    const std::vector<ClosedLoopInjection> script{injection};
    const ClosedLoopScenarioReport report =
        driver.run(ClosedLoopScenario::flat_straight, 47005U, script);
    if (!report.passed || report.cycles.size() < 2U) {
      throw std::runtime_error(
          "injected closed-loop event did not reach a safe outcome: " +
          serialize_closed_loop_report(report));
    }
    require(report.cycles.front().command_authorized &&
                !report.cycles[1].command_authorized &&
                report.cycles[1].controlled_stop_required &&
                report.cycles[1].invariants.all_passed() &&
                report.cycles[1].invariants.all_checked() &&
                report.cycles[1].old_lease_reuse_rejected,
            "injected event allowed the previous authorization to continue");
    require(serialize_closed_loop_report(report) ==
                serialize_closed_loop_report(driver.run(
                    ClosedLoopScenario::flat_straight, 47005U, script)),
            "identical injection script did not reproduce byte-identical output");
    if (injection == ClosedLoopInjection::cable_model_version_change) {
      require(report.cycles.size() == 3U &&
                  report.cycles.back().planning_succeeded &&
                  report.cycles.back().command_authorized &&
                  report.cycles.back().cable_model_version >
                      report.cycles.front().cable_model_version,
              "model version change did not produce a newly bound plan");
    }
  }
}

void complete_suite_is_deterministic_and_covers_every_required_scenario() {
  DeterministicClosedLoopDriver driver;
  const ClosedLoopRegressionReport report = driver.run_all(47006U);
  require(report.passed && report.scenarios.size() == 8U &&
              report.injection_runs.size() == 4U,
          "complete closed-loop regression suite is incomplete");
  const std::string first = serialize_closed_loop_report(report);
  const std::string second =
      serialize_closed_loop_report(driver.run_all(47006U));
  require(first == second &&
              first.find("\"schema_version\":\"closed-loop-regression/v1\"") !=
                  std::string::npos &&
              first.find("NO_PATH_JOINT_RISK_GUARANTEE") != std::string::npos,
          "closed-loop regression report is unstable or overstates risk");
}

}  // namespace

int main(const int argc, char** argv) {
  try {
    flat_straight_scenario_runs_the_complete_authorization_loop();
    detour_slope_and_step_scenarios_remain_fully_authorized();
    off_route_state_is_recovered_independently_of_obstacle_detours();
    unknown_gap_closes_the_scout_map_replan_loop();
    covariance_envelope_breach_fails_closed_without_a_lease();
    injected_time_order_telemetry_and_model_events_fail_closed();
    complete_suite_is_deterministic_and_covers_every_required_scenario();
    if (argc == 2) {
      const std::string report = serialize_closed_loop_report(
          DeterministicClosedLoopDriver().run_all(47006U));
      std::ofstream output(argv[1], std::ios::binary | std::ios::trunc);
      if (!output || !(output << report << '\n')) {
        throw std::runtime_error("could not write closed-loop regression report");
      }
    }
    std::cout << "level 1 closed-loop scenario tests passed\n";
    return EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::cerr << "level 1 closed-loop scenario tests failed: " << error.what()
              << '\n';
    return EXIT_FAILURE;
  }
}
