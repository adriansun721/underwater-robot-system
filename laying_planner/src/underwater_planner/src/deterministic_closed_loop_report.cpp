#include "underwater_planner/testing/deterministic_closed_loop.hpp"

#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>

namespace underwater_planner::testing {
namespace {

std::string escape_json(const std::string_view value) {
  std::string escaped;
  for (const char character : value) {
    const auto byte = static_cast<unsigned char>(character);
    switch (character) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        if (byte < 0x20U) {
          constexpr char hex[] = "0123456789abcdef";
          escaped += "\\u00";
          escaped += hex[(byte >> 4U) & 0x0fU];
          escaped += hex[byte & 0x0fU];
        } else {
          escaped += character;
        }
        break;
    }
  }
  return escaped;
}

void append_bool(std::ostringstream& output, const bool value) {
  output << (value ? "true" : "false");
}

void append_evidence(std::ostringstream& output,
                     const ClosedLoopInvariantAudit& audit) {
  output << ",\"evidence\":{\"robot_operating_area\":\""
         << escape_json(audit.robot_operating_area_evidence)
         << "\",\"cable_corridor\":\""
         << escape_json(audit.cable_corridor_evidence)
         << "\",\"cable_mechanical_constraints\":\""
         << escape_json(audit.cable_mechanical_constraints_evidence)
         << "\",\"dependency_versions\":\""
         << escape_json(audit.dependency_versions_evidence)
         << "\",\"execution_lease\":\""
         << escape_json(audit.execution_lease_evidence) << "\"}";
}

}  // namespace

std::string serialize_closed_loop_report(
    const ClosedLoopScenarioReport& report) {
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << std::setprecision(std::numeric_limits<double>::max_digits10);
  output << "{\"schema_version\":\"" << escape_json(report.schema_version)
         << "\",\"scenario\":\"" << to_string(report.scenario)
         << "\",\"seed\":" << report.seed << ",\"input_version\":\""
         << escape_json(report.input_version) << "\",\"unit_system\":\""
         << escape_json(report.unit_system)
         << "\",\"operating_domain_id\":\""
         << escape_json(report.operating_domain_id)
         << "\",\"risk_semantics\":\""
         << escape_json(report.risk_semantics) << "\",\"passed\":";
  append_bool(output, report.passed);
  output << ",\"cycles\":[";
  for (std::size_t index = 0; index < report.cycles.size(); ++index) {
    if (index != 0U) output << ',';
    const ClosedLoopCycleReport& cycle = report.cycles[index];
    output << "{\"cycle_sequence\":" << cycle.cycle_sequence
           << ",\"time_ns\":" << cycle.time_ns
           << ",\"source_revision\":" << cycle.source_revision
           << ",\"map_sequence\":" << cycle.map_sequence
           << ",\"cable_model_version\":" << cycle.cable_model_version
           << ",\"plan_sequence\":" << cycle.plan_sequence
           << ",\"lease_sequence\":" << cycle.lease_sequence
           << ",\"executed_stages\":" << cycle.executed_stages
           << ",\"planning_status\":\""
           << escape_json(cycle.planning_status)
           << "\",\"planning_state\":\""
           << escape_json(cycle.planning_state) << "\",\"event\":\""
           << escape_json(cycle.event) << "\",\"terrain_condition\":\""
           << escape_json(cycle.terrain_condition)
           << "\",\"planning_succeeded\":";
    append_bool(output, cycle.planning_succeeded);
    output << ",\"command_authorized\":";
    append_bool(output, cycle.command_authorized);
    output << ",\"controlled_stop_required\":";
    append_bool(output, cycle.controlled_stop_required);
    output << ",\"replan_required\":";
    append_bool(output, cycle.replan_required);
    output << ",\"scout_requested\":";
    append_bool(output, cycle.scout_requested);
    output << ",\"map_updated\":";
    append_bool(output, cycle.map_updated);
    output << ",\"route_deviation_recovered\":";
    append_bool(output, cycle.route_deviation_recovered);
    output << ",\"real_search_executed\":";
    append_bool(output, cycle.real_search_executed);
    output << ",\"independent_robot_path_validation_executed\":";
    append_bool(output, cycle.independent_robot_path_validation_executed);
    output << ",\"cable_prediction_executed\":";
    append_bool(output, cycle.cable_prediction_executed);
    output << ",\"plan_revalidation_executed\":";
    append_bool(output, cycle.plan_revalidation_executed);
    output << ",\"old_lease_reuse_rejected\":";
    append_bool(output, cycle.old_lease_reuse_rejected);
    output << ",\"maximum_robot_lateral_offset_m\":"
           << cycle.maximum_robot_lateral_offset_m
           << ",\"maximum_touchdown_lateral_offset_m\":"
           << cycle.maximum_touchdown_lateral_offset_m
           << ",\"invariants\":{\"robot_operating_area\":";
    append_bool(output, cycle.invariants.robot_operating_area);
    output << ",\"cable_corridor\":";
    append_bool(output, cycle.invariants.cable_corridor);
    output << ",\"cable_mechanical_constraints\":";
    append_bool(output, cycle.invariants.cable_mechanical_constraints);
    output << ",\"dependency_versions\":";
    append_bool(output, cycle.invariants.dependency_versions);
    output << ",\"execution_lease\":";
    append_bool(output, cycle.invariants.execution_lease);
    output << ",\"dispositions\":{\"robot_operating_area\":\""
           << to_string(cycle.invariants.robot_operating_area_disposition)
           << "\",\"cable_corridor\":\""
           << to_string(cycle.invariants.cable_corridor_disposition)
           << "\",\"cable_mechanical_constraints\":\""
           << to_string(
                  cycle.invariants.cable_mechanical_constraints_disposition)
           << "\",\"dependency_versions\":\""
           << to_string(cycle.invariants.dependency_versions_disposition)
           << "\",\"execution_lease\":\""
           << to_string(cycle.invariants.execution_lease_disposition) << "\"}";
    append_evidence(output, cycle.invariants);
    output << "},\"diagnostics\":[";
    for (std::size_t diagnostic_index = 0;
         diagnostic_index < cycle.diagnostics.size(); ++diagnostic_index) {
      if (diagnostic_index != 0U) output << ',';
      output << '"' << escape_json(cycle.diagnostics[diagnostic_index]) << '"';
    }
    output << "]}";
  }
  output << "],\"issues\":[";
  for (std::size_t index = 0; index < report.issues.size(); ++index) {
    if (index != 0U) output << ',';
    output << '"' << escape_json(report.issues[index]) << '"';
  }
  output << "]}";
  return output.str();
}

std::string serialize_closed_loop_report(
    const ClosedLoopRegressionReport& report) {
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << "{\"schema_version\":\"" << escape_json(report.schema_version)
         << "\",\"seed\":" << report.seed << ",\"input_version\":\""
         << escape_json(report.input_version) << "\",\"unit_system\":\""
         << escape_json(report.unit_system)
         << "\",\"operating_domain_id\":\""
         << escape_json(report.operating_domain_id)
         << "\",\"risk_semantics\":\""
         << escape_json(report.risk_semantics) << "\",\"passed\":";
  append_bool(output, report.passed);
  output << ",\"scenarios\":[";
  for (std::size_t index = 0; index < report.scenarios.size(); ++index) {
    if (index != 0U) output << ',';
    output << serialize_closed_loop_report(report.scenarios[index]);
  }
  output << "],\"injection_runs\":[";
  for (std::size_t index = 0; index < report.injection_runs.size(); ++index) {
    if (index != 0U) output << ',';
    output << serialize_closed_loop_report(report.injection_runs[index]);
  }
  output << "]}";
  return output.str();
}

}  // namespace underwater_planner::testing
