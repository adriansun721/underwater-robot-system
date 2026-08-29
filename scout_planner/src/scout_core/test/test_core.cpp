#include "scout_planner/core/core.hpp"
#include "test_support.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {

constexpr std::uint64_t kSeed = 0x5C0A7B13ULL;
const std::string failure_context =
    scout_planner::test_support::default_failure_context(kSeed);

void test_blank_core_declares_its_boundary() {
  const auto& info = scout_planner::core::core_build_info();
  scout_planner::test_support::require(
      info.component == "scout_planner_core", "unexpected component name");
  scout_planner::test_support::require(info.version == "0.1.0",
                                       "unexpected bootstrap version");
  scout_planner::test_support::require(
      info.ros_independent, "algorithm core must remain ROS-independent");
  scout_planner::test_support::require(
      !info.production_ready,
      "bootstrap core must not claim production readiness");
}

}  // namespace

int main() {
  try {
    test_blank_core_declares_its_boundary();
    std::cout << "[pass] blank_core_declares_its_boundary\n";
  } catch (const std::exception& error) {
    std::cerr << "[failure] " << failure_context
              << " test=blank_core_declares_its_boundary error=" << error.what()
              << '\n';
    return 1;
  }

  return 0;
}
