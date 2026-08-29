#pragma once

#include <string_view>

namespace scout_planner::core {

struct CoreBuildInfo {
  std::string_view component;
  std::string_view version;
  bool ros_independent;
  bool production_ready;
};

[[nodiscard]] const CoreBuildInfo& core_build_info() noexcept;

}  // namespace scout_planner::core
