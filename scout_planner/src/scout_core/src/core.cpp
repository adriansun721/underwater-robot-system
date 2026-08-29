#include "scout_planner/core/core.hpp"

namespace scout_planner::core {

const CoreBuildInfo& core_build_info() noexcept {
  static constexpr CoreBuildInfo info{
      "scout_planner_core",
      "0.1.0",
      true,
      false,
  };
  return info;
}

}  // namespace scout_planner::core
