#pragma once

#include <string_view>

namespace underwater_planner::core {

[[nodiscard]] std::string_view api_version() noexcept;

}  // namespace underwater_planner::core
