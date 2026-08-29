#pragma once

#include "scout_planner/testing/deterministic_fixture.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>

namespace scout_planner::test_support {

inline void require(const bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

inline std::string default_failure_context(const std::uint64_t seed) {
  return testing::format_failure_context(testing::make_fixture_metadata(seed));
}

}  // namespace scout_planner::test_support
