#pragma once

#include "underwater_planner/core/data_contract.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace underwater_planner::core {

// A published result is never exposed as a mutable PlanningResult.  The
// mutable contract remains an assembly type for the planning pipeline; this
// handle is the execution-facing, immutable artifact.
class ImmutablePlanningResult {
 public:
  ImmutablePlanningResult() = delete;

  [[nodiscard]] const PlanningResult& value() const noexcept { return *value_; }
  [[nodiscard]] const PlanningResult* operator->() const noexcept {
    return value_.get();
  }
  [[nodiscard]] const PlanningResult& operator*() const noexcept { return *value_; }

 private:
  explicit ImmutablePlanningResult(std::shared_ptr<const PlanningResult> value)
      : value_(std::move(value)) {}

  std::shared_ptr<const PlanningResult> value_;
  friend class PlanningResultPublisher;
};

enum class PlanningResultPublishStatus {
  published,
  invalid,
  sequence_not_monotonic,
};

struct PlanningResultPublication {
  PlanningResultPublishStatus status{PlanningResultPublishStatus::invalid};
  std::optional<ImmutablePlanningResult> result;
  std::vector<std::string> issues;

  [[nodiscard]] bool published() const noexcept {
    return status == PlanningResultPublishStatus::published && result.has_value();
  }
};

// Owns the publication sequence and the last immutable result.  A candidate
// is copied only after validation succeeds, so callers cannot mutate a result
// after it has crossed the publication boundary.
class PlanningResultPublisher {
 public:
  explicit PlanningResultPublisher(std::uint64_t last_sequence = 0) noexcept
      : last_sequence_(last_sequence) {}

  [[nodiscard]] PlanningResultPublication publish(
      const PlanningResult& candidate);
  [[nodiscard]] std::uint64_t last_sequence() const noexcept {
    return last_sequence_;
  }
  [[nodiscard]] const std::optional<ImmutablePlanningResult>& current() const
      noexcept {
    return current_;
  }

 private:
  std::uint64_t last_sequence_{};
  std::optional<ImmutablePlanningResult> current_;
};

}  // namespace underwater_planner::core
