#pragma once

#include "underwater_planner/core/plan_validity_evaluator.hpp"

#include <memory>
#include <optional>
#include <string>
#include <functional>
#include <utility>

namespace underwater_planner::core {

struct PathHysteresisConfig {
  // Fraction of the current cost that a candidate must improve by.  A value
  // of 0.1 implements the design baseline's 10% relative threshold.
  double relative_cost_threshold{0.1};
  // Optional calibrated Hausdorff threshold.  When present, a topologically
  // different candidate may switch even if its soft-cost improvement is small.
  std::optional<double> topology_distance_threshold_m;
};

enum class PathSwitchAction { keep_current, switch_to_candidate, stop };

struct PathSwitchDecision {
  PathSwitchAction action{PathSwitchAction::stop};
  std::optional<PlanValidationLease> lease;
  std::shared_ptr<const TimedPath> remaining_path;
  std::string reason;

  [[nodiscard]] bool should_switch() const noexcept {
    return action == PathSwitchAction::switch_to_candidate;
  }
  [[nodiscard]] bool should_keep_current() const noexcept {
    return action == PathSwitchAction::keep_current;
  }
};

struct CommitmentSegmentConfig {
  // The latency/commitment policy is expressed in seconds.  A non-positive
  // value is invalid; a zero safety margin is allowed when explicitly
  // calibrated.
  double commitment_time_s{};
  double safety_margin_m{};
  // Supplied by the certified execution/lease model.  The stability layer
  // never substitutes an uncalibrated kinematic braking estimate.
  std::optional<double> certified_worst_case_stopping_distance_m;
};

enum class CommitmentExtractionStatus {
  valid,
  input_invalid,
  stopping_distance_unavailable,
  authorization_range_insufficient,
};

struct CommitmentExtractionResult {
  CommitmentExtractionStatus status{
      CommitmentExtractionStatus::input_invalid};
  std::optional<TimedPath> segment;
  double required_length_m{};
  double available_length_m{};
  std::string reason;
};

struct ExecutionJoinTolerances {
  double ground_speed_mps{};
  double ground_acceleration_mps2{};
  double payout_speed_mps{};
  double tension_n{};
  double payout_acceleration_mps2{};
};

struct TimedPathMergeResult {
  bool valid{};
  std::optional<TimedPath> trajectory;
  std::string reason;
};

using TimedPathFinalVerifier = std::function<bool(const TimedPath&)>;

// Owns only the quality-stability policy.  Safety validity and lease pairing
// are checked by decide_path_switch before this class applies hysteresis.
class StabilityManager {
 public:
  using RevalidationResult =
      std::pair<std::optional<PlanValidityEvaluation>, PlanValidityEvaluation>;
  using RevalidationCallback = std::function<RevalidationResult()>;

  explicit StabilityManager(PathHysteresisConfig config = {}) noexcept;

  // Applies the design equation:
  //   switch iff new_cost < current_cost - current_cost * threshold.
  // Invalid costs or paths fail closed and return false.
  [[nodiscard]] bool should_switch_path(const GeometricPath& current_path,
                                        const GeometricPath& new_path,
                                        double current_cost,
                                        double new_cost) const noexcept;

  // Design-document spelling retained for adapters using the pseudocode API.
  [[nodiscard]] bool shouldSwitchPath(const GeometricPath& current_path,
                                      const GeometricPath& new_path,
                                      double current_cost,
                                      double new_cost) const noexcept {
    return should_switch_path(current_path, new_path, current_cost, new_cost);
  }

  // Decides publication only from complete, current validation results.  A
  // lease is returned with every keep/switch decision so publication cannot
  // accidentally detach a plan from the lease issued by this recapture.
  [[nodiscard]] PathSwitchDecision decide_path_switch(
      const std::optional<PlanValidityEvaluation>& current,
      const PlanValidityEvaluation& candidate, double current_cost,
      double candidate_cost, MonotonicTime now) const;

  // If planning crossed a lease expiry, the callback must capture one fresh
  // synchronized context and fully validate both remaining plans before this
  // method compares them.  A missing callback fails closed.
  [[nodiscard]] PathSwitchDecision decide_path_switch(
      const std::optional<PlanValidityEvaluation>& current,
      const PlanValidityEvaluation& candidate, double current_cost,
      double candidate_cost, MonotonicTime now,
      const RevalidationCallback& recapture_and_revalidate) const;

  [[nodiscard]] PathSwitchDecision decidePathSwitch(
      const std::optional<PlanValidityEvaluation>& current,
      const PlanValidityEvaluation& candidate, double current_cost,
      double candidate_cost, MonotonicTime now) const {
    return decide_path_switch(current, candidate, current_cost, candidate_cost,
                              now);
  }

  // Symmetric Hausdorff distance in metres over path samples.  Non-finite or
  // empty paths return positive infinity so callers cannot treat malformed
  // geometry as a close topology.
  [[nodiscard]] static double topology_distance_m(const GeometricPath& left,
                                                   const GeometricPath& right) noexcept;

  [[nodiscard]] const PathHysteresisConfig& config() const noexcept {
    return config_;
  }

  // Extracts the immutable near-term prefix of an already authorized
  // remaining trajectory.  The input trajectory is the authorization range;
  // no extension beyond its terminal sample is possible.
  [[nodiscard]] CommitmentExtractionResult extract_commitment_segment(
      const TimedPath& current_trajectory, const RobotState& robot_state,
      const CommitmentSegmentConfig& config) const;

  [[nodiscard]] CommitmentExtractionResult extractCommitmentSegment(
      const TimedPath& current_trajectory, const RobotState& robot_state,
      const CommitmentSegmentConfig& config) const {
    return extract_commitment_segment(current_trajectory, robot_state, config);
  }

  // Merges an unchanged commitment prefix and a newly parameterized tail.
  // The result owns a new execution-profile version and never mutates either
  // input profile or geometry.
  [[nodiscard]] TimedPathMergeResult merge_timed_paths(
      const TimedPath& commitment, const TimedPath& new_tail,
      const PathG2MergeLimits& geometric_tolerances,
      const ExecutionJoinTolerances& execution_tolerances,
      const TimedPathFinalVerifier& final_verifier = {}) const;

  [[nodiscard]] TimedPathMergeResult mergeTimedPaths(
      const TimedPath& commitment, const TimedPath& new_tail,
      const PathG2MergeLimits& geometric_tolerances,
      const ExecutionJoinTolerances& execution_tolerances,
      const TimedPathFinalVerifier& final_verifier = {}) const {
    return merge_timed_paths(commitment, new_tail, geometric_tolerances,
                             execution_tolerances, final_verifier);
  }

 private:
  [[nodiscard]] static bool valid_path(const GeometricPath& path) noexcept;
  [[nodiscard]] static bool valid_evaluation(
      const PlanValidityEvaluation& evaluation, MonotonicTime now) noexcept;
  [[nodiscard]] static bool lease_requires_recapture(
      const std::optional<PlanValidityEvaluation>& evaluation,
      MonotonicTime now) noexcept;
  [[nodiscard]] static bool same_validation_context(
      const PlanValidationLease& left,
      const PlanValidationLease& right) noexcept;

  PathHysteresisConfig config_;
};

}  // namespace underwater_planner::core
