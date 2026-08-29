#pragma once

#include "underwater_planner/core/cable_corridor_evaluator.hpp"
#include "underwater_planner/core/cable_laying_evaluator.hpp"
#include "underwater_planner/core/cable_model.hpp"
#include "underwater_planner/core/cable_uncertainty_envelope_manager.hpp"

#include <optional>
#include <string>
#include <vector>

namespace underwater_planner::core {

enum class TimedCableValidationStatus {
  valid,
  input_invalid,
  cable_model_invalid,
  covariance_envelope_unavailable,
  covariance_envelope_breach,
  corridor_invalid,
  corridor_violation,
  laying_invalid,
};

struct TimedCableCandidateInput {
  CableState initial_cable_state;
  TimedPath robot_path;
  CableContext cable_context;
  ReferenceLine reference_line;
  CableCorridorRiskPolicy corridor_policy;
  CableCorridorIntervalBoundCertificate interval_bound_certificate;
  std::vector<double> reference_progress_m;
  TerrainLayers terrain;
  CableLayingLimits laying_limits;
  CableHistoryBoundary history_boundary{
      CableHistoryBoundary::actual_laying_history};
  bool reference_is_deterministic{true};
  bool covariance_includes_coordinate_transform_error{};
  double envelope_audit_tolerance_m{};
  MonotonicTime evaluation_timestamp;
  std::optional<LockedCableUncertaintyEnvelope> locked_envelope;
};

struct TimedCableCandidateResult {
  TimedCableValidationStatus status{
      TimedCableValidationStatus::input_invalid};
  bool valid{};
  bool stop_required{};
  std::optional<CablePrediction> cable_prediction;
  std::optional<CableState> terminal_cable_state;
  CableCorridorResult corridor_result;
  CableLayingEvaluation laying_result;
  double maximum_actual_lateral_stddev_m{};
  double maximum_allowed_lateral_stddev_m{};
  std::vector<DiagnosticEntry> diagnostics;
  std::vector<std::string> issues;
  std::string risk_semantics{kPointwiseEnvelopeRiskSemantics};
};

class TimedCableCandidateVerifier {
 public:
  explicit TimedCableCandidateVerifier(
      CableModel model,
      CableUncertaintyEnvelopeManager* envelope_manager = nullptr);

  [[nodiscard]] TimedCableCandidateResult validate(
      const TimedCableCandidateInput& input) const;

 private:
  CableModel model_;
  CableUncertaintyEnvelopeManager* envelope_manager_{};
};

using TimedCablePathVerifier = TimedCableCandidateVerifier;

}  // namespace underwater_planner::core
