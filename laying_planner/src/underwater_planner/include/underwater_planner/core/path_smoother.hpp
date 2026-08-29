#pragma once

#include "underwater_planner/core/data_contract.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace underwater_planner::core {

enum class PathBoundarySource {
  synchronized_actual_state,
  committed_segment_terminal,
  planned_goal,
};

struct PathBoundary {
  double x_m{};
  double y_m{};
  double heading_rad{};
  std::optional<double> curvature_per_m;
  PathBoundarySource curvature_source{PathBoundarySource::planned_goal};
  MonotonicTime pose_timestamp;
  MonotonicTime curvature_timestamp;
  std::uint64_t source_sequence_number{};
};

using ConstraintResiduals = PathConstraintResiduals;

struct SmoothingObjectiveWeights {
  double deviation{};
  double curvature{};
  double curvature_rate{};
  double length{};
};

struct SmoothingLimits {
  std::uint64_t version{};
  std::uint64_t output_path_version{};
  double spatial_step_m{};
  double maximum_curvature_per_m{};
  double maximum_curvature_rate_per_m2{};
  double minimum_segment_length_m{};
  double topology_tube_radius_m{};
  Duration timeout;
  Duration maximum_boundary_time_skew;
  ConstraintResiduals allowed_residuals;
  SmoothingObjectiveWeights objective_weights;
};

[[nodiscard]] std::string serialize_smoothing_limits(
    const SmoothingLimits& limits);

enum class SmoothingStatus {
  success,
  boundary_state_invalid,
  seed_infeasible,
  solver_timeout,
  solver_failed,
  constraint_residual_exceeded,
  trackability_validation_failed,
};

struct SmoothingSolverResult {
  SmoothingStatus status{SmoothingStatus::solver_failed};
  GeometricPath candidate;
  std::size_t iterations{};
};

class PathSmoothingSolver {
 public:
  virtual ~PathSmoothingSolver() = default;
  [[nodiscard]] virtual SmoothingSolverResult solve(
      const GeometricPath& raw_path, const PathBoundary& start,
      const PathBoundary& goal, const SmoothingLimits& limits) const = 0;
};

struct SmoothingResult {
  SmoothingStatus status{SmoothingStatus::solver_failed};
  std::optional<GeometricPath> path;
  ConstraintResiduals residuals;
  struct Audit {
    struct Objective {
      double deviation{};
      double curvature{};
      double curvature_rate{};
      double length{};
      double reference_line_proxy{};
      double total{};
    } objective;
    std::string smoother_version;
    std::string solver_status;
    std::uint64_t limits_version{};
    std::size_t solver_iterations{};
    double maximum_absolute_curvature_per_m{};
    double maximum_absolute_curvature_rate_per_m2{};
  } audit;
};

struct TrackabilityResult {
  bool valid{};
  std::string reason;
  ConstraintResiduals residuals;
};

class PathSmoother {
 public:
  PathSmoother();
  explicit PathSmoother(std::shared_ptr<const PathSmoothingSolver> solver);

  [[nodiscard]] SmoothingResult smooth(const GeometricPath& raw_path,
                                       const PathBoundary& start,
                                       const PathBoundary& goal,
                                       const SmoothingLimits& limits) const;

  [[nodiscard]] TrackabilityResult validateTrackability(
      const GeometricPath& path, const GeometricPath& raw_path,
      const PathBoundary& start, const PathBoundary& goal,
      const SmoothingLimits& limits) const;

 private:
  std::shared_ptr<const PathSmoothingSolver> solver_;
};

}  // namespace underwater_planner::core
