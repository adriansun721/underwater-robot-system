#pragma once

#include "underwater_planner/core/path_smoother.hpp"
#include "underwater_planner/core/traversability_evaluator.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace underwater_planner::core {

// The audit is deliberately independent of the curvature values carried by a
// PathPoint.  It reconstructs the tangent and signed curvature from positions
// so stale or edited path metadata cannot make a candidate appear safe.
struct PathGeometryAudit {
  bool valid{};
  std::string reason;
  double maximum_heading_residual_rad{};
  double maximum_geometric_curvature_residual_per_m{};
  double maximum_absolute_geometric_curvature_per_m{};
  double maximum_curvature_rate_per_m2{};
  PathConstraintResiduals boundary_residuals;
  std::size_t worst_heading_index{};
  std::size_t worst_curvature_index{};
  std::size_t worst_curvature_rate_index{};
  Vector2m worst_heading_position_m;
  Vector2m worst_curvature_position_m;
  Vector2m worst_curvature_rate_position_m;
};

struct PathCandidateVerificationContext {
  MapSnapshot map;
  TerrainLayers terrain;
  RobotOperatingArea robot_operating_area;
  Covariance2dM2 robot_relative_obstacle_covariance_m2;
  RobotCollisionRiskPolicy collision_risk_policy;
  RobotCapability robot_capability;
  TrackFootprint track_footprint;
  TerrainGradientRiskPolicy terrain_gradient_risk_policy;
  double maximum_sweep_spacing_fraction{};
  double operating_area_clearance_m{};
  double geometric_curvature_tolerance_per_m{};
  double heading_tolerance_rad{};
  double curvature_rate_tolerance_per_m2{};
};

enum class PathCandidateVerificationStatus {
  valid,
  input_invalid,
  geometry_invalid,
  boundary_residual_exceeded,
  curvature_limit_exceeded,
  curvature_rate_limit_exceeded,
  operating_area_violation,
  collision_violation,
  traversability_violation,
};

struct PathCandidateVerificationResult {
  PathCandidateVerificationStatus status{
      PathCandidateVerificationStatus::input_invalid};
  bool valid{};
  PathGeometryAudit geometry;
  CollisionSweepResult collision;
  TraversabilityResult traversability;
  std::size_t failing_sample_index{};
  Vector2m failing_position_m;
  std::vector<std::string> issues;
};

struct PathG2MergeLimits {
  double position_tolerance_m{};
  double heading_tolerance_rad{};
  double curvature_tolerance_per_m{};
};

struct PathG2MergeResult {
  bool valid{};
  std::string reason;
  GeometricPath path;
  PathConstraintResiduals junction_residuals;
};

[[nodiscard]] PathGeometryAudit auditPathGeometry(
    const GeometricPath& path, const PathBoundary& start,
    const PathBoundary& goal, const SmoothingLimits& limits,
    double geometric_curvature_tolerance_per_m,
    double heading_tolerance_rad,
    double curvature_rate_tolerance_per_m2);

[[nodiscard]] PathG2MergeResult mergePathsG2(
    const GeometricPath& committed, const GeometricPath& tail,
    const PathG2MergeLimits& limits);

class PathCandidateVerifier {
 public:
  [[nodiscard]] PathCandidateVerificationResult verify(
      const GeometricPath& path, const PathBoundary& start,
      const PathBoundary& goal, const SmoothingLimits& smoothing_limits,
      const PathCandidateVerificationContext& context) const;
};

}  // namespace underwater_planner::core
