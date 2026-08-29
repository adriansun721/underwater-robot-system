#include "underwater_planner/core/path_candidate_verifier.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

using namespace underwater_planner::core;

void require(const bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

GeometricPath straight_path(const double y = 2.0) {
  GeometricPath path;
  path.points = {{0.0, 2.0, y, 0.0, 0.0}, {2.0, 4.0, y, 0.0, 0.0},
                 {4.0, 6.0, y, 0.0, 0.0}};
  path.metadata = {27U, "map", 4U, "constant-curvature-exact"};
  return path;
}

SmoothingLimits smoothing_limits() {
  SmoothingLimits limits;
  limits.version = 1U;
  limits.output_path_version = 2U;
  limits.minimum_segment_length_m = 0.01;
  limits.maximum_curvature_per_m = 0.5;
  limits.maximum_curvature_rate_per_m2 = 0.5;
  limits.maximum_boundary_time_skew.nanoseconds = 0;
  limits.allowed_residuals.start_position_residual_m = 1.0e-8;
  limits.allowed_residuals.start_heading_residual_rad = 1.0e-8;
  limits.allowed_residuals.start_curvature_residual_per_m = 1.0e-8;
  limits.allowed_residuals.goal_position_residual_m = 1.0e-8;
  limits.allowed_residuals.goal_heading_residual_rad = 1.0e-8;
  limits.allowed_residuals.goal_curvature_residual_per_m = 1.0e-8;
  return limits;
}

PathBoundary start_boundary() {
  PathBoundary boundary;
  boundary.x_m = 2.0;
  boundary.y_m = 2.0;
  boundary.heading_rad = 0.0;
  boundary.curvature_per_m = 0.0;
  boundary.curvature_source = PathBoundarySource::synchronized_actual_state;
  boundary.pose_timestamp.nanoseconds = 1;
  boundary.curvature_timestamp.nanoseconds = 1;
  boundary.source_sequence_number = 1U;
  return boundary;
}

PathBoundary goal_boundary() {
  PathBoundary boundary;
  boundary.x_m = 6.0;
  boundary.y_m = 2.0;
  boundary.heading_rad = 0.0;
  boundary.curvature_per_m = 0.0;
  boundary.curvature_source = PathBoundarySource::planned_goal;
  return boundary;
}

PathCandidateVerificationContext context() {
  PathCandidateVerificationContext result;
  result.map.version = {"audit-map", 1U, MonotonicTime{1}, "map"};
  result.map.width = 40U;
  result.map.height = 40U;
  result.map.resolution_m = 0.25;
  result.map.derived_configuration_version = 9U;
  result.map.cells.assign(result.map.width * result.map.height,
                          MapCell{0.0, 0.01, 1.0, true, MonotonicTime{1}});
  result.terrain.source_map_version = result.map.version;
  result.terrain.analysis_config_version = 9U;
  result.terrain.operating_domain_id = "audit-domain";
  result.terrain.surface.width = result.map.width;
  result.terrain.surface.height = result.map.height;
  result.terrain.surface.resolution_m = result.map.resolution_m;
  result.terrain.surface.cells.assign(result.map.width * result.map.height,
                                      SurfaceEstimate{});
  for (SurfaceEstimate& cell : result.terrain.surface.cells) {
    cell.status = TerrainEstimateStatus::valid;
    cell.support_ratio = 1.0;
  }
  result.robot_operating_area = {1U, "area", {{0.0, 0.0}, {10.0, 0.0},
                                               {10.0, 10.0}, {0.0, 10.0}}};
  result.collision_risk_policy = {1U, "collision-cal-v1", "audit-domain",
                                  0.1, 0.5, 0.0};
  result.robot_relative_obstacle_covariance_m2 = {0.0, 0.0, 0.0, 0.0};
  result.robot_capability = {1.0, 1.0, 1.0, 1.0, 0.3, 0.3, 0.5, 0.5,
                             0.2, 0.1, 1.0};
  result.track_footprint = {{{-0.4, -0.4}, {0.4, -0.4}, {0.4, 0.4},
                             {-0.4, 0.4}},
                            {{-0.35, 0.1}, {0.35, 0.1}, {0.35, 0.35},
                             {-0.35, 0.35}},
                            {{-0.35, -0.35}, {0.35, -0.35}, {0.35, -0.1},
                             {-0.35, -0.1}}};
  result.terrain_gradient_risk_policy =
      {1U, 9U, 0.05, 2.0, GradientCoverageModel::empirical_bounded,
       "terrain-cal-v1", "audit-domain", true};
  result.maximum_sweep_spacing_fraction = 0.5;
  result.operating_area_clearance_m = 0.0;
  result.geometric_curvature_tolerance_per_m = 1.0e-9;
  result.heading_tolerance_rad = 1.0e-9;
  result.curvature_rate_tolerance_per_m2 = 1.0e-9;
  return result;
}

void metadata_tampering_is_detected() {
  // Design: 18.2.5-5
  GeometricPath path = straight_path();
  path.points[1].curvature_per_m = 0.2;
  const PathGeometryAudit audit = auditPathGeometry(
      path, start_boundary(), goal_boundary(), smoothing_limits(), 1.0e-6,
      1.0e-6, 1.0e-6);
  require(!audit.valid && audit.reason == "geometric_curvature_residual_exceeded",
          "independent three-point curvature audit missed metadata tampering");
}

void independent_audit_enforces_start_curvature_provenance() {
  // Design: 18.2.5-12
  PathBoundary invalid_start = start_boundary();
  invalid_start.curvature_source = PathBoundarySource::planned_goal;
  const PathGeometryAudit direct_audit = auditPathGeometry(
      straight_path(), invalid_start, goal_boundary(), smoothing_limits(),
      1.0e-9, 1.0e-9, 1.0e-9);
  const PathCandidateVerifier verifier;

  const PathCandidateVerificationResult first = verifier.verify(
      straight_path(), invalid_start, goal_boundary(), smoothing_limits(),
      context());
  const PathCandidateVerificationResult replay = verifier.verify(
      straight_path(), invalid_start, goal_boundary(), smoothing_limits(),
      context());

  require(!direct_audit.valid &&
              direct_audit.reason == "start_boundary_provenance_invalid" &&
              !first.valid &&
              first.status == PathCandidateVerificationStatus::input_invalid &&
              first.geometry.reason == "start_boundary_provenance_invalid" &&
              first.issues.size() == 1U &&
              first.issues.front() == "start_boundary_provenance_invalid" &&
              first.collision.evaluated_sweep_poses == 0U &&
              first.traversability.evaluated_sweep_poses == 0U,
          "planned-goal start curvature reached complete-path sweep gates");
  require(replay.valid == first.valid && replay.status == first.status &&
              replay.geometry.reason == first.geometry.reason &&
              replay.issues == first.issues &&
              replay.collision.evaluated_sweep_poses ==
                  first.collision.evaluated_sweep_poses &&
              replay.traversability.evaluated_sweep_poses ==
                  first.traversability.evaluated_sweep_poses,
          "planned-goal start rejection was not deterministic");

  const PathCandidateVerificationResult actual = verifier.verify(
      straight_path(), start_boundary(), goal_boundary(), smoothing_limits(),
      context());
  PathBoundary committed_start = start_boundary();
  committed_start.curvature_source =
      PathBoundarySource::committed_segment_terminal;
  committed_start.source_sequence_number = 2U;
  const PathCandidateVerificationResult committed = verifier.verify(
      straight_path(), committed_start, goal_boundary(), smoothing_limits(),
      context());
  require(actual.valid && committed.valid &&
              actual.collision.evaluated_sweep_poses > 0U &&
              committed.collision.evaluated_sweep_poses > 0U &&
              goal_boundary().curvature_source ==
                  PathBoundarySource::planned_goal,
          "a legal start source or planned-goal terminal boundary was rejected");
}

void invalid_start_boundary_metadata_stops_before_sweep() {
  // Design: 18.2.5-12
  PathBoundary missing_curvature = start_boundary();
  missing_curvature.curvature_per_m.reset();
  PathBoundary zero_sequence = start_boundary();
  zero_sequence.source_sequence_number = 0U;
  PathBoundary negative_pose_time = start_boundary();
  negative_pose_time.pose_timestamp.nanoseconds = -1;
  PathBoundary negative_curvature_time = start_boundary();
  negative_curvature_time.curvature_timestamp.nanoseconds = -1;
  PathBoundary skewed_time = start_boundary();
  skewed_time.curvature_timestamp.nanoseconds = 2;
  const std::array<PathBoundary, 5> invalid_starts{
      missing_curvature, zero_sequence, negative_pose_time,
      negative_curvature_time, skewed_time};
  const std::array<const char*, 5> expected_reasons{
      "path_geometry_or_boundary_invalid", "path_geometry_or_boundary_invalid",
      "path_geometry_or_boundary_invalid", "path_geometry_or_boundary_invalid",
      "boundary_timestamp_skew_exceeded"};
  const PathCandidateVerifier verifier;

  for (std::size_t index = 0U; index < invalid_starts.size(); ++index) {
    const PathCandidateVerificationResult result = verifier.verify(
        straight_path(), invalid_starts[index], goal_boundary(),
        smoothing_limits(), context());
    require(!result.valid &&
                result.status == PathCandidateVerificationStatus::input_invalid &&
                result.geometry.reason == expected_reasons[index] &&
                result.issues.size() == 1U &&
                result.issues.front() == expected_reasons[index] &&
                result.collision.evaluated_sweep_poses == 0U &&
                result.traversability.evaluated_sweep_poses == 0U,
            "invalid start-boundary metadata reached a sweep gate");
  }
}

void g2_merge_rejects_junction_mismatch() {
  // Design: 18.2.5-4
  // Design: 18.2.5-invariant-2
  GeometricPath committed = straight_path();
  GeometricPath tail = straight_path();
  tail.points = {{0.0, 6.0, 2.0, 0.0, 0.0},
                 {1.0, 7.0, 2.0, 0.0, 0.0},
                 {2.0, 8.0, 2.0, 0.0, 0.0}};
  const PathG2MergeLimits limits{0.01, 0.01, 0.01};

  GeometricPath position_mismatch = tail;
  position_mismatch.points.front().x_m += 0.02;
  const PathG2MergeResult position =
      mergePathsG2(committed, position_mismatch, limits);
  require(!position.valid &&
              position.reason == "g2_junction_residual_exceeded" &&
              position.junction_residuals.start_position_residual_m >
                  limits.position_tolerance_m &&
              position.junction_residuals.start_heading_residual_rad == 0.0 &&
              position.junction_residuals.start_curvature_residual_per_m == 0.0,
          "G2 merge did not isolate a position residual");

  GeometricPath heading_mismatch = tail;
  heading_mismatch.points.front().heading_rad = 0.02;
  const PathG2MergeResult heading =
      mergePathsG2(committed, heading_mismatch, limits);
  require(!heading.valid &&
              heading.reason == "g2_junction_residual_exceeded" &&
              heading.junction_residuals.start_position_residual_m == 0.0 &&
              heading.junction_residuals.start_heading_residual_rad >
                  limits.heading_tolerance_rad &&
              heading.junction_residuals.start_curvature_residual_per_m == 0.0,
          "equal-position heading discontinuity bypassed the G2 merge gate");

  GeometricPath curvature_mismatch = tail;
  curvature_mismatch.points.front().curvature_per_m = -0.02;
  const PathG2MergeResult curvature =
      mergePathsG2(committed, curvature_mismatch, limits);
  require(!curvature.valid &&
              curvature.reason == "g2_junction_residual_exceeded" &&
              curvature.junction_residuals.start_position_residual_m == 0.0 &&
              curvature.junction_residuals.start_heading_residual_rad == 0.0 &&
              curvature.junction_residuals.start_curvature_residual_per_m >
                  limits.curvature_tolerance_per_m,
          "equal-position curvature discontinuity bypassed the G2 merge gate");
}

void merged_path_sweep_rechecks_the_committed_prefix_on_the_current_map() {
  // Design: 18.2.5-7
  const GeometricPath committed = straight_path();
  GeometricPath tail = straight_path();
  tail.points = {{0.0, 6.0, 2.0, 0.0, 0.0},
                 {1.0, 7.0, 2.0, 0.0, 0.0},
                 {2.0, 8.0, 2.0, 0.0, 0.0}};
  const PathG2MergeResult merged = mergePathsG2(
      committed, tail, PathG2MergeLimits{1.0e-9, 1.0e-9, 1.0e-9});
  require(merged.valid,
          "test setup did not produce an individually G2-valid merge");

  PathCandidateVerifier verifier;
  const auto committed_clear = verifier.verify(
      committed, start_boundary(), goal_boundary(), smoothing_limits(),
      context());
  require(committed_clear.valid,
          "the committed segment was not legal on its approved map");

  PathCandidateVerificationContext updated = context();
  updated.map.cells.at(8U * updated.map.width + 16U).obstacle = true;
  PathBoundary tail_start = start_boundary();
  tail_start.x_m = 6.0;
  PathBoundary tail_goal = goal_boundary();
  tail_goal.x_m = 8.0;
  const auto tail_clear = verifier.verify(
      tail, tail_start, tail_goal, smoothing_limits(), updated);
  require(tail_clear.valid,
          "the new tail was not individually legal on the updated map");

  PathBoundary merged_goal = goal_boundary();
  merged_goal.x_m = 8.0;
  const auto rejected = verifier.verify(
      merged.path, start_boundary(), merged_goal, smoothing_limits(), updated);
  require(!rejected.valid &&
              rejected.status ==
                  PathCandidateVerificationStatus::collision_violation &&
              std::isfinite(rejected.failing_position_m.x_m) &&
              std::isfinite(rejected.failing_position_m.y_m) &&
              std::abs(rejected.failing_position_m.x_m - 4.0) < 1.0,
          "the complete merged sweep missed a new committed-prefix obstacle");
}

void complete_path_is_swept_and_rechecked() {
  PathCandidateVerifier verifier;
  const PathCandidateVerificationResult result = verifier.verify(
      straight_path(), start_boundary(), goal_boundary(), smoothing_limits(),
      context());
  require(result.valid &&
              result.status == PathCandidateVerificationStatus::valid &&
              result.collision.evaluated_sweep_poses > 2U &&
              result.traversability.evaluated_sweep_poses > 0U,
          "a valid complete path did not pass independent sweep recheck");

  PathCandidateVerificationContext blocked = context();
  blocked.map.cells.at(8U * blocked.map.width + 16U).obstacle = true;
  const PathCandidateVerificationResult rejected = verifier.verify(
      straight_path(), start_boundary(), goal_boundary(), smoothing_limits(),
      blocked);
  require(!rejected.valid &&
              rejected.status == PathCandidateVerificationStatus::collision_violation,
          "complete-path obstacle was not rejected");
}

}  // namespace

int main() {
  try {
    metadata_tampering_is_detected();
    independent_audit_enforces_start_curvature_provenance();
    invalid_start_boundary_metadata_stops_before_sweep();
    g2_merge_rejects_junction_mismatch();
    merged_path_sweep_rechecks_the_committed_prefix_on_the_current_map();
    complete_path_is_swept_and_rechecked();
  } catch (const std::exception& error) {
    std::cerr << "T27 failure: " << error.what() << '\n';
    return 1;
  }
  std::cout << "T27 path candidate verifier checks passed\n";
  return 0;
}
