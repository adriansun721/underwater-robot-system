#include "storage_estimation.hpp"

#include <string>

namespace underwater_planner::core::detail {
namespace {

std::size_t string_storage_bytes(const std::string& value) {
  return value.capacity() * sizeof(char);
}

}  // namespace

std::size_t dynamic_storage_bytes(const CableConstraintMemory& memory) {
  return memory.previous_distinct_touchdown_points_m.capacity() *
             sizeof(Vector2m) +
         memory.trailing_support_samples.capacity() * sizeof(CableHistorySample);
}

std::size_t dynamic_storage_bytes(const CableState& state) {
  return dynamic_storage_bytes(state.laying_memory);
}

std::size_t dynamic_storage_bytes(const GeometricPath& path) {
  std::size_t bytes = path.points.capacity() * sizeof(PathPoint) +
                      string_storage_bytes(path.metadata.coordinate_frame) +
                      string_storage_bytes(path.metadata.interpolation_rule);
  if (path.metadata.smoothing.has_value()) {
    bytes += string_storage_bytes(path.metadata.smoothing->smoother_version) +
             string_storage_bytes(path.metadata.smoothing->solver_status);
  }
  return bytes;
}

std::size_t dynamic_storage_bytes(const CablePrediction& prediction) {
  std::size_t bytes = dynamic_storage_bytes(prediction.terminal_state) +
                      dynamic_storage_bytes(prediction.touchdown_path) +
                      prediction.robot_arc_length_profile_m.capacity() *
                          sizeof(double) +
                      prediction.state_profile.capacity() * sizeof(CableState);
  for (const CableState& state : prediction.state_profile) {
    bytes += dynamic_storage_bytes(state);
  }
  if (prediction.touchdown_covariance_profile_m2.has_value()) {
    bytes += prediction.touchdown_covariance_profile_m2->capacity() *
             sizeof(Covariance2dM2);
  }
  bytes += string_storage_bytes(prediction.dependencies.calibration_dataset_id) +
           string_storage_bytes(prediction.dependencies.operating_domain_id) +
           string_storage_bytes(
               prediction.dependencies.execution_operating_domain_id) +
           prediction.issues.capacity() * sizeof(std::string);
  for (const std::string& issue : prediction.issues) {
    bytes += string_storage_bytes(issue);
  }
  return bytes;
}

}  // namespace underwater_planner::core::detail
