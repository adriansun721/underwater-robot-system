# Scout 4D planning-result contract v1

## Scope and authority

`ScoutPlanningResult` is the sole public publication seam for a Scout planner
solve. A successful result may contain an immutable `ScoutPlan`; it MUST NOT authorize motion or grant execution authority. Only the later Scout-specific
authorization bundle may grant a finite execution prefix. The existing
`PlanningResult`, `ImmutablePlan`, and track/cable `ExecutionProfile` remain
exclusive to the laying-robot authorization domain.

The canonical publisher is `ScoutMotionPlanner` on the Scout NUC. Its header
uses `STREAM_SCOUT_PLANNING_RESULT`, a Scout-NUC boot-scoped monotonic clock
domain, and the exact `scout_4d_planning_result_v1` Manifest feature. A new
producer process creates a new session; a retired session cannot resume.

## Canonical four-dimensional trajectory

`ScoutTrajectory4d.frame_id` is exactly `mission_enu`. The trajectory is an
ordered sequence of degree-five Bézier segments. Each segment has exactly six
finite `ScoutBezierControlPoint3dEnu` values, exactly six finite
`yaw_offset_control_points_rad` values, a contiguous
`start_time_offset_ns`, and a strictly positive `duration_ns`.

`initial_yaw_rad` is an instantaneous ENU yaw and is normalized to
`[-pi, pi)`. The first yaw offset is zero. Every yaw offset is a continuous
rotation displacement relative to `initial_yaw_rad`; offsets are never wrapped
at `+pi/-pi` and do not reset at segment boundaries. Instantaneous display or
feedback yaw is reconstructed and then normalized. Adding `2*pi` to an offset
changes the trajectory and its content identity.

Adjacent segments are contiguous in time and satisfy position and yaw-offset
`C2` continuity after segment duration is applied to both first and second
derivatives. A missing control value, non-finite value, wrong control count,
zero duration, time gap/overlap, discontinuity, invalid frame, or non-normalized
initial yaw rejects the whole trajectory as `INPUT_INVALID`.

## Immutable candidate and dependencies

`ScoutPlan` binds:

- its plan sequence and Scout-domain creation time;
- the canonical `ScoutTrajectory4d`;
- one `ScoutPlanningDependencies` snapshot;
- one `SurveyPlanEvidence` prediction; and
- one independent `ScoutPlanValidationReport`.

The dependency snapshot captures the exact mission, hybrid map, navigation
state, ordered sensor geometry/health pairs, current model, capability profile,
thruster health, energy model/state, main-robot prediction, coordination
constraint, and planner-configuration versions and content identities used by
the solve, together with the exact `TimingProfile` and `InterfaceLimits`
references. Sensor entries are unique and ascending by `sensor_id`. No adapter
may substitute a latest value, infer an omitted identity, or combine versions
from different captures.

`SurveyPlanEvidence` is only a model-based coverage prediction. It binds the
same mission and baseline map as the candidate. It is not
`SurveyCompletionEvidence`, does not prove that an observation occurred, and
does not prove that the map advanced.

The independent validator receives the immutable trajectory, dependencies,
and survey evidence. A candidate is eligible for later authorization review
only when `ScoutPlanValidationReport.status` is
`SCOUT_PLAN_VALIDATION_SAFE`, its primary outcome is `OUTCOME_SUCCESS`, and
its three validated content identities exactly match those candidate inputs.
For a `SAFE` report, collision, separation, energy and capability margins plus
survey coverage are all present, finite and non-negative; coverage is within
`[0, 1]`, and `earliest_failure_time_offset_ns` is absent. Optimizer success is
not validation evidence.

## Result gates and terminal outcomes

`OUTCOME_SUCCESS` must carry exactly one candidate. Every non-success result
must omit the candidate and carry a valid `evaluated_at_monotonic_ns`, a hashed
dependency summary containing every dependency accepted before termination,
and at least one structured `CodeRef` diagnostic. Zero-valued or unavailable
dependency facts stay explicitly unavailable; consumers must not replace them
with defaults. The receiver recomputes `dependencies_content_identity` for a
failure just as it does for a success; a 32-byte placeholder is not valid.

The Scout result vocabulary is:

- `OUTCOME_SUCCESS`;
- `OUTCOME_INPUT_INVALID`;
- `OUTCOME_DEPENDENCY_STALE`;
- `OUTCOME_CAPABILITY_INFEASIBLE`;
- `OUTCOME_ENERGY_INSUFFICIENT`;
- `OUTCOME_COORDINATION_INFEASIBLE`;
- `OUTCOME_NO_SOLUTION` for the design's `NO_PATH` result;
- `OUTCOME_SMOOTHING_FAILED`;
- `OUTCOME_SURVEY_INFEASIBLE`;
- `OUTCOME_VALIDATION_REJECTED`;
- `OUTCOME_VALIDATION_INCONCLUSIVE`;
- `OUTCOME_TIMEOUT`;
- `OUTCOME_CANCELLED`; and
- `OUTCOME_NUMERICALLY_INVALID`.

When multiple failures exist, the single primary outcome follows the order
above beginning with `INPUT_INVALID`, then `DEPENDENCY_STALE`, capability,
energy, coordination, `NO_SOLUTION`, smoothing, survey, rejected validation,
inconclusive validation, `TIMEOUT`, `CANCELLED`, and finally
`NUMERICALLY_INVALID`. Secondary causes remain structured diagnostics. This
priority is normative and is not derived from enum numeric order.

## Identity, limits, and failure closure

Consumers recompute all nested identities according to `HASHING.md` before
acceptance, compare the 32 bytes in constant time, and reject any mismatch.
The result's candidate dependencies and top-level dependency summary must be
byte-identical. Unknown fields, unknown enums, non-NFC strings, non-finite
values, missing presence, invalid ordering, identity conflict, and exact
Manifest mismatch fail closed.

`InterfaceLimits.maximum_scout_planning_result_bytes` bounds the complete
message and `maximum_scout_plan_segments` bounds the trajectory. Existing
sensor and diagnostic limits also apply. A receiver returns
`RESOURCE_LIMIT_EXCEEDED`; it must not truncate, resample, or partially install
an oversized candidate. The `integration/v1` values are `NON_PRODUCTION` and
are not vehicle capability, safety, latency, or certification evidence.

## Adapter requirements

The planner C++ type and Protobuf representation require a field-by-field,
bidirectional round trip for every dependency, optional presence bit, control
point, yaw offset, report margin, outcome, diagnostic, and identity. ROS 2
adapters perform only transport, synchronization, and conversion. They must
not normalize yaw offsets, regenerate a trajectory, change a validation
decision, inject latest dependencies, or treat publication success as
execution authorization.

Generated-code round trips alone are insufficient. Adapter suites also cover
crossing `+pi/-pi`, negative zero/NFC canonicalization, malformed segments,
unknown fields/enums, result/candidate gate inversions, hash tampering,
duplicate/reorder/session restart, resource limits, and exact Manifest feature
rejection.
