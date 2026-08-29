# Current planner to public-contract mapping

This is an adapter guide, not a second schema. The current planner C++ types
remain internal. Protobuf remains authoritative at the cross-process boundary.

| Current planner C++ | Public v1 mapping | Required adapter check |
|---|---|---|
| `MonotonicTime` | `*_monotonic_ns` plus `source_clock_domain_id` | Must be the main-NUC safety Clock Domain; restart invalidates authority |
| `Pose2d.heading_rad` / `PathPoint.heading_rad` | `yaw_rad` | Prove source is `mission_enu`, normalize to `[-pi, pi)`, reject NED/FRD |
| `MapVersion.coordinate_frame` | `MapVersion.frame_id` | Must be `mission_enu` at the public boundary |
| `PlanningResult.sequence_number` | result/plan sequence assigned at the appropriate owner boundary | Non-zero and strictly monotonic in its producer session |
| `PlanningResult.timestamp` | plan/result creation monotonic time | Never use as UTC or a lease expiry |
| `PlanningResult.validity_duration` | initial candidate acceptance policy | Does not become or extend an `ExecutionLease` |
| `TimedPath.geometry` | `GeometricPath` | SI, finite, bounded, yaw-normalized, same reference version |
| `ExecutionProfile` | public `ExecutionProfile` | Copy all samples and approved envelope without resampling or target changes |
| `PlanningDependencyVersions` | public dependency message | Exact complete tuple; no field fallback |
| `ImmutablePlanningResult` | `ImmutablePlan` candidate | Recompute canonical identity after lossless mapping |
| `PlanValidationLease` | public `ExecutionLease` | Bind exact plan/profile/dependencies/remainder and shared Clock Domain |
| internal authorized plan + remainder + lease | input to ExecutionAuthority Bundle assembly | Only ExecutionAuthority may assign Bundle identity/sequence and publish it |
| current `ExecutionFeedback` | `measured_state` portion | Adapter separately supplies profile target, applied target, CAN state, and local override |
| `PlanningState::communication_degraded` | planning-internal outcome only | Public control behavior uses per-ChannelId CommunicationState/LossPolicy; do not export it as a global control state |
| `DiagnosticEntry.code/message` | registry code plus optional human text | No free string may drive control; unmapped safety reason is rejected |

## Required boundary behavior

1. Validate the internal object using the planner's own contract.
2. Map every field explicitly; no C++ structure memory-copy ABI is permitted.
3. Apply public presence, SI, finite/range, Clock Domain, frame, version, and
   InterfaceLimits validation.
4. Compute the public canonical identity according to `interfaces/HASHING.md`.
5. Submit the candidate to ExecutionAuthority. Mapping success alone does not
   authorize execution.

## Corrected control responsibility

The older control-framework document says in places that `mission_manager`
calculates `T_set` and `v_cable_ff`. That statement is superseded at the
planner/control boundary. Those values originate in the planner-owned joint
ExecutionProfile. `MissionManager` manages mission state and submits authority
events; it does not recalculate them. `ExecutionAdapter` samples and converts
the profile and reports any conservative limiting.

