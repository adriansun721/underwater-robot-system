# Main-robot prediction and Scout coordination contract v1

Status: non-production contract baseline. Normative keywords **MUST**, **MUST
NOT**, and **MAY** have their usual requirements meanings.

## Scope and ownership

The laying-side prediction authority is the sole publisher of
`MainRobotPrediction`. The laying-side coordination authority is the sole
publisher of `ScoutCoordinationConstraint`. Both messages are immutable Scout
planning inputs. Neither message is an execution lease, authorization, lease
renewal, revocation, FCU command, or proof that the laying robot will execute
the predicted motion.

Every prediction and constraint binds the exact public Scout mission tuple
`(mission_id, mission_version, mission_content_identity)`. A constraint also
binds the exact `(prediction_id, prediction_version,
prediction_content_identity)`. A task or prediction mismatch rejects the
whole planning context; an adapter MUST NOT repair it by substituting the
latest locally cached object.

## Conservative moving occupancy

`MainRobotPrediction.alignment_epoch` is the common
`SynchronizedObservationTime` from which interval offsets are measured. Each
`MainRobotOccupiedInterval` is the closed interval
`[start_offset_ns, end_offset_ns]`. The first interval starts at zero;
successive intervals are contiguous, have positive duration, and share the
same boundary center. The last interval's end is a valid prediction point. A
plan ending after that point is outside the prediction horizon and is denied.

`ConservativeSweptSphere3dEnu` linearly sweeps its center in `mission_enu`.
Its `conservative_occupied_radius_m` MUST be at least
`physical_radius_m + position_uncertainty_radius_m`. Missing/non-finite
coordinates or radii, another frame, a prediction gap, an empty interval set,
or a horizon longer than the publisher validity window rejects the entire
prediction. No interpolation across a gap is allowed.

At a shared closed endpoint, a consumer uses the union of both interval
volumes. For a Scout point, separation passes only when center distance minus
the conservative occupied radius is greater than or equal to
`minimum_separation_m`. The geometric communication check passes only when
center distance is less than or equal to
`maximum_communication_distance_m`. Equality is permitted at both boundaries.

## Time domains and freshness

`source_valid_from_monotonic_ns` and `source_valid_until_monotonic_ns` belong
to each message header's `source_clock_domain_id`. They validate publisher
ordering and bound its prediction artifact; the Scout NUC does not compare
them with its own monotonic clock. Prediction and constraint source clock
domains MUST match.

The synchronized epoch is used only to align main-robot occupancy with the
Scout plan and to audit uncertainty. A consumer **MUST NOT use synchronized time to drive a local lease, deadline, or watchdog**. New exploration requires
`TIME_SYNC_SYNCHRONIZED` and uncertainty no greater than
`coordination_maximum_sync_uncertainty_ns`. Scout-local freshness is measured
from local receipt using `main_robot_prediction_reject_ns` and
`coordination_constraint_reject_ns`. Expiry returns `DEPENDENCY_STALE` and
cannot be bypassed by a still-future sender-domain validity bound. A negative
local receive age or a source-domain mismatch returns
`CLOCK_DOMAIN_MISMATCH`.

## Coordination and link-assurance boundary

The constraint channel is exactly `CHANNEL_MAIN_SCOUT_COOP`.
`minimum_separation_m` is positive and
`maximum_communication_distance_m` MUST be strictly larger; otherwise the
coordination set is structurally infeasible and rejected before search.

`LINK_ASSURANCE_GEOMETRIC_DISTANCE_ONLY` asserts only the distance boundary.
It MUST NOT carry a calibrated link model and MUST NOT be reported as a link
quality, delivery, latency, or packet-loss guarantee.
`LINK_ASSURANCE_CALIBRATED_LINK_MODEL` requires an exact immutable
`calibrated_link_model` profile reference. Unknown assurance values fail
closed.

The `scout_loss_policy` and `main_loss_policy` are independent immutable
`LossPolicy` references. A consumer cannot infer one side's action from the
other, use either reference as motion authority, or silently replace a missing
or unknown policy. Ticket 11 publishes the complete policy artifact and state
transition rules; this contract fixes the two required references and their
directional ownership.

## Ordering, recovery, and deterministic rejection

Sequence is scoped to producer, producer session, and stream. Prediction and
coordination business versions have independent watermarks. A lower version
returns `VERSION_INCOMPATIBLE`; a new delivery reusing a business version or a
same-version different identity returns `INPUT_INVALID`. Same sequence is an
idempotent duplicate only for the byte-identical delivery.

After `MAIN_SCOUT_COOP` loss, no old cached context permits new exploration.
Recovery requires a new producer session and a fresh post-boundary watermark
for both `STREAM_MAIN_ROBOT_PREDICTION` and
`STREAM_SCOUT_COORDINATION_CONSTRAINT`. A retired session cannot return.
Receiving only one refreshed stream leaves the context in resynchronization;
it does not revive the other stream or restore authorization.

Validation order is: Manifest and InterfaceLimits; header/session/stream;
known enums, finite values, frames and validity bounds; delivery and business
watermarks; mission/prediction pairing; local freshness and synchronization;
continuous occupied intervals; coordination feasibility and policies; content
identity. The first failure determines the stable result.

## Identity, resources, and adapters

`MainRobotPrediction` and `ScoutCoordinationConstraint` use the canonical
business identity rules in `HASHING.md`. Unknown fields, non-NFC strings,
invalid UTF-8, identity mismatch, too many intervals, or a serialized object
larger than its `InterfaceLimits` bound rejects the entire input. The
`scout_main_robot_coordination_v1` Manifest feature must be accepted exactly.

The planner C++ <-> Protobuf and ROS 2 <-> Protobuf adapters MUST preserve
every field, optional presence, interval order, raw identity, producer session,
clock-domain identity, assurance basis, and both LossPolicy references in both
directions. Adapter tests cover moving occupancy, shared endpoints, gaps,
expiry, task/prediction mismatch, unknown enums/fields, non-finite values,
version rollback, identity conflict, link-assurance overclaim, loss and
post-loss recovery. ROS 2 callbacks may synchronize and transport inputs; they
cannot alter prediction geometry, relax coordination limits, or grant motion
authority.
