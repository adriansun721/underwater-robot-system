# Scout navigation-state contract

`ScoutNavigationState` is the sole public localization fact consumed by the
Scout planner. The localization authority publishes it; ROS, MAVLink, and core
C++ types are adapters and MUST NOT add a competing pose, velocity, health, or
freshness meaning.

## Snapshot semantics

- `navigation_version` is the localization authority's strictly increasing
  business version. Header sequence remains delivery order and is not a
  substitute for this version.
- `observed_at_monotonic_ns` is the source observation tick in
  `header.source_clock_domain_id`; `header.generated_at_monotonic_ns` is the
  later publication tick in that same domain.
- `header.observed_at` is synchronized audit time only. A consumer MUST NOT compare monotonic ticks across clock domains. Planning requires the consumer
  and snapshot to name the same Scout NUC safety clock domain.
- `pose` is the transform from `base_link` into `mission_enu`. Its quaternion
  rotates FLU body vectors into the ENU world frame and MUST have unit norm
  within the contract tolerance.
- `body_twist` contains linear and angular velocity in `base_link` (FLU:
  forward, left, up). Public world position is always `mission_enu` (east,
  north, up). NED and FRD exist only inside the FCU adapter.
- `position_covariance_m2` is the ENU x/y/z covariance in square metres.
  `attitude_covariance_rad2` is the FLU roll/pitch/yaw small-angle covariance
  in square radians. Each is exactly nine finite row-major values and MUST be
  symmetric positive semidefinite.
- `timing_profile` identifies the immutable profile that supplies publication,
  warning, and rejection ages. The integration values are non-production.
- The same accepted profile supplies `maximum_navigation_state_bytes` and the
  common string limit. Exceeding either rejects the whole snapshot; adapters
  MUST NOT truncate strings or drop fields.
- `navigation_content_identity` is the business-content SHA-256 defined in
  `HASHING.md`; it is integrity, not source authentication.

## Planning-consumer gate

The planner accepts a snapshot only when all of the following hold:

1. The header has the exact navigation stream, non-empty producer, 16-byte
   producer session, expected local clock domain, generation tick, complete
   synchronized observation audit time, and the exact negotiated Manifest
   identity with `scout_navigation_state_v1` enabled.
2. The navigation version is non-zero; the observation tick is non-negative,
   no later than generation or local receipt, and no older than the bound
   profile's `scout_navigation_reject_ns`.
3. Every required scalar is present and finite; the pose quaternion is unit;
   both covariance matrices are correctly sized, symmetric, and positive
   semidefinite.
4. All frames are exact: pose world is `mission_enu`, pose body and twist are
   `base_link`, position covariance is `mission_enu`, and attitude covariance
   is `base_link`.
5. The timing profile ID, version, and identity exactly match the accepted
   Manifest/profile; the locally recomputed navigation identity matches in
   constant time.
6. `NAVIGATION_SOLUTION_VALID` is present. NAVIGATION_SOLUTION_DEGRADED is not execution-authorizable;
   degraded data may be reported for diagnosis and risk-action selection but
   cannot seed a newly authorized plan. Invalid and unknown values fail closed.

The entire snapshot MUST be rejected when any condition fails. Receivers MUST
NOT repair frames, normalize a malformed quaternion, clamp a covariance,
replace a missing value, infer a profile, or reuse a previously valid field.

The conformance oracle uses `abs(norm_squared(q) - 1) <= 1e-6` for the unit
quaternion. For a covariance whose largest absolute element is `s >= 1`, the
symmetry and diagonal tolerances are `1e-12*s`, the 2x2 principal-minor
tolerance is `1e-12*s^2`, and the determinant tolerance is `1e-12*s^3`.
Applying the determinant-scale tolerance directly to elements is forbidden.
Unknown Protobuf fields are rejected before hashing. Canonical hashing
normalizes floating negative zero to positive zero and strings to Unicode NFC.

## Required adapter mapping

Ticket 14 must verify both `C++ <-> Protobuf` and `ROS 2 <-> Protobuf` at this
seam. Every field is mapped in both directions, field by field:

| Public field group | Required adapter value |
|---|---|
| `header` | schema/Manifest, producer/session/stream/sequence, clock, both times, event lineage |
| `navigation_version` | localization authority business version |
| `observed_at_monotonic_ns` | source-domain observation tick |
| `pose` | ENU xyz, base_link-to-mission_enu quaternion, both exact frame IDs |
| `body_twist` | all three FLU linear and all three FLU angular components plus frame |
| covariance fields | all nine row-major elements and exact frame for each matrix |
| `validity` | exact known enum value, including degraded/invalid reporting |
| `timing_profile` | exact ID, version, and 32-byte accepted profile identity |
| `navigation_content_identity` | exact 32 business-identity bytes |

Adapters MUST NOT synthesize missing values, renormalize malformed input,
drop covariance cross terms, collapse degraded into valid, or replace source
times with adapter receipt time. After either direction and a deterministic
round trip, adapter output MUST pass the same planning-consumer gate; field
presence, values, frames, enum number, version, profile identity, and content
identity must be unchanged.

## Version and session ordering

Within one producer session, a higher navigation version advances the current
fact. Equal version and equal content identity is an idempotent duplicate;
equal version with different identity is an integrity conflict. A lower version
is rollback. Header sequence is strictly increasing in the same session. Only
the exact same sequence, navigation version, content identity, and complete
serialized message bytes is an idempotent retransmission. A consumer may cache
a local SHA-256 of those exact bytes for constant-time comparison; this local
delivery fingerprint is not a new public identity. A lower sequence is reorder,
and a higher sequence cannot reuse the current navigation version. A new producer session starts a
new non-zero sequence/version ordering scope, records the
previous session as retired, and invalidates cached freshness or safety
conclusions from it. A retired session ID MUST be rejected if replayed; it
cannot become a new session again during the consumer process lifetime.

## FCU adapter conversion golden vectors

The FCU/MAVLink adapter is the only boundary allowed to expose NED or FRD. Its
world-vector rotation is `(E, N, U) -> (N, E, -U)` and its body-vector rotation
is `(F, L, U) -> (F, -L, -U)`. Both rotations are their own inverses. They apply
to positions, polar vectors, angular vectors, and covariance bases; covariance
conversion uses `R * covariance * transpose(R)` rather than individual sign
patches.

For instantaneous yaw, normalize after applying
`yaw_ned = pi/2 - yaw_enu`. The required cardinal vectors are:

| Public input | FCU output |
|---|---|
| ENU east `(1, 0, 0)` | NED east `(0, 1, 0)` |
| ENU north `(0, 1, 0)` | NED north `(1, 0, 0)` |
| ENU up `(0, 0, 1)` | NED down `(0, 0, -1)` |
| FLU forward `(1, 0, 0)` | FRD forward `(1, 0, 0)` |
| FLU left `(0, 1, 0)` | FRD right-axis value `(0, -1, 0)` |
| FLU up `(0, 0, 1)` | FRD down-axis value `(0, 0, -1)` |
| ENU yaw `0` (east) | NED yaw `pi/2` (east) |
| ENU yaw `pi/2` (north) | NED yaw `0` (north) |
| positive FLU yaw rate | negative FRD yaw-rate component |

Every FCU adapter MUST run these vectors in both directions, including the
inverse round trip, before its conversion is accepted. Quaternion conversion
must compose the same fixed world/body basis rotations and then pass the
cardinal-direction tests; swapping quaternion components is not an accepted
substitute.
