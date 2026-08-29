# Contract conformance suite

Run from the system root:

```powershell
./interfaces/tests/run.ps1
```

The repository suite verifies schema compilation, stable code mapping,
canonical units/frames, timing ordering, bounded integration profiles, DBC
presence/mapping, CRC vectors, atomic authorization fields, and documented
fail-closed rules. It also compiles generated Protobuf bindings and exercises
the Scout mission admission, cancellation, completion-evidence, identity,
clock-domain, unknown-enum, hashing, and Manifest feature-gate seam.
The hybrid-map seam additionally covers all five map layers, overhanging
obstacle representation, deterministic serialization, out-of-order/duplicate
chunk reassembly, missing-chunk retry, CRC/SHA rejection, version watermarks,
resource limits, canonical identities, and its exact Manifest feature gate.
The Scout-navigation seam covers complete 3D pose and body-twist round trips,
finite/unit-attitude/covariance validation, local-clock freshness, session and
version watermarks, content identity, exact Manifest gating, and bidirectional
ENU/NED plus FLU/FRD golden vectors.
The Scout sensor/current seam covers calibrated fixed mounting, FOV/range/
resolution, conservative occlusion, independent geometry and health versions,
health freshness, bounded current applicability, complete error bounds,
optional gradients, content identities, resource limits, exact Manifest
and profile-artifact gating, session/version watermarks, same-sensor context
pairing, non-trivial mounting golden vectors, and explicit rejection of
non-production evidence.
The Scout capability/energy seam covers nominal and degraded calibrated
profiles, active-thruster-health binding, operating-envelope no-extrapolation,
active braking authority, conservative versioned power models, return/risk
action energy plus reserve, freshness, identities, resource limits, exact
Manifest gating, session/version watermarks, and explicit NON_PRODUCTION
rejection.
The main-robot prediction/Scout coordination seam covers continuous swept
three-dimensional occupancy, explicit uncertainty, closed prediction-horizon
endpoints, separation and geometric communication boundaries, exact mission
and prediction pairing, local freshness, synchronized-time alignment only,
directional LossPolicy references, stream watermarks, post-loss new-session
recovery, identities, resource limits, and its exact Manifest feature gate.
The Scout 4D planning-result seam covers dedicated non-authorizing publication,
piecewise quintic Bézier position and unwrapped-yaw reconstruction, C2 segment
continuity, complete immutable dependency capture, planned survey evidence,
independent validation-report bindings, deterministic identities, terminal
outcome priority, success/failure candidate gates, malformed/unknown/oversized
rejection, and its exact Manifest feature gate.
The execution-authority-domain seam verifies distinct main-laying and Scout
state identities, canonical publishers, local monotonic clock ownership,
cross-domain non-authority, and mixed-version failure closure.
The Scout authorization seam covers dedicated plan/lease/Bundle/ACK types,
atomic serialization, exact plan/dependency/report/profile identities, fixed
execution epoch, bounded authorized interval, wrong domain/clock/version/
profile rejection, duplicate/conflict/reorder/restart watermarks, revoked or
expired non-revival, unknown-field rejection, resource bounds, hashing, and
both exact Manifest feature gates.
The Scout execution-monitoring seam covers exact Bundle/plan/trajectory/lease
and FCU-session binding, profile/applied/measured views, explicit local limits
and risk actions, fixed-epoch time, feedback and FCU watermarks, stale/expired/
old-session rejection, tracking deviation, health/map/communication/expiry
revocation reasons, high-priority idempotent retry, ACK loss, and non-revival.
The Scout state/code/profile seam executes the exact orthogonal transition
registry, rejects illegal cross-domain transitions, checks stable Scout fault
and diagnostic safety semantics, validates atomic capability/energy/sensor
configuration references, applies distinct cooperation and FCU loss policies,
enforces timeout/resource ordering, and gates the exact Manifest feature.

Each adapter must add bidirectional tests for its own conversions:

- planner C++ type <-> Protobuf, field by field;
- ROS 1 and ROS 2 <-> Protobuf round trips;
- Protobuf physical values <-> CAN pack/unpack golden vectors;
- state transition and illegal-transition rejection;
- Bundle duplicate/reorder/restart/late-window behavior;
- revocation/ACK loss, lease expiry, and stale feedback;
- per-channel degradation/resynchronization fault injection;
- hard E-stop bypass, latch, local reset, and mandatory new Bundle;
- the assertion that no non-safe CAN command is emitted without a live exact
  Bundle.

Approved mixed-version pairs also require bidirectional old-reader/new-writer
and new-reader/old-writer suites. Schema compilation alone never approves a
pair.
