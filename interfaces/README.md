# Public system interfaces

This directory owns the public contracts shared by the planner, control
system, both NUCs, and the main robot STM32F4 adapter.

## Authorities

- `proto/underwater/contracts/v1/` is the only semantic schema for NUC-level,
  cross-process, and cross-robot messages.
- `can/main_robot_can_v1.dbc` together with
  `can/main_robot_can_v1.md` is the only NUC-to-STM32F4 wire definition.
- `registry/codes-v1.json` is the stable Outcome/Fault/Diagnostic code
  registry and records the restricted CAN fault mapping.
- `registry/scout-state-transitions-v1.json` is the executable registry for
  Scout mission, Scout authority, and per-channel communication transitions.
- `profiles/integration-v1.json` is a non-production timing and interface
  limits profile.
- `tests/` owns the transport-independent safety conformance suite.

Message and artifact content identities follow [`HASHING.md`](HASHING.md).
The normative Scout survey-task admission, cancellation, and completion rules
are in [`SCOUT_MISSION_LIFECYCLE.md`](SCOUT_MISSION_LIFECYCLE.md).
The immutable hybrid 3D map payload, chunk reassembly, retry, identity, and
fail-closed rules are in [`HYBRID_MAP_SNAPSHOT.md`](HYBRID_MAP_SNAPSHOT.md).
The authoritative Scout 3D navigation snapshot, freshness gate, covariance
rules, and ENU/NED plus FLU/FRD adapter vectors are in
[`SCOUT_NAVIGATION_STATE.md`](SCOUT_NAVIGATION_STATE.md).
The calibrated Scout observation geometry, independent sensor health, and
bounded local-current input gates are in
[`SCOUT_SENSOR_AND_CURRENT.md`](SCOUT_SENSOR_AND_CURRENT.md).
The calibrated Scout motion/braking envelopes, thruster-health binding,
conservative power model, and hard energy-reserve gates are in
[`SCOUT_CAPABILITY_AND_ENERGY.md`](SCOUT_CAPABILITY_AND_ENERGY.md).
The laying-robot swept occupancy prediction, Scout separation/communication
constraints, synchronized alignment boundary, and directional LossPolicy
  references are in
  [`SCOUT_MAIN_ROBOT_COORDINATION.md`](SCOUT_MAIN_ROBOT_COORDINATION.md).
The non-authorizing Scout 4D plan/result surface and independent validation
bindings are in
[`SCOUT_4D_PLANNING_RESULT.md`](SCOUT_4D_PLANNING_RESULT.md).
The independent Scout-motion authority domain, atomic Bundle/lease, install
ACK, clock, interval, watermark, and fail-closed rules are in
[`SCOUT_AUTHORIZATION_BUNDLE.md`](SCOUT_AUTHORIZATION_BUNDLE.md).
The Scout FCU profile/applied/measured feedback views, exact revocation,
high-priority retry, ACK, session, and stale-evidence rules are in
[`SCOUT_EXECUTION_FEEDBACK_REVOCATION.md`](SCOUT_EXECUTION_FEEDBACK_REVOCATION.md).
The orthogonal Scout state transitions, stable codes, atomic configuration,
per-channel loss policies, and one-way recovery chain are in
[`SCOUT_STATE_CODES_PROFILES.md`](SCOUT_STATE_CODES_PROFILES.md).

ROS 1, ROS 2, and planner C++ types are adapters or internal implementation
types. They must not add public semantics.

The normative system rules are in
[`docs/system-integration-contract.md`](../docs/system-integration-contract.md).
