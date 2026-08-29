# Scout capability and energy contract v1

This contract defines the public planning inputs owned by the Scout vehicle,
thruster-health, and energy authorities. The wire schema is
`proto/underwater/contracts/v1/capability.proto`. Schema presence is not proof
that a vehicle can operate safely; the complete gates below apply atomically.

## Authorities and independent dependencies

- `ScoutCapabilityProfile` is an immutable calibrated envelope for exactly one
  vehicle and one `ScoutThrusterHealthProfile`. Its
  `capability_profile_version` and content identity are planning dependencies.
- `ScoutThrusterHealthState` is latest-value health evidence. It names every
  installed thruster and references the exact active capability profile.
- `ScoutEnergyModelProfile` is an immutable conservative power model. Its
  `energy_model_version` is independent of the capability and health versions.
- `ScoutEnergyState` is latest-value usable energy and contingency evidence. It
  references the exact energy model and never embeds a mutable model copy.

The four streams have independent producer-session, delivery-sequence,
business-version, and content-identity watermarks. An exact repeat in one
session is idempotent. Reorder, same-sequence byte conflict, business-version
rollback, same-version content conflict, and a retired-session replay are
rejected. A new producer session never revives an old business version.

Health degradation invalidates every candidate and authorization that binds the
previous health or capability dependency. A degraded state is usable for new
planning only when it references an exact calibrated degraded profile; an
adapter cannot scale a nominal envelope or invent a degraded one.

## Capability hard gate

Every profile carries finite, present, positive limits for water-relative
speed, translational and vertical acceleration/speed, yaw rate and
acceleration, roll/pitch, and translational/rotational thrust margins. The
thruster allocator version is part of the immutable identity. Margins are
fractions strictly between zero and one.

The capability profile also binds a configuration identity and the exact
unique, lexicographically ascending calibrated thruster-health combination. A
health message must contain that same complete ID/state set in canonical order;
omitting a failed unit, moving a degradation to another unit, or adding an
uncalibrated unit rejects the context.

The operating envelope binds depth, current speed, bus voltage, water density,
and `operating_domain_id`. Consumers MUST NOT extrapolate outside any bound,
including by epsilon, and must reject unknown health profiles. Boundary values
are inclusive only after the entire profile has validated.

The braking envelope binds command latency, minimum translational and yaw
deceleration, and stopping-distance margin. Zero thrust is not braking. V1
therefore requires `requires_active_thrust = true`; a missing or false value is
not evidence of a safe stop. Stopping distance is evaluated conservatively as
`v * command_latency_s + v^2 / (2 * minimum_translational_deceleration_mps2) +
stopping_distance_margin_m` by the later safety implementation.

## Energy hard gate

The model is a conservative upper bound over its exact capability profile and
operating domain:

`P = hotel + k1*|v_water| + k3*|v_water|^3 + ka*|a| + kr*|yaw_rate| + error_bound`.

All coefficients are finite and non-negative, and hotel power is positive.
The model and capability health profiles must match exactly. No consumer may
reuse a nominal power model for a degraded capability profile.

`available_energy_j` is the current usable energy before plan and contingency
deductions; it is not state-of-charge percentage. Energy is a hard constraint:

- return-required: `E_available >= E_plan + E_return + E_reserve`;
- risk-action-required: `E_available >= E_plan + E_risk_action + E_reserve`.

Unknown contingency values, non-finite values, negative requirements, missing
presence, stale state, or an inexact model reference reject the whole planning
context. Energy sufficient only for arrival is insufficient.

## Time, identity, and production evidence

Health and energy-state observation/validity ticks belong only to
`MessageHeader.source_clock_domain_id`. Consumers MUST NOT compare monotonic
ticks across clock domains. The exact Manifest-bound timing profile defines
publish, warning, and reject ages. Past the reject age or validity deadline the
latest value fails closed; recovery requires a fresh accepted message.

Capability and energy-model production profiles require non-empty vehicle and
device identities, calibration dataset, calibration method, operating domain,
calibration UTC date, business version, content identity, and explicit
production approval. `integration/v1` and all values in it are NON_PRODUCTION;
changing a copied boolean does not create production evidence because the
profile artifact hash and Manifest identity must still match exactly.

All unknown fields, unknown safety-critical enums, missing optional safety
scalars, NaN/infinity, non-NFC strings, over-limit messages, excessive
thruster counts, duplicate thruster IDs, and non-ascending fault-code sets are
rejected before installation. Content identities follow `HASHING.md`.

## Adapter boundary

C++ <-> Protobuf and ROS 2 <-> Protobuf adapters require field-by-field
bidirectional round trips for all four messages, including optional presence,
units, enums, nested profile references, and identities. Adapters MUST NOT synthesize missing values,
select fallback profiles, clamp into an operating
envelope, estimate energy reserves, reinterpret frame or clock domains, or
turn NON_PRODUCTION artifacts into production evidence.

The FCU/MAVLink adapter reports raw vehicle observations into the owning
authorities; it cannot publish a competing capability profile, energy model,
planning decision, or execution authorization.
