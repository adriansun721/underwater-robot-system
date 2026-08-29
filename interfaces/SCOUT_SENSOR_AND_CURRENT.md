# Scout sensor geometry, health, and current contract v1

This document is normative for the planning inputs published in
`sensing.proto`. Protobuf is the sole public semantic authority. C++, ROS 2,
sensor-driver, and FCU types are adapters only.

## 1. Feature and publishers

The exact Contract Manifest feature is `scout_sensor_and_current_v1`.
Installation requires that exact feature and Manifest identity before any
message in this contract can enter a planning context.

- The calibrated-sensor configuration authority is the only canonical
  publisher of `ScoutSensorGeometry` for a `sensor_id`.
- The device-health authority is the only canonical publisher of
  `ScoutSensorHealthState` for that sensor.
- The current estimator is the only canonical publisher of
  `ScoutCurrentEstimate` for a `current_model_id`.

Each publisher has its own producer session, sequence watermark, and business
version. A new producer session retires the old session; an old session cannot
be resurrected. Same-session duplicate delivery is idempotent only when the
complete bytes, including the header, are identical.

Within a live session, each stream sequence and its corresponding geometry,
health, or current-model business version are strictly increasing. A lower
sequence or business version, a reused version on a new delivery, or a
same-sequence delivery with different complete bytes is rejected. Consumers
retain retired session IDs and reject their replay.

Message sequence is session-scoped; business version is not. Across a producer
restart, a lower business version remains a rollback and is rejected. The same
business version is accepted in a new delivery session only when its exact
business content identity is unchanged.

## 2. Calibrated sensor geometry

`ScoutSensorGeometry` is an immutable geometry/configuration snapshot. It
binds all of the following:

- stable `sensor_id`, sensor type, and `geometry_version`;
- fixed `base_link`/FLU-to-sensor translation and a unit quaternion rotating
  sensor-frame vectors into `base_link`/FLU;
- full horizontal and vertical FOV about sensor +X, minimum/maximum range,
  range resolution, and both angular resolutions;
- the conservative occlusion policy;
- operating domain, device serial, calibration dataset and method versions,
  calibration UTC provenance, production-approval flag, and content identity.

All optional numeric fields listed by the schema are required at the consumer
gate and must be finite. Frames other than exact `base_link` and the declared
non-empty sensor frame are rejected. The quaternion squared norm must be
within `1e-9` of one. FOVs and resolutions must be positive; range must satisfy
`0 <= minimum_range_m < maximum_range_m`. Visibility range is the inclusive
interval `[minimum_range_m, maximum_range_m]`; values outside it are rejected.

The mounting quaternion rotates sensor-frame vectors into `base_link`/FLU.
The conformance golden vector for a +90 degree FLU yaw mount is quaternion
`(x=0, y=0, z=sqrt(1/2), w=sqrt(1/2))`, which maps the sensor +X boresight to
base_link +Y (left), not -Y.

The only v1 production-authorizable occlusion policy is
`SENSOR_OCCLUSION_KNOWN_FREE_RAYCAST_REQUIRED`. Every visibility ray stops at
UNKNOWN, CONFLICTED, occupied, or out-of-map space. No adapter may replace it
with optimistic line of sight.

`production_approved=true` is meaningful only with a separately released
production profile whose device, calibration dataset, operating domain, date,
version, and content identity have passed the production process. The checked
in `integration/v1` profile and its example calibrations are NON_PRODUCTION.
The installed profile's exact artifact SHA-256, ID, version, and production
flag must match the accepted Manifest. Changing an in-memory production flag
or geometry approval bit without a newly accepted Manifest cannot promote a
test profile.

## 3. Independent sensor health

`ScoutSensorHealthState.health_version` is independent of
`ScoutSensorGeometry.geometry_version`. PlanningContext captures both exact
version/content-identity pairs. Any change to either dependency requires plan revalidation.
A health change does not silently edit geometry, range, or resolution. A
geometry change does not reset or advance the health watermark.

Health observation and validity ticks belong to
`header.source_clock_domain_id`. Consumers require the exact Scout-local clock
domain, `observed <= now <= valid_until`, and the stricter profile rejection
age. They MUST NOT compare monotonic ticks across clock domains.

Only `SCOUT_SENSOR_HEALTH_NOMINAL` enters new v1 planning. DEGRADED triggers
revalidation and needs a newly approved geometry profile before it can become
usable; UNCALIBRATED, FAILED, UNSPECIFIED, stale, or expired state fails closed.
Fault-code numeric identities are unique and strictly ascending.
For a new plan the accepted NOMINAL health state has no active fault codes;
every non-empty active set fails closed. Other health telemetry may only carry
numeric identities present in the exact Manifest-bound code registry.

## 4. Bounded current estimate

`ScoutCurrentEstimate` is a bounded local affine model in `mission_enu`.
It binds model ID/version/source, operating domain, source-clock observation
and validity interval, a strict ENU AABB, a reference position, reference
velocity, non-negative component error bounds, a Euclidean speed-error bound,
optional spatial gradient, validity, and content identity.

The speed-error bound must be present, finite, non-negative, and at least the
Euclidean norm of the three component bounds. Therefore an incomplete error
bound never enters planning. When `spatial_gradient` is absent, velocity is
uniform over the AABB within the same complete error bound; no gradient is
synthesized. When present, it is exactly nine finite row-major `dc_i/dp_j`
values in `mission_enu`, with units `1/s`.

The reference position must lie inside the applicable AABB. A candidate may
query the model only while its position is inside that AABB and its local time
is inside `[valid_from, valid_until]`. The profile rejection age is an
additional upper bound. `CURRENT_ESTIMATE_DEGRADED`, INVALID, or UNSPECIFIED
does not authorize new exploration in v1.

Synchronized observation time in the header is for cross-device alignment and
audit only. Local validity and freshness use only ticks from the exact source
clock domain. They MUST NOT compare monotonic ticks across clock domains and
must never use this current-validity interval as an execution lease.

## 5. Whole-object rejection and dependency changes

Uncalibrated, stale, out-of-domain, frame-mismatched, non-finite, unknown-enum,
unknown-field, identity-mismatched, Manifest-mismatched, version-regressed, or
resource-limit-exceeding input is rejected as a whole. Consumers do not clamp,
truncate, discard unknown fields, infer a frame, or retain selected fields from
an invalid object.

An accepted plan records each sensor geometry pair, each sensor health pair,
and the current model/version/content identity. Geometry or health changes,
current version changes, validity expiry, or leaving the current AABB makes the
affected plan require revalidation. A stale dependency cannot be repaired by
updating only its version number.

Geometry and health are installed together only when their exact `sensor_id`
values match. A health snapshot for another mounting cannot satisfy the
geometry dependency even when both objects pass their individual gates.

## 6. Adapter conformance

Every adapter has bidirectional, field-complete seam tests for:

- C++ <-> Protobuf;
- ROS 2 <-> Protobuf;
- sensor/estimator native frames <-> canonical FLU/ENU values.

Every field is mapped in both directions, including optional presence,
versions, clock domains, calibration provenance, error bounds, profiles, and
content identities. Adapters MUST NOT synthesize missing values, relax
occlusion, replace an absent gradient with zeros, or label test calibration as
production. Adapter output must pass the same planning-consumer gate.
