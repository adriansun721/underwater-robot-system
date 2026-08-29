# Scout mission lifecycle contract v1

Status: non-production contract baseline. Normative keywords **MUST**, **MUST
NOT**, and **MAY** have their usual requirements meanings.

## Scope and ownership

The laying-side mission authority is the sole publisher of `ScoutMission`,
`ScoutMissionCancellation`, and `SurveyCompletionAck`. The Scout mission
authority is the sole publisher of `ScoutMissionDecision`,
`ScoutMissionCancellationAck`, `SurveyCompletionEvidence`, and the ScoutMission
state domain. The mapping from the design term `SurveyRequest` to the public
contract is exactly one `ScoutMission`; an adapter MUST NOT assign another
request ID, version, deadline, or content identity.

`mission_id`, `mission_version`, and `mission_content_identity` together are
the sole public mission identity. Every decision, cancellation, status, plan
evidence, and completion evidence MUST bind that exact tuple. A message with a
zero ID/version, a content identity other than 32 bytes, an empty required or
allowed region, a region outside `mission_enu`, an `xyz_m` array other than the
six-value `[min_x, min_y, min_z, max_x, max_y, max_z]` AABB with strict bounds, a
non-finite scalar, a missing required coverage/resolution/business deadline,
a coverage ratio outside `[0, 1]`, a non-positive resolution/evidence age, a
zero coordination version, or a missing/unknown safety-critical enum is
invalid as a whole.

## Clocks and admission

`ScoutMission.business_deadline_monotonic_ns` is a laying-side business
deadline in `ScoutMission.header.source_clock_domain_id`. A Scout receiver
MUST NOT compare it with a Scout-local monotonic time. It may use a separately
validated clock translation for scheduling, but that translation cannot
become an execution lease or watchdog deadline.

On receipt, the Scout records its own local receive time and processes the
message within `scout_mission_admission_window_ns`. The returned
`ScoutMissionDecision.received_at_monotonic_ns` and
`admission_valid_until_monotonic_ns` are both in the decision header's Scout
clock domain. A decision SHOULD be published within
`scout_mission_decision_timeout_ns`. Expiry denies admission; it never converts
an old request into a lower-urgency request. These integration values are not
production calibration.

## Ordering, idempotency, and fail-closed results

Validation order is: manifest and resource limits; header/session/stream;
known enum and scalar/region validity; stream sequence/content conflict;
mission version/content conflict; local admission age; referenced
coordination version. The first failure determines the stable outcome. A
receiver MUST retain enough state for the active producer session and active
mission to make retransmission deterministic.

| Condition | Required result |
|---|---|
| same sequence + same content identity | idempotent duplicate; return the byte-identical prior decision/ACK |
| same sequence + different content identity | `INPUT_INVALID`; diagnose `HASH_MISMATCH` |
| lower sequence in the same producer session | `SEQUENCE_REJECTED` |
| mission version rollback | `VERSION_INCOMPATIBLE` |
| same mission version + different content identity | `INPUT_INVALID`; diagnose `HASH_MISMATCH` |
| local admission window expired | `DEPENDENCY_STALE` |
| unknown safety-critical enum | `INPUT_INVALID` |
| cancellation version rollback | `SEQUENCE_REJECTED` |
| completion without observation IDs | `INPUT_INVALID` |
| resulting map version not newer than baseline | `INPUT_INVALID` |

A new producer session does not revive a cancelled or superseded mission.
The receiver re-establishes stream ordering for that session, then still
applies the mission-version and content-conflict rules. An unknown decision,
cancellation, or completion disposition is rejected; it is never treated as
accepted, applied, or an idempotent duplicate.

Lifecycle business content identities exclude the top-level delivery header.
Therefore, the same mission version and business content retains its identity
across a legitimate producer restart, while a changed business field remains
a same-version conflict. Session/stream ordering is evaluated separately.

## Application delivery

`ScoutMission`, `ScoutMissionCancellation`, and `SurveyCompletionEvidence` use
at-least-once delivery. The publisher retries the exact bytes, including header
sequence and event ID, until it receives the matching decision/ACK or reaches
its local retry limit. `SurveyCompletionAck` binds the exact completion content
identity; accepting it acknowledges receipt and validation, not cable-laying
sufficiency. A retry MUST NOT allocate a new business version. A same-sequence
content conflict is never retried as a duplicate.

An accepted mission may drive the independent ScoutMission state domain. A
`ScoutMissionDecision` is admission only: it grants no motion authority. An
applied cancellation prevents new planning or authorization for the exact
mission and submits an event to the Scout execution authority; it does not
itself impersonate a software revocation or FCU stop command.

## Adapter requirements

The core C++ <-> Protobuf adapter and ROS 2 <-> Protobuf transport adapter MUST
map every field in both directions, preserve optional presence, preserve raw
identity bytes and repeated-element order, and reject the whole message before
creating an internal value when public validation fails. They MUST NOT synthesize or replace
a mission ID, version, content identity, business deadline, coordination
version, observation ID, map identity, disposition, or stable code.

Before an adapter enables `scout_mission_lifecycle_v1`, it MUST provide
field-by-field bidirectional tests for every lifecycle message, including zero
valid optional values, absent required safety fields, unknown enum values,
non-finite values, resource-limit overflow, duplicates, reordering, restart,
expiry, cancellation, and completion conflicts. ROS lifecycle callbacks may
schedule conversion and publication only; they cannot alter admission,
cancellation, completion, or execution-authority conclusions.

## Plan evidence and completion evidence

`SurveyPlanEvidence` is an immutable planning-time prediction. It binds the
mission plus the baseline map ID, version, and content identity and reports
conservative predicted coverage. It MUST NOT contain actual observation IDs
and cannot complete a mission.

`SurveyCompletionEvidence` is an execution-time result. It is valid only when
all of the following hold:

- it binds the accepted, non-cancelled mission identity;
- it binds the exact baseline map ID, version, and content identity used by
  the plan evidence;
- `observation_ids` is non-empty, contains no empty/duplicate IDs, and stays
  within `InterfaceLimits.maximum_repeated_items`;
- the resulting map ID is the expected map lineage and its version is strictly
  newer than `baseline_map_version`;
- the resulting map content identity is exactly 32 bytes;
- achieved coverage and resolution meet the mission requirements;
- the oldest contributing observation has synchronized, bounded-uncertainty
  audit time and satisfies `maximum_evidence_age_ns` under the agreed time
  synchronization profile.

Completion evidence proves observation and map refresh only. The laying side
still decides independently whether the new map is sufficient for cable
laying. Publishing completion evidence does not grant either robot execution
authority.
