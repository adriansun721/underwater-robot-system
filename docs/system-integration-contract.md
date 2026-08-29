# Planner–control system integration contract

Status: v1 integration baseline, non-production  
Normative keywords: **MUST**, **MUST NOT**, **SHOULD**, and **MAY** are used in
their usual requirements sense.

## 1. Scope and authority

This contract joins the cable-laying planner to the control system while
preserving the three-layer architecture:

1. NUC planning and coordination;
2. NUC execution adaptation and authority;
3. STM32F4 local track, tension/payout, mechanism, and safety loops.

The planner is the sole owner of the immutable joint `ExecutionProfile`. It
generates ground-motion, payout-speed/acceleration, and tension targets as one
validated plan product. `MissionManager` and `ExecutionAdapter` MUST NOT
recalculate, loosen, or silently change these targets. The adapter may sample,
convert protocol/units, and apply a more conservative safety limit. A changed
physical situation requires revocation and replanning.

This rule supersedes statements in the earlier control-framework document
that assign `T_set` or `v_cable_ff` recomputation to `mission_manager`; the
architecture remains three-layered, but that responsibility is removed.

`AuthorizedExecutionBundle` is the only main-laying planner-to-executor
execution grant. It is restricted to the Main laying ExecutionAuthority on the
main NUC and cannot grant Scout motion. `ScoutAuthorizedExecutionBundle` is the
only Scout-motion grant and is restricted to the Scout motion
ExecutionAuthority on the Scout NUC. An independent `PlanningResult`,
`ScoutPlanningResult`, state, diagnostic, or revocation message has no grant
semantics. Consumers MUST NOT cache a plan and lease separately and assemble
authority locally.

There is exactly one software authority per physical execution domain, not one
global software authority for both robots. Neither authority may sign, renew,
revoke, acknowledge, or advance sequence watermarks for the other domain.
Each authority uses its own NUC local monotonic clock: the main NUC local
monotonic clock for main laying, and the Scout NUC local monotonic clock for
Scout motion. Any proposed authority crossing a device or domain boundary
must be re-issued by the target authority after local revalidation; the source
bundle itself is never forwarded as actuator authority.

## 2. Canonical artifacts

| Boundary | Authority | Location |
|---|---|---|
| NUC process and dual-robot messages | Protobuf v3 | `interfaces/proto/underwater/contracts/v1` |
| NUC–STM32F4 wire protocol | DBC plus fixed-protocol supplement | `interfaces/can` |
| Outcome/Fault/Diagnostic identities | Stable numeric registry | `interfaces/registry/codes-v1.json` |
| Scout legal state transitions | Stable executable registry | `interfaces/registry/scout-state-transitions-v1.json` |
| Non-production timing and resource limits | `integration/v1` | `interfaces/profiles/integration-v1.json` |

ROS 1, ROS 2, and planner C++ types are adapters/internal representations.
They MUST NOT add public semantics. A field that cannot be mapped without
loss is a contract change, not an adapter convenience.

Safety-critical optional fields MUST have explicit presence and validation.
A missing safety-critical field, an unspecified value, or an unknown
safety-critical enum MUST reject the whole message.

## 3. Trust boundary

v1 operates only inside a trusted, isolated control domain:

- main-NUC processes use OS identity, allowlists, and least privilege;
- the two NUCs use a physically isolated point-to-point network;
- SHA-256 and CRC provide identity/integrity or error detection, not source
  authentication;
- emergency-stop reset is local and authenticated by default.

Connecting shore networks, external switches, wireless links, or untrusted
processes invalidates this assumption. The system MUST NOT arm until a new
security review and authenticated mechanism are supplied. v1 does not define
PKI or key lifecycle.

## 4. Logical ownership and pub/sub

| Message/state | Canonical publisher | Subscribers | Meaning |
|---|---|---|---|
| `PlanningResult` | Laying Planner | Main laying ExecutionAuthority, MissionManager, recorder | Main-laying candidate only |
| `AuthorizedExecutionBundle` | Main laying ExecutionAuthority | Main ExecutionAdapter | Sole main-laying task execution grant |
| `BundleAck` | Main ExecutionAdapter | Main laying ExecutionAuthority | Main-domain install/reject acknowledgement; never renews |
| `ExecutionRevocation` | Main laying ExecutionAuthority | Main ExecutionAdapter, MissionManager | Main-domain canonical software revocation |
| `ExecutionFeedback` | Main ExecutionAdapter | Main laying ExecutionAuthority, Laying Planner | Main-domain tracking evidence |
| `ExecutionStatus` | Main ExecutionAdapter | Main laying ExecutionAuthority, MissionManager, SafetySupervisor | Main-domain execution fact |
| `ScoutPlanningResult` | ScoutMotionPlanner | Scout motion ExecutionAuthority, Scout mission authority, recorder | Scout candidate only |
| `ScoutAuthorizedExecutionBundle` | Scout motion ExecutionAuthority | Scout FCU adapter | Sole Scout-motion execution grant |
| `ScoutBundleAck` | Scout FCU adapter | Scout motion ExecutionAuthority | Exact Scout install/reject acknowledgement; never renews |
| `ScoutExecutionFeedback` | Scout FCU adapter | Scout motion ExecutionAuthority, ScoutMotionPlanner, SafetySupervisor | Exact profile/applied/measured Scout execution evidence; never renews by itself |
| `ScoutExecutionRevocation` | Scout motion ExecutionAuthority | Scout FCU adapter, Scout mission authority, SafetySupervisor | Exact Scout-domain software revocation; local stop does not wait for ACK |
| `ScoutExecutionRevocationAck` | Scout FCU adapter | Scout motion ExecutionAuthority | Exact revocation receipt/application observation; never restores authority |
| Main-laying authority state | Main laying ExecutionAuthority | Main ExecutionAdapter, MissionManager, recorder | Main-domain authorization fact and watermark only |
| Scout-motion authority state | Scout motion ExecutionAuthority | Scout FCU adapter, Scout mission authority, recorder | Scout-domain authorization fact and watermark only |
| `AuthorityEvent` | Fact-producing modules | ExecutionAuthority | Request/evidence only |
| `FaultReport` | Responsible detector | SafetySupervisor, ExecutionAuthority, MissionManager | Active/latched fault fact |
| `DiagnosticEvent` | Any module | Monitoring/recording | No control permission |
| `ScoutMission` / `ScoutMissionCancellation` | Laying-side mission authority | Scout mission authority | Versioned survey intent/cancellation; no execution grant |
| `ScoutMissionDecision` / `ScoutMissionCancellationAck` | Scout mission authority | Laying-side mission authority | Exact-identity admission/cancellation result |
| `SurveyCompletionEvidence` | Scout mission authority | Laying-side mission authority, recorder | Actual observations plus a newer immutable map; no laying-safety conclusion |
| `SurveyCompletionAck` | Laying-side mission authority | Scout mission authority | Exact-evidence receipt/validation result; no laying execution grant |
| Emergency trigger | Operator, SafetySupervisor, or STM32F4 local protection | Local safety path and state owner | Immediate hard action |
| Emergency-stop state | EmergencyStop domain owner | All safety consumers | Sole normalized E-stop state/watermark |
| State-domain snapshot | That domain's owner | Interested modules | State fact for one domain |

`ExecutionAdapter` is the only NUC producer of CAN control commands.
`MissionManager` owns task orchestration, not motion, payout, or tension
targets. Redundant implementations must select one owner internally; a
subscriber MUST NOT arbitrate competing canonical publishers.

Scout mission messages use the normative lifecycle, identity, local-admission,
retry, cancellation, and completion rules in
`interfaces/SCOUT_MISSION_LIFECYCLE.md`. The design term `SurveyRequest` is an
internal value-object view of `ScoutMission`, not a second public request or
identity. An accepted mission and a completion evidence message both remain
non-authorizing facts.

## 5. Bundle and lease lifecycle

The main-laying Bundle atomically binds:

- immutable plan and `ExecutionProfile`;
- plan, profile, bundle, and lease sequences;
- approved remaining arc-length interval;
- complete dependency versions;
- Timing, InterfaceLimits, Cable, and EmergencyStop policy references;
- `valid_not_before`, unique `execution_epoch`, and lease expiry;
- ContractManifest and canonical SHA-256 identities.

The Main laying ExecutionAuthority and ExecutionAdapter run on the main NUC and MUST share the
same safety Clock Domain. All profile sample times are relative to the fixed
`execution_epoch`. The adapter MUST NOT reset or shift that epoch. A wrong
Clock Domain, late arrival, missed start window, incomplete validation, or
inability to atomically install before the epoch rejects the Bundle.

A new Bundle atomically replaces the previous one. Same
`bundle_sequence + hash` is an idempotent duplicate; the same sequence with a
different hash is a fail-closed integrity fault. Revoked and expired lease
watermarks are persistent for the producer session. Delayed feedback cannot
revive them.

Cross-device authorization MUST be re-issued in the target device's local
Clock Domain. STM32F4 consumes sampled CAN targets and never interprets the
main NUC execution epoch. The Scout FCU adapter accepts only a live
`ScoutAuthorizedExecutionBundle` issued in its Scout NUC local monotonic clock;
it MUST reject `AuthorizedExecutionBundle`, `ImmutablePlan`, and all main-domain
lease or revocation watermarks.

The Scout Bundle atomically binds the exact immutable `ScoutPlan`, canonical
plan identity, complete dependencies, Scout lease, authorized trajectory-time
interval, fixed Scout execution epoch, TimingProfile, InterfaceLimits, and
SafetyGate configuration. Installation, ACK, duplicate/conflict handling,
retired sessions, and revoked/expired watermark rules are normative in
`interfaces/SCOUT_AUTHORIZATION_BUNDLE.md`.

## 6. Coordinates, units, and directions

- World/map frame: right-handed `mission_enu`, +X east, +Y north, +Z up.
- Robot frame: right-handed `base_link` (FLU), +X forward, +Y left, +Z up.
- Public angular value: ENU yaw about +Z, counter-clockwise positive, radians,
  normalized to `[-pi, pi)` when published.
- The ambiguous term/field `heading` MUST NOT appear in public v1 schemas.
- Longitudinal velocity is positive along `base_link +X`.
- Payout velocity is positive when cable leaves the cable store.
- All physical values use SI and unit-bearing field names.

NED/FRD conversion exists only in the MAVLink/ArduSub adapter. CAN direction,
integer scaling, and saturation exist only in the STM32F4 adapter. Each
conversion boundary requires single-axis and single-direction conformance
tests. Non-static transforms are versioned dependencies.

## 7. Time and freshness

Safety deadlines, watchdogs, leases, and ordering use local monotonic time
only. A `source_clock_domain_id` identifies the comparison domain. Restart or
Clock Domain change invalidates prior authority.

Synchronized UTC carries status and uncertainty and is used only for
cross-device observation alignment and audit. It MUST NOT drive local safety
deadlines. Receiver timeout is measured from the receiver's last valid
receive time using its own monotonic clock. STM32F4 uses local receive ticks,
session, sequence, and watchdog; it does not compare NUC absolute time.

All policy values come from a versioned `TimingProfile`. Production MUST NOT
use missing/default timing. The following invariant is mandatory per safety
stream:

```text
publish period < stale warning < software revocation
               < STM32 command watchdog <= hard safety timeout
```

`integration/v1` is a non-production bench baseline. Planner-internal defaults
such as 2 s feedback age or 5 s lease MUST NOT cross the Bundle boundary.

## 8. Sequences, versions, and content identity

Every public message carries producer, producer session, stream, and `uint64`
sequence. Sequence is strictly increasing only within the tuple
`(producer_id, producer_session_id, stream_id)`. Restart creates a new session;
old-session messages are permanently rejected.

Each ExecutionAuthority uniquely maintains its own domain's plan, profile,
lease, Bundle, and revocation business sequences. No sequence or watermark is
comparable across domains. A revocation names the exact same-domain Bundle,
plan, and lease. CAN's 8-bit `ProfileSequence` is a short pairing value under a
32-bit CAN session; it is never a plan version.

Versions express evolution order. Canonical SHA-256 expresses immutable
content identity. Neither substitutes for the other. Normative message and
artifact canonicalization is defined in `interfaces/HASHING.md`.

## 9. Delivery, ACK, retry, and ordering

Application semantics do not derive from ROS, TCP, UDP, or CAN:

- Bundle: at-least-once, explicit installed/rejected ACK, idempotent by
  sequence and hash.
- Revocation: high-priority at-least-once until ACK or lease expiry. Stop does
  not wait for ACK.
- Authority events, transition requests, and Profile transactions:
  at-least-once with stable request IDs and idempotent handling.
- Feedback and continuous status: latest-value; bounded reorder window; no
  backlog replay after recovery.
- Diagnostics: best-effort is allowed; latched Fault and state snapshots must
  be repeated or queryable.
- Emergency stop: immediate, latched, repeated; ACK observes but never gates
  the physical action. Reset is a separate authenticated idempotent request.

Retries MUST NOT duplicate physical actions or restore invalid authority.

## 10. Execution feedback

`ExecutionFeedback` separates:

1. `profile_target`: the approved profile sample;
2. `applied_target`: what the adapter actually handed to CAN after permitted
   conservative limiting;
3. `measured_state`: physical observations from STM32F4 and main sensors.

It also reports Bundle/plan/profile/lease identities, profile time/arc length,
CAN session and short sequence, `local_control_state`, and
`local_safety_override`.

Limiting may only narrow the approved envelope. It must be visible. If the
joint ground-motion/payout/tension relationship leaves the approved envelope,
or a permitted transient limit persists past TimingProfile policy, the
adapter submits a revocation event. Planner replanning uses measured state,
not applied target.

## 11. Orthogonal states and execution predicate

The system has independent Planning, MainLayingExecutionAuthority,
ScoutMotionExecutionAuthority, MainRobot, CableLaying, per-channel
Communication, EmergencyStop, and ScoutMission state domains. There is no
combined global enum. A transition changes only its own domain; cross-domain
effects require explicit events.

Main-laying `can_execute` is derived, never directly writable:

```text
authority is AUTHORIZED
and Bundle and lease are live
and main-robot mode permits motion
and cable state permits the requested operation
and every relevant channel LossPolicy permits the authorized prefix
and emergency-stop state is CLEARED
```

Communication recovery, planning success, condition disappearance, and fault
clear do not implicitly change other domains or restore authority. Normative
transitions are in `docs/state-machine-contract.md`; exact Scout transition
tuples and stable trigger identities are in
`interfaces/registry/scout-state-transitions-v1.json`.

Scout `can_execute` is a separate derivation over the exact live Scout Bundle
and lease, Scout motion authority state, Scout mission phase, relevant channel
LossPolicies, local hard-stop state, and Scout faults. Neither predicate can
grant, renew, or restore the other domain.

## 12. Communication fault domains

Each link owns `ChannelId + CommunicationState + TimingProfile + LossPolicy`.
There is no global control state called `COMM_DEGRADED`.

| Channel | Degraded/lost behavior |
|---|---|
| `PLANNER_EXECUTOR_IPC` | Stop accepting new Bundles; stale threshold revokes and requests coordinated stop; cached execution is bounded by lease |
| `NUC_STM32_CAN` | Stale, bus-off, or send failure revokes; STM32F4 watchdog performs local protective stop; reconnect creates a new session |
| `MAIN_SCOUT_COOP` | No new scout task or unevidenced map; main may use only a fresh, leased, policy-approved prefix; scout follows local LossPolicy |
| `SCOUT_NUC_FCU` | Scout local risk action and subsequent map invalidation; no cross-robot actuator takeover |

Recovery enters resynchronization. It requires post-boundary stream watermarks,
fresh state, compatible manifests/profiles, a new session where applicable,
revalidation, and a new same-domain Bundle.

## 13. Stop and recovery

All three stop levels terminate the current task Bundle in the affected
physical execution authority domain:

- `CONTROLLED_STOP`: motion after revocation is allowed only by a separate,
  certified stop authorization/profile and valid braking evidence.
- `PROTECTIVE_STOP`: local pre-certified fast coordinated stop; no planner or
  network dependency.
- `EMERGENCY_STOP`: independent hard, latched local action using the active
  CableProfile's certified `EmergencyStopPolicy`.

"Send zero" is not a stop definition. Braking payout during jam/overtension
may increase danger. Every armable CableProfile MUST bind a mechanically and
electrically certified EmergencyStopPolicy or arming is denied.

Recovery is strictly one-way:

```text
CLEARED_CONDITION fact
-> explicit Fault clear according to registry policy
-> authenticated local E-stop reset when applicable
-> SELF_CHECK
-> ContractManifest exchange
-> Profile prepare/activate
-> new sessions
-> STANDBY/READY
-> replanning
-> new AuthorizedExecutionBundle
```

No old session, Profile authorization, lease, or Bundle can revive.

## 14. Profiles and parameter constraints

Safety profiles are immutable ID/version/hash snapshots. Bundle references are
explicit dependencies. Validation includes presence, finite values, SI units,
single-field ranges, cross-field relations, operating domain, calibration
provenance, and schema/registry compatibility. Example relation:

```text
T_min < T_set <= T_warning < T_trip
```

Safety profiles change only in STANDBY or maintenance through idempotent
prepare/activate. Activation binds a new session and establishes configuration
consistency only; it never grants execution. A run-time safety parameter change
requires a new Profile, revocation, replanning, and a new Bundle.

Scout activation is atomic over its TimingProfile, InterfaceLimits, code and
transition registries, capability profile, energy model, sensor geometries,
planner/SafetyGate configurations, both channel-specific LossPolicies, and
risk-action rules. Missing production evidence or one incompatible nested
identity rejects and audits the whole profile. `integration/v1` is explicitly
non-production and cannot arm a production consumer.

## 15. Codes, faults, and diagnostics

`OutcomeCode` is an operation result. `FaultCode` is an active/latched fault.
`DiagnosticCode` is an observation or secondary rejection reason. Text is
human-readable only and MUST NOT drive control.

Local same-version registry data determines severity, safety effect, latch,
clear authority, and retryability. Unknown safety Fault fails closed. CAN
fault bits are restricted registry mappings, not a second fault system. A
cleared physical condition is not a cleared latched Fault.

## 16. Compatibility

`ContractManifest` binds Protobuf descriptor, code registry, DBC, TimingProfile,
InterfaceLimits, and supported features. It is checked before link use,
Profile activation, and arming.

The bootstrap Manifest is the sole exception to the normal header Manifest
reference: it carries its own content identity with `header.manifest` absent.
After acceptance, every other top-level safety message must reference the
accepted identity.

Minor changes may add only genuinely ignorable non-safety optional data.
Safety field presence/meaning/unit/range, safety enum, transition, authority,
revocation, timeout, and existing numeric-code changes require a major version.
Removed Protobuf field numbers and names are permanently reserved. DBC safety
protocol defaults to exact version matching.

The current artifacts are an unreleased, non-production v1 development
baseline. Before the first stable v1 freeze, a new gated Scout message family
replaces the complete descriptor/profile/Manifest set atomically while keeping
`schema 1.0`; no old/new pair is approved and exact identities plus every
required feature gate must match. After the stable v1 freeze, any safety enum,
revocation, timeout, field-presence, authority, or transition change increments
the schema major version. This development-baseline rule does not permit a
peer to infer compatibility from `schema_major/schema_minor` alone.

Mixed versions cannot execute unless an explicit compatibility matrix and
bidirectional conformance evidence approve the exact pair. Parseability is not
safety compatibility.

`independent_execution_authority_domains_v1` is an exact safety-semantic
feature gate. Existing v1 `AuthorizedExecutionBundle`, `ExecutionLease`,
`BundleAck`, `ExecutionRevocation`, `ExecutionFeedback`, `ExecutionStatus`, and
`STATE_DOMAIN_EXECUTION_AUTHORITY` retain their main-laying wire meaning;
`STATE_DOMAIN_EXECUTION_AUTHORITY` is a deprecated source alias for
`STATE_DOMAIN_MAIN_EXECUTION_AUTHORITY`. New Scout-domain messages and state
values are not ignorable extensions. A peer lacking the exact gate or exact
descriptor/profile identities MUST keep both domains unauthorized and MUST NOT
map, proxy, or infer one domain from the other. No mixed-version pair is
approved by this baseline.

`scout_authorization_bundle_v1` and
`scout_execution_feedback_revocation_v1` jointly gate the Scout execution
lifecycle. A peer lacking either feature cannot install Scout authority,
consume execution feedback as current evidence, or acknowledge a Scout
revocation. Feedback and revocation ACKs are non-authorizing; a persisted
revocation and local stop remain effective when an ACK is lost or delayed.

`scout_mission_lifecycle_v1` is a gated safety-semantic feature of the current
v1 development baseline. A peer lacking that feature or the exact descriptor,
code-registry, and integration-profile identities may observe no part of the
mission lifecycle as accepted and MUST NOT infer admission from parseability.

`scout_hybrid_map_snapshot_v1` is an independent exact feature gate. A Scout
planner lacking that feature or any exact descriptor/profile identity MUST NOT
install received map chunks. Reassembly publishes one immutable snapshot only
after full size, CRC, SHA-256, version, grid, layer, enum, dependency, and
resource-limit validation; partial layers never become planning facts.

## 17. Resource limits and priority

Every interface is bounded by versioned `InterfaceLimits`: Bundle bytes,
path/profile samples, repeated fields, strings, diagnostics, pending requests,
map chunks, and audit/telemetry queues. Check limits at the earliest safe point.
An over-limit object is rejected in full; interfaces MUST NOT truncate or
downsample safety content.

Safety traffic (Bundle, revocation, E-stop, feedback) has independent higher
priority than map and diagnostic traffic. Diagnostic floods and map backlogs
must not block safety channels.

## 18. Audit

Bundle/lease events, transitions, Profile transactions, Fault lifecycle,
stops/resets, manifests, sessions, and compatibility rejection produce
structured `SafetyAuditEvent` records. Event, correlation, and causal IDs must
allow deterministic reconstruction from input fact to transition, revocation,
and physical safety action.

Audit writes never block revocation or stopping. Drops are explicitly counted
and diagnosed. v1 defines record and causality semantics, not storage,
retention, shore upload, or analysis infrastructure.

## 19. Remaining production gates

The following are intentionally not frozen by this contract:

- production TimingProfile and InterfaceLimits values;
- track command allocation and controller gains;
- robot braking capability and safe stopping distance model;
- physical CableProfile limits and certified EmergencyStopPolicy actions;
- final MAVLink control mode and real hardware coordinate tests;
- long-duration, DAVE/Gazebo, water-tank, and sea-trial evidence.

They must be supplied by calibration and platform verification. The
`integration/v1` profile cannot be cited as production evidence.
