# Scout motion authorization bundle contract

Status: v1 development baseline, non-production
Normative keywords: **MUST**, **MUST NOT**, **SHOULD**, and **MAY** are used in
their usual requirements sense.

## 1. Authority boundary

`ScoutAuthorizedExecutionBundle` is the only execution grant for Scout FCU
motion. `ScoutPlanningResult`, `ScoutPlan`, mission admission, state,
diagnostics, a standalone lease, or any main-laying message has no Scout grant
semantics. The canonical publisher is the Scout motion ExecutionAuthority on
the Scout NUC, with exact `MessageHeader.producer_id`
`scout-execution-authority`; the canonical consumer is the Scout FCU adapter
on that NUC. Its ACK publisher identity is exactly `scout-fcu-adapter`.

The main-laying `AuthorizedExecutionBundle`, `ExecutionLease`, `BundleAck`,
`ImmutablePlan`, and their watermarks MUST NOT be mapped into these Scout
types. Neither domain may sign, renew, revoke, acknowledge, or advance a
watermark for the other.

## 2. Atomic content

A Scout Bundle atomically binds:

- the strong `ScoutMotionExecutionAuthorityDomain` marker;
- one exact immutable `ScoutPlan` and its canonical plan identity;
- one `ScoutExecutionLease` with strictly positive plan, lease, and Bundle
  sequences;
- the exact plan dependencies and their canonical identity;
- a non-empty authorized trajectory-time interval wholly inside the validated
  trajectory;
- a fixed `execution_epoch_monotonic_ns`, an installation window, validation
  time, and lease expiry in the Scout NUC local clock domain;
- exact TimingProfile, InterfaceLimits, and SafetyGate configuration
  references; and
- the canonical Bundle identity and exact accepted ContractManifest identity.

The consumer MUST atomically validate and install the whole Bundle. It MUST NOT
assemble authority from separately cached plan, lease, profile, state, or ACK.
The execution epoch MUST NOT be shifted to receipt, installation, or first FCU
command time. Authorized offsets are evaluated only from the fixed epoch.

## 3. Admission and installation

Before installation the consumer MUST reject the whole Bundle when any of the
following is true:

- the domain marker, canonical publisher, Scout stream, schema, exact Manifest
  feature, or Scout NUC clock domain is wrong;
- the plan, lease, dependencies, validation report, trajectory, profile, or
  Bundle identity is missing or fails canonical recomputation;
- the validation status is not `SCOUT_PLAN_VALIDATION_SAFE`, or the report does
  not bind the exact dependency, trajectory, and survey-evidence identities;
- plan, lease, dependency, TimingProfile, InterfaceLimits, or SafetyGate
  configuration references disagree, or any of those three profile references
  differs from the exact locally accepted configuration;
- a timestamp is absent, non-local, impossible to order, or the installation
  attempt is outside `[valid_not_before, execution_epoch)`;
- the authorized interval is empty, exceeds the validated trajectory, or its
  epoch-relative end exceeds lease expiry;
- a business or delivery sequence is zero, reordered, reused with conflicting
  content, retired, revoked, or expired;
- the message contains an unknown field, unknown safety enum, non-finite value,
  non-NFC string, incomplete identity, or exceeds InterfaceLimits.

These checks fail closed with stable Outcome/Fault/Diagnostic identities. A
wrong clock domain or incompatible Manifest produces `VERSION_INCOMPATIBLE`;
canonical content mismatch produces `HASH_MISMATCH`; stale dependencies or
profiles produce `DEPENDENCY_STALE`; ordering and window failures produce
`SEQUENCE_REJECTED` or `EXPIRED` as applicable.

## 4. ACK, delivery, and watermarks

Bundle delivery is at least once. `ScoutBundleAck` binds the exact Bundle,
plan, lease, observed Bundle identity, install time, state, outcome, and
diagnostics. The ACK consumer MUST validate its canonical publisher, Scout ACK
stream, schema, Manifest, local clock/session, all three sequences, observed
Bundle identity, timestamp, state, and known outcome before accepting it. An
ACK observes installation or rejection and never grants, renews, or extends
authority.

`MessageHeader.sequence` is the delivery watermark and `bundle_sequence` is
the business watermark. Both MUST be positive, but they are orthogonal and
MUST NOT be required to have the same numeric value.

Within one producer session, the same bundle_sequence, lease sequence, and
Bundle identity is an idempotent duplicate. The same bundle_sequence with a
different identity is an integrity conflict. Lower or reused delivery,
Bundle, plan, or lease sequences fail closed. Revoked or expired lease
watermarks are persistent for the producer session and cannot be revived by a
retry, delayed ACK, delayed status, or communication recovery.

A producer restart creates a new authority session and local clock domain.
The consumer retires the old session and accepts the new one only across an
explicit resynchronization boundary with an exact compatible Manifest and
fresh dependencies. Old-session traffic remains permanently rejected.

## 5. Timing and resources

`TimingProfile` supplies Scout-specific Bundle ACK timeout, lease duration,
renewal margin, and installation-window values. The invariant is:

```text
Scout Bundle ACK timeout < Scout lease duration
Scout lease renewal margin < Scout lease duration
Scout authorization start window < Scout lease duration
```

Consumers MUST enforce the referenced values, not merely bind their profile
identity: `expires_at - validated_at` MUST NOT exceed the Scout lease duration,
`execution_epoch - valid_not_before` MUST NOT exceed the Scout authorization
start window, and an installed ACK generated more than the Scout Bundle ACK
timeout after installation is rejected as late observation.

`InterfaceLimits.maximum_scout_authorized_bundle_bytes` bounds the complete
serialized Bundle. Over-limit input is rejected; it is never truncated or
downsampled. `integration/v1` remains non-production and cannot establish a
vehicle-safe lease duration or start window.

## 6. Compatibility and adapters

`independent_execution_authority_domains_v1` and
`scout_authorization_bundle_v1` are both exact safety-semantic feature gates.
A peer lacking either gate or any exact descriptor/profile identity keeps the
Scout motion domain unauthorized. No mixed-version pair is approved.

Protobuf-to-C++ and Protobuf-to-ROS adapters MUST map every field
bidirectionally without inventing defaults. The FCU/MAVLink adapter consumes
only an already installed live Scout Bundle and converts coordinates and
commands at that boundary; it cannot change validation, interval, epoch,
lease, profile, or watermark semantics.

Execution-time observation and termination follow
`SCOUT_EXECUTION_FEEDBACK_REVOCATION.md`. Feedback and revocation ACKs remain
non-authorizing evidence; a persisted exact revocation terminates the Bundle
without waiting for network acknowledgement.
