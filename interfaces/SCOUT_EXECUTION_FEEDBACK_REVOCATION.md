# Scout execution feedback and revocation contract

Status: v1 development baseline, non-production
Normative keywords: **MUST**, **MUST NOT**, **SHOULD**, and **MAY** are used in
their usual requirements sense.

## 1. Authority and stream ownership

`ScoutExecutionFeedback` is published only by `scout-fcu-adapter` on the Scout
NUC. `ScoutExecutionRevocation` is published only by
`scout-execution-authority` in the Scout motion authority domain, and
`ScoutExecutionRevocationAck` is published only by `scout-fcu-adapter`. These
three dedicated Scout streams and their watermarks are independent of the
main-laying `ExecutionFeedback` and `ExecutionRevocation` streams. Neither
domain may proxy, acknowledge, revoke, or advance a watermark for the other.

Feedback, revocation, and an ACK never grant or renew execution. Only an exact,
live `ScoutAuthorizedExecutionBundle` grants a bounded trajectory prefix.

## 2. Feedback identity and three execution views

Every feedback sample MUST bind the exact Bundle, plan, trajectory, and lease
sequences and canonical identities that are installed locally. It also binds
the FCU boot session, FCU command sequence, fixed-epoch trajectory offset,
control mode, the adapter producer session, and non-empty event/correlation
identities. All positions, velocities and yaw values use `mission_enu`; yaw is
normalized to `[-pi, pi)`.

The three views have distinct meanings:

- `profile_target` is the unmodified target sampled from the approved Scout
  trajectory at `profile_time_offset_ns`;
- `applied_target` is the target actually given to the FCU after an allowed,
  more conservative local safety limit; and
- `measured_state` is the observed physical state and observation tick.

The adapter MUST NOT silently change a profile target. Any difference between
profile and applied targets requires `limit_applied = true`, the registered
`SAFETY_LIMIT_APPLIED` diagnostic, and an explicit local-control view. A local
safety override additionally reports its stable fault, stop level, start tick,
and selected `ScoutRiskAction`. `SCOUT_RISK_ACTION_NONE` is explicit only when
no override is active.

## 3. Feedback admission and renewal boundary

The consumer rejects the complete feedback sample when its schema, exact
Manifest feature, canonical publisher, stream, Scout clock domain, installed
authorization identity, adapter session, FCU session, enum, presence, frame,
finite value, or resource bound is wrong. The generated tick equals the fixed
execution epoch plus the reported trajectory offset. The measured tick cannot
be later than the generated tick.

Delivery sequence and trajectory offset advance monotonically within their
respective sessions. The offset must remain inside the authorized interval and
the lease must still be live. Feedback older than the configured reject time,
from an old FCU session, after lease expiry, or with a trajectory-time rollback
cannot renew, extend, or restore authorization. Communication recovery requires
new sessions, fresh dependencies, complete revalidation, and a new Bundle.

Tracking error beyond the active SafetyGate thresholds, stale feedback, health
degradation, a relevant map change, communication loss, lease expiry, invalid
localization, insufficient energy, invalid sensing or coordination, plan
integrity failure, and emergency stop all trigger explicit revocation evidence.

## 4. Exact revocation and ACK behavior

`ScoutExecutionRevocation` names the exact installed Bundle, plan, trajectory,
and lease by both sequence and identity. It carries a non-unspecified
`ScoutRevocationReason`, stop level, selected risk action, effective Scout-local
tick, optional registered primary fault, ordered secondary diagnostics, strong
Scout-domain marker, and canonical revocation identity.

Revocation traffic is high-priority safety traffic and MUST NOT be blocked by
telemetry, diagnostics, map chunks, or audit backpressure. Before transmission,
the authority persists the revocation watermark and the local execution path
starts the requested stop or risk action. Stopping does not wait for an ACK.

Delivery is at least once. An idempotent retry reuses the exact header,
delivery sequence, revocation sequence, event identity, content identity, and
message bytes. The same sequence with different content is an integrity
conflict; a lower sequence is rejected. Retries continue at the configured
period until the exact ACK is accepted or local terminal handling supersedes
the network observation. ACK timeout does not postpone or reverse stopping.

The ACK binds the exact revocation identity and all authorization sequences,
plus the adapter and FCU sessions, application state, outcome, acknowledgement
tick, and diagnostics. A lost, delayed, conflicting, old-session, or rejected
ACK cannot renew a revoked lease. Revoked and expired watermarks survive normal
communication recovery for the session.

Each monitor fact that can trigger revocation MUST carry its own 16-byte event
identity and the 16-byte correlation identity of the execution audit chain. The
authority MUST copy those exact values into the revocation's
`caused_by_event_id` and `correlation_id`, and MUST reject a reason that does not
match the triggering fact. Its ACK MUST then use the revocation event as
`caused_by_event_id` and retain the same correlation identity, so the
fact-to-revocation-to-local-stop-to-physical-action chain can be reconstructed
deterministically. The local stop watermark records the accepted trigger event
and correlation identities before awaiting an ACK. Missing, malformed, or
mismatched event, correlation, causal identity, or trigger reason rejects the
complete message.

## 5. Timing, resources, and compatibility

The active `TimingProfile` enforces:

```text
Scout feedback publish period
  < Scout feedback stale warning
  < Scout feedback software revoke
  < Scout lease duration
Scout revocation retry period < Scout revocation ACK timeout
```

`InterfaceLimits` independently bounds the complete serialized feedback,
revocation, and ACK messages. Over-limit input is rejected as a whole; it is
never truncated. `integration/v1` is non-production and provides no FCU,
tracking, braking, communication, or vehicle-safety evidence.

`scout_execution_feedback_revocation_v1`,
`scout_authorization_bundle_v1`, and
`independent_execution_authority_domains_v1` are exact safety-semantic feature
gates. A peer missing any gate or exact descriptor/profile identity keeps the
Scout motion domain unauthorized. No mixed-version pair is approved.
