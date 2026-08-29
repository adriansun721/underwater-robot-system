# Orthogonal state-machine contract

This file defines legal v1 state transitions. Every transition carries the
previous/next state, trigger code, state version, event sequence, monotonic
entry time, and causal audit identity. Unknown or illegal transitions are
rejected and audited; they never coerce to a nearby state.

For Scout domains, the executable source of legal tuples and stable triggers
is `interfaces/registry/scout-state-transitions-v1.json`. A publisher must use
the exact registry `CodeRef trigger`; the deprecated scalar trigger field is
read only for v1 wire compatibility and cannot authorize a transition. Every
accepted tuple emits `AUDIT_STATE_TRANSITION`; every rejected tuple preserves
the previous state and emits `AUDIT_ILLEGAL_TRANSITION_REJECTED`.

## MainLayingExecutionAuthority

| From | To | Required trigger/evidence |
|---|---|---|
| `NO_AUTHORIZATION` | `AUTHORIZED` | Valid candidate, complete Bundle, compatible Manifest/Profiles, live lease |
| `AUTHORIZED` | `REVOKING` | Revocation event, expiry, dependency change, stop, E-stop, or safety fault |
| `REVOKING` | `REVOKED` | Revocation persisted and dispatched; stopping does not wait for ACK |
| `REVOKED` | `AUTHORIZED` | Complete revalidation and a strictly newer Bundle/lease |
| any | `NO_AUTHORIZATION` | New authority session before any grant |

There is no transition from communication recovery, Fault clear, or E-stop
reset directly to `AUTHORIZED`.

This state and all bundle, lease, revocation, and ACK watermarks are owned by
the Main laying ExecutionAuthority on the main NUC and apply only to tracks,
payout, and tension.

## ScoutMotionExecutionAuthority

| From | To | Required trigger/evidence |
|---|---|---|
| `NO_AUTHORIZATION` | `AUTHORIZED` | Exact safe Scout plan, complete Scout bundle, compatible Manifest/Profiles, live Scout lease |
| `AUTHORIZED` | `REVOKING` | Scout revocation event, expiry, dependency change, stop, E-stop, or safety fault |
| `REVOKING` | `REVOKED` | Scout revocation watermark persisted and dispatched; stopping does not wait for ACK |
| `REVOKED` | `AUTHORIZED` | Complete local revalidation and a strictly newer Scout bundle/lease |
| any | `NO_AUTHORIZATION` | New Scout authority session before any grant |

This domain is owned by the Scout motion ExecutionAuthority on the Scout NUC
and applies only to FCU motion. No transition in either domain changes the other domain,
shares a sequence watermark, or revives the other domain's authorization.
`ScoutExecutionRevocation` is the canonical software trigger at this boundary;
`ScoutExecutionRevocationAck` only observes receipt/application. A stale
`ScoutExecutionFeedback`, an old FCU session, a delayed ACK, or communication
recovery cannot transition `REVOKED` back to `AUTHORIZED`.

## MainRobot

```text
INIT -> SELF_CHECK -> STANDBY -> ARMED -> NAVIGATING
NAVIGATING -> HOLD -> STANDBY
any operational state -> FAULT
FAULT -> SELF_CHECK          (explicit clear only)
```

`ARMED` requires compatible Manifest, active Profiles, new sessions, cleared
E-stop, and no arm-blocking Fault. `NAVIGATING` additionally requires
`can_execute`.

## CableLaying

```text
PROFILE_LOAD -> THREADING_CHECK -> PRE_TENSION -> READY -> LAYING
LAYING <-> CORNER_SLOW
LAYING/CORNER_SLOW -> PAUSE_HOLD
PAUSE_HOLD -> READY           (fresh checks; still requires new Bundle)
PAUSE_HOLD -> RECOVERY -> READY/FAULT
LAYING/CORNER_SLOW -> FINISH
any operational state -> FAULT
FAULT -> PROFILE_LOAD         (explicit clear and complete recheck)
```

Profile activation does not transition `READY` to `LAYING`.

## Communication per ChannelId

```text
LINK_DOWN -> CONNECTING -> SYNCHRONIZING -> NORMAL
NORMAL -> DEGRADED -> LOST
DEGRADED/LOST -> RESYNCHRONIZING -> NORMAL
any -> LINK_DOWN
```

`RESYNCHRONIZING -> NORMAL` requires post-boundary watermarks, compatible
Manifest/Profile state, and link-specific LossPolicy checks. It does not
restore execution authority.

## EmergencyStop

```text
CLEARED -> TRIGGERED -> LATCHED -> AWAITING_LOCAL_RESET
AWAITING_LOCAL_RESET -> CLEARED
```

Any new physical assertion returns to `TRIGGERED`. The final transition
requires all physical sources released, an authenticated local reset, and the
registered clear conditions. It clears only the E-stop domain. SELF_CHECK,
sessions, replanning, and a new Bundle remain mandatory.

## ScoutMission

```text
IDLE -> PLANNING -> READY -> EXECUTING -> OBSERVING
OBSERVING -> WAITING_MAP -> REPLANNING -> EXECUTING
OBSERVING/WAITING_MAP -> COMPLETED
EXECUTING/OBSERVING -> DEGRADED -> RISK_ACTION
RISK_ACTION -> REPLANNING/FAILED
```

Every non-idle status binds the exact accepted `mission_id`,
`mission_version`, and `mission_content_identity`. Admission is represented by
`ScoutMissionDecision`, not by entering `PLANNING` alone. An applied
cancellation prevents new planning/authorization immediately and submits an
event to the Scout execution authority; an executing task reaches a terminal
or idle mission state only after the independently authorized safety action.
`SurveyCompletionEvidence` may trigger `COMPLETED` only when it binds actual
observations and a strictly newer immutable map as required by
`interfaces/SCOUT_MISSION_LIFECYCLE.md`.

Without new synchronized evidence, safety severity may stay equal or increase
only. A planner success alone cannot lower it.

## Derived can_execute

`SystemStateSnapshot.derived_can_execute` is the main-laying reported
derivation, not a
writable command. Every consumer recomputes and checks it from the same
versioned snapshot. A mismatch is a contract fault.

Execution requires all of:

- authority `AUTHORIZED` and exact live Bundle/lease;
- main robot `ARMED` or an explicitly executing mode;
- CableLaying state compatible with the requested profile segment;
- every relevant ChannelId allowed by its current LossPolicy;
- E-stop `CLEARED`;
- no registry Fault whose safety effect denies execution.

`SystemStateSnapshot.derived_scout_can_execute` is independently derived from
the Scout-motion authority state, exact live Scout bundle/lease, Scout mission
state, Scout link LossPolicies, local safety state, and Scout faults. Neither
derived predicate is an input to the other.
