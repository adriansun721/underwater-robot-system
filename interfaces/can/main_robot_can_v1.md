# Main robot CAN protocol v1

This document and `main_robot_can_v1.dbc` together are the authoritative
NUC-to-STM32F4 fixed protocol. Protobuf memory layouts must never be copied to
CAN.

## Link and encoding

- CAN 2.0A, 11-bit identifiers, 1 Mbit/s, 8-byte data frames.
- Multi-byte values are little-endian; signed values use two's complement.
- DBC physical units are normative. Public units are SI.
- The NUC-side `ExecutionAdapter` is the only producer of control frames.
- Periodic control queues retain only the newest value. Recovery never
  replays queued commands.

## Protected-frame CRC

Frames carrying heartbeat, emergency, session, commands, or profile
transactions reserve bytes 6-7 for CRC-16/CCITT-FALSE:

- polynomial `0x1021`
- initial value `0xFFFF`
- no reflection
- xor-out `0x0000`
- input bytes: CAN ID encoded as little-endian `uint16`, followed by payload
  bytes 0-5
- output stored little-endian in payload bytes 6-7

A bad CRC rejects the entire frame and does not refresh any watchdog.

## Session and sequence rules

`SYS_SESSION` establishes a non-zero 32-bit `SessionId`. A restart, contract
change, Profile activation, bus-off recovery, or Clock Domain change requires
a new value. Old-session commands are rejected.

The 8-bit `ProfileSequence` only pairs `CMD_TRACK_CTRL` and
`CMD_CABLE_CTRL` over a short interval. It is not a plan or profile version.
Both commands must have the same value before STM32F4 updates the joint
target. A mismatch holds the last paired target only within the active
watchdog window and requests retransmission; watchdog expiry performs a local
protective stop.

## Profile prepare/activate

1. `CFG_PROFILE_PREPARE` starts an idempotent transaction.
2. Eight `CFG_PROFILE_HASH_CHUNK` frames transfer the canonical 32-byte
   SHA-256 in chunk-index order. Duplicate identical chunks are accepted;
   conflicting duplicates reject the transaction.
3. STM32F4 validates the locally provisioned profile and reports `PREPARED`.
4. `CFG_PROFILE_ACTIVATE` binds the profile to `NewSessionId`.
5. STM32F4 reports `ACTIVE`; NUC then establishes `SYS_SESSION`.

Activation establishes configuration consistency only. Motion remains
forbidden until the NUC ExecutionAdapter has a live
`AuthorizedExecutionBundle`.

## Fault mapping

`TrackFaultBits` and `CableFaultBits` are restricted encodings of the shared
FaultCode registry. `registry/codes-v1.json` is authoritative for each bit.
Unknown set bits are treated as unknown safety-critical faults and fail
closed. A bit clearing is only `CLEARED_CONDITION`; registry latching and
clear-authority rules still apply.

## Watchdog and stopping

The active TimingProfile supplies heartbeat and command periods and the local
watchdog. `integration/v1` uses a 500 ms CAN command watchdog. Timeout,
bus-off, invalid session, or a persistent unpaired target invokes the
pre-certified local `PROTECTIVE_STOP` behavior and invalidates the task
authorization. Emergency inputs invoke the active CableProfile's certified
`EmergencyStopPolicy` independently of CAN acknowledgement.

