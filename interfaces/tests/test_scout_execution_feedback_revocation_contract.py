"""Public-contract tests for Scout execution feedback and revocation."""

from __future__ import annotations

import importlib
import hashlib
import hmac
import json
import math
import pathlib
import subprocess
import sys
import tempfile
import unittest
from dataclasses import dataclass, field


SYSTEM_ROOT = pathlib.Path(__file__).resolve().parents[2]
INTERFACES = SYSTEM_ROOT / "interfaces"
PROTO_ROOT = INTERFACES / "proto"
PROTO_V1 = PROTO_ROOT / "underwater" / "contracts" / "v1"
PROFILE_PATH = INTERFACES / "profiles" / "integration-v1.json"
MANIFEST_PATH = INTERFACES / "compatibility" / "contract-manifest-v1.json"
CONTRACT_PATH = INTERFACES / "SCOUT_EXECUTION_FEEDBACK_REVOCATION.md"
SCOUT_FEEDBACK_STREAM_ID = 37
SCOUT_REVOCATION_STREAM_ID = 38
SCOUT_REVOCATION_ACK_STREAM_ID = 39


@dataclass(frozen=True)
class InstalledScoutAuthorization:
    bundle_sequence: int
    bundle_identity: bytes
    plan_sequence: int
    plan_identity: bytes
    trajectory_identity: bytes
    lease_sequence: int
    lease_identity: bytes
    execution_epoch_monotonic_ns: int
    authorized_start_time_offset_ns: int
    authorized_end_time_offset_ns: int
    expires_at_monotonic_ns: int


@dataclass
class ScoutFeedbackWatermarkState:
    adapter_session_id: bytes
    fcu_session_id: bytes
    delivery_sequence: int
    profile_time_offset_ns: int
    fcu_command_sequence: int = 0
    revoked_lease_sequences: set[int] = field(default_factory=set)


@dataclass
class ScoutRevocationWatermarkState:
    authority_session_id: bytes
    delivery_sequence: int
    revocation_sequence: int
    revocation_identity: bytes
    revoked_bundle_sequences: set[int] = field(default_factory=set)
    revoked_lease_sequences: set[int] = field(default_factory=set)
    local_stop_started: bool = False
    local_stop_trigger_event_id: bytes | None = None
    local_stop_correlation_id: bytes | None = None


@dataclass(frozen=True)
class ScoutMonitorFacts:
    trigger_event_id: bytes
    correlation_id: bytes
    position_tracking_error_m: float = 0.0
    maximum_position_tracking_error_m: float = 0.0
    thruster_health_changed: bool = False
    map_changed: bool = False
    communication_lost: bool = False
    now_monotonic_ns: int = 0
    lease_expires_at_monotonic_ns: int = 1


@dataclass
class ScoutRevocationRetryState:
    exact_revocation_bytes: bytes
    next_retry_at_monotonic_ns: int
    attempts: int = 0
    exact_ack_accepted: bool = False
    locally_superseded: bool = False


def _valid_identity(identity: object) -> bool:
    return len(identity.sha256) == 32


def canonical_revocation_identity(revocation: object) -> bytes:
    canonical = type(revocation)()
    canonical.CopyFrom(revocation)
    canonical.ClearField("revocation_content_identity")
    return hashlib.sha256(
        canonical.SerializeToString(deterministic=True)
    ).digest()


def _encode_varint(value: int) -> bytes:
    encoded = bytearray()
    while value >= 0x80:
        encoded.append((value & 0x7F) | 0x80)
        value >>= 7
    encoded.append(value)
    return bytes(encoded)


def _valid_code_ref(reference: object) -> bool:
    return (
        reference.numeric_code > 0
        and reference.registry_id == "underwater-system-codes"
        and reference.registry_version == 1
    )


def _codes_are_unique_ascending(references: object) -> bool:
    codes = [reference.numeric_code for reference in references]
    return codes == sorted(set(codes))


def _known_enum(message: object, field_name: str) -> bool:
    field = message.DESCRIPTOR.fields_by_name[field_name]
    return getattr(message, field_name) in field.enum_type.values_by_number


def _has_unknown_fields(message: object) -> bool:
    original = message.SerializeToString(deterministic=True)
    known_only = type(message)()
    known_only.CopyFrom(message)
    known_only.DiscardUnknownFields()
    return original != known_only.SerializeToString(deterministic=True)


def _validate_safety_header(
    header: object,
    *,
    producer_id: str,
    stream_id: int,
    producer_session_id: bytes,
    local_clock_domain_id: str,
    accepted_manifest_identity: bytes,
    now_monotonic_ns: int,
    require_cause: bool = False,
    expected_cause_event_id: bytes | None = None,
    expected_correlation_id: bytes | None = None,
) -> None:
    if (
        header.schema_major != 1
        or header.schema_minor != 0
        or header.producer_id != producer_id
        or header.stream_id != stream_id
        or bytes(header.producer_session_id) != producer_session_id
        or len(header.producer_session_id) != 16
        or header.sequence == 0
        or header.source_clock_domain_id != local_clock_domain_id
        or not header.HasField("generated_at_monotonic_ns")
        or header.generated_at_monotonic_ns > now_monotonic_ns
        or len(header.event_id) != 16
        or len(header.correlation_id) != 16
        or not header.HasField("manifest")
        or header.manifest.schema_major != 1
        or header.manifest.schema_minor != 0
        or not hmac.compare_digest(
            header.manifest.manifest_identity.sha256,
            accepted_manifest_identity,
        )
        or (require_cause and not header.HasField("caused_by_event_id"))
        or (
            header.HasField("caused_by_event_id")
            and len(header.caused_by_event_id) != 16
        )
        or (
            expected_cause_event_id is not None
            and not hmac.compare_digest(
                header.caused_by_event_id,
                expected_cause_event_id,
            )
        )
        or (
            expected_correlation_id is not None
            and not hmac.compare_digest(
                header.correlation_id,
                expected_correlation_id,
            )
        )
    ):
        raise ValueError("VERSION_INCOMPATIBLE: invalid safety message header")


def _valid_target(target: object) -> bool:
    names = (
        "x_m",
        "y_m",
        "z_m",
        "yaw_rad",
        "velocity_x_mps",
        "velocity_y_mps",
        "velocity_z_mps",
        "yaw_rate_radps",
    )
    return (
        target.frame_id == "mission_enu"
        and all(target.HasField(name) for name in names)
        and all(math.isfinite(getattr(target, name)) for name in names)
        and -math.pi <= target.yaw_rad < math.pi
    )


def _valid_measured_state(measured: object) -> bool:
    pose_names = ("x_m", "y_m", "z_m", "yaw_rad")
    velocity_names = ("x_mps", "y_mps", "z_mps", "yaw_rate_radps")
    return (
        measured.HasField("pose")
        and measured.HasField("velocity")
        and measured.HasField("observed_at_monotonic_ns")
        and measured.pose.frame_id == "mission_enu"
        and measured.velocity.frame_id == "mission_enu"
        and all(measured.pose.HasField(name) for name in pose_names)
        and all(measured.velocity.HasField(name) for name in velocity_names)
        and all(math.isfinite(getattr(measured.pose, name)) for name in pose_names)
        and all(
            math.isfinite(getattr(measured.velocity, name))
            for name in velocity_names
        )
        and -math.pi <= measured.pose.yaw_rad < math.pi
    )


def decide_scout_revocation_reason(facts: ScoutMonitorFacts) -> str | None:
    if (
        not math.isfinite(facts.position_tracking_error_m)
        or not math.isfinite(facts.maximum_position_tracking_error_m)
        or facts.maximum_position_tracking_error_m < 0.0
    ):
        return "SCOUT_REVOCATION_PLAN_INTEGRITY_MISMATCH"
    if facts.position_tracking_error_m > facts.maximum_position_tracking_error_m:
        return "SCOUT_REVOCATION_TRACKING_DEVIATION"
    if facts.thruster_health_changed:
        return "SCOUT_REVOCATION_THRUSTER_HEALTH_CHANGED"
    if facts.map_changed:
        return "SCOUT_REVOCATION_MAP_CHANGED"
    if facts.communication_lost:
        return "SCOUT_REVOCATION_COMMUNICATION_LOST"
    if facts.now_monotonic_ns >= facts.lease_expires_at_monotonic_ns:
        return "SCOUT_REVOCATION_LEASE_EXPIRED"
    return None


def attempt_scout_revocation_retry(
    state: ScoutRevocationRetryState,
    *,
    now_monotonic_ns: int,
    retry_period_ns: int,
    ack_timeout_ns: int,
    map_queue_depth: int,
    diagnostic_queue_depth: int,
) -> bytes | None:
    if (
        retry_period_ns <= 0
        or ack_timeout_ns <= retry_period_ns
        or map_queue_depth < 0
        or diagnostic_queue_depth < 0
    ):
        raise ValueError("INPUT_INVALID: invalid Scout revocation retry configuration")
    if state.exact_ack_accepted or state.locally_superseded:
        return None
    if now_monotonic_ns < state.next_retry_at_monotonic_ns:
        return None
    # Revocation uses its own safety lane; map and diagnostic queue depths are
    # validated inputs but intentionally never gate this send.
    state.attempts += 1
    state.next_retry_at_monotonic_ns = now_monotonic_ns + retry_period_ns
    return state.exact_revocation_bytes


def validate_scout_feedback(
    feedback: object,
    *,
    authorization: InstalledScoutAuthorization,
    state: ScoutFeedbackWatermarkState,
    now_monotonic_ns: int,
    local_clock_domain_id: str,
    accepted_manifest_identity: bytes,
    feedback_reject_ns: int,
    maximum_position_tracking_error_m: float,
    maximum_yaw_tracking_error_rad: float,
    maximum_feedback_bytes: int = 65_536,
) -> str:
    header = feedback.header
    if feedback.lease_sequence in state.revoked_lease_sequences:
        raise ValueError("REVOKED: delayed Scout feedback cannot revive a lease")
    if _has_unknown_fields(feedback):
        raise ValueError("INPUT_INVALID: unknown Scout feedback field")
    if len(feedback.SerializeToString(deterministic=True)) > maximum_feedback_bytes:
        raise ValueError("RESOURCE_LIMIT_EXCEEDED: Scout feedback is oversized")
    _validate_safety_header(
        header,
        producer_id="scout-fcu-adapter",
        stream_id=SCOUT_FEEDBACK_STREAM_ID,
        producer_session_id=state.adapter_session_id,
        local_clock_domain_id=local_clock_domain_id,
        accepted_manifest_identity=accepted_manifest_identity,
        now_monotonic_ns=now_monotonic_ns,
    )
    if header.sequence <= state.delivery_sequence:
        raise ValueError("SEQUENCE_REJECTED: Scout feedback delivery rollback")
    if (
        feedback.bundle_sequence != authorization.bundle_sequence
        or feedback.plan_sequence != authorization.plan_sequence
        or feedback.lease_sequence != authorization.lease_sequence
        or not hmac.compare_digest(
            feedback.bundle_content_identity.sha256,
            authorization.bundle_identity,
        )
        or not hmac.compare_digest(
            feedback.plan_content_identity.sha256,
            authorization.plan_identity,
        )
        or not hmac.compare_digest(
            feedback.trajectory_content_identity.sha256,
            authorization.trajectory_identity,
        )
        or not hmac.compare_digest(
            feedback.lease_content_identity.sha256,
            authorization.lease_identity,
        )
    ):
        raise ValueError("HASH_MISMATCH: Scout feedback authorization mismatch")
    if (
        bytes(feedback.fcu_session_id) != state.fcu_session_id
        or len(feedback.fcu_session_id) != 16
        or feedback.fcu_command_sequence <= state.fcu_command_sequence
        or not feedback.HasField("profile_time_offset_ns")
        or feedback.profile_time_offset_ns <= state.profile_time_offset_ns
        or feedback.profile_time_offset_ns
        != header.generated_at_monotonic_ns
        - authorization.execution_epoch_monotonic_ns
        or feedback.profile_time_offset_ns
        < authorization.authorized_start_time_offset_ns
        or feedback.profile_time_offset_ns
        > authorization.authorized_end_time_offset_ns
        or header.generated_at_monotonic_ns > now_monotonic_ns
        or now_monotonic_ns - header.generated_at_monotonic_ns > feedback_reject_ns
        or now_monotonic_ns >= authorization.expires_at_monotonic_ns
    ):
        raise ValueError("EXPIRED: Scout feedback is stale or outside authorization")
    if (
        not _valid_target(feedback.profile_target)
        or not _valid_target(feedback.applied_target)
        or not _valid_measured_state(feedback.measured_state)
        or feedback.measured_state.observed_at_monotonic_ns
        > header.generated_at_monotonic_ns
        or not feedback.HasField("limit_applied")
        or not _known_enum(feedback, "control_mode")
        or feedback.control_mode == 0
        or not feedback.safety_override.HasField("active")
        or not _known_enum(feedback.safety_override, "stop_level")
        or not _known_enum(feedback.safety_override, "risk_action")
        or feedback.safety_override.risk_action == 0
    ):
        raise ValueError("INPUT_INVALID: malformed Scout feedback")
    target_changed = (
        feedback.profile_target.SerializeToString(deterministic=True)
        != feedback.applied_target.SerializeToString(deterministic=True)
    )
    if target_changed and (
        not feedback.limit_applied
        or not feedback.HasField("limit_diagnostic")
        or not _valid_code_ref(feedback.limit_diagnostic)
        or feedback.limit_diagnostic.numeric_code != 131081
    ):
        raise ValueError("INPUT_INVALID: Scout target was changed silently")
    if not feedback.limit_applied and feedback.HasField("limit_diagnostic"):
        raise ValueError("INPUT_INVALID: limit diagnostic without an applied limit")
    if feedback.safety_override.active:
        if (
            not feedback.safety_override.HasField("reason_fault")
            or not _valid_code_ref(feedback.safety_override.reason_fault)
            or not feedback.safety_override.HasField("active_since_monotonic_ns")
            or feedback.safety_override.active_since_monotonic_ns
            > header.generated_at_monotonic_ns
            or feedback.safety_override.stop_level == 0
            or feedback.safety_override.risk_action in (0, 1)
        ):
            raise ValueError("INPUT_INVALID: incomplete Scout safety override")
    elif feedback.safety_override.risk_action != 1:
        raise ValueError("INPUT_INVALID: inactive override reports a risk action")

    dx = feedback.measured_state.pose.x_m - feedback.profile_target.x_m
    dy = feedback.measured_state.pose.y_m - feedback.profile_target.y_m
    dz = feedback.measured_state.pose.z_m - feedback.profile_target.z_m
    yaw_error = math.remainder(
        feedback.measured_state.pose.yaw_rad - feedback.profile_target.yaw_rad,
        2.0 * math.pi,
    )
    if (
        math.sqrt(dx * dx + dy * dy + dz * dz)
        > maximum_position_tracking_error_m
        or abs(yaw_error) > maximum_yaw_tracking_error_rad
    ):
        raise ValueError("REVOKED: Scout tracking deviation")

    state.delivery_sequence = header.sequence
    state.profile_time_offset_ns = feedback.profile_time_offset_ns
    state.fcu_command_sequence = feedback.fcu_command_sequence
    return "accepted"


def apply_scout_revocation(
    revocation: object,
    *,
    authorization: InstalledScoutAuthorization,
    state: ScoutRevocationWatermarkState,
    feedback_state: ScoutFeedbackWatermarkState,
    trigger_facts: ScoutMonitorFacts,
    now_monotonic_ns: int,
    local_clock_domain_id: str,
    accepted_manifest_identity: bytes,
    maximum_revocation_bytes: int = 32_768,
) -> str:
    header = revocation.header
    if _has_unknown_fields(revocation):
        raise ValueError("INPUT_INVALID: unknown Scout revocation field")
    if (
        len(revocation.SerializeToString(deterministic=True))
        > maximum_revocation_bytes
    ):
        raise ValueError("RESOURCE_LIMIT_EXCEEDED: Scout revocation is oversized")
    _validate_safety_header(
        header,
        producer_id="scout-execution-authority",
        stream_id=SCOUT_REVOCATION_STREAM_ID,
        producer_session_id=state.authority_session_id,
        local_clock_domain_id=local_clock_domain_id,
        accepted_manifest_identity=accepted_manifest_identity,
        now_monotonic_ns=now_monotonic_ns,
        require_cause=True,
        expected_cause_event_id=trigger_facts.trigger_event_id,
        expected_correlation_id=trigger_facts.correlation_id,
    )
    expected_reason_name = decide_scout_revocation_reason(trigger_facts)
    reason_value = (
        revocation.DESCRIPTOR.fields_by_name["reason"]
        .enum_type.values_by_number.get(revocation.reason)
    )
    if (
        expected_reason_name is None
        or reason_value is None
        or reason_value.name != expected_reason_name
    ):
        raise ValueError("INPUT_INVALID: revocation does not match trigger facts")
    if not revocation.HasField("domain"):
        raise ValueError("VERSION_INCOMPATIBLE: missing Scout authority domain")
    if (
        revocation.bundle_sequence != authorization.bundle_sequence
        or revocation.plan_sequence != authorization.plan_sequence
        or revocation.lease_sequence != authorization.lease_sequence
        or not hmac.compare_digest(
            revocation.bundle_content_identity.sha256,
            authorization.bundle_identity,
        )
        or not hmac.compare_digest(
            revocation.plan_content_identity.sha256,
            authorization.plan_identity,
        )
        or not hmac.compare_digest(
            revocation.trajectory_content_identity.sha256,
            authorization.trajectory_identity,
        )
        or not hmac.compare_digest(
            revocation.lease_content_identity.sha256,
            authorization.lease_identity,
        )
    ):
        raise ValueError("HASH_MISMATCH: Scout revocation authorization mismatch")
    if (
        revocation.revocation_sequence == 0
        or header.sequence == 0
        or not _known_enum(revocation, "requested_stop")
        or revocation.requested_stop == 0
        or not _known_enum(revocation, "requested_risk_action")
        or revocation.requested_risk_action in (0, 1)
        or not _known_enum(revocation, "reason")
        or revocation.reason == 0
        or not revocation.HasField("effective_at_monotonic_ns")
        or revocation.effective_at_monotonic_ns
        > header.generated_at_monotonic_ns
        or not _valid_identity(revocation.revocation_content_identity)
        or not hmac.compare_digest(
            revocation.revocation_content_identity.sha256,
            canonical_revocation_identity(revocation),
        )
        or (
            revocation.HasField("primary_fault")
            and not _valid_code_ref(revocation.primary_fault)
        )
        or any(
            not _valid_code_ref(code)
            for code in revocation.secondary_diagnostics
        )
        or not _codes_are_unique_ascending(revocation.secondary_diagnostics)
    ):
        raise ValueError("INPUT_INVALID: malformed Scout revocation")

    incoming_identity = bytes(revocation.revocation_content_identity.sha256)
    if (
        header.sequence == state.delivery_sequence
        and revocation.revocation_sequence == state.revocation_sequence
        and hmac.compare_digest(incoming_identity, state.revocation_identity)
    ):
        return "idempotent retry"
    if (
        header.sequence <= state.delivery_sequence
        or revocation.revocation_sequence <= state.revocation_sequence
    ):
        raise ValueError("SEQUENCE_REJECTED: Scout revocation watermark reused")

    state.delivery_sequence = header.sequence
    state.revocation_sequence = revocation.revocation_sequence
    state.revocation_identity = incoming_identity
    state.revoked_bundle_sequences.add(revocation.bundle_sequence)
    state.revoked_lease_sequences.add(revocation.lease_sequence)
    feedback_state.revoked_lease_sequences.add(revocation.lease_sequence)
    state.local_stop_started = True
    state.local_stop_trigger_event_id = bytes(trigger_facts.trigger_event_id)
    state.local_stop_correlation_id = bytes(trigger_facts.correlation_id)
    return "applied"


def validate_scout_revocation_ack(
    ack: object,
    *,
    revocation: object,
    adapter_session_id: bytes,
    fcu_session_id: bytes,
    now_monotonic_ns: int,
    local_clock_domain_id: str,
    accepted_manifest_identity: bytes,
    maximum_ack_bytes: int = 32_768,
) -> str:
    header = ack.header
    if _has_unknown_fields(ack):
        raise ValueError("INPUT_INVALID: unknown Scout revocation ACK field")
    if len(ack.SerializeToString(deterministic=True)) > maximum_ack_bytes:
        raise ValueError("RESOURCE_LIMIT_EXCEEDED: Scout revocation ACK is oversized")
    _validate_safety_header(
        header,
        producer_id="scout-fcu-adapter",
        stream_id=SCOUT_REVOCATION_ACK_STREAM_ID,
        producer_session_id=adapter_session_id,
        local_clock_domain_id=local_clock_domain_id,
        accepted_manifest_identity=accepted_manifest_identity,
        now_monotonic_ns=now_monotonic_ns,
        require_cause=True,
        expected_cause_event_id=bytes(revocation.header.event_id),
        expected_correlation_id=bytes(revocation.header.correlation_id),
    )
    if bytes(ack.fcu_session_id) != fcu_session_id or len(ack.fcu_session_id) != 16:
        raise ValueError("VERSION_INCOMPATIBLE: wrong Scout revocation ACK session")
    if (
        ack.revocation_sequence != revocation.revocation_sequence
        or ack.bundle_sequence != revocation.bundle_sequence
        or ack.plan_sequence != revocation.plan_sequence
        or ack.lease_sequence != revocation.lease_sequence
        or not _valid_identity(ack.observed_revocation_identity)
        or not hmac.compare_digest(
            ack.observed_revocation_identity.sha256,
            revocation.revocation_content_identity.sha256,
        )
        or not _known_enum(ack, "state")
        or ack.state == 0
        or not _known_enum(ack, "outcome")
        or ack.outcome == 0
        or not ack.HasField("acknowledged_at_monotonic_ns")
        or ack.acknowledged_at_monotonic_ns
        < revocation.effective_at_monotonic_ns
        or ack.acknowledged_at_monotonic_ns
        > header.generated_at_monotonic_ns
        or any(not _valid_code_ref(code) for code in ack.diagnostics)
        or not _codes_are_unique_ascending(ack.diagnostics)
    ):
        raise ValueError("INPUT_INVALID: malformed Scout revocation ACK")
    applied_state = ack.DESCRIPTOR.fields_by_name[
        "state"
    ].enum_type.values_by_name["SCOUT_REVOCATION_APPLIED"].number
    success_outcome = ack.DESCRIPTOR.fields_by_name[
        "outcome"
    ].enum_type.values_by_name["OUTCOME_SUCCESS"].number
    if ack.state == applied_state and ack.outcome != success_outcome:
        raise ValueError("INPUT_INVALID: applied Scout revocation ACK is not success")
    return "accepted"


class ScoutExecutionFeedbackRevocationContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.generated_directory = tempfile.TemporaryDirectory()
        subprocess.run(
            [
                "protoc",
                f"--proto_path={PROTO_ROOT}",
                f"--python_out={cls.generated_directory.name}",
                *sorted(str(path) for path in PROTO_V1.glob("*.proto")),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        sys.path.insert(0, cls.generated_directory.name)
        cls.common = importlib.import_module("underwater.contracts.v1.common_pb2")
        cls.execution = importlib.import_module(
            "underwater.contracts.v1.execution_pb2"
        )
        cls.profiles = importlib.import_module("underwater.contracts.v1.profiles_pb2")
        cls.accepted_manifest_identity = b"m" * 32

    @classmethod
    def tearDownClass(cls) -> None:
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def test_schema_has_dedicated_feedback_revocation_ack_and_streams(self) -> None:
        for message_name in (
            "ScoutExecutionFeedback",
            "ScoutExecutionRevocation",
            "ScoutExecutionRevocationAck",
        ):
            self.assertTrue(hasattr(self.execution, message_name), message_name)
        stream_names = (
            "STREAM_SCOUT_EXECUTION_FEEDBACK",
            "STREAM_SCOUT_EXECUTION_REVOCATION",
            "STREAM_SCOUT_EXECUTION_REVOCATION_ACK",
        )
        stream_values = []
        for stream_name in stream_names:
            self.assertTrue(hasattr(self.common, stream_name), stream_name)
            stream_values.append(getattr(self.common, stream_name))
        self.assertEqual(len(set(stream_values)), len(stream_values))
        self.assertNotIn(
            self.common.STREAM_EXECUTION_FEEDBACK,
            stream_values,
        )
        self.assertNotIn(
            self.common.STREAM_EXECUTION_REVOCATION,
            stream_values,
        )

    def test_feedback_round_trip_binds_exact_execution_and_exposes_all_views(self) -> None:
        identity = self.common.ContentIdentity(sha256=b"i" * 32)
        target = self.execution.ScoutMotionTarget3dEnu(
            x_m=1.0,
            y_m=2.0,
            z_m=-3.0,
            yaw_rad=0.25,
            velocity_x_mps=0.4,
            velocity_y_mps=0.1,
            velocity_z_mps=-0.2,
            yaw_rate_radps=0.05,
            frame_id="mission_enu",
        )
        measured = self.execution.ScoutMeasuredExecutionState(
            pose=self.execution.ScoutMeasuredPose3dEnu(
                x_m=1.1,
                y_m=2.1,
                z_m=-3.1,
                yaw_rad=0.3,
                frame_id="mission_enu",
            ),
            velocity=self.execution.ScoutMeasuredVelocity3dEnu(
                x_mps=0.39,
                y_mps=0.09,
                z_mps=-0.19,
                yaw_rate_radps=0.04,
                frame_id="mission_enu",
            ),
            observed_at_monotonic_ns=1_250,
        )
        feedback = self.execution.ScoutExecutionFeedback(
            header=self.common.MessageHeader(
                schema_major=1,
                schema_minor=0,
                producer_id="scout-fcu-adapter",
                producer_session_id=b"a" * 16,
                stream_id=self.common.STREAM_SCOUT_EXECUTION_FEEDBACK,
                sequence=11,
                source_clock_domain_id="scout-clock/boot-7",
                generated_at_monotonic_ns=1_300,
            ),
            bundle_sequence=7,
            bundle_content_identity=identity,
            plan_sequence=41,
            plan_content_identity=identity,
            trajectory_content_identity=identity,
            lease_sequence=9,
            lease_content_identity=identity,
            profile_time_offset_ns=200,
            profile_target=target,
            applied_target=target,
            measured_state=measured,
            limit_applied=True,
            limit_diagnostic=self.common.CodeRef(
                numeric_code=131081,
                registry_id="underwater-system-codes",
                registry_version=1,
            ),
            control_mode=self.execution.SCOUT_FCU_CONTROL_POSITION_YAW,
            safety_override=self.execution.ScoutLocalSafetyOverride(
                active=True,
                reason_fault=self.common.CodeRef(
                    numeric_code=66305,
                    registry_id="underwater-system-codes",
                    registry_version=1,
                ),
                stop_level=1,
                active_since_monotonic_ns=1_275,
                risk_action=self.execution.SCOUT_RISK_ACTION_BRAKE,
            ),
            fcu_session_id=b"f" * 16,
            fcu_command_sequence=23,
        )
        decoded = self.execution.ScoutExecutionFeedback.FromString(
            feedback.SerializeToString(deterministic=True)
        )
        self.assertEqual(decoded, feedback)
        self.assertNotEqual(decoded.profile_target.x_m, decoded.measured_state.pose.x_m)
        self.assertTrue(decoded.limit_applied)
        self.assertEqual(decoded.safety_override.risk_action, self.execution.SCOUT_RISK_ACTION_BRAKE)

    def test_revocation_and_ack_round_trip_bind_the_exact_authorization(self) -> None:
        identity = self.common.ContentIdentity(sha256=b"r" * 32)
        binding = {
            "bundle_sequence": 7,
            "plan_sequence": 41,
            "lease_sequence": 9,
        }
        revocation = self.execution.ScoutExecutionRevocation(
            header=self.common.MessageHeader(
                schema_major=1,
                schema_minor=0,
                producer_id="scout-execution-authority",
                producer_session_id=b"a" * 16,
                stream_id=self.common.STREAM_SCOUT_EXECUTION_REVOCATION,
                sequence=5,
                source_clock_domain_id="scout-clock/boot-7",
                generated_at_monotonic_ns=1_400,
            ),
            domain=self.execution.ScoutMotionExecutionAuthorityDomain(),
            revocation_sequence=3,
            bundle_content_identity=identity,
            plan_content_identity=identity,
            trajectory_content_identity=identity,
            lease_content_identity=identity,
            requested_stop=1,
            requested_risk_action=self.execution.SCOUT_RISK_ACTION_BRAKE,
            primary_fault=self.common.CodeRef(
                numeric_code=66305,
                registry_id="underwater-system-codes",
                registry_version=1,
            ),
            secondary_diagnostics=[
                self.common.CodeRef(
                    numeric_code=131075,
                    registry_id="underwater-system-codes",
                    registry_version=1,
                )
            ],
            effective_at_monotonic_ns=1_400,
            revocation_content_identity=identity,
            **binding,
        )
        decoded_revocation = self.execution.ScoutExecutionRevocation.FromString(
            revocation.SerializeToString(deterministic=True)
        )
        self.assertEqual(decoded_revocation, revocation)

        ack = self.execution.ScoutExecutionRevocationAck(
            header=self.common.MessageHeader(
                schema_major=1,
                schema_minor=0,
                producer_id="scout-fcu-adapter",
                producer_session_id=b"a" * 16,
                stream_id=self.common.STREAM_SCOUT_EXECUTION_REVOCATION_ACK,
                sequence=6,
                source_clock_domain_id="scout-clock/boot-7",
                generated_at_monotonic_ns=1_425,
            ),
            revocation_sequence=3,
            observed_revocation_identity=identity,
            state=self.execution.SCOUT_REVOCATION_APPLIED,
            acknowledged_at_monotonic_ns=1_420,
            outcome=1,
            fcu_session_id=b"f" * 16,
            **binding,
        )
        decoded_ack = self.execution.ScoutExecutionRevocationAck.FromString(
            ack.SerializeToString(deterministic=True)
        )
        self.assertEqual(decoded_ack, ack)
        self.assertEqual(decoded_ack.revocation_sequence, decoded_revocation.revocation_sequence)

    def _valid_feedback(self) -> object:
        identity = self.common.ContentIdentity(sha256=b"i" * 32)
        target = self.execution.ScoutMotionTarget3dEnu(
            x_m=1.0,
            y_m=2.0,
            z_m=-3.0,
            yaw_rad=0.25,
            velocity_x_mps=0.4,
            velocity_y_mps=0.1,
            velocity_z_mps=-0.2,
            yaw_rate_radps=0.05,
            frame_id="mission_enu",
        )
        return self.execution.ScoutExecutionFeedback(
            header=self.common.MessageHeader(
                schema_major=1,
                schema_minor=0,
                producer_id="scout-fcu-adapter",
                producer_session_id=b"a" * 16,
                stream_id=self.common.STREAM_SCOUT_EXECUTION_FEEDBACK,
                sequence=11,
                source_clock_domain_id="scout-clock/boot-7",
                generated_at_monotonic_ns=1_200,
                event_id=b"e" * 16,
                correlation_id=b"c" * 16,
                manifest=self.common.ContractManifestRef(
                    schema_major=1,
                    schema_minor=0,
                    manifest_identity=self.common.ContentIdentity(
                        sha256=self.accepted_manifest_identity
                    ),
                ),
            ),
            bundle_sequence=7,
            bundle_content_identity=identity,
            plan_sequence=41,
            plan_content_identity=identity,
            trajectory_content_identity=identity,
            lease_sequence=9,
            lease_content_identity=identity,
            profile_time_offset_ns=200,
            profile_target=target,
            applied_target=target,
            measured_state=self.execution.ScoutMeasuredExecutionState(
                pose=self.execution.ScoutMeasuredPose3dEnu(
                    x_m=1.0,
                    y_m=2.0,
                    z_m=-3.0,
                    yaw_rad=0.25,
                    frame_id="mission_enu",
                ),
                velocity=self.execution.ScoutMeasuredVelocity3dEnu(
                    x_mps=0.4,
                    y_mps=0.1,
                    z_mps=-0.2,
                    yaw_rate_radps=0.05,
                    frame_id="mission_enu",
                ),
                observed_at_monotonic_ns=1_195,
            ),
            limit_applied=False,
            control_mode=self.execution.SCOUT_FCU_CONTROL_POSITION_YAW,
            safety_override=self.execution.ScoutLocalSafetyOverride(
                active=False,
                stop_level=1,
                risk_action=self.execution.SCOUT_RISK_ACTION_NONE,
            ),
            fcu_session_id=b"f" * 16,
            fcu_command_sequence=23,
        )

    def test_feedback_consumer_fails_closed_for_stale_or_unsafe_evidence(self) -> None:
        authorization = InstalledScoutAuthorization(
            bundle_sequence=7,
            bundle_identity=b"i" * 32,
            plan_sequence=41,
            plan_identity=b"i" * 32,
            trajectory_identity=b"i" * 32,
            lease_sequence=9,
            lease_identity=b"i" * 32,
            execution_epoch_monotonic_ns=1_000,
            authorized_start_time_offset_ns=100,
            authorized_end_time_offset_ns=400,
            expires_at_monotonic_ns=1_500,
        )
        baseline = ScoutFeedbackWatermarkState(
            adapter_session_id=b"a" * 16,
            fcu_session_id=b"f" * 16,
            delivery_sequence=10,
            profile_time_offset_ns=100,
        )
        self.assertEqual(
            validate_scout_feedback(
                self._valid_feedback(),
                authorization=authorization,
                state=baseline,
                now_monotonic_ns=1_225,
                local_clock_domain_id="scout-clock/boot-7",
                accepted_manifest_identity=self.accepted_manifest_identity,
                feedback_reject_ns=250,
                maximum_position_tracking_error_m=0.05,
                maximum_yaw_tracking_error_rad=0.1,
            ),
            "accepted",
        )
        self.assertEqual((baseline.delivery_sequence, baseline.profile_time_offset_ns), (11, 200))

        cases = []
        old_adapter_session = self._valid_feedback()
        old_adapter_session.header.producer_session_id = b"o" * 16
        cases.append(("old adapter session", old_adapter_session, 1_225))
        old_fcu_session = self._valid_feedback()
        old_fcu_session.fcu_session_id = b"o" * 16
        cases.append(("old FCU session", old_fcu_session, 1_225))
        time_rollback = self._valid_feedback()
        time_rollback.profile_time_offset_ns = 100
        time_rollback.header.generated_at_monotonic_ns = 1_100
        time_rollback.measured_state.observed_at_monotonic_ns = 1_095
        cases.append(("trajectory time rollback", time_rollback, 1_125))
        delayed = self._valid_feedback()
        cases.append(("delayed feedback", delayed, 1_451))
        expired = self._valid_feedback()
        cases.append(("expired lease", expired, 1_500))
        silent_limit = self._valid_feedback()
        silent_limit.applied_target.x_m = 0.9
        cases.append(("silent applied-target change", silent_limit, 1_225))
        tracking_deviation = self._valid_feedback()
        tracking_deviation.measured_state.pose.x_m = 1.2
        cases.append(("tracking deviation", tracking_deviation, 1_225))

        for label, feedback, now in cases:
            with self.subTest(label=label):
                state = ScoutFeedbackWatermarkState(
                    adapter_session_id=b"a" * 16,
                    fcu_session_id=b"f" * 16,
                    delivery_sequence=10,
                    profile_time_offset_ns=100,
                )
                with self.assertRaises(ValueError):
                    validate_scout_feedback(
                        feedback,
                        authorization=authorization,
                        state=state,
                        now_monotonic_ns=now,
                        local_clock_domain_id="scout-clock/boot-7",
                        accepted_manifest_identity=self.accepted_manifest_identity,
                        feedback_reject_ns=250,
                        maximum_position_tracking_error_m=0.05,
                        maximum_yaw_tracking_error_rad=0.1,
                    )
                self.assertEqual((state.delivery_sequence, state.profile_time_offset_ns), (10, 100))

    def test_revocation_reasons_cover_required_monitor_triggers(self) -> None:
        for name in (
            "SCOUT_REVOCATION_TRACKING_DEVIATION",
            "SCOUT_REVOCATION_THRUSTER_HEALTH_CHANGED",
            "SCOUT_REVOCATION_MAP_CHANGED",
            "SCOUT_REVOCATION_COMMUNICATION_LOST",
            "SCOUT_REVOCATION_LEASE_EXPIRED",
            "SCOUT_REVOCATION_LOCALIZATION_INVALID",
            "SCOUT_REVOCATION_ENERGY_INSUFFICIENT",
            "SCOUT_REVOCATION_SENSOR_INVALID",
            "SCOUT_REVOCATION_COORDINATION_INVALID",
            "SCOUT_REVOCATION_PLAN_INTEGRITY_MISMATCH",
            "SCOUT_REVOCATION_EMERGENCY_STOP",
        ):
            self.assertTrue(hasattr(self.execution, name), name)
        self.assertIn(
            "reason",
            self.execution.ScoutExecutionRevocation.DESCRIPTOR.fields_by_name,
        )

    def test_health_map_communication_and_expiry_triggers_are_actionable(self) -> None:
        authorization = InstalledScoutAuthorization(
            bundle_sequence=7,
            bundle_identity=b"i" * 32,
            plan_sequence=41,
            plan_identity=b"i" * 32,
            trajectory_identity=b"i" * 32,
            lease_sequence=9,
            lease_identity=b"i" * 32,
            execution_epoch_monotonic_ns=1_000,
            authorized_start_time_offset_ns=100,
            authorized_end_time_offset_ns=400,
            expires_at_monotonic_ns=1_500,
        )
        cases = (
            (
                ScoutMonitorFacts(
                    trigger_event_id=b"t" * 16,
                    correlation_id=b"c" * 16,
                    position_tracking_error_m=0.2,
                    maximum_position_tracking_error_m=0.1,
                ),
                "SCOUT_REVOCATION_TRACKING_DEVIATION",
            ),
            (
                ScoutMonitorFacts(
                    trigger_event_id=b"t" * 16,
                    correlation_id=b"c" * 16,
                    thruster_health_changed=True,
                ),
                "SCOUT_REVOCATION_THRUSTER_HEALTH_CHANGED",
            ),
            (
                ScoutMonitorFacts(
                    trigger_event_id=b"t" * 16,
                    correlation_id=b"c" * 16,
                    map_changed=True,
                ),
                "SCOUT_REVOCATION_MAP_CHANGED",
            ),
            (
                ScoutMonitorFacts(
                    trigger_event_id=b"t" * 16,
                    correlation_id=b"c" * 16,
                    communication_lost=True,
                ),
                "SCOUT_REVOCATION_COMMUNICATION_LOST",
            ),
            (
                ScoutMonitorFacts(
                    trigger_event_id=b"t" * 16,
                    correlation_id=b"c" * 16,
                    now_monotonic_ns=1_500,
                    lease_expires_at_monotonic_ns=1_500,
                ),
                "SCOUT_REVOCATION_LEASE_EXPIRED",
            ),
        )
        for facts, expected_reason_name in cases:
            with self.subTest(reason=expected_reason_name):
                reason_name = decide_scout_revocation_reason(facts)
                self.assertEqual(reason_name, expected_reason_name)
                revocation = self._valid_revocation()
                revocation.reason = getattr(self.execution, reason_name)
                revocation.ClearField("primary_fault")
                revocation.revocation_content_identity.sha256 = (
                    canonical_revocation_identity(revocation)
                )
                feedback_state = ScoutFeedbackWatermarkState(
                    adapter_session_id=b"a" * 16,
                    fcu_session_id=b"f" * 16,
                    delivery_sequence=10,
                    profile_time_offset_ns=100,
                )
                state = ScoutRevocationWatermarkState(
                    authority_session_id=b"a" * 16,
                    delivery_sequence=4,
                    revocation_sequence=2,
                    revocation_identity=b"q" * 32,
                )
                wrong_cause = type(revocation)()
                wrong_cause.CopyFrom(revocation)
                wrong_cause.header.caused_by_event_id = b"w" * 16
                wrong_cause.revocation_content_identity.sha256 = (
                    canonical_revocation_identity(wrong_cause)
                )
                wrong_correlation = type(revocation)()
                wrong_correlation.CopyFrom(revocation)
                wrong_correlation.header.correlation_id = b"w" * 16
                wrong_correlation.revocation_content_identity.sha256 = (
                    canonical_revocation_identity(wrong_correlation)
                )
                for label, mismatched in (
                    ("cause", wrong_cause),
                    ("correlation", wrong_correlation),
                ):
                    with self.subTest(binding=label), self.assertRaises(ValueError):
                        apply_scout_revocation(
                            mismatched,
                            authorization=authorization,
                            state=state,
                            feedback_state=feedback_state,
                            trigger_facts=facts,
                            now_monotonic_ns=1_310,
                            local_clock_domain_id="scout-clock/boot-7",
                            accepted_manifest_identity=self.accepted_manifest_identity,
                        )
                self.assertFalse(state.local_stop_started)
                self.assertEqual(
                    apply_scout_revocation(
                        revocation,
                        authorization=authorization,
                        state=state,
                        feedback_state=feedback_state,
                        trigger_facts=facts,
                        now_monotonic_ns=1_310,
                        local_clock_domain_id="scout-clock/boot-7",
                        accepted_manifest_identity=self.accepted_manifest_identity,
                    ),
                    "applied",
                )
                self.assertTrue(state.local_stop_started)
                self.assertEqual(state.local_stop_trigger_event_id, b"t" * 16)
                self.assertEqual(state.local_stop_correlation_id, b"c" * 16)
                self.assertIn(9, feedback_state.revoked_lease_sequences)

    def _valid_revocation(self) -> object:
        revocation = self.execution.ScoutExecutionRevocation(
            header=self.common.MessageHeader(
                schema_major=1,
                schema_minor=0,
                producer_id="scout-execution-authority",
                producer_session_id=b"a" * 16,
                stream_id=self.common.STREAM_SCOUT_EXECUTION_REVOCATION,
                sequence=5,
                source_clock_domain_id="scout-clock/boot-7",
                generated_at_monotonic_ns=1_300,
                event_id=b"r" * 16,
                correlation_id=b"c" * 16,
                caused_by_event_id=b"t" * 16,
                manifest=self.common.ContractManifestRef(
                    schema_major=1,
                    schema_minor=0,
                    manifest_identity=self.common.ContentIdentity(
                        sha256=self.accepted_manifest_identity
                    ),
                ),
            ),
            domain=self.execution.ScoutMotionExecutionAuthorityDomain(),
            revocation_sequence=3,
            bundle_sequence=7,
            bundle_content_identity=self.common.ContentIdentity(sha256=b"i" * 32),
            plan_sequence=41,
            plan_content_identity=self.common.ContentIdentity(sha256=b"i" * 32),
            trajectory_content_identity=self.common.ContentIdentity(sha256=b"i" * 32),
            lease_sequence=9,
            lease_content_identity=self.common.ContentIdentity(sha256=b"i" * 32),
            requested_stop=1,
            requested_risk_action=self.execution.SCOUT_RISK_ACTION_BRAKE,
            reason=self.execution.SCOUT_REVOCATION_TRACKING_DEVIATION,
            primary_fault=self.common.CodeRef(
                numeric_code=66305,
                registry_id="underwater-system-codes",
                registry_version=1,
            ),
            effective_at_monotonic_ns=1_300,
        )
        revocation.revocation_content_identity.sha256 = canonical_revocation_identity(
            revocation
        )
        return revocation

    def _valid_revocation_ack(self, revocation: object) -> object:
        return self.execution.ScoutExecutionRevocationAck(
            header=self.common.MessageHeader(
                schema_major=1,
                schema_minor=0,
                producer_id="scout-fcu-adapter",
                producer_session_id=b"a" * 16,
                stream_id=self.common.STREAM_SCOUT_EXECUTION_REVOCATION_ACK,
                sequence=6,
                source_clock_domain_id="scout-clock/boot-7",
                generated_at_monotonic_ns=1_325,
                event_id=b"k" * 16,
                correlation_id=revocation.header.correlation_id,
                caused_by_event_id=revocation.header.event_id,
                manifest=self.common.ContractManifestRef(
                    schema_major=1,
                    schema_minor=0,
                    manifest_identity=self.common.ContentIdentity(
                        sha256=self.accepted_manifest_identity
                    ),
                ),
            ),
            revocation_sequence=revocation.revocation_sequence,
            bundle_sequence=revocation.bundle_sequence,
            plan_sequence=revocation.plan_sequence,
            lease_sequence=revocation.lease_sequence,
            observed_revocation_identity=revocation.revocation_content_identity,
            state=self.execution.SCOUT_REVOCATION_APPLIED,
            acknowledged_at_monotonic_ns=1_320,
            outcome=1,
            fcu_session_id=b"f" * 16,
        )

    def test_revocation_is_idempotent_and_stops_without_waiting_for_ack(self) -> None:
        authorization = InstalledScoutAuthorization(
            bundle_sequence=7,
            bundle_identity=b"i" * 32,
            plan_sequence=41,
            plan_identity=b"i" * 32,
            trajectory_identity=b"i" * 32,
            lease_sequence=9,
            lease_identity=b"i" * 32,
            execution_epoch_monotonic_ns=1_000,
            authorized_start_time_offset_ns=100,
            authorized_end_time_offset_ns=400,
            expires_at_monotonic_ns=1_500,
        )
        feedback_state = ScoutFeedbackWatermarkState(
            adapter_session_id=b"a" * 16,
            fcu_session_id=b"f" * 16,
            delivery_sequence=10,
            profile_time_offset_ns=100,
        )
        state = ScoutRevocationWatermarkState(
            authority_session_id=b"a" * 16,
            delivery_sequence=4,
            revocation_sequence=2,
            revocation_identity=b"q" * 32,
        )
        trigger_facts = ScoutMonitorFacts(
            trigger_event_id=b"t" * 16,
            correlation_id=b"c" * 16,
            position_tracking_error_m=0.2,
            maximum_position_tracking_error_m=0.1,
        )
        revocation = self._valid_revocation()
        self.assertEqual(
            apply_scout_revocation(
                revocation,
                authorization=authorization,
                state=state,
                feedback_state=feedback_state,
                trigger_facts=trigger_facts,
                now_monotonic_ns=1_310,
                local_clock_domain_id="scout-clock/boot-7",
                accepted_manifest_identity=self.accepted_manifest_identity,
            ),
            "applied",
        )
        self.assertTrue(state.local_stop_started)
        self.assertIn(7, state.revoked_bundle_sequences)
        self.assertIn(9, feedback_state.revoked_lease_sequences)

        # The ACK may be lost. Exact high-priority retries remain idempotent,
        # while the local stop and revoked watermark are already effective.
        self.assertEqual(
            apply_scout_revocation(
                revocation,
                authorization=authorization,
                state=state,
                feedback_state=feedback_state,
                trigger_facts=trigger_facts,
                now_monotonic_ns=1_335,
                local_clock_domain_id="scout-clock/boot-7",
                accepted_manifest_identity=self.accepted_manifest_identity,
            ),
            "idempotent retry",
        )
        delayed_feedback = self._valid_feedback()
        with self.assertRaises(ValueError):
            validate_scout_feedback(
                delayed_feedback,
                authorization=authorization,
                state=feedback_state,
                now_monotonic_ns=1_340,
                local_clock_domain_id="scout-clock/boot-7",
                accepted_manifest_identity=self.accepted_manifest_identity,
                feedback_reject_ns=250,
                maximum_position_tracking_error_m=0.05,
                maximum_yaw_tracking_error_rad=0.1,
            )

        conflict = self.execution.ScoutExecutionRevocation()
        conflict.CopyFrom(revocation)
        conflict.reason = self.execution.SCOUT_REVOCATION_MAP_CHANGED
        conflict.revocation_content_identity.sha256 = canonical_revocation_identity(conflict)
        with self.assertRaises(ValueError):
            apply_scout_revocation(
                conflict,
                authorization=authorization,
                state=state,
                feedback_state=feedback_state,
                trigger_facts=trigger_facts,
                now_monotonic_ns=1_340,
                local_clock_domain_id="scout-clock/boot-7",
                accepted_manifest_identity=self.accepted_manifest_identity,
            )

        ack = self._valid_revocation_ack(revocation)
        self.assertEqual(
            validate_scout_revocation_ack(
                ack,
                revocation=revocation,
                adapter_session_id=b"a" * 16,
                fcu_session_id=b"f" * 16,
                now_monotonic_ns=1_330,
                local_clock_domain_id="scout-clock/boot-7",
                accepted_manifest_identity=self.accepted_manifest_identity,
            ),
            "accepted",
        )
        self.assertTrue(state.local_stop_started)

    def test_revocation_retry_lane_ignores_map_and_diagnostic_backlogs(self) -> None:
        revocation = self._valid_revocation()
        exact_bytes = revocation.SerializeToString(deterministic=True)
        retry = ScoutRevocationRetryState(
            exact_revocation_bytes=exact_bytes,
            next_retry_at_monotonic_ns=1_300,
        )
        congested = {
            "map_queue_depth": 1_000_000,
            "diagnostic_queue_depth": 1_000_000,
        }
        self.assertEqual(
            attempt_scout_revocation_retry(
                retry,
                now_monotonic_ns=1_300,
                retry_period_ns=25,
                ack_timeout_ns=100,
                **congested,
            ),
            exact_bytes,
        )
        self.assertIsNone(
            attempt_scout_revocation_retry(
                retry,
                now_monotonic_ns=1_324,
                retry_period_ns=25,
                ack_timeout_ns=100,
                **congested,
            )
        )
        self.assertEqual(
            attempt_scout_revocation_retry(
                retry,
                now_monotonic_ns=1_325,
                retry_period_ns=25,
                ack_timeout_ns=100,
                **congested,
            ),
            exact_bytes,
        )
        # ACK timeout is an observation threshold, not permission to stop
        # retrying or to undo the already-started local stop.
        self.assertEqual(
            attempt_scout_revocation_retry(
                retry,
                now_monotonic_ns=1_400,
                retry_period_ns=25,
                ack_timeout_ns=100,
                **congested,
            ),
            exact_bytes,
        )
        retry.exact_ack_accepted = True
        self.assertIsNone(
            attempt_scout_revocation_retry(
                retry,
                now_monotonic_ns=1_425,
                retry_period_ns=25,
                ack_timeout_ns=100,
                **congested,
            )
        )
        self.assertEqual(retry.attempts, 3)

    def test_manifest_timing_resources_and_priority_rules_are_explicit(self) -> None:
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
        timing_fields = self.profiles.TimingProfile.DESCRIPTOR.fields_by_name
        limit_fields = self.profiles.InterfaceLimits.DESCRIPTOR.fields_by_name
        for name in (
            "scout_execution_feedback_publish_period_ns",
            "scout_execution_feedback_stale_warning_ns",
            "scout_execution_feedback_software_revoke_ns",
            "scout_revocation_retry_period_ns",
            "scout_revocation_ack_timeout_ns",
        ):
            self.assertIn(name, timing_fields)
            self.assertGreater(profile["timing"][name], 0)
        for name in (
            "maximum_scout_execution_feedback_bytes",
            "maximum_scout_execution_revocation_bytes",
            "maximum_scout_execution_revocation_ack_bytes",
        ):
            self.assertIn(name, limit_fields)
            self.assertGreater(profile["interface_limits"][name], 0)
        timing = profile["timing"]
        self.assertLess(
            timing["scout_execution_feedback_publish_period_ns"],
            timing["scout_execution_feedback_stale_warning_ns"],
        )
        self.assertLess(
            timing["scout_execution_feedback_stale_warning_ns"],
            timing["scout_execution_feedback_software_revoke_ns"],
        )
        self.assertLess(
            timing["scout_execution_feedback_software_revoke_ns"],
            timing["scout_lease_duration_ns"],
        )
        self.assertLess(
            timing["scout_revocation_retry_period_ns"],
            timing["scout_revocation_ack_timeout_ns"],
        )
        self.assertIn(
            "scout_execution_feedback_revocation_v1",
            manifest["supported_features"],
        )
        contract = CONTRACT_PATH.read_text(encoding="utf-8").lower()
        for phrase in (
            "must not silently",
            "does not wait for an ack",
            "high-priority",
            "idempotent retry",
            "cannot renew",
            "old fcu session",
        ):
            self.assertIn(phrase, contract)

    def test_feedback_rejects_command_rollback_unknown_enum_wire_and_oversize(self) -> None:
        authorization = InstalledScoutAuthorization(
            bundle_sequence=7,
            bundle_identity=b"i" * 32,
            plan_sequence=41,
            plan_identity=b"i" * 32,
            trajectory_identity=b"i" * 32,
            lease_sequence=9,
            lease_identity=b"i" * 32,
            execution_epoch_monotonic_ns=1_000,
            authorized_start_time_offset_ns=100,
            authorized_end_time_offset_ns=400,
            expires_at_monotonic_ns=1_500,
        )

        command_rollback = self._valid_feedback()
        unknown_control_mode = self._valid_feedback()
        unknown_control_mode.control_mode = 127
        missing_event_identity = self._valid_feedback()
        missing_event_identity.header.ClearField("event_id")
        wire = self._valid_feedback().SerializeToString(deterministic=True)
        unknown_wire = type(self._valid_feedback()).FromString(
            wire + _encode_varint((127 << 3) | 0) + _encode_varint(1)
        )
        cases = (
            ("FCU command rollback", command_rollback, 65_536),
            ("unknown control mode", unknown_control_mode, 65_536),
            ("missing event identity", missing_event_identity, 65_536),
            ("unknown wire field", unknown_wire, 65_536),
            ("oversize", self._valid_feedback(), 1),
        )
        for label, feedback, maximum_bytes in cases:
            with self.subTest(label=label):
                state = ScoutFeedbackWatermarkState(
                    adapter_session_id=b"a" * 16,
                    fcu_session_id=b"f" * 16,
                    delivery_sequence=10,
                    profile_time_offset_ns=100,
                    fcu_command_sequence=23,
                )
                with self.assertRaises(ValueError):
                    validate_scout_feedback(
                        feedback,
                        authorization=authorization,
                        state=state,
                        now_monotonic_ns=1_225,
                        local_clock_domain_id="scout-clock/boot-7",
                        accepted_manifest_identity=self.accepted_manifest_identity,
                        feedback_reject_ns=250,
                        maximum_position_tracking_error_m=0.05,
                        maximum_yaw_tracking_error_rad=0.1,
                        maximum_feedback_bytes=maximum_bytes,
                    )

    def test_revocation_and_ack_reject_unknown_wire_old_sessions_and_oversize(self) -> None:
        authorization = InstalledScoutAuthorization(
            bundle_sequence=7,
            bundle_identity=b"i" * 32,
            plan_sequence=41,
            plan_identity=b"i" * 32,
            trajectory_identity=b"i" * 32,
            lease_sequence=9,
            lease_identity=b"i" * 32,
            execution_epoch_monotonic_ns=1_000,
            authorized_start_time_offset_ns=100,
            authorized_end_time_offset_ns=400,
            expires_at_monotonic_ns=1_500,
        )
        valid = self._valid_revocation()
        wire = valid.SerializeToString(deterministic=True)
        unknown = type(valid).FromString(
            wire + _encode_varint((127 << 3) | 0) + _encode_varint(1)
        )
        old_session = self._valid_revocation()
        old_session.header.producer_session_id = b"o" * 16
        missing_cause = self._valid_revocation()
        missing_cause.header.ClearField("caused_by_event_id")
        missing_cause.revocation_content_identity.sha256 = canonical_revocation_identity(
            missing_cause
        )
        revocation_cases = (
            ("unknown wire", unknown, 32_768),
            ("old authority session", old_session, 32_768),
            ("missing causal identity", missing_cause, 32_768),
            ("oversize", self._valid_revocation(), 1),
        )
        for label, revocation, maximum_bytes in revocation_cases:
            with self.subTest(label=label):
                state = ScoutRevocationWatermarkState(
                    authority_session_id=b"a" * 16,
                    delivery_sequence=4,
                    revocation_sequence=2,
                    revocation_identity=b"q" * 32,
                )
                feedback_state = ScoutFeedbackWatermarkState(
                    adapter_session_id=b"a" * 16,
                    fcu_session_id=b"f" * 16,
                    delivery_sequence=10,
                    profile_time_offset_ns=100,
                )
                with self.assertRaises(ValueError):
                    apply_scout_revocation(
                        revocation,
                        authorization=authorization,
                        state=state,
                        feedback_state=feedback_state,
                        trigger_facts=ScoutMonitorFacts(
                            trigger_event_id=b"t" * 16,
                            correlation_id=b"c" * 16,
                            position_tracking_error_m=0.2,
                            maximum_position_tracking_error_m=0.1,
                        ),
                        now_monotonic_ns=1_310,
                        local_clock_domain_id="scout-clock/boot-7",
                        accepted_manifest_identity=self.accepted_manifest_identity,
                        maximum_revocation_bytes=maximum_bytes,
                    )
                self.assertFalse(state.local_stop_started)

        revocation = self._valid_revocation()
        ack = self._valid_revocation_ack(revocation)
        ack_wire = ack.SerializeToString(deterministic=True)
        unknown_ack = type(ack).FromString(
            ack_wire + _encode_varint((127 << 3) | 0) + _encode_varint(1)
        )
        old_fcu_ack = self._valid_revocation_ack(revocation)
        old_fcu_ack.fcu_session_id = b"o" * 16
        wrong_cause_ack = self._valid_revocation_ack(revocation)
        wrong_cause_ack.header.caused_by_event_id = b"w" * 16
        ack_cases = (
            ("unknown ACK wire", unknown_ack, 32_768),
            ("old FCU session", old_fcu_ack, 32_768),
            ("wrong causal identity", wrong_cause_ack, 32_768),
            ("oversize ACK", self._valid_revocation_ack(revocation), 1),
        )
        for label, candidate, maximum_bytes in ack_cases:
            with self.subTest(label=label):
                with self.assertRaises(ValueError):
                    validate_scout_revocation_ack(
                        candidate,
                        revocation=revocation,
                        adapter_session_id=b"a" * 16,
                        fcu_session_id=b"f" * 16,
                        now_monotonic_ns=1_330,
                        local_clock_domain_id="scout-clock/boot-7",
                        accepted_manifest_identity=self.accepted_manifest_identity,
                        maximum_ack_bytes=maximum_bytes,
                    )


if __name__ == "__main__":
    unittest.main()
