"""Public-contract tests for Scout-motion atomic authorization bundles."""

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
import unicodedata
from dataclasses import dataclass, field, replace

from google.protobuf.descriptor import FieldDescriptor


SYSTEM_ROOT = pathlib.Path(__file__).resolve().parents[2]
INTERFACES = SYSTEM_ROOT / "interfaces"
PROTO_ROOT = INTERFACES / "proto"
PROTO_V1 = PROTO_ROOT / "underwater" / "contracts" / "v1"
MANIFEST_PATH = INTERFACES / "compatibility" / "contract-manifest-v1.json"
PROFILE_PATH = INTERFACES / "profiles" / "integration-v1.json"
CONTRACT_PATH = INTERFACES / "SCOUT_AUTHORIZATION_BUNDLE.md"
HASHING_PATH = INTERFACES / "HASHING.md"
SCOUT_AUTHORITY_PRODUCER_ID = "scout-execution-authority"
SCOUT_ACK_PRODUCER_ID = "scout-fcu-adapter"
ACCEPTED_SCOUT_PROFILE = ("scout/profile/v1", 1, b"p" * 32)
INTEGRATION_TIMING = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))["timing"]


@dataclass
class ScoutBundleWatermarkState:
    producer_session_id: bytes
    delivery_sequence: int
    bundle_sequence: int
    plan_sequence: int
    lease_sequence: int
    bundle_identity: bytes
    retired_sessions: set[bytes] = field(default_factory=set)
    revoked_lease_sequences: set[int] = field(default_factory=set)
    expired_lease_sequences: set[int] = field(default_factory=set)


def _normalize_canonical_message(message: object) -> None:
    for field, value in message.ListFields():
        if field.label == FieldDescriptor.LABEL_REPEATED:
            if field.type == FieldDescriptor.TYPE_MESSAGE:
                for item in value:
                    _normalize_canonical_message(item)
            elif field.type in (FieldDescriptor.TYPE_DOUBLE, FieldDescriptor.TYPE_FLOAT):
                value[:] = [0.0 if item == 0.0 else item for item in value]
            elif field.type == FieldDescriptor.TYPE_STRING:
                value[:] = [unicodedata.normalize("NFC", item) for item in value]
        elif field.type == FieldDescriptor.TYPE_MESSAGE:
            _normalize_canonical_message(value)
        elif field.type in (FieldDescriptor.TYPE_DOUBLE, FieldDescriptor.TYPE_FLOAT):
            if value == 0.0:
                setattr(message, field.name, 0.0)
        elif field.type == FieldDescriptor.TYPE_STRING:
            setattr(message, field.name, unicodedata.normalize("NFC", value))


def canonical_business_identity(message: object, identity_field: str) -> bytes:
    canonical = type(message)()
    canonical.CopyFrom(message)
    canonical.ClearField(identity_field)
    _normalize_canonical_message(canonical)
    return hashlib.sha256(
        canonical.SerializeToString(deterministic=True)
    ).digest()


def _rehash_plan_lease_bundle(bundle: object) -> None:
    """Refresh the parent identity chain after mutating nested plan content."""
    bundle.plan.plan_content_identity.sha256 = canonical_business_identity(
        bundle.plan, "plan_content_identity"
    )
    bundle.lease.plan_content_identity.CopyFrom(bundle.plan.plan_content_identity)
    bundle.lease.content_identity.sha256 = canonical_business_identity(
        bundle.lease, "content_identity"
    )
    bundle.bundle_content_identity.sha256 = canonical_business_identity(
        bundle, "bundle_content_identity"
    )


def _rehash_trajectory_report_plan_lease_bundle(bundle: object) -> None:
    bundle.plan.trajectory.trajectory_content_identity.sha256 = (
        canonical_business_identity(
            bundle.plan.trajectory,
            "trajectory_content_identity",
        )
    )
    bundle.plan.validation_report.validated_trajectory_content_identity.CopyFrom(
        bundle.plan.trajectory.trajectory_content_identity
    )
    bundle.plan.validation_report.validation_report_content_identity.sha256 = (
        canonical_business_identity(
            bundle.plan.validation_report,
            "validation_report_content_identity",
        )
    )
    _rehash_plan_lease_bundle(bundle)


def _encode_varint(value: int) -> bytes:
    encoded = bytearray()
    while value >= 0x80:
        encoded.append((value & 0x7F) | 0x80)
        value >>= 7
    encoded.append(value)
    return bytes(encoded)


def _valid_identity(identity: object) -> bool:
    return len(identity.sha256) == 32


def _valid_profile(reference: object) -> bool:
    return (
        bool(reference.profile_id)
        and reference.version > 0
        and _valid_identity(reference.content_identity)
    )


def _profile_key(reference: object) -> tuple[str, int, bytes]:
    return (
        reference.profile_id,
        reference.version,
        bytes(reference.content_identity.sha256),
    )


def _valid_complete_dependencies(dependencies: object) -> bool:
    sensor_ids = [sensor.sensor_id for sensor in dependencies.sensors]
    return not (
        dependencies.mission_id == 0
        or dependencies.mission_version == 0
        or not _valid_identity(dependencies.mission_content_identity)
        or not dependencies.map_id
        or dependencies.map_version == 0
        or not _valid_identity(dependencies.map_content_identity)
        or dependencies.navigation_version == 0
        or not _valid_identity(dependencies.navigation_content_identity)
        or not dependencies.sensors
        or sensor_ids != sorted(set(sensor_ids))
        or any(
            not sensor.sensor_id
            or sensor.geometry_version == 0
            or not _valid_identity(sensor.geometry_content_identity)
            or sensor.health_version == 0
            or not _valid_identity(sensor.health_content_identity)
            for sensor in dependencies.sensors
        )
        or not dependencies.current_model_id
        or dependencies.current_model_version == 0
        or not _valid_identity(dependencies.current_content_identity)
        or not _valid_profile(dependencies.capability_profile)
        or dependencies.thruster_health_version == 0
        or not _valid_identity(dependencies.thruster_health_content_identity)
        or not _valid_profile(dependencies.energy_model)
        or not dependencies.energy_store_id
        or dependencies.energy_state_version == 0
        or not _valid_identity(dependencies.energy_state_content_identity)
        or not dependencies.prediction_id
        or dependencies.prediction_version == 0
        or not _valid_identity(dependencies.prediction_content_identity)
        or dependencies.coordination_version == 0
        or not _valid_identity(dependencies.coordination_content_identity)
        or not _valid_profile(dependencies.planner_configuration)
        or not _valid_profile(dependencies.timing_profile)
        or not _valid_profile(dependencies.interface_limits)
    )


def _message_scalars_are_canonical(message: object) -> bool:
    for descriptor, value in message.ListFields():
        if descriptor.label == FieldDescriptor.LABEL_REPEATED:
            if descriptor.type == FieldDescriptor.TYPE_MESSAGE:
                if not all(_message_scalars_are_canonical(item) for item in value):
                    return False
            elif descriptor.type == FieldDescriptor.TYPE_STRING:
                if any(item != unicodedata.normalize("NFC", item) for item in value):
                    return False
            elif descriptor.type in (FieldDescriptor.TYPE_DOUBLE, FieldDescriptor.TYPE_FLOAT):
                if any(not math.isfinite(item) for item in value):
                    return False
        elif descriptor.type == FieldDescriptor.TYPE_MESSAGE:
            if not _message_scalars_are_canonical(value):
                return False
        elif descriptor.type == FieldDescriptor.TYPE_STRING:
            if value != unicodedata.normalize("NFC", value):
                return False
        elif descriptor.type in (FieldDescriptor.TYPE_DOUBLE, FieldDescriptor.TYPE_FLOAT):
            if not math.isfinite(value):
                return False
    return True


def _enum_value_is_known(message: object, field_name: str, value: int) -> bool:
    enum_type = message.DESCRIPTOR.fields_by_name[field_name].enum_type
    return any(enum_value.number == value for enum_value in enum_type.values)


def _enum_number(message: object, field_name: str, value_name: str) -> int:
    enum_type = message.DESCRIPTOR.fields_by_name[field_name].enum_type
    return enum_type.values_by_name[value_name].number


def _close(left: float, right: float) -> bool:
    return math.isclose(left, right, rel_tol=1e-12, abs_tol=1e-12)


def _vector_close(left: tuple[float, ...], right: tuple[float, ...]) -> bool:
    return all(_close(a, b) for a, b in zip(left, right, strict=True))


def _has_unknown_fields(message: object) -> bool:
    known_only = type(message)()
    known_only.CopyFrom(message)
    known_only.DiscardUnknownFields()
    return known_only.SerializeToString(deterministic=True) != message.SerializeToString(
        deterministic=True
    )


def _valid_scout_trajectory(trajectory: object) -> bool:
    if (
        trajectory.frame_id != "mission_enu"
        or not trajectory.HasField("initial_yaw_rad")
        or not math.isfinite(trajectory.initial_yaw_rad)
        or not -math.pi <= trajectory.initial_yaw_rad < math.pi
        or not trajectory.segments
    ):
        return False
    expected_start_ns = 0
    previous_positions: list[tuple[float, float, float]] | None = None
    previous_yaws: tuple[float, ...] | None = None
    previous_duration_s = 0.0
    for segment in trajectory.segments:
        if (
            segment.start_time_offset_ns != expected_start_ns
            or segment.duration_ns == 0
            or len(segment.position_control_points) != 6
            or len(segment.yaw_offset_control_points_rad) != 6
        ):
            return False
        for point in segment.position_control_points:
            if not all(
                point.HasField(axis) and math.isfinite(getattr(point, axis))
                for axis in ("x_m", "y_m", "z_m")
            ):
                return False
        positions = [
            (point.x_m, point.y_m, point.z_m)
            for point in segment.position_control_points
        ]
        yaws = tuple(segment.yaw_offset_control_points_rad)
        if not all(math.isfinite(value) for value in yaws):
            return False
        duration_s = segment.duration_ns / 1_000_000_000.0
        if previous_positions is None or previous_yaws is None:
            if not _close(yaws[0], 0.0):
                return False
        else:
            previous_velocity = tuple(
                5.0 * (previous_positions[5][axis] - previous_positions[4][axis])
                / previous_duration_s
                for axis in range(3)
            )
            velocity = tuple(
                5.0 * (positions[1][axis] - positions[0][axis]) / duration_s
                for axis in range(3)
            )
            previous_acceleration = tuple(
                20.0
                * (
                    previous_positions[5][axis]
                    - 2.0 * previous_positions[4][axis]
                    + previous_positions[3][axis]
                )
                / (previous_duration_s * previous_duration_s)
                for axis in range(3)
            )
            acceleration = tuple(
                20.0
                * (positions[2][axis] - 2.0 * positions[1][axis] + positions[0][axis])
                / (duration_s * duration_s)
                for axis in range(3)
            )
            previous_yaw_rate = (
                5.0 * (previous_yaws[5] - previous_yaws[4]) / previous_duration_s
            )
            yaw_rate = 5.0 * (yaws[1] - yaws[0]) / duration_s
            previous_yaw_acceleration = (
                20.0
                * (previous_yaws[5] - 2.0 * previous_yaws[4] + previous_yaws[3])
                / (previous_duration_s * previous_duration_s)
            )
            yaw_acceleration = (
                20.0 * (yaws[2] - 2.0 * yaws[1] + yaws[0])
                / (duration_s * duration_s)
            )
            if not (
                _vector_close(previous_positions[5], positions[0])
                and _close(previous_yaws[5], yaws[0])
                and _vector_close(previous_velocity, velocity)
                and _vector_close(previous_acceleration, acceleration)
                and _close(previous_yaw_rate, yaw_rate)
                and _close(previous_yaw_acceleration, yaw_acceleration)
            ):
                return False
        expected_start_ns += segment.duration_ns
        previous_positions = positions
        previous_yaws = yaws
        previous_duration_s = duration_s
    return True


def _valid_survey_evidence(plan: object) -> bool:
    evidence = plan.survey_evidence
    dependencies = plan.dependencies
    region = evidence.predicted_covered_region
    return (
        plan.HasField("survey_evidence")
        and evidence.mission_id == dependencies.mission_id
        and evidence.mission_version == dependencies.mission_version
        and hmac.compare_digest(
            evidence.mission_content_identity.sha256,
            dependencies.mission_content_identity.sha256,
        )
        and bool(evidence.baseline_map_id)
        and evidence.baseline_map_id == dependencies.map_id
        and evidence.baseline_map_version == dependencies.map_version
        and hmac.compare_digest(
            evidence.baseline_map_content_identity.sha256,
            dependencies.map_content_identity.sha256,
        )
        and _valid_identity(evidence.evidence_content_identity)
        and hmac.compare_digest(
            evidence.evidence_content_identity.sha256,
            canonical_business_identity(evidence, "evidence_content_identity"),
        )
        and region.frame_id == "mission_enu"
        and len(region.xyz_m) == 6
        and all(math.isfinite(value) for value in region.xyz_m)
        and all(region.xyz_m[index] < region.xyz_m[index + 3] for index in range(3))
        and evidence.HasField("conservative_predicted_coverage_ratio")
        and math.isfinite(evidence.conservative_predicted_coverage_ratio)
        and 0.0 <= evidence.conservative_predicted_coverage_ratio <= 1.0
        and evidence.HasField("predicted_resolution_m")
        and math.isfinite(evidence.predicted_resolution_m)
        and evidence.predicted_resolution_m > 0.0
    )


def validate_scout_bundle(
    bundle: object,
    *,
    now_monotonic_ns: int,
    local_clock_domain_id: str,
    accepted_manifest_identity: bytes,
    maximum_bundle_bytes: int,
    accepted_timing_profile: tuple[str, int, bytes] = ACCEPTED_SCOUT_PROFILE,
    accepted_interface_limits: tuple[str, int, bytes] = ACCEPTED_SCOUT_PROFILE,
    accepted_safety_gate_configuration: tuple[str, int, bytes] = ACCEPTED_SCOUT_PROFILE,
    scout_lease_duration_ns: int = INTEGRATION_TIMING["scout_lease_duration_ns"],
    scout_authorization_start_window_ns: int = INTEGRATION_TIMING[
        "scout_authorization_start_window_ns"
    ],
) -> str:
    if _has_unknown_fields(bundle):
        raise ValueError("INPUT_INVALID: unknown field")
    if not _message_scalars_are_canonical(bundle):
        raise ValueError("INPUT_INVALID: non-canonical scalar")
    if (
        bundle.header.schema_major != 1
        or bundle.header.schema_minor != 0
        or bundle.header.stream_id != 35
        or bundle.header.producer_id != SCOUT_AUTHORITY_PRODUCER_ID
        or len(bundle.header.producer_session_id) != 16
        or not bundle.HasField("domain")
        or bundle.bundle_sequence == 0
        or bundle.header.sequence == 0
        or bundle.header.source_clock_domain_id != local_clock_domain_id
        or not bundle.header.HasField("generated_at_monotonic_ns")
        or not bundle.header.HasField("manifest")
        or bundle.header.manifest.schema_major != 1
        or bundle.header.manifest.schema_minor != 0
        or not hmac.compare_digest(
            bundle.header.manifest.manifest_identity.sha256,
            accepted_manifest_identity,
        )
    ):
        raise ValueError("VERSION_INCOMPATIBLE: wrong Scout authority envelope")
    if bundle.plan.validation_report.status not in (0, 1, 2, 3, 4):
        raise ValueError("INPUT_INVALID: unknown Scout validation status")
    if bundle.plan.validation_report.primary_outcome != 1:
        raise ValueError("INPUT_INVALID: invalid Scout validation outcome")
    report = bundle.plan.validation_report
    required_safe_metrics = (
        "minimum_collision_margin_m",
        "minimum_separation_margin_m",
        "minimum_energy_margin_j",
        "minimum_capability_margin",
        "survey_coverage_ratio",
    )
    if (
        report.HasField("earliest_failure_time_offset_ns")
        or not all(report.HasField(name) for name in required_safe_metrics)
        or report.minimum_collision_margin_m < 0.0
        or report.minimum_separation_margin_m < 0.0
        or report.minimum_energy_margin_j < 0.0
        or report.minimum_capability_margin < 0.0
        or not 0.0 <= report.survey_coverage_ratio <= 1.0
    ):
        raise ValueError("INPUT_INVALID: incomplete Scout SAFE report")
    if (
        not _valid_complete_dependencies(bundle.plan.dependencies)
        or not _valid_complete_dependencies(bundle.lease.dependencies)
        or not _valid_scout_trajectory(bundle.plan.trajectory)
        or not _valid_survey_evidence(bundle.plan)
    ):
        raise ValueError("INPUT_INVALID: malformed Scout plan")
    if (
        bundle.plan.plan_sequence == 0
        or bundle.lease.lease_sequence == 0
        or bundle.plan.plan_sequence != bundle.lease.plan_sequence
        or not _valid_identity(bundle.plan.plan_content_identity)
        or not hmac.compare_digest(
            bundle.plan.plan_content_identity.sha256,
            bundle.lease.plan_content_identity.sha256,
        )
        or not hmac.compare_digest(
            bundle.plan.plan_content_identity.sha256,
            canonical_business_identity(bundle.plan, "plan_content_identity"),
        )
        or bundle.plan.validation_report.status != 1
        or not hmac.compare_digest(
            bundle.plan.validation_report.validated_dependencies_content_identity.sha256,
            bundle.plan.dependencies.dependencies_content_identity.sha256,
        )
        or not hmac.compare_digest(
            bundle.plan.validation_report.validated_trajectory_content_identity.sha256,
            bundle.plan.trajectory.trajectory_content_identity.sha256,
        )
        or not hmac.compare_digest(
            bundle.plan.trajectory.trajectory_content_identity.sha256,
            canonical_business_identity(
                bundle.plan.trajectory,
                "trajectory_content_identity",
            ),
        )
        or not hmac.compare_digest(
            bundle.plan.validation_report.validated_survey_evidence_content_identity.sha256,
            bundle.plan.survey_evidence.evidence_content_identity.sha256,
        )
        or not hmac.compare_digest(
            bundle.plan.validation_report.validation_report_content_identity.sha256,
            canonical_business_identity(
                bundle.plan.validation_report,
                "validation_report_content_identity",
            ),
        )
        or not hmac.compare_digest(
            bundle.plan.dependencies.dependencies_content_identity.sha256,
            canonical_business_identity(
                bundle.plan.dependencies,
                "dependencies_content_identity",
            ),
        )
        or not hmac.compare_digest(
            bundle.lease.dependencies.dependencies_content_identity.sha256,
            canonical_business_identity(
                bundle.lease.dependencies,
                "dependencies_content_identity",
            ),
        )
    ):
        raise ValueError("HASH_MISMATCH: plan identity or validation mismatch")
    if (
        not hmac.compare_digest(
            bundle.plan.dependencies.dependencies_content_identity.sha256,
            bundle.lease.dependencies.dependencies_content_identity.sha256,
        )
        or not hmac.compare_digest(
            bundle.plan.dependencies.timing_profile.SerializeToString(deterministic=True),
            bundle.timing_profile.SerializeToString(deterministic=True),
        )
        or not hmac.compare_digest(
            bundle.plan.dependencies.interface_limits.SerializeToString(deterministic=True),
            bundle.interface_limits.SerializeToString(deterministic=True),
        )
        or not all(
            _valid_profile(reference)
            for reference in (
                bundle.timing_profile,
                bundle.interface_limits,
                bundle.safety_gate_configuration,
            )
        )
        or _profile_key(bundle.timing_profile) != accepted_timing_profile
        or _profile_key(bundle.interface_limits) != accepted_interface_limits
        or _profile_key(bundle.safety_gate_configuration)
        != accepted_safety_gate_configuration
    ):
        raise ValueError("DEPENDENCY_STALE: dependency or profile mismatch")
    trajectory_end_ns = sum(
        segment.duration_ns for segment in bundle.plan.trajectory.segments
    )
    if (
        not bundle.HasField("valid_not_before_monotonic_ns")
        or not bundle.HasField("execution_epoch_monotonic_ns")
        or not bundle.lease.HasField("validated_at_monotonic_ns")
        or not bundle.lease.HasField("expires_at_monotonic_ns")
        or not bundle.lease.HasField("authorized_start_time_offset_ns")
        or not bundle.plan.HasField("created_at_monotonic_ns")
        or bundle.plan.created_at_monotonic_ns
        > bundle.lease.validated_at_monotonic_ns
        or bundle.lease.validated_at_monotonic_ns
        > bundle.header.generated_at_monotonic_ns
        or bundle.header.generated_at_monotonic_ns > now_monotonic_ns
        or bundle.header.generated_at_monotonic_ns
        > bundle.valid_not_before_monotonic_ns
        or now_monotonic_ns < bundle.valid_not_before_monotonic_ns
        or now_monotonic_ns >= bundle.execution_epoch_monotonic_ns
        or bundle.valid_not_before_monotonic_ns > bundle.execution_epoch_monotonic_ns
        or bundle.lease.validated_at_monotonic_ns > now_monotonic_ns
        or bundle.lease.expires_at_monotonic_ns
        < bundle.execution_epoch_monotonic_ns
        + bundle.lease.authorized_end_time_offset_ns
        or bundle.lease.expires_at_monotonic_ns
        - bundle.lease.validated_at_monotonic_ns
        > scout_lease_duration_ns
        or bundle.execution_epoch_monotonic_ns
        - bundle.valid_not_before_monotonic_ns
        > scout_authorization_start_window_ns
        or bundle.lease.authorized_start_time_offset_ns
        >= bundle.lease.authorized_end_time_offset_ns
        or bundle.lease.authorized_end_time_offset_ns > trajectory_end_ns
    ):
        raise ValueError("SEQUENCE_REJECTED: invalid or missed authorization window")
    if (
        not hmac.compare_digest(
            bundle.lease.content_identity.sha256,
            canonical_business_identity(bundle.lease, "content_identity"),
        )
        or not hmac.compare_digest(
            bundle.bundle_content_identity.sha256,
            canonical_business_identity(bundle, "bundle_content_identity"),
        )
    ):
        raise ValueError("HASH_MISMATCH: lease or bundle identity mismatch")
    if len(bundle.SerializeToString(deterministic=True)) > maximum_bundle_bytes:
        raise ValueError("INPUT_INVALID: Scout bundle exceeds resource limit")
    return "accepted"


def validate_scout_bundle_ack(
    ack: object,
    *,
    installed_bundle: object,
    now_monotonic_ns: int,
    local_clock_domain_id: str,
    accepted_manifest_identity: bytes,
    scout_bundle_ack_timeout_ns: int = INTEGRATION_TIMING[
        "scout_authorized_bundle_ack_timeout_ns"
    ],
) -> str:
    if _has_unknown_fields(ack):
        raise ValueError("INPUT_INVALID: unknown ACK field")
    if not _message_scalars_are_canonical(ack):
        raise ValueError("INPUT_INVALID: non-canonical ACK scalar")
    if (
        ack.header.schema_major != 1
        or ack.header.schema_minor != 0
        or ack.header.producer_id != SCOUT_ACK_PRODUCER_ID
        or len(ack.header.producer_session_id) != 16
        or ack.header.stream_id != 36
        or ack.header.sequence == 0
        or ack.header.source_clock_domain_id != local_clock_domain_id
        or not ack.header.HasField("generated_at_monotonic_ns")
        or ack.header.generated_at_monotonic_ns > now_monotonic_ns
        or not ack.header.HasField("manifest")
        or ack.header.manifest.schema_major != 1
        or ack.header.manifest.schema_minor != 0
        or not hmac.compare_digest(
            ack.header.manifest.manifest_identity.sha256,
            accepted_manifest_identity,
        )
    ):
        raise ValueError("VERSION_INCOMPATIBLE: wrong Scout ACK envelope")
    installed_state = _enum_number(ack, "state", "BUNDLE_ACK_INSTALLED")
    rejected_state = _enum_number(ack, "state", "BUNDLE_ACK_REJECTED")
    success_outcome = _enum_number(ack, "outcome", "OUTCOME_SUCCESS")
    unspecified_outcome = _enum_number(
        ack, "outcome", "OUTCOME_CODE_UNSPECIFIED"
    )
    if ack.state not in (installed_state, rejected_state) or not _enum_value_is_known(
        ack, "outcome", ack.outcome
    ):
        raise ValueError("INPUT_INVALID: unknown Scout ACK enum")
    if (
        ack.bundle_sequence != installed_bundle.bundle_sequence
        or ack.plan_sequence != installed_bundle.plan.plan_sequence
        or ack.lease_sequence != installed_bundle.lease.lease_sequence
        or not _valid_identity(ack.observed_bundle_identity)
        or not hmac.compare_digest(
            ack.observed_bundle_identity.sha256,
            installed_bundle.bundle_content_identity.sha256,
        )
    ):
        raise ValueError("HASH_MISMATCH: Scout ACK binding mismatch")
    if ack.state == installed_state:
        if (
            ack.outcome != success_outcome
            or not ack.HasField("installed_at_monotonic_ns")
            or ack.installed_at_monotonic_ns
            < installed_bundle.valid_not_before_monotonic_ns
            or ack.installed_at_monotonic_ns
            >= installed_bundle.execution_epoch_monotonic_ns
            or ack.installed_at_monotonic_ns
            > ack.header.generated_at_monotonic_ns
            or ack.header.generated_at_monotonic_ns
            - ack.installed_at_monotonic_ns
            > scout_bundle_ack_timeout_ns
        ):
            raise ValueError("INPUT_INVALID: invalid installed Scout ACK")
    elif ack.outcome in (unspecified_outcome, success_outcome) or ack.HasField(
        "installed_at_monotonic_ns"
    ):
        raise ValueError("INPUT_INVALID: invalid rejected Scout ACK")
    return "accepted"


def apply_scout_bundle_watermark(
    state: ScoutBundleWatermarkState,
    bundle: object,
    *,
    now_monotonic_ns: int,
    recovery_boundary: bool,
) -> str:
    incoming_session = bytes(bundle.header.producer_session_id)
    current_session = state.producer_session_id
    if incoming_session in state.retired_sessions:
        raise ValueError("SEQUENCE_REJECTED: retired Scout authority session")
    if incoming_session != current_session:
        if not recovery_boundary:
            raise ValueError("SEQUENCE_REJECTED: new session requires recovery boundary")
        if bundle.lease.expires_at_monotonic_ns <= now_monotonic_ns:
            # Do not partially install an already stale producer restart. The
            # rejected session is retired so changing content cannot revive it.
            state.retired_sessions.add(incoming_session)
            raise ValueError("EXPIRED: new Scout authority session is already expired")
        state.retired_sessions.add(current_session)
        state.producer_session_id = incoming_session
        state.delivery_sequence = bundle.header.sequence
        state.bundle_sequence = bundle.bundle_sequence
        state.plan_sequence = bundle.plan.plan_sequence
        state.lease_sequence = bundle.lease.lease_sequence
        state.bundle_identity = bytes(bundle.bundle_content_identity.sha256)
        state.revoked_lease_sequences = set()
        state.expired_lease_sequences = set()
        return "new session installed"

    if bundle.lease.lease_sequence in state.revoked_lease_sequences:
        raise ValueError("REVOKED: Scout lease watermark cannot revive")
    if bundle.lease.lease_sequence in state.expired_lease_sequences:
        raise ValueError("EXPIRED: Scout lease watermark cannot revive")
    if bundle.lease.expires_at_monotonic_ns <= now_monotonic_ns:
        state.expired_lease_sequences.add(bundle.lease.lease_sequence)
        raise ValueError("EXPIRED: Scout lease already expired")

    current_delivery = state.delivery_sequence
    current_bundle = state.bundle_sequence
    current_plan = state.plan_sequence
    current_lease = state.lease_sequence
    current_identity = state.bundle_identity
    incoming_identity = bytes(bundle.bundle_content_identity.sha256)
    if bundle.header.sequence < current_delivery:
        raise ValueError("SEQUENCE_REJECTED: Scout bundle delivery rollback")
    if bundle.header.sequence == current_delivery:
        if (
            bundle.bundle_sequence == current_bundle
            and bundle.plan.plan_sequence == current_plan
            and bundle.lease.lease_sequence == current_lease
            and hmac.compare_digest(incoming_identity, current_identity)
        ):
            return "idempotent duplicate"
        raise ValueError("INPUT_INVALID: same Scout delivery sequence conflicts")
    if (
        bundle.bundle_sequence <= current_bundle
        or bundle.plan.plan_sequence <= current_plan
        or bundle.lease.lease_sequence <= current_lease
        or hmac.compare_digest(incoming_identity, current_identity)
    ):
        raise ValueError("SEQUENCE_REJECTED: Scout business watermark reused")
    state.delivery_sequence = bundle.header.sequence
    state.bundle_sequence = bundle.bundle_sequence
    state.plan_sequence = bundle.plan.plan_sequence
    state.lease_sequence = bundle.lease.lease_sequence
    state.bundle_identity = incoming_identity
    return "installed"


class ScoutAuthorizationBundleContractTest(unittest.TestCase):
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
        cls.execution = importlib.import_module("underwater.contracts.v1.execution_pb2")
        cls.planning = importlib.import_module("underwater.contracts.v1.planning_pb2")
        cls.cooperation = importlib.import_module("underwater.contracts.v1.cooperation_pb2")
        cls.codes = importlib.import_module("underwater.contracts.v1.codes_pb2")
        cls.accepted_manifest_identity = b"m" * 32

    @classmethod
    def tearDownClass(cls) -> None:
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def test_schema_has_dedicated_scout_bundle_lease_ack_and_streams(self) -> None:
        self.assertTrue(hasattr(self.execution, "ScoutExecutionLease"))
        self.assertTrue(hasattr(self.execution, "ScoutAuthorizedExecutionBundle"))
        self.assertTrue(hasattr(self.execution, "ScoutBundleAck"))
        self.assertTrue(hasattr(self.common, "STREAM_SCOUT_AUTHORIZED_EXECUTION_BUNDLE"))
        self.assertTrue(hasattr(self.common, "STREAM_SCOUT_BUNDLE_ACK"))
        self.assertNotEqual(
            self.common.STREAM_SCOUT_AUTHORIZED_EXECUTION_BUNDLE,
            self.common.STREAM_AUTHORIZED_EXECUTION_BUNDLE,
        )

    def test_bundle_round_trip_atomically_binds_plan_lease_interval_and_profiles(self) -> None:
        identity = self.common.ContentIdentity(sha256=b"i" * 32)
        profile = self.common.ProfileRef(
            profile_id="scout/profile/v1", version=1, content_identity=identity
        )
        plan = self.planning.ScoutPlan(plan_sequence=41, plan_content_identity=identity)
        lease = self.execution.ScoutExecutionLease(
            lease_sequence=7,
            plan_sequence=41,
            plan_content_identity=identity,
            validated_at_monotonic_ns=1_000,
            expires_at_monotonic_ns=3_000,
            authorized_start_time_offset_ns=0,
            authorized_end_time_offset_ns=1_000,
            content_identity=identity,
        )
        bundle = self.execution.ScoutAuthorizedExecutionBundle(
            domain=self.execution.ScoutMotionExecutionAuthorityDomain(),
            bundle_sequence=9,
            plan=plan,
            lease=lease,
            valid_not_before_monotonic_ns=1_500,
            execution_epoch_monotonic_ns=2_000,
            timing_profile=profile,
            interface_limits=profile,
            safety_gate_configuration=profile,
            bundle_content_identity=identity,
        )

        decoded = self.execution.ScoutAuthorizedExecutionBundle.FromString(
            bundle.SerializeToString(deterministic=True)
        )
        self.assertEqual(decoded.plan.plan_sequence, 41)
        self.assertEqual(decoded.lease.lease_sequence, 7)
        self.assertTrue(decoded.lease.HasField("authorized_start_time_offset_ns"))
        self.assertEqual(decoded.lease.authorized_end_time_offset_ns, 1_000)
        self.assertEqual(decoded.execution_epoch_monotonic_ns, 2_000)
        self.assertEqual(decoded.safety_gate_configuration.profile_id, "scout/profile/v1")
        self.assertFalse(hasattr(decoded, "execution_profile"))
        self.assertFalse(hasattr(decoded.plan, "execution_profile"))

    def _valid_bundle(self) -> object:
        identity = self.common.ContentIdentity(sha256=b"p" * 32)
        profile = self.common.ProfileRef(
            profile_id="scout/profile/v1", version=1, content_identity=identity
        )
        dependencies = self.planning.ScoutPlanningDependencies(
            mission_id=5,
            mission_version=2,
            mission_content_identity=identity,
            map_id="survey-map",
            map_version=7,
            map_content_identity=identity,
            navigation_version=12,
            navigation_content_identity=identity,
            sensors=[
                self.planning.ScoutSensorDependency(
                    sensor_id="forward-sonar",
                    geometry_version=2,
                    geometry_content_identity=identity,
                    health_version=5,
                    health_content_identity=identity,
                )
            ],
            current_model_id="current/local-affine",
            current_model_version=3,
            current_content_identity=identity,
            capability_profile=profile,
            thruster_health_version=6,
            thruster_health_content_identity=identity,
            energy_model=profile,
            energy_store_id="battery-a",
            energy_state_version=7,
            energy_state_content_identity=identity,
            prediction_id="laying-prediction",
            prediction_version=9,
            prediction_content_identity=identity,
            coordination_version=11,
            coordination_content_identity=identity,
            planner_configuration=profile,
            timing_profile=profile,
            interface_limits=profile,
        )
        dependencies.dependencies_content_identity.sha256 = canonical_business_identity(
            dependencies, "dependencies_content_identity"
        )
        trajectory = self.planning.ScoutTrajectory4d(
            frame_id="mission_enu",
            initial_yaw_rad=0.0,
            segments=[
                self.planning.ScoutBezierSegment4d(
                    start_time_offset_ns=0,
                    duration_ns=1_000,
                    position_control_points=[
                        self.planning.ScoutBezierControlPoint3dEnu(
                            x_m=float(index),
                            y_m=float(index) / 2.0,
                            z_m=-1.0,
                        )
                        for index in range(6)
                    ],
                    yaw_offset_control_points_rad=[0.0] * 6,
                )
            ],
        )
        trajectory.trajectory_content_identity.sha256 = canonical_business_identity(
            trajectory, "trajectory_content_identity"
        )
        survey_evidence = self.cooperation.SurveyPlanEvidence(
            mission_id=5,
            mission_version=2,
            mission_content_identity=identity,
            baseline_map_id="survey-map",
            baseline_map_version=7,
            conservative_predicted_coverage_ratio=0.9,
            predicted_resolution_m=0.1,
            predicted_covered_region=self.cooperation.Region3dEnu(
                xyz_m=[0.0, 0.0, -2.0, 5.0, 5.0, -0.5],
                frame_id="mission_enu",
            ),
            baseline_map_content_identity=identity,
        )
        survey_evidence.evidence_content_identity.sha256 = canonical_business_identity(
            survey_evidence, "evidence_content_identity"
        )
        report = self.planning.ScoutPlanValidationReport(
            status=self.planning.SCOUT_PLAN_VALIDATION_SAFE,
            primary_outcome=self.codes.OUTCOME_SUCCESS,
            minimum_collision_margin_m=1.1,
            minimum_separation_margin_m=1.2,
            minimum_energy_margin_j=2_000.0,
            minimum_capability_margin=0.15,
            survey_coverage_ratio=0.9,
            refinement_depth=4,
            validated_dependencies_content_identity=dependencies.dependencies_content_identity,
            validated_trajectory_content_identity=trajectory.trajectory_content_identity,
            validated_survey_evidence_content_identity=survey_evidence.evidence_content_identity,
        )
        report.validation_report_content_identity.sha256 = canonical_business_identity(
            report, "validation_report_content_identity"
        )
        plan = self.planning.ScoutPlan(
            plan_sequence=41,
            created_at_monotonic_ns=1_000,
            trajectory=trajectory,
            dependencies=dependencies,
            survey_evidence=survey_evidence,
            validation_report=report,
        )
        plan.plan_content_identity.sha256 = canonical_business_identity(
            plan, "plan_content_identity"
        )
        lease = self.execution.ScoutExecutionLease(
            lease_sequence=7,
            plan_sequence=41,
            plan_content_identity=plan.plan_content_identity,
            validated_at_monotonic_ns=1_200,
            expires_at_monotonic_ns=3_000,
            authorized_start_time_offset_ns=0,
            authorized_end_time_offset_ns=800,
            dependencies=dependencies,
        )
        lease.content_identity.sha256 = canonical_business_identity(
            lease, "content_identity"
        )
        header = self.common.MessageHeader(
            schema_major=1,
            schema_minor=0,
            producer_id="scout-execution-authority",
            producer_session_id=b"s" * 16,
            stream_id=self.common.STREAM_SCOUT_AUTHORIZED_EXECUTION_BUNDLE,
            sequence=9,
            source_clock_domain_id="scout-nuc/boot-9",
            generated_at_monotonic_ns=1_300,
            manifest=self.common.ContractManifestRef(
                schema_major=1,
                schema_minor=0,
                manifest_identity=self.common.ContentIdentity(
                    sha256=self.accepted_manifest_identity
                ),
            ),
        )
        bundle = self.execution.ScoutAuthorizedExecutionBundle(
            header=header,
            domain=self.execution.ScoutMotionExecutionAuthorityDomain(),
            bundle_sequence=9,
            plan=plan,
            lease=lease,
            valid_not_before_monotonic_ns=1_400,
            execution_epoch_monotonic_ns=2_000,
            timing_profile=profile,
            interface_limits=profile,
            safety_gate_configuration=profile,
        )
        bundle.bundle_content_identity.sha256 = canonical_business_identity(
            bundle, "bundle_content_identity"
        )
        return bundle

    def test_complete_exact_bundle_is_accepted_before_its_start_window(self) -> None:
        bundle = self._valid_bundle()
        self.assertEqual(
            validate_scout_bundle(
                bundle,
                now_monotonic_ns=1_500,
                local_clock_domain_id="scout-nuc/boot-9",
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_bundle_bytes=65_536,
            ),
            "accepted",
        )

    def test_wrong_schema_version_is_rejected_even_when_the_message_parses(self) -> None:
        bundle = self._valid_bundle()
        bundle.header.schema_major = 2
        bundle.bundle_content_identity.sha256 = canonical_business_identity(
            bundle, "bundle_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "VERSION_INCOMPATIBLE"):
            validate_scout_bundle(
                bundle,
                now_monotonic_ns=1_500,
                local_clock_domain_id="scout-nuc/boot-9",
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_bundle_bytes=65_536,
            )

    def test_noncanonical_scout_authority_publisher_is_rejected(self) -> None:
        bundle = self._valid_bundle()
        bundle.header.producer_id = "main-execution-authority"
        bundle.bundle_content_identity.sha256 = canonical_business_identity(
            bundle, "bundle_content_identity"
        )

        with self.assertRaisesRegex(ValueError, "VERSION_INCOMPATIBLE"):
            validate_scout_bundle(
                bundle,
                now_monotonic_ns=1_500,
                local_clock_domain_id="scout-nuc/boot-9",
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_bundle_bytes=65_536,
            )

    def test_nested_trajectory_identity_is_recomputed_before_authority(self) -> None:
        bundle = self._valid_bundle()
        bundle.plan.trajectory.initial_yaw_rad = 0.25
        _rehash_plan_lease_bundle(bundle)

        with self.assertRaisesRegex(ValueError, "HASH_MISMATCH"):
            validate_scout_bundle(
                bundle,
                now_monotonic_ns=1_500,
                local_clock_domain_id="scout-nuc/boot-9",
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_bundle_bytes=65_536,
            )

    def test_incomplete_quintic_segment_cannot_receive_authority(self) -> None:
        bundle = self._valid_bundle()
        bundle.plan.trajectory.segments[0].ClearField("position_control_points")
        bundle.plan.trajectory.trajectory_content_identity.sha256 = (
            canonical_business_identity(
                bundle.plan.trajectory,
                "trajectory_content_identity",
            )
        )
        bundle.plan.validation_report.validated_trajectory_content_identity.CopyFrom(
            bundle.plan.trajectory.trajectory_content_identity
        )
        bundle.plan.validation_report.validation_report_content_identity.sha256 = (
            canonical_business_identity(
                bundle.plan.validation_report,
                "validation_report_content_identity",
            )
        )
        _rehash_plan_lease_bundle(bundle)

        with self.assertRaisesRegex(ValueError, "INPUT_INVALID"):
            validate_scout_bundle(
                bundle,
                now_monotonic_ns=1_500,
                local_clock_domain_id="scout-nuc/boot-9",
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_bundle_bytes=65_536,
            )

    def test_trajectory_start_and_c2_continuity_are_revalidated(self) -> None:
        first_yaw_wrong = self._valid_bundle()
        first_yaw_wrong.plan.trajectory.segments[0].yaw_offset_control_points_rad[0] = 1.0
        _rehash_trajectory_report_plan_lease_bundle(first_yaw_wrong)

        discontinuous = self._valid_bundle()
        discontinuous.plan.trajectory.segments.add(
            start_time_offset_ns=1_000,
            duration_ns=1_000,
            position_control_points=[
                self.planning.ScoutBezierControlPoint3dEnu(
                    x_m=float(index),
                    y_m=float(index) / 2.0,
                    z_m=-1.0,
                )
                for index in range(6, 12)
            ],
            yaw_offset_control_points_rad=[0.0] * 6,
        )
        discontinuous.lease.authorized_end_time_offset_ns = 1_800
        discontinuous.lease.expires_at_monotonic_ns = 4_000
        _rehash_trajectory_report_plan_lease_bundle(discontinuous)

        for name, bundle in (
            ("first yaw offset", first_yaw_wrong),
            ("position C2 discontinuity", discontinuous),
        ):
            with self.subTest(name=name):
                with self.assertRaisesRegex(ValueError, "INPUT_INVALID"):
                    validate_scout_bundle(
                        bundle,
                        now_monotonic_ns=1_500,
                        local_clock_domain_id="scout-nuc/boot-9",
                        accepted_manifest_identity=self.accepted_manifest_identity,
                        maximum_bundle_bytes=65_536,
                    )

    def test_validation_report_must_bind_exact_survey_evidence(self) -> None:
        bundle = self._valid_bundle()
        bundle.plan.validation_report.validated_survey_evidence_content_identity.sha256 = (
            b"e" * 32
        )
        bundle.plan.validation_report.validation_report_content_identity.sha256 = (
            canonical_business_identity(
                bundle.plan.validation_report,
                "validation_report_content_identity",
            )
        )
        _rehash_plan_lease_bundle(bundle)

        with self.assertRaisesRegex(ValueError, "HASH_MISMATCH"):
            validate_scout_bundle(
                bundle,
                now_monotonic_ns=1_500,
                local_clock_domain_id="scout-nuc/boot-9",
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_bundle_bytes=65_536,
            )

    def test_unknown_safety_outcome_enum_fails_closed(self) -> None:
        bundle = self._valid_bundle()
        bundle.plan.validation_report.primary_outcome = 999
        bundle.plan.validation_report.validation_report_content_identity.sha256 = (
            canonical_business_identity(
                bundle.plan.validation_report,
                "validation_report_content_identity",
            )
        )
        _rehash_plan_lease_bundle(bundle)

        with self.assertRaisesRegex(ValueError, "INPUT_INVALID"):
            validate_scout_bundle(
                bundle,
                now_monotonic_ns=1_500,
                local_clock_domain_id="scout-nuc/boot-9",
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_bundle_bytes=65_536,
            )

    def test_safe_report_requires_complete_finite_hard_gate_metrics(self) -> None:
        bundle = self._valid_bundle()
        bundle.plan.validation_report.minimum_collision_margin_m = math.nan
        bundle.plan.validation_report.validation_report_content_identity.sha256 = (
            canonical_business_identity(
                bundle.plan.validation_report,
                "validation_report_content_identity",
            )
        )
        _rehash_plan_lease_bundle(bundle)

        with self.assertRaisesRegex(ValueError, "INPUT_INVALID"):
            validate_scout_bundle(
                bundle,
                now_monotonic_ns=1_500,
                local_clock_domain_id="scout-nuc/boot-9",
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_bundle_bytes=65_536,
            )

    def test_incomplete_planning_dependencies_cannot_receive_authority(self) -> None:
        bundle = self._valid_bundle()
        bundle.plan.dependencies.ClearField("sensors")
        bundle.lease.dependencies.ClearField("sensors")
        for dependencies in (bundle.plan.dependencies, bundle.lease.dependencies):
            dependencies.dependencies_content_identity.sha256 = canonical_business_identity(
                dependencies, "dependencies_content_identity"
            )
        bundle.plan.validation_report.validated_dependencies_content_identity.CopyFrom(
            bundle.plan.dependencies.dependencies_content_identity
        )
        bundle.plan.validation_report.validation_report_content_identity.sha256 = (
            canonical_business_identity(
                bundle.plan.validation_report,
                "validation_report_content_identity",
            )
        )
        _rehash_plan_lease_bundle(bundle)

        with self.assertRaisesRegex(ValueError, "INPUT_INVALID"):
            validate_scout_bundle(
                bundle,
                now_monotonic_ns=1_500,
                local_clock_domain_id="scout-nuc/boot-9",
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_bundle_bytes=65_536,
            )

    def test_unaccepted_safety_gate_profile_is_rejected(self) -> None:
        bundle = self._valid_bundle()
        bundle.safety_gate_configuration.profile_id = "unexpected/safety/v99"
        bundle.safety_gate_configuration.version = 99
        bundle.safety_gate_configuration.content_identity.sha256 = b"u" * 32
        bundle.bundle_content_identity.sha256 = canonical_business_identity(
            bundle, "bundle_content_identity"
        )

        with self.assertRaisesRegex(ValueError, "DEPENDENCY_STALE"):
            validate_scout_bundle(
                bundle,
                now_monotonic_ns=1_500,
                local_clock_domain_id="scout-nuc/boot-9",
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_bundle_bytes=65_536,
            )

    def test_delivery_and_business_sequences_are_orthogonal(self) -> None:
        bundle = self._valid_bundle()
        bundle.header.sequence = 14
        bundle.bundle_content_identity.sha256 = canonical_business_identity(
            bundle, "bundle_content_identity"
        )

        self.assertEqual(
            validate_scout_bundle(
                bundle,
                now_monotonic_ns=1_500,
                local_clock_domain_id="scout-nuc/boot-9",
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_bundle_bytes=65_536,
            ),
            "accepted",
        )

    def test_profile_lease_and_start_window_bounds_are_enforced(self) -> None:
        overlong_lease = self._valid_bundle()
        overlong_lease.lease.expires_at_monotonic_ns = 500_001_201
        overlong_lease.lease.content_identity.sha256 = canonical_business_identity(
            overlong_lease.lease, "content_identity"
        )
        overlong_lease.bundle_content_identity.sha256 = canonical_business_identity(
            overlong_lease, "bundle_content_identity"
        )

        overlong_start_window = self._valid_bundle()
        overlong_start_window.execution_epoch_monotonic_ns = 150_001_401
        overlong_start_window.lease.expires_at_monotonic_ns = 150_002_500
        overlong_start_window.lease.content_identity.sha256 = canonical_business_identity(
            overlong_start_window.lease, "content_identity"
        )
        overlong_start_window.bundle_content_identity.sha256 = canonical_business_identity(
            overlong_start_window, "bundle_content_identity"
        )

        for name, bundle in (
            ("lease duration", overlong_lease),
            ("start window", overlong_start_window),
        ):
            with self.subTest(name=name):
                with self.assertRaisesRegex(ValueError, "SEQUENCE_REJECTED"):
                    validate_scout_bundle(
                        bundle,
                        now_monotonic_ns=1_500,
                        local_clock_domain_id="scout-nuc/boot-9",
                        accepted_manifest_identity=self.accepted_manifest_identity,
                        maximum_bundle_bytes=65_536,
                    )

    def test_safe_label_cannot_replace_exact_validation_dependency_binding(self) -> None:
        bundle = self._valid_bundle()
        bundle.plan.validation_report.validated_dependencies_content_identity.sha256 = b"x" * 32
        bundle.plan.validation_report.validation_report_content_identity.sha256 = (
            canonical_business_identity(
                bundle.plan.validation_report,
                "validation_report_content_identity",
            )
        )
        _rehash_plan_lease_bundle(bundle)

        with self.assertRaisesRegex(ValueError, "HASH_MISMATCH"):
            validate_scout_bundle(
                bundle,
                now_monotonic_ns=1_500,
                local_clock_domain_id="scout-nuc/boot-9",
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_bundle_bytes=65_536,
            )

    def test_dependency_content_must_match_its_bound_identity(self) -> None:
        bundle = self._valid_bundle()
        bundle.plan.dependencies.map_version = 99
        bundle.lease.dependencies.map_version = 99
        bundle.plan.survey_evidence.baseline_map_version = 99
        bundle.plan.survey_evidence.evidence_content_identity.sha256 = (
            canonical_business_identity(
                bundle.plan.survey_evidence,
                "evidence_content_identity",
            )
        )
        bundle.plan.validation_report.validated_survey_evidence_content_identity.CopyFrom(
            bundle.plan.survey_evidence.evidence_content_identity
        )
        bundle.plan.validation_report.validation_report_content_identity.sha256 = (
            canonical_business_identity(
                bundle.plan.validation_report,
                "validation_report_content_identity",
            )
        )
        _rehash_plan_lease_bundle(bundle)

        with self.assertRaisesRegex(ValueError, "HASH_MISMATCH"):
            validate_scout_bundle(
                bundle,
                now_monotonic_ns=1_500,
                local_clock_domain_id="scout-nuc/boot-9",
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_bundle_bytes=65_536,
            )

    def test_domain_clock_plan_profile_and_window_mismatches_fail_closed(self) -> None:
        cases: list[tuple[str, object, int, str]] = []

        wrong_domain = self._valid_bundle()
        wrong_domain.ClearField("domain")
        wrong_domain.bundle_content_identity.sha256 = canonical_business_identity(
            wrong_domain, "bundle_content_identity"
        )
        cases.append(("wrong domain", wrong_domain, 1_500, "VERSION_INCOMPATIBLE"))

        wrong_clock = self._valid_bundle()
        wrong_clock.header.source_clock_domain_id = "main-nuc/boot-4"
        wrong_clock.bundle_content_identity.sha256 = canonical_business_identity(
            wrong_clock, "bundle_content_identity"
        )
        cases.append(("wrong clock", wrong_clock, 1_500, "VERSION_INCOMPATIBLE"))

        wrong_plan = self._valid_bundle()
        wrong_plan.plan.plan_content_identity.sha256 = b"z" * 32
        wrong_plan.lease.plan_content_identity.sha256 = b"z" * 32
        wrong_plan.lease.content_identity.sha256 = canonical_business_identity(
            wrong_plan.lease, "content_identity"
        )
        wrong_plan.bundle_content_identity.sha256 = canonical_business_identity(
            wrong_plan, "bundle_content_identity"
        )
        cases.append(("wrong plan", wrong_plan, 1_500, "HASH_MISMATCH"))

        wrong_profile = self._valid_bundle()
        wrong_profile.timing_profile.version = 2
        wrong_profile.bundle_content_identity.sha256 = canonical_business_identity(
            wrong_profile, "bundle_content_identity"
        )
        cases.append(("wrong profile", wrong_profile, 1_500, "DEPENDENCY_STALE"))

        missed_start = self._valid_bundle()
        cases.append(("missed start", missed_start, 2_000, "SEQUENCE_REJECTED"))

        invalid_interval = self._valid_bundle()
        invalid_interval.lease.authorized_end_time_offset_ns = 1_001
        invalid_interval.lease.content_identity.sha256 = canonical_business_identity(
            invalid_interval.lease, "content_identity"
        )
        invalid_interval.bundle_content_identity.sha256 = canonical_business_identity(
            invalid_interval, "bundle_content_identity"
        )
        cases.append(("outside validated trajectory", invalid_interval, 1_500, "SEQUENCE_REJECTED"))

        for name, bundle, now_ns, expected in cases:
            with self.subTest(name=name):
                with self.assertRaisesRegex(ValueError, expected):
                    validate_scout_bundle(
                        bundle,
                        now_monotonic_ns=now_ns,
                        local_clock_domain_id="scout-nuc/boot-9",
                        accepted_manifest_identity=self.accepted_manifest_identity,
                        maximum_bundle_bytes=65_536,
                    )

    def test_install_ack_binds_exact_scout_bundle_plan_lease_and_identity(self) -> None:
        ack = self.execution.ScoutBundleAck(
            header=self.common.MessageHeader(
                stream_id=self.common.STREAM_SCOUT_BUNDLE_ACK,
                source_clock_domain_id="scout-nuc/boot-9",
            ),
            bundle_sequence=9,
            plan_sequence=41,
            lease_sequence=7,
            state=self.execution.BUNDLE_ACK_INSTALLED,
            installed_at_monotonic_ns=1_500,
            observed_bundle_identity=self.common.ContentIdentity(sha256=b"b" * 32),
            outcome=self.codes.OUTCOME_SUCCESS,
        )
        decoded = self.execution.ScoutBundleAck.FromString(
            ack.SerializeToString(deterministic=True)
        )
        self.assertEqual(decoded.header.stream_id, self.common.STREAM_SCOUT_BUNDLE_ACK)
        self.assertEqual(decoded.bundle_sequence, 9)
        self.assertEqual(decoded.plan_sequence, 41)
        self.assertEqual(decoded.lease_sequence, 7)
        self.assertEqual(decoded.state, self.execution.BUNDLE_ACK_INSTALLED)
        self.assertEqual(decoded.observed_bundle_identity.sha256, b"b" * 32)

    def test_ack_consumer_rejects_wrong_binding_envelope_and_unknown_enums(self) -> None:
        bundle = self._valid_bundle()
        ack = self.execution.ScoutBundleAck(
            header=self.common.MessageHeader(
                schema_major=1,
                schema_minor=0,
                producer_id="scout-fcu-adapter",
                producer_session_id=b"f" * 16,
                stream_id=self.common.STREAM_SCOUT_BUNDLE_ACK,
                sequence=23,
                source_clock_domain_id="scout-nuc/boot-9",
                generated_at_monotonic_ns=1_550,
                manifest=bundle.header.manifest,
            ),
            bundle_sequence=bundle.bundle_sequence,
            plan_sequence=bundle.plan.plan_sequence,
            lease_sequence=bundle.lease.lease_sequence,
            state=self.execution.BUNDLE_ACK_INSTALLED,
            installed_at_monotonic_ns=1_500,
            observed_bundle_identity=bundle.bundle_content_identity,
            outcome=self.codes.OUTCOME_SUCCESS,
        )
        self.assertEqual(
            validate_scout_bundle_ack(
                ack,
                installed_bundle=bundle,
                now_monotonic_ns=1_600,
                local_clock_domain_id="scout-nuc/boot-9",
                accepted_manifest_identity=self.accepted_manifest_identity,
            ),
            "accepted",
        )

        cases: list[tuple[str, object]] = []
        wrong_identity = self.execution.ScoutBundleAck()
        wrong_identity.CopyFrom(ack)
        wrong_identity.observed_bundle_identity.sha256 = b"x" * 32
        cases.append(("wrong identity", wrong_identity))
        wrong_publisher = self.execution.ScoutBundleAck()
        wrong_publisher.CopyFrom(ack)
        wrong_publisher.header.producer_id = "scout-execution-authority"
        cases.append(("wrong publisher", wrong_publisher))
        wrong_clock = self.execution.ScoutBundleAck()
        wrong_clock.CopyFrom(ack)
        wrong_clock.header.source_clock_domain_id = "main-nuc/boot-4"
        cases.append(("wrong clock", wrong_clock))
        unknown_state = self.execution.ScoutBundleAck()
        unknown_state.CopyFrom(ack)
        unknown_state.state = 999
        cases.append(("unknown state", unknown_state))
        unknown_outcome = self.execution.ScoutBundleAck()
        unknown_outcome.CopyFrom(ack)
        unknown_outcome.outcome = 999
        cases.append(("unknown outcome", unknown_outcome))
        outside_install_window = self.execution.ScoutBundleAck()
        outside_install_window.CopyFrom(ack)
        outside_install_window.installed_at_monotonic_ns = 1_300
        cases.append(("outside install window", outside_install_window))
        late_ack = self.execution.ScoutBundleAck()
        late_ack.CopyFrom(ack)
        late_ack.header.generated_at_monotonic_ns = 100_001_501
        cases.append(("ACK timeout", late_ack))

        for name, invalid_ack in cases:
            with self.subTest(name=name):
                with self.assertRaises(ValueError):
                    validate_scout_bundle_ack(
                        invalid_ack,
                        installed_bundle=bundle,
                        now_monotonic_ns=max(
                            1_600,
                            invalid_ack.header.generated_at_monotonic_ns,
                        ),
                        local_clock_domain_id="scout-nuc/boot-9",
                        accepted_manifest_identity=self.accepted_manifest_identity,
                    )

    def test_install_watermark_rejects_conflict_reorder_revoked_expired_and_old_session(self) -> None:
        bundle = self._valid_bundle()
        state = ScoutBundleWatermarkState(
            producer_session_id=b"s" * 16,
            delivery_sequence=8,
            bundle_sequence=8,
            plan_sequence=40,
            lease_sequence=6,
            bundle_identity=b"a" * 32,
            revoked_lease_sequences={5},
        )
        self.assertEqual(
            apply_scout_bundle_watermark(
                state, bundle, now_monotonic_ns=1_500, recovery_boundary=False
            ),
            "installed",
        )
        self.assertEqual(
            apply_scout_bundle_watermark(
                state, bundle, now_monotonic_ns=1_500, recovery_boundary=False
            ),
            "idempotent duplicate",
        )

        conflict = self.execution.ScoutAuthorizedExecutionBundle()
        conflict.CopyFrom(bundle)
        conflict.bundle_content_identity.sha256 = b"c" * 32
        with self.assertRaisesRegex(ValueError, "INPUT_INVALID"):
            apply_scout_bundle_watermark(
                state, conflict, now_monotonic_ns=1_500, recovery_boundary=False
            )

        plan_rollback = self._valid_bundle()
        plan_rollback.header.sequence = 10
        plan_rollback.bundle_sequence = 10
        plan_rollback.plan.plan_sequence = 40
        plan_rollback.plan.plan_content_identity.sha256 = canonical_business_identity(
            plan_rollback.plan, "plan_content_identity"
        )
        plan_rollback.lease.plan_sequence = 40
        plan_rollback.lease.plan_content_identity.CopyFrom(
            plan_rollback.plan.plan_content_identity
        )
        plan_rollback.lease.lease_sequence = 8
        plan_rollback.lease.content_identity.sha256 = canonical_business_identity(
            plan_rollback.lease, "content_identity"
        )
        plan_rollback.bundle_content_identity.sha256 = canonical_business_identity(
            plan_rollback, "bundle_content_identity"
        )
        rollback_state = replace(
            state,
            retired_sessions=set(state.retired_sessions),
            revoked_lease_sequences=set(state.revoked_lease_sequences),
            expired_lease_sequences=set(state.expired_lease_sequences),
        )
        with self.assertRaisesRegex(ValueError, "SEQUENCE_REJECTED"):
            apply_scout_bundle_watermark(
                rollback_state,
                plan_rollback,
                now_monotonic_ns=1_500,
                recovery_boundary=False,
            )

        reordered = self._valid_bundle()
        reordered.header.sequence = 7
        reordered.bundle_sequence = 7
        reordered.lease.lease_sequence = 4
        with self.assertRaisesRegex(ValueError, "SEQUENCE_REJECTED"):
            apply_scout_bundle_watermark(
                state, reordered, now_monotonic_ns=1_500, recovery_boundary=False
            )

        revoked = self._valid_bundle()
        state.revoked_lease_sequences.add(revoked.lease.lease_sequence)
        with self.assertRaisesRegex(ValueError, "REVOKED"):
            apply_scout_bundle_watermark(
                state, revoked, now_monotonic_ns=1_500, recovery_boundary=False
            )
        state.revoked_lease_sequences.remove(revoked.lease.lease_sequence)

        with self.assertRaisesRegex(ValueError, "EXPIRED"):
            apply_scout_bundle_watermark(
                state, bundle, now_monotonic_ns=3_000, recovery_boundary=False
            )

        expiry_state = ScoutBundleWatermarkState(
            producer_session_id=b"s" * 16,
            delivery_sequence=8,
            bundle_sequence=8,
            plan_sequence=40,
            lease_sequence=6,
            bundle_identity=b"a" * 32,
        )
        expired_first = self._valid_bundle()
        with self.assertRaisesRegex(ValueError, "EXPIRED"):
            apply_scout_bundle_watermark(
                expiry_state,
                expired_first,
                now_monotonic_ns=3_000,
                recovery_boundary=False,
            )
        freshened_same_lease = self._valid_bundle()
        freshened_same_lease.lease.expires_at_monotonic_ns = 4_000
        freshened_same_lease.lease.content_identity.sha256 = canonical_business_identity(
            freshened_same_lease.lease,
            "content_identity",
        )
        freshened_same_lease.bundle_content_identity.sha256 = canonical_business_identity(
            freshened_same_lease,
            "bundle_content_identity",
        )
        with self.assertRaisesRegex(ValueError, "EXPIRED"):
            apply_scout_bundle_watermark(
                expiry_state,
                freshened_same_lease,
                now_monotonic_ns=3_100,
                recovery_boundary=False,
            )

        restarted = self._valid_bundle()
        restarted.header.producer_session_id = b"n" * 16
        restarted.header.sequence = 1
        restarted.bundle_sequence = 1
        # Numeric lease watermarks are scoped to the producer session. The old
        # session has retired/expired sequence 7, but a recovered new session
        # may start with that same numeric value.
        restarted.lease.lease_sequence = 7
        restarted.lease.content_identity.sha256 = canonical_business_identity(
            restarted.lease, "content_identity"
        )
        restarted.bundle_content_identity.sha256 = canonical_business_identity(
            restarted, "bundle_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "SEQUENCE_REJECTED"):
            apply_scout_bundle_watermark(
                state, restarted, now_monotonic_ns=1_500, recovery_boundary=False
            )
        self.assertEqual(
            apply_scout_bundle_watermark(
                state, restarted, now_monotonic_ns=1_500, recovery_boundary=True
            ),
            "new session installed",
        )

    def test_manifest_profile_hashing_and_normative_rules_are_exact_and_bounded(self) -> None:
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
        contract = CONTRACT_PATH.read_text(encoding="utf-8")
        hashing = HASHING_PATH.read_text(encoding="utf-8")

        self.assertIn("scout_authorization_bundle_v1", manifest["supported_features"])
        self.assertIn(
            "independent_execution_authority_domains_v1",
            manifest["supported_features"],
        )
        timing = profile["timing"]
        self.assertLess(
            timing["scout_authorized_bundle_ack_timeout_ns"],
            timing["scout_lease_duration_ns"],
        )
        self.assertLess(
            timing["scout_lease_renewal_margin_ns"],
            timing["scout_lease_duration_ns"],
        )
        self.assertLess(
            timing["scout_authorization_start_window_ns"],
            timing["scout_lease_duration_ns"],
        )
        self.assertGreater(
            profile["interface_limits"]["maximum_scout_authorized_bundle_bytes"],
            0,
        )
        for message_name in (
            "ScoutExecutionLease",
            "ScoutAuthorizedExecutionBundle",
            "ScoutBundleAck",
        ):
            self.assertIn(message_name, hashing)
        contract_lower = contract.lower()
        for rule in (
            "only execution grant",
            "MUST NOT be shifted",
            "same bundle_sequence",
            "revoked or expired",
            "wrong clock domain",
            "main-laying",
            "unknown field",
        ):
            self.assertIn(rule.lower(), contract_lower)

    def test_unknown_wire_field_is_rejected_before_bundle_identity_installation(self) -> None:
        bundle = self._valid_bundle()
        wire = bundle.SerializeToString(deterministic=True)
        unknown_wire = wire + _encode_varint((127 << 3) | 0) + _encode_varint(1)
        parsed = self.execution.ScoutAuthorizedExecutionBundle.FromString(unknown_wire)
        with self.assertRaisesRegex(ValueError, "INPUT_INVALID: unknown field"):
            validate_scout_bundle(
                parsed,
                now_monotonic_ns=1_500,
                local_clock_domain_id="scout-nuc/boot-9",
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_bundle_bytes=65_536,
            )

    def test_bundle_hash_normalizes_nfc_configuration_references(self) -> None:
        decomposed = self._valid_bundle()
        composed = self._valid_bundle()
        decomposed.safety_gate_configuration.profile_id = "safe\u0301ty/v1"
        composed.safety_gate_configuration.profile_id = unicodedata.normalize(
            "NFC", decomposed.safety_gate_configuration.profile_id
        )
        self.assertNotEqual(
            decomposed.safety_gate_configuration.profile_id,
            composed.safety_gate_configuration.profile_id,
        )
        self.assertEqual(
            canonical_business_identity(decomposed, "bundle_content_identity"),
            canonical_business_identity(composed, "bundle_content_identity"),
        )
        decomposed.bundle_content_identity.sha256 = canonical_business_identity(
            decomposed, "bundle_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "INPUT_INVALID"):
            validate_scout_bundle(
                decomposed,
                now_monotonic_ns=1_500,
                local_clock_domain_id="scout-nuc/boot-9",
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_bundle_bytes=65_536,
            )

    def test_impossible_local_timestamp_order_is_rejected(self) -> None:
        bundle = self._valid_bundle()
        bundle.lease.validated_at_monotonic_ns = 900
        bundle.lease.content_identity.sha256 = canonical_business_identity(
            bundle.lease, "content_identity"
        )
        bundle.bundle_content_identity.sha256 = canonical_business_identity(
            bundle, "bundle_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "SEQUENCE_REJECTED"):
            validate_scout_bundle(
                bundle,
                now_monotonic_ns=1_500,
                local_clock_domain_id="scout-nuc/boot-9",
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_bundle_bytes=65_536,
            )


if __name__ == "__main__":
    unittest.main()
