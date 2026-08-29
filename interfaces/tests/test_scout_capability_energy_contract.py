"""Public-contract tests for Scout capability, thruster health, and energy inputs."""

from __future__ import annotations

import hashlib
import hmac
import importlib
import json
import math
import pathlib
import subprocess
import sys
import tempfile
import unittest
import unicodedata

from google.protobuf.descriptor import FieldDescriptor


INTERFACES = pathlib.Path(__file__).resolve().parents[1]
PROTO_ROOT = INTERFACES / "proto"
PROTO_V1 = INTERFACES / "proto" / "underwater" / "contracts" / "v1"
PROFILE_PATH = INTERFACES / "profiles" / "integration-v1.json"
MANIFEST_PATH = INTERFACES / "compatibility" / "contract-manifest-v1.json"
CONTRACT_PATH = INTERFACES / "SCOUT_CAPABILITY_AND_ENERGY.md"
HASHING_PATH = INTERFACES / "HASHING.md"
REGISTRY_PATH = INTERFACES / "registry" / "codes-v1.json"
REGISTERED_FAULT_CODES = {
    entry["code"]
    for entry in json.loads(REGISTRY_PATH.read_text(encoding="utf-8"))["fault_codes"]
}


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
    canonical.ClearField("header")
    canonical.ClearField(identity_field)
    _normalize_canonical_message(canonical)
    return hashlib.sha256(canonical.SerializeToString(deterministic=True)).digest()


def exact_delivery_identity(message: object) -> bytes:
    return hashlib.sha256(message.SerializeToString(deterministic=True)).digest()


def apply_versioned_stream_watermark(
    *,
    current_session: bytes,
    current_sequence: int,
    current_version: int,
    current_identity: bytes,
    current_delivery_identity: bytes,
    incoming_session: bytes,
    incoming_sequence: int,
    incoming_version: int,
    incoming_identity: bytes,
    incoming_delivery_identity: bytes,
    retired_sessions: set[bytes],
) -> str:
    if (
        current_sequence == 0
        or incoming_sequence == 0
        or current_version == 0
        or incoming_version == 0
        or len(current_identity) != 32
        or len(incoming_identity) != 32
        or len(current_delivery_identity) != 32
        or len(incoming_delivery_identity) != 32
    ):
        raise ValueError("SEQUENCE_REJECTED: invalid stream watermark")
    if incoming_session != current_session:
        if incoming_session in retired_sessions:
            raise ValueError("SEQUENCE_REJECTED: retired producer session")
        if incoming_version < current_version:
            raise ValueError("VERSION_INCOMPATIBLE: cross-session business version rollback")
        if incoming_version == current_version and not hmac.compare_digest(
            incoming_identity, current_identity
        ):
            raise ValueError("INPUT_INVALID: cross-session version identity conflict")
        retired_sessions.add(current_session)
        return "new session"
    if incoming_sequence < current_sequence:
        raise ValueError("SEQUENCE_REJECTED: message reorder")
    if incoming_sequence == current_sequence:
        if (
            incoming_version == current_version
            and hmac.compare_digest(incoming_identity, current_identity)
            and hmac.compare_digest(
                incoming_delivery_identity, current_delivery_identity
            )
        ):
            return "idempotent duplicate"
        raise ValueError("INPUT_INVALID: sequence identity conflict")
    if incoming_version < current_version:
        raise ValueError("VERSION_INCOMPATIBLE: business version rollback")
    if incoming_version == current_version:
        raise ValueError("INPUT_INVALID: new delivery reused business version")
    return "accepted"


def _has_unknown_fields(message: object) -> bool:
    known_only = type(message)()
    known_only.CopyFrom(message)
    known_only.DiscardUnknownFields()
    return known_only.SerializeToString(deterministic=True) != message.SerializeToString(
        deterministic=True
    )


def _message_strings(message: object) -> list[str]:
    strings: list[str] = []
    for field, value in message.ListFields():
        if field.label == FieldDescriptor.LABEL_REPEATED:
            if field.type == FieldDescriptor.TYPE_MESSAGE:
                for item in value:
                    strings.extend(_message_strings(item))
            elif field.type == FieldDescriptor.TYPE_STRING:
                strings.extend(value)
        elif field.type == FieldDescriptor.TYPE_MESSAGE:
            strings.extend(_message_strings(value))
        elif field.type == FieldDescriptor.TYPE_STRING:
            strings.append(value)
    return strings


def _all_strings_are_nfc(message: object) -> bool:
    return all(
        value == unicodedata.normalize("NFC", value)
        for value in _message_strings(message)
    )


def _finite_present(message: object, fields: tuple[str, ...]) -> bool:
    return all(message.HasField(field) and math.isfinite(getattr(message, field)) for field in fields)


def _validate_bound_profile(
    profile: dict[str, object],
    profile_artifact_bytes: bytes,
    manifest: dict[str, object],
) -> None:
    manifest_profile = manifest["integration_profile"]
    if (
        hashlib.sha256(profile_artifact_bytes).hexdigest() != manifest_profile["sha256"]
        or json.loads(profile_artifact_bytes.decode("utf-8")) != profile
        or profile["profile_id"] != manifest_profile["id"]
        or profile["version"] != manifest_profile["version"]
        or profile["production"] != manifest_profile["production"]
    ):
        raise ValueError("profile is not the exact Manifest-bound artifact")


def _valid_header(
    header: object,
    common: object,
    *,
    stream_id: int,
    clock_domain_id: str,
    manifest: dict[str, object],
    accepted_manifest_identity: bytes,
) -> bool:
    return (
        header.schema_major == manifest["schema_major"]
        and header.schema_minor == manifest["schema_minor"]
        and header.stream_id == stream_id
        and bool(header.producer_id)
        and len(header.producer_session_id) == 16
        and header.sequence > 0
        and header.source_clock_domain_id == clock_domain_id
        and header.HasField("generated_at_monotonic_ns")
        and header.HasField("observed_at")
        and header.observed_at.HasField("utc_time_ns")
        and header.observed_at.HasField("uncertainty_ns")
        and header.observed_at.status
        in {
            common.TIME_SYNC_UNSYNCHRONIZED,
            common.TIME_SYNC_SYNCHRONIZED,
            common.TIME_SYNC_DEGRADED,
        }
        and header.HasField("manifest")
        and header.manifest.schema_major == manifest["schema_major"]
        and header.manifest.schema_minor == manifest["schema_minor"]
        and hmac.compare_digest(
            header.manifest.manifest_identity.sha256,
            accepted_manifest_identity,
        )
        and "scout_capability_and_energy_v1" in manifest["supported_features"]
    )


def validate_capability_profile(
    capability_profile: object,
    capability: object,
    common: object,
    *,
    clock_domain_id: str,
    manifest: dict[str, object],
    accepted_manifest_identity: bytes,
    profile: dict[str, object],
    profile_artifact_bytes: bytes,
    production_planning: bool,
) -> None:
    _validate_bound_profile(profile, profile_artifact_bytes, manifest)
    limits = profile["interface_limits"]
    if _has_unknown_fields(capability_profile):
        raise ValueError("unknown field in capability profile")
    if not _all_strings_are_nfc(capability_profile):
        raise ValueError("non-NFC string in capability profile")
    if (
        len(capability_profile.SerializeToString(deterministic=True))
        > limits["maximum_capability_profile_bytes"]
        or any(
            len(value.encode("utf-8")) > limits["maximum_string_bytes"]
            for value in _message_strings(capability_profile)
        )
    ):
        raise ValueError("RESOURCE_LIMIT_EXCEEDED: capability profile")
    if not _valid_header(
        capability_profile.header,
        common,
        stream_id=common.STREAM_SCOUT_CAPABILITY_PROFILE,
        clock_domain_id=clock_domain_id,
        manifest=manifest,
        accepted_manifest_identity=accepted_manifest_identity,
    ):
        raise ValueError("invalid capability header, clock, or manifest")
    if (
        not capability_profile.capability_profile_id
        or capability_profile.capability_profile_version == 0
        or capability_profile.health_profile
        not in {
            capability.SCOUT_THRUSTER_HEALTH_NOMINAL,
            capability.SCOUT_THRUSTER_HEALTH_DEGRADED_A,
            capability.SCOUT_THRUSTER_HEALTH_DEGRADED_B,
        }
        or capability_profile.operating_domain_id != profile["operating_domain_id"]
        or not capability_profile.vehicle_id
        or not capability_profile.device_serial_number
        or not capability_profile.calibration_dataset_id
        or not capability_profile.calibration_method_version
        or not capability_profile.thruster_configuration_id
        or not capability_profile.calibrated_thruster_states
        or len(capability_profile.calibrated_thruster_states)
        > limits["maximum_thrusters_per_vehicle"]
        or not capability_profile.HasField("calibrated_at_utc_ns")
        or capability_profile.calibrated_at_utc_ns <= 0
        or not capability_profile.HasField("production_approved")
    ):
        raise ValueError("uncalibrated or unsupported capability profile")
    if production_planning and (
        not profile["production"] or not capability_profile.production_approved
    ):
        raise ValueError("NON_PRODUCTION capability profile")

    calibrated_pairs = [
        (state.thruster_id, state.health)
        for state in capability_profile.calibrated_thruster_states
    ]
    calibrated_ids = [pair[0] for pair in calibrated_pairs]
    allowed_unit_health = {
        capability.SCOUT_THRUSTER_UNIT_HEALTH_NOMINAL,
        capability.SCOUT_THRUSTER_UNIT_HEALTH_DEGRADED,
        capability.SCOUT_THRUSTER_UNIT_HEALTH_FAILED,
    }
    if (
        any(not thruster_id for thruster_id in calibrated_ids)
        or calibrated_ids != sorted(set(calibrated_ids))
        or any(state not in allowed_unit_health for _, state in calibrated_pairs)
        or (
            capability_profile.health_profile
            == capability.SCOUT_THRUSTER_HEALTH_NOMINAL
            and any(
                state != capability.SCOUT_THRUSTER_UNIT_HEALTH_NOMINAL
                for _, state in calibrated_pairs
            )
        )
        or (
            capability_profile.health_profile
            in {
                capability.SCOUT_THRUSTER_HEALTH_DEGRADED_A,
                capability.SCOUT_THRUSTER_HEALTH_DEGRADED_B,
            }
            and all(
                state == capability.SCOUT_THRUSTER_UNIT_HEALTH_NOMINAL
                for _, state in calibrated_pairs
            )
        )
    ):
        raise ValueError("invalid calibrated thruster health combination")

    motion = capability_profile.motion_envelope
    motion_fields = (
        "maximum_water_relative_speed_mps",
        "maximum_translational_acceleration_mps2",
        "maximum_vertical_ascent_speed_mps",
        "maximum_vertical_descent_speed_mps",
        "maximum_vertical_acceleration_mps2",
        "maximum_yaw_rate_radps",
        "maximum_yaw_acceleration_radps2",
        "maximum_absolute_roll_rad",
        "maximum_absolute_pitch_rad",
        "minimum_translational_thrust_margin_fraction",
        "minimum_rotational_thrust_margin_fraction",
    )
    if (
        not _finite_present(motion, motion_fields)
        or any(getattr(motion, field) <= 0.0 for field in motion_fields[:9])
        or any(not 0.0 < getattr(motion, field) < 1.0 for field in motion_fields[9:])
        or motion.maximum_absolute_roll_rad >= math.pi / 2.0
        or motion.maximum_absolute_pitch_rad >= math.pi / 2.0
        or not motion.thruster_allocator_version
    ):
        raise ValueError("invalid motion capability envelope")

    braking = capability_profile.braking_envelope
    if (
        not _finite_present(
            braking,
            (
                "command_latency_s",
                "minimum_translational_deceleration_mps2",
                "minimum_yaw_deceleration_radps2",
                "stopping_distance_margin_m",
            ),
        )
        or braking.command_latency_s < 0.0
        or braking.minimum_translational_deceleration_mps2 <= 0.0
        or braking.minimum_yaw_deceleration_radps2 <= 0.0
        or braking.stopping_distance_margin_m < 0.0
        or not braking.HasField("requires_active_thrust")
        or not braking.requires_active_thrust
    ):
        raise ValueError("zero thrust is not calibrated braking capability")

    operating = capability_profile.operating_envelope
    operating_fields = (
        "minimum_depth_m",
        "maximum_depth_m",
        "maximum_current_speed_mps",
        "minimum_bus_voltage_v",
        "maximum_bus_voltage_v",
        "minimum_water_density_kgpm3",
        "maximum_water_density_kgpm3",
    )
    if (
        not _finite_present(operating, operating_fields)
        or operating.minimum_depth_m < 0.0
        or operating.maximum_depth_m <= operating.minimum_depth_m
        or operating.maximum_current_speed_mps < 0.0
        or operating.minimum_bus_voltage_v <= 0.0
        or operating.maximum_bus_voltage_v <= operating.minimum_bus_voltage_v
        or operating.minimum_water_density_kgpm3 <= 0.0
        or operating.maximum_water_density_kgpm3
        <= operating.minimum_water_density_kgpm3
    ):
        raise ValueError("invalid capability operating envelope")
    if (
        len(capability_profile.capability_content_identity.sha256) != 32
        or not hmac.compare_digest(
            capability_profile.capability_content_identity.sha256,
            canonical_business_identity(
                capability_profile, "capability_content_identity"
            ),
        )
    ):
        raise ValueError("capability content identity mismatch")


def capability_contains_conditions(
    capability_profile: object,
    *,
    depth_m: float,
    current_speed_mps: float,
    bus_voltage_v: float,
    water_density_kgpm3: float,
) -> bool:
    if not all(
        math.isfinite(value)
        for value in (depth_m, current_speed_mps, bus_voltage_v, water_density_kgpm3)
    ):
        return False
    operating = capability_profile.operating_envelope
    return (
        operating.minimum_depth_m <= depth_m <= operating.maximum_depth_m
        and 0.0 <= current_speed_mps <= operating.maximum_current_speed_mps
        and operating.minimum_bus_voltage_v
        <= bus_voltage_v
        <= operating.maximum_bus_voltage_v
        and operating.minimum_water_density_kgpm3
        <= water_density_kgpm3
        <= operating.maximum_water_density_kgpm3
    )


def _profile_ref_matches(reference: object, *, profile_id: str, version: int, identity: bytes) -> bool:
    return (
        reference.profile_id == profile_id
        and reference.version == version
        and len(reference.content_identity.sha256) == 32
        and hmac.compare_digest(reference.content_identity.sha256, identity)
    )


def validate_thruster_health(
    health: object,
    capability_profile: object,
    capability: object,
    common: object,
    *,
    clock_domain_id: str,
    now_monotonic_ns: int,
    manifest: dict[str, object],
    accepted_manifest_identity: bytes,
    profile: dict[str, object],
    profile_artifact_bytes: bytes,
) -> None:
    _validate_bound_profile(profile, profile_artifact_bytes, manifest)
    limits = profile["interface_limits"]
    if _has_unknown_fields(health):
        raise ValueError("unknown field in thruster health")
    if not _all_strings_are_nfc(health):
        raise ValueError("non-NFC string in thruster health")
    if (
        len(health.SerializeToString(deterministic=True))
        > limits["maximum_thruster_health_bytes"]
        or len(health.thrusters) > limits["maximum_thrusters_per_vehicle"]
        or len(health.active_fault_codes) > limits["maximum_diagnostics"]
        or any(
            len(value.encode("utf-8")) > limits["maximum_string_bytes"]
            for value in _message_strings(health)
        )
    ):
        raise ValueError("RESOURCE_LIMIT_EXCEEDED: thruster health")
    if not _valid_header(
        health.header,
        common,
        stream_id=common.STREAM_SCOUT_THRUSTER_HEALTH,
        clock_domain_id=clock_domain_id,
        manifest=manifest,
        accepted_manifest_identity=accepted_manifest_identity,
    ):
        raise ValueError("invalid thruster health header, clock, or manifest")
    if (
        not health.vehicle_id
        or health.vehicle_id != capability_profile.vehicle_id
        or health.health_version == 0
        or not health.HasField("observed_at_monotonic_ns")
        or not health.HasField("valid_until_monotonic_ns")
        or health.observed_at_monotonic_ns < 0
        or health.valid_until_monotonic_ns < health.observed_at_monotonic_ns
        or now_monotonic_ns < health.observed_at_monotonic_ns
        or now_monotonic_ns > health.valid_until_monotonic_ns
        or now_monotonic_ns - health.observed_at_monotonic_ns
        > profile["timing"]["scout_thruster_health_reject_ns"]
    ):
        raise ValueError("stale or invalid thruster health")
    if (
        health.health_profile != capability_profile.health_profile
        or not _profile_ref_matches(
            health.active_capability_profile,
            profile_id=capability_profile.capability_profile_id,
            version=capability_profile.capability_profile_version,
            identity=capability_profile.capability_content_identity.sha256,
        )
    ):
        raise ValueError("thruster health does not reference the exact active capability profile")
    if not health.thrusters:
        raise ValueError("thruster health set is empty")
    thruster_ids = [thruster.thruster_id for thruster in health.thrusters]
    allowed_unit_health = {
        capability.SCOUT_THRUSTER_UNIT_HEALTH_NOMINAL,
        capability.SCOUT_THRUSTER_UNIT_HEALTH_DEGRADED,
        capability.SCOUT_THRUSTER_UNIT_HEALTH_FAILED,
    }
    if (
        any(not thruster_id for thruster_id in thruster_ids)
        or thruster_ids != sorted(set(thruster_ids))
        or any(thruster.health not in allowed_unit_health for thruster in health.thrusters)
        or list(health.active_fault_codes) != sorted(set(health.active_fault_codes))
        or any(code not in REGISTERED_FAULT_CODES for code in health.active_fault_codes)
    ):
        raise ValueError("invalid thruster identities, health, or active fault codes")
    calibrated_states = capability_profile.calibrated_thruster_states
    calibrated_pairs = [
        (state.thruster_id, state.health) for state in calibrated_states
    ]
    if thruster_ids != [state.thruster_id for state in calibrated_states]:
        raise ValueError("health does not contain the exact calibrated thruster set")
    if list(zip(thruster_ids, (state.health for state in health.thrusters))) != calibrated_pairs:
        raise ValueError("health does not match the exact calibrated health combination")
    degraded_units = any(
        thruster.health != capability.SCOUT_THRUSTER_UNIT_HEALTH_NOMINAL
        for thruster in health.thrusters
    )
    if health.health_profile == capability.SCOUT_THRUSTER_HEALTH_NOMINAL and (
        degraded_units or health.active_fault_codes
    ):
        raise ValueError("nominal health cannot contain degraded thrusters or faults")
    if health.health_profile in {
        capability.SCOUT_THRUSTER_HEALTH_DEGRADED_A,
        capability.SCOUT_THRUSTER_HEALTH_DEGRADED_B,
    } and not degraded_units:
        raise ValueError("degraded profile requires degraded thruster evidence")
    if (
        len(health.health_content_identity.sha256) != 32
        or not hmac.compare_digest(
            health.health_content_identity.sha256,
            canonical_business_identity(health, "health_content_identity"),
        )
    ):
        raise ValueError("thruster health content identity mismatch")


def validate_capability_context(
    capability_profile: object,
    health: object,
    capability: object,
    common: object,
    *,
    clock_domain_id: str,
    now_monotonic_ns: int,
    manifest: dict[str, object],
    accepted_manifest_identity: bytes,
    profile: dict[str, object],
    profile_artifact_bytes: bytes,
    previous_dependencies: tuple[str, int, bytes, int, bytes] | None = None,
) -> tuple[tuple[str, int, bytes, int, bytes], bool]:
    validate_capability_profile(
        capability_profile,
        capability,
        common,
        clock_domain_id=clock_domain_id,
        manifest=manifest,
        accepted_manifest_identity=accepted_manifest_identity,
        profile=profile,
        profile_artifact_bytes=profile_artifact_bytes,
        production_planning=False,
    )
    validate_thruster_health(
        health,
        capability_profile,
        capability,
        common,
        clock_domain_id=clock_domain_id,
        now_monotonic_ns=now_monotonic_ns,
        manifest=manifest,
        accepted_manifest_identity=accepted_manifest_identity,
        profile=profile,
        profile_artifact_bytes=profile_artifact_bytes,
    )
    incoming = (
        capability_profile.capability_profile_id,
        capability_profile.capability_profile_version,
        capability_profile.capability_content_identity.sha256,
        health.health_version,
        health.health_content_identity.sha256,
    )
    if previous_dependencies is None:
        return incoming, False
    if (
        incoming[0] == previous_dependencies[0]
        and incoming[1] < previous_dependencies[1]
    ) or incoming[3] < previous_dependencies[3]:
        raise ValueError("VERSION_INCOMPATIBLE: capability dependency rollback")
    if (
        incoming[0] == previous_dependencies[0]
        and incoming[1] == previous_dependencies[1]
        and not hmac.compare_digest(incoming[2], previous_dependencies[2])
    ) or (
        incoming[3] == previous_dependencies[3]
        and not hmac.compare_digest(incoming[4], previous_dependencies[4])
    ):
        raise ValueError("INPUT_INVALID: capability dependency identity conflict")
    return incoming, incoming != previous_dependencies


def validate_energy_model(
    energy_model: object,
    capability_profile: object,
    capability: object,
    common: object,
    *,
    clock_domain_id: str,
    manifest: dict[str, object],
    accepted_manifest_identity: bytes,
    profile: dict[str, object],
    profile_artifact_bytes: bytes,
    production_planning: bool,
) -> None:
    _validate_bound_profile(profile, profile_artifact_bytes, manifest)
    limits = profile["interface_limits"]
    if _has_unknown_fields(energy_model):
        raise ValueError("unknown field in energy model")
    if not _all_strings_are_nfc(energy_model):
        raise ValueError("non-NFC string in energy model")
    if (
        len(energy_model.SerializeToString(deterministic=True))
        > limits["maximum_energy_model_bytes"]
        or any(
            len(value.encode("utf-8")) > limits["maximum_string_bytes"]
            for value in _message_strings(energy_model)
        )
    ):
        raise ValueError("RESOURCE_LIMIT_EXCEEDED: energy model")
    if not _valid_header(
        energy_model.header,
        common,
        stream_id=common.STREAM_SCOUT_ENERGY_MODEL,
        clock_domain_id=clock_domain_id,
        manifest=manifest,
        accepted_manifest_identity=accepted_manifest_identity,
    ):
        raise ValueError("invalid energy model header, clock, or manifest")
    if (
        not energy_model.energy_model_id
        or energy_model.energy_model_version == 0
        or energy_model.health_profile != capability_profile.health_profile
        or not _profile_ref_matches(
            energy_model.capability_profile,
            profile_id=capability_profile.capability_profile_id,
            version=capability_profile.capability_profile_version,
            identity=capability_profile.capability_content_identity.sha256,
        )
        or energy_model.operating_domain_id != profile["operating_domain_id"]
        or energy_model.vehicle_id != capability_profile.vehicle_id
        or not energy_model.device_serial_number
        or not energy_model.calibration_dataset_id
        or not energy_model.calibration_method_version
        or not energy_model.HasField("calibrated_at_utc_ns")
        or energy_model.calibrated_at_utc_ns <= 0
        or not energy_model.HasField("production_approved")
    ):
        raise ValueError("uncalibrated, mismatched, or unsupported energy model")
    if production_planning and (
        not profile["production"] or not energy_model.production_approved
    ):
        raise ValueError("NON_PRODUCTION energy model")
    power = energy_model.power_model
    power_fields = (
        "hotel_power_w",
        "speed_linear_w_per_mps",
        "speed_cubic_w_per_mps3",
        "acceleration_w_per_mps2",
        "yaw_rate_w_per_radps",
        "model_error_upper_bound_w",
    )
    if (
        not _finite_present(power, power_fields)
        or power.hotel_power_w <= 0.0
        or any(getattr(power, field) < 0.0 for field in power_fields[1:])
    ):
        raise ValueError("invalid or non-finite conservative power model")
    if (
        len(energy_model.energy_model_content_identity.sha256) != 32
        or not hmac.compare_digest(
            energy_model.energy_model_content_identity.sha256,
            canonical_business_identity(
                energy_model, "energy_model_content_identity"
            ),
        )
    ):
        raise ValueError("energy model content identity mismatch")


def validate_energy_state(
    energy_state: object,
    energy_model: object,
    capability: object,
    common: object,
    *,
    clock_domain_id: str,
    now_monotonic_ns: int,
    manifest: dict[str, object],
    accepted_manifest_identity: bytes,
    profile: dict[str, object],
    profile_artifact_bytes: bytes,
) -> None:
    _validate_bound_profile(profile, profile_artifact_bytes, manifest)
    limits = profile["interface_limits"]
    if _has_unknown_fields(energy_state):
        raise ValueError("unknown field in energy state")
    if not _all_strings_are_nfc(energy_state):
        raise ValueError("non-NFC string in energy state")
    if (
        len(energy_state.SerializeToString(deterministic=True))
        > limits["maximum_energy_state_bytes"]
        or any(
            len(value.encode("utf-8")) > limits["maximum_string_bytes"]
            for value in _message_strings(energy_state)
        )
    ):
        raise ValueError("RESOURCE_LIMIT_EXCEEDED: energy state")
    if not _valid_header(
        energy_state.header,
        common,
        stream_id=common.STREAM_SCOUT_ENERGY_STATE,
        clock_domain_id=clock_domain_id,
        manifest=manifest,
        accepted_manifest_identity=accepted_manifest_identity,
    ):
        raise ValueError("invalid energy state header, clock, or manifest")
    energy_fields = (
        "available_energy_j",
        "reserve_energy_j",
        "required_return_energy_j",
        "required_risk_action_energy_j",
    )
    if (
        not energy_state.vehicle_id
        or energy_state.vehicle_id != energy_model.vehicle_id
        or not energy_state.energy_store_id
        or energy_state.energy_state_version == 0
        or not energy_state.HasField("observed_at_monotonic_ns")
        or not energy_state.HasField("valid_until_monotonic_ns")
        or energy_state.observed_at_monotonic_ns < 0
        or energy_state.valid_until_monotonic_ns
        < energy_state.observed_at_monotonic_ns
        or now_monotonic_ns < energy_state.observed_at_monotonic_ns
        or now_monotonic_ns > energy_state.valid_until_monotonic_ns
        or now_monotonic_ns - energy_state.observed_at_monotonic_ns
        > profile["timing"]["scout_energy_state_reject_ns"]
        or not _profile_ref_matches(
            energy_state.energy_model,
            profile_id=energy_model.energy_model_id,
            version=energy_model.energy_model_version,
            identity=energy_model.energy_model_content_identity.sha256,
        )
        or not _finite_present(energy_state, energy_fields)
        or energy_state.available_energy_j <= 0.0
        or any(getattr(energy_state, field) < 0.0 for field in energy_fields[1:])
        or energy_state.contingency_requirement
        not in {
            capability.SCOUT_ENERGY_RETURN_REQUIRED,
            capability.SCOUT_ENERGY_RISK_ACTION_REQUIRED,
        }
        or energy_state.operating_domain_id != profile["operating_domain_id"]
    ):
        raise ValueError("stale, incomplete, non-finite, or mismatched energy state")
    if (
        len(energy_state.energy_state_content_identity.sha256) != 32
        or not hmac.compare_digest(
            energy_state.energy_state_content_identity.sha256,
            canonical_business_identity(
                energy_state, "energy_state_content_identity"
            ),
        )
    ):
        raise ValueError("energy state content identity mismatch")


def validate_energy_context(
    capability_profile: object,
    energy_model: object,
    energy_state: object,
    capability: object,
    common: object,
    *,
    clock_domain_id: str,
    now_monotonic_ns: int,
    manifest: dict[str, object],
    accepted_manifest_identity: bytes,
    profile: dict[str, object],
    profile_artifact_bytes: bytes,
    production_planning: bool,
) -> None:
    validate_capability_profile(
        capability_profile,
        capability,
        common,
        clock_domain_id=clock_domain_id,
        manifest=manifest,
        accepted_manifest_identity=accepted_manifest_identity,
        profile=profile,
        profile_artifact_bytes=profile_artifact_bytes,
        production_planning=production_planning,
    )
    validate_energy_model(
        energy_model,
        capability_profile,
        capability,
        common,
        clock_domain_id=clock_domain_id,
        manifest=manifest,
        accepted_manifest_identity=accepted_manifest_identity,
        profile=profile,
        profile_artifact_bytes=profile_artifact_bytes,
        production_planning=production_planning,
    )
    validate_energy_state(
        energy_state,
        energy_model,
        capability,
        common,
        clock_domain_id=clock_domain_id,
        now_monotonic_ns=now_monotonic_ns,
        manifest=manifest,
        accepted_manifest_identity=accepted_manifest_identity,
        profile=profile,
        profile_artifact_bytes=profile_artifact_bytes,
    )


def energy_budget_accepts(
    planned_energy_j: float, energy_state: object, capability: object
) -> tuple[bool, float]:
    if not math.isfinite(planned_energy_j) or planned_energy_j < 0.0:
        raise ValueError("non-finite or negative planned energy")
    if energy_state.contingency_requirement == capability.SCOUT_ENERGY_RETURN_REQUIRED:
        contingency_energy_j = energy_state.required_return_energy_j
    elif (
        energy_state.contingency_requirement
        == capability.SCOUT_ENERGY_RISK_ACTION_REQUIRED
    ):
        contingency_energy_j = energy_state.required_risk_action_energy_j
    else:
        raise ValueError("unknown safety-critical energy contingency requirement")
    required_energy_j = (
        planned_energy_j + contingency_energy_j + energy_state.reserve_energy_j
    )
    return energy_state.available_energy_j >= required_energy_j, required_energy_j


def conservative_stopping_distance_m(speed_mps: float, braking: object) -> float:
    if not math.isfinite(speed_mps) or speed_mps < 0.0:
        raise ValueError("speed must be finite and non-negative")
    if (
        not braking.HasField("requires_active_thrust")
        or not braking.requires_active_thrust
        or braking.minimum_translational_deceleration_mps2 <= 0.0
    ):
        raise ValueError("braking profile lacks active minimum deceleration")
    return (
        speed_mps * braking.command_latency_s
        + speed_mps**2
        / (2.0 * braking.minimum_translational_deceleration_mps2)
        + braking.stopping_distance_margin_m
    )


class ScoutCapabilityEnergyContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.generated = tempfile.TemporaryDirectory()
        proto_files = [str(path) for path in sorted(PROTO_V1.glob("*.proto"))]
        subprocess.run(
            [
                "protoc",
                f"--proto_path={PROTO_ROOT}",
                f"--python_out={cls.generated.name}",
                *proto_files,
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        sys.path.insert(0, cls.generated.name)
        cls.common = importlib.import_module("underwater.contracts.v1.common_pb2")
        cls.capability = importlib.import_module("underwater.contracts.v1.capability_pb2")
        cls.profile_artifact_bytes = PROFILE_PATH.read_bytes()
        cls.profile = json.loads(cls.profile_artifact_bytes.decode("utf-8"))
        cls.manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        cls.accepted_manifest_identity = b"m" * 32

    @classmethod
    def tearDownClass(cls) -> None:
        sys.path.remove(cls.generated.name)
        cls.generated.cleanup()

    def _header(self, stream_id: int, sequence: int) -> object:
        return self.common.MessageHeader(
            schema_major=1,
            schema_minor=0,
            producer_id="scout-vehicle-authority",
            producer_session_id=b"0123456789abcdef",
            stream_id=stream_id,
            sequence=sequence,
            source_clock_domain_id="scout-nuc/boot-12",
            generated_at_monotonic_ns=1_020_000_000,
            observed_at=self.common.SynchronizedObservationTime(
                utc_time_ns=1_789_000_000_000_000_000,
                status=self.common.TIME_SYNC_SYNCHRONIZED,
                uncertainty_ns=2_000_000,
            ),
            manifest=self.common.ContractManifestRef(
                schema_major=1,
                schema_minor=0,
                manifest_identity=self.common.ContentIdentity(
                    sha256=self.accepted_manifest_identity
                ),
            ),
        )

    def _profile_ref(self, profile_id: str, version: int, identity: bytes) -> object:
        return self.common.ProfileRef(
            profile_id=profile_id,
            version=version,
            content_identity=self.common.ContentIdentity(sha256=identity),
        )

    def _capability_profile(self, *, degraded: bool = False) -> object:
        health = (
            self.capability.SCOUT_THRUSTER_HEALTH_DEGRADED_A
            if degraded
            else self.capability.SCOUT_THRUSTER_HEALTH_NOMINAL
        )
        speed = 1.1 if degraded else 1.8
        profile = self.capability.ScoutCapabilityProfile(
            header=self._header(self.common.STREAM_SCOUT_CAPABILITY_PROFILE, 101),
            capability_profile_id=(
                "scout-capability/degraded-a" if degraded else "scout-capability/nominal"
            ),
            capability_profile_version=4 if degraded else 7,
            health_profile=health,
            motion_envelope=self.capability.MotionCapabilityEnvelope(
                maximum_water_relative_speed_mps=speed,
                maximum_translational_acceleration_mps2=0.55 if degraded else 0.9,
                maximum_vertical_ascent_speed_mps=0.45 if degraded else 0.7,
                maximum_vertical_descent_speed_mps=0.4 if degraded else 0.65,
                maximum_vertical_acceleration_mps2=0.38 if degraded else 0.62,
                maximum_yaw_rate_radps=0.35 if degraded else 0.6,
                maximum_yaw_acceleration_radps2=0.22 if degraded else 0.4,
                maximum_absolute_roll_rad=0.35,
                maximum_absolute_pitch_rad=0.35,
                minimum_translational_thrust_margin_fraction=0.25,
                minimum_rotational_thrust_margin_fraction=0.2,
                thruster_allocator_version="allocator/v3",
            ),
            braking_envelope=self.capability.BrakingCapabilityEnvelope(
                command_latency_s=0.18,
                minimum_translational_deceleration_mps2=0.42 if degraded else 0.7,
                minimum_yaw_deceleration_radps2=0.25 if degraded else 0.45,
                stopping_distance_margin_m=0.6,
                requires_active_thrust=True,
            ),
            operating_envelope=self.capability.CapabilityOperatingEnvelope(
                minimum_depth_m=1.0,
                maximum_depth_m=40.0,
                maximum_current_speed_mps=0.8,
                minimum_bus_voltage_v=18.0,
                maximum_bus_voltage_v=25.2,
                minimum_water_density_kgpm3=995.0,
                maximum_water_density_kgpm3=1030.0,
            ),
            operating_domain_id=self.profile["operating_domain_id"],
            vehicle_id="scout-01",
            device_serial_number="SCOUT-TEST-001",
            calibration_dataset_id="cal/scout-capability/2026-08-bench",
            calibration_method_version="capability-fit/v2",
            calibrated_at_utc_ns=1_787_000_000_000_000_000,
            production_approved=False,
            calibrated_thruster_states=[
                self.capability.ScoutThrusterUnitState(
                    thruster_id=f"thruster-{index}",
                    health=(
                        self.capability.SCOUT_THRUSTER_UNIT_HEALTH_DEGRADED
                        if degraded and index == 1
                        else self.capability.SCOUT_THRUSTER_UNIT_HEALTH_NOMINAL
                    ),
                )
                for index in range(1, 9)
            ],
            thruster_configuration_id="scout-01/thrusters/8x-v2",
        )
        profile.capability_content_identity.sha256 = canonical_business_identity(
            profile, "capability_content_identity"
        )
        return profile

    def _thruster_health(self, capability_profile: object | None = None) -> object:
        capability_profile = capability_profile or self._capability_profile()
        health = self.capability.ScoutThrusterHealthState(
            header=self._header(self.common.STREAM_SCOUT_THRUSTER_HEALTH, 102),
            vehicle_id="scout-01",
            health_version=12,
            observed_at_monotonic_ns=1_000_000_000,
            valid_until_monotonic_ns=1_200_000_000,
            health_profile=capability_profile.health_profile,
            thrusters=[
                self.capability.ScoutThrusterUnitState(
                    thruster_id=state.thruster_id,
                    health=state.health,
                )
                for state in capability_profile.calibrated_thruster_states
            ],
            active_capability_profile=self._profile_ref(
                capability_profile.capability_profile_id,
                capability_profile.capability_profile_version,
                capability_profile.capability_content_identity.sha256,
            ),
        )
        health.health_content_identity.sha256 = canonical_business_identity(
            health, "health_content_identity"
        )
        return health

    def _energy_model(self, capability_profile: object | None = None) -> object:
        capability_profile = capability_profile or self._capability_profile()
        model = self.capability.ScoutEnergyModelProfile(
            header=self._header(self.common.STREAM_SCOUT_ENERGY_MODEL, 103),
            energy_model_id="scout-energy/bench-v1",
            energy_model_version=5,
            health_profile=capability_profile.health_profile,
            capability_profile=self._profile_ref(
                capability_profile.capability_profile_id,
                capability_profile.capability_profile_version,
                capability_profile.capability_content_identity.sha256,
            ),
            power_model=self.capability.PowerModelCoefficients(
                hotel_power_w=85.0,
                speed_linear_w_per_mps=55.0,
                speed_cubic_w_per_mps3=120.0,
                acceleration_w_per_mps2=40.0,
                yaw_rate_w_per_radps=35.0,
                model_error_upper_bound_w=25.0,
            ),
            operating_domain_id=self.profile["operating_domain_id"],
            vehicle_id="scout-01",
            device_serial_number="SCOUT-TEST-001/BAT-01",
            calibration_dataset_id="cal/scout-energy/2026-08-bench",
            calibration_method_version="power-fit/v1",
            calibrated_at_utc_ns=1_787_000_000_000_000_000,
            production_approved=False,
        )
        model.energy_model_content_identity.sha256 = canonical_business_identity(
            model, "energy_model_content_identity"
        )
        return model

    def _energy_state(self, energy_model: object | None = None) -> object:
        energy_model = energy_model or self._energy_model()
        state = self.capability.ScoutEnergyState(
            header=self._header(self.common.STREAM_SCOUT_ENERGY_STATE, 104),
            vehicle_id="scout-01",
            energy_store_id="battery-main",
            energy_state_version=33,
            observed_at_monotonic_ns=1_000_000_000,
            valid_until_monotonic_ns=1_200_000_000,
            energy_model=self._profile_ref(
                energy_model.energy_model_id,
                energy_model.energy_model_version,
                energy_model.energy_model_content_identity.sha256,
            ),
            available_energy_j=1_200_000.0,
            reserve_energy_j=180_000.0,
            required_return_energy_j=260_000.0,
            required_risk_action_energy_j=90_000.0,
            contingency_requirement=self.capability.SCOUT_ENERGY_RETURN_REQUIRED,
            operating_domain_id=self.profile["operating_domain_id"],
        )
        state.energy_state_content_identity.sha256 = canonical_business_identity(
            state, "energy_state_content_identity"
        )
        return state

    def _validate_capability_profile(
        self, capability_profile: object, *, production: bool = False
    ) -> None:
        validate_capability_profile(
            capability_profile,
            self.capability,
            self.common,
            clock_domain_id="scout-nuc/boot-12",
            manifest=self.manifest,
            accepted_manifest_identity=self.accepted_manifest_identity,
            profile=self.profile,
            profile_artifact_bytes=self.profile_artifact_bytes,
            production_planning=production,
        )

    def _validate_energy_context(
        self,
        capability_profile: object,
        energy_model: object,
        energy_state: object,
        *,
        now: int = 1_030_000_000,
        production: bool = False,
    ) -> None:
        validate_energy_context(
            capability_profile,
            energy_model,
            energy_state,
            self.capability,
            self.common,
            clock_domain_id="scout-nuc/boot-12",
            now_monotonic_ns=now,
            manifest=self.manifest,
            accepted_manifest_identity=self.accepted_manifest_identity,
            profile=self.profile,
            profile_artifact_bytes=self.profile_artifact_bytes,
            production_planning=production,
        )

    def test_schema_exposes_independent_capability_health_model_and_energy_streams(self) -> None:
        common = (PROTO_V1 / "common.proto").read_text(encoding="utf-8")
        capability = (PROTO_V1 / "capability.proto").read_text(encoding="utf-8")

        for stream in (
            "STREAM_SCOUT_CAPABILITY_PROFILE",
            "STREAM_SCOUT_THRUSTER_HEALTH",
            "STREAM_SCOUT_ENERGY_MODEL",
            "STREAM_SCOUT_ENERGY_STATE",
        ):
            self.assertIn(stream, common)
        for message in (
            "message ScoutCapabilityProfile",
            "message ScoutThrusterHealthState",
            "message ScoutEnergyModelProfile",
            "message ScoutEnergyState",
        ):
            self.assertIn(message, capability)

    def test_schema_carries_calibrated_envelopes_braking_energy_and_provenance(self) -> None:
        capability = (PROTO_V1 / "capability.proto").read_text(encoding="utf-8")

        for field in (
            "ScoutThrusterHealthProfile health_profile",
            "MotionCapabilityEnvelope motion_envelope",
            "optional double maximum_vertical_acceleration_mps2",
            "BrakingCapabilityEnvelope braking_envelope",
            "CapabilityOperatingEnvelope operating_envelope",
            "ProfileRef active_capability_profile",
            "PowerModelCoefficients power_model",
            "optional double available_energy_j",
            "optional double reserve_energy_j",
            "optional double required_return_energy_j",
            "optional double required_risk_action_energy_j",
            "string device_serial_number",
            "string calibration_dataset_id",
            "string operating_domain_id",
            "optional int64 calibrated_at_utc_ns",
            "optional bool production_approved",
            "ContentIdentity capability_content_identity",
            "repeated ScoutThrusterUnitState calibrated_thruster_states",
            "string thruster_configuration_id",
            "ContentIdentity energy_model_content_identity",
            "ContentIdentity energy_state_content_identity",
        ):
            self.assertIn(field, capability)

    def test_complete_inputs_round_trip_with_bounded_non_production_profile(self) -> None:
        capability_profile = self._capability_profile()
        health = self._thruster_health(capability_profile)
        energy_model = self._energy_model(capability_profile)
        energy_state = self._energy_state(energy_model)

        for message in (capability_profile, health, energy_model, energy_state):
            self.assertEqual(type(message).FromString(message.SerializeToString()), message)
        for key in (
            "maximum_capability_profile_bytes",
            "maximum_thruster_health_bytes",
            "maximum_energy_model_bytes",
            "maximum_energy_state_bytes",
            "maximum_thrusters_per_vehicle",
        ):
            self.assertGreater(self.profile["interface_limits"][key], 0)
        for prefix in ("scout_thruster_health", "scout_energy_state"):
            self.assertLess(
                self.profile["timing"][f"{prefix}_publish_period_ns"],
                self.profile["timing"][f"{prefix}_stale_warning_ns"],
            )
            self.assertLess(
                self.profile["timing"][f"{prefix}_stale_warning_ns"],
                self.profile["timing"][f"{prefix}_reject_ns"],
            )
        self.assertFalse(self.profile["production"])

    def test_nominal_and_degraded_profiles_validate_without_operating_extrapolation(self) -> None:
        nominal = self._capability_profile()
        degraded = self._capability_profile(degraded=True)
        for capability_profile in (nominal, degraded):
            validate_capability_profile(
                capability_profile,
                self.capability,
                self.common,
                clock_domain_id="scout-nuc/boot-12",
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                profile=self.profile,
                profile_artifact_bytes=self.profile_artifact_bytes,
                production_planning=False,
            )

        self.assertLess(
            degraded.motion_envelope.maximum_water_relative_speed_mps,
            nominal.motion_envelope.maximum_water_relative_speed_mps,
        )
        self.assertTrue(
            capability_contains_conditions(
                nominal,
                depth_m=40.0,
                current_speed_mps=0.8,
                bus_voltage_v=18.0,
                water_density_kgpm3=1030.0,
            )
        )
        self.assertFalse(
            capability_contains_conditions(
                nominal,
                depth_m=40.000001,
                current_speed_mps=0.8,
                bus_voltage_v=18.0,
                water_density_kgpm3=1030.0,
            )
        )

    def test_health_degradation_requires_exact_degraded_profile_and_revalidation(self) -> None:
        nominal = self._capability_profile()
        nominal_health = self._thruster_health(nominal)
        original_dependencies, requires_revalidation = validate_capability_context(
            nominal,
            nominal_health,
            self.capability,
            self.common,
            clock_domain_id="scout-nuc/boot-12",
            now_monotonic_ns=1_030_000_000,
            manifest=self.manifest,
            accepted_manifest_identity=self.accepted_manifest_identity,
            profile=self.profile,
            profile_artifact_bytes=self.profile_artifact_bytes,
        )
        self.assertFalse(requires_revalidation)

        degraded = self._capability_profile(degraded=True)
        degraded_health = self._thruster_health(degraded)
        degraded_health.health_version += 1
        degraded_health.thrusters[0].health = (
            self.capability.SCOUT_THRUSTER_UNIT_HEALTH_DEGRADED
        )
        degraded_health.health_content_identity.sha256 = canonical_business_identity(
            degraded_health, "health_content_identity"
        )
        degraded_dependencies, requires_revalidation = validate_capability_context(
            degraded,
            degraded_health,
            self.capability,
            self.common,
            clock_domain_id="scout-nuc/boot-12",
            now_monotonic_ns=1_030_000_000,
            manifest=self.manifest,
            accepted_manifest_identity=self.accepted_manifest_identity,
            profile=self.profile,
            profile_artifact_bytes=self.profile_artifact_bytes,
            previous_dependencies=original_dependencies,
        )
        self.assertTrue(requires_revalidation)
        self.assertNotEqual(degraded_dependencies, original_dependencies)

        mismatched_health = self._thruster_health(nominal)
        mismatched_health.health_profile = self.capability.SCOUT_THRUSTER_HEALTH_DEGRADED_A
        mismatched_health.thrusters[0].health = (
            self.capability.SCOUT_THRUSTER_UNIT_HEALTH_DEGRADED
        )
        mismatched_health.health_content_identity.sha256 = canonical_business_identity(
            mismatched_health, "health_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "exact active capability profile"):
            validate_capability_context(
                nominal,
                mismatched_health,
                self.capability,
                self.common,
                clock_domain_id="scout-nuc/boot-12",
                now_monotonic_ns=1_030_000_000,
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                profile=self.profile,
                profile_artifact_bytes=self.profile_artifact_bytes,
            )

        omitted_thruster = self._thruster_health(nominal)
        omitted_thruster.thrusters.pop()
        omitted_thruster.health_content_identity.sha256 = canonical_business_identity(
            omitted_thruster, "health_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "exact calibrated thruster set"):
            validate_capability_context(
                nominal,
                omitted_thruster,
                self.capability,
                self.common,
                clock_domain_id="scout-nuc/boot-12",
                now_monotonic_ns=1_030_000_000,
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                profile=self.profile,
                profile_artifact_bytes=self.profile_artifact_bytes,
            )

        wrong_degraded_combination = self._thruster_health(degraded)
        wrong_degraded_combination.thrusters[0].health = (
            self.capability.SCOUT_THRUSTER_UNIT_HEALTH_NOMINAL
        )
        wrong_degraded_combination.thrusters[1].health = (
            self.capability.SCOUT_THRUSTER_UNIT_HEALTH_DEGRADED
        )
        wrong_degraded_combination.health_content_identity.sha256 = (
            canonical_business_identity(
                wrong_degraded_combination, "health_content_identity"
            )
        )
        with self.assertRaisesRegex(ValueError, "exact calibrated health combination"):
            validate_capability_context(
                degraded,
                wrong_degraded_combination,
                self.capability,
                self.common,
                clock_domain_id="scout-nuc/boot-12",
                now_monotonic_ns=1_030_000_000,
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                profile=self.profile,
                profile_artifact_bytes=self.profile_artifact_bytes,
            )

    def test_energy_only_sufficient_for_arrival_fails_without_return_or_risk_reserve(self) -> None:
        capability_profile = self._capability_profile()
        energy_model = self._energy_model(capability_profile)
        energy_state = self._energy_state(energy_model)
        validate_energy_context(
            capability_profile,
            energy_model,
            energy_state,
            self.capability,
            self.common,
            clock_domain_id="scout-nuc/boot-12",
            now_monotonic_ns=1_030_000_000,
            manifest=self.manifest,
            accepted_manifest_identity=self.accepted_manifest_identity,
            profile=self.profile,
            profile_artifact_bytes=self.profile_artifact_bytes,
            production_planning=False,
        )

        plan_energy_j = energy_state.available_energy_j - 1.0
        accepted, required_energy_j = energy_budget_accepts(
            plan_energy_j, energy_state, self.capability
        )
        self.assertFalse(accepted)
        self.assertEqual(
            required_energy_j,
            plan_energy_j
            + energy_state.required_return_energy_j
            + energy_state.reserve_energy_j,
        )

        exactly_sufficient_plan_j = (
            energy_state.available_energy_j
            - energy_state.required_return_energy_j
            - energy_state.reserve_energy_j
        )
        self.assertTrue(
            energy_budget_accepts(
                exactly_sufficient_plan_j, energy_state, self.capability
            )[0]
        )

        risk_state = self._energy_state(energy_model)
        risk_state.contingency_requirement = (
            self.capability.SCOUT_ENERGY_RISK_ACTION_REQUIRED
        )
        risk_state.energy_state_content_identity.sha256 = canonical_business_identity(
            risk_state, "energy_state_content_identity"
        )
        risk_plan_j = (
            risk_state.available_energy_j
            - risk_state.required_risk_action_energy_j
            - risk_state.reserve_energy_j
        )
        self.assertTrue(energy_budget_accepts(risk_plan_j, risk_state, self.capability)[0])

    def test_stream_watermarks_reject_reorder_conflict_rollback_and_retired_session(self) -> None:
        for message, version_field, identity_field in (
            (
                self._capability_profile(),
                "capability_profile_version",
                "capability_content_identity",
            ),
            (self._thruster_health(), "health_version", "health_content_identity"),
            (
                self._energy_model(),
                "energy_model_version",
                "energy_model_content_identity",
            ),
            (
                self._energy_state(),
                "energy_state_version",
                "energy_state_content_identity",
            ),
        ):
            with self.subTest(message=type(message).__name__):
                version = getattr(message, version_field)
                identity = getattr(message, identity_field).sha256
                delivery = exact_delivery_identity(message)
                arguments = {
                    "current_session": message.header.producer_session_id,
                    "current_sequence": message.header.sequence,
                    "current_version": version,
                    "current_identity": identity,
                    "current_delivery_identity": delivery,
                    "incoming_session": message.header.producer_session_id,
                    "incoming_sequence": message.header.sequence,
                    "incoming_version": version,
                    "incoming_identity": identity,
                    "incoming_delivery_identity": delivery,
                    "retired_sessions": set(),
                }
                self.assertEqual(
                    apply_versioned_stream_watermark(**arguments),
                    "idempotent duplicate",
                )
                with self.assertRaisesRegex(ValueError, "SEQUENCE_REJECTED"):
                    apply_versioned_stream_watermark(
                        **dict(arguments, incoming_sequence=message.header.sequence - 1)
                    )
                with self.assertRaisesRegex(ValueError, "INPUT_INVALID"):
                    apply_versioned_stream_watermark(
                        **dict(arguments, incoming_delivery_identity=b"x" * 32)
                    )
                with self.assertRaisesRegex(ValueError, "VERSION_INCOMPATIBLE"):
                    apply_versioned_stream_watermark(
                        **dict(
                            arguments,
                            incoming_sequence=message.header.sequence + 1,
                            incoming_version=version - 1,
                            incoming_identity=b"y" * 32,
                            incoming_delivery_identity=b"z" * 32,
                        )
                    )
                with self.assertRaisesRegex(ValueError, "SEQUENCE_REJECTED"):
                    apply_versioned_stream_watermark(
                        **dict(
                            arguments,
                            incoming_session=b"fedcba9876543210",
                            incoming_sequence=1,
                            incoming_version=version + 1,
                            incoming_identity=b"a" * 32,
                            incoming_delivery_identity=b"b" * 32,
                            retired_sessions={b"fedcba9876543210"},
                        )
                    )

                retired_sessions: set[bytes] = set()
                self.assertEqual(
                    apply_versioned_stream_watermark(
                        **dict(
                            arguments,
                            incoming_session=b"fedcba9876543210",
                            incoming_sequence=1,
                            incoming_version=version + 1,
                            incoming_identity=b"c" * 32,
                            incoming_delivery_identity=b"d" * 32,
                            retired_sessions=retired_sessions,
                        )
                    ),
                    "new session",
                )
                self.assertIn(message.header.producer_session_id, retired_sessions)
                with self.assertRaisesRegex(ValueError, "SEQUENCE_REJECTED"):
                    apply_versioned_stream_watermark(
                        current_session=b"fedcba9876543210",
                        current_sequence=1,
                        current_version=version + 1,
                        current_identity=b"c" * 32,
                        current_delivery_identity=b"d" * 32,
                        incoming_session=message.header.producer_session_id,
                        incoming_sequence=message.header.sequence,
                        incoming_version=version,
                        incoming_identity=identity,
                        incoming_delivery_identity=delivery,
                        retired_sessions=retired_sessions,
                    )

    def test_normative_docs_define_hard_gates_hashing_and_adapter_boundaries(self) -> None:
        contract = CONTRACT_PATH.read_text(encoding="utf-8")
        for rule in (
            "capability_profile_version",
            "energy_model_version",
            "Health degradation invalidates every candidate and authorization",
            "MUST NOT extrapolate",
            "Zero thrust is not braking",
            "E_available >= E_plan + E_return + E_reserve",
            "E_available >= E_plan + E_risk_action + E_reserve",
            "C++ <-> Protobuf",
            "ROS 2 <-> Protobuf",
            "Adapters MUST NOT synthesize missing values",
            "NON_PRODUCTION",
        ):
            self.assertIn(rule, contract)

        hashing = HASHING_PATH.read_text(encoding="utf-8")
        for identity_rule in (
            "`ScoutCapabilityProfile`: clear `header` and `capability_content_identity`",
            "`ScoutThrusterHealthState`: clear `header` and `health_content_identity`",
            "`ScoutEnergyModelProfile`: clear `header` and `energy_model_content_identity`",
            "`ScoutEnergyState`: clear `header` and `energy_state_content_identity`",
        ):
            self.assertIn(identity_rule, hashing)

    def test_nonfinite_zero_thrust_stale_unknown_and_unregistered_values_fail_closed(self) -> None:
        nonfinite_capability = self._capability_profile()
        nonfinite_capability.motion_envelope.maximum_water_relative_speed_mps = math.nan
        nonfinite_capability.capability_content_identity.sha256 = canonical_business_identity(
            nonfinite_capability, "capability_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "motion capability"):
            self._validate_capability_profile(nonfinite_capability)

        zero_thrust_stop = self._capability_profile()
        zero_thrust_stop.braking_envelope.requires_active_thrust = False
        zero_thrust_stop.capability_content_identity.sha256 = canonical_business_identity(
            zero_thrust_stop, "capability_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "zero thrust"):
            self._validate_capability_profile(zero_thrust_stop)

        capability_profile = self._capability_profile()
        energy_model = self._energy_model(capability_profile)
        nonfinite_model = self._energy_model(capability_profile)
        nonfinite_model.power_model.speed_cubic_w_per_mps3 = math.inf
        nonfinite_model.energy_model_content_identity.sha256 = canonical_business_identity(
            nonfinite_model, "energy_model_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "non-finite"):
            self._validate_energy_context(
                capability_profile, nonfinite_model, self._energy_state(nonfinite_model)
            )

        stale_state = self._energy_state(energy_model)
        with self.assertRaisesRegex(ValueError, "stale"):
            self._validate_energy_context(
                capability_profile, energy_model, stale_state, now=1_500_000_001
            )

        unknown_contingency = self._energy_state(energy_model)
        unknown_contingency.contingency_requirement = (
            self.capability.SCOUT_ENERGY_CONTINGENCY_UNSPECIFIED
        )
        unknown_contingency.energy_state_content_identity.sha256 = canonical_business_identity(
            unknown_contingency, "energy_state_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "energy state"):
            self._validate_energy_context(
                capability_profile, energy_model, unknown_contingency
            )

        unknown_numeric_enum = self._energy_state(energy_model)
        unknown_numeric_enum.contingency_requirement = 99
        unknown_numeric_enum.energy_state_content_identity.sha256 = canonical_business_identity(
            unknown_numeric_enum, "energy_state_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "energy state"):
            self._validate_energy_context(
                capability_profile, energy_model, unknown_numeric_enum
            )

        non_nfc = self._capability_profile()
        non_nfc.capability_profile_id = "capacite\u0301/decomposed"
        non_nfc.capability_content_identity.sha256 = canonical_business_identity(
            non_nfc, "capability_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "NFC"):
            self._validate_capability_profile(non_nfc)

        degraded = self._capability_profile(degraded=True)
        unregistered_fault = self._thruster_health(degraded)
        unregistered_fault.thrusters[0].health = (
            self.capability.SCOUT_THRUSTER_UNIT_HEALTH_DEGRADED
        )
        unregistered_fault.active_fault_codes.append(4_294_967_295)
        unregistered_fault.health_content_identity.sha256 = canonical_business_identity(
            unregistered_fault, "health_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "active fault codes"):
            validate_thruster_health(
                unregistered_fault,
                degraded,
                self.capability,
                self.common,
                clock_domain_id="scout-nuc/boot-12",
                now_monotonic_ns=1_030_000_000,
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                profile=self.profile,
                profile_artifact_bytes=self.profile_artifact_bytes,
            )

    def test_braking_distance_uses_latency_minimum_deceleration_and_margin(self) -> None:
        braking = self._capability_profile().braking_envelope
        self.assertAlmostEqual(
            conservative_stopping_distance_m(1.4, braking),
            1.4 * 0.18 + (1.4**2) / (2.0 * 0.7) + 0.6,
        )
        self.assertAlmostEqual(
            conservative_stopping_distance_m(0.0, braking),
            braking.stopping_distance_margin_m,
        )
        with self.assertRaisesRegex(ValueError, "speed"):
            conservative_stopping_distance_m(math.nan, braking)

    def test_identity_manifest_resource_and_non_production_gates_fail_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "NON_PRODUCTION"):
            self._validate_capability_profile(
                self._capability_profile(), production=True
            )
        capability_profile = self._capability_profile()
        energy_model = self._energy_model(capability_profile)
        with self.assertRaisesRegex(ValueError, "NON_PRODUCTION"):
            self._validate_energy_context(
                capability_profile,
                energy_model,
                self._energy_state(energy_model),
                production=True,
            )

        tampered = self._capability_profile()
        tampered.motion_envelope.maximum_water_relative_speed_mps += 0.1
        with self.assertRaisesRegex(ValueError, "content identity"):
            self._validate_capability_profile(tampered)

        unknown = self.capability.ScoutEnergyState.FromString(
            self._energy_state().SerializeToString(deterministic=True)
            + b"\xa0\x06\x01"
        )
        with self.assertRaisesRegex(ValueError, "unknown field"):
            validate_energy_state(
                unknown,
                self._energy_model(),
                self.capability,
                self.common,
                clock_domain_id="scout-nuc/boot-12",
                now_monotonic_ns=1_030_000_000,
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                profile=self.profile,
                profile_artifact_bytes=self.profile_artifact_bytes,
            )

        oversized = self._capability_profile()
        oversized.capability_profile_id = "x" * (
            self.profile["interface_limits"]["maximum_string_bytes"] + 1
        )
        oversized.capability_content_identity.sha256 = canonical_business_identity(
            oversized, "capability_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "RESOURCE_LIMIT_EXCEEDED"):
            self._validate_capability_profile(oversized)

        forged_profile = json.loads(json.dumps(self.profile))
        forged_profile["production"] = True
        with self.assertRaisesRegex(ValueError, "Manifest-bound"):
            validate_capability_profile(
                self._capability_profile(),
                self.capability,
                self.common,
                clock_domain_id="scout-nuc/boot-12",
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                profile=forged_profile,
                profile_artifact_bytes=json.dumps(forged_profile).encode("utf-8"),
                production_planning=True,
            )

        self.assertIn(
            "scout_capability_and_energy_v1", self.manifest["supported_features"]
        )


if __name__ == "__main__":
    unittest.main()
